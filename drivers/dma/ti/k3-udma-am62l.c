// SPDX-License-Identifier: GPL-2.0
/*
 *  Derived from K3 UDMA driver (k3-udma.c)
 *  Copyright (C) 2024-2025 Texas Instruments Incorporated - http://www.ti.com
 *  Author: Peter Ujfalusi <peter.ujfalusi@ti.com>
 *  Author: Sai Sree Kartheek Adivi <s-adivi@ti.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/sys_soc.h>
#include <linux/of.h>
#include <linux/of_dma.h>
#include <linux/of_irq.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/iopoll.h>
#include <linux/soc/ti/k3-ringacc.h>
#include <linux/dma/k3-event-router.h>
#include <linux/dma/ti-cppi5.h>

#include "../virt-dma.h"
#include "k3-udma.h"
#include "k3-psil-priv.h"

#define UDMA_CHAN_RT_STATIC_TR_XY_REG	0x800
#define UDMA_CHAN_RT_STATIC_TR_Z_REG	0x804
#define UDMA_CHAN_RT_PERIPH_BCNT_REG	0x810

static int am62l_udma_check_chan_autopair_completion(struct udma_chan *uc)
{
	u32 val;

	val = udma_chanrt_read(uc, UDMA_CHAN_RT_CTL_REG);
	if (val & UDMA_CHAN_RT_CTL_PAIR_TIMEOUT)
		return -ETIMEDOUT;
	else if (val & UDMA_CHAN_RT_CTL_PAIR_COMPLETE)
		return 1;

	/* timeout didn't occur and also pairing didn't happen yet. */
	return 0;
}

static bool am62l_udma_is_chan_paused(struct udma_chan *uc)
{
	u32 val, pause_mask;

	if (uc->config.dir == DMA_MEM_TO_MEM) {
		val = udma_chanrt_read(uc, UDMA_CHAN_RT_CTL_REG);
		pause_mask = UDMA_CHAN_RT_CTL_PAUSE;
	} else {
		val = udma_chanrt_read(uc, UDMA_CHAN_RT_PDMA_STATE_REG);
		pause_mask = UDMA_CHAN_RT_PDMA_STATE_PAUSE;
	}

	if (val & pause_mask)
		return true;

	return false;
}

static void am62l_udma_reset_rings(struct udma_chan *uc)
{
	struct k3_ring *ring1 = NULL;
	struct k3_ring *ring2 = NULL;

	switch (uc->config.dir) {
	case DMA_DEV_TO_MEM:
		if (uc->rchan) {
			ring1 = uc->rflow->fd_ring;
			ring2 = uc->rflow->r_ring;
		}
		break;
	case DMA_MEM_TO_DEV:
	case DMA_MEM_TO_MEM:
		if (uc->tchan) {
			ring1 = uc->tchan->t_ring;
			ring2 = uc->tchan->tc_ring;
		}
		break;
	default:
		break;
	}

	/* make sure we are not leaking memory by stalled descriptor */
	if (uc->terminated_desc) {
		udma_desc_free(&uc->terminated_desc->vd);
		uc->terminated_desc = NULL;
	}
}

static void am62l_udma_decrement_byte_counters(struct udma_chan *uc, u32 val)
{
	udma_chanrt_write(uc, UDMA_CHAN_RT_BCNT_REG, val);
	udma_chanrt_write(uc, UDMA_CHAN_RT_SBCNT_REG, val);
	if (uc->config.ep_type != PSIL_EP_NATIVE)
		udma_chanrt_write(uc, UDMA_CHAN_RT_PERIPH_BCNT_REG, val);
}

static void am62l_udma_reset_counters(struct udma_chan *uc)
{
	u32 val;

	val = udma_chanrt_read(uc, UDMA_CHAN_RT_BCNT_REG);
	udma_chanrt_write(uc, UDMA_CHAN_RT_BCNT_REG, val);

	val = udma_chanrt_read(uc, UDMA_CHAN_RT_SBCNT_REG);
	udma_chanrt_write(uc, UDMA_CHAN_RT_SBCNT_REG, val);

	val = udma_chanrt_read(uc, UDMA_CHAN_RT_PCNT_REG);
	udma_chanrt_write(uc, UDMA_CHAN_RT_PCNT_REG, val);

	if (!uc->bchan) {
		val = udma_chanrt_read(uc, UDMA_CHAN_RT_PERIPH_BCNT_REG);
		udma_chanrt_write(uc, UDMA_CHAN_RT_PERIPH_BCNT_REG, val);
	}
}

static int am62l_udma_reset_chan(struct udma_chan *uc, bool hard)
{
	udma_chanrt_write(uc, UDMA_CHAN_RT_CTL_REG, 0);

	/* Reset all counters */
	am62l_udma_reset_counters(uc);

	/* Hard reset: re-initialize the channel to reset */
	if (hard) {
		struct udma_chan_config ucc_backup;
		int ret;

		memcpy(&ucc_backup, &uc->config, sizeof(uc->config));
		uc->ud->ddev.device_free_chan_resources(&uc->vc.chan);

		/* restore the channel configuration */
		memcpy(&uc->config, &ucc_backup, sizeof(uc->config));
		ret = uc->ud->ddev.device_alloc_chan_resources(&uc->vc.chan);
		if (ret)
			return ret;

		/*
		 * Setting forced teardown after forced reset helps recovering
		 * the rchan.
		 */
		if (uc->config.dir == DMA_DEV_TO_MEM)
			udma_chanrt_update_bits(uc, UDMA_CHAN_RT_CTL_REG,
					UDMA_CHAN_RT_CTL_EN | UDMA_CHAN_RT_CTL_TDOWN |
					UDMA_CHAN_RT_CTL_FTDOWN,
					UDMA_CHAN_RT_CTL_EN | UDMA_CHAN_RT_CTL_TDOWN |
					UDMA_CHAN_RT_CTL_FTDOWN);
	}
	uc->state = UDMA_CHAN_IS_IDLE;

	return 0;
}

static int am62l_udma_start(struct udma_chan *uc)
{
	struct virt_dma_desc *vd = vchan_next_desc(&uc->vc);
	struct udma_dev *ud = uc->ud;
	int status, ret;

	if (!vd) {
		uc->desc = NULL;
		return -ENOENT;
	}

	list_del(&vd->node);

	uc->desc = to_udma_desc(&vd->tx);

	/* Channel is already running and does not need reconfiguration */
	if (udma_is_chan_running(uc) && !udma_chan_needs_reconfiguration(uc)) {
		udma_start_desc(uc);
		goto out;
	}

	/* Make sure that we clear the teardown bit, if it is set */
	ud->udma_reset_chan(uc, false);

	/* Push descriptors before we start the channel */
	udma_start_desc(uc);

	switch (uc->desc->dir) {
	case DMA_DEV_TO_MEM:
		/* Config remote TR */
		if (uc->config.ep_type == PSIL_EP_PDMA_XY) {
			u32 val = PDMA_STATIC_TR_Y(uc->desc->static_tr.elcnt) |
				  PDMA_STATIC_TR_X(uc->desc->static_tr.elsize);
			const struct udma_match_data *match_data =
							uc->ud->match_data;

			if (uc->config.enable_acc32)
				val |= PDMA_STATIC_TR_XY_ACC32;
			if (uc->config.enable_burst)
				val |= PDMA_STATIC_TR_XY_BURST;

			udma_chanrt_write(uc,
					   UDMA_CHAN_RT_STATIC_TR_XY_REG,
					   val);

			udma_chanrt_write(uc,
				UDMA_CHAN_RT_STATIC_TR_Z_REG,
				PDMA_STATIC_TR_Z(uc->desc->static_tr.bstcnt,
						 match_data->statictr_z_mask));

			/* save the current staticTR configuration */
			memcpy(&uc->static_tr, &uc->desc->static_tr,
			       sizeof(uc->static_tr));
		}

		udma_chanrt_write(uc, UDMA_CHAN_RT_CTL_REG,
				UDMA_CHAN_RT_CTL_EN | UDMA_CHAN_RT_CTL_AUTOPAIR);

		/* Poll for autopair completion */
		ret = read_poll_timeout_atomic(am62l_udma_check_chan_autopair_completion,
				status, status != 0, 100, 500, false, uc);

		if (status <= 0)
			return -ETIMEDOUT;

		break;
	case DMA_MEM_TO_DEV:
		/* Config remote TR */
		if (uc->config.ep_type == PSIL_EP_PDMA_XY) {
			u32 val = PDMA_STATIC_TR_Y(uc->desc->static_tr.elcnt) |
				  PDMA_STATIC_TR_X(uc->desc->static_tr.elsize);

			if (uc->config.enable_acc32)
				val |= PDMA_STATIC_TR_XY_ACC32;
			if (uc->config.enable_burst)
				val |= PDMA_STATIC_TR_XY_BURST;

			udma_chanrt_write(uc,
					   UDMA_CHAN_RT_STATIC_TR_XY_REG,
					   val);

			/* save the current staticTR configuration */
			memcpy(&uc->static_tr, &uc->desc->static_tr,
			       sizeof(uc->static_tr));
		}

		udma_chanrt_write(uc, UDMA_CHAN_RT_CTL_REG,
				UDMA_CHAN_RT_CTL_EN | UDMA_CHAN_RT_CTL_AUTOPAIR);

		/* Poll for autopair completion */
		ret = read_poll_timeout_atomic(am62l_udma_check_chan_autopair_completion,
				status, status != 0, 100, 500, false, uc);

		if (status <= 0)
			return -ETIMEDOUT;

		break;
	case DMA_MEM_TO_MEM:
		udma_bchanrt_write(uc, UDMA_CHAN_RT_CTL_REG,
				   UDMA_CHAN_RT_CTL_EN);
		udma_bchanrt_write(uc, UDMA_CHAN_RT_CTL_REG,
				   UDMA_CHAN_RT_CTL_EN);

		break;
	default:
		return -EINVAL;
	}

	uc->state = UDMA_CHAN_IS_ACTIVE;
out:

	return 0;
}

static int am62l_udma_stop(struct udma_chan *uc)
{
	if (uc->ud->match_data->type == DMA_TYPE_BCDMA_V2)
		return 0;
	uc->state = UDMA_CHAN_IS_TERMINATING;
	reinit_completion(&uc->teardown_completed);

	if (uc->config.dir == DMA_DEV_TO_MEM) {
		if (!uc->cyclic && !uc->desc)
			udma_push_to_ring(uc, -1);
	}

	udma_chanrt_update_bits(uc, UDMA_CHAN_RT_CTL_REG,
		UDMA_CHAN_RT_CTL_EN | UDMA_CHAN_RT_CTL_TDOWN,
		UDMA_CHAN_RT_CTL_EN | UDMA_CHAN_RT_CTL_TDOWN);

	return 0;
}

static bool am62l_udma_is_desc_really_done(struct udma_chan *uc, struct udma_desc *d)
{
	u32 peer_bcnt, bcnt;

	/*
	 * Only TX towards PDMA is affected.
	 * If DMA_PREP_INTERRUPT is not set by consumer then skip the transfer
	 * completion calculation, consumer must ensure that there is no stale
	 * data in DMA fabric in this case.
	 */
	if (uc->config.ep_type == PSIL_EP_NATIVE ||
	    uc->config.dir != DMA_MEM_TO_DEV || !(uc->config.tx_flags & DMA_PREP_INTERRUPT))
		return true;

	peer_bcnt = udma_chanrt_read(uc, UDMA_CHAN_RT_PERIPH_BCNT_REG);
	bcnt = udma_chanrt_read(uc, UDMA_CHAN_RT_BCNT_REG);

	/* Transfer is incomplete, store current residue and time stamp */
	if (peer_bcnt < bcnt) {
		uc->tx_drain.residue = bcnt - peer_bcnt;
		uc->tx_drain.tstamp = ktime_get();
		return false;
	}

	return true;
}

static irqreturn_t am62l_udma_udma_irq_handler(int irq, void *data)
{
	struct udma_chan *uc = data;
	struct udma_dev *ud = uc->ud;
	struct udma_desc *d;

	switch (uc->config.dir) {
	case DMA_DEV_TO_MEM:
		k3_ringacc_ring_clear_irq(uc->rflow->r_ring);
		break;
	case DMA_MEM_TO_DEV:
	case DMA_MEM_TO_MEM:
		k3_ringacc_ring_clear_irq(uc->tchan->tc_ring);
		break;
	default:
		return -ENOENT;
	}

	spin_lock(&uc->vc.lock);
	d = uc->desc;
	if (d) {
		d->tr_idx = (d->tr_idx + 1) % d->sglen;

		if (uc->cyclic) {
			vchan_cyclic_callback(&d->vd);
		} else {
			/* TODO: figure out the real amount of data */
			ud->udma_decrement_byte_counters(uc, d->residue);
			ud->udma_start(uc);
			vchan_cookie_complete(&d->vd);
		}
	}

	spin_unlock(&uc->vc.lock);

	return IRQ_HANDLED;
}

static irqreturn_t am62l_udma_ring_irq_handler(int irq, void *data)
{
	struct udma_chan *uc = data;
	struct udma_dev *ud = uc->ud;
	struct udma_desc *d;
	dma_addr_t paddr = 0;
	u32 intr_status;

	switch (uc->config.dir) {
	case DMA_DEV_TO_MEM:
		intr_status =  k3_ringacc_ring_get_irq_status(uc->rflow->r_ring);
		break;
	case DMA_MEM_TO_DEV:
	case DMA_MEM_TO_MEM:
		intr_status =  k3_ringacc_ring_get_irq_status(uc->tchan->tc_ring);
		break;
	default:
		return -ENOENT;
	}

	if (intr_status & K3_RINGACC_RT_INT_STATUS_TR)
		return am62l_udma_udma_irq_handler(irq, data);

	if (udma_pop_from_ring(uc, &paddr) || !paddr)
		return IRQ_HANDLED;

	spin_lock(&uc->vc.lock);

	/* Teardown completion message */
	if (cppi5_desc_is_tdcm(paddr)) {
		complete_all(&uc->teardown_completed);

		if (uc->terminated_desc) {
			udma_desc_free(&uc->terminated_desc->vd);
			uc->terminated_desc = NULL;
		}

		if (!uc->desc)
			ud->udma_start(uc);

		goto out;
	}

	d = udma_udma_desc_from_paddr(uc, paddr);

	if (d) {
		dma_addr_t desc_paddr = udma_curr_cppi5_desc_paddr(d,
								   d->desc_idx);
		if (desc_paddr != paddr) {
			dev_err(uc->ud->dev, "not matching descriptors!\n");
			goto out;
		}

		if (d == uc->desc) {
			/* active descriptor */
			if (uc->cyclic) {
				udma_cyclic_packet_elapsed(uc);
				vchan_cyclic_callback(&d->vd);
			} else {
				if (ud->udma_is_desc_really_done(uc, d)) {
					ud->udma_decrement_byte_counters(uc, d->residue);
					ud->udma_start(uc);
					vchan_cookie_complete(&d->vd);
				} else {
					schedule_delayed_work(&uc->tx_drain.work,
							      0);
				}
			}
		} else {
			/*
			 * terminated descriptor, mark the descriptor as
			 * completed to update the channel's cookie marker
			 */
			dma_cookie_complete(&d->vd.tx);
		}
	}
out:
	spin_unlock(&uc->vc.lock);

	return IRQ_HANDLED;
}

static int am62l_bcdma_v2_get_bchan(struct udma_chan *uc)
{
	struct udma_dev *ud = uc->ud;
	enum udma_tp_level tpl;
	int ret;

	if (uc->bchan) {
		dev_dbg(ud->dev, "chan%d: already have bchan%d allocated\n",
			uc->id, uc->bchan->id);
		return 0;
	}

	/*
	 * Use normal channels for peripherals, and highest TPL channel for
	 * mem2mem
	 */
	if (uc->config.tr_trigger_type)
		tpl = 0;
	else
		tpl = ud->bchan_tpl.levels - 1;

	uc->bchan = __udma_reserve_bchan(ud, tpl, uc->id);
	if (IS_ERR(uc->bchan)) {
		ret = PTR_ERR(uc->bchan);
		uc->bchan = NULL;
		return ret;
	}
	uc->chan = uc->bchan;
	uc->tchan = uc->bchan;

	return 0;
}

static int am62l_bcdma_v2_alloc_bchan_resources(struct udma_chan *uc)
{
	struct k3_ring_cfg ring_cfg;
	struct udma_dev *ud = uc->ud;
	int ret;

	ret = am62l_bcdma_v2_get_bchan(uc);
	if (ret)
		return ret;

	ret = k3_ringacc_request_rings_pair(ud->ringacc, ud->match_data->chan_cnt + uc->id, -1,
			&uc->bchan->t_ring,
			&uc->bchan->tc_ring);
	if (ret) {
		ret = -EBUSY;
		goto err_ring;
	}

	memset(&ring_cfg, 0, sizeof(ring_cfg));
	ring_cfg.size = K3_UDMA_DEFAULT_RING_SIZE;
	ring_cfg.elm_size = K3_RINGACC_RING_ELSIZE_8;
	ring_cfg.mode = K3_RINGACC_RING_MODE_RING;

	k3_configure_chan_coherency(&uc->vc.chan, ud->asel);
	ring_cfg.asel = ud->asel;
	ring_cfg.dma_dev = dmaengine_get_dma_device(&uc->vc.chan);

	ret = k3_ringacc_ring_cfg(uc->bchan->t_ring, &ring_cfg);
	if (ret)
		goto err_ringcfg;

	return 0;

err_ringcfg:
	k3_ringacc_ring_free(uc->bchan->tc_ring);
	uc->bchan->tc_ring = NULL;
	k3_ringacc_ring_free(uc->bchan->t_ring);
	uc->bchan->t_ring = NULL;
	k3_configure_chan_coherency(&uc->vc.chan, 0);
err_ring:
	bcdma_put_bchan(uc);

	return ret;
}

static int am62l_udma_alloc_tx_resources(struct udma_chan *uc)
{
	struct k3_ring_cfg ring_cfg;
	struct udma_dev *ud = uc->ud;
	struct udma_tchan *tchan;
	int ring_idx, ret;

	ret = udma_get_tchan(uc);
	if (ret)
		return ret;

	tchan = uc->tchan;
	if (tchan->tflow_id >= 0)
		ring_idx = tchan->tflow_id;
	else
		ring_idx = tchan->id;

	ret = k3_ringacc_request_rings_pair(ud->ringacc, ring_idx, -1,
					    &tchan->t_ring,
					    &tchan->tc_ring);
	if (ret) {
		ret = -EBUSY;
		goto err_ring;
	}

	memset(&ring_cfg, 0, sizeof(ring_cfg));
	ring_cfg.size = K3_UDMA_DEFAULT_RING_SIZE;
	ring_cfg.elm_size = K3_RINGACC_RING_ELSIZE_8;
	ring_cfg.mode = K3_RINGACC_RING_MODE_RING;

	k3_configure_chan_coherency(&uc->vc.chan, uc->config.asel);
	ring_cfg.asel = uc->config.asel;
	ring_cfg.dma_dev = dmaengine_get_dma_device(&uc->vc.chan);

	ret = k3_ringacc_ring_cfg(tchan->t_ring, &ring_cfg);
	ret |= k3_ringacc_ring_cfg(tchan->tc_ring, &ring_cfg);

	if (ret)
		goto err_ringcfg;

	return 0;

err_ringcfg:
	k3_ringacc_ring_free(uc->tchan->tc_ring);
	uc->tchan->tc_ring = NULL;
	k3_ringacc_ring_free(uc->tchan->t_ring);
	uc->tchan->t_ring = NULL;
err_ring:
	udma_put_tchan(uc);

	return ret;
}

static int am62l_udma_alloc_rx_resources(struct udma_chan *uc)
{
	struct udma_dev *ud = uc->ud;
	struct k3_ring_cfg ring_cfg;
	struct udma_rflow *rflow;
	int fd_ring_id;
	int ret;

	ret = udma_get_rchan(uc);
	if (ret)
		return ret;

	/* For MEM_TO_MEM we don't need rflow or rings */
	if (uc->config.dir == DMA_MEM_TO_MEM)
		return 0;

	if (uc->config.default_flow_id >= 0)
		ret = udma_get_rflow(uc, uc->config.default_flow_id);
	else
		ret = udma_get_rflow(uc, uc->rchan->id);

	if (ret) {
		ret = -EBUSY;
		goto err_rflow;
	}

	rflow = uc->rflow;
	if (ud->tflow_cnt)
		fd_ring_id = ud->tflow_cnt + rflow->id;
	else
		fd_ring_id = uc->rchan->id;
	ret = k3_ringacc_request_rings_pair(ud->ringacc, fd_ring_id, -1,
					    &rflow->fd_ring, &rflow->r_ring);
	if (ret) {
		ret = -EBUSY;
		goto err_ring;
	}

	memset(&ring_cfg, 0, sizeof(ring_cfg));

	ring_cfg.elm_size = K3_RINGACC_RING_ELSIZE_8;
	ring_cfg.size = K3_UDMA_DEFAULT_RING_SIZE;
	ring_cfg.mode = K3_RINGACC_RING_MODE_RING;

	k3_configure_chan_coherency(&uc->vc.chan, uc->config.asel);
	ring_cfg.asel = uc->config.asel;
	ring_cfg.dma_dev = dmaengine_get_dma_device(&uc->vc.chan);

	ret = k3_ringacc_ring_cfg(rflow->fd_ring, &ring_cfg);

	ring_cfg.size = K3_UDMA_DEFAULT_RING_SIZE;
	ret |= k3_ringacc_ring_cfg(rflow->r_ring, &ring_cfg);

	if (ret)
		goto err_ringcfg;

	return 0;

err_ringcfg:
	k3_ringacc_ring_free(rflow->r_ring);
	rflow->r_ring = NULL;
	k3_ringacc_ring_free(rflow->fd_ring);
	rflow->fd_ring = NULL;
err_ring:
	udma_put_rflow(uc);
err_rflow:
	udma_put_rchan(uc);

	return ret;
}

static int am62l_bcdma_v2_alloc_chan_resources(struct dma_chan *chan)
{
	struct udma_chan *uc = to_udma_chan(chan);
	struct udma_dev *ud = to_udma_dev(chan->device);
	u32 irq_ring_idx;
	__be32 addr[2] = {0, 0};
	struct of_phandle_args out_irq;
	int ret;

	/* Only TR mode is supported */
	uc->config.pkt_mode = false;

	/*
	 * Make sure that the completion is in a known state:
	 * No teardown, the channel is idle
	 */
	reinit_completion(&uc->teardown_completed);
	complete_all(&uc->teardown_completed);
	uc->state = UDMA_CHAN_IS_IDLE;

	switch (uc->config.dir) {
	case DMA_MEM_TO_MEM:
		/* Non synchronized - mem to mem type of transfer */
		dev_dbg(uc->ud->dev, "%s: chan%d as MEM-to-MEM\n", __func__,
			uc->id);

		ret = am62l_bcdma_v2_alloc_bchan_resources(uc);
		if (ret)
			return ret;

		irq_ring_idx = ud->match_data->chan_cnt + uc->id;
		break;
	case DMA_MEM_TO_DEV:
		/* Slave transfer synchronized - mem to dev (TX) trasnfer */
		dev_dbg(uc->ud->dev, "%s: chan%d as MEM-to-DEV\n", __func__,
			uc->id);

		ret = am62l_udma_alloc_tx_resources(uc);
		if (ret) {
			uc->config.remote_thread_id = -1;
			return ret;
		}

		uc->config.src_thread = ud->psil_base + uc->tchan->id;
		uc->config.dst_thread = uc->config.remote_thread_id;
		uc->config.dst_thread |= K3_PSIL_DST_THREAD_ID_OFFSET;

		irq_ring_idx = uc->tchan->id;

		break;
	case DMA_DEV_TO_MEM:
		/* Slave transfer synchronized - dev to mem (RX) trasnfer */
		dev_dbg(uc->ud->dev, "%s: chan%d as DEV-to-MEM\n", __func__,
			uc->id);

		ret = am62l_udma_alloc_rx_resources(uc);
		if (ret) {
			uc->config.remote_thread_id = -1;
			return ret;
		}

		uc->config.src_thread = uc->config.remote_thread_id;
		uc->config.dst_thread = (ud->psil_base + uc->rchan->id) |
					K3_PSIL_DST_THREAD_ID_OFFSET;

		irq_ring_idx = uc->rchan->id;

		break;
	default:
		/* Can not happen */
		dev_err(uc->ud->dev, "%s: chan%d invalid direction (%u)\n",
			__func__, uc->id, uc->config.dir);
		return -EINVAL;
	}

	/* check if the channel configuration was successful */
	if (ret)
		goto err_res_free;

	if (udma_is_chan_running(uc)) {
		dev_warn(ud->dev, "chan%d: is running!\n", uc->id);
		ud->udma_reset_chan(uc, false);
		if (udma_is_chan_running(uc)) {
			dev_err(ud->dev, "chan%d: won't stop!\n", uc->id);
			ret = -EBUSY;
			goto err_res_free;
		}
	}

	uc->dma_dev = dmaengine_get_dma_device(chan);
	if (uc->config.dir == DMA_MEM_TO_MEM  && !uc->config.tr_trigger_type) {
		uc->config.hdesc_size = cppi5_trdesc_calc_size(
					sizeof(struct cppi5_tr_type15_t), 2);

		uc->hdesc_pool = dma_pool_create(uc->name, ud->ddev.dev,
						 uc->config.hdesc_size,
						 ud->desc_align,
						 0);
		if (!uc->hdesc_pool) {
			dev_err(ud->ddev.dev,
				"Descriptor pool allocation failed\n");
			uc->use_dma_pool = false;
			ret = -ENOMEM;
			goto err_res_free;
		}

		uc->use_dma_pool = true;
	} else if (uc->config.dir != DMA_MEM_TO_MEM) {
		uc->psil_paired = true;
	}

	out_irq.np = dev_of_node(ud->dev);
	out_irq.args_count = 1;
	out_irq.args[0] = irq_ring_idx;
	ret = of_irq_parse_raw(addr, &out_irq);
	if (ret)
		return ret;

	uc->irq_num_ring = irq_create_of_mapping(&out_irq);

	ret = devm_request_irq(ud->dev, uc->irq_num_ring, am62l_udma_ring_irq_handler,
			IRQF_TRIGGER_HIGH, uc->name, uc);
	if (ret) {
		dev_err(ud->dev, "chan%d: ring irq request failed\n", uc->id);
		goto err_irq_free;
	}

	ud->udma_reset_rings(uc);

	INIT_DELAYED_WORK_ONSTACK(&uc->tx_drain.work,
				  udma_check_tx_completion);
	return 0;

err_irq_free:
	uc->irq_num_ring = 0;
	uc->irq_num_udma = 0;
err_res_free:
	bcdma_free_bchan_resources(uc);
	udma_free_tx_resources(uc);
	udma_free_rx_resources(uc);

	udma_reset_uchan(uc);

	if (uc->use_dma_pool) {
		dma_pool_destroy(uc->hdesc_pool);
		uc->use_dma_pool = false;
	}

	return ret;
}

static enum dma_status am62l_udma_tx_status(struct dma_chan *chan,
				      dma_cookie_t cookie,
				      struct dma_tx_state *txstate)
{
	struct udma_chan *uc = to_udma_chan(chan);
	enum dma_status ret;
	unsigned long flags;

	spin_lock_irqsave(&uc->vc.lock, flags);

	ret = dma_cookie_status(chan, cookie, txstate);

	if (!udma_is_chan_running(uc))
		ret = DMA_COMPLETE;

	if (ret == DMA_IN_PROGRESS && am62l_udma_is_chan_paused(uc))
		ret = DMA_PAUSED;

	if (ret == DMA_COMPLETE || !txstate)
		goto out;

	if (uc->desc && uc->desc->vd.tx.cookie == cookie) {
		u32 peer_bcnt = 0;
		u32 bcnt = 0;
		u32 residue = uc->desc->residue;
		u32 delay = 0;

		if (uc->desc->dir == DMA_MEM_TO_DEV) {
			bcnt = udma_chanrt_read(uc, UDMA_CHAN_RT_SBCNT_REG);

			if (uc->config.ep_type != PSIL_EP_NATIVE) {
				peer_bcnt = udma_chanrt_read(uc, 0x810);

				if (bcnt > peer_bcnt)
					delay = bcnt - peer_bcnt;
			}
		} else if (uc->desc->dir == DMA_DEV_TO_MEM) {
			bcnt = udma_chanrt_read(uc, UDMA_CHAN_RT_BCNT_REG);

			if (uc->config.ep_type != PSIL_EP_NATIVE) {
				peer_bcnt = udma_chanrt_read(uc, 0x810);

				if (peer_bcnt > bcnt)
					delay = peer_bcnt - bcnt;
			}
		} else {
			bcnt = udma_chanrt_read(uc, UDMA_CHAN_RT_BCNT_REG);
		}

		if (bcnt && !(bcnt % uc->desc->residue))
			residue = 0;
		else
			residue -= bcnt % uc->desc->residue;

		if (!residue && (uc->config.dir == DMA_DEV_TO_MEM || !delay)) {
			ret = DMA_COMPLETE;
			delay = 0;
		}

		dma_set_residue(txstate, residue);
		dma_set_in_flight_bytes(txstate, delay);

	} else {
		ret = DMA_COMPLETE;
	}

out:
	spin_unlock_irqrestore(&uc->vc.lock, flags);
	return ret;
}

static int am62l_udma_pause(struct dma_chan *chan)
{
	struct udma_chan *uc = to_udma_chan(chan);

	/* pause the channel */
	udma_chanrt_update_bits(uc, UDMA_CHAN_RT_CTL_REG,
			UDMA_CHAN_RT_CTL_PAUSE, UDMA_CHAN_RT_CTL_PAUSE);

	return 0;
}

static int am62l_udma_resume(struct dma_chan *chan)
{
	struct udma_chan *uc = to_udma_chan(chan);

	/* resume the channel */
	udma_chanrt_update_bits(uc, UDMA_CHAN_RT_CTL_REG,
			UDMA_CHAN_RT_CTL_PAUSE, 0);

	return 0;
}

static struct platform_driver am62l_bcdma_v2_driver;

static bool am62l_udma_dma_filter_fn(struct dma_chan *chan, void *param)
{
	struct udma_chan_config *ucc;
	struct psil_endpoint_config *ep_config;
	struct am62l_udma_filter_param *filter_param;
	struct udma_chan *uc;
	struct udma_dev *ud;

	if (chan->device->dev->driver != &am62l_bcdma_v2_driver.driver)
		return false;

	uc = to_udma_chan(chan);
	ucc = &uc->config;
	ud = uc->ud;
	filter_param = param;

	if (filter_param->asel > 15) {
		dev_err(ud->dev, "Invalid channel asel: %u\n",
			filter_param->asel);
		return false;
	}

	ucc->remote_thread_id = filter_param->remote_thread_id;
	ucc->asel = filter_param->asel;
	ucc->tr_trigger_type = filter_param->tr_trigger_type;

	if (ucc->tr_trigger_type) {
		ucc->dir = DMA_MEM_TO_MEM;
		goto triggered_bchan;
	} else if (ucc->remote_thread_id & K3_PSIL_DST_THREAD_ID_OFFSET) {
		ucc->dir = DMA_MEM_TO_DEV;
	} else {
		ucc->dir = DMA_DEV_TO_MEM;
	}

	ep_config = psil_get_ep_config(ucc->remote_thread_id);
	if (IS_ERR(ep_config)) {
		dev_err(ud->dev, "No configuration for psi-l thread 0x%04x\n",
			ucc->remote_thread_id);
		ucc->dir = DMA_MEM_TO_MEM;
		ucc->remote_thread_id = -1;
		ucc->atype = 0;
		ucc->asel = 0;
		return false;
	}

	ucc->pkt_mode = ep_config->pkt_mode;
	ucc->channel_tpl = ep_config->channel_tpl;
	ucc->notdpkt = ep_config->notdpkt;
	ucc->ep_type = ep_config->ep_type;

	if ((ud->match_data->type == DMA_TYPE_BCDMA_V2) &&
		ep_config->mapped_channel_id >= 0) {
		ucc->mapped_channel_id = ep_config->mapped_channel_id;
		ucc->default_flow_id = ep_config->default_flow_id;
	} else {
		ucc->mapped_channel_id = -1;
		ucc->default_flow_id = -1;
	}

	ucc->needs_epib = ep_config->needs_epib;
	ucc->psd_size = ep_config->psd_size;
	ucc->metadata_size =
		(ucc->needs_epib ? CPPI5_INFO0_HDESC_EPIB_SIZE : 0) +
		ucc->psd_size;

	if (ucc->ep_type != PSIL_EP_NATIVE) {
		const struct udma_match_data *match_data = ud->match_data;

		if ((match_data->flags & UDMA_FLAG_PDMA_ACC32) && (ep_config->pdma_acc32))
			ucc->enable_acc32 = true;
		else
			ucc->enable_acc32 = false;

		if ((match_data->flags & UDMA_FLAG_PDMA_BURST) && (ep_config->pdma_burst))
			ucc->enable_burst = true;
		else
			ucc->enable_burst = false;
	}
	if (ucc->pkt_mode)
		ucc->hdesc_size = ALIGN(sizeof(struct cppi5_host_desc_t) +
				 ucc->metadata_size, ud->desc_align);

	dev_dbg(ud->dev, "chan%d: Remote thread: 0x%04x (%s)\n", uc->id,
		ucc->remote_thread_id, dmaengine_get_direction_text(ucc->dir));

	return true;

triggered_bchan:
	dev_dbg(ud->dev, "chan%d: triggered channel (type: %u)\n", uc->id,
		ucc->tr_trigger_type);

	return true;
}

static struct dma_chan *am62l_udma_of_xlate(struct of_phandle_args *dma_spec,
				      struct of_dma *ofdma)
{
	struct udma_dev *ud = ofdma->of_dma_data;
	dma_cap_mask_t mask = ud->ddev.cap_mask;
	struct am62l_udma_filter_param filter_param;
	struct dma_chan *chan;

	if (ud->match_data->type == DMA_TYPE_BCDMA_V2) {
		if (dma_spec->args_count != 4)
			return NULL;

		filter_param.tr_trigger_type = dma_spec->args[0];
		filter_param.trigger_param = dma_spec->args[1];
		filter_param.remote_thread_id = dma_spec->args[2];
		filter_param.asel = dma_spec->args[3];
	} else {
		if (dma_spec->args_count != 1 && dma_spec->args_count != 2)
			return NULL;

		filter_param.remote_thread_id = dma_spec->args[0];
		filter_param.tr_trigger_type = 0;
		if (dma_spec->args_count == 2)
			filter_param.asel = dma_spec->args[1];
		else
			filter_param.asel = 0;
	}

	chan = __dma_request_channel(&mask, am62l_udma_dma_filter_fn, &filter_param,
				     ofdma->of_node);
	if (!chan) {
		dev_err(ud->dev, "get channel fail in %s.\n", __func__);
		return ERR_PTR(-EINVAL);
	}

	return chan;
}

static struct udma_match_data am62l_bcdma_v2_data = {
	.type = DMA_TYPE_BCDMA_V2,
	.psil_base = 0x2000, /* for tchan and rchan, not applicable to bchan */
	.enable_memcpy_support = true, /* Supported via bchan */
	.flags = UDMA_FLAGS_J7_CLASS,
	.statictr_z_mask = GENMASK(23, 0),
	.burst_size = {
		TI_SCI_RM_UDMAP_CHAN_BURST_SIZE_64_BYTES, /* Normal Channels */
		0, /* No H Channels */
		0, /* No UH Channels */
	},
	.bchan_cnt = 16,
	.chan_cnt = 128,
	.tchan_cnt = 128,
	.rchan_cnt = 128,
};

static const struct of_device_id udma_of_match[] = {
	{
		.compatible = "ti,am62l-dmss-bcdma",
		.data = &am62l_bcdma_v2_data,
	},
	{ /* Sentinel */ },
};

static const struct soc_device_attribute k3_soc_devices[] = {
	{ .family = "AM62LX", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, udma_of_match);

static int am62l_udma_get_mmrs(struct platform_device *pdev, struct udma_dev *ud)
{
	u32 cap2, cap3;
	int i;

	ud->mmrs[AM62L_MMR_GCFG] = devm_platform_ioremap_resource_byname(pdev,
			am62l_mmr_names[AM62L_MMR_GCFG]);
	if (IS_ERR(ud->mmrs[AM62L_MMR_GCFG]))
		return PTR_ERR(ud->mmrs[AM62L_MMR_GCFG]);

	cap2 = udma_read(ud->mmrs[AM62L_MMR_GCFG], 0x28);
	cap3 = udma_read(ud->mmrs[AM62L_MMR_GCFG], 0x2c);

	ud->bchan_cnt = ud->match_data->bchan_cnt;
	/* There are no tchan and rchan in BCDMA_V2.
	 * Duplicate chan as tchan and rchan to keep the common code
	 * in k3-udma-common.c functional for BCDMA_V2.
	 */
	ud->chan_cnt = ud->match_data->chan_cnt;
	ud->tchan_cnt = ud->match_data->chan_cnt;
	ud->rchan_cnt = ud->match_data->chan_cnt;
	ud->rflow_cnt = ud->chan_cnt;

	for (i = 1; i < AM62L_MMR_LAST; i++) {
		if (i == AM62L_MMR_BCHANRT && ud->bchan_cnt == 0)
			continue;
		if (i == AM62L_MMR_CHANRT && ud->chan_cnt == 0)
			continue;

		ud->mmrs[i] = devm_platform_ioremap_resource_byname(pdev, am62l_mmr_names[i]);
		if (IS_ERR(ud->mmrs[i]))
			return PTR_ERR(ud->mmrs[i]);
	}

	return 0;
}

static int am62l_udma_probe(struct platform_device *pdev)
{
	const struct soc_device_attribute *soc;
	struct device *dev = &pdev->dev;
	struct udma_dev *ud;
	const struct of_device_id *match;
	int i, ret;
	int ch_count;

	ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(48));
	if (ret)
		dev_err(dev, "failed to set dma mask stuff\n");

	ud = devm_kzalloc(dev, sizeof(*ud), GFP_KERNEL);
	if (!ud)
		return -ENOMEM;

	match = of_match_node(udma_of_match, dev->of_node);
	if (!match) {
		dev_err(dev, "No compatible match found\n");
		return -ENODEV;
	}
	ud->match_data = match->data;

	ud->soc_data = ud->match_data->soc_data;
	if (!ud->soc_data) {
		soc = soc_device_match(k3_soc_devices);
		if (!soc) {
			dev_err(dev, "No compatible SoC found\n");
			return -ENODEV;
		}
		ud->soc_data = soc->data;
	}
	// Setup function pointers
	ud->udma_start = am62l_udma_start;
	ud->udma_stop = am62l_udma_stop;
	ud->udma_reset_chan = am62l_udma_reset_chan;
	ud->udma_reset_rings = am62l_udma_reset_rings;
	ud->udma_is_desc_really_done = am62l_udma_is_desc_really_done;
	ud->udma_decrement_byte_counters = am62l_udma_decrement_byte_counters;

	ret = am62l_udma_get_mmrs(pdev, ud);
	if (ret)
		return ret;

	struct k3_ringacc_init_data ring_init_data = {0};

	ring_init_data.num_rings = ud->bchan_cnt + ud->chan_cnt;

	ud->ringacc = k3_ringacc_dmarings_init(pdev, &ring_init_data);

	if (IS_ERR(ud->ringacc))
		return PTR_ERR(ud->ringacc);

	dma_cap_set(DMA_SLAVE, ud->ddev.cap_mask);

	dma_cap_set(DMA_CYCLIC, ud->ddev.cap_mask);
	ud->ddev.device_prep_dma_cyclic = udma_prep_dma_cyclic;

	ud->ddev.device_config = udma_slave_config;
	ud->ddev.device_prep_slave_sg = udma_prep_slave_sg;
	ud->ddev.device_issue_pending = udma_issue_pending;
	ud->ddev.device_tx_status = am62l_udma_tx_status;
	ud->ddev.device_pause = am62l_udma_pause;
	ud->ddev.device_resume = am62l_udma_resume;
	ud->ddev.device_terminate_all = udma_terminate_all;
	ud->ddev.device_synchronize = udma_synchronize;
#ifdef CONFIG_DEBUG_FS
	ud->ddev.dbg_summary_show = udma_dbg_summary_show;
#endif

	ud->ddev.device_alloc_chan_resources =
		am62l_bcdma_v2_alloc_chan_resources;

	ud->ddev.device_free_chan_resources = udma_free_chan_resources;

	ud->ddev.src_addr_widths = TI_UDMAC_BUSWIDTHS;
	ud->ddev.dst_addr_widths = TI_UDMAC_BUSWIDTHS;
	ud->ddev.directions = BIT(DMA_DEV_TO_MEM) | BIT(DMA_MEM_TO_DEV);
	ud->ddev.residue_granularity = DMA_RESIDUE_GRANULARITY_BURST;
	ud->ddev.desc_metadata_modes = DESC_METADATA_CLIENT |
				       DESC_METADATA_ENGINE;
	if (ud->match_data->enable_memcpy_support &&
	    !(ud->match_data->type == DMA_TYPE_BCDMA && ud->bchan_cnt == 0)) {
		dma_cap_set(DMA_MEMCPY, ud->ddev.cap_mask);
		ud->ddev.device_prep_dma_memcpy = udma_prep_dma_memcpy;
		ud->ddev.directions |= BIT(DMA_MEM_TO_MEM);
	}

	ud->ddev.dev = dev;
	ud->dev = dev;
	ud->psil_base = ud->match_data->psil_base;

	INIT_LIST_HEAD(&ud->ddev.channels);
	INIT_LIST_HEAD(&ud->desc_to_purge);

	ch_count = setup_resources(ud);
	if (ch_count <= 0)
		return ch_count;

	spin_lock_init(&ud->lock);
	INIT_WORK(&ud->purge_work, udma_purge_desc_work);

	ud->desc_align = 64;
	if (ud->desc_align < dma_get_cache_alignment())
		ud->desc_align = dma_get_cache_alignment();

	ret = udma_setup_rx_flush(ud);
	if (ret)
		return ret;

	for (i = 0; i < ud->bchan_cnt; i++) {
		struct udma_bchan *bchan = &ud->bchans[i];

		bchan->id = i;
		bchan->reg_rt = ud->mmrs[AM62L_MMR_BCHANRT] + i * 0x1000;
	}

	for (i = 0; i < ud->tchan_cnt; i++) {
		struct udma_tchan *tchan = &ud->tchans[i];

		tchan->id = i;
		tchan->reg_rt = ud->mmrs[AM62L_MMR_CHANRT] + i * 0x1000;
	}

	for (i = 0; i < ud->rchan_cnt; i++) {
		struct udma_rchan *rchan = &ud->rchans[i];

		rchan->id = i;
		rchan->reg_rt = ud->mmrs[AM62L_MMR_CHANRT] + i * 0x1000;
	}

	for (i = 0; i < ud->rflow_cnt; i++) {
		struct udma_rflow *rflow = &ud->rflows[i];

		rflow->id = i;
		rflow->reg_rt = ud->rflow_rt + i * 0x2000;
	}

	for (i = 0; i < ch_count; i++) {
		struct udma_chan *uc = &ud->channels[i];

		uc->ud = ud;
		uc->vc.desc_free = udma_desc_free;
		uc->id = i;
		uc->bchan = NULL;
		uc->tchan = NULL;
		uc->rchan = NULL;
		uc->config.remote_thread_id = -1;
		uc->config.mapped_channel_id = -1;
		uc->config.default_flow_id = -1;
		uc->config.dir = DMA_MEM_TO_MEM;
		uc->name = devm_kasprintf(dev, GFP_KERNEL, "%s chan%d",
					  dev_name(dev), i);

		vchan_init(&uc->vc, &ud->ddev);
		/* Use custom vchan completion handling */
		tasklet_setup(&uc->vc.task, udma_vchan_complete);
		init_completion(&uc->teardown_completed);
		INIT_DELAYED_WORK(&uc->tx_drain.work, udma_check_tx_completion);
	}

	/* Configure the copy_align to the maximum burst size the device supports */
	ud->ddev.copy_align = udma_get_copy_align(ud);

	ret = dma_async_device_register(&ud->ddev);
	if (ret) {
		dev_err(dev, "failed to register slave DMA engine: %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, ud);

	ret = of_dma_controller_register(dev->of_node, am62l_udma_of_xlate, ud);
	if (ret) {
		dev_err(dev, "failed to register of_dma controller\n");
		dma_async_device_unregister(&ud->ddev);
	}

	return ret;
}

static int __maybe_unused am62l_udma_pm_suspend(struct device *dev)
{
	struct udma_dev *ud = dev_get_drvdata(dev);
	struct dma_device *dma_dev = &ud->ddev;
	struct dma_chan *chan;
	struct udma_chan *uc;

	list_for_each_entry(chan, &dma_dev->channels, device_node) {
		if (chan->client_count) {
			uc = to_udma_chan(chan);
			/* backup the channel configuration */
			memcpy(&uc->backup_config, &uc->config,
			       sizeof(struct udma_chan_config));
			dev_dbg(dev, "Suspending channel %s\n",
				dma_chan_name(chan));
			ud->ddev.device_free_chan_resources(chan);
		}
	}

	return 0;
}

static int __maybe_unused am62l_udma_pm_resume(struct device *dev)
{
	struct udma_dev *ud = dev_get_drvdata(dev);
	struct dma_device *dma_dev = &ud->ddev;
	struct dma_chan *chan;
	struct udma_chan *uc;
	int ret;

	list_for_each_entry(chan, &dma_dev->channels, device_node) {
		if (chan->client_count) {
			uc = to_udma_chan(chan);
			/* restore the channel configuration */
			memcpy(&uc->config, &uc->backup_config,
			       sizeof(struct udma_chan_config));
			dev_dbg(dev, "Resuming channel %s\n",
				dma_chan_name(chan));
			ret = ud->ddev.device_alloc_chan_resources(chan);
			if (ret)
				return ret;
		}
	}

	return 0;
}

static const struct dev_pm_ops udma_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(am62l_udma_pm_suspend, am62l_udma_pm_resume)
};

static struct platform_driver am62l_bcdma_v2_driver = {
	.driver = {
		.name	= "ti-udma-am62l",
		.of_match_table = udma_of_match,
		.suppress_bind_attrs = true,
		.pm = &udma_pm_ops,
	},
	.probe		= am62l_udma_probe,
};

module_platform_driver(am62l_bcdma_v2_driver);
MODULE_DESCRIPTION("Texas Instruments K3 AM62L UDMA support");
MODULE_LICENSE("GPL");

