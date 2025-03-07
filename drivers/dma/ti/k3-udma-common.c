// SPDX-License-Identifier: GPL-2.0
/*
 *  Copyright (C) 2025 Texas Instruments Incorporated - http://www.ti.com
 *  Author: Peter Ujfalusi <peter.ujfalusi@ti.com>
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
#include <linux/soc/ti/k3-ringacc.h>
#include <linux/soc/ti/ti_sci_protocol.h>
#include <linux/soc/ti/ti_sci_inta_msi.h>
#include <linux/dma/k3-event-router.h>
#include <linux/dma/ti-cppi5.h>

#include "../virt-dma.h"
#include "k3-udma.h"
#include "k3-psil-priv.h"

static const char * const range_names[] = {
	[RM_RANGE_BCHAN] = "ti,sci-rm-range-bchan",
	[RM_RANGE_TCHAN] = "ti,sci-rm-range-tchan",
	[RM_RANGE_RCHAN] = "ti,sci-rm-range-rchan",
	[RM_RANGE_RFLOW] = "ti,sci-rm-range-rflow",
	[RM_RANGE_TFLOW] = "ti,sci-rm-range-tflow",
};

void k3_configure_chan_coherency(struct dma_chan *chan, u32 asel)
{
	struct device *chan_dev = &chan->dev->device;

	if (asel == 0) {
		/* No special handling for the channel */
		chan->dev->chan_dma_dev = false;

		chan_dev->dma_coherent = false;
		chan_dev->dma_parms = NULL;
	} else if (asel == 14 || asel == 15) {
		chan->dev->chan_dma_dev = true;

		chan_dev->dma_coherent = true;
		dma_coerce_mask_and_coherent(chan_dev, DMA_BIT_MASK(48));
		chan_dev->dma_parms = chan_dev->parent->dma_parms;
	} else {
		dev_warn(chan->device->dev, "Invalid ASEL value: %u\n", asel);

		chan_dev->dma_coherent = false;
		chan_dev->dma_parms = NULL;
	}
}

u8 udma_get_chan_tpl_index(struct udma_tpl *tpl_map, int chan_id)
{
	int i;

	for (i = 0; i < tpl_map->levels; i++) {
		if (chan_id >= tpl_map->start_idx[i])
			return i;
	}

	return 0;
}

void udma_reset_uchan(struct udma_chan *uc)
{
	memset(&uc->config, 0, sizeof(uc->config));
	uc->config.remote_thread_id = -1;
	uc->config.mapped_channel_id = -1;
	uc->config.default_flow_id = -1;
	uc->state = UDMA_CHAN_IS_IDLE;
}

void udma_dump_chan_stdata(struct udma_chan *uc)
{
	struct device *dev = uc->ud->dev;
	u32 offset;
	int i;

	if (uc->config.dir == DMA_MEM_TO_DEV || uc->config.dir == DMA_MEM_TO_MEM) {
		dev_dbg(dev, "TCHAN State data:\n");
		for (i = 0; i < 32; i++) {
			offset = UDMA_CHAN_RT_STDATA_REG + i * 4;
			dev_dbg(dev, "TRT_STDATA[%02d]: 0x%08x\n", i,
				udma_tchanrt_read(uc, offset));
		}
	}

	if (uc->config.dir == DMA_DEV_TO_MEM || uc->config.dir == DMA_MEM_TO_MEM) {
		dev_dbg(dev, "RCHAN State data:\n");
		for (i = 0; i < 32; i++) {
			offset = UDMA_CHAN_RT_STDATA_REG + i * 4;
			dev_dbg(dev, "RRT_STDATA[%02d]: 0x%08x\n", i,
				udma_rchanrt_read(uc, offset));
		}
	}
}

struct udma_desc *udma_udma_desc_from_paddr(struct udma_chan *uc,
						   dma_addr_t paddr)
{
	struct udma_desc *d = uc->terminated_desc;

	if (d) {
		dma_addr_t desc_paddr = udma_curr_cppi5_desc_paddr(d,
								   d->desc_idx);

		if (desc_paddr != paddr)
			d = NULL;
	}

	if (!d) {
		d = uc->desc;
		if (d) {
			dma_addr_t desc_paddr = udma_curr_cppi5_desc_paddr(d,
								d->desc_idx);

			if (desc_paddr != paddr)
				d = NULL;
		}
	}

	return d;
}

void udma_free_hwdesc(struct udma_chan *uc, struct udma_desc *d)
{
	if (uc->use_dma_pool) {
		int i;

		for (i = 0; i < d->hwdesc_count; i++) {
			if (!d->hwdesc[i].cppi5_desc_vaddr)
				continue;

			dma_pool_free(uc->hdesc_pool,
				      d->hwdesc[i].cppi5_desc_vaddr,
				      d->hwdesc[i].cppi5_desc_paddr);

			d->hwdesc[i].cppi5_desc_vaddr = NULL;
		}
	} else if (d->hwdesc[0].cppi5_desc_vaddr) {
		dma_free_coherent(uc->dma_dev, d->hwdesc[0].cppi5_desc_size,
				  d->hwdesc[0].cppi5_desc_vaddr,
				  d->hwdesc[0].cppi5_desc_paddr);

		d->hwdesc[0].cppi5_desc_vaddr = NULL;
	}
}

void udma_purge_desc_work(struct work_struct *work)
{
	struct udma_dev *ud = container_of(work, typeof(*ud), purge_work);
	struct virt_dma_desc *vd, *_vd;
	unsigned long flags;
	LIST_HEAD(head);

	spin_lock_irqsave(&ud->lock, flags);
	list_splice_tail_init(&ud->desc_to_purge, &head);
	spin_unlock_irqrestore(&ud->lock, flags);

	list_for_each_entry_safe(vd, _vd, &head, node) {
		struct udma_chan *uc = to_udma_chan(vd->tx.chan);
		struct udma_desc *d = to_udma_desc(&vd->tx);

		udma_free_hwdesc(uc, d);
		list_del(&vd->node);
		kfree(d);
	}

	/* If more to purge, schedule the work again */
	if (!list_empty(&ud->desc_to_purge))
		schedule_work(&ud->purge_work);
}

void udma_desc_free(struct virt_dma_desc *vd)
{
	struct udma_dev *ud = to_udma_dev(vd->tx.chan->device);
	struct udma_chan *uc = to_udma_chan(vd->tx.chan);
	struct udma_desc *d = to_udma_desc(&vd->tx);
	unsigned long flags;

	if (uc->terminated_desc == d)
		uc->terminated_desc = NULL;

	if (uc->use_dma_pool) {
		udma_free_hwdesc(uc, d);
		kfree(d);
		return;
	}

	spin_lock_irqsave(&ud->lock, flags);
	list_add_tail(&vd->node, &ud->desc_to_purge);
	spin_unlock_irqrestore(&ud->lock, flags);

	schedule_work(&ud->purge_work);
}

bool udma_is_chan_running(struct udma_chan *uc)
{
	u32 trt_ctl = 0;
	u32 rrt_ctl = 0;

	if (uc->tchan)
		trt_ctl = udma_tchanrt_read(uc, UDMA_CHAN_RT_CTL_REG);
	if (uc->rchan)
		rrt_ctl = udma_rchanrt_read(uc, UDMA_CHAN_RT_CTL_REG);

	if (trt_ctl & UDMA_CHAN_RT_CTL_EN || rrt_ctl & UDMA_CHAN_RT_CTL_EN)
		return true;

	return false;
}

void udma_reset_rings(struct udma_chan *uc)
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

	if (ring1)
		k3_ringacc_ring_reset_dma(ring1,
				k3_ringacc_ring_get_occ(ring1));
	if (ring2)
		k3_ringacc_ring_reset(ring2);

	/* make sure we are not leaking memory by stalled descriptor */
	if (uc->terminated_desc) {
		udma_desc_free(&uc->terminated_desc->vd);
		uc->terminated_desc = NULL;
	}
}

int udma_push_to_ring(struct udma_chan *uc, int idx)
{
	struct udma_desc *d = uc->desc;
	struct k3_ring *ring = NULL;
	dma_addr_t paddr;

	switch (uc->config.dir) {
	case DMA_DEV_TO_MEM:
		ring = uc->rflow->fd_ring;
		break;
	case DMA_MEM_TO_DEV:
	case DMA_MEM_TO_MEM:
		ring = uc->tchan->t_ring;
		break;
	default:
		return -EINVAL;
	}

	/* RX flush packet: idx == -1 is only passed in case of DEV_TO_MEM */
	if (idx == -1) {
		paddr = udma_get_rx_flush_hwdesc_paddr(uc);
	} else {
		paddr = udma_curr_cppi5_desc_paddr(d, idx);

		wmb(); /* Ensure that writes are not moved over this point */
	}

	return k3_ringacc_ring_push(ring, &paddr);
}

bool udma_desc_is_rx_flush(struct udma_chan *uc, dma_addr_t addr)
{
	if (uc->config.dir != DMA_DEV_TO_MEM)
		return false;

	if (addr == udma_get_rx_flush_hwdesc_paddr(uc))
		return true;

	return false;
}

int udma_pop_from_ring(struct udma_chan *uc, dma_addr_t *addr)
{
	struct k3_ring *ring = NULL;
	int ret;

	switch (uc->config.dir) {
	case DMA_DEV_TO_MEM:
		ring = uc->rflow->r_ring;
		break;
	case DMA_MEM_TO_DEV:
	case DMA_MEM_TO_MEM:
		ring = uc->tchan->tc_ring;
		break;
	default:
		return -ENOENT;
	}

	ret = k3_ringacc_ring_pop(ring, addr);
	if (ret)
		return ret;

	rmb(); /* Ensure that reads are not moved before this point */

	/* Teardown completion */
	if (cppi5_desc_is_tdcm(*addr))
		return 0;

	/* Check for flush descriptor */
	if (udma_desc_is_rx_flush(uc, *addr))
		return -ENOENT;

	return 0;
}

void udma_start_desc(struct udma_chan *uc)
{
	struct udma_chan_config *ucc = &uc->config;

	if (uc->ud->match_data->type == DMA_TYPE_UDMA && ucc->pkt_mode &&
	    (uc->cyclic || ucc->dir == DMA_DEV_TO_MEM)) {
		int i;

		/*
		 * UDMA only: Push all descriptors to ring for packet mode
		 * cyclic or RX
		 * PKTDMA supports pre-linked descriptor and cyclic is not
		 * supported
		 */
		for (i = 0; i < uc->desc->sglen; i++)
			udma_push_to_ring(uc, i);
	} else {
		udma_push_to_ring(uc, 0);
	}
}

bool udma_chan_needs_reconfiguration(struct udma_chan *uc)
{
	/* Only PDMAs have staticTR */
	if (uc->config.ep_type == PSIL_EP_NATIVE)
		return false;

	/* Check if the staticTR configuration has changed for TX */
	if (memcmp(&uc->static_tr, &uc->desc->static_tr, sizeof(uc->static_tr)))
		return true;

	return false;
}

void udma_cyclic_packet_elapsed(struct udma_chan *uc)
{
	struct udma_desc *d = uc->desc;
	struct cppi5_host_desc_t *h_desc;

	h_desc = d->hwdesc[d->desc_idx].cppi5_desc_vaddr;
	cppi5_hdesc_reset_to_original(h_desc);
	udma_push_to_ring(uc, d->desc_idx);
	d->desc_idx = (d->desc_idx + 1) % d->sglen;
}

void udma_check_tx_completion(struct work_struct *work)
{
	struct udma_chan *uc = container_of(work, typeof(*uc),
					    tx_drain.work.work);
	struct udma_dev *ud = uc->ud;
	bool desc_done = true;
	u32 residue_diff;
	ktime_t time_diff;
	unsigned long delay;

	while (1) {
		if (uc->desc) {
			/* Get previous residue and time stamp */
			residue_diff = uc->tx_drain.residue;
			time_diff = uc->tx_drain.tstamp;
			/*
			 * Get current residue and time stamp or see if
			 * transfer is complete
			 */
			desc_done = ud->udma_is_desc_really_done(uc, uc->desc);
		}

		if (!desc_done) {
			/*
			 * Find the time delta and residue delta w.r.t
			 * previous poll
			 */
			time_diff = ktime_sub(uc->tx_drain.tstamp,
					      time_diff) + 1;
			residue_diff -= uc->tx_drain.residue;
			if (residue_diff) {
				/*
				 * Try to guess when we should check
				 * next time by calculating rate at
				 * which data is being drained at the
				 * peer device
				 */
				delay = (time_diff / residue_diff) *
					uc->tx_drain.residue;
			} else {
				/* No progress, check again in 1 second  */
				schedule_delayed_work(&uc->tx_drain.work, HZ);
				break;
			}

			usleep_range(ktime_to_us(delay),
				     ktime_to_us(delay) + 10);
			continue;
		}

		if (uc->desc) {
			struct udma_desc *d = uc->desc;

			ud->udma_decrement_byte_counters(uc, d->residue);
			ud->udma_start(uc);
			vchan_cookie_complete(&d->vd);
			break;
		}

		break;
	}
}

/**
 * __udma_alloc_gp_rflow_range - alloc range of GP RX flows
 * @ud: UDMA device
 * @from: Start the search from this flow id number
 * @cnt: Number of consecutive flow ids to allocate
 *
 * Allocate range of RX flow ids for future use, those flows can be requested
 * only using explicit flow id number. if @from is set to -1 it will try to find
 * first free range. if @from is positive value it will force allocation only
 * of the specified range of flows.
 *
 * Returns -ENOMEM if can't find free range.
 * -EEXIST if requested range is busy.
 * -EINVAL if wrong input values passed.
 * Returns flow id on success.
 */
int __udma_alloc_gp_rflow_range(struct udma_dev *ud, int from, int cnt)
{
	int start, tmp_from;
	DECLARE_BITMAP(tmp, K3_UDMA_MAX_RFLOWS);

	tmp_from = from;
	if (tmp_from < 0)
		tmp_from = ud->rchan_cnt;
	/* default flows can't be allocated and accessible only by id */
	if (tmp_from < ud->rchan_cnt)
		return -EINVAL;

	if (tmp_from + cnt > ud->rflow_cnt)
		return -EINVAL;

	bitmap_or(tmp, ud->rflow_gp_map, ud->rflow_gp_map_allocated,
		  ud->rflow_cnt);

	start = bitmap_find_next_zero_area(tmp,
					   ud->rflow_cnt,
					   tmp_from, cnt, 0);
	if (start >= ud->rflow_cnt)
		return -ENOMEM;

	if (from >= 0 && start != from)
		return -EEXIST;

	bitmap_set(ud->rflow_gp_map_allocated, start, cnt);
	return start;
}

int __udma_free_gp_rflow_range(struct udma_dev *ud, int from, int cnt)
{
	if (from < ud->rchan_cnt)
		return -EINVAL;
	if (from + cnt > ud->rflow_cnt)
		return -EINVAL;

	bitmap_clear(ud->rflow_gp_map_allocated, from, cnt);
	return 0;
}

struct udma_rflow *__udma_get_rflow(struct udma_dev *ud, int id)
{
	/*
	 * Attempt to request rflow by ID can be made for any rflow
	 * if not in use with assumption that caller knows what's doing.
	 * TI-SCI FW will perform additional permission check ant way, it's
	 * safe
	 */

	if (id < 0 || id >= ud->rflow_cnt)
		return ERR_PTR(-ENOENT);

	if (test_bit(id, ud->rflow_in_use))
		return ERR_PTR(-ENOENT);

	if (ud->rflow_gp_map) {
		/* GP rflow has to be allocated first */
		if (!test_bit(id, ud->rflow_gp_map) &&
		    !test_bit(id, ud->rflow_gp_map_allocated))
			return ERR_PTR(-EINVAL);
	}

	dev_dbg(ud->dev, "get rflow%d\n", id);
	set_bit(id, ud->rflow_in_use);
	return &ud->rflows[id];
}

void __udma_put_rflow(struct udma_dev *ud, struct udma_rflow *rflow)
{
	if (!test_bit(rflow->id, ud->rflow_in_use)) {
		dev_err(ud->dev, "attempt to put unused rflow%d\n", rflow->id);
		return;
	}

	dev_dbg(ud->dev, "put rflow%d\n", rflow->id);
	clear_bit(rflow->id, ud->rflow_in_use);
}

#define UDMA_RESERVE_RESOURCE(res)					\
struct udma_##res *__udma_reserve_##res(struct udma_dev *ud,	\
					       enum udma_tp_level tpl,	\
					       int id)			\
{									\
	if (id >= 0) {							\
		if (test_bit(id, ud->res##_map)) {			\
			dev_err(ud->dev, "res##%d is in use\n", id);	\
			return ERR_PTR(-ENOENT);			\
		}							\
	} else {							\
		int start;						\
									\
		if (tpl >= ud->res##_tpl.levels)			\
			tpl = ud->res##_tpl.levels - 1;			\
									\
		start = ud->res##_tpl.start_idx[tpl];			\
									\
		id = find_next_zero_bit(ud->res##_map, ud->res##_cnt,	\
					start);				\
		if (id == ud->res##_cnt) {				\
			return ERR_PTR(-ENOENT);			\
		}							\
	}								\
									\
	set_bit(id, ud->res##_map);					\
	return &ud->res##s[id];						\
}

UDMA_RESERVE_RESOURCE(bchan);
UDMA_RESERVE_RESOURCE(tchan);
UDMA_RESERVE_RESOURCE(rchan);

int udma_get_tchan(struct udma_chan *uc)
{
	struct udma_dev *ud = uc->ud;
	int ret;

	if (uc->tchan) {
		dev_dbg(ud->dev, "chan%d: already have tchan%d allocated\n",
			uc->id, uc->tchan->id);
		return 0;
	}

	/*
	 * mapped_channel_id is -1 for UDMA, BCDMA and PKTDMA unmapped channels.
	 * For PKTDMA mapped channels it is configured to a channel which must
	 * be used to service the peripheral.
	 */
	uc->tchan = __udma_reserve_tchan(ud, uc->config.channel_tpl,
					 uc->config.mapped_channel_id);
	if (IS_ERR(uc->tchan)) {
		ret = PTR_ERR(uc->tchan);
		uc->tchan = NULL;
		return ret;
	}
	if (ud->match_data->type == DMA_TYPE_BCDMA_V2)
		uc->chan = uc->tchan;

	if (ud->tflow_cnt) {
		int tflow_id;

		/* Only PKTDMA have support for tx flows */
		if (uc->config.default_flow_id >= 0)
			tflow_id = uc->config.default_flow_id;
		else
			tflow_id = uc->tchan->id;

		if (test_bit(tflow_id, ud->tflow_map)) {
			dev_err(ud->dev, "tflow%d is in use\n", tflow_id);
			clear_bit(uc->tchan->id, ud->tchan_map);
			uc->tchan = NULL;
			return -ENOENT;
		}

		uc->tchan->tflow_id = tflow_id;
		set_bit(tflow_id, ud->tflow_map);
	} else {
		uc->tchan->tflow_id = -1;
	}

	return 0;
}

int udma_get_rchan(struct udma_chan *uc)
{
	struct udma_dev *ud = uc->ud;
	int ret;

	if (uc->rchan) {
		dev_dbg(ud->dev, "chan%d: already have rchan%d allocated\n",
			uc->id, uc->rchan->id);
		return 0;
	}

	/*
	 * mapped_channel_id is -1 for UDMA, BCDMA and PKTDMA unmapped channels.
	 * For PKTDMA mapped channels it is configured to a channel which must
	 * be used to service the peripheral.
	 */
	uc->rchan = __udma_reserve_rchan(ud, uc->config.channel_tpl,
					 uc->config.mapped_channel_id);
	if (IS_ERR(uc->rchan)) {
		ret = PTR_ERR(uc->rchan);
		uc->rchan = NULL;
		return ret;
	}
	if (ud->match_data->type == DMA_TYPE_BCDMA_V2)
		uc->chan = uc->rchan;

	return 0;
}

int udma_get_chan_pair(struct udma_chan *uc)
{
	struct udma_dev *ud = uc->ud;
	int chan_id, end;

	if ((uc->tchan && uc->rchan) && uc->tchan->id == uc->rchan->id) {
		dev_info(ud->dev, "chan%d: already have %d pair allocated\n",
			 uc->id, uc->tchan->id);
		return 0;
	}

	if (uc->tchan) {
		dev_err(ud->dev, "chan%d: already have tchan%d allocated\n",
			uc->id, uc->tchan->id);
		return -EBUSY;
	} else if (uc->rchan) {
		dev_err(ud->dev, "chan%d: already have rchan%d allocated\n",
			uc->id, uc->rchan->id);
		return -EBUSY;
	}

	/* Can be optimized, but let's have it like this for now */
	end = min(ud->tchan_cnt, ud->rchan_cnt);
	/*
	 * Try to use the highest TPL channel pair for MEM_TO_MEM channels
	 * Note: in UDMAP the channel TPL is symmetric between tchan and rchan
	 */
	chan_id = ud->tchan_tpl.start_idx[ud->tchan_tpl.levels - 1];
	for (; chan_id < end; chan_id++) {
		if (!test_bit(chan_id, ud->tchan_map) &&
		    !test_bit(chan_id, ud->rchan_map))
			break;
	}

	if (chan_id == end)
		return -ENOENT;

	set_bit(chan_id, ud->tchan_map);
	set_bit(chan_id, ud->rchan_map);
	uc->tchan = &ud->tchans[chan_id];
	uc->rchan = &ud->rchans[chan_id];

	/* UDMA does not use tx flows */
	uc->tchan->tflow_id = -1;

	return 0;
}

int udma_get_rflow(struct udma_chan *uc, int flow_id)
{
	struct udma_dev *ud = uc->ud;
	int ret;

	if (!uc->rchan) {
		dev_err(ud->dev, "chan%d: does not have rchan??\n", uc->id);
		return -EINVAL;
	}

	if (uc->rflow) {
		dev_dbg(ud->dev, "chan%d: already have rflow%d allocated\n",
			uc->id, uc->rflow->id);
		return 0;
	}

	uc->rflow = __udma_get_rflow(ud, flow_id);
	if (IS_ERR(uc->rflow)) {
		ret = PTR_ERR(uc->rflow);
		uc->rflow = NULL;
		return ret;
	}

	return 0;
}

void bcdma_put_bchan(struct udma_chan *uc)
{
	struct udma_dev *ud = uc->ud;

	if (uc->bchan) {
		dev_dbg(ud->dev, "chan%d: put bchan%d\n", uc->id,
			uc->bchan->id);
		clear_bit(uc->bchan->id, ud->bchan_map);
		uc->bchan = NULL;
		uc->tchan = NULL;
	}
}

void udma_put_rchan(struct udma_chan *uc)
{
	struct udma_dev *ud = uc->ud;

	if (uc->rchan) {
		dev_dbg(ud->dev, "chan%d: put rchan%d\n", uc->id,
			uc->rchan->id);
		clear_bit(uc->rchan->id, ud->rchan_map);
		uc->rchan = NULL;
	}
}

void udma_put_tchan(struct udma_chan *uc)
{
	struct udma_dev *ud = uc->ud;

	if (uc->tchan) {
		dev_dbg(ud->dev, "chan%d: put tchan%d\n", uc->id,
			uc->tchan->id);
		clear_bit(uc->tchan->id, ud->tchan_map);

		if (uc->tchan->tflow_id >= 0)
			clear_bit(uc->tchan->tflow_id, ud->tflow_map);

		uc->tchan = NULL;
	}
}

void udma_put_rflow(struct udma_chan *uc)
{
	struct udma_dev *ud = uc->ud;

	if (uc->rflow) {
		dev_dbg(ud->dev, "chan%d: put rflow%d\n", uc->id,
			uc->rflow->id);
		__udma_put_rflow(ud, uc->rflow);
		uc->rflow = NULL;
	}
}

void bcdma_free_bchan_resources(struct udma_chan *uc)
{
	if (!uc->bchan)
		return;

	k3_ringacc_ring_free(uc->bchan->tc_ring);
	k3_ringacc_ring_free(uc->bchan->t_ring);
	uc->bchan->tc_ring = NULL;
	uc->bchan->t_ring = NULL;
	k3_configure_chan_coherency(&uc->vc.chan, 0);

	bcdma_put_bchan(uc);
}

void udma_free_tx_resources(struct udma_chan *uc)
{
	if (!uc->tchan)
		return;

	k3_ringacc_ring_free(uc->tchan->t_ring);
	k3_ringacc_ring_free(uc->tchan->tc_ring);
	uc->tchan->t_ring = NULL;
	uc->tchan->tc_ring = NULL;

	udma_put_tchan(uc);
}

void udma_free_rx_resources(struct udma_chan *uc)
{
	if (!uc->rchan)
		return;

	if (uc->rflow) {
		struct udma_rflow *rflow = uc->rflow;

		k3_ringacc_ring_free(rflow->fd_ring);
		k3_ringacc_ring_free(rflow->r_ring);
		rflow->fd_ring = NULL;
		rflow->r_ring = NULL;

		udma_put_rflow(uc);
	}

	udma_put_rchan(uc);
}

int udma_slave_config(struct dma_chan *chan,
			     struct dma_slave_config *cfg)
{
	struct udma_chan *uc = to_udma_chan(chan);

	memcpy(&uc->cfg, cfg, sizeof(uc->cfg));

	return 0;
}

struct udma_desc *udma_alloc_tr_desc(struct udma_chan *uc,
					    size_t tr_size, int tr_count,
					    enum dma_transfer_direction dir)
{
	struct udma_hwdesc *hwdesc;
	struct cppi5_desc_hdr_t *tr_desc;
	struct udma_desc *d;
	u32 reload_count = 0;
	u32 ring_id;

	switch (tr_size) {
	case 16:
	case 32:
	case 64:
	case 128:
		break;
	default:
		dev_err(uc->ud->dev, "Unsupported TR size of %zu\n", tr_size);
		return NULL;
	}

	/* We have only one descriptor containing multiple TRs */
	d = kzalloc(sizeof(*d) + sizeof(d->hwdesc[0]), GFP_NOWAIT);
	if (!d)
		return NULL;

	d->sglen = tr_count;

	d->hwdesc_count = 1;
	hwdesc = &d->hwdesc[0];

	/* Allocate memory for DMA ring descriptor */
	if (uc->use_dma_pool) {
		hwdesc->cppi5_desc_size = uc->config.hdesc_size;
		hwdesc->cppi5_desc_vaddr = dma_pool_zalloc(uc->hdesc_pool,
						GFP_NOWAIT,
						&hwdesc->cppi5_desc_paddr);
	} else {
		hwdesc->cppi5_desc_size = cppi5_trdesc_calc_size(tr_size,
								 tr_count);
		hwdesc->cppi5_desc_size = ALIGN(hwdesc->cppi5_desc_size,
						uc->ud->desc_align);
		hwdesc->cppi5_desc_vaddr = dma_alloc_coherent(uc->ud->dev,
						hwdesc->cppi5_desc_size,
						&hwdesc->cppi5_desc_paddr,
						GFP_NOWAIT);
	}

	if (!hwdesc->cppi5_desc_vaddr) {
		kfree(d);
		return NULL;
	}

	/* Start of the TR req records */
	hwdesc->tr_req_base = hwdesc->cppi5_desc_vaddr + tr_size;
	/* Start address of the TR response array */
	hwdesc->tr_resp_base = hwdesc->tr_req_base + tr_size * tr_count;

	tr_desc = hwdesc->cppi5_desc_vaddr;

	if (uc->cyclic)
		reload_count = CPPI5_INFO0_TRDESC_RLDCNT_INFINITE;

	if (dir == DMA_DEV_TO_MEM)
		ring_id = k3_ringacc_get_ring_id(uc->rflow->r_ring);
	else
		ring_id = k3_ringacc_get_ring_id(uc->tchan->tc_ring);

	cppi5_trdesc_init(tr_desc, tr_count, tr_size, 0, reload_count);
	cppi5_desc_set_pktids(tr_desc, uc->id,
			      CPPI5_INFO1_DESC_FLOWID_DEFAULT);
	cppi5_desc_set_retpolicy(tr_desc, 0, ring_id);

	return d;
}

/**
 * udma_get_tr_counters - calculate TR counters for a given length
 * @len: Length of the trasnfer
 * @align_to: Preferred alignment
 * @tr0_cnt0: First TR icnt0
 * @tr0_cnt1: First TR icnt1
 * @tr1_cnt0: Second (if used) TR icnt0
 *
 * For len < SZ_64K only one TR is enough, tr1_cnt0 is not updated
 * For len >= SZ_64K two TRs are used in a simple way:
 * First TR: SZ_64K-alignment blocks (tr0_cnt0, tr0_cnt1)
 * Second TR: the remaining length (tr1_cnt0)
 *
 * Returns the number of TRs the length needs (1 or 2)
 * -EINVAL if the length can not be supported
 */
int udma_get_tr_counters(size_t len, unsigned long align_to,
				u16 *tr0_cnt0, u16 *tr0_cnt1, u16 *tr1_cnt0)
{
	if (len < SZ_64K) {
		*tr0_cnt0 = len;
		*tr0_cnt1 = 1;

		return 1;
	}

	if (align_to > 3)
		align_to = 3;

realign:
	*tr0_cnt0 = SZ_64K - BIT(align_to);
	if (len / *tr0_cnt0 >= SZ_64K) {
		if (align_to) {
			align_to--;
			goto realign;
		}
		return -EINVAL;
	}

	*tr0_cnt1 = len / *tr0_cnt0;
	*tr1_cnt0 = len % *tr0_cnt0;

	return 2;
}

struct udma_desc *
udma_prep_slave_sg_tr(struct udma_chan *uc, struct scatterlist *sgl,
		      unsigned int sglen, enum dma_transfer_direction dir,
		      unsigned long tx_flags, void *context)
{
	struct scatterlist *sgent;
	struct udma_desc *d;
	struct cppi5_tr_type1_t *tr_req = NULL;
	u16 tr0_cnt0, tr0_cnt1, tr1_cnt0;
	unsigned int i;
	size_t tr_size;
	int num_tr = 0;
	int tr_idx = 0;
	u32 extra_flags = 0;
	u64 asel;

	/* estimate the number of TRs we will need */
	for_each_sg(sgl, sgent, sglen, i) {
		if (sg_dma_len(sgent) < SZ_64K)
			num_tr++;
		else
			num_tr += 2;
	}

	/* Now allocate and setup the descriptor. */
	tr_size = sizeof(struct cppi5_tr_type1_t);
	d = udma_alloc_tr_desc(uc, tr_size, num_tr, dir);
	if (!d)
		return NULL;

	d->sglen = sglen;

	if (uc->ud->match_data->type == DMA_TYPE_UDMA)
		asel = 0;
	else
		asel = (u64)uc->config.asel << K3_ADDRESS_ASEL_SHIFT;

	if (dir == DMA_MEM_TO_DEV && uc->ud->match_data->type == DMA_TYPE_BCDMA_V2)
		extra_flags = CPPI5_TR_CSF_EOP;

	tr_req = d->hwdesc[0].tr_req_base;
	for_each_sg(sgl, sgent, sglen, i) {
		dma_addr_t sg_addr = sg_dma_address(sgent);

		num_tr = udma_get_tr_counters(sg_dma_len(sgent), __ffs(sg_addr),
					      &tr0_cnt0, &tr0_cnt1, &tr1_cnt0);
		if (num_tr < 0) {
			dev_err(uc->ud->dev, "size %u is not supported\n",
				sg_dma_len(sgent));
			udma_free_hwdesc(uc, d);
			kfree(d);
			return NULL;
		}

		cppi5_tr_init(&tr_req[tr_idx].flags, CPPI5_TR_TYPE1, false,
			      false, CPPI5_TR_EVENT_SIZE_COMPLETION, 0);
		cppi5_tr_csf_set(&tr_req[tr_idx].flags, CPPI5_TR_CSF_SUPR_EVT | extra_flags);

		sg_addr |= asel;
		tr_req[tr_idx].addr = sg_addr;
		tr_req[tr_idx].icnt0 = tr0_cnt0;
		tr_req[tr_idx].icnt1 = tr0_cnt1;
		tr_req[tr_idx].dim1 = tr0_cnt0;
		tr_idx++;

		if (num_tr == 2) {
			cppi5_tr_init(&tr_req[tr_idx].flags, CPPI5_TR_TYPE1,
				      false, false,
				      CPPI5_TR_EVENT_SIZE_COMPLETION, 0);
			cppi5_tr_csf_set(&tr_req[tr_idx].flags,
					 CPPI5_TR_CSF_SUPR_EVT | extra_flags);

			tr_req[tr_idx].addr = sg_addr + tr0_cnt1 * tr0_cnt0;
			tr_req[tr_idx].icnt0 = tr1_cnt0;
			tr_req[tr_idx].icnt1 = 1;
			tr_req[tr_idx].dim1 = tr1_cnt0;
			tr_idx++;
		}

		d->residue += sg_dma_len(sgent);
	}

	cppi5_tr_csf_set(&tr_req[tr_idx - 1].flags,
			 CPPI5_TR_CSF_SUPR_EVT | CPPI5_TR_CSF_EOP);

	return d;
}

struct udma_desc *
udma_prep_slave_sg_triggered_tr(struct udma_chan *uc, struct scatterlist *sgl,
				unsigned int sglen,
				enum dma_transfer_direction dir,
				unsigned long tx_flags, void *context)
{
	struct scatterlist *sgent;
	struct cppi5_tr_type15_t *tr_req = NULL;
	enum dma_slave_buswidth dev_width;
	u32 csf = CPPI5_TR_CSF_SUPR_EVT;
	u16 tr_cnt0, tr_cnt1;
	dma_addr_t dev_addr;
	struct udma_desc *d;
	unsigned int i;
	size_t tr_size, sg_len;
	int num_tr = 0;
	int tr_idx = 0;
	u32 burst, trigger_size, port_window;
	u64 asel;

	if (dir == DMA_DEV_TO_MEM) {
		dev_addr = uc->cfg.src_addr;
		dev_width = uc->cfg.src_addr_width;
		burst = uc->cfg.src_maxburst;
		port_window = uc->cfg.src_port_window_size;
	} else if (dir == DMA_MEM_TO_DEV) {
		dev_addr = uc->cfg.dst_addr;
		dev_width = uc->cfg.dst_addr_width;
		burst = uc->cfg.dst_maxburst;
		port_window = uc->cfg.dst_port_window_size;
	} else {
		dev_err(uc->ud->dev, "%s: bad direction?\n", __func__);
		return NULL;
	}

	if (!burst)
		burst = 1;

	if (port_window) {
		if (port_window != burst) {
			dev_err(uc->ud->dev,
				"The burst must be equal to port_window\n");
			return NULL;
		}

		tr_cnt0 = dev_width * port_window;
		tr_cnt1 = 1;
	} else {
		tr_cnt0 = dev_width;
		tr_cnt1 = burst;
	}
	trigger_size = tr_cnt0 * tr_cnt1;

	/* estimate the number of TRs we will need */
	for_each_sg(sgl, sgent, sglen, i) {
		sg_len = sg_dma_len(sgent);

		if (sg_len % trigger_size) {
			dev_err(uc->ud->dev,
				"Not aligned SG entry (%zu for %u)\n", sg_len,
				trigger_size);
			return NULL;
		}

		if (sg_len / trigger_size < SZ_64K)
			num_tr++;
		else
			num_tr += 2;
	}

	/* Now allocate and setup the descriptor. */
	tr_size = sizeof(struct cppi5_tr_type15_t);
	d = udma_alloc_tr_desc(uc, tr_size, num_tr, dir);
	if (!d)
		return NULL;

	d->sglen = sglen;

	if (uc->ud->match_data->type == DMA_TYPE_UDMA) {
		asel = 0;
		csf |= CPPI5_TR_CSF_EOL_ICNT0;
	} else {
		asel = (u64)uc->config.asel << K3_ADDRESS_ASEL_SHIFT;
		dev_addr |= asel;
	}

	tr_req = d->hwdesc[0].tr_req_base;
	for_each_sg(sgl, sgent, sglen, i) {
		u16 tr0_cnt2, tr0_cnt3, tr1_cnt2;
		dma_addr_t sg_addr = sg_dma_address(sgent);

		sg_len = sg_dma_len(sgent);
		num_tr = udma_get_tr_counters(sg_len / trigger_size, 0,
					      &tr0_cnt2, &tr0_cnt3, &tr1_cnt2);
		if (num_tr < 0) {
			dev_err(uc->ud->dev, "size %zu is not supported\n",
				sg_len);
			udma_free_hwdesc(uc, d);
			kfree(d);
			return NULL;
		}

		cppi5_tr_init(&tr_req[tr_idx].flags, CPPI5_TR_TYPE15, false,
			      true, CPPI5_TR_EVENT_SIZE_COMPLETION, 0);
		cppi5_tr_csf_set(&tr_req[tr_idx].flags, csf);
		cppi5_tr_set_trigger(&tr_req[tr_idx].flags,
				     uc->config.tr_trigger_type,
				     CPPI5_TR_TRIGGER_TYPE_ICNT2_DEC, 0, 0);

		sg_addr |= asel;
		if (dir == DMA_DEV_TO_MEM) {
			tr_req[tr_idx].addr = dev_addr;
			tr_req[tr_idx].icnt0 = tr_cnt0;
			tr_req[tr_idx].icnt1 = tr_cnt1;
			tr_req[tr_idx].icnt2 = tr0_cnt2;
			tr_req[tr_idx].icnt3 = tr0_cnt3;
			tr_req[tr_idx].dim1 = (-1) * tr_cnt0;

			tr_req[tr_idx].daddr = sg_addr;
			tr_req[tr_idx].dicnt0 = tr_cnt0;
			tr_req[tr_idx].dicnt1 = tr_cnt1;
			tr_req[tr_idx].dicnt2 = tr0_cnt2;
			tr_req[tr_idx].dicnt3 = tr0_cnt3;
			tr_req[tr_idx].ddim1 = tr_cnt0;
			tr_req[tr_idx].ddim2 = trigger_size;
			tr_req[tr_idx].ddim3 = trigger_size * tr0_cnt2;
		} else {
			tr_req[tr_idx].addr = sg_addr;
			tr_req[tr_idx].icnt0 = tr_cnt0;
			tr_req[tr_idx].icnt1 = tr_cnt1;
			tr_req[tr_idx].icnt2 = tr0_cnt2;
			tr_req[tr_idx].icnt3 = tr0_cnt3;
			tr_req[tr_idx].dim1 = tr_cnt0;
			tr_req[tr_idx].dim2 = trigger_size;
			tr_req[tr_idx].dim3 = trigger_size * tr0_cnt2;

			tr_req[tr_idx].daddr = dev_addr;
			tr_req[tr_idx].dicnt0 = tr_cnt0;
			tr_req[tr_idx].dicnt1 = tr_cnt1;
			tr_req[tr_idx].dicnt2 = tr0_cnt2;
			tr_req[tr_idx].dicnt3 = tr0_cnt3;
			tr_req[tr_idx].ddim1 = (-1) * tr_cnt0;
		}

		tr_idx++;

		if (num_tr == 2) {
			cppi5_tr_init(&tr_req[tr_idx].flags, CPPI5_TR_TYPE15,
				      false, true,
				      CPPI5_TR_EVENT_SIZE_COMPLETION, 0);
			cppi5_tr_csf_set(&tr_req[tr_idx].flags, csf);
			cppi5_tr_set_trigger(&tr_req[tr_idx].flags,
					     uc->config.tr_trigger_type,
					     CPPI5_TR_TRIGGER_TYPE_ICNT2_DEC,
					     0, 0);

			sg_addr += trigger_size * tr0_cnt2 * tr0_cnt3;
			if (dir == DMA_DEV_TO_MEM) {
				tr_req[tr_idx].addr = dev_addr;
				tr_req[tr_idx].icnt0 = tr_cnt0;
				tr_req[tr_idx].icnt1 = tr_cnt1;
				tr_req[tr_idx].icnt2 = tr1_cnt2;
				tr_req[tr_idx].icnt3 = 1;
				tr_req[tr_idx].dim1 = (-1) * tr_cnt0;

				tr_req[tr_idx].daddr = sg_addr;
				tr_req[tr_idx].dicnt0 = tr_cnt0;
				tr_req[tr_idx].dicnt1 = tr_cnt1;
				tr_req[tr_idx].dicnt2 = tr1_cnt2;
				tr_req[tr_idx].dicnt3 = 1;
				tr_req[tr_idx].ddim1 = tr_cnt0;
				tr_req[tr_idx].ddim2 = trigger_size;
			} else {
				tr_req[tr_idx].addr = sg_addr;
				tr_req[tr_idx].icnt0 = tr_cnt0;
				tr_req[tr_idx].icnt1 = tr_cnt1;
				tr_req[tr_idx].icnt2 = tr1_cnt2;
				tr_req[tr_idx].icnt3 = 1;
				tr_req[tr_idx].dim1 = tr_cnt0;
				tr_req[tr_idx].dim2 = trigger_size;

				tr_req[tr_idx].daddr = dev_addr;
				tr_req[tr_idx].dicnt0 = tr_cnt0;
				tr_req[tr_idx].dicnt1 = tr_cnt1;
				tr_req[tr_idx].dicnt2 = tr1_cnt2;
				tr_req[tr_idx].dicnt3 = 1;
				tr_req[tr_idx].ddim1 = (-1) * tr_cnt0;
			}
			tr_idx++;
		}

		d->residue += sg_len;
	}

	cppi5_tr_csf_set(&tr_req[tr_idx - 1].flags, csf | CPPI5_TR_CSF_EOP);

	return d;
}

int udma_configure_statictr(struct udma_chan *uc, struct udma_desc *d,
				   enum dma_slave_buswidth dev_width,
				   u16 elcnt)
{
	if (uc->config.ep_type != PSIL_EP_PDMA_XY)
		return 0;

	/* Bus width translates to the element size (ES) */
	switch (dev_width) {
	case DMA_SLAVE_BUSWIDTH_1_BYTE:
		d->static_tr.elsize = 0;
		break;
	case DMA_SLAVE_BUSWIDTH_2_BYTES:
		d->static_tr.elsize = 1;
		break;
	case DMA_SLAVE_BUSWIDTH_3_BYTES:
		d->static_tr.elsize = 2;
		break;
	case DMA_SLAVE_BUSWIDTH_4_BYTES:
		d->static_tr.elsize = 3;
		break;
	case DMA_SLAVE_BUSWIDTH_8_BYTES:
		d->static_tr.elsize = 4;
		break;
	default: /* not reached */
		return -EINVAL;
	}

	d->static_tr.elcnt = elcnt;

	if (uc->config.pkt_mode || !uc->cyclic) {
		/*
		 * PDMA must close the packet when the channel is in packet mode.
		 * For TR mode when the channel is not cyclic we also need PDMA
		 * to close the packet otherwise the transfer will stall because
		 * PDMA holds on the data it has received from the peripheral.
		 */
		unsigned int div = dev_width * elcnt;

		if (uc->cyclic)
			d->static_tr.bstcnt = d->residue / d->sglen / div;
		else
			d->static_tr.bstcnt = d->residue / div;
	} else if ((uc->ud->match_data->type == DMA_TYPE_BCDMA ||
				uc->ud->match_data->type == DMA_TYPE_BCDMA_V2) &&
		   uc->config.dir == DMA_DEV_TO_MEM &&
		   uc->cyclic) {
		/*
		 * For cyclic mode with BCDMA we have to set EOP in each TR to
		 * prevent short packet errors seen on channel teardown. So the
		 * PDMA must close the packet after every TR transfer by setting
		 * burst count equal to the number of bytes transferred.
		 */
		struct cppi5_tr_type1_t *tr_req = d->hwdesc[0].tr_req_base;

		d->static_tr.bstcnt =
			(tr_req->icnt0 * tr_req->icnt1) / dev_width;
	} else {
		d->static_tr.bstcnt = 0;
	}

	if (uc->config.dir == DMA_DEV_TO_MEM &&
	    d->static_tr.bstcnt > uc->ud->match_data->statictr_z_mask)
		return -EINVAL;

	return 0;
}

struct udma_desc *
udma_prep_slave_sg_pkt(struct udma_chan *uc, struct scatterlist *sgl,
		       unsigned int sglen, enum dma_transfer_direction dir,
		       unsigned long tx_flags, void *context)
{
	struct scatterlist *sgent;
	struct cppi5_host_desc_t *h_desc = NULL;
	struct udma_desc *d;
	u32 ring_id;
	unsigned int i;
	u64 asel;

	d = kzalloc(struct_size(d, hwdesc, sglen), GFP_NOWAIT);
	if (!d)
		return NULL;

	d->sglen = sglen;
	d->hwdesc_count = sglen;

	if (dir == DMA_DEV_TO_MEM)
		ring_id = k3_ringacc_get_ring_id(uc->rflow->r_ring);
	else
		ring_id = k3_ringacc_get_ring_id(uc->tchan->tc_ring);

	if (uc->ud->match_data->type == DMA_TYPE_UDMA)
		asel = 0;
	else
		asel = (u64)uc->config.asel << K3_ADDRESS_ASEL_SHIFT;

	for_each_sg(sgl, sgent, sglen, i) {
		struct udma_hwdesc *hwdesc = &d->hwdesc[i];
		dma_addr_t sg_addr = sg_dma_address(sgent);
		struct cppi5_host_desc_t *desc;
		size_t sg_len = sg_dma_len(sgent);

		hwdesc->cppi5_desc_vaddr = dma_pool_zalloc(uc->hdesc_pool,
						GFP_NOWAIT,
						&hwdesc->cppi5_desc_paddr);
		if (!hwdesc->cppi5_desc_vaddr) {
			dev_err(uc->ud->dev,
				"descriptor%d allocation failed\n", i);

			udma_free_hwdesc(uc, d);
			kfree(d);
			return NULL;
		}

		d->residue += sg_len;
		hwdesc->cppi5_desc_size = uc->config.hdesc_size;
		desc = hwdesc->cppi5_desc_vaddr;

		if (i == 0) {
			cppi5_hdesc_init(desc, 0, 0);
			/* Flow and Packed ID */
			cppi5_desc_set_pktids(&desc->hdr, uc->id,
					      CPPI5_INFO1_DESC_FLOWID_DEFAULT);
			cppi5_desc_set_retpolicy(&desc->hdr, 0, ring_id);
		} else {
			cppi5_hdesc_reset_hbdesc(desc);
			cppi5_desc_set_retpolicy(&desc->hdr, 0, 0xffff);
		}

		/* attach the sg buffer to the descriptor */
		sg_addr |= asel;
		cppi5_hdesc_attach_buf(desc, sg_addr, sg_len, sg_addr, sg_len);

		/* Attach link as host buffer descriptor */
		if (h_desc)
			cppi5_hdesc_link_hbdesc(h_desc,
						hwdesc->cppi5_desc_paddr | asel);

		if (uc->ud->match_data->type == DMA_TYPE_PKTDMA ||
		    dir == DMA_MEM_TO_DEV)
			h_desc = desc;
	}

	if (d->residue >= SZ_4M) {
		dev_err(uc->ud->dev,
			"%s: Transfer size %u is over the supported 4M range\n",
			__func__, d->residue);
		udma_free_hwdesc(uc, d);
		kfree(d);
		return NULL;
	}

	h_desc = d->hwdesc[0].cppi5_desc_vaddr;
	cppi5_hdesc_set_pktlen(h_desc, d->residue);

	return d;
}

int udma_attach_metadata(struct dma_async_tx_descriptor *desc,
				void *data, size_t len)
{
	struct udma_desc *d = to_udma_desc(desc);
	struct udma_chan *uc = to_udma_chan(desc->chan);
	struct cppi5_host_desc_t *h_desc;
	u32 psd_size = len;
	u32 flags = 0;

	if (!uc->config.pkt_mode || !uc->config.metadata_size)
		return -EOPNOTSUPP;

	if (!data || len > uc->config.metadata_size)
		return -EINVAL;

	if (uc->config.needs_epib && len < CPPI5_INFO0_HDESC_EPIB_SIZE)
		return -EINVAL;

	h_desc = d->hwdesc[0].cppi5_desc_vaddr;
	if (d->dir == DMA_MEM_TO_DEV)
		memcpy(h_desc->epib, data, len);

	if (uc->config.needs_epib)
		psd_size -= CPPI5_INFO0_HDESC_EPIB_SIZE;

	d->metadata = data;
	d->metadata_size = len;
	if (uc->config.needs_epib)
		flags |= CPPI5_INFO0_HDESC_EPIB_PRESENT;

	cppi5_hdesc_update_flags(h_desc, flags);
	cppi5_hdesc_update_psdata_size(h_desc, psd_size);

	return 0;
}

void *udma_get_metadata_ptr(struct dma_async_tx_descriptor *desc,
				   size_t *payload_len, size_t *max_len)
{
	struct udma_desc *d = to_udma_desc(desc);
	struct udma_chan *uc = to_udma_chan(desc->chan);
	struct cppi5_host_desc_t *h_desc;

	if (!uc->config.pkt_mode || !uc->config.metadata_size)
		return ERR_PTR(-EOPNOTSUPP);

	h_desc = d->hwdesc[0].cppi5_desc_vaddr;

	*max_len = uc->config.metadata_size;

	*payload_len = cppi5_hdesc_epib_present(&h_desc->hdr) ?
		       CPPI5_INFO0_HDESC_EPIB_SIZE : 0;
	*payload_len += cppi5_hdesc_get_psdata_size(h_desc);

	return h_desc->epib;
}

int udma_set_metadata_len(struct dma_async_tx_descriptor *desc,
				 size_t payload_len)
{
	struct udma_desc *d = to_udma_desc(desc);
	struct udma_chan *uc = to_udma_chan(desc->chan);
	struct cppi5_host_desc_t *h_desc;
	u32 psd_size = payload_len;
	u32 flags = 0;

	if (!uc->config.pkt_mode || !uc->config.metadata_size)
		return -EOPNOTSUPP;

	if (payload_len > uc->config.metadata_size)
		return -EINVAL;

	if (uc->config.needs_epib && payload_len < CPPI5_INFO0_HDESC_EPIB_SIZE)
		return -EINVAL;

	h_desc = d->hwdesc[0].cppi5_desc_vaddr;

	if (uc->config.needs_epib) {
		psd_size -= CPPI5_INFO0_HDESC_EPIB_SIZE;
		flags |= CPPI5_INFO0_HDESC_EPIB_PRESENT;
	}

	cppi5_hdesc_update_flags(h_desc, flags);
	cppi5_hdesc_update_psdata_size(h_desc, psd_size);

	return 0;
}

struct dma_descriptor_metadata_ops metadata_ops = {
	.attach = udma_attach_metadata,
	.get_ptr = udma_get_metadata_ptr,
	.set_len = udma_set_metadata_len,
};

struct dma_async_tx_descriptor *
udma_prep_slave_sg(struct dma_chan *chan, struct scatterlist *sgl,
		   unsigned int sglen, enum dma_transfer_direction dir,
		   unsigned long tx_flags, void *context)
{
	struct udma_chan *uc = to_udma_chan(chan);
	enum dma_slave_buswidth dev_width;
	struct udma_desc *d;
	u32 burst;

	if (dir != uc->config.dir &&
	    (uc->config.dir == DMA_MEM_TO_MEM && !uc->config.tr_trigger_type)) {
		dev_err(chan->device->dev,
			"%s: chan%d is for %s, not supporting %s\n",
			__func__, uc->id,
			dmaengine_get_direction_text(uc->config.dir),
			dmaengine_get_direction_text(dir));
		return NULL;
	}

	if (dir == DMA_DEV_TO_MEM) {
		dev_width = uc->cfg.src_addr_width;
		burst = uc->cfg.src_maxburst;
	} else if (dir == DMA_MEM_TO_DEV) {
		dev_width = uc->cfg.dst_addr_width;
		burst = uc->cfg.dst_maxburst;
	} else {
		dev_err(chan->device->dev, "%s: bad direction?\n", __func__);
		return NULL;
	}

	if (!burst)
		burst = 1;

	uc->config.tx_flags = tx_flags;

	if (uc->config.pkt_mode)
		d = udma_prep_slave_sg_pkt(uc, sgl, sglen, dir, tx_flags,
					   context);
	else if (is_slave_direction(uc->config.dir))
		d = udma_prep_slave_sg_tr(uc, sgl, sglen, dir, tx_flags,
					  context);
	else
		d = udma_prep_slave_sg_triggered_tr(uc, sgl, sglen, dir,
						    tx_flags, context);

	if (!d)
		return NULL;

	d->dir = dir;
	d->desc_idx = 0;
	d->tr_idx = 0;

	/* static TR for remote PDMA */
	if (udma_configure_statictr(uc, d, dev_width, burst)) {
		dev_err(uc->ud->dev,
			"%s: StaticTR Z is limited to maximum %u (%u)\n",
			__func__, uc->ud->match_data->statictr_z_mask,
			d->static_tr.bstcnt);

		udma_free_hwdesc(uc, d);
		kfree(d);
		return NULL;
	}

	if (uc->config.metadata_size)
		d->vd.tx.metadata_ops = &metadata_ops;

	return vchan_tx_prep(&uc->vc, &d->vd, tx_flags);
}

struct udma_desc *
udma_prep_dma_cyclic_tr(struct udma_chan *uc, dma_addr_t buf_addr,
			size_t buf_len, size_t period_len,
			enum dma_transfer_direction dir, unsigned long flags)
{
	struct udma_desc *d;
	size_t tr_size, period_addr;
	struct cppi5_tr_type1_t *tr_req;
	unsigned int periods = buf_len / period_len;
	u16 tr0_cnt0, tr0_cnt1, tr1_cnt0;
	unsigned int i;
	int num_tr;
	u32 period_csf = 0;

	num_tr = udma_get_tr_counters(period_len, __ffs(buf_addr), &tr0_cnt0,
				      &tr0_cnt1, &tr1_cnt0);
	if (num_tr < 0) {
		dev_err(uc->ud->dev, "size %zu is not supported\n",
			period_len);
		return NULL;
	}

	/* Now allocate and setup the descriptor. */
	tr_size = sizeof(struct cppi5_tr_type1_t);
	d = udma_alloc_tr_desc(uc, tr_size, periods * num_tr, dir);
	if (!d)
		return NULL;

	tr_req = d->hwdesc[0].tr_req_base;
	if (uc->ud->match_data->type == DMA_TYPE_UDMA)
		period_addr = buf_addr;
	else
		period_addr = buf_addr |
			((u64)uc->config.asel << K3_ADDRESS_ASEL_SHIFT);

	/*
	 * For BCDMA <-> PDMA transfers, the EOP flag needs to be set on the
	 * last TR of a descriptor, to mark the packet as complete.
	 * This is required for getting the teardown completion message in case
	 * of TX, and to avoid short-packet error in case of RX.
	 *
	 * As we are in cyclic mode, we do not know which period might be the
	 * last one, so set the flag for each period.
	 */
	if (uc->config.ep_type == PSIL_EP_PDMA_XY &&
	    (uc->ud->match_data->type == DMA_TYPE_BCDMA ||
		 uc->ud->match_data->type == DMA_TYPE_BCDMA_V2)) {
		period_csf = CPPI5_TR_CSF_EOP;
	}

	for (i = 0; i < periods; i++) {
		int tr_idx = i * num_tr;

		cppi5_tr_init(&tr_req[tr_idx].flags, CPPI5_TR_TYPE1, false,
			      false, CPPI5_TR_EVENT_SIZE_COMPLETION, 0);

		tr_req[tr_idx].addr = period_addr;
		tr_req[tr_idx].icnt0 = tr0_cnt0;
		tr_req[tr_idx].icnt1 = tr0_cnt1;
		tr_req[tr_idx].dim1 = tr0_cnt0;

		if (num_tr == 2) {
			cppi5_tr_csf_set(&tr_req[tr_idx].flags,
					 CPPI5_TR_CSF_SUPR_EVT);
			tr_idx++;

			cppi5_tr_init(&tr_req[tr_idx].flags, CPPI5_TR_TYPE1,
				      false, false,
				      CPPI5_TR_EVENT_SIZE_COMPLETION, 0);

			tr_req[tr_idx].addr = period_addr + tr0_cnt1 * tr0_cnt0;
			tr_req[tr_idx].icnt0 = tr1_cnt0;
			tr_req[tr_idx].icnt1 = 1;
			tr_req[tr_idx].dim1 = tr1_cnt0;
		}

		if (!(flags & DMA_PREP_INTERRUPT))
			period_csf |= CPPI5_TR_CSF_SUPR_EVT;

		if (period_csf)
			cppi5_tr_csf_set(&tr_req[tr_idx].flags, period_csf);

		period_addr += period_len;
	}

	return d;
}

struct udma_desc *
udma_prep_dma_cyclic_pkt(struct udma_chan *uc, dma_addr_t buf_addr,
			 size_t buf_len, size_t period_len,
			 enum dma_transfer_direction dir, unsigned long flags)
{
	struct udma_desc *d;
	u32 ring_id;
	int i;
	int periods = buf_len / period_len;

	if (periods > (K3_UDMA_DEFAULT_RING_SIZE - 1))
		return NULL;

	if (period_len >= SZ_4M)
		return NULL;

	d = kzalloc(struct_size(d, hwdesc, periods), GFP_NOWAIT);
	if (!d)
		return NULL;

	d->hwdesc_count = periods;

	/* TODO: re-check this... */
	if (dir == DMA_DEV_TO_MEM)
		ring_id = k3_ringacc_get_ring_id(uc->rflow->r_ring);
	else
		ring_id = k3_ringacc_get_ring_id(uc->tchan->tc_ring);

	if (uc->ud->match_data->type != DMA_TYPE_UDMA)
		buf_addr |= (u64)uc->config.asel << K3_ADDRESS_ASEL_SHIFT;

	for (i = 0; i < periods; i++) {
		struct udma_hwdesc *hwdesc = &d->hwdesc[i];
		dma_addr_t period_addr = buf_addr + (period_len * i);
		struct cppi5_host_desc_t *h_desc;

		hwdesc->cppi5_desc_vaddr = dma_pool_zalloc(uc->hdesc_pool,
						GFP_NOWAIT,
						&hwdesc->cppi5_desc_paddr);
		if (!hwdesc->cppi5_desc_vaddr) {
			dev_err(uc->ud->dev,
				"descriptor%d allocation failed\n", i);

			udma_free_hwdesc(uc, d);
			kfree(d);
			return NULL;
		}

		hwdesc->cppi5_desc_size = uc->config.hdesc_size;
		h_desc = hwdesc->cppi5_desc_vaddr;

		cppi5_hdesc_init(h_desc, 0, 0);
		cppi5_hdesc_set_pktlen(h_desc, period_len);

		/* Flow and Packed ID */
		cppi5_desc_set_pktids(&h_desc->hdr, uc->id,
				      CPPI5_INFO1_DESC_FLOWID_DEFAULT);
		cppi5_desc_set_retpolicy(&h_desc->hdr, 0, ring_id);

		/* attach each period to a new descriptor */
		cppi5_hdesc_attach_buf(h_desc,
				       period_addr, period_len,
				       period_addr, period_len);
	}

	return d;
}

struct dma_async_tx_descriptor *
udma_prep_dma_cyclic(struct dma_chan *chan, dma_addr_t buf_addr, size_t buf_len,
		     size_t period_len, enum dma_transfer_direction dir,
		     unsigned long flags)
{
	struct udma_chan *uc = to_udma_chan(chan);
	enum dma_slave_buswidth dev_width;
	struct udma_desc *d;
	u32 burst;

	if (dir != uc->config.dir) {
		dev_err(chan->device->dev,
			"%s: chan%d is for %s, not supporting %s\n",
			__func__, uc->id,
			dmaengine_get_direction_text(uc->config.dir),
			dmaengine_get_direction_text(dir));
		return NULL;
	}

	uc->cyclic = true;

	if (dir == DMA_DEV_TO_MEM) {
		dev_width = uc->cfg.src_addr_width;
		burst = uc->cfg.src_maxburst;
	} else if (dir == DMA_MEM_TO_DEV) {
		dev_width = uc->cfg.dst_addr_width;
		burst = uc->cfg.dst_maxburst;
	} else {
		dev_err(uc->ud->dev, "%s: bad direction?\n", __func__);
		return NULL;
	}

	if (!burst)
		burst = 1;

	if (uc->config.pkt_mode)
		d = udma_prep_dma_cyclic_pkt(uc, buf_addr, buf_len, period_len,
					     dir, flags);
	else
		d = udma_prep_dma_cyclic_tr(uc, buf_addr, buf_len, period_len,
					    dir, flags);

	if (!d)
		return NULL;

	d->sglen = buf_len / period_len;

	d->dir = dir;
	d->residue = buf_len;

	/* static TR for remote PDMA */
	if (udma_configure_statictr(uc, d, dev_width, burst)) {
		dev_err(uc->ud->dev,
			"%s: StaticTR Z is limited to maximum %u (%u)\n",
			__func__, uc->ud->match_data->statictr_z_mask,
			d->static_tr.bstcnt);

		udma_free_hwdesc(uc, d);
		kfree(d);
		return NULL;
	}

	if (uc->config.metadata_size)
		d->vd.tx.metadata_ops = &metadata_ops;

	return vchan_tx_prep(&uc->vc, &d->vd, flags);
}

struct dma_async_tx_descriptor *
udma_prep_dma_memcpy(struct dma_chan *chan, dma_addr_t dest, dma_addr_t src,
		     size_t len, unsigned long tx_flags)
{
	struct udma_chan *uc = to_udma_chan(chan);
	struct udma_desc *d;
	struct cppi5_tr_type15_t *tr_req;
	int num_tr;
	size_t tr_size = sizeof(struct cppi5_tr_type15_t);
	u16 tr0_cnt0, tr0_cnt1, tr1_cnt0;
	u32 csf = CPPI5_TR_CSF_SUPR_EVT;

	if (uc->config.dir != DMA_MEM_TO_MEM) {
		dev_err(chan->device->dev,
			"%s: chan%d is for %s, not supporting %s\n",
			__func__, uc->id,
			dmaengine_get_direction_text(uc->config.dir),
			dmaengine_get_direction_text(DMA_MEM_TO_MEM));
		return NULL;
	}

	num_tr = udma_get_tr_counters(len, __ffs(src | dest), &tr0_cnt0,
				      &tr0_cnt1, &tr1_cnt0);
	if (num_tr < 0) {
		dev_err(uc->ud->dev, "size %zu is not supported\n",
			len);
		return NULL;
	}

	d = udma_alloc_tr_desc(uc, tr_size, num_tr, DMA_MEM_TO_MEM);
	if (!d)
		return NULL;

	d->dir = DMA_MEM_TO_MEM;
	d->desc_idx = 0;
	d->tr_idx = 0;
	d->residue = len;

	if (uc->ud->match_data->type != DMA_TYPE_UDMA) {
		src |= (u64)uc->ud->asel << K3_ADDRESS_ASEL_SHIFT;
		dest |= (u64)uc->ud->asel << K3_ADDRESS_ASEL_SHIFT;
	} else {
		csf |= CPPI5_TR_CSF_EOL_ICNT0;
	}

	tr_req = d->hwdesc[0].tr_req_base;

	cppi5_tr_init(&tr_req[0].flags, CPPI5_TR_TYPE15, false, true,
		      CPPI5_TR_EVENT_SIZE_COMPLETION, 0);
	cppi5_tr_csf_set(&tr_req[0].flags, csf);

	tr_req[0].addr = src;
	tr_req[0].icnt0 = tr0_cnt0;
	tr_req[0].icnt1 = tr0_cnt1;
	tr_req[0].icnt2 = 1;
	tr_req[0].icnt3 = 1;
	tr_req[0].dim1 = tr0_cnt0;

	tr_req[0].daddr = dest;
	tr_req[0].dicnt0 = tr0_cnt0;
	tr_req[0].dicnt1 = tr0_cnt1;
	tr_req[0].dicnt2 = 1;
	tr_req[0].dicnt3 = 1;
	tr_req[0].ddim1 = tr0_cnt0;

	if (num_tr == 2) {
		cppi5_tr_init(&tr_req[1].flags, CPPI5_TR_TYPE15, false, true,
			      CPPI5_TR_EVENT_SIZE_COMPLETION, 0);
		cppi5_tr_csf_set(&tr_req[1].flags, csf);

		tr_req[1].addr = src + tr0_cnt1 * tr0_cnt0;
		tr_req[1].icnt0 = tr1_cnt0;
		tr_req[1].icnt1 = 1;
		tr_req[1].icnt2 = 1;
		tr_req[1].icnt3 = 1;

		tr_req[1].daddr = dest + tr0_cnt1 * tr0_cnt0;
		tr_req[1].dicnt0 = tr1_cnt0;
		tr_req[1].dicnt1 = 1;
		tr_req[1].dicnt2 = 1;
		tr_req[1].dicnt3 = 1;
	}

	cppi5_tr_csf_set(&tr_req[num_tr - 1].flags, csf | CPPI5_TR_CSF_EOP);

	if (uc->config.metadata_size)
		d->vd.tx.metadata_ops = &metadata_ops;

	return vchan_tx_prep(&uc->vc, &d->vd, tx_flags);
}

void udma_issue_pending(struct dma_chan *chan)
{
	struct udma_chan *uc = to_udma_chan(chan);
	struct udma_dev *ud = to_udma_dev(chan->device);
	unsigned long flags;

	spin_lock_irqsave(&uc->vc.lock, flags);

	/* If we have something pending and no active descriptor, then */
	if (vchan_issue_pending(&uc->vc) && !uc->desc) {
		/*
		 * start a descriptor if the channel is NOT [marked as
		 * terminating _and_ it is still running (teardown has not
		 * completed yet)].
		 */
		if (!(uc->state == UDMA_CHAN_IS_TERMINATING &&
		      udma_is_chan_running(uc)))
			ud->udma_start(uc);
	}

	spin_unlock_irqrestore(&uc->vc.lock, flags);
}

int udma_terminate_all(struct dma_chan *chan)
{
	struct udma_chan *uc = to_udma_chan(chan);
	struct udma_dev *ud = to_udma_dev(chan->device);
	unsigned long flags;
	LIST_HEAD(head);

	spin_lock_irqsave(&uc->vc.lock, flags);

	if (udma_is_chan_running(uc))
		ud->udma_stop(uc);

	if (uc->desc) {
		uc->terminated_desc = uc->desc;
		uc->desc = NULL;
		uc->terminated_desc->terminated = true;
		cancel_delayed_work(&uc->tx_drain.work);
	}

	uc->paused = false;

	vchan_get_all_descriptors(&uc->vc, &head);
	spin_unlock_irqrestore(&uc->vc.lock, flags);
	vchan_dma_desc_free_list(&uc->vc, &head);

	return 0;
}

void udma_synchronize(struct dma_chan *chan)
{
	struct udma_chan *uc = to_udma_chan(chan);
	struct udma_dev *ud = to_udma_dev(chan->device);
	unsigned long timeout = msecs_to_jiffies(1000);

	vchan_synchronize(&uc->vc);

	if (uc->state == UDMA_CHAN_IS_TERMINATING) {
		timeout = wait_for_completion_timeout(&uc->teardown_completed,
						      timeout);
		if (!timeout) {
			dev_warn(uc->ud->dev, "chan%d teardown timeout!\n",
				 uc->id);
			udma_dump_chan_stdata(uc);
			ud->udma_reset_chan(uc, true);
		}
	}

	ud->udma_reset_chan(uc, false);
	if (udma_is_chan_running(uc))
		dev_warn(uc->ud->dev, "chan%d refused to stop!\n", uc->id);

	cancel_delayed_work_sync(&uc->tx_drain.work);
	udma_reset_rings(uc);
}

void udma_desc_pre_callback(struct virt_dma_chan *vc,
				   struct virt_dma_desc *vd,
				   struct dmaengine_result *result)
{
	struct udma_chan *uc = to_udma_chan(&vc->chan);
	struct udma_desc *d;
	u8 status;

	if (!vd)
		return;

	d = to_udma_desc(&vd->tx);

	if (d->metadata_size)
		udma_fetch_epib(uc, d);

	if (result) {
		void *desc_vaddr = udma_curr_cppi5_desc_vaddr(d, d->desc_idx);

		if (cppi5_desc_get_type(desc_vaddr) ==
		    CPPI5_INFO0_DESC_TYPE_VAL_HOST) {
			/* Provide residue information for the client */
			result->residue = d->residue -
					  cppi5_hdesc_get_pktlen(desc_vaddr);
			if (result->residue)
				result->result = DMA_TRANS_ABORTED;
			else
				result->result = DMA_TRANS_NOERROR;
		} else {
			result->residue = 0;
			/* Propagate TR Response errors to the client */
			status = d->hwdesc[0].tr_resp_base->status;
			if (status)
				result->result = DMA_TRANS_ABORTED;
			else
				result->result = DMA_TRANS_NOERROR;
		}
	}
}

/*
 * This tasklet handles the completion of a DMA descriptor by
 * calling its callback and freeing it.
 */
void udma_vchan_complete(struct tasklet_struct *t)
{
	struct virt_dma_chan *vc = from_tasklet(vc, t, task);
	struct virt_dma_desc *vd, *_vd;
	struct dmaengine_desc_callback cb;
	LIST_HEAD(head);

	spin_lock_irq(&vc->lock);
	list_splice_tail_init(&vc->desc_completed, &head);
	vd = vc->cyclic;
	if (vd) {
		vc->cyclic = NULL;
		dmaengine_desc_get_callback(&vd->tx, &cb);
	} else {
		memset(&cb, 0, sizeof(cb));
	}
	spin_unlock_irq(&vc->lock);

	udma_desc_pre_callback(vc, vd, NULL);
	dmaengine_desc_callback_invoke(&cb, NULL);

	list_for_each_entry_safe(vd, _vd, &head, node) {
		struct dmaengine_result result;

		dmaengine_desc_get_callback(&vd->tx, &cb);

		list_del(&vd->node);

		udma_desc_pre_callback(vc, vd, &result);
		dmaengine_desc_callback_invoke(&cb, &result);

		vchan_vdesc_fini(vd);
	}
}

void udma_free_chan_resources(struct dma_chan *chan)
{
	struct udma_chan *uc = to_udma_chan(chan);
	struct udma_dev *ud = to_udma_dev(chan->device);

	udma_terminate_all(chan);
	if (uc->terminated_desc) {
		ud->udma_reset_chan(uc, false);
		udma_reset_rings(uc);
	}

	cancel_delayed_work_sync(&uc->tx_drain.work);

	if (uc->irq_num_ring > 0) {
		free_irq(uc->irq_num_ring, uc);

		uc->irq_num_ring = 0;
	}
	if (uc->irq_num_udma > 0) {
		free_irq(uc->irq_num_udma, uc);

		uc->irq_num_udma = 0;
	}

	/* Release PSI-L pairing */
	if (uc->psil_paired) {
		if (ud->match_data->type < DMA_TYPE_BCDMA_V2 && IS_ENABLED(CONFIG_TI_K3_UDMA))
			navss_psil_unpair(ud, uc->config.src_thread,
					uc->config.dst_thread);
		uc->psil_paired = false;
	}

	vchan_free_chan_resources(&uc->vc);
	tasklet_kill(&uc->vc.task);

	bcdma_free_bchan_resources(uc);
	udma_free_tx_resources(uc);
	udma_free_rx_resources(uc);
	udma_reset_uchan(uc);

	if (uc->use_dma_pool) {
		dma_pool_destroy(uc->hdesc_pool);
		uc->use_dma_pool = false;
	}
}

int setup_resources(struct udma_dev *ud)
{
	struct device *dev = ud->dev;
	int ch_count, ret;

	switch (ud->match_data->type) {
	case DMA_TYPE_UDMA:
		ret = udma_setup_resources(ud);
		break;
	case DMA_TYPE_BCDMA:
	case DMA_TYPE_BCDMA_V2:
		ret = bcdma_setup_resources(ud);
		break;
	case DMA_TYPE_PKTDMA:
	case DMA_TYPE_PKTDMA_V2:
		ret = pktdma_setup_resources(ud);
		break;
	default:
		return -EINVAL;
	}

	if (ret)
		return ret;

	if (ud->match_data->type >= DMA_TYPE_BCDMA_V2) {
		ch_count = ud->bchan_cnt + ud->tchan_cnt;
		if (ud->bchan_cnt)
			ch_count -= bitmap_weight(ud->bchan_map, ud->bchan_cnt);
		ch_count -= bitmap_weight(ud->tchan_map, ud->tchan_cnt);
	} else {
		ch_count  = ud->bchan_cnt + ud->tchan_cnt + ud->rchan_cnt;
		if (ud->bchan_cnt)
			ch_count -= bitmap_weight(ud->bchan_map, ud->bchan_cnt);
		ch_count -= bitmap_weight(ud->tchan_map, ud->tchan_cnt);
		ch_count -= bitmap_weight(ud->rchan_map, ud->rchan_cnt);
	}
	if (!ch_count)
		return -ENODEV;

	ud->channels = devm_kcalloc(dev, ch_count, sizeof(*ud->channels),
				    GFP_KERNEL);
	if (!ud->channels)
		return -ENOMEM;

	switch (ud->match_data->type) {
	case DMA_TYPE_UDMA:
		dev_info(dev,
			 "Channels: %d (tchan: %u, rchan: %u, gp-rflow: %u)\n",
			 ch_count,
			 ud->tchan_cnt - bitmap_weight(ud->tchan_map,
						       ud->tchan_cnt),
			 ud->rchan_cnt - bitmap_weight(ud->rchan_map,
						       ud->rchan_cnt),
			 ud->rflow_cnt - bitmap_weight(ud->rflow_gp_map,
						       ud->rflow_cnt));
		break;
	case DMA_TYPE_BCDMA:
		dev_info(dev,
			 "Channels: %d (bchan: %u, tchan: %u, rchan: %u)\n",
			 ch_count,
			 ud->bchan_cnt - bitmap_weight(ud->bchan_map,
						       ud->bchan_cnt),
			 ud->tchan_cnt - bitmap_weight(ud->tchan_map,
						       ud->tchan_cnt),
			 ud->rchan_cnt - bitmap_weight(ud->rchan_map,
						       ud->rchan_cnt));
		break;
	case DMA_TYPE_BCDMA_V2:
		dev_info(dev,
			 "Channels: %d (bchan: %u, tchan + rchan: %u)\n",
			 ch_count,
			 ud->bchan_cnt - bitmap_weight(ud->bchan_map,
						       ud->bchan_cnt),
			 ud->chan_cnt - bitmap_weight(ud->chan_map,
						       ud->chan_cnt));
		break;
	case DMA_TYPE_PKTDMA:
		dev_info(dev,
			 "Channels: %d (tchan: %u, rchan: %u)\n",
			 ch_count,
			 ud->tchan_cnt - bitmap_weight(ud->tchan_map,
						       ud->tchan_cnt),
			 ud->rchan_cnt - bitmap_weight(ud->rchan_map,
						       ud->rchan_cnt));
		break;
	case DMA_TYPE_PKTDMA_V2:
		dev_info(dev,
			 "Channels: %d (tchan + rchan: %u)\n",
			 ch_count,
			 ud->chan_cnt - bitmap_weight(ud->chan_map,
						       ud->chan_cnt));
		break;
	default:
		break;
	}

	return ch_count;
}

void udma_mark_resource_ranges(struct udma_dev *ud, unsigned long *map,
				      struct ti_sci_resource_desc *rm_desc,
				      char *name)
{
	bitmap_clear(map, rm_desc->start, rm_desc->num);
	bitmap_clear(map, rm_desc->start_sec, rm_desc->num_sec);
	dev_dbg(ud->dev, "ti_sci resource range for %s: %d:%d | %d:%d\n", name,
		rm_desc->start, rm_desc->num, rm_desc->start_sec,
		rm_desc->num_sec);
}

int udma_setup_resources(struct udma_dev *ud)
{
	int ret, i, j;
	struct device *dev = ud->dev;
	struct ti_sci_resource *rm_res, irq_res;
	struct udma_tisci_rm *tisci_rm = &ud->tisci_rm;
	u32 cap3;

	/* Set up the throughput level start indexes */
	cap3 = udma_read(ud->mmrs[MMR_GCFG], 0x2c);
	if (of_device_is_compatible(dev->of_node,
				    "ti,am654-navss-main-udmap")) {
		ud->tchan_tpl.levels = 2;
		ud->tchan_tpl.start_idx[0] = 8;
	} else if (of_device_is_compatible(dev->of_node,
					   "ti,am654-navss-mcu-udmap")) {
		ud->tchan_tpl.levels = 2;
		ud->tchan_tpl.start_idx[0] = 2;
	} else if (UDMA_CAP3_UCHAN_CNT(cap3)) {
		ud->tchan_tpl.levels = 3;
		ud->tchan_tpl.start_idx[1] = UDMA_CAP3_UCHAN_CNT(cap3);
		ud->tchan_tpl.start_idx[0] = UDMA_CAP3_HCHAN_CNT(cap3);
	} else if (UDMA_CAP3_HCHAN_CNT(cap3)) {
		ud->tchan_tpl.levels = 2;
		ud->tchan_tpl.start_idx[0] = UDMA_CAP3_HCHAN_CNT(cap3);
	} else {
		ud->tchan_tpl.levels = 1;
	}

	ud->rchan_tpl.levels = ud->tchan_tpl.levels;
	ud->rchan_tpl.start_idx[0] = ud->tchan_tpl.start_idx[0];
	ud->rchan_tpl.start_idx[1] = ud->tchan_tpl.start_idx[1];

	ud->tchan_map = devm_kmalloc_array(dev, BITS_TO_LONGS(ud->tchan_cnt),
					   sizeof(unsigned long), GFP_KERNEL);
	ud->tchans = devm_kcalloc(dev, ud->tchan_cnt, sizeof(*ud->tchans),
				  GFP_KERNEL);
	ud->rchan_map = devm_kmalloc_array(dev, BITS_TO_LONGS(ud->rchan_cnt),
					   sizeof(unsigned long), GFP_KERNEL);
	ud->rchans = devm_kcalloc(dev, ud->rchan_cnt, sizeof(*ud->rchans),
				  GFP_KERNEL);
	ud->rflow_gp_map = devm_kmalloc_array(dev, BITS_TO_LONGS(ud->rflow_cnt),
					      sizeof(unsigned long),
					      GFP_KERNEL);
	ud->rflow_gp_map_allocated = devm_kcalloc(dev,
						  BITS_TO_LONGS(ud->rflow_cnt),
						  sizeof(unsigned long),
						  GFP_KERNEL);
	ud->rflow_in_use = devm_kcalloc(dev, BITS_TO_LONGS(ud->rflow_cnt),
					sizeof(unsigned long),
					GFP_KERNEL);
	ud->rflows = devm_kcalloc(dev, ud->rflow_cnt, sizeof(*ud->rflows),
				  GFP_KERNEL);

	if (!ud->tchan_map || !ud->rchan_map || !ud->rflow_gp_map ||
	    !ud->rflow_gp_map_allocated || !ud->tchans || !ud->rchans ||
	    !ud->rflows || !ud->rflow_in_use)
		return -ENOMEM;

	/*
	 * RX flows with the same Ids as RX channels are reserved to be used
	 * as default flows if remote HW can't generate flow_ids. Those
	 * RX flows can be requested only explicitly by id.
	 */
	bitmap_set(ud->rflow_gp_map_allocated, 0, ud->rchan_cnt);

	/* by default no GP rflows are assigned to Linux */
	bitmap_set(ud->rflow_gp_map, 0, ud->rflow_cnt);

	/* Get resource ranges from tisci */
	for (i = 0; i < RM_RANGE_LAST; i++) {
		if (i == RM_RANGE_BCHAN || i == RM_RANGE_TFLOW)
			continue;

		tisci_rm->rm_ranges[i] =
			devm_ti_sci_get_of_resource(tisci_rm->tisci, dev,
						    tisci_rm->tisci_dev_id,
						    (char *)range_names[i]);
	}

	/* tchan ranges */
	rm_res = tisci_rm->rm_ranges[RM_RANGE_TCHAN];
	if (IS_ERR(rm_res)) {
		bitmap_zero(ud->tchan_map, ud->tchan_cnt);
		irq_res.sets = 1;
	} else {
		bitmap_fill(ud->tchan_map, ud->tchan_cnt);
		for (i = 0; i < rm_res->sets; i++)
			udma_mark_resource_ranges(ud, ud->tchan_map,
						  &rm_res->desc[i], "tchan");
		irq_res.sets = rm_res->sets;
	}

	/* rchan and matching default flow ranges */
	rm_res = tisci_rm->rm_ranges[RM_RANGE_RCHAN];
	if (IS_ERR(rm_res)) {
		bitmap_zero(ud->rchan_map, ud->rchan_cnt);
		irq_res.sets++;
	} else {
		bitmap_fill(ud->rchan_map, ud->rchan_cnt);
		for (i = 0; i < rm_res->sets; i++)
			udma_mark_resource_ranges(ud, ud->rchan_map,
						  &rm_res->desc[i], "rchan");
		irq_res.sets += rm_res->sets;
	}

	irq_res.desc = kcalloc(irq_res.sets, sizeof(*irq_res.desc), GFP_KERNEL);
	if (!irq_res.desc)
		return -ENOMEM;
	rm_res = tisci_rm->rm_ranges[RM_RANGE_TCHAN];
	if (IS_ERR(rm_res)) {
		irq_res.desc[0].start = 0;
		irq_res.desc[0].num = ud->tchan_cnt;
		i = 1;
	} else {
		for (i = 0; i < rm_res->sets; i++) {
			irq_res.desc[i].start = rm_res->desc[i].start;
			irq_res.desc[i].num = rm_res->desc[i].num;
			irq_res.desc[i].start_sec = rm_res->desc[i].start_sec;
			irq_res.desc[i].num_sec = rm_res->desc[i].num_sec;
		}
	}
	rm_res = tisci_rm->rm_ranges[RM_RANGE_RCHAN];
	if (IS_ERR(rm_res)) {
		irq_res.desc[i].start = 0;
		irq_res.desc[i].num = ud->rchan_cnt;
	} else {
		for (j = 0; j < rm_res->sets; j++, i++) {
			if (rm_res->desc[j].num) {
				irq_res.desc[i].start = rm_res->desc[j].start +
						ud->soc_data->oes.udma_rchan;
				irq_res.desc[i].num = rm_res->desc[j].num;
			}
			if (rm_res->desc[j].num_sec) {
				irq_res.desc[i].start_sec = rm_res->desc[j].start_sec +
						ud->soc_data->oes.udma_rchan;
				irq_res.desc[i].num_sec = rm_res->desc[j].num_sec;
			}
		}
	}
	ret = ti_sci_inta_msi_domain_alloc_irqs(ud->dev, &irq_res);
	kfree(irq_res.desc);
	if (ret) {
		dev_err(ud->dev, "Failed to allocate MSI interrupts\n");
		return ret;
	}

	/* GP rflow ranges */
	rm_res = tisci_rm->rm_ranges[RM_RANGE_RFLOW];
	if (IS_ERR(rm_res)) {
		/* all gp flows are assigned exclusively to Linux */
		bitmap_clear(ud->rflow_gp_map, ud->rchan_cnt,
			     ud->rflow_cnt - ud->rchan_cnt);
	} else {
		for (i = 0; i < rm_res->sets; i++)
			udma_mark_resource_ranges(ud, ud->rflow_gp_map,
						  &rm_res->desc[i], "gp-rflow");
	}

	return 0;
}

int bcdma_setup_resources(struct udma_dev *ud)
{
	int ret, i, j;
	struct device *dev = ud->dev;
	struct ti_sci_resource *rm_res, irq_res;
	struct udma_tisci_rm *tisci_rm = &ud->tisci_rm;
	const struct udma_oes_offsets *oes = &ud->soc_data->oes;
	u32 cap;

	/* Set up the throughput level start indexes */
	cap = udma_read(ud->mmrs[MMR_GCFG], 0x2c);
	if (BCDMA_CAP3_UBCHAN_CNT(cap)) {
		ud->bchan_tpl.levels = 3;
		ud->bchan_tpl.start_idx[1] = BCDMA_CAP3_UBCHAN_CNT(cap);
		ud->bchan_tpl.start_idx[0] = BCDMA_CAP3_HBCHAN_CNT(cap);
	} else if (BCDMA_CAP3_HBCHAN_CNT(cap)) {
		ud->bchan_tpl.levels = 2;
		ud->bchan_tpl.start_idx[0] = BCDMA_CAP3_HBCHAN_CNT(cap);
	} else {
		ud->bchan_tpl.levels = 1;
	}

	cap = udma_read(ud->mmrs[MMR_GCFG], 0x30);
	if (BCDMA_CAP4_URCHAN_CNT(cap)) {
		ud->rchan_tpl.levels = 3;
		ud->rchan_tpl.start_idx[1] = BCDMA_CAP4_URCHAN_CNT(cap);
		ud->rchan_tpl.start_idx[0] = BCDMA_CAP4_HRCHAN_CNT(cap);
	} else if (BCDMA_CAP4_HRCHAN_CNT(cap)) {
		ud->rchan_tpl.levels = 2;
		ud->rchan_tpl.start_idx[0] = BCDMA_CAP4_HRCHAN_CNT(cap);
	} else {
		ud->rchan_tpl.levels = 1;
	}

	if (BCDMA_CAP4_UTCHAN_CNT(cap)) {
		ud->tchan_tpl.levels = 3;
		ud->tchan_tpl.start_idx[1] = BCDMA_CAP4_UTCHAN_CNT(cap);
		ud->tchan_tpl.start_idx[0] = BCDMA_CAP4_HTCHAN_CNT(cap);
	} else if (BCDMA_CAP4_HTCHAN_CNT(cap)) {
		ud->tchan_tpl.levels = 2;
		ud->tchan_tpl.start_idx[0] = BCDMA_CAP4_HTCHAN_CNT(cap);
	} else {
		ud->tchan_tpl.levels = 1;
	}

	ud->bchan_map = devm_kmalloc_array(dev, BITS_TO_LONGS(ud->bchan_cnt),
					   sizeof(unsigned long), GFP_KERNEL);
	bitmap_zero(ud->bchan_map, ud->bchan_cnt);
	ud->bchans = devm_kcalloc(dev, ud->bchan_cnt, sizeof(*ud->bchans),
				  GFP_KERNEL);
	ud->tchan_map = devm_kmalloc_array(dev, BITS_TO_LONGS(ud->tchan_cnt),
					   sizeof(unsigned long), GFP_KERNEL);
	bitmap_zero(ud->tchan_map, ud->tchan_cnt);
	ud->tchans = devm_kcalloc(dev, ud->tchan_cnt, sizeof(*ud->tchans),
				  GFP_KERNEL);
	if (ud->match_data->type == DMA_TYPE_BCDMA_V2) {
		ud->rchan_map = ud->tchan_map;
		ud->rchans = ud->tchans;
		ud->chan_map = ud->tchan_map;
		ud->chans = ud->tchans;
	} else {
		ud->rchan_map = devm_kmalloc_array(dev, BITS_TO_LONGS(ud->rchan_cnt),
				sizeof(unsigned long), GFP_KERNEL);
		bitmap_zero(ud->rchan_map, ud->rchan_cnt);
		ud->rchans = devm_kcalloc(dev, ud->rchan_cnt, sizeof(*ud->rchans),
				GFP_KERNEL);
	}
	/* BCDMA do not really have flows, but the driver expect it */
	ud->rflow_in_use = devm_kcalloc(dev, BITS_TO_LONGS(ud->rchan_cnt),
					sizeof(unsigned long),
					GFP_KERNEL);
	ud->rflows = devm_kcalloc(dev, ud->rchan_cnt, sizeof(*ud->rflows),
				  GFP_KERNEL);

	if (!ud->bchan_map || !ud->tchan_map || !ud->rchan_map ||
	    !ud->rflow_in_use || !ud->bchans || !ud->tchans || !ud->rchans ||
	    !ud->rflows)
		return -ENOMEM;

	if (ud->match_data->type == DMA_TYPE_BCDMA_V2)
		return 0;

	/* Get resource ranges from tisci */
	for (i = 0; i < RM_RANGE_LAST; i++) {
		if (i == RM_RANGE_RFLOW || i == RM_RANGE_TFLOW)
			continue;
		if (i == RM_RANGE_BCHAN && ud->bchan_cnt == 0)
			continue;
		if (i == RM_RANGE_TCHAN && ud->tchan_cnt == 0)
			continue;
		if (i == RM_RANGE_RCHAN && ud->rchan_cnt == 0)
			continue;

		tisci_rm->rm_ranges[i] =
			devm_ti_sci_get_of_resource(tisci_rm->tisci, dev,
						    tisci_rm->tisci_dev_id,
						    (char *)range_names[i]);
	}

	irq_res.sets = 0;

	/* bchan ranges */
	if (ud->bchan_cnt) {
		rm_res = tisci_rm->rm_ranges[RM_RANGE_BCHAN];
		if (IS_ERR(rm_res)) {
			bitmap_zero(ud->bchan_map, ud->bchan_cnt);
			irq_res.sets++;
		} else {
			bitmap_fill(ud->bchan_map, ud->bchan_cnt);
			for (i = 0; i < rm_res->sets; i++)
				udma_mark_resource_ranges(ud, ud->bchan_map,
							  &rm_res->desc[i],
							  "bchan");
			irq_res.sets += rm_res->sets;
		}
	}

	/* tchan ranges */
	if (ud->tchan_cnt) {
		rm_res = tisci_rm->rm_ranges[RM_RANGE_TCHAN];
		if (IS_ERR(rm_res)) {
			bitmap_zero(ud->tchan_map, ud->tchan_cnt);
			irq_res.sets += 2;
		} else {
			bitmap_fill(ud->tchan_map, ud->tchan_cnt);
			for (i = 0; i < rm_res->sets; i++)
				udma_mark_resource_ranges(ud, ud->tchan_map,
							  &rm_res->desc[i],
							  "tchan");
			irq_res.sets += rm_res->sets * 2;
		}
	}

	/* rchan ranges */
	if (ud->rchan_cnt) {
		rm_res = tisci_rm->rm_ranges[RM_RANGE_RCHAN];
		if (IS_ERR(rm_res)) {
			bitmap_zero(ud->rchan_map, ud->rchan_cnt);
			irq_res.sets += 2;
		} else {
			bitmap_fill(ud->rchan_map, ud->rchan_cnt);
			for (i = 0; i < rm_res->sets; i++)
				udma_mark_resource_ranges(ud, ud->rchan_map,
							  &rm_res->desc[i],
							  "rchan");
			irq_res.sets += rm_res->sets * 2;
		}
	}

	irq_res.desc = kcalloc(irq_res.sets, sizeof(*irq_res.desc), GFP_KERNEL);
	if (!irq_res.desc)
		return -ENOMEM;
	if (ud->bchan_cnt) {
		rm_res = tisci_rm->rm_ranges[RM_RANGE_BCHAN];
		if (IS_ERR(rm_res)) {
			irq_res.desc[0].start = oes->bcdma_bchan_ring;
			irq_res.desc[0].num = ud->bchan_cnt;
			i = 1;
		} else {
			for (i = 0; i < rm_res->sets; i++) {
				irq_res.desc[i].start = rm_res->desc[i].start +
							oes->bcdma_bchan_ring;
				irq_res.desc[i].num = rm_res->desc[i].num;

				if (rm_res->desc[i].num_sec) {
					irq_res.desc[i].start_sec = rm_res->desc[i].start_sec +
									oes->bcdma_bchan_ring;
					irq_res.desc[i].num_sec = rm_res->desc[i].num_sec;
				}
			}
		}
	} else {
		i = 0;
	}

	if (ud->tchan_cnt) {
		rm_res = tisci_rm->rm_ranges[RM_RANGE_TCHAN];
		if (IS_ERR(rm_res)) {
			irq_res.desc[i].start = oes->bcdma_tchan_data;
			irq_res.desc[i].num = ud->tchan_cnt;
			irq_res.desc[i + 1].start = oes->bcdma_tchan_ring;
			irq_res.desc[i + 1].num = ud->tchan_cnt;
			i += 2;
		} else {
			for (j = 0; j < rm_res->sets; j++, i += 2) {
				irq_res.desc[i].start = rm_res->desc[j].start +
							oes->bcdma_tchan_data;
				irq_res.desc[i].num = rm_res->desc[j].num;

				irq_res.desc[i + 1].start = rm_res->desc[j].start +
							oes->bcdma_tchan_ring;
				irq_res.desc[i + 1].num = rm_res->desc[j].num;

				if (rm_res->desc[j].num_sec) {
					irq_res.desc[i].start_sec = rm_res->desc[j].start_sec +
									oes->bcdma_tchan_data;
					irq_res.desc[i].num_sec = rm_res->desc[j].num_sec;
					irq_res.desc[i + 1].start_sec = rm_res->desc[j].start_sec +
									oes->bcdma_tchan_ring;
					irq_res.desc[i + 1].num_sec = rm_res->desc[j].num_sec;
				}
			}
		}
	}
	if (ud->rchan_cnt) {
		rm_res = tisci_rm->rm_ranges[RM_RANGE_RCHAN];
		if (IS_ERR(rm_res)) {
			irq_res.desc[i].start = oes->bcdma_rchan_data;
			irq_res.desc[i].num = ud->rchan_cnt;
			irq_res.desc[i + 1].start = oes->bcdma_rchan_ring;
			irq_res.desc[i + 1].num = ud->rchan_cnt;
			i += 2;
		} else {
			for (j = 0; j < rm_res->sets; j++, i += 2) {
				irq_res.desc[i].start = rm_res->desc[j].start +
							oes->bcdma_rchan_data;
				irq_res.desc[i].num = rm_res->desc[j].num;

				irq_res.desc[i + 1].start = rm_res->desc[j].start +
							oes->bcdma_rchan_ring;
				irq_res.desc[i + 1].num = rm_res->desc[j].num;

				if (rm_res->desc[j].num_sec) {
					irq_res.desc[i].start_sec = rm_res->desc[j].start_sec +
									oes->bcdma_rchan_data;
					irq_res.desc[i].num_sec = rm_res->desc[j].num_sec;
					irq_res.desc[i + 1].start_sec = rm_res->desc[j].start_sec +
									oes->bcdma_rchan_ring;
					irq_res.desc[i + 1].num_sec = rm_res->desc[j].num_sec;
				}
			}
		}
	}

	ret = ti_sci_inta_msi_domain_alloc_irqs(ud->dev, &irq_res);
	kfree(irq_res.desc);
	if (ret) {
		dev_err(ud->dev, "Failed to allocate MSI interrupts\n");
		return ret;
	}

	return 0;
}

int pktdma_setup_resources(struct udma_dev *ud)
{
	int ret, i, j;
	struct device *dev = ud->dev;
	struct ti_sci_resource *rm_res, irq_res;
	struct udma_tisci_rm *tisci_rm = &ud->tisci_rm;
	const struct udma_oes_offsets *oes = &ud->soc_data->oes;
	u32 cap3;

	/* Set up the throughput level start indexes */
	cap3 = udma_read(ud->mmrs[MMR_GCFG], 0x2c);
	if (UDMA_CAP3_UCHAN_CNT(cap3)) {
		ud->tchan_tpl.levels = 3;
		ud->tchan_tpl.start_idx[1] = UDMA_CAP3_UCHAN_CNT(cap3);
		ud->tchan_tpl.start_idx[0] = UDMA_CAP3_HCHAN_CNT(cap3);
	} else if (UDMA_CAP3_HCHAN_CNT(cap3)) {
		ud->tchan_tpl.levels = 2;
		ud->tchan_tpl.start_idx[0] = UDMA_CAP3_HCHAN_CNT(cap3);
	} else {
		ud->tchan_tpl.levels = 1;
	}

	ud->rchan_tpl.levels = ud->tchan_tpl.levels;
	ud->rchan_tpl.start_idx[0] = ud->tchan_tpl.start_idx[0];
	ud->rchan_tpl.start_idx[1] = ud->tchan_tpl.start_idx[1];

	ud->tchan_map = devm_kmalloc_array(dev, BITS_TO_LONGS(ud->tchan_cnt),
					   sizeof(unsigned long), GFP_KERNEL);
	bitmap_zero(ud->tchan_map, ud->tchan_cnt);
	ud->tchans = devm_kcalloc(dev, ud->tchan_cnt, sizeof(*ud->tchans),
				  GFP_KERNEL);
	if (ud->match_data->type == DMA_TYPE_PKTDMA_V2) {
		ud->rchan_map = ud->tchan_map;
		ud->rchans = ud->tchans;
		ud->chan_map = ud->tchan_map;
		ud->chans = ud->tchans;
	} else {
		ud->rchan_map = devm_kmalloc_array(dev, BITS_TO_LONGS(ud->rchan_cnt),
				sizeof(unsigned long), GFP_KERNEL);
		bitmap_zero(ud->rchan_map, ud->rchan_cnt);
		ud->rchans = devm_kcalloc(dev, ud->rchan_cnt, sizeof(*ud->rchans),
				GFP_KERNEL);
	}
	ud->rflow_in_use = devm_kcalloc(dev, BITS_TO_LONGS(ud->rflow_cnt),
					sizeof(unsigned long),
					GFP_KERNEL);
	ud->rflows = devm_kcalloc(dev, ud->rflow_cnt, sizeof(*ud->rflows),
				  GFP_KERNEL);
	ud->tflow_map = devm_kmalloc_array(dev, BITS_TO_LONGS(ud->tflow_cnt),
					   sizeof(unsigned long), GFP_KERNEL);
	bitmap_zero(ud->tflow_map, ud->tflow_cnt);

	if (!ud->tchan_map || !ud->rchan_map || !ud->tflow_map || !ud->tchans ||
	    !ud->rchans || !ud->rflows || !ud->rflow_in_use)
		return -ENOMEM;

	if (ud->match_data->type == DMA_TYPE_PKTDMA_V2)
		return 0;

	/* Get resource ranges from tisci */
	for (i = 0; i < RM_RANGE_LAST; i++) {
		if (i == RM_RANGE_BCHAN)
			continue;

		tisci_rm->rm_ranges[i] =
			devm_ti_sci_get_of_resource(tisci_rm->tisci, dev,
						    tisci_rm->tisci_dev_id,
						    (char *)range_names[i]);
	}

	/* tchan ranges */
	rm_res = tisci_rm->rm_ranges[RM_RANGE_TCHAN];
	if (IS_ERR(rm_res)) {
		bitmap_zero(ud->tchan_map, ud->tchan_cnt);
	} else {
		bitmap_fill(ud->tchan_map, ud->tchan_cnt);
		for (i = 0; i < rm_res->sets; i++)
			udma_mark_resource_ranges(ud, ud->tchan_map,
						  &rm_res->desc[i], "tchan");
	}

	/* rchan ranges */
	rm_res = tisci_rm->rm_ranges[RM_RANGE_RCHAN];
	if (IS_ERR(rm_res)) {
		bitmap_zero(ud->rchan_map, ud->rchan_cnt);
	} else {
		bitmap_fill(ud->rchan_map, ud->rchan_cnt);
		for (i = 0; i < rm_res->sets; i++)
			udma_mark_resource_ranges(ud, ud->rchan_map,
						  &rm_res->desc[i], "rchan");
	}

	/* rflow ranges */
	rm_res = tisci_rm->rm_ranges[RM_RANGE_RFLOW];
	if (IS_ERR(rm_res)) {
		/* all rflows are assigned exclusively to Linux */
		bitmap_zero(ud->rflow_in_use, ud->rflow_cnt);
		irq_res.sets = 1;
	} else {
		bitmap_fill(ud->rflow_in_use, ud->rflow_cnt);
		for (i = 0; i < rm_res->sets; i++)
			udma_mark_resource_ranges(ud, ud->rflow_in_use,
						  &rm_res->desc[i], "rflow");
		irq_res.sets = rm_res->sets;
	}

	/* tflow ranges */
	rm_res = tisci_rm->rm_ranges[RM_RANGE_TFLOW];
	if (IS_ERR(rm_res)) {
		/* all tflows are assigned exclusively to Linux */
		bitmap_zero(ud->tflow_map, ud->tflow_cnt);
		irq_res.sets++;
	} else {
		bitmap_fill(ud->tflow_map, ud->tflow_cnt);
		for (i = 0; i < rm_res->sets; i++)
			udma_mark_resource_ranges(ud, ud->tflow_map,
						  &rm_res->desc[i], "tflow");
		irq_res.sets += rm_res->sets;
	}

	irq_res.desc = kcalloc(irq_res.sets, sizeof(*irq_res.desc), GFP_KERNEL);
	if (!irq_res.desc)
		return -ENOMEM;
	rm_res = tisci_rm->rm_ranges[RM_RANGE_TFLOW];
	if (IS_ERR(rm_res)) {
		irq_res.desc[0].start = oes->pktdma_tchan_flow;
		irq_res.desc[0].num = ud->tflow_cnt;
		i = 1;
	} else {
		for (i = 0; i < rm_res->sets; i++) {
			irq_res.desc[i].start = rm_res->desc[i].start +
						oes->pktdma_tchan_flow;
			irq_res.desc[i].num = rm_res->desc[i].num;

			if (rm_res->desc[i].num_sec) {
				irq_res.desc[i].start_sec = rm_res->desc[i].start_sec +
								oes->pktdma_tchan_flow;
				irq_res.desc[i].num_sec = rm_res->desc[i].num_sec;
			}
		}
	}
	rm_res = tisci_rm->rm_ranges[RM_RANGE_RFLOW];
	if (IS_ERR(rm_res)) {
		irq_res.desc[i].start = oes->pktdma_rchan_flow;
		irq_res.desc[i].num = ud->rflow_cnt;
	} else {
		for (j = 0; j < rm_res->sets; j++, i++) {
			irq_res.desc[i].start = rm_res->desc[j].start +
						oes->pktdma_rchan_flow;
			irq_res.desc[i].num = rm_res->desc[j].num;

			if (rm_res->desc[j].num_sec) {
				irq_res.desc[i].start_sec = rm_res->desc[j].start_sec +
								oes->pktdma_rchan_flow;
				irq_res.desc[i].num_sec = rm_res->desc[j].num_sec;
			}
		}
	}
	ret = ti_sci_inta_msi_domain_alloc_irqs(ud->dev, &irq_res);
	kfree(irq_res.desc);
	if (ret) {
		dev_err(ud->dev, "Failed to allocate MSI interrupts\n");
		return ret;
	}

	return 0;
}

int udma_setup_rx_flush(struct udma_dev *ud)
{
	struct udma_rx_flush *rx_flush = &ud->rx_flush;
	struct cppi5_desc_hdr_t *tr_desc;
	struct cppi5_tr_type1_t *tr_req;
	struct cppi5_host_desc_t *desc;
	struct device *dev = ud->dev;
	struct udma_hwdesc *hwdesc;
	size_t tr_size;

	/* Allocate 1K buffer for discarded data on RX channel teardown */
	rx_flush->buffer_size = SZ_1K;
	rx_flush->buffer_vaddr = devm_kzalloc(dev, rx_flush->buffer_size,
					      GFP_KERNEL);
	if (!rx_flush->buffer_vaddr)
		return -ENOMEM;

	rx_flush->buffer_paddr = dma_map_single(dev, rx_flush->buffer_vaddr,
						rx_flush->buffer_size,
						DMA_TO_DEVICE);
	if (dma_mapping_error(dev, rx_flush->buffer_paddr))
		return -ENOMEM;

	/* Set up descriptor to be used for TR mode */
	hwdesc = &rx_flush->hwdescs[0];
	tr_size = sizeof(struct cppi5_tr_type1_t);
	hwdesc->cppi5_desc_size = cppi5_trdesc_calc_size(tr_size, 1);
	hwdesc->cppi5_desc_size = ALIGN(hwdesc->cppi5_desc_size,
					ud->desc_align);

	hwdesc->cppi5_desc_vaddr = devm_kzalloc(dev, hwdesc->cppi5_desc_size,
						GFP_KERNEL);
	if (!hwdesc->cppi5_desc_vaddr)
		return -ENOMEM;

	hwdesc->cppi5_desc_paddr = dma_map_single(dev, hwdesc->cppi5_desc_vaddr,
						  hwdesc->cppi5_desc_size,
						  DMA_TO_DEVICE);
	if (dma_mapping_error(dev, hwdesc->cppi5_desc_paddr))
		return -ENOMEM;

	/* Start of the TR req records */
	hwdesc->tr_req_base = hwdesc->cppi5_desc_vaddr + tr_size;
	/* Start address of the TR response array */
	hwdesc->tr_resp_base = hwdesc->tr_req_base + tr_size;

	tr_desc = hwdesc->cppi5_desc_vaddr;
	cppi5_trdesc_init(tr_desc, 1, tr_size, 0, 0);
	cppi5_desc_set_pktids(tr_desc, 0, CPPI5_INFO1_DESC_FLOWID_DEFAULT);
	cppi5_desc_set_retpolicy(tr_desc, 0, 0);

	tr_req = hwdesc->tr_req_base;
	cppi5_tr_init(&tr_req->flags, CPPI5_TR_TYPE1, false, false,
		      CPPI5_TR_EVENT_SIZE_COMPLETION, 0);
	cppi5_tr_csf_set(&tr_req->flags, CPPI5_TR_CSF_SUPR_EVT);

	tr_req->addr = rx_flush->buffer_paddr;
	tr_req->icnt0 = rx_flush->buffer_size;
	tr_req->icnt1 = 1;

	dma_sync_single_for_device(dev, hwdesc->cppi5_desc_paddr,
				   hwdesc->cppi5_desc_size, DMA_TO_DEVICE);

	/* Set up descriptor to be used for packet mode */
	hwdesc = &rx_flush->hwdescs[1];
	hwdesc->cppi5_desc_size = ALIGN(sizeof(struct cppi5_host_desc_t) +
					CPPI5_INFO0_HDESC_EPIB_SIZE +
					CPPI5_INFO0_HDESC_PSDATA_MAX_SIZE,
					ud->desc_align);

	hwdesc->cppi5_desc_vaddr = devm_kzalloc(dev, hwdesc->cppi5_desc_size,
						GFP_KERNEL);
	if (!hwdesc->cppi5_desc_vaddr)
		return -ENOMEM;

	hwdesc->cppi5_desc_paddr = dma_map_single(dev, hwdesc->cppi5_desc_vaddr,
						  hwdesc->cppi5_desc_size,
						  DMA_TO_DEVICE);
	if (dma_mapping_error(dev, hwdesc->cppi5_desc_paddr))
		return -ENOMEM;

	desc = hwdesc->cppi5_desc_vaddr;
	cppi5_hdesc_init(desc, 0, 0);
	cppi5_desc_set_pktids(&desc->hdr, 0, CPPI5_INFO1_DESC_FLOWID_DEFAULT);
	cppi5_desc_set_retpolicy(&desc->hdr, 0, 0);

	cppi5_hdesc_attach_buf(desc,
			       rx_flush->buffer_paddr, rx_flush->buffer_size,
			       rx_flush->buffer_paddr, rx_flush->buffer_size);

	dma_sync_single_for_device(dev, hwdesc->cppi5_desc_paddr,
				   hwdesc->cppi5_desc_size, DMA_TO_DEVICE);
	return 0;
}

#ifdef CONFIG_DEBUG_FS
void udma_dbg_summary_show_chan(struct seq_file *s,
				       struct dma_chan *chan)
{
	struct udma_chan *uc = to_udma_chan(chan);
	struct udma_chan_config *ucc = &uc->config;

	seq_printf(s, " %-13s| %s", dma_chan_name(chan),
		   chan->dbg_client_name ?: "in-use");
	if (ucc->tr_trigger_type)
		seq_puts(s, " (triggered, ");
	else
		seq_printf(s, " (%s, ",
			   dmaengine_get_direction_text(uc->config.dir));

	switch (uc->config.dir) {
	case DMA_MEM_TO_MEM:
		if (uc->ud->match_data->type == DMA_TYPE_BCDMA ||
			uc->ud->match_data->type == DMA_TYPE_BCDMA_V2) {
			seq_printf(s, "bchan%d)\n", uc->bchan->id);
			return;
		}

		seq_printf(s, "chan%d pair [0x%04x -> 0x%04x], ", uc->tchan->id,
			   ucc->src_thread, ucc->dst_thread);
		break;
	case DMA_DEV_TO_MEM:
		seq_printf(s, "rchan%d [0x%04x -> 0x%04x], ", uc->rchan->id,
			   ucc->src_thread, ucc->dst_thread);
		if (uc->ud->match_data->type == DMA_TYPE_PKTDMA)
			seq_printf(s, "rflow%d, ", uc->rflow->id);
		break;
	case DMA_MEM_TO_DEV:
		seq_printf(s, "tchan%d [0x%04x -> 0x%04x], ", uc->tchan->id,
			   ucc->src_thread, ucc->dst_thread);
		if (uc->ud->match_data->type == DMA_TYPE_PKTDMA)
			seq_printf(s, "tflow%d, ", uc->tchan->tflow_id);
		break;
	default:
		seq_puts(s, ")\n");
		return;
	}

	if (ucc->ep_type == PSIL_EP_NATIVE) {
		seq_puts(s, "PSI-L Native");
		if (ucc->metadata_size) {
			seq_printf(s, "[%s", ucc->needs_epib ? " EPIB" : "");
			if (ucc->psd_size)
				seq_printf(s, " PSDsize:%u", ucc->psd_size);
			seq_puts(s, " ]");
		}
	} else {
		seq_puts(s, "PDMA");
		if (ucc->enable_acc32 || ucc->enable_burst)
			seq_printf(s, "[%s%s ]",
				   ucc->enable_acc32 ? " ACC32" : "",
				   ucc->enable_burst ? " BURST" : "");
	}

	seq_printf(s, ", %s)\n", ucc->pkt_mode ? "Packet mode" : "TR mode");
}

void udma_dbg_summary_show(struct seq_file *s,
				  struct dma_device *dma_dev)
{
	struct dma_chan *chan;

	list_for_each_entry(chan, &dma_dev->channels, device_node) {
		if (chan->client_count)
			udma_dbg_summary_show_chan(s, chan);
	}
}
#endif /* CONFIG_DEBUG_FS */

enum dmaengine_alignment udma_get_copy_align(struct udma_dev *ud)
{
	const struct udma_match_data *match_data = ud->match_data;
	u8 tpl;

	if (!match_data->enable_memcpy_support)
		return DMAENGINE_ALIGN_8_BYTES;

	/* Get the highest TPL level the device supports for memcpy */
	if (ud->bchan_cnt)
		tpl = udma_get_chan_tpl_index(&ud->bchan_tpl, 0);
	else if (ud->tchan_cnt)
		tpl = udma_get_chan_tpl_index(&ud->tchan_tpl, 0);
	else
		return DMAENGINE_ALIGN_8_BYTES;

	switch (match_data->burst_size[tpl]) {
	case TI_SCI_RM_UDMAP_CHAN_BURST_SIZE_256_BYTES:
		return DMAENGINE_ALIGN_256_BYTES;
	case TI_SCI_RM_UDMAP_CHAN_BURST_SIZE_128_BYTES:
		return DMAENGINE_ALIGN_128_BYTES;
	case TI_SCI_RM_UDMAP_CHAN_BURST_SIZE_64_BYTES:
	fallthrough;
	default:
		return DMAENGINE_ALIGN_64_BYTES;
	}
}

/* Private interfaces to UDMA */
#include "k3-udma-private.c"
