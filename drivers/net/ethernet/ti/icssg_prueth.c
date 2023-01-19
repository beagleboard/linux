// SPDX-License-Identifier: GPL-2.0

/* Texas Instruments ICSSG Ethernet Driver
 *
 * Copyright (C) 2018-2021 Texas Instruments Incorporated - https://www.ti.com/
 *
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/etherdevice.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/genalloc.h>
#include <linux/if_vlan.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/of_platform.h>
#include <linux/phy.h>
#include <linux/pruss.h>
#include <linux/regmap.h>
#include <linux/remoteproc.h>
#include <linux/dma/ti-cppi5.h>

#include "icssg_prueth.h"
#include "icssg_switchdev.h"
#include "icss_mii_rt.h"
#include "k3-cppi-desc-pool.h"

#define PRUETH_MODULE_VERSION "0.1"
#define PRUETH_MODULE_DESCRIPTION "PRUSS ICSSG Ethernet driver"

/* MAX MTU set to match MII_G_RT_RX_STAT_MAX_SIZE_PRU0/1,
 * MII_G_RT_TX_STAT_MAX_SIZE_PORT0/1 defaults
 */
#define PRUETH_MAX_MTU		(2000 - ETH_HLEN - ETH_FCS_LEN)
#define PRUETH_MIN_PKT_SIZE	(VLAN_ETH_ZLEN)
#define PRUETH_MAX_PKT_SIZE	(PRUETH_MAX_MTU + ETH_HLEN + ETH_FCS_LEN)

/* Netif debug messages possible */
#define PRUETH_EMAC_DEBUG	(NETIF_MSG_DRV | \
				 NETIF_MSG_PROBE | \
				 NETIF_MSG_LINK | \
				 NETIF_MSG_TIMER | \
				 NETIF_MSG_IFDOWN | \
				 NETIF_MSG_IFUP | \
				 NETIF_MSG_RX_ERR | \
				 NETIF_MSG_TX_ERR | \
				 NETIF_MSG_TX_QUEUED | \
				 NETIF_MSG_INTR | \
				 NETIF_MSG_TX_DONE | \
				 NETIF_MSG_RX_STATUS | \
				 NETIF_MSG_PKTDATA | \
				 NETIF_MSG_HW | \
				 NETIF_MSG_WOL)

#define prueth_napi_to_emac(napi) container_of(napi, struct prueth_emac, napi)

/* CTRLMMR_ICSSG_RGMII_CTRL register bits */
#define ICSSG_CTRL_RGMII_ID_MODE		BIT(24)

#define IEP_DEFAULT_CYCLE_TIME_NS	1000000	/* 1 ms */

static int debug_level = -1;
module_param(debug_level, int, 0644);
MODULE_PARM_DESC(debug_level, "PRUETH debug level (NETIF_MSG bits)");

static void prueth_cleanup_rx_chns(struct prueth_emac *emac,
				   struct prueth_rx_chn *rx_chn,
				   int max_rflows)
{
	if (rx_chn->pg_pool) {
		page_pool_destroy(rx_chn->pg_pool);
		rx_chn->pg_pool = NULL;
	}

	if (rx_chn->desc_pool)
		k3_cppi_desc_pool_destroy(rx_chn->desc_pool);

	if (rx_chn->rx_chn)
		k3_udma_glue_release_rx_chn(rx_chn->rx_chn);
}

static void prueth_cleanup_tx_chns(struct prueth_emac *emac)
{
	int i;

	for (i = 0; i < emac->tx_ch_num; i++) {
		struct prueth_tx_chn *tx_chn = &emac->tx_chns[i];

		if (tx_chn->desc_pool)
			k3_cppi_desc_pool_destroy(tx_chn->desc_pool);

		if (tx_chn->tx_chn)
			k3_udma_glue_release_tx_chn(tx_chn->tx_chn);

		/* Assume prueth_cleanup_tx_chns() is called at the
		 * end after all channel resources are freed
		 */
		memset(tx_chn, 0, sizeof(*tx_chn));
	}
}

static void prueth_ndev_del_tx_napi(struct prueth_emac *emac, int num)
{
	int i;

	for (i = 0; i < num; i++) {
		struct prueth_tx_chn *tx_chn = &emac->tx_chns[i];

		if (tx_chn->irq)
			free_irq(tx_chn->irq, tx_chn);
		netif_napi_del(&tx_chn->napi_tx);
	}
}

static void prueth_xmit_free(struct prueth_tx_chn *tx_chn,
			     struct cppi5_host_desc_t *desc)
{
	struct cppi5_host_desc_t *first_desc, *next_desc;
	dma_addr_t buf_dma, next_desc_dma;
	struct prueth_swdata *swdata;
	u32 buf_dma_len;

	first_desc = desc;
	next_desc = first_desc;

	swdata = cppi5_hdesc_get_swdata(desc);
	if (swdata->type == PRUETH_SWDATA_PAGE) {
		page_pool_recycle_direct(swdata->rx_chn->pg_pool,
					 swdata->data.page);
		goto free_desc;
	}

	cppi5_hdesc_get_obuf(first_desc, &buf_dma, &buf_dma_len);
	k3_udma_glue_tx_cppi5_to_dma_addr(tx_chn->tx_chn, &buf_dma);

	dma_unmap_single(tx_chn->dma_dev, buf_dma, buf_dma_len,
			 DMA_TO_DEVICE);

	next_desc_dma = cppi5_hdesc_get_next_hbdesc(first_desc);
	k3_udma_glue_tx_cppi5_to_dma_addr(tx_chn->tx_chn, &next_desc_dma);
	while (next_desc_dma) {
		next_desc = k3_cppi_desc_pool_dma2virt(tx_chn->desc_pool,
						       next_desc_dma);
		cppi5_hdesc_get_obuf(next_desc, &buf_dma, &buf_dma_len);
		k3_udma_glue_tx_cppi5_to_dma_addr(tx_chn->tx_chn, &buf_dma);

		dma_unmap_page(tx_chn->dma_dev, buf_dma, buf_dma_len,
			       DMA_TO_DEVICE);

		next_desc_dma = cppi5_hdesc_get_next_hbdesc(next_desc);
		k3_udma_glue_tx_cppi5_to_dma_addr(tx_chn->tx_chn, &next_desc_dma);

		k3_cppi_desc_pool_free(tx_chn->desc_pool, next_desc);
	}

free_desc:
	k3_cppi_desc_pool_free(tx_chn->desc_pool, first_desc);
}

static int emac_tx_complete_packets(struct prueth_emac *emac, int chn,
				    int budget)
{
	struct net_device *ndev = emac->ndev;
	struct cppi5_host_desc_t *desc_tx;
	struct netdev_queue *netif_txq;
	struct prueth_tx_chn *tx_chn;
	unsigned int total_bytes = 0;
	struct prueth_swdata *swdata;
	struct xdp_frame *xdpf;
	struct sk_buff *skb;
	dma_addr_t desc_dma;
	int res, num_tx = 0;

	tx_chn = &emac->tx_chns[chn];

	while (budget--) {
		res = k3_udma_glue_pop_tx_chn(tx_chn->tx_chn, &desc_dma);
		if (res == -ENODATA)
			break;

		/* teardown completion */
		if (cppi5_desc_is_tdcm(desc_dma)) {
			if (atomic_dec_and_test(&emac->tdown_cnt))
				complete(&emac->tdown_complete);
			break;
		}

		desc_tx = k3_cppi_desc_pool_dma2virt(tx_chn->desc_pool,
						     desc_dma);
		swdata = cppi5_hdesc_get_swdata(desc_tx);

		/* was this command's TX complete? */
		if (emac->is_sr1 && swdata->type == PRUETH_SWDATA_CMD) {
			prueth_xmit_free(tx_chn, desc_tx);
			budget++;	/* not a data packet */
			continue;
		}

		switch (swdata->type) {
		case PRUETH_SWDATA_SKB:
			skb = swdata->data.skb;
			ndev->stats.tx_bytes += skb->len;
			ndev->stats.tx_packets++;
			total_bytes += skb->len;
			napi_consume_skb(skb, budget);
			break;
		case PRUETH_SWDATA_XDPF:
			xdpf = swdata->data.xdpf;
			ndev->stats.tx_bytes += xdpf->len;
			ndev->stats.tx_packets++;
			total_bytes += xdpf->len;
			xdp_return_frame(xdpf);
			break;
		default:
			netdev_err(ndev, "tx_complete: invalid swdata type %d\n", swdata->type);
			prueth_xmit_free(tx_chn, desc_tx);
			budget++;
			continue;
		}

		prueth_xmit_free(tx_chn, desc_tx);
		num_tx++;
	}

	if (!num_tx)
		return 0;

	netif_txq = netdev_get_tx_queue(ndev, chn);
	netdev_tx_completed_queue(netif_txq, num_tx, total_bytes);

	if (netif_tx_queue_stopped(netif_txq)) {
		/* If the TX queue was stopped, wake it now
		 * if we have enough room.
		 */
		__netif_tx_lock(netif_txq, smp_processor_id());
		if (netif_running(ndev) &&
		    (k3_cppi_desc_pool_avail(tx_chn->desc_pool) >=
		     MAX_SKB_FRAGS))
			netif_tx_wake_queue(netif_txq);
		__netif_tx_unlock(netif_txq);
	}

	return num_tx;
}

static int emac_napi_tx_poll(struct napi_struct *napi_tx, int budget)
{
	struct prueth_tx_chn *tx_chn = prueth_napi_to_tx_chn(napi_tx);
	struct prueth_emac *emac = tx_chn->emac;
	int num_tx_packets;

	num_tx_packets = emac_tx_complete_packets(emac, tx_chn->id, budget);

	if (num_tx_packets < budget) {
		napi_complete(napi_tx);
		enable_irq(tx_chn->irq);
	}

	return num_tx_packets;
}

static irqreturn_t prueth_tx_irq(int irq, void *dev_id)
{
	struct prueth_tx_chn *tx_chn = dev_id;

	disable_irq_nosync(irq);
	napi_schedule(&tx_chn->napi_tx);

	return IRQ_HANDLED;
}

static int prueth_ndev_add_tx_napi(struct prueth_emac *emac)
{
	struct prueth *prueth = emac->prueth;
	int i, ret;

	for (i = 0; i < emac->tx_ch_num; i++) {
		struct prueth_tx_chn *tx_chn = &emac->tx_chns[i];

		netif_tx_napi_add(emac->ndev, &tx_chn->napi_tx,
				  emac_napi_tx_poll, NAPI_POLL_WEIGHT);
		ret = request_irq(tx_chn->irq, prueth_tx_irq,
				  IRQF_TRIGGER_HIGH, tx_chn->name,
				  tx_chn);
		if (ret) {
			netif_napi_del(&tx_chn->napi_tx);
			dev_err(prueth->dev, "unable to request TX IRQ %d\n",
				tx_chn->irq);
			goto fail;
		}
	}

	return 0;
fail:
	prueth_ndev_del_tx_napi(emac, i);
	return ret;
}

static int prueth_init_tx_chns(struct prueth_emac *emac)
{
	struct net_device *ndev = emac->ndev;
	struct device *dev = emac->prueth->dev;
	struct k3_udma_glue_tx_channel_cfg tx_cfg;
	static const struct k3_ring_cfg ring_cfg = {
		.elm_size = K3_RINGACC_RING_ELSIZE_8,
		.mode = K3_RINGACC_RING_MODE_RING,
		.flags = 0,
		.size = PRUETH_MAX_TX_DESC,
	};
	int ret, slice, i;
	u32 hdesc_size;

	slice = prueth_emac_slice(emac);
	if (slice < 0)
		return slice;

	init_completion(&emac->tdown_complete);

	hdesc_size = cppi5_hdesc_calc_size(true, PRUETH_NAV_PS_DATA_SIZE,
					   PRUETH_NAV_SW_DATA_SIZE);
	memset(&tx_cfg, 0, sizeof(tx_cfg));
	tx_cfg.swdata_size = PRUETH_NAV_SW_DATA_SIZE;
	tx_cfg.tx_cfg = ring_cfg;
	tx_cfg.txcq_cfg = ring_cfg;

	for (i = 0; i < emac->tx_ch_num; i++) {
		struct prueth_tx_chn *tx_chn = &emac->tx_chns[i];

		/* To differentiate channels for SLICE0 vs SLICE1 */
		snprintf(tx_chn->name, sizeof(tx_chn->name),
			 "tx%d-%d", slice, i);

		tx_chn->emac = emac;
		tx_chn->id = i;
		tx_chn->descs_num = PRUETH_MAX_TX_DESC;

		tx_chn->tx_chn =
			k3_udma_glue_request_tx_chn(dev, tx_chn->name,
						    &tx_cfg);
		if (IS_ERR(tx_chn->tx_chn)) {
			ret = PTR_ERR(tx_chn->tx_chn);
			tx_chn->tx_chn = NULL;
			netdev_err(ndev,
				   "Failed to request tx dma ch: %d\n", ret);
			goto fail;
		}

		tx_chn->dma_dev = k3_udma_glue_tx_get_dma_device(tx_chn->tx_chn);
		tx_chn->desc_pool =
			k3_cppi_desc_pool_create_name(tx_chn->dma_dev,
						      tx_chn->descs_num,
						      hdesc_size,
						      tx_chn->name);
		if (IS_ERR(tx_chn->desc_pool)) {
			ret = PTR_ERR(tx_chn->desc_pool);
			tx_chn->desc_pool = NULL;
			netdev_err(ndev, "Failed to create tx pool: %d\n", ret);
			goto fail;
		}

		tx_chn->irq = k3_udma_glue_tx_get_irq(tx_chn->tx_chn);
		if (tx_chn->irq <= 0) {
			ret = -EINVAL;
			netdev_err(ndev, "failed to get tx irq\n");
			goto fail;
		}

		snprintf(tx_chn->name, sizeof(tx_chn->name), "%s-tx%d",
			 dev_name(dev), tx_chn->id);
	}

	return 0;

fail:
	prueth_cleanup_tx_chns(emac);
	return ret;
}

static int prueth_init_rx_chns(struct prueth_emac *emac,
			       struct prueth_rx_chn *rx_chn,
			       char *name, u32 max_rflows,
			       u32 max_desc_num)
{
	struct net_device *ndev = emac->ndev;
	struct device *dev = emac->prueth->dev;
	struct k3_udma_glue_rx_channel_cfg rx_cfg;
	u32 fdqring_id;
	u32 hdesc_size;
	int i, ret = 0, slice;

	slice = prueth_emac_slice(emac);
	if (slice < 0)
		return slice;

	/* To differentiate channels for SLICE0 vs SLICE1 */
	snprintf(rx_chn->name, sizeof(rx_chn->name), "%s%d", name, slice);

	hdesc_size = cppi5_hdesc_calc_size(true, PRUETH_NAV_PS_DATA_SIZE,
					   PRUETH_NAV_SW_DATA_SIZE);
	memset(&rx_cfg, 0, sizeof(rx_cfg));
	rx_cfg.swdata_size = PRUETH_NAV_SW_DATA_SIZE;
	rx_cfg.flow_id_num = max_rflows;
	rx_cfg.flow_id_base = -1; /* udmax will auto select flow id base */

	/* init all flows */
	rx_chn->dev = dev;
	rx_chn->descs_num = max_desc_num;

	rx_chn->rx_chn = k3_udma_glue_request_rx_chn(dev, rx_chn->name,
						     &rx_cfg);
	if (IS_ERR(rx_chn->rx_chn)) {
		ret = PTR_ERR(rx_chn->rx_chn);
		rx_chn->rx_chn = NULL;
		netdev_err(ndev, "Failed to request rx dma ch: %d\n", ret);
		goto fail;
	}

	rx_chn->dma_dev = k3_udma_glue_rx_get_dma_device(rx_chn->rx_chn);
	rx_chn->desc_pool = k3_cppi_desc_pool_create_name(rx_chn->dma_dev,
							  rx_chn->descs_num,
							  hdesc_size,
							  rx_chn->name);
	if (IS_ERR(rx_chn->desc_pool)) {
		ret = PTR_ERR(rx_chn->desc_pool);
		rx_chn->desc_pool = NULL;
		netdev_err(ndev, "Failed to create rx pool: %d\n", ret);
		goto fail;
	}

	if (!strncmp(name, "rxmgm", 5)) {
		emac->rx_mgm_flow_id_base = k3_udma_glue_rx_get_flow_id_base(rx_chn->rx_chn);
		netdev_dbg(ndev, "mgm flow id base = %d\n",
			   emac->rx_mgm_flow_id_base);
	} else {
		emac->rx_flow_id_base = k3_udma_glue_rx_get_flow_id_base(rx_chn->rx_chn);
		netdev_dbg(ndev, "flow id base = %d\n",
			   emac->rx_flow_id_base);
	}

	fdqring_id = K3_RINGACC_RING_ID_ANY;
	for (i = 0; i < rx_cfg.flow_id_num; i++) {
		struct k3_ring_cfg rxring_cfg = {
			.elm_size = K3_RINGACC_RING_ELSIZE_8,
			.mode = K3_RINGACC_RING_MODE_RING,
			.flags = 0,
		};
		struct k3_ring_cfg fdqring_cfg = {
			.elm_size = K3_RINGACC_RING_ELSIZE_8,
			.flags = K3_RINGACC_RING_SHARED,
		};
		struct k3_udma_glue_rx_flow_cfg rx_flow_cfg = {
			.rx_cfg = rxring_cfg,
			.rxfdq_cfg = fdqring_cfg,
			.ring_rxq_id = K3_RINGACC_RING_ID_ANY,
			.src_tag_lo_sel =
				K3_UDMA_GLUE_SRC_TAG_LO_USE_REMOTE_SRC_TAG,
		};

		rx_flow_cfg.ring_rxfdq0_id = fdqring_id;
		rx_flow_cfg.rx_cfg.size = max_desc_num;
		rx_flow_cfg.rxfdq_cfg.size = max_desc_num;
		rx_flow_cfg.rxfdq_cfg.mode = emac->prueth->pdata.fdqring_mode;

		ret = k3_udma_glue_rx_flow_init(rx_chn->rx_chn,
						i, &rx_flow_cfg);
		if (ret) {
			netdev_err(ndev, "Failed to init rx flow%d %d\n",
				   i, ret);
			goto fail;
		}
		if (!i)
			fdqring_id = k3_udma_glue_rx_flow_get_fdq_id(rx_chn->rx_chn,
								     i);
		rx_chn->irq[i] = k3_udma_glue_rx_get_irq(rx_chn->rx_chn, i);
		if (rx_chn->irq[i] <= 0) {
			netdev_err(ndev, "Failed to get rx dma irq");
			goto fail;
		}
	}

	return 0;

fail:
	prueth_cleanup_rx_chns(emac, rx_chn, max_rflows);
	return ret;
}

static int prueth_dma_rx_push_mapped(struct prueth_emac *emac,
				     struct prueth_rx_chn *rx_chn,
				     struct page *page, u32 buf_len)
{
	struct net_device *ndev = emac->ndev;
	struct cppi5_host_desc_t *desc_rx;
	struct prueth_swdata *swdata;
	dma_addr_t desc_dma;
	dma_addr_t buf_dma;

	buf_dma = page_pool_get_dma_addr(page) + PRUETH_HEADROOM;
	desc_rx = k3_cppi_desc_pool_alloc(rx_chn->desc_pool);
	if (!desc_rx) {
		netdev_err(ndev, "rx push: failed to allocate descriptor\n");
		return -ENOMEM;
	}
	desc_dma = k3_cppi_desc_pool_virt2dma(rx_chn->desc_pool, desc_rx);

	cppi5_hdesc_init(desc_rx, CPPI5_INFO0_HDESC_EPIB_PRESENT,
			 PRUETH_NAV_PS_DATA_SIZE);
	k3_udma_glue_rx_dma_to_cppi5_addr(rx_chn->rx_chn, &buf_dma);
	cppi5_hdesc_attach_buf(desc_rx, buf_dma, buf_len, buf_dma, buf_len);

	swdata = cppi5_hdesc_get_swdata(desc_rx);
	swdata->type = PRUETH_SWDATA_PAGE;
	swdata->data.page = page;
	swdata->rx_chn = rx_chn;

	return k3_udma_glue_push_rx_chn(rx_chn->rx_chn, 0,
					desc_rx, desc_dma);
}

static u64 icssg_ts_to_ns(u32 hi_sw, u32 hi, u32 lo, u32 cycle_time_ns)
{
	u32 iepcount_lo, iepcount_hi, hi_rollover_count;
	u64 ns;

	iepcount_lo = lo & GENMASK(19, 0);
	iepcount_hi = (hi & GENMASK(11, 0)) << 12 | lo >> 20;
	hi_rollover_count = hi >> 11;

	ns = ((u64)hi_rollover_count) << 23 | (iepcount_hi + hi_sw);
	ns = ns * cycle_time_ns + iepcount_lo;

	return ns;
}

static void emac_rx_timestamp(struct prueth_emac *emac,
			      struct sk_buff *skb, u32 *psdata)
{
	struct skb_shared_hwtstamps *ssh;
	u64 ns;

	if (emac->is_sr1) {
		ns = (u64)psdata[1] << 32 | psdata[0];
	} else {
		u32 hi_sw = readl(emac->prueth->shram.va +
				  TIMESYNC_FW_WC_COUNT_HI_SW_OFFSET_OFFSET);
		ns = icssg_ts_to_ns(hi_sw, psdata[1], psdata[0],
				    IEP_DEFAULT_CYCLE_TIME_NS);
	}

	ssh = skb_hwtstamps(skb);
	memset(ssh, 0, sizeof(*ssh));
	ssh->hwtstamp = ns_to_ktime(ns);
}

static unsigned int prueth_rxbuf_total_len(unsigned int len)
{
	len += PRUETH_HEADROOM;
	len += SKB_DATA_ALIGN(sizeof(struct skb_shared_info));

	return SKB_DATA_ALIGN(len);
}

static int emac_run_xdp(struct prueth_emac *emac, struct xdp_buff *xdp,
			struct page *page);

static int emac_rx_packet(struct prueth_emac *emac, u32 flow_id, int *xdp_state)
{
	struct prueth_rx_chn *rx_chn = &emac->rx_chns;
	u32 buf_dma_len, pkt_len, port_id = 0;
	struct net_device *ndev = emac->ndev;
	struct cppi5_host_desc_t *desc_rx;
	dma_addr_t desc_dma, buf_dma;
	struct prueth_swdata *swdata;
	struct page *page, *new_page;
	struct page_pool *pool;
	struct sk_buff *skb;
	struct xdp_buff xdp;
	u32 *psdata;
	void *pa;
	int ret;

	*xdp_state = 0;
	pool = rx_chn->pg_pool;
	ret = k3_udma_glue_pop_rx_chn(rx_chn->rx_chn, flow_id, &desc_dma);
	if (ret) {
		if (ret != -ENODATA)
			netdev_err(ndev, "rx pop: failed: %d\n", ret);
		return ret;
	}

	if (cppi5_desc_is_tdcm(desc_dma)) /* Teardown ? */
		return 0;

	desc_rx = k3_cppi_desc_pool_dma2virt(rx_chn->desc_pool, desc_dma);

	swdata = cppi5_hdesc_get_swdata(desc_rx);
	if (swdata->type != PRUETH_SWDATA_PAGE) {
		netdev_err(ndev, "rx_pkt: invliad swdata->type %d\n", swdata->type);
		return 0;
	}
	page = swdata->data.page;

	cppi5_hdesc_get_obuf(desc_rx, &buf_dma, &buf_dma_len);
	k3_udma_glue_rx_cppi5_to_dma_addr(rx_chn->rx_chn, &buf_dma);
	pkt_len = cppi5_hdesc_get_pktlen(desc_rx);
	/* firmware adds 4 CRC bytes, strip them */
	pkt_len -= 4;
	cppi5_desc_get_tags_ids(&desc_rx->hdr, &port_id, NULL);

	k3_cppi_desc_pool_free(rx_chn->desc_pool, desc_rx);

	if (!netif_running(ndev)) {
		page_pool_recycle_direct(pool, page);
		return 0;
	}

	/* if allocation fails we drop the packet but push the
	 * descriptor back to the ring with old page to prevent a stall
	 */
	new_page = page_pool_dev_alloc_pages(pool);
	if (unlikely(!new_page)) {
		new_page = page;
		ndev->stats.rx_dropped++;
		goto requeue;
	}

	pa = page_address(page);
	if (emac->xdp_prog) {
		/* xdp_init_buff(&xdp, PAGE_SIZE, rx_chn->xdp_rxq); */
		xdp.frame_sz = PAGE_SIZE;
		xdp.rxq = &rx_chn->xdp_rxq;

		/* xdp_prepare_buff(&xdp, pa, PRUETH_HEADROOM, pkt_len, false); */
		xdp.data_hard_start = pa;
		xdp.data = pa + PRUETH_HEADROOM;
		xdp.data_end = xdp.data + pkt_len;
		xdp.data_meta = xdp.data + 1;

		*xdp_state = emac_run_xdp(emac, &xdp, page);
		if (*xdp_state != ICSSG_XDP_PASS)
			goto requeue;
	}

	/* prepare skb and send to n/w stack */
	skb = build_skb(pa, prueth_rxbuf_total_len(pkt_len));
	if (!skb) {
		ndev->stats.rx_dropped++;
		page_pool_recycle_direct(pool, page);
		goto requeue;
	}

	skb_reserve(skb, PRUETH_HEADROOM);
	skb_put(skb, pkt_len);
	skb->dev = ndev;

	psdata = cppi5_hdesc_get_psdata(desc_rx);
	/* RX HW timestamp */
	if (emac->rx_ts_enabled)
		emac_rx_timestamp(emac, skb, psdata);

	if (emac->prueth->is_switch_mode)
		skb->offload_fwd_mark = emac->offload_fwd_mark;
	skb->protocol = eth_type_trans(skb, ndev);

	/* unmap page as no recycling of netstack skb page */
	page_pool_release_page(pool, page);
	netif_receive_skb(skb);
	ndev->stats.rx_bytes += pkt_len;
	ndev->stats.rx_packets++;

requeue:
	/* queue another RX DMA */
	ret = prueth_dma_rx_push_mapped(emac, &emac->rx_chns, new_page,
					PRUETH_MAX_PKT_SIZE);
	if (WARN_ON(ret < 0)) {
		page_pool_recycle_direct(pool, new_page);
		ndev->stats.rx_errors++;
		ndev->stats.rx_dropped++;
	}

	return ret;
}

static void prueth_rx_cleanup(void *data, dma_addr_t desc_dma)
{
	struct prueth_rx_chn *rx_chn = data;
	struct cppi5_host_desc_t *desc_rx;
	struct prueth_swdata *swdata;
	struct page_pool *pool;
	struct page *page;

	pool = rx_chn->pg_pool;

	desc_rx = k3_cppi_desc_pool_dma2virt(rx_chn->desc_pool, desc_dma);
	swdata = cppi5_hdesc_get_swdata(desc_rx);
	if (swdata->type == PRUETH_SWDATA_PAGE) {
		page = swdata->data.page;
		page_pool_recycle_direct(pool, page);
	}
	k3_cppi_desc_pool_free(rx_chn->desc_pool, desc_rx);
}

static int emac_get_tx_ts(struct prueth_emac *emac,
			  struct emac_tx_ts_response *rsp)
{
	struct prueth *prueth = emac->prueth;
	int slice = prueth_emac_slice(emac);
	int addr;

	addr = icssg_queue_pop(prueth, slice == 0 ?
			       ICSSG_TS_POP_SLICE0 : ICSSG_TS_POP_SLICE1);
	if (addr < 0)
		return addr;

	memcpy_fromio(rsp, prueth->shram.va + addr, sizeof(*rsp));
	/* return buffer back for to pool */
	icssg_queue_push(prueth, slice == 0 ?
			 ICSSG_TS_PUSH_SLICE0 : ICSSG_TS_PUSH_SLICE1, addr);

	return 0;
}

/* TODO: Convert this to use worker/workqueue mechanism to serialize the
 * request to firmware
 */
static int emac_send_command_sr1(struct prueth_emac *emac, u32 cmd)
{
	dma_addr_t desc_dma, buf_dma;
	struct prueth_tx_chn *tx_chn;
	struct cppi5_host_desc_t *first_desc;
	u32 pkt_len = sizeof(emac->cmd_data);
	struct prueth_swdata *swdata;
	u32 *data = emac->cmd_data;
	int ret = 0;
	u32 *epib;

	netdev_dbg(emac->ndev, "Sending cmd %x\n", cmd);

	/* only one command at a time allowed to firmware */
	mutex_lock(&emac->cmd_lock);
	data[0] = cpu_to_le32(cmd);

	/* highest priority channel for management messages */
	tx_chn = &emac->tx_chns[emac->tx_ch_num - 1];

	/* Map the linear buffer */
	buf_dma = dma_map_single(tx_chn->dma_dev, data, pkt_len, DMA_TO_DEVICE);
	if (dma_mapping_error(tx_chn->dma_dev, buf_dma)) {
		netdev_err(emac->ndev, "cmd %x: failed to map cmd buffer\n", cmd);
		ret = -EINVAL;
		goto err_unlock;
	}

	first_desc = k3_cppi_desc_pool_alloc(tx_chn->desc_pool);
	if (!first_desc) {
		netdev_err(emac->ndev, "cmd %x: failed to allocate descriptor\n", cmd);
		dma_unmap_single(tx_chn->dma_dev, buf_dma, pkt_len, DMA_TO_DEVICE);
		ret = -ENOMEM;
		goto err_unlock;
	}

	cppi5_hdesc_init(first_desc, CPPI5_INFO0_HDESC_EPIB_PRESENT,
			 PRUETH_NAV_PS_DATA_SIZE);
	cppi5_hdesc_set_pkttype(first_desc, PRUETH_PKT_TYPE_CMD);
	epib = first_desc->epib;
	epib[0] = 0;
	epib[1] = 0;

	cppi5_hdesc_attach_buf(first_desc, buf_dma, pkt_len, buf_dma, pkt_len);
	swdata = cppi5_hdesc_get_swdata(first_desc);
	swdata->type = PRUETH_SWDATA_CMD;
	swdata->data.cmd = cmd;

	cppi5_hdesc_set_pktlen(first_desc, pkt_len);
	desc_dma = k3_cppi_desc_pool_virt2dma(tx_chn->desc_pool, first_desc);

	/* send command */
	reinit_completion(&emac->cmd_complete);
	ret = k3_udma_glue_push_tx_chn(tx_chn->tx_chn, first_desc, desc_dma);
	if (ret) {
		netdev_err(emac->ndev, "cmd %x: push failed: %d\n", cmd, ret);
		goto free_desc;
	}
	ret = wait_for_completion_timeout(&emac->cmd_complete, msecs_to_jiffies(100));
	if (!ret)
		netdev_err(emac->ndev, "cmd %x: completion timeout\n", cmd);

	mutex_unlock(&emac->cmd_lock);

	return ret;
free_desc:
	prueth_xmit_free(tx_chn, first_desc);
err_unlock:
	mutex_unlock(&emac->cmd_lock);

	return ret;
}

static void emac_change_port_speed_duplex(struct prueth_emac *emac)
{
	u32 cmd = ICSSG_PSTATE_SPEED_DUPLEX_CMD, val;
	struct prueth *prueth = emac->prueth;
	int slice = prueth_emac_slice(emac);

	/* only full duplex supported for now */
	if (emac->duplex != DUPLEX_FULL)
		return;

	if (!emac->is_sr1)
		return;

	val = icssg_rgmii_get_speed(prueth->miig_rt, slice);
	/* firmware expects full duplex settings in bit 2-1 */
	val <<= 1;
	cmd |= val;

	val = icssg_rgmii_get_fullduplex(prueth->miig_rt, slice);
	/* firmware expects full duplex settings in bit 3 */
	val <<= 3;
	cmd |= val;

	emac_send_command_sr1(emac, cmd);
}

static int emac_shutdown(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);

	/* FIXME for SR2.0 */
	if (!emac->is_sr1)
		return 0;

	return emac_send_command_sr1(emac, ICSSG_SHUTDOWN_CMD);
}

static void tx_ts_work(struct prueth_emac *emac)
{
	u64 ns;
	struct skb_shared_hwtstamps ssh;
	struct sk_buff *skb;
	int ret = 0;
	struct emac_tx_ts_response tsr;
	u32 hi_sw;

	/* There may be more than one pending requests */
	while (1) {
		ret = emac_get_tx_ts(emac, &tsr);
		if (ret)	/* nothing more */
			break;

		if (tsr.cookie >= PRUETH_MAX_TX_TS_REQUESTS ||
		    !emac->tx_ts_skb[tsr.cookie]) {
			netdev_err(emac->ndev, "Invalid TX TS cookie 0x%x\n",
				   tsr.cookie);
			break;
		}

		skb = emac->tx_ts_skb[tsr.cookie];
		emac->tx_ts_skb[tsr.cookie] = NULL;	/* free slot */
		if (!skb) {
			netdev_err(emac->ndev, "Driver Bug! got NULL skb\n");
			break;
		}

		hi_sw = readl(emac->prueth->shram.va +
			      TIMESYNC_FW_WC_COUNT_HI_SW_OFFSET_OFFSET);
		ns = icssg_ts_to_ns(hi_sw, tsr.hi_ts, tsr.lo_ts,
				    IEP_DEFAULT_CYCLE_TIME_NS);

		memset(&ssh, 0, sizeof(ssh));
		ssh.hwtstamp = ns_to_ktime(ns);

		skb_tstamp_tx(skb, &ssh);
		dev_consume_skb_any(skb);

		if (atomic_dec_and_test(&emac->tx_ts_pending))	/* no more? */
			break;
	}

	return;
}

int prueth_tx_ts_cookie_get(struct prueth_emac *emac)
{
	int i;

	/* search and get the next free slot */
	for (i = 0; i < PRUETH_MAX_TX_TS_REQUESTS; i++) {
		if (!emac->tx_ts_skb[i]) {
			emac->tx_ts_skb[i] = ERR_PTR(-EBUSY); /* reserve slot */
			return i;
		}
	}

	return -EBUSY;
}

/**
 * emac_ndo_start_xmit - EMAC Transmit function
 * @skb: SKB pointer
 * @ndev: EMAC network adapter
 *
 * Called by the system to transmit a packet  - we queue the packet in
 * EMAC hardware transmit queue
 * Doesn't wait for completion we'll check for TX completion in
 * emac_tx_complete_packets().
 *
 * Returns success(NETDEV_TX_OK) or error code (typically out of descs)
 */
static int emac_ndo_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct cppi5_host_desc_t *first_desc, *next_desc, *cur_desc;
	struct prueth_emac *emac = netdev_priv(ndev);
	struct netdev_queue *netif_txq;
	struct prueth_tx_chn *tx_chn;
	dma_addr_t desc_dma, buf_dma;
	struct prueth_swdata *swdata;
	int i, ret = 0, q_idx;
	bool in_tx_ts = 0;
	int tx_ts_cookie;
	u32 pkt_len;
	u32 *epib;

	pkt_len = skb_headlen(skb);
	q_idx = skb_get_queue_mapping(skb);

	tx_chn = &emac->tx_chns[q_idx];
	netif_txq = netdev_get_tx_queue(ndev, q_idx);

	/* Map the linear buffer */
	buf_dma = dma_map_single(tx_chn->dma_dev, skb->data, pkt_len, DMA_TO_DEVICE);
	if (dma_mapping_error(tx_chn->dma_dev, buf_dma)) {
		netdev_err(ndev, "tx: failed to map skb buffer\n");
		ret = -EINVAL;
		goto drop_stop_q;
	}

	first_desc = k3_cppi_desc_pool_alloc(tx_chn->desc_pool);
	if (!first_desc) {
		netdev_dbg(ndev, "tx: failed to allocate descriptor\n");
		dma_unmap_single(tx_chn->dma_dev, buf_dma, pkt_len, DMA_TO_DEVICE);
		ret = -ENOMEM;
		goto drop_stop_q_busy;
	}

	cppi5_hdesc_init(first_desc, CPPI5_INFO0_HDESC_EPIB_PRESENT,
			 PRUETH_NAV_PS_DATA_SIZE);
	cppi5_hdesc_set_pkttype(first_desc, 0);
	epib = first_desc->epib;
	epib[0] = 0;
	epib[1] = 0;
	if (skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP &&
	    emac->tx_ts_enabled) {
		tx_ts_cookie = prueth_tx_ts_cookie_get(emac);
		if (tx_ts_cookie >= 0) {
			skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
			/* Request TX timestamp */
			epib[0] = (u32)tx_ts_cookie;
			epib[1] = 0x80000000;	/* TX TS request */
			emac->tx_ts_skb[tx_ts_cookie] = skb_get(skb);
			in_tx_ts = 1;
		}
	}

	/* set dst tag to indicate internal qid at the firmware which is at
	 * bit8..bit15. bit0..bit7 indicates port num for directed
	 * packets in case of switch mode operation
	 */
	cppi5_desc_set_tags_ids(&first_desc->hdr, 0, (emac->port_id | (q_idx << 8)));
	k3_udma_glue_tx_dma_to_cppi5_addr(tx_chn->tx_chn, &buf_dma);
	cppi5_hdesc_attach_buf(first_desc, buf_dma, pkt_len, buf_dma, pkt_len);
	swdata = cppi5_hdesc_get_swdata(first_desc);
	swdata->type = PRUETH_SWDATA_SKB;
	swdata->data.skb = skb;

	if (!skb_is_nonlinear(skb))
		goto tx_push;

	/* Handle the case where skb is fragmented in pages */
	cur_desc = first_desc;
	for (i = 0; i < skb_shinfo(skb)->nr_frags; i++) {
		skb_frag_t *frag = &skb_shinfo(skb)->frags[i];
		u32 frag_size = skb_frag_size(frag);

		next_desc = k3_cppi_desc_pool_alloc(tx_chn->desc_pool);
		if (!next_desc) {
			netdev_err(ndev,
				   "tx: failed to allocate frag. descriptor\n");
			ret = -ENOMEM;
			goto cleanup_tx_ts;
		}

		buf_dma = skb_frag_dma_map(tx_chn->dma_dev, frag, 0, frag_size,
					   DMA_TO_DEVICE);
		if (dma_mapping_error(tx_chn->dma_dev, buf_dma)) {
			netdev_err(ndev, "tx: Failed to map skb page\n");
			k3_cppi_desc_pool_free(tx_chn->desc_pool, next_desc);
			ret = -EINVAL;
			goto cleanup_tx_ts;
		}

		cppi5_hdesc_reset_hbdesc(next_desc);
		k3_udma_glue_tx_dma_to_cppi5_addr(tx_chn->tx_chn, &buf_dma);
		cppi5_hdesc_attach_buf(next_desc,
				       buf_dma, frag_size, buf_dma, frag_size);

		desc_dma = k3_cppi_desc_pool_virt2dma(tx_chn->desc_pool,
						      next_desc);
		k3_udma_glue_tx_dma_to_cppi5_addr(tx_chn->tx_chn, &desc_dma);
		cppi5_hdesc_link_hbdesc(cur_desc, desc_dma);

		pkt_len += frag_size;
		cur_desc = next_desc;
	}
	WARN_ON(pkt_len != skb->len);

tx_push:
	/* report bql before sending packet */
	netdev_tx_sent_queue(netif_txq, pkt_len);

	cppi5_hdesc_set_pktlen(first_desc, pkt_len);
	desc_dma = k3_cppi_desc_pool_virt2dma(tx_chn->desc_pool, first_desc);
	/* cppi5_desc_dump(first_desc, 64); */

	skb_tx_timestamp(skb);	/* SW timestamp if SKBTX_IN_PROGRESS not set */
	ret = k3_udma_glue_push_tx_chn(tx_chn->tx_chn, first_desc, desc_dma);
	if (ret) {
		netdev_err(ndev, "tx: push failed: %d\n", ret);
		goto drop_free_descs;
	}

	if (in_tx_ts)
		atomic_inc(&emac->tx_ts_pending);

	if (k3_cppi_desc_pool_avail(tx_chn->desc_pool) < MAX_SKB_FRAGS) {
		netif_tx_stop_queue(netif_txq);
		/* Barrier, so that stop_queue visible to other cpus */
		smp_mb__after_atomic();

		if (k3_cppi_desc_pool_avail(tx_chn->desc_pool) >=
		    MAX_SKB_FRAGS)
			netif_tx_wake_queue(netif_txq);
	}

	return NETDEV_TX_OK;

cleanup_tx_ts:
	if (in_tx_ts) {
		dev_kfree_skb_any(emac->tx_ts_skb[tx_ts_cookie]);
		emac->tx_ts_skb[tx_ts_cookie] = NULL;
	}

drop_free_descs:
	prueth_xmit_free(tx_chn, first_desc);
drop_stop_q:
	netif_tx_stop_queue(netif_txq);
	dev_kfree_skb_any(skb);

	/* error */
	ndev->stats.tx_dropped++;
	netdev_err(ndev, "tx: error: %d\n", ret);

	return ret;

drop_stop_q_busy:
	netif_tx_stop_queue(netif_txq);
	return NETDEV_TX_BUSY;
}

static void prueth_tx_cleanup(void *data, dma_addr_t desc_dma)
{
	struct prueth_tx_chn *tx_chn = data;
	struct cppi5_host_desc_t *desc_tx;
	struct prueth_swdata *swdata;
	struct xdp_frame *xdpf;
	struct sk_buff *skb;

	desc_tx = k3_cppi_desc_pool_dma2virt(tx_chn->desc_pool, desc_dma);
	swdata = cppi5_hdesc_get_swdata(desc_tx);

	switch (swdata->type) {
	case PRUETH_SWDATA_SKB:
		skb = swdata->data.skb;
		dev_kfree_skb_any(skb);
		break;
	case PRUETH_SWDATA_XDPF:
		xdpf = swdata->data.xdpf;
		xdp_return_frame(xdpf);
		break;
	default:
		break;
	}

	prueth_xmit_free(tx_chn, desc_tx);
}

static irqreturn_t prueth_tx_ts_irq(int irq, void *dev_id)
{
	struct prueth_emac *emac = dev_id;

	/* currently only TX timestamp is being returned */
	tx_ts_work(emac);

	return IRQ_HANDLED;
}

/* get one packet from requested flow_id
 *
 * Returns skb pointer if packet found else NULL
 * Caller must free the returned skb.
 */
static struct page *prueth_process_rx_mgm(struct prueth_emac *emac,
					  u32 flow_id)
{
	struct prueth_rx_chn *rx_chn = &emac->rx_mgm_chn;
	struct page_pool *pool = rx_chn->pg_pool;
	struct net_device *ndev = emac->ndev;
	struct cppi5_host_desc_t *desc_rx;
	struct page *page, *new_page;
	struct prueth_swdata *swdata;
	dma_addr_t desc_dma;
	int ret;

	ret = k3_udma_glue_pop_rx_chn(rx_chn->rx_chn, flow_id, &desc_dma);
	if (ret) {
		if (ret != -ENODATA)
			netdev_err(ndev, "rx mgm pop: failed: %d\n", ret);
		return NULL;
	}

	if (cppi5_desc_is_tdcm(desc_dma)) /* Teardown */
		return NULL;

	desc_rx = k3_cppi_desc_pool_dma2virt(rx_chn->desc_pool, desc_dma);

	/* Fix FW bug about incorrect PSDATA size */
	if (cppi5_hdesc_get_psdata_size(desc_rx) != PRUETH_NAV_PS_DATA_SIZE) {
		cppi5_hdesc_update_psdata_size(desc_rx,
					       PRUETH_NAV_PS_DATA_SIZE);
	}

	swdata = cppi5_hdesc_get_swdata(desc_rx);
	page = swdata->data.page;

	k3_cppi_desc_pool_free(rx_chn->desc_pool, desc_rx);

	/* if allocation fails we drop the packet but push the
	 * descriptor back to the ring with old page to prevent a stall
	 */
	new_page = page_pool_dev_alloc_pages(pool);
	if (unlikely(!new_page)) {
		netdev_err(ndev,
			   "page alloc failed, dropped mgm pkt from flow %d\n",
			   flow_id);
		new_page = page;
		page = NULL;
	}

	/* queue another DMA */
	ret = prueth_dma_rx_push_mapped(emac, &emac->rx_mgm_chn, new_page,
					PRUETH_MAX_PKT_SIZE);
	if (WARN_ON(ret < 0))
		page_pool_recycle_direct(pool, new_page);

	return page;
}

static void prueth_tx_ts_sr1(struct prueth_emac *emac,
			     struct emac_tx_ts_response_sr1 *tsr)
{
	u64 ns;
	struct skb_shared_hwtstamps ssh;
	struct sk_buff *skb;

	ns = (u64)tsr->hi_ts << 32 | tsr->lo_ts;

	if (tsr->cookie >= PRUETH_MAX_TX_TS_REQUESTS) {
		netdev_dbg(emac->ndev, "Invalid TX TS cookie 0x%x\n",
			   tsr->cookie);
		return;
	}

	skb = emac->tx_ts_skb[tsr->cookie];
	emac->tx_ts_skb[tsr->cookie] = NULL;	/* free slot */

	memset(&ssh, 0, sizeof(ssh));
	ssh.hwtstamp = ns_to_ktime(ns);

	skb_tstamp_tx(skb, &ssh);
	dev_consume_skb_any(skb);

	return;
}

static irqreturn_t prueth_rx_mgm_ts_thread_sr1(int irq, void *dev_id)
{
	struct prueth_emac *emac = dev_id;
	struct page *page;
	void *data;

	page = prueth_process_rx_mgm(emac, PRUETH_RX_MGM_FLOW_TIMESTAMP);
	if (!page)
		return IRQ_NONE;

	data = page_address(page) + PRUETH_HEADROOM;
	prueth_tx_ts_sr1(emac, (struct emac_tx_ts_response_sr1 *)data);
	page_pool_recycle_direct(emac->rx_mgm_chn.pg_pool, page);

	return IRQ_HANDLED;
}

static irqreturn_t prueth_rx_mgm_rsp_thread(int irq, void *dev_id)
{
	struct prueth_emac *emac = dev_id;
	struct page *page;
	void *data;
	u32 rsp;

	page = prueth_process_rx_mgm(emac, PRUETH_RX_MGM_FLOW_RESPONSE);
	if (!page)
		return IRQ_NONE;

	data = page_address(page) + PRUETH_HEADROOM;
	/* Process command response */
	rsp = le32_to_cpu(*(u32 *)data);
	if ((rsp & 0xffff0000) == ICSSG_SHUTDOWN_CMD) {
		netdev_dbg(emac->ndev,
			   "f/w Shutdown cmd resp %x\n", rsp);
		complete(&emac->cmd_complete);
	} else if ((rsp & 0xffff0000) ==
		ICSSG_PSTATE_SPEED_DUPLEX_CMD) {
		netdev_dbg(emac->ndev,
			   "f/w Speed/Duplex cmd rsp %x\n",
			    rsp);
		complete(&emac->cmd_complete);
	}

	page_pool_recycle_direct(emac->rx_mgm_chn.pg_pool, page);

	return IRQ_HANDLED;
}

static irqreturn_t prueth_rx_irq(int irq, void *dev_id)
{
	struct prueth_emac *emac = dev_id;

	disable_irq_nosync(irq);
	napi_schedule(&emac->napi_rx);

	return IRQ_HANDLED;
}

struct icssg_firmwares {
	char *pru;
	char *rtu;
	char *txpru;
};

static struct icssg_firmwares icssg_switch_firmwares[] = {
	{
		.pru = "ti-pruss/am65x-sr2-pru0-prusw-fw.elf",
		.rtu = "ti-pruss/am65x-sr2-rtu0-prusw-fw.elf",
		.txpru = "ti-pruss/am65x-sr2-txpru0-prusw-fw.elf",
	},
	{
		.pru = "ti-pruss/am65x-sr2-pru1-prusw-fw.elf",
		.rtu = "ti-pruss/am65x-sr2-rtu1-prusw-fw.elf",
		.txpru = "ti-pruss/am65x-sr2-txpru1-prusw-fw.elf",
	}
};

static struct icssg_firmwares icssg_emac_firmwares[] = {
	{
		.pru = "ti-pruss/am65x-sr2-pru0-prueth-fw.elf",
		.rtu = "ti-pruss/am65x-sr2-rtu0-prueth-fw.elf",
		.txpru = "ti-pruss/am65x-sr2-txpru0-prueth-fw.elf",
	},
	{
		.pru = "ti-pruss/am65x-sr2-pru1-prueth-fw.elf",
		.rtu = "ti-pruss/am65x-sr2-rtu1-prueth-fw.elf",
		.txpru = "ti-pruss/am65x-sr2-txpru1-prueth-fw.elf",
	}
};

static struct icssg_firmwares icssg_emac_firmwares_sr1[] = {
	{
		.pru = "ti-pruss/am65x-pru0-prueth-fw.elf",
		.rtu = "ti-pruss/am65x-rtu0-prueth-fw.elf",
	},
	{
		.pru = "ti-pruss/am65x-pru1-prueth-fw.elf",
		.rtu = "ti-pruss/am65x-rtu1-prueth-fw.elf",
	}
};

static int prueth_emac_start(struct prueth *prueth, struct prueth_emac *emac)
{
	struct icssg_firmwares *firmwares;
	struct device *dev = prueth->dev;
	int slice, ret;

	if (prueth->is_switch_mode)
		firmwares = icssg_switch_firmwares;
	else if (prueth->is_sr1)
		firmwares = icssg_emac_firmwares_sr1;
	else
		firmwares = icssg_emac_firmwares;

	slice = prueth_emac_slice(emac);
	if (slice < 0) {
		netdev_err(emac->ndev, "invalid port\n");
		return -EINVAL;
	}

	/* Set Load time configuration */
	if (emac->is_sr1) {
		icssg_config_sr1(prueth, emac, slice);
	} else {
		ret = icssg_config_sr2(prueth, emac, slice);
		if (ret)
			return ret;
	}

	ret = rproc_set_firmware(prueth->pru[slice], firmwares[slice].pru);
	ret = rproc_boot(prueth->pru[slice]);
	if (ret) {
		dev_err(dev, "failed to boot PRU%d: %d\n", slice, ret);
		return -EINVAL;
	}

	ret = rproc_set_firmware(prueth->rtu[slice], firmwares[slice].rtu);
	ret = rproc_boot(prueth->rtu[slice]);
	if (ret) {
		dev_err(dev, "failed to boot RTU%d: %d\n", slice, ret);
		goto halt_pru;
	}

	if (emac->is_sr1)
		goto done;

	ret = rproc_set_firmware(prueth->txpru[slice], firmwares[slice].txpru);
	ret = rproc_boot(prueth->txpru[slice]);
	if (ret) {
		dev_err(dev, "failed to boot TX_PRU%d: %d\n", slice, ret);
		goto halt_rtu;
	}

done:
	emac->fw_running = 1;
	return 0;

halt_rtu:
	rproc_shutdown(prueth->rtu[slice]);

halt_pru:
	rproc_shutdown(prueth->pru[slice]);

	return ret;
}

static void prueth_emac_stop(struct prueth_emac *emac)
{
	struct prueth *prueth = emac->prueth;
	int slice;

	switch (emac->port_id) {
	case PRUETH_PORT_MII0:
		slice = ICSS_SLICE0;
		break;
	case PRUETH_PORT_MII1:
		slice = ICSS_SLICE1;
		break;
	default:
		netdev_err(emac->ndev, "invalid port\n");
		return;
	}

	emac->fw_running = 0;
	if (!emac->is_sr1)
		rproc_shutdown(prueth->txpru[slice]);
	rproc_shutdown(prueth->rtu[slice]);
	rproc_shutdown(prueth->pru[slice]);
}

static void prueth_cleanup_tx_ts(struct prueth_emac *emac)
{
	int i;

	for (i = 0; i < PRUETH_MAX_TX_TS_REQUESTS; i++) {
		if (emac->tx_ts_skb[i]) {
			dev_kfree_skb_any(emac->tx_ts_skb[i]);
			emac->tx_ts_skb[i] = NULL;
		}
	}
}

/* called back by PHY layer if there is change in link state of hw port*/
static void emac_adjust_link(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct phy_device *phydev = emac->phydev;
	struct prueth *prueth = emac->prueth;
	bool new_state = false;
	unsigned long flags;

	if (phydev->link) {
		/* check the mode of operation - full/half duplex */
		if (phydev->duplex != emac->duplex) {
			new_state = true;
			emac->duplex = phydev->duplex;
		}
		if (phydev->speed != emac->speed) {
			new_state = true;
			emac->speed = phydev->speed;
		}
		if (!emac->link) {
			new_state = true;
			emac->link = 1;
		}
	} else if (emac->link) {
		new_state = true;
		emac->link = 0;
		/* defaults for no link */

		/* f/w should support 100 & 1000 */
		emac->speed = SPEED_1000;

		/* half duplex may not be supported by f/w */
		emac->duplex = DUPLEX_FULL;
	}

	if (new_state) {
		phy_print_status(phydev);

		/* update RGMII and MII configuration based on PHY negotiated
		 * values
		 */
		if (emac->link) {
			if (emac->duplex == DUPLEX_HALF)
				icssg_config_half_duplex(emac);
			/* Set the RGMII cfg for gig en and full duplex */
			icssg_update_rgmii_cfg(prueth->miig_rt, emac);

			/* update the Tx IPG based on 100M/1G speed */
			spin_lock_irqsave(&emac->lock, flags);
			icssg_config_ipg(emac);
			spin_unlock_irqrestore(&emac->lock, flags);
			icssg_config_set_speed(emac);
			if (!emac->is_sr1)
				emac_set_port_state(emac, ICSSG_EMAC_PORT_FORWARD);

		} else {
			if (!emac->is_sr1)
				emac_set_port_state(emac, ICSSG_EMAC_PORT_DISABLE);
		}

		/* send command to firmware to change speed and duplex
		 * setting when link is up.
		 */
		if (emac->link) {
			emac_change_port_speed_duplex(emac);
			icssg_qos_link_up(ndev);
		} else {
			icssg_qos_link_down(ndev);
		}
	}

	if (emac->link) {
		/* link ON */
		netif_carrier_on(ndev);
		/* reactivate the transmit queue */
		netif_tx_wake_all_queues(ndev);
	} else {
		/* link OFF */
		netif_carrier_off(ndev);
		netif_tx_stop_all_queues(ndev);
		prueth_cleanup_tx_ts(emac);
	}
}

static int emac_napi_rx_poll(struct napi_struct *napi_rx, int budget)
{
	struct prueth_emac *emac = prueth_napi_to_emac(napi_rx);
	int num_rx = 0;
	int flow = emac->is_sr1 ?
			PRUETH_MAX_RX_FLOWS_SR1 : PRUETH_MAX_RX_FLOWS_SR2;
	int rx_flow = emac->is_sr1 ?
			PRUETH_RX_FLOW_DATA_SR1 : PRUETH_RX_FLOW_DATA_SR2;
	int cur_budget;
	int ret;
	int xdp_state;
	int xdp_state_or = 0;

	while (flow--) {
		cur_budget = budget - num_rx;

		while (cur_budget--) {
			ret = emac_rx_packet(emac, flow, &xdp_state);
			xdp_state_or |= xdp_state;
			if (ret)
				break;
			num_rx++;
		}

		if (num_rx >= budget)
			break;
	}

	if (xdp_state_or & ICSSG_XDP_REDIR)
		xdp_do_flush();

	if (num_rx < budget) {
		napi_complete(napi_rx);
		enable_irq(emac->rx_chns.irq[rx_flow]);
	}

	return num_rx;
}

static struct page_pool *prueth_create_page_pool(struct prueth_emac *emac,
						 struct device *dma_dev,
						 int size)
{
	struct page_pool_params pp_params;
	struct page_pool *pool;

	pp_params.order = 0;
	pp_params.flags = PP_FLAG_DMA_MAP;
	pp_params.pool_size = size;
	pp_params.nid = NUMA_NO_NODE;
	pp_params.dma_dir = DMA_BIDIRECTIONAL;
	pp_params.dev = dma_dev;

	pool = page_pool_create(&pp_params);
	if (IS_ERR(pool))
		netdev_err(emac->ndev, "cannot create rx page pool\n");

	return pool;
}

static struct page *prueth_get_page_from_rx_chn(struct prueth_rx_chn *chn)
{
	struct cppi5_host_desc_t *desc_rx;
	struct prueth_swdata *swdata;
	dma_addr_t desc_dma;
	struct page *page;

	k3_udma_glue_pop_rx_chn(chn->rx_chn, 0, &desc_dma);
	desc_rx = k3_cppi_desc_pool_dma2virt(chn->desc_pool, desc_dma);
	swdata = cppi5_hdesc_get_swdata(desc_rx);
	page = swdata->data.page;

	return page;
}

static int prueth_prepare_rx_chan(struct prueth_emac *emac,
				  struct prueth_rx_chn *chn,
				  int buf_size)
{
	struct page_pool *pool;
	struct page *page;
	int i, ret, j;

	pool = prueth_create_page_pool(emac, chn->dma_dev, chn->descs_num);
	if (IS_ERR(pool))
		return PTR_ERR(pool);

	chn->pg_pool = pool;

	for (i = 0; i < chn->descs_num; i++) {
		/* NOTE: we're not using memory efficiently here.
		 * 1 full page (4KB?) used here instead of
		 * PRUETH_MAX_PKT_SIZE (~1.5KB?)
		 */
		page = page_pool_dev_alloc_pages(pool);
		if (!page) {
			netdev_err(emac->ndev, "couldn't allocate rx page\n");
			ret = -ENOMEM;
			goto recycle_alloc_pg;
		}

		ret = prueth_dma_rx_push_mapped(emac, chn, page, buf_size);
		if (ret < 0) {
			netdev_err(emac->ndev,
				   "cannot submit skb for rx chan %s ret %d\n",
				   chn->name, ret);
			page_pool_recycle_direct(pool, page);
			goto recycle_alloc_pg;
		}
	}

	return 0;

recycle_alloc_pg:
	for (j = 0; j < i; j++) {
		page = prueth_get_page_from_rx_chn(chn);
		page_pool_recycle_direct(pool, page);
	}
	page_pool_destroy(pool);
	chn->pg_pool = NULL;

	return ret;
}

static void prueth_reset_tx_chan(struct prueth_emac *emac, int ch_num,
				 bool free_skb)
{
	int i;

	for (i = 0; i < ch_num; i++) {
		if (free_skb)
			k3_udma_glue_reset_tx_chn(emac->tx_chns[i].tx_chn,
						  &emac->tx_chns[i],
						  prueth_tx_cleanup);
		k3_udma_glue_disable_tx_chn(emac->tx_chns[i].tx_chn);
	}
}

static void prueth_reset_rx_chan(struct prueth_rx_chn *chn,
				 int num_flows, bool disable)
{
	int i;

	for (i = 0; i < num_flows; i++)
		k3_udma_glue_reset_rx_chn(chn->rx_chn, i, chn,
					  prueth_rx_cleanup, !!i);
	if (disable)
		k3_udma_glue_disable_rx_chn(chn->rx_chn);
}

u64 prueth_iep_gettime(void *clockops_data, struct ptp_system_timestamp *sts)
{
	u32 hi_rollover_count, hi_rollover_count_r;
	struct prueth_emac *emac = clockops_data;
	struct prueth *prueth = emac->prueth;
	void __iomem *fw_hi_r_count_addr;
	void __iomem *fw_count_hi_addr;
	u32 iepcount_hi, iepcount_hi_r;
	unsigned long flags;
	u32 iepcount_lo;
	u64 ts = 0;

	fw_count_hi_addr = prueth->shram.va + TIMESYNC_FW_WC_COUNT_HI_SW_OFFSET_OFFSET;
	fw_hi_r_count_addr = prueth->shram.va + TIMESYNC_FW_WC_HI_ROLLOVER_COUNT_OFFSET;

	local_irq_save(flags);
	do {
		iepcount_hi = icss_iep_get_count_hi(emac->iep);
		iepcount_hi += readl(fw_count_hi_addr);
		hi_rollover_count = readl(fw_hi_r_count_addr);
		ptp_read_system_prets(sts);
		iepcount_lo = icss_iep_get_count_low(emac->iep);
		ptp_read_system_postts(sts);

		iepcount_hi_r = icss_iep_get_count_hi(emac->iep);
		iepcount_hi_r += readl(fw_count_hi_addr);
		hi_rollover_count_r = readl(fw_hi_r_count_addr);
	} while ((iepcount_hi_r != iepcount_hi) ||
		 (hi_rollover_count != hi_rollover_count_r));
	local_irq_restore(flags);

	ts = ((u64)hi_rollover_count) << 23 | iepcount_hi;
	ts = ts * (u64)IEP_DEFAULT_CYCLE_TIME_NS + iepcount_lo;

	return ts;
}

static void prueth_iep_settime(void *clockops_data, u64 ns)
{
	struct icssg_setclock_desc sc_desc, *sc_descp;
	struct prueth_emac *emac = clockops_data;
	u64 cyclecount;
	u32 cycletime;
	int timeout;

	if (!emac->fw_running)
		return;

	sc_descp = emac->prueth->shram.va + TIMESYNC_FW_WC_SETCLOCK_DESC_OFFSET;

	cycletime = IEP_DEFAULT_CYCLE_TIME_NS;
	cyclecount = ns / cycletime;

	memset(&sc_desc, 0, sizeof(sc_desc));
	sc_desc.margin = cycletime - 1000;
	sc_desc.cyclecounter0_set = cyclecount & GENMASK(31, 0);
	sc_desc.cyclecounter1_set = (cyclecount & GENMASK(63, 32)) >> 32;
	sc_desc.iepcount_set = ns % cycletime;
	sc_desc.CMP0_current = cycletime - 4; //Count from 0 to (cycle time)-4

	memcpy_toio(sc_descp, &sc_desc, sizeof(sc_desc));

	writeb(1, &sc_descp->request);

	timeout = 5;	/* fw should take 2-3 ms */
	while (timeout--) {
		if (readb(&sc_descp->acknowledgment))
			return;

		usleep_range(500, 1000);
	}

	dev_err(emac->prueth->dev, "settime timeout\n");
}

static int prueth_perout_enable(void *clockops_data,
				struct ptp_perout_request *req, int on,
				u64 *cmp)
{
	struct prueth_emac *emac = clockops_data;
	u32 reduction_factor = 0, offset = 0;
	struct timespec64 ts;
	u64 ns_period;

	if (!on)
		return 0;

	/* Any firmware specific stuff for PPS/PEROUT handling */
	ts.tv_sec = req->period.sec;
	ts.tv_nsec = req->period.nsec;
	ns_period = timespec64_to_ns(&ts);

	/* f/w doesn't support period less than cycle time */
	if (ns_period < IEP_DEFAULT_CYCLE_TIME_NS)
		return -ENXIO;

	reduction_factor = ns_period / IEP_DEFAULT_CYCLE_TIME_NS;
	offset = ns_period % IEP_DEFAULT_CYCLE_TIME_NS;

	/* f/w requires at least 1uS within a cycle so CMP
	 * can trigger after SYNC is enabled
	 */
	if (offset < 5 * NSEC_PER_USEC)
		offset = 5 * NSEC_PER_USEC;

	/* if offset is close to cycle time then we will miss
	 * the CMP event for last tick when IEP rolls over.
	 * In normal mode, IEP tick is 4ns.
	 * In slow compensation it could be 0ns or 8ns at
	 * every slow compensation cycle.
	 */
	if (offset > IEP_DEFAULT_CYCLE_TIME_NS - 8)
		offset = IEP_DEFAULT_CYCLE_TIME_NS - 8;

	/* we're in shadow mode so need to set upper 32-bits */
	*cmp = (u64)offset << 32;

	writel(reduction_factor, emac->prueth->shram.va +
		TIMESYNC_FW_WC_SYNCOUT_REDUCTION_FACTOR_OFFSET);

	/* HACK: till f/w supports START_TIME cyclcount we set it to 0 */
	writel(0, emac->prueth->shram.va +
		TIMESYNC_FW_WC_SYNCOUT_START_TIME_CYCLECOUNT_OFFSET);

	return 0;
}

const struct icss_iep_clockops prueth_iep_clockops = {
	.settime = prueth_iep_settime,
	.gettime = prueth_iep_gettime,
	/* FIXME: add adjtime to use relative mode */
	.perout_enable = prueth_perout_enable,
};

static int emac_phy_connect(struct prueth_emac *emac)
{
	struct prueth *prueth = emac->prueth;

	/* connect PHY */
	emac->phydev = of_phy_connect(emac->ndev, emac->phy_node,
				      &emac_adjust_link, 0, emac->phy_if);
	if (!emac->phydev) {
		dev_err(prueth->dev, "couldn't connect to phy %s\n",
			emac->phy_node->full_name);
		return -ENODEV;
	}

	/* remove unsupported modes */
	phy_remove_link_mode(emac->phydev, ETHTOOL_LINK_MODE_10baseT_Half_BIT);
	phy_remove_link_mode(emac->phydev, ETHTOOL_LINK_MODE_100baseT_Half_BIT);
	phy_remove_link_mode(emac->phydev, ETHTOOL_LINK_MODE_1000baseT_Half_BIT);
	phy_remove_link_mode(emac->phydev, ETHTOOL_LINK_MODE_Pause_BIT);
	phy_remove_link_mode(emac->phydev, ETHTOOL_LINK_MODE_Asym_Pause_BIT);

	if (emac->phy_if == PHY_INTERFACE_MODE_MII)
		phy_set_max_speed(emac->phydev, SPEED_100);

	return 0;
}

static int prueth_create_xdp_rxqs(struct prueth_emac *emac)
{
	struct xdp_rxq_info *rxq = &emac->rx_chns.xdp_rxq;
	struct page_pool *pool = emac->rx_chns.pg_pool;
	int ret;

	ret = xdp_rxq_info_reg(rxq, emac->ndev, 0);
	if (ret)
		return ret;

	ret = xdp_rxq_info_reg_mem_model(rxq, MEM_TYPE_PAGE_POOL, pool);
	if (ret)
		xdp_rxq_info_unreg(rxq);

	return ret;
}

static void prueth_destroy_xdp_rxqs(struct prueth_emac *emac)
{
	struct xdp_rxq_info *rxq = &emac->rx_chns.xdp_rxq;

	if (!xdp_rxq_info_is_reg(rxq))
		return;

	xdp_rxq_info_unreg(rxq);
}

/**
 * emac_ndo_open - EMAC device open
 * @ndev: network adapter device
 *
 * Called when system wants to start the interface.
 *
 * Returns 0 for a successful open, or appropriate error code
 */
static int emac_ndo_open(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	int ret, i, num_data_chn = emac->tx_ch_num;
	struct prueth *prueth = emac->prueth;
	int slice = prueth_emac_slice(emac);
	struct device *dev = prueth->dev;
	int max_rx_flows;
	int rx_flow;

	/* clear SMEM and MSMC settings for all slices */
	if (!prueth->emacs_initialized) {
		memset_io(prueth->msmcram.va, 0, prueth->msmcram.size);
		memset_io(prueth->shram.va, 0, ICSSG_CONFIG_OFFSET_SLICE1 * PRUETH_NUM_MACS);
	}

	if (emac->is_sr1) {
		/* For SR1, high priority channel is used exclusively for
		 * management messages. Do reduce number of data channels.
		 */
		num_data_chn--;
	}

	/* set h/w MAC as user might have re-configured */
	ether_addr_copy(emac->mac_addr, ndev->dev_addr);

	icssg_class_set_mac_addr(prueth->miig_rt, slice, emac->mac_addr);
	if (!emac->is_sr1)
		icssg_ft1_set_mac_addr(prueth->miig_rt, slice, emac->mac_addr);

	icssg_class_default(prueth->miig_rt, slice, 0, emac->is_sr1);

	netif_carrier_off(ndev);

	/* Notify the stack of the actual queue counts. */
	ret = netif_set_real_num_tx_queues(ndev, num_data_chn);
	if (ret) {
		dev_err(dev, "cannot set real number of tx queues\n");
		return ret;
	}

	init_completion(&emac->cmd_complete);
	ret = prueth_init_tx_chns(emac);
	if (ret) {
		dev_err(dev, "failed to init tx channel: %d\n", ret);
		return ret;
	}

	max_rx_flows = emac->is_sr1 ?
			PRUETH_MAX_RX_FLOWS_SR1 : PRUETH_MAX_RX_FLOWS_SR2;
	ret = prueth_init_rx_chns(emac, &emac->rx_chns, "rx",
				  max_rx_flows, PRUETH_MAX_RX_DESC);
	if (ret) {
		dev_err(dev, "failed to init rx channel: %d\n", ret);
		goto cleanup_tx;
	}

	if (emac->is_sr1) {
		ret = prueth_init_rx_chns(emac, &emac->rx_mgm_chn, "rxmgm",
					  PRUETH_MAX_RX_MGM_FLOWS,
					  PRUETH_MAX_RX_MGM_DESC);
		if (ret) {
			dev_err(dev, "failed to init rx mgmt channel: %d\n",
				ret);
			goto cleanup_rx;
		}
	}

	ret = prueth_ndev_add_tx_napi(emac);
	if (ret)
		goto cleanup_rx_mgm;

	/* we use only the highest priority flow for now i.e. @irq[3] */
	rx_flow = emac->is_sr1 ?
			PRUETH_RX_FLOW_DATA_SR1 : PRUETH_RX_FLOW_DATA_SR2;
	ret = request_irq(emac->rx_chns.irq[rx_flow], prueth_rx_irq,
			  IRQF_TRIGGER_HIGH, dev_name(dev), emac);
	if (ret) {
		dev_err(dev, "unable to request RX IRQ\n");
		goto cleanup_napi;
	}

	if (!emac->is_sr1)
		goto skip_mgm_irq;

	ret = request_threaded_irq(emac->rx_mgm_chn.irq[PRUETH_RX_MGM_FLOW_RESPONSE],
				   NULL, prueth_rx_mgm_rsp_thread,
				   IRQF_ONESHOT | IRQF_TRIGGER_HIGH,
				   dev_name(dev), emac);
	if (ret) {
		dev_err(dev, "unable to request RX Management RSP IRQ\n");
		goto free_rx_irq;
	}

	ret = request_threaded_irq(emac->rx_mgm_chn.irq[PRUETH_RX_MGM_FLOW_TIMESTAMP],
				   NULL, prueth_rx_mgm_ts_thread_sr1,
				   IRQF_ONESHOT | IRQF_TRIGGER_HIGH,
				   dev_name(dev), emac);
	if (ret) {
		dev_err(dev, "unable to request RX Management TS IRQ\n");
		goto free_rx_mgm_rsp_irq;
	}

skip_mgm_irq:
	/* reset and start PRU firmware */
	ret = prueth_emac_start(prueth, emac);
	if (ret)
		goto free_rx_mgmt_ts_irq;

	icssg_mii_update_mtu(prueth->mii_rt, slice, ndev->max_mtu);

	if (!emac->is_sr1 && !prueth->emacs_initialized) {
		ret = icss_iep_init(emac->iep, &prueth_iep_clockops,
				    emac, IEP_DEFAULT_CYCLE_TIME_NS);
	}

	if (!emac->is_sr1) {
		ret = request_threaded_irq(emac->tx_ts_irq, NULL, prueth_tx_ts_irq,
					   IRQF_ONESHOT, dev_name(dev), emac);
		if (ret)
			goto stop;
	}

	/* Prepare RX */
	ret = prueth_prepare_rx_chan(emac, &emac->rx_chns, PRUETH_MAX_PKT_SIZE);
	if (ret)
		goto destroy_xdp_rxqs;

	ret = prueth_create_xdp_rxqs(emac);
	if (ret)
		goto free_rx_ts_irq;

	if (emac->is_sr1) {
		ret = prueth_prepare_rx_chan(emac, &emac->rx_mgm_chn, PRUETH_MAX_PKT_SIZE);
		if (ret)
			goto reset_rx_chn;

		ret = k3_udma_glue_enable_rx_chn(emac->rx_mgm_chn.rx_chn);
		if (ret)
			goto reset_rx_chn;
	}

	ret = k3_udma_glue_enable_rx_chn(emac->rx_chns.rx_chn);
	if (ret)
		goto reset_rx_mgm_chn;

	for (i = 0; i < emac->tx_ch_num; i++) {
		ret = k3_udma_glue_enable_tx_chn(emac->tx_chns[i].tx_chn);
		if (ret)
			goto reset_tx_chan;
	}

	/* Enable NAPI in Tx and Rx direction */
	for (i = 0; i < emac->tx_ch_num; i++)
		napi_enable(&emac->tx_chns[i].napi_tx);
	napi_enable(&emac->napi_rx);

	icssg_qos_init(ndev);

	/* start PHY */
	phy_start(emac->phydev);

	prueth->emacs_initialized++;

	if (netif_msg_drv(emac))
		dev_notice(&ndev->dev, "started\n");

	if (prueth->is_switch_mode) {
		icssg_fdb_add_del(emac, eth_stp_addr, prueth->default_vlan,
				  ICSSG_FDB_ENTRY_P0_MEMBERSHIP |
				  ICSSG_FDB_ENTRY_P1_MEMBERSHIP |
				  ICSSG_FDB_ENTRY_P2_MEMBERSHIP |
				  ICSSG_FDB_ENTRY_BLOCK,
				  true);
		icssg_vtbl_modify(emac, emac->port_vlan, BIT(emac->port_id),
				  BIT(emac->port_id), true);
		icssg_set_pvid(emac->prueth, emac->port_vlan, emac->port_id);
		emac_set_port_state(emac, ICSSG_EMAC_PORT_VLAN_AWARE_ENABLE);
	}

	queue_work(system_long_wq, &emac->stats_work.work);

	return 0;

reset_tx_chan:
	/* Since interface is not yet up, there is wouldn't be
	 * any SKB for completion. So set false to free_skb
	 */
	prueth_reset_tx_chan(emac, i, false);
reset_rx_mgm_chn:
	if (emac->is_sr1)
		prueth_reset_rx_chan(&emac->rx_mgm_chn,
				     PRUETH_MAX_RX_MGM_FLOWS, true);
reset_rx_chn:
	prueth_reset_rx_chan(&emac->rx_chns, max_rx_flows, false);
destroy_xdp_rxqs:
	prueth_destroy_xdp_rxqs(emac);
free_rx_ts_irq:
	if (!emac->is_sr1)
		free_irq(emac->tx_ts_irq, emac);
stop:
	prueth_emac_stop(emac);
free_rx_mgmt_ts_irq:
	if (emac->is_sr1)
		free_irq(emac->rx_mgm_chn.irq[PRUETH_RX_MGM_FLOW_TIMESTAMP],
			 emac);
free_rx_mgm_rsp_irq:
	if (emac->is_sr1)
		free_irq(emac->rx_mgm_chn.irq[PRUETH_RX_MGM_FLOW_RESPONSE],
			 emac);
free_rx_irq:
	free_irq(emac->rx_chns.irq[rx_flow], emac);
cleanup_napi:
	prueth_ndev_del_tx_napi(emac, emac->tx_ch_num);
cleanup_rx_mgm:
	if (emac->is_sr1)
		prueth_cleanup_rx_chns(emac, &emac->rx_mgm_chn,
				       PRUETH_MAX_RX_MGM_FLOWS);
cleanup_rx:
	prueth_cleanup_rx_chns(emac, &emac->rx_chns, max_rx_flows);
cleanup_tx:
	prueth_cleanup_tx_chns(emac);

	return ret;
}

/**
 * emac_ndo_stop - EMAC device stop
 * @ndev: network adapter device
 *
 * Called when system wants to stop or down the interface.
 */
static int emac_ndo_stop(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth *prueth = emac->prueth;
	int ret, i;
	int max_rx_flows;
	int rx_flow = emac->is_sr1 ?
			PRUETH_RX_FLOW_DATA_SR1 : PRUETH_RX_FLOW_DATA_SR2;

	/* inform the upper layers. */
	netif_tx_stop_all_queues(ndev);

	/* block packets from wire */
	phy_stop(emac->phydev);

	icssg_class_disable(prueth->miig_rt, prueth_emac_slice(emac));

	/* send shutdown command */
	emac_shutdown(ndev);

	atomic_set(&emac->tdown_cnt, emac->tx_ch_num);
	/* ensure new tdown_cnt value is visible */
	smp_mb__after_atomic();
	/* tear down and disable UDMA channels */
	reinit_completion(&emac->tdown_complete);
	for (i = 0; i < emac->tx_ch_num; i++)
		k3_udma_glue_tdown_tx_chn(emac->tx_chns[i].tx_chn, false);

	ret = wait_for_completion_timeout(&emac->tdown_complete,
					  msecs_to_jiffies(1000));
	if (!ret)
		netdev_err(ndev, "tx teardown timeout\n");

	prueth_reset_tx_chan(emac, emac->tx_ch_num, true);
	for (i = 0; i < emac->tx_ch_num; i++)
		napi_disable(&emac->tx_chns[i].napi_tx);

	max_rx_flows = emac->is_sr1 ?
			PRUETH_MAX_RX_FLOWS_SR1 : PRUETH_MAX_RX_FLOWS_SR2;
	k3_udma_glue_tdown_rx_chn(emac->rx_chns.rx_chn, true);

	prueth_reset_rx_chan(&emac->rx_chns, max_rx_flows, true);
	if (emac->is_sr1) {
		/* Teardown RX MGM channel */
		k3_udma_glue_tdown_rx_chn(emac->rx_mgm_chn.rx_chn, true);
		prueth_reset_rx_chan(&emac->rx_mgm_chn,
				     PRUETH_MAX_RX_MGM_FLOWS, true);
	}

	prueth_destroy_xdp_rxqs(emac);

	napi_disable(&emac->napi_rx);

	if (!emac->is_sr1 && prueth->emacs_initialized == 1)
		icss_iep_exit(emac->iep);

	cancel_work_sync(&emac->rx_mode_work);

	/* Destroying the queued work in ndo_stop() */

	cancel_delayed_work_sync(&emac->stats_work);

	/* stop PRUs */
	prueth_emac_stop(emac);

	if (!emac->is_sr1)
		free_irq(emac->tx_ts_irq, emac);

	if (emac->is_sr1) {
		free_irq(emac->rx_mgm_chn.irq[PRUETH_RX_MGM_FLOW_TIMESTAMP],
			 emac);
		free_irq(emac->rx_mgm_chn.irq[PRUETH_RX_MGM_FLOW_RESPONSE],
			 emac);
	}
	free_irq(emac->rx_chns.irq[rx_flow], emac);
	prueth_ndev_del_tx_napi(emac, emac->tx_ch_num);
	prueth_cleanup_tx_chns(emac);

	if (emac->is_sr1)
		prueth_cleanup_rx_chns(emac, &emac->rx_mgm_chn,
				       PRUETH_MAX_RX_MGM_FLOWS);
	prueth_cleanup_rx_chns(emac, &emac->rx_chns, max_rx_flows);
	prueth_cleanup_tx_chns(emac);

	prueth->emacs_initialized--;

	if (netif_msg_drv(emac))
		dev_notice(&ndev->dev, "stopped\n");

	return 0;
}

static void emac_ndo_tx_timeout(struct net_device *ndev, unsigned int txqueue)
{
	struct prueth_emac *emac = netdev_priv(ndev);

	if (netif_msg_tx_err(emac))
		netdev_err(ndev, "xmit timeout");

	ndev->stats.tx_errors++;

	/* TODO: can we recover or need to reboot firmware? */
}

static void emac_ndo_set_rx_mode_sr1(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth *prueth = emac->prueth;
	int slice = prueth_emac_slice(emac);
	bool promisc = ndev->flags & IFF_PROMISC;
	bool allmulti = ndev->flags & IFF_ALLMULTI;

	if (promisc) {
		icssg_class_promiscuous_sr1(prueth->miig_rt, slice);
		return;
	}

	if (allmulti) {
		icssg_class_default(prueth->miig_rt, slice, 1, emac->is_sr1);
		return;
	}

	icssg_class_default(prueth->miig_rt, slice, 0, emac->is_sr1);
	if (!netdev_mc_empty(ndev)) {
		/* program multicast address list into Classifier */
		icssg_class_add_mcast_sr1(prueth->miig_rt, slice, ndev);
		return;
	}
}

static void emac_ndo_set_rx_mode_work(struct work_struct *work)
{
	struct prueth_emac *emac = container_of(work, struct prueth_emac, rx_mode_work);
	struct net_device *ndev = emac->ndev;
	bool promisc, allmulti;

	if (!(ndev->flags & IFF_UP))
		return;

	promisc = ndev->flags & IFF_PROMISC;
	allmulti = ndev->flags & IFF_ALLMULTI;
	emac_set_port_state(emac, ICSSG_EMAC_PORT_UC_FLOODING_DISABLE);
	emac_set_port_state(emac, ICSSG_EMAC_PORT_MC_FLOODING_DISABLE);

	if (promisc) {
		emac_set_port_state(emac, ICSSG_EMAC_PORT_UC_FLOODING_ENABLE);
		emac_set_port_state(emac, ICSSG_EMAC_PORT_MC_FLOODING_ENABLE);
		return;
	}

	if (allmulti) {
		emac_set_port_state(emac, ICSSG_EMAC_PORT_MC_FLOODING_ENABLE);
		return;
	}

	if (!netdev_mc_empty(ndev)) {
	/* TODO: Add FDB entries for multicast. till then enable allmulti */
		emac_set_port_state(emac, ICSSG_EMAC_PORT_MC_FLOODING_ENABLE);
		return;
	}
}

/**
 * emac_ndo_set_rx_mode - EMAC set receive mode function
 * @ndev: The EMAC network adapter
 *
 * Called when system wants to set the receive mode of the device.
 *
 */
static void emac_ndo_set_rx_mode(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth *prueth = emac->prueth;

	if (prueth->is_sr1) {
		emac_ndo_set_rx_mode_sr1(ndev);
		return;
	}

	queue_work(emac->cmd_wq, &emac->rx_mode_work);
}

static int emac_set_ts_config(struct net_device *ndev, struct ifreq *ifr)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct hwtstamp_config config;

	if (copy_from_user(&config, ifr->ifr_data, sizeof(config)))
		return -EFAULT;

	switch (config.tx_type) {
	case HWTSTAMP_TX_OFF:
		emac->tx_ts_enabled = 0;
		break;
	case HWTSTAMP_TX_ON:
		emac->tx_ts_enabled = 1;
		break;
	default:
		return -ERANGE;
	}

	switch (config.rx_filter) {
	case HWTSTAMP_FILTER_NONE:
		emac->rx_ts_enabled = 0;
		break;
	case HWTSTAMP_FILTER_ALL:
	case HWTSTAMP_FILTER_SOME:
	case HWTSTAMP_FILTER_PTP_V1_L4_EVENT:
	case HWTSTAMP_FILTER_PTP_V1_L4_SYNC:
	case HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ:
	case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ:
	case HWTSTAMP_FILTER_PTP_V2_L2_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L2_DELAY_REQ:
	case HWTSTAMP_FILTER_PTP_V2_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
	case HWTSTAMP_FILTER_NTP_ALL:
		emac->rx_ts_enabled = 1;
		config.rx_filter = HWTSTAMP_FILTER_ALL;
		break;
	default:
		return -ERANGE;
	}

	return copy_to_user(ifr->ifr_data, &config, sizeof(config)) ?
		-EFAULT : 0;
}

static int emac_get_ts_config(struct net_device *ndev, struct ifreq *ifr)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct hwtstamp_config config;

	config.flags = 0;
	config.tx_type = emac->tx_ts_enabled ? HWTSTAMP_TX_ON : HWTSTAMP_TX_OFF;
	config.rx_filter = emac->rx_ts_enabled ? HWTSTAMP_FILTER_ALL : HWTSTAMP_FILTER_NONE;

	return copy_to_user(ifr->ifr_data, &config, sizeof(config)) ?
			    -EFAULT : 0;
}

static int emac_ndo_ioctl(struct net_device *ndev, struct ifreq *ifr, int cmd)
{
	struct prueth_emac *emac = netdev_priv(ndev);

	switch (cmd) {
	case SIOCGHWTSTAMP:
		return emac_get_ts_config(ndev, ifr);
	case SIOCSHWTSTAMP:
		return emac_set_ts_config(ndev, ifr);
	default:
		break;
	}

	if (!emac->phydev)
		return -EOPNOTSUPP;

	return phy_mii_ioctl(emac->phydev, ifr, cmd);
}

static struct devlink_port *emac_ndo_get_devlink_port(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);

	return &emac->devlink_port;
}

/**
 * emac_xmit_xdp_frame - transmits an XDP frame
 * @emac: emac device
 * @xdpf: data to transmit
 * @page: page from page pool if already DMA mapped
 * @q_idx: queue id
 **/
static int emac_xmit_xdp_frame(struct prueth_emac *emac,
			       struct xdp_frame *xdpf,
			       struct page *page,
			       unsigned int q_idx)
{
	struct cppi5_host_desc_t *first_desc;
	struct net_device *ndev = emac->ndev;
	struct prueth_tx_chn *tx_chn;
	dma_addr_t desc_dma, buf_dma;
	struct prueth_swdata *swdata;
	u32 *epib;
	int ret;

	void *data = xdpf->data;
	u32 pkt_len = xdpf->len;

	if (q_idx >= PRUETH_MAX_TX_QUEUES) {
		netdev_err(ndev, "xdp tx: invalid q_id %d\n", q_idx);
		return ICSSG_XDP_CONSUMED;	/* drop */
	}

	tx_chn = &emac->tx_chns[q_idx];

	if (page) { /* already DMA mapped by page_pool */
		buf_dma = page_pool_get_dma_addr(page);
		buf_dma += xdpf->headroom + sizeof(struct xdp_frame);
	} else { /* Map the linear buffer */
		buf_dma = dma_map_single(tx_chn->dma_dev, data, pkt_len, DMA_TO_DEVICE);
		if (dma_mapping_error(tx_chn->dma_dev, buf_dma)) {
			netdev_err(ndev, "xdp tx: failed to map data buffer\n");
			return ICSSG_XDP_CONSUMED;	/* drop */
		}
	}

	first_desc = k3_cppi_desc_pool_alloc(tx_chn->desc_pool);
	if (!first_desc) {
		netdev_dbg(ndev, "xdp tx: failed to allocate descriptor\n");
		if (!page)
			dma_unmap_single(tx_chn->dma_dev, buf_dma, pkt_len, DMA_TO_DEVICE);
		return ICSSG_XDP_CONSUMED;	/* drop */
	}

	cppi5_hdesc_init(first_desc, CPPI5_INFO0_HDESC_EPIB_PRESENT,
			 PRUETH_NAV_PS_DATA_SIZE);
	cppi5_hdesc_set_pkttype(first_desc, 0);
	epib = first_desc->epib;
	epib[0] = 0;
	epib[1] = 0;

	/* set dst tag to indicate internal qid at the firmware which is at
	 * bit8..bit15. bit0..bit7 indicates port num for directed
	 * packets in case of switch mode operation
	 */
	cppi5_desc_set_tags_ids(&first_desc->hdr, 0, (emac->port_id | (q_idx << 8)));
	k3_udma_glue_tx_dma_to_cppi5_addr(tx_chn->tx_chn, &buf_dma);
	cppi5_hdesc_attach_buf(first_desc, buf_dma, pkt_len, buf_dma, pkt_len);
	swdata = cppi5_hdesc_get_swdata(first_desc);
	if (page) {
		swdata->type = PRUETH_SWDATA_PAGE;
		swdata->data.page = page;
		/* we assume page came from RX channel page pool */
		swdata->rx_chn = &emac->rx_chns;
	} else {
		swdata->type = PRUETH_SWDATA_XDPF;
		swdata->data.xdpf = xdpf;
	}

	cppi5_hdesc_set_pktlen(first_desc, pkt_len);
	desc_dma = k3_cppi_desc_pool_virt2dma(tx_chn->desc_pool, first_desc);

	ret = k3_udma_glue_push_tx_chn(tx_chn->tx_chn, first_desc, desc_dma);
	if (ret) {
		netdev_err(ndev, "xdp tx: push failed: %d\n", ret);
		goto drop_free_descs;
	}

	return ICSSG_XDP_TX;

drop_free_descs:
	prueth_xmit_free(tx_chn, first_desc);
	return ICSSG_XDP_CONSUMED;
}

/**
 * emac_xdp_xmit - Implements ndo_xdp_xmit
 * @dev: netdev
 * @n: number of frames
 * @frames: array of XDP buffer pointers
 * @flags: XDP extra info
 *
 * Returns number of frames successfully sent. Failed frames
 * will be free'ed by XDP core.
 *
 * For error cases, a negative errno code is returned and no-frames
 * are transmitted (caller must handle freeing frames).
 **/
static int emac_xdp_xmit(struct net_device *dev, int n, struct xdp_frame **frames,
			 u32 flags)
{
	struct prueth_emac *emac = netdev_priv(dev);
	unsigned int q_idx;
	int nxmit = 0;
	int i;

	q_idx = smp_processor_id();

	if (unlikely(flags & ~XDP_XMIT_FLAGS_MASK))
		return -EINVAL;

	for (i = 0; i < n; i++) {
		struct xdp_frame *xdpf = frames[i];
		int err;

		err = emac_xmit_xdp_frame(emac, xdpf, NULL, q_idx);
		if (err != ICSSG_XDP_TX)
			break;
		nxmit++;
	}

	return nxmit;
}

/**
 * emac_run_xdp - run an XDP program
 * @emac: emac device
 * @xdp: XDP buffer containing the frame
 * @page: page with RX data if already DMA mapped
 **/
static int emac_run_xdp(struct prueth_emac *emac, struct xdp_buff *xdp,
			struct page *page)
{
	int err, result = ICSSG_XDP_PASS;
	struct bpf_prog *xdp_prog;
	struct xdp_frame *xdpf;
	u32 act;
	int q_idx;

	xdp_prog = READ_ONCE(emac->xdp_prog);

	if (!xdp_prog)
		return result;

	act = bpf_prog_run_xdp(xdp_prog, xdp);
	switch (act) {
	case XDP_PASS:
		break;
	case XDP_TX:
		/* Send packet to TX ring for immediate transmission */
		xdpf = xdp_convert_buff_to_frame(xdp);
		if (unlikely(!xdpf))
			goto drop;

		q_idx = smp_processor_id();
		result = emac_xmit_xdp_frame(emac, xdpf, page, q_idx);
		if (result == ICSSG_XDP_CONSUMED)
			goto drop;
		break;
	case XDP_REDIRECT:
		err = xdp_do_redirect(emac->ndev, xdp, xdp_prog);
		if (err)
			goto drop;
		result = ICSSG_XDP_REDIR;
		break;
	default:
		bpf_warn_invalid_xdp_action(act);
		fallthrough;
	case XDP_ABORTED:
drop:
		trace_xdp_exception(emac->ndev, xdp_prog, act);
		fallthrough; /* handle aborts by dropping packet */
	case XDP_DROP:
		result = ICSSG_XDP_CONSUMED;
		page_pool_recycle_direct(emac->rx_chns.pg_pool, page);
		break;
	}

	return result;
}

/**
 * emac_xdp_setup - add/remove an XDP program
 * @emac: emac device
 * @prog: XDP program
 **/
static int emac_xdp_setup(struct prueth_emac *emac, struct netdev_bpf *bpf)
{
	struct bpf_prog *prog = bpf->prog;

	if (!emac->xdpi.prog && !prog)
		return 0;

	WRITE_ONCE(emac->xdp_prog, prog);

	xdp_attachment_setup(&emac->xdpi, bpf);

	return 0;
}

/**
 * emac_ndo_bpf - implements ndo_bpf for icssg_prueth
 * @ndev: network adapter device
 * @xdp: XDP command
 **/
static int emac_ndo_bpf(struct net_device *ndev, struct netdev_bpf *bpf)
{
	struct prueth_emac *emac = netdev_priv(ndev);

	switch (bpf->command) {
	case XDP_SETUP_PROG:
		return emac_xdp_setup(emac, bpf);
	default:
		return -EINVAL;
	}
}

static const struct net_device_ops emac_netdev_ops = {
	.ndo_open = emac_ndo_open,
	.ndo_stop = emac_ndo_stop,
	.ndo_start_xmit = emac_ndo_start_xmit,
	.ndo_set_mac_address = eth_mac_addr,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_tx_timeout = emac_ndo_tx_timeout,
	.ndo_set_rx_mode = emac_ndo_set_rx_mode,
	.ndo_do_ioctl = emac_ndo_ioctl,
	.ndo_get_devlink_port = emac_ndo_get_devlink_port,
	.ndo_setup_tc = icssg_qos_ndo_setup_tc,
	.ndo_bpf = emac_ndo_bpf,
	.ndo_xdp_xmit = emac_xdp_xmit,
};

/* get emac_port corresponding to eth_node name */
static int prueth_node_port(struct device_node *eth_node)
{
	if (!strcmp(eth_node->name, "ethernet-mii0"))
		return PRUETH_PORT_MII0;
	else if (!strcmp(eth_node->name, "ethernet-mii1"))
		return PRUETH_PORT_MII1;
	else
		return -EINVAL;
}

/* get MAC instance corresponding to eth_node name */
static int prueth_node_mac(struct device_node *eth_node)
{
	if (!strcmp(eth_node->name, "ethernet-mii0"))
		return PRUETH_MAC0;
	else if (!strcmp(eth_node->name, "ethernet-mii1"))
		return PRUETH_MAC1;
	else
		return -EINVAL;
}

static int prueth_config_rgmiidelay(struct prueth *prueth,
				    struct device_node *eth_np,
				    phy_interface_t phy_if)
{
	struct device *dev = prueth->dev;
	struct regmap *ctrl_mmr;
	u32 rgmii_tx_id = 0;
	u32 icssgctrl_reg;

	if (!phy_interface_mode_is_rgmii(phy_if))
		return 0;

	ctrl_mmr = syscon_regmap_lookup_by_phandle(eth_np, "syscon-rgmii-delay");
	if (IS_ERR(ctrl_mmr)) {
		dev_err(dev, "couldn't get syscon-rgmii-delay\n");
		return -ENODEV;
	}

	if (of_property_read_u32_index(eth_np, "syscon-rgmii-delay", 1,
				       &icssgctrl_reg)) {
		dev_err(dev, "couldn't get rgmii-delay reg. offset\n");
		return -ENODEV;
	}

	if (phy_if == PHY_INTERFACE_MODE_RGMII_ID ||
	    phy_if == PHY_INTERFACE_MODE_RGMII_TXID)
		rgmii_tx_id |= ICSSG_CTRL_RGMII_ID_MODE;

	regmap_update_bits(ctrl_mmr, icssgctrl_reg, ICSSG_CTRL_RGMII_ID_MODE, rgmii_tx_id);

	return 0;
}

extern const struct ethtool_ops icssg_ethtool_ops;

static int prueth_netdev_init(struct prueth *prueth,
			      struct device_node *eth_node)
{
	int ret, num_tx_chn = PRUETH_MAX_TX_QUEUES;
	struct prueth_emac *emac;
	struct net_device *ndev;
	enum prueth_port port;
	const char *irq_name;
	enum prueth_mac mac;
	const u8 *mac_addr;

	port = prueth_node_port(eth_node);
	if (port < 0)
		return -EINVAL;

	mac = prueth_node_mac(eth_node);
	if (mac < 0)
		return -EINVAL;

	/* Use 1 channel for management messages on SR1 */
	if (prueth->is_sr1)
		num_tx_chn--;

	ndev = alloc_etherdev_mq(sizeof(*emac), num_tx_chn);
	if (!ndev)
		return -ENOMEM;

	emac = netdev_priv(ndev);
	prueth->emac[mac] = emac;
	emac->prueth = prueth;
	emac->ndev = ndev;
	emac->port_id = port;
	emac->cmd_wq = create_singlethread_workqueue("icssg_cmd_wq");
	if (!emac->cmd_wq) {
		ret = -ENOMEM;
		goto free_ndev;
	}
	INIT_WORK(&emac->rx_mode_work, emac_ndo_set_rx_mode_work);

	emac_ethtool_stats_init(emac);
	INIT_DELAYED_WORK(&emac->stats_work, emac_stats_work_handler);

	ret = pruss_request_mem_region(prueth->pruss,
				       port == PRUETH_PORT_MII0 ?
				       PRUSS_MEM_DRAM0 : PRUSS_MEM_DRAM1,
				       &emac->dram);
	if (ret) {
		dev_err(prueth->dev, "unable to get DRAM: %d\n", ret);
		return -ENOMEM;
	}

	emac->is_sr1 = prueth->is_sr1;
	emac->tx_ch_num = 1;
	if (emac->is_sr1) {
		/* use a dedicated high priority channel for management
		 * messages which is +1 of highest priority data channel.
		 */
		emac->tx_ch_num++;
		goto skip_irq;
	}

	irq_name = "tx_ts0";
	if (emac->port_id == PRUETH_PORT_MII1)
		irq_name = "tx_ts1";
	emac->tx_ts_irq = platform_get_irq_byname_optional(prueth->pdev, irq_name);
	if (emac->tx_ts_irq < 0) {
		ret = dev_err_probe(prueth->dev, emac->tx_ts_irq, "could not get tx_ts_irq\n");
		goto free;
	}

skip_irq:
	SET_NETDEV_DEV(ndev, prueth->dev);
	emac->msg_enable = netif_msg_init(debug_level, PRUETH_EMAC_DEBUG);
	spin_lock_init(&emac->lock);
	mutex_init(&emac->cmd_lock);

	emac->phy_node = of_parse_phandle(eth_node, "phy-handle", 0);
	if (!emac->phy_node && !of_phy_is_fixed_link(eth_node)) {
		dev_err(prueth->dev, "couldn't find phy-handle\n");
		ret = -ENODEV;
		goto free;
	} else if (of_phy_is_fixed_link(eth_node)) {
		ret = of_phy_register_fixed_link(eth_node);
		if (ret) {
			ret = dev_err_probe(prueth->dev, ret,
					    "failed to register fixed-link phy\n");
			goto free;
		}

		emac->phy_node = eth_node;
	}

	ret = of_get_phy_mode(eth_node, &emac->phy_if);
	if (ret) {
		dev_err(prueth->dev, "could not get phy-mode property\n");
		goto free;
	}

	if (emac->phy_if != PHY_INTERFACE_MODE_MII &&
	    !phy_interface_mode_is_rgmii(emac->phy_if)) {
		dev_err(prueth->dev, "PHY mode unsupported %s\n", phy_modes(emac->phy_if));
		goto free;
	}

	ret = prueth_config_rgmiidelay(prueth, eth_node, emac->phy_if);
	if (ret)
		goto free;

	/* get mac address from DT and set private and netdev addr */
	mac_addr = of_get_mac_address(eth_node);
	if (!IS_ERR(mac_addr))
		ether_addr_copy(ndev->dev_addr, mac_addr);
	if (!is_valid_ether_addr(ndev->dev_addr)) {
		eth_hw_addr_random(ndev);
		dev_warn(prueth->dev, "port %d: using random MAC addr: %pM\n",
			 port, ndev->dev_addr);
	}
	ether_addr_copy(emac->mac_addr, ndev->dev_addr);

	ndev->min_mtu = PRUETH_MIN_PKT_SIZE;
	ndev->max_mtu = PRUETH_MAX_MTU;
	ndev->netdev_ops = &emac_netdev_ops;
	ndev->ethtool_ops = &icssg_ethtool_ops;
	ndev->hw_features = NETIF_F_SG;
	ndev->features = ndev->hw_features;

	netif_napi_add(ndev, &emac->napi_rx,
		       emac_napi_rx_poll, NAPI_POLL_WEIGHT);

	return 0;

free:
	pruss_release_mem_region(prueth->pruss, &emac->dram);
	destroy_workqueue(emac->cmd_wq);
free_ndev:
	free_netdev(ndev);
	prueth->emac[mac] = NULL;

	return ret;
}

static void prueth_netdev_exit(struct prueth *prueth,
			       struct device_node *eth_node)
{
	struct prueth_emac *emac;
	enum prueth_mac mac;

	mac = prueth_node_mac(eth_node);
	if (mac < 0)
		return;

	emac = prueth->emac[mac];
	if (!emac)
		return;

	if (of_phy_is_fixed_link(emac->phy_node))
		of_phy_deregister_fixed_link(emac->phy_node);

	netif_napi_del(&emac->napi_rx);

	pruss_release_mem_region(prueth->pruss, &emac->dram);
	destroy_workqueue(emac->cmd_wq);
	free_netdev(emac->ndev);
	prueth->emac[mac] = NULL;
}

static int prueth_get_cores(struct prueth *prueth, int slice)
{
	enum pruss_pru_id pruss_id;
	struct device *dev = prueth->dev;
	struct device_node *np = dev->of_node;
	int pru, rtu, txpru = -1, ret;

	switch (slice) {
	case ICSS_SLICE0:
		pru = 0;
		rtu = 1;
		if (!prueth->is_sr1)
			txpru = 2;
		break;
	case ICSS_SLICE1:
		if (prueth->is_sr1) {
			pru = 2;
			rtu = 3;
		} else {
			pru = 3;
			rtu = 4;
			txpru = 5;
		}
		break;
	default:
		return -EINVAL;
	}

	prueth->pru[slice] = pru_rproc_get(np, pru, &pruss_id);
	if (IS_ERR(prueth->pru[slice])) {
		ret = PTR_ERR(prueth->pru[slice]);
		prueth->pru[slice] = NULL;
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "unable to get PRU%d: %d\n", slice, ret);
		return ret;
	}
	prueth->pru_id[slice] = pruss_id;

	prueth->rtu[slice] = pru_rproc_get(np, rtu, NULL);
	if (IS_ERR(prueth->rtu[slice])) {
		ret = PTR_ERR(prueth->rtu[slice]);
		prueth->rtu[slice] = NULL;
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "unable to get RTU%d: %d\n", slice, ret);
		return ret;
	}

	if (prueth->is_sr1)
		return 0;

	prueth->txpru[slice] = pru_rproc_get(np, txpru, NULL);
	if (IS_ERR(prueth->txpru[slice])) {
		ret = PTR_ERR(prueth->txpru[slice]);
		prueth->txpru[slice] = NULL;
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "unable to get TX_PRU%d: %d\n",
				slice, ret);
		return ret;
	}

	return 0;
}

static void prueth_put_cores(struct prueth *prueth, int slice)
{
	if (prueth->txpru[slice])
		pru_rproc_put(prueth->txpru[slice]);

	if (prueth->rtu[slice])
		pru_rproc_put(prueth->rtu[slice]);

	if (prueth->pru[slice])
		pru_rproc_put(prueth->pru[slice]);
}

static void prueth_offload_fwd_mark_update(struct prueth *prueth)
{
	int set_val = 0;
	int i;

	if (prueth->br_members == (PRUETH_PORT_MII0 | PRUETH_PORT_MII1))
		set_val = 1;

	dev_dbg(prueth->dev, "set offload_fwd_mark %d\n", set_val);

	for (i = PRUETH_MAC0; i < PRUETH_NUM_MACS; i++) {
		struct prueth_emac *emac = prueth->emac[i];

		if (!emac || !emac->ndev)
			continue;

		emac->offload_fwd_mark = set_val;
	}
}

bool prueth_dev_check(const struct net_device *ndev)
{
	if (ndev->netdev_ops == &emac_netdev_ops && netif_running(ndev)) {
		struct prueth_emac *emac = netdev_priv(ndev);

		return emac->prueth->is_switch_mode;
	}

	return false;
}

static int prueth_netdevice_port_link(struct net_device *ndev, struct net_device *br_ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth *prueth = emac->prueth;

	if (!prueth->is_switch_mode)
		return NOTIFY_DONE;

	if (!prueth->br_members) {
		prueth->hw_bridge_dev = br_ndev;
	} else {
		/* This is adding the port to a second bridge, this is
		 * unsupported
		 */
		if (prueth->hw_bridge_dev != br_ndev)
			return -EOPNOTSUPP;
	}

	prueth->br_members |= BIT(emac->port_id);

	prueth_offload_fwd_mark_update(prueth);

	return NOTIFY_DONE;
}

static void prueth_netdevice_port_unlink(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth *prueth = emac->prueth;

	prueth->br_members &= ~BIT(emac->port_id);

	prueth_offload_fwd_mark_update(prueth);

	if (!prueth->br_members)
		prueth->hw_bridge_dev = NULL;
}

/* netdev notifier */
static int prueth_netdevice_event(struct notifier_block *unused,
				  unsigned long event, void *ptr)
{
	struct net_device *ndev = netdev_notifier_info_to_dev(ptr);
	struct netdev_notifier_changeupper_info *info;
	int ret = NOTIFY_DONE;

	if (ndev->netdev_ops != &emac_netdev_ops)
		return NOTIFY_DONE;

	switch (event) {
	case NETDEV_CHANGEUPPER:
		info = ptr;

		if (netif_is_bridge_master(info->upper_dev)) {
			if (info->linking)
				ret = prueth_netdevice_port_link(ndev, info->upper_dev);
			else
				prueth_netdevice_port_unlink(ndev);
		}
		break;
	default:
		return NOTIFY_DONE;
	}

	return notifier_from_errno(ret);
}

static int prueth_register_notifiers(struct prueth *prueth)
{
	int ret = 0;

	prueth->prueth_netdevice_nb.notifier_call = &prueth_netdevice_event;
	ret = register_netdevice_notifier(&prueth->prueth_netdevice_nb);
	if (ret) {
		dev_err(prueth->dev, "can't register netdevice notifier\n");
		return ret;
	}

	ret = prueth_switchdev_register_notifiers(prueth);
	if (ret)
		unregister_netdevice_notifier(&prueth->prueth_netdevice_nb);

	return ret;
}

static void prueth_unregister_notifiers(struct prueth *prueth)
{
	prueth_switchdev_unregister_notifiers(prueth);
	unregister_netdevice_notifier(&prueth->prueth_netdevice_nb);
}

static const struct devlink_ops prueth_devlink_ops = {};

static int prueth_dl_switch_mode_get(struct devlink *dl, u32 id,
				     struct devlink_param_gset_ctx *ctx)
{
	struct prueth_devlink *dl_priv = devlink_priv(dl);
	struct prueth *prueth = dl_priv->prueth;

	dev_dbg(prueth->dev, "%s id:%u\n", __func__, id);

	if (id != PRUETH_DL_PARAM_SWITCH_MODE)
		return -EOPNOTSUPP;

	ctx->val.vbool = prueth->is_switch_mode;

	return 0;
}

static int prueth_dl_switch_mode_set(struct devlink *dl, u32 id,
				     struct devlink_param_gset_ctx *ctx)
{
	struct prueth_devlink *dl_priv = devlink_priv(dl);
	struct prueth *prueth = dl_priv->prueth;
	bool switch_en = ctx->val.vbool;
	int i;

	dev_dbg(prueth->dev, "%s id:%u\n", __func__, id);

	if (id != PRUETH_DL_PARAM_SWITCH_MODE)
		return -EOPNOTSUPP;

	if (switch_en == prueth->is_switch_mode)
		return 0;

	if (!switch_en && prueth->br_members) {
		dev_err(prueth->dev, "Remove ports from bridge before disabling switch mode\n");
		return -EINVAL;
	}

	rtnl_lock();

	prueth->default_vlan = 1;
	prueth->is_switch_mode = switch_en;

	for (i = PRUETH_MAC0; i < PRUETH_NUM_MACS; i++) {
		struct net_device *sl_ndev = prueth->emac[i]->ndev;

		if (!sl_ndev || !netif_running(sl_ndev))
			continue;

		dev_err(prueth->dev, "Cannot switch modes when i/f are up\n");
		goto exit;
	}

	for (i = PRUETH_MAC0; i < PRUETH_NUM_MACS; i++) {
		struct net_device *sl_ndev = prueth->emac[i]->ndev;
		struct prueth_emac *emac;

		if (!sl_ndev)
			continue;

		emac = netdev_priv(sl_ndev);
		if (switch_en)
			emac->port_vlan = prueth->default_vlan;
		else
			emac->port_vlan = 0;
	}

	dev_info(prueth->dev, "Enabling %s mode\n",
		 switch_en ? "switch" : "Dual EMAC");

exit:
	rtnl_unlock();

	return 0;
}

static const struct devlink_param prueth_devlink_params[] = {
	DEVLINK_PARAM_DRIVER(PRUETH_DL_PARAM_SWITCH_MODE, "switch_mode",
			     DEVLINK_PARAM_TYPE_BOOL,
			     BIT(DEVLINK_PARAM_CMODE_RUNTIME),
			     prueth_dl_switch_mode_get,
			     prueth_dl_switch_mode_set, NULL),
};

static void prueth_unregister_devlink_ports(struct prueth *prueth)
{
	struct devlink_port *dl_port;
	struct prueth_emac *emac;
	int i;

	for (i = PRUETH_MAC0; i < PRUETH_NUM_MACS; i++) {
		emac = prueth->emac[i];
		if (!emac)
			continue;

		dl_port = &emac->devlink_port;

		if (dl_port->registered)
			devlink_port_unregister(dl_port);
	}
}

static int prueth_register_devlink(struct prueth *prueth)
{
	struct devlink_port_attrs attrs = {};
	struct prueth_devlink *dl_priv;
	struct device *dev = prueth->dev;
	struct devlink_port *dl_port;
	struct prueth_emac *emac;
	int ret = 0;
	int i;

	prueth->devlink =
		devlink_alloc(&prueth_devlink_ops, sizeof(*dl_priv));
	if (!prueth->devlink)
		return -ENOMEM;

	dl_priv = devlink_priv(prueth->devlink);
	dl_priv->prueth = prueth;

	ret = devlink_register(prueth->devlink, dev);
	if (ret) {
		dev_err(dev, "devlink reg fail ret:%d\n", ret);
		goto dl_free;
	}

	/* Provide devlink hook to switch mode when multiple external ports
	 * are present NUSS switchdev driver is enabled.
	 */
	if (prueth->is_switchmode_supported) {
		ret = devlink_params_register(prueth->devlink,
					      prueth_devlink_params,
					      ARRAY_SIZE(prueth_devlink_params));
		if (ret) {
			dev_err(dev, "devlink params reg fail ret:%d\n", ret);
			goto dl_unreg;
		}
		devlink_params_publish(prueth->devlink);
	}

	for (i = PRUETH_MAC0; i < PRUETH_NUM_MACS; i++) {
		emac = prueth->emac[i];
		if (!emac)
			continue;

		dl_port = &emac->devlink_port;

		attrs.flavour = DEVLINK_PORT_FLAVOUR_PHYSICAL;
		attrs.phys.port_number = emac->port_id;
		attrs.switch_id.id_len = sizeof(resource_size_t);
		memcpy(attrs.switch_id.id, prueth->switch_id, attrs.switch_id.id_len);
		devlink_port_attrs_set(dl_port, &attrs);

		ret = devlink_port_register(prueth->devlink, dl_port, emac->port_id);
		if (ret) {
			dev_err(dev, "devlink_port reg fail for port %d, ret:%d\n",
				emac->port_id, ret);
			goto dl_port_unreg;
		}
	}

	return ret;

dl_port_unreg:
	prueth_unregister_devlink_ports(prueth);
dl_unreg:
	devlink_unregister(prueth->devlink);
dl_free:
	devlink_free(prueth->devlink);

	return ret;
}

static void prueth_unregister_devlink(struct prueth *prueth)
{
	if (prueth->is_switchmode_supported) {
		devlink_params_unpublish(prueth->devlink);
		devlink_params_unregister(prueth->devlink, prueth_devlink_params,
					  ARRAY_SIZE(prueth_devlink_params));
	}

	prueth_unregister_devlink_ports(prueth);
	devlink_unregister(prueth->devlink);
	devlink_free(prueth->devlink);
}

static const struct of_device_id prueth_dt_match[];

static int prueth_probe(struct platform_device *pdev)
{
	struct prueth *prueth;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *eth0_node, *eth1_node;
	const struct of_device_id *match;
	struct pruss *pruss;
	int i, ret;
	u32 msmc_ram_size;
	struct genpool_data_align gp_data = {
		.align = SZ_64K,
	};

	if (sizeof(struct prueth_swdata) > PRUETH_NAV_SW_DATA_SIZE) {
		dev_err(dev, "insufficient SW_DATA size: %d vs %ld\n",
			PRUETH_NAV_SW_DATA_SIZE, sizeof(struct prueth_swdata));
		return -ENOMEM;
	}

	match = of_match_device(prueth_dt_match, dev);
	if (!match)
		return -ENODEV;

	prueth = devm_kzalloc(dev, sizeof(*prueth), GFP_KERNEL);
	if (!prueth)
		return -ENOMEM;

	dev_set_drvdata(dev, prueth);
	prueth->pdev = pdev;
	prueth->pdata = *(const struct prueth_pdata *)match->data;

	if (of_device_is_compatible(np, "ti,am654-icssg-prueth-sr1"))
		prueth->is_sr1 = true;

	prueth->dev = dev;
	eth0_node = of_get_child_by_name(np, "ethernet-mii0");
	if (!of_device_is_available(eth0_node)) {
		of_node_put(eth0_node);
		eth0_node = NULL;
	}

	eth1_node = of_get_child_by_name(np, "ethernet-mii1");
	if (!of_device_is_available(eth1_node)) {
		of_node_put(eth1_node);
		eth1_node = NULL;
	}

	/* At least one node must be present and available else we fail */
	if (!eth0_node && !eth1_node) {
		dev_err(dev, "neither ethernet-mii0 nor ethernet-mii1 node available\n");
		return -ENODEV;
	}

	prueth->eth_node[PRUETH_MAC0] = eth0_node;
	prueth->eth_node[PRUETH_MAC1] = eth1_node;

	prueth->miig_rt = syscon_regmap_lookup_by_phandle(np, "mii-g-rt");
	if (IS_ERR(prueth->miig_rt)) {
		dev_err(dev, "couldn't get mii-g-rt syscon regmap\n");
		return -ENODEV;
	}

	prueth->mii_rt = syscon_regmap_lookup_by_phandle(np, "mii-rt");
	if (IS_ERR(prueth->mii_rt)) {
		dev_err(dev, "couldn't get mii-rt syscon regmap\n");
		return -ENODEV;
	}

	if (eth0_node) {
		ret = prueth_get_cores(prueth, ICSS_SLICE0);
		if (ret)
			goto put_cores;
	}

	if (eth1_node) {
		ret = prueth_get_cores(prueth, ICSS_SLICE1);
		if (ret)
			goto put_cores;
	}

	pruss = pruss_get(eth0_node ?
			  prueth->pru[ICSS_SLICE0] : prueth->pru[ICSS_SLICE1]);
	if (IS_ERR(pruss)) {
		ret = PTR_ERR(pruss);
		dev_err(dev, "unable to get pruss handle\n");
		goto put_cores;
	}

	prueth->pruss = pruss;

	ret = pruss_request_mem_region(pruss, PRUSS_MEM_SHRD_RAM2,
				       &prueth->shram);
	if (ret) {
		dev_err(dev, "unable to get PRUSS SHRD RAM2: %d\n", ret);
		goto put_mem;
	}

	prueth->sram_pool = of_gen_pool_get(np, "sram", 0);
	if (!prueth->sram_pool) {
		dev_err(dev, "unable to get SRAM pool\n");
		ret = -ENODEV;

		goto put_mem;
	}

	msmc_ram_size = prueth->is_sr1 ? MSMC_RAM_SIZE_SR1 : MSMC_RAM_SIZE_SR2;
	prueth->is_switchmode_supported = prueth->pdata.switch_mode;
	if (prueth->is_switchmode_supported)
		msmc_ram_size = MSMC_RAM_SIZE_SR2_SWITCH_MODE;

	if (prueth->is_sr1) {
		prueth->msmcram.va =
			(void __iomem *)gen_pool_alloc(prueth->sram_pool,
						       msmc_ram_size);
	} else {
		/* TEMP: FW bug needs buffer base to be 64KB aligned */
		prueth->msmcram.va =
			(void __iomem *)gen_pool_alloc_algo(prueth->sram_pool,
							    msmc_ram_size,
							    gen_pool_first_fit_align,
							    &gp_data);
	}

	if (!prueth->msmcram.va) {
		ret = -ENOMEM;
		dev_err(dev, "unable to allocate MSMC resource\n");
		goto put_mem;
	}
	prueth->msmcram.pa = gen_pool_virt_to_phys(prueth->sram_pool,
						   (unsigned long)prueth->msmcram.va);
	prueth->msmcram.size = msmc_ram_size;
	memset(prueth->msmcram.va, 0, msmc_ram_size);
	dev_dbg(dev, "sram: pa %llx va %p size %zx\n", prueth->msmcram.pa,
		prueth->msmcram.va, prueth->msmcram.size);

	prueth->iep0 = icss_iep_get_idx(np, 0);
	if (IS_ERR(prueth->iep0)) {
		ret = dev_err_probe(dev, PTR_ERR(prueth->iep0), "iep0 get failed\n");
		prueth->iep0 = NULL;
		goto free_pool;
	}

	prueth->iep1 = icss_iep_get_idx(np, 1);
	if (IS_ERR(prueth->iep1)) {
		ret = dev_err_probe(dev, PTR_ERR(prueth->iep1), "iep1 get failed\n");
		icss_iep_put(prueth->iep0);
		prueth->iep0 = NULL;
		prueth->iep1 = NULL;
		goto free_pool;
	}

	if (prueth->is_sr1) {
		ret = icss_iep_init(prueth->iep0, NULL, NULL, 0);
		if (ret) {
			dev_err(dev, "failed to init iep0\n");
			goto free_iep;
		}

		ret = icss_iep_init(prueth->iep1, NULL, NULL, 0);
		if (ret) {
			dev_err(dev, "failed to init iep1\n");
			icss_iep_exit(prueth->iep1);
			goto free_iep;
		}
	} else if (prueth->pdata.quirk_10m_link_issue) {
		/* Enable IEP1 for FW in 64bit mode as W/A for 10M FD link detect issue under TX
		 * traffic.
		 */
		icss_iep_init_fw(prueth->iep1);
	}

	/* setup netdev interfaces */
	if (eth0_node) {
		ret = prueth_netdev_init(prueth, eth0_node);
		if (ret) {
			if (ret != -EPROBE_DEFER) {
				dev_err(dev, "netdev init %s failed: %d\n",
					eth0_node->name, ret);
			}
			goto exit_iep;
		}
		prueth->emac[PRUETH_MAC0]->iep = prueth->iep0;
	}

	if (eth1_node) {
		ret = prueth_netdev_init(prueth, eth1_node);
		if (ret) {
			if (ret != -EPROBE_DEFER) {
				dev_err(dev, "netdev init %s failed: %d\n",
					eth1_node->name, ret);
			}
			goto netdev_exit;
		}

		if (prueth->is_sr1)
			prueth->emac[PRUETH_MAC1]->iep = prueth->iep1;
		else
			prueth->emac[PRUETH_MAC1]->iep = prueth->iep0;
	}

	ret = prueth_register_devlink(prueth);
	if (ret)
		goto netdev_exit;

	/* register the network devices */
	if (eth0_node) {
		ret = register_netdev(prueth->emac[PRUETH_MAC0]->ndev);
		if (ret) {
			dev_err(dev, "can't register netdev for port MII0");
			goto netdev_exit;
		}

		devlink_port_type_eth_set(&prueth->emac[PRUETH_MAC0]->devlink_port,
					  prueth->emac[PRUETH_MAC0]->ndev);
		prueth->registered_netdevs[PRUETH_MAC0] = prueth->emac[PRUETH_MAC0]->ndev;

		emac_phy_connect(prueth->emac[PRUETH_MAC0]);
		/* Get attached phy details */
		phy_attached_info(prueth->emac[PRUETH_MAC0]->phydev);

	}

	if (eth1_node) {
		ret = register_netdev(prueth->emac[PRUETH_MAC1]->ndev);
		if (ret) {
			dev_err(dev, "can't register netdev for port MII1");
			goto netdev_unregister;
		}

		devlink_port_type_eth_set(&prueth->emac[PRUETH_MAC1]->devlink_port,
					  prueth->emac[PRUETH_MAC1]->ndev);

		prueth->registered_netdevs[PRUETH_MAC1] = prueth->emac[PRUETH_MAC1]->ndev;

		emac_phy_connect(prueth->emac[PRUETH_MAC1]);
		/* Get attached phy details */
		phy_attached_info(prueth->emac[PRUETH_MAC1]->phydev);
	}

	if (prueth->is_switchmode_supported) {
		ret = prueth_register_notifiers(prueth);
		if (ret)
			goto netdev_unregister;

		sprintf(prueth->switch_id, "%s", dev_name(dev));
	}

	dev_info(dev, "TI PRU ethernet driver initialized: %s EMAC mode\n",
		 (!eth0_node || !eth1_node) ? "single" : "dual");

	if (eth1_node)
		of_node_put(eth1_node);
	if (eth0_node)
		of_node_put(eth0_node);

	return 0;

netdev_unregister:
	for (i = 0; i < PRUETH_NUM_MACS; i++) {
		if (!prueth->registered_netdevs[i])
			continue;
		if (prueth->emac[i]->phydev) {
			phy_disconnect(prueth->emac[i]->phydev);
			prueth->emac[i]->phydev = NULL;
		}
		unregister_netdev(prueth->registered_netdevs[i]);
	}

netdev_exit:
	for (i = 0; i < PRUETH_NUM_MACS; i++) {
		struct device_node *eth_node;

		eth_node = prueth->eth_node[i];
		if (!eth_node)
			continue;

		prueth_netdev_exit(prueth, eth_node);
	}
exit_iep:
	if (prueth->is_sr1) {
		icss_iep_exit(prueth->iep1);
		icss_iep_exit(prueth->iep0);
	} else if (prueth->pdata.quirk_10m_link_issue) {
		icss_iep_exit_fw(prueth->iep1);
	}

free_iep:
	icss_iep_put(prueth->iep1);
	icss_iep_put(prueth->iep0);

free_pool:
	gen_pool_free(prueth->sram_pool,
		      (unsigned long)prueth->msmcram.va, msmc_ram_size);

put_mem:
	pruss_release_mem_region(prueth->pruss, &prueth->shram);
	pruss_put(prueth->pruss);

put_cores:
	if (eth1_node) {
		prueth_put_cores(prueth, ICSS_SLICE1);
		of_node_put(eth1_node);
	}

	if (eth0_node) {
		prueth_put_cores(prueth, ICSS_SLICE0);
		of_node_put(eth0_node);
	}

	return ret;
}

static int prueth_remove(struct platform_device *pdev)
{
	struct device_node *eth_node;
	struct prueth *prueth = platform_get_drvdata(pdev);
	int i;

	prueth_unregister_notifiers(prueth);

	for (i = 0; i < PRUETH_NUM_MACS; i++) {
		if (!prueth->registered_netdevs[i])
			continue;
		phy_disconnect(prueth->emac[i]->phydev);
		prueth->emac[i]->phydev = NULL;
		unregister_netdev(prueth->registered_netdevs[i]);
	}
	prueth_unregister_devlink(prueth);

	for (i = 0; i < PRUETH_NUM_MACS; i++) {
		eth_node = prueth->eth_node[i];
		if (!eth_node)
			continue;

		prueth_netdev_exit(prueth, eth_node);
	}

	if (prueth->is_sr1) {
		icss_iep_exit(prueth->iep1);
		icss_iep_exit(prueth->iep0);
	} else if (prueth->pdata.quirk_10m_link_issue) {
		icss_iep_exit_fw(prueth->iep1);
	}

	icss_iep_put(prueth->iep1);
	icss_iep_put(prueth->iep0);

	gen_pool_free(prueth->sram_pool,
		      (unsigned long)prueth->msmcram.va,
		      prueth->is_sr1 ? MSMC_RAM_SIZE_SR1 : MSMC_RAM_SIZE_SR2);

	pruss_release_mem_region(prueth->pruss, &prueth->shram);

	pruss_put(prueth->pruss);

	if (prueth->eth_node[PRUETH_MAC1])
		prueth_put_cores(prueth, ICSS_SLICE1);

	if (prueth->eth_node[PRUETH_MAC0])
		prueth_put_cores(prueth, ICSS_SLICE0);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int prueth_suspend(struct device *dev)
{
	struct prueth *prueth = dev_get_drvdata(dev);
	struct net_device *ndev;
	int i, ret;

	for (i = 0; i < PRUETH_NUM_MACS; i++) {
		ndev = prueth->registered_netdevs[i];

		if (!ndev)
			continue;

		if (netif_running(ndev)) {
			netif_device_detach(ndev);
			ret = emac_ndo_stop(ndev);
			if (ret < 0) {
				netdev_err(ndev, "failed to stop: %d", ret);
				return ret;
			}
		}
	}

	return 0;
}

static int prueth_resume(struct device *dev)
{
	struct prueth *prueth = dev_get_drvdata(dev);
	struct net_device *ndev;
	int i, ret;

	for (i = 0; i < PRUETH_NUM_MACS; i++) {
		ndev = prueth->registered_netdevs[i];

		if (!ndev)
			continue;

		if (netif_running(ndev)) {
			ret = emac_ndo_open(ndev);
			if (ret < 0) {
				netdev_err(ndev, "failed to start: %d", ret);
				return ret;
			}
			netif_device_attach(ndev);
		}
	}

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops prueth_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(prueth_suspend, prueth_resume)
};

static const struct prueth_pdata am654_icssg_pdata_sr1 = {
	.fdqring_mode = K3_RINGACC_RING_MODE_MESSAGE,
};

static const struct prueth_pdata am654_icssg_pdata = {
	.fdqring_mode = K3_RINGACC_RING_MODE_MESSAGE,
	.quirk_10m_link_issue = 1,
	.switch_mode = 1,
};

static const struct prueth_pdata am64x_icssg_pdata = {
	.fdqring_mode = K3_RINGACC_RING_MODE_RING,
};

static const struct of_device_id prueth_dt_match[] = {
	{ .compatible = "ti,am654-icssg-prueth-sr1", .data = &am654_icssg_pdata_sr1 },
	{ .compatible = "ti,am654-icssg-prueth", .data = &am654_icssg_pdata },
	{ .compatible = "ti,am642-icssg-prueth", .data = &am64x_icssg_pdata },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, prueth_dt_match);

static struct platform_driver prueth_driver = {
	.probe = prueth_probe,
	.remove = prueth_remove,
	.driver = {
		.name = "icssg-prueth",
		.of_match_table = prueth_dt_match,
		.pm = &prueth_dev_pm_ops,
	},
};
module_platform_driver(prueth_driver);

MODULE_AUTHOR("Roger Quadros <rogerq@ti.com>");
MODULE_DESCRIPTION("PRUSS ICSSG Ethernet Driver");
MODULE_LICENSE("GPL v2");
