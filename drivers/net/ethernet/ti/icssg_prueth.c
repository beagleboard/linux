// SPDX-License-Identifier: GPL-2.0
/* Texas Instruments ICSSG Ethernet Driver
 *
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 *
 */

#include <linux/etherdevice.h>
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
#include <linux/soc/ti/k3-navss-desc-pool.h>

#include "icssg_prueth.h"

#define PRUETH_MODULE_VERSION "0.1"
#define PRUETH_MODULE_DESCRIPTION "PRUSS ICSSG Ethernet driver"

/* Port queue size in MSMC from firmware
 * PORTQSZ_HP .set (0x1800)
 * PORTQSZ_HP2 .set (PORTQSZ_HP+128) ;include barrier area
 * 0x1880 x 8 bytes per slice  (port)
 */

#define MSMC_RAM_SIZE	(SZ_64K + SZ_32K + SZ_2K)	/* 0x1880 x 8 x 2 */

#define PRUETH_NAV_PS_DATA_SIZE	0	/* Protocol specific data size */
#define PRUETH_NAV_SW_DATA_SIZE	16	/* SW related data size */
#define PRUETH_MAX_TX_DESC	512
#define PRUETH_MAX_RX_DESC	512

#define PRUETH_MIN_PKT_SIZE	(VLAN_ETH_ZLEN)
#define PRUETH_MAX_PKT_SIZE	(VLAN_ETH_FRAME_LEN + ETH_FCS_LEN)

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

static int debug_level = -1;
module_param(debug_level, int, 0644);
MODULE_PARM_DESC(debug_level, "PRUETH debug level (NETIF_MSG bits)");

static void prueth_cleanup_rx_chns(struct prueth_emac *emac)
{
	struct prueth_rx_chn *rx_chn = &emac->rx_chns;

	if (rx_chn->irq)
		k3_nav_udmax_rx_put_irq(rx_chn->rx_chn, 0);

	if (rx_chn->rx_chn)
		k3_nav_udmax_release_rx_chn(rx_chn->rx_chn);

	if (rx_chn->desc_pool)
		k3_knav_pool_destroy(rx_chn->desc_pool);
}

static void prueth_cleanup_tx_chns(struct prueth_emac *emac)
{
	struct prueth_tx_chn *tx_chn = &emac->tx_chns;

	if (tx_chn->irq)
		k3_nav_udmax_tx_put_irq(tx_chn->tx_chn);

	if (tx_chn->tx_chn)
		k3_nav_udmax_release_tx_chn(tx_chn->tx_chn);

	if (tx_chn->desc_pool)
		k3_knav_pool_destroy(tx_chn->desc_pool);
}

static int prueth_init_tx_chns(struct prueth_emac *emac)
{
	struct net_device *ndev = emac->ndev;
	struct device *dev = emac->prueth->dev;
	struct k3_nav_udmax_tx_channel_cfg tx_cfg;
	static const struct k3_ring_cfg ring_cfg = {
		.elm_size = K3_RINGACC_RING_ELSIZE_8,
		.mode = K3_RINGACC_RING_MODE_RING,
		.flags = 0,
		.size = PRUETH_MAX_TX_DESC,
	};
	u32 hdesc_size;
	int ret, slice;
	struct prueth_tx_chn *tx_chn = &emac->tx_chns;
	char tx_chn_name[4];

	slice = prueth_emac_slice(emac);
	if (slice < 0)
		return slice;

	init_completion(&emac->tdown_complete);

	hdesc_size = cppi5_hdesc_calc_size(false, PRUETH_NAV_PS_DATA_SIZE,
					   PRUETH_NAV_SW_DATA_SIZE);
	memset(&tx_cfg, 0, sizeof(tx_cfg));
	tx_cfg.swdata_size = PRUETH_NAV_SW_DATA_SIZE;
	tx_cfg.tx_cfg = ring_cfg;
	tx_cfg.txcq_cfg = ring_cfg;

	/* To differentiate channels for SLICE0 vs SLICE1 */
	snprintf(tx_chn_name, sizeof(tx_chn_name), "tx%d", slice);

	tx_chn->descs_num = PRUETH_MAX_TX_DESC;
	spin_lock_init(&tx_chn->lock);
	tx_chn->desc_pool = k3_knav_pool_create_name(dev, tx_chn->descs_num,
						     hdesc_size, tx_chn_name);
	if (IS_ERR(tx_chn->desc_pool)) {
		ret = PTR_ERR(tx_chn->desc_pool);
		tx_chn->desc_pool = NULL;
		netdev_err(ndev, "Failed to create tx pool: %d\n", ret);
		goto fail;
	}

	tx_chn->tx_chn = k3_nav_udmax_request_tx_chn(dev, tx_chn_name, &tx_cfg);
	if (IS_ERR(tx_chn->tx_chn)) {
		ret = PTR_ERR(tx_chn->tx_chn);
		tx_chn->tx_chn = NULL;
		netdev_err(ndev, "Failed to request tx dma ch: %d\n", ret);
		goto fail;
	}

	ret = k3_nav_udmax_tx_get_irq(tx_chn->tx_chn, &tx_chn->irq,
				      IRQF_TRIGGER_HIGH, false, NULL);
	if (ret) {
		tx_chn->irq = 0;
		netdev_err(ndev, "failed to get tx irq\n");
		goto fail;
	}

	return 0;

fail:
	prueth_cleanup_tx_chns(emac);
	return ret;
}

static int prueth_init_rx_chns(struct prueth_emac *emac)
{
	struct net_device *ndev = emac->ndev;
	struct device *dev = emac->prueth->dev;
	struct k3_nav_udmax_rx_channel_cfg rx_cfg;
	static struct k3_ring_cfg rxring_cfg = {
		.elm_size = K3_RINGACC_RING_ELSIZE_8,
		.mode = K3_RINGACC_RING_MODE_RING,
		.flags = 0,
		.size = PRUETH_MAX_RX_DESC,
	};
	static struct k3_ring_cfg fdqring_cfg = {
		.elm_size = K3_RINGACC_RING_ELSIZE_8,
		.mode = K3_RINGACC_RING_MODE_MESSAGE,
		.flags = 0,
		.size = PRUETH_MAX_RX_DESC,
	};
	struct k3_nav_udmax_rx_flow_cfg rx_flow_cfg = {
		.rx_cfg = rxring_cfg,
		.rxfdq_cfg = fdqring_cfg,
		.ring_rxq_id = K3_RINGACC_RING_ID_ANY,
		.ring_rxfdq0_id = K3_RINGACC_RING_ID_ANY,
		.src_tag_lo_sel = K3_NAV_UDMAX_SRC_TAG_LO_KEEP,
	};

	u32 hdesc_size;
	int ret, slice;
	struct prueth_rx_chn	*rx_chn = &emac->rx_chns;
	char rx_chn_name[16];

	slice = prueth_emac_slice(emac);
	if (slice < 0)
		return slice;

	/* To differentiate channels for SLICE0 vs SLICE1 */
	snprintf(rx_chn_name, sizeof(rx_chn_name), "rx%d", slice);

	hdesc_size = cppi5_hdesc_calc_size(false, PRUETH_NAV_PS_DATA_SIZE,
					   PRUETH_NAV_SW_DATA_SIZE);
	memset(&rx_cfg, 0, sizeof(rx_cfg));
	rx_cfg.swdata_size = PRUETH_NAV_SW_DATA_SIZE;
	rx_cfg.flow_id_num = 1;		/* only 1 flow id used */
	rx_cfg.flow_id_base = -1;	/* udmax will auto select flow id base */
	rx_cfg.def_flow_cfg = &rx_flow_cfg;

	/* init all flows */
	rx_chn->descs_num = PRUETH_MAX_RX_DESC;
	spin_lock_init(&rx_chn->lock);
	rx_chn->desc_pool = k3_knav_pool_create_name(dev, rx_chn->descs_num,
						     hdesc_size, rx_chn_name);
	if (IS_ERR(rx_chn->desc_pool)) {
		ret = PTR_ERR(rx_chn->desc_pool);
		rx_chn->desc_pool = NULL;
		netdev_err(ndev, "Failed to create rx pool: %d\n", ret);
		goto fail;
	}

	rx_chn->rx_chn = k3_nav_udmax_request_rx_chn(dev, rx_chn_name, &rx_cfg);
	if (IS_ERR(rx_chn->rx_chn)) {
		ret = PTR_ERR(rx_chn->rx_chn);
		rx_chn->rx_chn = NULL;
		netdev_err(ndev, "Failed to request rx dma ch: %d\n", ret);
		goto fail;
	}

	emac->rx_flow_id_base = k3_nav_udmax_rx_get_flow_id_base(rx_chn->rx_chn);

	ret = k3_nav_udmax_rx_get_irq(rx_chn->rx_chn, 0, &rx_chn->irq,
				      IRQF_TRIGGER_HIGH, false, -1);
	if (ret) {
		rx_chn->irq = 0;
		netdev_err(ndev, "failed to get rx irq number\n");
		goto fail;
	}

	return 0;

fail:
	prueth_cleanup_rx_chns(emac);
	return ret;
}

static int prueth_dma_rx_push(struct prueth_emac *emac,
			      struct sk_buff *skb)
{
	struct cppi5_host_desc_t *desc_rx;
	struct prueth_rx_chn *rx_chn = &emac->rx_chns;
	struct device *dev = emac->prueth->dev;
	struct net_device *ndev = emac->ndev;
	dma_addr_t desc_dma;
	dma_addr_t buf_dma;
	u32 pkt_len = skb_tailroom(skb);
	void **swdata;

	desc_rx = k3_knav_pool_alloc(rx_chn->desc_pool);
	if (!desc_rx) {
		netdev_err(ndev, "rx push: failed to allocate descriptor\n");
		return -ENOMEM;
	}
	desc_dma = k3_knav_pool_virt2dma(rx_chn->desc_pool, desc_rx);

	buf_dma = dma_map_single(dev, skb->data, pkt_len, DMA_FROM_DEVICE);
	if (unlikely(dma_mapping_error(dev, buf_dma))) {
		k3_knav_pool_free(rx_chn->desc_pool, desc_rx);
		netdev_err(ndev, "rx push: failed to map rx pkt buffer\n");
		return -EINVAL;
	}

	cppi5_hdesc_init(desc_rx, 0, PRUETH_NAV_PS_DATA_SIZE);
	cppi5_hdesc_attach_buf(desc_rx, 0, 0, buf_dma, skb_tailroom(skb));

	swdata = cppi5_hdesc_get_swdata(desc_rx);
	*swdata = skb;

	return k3_nav_udmax_push_rx_chn(rx_chn->rx_chn, 0 /* flow num */,
					desc_rx, desc_dma);
}

/**
 * emac_rx_packet - Get one packet from RX ring and push to netdev.
 * Returns 0 on success, else error code.
 */
static int emac_rx_packet(struct prueth_emac *emac)
{
	struct prueth_rx_chn *rx_chn = &emac->rx_chns;
	struct device *dev = emac->prueth->dev;
	struct net_device *ndev = emac->ndev;
	struct cppi5_host_desc_t *desc_rx;
	dma_addr_t desc_dma, buf_dma;
	u32 buf_dma_len, pkt_len, port_id = 0;
	int ret;
	void **swdata;
	struct sk_buff *skb, *new_skb;

	ret = k3_nav_udmax_pop_rx_chn(rx_chn->rx_chn, 0, &desc_dma);
	if (ret) {
		if (ret != -ENODATA)
			netdev_err(ndev, "rx pop: failed: %d\n", ret);
		return ret;
	}

	if (desc_dma & 0x1) /* Teardown ? */
		return 0;

	desc_rx = k3_knav_pool_dma2virt(rx_chn->desc_pool, desc_dma);

	swdata = cppi5_hdesc_get_swdata(desc_rx);
	skb = *swdata;
	cppi5_hdesc_get_obuf(desc_rx, &buf_dma, &buf_dma_len);
	pkt_len = cppi5_hdesc_get_pktlen(desc_rx);
	/* firmware adds 4 CRC bytes, strip them */
	pkt_len -= 4;
	cppi5_desc_get_tags_ids(&desc_rx->hdr, &port_id, NULL);

	dma_unmap_single(dev, buf_dma, buf_dma_len, DMA_FROM_DEVICE);
	k3_knav_pool_free(rx_chn->desc_pool, desc_rx);

	skb->dev = ndev;
	if (!netif_running(skb->dev)) {
		dev_kfree_skb_any(skb);
		return -ENODEV;
	}

	new_skb = netdev_alloc_skb_ip_align(ndev, PRUETH_MAX_PKT_SIZE);
	/* if allocation fails we drop the packet but push the
	 * descriptor back to the ring with old skb to prevent a stall
	 */
	if (!new_skb) {
		ndev->stats.rx_dropped++;
		new_skb = skb;
	} else {
		/* send the filled skb up the n/w stack */
		skb_put(skb, pkt_len);
		skb->protocol = eth_type_trans(skb, ndev);
		netif_receive_skb(skb);
		ndev->stats.rx_bytes += pkt_len;
		ndev->stats.rx_packets++;
	}

	/* queue another RX DMA */
	ret = prueth_dma_rx_push(emac, new_skb);
	if (WARN_ON(ret < 0))
		dev_kfree_skb_any(new_skb);

	return ret;
}

static void prueth_rx_cleanup(void *data, dma_addr_t desc_dma)
{
	struct prueth_emac *emac = data;
	struct prueth_rx_chn *rx_chn = &emac->rx_chns;
	struct cppi5_host_desc_t *desc_rx;
	struct sk_buff *skb;
	dma_addr_t buf_dma;
	u32 buf_dma_len;
	void **swdata;

	desc_rx = k3_knav_pool_dma2virt(rx_chn->desc_pool, desc_dma);
	swdata = cppi5_hdesc_get_swdata(desc_rx);
	skb = *swdata;
	cppi5_hdesc_get_obuf(desc_rx, &buf_dma, &buf_dma_len);

	dma_unmap_single(emac->prueth->dev, buf_dma, buf_dma_len,
			 DMA_FROM_DEVICE);
	k3_knav_pool_free(rx_chn->desc_pool, desc_rx);

	dev_kfree_skb_any(skb);
}

static void prueth_xmit_free(struct prueth_tx_chn *tx_chn,
			     struct device *dev,
			     struct cppi5_host_desc_t *desc)
{
	struct cppi5_host_desc_t *first_desc, *next_desc;
	dma_addr_t buf_dma, next_desc_dma;
	u32 buf_dma_len;

	first_desc = desc;
	next_desc = first_desc;

	cppi5_hdesc_get_obuf(first_desc, &buf_dma, &buf_dma_len);

	dma_unmap_single(dev, buf_dma, buf_dma_len,
			 DMA_TO_DEVICE);

	next_desc_dma = cppi5_hdesc_get_next_hbdesc(first_desc);
	while (next_desc_dma) {
		next_desc = k3_knav_pool_dma2virt(tx_chn->desc_pool,
						  next_desc_dma);
		cppi5_hdesc_get_obuf(next_desc, &buf_dma, &buf_dma_len);

		dma_unmap_page(dev, buf_dma, buf_dma_len,
			       DMA_TO_DEVICE);

		next_desc_dma = cppi5_hdesc_get_next_hbdesc(next_desc);

		k3_knav_pool_free(tx_chn->desc_pool, next_desc);
	}

	k3_knav_pool_free(tx_chn->desc_pool, first_desc);
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
	struct prueth_emac *emac = netdev_priv(ndev);
	int ret = 0;
	struct device *dev = emac->prueth->dev;
	struct cppi5_host_desc_t *first_desc, *next_desc, *cur_desc;
	struct prueth_tx_chn *tx_chn;
	dma_addr_t desc_dma, buf_dma;
	u32 pkt_len;
	int i;
	void **swdata;

	/* frag list based linkage is not supported for now. */
	if (skb_shinfo(skb)->frag_list) {
		dev_err_ratelimited(dev, "NETIF_F_FRAGLIST not supported\n");
		ret = -EINVAL;
		goto drop_free_skb;
	}

	pkt_len = skb_headlen(skb);
	tx_chn = &emac->tx_chns;

	/* Map the linear buffer */
	buf_dma = dma_map_single(dev, skb->data, pkt_len, DMA_TO_DEVICE);
	if (dma_mapping_error(dev, buf_dma)) {
		netdev_err(ndev, "tx: failed to map skb buffer\n");
		ret = -EINVAL;
		goto drop_stop_q;
	}

	first_desc = k3_knav_pool_alloc(tx_chn->desc_pool);
	if (!first_desc) {
		dev_err(dev, "tx: failed to allocate descriptor\n");
		dma_unmap_single(dev, buf_dma, pkt_len, DMA_TO_DEVICE);
		ret = -ENOMEM;
		goto drop_stop_q;
	}

	cppi5_hdesc_init(first_desc, 0, PRUETH_NAV_PS_DATA_SIZE);
	cppi5_hdesc_attach_buf(first_desc, buf_dma, pkt_len, buf_dma, pkt_len);
	swdata = cppi5_hdesc_get_swdata(first_desc);
	*swdata = skb;

	if (!skb_is_nonlinear(skb))
		goto tx_push;

	/* Handle the case where skb is fragmented in pages */
	cur_desc = first_desc;
	for (i = 0; i < skb_shinfo(skb)->nr_frags; i++) {
		skb_frag_t *frag = &skb_shinfo(skb)->frags[i];
		u32 frag_size = skb_frag_size(frag);

		next_desc = k3_knav_pool_alloc(tx_chn->desc_pool);
		if (!next_desc) {
			netdev_err(ndev,
				   "tx: failed to allocate frag. descriptor\n");
			ret = -ENOMEM;
			goto drop_free_descs;
		}

		buf_dma = skb_frag_dma_map(dev, frag, 0, frag_size,
					   DMA_TO_DEVICE);
		if (dma_mapping_error(dev, buf_dma)) {
			netdev_err(ndev, "tx: Failed to map skb page\n");
			k3_knav_pool_free(tx_chn->desc_pool, next_desc);
			ret = -EINVAL;
			goto drop_free_descs;
		}

		cppi5_hdesc_reset_hbdesc(next_desc);
		cppi5_hdesc_attach_buf(next_desc,
				       buf_dma, frag_size, buf_dma, frag_size);

		desc_dma = k3_knav_pool_virt2dma(tx_chn->desc_pool, next_desc);
		cppi5_hdesc_link_hbdesc(cur_desc, desc_dma);

		pkt_len += frag_size;
		cur_desc = next_desc;
	}
	WARN_ON(pkt_len != skb->len);

tx_push:
	skb_tx_timestamp(skb);

	/* report bql before sending packet */
	netdev_sent_queue(ndev, pkt_len);

	cppi5_hdesc_set_pktlen(first_desc, pkt_len);
	desc_dma = k3_knav_pool_virt2dma(tx_chn->desc_pool, first_desc);
	/* cppi5_desc_dump(first_desc, 64); */

	ret = k3_nav_udmax_push_tx_chn(tx_chn->tx_chn, first_desc, desc_dma);
	if (ret) {
		netdev_err(ndev, "tx: push failed: %d\n", ret);
		goto drop_free_descs;
	}

	if (k3_knav_pool_avail(tx_chn->desc_pool) < MAX_SKB_FRAGS)
		netif_stop_queue(ndev);

	return NETDEV_TX_OK;

drop_free_descs:
	prueth_xmit_free(tx_chn, dev, first_desc);
drop_stop_q:
	netif_stop_queue(ndev);
drop_free_skb:
	dev_kfree_skb_any(skb);

	/* error */
	ndev->stats.tx_dropped++;
	netdev_err(ndev, "tx: error: %d\n", ret);

	return ret;
}

/**
 * emac_tx_complete_packets - Check if TX completed packets upto budget.
 * Returns number of completed TX packets.
 */
static int emac_tx_complete_packets(struct prueth_emac *emac, int budget)
{
	struct net_device *ndev = emac->ndev;
	struct cppi5_host_desc_t *desc_tx;
	struct device *dev = emac->prueth->dev;
	struct prueth_tx_chn *tx_chn;
	unsigned int total_bytes = 0;
	struct sk_buff *skb;
	dma_addr_t desc_dma;
	int res, num_tx = 0;
	void **swdata;

	tx_chn = &emac->tx_chns;

	while (budget--) {
		res = k3_nav_udmax_pop_tx_chn(tx_chn->tx_chn, &desc_dma);
		if (res == -ENODATA)
			break;

		/* teardown completion */
		if (desc_dma & 0x1) {
			complete(&emac->tdown_complete);
			break;
		}

		desc_tx = k3_knav_pool_dma2virt(tx_chn->desc_pool, desc_dma);
		swdata = cppi5_hdesc_get_swdata(desc_tx);
		skb = *(swdata);
		prueth_xmit_free(tx_chn, dev, desc_tx);

		ndev = skb->dev;
		ndev->stats.tx_packets++;
		ndev->stats.tx_bytes += skb->len;
		total_bytes += skb->len;
		napi_consume_skb(skb, budget);
		num_tx++;
	}

	if (!num_tx)
		return 0;

	netdev_completed_queue(ndev, num_tx, total_bytes);

	if (netif_queue_stopped(ndev)) {
		/* If the the TX queue was stopped, wake it now
		 * if we have enough room.
		 */
		netif_tx_lock(ndev);
		if (netif_running(ndev) &&
		    (k3_knav_pool_avail(tx_chn->desc_pool) >= MAX_SKB_FRAGS))
			netif_wake_queue(ndev);
		netif_tx_unlock(ndev);
	}

	return num_tx;
}

static void prueth_tx_cleanup(void *data, dma_addr_t desc_dma)
{
	struct prueth_emac *emac = data;
	struct prueth_tx_chn *tx_chn = &emac->tx_chns;
	struct cppi5_host_desc_t *desc_tx;
	struct sk_buff *skb;
	void **swdata;

	desc_tx = k3_knav_pool_dma2virt(tx_chn->desc_pool, desc_dma);
	swdata = cppi5_hdesc_get_swdata(desc_tx);
	skb = *(swdata);
	prueth_xmit_free(tx_chn, emac->prueth->dev, desc_tx);

	dev_kfree_skb_any(skb);
}

static irqreturn_t prueth_rx_irq(int irq, void *dev_id)
{
	struct prueth_emac *emac = dev_id;

	disable_irq_nosync(irq);
	napi_schedule(&emac->napi_rx);

	return IRQ_HANDLED;
}

static irqreturn_t prueth_tx_irq(int irq, void *dev_id)
{
	struct prueth_emac *emac = dev_id;

	disable_irq_nosync(irq);
	napi_schedule(&emac->napi_tx);

	return IRQ_HANDLED;
}

#define CONFIG_LENGTH 16

static int prueth_emac_start(struct prueth *prueth, struct prueth_emac *emac)
{
	struct device *dev = prueth->dev;
	int slice, ret;
	u32 config[CONFIG_LENGTH];
	u32 cmd;

	switch (emac->port_id) {
	case PRUETH_PORT_MII0:
		slice = ICSS_SLICE0;
		break;
	case PRUETH_PORT_MII1:
		slice = ICSS_SLICE1;
		break;
	default:
		netdev_err(emac->ndev, "invalid port\n");
		return -EINVAL;
	}

	ret = rproc_boot(prueth->pru[slice]);
	if (ret) {
		dev_err(dev, "failed to boot PRU%d: %d\n", slice, ret);
		return -EINVAL;
	}

	ret = rproc_boot(prueth->rtu[slice]);
	if (ret) {
		dev_err(dev, "failed to boot RTU%d: %d\n", slice, ret);
		goto halt_pru;
	}

	mdelay(2);	/* FW can take aobut 1ms */
	if (!icss_hs_is_fw_ready(prueth, slice)) {
		dev_err(dev, "slice %d: firmware not ready\n", slice);
		ret = -EIO;
		goto halt_rtu;
	}

	/* configure fw */
	memset(config, 0, sizeof(u32) * CONFIG_LENGTH);
	config[0] = 0;	/* number of packets to send/receive : indefinite */
	config[1] = lower_32_bits(prueth->msmcram.pa);
	config[2] = upper_32_bits(prueth->msmcram.pa);
	config[3] = emac->rx_flow_id_base; /* flow id for host port */
	config[4] = 0;	/* promisc, multicast, etc done by classifier */
	config[5] = 0;	/* future use */
	dev_info(dev, "setting rx flow id %d\n", config[3]);

	cmd = ICSS_CMD_SET_RUN;
	ret = icss_hs_send_cmd_wait_done(prueth, slice, cmd, config, 6);
	if (ret)
		goto err;

	/* send RXTX cmd */
	cmd = ICSS_CMD_RXTX;
	ret = icss_hs_send_cmd(prueth, slice, cmd, 0, 0);
	if (ret)
		goto err;

	return 0;

err:
	dev_err(dev, "slice %d: cmd %d failed: %d\n",
		slice, cmd, ret);
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

	rproc_shutdown(prueth->rtu[slice]);
	rproc_shutdown(prueth->pru[slice]);
}

/* called back by PHY layer if there is change in link state of hw port*/
static void emac_adjust_link(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct phy_device *phydev = emac->phydev;
	unsigned long flags;
	bool new_state = false;

	spin_lock_irqsave(&emac->lock, flags);

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

		/* f/w should support 10, 100 & 1000 */
		emac->speed = SPEED_1000;

		/* half duplex may not be supported by f/w */
		emac->duplex = DUPLEX_FULL;
	}

	/* FIXME: Do we need to update PHY status to Firmware? */

	if (new_state)
		phy_print_status(phydev);

	if (emac->link) {
		/* link ON */
		netif_carrier_on(ndev);
		/* reactivate the transmit queue */
		netif_tx_wake_all_queues(ndev);
	} else {
		/* link OFF */
		netif_carrier_off(ndev);
		netif_tx_stop_all_queues(ndev);
	}

	spin_unlock_irqrestore(&emac->lock, flags);
}

static int emac_napi_rx_poll(struct napi_struct *napi_rx, int budget)
{
	struct prueth_emac *emac = prueth_napi_to_emac(napi_rx);
	int num_rx_packets = 0;

	while (num_rx_packets < budget) {
		if (emac_rx_packet(emac))
			break;
		num_rx_packets++;
	}

	if (num_rx_packets < budget) {
		napi_complete(napi_rx);
		enable_irq(emac->rx_chns.irq);
	}

	return num_rx_packets;
}

static int emac_napi_tx_poll(struct napi_struct *napi_tx, int budget)
{
	struct prueth_emac *emac = prueth_napi_to_emac(napi_tx);
	int num_tx_packets;

	num_tx_packets = emac_tx_complete_packets(emac, budget);

	if (num_tx_packets < budget) {
		napi_complete(napi_tx);
		enable_irq(emac->tx_chns.irq);
	}

	return num_tx_packets;
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
	struct prueth *prueth = emac->prueth;
	struct device *dev = prueth->dev;
	int ret, i;
	struct sk_buff *skb;
	int slice = prueth_emac_slice(emac);

	/* clear SMEM of this slice */
	memset_io(prueth->shram.va + slice * ICSS_HS_OFFSET_SLICE1,
		  0, ICSS_HS_OFFSET_SLICE1);
	/* set h/w MAC as user might have re-configured */
	ether_addr_copy(emac->mac_addr, ndev->dev_addr);

	icssg_class_set_mac_addr(prueth->miig_rt, slice, emac->mac_addr);
	icssg_class_default(prueth->miig_rt, slice);

	netif_carrier_off(ndev);

	ret = prueth_init_tx_chns(emac);
	if (ret) {
		dev_err(dev, "failed to init tx channel: %d\n", ret);
		return ret;
	}

	ret = prueth_init_rx_chns(emac);
	if (ret) {
		dev_err(dev, "failed to init rx channel: %d\n", ret);
		goto cleanup_tx;
	}

	ret = request_irq(emac->tx_chns.irq, prueth_tx_irq, 0,
			  dev_name(dev), emac);
	if (ret) {
		dev_err(dev, "unable to request TX IRQ\n");
		goto cleanup_rx;
	}

	ret = request_irq(emac->rx_chns.irq, prueth_rx_irq, 0,
			  dev_name(dev), emac);
	if (ret) {
		dev_err(dev, "unable to request RX IRQ\n");
		goto free_tx_irq;
	}

	/* reset and start PRU firmware */
	ret = prueth_emac_start(prueth, emac);
	if (ret)
		goto free_rx_irq;

	/* start PHY */
	phy_start(emac->phydev);

	/* prepare RX & TX */
	for (i = 0; i < emac->rx_chns.descs_num; i++) {
		skb = __netdev_alloc_skb_ip_align(NULL,
						  PRUETH_MAX_PKT_SIZE,
						  GFP_KERNEL);
		if (!skb) {
			netdev_err(ndev, "cannot allocate skb\n");
			ret = -ENOMEM;
			goto err;
		}

		ret = prueth_dma_rx_push(emac, skb);
		if (ret < 0) {
			netdev_err(ndev, "cannot submit skb for rx: %d\n",
				   ret);
			kfree_skb(skb);
			goto err;
		}
	}

	k3_nav_udmax_enable_rx_chn(emac->rx_chns.rx_chn);
	k3_nav_udmax_enable_tx_chn(emac->tx_chns.tx_chn);

	napi_enable(&emac->napi_tx);
	napi_enable(&emac->napi_rx);

	if (netif_msg_drv(emac))
		dev_notice(&ndev->dev, "started\n");

	return 0;

err:
	prueth_emac_stop(emac);
free_rx_irq:
	free_irq(emac->rx_chns.irq, emac);
free_tx_irq:
	free_irq(emac->tx_chns.irq, emac);
cleanup_rx:
	prueth_cleanup_rx_chns(emac);
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
	int ret;
	int slice = prueth_emac_slice(emac);

	/* inform the upper layers. */
	netif_stop_queue(ndev);

	/* block packets from wire */
	icssg_class_disable(prueth->miig_rt, prueth_emac_slice(emac));

	/* tear down and disable UDMA channels */
	reinit_completion(&emac->tdown_complete);
	k3_nav_udmax_tdown_tx_chn(emac->tx_chns.tx_chn, false);
	ret = wait_for_completion_timeout(&emac->tdown_complete,
			msecs_to_jiffies(1000));
	if (!ret)
		netdev_err(ndev, "tx teardown timeout\n");

	k3_nav_udmax_reset_tx_chn(emac->tx_chns.tx_chn,
				  emac,
				  prueth_tx_cleanup);
	k3_nav_udmax_disable_tx_chn(emac->tx_chns.tx_chn);

	ret = icss_hs_send_cmd(prueth, slice, ICSS_HS_CMD_CANCEL, 0, 0);
	if (ret)
		netdev_err(ndev, "CANCEL failed: %d\n", ret);

	k3_nav_udmax_tdown_rx_chn(emac->rx_chns.rx_chn, true);
	k3_nav_udmax_reset_rx_chn(emac->rx_chns.rx_chn, 0, emac,
				  prueth_rx_cleanup, 0);
	k3_nav_udmax_disable_rx_chn(emac->rx_chns.rx_chn);

	napi_disable(&emac->napi_tx);
	napi_disable(&emac->napi_rx);

	/* stop PHY */
	phy_stop(emac->phydev);

	/* stop PRUs */
	prueth_emac_stop(emac);

	free_irq(emac->rx_chns.irq, emac);
	free_irq(emac->tx_chns.irq, emac);

	prueth_cleanup_rx_chns(emac);
	prueth_cleanup_tx_chns(emac);

	if (netif_msg_drv(emac))
		dev_notice(&ndev->dev, "stopped\n");

	return 0;
}

/**
 * emac_ndo_tx_timeout - EMAC Transmit timeout function
 * @ndev: The EMAC network adapter
 *
 * Called when system detects that a skb timeout period has expired
 * potentially due to a fault in the adapter in not being able to send
 * it out on the wire.
 */
static void emac_ndo_tx_timeout(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);

	if (netif_msg_tx_err(emac))
		netdev_err(ndev, "xmit timeout");

	ndev->stats.tx_errors++;

	/* TODO: can we recover or need to reboot firmware? */
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
	int slice = prueth_emac_slice(emac);

	if (ndev->flags & IFF_PROMISC) {
		/* enable promiscuous */
		if (!(emac->flags & IFF_PROMISC)) {
			icssg_class_promiscuous(prueth->miig_rt, slice);
			emac->flags |= IFF_PROMISC;
		}
		return;
	} else if (ndev->flags & IFF_ALLMULTI) {
		/* TODO: enable all multicast */
	} else {
		if (emac->flags & IFF_PROMISC) {
			/* local MAC + BC only */
			icssg_class_default(prueth->miig_rt, slice);
			emac->flags &= ~IFF_PROMISC;
		}

		/* TODO: specific multi */
	}
}

static const struct net_device_ops emac_netdev_ops = {
	.ndo_open = emac_ndo_open,
	.ndo_stop = emac_ndo_stop,
	.ndo_start_xmit = emac_ndo_start_xmit,
	.ndo_set_mac_address = eth_mac_addr,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_change_mtu	= eth_change_mtu,
	.ndo_tx_timeout = emac_ndo_tx_timeout,
	.ndo_set_rx_mode = emac_ndo_set_rx_mode,
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

extern const struct ethtool_ops icssg_ethtool_ops;

static int prueth_netdev_init(struct prueth *prueth,
			      struct device_node *eth_node)
{
	enum prueth_port port;
	enum prueth_mac mac;
	struct net_device *ndev;
	struct prueth_emac *emac;
	const u8 *mac_addr;
	int ret;

	port = prueth_node_port(eth_node);
	if (port < 0)
		return -EINVAL;

	mac = prueth_node_mac(eth_node);
	if (mac < 0)
		return -EINVAL;

	ndev = alloc_etherdev(sizeof(*emac));
	if (!ndev)
		return -ENOMEM;

	SET_NETDEV_DEV(ndev, prueth->dev);
	emac = netdev_priv(ndev);
	prueth->emac[mac] = emac;
	emac->prueth = prueth;
	emac->ndev = ndev;
	emac->port_id = port;
	emac->msg_enable = netif_msg_init(debug_level, PRUETH_EMAC_DEBUG);
	spin_lock_init(&emac->lock);

	emac->phy_node = of_parse_phandle(eth_node, "phy-handle", 0);
	if (!emac->phy_node) {
		dev_err(prueth->dev, "couldn't find phy-handle\n");
		ret = -ENODEV;
		goto free;
	}

	if (of_phy_is_fixed_link(emac->phy_node)) {
		ret = of_phy_register_fixed_link(emac->phy_node);
		if (ret) {
			if (ret != -EPROBE_DEFER) {
				dev_err(prueth->dev,
					"failed to register fixed-link phy: %d\n",
					ret);
			}

			goto free;
		}
	}

	emac->phy_if = of_get_phy_mode(eth_node);
	if (emac->phy_if < 0) {
		dev_err(prueth->dev, "could not get phy-mode property\n");
		ret = emac->phy_if;
		goto free;
	}

	/* connect PHY */
	emac->phydev = of_phy_connect(ndev, emac->phy_node,
				      &emac_adjust_link, 0, emac->phy_if);
	if (!emac->phydev) {
		dev_dbg(prueth->dev, "couldn't connect to phy %s\n",
			emac->phy_node->full_name);
		ret = -EPROBE_DEFER;
		goto free;
	}

	/* get mac address from DT and set private and netdev addr */
	mac_addr = of_get_mac_address(eth_node);
	if (mac_addr)
		ether_addr_copy(ndev->dev_addr, mac_addr);
	if (!is_valid_ether_addr(ndev->dev_addr)) {
		eth_hw_addr_random(ndev);
		dev_warn(prueth->dev, "port %d: using random MAC addr: %pM\n",
			 port, ndev->dev_addr);
	}
	ether_addr_copy(emac->mac_addr, ndev->dev_addr);

	ndev->netdev_ops = &emac_netdev_ops;
	ndev->ethtool_ops = &icssg_ethtool_ops;

	netif_tx_napi_add(ndev, &emac->napi_tx,
			  emac_napi_tx_poll, NAPI_POLL_WEIGHT);
	netif_tx_napi_add(ndev, &emac->napi_rx,
			  emac_napi_rx_poll, NAPI_POLL_WEIGHT);

	return 0;

free:
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

	phy_disconnect(emac->phydev);

	if (of_phy_is_fixed_link(emac->phy_node))
		of_phy_deregister_fixed_link(emac->phy_node);

	netif_napi_del(&emac->napi_rx);
	netif_napi_del(&emac->napi_tx);
	free_netdev(emac->ndev);
	prueth->emac[mac] = NULL;
}

static int prueth_get_cores(struct prueth *prueth, int slice)
{
	struct device *dev = prueth->dev;
	struct device_node *np = dev->of_node;
	int pru, rtu, ret;

	switch (slice) {
	case ICSS_SLICE0:
		pru = 0;
		rtu = 1;
		break;
	case ICSS_SLICE1:
		pru = 2;
		rtu = 3;
		break;
	default:
		return -EINVAL;
	}

	prueth->pru[slice] = pru_rproc_get(np, pru);
	if (IS_ERR(prueth->pru[slice])) {
		ret = PTR_ERR(prueth->pru[slice]);
		prueth->pru[slice] = NULL;
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "unable to get PRU%d: %d\n", slice, ret);
		return ret;
	}

	prueth->rtu[slice] = pru_rproc_get(np, rtu);
	if (IS_ERR(prueth->rtu[slice])) {
		ret = PTR_ERR(prueth->rtu[slice]);
		prueth->rtu[slice] = NULL;
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "unable to get RTU%d: %d\n", slice, ret);
		return ret;
	}

	return 0;
}

static void prueth_put_cores(struct prueth *prueth, int slice)
{
	if (prueth->rtu[slice])
		pru_rproc_put(prueth->rtu[slice]);

	if (prueth->pru[slice])
		pru_rproc_put(prueth->pru[slice]);
}

static int prueth_config_rgmiidelay(struct prueth *prueth,
				    struct device_node *eth_np)
{
	struct device *dev = prueth->dev;
	struct regmap *ctrl_mmr;
	u32 icssgctrl;
	u32 val;
	struct device_node *np = dev->of_node;

	if (!of_device_is_compatible(np, "ti,am654-icssg-prueth"))
		return 0;

	ctrl_mmr = syscon_regmap_lookup_by_phandle(eth_np, "syscon-rgmii-delay");
	if (IS_ERR(ctrl_mmr)) {
		dev_err(dev, "couldn't get syscon-rgmii-delay\n");
		return -ENODEV;
	}

	if (of_property_read_u32_index(eth_np, "syscon-rgmii-delay", 1,
				       &icssgctrl)) {
		dev_err(dev, "couldn't get rgmii-delay reg. offset\n");
		return -ENODEV;
	}

	if (of_property_read_bool(eth_np, "enable-rgmii-delay"))
		val = 0;
	else
		val = ICSSG_CTRL_RGMII_ID_MODE;

	regmap_update_bits(ctrl_mmr, icssgctrl, ICSSG_CTRL_RGMII_ID_MODE, val);

	return 0;
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

	if (!np)
		return -ENODEV;	/* we don't support non DT */

	match = of_match_device(prueth_dt_match, dev);
	if (!match)
		return -ENODEV;

	prueth = devm_kzalloc(dev, sizeof(*prueth), GFP_KERNEL);
	if (!prueth)
		return -ENOMEM;

	platform_set_drvdata(pdev, prueth);

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

	if (eth0_node) {
		ret = prueth_config_rgmiidelay(prueth, eth0_node);
		if (ret)
			goto put_cores;

		ret = prueth_get_cores(prueth, ICSS_SLICE0);
		if (ret)
			goto put_cores;
	}

	if (eth1_node) {
		ret = prueth_config_rgmiidelay(prueth, eth1_node);
		if (ret)
			goto put_cores;

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
	prueth->msmcram.va =
			(void __iomem *)gen_pool_alloc(prueth->sram_pool,
						       MSMC_RAM_SIZE);
	if (!prueth->msmcram.va) {
		ret = -ENOMEM;
		dev_err(dev, "unable to allocate MSMC resource\n");
		goto put_mem;
	}
	prueth->msmcram.pa = gen_pool_virt_to_phys(prueth->sram_pool,
						   (unsigned long)prueth->msmcram.va);
	prueth->msmcram.size = MSMC_RAM_SIZE;
	dev_dbg(dev, "sram: pa %pa va %p size %zx\n", &prueth->msmcram.pa,
		prueth->msmcram.va, prueth->msmcram.size);

	/* setup netdev interfaces */
	if (eth0_node) {
		ret = prueth_netdev_init(prueth, eth0_node);
		if (ret) {
			if (ret != -EPROBE_DEFER) {
				dev_err(dev, "netdev init %s failed: %d\n",
					eth0_node->name, ret);
			}
			goto free_pool;
		}
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
	}

	/* register the network devices */
	if (eth0_node) {
		ret = register_netdev(prueth->emac[PRUETH_MAC0]->ndev);
		if (ret) {
			dev_err(dev, "can't register netdev for port MII0");
			goto netdev_exit;
		}

		prueth->registered_netdevs[PRUETH_MAC0] = prueth->emac[PRUETH_MAC0]->ndev;
	}

	if (eth1_node) {
		ret = register_netdev(prueth->emac[PRUETH_MAC1]->ndev);
		if (ret) {
			dev_err(dev, "can't register netdev for port MII1");
			goto netdev_unregister;
		}

		prueth->registered_netdevs[PRUETH_MAC1] = prueth->emac[PRUETH_MAC1]->ndev;
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

free_pool:
	gen_pool_free(prueth->sram_pool,
		      (unsigned long)prueth->msmcram.va, MSMC_RAM_SIZE);

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

	for (i = 0; i < PRUETH_NUM_MACS; i++) {
		if (!prueth->registered_netdevs[i])
			continue;
		unregister_netdev(prueth->registered_netdevs[i]);
	}

	for (i = 0; i < PRUETH_NUM_MACS; i++) {
		eth_node = prueth->eth_node[i];
		if (!eth_node)
			continue;

		prueth_netdev_exit(prueth, eth_node);
	}

	gen_pool_free(prueth->sram_pool,
		      (unsigned long)prueth->msmcram.va,
		      MSMC_RAM_SIZE);

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

static const struct of_device_id prueth_dt_match[] = {
	{ .compatible = "ti,am654-icssg-prueth", },
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
