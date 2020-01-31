// SPDX-License-Identifier: GPL-2.0
/* Texas Instruments K3 J721 Virt Ethernet Switch MAC Driver
 *
 * Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
 */

#include <linux/etherdevice.h>
#include <linux/if_vlan.h>
#include <linux/interrupt.h>
#include <linux/inetdevice.h>
#include <linux/kernel.h>
#include <linux/kmemleak.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/dma/ti-cppi5.h>
#include <linux/dma/k3-udma-glue.h>
#include <linux/rpmsg-remotedev/rpmsg-remotedev.h>
#include <linux/soc/ti/k3-navss-desc-pool.h>

#define VIRT_CPSW_DRV_VER "0.1"

#define VIRT_CPSW_MAX_TX_QUEUES	1
#define VIRT_CPSW_MAX_RX_QUEUES	1
#define VIRT_CPSW_MAX_RX_FLOWS	1

#define VIRT_CPSW_MIN_PACKET_SIZE	ETH_ZLEN
#define VIRT_CPSW_MAX_PACKET_SIZE	(VLAN_ETH_FRAME_LEN + ETH_FCS_LEN)

/* Number of TX/RX descriptors */
#define VIRT_CPSW_MAX_TX_DESC	256
#define VIRT_CPSW_MAX_RX_DESC	256

#define VIRT_CPSW_NAV_PS_DATA_SIZE 16
#define VIRT_CPSW_NAV_SW_DATA_SIZE 16

#define VIRT_CPSW_DRV_NAME "j721e-cpsw-virt-mac"

struct virt_cpsw_tx_chn {
	struct device *dev;
	struct k3_knav_desc_pool *desc_pool;
	struct k3_udma_glue_tx_channel *tx_chn;
	u32 descs_num;
	unsigned int irq;
	u32 id;
};

struct virt_cpsw_rx_chn {
	struct device *dev;
	struct k3_knav_desc_pool *desc_pool;
	struct k3_udma_glue_rx_channel *rx_chn;
	u32 descs_num;
	unsigned int irq;
};

struct virt_cpsw_port {
	struct virt_cpsw_common *common;
	struct net_device *ndev;
	const char *name;
	u8 local_mac_addr[ETH_ALEN];
};

struct virt_cpsw_common {
	struct device *dev;
	struct virt_cpsw_port ports;

	struct virt_cpsw_tx_chn tx_chns;
	struct napi_struct napi_tx;
	struct completion tdown_complete;
	atomic_t tdown_cnt;
	struct virt_cpsw_rx_chn rx_chns;
	struct napi_struct napi_rx;

	const char *rdev_name;
	struct rpmsg_remotedev *rdev;
	struct rpmsg_remotedev_eth_switch_ops *rdev_switch_ops;
	u32 rdev_features;
	u32 rdev_mtu;
	u8 rdev_mac_addr[ETH_ALEN];
	u32 rdev_tx_psil_dst_id;
	u32 tx_psil_id_base;
	u32 rdev_rx_flow_id;
};

struct virt_cpsw_ndev_stats {
	u64 tx_packets;
	u64 tx_bytes;
	u64 rx_packets;
	u64 rx_bytes;
	struct u64_stats_sync syncp;
};

struct virt_cpsw_ndev_priv {
	struct virt_cpsw_ndev_stats __percpu *stats;
	struct virt_cpsw_port	*port;
};

#define virt_ndev_to_priv(ndev) \
	((struct virt_cpsw_ndev_priv *)netdev_priv(ndev))
#define virt_ndev_to_port(ndev) (virt_ndev_to_priv(ndev)->port)
#define virt_ndev_to_common(ndev) (virt_ndev_to_port(ndev)->common)

static void virt_cpsw_nuss_ndo_host_tx_timeout(struct net_device *ndev)
{
	struct virt_cpsw_common *common = virt_ndev_to_common(ndev);
	struct virt_cpsw_tx_chn *tx_chn = &common->tx_chns;
	struct netdev_queue *netif_txq;
	unsigned long trans_start;

	/* process every txq*/
	netif_txq = netdev_get_tx_queue(ndev, 0);
	trans_start = netif_txq->trans_start;
	if (netif_xmit_stopped(netif_txq) &&
	    time_after(jiffies, (trans_start + ndev->watchdog_timeo))) {
		netdev_err(ndev, "txq:%d DRV_XOFF:%d tmo:%u dql_avail:%d free_desc:%zu\n",
			   0,
			   netif_tx_queue_stopped(netif_txq),
			   jiffies_to_msecs(jiffies - trans_start),
			   dql_avail(&netif_txq->dql),
			   k3_knav_pool_avail(tx_chn->desc_pool));

		if (netif_tx_queue_stopped(netif_txq)) {
			/* try recover if stopped by us */
			txq_trans_update(netif_txq);
			netif_tx_wake_queue(netif_txq);
		}
	}
}

static int virt_cpsw_nuss_rx_push(struct virt_cpsw_common *common,
				  struct sk_buff *skb)
{
	struct cppi5_host_desc_t *desc_rx;
	struct virt_cpsw_rx_chn *rx_chn = &common->rx_chns;
	struct device *dev = common->dev;
	dma_addr_t desc_dma;
	dma_addr_t buf_dma;
	u32 pkt_len = skb_tailroom(skb);
	void *swdata;

	desc_rx = k3_knav_pool_alloc(rx_chn->desc_pool);
	if (!desc_rx) {
		dev_err(dev, "Failed to allocate RXFDQ descriptor\n");
		return -ENOMEM;
	}
	desc_dma = k3_knav_pool_virt2dma(rx_chn->desc_pool, desc_rx);

	buf_dma = dma_map_single(dev, skb->data, pkt_len, DMA_FROM_DEVICE);
	if (unlikely(dma_mapping_error(dev, buf_dma))) {
		k3_knav_pool_free(rx_chn->desc_pool, desc_rx);
		dev_err(dev, "Failed to map rx skb buffer\n");
		return -EINVAL;
	}

	cppi5_hdesc_init(desc_rx, CPPI5_INFO0_HDESC_EPIB_PRESENT,
			 VIRT_CPSW_NAV_PS_DATA_SIZE);
	cppi5_hdesc_attach_buf(desc_rx, 0, 0, buf_dma, skb_tailroom(skb));
	swdata = cppi5_hdesc_get_swdata(desc_rx);
	*((void **)swdata) = skb;

	return k3_udma_glue_push_rx_chn(rx_chn->rx_chn, 0, desc_rx, desc_dma);
}

static int virt_cpsw_nuss_common_open(struct virt_cpsw_common *common,
				      netdev_features_t features)
{
	struct sk_buff *skb;
	int i, ret;

	for (i = 0; i < common->rx_chns.descs_num; i++) {
		skb = __netdev_alloc_skb_ip_align(NULL,
						  VIRT_CPSW_MAX_PACKET_SIZE,
						  GFP_KERNEL);
		if (!skb) {
			dev_err(common->dev, "cannot allocate skb\n");
			return -ENOMEM;
		}

		ret = virt_cpsw_nuss_rx_push(common, skb);
		if (ret < 0) {
			dev_err(common->dev,
				"cannot submit skb to channel rx, error %d\n",
				ret);
			kfree_skb(skb);
			return ret;
		}
		kmemleak_not_leak(skb);
	}
	ret = k3_udma_glue_rx_flow_enable(common->rx_chns.rx_chn, 0);
	if (ret)
		return ret;

	ret = k3_udma_glue_enable_tx_chn(common->tx_chns.tx_chn);
	if (ret)
		return ret;

	napi_enable(&common->napi_tx);
	napi_enable(&common->napi_rx);

	return 0;
}

static void virt_cpsw_nuss_tx_cleanup(void *data, dma_addr_t desc_dma);
static void virt_cpsw_nuss_rx_cleanup(void *data, dma_addr_t desc_dma);

static void virt_cpsw_nuss_common_stop(struct virt_cpsw_common *common)
{
	int i;

	/* shutdown tx channels */
	atomic_set(&common->tdown_cnt, VIRT_CPSW_MAX_TX_QUEUES);
	/* ensure new tdown_cnt value is visible */
	smp_mb__after_atomic();
	reinit_completion(&common->tdown_complete);

	k3_udma_glue_tdown_tx_chn(common->tx_chns.tx_chn, false);

	i = wait_for_completion_timeout(&common->tdown_complete,
					msecs_to_jiffies(1000));
	if (!i)
		dev_err(common->dev, "tx teardown timeout\n");

	k3_udma_glue_reset_tx_chn(common->tx_chns.tx_chn,
				  &common->tx_chns,
				  virt_cpsw_nuss_tx_cleanup);
	k3_udma_glue_disable_tx_chn(common->tx_chns.tx_chn);
	napi_disable(&common->napi_tx);

	k3_udma_glue_rx_flow_disable(common->rx_chns.rx_chn, 0);
	/* Need some delay to process RX ring before reset */
	msleep(100);
	k3_udma_glue_reset_rx_chn(common->rx_chns.rx_chn, 0,
				  &common->rx_chns,
				  virt_cpsw_nuss_rx_cleanup, false);
	napi_disable(&common->napi_rx);
}

static int virt_cpsw_nuss_ndo_stop(struct net_device *ndev)
{
	struct virt_cpsw_common *common = virt_ndev_to_common(ndev);
	struct rpmsg_remotedev_eth_switch_ops *rdev_ops;
	struct device *dev = common->dev;
	int ret;

	rdev_ops = common->rdev_switch_ops;
	netif_tx_stop_all_queues(ndev);
	netif_carrier_off(ndev);

	ret = rdev_ops->unregister_mac(common->rdev, ndev->dev_addr,
				       common->rdev_rx_flow_id);
	if (ret)
		dev_err(dev, "unregister_mac rpmsg - fail %d\n", ret);

	virt_cpsw_nuss_common_stop(common);

	dev_info(common->dev, "virt_cpsw_nuss mac stopped\n");
	return 0;
}

static int virt_cpsw_nuss_ndo_open(struct net_device *ndev)
{
	struct virt_cpsw_common *common = virt_ndev_to_common(ndev);
	struct rpmsg_remotedev_eth_switch_ops *rdev_ops;
	struct device *dev = common->dev;
	int ret;

	rdev_ops = common->rdev_switch_ops;
	netdev_tx_reset_queue(netdev_get_tx_queue(ndev, 0));

	ret = virt_cpsw_nuss_common_open(common, ndev->features);
	if (ret)
		return ret;

	ret = rdev_ops->register_mac(common->rdev,
				     ndev->dev_addr,
				     common->rdev_rx_flow_id);
	if (ret) {
		dev_err(dev, "register_mac rpmsg - fail %d\n", ret);
		virt_cpsw_nuss_common_stop(common);
		return ret;
	}

	netif_tx_wake_all_queues(ndev);
	netif_carrier_on(ndev);

	dev_info(common->dev, "virt_cpsw_nuss mac started\n");
	return 0;
}

static void virt_cpsw_nuss_rx_cleanup(void *data, dma_addr_t desc_dma)
{
	struct virt_cpsw_rx_chn *rx_chn = data;
	struct cppi5_host_desc_t *desc_rx;
	struct sk_buff *skb;
	dma_addr_t buf_dma;
	u32 buf_dma_len;
	void **swdata;

	desc_rx = k3_knav_pool_dma2virt(rx_chn->desc_pool, desc_dma);
	swdata = cppi5_hdesc_get_swdata(desc_rx);
	skb = *swdata;
	cppi5_hdesc_get_obuf(desc_rx, &buf_dma, &buf_dma_len);

	dma_unmap_single(rx_chn->dev, buf_dma, buf_dma_len, DMA_FROM_DEVICE);
	k3_knav_pool_free(rx_chn->desc_pool, desc_rx);

	dev_kfree_skb_any(skb);
}

/* RX psdata[2] word format - checksum information */
#define AM65_CPSW_RX_PSD_CSUM_ADD	GENMASK(15, 0)
#define AM65_CPSW_RX_PSD_CSUM_ERR	BIT(16)
#define AM65_CPSW_RX_PSD_IS_FRAGMENT	BIT(17)
#define AM65_CPSW_RX_PSD_IS_TCP		BIT(18)
#define AM65_CPSW_RX_PSD_IPV6_VALID	BIT(19)
#define AM65_CPSW_RX_PSD_IPV4_VALID	BIT(20)

static void virt_cpsw_nuss_rx_csum(struct sk_buff *skb, u32 csum_info)
{
	/* HW can verify IPv4/IPv6 TCP/UDP packets checksum
	 * csum information provides in psdata[2] word:
	 * AM65_CPSW_RX_PSD_CSUM_ERR bit - indicates csum error
	 * AM65_CPSW_RX_PSD_IPV6_VALID and AM65_CPSW_RX_PSD_IPV4_VALID
	 * bits - indicates IPv4/IPv6 packet
	 * AM65_CPSW_RX_PSD_IS_FRAGMENT bit - indicates fragmented packet
	 * AM65_CPSW_RX_PSD_CSUM_ADD has value 0xFFFF for non fragmented packets
	 * or csum value for fragmented packets if !AM65_CPSW_RX_PSD_CSUM_ERR
	 */
	skb_checksum_none_assert(skb);

	if (unlikely(!(skb->dev->features & NETIF_F_RXCSUM)))
		return;

	if ((csum_info & (AM65_CPSW_RX_PSD_IPV6_VALID |
			  AM65_CPSW_RX_PSD_IPV4_VALID)) &&
			  !(csum_info & AM65_CPSW_RX_PSD_CSUM_ERR)) {
		if (csum_info & AM65_CPSW_RX_PSD_IS_FRAGMENT) {
			skb->ip_summed = CHECKSUM_COMPLETE;
			skb->csum = csum_unfold(csum_info &
						AM65_CPSW_RX_PSD_CSUM_ADD);
		} else {
			skb->ip_summed = CHECKSUM_UNNECESSARY;
		}
	}
}

static int virt_cpsw_nuss_rx_packets(struct virt_cpsw_common *common,
				     u32 flow_idx)
{
	struct virt_cpsw_rx_chn *rx_chn = &common->rx_chns;
	struct device *dev = common->dev;
	struct virt_cpsw_ndev_priv *ndev_priv;
	struct virt_cpsw_ndev_stats *stats;
	struct net_device *ndev;
	struct cppi5_host_desc_t *desc_rx;
	struct sk_buff *skb, *new_skb;
	dma_addr_t desc_dma, buf_dma;
	u32 buf_dma_len, pkt_len, port_id = 0, csum_info;
	int ret = 0;
	void **swdata;
	u32 *psdata;

	ret = k3_udma_glue_pop_rx_chn(rx_chn->rx_chn, flow_idx, &desc_dma);
	if (ret) {
		if (ret != -ENODATA)
			dev_err(dev, "RX: pop chn fail %d\n", ret);
		return ret;
	}

	if (desc_dma & 0x1) {
		dev_dbg(dev, "%s RX tdown flow: %u\n", __func__, flow_idx);
		return 0;
	}

	desc_rx = k3_knav_pool_dma2virt(rx_chn->desc_pool, desc_dma);
	dev_dbg(dev, "%s flow_idx: %u desc %pad\n",
		__func__, flow_idx, &desc_dma);

	swdata = cppi5_hdesc_get_swdata(desc_rx);
	skb = *swdata;
	cppi5_hdesc_get_obuf(desc_rx, &buf_dma, &buf_dma_len);
	pkt_len = cppi5_hdesc_get_pktlen(desc_rx);
	cppi5_desc_get_tags_ids(&desc_rx->hdr, &port_id, NULL);
	/* read port for dbg */
	dev_dbg(dev, "%s rx port_id:%d\n", __func__, port_id);
	ndev = common->ports.ndev;
	skb->dev = ndev;

	psdata = cppi5_hdesc_get_psdata32(desc_rx);
	csum_info = psdata[2];
	dev_dbg(dev, "%s rx csum_info:%#x\n", __func__, csum_info);

	dma_unmap_single(dev, buf_dma, buf_dma_len, DMA_FROM_DEVICE);

	k3_knav_pool_free(rx_chn->desc_pool, desc_rx);

	if (unlikely(!netif_running(skb->dev))) {
		dev_kfree_skb_any(skb);
		return -ENODEV;
	}

	new_skb = netdev_alloc_skb_ip_align(ndev, VIRT_CPSW_MAX_PACKET_SIZE);
	if (new_skb) {
		skb_put(skb, pkt_len);
		skb->protocol = eth_type_trans(skb, ndev);
		virt_cpsw_nuss_rx_csum(skb, csum_info);
		napi_gro_receive(&common->napi_rx, skb);

		ndev_priv = netdev_priv(ndev);
		stats = this_cpu_ptr(ndev_priv->stats);

		u64_stats_update_begin(&stats->syncp);
		stats->rx_packets++;
		stats->rx_bytes += pkt_len;
		u64_stats_update_end(&stats->syncp);
		kmemleak_not_leak(new_skb);
	} else {
		ndev->stats.rx_dropped++;
		new_skb = skb;
	}

	if (netif_dormant(ndev)) {
		dev_kfree_skb_any(new_skb);
		ndev->stats.rx_dropped++;
		return -ENODEV;
	}

	ret = virt_cpsw_nuss_rx_push(common, new_skb);
	if (WARN_ON(ret < 0)) {
		dev_kfree_skb_any(new_skb);
		ndev->stats.rx_errors++;
		ndev->stats.rx_dropped++;
	}

	return ret;
}

static int virt_cpsw_nuss_rx_poll(struct napi_struct *napi_rx, int budget)
{
	struct virt_cpsw_common *common =
			container_of(napi_rx, struct virt_cpsw_common, napi_rx);
	int num_rx = 0;
	int cur_budget;
	int ret;

	/* process every flow */
	cur_budget = budget;

	while (cur_budget--) {
		ret = virt_cpsw_nuss_rx_packets(common, 0);
		if (ret)
			break;
		num_rx++;
	}

	dev_dbg(common->dev, "%s num_rx:%d %d\n", __func__, num_rx, budget);

	if (num_rx < budget && napi_complete_done(napi_rx, num_rx))
		enable_irq(common->rx_chns.irq);

	return num_rx;
}

static void virt_cpsw_nuss_xmit_free(struct virt_cpsw_tx_chn *tx_chn,
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

static void virt_cpsw_nuss_tx_cleanup(void *data, dma_addr_t desc_dma)
{
	struct virt_cpsw_tx_chn *tx_chn = data;
	struct cppi5_host_desc_t *desc_tx;
	struct sk_buff *skb;
	void **swdata;

	desc_tx = k3_knav_pool_dma2virt(tx_chn->desc_pool, desc_dma);
	swdata = cppi5_hdesc_get_swdata(desc_tx);
	skb = *(swdata);
	virt_cpsw_nuss_xmit_free(tx_chn, tx_chn->dev, desc_tx);

	dev_kfree_skb_any(skb);
}

static int virt_cpsw_nuss_tx_compl_packets(struct virt_cpsw_common *common,
					   int chn, unsigned int budget)
{
	struct cppi5_host_desc_t *desc_tx;
	struct device *dev = common->dev;
	struct netdev_queue *netif_txq;
	struct virt_cpsw_tx_chn *tx_chn;
	struct net_device *ndev;
	unsigned int total_bytes = 0;
	struct sk_buff *skb;
	dma_addr_t desc_dma;
	int res, num_tx = 0;
	void **swdata;

	tx_chn = &common->tx_chns;

	while (budget--) {
		struct virt_cpsw_ndev_priv *ndev_priv;
		struct virt_cpsw_ndev_stats *stats;

		res = k3_udma_glue_pop_tx_chn(tx_chn->tx_chn, &desc_dma);
		if (res == -ENODATA)
			break;

		if (desc_dma & 0x1) {
			if (atomic_dec_and_test(&common->tdown_cnt))
				complete(&common->tdown_complete);
			break;
		}

		desc_tx = k3_knav_pool_dma2virt(tx_chn->desc_pool, desc_dma);
		swdata = cppi5_hdesc_get_swdata(desc_tx);
		skb = *(swdata);
		virt_cpsw_nuss_xmit_free(tx_chn, dev, desc_tx);

		ndev = skb->dev;

		ndev_priv = netdev_priv(ndev);
		stats = this_cpu_ptr(ndev_priv->stats);
		u64_stats_update_begin(&stats->syncp);
		stats->tx_packets++;
		stats->tx_bytes += skb->len;
		u64_stats_update_end(&stats->syncp);

		total_bytes += skb->len;
		napi_consume_skb(skb, budget);
		num_tx++;
	}

	if (!num_tx)
		return 0;

	netif_txq = netdev_get_tx_queue(ndev, 0);

	netdev_tx_completed_queue(netif_txq, num_tx, total_bytes);
	dev_dbg(dev, "compl 0 %d Bytes\n", total_bytes);

	if (netif_tx_queue_stopped(netif_txq)) {
		/* Check whether the queue is stopped due to stalled tx dma,
		 * if the queue is stopped then wake the queue as
		 * we have free desc for tx
		 */
		__netif_tx_lock(netif_txq, smp_processor_id());
		if (netif_running(ndev) &&
		    (k3_knav_pool_avail(tx_chn->desc_pool) >= MAX_SKB_FRAGS))
			netif_tx_wake_queue(netif_txq);

		__netif_tx_unlock(netif_txq);
	}
	dev_dbg(dev, "%s:%u pkt:%d\n", __func__, chn, num_tx);

	return num_tx;
}

static int virt_cpsw_nuss_tx_poll(struct napi_struct *napi_tx, int budget)
{
	struct virt_cpsw_common *common =
			container_of(napi_tx, struct virt_cpsw_common, napi_tx);
	int num_tx;

	/* process every unprocessed channel */
	num_tx = virt_cpsw_nuss_tx_compl_packets(common, 0, budget);

	if (num_tx < budget) {
		napi_complete(napi_tx);
		enable_irq(common->tx_chns.irq);
	}

	return num_tx;
}

static irqreturn_t virt_cpsw_nuss_rx_irq(int irq, void *dev_id)
{
	struct virt_cpsw_common *common = dev_id;

	disable_irq_nosync(irq);
	napi_schedule(&common->napi_rx);

	return IRQ_HANDLED;
}

static irqreturn_t virt_cpsw_nuss_tx_irq(int irq, void *dev_id)
{
	struct virt_cpsw_common *common = dev_id;

	disable_irq_nosync(irq);
	napi_schedule(&common->napi_tx);

	return IRQ_HANDLED;
}

static netdev_tx_t virt_cpsw_nuss_ndo_xmit(struct sk_buff *skb,
					   struct net_device *ndev)
{
	struct virt_cpsw_common *common = virt_ndev_to_common(ndev);
	struct device *dev = common->dev;
	struct cppi5_host_desc_t *first_desc, *next_desc, *cur_desc;
	struct virt_cpsw_tx_chn *tx_chn;
	struct netdev_queue *netif_txq;
	dma_addr_t desc_dma, buf_dma;
	int ret, i;
	u32 pkt_len;
	void **swdata;
	u32 *psdata;

	/* frag list based linkage is not supported for now. */
	if (skb_shinfo(skb)->frag_list) {
		dev_err_ratelimited(dev, "NETIF_F_FRAGLIST not supported\n");
		ret = -EINVAL;
		goto drop_free_skb;
	}

	/* padding enabled in hw */
	pkt_len = skb_headlen(skb);

	tx_chn = &common->tx_chns;
	netif_txq = netdev_get_tx_queue(ndev, 0);

	/* Map the linear buffer */
	buf_dma = dma_map_single(dev, skb->data, pkt_len,
				 DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(dev, buf_dma))) {
		dev_err(dev, "Failed to map tx skb buffer\n");
		ret = -EINVAL;
		ndev->stats.tx_errors++;
		goto drop_stop_q;
	}

	first_desc = k3_knav_pool_alloc(tx_chn->desc_pool);
	if (!first_desc) {
		dev_dbg(dev, "Failed to allocate descriptor\n");
		dma_unmap_single(dev, buf_dma, pkt_len, DMA_TO_DEVICE);
		goto drop_stop_q_busy;
	}

	cppi5_hdesc_init(first_desc, CPPI5_INFO0_HDESC_EPIB_PRESENT,
			 VIRT_CPSW_NAV_PS_DATA_SIZE);
	cppi5_desc_set_pktids(&first_desc->hdr, 0, 0x3FFF);
	cppi5_hdesc_set_pkttype(first_desc, 0x7);
	/* target port has to be 0 */
	cppi5_desc_set_tags_ids(&first_desc->hdr, 0, 0);

	cppi5_hdesc_attach_buf(first_desc, buf_dma, pkt_len, buf_dma, pkt_len);
	swdata = cppi5_hdesc_get_swdata(first_desc);
	*(swdata) = skb;
	psdata = cppi5_hdesc_get_psdata32(first_desc);

	/* HW csum offload if enabled */
	psdata[2] = 0;
	if (likely(skb->ip_summed == CHECKSUM_PARTIAL)) {
		unsigned int cs_start, cs_offset;

		cs_start = skb_transport_offset(skb);
		cs_offset = cs_start + skb->csum_offset;
		/* HW numerates bytes starting from 1 */
		psdata[2] = ((cs_offset + 1) << 24) |
			    ((cs_start + 1) << 16) | (skb->len - cs_start);
		dev_dbg(dev, "%s tx psdata:%#x\n", __func__, psdata[2]);
	}

	if (!skb_is_nonlinear(skb))
		goto done_tx;

	dev_dbg(dev, "fragmented SKB\n");

	/* Handle the case where skb is fragmented in pages */
	cur_desc = first_desc;
	for (i = 0; i < skb_shinfo(skb)->nr_frags; i++) {
		skb_frag_t *frag = &skb_shinfo(skb)->frags[i];
		u32 frag_size = skb_frag_size(frag);

		next_desc = k3_knav_pool_alloc(tx_chn->desc_pool);

		if (!next_desc) {
			dev_err(dev, "Failed to allocate descriptor\n");
			ret = -ENOMEM;
			goto drop_free_descs;
		}

		buf_dma = skb_frag_dma_map(dev, frag, 0, frag_size,
					   DMA_TO_DEVICE);
		if (unlikely(dma_mapping_error(dev, buf_dma))) {
			dev_err(dev, "Failed to map tx skb page\n");
			k3_knav_pool_free(tx_chn->desc_pool, next_desc);
			ret = -EINVAL;
			ndev->stats.tx_errors++;
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

done_tx:
	skb_tx_timestamp(skb);

	/* report bql before sending packet */
	dev_dbg(dev, "push 0 %d Bytes\n", pkt_len);

	netdev_tx_sent_queue(netif_txq, pkt_len);

	cppi5_hdesc_set_pktlen(first_desc, pkt_len);
	desc_dma = k3_knav_pool_virt2dma(tx_chn->desc_pool, first_desc);
	ret = k3_udma_glue_push_tx_chn(tx_chn->tx_chn, first_desc, desc_dma);
	if (ret) {
		dev_err(dev, "can't push desc %d\n", ret);
		ndev->stats.tx_errors++;
		goto drop_free_descs;
	}

	if (k3_knav_pool_avail(tx_chn->desc_pool) < MAX_SKB_FRAGS) {
		netif_tx_stop_queue(netif_txq);
		/* Barrier, so that stop_queue visible to other cpus */
		smp_mb__after_atomic();
		dev_err(dev, "netif_tx_stop_queue %d\n", 0);

		/* re-check for smp */
		if (k3_knav_pool_avail(tx_chn->desc_pool) >= MAX_SKB_FRAGS) {
			netif_tx_wake_queue(netif_txq);
			dev_err(dev, "netif_tx_wake_queue %d\n", 0);
		}
	}

	return NETDEV_TX_OK;

drop_free_descs:
	virt_cpsw_nuss_xmit_free(tx_chn, dev, first_desc);
drop_stop_q:
	netif_tx_stop_queue(netif_txq);
drop_free_skb:
	ndev->stats.tx_dropped++;
	dev_kfree_skb_any(skb);
	return ret;

drop_stop_q_busy:
	netif_tx_stop_queue(netif_txq);
	return NETDEV_TX_BUSY;
}

static void virt_cpsw_nuss_ndo_get_stats(struct net_device *dev,
					 struct rtnl_link_stats64 *stats)
{
	struct virt_cpsw_ndev_priv *ndev_priv = netdev_priv(dev);
	unsigned int start;
	int cpu;

	for_each_possible_cpu(cpu) {
		struct virt_cpsw_ndev_stats *cpu_stats;
		u64 rx_packets;
		u64 rx_bytes;
		u64 tx_packets;
		u64 tx_bytes;

		cpu_stats = per_cpu_ptr(ndev_priv->stats, cpu);
		do {
			start = u64_stats_fetch_begin_irq(&cpu_stats->syncp);
			rx_packets = cpu_stats->rx_packets;
			rx_bytes   = cpu_stats->rx_bytes;
			tx_packets = cpu_stats->tx_packets;
			tx_bytes   = cpu_stats->tx_bytes;
		} while (u64_stats_fetch_retry_irq(&cpu_stats->syncp, start));

		stats->rx_packets += rx_packets;
		stats->rx_bytes   += rx_bytes;
		stats->tx_packets += tx_packets;
		stats->tx_bytes   += tx_bytes;
	}

	stats->rx_errors	= dev->stats.rx_errors;
	stats->rx_dropped	= dev->stats.rx_dropped;
	stats->tx_dropped	= dev->stats.tx_dropped;
}

static const struct net_device_ops virt_cpsw_nuss_netdev_ops = {
	.ndo_open		= virt_cpsw_nuss_ndo_open,
	.ndo_stop		= virt_cpsw_nuss_ndo_stop,
	.ndo_start_xmit		= virt_cpsw_nuss_ndo_xmit,
	.ndo_get_stats64        = virt_cpsw_nuss_ndo_get_stats,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_tx_timeout		= virt_cpsw_nuss_ndo_host_tx_timeout,
};

static void virt_cpsw_nuss_get_drvinfo(struct net_device *ndev,
				       struct ethtool_drvinfo *info)
{
	struct virt_cpsw_common *common = virt_ndev_to_common(ndev);
	struct rpmsg_remotedev_eth_switch_ops *rdev_ops;
	char fw_version[ETHTOOL_FWVERS_LEN];

	rdev_ops = common->rdev_switch_ops;

	strlcpy(info->driver, dev_driver_string(common->dev),
		sizeof(info->driver));
	strlcpy(info->version, VIRT_CPSW_DRV_VER, sizeof(info->version));
	strlcpy(info->bus_info, dev_name(common->dev), sizeof(info->bus_info));

	rdev_ops->get_fw_ver(common->rdev, fw_version, ETHTOOL_FWVERS_LEN);
	strlcpy(info->fw_version, fw_version, ETHTOOL_FWVERS_LEN);
}

static const char virt_cpsw_nuss_ethtool_priv_flags[][ETH_GSTRING_LEN] = {
	"RPMSG Ping test",
	"RPMSG Read reg",
	"RPMSG Dump stat",
};

static int
virt_cpsw_nuss_get_sset_count(struct net_device __always_unused *ndev, int sset)
{
	switch (sset) {
	case ETH_SS_TEST:
		return ARRAY_SIZE(virt_cpsw_nuss_ethtool_priv_flags);
	default:
		return -EOPNOTSUPP;
	}
}

static void
virt_cpsw_nuss_get_strings(struct net_device __always_unused *ndev,
			   u32 stringset, u8 *data)
{
	switch (stringset) {
	case ETH_SS_TEST:
		memcpy(data, virt_cpsw_nuss_ethtool_priv_flags,
		       sizeof(virt_cpsw_nuss_ethtool_priv_flags));
		break;
	}
}

static void virt_cpsw_nuss_self_test(struct net_device *ndev,
				     struct ethtool_test *eth_test, u64 *data)
{
	struct virt_cpsw_common *common = virt_ndev_to_common(ndev);
	struct device *dev = common->dev;
	static const char ping_data[] = "0123456789";
	u32 reg_val;
	int ret;

	data[0] = 0;
	ret = common->rdev_switch_ops->ping(common->rdev,
					    ping_data, strlen(ping_data));
	if (ret) {
		dev_err(dev, "rpmsg ping fail %d\n", ret);
		eth_test->flags |= ETH_TEST_FL_FAILED;
		data[0] = 1;
	}

	data[1] = 0;
	ret = common->rdev_switch_ops->read_reg(common->rdev,
						0x0C000000, &reg_val);
	if (ret) {
		dev_err(dev, "rpmsg read_reg fail %d\n", ret);
		eth_test->flags |= ETH_TEST_FL_FAILED;
		data[1] = 1;
	}
	dev_dbg(dev, "read_reg rpmsg cpsw_nuss_ver - 0x0C000000:%08X\n",
		reg_val);

	ret = common->rdev_switch_ops->read_reg(common->rdev,
						0x0C020000, &reg_val);
	if (ret) {
		dev_err(dev, "rpmsg read_reg fail %d\n", ret);
		eth_test->flags |= ETH_TEST_FL_FAILED;
		data[1] = 1;
	}
	dev_dbg(dev, "read_reg rpmsg cpsw_ver - 0x0C020000:%08X\n",
		reg_val);

	ret = 0;
	data[2] = 0;
	if (common->rdev_features & RPMSG_KDRV_ETHSWITCH_FEATURE_DUMP_STATS)
		ret = common->rdev_switch_ops->dbg_dump_stats(common->rdev);
	if (ret) {
		dev_err(dev, "rpmsg dump_stats fail %d\n", ret);
		eth_test->flags |= ETH_TEST_FL_FAILED;
		data[2] = 1;
	}
}

const struct ethtool_ops virt_cpsw_nuss_ethtool_ops = {
	.get_drvinfo		= virt_cpsw_nuss_get_drvinfo,
	.get_sset_count		= virt_cpsw_nuss_get_sset_count,
	.get_strings		= virt_cpsw_nuss_get_strings,
	.self_test		= virt_cpsw_nuss_self_test,
	.get_link		= ethtool_op_get_link,
};

static void virt_cpsw_nuss_free_tx_chns(void *data)
{
	struct virt_cpsw_common *common = data;
	struct virt_cpsw_tx_chn	*tx_chn = &common->tx_chns;

	if (!IS_ERR_OR_NULL(tx_chn->tx_chn))
		k3_udma_glue_release_tx_chn(tx_chn->tx_chn);

	if (!IS_ERR_OR_NULL(tx_chn->desc_pool))
		k3_knav_pool_destroy(tx_chn->desc_pool);

	memset(tx_chn, 0, sizeof(*tx_chn));
}

static int virt_cpsw_nuss_init_tx_chns(struct virt_cpsw_common *common)
{
	u32 max_desc_num = ALIGN(VIRT_CPSW_MAX_TX_DESC, MAX_SKB_FRAGS);
	struct virt_cpsw_tx_chn	*tx_chn = &common->tx_chns;
	struct k3_udma_glue_tx_channel_cfg tx_cfg = { 0 };
	struct device *dev = common->dev;
	struct k3_ring_cfg ring_cfg = {
		.elm_size = K3_RINGACC_RING_ELSIZE_8,
		.mode = K3_RINGACC_RING_MODE_RING,
		.flags = 0
	};
	char tx_chn_name[IFNAMSIZ];
	u32 hdesc_size, tx_chn_num;
	int ret, ret1;

	/* convert to tx chn offset */
	tx_chn_num = common->rdev_tx_psil_dst_id - common->tx_psil_id_base;
	snprintf(tx_chn_name, sizeof(tx_chn_name), "tx%d", tx_chn_num);

	init_completion(&common->tdown_complete);

	hdesc_size = cppi5_hdesc_calc_size(true, VIRT_CPSW_NAV_PS_DATA_SIZE,
					   VIRT_CPSW_NAV_SW_DATA_SIZE);

	tx_cfg.swdata_size = VIRT_CPSW_NAV_SW_DATA_SIZE;
	tx_cfg.tx_cfg = ring_cfg;
	tx_cfg.txcq_cfg = ring_cfg;
	tx_cfg.tx_cfg.size = max_desc_num;
	tx_cfg.txcq_cfg.size = max_desc_num;

	tx_chn->dev = dev;
	tx_chn->id = 0;
	tx_chn->descs_num = max_desc_num;
	tx_chn->desc_pool = k3_knav_pool_create_name(dev,
						     tx_chn->descs_num,
						     hdesc_size,
						     tx_chn_name);
	if (IS_ERR(tx_chn->desc_pool)) {
		ret = PTR_ERR(tx_chn->desc_pool);
		dev_err(dev, "Failed to create poll %d\n", ret);
		goto err;
	}

	tx_chn->tx_chn = k3_udma_glue_request_tx_chn(dev, tx_chn_name, &tx_cfg);
	if (IS_ERR(tx_chn->tx_chn)) {
		ret = PTR_ERR(tx_chn->tx_chn);
		dev_err(dev, "Failed to request tx dma channel %d\n", ret);
		goto err;
	}

	tx_chn->irq = k3_udma_glue_tx_get_irq(tx_chn->tx_chn);
	if (tx_chn->irq <= 0) {
		dev_err(dev, "Failed to get tx dma irq %d\n", tx_chn->irq);
		ret = -ENXIO;
	}

err:
	ret1 = devm_add_action(dev, virt_cpsw_nuss_free_tx_chns, common);
	if (ret1) {
		dev_err(dev, "failed to add free_tx_chns action %d", ret1);
		return ret1;
	}

	return ret;
}

static void virt_cpsw_nuss_free_rx_chns(void *data)
{
	struct virt_cpsw_common *common = data;
	struct virt_cpsw_rx_chn *rx_chn = &common->rx_chns;

	if (!IS_ERR_OR_NULL(rx_chn->rx_chn))
		k3_udma_glue_release_rx_chn(rx_chn->rx_chn);

	if (!IS_ERR_OR_NULL(rx_chn->desc_pool))
		k3_knav_pool_destroy(rx_chn->desc_pool);
}

static int virt_cpsw_nuss_init_rx_chns(struct virt_cpsw_common *common)
{
	struct virt_cpsw_rx_chn *rx_chn = &common->rx_chns;
	struct k3_udma_glue_rx_channel_cfg rx_cfg = {0};
	u32  max_desc_num = VIRT_CPSW_MAX_RX_DESC;
	struct device *dev = common->dev;
	u32 hdesc_size;
	int ret = 0, ret1;
	struct k3_ring_cfg rxring_cfg = {
		.elm_size = K3_RINGACC_RING_ELSIZE_8,
		.mode = K3_RINGACC_RING_MODE_MESSAGE,
		.flags = 0,
	};
	struct k3_ring_cfg fdqring_cfg = {
		.elm_size = K3_RINGACC_RING_ELSIZE_8,
		.mode = K3_RINGACC_RING_MODE_MESSAGE,
		.flags = 0,
	};
	struct k3_udma_glue_rx_flow_cfg rx_flow_cfg = {
		.rx_cfg = rxring_cfg,
		.rxfdq_cfg = fdqring_cfg,
		.ring_rxq_id = K3_RINGACC_RING_ID_ANY,
		.ring_rxfdq0_id = K3_RINGACC_RING_ID_ANY,
		.src_tag_lo_sel = K3_UDMA_GLUE_SRC_TAG_LO_USE_REMOTE_SRC_TAG,
	};

	hdesc_size = cppi5_hdesc_calc_size(true, VIRT_CPSW_NAV_PS_DATA_SIZE,
					   VIRT_CPSW_NAV_SW_DATA_SIZE);

	rx_cfg.swdata_size = VIRT_CPSW_NAV_SW_DATA_SIZE;
	rx_cfg.flow_id_num = VIRT_CPSW_MAX_RX_FLOWS;
	rx_cfg.flow_id_base = common->rdev_rx_flow_id;
	rx_cfg.remote = true;

	/* init all flows */
	rx_chn->dev = dev;
	rx_chn->descs_num = max_desc_num;
	rx_chn->desc_pool = k3_knav_pool_create_name(dev, rx_chn->descs_num,
						     hdesc_size, "rx");
	if (IS_ERR(rx_chn->desc_pool)) {
		ret = PTR_ERR(rx_chn->desc_pool);
		dev_err(dev, "Failed to create rx poll %d\n", ret);
		goto err;
	}

	rx_chn->rx_chn = k3_udma_glue_request_rx_chn(dev, "rx", &rx_cfg);
	if (IS_ERR(rx_chn->rx_chn)) {
		ret = PTR_ERR(rx_chn->rx_chn);
		dev_err(dev, "Failed to request rx dma channel %d\n", ret);
		goto err;
	}

	common->rdev_rx_flow_id =
			k3_udma_glue_rx_get_flow_id_base(rx_chn->rx_chn);
	dev_dbg(dev, "used flow-id-base %u\n", common->rdev_rx_flow_id);

	rx_flow_cfg.rx_cfg.size = max_desc_num;
	rx_flow_cfg.rxfdq_cfg.size = max_desc_num;
	ret = k3_udma_glue_rx_flow_init(rx_chn->rx_chn,
					0, &rx_flow_cfg);
	if (ret) {
		dev_err(dev, "Failed to init rx flow%d %d\n", 0, ret);
		goto err;
	}

	rx_chn->irq = k3_udma_glue_rx_get_irq(rx_chn->rx_chn, 0);
	if (rx_chn->irq <= 0) {
		ret = -ENXIO;
		dev_err(dev, "Failed to get rx dma irq %d\n", rx_chn->irq);
	}

err:
	ret1 = devm_add_action(dev, virt_cpsw_nuss_free_rx_chns, common);
	if (ret1) {
		dev_err(dev, "failed to add free_rx_chns action %d", ret1);
		return ret1;
	}

	return ret;
}

static int virt_cpsw_nuss_of(struct virt_cpsw_common *common)
{
	struct device *dev = common->dev;
	struct device_node *port_np;
	struct virt_cpsw_port *port;
	const void *mac_addr;
	int ret;

	ret = of_property_read_u32(dev->of_node, "ti,psil-base",
				   &common->tx_psil_id_base);
	if (ret) {
		dev_err(dev, "ti,psil-base read fail %d\n", ret);
		return ret;
	}

	port_np = of_get_child_by_name(dev->of_node, "virt_emac_port");
	if (!port_np)
		return -ENOENT;

	port = &common->ports;
	port->common = common;
	port->name = of_get_property(port_np, "ti,label", NULL);

	mac_addr = of_get_mac_address(port_np);
	if (!IS_ERR(mac_addr))
		ether_addr_copy(port->local_mac_addr, mac_addr);

	of_node_put(port_np);
	return 0;
}

static int virt_cpsw_nuss_rdev_init(struct virt_cpsw_common *common)
{
	struct rpmsg_rdev_eth_switch_attach_ext_info attach_info = { 0 };
	struct device *dev = common->dev;
	int ret;

	ret = common->rdev_switch_ops->attach_ext(common->rdev, &attach_info);
	if (ret) {
		dev_err(dev, "rpmsg attach - fail %d\n", ret);
		return ret;
	}
	dev_dbg(dev, "rpmsg attach_ext - rx_mtu:%d features:%08X tx_mtu[0]:%d flow_idx:%d tx_cpsw_psil_dst_id:%d mac_addr:%pM\n",
		attach_info.rx_mtu, attach_info.features,
		attach_info.tx_mtu[0],
		attach_info.flow_idx,
		attach_info.tx_cpsw_psil_dst_id,
		attach_info.mac_addr);
	common->rdev_features = attach_info.features;
	common->rdev_mtu = VIRT_CPSW_MAX_PACKET_SIZE;
	common->rdev_tx_psil_dst_id = attach_info.tx_cpsw_psil_dst_id &
				     (~0x8000);
	common->rdev_rx_flow_id = attach_info.flow_idx;
	ether_addr_copy(common->rdev_mac_addr, attach_info.mac_addr);

	return 0;
}

static int virt_cpsw_nuss_init_ndev(struct virt_cpsw_common *common)
{
	struct virt_cpsw_ndev_priv *ndev_priv;
	struct device *dev = common->dev;
	struct virt_cpsw_port *port;
	int ret;

	port = &common->ports;

	/* alloc netdev */
	port->ndev = devm_alloc_etherdev_mqs(common->dev,
					     sizeof(struct virt_cpsw_ndev_priv),
					     1, 1);
	if (!port->ndev) {
		dev_err(dev, "error allocating net_device\n");
		return -ENOMEM;
	}

	ndev_priv = netdev_priv(port->ndev);
	ndev_priv->port = port;
	SET_NETDEV_DEV(port->ndev, dev);

	if (is_valid_ether_addr(port->local_mac_addr))
		ether_addr_copy(port->ndev->dev_addr, port->local_mac_addr);
	else if (is_valid_ether_addr(common->rdev_mac_addr))
		ether_addr_copy(port->ndev->dev_addr, common->rdev_mac_addr);

	port->ndev->min_mtu = VIRT_CPSW_MIN_PACKET_SIZE;
	port->ndev->max_mtu = VIRT_CPSW_MAX_PACKET_SIZE;
	port->ndev->hw_features = NETIF_F_SG |
				  NETIF_F_RXCSUM;
	port->ndev->features = port->ndev->hw_features;
	port->ndev->vlan_features |=  NETIF_F_SG;
	port->ndev->netdev_ops = &virt_cpsw_nuss_netdev_ops;
	port->ndev->ethtool_ops = &virt_cpsw_nuss_ethtool_ops;

	/* TX checksum offload if supported */
	if (common->rdev_features & RPMSG_KDRV_ETHSWITCH_FEATURE_TXCSUM)
		port->ndev->features |= NETIF_F_HW_CSUM;

	ndev_priv->stats = netdev_alloc_pcpu_stats(struct virt_cpsw_ndev_stats);
	if (!ndev_priv->stats)
		return -ENOMEM;

	ret = devm_add_action_or_reset(dev, (void(*)(void *))free_percpu,
				       ndev_priv->stats);
	if (ret) {
		dev_err(dev, "failed to add percpu stat free action %d", ret);
		return ret;
	}

	netif_tx_napi_add(port->ndev, &common->napi_tx,
			  virt_cpsw_nuss_tx_poll, NAPI_POLL_WEIGHT);
	netif_napi_add(port->ndev, &common->napi_rx,
		       virt_cpsw_nuss_rx_poll, NAPI_POLL_WEIGHT);

	ret = register_netdev(port->ndev);
	if (ret)
		dev_err(dev, "error registering slave net device %d\n", ret);

	/* can't auto unregister ndev using devm_add_action() due to broken
	 * devres release sequence in DD core
	 */

	return ret;
}

static void virt_cpsw_nuss_cleanup_ndev(struct virt_cpsw_common *common)
{
	if (common->ports.ndev)
		unregister_netdev(common->ports.ndev);
}

static bool virt_cpsw_dev_check(const struct net_device *ndev)
{
	return ndev->netdev_ops == &virt_cpsw_nuss_netdev_ops;
}

static int virt_cpsw_inetaddr_event(struct notifier_block *unused,
				    unsigned long event, void *ptr)
{
	struct in_ifaddr *ifa = (struct in_ifaddr *)ptr;
	struct rpmsg_remotedev_eth_switch_ops *rdev_ops;
	struct net_device *ndev = ifa->ifa_dev->dev;
	struct virt_cpsw_common *common;
	int ret = 0;

	if (!virt_cpsw_dev_check(ndev))
		goto out;

	common = virt_ndev_to_common(ndev);
	rdev_ops = common->rdev_switch_ops;
	switch (event) {
	case NETDEV_UP:
		ret = rdev_ops->register_ipv4(common->rdev,
					      ndev->dev_addr,
					      ifa->ifa_address);
		if (ret)
			dev_err(common->dev, "register_ipv4 rpmsg - fail %d\n",
				ret);
		dev_dbg(common->dev, "NETDEV_UP %pI4 %s\n",
			&ifa->ifa_address, ifa->ifa_label);
		break;

	case NETDEV_DOWN:
		ret = rdev_ops->unregister_ipv4(common->rdev,
						ifa->ifa_address);
		if (ret)
			dev_err(common->dev, "unregister_ipv4 rpmsg - fail %d\n",
				ret);
		dev_dbg(common->dev, "NETDEV_DOWN %pI4\n", &ifa->ifa_address);
		break;
	}

out:
	return notifier_from_errno(ret);
}

static struct notifier_block virt_cpsw_inetaddr_nb __read_mostly = {
	.notifier_call = virt_cpsw_inetaddr_event,
};

static const struct of_device_id virt_cpsw_virt_of_mtable[] = {
	{ .compatible = "ti,j721e-cpsw-virt-mac", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, virt_cpsw_virt_of_mtable);

static int virt_cpsw_nuss_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct virt_cpsw_common *common;
	int ret;

	common = devm_kzalloc(dev, sizeof(struct virt_cpsw_common), GFP_KERNEL);
	if (!common)
		return -ENOMEM;
	common->dev = dev;

	ret = of_property_read_string(dev->of_node, "ti,remote-name",
				      &common->rdev_name);
	if (ret < 0) {
		dev_info(dev, "remote-name is not set %d\n", ret);
		return ret;
	}

	common->rdev = rpmsg_remotedev_get_named_device(common->rdev_name);
	if (!common->rdev)
		return -EPROBE_DEFER;
	if (IS_ERR(common->rdev)) {
		ret = PTR_ERR(common->rdev);
		return ret;
	}
	common->rdev_switch_ops = common->rdev->device.eth_switch.ops;
	ret = devm_add_action_or_reset(dev,
				       (void(*)(void *))rpmsg_remotedev_put_device,
				       common->rdev);
	if (ret) {
		dev_err(dev, "add remotedev put device action fail:%d", ret);
		return ret;
	}

	ret = virt_cpsw_nuss_of(common);
	if (ret)
		return ret;

	ret = virt_cpsw_nuss_rdev_init(common);
	if (ret)
		return ret;
	/* init tx channels */
	ret = virt_cpsw_nuss_init_tx_chns(common);
	if (ret)
		return ret;
	ret = virt_cpsw_nuss_init_rx_chns(common);
	if (ret)
		return ret;

	if (common->tx_chns.irq == 0 || common->rx_chns.irq == 0)
		return -ENXIO;

	dev_set_drvdata(dev, common);

	ret = virt_cpsw_nuss_init_ndev(common);
	if (ret)
		return ret;

	ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(48));
	if (ret) {
		dev_err(dev, "error setting dma mask: %d\n", ret);
		goto unreg_ndev;
	}

	ret = devm_request_irq(dev, common->tx_chns.irq,
			       virt_cpsw_nuss_tx_irq,

			       0, dev_name(dev), common);
	if (ret) {
		dev_err(dev, "failure requesting tx irq %u, %d\n",
			common->tx_chns.irq, ret);
		goto unreg_ndev;
	}

	ret = devm_request_irq(dev, common->rx_chns.irq,
			       virt_cpsw_nuss_rx_irq,
			       0, dev_name(dev), common);
	if (ret) {
		dev_err(dev, "failure requesting rx irq %u, %d\n",
			common->rx_chns.irq, ret);
		goto unreg_ndev;
	}

	register_inetaddr_notifier(&virt_cpsw_inetaddr_nb);

	dev_info(common->dev, "virt_cpsw_nuss mac loaded\n");
	dev_info(dev, "rdev_features:%08X rdev_mtu:%d flow_id:%d tx_psil_dst_id:%04X\n",
		 common->rdev_features,
		 common->rdev_mtu,
		 common->rdev_rx_flow_id,
		 common->rdev_tx_psil_dst_id);
	dev_info(dev, "local_mac_addr:%pM rdev_mac_addr:%pM\n",
		 common->ports.local_mac_addr,
		 common->rdev_mac_addr);

	return 0;

unreg_ndev:
	virt_cpsw_nuss_cleanup_ndev(common);
	return ret;
}

static int virt_cpsw_nuss_remove(struct platform_device *pdev)
{
	struct virt_cpsw_common *common = platform_get_drvdata(pdev);
	struct device *dev = common->dev;
	int ret;

	unregister_inetaddr_notifier(&virt_cpsw_inetaddr_nb);

	/* must unregister ndevs here because DD release_driver routine calls
	 * dma_deconfigure(dev) before devres_release_all(dev)
	 */
	virt_cpsw_nuss_cleanup_ndev(common);

	ret = common->rdev_switch_ops->detach(common->rdev);
	if (ret)
		dev_err(dev, "rpmsg  detach - fail %d\n", ret);

	return 0;
}

static struct platform_driver virt_cpsw_nuss_driver = {
	.driver = {
		.name = VIRT_CPSW_DRV_NAME,
		.of_match_table = virt_cpsw_virt_of_mtable,
	},
	.probe = virt_cpsw_nuss_probe,
	.remove = virt_cpsw_nuss_remove,
};

module_platform_driver(virt_cpsw_nuss_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Grygorii Strashko <grygorii.strashko@ti.com>");
MODULE_DESCRIPTION("TI J721E VIRT CPSW Ethernet mac driver");
