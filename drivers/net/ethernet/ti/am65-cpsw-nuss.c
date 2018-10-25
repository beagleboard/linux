// SPDX-License-Identifier: GPL-2.0
/*
 * Texas Instruments K3 AM65 Ethernet Switch SubSystem Driver
 *
 * Copyright (C) 2017-2018 Texas Instruments Incorporated - http://www.ti.com/
 *
 */

#include <linux/clk.h>
#include <linux/etherdevice.h>
#include <linux/if_vlan.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/kmemleak.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/net_tstamp.h>
#include <linux/of.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/of_device.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/soc/ti/k3-navss-desc-pool.h>
#include <linux/dma/ti-cppi5.h>
#include <linux/dma/k3-navss-udma.h>
#include <linux/net_switch_config.h>

#include "cpsw_ale.h"
#include "cpsw_sl.h"
#include "cpsw.h"
#include "am65-cpsw-nuss.h"
#include "davinci_mdio_int.h"
#include "am65-cpts.h"

#define AM65_CPSW_SS_BASE	0x0
#define AM65_CPSW_SGMII_BASE	0x100
#define AM65_CPSW_MDIO_BASE	0xf00
#define AM65_CPSW_XGMII_BASE	0x2100
#define AM65_CPSW_CPSW_NU_BASE	0x20000
#define AM65_CPSW_NU_PORTS_BASE	0x1000
#define AM65_CPSW_NU_STATS_BASE	0x1a000
#define AM65_CPSW_NU_ALE_BASE	0x1e000
#define AM65_CPSW_NU_CPTS_BASE	0x1d000

#define AM65_CPSW_NU_PORTS_OFFSET	0x1000
#define AM65_CPSW_NU_STATS_PORT_OFFSET	0x200

#define AM65_CPSW_MAX_PORTS	9

#define AM65_CPSW_MIN_PACKET_SIZE	VLAN_ETH_ZLEN
#define AM65_CPSW_MAX_PACKET_SIZE	(VLAN_ETH_FRAME_LEN + ETH_FCS_LEN)

#define AM65_CPSW_REG_CTL		0x004
#define AM65_CPSW_REG_STAT_PORT_EN	0x014
#define AM65_CPSW_REG_PTYPE		0x018

#define AM65_CPSW_P0_REG_CTL			0x004
#define AM65_CPSW_PORT0_REG_FLOW_ID_OFFSET	0x008

#define AM65_CPSW_PORT_REG_PRI_CTL		0x01c
#define AM65_CPSW_PORT_REG_RX_PRI_MAP		0x020
#define AM65_CPSW_PORT_REG_RX_MAXLEN		0x024

#define AM65_CPSW_PORTN_REG_SA_L		0x308
#define AM65_CPSW_PORTN_REG_SA_H		0x30c
#define AM65_CPSW_PORTN_REG_TS_CTL              0x310
#define AM65_CPSW_PORTN_REG_TS_SEQ_LTYPE_REG	0x314
#define AM65_CPSW_PORTN_REG_TS_VLAN_LTYPE_REG	0x318
#define AM65_CPSW_PORTN_REG_TS_CTL_LTYPE2       0x31C

#define AM65_CPSW_CTL_VLAN_AWARE		BIT(1)
#define AM65_CPSW_CTL_P0_ENABLE			BIT(2)
#define AM65_CPSW_CTL_P0_TX_CRC_REMOVE		BIT(13)
#define AM65_CPSW_CTL_P0_RX_PAD			BIT(14)

/* AM65_CPSW_P0_REG_CTL */
#define AM65_CPSW_P0_REG_CTL_RX_CHECKSUM_EN	BIT(0)

/* AM65_CPSW_PORT_REG_PRI_CTL */
#define AM65_CPSW_PORT_REG_PRI_CTL_RX_PTYPE_RROBIN	BIT(8)

/* AM65_CPSW_PN_TS_CTL register fields */
#define AM65_CPSW_PN_TS_CTL_TX_ANX_F_EN		BIT(4)
#define AM65_CPSW_PN_TS_CTL_TX_VLAN_LT1_EN	BIT(5)
#define AM65_CPSW_PN_TS_CTL_TX_VLAN_LT2_EN	BIT(6)
#define AM65_CPSW_PN_TS_CTL_TX_ANX_D_EN		BIT(7)
#define AM65_CPSW_PN_TS_CTL_TX_ANX_E_EN		BIT(10)
#define AM65_CPSW_PN_TS_CTL_TX_HOST_TS_EN	BIT(11)
#define AM65_CPSW_PN_TS_CTL_MSG_TYPE_EN_SHIFT	16

/* AM65_CPSW_PORTN_REG_TS_SEQ_LTYPE_REG register fields */
#define AM65_CPSW_PN_TS_SEQ_ID_OFFSET_SHIFT	16

/* AM65_CPSW_PORTN_REG_TS_CTL_LTYPE2 */
#define AM65_CPSW_PN_TS_CTL_LTYPE2_TS_107	BIT(16)
#define AM65_CPSW_PN_TS_CTL_LTYPE2_TS_129	BIT(17)
#define AM65_CPSW_PN_TS_CTL_LTYPE2_TS_130	BIT(18)
#define AM65_CPSW_PN_TS_CTL_LTYPE2_TS_131	BIT(19)
#define AM65_CPSW_PN_TS_CTL_LTYPE2_TS_132	BIT(20)
#define AM65_CPSW_PN_TS_CTL_LTYPE2_TS_319	BIT(21)
#define AM65_CPSW_PN_TS_CTL_LTYPE2_TS_320	BIT(22)
#define AM65_CPSW_PN_TS_CTL_LTYPE2_TS_TTL_NONZERO BIT(23)

/* The PTP event messages - Sync, Delay_Req, Pdelay_Req, and Pdelay_Resp. */
#define AM65_CPSW_TS_EVENT_MSG_TYPE_BITS (BIT(0) | BIT(1) | BIT(2) | BIT(3))

#define AM65_CPSW_TS_SEQ_ID_OFFSET (0x1e)

#define AM65_CPSW_TS_TX_ANX_ALL_EN		\
	(AM65_CPSW_PN_TS_CTL_TX_ANX_D_EN |	\
	 AM65_CPSW_PN_TS_CTL_TX_ANX_E_EN |	\
	 AM65_CPSW_PN_TS_CTL_TX_ANX_F_EN)

#define AM65_CPSW_ALE_AGEOUT_DEFAULT	30
/* Number of TX/RX descriptors */
#define AM65_CPSW_MAX_TX_DESC	500
#define AM65_CPSW_MAX_RX_DESC	500

#define mac_hi(mac)	(((mac)[0] << 0) | ((mac)[1] << 8) |	\
			 ((mac)[2] << 16) | ((mac)[3] << 24))
#define mac_lo(mac)	(((mac)[4] << 0) | ((mac)[5] << 8))

#define AM65_CPSW_NAV_PS_DATA_SIZE 16
#define AM65_CPSW_NAV_SW_DATA_SIZE 16

static int am65_cpsw_debug_level;
module_param(am65_cpsw_debug_level, int, 0);
MODULE_PARM_DESC(am65_cpsw_debug_level,
		 "TI AM65x cpsw debug level (NETIF_MSG bits)");

static void am65_cpsw_port_set_sl_mac(struct am65_cpsw_port *slave,
				      const u8 *dev_addr)
{
	writel(mac_hi(dev_addr), slave->port_base + AM65_CPSW_PORTN_REG_SA_H);
	writel(mac_lo(dev_addr), slave->port_base + AM65_CPSW_PORTN_REG_SA_L);
}

static void am65_cpsw_nuss_get_ver(struct am65_cpsw_common *common)
{
	common->nuss_ver = readl(common->ss_base);
	common->cpsw_ver = readl(common->cpsw_base);
	dev_info(common->dev,
		 "initializing am65 cpsw nuss version 0x%08X,"
		" cpsw version 0x%08X"
		" Ports: %u\n",
		common->nuss_ver,
		common->cpsw_ver,
		common->port_num);
}

void am65_cpsw_nuss_adjust_link(struct net_device *ndev)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct am65_cpsw_common *common = am65_ndev_to_common(ndev);
	struct phy_device *phy = port->slave.phy;
	u32 mac_control = 0;

	if (!phy)
		return;

	if (phy->link) {
		mac_control = CPSW_SL_CTL_GMII_EN;

		if (phy->speed == 1000)
			mac_control |= CPSW_SL_CTL_GIG;
		if (phy->duplex)
			mac_control |= CPSW_SL_CTL_FULLDUPLEX;

		/* RGMII speed is 100M if !CPSW_SL_CTL_GIG*/

		/* rx_pause/tx_pause */
		if (port->slave.rx_pause)
			mac_control |= CPSW_SL_CTL_RX_FLOW_EN;

		if (port->slave.tx_pause)
			mac_control |= CPSW_SL_CTL_TX_FLOW_EN;

		cpsw_sl_ctl_set(port->slave.mac_sl, mac_control);

		/* enable forwarding */
		cpsw_ale_control_set(common->ale, port->port_id,
				     ALE_PORT_STATE, ALE_PORT_STATE_FORWARD);

		netif_carrier_on(ndev);
		netif_tx_wake_all_queues(ndev);
	} else {
		/* disable forwarding */
		cpsw_ale_control_set(common->ale, port->port_id,
				     ALE_PORT_STATE, ALE_PORT_STATE_DISABLE);

		cpsw_sl_wait_for_idle(port->slave.mac_sl);

		cpsw_sl_ctl_reset(port->slave.mac_sl);

		netif_carrier_off(ndev);
		netif_tx_stop_all_queues(ndev);
	}

	phy_print_status(phy);
}
EXPORT_SYMBOL_GPL(am65_cpsw_nuss_adjust_link);

static int am65_cpsw_nuss_ndo_slave_add_vid(struct net_device *ndev,
					    __be16 proto, u16 vid)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct am65_cpsw_common *common = am65_ndev_to_common(ndev);
	u32 port_mask, unrag_mask = 0;
	int ret;

	ret = pm_runtime_get_sync(common->dev);
	if (ret < 0) {
		pm_runtime_put_noidle(common->dev);
		return ret;
	}

	port_mask = BIT(port->port_id) | ALE_PORT_HOST;
	if (!vid)
		unrag_mask = port_mask;
	dev_info(common->dev, "Adding vlan %d to vlan filter\n", vid);
	ret = cpsw_ale_add_vlan(common->ale, vid, port_mask,
				unrag_mask, port_mask, 0);

	pm_runtime_put(common->dev);
	return ret;
}

static int am65_cpsw_nuss_ndo_slave_kill_vid(struct net_device *ndev,
					     __be16 proto, u16 vid)
{
	struct am65_cpsw_common *common = am65_ndev_to_common(ndev);
	int ret;

	ret = pm_runtime_get_sync(common->dev);
	if (ret < 0) {
		pm_runtime_put_noidle(common->dev);
		return ret;
	}

	dev_info(common->dev, "Removing vlan %d from vlan filter\n", vid);
	ret = cpsw_ale_del_vlan(common->ale, vid, 0);

	pm_runtime_put(common->dev);
	return ret;
}

static void am65_cpsw_slave_set_promisc_2g(struct am65_cpsw_port *port,
					   bool promisc)
{
	struct am65_cpsw_common *common = port->common;

	cpsw_ale_control_set(common->ale, ALE_PORT_HOST,
			     ALE_BYPASS, promisc);

	dev_dbg(common->dev, "promisc disabled %s\n",
		promisc ? "enabled" : "disabled");
}

static void am65_cpsw_slave_set_promisc_slave(struct am65_cpsw_port *port,
					      bool promisc)
{
	struct am65_cpsw_common *common = port->common;

	if (promisc) {
		/* Enable promiscuous mode */
		cpsw_ale_control_set(common->ale, port->port_id,
				     ALE_PORT_NOLEARN, 1);
		cpsw_ale_control_set(common->ale, port->port_id,
				     ALE_PORT_NO_SA_UPDATE, 1);
		cpsw_ale_control_set(common->ale, port->port_id,
				     ALE_PORT_MACONLY_CAF, 1);
		dev_dbg(common->dev, "promisc enabled\n");
	} else {
		/* Disable promiscuous mode */
		cpsw_ale_control_set(common->ale, port->port_id,
				     ALE_PORT_NOLEARN, 0);
		cpsw_ale_control_set(common->ale, port->port_id,
				     ALE_PORT_NO_SA_UPDATE, 0);
		cpsw_ale_control_set(common->ale, port->port_id,
				     ALE_PORT_MACONLY_CAF, 0);
		dev_dbg(common->dev, "promisc disabled\n");
	}
}

static void am65_cpsw_nuss_ndo_slave_set_rx_mode(struct net_device *ndev)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct am65_cpsw_common *common = am65_ndev_to_common(ndev);
	u32 port_mask;
	bool promisc;

	promisc = !!(ndev->flags & IFF_PROMISC);
	if (AM65_CPSW_IS_CPSW2G(common))
		am65_cpsw_slave_set_promisc_2g(port, promisc);
	else
		am65_cpsw_slave_set_promisc_slave(port, promisc);

	if (promisc)
		return;

	/* Restore allmulti on vlans if necessary */
	cpsw_ale_set_allmulti(common->ale, ndev->flags & IFF_ALLMULTI);

	port_mask = BIT(port->port_id) | ALE_PORT_HOST;
	/* Clear all mcast from ALE */
	cpsw_ale_flush_multicast(common->ale, port_mask, -1);

	if (!netdev_mc_empty(ndev)) {
		struct netdev_hw_addr *ha;

		/* program multicast address list into ALE register */
		netdev_for_each_mc_addr(ha, ndev) {
			cpsw_ale_add_mcast(common->ale, ha->addr,
					   port_mask, 0, 0, 0);
		}
	}
}

static void am65_cpsw_nuss_ndo_host_tx_timeout(struct net_device *ndev)
{
	struct am65_cpsw_common *common = am65_ndev_to_common(ndev);
	int ch;

	/* process every txq*/
	for (ch = 0; ch < common->tx_ch_num; ch++) {
		struct am65_cpsw_tx_chn *tx_chn;
		struct netdev_queue *netif_txq;
		unsigned long trans_start;

		netif_txq = netdev_get_tx_queue(ndev, ch);
		trans_start = netif_txq->trans_start;
		if (netif_xmit_stopped(netif_txq) &&
		    time_after(jiffies, (trans_start + ndev->watchdog_timeo))) {
			tx_chn = &common->tx_chns[ch];
			netdev_err(ndev, "txq:%d DRV_XOFF:%d tmo:%u dql_avail:%d free_desc:%zu\n",
				   ch,
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
}

static int am65_cpsw_nuss_rx_push(struct am65_cpsw_common *common,
				  struct sk_buff *skb)
{
	struct cppi5_host_desc_t *desc_rx;
	struct am65_cpsw_rx_chn *rx_chn = &common->rx_chns;
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
			 AM65_CPSW_NAV_PS_DATA_SIZE);
	cppi5_hdesc_attach_buf(desc_rx, 0, 0, buf_dma, skb_tailroom(skb));
	swdata = cppi5_hdesc_get_swdata(desc_rx);
	*((void **)swdata) = skb;

	return k3_nav_udmax_push_rx_chn(rx_chn->rx_chn, 0, desc_rx, desc_dma);
}

void am65_cpsw_nuss_set_p0_ptype(struct am65_cpsw_common *common)
{
	struct am65_cpsw_port *host_p = am65_common_get_host_port(common);
	u32 val, pri_map;

	/* P0 set Receive Priority Type */
	val = readl(host_p->port_base + AM65_CPSW_PORT_REG_PRI_CTL);

	if (common->pf_p0_rx_ptype_rrobin) {
		val |= AM65_CPSW_PORT_REG_PRI_CTL_RX_PTYPE_RROBIN;
		/* Enet Ports fifos works in fixed priority mode only, so
		 * reset P0_Rx_Pri_Map so all packet will go in Enet fifo 0
		 */
		pri_map = 0x0;
	} else {
		val &= ~AM65_CPSW_PORT_REG_PRI_CTL_RX_PTYPE_RROBIN;
		/* restore P0_Rx_Pri_Map */
		pri_map = 0x76543210;
	}

	writel(pri_map, host_p->port_base + AM65_CPSW_PORT_REG_RX_PRI_MAP);
	writel(val, host_p->port_base + AM65_CPSW_PORT_REG_PRI_CTL);
}

static int am65_cpsw_nuss_common_open(struct am65_cpsw_common *common,
				      netdev_features_t features)
{
	struct am65_cpsw_port *host_p = am65_common_get_host_port(common);
	int port_idx, i, ret;
	struct sk_buff *skb;
	u32 val, port_mask;

	if (common->usage_count)
		return 0;

	/* Control register */
	writel(AM65_CPSW_CTL_P0_ENABLE | AM65_CPSW_CTL_P0_TX_CRC_REMOVE |
	       AM65_CPSW_CTL_VLAN_AWARE | AM65_CPSW_CTL_P0_RX_PAD,
	       common->cpsw_base + AM65_CPSW_REG_CTL);
	/* Max length register */
	writel(AM65_CPSW_MAX_PACKET_SIZE,
	       host_p->port_base + AM65_CPSW_PORT_REG_RX_MAXLEN);
	/* set base flow_id */
	writel(common->rx_flow_id_base,
	       host_p->port_base + AM65_CPSW_PORT0_REG_FLOW_ID_OFFSET);
	/* en tx crc offload */
	if (features & NETIF_F_HW_CSUM)
		writel(AM65_CPSW_P0_REG_CTL_RX_CHECKSUM_EN,
		       host_p->port_base + AM65_CPSW_P0_REG_CTL);

	am65_cpsw_nuss_set_p0_ptype(common);

	/* enable statistic */
	val = 0;
	for (port_idx = 0; port_idx < common->port_num; port_idx++) {
		if (!common->ports[port_idx].disabled)
			val |=  BIT(common->ports[port_idx].port_id);
	}
	writel(val, common->cpsw_base + AM65_CPSW_REG_STAT_PORT_EN);

	/* disable priority elevation */
	writel(0, common->cpsw_base + AM65_CPSW_REG_PTYPE);

	cpsw_ale_start(common->ale);

	/* switch to vlan unaware mode */
	cpsw_ale_control_set(common->ale, HOST_PORT_NUM, ALE_VLAN_AWARE, 1);
	cpsw_ale_control_set(common->ale, HOST_PORT_NUM,
			     ALE_PORT_STATE, ALE_PORT_STATE_FORWARD);

	/* default vlan cfg */
	/* create mask based on enabled ports */
	port_mask = GENMASK(common->port_num - 1, 0) &
		    ~common->disabled_ports_mask;

	cpsw_ale_add_vlan(common->ale, 0, port_mask,
			  port_mask, port_mask,
			  port_mask & ~ALE_PORT_HOST);

	for (i = 0; i < common->rx_chns.descs_num; i++) {
		skb = __netdev_alloc_skb_ip_align(NULL,
						  AM65_CPSW_MAX_PACKET_SIZE,
						  GFP_KERNEL);
		if (!skb) {
			dev_err(common->dev, "cannot allocate skb\n");
			return -ENOMEM;
		}

		ret = am65_cpsw_nuss_rx_push(common, skb);
		if (ret < 0) {
			dev_err(common->dev,
				"cannot submit skb to channel rx, error %d\n",
				ret);
			kfree_skb(skb);
			return ret;
		}
		kmemleak_not_leak(skb);
	}
	k3_nav_udmax_enable_rx_chn(common->rx_chns.rx_chn);

	for (i = 0; i < AM65_CPSW_MAX_TX_QUEUES; i++) {
		ret = k3_nav_udmax_enable_tx_chn(common->tx_chns[i].tx_chn);
		if (ret)
			return ret;
	}

	napi_enable(&common->napi_tx);
	napi_enable(&common->napi_rx);

	dev_dbg(common->dev, "cpsw_nuss started\n");
	return 0;
}

static void am65_cpsw_nuss_tx_cleanup(void *data, dma_addr_t desc_dma);
static void am65_cpsw_nuss_rx_cleanup(void *data, dma_addr_t desc_dma);

static int am65_cpsw_nuss_common_stop(struct am65_cpsw_common *common)
{
	int i;

	if (common->usage_count != 1)
		return 0;

	cpsw_ale_control_set(common->ale, HOST_PORT_NUM,
			     ALE_PORT_STATE, ALE_PORT_STATE_DISABLE);

	/* shutdown tx channels */
	atomic_set(&common->tdown_cnt, AM65_CPSW_MAX_TX_QUEUES);
	/* ensure new tdown_cnt value is visible */
	smp_mb__after_atomic();
	reinit_completion(&common->tdown_complete);

	for (i = 0; i < AM65_CPSW_MAX_TX_QUEUES; i++)
		k3_nav_udmax_tdown_tx_chn(common->tx_chns[i].tx_chn, false);

	i = wait_for_completion_timeout(&common->tdown_complete,
					msecs_to_jiffies(1000));
	if (!i)
		dev_err(common->dev, "tx timeout\n");

	for (i = 0; i < AM65_CPSW_MAX_TX_QUEUES; i++) {
		k3_nav_udmax_reset_tx_chn(
				common->tx_chns[i].tx_chn,
				&common->tx_chns[i],
				am65_cpsw_nuss_tx_cleanup);
		k3_nav_udmax_disable_tx_chn(common->tx_chns[i].tx_chn);
	}

	k3_nav_udmax_tdown_rx_chn(common->rx_chns.rx_chn, true);

	for (i = 0; i < AM65_CPSW_MAX_RX_FLOWS; i++)
		k3_nav_udmax_reset_rx_chn(
				common->rx_chns.rx_chn, i, &common->rx_chns,
				am65_cpsw_nuss_rx_cleanup, !!i);

	k3_nav_udmax_disable_rx_chn(common->rx_chns.rx_chn);

	napi_disable(&common->napi_rx);
	napi_disable(&common->napi_tx);

	cpsw_ale_stop(common->ale);

	writel(0, common->cpsw_base + AM65_CPSW_REG_CTL);
	writel(0, common->cpsw_base + AM65_CPSW_REG_STAT_PORT_EN);

	dev_dbg(common->dev, "cpsw_nuss stopped\n");
	return 0;
}

static int am65_cpsw_nuss_ndo_slave_stop(struct net_device *ndev)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct am65_cpsw_common *common = am65_ndev_to_common(ndev);
	int ret;

	cpsw_ale_control_set(common->ale, port->port_id,
			     ALE_PORT_STATE, ALE_PORT_STATE_DISABLE);

	netif_tx_stop_all_queues(ndev);
	netif_carrier_off(ndev);

	if (port->slave.phy) {
		phy_stop(port->slave.phy);
		phy_disconnect(port->slave.phy);
		port->slave.phy = NULL;
	}

	ret = am65_cpsw_nuss_common_stop(common);
	if (ret)
		return ret;

	cpsw_sl_reset(port->slave.mac_sl, 100);

	common->usage_count--;
	pm_runtime_put(common->dev);
	return 0;
}

static int am65_cpsw_nuss_ndo_slave_open(struct net_device *ndev)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct am65_cpsw_common *common = am65_ndev_to_common(ndev);
	u32 port_mask;
	int ret, i;

	ret = pm_runtime_get_sync(common->dev);
	if (ret < 0) {
		pm_runtime_put_noidle(common->dev);
		return ret;
	}

	/* Notify the stack of the actual queue counts. */
	ret = netif_set_real_num_tx_queues(ndev, common->tx_ch_num);
	if (ret) {
		dev_err(common->dev, "cannot set real number of tx queues\n");
		return ret;
	}

	ret = netif_set_real_num_rx_queues(ndev, common->rx_ch_num);
	if (ret) {
		dev_err(common->dev, "cannot set real number of rx queues\n");
		return ret;
	}

	for (i = 0; i < AM65_CPSW_MAX_TX_QUEUES; i++)
		netdev_tx_reset_queue(netdev_get_tx_queue(ndev, i));

	ret = am65_cpsw_nuss_common_open(common, ndev->features);
	if (ret)
		return ret;

	common->usage_count++;

	/* Max length register */
	writel(AM65_CPSW_MAX_PACKET_SIZE,
	       port->port_base + AM65_CPSW_PORT_REG_RX_MAXLEN);

	am65_cpsw_port_set_sl_mac(port, ndev->dev_addr);

	if (port->slave.mac_only)
		/* enable mac-only mode on port */
		cpsw_ale_control_set(common->ale, port->port_id,
				     ALE_PORT_MACONLY, 1);

	port_mask = BIT(port->port_id) | ALE_PORT_HOST;
	cpsw_ale_add_ucast(common->ale, ndev->dev_addr,
			   HOST_PORT_NUM, ALE_SECURE, 0);
	cpsw_ale_add_mcast(common->ale, ndev->broadcast,
			   port_mask, 0, 0, ALE_MCAST_FWD_2);

	/* mac_sl should be configured via phy-link interface */
	cpsw_sl_reset(port->slave.mac_sl, 100);
	cpsw_sl_ctl_reset(port->slave.mac_sl);

	cpsw_ale_control_set(common->ale, port->port_id,
			     ALE_PORT_STATE, ALE_PORT_STATE_FORWARD);

	cpsw_phy_sel(common->dev, port->slave.phy_if, port->port_id);

	if (port->slave.phy_node) {
		port->slave.phy = of_phy_connect(ndev,
						 port->slave.phy_node,
						 &am65_cpsw_nuss_adjust_link,
						 0, port->slave.phy_if);
		if (!port->slave.phy) {
			dev_err(common->dev, "phy %pOF not found on slave %d\n",
				port->slave.phy_node,
				port->port_id);
			ret = -ENODEV;
			goto error_cleanup;
		}
	}

	phy_attached_info(port->slave.phy);
	phy_start(port->slave.phy);

	return 0;

error_cleanup:
	am65_cpsw_nuss_ndo_slave_stop(ndev);
	return ret;
}

static void am65_cpsw_nuss_rx_cleanup(void *data, dma_addr_t desc_dma)
{
	struct am65_cpsw_rx_chn *rx_chn = data;
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

static void am65_cpsw_nuss_rx_ts(struct sk_buff *skb, u32 *psdata)
{
	struct skb_shared_hwtstamps *ssh;
	u64 ns;

	ns = ((u64)psdata[1] << 32) | psdata[0];

	ssh = skb_hwtstamps(skb);
	memset(ssh, 0, sizeof(*ssh));
	ssh->hwtstamp = ns_to_ktime(ns);
}

/* RX psdata[2] word format - checksum information */
#define AM65_CPSW_RX_PSD_CSUM_ADD	GENMASK(15, 0)
#define AM65_CPSW_RX_PSD_CSUM_ERR	BIT(16)
#define AM65_CPSW_RX_PSD_IS_FRAGMENT	BIT(17)
#define AM65_CPSW_RX_PSD_IS_TCP		BIT(18)
#define AM65_CPSW_RX_PSD_IPV6_VALID	BIT(19)
#define AM65_CPSW_RX_PSD_IPV4_VALID	BIT(20)

static void am65_cpsw_nuss_rx_csum(struct sk_buff *skb, u32 csum_info)
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

	if (!(csum_info & AM65_CPSW_RX_PSD_CSUM_ERR) &&
	    (csum_info & (AM65_CPSW_RX_PSD_IPV6_VALID |
			  AM65_CPSW_RX_PSD_IPV6_VALID))) {
		if (csum_info & AM65_CPSW_RX_PSD_IS_FRAGMENT) {
			skb->ip_summed = CHECKSUM_COMPLETE;
			skb->csum = csum_unfold(csum_info &
						AM65_CPSW_RX_PSD_CSUM_ADD);
		} else {
			skb->ip_summed = CHECKSUM_UNNECESSARY;
		}
	}
}

static int am65_cpsw_nuss_rx_packets(struct am65_cpsw_common *common,
				     u32 flow_idx)
{
	struct am65_cpsw_rx_chn *rx_chn = &common->rx_chns;
	struct device *dev = common->dev;
	struct am65_cpsw_ndev_priv *ndev_priv;
	struct am65_cpsw_ndev_stats *stats;
	struct net_device *ndev;
	struct cppi5_host_desc_t *desc_rx;
	struct sk_buff *skb, *new_skb;
	dma_addr_t desc_dma, buf_dma;
	u32 buf_dma_len, pkt_len, port_id = 0, csum_info;
	int ret = 0;
	void **swdata;
	u32 *psdata;

	ret = k3_nav_udmax_pop_rx_chn(rx_chn->rx_chn, flow_idx, &desc_dma);
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
	if (port_id)
		port_id = (port_id % common->port_num);
	dev_dbg(dev, "%s rx port_id:%d\n", __func__, port_id);
	ndev = common->ports[port_id].ndev;
	skb->dev = ndev;

	psdata = cppi5_hdesc_get_psdata32(desc_rx);
	/* add RX timestamp */
	if (common->ports[port_id].rx_ts_enabled)
		am65_cpsw_nuss_rx_ts(skb, psdata);
	csum_info = psdata[2];
	dev_dbg(dev, "%s rx csum_info:%#x\n", __func__, csum_info);

	dma_unmap_single(dev, buf_dma, buf_dma_len, DMA_FROM_DEVICE);

	k3_knav_pool_free(rx_chn->desc_pool, desc_rx);

	if (unlikely(!netif_running(skb->dev))) {
		dev_kfree_skb_any(skb);
		return -ENODEV;
	}

	new_skb = netdev_alloc_skb_ip_align(ndev, AM65_CPSW_MAX_PACKET_SIZE);
	if (new_skb) {
		skb_put(skb, pkt_len);
		skb->protocol = eth_type_trans(skb, ndev);
		am65_cpsw_nuss_rx_csum(skb, csum_info);
		netif_receive_skb(skb);

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

	ret = am65_cpsw_nuss_rx_push(common, new_skb);
	if (WARN_ON(ret < 0)) {
		dev_kfree_skb_any(new_skb);
		ndev->stats.rx_errors++;
		ndev->stats.rx_dropped++;
	}

	return ret;
}

static int am65_cpsw_nuss_rx_poll(struct napi_struct *napi_rx, int budget)
{
	struct am65_cpsw_common *common = am65_cpsw_napi_to_common(napi_rx);
	int num_rx = 0;
	int cur_budget, flow = AM65_CPSW_MAX_RX_FLOWS;
	int ret;

	/* process every flow */
	while (flow--) {
		cur_budget = budget - num_rx;

		while (cur_budget--) {
			ret = am65_cpsw_nuss_rx_packets(common, flow);
			if (ret)
				break;
			num_rx++;
		}

		if (num_rx >= budget)
			break;
	}

	dev_dbg(common->dev, "%s num_rx:%d %d\n", __func__, num_rx, budget);

	if (num_rx < budget) {
		napi_complete(napi_rx);
		enable_irq(common->rx_chns.irq);
	}

	return num_rx;
}

static void am65_cpsw_nuss_xmit_free(struct am65_cpsw_tx_chn *tx_chn,
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

static void am65_cpsw_nuss_tx_cleanup(void *data, dma_addr_t desc_dma)
{
	struct am65_cpsw_tx_chn *tx_chn = data;
	struct cppi5_host_desc_t *desc_tx;
	struct sk_buff *skb;
	void **swdata;

	desc_tx = k3_knav_pool_dma2virt(tx_chn->desc_pool, desc_dma);
	swdata = cppi5_hdesc_get_swdata(desc_tx);
	skb = *(swdata);
	am65_cpsw_nuss_xmit_free(tx_chn, tx_chn->dev, desc_tx);

	dev_kfree_skb_any(skb);
}

static int am65_cpsw_nuss_tx_compl_packets(struct am65_cpsw_common *common,
					   int chn, unsigned int budget)
{
	struct cppi5_host_desc_t *desc_tx;
	struct device *dev = common->dev;
	struct netdev_queue *netif_txq;
	struct am65_cpsw_tx_chn *tx_chn;
	struct net_device *ndev;
	unsigned int total_bytes = 0;
	struct sk_buff *skb;
	dma_addr_t desc_dma;
	int res, num_tx = 0;
	void **swdata;

	tx_chn = &common->tx_chns[chn - 1];

	while (budget--) {
		struct am65_cpsw_ndev_priv *ndev_priv;
		struct am65_cpsw_ndev_stats *stats;

		res = k3_nav_udmax_pop_tx_chn(tx_chn->tx_chn, &desc_dma);
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
		am65_cpsw_nuss_xmit_free(tx_chn, dev, desc_tx);

		ndev = skb->dev;

		if (skb_shinfo(skb)->tx_flags & SKBTX_IN_PROGRESS)
			am65_cpts_tx_timestamp(common->cpts, skb);

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

	netif_txq = netdev_get_tx_queue(ndev, chn - 1);

	netdev_tx_completed_queue(netif_txq, num_tx, total_bytes);

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

static int am65_cpsw_nuss_tx_poll(struct napi_struct *napi_tx, int budget)
{
	struct am65_cpsw_common *common = am65_cpsw_napi_to_common(napi_tx);
	int cur_budget, ch, num_tx = 0;

	/* process every unprocessed channel */
	for (ch = common->tx_ch_num; ch; ch--) {
		u32 cur_ch = ch;
		cur_budget = budget - num_tx;

		if (common->pf_p0_rx_ptype_rrobin) {
			common->cur_txq = (common->cur_txq + 1) %
					   common->tx_ch_num;
			cur_ch = common->cur_txq + 1;
		}
		num_tx += am65_cpsw_nuss_tx_compl_packets(
				common, cur_ch, cur_budget);
		if (num_tx >= budget)
			break;
	}

	if (num_tx < budget) {
		napi_complete(napi_tx);
		enable_irq(common->tx_chns[0].irq);
	}

	return num_tx;
}

static irqreturn_t am65_cpsw_nuss_rx_irq(int irq, void *dev_id)
{
	struct am65_cpsw_common *common = dev_id;

	disable_irq_nosync(irq);
	napi_schedule(&common->napi_rx);

	return IRQ_HANDLED;
}

static irqreturn_t am65_cpsw_nuss_tx_irq(int irq, void *dev_id)
{
	struct am65_cpsw_common *common = dev_id;

	disable_irq_nosync(irq);
	napi_schedule(&common->napi_tx);

	return IRQ_HANDLED;
}

static netdev_tx_t am65_cpsw_nuss_ndo_slave_xmit(struct sk_buff *skb,
						 struct net_device *ndev)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct am65_cpsw_common *common = am65_ndev_to_common(ndev);
	struct device *dev = common->dev;
	struct cppi5_host_desc_t *first_desc, *next_desc, *cur_desc;
	struct am65_cpsw_tx_chn *tx_chn;
	struct netdev_queue *netif_txq;
	dma_addr_t desc_dma, buf_dma;
	int ret, q_idx, i;
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

	/* SKB TX timestamp */
	if (skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP && port->tx_ts_enabled)
		skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;

	q_idx = skb_get_queue_mapping(skb);
	dev_dbg(dev, "%s skb_queue:%d\n", __func__, q_idx);
	q_idx = q_idx % common->tx_ch_num;

	tx_chn = &common->tx_chns[q_idx];
	netif_txq = netdev_get_tx_queue(ndev, q_idx);

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
			 AM65_CPSW_NAV_PS_DATA_SIZE);
	cppi5_desc_set_pktids(&first_desc->hdr, 0, 0x3FFF);
	cppi5_hdesc_set_pkttype(first_desc, 0x7);
	cppi5_desc_set_tags_ids(&first_desc->hdr, 0, port->port_id);

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
	netdev_tx_sent_queue(netif_txq, pkt_len);

	cppi5_hdesc_set_pktlen(first_desc, pkt_len);
	desc_dma = k3_knav_pool_virt2dma(tx_chn->desc_pool, first_desc);
	ret = k3_nav_udmax_push_tx_chn(tx_chn->tx_chn, first_desc, desc_dma);
	if (ret) {
		dev_err(dev, "can't push desc %d\n", ret);
		ndev->stats.tx_errors++;
		goto drop_free_descs;
	}

	if (k3_knav_pool_avail(tx_chn->desc_pool) < MAX_SKB_FRAGS) {
		netif_tx_stop_queue(netif_txq);
		/* Barrier, so that stop_queue visible to other cpus */
		smp_mb__after_atomic();
		dev_err(dev, "netif_tx_stop_queue %d\n", q_idx);

		/* re-check for smp */
		if (k3_knav_pool_avail(tx_chn->desc_pool) >= MAX_SKB_FRAGS) {
			netif_tx_wake_queue(netif_txq);
			dev_err(dev, "netif_tx_wake_queue %d\n", q_idx);
		}
	}

	return NETDEV_TX_OK;

drop_free_descs:
	am65_cpsw_nuss_xmit_free(tx_chn, dev, first_desc);
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

static int am65_cpsw_nuss_ndo_slave_set_mac_address(
		struct net_device *ndev, void *addr)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct am65_cpsw_common *common = am65_ndev_to_common(ndev);
	struct sockaddr *sockaddr = (struct sockaddr *)addr;
	int ret;

	ret = eth_prepare_mac_addr_change(ndev, addr);
	if (ret < 0)
		return ret;

	ret = pm_runtime_get_sync(common->dev);
	if (ret < 0) {
		pm_runtime_put_noidle(common->dev);
		return ret;
	}

	cpsw_ale_del_ucast(common->ale, ndev->dev_addr,
			   HOST_PORT_NUM, 0, 0);
	cpsw_ale_add_ucast(common->ale, sockaddr->sa_data,
			   HOST_PORT_NUM, ALE_SECURE, 0);

	am65_cpsw_port_set_sl_mac(port, addr);
	eth_commit_mac_addr_change(ndev, sockaddr);

	pm_runtime_put(common->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_TI_AM65_CPTS)
static int am65_cpsw_nuss_hwtstamp_set(struct net_device *ndev,
				       struct ifreq *ifr)
{
	struct am65_cpsw_common *common = am65_ndev_to_common(ndev);
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct hwtstamp_config cfg;
	u32 ts_ctrl, seq_id, ts_ctrl_ltype2, ts_vlan_ltype;

	if (copy_from_user(&cfg, ifr->ifr_data, sizeof(cfg)))
		return -EFAULT;

	if (cfg.tx_type != HWTSTAMP_TX_OFF && cfg.tx_type != HWTSTAMP_TX_ON)
		return -ERANGE;

	switch (cfg.rx_filter) {
	case HWTSTAMP_FILTER_NONE:
		port->rx_ts_enabled = false;
		break;
	case HWTSTAMP_FILTER_PTP_V1_L4_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_EVENT:
	case HWTSTAMP_FILTER_ALL:
		port->rx_ts_enabled = true;
		cfg.rx_filter = HWTSTAMP_FILTER_ALL;
		break;
	default:
		return -ERANGE;
	}

	port->tx_ts_enabled = (cfg.tx_type == HWTSTAMP_TX_ON);

	/* cfg TX timestamp */
	seq_id = (AM65_CPSW_TS_SEQ_ID_OFFSET <<
		  AM65_CPSW_PN_TS_SEQ_ID_OFFSET_SHIFT) | ETH_P_1588;

	ts_vlan_ltype = ETH_P_8021Q;

	ts_ctrl_ltype2 = ETH_P_1588 |
			 AM65_CPSW_PN_TS_CTL_LTYPE2_TS_107 |
			 AM65_CPSW_PN_TS_CTL_LTYPE2_TS_129 |
			 AM65_CPSW_PN_TS_CTL_LTYPE2_TS_130 |
			 AM65_CPSW_PN_TS_CTL_LTYPE2_TS_131 |
			 AM65_CPSW_PN_TS_CTL_LTYPE2_TS_132 |
			 AM65_CPSW_PN_TS_CTL_LTYPE2_TS_319 |
			 AM65_CPSW_PN_TS_CTL_LTYPE2_TS_320 |
			 AM65_CPSW_PN_TS_CTL_LTYPE2_TS_TTL_NONZERO;

	ts_ctrl = AM65_CPSW_TS_EVENT_MSG_TYPE_BITS <<
		  AM65_CPSW_PN_TS_CTL_MSG_TYPE_EN_SHIFT;

	if (port->tx_ts_enabled)
		ts_ctrl |= AM65_CPSW_TS_TX_ANX_ALL_EN |
			   AM65_CPSW_PN_TS_CTL_TX_VLAN_LT1_EN;

	writel(seq_id, port->port_base + AM65_CPSW_PORTN_REG_TS_SEQ_LTYPE_REG);
	writel(ts_vlan_ltype, port->port_base +
	       AM65_CPSW_PORTN_REG_TS_VLAN_LTYPE_REG);
	writel(ts_ctrl_ltype2, port->port_base +
	       AM65_CPSW_PORTN_REG_TS_CTL_LTYPE2);
	writel(ts_ctrl, port->port_base + AM65_CPSW_PORTN_REG_TS_CTL);

	/* en/dis RX timestamp */
	am65_cpts_rx_enable(common->cpts, port->rx_ts_enabled);

	return copy_to_user(ifr->ifr_data, &cfg, sizeof(cfg)) ? -EFAULT : 0;
}

static int am65_cpsw_nuss_hwtstamp_get(struct net_device *ndev,
				       struct ifreq *ifr)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);
	struct hwtstamp_config cfg;

	cfg.flags = 0;
	cfg.tx_type = port->tx_ts_enabled ?
		      HWTSTAMP_TX_ON : HWTSTAMP_TX_OFF;
	cfg.rx_filter = port->rx_ts_enabled ?
			HWTSTAMP_FILTER_ALL : HWTSTAMP_FILTER_NONE;

	return copy_to_user(ifr->ifr_data, &cfg, sizeof(cfg)) ? -EFAULT : 0;
}
#else
static int am65_cpsw_nuss_hwtstamp_get(struct net_device *ndev,
				       struct ifreq *ifr)
{
	return -EOPNOTSUPP;
}

static int am65_cpsw_nuss_hwtstamp_set(struct net_device *ndev,
				       struct ifreq *ifr)
{
	return -EOPNOTSUPP;
}
#endif /* CONFIG_TI_AM65_CPTS */

static int am65_cpsw_switch_config_ioctl(struct net_device *ndev,
					 struct ifreq *ifrq, int cmd)
{
	struct am65_cpsw_common *common = am65_ndev_to_common(ndev);
	struct net_switch_config config;
	int ret = -EINVAL;

	/* Only SIOCSWITCHCONFIG is used as cmd argument and hence, there is no
	 * switch statement required.
	 * Function calls are based on switch_config.cmd
	 */

	if (copy_from_user(&config, ifrq->ifr_data, sizeof(config)))
		return -EFAULT;

	switch (config.cmd) {
	case CONFIG_SWITCH_RATELIMIT:
	{
		if (config.port > 1) {
			dev_err(common->dev, "Invalid Port number\n");
			break;
		}

		ret = cpsw_ale_set_ratelimit(common->ale,
					     common->bus_freq_mhz * 1000000,
					     config.port,
					     config.bcast_rate_limit,
					     config.mcast_rate_limit,
					     !!config.direction);
		if (ret)
			dev_err(common->dev, "CPSW_ALE set ratelimit failed");
		break;
	}

	default:
		ret = -EOPNOTSUPP;
	}

	return ret;
}

static int am65_cpsw_nuss_ndo_slave_ioctl(struct net_device *ndev,
					  struct ifreq *req, int cmd)
{
	struct am65_cpsw_port *port = am65_ndev_to_port(ndev);

	if (!netif_running(ndev))
		return -EINVAL;

	switch (cmd) {
	case SIOCSHWTSTAMP:
		return am65_cpsw_nuss_hwtstamp_set(ndev, req);
	case SIOCGHWTSTAMP:
		return am65_cpsw_nuss_hwtstamp_get(ndev, req);
	case SIOCSWITCHCONFIG:
		return am65_cpsw_switch_config_ioctl(ndev, req, cmd);
	}

	if (!port->slave.phy)
		return -EOPNOTSUPP;

	return phy_mii_ioctl(port->slave.phy, req, cmd);
}

static void am65_cpsw_nuss_ndo_get_stats(struct net_device *dev,
					 struct rtnl_link_stats64 *stats)
{
	struct am65_cpsw_ndev_priv *ndev_priv = netdev_priv(dev);
	unsigned int start;
	int cpu;

	for_each_possible_cpu(cpu) {
		struct am65_cpsw_ndev_stats *cpu_stats;
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

static int am65_cpsw_nuss_ndo_slave_set_features(struct net_device *ndev,
						 netdev_features_t features)
{
	struct am65_cpsw_common *common = am65_ndev_to_common(ndev);
	struct am65_cpsw_port *host_p = am65_common_get_host_port(common);
	netdev_features_t changes = features ^ ndev->features;

	if (changes & NETIF_F_HW_CSUM) {
		bool enable = !!(features & NETIF_F_HW_CSUM);

		dev_err(common->dev, "Turn %s tx-checksum-ip-generic\n",
			enable ? "ON" : "OFF");
		if (enable)
			writel(AM65_CPSW_P0_REG_CTL_RX_CHECKSUM_EN,
			       host_p->port_base + AM65_CPSW_P0_REG_CTL);
		else
			writel(0,
			       host_p->port_base + AM65_CPSW_P0_REG_CTL);
	}

	return 0;
}

static const struct net_device_ops am65_cpsw_nuss_netdev_ops_2g = {
	.ndo_open		= am65_cpsw_nuss_ndo_slave_open,
	.ndo_stop		= am65_cpsw_nuss_ndo_slave_stop,
	.ndo_start_xmit		= am65_cpsw_nuss_ndo_slave_xmit,
	.ndo_set_rx_mode	= am65_cpsw_nuss_ndo_slave_set_rx_mode,
	.ndo_get_stats64        = am65_cpsw_nuss_ndo_get_stats,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_set_mac_address	= am65_cpsw_nuss_ndo_slave_set_mac_address,
	.ndo_tx_timeout		= am65_cpsw_nuss_ndo_host_tx_timeout,
	.ndo_vlan_rx_add_vid	= am65_cpsw_nuss_ndo_slave_add_vid,
	.ndo_vlan_rx_kill_vid	= am65_cpsw_nuss_ndo_slave_kill_vid,
	.ndo_do_ioctl		= am65_cpsw_nuss_ndo_slave_ioctl,
	.ndo_set_features	= am65_cpsw_nuss_ndo_slave_set_features,
};

static void am65_cpsw_nuss_slave_disable_unused(struct am65_cpsw_port *port)
{
	struct am65_cpsw_common *common = port->common;

	if (!port->disabled)
		return;

	common->disabled_ports_mask |= BIT(port->port_id);
	cpsw_ale_control_set(common->ale, port->port_id,
			     ALE_PORT_STATE, ALE_PORT_STATE_DISABLE);

	cpsw_sl_reset(port->slave.mac_sl, 100);
	cpsw_sl_ctl_reset(port->slave.mac_sl);
}

static void am65_cpsw_nuss_free_tx_chns(void *data)
{
	struct am65_cpsw_common *common = data;
	int i;

	for (i = 0; i < AM65_CPSW_MAX_TX_QUEUES; i++) {
		struct am65_cpsw_tx_chn	*tx_chn = &common->tx_chns[i];

		if (tx_chn->irq > 0)
			k3_nav_udmax_tx_put_irq(tx_chn->tx_chn);

		if (!IS_ERR_OR_NULL(tx_chn->tx_chn))
			k3_nav_udmax_release_tx_chn(tx_chn->tx_chn);

		if (!IS_ERR_OR_NULL(tx_chn->desc_pool))
			k3_knav_pool_destroy(tx_chn->desc_pool);

		memset(tx_chn, 0, sizeof(*tx_chn));
	};
}

static int am65_cpsw_nuss_init_tx_chns(struct am65_cpsw_common *common)
{
	struct device *dev = common->dev;
	u32  max_desc_num = ALIGN(AM65_CPSW_MAX_TX_DESC, MAX_SKB_FRAGS);
	struct k3_nav_udmax_tx_channel_cfg tx_cfg = {0};
	struct k3_nav_ring_cfg ring_cfg = {
		.elm_size = K3_NAV_RINGACC_RING_ELSIZE_8,
		.mode = K3_NAV_RINGACC_RING_MODE_RING,
		.flags = 0
	};
	u32 hdesc_size;
	int i, ret;

	init_completion(&common->tdown_complete);

	hdesc_size = cppi5_hdesc_calc_size(true, AM65_CPSW_NAV_PS_DATA_SIZE,
					   AM65_CPSW_NAV_SW_DATA_SIZE);

	tx_cfg.swdata_size = AM65_CPSW_NAV_SW_DATA_SIZE;
	tx_cfg.tx_cfg = ring_cfg;
	tx_cfg.txcq_cfg = ring_cfg;
	tx_cfg.tx_cfg.size = max_desc_num;
	tx_cfg.txcq_cfg.size = max_desc_num;

	for (i = 0; i < AM65_CPSW_MAX_TX_QUEUES; i++) {
		struct am65_cpsw_tx_chn	*tx_chn = &common->tx_chns[i];
		char	tx_chn_name[IFNAMSIZ];

		snprintf(tx_chn_name, sizeof(tx_chn_name), "tx%d", i);

		tx_chn->dev = dev;
		tx_chn->id = i;
		tx_chn->descs_num = max_desc_num;
		tx_chn->desc_pool = k3_knav_pool_create_name(
					dev, tx_chn->descs_num,
					hdesc_size, tx_chn_name);
		if (IS_ERR(tx_chn->desc_pool)) {
			ret = PTR_ERR(tx_chn->desc_pool);
			dev_err(dev, "Failed to create poll %d\n", ret);
			goto err;
		}

		tx_chn->tx_chn =
			k3_nav_udmax_request_tx_chn(dev, tx_chn_name, &tx_cfg);
		if (IS_ERR(tx_chn->tx_chn)) {
			ret = PTR_ERR(tx_chn->tx_chn);
			dev_err(dev, "Failed to request tx dma channel %d\n",
				ret);
			goto err;
		}
		ret = k3_nav_udmax_tx_get_irq(
				tx_chn->tx_chn, &tx_chn->irq,
				IRQF_TRIGGER_HIGH, true,
				i ? common->tx_chns[0].tx_chn : NULL);
		if (ret) {
			dev_err(dev, "Failed to get tx dma irq %d\n", ret);
			goto err;
		}
	}

err:
	i = devm_add_action(dev, am65_cpsw_nuss_free_tx_chns, common);
	if (i) {
		dev_err(dev, "failed to add free_tx_chns action %d", i);
		return i;
	}

	return ret;
}

static void am65_cpsw_nuss_free_rx_chns(void *data)
{
	struct am65_cpsw_common *common = data;
	struct am65_cpsw_rx_chn *rx_chn = &common->rx_chns;
	int i;

	if (!IS_ERR_OR_NULL(rx_chn->rx_chn)) {
		for (i = 0; i < AM65_CPSW_MAX_RX_FLOWS; i++)
			k3_nav_udmax_rx_put_irq(rx_chn->rx_chn, i);

		k3_nav_udmax_release_rx_chn(rx_chn->rx_chn);
	}

	if (!IS_ERR_OR_NULL(rx_chn->desc_pool))
		k3_knav_pool_destroy(rx_chn->desc_pool);
}

static int am65_cpsw_nuss_init_rx_chns(struct am65_cpsw_common *common)
{
	struct device *dev = common->dev;
	u32  max_desc_num = AM65_CPSW_MAX_RX_DESC;
	struct k3_nav_udmax_rx_channel_cfg rx_cfg = {0};
	u32 hdesc_size;
	u32 fdqring_id;
	int i, ret = 0;
	struct am65_cpsw_rx_chn *rx_chn = &common->rx_chns;

	/* TODO: we need to calc total number of flows required
	 * N = n_mac_only*8 [+ 8 if switch ports] rest can be used for class
	 */

	hdesc_size = cppi5_hdesc_calc_size(true, AM65_CPSW_NAV_PS_DATA_SIZE,
					   AM65_CPSW_NAV_SW_DATA_SIZE);

	rx_cfg.swdata_size = AM65_CPSW_NAV_SW_DATA_SIZE;
	rx_cfg.flow_id_num = AM65_CPSW_MAX_RX_FLOWS;
	rx_cfg.flow_id_base = common->rx_flow_id_base;

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

	rx_chn->rx_chn = k3_nav_udmax_request_rx_chn(dev, "rx", &rx_cfg);
	if (IS_ERR(rx_chn->rx_chn)) {
		ret = PTR_ERR(rx_chn->rx_chn);
		dev_err(dev, "Failed to request rx dma channel %d\n", ret);
		goto err;
	}

	common->rx_flow_id_base =
			k3_nav_udmax_rx_get_flow_id_base(rx_chn->rx_chn);
	dev_info(dev, "set new flow-id-base %u\n", common->rx_flow_id_base);

	fdqring_id = K3_NAV_RINGACC_RING_ID_ANY;
	for (i = 0; i < rx_cfg.flow_id_num; i++) {
		struct k3_nav_ring_cfg rxring_cfg = {
			.elm_size = K3_NAV_RINGACC_RING_ELSIZE_8,
			.mode = K3_NAV_RINGACC_RING_MODE_RING,
			.flags = 0,
		};
		struct k3_nav_ring_cfg fdqring_cfg = {
			.elm_size = K3_NAV_RINGACC_RING_ELSIZE_8,
			.mode = K3_NAV_RINGACC_RING_MODE_MESSAGE,
			.flags = K3_NAV_RINGACC_RING_SHARED,
		};
		struct k3_nav_udmax_rx_flow_cfg rx_flow_cfg = {
			.rx_cfg = rxring_cfg,
			.rxfdq_cfg = fdqring_cfg,
			.ring_rxq_id = K3_NAV_RINGACC_RING_ID_ANY,
			.src_tag_lo_sel =
				K3_NAV_UDMAX_SRC_TAG_LO_USE_REMOTE_SRC_TAG,
		};

		rx_flow_cfg.ring_rxfdq0_id = fdqring_id;
		rx_flow_cfg.rx_cfg.size = max_desc_num;
		rx_flow_cfg.rxfdq_cfg.size = max_desc_num;

		ret = k3_nav_udmax_rx_flow_init(rx_chn->rx_chn,
						i, &rx_flow_cfg);
		if (ret) {
			dev_err(dev, "Failed to init rx flow%d %d\n", i, ret);
			goto err;
		}
		if (!i)
			fdqring_id = k3_nav_udmax_rx_flow_get_fdq_id(
					rx_chn->rx_chn, i);

		ret = k3_nav_udmax_rx_get_irq(rx_chn->rx_chn, i, &rx_chn->irq,
					      IRQF_TRIGGER_HIGH,
					      true, i ? 0 : -1);
		if (ret) {
			dev_err(dev, "Failed to get rx dma irq %d\n", ret);
			goto err;
		}
	}

err:
	i = devm_add_action(dev, am65_cpsw_nuss_free_rx_chns, common);
	if (i) {
		dev_err(dev, "failed to add free_rx_chns action %d", i);
		return i;
	}

	return ret;
}

static int am65_cpsw_nuss_init_host_p(struct am65_cpsw_common *common)
{
	struct am65_cpsw_port *host_p = am65_common_get_host_port(common);
	struct device *dev = common->dev;
	struct device_node *node, *host_np;

	host_p->common = common;

	node = of_get_child_by_name(dev->of_node, "ports");
	if (!node)
		return -ENOENT;
	host_np = of_get_child_by_name(node, "host");
	if (!host_np)
		return -ENOENT;

	host_p->name = of_get_property(host_np, "ti,label", NULL);

	of_node_put(host_np);
	of_node_put(node);

	host_p->port_base = common->cpsw_base + AM65_CPSW_NU_PORTS_BASE;
	host_p->stat_base = common->cpsw_base + AM65_CPSW_NU_STATS_BASE;

	return 0;
}

static int am65_cpsw_init_mdio(struct am65_cpsw_common *common)
{
	struct device *dev = common->dev;
	struct device_node *node;
	struct davinci_mdio_data *mdio;
	void __iomem *reg_base;

	node = of_get_child_by_name(dev->of_node, "mdio");
	if (!node)
		return -ENOENT;

	reg_base = common->ss_base + AM65_CPSW_MDIO_BASE;
	mdio = davinci_mdio_create(dev, node, reg_base, "fck");
	if (IS_ERR(mdio)) {
		of_node_put(node);
		dev_err(dev, "MDIO init err %ld\n", PTR_ERR(mdio));
		return PTR_ERR(mdio);
	}
	common->mdio = mdio;

	return 0;
}

static int am65_cpsw_am654_get_efuse_macid(struct device_node *of_node,
					   int slave, u8 *mac_addr)
{
	struct regmap *syscon;
	u32 mac_lo, mac_hi, offset;
	int ret;

	syscon = syscon_regmap_lookup_by_phandle(of_node, "ti,syscon-efuse");
	if (IS_ERR(syscon)) {
		if (PTR_ERR(syscon) == -ENODEV)
			return 0;
		return PTR_ERR(syscon);
	}

	ret = of_property_read_u32_index(of_node, "ti,syscon-efuse", 1,
					 &offset);
	if (ret)
		return ret;

	regmap_read(syscon, offset, &mac_lo);
	regmap_read(syscon, offset + 4, &mac_hi);

	mac_addr[0] = (mac_hi >> 8) & 0xff;
	mac_addr[1] = mac_hi & 0xff;
	mac_addr[2] = (mac_lo >> 24) & 0xff;
	mac_addr[3] = (mac_lo >> 16) & 0xff;
	mac_addr[4] = (mac_lo >> 8) & 0xff;
	mac_addr[5] = mac_lo & 0xff;

	return 0;
}

static int am65_cpsw_init_cpts(struct am65_cpsw_common *common)
{
	struct device *dev = common->dev;
	struct device_node *node;
	struct am65_cpts *cpts;
	void __iomem *reg_base;

	node = of_get_child_by_name(dev->of_node, "cpts");
	if (!node) {
		dev_err(dev, "%s cpts not found\n", __func__);
		return -ENOENT;
	}

	reg_base = common->cpsw_base + AM65_CPSW_NU_CPTS_BASE;
	cpts = am65_cpts_create(dev, reg_base, node);
	if (IS_ERR(cpts)) {
		int ret = PTR_ERR(cpts);

		if (ret == -EOPNOTSUPP) {
			dev_info(dev, "cpts disabled\n");
			return 0;
		}

		dev_err(dev, "cpts create err %d\n", ret);
		return ret;
	}
	common->cpts = cpts;

	return 0;
}

static int am65_cpsw_nuss_init_slave_ports(struct am65_cpsw_common *common)
{
	struct device *dev = common->dev;
	struct device_node *node, *port_np;
	int ret;

	node = of_get_child_by_name(dev->of_node, "ports");
	if (!node)
		return -ENOENT;

	for_each_child_of_node(node, port_np) {
		struct am65_cpsw_port *port;
		const void *mac_addr;
		u32 port_id;

		/* it is not a slave port node, continue */
		if (strcmp(port_np->name, "port"))
			continue;

		ret = of_property_read_u32(port_np, "reg", &port_id);
		if (ret < 0) {
			dev_err(dev, "%pOF error reading port_id %d\n",
				port_np, ret);
			return ret;
		}

		if (!port_id || port_id >= common->port_num) {
			dev_err(dev, "%pOF has invalid port_id %u %s\n",
				port_np, port_id, port_np->name);
			return -EINVAL;
		}

		port = &common->ports[port_id];
		port->port_id = port_id;
		port->common = common;
		port->port_base = common->cpsw_base + AM65_CPSW_NU_PORTS_BASE +
				  AM65_CPSW_NU_PORTS_OFFSET * (port_id);
		port->stat_base = common->cpsw_base + AM65_CPSW_NU_STATS_BASE +
				  (AM65_CPSW_NU_STATS_PORT_OFFSET * port_id);
		port->name = of_get_property(port_np, "ti,label", NULL);

		port->disabled = !of_device_is_available(port_np);
		if (port->disabled)
			continue;

		port->slave.mac_only =
				of_property_read_bool(port_np, "ti,mac-only");

		/* get phy/link info */
		if (of_phy_is_fixed_link(port_np)) {
			ret = of_phy_register_fixed_link(port_np);
			if (ret) {
				if (ret != -EPROBE_DEFER)
					dev_err(dev, "%pOF failed to register fixed-link phy: %d\n",
						port_np, ret);
				return ret;
			}
			port->slave.phy_node = of_node_get(port_np);
		} else {
			port->slave.phy_node =
				of_parse_phandle(port_np, "phy-handle", 0);
		}

		if (!port->slave.phy_node) {
			dev_err(dev,
				"slave[%d] no phy found\n", port_id);
			return -ENODEV;
		}

		port->slave.phy_if = of_get_phy_mode(port_np);
		if (port->slave.phy_if < 0) {
			dev_err(dev, "%pOF read phy-mode err %d\n",
				port_np, port->slave.phy_if);
			return port->slave.phy_if;
		}

		port->slave.mac_sl = cpsw_sl_get("am65", dev, port->port_base);
		if (IS_ERR(port->slave.mac_sl))
			return PTR_ERR(port->slave.mac_sl);

		mac_addr = of_get_mac_address(port_np);
		if (mac_addr && is_valid_ether_addr(mac_addr)) {
			ether_addr_copy(port->slave.mac_addr, mac_addr);
		} else if (am65_cpsw_am654_get_efuse_macid(
				port_np, port->port_id, port->slave.mac_addr)) {
			random_ether_addr(port->slave.mac_addr);
			dev_err(dev, "Use rundom MAC address\n");
		}

		/* alloc netdev */
		port->ndev = devm_alloc_etherdev_mqs(
				common->dev,
				sizeof(struct am65_cpsw_ndev_priv),
				common->tx_ch_num, common->rx_ch_num);
		if (!port->ndev) {
			dev_err(dev, "error allocating slave net_device %u\n",
				port_id);
			return -ENOMEM;
		}
	}
	of_node_put(node);

	return 0;
}

static int am65_cpsw_nuss_init_ndev_2g(struct am65_cpsw_common *common)
{
	struct device *dev = common->dev;
	struct am65_cpsw_ndev_priv *ndev_priv;
	struct am65_cpsw_port *port;
	int ret;

	port = &common->ports[1];

	/* alloc netdev */
	port->ndev = devm_alloc_etherdev_mqs(
			common->dev,
			sizeof(struct am65_cpsw_ndev_priv),
			common->tx_ch_num, common->rx_ch_num);
	if (!port->ndev) {
		dev_err(dev, "error allocating slave net_device %u\n",
			port->port_id);
		return -ENOMEM;
	}

	ndev_priv = netdev_priv(port->ndev);
	ndev_priv->port = port;
	ndev_priv->msg_enable = netif_msg_init(am65_cpsw_debug_level,
					       AM65_CPSW_DEBUG);
	SET_NETDEV_DEV(port->ndev, dev);

	ether_addr_copy(port->ndev->dev_addr, port->slave.mac_addr);

	port->ndev->min_mtu = AM65_CPSW_MIN_PACKET_SIZE;
	port->ndev->max_mtu = AM65_CPSW_MAX_PACKET_SIZE;
	port->ndev->hw_features = NETIF_F_SG |
				  NETIF_F_RXCSUM |
				  NETIF_F_HW_CSUM;
	port->ndev->features = port->ndev->hw_features |
			       NETIF_F_HW_VLAN_CTAG_FILTER;
	/* Disable TX checksum offload by default due to HW bug */
	port->ndev->features &= ~NETIF_F_HW_CSUM;
	port->ndev->vlan_features |=  NETIF_F_SG;
	port->ndev->netdev_ops = &am65_cpsw_nuss_netdev_ops_2g;
	port->ndev->ethtool_ops = &am65_cpsw_ethtool_ops_slave;

	ndev_priv->stats = netdev_alloc_pcpu_stats(struct am65_cpsw_ndev_stats);
	if (!ndev_priv->stats)
		return -ENOMEM;

	ret = devm_add_action_or_reset(dev, (void(*)(void *))free_percpu,
				       ndev_priv->stats);
	if (ret) {
		dev_err(dev, "failed to add percpu stat free action %d", ret);
		return ret;
	}

	netif_tx_napi_add(port->ndev, &common->napi_tx,
			  am65_cpsw_nuss_tx_poll, NAPI_POLL_WEIGHT);
	netif_tx_napi_add(port->ndev, &common->napi_rx,
			  am65_cpsw_nuss_rx_poll, NAPI_POLL_WEIGHT);

	common->pf_p0_rx_ptype_rrobin = true;
	ret = register_netdev(port->ndev);
	if (ret)
		dev_err(dev, "error registering slave net device %d\n", ret);

	/* can't auto unregister ndev using devm_add_action() due to broken
	 * devres release sequence in DD core
	 */

	return ret;
}

static void am65_cpsw_nuss_cleanup_ndev(struct am65_cpsw_common *common)
{
	struct am65_cpsw_port *port;
	int i;

	for (i = 0; i < common->port_num; i++) {
		port = &common->ports[i];
		if (port->ndev)
			unregister_netdev(port->ndev);
	}
}

static int am65_cpsw_nuss_probe(struct platform_device *pdev)
{
	struct cpsw_ale_params ale_params;
	struct device *dev = &pdev->dev;
	struct am65_cpsw_common *common;
	struct device_node *node;
	struct resource *res;
	struct clk *clk;
	int ret, i;

	common = devm_kzalloc(dev, sizeof(struct am65_cpsw_common), GFP_KERNEL);
	if (!common)
		return -ENOMEM;
	common->dev = dev;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "cpsw_nuss");
	common->ss_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(common->ss_base))
		return PTR_ERR(common->ss_base);
	common->cpsw_base = common->ss_base + AM65_CPSW_CPSW_NU_BASE;

	node = of_get_child_by_name(dev->of_node, "ports");
	if (!node)
		return -ENOENT;
	common->port_num = of_get_child_count(node);
	if (common->port_num < 2 || common->port_num > AM65_CPSW_MAX_PORTS)
		return -ENOENT;
	of_node_put(node);

	common->rx_flow_id_base = -1;
	ret = of_property_read_u32(dev->of_node, "ti,rx-flow-id-base",
				   &common->rx_flow_id_base);
	if (ret < 0)
		dev_info(dev, "rx-flow-id-base is not set %d\n", ret);

	clk = devm_clk_get(dev, "fck");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);

		if (ret != -EPROBE_DEFER)
			dev_err(dev, "error getting fck clock %d\n", ret);
		return ret;
	}
	common->bus_freq_mhz = clk_get_rate(clk) / 1000000;

	pm_runtime_enable(dev);
	ret = devm_add_action_or_reset(dev, (void(*)(void *))pm_runtime_disable,
				       dev);
	if (ret) {
		dev_err(dev, "failed to add pm reset action %d", ret);
		return ret;
	}

	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		pm_runtime_put_noidle(dev);
		return ret;
	}

	/* register devres action here, so dev will be disabled
	 * at right moment. It's required to enable dev in .remove() callback
	 * unconditionally.
	 */
	ret = devm_add_action_or_reset(dev,
				       (void(*)(void *))pm_runtime_put_sync,
				       dev);
	if (ret) {
		dev_err(dev, "failed to add pm put reset action %d", ret);
		return ret;
	}

	ret = of_platform_populate(dev->of_node, NULL, NULL, dev);
	/* We do not want to force this, as in some cases may not have child */
	if (ret)
		dev_warn(dev, "Doesn't have any child node\n");

	common->tx_ch_num = AM65_CPSW_MAX_TX_QUEUES;
	common->rx_ch_num = AM65_CPSW_MAX_RX_QUEUES;

	am65_cpsw_nuss_get_ver(common);

	common->ports = devm_kcalloc(dev, common->port_num,
				     sizeof(struct am65_cpsw_port),
				     GFP_KERNEL);
	if (!common->ports)
		return -ENOMEM;

	/* init tx channels */
	ret = am65_cpsw_nuss_init_tx_chns(common);
	if (ret)
		return ret;
	ret = am65_cpsw_nuss_init_rx_chns(common);
	if (ret)
		return ret;

	if (common->tx_chns[0].irq == 0 || common->rx_chns.irq == 0)
		return -ENXIO;

	ret = am65_cpsw_nuss_init_host_p(common);
	if (ret)
		return ret;

	ret = am65_cpsw_nuss_init_slave_ports(common);
	if (ret)
		return ret;

	ret = am65_cpsw_init_cpts(common);
	if (ret)
		return ret;

	/* init common data */
	ale_params.dev = dev;
	ale_params.ale_ageout = AM65_CPSW_ALE_AGEOUT_DEFAULT;
	ale_params.ale_entries = 0;
	ale_params.ale_ports = common->port_num;
	ale_params.ale_regs = common->cpsw_base + AM65_CPSW_NU_ALE_BASE;
	ale_params.nu_switch_ale = true;

	common->ale = cpsw_ale_create(&ale_params);
	if (!common->ale) {
		dev_err(dev, "error initializing ale engine\n");
		return -ENODEV;
	}

	ret = am65_cpsw_init_mdio(common);
	if (ret)
		return ret;

	/* init ports */
	for (i = 0; i < common->port_num; i++)
		am65_cpsw_nuss_slave_disable_unused(&common->ports[i]);

	if (common->port_num != 2)
		return -EOPNOTSUPP;

	dev_set_drvdata(dev, common);

	ret = am65_cpsw_nuss_init_ndev_2g(common);
	if (ret)
		return ret;

	ret = devm_request_irq(dev, common->tx_chns[0].irq,
			       am65_cpsw_nuss_tx_irq,
			       0, dev_name(dev), common);
	if (ret) {
		dev_err(dev, "failure requesting tx irq %u, %d\n",
			common->tx_chns[0].irq, ret);
		goto unreg_ndev;
	}

	ret = devm_request_irq(dev, common->rx_chns.irq,
			       am65_cpsw_nuss_rx_irq,
			       0, dev_name(dev), common);
	if (ret) {
		dev_err(dev, "failure requesting rx irq %u, %d\n",
			common->rx_chns.irq, ret);
		goto unreg_ndev;
	}

	pm_runtime_put(dev);
	return 0;

unreg_ndev:
	am65_cpsw_nuss_cleanup_ndev(common);
	return ret;
}

static int am65_cpsw_nuss_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct am65_cpsw_common *common = dev_get_drvdata(dev);
	int ret;

	/* enable dev here it will be released later by devres action */
	ret = pm_runtime_get_sync(&pdev->dev);
	if (ret < 0)
		pm_runtime_put_noidle(&pdev->dev);

	/* must unregister ndevs here because DD release_driver routine calls
	 * dma_deconfigure(dev) before devres_release_all(dev)
	 */
	am65_cpsw_nuss_cleanup_ndev(common);

	return 0;
}

static const struct of_device_id am65_cpsw_nuss_of_mtable[] = {
	{ .compatible = "ti,am654-cpsw-nuss", .data = NULL },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, am65_cpsw_nuss_of_mtable);

static struct platform_driver am65_cpsw_nuss_driver = {
	.driver = {
		.name	 = AM65_CPSW_DRV_NAME,
		.of_match_table = am65_cpsw_nuss_of_mtable,
	},
	.probe = am65_cpsw_nuss_probe,
	.remove = am65_cpsw_nuss_remove,
};

module_platform_driver(am65_cpsw_nuss_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Grygorii Strashko <grygorii.strashko@ti.com>");
MODULE_DESCRIPTION("TI AM65 CPSW Ethernet driver");
