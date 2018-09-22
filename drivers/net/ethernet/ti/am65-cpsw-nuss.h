/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2016-2017 Texas Instruments Incorporated - http://www.ti.com/
 *
 */

#ifndef AM65_CPSW_NUSS_H_
#define AM65_CPSW_NUSS_H_

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>

#define AM65_CPSW_DEBUG	(NETIF_MSG_HW		| NETIF_MSG_WOL		| \
			 NETIF_MSG_DRV		| NETIF_MSG_LINK	| \
			 NETIF_MSG_IFUP		| NETIF_MSG_INTR	| \
			 NETIF_MSG_PROBE	| NETIF_MSG_TIMER	| \
			 NETIF_MSG_IFDOWN	| NETIF_MSG_RX_ERR	| \
			 NETIF_MSG_TX_ERR	| NETIF_MSG_TX_DONE	| \
			 NETIF_MSG_PKTDATA	| NETIF_MSG_TX_QUEUED	| \
			 NETIF_MSG_RX_STATUS)

#define	am65_nav_dbg(dev, arg...) dev_err(dev, arg)

#define HOST_PORT_NUM		0

#define AM65_CPSW_MAX_TX_QUEUES	8
#define AM65_CPSW_MAX_RX_QUEUES	1
#define AM65_CPSW_MAX_RX_FLOWS	8

struct am65_cpsw_slave_data {
	bool				mac_only;
	struct cpsw_sl			*mac_sl;
	struct device_node		*phy_node;
	struct phy_device		*phy;
	int				phy_if;
	bool				rx_pause;
	bool				tx_pause;
	u8				mac_addr[ETH_ALEN];
};

struct am65_cpsw_port {
	struct am65_cpsw_common		*common;
	struct net_device		*ndev;
	const char			*name;
	u32				port_id;
	void __iomem			*port_base;
	void __iomem			*stat_base;
	bool				disabled;
	struct am65_cpsw_slave_data	slave;
	bool				tx_ts_enabled;
	bool				rx_ts_enabled;
};

struct am65_cpsw_tx_chn {
	struct device *dev;
	struct k3_knav_desc_pool *desc_pool;
	struct k3_nav_udmax_tx_channel *tx_chn;
	u32 descs_num;
	unsigned int irq;
	u32 id;
};

struct am65_cpsw_rx_chn {
	struct device *dev;
	struct k3_knav_desc_pool *desc_pool;
	struct k3_nav_udmax_rx_channel *rx_chn;
	u32 descs_num;
	unsigned int irq;
};

struct am65_cpsw_common {
	struct device		*dev;

	void __iomem		*ss_base;
	void __iomem		*cpsw_base;

	u32			port_num;
	struct am65_cpsw_port	*ports;
	u32			disabled_ports_mask;

	int			usage_count; /* number of opened ports */
	struct cpsw_ale		*ale;
	int			rx_ch_num;
	int			tx_ch_num;
	u32			rx_flow_id_base;

	struct am65_cpsw_tx_chn	tx_chns[AM65_CPSW_MAX_TX_QUEUES];
	struct napi_struct	napi_tx;
	struct completion	tdown_complete;
	atomic_t		tdown_cnt;

	struct am65_cpsw_rx_chn	rx_chns;
	struct napi_struct	napi_rx;

	u32			nuss_ver;
	u32			cpsw_ver;
	u32			bus_freq_mhz;
	struct davinci_mdio_data *mdio;
	struct am65_cpts *cpts;

	bool			pf_p0_rx_ptype_rrobin;
	u32			cur_txq;
};

struct am65_cpsw_ndev_stats {
	u64 tx_packets;
	u64 tx_bytes;
	u64 rx_packets;
	u64 rx_bytes;
	struct u64_stats_sync syncp;
};

struct am65_cpsw_ndev_priv {
	u32			msg_enable;
	struct am65_cpsw_port	*port;
	struct am65_cpsw_ndev_stats __percpu *stats;
};

#define am65_ndev_to_priv(ndev) \
	((struct am65_cpsw_ndev_priv *)netdev_priv(ndev))
#define am65_ndev_to_port(ndev) (am65_ndev_to_priv(ndev)->port)
#define am65_ndev_to_common(ndev) (am65_ndev_to_port(ndev)->common)
#define am65_ndev_to_slave(ndev) (&am65_ndev_to_port(ndev)->slave)

#define am65_common_get_host_port(common) (&(common)->ports[HOST_PORT_NUM])

#define am65_cpsw_napi_to_common(napi) \
	container_of(napi, struct am65_cpsw_common, napi)

#define AM65_CPSW_DRV_NAME "am65-cpsw-nuss"

#define AM65_CPSW_IS_CPSW2G(common) (common->port_num == 2)

extern const struct ethtool_ops am65_cpsw_ethtool_ops_slave;

void am65_cpsw_nuss_adjust_link(struct net_device *ndev);
void am65_cpsw_nuss_set_p0_ptype(struct am65_cpsw_common *common);

#define	am65_ndev_dbg(ndev, arg...) \
	do {                                                          \
		struct device *dev = am65_ndev_to_common(ndev)->dev;  \
		dev_err(dev, arg); \
	} while (0)

#endif /* AM65_CPSW_NUSS_H_ */
