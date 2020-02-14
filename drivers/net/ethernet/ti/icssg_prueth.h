/* SPDX-License-Identifier: GPL-2.0 */
/* Texas Instruments ICSSG Ethernet driver
 *
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 *
 */

#ifndef __NET_TI_ICSSG_PRUETH_H
#define __NET_TI_ICSSG_PRUETH_H

#include <linux/etherdevice.h>
#include <linux/genalloc.h>
#include <linux/if_vlan.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/of_platform.h>
#include <linux/mfd/syscon.h>
#include <linux/mutex.h>
#include <linux/net_tstamp.h>
#include <linux/phy.h>
#include <linux/pruss.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/remoteproc.h>

#include <linux/dma-mapping.h>
#include <linux/dma/ti-cppi5.h>
#include <linux/dma/k3-udma-glue.h>

#include "icssg_config.h"
#include "icssg_iep.h"

#define ICSS_SLICE0	0
#define ICSS_SLICE1	1

#define ICSS_FW_PRU	0
#define ICSS_FW_RTU	1

/* Firmware status codes */
#define ICSS_HS_FW_READY 0x55555555
#define ICSS_HS_FW_DEAD 0xDEAD0000	/* lower 16 bits contain error code */

/* Firmware command codes */
#define ICSS_HS_CMD_BUSY 0x40000000
#define ICSS_HS_CMD_DONE 0x80000000
#define ICSS_HS_CMD_CANCEL 0x10000000

/* Firmware commands */
#define ICSS_CMD_SPAD 0x20
#define ICSS_CMD_RXTX 0x10
#define ICSS_CMD_ADD_FDB 0x1
#define ICSS_CMD_SET_RUN 0x4
#define ICSS_CMD_ENABLE_VLAN 0x5
#define ICSS_CMD_DISABLE_VLAN 0x6
#define ICSS_CMD_ADD_FILTER 0x7
#define ICSS_CMD_ADD_MAC 0x8

/* Firmware flags */
#define ICSS_SET_RUN_FLAG_VLAN_ENABLE		BIT(0)	/* switch only */
#define ICSS_SET_RUN_FLAG_FLOOD_UNICAST		BIT(1)	/* switch only */
#define ICSS_SET_RUN_FLAG_PROMISC		BIT(2)	/* MAC only */
#define ICSS_SET_RUN_FLAG_MULTICAST_PROMISC	BIT(3)	/* MAC only */

/* In switch mode there are 3 real ports i.e. 3 mac addrs.
 * however Linux sees only the host side port. The other 2 ports
 * are the switch ports.
 * In emac mode there are 2 real ports i.e. 2 mac addrs.
 * Linux sees both the ports.
 */
enum prueth_port {
	PRUETH_PORT_HOST = 0,	/* host side port */
	PRUETH_PORT_MII0,	/* physical port RG/SG MII 0 */
	PRUETH_PORT_MII1,	/* physical port RG/SG MII 1 */
};

enum prueth_mac {
	PRUETH_MAC0 = 0,
	PRUETH_MAC1,
	PRUETH_NUM_MACS,
};

struct prueth_tx_chn {
	struct k3_knav_desc_pool *desc_pool;
	struct k3_udma_glue_tx_channel *tx_chn;
	u32 descs_num;
	spinlock_t lock;	/* to serialize */
	unsigned int irq;
};

struct prueth_rx_chn {
	struct device *dev;
	struct k3_knav_desc_pool *desc_pool;
	struct k3_udma_glue_rx_channel *rx_chn;
	u32 descs_num;
	spinlock_t lock;	/* to serialize */
	unsigned int irq[4];	/* separate irq per flow */
};

enum prueth_state_flags {
	__STATE_TX_TS_IN_PROGRESS,
};

/* data for each emac port */
struct prueth_emac {
	struct prueth *prueth;
	struct net_device *ndev;
	u8 mac_addr[6];
	struct napi_struct napi_tx;
	struct napi_struct napi_rx;
	u32 msg_enable;

	int link;
	int speed;
	int duplex;

	const char *phy_id;
	struct device_node *phy_node;
	int phy_if;
	struct phy_device *phydev;
	enum prueth_port port_id;
	struct icssg_iep iep;
	struct hwtstamp_config tstamp_config;
	unsigned int rx_ts_enabled : 1;
	unsigned int tx_ts_enabled : 1;

	/* DMA related */
	struct prueth_tx_chn tx_chns;
	struct completion tdown_complete;
	struct prueth_rx_chn rx_chns;
	int rx_flow_id_base;
	struct prueth_rx_chn rx_mgm_chn;
	int rx_mgm_flow_id_base;

	spinlock_t lock;	/* serialize access */

	/* TX HW Timestamping */
	u32 tx_ts_cookie;
	struct sk_buff *tx_ts_skb;
	unsigned long state;

	/* shutdown related */
	u32 cmd_data[4];
	struct completion cmd_complete;
	/* Mutex to serialize access to firmware command interface */
	struct mutex cmd_lock;
};

/**
 * struct prueth - PRUeth structure
 * @dev: device
 * @pruss: pruss handle
 * @pru0: rproc instance of PRUs
 * @rtu0: rproc instance of RTUs
 * @shram: PRUSS shared RAM region
 * @sram_pool: MSMC RAM pool for buffers
 * @msmcram: MSMC RAM region
 * @eth_node: DT node for the port
 * @emac: private EMAC data structure
 * @registered_netdevs: list of registered netdevs
 * @fw_data: firmware names to be used with PRU remoteprocs
 * @config: firmware load time configuration per slice
 * @miig_rt: regmap to mii_g_rt block
 */
struct prueth {
	struct device *dev;
	struct pruss *pruss;
	struct rproc *pru[PRUSS_NUM_PRUS];
	struct rproc *rtu[PRUSS_NUM_PRUS];
	struct pruss_mem_region shram;
	struct gen_pool *sram_pool;
	struct pruss_mem_region msmcram;

	struct device_node *eth_node[PRUETH_NUM_MACS];
	struct prueth_emac *emac[PRUETH_NUM_MACS];
	struct net_device *registered_netdevs[PRUETH_NUM_MACS];
	const struct prueth_private_data *fw_data;
	struct icssg_config config[PRUSS_NUM_PRUS];
	struct regmap *miig_rt;
	struct regmap *mii_rt;
};

struct emac_tx_ts_response {
	u32 lo_ts;
	u32 hi_ts;
	u32 reserved;
	u32 cookie;
};

/* Classifier helpers */
void icssg_class_set_mac_addr(struct regmap *miig_rt, int slice, u8 *mac);
void icssg_class_disable(struct regmap *miig_rt, int slice);
void icssg_class_default(struct regmap *miig_rt, int slice, bool allmulti);
void icssg_class_promiscuous(struct regmap *miig_rt, int slice);
void icssg_class_add_mcast(struct regmap *miig_rt, int slice,
			   struct net_device *ndev);

/* get PRUSS SLICE number from prueth_emac */
static inline int prueth_emac_slice(struct prueth_emac *emac)
{
	switch (emac->port_id) {
	case PRUETH_PORT_MII0:
		return ICSS_SLICE0;
	case PRUETH_PORT_MII1:
		return ICSS_SLICE1;
	default:
		return -EINVAL;
	}
}
#endif /* __NET_TI_ICSSG_PRUETH_H */
