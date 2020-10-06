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
#include "icss_iep.h"
#include "icssg_switch_map.h"

#define ICSS_SLICE0	0
#define ICSS_SLICE1	1

#define ICSS_FW_PRU	0
#define ICSS_FW_RTU	1

#define ICSSG_MAX_RFLOWS	8	/* per slice */

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
	struct napi_struct napi_tx;
	struct k3_cppi_desc_pool *desc_pool;
	struct k3_udma_glue_tx_channel *tx_chn;
	struct prueth_emac *emac;
	u32 id;
	u32 descs_num;
	unsigned int irq;
	char name[32];
};

struct prueth_rx_chn {
	struct device *dev;
	struct k3_cppi_desc_pool *desc_pool;
	struct k3_udma_glue_rx_channel *rx_chn;
	u32 descs_num;
	unsigned int irq[ICSSG_MAX_RFLOWS];	/* separate irq per flow */
	char name[32];
};

enum prueth_state_flags {
	__STATE_TX_TS_IN_PROGRESS,
};

/* There are 4 Tx DMA channels, but the highest priority is CH3 (thread 3)
 * and lower three are lower priority channels or threads.
 */
#define PRUETH_MAX_TX_QUEUES	4

/* data for each emac port */
struct prueth_emac {
	bool is_sr1;
	bool fw_running;
	struct prueth *prueth;
	struct net_device *ndev;
	u8 mac_addr[6];
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
	struct icss_iep *iep;
	bool iep_initialized;
	struct hwtstamp_config tstamp_config;
	unsigned int rx_ts_enabled : 1;
	unsigned int tx_ts_enabled : 1;

	/* DMA related */
	struct prueth_tx_chn tx_chns[PRUETH_MAX_TX_QUEUES];
	struct completion tdown_complete;
	atomic_t tdown_cnt;
	struct prueth_rx_chn rx_chns;
	int rx_flow_id_base;
	int tx_ch_num;

	/* SR1.0 Management channel */
	struct prueth_rx_chn rx_mgm_chn;
	int rx_mgm_flow_id_base;

	spinlock_t lock;	/* serialize access */

	/* TX HW Timestamping */
	u32 tx_ts_cookie;
	struct sk_buff *tx_ts_skb;
	unsigned long state;
	int irq;

	u8 cmd_seq;
	/* shutdown related */
	u32 cmd_data[4];
	struct completion cmd_complete;
	/* Mutex to serialize access to firmware command interface */
	struct mutex cmd_lock;

	struct work_struct ts_work;
	struct pruss_mem_region dram;
};

/**
 * struct prueth - PRUeth structure
 * @is_sr1: device is pg1.0 (pg1.0 will be deprecated upstream)
 * @dev: device
 * @pruss: pruss handle
 * @pru: rproc instances of PRUs
 * @rtu: rproc instances of RTUs
 * @rtu: rproc instances of TX_PRUs
 * @shram: PRUSS shared RAM region
 * @sram_pool: MSMC RAM pool for buffers
 * @msmcram: MSMC RAM region
 * @eth_node: DT node for the port
 * @emac: private EMAC data structure
 * @registered_netdevs: list of registered netdevs
 * @fw_data: firmware names to be used with PRU remoteprocs
 * @config: firmware load time configuration per slice
 * @miig_rt: regmap to mii_g_rt block
 * @pa_stats: regmap to pa_stats block
 */
struct prueth {
	bool is_sr1;
	struct device *dev;
	struct pruss *pruss;
	struct rproc *pru[PRUSS_NUM_PRUS];
	struct rproc *rtu[PRUSS_NUM_PRUS];
	struct rproc *txpru[PRUSS_NUM_PRUS];
	struct pruss_mem_region shram;
	struct gen_pool *sram_pool;
	struct pruss_mem_region msmcram;

	struct device_node *eth_node[PRUETH_NUM_MACS];
	struct prueth_emac *emac[PRUETH_NUM_MACS];
	struct net_device *registered_netdevs[PRUETH_NUM_MACS];
	const struct prueth_private_data *fw_data;
	struct icssg_config_sr1 config[PRUSS_NUM_PRUS];
	struct regmap *miig_rt;
	struct regmap *mii_rt;
	struct regmap *pa_stats;
};

struct emac_tx_ts_response_sr1 {
	u32 lo_ts;
	u32 hi_ts;
	u32 reserved;
	u32 cookie;
};

struct emac_tx_ts_response {
	u32 reserved[2];
	u32 cookie;
	u32 lo_ts;
	u32 hi_ts;
};

static inline u64 icssg_ts_to_ns(u32 hi_sw, u32 hi, u32 lo, u32 cycle_time_ns)
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

/* Classifier helpers */
void icssg_class_set_mac_addr(struct regmap *miig_rt, int slice, u8 *mac);
void icssg_class_disable(struct regmap *miig_rt, int slice);
void icssg_class_default(struct regmap *miig_rt, int slice, bool allmulti,
			 bool is_sr1);
void icssg_class_promiscuous_sr1(struct regmap *miig_rt, int slice);
void icssg_class_add_mcast_sr1(struct regmap *miig_rt, int slice,
			       struct net_device *ndev);
void icssg_ft1_set_mac_addr(struct regmap *miig_rt, int slice, u8 *mac_addr);

/* Buffer queue helpers */
int icssg_queue_pop(struct prueth *prueth, u8 queue);
void icssg_queue_push(struct prueth *prueth, int queue, u16 addr);
u32 icssg_queue_level(struct prueth *prueth, int queue);

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

/* config helpers */
void icssg_config_ipg(struct prueth *prueth, int speed, int mii);
void icssg_config_sr1(struct prueth *prueth, struct prueth_emac *emac,
		      int slice);
int icssg_config_sr2(struct prueth *prueth, struct prueth_emac *emac,
		     int slice);
int emac_set_port_state(struct prueth_emac *emac,
			enum icssg_port_state_cmd state);
#define prueth_napi_to_tx_chn(pnapi) \
	container_of(pnapi, struct prueth_tx_chn, napi_tx)

#endif /* __NET_TI_ICSSG_PRUETH_H */
