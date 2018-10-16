// SPDX-License-Identifier: GPL-2.0
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
#include <linux/phy.h>
#include <linux/pruss.h>
#include <linux/remoteproc.h>

#include <linux/dma-mapping.h>
#include <linux/dma/ti-cppi5.h>
#include <linux/dma/k3-navss-udma.h>

#define ICSS_SLICE0	0
#define ICSS_SLICE1	1

#define ICSS_FW_PRU	0
#define ICSS_FW_RTU	1

/* Handshake region lies in shared RAM */
#define ICSS_HS_OFFSET_SLICE0	0
#define ICSS_HS_OFFSET_SLICE1	0x8000

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

/* Fiwmware Handshake */
struct icss_hs {
			/* Owner : Description */
	__le32 fw_status;	/* FW : firmware boot status */
	__le32 cmd;		/* bits 31 and 30 owned by Firwmware, rest by Host */
	__le16 log_len;		/* FW: length of log area (in 32-bit words) */
	__le16 ilen_max;	/* FW:  max. length of input params (32-bit words) */
	__le32 ilen;		/* HOST: number of input parameters (32-bit words) */
	__le32 ioffset;		/* FW: offset to input parameters */
	__le32 olen;		/* FW: length of output data (in 32-bit words) */
	__le32 ooffset;		/* FW: offset to output data */
	__le32 log_offset;	/* FW: offset to log area */
} __packed;

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
	struct k3_nav_udmax_tx_channel *tx_chn;
	u32 descs_num;
	spinlock_t lock;	/* to serialize */
	unsigned int irq;
};

struct prueth_rx_chn {
	struct k3_knav_desc_pool *desc_pool;
	struct k3_nav_udmax_rx_channel *rx_chn;
	u32 descs_num;
	spinlock_t lock;	/* to serialize */
	unsigned int irq;
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

	/* DMA related */
	struct prueth_tx_chn tx_chns;
	struct completion tdown_complete;
	struct prueth_rx_chn rx_chns;
	int rx_flow_id_base;

	spinlock_t lock;	/* serialize access */
	unsigned int flags;
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
 * @hs: firmware handshake data per slice
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
	struct icss_hs hs[PRUSS_NUM_PRUS];
	struct regmap *miig_rt;
};

bool icss_hs_is_fw_dead(struct prueth *prueth, int slice, u16 *err_code);
bool icss_hs_is_fw_ready(struct prueth *prueth, int slice);
int icss_hs_send_cmd(struct prueth *prueth, int slice, u32 cmd,
		     u32 *idata, u32 ilen);
void icss_hs_cmd_cancel(struct prueth *prueth, int slice);
bool icss_hs_is_cmd_done(struct prueth *prueth, int slice);
int icss_hs_send_cmd_wait_done(struct prueth *prueth, int slice,
			       u32 cmd, u32 *idata, u32 ilen);
int icss_hs_get_result(struct prueth *prueth, int slice, u32 *odata, u32 olen);

/* Classifier helpers */
void icssg_class_set_mac_addr(struct regmap *miig_rt, int slice, u8 *mac);
void icssg_class_disable(struct regmap *miig_rt, int slice);
void icssg_class_default(struct regmap *miig_rt, int slice);
void icssg_class_promiscuous(struct regmap *miig_rt, int slice);

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
