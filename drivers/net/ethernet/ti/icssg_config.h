/* SPDX-License-Identifier: GPL-2.0 */
/* Texas Instruments ICSSG Ethernet driver
 *
 * Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/
 *
 */

#ifndef __NET_TI_ICSSG_CONFIG_H
#define __NET_TI_ICSSG_CONFIG_H

struct icssg_buffer_pool_cfg {
	__le32	addr;
	__le32	len;
} __packed;

struct icssg_flow_cfg {
	__le16 rx_base_flow;
	__le16 mgm_base_flow;
} __packed;

/*------------------------ SR1.0 related --------------------------*/

/* Port queue size in MSMC from firmware
 * PORTQSZ_HP .set (0x1800)
 * PORTQSZ_HP2 .set (PORTQSZ_HP+128) ;include barrier area
 * 0x1880 x 8 bytes per slice  (port)
 */

#define MSMC_RAM_SIZE_SR1	(SZ_64K + SZ_32K + SZ_2K) /* 0x1880 x 8 x 2 */

#define PRUETH_MAX_RX_MGM_DESC	8
#define PRUETH_MAX_RX_FLOWS_SR1	4	/* excluding default flow */
#define PRUETH_RX_FLOW_DATA_SR1	3       /* highest priority flow */
#define PRUETH_MAX_RX_MGM_FLOWS	2	/* excluding default flow */
#define PRUETH_RX_MGM_FLOW_RESPONSE	0
#define PRUETH_RX_MGM_FLOW_TIMESTAMP	1
#define PRUETH_RX_MGM_FLOW_OTHER	2

#define PRUETH_NUM_BUF_POOLS_SR1	16
#define PRUETH_EMAC_BUF_POOL_START_SR1	8
#define PRUETH_EMAC_BUF_POOL_SIZE_SR1	0x1800

/* Config area lies in shared RAM */
#define ICSSG_CONFIG_OFFSET_SLICE0   0
#define ICSSG_CONFIG_OFFSET_SLICE1   0x8000

struct icssg_config_sr1 {
	__le32 status;	/* Firmware status */
	__le32 addr_lo;	/* MSMC Buffer pool base address low. */
	__le32 addr_hi;	/* MSMC Buffer pool base address high. Must be 0 */
	__le32 tx_buf_sz[16];	/* Array of buffer pool sizes */
	__le32 num_tx_threads;	/* Number of active egress threads, 1 to 4 */
	__le32 tx_rate_lim_en;	/* Bitmask: Egress rate limit en per thread */
	__le32 rx_flow_id;	/* RX flow id for first rx ring */
	__le32 rx_mgr_flow_id;	/* RX flow id for the first management ring */
	__le32 flags;		/* TBD */
	__le32 n_burst;		/* for debug */
	__le32 rtu_status;	/* RTU status */
	__le32 info;		/* reserved */
} __packed;

/* Shutdown command to stop processing at firmware.
 * Command format : 0x8101ss00. ss - sequence number. Currently not used
 * by driver.
 */
#define ICSSG_SHUTDOWN_CMD		0x81010000

/* pstate speed/duplex command to set speed and duplex settings
 * in firmware.
 * Command format : 0x8102ssPN. ss - sequence number: currently not
 * used by driver, P - port number: For switch, N - Speed/Duplex state
 * - Possible values of N:
 * 0x0 - 10Mbps/Half duplex ;
 * 0x8 - 10Mbps/Full duplex ;
 * 0x2 - 100Mbps/Half duplex;
 * 0xa - 100Mbps/Full duplex;
 * 0xc - 1Gbps/Full duplex;
 * NOTE: The above are same as bits [3..1](slice 0) or bits [8..6](slice 1) of
 * RGMII CFG register. So suggested to read the register to populate the command
 * bits.
 */
#define ICSSG_PSTATE_SPEED_DUPLEX_CMD	0x81020000

/*------------------------ SR2.0 related --------------------------*/

#define PRUETH_PKT_TYPE_CMD	0x10
#define PRUETH_NAV_PS_DATA_SIZE	16	/* Protocol specific data size */
#define PRUETH_NAV_SW_DATA_SIZE	16	/* SW related data size */
#define PRUETH_MAX_TX_DESC	512
#define PRUETH_MAX_RX_DESC	512
#define PRUETH_MAX_RX_FLOWS_SR2	5	/* excluding default flow */
#define PRUETH_RX_FLOW_DATA_SR2	0	/* FIXME: f/w bug to change to highest priority flow */

#define PRUETH_EMAC_BUF_POOL_SIZE_SR2	SZ_8K
#define PRUETH_EMAC_POOLS_PER_SLICE	24
#define PRUETH_EMAC_BUF_POOL_START_SR2	8
#define PRUETH_NUM_BUF_POOLS_SR2	8
#define PRUETH_EMAC_RX_CTX_BUF_SIZE	SZ_16K	/* per slice */
#define MSMC_RAM_SIZE_SR2	(2 * (PRUETH_EMAC_BUF_POOL_SIZE_SR2 * PRUETH_NUM_BUF_POOLS_SR2 + PRUETH_EMAC_RX_CTX_BUF_SIZE))

struct icssg_rxq_ctx {
	__le32 start[3];
	__le32 end;
} __packed;

/* Load time Fiwmware Configuration */

struct icssg_cmd {
	/* Optional parameter for TX to FW, ERROR code for RX from FW */
	u8 param;
	u8 seq;
	/* Command Type */
	u8 type;
	/* Command Header */
	u8 hdr;
	__le32 data[3];
} __packed;

#define ICSSG_FW_MGMT_CMD_HEADER	0x81
#define ICSSG_FW_MGMT_FDB_CMD_TYPE	0x03
#define ICSSG_FW_MGMT_CMD_TYPE		0x04
#define ICSSG_FW_MGMT_PKT		0x80000000

enum icssg_port_state_cmd {
	ICSSG_PORT_STATE_DISABLE = 2,
	ICSSG_PORT_STATE_BLOCKING,
	ICSSG_PORT_STATE_FORWARD,
	ICSSG_PORT_STATE_FORWARD_WO_LEARNING,
	ICSSG_PORT_STATE_TAS_TRIGGER,
	ICSSG_PORT_STATE_TAS_ENABLE,
	ICSSG_PORT_STATE_TAS_RESET,
	ICSSG_PORT_STATE_TAS_DISABLE,
};

/* Config area lies in DRAM */
#define ICSSG_CONFIG_OFFSET			0x0

#define ICSSG_NUM_NORMAL_PDS	64
#define ICSSG_NUM_SPECIAL_PDS	16

#define ICSSG_NORMAL_PD_SIZE	8
#define ICSSG_SPECIAL_PD_SIZE	20

#define ICSSG_FLAG_MASK		0xff00ffff

struct icssg_setclock_desc {
	u8 request;
	u8 restore;
	u8 acknowledgment;
	u8 cmp_status;
	u32 margin;
	u32 cyclecounter0_set;
	u32 cyclecounter1_set;
	u32 iepcount_set;
	u32 rsvd1;
	u32 rsvd2;
	u32 CMP0_current;
	u32 iepcount_current;
	u32 difference;
	u32 cyclecounter0_new;
	u32 cyclecounter1_new;
	u32 CMP0_new;
} __packed;

#define ICSSG_CMD_POP_SLICE0	56
#define ICSSG_CMD_POP_SLICE1	60

#define ICSSG_CMD_PUSH_SLICE0	57
#define ICSSG_CMD_PUSH_SLICE1	61

#define ICSSG_RSP_POP_SLICE0	58
#define ICSSG_RSP_POP_SLICE1	62

#define ICSSG_RSP_PUSH_SLICE0	56
#define ICSSG_RSP_PUSH_SLICE1	60

#define ICSSG_TS_POP_SLICE0	59
#define ICSSG_TS_POP_SLICE1	63

#define ICSSG_TS_PUSH_SLICE0	40
#define ICSSG_TS_PUSH_SLICE1	41

#endif /* __NET_TI_ICSSG_CONFIG_H */
