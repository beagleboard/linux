/* SPDX-License-Identifier: GPL-2.0 */
/* Texas Instruments ICSSG Ethernet driver
 *
 * Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/
 *
 */

#ifndef __NET_TI_ICSSG_CONFIG_H
#define __NET_TI_ICSSG_CONFIG_H

/* Config area lies in shared RAM */
#define ICSSG_CONFIG_OFFSET_SLICE0   0
#define ICSSG_CONFIG_OFFSET_SLICE1   0x8000

/* Load time Fiwmware Configuration */
struct icssg_config {
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

#endif /* __NET_TI_ICSSG_CONFIG_H */
