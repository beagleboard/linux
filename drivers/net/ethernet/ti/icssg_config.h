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

#endif /* __NET_TI_ICSSG_CONFIG_H */
