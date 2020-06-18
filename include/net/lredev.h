/* SPDX-License-Identifier: GPL-2.0 */
/*
 * include/net/lredev.h - lre device API
 * Copyright (c) 2020 Texas Instruments Incorporated - http://www.ti.com/
 */
#ifndef _LINUX_LREDEV_H_
#define _LINUX_LREDEV_H_

#include <linux/netdevice.h>

/* PRP duplicate discard modes */
enum iec62439_3_dd_modes {
	IEC62439_3_DA = 1,
	IEC62439_3_DD,
};

/* HSR modes */
enum iec62439_3_hsr_modes {
	IEC62439_3_HSR_MODE_H = 1,
	IEC62439_3_HSR_MODE_N,
	IEC62439_3_HSR_MODE_T,
	IEC62439_3_HSR_MODE_U,
	IEC62439_3_HSR_MODE_M,
};

/* PRP Transparent reception modes */
enum iec62439_3_tr_modes {
	IEC62439_3_TR_REMOVE_RCT = 1,
	IEC62439_3_TR_PASS_RCT,
};

enum iec62439_3_clear_nt_cmd {
	IEC62439_3_CLEAR_NT_NO_OP,
	IEC62439_3_CLEAR_NT,
};

enum lredev_attr_id {
	LREDEV_ATTR_ID_UNDEFINED,
	/* For HSR only */
	LREDEV_ATTR_ID_HSR_MODE,
	/* Duplicate discard for PRP */
	LREDEV_ATTR_ID_DD_MODE,
	/* Duplicate List Reside Max time for HSR & PRP */
	LREDEV_ATTR_ID_DLRMT,
	/* Transparent reception for PRP */
	LREDEV_ATTR_ID_PRP_TR,
	/* Clear node table - for HSR & PRP */
	LREDEV_ATTR_ID_CLEAR_NT,
};

struct lredev_attr {
	enum lredev_attr_id id;
	union {
		enum iec62439_3_hsr_modes mode;
		enum iec62439_3_dd_modes dd_mode;
		/* in msec */
		u32  dl_reside_max_time;
		enum iec62439_3_tr_modes tr_mode;
		enum iec62439_3_clear_nt_cmd clear_nt_cmd;
	};
};

enum iec62439_node_type {
	IEC62439_3_DANP,
	IEC62439_3_REDBOXP,
	IEC62439_3_VDANP,
	IEC62439_3_DANH,
	IEC62439_3_REDBOXH,
	IEC62439_3_VDANH,
};

struct lre_node_table_entry {
	u8 mac_address[ETH_ALEN];
	/* in 1/100 seconds */
	u32 time_last_seen_a;
	/* in 1/100 seconds */
	u32 time_last_seen_b;
	enum iec62439_node_type node_type;
};

#define LRE_MAX_NT_ENTRIES	256

struct lre_stats {
	u32 cnt_tx_a;
	u32 cnt_tx_b;
	u32 cnt_tx_c;
	u32 cnt_errwronglan_a;
	u32 cnt_errwronglan_b;
	u32 cnt_errwronglan_c;
	u32 cnt_rx_a;
	u32 cnt_rx_b;
	u32 cnt_rx_c;
	u32 cnt_errors_a;
	u32 cnt_errors_b;
	u32 cnt_errors_c;
	u32 cnt_nodes;
	u32 cnt_proxy_nodes;
	u32 cnt_unique_rx_a;
	u32 cnt_unique_rx_b;
	u32 cnt_unique_rx_c;
	u32 cnt_duplicate_rx_a;
	u32 cnt_duplicate_rx_b;
	u32 cnt_duplicate_rx_c;
	u32 cnt_multiple_rx_a;
	u32 cnt_multiple_rx_b;
	u32 cnt_multiple_rx_c;
	u32 cnt_own_rx_a;
	u32 cnt_own_rx_b;
};

/**
 * struct lredev_ops - lredev operations
 *
 * @lredev_attr_get: Get a attribute (see switchdev_attr).
 *
 * @lredev_port_attr_set: Set a attribute (see switchdev_attr).
 */
struct lredev_ops {
	int	(*lredev_attr_get)(struct net_device *dev,
				   struct lredev_attr *attr);
	int	(*lredev_attr_set)(struct net_device *dev,
				   struct lredev_attr *attr);
	int	(*lredev_get_node_table)(struct net_device *dev,
					 struct lre_node_table_entry table[],
					 int size);
	int	(*lredev_get_stats)(struct net_device *dev,
				    struct lre_stats *stats);
};

#endif
