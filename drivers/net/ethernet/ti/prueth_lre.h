/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com
 */

#ifndef __NET_TI_PRUETH_LRE_H
#define __NET_TI_PRUETH_LRE_H

#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/if_vlan.h>

#include "prueth.h"
#include "icss_lre_firmware.h"

#define PRUETH_MAX_PKTLEN_LRE		(VLAN_ETH_FRAME_LEN +  ETH_FCS_LEN + \
					 ICSS_LRE_TAG_RCT_SIZE)
#define PRUETH_MAC_QUEUE_MAX_SHIFT			6
#define PRUETH_MAC_QUEUE_MAX		BIT(PRUETH_MAC_QUEUE_MAX_SHIFT)
#define PRUETH_LRE_INDEX_TBL_MAX_ENTRIES	256
#define PRUETH_LRE_BIN_TBL_MAX_ENTRIES		256
#define PRUETH_LRE_NODE_TBL_MAX_ENTRIES		256
#define LRE_PROTO_HSR			0
#define LRE_PROTO_PRP			1
#define LRE_OK				0
#define LRE_ERR				-1
#define LRE_SV_FRAME_OFFSET		20

/* Link Redundancy Entity stats counters */
struct lre_statistics {
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

	u32 duplicate_discard;
	u32 transparent_reception;

	u32 node_table_lookup_error_a;
	u32 node_table_lookup_error_b;
	u32 node_table_full;
	u32 lre_multicast_dropped;
	u32 lre_vlan_dropped;
	u32 lre_intr_tmr_exp;

	/* additional debug counters */
	u32 lre_total_rx_a; /* count of all frames received at port-A */
	u32 lre_total_rx_b; /* count of all frames received at port-B */
	u32 lre_overflow_pru0; /* count of overflow frames to host on PRU 0 */
	u32 lre_overflow_pru1; /* count of overflow frames to host on PRU 1 */
	u32 lre_cnt_dd_pru0; /* count of DD frames to host on PRU 0 */
	u32 lre_cnt_dd_pru1; /* count of DD frames to host on PRU 1 */
	u32 lre_cnt_sup_pru0; /* count of supervisor frames to host on PRU 0 */
	u32 lre_cnt_sup_pru1; /* count of supervisor frames to host on PRU 1 */
} __packed;

/* node table info */
struct prueth_lre_node {
	u8 mac[6];
	u8 state;
	u8 status;

	u32 cnt_rx_a;
	u32 cnt_rx_b;

	u32 prp_lid_err_a;
	u32 prp_lid_err_b;

	u8 cnt_rx_sup_a;
	u8 cnt_rx_sup_b;
	u16 time_last_seen_sup;

	u16 time_last_seen_a;
	u16 time_last_seen_b;
} __packed;

/* NT queue definitions */
struct nt_queue_entry {
	u8 mac[ETH_ALEN];
	unsigned int sv_frame:1;
	unsigned int proto:1;
	int port_id:6;
};

struct nt_queue_t {
	struct nt_queue_entry nt_queue[PRUETH_MAC_QUEUE_MAX];
	int rd_ind;
	int wr_ind;
	bool full;
};

struct node_index_tbl_t {
	u16 bin_offset;
	u16 bin_no_entries;
	u8  lin_bin;	/* 0 - linear; 1 - binary; */
	u8  res1;
} __packed;

struct bin_tbl_t {
	u8 src_mac_id[ETH_ALEN];
	u16 node_tbl_offset;
} __packed;

struct node_tbl_t {
	u8 mac[ETH_ALEN];
	u8  entry_state;
	u8  status;
	u32 cnt_ra;
	u32 cnt_rb;
	u32 err_wla;
	u32 err_wlb;
	u8  cnt_rx_sup_a;
	u8  cnt_rx_sup_b;
	u16 time_last_seen_s;
	u16 time_last_seen_a;
	u16 time_last_seen_b;
} __packed;

struct node_tbl_lre_cnt_t {
	u16 lre_cnt;
} __packed;

struct node_tbl_info_t {
	u32 next_free_slot;
	u8  arm_lock;
	u8  res;
	u16 fw_lock; /* firmware use this field as 2 independent bytes
		      * first byte for PRU0, second for PRU1
		      */
} __packed;

struct nt_array_t {
	struct node_tbl_t	node_tbl[PRUETH_LRE_NODE_TBL_MAX_ENTRIES];
} __packed;
struct index_array_t {
	struct node_index_tbl_t index_tbl[PRUETH_LRE_INDEX_TBL_MAX_ENTRIES];
} __packed;
struct bin_array_t {
	struct bin_tbl_t	bin_tbl[PRUETH_LRE_BIN_TBL_MAX_ENTRIES];
} __packed;

struct node_tbl {
	struct bin_array_t *bin_array;
	struct index_array_t *index_array;
	struct nt_array_t *nt_array;
	struct node_tbl_info_t *nt_info;
	struct node_tbl_lre_cnt_t *nt_lre_cnt;
	u32 index_array_max_entries;
	u32 bin_array_max_entries;
	u32 nt_array_max_entries;
	u16 hash_mask;
};

void prueth_lre_config(struct prueth *prueth);
void prueth_lre_cleanup(struct prueth *prueth);
int prueth_lre_init_node_table(struct prueth *prueth);
int prueth_lre_request_irqs(struct prueth_emac *emac);
void prueth_lre_free_irqs(struct prueth_emac *emac);
int prueth_lre_get_sset_count(struct prueth *prueth);
void prueth_lre_get_strings(struct prueth *prueth, u8 *data);
void prueth_lre_update_stats(struct prueth *prueth, u64 *data);
void prueth_lre_set_stats(struct prueth *prueth,
			  struct lre_statistics *pstats);
void prueth_lre_get_stats(struct prueth *prueth,
			  struct lre_statistics *pstats);
void prueth_lre_config_check_flags(struct prueth *prueth);
void prueth_lre_free_memory(struct prueth *prueth);
int prueth_lre_nt_insert(struct prueth *prueth,
			 u8 *mac, int port, int sv_frame, int proto);

extern const struct lredev_ops prueth_lredev_ops;

#endif /* __NET_TI_PRUETH_LRE_H */
