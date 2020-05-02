/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2019-2020 Texas Instruments Incorporated - http://www.ti.com */
#ifndef __NET_TI_PRUSS_FDB_TBL_H
#define __NET_TI_PRUSS_FDB_TBL_H

#include <linux/kernel.h>
#include <linux/debugfs.h>
#include "prueth.h"

#define ETHER_ADDR_LEN 6

/* 4 bytes */
struct fdb_index_tbl_entry_t {
	u16 bucket_idx;     /* Bucket Table index of first Bucket
			     * with this MAC address
			     */
	u16 bucket_entries; /* Number of entries in this bucket */
} __packed;

/* 4 * 256 = 1024 = 0x200 bytes */
struct fdb_index_array_t {
	struct fdb_index_tbl_entry_t index_tbl_entry[FDB_INDEX_TBL_MAX_ENTRIES];
} __packed;

/* 10 bytes */
struct fdb_mac_tbl_entry_t {
	u8  mac[ETHER_ADDR_LEN];
	u16 age;
	u8  port; /* 0 based: 0=port1, 1=port2 */
	u8  is_static:1;
	u8  active:1;
} __packed;

/* 10 * 256 = 2560 = 0xa00 bytes */
struct fdb_mac_tbl_array_t {
	struct fdb_mac_tbl_entry_t mac_tbl_entry[FDB_MAC_TBL_MAX_ENTRIES];
} __packed;

/* 1 byte */
struct fdb_stp_config {
	u8  state; /* per-port STP state (defined in FW header) */
} __packed;

/* 1 byte */
struct fdb_flood_config {
	u8 host_flood_enable:1;
	u8 port1_flood_enable:1;
	u8 port2_flood_enable:1;
} __packed;

/* 2 byte */
struct fdb_arbitration {
	u8  host_lock;
	u8  pru_locks;
} __packed;

struct fdb_tbl {
	struct fdb_index_array_t *index_a; /* fdb index table */
	struct fdb_mac_tbl_array_t *mac_tbl_a; /* fdb mac table */
	struct fdb_stp_config *port1_stp_cfg; /* port 1 strp config */
	struct fdb_stp_config *port2_stp_cfg; /* port 2 strp config */
	struct fdb_flood_config *flood_enable_flags; /* per-port flood enable */
	struct fdb_arbitration *locks; /* fdb locking mechanism */
	u16 total_entries; /* total num entries in hash table */
};

#endif
