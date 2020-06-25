// SPDX-License-Identifier: GPL-2.0
/* Texas Instruments PRUETH Switch Driver
 *
 * Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com
 */

#include <linux/kernel.h>
#include <linux/remoteproc.h>
#include <net/switchdev.h>
#include "prueth.h"
#include "prueth_switch.h"
#include "prueth_fdb_tbl.h"

#define FDB_IDX_TBL() \
	(&prueth->fdb_tbl->index_a->index_tbl_entry[0])

#define FDB_IDX_TBL_ENTRY(n) \
	(&prueth->fdb_tbl->index_a->index_tbl_entry[n])

#define FDB_MAC_TBL() \
	(&prueth->fdb_tbl->mac_tbl_a->mac_tbl_entry[0])

#define FDB_MAC_TBL_ENTRY(n) \
	(&prueth->fdb_tbl->mac_tbl_a->mac_tbl_entry[n])

#define FDB_LEARN  1
#define FDB_DELETE 2
#define FDB_PURGE  3

struct prueth_sw_fdb_work {
	struct work_struct work;
	struct prueth_emac *emac;
	u8 addr[ETH_ALEN];
	int event;
};

static inline
u8 prueth_sw_port_get_stp_state(struct prueth *prueth, enum prueth_port port)
{
	struct fdb_tbl *t = prueth->fdb_tbl;
	u8 state;

	state = readb(port - 1 ?
		      &t->port2_stp_cfg->state : &t->port1_stp_cfg->state);
	return state;
}

const struct prueth_queue_info sw_queue_infos[][NUM_QUEUES] = {
	[PRUETH_PORT_QUEUE_HOST] = {
		[PRUETH_QUEUE1] = {
			P0_Q1_BUFFER_OFFSET,
			P0_QUEUE_DESC_OFFSET,
			P0_Q1_BD_OFFSET,
			P0_Q1_BD_OFFSET + ((HOST_QUEUE_1_SIZE - 1) * BD_SIZE),
		},
		[PRUETH_QUEUE2] = {
			P0_Q2_BUFFER_OFFSET,
			P0_QUEUE_DESC_OFFSET + 8,
			P0_Q2_BD_OFFSET,
			P0_Q2_BD_OFFSET + ((HOST_QUEUE_2_SIZE - 1) * BD_SIZE),
		},
		[PRUETH_QUEUE3] = {
			P0_Q3_BUFFER_OFFSET,
			P0_QUEUE_DESC_OFFSET + 16,
			P0_Q3_BD_OFFSET,
			P0_Q3_BD_OFFSET + ((HOST_QUEUE_3_SIZE - 1) * BD_SIZE),
		},
		[PRUETH_QUEUE4] = {
			P0_Q4_BUFFER_OFFSET,
			P0_QUEUE_DESC_OFFSET + 24,
			P0_Q4_BD_OFFSET,
			P0_Q4_BD_OFFSET + ((HOST_QUEUE_4_SIZE - 1) * BD_SIZE),
		},
	},
	[PRUETH_PORT_QUEUE_MII0] = {
		[PRUETH_QUEUE1] = {
			P1_Q1_BUFFER_OFFSET,
			P1_Q1_BUFFER_OFFSET +
				((QUEUE_1_SIZE - 1) * ICSS_BLOCK_SIZE),
			P1_Q1_BD_OFFSET,
			P1_Q1_BD_OFFSET + ((QUEUE_1_SIZE - 1) * BD_SIZE),
		},
		[PRUETH_QUEUE2] = {
			P1_Q2_BUFFER_OFFSET,
			P1_Q2_BUFFER_OFFSET +
				((QUEUE_2_SIZE - 1) * ICSS_BLOCK_SIZE),
			P1_Q2_BD_OFFSET,
			P1_Q2_BD_OFFSET + ((QUEUE_2_SIZE - 1) * BD_SIZE),
		},
		[PRUETH_QUEUE3] = {
			P1_Q3_BUFFER_OFFSET,
			P1_Q3_BUFFER_OFFSET +
				((QUEUE_3_SIZE - 1) * ICSS_BLOCK_SIZE),
			P1_Q3_BD_OFFSET,
			P1_Q3_BD_OFFSET + ((QUEUE_3_SIZE - 1) * BD_SIZE),
		},
		[PRUETH_QUEUE4] = {
			P1_Q4_BUFFER_OFFSET,
			P1_Q4_BUFFER_OFFSET +
				((QUEUE_4_SIZE - 1) * ICSS_BLOCK_SIZE),
			P1_Q4_BD_OFFSET,
			P1_Q4_BD_OFFSET + ((QUEUE_4_SIZE - 1) * BD_SIZE),
		},
	},
	[PRUETH_PORT_QUEUE_MII1] = {
		[PRUETH_QUEUE1] = {
			P2_Q1_BUFFER_OFFSET,
			P2_Q1_BUFFER_OFFSET +
				((QUEUE_1_SIZE - 1) * ICSS_BLOCK_SIZE),
			P2_Q1_BD_OFFSET,
			P2_Q1_BD_OFFSET + ((QUEUE_1_SIZE - 1) * BD_SIZE),
		},
		[PRUETH_QUEUE2] = {
			P2_Q2_BUFFER_OFFSET,
			P2_Q2_BUFFER_OFFSET +
				((QUEUE_2_SIZE - 1) * ICSS_BLOCK_SIZE),
			P2_Q2_BD_OFFSET,
			P2_Q2_BD_OFFSET + ((QUEUE_2_SIZE - 1) * BD_SIZE),
		},
		[PRUETH_QUEUE3] = {
			P2_Q3_BUFFER_OFFSET,
			P2_Q3_BUFFER_OFFSET +
				((QUEUE_3_SIZE - 1) * ICSS_BLOCK_SIZE),
			P2_Q3_BD_OFFSET,
			P2_Q3_BD_OFFSET + ((QUEUE_3_SIZE - 1) * BD_SIZE),
		},
		[PRUETH_QUEUE4] = {
			P2_Q4_BUFFER_OFFSET,
			P2_Q4_BUFFER_OFFSET +
				((QUEUE_4_SIZE - 1) * ICSS_BLOCK_SIZE),
			P2_Q4_BD_OFFSET,
			P2_Q4_BD_OFFSET + ((QUEUE_4_SIZE - 1) * BD_SIZE),
		},
	},
};

static const struct prueth_queue_info rx_queue_infos[][NUM_QUEUES] = {
	[PRUETH_PORT_QUEUE_HOST] = {
		[PRUETH_QUEUE1] = {
			P0_Q1_BUFFER_OFFSET,
			HOST_QUEUE_DESC_OFFSET,
			P0_Q1_BD_OFFSET,
			P0_Q1_BD_OFFSET + ((HOST_QUEUE_1_SIZE - 1) * BD_SIZE),
		},
		[PRUETH_QUEUE2] = {
			P0_Q2_BUFFER_OFFSET,
			HOST_QUEUE_DESC_OFFSET + 8,
			P0_Q2_BD_OFFSET,
			P0_Q2_BD_OFFSET + ((HOST_QUEUE_2_SIZE - 1) * BD_SIZE),
		},
		[PRUETH_QUEUE3] = {
			P0_Q3_BUFFER_OFFSET,
			HOST_QUEUE_DESC_OFFSET + 16,
			P0_Q3_BD_OFFSET,
			P0_Q3_BD_OFFSET + ((HOST_QUEUE_3_SIZE - 1) * BD_SIZE),
		},
		[PRUETH_QUEUE4] = {
			P0_Q4_BUFFER_OFFSET,
			HOST_QUEUE_DESC_OFFSET + 24,
			P0_Q4_BD_OFFSET,
			P0_Q4_BD_OFFSET + ((HOST_QUEUE_4_SIZE - 1) * BD_SIZE),
		},
	},
	[PRUETH_PORT_QUEUE_MII0] = {
		[PRUETH_QUEUE1] = {
			P1_Q1_BUFFER_OFFSET,
			P1_QUEUE_DESC_OFFSET,
			P1_Q1_BD_OFFSET,
			P1_Q1_BD_OFFSET + ((QUEUE_1_SIZE - 1) * BD_SIZE),
		},
		[PRUETH_QUEUE2] = {
			P1_Q2_BUFFER_OFFSET,
			P1_QUEUE_DESC_OFFSET + 8,
			P1_Q2_BD_OFFSET,
			P1_Q2_BD_OFFSET + ((QUEUE_2_SIZE - 1) * BD_SIZE),
		},
		[PRUETH_QUEUE3] = {
			P1_Q3_BUFFER_OFFSET,
			P1_QUEUE_DESC_OFFSET + 16,
			P1_Q3_BD_OFFSET,
			P1_Q3_BD_OFFSET + ((QUEUE_3_SIZE - 1) * BD_SIZE),
		},
		[PRUETH_QUEUE4] = {
			P1_Q4_BUFFER_OFFSET,
			P1_QUEUE_DESC_OFFSET + 24,
			P1_Q4_BD_OFFSET,
			P1_Q4_BD_OFFSET + ((QUEUE_4_SIZE - 1) * BD_SIZE),
		},
	},
	[PRUETH_PORT_QUEUE_MII1] = {
		[PRUETH_QUEUE1] = {
			P2_Q1_BUFFER_OFFSET,
			P2_QUEUE_DESC_OFFSET,
			P2_Q1_BD_OFFSET,
			P2_Q1_BD_OFFSET + ((QUEUE_1_SIZE - 1) * BD_SIZE),
		},
		[PRUETH_QUEUE2] = {
			P2_Q2_BUFFER_OFFSET,
			P2_QUEUE_DESC_OFFSET + 8,
			P2_Q2_BD_OFFSET,
			P2_Q2_BD_OFFSET + ((QUEUE_2_SIZE - 1) * BD_SIZE),
		},
		[PRUETH_QUEUE3] = {
			P2_Q3_BUFFER_OFFSET,
			P2_QUEUE_DESC_OFFSET + 16,
			P2_Q3_BD_OFFSET,
			P2_Q3_BD_OFFSET + ((QUEUE_3_SIZE - 1) * BD_SIZE),
		},
		[PRUETH_QUEUE4] = {
			P2_Q4_BUFFER_OFFSET,
			P2_QUEUE_DESC_OFFSET + 24,
			P2_Q4_BD_OFFSET,
			P2_Q4_BD_OFFSET + ((QUEUE_4_SIZE - 1) * BD_SIZE),
		},
	},
};

static const struct prueth_col_rx_context_info col_rx_context_infos[] = {
	[PRUETH_PORT_QUEUE_HOST] = {
		P0_COL_BUFFER_OFFSET,
		P0_COL_BUFFER_OFFSET,
		P0_COL_QUEUE_DESC_OFFSET,
		END_OF_BD_POOL,
		END_OF_BD_POOL + ((COL_QUEUE_SIZE - 1) * BD_SIZE)
	},
	[PRUETH_PORT_QUEUE_MII0] = {
		P0_COL_BUFFER_OFFSET + (COL_QUEUE_SIZE * ICSS_BLOCK_SIZE),
		P0_COL_BUFFER_OFFSET + (COL_QUEUE_SIZE * ICSS_BLOCK_SIZE),
		P0_COL_QUEUE_DESC_OFFSET + 8,
		END_OF_BD_POOL,
		END_OF_BD_POOL + ((COL_QUEUE_SIZE - 1) * BD_SIZE)
	},

	[PRUETH_PORT_QUEUE_MII1] = {
		P0_COL_BUFFER_OFFSET + (COL_QUEUE_SIZE * ICSS_BLOCK_SIZE),
		P0_COL_BUFFER_OFFSET + (COL_QUEUE_SIZE * ICSS_BLOCK_SIZE),
		P0_COL_QUEUE_DESC_OFFSET + 16,
		END_OF_BD_POOL,
		END_OF_BD_POOL + ((COL_QUEUE_SIZE - 1) * BD_SIZE)
	},
};

static const struct prueth_col_tx_context_info col_tx_context_infos[] = {
	[PRUETH_PORT_QUEUE_HOST] = {
		P0_COL_BUFFER_OFFSET,
		P0_COL_BUFFER_OFFSET,
		P0_COL_BUFFER_OFFSET + ((COL_QUEUE_SIZE - 1) * ICSS_BLOCK_SIZE),
	},
	[PRUETH_PORT_QUEUE_MII0] = {
		P0_COL_BUFFER_OFFSET + (COL_QUEUE_SIZE * ICSS_BLOCK_SIZE),
		P0_COL_BUFFER_OFFSET + (COL_QUEUE_SIZE * ICSS_BLOCK_SIZE),
		P0_COL_BUFFER_OFFSET + ((COL_QUEUE_SIZE - 1) * ICSS_BLOCK_SIZE),
	},
	[PRUETH_PORT_QUEUE_MII1] = {
		P0_COL_BUFFER_OFFSET + (COL_QUEUE_SIZE * ICSS_BLOCK_SIZE),
		P0_COL_BUFFER_OFFSET + (COL_QUEUE_SIZE * ICSS_BLOCK_SIZE),
		P0_COL_BUFFER_OFFSET + ((COL_QUEUE_SIZE - 1) * ICSS_BLOCK_SIZE),
	}
};

static const struct prueth_queue_desc col_queue_descs[3] = {
	[PRUETH_PORT_QUEUE_MII0] = {
		.rd_ptr = END_OF_BD_POOL, .wr_ptr = END_OF_BD_POOL, },
	[PRUETH_PORT_QUEUE_MII1] = {
		.rd_ptr = END_OF_BD_POOL, .wr_ptr = END_OF_BD_POOL, }
};

void prueth_sw_free_fdb_table(struct prueth *prueth)
{
	if (prueth->emac_configured)
		return;

	kfree(prueth->fdb_tbl);
	prueth->fdb_tbl = NULL;
}

void prueth_sw_hostconfig(struct prueth *prueth)
{
	void __iomem *dram1_base = prueth->mem[PRUETH_MEM_DRAM1].va;
	void __iomem *dram;

	/* queue information table */
	dram = dram1_base + P0_Q1_RX_CONTEXT_OFFSET;
	memcpy_toio(dram, sw_queue_infos[PRUETH_PORT_QUEUE_HOST],
		    sizeof(sw_queue_infos[PRUETH_PORT_QUEUE_HOST]));

	dram = dram1_base + COL_RX_CONTEXT_P0_OFFSET_ADDR;
	memcpy_toio(dram, &col_rx_context_infos[PRUETH_PORT_QUEUE_HOST],
		    sizeof(col_rx_context_infos[PRUETH_PORT_QUEUE_HOST]));

	/* buffer descriptor offset table*/
	dram = dram1_base + QUEUE_DESCRIPTOR_OFFSET_ADDR;
	writew(P0_Q1_BD_OFFSET, dram);
	writew(P0_Q2_BD_OFFSET, dram + 2);
	writew(P0_Q3_BD_OFFSET, dram + 4);
	writew(P0_Q4_BD_OFFSET, dram + 6);

	/* buffer offset table */
	dram = dram1_base + QUEUE_OFFSET_ADDR;
	writew(P0_Q1_BUFFER_OFFSET, dram);
	writew(P0_Q2_BUFFER_OFFSET, dram + 2);
	writew(P0_Q3_BUFFER_OFFSET, dram + 4);
	writew(P0_Q4_BUFFER_OFFSET, dram + 6);

	/* queue size lookup table */
	dram = dram1_base + QUEUE_SIZE_ADDR;
	writew(HOST_QUEUE_1_SIZE, dram);
	writew(HOST_QUEUE_1_SIZE, dram + 2);
	writew(HOST_QUEUE_1_SIZE, dram + 4);
	writew(HOST_QUEUE_1_SIZE, dram + 6);

	/* queue table */
	dram = dram1_base + P0_QUEUE_DESC_OFFSET;
	memcpy_toio(dram, queue_descs[PRUETH_PORT_QUEUE_HOST],
		    sizeof(queue_descs[PRUETH_PORT_QUEUE_HOST]));
}

static int prueth_sw_port_config(struct prueth *prueth,
				 enum prueth_port port_id)
{
	unsigned int tx_context_ofs_addr, col_tx_context_ofs_addr,
		     rx_context_ofs, col_rx_context_ofs_addr,
		     queue_desc_ofs, col_queue_desc_ofs;
	void __iomem *dram, *dram_base, *dram_mac;
	struct prueth_emac *emac;
	void __iomem *dram1_base = prueth->mem[PRUETH_MEM_DRAM1].va;

	emac = prueth->emac[port_id - 1];
	switch (port_id) {
	case PRUETH_PORT_MII0:
		tx_context_ofs_addr     = TX_CONTEXT_P1_Q1_OFFSET_ADDR;
		col_tx_context_ofs_addr = COL_TX_CONTEXT_P1_Q1_OFFSET_ADDR;
		rx_context_ofs          = P1_Q1_RX_CONTEXT_OFFSET;
		col_rx_context_ofs_addr = COL_RX_CONTEXT_P1_OFFSET_ADDR;
		queue_desc_ofs          = P1_QUEUE_DESC_OFFSET;
		col_queue_desc_ofs      = P1_COL_QUEUE_DESC_OFFSET;

		/* for switch PORT MII0 mac addr is in DRAM0. */
		dram_mac = prueth->mem[PRUETH_MEM_DRAM0].va;
		break;
	case PRUETH_PORT_MII1:
		tx_context_ofs_addr     = TX_CONTEXT_P2_Q1_OFFSET_ADDR;
		col_tx_context_ofs_addr = COL_TX_CONTEXT_P2_Q1_OFFSET_ADDR;
		rx_context_ofs          = P2_Q1_RX_CONTEXT_OFFSET;
		col_rx_context_ofs_addr = COL_RX_CONTEXT_P2_OFFSET_ADDR;
		queue_desc_ofs          = P2_QUEUE_DESC_OFFSET;
		col_queue_desc_ofs      = P2_COL_QUEUE_DESC_OFFSET;

		/* for switch PORT MII1 mac addr is in DRAM1. */
		dram_mac = prueth->mem[PRUETH_MEM_DRAM1].va;
		break;
	default:
		netdev_err(emac->ndev, "invalid port\n");
		return -EINVAL;
	}

	/* setup mac address */
	memcpy_toio(dram_mac + PORT_MAC_ADDR, emac->mac_addr, 6);

	/* Remaining switch port configs are in DRAM1 */
	dram_base = prueth->mem[PRUETH_MEM_DRAM1].va;

	/* queue information table */
	memcpy_toio(dram_base + tx_context_ofs_addr,
		    sw_queue_infos[port_id],
		    sizeof(sw_queue_infos[port_id]));

	memcpy_toio(dram_base + col_tx_context_ofs_addr,
		    &col_tx_context_infos[port_id],
		    sizeof(col_tx_context_infos[port_id]));

	memcpy_toio(dram_base + rx_context_ofs,
		    rx_queue_infos[port_id],
		    sizeof(rx_queue_infos[port_id]));

	memcpy_toio(dram_base + col_rx_context_ofs_addr,
		    &col_rx_context_infos[port_id],
		    sizeof(col_rx_context_infos[port_id]));

	/* buffer descriptor offset table*/
	dram = dram_base + QUEUE_DESCRIPTOR_OFFSET_ADDR +
	       (port_id * NUM_QUEUES * sizeof(u16));
	writew(sw_queue_infos[port_id][PRUETH_QUEUE1].buffer_desc_offset, dram);
	writew(sw_queue_infos[port_id][PRUETH_QUEUE2].buffer_desc_offset,
	       dram + 2);
	writew(sw_queue_infos[port_id][PRUETH_QUEUE3].buffer_desc_offset,
	       dram + 4);
	writew(sw_queue_infos[port_id][PRUETH_QUEUE4].buffer_desc_offset,
	       dram + 6);

	/* buffer offset table */
	dram = dram_base + QUEUE_OFFSET_ADDR +
	       port_id * NUM_QUEUES * sizeof(u16);
	writew(sw_queue_infos[port_id][PRUETH_QUEUE1].buffer_offset, dram);
	writew(sw_queue_infos[port_id][PRUETH_QUEUE2].buffer_offset,
	       dram + 2);
	writew(sw_queue_infos[port_id][PRUETH_QUEUE3].buffer_offset,
	       dram + 4);
	writew(sw_queue_infos[port_id][PRUETH_QUEUE4].buffer_offset,
	       dram + 6);

	/* queue size lookup table */
	dram = dram_base + QUEUE_SIZE_ADDR +
	       port_id * NUM_QUEUES * sizeof(u16);
	writew(QUEUE_1_SIZE, dram);
	writew(QUEUE_2_SIZE, dram + 2);
	writew(QUEUE_3_SIZE, dram + 4);
	writew(QUEUE_4_SIZE, dram + 6);

	/* collision queue table */
	memcpy_toio(dram_base + col_queue_desc_ofs,
		    &col_queue_descs[port_id],
		    sizeof(col_queue_descs[port_id]));

	/* queue table */
	memcpy_toio(dram_base + queue_desc_ofs,
		    &queue_descs[port_id][0],
		    4 * sizeof(queue_descs[port_id][0]));

	emac->rx_queue_descs = dram1_base + P0_QUEUE_DESC_OFFSET;
	emac->tx_queue_descs = dram1_base +
		rx_queue_infos[port_id][PRUETH_QUEUE1].queue_desc_offset;

	return 0;
}

int prueth_sw_emac_config(struct prueth_emac *emac)
{
	struct prueth *prueth = emac->prueth;

	/* PRU needs local shared RAM address for C28 */
	u32 sharedramaddr = ICSS_LOCAL_SHARED_RAM;
	/* PRU needs real global OCMC address for C30*/
	u32 ocmcaddr = (u32)prueth->mem[PRUETH_MEM_OCMC].pa;
	int ret;

	if (prueth->emac_configured & BIT(emac->port_id))
		return 0;

	ret = prueth_sw_port_config(prueth, emac->port_id);
	if (ret)
		return ret;

	if (!prueth->emac_configured) {
		/* Set in constant table C28 of PRUn to ICSS Shared memory */
		pru_rproc_set_ctable(prueth->pru0, PRU_C28, sharedramaddr);
		pru_rproc_set_ctable(prueth->pru1, PRU_C28, sharedramaddr);

		/* Set in constant table C30 of PRUn to OCMC memory */
		pru_rproc_set_ctable(prueth->pru0, PRU_C30, ocmcaddr);
		pru_rproc_set_ctable(prueth->pru1, PRU_C30, ocmcaddr);
	}
	return 0;
}

void prueth_sw_fdb_tbl_init(struct prueth *prueth)
{
	struct fdb_tbl *t = prueth->fdb_tbl;

	t->index_a = prueth->mem[V2_1_FDB_TBL_LOC].va +
			V2_1_FDB_TBL_OFFSET;

	t->mac_tbl_a          = (void __iomem *)t->index_a +
				FDB_INDEX_TBL_MAX_ENTRIES *
				sizeof(struct fdb_index_tbl_entry_t);

	t->port1_stp_cfg      = (void __iomem *)t->mac_tbl_a +
				FDB_MAC_TBL_MAX_ENTRIES *
				sizeof(struct fdb_mac_tbl_entry_t);

	t->port2_stp_cfg      = (void __iomem *)t->port1_stp_cfg +
				sizeof(struct fdb_stp_config);

	t->flood_enable_flags = (void __iomem *)t->port2_stp_cfg +
				sizeof(struct fdb_stp_config);

	t->locks              = (void __iomem *)t->flood_enable_flags +
				sizeof(struct fdb_flood_config);

	t->flood_enable_flags->host_flood_enable  = 1;
	t->flood_enable_flags->port1_flood_enable = 1;
	t->flood_enable_flags->port2_flood_enable = 1;
	t->locks->host_lock                       = 0;
	t->total_entries                          = 0;
}

static void prueth_sw_fdb_spin_lock(struct fdb_tbl *fdb_tbl)
{
	/* Take the host lock */
	writeb(1, &fdb_tbl->locks->host_lock);

	/* Wait for the PRUs to release their locks */
	while (readb(&fdb_tbl->locks->pru_locks))
		;
}

static inline void prueth_sw_fdb_spin_unlock(struct fdb_tbl *fdb_tbl)
{
	writeb(0, &fdb_tbl->locks->host_lock);
}

static void mac_copy(u8 *dst, const u8 *src)
{
	u8 i;

	for (i = 0; i < 6; i++) {
		*(dst) = *(src);
		dst++;
		src++;
	}
}

/* -1  mac_a <  mac_b
 *  0  mac_a == mac_b
 *  1  mac_a >  mac_b
 */
static s8 mac_cmp(const u8 *mac_a, const u8 *mac_b)
{
	s8  ret = 0, i;

	for (i = 0; i < 6; i++) {
		if (mac_a[i] == mac_b[i])
			continue;

		ret = mac_a[i] < mac_b[i] ? -1 : 1;
		break;
	}

	return ret;
}

static inline u8 prueth_sw_fdb_hash(const u8 *mac)
{
	return mac[0] ^ mac[1] ^ mac[2] ^ mac[3] ^ mac[4] ^ mac[5];
}

static s16
prueth_sw_fdb_search(struct fdb_mac_tbl_array_t *mac_tbl,
		     struct fdb_index_tbl_entry_t *bucket_info,
		     const u8 *mac)
{
	int i;
	u8 mac_tbl_idx = bucket_info->bucket_idx;

	for (i = 0; i < bucket_info->bucket_entries; i++, mac_tbl_idx++) {
		if (!mac_cmp(mac, mac_tbl->mac_tbl_entry[mac_tbl_idx].mac))
			return mac_tbl_idx;
	}

	return -ENODATA;
}

static u16 prueth_sw_fdb_find_open_slot(struct fdb_tbl *fdb_tbl)
{
	u16 i;

	for (i = 0; i < FDB_MAC_TBL_MAX_ENTRIES; i++) {
		if (!fdb_tbl->mac_tbl_a->mac_tbl_entry[i].active)
			break;
	}

	return i;
}

/* port: 0 based: 0=port1, 1=port2 */
static s16
prueth_sw_fdb_find_bucket_insert_point(struct fdb_tbl *fdb,
				       struct fdb_index_tbl_entry_t *bkt_info,
				       const u8 *mac, const u8 port)
{
	struct fdb_mac_tbl_array_t *mac_tbl = fdb->mac_tbl_a;
	struct fdb_mac_tbl_entry_t *e;
	int i;
	u8 mac_tbl_idx;
	s8 cmp;

	mac_tbl_idx = bkt_info->bucket_idx;

	for (i = 0; i < bkt_info->bucket_entries; i++, mac_tbl_idx++) {
		e = &mac_tbl->mac_tbl_entry[mac_tbl_idx];
		cmp = mac_cmp(mac, e->mac);
		if (cmp < 0) {
			return mac_tbl_idx;
		} else if (cmp == 0) {
			if (e->port != port) {
				/* mac is already in FDB, only port is
				 * different. So just update the port.
				 * Note: total_entries and bucket_entries
				 * remain the same.
				 */
				prueth_sw_fdb_spin_lock(fdb);
				e->port = port;
				prueth_sw_fdb_spin_unlock(fdb);
			}

			/* mac and port are the same, touch the fdb */
			e->age = 0;
			return -1;
		}
	}

	return mac_tbl_idx;
}

static s16
prueth_sw_fdb_check_empty_slot_left(struct fdb_mac_tbl_array_t *mac_tbl,
				    u8 mac_tbl_idx)
{
	s16 i;

	for (i = mac_tbl_idx - 1; i > -1; i--) {
		if (!mac_tbl->mac_tbl_entry[i].active)
			break;
	}

	return i;
}

static s16
prueth_sw_fdb_check_empty_slot_right(struct fdb_mac_tbl_array_t *mac_tbl,
				     u8 mac_tbl_idx)
{
	s16 i;

	for (i = mac_tbl_idx; i < FDB_MAC_TBL_MAX_ENTRIES; i++) {
		if (!mac_tbl->mac_tbl_entry[i].active)
			return i;
	}

	return -1;
}

static void prueth_sw_fdb_move_range_left(struct prueth *prueth,
					  u16 left, u16 right)
{
	u16 i;
	u8 *src, *dst;
	u32 sz = 0;

	for (i = left; i < right; i++) {
		dst = (u8 *)FDB_MAC_TBL_ENTRY(i);
		src = (u8 *)FDB_MAC_TBL_ENTRY(i + 1);
		sz = sizeof(struct fdb_mac_tbl_entry_t);
		memcpy_toio(dst, src, sz);
	}
}

static void prueth_sw_fdb_move_range_right(struct prueth *prueth,
					   u16 left, u16 right)
{
	u16 i;
	u8 *src, *dst;
	u32 sz = 0;

	for (i = right; i > left; i--) {
		dst = (u8 *)FDB_MAC_TBL_ENTRY(i);
		src = (u8 *)FDB_MAC_TBL_ENTRY(i - 1);
		sz = sizeof(struct fdb_mac_tbl_entry_t);
		memcpy_toio(dst, src, sz);
	}
}

static void prueth_sw_fdb_update_index_tbl(struct prueth *prueth,
					   u16 left, u16 right)
{
	u16 i;
	u8 hash, hash_prev;

	/* To ensure we don't improperly update the
	 * bucket index, initialize with an invalid
	 * hash in case we are in leftmost slot
	 */
	hash_prev = 0xff;

	if (left > 0) {
		hash_prev =
			prueth_sw_fdb_hash(FDB_MAC_TBL_ENTRY(left - 1)->mac);
	}

	/* For each moved element, update the bucket index */
	for (i = left; i <= right; i++) {
		hash = prueth_sw_fdb_hash(FDB_MAC_TBL_ENTRY(i)->mac);

		/* Only need to update buckets once */
		if (hash != hash_prev)
			FDB_IDX_TBL_ENTRY(hash)->bucket_idx = i;

		hash_prev = hash;
	}
}

static struct fdb_mac_tbl_entry_t *
prueth_sw_get_empty_mac_tbl_entry(struct prueth *prueth,
				  struct fdb_index_tbl_entry_t *bucket_info,
				  u8 suggested_mac_tbl_idx,
				  bool *update_indexes)
{
	struct fdb_tbl *fdb = prueth->fdb_tbl;
	struct fdb_mac_tbl_array_t *mt = fdb->mac_tbl_a;
	s16 empty_slot_idx = 0, left = 0, right = 0;
	u8 mti = suggested_mac_tbl_idx;

	if (!FDB_MAC_TBL_ENTRY(mti)->active) {
		/* Claim the entry */
		FDB_MAC_TBL_ENTRY(mti)->active = 1;

		return FDB_MAC_TBL_ENTRY(mti);
	}

	if (fdb->total_entries == FDB_MAC_TBL_MAX_ENTRIES)
		return NULL;

	empty_slot_idx = prueth_sw_fdb_check_empty_slot_left(mt, mti);
	if (empty_slot_idx == -1) {
		/* Nothing available on the left. But table isn't full
		 * so there must be space to the right,
		 */
		empty_slot_idx = prueth_sw_fdb_check_empty_slot_right(mt, mti);

		/* Shift right */
		left = mti;
		right = empty_slot_idx;
		prueth_sw_fdb_move_range_right(prueth, left, right);

		/* Claim the entry */
		FDB_MAC_TBL_ENTRY(mti)->active = 1;

		/* There is a chance we moved something in a
		 * different bucket, update index table
		 */
		prueth_sw_fdb_update_index_tbl(prueth, left, right);

		return FDB_MAC_TBL_ENTRY(mti);
	}

	if (empty_slot_idx == mti - 1) {
		/* There is space immediately left of the open slot,
		 * which means the inserted MAC address.
		 * Must be the lowest-valued MAC address in bucket.
		 * Update bucket pointer accordingly.
		 */
		bucket_info->bucket_idx = empty_slot_idx;

		/* Claim the entry */
		FDB_MAC_TBL_ENTRY(empty_slot_idx)->active = 1;

		return FDB_MAC_TBL_ENTRY(empty_slot_idx);
	}

	/* There is empty space to the left, shift MAC table entries left */
	left = empty_slot_idx;
	right = mti - 1;
	prueth_sw_fdb_move_range_left(prueth, left, right);

	/* Claim the entry */
	FDB_MAC_TBL_ENTRY(mti - 1)->active = 1;

	/* There is a chance we moved something in a
	 * different bucket, update index table
	 */
	prueth_sw_fdb_update_index_tbl(prueth, left, right);

	return FDB_MAC_TBL_ENTRY(mti - 1);
}

static int prueth_sw_insert_fdb_entry(struct prueth_emac *emac,
				      const u8 *mac, u8 is_static)
{
	struct prueth *prueth = emac->prueth;
	struct prueth_emac *other_emac;
	struct fdb_tbl *fdb = prueth->fdb_tbl;
	struct fdb_index_tbl_entry_t *bucket_info;
	struct fdb_mac_tbl_entry_t *mac_info;
	u8 hash_val, mac_tbl_idx;
	s16 ret;

	other_emac = prueth->emac[other_port_id(emac->port_id) - 1];

	if (fdb->total_entries == FDB_MAC_TBL_MAX_ENTRIES)
		return -ENOMEM;

	if (mac_cmp(mac, emac->mac_addr) == 0 ||
	    mac_cmp(mac, other_emac->mac_addr) == 0) {
		/* Don't insert fdb of own mac addr */
		return -EINVAL;
	}

	/* Empty mac table entries are available */

	/* Get the bucket that the mac belongs to */
	hash_val = prueth_sw_fdb_hash(mac);
	bucket_info = FDB_IDX_TBL_ENTRY(hash_val);

	if (!bucket_info->bucket_entries) {
		mac_tbl_idx = prueth_sw_fdb_find_open_slot(fdb);
		bucket_info->bucket_idx = mac_tbl_idx;
	}

	ret = prueth_sw_fdb_find_bucket_insert_point(fdb, bucket_info, mac,
						     emac->port_id - 1);

	if (ret < 0)
		/* mac is already in fdb table */
		return 0;

	mac_tbl_idx = ret;

	prueth_sw_fdb_spin_lock(fdb);

	mac_info = prueth_sw_get_empty_mac_tbl_entry(prueth, bucket_info,
						     mac_tbl_idx, NULL);
	if (!mac_info) {
		/* Should not happen */
		dev_warn(prueth->dev, "OUT of MEM\n");
		return -ENOMEM;
	}

	mac_copy(mac_info->mac, mac);
	mac_info->active = 1;
	mac_info->age = 0;
	mac_info->port = emac->port_id - 1;
	mac_info->is_static = is_static;

	bucket_info->bucket_entries++;
	fdb->total_entries++;

	prueth_sw_fdb_spin_unlock(fdb);

	dev_dbg(prueth->dev, "added fdb: %pM port=%d total_entries=%u\n",
		mac, emac->port_id, fdb->total_entries);

	return 0;
}

static int prueth_sw_delete_fdb_entry(struct prueth_emac *emac,
				      const u8 *mac, u8 is_static)
{
	struct prueth *prueth = emac->prueth;
	struct fdb_tbl *fdb = prueth->fdb_tbl;
	struct fdb_mac_tbl_array_t *mt = fdb->mac_tbl_a;
	struct fdb_index_tbl_entry_t *bucket_info;
	struct fdb_mac_tbl_entry_t *mac_info;
	u8 hash_val, mac_tbl_idx;
	s16 ret, left, right;

	if (fdb->total_entries == 0)
		return 0;

	/* Get the bucket that the mac belongs to */
	hash_val = prueth_sw_fdb_hash(mac);
	bucket_info = FDB_IDX_TBL_ENTRY(hash_val);

	ret = prueth_sw_fdb_search(mt, bucket_info, mac);
	if (ret < 0)
		return ret;

	mac_tbl_idx = ret;
	mac_info = FDB_MAC_TBL_ENTRY(mac_tbl_idx);

	prueth_sw_fdb_spin_lock(fdb);

	/* Shift all elements in bucket to the left. No need to
	 * update index table since only shifting within bucket.
	 */
	left = mac_tbl_idx;
	right = bucket_info->bucket_idx + bucket_info->bucket_entries - 1;
	prueth_sw_fdb_move_range_left(prueth, left, right);

	/* Remove end of bucket from table */
	mac_info = FDB_MAC_TBL_ENTRY(right);
	mac_info->active = 0;
	bucket_info->bucket_entries--;
	fdb->total_entries--;

	prueth_sw_fdb_spin_unlock(fdb);

	dev_dbg(prueth->dev, "del fdb: %pM total_entries=%u\n",
		mac, fdb->total_entries);

	return 0;
}

static int prueth_sw_do_purge_fdb(struct prueth_emac *emac)
{
	struct prueth *prueth = emac->prueth;
	struct fdb_tbl *fdb = prueth->fdb_tbl;
	s16 i;

	if (fdb->total_entries == 0)
		return 0;

	prueth_sw_fdb_spin_lock(fdb);

	for (i = 0; i < FDB_INDEX_TBL_MAX_ENTRIES; i++)
		fdb->index_a->index_tbl_entry[i].bucket_entries = 0;

	for (i = 0; i < FDB_MAC_TBL_MAX_ENTRIES; i++)
		fdb->mac_tbl_a->mac_tbl_entry[i].active = 0;

	fdb->total_entries = 0;

	prueth_sw_fdb_spin_unlock(fdb);
	return 0;
}

static void prueth_sw_fdb_work(struct work_struct *work)
{
	struct prueth_sw_fdb_work *fdb_work =
		container_of(work, struct prueth_sw_fdb_work, work);
	struct prueth_emac *emac = fdb_work->emac;

	rtnl_lock();

	/* Interface is not up */
	if (!emac->prueth->fdb_tbl) {
		rtnl_unlock();
		return;
	}

	switch (fdb_work->event) {
	case FDB_LEARN:
		prueth_sw_insert_fdb_entry(emac, fdb_work->addr, 0);
		break;
	case FDB_PURGE:
		prueth_sw_do_purge_fdb(emac);
		break;
	default:
		break;
	}
	rtnl_unlock();

	kfree(fdb_work);
	dev_put(emac->ndev);
}

int prueth_sw_learn_fdb(struct prueth_emac *emac, u8 *src_mac)
{
	struct prueth_sw_fdb_work *fdb_work;

	fdb_work = kzalloc(sizeof(*fdb_work), GFP_ATOMIC);
	if (WARN_ON(!fdb_work))
		return -ENOMEM;

	INIT_WORK(&fdb_work->work, prueth_sw_fdb_work);

	fdb_work->event = FDB_LEARN;
	fdb_work->emac  = emac;
	ether_addr_copy(fdb_work->addr, src_mac);

	dev_hold(emac->ndev);
	queue_work(system_long_wq, &fdb_work->work);
	return 0;
}

static int prueth_sw_purge_fdb(struct prueth_emac *emac)
{
	struct prueth_sw_fdb_work *fdb_work;

	fdb_work = kzalloc(sizeof(*fdb_work), GFP_ATOMIC);
	if (WARN_ON(!fdb_work))
		return -ENOMEM;

	INIT_WORK(&fdb_work->work, prueth_sw_fdb_work);

	fdb_work->event = FDB_PURGE;
	fdb_work->emac  = emac;

	dev_hold(emac->ndev);
	queue_work(system_long_wq, &fdb_work->work);
	return 0;
}

int prueth_sw_init_fdb_table(struct prueth *prueth)
{
	if (prueth->emac_configured)
		return 0;

	prueth->fdb_tbl = kmalloc(sizeof(*prueth->fdb_tbl), GFP_KERNEL);
	if (!prueth->fdb_tbl)
		return -ENOMEM;

	prueth_sw_fdb_tbl_init(prueth);

	return 0;
}

int prueth_sw_boot_prus(struct prueth *prueth, struct net_device *ndev)
{
	const struct prueth_firmware *pru_firmwares;
	const char *fw_name, *fw_name1;
	int ret;

	if (prueth->emac_configured)
		return 0;

	pru_firmwares = &prueth->fw_data->fw_pru[PRUSS_PRU0];
	fw_name = pru_firmwares->fw_name[PRUSS_ETHTYPE_SWITCH];
	pru_firmwares = &prueth->fw_data->fw_pru[PRUSS_PRU1];
	fw_name1 = pru_firmwares->fw_name[PRUSS_ETHTYPE_SWITCH];

	ret = rproc_set_firmware(prueth->pru0, fw_name);
	if (ret) {
		netdev_err(ndev, "failed to set PRU0 firmware %s: %d\n",
			   fw_name, ret);
		return ret;
	}
	ret = rproc_boot(prueth->pru0);
	if (ret) {
		netdev_err(ndev, "failed to boot PRU: %d\n", ret);
		return ret;
	}

	ret = rproc_set_firmware(prueth->pru1, fw_name1);
	if (ret) {
		netdev_err(ndev, "failed to set PRU1 firmware %s: %d\n",
			   fw_name, ret);
		goto rproc0_shutdown;
	}
	ret = rproc_boot(prueth->pru1);
	if (ret) {
		netdev_err(ndev, "failed to boot PRU: %d\n", ret);
		goto rproc0_shutdown;
	}

	return 0;

rproc0_shutdown:
	rproc_shutdown(prueth->pru0);
	return ret;
}

int prueth_sw_shutdown_prus(struct prueth_emac *emac, struct net_device *ndev)
{
	struct prueth *prueth = emac->prueth;

	if (prueth->emac_configured)
		return 0;

	rproc_shutdown(prueth->pru0);
	rproc_shutdown(prueth->pru1);

	return 0;
}

static int prueth_switchdev_attr_set(struct net_device *ndev,
				     const struct switchdev_attr *attr,
				     struct switchdev_trans *trans)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth *prueth = emac->prueth;
	int err = 0;
	u8 o_state;

	/* Interface is not up */
	if (!prueth->fdb_tbl)
		return 0;

	switch (attr->id) {
	case SWITCHDEV_ATTR_ID_PORT_STP_STATE:
		o_state = prueth_sw_port_get_stp_state(prueth, emac->port_id);
		prueth_sw_port_set_stp_state(prueth, emac->port_id,
					     attr->u.stp_state);

		if (o_state != attr->u.stp_state)
			prueth_sw_purge_fdb(emac);

		dev_dbg(prueth->dev, "attr set: stp state:%u port:%u\n",
			attr->u.stp_state, emac->port_id);
		break;
	default:
		err = -EOPNOTSUPP;
		break;
	}

	return err;
}

/* switchdev notifiers */
static int prueth_sw_switchdev_blocking_event(struct notifier_block *unused,
					      unsigned long event, void *ptr)
{
	struct net_device *ndev = switchdev_notifier_info_to_dev(ptr);
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth *prueth = emac->prueth;
	int err;

	if (!PRUETH_IS_SWITCH(prueth))
		return NOTIFY_DONE;

	switch (event) {
	case SWITCHDEV_PORT_ATTR_SET:
		err = switchdev_handle_port_attr_set(ndev, ptr,
						     prueth_sw_port_dev_check,
						     prueth_switchdev_attr_set);
		return notifier_from_errno(err);
	default:
		break;
	}

	return NOTIFY_DONE;
}

/* switchev event work */
struct prueth_sw_switchdev_event_work {
	struct work_struct work;
	struct switchdev_notifier_fdb_info fdb_info;
	struct prueth_emac *emac;
	unsigned long event;
};

static void
prueth_sw_fdb_offload_notify(struct net_device *ndev,
			     struct switchdev_notifier_fdb_info *rcv)
{
	struct switchdev_notifier_fdb_info info;

	info.addr = rcv->addr;
	info.vid = rcv->vid;
	call_switchdev_notifiers(SWITCHDEV_FDB_OFFLOADED, ndev, &info.info,
				 NULL);
}

static void prueth_sw_fdb_add(struct prueth_emac *emac,
			      struct switchdev_notifier_fdb_info *fdb)
{
	prueth_sw_insert_fdb_entry(emac, fdb->addr, 1);
}

static void prueth_sw_fdb_del(struct prueth_emac *emac,
			      struct switchdev_notifier_fdb_info *fdb)
{
	prueth_sw_delete_fdb_entry(emac, fdb->addr, 1);
}

static void prueth_sw_switchdev_event_work(struct work_struct *work)
{
	struct prueth_sw_switchdev_event_work *switchdev_work =
		container_of(work, struct prueth_sw_switchdev_event_work, work);
	struct prueth_emac *emac = switchdev_work->emac;
	struct switchdev_notifier_fdb_info *fdb;
	struct prueth *prueth = emac->prueth;
	int port = emac->port_id;

	rtnl_lock();
	switch (switchdev_work->event) {
	case SWITCHDEV_FDB_ADD_TO_DEVICE:
		fdb = &switchdev_work->fdb_info;
		dev_dbg(prueth->dev,
			"prueth fdb add: MACID = %pM vid = %u flags = %u -- port %d\n",
			fdb->addr, fdb->vid, fdb->added_by_user, port);

		if (!fdb->added_by_user)
			break;

		prueth_sw_fdb_add(emac, fdb);
		prueth_sw_fdb_offload_notify(emac->ndev, fdb);
		break;
	case SWITCHDEV_FDB_DEL_TO_DEVICE:
		fdb = &switchdev_work->fdb_info;
		dev_dbg(prueth->dev,
			"prueth fdb del: MACID = %pM vid = %u flags = %u -- port %d\n",
			fdb->addr, fdb->vid, fdb->added_by_user, port);

		if (!fdb->added_by_user)
			break;

		prueth_sw_fdb_del(emac, fdb);
		break;
	default:
		break;
	}
	rtnl_unlock();

	kfree(switchdev_work->fdb_info.addr);
	kfree(switchdev_work);
	dev_put(emac->ndev);
}

/* called under rcu_read_lock() */
static int prueth_sw_switchdev_event(struct notifier_block *unused,
				     unsigned long event, void *ptr)
{
	struct net_device *ndev = switchdev_notifier_info_to_dev(ptr);
	struct switchdev_notifier_fdb_info *fdb_info = ptr;
	struct prueth_sw_switchdev_event_work *switchdev_work;
	struct prueth_emac *emac = netdev_priv(ndev);
	int err;

	netdev_dbg(ndev, "switchdev_event: event=%lu", event);

	if (event == SWITCHDEV_PORT_ATTR_SET) {
		err = switchdev_handle_port_attr_set(ndev, ptr,
						     prueth_sw_port_dev_check,
						     prueth_switchdev_attr_set);
		return notifier_from_errno(err);
	}

	if (!prueth_sw_port_dev_check(ndev))
		return NOTIFY_DONE;

	switchdev_work = kzalloc(sizeof(*switchdev_work), GFP_ATOMIC);
	if (WARN_ON(!switchdev_work))
		return NOTIFY_BAD;

	INIT_WORK(&switchdev_work->work, prueth_sw_switchdev_event_work);
	switchdev_work->emac = emac;
	switchdev_work->event = event;

	switch (event) {
	case SWITCHDEV_FDB_ADD_TO_DEVICE:
	case SWITCHDEV_FDB_DEL_TO_DEVICE:
		memcpy(&switchdev_work->fdb_info, ptr,
		       sizeof(switchdev_work->fdb_info));
		switchdev_work->fdb_info.addr = kzalloc(ETH_ALEN, GFP_ATOMIC);
		if (!switchdev_work->fdb_info.addr)
			goto err_addr_alloc;
		ether_addr_copy((u8 *)switchdev_work->fdb_info.addr,
				fdb_info->addr);
		dev_hold(ndev);
		break;
	default:
		kfree(switchdev_work);
		return NOTIFY_DONE;
	}

	queue_work(system_long_wq, &switchdev_work->work);

	return NOTIFY_DONE;

err_addr_alloc:
	kfree(switchdev_work);
	return NOTIFY_BAD;
}

int prueth_sw_register_notifiers(struct prueth *prueth)
{
	struct notifier_block *nb;
	int ret;

	nb = &prueth->prueth_sw_switchdev_notifier;
	nb->notifier_call = prueth_sw_switchdev_event;
	ret = register_switchdev_notifier(nb);
	if (ret) {
		dev_err(prueth->dev,
			"register switchdev notifier failed ret:%d\n", ret);
		return ret;
	}

	nb = &prueth->prueth_sw_switchdev_bl_notifier;
	nb->notifier_call = prueth_sw_switchdev_blocking_event;
	ret = register_switchdev_blocking_notifier(nb);
	if (ret) {
		dev_err(prueth->dev, "register switchdev blocking notifier failed ret:%d\n",
			ret);
		nb = &prueth->prueth_sw_switchdev_notifier;
		unregister_switchdev_notifier(nb);
		return ret;
	}

	return 0;
}

void prueth_sw_unregister_notifiers(struct prueth *prueth)
{
	unregister_switchdev_blocking_notifier(&prueth->prueth_sw_switchdev_bl_notifier);
	unregister_switchdev_notifier(&prueth->prueth_sw_switchdev_notifier);
}
