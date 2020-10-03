/* SPDX-License-Identifier: GPL-2.0 */

/* PRU ICSS Ethernet Driver
 *
 * Copyright (C) 2015-2018 Texas Instruments Incorporated - http://www.ti.com
 *	Roger Quadros <rogerq@ti.com>
 *	Andrew F. Davis <afd@ti.com>
 */

#include <linux/etherdevice.h>
#include <linux/genalloc.h>
#include <linux/if_bridge.h>
#include <linux/if_vlan.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/net_tstamp.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/of_platform.h>
#include <linux/phy.h>
#include <linux/pruss.h>
#include <linux/ptp_classify.h>
#include <linux/regmap.h>
#include <linux/remoteproc.h>
#include <net/pkt_cls.h>

#include "prueth.h"
#include "icss_mii_rt.h"
#include "icss_vlan_mcast_filter_mmap.h"
#include "prueth_ecap.h"
#include "prueth_lre.h"
#include "prueth_switch.h"
#include "icss_iep.h"

#define PRUETH_MODULE_VERSION "0.2"
#define PRUETH_MODULE_DESCRIPTION "PRUSS Ethernet driver"

#define OCMC_RAM_SIZE		(SZ_64K - SZ_8K)
#define PRUETH_ETH_TYPE_OFFSET		12
#define PRUETH_ETH_TYPE_UPPER_SHIFT	8

/* TX Minimum Inter packet gap */
#define TX_MIN_IPG		0xb8

#define TX_START_DELAY		0x40
#define TX_CLK_DELAY_100M	0x6
#define TX_CLK_DELAY_10M	0

/* PRUSS_IEP_GLOBAL_CFG register definitions */
#define PRUSS_IEP_GLOBAL_CFG	0

#define PRUSS_IEP_GLOBAL_CFG_CNT_ENABLE		BIT(0)

/* Netif debug messages possible */
#define PRUETH_EMAC_DEBUG	(NETIF_MSG_DRV | \
				 NETIF_MSG_PROBE | \
				 NETIF_MSG_LINK | \
				 NETIF_MSG_TIMER | \
				 NETIF_MSG_IFDOWN | \
				 NETIF_MSG_IFUP | \
				 NETIF_MSG_RX_ERR | \
				 NETIF_MSG_TX_ERR | \
				 NETIF_MSG_TX_QUEUED | \
				 NETIF_MSG_INTR | \
				 NETIF_MSG_TX_DONE | \
				 NETIF_MSG_RX_STATUS | \
				 NETIF_MSG_PKTDATA | \
				 NETIF_MSG_HW | \
				 NETIF_MSG_WOL)

static int debug_level = -1;
module_param(debug_level, int, 0444);
MODULE_PARM_DESC(debug_level, "PRUETH debug level (NETIF_MSG bits)");

/* ensure that order of PRUSS mem regions is same as enum prueth_mem */
static enum pruss_mem pruss_mem_ids[] = { PRUSS_MEM_DRAM0, PRUSS_MEM_DRAM1,
					  PRUSS_MEM_SHRD_RAM2 };

static struct prueth_fw_offsets fw_offsets_v1_0 = {
	.hash_mask = ICSS_LRE_V1_0_HASH_MASK,
	.index_array_offset = ICSS_LRE_V1_0_INDEX_ARRAY_NT,
	.bin_array_offset = ICSS_LRE_V1_0_BIN_ARRAY,
	.nt_array_offset = ICSS_LRE_V1_0_NODE_TABLE_NEW,
	.index_array_loc = ICSS_LRE_V1_0_INDEX_ARRAY_LOC,
	.bin_array_loc = ICSS_LRE_V1_0_BIN_ARRAY_LOC,
	.nt_array_loc = ICSS_LRE_V1_0_NODE_TABLE_LOC,
	.index_array_max_entries = ICSS_LRE_V1_0_INDEX_TBL_MAX_ENTRIES,
	.bin_array_max_entries = ICSS_LRE_V1_0_BIN_TBL_MAX_ENTRIES,
	.nt_array_max_entries = ICSS_LRE_V1_0_NODE_TBL_MAX_ENTRIES,
	.iep_wrap = NSEC_PER_SEC,
};

static struct prueth_fw_offsets fw_offsets_v2_1 = {
	.hash_mask = ICSS_LRE_V2_1_HASH_MASK,
	.index_array_offset = ICSS_LRE_V2_1_INDEX_ARRAY_NT,
	.bin_array_offset = ICSS_LRE_V2_1_BIN_ARRAY,
	.nt_array_offset = ICSS_LRE_V2_1_NODE_TABLE_NEW,
	.index_array_loc = ICSS_LRE_V2_1_INDEX_ARRAY_LOC,
	.bin_array_loc = ICSS_LRE_V2_1_BIN_ARRAY_LOC,
	.nt_array_loc = ICSS_LRE_V2_1_NODE_TABLE_LOC,
	.index_array_max_entries = ICSS_LRE_V2_1_INDEX_TBL_MAX_ENTRIES,
	.bin_array_max_entries = ICSS_LRE_V2_1_BIN_TBL_MAX_ENTRIES,
	.nt_array_max_entries = ICSS_LRE_V2_1_NODE_TBL_MAX_ENTRIES,
	.iep_wrap = 0xffffffff,
};

static void prueth_set_fw_offsets(struct prueth *prueth)
{
	/* Set VLAN/Multicast filter control and table offsets */
	if (PRUETH_IS_EMAC(prueth) || PRUETH_IS_SWITCH(prueth)) {
		prueth->fw_offsets->vlan_ctrl_byte  =
			ICSS_EMAC_FW_VLAN_FILTER_CTRL_BITMAP_OFFSET;
		prueth->fw_offsets->vlan_filter_tbl =
			ICSS_EMAC_FW_VLAN_FLTR_TBL_BASE_ADDR;

		prueth->fw_offsets->mc_ctrl_byte  =
			ICSS_EMAC_FW_MULTICAST_FILTER_CTRL_OFFSET;
		prueth->fw_offsets->mc_filter_mask =
			ICSS_EMAC_FW_MULTICAST_FILTER_MASK_OFFSET;
		prueth->fw_offsets->mc_filter_tbl =
			ICSS_EMAC_FW_MULTICAST_FILTER_TABLE;

	} else {
		prueth->fw_offsets->vlan_ctrl_byte  =
			ICSS_LRE_FW_VLAN_FLTR_CTRL_BYTE;
		prueth->fw_offsets->vlan_filter_tbl =
			ICSS_LRE_FW_VLAN_FLTR_TBL_BASE_ADDR;

		prueth->fw_offsets->mc_ctrl_byte  =
			ICSS_LRE_FW_MULTICAST_TABLE_SEARCH_OP_CONTROL_BIT;
		prueth->fw_offsets->mc_filter_mask =
			ICSS_LRE_FW_MULTICAST_FILTER_MASK;
		prueth->fw_offsets->mc_filter_tbl =
			ICSS_LRE_FW_MULTICAST_FILTER_TABLE;

	}
}

static inline u32 prueth_read_reg(struct prueth *prueth,
				  enum prueth_mem region,
				  unsigned int reg)
{
	return readl_relaxed(prueth->mem[region].va + reg);
}

static inline void prueth_write_reg(struct prueth *prueth,
				    enum prueth_mem region,
				    unsigned int reg, u32 val)
{
	writel_relaxed(val, prueth->mem[region].va + reg);
}

static inline void prueth_ptp_ts_enable(struct prueth_emac *emac)
{
	void __iomem *sram = emac->prueth->mem[PRUETH_MEM_SHARED_RAM].va;
	u8 val = 0;

	if (emac->ptp_tx_enable) {
		/* Disable fw background task */
		val &= ~TIMESYNC_CTRL_BG_ENABLE;
		/* Enable forced 2-step */
		val |= TIMESYNC_CTRL_FORCED_2STEP;
	}

	writeb(val, sram + TIMESYNC_CTRL_VAR_OFFSET);
}

static inline void prueth_ptp_tx_ts_enable(struct prueth_emac *emac, bool enable)
{
	emac->ptp_tx_enable = enable;
	prueth_ptp_ts_enable(emac);
}

static inline bool prueth_ptp_tx_ts_is_enabled(struct prueth_emac *emac)
{
	return !!emac->ptp_tx_enable;
}

static inline void prueth_ptp_rx_ts_enable(struct prueth_emac *emac, bool enable)
{
	emac->ptp_rx_enable = enable;
	prueth_ptp_ts_enable(emac);
}

static inline bool prueth_ptp_rx_ts_is_enabled(struct prueth_emac *emac)
{
	return !!emac->ptp_rx_enable;
}

static inline
void prueth_set_reg(struct prueth *prueth, enum prueth_mem region,
		    unsigned int reg, u32 mask, u32 set)
{
	u32 val;

	val = prueth_read_reg(prueth, region, reg);
	val &= ~mask;
	val |= (set & mask);
	prueth_write_reg(prueth, region, reg, val);
}

static const struct prueth_queue_info queue_infos[][NUM_QUEUES] = {
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
			P1_Q1_BUFFER_OFFSET + ((QUEUE_1_SIZE - 1) * ICSS_BLOCK_SIZE),
			P1_Q1_BD_OFFSET,
			P1_Q1_BD_OFFSET + ((QUEUE_1_SIZE - 1) * BD_SIZE),
		},
		[PRUETH_QUEUE2] = {
			P1_Q2_BUFFER_OFFSET,
			P1_Q2_BUFFER_OFFSET + ((QUEUE_2_SIZE - 1) * ICSS_BLOCK_SIZE),
			P1_Q2_BD_OFFSET,
			P1_Q2_BD_OFFSET + ((QUEUE_2_SIZE - 1) * BD_SIZE),
		},
		[PRUETH_QUEUE3] = {
			P1_Q3_BUFFER_OFFSET,
			P1_Q3_BUFFER_OFFSET + ((QUEUE_3_SIZE - 1) * ICSS_BLOCK_SIZE),
			P1_Q3_BD_OFFSET,
			P1_Q3_BD_OFFSET + ((QUEUE_3_SIZE - 1) * BD_SIZE),
		},
		[PRUETH_QUEUE4] = {
			P1_Q4_BUFFER_OFFSET,
			P1_Q4_BUFFER_OFFSET + ((QUEUE_4_SIZE - 1) * ICSS_BLOCK_SIZE),
			P1_Q4_BD_OFFSET,
			P1_Q4_BD_OFFSET + ((QUEUE_4_SIZE - 1) * BD_SIZE),
		},
	},
	[PRUETH_PORT_QUEUE_MII1] = {
		[PRUETH_QUEUE1] = {
			P2_Q1_BUFFER_OFFSET,
			P2_Q1_BUFFER_OFFSET + ((QUEUE_1_SIZE - 1) * ICSS_BLOCK_SIZE),
			P2_Q1_BD_OFFSET,
			P2_Q1_BD_OFFSET + ((QUEUE_1_SIZE - 1) * BD_SIZE),
		},
		[PRUETH_QUEUE2] = {
			P2_Q2_BUFFER_OFFSET,
			P2_Q2_BUFFER_OFFSET + ((QUEUE_2_SIZE - 1) * ICSS_BLOCK_SIZE),
			P2_Q2_BD_OFFSET,
			P2_Q2_BD_OFFSET + ((QUEUE_2_SIZE - 1) * BD_SIZE),
		},
		[PRUETH_QUEUE3] = {
			P2_Q3_BUFFER_OFFSET,
			P2_Q3_BUFFER_OFFSET + ((QUEUE_3_SIZE - 1) * ICSS_BLOCK_SIZE),
			P2_Q3_BD_OFFSET,
			P2_Q3_BD_OFFSET + ((QUEUE_3_SIZE - 1) * BD_SIZE),
		},
		[PRUETH_QUEUE4] = {
			P2_Q4_BUFFER_OFFSET,
			P2_Q4_BUFFER_OFFSET + ((QUEUE_4_SIZE - 1) * ICSS_BLOCK_SIZE),
			P2_Q4_BD_OFFSET,
			P2_Q4_BD_OFFSET + ((QUEUE_4_SIZE - 1) * BD_SIZE),
		},
	},
};

const struct prueth_queue_desc queue_descs[][NUM_QUEUES] = {
	[PRUETH_PORT_QUEUE_HOST] = {
		{ .rd_ptr = P0_Q1_BD_OFFSET, .wr_ptr = P0_Q1_BD_OFFSET, },
		{ .rd_ptr = P0_Q2_BD_OFFSET, .wr_ptr = P0_Q2_BD_OFFSET, },
		{ .rd_ptr = P0_Q3_BD_OFFSET, .wr_ptr = P0_Q3_BD_OFFSET, },
		{ .rd_ptr = P0_Q4_BD_OFFSET, .wr_ptr = P0_Q4_BD_OFFSET, },
	},
	[PRUETH_PORT_QUEUE_MII0] = {
		{ .rd_ptr = P1_Q1_BD_OFFSET, .wr_ptr = P1_Q1_BD_OFFSET, },
		{ .rd_ptr = P1_Q2_BD_OFFSET, .wr_ptr = P1_Q2_BD_OFFSET, },
		{ .rd_ptr = P1_Q3_BD_OFFSET, .wr_ptr = P1_Q3_BD_OFFSET, },
		{ .rd_ptr = P1_Q4_BD_OFFSET, .wr_ptr = P1_Q4_BD_OFFSET, },
	},
	[PRUETH_PORT_QUEUE_MII1] = {
		{ .rd_ptr = P2_Q1_BD_OFFSET, .wr_ptr = P2_Q1_BD_OFFSET, },
		{ .rd_ptr = P2_Q2_BD_OFFSET, .wr_ptr = P2_Q2_BD_OFFSET, },
		{ .rd_ptr = P2_Q3_BD_OFFSET, .wr_ptr = P2_Q3_BD_OFFSET, },
		{ .rd_ptr = P2_Q4_BD_OFFSET, .wr_ptr = P2_Q4_BD_OFFSET, },
	}
};

static enum hrtimer_restart prueth_timer(struct hrtimer *timer)
{
	struct prueth *prueth = container_of(timer, struct prueth,
					     tbl_check_timer);
	enum hrtimer_restart ret = HRTIMER_NORESTART;
	unsigned int timeout = PRUETH_NSP_TIMER_MS;
	struct prueth_emac *emac;
	enum prueth_mac mac;
	unsigned long flags;

	if (PRUETH_IS_LRE(prueth))
		timeout = PRUETH_TIMER_MS;

	hrtimer_forward_now(timer, ms_to_ktime(timeout));
	if (PRUETH_IS_LRE(prueth) &&
	    prueth->emac_configured !=
	    (BIT(PRUETH_PORT_MII0) | BIT(PRUETH_PORT_MII1)))
		return HRTIMER_RESTART;

	for (mac = PRUETH_MAC0; mac <= PRUETH_MAC1; mac++) {
		emac = prueth->emac[mac];

		/* skip if in single emac mode */
		if (!emac)
			continue;

		if (!netif_running(emac->ndev))
			continue;

		spin_lock_irqsave(&emac->nsp_lock, flags);

		if (!emac->nsp_enabled) {
			spin_unlock_irqrestore(&emac->nsp_lock, flags);
			continue;
		}

		ret = HRTIMER_RESTART;
		prueth_enable_nsp(emac);
		spin_unlock_irqrestore(&emac->nsp_lock, flags);
	}

	if (PRUETH_IS_LRE(prueth)) {
		ret = HRTIMER_RESTART;
		prueth_lre_process_check_flags_event(prueth);
	}

	return ret;
}

static void prueth_init_timer(struct prueth *prueth)
{
	hrtimer_init(&prueth->tbl_check_timer, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL);
	prueth->tbl_check_timer.function = prueth_timer;
}

void prueth_start_timer(struct prueth_emac *emac)
{
	unsigned int timeout = PRUETH_NSP_TIMER_MS;

	if (hrtimer_active(&emac->prueth->tbl_check_timer))
		return;

	if (PRUETH_IS_LRE(emac->prueth)) {
		timeout = PRUETH_TIMER_MS;
		emac->nsp_timer_count = PRUETH_NSP_TIMER_COUNT;
	}

	hrtimer_start(&emac->prueth->tbl_check_timer, ms_to_ktime(timeout),
		      HRTIMER_MODE_REL);
}

static void prueth_hostconfig(struct prueth *prueth)
{
	void __iomem *sram_base = prueth->mem[PRUETH_MEM_SHARED_RAM].va;
	void __iomem *sram;

	/* queue size lookup table */
	sram = sram_base + HOST_QUEUE_SIZE_ADDR;
	writew(HOST_QUEUE_1_SIZE, sram);
	writew(HOST_QUEUE_2_SIZE, sram + 2);
	writew(HOST_QUEUE_3_SIZE, sram + 4);
	writew(HOST_QUEUE_4_SIZE, sram + 6);

	/* queue information table */
	sram = sram_base + HOST_Q1_RX_CONTEXT_OFFSET;
	memcpy_toio(sram, queue_infos[PRUETH_PORT_QUEUE_HOST],
		    sizeof(queue_infos[PRUETH_PORT_QUEUE_HOST]));

	/* buffer offset table */
	sram = sram_base + HOST_QUEUE_OFFSET_ADDR;
	writew(P0_Q1_BUFFER_OFFSET, sram);
	writew(P0_Q2_BUFFER_OFFSET, sram + 2);
	writew(P0_Q3_BUFFER_OFFSET, sram + 4);
	writew(P0_Q4_BUFFER_OFFSET, sram + 6);

	/* buffer descriptor offset table*/
	sram = sram_base + HOST_QUEUE_DESCRIPTOR_OFFSET_ADDR;
	writew(P0_Q1_BD_OFFSET, sram);
	writew(P0_Q2_BD_OFFSET, sram + 2);
	writew(P0_Q3_BD_OFFSET, sram + 4);
	writew(P0_Q4_BD_OFFSET, sram + 6);

	/* queue table */
	sram = sram_base + HOST_QUEUE_DESC_OFFSET;
	memcpy_toio(sram, queue_descs[PRUETH_PORT_QUEUE_HOST],
		    sizeof(queue_descs[PRUETH_PORT_QUEUE_HOST]));
}

#define prueth_mii_set(dir, port, mask, set) \
	regmap_update_bits(prueth->mii_rt, PRUSS_MII_RT_##dir##CFG##port, \
			   PRUSS_MII_RT_##dir##CFG_##dir##_##mask, set)

static void prueth_mii_init(struct prueth *prueth)
{
	/* Configuration of Port 0 Rx */
	prueth_mii_set(RX, 0, ENABLE, PRUSS_MII_RT_RXCFG_RX_ENABLE);
	prueth_mii_set(RX, 0, DATA_RDY_MODE_DIS,
		       PRUSS_MII_RT_RXCFG_RX_DATA_RDY_MODE_DIS);
	prueth_mii_set(RX, 0, MUX_SEL, 0x0);
	prueth_mii_set(RX, 0, L2_EN, PRUSS_MII_RT_RXCFG_RX_L2_EN);
	prueth_mii_set(RX, 0, CUT_PREAMBLE, PRUSS_MII_RT_RXCFG_RX_CUT_PREAMBLE);
	prueth_mii_set(RX, 0, L2_EOF_SCLR_DIS,
		       PRUSS_MII_RT_RXCFG_RX_L2_EOF_SCLR_DIS);

	/* Configuration of Port 0 Tx */
	prueth_mii_set(TX, 0, ENABLE, PRUSS_MII_RT_TXCFG_TX_ENABLE);
	prueth_mii_set(TX, 0, AUTO_PREAMBLE,
		       PRUSS_MII_RT_TXCFG_TX_AUTO_PREAMBLE);
	prueth_mii_set(TX, 0, 32_MODE_EN, PRUSS_MII_RT_TXCFG_TX_32_MODE_EN);
	if (!PRUETH_IS_EMAC(prueth))
		prueth_mii_set(TX, 0, MUX_SEL, PRUSS_MII_RT_TXCFG_TX_MUX_SEL);
	else
		prueth_mii_set(TX, 0, MUX_SEL, 0x0);
	prueth_mii_set(TX, 0, START_DELAY_MASK,
		       TX_START_DELAY << PRUSS_MII_RT_TXCFG_TX_START_DELAY_SHIFT);
	prueth_mii_set(TX, 0, CLK_DELAY_MASK,
		       TX_CLK_DELAY_100M << PRUSS_MII_RT_TXCFG_TX_CLK_DELAY_SHIFT);

	/* Configuration of Port 1 Rx */
	prueth_mii_set(RX, 1, ENABLE, PRUSS_MII_RT_RXCFG_RX_ENABLE);
	prueth_mii_set(RX, 1,
		       DATA_RDY_MODE_DIS, PRUSS_MII_RT_RXCFG_RX_DATA_RDY_MODE_DIS);
	prueth_mii_set(RX, 1, MUX_SEL, PRUSS_MII_RT_RXCFG_RX_MUX_SEL);
	prueth_mii_set(RX, 1, L2_EN, PRUSS_MII_RT_RXCFG_RX_L2_EN);
	prueth_mii_set(RX, 1, CUT_PREAMBLE, PRUSS_MII_RT_RXCFG_RX_CUT_PREAMBLE);
	prueth_mii_set(RX, 1, L2_EOF_SCLR_DIS,
		       PRUSS_MII_RT_RXCFG_RX_L2_EOF_SCLR_DIS);

	/* Configuration of Port 1 Tx */
	prueth_mii_set(TX, 1, ENABLE, PRUSS_MII_RT_TXCFG_TX_ENABLE);
	prueth_mii_set(TX, 1, AUTO_PREAMBLE,
		       PRUSS_MII_RT_TXCFG_TX_AUTO_PREAMBLE);
	prueth_mii_set(TX, 1, 32_MODE_EN, PRUSS_MII_RT_TXCFG_TX_32_MODE_EN);
	if (!PRUETH_IS_EMAC(prueth))
		prueth_mii_set(TX, 1, MUX_SEL, 0x0);
	else
		prueth_mii_set(TX, 1, MUX_SEL, PRUSS_MII_RT_TXCFG_TX_MUX_SEL);
	prueth_mii_set(TX, 1, START_DELAY_MASK,
		       TX_START_DELAY << PRUSS_MII_RT_TXCFG_TX_START_DELAY_SHIFT);
	prueth_mii_set(TX, 1, CLK_DELAY_MASK,
		       TX_CLK_DELAY_100M << PRUSS_MII_RT_TXCFG_TX_CLK_DELAY_SHIFT);

	/* Min frame length should be set to 64 to allow receive of standard
	 * Ethernet frames such as PTP, LLDP that will not have the tag/rct.
	 * Actual size written to register is size - 1 per TRM. This also
	 * includes CRC/FCS.
	 */
	regmap_update_bits(prueth->mii_rt,
			   PRUSS_MII_RT_RX_FRMS0,
			   PRUSS_MII_RT_RX_FRMS_MIN_FRM_MASK,
			   (PRUSS_MII_RT_RX_FRMS_MIN_FRM - 1) <<
			   PRUSS_MII_RT_RX_FRMS_MIN_FRM_SHIFT);

	regmap_update_bits(prueth->mii_rt,
			   PRUSS_MII_RT_RX_FRMS1,
			   PRUSS_MII_RT_RX_FRMS_MIN_FRM_MASK,
			   (PRUSS_MII_RT_RX_FRMS_MIN_FRM - 1) <<
			   PRUSS_MII_RT_RX_FRMS_MIN_FRM_SHIFT);

	/* For EMAC, set Max frame size to 1522 i.e size with VLAN and for
	 * HSR/PRP set it to 1528 i.e size with tag or rct. Actual size
	 * written to register is size - 1 as per TRM. Since driver
	 * support run time change of protocol, driver must overwrite
	 * the values based on Ethernet type.
	 */
	if (PRUETH_IS_LRE(prueth)) {
		regmap_update_bits(prueth->mii_rt,
				   PRUSS_MII_RT_RX_FRMS0,
				   PRUSS_MII_RT_RX_FRMS_MAX_FRM_MASK,
				   (PRUSS_MII_RT_RX_FRMS_MAX_FRM_LRE - 1) <<
				   PRUSS_MII_RT_RX_FRMS_MAX_FRM_SHIFT);

		regmap_update_bits(prueth->mii_rt,
				   PRUSS_MII_RT_RX_FRMS1,
				   PRUSS_MII_RT_RX_FRMS_MAX_FRM_MASK,
				   (PRUSS_MII_RT_RX_FRMS_MAX_FRM_LRE - 1) <<
				   PRUSS_MII_RT_RX_FRMS_MAX_FRM_SHIFT);
	} else {
		regmap_update_bits(prueth->mii_rt,
				   PRUSS_MII_RT_RX_FRMS0,
				   PRUSS_MII_RT_RX_FRMS_MAX_FRM_MASK,
				   (PRUSS_MII_RT_RX_FRMS_MAX - 1) <<
				   PRUSS_MII_RT_RX_FRMS_MAX_FRM_SHIFT);

		regmap_update_bits(prueth->mii_rt,
				   PRUSS_MII_RT_RX_FRMS1,
				   PRUSS_MII_RT_RX_FRMS_MAX_FRM_MASK,
				   (PRUSS_MII_RT_RX_FRMS_MAX - 1) <<
				   PRUSS_MII_RT_RX_FRMS_MAX_FRM_SHIFT);
	}
}

static void prueth_clearmem(struct prueth *prueth, enum prueth_mem region)
{
	memset_io(prueth->mem[region].va, 0, prueth->mem[region].size);
}

static void prueth_hostinit(struct prueth *prueth)
{
	/* Clear shared RAM */
	prueth_clearmem(prueth, PRUETH_MEM_SHARED_RAM);

	/* Clear OCMC RAM */
	prueth_clearmem(prueth, PRUETH_MEM_OCMC);

	/* Clear data RAMs */
	if (prueth->eth_node[PRUETH_MAC0])
		prueth_clearmem(prueth, PRUETH_MEM_DRAM0);
	if (prueth->eth_node[PRUETH_MAC1])
		prueth_clearmem(prueth, PRUETH_MEM_DRAM1);

	/* Initialize host queues in shared RAM */
	if (!PRUETH_IS_EMAC(prueth))
		prueth_sw_hostconfig(prueth);
	else
		prueth_hostconfig(prueth);

	/* Configure MII_RT */
	prueth_mii_init(prueth);
}

/* This function initialize the driver in EMAC or HSR or PRP mode
 * based on eth_type
 */
static void prueth_init_ethernet_mode(struct prueth *prueth)
{
	prueth_set_fw_offsets(prueth);
	prueth_hostinit(prueth);
	if (PRUETH_IS_LRE(prueth))
		prueth_lre_config(prueth);
}

static void prueth_port_enable(struct prueth_emac *emac, bool enable)
{
	void __iomem *port_ctrl, *vlan_ctrl;
	struct prueth *prueth = emac->prueth;
	u32 vlan_ctrl_offset = prueth->fw_offsets->vlan_ctrl_byte;
	void __iomem *ram = prueth->mem[emac->dram].va;

	port_ctrl = ram + PORT_CONTROL_ADDR;
	writeb(!!enable, port_ctrl);

	/* HSR/PRP firmware use a different memory and offset
	 * for VLAN filter control
	 */
	if (PRUETH_IS_LRE(prueth))
		ram = prueth->mem[PRUETH_MEM_SHARED_RAM].va;
	vlan_ctrl = ram + vlan_ctrl_offset;
	writeb(!!enable, vlan_ctrl);
}

static void prueth_emac_config(struct prueth_emac *emac)
{
	struct prueth *prueth = emac->prueth;

	/* PRU needs local shared RAM address for C28 */
	u32 sharedramaddr = ICSS_LOCAL_SHARED_RAM;

	/* PRU needs real global OCMC address for C30*/
	u32 ocmcaddr = (u32)prueth->mem[PRUETH_MEM_OCMC].pa;
	void __iomem *sram = prueth->mem[PRUETH_MEM_SHARED_RAM].va;
	void __iomem *dram_base;
	void __iomem *mac_addr;
	void __iomem *dram;

	/* Clear data RAM */
	prueth_clearmem(prueth, emac->dram);

	dram_base = prueth->mem[emac->dram].va;

	/* setup mac address */
	mac_addr = dram_base + PORT_MAC_ADDR;
	memcpy_toio(mac_addr, emac->mac_addr, 6);

	/* queue information table */
	dram = dram_base + TX_CONTEXT_Q1_OFFSET_ADDR;
	memcpy_toio(dram, queue_infos[emac->port_id],
		    sizeof(queue_infos[emac->port_id]));

	/* queue table */
	dram = dram_base + PORT_QUEUE_DESC_OFFSET;
	memcpy_toio(dram, queue_descs[emac->port_id],
		    sizeof(queue_descs[emac->port_id]));

	emac->rx_queue_descs = sram + HOST_QUEUE_DESC_OFFSET;
	emac->tx_queue_descs = dram;

	/* Set in constant table C28 of PRU0 to ICSS Shared memory */
	pru_rproc_set_ctable(emac->pru, PRU_C28, sharedramaddr);

	/* Set in constant table C30 of PRU0 to OCMC memory */
	pru_rproc_set_ctable(emac->pru, PRU_C30, ocmcaddr);
}

/* update phy/port status information for firmware */
static void emac_update_phystatus(struct prueth_emac *emac)
{
	struct prueth *prueth = emac->prueth;
	enum prueth_mem region;
	u32 phy_speed, port_status = 0;
	u8 delay;

	region = emac->dram;
	phy_speed = emac->speed;
	prueth_write_reg(prueth, region, PHY_SPEED_OFFSET, phy_speed);

	if (phy_speed == SPEED_10)
		delay = TX_CLK_DELAY_10M;
	else
		delay = TX_CLK_DELAY_100M;

	if (emac->port_id) {
		prueth_mii_set(TX, 1, CLK_DELAY_MASK,
			       delay << PRUSS_MII_RT_TXCFG_TX_CLK_DELAY_SHIFT);
	} else {
		prueth_mii_set(TX, 0, CLK_DELAY_MASK,
			       delay << PRUSS_MII_RT_TXCFG_TX_CLK_DELAY_SHIFT);
	}

	if (emac->duplex == DUPLEX_HALF)
		port_status |= PORT_IS_HD_MASK;
	if (emac->link)
		port_status |= PORT_LINK_MASK;
	writeb(port_status, prueth->mem[region].va + PORT_STATUS_OFFSET);
}

/* called back by PHY layer if there is change in link state of hw port*/
static void emac_adjust_link(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct phy_device *phydev = emac->phydev;
	unsigned long flags;
	bool new_state = false;

	spin_lock_irqsave(&emac->lock, flags);

	if (phydev->link) {
		/* check the mode of operation - full/half duplex */
		if (phydev->duplex != emac->duplex) {
			new_state = true;
			emac->duplex = phydev->duplex;
		}
		if (phydev->speed != emac->speed) {
			new_state = true;
			emac->speed = phydev->speed;
		}
		if (!emac->link) {
			new_state = true;
			emac->link = 1;
		}
	} else if (emac->link) {
		new_state = true;
		emac->link = 0;
		/* defaults for no link */

		/* f/w only support 10 or 100 */
		emac->speed = SPEED_100;

		/* half duplex may not be supported by f/w */
		emac->duplex = DUPLEX_FULL;
	}

	emac_update_phystatus(emac);

	if (new_state)
		phy_print_status(phydev);

	if (emac->link) {
		/* link ON */
		netif_carrier_on(ndev);

		/* reactivate the transmit queue if it is stopped */
		if (netif_running(ndev) && netif_queue_stopped(ndev))
			netif_wake_queue(ndev);
	} else {
		/* link OFF */
		netif_carrier_off(ndev);
		if (!netif_queue_stopped(ndev))
			netif_stop_queue(ndev);
	}

	spin_unlock_irqrestore(&emac->lock, flags);
}

/**
 * emac_tx_hardirq - EMAC Tx interrupt handler
 * @irq: interrupt number
 * @dev_id: pointer to net_device
 *
 * This is called whenever a packet has finished being transmitted, this clears
 * up hardware buffer space, our only task is to re-enable the transmit queue
 * if it was previously disabled due to hardware queue being full
 *
 * Returns interrupt handled condition
 */
static irqreturn_t emac_tx_hardirq(int irq, void *dev_id)
{
	struct net_device *ndev = (struct net_device *)dev_id;

	if (unlikely(netif_queue_stopped(ndev)))
		netif_wake_queue(ndev);

	return IRQ_HANDLED;
}

/**
 * emac_rx_hardirq - EMAC Rx interrupt handler
 * @irq: interrupt number
 * @dev_id: pointer to net_device
 *
 * EMAC Interrupt handler - we only schedule NAPI and not process any packets
 * here.
 *
 * Returns interrupt handled condition
 */
static irqreturn_t emac_rx_hardirq(int irq, void *dev_id)
{
	struct net_device *ndev = (struct net_device *)dev_id;
	struct prueth_emac *emac = netdev_priv(ndev);

	if (likely(netif_running(ndev))) {
		/* disable Rx system event */
		disable_irq_nosync(emac->rx_irq);
		napi_schedule(&emac->napi);
	}

	return IRQ_HANDLED;
}

static u8 ptp_event_type(struct sk_buff *skb)
{
	unsigned int offset = 0, ptp_class = ptp_classify_raw(skb);
	u8 *msgtype, *data;
	u16 *seqid;

	data = skb->data;
	if (ptp_class == PTP_CLASS_NONE)
		return PTP_EVENT_MASK;

	if (ptp_class & PTP_CLASS_VLAN)
		offset += VLAN_HLEN;

	switch (ptp_class & PTP_CLASS_PMASK) {
	case PTP_CLASS_IPV4:
		offset += ETH_HLEN + IPV4_HLEN(data + offset) + UDP_HLEN;
		break;
	case PTP_CLASS_IPV6:
		offset += ETH_HLEN + IP6_HLEN + UDP_HLEN;
		break;
	case PTP_CLASS_L2:
		offset += ETH_HLEN;
		break;
	default:
		return PTP_EVENT_MASK;
	}

	if (skb->len + ETH_HLEN < offset + OFF_PTP_SEQUENCE_ID + sizeof(*seqid))
		return PTP_EVENT_MASK;

	if (unlikely(ptp_class & PTP_CLASS_V1))
		msgtype = data + offset + OFF_PTP_CONTROL;
	else
		msgtype = data + offset;

	return (*msgtype & PTP_EVENT_MASK);
}

static u8 prueth_ptp_ts_event_type(struct sk_buff *skb)
{
	u8 ptp_type, event_type;

	ptp_type = ptp_event_type(skb);
	/*
	 * Treat E2E Delay Req/Resp messages sane as P2P peer delay req/resp
	 * in driver here since firmware stores timestamps in the same memory
	 * location for either (since they cannot operate simultaneously
	 * anyway)
	 */
	switch (ptp_type) {
	case PTP_SYNC_MSG_ID:
		event_type = PRUETH_PTP_SYNC;
		break;
	case PTP_DLY_REQ_MSG_ID:
	case PTP_PDLY_REQ_MSG_ID:
		event_type = PRUETH_PTP_DLY_REQ;
		break;
	case PTP_DLY_RESP_MSG_ID:
	case PTP_PDLY_RSP_MSG_ID:
		event_type = PRUETH_PTP_DLY_RESP;
		break;
	default:
		event_type = PRUETH_PTP_TS_EVENTS;
	}

	return event_type;
}

static void prueth_ptp_tx_ts_reset(struct prueth_emac *emac, u8 event)
{
	void __iomem *sram = emac->prueth->mem[PRUETH_MEM_SHARED_RAM].va;
	u32 ts_notify_offs, ts_offs;

	ts_offs = prueth_tx_ts_offs_get(emac->port_id - 1, event);
	ts_notify_offs = prueth_tx_ts_notify_offs_get(emac->port_id - 1, event);

	writeb(0, sram + ts_notify_offs);
	memset_io(sram + ts_offs, 0, sizeof(u64));
}

static int prueth_ptp_tx_ts_enqueue(struct prueth_emac *emac, struct sk_buff *skb)
{
	struct skb_redundant_info *sred = skb_redinfo(skb);
	u8 event, changed = 0;
	unsigned long flags;

	if (skb_vlan_tagged(skb)) {
		__skb_pull(skb, VLAN_HLEN);
		changed += VLAN_HLEN;
	}

	if (sred && sred->ethertype == ETH_P_HSR) {
		__skb_pull(skb, ICSS_LRE_TAG_RCT_SIZE);
		changed += ICSS_LRE_TAG_RCT_SIZE;
	}

	event = prueth_ptp_ts_event_type(skb);
	__skb_push(skb, changed);
	if (event == PRUETH_PTP_TS_EVENTS) {
		netdev_err(emac->ndev, "invalid PTP event\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&emac->ptp_skb_lock, flags);
	if (emac->ptp_skb[event]) {
		dev_consume_skb_any(emac->ptp_skb[event]);
		prueth_ptp_tx_ts_reset(emac, event);
		netdev_warn(emac->ndev, "Dropped event waiting for tx ts.\n");
	}

	skb_get(skb);
	emac->ptp_skb[event] = skb;
	spin_unlock_irqrestore(&emac->ptp_skb_lock, flags);

	return 0;
}

irqreturn_t prueth_ptp_tx_irq_handle(int irq, void *dev)
{
	struct net_device *ndev = (struct net_device *)dev;
	struct prueth_emac *emac = netdev_priv(ndev);

	if (unlikely(netif_queue_stopped(ndev)))
		netif_wake_queue(ndev);

	if (prueth_ptp_tx_ts_is_enabled(emac))
		return IRQ_WAKE_THREAD;

	return IRQ_HANDLED;
}

static u64 prueth_ptp_ts_get(struct prueth_emac *emac, u32 ts_offs)
{
	void __iomem *sram = emac->prueth->mem[PRUETH_MEM_SHARED_RAM].va;
	u64 cycles;

	memcpy_fromio(&cycles, sram + ts_offs, sizeof(cycles));
	memset_io(sram + ts_offs, 0, sizeof(cycles));

	return cycles;
}

static void prueth_ptp_tx_ts_get(struct prueth_emac *emac, u8 event)
{
	struct skb_shared_hwtstamps *red_ssh;
	struct skb_shared_hwtstamps ssh;
	struct sk_buff *skb;
	unsigned long flags;
	bool ct_ts = false;
	u64 ns;

	/* get the msg from list */
	spin_lock_irqsave(&emac->ptp_skb_lock, flags);
	skb = emac->ptp_skb[event];
	emac->ptp_skb[event] = NULL;
	if (!skb) {
		/* In case of HSR, tx timestamp may be generated by
		 * cut-through packets such as SYNC, pass this ts in skb redinfo.
		 */
		skb = emac->ptp_ct_skb[event];
		emac->ptp_ct_skb[event] = NULL;
		ct_ts = true;
	}
	spin_unlock_irqrestore(&emac->ptp_skb_lock, flags);
	if (!skb) {
		/* TS for cut throguh packet might have already be read by emac_rx_packet()
		 * So ignore this interrupt for HSR.
		 */
		if (!PRUETH_IS_HSR(emac->prueth))
			netdev_err(emac->ndev, "no tx msg %u found waiting for ts\n", event);
		return;
	}

	/* get timestamp */
	ns = prueth_ptp_ts_get(emac,
			       prueth_tx_ts_offs_get(emac->port_id - 1, event));
	if (ct_ts) {
		/* Save the cut-through tx ts in skb redinfo. */
		red_ssh = skb_redinfo_hwtstamps(skb);
		memset(red_ssh, 0, sizeof(*red_ssh));
		red_ssh->hwtstamp = ns_to_ktime(ns);
		skb->protocol = eth_type_trans(skb, emac->ndev);
		netif_receive_skb(skb);
	} else {
		memset(&ssh, 0, sizeof(ssh));
		ssh.hwtstamp = ns_to_ktime(ns);
		skb_tstamp_tx(skb, &ssh);
		dev_consume_skb_any(skb);
	}
}

irqreturn_t prueth_ptp_tx_irq_work(int irq, void *dev)
{
	struct prueth_emac *emac = netdev_priv(dev);
	u32 ts_notify_offs, ts_notify_mask, i;
	void __iomem *sram;

	/* get and reset the ts notifications */
	sram = emac->prueth->mem[PRUETH_MEM_SHARED_RAM].va;
	for (i = 0; i < PRUETH_PTP_TS_EVENTS; i++) {
		ts_notify_offs = prueth_tx_ts_notify_offs_get(emac->port_id - 1,
							      i);
		memcpy_fromio(&ts_notify_mask, sram + ts_notify_offs,
			      PRUETH_PTP_TS_NOTIFY_SIZE);
		memset_io(sram + ts_notify_offs, 0, PRUETH_PTP_TS_NOTIFY_SIZE);

		if (ts_notify_mask & PRUETH_PTP_TS_NOTIFY_MASK)
			prueth_ptp_tx_ts_get(emac, i);
	}

	return IRQ_HANDLED;
}

/**
 * prueth_tx_enqueue - queue a packet to firmware for transmission
 *
 * @emac: EMAC data structure
 * @skb: packet data buffer
 * @queue_id: priority queue id
 */
static int prueth_tx_enqueue(struct prueth_emac *emac, struct sk_buff *skb,
			     enum prueth_queue_id queue_id)
{
	struct net_device *ndev = emac->ndev;
	struct prueth *prueth = emac->prueth;
	int pktlen;
	struct prueth_queue_desc __iomem *queue_desc;
	const struct prueth_queue_info *txqueue;
	u16 bd_rd_ptr, bd_wr_ptr, update_wr_ptr;
	int write_block, read_block, free_blocks, update_block, pkt_block_size;
	unsigned int buffer_desc_count;
	bool buffer_wrapped = false;
	void *src_addr;
	void *dst_addr;

	/* OCMC RAM is not cached and write order is not important */
	void *ocmc_ram = (__force void *)emac->prueth->mem[PRUETH_MEM_OCMC].va;
	void __iomem *sram = prueth->mem[PRUETH_MEM_SHARED_RAM].va;
	void __iomem *dram;
	u32 wr_buf_desc;
	int ret;
	int txport = emac->tx_port_queue; /* which port to tx: MII0 or MII1 */

	if (!PRUETH_IS_EMAC(prueth))
		dram = prueth->mem[PRUETH_MEM_DRAM1].va;
	else
		dram = emac->prueth->mem[emac->dram].va;

	if (eth_skb_pad(skb)) {
		if (netif_msg_tx_err(emac) && net_ratelimit())
			netdev_err(ndev, "packet pad failed");
		return ret;
	}
	src_addr = skb->data;

	pktlen = skb->len;

	/* Get the tx queue */
	queue_desc = emac->tx_queue_descs + queue_id;
	if (!PRUETH_IS_EMAC(prueth))
		txqueue = &sw_queue_infos[txport][queue_id];
	else
		txqueue = &queue_infos[txport][queue_id];
	buffer_desc_count = txqueue->buffer_desc_end -
			    txqueue->buffer_desc_offset;
	buffer_desc_count /= BD_SIZE;
	buffer_desc_count++;

	bd_rd_ptr = readw(&queue_desc->rd_ptr);
	bd_wr_ptr = readw(&queue_desc->wr_ptr);

	/* the PRU firmware deals mostly in pointers already
	 * offset into ram, we would like to deal in indexes
	 * within the queue we are working with for code
	 * simplicity, calculate this here
	 */
	write_block = (bd_wr_ptr - txqueue->buffer_desc_offset) / BD_SIZE;
	read_block = (bd_rd_ptr - txqueue->buffer_desc_offset) / BD_SIZE;
	if (write_block > read_block) {
		free_blocks = buffer_desc_count - write_block;
		free_blocks += read_block;
	} else if (write_block < read_block) {
		free_blocks = read_block - write_block;
	} else { /* they are all free */
		free_blocks = buffer_desc_count;
	}
	pkt_block_size = DIV_ROUND_UP(pktlen, ICSS_BLOCK_SIZE);
	if (pkt_block_size > free_blocks) /* out of queue space */
		return -ENOBUFS;

	/* calculate end BD address post write */
	update_block = write_block + pkt_block_size;

	/* Check for wrap around */
	if (update_block >= buffer_desc_count) {
		update_block %= buffer_desc_count;
		buffer_wrapped = true;
	}

	dst_addr = ocmc_ram + txqueue->buffer_offset +
		   (write_block * ICSS_BLOCK_SIZE);

	/* Copy the data from socket buffer(DRAM) to PRU buffers(OCMC) */
	if (buffer_wrapped) { /* wrapped around buffer */
		int bytes = (buffer_desc_count - write_block) * ICSS_BLOCK_SIZE;
		int remaining;

		/* bytes is integral multiple of ICSS_BLOCK_SIZE but
		 * entire packet may have fit within the last BD
		 * if pkt_info.length is not integral multiple of
		 * ICSS_BLOCK_SIZE
		 */
		if (pktlen < bytes)
			bytes = pktlen;

		/* copy non-wrapped part */
		memcpy(dst_addr, src_addr, bytes);

		/* copy wrapped part */
		src_addr += bytes;
		remaining = pktlen - bytes;
		dst_addr = ocmc_ram + txqueue->buffer_offset;
		memcpy(dst_addr, src_addr, remaining);
	} else {
		memcpy(dst_addr, src_addr, pktlen);
	}

	if (skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP &&
	    prueth_ptp_tx_ts_is_enabled(emac)) {
		skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
		prueth_ptp_tx_ts_enqueue(emac, skb);
	}

	/* update first buffer descriptor */
	wr_buf_desc = (pktlen << PRUETH_BD_LENGTH_SHIFT) & PRUETH_BD_LENGTH_MASK;
	if (PRUETH_IS_HSR(prueth))
		wr_buf_desc |= BIT(PRUETH_BD_HSR_FRAME_SHIFT);

	if (!PRUETH_IS_EMAC(prueth))
		writel(wr_buf_desc, sram + bd_wr_ptr);
	else
		writel(wr_buf_desc, dram + bd_wr_ptr);

	/* update the write pointer in this queue descriptor, the firmware
	 * polls for this change so this will signal the start of transmission
	 */
	update_wr_ptr = txqueue->buffer_desc_offset + (update_block * BD_SIZE);
	writew(update_wr_ptr, &queue_desc->wr_ptr);

	return 0;
}

void parse_packet_info(struct prueth *prueth, u32 buffer_descriptor,
		       struct prueth_packet_info *pkt_info)
{
	/* For HSR, start_offset indicates Tag is not present and actual
	 * data starts at an offset of 6 bytes from start of the buffer.
	 * For example, for Supervisor frame start_offset is set, but for
	 * data frame it is reset. For PRP, start_offset indicate if RCT
	 * is present in the data or not. i.e in this case, depending upon
	 * LRE_TRANSPARENT_RECEPTION state RCT is to be stripped or not
	 * before passing data to upper layer. Software adjust the skb->len
	 * accordingly. TODO Support for LRE_TRANSPARENT_RECEPTION set to
	 * passRCT is TBD.
	 */
	if (PRUETH_IS_LRE(prueth))
		pkt_info->start_offset = !!(buffer_descriptor &
					    PRUETH_BD_START_FLAG_MASK);
	else
		pkt_info->start_offset = false;

	pkt_info->shadow = !!(buffer_descriptor & PRUETH_BD_SHADOW_MASK);
	pkt_info->port = (buffer_descriptor & PRUETH_BD_PORT_MASK) >>
			 PRUETH_BD_PORT_SHIFT;
	pkt_info->length = (buffer_descriptor & PRUETH_BD_LENGTH_MASK) >>
			   PRUETH_BD_LENGTH_SHIFT;
	pkt_info->broadcast = !!(buffer_descriptor & PRUETH_BD_BROADCAST_MASK);
	pkt_info->error = !!(buffer_descriptor & PRUETH_BD_ERROR_MASK);
	if (PRUETH_IS_LRE(prueth))
		pkt_info->sv_frame = !!(buffer_descriptor &
					PRUETH_BD_SUP_HSR_FRAME_MASK);
	else
		pkt_info->sv_frame = false;
	pkt_info->lookup_success = !!(buffer_descriptor &
				      PRUETH_BD_LOOKUP_SUCCESS_MASK);
	pkt_info->flood = !!(buffer_descriptor & PRUETH_BD_SW_FLOOD_MASK);
	pkt_info->timestamp = !!(buffer_descriptor & PRUETH_BD_TIMESTAMP_MASK);
}

static int prueth_hsr_ptp_ct_tx_ts_enqueue(struct prueth_emac *emac, struct sk_buff *skb, u16 type)
{
	struct prueth_emac *other_emac = emac->prueth->emac[other_port_id(emac->port_id) - 1];
	struct skb_shared_hwtstamps *red_ssh;
	unsigned long flags;
	u8 ptp_type, event;
	int changed = 0;
	u64 ns;

	if (type == ETH_P_8021Q) {
		__skb_pull(skb, VLAN_HLEN);
		changed += VLAN_HLEN;
	}

	__skb_pull(skb, ICSS_LRE_TAG_RCT_SIZE);
	changed += ICSS_LRE_TAG_RCT_SIZE;

	ptp_type = ptp_event_type(skb);
	event = prueth_ptp_ts_event_type(skb);

	__skb_push(skb, changed);

	/* Store skbs for only cut through packets */
	if (ptp_type != PTP_SYNC_MSG_ID && ptp_type != PTP_DLY_REQ_MSG_ID)
		return 0;

	/* cut through packet might have already be forwarded before the rx packet has reached
	 * the host. In this case tx irq handler ignores the interrupt as there is no skb stored.
	 * So check if ts is already available before storing the skb.
	 */
	ns = prueth_ptp_ts_get(other_emac, prueth_tx_ts_offs_get(other_emac->port_id - 1, event));
	if (ns || !other_emac->link) {
		/* Save the cut-through tx ts in skb redinfo. */
		red_ssh = skb_redinfo_hwtstamps(skb);
		memset(red_ssh, 0, sizeof(*red_ssh));
		red_ssh->hwtstamp = ns_to_ktime(ns);

		return 0;
	}

	/* Store the skb so that tx irq handler will populate the ts */
	spin_lock_irqsave(&other_emac->ptp_skb_lock, flags);
	if (other_emac->ptp_ct_skb[event]) {
		dev_consume_skb_any(other_emac->ptp_skb[event]);
		prueth_ptp_tx_ts_reset(other_emac, event);
		netdev_warn(other_emac->ndev, "Dropped cut through event waiting for tx ts.\n");
	}

	skb_get(skb);
	other_emac->ptp_ct_skb[event] = skb;
	spin_unlock_irqrestore(&other_emac->ptp_skb_lock, flags);

	return -EAGAIN;
}

/* get packet from queue
 * negative for error
 */
int emac_rx_packet(struct prueth_emac *emac, u16 *bd_rd_ptr,
		   struct prueth_packet_info pkt_info,
		   const struct prueth_queue_info *rxqueue)
{
	struct net_device *ndev = emac->ndev;
	struct prueth *prueth = emac->prueth;
	const struct prueth_private_data *fw_data = prueth->fw_data;
	int read_block, update_block, pkt_block_size;
	bool buffer_wrapped = false, prp_rct = false;
	unsigned int buffer_desc_count;
	struct sk_buff *skb;
	void *src_addr;
	void *dst_addr;
	void *nt_dst_addr;
	u8 macid[6];
	/* OCMC RAM is not cached and read order is not important */
	void *ocmc_ram = (__force void *)emac->prueth->mem[PRUETH_MEM_OCMC].va;
	struct skb_shared_hwtstamps *ssh;
	unsigned int actual_pkt_len;
	u16 start_offset = 0, type;
	u8 offset = 0, *ptr;
	u64 ts;
	int ret;

	if (PRUETH_IS_HSR(prueth))
		start_offset = (pkt_info.start_offset ?
				ICSS_LRE_TAG_RCT_SIZE : 0);
	else if (PRUETH_IS_PRP(prueth) && pkt_info.start_offset)
		prp_rct = true;

	/* the PRU firmware deals mostly in pointers already
	 * offset into ram, we would like to deal in indexes
	 * within the queue we are working with for code
	 * simplicity, calculate this here
	 */
	buffer_desc_count = rxqueue->buffer_desc_end -
			    rxqueue->buffer_desc_offset;
	buffer_desc_count /= BD_SIZE;
	buffer_desc_count++;
	read_block = (*bd_rd_ptr - rxqueue->buffer_desc_offset) / BD_SIZE;
	pkt_block_size = DIV_ROUND_UP(pkt_info.length, ICSS_BLOCK_SIZE);

	/* calculate end BD address post read */
	update_block = read_block + pkt_block_size;

	/* Check for wrap around */
	if (update_block >= buffer_desc_count) {
		update_block %= buffer_desc_count;
		buffer_wrapped = true;
	}

	/* calculate new pointer in ram */
	*bd_rd_ptr = rxqueue->buffer_desc_offset + (update_block * BD_SIZE);

	/* Pkt len w/ HSR tag removed, If applicable */
	actual_pkt_len = pkt_info.length - start_offset;

	/* Allocate a socket buffer for this packet */
	skb = netdev_alloc_skb_ip_align(ndev, actual_pkt_len);
	if (!skb) {
		if (netif_msg_rx_err(emac) && net_ratelimit())
			netdev_err(ndev, "failed rx buffer alloc\n");
		return -ENOMEM;
	}
	dst_addr = skb->data;
	nt_dst_addr = dst_addr;

	/* Get the start address of the first buffer from
	 * the read buffer description
	 */
	if (pkt_info.shadow) {
		src_addr = ocmc_ram + P0_COL_BUFFER_OFFSET;
	} else {
		src_addr = ocmc_ram + rxqueue->buffer_offset +
			   (read_block * ICSS_BLOCK_SIZE);
	}
	src_addr += start_offset;

	/* Copy the data from PRU buffers(OCMC) to socket buffer(DRAM) */
	if (buffer_wrapped) { /* wrapped around buffer */
		int bytes = (buffer_desc_count - read_block) * ICSS_BLOCK_SIZE;
		int remaining;

		/* bytes is integral multiple of ICSS_BLOCK_SIZE but
		 * entire packet may have fit within the last BD
		 * if pkt_info.length is not integral multiple of
		 * ICSS_BLOCK_SIZE
		 */
		if (pkt_info.length < bytes)
			bytes = pkt_info.length;

		/* If applicable, account for the HSR tag removed */
		bytes -= start_offset;

		/* copy non-wrapped part */
		memcpy(dst_addr, src_addr, bytes);

		/* copy wrapped part */
		dst_addr += bytes;
		remaining = actual_pkt_len - bytes;
		if (pkt_info.shadow)
			src_addr += bytes;
		else
			src_addr = ocmc_ram + rxqueue->buffer_offset;
		memcpy(dst_addr, src_addr, remaining);
		src_addr += remaining;
	} else {
		memcpy(dst_addr, src_addr, actual_pkt_len);
		src_addr += actual_pkt_len;
	}

	if (pkt_info.timestamp) {
		src_addr = (void *)roundup((uintptr_t)src_addr, ICSS_BLOCK_SIZE);
		dst_addr = &ts;
		memcpy(dst_addr, src_addr, sizeof(ts));
	}

	/* Check if VLAN tag is present since SV payload location will change
	 * based on that
	 */
	if (PRUETH_IS_LRE(prueth)) {
		ptr = nt_dst_addr + PRUETH_ETH_TYPE_OFFSET;
		type = (*ptr++) << PRUETH_ETH_TYPE_UPPER_SHIFT;
		type |= *ptr++;
		if (type == ETH_P_8021Q)
			offset = 4;
	}

	/* TODO. The check for FW_REV_V1_0 is a workaround since
	 * lookup of MAC address in Node table by this version of firmware
	 * is not reliable. Once this issue is fixed in firmware, this driver
	 * check has to be removed.
	 */
	if (PRUETH_IS_LRE(prueth) &&
	    (!pkt_info.lookup_success || !fw_data->rev_2_1)) {
		if (PRUETH_IS_PRP(prueth)) {
			memcpy(macid,
			       ((pkt_info.sv_frame) ?
				nt_dst_addr + LRE_SV_FRAME_OFFSET + offset :
				nt_dst_addr + ICSS_LRE_TAG_RCT_SIZE),
				ICSS_LRE_TAG_RCT_SIZE);

			prueth_lre_nt_insert(prueth, macid, emac->port_id,
					     pkt_info.sv_frame,
					     LRE_PROTO_PRP);

		} else if (pkt_info.sv_frame) {
			memcpy(macid,
			       nt_dst_addr + LRE_SV_FRAME_OFFSET + offset,
			       ICSS_LRE_TAG_RCT_SIZE);
			prueth_lre_nt_insert(prueth, macid, emac->port_id,
					     pkt_info.sv_frame,
					     LRE_PROTO_HSR);
		}
	}

	/* For PRP, firmware always send us RCT. So skip Tag if
	 * prp_tr_mode is IEC62439_3_TR_REMOVE_RCT
	 */
	if (prp_rct && prueth->prp_tr_mode == IEC62439_3_TR_REMOVE_RCT)
		actual_pkt_len -= ICSS_LRE_TAG_RCT_SIZE;

	if (!pkt_info.sv_frame) {
		skb_put(skb, actual_pkt_len);

		if (PRUETH_IS_SWITCH(emac->prueth)) {
			skb->offload_fwd_mark = emac->offload_fwd_mark;
			if (!pkt_info.lookup_success)
				prueth_sw_learn_fdb(emac, skb->data + ETH_ALEN);
		}

		if (prueth_ptp_rx_ts_is_enabled(emac) && pkt_info.timestamp) {
			ssh = skb_hwtstamps(skb);
			memset(ssh, 0, sizeof(*ssh));
			ssh->hwtstamp = ns_to_ktime(ts);
			if (PRUETH_IS_HSR(prueth)) {
				ret = prueth_hsr_ptp_ct_tx_ts_enqueue(emac, skb, type);
				if (ret == -EAGAIN)
					goto out;
			}
		}

		/* send packet up the stack */
		skb->protocol = eth_type_trans(skb, ndev);
		netif_receive_skb(skb);
	} else {
		dev_kfree_skb_any(skb);
	}
out:

	/* update stats */
	ndev->stats.rx_bytes += actual_pkt_len;
	ndev->stats.rx_packets++;

	return 0;
}

/* get upto quota number of packets */
static int emac_rx_packets(struct prueth_emac *emac, int quota)
{
	struct prueth *prueth = emac->prueth;
	int start_queue, end_queue;
	struct prueth_queue_desc __iomem *queue_desc;
	const struct prueth_queue_info *rxqueue;
	u8 overflow_cnt;
	u16 bd_rd_ptr, bd_wr_ptr, update_rd_ptr;
	u32 rd_buf_desc;
	void __iomem *shared_ram = emac->prueth->mem[PRUETH_MEM_SHARED_RAM].va;
	struct prueth_packet_info pkt_info;
	struct net_device_stats *ndevstats = &emac->ndev->stats;
	int i, ret, used = 0;
	struct prueth_emac *other_emac;

	other_emac = prueth->emac[other_port_id(emac->port_id) - 1];

	if (PRUETH_IS_SWITCH(prueth)) {
		start_queue = PRUETH_QUEUE1;
		end_queue = PRUETH_QUEUE4;
	} else {
		start_queue = emac->rx_queue_start;
		end_queue = emac->rx_queue_end;
	}

	/* search host queues for packets */
	for (i = start_queue; i <= end_queue; i++) {
		queue_desc = emac->rx_queue_descs + i;
		if (PRUETH_IS_SWITCH(emac->prueth))
			rxqueue = &sw_queue_infos[PRUETH_PORT_HOST][i];
		else
			rxqueue = &queue_infos[PRUETH_PORT_HOST][i];

		overflow_cnt = readb(&queue_desc->overflow_cnt);
		if (overflow_cnt > 0) {
			emac->ndev->stats.rx_over_errors += overflow_cnt;

			/* reset to zero */
			writeb(0, &queue_desc->overflow_cnt);
		}

		bd_rd_ptr = readw(&queue_desc->rd_ptr);
		bd_wr_ptr = readw(&queue_desc->wr_ptr);

		/* while packets are available in this queue */
		while (bd_rd_ptr != bd_wr_ptr) {
			/* get packet info from the read buffer descriptor */
			rd_buf_desc = readl(shared_ram + bd_rd_ptr);
			parse_packet_info(prueth, rd_buf_desc, &pkt_info);

			if (pkt_info.length <= 0) {
				/* a packet length of zero will cause us to
				 * never move the read pointer ahead, locking
				 * the driver, so we manually have to move it
				 * to the write pointer, discarding all
				 * remaining packets in this queue. This should
				 * never happen.
				 */
				update_rd_ptr = bd_wr_ptr;
				ndevstats->rx_length_errors++;
			} else if (pkt_info.length > EMAC_MAX_PKTLEN) {
				/* if the packet is too large we skip it but we
				 * still need to move the read pointer ahead
				 * and assume something is wrong with the read
				 * pointer as the firmware should be filtering
				 * these packets
				 */
				update_rd_ptr = bd_wr_ptr;
				ndevstats->rx_length_errors++;
			} else {
				update_rd_ptr = bd_rd_ptr;
				if (PRUETH_IS_SWITCH(emac->prueth)) {
					if (pkt_info.port ==
						other_emac->port_id) {
						emac = other_emac;
					}
				}

				ret = emac_rx_packet(emac, &update_rd_ptr,
						     pkt_info, rxqueue);
				if (ret)
					return ret;

				used++;
			}

			/* after reading the buffer descriptor we clear it
			 * to prevent improperly moved read pointer errors
			 * from simply looking like old packets.
			 */
			writel(0, shared_ram + bd_rd_ptr);

			/* update read pointer in queue descriptor */
			writew(update_rd_ptr, &queue_desc->rd_ptr);
			bd_rd_ptr = update_rd_ptr;

			/* all we have room for? */
			if (used >= quota)
				return used;
		}
	}

	return used;
}

/* get statistics maintained by the PRU firmware into @pstats */
static void emac_get_stats(struct prueth_emac *emac,
			   struct port_statistics *pstats)
{
	void __iomem *dram;

	dram = emac->prueth->mem[emac->dram].va;
	memcpy_fromio(pstats, dram + STATISTICS_OFFSET, STAT_SIZE);

	pstats->vlan_dropped =
		readl(dram + ICSS_EMAC_FW_VLAN_FILTER_DROP_CNT_OFFSET);
	pstats->multicast_dropped =
		readl(dram + ICSS_EMAC_FW_MULTICAST_FILTER_DROP_CNT_OFFSET);
}

/* set PRU firmware statistics */
static void emac_set_stats(struct prueth_emac *emac,
			   struct port_statistics *pstats)
{
	void __iomem *dram;

	dram = emac->prueth->mem[emac->dram].va;
	memcpy_toio(dram + STATISTICS_OFFSET, pstats, STAT_SIZE);

	writel(pstats->vlan_dropped, dram +
			ICSS_EMAC_FW_VLAN_FILTER_DROP_CNT_OFFSET);
	writel(pstats->multicast_dropped, dram +
			ICSS_EMAC_FW_MULTICAST_FILTER_DROP_CNT_OFFSET);
}

/**
 * emac_napi_poll - EMAC NAPI Poll function
 * @napi: ptr to napi instance associated with the emac
 * @budget: Number of receive packets to process (as told by NAPI layer)
 *
 * NAPI Poll function implemented to process packets as per budget. We check
 * the type of interrupt on the device and accordingly call the TX or RX
 * packet processing functions. We follow the budget for RX processing and
 * also put a cap on number of TX pkts processed through config param. The
 * NAPI schedule function is called if more packets pending.
 *
 * Returns number of packets received (in most cases; else TX pkts - rarely)
 */
static int emac_napi_poll(struct napi_struct *napi, int budget)
{
	struct prueth_emac *emac = container_of(napi, struct prueth_emac, napi);
	int num_rx_packets;

	num_rx_packets = emac_rx_packets(emac, budget);
	if (num_rx_packets < budget)
		emac_finish_napi(emac, napi, emac->rx_irq);

	return num_rx_packets;
}

static int emac_set_boot_pru(struct prueth_emac *emac, struct net_device *ndev)
{
	const struct prueth_firmware *pru_firmwares;
	struct prueth *prueth = emac->prueth;
	const char *fw_name;
	int ret = 0;

	pru_firmwares = &prueth->fw_data->fw_pru[emac->port_id - 1];
	fw_name = pru_firmwares->fw_name[prueth->eth_type];
	if (!fw_name) {
		netdev_err(ndev, "eth_type %d not supported\n",
			   prueth->eth_type);
		return -ENODEV;
	}

	ret = rproc_set_firmware(emac->pru, fw_name);
	if (ret) {
		netdev_err(ndev, "failed to set PRU0 firmware %s: %d\n",
			   fw_name, ret);
		return ret;
	}

	ret = rproc_boot(emac->pru);
	if (ret) {
		netdev_err(ndev, "failed to boot PRU0: %d\n", ret);
		return ret;
	}

	return ret;
}

static int emac_request_irqs(struct prueth_emac *emac)
{
	struct net_device *ndev = emac->ndev;
	int ret = 0;

	ret = request_irq(emac->rx_irq, emac_rx_hardirq,
			  IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
			  ndev->name, ndev);
	if (ret) {
		netdev_err(ndev, "unable to request RX IRQ\n");
		return ret;
	}

	if (PRUETH_IS_EMAC(emac->prueth) && emac->tx_irq > 0) {
		ret = request_irq(emac->tx_irq, emac_tx_hardirq,
				  IRQF_TRIGGER_HIGH, ndev->name, ndev);
		if (ret) {
			netdev_err(ndev, "unable to request TX IRQ\n");
			free_irq(emac->rx_irq, ndev);
			return ret;
		}
	}

	if (emac->emac_ptp_tx_irq) {
		ret = request_threaded_irq(emac->emac_ptp_tx_irq,
					   prueth_ptp_tx_irq_handle,
					   prueth_ptp_tx_irq_work,
					   IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
					   ndev->name, ndev);
		if (ret) {
			netdev_err(ndev, "unable to request PTP TX IRQ\n");
			free_irq(emac->rx_irq, ndev);
			free_irq(emac->tx_irq, ndev);
		}

	}

	return ret;
}

static int emac_sanitize_feature_flags(struct prueth_emac *emac)
{
	if (PRUETH_IS_HSR(emac->prueth) &&
	    !(emac->ndev->features & NETIF_F_HW_HSR_RX_OFFLOAD)) {
		netdev_err(emac->ndev, "Error: Turn ON HSR offload\n");
		return -EINVAL;
	}

	if (PRUETH_IS_PRP(emac->prueth) &&
	    !(emac->ndev->features & NETIF_F_HW_PRP_RX_OFFLOAD)) {
		netdev_err(emac->ndev, "Error: Turn ON PRP offload\n");
		return -EINVAL;
	}

	if ((PRUETH_IS_EMAC(emac->prueth) || PRUETH_IS_SWITCH(emac->prueth)) &&
	    (emac->ndev->features & (NETIF_F_HW_PRP_RX_OFFLOAD |
	     NETIF_F_HW_HSR_RX_OFFLOAD))) {
		netdev_err(emac->ndev, "Error: Turn OFF %s offload\n",
			   (emac->ndev->features &
			   NETIF_F_HW_HSR_RX_OFFLOAD) ? "HSR" : "PRP");
		return -EINVAL;
	}

	return 0;
}

/* Function to free memory related to sw/lre */
static void prueth_free_memory(struct prueth *prueth)
{
	if (PRUETH_IS_SWITCH(prueth))
		prueth_sw_free_fdb_table(prueth);
	if (PRUETH_IS_LRE(prueth))
		prueth_lre_free_memory(prueth);
}

/**
 * emac_ndo_open - EMAC device open
 * @ndev: network adapter device
 *
 * Called when system wants to start the interface.
 *
 * Returns 0 for a successful open, or appropriate error code
 */
static int emac_ndo_open(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth *prueth = emac->prueth;
	struct prueth_ecap *ecap = prueth->ecap;
	int ret;

	/* set h/w MAC as user might have re-configured */
	ether_addr_copy(emac->mac_addr, ndev->dev_addr);

	netif_carrier_off(ndev);

	mutex_lock(&prueth->mlock);
	if (!prueth->emac_configured)
		prueth_init_ethernet_mode(prueth);

	ret = emac_sanitize_feature_flags(emac);
	if (ret)
		goto unlock_mutex;

	/* reset and start PRU firmware */
	if (!PRUETH_IS_EMAC(prueth)) {
		ret = prueth_sw_emac_config(emac);
		if (ret)
			goto unlock_mutex;

		if (PRUETH_IS_SWITCH(prueth)) {
			ret = prueth_sw_init_fdb_table(prueth);
		} else {
			/* HSR/PRP */
			prueth_lre_config_check_flags(prueth);
			ret = prueth_lre_init_node_table(prueth);
		}

	} else {
		prueth_emac_config(emac);
	}

	if (ret)
		goto unlock_mutex;

	/* restore stats */
	emac_set_stats(emac, &emac->stats);
	if (PRUETH_IS_LRE(prueth))
		prueth_lre_set_stats(prueth, prueth->lre_stats);

	if (!prueth->emac_configured) {
		ret = icss_iep_init(prueth->iep, NULL, NULL, 0);
		if (ret) {
			netdev_err(ndev, "Failed to initialize iep: %d\n", ret);
			goto free_mem;
		}
	}

	/* initialize ecap for interrupt pacing */
	if (!IS_ERR(ecap))
		ecap->init(emac);

	if (!PRUETH_IS_EMAC(prueth)) {
		ret = prueth_sw_boot_prus(prueth, ndev);
		if (ret)
			goto iep_exit;
	} else {
		/* boot the PRU */
		ret = emac_set_boot_pru(emac, ndev);
		if (ret) {
			netdev_err(ndev, "failed to boot PRU: %d\n", ret);
			goto iep_exit;
		}
	}

	if (PRUETH_IS_EMAC(prueth) || PRUETH_IS_SWITCH(prueth))
		ret = emac_request_irqs(emac);
	else
		ret = prueth_lre_request_irqs(emac);
	if (ret)
		goto rproc_shutdown;

	if (PRUETH_IS_EMAC(prueth) || PRUETH_IS_SWITCH(prueth)) {
		napi_enable(&emac->napi);
	} else {
		/* HSR/PRP. Enable NAPI when first port is initialized */
		if (!prueth->emac_configured) {
			napi_enable(&prueth->napi_hpq);
			napi_enable(&prueth->napi_lpq);
		}
	}

	/* start PHY */
	phy_start(emac->phydev);

	/* enable the port and vlan */
	prueth_port_enable(emac, true);

	/* timer used for NSP as well for HSR/PRP */
	if (emac->nsp_enabled || PRUETH_IS_LRE(prueth))
		prueth_start_timer(emac);

	prueth->emac_configured |= BIT(emac->port_id);
	mutex_unlock(&prueth->mlock);

	if (PRUETH_IS_SWITCH(prueth))
		prueth_sw_port_set_stp_state(prueth, emac->port_id,
					     BR_STATE_LEARNING);
	if (netif_msg_drv(emac))
		dev_notice(&ndev->dev, "started\n");

	return 0;

rproc_shutdown:
	if (!PRUETH_IS_EMAC(prueth))
		prueth_sw_shutdown_prus(emac, ndev);
	else
		rproc_shutdown(emac->pru);
iep_exit:
	if (!prueth->emac_configured)
		icss_iep_exit(prueth->iep);
free_mem:
	prueth_free_memory(emac->prueth);
unlock_mutex:
	mutex_unlock(&prueth->mlock);
	return ret;
}

/**
 * emac_ndo_stop - EMAC device stop
 * @ndev: network adapter device
 *
 * Called when system wants to stop or down the interface.
 */
static int emac_ndo_stop(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth *prueth = emac->prueth;
	int i;

	mutex_lock(&prueth->mlock);
	prueth->emac_configured &= ~BIT(emac->port_id);

	/* disable the mac port */
	prueth_port_enable(emac, false);

	/* stop PHY */
	phy_stop(emac->phydev);

	/* inform the upper layers. */
	netif_stop_queue(ndev);
	if (PRUETH_IS_EMAC(prueth) || PRUETH_IS_SWITCH(prueth)) {
		napi_disable(&emac->napi);
	} else {
		/* HSR/PRP. Disable NAPI when last port is down */
		if (!prueth->emac_configured) {
			hrtimer_cancel(&prueth->tbl_check_timer);
			kthread_cancel_work_sync(&prueth->nt_work);
			kthread_destroy_worker(prueth->nt_kworker);
			napi_disable(&prueth->napi_lpq);
			napi_disable(&prueth->napi_hpq);
		}
	}

	netif_carrier_off(ndev);

	/* stop the PRU */
	if (!PRUETH_IS_EMAC(prueth))
		prueth_sw_shutdown_prus(emac, ndev);
	else
		rproc_shutdown(emac->pru);

	/* save and lre stats */
	emac_get_stats(emac, &emac->stats);
	if (PRUETH_IS_LRE(prueth) && !prueth->emac_configured)
		prueth_lre_get_stats(prueth, prueth->lre_stats);

	if (!prueth->emac_configured)
		icss_iep_exit(prueth->iep);

	/* Cleanup ptp related stuff for all protocols */
	prueth_ptp_tx_ts_enable(emac, 0);
	prueth_ptp_rx_ts_enable(emac, 0);
	for (i = 0; i < PRUETH_PTP_TS_EVENTS; i++) {
		if (emac->ptp_skb[i]) {
			prueth_ptp_tx_ts_reset(emac, i);
			dev_consume_skb_any(emac->ptp_skb[i]);
			emac->ptp_skb[i] = NULL;
		}
		if (emac->ptp_ct_skb[i]) {
			prueth_ptp_tx_ts_reset(emac, i);
			dev_consume_skb_any(emac->ptp_ct_skb[i]);
			emac->ptp_ct_skb[i] = NULL;
		}
	}

	/* free rx and tx interrupts */
	if (PRUETH_IS_EMAC(emac->prueth) && emac->tx_irq > 0)
		free_irq(emac->tx_irq, ndev);
	/* For EMAC and Switch, interrupt is per port.
	 * So free interrupts same way
	 */
	if (PRUETH_IS_EMAC(emac->prueth) || PRUETH_IS_SWITCH(prueth)) {
		free_irq(emac->rx_irq, ndev);
		if (emac->emac_ptp_tx_irq)
			free_irq(emac->emac_ptp_tx_irq, ndev);
	} else {
		if (emac->hsr_ptp_tx_irq)
			free_irq(emac->hsr_ptp_tx_irq, emac->ndev);

		/* Free interrupts on last port */
		prueth_lre_free_irqs(emac);
	}

	/* free memory related to sw/lre */
	prueth_free_memory(emac->prueth);

	mutex_unlock(&prueth->mlock);

	if (netif_msg_drv(emac))
		dev_notice(&ndev->dev, "stopped\n");

	return 0;
}

static void prueth_change_to_switch_mode(struct prueth *prueth)
{
	bool portstatus[PRUETH_NUM_MACS];
	struct prueth_emac *emac;
	struct net_device *ndev;
	int i, ret;

	for (i = 0; i < PRUETH_NUM_MACS; i++) {
		emac = prueth->emac[i];
		ndev = emac->ndev;

		portstatus[i] = netif_running(ndev);
		if (!portstatus[i])
			continue;

		ret = ndev->netdev_ops->ndo_stop(ndev);
		if (ret < 0) {
			netdev_err(ndev, "failed to stop: %d", ret);
			return;
		}
	}

	prueth->eth_type = PRUSS_ETHTYPE_SWITCH;

	for (i = 0; i < PRUETH_NUM_MACS; i++) {
		emac = prueth->emac[i];
		ndev = emac->ndev;

		if (!portstatus[i])
			continue;

		ret = ndev->netdev_ops->ndo_open(ndev);
		if (ret < 0) {
			netdev_err(ndev, "failed to start: %d", ret);
			return;
		}
	}

	dev_info(prueth->dev, "TI PRU ethernet now in Switch mode\n");
}

static void prueth_change_to_emac_mode(struct prueth *prueth)
{
	struct prueth_emac *emac;
	struct net_device *ndev;
	bool portstatus[PRUETH_NUM_MACS];
	int i, ret;

	for (i = 0; i < PRUETH_NUM_MACS; i++) {
		emac = prueth->emac[i];
		ndev = emac->ndev;

		portstatus[i] = netif_running(ndev);
		if (!portstatus[i])
			continue;

		ret = ndev->netdev_ops->ndo_stop(ndev);
		if (ret < 0) {
			netdev_err(ndev, "failed to stop: %d", ret);
			return;
		}
	}

	prueth->eth_type = PRUSS_ETHTYPE_EMAC;

	for (i = 0; i < PRUETH_NUM_MACS; i++) {
		emac = prueth->emac[i];
		ndev = emac->ndev;

		if (!portstatus[i])
			continue;

		ret = ndev->netdev_ops->ndo_open(ndev);
		if (ret < 0) {
			netdev_err(ndev, "failed to start: %d", ret);
			return;
		}
	}

	dev_info(prueth->dev, "TI PRU ethernet now in Dual EMAC mode\n");
}

/* VLAN-tag PCP to priority queue map for EMAC used by driver. Should be
 * in sync with fw_pcp_default_priority_queue_map[]
 * Index is PCP val.
 *   low  - pcp 0..1 maps to Q4
 *              2..3 maps to Q3
 *              4..5 maps to Q2
 *   high - pcp 6..7 maps to Q1.
 *
 * VLAN-tag PCP to priority queue map for Switch/HSR/PRP used by driver
 * Index is PCP val / 2.
 *   low  - pcp 0..3 maps to Q4 for Host
 *   high - pcp 4..7 maps to Q3 for Host
 *   low  - pcp 0..3 maps to Q2 for PRU-x where x = 1 for PRUETH_PORT_MII0
 *          0 for PRUETH_PORT_MII1
 *   high - pcp 4..7 maps to Q1 for PRU-x
 */
static const unsigned short emac_pcp_tx_priority_queue_map[] = {
	PRUETH_QUEUE4, PRUETH_QUEUE4,
	PRUETH_QUEUE3, PRUETH_QUEUE3,
	PRUETH_QUEUE2, PRUETH_QUEUE2,
	PRUETH_QUEUE1, PRUETH_QUEUE1,
};

static u16 prueth_get_tx_queue_id(struct prueth *prueth, struct sk_buff *skb)
{
	u16 vlan_tci, pcp;
	int err;

	err = vlan_get_tag(skb, &vlan_tci);
	if (likely(err))
		pcp = 0;
	else
		pcp = (vlan_tci & VLAN_PRIO_MASK) >> VLAN_PRIO_SHIFT;
	/* For HSR/PRP, we use only QUEUE4 and QUEUE3 at the egress. QUEUE2 and
	 * QUEUE1 are used for port to port traffic. Current version of SWITCH
	 * firmware uses 4 egress queues.
	 */
	if (PRUETH_IS_LRE(prueth))
		pcp >>= 1;

	return emac_pcp_tx_priority_queue_map[pcp];
}

/**
 * emac_ndo_start_xmit - EMAC Transmit function
 * @skb: SKB pointer
 * @ndev: EMAC network adapter
 *
 * Called by the system to transmit a packet  - we queue the packet in
 * EMAC hardware transmit queue
 *
 * Returns success(NETDEV_TX_OK) or error code (typically out of desc's)
 */
static int emac_ndo_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	int ret = 0;
	u16 qid;

	if (unlikely(!emac->link)) {
		if (netif_msg_tx_err(emac) && net_ratelimit())
			netdev_err(ndev, "No link to transmit");
		goto fail_tx;
	}

	qid = prueth_get_tx_queue_id(emac->prueth, skb);
	ret = prueth_tx_enqueue(emac, skb, qid);
	if (ret) {
		if (ret != -ENOBUFS && netif_msg_tx_err(emac) &&
		    net_ratelimit())
			netdev_err(ndev, "packet queue failed: %d\n", ret);
		goto fail_tx;
	}

	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += skb->len;
	dev_kfree_skb_any(skb);

	return NETDEV_TX_OK;

fail_tx:
	if (ret == -ENOBUFS) {
		/* no free TX queue */
		if (emac->tx_irq > 0)
			netif_stop_queue(ndev);
		ret = NETDEV_TX_BUSY;
	} else {
		/* error */
		ndev->stats.tx_dropped++;
		ret = NET_XMIT_DROP;
	}

	return ret;
}

/**
 * emac_ndo_tx_timeout - EMAC Transmit timeout function
 * @ndev: The EMAC network adapter
 *
 * Called when system detects that a skb timeout period has expired
 * potentially due to a fault in the adapter in not being able to send
 * it out on the wire.
 */
static void emac_ndo_tx_timeout(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);

	if (netif_msg_tx_err(emac))
		netdev_err(ndev, "xmit timeout");

	ndev->stats.tx_errors++;

	/* TODO: can we recover or need to reboot firmware? */

	netif_wake_queue(ndev);
}

/**
 * emac_ndo_getstats - EMAC get statistics function
 * @ndev: The EMAC network adapter
 *
 * Called when system wants to get statistics from the device.
 *
 * We return the statistics in net_device_stats structure pulled from emac
 */
static struct net_device_stats *emac_ndo_get_stats(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct port_statistics pstats;
	struct net_device_stats *stats = &ndev->stats;

	emac_get_stats(emac, &pstats);
	stats->collisions = pstats.late_coll + pstats.single_coll +
			    pstats.multi_coll + pstats.excess_coll;
	stats->multicast = pstats.rx_mcast;

	return stats;
}

/* enable/disable MC filter */
static void emac_mc_filter_ctrl(struct prueth_emac *emac, bool enable)
{
	struct prueth *prueth = emac->prueth;
	void __iomem *ram = prueth->mem[emac->dram].va;
	u32 mc_ctrl_byte = prueth->fw_offsets->mc_ctrl_byte;
	void __iomem *mc_filter_ctrl;
	u32 reg;

	if (PRUETH_IS_LRE(prueth))
		ram = prueth->mem[PRUETH_MEM_DRAM1].va;

	mc_filter_ctrl = ram + mc_ctrl_byte;

	if (enable)
		reg = ICSS_EMAC_FW_MULTICAST_FILTER_CTRL_ENABLED;
	else
		reg = ICSS_EMAC_FW_MULTICAST_FILTER_CTRL_DISABLED;

	writeb(reg, mc_filter_ctrl);
}

/* reset MC filter bins */
static void emac_mc_filter_reset(struct prueth_emac *emac)
{
	struct prueth *prueth = emac->prueth;
	void __iomem *ram = prueth->mem[emac->dram].va;
	u32 mc_filter_tbl_base = prueth->fw_offsets->mc_filter_tbl;
	void __iomem *mc_filter_tbl;

	if (PRUETH_IS_LRE(prueth))
		ram = prueth->mem[PRUETH_MEM_DRAM1].va;

	mc_filter_tbl = ram + mc_filter_tbl_base;
	memset_io(mc_filter_tbl, 0, ICSS_EMAC_FW_MULTICAST_TABLE_SIZE_BYTES);
}

/* set MC filter hashmask */
static void emac_mc_filter_hashmask(struct prueth_emac *emac,
				    u8 mask[ICSS_EMAC_FW_MULTICAST_FILTER_MASK_SIZE_BYTES])
{
	struct prueth *prueth = emac->prueth;
	void __iomem *ram = prueth->mem[emac->dram].va;
	u32 mc_filter_mask_base = prueth->fw_offsets->mc_filter_mask;
	void __iomem *mc_filter_mask;

	if (PRUETH_IS_LRE(prueth))
		ram = prueth->mem[PRUETH_MEM_DRAM1].va;

	mc_filter_mask = ram + mc_filter_mask_base;
	memcpy_toio(mc_filter_mask, mask,
		    ICSS_EMAC_FW_MULTICAST_FILTER_MASK_SIZE_BYTES);
}

static void emac_mc_filter_bin_update(struct prueth_emac *emac, u8 hash, u8 val)
{
	struct prueth *prueth = emac->prueth;
	u32 mc_filter_tbl_base = prueth->fw_offsets->mc_filter_tbl;
	void __iomem *mc_filter_tbl;
	void __iomem *ram = prueth->mem[emac->dram].va;

	if (PRUETH_IS_LRE(prueth))
		ram = prueth->mem[PRUETH_MEM_DRAM1].va;

	mc_filter_tbl = ram + mc_filter_tbl_base;
	writeb(val, mc_filter_tbl + hash);
}

void emac_mc_filter_bin_allow(struct prueth_emac *emac, u8 hash)
{
	emac_mc_filter_bin_update(emac, hash, ICSS_EMAC_FW_MULTICAST_FILTER_HOST_RCV_ALLOWED);
}

void emac_mc_filter_bin_disallow(struct prueth_emac *emac, u8 hash)
{
	emac_mc_filter_bin_update(emac, hash, ICSS_EMAC_FW_MULTICAST_FILTER_HOST_RCV_NOT_ALLOWED);
}

u8 emac_get_mc_hash(u8 *mac, u8 *mask)
{
	int j;
	u8 hash;

	for (j = 0, hash = 0; j < ETH_ALEN; j++)
		hash ^= (mac[j] & mask[j]);

	return hash;
}

/**
 * emac_ndo_set_rx_mode - EMAC set receive mode function
 * @ndev: The EMAC network adapter
 *
 * Called when system wants to set the receive mode of the device.
 *
 */
static void emac_ndo_set_rx_mode(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth *prueth = emac->prueth;
	void __iomem *sram = prueth->mem[PRUETH_MEM_SHARED_RAM].va;
	u32 reg = readl(sram + EMAC_PROMISCUOUS_MODE_OFFSET);
	bool promisc = ndev->flags & IFF_PROMISC;
	struct netdev_hw_addr *ha;
	unsigned long flags;
	u32 mask;
	u8 hash;

	if (promisc && PRUETH_IS_LRE(prueth)) {
		netdev_dbg(ndev,
			   "%s: promisc mode not supported for LRE\n",
			   __func__);
		return;
	}

	/* for LRE, it is a shared table. So lock the access */
	spin_lock_irqsave(&emac->addr_lock, flags);

	/* Disable and reset multicast filter, allows allmulti */
	emac_mc_filter_ctrl(emac, false);
	emac_mc_filter_reset(emac);
	emac_mc_filter_hashmask(emac, emac->mc_filter_mask);

	if (PRUETH_IS_EMAC(prueth)) {
		switch (emac->port_id) {
		case PRUETH_PORT_MII0:
			mask = EMAC_P1_PROMISCUOUS_BIT;
			break;
		case PRUETH_PORT_MII1:
			mask = EMAC_P2_PROMISCUOUS_BIT;
			break;
		default:
			netdev_err(ndev, "%s: invalid port\n", __func__);
			goto unlock;
		}

		if (promisc) {
			/* Enable promiscuous mode */
			reg |= mask;
		} else {
			/* Disable promiscuous mode */
			reg &= ~mask;
		}

		writel(reg, sram + EMAC_PROMISCUOUS_MODE_OFFSET);

		if (promisc)
			goto unlock;
	}

	if (ndev->flags & IFF_ALLMULTI && !PRUETH_IS_SWITCH(prueth))
		goto unlock;

	emac_mc_filter_ctrl(emac, true);	/* all multicast blocked */

	if (netdev_mc_empty(ndev))
		goto unlock;

	netdev_for_each_mc_addr(ha, ndev) {
		hash = emac_get_mc_hash(ha->addr, emac->mc_filter_mask);
		emac_mc_filter_bin_allow(emac, hash);
	}

	/* Add bridge device's MC addresses as well */
	if (prueth->hw_bridge_dev) {
		netdev_for_each_mc_addr(ha, prueth->hw_bridge_dev) {
			hash = emac_get_mc_hash(ha->addr, emac->mc_filter_mask);
			emac_mc_filter_bin_allow(emac, hash);
		}
	}
unlock:
	spin_unlock_irqrestore(&emac->addr_lock, flags);
}

static int emac_hwtstamp_config_set(struct net_device *ndev, struct ifreq *ifr)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct hwtstamp_config cfg;

	if (copy_from_user(&cfg, ifr->ifr_data, sizeof(cfg)))
		return -EFAULT;

	/* reserved for future extensions */
	if (cfg.flags)
		return -EINVAL;

	if (cfg.tx_type != HWTSTAMP_TX_OFF && cfg.tx_type != HWTSTAMP_TX_ON)
		return -ERANGE;

	switch (cfg.rx_filter) {
	case HWTSTAMP_FILTER_NONE:
		prueth_ptp_rx_ts_enable(emac, 0);
		break;
	case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ:
	case HWTSTAMP_FILTER_PTP_V2_L2_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_L2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L2_DELAY_REQ:
	case HWTSTAMP_FILTER_PTP_V2_EVENT:
	case HWTSTAMP_FILTER_PTP_V2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
		prueth_ptp_rx_ts_enable(emac, 1);
		cfg.rx_filter = HWTSTAMP_FILTER_PTP_V2_EVENT;
		break;
	case HWTSTAMP_FILTER_ALL:
	case HWTSTAMP_FILTER_PTP_V1_L4_EVENT:
	case HWTSTAMP_FILTER_PTP_V1_L4_SYNC:
	case HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ:
	default:
		return -ERANGE;
	}

	prueth_ptp_tx_ts_enable(emac, cfg.tx_type == HWTSTAMP_TX_ON);

	return copy_to_user(ifr->ifr_data, &cfg, sizeof(cfg)) ? -EFAULT : 0;
}
static int emac_hwtstamp_config_get(struct net_device *ndev, struct ifreq *ifr)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct hwtstamp_config cfg;

	cfg.flags = 0;
	cfg.tx_type = prueth_ptp_tx_ts_is_enabled(emac) ?
		      HWTSTAMP_TX_ON : HWTSTAMP_TX_OFF;
	cfg.rx_filter = prueth_ptp_rx_ts_is_enabled(emac) ?
			HWTSTAMP_FILTER_PTP_V2_EVENT : HWTSTAMP_FILTER_NONE;

	return copy_to_user(ifr->ifr_data, &cfg, sizeof(cfg)) ? -EFAULT : 0;
}

static int emac_ndo_ioctl(struct net_device *ndev, struct ifreq *ifr, int cmd)
{
	struct prueth_emac *emac = netdev_priv(ndev);

        switch (cmd) {
        case SIOCSHWTSTAMP:
                return emac_hwtstamp_config_set(ndev, ifr);
        case SIOCGHWTSTAMP:
                return emac_hwtstamp_config_get(ndev, ifr);
        }

	return phy_mii_ioctl(emac->phydev, ifr, cmd);
}

int emac_add_del_vid(struct prueth_emac *emac,
		     bool add, __be16 proto, u16 vid)
{
	struct prueth *prueth = emac->prueth;
	u32 vlan_filter_tbl = prueth->fw_offsets->vlan_filter_tbl;
	void __iomem *ram = prueth->mem[emac->dram].va;
	unsigned long flags;
	u8 bit_index, val;
	u16 byte_index;

	if (proto != htons(ETH_P_8021Q))
		return -EINVAL;

	if (vid >= ICSS_EMAC_FW_VLAN_FILTER_VID_MAX)
		return -EINVAL;

	if (PRUETH_IS_LRE(prueth))
		ram =  prueth->mem[PRUETH_MEM_SHARED_RAM].va;

	/* By default, VLAN ID 0 (priority tagged packets) is routed to
	 * host, so nothing to be done if vid = 0
	 */
	if (!vid)
		return 0;

	/* for LRE, it is a shared table. So lock the access */
	spin_lock_irqsave(&emac->addr_lock, flags);

	/* VLAN filter table is 512 bytes (4096 bit) bitmap.
	 * Each bit controls enabling or disabling corresponding
	 * VID. Therefore byte index that controls a given VID is
	 * can calculated as vid / 8 and the bit within that byte
	 * that controls VID is given by vid % 8. Allow untagged
	 * frames to host by default.
	 */
	byte_index = vid / BITS_PER_BYTE;
	bit_index = vid % BITS_PER_BYTE;
	val = readb(ram + vlan_filter_tbl + byte_index);
	if (add)
		val |= BIT(bit_index);
	else
		val &= ~BIT(bit_index);
	writeb(val, ram + vlan_filter_tbl + byte_index);

	spin_unlock_irqrestore(&emac->addr_lock, flags);

	netdev_dbg(emac->ndev, "%s VID bit at index %d and bit %d\n",
		   add ? "Setting" : "Clearing", byte_index, bit_index);

	return 0;
}

static int emac_ndo_vlan_rx_add_vid(struct net_device *dev,
				    __be16 proto, u16 vid)
{
	struct prueth_emac *emac = netdev_priv(dev);

	return emac_add_del_vid(emac, true, proto, vid);
}

static int emac_ndo_vlan_rx_kill_vid(struct net_device *dev,
				     __be16 proto, u16 vid)
{
	struct prueth_emac *emac = netdev_priv(dev);

	return emac_add_del_vid(emac, false, proto, vid);
}

static int emac_get_port_parent_id(struct net_device *dev,
				   struct netdev_phys_item_id *ppid)
{
	struct prueth_emac *emac = netdev_priv(dev);
	struct prueth *prueth = emac->prueth;

	ppid->id_len = sizeof(prueth->base_mac);
	memcpy(&ppid->id, &prueth->base_mac, ppid->id_len);

	return 0;
}

/**
 * emac_ndo_set_features - function to set feature flag
 * @ndev: The network adapter device
 * @features: feature flags in the netdevice
 *
 * Called when ethtool -K option is invoked by user
 *
 * Change the eth_type in the prueth structure  based on hsr or prp
 * offload options from user through ethtool -K command. If the device
 * is running or if the other paired device is running, then don't accept.
 * Otherwise, set the ethernet type and offload feature flag
 *
 * Returns success if eth_type and feature flags are updated  or error
 * otherwise.
 */
static int emac_ndo_set_features(struct net_device *ndev,
				 netdev_features_t features)
{
	struct prueth_emac *emac = netdev_priv(ndev), *other_emac;
	struct prueth *prueth = emac->prueth;
	enum prueth_port other_port;
	netdev_features_t wanted = features &
		(NETIF_F_HW_HSR_RX_OFFLOAD | NETIF_F_HW_PRP_RX_OFFLOAD);
	netdev_features_t have = ndev->features &
		(NETIF_F_HW_HSR_RX_OFFLOAD | NETIF_F_HW_PRP_RX_OFFLOAD);
	bool change_request = ((wanted ^ have) != 0);
	int ret = -EBUSY;

	if (!prueth->support_lre)
		return 0;

	if (PRUETH_IS_SWITCH(prueth)) {
		/* Don't allow switching to HSR/PRP ethtype from Switch.
		 * User needs to first remove eth ports from a bridge which
		 * will automatically put the ethtype back to EMAC. So
		 * disallow this.
		 */
		netdev_err(ndev,
			   "Switch to HSR/PRP/EMAC not allowed\n");
		return -EINVAL;
	}

	if (netif_running(ndev) && change_request) {
		netdev_err(ndev,
			   "Can't change feature when device runs\n");
		return ret;
	}

	other_port = other_port_id(emac->port_id);
	/* MAC instance index starts from 0. So index by port_id - 1 */
	other_emac = prueth->emac[other_port - 1];
	if (other_emac && netif_running(other_emac->ndev) && change_request) {
		netdev_err(ndev,
			   "Can't change feature when other device runs\n");
		return ret;
	}

	if (features & NETIF_F_HW_HSR_RX_OFFLOAD) {
		prueth->eth_type = PRUSS_ETHTYPE_HSR;
		ndev->features = ndev->features & ~NETIF_F_HW_PRP_RX_OFFLOAD;
		ndev->features |= (NETIF_F_HW_HSR_RX_OFFLOAD |
				   NETIF_F_HW_L2FW_DOFFLOAD);

	} else if (features & NETIF_F_HW_PRP_RX_OFFLOAD) {
		prueth->eth_type = PRUSS_ETHTYPE_PRP;
		ndev->features = ndev->features & ~NETIF_F_HW_HSR_RX_OFFLOAD;
		ndev->features |= NETIF_F_HW_PRP_RX_OFFLOAD;
		ndev->features &= ~NETIF_F_HW_L2FW_DOFFLOAD;
	} else {
		prueth->eth_type = PRUSS_ETHTYPE_EMAC;
		ndev->features |= NETIF_F_HW_L2FW_DOFFLOAD;
		ndev->features =
			(ndev->features & ~(NETIF_F_HW_HSR_RX_OFFLOAD |
					NETIF_F_HW_PRP_RX_OFFLOAD));
	}

	return 0;
}

static const struct net_device_ops emac_netdev_ops = {
	.ndo_open = emac_ndo_open,
	.ndo_stop = emac_ndo_stop,
	.ndo_start_xmit = emac_ndo_start_xmit,
	.ndo_set_mac_address = eth_mac_addr,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_change_mtu	= eth_change_mtu,
	.ndo_tx_timeout = emac_ndo_tx_timeout,
	.ndo_get_stats = emac_ndo_get_stats,
	.ndo_set_rx_mode = emac_ndo_set_rx_mode,
	.ndo_do_ioctl = emac_ndo_ioctl,
	.ndo_set_features = emac_ndo_set_features,
	.ndo_vlan_rx_add_vid = emac_ndo_vlan_rx_add_vid,
	.ndo_vlan_rx_kill_vid = emac_ndo_vlan_rx_kill_vid,
	.ndo_setup_tc = emac_ndo_setup_tc,
	.ndo_get_port_parent_id = emac_get_port_parent_id,
};

/**
 * emac_get_drvinfo - Get EMAC driver information
 * @ndev: The network adapter
 * @info: ethtool info structure containing name and version
 *
 * Returns EMAC driver information (name and version)
 */
static void emac_get_drvinfo(struct net_device *ndev,
			     struct ethtool_drvinfo *info)
{
	strlcpy(info->driver, PRUETH_MODULE_DESCRIPTION, sizeof(info->driver));
	strlcpy(info->version, PRUETH_MODULE_VERSION, sizeof(info->version));
}

/**
 * emac_get_link_ksettings - Get EMAC settings
 * @ndev: The network adapter
 * @ecmd: ethtool command
 *
 * Executes ethool get command
 */
static int emac_get_link_ksettings(struct net_device *ndev,
				   struct ethtool_link_ksettings *ecmd)
{
	struct prueth_emac *emac = netdev_priv(ndev);

	if (!emac->phydev)
		return -EOPNOTSUPP;

	phy_ethtool_ksettings_get(emac->phydev, ecmd);
	return 0;
}

/**
 * emac_set_link_ksettings - Set EMAC settings
 * @ndev: The EMAC network adapter
 * @ecmd: ethtool command
 *
 * Executes ethool set command
 */
static int emac_set_link_ksettings(struct net_device *ndev,
				   const struct ethtool_link_ksettings *ecmd)
{
	struct prueth_emac *emac = netdev_priv(ndev);

	if (!emac->phydev)
		return -EOPNOTSUPP;

	return phy_ethtool_ksettings_set(emac->phydev, ecmd);
}

#define PRUETH_STAT_OFFSET(m) offsetof(struct port_statistics, m)

static const struct {
	char string[ETH_GSTRING_LEN];
	u32 offset;
} prueth_ethtool_stats[] = {
	{"txBcast", PRUETH_STAT_OFFSET(tx_bcast)},
	{"txMcast", PRUETH_STAT_OFFSET(tx_mcast)},
	{"txUcast", PRUETH_STAT_OFFSET(tx_ucast)},
	{"txOctets", PRUETH_STAT_OFFSET(tx_octets)},
	{"rxBcast", PRUETH_STAT_OFFSET(rx_bcast)},
	{"rxMcast", PRUETH_STAT_OFFSET(rx_mcast)},
	{"rxUcast", PRUETH_STAT_OFFSET(rx_ucast)},
	{"rxOctets", PRUETH_STAT_OFFSET(rx_octets)},

	{"tx64byte", PRUETH_STAT_OFFSET(tx64byte)},
	{"tx65_127byte", PRUETH_STAT_OFFSET(tx65_127byte)},
	{"tx128_255byte", PRUETH_STAT_OFFSET(tx128_255byte)},
	{"tx256_511byte", PRUETH_STAT_OFFSET(tx256_511byte)},
	{"tx512_1023byte", PRUETH_STAT_OFFSET(tx512_1023byte)},
	{"tx1024byte", PRUETH_STAT_OFFSET(tx1024byte)},
	{"rx64byte", PRUETH_STAT_OFFSET(rx64byte)},
	{"rx65_127byte", PRUETH_STAT_OFFSET(rx65_127byte)},
	{"rx128_255byte", PRUETH_STAT_OFFSET(rx128_255byte)},
	{"rx256_511byte", PRUETH_STAT_OFFSET(rx256_511byte)},
	{"rx512_1023byte", PRUETH_STAT_OFFSET(rx512_1023byte)},
	{"rx1024byte", PRUETH_STAT_OFFSET(rx1024byte)},

	{"lateColl", PRUETH_STAT_OFFSET(late_coll)},
	{"singleColl", PRUETH_STAT_OFFSET(single_coll)},
	{"multiColl", PRUETH_STAT_OFFSET(multi_coll)},
	{"excessColl", PRUETH_STAT_OFFSET(excess_coll)},

	{"rxMisAlignmentFrames", PRUETH_STAT_OFFSET(rx_misalignment_frames)},
	{"stormPrevCounterBC", PRUETH_STAT_OFFSET(stormprev_counter_bc)},
	{"stormPrevCounterMC", PRUETH_STAT_OFFSET(stormprev_counter_mc)},
	{"stormPrevCounterUC", PRUETH_STAT_OFFSET(stormprev_counter_uc)},
	{"macRxError", PRUETH_STAT_OFFSET(mac_rxerror)},
	{"SFDError", PRUETH_STAT_OFFSET(sfd_error)},
	{"defTx", PRUETH_STAT_OFFSET(def_tx)},
	{"macTxError", PRUETH_STAT_OFFSET(mac_txerror)},
	{"rxOverSizedFrames", PRUETH_STAT_OFFSET(rx_oversized_frames)},
	{"rxUnderSizedFrames", PRUETH_STAT_OFFSET(rx_undersized_frames)},
	{"rxCRCFrames", PRUETH_STAT_OFFSET(rx_crc_frames)},
	{"droppedPackets", PRUETH_STAT_OFFSET(dropped_packets)},

	{"txHWQOverFlow", PRUETH_STAT_OFFSET(tx_hwq_overflow)},
	{"txHWQUnderFlow", PRUETH_STAT_OFFSET(tx_hwq_underflow)},
	{"vlanDropped", PRUETH_STAT_OFFSET(vlan_dropped)},
	{"multicastDropped", PRUETH_STAT_OFFSET(multicast_dropped)},
};

static int emac_get_sset_count(struct net_device *ndev, int stringset)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	int a_size;

	switch (stringset) {
	case ETH_SS_STATS:
		a_size = ARRAY_SIZE(prueth_ethtool_stats);
		a_size += prueth_lre_get_sset_count(emac->prueth);

		return a_size;
	default:
		return -EOPNOTSUPP;
	}
}

static void emac_get_strings(struct net_device *ndev, u32 stringset, u8 *data)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	u8 *p = data;
	int i;

	switch (stringset) {
	case ETH_SS_STATS:
		for (i = 0; i < ARRAY_SIZE(prueth_ethtool_stats); i++) {
			memcpy(p, prueth_ethtool_stats[i].string,
			       ETH_GSTRING_LEN);
			p += ETH_GSTRING_LEN;
		}
		prueth_lre_get_strings(emac->prueth, p);
		break;
	default:
		break;
	}
}

static void emac_get_ethtool_stats(struct net_device *ndev,
				   struct ethtool_stats *stats, u64 *data)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct port_statistics pstats;
	u32 val;
	int i;
	void *ptr;

	emac_get_stats(emac, &pstats);

	for (i = 0; i < ARRAY_SIZE(prueth_ethtool_stats); i++) {
		ptr = &pstats;
		ptr += prueth_ethtool_stats[i].offset;
		val = *(u32 *)ptr;
		data[i] = val;
	}
	prueth_lre_update_stats(emac->prueth, &data[i]);
}

static int emac_get_regs_len(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth *prueth = emac->prueth;

	/* VLAN Table at the end of the memory map, after MultiCast
	 * filter region. So VLAN table base +
	 * size will give the entire size of reg dump in case of
	 * Dual-EMAC firmware.
	 */
	if (PRUETH_IS_EMAC(prueth) || PRUETH_IS_SWITCH(prueth)) {
		return ICSS_EMAC_FW_VLAN_FLTR_TBL_BASE_ADDR +
		       ICSS_EMAC_FW_VLAN_FILTER_TABLE_SIZE_BYTES;
	}

	/* MultiCast table and VLAN filter table are in different
	 * memories in case of HSR/PRP firmware. Therefore add the sizes
	 * of individual region.
	 */
	if (PRUETH_IS_LRE(prueth)) {
		return ICSS_LRE_FW_VLAN_FLTR_TBL_BASE_ADDR +
		       ICSS_EMAC_FW_VLAN_FILTER_TABLE_SIZE_BYTES +
		       ICSS_LRE_FW_MULTICAST_FILTER_TABLE +
		       ICSS_EMAC_FW_MULTICAST_TABLE_SIZE_BYTES;
	}

	return 0;
}

static void emac_get_regs(struct net_device *ndev, struct ethtool_regs *regs,
			  void *p)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth *prueth = emac->prueth;
	void __iomem *ram;
	u8 *reg = p;

	regs->version = PRUETH_REG_DUMP_GET_VER(prueth);

	/* Dump firmware's VLAN and MC tables */
	if (PRUETH_IS_EMAC(prueth) || PRUETH_IS_SWITCH(prueth)) {
		ram = prueth->mem[emac->dram].va;
		memcpy_fromio(reg, ram, emac_get_regs_len(ndev));
		return;
	}

	if (PRUETH_IS_LRE(prueth)) {
		size_t len = ICSS_LRE_FW_VLAN_FLTR_TBL_BASE_ADDR +
			     ICSS_EMAC_FW_VLAN_FILTER_TABLE_SIZE_BYTES;

		ram =  prueth->mem[PRUETH_MEM_SHARED_RAM].va;
		memcpy_fromio(reg, ram, len);

		reg += len;

		ram = prueth->mem[PRUETH_MEM_DRAM1].va;
		len = ICSS_LRE_FW_MULTICAST_FILTER_TABLE +
		      ICSS_EMAC_FW_MULTICAST_TABLE_SIZE_BYTES;
		memcpy_fromio(reg, ram, len);
	}
}

static int emac_get_ts_info(struct net_device *ndev,
			    struct ethtool_ts_info *info)
{
	struct prueth_emac *emac = netdev_priv(ndev);

	if ((PRUETH_IS_EMAC(emac->prueth) && !emac->emac_ptp_tx_irq) ||
	    (PRUETH_IS_LRE(emac->prueth) && !emac->hsr_ptp_tx_irq))
		return ethtool_op_get_ts_info(ndev, info);

	info->so_timestamping =
		SOF_TIMESTAMPING_TX_HARDWARE |
		SOF_TIMESTAMPING_TX_SOFTWARE |
		SOF_TIMESTAMPING_RX_HARDWARE |
		SOF_TIMESTAMPING_RX_SOFTWARE |
		SOF_TIMESTAMPING_SOFTWARE |
		SOF_TIMESTAMPING_RAW_HARDWARE;

	info->phc_index = ptp_clock_index(icss_iep_get_ptp_clock(emac->prueth->iep));
	info->tx_types = BIT(HWTSTAMP_TX_OFF) | BIT(HWTSTAMP_TX_ON);
	info->rx_filters = BIT(HWTSTAMP_FILTER_NONE) | BIT(HWTSTAMP_FILTER_PTP_V2_EVENT);

	return 0;
}

static int emac_get_coalesce(struct net_device *ndev,
			     struct ethtool_coalesce *coal)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth *prueth = emac->prueth;
	struct prueth_ecap *ecap = prueth->ecap;

	if (IS_ERR(ecap))
		return -EOPNOTSUPP;

	return ecap->get_coalesce(emac, &coal->use_adaptive_rx_coalesce,
				  &coal->rx_coalesce_usecs);
}

static int emac_set_coalesce(struct net_device *ndev,
			     struct ethtool_coalesce *coal)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth *prueth = emac->prueth;
	struct prueth_ecap *ecap = prueth->ecap;

	if (IS_ERR(ecap))
		return -EOPNOTSUPP;

	return ecap->set_coalesce(emac, coal->use_adaptive_rx_coalesce,
				  coal->rx_coalesce_usecs);
}

/* Ethtool support for EMAC adapter */
static const struct ethtool_ops emac_ethtool_ops = {
	.get_drvinfo = emac_get_drvinfo,
	.get_link_ksettings = emac_get_link_ksettings,
	.set_link_ksettings = emac_set_link_ksettings,
	.get_link = ethtool_op_get_link,
	.get_ts_info = emac_get_ts_info,
	.get_sset_count = emac_get_sset_count,
	.get_strings = emac_get_strings,
	.get_ethtool_stats = emac_get_ethtool_stats,
	.get_regs = emac_get_regs,
	.get_regs_len = emac_get_regs_len,
	.get_coalesce = emac_get_coalesce,
	.set_coalesce = emac_set_coalesce,
};

/* get emac_port corresponding to eth_node name */
static int prueth_node_port(struct device_node *eth_node)
{
	if (!strcmp(eth_node->name, "ethernet-mii0"))
		return PRUETH_PORT_MII0;
	else if (!strcmp(eth_node->name, "ethernet-mii1"))
		return PRUETH_PORT_MII1;
	else
		return -EINVAL;
}

/* get MAC instance corresponding to eth_node name */
static int prueth_node_mac(struct device_node *eth_node)
{
	if (!strcmp(eth_node->name, "ethernet-mii0"))
		return PRUETH_MAC0;
	else if (!strcmp(eth_node->name, "ethernet-mii1"))
		return PRUETH_MAC1;
	else
		return -EINVAL;
}

static int prueth_netdev_init(struct prueth *prueth,
			      struct device_node *eth_node)
{
	const struct prueth_private_data *fw_data = prueth->fw_data;
	enum prueth_port port;
	enum prueth_mac mac;
	struct net_device *ndev;
	struct prueth_emac *emac;
	const u8 *mac_addr;
	int ret;

	port = prueth_node_port(eth_node);
	if (port < 0)
		return -EINVAL;

	mac = prueth_node_mac(eth_node);
	if (mac < 0)
		return -EINVAL;

	ndev = devm_alloc_etherdev(prueth->dev, sizeof(*emac));
	if (!ndev)
		return -ENOMEM;

	SET_NETDEV_DEV(ndev, prueth->dev);
	emac = netdev_priv(ndev);
	prueth->emac[mac] = emac;
	emac->prueth = prueth;
	emac->ndev = ndev;
	emac->port_id = port;
	memset(&emac->mc_filter_mask[0], 0xff, ETH_ALEN); /* default mask */

	/* by default eth_type is EMAC */
	switch (port) {
	case PRUETH_PORT_MII0:
		emac->tx_port_queue = PRUETH_PORT_QUEUE_MII0;

		/* packets from MII0 are on queues 1 through 2 */
		emac->rx_queue_start = PRUETH_QUEUE1;
		emac->rx_queue_end = PRUETH_QUEUE2;

		emac->dram = PRUETH_MEM_DRAM0;
		emac->pru = prueth->pru0;
		break;
	case PRUETH_PORT_MII1:
		emac->tx_port_queue = PRUETH_PORT_QUEUE_MII1;

		/* packets from MII1 are on queues 3 through 4 */
		emac->rx_queue_start = PRUETH_QUEUE3;
		emac->rx_queue_end = PRUETH_QUEUE4;

		emac->dram = PRUETH_MEM_DRAM1;
		emac->pru = prueth->pru1;
		break;
	default:
		return -EINVAL;
	}

	emac->rx_irq = of_irq_get_byname(eth_node, "rx");
	if (emac->rx_irq < 0) {
		ret = emac->rx_irq;
		if (ret != -EPROBE_DEFER)
			dev_err(prueth->dev, "could not get rx irq\n");
		goto free;
	}
	emac->tx_irq = of_irq_get_byname(eth_node, "tx");
	if (emac->tx_irq < 0) {
		if (emac->tx_irq != -EPROBE_DEFER)
			dev_dbg(prueth->dev, "tx irq not configured\n");
	}

	emac->emac_ptp_tx_irq = of_irq_get_byname(eth_node, "emac_ptp_tx");
	if (emac->emac_ptp_tx_irq < 0) {
		emac->emac_ptp_tx_irq = 0;
		dev_err(prueth->dev, "could not get ptp tx irq. Skipping PTP support\n");
	}

	emac->hsr_ptp_tx_irq = of_irq_get_byname(eth_node, "hsr_ptp_tx");
	if (emac->hsr_ptp_tx_irq < 0) {
		emac->hsr_ptp_tx_irq = 0;
		dev_err(prueth->dev, "could not get hsr ptp tx irq. Skipping PTP support\n");
	}

	emac->msg_enable = netif_msg_init(debug_level, PRUETH_EMAC_DEBUG);
	spin_lock_init(&emac->lock);
	spin_lock_init(&emac->nsp_lock);
	spin_lock_init(&emac->addr_lock);
	spin_lock_init(&emac->ptp_skb_lock);

	/* get mac address from DT and set private and netdev addr */
	mac_addr = of_get_mac_address(eth_node);
	if (!IS_ERR(mac_addr))
		ether_addr_copy(ndev->dev_addr, mac_addr);
	if (!is_valid_ether_addr(ndev->dev_addr)) {
		eth_hw_addr_random(ndev);
		dev_warn(prueth->dev, "port %d: using random MAC addr: %pM\n",
			 port, ndev->dev_addr);
	}
	ether_addr_copy(emac->mac_addr, ndev->dev_addr);

	emac->phy_node = of_parse_phandle(eth_node, "phy-handle", 0);
	if (!emac->phy_node) {
		dev_err(prueth->dev, "couldn't find phy-handle\n");
		ret = -ENODEV;
		goto free;
	}

	emac->phy_if = of_get_phy_mode(eth_node);
	if (emac->phy_if < 0) {
		dev_err(prueth->dev, "could not get phy-mode property\n");
		ret = emac->phy_if;
		goto free;
	}

	/* connect PHY */
	emac->phydev = of_phy_connect(ndev, emac->phy_node,
				      &emac_adjust_link, 0, emac->phy_if);
	if (!emac->phydev) {
		dev_dbg(prueth->dev, "couldn't connect to phy %s\n",
			emac->phy_node->full_name);
		ret = -EPROBE_DEFER;
		goto free;
	}

	/* remove unsupported modes */
	phy_remove_link_mode(emac->phydev, ETHTOOL_LINK_MODE_10baseT_Half_BIT);
	phy_remove_link_mode(emac->phydev, ETHTOOL_LINK_MODE_10baseT_Full_BIT);

	if (of_property_read_bool(eth_node, "ti,no-half-duplex")) {
		phy_remove_link_mode(emac->phydev,
				     ETHTOOL_LINK_MODE_100baseT_Half_BIT);
	}

	phy_remove_link_mode(emac->phydev, ETHTOOL_LINK_MODE_Pause_BIT);
	phy_remove_link_mode(emac->phydev, ETHTOOL_LINK_MODE_Asym_Pause_BIT);

	ndev->features |= NETIF_F_HW_VLAN_CTAG_FILTER | NETIF_F_HW_TC;
	if (fw_data->support_switch || prueth->support_lre)
		ndev->features |= NETIF_F_HW_L2FW_DOFFLOAD;

	/* HSR uses two offload flags; NETIF_F_HW_HSR_RX_OFFLOAD and
	 * NETIF_F_HW_L2FW_DOFFLOAD since it supports cut-through switching,
	 * i.e a kind of L2 offload similar to Switch, but no routing. So
	 * driver currently sets NETIF_F_HW_L2FW_DOFFLOAD as well for HSR.
	 * To differentiate between pure switch and HSR switch,
	 * NETIF_F_HW_HSR_RX_OFFLOAD is used currently. For example to if
	 * need to process switch events, check if NETIF_F_HW_HSR_RX_OFFLOAD
	 * is set in feature flag and NETIF_F_HW_HSR_RX_OFFLOAD is reset.
	 */
	if (prueth->support_lre)
		ndev->hw_features |= (NETIF_F_HW_PRP_RX_OFFLOAD |
				      NETIF_F_HW_HSR_RX_OFFLOAD |
				      NETIF_F_HW_L2FW_DOFFLOAD);
	if (fw_data->support_switch)
		ndev->hw_features |= NETIF_F_HW_L2FW_DOFFLOAD;

	ndev->hw_features |= NETIF_F_HW_VLAN_CTAG_FILTER;

	ndev->netdev_ops = &emac_netdev_ops;
	ndev->ethtool_ops = &emac_ethtool_ops;
#if (IS_ENABLED(CONFIG_HSR))
	if (prueth->support_lre)
		ndev->lredev_ops = &prueth_lredev_ops;
#endif

	if (PRUETH_IS_EMAC(prueth) || PRUETH_IS_SWITCH(prueth))
		netif_napi_add(ndev, &emac->napi, emac_napi_poll,
			       EMAC_POLL_WEIGHT);
	/* for HSR/PRP,  register napi for port 1 */
	if (prueth->support_lre && emac->port_id == PRUETH_PORT_MII0) {
		netif_napi_add(ndev, &prueth->napi_hpq,
			       prueth_lre_napi_poll_hpq, EMAC_POLL_WEIGHT);
		netif_napi_add(ndev, &prueth->napi_lpq,
			       prueth_lre_napi_poll_lpq, EMAC_POLL_WEIGHT);
		prueth->hp->ndev = ndev;
		prueth->hp->priority = 0;
		prueth->lp->ndev = ndev;
		prueth->lp->priority = 1;
	}

	return 0;

free:
	prueth->emac[mac] = NULL;

	return ret;
}

static void prueth_netdev_exit(struct prueth *prueth,
			       struct device_node *eth_node)
{
	struct prueth_emac *emac;
	enum prueth_mac mac;

	mac = prueth_node_mac(eth_node);
	if (mac < 0)
		return;

	emac = prueth->emac[mac];
	if (!emac)
		return;

	phy_disconnect(emac->phydev);

	if (PRUETH_IS_EMAC(prueth) || PRUETH_IS_SWITCH(prueth)) {
		netif_napi_del(&emac->napi);
	} else {
		if (prueth->support_lre &&
		    emac->port_id == PRUETH_PORT_MII0) {
			netif_napi_del(&prueth->napi_hpq);
			netif_napi_del(&prueth->napi_lpq);
		}
	}
	prueth->emac[mac] = NULL;
}

bool prueth_sw_port_dev_check(const struct net_device *ndev)
{
	if (ndev->netdev_ops != &emac_netdev_ops)
		return false;

	if ((ndev->features & NETIF_F_HW_L2FW_DOFFLOAD) &&
	    !(ndev->features & NETIF_F_HW_HSR_RX_OFFLOAD))
		return true;

	return false;
}

static void prueth_port_offload_fwd_mark_update(struct prueth *prueth)
{
	int set_val = 0;
	int i;
	u8 all_slaves = BIT(PRUETH_PORT_MII0) | BIT(PRUETH_PORT_MII1);

	if (prueth->br_members == all_slaves)
		set_val = 1;

	dev_dbg(prueth->dev, "set offload_fwd_mark %d, mbrs=0x%x\n",
		set_val, prueth->br_members);

	for (i = 0; i < PRUETH_NUM_MACS; i++)
		prueth->emac[i]->offload_fwd_mark = set_val;

	/* Bridge is created, load switch firmware, if not already in
	 * that mode
	 */
	if (set_val && !PRUETH_IS_SWITCH(prueth))
		prueth_change_to_switch_mode(prueth);

	/* Bridge is deleted, switch to Dual EMAC mode */
	if (!prueth->br_members && !PRUETH_IS_EMAC(prueth))
		prueth_change_to_emac_mode(prueth);
}

static int prueth_ndev_port_link(struct net_device *ndev,
				 struct net_device *br_ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth *prueth = emac->prueth;

	dev_dbg(prueth->dev, "%s: br_mbrs=0x%x %s\n",
		__func__, prueth->br_members, ndev->name);

	if (!prueth->br_members) {
		prueth->hw_bridge_dev = br_ndev;
	} else {
		/* This is adding the port to a second bridge, this is
		 * unsupported
		 */
		if (prueth->hw_bridge_dev != br_ndev)
			return -EOPNOTSUPP;
	}

	prueth->br_members |= BIT(emac->port_id);

	prueth_port_offload_fwd_mark_update(prueth);

	return NOTIFY_DONE;
}

static void prueth_ndev_port_unlink(struct net_device *ndev)
{
	struct prueth_emac *emac = netdev_priv(ndev);
	struct prueth *prueth = emac->prueth;

	dev_dbg(prueth->dev, "emac_sw_ndev_port_unlink\n");

	prueth->br_members &= ~BIT(emac->port_id);

	prueth_port_offload_fwd_mark_update(prueth);

	if (!prueth->br_members)
		prueth->hw_bridge_dev = NULL;
}

static int prueth_ndev_event(struct notifier_block *unused,
			     unsigned long event, void *ptr)
{
	struct net_device *ndev = netdev_notifier_info_to_dev(ptr);
	struct netdev_notifier_changeupper_info *info;
	int ret = NOTIFY_DONE;

	if (!prueth_sw_port_dev_check(ndev))
		return NOTIFY_DONE;

	switch (event) {
	case NETDEV_CHANGEUPPER:
		info = ptr;

		if (netif_is_bridge_master(info->upper_dev)) {
			if (info->linking)
				ret = prueth_ndev_port_link(ndev,
							    info->upper_dev);
			else
				prueth_ndev_port_unlink(ndev);
		}
		break;
	default:
		return NOTIFY_DONE;
	}

	return notifier_from_errno(ret);
}

static int prueth_register_notifiers(struct prueth *prueth)
{
	struct notifier_block *nb;
	int ret;

	nb = &prueth->prueth_ndev_nb;
	nb->notifier_call = prueth_ndev_event;
	ret = register_netdevice_notifier(nb);
	if (ret) {
		dev_err(prueth->dev,
			"register netdevice notifier failed ret: %d\n", ret);
		return ret;
	}

	ret = prueth_sw_register_notifiers(prueth);
	if (ret) {
		unregister_netdevice_notifier(nb);
		return ret;
	}

	return 0;
}
static const struct of_device_id prueth_dt_match[];

static int prueth_probe(struct platform_device *pdev)
{
	struct prueth *prueth;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *eth0_node, *eth1_node;
	const struct of_device_id *match;
	bool has_lre = false;
	struct pruss *pruss;
	int i, ret;

	if (!np)
		return -ENODEV;	/* we don't support non DT */

	match = of_match_device(prueth_dt_match, dev);
	if (!match)
		return -ENODEV;

	prueth = devm_kzalloc(dev, sizeof(*prueth), GFP_KERNEL);
	if (!prueth)
		return -ENOMEM;

	platform_set_drvdata(pdev, prueth);

	prueth->dev = dev;
	prueth->fw_data = match->data;
	prueth->prueth_np = np;

	if (prueth->fw_data->rev_2_1)
		prueth->fw_offsets = &fw_offsets_v2_1;
	else
		prueth->fw_offsets = &fw_offsets_v1_0;

	eth0_node = of_get_child_by_name(np, "ethernet-mii0");
	if (!of_device_is_available(eth0_node)) {
		of_node_put(eth0_node);
		eth0_node = NULL;
	}

	eth1_node = of_get_child_by_name(np, "ethernet-mii1");
	if (!of_device_is_available(eth1_node)) {
		of_node_put(eth1_node);
		eth1_node = NULL;
	}

	/* At least one node must be present and available else we fail */
	if (!eth0_node && !eth1_node) {
		dev_err(dev, "neither ethernet-mii0 nor ethernet-mii1 node available\n");
		ret = -ENODEV;
		goto put_node;
	}

	prueth->eth_node[PRUETH_MAC0] = eth0_node;
	prueth->eth_node[PRUETH_MAC1] = eth1_node;

	prueth->mii_rt = syscon_regmap_lookup_by_phandle(np, "mii-rt");
	if (IS_ERR(prueth->mii_rt)) {
		dev_err(dev, "couldn't get mii-rt syscon regmap\n");
		return -ENODEV;
	}

	if (eth0_node) {
		prueth->pru0 = pru_rproc_get(np, 0);
		if (IS_ERR(prueth->pru0)) {
			ret = PTR_ERR(prueth->pru0);
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "unable to get PRU0: %d\n", ret);
			goto put_node;
		}
	}

	if (eth1_node) {
		prueth->pru1 = pru_rproc_get(np, 1);
		if (IS_ERR(prueth->pru1)) {
			ret = PTR_ERR(prueth->pru1);
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "unable to get PRU1: %d\n", ret);
			goto put_pru0;
		}
	}

	pruss = pruss_get(prueth->pru0 ? prueth->pru0 : prueth->pru1);
	if (IS_ERR(pruss)) {
		ret = PTR_ERR(pruss);
		dev_err(dev, "unable to get pruss handle\n");
		goto put_pru1;
	}
	prueth->pruss = pruss;

	ret = pruss_cfg_ocp_master_ports(prueth->pruss, 1);
	if (ret) {
		dev_err(dev, "couldn't enabled ocp master port: %d\n", ret);
		goto put_pruss;
	}

	/* Configure PRUSS */
	if (eth0_node)
		pruss_cfg_gpimode(pruss, prueth->pru0, PRUSS_GPI_MODE_MII);
	if (eth1_node)
		pruss_cfg_gpimode(pruss, prueth->pru1, PRUSS_GPI_MODE_MII);
	pruss_cfg_miirt_enable(pruss, true);
	pruss_cfg_xfr_enable(pruss, true);

	/* Get PRUSS mem resources */
	/* OCMC is system resource which we get separately */
	for (i = 0; i < ARRAY_SIZE(pruss_mem_ids); i++) {
		/* skip appropriate DRAM if not required */
		if (!eth0_node && i == PRUETH_MEM_DRAM0)
			continue;

		if (!eth1_node && i == PRUETH_MEM_DRAM1)
			continue;

		ret = pruss_request_mem_region(pruss, pruss_mem_ids[i],
					       &prueth->mem[i]);
		if (ret) {
			dev_err(dev, "unable to get PRUSS resource %d: %d\n",
				i, ret);
			goto put_mem;
		}
	}

	prueth->sram_pool = of_gen_pool_get(np, "sram", 0);
	if (!prueth->sram_pool) {
		dev_err(dev, "unable to get SRAM pool\n");
		ret = -ENODEV;

		goto put_mem;
	}
	prueth->mem[PRUETH_MEM_OCMC].va =
			(void __iomem *)gen_pool_alloc(prueth->sram_pool,
						       OCMC_RAM_SIZE);
	if (!prueth->mem[PRUETH_MEM_OCMC].va) {
		dev_err(dev, "unable to allocate OCMC resource\n");
		ret = -ENOMEM;
		goto put_mem;
	}
	prueth->mem[PRUETH_MEM_OCMC].pa =
			gen_pool_virt_to_phys(prueth->sram_pool,
					      (unsigned long)prueth->mem[PRUETH_MEM_OCMC].va);
	prueth->mem[PRUETH_MEM_OCMC].size = OCMC_RAM_SIZE;
	dev_dbg(dev, "ocmc: pa %pa va %p size %#zx\n",
		&prueth->mem[PRUETH_MEM_OCMC].pa,
		prueth->mem[PRUETH_MEM_OCMC].va,
		prueth->mem[PRUETH_MEM_OCMC].size);

	if (IS_ENABLED(CONFIG_HSR) && prueth->fw_data->support_lre)
		has_lre = true;

	/* if lre is supported, then both eth nodes to be present in
	 * DT node. If not, reset the support flag
	 */
	if (has_lre && (!eth0_node || !eth1_node))
		has_lre = false;

	if (has_lre) {
		/* need to configure interrupts per queue common for
		 * both ports
		 */
		prueth->hp = devm_kzalloc(dev,
					  sizeof(struct prueth_ndev_priority),
					  GFP_KERNEL);
		if (!prueth->hp) {
			ret = -ENOMEM;
			goto free_pool;
		}
		prueth->lp = devm_kzalloc(dev,
					  sizeof(struct prueth_ndev_priority),
					  GFP_KERNEL);
		if (!prueth->hp) {
			ret = -ENOMEM;
			goto free_pool;
		}

		prueth->lre_stats = devm_kzalloc(dev,
						 sizeof(*prueth->lre_stats),
						 GFP_KERNEL);
		if (!prueth->lre_stats) {
			ret = -ENOMEM;
			goto free_pool;
		}

		prueth->rx_lpq_irq = of_irq_get_byname(np, "rx_lre_lp");
		prueth->rx_hpq_irq = of_irq_get_byname(np, "rx_lre_hp");
		if (prueth->rx_lpq_irq < 0 || prueth->rx_hpq_irq < 0)
			has_lre = false;
	}
	prueth->support_lre = has_lre;

	/* setup netdev interfaces */
	mutex_init(&prueth->mlock);
	if (eth0_node) {
		ret = prueth_netdev_init(prueth, eth0_node);
		if (ret) {
			if (ret != -EPROBE_DEFER) {
				dev_err(dev, "netdev init %s failed: %d\n",
					eth0_node->name, ret);
			}
			goto free_pool;
		}
	}

	if (eth1_node) {
		ret = prueth_netdev_init(prueth, eth1_node);
		if (ret) {
			if (ret != -EPROBE_DEFER) {
				dev_err(dev, "netdev init %s failed: %d\n",
					eth1_node->name, ret);
			}
			goto netdev_exit;
		}
	}

	prueth->iep = icss_iep_get(np);
	if (IS_ERR(prueth->iep)) {
		ret = PTR_ERR(prueth->iep);
		dev_err(dev, "unable to get IEP\n");
		goto netdev_exit;
	}

	/* Make rx interrupt pacing optional so that users can use ECAP for
	 * other use cases if needed
	 */
	prueth->ecap = prueth_ecap_get(np);
	if (IS_ERR(prueth->ecap)) {
		ret = PTR_ERR(prueth->ecap);
		if (ret != -EPROBE_DEFER)
			dev_info(dev,
				 "No ECAP. Rx interrupt pacing disabled\n");
		else
			goto iep_put;
	}

	prueth_set_fw_offsets(prueth);
	prueth_hostinit(prueth);

	/* register the network devices */
	if (eth0_node) {
		ret = register_netdev(prueth->emac[PRUETH_MAC0]->ndev);
		if (ret) {
			dev_err(dev, "can't register netdev for port MII0");
			goto ecap_put;
		}

		prueth->registered_netdevs[PRUETH_MAC0] = prueth->emac[PRUETH_MAC0]->ndev;
	}

	if (eth1_node) {
		ret = register_netdev(prueth->emac[PRUETH_MAC1]->ndev);
		if (ret) {
			dev_err(dev, "can't register netdev for port MII1");
			goto netdev_unregister;
		}

		prueth->registered_netdevs[PRUETH_MAC1] = prueth->emac[PRUETH_MAC1]->ndev;
	}

	prueth_init_timer(prueth);

	ret = prueth_register_notifiers(prueth);
	if (ret) {
		dev_err(dev, "can't register switchdev notifiers");
		goto netdev_unregister;
	}

	eth_random_addr(prueth->base_mac);

	dev_info(dev, "TI PRU ethernet driver initialized: %s EMAC mode\n",
		 (!eth0_node || !eth1_node) ? "single" : "dual");

	return 0;

netdev_unregister:
	for (i = 0; i < PRUETH_NUM_MACS; i++) {
		if (!prueth->registered_netdevs[i])
			continue;
		unregister_netdev(prueth->registered_netdevs[i]);
	}

ecap_put:
	if (!IS_ERR(prueth->ecap))
		prueth_ecap_put(prueth->ecap);
iep_put:
	icss_iep_put(prueth->iep);
netdev_exit:
	for (i = 0; i < PRUETH_NUM_MACS; i++) {
		struct device_node *eth_node;

		eth_node = prueth->eth_node[i];
		if (!eth_node)
			continue;

		prueth_netdev_exit(prueth, eth_node);
	}

free_pool:
	gen_pool_free(prueth->sram_pool,
		      (unsigned long)prueth->mem[PRUETH_MEM_OCMC].va, OCMC_RAM_SIZE);

put_mem:
	pruss_cfg_ocp_master_ports(prueth->pruss, 0);
	for (i = PRUETH_MEM_DRAM0; i < PRUETH_MEM_OCMC; i++) {
		if (prueth->mem[i].va)
			pruss_release_mem_region(pruss, &prueth->mem[i]);
	}

put_pruss:
	pruss_put(prueth->pruss);

put_pru1:
	if (eth1_node)
		pru_rproc_put(prueth->pru1);
put_pru0:
	if (eth0_node)
		pru_rproc_put(prueth->pru0);

put_node:
	of_node_put(eth1_node);
	of_node_put(eth0_node);

	return ret;
}

static int prueth_remove(struct platform_device *pdev)
{
	struct device_node *eth_node;
	struct prueth *prueth = platform_get_drvdata(pdev);
	int i;

	hrtimer_cancel(&prueth->tbl_check_timer);
	unregister_netdevice_notifier(&prueth->prueth_ndev_nb);
	prueth_sw_unregister_notifiers(prueth);

	for (i = 0; i < PRUETH_NUM_MACS; i++) {
		if (!prueth->registered_netdevs[i])
			continue;
		unregister_netdev(prueth->registered_netdevs[i]);
	}

	for (i = 0; i < PRUETH_NUM_MACS; i++) {
		eth_node = prueth->eth_node[i];
		if (!eth_node)
			continue;

		prueth_netdev_exit(prueth, eth_node);
		of_node_put(eth_node);
	}

	gen_pool_free(prueth->sram_pool,
		      (unsigned long)prueth->mem[PRUETH_MEM_OCMC].va,
		      OCMC_RAM_SIZE);

	for (i = PRUETH_MEM_DRAM0; i < PRUETH_MEM_OCMC; i++) {
		if (prueth->mem[i].va)
			pruss_release_mem_region(prueth->pruss, &prueth->mem[i]);
	}

	if (!IS_ERR(prueth->ecap))
		prueth_ecap_put(prueth->ecap);
	icss_iep_put(prueth->iep);

	pruss_put(prueth->pruss);

	if (prueth->eth_node[PRUETH_MAC0])
		pru_rproc_put(prueth->pru1);
	if (prueth->eth_node[PRUETH_MAC1])
		pru_rproc_put(prueth->pru0);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int prueth_suspend(struct device *dev)
{
	struct prueth *prueth = dev_get_drvdata(dev);
	struct net_device *ndev;
	int i, ret;

	for (i = 0; i < PRUETH_NUM_MACS; i++) {
		ndev = prueth->registered_netdevs[i];

		if (!ndev)
			continue;

		if (netif_running(ndev)) {
			netif_device_detach(ndev);
			ret = emac_ndo_stop(ndev);
			if (ret < 0) {
				netdev_err(ndev, "failed to stop: %d", ret);
				return ret;
			}
		}
	}

	pruss_cfg_ocp_master_ports(prueth->pruss, 0);

	return 0;
}

static int prueth_resume(struct device *dev)
{
	struct prueth *prueth = dev_get_drvdata(dev);
	struct net_device *ndev;
	int i, ret;

	pruss_cfg_ocp_master_ports(prueth->pruss, 1);

	for (i = 0; i < PRUETH_NUM_MACS; i++) {
		ndev = prueth->registered_netdevs[i];

		if (!ndev)
			continue;

		if (netif_running(ndev)) {
			ret = emac_ndo_open(ndev);
			if (ret < 0) {
				netdev_err(ndev, "failed to start: %d", ret);
				return ret;
			}
			netif_device_attach(ndev);
		}
	}

	return 0;
}

#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops prueth_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(prueth_suspend, prueth_resume)
};

/* AM33xx SoC-specific firmware data */
static struct prueth_private_data am335x_prueth_pdata = {
	.fw_pru[PRUSS_PRU0] = {
		.fw_name[PRUSS_ETHTYPE_EMAC] =
			"ti-pruss/am335x-pru0-prueth-fw.elf",
		.fw_name[PRUSS_ETHTYPE_HSR] =
			"ti-pruss/am335x-pru0-pruhsr-fw.elf",
		.fw_name[PRUSS_ETHTYPE_PRP] =
			"ti-pruss/am335x-pru0-pruprp-fw.elf"
	},
	.fw_pru[PRUSS_PRU1] = {
		.fw_name[PRUSS_ETHTYPE_EMAC] =
			"ti-pruss/am335x-pru1-prueth-fw.elf",
		.fw_name[PRUSS_ETHTYPE_HSR] =
			"ti-pruss/am335x-pru1-pruhsr-fw.elf",
		.fw_name[PRUSS_ETHTYPE_PRP] =
			"ti-pruss/am335x-pru1-pruprp-fw.elf"
	},
	.support_lre = true,
};

/* AM437x SoC-specific firmware data */
static struct prueth_private_data am437x_prueth_pdata = {
	.fw_pru[PRUSS_PRU0] = {
		.fw_name[PRUSS_ETHTYPE_EMAC] =
			"ti-pruss/am437x-pru0-prueth-fw.elf",
		.fw_name[PRUSS_ETHTYPE_HSR] =
			"ti-pruss/am437x-pru0-pruhsr-fw.elf",
		.fw_name[PRUSS_ETHTYPE_PRP] =
			"ti-pruss/am437x-pru0-pruprp-fw.elf"
	},
	.fw_pru[PRUSS_PRU1] = {
		.fw_name[PRUSS_ETHTYPE_EMAC] =
			"ti-pruss/am437x-pru1-prueth-fw.elf",
		.fw_name[PRUSS_ETHTYPE_HSR] =
			"ti-pruss/am437x-pru1-pruhsr-fw.elf",
		.fw_name[PRUSS_ETHTYPE_PRP] =
			"ti-pruss/am437x-pru1-pruprp-fw.elf"
	},
	.support_lre = true,
};

/* AM57xx SoC-specific firmware data */
static struct prueth_private_data am57xx_prueth_pdata = {
	.fw_pru[PRUSS_PRU0] = {
		.fw_name[PRUSS_ETHTYPE_EMAC] =
			"ti-pruss/am57xx-pru0-prueth-fw.elf",
		.fw_name[PRUSS_ETHTYPE_HSR] =
			"ti-pruss/am57xx-pru0-pruhsr-fw.elf",
		.fw_name[PRUSS_ETHTYPE_PRP] =
			"ti-pruss/am57xx-pru0-pruprp-fw.elf",
		.fw_name[PRUSS_ETHTYPE_SWITCH] =
			"ti-pruss/am57xx-pru0-prusw-fw.elf",
	},
	.fw_pru[PRUSS_PRU1] = {
		.fw_name[PRUSS_ETHTYPE_EMAC] =
			"ti-pruss/am57xx-pru1-prueth-fw.elf",
		.fw_name[PRUSS_ETHTYPE_HSR] =
			"ti-pruss/am57xx-pru1-pruhsr-fw.elf",
		.fw_name[PRUSS_ETHTYPE_PRP] =
			"ti-pruss/am57xx-pru1-pruprp-fw.elf",
		.fw_name[PRUSS_ETHTYPE_SWITCH] =
			"ti-pruss/am57xx-pru1-prusw-fw.elf",
	},
	.rev_2_1 = true,
	.support_lre = true,
	.support_switch = true,
};

/* 66AK2G SoC-specific firmware data */
static struct prueth_private_data k2g_prueth_pdata = {
	.fw_pru[PRUSS_PRU0] = {
		.fw_name[PRUSS_ETHTYPE_EMAC] =
			"ti-pruss/k2g-pru0-prueth-fw.elf",
	},
	.fw_pru[PRUSS_PRU1] = {
		.fw_name[PRUSS_ETHTYPE_EMAC] =
			"ti-pruss/k2g-pru1-prueth-fw.elf",
	},
	.rev_2_1 = true,
};

static const struct of_device_id prueth_dt_match[] = {
	{ .compatible = "ti,am57-prueth", .data = &am57xx_prueth_pdata, },
	{ .compatible = "ti,am4376-prueth", .data = &am437x_prueth_pdata, },
	{ .compatible = "ti,am3359-prueth", .data = &am335x_prueth_pdata, },
	{ .compatible = "ti,k2g-prueth", .data = &k2g_prueth_pdata, },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, prueth_dt_match);

static struct platform_driver prueth_driver = {
	.probe = prueth_probe,
	.remove = prueth_remove,
	.driver = {
		.name = "prueth",
		.of_match_table = prueth_dt_match,
		.pm = &prueth_dev_pm_ops,
	},
};
module_platform_driver(prueth_driver);

MODULE_AUTHOR("Roger Quadros <rogerq@ti.com>");
MODULE_AUTHOR("Andrew F. Davis <afd@ti.com>");
MODULE_DESCRIPTION("PRU Ethernet Driver");
MODULE_LICENSE("GPL v2");
