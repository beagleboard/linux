/*
 * Keystone NetCP PA2 (Packet Accelerator 2) Driver
 * PA2 is a newer version of the hardware available on K2E/L SoCs,
 *
 * Copyright (C) 2012-2015 Texas Instruments Incorporated
 * Author: Murali Karicheri <m-karicheri2@ti.com>
 *
 * Other contributors:	Sandeep Paulraj (Initial version of the driver)
 *			Reece Pollack (Maintenance)
 *			Sandeep Nair (Maintenance)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/delay.h>
#include <linux/errqueue.h>
#include <linux/firmware.h>
#include <linux/if_vlan.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/net_tstamp.h>
#include <linux/platform_device.h>
#include <linux/byteorder/generic.h>
#include <linux/soc/ti/knav_dma.h>
#include <linux/soc/ti/knav_qmss.h>
#include <linux/spinlock.h>

#include "netcp.h"
#include "netcp_pa_core.h"
#include "netcp_pa2_host.h"
#include "netcp_pa2_fw.h"

enum {
	PA2_CLUSTER_0 = 0,
	PA2_CLUSTER_1,
	PA2_CLUSTER_2,
	PA2_CLUSTER_3,
	PA2_CLUSTER_4,
	PA2_CLUSTER_5,
	PA2_CLUSTER_6,
	PA2_CLUSTER_7,
	PA2_CLUSTER_8,
	PA2_NUM_CLUSTERS
};

enum pa2_pdsp {
	PA2_PDSP0 = 0,
	PA2_PDSP1,
	PA2_PDSP2,
	PA2_PDSP3,
	PA2_PDSP4,
	PA2_PDSP5,
	PA2_PDSP6,
	PA2_PDSP7,
	PA2_PDSP8,
	PA2_PDSP9,
	PA2_PDSP10,
	PA2_PDSP11,
	PA2_PDSP12,
	PA2_PDSP13,
	PA2_PDSP14,
	PA2_NUM_PDSPS
};

/* PA Packet Processing Unit registers */
struct pa2_mailbox_regs {
	u32 pdsp_mailbox_slot0;
	u32 pdsp_mailbox_slot1;
	u32 pdsp_mailbox_slot2;
	u32 pdsp_mailbox_slot3;
};

struct pa2_ra_bridge_regs {
	u32 rsvd0;
	u32 config;
};

struct pa2_thread_mapper_regs {
	u32 map[16];
};

struct pa2_ra_heap_region_regs {
	u32 low;
	u32 high;
};

struct pa2_ra_flow_override_regs {
	u32 timeout;
	u32 critical_err;
	u32 non_critical_err;
	u32 rsvd0;
};

struct pa2_ra_stats_regs {
	u32 pkts_reasm;
	u32 total_frags;
	u32 total_pkts;
	u32 context_timeout_w_sop;
	u32 context_timeout_w_sop_bytes;
	u32 context_timeout_wo_sop;
	u32 context_timeout_wo_sop_bytes;
	u32 rsvd0[2];
	u32 overlap_ipv6_discard;
	u32 overlap_ipv6_discard_bytes;
	u32 large_pkts;
	u32 ipv4_tcp_err;
	u32 frag_len_err;
	u32 illegal_ipv4_ihl;
	u32 illegal_small_pkt;
	u32 illegal_frag_len;
	u32 already_completed_discard;
	u32 already_completed_discard_bytes;
	u32 rsvd1[5];
};

struct pa2_ra_regs {
	u32 revision;
	u32 config;
	u32 total_contexts;
	u32 discard_thresh;
	u32 timeout_val;
	u32 tick_val;
	u32 vbusm_config;
	u32 heap_region_thresh;
	struct pa2_ra_heap_region_regs heap_region[2];
	u32 rsvd0[4];
	struct pa2_ra_flow_override_regs flow_override[2];
	u32 rsvd1[12];
	u32 context_forced_timeout;
	u32 rsvd2[3];
	struct pa2_ra_stats_regs stats[2];
};

struct pa2_stats_ctl_regs {
	u32 revision;
	u32 soft_reset;
	u32 enable_alloc;
	u32 counter_update;
	u32 timer_ctl;
	u32 timer_load;
	u32 timer_val;
	u32 pkt_routing_info;
};

struct pa2_query_stats_regs {
	u32 stats[0x1000];
};

struct pa2_collect_stats_regs {
	u32 stats[0x1000];
};

struct pa2_cl_splitter_regs {
	u32 revision;
	u32 rsvd0[3];
	u32 sop_ctl;
	u32 eop_ctl;
	u32 rsvd1[2];
	u32 mop_buf_size;
	u32 mop_buf_ptr;
};

struct pa2_cluster {
	/* PA Cluster splitter regs, offset 0x9800 + cluster base */
	struct pa2_cl_splitter_regs __iomem *splitter;
	/* PA SRAM, offset 0x80000 + PPU base */
	void __iomem *sram;
};

struct pa2_ppu_ctl_status_regs {
	u32 control;
	u32 status;
	u32 wakeup_enable;
	u32 cycle_count;
	u32 stall_count;
	u32 rsvd[3];
	u32 const_tbl_blk_index0;
	u32 const_tbl_blk_index1;
	u32 const_tbl_prog_pointer0;
	u32 const_tbl_prog_pointer1;
};

struct pa2_ppu_debug_regs {
	u32 igp[32];	/* Internal General Purpose Register */
	u32 icte[32];	/* Internal Contants Table Entry Register */
};

struct pa2_ppu_cp_timer_regs {
	u32 timer_control;
	u32 timer_load;
	u32 timer_value;
	u32 timer_interrupt;
};

struct pa2_ppu_lut1_regs {
	u32 revision;
	u32 control;
	u32 config;
};

struct pa2_ppu_lut2_regs {
	u32 revision;
	u32 clr_table;
	u32 rsvd0[2];
	u32 max_entry_count;
	u32 curr_entry_count;
	u32 rsvd1[2];
	u32 add_data[4];
	u32 add_del_key[2];
	u32 add_del_control;
};

struct pa2_pcheck_recipe_regs {
	u32 control;
	u32 table[15];
};

struct pa2_ppu_pcheck_regs {
	u32 revision;
	u32 rsvd0[15];
	struct pa2_pcheck_recipe_regs recipe[4];
};

/* PA PPU registers, offset: 0x8000 + (pdsp num) * 0x10000 + cluster offset */
struct pa2_ppu {
	/* PPU PDSP control/status regs, offset 0x0000 + PPU base */
	struct pa2_ppu_ctl_status_regs __iomem	*ctl_status;
	/* PPU PDSP debug regs, offset 0x0400 + PPU base */
	struct pa2_ppu_debug_regs __iomem	*debug;
	/* PPU CP Timer regs, offset 0x0800 + PPU base */
	struct pa2_ppu_cp_timer_regs __iomem	*cp_timer;
	/* PPU LUT1 regs, offset 0x1000 + PPU base */
	struct pa2_ppu_lut1_regs __iomem	*lut1;
	/* PPU LUT2 regs, offset 0x1400 + PPU base */
	struct pa2_ppu_lut2_regs __iomem	*lut2;
	/* PPU pcheck regs, offset 0x1c00 + PPU base */
	struct pa2_ppu_pcheck_regs __iomem	*pcheck;
	void __iomem				*iram;
};

struct pa2_device {
	struct pa_core_device			core_dev;
	struct pa2_mailbox_regs __iomem		*reg_mailbox;
	struct pa2_ra_bridge_regs __iomem	*reg_ra_bridge;
	struct pa2_thread_mapper_regs __iomem	*reg_thread_mapper;
	struct pa2_ra_regs __iomem		*reg_ra;
	struct pa2_stats_ctl_regs   __iomem	*reg_stats_ctl;
	struct pa2_query_stats_regs   __iomem	*reg_stats_block;
	struct pa2_cluster	cluster[PA2_NUM_CLUSTERS];
	struct pa2_ppu		ppu[PA2_NUM_PDSPS];
};

#define PA2_MB_REG_INDEX			0
#define PA2_RA_BRIDGE_INDEX			1
#define PA2_TMAPPER_INDEX			2
#define PA2_RA_INDEX				3
#define PA2_STATS_CTL_INDEX			4
#define PA2_STATS_BLOCK_INDEX			5
#define PA2_CLUSTER_INDEX			6
#define PA2_STATS_CTL_ENABLE_ALLOC_MASK		BIT(31)

/* PA cluster registers */
#define PA2_CLUSTER_REGS_SIZE			0x00100000
#define PA2_CLUSTER_REGS(x)			(PA2_CLUSTER_REGS_SIZE * x)
#define PA2_CLUSTER_SPLITTER_REGS_OFFSET	0x09800
#define PA2_CLUSTER_SPLITTER_REGS(x)	(PA2_CLUSTER_SPLITTER_REGS_OFFSET \
					 + PA2_CLUSTER_REGS(x))
#define PA2_CLUSTER_SRAM_OFFSET			0x80000
#define PA2_CLUSTER_SRAM_SIZE			0x10000
#define PA2_CLUSTER_SRAM_REGS(x)	(PA2_CLUSTER_SRAM_OFFSET \
						 + PA2_CLUSTER_REGS(x))
#define PA2_CLUSTER_INGRESS0			PA2_CLUSTER_0
#define PA2_CLUSTER_INGRESS1			PA2_CLUSTER_1
#define PA2_CLUSTER_INGRESS2			PA2_CLUSTER_2
#define PA2_CLUSTER_INGRESS3			PA2_CLUSTER_3
#define PA2_CLUSTER_INGRESS4			PA2_CLUSTER_4
#define PA2_CLUSTER_POST			PA2_CLUSTER_5
#define PA2_CLUSTER_EGRESS0			PA2_CLUSTER_6
#define PA2_CLUSTER_EGRESS1			PA2_CLUSTER_7
#define PA2_CLUSTER_EGRESS2			PA2_CLUSTER_8

#define PA2_COMID_L2				(0 << 14)

/* PA Cluster Splitter
 * to avoid hardware bug: SOP + EOP + 32 + Control size <= 128
 * Restrict control size to 96, the SOP should be 896 - 128
 */
#define PA2_SPLITTER_SOP_CTL_ENABLE_MASK	0x80000000

#define PA2_CLUSTER_SPLITTER_EOP_CTL		128
#define PA2_CLUSTER_SPLITTER_EOP_BUF_SIZE(x)	((x == 0) ? 0x4000 : 0x10000)
#define PA2_CLUSTER_SPLITTER_MOP_BUF_PTR	0xFFFC0000
#define PA2_CLUSTER_SPLITTER_SOP_CTL \
	(PA2_SPLITTER_SOP_CTL_ENABLE_MASK | (896 - 128))
#define PA2_PPU_REGS_OFFSET			0x08000
#define PA2_PPU_REGS_SIZE			0x10000
#define PA2_PPU_REGS(x, y)		(PA2_CLUSTER_REGS(x) + \
					 (PA2_PPU_REGS_OFFSET + \
					 (PA2_PPU_REGS_SIZE * y)))
#define PA2_PPU_CTL_STATUS_REGS_OFFSET		0x0000
#define PA2_PPU_DEBUG_REGS_OFFSET		0x0400
#define PA2_PPU_CP_TIMER_REGS_OFFSET		0x0800
#define PA2_PPU_LUT1_REGS_OFFSET		0x1000
#define PA2_PPU_LUT2_REGS_OFFSET		0x1400
#define PA2_PPU_PCHECK_REGS_OFFSET		0x1c00
#define PA2_PPU_IRAM_OFFSET			0x4000
#define PA2_PPU_IRAM_SIZE			0x3000
#define PA2_INGRESS4_PDSP1			PA2_PDSP7
#define PA2_EGRESS0_PDSP2			PA2_PDSP12

struct pa2_cluster_pdsp_map {
	u32 cluster;
	u32 pdsp;
	u32 ver_base_addr;
};

#define PA2_PDSP_VERSION_SIZE			0x20
#define PA2_PDSP_VERSION_OFFSET(x, y)	(x + (y * PA2_PDSP_VERSION_SIZE))

/* PDSP/Cluster Mapping */
static const struct pa2_cluster_pdsp_map pa2_cluster_pdsp_map[PA2_NUM_PDSPS] = {
	{PA2_CLUSTER_INGRESS0, 0, 0x3f04},
	{PA2_CLUSTER_INGRESS0, 1, 0x3f04},
	{PA2_CLUSTER_INGRESS1, 0, 0x3f04},
	{PA2_CLUSTER_INGRESS1, 1, 0x3f04},
	{PA2_CLUSTER_INGRESS2, 0, 0x1f04},
	{PA2_CLUSTER_INGRESS3, 0, 0x1f04},
	{PA2_CLUSTER_INGRESS4, 0, 0x3f04},
	{PA2_CLUSTER_INGRESS4, 1, 0x3f04},
	{PA2_CLUSTER_POST, 0, 0x3f04},
	{PA2_CLUSTER_POST, 1, 0x3f04},
	{PA2_CLUSTER_EGRESS0, 0, 0x1f04},
	{PA2_CLUSTER_EGRESS0, 1, 0x1f04},
	{PA2_CLUSTER_EGRESS0, 2, 0x1f04},
	{PA2_CLUSTER_EGRESS1, 0, 0x0f04},
	{PA2_CLUSTER_EGRESS2, 0, 0x0f04}
};

#define PSTREAM_ROUTE_INGRESS0				2
#define PA2_PDSP_ALREADY_ACTIVE				0
#define PA2_PDSP_RESET_RELEASED				1
#define PA2_PDSP_NO_RESTART				2
#define PA2_MAX_PDSP_ENABLE_LOOP_COUNT			50000
#define PA2_INVALID_PORT				0xff
#define PA2_STATE_RESET			0  /* Sub-system state reset */
#define PA2_STATE_ENABLE		1  /* Sub-system state enable  */
#define PA2_STATE_QUERY			2  /* Query the Sub-system state */
#define PA2_STATE_INCONSISTENT		3  /* Sub-system is partially enabled */
#define PA2_STATE_INVALID_REQUEST	4  /* Invalid state command to the
					    * Sub-system
					    */
#define PA2_STATE_ENABLE_FAILED		5  /* The Sub-system did not respond
					    * after restart
					    */

/* pdsp LUT2 register */
#define PA2_REG_VAL_PDSP_LUT2_CLR_TABLE_GO		BIT(0)

/* pdsp control status register */
#define PA2_REG_VAL_PDSP_CTL_DISABLE_PDSP		1
#define PA2_REG_VAL_PDSP_CTL_RESET_PDSP			0
#define PA2_REG_VAL_PDSP_CTL_STATE			BIT(15)
#define PA2_REG_VAL_PDSP_CTL_ENABLE			BIT(1)
#define PA2_REG_VAL_PDSP_CTL_SOFT_RESET			BIT(0)
#define PA2_REG_VAL_PDSP_CTL_ENABLE_PDSP(pcval)		\
					(((pcval) << 16) | \
					PA2_REG_VAL_PDSP_CTL_ENABLE | \
					PA2_REG_VAL_PDSP_CTL_SOFT_RESET)
#define PA2_PDSP_CONST_NUM_REG				32
#define PA2_PCHECK_CONTROL_RSHIFT_MASK			(0x00000002u)
#define PA2_PCHECK_CONTROL_FINAL_NOT_MASK		(0x00000001u)

static const u32 pa2_ppu_regs_offset[PA2_NUM_PDSPS] = {
	PA2_PPU_REGS(PA2_CLUSTER_INGRESS0, 0),
	PA2_PPU_REGS(PA2_CLUSTER_INGRESS0, 1),
	PA2_PPU_REGS(PA2_CLUSTER_INGRESS1, 0),
	PA2_PPU_REGS(PA2_CLUSTER_INGRESS1, 1),
	PA2_PPU_REGS(PA2_CLUSTER_INGRESS2, 0),
	PA2_PPU_REGS(PA2_CLUSTER_INGRESS3, 0),
	PA2_PPU_REGS(PA2_CLUSTER_INGRESS4, 0),
	PA2_PPU_REGS(PA2_CLUSTER_INGRESS4, 1),
	PA2_PPU_REGS(PA2_CLUSTER_POST, 0),
	PA2_PPU_REGS(PA2_CLUSTER_POST, 1),
	PA2_PPU_REGS(PA2_CLUSTER_EGRESS0, 0),
	PA2_PPU_REGS(PA2_CLUSTER_EGRESS0, 1),
	PA2_PPU_REGS(PA2_CLUSTER_EGRESS0, 2),
	PA2_PPU_REGS(PA2_CLUSTER_EGRESS1, 0),
	PA2_PPU_REGS(PA2_CLUSTER_EGRESS2, 0)
};

static struct pa_core_ops *core_ops = &netcp_pa_core_ops;

/* PA2 firmware files */
static const char *pa2_pdsp_firmwares[PA2_NUM_PDSPS][PA_MAX_FIRMWARES] = {
	{ "ks2_pa_in0_pdsp0.bin", },	/*  0 */
	{ "ks2_pa_in0_pdsp1.bin", },	/*  1 */
	{ "ks2_pa_in1_pdsp0.bin", },	/*  2 */
	{ "ks2_pa_in1_pdsp1.bin", },	/*  3 */
	{ "ks2_pa_in2_pdsp0.bin", },	/*  4 */
	{ "ks2_pa_in3_pdsp0.bin", },	/*  5 */
	{ "ks2_pa_in4_pdsp0.bin", },	/*  6 */
	{ "ks2_pa_in4_pdsp1.bin", },	/*  7 */
	{ "ks2_pa_post_pdsp0.bin", },	/*  8 */
	{ "ks2_pa_post_pdsp1.bin", },	/*  9 */
	{ "ks2_pa_eg0_pdsp0.bin", },	/* 10 */
	{ "ks2_pa_eg0_pdsp1.bin", },	/* 11 */
	{ "ks2_pa_eg0_pdsp2.bin", },	/* 12 */
	{ "ks2_pa_eg1_pdsp0.bin", },	/* 13 */
	{ "ks2_pa_eg2_pdsp0.bin", },	/* 14 */
};

#define to_pa(data)	container_of(data, struct pa2_device, core_dev)

static int pa2_set_firmware(struct pa_core_device *core_dev,
			    int pdsp, const unsigned int *buffer, int len)
{
	struct pa2_device *pa_dev = to_pa(core_dev);
	struct pa2_ppu_debug_regs __iomem *debug_reg = pa_dev->ppu[pdsp].debug;
	char *version;
	int size_header = PA2_PDSP_CONST_NUM_REG + (PA_SIZE_VERSION >> 2);
	u32 i;

	if ((pdsp < 0) || (pdsp >= PA2_NUM_PDSPS))
		return -EINVAL;

	version = core_dev->version + (pdsp * PA_SIZE_VERSION);
	/* extract version from the start of the buffer */
	memcpy(version, buffer, PA_SIZE_VERSION);
	*(version + PA_SIZE_VERSION - 1) = '\0';

	if (len > PA2_PPU_IRAM_SIZE)
		return -ENODEV;

	/* offset of constants */
	buffer += (PA_SIZE_VERSION >> 2);

	for (i = 0; i < PA2_PDSP_CONST_NUM_REG; i++)
		writel(buffer[i], &debug_reg->icte[i]);

	buffer += PA2_PDSP_CONST_NUM_REG;

	core_ops->load_firmware((u32 *)(pa_dev->ppu[pdsp].iram), buffer,
				(len >> 2) - size_header);

	return 0;
}

static int pa2_pdsp_run(struct pa2_device *pa_dev, int pdsp)
{
	struct pa2_ppu_ctl_status_regs __iomem *ctrl_reg =
						pa_dev->ppu[pdsp].ctl_status;
	struct pa2_mailbox_regs __iomem *mailbox_reg =
						&pa_dev->reg_mailbox[pdsp];
	u32 i, v;

	/* Check for enabled PDSP */
	v = readl(&ctrl_reg->control);
	if ((v & PA2_REG_VAL_PDSP_CTL_ENABLE) ==
	    PA2_REG_VAL_PDSP_CTL_ENABLE) {
		/* Already enabled */
		return PA2_PDSP_ALREADY_ACTIVE;
	}

	/* Clear the mailbox */
	writel(0, &mailbox_reg->pdsp_mailbox_slot0);

	/* Set PDSP PC to 0, enable the PDSP */
	writel(PA2_REG_VAL_PDSP_CTL_ENABLE | PA2_REG_VAL_PDSP_CTL_SOFT_RESET,
	       &ctrl_reg->control);

	/* Wait for the mailbox to become non-zero */
	for (i = 0; i < PA2_MAX_PDSP_ENABLE_LOOP_COUNT; i++) {
		v = readl(&mailbox_reg->pdsp_mailbox_slot0);
		if (v != 0)
			return PA2_PDSP_RESET_RELEASED;
	}

	return PA2_PDSP_NO_RESTART;
}

static void pa2_reset(struct pa2_device *pa_dev)
{
	int i;

	/* Put each of the PDSPs into reset (PC = 0) and reset timers */
	for (i = 0; i < PA2_NUM_PDSPS; i++)  {
		writel(0, &pa_dev->ppu[i].ctl_status->control);
		writel(0, &pa_dev->ppu[i].cp_timer->timer_control);
	}

	/* Reset LUT2 */
	writel(PA2_REG_VAL_PDSP_LUT2_CLR_TABLE_GO,
	       &pa_dev->ppu[PA2_INGRESS4_PDSP1].lut2->clr_table);
}

static int pa2_enable(struct pa2_device *pa_dev)
{
	int i, res, ret  = PA2_STATE_ENABLE;

	/* Do nothing if a pdsp is already out of reset.
	 * If any PDSPs are out of reset
	 * a global init is not performed
	 */
	for (i = 0; i < PA2_NUM_PDSPS; i++) {
		res = pa2_pdsp_run(pa_dev, i);

		if (res == PA2_PDSP_NO_RESTART)
			ret = PA2_STATE_ENABLE_FAILED;
	}

	for (i = 0; i < PA2_NUM_PDSPS; i++) {
		struct pa2_mailbox_regs __iomem *mbox_reg =
				&pa_dev->reg_mailbox[i];
		writel(0, &mbox_reg->pdsp_mailbox_slot0);
	}

	return ret;
}

static int pa2_post_init(struct pa_core_device *core_dev)
{
	struct pa2_device *pa_dev = to_pa(core_dev);
	struct device *dev = core_dev->dev;
	u32 version, i;
	char *str;
	int ret = pa2_enable(pa_dev);

	if (ret != PA2_STATE_ENABLE) {
		dev_err(dev, "enable failed, ret = %d\n", ret);
		return -ENODEV;
	}

	for (i = 0; i < PA2_NUM_PDSPS; i++) {
		u32 cluster = pa2_cluster_pdsp_map[i].cluster;
		void __iomem *sram = pa_dev->cluster[cluster].sram;
		u32 base = pa2_cluster_pdsp_map[i].ver_base_addr;
		u32 pdsp = pa2_cluster_pdsp_map[i].pdsp;

		version = readl(sram +
				PA2_PDSP_VERSION_OFFSET(base, pdsp));
		str =  pa_dev->core_dev.version + (i * PA_SIZE_VERSION);
		dev_info(dev, "Using PA fw version %s:0x%08x for pdsp %d\n",
			 str, version, i);
	}

	return 0;
}

static void pa2_pre_init(struct pa_core_device *core_dev)
{
	struct pa2_device *pa_dev = to_pa(core_dev);
	int i;

	/* System Statistics initialization */
	writel(PA2_STATS_CTL_ENABLE_ALLOC_MASK,
	       &pa_dev->reg_stats_ctl->enable_alloc);
	writel(1, &pa_dev->reg_stats_ctl->soft_reset);

	/* Initialize all clusters */
	for (i = 0; i < PA2_NUM_CLUSTERS; i++) {
		writel(PA2_CLUSTER_SPLITTER_EOP_CTL,
		       &pa_dev->cluster[i].splitter->eop_ctl);
		writel(PA2_CLUSTER_SPLITTER_EOP_BUF_SIZE(i),
		       &pa_dev->cluster[i].splitter->mop_buf_size);
		writel(PA2_CLUSTER_SPLITTER_MOP_BUF_PTR,
		       &pa_dev->cluster[i].splitter->mop_buf_ptr);
		writel(PA2_CLUSTER_SPLITTER_SOP_CTL,
		       &pa_dev->cluster[i].splitter->sop_ctl);
	}
	pa2_reset(pa_dev);
}

static inline void swizfwd(struct pa2_frm_forward *fwd)
{
	fwd->queue = cpu_to_be16(fwd->queue);
	if (fwd->forward_type == PA2FRM_FORWARD_TYPE_HOST)
		fwd->u.host.context = cpu_to_be32(fwd->u.host.context);
}

static inline int swizal1(struct pa2_frm_cmd_add_lut1 *al1)
{
	al1->index		=  cpu_to_be16(al1->index);
	al1->vlink_num		=  cpu_to_be16(al1->vlink_num);
	al1->stats_index	=  cpu_to_be16(al1->stats_index);

	if (al1->type == PA2FRM_COM_ADD_LUT1_STANDARD) {
		al1->u.mac.etype	= cpu_to_be16(al1->u.mac.etype);
		al1->u.mac.session_id	= cpu_to_be16(al1->u.mac.session_id);
		al1->u.mac.mpls		= cpu_to_be32(al1->u.mac.mpls);
		al1->u.mac.vlan_id1	= cpu_to_be16(al1->u.mac.vlan_id1);
		al1->u.mac.vlan_id2	= cpu_to_be16(al1->u.mac.vlan_id2);
		al1->u.mac.vlan_pri1	= cpu_to_be16(al1->u.mac.vlan_pri1);
		al1->u.mac.vlan_pri2	= cpu_to_be16(al1->u.mac.vlan_pri2);
		al1->u.mac.src_vc	= cpu_to_be16(al1->u.mac.src_vc);
	} else if (al1->type == PA2FRM_COM_ADD_LUT1_CUSTOM) {
		al1->u.custom.src_vc	=  cpu_to_be16(al1->u.custom.src_vc);
	} else {
		return -EINVAL;
	}

	al1->range1_hi	=  cpu_to_be16(al1->range1_hi);
	al1->range0_hi	=  cpu_to_be16(al1->range0_hi);
	al1->cbwords0	=  cpu_to_be32(al1->cbwords0);
	al1->cbwords1	=  cpu_to_be32(al1->cbwords1);
	al1->bit_mask	=  cpu_to_be16(al1->bit_mask);
	al1->priority	=  cpu_to_be16(al1->priority);

	swizfwd(&al1->match);
	swizfwd(&al1->next_fail);

	return 0;
}

static inline void swizfcmd(struct pa2_frm_command *fcmd)
{
	fcmd->command_result	=  cpu_to_be16(fcmd->command_result);
	fcmd->com_id		=  cpu_to_be16(fcmd->com_id);
	fcmd->ret_context	=  cpu_to_be32(fcmd->ret_context);
	fcmd->reply_queue	=  cpu_to_be16(fcmd->reply_queue);
}

#define	PA2_CONTEXT_MASK	0xffff0000
#define	PA2_CONTEXT_CONFIG	0xdead0000

static void pa2_rx_packet_handler(void *param)
{
	struct pa_packet *p_info = param;
	struct pa_core_device *core_dev = p_info->core_dev;
	struct pa2_frm_command *fcmd;

	switch (p_info->epib[1] & PA2_CONTEXT_MASK) {
	case PA2_CONTEXT_CONFIG:
		fcmd = p_info->data;
		swizfcmd(fcmd);

		if (fcmd->command_result != PA2FRM_COMMAND_RESULT_SUCCESS) {
			dev_dbg(core_dev->dev, "Command Result = 0x%x\n",
				fcmd->command_result);
			dev_dbg(core_dev->dev, "Command = 0x%x\n",
				fcmd->command);
			dev_dbg(core_dev->dev, "Magic = 0x%x\n", fcmd->magic);
			dev_dbg(core_dev->dev, "Com ID = 0x%x\n", fcmd->com_id);
			dev_dbg(core_dev->dev, "ret Context = 0x%x\n",
				fcmd->ret_context);
			dev_dbg(core_dev->dev, "Flow ID = 0x%x\n",
				fcmd->flow_id);
			dev_dbg(core_dev->dev, "reply Queue = 0x%x\n",
				fcmd->reply_queue);
			dev_dbg(core_dev->dev, "reply dest = 0x%x\n",
				fcmd->reply_dest);
		}
		dev_dbg(core_dev->dev, "command response complete\n");
		break;

	default:
		dev_warn(core_dev->dev, "bad response context, got 0x%08x\n",
			 p_info->epib[1]);
		break;
	}
}

static struct
pa2_frm_command *pa2_format_fcmd_hdr(void *p_cmd,
				     struct pa_core_device *priv,
				     u8 cmd, u16 com_id,
				     u8 first_pdsp, u16 cmd_size)
{
	struct pa2_frm_command *fcmd;

	memset(p_cmd, 0, cmd_size);
	fcmd			= (struct pa2_frm_command *)p_cmd;
	fcmd->status		= PA2FRM_CFG_CMD_STATUS_PROC;
	fcmd->pdsp_index	= first_pdsp;
	fcmd->command		= cmd;
	fcmd->magic		= PA2FRM_CONFIG_COMMAND_SEC_BYTE;
	fcmd->com_id		= com_id;
	fcmd->ret_context	= PA2_CONTEXT_CONFIG;
	fcmd->flow_id		= priv->cmd_flow_num;
	fcmd->reply_queue	= priv->cmd_queue_num;
	fcmd->reply_dest	= PA2FRM_DEST_PKTDMA;

	return fcmd;
}

static int pa2_conv_fc_routing_info(struct pa2_frm_forward *fwd_info,
				    struct pa2_ef_op_info *ef_info)
{
	if (!ef_info)
		return PA2_ERR_CONFIG;

	fwd_info->forward_type = PA2FRM_FORWARD_TYPE_EFLOW;

	if (ef_info->ctrl_flags & PA2_EF_OP_CONTROL_FLAG_FC_LOOKUP)
		/* Trigger Flow cache operation */
		fwd_info->u.ef.ctrl_flags = PA2FRM_EF_CTRL_FC_LOOKUP;
	else {
		/* Use Egress Flow records directly */
		fwd_info->u.ef.valid_bitmap  = (u8)ef_info->valid_bitmap;
		fwd_info->u.ef.lvl1_rec_idx = (u8)ef_info->lvl1_index;
		fwd_info->u.ef.lvl2_rec_idx = (u8)ef_info->lvl2_index;
		fwd_info->u.ef.lvl3_rec_idx = (u8)ef_info->lvl3_index;
		fwd_info->u.ef.lvl4_rec_idx = (u8)ef_info->lvl4_index;
	}

	return 0;
}

static int pa2_conv_routing_info(struct pa2_frm_forward *fwd_info,
				 struct pa2_route_info2 *route_info,
				 int cmd_dest, u16 fail_route,
				 u16 dest_pdsp, u8 pa_flags)
{
	u8 *pcmd = fwd_info->u.host.cmd;
	u8 ps_flags = 0;
	u32 no_fcmd = 0;

	fwd_info->flow_id = route_info->flow_id;
	fwd_info->queue   = route_info->queue;

	if ((route_info->dest == PA2_DEST_HOST) ||
	    (route_info->dest == PA2_DEST_EMAC)) {
		if (route_info->valid_bitmap &
		    PA2_ROUTE_INFO_VALID_PKTTYPE_EMAC) {
			ps_flags = (route_info->pkt_type_emac_ctrl &
				    PA2_EMAC_CTRL_CRC_DISABLE) ?
				    PA2FRM_ETH_PS_FLAGS_DISABLE_CRC : 0;
			ps_flags |= ((route_info->pkt_type_emac_ctrl &
				      PA2_EMAC_CTRL_PORT_MASK) <<
				     PA2FRM_ETH_PS_FLAGS_PORT_SHIFT);
		}
	}

	if (route_info->dest == PA2_DEST_HOST) {
		fwd_info->forward_type   = PA2FRM_FORWARD_TYPE_HOST;
		fwd_info->u.host.context = route_info->sw_info_0;
		fwd_info->u.host.ps_flags = ps_flags;

		if (route_info->valid_bitmap &
		    PA2_ROUTE_INFO_VALID_PRIORITY_TYPE) {
			if (route_info->priority_type ==
				PA2_ROUTE_PRIORITY_VLAN)
				fwd_info->u.host.ctrl_bitmap |=
					PA2FRM_ROUTING_PRIORITY_VLAN_ENABLE;
			else if (route_info->priority_type ==
				PA2_ROUTE_PRIORITY_DSCP)
				fwd_info->u.host.ctrl_bitmap |=
					PA2FRM_ROUTING_PRIORITY_DSCP_ENABLE;
			else if (route_info->priority_type == PA2_ROUTE_INTF)
				fwd_info->u.host.ctrl_bitmap |=
					PA2FRM_ROUTING_IF_DEST_SELECT_ENABLE;
			else if (route_info->priority_type ==
				 PA2_ROUTE_INTF_W_FLOW)
				fwd_info->u.host.ctrl_bitmap |=
					(PA2FRM_ROUTING_IF_DEST_SELECT_ENABLE |
					 PA2FRM_ROUTING_FLOW_IF_BASE_ENABLE);
			else
				return PA2_ERR_CONFIG;
		}

		if (route_info->valid_bitmap &
		    PA2_ROUTE_INFO_VALID_MROUTEINDEX) {
			if (route_info->m_route_index >= 0) {
				if (route_info->m_route_index >=
					PA2_MAX_MULTI_ROUTE_SETS)
					return PA2_ERR_CONFIG;
				fwd_info->u.host.ctrl_bitmap |=
					PA2FRM_MULTIROUTE_ENABLE;
				fwd_info->u.host.multi_idx =
					route_info->m_route_index;
				fwd_info->u.host.pa_pdsp_router	=
					PA2FRM_DEST_PA_M_0;
			}
		}
	} else if (route_info->dest == PA2_DEST_DISCARD)	{
		fwd_info->forward_type = PA2FRM_FORWARD_TYPE_DISCARD;
	} else if (route_info->dest == PA2_DEST_EMAC) {
		fwd_info->forward_type = PA2FRM_FORWARD_TYPE_ETH;
		fwd_info->u.eth.ps_flags = ps_flags;
	} else if (fail_route) {
		return PA2_ERR_CONFIG;

	} else if (((route_info->dest == PA2_DEST_CONTINUE_PARSE_LUT1) &&
		    (route_info->custom_type != PA2_CUSTOM_TYPE_LUT2)) ||
		   ((route_info->dest == PA2_DEST_CONTINUE_PARSE_LUT2) &&
		    (route_info->custom_type != PA2_CUSTOM_TYPE_LUT1))) {
		/* Custom Error check */
		if (((route_info->custom_type == PA2_CUSTOM_TYPE_LUT1) &&
		     (route_info->custom_index >= PA2_MAX_CUSTOM_TYPES_LUT1)) ||
		    ((route_info->custom_type == PA2_CUSTOM_TYPE_LUT2) &&
		     (route_info->custom_index >= PA2_MAX_CUSTOM_TYPES_LUT2)))
			return PA2_ERR_CONFIG;

		fwd_info->forward_type = PA2FRM_FORWARD_TYPE_PA;
		fwd_info->u.pa.custom_type = (u8)route_info->custom_type;
		fwd_info->u.pa.custom_idx  = route_info->custom_index;
		fwd_info->u.pa.flags  = pa_flags;

		if (route_info->dest == PA2_DEST_CONTINUE_PARSE_LUT2) {
			fwd_info->u.pa.pa_dest = PA2FRM_DEST_INGRESS4;
		} else {
			/* cmd_dest is provided by calling function
			 * There is no need to check error case
			 */
			if (cmd_dest == PA2_CMD_TX_DEST_0) {
				/* Layer 2 entry */
				fwd_info->u.pa.pa_dest = PA2FRM_DEST_INGRESS1;
			} else if (cmd_dest == PA2_CMD_TX_DEST_1) {
				fwd_info->u.pa.pa_dest = (dest_pdsp == 0) ?
				  PA2FRM_DEST_INGRESS1 : PA2FRM_DEST_INGRESS3;
				if (route_info->custom_type ==
						PA2_CUSTOM_TYPE_LUT1)
					fwd_info->u.pa.pa_dest =
						PA2FRM_DEST_INGRESS3;
			} else if (cmd_dest == PA2_CMD_TX_DEST_3) {
				fwd_info->u.pa.pa_dest = PA2FRM_DEST_INGRESS4;
			} else {
				return PA2_ERR_CONFIG;
			}
		}
		no_fcmd = 1;
	} else if (route_info->dest == PA2_DEST_CASCADED_FORWARDING_LUT1) {
		fwd_info->forward_type = PA2FRM_FORWARD_TYPE_PA;
		fwd_info->u.pa.pa_dest = (cmd_dest == PA2_CMD_TX_DEST_0) ?
				PA2FRM_DEST_INGRESS1 : PA2FRM_DEST_INGRESS4;
		fwd_info->u.pa.flags   |= PA2FRM_CASCADED_FORWARDING;
		no_fcmd = 1;
	} else if (route_info->dest == PA2_DEST_EFLOW) {
		return pa2_conv_fc_routing_info(fwd_info,
						route_info->ef_op);
	} else {
		return PA2_ERR_CONFIG;
	}

	if (pcmd && (route_info->valid_bitmap & PA2_ROUTE_INFO_VALID_PCMD)) {
		struct pa2_cmd_info *pacmd = route_info->pcmd;
		struct pa2_patch_info *patch_info;
		struct pa2_cmd_set *cmd_set;
		struct pa2_cmd_usr_stats *usr_stats;
		struct pa2_cmd_set_usr_stats *cmd_set_usr_stats;

		switch (pacmd->cmd) {
		case PA2_CMD_PATCH_DATA:
			patch_info = &pacmd->params.patch;
			if ((patch_info->n_patch_bytes > 2) ||
			    (!(patch_info->ctrl_bit_field &
			    PA2_PATCH_OP_INSERT)) ||
				(!patch_info->patch_data))
				return PA2_ERR_CONFIG;

			pcmd[0] = PA2FRM_RX_CMD_PATCH_DATA;
			pcmd[1] = patch_info->n_patch_bytes;
			pcmd[2] = patch_info->patch_data[0];
			pcmd[3] = patch_info->patch_data[1];
			break;

		case PA2_CMD_CMDSET:
			cmd_set = &pacmd->params.cmd_set;
			if (no_fcmd || (cmd_set->index >= PA2_MAX_CMD_SETS))
				return PA2_ERR_CONFIG;

			pcmd[0] = PA2FRM_RX_CMD_CMDSET;
			pcmd[1] = (u8)cmd_set->index;
			break;

		case PA2_CMD_USR_STATS:
			usr_stats = &pacmd->params.usr_stats;
			if (usr_stats->index >= 512)
				return PA2_ERR_CONFIG;

			pcmd[0] = PA2FRM_RX_CMD_USR_STATS;
			pcmd[1] = 4;
			pcmd[2] = usr_stats->index >> 8;
			pcmd[3] = usr_stats->index & 0xFF;
			break;

		case PA2_CMD_CMDSET_AND_USR_STATS:
			cmd_set_usr_stats = &pacmd->params.cmd_set_usr_stats;
			if ((no_fcmd) ||
			    (cmd_set_usr_stats->set_index >=
			    PA2_MAX_CMD_SETS) ||
			    (cmd_set_usr_stats->stats_index >= 512))
				return PA2_ERR_CONFIG;

			pcmd[0] = PA2FRM_RX_CMD_CMDSET_USR_STATS;
			pcmd[1] = (u8)cmd_set_usr_stats->set_index;
			pcmd[2] = cmd_set_usr_stats->stats_index >> 8;
			pcmd[3] = cmd_set_usr_stats->stats_index & 0xFF;
			break;

		default:
			return PA2_ERR_CONFIG;
		}
	}
	return PA2_OK;
}

static int pa2_add_mac_rule(struct pa_intf *pa_intf, int index,
			    const u8 *smac, const u8 *dmac, int rule,
			    unsigned etype, int port)
{
	struct pa_core_device *core_dev = pa_intf->core_dev;
	struct pa2_route_info2 route_info, fail_info;
	struct pa2_frm_command *fcmd;
	struct pa2_frm_cmd_add_lut1 *al1;
	u16 priority, bit_mask = 0;
	struct pa_packet *tx;
	u32 cbwords0, cbwords1;
	int size, ret;

	dev_dbg(core_dev->dev,
		"add mac, index %d, smac %pM, dmac %pM, rule %d,",
		index, smac, dmac, rule);
	dev_dbg(core_dev->dev,
		"type %04x, port %d\n", etype, port);

	memset(&fail_info, 0, sizeof(fail_info));
	memset(&route_info, 0, sizeof(route_info));

	if (rule == PACKET_HST) {
		route_info.dest			= PA2_DEST_HOST;
		route_info.flow_id		= pa_intf->data_flow_num;
		route_info.queue		= pa_intf->data_queue_num;
		route_info.m_route_index	= PA2_NO_MULTI_ROUTE;
		fail_info.dest			= PA2_DEST_HOST;
		fail_info.flow_id		= pa_intf->data_flow_num;
		fail_info.queue			= pa_intf->data_queue_num;
		fail_info.m_route_index		= PA2_NO_MULTI_ROUTE;
	} else if (rule == PACKET_PARSE) {
		route_info.dest			= PA2_DEST_CONTINUE_PARSE_LUT1;
		route_info.m_route_index	= PA2_NO_MULTI_ROUTE;
		fail_info.dest			= PA2_DEST_HOST;
		fail_info.flow_id		= pa_intf->data_flow_num;
		fail_info.queue			= pa_intf->data_queue_num;
		fail_info.m_route_index		= PA2_NO_MULTI_ROUTE;
	} else if (rule == PACKET_DROP) {
		route_info.dest			= PA2_DEST_DISCARD;
		route_info.m_route_index	= PA2_NO_MULTI_ROUTE;
		fail_info.dest			= PA2_DEST_DISCARD;
		fail_info.m_route_index		= PA2_NO_MULTI_ROUTE;
	}

	if (route_info.m_route_index != PA2_NO_MULTI_ROUTE)
		route_info.valid_bitmap |= PA2_ROUTE_INFO_VALID_MROUTEINDEX;
	if (route_info.pkt_type_emac_ctrl)
		route_info.valid_bitmap |= PA2_ROUTE_INFO_VALID_PKTTYPE_EMAC;
	if (route_info.pcmd)
		route_info.valid_bitmap |= PA2_ROUTE_INFO_VALID_PCMD;
	if (fail_info.m_route_index != PA2_NO_MULTI_ROUTE)
		fail_info.valid_bitmap |= PA2_ROUTE_INFO_VALID_MROUTEINDEX;
	if (fail_info.pkt_type_emac_ctrl)
		fail_info.valid_bitmap |= PA2_ROUTE_INFO_VALID_PKTTYPE_EMAC;
	if (fail_info.pcmd)
		fail_info.valid_bitmap |= PA2_ROUTE_INFO_VALID_PCMD;

	size = (sizeof(struct pa2_frm_command) +
		sizeof(struct pa2_frm_cmd_add_lut1) - sizeof(u32));
	tx = core_ops->alloc_packet(core_dev, size, PA2_CLUSTER_0);
	if (!tx) {
		dev_err(core_dev->dev, "could not allocate cmd tx packet\n");
		return -ENOMEM;
	}

	fcmd = pa2_format_fcmd_hdr((void *)tx->data,
				   core_dev,
				   PA2FRM_CONFIG_COMMAND_ADDREP_LUT1,
				   PA2_COMID_L2,
				   0,
				   size);

	al1		= (struct pa2_frm_cmd_add_lut1 *)&fcmd->cmd;
	al1->index	= index;
	al1->type	= PA2FRM_COM_ADD_LUT1_STANDARD;

	cbwords0	= PA2FRM_LUT1_CLASS_STANDARD << PA2FRM_LUT1_CLASS_SHIFT;
	cbwords1	= PA2FRM_LUT1_VALID_PKTTYPE;
	priority	= 0;

	al1->u.mac.pkt_type = PA2FRM_L2_PKT_TYPE_MAC;

	if (etype) {
		al1->u.mac.etype = etype;
		cbwords0 |=  PA2FRM_LUT1_VALID_ETHERTYPE;
		priority += 10;
	}

	al1->u.mac.vlan_id1 = 0;
	al1->u.mac.mpls	= 0;
	if (port) {
		al1->u.mac.in_port = port;
		cbwords1 |=  PA2FRM_LUT1_VALID_INPORT;
		priority += 10;
	}

	if (dmac) {
		memcpy(al1->u.mac.dmac, dmac, 6);
		cbwords0 |= PA2FRM_LUT1_VALID_DMAC_ALL;
		priority += 10;
	}
	if (smac) {
		memcpy(al1->u.mac.smac, smac, 6);
		cbwords0 |= PA2FRM_LUT1_VALID_SMAC;
		priority += 10;
	}

	al1->cbwords0 = cbwords0;
	al1->cbwords1 = cbwords1;
	al1->priority = priority;
	al1->bit_mask = bit_mask;

	ret = pa2_conv_routing_info(&al1->match, &route_info, 0, 0, 0, 0);
	if (ret != 0)
		dev_err(core_dev->dev, "route info config failed\n");

	ret = pa2_conv_routing_info(&al1->next_fail, &fail_info, 0, 1, 0, 0);
	if (ret != 0)
		dev_err(core_dev->dev, "fail info config failed\n");

	swizfcmd(fcmd);
	ret = swizal1((struct pa2_frm_cmd_add_lut1 *)&fcmd->cmd);
	if (ret < 0)
		return ret;

	tx->psdata[0] = PAHO2_PACFG_CMD;

	tx->epib[1] = 0;
	tx->epib[2] = 0;
	tx->epib[3] = 0;

	dev_dbg(core_dev->dev, "waiting for command transmit complete\n");

	return core_ops->submit_packet(tx, PA2_CLUSTER_0);
}

static int pa2_fmtcmd_next_route(struct netcp_packet *p_info, int eth_port)
{
	u8 ps_flags = (eth_port & PA2_EMAC_CTRL_PORT_MASK) <<
				PA2FRM_ETH_PS_FLAGS_PORT_SHIFT;
	struct paho_next_route *nr;

	nr = (struct paho_next_route *)netcp_push_psdata(p_info, sizeof(*nr));
	if (!nr)
		return -ENOMEM;

	/* Construct word0 */
	nr->word0 = 0;
	PAHO_SET_CMDID(nr, PAHO_PAMOD_NROUTE);
	PAHO_SET_E(nr, 1);
	PAHO_SET_DEST(nr, PA2FRM_DEST_ETH);
	PAHO_SET_FLOW(nr, 0);
	PAHO_SET_QUEUE(nr, 0);

	/* Construct sw_info0 and sw_info1 */
	nr->sw_info0 = 0;
	nr->sw_info1 = 0;

	/* Construct word1 */
	nr->word1 = 0;
	PAHO_SET_PKTTYPE(nr, ps_flags);

	return sizeof(*nr);
}

static int pa2_fmtcmd_tx_csum(struct netcp_packet *p_info)
{
	struct sk_buff *skb = p_info->skb;
	struct paho_com_chk_crc *ptx;
	int start, len;
	int size;

	size = sizeof(*ptx);
	ptx = (struct paho_com_chk_crc *)netcp_push_psdata(p_info, size);

	start = skb_checksum_start_offset(skb);
	len = skb->len - start;

	ptx->word0 = 0;
	ptx->word1 = 0;
	ptx->word2 = 0;
	PAHO_SET_CMDID(ptx, PAHO_PAMOD_CMPT_CHKSUM);
	PAHO_CHKCRC_SET_START(ptx, start);
	PAHO_CHKCRC_SET_LEN(ptx, len);
	PAHO_CHKCRC_SET_RESULT_OFF(ptx, skb->csum_offset);
	PAHO_CHKCRC_SET_INITVAL(ptx, 0);
	PAHO_CHKCRC_SET_NEG0(ptx, 0);

	return size;
}

static inline int pa2_fmtcmd_align(struct netcp_packet *p_info,
				   const unsigned bytes)
{
	struct paho_cmd_info	*pa_cmd_info;
	int i;

	if ((bytes & 0x03) != 0)
		return -EINVAL;

	pa_cmd_info =
		(struct paho_cmd_info *)netcp_push_psdata(p_info, bytes);

	for (i = bytes / sizeof(u32); i > 0; --i) {
		pa_cmd_info->word0 = 0;
		PAHO_SET_CMDID(pa_cmd_info, PAHO_PAMOD_DUMMY);
		++pa_cmd_info;
	}

	return bytes;
}

static u32 pa2_get_streaming_switch(struct pa_core_device *core_dev, int port)
{
	struct pa2_device *pa_dev = to_pa(core_dev);
	void __iomem *thread_map = pa_dev->reg_thread_mapper;
	u32 reg, offset = 0;

	/* each port has 8 priorities, which needs 8 bytes setting */
	if (port > 0)
		offset = (port - 1) * 8;

	reg = readl(thread_map + offset) & 0xff;
	return reg;
}

static u32 pa2_set_streaming_switch(struct pa_core_device *core_dev,
				    int port, u32 new_value)
{
	struct pa2_device *pa_dev = to_pa(core_dev);
	void __iomem *thread_map = pa_dev->reg_thread_mapper;
	u32 reg, offset, old_value;
	int i;

	reg = (new_value << 24) | (new_value << 16) |
		(new_value << 8) | (new_value);

	if (port == 0) {
		/* return 1st port priority 0 setting for all the ports */
		old_value = readl(thread_map);
		old_value &= 0xff;
		for (i = 0; i < 16; i++, thread_map += 4)
			writel(reg, thread_map);
	} else {
		/* each port has 8 priorities, which needs 8 bytes setting */
		offset = (port - 1) * 8;

		/* return priority 0 setting for the port */
		old_value = readl(thread_map + offset);
		old_value &= 0xff;
		writel(reg, thread_map + offset);
		writel(reg, thread_map + offset + 4);
	}

	return old_value;
}

static void pa2_unmap_resources(struct pa_core_device *core_dev)
{
	struct pa2_device *pa_dev = to_pa(core_dev);
	int i;

	if (pa_dev->reg_mailbox)
		iounmap(pa_dev->reg_mailbox);
	if (pa_dev->reg_ra_bridge)
		iounmap(pa_dev->reg_ra_bridge);
	if (pa_dev->reg_thread_mapper)
		iounmap(pa_dev->reg_thread_mapper);
	if (pa_dev->reg_ra)
		iounmap(pa_dev->reg_ra);
	if (pa_dev->reg_stats_ctl)
		iounmap(pa_dev->reg_stats_ctl);
	if (pa_dev->reg_stats_block)
		iounmap(pa_dev->reg_stats_block);

	for (i = 0; i < PA2_NUM_CLUSTERS; i++) {
		if (pa_dev->cluster[i].splitter)
			iounmap(pa_dev->cluster[i].splitter);
		if (pa_dev->cluster[i].sram)
			iounmap(pa_dev->cluster[i].sram);
	}

	for (i = 0; i < PA2_NUM_PDSPS; i++) {
		if (pa_dev->ppu[i].ctl_status)
			iounmap(pa_dev->ppu[i].ctl_status);
		if (pa_dev->ppu[i].debug)
			iounmap(pa_dev->ppu[i].debug);
		if (pa_dev->ppu[i].cp_timer)
			iounmap(pa_dev->ppu[i].cp_timer);
		if (pa_dev->ppu[i].lut1)
			iounmap(pa_dev->ppu[i].lut1);
		if (pa_dev->ppu[i].lut2)
			iounmap(pa_dev->ppu[i].lut2);
		if (pa_dev->ppu[i].pcheck)
			iounmap(pa_dev->ppu[i].pcheck);
		if (pa_dev->ppu[i].iram)
			iounmap(pa_dev->ppu[i].iram);
	}
}

static int pa2_map_resources(struct pa_core_device *core_dev,
			     struct device_node *node)
{
	struct pa2_device *pa_dev = to_pa(core_dev);
	struct device *dev = core_dev->dev;
	struct resource res;
	int i, ret = -ENODEV;

	pa_dev->reg_mailbox =
		core_ops->map_resource(dev, node, PA2_MB_REG_INDEX);
	if (!pa_dev->reg_mailbox)
		return ret;

	pa_dev->reg_ra_bridge =
		core_ops->map_resource(dev, node, PA2_RA_BRIDGE_INDEX);
	if (!pa_dev->reg_ra_bridge)
		goto unmap;

	pa_dev->reg_thread_mapper =
		core_ops->map_resource(dev, node, PA2_TMAPPER_INDEX);
	if (!pa_dev->reg_thread_mapper)
		goto unmap;

	pa_dev->reg_ra = core_ops->map_resource(dev, node, PA2_RA_INDEX);
	if (!pa_dev->reg_ra)
		goto unmap;

	pa_dev->reg_stats_ctl =
		core_ops->map_resource(dev, node, PA2_STATS_CTL_INDEX);
	if (!pa_dev->reg_stats_ctl)
		goto unmap;

	pa_dev->reg_stats_block =
		core_ops->map_resource(dev, node, PA2_STATS_BLOCK_INDEX);
	if (!pa_dev->reg_stats_block)
		goto unmap;

	ret = of_address_to_resource(node, PA2_CLUSTER_INDEX, &res);
	if (ret) {
		dev_err(core_dev->dev,
			"Can't xlate pa2 cluster node at %d\n",
			PA2_CLUSTER_INDEX);
		goto unmap;
	}

	for (i = 0; i < PA2_NUM_CLUSTERS; i++) {
		pa_dev->cluster[i].splitter =
			ioremap_nocache(res.start +
					PA2_CLUSTER_SPLITTER_REGS(i),
					sizeof(struct pa2_cl_splitter_regs));
		if (!pa_dev->cluster[i].splitter)
			goto unmap;

		pa_dev->cluster[i].sram =
			ioremap_nocache(res.start + PA2_CLUSTER_SRAM_REGS(i),
					PA2_CLUSTER_SRAM_SIZE);
		if (!pa_dev->cluster[i].sram)
			goto unmap;
	}

	for (i = 0; i < PA2_NUM_PDSPS; i++) {
		pa_dev->ppu[i].ctl_status =
			ioremap_nocache(res.start + pa2_ppu_regs_offset[i] +
					PA2_PPU_CTL_STATUS_REGS_OFFSET,
					sizeof(struct pa2_ppu_ctl_status_regs));
		if (!pa_dev->ppu[i].ctl_status)
			goto unmap;

		pa_dev->ppu[i].debug =
			ioremap_nocache(res.start + pa2_ppu_regs_offset[i] +
					PA2_PPU_DEBUG_REGS_OFFSET,
					sizeof(struct pa2_ppu_debug_regs));
		if (!pa_dev->ppu[i].debug)
			goto unmap;

		pa_dev->ppu[i].cp_timer =
			ioremap_nocache(res.start + pa2_ppu_regs_offset[i] +
					PA2_PPU_CP_TIMER_REGS_OFFSET,
					sizeof(struct pa2_ppu_cp_timer_regs));
		if (!pa_dev->ppu[i].cp_timer)
			goto unmap;

		pa_dev->ppu[i].lut1 =
			ioremap_nocache(res.start + pa2_ppu_regs_offset[i] +
					PA2_PPU_LUT1_REGS_OFFSET,
					sizeof(struct pa2_ppu_lut1_regs));
		if (!pa_dev->ppu[i].lut1)
			goto unmap;

		pa_dev->ppu[i].lut2 =
			ioremap_nocache(res.start + pa2_ppu_regs_offset[i] +
					PA2_PPU_LUT2_REGS_OFFSET,
					sizeof(struct pa2_ppu_lut2_regs));
		if (!pa_dev->ppu[i].lut2)
			goto unmap;

		pa_dev->ppu[i].pcheck =
			ioremap_nocache(res.start + pa2_ppu_regs_offset[i] +
					PA2_PPU_PCHECK_REGS_OFFSET,
					sizeof(struct pa2_ppu_pcheck_regs));
		if (!pa_dev->ppu[i].pcheck)
			goto unmap;

		pa_dev->ppu[i].iram =
			ioremap_nocache(res.start + pa2_ppu_regs_offset[i] +
					PA2_PPU_IRAM_OFFSET, PA2_PPU_IRAM_SIZE);
		if (!pa_dev->ppu[i].iram)
			goto unmap;
	}

	return 0;

unmap:	pa2_unmap_resources(core_dev);
	return ret;
}

static struct pa_hw netcp_pa2_hw = {
	.fmtcmd_next_route = pa2_fmtcmd_next_route,
	.fmtcmd_tx_csum = pa2_fmtcmd_tx_csum,
	.fmtcmd_align = pa2_fmtcmd_align,
	.num_clusters = PA2_NUM_CLUSTERS,
	.num_pdsps = PA2_NUM_PDSPS,
	.ingress_l2_cluster_id = PA2_CLUSTER_0,
	.ingress_l3_cluster_id = PA2_CLUSTER_1,
	.egress_cluster_id = PA2_CLUSTER_6,
	.streaming_pdsp = PSTREAM_ROUTE_INGRESS0,
	.map_resources	= pa2_map_resources,
	.unmap_resources = pa2_unmap_resources,
	.pre_init = pa2_pre_init,
	.post_init = pa2_post_init,
	.set_firmware = pa2_set_firmware,
	.rx_packet_handler = pa2_rx_packet_handler,
	.add_mac_rule = pa2_add_mac_rule,
	.set_streaming_switch = pa2_set_streaming_switch,
	.get_streaming_switch = pa2_get_streaming_switch,
};

static int pa2_probe(struct netcp_device *netcp_device,
		     struct device *dev,
		     struct device_node *node,
		     void **inst_priv)
{
	int ret, size = sizeof(struct pa2_device);
	struct pa2_device *pa_dev;

	if (!node) {
		dev_err(dev, "device tree info unavailable\n");
		return -ENODEV;
	}

	pa_dev = (struct pa2_device *)core_ops->init(netcp_device, dev, node,
						     size, &ret, &netcp_pa2_hw,
						     pa2_pdsp_firmwares);
	if (!pa_dev)
		return ret;
	*inst_priv = pa_dev;

	return ret;
}

static int pa2_ioctl(void *intf_priv, struct ifreq *req, int cmd)
{
	return -EOPNOTSUPP;
}

static struct netcp_module pa2_module = {
	.name		= "netcp-pa2",
	.owner		= THIS_MODULE,
	.probe		= pa2_probe,
	.close		= pa_core_close,
	.open		= pa_core_open,
	.remove		= pa_core_remove,
	.attach		= pa_core_attach,
	.release	= pa_core_release,
	.add_addr	= pa_core_add_addr,
	.del_addr	= pa_core_del_addr,
	.ioctl		= pa2_ioctl,
};

static int __init netcp_pa2_init(void)
{
	return netcp_register_module(&pa2_module);
}
module_init(netcp_pa2_init);

static void __exit netcp_pa2_exit(void)
{
	netcp_unregister_module(&pa2_module);
}
module_exit(netcp_pa2_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Murali Karicheri m-karicheri2@ti.com>");
MODULE_DESCRIPTION("NetCP Packet Accelerator 2 driver for Keystone devices");
