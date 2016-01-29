/*
 * Keystone NetCP PA (Packet Accelerator) Driver
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
#include <linux/clocksource.h>
#include <linux/delay.h>
#include <linux/errqueue.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/net_tstamp.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/byteorder/generic.h>
#include <linux/soc/ti/knav_dma.h>
#include <linux/soc/ti/knav_qmss.h>

#include "netcp.h"
#include "netcp_pa_core.h"
#include "netcp_pa_host.h"
#include "netcp_pa_fw.h"

#define PSTREAM_ROUTE_PDSP0			0
#define PA_PDSP_ALREADY_ACTIVE			0
#define PA_PDSP_RESET_RELEASED			1
#define PA_PDSP_NO_RESTART			2
#define PA_MAX_PDSP_ENABLE_LOOP_COUNT	100000
/* Sub-system state enable  */
#define PA_STATE_ENABLE				1
/*  The Sub-system did not respond after restart */
#define PA_STATE_ENABLE_FAILED			3
#define PAFRM_SRAM_SIZE				0x2000
#define PAFRM_SYS_TIMESTAMP_ADDR		0x6460
/* PDSP Versions */
#define PAFRM_PDSP_VERSION_BASE			0x7F04
#define DEVICE_PA_REGION_SIZE			0x48000
#define PA_MEM_PDSP_IRAM(pdsp)			((pdsp) * 0x8000)
#define PA_MEM_PDSP_SRAM(num)			((num) * 0x2000)
#define PA_PDSP_CONST_REG_INDEX_C25_C24		0
#define PA_PDSP_CONST_REG_INDEX_C27_C26		1
#define PA_PDSP_CONST_REG_INDEX_C29_C28		2
#define PA_PDSP_CONST_REG_INDEX_C31_C30		3

/* The pdsp control register bits */
#define PA_REG_VAL_PDSP_CTL_RESET_PDSP	        0
#define PA_REG_VAL_PDSP_CTL_STATE		BIT(15)
#define PA_REG_VAL_PDSP_CTL_ENABLE		BIT(1)
#define PA_REG_VAL_PDSP_CTL_SOFT_RESET		BIT(0)

/* register indices in DT bindings */
#define PA_MB_REG_INDEX				0
#define PA_PACKET_ID_REG_INDEX			1
#define PA_LUT2_REG_INDEX			2
#define PA_SS_INDEX				3
#define PA_CONTROL_REG_INDEX			4
#define PA_TIMER_REG_INDEX			5
#define PA_STATS_REG_INDEX			6
#define PA_IRAM_INDEX				7
#define PA_SRAM_INDEX				8

#define PA_CONTEXT_MASK				0xffff0000
#define PA_CONTEXT_CONFIG			0xdead0000
#define	PA_CONTEXT_TSTAMP			0xbeef0000
#define	TSTAMP_TIMEOUT	(HZ * 5)	/* 5 seconds (arbitrary) */

static struct pa_core_ops *core_ops = &netcp_pa_core_ops;

enum clusters {
	PA_CLUSTER_0,
	PA_CLUSTER_1,
	PA_CLUSTER_2,
	PA_CLUSTER_3,
	PA_CLUSTER_4,
	PA_CLUSTER_5,
	PA_NUM_CLUSTERS
};

#define PA_NUM_CONSTANTS			4
#define PA_NUM_PDSPS				6
/* firmwares used for various PDSPs. This list is to be kept
 * with latest firmware first followed by older firmares
 */
static const char *pa_pdsp_firmwares[PA_NUM_PDSPS][PA_MAX_FIRMWARES] = {
	{ "ks2_pa_pdsp0_classify1.bin", },
	{ "ks2_pa_pdsp1_classify1.bin", },
	{ "ks2_pa_pdsp2_classify1.bin", },
	{ "ks2_pa_pdsp3_classify2.bin", },
	{ "ks2_pa_pdsp4_pam.bin", },
	{ "ks2_pa_pdsp5_pam.bin", },
};

struct pa_mailbox_regs {
	u32 pdsp_mailbox_slot0;
	u32 pdsp_mailbox_slot1;
	u32 pdsp_mailbox_slot2;
	u32 pdsp_mailbox_slot3;
};

struct pa_packet_id_alloc_regs {
	u32 revision;
	u32 soft_reset;
	u32 range_limit;
	u32 idvalue;
};

struct pa_lut2_control_regs {
	u32 revision;
	u32 soft_reset;
	u32 rsvd[6];
	u32 add_data0;
	u32 add_data1;
	u32 add_data2;
	u32 add_data3;
	u32 add_del_key;
	u32 add_del_control;
};

struct pa_pdsp_control_regs {
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
	u32 rsvd1[52];
};

struct pa_pdsp_timer_regs {
	u32 timer_control;
	u32 timer_load;
	u32 timer_value;
	u32 timer_interrupt;
	u32 rsvd[60];
};

struct pa_statistics_regs {
	u32 revision;
	u32 soft_reset;
	u32 incr_flags;
	u32 stats_capture;
	u32 rsvd[4];
	u32 stats_red[32];
};

struct pa_device {
	struct pa_core_device	core_dev;
	struct pa_mailbox_regs __iomem		*reg_mailbox;
	struct pa_packet_id_alloc_regs __iomem	*reg_packet_id;
	struct pa_lut2_control_regs __iomem	*reg_lut2;
	struct pa_pdsp_control_regs __iomem	*reg_control;
	struct pa_pdsp_timer_regs   __iomem	*reg_timer;
	struct pa_statistics_regs   __iomem	*reg_stats;
	void __iomem	*pa_sram;
	void __iomem	*pa_iram;
	void __iomem	*streaming_switch;
};

#define to_pa(data)	container_of(data, struct pa_device, core_dev)

static inline void swiz_fwd(struct pa_frm_forward *fwd)
{
	fwd->queue = cpu_to_be16(fwd->queue);
	if (fwd->forward_type == PAFRM_FORWARD_TYPE_HOST)
		fwd->u.host.context = cpu_to_be32(fwd->u.host.context);
}

static inline void swizfcmd(struct pa_frm_command *fcmd)
{
	fcmd->command_result = cpu_to_be32(fcmd->command_result);
	fcmd->com_id = cpu_to_be16(fcmd->com_id);
	fcmd->ret_context = cpu_to_be32(fcmd->ret_context);
	fcmd->reply_queue = cpu_to_be16(fcmd->reply_queue);
}

static inline void swizal1(struct pa_frm_cmd_add_lut1 *al1)
{
	if (al1->type == PAFRM_COM_ADD_LUT1_STANDARD) {
		al1->u.eth_ip.etype = cpu_to_be16(al1->u.eth_ip.etype);
		al1->u.eth_ip.vlan = cpu_to_be16(al1->u.eth_ip.vlan);
		al1->u.eth_ip.spi = cpu_to_be32(al1->u.eth_ip.spi);
		al1->u.eth_ip.flow = cpu_to_be32(al1->u.eth_ip.flow);

		if (al1->u.eth_ip.key & PAFRM_LUT1_KEY_MPLS) {
			al1->u.eth_ip.pm.mpls =
				cpu_to_be32(al1->u.eth_ip.pm.mpls);
		} else {
			al1->u.eth_ip.pm.ports[0] =
				cpu_to_be16(al1->u.eth_ip.pm.ports[0]);
			al1->u.eth_ip.pm.ports[1] =
				cpu_to_be16(al1->u.eth_ip.pm.ports[1]);
		}
		al1->u.eth_ip.match_flags =
				cpu_to_be16(al1->u.eth_ip.match_flags);
	} else {
		al1->u.custom.etype = cpu_to_be16(al1->u.custom.etype);
		al1->u.custom.vlan = cpu_to_be16(al1->u.custom.vlan);
		al1->u.custom.match_flags =
				cpu_to_be16(al1->u.custom.match_flags);
	}

	swiz_fwd(&al1->match);
	swiz_fwd(&al1->next_fail);
}

static int pa_conv_routing_info(struct pa_frm_forward *fwd_info,
				struct pa_route_info *route_info,
				int cmd_dest, u16 fail_route)
{
	u8 *pcmd = NULL;

	fwd_info->flow_id = route_info->flow_id;
	fwd_info->queue = route_info->queue;

	if (route_info->dest == PA_DEST_HOST) {
		fwd_info->forward_type   = PAFRM_FORWARD_TYPE_HOST;
		fwd_info->u.host.context = route_info->sw_info_0;

		if (route_info->route_type)
			fwd_info->u.host.ctrl_bm |=
				PAFRM_ROUTING_IF_DEST_SELECT_ENABLE;

		if (route_info->route_type == PA_ROUTE_INTF_FLOW)
			fwd_info->u.host.ctrl_bm |=
				PAFRM_ROUTING_FLOW_IF_BASE_ENABLE;

		if (route_info->m_route_index >= 0) {
			if (route_info->m_route_index >=
			    PA_MAX_MULTI_ROUTE_SETS)
				return PA_ERR_CONFIG;

			fwd_info->u.host.ctrl_bm |= PAFRM_MULTIROUTE_ENABLE;
			fwd_info->u.host.multi_idx = route_info->m_route_index;
			fwd_info->u.host.pa_pdsp_router	= PAFRM_DEST_PA_M_0;
		}
		pcmd = fwd_info->u.host.cmd;
	} else if (route_info->dest == PA_DEST_DISCARD)	{
		fwd_info->forward_type = PAFRM_FORWARD_TYPE_DISCARD;
	} else if (fail_route) {
		return PA_ERR_CONFIG;

	} else if (((route_info->dest == PA_DEST_CONTINUE_PARSE_LUT1) &&
		    (route_info->custom_type != PA_CUSTOM_TYPE_LUT2)) ||
		   ((route_info->dest == PA_DEST_CONTINUE_PARSE_LUT2) &&
		    (route_info->custom_type != PA_CUSTOM_TYPE_LUT1))) {
		/* Custom Error check */
		if (((route_info->custom_type == PA_CUSTOM_TYPE_LUT1) &&
		     (route_info->custom_index >= PA_MAX_CUSTOM_TYPES_LUT1)) ||
		    ((route_info->custom_type == PA_CUSTOM_TYPE_LUT2) &&
		     (route_info->custom_index >= PA_MAX_CUSTOM_TYPES_LUT2)))
			return PA_ERR_CONFIG;

		fwd_info->forward_type = PAFRM_FORWARD_TYPE_PA;
		fwd_info->u.pa.custom_type = (u8)route_info->custom_type;
		fwd_info->u.pa.custom_idx  = route_info->custom_index;

		if (route_info->dest == PA_DEST_CONTINUE_PARSE_LUT2) {
			fwd_info->u.pa.pa_dest = PAFRM_DEST_PA_C2;
		} else {
			/* cmd_dest is provided by calling function
			 * There is no need to check error case
			 */
			fwd_info->u.pa.pa_dest =
				(cmd_dest == PA_CMD_TX_DEST_0) ?
				PAFRM_DEST_PA_C1_1 : PAFRM_DEST_PA_C1_2;
		}
	} else {
		return PA_ERR_CONFIG;
	}

	if (pcmd && route_info->pcmd) {
		struct pa_cmd_info *pacmd = route_info->pcmd;
		struct pa_patch_info *patch_info;
		struct pa_cmd_set *cmd_set;

		switch (pacmd->cmd) {
		case PA_CMD_PATCH_DATA:
			patch_info = &pacmd->params.patch;
			if ((patch_info->n_patch_bytes > 2) ||
			    (patch_info->overwrite) ||
			    (!patch_info->patch_data))
				return PA_ERR_CONFIG;

			pcmd[0] = PAFRM_RX_CMD_CMDSET;
			pcmd[1] = patch_info->n_patch_bytes;
			pcmd[2] = patch_info->patch_data[0];
			pcmd[3] = patch_info->patch_data[1];
			break;

		case PA_CMD_CMDSET:
			cmd_set = &pacmd->params.cmd_set;
			if (cmd_set->index >= PA_MAX_CMD_SETS)
				return PA_ERR_CONFIG;

			pcmd[0] = PAFRM_RX_CMD_CMDSET;
			pcmd[1] = (u8)cmd_set->index;
			break;
		default:
			return PA_ERR_CONFIG;
		}
	}
	return PA_OK;
}

static void pa_reset(struct pa_core_device *core_dev)
{
	struct pa_device *pa_dev = to_pa(core_dev);
	struct pa_packet_id_alloc_regs
		__iomem *packet_id_regs = pa_dev->reg_packet_id;
	struct pa_lut2_control_regs
		__iomem	*lut2_regs = pa_dev->reg_lut2;
	struct pa_statistics_regs
		__iomem	*stats_regs = pa_dev->reg_stats;
	/* per experiment it takes about 6 msec to reset PDSPS  */
	int count = 10;
	u32 i, val;

	/* Reset and disable all PDSPs */
	for (i = 0; i < PA_NUM_PDSPS; i++) {
		struct pa_pdsp_control_regs __iomem *ctrl_reg =
					&pa_dev->reg_control[i];

		writel(PA_REG_VAL_PDSP_CTL_RESET_PDSP,
		       &ctrl_reg->control);

		while (count--) {
			val = readl(&ctrl_reg->control) &
				    PA_REG_VAL_PDSP_CTL_STATE;
			if (val)
				usleep_range(100, 1000);
			else
				break;
		}
	}
	WARN((count == 0), "PA reset failure\n");

	/* Reset packet Id */
	writel(1, &packet_id_regs->soft_reset);

	/* Reset LUT2 */
	writel(1, &lut2_regs->soft_reset);

	/* Reset statistic */
	writel(1, &stats_regs->soft_reset);

	/* Reset timers */
	for (i = 0; i < PA_NUM_PDSPS; i++) {
		struct pa_pdsp_timer_regs __iomem *timer_reg =
					&pa_dev->reg_timer[i];

		writel(0, &timer_reg->timer_control);
	}
}

/* Convert a raw PA timer count to nanoseconds
 */
static inline u64 tstamp_raw_to_ns(struct pa_core_device *core_dev, u32 lo,
				   u32 hi)
{
	u32 mult = core_dev->timestamp_info.mult;
	u32 shift = core_dev->timestamp_info.shift;
	u64 result;

	/* Minimize overflow errors by doing this in pieces */
	result  = ((u64)lo * mult) >> shift;
	result += ((u64)hi << (32 - shift)) * mult;

	return result;
}

static int pa_pdsp_run(struct pa_device *pa_dev, int pdsp)
{
	struct pa_pdsp_control_regs __iomem *ctrl_reg =
				&pa_dev->reg_control[pdsp];
	struct pa_mailbox_regs __iomem *mailbox_reg =
				&pa_dev->reg_mailbox[pdsp];
	u32 i, v;

	/* Check for enabled PDSP */
	v = readl(&ctrl_reg->control);
	if ((v & PA_REG_VAL_PDSP_CTL_ENABLE) == PA_REG_VAL_PDSP_CTL_ENABLE) {
		/* Already enabled */
		return PA_PDSP_ALREADY_ACTIVE;
	}

	/* Clear the mailbox */
	writel(0, &mailbox_reg->pdsp_mailbox_slot0);

	/* Set PDSP PC to 0, enable the PDSP */
	writel(PA_REG_VAL_PDSP_CTL_ENABLE | PA_REG_VAL_PDSP_CTL_SOFT_RESET,
	       &ctrl_reg->control);

	/* Wait for the mailbox to become non-zero */
	for (i = 0; i < PA_MAX_PDSP_ENABLE_LOOP_COUNT; i++) {
		v = readl(&mailbox_reg->pdsp_mailbox_slot0);
		if (v != 0)
			return PA_PDSP_RESET_RELEASED;
	}

	return PA_PDSP_NO_RESTART;
}

static int pa_enable(struct pa_device *pa_dev)
{
	struct pa_mailbox_regs __iomem *mailbox_reg = &pa_dev->reg_mailbox[0];
	int do_global_reset = 1, i, res, ret =  PA_STATE_ENABLE;

	/* Do nothing if a pdsp is already out of reset.
	 * If any PDSPs are out of reset
	 * a global init is not performed
	 */
	for (i = 0; i < PA_NUM_PDSPS; i++) {
		res = pa_pdsp_run(pa_dev, i);

		if (res == PA_PDSP_ALREADY_ACTIVE)
			do_global_reset = 0;

		if (res == PA_PDSP_NO_RESTART) {
			ret = PA_STATE_ENABLE_FAILED;
			do_global_reset = 0;
		}
	}

	/* If global reset is required any PDSP can do it */
	if (do_global_reset) {
		int count = 5; /* found by experiment */

		writel(1, &mailbox_reg->pdsp_mailbox_slot1);
		writel(0, &mailbox_reg->pdsp_mailbox_slot0);

		while (count-- &&
		       (readl(&mailbox_reg->pdsp_mailbox_slot1) != 0))
			usleep_range(100, 1000);

		WARN((count == 0), "PA reset failure\n");

		for (i = 1; i < PA_NUM_PDSPS; i++) {
			struct pa_mailbox_regs __iomem *mbox_reg =
				&pa_dev->reg_mailbox[i];

			writel(0, &mbox_reg->pdsp_mailbox_slot0);
		}
	} else {
		for (i = 0; i < PA_NUM_PDSPS; i++) {
			struct pa_mailbox_regs __iomem *mbox_reg =
				&pa_dev->reg_mailbox[i];

			writel(0, &mbox_reg->pdsp_mailbox_slot0);
		}
	}

	return ret;
}

static int pa_post_init(struct pa_core_device *core_dev)
{
	struct pa_pdsp_timer_regs __iomem *timer_reg;
	struct pa_device *pa_dev = to_pa(core_dev);
	struct device *dev = pa_dev->core_dev.dev;
	int i, version, ret;
	char *str;

	ret = pa_enable(pa_dev);
	if (ret != PA_STATE_ENABLE) {
		dev_err(dev, "%s: enable failed, ret = %d\n", __func__, ret);
		return -ENODEV;
	}

	version = readl(pa_dev->pa_sram + PAFRM_PDSP_VERSION_BASE);
	for (i = 0; i < core_dev->hw->num_pdsps; i++) {
		str = core_dev->version + (i * PA_SIZE_VERSION);
		dev_info(dev, "Using PA fw version %s:0x%08x on pdsp %d\n",
			 str, version, i);
	}

	timer_reg = &pa_dev->reg_timer[0];
	/* Start PDSP timer at a prescaler of divide by 2 */
	writel(0xffff, &timer_reg->timer_load);
	writel((PA_SS_TIMER_CNTRL_REG_GO | PA_SS_TIMER_CNTRL_REG_MODE |
	       PA_SS_TIMER_CNTRL_REG_PSE |
	       (0 << PA_SS_TIMER_CNTRL_REG_PRESCALE_SHIFT)),
	       &timer_reg->timer_control);

	return 0;
}

static int pa_set_firmware(struct pa_core_device *core_dev,
			   int pdsp, const unsigned int *buffer,
			   int len)
{
	struct pa_device *pa_dev = to_pa(core_dev);
	struct pa_pdsp_control_regs __iomem *ctrl_reg =
						&pa_dev->reg_control[pdsp];
	int size_header = PA_NUM_CONSTANTS + (PA_SIZE_VERSION >> 2);
	char *version;

	if ((pdsp < 0) || (pdsp >= PA_NUM_PDSPS))
		return -EINVAL;

	version = core_dev->version + (pdsp * PA_SIZE_VERSION);
	/* extract version from the top of the buffer */
	memcpy(version, buffer, PA_SIZE_VERSION);
	*(version + PA_SIZE_VERSION - 1) = '\0';

	/* offset of constants */
	buffer += (PA_SIZE_VERSION >> 2);

	writel(buffer[PA_PDSP_CONST_REG_INDEX_C25_C24],
	       &ctrl_reg->const_tbl_blk_index0);

	writel(buffer[PA_PDSP_CONST_REG_INDEX_C27_C26],
	       &ctrl_reg->const_tbl_blk_index1);

	writel(buffer[PA_PDSP_CONST_REG_INDEX_C29_C28],
	       &ctrl_reg->const_tbl_prog_pointer0);

	writel(buffer[PA_PDSP_CONST_REG_INDEX_C31_C30],
	       &ctrl_reg->const_tbl_prog_pointer1);

	buffer += PA_NUM_CONSTANTS;
	core_ops->load_firmware((pa_dev->pa_iram + PA_MEM_PDSP_IRAM(pdsp)),
				buffer, (len >> 2) - size_header);
	return 0;
}

struct tstamp_pending {
	struct list_head list;
	u32 context;
	struct sk_buff *skb;
	struct pa_device *pa_dev;
	struct timer_list timeout;
};

static spinlock_t tstamp_lock;
static atomic_t	tstamp_sequence = ATOMIC_INIT(0);
static struct list_head tstamp_pending = LIST_HEAD_INIT(tstamp_pending);

static struct tstamp_pending *tstamp_remove_pending(u32 context)
{
	struct tstamp_pending	*pend;

	spin_lock(&tstamp_lock);
	list_for_each_entry(pend, &tstamp_pending, list) {
		if (pend->context == context) {
			del_timer(&pend->timeout);
			list_del(&pend->list);
			spin_unlock(&tstamp_lock);
			return pend;
		}
	}
	spin_unlock(&tstamp_lock);

	return NULL;
}

static void tstamp_complete(u32 context, struct pa_packet *p_info)
{
	struct skb_shared_hwtstamps *shhwtstamps;
	struct pa_core_device *core_dev;
	struct tstamp_pending *pend;
	struct pa_device *pa_dev;
	struct sk_buff *skb;
	u64 pa_ns;

	pend = tstamp_remove_pending(context);
	if (!pend)
		return;

	pa_dev = pend->pa_dev;
	core_dev = &pa_dev->core_dev;
	skb = pend->skb;

	shhwtstamps = skb_hwtstamps(skb);
	memset(shhwtstamps, 0, sizeof(*shhwtstamps));
	if (!p_info) {
		dev_warn(core_dev->dev, "%s: Timestamp completion timeout\n",
			 __func__);
	} else {
		pa_ns = tstamp_raw_to_ns(core_dev,
					 p_info->epib[0], p_info->epib[2]);
		shhwtstamps->hwtstamp = ns_to_ktime(pa_ns);
	}
	skb_complete_tx_timestamp(skb, shhwtstamps);
	kfree(pend);
}

static void pa_tstamp_purge_pending(struct pa_core_device *core_dev)
{
	struct tstamp_pending *pend;
	struct pa_device *pa_dev = to_pa(core_dev);
	bool found;

	do {
		found = false;
		spin_lock(&tstamp_lock);
		list_for_each_entry(pend, &tstamp_pending, list) {
			if (pend->pa_dev == pa_dev) {
				found = true;
				break;
			}
		}
		spin_unlock(&tstamp_lock);
		if (found)
			tstamp_complete(pend->context, NULL);
	} while (found);
}

static void tstamp_timeout(unsigned long context)
{
	tstamp_complete((u32)context, NULL);
}

static void tstamp_add_pending(struct tstamp_pending *pend)
{
	init_timer(&pend->timeout);
	pend->timeout.expires = jiffies + TSTAMP_TIMEOUT;
	pend->timeout.function = tstamp_timeout;
	pend->timeout.data = (unsigned long)pend->context;

	spin_lock(&tstamp_lock);
	add_timer(&pend->timeout);
	list_add_tail(&pend->list, &tstamp_pending);
	spin_unlock(&tstamp_lock);
}

static void pa_rx_packet_handler(void *param)
{
	struct pa_packet *p_info = param;
	struct pa_core_device *core_dev = p_info->core_dev;
	struct pa_frm_command *fcmd;

	switch (p_info->hwdesc->epib[1] & PA_CONTEXT_MASK) {
	case PA_CONTEXT_CONFIG:
		fcmd = p_info->data;
		swizfcmd(fcmd);

		if (fcmd->command_result != PAFRM_COMMAND_RESULT_SUCCESS) {
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

	case PA_CONTEXT_TSTAMP:
		tstamp_complete(p_info->epib[1], p_info);
		break;

	default:
		dev_warn(core_dev->dev,
			 "%s: bad response context, got 0x%08x\n",
			 __func__, p_info->epib[1]);
		break;
	}
}

static void
pa_format_cmd_hdr(struct pa_device *pa_dev,
		  struct pa_frm_command *fcmd, u8 cmd, u16 cmd_id, u32 ctx)
{
	struct pa_core_device *core_dev = &pa_dev->core_dev;

	memset(fcmd, 0, sizeof(*fcmd));
	fcmd->command = cmd;
	fcmd->magic = PAFRM_CONFIG_COMMAND_SEC_BYTE;
	fcmd->com_id = cpu_to_be16(cmd_id);
	fcmd->ret_context = cpu_to_be32(ctx);
	fcmd->flow_id = core_dev->cmd_flow_num;
	fcmd->reply_queue = cpu_to_be16(core_dev->cmd_queue_num);
	fcmd->reply_dest = PAFRM_DEST_PKTDMA;
}

static int pa_add_ip_proto(struct pa_core_device *core_dev, int index,
			   u8 proto, int rule)
{
	struct pa_device *pa_dev = to_pa(core_dev);
	struct pa_route_info route_info, fail_info;
	struct pa_frm_cmd_add_lut1 *al1;
	u32 context = PA_CONTEXT_CONFIG;
	struct pa_frm_command *fcmd;
	unsigned flow_num, q_num;
	struct pa_packet *tx;
	int size, ret;

	dev_dbg(core_dev->dev, "index %d, rule %d, proto %d\n",
		index, rule, proto);

	memset(&fail_info, 0, sizeof(fail_info));
	memset(&route_info, 0, sizeof(route_info));

	q_num = core_dev->rx_queue_base;
	flow_num = core_dev->rx_flow_base;

	if (rule == PACKET_HST) {
		route_info.dest			= PA_DEST_HOST;
		route_info.flow_id		= flow_num;
		route_info.queue		= q_num;
		route_info.m_route_index	= -1;
		route_info.route_type		= PA_ROUTE_INTF_FLOW;
		fail_info.dest			= PA_DEST_HOST;
		fail_info.flow_id		= flow_num;
		fail_info.queue			= q_num;
		fail_info.m_route_index		= -1;
		fail_info.route_type		= PA_ROUTE_INTF_FLOW;
	} else if (rule == PACKET_PARSE) {
		route_info.dest			= PA_DEST_CONTINUE_PARSE_LUT2;
		route_info.m_route_index	= -1;
		fail_info.dest			= PA_DEST_HOST;
		fail_info.flow_id		= flow_num;
		fail_info.queue			= q_num;
		fail_info.m_route_index		= -1;
		fail_info.route_type		= PA_ROUTE_INTF_FLOW;
	} else if (rule == PACKET_DROP) {
		route_info.dest			= PA_DEST_DISCARD;
		route_info.m_route_index	= -1;
		fail_info.dest			= PA_DEST_DISCARD;
		fail_info.m_route_index		= -1;
	}

	size = (sizeof(struct pa_frm_command) +
		sizeof(struct pa_frm_cmd_add_lut1) + 4);

	tx = core_ops->alloc_packet(core_dev, size, PA_CLUSTER_1);
	if (!tx) {
		dev_err(core_dev->dev,
			"%s: could not allocate cmd tx packet\n",
			__func__);
		return -ENOMEM;
	}

	fcmd = tx->data;
	al1 = (struct pa_frm_cmd_add_lut1 *)&fcmd->cmd;
	memset(al1, 0, sizeof(*al1));
	pa_format_cmd_hdr(pa_dev, fcmd, PAFRM_CONFIG_COMMAND_ADDREP_LUT1,
			  PA_COMID_L3, context);

	al1->index = index;
	al1->type = PAFRM_COM_ADD_LUT1_STANDARD;
	al1->u.eth_ip.proto_next = proto;
	al1->u.eth_ip.match_flags |= PAFRM_LUT1_MATCH_PROTO;
	al1->u.eth_ip.match_flags = cpu_to_be16(al1->u.eth_ip.match_flags);
	ret = pa_conv_routing_info(&al1->match, &route_info, 0, 0);
	if (ret != 0) {
		dev_err(core_dev->dev, "%s:route info config failed\n",
			__func__);
		goto fail;
	}
	ret = pa_conv_routing_info(&al1->next_fail, &fail_info, 0, 1);
	if (ret != 0) {
		dev_err(core_dev->dev, "%s:fail info config failed\n",
			__func__);
		goto fail;
	}

	swiz_fwd(&al1->match);
	swiz_fwd(&al1->next_fail);

	/* Indicate that it is a configuration command */
	tx->psdata[0] = BIT(31);
	return core_ops->submit_packet(tx, PA_CLUSTER_1);

fail:
	core_ops->free_packet(core_dev, tx);
	return ret;
}

/* Configure route for exception packets in PA
 * All exceptions will be routed to Linux
 */
static int pa_config_exception_route(struct pa_core_device *core_dev)
{
	struct pa_device *pa_dev = to_pa(core_dev);
	struct pa_route_info eroutes[EROUTE_N_MAX];
	struct pa_frm_command_sys_config_pa *cpa;
	u32 context = PA_CONTEXT_CONFIG;
	struct pa_frm_command *fcmd;
	struct pa_packet *tx;
	int i, size, ret;

	memset(eroutes, 0, sizeof(eroutes));
	size = (sizeof(struct pa_frm_command) +
		sizeof(struct pa_frm_command_sys_config_pa) + 4);

	tx = core_ops->alloc_packet(core_dev, size, PA_CLUSTER_1);
	if (!tx) {
		dev_err(core_dev->dev,
			"%s: could not allocate cmd tx packet\n",
			__func__);
		ret = -ENOMEM;
		goto fail;
	}

	fcmd = tx->data;
	cpa = (struct pa_frm_command_sys_config_pa *)&fcmd->cmd;
	memset(cpa, 0, sizeof(*cpa));
	pa_format_cmd_hdr(pa_dev, fcmd, PAFRM_CONFIG_COMMAND_SYS_CONFIG,
			  0, context);
	cpa->cfg_code = PAFRM_SYSTEM_CONFIG_CODE_EROUTE;

	for (i = 0; i < EROUTE_N_MAX; i++) {
		eroutes[i].dest			= PA_DEST_HOST;
		eroutes[i].flow_id		= core_dev->rx_flow_base;
		eroutes[i].queue		= core_dev->rx_queue_base;
		eroutes[i].m_route_index	= -1;
		eroutes[i].route_type		= PA_ROUTE_INTF_FLOW;
		cpa->u.eroute.route_bitmap |= (1 << i);

		ret =  pa_conv_routing_info(&cpa->u.eroute.eroute[i],
					    &eroutes[i], PA_CMD_TX_DEST_5, 0);
		if (ret != 0) {
			dev_err(core_dev->dev,
				"%s: route info config failed\n",
				__func__);
			goto fail;
		}
		swiz_fwd(&cpa->u.eroute.eroute[i]);
	}
	cpa->u.eroute.route_bitmap = cpu_to_be32(cpa->u.eroute.route_bitmap);

	/* Indicate that it is a configuration command */
	tx->psdata[0] = BIT(31);
	return core_ops->submit_packet(tx, PA_CLUSTER_1);

fail:
	core_ops->free_packet(core_dev, tx);
	return ret;
}

static int pa_add_mac_rule(struct pa_intf *pa_intf, int index,
			   const u8 *smac, const u8 *dmac, int rule,
			   unsigned etype, int port)
{
	struct pa_core_device *core_dev = pa_intf->core_dev;
	struct pa_route_info route_info, fail_info;
	struct pa_frm_command *fcmd;
	struct pa_frm_cmd_add_lut1 *al1;
	u32 context = PA_CONTEXT_CONFIG;
	struct pa_packet *tx;
	int size, ret;

	dev_dbg(core_dev->dev,
		"add mac, index %d, smac %pM, dmac %pM, rule %d,",
		index, smac, dmac, rule);
	dev_dbg(core_dev->dev,
		"type %04x, port %d\n", etype, port);

	memset(&fail_info, 0, sizeof(fail_info));
	memset(&route_info, 0, sizeof(route_info));

	if (rule == PACKET_HST) {
		route_info.dest			= PA_DEST_HOST;
		route_info.flow_id		= pa_intf->data_flow_num;
		route_info.queue		= pa_intf->data_queue_num;
		route_info.m_route_index	= -1;
		fail_info.dest			= PA_DEST_HOST;
		fail_info.flow_id		= pa_intf->data_flow_num;
		fail_info.queue			= pa_intf->data_queue_num;
		fail_info.m_route_index		= -1;
	} else if (rule == PACKET_PARSE) {
		route_info.dest			= PA_DEST_CONTINUE_PARSE_LUT1;
		route_info.m_route_index	= -1;
		fail_info.dest			= PA_DEST_HOST;
		fail_info.flow_id		= pa_intf->data_flow_num;
		fail_info.queue			= pa_intf->data_queue_num;
		fail_info.m_route_index		= -1;
	} else if (rule == PACKET_DROP) {
		route_info.dest			= PA_DEST_DISCARD;
		route_info.m_route_index	= -1;
		fail_info.dest			= PA_DEST_DISCARD;
		fail_info.m_route_index		= -1;
	}

	size = (sizeof(struct pa_frm_command) +
		sizeof(struct pa_frm_cmd_add_lut1) + 4);
	tx = core_ops->alloc_packet(core_dev, size, PA_CLUSTER_0);
	if (!tx) {
		dev_err(core_dev->dev,
			"%s: could not allocate cmd tx packet\n",
			__func__);
		return -ENOMEM;
	}

	fcmd = tx->data;
	al1 = (struct pa_frm_cmd_add_lut1 *)&fcmd->cmd;

	fcmd->command_result	= 0;
	fcmd->command		= PAFRM_CONFIG_COMMAND_ADDREP_LUT1;
	fcmd->magic		= PAFRM_CONFIG_COMMAND_SEC_BYTE;
	fcmd->com_id		= PA_COMID_L2;
	fcmd->ret_context	= context;
	fcmd->flow_id		= core_dev->cmd_flow_num;
	fcmd->reply_queue	= core_dev->cmd_queue_num;
	fcmd->reply_dest	= PAFRM_DEST_PKTDMA;

	al1->index		= index;
	al1->type		= PAFRM_COM_ADD_LUT1_STANDARD;
	if (etype) {
		al1->u.eth_ip.etype	= etype;
		al1->u.eth_ip.match_flags |= PAFRM_LUT1_MATCH_ETYPE;
	}
	al1->u.eth_ip.vlan	= 0;
	al1->u.eth_ip.pm.mpls	= 0;
	if (port) {
		al1->u.eth_ip.inport    = port;
		al1->u.eth_ip.match_flags |= PAFRM_LUT1_MATCH_PORT;
	}
	if (dmac) {
		al1->u.eth_ip.dmac[0] = dmac[0];
		al1->u.eth_ip.dmac[1] = dmac[1];
		al1->u.eth_ip.dmac[2] = dmac[2];
		al1->u.eth_ip.dmac[3] = dmac[3];
		al1->u.eth_ip.dmac[4] = dmac[4];
		al1->u.eth_ip.dmac[5] = dmac[5];
		al1->u.eth_ip.match_flags |= PAFRM_LUT1_MATCH_DMAC;
	}
	if (smac) {
		al1->u.eth_ip.smac[0] = smac[0];
		al1->u.eth_ip.smac[1] = smac[1];
		al1->u.eth_ip.smac[2] = smac[2];
		al1->u.eth_ip.smac[3] = smac[3];
		al1->u.eth_ip.smac[4] = smac[4];
		al1->u.eth_ip.smac[5] = smac[5];
		al1->u.eth_ip.match_flags |= PAFRM_LUT1_MATCH_SMAC;
	}
	al1->u.eth_ip.key |= PAFRM_LUT1_KEY_MAC;
	al1->u.eth_ip.match_flags |= PAFRM_LUT1_CUSTOM_MATCH_KEY;

	ret = pa_conv_routing_info(&al1->match, &route_info, 0, 0);
	if (ret) {
		dev_err(core_dev->dev, "%s: route info config failed\n",
			__func__);
		goto fail;
	}

	ret = pa_conv_routing_info(&al1->next_fail, &fail_info, 0, 1);
	if (ret) {
		dev_err(core_dev->dev, "%s:fail info config failed\n",
			__func__);
		goto fail;
	}

	swizfcmd(fcmd);
	swizal1((struct pa_frm_cmd_add_lut1 *)&fcmd->cmd);

	tx->psdata[0] = BIT(31);
	return core_ops->submit_packet(tx, PA_CLUSTER_0);

fail:
	core_ops->free_packet(core_dev, tx);
	return ret;
}

static int pa_fmtcmd_tx_csum(struct netcp_packet *p_info)
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

static int pa_fmtcmd_next_route(struct netcp_packet *p_info, int eth_port)
{
	u8 ps_flags = (eth_port & GENMASK(2, 0))
				<< PAFRM_ETH_PS_FLAGS_PORT_SHIFT;

	struct paho_next_route *nr;

	nr = (struct paho_next_route *)netcp_push_psdata(p_info, sizeof(*nr));
	if (!nr)
		return -ENOMEM;

	/* Construct word0 */
	nr->word0 = 0;
	PAHO_SET_CMDID(nr, PAHO_PAMOD_NROUTE);
	PAHO_SET_E(nr, 1);
	PAHO_SET_DEST(nr, PAFRM_DEST_ETH);
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

static inline int
pa_fmtcmd_tx_timestamp(struct netcp_packet *p_info,
		       const struct pa_cmd_tx_timestamp *tx_ts)
{
	struct paho_report_timestamp	*rt_info;
	int				 size;

	size = sizeof(*rt_info);
	rt_info =
	(struct paho_report_timestamp *)netcp_push_psdata(p_info, size);
	if (!rt_info)
		return -ENOMEM;

	rt_info->word0 = 0;
	PAHO_SET_CMDID(rt_info, PAHO_PAMOD_REPORT_TIMESTAMP);
	PAHO_SET_REPORT_FLOW(rt_info, (u8)tx_ts->flow_id);
	PAHO_SET_REPORT_QUEUE(rt_info, tx_ts->dest_queue);
	rt_info->sw_info0 = tx_ts->sw_info0;

	return size;
}

static int pa_fmtcmd_align(struct netcp_packet *p_info,
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

static void pa_rx_timestamp_hook(struct pa_intf *pa_intf,
				 struct netcp_packet *p_info)
{
	struct pa_core_device *core_dev = pa_intf->core_dev;
	struct skb_shared_hwtstamps *sh_hw_tstamps;
	struct sk_buff *skb = p_info->skb;
	u64 pa_ns;

	if (!pa_intf->rx_timestamp_enable)
		return;

	if (p_info->rxtstamp_complete)
		return;

	pa_ns = tstamp_raw_to_ns(core_dev, p_info->epib[0], p_info->psdata[6]);
	sh_hw_tstamps = skb_hwtstamps(skb);
	memset(sh_hw_tstamps, 0, sizeof(*sh_hw_tstamps));
	sh_hw_tstamps->hwtstamp = ns_to_ktime(pa_ns);

	p_info->rxtstamp_complete = true;
}

static int pa_do_tx_timestamp(struct pa_core_device *core_dev,
			      struct netcp_packet *p_info)
{
	struct pa_device *pa_dev = to_pa(core_dev);
	struct sk_buff *skb = p_info->skb;
	struct pa_cmd_tx_timestamp tx_ts;
	struct tstamp_pending *pend;
	void *saved_sp;

	pend = kzalloc(sizeof(*pend), GFP_ATOMIC);
	if (unlikely(!pend))
		return -ENOMEM;

	/* The SA module may have reused skb->sp */
	saved_sp = skb->sp;
	skb->sp = NULL;
	pend->skb = skb_clone_sk(skb);
	skb->sp = saved_sp;

	if (unlikely(!pend->skb)) {
		kfree(pend);
		return -ENOMEM;
	}

	pend->pa_dev = pa_dev;
	pend->context =  PA_CONTEXT_TSTAMP | (~PA_CONTEXT_MASK &
			 atomic_inc_return(&tstamp_sequence));
	tstamp_add_pending(pend);

	memset(&tx_ts, 0, sizeof(tx_ts));
	tx_ts.dest_queue = core_dev->cmd_queue_num;
	tx_ts.flow_id    = core_dev->cmd_flow_num;
	tx_ts.sw_info0   = pend->context;

	return pa_fmtcmd_tx_timestamp(p_info, &tx_ts);
}

/*  The NETCP sub-system performs IPv4 header checksum, UDP/TCP checksum and
 *  SCTP CRC-32c checksum autonomously.
 *  The checksum and CRC verification results are recorded at the 4-bit error
 *  flags in the CPPI packet descriptor as described below:
 *  bit 3: IPv4 header checksum error
 *  bit 2: UDP/TCP or SCTP CRC-32c checksum error
 *  bit 1: Custom CRC checksum error
 *  bit 0: reserved
 */
static void pa_rx_checksum_hook(struct netcp_packet *p_info)
{
	struct paho_long_info *linfo =
		(struct paho_long_info *)p_info->psdata;

	if (likely(PAHO_LINFO_READ_L5_OFFSET(linfo))) {
		/* check for L3 & L4 checksum error */
		if (likely(!((p_info->eflags >> 2) & GENMASK(1, 0))))
			p_info->skb->ip_summed = CHECKSUM_UNNECESSARY;
	}
}

static int pa_hwtstamp_ioctl(struct pa_intf *pa_intf,
			     struct ifreq *ifr, int cmd)
{
	struct hwtstamp_config cfg;
	struct pa_core_device *core_dev = pa_intf->core_dev;

	if (core_dev->disable_hw_tstamp)
		return -EOPNOTSUPP;

	if (copy_from_user(&cfg, ifr->ifr_data, sizeof(cfg)))
		return -EFAULT;

	if (cfg.flags)
		return -EINVAL;

	switch (cfg.tx_type) {
	case HWTSTAMP_TX_OFF:
		pa_intf->tx_timestamp_enable = false;
		break;
	case HWTSTAMP_TX_ON:
		pa_intf->tx_timestamp_enable = true;
		break;
	default:
		return -ERANGE;
	}

	switch (cfg.rx_filter) {
	case HWTSTAMP_FILTER_NONE:
		pa_intf->rx_timestamp_enable = false;
		break;
	default:
		pa_intf->rx_timestamp_enable = true;
		break;
	}

	return copy_to_user(ifr->ifr_data, &cfg, sizeof(cfg)) ? -EFAULT : 0;
}

static u32 pa_get_streaming_switch(struct pa_core_device *core_dev, int port)
{
	struct pa_device *pa_dev = to_pa(core_dev);
	u32 reg;

	reg = readl(pa_dev->streaming_switch);
	return (port == 0) ? reg : (reg >> ((port - 1) * 8)) & 0xff;
}

static u32 pa_set_streaming_switch(struct pa_core_device *core_dev,
				   int port, u32 new_value)
{
	struct pa_device *pa_dev = to_pa(core_dev);
	u32 reg, old_value;

	reg = readl(pa_dev->streaming_switch);

	if (port == 0) {
		old_value = reg;
		reg = (new_value << 24) | (new_value << 16) |
			(new_value << 8) | new_value;
	} else {
		int shift = (port - 1) * 8;

		old_value = (reg >> shift) & 0xff;
		reg &= ~(0xff << shift);
		reg |= (new_value & 0xff) << shift;
	}
	writel(reg, pa_dev->streaming_switch);

	return old_value;
}

static void pa_unmap_resources(struct pa_core_device *core_dev)
{
	struct pa_device *pa_dev = to_pa(core_dev);

	if (pa_dev->reg_mailbox)
		iounmap(pa_dev->reg_mailbox);
	if (pa_dev->reg_packet_id)
		iounmap(pa_dev->reg_packet_id);
	if (pa_dev->reg_lut2)
		iounmap(pa_dev->reg_lut2);
	if (pa_dev->streaming_switch)
		iounmap(pa_dev->streaming_switch);
	if (pa_dev->reg_control)
		iounmap(pa_dev->reg_control);
	if (pa_dev->reg_timer)
		iounmap(pa_dev->reg_timer);
	if (pa_dev->reg_stats)
		iounmap(pa_dev->reg_stats);
	if (pa_dev->pa_iram)
		iounmap(pa_dev->pa_iram);
	if (pa_dev->pa_sram)
		iounmap(pa_dev->pa_sram);
}

static int pa_map_resources(struct pa_core_device *core_dev,
			    struct device_node *node)
{
	struct pa_device *pa_dev = to_pa(core_dev);
	struct device *dev = core_dev->dev;
	int ret = -ENODEV;

	pa_dev->reg_mailbox =
		core_ops->map_resource(dev, node, PA_MB_REG_INDEX);
	if (!pa_dev->reg_mailbox)
		return ret;

	pa_dev->reg_packet_id =
		core_ops->map_resource(dev, node, PA_PACKET_ID_REG_INDEX);
	if (!pa_dev->reg_packet_id)
		goto unmap;

	pa_dev->reg_lut2 = core_ops->map_resource(dev, node, PA_LUT2_REG_INDEX);
	if (!pa_dev->reg_lut2)
		goto unmap;

	pa_dev->streaming_switch =
		core_ops->map_resource(dev, node, PA_SS_INDEX);
	if (!pa_dev->streaming_switch)
		goto unmap;

	pa_dev->reg_control =
		core_ops->map_resource(dev, node, PA_CONTROL_REG_INDEX);
	if (!pa_dev->reg_control)
		goto unmap;

	pa_dev->reg_timer =
		core_ops->map_resource(dev, node, PA_TIMER_REG_INDEX);
	if (!pa_dev->reg_timer)
		goto unmap;

	pa_dev->reg_stats =
		core_ops->map_resource(dev, node, PA_STATS_REG_INDEX);
	if (!pa_dev->reg_stats)
		goto unmap;

	pa_dev->pa_iram = core_ops->map_resource(dev, node, PA_IRAM_INDEX);
	if (!pa_dev->pa_iram)
		goto unmap;

	pa_dev->pa_sram = core_ops->map_resource(dev, node, PA_SRAM_INDEX);
	if (!pa_dev->pa_sram)
		goto unmap;

	return 0;

unmap:	pa_unmap_resources(core_dev);
	return ret;
}

static struct pa_hw netcp_pa_hw = {
	.features = (PA_RX_CHECKSUM | PA_TIMESTAMP),
	.fmtcmd_next_route = pa_fmtcmd_next_route,
	.do_tx_timestamp = pa_do_tx_timestamp,
	.fmtcmd_tx_csum = pa_fmtcmd_tx_csum,
	.fmtcmd_align = pa_fmtcmd_align,
	.rx_checksum_hook = pa_rx_checksum_hook,
	.rx_timestamp_hook = pa_rx_timestamp_hook,
	.num_clusters = PA_NUM_CLUSTERS,
	.num_pdsps = PA_NUM_PDSPS,
	.ingress_l2_cluster_id = PA_CLUSTER_0,
	.ingress_l3_cluster_id = PA_CLUSTER_1,
	.egress_cluster_id = PA_CLUSTER_5,
	.streaming_pdsp = PSTREAM_ROUTE_PDSP0,
	.map_resources	= pa_map_resources,
	.unmap_resources = pa_unmap_resources,
	.pre_init = pa_reset,
	.post_init = pa_post_init,
	.set_firmware = pa_set_firmware,
	.rx_packet_handler = pa_rx_packet_handler,
	.add_mac_rule = pa_add_mac_rule,
	.config_exception_route = pa_config_exception_route,
	.set_streaming_switch = pa_set_streaming_switch,
	.get_streaming_switch = pa_get_streaming_switch,
	.cleanup = pa_tstamp_purge_pending,
	.add_ip_proto = pa_add_ip_proto,
};

static int pa_probe(struct netcp_device *netcp_device,
		    struct device *dev,
		    struct device_node *node,
		    void **inst_priv)
{
	int ret, size = sizeof(struct pa_device);
	struct pa_device *pa_dev;

	if (!node) {
		dev_err(dev, "%s: device tree info unavailable\n",
			__func__);
		return -ENODEV;
	}

	pa_dev = (struct pa_device *)core_ops->init(netcp_device, dev, node,
						     size, &ret, &netcp_pa_hw,
						     pa_pdsp_firmwares);
	if (!pa_dev)
		return ret;
	*inst_priv = pa_dev;

	spin_lock_init(&tstamp_lock);

	return ret;
}

static int pa_ioctl(void *intf_priv, struct ifreq *req, int cmd)
{
	struct pa_intf *pa_intf = intf_priv;

	if (cmd == SIOCSHWTSTAMP)
		return pa_hwtstamp_ioctl(pa_intf, req, cmd);

	return -EOPNOTSUPP;
}

static struct netcp_module pa_module = {
	.name		= "netcp-pa",
	.owner		= THIS_MODULE,
	.probe		= pa_probe,
	.close		= pa_core_close,
	.open		= pa_core_open,
	.remove		= pa_core_remove,
	.attach		= pa_core_attach,
	.release	= pa_core_release,
	.add_addr	= pa_core_add_addr,
	.del_addr	= pa_core_del_addr,
	.ioctl		= pa_ioctl,
};

static int __init netcp_pa_init(void)
{
	return netcp_register_module(&pa_module);
}
module_init(netcp_pa_init);

static void __exit netcp_pa_exit(void)
{
	netcp_unregister_module(&pa_module);
}
module_exit(netcp_pa_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Murali Karicheri m-karicheri2@ti.com>");
MODULE_DESCRIPTION("TI NetCP Packet Accelerator driver for Keystone devices");
