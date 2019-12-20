// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2016-2018 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Jyri Sarha <jsarha@ti.com>
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>

#include <drm/drm_fourcc.h>
#include <drm/drm_panel.h>
#include <drm/drm_of.h>

#include "tidss_drv.h"
#include "tidss_crtc.h"
#include "tidss_plane.h"
#include "tidss_encoder.h"

#include "tidss_dispc.h"
#include "tidss_scale_coefs.h"
#include "tidss_dispc7.h"

static const char *dispc7_plane_name(struct dispc_device *dispc, u32 hw_plane);

static const struct dispc7_features dispc7_am6_feats = {
	.max_pclk_kHz = {
		[DISPC7_VP_DPI] = 165000,
		[DISPC7_VP_OLDI] = 165000,
	},

	.num_commons = 1,
	.common_name = { "common" },
	.common_cfg = { true },

	.scaling = {
		.in_width_max_5tap_rgb = 1280,
		.in_width_max_3tap_rgb = 2560,
		.in_width_max_5tap_yuv = 2560,
		.in_width_max_3tap_yuv = 4096,
		.upscale_limit = 16,
		.downscale_limit_5tap = 4,
		.downscale_limit_3tap = 2,
		/*
		 * The max supported pixel inc value is 255. The value
		 * of pixel inc is calculated like this: 1+(xinc-1)*bpp.
		 * The maximum bpp of all formats supported by the HW
		 * is 8. So the maximum supported xinc value is 32,
		 * because 1+(32-1)*8 < 255 < 1+(33-1)*4.
		 */
		.xinc_max = 32,
	},

	.subrev = DSS7_AM6,

	.num_vps = 2,
	.vp_name = { "vp1", "vp2" },
	.ovr_name = { "ovr1", "ovr2" },
	.vpclk_name =  { "vp1", "vp2" },
	.vp_bus_type = { DISPC7_VP_OLDI, DISPC7_VP_DPI },

	.vp_feat = { .color = {
			.has_ctm = true,
			.gamma_size = 256,
			.gamma_type = TIDSS_GAMMA_8BIT,
		},
	},

	.num_planes = 2,
	/* note: vid is plane_id 0 and vidl1 is plane_id 1 */
	.vid_name = { "vid", "vidl1" },
	.vid_lite = { false, true, },
	.vid_order = { 1, 0 },

	.errata = {
		.i2000 = true,
	},
};

static const struct dispc7_features dispc7_j721e_feats = {
	.max_pclk_kHz = {
		[DISPC7_VP_DPI] = 170000,
		[DISPC7_VP_INTERNAL] = 600000,
	},

	.num_commons = 4,
	.common_name = { "common_m", "common_s0", "common_s1", "common_s2" },
	.common_cfg = { true, false, false, false },

	.scaling = {
		.in_width_max_5tap_rgb = 2048,
		.in_width_max_3tap_rgb = 4096,
		.in_width_max_5tap_yuv = 4096,
		.in_width_max_3tap_yuv = 4096,
		.upscale_limit = 16,
		.downscale_limit_5tap = 4,
		.downscale_limit_3tap = 2,
		/*
		 * The max supported pixel inc value is 255. The value
		 * of pixel inc is calculated like this: 1+(xinc-1)*bpp.
		 * The maximum bpp of all formats supported by the HW
		 * is 8. So the maximum supported xinc value is 32,
		 * because 1+(32-1)*8 < 255 < 1+(33-1)*4.
		 */
		.xinc_max = 32,
	},

	.subrev = DSS7_J721E,

	.num_vps = 4,
	.vp_name = { "vp1", "vp2", "vp3", "vp4" },
	.ovr_name = { "ovr1", "ovr2", "ovr3", "ovr4" },
	.vpclk_name = { "vp1", "vp2", "vp3", "vp4" },
	/* Currently hard coded VP routing (see dispc7_initial_config()) */
	.vp_bus_type =	{ DISPC7_VP_INTERNAL, DISPC7_VP_DPI,
			  DISPC7_VP_INTERNAL, DISPC7_VP_DPI, },
	.vp_feat = { .color = {
			.has_ctm = true,
			.gamma_size = 1024,
			.gamma_type = TIDSS_GAMMA_10BIT,
		},
	},
	.num_planes = 4,
	.vid_name = { "vid1", "vidl1", "vid2", "vidl2" },
	.vid_lite = { 0, 1, 0, 1, },
	.vid_order = { 1, 3, 0, 2 },
	.has_writeback = true,
	.wb_name = "wb",
};

static const struct of_device_id dispc7_of_table[] = {
	{ .compatible = "ti,am6-dss", .data = &dispc7_am6_feats },
	{ .compatible = "ti,j721e-dss", .data = &dispc7_j721e_feats },
	{ }
};

/*
 * TRM gives bitfields as start:end, where start is the higher bit
 * number. For example 7:0
 */

#define FLD_MASK(start, end)	(((1 << ((start) - (end) + 1)) - 1) << (end))
#define FLD_VAL(val, start, end) (((val) << (end)) & FLD_MASK(start, end))
#define FLD_GET(val, start, end) (((val) & FLD_MASK(start, end)) >> (end))
#define FLD_MOD(orig, val, start, end) \
	(((orig) & ~FLD_MASK(start, end)) | FLD_VAL(val, start, end))

#define CFG_REG_GET(dispc, idx, start, end) \
	FLD_GET(dispc7_cfg_read(dispc, idx), start, end)

#define CFG_REG_FLD_MOD(dispc, idx, val, start, end) \
	dispc7_cfg_write(dispc, idx, FLD_MOD(dispc7_cfg_read(dispc, idx), val, start, end))

#define VID_REG_GET(dispc, hw_plane, idx, start, end) \
	FLD_GET(dispc7_vid_read(dispc, hw_plane, idx), start, end)

#define VID_REG_FLD_MOD(dispc, hw_plane, idx, val, start, end) \
	dispc7_vid_write(dispc, hw_plane, idx, FLD_MOD(dispc7_vid_read(dispc, hw_plane, idx), val, start, end))

#define WB_REG_GET(dispc, idx, start, end) \
	FLD_GET(dispc7_wb_read(dispc, idx), start, end)

#define WB_REG_FLD_MOD(dispc, idx, val, start, end) \
	dispc7_wb_write(dispc, idx, FLD_MOD(dispc7_wb_read(dispc, idx), val, start, end))

#define VP_REG_GET(dispc, vp, idx, start, end) \
	FLD_GET(dispc7_vp_read(dispc, vp, idx), start, end)

#define VP_REG_FLD_MOD(dispc, vp, idx, val, start, end) \
	dispc7_vp_write(dispc, vp, idx, FLD_MOD(dispc7_vp_read(dispc, vp, idx), val, start, end))

#define OVR_REG_GET(dispc, ovr, idx, start, end) \
	FLD_GET(dispc7_ovr_read(dispc, ovr, idx), start, end)

#define OVR_REG_FLD_MOD(dispc, ovr, idx, val, start, end) \
	dispc7_ovr_write(dispc, ovr, idx, FLD_MOD(dispc7_ovr_read(dispc, ovr, idx), val, start, end))

static const u16 tidss_am6_common_regs[DSS7_COMMON_REG_TABLE_LEN] = {
	[DSS_REVISION_OFF] =			0x4,
	[DSS_SYSCONFIG_OFF] =			0x8,
	[DSS_SYSSTATUS_OFF] =			0x20,
	[DISPC_IRQ_EOI_OFF] =			0x24,
	[DISPC_IRQSTATUS_RAW_OFF] =		0x28,
	[DISPC_IRQSTATUS_OFF] =			0x2c,
	[DISPC_IRQENABLE_SET_OFF] =		0x30,
	[DISPC_IRQENABLE_CLR_OFF] =		0x40,
	[DISPC_VID_IRQENABLE_OFF] =		0x44,
	[DISPC_VID_IRQSTATUS_OFF] =		0x58,
	[DISPC_VP_IRQENABLE_OFF] =		0x70,
	[DISPC_VP_IRQSTATUS_OFF] =		0x7c,

	[WB_IRQENABLE_OFF] =			0x88,
	[WB_IRQSTATUS_OFF] =			0x8c,

	[DISPC_GLOBAL_MFLAG_ATTRIBUTE_OFF] =	0x90,
	[DISPC_GLOBAL_OUTPUT_ENABLE_OFF] =	0x94,
	[DISPC_GLOBAL_BUFFER_OFF] =		0x98,
	[DSS_CBA_CFG_OFF] =			0x9c,
	[DISPC_DBG_CONTROL_OFF] =		0xa0,
	[DISPC_DBG_STATUS_OFF] =		0xa4,
	[DISPC_CLKGATING_DISABLE_OFF] =		0xa8,
	[DISPC_SECURE_DISABLE_OFF] =		0xac,
};

static const u16 tidss_j721e_common_regs[DSS7_COMMON_REG_TABLE_LEN] = {
	[DSS_REVISION_OFF] =			0x4,
	[DSS_SYSCONFIG_OFF] =			0x8,
	[DSS_SYSSTATUS_OFF] =			0x20,
	[DISPC_IRQ_EOI_OFF] =			0x80,
	[DISPC_IRQSTATUS_RAW_OFF] =		0x28,
	[DISPC_IRQSTATUS_OFF] =			0x2c,
	[DISPC_IRQENABLE_SET_OFF] =		0x30,
	[DISPC_IRQENABLE_CLR_OFF] =		0x34,
	[DISPC_VID_IRQENABLE_OFF] =		0x38,
	[DISPC_VID_IRQSTATUS_OFF] =		0x48,
	[DISPC_VP_IRQENABLE_OFF] =		0x58,
	[DISPC_VP_IRQSTATUS_OFF] =		0x68,

	[WB_IRQENABLE_OFF] =			0x78,
	[WB_IRQSTATUS_OFF] =			0x7c,

	[DISPC_GLOBAL_MFLAG_ATTRIBUTE_OFF] =	0x98,
	[DISPC_GLOBAL_OUTPUT_ENABLE_OFF] =	0x9c,
	[DISPC_GLOBAL_BUFFER_OFF] =		0xa0,
	[DSS_CBA_CFG_OFF] =			0xa4,
	[DISPC_DBG_CONTROL_OFF] =		0xa8,
	[DISPC_DBG_STATUS_OFF] =		0xac,
	[DISPC_CLKGATING_DISABLE_OFF] =		0xb0,
	[DISPC_SECURE_DISABLE_OFF] =		0x90,

	[FBDC_REVISION_1_OFF] =			0xb8,
	[FBDC_REVISION_2_OFF] =			0xbc,
	[FBDC_REVISION_3_OFF] =			0xc0,
	[FBDC_REVISION_4_OFF] =			0xc4,
	[FBDC_REVISION_5_OFF] =			0xc8,
	[FBDC_REVISION_6_OFF] =			0xcc,
	[FBDC_COMMON_CONTROL_OFF] =		0xd0,
	[FBDC_CONSTANT_COLOR_0_OFF] =		0xd4,
	[FBDC_CONSTANT_COLOR_1_OFF] =		0xd8,
	[DISPC_CONNECTIONS_OFF] =		0xe4,
	[DISPC_MSS_VP1_OFF] =			0xe8,
	[DISPC_MSS_VP3_OFF] =			0xec,
};

static const u16 *dispc7_common_regmap;

struct dss_vp_data {
	u32 *gamma_table;
};

struct dss_plane_data {
	u32 zorder;
	u32 hw_videoport;
};

struct dispc_device {
	struct tidss_device *tidss;
	struct device *dev;

	void __iomem *base_common_cfg;
	void __iomem *base_common_intr;
	void __iomem *base_vid[DISPC7_MAX_PLANES];
	void __iomem *base_ovr[DISPC7_MAX_PORTS];
	void __iomem *base_vp[DISPC7_MAX_PORTS];
	void __iomem *base_wb;

	int irq;

	bool has_cfg_common;

	struct regmap *syscon;

	struct clk *vp_clk[DISPC7_MAX_PORTS];

	const struct dispc7_features *feat;

	bool vp_managed[DISPC7_MAX_PORTS];
	bool plane_managed[DISPC7_MAX_PLANES];
	bool wb_managed;
	u32 wb_reserved_ovr;

	struct clk *fclk;

	bool is_enabled;

	struct dss_vp_data vp_data[DISPC7_MAX_PORTS];

	struct dss_plane_data plane_data[DISPC7_MAX_PLANES];
};

static bool dispc7_has_writeback(struct dispc_device *dispc);

#define dispc_for_each_managed_vp(dispc, hw_videoport) \
	for ((hw_videoport) = 0; (hw_videoport) < (dispc)->feat->num_vps; (hw_videoport)++) \
		if ((dispc)->vp_managed[(hw_videoport)])

#define dispc_for_each_managed_plane(dispc, hw_plane) \
	for ((hw_plane) = 0; (hw_plane) < (dispc)->feat->num_planes; (hw_plane)++) \
		if ((dispc)->plane_managed[(hw_plane)])

static u32 dispc7_get_next_managed_plane(struct dispc_device *dispc,
					 u32 *plane_idx)
{
	while (*plane_idx < dispc->feat->num_planes) {
		u32 plane_id = dispc->feat->vid_order[(*plane_idx)++];

		if (dispc->plane_managed[plane_id])
			return plane_id;
	}

	return dispc->feat->num_planes;
}

static void dispc7_intr_write(struct dispc_device *dispc, u16 reg, u32 val)
{
	iowrite32(val, dispc->base_common_intr + reg);
}

static u32 dispc7_intr_read(struct dispc_device *dispc, u16 reg)
{
	return ioread32(dispc->base_common_intr + reg);
}

static void dispc7_cfg_write(struct dispc_device *dispc, u16 reg, u32 val)
{
	iowrite32(val, dispc->base_common_cfg + reg);
}

static u32 dispc7_cfg_read(struct dispc_device *dispc, u16 reg)
{
	return ioread32(dispc->base_common_cfg + reg);
}

static void dispc7_vid_write(struct dispc_device *dispc, u32 hw_plane, u16 reg, u32 val)
{
	void __iomem *base = dispc->base_vid[hw_plane];

	iowrite32(val, base + reg);
}

static u32 dispc7_vid_read(struct dispc_device *dispc, u32 hw_plane, u16 reg)
{
	void __iomem *base = dispc->base_vid[hw_plane];

	return ioread32(base + reg);
}

static void dispc7_wb_write(struct dispc_device *dispc, u16 reg, u32 val)
{
	void __iomem *base = dispc->base_wb;

	iowrite32(val, base + reg);
}

static u32 dispc7_wb_read(struct dispc_device *dispc, u16 reg)
{
	void __iomem *base = dispc->base_wb;

	return ioread32(base + reg);
}

static void dispc7_ovr_write(struct dispc_device *dispc, u32 hw_videoport, u16 reg, u32 val)
{
	void __iomem *base = dispc->base_ovr[hw_videoport];

	iowrite32(val, base + reg);
}

static u32 dispc7_ovr_read(struct dispc_device *dispc, u32 hw_videoport, u16 reg)
{
	void __iomem *base = dispc->base_ovr[hw_videoport];

	return ioread32(base + reg);
}

static void dispc7_vp_write(struct dispc_device *dispc, u32 hw_videoport, u16 reg, u32 val)
{
	void __iomem *base = dispc->base_vp[hw_videoport];

	iowrite32(val, base + reg);
}

static u32 dispc7_vp_read(struct dispc_device *dispc, u32 hw_videoport, u16 reg)
{
	void __iomem *base = dispc->base_vp[hw_videoport];

	return ioread32(base + reg);
}


static int dispc7_runtime_get(struct dispc_device *dispc)
{
	int r;

	dev_dbg(dispc->dev, "%s\n", __func__);

	r = pm_runtime_get_sync(dispc->dev);
	WARN_ON(r < 0);
	return r < 0 ? r : 0;
}

static void dispc7_runtime_put(struct dispc_device *dispc)
{
	int r;

	dev_dbg(dispc->dev, "%s\n", __func__);

	r = pm_runtime_put_sync(dispc->dev);
	WARN_ON(r < 0);
}

static u64 dispc7_vp_irq_from_raw(u32 stat, u32 hw_videoport)
{
	u64 vp_stat = 0;

	if (stat & BIT(0))
		vp_stat |= DSS_IRQ_VP_FRAME_DONE(hw_videoport);
	if (stat & BIT(1))
		vp_stat |= DSS_IRQ_VP_VSYNC_EVEN(hw_videoport);
	if (stat & BIT(2))
		vp_stat |= DSS_IRQ_VP_VSYNC_ODD(hw_videoport);
	if (stat & BIT(4))
		vp_stat |= DSS_IRQ_VP_SYNC_LOST(hw_videoport);

	return vp_stat;
}

static u32 dispc7_vp_irq_to_raw(u64 vpstat, u32 hw_videoport)
{
	u32 stat = 0;

	if (vpstat & DSS_IRQ_VP_FRAME_DONE(hw_videoport))
		stat |= BIT(0);
	if (vpstat & DSS_IRQ_VP_VSYNC_EVEN(hw_videoport))
		stat |= BIT(1);
	if (vpstat & DSS_IRQ_VP_VSYNC_ODD(hw_videoport))
		stat |= BIT(2);
	if (vpstat & DSS_IRQ_VP_SYNC_LOST(hw_videoport))
		stat |= BIT(4);

	return stat;
}

static u64 dispc7_vid_irq_from_raw(u32 stat, u32 hw_plane)
{
	u64 vid_stat = 0;

	if (stat & BIT(0))
		vid_stat |= DSS_IRQ_PLANE_FIFO_UNDERFLOW(hw_plane);

	return vid_stat;
}

static u32 dispc7_vid_irq_to_raw(u64 vidstat, u32 hw_plane)
{
	u32 stat = 0;

	if (vidstat & DSS_IRQ_PLANE_FIFO_UNDERFLOW(hw_plane))
		stat |= BIT(0);

	return stat;
}

static u64 dispc7_wb_irq_from_raw(u32 stat)
{
	u64 vp_stat = 0;

	if (stat & BIT(0))
		vp_stat |= DSS_IRQ_DEVICE_WBBUFFEROVERFLOW;
	if (stat & BIT(1))
		vp_stat |= DSS_IRQ_DEVICE_WBUNCOMPLETEERROR;
	if (stat & BIT(2))
		vp_stat |= DSS_IRQ_DEVICE_FRAMEDONEWB;
	if (stat & BIT(3))
		vp_stat |= DSS_IRQ_DEVICE_WBSECURITYVIOLATION;
	if (stat & BIT(4))
		vp_stat |= DSS_IRQ_DEVICE_WBSYNC;

	return vp_stat;
}

static u32 dispc7_wb_irq_to_raw(u64 vpstat)
{
	u32 stat = 0;

	if (vpstat & DSS_IRQ_DEVICE_WBBUFFEROVERFLOW)
		stat |= BIT(0);
	if (vpstat & DSS_IRQ_DEVICE_WBUNCOMPLETEERROR)
		stat |= BIT(1);
	if (vpstat & DSS_IRQ_DEVICE_FRAMEDONEWB)
		stat |= BIT(2);
	if (vpstat & DSS_IRQ_DEVICE_WBSECURITYVIOLATION)
		stat |= BIT(3);
	if (vpstat & DSS_IRQ_DEVICE_WBSYNC)
		stat |= BIT(4);

	return stat;
}

static u64 dispc7_vp_read_irqstatus(struct dispc_device *dispc,
				    u32 hw_videoport)
{
	u32 stat = dispc7_intr_read(dispc, DISPC_VP_IRQSTATUS(hw_videoport));

	return dispc7_vp_irq_from_raw(stat, hw_videoport);
}

static void dispc7_vp_write_irqstatus(struct dispc_device *dispc,
				      u32 hw_videoport, u64 vpstat)
{
	u32 stat = dispc7_vp_irq_to_raw(vpstat, hw_videoport);

	dispc7_intr_write(dispc, DISPC_VP_IRQSTATUS(hw_videoport), stat);
}

static u64 dispc7_vid_read_irqstatus(struct dispc_device *dispc,
				     u32 hw_plane)
{
	u32 stat = dispc7_intr_read(dispc, DISPC_VID_IRQSTATUS(hw_plane));

	return dispc7_vid_irq_from_raw(stat, hw_plane);
}

static void dispc7_vid_write_irqstatus(struct dispc_device *dispc,
				       u32 hw_plane, u64 vidstat)
{
	u32 stat = dispc7_vid_irq_to_raw(vidstat, hw_plane);

	dispc7_intr_write(dispc, DISPC_VID_IRQSTATUS(hw_plane), stat);
}

static u64 dispc7_wb_read_irqstatus(struct dispc_device *dispc)
{
	u32 stat = dispc7_intr_read(dispc, WB_IRQSTATUS);

	return dispc7_wb_irq_from_raw(stat);
}

static void dispc7_wb_write_irqstatus(struct dispc_device *dispc,
				      u64 vpstat)
{
	u32 stat = dispc7_wb_irq_to_raw(vpstat);

	dispc7_intr_write(dispc, WB_IRQSTATUS, stat);
}

static u64 dispc7_vp_read_irqenable(struct dispc_device *dispc,
				    u32 hw_videoport)
{
	u32 stat = dispc7_intr_read(dispc, DISPC_VP_IRQENABLE(hw_videoport));

	return dispc7_vp_irq_from_raw(stat, hw_videoport);
}

static void dispc7_vp_write_irqenable(struct dispc_device *dispc,
				      u32 hw_videoport, u64 vpstat)
{
	u32 stat = dispc7_vp_irq_to_raw(vpstat, hw_videoport);

	dispc7_intr_write(dispc, DISPC_VP_IRQENABLE(hw_videoport), stat);
}


static u64 dispc7_vid_read_irqenable(struct dispc_device *dispc,
				     u32 hw_plane)
{
	u32 stat = dispc7_intr_read(dispc, DISPC_VID_IRQENABLE(hw_plane));

	return dispc7_vid_irq_from_raw(stat, hw_plane);
}

static void dispc7_vid_write_irqenable(struct dispc_device *dispc,
				       u32 hw_plane, u64 vidstat)
{
	u32 stat = dispc7_vid_irq_to_raw(vidstat, hw_plane);

	dispc7_intr_write(dispc, DISPC_VID_IRQENABLE(hw_plane), stat);
}

static u64 dispc7_wb_read_irqenable(struct dispc_device *dispc)
{
	u32 stat = dispc7_intr_read(dispc, WB_IRQENABLE);

	return dispc7_wb_irq_from_raw(stat);
}

static void dispc7_wb_write_irqenable(struct dispc_device *dispc,
				      u64 vpstat)
{
	u32 stat = dispc7_wb_irq_to_raw(vpstat);

	dispc7_intr_write(dispc, WB_IRQENABLE, stat);
}

static void dispc7_clear_irqstatus(struct dispc_device *dispc, u64 clearmask)
{
	unsigned int i;
	u32 top_clear = 0;

	dispc_for_each_managed_vp(dispc, i) {
		if (clearmask & DSS_IRQ_VP_MASK(i)) {
			dispc7_vp_write_irqstatus(dispc, i, clearmask);
			top_clear |= BIT(i);
		}
	}
	dispc_for_each_managed_plane(dispc, i) {
		if (clearmask & DSS_IRQ_PLANE_MASK(i)) {
			dispc7_vid_write_irqstatus(dispc, i, clearmask);
			top_clear |= BIT(4 + i);
		}
	}
	if (dispc7_has_writeback(dispc)) {
		if (clearmask & DSS_IRQ_DEVICE_WB_MASK) {
			dispc7_wb_write_irqstatus(dispc, clearmask);
			top_clear |= BIT(14);
		}
	}
	dispc7_intr_write(dispc, DISPC_IRQSTATUS, top_clear);

	/* Flush posted writes */
	dispc7_intr_read(dispc, DISPC_IRQSTATUS);
}

static u64 dispc7_read_and_clear_irqstatus(struct dispc_device *dispc)
{
	u64 status = 0;
	unsigned int i;

	dispc_for_each_managed_vp(dispc, i)
		status |= dispc7_vp_read_irqstatus(dispc, i);

	dispc_for_each_managed_plane(dispc, i)
		status |= dispc7_vid_read_irqstatus(dispc, i);

	if (dispc7_has_writeback(dispc))
		status |= dispc7_wb_read_irqstatus(dispc);

	dispc7_clear_irqstatus(dispc, status);

	return status;
}

static u64 dispc7_read_irqenable(struct dispc_device *dispc)
{
	u64 enable = 0;
	unsigned int i;

	dispc_for_each_managed_vp(dispc, i)
		enable |= dispc7_vp_read_irqenable(dispc, i);

	dispc_for_each_managed_plane(dispc, i)
		enable |= dispc7_vid_read_irqenable(dispc, i);

	if (dispc7_has_writeback(dispc))
		enable |= dispc7_wb_read_irqenable(dispc);

	return enable;
}

static void dispc7_write_irqenable(struct dispc_device *dispc, u64 mask)
{
	unsigned int i;
	u32 main_enable = 0, main_disable = 0;
	u64 old_mask;

	old_mask = dispc7_read_irqenable(dispc);

	/* clear the irqstatus for newly enabled irqs */
	dispc7_clear_irqstatus(dispc, (old_mask ^ mask) & mask);

	dispc_for_each_managed_vp(dispc, i) {
		dispc7_vp_write_irqenable(dispc, i, mask);
		if (mask & DSS_IRQ_VP_MASK(i))
			main_enable |= BIT(i);		/* VP IRQ */
		else
			main_disable |= BIT(i);		/* VP IRQ */
	}

	dispc_for_each_managed_plane(dispc, i) {
		dispc7_vid_write_irqenable(dispc, i, mask);
		if (mask & DSS_IRQ_PLANE_MASK(i))
			main_enable |= BIT(i + 4);	/* VID IRQ */
		else
			main_disable |= BIT(i + 4);	/* VID IRQ */
	}

	if (dispc7_has_writeback(dispc)) {
		dispc7_wb_write_irqenable(dispc, mask);
		if (mask & DSS_IRQ_DEVICE_WB_MASK)
			main_enable |= BIT(14);		/* WB_IRQ */
		else
			main_disable |= BIT(14);	/* WB_IRQ */
	}

	if (main_enable)
		dispc7_intr_write(dispc, DISPC_IRQENABLE_SET, main_enable);

	if (main_disable)
		dispc7_intr_write(dispc, DISPC_IRQENABLE_CLR, main_disable);

	/* Flush posted writes */
	dispc7_intr_read(dispc, DISPC_IRQENABLE_SET);
}

enum dispc7_oldi_mode_reg_val { SPWG_18 = 0, JEIDA_24 = 1, SPWG_24 = 2 };

struct dispc7_bus_format {
	u32 bus_fmt;
	u32 data_width;
	bool is_oldi_fmt;
	enum dispc7_oldi_mode_reg_val oldi_mode_reg_val;
};

static const struct dispc7_bus_format dispc7_bus_formats[] = {
	{ MEDIA_BUS_FMT_RGB444_1X12,		12, false, 0 },
	{ MEDIA_BUS_FMT_RGB565_1X16,		16, false, 0 },
	{ MEDIA_BUS_FMT_RGB666_1X18,		18, false, 0 },
	{ MEDIA_BUS_FMT_RGB888_1X24,		24, false, 0 },
	{ MEDIA_BUS_FMT_RGB101010_1X30,		30, false, 0 },
	{ MEDIA_BUS_FMT_RGB121212_1X36,		36, false, 0 },
	{ MEDIA_BUS_FMT_RGB666_1X7X3_SPWG,	18, true, SPWG_18 },
	{ MEDIA_BUS_FMT_RGB888_1X7X4_SPWG,	24, true, SPWG_24 },
	{ MEDIA_BUS_FMT_RGB888_1X7X4_JEIDA,	24, true, JEIDA_24 },
};

static const
struct dispc7_bus_format *dispc7_vp_find_bus_fmt(struct dispc_device *dispc,
						 u32 hw_videoport,
						 u32 bus_fmt, u32 bus_flags)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(dispc7_bus_formats); ++i) {
		if (dispc7_bus_formats[i].bus_fmt == bus_fmt)
			return &dispc7_bus_formats[i];
	}

	return NULL;
}

static void dispc7_oldi_tx_power(struct dispc_device *dispc, bool power)
{
	u32 val = power ? 0 : CTRLMMR0P1_OLDI_PWRDN_TX;

	if (WARN_ON(!dispc->syscon))
		return;

	regmap_update_bits(dispc->syscon, CTRLMMR0P1_OLDI_DAT0_IO_CTRL,
			   CTRLMMR0P1_OLDI_PWRDN_TX, val);
	regmap_update_bits(dispc->syscon, CTRLMMR0P1_OLDI_DAT1_IO_CTRL,
			   CTRLMMR0P1_OLDI_PWRDN_TX, val);
	regmap_update_bits(dispc->syscon, CTRLMMR0P1_OLDI_DAT2_IO_CTRL,
			   CTRLMMR0P1_OLDI_PWRDN_TX, val);
	regmap_update_bits(dispc->syscon, CTRLMMR0P1_OLDI_DAT3_IO_CTRL,
			   CTRLMMR0P1_OLDI_PWRDN_TX, val);
	regmap_update_bits(dispc->syscon, CTRLMMR0P1_OLDI_CLK_IO_CTRL,
			   CTRLMMR0P1_OLDI_PWRDN_TX, val);
}

static void dispc7_set_num_datalines(struct dispc_device *dispc,
				     u32 hw_videoport, int num_lines)
{
	int v;

	switch (num_lines) {
	case 12:
		v = 0; break;
	case 16:
		v = 1; break;
	case 18:
		v = 2; break;
	case 24:
		v = 3; break;
	case 30:
		v = 4; break;
	case 36:
		v = 5; break;
	default:
		WARN_ON(1);
		v = 3;
	}

	VP_REG_FLD_MOD(dispc, hw_videoport, DISPC_VP_CONTROL, v, 10, 8);
}

static void dispc7_enable_oldi(struct dispc_device *dispc, u32 hw_videoport,
			       const struct dispc7_bus_format *fmt)
{
	u32 oldi_cfg = 0;
	u32 oldi_reset_bit = BIT(5 + hw_videoport);
	int count = 0;

	/*
	 * On am6 DUALMODESYNC, MASTERSLAVE, MODE, and SRC are set
	 * statically to 0.
	 */

	if (fmt->data_width == 24)
		oldi_cfg |= BIT(8); /* MSB */
	else if (fmt->data_width != 18)
		dev_warn(dispc->dev, "%s: %d port width not supported\n",
			 __func__, fmt->data_width);

	oldi_cfg |= BIT(7); /* DEPOL */

	oldi_cfg = FLD_MOD(oldi_cfg, fmt->oldi_mode_reg_val, 3, 1);

	oldi_cfg |= BIT(12); /* SOFTRST */

	oldi_cfg |= BIT(0); /* ENABLE */

	dispc7_vp_write(dispc, hw_videoport, DISPC_VP_DSS_OLDI_CFG, oldi_cfg);

	while (!(oldi_reset_bit & dispc7_cfg_read(dispc, DSS_SYSSTATUS)) &&
	       count < 10000)
		count++;

	if (!(oldi_reset_bit & dispc7_cfg_read(dispc, DSS_SYSSTATUS)))
		dev_warn(dispc->dev, "%s: timeout waiting OLDI reset done\n",
			 __func__);
}

static void dispc7_vp_prepare(struct dispc_device *dispc, u32 hw_videoport,
			      const struct drm_crtc_state *state)
{
	const struct tidss_crtc_state *tstate = to_tidss_crtc_state(state);
	const struct dispc7_bus_format *fmt;

	fmt = dispc7_vp_find_bus_fmt(dispc, hw_videoport, tstate->bus_format,
				     tstate->bus_flags);

	if (WARN_ON(!fmt))
		return;

	if (dispc->feat->vp_bus_type[hw_videoport] == DISPC7_VP_OLDI) {
		dispc7_oldi_tx_power(dispc, true);

		if (dispc->has_cfg_common)
			dispc7_enable_oldi(dispc, hw_videoport, fmt);
	}
}

static void dispc7_vp_enable(struct dispc_device *dispc, u32 hw_videoport,
			      const struct drm_crtc_state *state)
{
	const struct drm_display_mode *mode = &state->adjusted_mode;
	const struct tidss_crtc_state *tstate = to_tidss_crtc_state(state);
	bool align, onoff, rf, ieo, ipc, ihs, ivs;
	const struct dispc7_bus_format *fmt;
	u32 hsw, hfp, hbp, vsw, vfp, vbp;

	fmt = dispc7_vp_find_bus_fmt(dispc, hw_videoport, tstate->bus_format,
				     tstate->bus_flags);

	if (WARN_ON(!fmt))
		return;

	dispc7_set_num_datalines(dispc, hw_videoport, fmt->data_width);

	hfp = mode->hsync_start - mode->hdisplay;
	hsw = mode->hsync_end - mode->hsync_start;
	hbp = mode->htotal - mode->hsync_end;

	vfp = mode->vsync_start - mode->vdisplay;
	vsw = mode->vsync_end - mode->vsync_start;
	vbp = mode->vtotal - mode->vsync_end;

	dispc7_vp_write(dispc, hw_videoport, DISPC_VP_TIMING_H,
			FLD_VAL(hsw - 1, 7, 0) |
			FLD_VAL(hfp - 1, 19, 8) |
			FLD_VAL(hbp - 1, 31, 20));

	dispc7_vp_write(dispc, hw_videoport, DISPC_VP_TIMING_V,
			FLD_VAL(vsw - 1, 7, 0) |
			FLD_VAL(vfp, 19, 8) |
			FLD_VAL(vbp, 31, 20));

	ivs = !!(mode->flags & DRM_MODE_FLAG_NVSYNC);

	ihs = !!(mode->flags & DRM_MODE_FLAG_NHSYNC);

	ieo = !!(tstate->bus_flags & DRM_BUS_FLAG_DE_LOW);

	ipc = !!(tstate->bus_flags & DRM_BUS_FLAG_PIXDATA_NEGEDGE);

	/* always use the 'rf' setting */
	onoff = true;

	rf = !!(tstate->bus_flags & DRM_BUS_FLAG_SYNC_POSEDGE);

	/* always use aligned syncs */
	align = true;

	/* always use DE_HIGH for OLDI */
	if (dispc->feat->vp_bus_type[hw_videoport] == DISPC7_VP_OLDI)
		ieo = false;

	dispc7_vp_write(dispc, hw_videoport, DISPC_VP_POL_FREQ,
			FLD_VAL(align, 18, 18) |
			FLD_VAL(onoff, 17, 17) |
			FLD_VAL(rf, 16, 16) |
			FLD_VAL(ieo, 15, 15) |
			FLD_VAL(ipc, 14, 14) |
			FLD_VAL(ihs, 13, 13) |
			FLD_VAL(ivs, 12, 12));

	dispc7_vp_write(dispc, hw_videoport, DISPC_VP_SIZE_SCREEN,
			FLD_VAL(mode->hdisplay - 1, 11, 0) |
			FLD_VAL(mode->vdisplay - 1, 27, 16));

	VP_REG_FLD_MOD(dispc, hw_videoport, DISPC_VP_CONTROL, 1, 0, 0);
}

static void dispc7_vp_disable(struct dispc_device *dispc, u32 hw_videoport)
{
	VP_REG_FLD_MOD(dispc, hw_videoport, DISPC_VP_CONTROL, 0, 0, 0);
}

static void dispc7_vp_unprepare(struct dispc_device *dispc, u32 hw_videoport)
{
	if (dispc->feat->vp_bus_type[hw_videoport] == DISPC7_VP_OLDI) {
		dispc7_vp_write(dispc, hw_videoport, DISPC_VP_DSS_OLDI_CFG, 0);

		dispc7_oldi_tx_power(dispc, false);
	}
}

static bool dispc7_vp_go_busy(struct dispc_device *dispc,
			      u32 hw_videoport)
{
	return VP_REG_GET(dispc, hw_videoport, DISPC_VP_CONTROL, 5, 5);
}

static void dispc7_vp_go(struct dispc_device *dispc, u32 hw_videoport)
{
	WARN_ON(VP_REG_GET(dispc, hw_videoport, DISPC_VP_CONTROL, 5, 5));
	VP_REG_FLD_MOD(dispc, hw_videoport, DISPC_VP_CONTROL, 1, 5, 5);
}

enum c8_to_c12_mode { C8_TO_C12_REPLICATE, C8_TO_C12_MAX, C8_TO_C12_MIN };

static u16 c8_to_c12(u8 c8, enum c8_to_c12_mode mode)
{
	u16 c12;

	c12 = c8 << 4;

	switch (mode) {
	case C8_TO_C12_REPLICATE:
		/* Copy c8 4 MSB to 4 LSB for full scale c12 */
		c12 |= c8 >> 4;
		break;
	case C8_TO_C12_MAX:
		c12 |= 0xF;
		break;
	default:
	case C8_TO_C12_MIN:
		break;
	}

	return c12;
}

static u64 argb8888_to_argb12121212(u32 argb8888, enum c8_to_c12_mode m)
{
	u8 a, r, g, b;
	u64 v;

	a = (argb8888 >> 24) & 0xff;
	r = (argb8888 >> 16) & 0xff;
	g = (argb8888 >> 8) & 0xff;
	b = (argb8888 >> 0) & 0xff;

	v = ((u64)c8_to_c12(a, m) << 36) | ((u64)c8_to_c12(r, m) << 24) |
		((u64)c8_to_c12(g, m) << 12) | (u64)c8_to_c12(b, m);

	return v;
}

static void dispc7_vp_set_default_color(struct dispc_device *dispc,
					u32 hw_videoport, u32 default_color)
{
	u64 v;

	v = argb8888_to_argb12121212(default_color, C8_TO_C12_REPLICATE);

	dispc7_ovr_write(dispc, hw_videoport,
			 DISPC_OVR_DEFAULT_COLOR, v & 0xffffffff);
	dispc7_ovr_write(dispc, hw_videoport,
			 DISPC_OVR_DEFAULT_COLOR2, (v >> 32) & 0xffff);
}

static enum drm_mode_status dispc7_vp_mode_valid(struct dispc_device *dispc,
						 u32 hw_videoport,
						 const struct drm_display_mode *mode)
{
	u32 hsw, hfp, hbp, vsw, vfp, vbp;
	enum dispc7_vp_bus_type bus_type;
	int max_pclk;

	bus_type = dispc->feat->vp_bus_type[hw_videoport];

	max_pclk = dispc->feat->max_pclk_kHz[bus_type];

	if (WARN_ON(max_pclk == 0))
		return MODE_BAD;

	if (mode->clock > max_pclk)
		return MODE_CLOCK_HIGH;

	if (mode->hdisplay > 4096)
		return MODE_BAD;

	if (mode->vdisplay > 4096)
		return MODE_BAD;

	/* TODO: add interlace support */
	if (mode->flags & DRM_MODE_FLAG_INTERLACE)
		return MODE_NO_INTERLACE;

	/*
	 * Enforce the output width is divisible by 2. Actually this
	 * is only needed in following cases:
	 * - YUV output selected (BT656, BT1120)
	 * - Dithering enabled
	 * - TDM with TDMCycleFormat == 3
	 * But for simplicity we enforce that always.
	 */
	if ((mode->hdisplay % 2) != 0)
		return MODE_BAD_HVALUE;

	hfp = mode->hsync_start - mode->hdisplay;
	hsw = mode->hsync_end - mode->hsync_start;
	hbp = mode->htotal - mode->hsync_end;

	vfp = mode->vsync_start - mode->vdisplay;
	vsw = mode->vsync_end - mode->vsync_start;
	vbp = mode->vtotal - mode->vsync_end;

	if (hsw < 1 || hsw > 256 ||
	    hfp < 1 || hfp > 4096 ||
	    hbp < 1 || hbp > 4096)
		return MODE_BAD_HVALUE;

	if (vsw < 1 || vsw > 256 ||
	    vfp > 4095 || vbp > 4095)
		return MODE_BAD_VVALUE;

	return MODE_OK;
}

static int dispc7_vp_check(struct dispc_device *dispc, u32 hw_videoport,
			   const struct drm_crtc_state *state)
{
	const struct drm_display_mode *mode = &state->adjusted_mode;
	const struct tidss_crtc_state *tstate = to_tidss_crtc_state(state);
	const struct dispc7_bus_format *fmt;
	enum drm_mode_status ok;

	ok = dispc7_vp_mode_valid(dispc, hw_videoport, mode);
	if (ok != MODE_OK) {
		dev_dbg(dispc->dev, "%s: bad mode: %ux%u pclk %u kHz\n",
			__func__, mode->hdisplay, mode->vdisplay, mode->clock);
		return -EINVAL;
	}

	fmt = dispc7_vp_find_bus_fmt(dispc, hw_videoport, tstate->bus_format,
				     tstate->bus_flags);
	if (!fmt) {
		dev_dbg(dispc->dev, "%s: Unsupported bus format: %u\n",
			__func__, tstate->bus_format);
		return -EINVAL;
	}

	if (dispc->feat->vp_bus_type[hw_videoport] != DISPC7_VP_OLDI &&
	    fmt->is_oldi_fmt) {
		dev_dbg(dispc->dev, "%s: %s is not OLDI-port\n",
			__func__, dispc->feat->vp_name[hw_videoport]);
		return -EINVAL;
	}

	return 0;
}

static int dispc7_vp_enable_clk(struct dispc_device *dispc, u32 hw_videoport)
{
	int ret = clk_prepare_enable(dispc->vp_clk[hw_videoport]);

	if (ret)
		dev_err(dispc->dev, "%s: enabling clk failed: %d\n", __func__,
			ret);

	return ret;
}

static void dispc7_vp_disable_clk(struct dispc_device *dispc, u32 hw_videoport)
{
	clk_disable_unprepare(dispc->vp_clk[hw_videoport]);
}

/*
 * Calculate the percentage difference between the requested pixel clock rate
 * and the effective rate resulting from calculating the clock divider value.
 */
static unsigned int dispc7_pclk_diff(unsigned long rate,
				     unsigned long real_rate)
{
	int r = rate / 100, rr = real_rate / 100;

	return (unsigned int)(abs(((rr - r) * 100) / r));
}

static int dispc7_vp_set_clk_rate(struct dispc_device *dispc, u32 hw_videoport,
				  unsigned long rate)
{
	int r;
	unsigned long new_rate;

	r = clk_set_rate(dispc->vp_clk[hw_videoport], rate);
	if (r) {
		dev_err(dispc->dev, "vp%d: failed to set clk rate to %lu\n",
			hw_videoport, rate);
		return r;
	}

	new_rate = clk_get_rate(dispc->vp_clk[hw_videoport]);

	if (dispc7_pclk_diff(rate, new_rate) > 5)
		dev_warn(dispc->dev,
			 "vp%d: Clock rate %lu differs over 5%% from requsted %lu\n",
			 hw_videoport, new_rate, rate);

	dev_dbg(dispc->dev, "vp%d: new rate %lu Hz (requested %lu Hz)\n",
		hw_videoport, clk_get_rate(dispc->vp_clk[hw_videoport]), rate);

	return 0;
}

/* OVR */
static void dispc7_am6_ovr_set_plane(struct dispc_device *dispc,
				     u32 hw_plane, u32 hw_videoport,
				     u32 x, u32 y, u32 zpos)
{
	OVR_REG_FLD_MOD(dispc, hw_videoport, DISPC_OVR_ATTRIBUTES(zpos),
			hw_plane, 4, 1);
	OVR_REG_FLD_MOD(dispc, hw_videoport, DISPC_OVR_ATTRIBUTES(zpos),
			x, 17, 6);
	OVR_REG_FLD_MOD(dispc, hw_videoport, DISPC_OVR_ATTRIBUTES(zpos),
			y, 30, 19);
}

static void dispc7_j721e_ovr_set_plane(struct dispc_device *dispc,
				      u32 hw_plane, u32 hw_videoport,
				      u32 x, u32 y, u32 zpos)
{
	OVR_REG_FLD_MOD(dispc, hw_videoport, DISPC_OVR_ATTRIBUTES(zpos),
			hw_plane, 4, 1);
	OVR_REG_FLD_MOD(dispc, hw_videoport, DISPC_OVR_ATTRIBUTES2(zpos),
			x, 13, 0);
	OVR_REG_FLD_MOD(dispc, hw_videoport, DISPC_OVR_ATTRIBUTES2(zpos),
			y, 29, 16);
}

static void dispc7_ovr_set_plane(struct dispc_device *dispc,
				 u32 hw_plane, u32 hw_videoport,
				 u32 x, u32 y, u32 zpos)
{
	switch (dispc->feat->subrev) {
	case DSS7_AM6:
		dispc7_am6_ovr_set_plane(dispc, hw_plane, hw_videoport,
					 x, y, zpos);
		break;
	case DSS7_J721E:
		dispc7_j721e_ovr_set_plane(dispc, hw_plane, hw_videoport,
					  x, y, zpos);
		break;
	default:
		WARN_ON(1);
		break;
	}
}

static void dispc7_ovr_enable_plane(struct dispc_device *dispc,
				    u32 hw_videoport, u32 zpos, bool enable)
{
	OVR_REG_FLD_MOD(dispc, hw_videoport, DISPC_OVR_ATTRIBUTES(zpos),
			!!enable, 0, 0);
}

/* CSC */
enum csc_ctm {
	CSC_RR, CSC_RG, CSC_RB,
	CSC_GR, CSC_GG, CSC_GB,
	CSC_BR, CSC_BG, CSC_BB,
};

enum csc_yuv2rgb {
	CSC_RY, CSC_RCb, CSC_RCr,
	CSC_GY, CSC_GCb, CSC_GCr,
	CSC_BY, CSC_BCb, CSC_BCr,
};

enum csc_rgb2yuv {
	CSC_YR,  CSC_YG,  CSC_YB,
	CSC_CbR, CSC_CbG, CSC_CbB,
	CSC_CrR, CSC_CrG, CSC_CrB,
};

struct dispc7_csc_coef {
	void (*to_regval)(const struct dispc7_csc_coef *csc, u32 *regval);
	int m[9];
	int preoffset[3];
	int postoffset[3];
	enum { CLIP_LIMITED_RANGE = 0, CLIP_FULL_RANGE = 1, } cliping;
	const char *name;
};

#define DISPC7_CSC_REGVAL_LEN 8

static void dispc7_csc_offset_regval(const struct dispc7_csc_coef *csc,
				     u32 *regval)
{
#define OVAL(x, y) (FLD_VAL(x, 15, 3) | FLD_VAL(y, 31, 19))
	regval[5] = OVAL(csc->preoffset[0], csc->preoffset[1]);
	regval[6] = OVAL(csc->preoffset[2], csc->postoffset[0]);
	regval[7] = OVAL(csc->postoffset[1], csc->postoffset[2]);
#undef OVAL
}

#define CVAL(x, y) (FLD_VAL(x, 10, 0) | FLD_VAL(y, 26, 16))
static void dispc7_csc_yuv2rgb_regval(const struct dispc7_csc_coef *csc,
				      u32 *regval)
{
	regval[0] = CVAL(csc->m[CSC_RY], csc->m[CSC_RCr]);
	regval[1] = CVAL(csc->m[CSC_RCb], csc->m[CSC_GY]);
	regval[2] = CVAL(csc->m[CSC_GCr], csc->m[CSC_GCb]);
	regval[3] = CVAL(csc->m[CSC_BY], csc->m[CSC_BCr]);
	regval[4] = CVAL(csc->m[CSC_BCb], 0);

	dispc7_csc_offset_regval(csc, regval);
}

static void __maybe_unused dispc7_csc_rgb2yuv_regval(const struct dispc7_csc_coef *csc,
				      u32 *regval)
{
	regval[0] = CVAL(csc->m[CSC_YR], csc->m[CSC_YG]);
	regval[1] = CVAL(csc->m[CSC_YB], csc->m[CSC_CrR]);
	regval[2] = CVAL(csc->m[CSC_CrG], csc->m[CSC_CrB]);
	regval[3] = CVAL(csc->m[CSC_CbR], csc->m[CSC_CbG]);
	regval[4] = CVAL(csc->m[CSC_CbB], 0);

	dispc7_csc_offset_regval(csc, regval);
}

static void dispc7_csc_cpr_regval(const struct dispc7_csc_coef *csc,
				  u32 *regval)
{
	regval[0] = CVAL(csc->m[CSC_RR], csc->m[CSC_RG]);
	regval[1] = CVAL(csc->m[CSC_RB], csc->m[CSC_GR]);
	regval[2] = CVAL(csc->m[CSC_GG], csc->m[CSC_GB]);
	regval[3] = CVAL(csc->m[CSC_BR], csc->m[CSC_BG]);
	regval[4] = CVAL(csc->m[CSC_BB], 0);

	dispc7_csc_offset_regval(csc, regval);
}
#undef CVAL


static void dispc7_vid_write_csc(struct dispc_device *dispc, u32 hw_plane,
				 const struct dispc7_csc_coef *csc)
{
	static const u16 dispc_vid_csc_coef_reg[DISPC7_CSC_REGVAL_LEN] = {
		DISPC_VID_CSC_COEF(0), DISPC_VID_CSC_COEF(1),
		DISPC_VID_CSC_COEF(2), DISPC_VID_CSC_COEF(3),
		DISPC_VID_CSC_COEF(4), DISPC_VID_CSC_COEF(5),
		DISPC_VID_CSC_COEF(6), DISPC_VID_CSC_COEF7,
	};
	u32 regval[DISPC7_CSC_REGVAL_LEN];
	unsigned int i;

	csc->to_regval(csc, regval);

	for (i = 0; i < ARRAY_SIZE(dispc_vid_csc_coef_reg); i++)
		dispc7_vid_write(dispc, hw_plane, dispc_vid_csc_coef_reg[i],
				regval[i]);
}

static void dispc7_wb_write_csc(struct dispc_device *dispc,
				 const struct dispc7_csc_coef *csc)
{
	static const u16 dispc_wb_csc_coef_reg[DISPC7_CSC_REGVAL_LEN] = {
		DISPC_WB_CSC_COEF(0), DISPC_WB_CSC_COEF(1),
		DISPC_WB_CSC_COEF(2), DISPC_WB_CSC_COEF(3),
		DISPC_WB_CSC_COEF(4), DISPC_WB_CSC_COEF(5),
		DISPC_WB_CSC_COEF(6), DISPC_WB_CSC_COEF7,
	};
	u32 regval[DISPC7_CSC_REGVAL_LEN];
	unsigned int i;

	csc->to_regval(csc, regval);

	for (i = 0; i < ARRAY_SIZE(dispc_wb_csc_coef_reg); i++)
		dispc7_wb_write(dispc, dispc_wb_csc_coef_reg[i],
				regval[i]);
}

/* YUV -> RGB, ITU-R BT.601, full range */
static const struct dispc7_csc_coef csc_yuv2rgb_bt601_full = {
	dispc7_csc_yuv2rgb_regval,
	{ 256,   0,  358,	/* ry, rcb, rcr |1.000  0.000  1.402|*/
	  256, -88, -182,	/* gy, gcb, gcr |1.000 -0.344 -0.714|*/
	  256, 452,    0, },	/* by, bcb, bcr |1.000  1.772  0.000|*/
	{    0, -2048, -2048, },	/* full range */
	{    0,     0,     0, },
	CLIP_FULL_RANGE,
	"BT.601 Full",
};

/* YUV -> RGB, ITU-R BT.601, limited range */
static const struct dispc7_csc_coef csc_yuv2rgb_bt601_lim = {
	dispc7_csc_yuv2rgb_regval,
	{ 298,    0,  409,	/* ry, rcb, rcr |1.164  0.000  1.596|*/
	  298, -100, -208,	/* gy, gcb, gcr |1.164 -0.392 -0.813|*/
	  298,  516,    0, },	/* by, bcb, bcr |1.164  2.017  0.000|*/
	{ -256, -2048, -2048, },	/* limited range */
	{    0,     0,     0, },
	CLIP_FULL_RANGE,
	"BT.601 Limited",
};

/* YUV -> RGB, ITU-R BT.709, full range */
static const struct dispc7_csc_coef csc_yuv2rgb_bt709_full = {
	dispc7_csc_yuv2rgb_regval,
	{ 256,	  0,  402,	/* ry, rcb, rcr |1.000	0.000  1.570|*/
	  256,  -48, -120,	/* gy, gcb, gcr |1.000 -0.187 -0.467|*/
	  256,  475,    0, },	/* by, bcb, bcr |1.000	1.856  0.000|*/
	{    0, -2048, -2048, },	/* full range */
	{    0,     0,     0, },
	CLIP_FULL_RANGE,
	"BT.709 Full",
};

/* YUV -> RGB, ITU-R BT.709, limited range */
static const struct dispc7_csc_coef csc_yuv2rgb_bt709_lim = {
	dispc7_csc_yuv2rgb_regval,
	{ 298,    0,  459,	/* ry, rcb, rcr |1.164  0.000  1.793|*/
	  298,  -55, -136,	/* gy, gcb, gcr |1.164 -0.213 -0.533|*/
	  298,  541,    0, },	/* by, bcb, bcr |1.164  2.112  0.000|*/
	{ -256, -2048, -2048, },	/* limited range */
	{    0,     0,     0, },
	CLIP_FULL_RANGE,
	"BT.709 Limited",
};

/* RGB -> YUV, ITU-R BT.601, full range */
static const struct dispc7_csc_coef csc_rgb2yuv_bt601_full = {
	dispc7_csc_rgb2yuv_regval,
	{ 77,  150,  29,	/* yr,   yg,  yb | 0.299  0.587  0.114|*/
	 -43,  -85, 128,	/* cbr, cbg, cbb |-0.173 -0.339  0.511|*/
	 128, -107, -21 },	/* crr, crg, crb | 0.511 -0.428 -0.083|*/
	{    0,     0,     0, },
	{    0,  2048,  2048, },	/* full range */
	CLIP_FULL_RANGE,
	"BT.601 Full",
};

/* RGB -> YUV, ITU-R BT.601, limited range */
static const struct dispc7_csc_coef csc_rgb2yuv_bt601_lim = {
	dispc7_csc_rgb2yuv_regval,
	{ 66,  129,  25,	/* yr,   yg,  yb | 0.257  0.504  0.098|*/
	 -38,  -74, 112,	/* cbr, cbg, cbb |-0.148 -0.291  0.439|*/
	 112,  -94, -18 },	/* crr, crg, crb | 0.439 -0.368 -0.071|*/
	{    0,     0,     0, },
	{  256,  2048,  2048, },	/* limited range */
	CLIP_FULL_RANGE,
	"BT.601 Limited",
};

/* RGB -> YUV, ITU-R BT.709, full range */
static const struct dispc7_csc_coef csc_rgb2yuv_bt709_full = {
	dispc7_csc_rgb2yuv_regval,
	{ 54,  183,  18,	/* yr,   yg,  yb | 0.1826  0.6142  0.0620|*/
	 -30, -101, 131,	/* cbr, cbg, cbb |-0.1006 -0.3386  0.4392|*/
	 131, -119, -12, },	/* crr, crg, crb | 0.4392 -0.3989 -0.0403|*/
	{    0,     0,     0, },
	{    0,  2048,  2048, },	/* full range */
	CLIP_FULL_RANGE,
	"BT.709 Full",
};

/* RGB -> YUV, ITU-R BT.709, limited range */
static const struct dispc7_csc_coef csc_rgb2yuv_bt709_lim = {
	dispc7_csc_rgb2yuv_regval,
	{ 47,  157,   16,	/* yr,   yg,  yb | 0.1826  0.6142  0.0620|*/
	 -26,  -87,  112,	/* cbr, cbg, cbb |-0.1006 -0.3386  0.4392|*/
	 112, -102,  -10, },	/* crr, crg, crb | 0.4392 -0.3989 -0.0403|*/
	{    0,     0,     0, },
	{  256,  2048,  2048, },	/* limited range */
	CLIP_FULL_RANGE,
	"BT.709 Limited",
};

struct dispc7_csc_entry {
	enum drm_color_encoding encoding;
	enum drm_color_range range;
	const struct dispc7_csc_coef *csc;
};

static const struct dispc7_csc_entry dispc7_yuv2rgb_table[] = {
	{ DRM_COLOR_YCBCR_BT601, DRM_COLOR_YCBCR_FULL_RANGE,
	  &csc_yuv2rgb_bt601_full, },
	{ DRM_COLOR_YCBCR_BT601, DRM_COLOR_YCBCR_LIMITED_RANGE,
	  &csc_yuv2rgb_bt601_lim, },
	{ DRM_COLOR_YCBCR_BT709, DRM_COLOR_YCBCR_FULL_RANGE,
	  &csc_yuv2rgb_bt709_full, },
	{ DRM_COLOR_YCBCR_BT709, DRM_COLOR_YCBCR_LIMITED_RANGE,
	  &csc_yuv2rgb_bt709_lim, },
};

static const struct dispc7_csc_entry dispc7_rgb2yuv_table[] = {
	{ DRM_COLOR_YCBCR_BT601, DRM_COLOR_YCBCR_FULL_RANGE,
	  &csc_rgb2yuv_bt601_full, },
	{ DRM_COLOR_YCBCR_BT601, DRM_COLOR_YCBCR_LIMITED_RANGE,
	  &csc_rgb2yuv_bt601_lim, },
	{ DRM_COLOR_YCBCR_BT709, DRM_COLOR_YCBCR_FULL_RANGE,
	  &csc_rgb2yuv_bt709_full, },
	{ DRM_COLOR_YCBCR_BT709, DRM_COLOR_YCBCR_LIMITED_RANGE,
	  &csc_rgb2yuv_bt709_lim, },
};

enum dispc7_csc_direction {
	DISPC7_YUV2RGB,
	DISPC7_RGB2YUV,
};

static const
struct dispc7_csc_coef *dispc7_find_csc(enum dispc7_csc_direction direction,
					enum drm_color_encoding encoding,
					enum drm_color_range range)
{
	unsigned int i;
	const struct dispc7_csc_entry *csc_table;

	if (direction == DISPC7_YUV2RGB)
		csc_table = dispc7_yuv2rgb_table;
	else
		csc_table = dispc7_rgb2yuv_table;

	for (i = 0; i < ARRAY_SIZE(dispc7_yuv2rgb_table); i++) {
		if (csc_table[i].encoding == encoding &&
		    csc_table[i].range == range) {
			return csc_table[i].csc;
		}
	}
	return NULL;
}

static void dispc7_vid_csc_setup(struct dispc_device *dispc, u32 hw_plane,
				 const struct drm_plane_state *state)
{
	static const struct dispc7_csc_coef *coef;

	coef = dispc7_find_csc(DISPC7_YUV2RGB, state->color_encoding,
			       state->color_range);
	if (!coef) {
		dev_err(dispc->dev, "%s: YUV2RGB CSC (%u,%u) not found\n",
			__func__, state->color_encoding, state->color_range);
		return;
	}

	dispc7_vid_write_csc(dispc, hw_plane, coef);
}

static void dispc7_vid_csc_enable(struct dispc_device *dispc, u32 hw_plane,
				  bool enable)
{
	VID_REG_FLD_MOD(dispc, hw_plane, DISPC_VID_ATTRIBUTES, !!enable, 9, 9);
}

static void dispc7_wb_csc_setup(struct dispc_device *dispc,
				const struct drm_plane_state *state)
{
	static const struct dispc7_csc_coef *coef;

	coef = dispc7_find_csc(DISPC7_RGB2YUV, state->color_encoding,
			       state->color_range);
	if (!coef) {
		dev_err(dispc->dev, "%s: RGB2YUV CSC (%u,%u) not found\n",
			__func__, state->color_encoding, state->color_range);
		return;
	}
	dispc7_wb_write_csc(dispc, coef);
}

static void dispc7_wb_csc_enable(struct dispc_device *dispc, bool enable)
{
	WB_REG_FLD_MOD(dispc, DISPC_WB_ATTRIBUTES, !!enable, 11, 11);
}

/* SCALER */

static u32 dispc7_calc_fir_inc(u32 in, u32 out)
{
	return (u32)div_u64(0x200000ull * in, out);
}

enum dispc7_vid_fir_coef_set {
	DISPC7_VID_FIR_COEF_HORIZ,
	DISPC7_VID_FIR_COEF_HORIZ_UV,
	DISPC7_VID_FIR_COEF_VERT,
	DISPC7_VID_FIR_COEF_VERT_UV,
};

static void dispc7_vid_write_fir_coefs(struct dispc_device *dispc,
				       u32 hw_plane,
				       enum dispc7_vid_fir_coef_set coef_set,
				       const struct tidss_scale_coefs *coefs)
{
	static const u16 c0_regs[] = {
		[DISPC7_VID_FIR_COEF_HORIZ] = DISPC_VID_FIR_COEFS_H0,
		[DISPC7_VID_FIR_COEF_HORIZ_UV] = DISPC_VID_FIR_COEFS_H0_C,
		[DISPC7_VID_FIR_COEF_VERT] = DISPC_VID_FIR_COEFS_V0,
		[DISPC7_VID_FIR_COEF_VERT_UV] = DISPC_VID_FIR_COEFS_V0_C,
	};

	static const u16 c12_regs[] = {
		[DISPC7_VID_FIR_COEF_HORIZ] = DISPC_VID_FIR_COEFS_H12,
		[DISPC7_VID_FIR_COEF_HORIZ_UV] = DISPC_VID_FIR_COEFS_H12_C,
		[DISPC7_VID_FIR_COEF_VERT] = DISPC_VID_FIR_COEFS_V12,
		[DISPC7_VID_FIR_COEF_VERT_UV] = DISPC_VID_FIR_COEFS_V12_C,
	};

	const u16 c0_base = c0_regs[coef_set];
	const u16 c12_base = c12_regs[coef_set];
	int phase;

	if (!coefs) {
		dev_err(dispc->dev, "%s: No coefficients given.\n", __func__);
		return;
	}

	for (phase = 0; phase <= 8; ++phase) {
		u16 reg = c0_base + phase * 4;
		u16 c0 = coefs->c0[phase];

		dispc7_vid_write(dispc, hw_plane, reg, c0);
	}

	for (phase = 0; phase <= 15; ++phase) {
		u16 reg = c12_base + phase * 4;
		s16 c1, c2;
		u32 c12;

		c1 = coefs->c1[phase];
		c2 = coefs->c2[phase];
		c12 = FLD_VAL(c1, 19, 10) | FLD_VAL(c2, 29, 20);

		dispc7_vid_write(dispc, hw_plane, reg, c12);
	}
}

enum dispc7_wb_fir_coef_set {
	DISPC7_WB_FIR_COEF_HORIZ,
	DISPC7_WB_FIR_COEF_HORIZ_UV,
	DISPC7_WB_FIR_COEF_VERT,
	DISPC7_WB_FIR_COEF_VERT_UV,
};

static void dispc7_wb_write_fir_coefs(struct dispc_device *dispc,
				      enum dispc7_wb_fir_coef_set coef_set,
				      const struct tidss_scale_coefs *coefs)
{
	static const u16 c0_regs[] = {
		[DISPC7_WB_FIR_COEF_HORIZ] = DISPC_WB_FIR_COEFS_H0,
		[DISPC7_WB_FIR_COEF_HORIZ_UV] = DISPC_WB_FIR_COEFS_H0_C,
		[DISPC7_WB_FIR_COEF_VERT] = DISPC_WB_FIR_COEFS_V0,
		[DISPC7_WB_FIR_COEF_VERT_UV] = DISPC_WB_FIR_COEFS_V0_C,
	};

	static const u16 c12_regs[] = {
		[DISPC7_WB_FIR_COEF_HORIZ] = DISPC_WB_FIR_COEFS_H12,
		[DISPC7_WB_FIR_COEF_HORIZ_UV] = DISPC_WB_FIR_COEFS_H12_C,
		[DISPC7_WB_FIR_COEF_VERT] = DISPC_WB_FIR_COEFS_V12,
		[DISPC7_WB_FIR_COEF_VERT_UV] = DISPC_WB_FIR_COEFS_V12_C,
	};

	const u16 c0_base = c0_regs[coef_set];
	const u16 c12_base = c12_regs[coef_set];
	int phase;

	if (!coefs) {
		dev_err(dispc->dev, "%s: No coefficients given.\n", __func__);
		return;
	}

	for (phase = 0; phase <= 8; ++phase) {
		u16 reg = c0_base + phase * 4;
		u16 c0 = coefs->c0[phase];

		dispc7_wb_write(dispc, reg, c0);
	}

	for (phase = 0; phase <= 15; ++phase) {
		u16 reg = c12_base + phase * 4;
		s16 c1, c2;
		u32 c12;

		c1 = coefs->c1[phase];
		c2 = coefs->c2[phase];
		c12 = FLD_VAL(c1, 19, 10) | FLD_VAL(c2, 29, 20);

		dispc7_wb_write(dispc, reg, c12);
	}
}

static bool dispc7_fourcc_is_yuv(u32 fourcc)
{
	switch (fourcc) {
	case DRM_FORMAT_YUYV:
	case DRM_FORMAT_UYVY:
	case DRM_FORMAT_NV12:
		return true;
	default:
		return false;
	}
}

struct dispc7_scaling_params {
	int xinc, yinc;
	u32 in_w, in_h, in_w_uv, in_h_uv;
	u32 out_w, out_h, out_w_uv, out_h_uv;
	u32 fir_xinc, fir_yinc, fir_xinc_uv, fir_yinc_uv;
	bool scale_x, scale_y;
	const struct tidss_scale_coefs *xcoef, *ycoef, *xcoef_uv, *ycoef_uv;
	bool five_taps;
};

static int dispc7_vid_calc_scaling(struct dispc_device *dispc,
				   const struct drm_plane_state *state,
				   struct dispc7_scaling_params *sp,
				   bool lite_plane)
{
	const struct dispc7_features_scaling *f = &dispc->feat->scaling;
	u32 fourcc = state->fb->format->format;
	u32 in_width_max_5tap = f->in_width_max_5tap_rgb;
	u32 in_width_max_3tap = f->in_width_max_3tap_rgb;
	u32 downscale_limit;
	u32 in_width_max;

	memset(sp, 0, sizeof(*sp));
	sp->xinc = sp->yinc = 1;
	sp->in_w = sp->in_w_uv = state->src_w >> 16;
	sp->in_h = sp->in_h_uv = state->src_h >> 16;

	sp->scale_x = sp->in_w != state->crtc_w;
	sp->scale_y = sp->in_h != state->crtc_h;

	if (dispc7_fourcc_is_yuv(fourcc)) {
		in_width_max_5tap = f->in_width_max_5tap_yuv;
		in_width_max_3tap = f->in_width_max_3tap_yuv;

		sp->in_w_uv >>= 1;
		sp->scale_x = true;

		if (fourcc == DRM_FORMAT_NV12) {
			sp->in_h_uv >>= 1;
			sp->scale_y = true;
		}
	}

	/* Skip the rest if no scaling is used */
	if ((!sp->scale_x && !sp->scale_y) || lite_plane)
		return 0;

	if (sp->in_w > in_width_max_5tap) {
		sp->five_taps = false;
		in_width_max = in_width_max_3tap;
		downscale_limit = f->downscale_limit_3tap;
	} else {
		sp->five_taps = true;
		in_width_max = in_width_max_5tap;
		downscale_limit = f->downscale_limit_5tap;
	}

	if (sp->scale_x) {
		sp->fir_xinc = dispc7_calc_fir_inc(sp->in_w, state->crtc_w);

		if (sp->fir_xinc < dispc7_calc_fir_inc(1, f->upscale_limit)) {
			dev_dbg(dispc->dev,
				"%s: X-scaling factor %u/%u > %u\n",
				__func__, state->crtc_w, state->src_w >> 16,
				f->upscale_limit);
			return -EINVAL;
		}

		if (sp->fir_xinc >= dispc7_calc_fir_inc(downscale_limit, 1)) {
			sp->xinc = DIV_ROUND_UP(DIV_ROUND_UP(sp->in_w,
							     state->crtc_w),
						downscale_limit);

			if (sp->xinc > f->xinc_max) {
				dev_dbg(dispc->dev,
					"%s: X-scaling factor %u/%u < 1/%u\n",
					__func__, state->crtc_w,
					state->src_w >> 16,
					downscale_limit * f->xinc_max);
				return -EINVAL;
			}

			sp->in_w = (state->src_w >> 16) / sp->xinc;
		}

		while (sp->in_w > in_width_max) {
			sp->xinc++;
			sp->in_w = (state->src_w >> 16) / sp->xinc;
		}

		if (sp->xinc > f->xinc_max) {
			dev_dbg(dispc->dev,
				"%s: Too wide input bufer %u > %u\n", __func__,
				state->src_w >> 16, in_width_max * f->xinc_max);
			return -EINVAL;
		}

		/*
		 * We need even line length for YUV formats. Decimation
		 * can lead to odd length, so we need to make it even
		 * again.
		 */
		if (dispc7_fourcc_is_yuv(fourcc))
			sp->in_w &= ~1;

		sp->fir_xinc = dispc7_calc_fir_inc(sp->in_w, state->crtc_w);
	}

	if (sp->scale_y) {
		sp->fir_yinc = dispc7_calc_fir_inc(sp->in_h, state->crtc_h);

		if (sp->fir_yinc < dispc7_calc_fir_inc(1, f->upscale_limit)) {
			dev_dbg(dispc->dev,
				"%s: Y-scaling factor %u/%u > %u\n",
				__func__, state->crtc_h, state->src_h >> 16,
				f->upscale_limit);
			return -EINVAL;
		}

		if (sp->fir_yinc >= dispc7_calc_fir_inc(downscale_limit, 1)) {
			sp->yinc = DIV_ROUND_UP(DIV_ROUND_UP(sp->in_h,
							     state->crtc_h),
						downscale_limit);

			sp->in_h /= sp->yinc;
			sp->fir_yinc = dispc7_calc_fir_inc(sp->in_h,
							   state->crtc_h);
		}
	}

	dev_dbg(dispc->dev,
		"%s: %ux%u decim %ux%u -> %ux%u firinc %u.%03ux%u.%03u taps %u -> %ux%u\n",
		__func__, state->src_w >> 16, state->src_h >> 16,
		sp->xinc, sp->yinc, sp->in_w, sp->in_h,
		sp->fir_xinc / 0x200000u,
		((sp->fir_xinc & 0x1FFFFFu) * 999u) / 0x1FFFFFu,
		sp->fir_yinc / 0x200000u,
		((sp->fir_yinc & 0x1FFFFFu) * 999u) / 0x1FFFFFu,
		sp->five_taps ? 5 : 3,
		state->crtc_w, state->crtc_h);

	if (dispc7_fourcc_is_yuv(fourcc)) {
		if (sp->scale_x) {
			sp->in_w_uv /= sp->xinc;
			sp->fir_xinc_uv = dispc7_calc_fir_inc(sp->in_w_uv,
							      state->crtc_w);
			sp->xcoef_uv = tidss_get_scale_coefs(dispc->dev,
							     sp->fir_xinc_uv,
							     true);
		}
		if (sp->scale_y) {
			sp->in_h_uv /= sp->yinc;
			sp->fir_yinc_uv = dispc7_calc_fir_inc(sp->in_h_uv,
							      state->crtc_h);
			sp->ycoef_uv = tidss_get_scale_coefs(dispc->dev,
							     sp->fir_yinc_uv,
							     sp->five_taps);
		}
	}

	if (sp->scale_x)
		sp->xcoef = tidss_get_scale_coefs(dispc->dev, sp->fir_xinc,
						  true);

	if (sp->scale_y)
		sp->ycoef = tidss_get_scale_coefs(dispc->dev, sp->fir_yinc,
						  sp->five_taps);

	return 0;
}

static int dispc7_wb_calc_scaling(struct dispc_device *dispc,
				  const struct drm_plane_state *state,
				  struct dispc7_scaling_params *sp)
{
	const struct dispc7_features_scaling *f = &dispc->feat->scaling;
	u32 fourcc = state->fb->format->format;
	u32 in_width_max_5tap = f->in_width_max_5tap_rgb;
	u32 in_width_max_3tap = f->in_width_max_3tap_rgb;
	u32 downscale_limit;
	u32 in_width_max;

	memset(sp, 0, sizeof(*sp));
	sp->xinc = sp->yinc = 1;
	sp->in_w = sp->in_w_uv = state->src_w >> 16;
	sp->in_h = sp->in_h_uv = state->src_h >> 16;
	sp->out_w = sp->out_w_uv = state->crtc_w;
	sp->out_h = sp->out_h_uv = state->crtc_h;

	sp->scale_x = sp->in_w != sp->out_w;
	sp->scale_y = sp->in_h != sp->out_h;

	if (dispc7_fourcc_is_yuv(fourcc)) {
		in_width_max_5tap = f->in_width_max_5tap_yuv;
		in_width_max_3tap = f->in_width_max_3tap_yuv;

		sp->out_w_uv >>= 1;
		sp->scale_x = true;

		if (fourcc == DRM_FORMAT_NV12) {
			sp->out_h_uv >>= 1;
			sp->scale_y = true;
		}
	}

	/* Skip the rest if no scaling is used */
	if (!sp->scale_x && !sp->scale_y)
		return 0;

	if (sp->in_w > in_width_max_5tap) {
		sp->five_taps = false;
		in_width_max = in_width_max_3tap;
		downscale_limit = f->downscale_limit_3tap;
	} else {
		sp->five_taps = true;
		in_width_max = in_width_max_5tap;
		downscale_limit = f->downscale_limit_5tap;
	}

	if (sp->scale_x) {
		sp->fir_xinc = dispc7_calc_fir_inc(sp->in_w, sp->out_w);

		if (sp->fir_xinc < dispc7_calc_fir_inc(1, f->upscale_limit)) {
			dev_dbg(dispc->dev,
				"%s: X-scaling factor %u/%u > %u\n",
				__func__, state->crtc_w, state->src_w >> 16,
				f->upscale_limit);
			return -EINVAL;
		}

		if (sp->fir_xinc >= dispc7_calc_fir_inc(downscale_limit, 1)) {
			sp->xinc = DIV_ROUND_UP(DIV_ROUND_UP(sp->in_w,
							     sp->out_w),
						downscale_limit);

			if (sp->xinc > f->xinc_max) {
				dev_dbg(dispc->dev,
					"%s: X-scaling factor %u/%u < 1/%u\n",
					__func__, state->crtc_w,
					state->src_w >> 16,
					downscale_limit * f->xinc_max);
				return -EINVAL;
			}

			sp->in_w = (state->src_w >> 16) / sp->xinc;
		}

		while (sp->in_w > in_width_max) {
			sp->xinc++;
			sp->in_w = (state->src_w >> 16) / sp->xinc;
		}

		if (sp->xinc > f->xinc_max) {
			dev_dbg(dispc->dev,
				"%s: Too wide input bufer %u > %u\n", __func__,
				state->src_w >> 16, in_width_max * f->xinc_max);
			return -EINVAL;
		}

		/*
		 * We need even line length for YUV formats. Decimation
		 * can lead to odd length, so we need to make it even
		 * again.
		 */
		if (dispc7_fourcc_is_yuv(fourcc))
			sp->in_w &= ~1;

		sp->fir_xinc = dispc7_calc_fir_inc(sp->in_w, sp->out_w);
	}

	if (sp->scale_y) {
		sp->fir_yinc = dispc7_calc_fir_inc(sp->in_h, sp->out_h);

		if (sp->fir_yinc < dispc7_calc_fir_inc(1, f->upscale_limit)) {
			dev_dbg(dispc->dev,
				"%s: Y-scaling factor %u/%u > %u\n",
				__func__, state->crtc_h, state->src_h >> 16,
				f->upscale_limit);
			return -EINVAL;
		}

		if (sp->fir_yinc >= dispc7_calc_fir_inc(downscale_limit, 1)) {
			sp->yinc = DIV_ROUND_UP(DIV_ROUND_UP(sp->in_h,
							     sp->out_h),
						downscale_limit);

			sp->in_h /= sp->yinc;
			sp->fir_yinc = dispc7_calc_fir_inc(sp->in_h,
							   sp->out_h);
		}
	}

	dev_dbg(dispc->dev,
		"%s: %ux%u decim %ux%u -> %ux%u firinc %u.%03ux%u.%03u taps %u -> %ux%u\n",
		__func__, state->src_w >> 16, state->src_h >> 16,
		sp->xinc, sp->yinc, sp->in_w, sp->in_h,
		sp->fir_xinc / 0x200000u,
		((sp->fir_xinc & 0x1FFFFFu) * 999u) / 0x1FFFFFu,
		sp->fir_yinc / 0x200000u,
		((sp->fir_yinc & 0x1FFFFFu) * 999u) / 0x1FFFFFu,
		sp->five_taps ? 5 : 3,
		state->crtc_w, state->crtc_h);

	if (dispc7_fourcc_is_yuv(fourcc)) {
		if (sp->scale_x) {
			sp->in_w_uv /= sp->xinc;
			sp->fir_xinc_uv = dispc7_calc_fir_inc(sp->in_w_uv,
							      sp->out_w_uv);
			sp->xcoef_uv = tidss_get_scale_coefs(dispc->dev,
							     sp->fir_xinc_uv,
							     true);
		}
		if (sp->scale_y) {
			sp->in_h_uv /= sp->yinc;
			sp->fir_yinc_uv = dispc7_calc_fir_inc(sp->in_h_uv,
							      sp->out_h_uv);
			sp->ycoef_uv = tidss_get_scale_coefs(dispc->dev,
							     sp->fir_yinc_uv,
							     sp->five_taps);
		}
	}

	if (sp->scale_x)
		sp->xcoef = tidss_get_scale_coefs(dispc->dev, sp->fir_xinc,
						  true);

	if (sp->scale_y)
		sp->ycoef = tidss_get_scale_coefs(dispc->dev, sp->fir_yinc,
						  sp->five_taps);

	return 0;
}

static void dispc7_vid_set_scaling(struct dispc_device *dispc,
				   u32 hw_plane,
				   struct dispc7_scaling_params *sp,
				   u32 fourcc)
{
	/* HORIZONTAL RESIZE ENABLE */
	VID_REG_FLD_MOD(dispc, hw_plane, DISPC_VID_ATTRIBUTES,
			sp->scale_x, 7, 7);

	/* VERTICAL RESIZE ENABLE */
	VID_REG_FLD_MOD(dispc, hw_plane, DISPC_VID_ATTRIBUTES,
			sp->scale_y, 8, 8);

	/* Skip the rest if no scaling is used */
	if (!sp->scale_x && !sp->scale_y)
		return;

	/* VERTICAL 5-TAPS  */
	VID_REG_FLD_MOD(dispc, hw_plane, DISPC_VID_ATTRIBUTES,
			sp->five_taps, 21, 21);

	if (dispc7_fourcc_is_yuv(fourcc)) {
		if (sp->scale_x) {
			dispc7_vid_write(dispc, hw_plane, DISPC_VID_FIRH2,
					 sp->fir_xinc_uv);
			dispc7_vid_write_fir_coefs(dispc, hw_plane,
						   DISPC7_VID_FIR_COEF_HORIZ_UV,
						   sp->xcoef_uv);
		}
		if (sp->scale_y) {
			dispc7_vid_write(dispc, hw_plane, DISPC_VID_FIRV2,
					 sp->fir_yinc_uv);
			dispc7_vid_write_fir_coefs(dispc, hw_plane,
						   DISPC7_VID_FIR_COEF_VERT_UV,
						   sp->ycoef_uv);
		}
	}

	if (sp->scale_x) {
		dispc7_vid_write(dispc, hw_plane, DISPC_VID_FIRH, sp->fir_xinc);
		dispc7_vid_write_fir_coefs(dispc, hw_plane,
					   DISPC7_VID_FIR_COEF_HORIZ,
					   sp->xcoef);
	}

	if (sp->scale_y) {
		dispc7_vid_write(dispc, hw_plane, DISPC_VID_FIRV, sp->fir_yinc);
		dispc7_vid_write_fir_coefs(dispc, hw_plane,
					   DISPC7_VID_FIR_COEF_VERT, sp->ycoef);
	}
}

static void dispc7_wb_set_scaling(struct dispc_device *dispc,
				  struct dispc7_scaling_params *sp,
				  u32 fourcc)
{
	/* HORIZONTAL RESIZE ENABLE */
	WB_REG_FLD_MOD(dispc, DISPC_WB_ATTRIBUTES, sp->scale_x, 7, 7);

	/* VERTICAL RESIZE ENABLE */
	WB_REG_FLD_MOD(dispc, DISPC_WB_ATTRIBUTES, sp->scale_y, 8, 8);

	/* Skip the rest if no scaling is used */
	if (!sp->scale_x && !sp->scale_y)
		return;

	/* VERTICAL 5-TAPS  */
	WB_REG_FLD_MOD(dispc, DISPC_WB_ATTRIBUTES, sp->five_taps, 21, 21);

	if (dispc7_fourcc_is_yuv(fourcc)) {
		if (sp->scale_x) {
			dispc7_wb_write(dispc, DISPC_WB_FIRH2, sp->fir_xinc_uv);
			dispc7_wb_write_fir_coefs(dispc,
						  DISPC7_WB_FIR_COEF_HORIZ_UV,
						  sp->xcoef_uv);
		}
		if (sp->scale_y) {
			dispc7_wb_write(dispc, DISPC_WB_FIRV2, sp->fir_yinc_uv);
			dispc7_wb_write_fir_coefs(dispc,
						  DISPC7_WB_FIR_COEF_VERT_UV,
						  sp->ycoef_uv);
		}
	}

	if (sp->scale_x) {
		dispc7_wb_write(dispc, DISPC_WB_FIRH, sp->fir_xinc);
		dispc7_wb_write_fir_coefs(dispc, DISPC7_WB_FIR_COEF_HORIZ,
					  sp->xcoef);
	}

	if (sp->scale_y) {
		dispc7_wb_write(dispc, DISPC_WB_FIRV, sp->fir_yinc);
		dispc7_wb_write_fir_coefs(dispc, DISPC7_WB_FIR_COEF_VERT,
					  sp->ycoef);
	}
}

/* OTHER */

static const struct {
	u32 fourcc;
	u8 dss_code;
} dispc7_color_formats[] = {
	{ DRM_FORMAT_ARGB4444, 0x0, },
	{ DRM_FORMAT_ABGR4444, 0x1, },
	{ DRM_FORMAT_RGBA4444, 0x2, },

	{ DRM_FORMAT_RGB565, 0x3, },
	{ DRM_FORMAT_BGR565, 0x4, },

	{ DRM_FORMAT_ARGB1555, 0x5, },
	{ DRM_FORMAT_ABGR1555, 0x6, },

	{ DRM_FORMAT_ARGB8888, 0x7, },
	{ DRM_FORMAT_ABGR8888, 0x8, },
	{ DRM_FORMAT_RGBA8888, 0x9, },
	{ DRM_FORMAT_BGRA8888, 0xa, },

	{ DRM_FORMAT_RGB888, 0xb, },
	{ DRM_FORMAT_BGR888, 0xc, },

	{ DRM_FORMAT_ARGB2101010, 0xe, },
	{ DRM_FORMAT_ABGR2101010, 0xf, },

	{ DRM_FORMAT_XRGB4444, 0x20, },
	{ DRM_FORMAT_XBGR4444, 0x21, },
	{ DRM_FORMAT_RGBX4444, 0x22, },

	{ DRM_FORMAT_ARGB1555, 0x25, },
	{ DRM_FORMAT_ABGR1555, 0x26, },

	{ DRM_FORMAT_XRGB8888, 0x27, },
	{ DRM_FORMAT_XBGR8888, 0x28, },
	{ DRM_FORMAT_RGBX8888, 0x29, },
	{ DRM_FORMAT_BGRX8888, 0x2a, },

	{ DRM_FORMAT_XRGB2101010, 0x2e, },
	{ DRM_FORMAT_XBGR2101010, 0x2f, },

	{ DRM_FORMAT_YUYV, 0x3e, },
	{ DRM_FORMAT_UYVY, 0x3f, },

	{ DRM_FORMAT_NV12, 0x3d, },
};

static void dispc7_plane_set_pixel_format(struct dispc_device *dispc,
					  u32 hw_plane, u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(dispc7_color_formats); ++i) {
		if (dispc7_color_formats[i].fourcc == fourcc) {
			VID_REG_FLD_MOD(dispc, hw_plane, DISPC_VID_ATTRIBUTES,
					dispc7_color_formats[i].dss_code,
					6, 1);
			return;
		}
	}

	WARN_ON(1);
}

static void dispc7_wb_set_pixel_format(struct dispc_device *dispc,
				       u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(dispc7_color_formats); ++i) {
		if (dispc7_color_formats[i].fourcc == fourcc) {
			WB_REG_FLD_MOD(dispc, DISPC_WB_ATTRIBUTES,
				       dispc7_color_formats[i].dss_code, 6, 1);
			return;
		}
	}

	WARN_ON(1);
}

static s32 pixinc(int pixels, u8 ps)
{
	if (pixels == 1)
		return 1;
	else if (pixels > 1)
		return 1 + (pixels - 1) * ps;
	else if (pixels < 0)
		return 1 - (-pixels + 1) * ps;

	WARN_ON(1);
	return 0;
}

static
const struct tidss_plane_feat *dispc7_plane_feat(struct dispc_device *dispc,
						 u32 hw_plane)
{
	static const struct tidss_plane_feat pfeat = {
		.color = {
			.encodings = (BIT(DRM_COLOR_YCBCR_BT601) |
				      BIT(DRM_COLOR_YCBCR_BT709)),
			.ranges = (BIT(DRM_COLOR_YCBCR_FULL_RANGE) |
				   BIT(DRM_COLOR_YCBCR_LIMITED_RANGE)),
			.default_encoding = DRM_COLOR_YCBCR_BT601,
			.default_range = DRM_COLOR_YCBCR_FULL_RANGE,
		},
		.blend = {
			.global_alpha = true,
		},
	};

	return &pfeat;
}

static int dispc7_plane_check(struct dispc_device *dispc, u32 hw_plane,
			      const struct drm_plane_state *state,
			      u32 hw_videoport)
{
	bool lite = dispc->feat->vid_lite[hw_plane];
	u32 fourcc = state->fb->format->format;
	bool need_scaling = state->src_w >> 16 != state->crtc_w ||
		state->src_h >> 16 != state->crtc_h;
	struct dispc7_scaling_params scaling;
	int ret;

	if (dispc7_fourcc_is_yuv(fourcc)) {
		if (!dispc7_find_csc(DISPC7_YUV2RGB, state->color_encoding,
				     state->color_range)) {
			dev_dbg(dispc->dev,
				"%s: Unsupported CSC (%u,%u) for HW plane %u\n",
				__func__, state->color_encoding,
				state->color_range, hw_plane);
			return -EINVAL;
		}
	}

	if (need_scaling) {
		if (lite) {
			dev_dbg(dispc->dev,
				"%s: Lite plane %u can't scale %ux%u!=%ux%u\n",
				__func__, hw_plane,
				state->src_w >> 16, state->src_h >> 16,
				state->crtc_w, state->crtc_h);
			return -EINVAL;
		}
		ret = dispc7_vid_calc_scaling(dispc, state, &scaling, false);
		if (ret)
			return ret;
	}

	return 0;
}

static int dispc7_plane_setup(struct dispc_device *dispc, u32 hw_plane,
			      const struct drm_plane_state *state,
			      u32 hw_videoport)
{
	bool lite = dispc->feat->vid_lite[hw_plane];
	u32 fourcc = state->fb->format->format;
	u16 cpp = state->fb->format->cpp[0];
	u32 fb_width = state->fb->pitches[0] / cpp;
	dma_addr_t paddr = dispc7_plane_state_paddr(state);
	struct dispc7_scaling_params scale;

	dispc7_vid_calc_scaling(dispc, state, &scale, lite);

	dispc7_plane_set_pixel_format(dispc, hw_plane, fourcc);

	dispc7_vid_write(dispc, hw_plane, DISPC_VID_BA_0, paddr & 0xffffffff);
	dispc7_vid_write(dispc, hw_plane, DISPC_VID_BA_EXT_0, (u64)paddr >> 32);
	dispc7_vid_write(dispc, hw_plane, DISPC_VID_BA_1, paddr & 0xffffffff);
	dispc7_vid_write(dispc, hw_plane, DISPC_VID_BA_EXT_1, (u64)paddr >> 32);

	dispc7_vid_write(dispc, hw_plane, DISPC_VID_PICTURE_SIZE,
			 (scale.in_w - 1) | ((scale.in_h - 1) << 16));

	/* For YUV422 format we use the macropixel size for pixel inc */
	if (fourcc == DRM_FORMAT_YUYV || fourcc == DRM_FORMAT_UYVY)
		dispc7_vid_write(dispc, hw_plane, DISPC_VID_PIXEL_INC,
				 pixinc(scale.xinc, cpp * 2));
	else
		dispc7_vid_write(dispc, hw_plane, DISPC_VID_PIXEL_INC,
				 pixinc(scale.xinc, cpp));

	dispc7_vid_write(dispc, hw_plane, DISPC_VID_ROW_INC,
			 pixinc(1 + (scale.yinc * fb_width -
				     scale.xinc * scale.in_w),
				cpp));

	if (state->fb->format->num_planes == 2) {
		u16 cpp_uv = state->fb->format->cpp[1];
		u32 fb_width_uv = state->fb->pitches[1] / cpp_uv;
		dma_addr_t p_uv_addr = dispc7_plane_state_p_uv_addr(state);

		dispc7_vid_write(dispc, hw_plane,
				 DISPC_VID_BA_UV_0, p_uv_addr & 0xffffffff);
		dispc7_vid_write(dispc, hw_plane,
				 DISPC_VID_BA_UV_EXT_0, (u64)p_uv_addr >> 32);
		dispc7_vid_write(dispc, hw_plane,
				 DISPC_VID_BA_UV_1, p_uv_addr & 0xffffffff);
		dispc7_vid_write(dispc, hw_plane,
				 DISPC_VID_BA_UV_EXT_1, (u64)p_uv_addr >> 32);

		dispc7_vid_write(dispc, hw_plane, DISPC_VID_ROW_INC_UV,
				 pixinc(1 + (scale.yinc * fb_width_uv -
					     scale.xinc * scale.in_w_uv),
					cpp_uv));
	}

	if (!lite) {
		dispc7_vid_write(dispc, hw_plane, DISPC_VID_SIZE,
				 (state->crtc_w - 1) |
				 ((state->crtc_h - 1) << 16));

		dispc7_vid_set_scaling(dispc, hw_plane, &scale, fourcc);
	}

	/* enable YUV->RGB color conversion */
	if (dispc7_fourcc_is_yuv(fourcc)) {
		dispc7_vid_csc_setup(dispc, hw_plane, state);
		dispc7_vid_csc_enable(dispc, hw_plane, true);
	} else {
		dispc7_vid_csc_enable(dispc, hw_plane, false);
	}

	dispc7_vid_write(dispc, hw_plane, DISPC_VID_GLOBAL_ALPHA,
			 0xFF & (state->alpha >> 8));

	/* Set pre-multiplied alpha as default. */
	VID_REG_FLD_MOD(dispc, hw_plane, DISPC_VID_ATTRIBUTES, 1, 28, 28);

	dispc7_ovr_set_plane(dispc, hw_plane, hw_videoport,
			     state->crtc_x, state->crtc_y,
			     state->normalized_zpos);

	dispc->plane_data[hw_plane].zorder = state->normalized_zpos;
	dispc->plane_data[hw_plane].hw_videoport = hw_videoport;

	return 0;
}

static int dispc7_plane_enable(struct dispc_device *dispc,
			       u32 hw_plane, bool enable)
{
	dispc7_ovr_enable_plane(dispc, dispc->plane_data[hw_plane].hw_videoport,
				dispc->plane_data[hw_plane].zorder, enable);

	VID_REG_FLD_MOD(dispc, hw_plane, DISPC_VID_ATTRIBUTES, !!enable, 0, 0);

	return 0;
}

static int dispc7_wb_setup(struct dispc_device *dispc,
			   const struct drm_plane_state *state,
			   bool mem_to_mem, u32 src_hw_plane,
			   u32 src_hw_videoport)
{
	u32 fourcc = state->fb->format->format;
	u16 cpp = state->fb->format->cpp[0];
	u32 fb_width = state->fb->pitches[0] / cpp;
	dma_addr_t paddr = dispc7_plane_state_paddr(state);
	struct dispc7_scaling_params scale;
	enum dss7_writeback_connections wb_conn;

	switch (src_hw_videoport) {
	case 0: /* OVR_1 */
		wb_conn = DSS7_WB_OVR1;
		break;
	case 1: /* OVR_2 */
		wb_conn = DSS7_WB_OVR2;
		break;
	case 2: /* OVR_3 */
		wb_conn = DSS7_WB_OVR3;
		break;
	case 3: /* OVR_4 */
		wb_conn = DSS7_WB_OVR4;
		break;
	default:
		return -EINVAL;
	}

	dispc7_wb_calc_scaling(dispc, state, &scale);

	dispc7_wb_set_pixel_format(dispc, fourcc);

	dispc7_wb_write(dispc, DISPC_WB_BA_0, paddr & 0xffffffff);
	dispc7_wb_write(dispc, DISPC_WB_BA_EXT_0, (u64)paddr >> 32);
	dispc7_wb_write(dispc, DISPC_WB_BA_1, paddr & 0xffffffff);
	dispc7_wb_write(dispc, DISPC_WB_BA_EXT_1, (u64)paddr >> 32);

	dispc7_wb_write(dispc, DISPC_WB_SIZE,
			 (scale.in_w - 1) | ((scale.in_h - 1) << 16));

	dispc7_wb_write(dispc, DISPC_WB_ROW_INC,
			 pixinc(1 + (scale.yinc * fb_width -
				     scale.xinc * scale.out_w),
				cpp));

	if (state->fb->format->num_planes == 2) {
		u16 cpp_uv = state->fb->format->cpp[1];
		u32 fb_width_uv = state->fb->pitches[1] / cpp_uv;
		dma_addr_t p_uv_addr = dispc7_plane_state_p_uv_addr(state);

		dispc7_wb_write(dispc, DISPC_WB_BA_UV_0,
				p_uv_addr & 0xffffffff);
		dispc7_wb_write(dispc, DISPC_WB_BA_UV_EXT_0,
				(u64)p_uv_addr >> 32);
		dispc7_wb_write(dispc, DISPC_WB_BA_UV_1,
				p_uv_addr & 0xffffffff);
		dispc7_wb_write(dispc, DISPC_WB_BA_UV_EXT_1,
				(u64)p_uv_addr >> 32);

		dispc7_wb_write(dispc, DISPC_WB_ROW_INC_UV,
				pixinc(1 + (scale.yinc * fb_width_uv -
					    scale.xinc * scale.out_w_uv),
				       cpp_uv));
	}

	dispc7_wb_write(dispc, DISPC_WB_PICTURE_SIZE,
			(state->crtc_w - 1) | ((state->crtc_h - 1) << 16));

	dispc7_wb_set_scaling(dispc, &scale, fourcc);

	/* enable YUV->RGB color conversion */
	if (dispc7_fourcc_is_yuv(fourcc)) {
		dispc7_wb_csc_setup(dispc, state);
		dispc7_wb_csc_enable(dispc, true);
	} else {
		dispc7_wb_csc_enable(dispc, false);
	}

	/* Set writeback mode */
	WB_REG_FLD_MOD(dispc, DISPC_WB_ATTRIBUTES, mem_to_mem, 19, 19);

	CFG_REG_FLD_MOD(dispc, DISPC_CONNECTIONS, wb_conn, 20, 16);

	return 0;
}

static int dispc7_wb_enable(struct dispc_device *dispc, bool enable)
{
	WB_REG_FLD_MOD(dispc, DISPC_WB_ATTRIBUTES, !!enable, 0, 0);

	return 0;
}

static bool dispc7_has_writeback(struct dispc_device *dispc)
{
	return dispc->wb_managed;
}

static u32 dispc7_vid_get_fifo_size(struct dispc_device *dispc, u32 hw_plane)
{
	return VID_REG_GET(dispc, hw_plane, DISPC_VID_BUF_SIZE_STATUS, 15, 0);
}

static void dispc7_vid_set_mflag_threshold(struct dispc_device *dispc,
					   u32 hw_plane, u32 low, u32 high)
{
	dispc7_vid_write(dispc, hw_plane, DISPC_VID_MFLAG_THRESHOLD,
			 FLD_VAL(high, 31, 16) | FLD_VAL(low, 15, 0));
}

static void dispc7_vid_set_buf_threshold(struct dispc_device *dispc,
					 u32 hw_plane, u32 low, u32 high)
{
	dispc7_vid_write(dispc, hw_plane, DISPC_VID_BUF_THRESHOLD,
			 FLD_VAL(high, 31, 16) | FLD_VAL(low, 15, 0));
}

static u32 dispc7_wb_get_fifo_size(struct dispc_device *dispc)
{
	return WB_REG_GET(dispc, DISPC_VID_BUF_SIZE_STATUS, 15, 0);
}

static void dispc7_wb_set_mflag_threshold(struct dispc_device *dispc,
					  u32 low, u32 high)
{
	dispc7_wb_write(dispc, DISPC_WB_MFLAG_THRESHOLD,
			FLD_VAL(high, 31, 16) | FLD_VAL(low, 15, 0));
}

static void dispc7_wb_set_buf_threshold(struct dispc_device *dispc,
					 u32 low, u32 high)
{
	dispc7_wb_write(dispc, DISPC_WB_BUF_THRESHOLD,
			FLD_VAL(high, 31, 16) | FLD_VAL(low, 15, 0));
}

static void dispc7_plane_init(struct dispc_device *dispc)
{
	unsigned int hw_plane;

	dev_dbg(dispc->dev, "%s()\n", __func__);

	if (dispc->has_cfg_common) {
		u32 cba_lo_pri = 1;
		u32 cba_hi_pri = 0;

		CFG_REG_FLD_MOD(dispc, DSS_CBA_CFG, cba_lo_pri, 2, 0);
		CFG_REG_FLD_MOD(dispc, DSS_CBA_CFG, cba_hi_pri, 5, 3);

		/* MFLAG_CTRL = ENABLED */
		CFG_REG_FLD_MOD(dispc, DISPC_GLOBAL_MFLAG_ATTRIBUTE, 2, 1, 0);
		/* MFLAG_START = MFLAGNORMALSTARTMODE */
		CFG_REG_FLD_MOD(dispc, DISPC_GLOBAL_MFLAG_ATTRIBUTE, 0, 6, 6);
	}

	dispc_for_each_managed_plane(dispc, hw_plane) {
		u32 size = dispc7_vid_get_fifo_size(dispc, hw_plane);
		u32 thr_low, thr_high;
		u32 mflag_low, mflag_high;
		u32 preload;

		thr_high = size - 1;
		thr_low = size / 2;

		mflag_high = size * 2 / 3;
		mflag_low = size / 3;

		preload = thr_low;

		dev_dbg(dispc->dev,
			"%s: bufsize %u, buf_threshold %u/%u, mflag threshold %u/%u preload %u\n",
			dispc7_plane_name(dispc, hw_plane),
			size,
			thr_high, thr_low,
			mflag_high, mflag_low,
			preload);

		dispc7_vid_set_buf_threshold(dispc, hw_plane,
					     thr_low, thr_high);
		dispc7_vid_set_mflag_threshold(dispc, hw_plane,
					       mflag_low, mflag_high);

		dispc7_vid_write(dispc, hw_plane, DISPC_VID_PRELOAD, preload);

		/*
		 * Prefech up to fifo high-threshold value to minimize the
		 * possibility of underflows. Note that this means the PRELOAD
		 * register is ignored.
		 */
		VID_REG_FLD_MOD(dispc, hw_plane, DISPC_VID_ATTRIBUTES, 1, 19, 19);
	}

	if (dispc7_has_writeback(dispc)) {
		u32 size = dispc7_wb_get_fifo_size(dispc);
		u32 thr_low, thr_high;
		u32 mflag_low, mflag_high;

		thr_high = size - 1;
		thr_low = size / 2;

		mflag_high = size * 2 / 3;
		mflag_low = size / 3;

		dev_dbg(dispc->dev,
			"wb: bufsize %u, buf_threshold %u/%u, mflag threshold %u/%u\n",
			size,
			thr_high, thr_low,
			mflag_high, mflag_low);

		dispc7_wb_set_buf_threshold(dispc, thr_low, thr_high);
		dispc7_wb_set_mflag_threshold(dispc, mflag_low, mflag_high);
	}
}

static void dispc7_vp_init(struct dispc_device *dispc)
{
	unsigned int i;

	dev_dbg(dispc->dev, "%s()\n", __func__);

	/* Enable the gamma Shadow bit-field for all VPs*/
	dispc_for_each_managed_vp(dispc, i)
		VP_REG_FLD_MOD(dispc, i, DISPC_VP_CONFIG, 1, 2, 2);
}

static void dispc7_initial_config(struct dispc_device *dispc)
{
	dispc7_plane_init(dispc);
	dispc7_vp_init(dispc);

	/* Note: Harcdoded DPI routing on J721E for now */
	if (dispc->feat->subrev == DSS7_J721E && dispc->has_cfg_common) {
		dispc7_cfg_write(dispc, DISPC_CONNECTIONS,
			     FLD_VAL(2, 3, 0) |		/* VP1 to DPI0 */
			     FLD_VAL(8, 7, 4)		/* VP3 to DPI1 */
			    );
	}
}

static int dispc7_get_num_planes(struct dispc_device *dispc)
{
	return dispc->feat->num_planes;
}

static int dispc7_get_num_vps(struct dispc_device *dispc)
{
	return dispc->feat->num_vps;
}

static const struct tidss_vp_feat *dispc7_vp_feat(struct dispc_device *dispc,
						  u32 hw_videoport)
{
	return &dispc->feat->vp_feat;
}

static void dispc7_vp_write_gamma_table(struct dispc_device *dispc,
					u32 hw_videoport)
{
	u32 *table = dispc->vp_data[hw_videoport].gamma_table;
	u32 hwlen = dispc->feat->vp_feat.color.gamma_size;
	unsigned int i;

	dev_dbg(dispc->dev, "%s: hw_videoport %d\n", __func__, hw_videoport);

	for (i = 0; i < hwlen; ++i) {
		u32 v = table[i];

		switch (dispc->feat->vp_feat.color.gamma_type) {
		case TIDSS_GAMMA_8BIT:
			v |= i << 24;
			break;
		case TIDSS_GAMMA_10BIT:
			if (i == 0)
				v |= 1 << 31;
			break;
		default:
			WARN_ON(1);
			return;
		}

		dispc7_vp_write(dispc, hw_videoport, DISPC_VP_GAMMA_TABLE, v);
	}
}

static void dispc7_restore_gamma_tables(struct dispc_device *dispc)
{
	unsigned int i;

	dev_dbg(dispc->dev, "%s()\n", __func__);

	dispc_for_each_managed_vp(dispc, i)
		dispc7_vp_write_gamma_table(dispc, i);
}

static const struct drm_color_lut dispc7_vp_gamma_default_lut[] = {
	{ .red = 0, .green = 0, .blue = 0, },
	{ .red = U16_MAX, .green = U16_MAX, .blue = U16_MAX, },
};

static void dispc7_vp_set_gamma(struct dispc_device *dispc,
				u32 hw_videoport,
				const struct drm_color_lut *lut,
				unsigned int length)
{
	u32 *table = dispc->vp_data[hw_videoport].gamma_table;
	u32 hwlen = dispc->feat->vp_feat.color.gamma_size;
	u32 hwbits;
	unsigned int i;

	dev_dbg(dispc->dev, "%s: hw_videoport %d, lut len %u, hw len %u\n",
		__func__, hw_videoport, length, hwlen);

	if (dispc->feat->vp_feat.color.gamma_type == TIDSS_GAMMA_10BIT)
		hwbits = 10;
	else
		hwbits = 8;

	if (lut == NULL || length < 2) {
		lut = dispc7_vp_gamma_default_lut;
		length = ARRAY_SIZE(dispc7_vp_gamma_default_lut);
	}

	for (i = 0; i < length - 1; ++i) {
		unsigned int first = i * (hwlen - 1) / (length - 1);
		unsigned int last = (i + 1) * (hwlen - 1) / (length - 1);
		unsigned int w = last - first;
		u16 r, g, b;
		unsigned int j;

		if (w == 0)
			continue;

		for (j = 0; j <= w; j++) {
			r = (lut[i].red * (w - j) + lut[i + 1].red * j) / w;
			g = (lut[i].green * (w - j) + lut[i + 1].green * j) / w;
			b = (lut[i].blue * (w - j) + lut[i + 1].blue * j) / w;

			r >>= 16 - hwbits;
			g >>= 16 - hwbits;
			b >>= 16 - hwbits;

			table[first + j] = (r << (hwbits * 2)) |
					   (g << hwbits) | b;
		}
	}

	if (dispc->is_enabled)
		dispc7_vp_write_gamma_table(dispc, hw_videoport);
}

static s16 dispc7_S31_32_to_s2_8(s64 coef)
{
	u64 sign_bit = 1ULL << 63;
	u64 cbits = (u64) coef;
	s16 ret = clamp_val(((cbits & ~sign_bit) >> 24), 0, 0x1FF);

	if (cbits & sign_bit)
		ret = -ret;

	return ret;
}

static void dispc7_cpr_csc_from_ctm(const struct drm_color_ctm *ctm,
				    struct dispc7_csc_coef *cpr)
{
	memset(cpr, 0, sizeof(*cpr));

	cpr->to_regval = dispc7_csc_cpr_regval;
	cpr->m[CSC_RR] = dispc7_S31_32_to_s2_8(ctm->matrix[0]);
	cpr->m[CSC_RG] = dispc7_S31_32_to_s2_8(ctm->matrix[1]);
	cpr->m[CSC_RB] = dispc7_S31_32_to_s2_8(ctm->matrix[2]);
	cpr->m[CSC_GR] = dispc7_S31_32_to_s2_8(ctm->matrix[3]);
	cpr->m[CSC_GG] = dispc7_S31_32_to_s2_8(ctm->matrix[4]);
	cpr->m[CSC_GB] = dispc7_S31_32_to_s2_8(ctm->matrix[5]);
	cpr->m[CSC_BR] = dispc7_S31_32_to_s2_8(ctm->matrix[6]);
	cpr->m[CSC_BG] = dispc7_S31_32_to_s2_8(ctm->matrix[7]);
	cpr->m[CSC_BB] = dispc7_S31_32_to_s2_8(ctm->matrix[8]);
}

static void dispc7_vp_write_csc(struct dispc_device *dispc, u32 hw_videoport,
				const struct dispc7_csc_coef *csc)
{
	static const u16 dispc_vp_csc_coef_reg[DISPC7_CSC_REGVAL_LEN] = {
		DISPC_VP_CSC_COEF0, DISPC_VP_CSC_COEF1, DISPC_VP_CSC_COEF2,
		DISPC_VP_CSC_COEF3, DISPC_VP_CSC_COEF4, DISPC_VP_CSC_COEF5,
		DISPC_VP_CSC_COEF6, DISPC_VP_CSC_COEF7,
	};
	u32 regval[DISPC7_CSC_REGVAL_LEN];
	unsigned int i;

	csc->to_regval(csc, regval);

	for (i = 0; i < ARRAY_SIZE(regval); i++)
		dispc7_vp_write(dispc, hw_videoport, dispc_vp_csc_coef_reg[i],
				regval[i]);
}

static void dispc7_vp_set_color_mgmt(struct dispc_device *dispc,
				     u32 hw_videoport,
				     const struct drm_crtc_state *state)
{
	struct drm_color_lut *lut = NULL;
	unsigned int length = 0;
	bool colorconvenable = false;

	if (!state->color_mgmt_changed)
		return;

	if (state->gamma_lut) {
		lut = (struct drm_color_lut *) state->gamma_lut->data;
		length = state->gamma_lut->length / sizeof(*lut);
	}

	if (state->ctm) {
		struct drm_color_ctm *ctm =
			(struct drm_color_ctm *) state->ctm->data;
		struct dispc7_csc_coef cpr;

		dispc7_cpr_csc_from_ctm(ctm, &cpr);
		dispc7_vp_write_csc(dispc, hw_videoport, &cpr);

		colorconvenable = true;
	}

	dispc7_vp_set_gamma(dispc, hw_videoport, lut, length);
	VP_REG_FLD_MOD(dispc, hw_videoport, DISPC_VP_CONFIG,
		       colorconvenable, 24, 24);
}

static void dispc7_vp_setup(struct dispc_device *dispc, u32 hw_videoport,
			    const struct drm_crtc_state *state)
{
	dispc7_vp_set_default_color(dispc, hw_videoport, 0);
	dispc7_vp_set_color_mgmt(dispc, hw_videoport, state);
}

static int dispc7_init_gamma_tables(struct dispc_device *dispc)
{
	unsigned int i;

	dev_dbg(dispc->dev, "%s()\n", __func__);

	dispc_for_each_managed_vp(dispc, i)
		dispc7_vp_set_gamma(dispc, i, NULL, 0);

	return 0;
}

static const char *dispc7_plane_name(struct dispc_device *dispc, u32 hw_plane)
{
	if (WARN_ON(hw_plane >= dispc->feat->num_planes))
		return "ERROR";

	return dispc->feat->vid_name[hw_plane];
}

static const char *dispc7_vp_name(struct dispc_device *dispc, u32 hw_videoport)
{
	if (WARN_ON(hw_videoport >= dispc->feat->num_vps))
		return "ERROR";

	return dispc->feat->vp_name[hw_videoport];
}

static int dispc7_runtime_suspend(struct dispc_device *dispc)
{
	dev_dbg(dispc->dev, "suspend\n");

	dispc->is_enabled = false;

	clk_disable_unprepare(dispc->fclk);

	return 0;
}

static int dispc7_runtime_resume(struct dispc_device *dispc)
{
	dev_dbg(dispc->dev, "resume\n");

	clk_prepare_enable(dispc->fclk);

	if (!dispc->has_cfg_common)
		goto no_cfg;

	if (CFG_REG_GET(dispc, DSS_SYSSTATUS, 0, 0) == 0)
		dev_warn(dispc->dev, "DSS FUNC RESET not done!\n");

	dev_dbg(dispc->dev, "OMAP DSS7 rev 0x%x\n",
		dispc7_cfg_read(dispc, DSS_REVISION));

	dev_dbg(dispc->dev, "VP RESETDONE %d,%d,%d\n",
		CFG_REG_GET(dispc, DSS_SYSSTATUS, 1, 1),
		CFG_REG_GET(dispc, DSS_SYSSTATUS, 2, 2),
		CFG_REG_GET(dispc, DSS_SYSSTATUS, 3, 3));

	dev_dbg(dispc->dev, "OLDI RESETDONE %d,%d,%d\n",
		CFG_REG_GET(dispc, DSS_SYSSTATUS, 5, 5),
		CFG_REG_GET(dispc, DSS_SYSSTATUS, 6, 6),
		CFG_REG_GET(dispc, DSS_SYSSTATUS, 7, 7));

	dev_dbg(dispc->dev, "DISPC IDLE %d\n",
		CFG_REG_GET(dispc, DSS_SYSSTATUS, 9, 9));

no_cfg:
	dispc7_initial_config(dispc);

	dispc7_restore_gamma_tables(dispc);

	dispc->is_enabled = true;

	return 0;
}

static void dispc7_wb_find_free_ovr(struct dispc_device *dispc)
{
	struct tidss_device *tidss = dispc->tidss;
	int i, j;
	bool found;
	u32 ovr_id = 0xff;

	dispc_for_each_managed_vp(dispc, i) {
		found = false;
		for (j = 0; j < tidss->num_crtcs; j++) {
			struct drm_crtc *crtc = tidss->crtcs[j];
			struct tidss_crtc *tcrtc = to_tidss_crtc(crtc);

			if (tcrtc->hw_videoport == i) {
				found = true;
				break;
			}
		}
		if (!found) {
			/* this means this ovr is available */
			ovr_id = i;
			break;
		}
	}

	if (ovr_id != 0xff)
		dispc->wb_reserved_ovr = ovr_id;
	else
		dispc->wb_managed = false;
}

static u32 dispc7_wb_get_reserved_ovr(struct dispc_device *dispc)
{
	return dispc->wb_reserved_ovr;
}

static int dispc7_modeset_init(struct dispc_device *dispc)
{
	struct tidss_device *tidss = dispc->tidss;
	struct device *dev = tidss->dev;
	u32 fourccs[ARRAY_SIZE(dispc7_color_formats)];
	unsigned int i, num_fourccs;

	struct pipe {
		u32 hw_videoport;
		struct drm_bridge *bridge;
		u32 enc_type;
	};

	struct pipe pipes[DISPC7_MAX_PORTS];
	u32 num_pipes = 0;
	u32 plane_idx = 0;
	u32 crtc_mask;

	num_fourccs = 0;
	for (i = 0; i < ARRAY_SIZE(fourccs); ++i) {
		if (dispc->feat->errata.i2000 &&
		    dispc7_fourcc_is_yuv(dispc7_color_formats[i].fourcc))
			continue;

		fourccs[num_fourccs++] = dispc7_color_formats[i].fourcc;
	}

	/* first find all the connected panels & bridges */
	/* exclude the VPs that are not managed.         */

	dispc_for_each_managed_vp(dispc, i) {
		struct drm_panel *panel;
		struct drm_bridge *bridge;
		u32 enc_type = DRM_MODE_ENCODER_NONE;
		int ret;

		ret = drm_of_find_panel_or_bridge(dev->of_node, i, 0,
						  &panel, &bridge);
		if (ret == -ENODEV) {
			dev_dbg(dev, "no panel/bridge for port %d\n", i);
			continue;
		} else if (ret) {
			dev_dbg(dev, "port %d probe returned %d\n", i, ret);
			return ret;
		}

		if (panel) {
			u32 conn_type;

			dev_dbg(dev, "Setting up panel for port %d\n", i);

			switch (dispc->feat->vp_bus_type[i]) {
			case DISPC7_VP_OLDI:
				enc_type = DRM_MODE_ENCODER_LVDS;
				conn_type = DRM_MODE_CONNECTOR_LVDS;
				break;
			case DISPC7_VP_DPI:
				enc_type = DRM_MODE_ENCODER_DPI;
				conn_type = DRM_MODE_CONNECTOR_DPI;
				break;
			default:
				dev_warn(dev, "%s: Bad vp bus type: %d\n",
					 __func__, dispc->feat->vp_bus_type[i]);
				conn_type = DRM_MODE_CONNECTOR_Unknown;
				break;
			}

			bridge = devm_drm_panel_bridge_add(dev, panel,
							   conn_type);
			if (IS_ERR(bridge)) {
				dev_err(dev, "failed to set up panel bridge for port %d\n", i);
				return PTR_ERR(bridge);
			}
		}

		pipes[num_pipes].hw_videoport = i;
		pipes[num_pipes].bridge = bridge;
		pipes[num_pipes].enc_type = enc_type;
		num_pipes++;
	}

	/* all planes can be on any crtc */
	crtc_mask = (1 << num_pipes) - 1;

	/* then create a plane, a crtc and an encoder for each panel/bridge */

	for (i = 0; i < num_pipes; ++i) {
		struct tidss_plane *tplane;
		struct tidss_crtc *tcrtc;
		struct drm_encoder *enc;
		u32 hw_plane_id;
		int ret;

		hw_plane_id = dispc7_get_next_managed_plane(dispc, &plane_idx);
		if (hw_plane_id == dispc->feat->num_planes) {
			dev_err(tidss->dev, "no managed HW plane found for CRTC\n");
			return -EINVAL;
		}

		tplane = tidss_plane_create(tidss, hw_plane_id,
					    DRM_PLANE_TYPE_PRIMARY, crtc_mask,
					    fourccs, num_fourccs);
		if (IS_ERR(tplane)) {
			dev_err(tidss->dev, "plane create failed\n");
			return PTR_ERR(tplane);
		}

		hw_plane_id++;

		tidss->planes[tidss->num_planes++] = &tplane->plane;

		tcrtc = tidss_crtc_create(tidss, pipes[i].hw_videoport,
					  &tplane->plane);
		if (IS_ERR(tcrtc)) {
			dev_err(tidss->dev, "crtc create failed\n");
			return PTR_ERR(tcrtc);
		}

		tidss->crtcs[tidss->num_crtcs++] = &tcrtc->crtc;

		enc = tidss_encoder_create(tidss, pipes[i].enc_type,
					   1 << tcrtc->crtc.index);
		if (IS_ERR(enc)) {
			dev_err(tidss->dev, "encoder create failed\n");
			return PTR_ERR(enc);
		}

		ret = drm_bridge_attach(enc, pipes[i].bridge, NULL);
		if (ret) {
			dev_err(tidss->dev, "bridge attach failed: %d\n", ret);
			return ret;
		}
	}

	/* Try to find an available OVR to use for WB */
	if (dispc7_has_writeback(dispc))
		dispc7_wb_find_free_ovr(dispc);

	/* create overlay planes of the leftover planes */

	while (tidss->num_planes < dispc->feat->num_planes) {
		struct tidss_plane *tplane;
		u32 hw_plane_id;

		hw_plane_id = dispc7_get_next_managed_plane(dispc, &plane_idx);
		if (hw_plane_id == dispc->feat->num_planes)
			break;

		tplane = tidss_plane_create(tidss, hw_plane_id,
					    DRM_PLANE_TYPE_OVERLAY, crtc_mask,
					    fourccs, num_fourccs);

		if (IS_ERR(tplane)) {
			dev_err(tidss->dev, "plane create failed\n");
			return PTR_ERR(tplane);
		}

		hw_plane_id++;

		tidss->planes[tidss->num_planes++] = &tplane->plane;
	}

	return 0;
}

static int dispc7_get_irq(struct dispc_device *dispc)
{
	return dispc->irq;
}

static void dispc7_remove(struct dispc_device *dispc)
{
	struct device *dev = dispc->dev;

	dev_dbg(dev, "%s\n", __func__);

	dispc->tidss->dispc_ops = NULL;
	dispc->tidss->dispc = NULL;
}

static const struct tidss_dispc_ops dispc7_ops = {
	.read_and_clear_irqstatus = dispc7_read_and_clear_irqstatus,
	.write_irqenable = dispc7_write_irqenable,

	.get_num_vps = dispc7_get_num_vps,
	.vp_name = dispc7_vp_name,
	.vp_feat = dispc7_vp_feat,
	.vp_mode_valid = dispc7_vp_mode_valid,
	.vp_check = dispc7_vp_check,
	.vp_setup = dispc7_vp_setup,
	.vp_prepare = dispc7_vp_prepare,
	.vp_enable = dispc7_vp_enable,
	.vp_disable = dispc7_vp_disable,
	.vp_unprepare = dispc7_vp_unprepare,
	.vp_go_busy = dispc7_vp_go_busy,
	.vp_go = dispc7_vp_go,

	.vp_set_clk_rate = dispc7_vp_set_clk_rate,
	.vp_enable_clk = dispc7_vp_enable_clk,
	.vp_disable_clk = dispc7_vp_disable_clk,

	.get_num_planes = dispc7_get_num_planes,
	.plane_name = dispc7_plane_name,
	.plane_feat = dispc7_plane_feat,
	.plane_enable = dispc7_plane_enable,
	.plane_check = dispc7_plane_check,
	.plane_setup = dispc7_plane_setup,

	.runtime_get = dispc7_runtime_get,
	.runtime_put = dispc7_runtime_put,

	.runtime_suspend = dispc7_runtime_suspend,
	.runtime_resume = dispc7_runtime_resume,

	.remove = dispc7_remove,

	.modeset_init = dispc7_modeset_init,

	.get_irq = dispc7_get_irq,

	.has_writeback = dispc7_has_writeback,
	.wb_setup = dispc7_wb_setup,
	.wb_enable = dispc7_wb_enable,
	.wb_get_reserved_ovr = dispc7_wb_get_reserved_ovr,
};

static int dispc7_iomap_resource(struct platform_device *pdev, const char *name,
				 void __iomem **base)
{
	struct resource *res;
	void __iomem *b;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (!res) {
		dev_err(&pdev->dev, "cannot get mem resource '%s'\n", name);
		return -EINVAL;
	}

	b = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(b)) {
		dev_err(&pdev->dev, "cannot ioremap resource '%s'\n", name);
		return PTR_ERR(b);
	}

	*base = b;

	return 0;
}

static int dispc_j721e_get_managed_common_intr(struct dispc_device *dispc,
		u32 *intr)
{
	int ret;
	struct tidss_device *tidss = dispc->tidss;
	struct device *dev = tidss->dev;
	struct device_node *dss_commons_node;
	struct device_node *intr_node;
	u32 value;

	dss_commons_node = of_get_child_by_name(dev->of_node, "dss-commons");
	if (!dss_commons_node) {
		*intr = 0;
		return 0;
	}

	intr_node = of_get_child_by_name(dss_commons_node, "interrupt-common");
	if (!intr_node) {
		ret = -ENOENT;
		goto out;
	}

	ret = of_property_read_u32(intr_node, "reg", &value);
	if (ret)
		goto out2;

	*intr = value;

out2:
	of_node_put(intr_node);
out:
	of_node_put(dss_commons_node);
	return ret;
}

static int dispc_j721e_get_managed_common_cfg(struct dispc_device *dispc,
		u32 *cfg)
{
	int ret;
	struct tidss_device *tidss = dispc->tidss;
	struct device *dev = tidss->dev;
	struct device_node *dss_commons_node;
	struct device_node *cfg_node;
	u32 value;

	dss_commons_node = of_get_child_by_name(dev->of_node, "dss-commons");
	if (!dss_commons_node) {
		*cfg = 0;
		return 0;
	}

	cfg_node = of_get_child_by_name(dss_commons_node, "config-common");
	if (!cfg_node) {
		ret = -ENOENT;
		goto out;
	}

	if (!of_device_is_available(cfg_node)) {
		ret = -ENOENT;
		goto out;
	}

	ret = of_property_read_u32(cfg_node, "reg", &value);
	if (ret)
		goto out2;

	*cfg = value;

out2:
	of_node_put(cfg_node);
out:
	of_node_put(dss_commons_node);
	return ret;
}

/*
 * The logic for J721E is simple:
 * 1. Must find an interrupt common, the driver cannot work
 *    without one.
 *
 *    If the dss device-tree node does not have a subnode
 *    "dss-commons", assume tidss is the only module handling
 *    DSS and therefore use intr_common = COMMON_M.
 *
 *    If "dss-commons" subnode is present, then it must have
 *    a child node "interrupt-common", or else fail. And then,
 *    interrupt-common must have a "reg" property that
 *    indicates which common area to use for interrupts. Must
 *    be 0 to feat->num_commons
 *
 * 2. Optionally, find a configuration region, or make certain
 *    assumptions and proceed
 *
 *    If the dss has a remote device, assume tidss is a slave
 *    and do not search for a config_common
 *
 *    If the dss device-tree node does not have a
 *    subnode called "dss-commons", assume tidss is the only
 *    module handling DSS and therefore use config_common =
 *    COMMON_M.
 *
 *    If "dss-commons" is present, search for a child
 *    "config-common". If no such child is present, tidss
 *    assumes that DSS is early-configured and does not
 *    search for a config-common.
 *
 *    If "config-common" child is present, then it must have a
 *    "reg" property that indicates which common area to use for
 *    configuration. Must be COMMON_M
 */
static int dispc7_j721e_setup_commons(struct dispc_device *dispc)
{
	int r;
	struct tidss_device *tidss = dispc->tidss;
	struct device *dev = tidss->dev;
	struct platform_device *pdev = to_platform_device(dev);
	u32 common_intr_id, common_cfg_id;

	r = dispc_j721e_get_managed_common_intr(dispc, &common_intr_id);
	if (r || common_intr_id >= dispc->feat->num_commons)
		return -EINVAL;

	r = dispc7_iomap_resource(pdev, dispc->feat->common_name[common_intr_id],
			&dispc->base_common_intr);
	if (r)
		return r;

	dispc->irq = platform_get_irq(pdev, common_intr_id);
	if (dispc->irq < 0)
		return dispc->irq;

	if (tidss->rdev) {
		dev_dbg(dev, "%s: continuing with remote device\n", __func__);
		dispc->has_cfg_common = false;
		return 0;
	}

	r = dispc_j721e_get_managed_common_cfg(dispc, &common_cfg_id);
	if (r) {
		dev_dbg(dev, "%s: continuing without configuration common\n", __func__);
		dispc->has_cfg_common = false;
		return 0;
	}

	if (common_cfg_id >= dispc->feat->num_commons ||
			!dispc->feat->common_cfg[common_cfg_id])
		return -EINVAL;

	if (common_intr_id == common_cfg_id)
		dispc->base_common_cfg = dispc->base_common_intr;
	else {
		r = dispc7_iomap_resource(pdev, dispc->feat->common_name[common_cfg_id],
				&dispc->base_common_cfg);
		if (r)
			return r;
	}

	dispc->has_cfg_common = true;

	return 0;
}

static int dispc7_am6_setup_commons(struct dispc_device *dispc)
{
	int r;
	struct tidss_device *tidss = dispc->tidss;
	struct platform_device *pdev = to_platform_device(tidss->dev);

	r = dispc7_iomap_resource(pdev, "common", &dispc->base_common_cfg);
	if (r)
		return r;

	dispc->base_common_intr = dispc->base_common_cfg;

	dispc->irq = platform_get_irq(pdev, 0);
	if (dispc->irq < 0)
		return dispc->irq;

	dispc->has_cfg_common = true;

	return 0;
}

static int dispc7_setup_commons(struct dispc_device *dispc)
{
	switch (dispc->feat->subrev) {
	case DSS7_AM6:
		return dispc7_am6_setup_commons(dispc);
	case DSS7_J721E:
		return dispc7_j721e_setup_commons(dispc);
	default:
		WARN_ON(1);
		return -EINVAL;
	}
}

static struct device_node *dispc7_of_dss_plane_for_id(struct device_node *parent, u32 id)
{
	struct device_node *dss_planes_node, *plane;

	dss_planes_node = of_get_child_by_name(parent, "dss-planes");
	if (!dss_planes_node)
		return NULL;

	for_each_child_of_node(dss_planes_node, plane) {
		u32 plane_id = 0;

		if (of_node_cmp(plane->name, "plane") != 0)
			continue;
		of_property_read_u32(plane, "reg", &plane_id);
		if (id == plane_id)
			break;
	}

	of_node_put(dss_planes_node);

	return plane;
}

static struct device_node *dispc7_of_dss_vp_for_id(struct device_node *parent, u32 id)
{
	struct device_node *dss_vps_node, *vp;

	dss_vps_node = of_get_child_by_name(parent, "dss-vps");
	if (!dss_vps_node)
		return NULL;

	for_each_child_of_node(dss_vps_node, vp) {
		u32 vp_id = 0;

		if (of_node_cmp(vp->name, "vp") != 0)
			continue;
		of_property_read_u32(vp, "reg", &vp_id);
		if (id == vp_id)
			break;
	}

	of_node_put(dss_vps_node);

	return vp;
}

static struct device_node *dispc7_of_dss_wb(struct device_node *parent)
{
	struct device_node *dss_wbs_node, *wb;

	dss_wbs_node = of_get_child_by_name(parent, "dss-wbs");
	if (!dss_wbs_node)
		return NULL;

	for_each_child_of_node(dss_wbs_node, wb) {
		if (of_node_cmp(wb->name, "wb") == 0)
			break;
	}

	of_node_put(dss_wbs_node);

	return wb;
}

static bool dispc7_is_plane_managed(struct tidss_device *tidss, u32 plane_id)
{
	struct device *dev = tidss->dev;
	struct device_node *plane;
	u32 managed;
	bool ret;

	plane = dispc7_of_dss_plane_for_id(dev->of_node, plane_id);
	if (!plane)
		return true;

	ret = true;

	if (of_property_read_u32(plane, "managed", &managed))
		goto out;

	if (!managed)
		ret = false;

out:
	of_node_put(plane);
	return ret;
}

static bool dispc7_is_vp_managed(struct tidss_device *tidss, u32 vp_id)
{
	struct device *dev = tidss->dev;
	struct device_node *vp;
	u32 managed;
	bool ret;

	vp = dispc7_of_dss_vp_for_id(dev->of_node, vp_id);
	if (!vp)
		return true;

	ret = true;

	if (of_property_read_u32(vp, "managed", &managed))
		goto out;

	if (!managed)
		ret = false;

out:
	of_node_put(vp);
	return ret;
}

static bool dispc7_is_wb_managed(struct tidss_device *tidss)
{
	struct device *dev = tidss->dev;
	struct device_node *wb;
	u32 managed;
	bool ret;

	wb = dispc7_of_dss_wb(dev->of_node);
	if (!wb)
		return true;

	ret = true;

	if (of_property_read_u32(wb, "managed", &managed))
		goto out;

	if (!managed)
		ret = false;

out:
	of_node_put(wb);
	return ret;
}

int dispc7_init(struct tidss_device *tidss)
{
	struct device *dev = tidss->dev;
	struct platform_device *pdev = to_platform_device(dev);
	struct dispc_device *dispc;
	const struct dispc7_features *feat;
	unsigned int i;
	int r = 0;

	dev_dbg(dev, "%s\n", __func__);

	r = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(48));
	if (r)
		dev_warn(dev, "cannot set DMA masks to 48-bit\n");

	feat = of_match_device(dispc7_of_table, dev)->data;

	dispc = devm_kzalloc(dev, sizeof(*dispc), GFP_KERNEL);
	if (!dispc)
		return -ENOMEM;

	dispc->tidss = tidss;
	dispc->dev = dev;
	dispc->feat = feat;

	switch (feat->subrev) {
	case DSS7_AM6:
		dispc7_common_regmap = tidss_am6_common_regs;
		break;
	case DSS7_J721E:
		dispc7_common_regmap = tidss_j721e_common_regs;
		break;
	default:
		WARN_ON(1);
		return -EINVAL;
	}

	r = dispc7_setup_commons(dispc);
	if (r) {
		dev_err(dev, "%s: could not setup common regions\n", __func__);
		return r;
	}

	for (i = 0; i < dispc->feat->num_vps; i++)
		dispc->vp_managed[i] = dispc7_is_vp_managed(tidss, i);

	for (i = 0; i < dispc->feat->num_planes; i++)
		dispc->plane_managed[i] = dispc7_is_plane_managed(tidss, i);

	if (dispc->feat->has_writeback)
		dispc->wb_managed = dispc7_is_wb_managed(tidss);

	dispc_for_each_managed_plane(dispc, i) {
		r = dispc7_iomap_resource(pdev, dispc->feat->vid_name[i],
					  &dispc->base_vid[i]);
		dev_dbg(dev, "%s: %u %s %d\n", __func__,
			i, dispc->feat->vid_name[i], r);
		if (r)
			return r;
	}

	if (dispc7_has_writeback(dispc)) {
		r = dispc7_iomap_resource(pdev, dispc->feat->wb_name,
					  &dispc->base_wb);
		dev_dbg(dev, "%s: %s %d\n", __func__,
			dispc->feat->wb_name, r);
		if (r)
			return r;
	}

	dispc_for_each_managed_vp(dispc, i) {
		struct clk *clk;

		r = dispc7_iomap_resource(pdev, dispc->feat->ovr_name[i],
					  &dispc->base_ovr[i]);
		dev_dbg(dev, "%s: %u %s %d\n", __func__,
			i, dispc->feat->ovr_name[i], r);
		if (r)
			return r;

		r = dispc7_iomap_resource(pdev, dispc->feat->vp_name[i],
					  &dispc->base_vp[i]);
		dev_dbg(dev, "%s: %u %s %d\n", __func__,
			i, dispc->feat->vp_name[i], r);
		if (r)
			return r;

		clk = devm_clk_get(dev, dispc->feat->vpclk_name[i]);
		if (IS_ERR(clk)) {
			dev_err(dev, "%s: Failed to get clk %s:%ld\n", __func__,
				dispc->feat->vpclk_name[i], PTR_ERR(clk));
			return PTR_ERR(clk);
		}
		dispc->vp_clk[i] = clk;

		dispc->vp_data[i].gamma_table = devm_kmalloc_array(
			dev, dispc->feat->vp_feat.color.gamma_size,
			sizeof(*dispc->vp_data[i].gamma_table), GFP_KERNEL);
		if (!dispc->vp_data[i].gamma_table)
			return -ENOMEM;
	}

	if (feat->subrev == DSS7_AM6) {
		dispc->syscon = syscon_regmap_lookup_by_phandle(dev->of_node,
								"syscon");
		if (IS_ERR(dispc->syscon)) {
			dev_err(dev, "%s: syscon_regmap_lookup_by_phandle failed %ld\n",
				__func__, PTR_ERR(dispc->syscon));
			return PTR_ERR(dispc->syscon);
		}
	}

	dispc->fclk = devm_clk_get(dev, "fck");
	if (IS_ERR(dispc->fclk)) {
		dev_err(dev, "Failed to get fclk\n");
		return PTR_ERR(dispc->fclk);
	}
	dev_dbg(dev, "DSS fclk %lu Hz\n", clk_get_rate(dispc->fclk));

	r = dispc7_init_gamma_tables(dispc);
	if (r)
		return r;

	tidss->dispc_ops = &dispc7_ops;
	tidss->dispc = dispc;

	return 0;
}
