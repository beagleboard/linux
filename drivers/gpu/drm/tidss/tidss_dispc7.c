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
#include "tidss_dispc7.h"

static const struct dispc7_features dispc7_am6_feats = {
	.min_pclk = 1000,
	.max_pclk = 200000000,

	.num_vps = 2,
	.vp_name = { "vp1", "vp2" },
	.ovr_name = { "ovr1", "ovr2" },
	.vpclk_name =  { "vp1", "vp2" },
	.vp_enc_type =	{ DRM_MODE_ENCODER_LVDS, DRM_MODE_ENCODER_DPI },

	.num_planes = 2,
	/* note: vid is plane_id 0 and vidl1 is plane_id 1 */
	.vid_name = { "vid", "vidl1" },
	.vid_lite = { false, true, },
};

static const struct of_device_id dispc7_of_table[] = {
	{ .compatible = "ti,am6-dss", .data = &dispc7_am6_feats },
	{ }
};

enum dispc7_oldi_mode { SPWG_18 = 0, JEIDA_24 = 1, SPWG_24 = 2 };

struct dispc7_bus_format {
	u32 bus_fmt;
	u32 data_width;
	bool oldi;
	enum dispc7_oldi_mode oldi_mode;
};

static const struct dispc7_bus_format dispc7_bus_formats[] = {
	{ MEDIA_BUS_FMT_RGB444_1X12,		12, false, 0 },
	{ MEDIA_BUS_FMT_RGB565_1X16,		16, false, 0 },
	{ MEDIA_BUS_FMT_RGB666_1X18,		18, false, 0 },
	{ MEDIA_BUS_FMT_RGB888_1X24,		24, false, 0 },
	{ MEDIA_BUS_FMT_RGB666_1X7X3_SPWG,	18, true, SPWG_18 },
	{ MEDIA_BUS_FMT_RGB888_1X7X4_SPWG,	24, true, SPWG_24 },
	{ MEDIA_BUS_FMT_RGB888_1X7X4_JEIDA,	24, true, JEIDA_24 },
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

#define REG_GET(dispc, idx, start, end) \
	FLD_GET(dispc7_read(dispc, idx), start, end)

#define REG_FLD_MOD(dispc, idx, val, start, end) \
	dispc7_write(dispc, idx, FLD_MOD(dispc7_read(dispc, idx), val, start, end))

#define VID_REG_GET(dispc, hw_plane, idx, start, end) \
	FLD_GET(dispc7_vid_read(dispc, hw_plane, idx), start, end)

#define VID_REG_FLD_MOD(dispc, hw_plane, idx, val, start, end) \
	dispc7_vid_write(dispc, hw_plane, idx, FLD_MOD(dispc7_vid_read(dispc, hw_plane, idx), val, start, end))

#define VP_REG_GET(dispc, vp, idx, start, end) \
	FLD_GET(dispc7_vp_read(dispc, vp, idx), start, end)

#define VP_REG_FLD_MOD(dispc, vp, idx, val, start, end) \
	dispc7_vp_write(dispc, vp, idx, FLD_MOD(dispc7_vp_read(dispc, vp, idx), val, start, end))

#define OVR_REG_GET(dispc, ovr, idx, start, end) \
	FLD_GET(dispc7_ovr_read(dispc, ovr, idx), start, end)

#define OVR_REG_FLD_MOD(dispc, ovr, idx, val, start, end) \
	dispc7_ovr_write(dispc, ovr, idx, FLD_MOD(dispc7_ovr_read(dispc, ovr, idx), val, start, end))

struct dss_vp_data {
	u32 gamma_table[256];
	bool oldi;
};

struct dss_plane_data {
	uint zorder;
	uint hw_videoport;
};

struct dispc_device {
	struct tidss_device *tidss;
	struct device *dev;

	void __iomem *base_common;
	void __iomem *base_vid[DISPC7_MAX_PLANES];
	void __iomem *base_ovr[DISPC7_MAX_PORTS];
	void __iomem *base_vp[DISPC7_MAX_PORTS];

	struct regmap *syscon;

	struct clk *vp_clk[DISPC7_MAX_PORTS];

	const struct dispc7_features *feat;

	struct clk *fclk;

	bool is_enabled;

	struct dss_vp_data vp_data[DISPC7_MAX_PORTS];

	struct dss_plane_data plane_data[DISPC7_MAX_PLANES];
};


static void dispc7_write(struct dispc_device *dispc, u16 reg, u32 val)
{
	iowrite32(val, dispc->base_common + reg);
}

static u32 dispc7_read(struct dispc_device *dispc, u16 reg)
{
	return ioread32(dispc->base_common + reg);
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

static u64 dispc7_vp_read_irqstatus(struct dispc_device *dispc,
				    u32 hw_videoport)
{
	u32 stat = dispc7_read(dispc, DISPC_VP_IRQSTATUS(hw_videoport));

	return dispc7_vp_irq_from_raw(stat, hw_videoport);
}

static void dispc7_vp_write_irqstatus(struct dispc_device *dispc,
				      u32 hw_videoport, u64 vpstat)
{
	u32 stat = dispc7_vp_irq_to_raw(vpstat, hw_videoport);

	dispc7_write(dispc, DISPC_VP_IRQSTATUS(hw_videoport), stat);
}

static u64 dispc7_vid_read_irqstatus(struct dispc_device *dispc,
				     u32 hw_plane)
{
	u32 stat = dispc7_read(dispc, DISPC_VID_IRQSTATUS(hw_plane));

	return dispc7_vid_irq_from_raw(stat, hw_plane);
}

static void dispc7_vid_write_irqstatus(struct dispc_device *dispc,
				       u32 hw_plane, u64 vidstat)
{
	u32 stat = dispc7_vid_irq_to_raw(vidstat, hw_plane);

	dispc7_write(dispc, DISPC_VID_IRQSTATUS(hw_plane), stat);
}

static u64 dispc7_vp_read_irqenable(struct dispc_device *dispc,
				    u32 hw_videoport)
{
	u32 stat = dispc7_read(dispc, DISPC_VP_IRQENABLE(hw_videoport));

	return dispc7_vp_irq_from_raw(stat, hw_videoport);
}

static void dispc7_vp_write_irqenable(struct dispc_device *dispc,
				      u32 hw_videoport, u64 vpstat)
{
	u32 stat = dispc7_vp_irq_to_raw(vpstat, hw_videoport);

	dispc7_write(dispc, DISPC_VP_IRQENABLE(hw_videoport), stat);
}


static u64 dispc7_vid_read_irqenable(struct dispc_device *dispc,
				     u32 hw_plane)
{
	u32 stat = dispc7_read(dispc, DISPC_VID_IRQENABLE(hw_plane));

	return dispc7_vid_irq_from_raw(stat, hw_plane);
}

static void dispc7_vid_write_irqenable(struct dispc_device *dispc,
				       u32 hw_plane, u64 vidstat)
{
	u32 stat = dispc7_vid_irq_to_raw(vidstat, hw_plane);

	dispc7_write(dispc, DISPC_VID_IRQENABLE(hw_plane), stat);
}

static void dispc7_clear_irqstatus(struct dispc_device *dispc, u64 clearmask)
{
	uint i;
	u32 top_clear = 0;

	for (i = 0; i < dispc->feat->num_vps; ++i) {
		if (clearmask & DSS_IRQ_VP_MASK(i)) {
			dispc7_vp_write_irqstatus(dispc, i, clearmask);
			top_clear |= BIT(i);
		}
	}
	for (i = 0; i < dispc->feat->num_planes; ++i) {
		if (clearmask & DSS_IRQ_PLANE_MASK(i)) {
			dispc7_vid_write_irqstatus(dispc, i, clearmask);
			top_clear |= BIT(4 + i);
		}
	}
	dispc7_write(dispc, DISPC_IRQSTATUS, top_clear);

	/* Flush posted writes */
	dispc7_read(dispc, DISPC_IRQSTATUS);
}

static u64 dispc7_read_and_clear_irqstatus(struct dispc_device *dispc)
{
	u64 status = 0;
	uint i;

	for (i = 0; i < dispc->feat->num_vps; ++i)
		status |= dispc7_vp_read_irqstatus(dispc, i);

	for (i = 0; i < dispc->feat->num_planes; ++i)
		status |= dispc7_vid_read_irqstatus(dispc, i);

	dispc7_clear_irqstatus(dispc, status);

	return status;
}

static u64 dispc7_read_irqenable(struct dispc_device *dispc)
{
	u64 enable = 0;
	uint i;

	for (i = 0; i < dispc->feat->num_vps; ++i)
		enable |= dispc7_vp_read_irqenable(dispc, i);

	for (i = 0; i < dispc->feat->num_planes; ++i)
		enable |= dispc7_vid_read_irqenable(dispc, i);

	return enable;
}

static void dispc7_write_irqenable(struct dispc_device *dispc, u64 mask)
{
	uint i;
	u32 main_enable = 0, main_disable = 0;
	u64 old_mask;

	old_mask = dispc7_read_irqenable(dispc);

	/* clear the irqstatus for newly enabled irqs */
	dispc7_clear_irqstatus(dispc, (old_mask ^ mask) & mask);

	for (i = 0; i < dispc->feat->num_vps; ++i) {
		dispc7_vp_write_irqenable(dispc, i, mask);
		if (mask & DSS_IRQ_VP_MASK(i))
			main_enable |= BIT(i);		/* VP IRQ */
		else
			main_disable |= BIT(i);		/* VP IRQ */
	}

	for (i = 0; i < dispc->feat->num_planes; ++i) {
		dispc7_vid_write_irqenable(dispc, i, mask);
		if (mask & DSS_IRQ_PLANE_MASK(i))
			main_enable |= BIT(i + 4);	/* VID IRQ */
		else
			main_disable |= BIT(i + 4);	/* VID IRQ */
	}

	if (main_enable)
		dispc7_write(dispc, DISPC_IRQENABLE_SET, main_enable);

	if (main_disable)
		dispc7_write(dispc, DISPC_IRQENABLE_CLR, main_disable);

	/* Flush posted writes */
	dispc7_read(dispc, DISPC_IRQENABLE_SET);
}


static void dispc7_oldi_tx_power(struct dispc_device *dispc, bool power)
{
	u32 val = power ? 0 : CTRLMMR0P1_OLDI_PWRDN_TX;

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

	oldi_cfg = FLD_MOD(oldi_cfg, fmt->oldi_mode, 3, 1);

	oldi_cfg |= BIT(12); /* SOFTRST */

	oldi_cfg |= BIT(0); /* ENABLE */

	dispc7_vp_write(dispc, hw_videoport, DISPC_VP_DSS_OLDI_CFG, oldi_cfg);

	while (!(oldi_reset_bit & dispc7_read(dispc, DSS_SYSSTATUS)) &&
	       count < 10000)
		count++;

	if (!(oldi_reset_bit & dispc7_read(dispc, DSS_SYSSTATUS)))
		dev_warn(dispc->dev, "%s: timeout waiting OLDI reset done\n",
			 __func__);
}

static void dispc7_vp_prepare(struct dispc_device *dispc, u32 hw_videoport,
			      const struct drm_display_mode *mode,
			      u32 bus_fmt, u32 bus_flags)
{
	const struct dispc7_bus_format *fmt = NULL;
	int i;

	for (i = 0; i < ARRAY_SIZE(dispc7_bus_formats); ++i) {
		if (dispc7_bus_formats[i].bus_fmt != bus_fmt)
			continue;

		fmt = &dispc7_bus_formats[i];
		break;
	}

	if (WARN_ON(!fmt))
		return;

	if (fmt->oldi) {
		dispc7_oldi_tx_power(dispc, true);

		dispc7_enable_oldi(dispc, hw_videoport, fmt);

		dispc->vp_data[hw_videoport].oldi = true;
	}
}

static void dispc7_vp_enable(struct dispc_device *dispc, u32 hw_videoport,
			     const struct drm_display_mode *mode,
			     u32 bus_fmt, u32 bus_flags)
{
	bool align, onoff, rf, ieo, ipc, ihs, ivs;
	int i;
	const struct dispc7_bus_format *fmt = NULL;
	u32 hsw, hfp, hbp, vsw, vfp, vbp;

	for (i = 0; i < ARRAY_SIZE(dispc7_bus_formats); ++i) {
		if (dispc7_bus_formats[i].bus_fmt != bus_fmt)
			continue;

		fmt = &dispc7_bus_formats[i];
		break;
	}

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

	if (mode->flags & DRM_MODE_FLAG_NVSYNC)
		ivs = true;
	else
		ivs = false;

	if (mode->flags & DRM_MODE_FLAG_NHSYNC)
		ihs = true;
	else
		ihs = false;

	if (bus_flags & DRM_BUS_FLAG_DE_LOW)
		ieo = true;
	else
		ieo = false;

	if (bus_flags & DRM_BUS_FLAG_PIXDATA_NEGEDGE)
		ipc = true;
	else
		ipc = false;

	/* always use the 'rf' setting */
	onoff = true;

	if (bus_flags & DRM_BUS_FLAG_SYNC_NEGEDGE)
		rf = false;
	else
		rf = true;

	/* always use aligned syncs */
	align = true;

	/* always use DE_HIGH for OLDI */
	if (fmt->oldi)
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
	if (dispc->vp_data[hw_videoport].oldi) {
		dispc7_vp_write(dispc, hw_videoport, DISPC_VP_DSS_OLDI_CFG, 0);

		dispc7_oldi_tx_power(dispc, false);

		dispc->vp_data[hw_videoport].oldi = false;
	}
}

static bool dispc7_vp_go_busy(struct dispc_device *dispc,
			      u32 hw_videoport)
{
	return VP_REG_GET(dispc, hw_videoport, DISPC_VP_CONTROL, 5, 5);
}

static void dispc7_vp_go(struct dispc_device *dispc, u32 hw_videoport)
{
	VP_REG_FLD_MOD(dispc, hw_videoport, DISPC_VP_CONTROL, 1, 5, 5);
}

static u16 c8_to_c12(u8 c8)
{
	u16 c12;

	c12 = c8 << 4;

	/* Replication logic: Copy c8 4 MSB to 4 LSB for full scale c12 */
	c12 |= c8 >> 4;

	return c12;
}

static u64 argb8888_to_argb12121212(u32 argb8888)
{
	u8 a, r, g, b;
	u64 v;

	a = (argb8888 >> 24) & 0xff;
	r = (argb8888 >> 16) & 0xff;
	g = (argb8888 >> 8) & 0xff;
	b = (argb8888 >> 0) & 0xff;

	v = ((u64)c8_to_c12(a) << 36) | ((u64)c8_to_c12(r) << 24) |
	    ((u64)c8_to_c12(g) << 12) | (u64)c8_to_c12(b);

	return v;
}

static void dispc7_vp_setup(struct dispc_device *dispc, u32 hw_videoport,
			    const struct tidss_vp_info *info)
{
	u64 v;

	v = argb8888_to_argb12121212(info->default_color);

	dispc7_ovr_write(dispc, hw_videoport,
			 DISPC_OVR_DEFAULT_COLOR, v & 0xffffffff);
	dispc7_ovr_write(dispc, hw_videoport,
			 DISPC_OVR_DEFAULT_COLOR2, (v >> 32) & 0xffff);
}

static enum drm_mode_status dispc7_vp_check_mode(struct dispc_device *dispc,
						 u32 hw_videoport,
						 const struct drm_display_mode *mode)
{
	u32 hsw, hfp, hbp, vsw, vfp, vbp;

	if (mode->clock * 1000 < dispc->feat->min_pclk)
		return MODE_CLOCK_LOW;

	if (mode->clock * 1000 > dispc->feat->max_pclk)
		return MODE_CLOCK_HIGH;

	if (mode->hdisplay > 4096)
		return MODE_BAD;

	if (mode->vdisplay > 4096)
		return MODE_BAD;

	/* TODO: add interlace support */
	if (mode->flags & DRM_MODE_FLAG_INTERLACE)
		return MODE_NO_INTERLACE;

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
	    vfp < 0 || vfp > 4095 ||
	    vbp < 0 || vbp > 4095)
		return MODE_BAD_VVALUE;

	return MODE_OK;
}

static int dispc7_vp_check_config(struct dispc_device *dispc, u32 hw_videoport,
				  const struct drm_display_mode *mode,
				  u32 bus_fmt, u32 bus_flags)
{
	enum drm_mode_status ok;
	int i;

	ok = dispc7_vp_check_mode(dispc, hw_videoport, mode);
	if (ok != MODE_OK)
		return -EINVAL;


	for (i = 0; i < ARRAY_SIZE(dispc7_bus_formats); ++i) {
		if (dispc7_bus_formats[i].bus_fmt == bus_fmt)
			break;
	}

	if (i == ARRAY_SIZE(dispc7_bus_formats))
		return -EINVAL;

	return 0;
}

static int dispc7_vp_enable_clk(struct dispc_device *dispc, u32 hw_videoport)
{
	return clk_prepare_enable(dispc->vp_clk[hw_videoport]);
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
		dev_err(dispc->dev, "Failed to set vp%d clk rate to %lu\n",
			hw_videoport, rate);
		return r;
	}

	new_rate = clk_get_rate(dispc->vp_clk[hw_videoport]);

	if (dispc7_pclk_diff(rate, new_rate) > 5)
		dev_warn(dispc->dev,
			 "Clock rate %lu differs over 5%% from requsted %lu\n",
			 new_rate, rate);

	dev_dbg(dispc->dev, "New VP%d rate %lu Hz (requested %lu Hz)\n",
		hw_videoport, clk_get_rate(dispc->vp_clk[hw_videoport]), rate);

	return 0;
}

/* CSC */

struct color_conv_coef {
	int ry, rcb, rcr;
	int gy, gcb, gcr;
	int by, bcb, bcr;
	int roffset, goffset, boffset;
	bool full_range;
};

static void dispc7_vid_write_color_conv_coefs(struct dispc_device *dispc,
					      u32 hw_plane,
					      const struct color_conv_coef *ct)
{
#define CVAL(x, y) (FLD_VAL(x, 26, 16) | FLD_VAL(y, 10, 0))

	dispc7_vid_write(dispc, hw_plane,
			 DISPC_VID_CSC_COEF(0), CVAL(ct->rcr, ct->ry));
	dispc7_vid_write(dispc, hw_plane,
			 DISPC_VID_CSC_COEF(1), CVAL(ct->gy,  ct->rcb));
	dispc7_vid_write(dispc, hw_plane,
			 DISPC_VID_CSC_COEF(2), CVAL(ct->gcb, ct->gcr));
	dispc7_vid_write(dispc, hw_plane,
			 DISPC_VID_CSC_COEF(3), CVAL(ct->bcr, ct->by));
	dispc7_vid_write(dispc, hw_plane,
			 DISPC_VID_CSC_COEF(4), CVAL(0, ct->bcb));

	dispc7_vid_write(dispc, hw_plane, DISPC_VID_CSC_COEF(5),
			 FLD_VAL(ct->roffset, 15, 3) |
			 FLD_VAL(ct->goffset, 31, 19));
	dispc7_vid_write(dispc, hw_plane, DISPC_VID_CSC_COEF(6),
			 FLD_VAL(ct->boffset, 15, 3));

	VID_REG_FLD_MOD(dispc, hw_plane, DISPC_VID_ATTRIBUTES,
			ct->full_range, 11, 11);

#undef CVAL
}

static void dispc7_vid_csc_setup(struct dispc_device *dispc)
{
	/* YUV -> RGB, ITU-R BT.601, full range */
	const struct color_conv_coef yuv2rgb_bt601_full = {
		256,   0,  358,
		256, -88, -182,
		256, 452,    0,
		0, -2048, -2048,
		true,
	};
	int i;

	for (i = 0; i < dispc->feat->num_planes; i++)
		dispc7_vid_write_color_conv_coefs(dispc, i,
						  &yuv2rgb_bt601_full);
}

static void dispc7_vid_csc_enable(struct dispc_device *dispc, u32 hw_plane,
				  bool enable)
{
	VID_REG_FLD_MOD(dispc, hw_plane, DISPC_VID_ATTRIBUTES, !!enable, 9, 9);
}

/* SCALER */

static u32 dispc7_calc_fir_inc(uint in, uint out)
{
	return (u32)div_u64(0x200000ull * in, out);
}

struct dispc7_vid_fir_coefs {
	s16 c2[16];
	s16 c1[16];
	u16 c0[9];
};

static const struct dispc7_vid_fir_coefs dispc7_fir_coefs_null = {
	.c2 = {	0 },
	.c1 = { 0 },
	.c0 = { 512, 512, 512, 512, 512, 512, 512, 512, 256,  },
};

/* M=8, Upscale x >= 1 */
static const struct dispc7_vid_fir_coefs dispc7_fir_coefs_m8 = {
	.c2 = {	0, -4, -8, -16, -24, -32, -40, -48, 0, 2, 4, 6, 8, 6, 4, 2,  },
	.c1 = { 0, 28, 56, 94, 132, 176, 220, 266, -56, -60, -64, -62, -60, -50, -40, -20,  },
	.c0 = { 512, 506, 500, 478, 456, 424, 392, 352, 312,  },
};

/* 5-tap, M=22, Downscale Ratio 2.5 < x < 3 */
static const struct dispc7_vid_fir_coefs dispc7_fir_coefs_m22_5tap = {
	.c2 = { 16, 20, 24, 30, 36, 42, 48, 56, 0, 0, 0, 2, 4, 8, 12, 14,  },
	.c1 = { 132, 140, 148, 156, 164, 172, 180, 186, 64, 72, 80, 88, 96, 104, 112, 122,  },
	.c0 = { 216, 216, 216, 214, 212, 208, 204, 198, 192,  },
};

/* 3-tap, M=22, Downscale Ratio 2.5 < x < 3 */
static const struct dispc7_vid_fir_coefs dispc7_fir_coefs_m22_3tap = {
	.c1 = { 100, 118, 136, 156, 176, 196, 216, 236, 0, 10, 20, 30, 40, 54, 68, 84,	},
	.c0 = { 312, 310, 308, 302, 296, 286, 276, 266, 256,  },
};

enum dispc7_vid_fir_coef_set {
	DISPC7_VID_FIR_COEF_HORIZ,
	DISPC7_VID_FIR_COEF_HORIZ_UV,
	DISPC7_VID_FIR_COEF_VERT,
	DISPC7_VID_FIR_COEF_VERT_UV,
};

static void dispc7_vid_write_fir_coefs(struct dispc_device *dispc,
				       u32 hw_plane,
				       enum dispc7_vid_fir_coef_set coef_set,
				       const struct dispc7_vid_fir_coefs *coefs)
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

static void dispc7_vid_write_scale_coefs(struct dispc_device *dispc,
					 u32 hw_plane)
{
	dispc7_vid_write_fir_coefs(dispc, hw_plane, DISPC7_VID_FIR_COEF_HORIZ, &dispc7_fir_coefs_null);
	dispc7_vid_write_fir_coefs(dispc, hw_plane, DISPC7_VID_FIR_COEF_HORIZ_UV, &dispc7_fir_coefs_null);
	dispc7_vid_write_fir_coefs(dispc, hw_plane, DISPC7_VID_FIR_COEF_VERT, &dispc7_fir_coefs_null);
	dispc7_vid_write_fir_coefs(dispc, hw_plane, DISPC7_VID_FIR_COEF_VERT_UV, &dispc7_fir_coefs_null);
}

static void dispc7_vid_set_scaling(struct dispc_device *dispc,
				   u32 hw_plane,
				   uint orig_width, uint orig_height,
				   uint out_width, uint out_height,
				   u32 fourcc)
{
	uint in_w, in_h, in_w_uv, in_h_uv;
	uint fir_hinc, fir_vinc, fir_hinc_uv, fir_vinc_uv;
	bool scale_x, scale_y;
	bool five_taps = false;		/* XXX always 3-tap for now */

	in_w = in_w_uv = orig_width;
	in_h = in_h_uv = orig_height;

	switch (fourcc) {
	case DRM_FORMAT_NV12:
		/* UV is subsampled by 2 horizontally and vertically */
		in_h_uv >>= 1;
		in_w_uv >>= 1;
		break;

	case DRM_FORMAT_YUYV:
	case DRM_FORMAT_UYVY:
		/* UV is subsampled by 2 horizontally */
		in_w_uv >>= 1;
		break;

	default:
		break;
	}

	scale_x = in_w != out_width || in_w_uv != out_width;
	scale_y = in_h != out_height || in_h_uv != out_height;

	/* HORIZONTAL RESIZE ENABLE */
	VID_REG_FLD_MOD(dispc, hw_plane, DISPC_VID_ATTRIBUTES, scale_x, 7, 7);

	/* VERTICAL RESIZE ENABLE */
	VID_REG_FLD_MOD(dispc, hw_plane, DISPC_VID_ATTRIBUTES, scale_y, 8, 8);

	/* Skip the rest if no scaling is used */
	if (!scale_x && !scale_y)
		return;

	/* VERTICAL 5-TAPS  */
	VID_REG_FLD_MOD(dispc, hw_plane, DISPC_VID_ATTRIBUTES,
			five_taps, 21, 21);

	/* FIR INC */

	fir_hinc = dispc7_calc_fir_inc(in_w, out_width);
	fir_vinc = dispc7_calc_fir_inc(in_h, out_height);
	fir_hinc_uv = dispc7_calc_fir_inc(in_w_uv, out_width);
	fir_vinc_uv = dispc7_calc_fir_inc(in_h_uv, out_height);

	dispc7_vid_write(dispc, hw_plane, DISPC_VID_FIRH, fir_hinc);
	dispc7_vid_write(dispc, hw_plane, DISPC_VID_FIRV, fir_vinc);
	dispc7_vid_write(dispc, hw_plane, DISPC_VID_FIRH2, fir_hinc_uv);
	dispc7_vid_write(dispc, hw_plane, DISPC_VID_FIRV2, fir_vinc_uv);

	dispc7_vid_write_scale_coefs(dispc, hw_plane);
}

/* OTHER */

static const struct {
	u32 fourcc;
	u8 dss_code;
	u8 bytespp;
} dispc7_color_formats[] = {
	{ DRM_FORMAT_ARGB4444, 0x0, 2, },
	{ DRM_FORMAT_ABGR4444, 0x1, 2, },
	{ DRM_FORMAT_RGBA4444, 0x2, 2, },

	{ DRM_FORMAT_RGB565, 0x3, 2, },
	{ DRM_FORMAT_BGR565, 0x4, 2, },

	{ DRM_FORMAT_ARGB1555, 0x5, 2, },
	{ DRM_FORMAT_ABGR1555, 0x6, 2, },

	{ DRM_FORMAT_ARGB8888, 0x7, 4, },
	{ DRM_FORMAT_ABGR8888, 0x8, 4, },
	{ DRM_FORMAT_RGBA8888, 0x9, 4, },
	{ DRM_FORMAT_BGRA8888, 0xa, 4, },

	{ DRM_FORMAT_RGB888, 0xb, 3, },
	{ DRM_FORMAT_BGR888, 0xc, 3, },

	{ DRM_FORMAT_ARGB2101010, 0xe, 4, },
	{ DRM_FORMAT_ABGR2101010, 0xf, 4, },
	{ DRM_FORMAT_RGBA1010102, 0x10, 4, },
	{ DRM_FORMAT_BGRA1010102, 0x11, 4, },

	{ DRM_FORMAT_XRGB4444, 0x20, 2, },
	{ DRM_FORMAT_XBGR4444, 0x21, 2, },
	{ DRM_FORMAT_RGBX4444, 0x22, 2, },

	{ DRM_FORMAT_ARGB1555, 0x25, 2, },
	{ DRM_FORMAT_ABGR1555, 0x26, 2, },

	{ DRM_FORMAT_XRGB8888, 0x27, 4, },
	{ DRM_FORMAT_XBGR8888, 0x28, 4, },
	{ DRM_FORMAT_RGBX8888, 0x29, 4, },
	{ DRM_FORMAT_BGRX8888, 0x2a, 4, },

	{ DRM_FORMAT_XRGB2101010, 0x2e, 4, },
	{ DRM_FORMAT_XBGR2101010, 0x2f, 4, },
	{ DRM_FORMAT_RGBX1010102, 0x30, 4, },
	{ DRM_FORMAT_BGRX1010102, 0x31, 4, },

	{ DRM_FORMAT_YUYV, 0x3e, 2, },
	{ DRM_FORMAT_UYVY, 0x3f, 2, },

	{ DRM_FORMAT_NV12, 0x3d, 2, },
};

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

static void dispc7_plane_set_pixel_format(struct dispc_device *dispc,
					  u32 hw_plane, u32 fourcc)
{
	int i;

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

static int dispc7_fourcc_to_bytespp(u32 fourcc)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dispc7_color_formats); ++i) {
		if (dispc7_color_formats[i].fourcc == fourcc)
			return dispc7_color_formats[i].bytespp;
	}

	WARN_ON(1);
	return 4;
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

static int dispc7_plane_setup(struct dispc_device *dispc, u32 hw_plane,
			      const struct tidss_plane_info *pi,
			      u32 hw_videoport)
{
	bool lite = dispc->feat->vid_lite[hw_plane];
	u32 fourcc = pi->fourcc;
	int bytespp = dispc7_fourcc_to_bytespp(fourcc);

	if (dispc7_fourcc_is_yuv(fourcc) && (pi->width & 1)) {
		dev_err(dispc->dev, "non even input width %d for YUV format\n",
			pi->width);
		return -EINVAL;
	}

	dispc7_plane_set_pixel_format(dispc, hw_plane, fourcc);

	dispc7_vid_write(dispc, hw_plane, DISPC_VID_BA_0, pi->paddr & 0xffffffff);
	dispc7_vid_write(dispc, hw_plane, DISPC_VID_BA_EXT_0, (u64)pi->paddr >> 32);
	dispc7_vid_write(dispc, hw_plane, DISPC_VID_BA_1, pi->paddr & 0xffffffff);
	dispc7_vid_write(dispc, hw_plane, DISPC_VID_BA_EXT_1, (u64)pi->paddr >> 32);

	dispc7_vid_write(dispc, hw_plane, DISPC_VID_BA_UV_0, pi->p_uv_addr & 0xffffffff);
	dispc7_vid_write(dispc, hw_plane, DISPC_VID_BA_UV_EXT_0, (u64)pi->p_uv_addr >> 32);
	dispc7_vid_write(dispc, hw_plane, DISPC_VID_BA_UV_1, pi->p_uv_addr & 0xffffffff);
	dispc7_vid_write(dispc, hw_plane, DISPC_VID_BA_UV_EXT_1, (u64)pi->p_uv_addr >> 32);

	dispc7_vid_write(dispc, hw_plane, DISPC_VID_PICTURE_SIZE,
			 (pi->width - 1) | ((pi->height - 1) << 16));

	dispc7_vid_write(dispc, hw_plane, DISPC_VID_PIXEL_INC,
			 pixinc(1, bytespp));
	dispc7_vid_write(dispc, hw_plane, DISPC_VID_ROW_INC,
			 pixinc(1 + pi->fb_width - pi->width, bytespp));

	if (fourcc == DRM_FORMAT_NV12)
		dispc7_vid_write(dispc, hw_plane, DISPC_VID_ROW_INC_UV,
				 pixinc(1 + pi->fb_width - pi->width, bytespp));

	if (!lite) {
		dispc7_vid_write(dispc, hw_plane, DISPC_VID_SIZE,
				 (pi->out_width - 1) |
				 ((pi->out_height - 1) << 16));

		dispc7_vid_set_scaling(dispc, hw_plane,
				       pi->width, pi->height,
				       pi->out_width, pi->out_height,
				       fourcc);
	}

	/* enable YUV->RGB color conversion */
	if (dispc7_fourcc_is_yuv(fourcc))
		dispc7_vid_csc_enable(dispc, hw_plane, true);
	else
		dispc7_vid_csc_enable(dispc, hw_plane, false);

	OVR_REG_FLD_MOD(dispc, hw_videoport, DISPC_OVR_ATTRIBUTES(pi->zorder),
			hw_plane, 4, 1);
	OVR_REG_FLD_MOD(dispc, hw_videoport, DISPC_OVR_ATTRIBUTES(pi->zorder),
			pi->pos_x, 17, 6);
	OVR_REG_FLD_MOD(dispc, hw_videoport, DISPC_OVR_ATTRIBUTES(pi->zorder),
			pi->pos_y, 30, 19);

	dispc->plane_data[hw_plane].zorder = pi->zorder;
	dispc->plane_data[hw_plane].hw_videoport = hw_videoport;

	return 0;
}

static int dispc7_plane_enable(struct dispc_device *dispc,
			       u32 hw_plane, bool enable)
{
	OVR_REG_FLD_MOD(dispc, dispc->plane_data[hw_plane].hw_videoport,
			DISPC_OVR_ATTRIBUTES(dispc->plane_data[hw_plane].zorder),
			!!enable, 0, 0);
	VID_REG_FLD_MOD(dispc, hw_plane, DISPC_VID_ATTRIBUTES, !!enable, 0, 0);
	return 0;
}

static u32 dispc7_vid_get_fifo_size(struct dispc_device *dispc,
				    u32 hw_plane)
{
	const u32 unit_size = 16;	/* 128-bits */

	return VID_REG_GET(dispc, hw_plane, DISPC_VID_BUF_SIZE_STATUS, 15, 0) *
	       unit_size;
}

static void dispc7_vid_set_mflag_threshold(struct dispc_device *dispc,
					   u32 hw_plane, uint low, uint high)
{
	dispc7_vid_write(dispc, hw_plane, DISPC_VID_MFLAG_THRESHOLD,
			 FLD_VAL(high, 31, 16) | FLD_VAL(low, 15, 0));
}

static void __maybe_unused dispc7_mflag_setup(struct dispc_device *dispc)
{
	u32 hw_plane = 0;
	const u32 unit_size = 16;	/* 128-bits */
	u32 size = dispc7_vid_get_fifo_size(dispc, hw_plane);
	u32 low, high;

	/* MFLAG_CTRL = MFLAGFORCE */
	REG_FLD_MOD(dispc, DISPC_GLOBAL_MFLAG_ATTRIBUTE, 1, 1, 0);
	/* MFLAG_START = MFLAGNORMALSTARTMODE */
	REG_FLD_MOD(dispc, DISPC_GLOBAL_MFLAG_ATTRIBUTE, 0, 6, 6);

	/*
	 * Simulation team suggests below thesholds:
	 * HT = fifosize * 5 / 8;
	 * LT = fifosize * 4 / 8;
	 */

	low = size * 4 / 8 / unit_size;
	high = size * 5 / 8 / unit_size;

	dispc7_vid_set_mflag_threshold(dispc, hw_plane, low, high);
}

static void dispc7_vp_init(struct dispc_device *dispc)
{
	unsigned int i;

	dev_dbg(dispc->dev, "%s()\n", __func__);

	/* Enable the gamma Shadow bit-field for all VPs*/
	for (i = 0; i < dispc->feat->num_vps; i++)
		VP_REG_FLD_MOD(dispc, i, DISPC_VP_CONFIG, 1, 2, 2);
}

static void dispc7_initial_config(struct dispc_device *dispc)
{
	dispc7_vid_csc_setup(dispc);
	//dispc7_mflag_setup(dispc);
	dispc7_vp_init(dispc);
}

static int dispc7_get_num_planes(struct dispc_device *dispc)
{
	return dispc->feat->num_planes;
}

static int dispc7_get_num_vps(struct dispc_device *dispc)
{
	return dispc->feat->num_vps;
}

static u32 dispc7_vp_gamma_size(struct dispc_device *dispc,
				u32 hw_videoport)
{
	return ARRAY_SIZE(dispc->vp_data[hw_videoport].gamma_table);
}

static void dispc7_vp_write_gamma_table(struct dispc_device *dispc,
					u32 hw_videoport)
{
	u32 *table = dispc->vp_data[hw_videoport].gamma_table;
	uint hwlen = ARRAY_SIZE(dispc->vp_data[hw_videoport].gamma_table);
	unsigned int i;

	dev_dbg(dispc->dev, "%s: hw_videoport %d\n", __func__, hw_videoport);

	for (i = 0; i < hwlen; ++i) {
		u32 v = table[i];

		v |= i << 24;

		dispc7_vp_write(dispc, hw_videoport, DISPC_VP_GAMMA_TABLE, v);
	}
}

static void dispc7_restore_gamma_tables(struct dispc_device *dispc)
{
	unsigned int i;

	dev_dbg(dispc->dev, "%s()\n", __func__);

	for (i = 0; i < dispc->feat->num_vps; i++)
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
	uint hwlen = ARRAY_SIZE(dispc->vp_data[hw_videoport].gamma_table);
	static const uint hwbits = 8;
	uint i;

	dev_dbg(dispc->dev, "%s: hw_videoport %d, lut len %u, hw len %u\n",
		__func__, hw_videoport, length, hwlen);

	if (lut == NULL || length < 2) {
		lut = dispc7_vp_gamma_default_lut;
		length = ARRAY_SIZE(dispc7_vp_gamma_default_lut);
	}

	for (i = 0; i < length - 1; ++i) {
		uint first = i * (hwlen - 1) / (length - 1);
		uint last = (i + 1) * (hwlen - 1) / (length - 1);
		uint w = last - first;
		u16 r, g, b;
		uint j;

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

static int dispc7_init_gamma_tables(struct dispc_device *dispc)
{
	unsigned int i;

	dev_dbg(dispc->dev, "%s()\n", __func__);

	for (i = 0; i < dispc->feat->num_vps; i++)
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

	if (REG_GET(dispc, DSS_SYSSTATUS, 0, 0) == 0)
		dev_warn(dispc->dev, "DSS FUNC RESET not done!\n");

	dev_dbg(dispc->dev, "OMAP DSS7 rev 0x%x\n",
		dispc7_read(dispc, DSS_REVISION));

	dev_dbg(dispc->dev, "VP RESETDONE %d,%d,%d\n",
		REG_GET(dispc, DSS_SYSSTATUS, 1, 1),
		REG_GET(dispc, DSS_SYSSTATUS, 2, 2),
		REG_GET(dispc, DSS_SYSSTATUS, 3, 3));

	dev_dbg(dispc->dev, "OLDI RESETDONE %d,%d,%d\n",
		REG_GET(dispc, DSS_SYSSTATUS, 5, 5),
		REG_GET(dispc, DSS_SYSSTATUS, 6, 6),
		REG_GET(dispc, DSS_SYSSTATUS, 7, 7));

	dev_dbg(dispc->dev, "DISPC IDLE %d\n",
		REG_GET(dispc, DSS_SYSSTATUS, 9, 9));

	dispc7_initial_config(dispc);

	dispc7_restore_gamma_tables(dispc);

	dispc->is_enabled = true;

	return 0;
}

static int dispc7_modeset_init(struct dispc_device *dispc)
{
	struct tidss_device *tidss = dispc->tidss;
	struct device *dev = tidss->dev;
	u32 fourccs[ARRAY_SIZE(dispc7_color_formats)];
	int i;

	struct pipe {
		u32 hw_videoport;
		struct drm_bridge *bridge;
		u32 enc_type;
		struct device_node *epnode;
	};

	u32 max_vps = dispc->feat->num_vps;
	u32 max_planes = dispc->feat->num_planes;

	struct pipe pipes[DISPC7_MAX_PORTS];
	u32 num_pipes = 0;
	u32 crtc_mask;

	for (i = 0; i < ARRAY_SIZE(fourccs); ++i)
		fourccs[i] = dispc7_color_formats[i].fourcc;

	/* first find all the connected panels & bridges */

	for (i = 0; i < max_vps; i++) {
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

			enc_type = dispc->feat->vp_enc_type[i];
			switch (enc_type) {
			case DRM_MODE_ENCODER_LVDS:
				conn_type = DRM_MODE_CONNECTOR_LVDS;
				break;
			case DRM_MODE_ENCODER_DPI:
				conn_type = DRM_MODE_CONNECTOR_DPI;
				break;
			default:
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
		pipes[num_pipes].epnode =
			of_graph_get_endpoint_by_regs(dev->of_node, i, 0);
		if (WARN_ON(!pipes[num_pipes].epnode))
			return -EINVAL;

		num_pipes++;
	}

	/* all planes can be on any crtc */
	crtc_mask = (1 << num_pipes) - 1;

	/* then create a plane, a crtc and an encoder for each panel/bridge */

	for (i = 0; i < num_pipes; ++i) {
		struct tidss_plane *tplane;
		struct tidss_crtc *tcrtc;
		struct tidss_encoder *tenc;
		u32 hw_plane_id = tidss->num_planes;
		int ret;

		tplane = tidss_plane_create(tidss, hw_plane_id,
					    DRM_PLANE_TYPE_PRIMARY, crtc_mask,
					    fourccs, ARRAY_SIZE(fourccs));
		if (IS_ERR(tplane)) {
			dev_err(tidss->dev, "plane create failed\n");
			return PTR_ERR(tplane);
		}

		tidss->planes[tidss->num_planes++] = &tplane->plane;

		tcrtc = tidss_crtc_create(tidss, pipes[i].hw_videoport,
					  &tplane->plane, pipes[i].epnode);
		if (IS_ERR(tcrtc)) {
			dev_err(tidss->dev, "crtc create failed\n");
			return PTR_ERR(tcrtc);
		}

		tidss->crtcs[tidss->num_crtcs++] = &tcrtc->crtc;

		tenc = tidss_encoder_create(tidss, pipes[i].enc_type,
					    1 << tcrtc->crtc.index);
		if (IS_ERR(tenc)) {
			dev_err(tidss->dev, "encoder create failed\n");
			return PTR_ERR(tenc);
		}

		ret = drm_bridge_attach(&tenc->encoder, pipes[i].bridge, NULL);
		if (ret) {
			dev_err(tidss->dev, "bridge attach failed: %d\n", ret);
			return ret;
		}
	}

	/* create overlay planes of the leftover planes */

	while (tidss->num_planes < max_planes) {
		struct tidss_plane *tplane;
		u32 hw_plane_id = tidss->num_planes;

		tplane = tidss_plane_create(tidss, hw_plane_id,
					    DRM_PLANE_TYPE_OVERLAY, crtc_mask,
					    fourccs, ARRAY_SIZE(fourccs));

		if (IS_ERR(tplane)) {
			dev_err(tidss->dev, "plane create failed\n");
			return PTR_ERR(tplane);
		}

		tidss->planes[tidss->num_planes++] = &tplane->plane;
	}

	return 0;
}

static void dispc7_remove(struct dispc_device *dispc)
{
	struct device *dev = dispc->dev;

	dev_dbg(dev, "%s\n", __func__);

	dispc->tidss->dispc_ops = NULL;
	dispc->tidss->dispc = NULL;
}

static const struct dispc_ops dispc7_ops = {
	.read_and_clear_irqstatus = dispc7_read_and_clear_irqstatus,
	.write_irqenable = dispc7_write_irqenable,

	.runtime_get = dispc7_runtime_get,
	.runtime_put = dispc7_runtime_put,

	.get_num_planes = dispc7_get_num_planes,
	.get_num_vps = dispc7_get_num_vps,

	.plane_name = dispc7_plane_name,
	.vp_name = dispc7_vp_name,

	.vp_prepare = dispc7_vp_prepare,
	.vp_enable = dispc7_vp_enable,
	.vp_disable = dispc7_vp_disable,
	.vp_unprepare = dispc7_vp_unprepare,
	.vp_go_busy = dispc7_vp_go_busy,
	.vp_go = dispc7_vp_go,

	.vp_setup = dispc7_vp_setup,
	.vp_check_mode = dispc7_vp_check_mode,
	.vp_check_config = dispc7_vp_check_config,

	.vp_gamma_size = dispc7_vp_gamma_size,
	.vp_set_gamma = dispc7_vp_set_gamma,

	.plane_enable = dispc7_plane_enable,
	.plane_setup = dispc7_plane_setup,

	.vp_set_clk_rate = dispc7_vp_set_clk_rate,
	.vp_enable_clk = dispc7_vp_enable_clk,
	.vp_disable_clk = dispc7_vp_disable_clk,

	.runtime_suspend = dispc7_runtime_suspend,
	.runtime_resume = dispc7_runtime_resume,

	.remove = dispc7_remove,

	.modeset_init = dispc7_modeset_init,
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

int dispc7_init(struct tidss_device *tidss)
{
	struct device *dev = tidss->dev;
	struct platform_device *pdev = tidss->pdev;
	struct dispc_device *dispc;
	const struct dispc7_features *feat;
	int r = 0;
	uint i;

	dev_info(dev, "%s\n", __func__);

	feat = of_match_device(dispc7_of_table, dev)->data;

	dispc = devm_kzalloc(dev, sizeof(*dispc), GFP_KERNEL);
	if (!dispc)
		return -ENOMEM;

	dispc->tidss = tidss;
	dispc->dev = dev;
	dispc->feat = feat;

	r = dispc7_iomap_resource(pdev, "common", &dispc->base_common);
	if (r)
		return r;

	for (i = 0; i < dispc->feat->num_planes; i++) {
		r = dispc7_iomap_resource(pdev, dispc->feat->vid_name[i],
					  &dispc->base_vid[i]);
		dev_dbg(dev, "%s: %u %s %d\n", __func__,
			i, dispc->feat->vid_name[i], r);
		if (r)
			return r;
	}

	for (i = 0; i < dispc->feat->num_vps; i++) {
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
	}

	dispc->syscon = syscon_regmap_lookup_by_phandle(dev->of_node, "syscon");
	if (IS_ERR(dispc->syscon)) {
		dev_err(dev, "%s: syscon_regmap_lookup_by_phandle failed %ld\n",
			__func__, PTR_ERR(dispc->syscon));
		return PTR_ERR(dispc->syscon);
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
