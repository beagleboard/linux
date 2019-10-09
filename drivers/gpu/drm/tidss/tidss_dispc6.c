// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
 */

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/delay.h>
#include <linux/export.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include <drm/drm_fourcc.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>

#include "tidss_crtc.h"
#include "tidss_drv.h"
#include "tidss_encoder.h"
#include "tidss_plane.h"

#include "tidss_dispc6.h"

static const struct {
	u32 fmt;
	u32 port_width;
} dispc6_bus_formats[] = {
	{ MEDIA_BUS_FMT_RGB444_1X12, 12 },
	{ MEDIA_BUS_FMT_RGB565_1X16, 16 },
	{ MEDIA_BUS_FMT_RGB666_1X18, 18 },
	{ MEDIA_BUS_FMT_RGB888_1X24, 24 },
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


#define REG_GET(dispc, idx, start, end)	\
	FLD_GET(dispc6_read(dispc, idx), start, end)

#define REG_FLD_MOD(dispc, idx, val, start, end) \
	dispc6_write(dispc, idx, FLD_MOD(dispc6_read(dispc, idx), val, start, end))

#define VID_REG_GET(dispc, plane, idx, start, end) \
	FLD_GET(dispc6_vid_read(dispc, plane, idx), start, end)

#define VID_REG_FLD_MOD(dispc, plane, idx, val, start, end) \
	dispc6_vid_write(dispc, plane, idx, FLD_MOD(dispc6_vid_read(dispc, plane, idx), val, start, end))


#define VP_REG_GET(dispc, vp, idx, start, end) \
	FLD_GET(dispc6_vp_read(dispc, vp, idx), start, end)

#define VP_REG_FLD_MOD(dispc, vp, idx, val, start, end)	\
	dispc6_vp_write(dispc, vp, idx, FLD_MOD(dispc6_vp_read(dispc, vp, idx), val, start, end))

#define DISPC6_GAMMA_TABLE_SIZE 256

struct dispc_features {
	/* XXX should these come from the .dts? Min pclk is not feature of DSS IP */
	unsigned long min_pclk;
	unsigned long max_pclk;
};

/* Note: 9MHz is a special allowed case, and is handled separately in the code */
static const struct dispc_features k2g_dispc_feats = {
	.min_pclk = 43750000,
	.max_pclk = 150000000,
};

static const struct of_device_id dispc6_of_match[] = {
	{ .compatible = "ti,k2g-dss", .data = &k2g_dispc_feats, },
	{},
};

struct dispc_device {
	struct device *dev;

	void __iomem *base_cfg;
	void __iomem *base_common;
	void __iomem *base_vid1;
	void __iomem *base_ovr1;
	void __iomem *base_vp1;

	const struct dispc_features *feat;

	struct clk *fclk;
	struct clk *vp_clk;

	u32 memory_bandwidth_limit;

	bool is_enabled;

	u32 gamma_table[DISPC6_GAMMA_TABLE_SIZE];

	struct tidss_device *tidss;
};

static void dispc6_write(struct dispc_device *dispc, u16 reg, u32 val)
{
	iowrite32(val, dispc->base_common + reg);
}

static u32 dispc6_read(struct dispc_device *dispc, u16 reg)
{
	return ioread32(dispc->base_common + reg);
}

static void dispc6_vid_write(struct dispc_device *dispc,
			     u32 hw_plane, u16 reg, u32 val)
{
	void __iomem *base = dispc->base_vid1;

	iowrite32(val, base + reg);
}

static u32 dispc6_vid_read(struct dispc_device *dispc,
			   u32 hw_plane, u16 reg)
{
	void __iomem *base = dispc->base_vid1;

	return ioread32(base + reg);
}

static void dispc6_ovr_write(struct dispc_device *dispc,
			     u32 hw_videoport, u16 reg, u32 val)
{
	void __iomem *base = dispc->base_ovr1;

	iowrite32(val, base + reg);
}

__maybe_unused
static u32 dispc6_ovr_read(struct dispc_device *dispc,
			   u32 hw_videoport, u16 reg)
{
	void __iomem *base = dispc->base_ovr1;

	return ioread32(base + reg);
}

static void dispc6_vp_write(struct dispc_device *dispc,
			    u32 hw_videoport, u16 reg, u32 val)
{
	void __iomem *base = dispc->base_vp1;

	iowrite32(val, base + reg);
}

static u32 dispc6_vp_read(struct dispc_device *dispc,
			  u32 hw_videoport, u16 reg)
{
	void __iomem *base = dispc->base_vp1;

	return ioread32(base + reg);
}

static int dispc6_runtime_get(struct dispc_device *dispc)
{
	int r;

	dev_dbg(dispc->dev, "dispc_runtime_get\n");

	r = pm_runtime_get_sync(dispc->dev);
	WARN_ON(r < 0);
	return r < 0 ? r : 0;
}

static void dispc6_runtime_put(struct dispc_device *dispc)
{
	int r;

	dev_dbg(dispc->dev, "dispc_runtime_put\n");

	r = pm_runtime_put_sync(dispc->dev);
	WARN_ON(r < 0);
}

static u64 dispc6_vp_irq_from_raw(u32 stat)
{
	u32 vp = 0;
	u64 vp_stat = 0;

	if (stat & BIT(0))
		vp_stat |= DSS_IRQ_VP_FRAME_DONE(vp);
	if (stat & BIT(1))
		vp_stat |= DSS_IRQ_VP_VSYNC_EVEN(vp);
	if (stat & BIT(2))
		vp_stat |= DSS_IRQ_VP_VSYNC_ODD(vp);
	if (stat & BIT(4))
		vp_stat |= DSS_IRQ_VP_SYNC_LOST(vp);

	return vp_stat;
}

static u32 dispc6_vp_irq_to_raw(u64 vpstat)
{
	u32 vp = 0;
	u32 stat = 0;

	if (vpstat & DSS_IRQ_VP_FRAME_DONE(vp))
		stat |= BIT(0);
	if (vpstat & DSS_IRQ_VP_VSYNC_EVEN(vp))
		stat |= BIT(1);
	if (vpstat & DSS_IRQ_VP_VSYNC_ODD(vp))
		stat |= BIT(2);
	if (vpstat & DSS_IRQ_VP_SYNC_LOST(vp))
		stat |= BIT(4);

	return stat;
}

static u64 dispc6_vid_irq_from_raw(u32 stat)
{
	u32 plane = 0;
	u64 vid_stat = 0;

	if (stat & BIT(0))
		vid_stat |= DSS_IRQ_PLANE_FIFO_UNDERFLOW(plane);

	return vid_stat;
}

static u32 dispc6_vid_irq_to_raw(u64 vidstat)
{
	u32 plane = 0;
	u32 stat = 0;

	if (vidstat & DSS_IRQ_PLANE_FIFO_UNDERFLOW(plane))
		stat |= BIT(0);

	return stat;
}


static u64 dispc6_vp_read_irqstatus(struct dispc_device *dispc,
				    u32 hw_videoport)
{
	u32 stat = dispc6_vp_read(dispc, hw_videoport, DISPC_VP_IRQSTATUS);

	return dispc6_vp_irq_from_raw(stat);
}

static void dispc6_vp_write_irqstatus(struct dispc_device *dispc,
				      u32 hw_videoport,
				      u64 vpstat)
{
	u32 stat = dispc6_vp_irq_to_raw(vpstat);

	dispc6_vp_write(dispc, hw_videoport, DISPC_VP_IRQSTATUS, stat);
}

static u64 dispc6_vid_read_irqstatus(struct dispc_device *dispc,
				     u32 hw_plane)
{
	u32 stat = dispc6_vid_read(dispc, hw_plane, DISPC_VID_IRQSTATUS);

	return dispc6_vid_irq_from_raw(stat);
}

static void dispc6_vid_write_irqstatus(struct dispc_device *dispc,
				       u32 hw_plane,
				       u64 vidstat)
{
	u32 stat = dispc6_vid_irq_to_raw(vidstat);

	dispc6_vid_write(dispc, hw_plane, DISPC_VID_IRQSTATUS, stat);
}


static u64 dispc6_vp_read_irqenable(struct dispc_device *dispc,
				    u32 hw_videoport)
{
	u32 stat = dispc6_vp_read(dispc, hw_videoport, DISPC_VP_IRQENABLE);

	return dispc6_vp_irq_from_raw(stat);
}

static void dispc6_vp_write_irqenable(struct dispc_device *dispc,
				      u32 hw_videoport,
				      u64 vpstat)
{
	u32 stat = dispc6_vp_irq_to_raw(vpstat);

	dispc6_vp_write(dispc, hw_videoport, DISPC_VP_IRQENABLE, stat);
}

static u64 dispc6_vid_read_irqenable(struct dispc_device *dispc,
				     u32 hw_plane)
{
	u32 stat = dispc6_vid_read(dispc, hw_plane, DISPC_VID_IRQENABLE);

	return dispc6_vid_irq_from_raw(stat);
}

static void dispc6_vid_write_irqenable(struct dispc_device *dispc,
				       u32 hw_plane,
				       u64 vidstat)
{
	u32 stat = dispc6_vid_irq_to_raw(vidstat);

	dispc6_vid_write(dispc, hw_plane, DISPC_VID_IRQENABLE, stat);
}


static void dispc6_clear_irqstatus(struct dispc_device *dispc, u64 mask)
{
	dispc6_vp_write_irqstatus(dispc, 0, mask);
	dispc6_vid_write_irqstatus(dispc, 0, mask);
}

static u64 dispc6_read_and_clear_irqstatus(struct dispc_device *dispc)
{
	u64 stat = 0;

	/* always clear the top level irqstatus */
	dispc6_write(dispc, DISPC_IRQSTATUS,
		     dispc6_read(dispc, DISPC_IRQSTATUS));

	stat |= dispc6_vp_read_irqstatus(dispc, 0);
	stat |= dispc6_vid_read_irqstatus(dispc, 0);

	dispc6_clear_irqstatus(dispc, stat);

	return stat;
}

static u64 dispc6_read_irqenable(struct dispc_device *dispc)
{
	u64 stat = 0;

	stat |= dispc6_vp_read_irqenable(dispc, 0);
	stat |= dispc6_vid_read_irqenable(dispc, 0);

	return stat;
}

static void dispc6_write_irqenable(struct dispc_device *dispc, u64 mask)
{
	u64 old_mask = dispc6_read_irqenable(dispc);

	/* clear the irqstatus for newly enabled irqs */
	dispc6_clear_irqstatus(dispc, (mask ^ old_mask) & mask);

	dispc6_vp_write_irqenable(dispc, 0, mask);
	dispc6_vid_write_irqenable(dispc, 0, mask);

	dispc6_write(dispc, DISPC_IRQENABLE_SET, (1 << 0) | (1 << 7));

	/* flush posted write */
	dispc6_read_irqenable(dispc);
}

static void dispc6_set_num_datalines(struct dispc_device *dispc,
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
		break;
	}

	VP_REG_FLD_MOD(dispc, hw_videoport, DISPC_VP_CONTROL, v, 10, 8);
}

static void dispc6_vp_enable(struct dispc_device *dispc, u32 hw_videoport,
			     const struct drm_crtc_state *state)
{
	const struct drm_display_mode *mode = &state->adjusted_mode;
	const struct tidss_crtc_state *tstate = to_tidss_crtc_state(state);
	bool align, onoff, rf, ieo, ipc, ihs, ivs;
	unsigned int i;
	u32 port_width;
	u32 hsw, hfp, hbp, vsw, vfp, vbp;

	for (i = 0; i < ARRAY_SIZE(dispc6_bus_formats); ++i) {
		if (dispc6_bus_formats[i].fmt != tstate->bus_format)
			continue;

		port_width = dispc6_bus_formats[i].port_width;
		break;
	}

	if (WARN_ON(i == ARRAY_SIZE(dispc6_bus_formats)))
		return;

	dispc6_set_num_datalines(dispc, hw_videoport, port_width);

	hfp = mode->hsync_start - mode->hdisplay;
	hsw = mode->hsync_end - mode->hsync_start;
	hbp = mode->htotal - mode->hsync_end;

	vfp = mode->vsync_start - mode->vdisplay;
	vsw = mode->vsync_end - mode->vsync_start;
	vbp = mode->vtotal - mode->vsync_end;

	dispc6_vp_write(dispc, hw_videoport, DISPC_VP_TIMING_H,
			FLD_VAL(hsw - 1, 7, 0) |
			FLD_VAL(hfp - 1, 19, 8) |
			FLD_VAL(hbp - 1, 31, 20));

	dispc6_vp_write(dispc, hw_videoport, DISPC_VP_TIMING_V,
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

	if (tstate->bus_flags & DRM_BUS_FLAG_DE_LOW)
		ieo = true;
	else
		ieo = false;

	if (tstate->bus_flags & DRM_BUS_FLAG_PIXDATA_NEGEDGE)
		ipc = true;
	else
		ipc = false;

	/* always use the 'rf' setting */
	onoff = true;

	if (tstate->bus_flags & DRM_BUS_FLAG_SYNC_NEGEDGE)
		rf = false;
	else
		rf = true;

	/* always use aligned syncs */
	align = true;

	dispc6_vp_write(dispc, hw_videoport, DISPC_VP_POL_FREQ,
			FLD_VAL(align, 18, 18) |
			FLD_VAL(onoff, 17, 17) |
			FLD_VAL(rf, 16, 16) |
			FLD_VAL(ieo, 15, 15) |
			FLD_VAL(ipc, 14, 14) |
			FLD_VAL(ihs, 13, 13) |
			FLD_VAL(ivs, 12, 12));

	dispc6_vp_write(dispc, hw_videoport, DISPC_VP_SIZE_SCREEN,
			FLD_VAL(mode->hdisplay - 1, 11, 0) |
			FLD_VAL(mode->vdisplay - 1, 27, 16));

	VP_REG_FLD_MOD(dispc, hw_videoport, DISPC_VP_CONTROL, 1, 0, 0);
}

static void dispc6_vp_disable(struct dispc_device *dispc, u32 hw_videoport)
{
	VP_REG_FLD_MOD(dispc, hw_videoport, DISPC_VP_CONTROL, 0, 0, 0);
}

static bool dispc6_vp_go_busy(struct dispc_device *dispc,
			      u32 hw_videoport)
{
	return VP_REG_GET(dispc, hw_videoport, DISPC_VP_CONTROL, 5, 5);
}

static void dispc6_vp_go(struct dispc_device *dispc,
			 u32 hw_videoport)
{
	VP_REG_FLD_MOD(dispc, hw_videoport, DISPC_VP_CONTROL, 1, 5, 5);
}

static u16 c8_to_c12(u8 c8)
{
	u16 c12;

	c12 = c8 << 4;

	/* Replication logic: Copy c8 4 MSB to 4 LSB for full scale c12 */
	c12 = c8 >> 4;

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

static void dispc6_vp_set_default_color(struct dispc_device *dispc,
					u32 hw_videoport, u32 default_color)
{
	u64 v;

	v = argb8888_to_argb12121212(default_color);

	dispc6_ovr_write(dispc, 0, DISPC_OVR_DEFAULT_COLOR, v & 0xffffffff);
	dispc6_ovr_write(dispc, 0, DISPC_OVR_DEFAULT_COLOR2,
			 (v >> 32) & 0xffff);
}

static enum drm_mode_status dispc6_vp_mode_valid(struct dispc_device *dispc,
						 u32 hw_videoport,
						 const struct drm_display_mode *mode)
{
	u32 hsw, hfp, hbp, vsw, vfp, vbp;

	/* special case for 9MHz */
	if (mode->clock * 1000 < dispc->feat->min_pclk && mode->clock != 9000)
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
	    vfp > 4095 || vbp > 4095)
		return MODE_BAD_VVALUE;

	if (dispc->memory_bandwidth_limit) {
		const unsigned int bpp = 4;
		u64 bandwidth;

		bandwidth = 1000 * mode->clock;
		bandwidth = bandwidth * mode->hdisplay * mode->vdisplay * bpp;
		bandwidth = div_u64(bandwidth, mode->htotal * mode->vtotal);

		if (dispc->memory_bandwidth_limit < bandwidth)
			return MODE_BAD;
	}

	return MODE_OK;
}

static int dispc6_vp_check(struct dispc_device *dispc, u32 hw_videoport,
			   const struct drm_crtc_state *state)
{
	const struct drm_display_mode *mode = &state->adjusted_mode;
	const struct tidss_crtc_state *tstate = to_tidss_crtc_state(state);
	enum drm_mode_status ok;
	unsigned int i;

	ok = dispc6_vp_mode_valid(dispc, hw_videoport, mode);
	if (ok != MODE_OK) {
		dev_dbg(dispc->dev, "%s: bad mode: %ux%u pclk %u kHz\n",
			__func__, mode->hdisplay, mode->vdisplay, mode->clock);
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(dispc6_bus_formats); ++i) {
		if (dispc6_bus_formats[i].fmt == tstate->bus_format)
			break;
	}

	if (i == ARRAY_SIZE(dispc6_bus_formats)) {
		dev_dbg(dispc->dev, "%s: Unsupported bus format: %u\n",
			__func__, tstate->bus_format);
		return -EINVAL;
	}

	return 0;
}

static int dispc6_vp_enable_clk(struct dispc_device *dispc, u32 hw_videoport)
{
	int ret = clk_prepare_enable(dispc->vp_clk);

	if (ret)
		dev_err(dispc->dev, "%s: enabling clk failed: %d\n", __func__,
			ret);

	return ret;
}

static void dispc6_vp_disable_clk(struct dispc_device *dispc,
				  u32 hw_videoport)
{
	clk_disable_unprepare(dispc->vp_clk);
}

static int dispc6_vp_set_clk_rate(struct dispc_device *dispc,
				  u32 hw_videoport, unsigned long rate)
{
	int r;
	unsigned long new_rate;

	r = clk_set_rate(dispc->vp_clk, rate);
	if (r) {
		dev_err(dispc->dev, "Failed to set vp clk rate to %lu\n",
			rate);
		return r;
	}

	new_rate = clk_get_rate(dispc->vp_clk);

	if (rate != new_rate)
		dev_warn(dispc->dev,
			 "Failed to get exact pix clock %lu != %lu\n",
			 rate, new_rate);

	dev_dbg(dispc->dev, "New VP rate %lu Hz (requested %lu Hz)\n",
		clk_get_rate(dispc->vp_clk), rate);

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

static void dispc6_vid_write_color_conv_coefs(struct dispc_device *dispc,
					      u32 hw_plane,
					      const struct color_conv_coef *ct)
{
#define CVAL(x, y) (FLD_VAL(x, 26, 16) | FLD_VAL(y, 10, 0))

	dispc6_vid_write(dispc, hw_plane,
			 DISPC_VID_CONV_COEF(0), CVAL(ct->rcr, ct->ry));
	dispc6_vid_write(dispc, hw_plane,
			 DISPC_VID_CONV_COEF(1), CVAL(ct->gy,  ct->rcb));
	dispc6_vid_write(dispc, hw_plane,
			 DISPC_VID_CONV_COEF(2), CVAL(ct->gcb, ct->gcr));
	dispc6_vid_write(dispc, hw_plane,
			 DISPC_VID_CONV_COEF(3), CVAL(ct->bcr, ct->by));
	dispc6_vid_write(dispc, hw_plane,
			 DISPC_VID_CONV_COEF(4), CVAL(0, ct->bcb));

	dispc6_vid_write(dispc, hw_plane, DISPC_VID_CONV_COEF(5),
			 FLD_VAL(ct->roffset, 15, 3) |
			 FLD_VAL(ct->goffset, 31, 19));
	dispc6_vid_write(dispc, hw_plane, DISPC_VID_CONV_COEF(6),
			 FLD_VAL(ct->boffset, 15, 3));

	VID_REG_FLD_MOD(dispc, hw_plane, DISPC_VID_ATTRIBUTES,
			ct->full_range, 11, 11);

#undef CVAL
}

static void dispc6_vid_csc_setup(struct dispc_device *dispc)
{
	/* YUV -> RGB, ITU-R BT.601, full range */
	const struct color_conv_coef coefs_yuv2rgb_bt601_full = {
		256,   0,  358,
		256, -88, -182,
		256, 452,    0,
		0, -2048, -2048,
		true,
	};

	dispc6_vid_write_color_conv_coefs(dispc, 0, &coefs_yuv2rgb_bt601_full);
}

static void dispc6_vid_csc_enable(struct dispc_device *dispc,
				  u32 hw_plane, bool enable)
{
	VID_REG_FLD_MOD(dispc, hw_plane, DISPC_VID_ATTRIBUTES, !!enable, 9, 9);
}

/* SCALER */

static u32 dispc6_calc_fir_inc(u32 in, u32 out)
{
	return (u32)div_u64(0x200000ull * in, out);
}

struct dispc6_vid_fir_coefs {
	s16 c2[16];
	s16 c1[16];
	u16 c0[9];
};

static const struct dispc6_vid_fir_coefs dispc6_fir_coefs_null = {
	.c2 = {	0 },
	.c1 = { 0 },
	.c0 = { 512, 512, 512, 512, 512, 512, 512, 512, 256,  },
};

enum dispc6_vid_fir_coef_set {
	DISPC6_VID_FIR_COEF_HORIZ,
	DISPC6_VID_FIR_COEF_HORIZ_UV,
	DISPC6_VID_FIR_COEF_VERT,
	DISPC6_VID_FIR_COEF_VERT_UV,
};

static void dispc6_vid_write_fir_coefs(struct dispc_device *dispc,
				       u32 hw_plane,
				       enum dispc6_vid_fir_coef_set coef_set,
				       const struct dispc6_vid_fir_coefs *coefs)
{
	static const u16 c0_regs[] = {
		[DISPC6_VID_FIR_COEF_HORIZ] = DISPC_VID_FIR_COEFS_H0,
		[DISPC6_VID_FIR_COEF_HORIZ_UV] = DISPC_VID_FIR_COEFS_H0_C,
		[DISPC6_VID_FIR_COEF_VERT] = DISPC_VID_FIR_COEFS_V0,
		[DISPC6_VID_FIR_COEF_VERT_UV] = DISPC_VID_FIR_COEFS_V0_C,
	};

	static const u16 c12_regs[] = {
		[DISPC6_VID_FIR_COEF_HORIZ] = DISPC_VID_FIR_COEFS_H12,
		[DISPC6_VID_FIR_COEF_HORIZ_UV] = DISPC_VID_FIR_COEFS_H12_C,
		[DISPC6_VID_FIR_COEF_VERT] = DISPC_VID_FIR_COEFS_V12,
		[DISPC6_VID_FIR_COEF_VERT_UV] = DISPC_VID_FIR_COEFS_V12_C,
	};

	const u16 c0_base = c0_regs[coef_set];
	const u16 c12_base = c12_regs[coef_set];
	int phase;

	for (phase = 0; phase <= 8; ++phase) {
		u16 reg = c0_base + phase * 4;
		u16 c0 = coefs->c0[phase];

		dispc6_vid_write(dispc, hw_plane, reg, c0);
	}

	for (phase = 0; phase <= 15; ++phase) {
		u16 reg = c12_base + phase * 4;
		s16 c1, c2;
		u32 c12;

		c1 = coefs->c1[phase];
		c2 = coefs->c2[phase];
		c12 = FLD_VAL(c1, 19, 10) | FLD_VAL(c2, 29, 20);

		dispc6_vid_write(dispc, hw_plane, reg, c12);
	}
}

static void dispc6_vid_write_scale_coefs(struct dispc_device *dispc,
					 u32 hw_plane)
{
	dispc6_vid_write_fir_coefs(dispc, hw_plane, DISPC6_VID_FIR_COEF_HORIZ,
				   &dispc6_fir_coefs_null);
	dispc6_vid_write_fir_coefs(dispc, hw_plane, DISPC6_VID_FIR_COEF_HORIZ_UV,
				   &dispc6_fir_coefs_null);
	dispc6_vid_write_fir_coefs(dispc, hw_plane, DISPC6_VID_FIR_COEF_VERT,
				   &dispc6_fir_coefs_null);
	dispc6_vid_write_fir_coefs(dispc, hw_plane, DISPC6_VID_FIR_COEF_VERT_UV,
				   &dispc6_fir_coefs_null);
}

static void dispc6_vid_set_scaling(struct dispc_device *dispc,
				   u32 hw_plane,
				   u32 orig_width, u32 orig_height,
				   u32 out_width, u32 out_height,
				   u32 fourcc)
{
	u32 in_w, in_h, in_w_uv, in_h_uv;
	u32 fir_hinc, fir_vinc, fir_hinc_uv, fir_vinc_uv;
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
	VID_REG_FLD_MOD(dispc, hw_plane, DISPC_VID_ATTRIBUTES, five_taps, 21, 21);

	/* FIR INC */

	fir_hinc = dispc6_calc_fir_inc(in_w, out_width);
	fir_vinc = dispc6_calc_fir_inc(in_h, out_height);
	fir_hinc_uv = dispc6_calc_fir_inc(in_w_uv, out_width);
	fir_vinc_uv = dispc6_calc_fir_inc(in_h_uv, out_height);

	dispc6_vid_write(dispc, hw_plane, DISPC_VID_FIRH, fir_hinc);
	dispc6_vid_write(dispc, hw_plane, DISPC_VID_FIRV, fir_vinc);
	dispc6_vid_write(dispc, hw_plane, DISPC_VID_FIRH2, fir_hinc_uv);
	dispc6_vid_write(dispc, hw_plane, DISPC_VID_FIRV2, fir_vinc_uv);

	dispc6_vid_write_scale_coefs(dispc, hw_plane);
}

/* OTHER */

static const struct {
	u32 fourcc;
	u8 dss_code;
} dispc6_color_formats[] = {
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

	{ DRM_FORMAT_XRGB8888, 0x27, },
	{ DRM_FORMAT_XBGR8888, 0x28, },
	{ DRM_FORMAT_RGBX8888, 0x29, },
	{ DRM_FORMAT_BGRX8888, 0x2a, },

	{ DRM_FORMAT_YUYV, 0x3e, },
	{ DRM_FORMAT_UYVY, 0x3f, },

	{ DRM_FORMAT_NV12, 0x3d, },
};

static bool dispc6_fourcc_is_yuv(u32 fourcc)
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

static void dispc6_plane_set_pixel_format(struct dispc_device *dispc,
					  u32 hw_plane, u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(dispc6_color_formats); ++i) {
		if (dispc6_color_formats[i].fourcc == fourcc) {
			VID_REG_FLD_MOD(dispc, hw_plane, DISPC_VID_ATTRIBUTES,
					dispc6_color_formats[i].dss_code,
					6, 1);
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
const struct tidss_plane_feat *dispc6_plane_feat(struct dispc_device *dispc,
						 u32 hw_plane)
{
	static const struct tidss_plane_feat pfeat = {
		.color = {
			.encodings = BIT(DRM_COLOR_YCBCR_BT601),
			.ranges = BIT(DRM_COLOR_YCBCR_FULL_RANGE),
			.default_encoding = DRM_COLOR_YCBCR_BT601,
			.default_range = DRM_COLOR_YCBCR_FULL_RANGE,
		},
		.blend = {
			.global_alpha = false,
		},
	};

	return &pfeat;
}

static int dispc6_plane_check(struct dispc_device *dispc, u32 hw_plane,
			      const struct drm_plane_state *state,
			      u32 hw_videoport)
{
	return 0; /* XXX: Dummy check function to be implemented later */
}

static int dispc6_plane_setup(struct dispc_device *dispc, u32 hw_plane,
			      const struct drm_plane_state *state,
			      u32 hw_videoport)
{
	u32 fourcc = state->fb->format->format;
	dma_addr_t paddr = dispc7_plane_state_paddr(state);
	u16 cpp = state->fb->format->cpp[0];
	u32 fb_width = state->fb->pitches[0] / cpp;
	u32 src_w = state->src_w >> 16;
	u32 src_h = state->src_h >> 16;

	dispc6_plane_set_pixel_format(dispc, hw_plane, fourcc);

	dispc6_vid_write(dispc, hw_plane, DISPC_VID_BA_0, paddr);
	dispc6_vid_write(dispc, hw_plane, DISPC_VID_BA_1, paddr);

	if (state->fb->format->num_planes == 2) {
		dma_addr_t p_uv_addr = dispc7_plane_state_p_uv_addr(state);

		dispc6_vid_write(dispc, hw_plane, DISPC_VID_BA_UV_0, p_uv_addr);
		dispc6_vid_write(dispc, hw_plane, DISPC_VID_BA_UV_1, p_uv_addr);
	}

	dispc6_vid_write(dispc, hw_plane, DISPC_VID_PICTURE_SIZE,
			 (src_w - 1) | ((src_h - 1) << 16));

	dispc6_vid_write(dispc, hw_plane, DISPC_VID_PIXEL_INC,
			 pixinc(1, cpp));
	dispc6_vid_write(dispc, hw_plane, DISPC_VID_ROW_INC,
			 pixinc(1 + fb_width - src_w, cpp));

	dispc6_vid_write(dispc, hw_plane, DISPC_VID_POSITION,
			 state->crtc_x | (state->crtc_y << 16));

	dispc6_vid_write(dispc, hw_plane, DISPC_VID_SIZE,
			 (state->crtc_w - 1) | ((state->crtc_h - 1) << 16));

	dispc6_vid_set_scaling(dispc, hw_plane, src_w, src_h,
			       state->crtc_w, state->crtc_h,
			       fourcc);

	/* enable YUV->RGB color conversion */
	if (dispc6_fourcc_is_yuv(fourcc))
		dispc6_vid_csc_enable(dispc, hw_plane, true);
	else
		dispc6_vid_csc_enable(dispc, hw_plane, false);

	/* hw_videoport */
	VID_REG_FLD_MOD(dispc, hw_plane, DISPC_VID_ATTRIBUTES, 0, 16, 14);

	return 0;
}

static int dispc6_plane_enable(struct dispc_device *dispc,
			       u32 hw_plane, bool enable)
{
	VID_REG_FLD_MOD(dispc, hw_plane, DISPC_VID_ATTRIBUTES, !!enable, 0, 0);
	return 0;
}

static bool dispc6_has_writeback(struct dispc_device *dispc)
{
	return false;
}

static u32 dispc6_vid_get_fifo_size(struct dispc_device *dispc,
				    u32 hw_plane)
{
	const u32 unit_size = 16;	/* 128-bits */

	return VID_REG_GET(dispc, hw_plane, DISPC_VID_BUF_SIZE_STATUS, 15, 0) *
	       unit_size;
}

static void dispc6_vid_set_mflag_threshold(struct dispc_device *dispc,
					   u32 hw_plane,
					   u32 low, u32 high)
{
	dispc6_vid_write(dispc, hw_plane, DISPC_VID_MFLAG_THRESHOLD,
			 FLD_VAL(high, 31, 16) | FLD_VAL(low, 15, 0));
}

static void dispc6_mflag_setup(struct dispc_device *dispc)
{
	u32 hw_plane = 0;
	const u32 unit_size = 16;	/* 128-bits */
	u32 size = dispc6_vid_get_fifo_size(dispc, hw_plane);
	u32 low, high;

	/* MFLAG_CTRL */
	REG_FLD_MOD(dispc, DISPC_GLOBAL_MFLAG_ATTRIBUTE, 1, 1, 0);
	/* MFLAG_START */
	REG_FLD_MOD(dispc, DISPC_GLOBAL_MFLAG_ATTRIBUTE, 0, 2, 2);

	/*
	 * Simulation team suggests below thesholds:
	 * HT = fifosize * 5 / 8;
	 * LT = fifosize * 4 / 8;
	 */

	low = size * 4 / 8 / unit_size;
	high = size * 5 / 8 / unit_size;

	dispc6_vid_set_mflag_threshold(dispc, hw_plane, low, high);
}

static void dispc6_initial_config(struct dispc_device *dispc)
{
	dispc6_vid_csc_setup(dispc);
	dispc6_mflag_setup(dispc);

	/* Enable the gamma Shadow bit-field */
	VP_REG_FLD_MOD(dispc, 0, DISPC_VP_CONFIG, 1, 2, 2);
}

static int dispc6_init_features(struct dispc_device *dispc)
{
	const struct of_device_id *match;

	match = of_match_node(dispc6_of_match, dispc->dev->of_node);
	if (!match) {
		dev_err(dispc->dev, "Unsupported DISPC version\n");
		return -ENODEV;
	}

	dispc->feat = match->data;

	return 0;
}

static int dispc6_get_num_planes(struct dispc_device *dispc)
{
	return 1;
}

static int dispc6_get_num_vps(struct dispc_device *dispc)
{
	return 1;
}

static const struct tidss_vp_feat *dispc6_vp_feat(struct dispc_device *dispc,
						  u32 hw_videoport)
{
	static const struct tidss_vp_feat vp_feat = {
		.color = {
			.gamma_size = DISPC6_GAMMA_TABLE_SIZE,
			.gamma_type = TIDSS_GAMMA_8BIT,
			.has_ctm = false, /* Driver implementation missing */
		},
	};

	return &vp_feat;
}

static void dispc6_vp_write_gamma_table(struct dispc_device *dispc,
					u32 hw_videoport)
{
	u32 *table = dispc->gamma_table;
	unsigned int hwlen = ARRAY_SIZE(dispc->gamma_table);
	unsigned int i;

	dev_dbg(dispc->dev, "%s: hw_videoport %d\n", __func__, hw_videoport);

	for (i = 0; i < hwlen; ++i) {
		u32 v = table[i];

		v |= i << 24;

		dispc6_vp_write(dispc, hw_videoport, DISPC_VP_GAMMA_TABLE, v);
	}
}

static void dispc6_restore_gamma_tables(struct dispc_device *dispc)
{
	dev_dbg(dispc->dev, "%s()\n", __func__);

	dispc6_vp_write_gamma_table(dispc, 0);
}

static const struct drm_color_lut dispc6_vp_gamma_default_lut[] = {
	{ .red = 0, .green = 0, .blue = 0, },
	{ .red = U16_MAX, .green = U16_MAX, .blue = U16_MAX, },
};

static void dispc6_vp_set_gamma(struct dispc_device *dispc,
				u32 hw_videoport,
				const struct drm_color_lut *lut,
				unsigned int length)
{
	u32 *table = dispc->gamma_table;
	unsigned int hwlen = ARRAY_SIZE(dispc->gamma_table);
	static const unsigned int hwbits = 8;
	unsigned int i;

	dev_dbg(dispc->dev, "%s: hw_videoport %d, lut len %u, hw len %u\n",
		__func__, hw_videoport, length, hwlen);

	if (lut == NULL || length < 2) {
		lut = dispc6_vp_gamma_default_lut;
		length = ARRAY_SIZE(dispc6_vp_gamma_default_lut);
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
		dispc6_vp_write_gamma_table(dispc, hw_videoport);
}

static void dispc6_vp_set_color_mgmt(struct dispc_device *dispc,
				     u32 hw_videoport,
				     const struct drm_crtc_state *state)
{
	struct drm_color_lut *lut = NULL;
	unsigned int length = 0;

	if (!state->color_mgmt_changed)
		return;

	if (state->gamma_lut) {
		lut = (struct drm_color_lut *) state->gamma_lut->data;
		length = state->gamma_lut->length / sizeof(*lut);
	}

	dispc6_vp_set_gamma(dispc, hw_videoport, lut, length);
}

static void dispc6_vp_setup(struct dispc_device *dispc, u32 hw_videoport,
			    const struct drm_crtc_state *state)
{
	dispc6_vp_set_default_color(dispc, hw_videoport, 0);
	dispc6_vp_set_color_mgmt(dispc, hw_videoport, state);
}

static int dispc6_init_gamma_tables(struct dispc_device *dispc)
{
	dispc6_vp_set_gamma(dispc, 0, NULL, 0);

	return 0;
}

static const char *dispc6_plane_name(struct dispc_device *dispc,
				     u32 hw_plane)
{
	return "vid1";
}

static const char *dispc6_vp_name(struct dispc_device *dispc,
				  u32 hw_videoport)
{
	return "vp1";
}

static int dispc6_runtime_suspend(struct dispc_device *dispc)
{
	struct device *dev = dispc->dev;

	dev_dbg(dev, "suspend\n");

	dispc->is_enabled = false;

	clk_disable_unprepare(dispc->fclk);

	return 0;
}

static int dispc6_runtime_resume(struct dispc_device *dispc)
{
	struct device *dev = dispc->dev;

	dev_dbg(dev, "resume\n");

	clk_prepare_enable(dispc->fclk);

	if (REG_GET(dispc, DISPC_SYSSTATUS, 0, 0) == 0)
		dev_warn(dev, "DISPC FUNC RESET not done!\n");
	if (REG_GET(dispc, DISPC_SYSSTATUS, 1, 1) == 0)
		dev_warn(dev, "DISPC VP RESET not done!\n");

	dispc6_initial_config(dispc);

	dispc6_restore_gamma_tables(dispc);

	dispc->is_enabled = true;

	return 0;
}

static int dispc6_modeset_init(struct dispc_device *dispc)
{
	struct tidss_device *tidss = dispc->tidss;
	struct device *dev = tidss->dev;
	const u32 hw_videoport = 0;
	const u32 crtc_mask = 1;
	const u32 hw_plane_id = 0;
	struct drm_panel *panel;
	struct drm_bridge *bridge;
	u32 enc_type;
	int ret;
	struct tidss_plane *tplane;
	struct tidss_crtc *tcrtc;
	struct drm_encoder *enc;
	u32 fourccs[ARRAY_SIZE(dispc6_color_formats)];
	unsigned int i;

	/* first find if there is a connected panel/bridge */

	ret = drm_of_find_panel_or_bridge(dev->of_node, hw_videoport, 0, &panel, &bridge);
	if (ret) {
		dev_dbg(dev, "no panel or bridge found\n");
		return ret;
	}

	if (panel) {
		dev_dbg(dev, "Setting up panel\n");

		enc_type = DRM_MODE_ENCODER_DPI;

		bridge = devm_drm_panel_bridge_add(dev, panel, DRM_MODE_CONNECTOR_DPI);
		if (IS_ERR(bridge)) {
			dev_err(dev, "failed to set up panel bridge\n");
			return PTR_ERR(bridge);
		}
	} else {
		enc_type = DRM_MODE_ENCODER_NONE;
	}

	/* then create a plane, a crtc and an encoder for the panel/bridge */

	for (i = 0; i < ARRAY_SIZE(dispc6_color_formats); ++i)
		fourccs[i] = dispc6_color_formats[i].fourcc;

	tplane = tidss_plane_create(tidss, hw_plane_id, DRM_PLANE_TYPE_PRIMARY,
				    crtc_mask, fourccs, ARRAY_SIZE(fourccs));
	if (IS_ERR(tplane)) {
		dev_err(tidss->dev, "plane create failed\n");
		return PTR_ERR(tplane);
	}

	tidss->planes[tidss->num_planes++] = &tplane->plane;

	tcrtc = tidss_crtc_create(tidss, hw_videoport, &tplane->plane);
	if (IS_ERR(tcrtc)) {
		dev_err(tidss->dev, "crtc create failed\n");
		return PTR_ERR(tcrtc);
	}

	tidss->crtcs[tidss->num_crtcs++] = &tcrtc->crtc;

	enc = tidss_encoder_create(tidss, enc_type, 1 << tcrtc->crtc.index);
	if (IS_ERR(enc)) {
		dev_err(tidss->dev, "encoder create failed\n");
		return PTR_ERR(enc);
	}

	ret = drm_bridge_attach(enc, bridge, NULL);
	if (ret) {
		dev_err(tidss->dev, "bridge attach failed: %d\n", ret);
		return ret;
	}

	return 0;
}

static int dispc6_get_irq(struct dispc_device *dispc)
{
	return platform_get_irq(to_platform_device(dispc->tidss->dev), 0);
}

static void dispc6_remove(struct dispc_device *dispc);

static const struct tidss_dispc_ops dispc6_ops = {
	.read_and_clear_irqstatus = dispc6_read_and_clear_irqstatus,
	.write_irqenable = dispc6_write_irqenable,

	.get_num_vps = dispc6_get_num_vps,
	.vp_name = dispc6_vp_name,
	.vp_feat = dispc6_vp_feat,
	.vp_enable = dispc6_vp_enable,
	.vp_disable = dispc6_vp_disable,
	.vp_go_busy = dispc6_vp_go_busy,
	.vp_go = dispc6_vp_go,
	.vp_mode_valid = dispc6_vp_mode_valid,
	.vp_check = dispc6_vp_check,
	.vp_setup = dispc6_vp_setup,

	.vp_set_clk_rate = dispc6_vp_set_clk_rate,
	.vp_enable_clk = dispc6_vp_enable_clk,
	.vp_disable_clk = dispc6_vp_disable_clk,

	.get_num_planes = dispc6_get_num_planes,
	.plane_name = dispc6_plane_name,
	.plane_feat = dispc6_plane_feat,
	.plane_enable = dispc6_plane_enable,
	.plane_check = dispc6_plane_check,
	.plane_setup = dispc6_plane_setup,

	.runtime_get = dispc6_runtime_get,
	.runtime_put = dispc6_runtime_put,

	.runtime_suspend = dispc6_runtime_suspend,
	.runtime_resume = dispc6_runtime_resume,

	.remove = dispc6_remove,

	.modeset_init = dispc6_modeset_init,

	.get_irq = dispc6_get_irq,

	.has_writeback = dispc6_has_writeback,
};

static int dispc6_iomap_resource(struct platform_device *pdev, const char *name,
				 void __iomem **base)
{
	struct resource *res;
	void __iomem *b;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	b = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(b)) {
		dev_err(&pdev->dev, "cannot ioremap resource '%s'\n", name);
		return PTR_ERR(b);
	}

	*base = b;

	return 0;
}

int dispc6_init(struct tidss_device *tidss)
{
	struct device *dev = tidss->dev;
	struct platform_device *pdev = to_platform_device(dev);
	struct dispc_device *dispc;
	int r;

	dev_dbg(dev, "%s\n", __func__);

	dispc = devm_kzalloc(dev, sizeof(*dispc), GFP_KERNEL);
	if (!dispc)
		return -ENOMEM;

	dispc->tidss = tidss;
	dispc->dev = dev;

	r = dispc6_init_features(dispc);
	if (r)
		goto err_free;

	r = dispc6_iomap_resource(pdev, "cfg", &dispc->base_cfg);
	if (r)
		goto err_free;

	r = dispc6_iomap_resource(pdev, "common", &dispc->base_common);
	if (r)
		goto err_free;

	r = dispc6_iomap_resource(pdev, "vid1", &dispc->base_vid1);
	if (r)
		goto err_free;

	r = dispc6_iomap_resource(pdev, "ovr1", &dispc->base_ovr1);
	if (r)
		goto err_free;

	r = dispc6_iomap_resource(pdev, "vp1", &dispc->base_vp1);
	if (r)
		goto err_free;

	dev_dbg(dev, "dispc6_bind: iores ok\n");

	dispc->fclk = devm_clk_get(dev, "fck");
	if (IS_ERR(dispc->fclk)) {
		dev_err(dev, "Failed to get fclk\n");
		r = PTR_ERR(dispc->fclk);
		goto err_free;
	}

	dispc->vp_clk = devm_clk_get(dev, "vp1");
	if (IS_ERR(dispc->vp_clk)) {
		dev_err(dev, "Failed to get vp1 clk\n");
		r = PTR_ERR(dispc->vp_clk);
		goto err_free;
	}

	of_property_read_u32(dispc->dev->of_node, "max-memory-bandwidth",
			     &dispc->memory_bandwidth_limit);

	r = dispc6_init_gamma_tables(dispc);
	if (r)
		goto err_free;

	tidss->dispc_ops = &dispc6_ops;
	tidss->dispc = dispc;

	dev_dbg(dev, "%s done\n", __func__);

	return 0;
err_free:
	dev_err(dev, "%s failed: %d\n", __func__, r);
	return r;
}

static void dispc6_remove(struct dispc_device *dispc)
{
	struct device *dev = dispc->dev;

	dev_dbg(dev, "dispc6_unbind\n");

	dispc->tidss->dispc_ops = NULL;
	dispc->tidss->dispc = NULL;

	dev_dbg(dev, "dispc6_unbind done\n");
}
