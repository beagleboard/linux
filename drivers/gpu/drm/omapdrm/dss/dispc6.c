/*
 * Copyright (C) 2016 Texas Instruments Ltd
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/delay.h>
#include <linux/export.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <drm/drm_fourcc.h>

#include "omapdss.h"
#include "dss6.h"
#include "dispc6.h"

#define REG_GET(idx, start, end) \
	FLD_GET(dispc6_read(idx), start, end)

#define REG_FLD_MOD(idx, val, start, end)				\
	dispc6_write(idx, FLD_MOD(dispc6_read(idx), val, start, end))

#define VID_REG_GET(plane, idx, start, end) \
	FLD_GET(dispc6_vid_read(plane, idx), start, end)

#define VID_REG_FLD_MOD(plane, idx, val, start, end)				\
	dispc6_vid_write(plane, idx, FLD_MOD(dispc6_vid_read(plane, idx), val, start, end))


#define VP_REG_GET(vp, idx, start, end) \
	FLD_GET(dispc6_vp_read(vp, idx), start, end)

#define VP_REG_FLD_MOD(vp, idx, val, start, end)				\
	dispc6_vp_write(vp, idx, FLD_MOD(dispc6_vp_read(vp, idx), val, start, end))

struct dispc_features {
	/* XXX should these come from the .dts? Min pclk is not feature of DSS IP */
	unsigned long min_pclk;
	unsigned long max_pclk;
};

static const struct dispc_features k2g_dispc_feats = {
	.min_pclk = 43750000,
	.max_pclk = 150000000,
};

static const struct of_device_id dispc6_of_match[];

static struct {
	struct platform_device *pdev;

	void __iomem *base_common;
	void __iomem *base_vid1;
	void __iomem *base_ovr1;
	void __iomem *base_vp1;

	int irq;
	irq_handler_t user_handler;
	void *user_data;

	const struct dispc_features *feat;

	struct clk *fclk;
	struct clk *vp_clk;

	bool is_enabled;

	bool ctx_valid;
	u32 ctx_vid1[0x400];

	u32 gamma_table[256];
} dispc;

static void dispc6_write(u16 reg, u32 val)
{
	iowrite32(val, dispc.base_common + reg);
}

static u32 dispc6_read(u16 reg)
{
	return ioread32(dispc.base_common + reg);
}

static void dispc6_vid_write(enum omap_plane plane, u16 reg, u32 val)
{
	void __iomem *base = dispc.base_vid1;
	iowrite32(val, base + reg);
}

static u32 dispc6_vid_read(enum omap_plane plane, u16 reg)
{
	void __iomem *base = dispc.base_vid1;
	return ioread32(base + reg);
}

static void dispc6_ovr_write(enum omap_channel channel, u16 reg, u32 val)
{
	void __iomem *base = dispc.base_ovr1;
	iowrite32(val, base + reg);
}
#if 0	/* not used at the moment */
static u32 dispc6_ovr_read(enum omap_channel channel, u16 reg)
{
	void __iomem *base = dispc.base_ovr1;
	return ioread32(base + reg);
}
#endif
static void dispc6_vp_write(enum omap_channel channel, u16 reg, u32 val)
{
	void __iomem *base = dispc.base_vp1;
	iowrite32(val, base + reg);
}

static u32 dispc6_vp_read(enum omap_channel channel, u16 reg)
{
	void __iomem *base = dispc.base_vp1;
	return ioread32(base + reg);
}

int dispc6_runtime_get(void)
{
	int r;

	dev_dbg(&dispc.pdev->dev, "dispc_runtime_get\n");

	r = pm_runtime_get_sync(&dispc.pdev->dev);
	WARN_ON(r < 0);
	return r < 0 ? r : 0;
}

void dispc6_runtime_put(void)
{
	int r;

	dev_dbg(&dispc.pdev->dev, "dispc_runtime_put\n");

	r = pm_runtime_put_sync(&dispc.pdev->dev);
	WARN_ON(r < 0);
}

static void dispc6_save_context(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dispc.ctx_vid1); ++i)
		dispc.ctx_vid1[i] = dispc6_vid_read(0, i);

	dispc.ctx_valid = true;
}

static void dispc6_restore_context(void)
{
	int i;

	if (!dispc.ctx_valid)
		return;

	for (i = 0; i < ARRAY_SIZE(dispc.ctx_vid1); ++i)
		dispc6_vid_write(0, i, dispc.ctx_vid1[i]);
}

static irqreturn_t dispc6_irq_handler(int irq, void *arg)
{
	u32 stat;

	if (!dispc.is_enabled)
		return IRQ_NONE;

	stat = dispc6_read(DISPC_IRQSTATUS);

	if (stat == 0)
		return IRQ_NONE;

	dispc6_write(DISPC_IRQSTATUS, stat);

	return dispc.user_handler(irq, dispc.user_data);
}

static int dispc6_request_irq(irq_handler_t handler, void *dev_id)
{
	int r;

	if (dispc.user_handler != NULL)
		return -EBUSY;

	dispc.user_handler = handler;
	dispc.user_data = dev_id;

	/* ensure the dispc6_irq_handler sees the values above */
	smp_wmb();

	r = devm_request_irq(&dispc.pdev->dev, dispc.irq, dispc6_irq_handler,
			     IRQF_SHARED, "DISPC", &dispc);
	if (r) {
		dispc.user_handler = NULL;
		dispc.user_data = NULL;
	}

	return r;
}

static void dispc6_free_irq(void *dev_id)
{
	dispc6_write(DISPC_IRQENABLE_CLR, 0xffffffff);

	devm_free_irq(&dispc.pdev->dev, dispc.irq, &dispc);

	dispc.user_handler = NULL;
	dispc.user_data = NULL;
}


enum omapdss_vp_irq {
	OMAP_DSS_VP_FRAMEDONE	= (1 << 0),
	OMAP_DSS_VP_VSYNC_EVEN	= (1 << 1),
	OMAP_DSS_VP_VSYNC_ODD	= (1 << 2),
	OMAP_DSS_VP_SYNC_LOST	= (1 << 3),
};

enum omapdss_vid_irq {
	OMAP_DSS_VID_UNDERFLOW	= (1 << 0),
};

static enum omapdss_vp_irq dispc6_vp_irq_from_raw(u32 stat)
{
	enum omapdss_vp_irq vp_stat = 0;

	if (stat & BIT(0))
		vp_stat |= OMAP_DSS_VP_FRAMEDONE;
	if (stat & BIT(1))
		vp_stat |= OMAP_DSS_VP_VSYNC_EVEN;
	if (stat & BIT(2))
		vp_stat |= OMAP_DSS_VP_VSYNC_ODD;
	if (stat & BIT(4))
		vp_stat |= OMAP_DSS_VP_SYNC_LOST;

	return vp_stat;
}

static u32 dispc6_vp_irq_to_raw(enum omapdss_vp_irq vpstat)
{
	u32 stat = 0;

	if (vpstat & OMAP_DSS_VP_FRAMEDONE)
		stat |= BIT(0);
	if (vpstat & OMAP_DSS_VP_VSYNC_EVEN)
		stat |= BIT(1);
	if (vpstat & OMAP_DSS_VP_VSYNC_ODD)
		stat |= BIT(2);
	if (vpstat & OMAP_DSS_VP_SYNC_LOST)
		stat |= BIT(4);

	return stat;
}

static enum omapdss_vid_irq dispc6_vid_irq_from_raw(u32 stat)
{
	enum omapdss_vid_irq vid_stat = 0;

	if (stat & BIT(0))
		vid_stat |= OMAP_DSS_VID_UNDERFLOW;

	return vid_stat;
}

static u32 dispc6_vid_irq_to_raw(enum omapdss_vid_irq vidstat)
{
	u32 stat = 0;

	if (vidstat & OMAP_DSS_VID_UNDERFLOW)
		stat |= BIT(0);

	return stat;
}


static enum omapdss_vp_irq dispc6_vp_read_irqstatus(enum omap_channel channel)
{
	u32 stat = dispc6_vp_read(channel, DISPC_VP_IRQSTATUS);
	return dispc6_vp_irq_from_raw(stat);
}

static void dispc6_vp_write_irqstatus(enum omap_channel channel,
				      enum omapdss_vp_irq vpstat)
{
	u32 stat = dispc6_vp_irq_to_raw(vpstat);
	dispc6_vp_write(channel, DISPC_VP_IRQSTATUS, stat);
}

static enum omapdss_vid_irq dispc6_vid_read_irqstatus(enum omap_plane plane)
{
	u32 stat = dispc6_vid_read(plane, DISPC_VID_IRQSTATUS);
	return dispc6_vid_irq_from_raw(stat);
}

static void dispc6_vid_write_irqstatus(enum omap_plane plane,
				       enum omapdss_vid_irq vidstat)
{
	u32 stat = dispc6_vid_irq_to_raw(vidstat);
	dispc6_vid_write(plane, DISPC_VID_IRQSTATUS, stat);
}


static enum omapdss_vp_irq dispc6_vp_read_irqenable(enum omap_channel channel)
{
	u32 stat = dispc6_vp_read(channel, DISPC_VP_IRQENABLE);
	return dispc6_vp_irq_from_raw(stat);
}

static void dispc6_vp_write_irqenable(enum omap_channel channel,
				      enum omapdss_vp_irq vpstat)
{
	u32 stat = dispc6_vp_irq_to_raw(vpstat);
	dispc6_vp_write(channel, DISPC_VP_IRQENABLE, stat);
}

static enum omapdss_vid_irq dispc6_vid_read_irqenable(enum omap_plane plane)
{
	u32 stat = dispc6_vid_read(plane, DISPC_VID_IRQENABLE);
	return dispc6_vid_irq_from_raw(stat);
}

static void dispc6_vid_write_irqenable(enum omap_plane plane,
				       enum omapdss_vid_irq vidstat)
{
	u32 stat = dispc6_vid_irq_to_raw(vidstat);
	dispc6_vid_write(plane, DISPC_VID_IRQENABLE, stat);
}







static u32 dispc6_irq_to_dispc2(enum omapdss_vp_irq vpstat,
				enum omapdss_vid_irq vidstat)
{
	u32 stat = 0;

	if (vpstat & OMAP_DSS_VP_FRAMEDONE)
		stat |= DISPC_IRQ_FRAMEDONE;
	if (vpstat & OMAP_DSS_VP_VSYNC_EVEN)
		stat |= DISPC_IRQ_VSYNC;
	if (vpstat & OMAP_DSS_VP_SYNC_LOST)
		stat |= DISPC_IRQ_SYNC_LOST;

	if (vidstat & OMAP_DSS_VID_UNDERFLOW)
		stat |= DISPC_IRQ_GFX_FIFO_UNDERFLOW;

	return stat;
}

static void dispc2_irq_to_dispc6(u32 stat,
				 enum omapdss_vp_irq *vpstat,
				 enum omapdss_vid_irq *vidstat)
{
	*vpstat = 0;

	if (stat & DISPC_IRQ_FRAMEDONE)
		*vpstat |= OMAP_DSS_VP_FRAMEDONE;
	if (stat & DISPC_IRQ_VSYNC)
		*vpstat |= OMAP_DSS_VP_VSYNC_EVEN;
	if (stat & DISPC_IRQ_SYNC_LOST)
		*vpstat |= OMAP_DSS_VP_SYNC_LOST;

	*vidstat = 0;

	if (stat & DISPC_IRQ_GFX_FIFO_UNDERFLOW)
		*vidstat |= OMAP_DSS_VID_UNDERFLOW;
}

static u32 dispc6_read_legacy_irqstatus(void)
{
	enum omapdss_vp_irq vpstat;
	enum omapdss_vid_irq vidstat;

	vpstat = dispc6_vp_read_irqstatus(0);
	vidstat = dispc6_vid_read_irqstatus(0);

	return dispc6_irq_to_dispc2(vpstat, vidstat);
}

static void dispc6_clear_legacy_irqstatus(u32 mask)
{
	enum omapdss_vp_irq vpstat;
	enum omapdss_vid_irq vidstat;

	dispc2_irq_to_dispc6(mask, &vpstat, &vidstat);

	dispc6_vp_write_irqstatus(0, vpstat);
	dispc6_vid_write_irqstatus(0, vidstat);
}

static u32 dispc6_read_legacy_irqenable(void)
{
	enum omapdss_vp_irq vpstat;
	enum omapdss_vid_irq vidstat;

	vpstat = dispc6_vp_read_irqenable(0);
	vidstat = dispc6_vid_read_irqenable(0);

	return dispc6_irq_to_dispc2(vpstat, vidstat);
}

static void dispc6_write_legacy_irqenable(u32 mask)
{
	enum omapdss_vp_irq vpstat;
	enum omapdss_vid_irq vidstat;

	u32 old_mask = dispc6_read_legacy_irqenable();

	/* clear the irqstatus for newly enabled irqs */
	dispc6_clear_legacy_irqstatus((mask ^ old_mask) & mask);

	dispc2_irq_to_dispc6(mask, &vpstat, &vidstat);

	dispc6_vp_write_irqenable(0, vpstat);
	dispc6_vid_write_irqenable(0, vidstat);


	dispc6_write(DISPC_IRQENABLE_SET, (1 << 0) | (1 << 7));
}




static u32 dispc6_mgr_get_vsync_irq(enum omap_channel channel)
{
	return DISPC_IRQ_VSYNC;
}

static u32 dispc6_mgr_get_framedone_irq(enum omap_channel channel)
{
	return DISPC_IRQ_FRAMEDONE;
}

static u32 dispc6_mgr_get_sync_lost_irq(enum omap_channel channel)
{
	return DISPC_IRQ_SYNC_LOST;
}


static bool dispc6_mgr_go_busy(enum omap_channel channel)
{
	return VP_REG_GET(channel, DISPC_VP_CONTROL, 5, 5);
}

static void dispc6_mgr_go(enum omap_channel channel)
{
	VP_REG_FLD_MOD(channel, DISPC_VP_CONTROL, 1, 5, 5);
}

static void dispc6_mgr_enable(enum omap_channel channel, bool enable)
{
	VP_REG_FLD_MOD(channel, DISPC_VP_CONTROL, !!enable, 0, 0);
}

static bool dispc6_mgr_is_enabled(enum omap_channel channel)
{
	return VP_REG_GET(channel, DISPC_VP_CONTROL, 0, 0);
}

static u16 c8_to_c12(u8 c8)
{
	u16 c16;

	c16 = c8 << 4;

	if (c8 & BIT(7))
		c16 |= 0xf;

	return c16;
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

static void dispc6_mgr_setup(enum omap_channel channel,
			     const struct omap_overlay_manager_info *info)
{
	u64 v;

	v = argb8888_to_argb12121212(info->default_color);

	dispc6_ovr_write(0, DISPC_OVR_DEFAULT_COLOR, v & 0xffffffff);
	dispc6_ovr_write(0, DISPC_OVR_DEFAULT_COLOR2, (v >> 32) & 0xffff);
}

static void dispc6_set_num_datalines(enum omap_channel channel, int num_lines)
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
		BUG();
	}

	VP_REG_FLD_MOD(channel, DISPC_VP_CONTROL, v, 10, 8);
}

static void dispc6_mgr_set_lcd_config(enum omap_channel channel,
				      const struct dss_lcd_mgr_config *config)
{
	dispc6_set_num_datalines(channel, config->video_port_width);
}

static bool dispc6_lcd_timings_ok(int hsw, int hfp, int hbp,
				  int vsw, int vfp, int vbp)
{
	if (hsw < 1 || hsw > 256 ||
	    hfp < 1 || hfp > 4096 ||
	    hbp < 1 || hbp > 4096 ||
	    vsw < 1 || vsw > 256 ||
	    vfp < 0 || vfp > 4095 ||
	    vbp < 0 || vbp > 4095)
		return false;
	return true;
}

bool dispc6_mgr_timings_ok(enum omap_channel channel,
			   const struct omap_video_timings *timings)
{
	if (timings->pixelclock < dispc.feat->min_pclk &&
		timings->pixelclock != 9000000)
		return false;

	if (timings->pixelclock > dispc.feat->max_pclk)
		return false;

	if (timings->x_res > 4096)
		return false;

	if (timings->y_res > 4096)
		return false;

	/* TODO: add interlace support */
	if (timings->interlace)
		return false;

	if (!dispc6_lcd_timings_ok(timings->hsw, timings->hfp, timings->hbp,
				   timings->vsw, timings->vfp, timings->vbp))
		return false;

	return true;
}

static void dispc6_mgr_set_timings(enum omap_channel channel,
				   const struct omap_video_timings *t)
{
	bool align, onoff, rf, ieo, ipc, ihs, ivs;

	dispc6_vp_write(channel, DISPC_VP_TIMING_H,
			FLD_VAL(t->hsw - 1, 7, 0) |
			FLD_VAL(t->hfp - 1, 19, 8) |
			FLD_VAL(t->hbp - 1, 31, 20));

	dispc6_vp_write(channel, DISPC_VP_TIMING_V,
			FLD_VAL(t->vsw - 1, 7, 0) |
			FLD_VAL(t->vfp, 19, 8) |
			FLD_VAL(t->vbp, 31, 20));

	/* always use aligned syncs */
	align = true;

	/* always use the 'rf' setting */
	onoff = true;

	switch (t->sync_pclk_edge) {
	case OMAPDSS_DRIVE_SIG_FALLING_EDGE:
		rf = false;
		break;
	case OMAPDSS_DRIVE_SIG_RISING_EDGE:
		rf = true;
		break;
	default:
		BUG();
	}

	switch (t->de_level) {
	case OMAPDSS_SIG_ACTIVE_LOW:
		ieo = true;
		break;
	case OMAPDSS_SIG_ACTIVE_HIGH:
		ieo = false;
		break;
	default:
		BUG();
	}

	switch (t->data_pclk_edge) {
	case OMAPDSS_DRIVE_SIG_RISING_EDGE:
		ipc = false;
		break;
	case OMAPDSS_DRIVE_SIG_FALLING_EDGE:
		ipc = true;
		break;
	default:
		BUG();
	}

	switch (t->hsync_level) {
	case OMAPDSS_SIG_ACTIVE_LOW:
		ihs = true;
		break;
	case OMAPDSS_SIG_ACTIVE_HIGH:
		ihs = false;
		break;
	default:
		BUG();
	}

	switch (t->vsync_level) {
	case OMAPDSS_SIG_ACTIVE_LOW:
		ivs = true;
		break;
	case OMAPDSS_SIG_ACTIVE_HIGH:
		ivs = false;
		break;
	default:
		BUG();
	}

	dispc6_vp_write(channel, DISPC_VP_POL_FREQ,
			FLD_VAL(align, 18, 18) |
			FLD_VAL(onoff, 17, 17) |
			FLD_VAL(rf, 16, 16) |
			FLD_VAL(ieo, 15, 15) |
			FLD_VAL(ipc, 14, 14) |
			FLD_VAL(ihs, 13, 13) |
			FLD_VAL(ivs, 12, 12));

	dispc6_vp_write(channel, DISPC_VP_SIZE_SCREEN,
			FLD_VAL(t->x_res - 1, 11, 0) |
			FLD_VAL(t->y_res - 1, 27, 16));
}

int dispc6_vp_enable_clk(enum omap_channel channel)
{
	return clk_prepare_enable(dispc.vp_clk);
}
void dispc6_vp_disable_clk(enum omap_channel channel)
{
	clk_disable_unprepare(dispc.vp_clk);
}

int dispc6_vp_set_clk_rate(enum omap_channel channel, unsigned long rate)
{
	int r;
	unsigned long new_rate;

	r = clk_set_rate(dispc.vp_clk, rate);
	if (r) {
		dev_err(&dispc.pdev->dev, "Failed to set vp clk rate to %lu\n",
			rate);
		return r;
	}

	new_rate = clk_get_rate(dispc.vp_clk);

	if (rate != new_rate)
		dev_warn(&dispc.pdev->dev,
			 "Failed to get exact pix clock %lu != %lu\n",
			 rate, new_rate);

	dev_dbg(&dispc.pdev->dev, "New VP rate %lu Hz (requested %lu Hz)\n",
		clk_get_rate(dispc.vp_clk), rate);

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

static void dispc6_vid_write_color_conv_coefs(enum omap_plane plane,
		const struct color_conv_coef *ct)
{
#define CVAL(x, y) (FLD_VAL(x, 26, 16) | FLD_VAL(y, 10, 0))

	dispc6_vid_write(plane, DISPC_VID_CONV_COEF(0), CVAL(ct->rcr, ct->ry));
	dispc6_vid_write(plane, DISPC_VID_CONV_COEF(1), CVAL(ct->gy,  ct->rcb));
	dispc6_vid_write(plane, DISPC_VID_CONV_COEF(2), CVAL(ct->gcb, ct->gcr));
	dispc6_vid_write(plane, DISPC_VID_CONV_COEF(3), CVAL(ct->bcr, ct->by));
	dispc6_vid_write(plane, DISPC_VID_CONV_COEF(4), CVAL(0, ct->bcb));

	dispc6_vid_write(plane, DISPC_VID_CONV_COEF(5),
			 FLD_VAL(ct->roffset, 15, 3) | FLD_VAL(ct->goffset, 31, 19));
	dispc6_vid_write(plane, DISPC_VID_CONV_COEF(6),
			 FLD_VAL(ct->boffset, 15, 3));

	VID_REG_FLD_MOD(plane, DISPC_VID_ATTRIBUTES, ct->full_range, 11, 11);

#undef CVAL
}

static void dispc6_vid_csc_setup(void)
{
	/* YUV -> RGB, ITU-R BT.601, full range */
	const struct color_conv_coef coefs_yuv2rgb_bt601_full = {
		256,   0,  358,
		256, -88, -182,
		256, 452,    0,
		0, -2048, -2048,
		true,
	};

	dispc6_vid_write_color_conv_coefs(0, &coefs_yuv2rgb_bt601_full);
}

static void dispc6_vid_csc_enable(enum omap_plane plane, bool enable)
{
	VID_REG_FLD_MOD(plane, DISPC_VID_ATTRIBUTES, !!enable, 9, 9);
}

/* SCALER */

static u32 dispc6_calc_fir_inc(unsigned in, unsigned out)
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

/* M=8, Upscale x >= 1 */
static const struct dispc6_vid_fir_coefs dispc6_fir_coefs_m8 = {
	.c2 = {	0, -4, -8, -16, -24, -32, -40, -48, 0, 2, 4, 6, 8, 6, 4, 2,  },
	.c1 = { 0, 28, 56, 94, 132, 176, 220, 266, -56, -60, -64, -62, -60, -50, -40, -20,  },
	.c0 = { 512, 506, 500, 478, 456, 424, 392, 352, 312,  },
};

/* 5-tap, M=22, Downscale Ratio 2.5 < x < 3 */
static const struct dispc6_vid_fir_coefs dispc6_fir_coefs_m22_5tap = {
	.c2 = { 16, 20, 24, 30, 36, 42, 48, 56, 0, 0, 0, 2, 4, 8, 12, 14,  },
	.c1 = { 132, 140, 148, 156, 164, 172, 180, 186, 64, 72, 80, 88, 96, 104, 112, 122,  },
	.c0 = { 216, 216, 216, 214, 212, 208, 204, 198, 192,  },
};

/* 3-tap, M=22, Downscale Ratio 2.5 < x < 3 */
static const struct dispc6_vid_fir_coefs dispc6_fir_coefs_m22_3tap = {
	.c1 = { 100, 118, 136, 156, 176, 196, 216, 236, 0, 10, 20, 30, 40, 54, 68, 84,  },
	.c0 = { 312, 310, 308, 302, 296, 286, 276, 266, 256,  },
};

enum dispc6_vid_fir_coef_set {
	DISPC6_VID_FIR_COEF_HORIZ,
	DISPC6_VID_FIR_COEF_HORIZ_UV,
	DISPC6_VID_FIR_COEF_VERT,
	DISPC6_VID_FIR_COEF_VERT_UV,
};

static void dispc6_vid_write_fir_coefs(enum omap_plane plane,
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

		dispc6_vid_write(plane, reg, c0);
	}

	for (phase = 0; phase <= 15; ++phase) {
		u16 reg = c12_base + phase * 4;
		s16 c1, c2;
		u32 c12;

		c1 = coefs->c1[phase];
		c2 = coefs->c2[phase];
		c12 = FLD_VAL(c1, 19, 10) | FLD_VAL(c2, 29, 20);

		dispc6_vid_write(plane, reg, c12);
	}
}

static void dispc6_vid_write_scale_coefs(enum omap_plane plane)
{
	dispc6_vid_write_fir_coefs(plane, DISPC6_VID_FIR_COEF_HORIZ, &dispc6_fir_coefs_null);
	dispc6_vid_write_fir_coefs(plane, DISPC6_VID_FIR_COEF_HORIZ_UV, &dispc6_fir_coefs_null);
	dispc6_vid_write_fir_coefs(plane, DISPC6_VID_FIR_COEF_VERT, &dispc6_fir_coefs_null);
	dispc6_vid_write_fir_coefs(plane, DISPC6_VID_FIR_COEF_VERT_UV, &dispc6_fir_coefs_null);
}

static void dispc6_vid_set_scaling(enum omap_plane plane,
				   unsigned orig_width, unsigned orig_height,
				   unsigned out_width, unsigned out_height,
				   u32 fourcc)
{
	unsigned in_w, in_h, in_w_uv, in_h_uv;
	unsigned fir_hinc, fir_vinc, fir_hinc_uv, fir_vinc_uv;
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
	VID_REG_FLD_MOD(plane, DISPC_VID_ATTRIBUTES, scale_x, 7, 7);

	/* VERTICAL RESIZE ENABLE */
	VID_REG_FLD_MOD(plane, DISPC_VID_ATTRIBUTES, scale_y, 8, 8);

	/* Skip the rest if no scaling is used */
	if (!scale_x && !scale_y)
		return;

	/* VERTICAL 5-TAPS  */
	VID_REG_FLD_MOD(plane, DISPC_VID_ATTRIBUTES, five_taps, 21, 21);

	/* FIR INC */

	fir_hinc = dispc6_calc_fir_inc(in_w, out_width);
	fir_vinc = dispc6_calc_fir_inc(in_h, out_height);
	fir_hinc_uv = dispc6_calc_fir_inc(in_w_uv, out_width);
	fir_vinc_uv = dispc6_calc_fir_inc(in_h_uv, out_height);

	dispc6_vid_write(plane, DISPC_VID_FIRH, fir_hinc);
	dispc6_vid_write(plane, DISPC_VID_FIRV, fir_vinc);
	dispc6_vid_write(plane, DISPC_VID_FIRH2, fir_hinc_uv);
	dispc6_vid_write(plane, DISPC_VID_FIRV2, fir_vinc_uv);

	dispc6_vid_write_scale_coefs(plane);
}

/* OTHER */

static const struct {
	u32 fourcc;
	enum omap_color_mode color_mode;
	u8 dss_code;
	u8 bytespp;
} dispc6_color_formats[] = {
	{ DRM_FORMAT_XRGB8888, OMAP_DSS_COLOR_RGB24U, 0x27, 4, },
	{ DRM_FORMAT_ARGB8888, OMAP_DSS_COLOR_ARGB32, 0x7, 4, },

	{ DRM_FORMAT_RGBX8888, OMAP_DSS_COLOR_RGBX32, 0x29, 4, },
	{ DRM_FORMAT_RGBA8888, OMAP_DSS_COLOR_RGBA32, 0x9, 4, },

	{ DRM_FORMAT_YUYV, OMAP_DSS_COLOR_YUV2, 0x3e, 2, },
	{ DRM_FORMAT_UYVY, OMAP_DSS_COLOR_UYVY, 0x3f, 2, },

	{ DRM_FORMAT_NV12, OMAP_DSS_COLOR_NV12, 0x3d, 2, },
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

static u32 dispc6_dss_colormode_to_fourcc(enum omap_color_mode dss_mode)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dispc6_color_formats); ++i) {
		if (dispc6_color_formats[i].color_mode == dss_mode)
			return dispc6_color_formats[i].fourcc;
	}

	__WARN();
	return DRM_FORMAT_XRGB8888;
}

static void dispc6_ovl_set_channel_out(enum omap_plane plane, enum omap_channel channel)
{
	int val = 0;
	VID_REG_FLD_MOD(plane, DISPC_VID_ATTRIBUTES, val, 16, 14);
}

static void dispc6_ovl_set_pixel_format(enum omap_plane plane, u32 fourcc)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dispc6_color_formats); ++i) {
		if (dispc6_color_formats[i].fourcc == fourcc) {
			VID_REG_FLD_MOD(plane, DISPC_VID_ATTRIBUTES,
					dispc6_color_formats[i].dss_code,
					6, 1);
			return;
		}
	}

	__WARN();
}

static int dispc6_fourcc_to_bytespp(u32 fourcc)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dispc6_color_formats); ++i) {
		if (dispc6_color_formats[i].fourcc == fourcc)
			return dispc6_color_formats[i].bytespp;
	}

	__WARN();
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

	BUG();
	return 0;
}

static int dispc6_ovl_setup(enum omap_plane plane, const struct omap_overlay_info *oi,
			    bool replication, const struct omap_video_timings *mgr_timings,
			    bool mem_to_mem)
{
	u32 fourcc = dispc6_dss_colormode_to_fourcc(oi->color_mode);
	int bytespp = dispc6_fourcc_to_bytespp(fourcc);

	dispc6_ovl_set_pixel_format(plane, fourcc);

	dispc6_vid_write(plane, DISPC_VID_BA_0, oi->paddr);
	dispc6_vid_write(plane, DISPC_VID_BA_1, oi->paddr);

	dispc6_vid_write(plane, DISPC_VID_BA_UV_0, oi->p_uv_addr);
	dispc6_vid_write(plane, DISPC_VID_BA_UV_1, oi->p_uv_addr);

	dispc6_vid_write(plane, DISPC_VID_PICTURE_SIZE,
			 (oi->width - 1) | ((oi->height - 1) << 16));

	dispc6_vid_write(plane, DISPC_VID_PIXEL_INC, pixinc(1, bytespp));
	dispc6_vid_write(plane, DISPC_VID_ROW_INC,
			 pixinc(1 + oi->screen_width - oi->width, bytespp));

	dispc6_vid_write(plane, DISPC_VID_POSITION,
			 oi->pos_x | (oi->pos_y << 16));

	dispc6_vid_write(plane, DISPC_VID_SIZE,
			 (oi->out_width - 1) | ((oi->out_height - 1) << 16));

	dispc6_vid_set_scaling(plane,
			       oi->width, oi->height, oi->out_width, oi->out_height,
			       fourcc);

	/* enable YUV->RGB color conversion */
	if (dispc6_fourcc_is_yuv(fourcc))
		dispc6_vid_csc_enable(plane, true);
	else
		dispc6_vid_csc_enable(plane, false);

	return 0;
}

static int dispc6_ovl_enable(enum omap_plane plane, bool enable)
{
	VID_REG_FLD_MOD(plane, DISPC_VID_ATTRIBUTES, !!enable, 0, 0);
	return 0;
}

static bool dispc6_ovl_enabled(enum omap_plane plane)
{
	return VID_REG_GET(plane, DISPC_VID_ATTRIBUTES, 0, 0);
}

static u32 dispc6_vid_get_fifo_size(enum omap_plane plane)
{
	const u32 unit_size = 16;	/* 128-bits */

	return VID_REG_GET(plane, DISPC_VID_BUF_SIZE_STATUS, 15, 0) * unit_size;
}

static void dispc6_vid_set_mflag_threshold(enum omap_plane plane,
		unsigned low, unsigned high)
{
	dispc6_vid_write(plane, DISPC_VID_MFLAG_THRESHOLD,
			 FLD_VAL(high, 31, 16) |	FLD_VAL(low, 15, 0));
}

static void dispc6_mflag_setup(void)
{
	enum omap_plane plane = 0;
	const u32 unit_size = 16;	/* 128-bits */
	u32 size = dispc6_vid_get_fifo_size(plane);
	u32 low, high;

	/* MFLAG_CTRL */
	REG_FLD_MOD(DISPC_GLOBAL_MFLAG_ATTRIBUTE, 1, 1, 0);
	/* MFLAG_START */
	REG_FLD_MOD(DISPC_GLOBAL_MFLAG_ATTRIBUTE, 0, 2, 2);

	/*
	 * Simulation team suggests below thesholds:
	 * HT = fifosize * 5 / 8;
	 * LT = fifosize * 4 / 8;
	 */

	low = size * 4 / 8 / unit_size;
	high = size * 5 / 8 / unit_size;

	dispc6_vid_set_mflag_threshold(plane, low, high);
}

static void dispc6_vp_setup(void)
{
	/* Enable the gamma Shadow bit-field */
	VP_REG_FLD_MOD(0, DISPC_VP_CONFIG, 1, 2, 2);
}

static void dispc6_initial_config(void)
{
	dispc6_vid_csc_setup();
	dispc6_mflag_setup();
	dispc6_vp_setup();
}

static int dispc6_init_features(struct platform_device *pdev)
{
	const struct of_device_id *match;

	match = of_match_node(dispc6_of_match, pdev->dev.of_node);
	if (!match) {
		dev_err(&pdev->dev, "Unsupported DISPC version\n");
		return -ENODEV;
	}

	dispc.feat = match->data;

	return 0;
}

static enum omap_dss_output_id dispc6_mgr_get_supported_outputs(enum omap_channel channel)
{
	return OMAP_DSS_OUTPUT_DPI;
}

static enum omap_color_mode dispc6_ovl_get_color_modes(enum omap_plane plane)
{
	enum omap_color_mode formats = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(dispc6_color_formats); ++i)
		formats |= dispc6_color_formats[i].color_mode;

	return formats;
}

static bool dispc6_has_writeback(void)
{
	return false;
}

static int dispc6_get_num_ovls(void)
{
	return 1;
}

static int dispc6_get_num_mgrs(void)
{
	return 1;
}

static u32 dispc6_mgr_gamma_size(enum omap_channel channel)
{
	return ARRAY_SIZE(dispc.gamma_table);
}

static void dispc6_mgr_write_gamma_table(enum omap_channel channel)
{
	u32 *table = dispc.gamma_table;
	uint hwlen = ARRAY_SIZE(dispc.gamma_table);
	unsigned int i;

	dev_dbg(&dispc.pdev->dev, "%s: channel %d\n", __func__, channel);

	for (i = 0; i < hwlen; ++i) {
		u32 v = table[i];

		v |= i << 24;

		dispc6_vp_write(channel, DISPC_VP_GAMMA_TABLE, v);
	}
}

static void dispc6_restore_gamma_tables(void)
{
	dev_dbg(&dispc.pdev->dev, "%s()\n", __func__);

	dispc6_mgr_write_gamma_table(0);
}

static const struct drm_color_lut dispc6_mgr_gamma_default_lut[] = {
	{ .red = 0, .green = 0, .blue = 0, },
	{ .red = U16_MAX, .green = U16_MAX, .blue = U16_MAX, },
};

static void dispc6_mgr_set_gamma(enum omap_channel channel,
			 const struct drm_color_lut *lut,
			 unsigned int length)
{
	u32 *table = dispc.gamma_table;
	uint hwlen = ARRAY_SIZE(dispc.gamma_table);
	static const uint hwbits = 8;
	uint i;

	dev_dbg(&dispc.pdev->dev, "%s: channel %d, lut len %u, hw len %u\n",
		__func__, channel, length, hwlen);

	if (lut == NULL || length < 2) {
		lut = dispc6_mgr_gamma_default_lut;
		length = ARRAY_SIZE(dispc6_mgr_gamma_default_lut);
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
			r = (lut[i].red * (w - j) + lut[i+1].red * j) / w;
			g = (lut[i].green * (w - j) + lut[i+1].green * j) / w;
			b = (lut[i].blue * (w - j) + lut[i+1].blue * j) / w;

			r >>= 16 - hwbits;
			g >>= 16 - hwbits;
			b >>= 16 - hwbits;

			table[first + j] = (r << (hwbits * 2)) |
				(g << hwbits) | b;
		}
	}

	if (dispc.is_enabled)
		dispc6_mgr_write_gamma_table(channel);
}

static int dispc6_init_gamma_tables(void)
{
	dispc6_mgr_set_gamma(0, NULL, 0);

	return 0;
}

static const struct dispc_ops dispc6_ops = {
	.read_irqstatus = dispc6_read_legacy_irqstatus,
	.clear_irqstatus = dispc6_clear_legacy_irqstatus,
	.read_irqenable = dispc6_read_legacy_irqenable,
	.write_irqenable = dispc6_write_legacy_irqenable,

	.request_irq = dispc6_request_irq,
	.free_irq = dispc6_free_irq,

	.runtime_get = dispc6_runtime_get,
	.runtime_put = dispc6_runtime_put,

	.get_num_ovls = dispc6_get_num_ovls,
	.get_num_mgrs = dispc6_get_num_mgrs,

	.mgr_enable = dispc6_mgr_enable,
	.mgr_is_enabled = dispc6_mgr_is_enabled,
	.mgr_get_vsync_irq = dispc6_mgr_get_vsync_irq,
	.mgr_get_framedone_irq = dispc6_mgr_get_framedone_irq,
	.mgr_get_sync_lost_irq = dispc6_mgr_get_sync_lost_irq,
	.mgr_go_busy = dispc6_mgr_go_busy,
	.mgr_go = dispc6_mgr_go,
	.mgr_set_lcd_config = dispc6_mgr_set_lcd_config,
	.mgr_set_timings = dispc6_mgr_set_timings,
	.mgr_setup = dispc6_mgr_setup,
	.mgr_get_supported_outputs = dispc6_mgr_get_supported_outputs,
	.mgr_gamma_size = dispc6_mgr_gamma_size,
	.mgr_set_gamma = dispc6_mgr_set_gamma,

	.ovl_enable = dispc6_ovl_enable,
	.ovl_enabled = dispc6_ovl_enabled,
	.ovl_set_channel_out = dispc6_ovl_set_channel_out,
	.ovl_setup = dispc6_ovl_setup,
	.ovl_get_color_modes = dispc6_ovl_get_color_modes,

	.has_writeback = dispc6_has_writeback,
};

static int dispc6_iomap_resource(struct platform_device *pdev, const char *name,
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

static int dispc6_bind(struct device *dev, struct device *master, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	int r = 0;

	dispc.pdev = pdev;

	r = dispc6_init_features(dispc.pdev);
	if (r)
		return r;

	r = dispc6_iomap_resource(pdev, "common", &dispc.base_common);
	if (r)
		return r;

	r = dispc6_iomap_resource(pdev, "vid1", &dispc.base_vid1);
	if (r)
		return r;

	r = dispc6_iomap_resource(pdev, "ovr1", &dispc.base_ovr1);
	if (r)
		return r;

	r = dispc6_iomap_resource(pdev, "vp1", &dispc.base_vp1);
	if (r)
		return r;

	dispc.irq = platform_get_irq(dispc.pdev, 0);
	if (dispc.irq < 0) {
		dev_err(dev, "platform_get_irq failed\n");
		return -ENODEV;
	}

	dispc.fclk = devm_clk_get(dev, "fck");
	if (IS_ERR(dispc.fclk)) {
		dev_err(dev, "Failed to get fclk\n");
		return PTR_ERR(dispc.fclk);
	}

	dispc.vp_clk = devm_clk_get(dev, "vp");
	if (IS_ERR(dispc.vp_clk)) {
		dev_err(dev, "Failed to get vp clk\n");
		return PTR_ERR(dispc.vp_clk);
	}

	r = dispc6_init_gamma_tables();
	if (r)
		return r;

	pm_runtime_enable(&pdev->dev);

	pm_runtime_set_autosuspend_delay(&pdev->dev, 200);
	pm_runtime_use_autosuspend(&pdev->dev);

	dispc_set_ops(&dispc6_ops);

	return 0;
}

static void dispc6_unbind(struct device *dev, struct device *master,
			  void *data)
{
	dispc_set_ops(NULL);

	pm_runtime_disable(dev);
}

static const struct component_ops dispc6_component_ops = {
	.bind	= dispc6_bind,
	.unbind	= dispc6_unbind,
};

static int dispc6_probe(struct platform_device *pdev)
{
	return component_add(&pdev->dev, &dispc6_component_ops);
}

static int dispc6_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &dispc6_component_ops);
	return 0;
}

static int dispc6_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "suspend\n");

	dispc.is_enabled = false;
	/* ensure the dispc6_irq_handler sees the is_enabled value */
	smp_wmb();
	/* wait for current handler to finish before turning the DISPC off */
	synchronize_irq(dispc.irq);

	dispc6_save_context();

	clk_disable_unprepare(dispc.fclk);

	return 0;
}

static int dispc6_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "resume\n");

	clk_prepare_enable(dispc.fclk);

	if (REG_GET(DISPC_SYSSTATUS, 0, 0) == 0)
		dev_warn(dev, "DISPC FUNC RESET not done!\n");
	if (REG_GET(DISPC_SYSSTATUS, 1, 1) == 0)
		dev_warn(dev, "DISPC VP RESET not done!\n");

	dispc6_initial_config();

	dispc6_restore_context();

	dispc6_restore_gamma_tables();

	dispc.is_enabled = true;
	/* ensure the dispc6_irq_handler sees the is_enabled value */
	smp_wmb();

	return 0;
}

static const struct dev_pm_ops dispc_pm_ops = {
	.runtime_suspend = dispc6_runtime_suspend,
	.runtime_resume = dispc6_runtime_resume,
};

static const struct of_device_id dispc6_of_match[] = {
	{ .compatible = "ti,k2g-dispc", .data = &k2g_dispc_feats, },
	{},
};

static struct platform_driver dispc6_driver = {
	.probe		= dispc6_probe,
	.remove         = dispc6_remove,
	.driver         = {
		.name   = "omap_dispc6",
		.pm	= &dispc_pm_ops,
		.of_match_table = dispc6_of_match,
		.suppress_bind_attrs = true,
	},
};

int __init dispc6_init_platform_driver(void)
{
	return platform_driver_register(&dispc6_driver);
}

void dispc6_uninit_platform_driver(void)
{
	platform_driver_unregister(&dispc6_driver);
}
