/*
 * File: drivers/video/omap/omap2/dispc.c
 *
 * OMAP2 display controller support
 *
 * Copyright (C) 2005 Nokia Corporation
 * Author: Imre Deak <imre.deak@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include <linux/kernel.h>
#include <linux/dma-mapping.h>

#include <asm/io.h>

#include <asm/arch/omapfb.h>

#include <asm/hardware/clock.h>

#include "dispc.h"

/* #define OMAPFB_DBG	2 */

#include "debug.h"

#define MODULE_NAME			"omapfb-dispc"

#define DISPC_BASE			0x48050400

/* DISPC common */
#define DISPC_REVISION			0x0000
#define DISPC_SYSCONFIG			0x0010
#define DISPC_SYSSTATUS			0x0014
#define DISPC_IRQSTATUS			0x0018
#define DISPC_IRQENABLE			0x001C
#define DISPC_CONTROL			0x0040
#define DISPC_CONFIG			0x0044
#define DISPC_CAPABLE			0x0048
#define DISPC_DEFAULT_COLOR0		0x004C
#define DISPC_DEFAULT_COLOR1		0x0050
#define DISPC_TRANS_COLOR0		0x0054
#define DISPC_TRANS_COLOR1		0x0058
#define DISPC_LINE_STATUS		0x005C
#define DISPC_LINE_NUMBER		0x0060
#define DISPC_TIMING_H			0x0064
#define DISPC_TIMING_V			0x0068
#define DISPC_POL_FREQ			0x006C
#define DISPC_DIVISOR			0x0070
#define DISPC_SIZE_DIG			0x0078
#define DISPC_SIZE_LCD			0x007C

#define DISPC_DATA_CYCLE1		0x01D4
#define DISPC_DATA_CYCLE2		0x01D8
#define DISPC_DATA_CYCLE3		0x01DC

/* DISPC GFX plane */
#define DISPC_GFX_BA0			0x0080
#define DISPC_GFX_BA1			0x0084
#define DISPC_GFX_POSITION		0x0088
#define DISPC_GFX_SIZE			0x008C
#define DISPC_GFX_ATTRIBUTES		0x00A0
#define DISPC_GFX_FIFO_THRESHOLD	0x00A4
#define DISPC_GFX_FIFO_SIZE_STATUS	0x00A8
#define DISPC_GFX_ROW_INC		0x00AC
#define DISPC_GFX_PIXEL_INC		0x00B0
#define DISPC_GFX_WINDOW_SKIP		0x00B4
#define DISPC_GFX_TABLE_BA		0x00B8

/* DISPC Video plane 1/2 */
#define DISPC_VID1_BASE			0x00BC
#define DISPC_VID2_BASE			0x014C

/* Offsets into DISPC_VID1/2_BASE */
#define DISPC_VID_BA0			0x0000
#define DISPC_VID_BA1			0x0004
#define DISPC_VID_POSITION		0x0008
#define DISPC_VID_SIZE			0x000C
#define DISPC_VID_ATTRIBUTES		0x0010
#define DISPC_VID_FIFO_THRESHOLD	0x0014
#define DISPC_VID_FIFO_SIZE_STATUS	0x0018
#define DISPC_VID_ROW_INC		0x001C
#define DISPC_VID_PIXEL_INC		0x0020
#define DISPC_VID_FIR			0x0024
#define DISPC_VID_PICTURE_SIZE		0x0028
#define DISPC_VID_ACCU0			0x002C
#define DISPC_VID_ACCU1			0x0030

/* 8 elements in 8 byte increments */
#define DISPC_VID_FIR_COEF_H0		0x0034
/* 8 elements in 8 byte increments */
#define DISPC_VID_FIR_COEF_HV0		0x0038
/* 5 elements in 4 byte increments */
#define DISPC_VID_CONV_COEF0		0x0074

#define DISPC_IRQ_FRAMEMASK		0x0001
#define DISPC_IRQ_VSYNC			0x0002
#define DISPC_IRQ_EVSYNC_EVEN		0x0004
#define DISPC_IRQ_EVSYNC_ODD		0x0008
#define DISPC_IRQ_ACBIAS_COUNT_STAT	0x0010
#define DISPC_IRQ_PROG_LINE_NUM		0x0020
#define DISPC_IRQ_GFX_FIFO_UNDERFLOW	0x0040
#define DISPC_IRQ_GFX_END_WIN		0x0080
#define DISPC_IRQ_PAL_GAMMA_MASK	0x0100
#define DISPC_IRQ_OCP_ERR		0x0200
#define DISPC_IRQ_VID1_FIFO_UNDERFLOW	0x0400
#define DISPC_IRQ_VID1_END_WIN		0x0800
#define DISPC_IRQ_VID2_FIFO_UNDERFLOW	0x1000
#define DISPC_IRQ_VID2_END_WIN		0x2000
#define DISPC_IRQ_SYNC_LOST		0x4000

#define DISPC_IRQ_MASK_ALL		0x7fff

#define DISPC_IRQ_MASK_ERROR		(DISPC_IRQ_GFX_FIFO_UNDERFLOW |	\
					     DISPC_IRQ_VID1_FIFO_UNDERFLOW | \
					     DISPC_IRQ_VID2_FIFO_UNDERFLOW | \
					     DISPC_IRQ_SYNC_LOST)

#define MAX_PALETTE_SIZE		(256 * 16)

#define pr_err(fmt, args...) printk(KERN_ERR MODULE_NAME ": " fmt, ## args)

#define FLD_MASK(pos, len)	(((1 << len) - 1) << pos)

#define MOD_REG_FLD(reg, mask, val) \
	dispc_write_reg((reg), (dispc_read_reg(reg) & ~(mask)) | (val));

static struct {
	u32		base;
	void		*vram_virt;
	dma_addr_t	vram_phys;
	int		vram_size;

	int		ext_mode;

	unsigned long	enabled_irqs;
	void		(*irq_callback)(void *);
	void		*irq_callback_data;
	struct completion	frame_done;

	struct clk	*dss_ick, *dss1_fck;
	struct clk	*dss_54m_fck;

	int		active_plane_mask;

	enum omapfb_update_mode	update_mode;
	struct omapfb_device	*fbdev;
} dispc;

static void inline dispc_write_reg(int idx, u32 val)
{
	__raw_writel(val, dispc.base + idx);
}

static u32 inline dispc_read_reg(int idx)
{
	u32 l = __raw_readl(dispc.base + idx);
	return l;
}

/* Select RFBI or bypass mode */
static void enable_rfbi_mode(int enable)
{
	u32 l;

	l = dispc_read_reg(DISPC_CONTROL);
	/* Enable RFBI, GPIO0/1 */
	l &= ~((1 << 11) | (1 << 15) | (1 << 16));
	l |= enable ? (1 << 11) : 0;
	/* RFBI En: GPIO0/1=10  RFBI Dis: GPIO0/1=11 */
	l |= 1 << 15;
	l |= enable ? 0 : (1 << 16);
	dispc_write_reg(DISPC_CONTROL, l);
}

static void set_lcd_data_lines(int data_lines)
{
	u32 l;
	int code = 0;

	switch (data_lines) {
	case 12:
		code = 0;
		break;
	case 16:
		code = 1;
		break;
	case 18:
		code = 2;
		break;
	case 24:
		code = 3;
		break;
	default:
		BUG();
	}

	l = dispc_read_reg(DISPC_CONTROL);
	l &= ~(0x03 << 8);
	l |= code << 8;
	dispc_write_reg(DISPC_CONTROL, l);
}

static void set_load_mode(int mode)
{
	BUG_ON(mode & ~(DISPC_LOAD_CLUT_ONLY | DISPC_LOAD_FRAME_ONLY |
			DISPC_LOAD_CLUT_ONCE_FRAME));
	MOD_REG_FLD(DISPC_CONFIG, 0x03 << 1, mode << 1);
}

void omap_dispc_set_lcd_size(int x, int y)
{
	BUG_ON((x > (1 << 11)) || (y > (1 << 11)));
	MOD_REG_FLD(DISPC_SIZE_LCD, FLD_MASK(16, 11) | FLD_MASK(0, 11),
			((y - 1) << 16) | (x - 1));
}
EXPORT_SYMBOL(omap_dispc_set_lcd_size);

void omap_dispc_set_digit_size(int x, int y)
{
	BUG_ON((x > (1 << 11)) || (y > (1 << 11)));
	MOD_REG_FLD(DISPC_SIZE_DIG, FLD_MASK(16, 11) | FLD_MASK(0, 11),
			((y - 1) << 16) | (x - 1));
}
EXPORT_SYMBOL(omap_dispc_set_digit_size);

static void setup_plane_fifo(int plane)
{
	const u32 ftrs_reg[] = { DISPC_GFX_FIFO_THRESHOLD,
				DISPC_VID1_BASE + DISPC_VID_FIFO_THRESHOLD,
			        DISPC_VID2_BASE + DISPC_VID_FIFO_THRESHOLD };
	const u32 fsz_reg[] = { DISPC_GFX_FIFO_SIZE_STATUS,
				DISPC_VID1_BASE + DISPC_VID_FIFO_SIZE_STATUS,
			        DISPC_VID2_BASE + DISPC_VID_FIFO_SIZE_STATUS };

	u32 l;

	BUG_ON(plane > 2);

	l = dispc_read_reg(fsz_reg[plane]);
	l &= FLD_MASK(0, 9);
	/* HIGH=3/4 LOW=1/4 */
	MOD_REG_FLD(ftrs_reg[plane], FLD_MASK(16, 9) | FLD_MASK(0, 9),
			((l * 3 / 4) << 16) | (l / 4));
}

void omap_dispc_enable_lcd_out(int enable)
{
	MOD_REG_FLD(DISPC_CONTROL, 1, enable ? 1 : 0);
}
EXPORT_SYMBOL(omap_dispc_enable_lcd_out);

void omap_dispc_enable_digit_out(int enable)
{
	MOD_REG_FLD(DISPC_CONTROL, 1 << 1, enable ? 1 << 1 : 0);
}
EXPORT_SYMBOL(omap_dispc_enable_digit_out);

static int omap_dispc_setup_plane(int plane, int channel_out,
				  unsigned long offset, int screen_width,
				  int pos_x, int pos_y, int width, int height,
				  int color_mode)
{
	const u32 at_reg[] = { DISPC_GFX_ATTRIBUTES,
				DISPC_VID1_BASE + DISPC_VID_ATTRIBUTES,
			        DISPC_VID2_BASE + DISPC_VID_ATTRIBUTES };
	const u32 ba_reg[] = { DISPC_GFX_BA0, DISPC_VID1_BASE + DISPC_VID_BA0,
				DISPC_VID2_BASE + DISPC_VID_BA0 };
	const u32 ps_reg[] = { DISPC_GFX_POSITION,
				DISPC_VID1_BASE + DISPC_VID_POSITION,
				DISPC_VID2_BASE + DISPC_VID_POSITION };
	const u32 sz_reg[] = { DISPC_GFX_SIZE, DISPC_VID1_BASE + DISPC_VID_SIZE,
				DISPC_VID2_BASE + DISPC_VID_SIZE };
	const u32 ri_reg[] = { DISPC_GFX_ROW_INC,
				DISPC_VID1_BASE + DISPC_VID_ROW_INC,
			        DISPC_VID2_BASE + DISPC_VID_ROW_INC };
	int chout_shift, burst_shift;
	int chout_val;
	int color_code;
	int bpp;
	u32 l;

	DBGENTER(1);

	switch (plane) {
	case OMAPFB_PLANE_GFX:
		burst_shift = 6;
		chout_shift = 8;
		break;
	case OMAPFB_PLANE_VID1:
	case OMAPFB_PLANE_VID2:
		burst_shift = 14;
		chout_shift = 16;
		break;
	default:
		return -EINVAL;
	}

	switch (channel_out) {
	case OMAPFB_CHANNEL_OUT_LCD:
		chout_val = 0;
		break;
	case OMAPFB_CHANNEL_OUT_DIGIT:
		chout_val = 1;
		break;
	default:
		return -EINVAL;
	}

	switch (color_mode) {
	case OMAPFB_COLOR_RGB565:
		color_code = DISPC_RGB_16_BPP;
		bpp = 16;
		break;
	case OMAPFB_COLOR_YUV422:
		if (plane != 0)
			return -EINVAL;
		color_code = DISPC_UYVY_422;
		bpp = 16;
		break;
	case OMAPFB_COLOR_YUV420:
		if (plane != 0)
			return -EINVAL;
		color_code = DISPC_YUV2_422;
		bpp = 12;
		break;
	default:
		return -EINVAL;
	}

	l = dispc_read_reg(at_reg[plane]);

	l &= ~(0x0f << 1);
	l |= color_code << 1;

	l &= ~(0x03 << burst_shift);
	l |= DISPC_BURST_8x32 << burst_shift;

	l &= ~(1 << chout_shift);
	l |= chout_val << chout_shift;

	dispc_write_reg(at_reg[plane], l);


	dispc_write_reg(ba_reg[plane],
		       dispc.vram_phys + PAGE_ALIGN(MAX_PALETTE_SIZE) + offset);

	MOD_REG_FLD(ps_reg[plane],
		    FLD_MASK(16, 11) | FLD_MASK(0, 11), (pos_y << 16) | pos_x);

	MOD_REG_FLD(sz_reg[plane], FLD_MASK(16, 11) | FLD_MASK(0, 11),
			((height - 1) << 16) | (width - 1));

	dispc_write_reg(ri_reg[plane], (screen_width - width) * bpp / 8 + 1);

	return 0;
}

static int omap_dispc_enable_plane(int plane, int enable)
{
	const u32 at_reg[] = { DISPC_GFX_ATTRIBUTES,
				DISPC_VID1_BASE + DISPC_VID_ATTRIBUTES,
			        DISPC_VID2_BASE + DISPC_VID_ATTRIBUTES };
	DBGENTER(1);

	if ((unsigned int)plane > 2)
		return -EINVAL;
	MOD_REG_FLD(at_reg[plane], 1, enable ? 1 : 0);

	return 0;
}

static int omap_dispc_set_color_key(struct omapfb_color_key *ck)
{
	u32 df_reg, tr_reg;
	int shift, val;

	switch (ck->channel_out) {
	case OMAPFB_CHANNEL_OUT_LCD:
		df_reg = DISPC_DEFAULT_COLOR0;
		tr_reg = DISPC_TRANS_COLOR0;
		shift = 10;
		break;
	case OMAPFB_CHANNEL_OUT_DIGIT:
		df_reg = DISPC_DEFAULT_COLOR1;
		tr_reg = DISPC_TRANS_COLOR1;
		shift = 12;
		break;
	default:
		return -EINVAL;
	}
	switch (ck->key_type) {
	case OMAPFB_COLOR_KEY_DISABLED:
		val = 0;
		break;
	case OMAPFB_COLOR_KEY_GFX_DST:
		val = 1;
		break;
	case OMAPFB_COLOR_KEY_VID_SRC:
		val = 3;
		break;
	default:
		return -EINVAL;
	}
	MOD_REG_FLD(DISPC_CONFIG, FLD_MASK(shift, 2), val << shift);

	if (val != 0)
		dispc_write_reg(tr_reg, ck->trans_key);
	dispc_write_reg(df_reg, ck->background);

	return 0;
}

static void load_palette(void)
{
}

static int omap_dispc_set_update_mode(enum omapfb_update_mode mode)
{
	int r = 0;

	DBGENTER(1);

	if (mode != dispc.update_mode) {
		switch (mode) {
		case OMAPFB_AUTO_UPDATE:
			omap_dispc_enable_lcd_out(1);
			dispc.update_mode = mode;
			break;
		case OMAPFB_UPDATE_DISABLED:
			init_completion(&dispc.frame_done);
			omap_dispc_enable_lcd_out(0);
			if (!wait_for_completion_timeout(&dispc.frame_done,
					msecs_to_jiffies(500))) {
				pr_err("timeout waiting for FRAME DONE\n");
			}
			dispc.update_mode = mode;
			break;
		default:
			r = -EINVAL;
		}
	}

	DBGLEAVE(1);

	return r;
}

static enum omapfb_update_mode omap_dispc_get_update_mode(void)
{
	return dispc.update_mode;
}

static void calc_ck_div(int is_tft, int pck, int *lck_div, int *pck_div)
{
	unsigned long fck, lck;

	*lck_div = 1;
	pck = max(1, pck);
	fck = clk_get_rate(dispc.dss1_fck);
	lck = fck;
	*pck_div = lck / pck;
	if (is_tft)
		*pck_div = max(2, *pck_div);
	else
		*pck_div = max(3, *pck_div);
	if (*pck_div > 255) {
		*pck_div = 255;
		lck = pck * *pck_div;
		*lck_div = fck / lck;
		BUG_ON(*lck_div < 1);
		if (*lck_div > 255) {
			*lck_div = 255;
			printk(KERN_WARNING
				MODULE_NAME ": pixclock %d kHz too low.\n",
				 pck / 1000);
		}
	}
}

static void set_lcd_timings(void)
{
	u32 l;
	int lck_div, pck_div;
	struct lcd_panel *panel = dispc.fbdev->panel;
	int is_tft = panel->config & OMAP_LCDC_PANEL_TFT;
	unsigned long fck;

	DBGENTER(1);

	/* TFT dither, TFT/STN */
	l = (1 << 7) | (1 << 3);
	MOD_REG_FLD(DISPC_CONTROL, l, is_tft ? l : 0);

	l = dispc_read_reg(DISPC_TIMING_H);
	l &= ~(FLD_MASK(0, 6) | FLD_MASK(8, 8) | FLD_MASK(20, 8));
	l |= ( max(1, (min(64,  panel->hsw))) - 1 ) << 0;
	l |= ( max(1, (min(256, panel->hfp))) - 1 ) << 8;
	l |= ( max(1, (min(256, panel->hbp))) - 1 ) << 20;
	dispc_write_reg(DISPC_TIMING_H, l);

	l = dispc_read_reg(DISPC_TIMING_V);
	l &= ~(FLD_MASK(0, 6) | FLD_MASK(8, 8) | FLD_MASK(20, 8));
	l |= ( max(1, (min(64,  panel->vsw))) - 1 ) << 0;
	l |= ( max(0, (min(255, panel->vfp))) - 0 ) << 8;
	l |= ( max(0, (min(255, panel->vbp))) - 0 ) << 20;
	dispc_write_reg(DISPC_TIMING_V, l);

	l = dispc_read_reg(DISPC_POL_FREQ);
	l &= ~FLD_MASK(12, 6);
	l |= (panel->config & OMAP_LCDC_SIGNAL_MASK) << 12;
	l |= panel->acb & 0xff;
	dispc_write_reg(DISPC_POL_FREQ, l);

	calc_ck_div(is_tft, panel->pixel_clock * 1000, &lck_div, &pck_div);

	l = dispc_read_reg(DISPC_DIVISOR);
	l &= ~(FLD_MASK(16, 8) | FLD_MASK(0, 8));
	l |= (lck_div << 16) | (pck_div << 0);
	dispc_write_reg(DISPC_DIVISOR, l);

	/* update panel info with the exact clock */
	fck = clk_get_rate(dispc.dss1_fck);
	panel->pixel_clock = fck / lck_div / pck_div / 1000;
}

int omap_dispc_request_irq(void (*callback)(void *data), void *data)
{
	int r = 0;

	BUG_ON(callback == NULL);

	if (dispc.irq_callback)
		r = -EBUSY;
	else {
		dispc.irq_callback = callback;
		dispc.irq_callback_data = data;
	}

	return r;
}
EXPORT_SYMBOL(omap_dispc_request_irq);

void omap_dispc_enable_irqs(int irq_mask)
{
	dispc.enabled_irqs = irq_mask;
	irq_mask |= DISPC_IRQ_MASK_ERROR;
	MOD_REG_FLD(DISPC_IRQENABLE, 0x7fff, irq_mask);
}
EXPORT_SYMBOL(omap_dispc_enable_irqs);

void omap_dispc_disable_irqs(int irq_mask)
{
	dispc.enabled_irqs &= ~irq_mask;
	irq_mask &= ~DISPC_IRQ_MASK_ERROR;
	MOD_REG_FLD(DISPC_IRQENABLE, 0x7fff, irq_mask);
}
EXPORT_SYMBOL(omap_dispc_disable_irqs);

void omap_dispc_free_irq(void)
{
	omap_dispc_disable_irqs(DISPC_IRQ_MASK_ALL);
	dispc.irq_callback = NULL;
	dispc.irq_callback_data = NULL;
}
EXPORT_SYMBOL(omap_dispc_free_irq);

static irqreturn_t omap_dispc_irq_handler(int irq, void *dev, struct pt_regs *regs)
{
	u32 stat = dispc_read_reg(DISPC_IRQSTATUS);
	static int jabber;

	DBGENTER(2);

	if (stat & DISPC_IRQ_FRAMEMASK)
		complete(&dispc.frame_done);

	if (stat & DISPC_IRQ_MASK_ERROR) {
		if (jabber++ < 5) {
			pr_err("irq error status %04x\n", stat);
		} else {
			pr_err("disable irq\n");
			dispc_write_reg(DISPC_IRQENABLE, 0);
		}
	}

	if ((stat & dispc.enabled_irqs) && dispc.irq_callback)
		dispc.irq_callback(dispc.irq_callback_data);

	dispc_write_reg(DISPC_IRQSTATUS, stat);

	return IRQ_HANDLED;
}

static int get_dss_clocks(void)
{
	if (IS_ERR((dispc.dss_ick = clk_get(dispc.fbdev->dev, "dss_ick")))) {
		pr_err("can't get dss_ick");
		return PTR_ERR(dispc.dss_ick);
	}

	if (IS_ERR((dispc.dss1_fck = clk_get(dispc.fbdev->dev, "dss1_fck")))) {
		pr_err("can't get dss1_fck");
		clk_put(dispc.dss_ick);
		return PTR_ERR(dispc.dss1_fck);
	}

	if (IS_ERR((dispc.dss_54m_fck =
				clk_get(dispc.fbdev->dev, "dss_54m_fck")))) {
		pr_err("can't get dss_54m_fck");
		clk_put(dispc.dss_ick);
		clk_put(dispc.dss1_fck);
		return PTR_ERR(dispc.dss_54m_fck);
	}

	return 0;
}

static void put_dss_clocks(void)
{
	clk_put(dispc.dss_54m_fck);
	clk_put(dispc.dss1_fck);
	clk_put(dispc.dss_ick);
}

static void enable_lcd_clocks(int enable)
{
	if (enable) {
		clk_enable(dispc.dss_ick);
		clk_enable(dispc.dss1_fck);
	} else {
		clk_disable(dispc.dss1_fck);
		clk_disable(dispc.dss_ick);
	}
}

static void enable_digit_clocks(int enable)
{
	if (enable)
		clk_enable(dispc.dss_54m_fck);
	else
		clk_disable(dispc.dss_54m_fck);
}

static void omap_dispc_suspend(void)
{
	DBGENTER(1);

	if (dispc.update_mode == OMAPFB_AUTO_UPDATE) {
		init_completion(&dispc.frame_done);
		omap_dispc_enable_lcd_out(0);
		if (!wait_for_completion_timeout(&dispc.frame_done,
				msecs_to_jiffies(500))) {
			pr_err("timeout waiting for FRAME DONE\n");
		}
		enable_lcd_clocks(0);
	}

	DBGLEAVE(1);
}

static void omap_dispc_resume(void)
{
	DBGENTER(1);

	if (dispc.update_mode == OMAPFB_AUTO_UPDATE) {
		enable_lcd_clocks(1);
		set_lcd_timings();
		load_palette();
		omap_dispc_enable_lcd_out(1);
	}

	DBGLEAVE(1);
}

/* Called when used in bypass mode */
static int alloc_vram(int req_size)
{
	int frame_size;
	struct lcd_panel *panel = dispc.fbdev->panel;

	frame_size = PAGE_ALIGN(panel->x_res * panel->bpp / 8 * panel->y_res);
	if (req_size > frame_size)
		frame_size = req_size;
	dispc.vram_size = PAGE_ALIGN(MAX_PALETTE_SIZE) + frame_size;
	dispc.vram_virt = dma_alloc_writecombine(dispc.fbdev->dev,
			dispc.vram_size, &dispc.vram_phys, GFP_KERNEL);

	if (dispc.vram_virt == 0) {
		pr_err("unable to allocate fb DMA memory\n");
		return -ENOMEM;
	}

	return 0;
}

static void free_vram(void)
{
	dma_free_writecombine(dispc.fbdev->dev, dispc.vram_size,
			      dispc.vram_virt, dispc.vram_phys);
}

static void omap_dispc_get_vram_layout(unsigned long *size, void **virt,
					dma_addr_t *phys)
{
	*size = dispc.vram_size - PAGE_ALIGN(MAX_PALETTE_SIZE);
	*virt = (u8 *)dispc.vram_virt + PAGE_ALIGN(MAX_PALETTE_SIZE);
	*phys = dispc.vram_phys + PAGE_ALIGN(MAX_PALETTE_SIZE);
}

static int omap_dispc_init(struct omapfb_device *fbdev, int ext_mode,
			   int req_vram_size)
{
	int r;
	u32 l;
	struct lcd_panel *panel = fbdev->panel;
	int tmo = 10000;

	DBGENTER(1);

	memset(&dispc, 0, sizeof(dispc));

	dispc.base = io_p2v(DISPC_BASE);
	dispc.fbdev = fbdev;
	dispc.ext_mode = ext_mode;

	if ((r = get_dss_clocks()) < 0)
		goto fail0;

	enable_lcd_clocks(1);
	/* Reset monitoring works only w/ the 54M clk */
	enable_digit_clocks(1);

	l = dispc_read_reg(DISPC_REVISION);
	pr_info(MODULE_NAME ": version %d.%d\n", l >> 4 & 0x0f, l & 0x0f);

	/* Soft reset */
	MOD_REG_FLD(DISPC_SYSCONFIG, 1 << 1, 1 << 1);

	while (!(dispc_read_reg(DISPC_SYSSTATUS) & 1)) {
		if (!--tmo) {
			pr_err("soft reset failed\n");
			r = -ENODEV;
			enable_digit_clocks(0);
			goto fail1;
		}
	}

	enable_digit_clocks(0);

	if (!ext_mode && (r = alloc_vram(req_vram_size)) < 0)
		goto fail1;

	/* Set logic clock to the fck for now */
	MOD_REG_FLD(DISPC_DIVISOR, FLD_MASK(16, 8), 1);

	setup_plane_fifo(0);
	setup_plane_fifo(1);
	setup_plane_fifo(2);

	l = dispc_read_reg(DISPC_IRQSTATUS);
	dispc_write_reg(l, DISPC_IRQSTATUS);

	/* Enable those that we handle always */
	omap_dispc_enable_irqs(DISPC_IRQ_FRAMEMASK);

	if ((r = request_irq(INT_24XX_DSS_IRQ, omap_dispc_irq_handler,
			   0, MODULE_NAME, NULL)) < 0) {
		pr_err("can't get DSS IRQ\n");
		goto fail2;
	}

	set_lcd_data_lines(panel->data_lines);
	set_load_mode(DISPC_LOAD_FRAME_ONLY);

	if (!ext_mode) {
		omap_dispc_set_lcd_size(panel->x_res, panel->y_res);
		set_lcd_timings();
	}
	enable_rfbi_mode(ext_mode);

	DBGLEAVE(1);
	return 0;
fail2:
	if (ext_mode)
		free_vram();
fail1:
	enable_lcd_clocks(0);
	put_dss_clocks();
fail0:
	DBGLEAVE(1);
        return r;
}

static void omap_dispc_cleanup(void)
{
	free_irq(INT_24XX_DSS_IRQ, NULL);
	enable_lcd_clocks(0);
	put_dss_clocks();
	if (dispc.ext_mode)
		free_vram();
}

static unsigned long omap_dispc_get_caps(void)
{
	return 0;
}

struct lcd_ctrl omap2_int_ctrl = {
	.name			= "internal",
	.init			= omap_dispc_init,
	.cleanup		= omap_dispc_cleanup,
	.get_vram_layout	= omap_dispc_get_vram_layout,
	.get_caps		= omap_dispc_get_caps,
	.set_update_mode	= omap_dispc_set_update_mode,
	.get_update_mode	= omap_dispc_get_update_mode,
	.update_window		= NULL,
	.suspend		= omap_dispc_suspend,
	.resume			= omap_dispc_resume,
	.setup_plane		= omap_dispc_setup_plane,
	.enable_plane		= omap_dispc_enable_plane,
	.set_color_key		= omap_dispc_set_color_key,
};

MODULE_DESCRIPTION("TI OMAP LCDC controller");
MODULE_LICENSE("GPL");
