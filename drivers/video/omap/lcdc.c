/*
 * File: drivers/video/omap/omap1/lcdc.c
 *
 * OMAP1 internal LCD controller
 *
 * Copyright (C) 2004 Nokia Corporation
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

#include <linux/config.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/err.h>
#include <linux/mm.h>
#include <linux/fb.h>
#include <linux/dma-mapping.h>

#include <asm/arch/dma.h>
#include <asm/arch/omapfb.h>

#include <asm/mach-types.h>
#include <asm/hardware/clock.h>

/* #define OMAPFB_DBG 2 */

#include "debug.h"

#define MODULE_NAME			"omapfb-lcdc"

#define OMAP_LCDC_BASE			0xfffec000
#define OMAP_LCDC_SIZE			256
#define OMAP_LCDC_IRQ			INT_LCD_CTRL

#define OMAP_LCDC_CONTROL		(OMAP_LCDC_BASE + 0x00)
#define OMAP_LCDC_TIMING0		(OMAP_LCDC_BASE + 0x04)
#define OMAP_LCDC_TIMING1		(OMAP_LCDC_BASE + 0x08)
#define OMAP_LCDC_TIMING2		(OMAP_LCDC_BASE + 0x0c)
#define OMAP_LCDC_STATUS		(OMAP_LCDC_BASE + 0x10)
#define OMAP_LCDC_SUBPANEL		(OMAP_LCDC_BASE + 0x14)
#define OMAP_LCDC_LINE_INT		(OMAP_LCDC_BASE + 0x18)
#define OMAP_LCDC_DISPLAY_STATUS	(OMAP_LCDC_BASE + 0x1c)

#define OMAP_LCDC_STAT_DONE		(1 << 0)
#define OMAP_LCDC_STAT_VSYNC		(1 << 1)
#define OMAP_LCDC_STAT_SYNC_LOST	(1 << 2)
#define OMAP_LCDC_STAT_ABC		(1 << 3)
#define OMAP_LCDC_STAT_LINE_INT		(1 << 4)
#define OMAP_LCDC_STAT_FUF		(1 << 5)
#define OMAP_LCDC_STAT_LOADED_PALETTE	(1 << 6)

#define OMAP_LCDC_CTRL_LCD_EN		(1 << 0)
#define OMAP_LCDC_CTRL_LCD_TFT		(1 << 7)
#define OMAP_LCDC_CTRL_LINE_IRQ_CLR_SEL	(1 << 10)

#define OMAP_LCDC_IRQ_VSYNC		(1 << 2)
#define OMAP_LCDC_IRQ_DONE		(1 << 3)
#define OMAP_LCDC_IRQ_LOADED_PALETTE	(1 << 4)
#define OMAP_LCDC_IRQ_LINE_NIRQ		(1 << 5)
#define OMAP_LCDC_IRQ_LINE		(1 << 6)
#define OMAP_LCDC_IRQ_MASK		(((1 << 5) - 1) << 2)

#define MAX_PALETTE_SIZE		PAGE_SIZE

#define pr_err(fmt, args...) printk(KERN_ERR MODULE_NAME ": " fmt, ## args)

enum lcdc_load_mode {
	OMAP_LCDC_LOAD_PALETTE,
	OMAP_LCDC_LOAD_FRAME,
	OMAP_LCDC_LOAD_PALETTE_AND_FRAME
};

static struct omap_lcd_controller {
	enum omapfb_update_mode	update_mode;

	unsigned long		frame_offset;
	int			screen_width;

	enum omapfb_color_format	color_mode;
	int			bpp;
	int			palette_org;
	int			palette_code;
	int			palette_size;

	unsigned int		irq_mask;
	struct completion	last_frame_complete;
	struct completion	palette_load_complete;
	struct clk		*lcd_ck;
	struct omapfb_device	*fbdev;

	dma_addr_t		vram_phys;
	void			*vram_virt;
	unsigned long		vram_size;
} omap_lcdc;

static void inline enable_irqs(int mask)
{
	omap_lcdc.irq_mask |= mask;
}

static void inline disable_irqs(int mask)
{
	omap_lcdc.irq_mask &= ~mask;
}

static void set_load_mode(enum lcdc_load_mode mode)
{
	u32 l;

	l = omap_readl(OMAP_LCDC_CONTROL);
	l &= ~(3 << 20);
	switch (mode) {
	case OMAP_LCDC_LOAD_PALETTE:
		l |= 1 << 20;
		break;
	case OMAP_LCDC_LOAD_FRAME:
		l |= 2 << 20;
		break;
	case OMAP_LCDC_LOAD_PALETTE_AND_FRAME:
		break;
	default:
		BUG();
	}
	omap_writel(l, OMAP_LCDC_CONTROL);
}

static void enable_controller(void)
{
	u32 l;

	l = omap_readl(OMAP_LCDC_CONTROL);
	l |= OMAP_LCDC_CTRL_LCD_EN;
	l &= ~OMAP_LCDC_IRQ_MASK;
	l |= omap_lcdc.irq_mask | OMAP_LCDC_IRQ_DONE;	/* enabled IRQs */
	omap_writel(l, OMAP_LCDC_CONTROL);
}

static void disable_controller_async(void)
{
	u32 l;
	u32 mask;

	l = omap_readl(OMAP_LCDC_CONTROL);
	mask = OMAP_LCDC_CTRL_LCD_EN | OMAP_LCDC_IRQ_MASK;
	/* Preserve the DONE mask, since we still want to get the
	 * final DONE irq. It will be disabled in the IRQ handler.
	 */
	mask &= ~OMAP_LCDC_IRQ_DONE;
	l &= ~mask;
	omap_writel(l, OMAP_LCDC_CONTROL);
}

static void disable_controller(void)
{
	init_completion(&omap_lcdc.last_frame_complete);
	disable_controller_async();
	if (!wait_for_completion_timeout(&omap_lcdc.last_frame_complete,
				msecs_to_jiffies(500)))
		pr_err("timeout waiting for FRAME DONE\n");
}

static void reset_controller(u32 status)
{
	static unsigned long reset_count = 0;
	static unsigned long last_jiffies = 0;

	disable_controller_async();
	reset_count++;
	if (reset_count == 1 || time_after(jiffies, last_jiffies + HZ)) {
		pr_err("resetting (status %#010x,reset count %lu)\n",
			  status, reset_count);
		last_jiffies = jiffies;
	}
	if (reset_count < 100) {
		enable_controller();
	} else {
		reset_count = 0;
		pr_err("too many reset attempts, giving up.\n");
	}
}

/* Configure the LCD DMA according to the current mode specified by parameters
 * in omap_lcdc.fbdev and fbdev->var.
 */
static void setup_lcd_dma(void)
{
	static const int dma_elem_type[] = {
		0,
		OMAP_DMA_DATA_TYPE_S8,
		OMAP_DMA_DATA_TYPE_S16,
		0,
		OMAP_DMA_DATA_TYPE_S32,
	};
	struct fb_var_screeninfo *var = &omap_lcdc.fbdev->fb_info->var;
	struct lcd_panel *panel = omap_lcdc.fbdev->panel;
	unsigned long	src;
	int		esize, xelem, yelem;

	src = omap_lcdc.vram_phys + PAGE_ALIGN(MAX_PALETTE_SIZE) +
		omap_lcdc.frame_offset;

	switch (var->rotate) {
	case 0:
		esize = omap_lcdc.fbdev->mirror || (src & 3) ? 2 : 4;
		xelem = panel->x_res * omap_lcdc.bpp / 8 / esize;
		yelem = panel->y_res;
		break;
	case 90:
	case 180:
	case 270:
		if (cpu_is_omap15xx()) {
			BUG();
		}
		esize = 2;
		xelem = panel->y_res * omap_lcdc.bpp / 16;
		yelem = panel->x_res;
		break;
	default:
		BUG();
		return;
	}
	DBGPRINT(1, "setup_dma: src %#010lx esize %d xelem %d yelem %d\n",
		 src, esize, xelem, yelem);
	omap_set_lcd_dma_b1(src, xelem, yelem, dma_elem_type[esize]);
	omap_set_lcd_dma_single_transfer(0);
	if (!cpu_is_omap15xx()) {
		/* Set virtual xres elem size */
		omap_set_lcd_dma_b1_vxres(
			omap_lcdc.screen_width * omap_lcdc.bpp / 8 / esize);
		/* Setup transformations */
		omap_set_lcd_dma_b1_rotation(var->rotate);
		omap_set_lcd_dma_b1_mirror(omap_lcdc.fbdev->mirror);
	}
	omap_setup_lcd_dma();
}

static irqreturn_t lcdc_irq_handler(int irq, void *dev_id, struct pt_regs *fp)
{
	u32 status;

	status = omap_readl(OMAP_LCDC_STATUS);

	if (status & (OMAP_LCDC_STAT_FUF | OMAP_LCDC_STAT_SYNC_LOST))
		reset_controller(status);
	else {
		if (status & OMAP_LCDC_STAT_DONE) {
			u32 l;

			/* Disable IRQ_DONE. The status bit will be cleared
			 * only when the controller is reenabled and we don't
			 * want to get more interrupts.
			 */
			l = omap_readl(OMAP_LCDC_CONTROL);
			l &= ~OMAP_LCDC_IRQ_DONE;
			omap_writel(l, OMAP_LCDC_CONTROL);
			complete(&omap_lcdc.last_frame_complete);
		}
		if (status & OMAP_LCDC_STAT_LOADED_PALETTE) {
			disable_controller_async();
			complete(&omap_lcdc.palette_load_complete);
		}
	}

	/* Clear these interrupt status bits.
	 * Sync_lost, FUF bits were cleared by disabling the LCD controller
	 * LOADED_PALETTE can be cleared this way only in palette only
	 * load mode. In other load modes it's cleared by disabling the
	 * controller.
	 */
	status &= ~(OMAP_LCDC_STAT_VSYNC |
		    OMAP_LCDC_STAT_LOADED_PALETTE |
		    OMAP_LCDC_STAT_ABC |
		    OMAP_LCDC_STAT_LINE_INT);
	omap_writel(status, OMAP_LCDC_STATUS);
	return IRQ_HANDLED;
}

/* Change to a new video mode. We defer this to a later time to avoid any
 * flicker and not to mess up the current LCD DMA context. For this we disable
 * the LCD controler, which will generate a DONE irq after the last frame has
 * been transferred. Then it'll be safe to reconfigure both the LCD controller
 * as well as the LCD DMA.
 */
static int omap_lcdc_setup_plane(int plane, int channel_out,
				 unsigned long offset, int screen_width,
				 int pos_x, int pos_y, int width, int height,
				 int color_mode)
{
	struct fb_var_screeninfo *var = &omap_lcdc.fbdev->fb_info->var;
	struct lcd_panel *panel = omap_lcdc.fbdev->panel;
	int rot_x, rot_y;

	DBGENTER(1);

	if (var->rotate == 0) {
		rot_x = panel->x_res;
		rot_y = panel->y_res;
	} else {
		rot_x = panel->y_res;
		rot_y = panel->x_res;
	}
	if (plane != 0 || channel_out != 0 || pos_x != 0 || pos_y != 0 ||
	    width != rot_x || height != rot_y) {
		DBGPRINT(1, "invalid plane params plane %d pos_x %d "
			"pos_y %d w %d h %d\n", plane, pos_x, pos_y,
			width, height);
		return -EINVAL;
	}

	omap_lcdc.frame_offset = offset;
	omap_lcdc.screen_width = screen_width;
	omap_lcdc.color_mode = color_mode;

	switch (color_mode) {
	case OMAPFB_COLOR_CLUT_8BPP:
		omap_lcdc.bpp = 8;
		omap_lcdc.palette_code = 0x3000;
		omap_lcdc.palette_size = 512;
		break;
	case OMAPFB_COLOR_RGB565:
		omap_lcdc.bpp = 16;
		omap_lcdc.palette_code = 0x4000;
		omap_lcdc.palette_size = 32;
		break;
	default:
		/* FIXME: other BPPs.
		 * bpp1: code  0,     size 256
		 * bpp2: code  0x1000 size 256
		 * bpp4: code  0x2000 size 256
		 * bpp12: code 0x4000 size 32
		 */
		DBGPRINT(1, "invalid color mode %d\n", color_mode);
		return -1;
	}

	omap_lcdc.palette_org = PAGE_ALIGN(MAX_PALETTE_SIZE) -
					omap_lcdc.palette_size;

	if (omap_lcdc.update_mode == OMAPFB_AUTO_UPDATE) {
		disable_controller();
		omap_stop_lcd_dma();
		setup_lcd_dma();
		enable_controller();
	}

	DBGLEAVE(1);

	return 0;
}

static int omap_lcdc_enable_plane(int plane, int enable)
{
	if (plane != 0 || enable != 1)
		return -EINVAL;

	return 0;
}

/* Configure the LCD DMA for a palette load operation and do the palette
 * downloading synchronously. We don't use the frame+palette load mode of
 * the controller, since the palette can always be downloaded seperately.
 */
static void load_palette(void)
{
	u16	*palette;

	DBGENTER(1);

	palette = (u16 *)((u8 *)omap_lcdc.vram_virt + omap_lcdc.palette_org);

	*(u16 *)palette &= 0x0fff;
	*(u16 *)palette |= omap_lcdc.palette_code;

	omap_set_lcd_dma_b1(omap_lcdc.vram_phys + omap_lcdc.palette_org,
		omap_lcdc.palette_size / 4 + 1, 1, OMAP_DMA_DATA_TYPE_S32);

	omap_set_lcd_dma_single_transfer(1);
	omap_setup_lcd_dma();

	init_completion(&omap_lcdc.palette_load_complete);
	enable_irqs(OMAP_LCDC_IRQ_LOADED_PALETTE);
	set_load_mode(OMAP_LCDC_LOAD_PALETTE);
	enable_controller();
	if (!wait_for_completion_timeout(&omap_lcdc.palette_load_complete,
				msecs_to_jiffies(500)))
		pr_err("timeout waiting for FRAME DONE\n");
	/* The controller gets disabled in the irq handler */
	disable_irqs(OMAP_LCDC_IRQ_LOADED_PALETTE);
	omap_stop_lcd_dma();

	DBGLEAVE(1);
}

static void calc_ck_div(int is_tft, int pck, int *pck_div)
{
	unsigned long lck;

	pck = max(1, pck);
	lck = clk_get_rate(omap_lcdc.lcd_ck);
	*pck_div = lck / pck;
	if (is_tft)
		*pck_div = max(2, *pck_div);
	else
		*pck_div = max(3, *pck_div);
	if (*pck_div > 255) {
		/* FIXME: try to adjust logic clock divider as well */
		*pck_div = 255;
		printk(KERN_WARNING MODULE_NAME ": pixclock %d kHz too low.\n",
				pck / 1000);
	}
}

static void inline setup_regs(void)
{
	u32 l;
	struct lcd_panel *panel = omap_lcdc.fbdev->panel;
	int is_tft = panel->config & OMAP_LCDC_PANEL_TFT;
	unsigned long lck;
	int pcd;

	l = omap_readl(OMAP_LCDC_CONTROL);
	l &= ~OMAP_LCDC_CTRL_LCD_TFT;
	l |= is_tft ? OMAP_LCDC_CTRL_LCD_TFT : 0;
#ifdef CONFIG_MACH_OMAP_PALMTE
/* FIXME:if (machine_is_omap_palmte()) { */
		/* PalmTE uses alternate TFT setting in 8BPP mode */
		l |= (is_tft && panel->bpp == 8) ? 0x810000 : 0;
/*	} */
#endif
	omap_writel(l, OMAP_LCDC_CONTROL);

	l = omap_readl(OMAP_LCDC_TIMING2);
	l &= ~(((1 << 6) - 1) << 20);
	l |= (panel->config & OMAP_LCDC_SIGNAL_MASK) << 20;
	omap_writel(l, OMAP_LCDC_TIMING2);

	l = panel->x_res - 1;
	l |= (panel->hsw - 1) << 10;
	l |= (panel->hfp - 1) << 16;
	l |= (panel->hbp - 1) << 24;
	omap_writel(l, OMAP_LCDC_TIMING0);

	l = panel->y_res - 1;
	l |= (panel->vsw - 1) << 10;
	l |= panel->vfp << 16;
	l |= panel->vbp << 24;
	omap_writel(l, OMAP_LCDC_TIMING1);

	l = omap_readl(OMAP_LCDC_TIMING2);
	l &= ~0xff;

	lck = clk_get_rate(omap_lcdc.lcd_ck);

	if (!panel->pcd)
		calc_ck_div(is_tft, panel->pixel_clock * 1000, &pcd);
	else {
		printk(KERN_WARNING
		    MODULE_NAME ": Pixel clock divider value is obsolete.\n"
		    MODULE_NAME ": Try to set pixel_clock to %lu and pcd to 0 "
		    "in drivers/video/omap/lcd_%s.c and submit a patch.\n",
			lck / panel->pcd / 1000, panel->name);

		pcd = panel->pcd;
	}
	l |= pcd & 0xff;
	l |= panel->acb << 8;
	omap_writel(l, OMAP_LCDC_TIMING2);

	/* update panel info with the exact clock */
	panel->pixel_clock = lck / pcd / 1000;
}

/* Configure the LCD controller, download the color palette and start a looped
 * DMA transfer of the frame image data. */
static int omap_lcdc_set_update_mode(enum omapfb_update_mode mode)
{
	int r = 0;

	DBGENTER(1);

	if (mode != omap_lcdc.update_mode) {
		switch (mode) {
		case OMAPFB_AUTO_UPDATE:
			setup_regs();
			load_palette();

			/* Setup and start LCD DMA */
			setup_lcd_dma();

			set_load_mode(OMAP_LCDC_LOAD_FRAME);
			enable_irqs(OMAP_LCDC_IRQ_DONE);
			/* This will start the actual DMA transfer */
			enable_controller();
			omap_lcdc.update_mode = mode;
			break;
		case OMAPFB_UPDATE_DISABLED:
			disable_controller();
			omap_stop_lcd_dma();
			omap_lcdc.update_mode = mode;
			break;
		default:
			r = -EINVAL;
		}
	}

	DBGLEAVE(1);
	return r;
}

static enum omapfb_update_mode omap_lcdc_get_update_mode(void)
{
	return omap_lcdc.update_mode;
}

static void omap_lcdc_suspend(void)
{
	if (omap_lcdc.update_mode == OMAPFB_AUTO_UPDATE) {
		disable_controller();
		omap_stop_lcd_dma();
	}
}

static void omap_lcdc_resume(void)
{
	if (omap_lcdc.update_mode == OMAPFB_AUTO_UPDATE) {
		setup_regs();
		load_palette();
		setup_lcd_dma();
		set_load_mode(OMAP_LCDC_LOAD_FRAME);
		enable_irqs(OMAP_LCDC_IRQ_DONE);
		enable_controller();
	}
}

static void omap_lcdc_get_vram_layout(unsigned long *size, void **virt,
					dma_addr_t *phys)
{
	*size = omap_lcdc.vram_size - PAGE_ALIGN(MAX_PALETTE_SIZE);
	*virt = (u8 *)omap_lcdc.vram_virt + PAGE_ALIGN(MAX_PALETTE_SIZE);
	*phys = omap_lcdc.vram_phys + PAGE_ALIGN(MAX_PALETTE_SIZE);
}

static int omap_lcdc_init(struct omapfb_device *fbdev, int ext_mode,
			  int req_vram_size)
{
	int r;
	u32 l;
	int rate;
	struct clk *tc_ck;
	struct lcd_panel *panel = fbdev->panel;
	int frame_size;

	DBGENTER(1);

	omap_lcdc.irq_mask = 0;

	omap_lcdc.fbdev = fbdev;

	pr_info(MODULE_NAME ": init\n");

	l = 0;
	omap_writel(l, OMAP_LCDC_CONTROL);

	/* FIXME:
	 * According to errata some platforms have a clock rate limitiation
	 */
	omap_lcdc.lcd_ck = clk_get(NULL, "lcd_ck");
	if (IS_ERR(omap_lcdc.lcd_ck)) {
		pr_err("unable to access LCD clock\n");
		r = PTR_ERR(omap_lcdc.lcd_ck);
		goto fail0;
	}

	tc_ck = clk_get(NULL, "tc_ck");
	if (IS_ERR(tc_ck)) {
		pr_err("unable to access TC clock\n");
		r = PTR_ERR(tc_ck);
		goto fail1;
	}

	rate = clk_get_rate(tc_ck);
	clk_put(tc_ck);

	if (machine_is_omap_h3())
		rate /= 3;
	r = clk_set_rate(omap_lcdc.lcd_ck, rate);
	if (r) {
		pr_err("failed to adjust LCD rate\n");
		goto fail1;
	}
	clk_use(omap_lcdc.lcd_ck);

	r = request_irq(OMAP_LCDC_IRQ, lcdc_irq_handler, 0, "omap-lcdc",
			omap_lcdc.fbdev);
	if (r) {
		pr_err("unable to get IRQ\n");
		goto fail2;
	}

	r = omap_request_lcd_dma(NULL, NULL);
	if (r) {
		pr_err("unable to get LCD DMA\n");
		goto fail3;
	}

	frame_size = panel->x_res * panel->bpp * panel->y_res / 8;
	if (req_vram_size > frame_size)
		frame_size = req_vram_size;
	omap_lcdc.vram_size = PAGE_ALIGN(MAX_PALETTE_SIZE) + frame_size;
	omap_lcdc.vram_virt = dma_alloc_writecombine(fbdev->dev,
			omap_lcdc.vram_size, &omap_lcdc.vram_phys, GFP_KERNEL);

	if (omap_lcdc.vram_virt == NULL) {
		pr_err("unable to allocate fb DMA memory\n");
		r = -ENOMEM;
		goto fail4;
	}

	DBGLEAVE(1);
	return 0;
fail4:
	omap_free_lcd_dma();
fail3:
	free_irq(OMAP_LCDC_IRQ, omap_lcdc.fbdev);
fail2:
	clk_unuse(omap_lcdc.lcd_ck);
fail1:
	clk_put(omap_lcdc.lcd_ck);
fail0:
	DBGLEAVE(1);
        return r;
}

static void omap_lcdc_cleanup(void)
{
	dma_free_writecombine(omap_lcdc.fbdev->dev, omap_lcdc.vram_size,
			      omap_lcdc.vram_virt, omap_lcdc.vram_phys);
	omap_free_lcd_dma();
	free_irq(OMAP_LCDC_IRQ, omap_lcdc.fbdev);
	clk_unuse(omap_lcdc.lcd_ck);
	clk_put(omap_lcdc.lcd_ck);
}

static unsigned long omap_lcdc_get_caps(void)
{
	return 0;
}

static int omap_lcdc_setcolreg(u_int regno, u16 red, u16 green, u16 blue,
			       u16 transp, int update_hw_pal)
{
	u16 *palette;

	if (omap_lcdc.color_mode != OMAPFB_COLOR_CLUT_8BPP || regno > 255)
		return -EINVAL;

	palette = (u16 *)((u8*)omap_lcdc.vram_virt +
			PAGE_ALIGN(MAX_PALETTE_SIZE) - omap_lcdc.palette_size);

	palette[regno] &= ~0x0fff;
	palette[regno] |= ((red >> 12) << 8) | ((green >> 12) << 4 ) |
			   (blue >> 12);

	if (update_hw_pal) {
		disable_controller();
		omap_stop_lcd_dma();
		load_palette();
		setup_lcd_dma();
		set_load_mode(OMAP_LCDC_LOAD_FRAME);
		enable_controller();
	}

	return 0;
}

struct lcd_ctrl omap1_int_ctrl = {
	.name			= "internal",
	.init			= omap_lcdc_init,
	.cleanup		= omap_lcdc_cleanup,
	.get_vram_layout	= omap_lcdc_get_vram_layout,
	.get_caps		= omap_lcdc_get_caps,
	.set_update_mode	= omap_lcdc_set_update_mode,
	.get_update_mode	= omap_lcdc_get_update_mode,
	.update_window		= NULL,
	.suspend		= omap_lcdc_suspend,
	.resume			= omap_lcdc_resume,
	.setup_plane		= omap_lcdc_setup_plane,
	.enable_plane		= omap_lcdc_enable_plane,
	.setcolreg		= omap_lcdc_setcolreg,
};

MODULE_DESCRIPTION("TI OMAP LCDC controller");
MODULE_LICENSE("GPL");
