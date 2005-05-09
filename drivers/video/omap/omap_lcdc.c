/*
 * linux/arch/arm/mach-omap/omap_lcdc.c
 *
 * OMAP internal LCD controller
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

#include <asm/arch/dma.h>
#include <asm/mach-types.h>
#include <asm/hardware/clock.h>

#include "omapfb.h"
#include "debug.h"

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

enum lcdc_load_mode {
	OMAP_LCDC_LOAD_PALETTE,
	OMAP_LCDC_LOAD_FRAME,
	OMAP_LCDC_LOAD_PALETTE_AND_FRAME
};

static struct omap_lcd_controller {
	enum fb_update_mode	update_mode;
	unsigned int		irq_mask;
	struct completion	last_frame_complete;
	struct completion	palette_load_complete;
	struct clk		*lcd_ck;
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

	init_completion(&omap_lcdc.last_frame_complete);
	l = omap_readl(OMAP_LCDC_CONTROL);
	mask = OMAP_LCDC_CTRL_LCD_EN | OMAP_LCDC_IRQ_MASK;
	/* Preserve the DONE mask, since we still want to get the
	 * final DONE irq. It will be disabled in the IRQ handler.
	 */
	mask &= ~OMAP_LCDC_IRQ_DONE;
	l &= ~mask;
	omap_writel(l, OMAP_LCDC_CONTROL);
}

static void last_frame_timeout(unsigned long data)
{
	printk(KERN_ERR "omap_lcdc: timeout waiting for DONE flag\n");
	complete(&omap_lcdc.last_frame_complete);
}

static void inline wait_for_frame_done(void)
{
	struct timer_list timeout;

	init_timer(&timeout);
	timeout.function = last_frame_timeout;
	timeout.expires = jiffies + msecs_to_jiffies(500);
	add_timer(&timeout);
	wait_for_completion(&omap_lcdc.last_frame_complete);
	del_timer_sync(&timeout);
}

static void disable_controller(void)
{
	disable_controller_async();
	wait_for_frame_done();
}

static void reset_controller(u32 status)
{
        static unsigned long reset_count = 0;
        static unsigned long last_jiffies = 0;

        disable_controller_async();
        reset_count++;
        if (reset_count == 1 ||
            time_after(jiffies, last_jiffies + HZ)) {
                printk(KERN_ERR "omap_lcdc: resetting "
                                 "(status %#010x,reset count %lu)\n",
                                  status, reset_count);
                last_jiffies = jiffies;
        }
        if (reset_count < 100) {
                enable_controller();
        } else {
                reset_count = 0;
                printk(KERN_ERR "omap_lcdc: too many reset attempts, "
                                "giving up.\n");
        }
}

/* Configure the LCD DMA according to the current mode specified by parameters
 * in fbdev and fbdev->var.
 */
static void setup_lcd_dma(struct omapfb_device *fbdev)
{
	static const int dma_elem_type[] = {
		0,
		OMAP_DMA_DATA_TYPE_S8,
		OMAP_DMA_DATA_TYPE_S16,
		0,
		OMAP_DMA_DATA_TYPE_S32,
	};
	struct fb_var_screeninfo *var = &fbdev->fb_info->var;
	unsigned long	src;
	int		esize, xelem, yelem;

	src = fbdev->lcddma_handle + fbdev->vis_frame_org + fbdev->view_org;
	switch (var->rotate) {
	case 0:
		esize = fbdev->mirror || (src & 3) ? 2 : 4;
		xelem = var->xres * var->bits_per_pixel / 8 / esize;
		yelem = var->yres;
		break;
	case 90:
	case 180:
	case 270:
		esize = 2;
		xelem = var->xres * var->bits_per_pixel / 16;
		yelem = var->yres;
		break;
	default:
		BUG();
		return;
	}
	DBGPRINT(1, "setup_dma: src=%#010x esize=%d xelem=%d yelem=%d\n",
		 src, esize, xelem, yelem);
	omap_set_lcd_dma_b1(src, xelem, yelem, dma_elem_type[esize]);
	omap_set_lcd_dma_single_transfer(0);
	if (!cpu_is_omap1510()) {
		/* Set virtual xres elem size */
		omap_set_lcd_dma_b1_vxres(
			fbdev->fb_info->fix.line_length / esize);
		/* Setup transformations */
		omap_set_lcd_dma_b1_rotation(var->rotate);
		omap_set_lcd_dma_b1_mirror(fbdev->mirror);
		omap_set_lcd_dma_b1_scale(fbdev->xscale, fbdev->yscale);
	}
	omap_setup_lcd_dma();
}

static irqreturn_t lcdc_irq_handler(int irq, void *dev_id,
					 struct pt_regs *fp)
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
static void omap_lcdc_change_mode(struct omapfb_device *fbdev)
{
	DBGENTER(1);

	omap_stop_lcd_dma();
	disable_controller();
	setup_lcd_dma(fbdev);
	enable_controller();

	DBGLEAVE(1);
}

/* Configure the LCD DMA for a palette load operation and do the palette
 * downloading synchronously. We don't use the frame+palette load mode of
 * the controller, since the palette can always be downloaded seperately.
 */
static void load_palette(struct omapfb_device *fbdev)
{
	u8		*palette;
	u32		code;
	unsigned long	size;
	unsigned long	palette_org;

	DBGENTER(1);

	switch (fbdev->panel->video_mode->bpp) {
	case 1:
		/* 0 is already set */
		code = 0;
		size = 256;
		break;
	case 2:
		code = 0x1000;
		size = 256;
		break;
	case 4:
		code = 0x2000;
		size = 256;
		break;
	case 8:
		code = 0x3000;
		size = 256;
		break;
	case 12:
	case 16:
		code = 0x4000;
		size = 32;
		break;
	default:
		BUG();
		return;
	}
	palette_org = MAX_PALETTE_SIZE - size;
	palette = fbdev->lcddma_base + palette_org;
	memset(palette, 0, size);
	*(u32 *)palette = code;

	omap_set_lcd_dma_b1(fbdev->lcddma_handle + palette_org,
			    size / 4 + 1, 1, OMAP_DMA_DATA_TYPE_S32);
	omap_set_lcd_dma_single_transfer(1);
	omap_setup_lcd_dma();

	init_completion(&omap_lcdc.palette_load_complete);
	enable_irqs(OMAP_LCDC_IRQ_LOADED_PALETTE);
	set_load_mode(OMAP_LCDC_LOAD_PALETTE);
	enable_controller();
	wait_for_completion(&omap_lcdc.palette_load_complete);
	/* The controller gets disabled in the irq handler */
	disable_irqs(OMAP_LCDC_IRQ_LOADED_PALETTE);
	omap_stop_lcd_dma();

	DBGLEAVE(1);
}

static void inline setup_regs(struct omapfb_device *fbdev)
{
	u32 l;
	struct lcdc_video_mode *mode = fbdev->panel->video_mode;
	int tft = fbdev->panel->config & LCD_PANEL_TFT;
	int signal_levels = fbdev->panel->signals;

	l = omap_readl(OMAP_LCDC_CONTROL);
	l &= ~OMAP_LCDC_CTRL_LCD_TFT;
	l |= tft ? OMAP_LCDC_CTRL_LCD_TFT : 0;
	omap_writel(l, OMAP_LCDC_CONTROL);

	l = omap_readl(OMAP_LCDC_TIMING2);
	l &= ~(((1 << 6) - 1) << 20);
	l |= signal_levels << 20;
	omap_writel(l, OMAP_LCDC_TIMING2);

	l = mode->x_res - 1;
	l |= (mode->hsw - 1) << 10;
	l |= (mode->hfp - 1) << 16;
	l |= (mode->hbp - 1) << 24;
	omap_writel(l, OMAP_LCDC_TIMING0);

	l = mode->y_res - 1;
	l |= (mode->vsw - 1) << 10;
	l |= mode->vfp << 16;
	l |= mode->vbp << 24;
	omap_writel(l, OMAP_LCDC_TIMING1);

	l = omap_readl(OMAP_LCDC_TIMING2);
	l &= ~0xff;

	if (!cpu_is_omap730())
		l |= mode->pcd;

	if (machine_is_omap_perseus2()) {
		u32 clock1, clock2;
		int pcd;

		clock1 = mode->pixel_clock * 1000;
		clock2 = clk_get_rate(omap_lcdc.lcd_ck);

		if (clock1 != 0) {
			pcd = clock2 / clock1;
			if (pcd > 255)
				pcd = 0;
		} else {
			pcd = 0;
		}
		
		if (pcd == 0)
			l |= mode->pcd;
		else
			l |= pcd;

//		printk("%% ck1: %d  ck2: %d  pcd: %d %%\n",clock1, clock2, pcd);	
	}

	l |= mode->acb << 8;
	if (mode->flags & OMAP_LCDC_INV_PIX_CLOCK)
		l |= 1 << 22;
	omap_writel(l, OMAP_LCDC_TIMING2);
}

/* Configure the LCD controller, download the color palette and start a looped
 * DMA transfer of the frame image data. */
static int omap_lcdc_set_update_mode(struct omapfb_device *fbdev,
				     enum fb_update_mode mode)
{
	int r = 0;

	DBGENTER(1);

	if (mode != omap_lcdc.update_mode) {
		switch (mode) {
		case FB_AUTO_UPDATE:
			setup_regs(fbdev);
			load_palette(fbdev);

			/* Setup and start LCD DMA */
			setup_lcd_dma(fbdev);

			set_load_mode(OMAP_LCDC_LOAD_FRAME);
			enable_irqs(OMAP_LCDC_IRQ_DONE);
			/* This will start the actual DMA transfer */
			enable_controller();
			omap_lcdc.update_mode = mode;
			break;
		case FB_UPDATE_DISABLED:
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

static enum fb_update_mode omap_lcdc_get_update_mode(struct omapfb_device *fbdev)
{
	return omap_lcdc.update_mode;
}

static void omap_lcdc_suspend(struct omapfb_device *fbdev)
{
	if (omap_lcdc.update_mode == FB_AUTO_UPDATE) {
		disable_controller();
		omap_stop_lcd_dma();
	}
}

static void omap_lcdc_resume(struct omapfb_device *fbdev)
{
	if (omap_lcdc.update_mode == FB_AUTO_UPDATE) {
		setup_regs(fbdev);
		load_palette(fbdev);
		setup_lcd_dma(fbdev);
		set_load_mode(OMAP_LCDC_LOAD_FRAME);
		enable_irqs(OMAP_LCDC_IRQ_DONE);
		enable_controller();
	}
}

static int omap_lcdc_init(struct omapfb_device *fbdev)
{
	int r;
	u32 l;
	int rate;
	struct clk *tc_ck;

	DBGENTER(1);

	omap_lcdc.irq_mask = 0;

	l = 0;
	omap_writel(l, OMAP_LCDC_CONTROL);

	/* FIXME:
	 * According to errata some platforms have a clock rate limitiation
	 */
	omap_lcdc.lcd_ck = clk_get(NULL, "lcd_ck");
	if (IS_ERR(omap_lcdc.lcd_ck)) {
		printk(KERN_ERR "omap_lcdc: unable to access LCD clock\n");
		r = PTR_ERR(omap_lcdc.lcd_ck);
		goto fail0;
	}

	tc_ck = clk_get(NULL, "tc_ck");
	if (IS_ERR(tc_ck)) {
		printk(KERN_ERR "omap_lcdc: unable to access TC clock\n");
		r = PTR_ERR(tc_ck);
		goto fail1;
	}

	rate = clk_get_rate(tc_ck);
	clk_put(tc_ck);

	if (machine_is_omap_h3())
		rate /= 3;
	r = clk_set_rate(omap_lcdc.lcd_ck, rate);
	if (r) {
		printk(KERN_ERR "omap_lcdc: failed to adjust LCD rate\n");
		goto fail1;
	}
	clk_use(omap_lcdc.lcd_ck);

	r = request_irq(OMAP_LCDC_IRQ, lcdc_irq_handler, 0, "omap-lcdc", fbdev);
	if (r) {
		printk(KERN_ERR "omap_lcdc: unable to get IRQ\n");
		goto fail2;
	}

	r = omap_request_lcd_dma(NULL, NULL);
	if (r) {
		printk(KERN_ERR "omap_lcdc: unable to get LCD DMA\n");
		goto fail3;
	}

	printk(KERN_INFO "OMAP LCD controller initialized.\n");
	DBGLEAVE(1);
	return 0;
fail3:
	free_irq(OMAP_LCDC_IRQ, fbdev);
fail2:
	clk_unuse(omap_lcdc.lcd_ck);
fail1:
	clk_put(omap_lcdc.lcd_ck);
fail0:
	DBGLEAVE(1);
        return r;
}

static void omap_lcdc_cleanup(struct omapfb_device *fbdev)
{
	omap_free_lcd_dma();
	free_irq(OMAP_LCDC_IRQ, fbdev);
	clk_unuse(omap_lcdc.lcd_ck);
	clk_put(omap_lcdc.lcd_ck);
}

static void omap_lcdc_get_mem_layout(struct omapfb_device *fbdev,
				     unsigned long *size, unsigned long *fb_org)
{
	struct lcdc_video_mode *mode = fbdev->panel->video_mode;

	*size = MAX_PALETTE_SIZE;
	*fb_org = *size;
	*size += mode->x_res * mode->bpp / 8 * mode->y_res;
}

static unsigned long omap_lcdc_get_caps(struct omapfb_device *fbdev)
{
	return 0;
}

struct lcd_ctrl omapfb_lcdc_ctrl = {
	.name			= "internal",
	.init			= omap_lcdc_init,
	.cleanup		= omap_lcdc_cleanup,
	.get_mem_layout		= omap_lcdc_get_mem_layout,
	.get_caps		= omap_lcdc_get_caps,
	.set_update_mode	= omap_lcdc_set_update_mode,
	.get_update_mode	= omap_lcdc_get_update_mode,
	.update_window		= NULL,
	.suspend		= omap_lcdc_suspend, 
	.resume			= omap_lcdc_resume,
	.change_mode		= omap_lcdc_change_mode,
};

MODULE_DESCRIPTION("TI OMAP LCDC controller");
MODULE_LICENSE("GPL");
