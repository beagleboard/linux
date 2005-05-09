/*
 * File: drivers/video/omap_new/lcd-inn1510.c
 *
 * LCD panel support for the TI OMAP1510 Innovator board
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

#include <linux/module.h>

#include <asm/io.h>

#include <asm/arch/fpga.h>

#include "omapfb.h"

// #define OMAPFB_DBG 1

#include "debug.h"

static int innovator1510_panel_init(struct lcd_panel *panel)
{
	DBGENTER(1);
	DBGLEAVE(1);
	return 0;
}

static void innovator1510_panel_cleanup(struct lcd_panel *panel)
{
	DBGENTER(1);
	DBGLEAVE(1);
}

static int innovator1510_panel_enable(struct lcd_panel *panel)
{
	DBGENTER(1);

	fpga_write(0x7, OMAP1510_FPGA_LCD_PANEL_CONTROL);

	DBGLEAVE(1);
	return 0;
}

static void innovator1510_panel_disable(struct lcd_panel *panel)
{
	DBGENTER(1);

	fpga_write(0x0, OMAP1510_FPGA_LCD_PANEL_CONTROL);

	DBGLEAVE(1);
}

static unsigned long innovator1510_panel_get_caps(struct lcd_panel *panel)
{
	return 0;
}

static struct lcdc_video_mode mode240x320 = {
	.x_res = 240,
	.y_res = 320,
	.pixel_clock = 12500,
	.bpp = 16,
	.hsw = 40,
	.hfp = 40,
	.hbp = 72,
	.vsw = 1,
	.vfp = 1,
	.vbp = 0,
	.pcd = 12,
};

struct lcd_panel innovator1510_panel = {
	.name       = "inn1510",
	.config	    = LCD_PANEL_TFT,
	.video_mode = &mode240x320,

	.init	 = innovator1510_panel_init,
	.cleanup = innovator1510_panel_cleanup,
	.enable  = innovator1510_panel_enable,
	.disable = innovator1510_panel_disable,
	.get_caps= innovator1510_panel_get_caps,
};

