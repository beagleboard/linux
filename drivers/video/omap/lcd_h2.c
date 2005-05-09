/*
 * File: drivers/video/omap_new/lcd-h2.c
 *
 * LCD panel support for the TI OMAP H2 board 
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

#include <asm/arch/mux.h>

#include "omapfb.h"

// #define OMAPFB_DBG 1

#include "debug.h"
#include "../drivers/ssi/omap-uwire.h"

#define TSC2101_UWIRE_CS	1

static int tsc2101_write_reg(int page, int reg, u16 data)
{
	u16	cmd;
	int	r;

	DBGENTER(1);

	cmd = ((page & 3) << 11) | ((reg & 0x3f) << 5);
	if (omap_uwire_data_transfer(TSC2101_UWIRE_CS, cmd, 16, 0, NULL, 1))
		r = -1;
	else
		r = omap_uwire_data_transfer(TSC2101_UWIRE_CS, data, 16, 0, NULL, 0);

	DBGLEAVE(1);
	return r;
}

static int h2_panel_init(struct lcd_panel *panel)
{
	unsigned long uwire_flags;
	DBGENTER(1);

	 /* Configure N15 pin to be uWire CS1 */
	omap_cfg_reg(N15_1610_UWIRE_CS1);
	uwire_flags = UWIRE_READ_RISING_EDGE | UWIRE_WRITE_RISING_EDGE;
	uwire_flags |= UWIRE_FREQ_DIV_8;
	omap_uwire_configure_mode(TSC2101_UWIRE_CS, uwire_flags);

	DBGLEAVE(1);
	return 0;
}

static void h2_panel_cleanup(struct lcd_panel *panel)
{
	DBGENTER(1);
	DBGLEAVE(1);
}

static int h2_panel_enable(struct lcd_panel *panel)
{
	int r;

	DBGENTER(1);

	/* Assert LCD_EN, BKLIGHT_EN pins on LCD panel
	 * page2, GPIO config reg, GPIO(0,1) to out and asserted
	 */
	r = tsc2101_write_reg(2, 0x23, 0xCC00) ? -1 : 0;

	DBGLEAVE(1);
	return r;
}

static void h2_panel_disable(struct lcd_panel *panel)
{
	DBGENTER(1);

	/* Deassert LCD_EN and BKLIGHT_EN pins on LCD panel
	 * page2, GPIO config reg, GPIO(0,1) to out and deasserted
	 */
	if (tsc2101_write_reg(2, 0x23, 0x8800))
		PRNERR("failed to disable LCD panel\n");

	DBGLEAVE(1);
}

static unsigned long h2_panel_get_caps(struct lcd_panel *panel)
{
	return 0;
}

static struct lcdc_video_mode mode240x320 = {
	.x_res = 240,
	.y_res = 320,
	.pixel_clock = 12500,
	.bpp = 16,
	.hsw = 12,
	.hfp = 14,
	.hbp = 72 - 12,
	.vsw = 1,
	.vfp = 1,
	.vbp = 0,
	.pcd = 12,
};

struct lcd_panel h2_panel = {
	.name       = "h2",
	.config     = LCD_PANEL_TFT,
	.video_mode = &mode240x320,
	
	.init	 = h2_panel_init,
	.cleanup = h2_panel_cleanup,
	.enable  = h2_panel_enable,
	.disable = h2_panel_disable,
	.get_caps= h2_panel_get_caps,
};

