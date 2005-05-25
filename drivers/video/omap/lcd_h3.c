/*
 * File: drivers/video/omap_new/lcd-h3.c
 *
 * LCD panel support for the TI OMAP H3 board 
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

#include <asm/arch/gpio.h>
#include <asm/arch/tps65010.h>

#include "omapfb.h"

// #define OMAPFB_DBG 1

#include "debug.h"

static int h3_panel_init(struct lcd_panel *panel)
{
	DBGENTER(1);
	DBGLEAVE(1);
	return 0;
}

static void h3_panel_cleanup(struct lcd_panel *panel)
{
	DBGENTER(1);
	DBGLEAVE(1);
}

static int h3_panel_enable(struct lcd_panel *panel)
{
	int r = 0;

	DBGENTER(1);

	/* GPIO1 and GPIO2 of TPS65010 send LCD_ENBKL and LCD_ENVDD signals */
	r = tps65010_set_gpio_out_value(GPIO1, HIGH);
	if (!r)
		r = tps65010_set_gpio_out_value(GPIO2, HIGH);
	if (r)
		PRNERR("Unable to turn on LCD panel\n");

	DBGLEAVE(1);
	return r;
}

static void h3_panel_disable(struct lcd_panel *panel)
{
	int r = 0;

	DBGENTER(1);

	/* GPIO1 and GPIO2 of TPS65010 send LCD_ENBKL and LCD_ENVDD signals */
	r = tps65010_set_gpio_out_value(GPIO1, LOW);
	if (!r)
		tps65010_set_gpio_out_value(GPIO2, LOW);
	if (r)
		PRNERR("Unable to turn off LCD panel\n");

	DBGLEAVE(1);
}

static unsigned long h3_panel_get_caps(struct lcd_panel *panel)
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
	.pcd = 2,
};

struct lcd_panel h3_panel = {
	.name       = "h3",
	.config     = LCD_PANEL_TFT,
	.video_mode = &mode240x320,

	.init	 = h3_panel_init,
	.cleanup = h3_panel_cleanup,
	.enable  = h3_panel_enable,
	.disable = h3_panel_disable,
	.get_caps= h3_panel_get_caps,
};

