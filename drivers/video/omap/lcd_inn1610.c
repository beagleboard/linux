/*
 * File: drivers/video/omap_new/lcd-inn1610.c
 *
 * LCD panel support for the TI OMAP1610 Innovator board 
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

#include "omapfb.h"

// #define OMAPFB_DBG 1

#include "debug.h"

static int innovator1610_panel_init(struct lcd_panel *panel)
{
	int r = 0;

	DBGENTER(1);

	if (omap_request_gpio(14)) {
		PRNERR("can't request GPIO 14\n");
		r = -1;
		goto exit;
	}
	if (omap_request_gpio(15)) {
		PRNERR("can't request GPIO 15\n");
		omap_free_gpio(14);
		r = -1;
		goto exit;
	}
	/* configure GPIO(14, 15) as outputs */
	omap_set_gpio_direction(14, 0);
	omap_set_gpio_direction(15, 0);
exit:
	DBGLEAVE(1);
	return r;
}

static void innovator1610_panel_cleanup(struct lcd_panel *panel)
{
	DBGENTER(1);

	omap_free_gpio(15);
	omap_free_gpio(14);

	DBGLEAVE(1);
}

static int innovator1610_panel_enable(struct lcd_panel *panel)
{
	DBGENTER(1);

	/* set GPIO14 and GPIO15 high */
	omap_set_gpio_dataout(14, 1);
	omap_set_gpio_dataout(15, 1);

	DBGLEAVE(1);
	return 0;
}

static void innovator1610_panel_disable(struct lcd_panel *panel)
{
	DBGENTER(1);

	/* set GPIO13, GPIO14 and GPIO15 low */
	omap_set_gpio_dataout(14, 0);
	omap_set_gpio_dataout(15, 0);

	DBGLEAVE(1);
}

static unsigned long innovator1610_panel_get_caps(struct lcd_panel *panel)
{
	return 0;
}

static struct lcdc_video_mode mode320x240 = {
	.x_res = 320,
	.y_res = 240,
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

struct lcd_panel innovator1610_panel = {
	.name       = "inn1610",
	.config	    = LCD_PANEL_TFT,
	.video_mode = &mode320x240,
	
	.init	 = innovator1610_panel_init,
	.cleanup = innovator1610_panel_cleanup,
	.enable	 = innovator1610_panel_enable,
	.disable = innovator1610_panel_disable,
	.get_caps= innovator1610_panel_get_caps,
};

