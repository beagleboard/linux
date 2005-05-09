/*
 * File: drivers/video/omap_new/lcd-osk.c
 *
 * LCD panel support for the TI OMAP OSK board 
 *
 * Copyright (C) 2004 Nokia Corporation
 * Author: Imre Deak <imre.deak@nokia.com>
 * Adapted for OSK by <dirk.behme@de.bosch.com>
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
#include <asm/arch/mux.h>

#include "omapfb.h"

// #define OMAPFB_DBG 1

#include "debug.h"

static int osk_panel_init(struct lcd_panel *panel)
{
	DBGENTER(1);
	DBGLEAVE(1);
	return 0;
}

static void osk_panel_cleanup(struct lcd_panel *panel)
{
	DBGENTER(1);
	DBGLEAVE(1);
}

static int osk_panel_enable(struct lcd_panel *panel)
{
	DBGENTER(1);

	/* configure PWL pin */
	omap_cfg_reg(PWL);

	/* Enable PWL unit */
	omap_writeb(0x01, OMAP16XX_PWL_CLK_ENABLE);

	/* Set PWL level */
	omap_writeb(0xFF, OMAP16XX_PWL_ENABLE);

	/* configure GPIO2 as output */
	omap_set_gpio_direction(2, 0);

	/* set GPIO2 high */
	omap_set_gpio_dataout(2, 1);

	DBGLEAVE(1);
	return 0;
}

static void osk_panel_disable(struct lcd_panel *panel)
{
	DBGENTER(1);

	/* Set PWL level to zero */
	omap_writeb(0x00, OMAP16XX_PWL_ENABLE);

	/* Disable PWL unit */
	omap_writeb(0x00, OMAP16XX_PWL_CLK_ENABLE);

	/* set GPIO2 low */
	omap_set_gpio_dataout(2, 0);

	DBGLEAVE(1);
}

static unsigned long osk_panel_get_caps(struct lcd_panel *panel)
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

struct lcd_panel osk_panel = {
	.name       = "osk",
	.config     = LCD_PANEL_TFT,
	.video_mode = &mode240x320,

	.init	 = osk_panel_init,
	.cleanup = osk_panel_cleanup,
	.enable  = osk_panel_enable,
	.disable = osk_panel_disable,
	.get_caps= osk_panel_get_caps,
};

