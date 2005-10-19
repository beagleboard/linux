/*
 * File: drivers/video/omap/lcd-h4.c
 *
 * LCD panel support for the TI OMAP H4 board
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

#include <asm/arch/omapfb.h>

/* #define OMAPFB_DBG 1 */

#include "debug.h"

static int h4_panel_init(struct omapfb_device *fbdev)
{
	DBGENTER(1);
	DBGLEAVE(1);
	return 0;
}

static void h4_panel_cleanup(void)
{
	DBGENTER(1);
	DBGLEAVE(1);
}

static int h4_panel_enable(void)
{

	DBGENTER(1);
	DBGLEAVE(1);
	return 0;
}

static void h4_panel_disable(void)
{
	DBGENTER(1);
	DBGLEAVE(1);
}

static unsigned long h4_panel_get_caps(void)
{
	return 0;
}

struct lcd_panel h4_panel = {
	.name		= "h4",
	.config		= OMAP_LCDC_PANEL_TFT,

	.bpp		= 16,
	.data_lines	= 16,
	.x_res		= 240,
	.y_res		= 320,
	.pixel_clock	= 6250,
	.hsw		= 15,
	.hfp		= 15,
	.hbp		= 60,
	.vsw		= 1,
	.vfp		= 1,
	.vbp		= 1,

	.init		= h4_panel_init,
	.cleanup	= h4_panel_cleanup,
	.enable		= h4_panel_enable,
	.disable	= h4_panel_disable,
	.get_caps	= h4_panel_get_caps,
};

