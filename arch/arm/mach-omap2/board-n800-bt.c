/*
 * Nokia N800 platform-specific data for Bluetooth
 *
 * Copyright (C) 2005, 2006 Nokia Corporation
 * Contact: Ville Tervo <ville.tervo@nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <mach/board.h>
#include <mach/board-nokia.h>

static struct platform_device n800_bt_device = {
	.name           = "hci_h4p",
	.id             = -1,
	.num_resources  = 0,
};

void __init n800_bt_init(void)
{
	const struct omap_bluetooth_config *bt_config;

	bt_config = (void *) omap_get_config(OMAP_TAG_NOKIA_BT,
					     struct omap_bluetooth_config);
	n800_bt_device.dev.platform_data = (void *) bt_config;
	if (platform_device_register(&n800_bt_device) < 0)
		BUG();
}

