/**
 * @note Copyright (C) 2016 Philippe Gerum <rpm@xenomai.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#include <linux/module.h>
#include "gpio-core.h"

#define RTDM_SUBCLASS_MXC  2

static int __init mxc_gpio_init(void)
{
	return rtdm_gpiochip_scan_of(NULL, "fsl,imx6q-gpio",
				     RTDM_SUBCLASS_MXC);
}
module_init(mxc_gpio_init);

static void __exit mxc_gpio_exit(void)
{
	rtdm_gpiochip_remove_of(RTDM_SUBCLASS_MXC);
}
module_exit(mxc_gpio_exit);

MODULE_LICENSE("GPL");
