/**
 * @note Copyright (C) 2017 Greg Gallagher <greg@embeddedgreg.com>
 * 
 * This driver is inspired by:
 * gpio-bcm2835.c, please see original file for copyright information
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
#include <rtdm/gpio.h>

#define RTDM_SUBCLASS_ZYNQ7000  4

static int __init zynq7000_gpio_init(void)
{
 	return rtdm_gpiochip_scan_of(NULL, "xlnx,zynq-gpio-1.0", 
                     RTDM_SUBCLASS_ZYNQ7000);
}
module_init(zynq7000_gpio_init);

static void __exit zynq7000_gpio_exit(void)
{
	rtdm_gpiochip_remove_of(RTDM_SUBCLASS_ZYNQ7000);
}
module_exit(zynq7000_gpio_exit);

MODULE_LICENSE("GPL");

