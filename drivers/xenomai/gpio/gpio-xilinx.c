/**
 * @note Copyright (C) 2017 Greg Gallagher <greg@embeddedgreg.com>
 *
 * This driver controls the gpio that can be located on the PL
 * of the Zynq SOC
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

#define RTDM_SUBCLASS_XILINX  5

static int __init xilinx_gpio_init(void)
{
	return rtdm_gpiochip_scan_of(NULL, "xlnx,xps-gpio-1.00.a",
                     RTDM_SUBCLASS_XILINX);
}
module_init(xilinx_gpio_init);

static void __exit xilinx_gpio_exit(void)
{
	rtdm_gpiochip_remove_of(RTDM_SUBCLASS_XILINX);
}
module_exit(xilinx_gpio_exit);

MODULE_LICENSE("GPL");

