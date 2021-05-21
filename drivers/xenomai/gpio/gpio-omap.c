/**
 * @note Copyright (C) 2020 Greg Gallagher <greg@embeddedgreg.com>
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

#define RTDM_SUBCLASS_OMAP  6

static const char *compat_array[] = {
	"ti,omap4-gpio",
	"ti,omap3-gpio",
	"ti,omap2-gpio",
};

static int __init omap_gpio_init(void)
{
	return rtdm_gpiochip_scan_array_of(NULL, compat_array,
					   ARRAY_SIZE(compat_array),
					   RTDM_SUBCLASS_OMAP);
}
module_init(omap_gpio_init);

static void __exit omap_gpio_exit(void)
{
	rtdm_gpiochip_remove_of(RTDM_SUBCLASS_OMAP);
}
module_exit(omap_gpio_exit);

MODULE_LICENSE("GPL");
