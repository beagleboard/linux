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
#ifndef _RTDM_GPIO_CORE_H
#define _RTDM_GPIO_CORE_H

#include <linux/list.h>
#include <rtdm/driver.h>
#include <rtdm/uapi/gpio.h>

struct class;
struct device_node;

struct rtdm_gpio_chip {
	struct gpio_chip *gc;
	struct rtdm_driver driver;
	struct class *devclass;
	struct list_head pins;
	struct list_head next;
	rtdm_lock_t lock;
};

int rtdm_gpiochip_add(struct rtdm_gpio_chip *rgc,
		      struct gpio_chip *gc,
		      int gpio_subclass);

void rtdm_gpiochip_remove(struct rtdm_gpio_chip *rgc);

int rtdm_gpiochip_add_by_name(struct rtdm_gpio_chip *rgc,
			      const char *label, int gpio_subclass);

#ifdef CONFIG_OF

int rtdm_gpiochip_scan_of(struct device_node *from,
			  const char *compat, int type);

void rtdm_gpiochip_remove_of(int type);

extern struct list_head rtdm_gpio_chips;

#endif

#endif /* !_RTDM_GPIO_CORE_H */
