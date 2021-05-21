/**
 * Copyright (C) 2016 Philippe Gerum <rpm@xenomai.org>
 *
 * Xenomai is free software; you can redistribute it and/or
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
#ifndef _COBALT_RTDM_GPIO_H
#define _COBALT_RTDM_GPIO_H

#include <linux/list.h>
#include <rtdm/driver.h>
#include <rtdm/uapi/gpio.h>

struct class;
struct device_node;
struct gpio_desc;

struct rtdm_gpio_pin {
	struct rtdm_device dev;
	struct list_head next;
	rtdm_irq_t irqh;
	rtdm_event_t event;
	char *name;
	struct gpio_desc *desc;
	nanosecs_abs_t timestamp;
};

struct rtdm_gpio_chip {
	struct gpio_chip *gc;
	struct rtdm_driver driver;
	struct class *devclass;
	struct list_head next;
	rtdm_lock_t lock;
	struct rtdm_gpio_pin pins[0];
};

int rtdm_gpiochip_add(struct rtdm_gpio_chip *rgc,
		      struct gpio_chip *gc,
		      int gpio_subclass);

struct rtdm_gpio_chip *
rtdm_gpiochip_alloc(struct gpio_chip *gc,
		    int gpio_subclass);

void rtdm_gpiochip_remove(struct rtdm_gpio_chip *rgc);

int rtdm_gpiochip_add_by_name(struct rtdm_gpio_chip *rgc,
			      const char *label, int gpio_subclass);

int rtdm_gpiochip_post_event(struct rtdm_gpio_chip *rgc,
			     unsigned int offset);

#ifdef CONFIG_OF

int rtdm_gpiochip_scan_of(struct device_node *from,
			  const char *compat, int type);

int rtdm_gpiochip_scan_array_of(struct device_node *from,
				const char *compat[],
				int nentries, int type);

void rtdm_gpiochip_remove_of(int type);

#endif

#endif /* !_COBALT_RTDM_GPIO_H */
