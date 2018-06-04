/*
 * Copyright (C) 2006 Wolfgang Grandegger <wg@grandegger.com>
 *
 * Derived from RTnet project file stack/include/rtnet_internal.h:
 *
 * Copyright (C) 1999       Lineo, Inc
 *               1999, 2002 David A. Schleef <ds@schleef.org>
 *               2002       Ulrich Marx <marx@kammer.uni-hannover.de>
 *               2003-2005  Jan Kiszka <jan.kiszka@web.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __RTCAN_INTERNAL_H_
#define __RTCAN_INTERNAL_H_

#include <linux/module.h>
#include <rtdm/driver.h>

#ifdef CONFIG_XENO_DRIVERS_CAN_DEBUG
#define RTCAN_ASSERT(expr, func) \
    if (!(expr)) { \
	rtdm_printk("Assertion failed! %s:%s:%d %s\n", \
	__FILE__, __FUNCTION__, __LINE__, (#expr)); \
	func \
    }
#else
#define RTCAN_ASSERT(expr, func)
#endif /* CONFIG_RTCAN_CHECKED */

#ifdef CONFIG_XENO_DRIVERS_CAN_DEBUG
# define RTCAN_DBG(fmt,args...) do { printk(fmt ,##args); } while (0)
# define RTCAN_RTDM_DBG(fmt,args...) do { rtdm_printk(fmt ,##args); } while (0)
#else
# define RTCAN_DBG(fmt,args...) do {} while (0)
# define RTCAN_RTDM_DBG(fmt,args...) do {} while (0)
#endif

#define rtcan_priv(dev)			(dev)->priv
#define rtcandev_dbg(dev, fmt, args...)				\
	printk(KERN_DEBUG "%s: " fmt, (dev)->name, ##args)
#define rtcandev_info(dev, fmt, args...)			\
	printk(KERN_INFO "%s: " fmt, (dev)->name, ##args)
#define rtcandev_warn(dev, fmt, args...)			\
	printk(KERN_WARNING "%s: " fmt, (dev)->name, ##args)
#define rtcandev_err(dev, fmt, args...)				\
	printk(KERN_ERR "%s: " fmt, (dev)->name, ##args)

#endif /* __RTCAN_INTERNAL_H_ */
