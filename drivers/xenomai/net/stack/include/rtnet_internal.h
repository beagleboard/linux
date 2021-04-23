/***
 *
 *  rtnet_internal.h - internal declarations
 *
 *  RTnet - real-time networking subsystem
 *  Copyright (C) 1999       Lineo, Inc
 *                1999, 2002 David A. Schleef <ds@schleef.org>
 *                2002       Ulrich Marx <marx@kammer.uni-hannover.de>
 *                2003-2005  Jan Kiszka <jan.kiszka@web.de>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef __RTNET_INTERNAL_H_
#define __RTNET_INTERNAL_H_

#include <linux/module.h>
#include <linux/mutex.h>
#include <rtdm/driver.h>

#ifdef CONFIG_XENO_DRIVERS_NET_CHECKED
#define RTNET_ASSERT(expr, func)                                               \
	if (!(expr)) {                                                         \
		rtdm_printk("Assertion failed! %s:%s:%d %s\n", __FILE__,       \
			    __FUNCTION__, __LINE__, (#expr));                  \
		func                                                           \
	}
#else
#define RTNET_ASSERT(expr, func)
#endif /* CONFIG_XENO_DRIVERS_NET_CHECKED */

/* some configurables */

#define RTNET_DEF_STACK_PRIORITY                                               \
	RTDM_TASK_HIGHEST_PRIORITY + RTDM_TASK_LOWER_PRIORITY
/*#define RTNET_RTDEV_PRIORITY        5*/

struct rtnet_device;

/*struct rtnet_msg {
    int                 msg_type;
    struct rtnet_device *rtdev;
};*/

struct rtnet_mgr {
	rtdm_task_t task;
	/*    MBX     mbx;*/
	rtdm_event_t event;
};

extern struct rtnet_mgr STACK_manager;
extern struct rtnet_mgr RTDEV_manager;

extern const char rtnet_rtdm_provider_name[];

#ifdef CONFIG_XENO_OPT_VFILE
extern struct xnvfile_directory rtnet_proc_root;
#endif /* CONFIG_XENO_OPT_VFILE */

extern struct class *rtnet_class;

#endif /* __RTNET_INTERNAL_H_ */
