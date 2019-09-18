/*
 * Copyright (C) 2006 Wolfgang Grandegger <wg@grandegger.com>
 *
 * Derived from RTnet project file stack/rtdev.c:
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

#include <linux/spinlock.h>
#include <linux/if.h>
#include <linux/if_arp.h>
#include <linux/netdevice.h>
#include <linux/module.h>

#include "rtcan_internal.h"
#include "rtcan_dev.h"


static struct rtcan_device *rtcan_devices[RTCAN_MAX_DEVICES];
static DEFINE_RTDM_LOCK(rtcan_devices_rt_lock);

static int rtcan_global_init_done;

DEFINE_SEMAPHORE(rtcan_devices_nrt_lock);

/* Spinlock for all reception lists and also for some members in
 * struct rtcan_socket */
rtdm_lock_t rtcan_socket_lock;

/* Spinlock for all reception lists and also for some members in
 * struct rtcan_socket */
rtdm_lock_t rtcan_recv_list_lock;



static inline void rtcan_global_init(void)
{
    if (!rtcan_global_init_done) {
	rtdm_lock_init(&rtcan_socket_lock);
	rtdm_lock_init(&rtcan_recv_list_lock);
	rtcan_global_init_done = 1;
    }
}


static inline struct rtcan_device *__rtcan_dev_get_by_name(const char *name)
{
    int i;
    struct rtcan_device *dev;


    for (i = 0; i < RTCAN_MAX_DEVICES; i++) {
	dev = rtcan_devices[i];
	if ((dev != NULL) && (strncmp(dev->name, name, IFNAMSIZ) == 0))
	    return dev;
    }
    return NULL;
}


struct rtcan_device *rtcan_dev_get_by_name(const char *name)
{
    struct rtcan_device *dev;
#ifdef RTCAN_USE_REFCOUNT
    rtdm_lockctx_t context;
#endif


#ifdef RTCAN_USE_REFCOUNT
    rtdm_lock_get_irqsave(&rtcan_devices_rt_lock, context);
#endif

    dev = __rtcan_dev_get_by_name(name);

#ifdef RTCAN_USE_REFCOUNT
    if (dev != NULL)
	atomic_inc(&dev->refcount);
    rtdm_lock_put_irqrestore(&rtcan_devices_rt_lock, context);
#endif

    return dev;
}


static inline struct rtcan_device *__rtcan_dev_get_by_index(int ifindex)
{
    return rtcan_devices[ifindex - 1];
}


struct rtcan_device *rtcan_dev_get_by_index(int ifindex)
{
    struct rtcan_device *dev;
#ifdef RTCAN_USE_REFCOUNT
    rtdm_lockctx_t context;
#endif


    if ((ifindex <= 0) || (ifindex > RTCAN_MAX_DEVICES))
	return NULL;

#ifdef RTCAN_USE_REFCOUNT
    rtdm_lock_get_irqsave(&rtcan_devices_rt_lock, context);
#endif

    dev = __rtcan_dev_get_by_index(ifindex);

#ifdef RTCAN_USE_REFCOUNT
    if (dev != NULL)
	atomic_inc(&dev->refcount);
    rtdm_lock_put_irqrestore(&rtcan_devices_rt_lock, context);
#endif

    return dev;
}


void rtcan_dev_alloc_name(struct rtcan_device *dev, const char *mask)
{
    char buf[IFNAMSIZ];
    struct rtcan_device *tmp;
    int i;


    for (i = 0; i < RTCAN_MAX_DEVICES; i++) {
	ksformat(buf, IFNAMSIZ, mask, i);
	if ((tmp = rtcan_dev_get_by_name(buf)) == NULL) {
	    strncpy(dev->name, buf, IFNAMSIZ);
	    break;
	}
#ifdef RTCAN_USE_REFCOUNT
	else
	    rtcan_dev_dereference(tmp);
#endif
    }
}


struct rtcan_device *rtcan_dev_alloc(int sizeof_priv, int sizeof_board_priv)
{
    struct rtcan_device *dev;
    struct rtcan_recv *recv_list_elem;
    int alloc_size;
    int j;


    alloc_size = sizeof(*dev) + sizeof_priv + sizeof_board_priv;

    dev = (struct rtcan_device *)kmalloc(alloc_size, GFP_KERNEL);
    if (dev == NULL) {
	printk(KERN_ERR "rtcan: cannot allocate rtcan device\n");
	return NULL;
    }

    memset(dev, 0, alloc_size);

    sema_init(&dev->nrt_lock, 1);

    rtdm_lock_init(&dev->device_lock);

    /* Init TX Semaphore, will be destroyed forthwith
     * when setting stop mode */
    rtdm_sem_init(&dev->tx_sem, 0);
#ifdef RTCAN_USE_REFCOUNT
    atomic_set(&dev->refcount, 0);
#endif

    /* Initialize receive list */
    dev->empty_list = recv_list_elem = dev->receivers;
    for (j = 0; j < RTCAN_MAX_RECEIVERS - 1; j++, recv_list_elem++)
	recv_list_elem->next = recv_list_elem + 1;
    recv_list_elem->next = NULL;
    dev->free_entries = RTCAN_MAX_RECEIVERS;

    if (sizeof_priv)
	dev->priv = (void *)((unsigned long)dev + sizeof(*dev));
    if (sizeof_board_priv)
	dev->board_priv = (void *)((unsigned long)dev + sizeof(*dev) + sizeof_priv);

    return dev;
}

void rtcan_dev_free (struct rtcan_device *dev)
{
    if (dev != NULL) {
	rtdm_sem_destroy(&dev->tx_sem);
	kfree(dev);
    }
}


static inline int __rtcan_dev_new_index(void)
{
    int i;


    for (i = 0; i < RTCAN_MAX_DEVICES; i++)
	if (rtcan_devices[i] == NULL)
	     return i+1;

    return -ENOMEM;
}


int rtcan_dev_register(struct rtcan_device *dev)
{
    rtdm_lockctx_t context;
    int ret;

    down(&rtcan_devices_nrt_lock);

    rtcan_global_init();

    if ((ret = __rtcan_dev_new_index()) < 0) {
	up(&rtcan_devices_nrt_lock);
	return ret;
    }
    dev->ifindex = ret;

    if (strchr(dev->name,'%') != NULL)
	rtcan_dev_alloc_name(dev, dev->name);

    if (__rtcan_dev_get_by_name(dev->name) != NULL) {
	up(&rtcan_devices_nrt_lock);
	return -EEXIST;
    }

    rtdm_lock_get_irqsave(&rtcan_devices_rt_lock, context);

    rtcan_devices[dev->ifindex - 1] = dev;

    rtdm_lock_put_irqrestore(&rtcan_devices_rt_lock, context);
    rtcan_dev_create_proc(dev);

    up(&rtcan_devices_nrt_lock);

    printk("rtcan: registered %s\n", dev->name);

    return 0;
}


int rtcan_dev_unregister(struct rtcan_device *dev)
{
    rtdm_lockctx_t context;


    RTCAN_ASSERT(dev->ifindex != 0,
		 printk("RTCAN: device %s/%p was not registered\n",
			dev->name, dev); return -ENODEV;);

    /* If device is running, close it first. */
    if (CAN_STATE_OPERATING(dev->state))
	return -EBUSY;

    down(&rtcan_devices_nrt_lock);

    rtcan_dev_remove_proc(dev);

    rtdm_lock_get_irqsave(&rtcan_devices_rt_lock, context);

#ifdef RTCAN_USE_REFCOUNT
    while (atomic_read(&dev->refcount) > 0) {
	rtdm_lock_put_irqrestore(&rtcan_devices_rt_lock, context);
	up(&rtcan_devices_nrt_lock);

	RTCAN_DBG("RTCAN: unregistering %s deferred (refcount = %d)\n",
		  dev->name, atomic_read(&dev->refcount));
	set_current_state(TASK_UNINTERRUPTIBLE);
	schedule_timeout(1*HZ); /* wait a second */

	down(&rtcan_devices_nrt_lock);
	rtdm_lock_get_irqsave(&rtcan_devices_rt_lock, context);
    }
#endif
    rtcan_devices[dev->ifindex - 1] = NULL;

    rtdm_lock_put_irqrestore(&rtcan_devices_rt_lock, context);
    up(&rtcan_devices_nrt_lock);

#ifdef RTCAN_USE_REFCOUNT
    RTCAN_ASSERT(atomic_read(&dev->refcount) == 0,
		 printk("RTCAN: dev reference counter < 0!\n"););
#endif

    printk("RTCAN: unregistered %s\n", dev->name);

    return 0;
}


EXPORT_SYMBOL_GPL(rtcan_socket_lock);
EXPORT_SYMBOL_GPL(rtcan_recv_list_lock);

EXPORT_SYMBOL_GPL(rtcan_dev_free);

EXPORT_SYMBOL_GPL(rtcan_dev_alloc);
EXPORT_SYMBOL_GPL(rtcan_dev_alloc_name);

EXPORT_SYMBOL_GPL(rtcan_dev_register);
EXPORT_SYMBOL_GPL(rtcan_dev_unregister);

EXPORT_SYMBOL_GPL(rtcan_dev_get_by_name);
EXPORT_SYMBOL_GPL(rtcan_dev_get_by_index);
