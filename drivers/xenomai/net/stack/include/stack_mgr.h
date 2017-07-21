/***
 *
 *  stack_mgr.h
 *
 *  RTnet - real-time networking subsystem
 *  Copyright (C) 2002      Ulrich Marx <marx@fet.uni-hannover.de>
 *                2003-2006 Jan Kiszka <jan.kiszka@web.de>
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

#ifndef __STACK_MGR_H_
#define __STACK_MGR_H_

#ifdef __KERNEL__

#include <linux/list.h>

#include <rtnet_internal.h>
#include <rtdev.h>


/***
 * network layer protocol (layer 3)
 */

#define RTPACKET_HASH_TBL_SIZE  64
#define RTPACKET_HASH_KEY_MASK  (RTPACKET_HASH_TBL_SIZE-1)

struct rtpacket_type {
    struct list_head    list_entry;

    unsigned short      type;
    short               refcount;

    int                 (*handler)(struct rtskb *, struct rtpacket_type *);
    int                 (*err_handler)(struct rtskb *, struct rtnet_device *,
                                       struct rtpacket_type *);
    bool                (*trylock)(struct rtpacket_type *);
    void                (*unlock)(struct rtpacket_type *);

    struct module	*owner;
};


int __rtdev_add_pack(struct rtpacket_type *pt, struct module *module);
#define rtdev_add_pack(pt) \
    __rtdev_add_pack(pt, THIS_MODULE)

void rtdev_remove_pack(struct rtpacket_type *pt);

static inline bool rtdev_lock_pack(struct rtpacket_type *pt)
{
    return try_module_get(pt->owner);
}

static inline void rtdev_unlock_pack(struct rtpacket_type *pt)
{
    module_put(pt->owner);
}

void rt_stack_connect(struct rtnet_device *rtdev, struct rtnet_mgr *mgr);
void rt_stack_disconnect(struct rtnet_device *rtdev);

#if IS_ENABLED(CONFIG_XENO_DRIVERS_NET_DRV_LOOPBACK)
void rt_stack_deliver(struct rtskb *rtskb);
#endif /* CONFIG_XENO_DRIVERS_NET_DRV_LOOPBACK */

int rt_stack_mgr_init(struct rtnet_mgr *mgr);
void rt_stack_mgr_delete(struct rtnet_mgr *mgr);

void rtnetif_rx(struct rtskb *skb);

static inline void rtnetif_tx(struct rtnet_device *rtdev)
{
}

static inline void rt_mark_stack_mgr(struct rtnet_device *rtdev)
{
    rtdm_event_signal(rtdev->stack_event);
}

#endif /* __KERNEL__ */

#endif  /* __STACK_MGR_H_ */
