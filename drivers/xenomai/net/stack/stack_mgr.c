/***
 *
 *  stack/stack_mgr.c - Stack-Manager
 *
 *  Copyright (C) 2002 Ulrich Marx <marx@kammer.uni-hannover.de>
 *  Copyright (C) 2003-2006 Jan Kiszka <jan.kiszka@web.de>
 *  Copyright (C) 2006 Jorge Almeida <j-almeida@criticalsoftware.com>
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

#include <linux/moduleparam.h>

#include <rtdev.h>
#include <rtnet_internal.h>
#include <rtskb_fifo.h>
#include <stack_mgr.h>

static unsigned int stack_mgr_prio = RTNET_DEF_STACK_PRIORITY;
module_param(stack_mgr_prio, uint, 0444);
MODULE_PARM_DESC(stack_mgr_prio, "Priority of the stack manager task");

#if (CONFIG_XENO_DRIVERS_NET_RX_FIFO_SIZE &                                    \
     (CONFIG_XENO_DRIVERS_NET_RX_FIFO_SIZE - 1)) != 0
#error CONFIG_XENO_DRIVERS_NET_RX_FIFO_SIZE must be power of 2!
#endif
static DECLARE_RTSKB_FIFO(rx, CONFIG_XENO_DRIVERS_NET_RX_FIFO_SIZE);

struct list_head rt_packets[RTPACKET_HASH_TBL_SIZE];
#ifdef CONFIG_XENO_DRIVERS_NET_ETH_P_ALL
struct list_head rt_packets_all;
#endif /* CONFIG_XENO_DRIVERS_NET_ETH_P_ALL */
DEFINE_RTDM_LOCK(rt_packets_lock);

/***
 *  rtdev_add_pack:         add protocol (Layer 3)
 *  @pt:                    the new protocol
 */
int __rtdev_add_pack(struct rtpacket_type *pt, struct module *module)
{
	int ret = 0;
	rtdm_lockctx_t context;

	INIT_LIST_HEAD(&pt->list_entry);
	pt->refcount = 0;
	if (pt->trylock == NULL)
		pt->trylock = rtdev_lock_pack;
	if (pt->unlock == NULL)
		pt->unlock = rtdev_unlock_pack;
	pt->owner = module;

	rtdm_lock_get_irqsave(&rt_packets_lock, context);

	if (pt->type == htons(ETH_P_ALL))
#ifdef CONFIG_XENO_DRIVERS_NET_ETH_P_ALL
		list_add_tail(&pt->list_entry, &rt_packets_all);
#else /* !CONFIG_XENO_DRIVERS_NET_ETH_P_ALL */
		ret = -EINVAL;
#endif /* CONFIG_XENO_DRIVERS_NET_ETH_P_ALL */
	else
		list_add_tail(
			&pt->list_entry,
			&rt_packets[ntohs(pt->type) & RTPACKET_HASH_KEY_MASK]);

	rtdm_lock_put_irqrestore(&rt_packets_lock, context);

	return ret;
}

EXPORT_SYMBOL_GPL(__rtdev_add_pack);

/***
 *  rtdev_remove_pack:  remove protocol (Layer 3)
 *  @pt:                protocol
 */
void rtdev_remove_pack(struct rtpacket_type *pt)
{
	rtdm_lockctx_t context;

	RTNET_ASSERT(pt != NULL, return;);

	rtdm_lock_get_irqsave(&rt_packets_lock, context);
	list_del(&pt->list_entry);
	rtdm_lock_put_irqrestore(&rt_packets_lock, context);
}

EXPORT_SYMBOL_GPL(rtdev_remove_pack);

/***
 *  rtnetif_rx: will be called from the driver interrupt handler
 *  (IRQs disabled!) and send a message to rtdev-owned stack-manager
 *
 *  @skb - the packet
 */
void rtnetif_rx(struct rtskb *skb)
{
	RTNET_ASSERT(skb != NULL, return;);
	RTNET_ASSERT(skb->rtdev != NULL, return;);

	if (unlikely(rtskb_fifo_insert_inirq(&rx.fifo, skb) < 0)) {
		rtdm_printk("RTnet: dropping packet in %s()\n", __FUNCTION__);
		kfree_rtskb(skb);
	}
}

EXPORT_SYMBOL_GPL(rtnetif_rx);

#if IS_ENABLED(CONFIG_XENO_DRIVERS_NET_DRV_LOOPBACK)
#define __DELIVER_PREFIX
#else /* !CONFIG_XENO_DRIVERS_NET_DRV_LOOPBACK */
#define __DELIVER_PREFIX static inline
#endif /* CONFIG_XENO_DRIVERS_NET_DRV_LOOPBACK */

__DELIVER_PREFIX void rt_stack_deliver(struct rtskb *rtskb)
{
	unsigned short hash;
	struct rtpacket_type *pt_entry;
	rtdm_lockctx_t context;
	struct rtnet_device *rtdev = rtskb->rtdev;
	int err;
	int eth_p_all_hit = 0;

	rtcap_report_incoming(rtskb);

	rtskb->nh.raw = rtskb->data;

	rtdm_lock_get_irqsave(&rt_packets_lock, context);

#ifdef CONFIG_XENO_DRIVERS_NET_ETH_P_ALL
	eth_p_all_hit = 0;
	list_for_each_entry (pt_entry, &rt_packets_all, list_entry) {
		if (!pt_entry->trylock(pt_entry))
			continue;
		rtdm_lock_put_irqrestore(&rt_packets_lock, context);

		pt_entry->handler(rtskb, pt_entry);

		rtdm_lock_get_irqsave(&rt_packets_lock, context);
		pt_entry->unlock(pt_entry);
		eth_p_all_hit = 1;
	}
#endif /* CONFIG_XENO_DRIVERS_NET_ETH_P_ALL */

	hash = ntohs(rtskb->protocol) & RTPACKET_HASH_KEY_MASK;

	list_for_each_entry (pt_entry, &rt_packets[hash], list_entry)
		if (pt_entry->type == rtskb->protocol) {
			if (!pt_entry->trylock(pt_entry))
				continue;
			rtdm_lock_put_irqrestore(&rt_packets_lock, context);

			err = pt_entry->handler(rtskb, pt_entry);

			rtdm_lock_get_irqsave(&rt_packets_lock, context);
			pt_entry->unlock(pt_entry);

			if (likely(!err)) {
				rtdm_lock_put_irqrestore(&rt_packets_lock,
							 context);
				return;
			}
		}

	rtdm_lock_put_irqrestore(&rt_packets_lock, context);

	/* Don't warn if ETH_P_ALL listener were present or when running in
       promiscuous mode (RTcap). */
	if (unlikely(!eth_p_all_hit && !(rtdev->flags & IFF_PROMISC)))
		rtdm_printk("RTnet: no one cared for packet with layer 3 "
			    "protocol type 0x%04x\n",
			    ntohs(rtskb->protocol));

	kfree_rtskb(rtskb);
}

#if IS_ENABLED(CONFIG_XENO_DRIVERS_NET_DRV_LOOPBACK)
EXPORT_SYMBOL_GPL(rt_stack_deliver);
#endif /* CONFIG_XENO_DRIVERS_NET_DRV_LOOPBACK */

static void rt_stack_mgr_task(void *arg)
{
	rtdm_event_t *mgr_event = &((struct rtnet_mgr *)arg)->event;
	struct rtskb *rtskb;

	while (!rtdm_task_should_stop()) {
		if (rtdm_event_wait(mgr_event) < 0)
			break;

		/* we are the only reader => no locking required */
		while ((rtskb = __rtskb_fifo_remove(&rx.fifo)))
			rt_stack_deliver(rtskb);
	}
}

/***
 *  rt_stack_connect
 */
void rt_stack_connect(struct rtnet_device *rtdev, struct rtnet_mgr *mgr)
{
	rtdev->stack_event = &mgr->event;
}

EXPORT_SYMBOL_GPL(rt_stack_connect);

/***
 *  rt_stack_disconnect
 */
void rt_stack_disconnect(struct rtnet_device *rtdev)
{
	rtdev->stack_event = NULL;
}

EXPORT_SYMBOL_GPL(rt_stack_disconnect);

/***
 *  rt_stack_mgr_init
 */
int rt_stack_mgr_init(struct rtnet_mgr *mgr)
{
	int i;

	rtskb_fifo_init(&rx.fifo, CONFIG_XENO_DRIVERS_NET_RX_FIFO_SIZE);

	for (i = 0; i < RTPACKET_HASH_TBL_SIZE; i++)
		INIT_LIST_HEAD(&rt_packets[i]);
#ifdef CONFIG_XENO_DRIVERS_NET_ETH_P_ALL
	INIT_LIST_HEAD(&rt_packets_all);
#endif /* CONFIG_XENO_DRIVERS_NET_ETH_P_ALL */

	rtdm_event_init(&mgr->event, 0);

	return rtdm_task_init(&mgr->task, "rtnet-stack", rt_stack_mgr_task, mgr,
			      stack_mgr_prio, 0);
}

/***
 *  rt_stack_mgr_delete
 */
void rt_stack_mgr_delete(struct rtnet_mgr *mgr)
{
	rtdm_event_destroy(&mgr->event);
	rtdm_task_destroy(&mgr->task);
}
