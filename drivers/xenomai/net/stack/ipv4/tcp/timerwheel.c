/***
 *
 *  ipv4/tcp/timerwheel.c - timerwheel implementation for RTnet
 *
 *  Copyright (C) 2009 Vladimir Zapolskiy <vladimir.zapolskiy@siemens.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License, version 2, as
 *  published by the Free Software Foundation.
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

#include <linux/delay.h>
#include <linux/slab.h>

#include <rtdm/driver.h>
#include "timerwheel.h"

static struct {
	/* timer pivot task */
	rtdm_task_t pivot_task;

	/* time length for one period of rotation of timerwheel */
	nanosecs_rel_t timeout;

	/* timer wheel slots for storing timers up to timerwheel_timeout */
	unsigned int slots;

	/* timer wheel interval timeout */
	nanosecs_rel_t interval;

	/* timer wheel interval timeout */
	unsigned int interval_base;

	/* timerwheel array */
	struct list_head *ring;

	/* timerwheel slot counter */
	unsigned int current_slot;

	/* timerwheel current slot lock */
	rtdm_lock_t slot_lock;
} wheel;

static struct timerwheel_timer *timerwheel_get_from_current_slot(void)
{
	struct timerwheel_timer *timer = NULL;
	struct list_head *slot_list;
	rtdm_lockctx_t context;

	rtdm_lock_get_irqsave(&wheel.slot_lock, context);

	slot_list = &wheel.ring[wheel.current_slot];

	if (!list_empty(slot_list)) {
		timer = list_first_entry(slot_list, struct timerwheel_timer,
					 link);
		list_del(&timer->link);
		timer->slot = TIMERWHEEL_TIMER_UNUSED;
		timer->refcount++;
	}

	rtdm_lock_put_irqrestore(&wheel.slot_lock, context);

	return timer;
}

int timerwheel_add_timer(struct timerwheel_timer *timer, nanosecs_rel_t expires)
{
	rtdm_lockctx_t context;
	int slot;

	slot = expires >> wheel.interval_base;

	if (slot >= wheel.slots)
		return -EINVAL;

	rtdm_lock_get_irqsave(&wheel.slot_lock, context);

	/* cancel timer if it's still running */
	if (timer->slot >= 0)
		list_del(&timer->link);

	slot = slot + wheel.current_slot;
	if (slot >= wheel.slots)
		slot = slot - wheel.slots;

	list_add_tail(&timer->link, &wheel.ring[slot]);
	timer->slot = slot;

	rtdm_lock_put_irqrestore(&wheel.slot_lock, context);

	return 0;
}

static int timerwheel_sleep(void)
{
	int ret;

	ret = rtdm_task_sleep(wheel.interval);
	if (ret < 0)
		return ret;

	wheel.current_slot++;
	if (wheel.current_slot == wheel.slots)
		wheel.current_slot = 0;

	return 0;
}

static void timerwheel_pivot(void *arg)
{
	struct timerwheel_timer *timer;
	int ret;

	while (1) {
		ret = timerwheel_sleep();
		if (ret < 0) {
			rtdm_printk(
				"timerwheel: timerwheel_pivot interrupted %d\n",
				-ret);
			break;
		}

		while ((timer = timerwheel_get_from_current_slot())) {
			timer->handler(timer->data);

			smp_mb();
			timer->refcount--;
		}
	}
}

int timerwheel_remove_timer(struct timerwheel_timer *timer)
{
	rtdm_lockctx_t context;
	int ret;

	rtdm_lock_get_irqsave(&wheel.slot_lock, context);

	if (timer->slot >= 0) {
		list_del(&timer->link);
		timer->slot = TIMERWHEEL_TIMER_UNUSED;
		ret = 0;
	} else
		ret = -ENOENT;

	rtdm_lock_put_irqrestore(&wheel.slot_lock, context);

	return ret;
}

void timerwheel_remove_timer_sync(struct timerwheel_timer *timer)
{
	u64 interval_ms = wheel.interval;

	do_div(interval_ms, 1000000);

	timerwheel_remove_timer(timer);

	while (timer->refcount > 0)
		msleep(interval_ms);
}

/*
  timeout     - maximum expiration timeout for timers
  granularity - is an exponent of 2 representing nanoseconds for
  one wheel tick
  heapsize    - is a number of timers to allocate
*/
int __init timerwheel_init(nanosecs_rel_t timeout, unsigned int granularity)
{
	int i;
	int err;

	/* the least possible slot timeout is set for 1ms */
	if (granularity < 10)
		return -EINVAL;

	wheel.timeout = timeout;
	wheel.interval_base = granularity;
	wheel.slots = (timeout >> granularity) + 1;
	wheel.interval = (1 << granularity);
	wheel.current_slot = 0;

	wheel.ring =
		kmalloc(sizeof(struct list_head) * wheel.slots, GFP_KERNEL);
	if (!wheel.ring)
		return -ENOMEM;

	for (i = 0; i < wheel.slots; i++)
		INIT_LIST_HEAD(&wheel.ring[i]);

	rtdm_lock_init(&wheel.slot_lock);

	err = rtdm_task_init(&wheel.pivot_task, "rttcp timerwheel",
			     timerwheel_pivot, NULL, 1, 0);
	if (err) {
		printk("timerwheel: error on pivot task initialization: %d\n",
		       err);
		kfree(wheel.ring);
	}

	return err;
}

void timerwheel_cleanup(void)
{
	rtdm_task_destroy(&wheel.pivot_task);
	kfree(wheel.ring);
}
