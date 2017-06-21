/***
 *
 *  ipv4/tcp/timerwheel.h - timerwheel interface for RTnet
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

#ifndef __TIMERWHEEL_H_
#define __TIMERWHEEL_H_

#include <linux/list.h>
#include <rtnet.h>

#define TIMERWHEEL_TIMER_UNUSED    -1

typedef void (*timerwheel_timer_handler)(void *);

struct timerwheel_timer {
    struct list_head            link;
    timerwheel_timer_handler    handler;
    void                        *data;
    int                         slot;
    volatile int                refcount; /* only written by wheel task */
};

static inline void
timerwheel_init_timer(struct timerwheel_timer *timer,
                      timerwheel_timer_handler handler, void *data)
{
    timer->slot     = TIMERWHEEL_TIMER_UNUSED;
    timer->handler  = handler;
    timer->data     = data;
    timer->refcount = 0;
}

/* passed data must remain valid till a timer fireup */
int timerwheel_add_timer(struct timerwheel_timer *timer,
                         nanosecs_rel_t expires);

int timerwheel_remove_timer(struct timerwheel_timer *timer);

void timerwheel_remove_timer_sync(struct timerwheel_timer *timer);

int timerwheel_init(nanosecs_rel_t timeout,
                    unsigned int granularity);

void timerwheel_cleanup(void);

#endif
