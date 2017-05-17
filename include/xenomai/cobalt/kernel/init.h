/*
 * Copyright (C) 2013 Philippe Gerum <rpm@xenomai.org>.
 *
 * Xenomai is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version.
 *
 * Xenomai is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Xenomai; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 * 02111-1307, USA.
 */
#ifndef _COBALT_KERNEL_INIT_H
#define _COBALT_KERNEL_INIT_H

#include <linux/atomic.h>
#include <linux/notifier.h>
#include <cobalt/uapi/corectl.h>

extern atomic_t cobalt_runstate;

static inline enum cobalt_run_states realtime_core_state(void)
{
	return atomic_read(&cobalt_runstate);
}

static inline int realtime_core_enabled(void)
{
	return atomic_read(&cobalt_runstate) != COBALT_STATE_DISABLED;
}

static inline int realtime_core_running(void)
{
	return atomic_read(&cobalt_runstate) == COBALT_STATE_RUNNING;
}

static inline void set_realtime_core_state(enum cobalt_run_states state)
{
	atomic_set(&cobalt_runstate, state);
}

void cobalt_add_state_chain(struct notifier_block *nb);

void cobalt_remove_state_chain(struct notifier_block *nb);

void cobalt_call_state_chain(enum cobalt_run_states newstate);

#endif /* !_COBALT_KERNEL_INIT_H_ */
