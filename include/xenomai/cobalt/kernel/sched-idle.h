/*
 * Copyright (C) 2008 Philippe Gerum <rpm@xenomai.org>.
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
#ifndef _COBALT_KERNEL_SCHED_IDLE_H
#define _COBALT_KERNEL_SCHED_IDLE_H

#ifndef _COBALT_KERNEL_SCHED_H
#error "please don't include cobalt/kernel/sched-idle.h directly"
#endif

/**
 * @addtogroup cobalt_core_sched
 * @{
 */

/* Idle priority level - actually never used for indexing. */
#define XNSCHED_IDLE_PRIO	-1

extern struct xnsched_class xnsched_class_idle;

static inline void __xnsched_idle_setparam(struct xnthread *thread,
					   const union xnsched_policy_param *p)
{
	xnthread_clear_state(thread, XNWEAK);
	thread->cprio = p->idle.prio;
}

static inline void __xnsched_idle_getparam(struct xnthread *thread,
					   union xnsched_policy_param *p)
{
	p->idle.prio = thread->cprio;
}

static inline void __xnsched_idle_trackprio(struct xnthread *thread,
					    const union xnsched_policy_param *p)
{
	if (p)
		__xnsched_idle_setparam(thread, p);
	else
		thread->cprio = XNSCHED_IDLE_PRIO;
}

static inline int xnsched_idle_init_thread(struct xnthread *thread)
{
	return 0;
}

/** @} */

#endif /* !_COBALT_KERNEL_SCHED_IDLE_H */
