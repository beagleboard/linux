/*
 * Copyright (C) 2009 Philippe Gerum <rpm@xenomai.org>.
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
#ifndef _COBALT_KERNEL_SCHED_SPORADIC_H
#define _COBALT_KERNEL_SCHED_SPORADIC_H

#ifndef _COBALT_KERNEL_SCHED_H
#error "please don't include cobalt/kernel/sched-sporadic.h directly"
#endif

/**
 * @addtogroup cobalt_core_sched
 * @{
 */

#ifdef CONFIG_XENO_OPT_SCHED_SPORADIC

#define XNSCHED_SPORADIC_MIN_PRIO	1
#define XNSCHED_SPORADIC_MAX_PRIO	255
#define XNSCHED_SPORADIC_NR_PRIO	\
	(XNSCHED_SPORADIC_MAX_PRIO - XNSCHED_SPORADIC_MIN_PRIO + 1)

extern struct xnsched_class xnsched_class_sporadic;

struct xnsched_sporadic_repl {
	xnticks_t date;
	xnticks_t amount;
};

struct xnsched_sporadic_data {
	xnticks_t resume_date;
	xnticks_t budget;
	int repl_in;
	int repl_out;
	int repl_pending;
	struct xntimer repl_timer;
	struct xntimer drop_timer;
	struct xnsched_sporadic_repl repl_data[CONFIG_XENO_OPT_SCHED_SPORADIC_MAXREPL];
	struct xnsched_sporadic_param param;
	struct xnthread *thread;
};

struct xnsched_sporadic {
#if XENO_DEBUG(COBALT)
	unsigned long drop_retries;
#endif
};

static inline int xnsched_sporadic_init_thread(struct xnthread *thread)
{
	thread->pss = NULL;

	return 0;
}

#endif /* !CONFIG_XENO_OPT_SCHED_SPORADIC */

/** @} */

#endif /* !_COBALT_KERNEL_SCHED_SPORADIC_H */
