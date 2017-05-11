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
#ifndef _COBALT_KERNEL_SCHED_WEAK_H
#define _COBALT_KERNEL_SCHED_WEAK_H

#ifndef _COBALT_KERNEL_SCHED_H
#error "please don't include cobalt/kernel/sched-weak.h directly"
#endif

/**
 * @addtogroup cobalt_core_sched
 * @{
 */

#ifdef CONFIG_XENO_OPT_SCHED_WEAK

#define XNSCHED_WEAK_MIN_PRIO	0
#define XNSCHED_WEAK_MAX_PRIO	99
#define XNSCHED_WEAK_NR_PRIO	\
	(XNSCHED_WEAK_MAX_PRIO - XNSCHED_WEAK_MIN_PRIO + 1)

#if XNSCHED_WEAK_NR_PRIO > XNSCHED_CLASS_WEIGHT_FACTOR ||	\
	(defined(CONFIG_XENO_OPT_SCALABLE_SCHED) &&		\
	 XNSCHED_WEAK_NR_PRIO > XNSCHED_MLQ_LEVELS)
#error "WEAK class has too many priority levels"
#endif

extern struct xnsched_class xnsched_class_weak;

struct xnsched_weak {
	xnsched_queue_t runnable;	/*!< Runnable thread queue. */
};

static inline int xnsched_weak_init_thread(struct xnthread *thread)
{
	return 0;
}

#endif /* CONFIG_XENO_OPT_SCHED_WEAK */

/** @} */

#endif /* !_COBALT_KERNEL_SCHED_WEAK_H */
