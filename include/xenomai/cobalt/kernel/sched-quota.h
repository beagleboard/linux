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
#ifndef _COBALT_KERNEL_SCHED_QUOTA_H
#define _COBALT_KERNEL_SCHED_QUOTA_H

#ifndef _COBALT_KERNEL_SCHED_H
#error "please don't include cobalt/kernel/sched-quota.h directly"
#endif

/**
 * @addtogroup cobalt_core_sched
 * @{
 */

#ifdef CONFIG_XENO_OPT_SCHED_QUOTA

#define XNSCHED_QUOTA_MIN_PRIO	1
#define XNSCHED_QUOTA_MAX_PRIO	255
#define XNSCHED_QUOTA_NR_PRIO	\
	(XNSCHED_QUOTA_MAX_PRIO - XNSCHED_QUOTA_MIN_PRIO + 1)

extern struct xnsched_class xnsched_class_quota;

struct xnsched_quota_group {
	struct xnsched *sched;
	xnticks_t quota_ns;
	xnticks_t quota_peak_ns;
	xnticks_t run_start_ns;
	xnticks_t run_budget_ns;
	xnticks_t run_credit_ns;
	struct list_head members;
	struct list_head expired;
	struct list_head next;
	int nr_active;
	int nr_threads;
	int tgid;
	int quota_percent;
	int quota_peak_percent;
};

struct xnsched_quota {
	xnticks_t period_ns;
	struct xntimer refill_timer;
	struct xntimer limit_timer;
	struct list_head groups;
};

static inline int xnsched_quota_init_thread(struct xnthread *thread)
{
	thread->quota = NULL;
	INIT_LIST_HEAD(&thread->quota_expired);

	return 0;
}

int xnsched_quota_create_group(struct xnsched_quota_group *tg,
			       struct xnsched *sched,
			       int *quota_sum_r);

int xnsched_quota_destroy_group(struct xnsched_quota_group *tg,
				int force,
				int *quota_sum_r);

void xnsched_quota_set_limit(struct xnsched_quota_group *tg,
			     int quota_percent, int quota_peak_percent,
			     int *quota_sum_r);

struct xnsched_quota_group *
xnsched_quota_find_group(struct xnsched *sched, int tgid);

int xnsched_quota_sum_all(struct xnsched *sched);

#endif /* !CONFIG_XENO_OPT_SCHED_QUOTA */

/** @} */

#endif /* !_COBALT_KERNEL_SCHED_QUOTA_H */
