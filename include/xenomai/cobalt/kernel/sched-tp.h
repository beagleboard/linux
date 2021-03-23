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
#ifndef _COBALT_KERNEL_SCHED_TP_H
#define _COBALT_KERNEL_SCHED_TP_H

#ifndef _COBALT_KERNEL_SCHED_H
#error "please don't include cobalt/kernel/sched-tp.h directly"
#endif

/**
 * @addtogroup cobalt_core_sched
 * @{
 */

#ifdef CONFIG_XENO_OPT_SCHED_TP

#define XNSCHED_TP_MIN_PRIO	1
#define XNSCHED_TP_MAX_PRIO	255
#define XNSCHED_TP_NR_PRIO	\
	(XNSCHED_TP_MAX_PRIO - XNSCHED_TP_MIN_PRIO + 1)

extern struct xnsched_class xnsched_class_tp;

struct xnsched_tp_window {
	xnticks_t w_offset;
	int w_part;
};

struct xnsched_tp_schedule {
	int pwin_nr;
	xnticks_t tf_duration;
	atomic_t refcount;
	struct xnsched_tp_window pwins[0];
};

struct xnsched_tp {
	struct xnsched_tpslot {
		/** Per-partition runqueue. */
		xnsched_queue_t runnable;
	} partitions[CONFIG_XENO_OPT_SCHED_TP_NRPART];
	/** Idle slot for passive windows. */
	struct xnsched_tpslot idle;
	/** Active partition slot */
	struct xnsched_tpslot *tps;
	/** Time frame timer */
	struct xntimer tf_timer;
	/** Global partition schedule */
	struct xnsched_tp_schedule *gps;
	/** Window index of next partition */
	int wnext;
	/** Start of next time frame */
	xnticks_t tf_start;
	/** Assigned thread queue */
	struct list_head threads;
};

static inline int xnsched_tp_init_thread(struct xnthread *thread)
{
	thread->tps = NULL;

	return 0;
}

struct xnsched_tp_schedule *
xnsched_tp_set_schedule(struct xnsched *sched,
			struct xnsched_tp_schedule *gps);

void xnsched_tp_start_schedule(struct xnsched *sched);

void xnsched_tp_stop_schedule(struct xnsched *sched);

int xnsched_tp_get_partition(struct xnsched *sched);

struct xnsched_tp_schedule *
xnsched_tp_get_schedule(struct xnsched *sched);

void xnsched_tp_put_schedule(struct xnsched_tp_schedule *gps);

#endif /* CONFIG_XENO_OPT_SCHED_TP */

/** @} */

#endif /* !_COBALT_KERNEL_SCHED_TP_H */
