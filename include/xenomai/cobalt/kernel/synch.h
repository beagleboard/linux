/*
 * Copyright (C) 2001,2002,2003 Philippe Gerum <rpm@xenomai.org>.
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
#ifndef _COBALT_KERNEL_SYNCH_H
#define _COBALT_KERNEL_SYNCH_H

#include <cobalt/kernel/list.h>
#include <cobalt/kernel/assert.h>
#include <cobalt/kernel/timer.h>
#include <cobalt/uapi/kernel/synch.h>

/**
 * @addtogroup cobalt_core_synch
 * @{
 */
#define XNSYNCH_CLAIMED 0x10	/* Claimed by other thread(s) w/ PIP */

/* Spare flags usable by upper interfaces */
#define XNSYNCH_SPARE0  0x01000000
#define XNSYNCH_SPARE1  0x02000000
#define XNSYNCH_SPARE2  0x04000000
#define XNSYNCH_SPARE3  0x08000000
#define XNSYNCH_SPARE4  0x10000000
#define XNSYNCH_SPARE5  0x20000000
#define XNSYNCH_SPARE6  0x40000000
#define XNSYNCH_SPARE7  0x80000000

/* Statuses */
#define XNSYNCH_DONE    0	/* Resource available / operation complete */
#define XNSYNCH_WAIT    1	/* Calling thread blocked -- start rescheduling */
#define XNSYNCH_RESCHED 2	/* Force rescheduling */

struct xnthread;
struct xnsynch;

struct xnsynch {
	struct list_head link;	/** thread->claimq */
	int wprio;		/** wait prio in claimq */
	unsigned long status;	 /** Status word */
	struct list_head pendq;	 /** Pending threads */
	struct xnthread *owner;	/** Thread which owns the resource */
	atomic_t *fastlock; /** Pointer to fast lock word */
	void (*cleanup)(struct xnsynch *synch); /* Cleanup handler */
};

#define XNSYNCH_WAITQUEUE_INITIALIZER(__name) {		\
		.status = XNSYNCH_PRIO,			\
		.wprio = -1,				\
		.pendq = LIST_HEAD_INIT((__name).pendq),	\
		.owner = NULL,				\
		.cleanup = NULL,			\
		.fastlock = NULL,			\
	}

#define DEFINE_XNWAITQ(__name)	\
	struct xnsynch __name = XNSYNCH_WAITQUEUE_INITIALIZER(__name)

static inline void xnsynch_set_status(struct xnsynch *synch, int bits)
{
	synch->status |= bits;
}

static inline void xnsynch_clear_status(struct xnsynch *synch, int bits)
{
	synch->status &= ~bits;
}

#define xnsynch_for_each_sleeper(__pos, __synch)		\
	list_for_each_entry(__pos, &(__synch)->pendq, plink)

#define xnsynch_for_each_sleeper_safe(__pos, __tmp, __synch)	\
	list_for_each_entry_safe(__pos, __tmp, &(__synch)->pendq, plink)

static inline int xnsynch_pended_p(struct xnsynch *synch)
{
	return !list_empty(&synch->pendq);
}

static inline struct xnthread *xnsynch_owner(struct xnsynch *synch)
{
	return synch->owner;
}

#define xnsynch_fastlock(synch)		((synch)->fastlock)
#define xnsynch_fastlock_p(synch)	((synch)->fastlock != NULL)
#define xnsynch_owner_check(synch, thread) \
	xnsynch_fast_owner_check((synch)->fastlock, thread->handle)

#if XENO_DEBUG(MUTEX_RELAXED)

void xnsynch_detect_relaxed_owner(struct xnsynch *synch,
				  struct xnthread *sleeper);

void xnsynch_detect_claimed_relax(struct xnthread *owner);

#else /* !XENO_DEBUG(MUTEX_RELAXED) */

static inline void xnsynch_detect_relaxed_owner(struct xnsynch *synch,
				  struct xnthread *sleeper)
{
}

static inline void xnsynch_detect_claimed_relax(struct xnthread *owner)
{
}

#endif /* !XENO_DEBUG(MUTEX_RELAXED) */

void xnsynch_init(struct xnsynch *synch, int flags,
		  atomic_t *fastlock);

int xnsynch_destroy(struct xnsynch *synch);

static inline void xnsynch_set_owner(struct xnsynch *synch,
				     struct xnthread *thread)
{
	synch->owner = thread;
}

static inline void xnsynch_register_cleanup(struct xnsynch *synch,
					    void (*handler)(struct xnsynch *))
{
	synch->cleanup = handler;
}

int xnsynch_sleep_on(struct xnsynch *synch,
		     xnticks_t timeout,
		     xntmode_t timeout_mode);

struct xnthread *xnsynch_wakeup_one_sleeper(struct xnsynch *synch);

int xnsynch_wakeup_many_sleepers(struct xnsynch *synch, int nr);

void xnsynch_wakeup_this_sleeper(struct xnsynch *synch,
				 struct xnthread *sleeper);

int xnsynch_acquire(struct xnsynch *synch,
		    xnticks_t timeout,
		    xntmode_t timeout_mode);

int xnsynch_try_acquire(struct xnsynch *synch);

struct xnthread *xnsynch_release(struct xnsynch *synch,
				 struct xnthread *thread);

struct xnthread *xnsynch_peek_pendq(struct xnsynch *synch);

int xnsynch_flush(struct xnsynch *synch, int reason);

void xnsynch_release_all_ownerships(struct xnthread *thread);

void xnsynch_requeue_sleeper(struct xnthread *thread);

void xnsynch_forget_sleeper(struct xnthread *thread);

/** @} */

#endif /* !_COBALT_KERNEL_SYNCH_H_ */
