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
#ifndef _COBALT_KERNEL_SCHED_H
#define _COBALT_KERNEL_SCHED_H

#include <linux/percpu.h>
#include <cobalt/kernel/lock.h>
#include <cobalt/kernel/thread.h>
#include <cobalt/kernel/schedqueue.h>
#include <cobalt/kernel/sched-tp.h>
#include <cobalt/kernel/sched-weak.h>
#include <cobalt/kernel/sched-sporadic.h>
#include <cobalt/kernel/sched-quota.h>
#include <cobalt/kernel/vfile.h>
#include <cobalt/kernel/assert.h>
#include <asm/xenomai/machine.h>

/**
 * @addtogroup cobalt_core_sched
 * @{
 */

/* Sched status flags */
#define XNRESCHED	0x10000000	/* Needs rescheduling */
#define XNINSW		0x20000000	/* In context switch */
#define XNINTCK		0x40000000	/* In master tick handler context */

/* Sched local flags */
#define XNIDLE		0x00010000	/* Idle (no outstanding timer) */
#define XNHTICK		0x00008000	/* Host tick pending  */
#define XNINIRQ		0x00004000	/* In IRQ handling context */
#define XNHDEFER	0x00002000	/* Host tick deferred */

struct xnsched_rt {
	xnsched_queue_t runnable;	/*!< Runnable thread queue. */
};

/*!
 * \brief Scheduling information structure.
 */

struct xnsched {
	/*!< Scheduler specific status bitmask. */
	unsigned long status;
	/*!< Scheduler specific local flags bitmask. */
	unsigned long lflags;
	/*!< Current thread. */
	struct xnthread *curr;
#ifdef CONFIG_SMP
	/*!< Owner CPU id. */
	int cpu;
	/*!< Mask of CPUs needing rescheduling. */
	cpumask_t resched;
#endif
	/*!< Context of built-in real-time class. */
	struct xnsched_rt rt;
#ifdef CONFIG_XENO_OPT_SCHED_WEAK
	/*!< Context of weak scheduling class. */
	struct xnsched_weak weak;
#endif
#ifdef CONFIG_XENO_OPT_SCHED_TP
	/*!< Context of TP class. */
	struct xnsched_tp tp;
#endif
#ifdef CONFIG_XENO_OPT_SCHED_SPORADIC
	/*!< Context of sporadic scheduling class. */
	struct xnsched_sporadic pss;
#endif
#ifdef CONFIG_XENO_OPT_SCHED_QUOTA
	/*!< Context of runtime quota scheduling. */
	struct xnsched_quota quota;
#endif
	/*!< Interrupt nesting level. */
	volatile unsigned inesting;
	/*!< Host timer. */
	struct xntimer htimer;
	/*!< Round-robin timer. */
	struct xntimer rrbtimer;
	/*!< Root thread control block. */
	struct xnthread rootcb;
#ifdef CONFIG_IPIPE_WANT_PREEMPTIBLE_SWITCH
	struct xnthread *last;
#endif
#ifdef CONFIG_XENO_ARCH_FPU
	/*!< Thread owning the current FPU context. */
	struct xnthread *fpuholder;
#endif
#ifdef CONFIG_XENO_OPT_WATCHDOG
	/*!< Watchdog timer object. */
	struct xntimer wdtimer;
#endif
#ifdef CONFIG_XENO_OPT_STATS
	/*!< Last account switch date (ticks). */
	xnticks_t last_account_switch;
	/*!< Currently active account */
	xnstat_exectime_t *current_account;
#endif
};

DECLARE_PER_CPU(struct xnsched, nksched);

extern cpumask_t cobalt_cpu_affinity;

extern struct list_head nkthreadq;

extern int cobalt_nrthreads;

#ifdef CONFIG_XENO_OPT_VFILE
extern struct xnvfile_rev_tag nkthreadlist_tag;
#endif

union xnsched_policy_param;

struct xnsched_class {
	void (*sched_init)(struct xnsched *sched);
	void (*sched_enqueue)(struct xnthread *thread);
	void (*sched_dequeue)(struct xnthread *thread);
	void (*sched_requeue)(struct xnthread *thread);
	struct xnthread *(*sched_pick)(struct xnsched *sched);
	void (*sched_tick)(struct xnsched *sched);
	void (*sched_rotate)(struct xnsched *sched,
			     const union xnsched_policy_param *p);
	void (*sched_migrate)(struct xnthread *thread,
			      struct xnsched *sched);
	int (*sched_chkparam)(struct xnthread *thread,
			      const union xnsched_policy_param *p);
	/**
	 * Set base scheduling parameters. This routine is indirectly
	 * called upon a change of base scheduling settings through
	 * __xnthread_set_schedparam() -> xnsched_set_policy(),
	 * exclusively.
	 *
	 * The scheduling class implementation should do the necessary
	 * housekeeping to comply with the new settings.
	 * thread->base_class is up to date before the call is made,
	 * and should be considered for the new weighted priority
	 * calculation. On the contrary, thread->sched_class should
	 * NOT be referred to by this handler.
	 *
	 * sched_setparam() is NEVER involved in PI or PP
	 * management. However it must deny a priority update if it
	 * contradicts an ongoing boost for @a thread. This is
	 * typically what the xnsched_set_effective_priority() helper
	 * does for such handler.
	 *
	 * @param thread Affected thread.
	 * @param p New base policy settings.
	 *
	 * @return True if the effective priority was updated
	 * (thread->cprio).
	 */
	bool (*sched_setparam)(struct xnthread *thread,
			       const union xnsched_policy_param *p);
	void (*sched_getparam)(struct xnthread *thread,
			       union xnsched_policy_param *p);
	void (*sched_trackprio)(struct xnthread *thread,
				const union xnsched_policy_param *p);
	void (*sched_protectprio)(struct xnthread *thread, int prio);
	int (*sched_declare)(struct xnthread *thread,
			     const union xnsched_policy_param *p);
	void (*sched_forget)(struct xnthread *thread);
	void (*sched_kick)(struct xnthread *thread);
#ifdef CONFIG_XENO_OPT_VFILE
	int (*sched_init_vfile)(struct xnsched_class *schedclass,
				struct xnvfile_directory *vfroot);
	void (*sched_cleanup_vfile)(struct xnsched_class *schedclass);
#endif
	int nthreads;
	struct xnsched_class *next;
	int weight;
	int policy;
	const char *name;
};

#define XNSCHED_CLASS_WEIGHT(n)		(n * XNSCHED_CLASS_WEIGHT_FACTOR)

/* Placeholder for current thread priority */
#define XNSCHED_RUNPRIO   0x80000000

#define xnsched_for_each_thread(__thread)	\
	list_for_each_entry(__thread, &nkthreadq, glink)

#ifdef CONFIG_SMP
static inline int xnsched_cpu(struct xnsched *sched)
{
	return sched->cpu;
}
#else /* !CONFIG_SMP */
static inline int xnsched_cpu(struct xnsched *sched)
{
	return 0;
}
#endif /* CONFIG_SMP */

static inline struct xnsched *xnsched_struct(int cpu)
{
	return &per_cpu(nksched, cpu);
}

static inline struct xnsched *xnsched_current(void)
{
	/* IRQs off */
	return raw_cpu_ptr(&nksched);
}

static inline struct xnthread *xnsched_current_thread(void)
{
	return xnsched_current()->curr;
}

/* Test resched flag of given sched. */
static inline int xnsched_resched_p(struct xnsched *sched)
{
	return sched->status & XNRESCHED;
}

/* Set self resched flag for the current scheduler. */
static inline void xnsched_set_self_resched(struct xnsched *sched)
{
	sched->status |= XNRESCHED;
}

#define xnsched_realtime_domain  cobalt_pipeline.domain

/* Set resched flag for the given scheduler. */
#ifdef CONFIG_SMP

static inline void xnsched_set_resched(struct xnsched *sched)
{
	struct xnsched *current_sched = xnsched_current();

	if (current_sched == sched)
		current_sched->status |= XNRESCHED;
	else if (!xnsched_resched_p(sched)) {
		cpumask_set_cpu(xnsched_cpu(sched), &current_sched->resched);
		sched->status |= XNRESCHED;
		current_sched->status |= XNRESCHED;
	}
}

#define xnsched_realtime_cpus    cobalt_pipeline.supported_cpus

static inline int xnsched_supported_cpu(int cpu)
{
	return cpumask_test_cpu(cpu, &xnsched_realtime_cpus);
}

static inline int xnsched_threading_cpu(int cpu)
{
	return cpumask_test_cpu(cpu, &cobalt_cpu_affinity);
}

#else /* !CONFIG_SMP */

static inline void xnsched_set_resched(struct xnsched *sched)
{
	xnsched_set_self_resched(sched);
}

#define xnsched_realtime_cpus CPU_MASK_ALL

static inline int xnsched_supported_cpu(int cpu)
{
	return 1;
}

static inline int xnsched_threading_cpu(int cpu)
{
	return 1;
}

#endif /* !CONFIG_SMP */

#define for_each_realtime_cpu(cpu)		\
	for_each_online_cpu(cpu)		\
		if (xnsched_supported_cpu(cpu))	\

int ___xnsched_run(struct xnsched *sched);

void __xnsched_run_handler(void);

static inline int __xnsched_run(struct xnsched *sched)
{
	/*
	 * Reschedule if XNSCHED is pending, but never over an IRQ
	 * handler or in the middle of unlocked context switch.
	 */
	if (((sched->status|sched->lflags) &
	     (XNINIRQ|XNINSW|XNRESCHED)) != XNRESCHED)
		return 0;

	return ___xnsched_run(sched);
}

static inline int xnsched_run(void)
{
	struct xnsched *sched = xnsched_current();
	/*
	 * sched->curr is shared locklessly with ___xnsched_run().
	 * READ_ONCE() makes sure the compiler never uses load tearing
	 * for reading this pointer piecemeal, so that multiple stores
	 * occurring concurrently on remote CPUs never yield a
	 * spurious merged value on the local one.
	 */
	struct xnthread *curr = READ_ONCE(sched->curr);

	/*
	 * If running over the root thread, hard irqs must be off
	 * (asserted out of line in ___xnsched_run()).
	 */
	return curr->lock_count > 0 ? 0 : __xnsched_run(sched);
}

void xnsched_lock(void);

void xnsched_unlock(void);

static inline int xnsched_interrupt_p(void)
{
	return xnsched_current()->lflags & XNINIRQ;
}

static inline int xnsched_root_p(void)
{
	return xnthread_test_state(xnsched_current_thread(), XNROOT);
}

static inline int xnsched_unblockable_p(void)
{
	return xnsched_interrupt_p() || xnsched_root_p();
}

static inline int xnsched_primary_p(void)
{
	return !xnsched_unblockable_p();
}

#ifdef CONFIG_IPIPE_WANT_PREEMPTIBLE_SWITCH

struct xnsched *xnsched_finish_unlocked_switch(struct xnsched *sched);

#define xnsched_resched_after_unlocked_switch() xnsched_run()

static inline
int xnsched_maybe_resched_after_unlocked_switch(struct xnsched *sched)
{
	return sched->status & XNRESCHED;
}

#else /* !CONFIG_IPIPE_WANT_PREEMPTIBLE_SWITCH */

static inline struct xnsched *
xnsched_finish_unlocked_switch(struct xnsched *sched)
{
	XENO_BUG_ON(COBALT, !hard_irqs_disabled());
	return xnsched_current();
}

static inline void xnsched_resched_after_unlocked_switch(void) { }

static inline int
xnsched_maybe_resched_after_unlocked_switch(struct xnsched *sched)
{
	return 0;
}

#endif /* !CONFIG_IPIPE_WANT_PREEMPTIBLE_SWITCH */

bool xnsched_set_effective_priority(struct xnthread *thread,
				    int prio);

#include <cobalt/kernel/sched-idle.h>
#include <cobalt/kernel/sched-rt.h>

int xnsched_init_proc(void);

void xnsched_cleanup_proc(void);

void xnsched_register_classes(void);

void xnsched_init_all(void);

void xnsched_destroy_all(void);

struct xnthread *xnsched_pick_next(struct xnsched *sched);

void xnsched_putback(struct xnthread *thread);

int xnsched_set_policy(struct xnthread *thread,
		       struct xnsched_class *sched_class,
		       const union xnsched_policy_param *p);

void xnsched_track_policy(struct xnthread *thread,
			  struct xnthread *target);

void xnsched_protect_priority(struct xnthread *thread,
			      int prio);

void xnsched_migrate(struct xnthread *thread,
		     struct xnsched *sched);

void xnsched_migrate_passive(struct xnthread *thread,
			     struct xnsched *sched);

/**
 * @fn void xnsched_rotate(struct xnsched *sched, struct xnsched_class *sched_class, const union xnsched_policy_param *sched_param)
 * @brief Rotate a scheduler runqueue.
 *
 * The specified scheduling class is requested to rotate its runqueue
 * for the given scheduler. Rotation is performed according to the
 * scheduling parameter specified by @a sched_param.
 *
 * @note The nucleus supports round-robin scheduling for the members
 * of the RT class.
 *
 * @param sched The per-CPU scheduler hosting the target scheduling
 * class.
 *
 * @param sched_class The scheduling class which should rotate its
 * runqueue.
 *
 * @param sched_param The scheduling parameter providing rotation
 * information to the specified scheduling class.
 *
 * @coretags{unrestricted, atomic-entry}
 */
static inline void xnsched_rotate(struct xnsched *sched,
				  struct xnsched_class *sched_class,
				  const union xnsched_policy_param *sched_param)
{
	sched_class->sched_rotate(sched, sched_param);
}

static inline int xnsched_init_thread(struct xnthread *thread)
{
	int ret = 0;

	xnsched_idle_init_thread(thread);
	xnsched_rt_init_thread(thread);

#ifdef CONFIG_XENO_OPT_SCHED_TP
	ret = xnsched_tp_init_thread(thread);
	if (ret)
		return ret;
#endif /* CONFIG_XENO_OPT_SCHED_TP */
#ifdef CONFIG_XENO_OPT_SCHED_SPORADIC
	ret = xnsched_sporadic_init_thread(thread);
	if (ret)
		return ret;
#endif /* CONFIG_XENO_OPT_SCHED_SPORADIC */
#ifdef CONFIG_XENO_OPT_SCHED_QUOTA
	ret = xnsched_quota_init_thread(thread);
	if (ret)
		return ret;
#endif /* CONFIG_XENO_OPT_SCHED_QUOTA */

	return ret;
}

static inline int xnsched_root_priority(struct xnsched *sched)
{
	return sched->rootcb.cprio;
}

static inline struct xnsched_class *xnsched_root_class(struct xnsched *sched)
{
	return sched->rootcb.sched_class;
}

static inline void xnsched_tick(struct xnsched *sched)
{
	struct xnthread *curr = sched->curr;
	struct xnsched_class *sched_class = curr->sched_class;
	/*
	 * A thread that undergoes round-robin scheduling only
	 * consumes its time slice when it runs within its own
	 * scheduling class, which excludes temporary PI boosts, and
	 * does not hold the scheduler lock.
	 */
	if (sched_class == curr->base_class &&
	    sched_class->sched_tick &&
	    xnthread_test_state(curr, XNTHREAD_BLOCK_BITS|XNRRB) == XNRRB &&
		curr->lock_count == 0)
		sched_class->sched_tick(sched);
}

static inline int xnsched_chkparam(struct xnsched_class *sched_class,
				   struct xnthread *thread,
				   const union xnsched_policy_param *p)
{
	if (sched_class->sched_chkparam)
		return sched_class->sched_chkparam(thread, p);

	return 0;
}

static inline int xnsched_declare(struct xnsched_class *sched_class,
				  struct xnthread *thread,
				  const union xnsched_policy_param *p)
{
	int ret;

	if (sched_class->sched_declare) {
		ret = sched_class->sched_declare(thread, p);
		if (ret)
			return ret;
	}
	if (sched_class != thread->base_class)
		sched_class->nthreads++;

	return 0;
}

static inline int xnsched_calc_wprio(struct xnsched_class *sched_class,
				     int prio)
{
	return prio + sched_class->weight;
}

#ifdef CONFIG_XENO_OPT_SCHED_CLASSES

static inline void xnsched_enqueue(struct xnthread *thread)
{
	struct xnsched_class *sched_class = thread->sched_class;

	if (sched_class != &xnsched_class_idle)
		sched_class->sched_enqueue(thread);
}

static inline void xnsched_dequeue(struct xnthread *thread)
{
	struct xnsched_class *sched_class = thread->sched_class;

	if (sched_class != &xnsched_class_idle)
		sched_class->sched_dequeue(thread);
}

static inline void xnsched_requeue(struct xnthread *thread)
{
	struct xnsched_class *sched_class = thread->sched_class;

	if (sched_class != &xnsched_class_idle)
		sched_class->sched_requeue(thread);
}

static inline
bool xnsched_setparam(struct xnthread *thread,
		      const union xnsched_policy_param *p)
{
	return thread->base_class->sched_setparam(thread, p);
}

static inline void xnsched_getparam(struct xnthread *thread,
				    union xnsched_policy_param *p)
{
	thread->sched_class->sched_getparam(thread, p);
}

static inline void xnsched_trackprio(struct xnthread *thread,
				     const union xnsched_policy_param *p)
{
	thread->sched_class->sched_trackprio(thread, p);
	thread->wprio = xnsched_calc_wprio(thread->sched_class, thread->cprio);
}

static inline void xnsched_protectprio(struct xnthread *thread, int prio)
{
	thread->sched_class->sched_protectprio(thread, prio);
	thread->wprio = xnsched_calc_wprio(thread->sched_class, thread->cprio);
}

static inline void xnsched_forget(struct xnthread *thread)
{
	struct xnsched_class *sched_class = thread->base_class;

	--sched_class->nthreads;

	if (sched_class->sched_forget)
		sched_class->sched_forget(thread);
}

static inline void xnsched_kick(struct xnthread *thread)
{
	struct xnsched_class *sched_class = thread->base_class;

	xnthread_set_info(thread, XNKICKED);

	if (sched_class->sched_kick)
		sched_class->sched_kick(thread);

	xnsched_set_resched(thread->sched);
}

#else /* !CONFIG_XENO_OPT_SCHED_CLASSES */

/*
 * If only the RT and IDLE scheduling classes are compiled in, we can
 * fully inline common helpers for dealing with those.
 */

static inline void xnsched_enqueue(struct xnthread *thread)
{
	struct xnsched_class *sched_class = thread->sched_class;

	if (sched_class != &xnsched_class_idle)
		__xnsched_rt_enqueue(thread);
}

static inline void xnsched_dequeue(struct xnthread *thread)
{
	struct xnsched_class *sched_class = thread->sched_class;

	if (sched_class != &xnsched_class_idle)
		__xnsched_rt_dequeue(thread);
}

static inline void xnsched_requeue(struct xnthread *thread)
{
	struct xnsched_class *sched_class = thread->sched_class;

	if (sched_class != &xnsched_class_idle)
		__xnsched_rt_requeue(thread);
}

static inline bool xnsched_setparam(struct xnthread *thread,
				    const union xnsched_policy_param *p)
{
	struct xnsched_class *sched_class = thread->base_class;

	if (sched_class == &xnsched_class_idle)
		return __xnsched_idle_setparam(thread, p);

	return __xnsched_rt_setparam(thread, p);
}

static inline void xnsched_getparam(struct xnthread *thread,
				    union xnsched_policy_param *p)
{
	struct xnsched_class *sched_class = thread->sched_class;

	if (sched_class == &xnsched_class_idle)
		__xnsched_idle_getparam(thread, p);
	else
		__xnsched_rt_getparam(thread, p);
}

static inline void xnsched_trackprio(struct xnthread *thread,
				     const union xnsched_policy_param *p)
{
	struct xnsched_class *sched_class = thread->sched_class;

	if (sched_class == &xnsched_class_idle)
		__xnsched_idle_trackprio(thread, p);
	else
		__xnsched_rt_trackprio(thread, p);

	thread->wprio = xnsched_calc_wprio(sched_class, thread->cprio);
}

static inline void xnsched_protectprio(struct xnthread *thread, int prio)
{
	struct xnsched_class *sched_class = thread->sched_class;

	if (sched_class == &xnsched_class_idle)
		__xnsched_idle_protectprio(thread, prio);
	else
		__xnsched_rt_protectprio(thread, prio);

	thread->wprio = xnsched_calc_wprio(sched_class, thread->cprio);
}

static inline void xnsched_forget(struct xnthread *thread)
{
	--thread->base_class->nthreads;
	__xnsched_rt_forget(thread);
}

static inline void xnsched_kick(struct xnthread *thread)
{
	xnthread_set_info(thread, XNKICKED);
	xnsched_set_resched(thread->sched);
}

#endif /* !CONFIG_XENO_OPT_SCHED_CLASSES */

/** @} */

#endif /* !_COBALT_KERNEL_SCHED_H */
