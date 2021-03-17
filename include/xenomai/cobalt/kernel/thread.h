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
#ifndef _COBALT_KERNEL_THREAD_H
#define _COBALT_KERNEL_THREAD_H

#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/sched/rt.h>
#include <cobalt/kernel/list.h>
#include <cobalt/kernel/stat.h>
#include <cobalt/kernel/timer.h>
#include <cobalt/kernel/registry.h>
#include <cobalt/kernel/schedparam.h>
#include <cobalt/kernel/trace.h>
#include <cobalt/kernel/synch.h>
#include <cobalt/uapi/kernel/thread.h>
#include <cobalt/uapi/signal.h>
#include <asm/xenomai/machine.h>
#include <asm/xenomai/thread.h>

/**
 * @addtogroup cobalt_core_thread
 * @{
 */
#define XNTHREAD_BLOCK_BITS   (XNSUSP|XNPEND|XNDELAY|XNDORMANT|XNRELAX|XNMIGRATE|XNHELD|XNDBGSTOP)
#define XNTHREAD_MODE_BITS    (XNRRB|XNWARN|XNTRAPLB)

struct xnthread;
struct xnsched;
struct xnselector;
struct xnsched_class;
struct xnsched_tpslot;
struct xnthread_personality;
struct completion;

struct xnthread_init_attr {
	struct xnthread_personality *personality;
	cpumask_t affinity;
	int flags;
	const char *name;
};

struct xnthread_start_attr {
	int mode;
	void (*entry)(void *cookie);
	void *cookie;
};

struct xnthread_wait_context {
	int posted;
};

struct xnthread_personality {
	const char *name;
	unsigned int magic;
	int xid;
	atomic_t refcnt;
	struct {
		void *(*attach_process)(void);
		void (*detach_process)(void *arg);
		void (*map_thread)(struct xnthread *thread);
		struct xnthread_personality *(*relax_thread)(struct xnthread *thread);
		struct xnthread_personality *(*harden_thread)(struct xnthread *thread);
		struct xnthread_personality *(*move_thread)(struct xnthread *thread,
							    int dest_cpu);
		struct xnthread_personality *(*exit_thread)(struct xnthread *thread);
		struct xnthread_personality *(*finalize_thread)(struct xnthread *thread);
	} ops;
	struct module *module;
};

struct xnthread {
	struct xnarchtcb tcb;	/* Architecture-dependent block */

	__u32 state;		/* Thread state flags */
	__u32 info;		/* Thread information flags */
	__u32 local_info;	/* Local thread information flags */

	struct xnsched *sched;		/* Thread scheduler */
	struct xnsched_class *sched_class; /* Current scheduling class */
	struct xnsched_class *base_class; /* Base scheduling class */

#ifdef CONFIG_XENO_OPT_SCHED_TP
	struct xnsched_tpslot *tps;	/* Current partition slot for TP scheduling */
	struct list_head tp_link;	/* Link in per-sched TP thread queue */
#endif
#ifdef CONFIG_XENO_OPT_SCHED_SPORADIC
	struct xnsched_sporadic_data *pss; /* Sporadic scheduling data. */
#endif
#ifdef CONFIG_XENO_OPT_SCHED_QUOTA
	struct xnsched_quota_group *quota; /* Quota scheduling group. */
	struct list_head quota_expired;
	struct list_head quota_next;
#endif
	cpumask_t affinity;	/* Processor affinity. */

	/** Base priority (before PI/PP boost) */
	int bprio;

	/** Current (effective) priority */
	int cprio;

	/**
	 * Weighted priority (cprio + scheduling class weight).
	 */
	int wprio;

	int lock_count;	/** Scheduler lock count. */

	/**
	 * Thread holder in xnsched run queue. Ordered by
	 * thread->cprio.
	 */
	struct list_head rlink;

	/**
	 * Thread holder in xnsynch pendq. Prioritized by
	 * thread->cprio + scheduling class weight.
	 */
	struct list_head plink;

	/** Thread holder in global queue. */
	struct list_head glink;

	/**
	 * List of xnsynch owned by this thread which cause a priority
	 * boost due to one of the following reasons:
	 *
	 * - they are currently claimed by other thread(s) when
	 * enforcing the priority inheritance protocol (XNSYNCH_PI).
	 *
	 * - they require immediate priority ceiling (XNSYNCH_PP).
	 *
	 * This list is ordered by decreasing (weighted) thread
	 * priorities.
	 */
	struct list_head boosters;

	struct xnsynch *wchan;		/* Resource the thread pends on */

	struct xnsynch *wwake;		/* Wait channel the thread was resumed from */

	int res_count;			/* Held resources count */

	struct xntimer rtimer;		/* Resource timer */

	struct xntimer ptimer;		/* Periodic timer */

	xnticks_t rrperiod;		/* Allotted round-robin period (ns) */

  	struct xnthread_wait_context *wcontext;	/* Active wait context. */

	struct {
		xnstat_counter_t ssw;	/* Primary -> secondary mode switch count */
		xnstat_counter_t csw;	/* Context switches (includes secondary -> primary switches) */
		xnstat_counter_t xsc;	/* Xenomai syscalls */
		xnstat_counter_t pf;	/* Number of page faults */
		xnstat_exectime_t account; /* Execution time accounting entity */
		xnstat_exectime_t lastperiod; /* Interval marker for execution time reports */
	} stat;

	struct xnselector *selector;    /* For select. */

	xnhandle_t handle;	/* Handle in registry */

	char name[XNOBJECT_NAME_LEN]; /* Symbolic name of thread */

	void (*entry)(void *cookie); /* Thread entry routine */
	void *cookie;		/* Cookie to pass to the entry routine */

	/**
	 * Thread data visible from userland through a window on the
	 * global heap.
	 */
	struct xnthread_user_window *u_window;

	struct xnthread_personality *personality;

	struct completion exited;

#ifdef CONFIG_XENO_OPT_DEBUG
	const char *exe_path;	/* Executable path */
	u32 proghash;		/* Hash value for exe_path */
#endif
};

static inline int xnthread_get_state(const struct xnthread *thread)
{
	return thread->state;
}

static inline int xnthread_test_state(struct xnthread *thread, int bits)
{
	return thread->state & bits;
}

static inline void xnthread_set_state(struct xnthread *thread, int bits)
{
	thread->state |= bits;
}

static inline void xnthread_clear_state(struct xnthread *thread, int bits)
{
	thread->state &= ~bits;
}

static inline int xnthread_test_info(struct xnthread *thread, int bits)
{
	return thread->info & bits;
}

static inline void xnthread_set_info(struct xnthread *thread, int bits)
{
	thread->info |= bits;
}

static inline void xnthread_clear_info(struct xnthread *thread, int bits)
{
	thread->info &= ~bits;
}

static inline int xnthread_test_localinfo(struct xnthread *curr, int bits)
{
	return curr->local_info & bits;
}

static inline void xnthread_set_localinfo(struct xnthread *curr, int bits)
{
	curr->local_info |= bits;
}

static inline void xnthread_clear_localinfo(struct xnthread *curr, int bits)
{
	curr->local_info &= ~bits;
}

static inline struct xnarchtcb *xnthread_archtcb(struct xnthread *thread)
{
	return &thread->tcb;
}

static inline int xnthread_base_priority(const struct xnthread *thread)
{
	return thread->bprio;
}

static inline int xnthread_current_priority(const struct xnthread *thread)
{
	return thread->cprio;
}

static inline struct task_struct *xnthread_host_task(struct xnthread *thread)
{
	return xnthread_archtcb(thread)->core.host_task;
}

#define xnthread_for_each_booster(__pos, __thread)		\
	list_for_each_entry(__pos, &(__thread)->boosters, next)

#define xnthread_for_each_booster_safe(__pos, __tmp, __thread)	\
	list_for_each_entry_safe(__pos, __tmp, &(__thread)->boosters, next)

#define xnthread_run_handler(__t, __h, __a...)				\
	do {								\
		struct xnthread_personality *__p__ = (__t)->personality;	\
		if ((__p__)->ops.__h)					\
			(__p__)->ops.__h(__t, ##__a);			\
	} while (0)
	
#define xnthread_run_handler_stack(__t, __h, __a...)			\
	do {								\
		struct xnthread_personality *__p__ = (__t)->personality;	\
		do {							\
			if ((__p__)->ops.__h == NULL)			\
				break;					\
			__p__ = (__p__)->ops.__h(__t, ##__a);		\
		} while (__p__);					\
	} while (0)
	
static inline
struct xnthread_wait_context *xnthread_get_wait_context(struct xnthread *thread)
{
	return thread->wcontext;
}

static inline
int xnthread_register(struct xnthread *thread, const char *name)
{
	return xnregistry_enter(name, thread, &thread->handle, NULL);
}

static inline
struct xnthread *xnthread_lookup(xnhandle_t threadh)
{
	struct xnthread *thread = xnregistry_lookup(threadh, NULL);
	return thread && thread->handle == xnhandle_get_index(threadh) ? thread : NULL;
}

static inline void xnthread_sync_window(struct xnthread *thread)
{
	if (thread->u_window) {
		thread->u_window->state = thread->state;
		thread->u_window->info = thread->info;
	}
}

static inline
void xnthread_clear_sync_window(struct xnthread *thread, int state_bits)
{
	if (thread->u_window) {
		thread->u_window->state = thread->state & ~state_bits;
		thread->u_window->info = thread->info;
	}
}

static inline
void xnthread_set_sync_window(struct xnthread *thread, int state_bits)
{
	if (thread->u_window) {
		thread->u_window->state = thread->state | state_bits;
		thread->u_window->info = thread->info;
	}
}

static inline int normalize_priority(int prio)
{
	return prio < MAX_RT_PRIO ? prio : MAX_RT_PRIO - 1;
}

int __xnthread_init(struct xnthread *thread,
		    const struct xnthread_init_attr *attr,
		    struct xnsched *sched,
		    struct xnsched_class *sched_class,
		    const union xnsched_policy_param *sched_param);

void __xnthread_test_cancel(struct xnthread *curr);

void __xnthread_cleanup(struct xnthread *curr);

void __xnthread_discard(struct xnthread *thread);

/**
 * @fn struct xnthread *xnthread_current(void)
 * @brief Retrieve the current Cobalt core TCB.
 *
 * Returns the address of the current Cobalt core thread descriptor,
 * or NULL if running over a regular Linux task. This call is not
 * affected by the current runtime mode of the core thread.
 *
 * @note The returned value may differ from xnsched_current_thread()
 * called from the same context, since the latter returns the root
 * thread descriptor for the current CPU if the caller is running in
 * secondary mode.
 *
 * @coretags{unrestricted}
 */
static inline struct xnthread *xnthread_current(void)
{
	return ipipe_current_threadinfo()->thread;
}

/**
 * @fn struct xnthread *xnthread_from_task(struct task_struct *p)
 * @brief Retrieve the Cobalt core TCB attached to a Linux task.
 *
 * Returns the address of the Cobalt core thread descriptor attached
 * to the Linux task @a p, or NULL if @a p is a regular Linux
 * task. This call is not affected by the current runtime mode of the
 * core thread.
 *
 * @coretags{unrestricted}
 */
static inline struct xnthread *xnthread_from_task(struct task_struct *p)
{
	return ipipe_task_threadinfo(p)->thread;
}

/**
 * @fn void xnthread_test_cancel(void)
 * @brief Introduce a thread cancellation point.
 *
 * Terminates the current thread if a cancellation request is pending
 * for it, i.e. if xnthread_cancel() was called.
 *
 * @coretags{mode-unrestricted}
 */
static inline void xnthread_test_cancel(void)
{
	struct xnthread *curr = xnthread_current();

	if (curr && xnthread_test_info(curr, XNCANCELD))
		__xnthread_test_cancel(curr);
}

static inline
void xnthread_complete_wait(struct xnthread_wait_context *wc)
{
	wc->posted = 1;
}

static inline
int xnthread_wait_complete_p(struct xnthread_wait_context *wc)
{
	return wc->posted;
}

#ifdef CONFIG_XENO_ARCH_FPU
void xnthread_switch_fpu(struct xnsched *sched);
#else
static inline void xnthread_switch_fpu(struct xnsched *sched) { }
#endif /* CONFIG_XENO_ARCH_FPU */

void xnthread_init_shadow_tcb(struct xnthread *thread);

void xnthread_init_root_tcb(struct xnthread *thread);

void xnthread_deregister(struct xnthread *thread);

char *xnthread_format_status(unsigned long status,
			     char *buf, int size);

pid_t xnthread_host_pid(struct xnthread *thread);

int xnthread_set_clock(struct xnthread *thread,
		       struct xnclock *newclock);

xnticks_t xnthread_get_timeout(struct xnthread *thread,
			       xnticks_t ns);

xnticks_t xnthread_get_period(struct xnthread *thread);

void xnthread_prepare_wait(struct xnthread_wait_context *wc);

int xnthread_init(struct xnthread *thread,
		  const struct xnthread_init_attr *attr,
		  struct xnsched_class *sched_class,
		  const union xnsched_policy_param *sched_param);

int xnthread_start(struct xnthread *thread,
		   const struct xnthread_start_attr *attr);

int xnthread_set_mode(int clrmask,
		      int setmask);

void xnthread_suspend(struct xnthread *thread,
		      int mask,
		      xnticks_t timeout,
		      xntmode_t timeout_mode,
		      struct xnsynch *wchan);

void xnthread_resume(struct xnthread *thread,
		     int mask);

int xnthread_unblock(struct xnthread *thread);

int xnthread_set_periodic(struct xnthread *thread,
			  xnticks_t idate,
			  xntmode_t timeout_mode,
			  xnticks_t period);

int xnthread_wait_period(unsigned long *overruns_r);

int xnthread_set_slice(struct xnthread *thread,
		       xnticks_t quantum);

void xnthread_cancel(struct xnthread *thread);

int xnthread_join(struct xnthread *thread, bool uninterruptible);

int xnthread_harden(void);

void xnthread_relax(int notify, int reason);

void __xnthread_kick(struct xnthread *thread);

void xnthread_kick(struct xnthread *thread);

void __xnthread_demote(struct xnthread *thread);

void xnthread_demote(struct xnthread *thread);

void xnthread_signal(struct xnthread *thread,
		     int sig, int arg);

void xnthread_pin_initial(struct xnthread *thread);

int xnthread_map(struct xnthread *thread,
		 struct completion *done);

void xnthread_call_mayday(struct xnthread *thread, int reason);

static inline void xnthread_get_resource(struct xnthread *curr)
{
	if (xnthread_test_state(curr, XNWEAK|XNDEBUG))
		curr->res_count++;
}

static inline int xnthread_put_resource(struct xnthread *curr)
{
	if (xnthread_test_state(curr, XNWEAK) ||
	    IS_ENABLED(CONFIG_XENO_OPT_DEBUG_MUTEX_SLEEP)) {
		if (unlikely(curr->res_count == 0)) {
			if (xnthread_test_state(curr, XNWARN))
				xnthread_signal(curr, SIGDEBUG,
						SIGDEBUG_RESCNT_IMBALANCE);
			return -EPERM;
		}
		curr->res_count--;
	}

	return 0;
}

static inline void xnthread_commit_ceiling(struct xnthread *curr)
{
	if (curr->u_window->pp_pending)
		xnsynch_commit_ceiling(curr);
}

#ifdef CONFIG_SMP

void xnthread_migrate_passive(struct xnthread *thread,
			      struct xnsched *sched);
#else

static inline void xnthread_migrate_passive(struct xnthread *thread,
					    struct xnsched *sched)
{ }

#endif

int __xnthread_set_schedparam(struct xnthread *thread,
			      struct xnsched_class *sched_class,
			      const union xnsched_policy_param *sched_param);

int xnthread_set_schedparam(struct xnthread *thread,
			    struct xnsched_class *sched_class,
			    const union xnsched_policy_param *sched_param);

int xnthread_killall(int grace, int mask);

void __xnthread_propagate_schedparam(struct xnthread *curr);

static inline void xnthread_propagate_schedparam(struct xnthread *curr)
{
	if (xnthread_test_info(curr, XNSCHEDP))
		__xnthread_propagate_schedparam(curr);
}

extern struct xnthread_personality xenomai_personality;

/** @} */

#endif /* !_COBALT_KERNEL_THREAD_H */
