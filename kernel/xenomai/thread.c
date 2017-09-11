/*
 * Copyright (C) 2001-2013 Philippe Gerum <rpm@xenomai.org>.
 * Copyright (C) 2006-2010 Gilles Chanteperdrix <gilles.chanteperdrix@xenomai.org>
 * Copyright (C) 2001-2013 The Xenomai project <http://www.xenomai.org>
 *
 * SMP support Copyright (C) 2004 The HYADES project <http://www.hyades-itea.org>
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
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/signal.h>
#include <linux/pid.h>
#include <cobalt/kernel/sched.h>
#include <cobalt/kernel/timer.h>
#include <cobalt/kernel/synch.h>
#include <cobalt/kernel/heap.h>
#include <cobalt/kernel/intr.h>
#include <cobalt/kernel/registry.h>
#include <cobalt/kernel/clock.h>
#include <cobalt/kernel/stat.h>
#include <cobalt/kernel/trace.h>
#include <cobalt/kernel/assert.h>
#include <cobalt/kernel/select.h>
#include <cobalt/kernel/lock.h>
#include <cobalt/kernel/thread.h>
#include <trace/events/cobalt-core.h>
#include <asm-generic/xenomai/mayday.h>
#include "debug.h"

static DECLARE_WAIT_QUEUE_HEAD(join_all);

/**
 * @ingroup cobalt_core
 * @defgroup cobalt_core_thread Thread services
 * @{
 */

static void timeout_handler(struct xntimer *timer)
{
	struct xnthread *thread = container_of(timer, struct xnthread, rtimer);

	xnthread_set_info(thread, XNTIMEO);	/* Interrupts are off. */
	xnthread_resume(thread, XNDELAY);
}

static inline void fixup_ptimer_affinity(struct xnthread *thread)
{
#ifdef CONFIG_SMP
	struct xntimer *timer = &thread->ptimer;
	int cpu;
	/*
	 * The thread a periodic timer is affine to might have been
	 * migrated to another CPU while passive. Fix this up.
	 */
	if (thread->sched != timer->sched) {
		cpu = xnclock_get_default_cpu(xntimer_clock(timer),
					      xnsched_cpu(thread->sched));
		xntimer_set_sched(timer, xnsched_struct(cpu));
	}
#endif
}

static void periodic_handler(struct xntimer *timer)
{
	struct xnthread *thread = container_of(timer, struct xnthread, ptimer);
	/*
	 * Prevent unwanted round-robin, and do not wake up threads
	 * blocked on a resource.
	 */
	if (xnthread_test_state(thread, XNDELAY|XNPEND) == XNDELAY)
		xnthread_resume(thread, XNDELAY);

	fixup_ptimer_affinity(thread);
}

static inline void enlist_new_thread(struct xnthread *thread)
{				/* nklock held, irqs off */
	list_add_tail(&thread->glink, &nkthreadq);
	cobalt_nrthreads++;
	xnvfile_touch_tag(&nkthreadlist_tag);
}

struct kthread_arg {
	struct xnthread *thread;
	struct completion *done;
};

static int kthread_trampoline(void *arg)
{
	struct kthread_arg *ka = arg;
	struct xnthread *thread = ka->thread;
	struct sched_param param;
	int ret, policy, prio;

	/*
	 * It only makes sense to create Xenomai kthreads with the
	 * SCHED_FIFO, SCHED_NORMAL or SCHED_WEAK policies. So
	 * anything that is not from Xenomai's RT class is assumed to
	 * belong to SCHED_NORMAL linux-wise.
	 */
	if (thread->sched_class != &xnsched_class_rt) {
		policy = SCHED_NORMAL;
		prio = 0;
	} else {
		policy = SCHED_FIFO;
		prio = normalize_priority(thread->cprio);
	}

	param.sched_priority = prio;
	sched_setscheduler(current, policy, &param);

	ret = xnthread_map(thread, ka->done);
	if (ret) {
		printk(XENO_WARNING "failed to create kernel shadow %s\n",
		       thread->name);
		return ret;
	}

	trace_cobalt_shadow_entry(thread);

	thread->entry(thread->cookie);

	xnthread_cancel(thread);

	return 0;
}

static inline int spawn_kthread(struct xnthread *thread)
{
	DECLARE_COMPLETION_ONSTACK(done);
	struct kthread_arg ka = {
		.thread = thread,
		.done = &done
	};
	struct task_struct *p;

	p = kthread_run(kthread_trampoline, &ka, "%s", thread->name);
	if (IS_ERR(p))
		return PTR_ERR(p);

	wait_for_completion(&done);

	return 0;
}

int __xnthread_init(struct xnthread *thread,
		    const struct xnthread_init_attr *attr,
		    struct xnsched *sched,
		    struct xnsched_class *sched_class,
		    const union xnsched_policy_param *sched_param)
{
	int flags = attr->flags, ret, gravity;

	flags &= ~XNSUSP;
#ifndef CONFIG_XENO_ARCH_FPU
	flags &= ~XNFPU;
#endif
	if ((flags & XNROOT) == 0)
		flags |= XNDORMANT;

	if (attr->name)
		ksformat(thread->name,
			 sizeof(thread->name), "%s", attr->name);
	else
		ksformat(thread->name,
			 sizeof(thread->name), "@%p", thread);

	/*
	 * We mirror the global user debug state into the per-thread
	 * state, to speed up branch taking in lib/cobalt wherever
	 * this needs to be tested.
	 */
	if (IS_ENABLED(CONFIG_XENO_OPT_DEBUG_MUTEX_SLEEP))
		flags |= XNDEBUG;

	thread->personality = attr->personality;
	cpumask_and(&thread->affinity, &attr->affinity, &cobalt_cpu_affinity);
	thread->sched = sched;
	thread->state = flags;
	thread->info = 0;
	thread->local_info = 0;
	thread->lock_count = 0;
	thread->rrperiod = XN_INFINITE;
	thread->wchan = NULL;
	thread->wwake = NULL;
	thread->wcontext = NULL;
	thread->res_count = 0;
	thread->handle = XN_NO_HANDLE;
	memset(&thread->stat, 0, sizeof(thread->stat));
	thread->selector = NULL;
	INIT_LIST_HEAD(&thread->claimq);
	INIT_LIST_HEAD(&thread->glink);
	/* These will be filled by xnthread_start() */
	thread->entry = NULL;
	thread->cookie = NULL;
	init_completion(&thread->exited);

	gravity = flags & XNUSER ? XNTIMER_UGRAVITY : XNTIMER_KGRAVITY;
	xntimer_init(&thread->rtimer, &nkclock, timeout_handler,
		     sched, gravity);
	xntimer_set_name(&thread->rtimer, thread->name);
	xntimer_set_priority(&thread->rtimer, XNTIMER_HIPRIO);
	xntimer_init(&thread->ptimer, &nkclock, periodic_handler,
		     sched, gravity);
	xntimer_set_name(&thread->ptimer, thread->name);
	xntimer_set_priority(&thread->ptimer, XNTIMER_HIPRIO);

	thread->base_class = NULL; /* xnsched_set_policy() will set it. */
	ret = xnsched_init_thread(thread);
	if (ret)
		goto err_out;

	ret = xnsched_set_policy(thread, sched_class, sched_param);
	if (ret)
		goto err_out;

	if ((flags & (XNUSER|XNROOT)) == 0) {
		ret = spawn_kthread(thread);
		if (ret)
			goto err_out;
	}

	return 0;

err_out:
	xntimer_destroy(&thread->rtimer);
	xntimer_destroy(&thread->ptimer);

	return ret;
}

void xnthread_init_shadow_tcb(struct xnthread *thread)
{
	struct xnarchtcb *tcb = xnthread_archtcb(thread);
	struct task_struct *p = current;

	/*
	 * If the current task is a kthread, the pipeline will take
	 * the necessary steps to make the FPU usable in such
	 * context. The kernel already took care of this issue for
	 * userland tasks (e.g. setting up a clean backup area).
	 */
	__ipipe_share_current(0);

	memset(tcb, 0, sizeof(*tcb));
	tcb->core.host_task = p;
	tcb->core.tsp = &p->thread;
	tcb->core.mm = p->mm;
	tcb->core.active_mm = p->mm;
#ifdef CONFIG_XENO_ARCH_WANT_TIP
	tcb->core.tip = task_thread_info(p);
#endif
#ifdef CONFIG_XENO_ARCH_FPU
	tcb->core.user_fpu_owner = p;
#endif /* CONFIG_XENO_ARCH_FPU */
	xnarch_init_shadow_tcb(thread);
}

void xnthread_init_root_tcb(struct xnthread *thread)
{
	struct xnarchtcb *tcb = xnthread_archtcb(thread);
	struct task_struct *p = current;

	memset(tcb, 0, sizeof(*tcb));
	tcb->core.host_task = p;
	tcb->core.tsp = &tcb->core.ts;
	tcb->core.mm = p->mm;
#ifdef CONFIG_XENO_ARCH_WANT_TIP
	tcb->core.tip = NULL;
#endif
	xnarch_init_root_tcb(thread);
}

void xnthread_deregister(struct xnthread *thread)
{
	if (thread->handle != XN_NO_HANDLE)
		xnregistry_remove(thread->handle);

	thread->handle = XN_NO_HANDLE;
}

char *xnthread_format_status(unsigned long status, char *buf, int size)
{
	static const char labels[] = XNTHREAD_STATE_LABELS;
	int pos, c, mask;
	char *wp;

	for (mask = (int)status, pos = 0, wp = buf;
	     mask != 0 && wp - buf < size - 2;	/* 1-letter label + \0 */
	     mask >>= 1, pos++) {
		if ((mask & 1) == 0)
			continue;

		c = labels[pos];

		switch (1 << pos) {
		case XNROOT:
			c = 'R'; /* Always mark root as runnable. */
			break;
		case XNREADY:
			if (status & XNROOT)
				continue; /* Already reported on XNROOT. */
			break;
		case XNDELAY:
			/*
			 * Only report genuine delays here, not timed
			 * waits for resources.
			 */
			if (status & XNPEND)
				continue;
			break;
		case XNPEND:
			/* Report timed waits with lowercase symbol. */
			if (status & XNDELAY)
				c |= 0x20;
			break;
		default:
			if (c == '.')
				continue;
		}
		*wp++ = c;
	}

	*wp = '\0';

	return buf;
}

int xnthread_set_clock(struct xnthread *thread, struct xnclock *newclock)
{
	spl_t s;

	if (thread == NULL) {
		thread = xnthread_current();
		if (thread == NULL)
			return -EPERM;
	}
	
	/* Change the clock the thread's periodic timer is paced by. */
	xnlock_get_irqsave(&nklock, s);
	xntimer_set_clock(&thread->ptimer, newclock);
	xnlock_put_irqrestore(&nklock, s);

	return 0;
}
EXPORT_SYMBOL_GPL(xnthread_set_clock);

xnticks_t xnthread_get_timeout(struct xnthread *thread, xnticks_t ns)
{
	struct xntimer *timer;
	xnticks_t timeout;

	if (!xnthread_test_state(thread,XNDELAY))
		return 0LL;

	if (xntimer_running_p(&thread->rtimer))
		timer = &thread->rtimer;
	else if (xntimer_running_p(&thread->ptimer))
		timer = &thread->ptimer;
	else
		return 0LL;

	timeout = xntimer_get_date(timer);
	if (timeout <= ns)
		return 1;

	return timeout - ns;
}
EXPORT_SYMBOL_GPL(xnthread_get_timeout);

xnticks_t xnthread_get_period(struct xnthread *thread)
{
	xnticks_t period = 0;
	/*
	 * The current thread period might be:
	 * - the value of the timer interval for periodic threads (ns/ticks)
	 * - or, the value of the alloted round-robin quantum (ticks)
	 * - or zero, meaning "no periodic activity".
	 */
	if (xntimer_running_p(&thread->ptimer))
		period = xntimer_interval(&thread->ptimer);
	else if (xnthread_test_state(thread,XNRRB))
		period = thread->rrperiod;

	return period;
}
EXPORT_SYMBOL_GPL(xnthread_get_period);

void xnthread_prepare_wait(struct xnthread_wait_context *wc)
{
	struct xnthread *curr = xnthread_current();

	wc->posted = 0;
	curr->wcontext = wc;
}
EXPORT_SYMBOL_GPL(xnthread_prepare_wait);

static inline int moving_target(struct xnsched *sched, struct xnthread *thread)
{
	int ret = 0;
#ifdef CONFIG_XENO_ARCH_UNLOCKED_SWITCH
	/*
	 * When deleting a thread in the course of a context switch or
	 * in flight to another CPU with nklock unlocked on a distant
	 * CPU, do nothing, this case will be caught in
	 * xnsched_finish_unlocked_switch.
	 */
	ret = (sched->status & XNINSW) ||
		xnthread_test_state(thread, XNMIGRATE);
#endif
	return ret;
}

#ifdef CONFIG_XENO_ARCH_FPU

static inline void giveup_fpu(struct xnsched *sched,
			      struct xnthread *thread)
{
	if (thread == sched->fpuholder)
		sched->fpuholder = NULL;
}

void xnthread_switch_fpu(struct xnsched *sched)
{
	struct xnthread *curr = sched->curr;

	if (!xnthread_test_state(curr, XNFPU))
		return;

	xnarch_switch_fpu(sched->fpuholder, curr);
	sched->fpuholder = curr;
}

#else /* !CONFIG_XENO_ARCH_FPU */

static inline void giveup_fpu(struct xnsched *sched,
				      struct xnthread *thread)
{
}

#endif /* !CONFIG_XENO_ARCH_FPU */

static inline void cleanup_tcb(struct xnthread *thread) /* nklock held, irqs off */
{
	struct xnsched *sched = thread->sched;

	list_del(&thread->glink);
	cobalt_nrthreads--;
	xnvfile_touch_tag(&nkthreadlist_tag);

	if (xnthread_test_state(thread, XNREADY)) {
		XENO_BUG_ON(COBALT, xnthread_test_state(thread, XNTHREAD_BLOCK_BITS));
		xnsched_dequeue(thread);
		xnthread_clear_state(thread, XNREADY);
	}

	if (xnthread_test_state(thread, XNPEND))
		xnsynch_forget_sleeper(thread);

	xnthread_set_state(thread, XNZOMBIE);
	/*
	 * NOTE: we must be running over the root thread, or @thread
	 * is dormant, which means that we don't risk sched->curr to
	 * disappear due to voluntary rescheduling while holding the
	 * nklock, despite @thread bears the zombie bit.
	 */
	xnsynch_release_all_ownerships(thread);

	giveup_fpu(sched, thread);

	if (moving_target(sched, thread))
		return;

	xnsched_forget(thread);
	xnthread_deregister(thread);
}

void __xnthread_cleanup(struct xnthread *curr)
{
	spl_t s;

	secondary_mode_only();

	xntimer_destroy(&curr->rtimer);
	xntimer_destroy(&curr->ptimer);

	if (curr->selector) {
		xnselector_destroy(curr->selector);
		curr->selector = NULL;
	}

	xnlock_get_irqsave(&nklock, s);
	cleanup_tcb(curr);
	xnlock_put_irqrestore(&nklock, s);

	/* Wake up the joiner if any (we can't have more than one). */
	complete(&curr->exited);

	/* Notify our exit to xnthread_killall() if need be. */
	if (waitqueue_active(&join_all))
		wake_up(&join_all);

	/* Finalize last since this incurs releasing the TCB. */
	xnthread_run_handler_stack(curr, finalize_thread);
}

/*
 * Unwinds xnthread_init() ops for an unmapped thread.  Since the
 * latter must be dormant, it can't be part of any runqueue.
 */
void __xnthread_discard(struct xnthread *thread)
{
	spl_t s;

	secondary_mode_only();

	xntimer_destroy(&thread->rtimer);
	xntimer_destroy(&thread->ptimer);

	xnlock_get_irqsave(&nklock, s);
	if (!list_empty(&thread->glink)) {
		list_del(&thread->glink);
		cobalt_nrthreads--;
		xnvfile_touch_tag(&nkthreadlist_tag);
	}
	xnthread_deregister(thread);
	xnlock_put_irqrestore(&nklock, s);
}

/**
 * @fn void xnthread_init(struct xnthread *thread,const struct xnthread_init_attr *attr,struct xnsched_class *sched_class,const union xnsched_policy_param *sched_param)
 * @brief Initialize a new thread.
 *
 * Initializes a new thread. The thread is left dormant until it is
 * actually started by xnthread_start().
 *
 * @param thread The address of a thread descriptor Cobalt will use to
 * store the thread-specific data.  This descriptor must always be
 * valid while the thread is active therefore it must be allocated in
 * permanent memory. @warning Some architectures may require the
 * descriptor to be properly aligned in memory; this is an additional
 * reason for descriptors not to be laid in the program stack where
 * alignement constraints might not always be satisfied.
 *
 * @param attr A pointer to an attribute block describing the initial
 * properties of the new thread. Members of this structure are defined
 * as follows:
 *
 * - name: An ASCII string standing for the symbolic name of the
 * thread. This name is copied to a safe place into the thread
 * descriptor. This name might be used in various situations by Cobalt
 * for issuing human-readable diagnostic messages, so it is usually a
 * good idea to provide a sensible value here.  NULL is fine though
 * and means "anonymous".
 *
 * - flags: A set of creation flags affecting the operation. The
 * following flags can be part of this bitmask:
 *
 *   - XNSUSP creates the thread in a suspended state. In such a case,
 * the thread shall be explicitly resumed using the xnthread_resume()
 * service for its execution to actually begin, additionally to
 * issuing xnthread_start() for it. This flag can also be specified
 * when invoking xnthread_start() as a starting mode.
 *
 * - XNUSER shall be set if @a thread will be mapped over an existing
 * user-space task. Otherwise, a new kernel host task is created, then
 * paired with the new Xenomai thread.
 *
 * - XNFPU (enable FPU) tells Cobalt that the new thread may use the
 * floating-point unit. XNFPU is implicitly assumed for user-space
 * threads even if not set in @a flags.
 *
 * - affinity: The processor affinity of this thread. Passing
 * CPU_MASK_ALL means "any cpu" from the allowed core affinity mask
 * (cobalt_cpu_affinity). Passing an empty set is invalid.
 *
 * @param sched_class The initial scheduling class the new thread
 * should be assigned to.
 *
 * @param sched_param The initial scheduling parameters to set for the
 * new thread; @a sched_param must be valid within the context of @a
 * sched_class.
 *
 * @return 0 is returned on success. Otherwise, the following error
 * code indicates the cause of the failure:
 *
 * - -EINVAL is returned if @a attr->flags has invalid bits set, or @a
 *   attr->affinity is invalid (e.g. empty).
 *
 * @coretags{secondary-only}
 */
int xnthread_init(struct xnthread *thread,
		  const struct xnthread_init_attr *attr,
		  struct xnsched_class *sched_class,
		  const union xnsched_policy_param *sched_param)
{
	struct xnsched *sched;
	cpumask_t affinity;
	int ret;

	if (attr->flags & ~(XNFPU | XNUSER | XNSUSP))
		return -EINVAL;

	/*
	 * Pick an initial CPU for the new thread which is part of its
	 * affinity mask, and therefore also part of the supported
	 * CPUs. This CPU may change in pin_to_initial_cpu().
	 */
	cpumask_and(&affinity, &attr->affinity, &cobalt_cpu_affinity);
	if (cpumask_empty(&affinity))
		return -EINVAL;

	sched = xnsched_struct(cpumask_first(&affinity));

	ret = __xnthread_init(thread, attr, sched, sched_class, sched_param);
	if (ret)
		return ret;

	trace_cobalt_thread_init(thread, attr, sched_class);

	return 0;
}
EXPORT_SYMBOL_GPL(xnthread_init);

/**
 * @fn int xnthread_start(struct xnthread *thread,const struct xnthread_start_attr *attr)
 * @brief Start a newly created thread.
 *
 * Starts a (newly) created thread, scheduling it for the first
 * time. This call releases the target thread from the XNDORMANT
 * state. This service also sets the initial mode for the new thread.
 *
 * @param thread The descriptor address of the started thread which
 * must have been previously initialized by a call to xnthread_init().
 *
 * @param attr A pointer to an attribute block describing the
 * execution properties of the new thread. Members of this structure
 * are defined as follows:
 *
 * - mode: The initial thread mode. The following flags can be part of
 * this bitmask:
 *
 *   - XNLOCK causes the thread to lock the scheduler when it starts.
 * The target thread will have to call the xnsched_unlock()
 * service to unlock the scheduler. A non-preemptible thread may still
 * block, in which case, the lock is reasserted when the thread is
 * scheduled back in.
 *
 *   - XNSUSP makes the thread start in a suspended state. In such a
 * case, the thread will have to be explicitly resumed using the
 * xnthread_resume() service for its execution to actually begin.
 *
 * - entry: The address of the thread's body routine. In other words,
 * it is the thread entry point.
 *
 * - cookie: A user-defined opaque cookie Cobalt will pass to the
 * emerging thread as the sole argument of its entry point.
 *
 * @retval 0 if @a thread could be started ;
 *
 * @retval -EBUSY if @a thread was not dormant or stopped ;
 *
 * @coretags{task-unrestricted, might-switch}
 */
int xnthread_start(struct xnthread *thread,
		   const struct xnthread_start_attr *attr)
{
	spl_t s;

	xnlock_get_irqsave(&nklock, s);

	if (!xnthread_test_state(thread, XNDORMANT)) {
		xnlock_put_irqrestore(&nklock, s);
		return -EBUSY;
	}

	xnthread_set_state(thread, attr->mode & (XNTHREAD_MODE_BITS | XNSUSP));
	thread->entry = attr->entry;
	thread->cookie = attr->cookie;
	if (attr->mode & XNLOCK)
		thread->lock_count = 1;

	/*
	 * A user-space thread starts immediately Cobalt-wise since we
	 * already have an underlying Linux context for it, so we can
	 * enlist it now to make it visible from the /proc interface.
	 */
	if (xnthread_test_state(thread, XNUSER))
		enlist_new_thread(thread);

	trace_cobalt_thread_start(thread);

	xnthread_resume(thread, XNDORMANT);
	xnsched_run();

	xnlock_put_irqrestore(&nklock, s);

	return 0;
}
EXPORT_SYMBOL_GPL(xnthread_start);

/**
 * @fn void xnthread_set_mode(int clrmask,int setmask)
 * @brief Change control mode of the current thread.
 *
 * Change the control mode of the current thread. The control mode
 * affects several behaviours of the Cobalt core regarding this
 * thread.
 *
 * @param clrmask Clears the corresponding bits from the control mode
 * before setmask is applied. The scheduler lock held by the current
 * thread can be forcibly released by passing the XNLOCK bit in this
 * mask. In this case, the lock nesting count is also reset to zero.
 *
 * @param setmask The new thread mode. The following flags may be set
 * in this bitmask:
 *
 * - XNLOCK makes the current thread non-preemptible by other threads.
 * Unless XNTRAPLB is also set for the thread, the latter may still
 * block, dropping the lock temporarily, in which case, the lock will
 * be reacquired automatically when the thread resumes execution.
 *
 * - XNWARN enables debugging notifications for the current thread.  A
 * SIGDEBUG (Linux-originated) signal is sent when the following
 * atypical or abnormal behavior is detected:
 *
 *    - the current thread switches to secondary mode. Such notification
 *      comes in handy for detecting spurious relaxes.
 *
 *    - CONFIG_XENO_OPT_DEBUG_MUTEX_RELAXED is enabled in the kernel
 *      configuration, and the current thread is sleeping on a Cobalt
 *      mutex currently owned by a thread running in secondary mode,
 *      which reveals a priority inversion.
 *
 *    - the current thread is about to sleep while holding a Cobalt
 *      mutex, and CONFIG_XENO_OPT_DEBUG_MUTEX_SLEEP is enabled in the
 *      kernel configuration. Blocking for acquiring a mutex does not
 *      trigger such a signal though.
 *
 *    - the current thread has both XNTRAPLB and XNLOCK set, and
 *      attempts to block on a Cobalt service, which would cause a
 *      lock break.
 *
 * - XNTRAPLB disallows breaking the scheduler lock. In the default
 * case, a thread which holds the scheduler lock is allowed to drop it
 * temporarily for sleeping. If this mode bit is set, such thread
 * would return immediately with XNBREAK set from
 * xnthread_suspend(). If XNWARN is set for the current thread,
 * SIGDEBUG is sent in addition to raising the break condition.
 *
 * @coretags{primary-only, might-switch}
 *
 * @note Setting @a clrmask and @a setmask to zero leads to a nop,
 * in which case xnthread_set_mode() returns the current mode.
 */
int xnthread_set_mode(int clrmask, int setmask)
{
	int oldmode, lock_count;
	struct xnthread *curr;
	spl_t s;

	primary_mode_only();

	xnlock_get_irqsave(&nklock, s);
	curr = xnsched_current_thread();
	oldmode = xnthread_get_state(curr) & XNTHREAD_MODE_BITS;
	lock_count = curr->lock_count;
	xnthread_clear_state(curr, clrmask & XNTHREAD_MODE_BITS);
	xnthread_set_state(curr, setmask & XNTHREAD_MODE_BITS);
	trace_cobalt_thread_set_mode(curr);

	if (setmask & XNLOCK) {
		if (lock_count == 0)
			xnsched_lock();
	} else if (clrmask & XNLOCK) {
		if (lock_count > 0) {
			curr->lock_count = 0;
			xnthread_clear_localinfo(curr, XNLBALERT);
			xnsched_run();
		}
	}

	xnlock_put_irqrestore(&nklock, s);

	if (lock_count > 0)
		oldmode |= XNLOCK;

	return oldmode;
}
EXPORT_SYMBOL_GPL(xnthread_set_mode);

/**
 * @fn void xnthread_suspend(struct xnthread *thread, int mask,xnticks_t timeout, xntmode_t timeout_mode,struct xnsynch *wchan)
 * @brief Suspend a thread.
 *
 * Suspends the execution of a thread according to a given suspensive
 * condition. This thread will not be eligible for scheduling until it
 * all the pending suspensive conditions set by this service are
 * removed by one or more calls to xnthread_resume().
 *
 * @param thread The descriptor address of the suspended thread.
 *
 * @param mask The suspension mask specifying the suspensive condition
 * to add to the thread's wait mask. Possible values usable by the
 * caller are:
 *
 * - XNSUSP. This flag forcibly suspends a thread, regardless of any
 * resource to wait for. A reverse call to xnthread_resume()
 * specifying the XNSUSP bit must be issued to remove this condition,
 * which is cumulative with other suspension bits.@a wchan should be
 * NULL when using this suspending mode.
 *
 * - XNDELAY. This flags denotes a counted delay wait (in ticks) which
 * duration is defined by the value of the timeout parameter.
 *
 * - XNPEND. This flag denotes a wait for a synchronization object to
 * be signaled. The wchan argument must points to this object. A
 * timeout value can be passed to bound the wait. This suspending mode
 * should not be used directly by the client interface, but rather
 * through the xnsynch_sleep_on() call.
 *
 * @param timeout The timeout which may be used to limit the time the
 * thread pends on a resource. This value is a wait time given in
 * nanoseconds. It can either be relative, absolute monotonic, or
 * absolute adjustable depending on @a timeout_mode.
 *
 * Passing XN_INFINITE @b and setting @a timeout_mode to XN_RELATIVE
 * specifies an unbounded wait. All other values are used to
 * initialize a watchdog timer. If the current operation mode of the
 * system timer is oneshot and @a timeout elapses before
 * xnthread_suspend() has completed, then the target thread will not
 * be suspended, and this routine leads to a null effect.
 *
 * @param timeout_mode The mode of the @a timeout parameter. It can
 * either be set to XN_RELATIVE, XN_ABSOLUTE, or XN_REALTIME (see also
 * xntimer_start()).
 *
 * @param wchan The address of a pended resource. This parameter is
 * used internally by the synchronization object implementation code
 * to specify on which object the suspended thread pends. NULL is a
 * legitimate value when this parameter does not apply to the current
 * suspending mode (e.g. XNSUSP).
 *
 * @note If the target thread has received a Linux-originated signal,
 * then this service immediately exits without suspending the thread,
 * but raises the XNBREAK condition in its information mask.
 *
 * @coretags{unrestricted, might-switch}
 */
void xnthread_suspend(struct xnthread *thread, int mask,
		      xnticks_t timeout, xntmode_t timeout_mode,
		      struct xnsynch *wchan)
{
	unsigned long oldstate;
	struct xnsched *sched;
	spl_t s;

	/* No, you certainly do not want to suspend the root thread. */
	XENO_BUG_ON(COBALT, xnthread_test_state(thread, XNROOT));
	/* No built-in support for conjunctive wait. */
	XENO_BUG_ON(COBALT, wchan && thread->wchan);

	xnlock_get_irqsave(&nklock, s);

	trace_cobalt_thread_suspend(thread, mask, timeout, timeout_mode, wchan);

	sched = thread->sched;
	oldstate = thread->state;

	/*
	 * If attempting to suspend a runnable thread which is pending
	 * a forced switch to secondary mode (XNKICKED), just raise
	 * the XNBREAK status and return immediately, except if we
	 * are precisely doing such switch by applying XNRELAX.
	 *
	 * In the latter case, we also make sure to clear XNKICKED,
	 * since we won't go through prepare_for_signal() once
	 * relaxed.
	 */
	if (likely((oldstate & XNTHREAD_BLOCK_BITS) == 0)) {
		if (likely((mask & XNRELAX) == 0)) {
			if (xnthread_test_info(thread, XNKICKED))
				goto abort;
			if (thread == sched->curr &&
			    thread->lock_count > 0 &&
			    (oldstate & XNTRAPLB) != 0)
				goto lock_break;
		}
		/*
		 * Do not destroy the info left behind by yet unprocessed
		 * wakeups when suspending a remote thread.
		 */
		if (thread == sched->curr)
			xnthread_clear_info(thread, XNRMID|XNTIMEO|XNBREAK|
						    XNWAKEN|XNROBBED|XNKICKED);
	}

	/*
	 * Don't start the timer for a thread delayed indefinitely.
	 */
	if (timeout != XN_INFINITE || timeout_mode != XN_RELATIVE) {
		xntimer_set_sched(&thread->rtimer, thread->sched);
		if (xntimer_start(&thread->rtimer, timeout, XN_INFINITE,
				  timeout_mode)) {
			/* (absolute) timeout value in the past, bail out. */
			if (wchan) {
				thread->wchan = wchan;
				xnsynch_forget_sleeper(thread);
			}
			xnthread_set_info(thread, XNTIMEO);
			goto out;
		}
		xnthread_set_state(thread, XNDELAY);
	}

	if (oldstate & XNREADY) {
		xnsched_dequeue(thread);
		xnthread_clear_state(thread, XNREADY);
	}

	xnthread_set_state(thread, mask);

	/*
	 * We must make sure that we don't clear the wait channel if a
	 * thread is first blocked (wchan != NULL) then forcibly
	 * suspended (wchan == NULL), since these are conjunctive
	 * conditions.
	 */
	if (wchan)
		thread->wchan = wchan;

	/*
	 * If the current thread is being relaxed, we must have been
	 * called from xnthread_relax(), in which case we introduce an
	 * opportunity for interrupt delivery right before switching
	 * context, which shortens the uninterruptible code path.
	 *
	 * We have to shut irqs off before __xnsched_run() though: if
	 * an interrupt could preempt us in ___xnsched_run() right
	 * after the call to xnarch_escalate() but before we grab the
	 * nklock, we would enter the critical section in
	 * xnsched_run() while running in secondary mode, which would
	 * defeat the purpose of xnarch_escalate().
	 */
	if (likely(thread == sched->curr)) {
		xnsched_set_resched(sched);
		if (unlikely(mask & XNRELAX)) {
			xnlock_clear_irqon(&nklock);
			splmax();
			__xnsched_run(sched);
			return;
		}
		/*
		 * If the thread is runnning on another CPU,
		 * xnsched_run will trigger the IPI as required.
		 */
		__xnsched_run(sched);
		goto out;
	}

	/*
	 * Ok, this one is an interesting corner case, which requires
	 * a bit of background first. Here, we handle the case of
	 * suspending a _relaxed_ user shadow which is _not_ the
	 * current thread.
	 *
	 *  The net effect is that we are attempting to stop the
	 * shadow thread for Cobalt, whilst this thread is actually
	 * running some code under the control of the Linux scheduler
	 * (i.e. it's relaxed).
	 *
	 *  To make this possible, we force the target Linux task to
	 * migrate back to the Xenomai domain by sending it a
	 * SIGSHADOW signal the interface libraries trap for this
	 * specific internal purpose, whose handler is expected to
	 * call back Cobalt's migration service.
	 *
	 * By forcing this migration, we make sure that Cobalt
	 * controls, hence properly stops, the target thread according
	 * to the requested suspension condition. Otherwise, the
	 * shadow thread in secondary mode would just keep running
	 * into the Linux domain, thus breaking the most common
	 * assumptions regarding suspended threads.
	 *
	 * We only care for threads that are not current, and for
	 * XNSUSP, XNDELAY, XNDORMANT and XNHELD conditions, because:
	 *
	 * - There is no point in dealing with relaxed threads, since
	 * personalities have to ask for primary mode switch when
	 * processing any syscall which may block the caller
	 * (i.e. __xn_exec_primary).
	 *
	 * - among all blocking bits (XNTHREAD_BLOCK_BITS), only
	 * XNSUSP, XNDELAY and XNHELD may be applied by the current
	 * thread to a non-current thread. XNPEND is always added by
	 * the caller to its own state, XNMIGRATE and XNRELAX have
	 * special semantics escaping this issue.
	 *
	 * We don't signal threads which are already in a dormant
	 * state, since they are suspended by definition.
	 */
	if (((oldstate & (XNTHREAD_BLOCK_BITS|XNUSER)) == (XNRELAX|XNUSER)) &&
	    (mask & (XNDELAY | XNSUSP | XNHELD)) != 0)
		xnthread_signal(thread, SIGSHADOW, SIGSHADOW_ACTION_HARDEN);
out:
	xnlock_put_irqrestore(&nklock, s);
	return;

lock_break:
	/* NOTE: thread is current */
	if (xnthread_test_state(thread, XNWARN) &&
	    !xnthread_test_localinfo(thread, XNLBALERT)) {
		xnthread_set_info(thread, XNKICKED);
		xnthread_set_localinfo(thread, XNLBALERT);
		xnthread_signal(thread, SIGDEBUG, SIGDEBUG_LOCK_BREAK);
	}
abort:
	if (wchan) {
		thread->wchan = wchan;
		xnsynch_forget_sleeper(thread);
	}
	xnthread_clear_info(thread, XNRMID | XNTIMEO);
	xnthread_set_info(thread, XNBREAK);
	xnlock_put_irqrestore(&nklock, s);
}
EXPORT_SYMBOL_GPL(xnthread_suspend);

/**
 * @fn void xnthread_resume(struct xnthread *thread,int mask)
 * @brief Resume a thread.
 *
 * Resumes the execution of a thread previously suspended by one or
 * more calls to xnthread_suspend(). This call removes a suspensive
 * condition affecting the target thread. When all suspensive
 * conditions are gone, the thread is left in a READY state at which
 * point it becomes eligible anew for scheduling.
 *
 * @param thread The descriptor address of the resumed thread.
 *
 * @param mask The suspension mask specifying the suspensive condition
 * to remove from the thread's wait mask. Possible values usable by
 * the caller are:
 *
 * - XNSUSP. This flag removes the explicit suspension condition. This
 * condition might be additive to the XNPEND condition.
 *
 * - XNDELAY. This flag removes the counted delay wait condition.
 *
 * - XNPEND. This flag removes the resource wait condition. If a
 * watchdog is armed, it is automatically disarmed by this
 * call. Unlike the two previous conditions, only the current thread
 * can set this condition for itself, i.e. no thread can force another
 * one to pend on a resource.
 *
 * When the thread is eventually resumed by one or more calls to
 * xnthread_resume(), the caller of xnthread_suspend() in the awakened
 * thread that suspended itself should check for the following bits in
 * its own information mask to determine what caused its wake up:
 *
 * - XNRMID means that the caller must assume that the pended
 * synchronization object has been destroyed (see xnsynch_flush()).
 *
 * - XNTIMEO means that the delay elapsed, or the watchdog went off
 * before the corresponding synchronization object was signaled.
 *
 * - XNBREAK means that the wait has been forcibly broken by a call to
 * xnthread_unblock().
 *
 * @coretags{unrestricted, might-switch}
 */
void xnthread_resume(struct xnthread *thread, int mask)
{
	unsigned long oldstate;
	struct xnsched *sched;
	spl_t s;

	xnlock_get_irqsave(&nklock, s);

	trace_cobalt_thread_resume(thread, mask);

	xntrace_pid(xnthread_host_pid(thread), xnthread_current_priority(thread));

	sched = thread->sched;
	oldstate = thread->state;

	if ((oldstate & XNTHREAD_BLOCK_BITS) == 0) {
		if (oldstate & XNREADY)
			xnsched_dequeue(thread);
		goto enqueue;
	}

	/* Clear the specified block bit(s) */
	xnthread_clear_state(thread, mask);

	/*
	 * If XNDELAY was set in the clear mask, xnthread_unblock()
	 * was called for the thread, or a timeout has elapsed. In the
	 * latter case, stopping the timer is a no-op.
	 */
	if (mask & XNDELAY)
		xntimer_stop(&thread->rtimer);

	if (!xnthread_test_state(thread, XNTHREAD_BLOCK_BITS))
		goto clear_wchan;

	if (mask & XNDELAY) {
		mask = xnthread_test_state(thread, XNPEND);
		if (mask == 0)
			goto unlock_and_exit;
		if (thread->wchan)
			xnsynch_forget_sleeper(thread);
		goto recheck_state;
	}

	if (xnthread_test_state(thread, XNDELAY)) {
		if (mask & XNPEND) {
			/*
			 * A resource became available to the thread.
			 * Cancel the watchdog timer.
			 */
			xntimer_stop(&thread->rtimer);
			xnthread_clear_state(thread, XNDELAY);
		}
		goto recheck_state;
	}

	/*
	 * The thread is still suspended, but is no more pending on a
	 * resource.
	 */
	if ((mask & XNPEND) != 0 && thread->wchan)
		xnsynch_forget_sleeper(thread);

	goto unlock_and_exit;

recheck_state:
	if (xnthread_test_state(thread, XNTHREAD_BLOCK_BITS))
		goto unlock_and_exit;

clear_wchan:
	if ((mask & ~XNDELAY) != 0 && thread->wchan != NULL)
		/*
		 * If the thread was actually suspended, clear the
		 * wait channel.  -- this allows requests like
		 * xnthread_suspend(thread,XNDELAY,...)  not to run
		 * the following code when the suspended thread is
		 * woken up while undergoing a simple delay.
		 */
		xnsynch_forget_sleeper(thread);

	if (unlikely((oldstate & mask) & XNHELD)) {
		xnsched_requeue(thread);
		goto ready;
	}
enqueue:
	xnsched_enqueue(thread);
ready:
	xnthread_set_state(thread, XNREADY);
	xnsched_set_resched(sched);
unlock_and_exit:
	xnlock_put_irqrestore(&nklock, s);
}
EXPORT_SYMBOL_GPL(xnthread_resume);

/**
 * @fn int xnthread_unblock(struct xnthread *thread)
 * @brief Unblock a thread.
 *
 * Breaks the thread out of any wait it is currently in.  This call
 * removes the XNDELAY and XNPEND suspensive conditions previously put
 * by xnthread_suspend() on the target thread. If all suspensive
 * conditions are gone, the thread is left in a READY state at which
 * point it becomes eligible anew for scheduling.
 *
 * @param thread The descriptor address of the unblocked thread.
 *
 * This call neither releases the thread from the XNSUSP, XNRELAX,
 * XNDORMANT or XNHELD suspensive conditions.
 *
 * When the thread resumes execution, the XNBREAK bit is set in the
 * unblocked thread's information mask. Unblocking a non-blocked
 * thread is perfectly harmless.
 *
 * @return non-zero is returned if the thread was actually unblocked
 * from a pending wait state, 0 otherwise.
 *
 * @coretags{unrestricted, might-switch}
 */
int xnthread_unblock(struct xnthread *thread)
{
	int ret = 1;
	spl_t s;

	/*
	 * Attempt to abort an undergoing wait for the given thread.
	 * If this state is due to an alarm that has been armed to
	 * limit the sleeping thread's waiting time while it pends for
	 * a resource, the corresponding XNPEND state will be cleared
	 * by xnthread_resume() in the same move. Otherwise, this call
	 * may abort an undergoing infinite wait for a resource (if
	 * any).
	 */
	xnlock_get_irqsave(&nklock, s);

	trace_cobalt_thread_unblock(thread);

	if (xnthread_test_state(thread, XNDELAY))
		xnthread_resume(thread, XNDELAY);
	else if (xnthread_test_state(thread, XNPEND))
		xnthread_resume(thread, XNPEND);
	else
		ret = 0;

	/*
	 * We should not clear a previous break state if this service
	 * is called more than once before the target thread actually
	 * resumes, so we only set the bit here and never clear
	 * it. However, we must not raise the XNBREAK bit if the
	 * target thread was already awake at the time of this call,
	 * so that downstream code does not get confused by some
	 * "successful but interrupted syscall" condition. IOW, a
	 * break state raised here must always trigger an error code
	 * downstream, and an already successful syscall cannot be
	 * marked as interrupted.
	 */
	if (ret)
		xnthread_set_info(thread, XNBREAK);

	xnlock_put_irqrestore(&nklock, s);

	return ret;
}
EXPORT_SYMBOL_GPL(xnthread_unblock);

/**
 * @fn int xnthread_set_periodic(struct xnthread *thread,xnticks_t idate, xntmode_t timeout_mode, xnticks_t period)
 * @brief Make a thread periodic.
 *
 * Make a thread periodic by programming its first release point and
 * its period in the processor time line.  Subsequent calls to
 * xnthread_wait_period() will delay the thread until the next
 * periodic release point in the processor timeline is reached.
 *
 * @param thread The core thread to make periodic. If NULL, the
 * current thread is assumed.
 *
 * @param idate The initial (absolute) date of the first release
 * point, expressed in nanoseconds. The affected thread will be
 * delayed by the first call to xnthread_wait_period() until this
 * point is reached. If @a idate is equal to XN_INFINITE, the first
 * release point is set to @a period nanoseconds after the current
 * date. In the latter case, @a timeout_mode is not considered and can
 * have any valid value.
 *
 * @param timeout_mode The mode of the @a idate parameter. It can
 * either be set to XN_ABSOLUTE or XN_REALTIME with @a idate different
 * from XN_INFINITE (see also xntimer_start()).
 *
 * @param period The period of the thread, expressed in nanoseconds.
 * As a side-effect, passing XN_INFINITE attempts to stop the thread's
 * periodic timer; in the latter case, the routine always exits
 * succesfully, regardless of the previous state of this timer.
 *
 * @return 0 is returned upon success. Otherwise:
 *
 * - -ETIMEDOUT is returned @a idate is different from XN_INFINITE and
 * represents a date in the past.
 *
 * - -EINVAL is returned if @a period is different from XN_INFINITE
 * but shorter than the scheduling latency value for the target
 * system, as available from /proc/xenomai/latency. -EINVAL is also
 * returned if @a timeout_mode is not compatible with @a idate, such
 * as XN_RELATIVE with @a idate different from XN_INFINITE.
 *
 * - -EPERM is returned if @a thread is NULL, but the caller is not a
 * Xenomai thread.
 *
 * @coretags{task-unrestricted}
 */
int xnthread_set_periodic(struct xnthread *thread, xnticks_t idate,
			  xntmode_t timeout_mode, xnticks_t period)
{
	int ret = 0, cpu;
	spl_t s;

	if (thread == NULL) {
		thread = xnthread_current();
		if (thread == NULL)
			return -EPERM;
	}
		
	xnlock_get_irqsave(&nklock, s);

	if (period == XN_INFINITE) {
		if (xntimer_running_p(&thread->ptimer))
			xntimer_stop(&thread->ptimer);

		goto unlock_and_exit;
	}

	/*
	 * LART: detect periods which are shorter than the core clock
	 * gravity for kernel thread timers. This can't work, caller
	 * must have messed up arguments.
	 */
	if (period < xnclock_ticks_to_ns(&nkclock,
			 xnclock_get_gravity(&nkclock, kernel))) {
		ret = -EINVAL;
		goto unlock_and_exit;
	}

	/*
	 * Pin the periodic timer to a proper CPU, by order of
	 * preference: the CPU the timed thread runs on if possible,
	 * or the first CPU by logical number which can receive events
	 * from the clock device backing the timer, among the dynamic
	 * set of real-time CPUs currently enabled.
	 */
	cpu = xnclock_get_default_cpu(xntimer_clock(&thread->ptimer),
				      xnsched_cpu(thread->sched));
	xntimer_set_sched(&thread->ptimer, xnsched_struct(cpu));

	if (idate == XN_INFINITE)
		xntimer_start(&thread->ptimer, period, period, XN_RELATIVE);
	else {
		if (timeout_mode == XN_REALTIME)
			idate -= xnclock_get_offset(xntimer_clock(&thread->ptimer));
		else if (timeout_mode != XN_ABSOLUTE) {
			ret = -EINVAL;
			goto unlock_and_exit;
		}
		ret = xntimer_start(&thread->ptimer, idate, period,
				    XN_ABSOLUTE);
	}

unlock_and_exit:
	xnlock_put_irqrestore(&nklock, s);

	return ret;
}
EXPORT_SYMBOL_GPL(xnthread_set_periodic);

/**
 * @fn int xnthread_wait_period(unsigned long *overruns_r)
 * @brief Wait for the next periodic release point.
 *
 * Make the current thread wait for the next periodic release point in
 * the processor time line.
 *
 * @param overruns_r If non-NULL, @a overruns_r must be a pointer to a
 * memory location which will be written with the count of pending
 * overruns. This value is copied only when xnthread_wait_period()
 * returns -ETIMEDOUT or success; the memory location remains
 * unmodified otherwise. If NULL, this count will never be copied
 * back.
 *
 * @return 0 is returned upon success; if @a overruns_r is valid, zero
 * is copied to the pointed memory location. Otherwise:
 *
 * - -EWOULDBLOCK is returned if xnthread_set_periodic() has not
 * previously been called for the calling thread.
 *
 * - -EINTR is returned if xnthread_unblock() has been called for the
 * waiting thread before the next periodic release point has been
 * reached. In this case, the overrun counter is reset too.
 *
 * - -ETIMEDOUT is returned if the timer has overrun, which indicates
 * that one or more previous release points have been missed by the
 * calling thread. If @a overruns_r is valid, the count of pending
 * overruns is copied to the pointed memory location.
 *
 * @coretags{primary-only, might-switch}
 */
int xnthread_wait_period(unsigned long *overruns_r)
{
	unsigned long overruns = 0;
	struct xnthread *thread;
	struct xnclock *clock;
	xnticks_t now;
	int ret = 0;
	spl_t s;

	thread = xnthread_current();

	xnlock_get_irqsave(&nklock, s);

	if (unlikely(!xntimer_running_p(&thread->ptimer))) {
		ret = -EWOULDBLOCK;
		goto out;
	}

	trace_cobalt_thread_wait_period(thread);

	clock = xntimer_clock(&thread->ptimer);
	now = xnclock_read_raw(clock);
	if (likely((xnsticks_t)(now - xntimer_pexpect(&thread->ptimer)) < 0)) {
		xnthread_suspend(thread, XNDELAY, XN_INFINITE, XN_RELATIVE, NULL);
		if (unlikely(xnthread_test_info(thread, XNBREAK))) {
			ret = -EINTR;
			goto out;
		}

		now = xnclock_read_raw(clock);
	}

	overruns = xntimer_get_overruns(&thread->ptimer, now);
	if (overruns) {
		ret = -ETIMEDOUT;
		trace_cobalt_thread_missed_period(thread);
	}

	if (likely(overruns_r != NULL))
		*overruns_r = overruns;
 out:
	xnlock_put_irqrestore(&nklock, s);

	return ret;
}
EXPORT_SYMBOL_GPL(xnthread_wait_period);

/**
 * @fn int xnthread_set_slice(struct xnthread *thread, xnticks_t quantum)
 * @brief Set thread time-slicing information.
 *
 * Update the time-slicing information for a given thread. This
 * service enables or disables round-robin scheduling for the thread,
 * depending on the value of @a quantum. By default, times-slicing is
 * disabled for a new thread initialized by a call to xnthread_init().
 *
 * @param thread The descriptor address of the affected thread.
 *
 * @param quantum The time quantum assigned to the thread expressed in
 * nanoseconds. If @a quantum is different from XN_INFINITE, the
 * time-slice for the thread is set to that value and its current time
 * credit is refilled (i.e. the thread is given a full time-slice to
 * run next). Otherwise, if @a quantum equals XN_INFINITE,
 * time-slicing is stopped for that thread.
 *
 * @return 0 is returned upon success. Otherwise, -EINVAL is returned
 * if @a quantum is not XN_INFINITE and:
 *
 *   - the base scheduling class of the target thread does not support
 *   time-slicing,
 *
 *   - @a quantum is smaller than the master clock gravity for a user
 * thread, which denotes a spurious value.
 *
 * @coretags{task-unrestricted}
 */
int xnthread_set_slice(struct xnthread *thread, xnticks_t quantum)
{
	struct xnsched *sched;
	spl_t s;

	if (quantum <= xnclock_get_gravity(&nkclock, user))
		return -EINVAL;

	xnlock_get_irqsave(&nklock, s);

	sched = thread->sched;
	thread->rrperiod = quantum;

	if (quantum != XN_INFINITE) {
		if (thread->base_class->sched_tick == NULL) {
			xnlock_put_irqrestore(&nklock, s);
			return -EINVAL;
		}
		xnthread_set_state(thread, XNRRB);
		if (sched->curr == thread)
			xntimer_start(&sched->rrbtimer,
				      quantum, XN_INFINITE, XN_RELATIVE);
	} else {
		xnthread_clear_state(thread, XNRRB);
		if (sched->curr == thread)
			xntimer_stop(&sched->rrbtimer);
	}

	xnlock_put_irqrestore(&nklock, s);

	return 0;
}
EXPORT_SYMBOL_GPL(xnthread_set_slice);

/**
 * @fn void xnthread_cancel(struct xnthread *thread)
 * @brief Cancel a thread.
 *
 * Request cancellation of a thread. This service forces @a thread to
 * exit from any blocking call, then to switch to secondary mode.
 * @a thread will terminate as soon as it reaches a cancellation
 * point. Cancellation points are defined for the following
 * situations:
 *
 * - @a thread self-cancels by a call to xnthread_cancel().
 * - @a thread invokes a Linux syscall (user-space shadow only).
 * - @a thread receives a Linux signal (user-space shadow only).
 * - @a thread unblocks from a Xenomai syscall (user-space shadow only).
 * - @a thread attempts to block on a Xenomai syscall (user-space shadow only).
 * - @a thread explicitly calls xnthread_test_cancel().
 *
 * @param thread The descriptor address of the thread to terminate.
 *
 * @coretags{task-unrestricted, might-switch}
 *
 * @note In addition to the common actions taken upon cancellation, a
 * thread which belongs to the SCHED_WEAK class is sent a regular
 * SIGTERM signal.
 */
void xnthread_cancel(struct xnthread *thread)
{
	spl_t s;

	/* Right, so you want to kill the kernel?! */
	XENO_BUG_ON(COBALT, xnthread_test_state(thread, XNROOT));

	xnlock_get_irqsave(&nklock, s);

	if (xnthread_test_info(thread, XNCANCELD))
		goto check_self_cancel;

	trace_cobalt_thread_cancel(thread);

	xnthread_set_info(thread, XNCANCELD);

	/*
	 * If @thread is not started yet, fake a start request,
	 * raising the kicked condition bit to make sure it will reach
	 * xnthread_test_cancel() on its wakeup path.
	 */
	if (xnthread_test_state(thread, XNDORMANT)) {
		xnthread_set_info(thread, XNKICKED);
		xnthread_resume(thread, XNDORMANT);
		goto out;
	}

check_self_cancel:
	if (xnthread_current() == thread) {
		xnlock_put_irqrestore(&nklock, s);
		xnthread_test_cancel();
		/*
		 * May return if on behalf of an IRQ handler which has
		 * preempted @thread.
		 */
		return;
	}

	/*
	 * Force the non-current thread to exit:
	 *
	 * - unblock a user thread, switch it to weak scheduling,
	 * then send it SIGTERM.
	 *
	 * - just unblock a kernel thread, it is expected to reach a
	 * cancellation point soon after
	 * (i.e. xnthread_test_cancel()).
	 */
	if (xnthread_test_state(thread, XNUSER)) {
		__xnthread_demote(thread);
		xnthread_signal(thread, SIGTERM, 0);
	} else
		__xnthread_kick(thread);
out:
	xnlock_put_irqrestore(&nklock, s);

	xnsched_run();
}
EXPORT_SYMBOL_GPL(xnthread_cancel);

struct wait_grace_struct {
	struct completion done;
	struct rcu_head rcu;
};

static void grace_elapsed(struct rcu_head *head)
{
	struct wait_grace_struct *wgs;

	wgs = container_of(head, struct wait_grace_struct, rcu);
	complete(&wgs->done);
}

static void wait_for_rcu_grace_period(struct pid *pid)
{
	struct wait_grace_struct wait = {
		.done = COMPLETION_INITIALIZER_ONSTACK(wait.done),
	};
	struct task_struct *p;

	init_rcu_head_on_stack(&wait.rcu);
	
	for (;;) {
		call_rcu(&wait.rcu, grace_elapsed);
		wait_for_completion(&wait.done);
		if (pid == NULL)
			break;
		rcu_read_lock();
		p = pid_task(pid, PIDTYPE_PID);
		rcu_read_unlock();
		if (p == NULL)
			break;
		reinit_completion(&wait.done);
	}
}

/**
 * @fn void xnthread_join(struct xnthread *thread, bool uninterruptible)
 * @brief Join with a terminated thread.
 *
 * This service waits for @a thread to terminate after a call to
 * xnthread_cancel().  If that thread has already terminated or is
 * dormant at the time of the call, then xnthread_join() returns
 * immediately.
 *
 * xnthread_join() adapts to the calling context (primary or
 * secondary), switching to secondary mode if needed for the duration
 * of the wait. Upon return, the original runtime mode is restored,
 * unless a Linux signal is pending.
 *
 * @param thread The descriptor address of the thread to join with.
 *
 * @param uninterruptible Boolean telling whether the service should
 * wait for completion uninterruptible.
 *
 * @return 0 is returned on success. Otherwise, the following error
 * codes indicate the cause of the failure:
 *
 * - -EDEADLK is returned if the current thread attempts to join
 * itself.
 *
 * - -EINTR is returned if the current thread was unblocked while
 *   waiting for @a thread to terminate.
 *
 * - -EBUSY indicates that another thread is already waiting for @a
 *   thread to terminate.
 *
 * @coretags{task-unrestricted, might-switch}
 */
int xnthread_join(struct xnthread *thread, bool uninterruptible)
{
	struct xnthread *curr = xnthread_current();
	int ret = 0, switched = 0;
	struct pid *pid;
	pid_t tpid;
	spl_t s;

	XENO_BUG_ON(COBALT, xnthread_test_state(thread, XNROOT));

	if (thread == curr)
		return -EDEADLK;

	xnlock_get_irqsave(&nklock, s);

	if (xnthread_test_state(thread, XNJOINED)) {
		ret = -EBUSY;
		goto out;
	}

	if (xnthread_test_info(thread, XNDORMANT))
		goto out;

	trace_cobalt_thread_join(thread);

	xnthread_set_state(thread, XNJOINED);
	tpid = xnthread_host_pid(thread);
	
	if (curr && !xnthread_test_state(curr, XNRELAX)) {
		xnlock_put_irqrestore(&nklock, s);
		xnthread_relax(0, 0);
		switched = 1;
	} else
		xnlock_put_irqrestore(&nklock, s);

	/*
	 * Since in theory, we might be sleeping there for a long
	 * time, we get a reference on the pid struct holding our
	 * target, then we check for its existence upon wake up.
	 */
	pid = find_get_pid(tpid);
	if (pid == NULL)
		goto done;

	/*
	 * We have a tricky issue to deal with, which involves code
	 * relying on the assumption that a destroyed thread will have
	 * scheduled away from do_exit() before xnthread_join()
	 * returns. A typical example is illustrated by the following
	 * sequence, with a RTDM kernel task implemented in a
	 * dynamically loaded module:
	 *
	 * CPU0:  rtdm_task_destroy(ktask)
	 *           xnthread_cancel(ktask)
	 *           xnthread_join(ktask)
	 *        ...<back to user>..
	 *        rmmod(module)
	 *
	 * CPU1:  in ktask()
	 *        ...
	 *        ...
	 *          __xnthread_test_cancel()
	 *             do_exit()
         *                schedule()
	 *
	 * In such a sequence, the code on CPU0 would expect the RTDM
	 * task to have scheduled away upon return from
	 * rtdm_task_destroy(), so that unmapping the destroyed task
	 * code and data memory when unloading the module is always
	 * safe.
	 *
	 * To address this, the joiner first waits for the joinee to
	 * signal completion from the Cobalt thread cleanup handler
	 * (__xnthread_cleanup), then waits for a full RCU grace
	 * period to have elapsed. Since the completion signal is sent
	 * on behalf of do_exit(), we may assume that the joinee has
	 * scheduled away before the RCU grace period ends.
	 */
	if (uninterruptible)
		wait_for_completion(&thread->exited);
	else {
		ret = wait_for_completion_interruptible(&thread->exited);
		if (ret < 0) {
			put_pid(pid);
			return -EINTR;
		}
	}

	/* Make sure the joinee has scheduled away ultimately. */
	wait_for_rcu_grace_period(pid);

	put_pid(pid);
done:
	ret = 0;
	if (switched)
		ret = xnthread_harden();

	return ret;
out:
	xnlock_put_irqrestore(&nklock, s);

	return ret;
}
EXPORT_SYMBOL_GPL(xnthread_join);

#ifdef CONFIG_SMP

/**
 * @fn int xnthread_migrate(int cpu)
 * @brief Migrate the current thread.
 *
 * This call makes the current thread migrate to another (real-time)
 * CPU if its affinity allows it. This call is available from
 * primary mode only.
 *
 * @param cpu The destination CPU.
 *
 * @retval 0 if the thread could migrate ;
 * @retval -EPERM if the calling context is invalid, or the
 * scheduler is locked.
 * @retval -EINVAL if the current thread affinity forbids this
 * migration.
 *
 * @coretags{primary-only, might-switch}
 */
int xnthread_migrate(int cpu)
{
	struct xnthread *curr;
	struct xnsched *sched;
	int ret = 0;
	spl_t s;

	xnlock_get_irqsave(&nklock, s);

	curr = xnthread_current();
	if (!xnsched_primary_p() || curr->lock_count > 0) {
		ret = -EPERM;
		goto unlock_and_exit;
	}

	if (!cpumask_test_cpu(cpu, &curr->affinity)) {
		ret = -EINVAL;
		goto unlock_and_exit;
	}

	sched = xnsched_struct(cpu);
	if (sched == curr->sched)
		goto unlock_and_exit;

	trace_cobalt_thread_migrate(curr, cpu);

	/* Move to remote scheduler. */
	xnsched_migrate(curr, sched);

	/*
	 * Migrate the thread's periodic timer. We don't have to care
	 * about the resource timer, since we can only deal with the
	 * current thread, which is, well, running, so it can't be
	 * sleeping on any timed wait at the moment.
	 */
	__xntimer_migrate(&curr->ptimer, sched);

	/*
	 * Reset execution time measurement period so that we don't
	 * mess up per-CPU statistics.
	 */
	xnstat_exectime_reset_stats(&curr->stat.lastperiod);

	/*
	 * So that xnthread_relax() will pin the linux mate on the
	 * same CPU next time the thread switches to secondary mode.
	 */
	xnthread_set_localinfo(curr, XNMOVED);

	xnsched_run();

 unlock_and_exit:
	xnlock_put_irqrestore(&nklock, s);

	return ret;
}
EXPORT_SYMBOL_GPL(xnthread_migrate);

void xnthread_migrate_passive(struct xnthread *thread, struct xnsched *sched)
{				/* nklocked, IRQs off */
	if (thread->sched == sched)
		return;

	trace_cobalt_thread_migrate_passive(thread, xnsched_cpu(sched));
	/*
	 * Timer migration is postponed until the next timeout happens
	 * for the periodic and rrb timers. The resource timer will be
	 * moved to the right CPU next time it is armed in
	 * xnthread_suspend().
	 */
	xnsched_migrate_passive(thread, sched);

	xnstat_exectime_reset_stats(&thread->stat.lastperiod);
}

#endif	/* CONFIG_SMP */

/**
 * @fn int xnthread_set_schedparam(struct xnthread *thread,struct xnsched_class *sched_class,const union xnsched_policy_param *sched_param)
 * @brief Change the base scheduling parameters of a thread.
 *
 * Changes the base scheduling policy and paramaters of a thread. If
 * the thread is currently blocked, waiting in priority-pending mode
 * (XNSYNCH_PRIO) for a synchronization object to be signaled, Cobalt
 * will attempt to reorder the object's wait queue so that it reflects
 * the new sleeper's priority, unless the XNSYNCH_DREORD flag has been
 * set for the pended object.
 *
 * @param thread The descriptor address of the affected thread. See
 * note.
 *
 * @param sched_class The new scheduling class the thread should be
 * assigned to.
 *
 * @param sched_param The scheduling parameters to set for the thread;
 * @a sched_param must be valid within the context of @a sched_class.
 *
 * It is absolutely required to use this service to change a thread
 * priority, in order to have all the needed housekeeping chores
 * correctly performed. i.e. Do *not* call xnsched_set_policy()
 * directly or worse, change the thread.cprio field by hand in any
 * case.
 *
 * @return 0 is returned on success. Otherwise, a negative error code
 * indicates the cause of a failure that happened in the scheduling
 * class implementation for @a sched_class. Invalid parameters passed
 * into @a sched_param are common causes of error.
 *
 * @sideeffect
 *
 * - This service does not call the rescheduling procedure but may
 * affect the state of the runnable queue for the previous and new
 * scheduling classes.
 *
 * - Assigning the same scheduling class and parameters to a running
 * or ready thread moves it to the end of the runnable queue, thus
 * causing a manual round-robin.
 *
 * @coretags{task-unregistred}
 *
 * @note The changes only apply to the Xenomai scheduling parameters
 * for @a thread. There is no propagation/translation of such changes
 * to the Linux scheduler for the task mated to the Xenomai target
 * thread.
 */
int xnthread_set_schedparam(struct xnthread *thread,
			    struct xnsched_class *sched_class,
			    const union xnsched_policy_param *sched_param)
{
	spl_t s;
	int ret;

	xnlock_get_irqsave(&nklock, s);
	ret = __xnthread_set_schedparam(thread, sched_class, sched_param);
	xnlock_put_irqrestore(&nklock, s);

	return ret;
}
EXPORT_SYMBOL_GPL(xnthread_set_schedparam);

int __xnthread_set_schedparam(struct xnthread *thread,
			      struct xnsched_class *sched_class,
			      const union xnsched_policy_param *sched_param)
{
	int old_wprio, new_wprio, ret;

	/*
	 * NOTE: we do not prevent the caller from altering the
	 * scheduling parameters of a thread that currently undergoes
	 * a PIP boost.
	 *
	 * Rationale: Calling xnthread_set_schedparam() carelessly
	 * with no consideration for resource management is a bug in
	 * essence, and xnthread_set_schedparam() does not have to
	 * paper over it, especially at the cost of more complexity
	 * when dealing with multiple scheduling classes.
	 *
	 * In short, callers have to make sure that lowering a thread
	 * priority is safe with respect to what their application
	 * currently does.
	 */
	old_wprio = thread->wprio;

	ret = xnsched_set_policy(thread, sched_class, sched_param);
	if (ret)
		return ret;

	new_wprio = thread->wprio;

	/*
	 * Update the pending order of the thread inside its wait
	 * queue, unless this behaviour has been explicitly disabled
	 * for the pended synchronization object, or the requested
	 * (weighted) priority has not changed, thus preventing
	 * spurious round-robin effects.
	 */
	if (old_wprio != new_wprio && thread->wchan != NULL &&
	    (thread->wchan->status & XNSYNCH_DREORD) == 0)
		xnsynch_requeue_sleeper(thread);
	/*
	 * We don't need/want to move the thread at the end of its
	 * priority group whenever:
	 * - it is blocked and thus not runnable;
	 * - it bears the ready bit in which case xnsched_set_policy()
	 * already reordered the runnable queue;
	 * - we currently hold the scheduler lock, so we don't want
	 * any round-robin effect to take place.
	 */
	if (!xnthread_test_state(thread, XNTHREAD_BLOCK_BITS|XNREADY) &&
	    thread->lock_count == 0)
		xnsched_putback(thread);

	return ret;
}

void __xnthread_test_cancel(struct xnthread *curr)
{
	/*
	 * Just in case xnthread_test_cancel() is called from an IRQ
	 * handler, in which case we may not take the exit path.
	 *
	 * NOTE: curr->sched is stable from our POV and can't change
	 * under our feet.
	 */
	if (curr->sched->lflags & XNINIRQ)
		return;

	if (!xnthread_test_state(curr, XNRELAX))
		xnthread_relax(0, 0);

	do_exit(0);
	/* ... won't return ... */
	XENO_BUG(COBALT);
}
EXPORT_SYMBOL_GPL(__xnthread_test_cancel);

/**
 * @internal
 * @fn int xnthread_harden(void);
 * @brief Migrate a Linux task to the Xenomai domain.
 *
 * This service causes the transition of "current" from the Linux
 * domain to Xenomai. The shadow will resume in the Xenomai domain as
 * returning from schedule().
 *
 * @coretags{secondary-only, might-switch}
 */
int xnthread_harden(void)
{
	struct task_struct *p = current;
	struct xnthread *thread;
	struct xnsched *sched;
	int ret;

	secondary_mode_only();

	thread = xnthread_current();
	if (thread == NULL)
		return -EPERM;

	if (signal_pending(p))
		return -ERESTARTSYS;

	trace_cobalt_shadow_gohard(thread);

	xnthread_clear_sync_window(thread, XNRELAX);

	ret = __ipipe_migrate_head();
	if (ret) {
		xnthread_set_sync_window(thread, XNRELAX);
		return ret;
	}

	/* "current" is now running into the Xenomai domain. */
	sched = xnsched_finish_unlocked_switch(thread->sched);
	xnthread_switch_fpu(sched);

	xnlock_clear_irqon(&nklock);
	xnsched_resched_after_unlocked_switch();
	xnthread_test_cancel();

	trace_cobalt_shadow_hardened(thread);

	/*
	 * Recheck pending signals once again. As we block task
	 * wakeups during the migration and handle_sigwake_event()
	 * ignores signals until XNRELAX is cleared, any signal
	 * between entering TASK_HARDENING and starting the migration
	 * is just silently queued up to here.
	 */
	if (signal_pending(p)) {
		xnthread_relax(!xnthread_test_state(thread, XNSSTEP),
			       SIGDEBUG_MIGRATE_SIGNAL);
		return -ERESTARTSYS;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(xnthread_harden);

struct lostage_wakeup {
	struct ipipe_work_header work; /* Must be first. */
	struct task_struct *task;
};

static void lostage_task_wakeup(struct ipipe_work_header *work)
{
	struct lostage_wakeup *rq;
	struct task_struct *p;

	rq = container_of(work, struct lostage_wakeup, work);
	p = rq->task;

	trace_cobalt_lostage_wakeup(p);

	wake_up_process(p);
}

static void post_wakeup(struct task_struct *p)
{
	struct lostage_wakeup wakework = {
		.work = {
			.size = sizeof(wakework),
			.handler = lostage_task_wakeup,
		},
		.task = p,
	};

	trace_cobalt_lostage_request("wakeup", wakework.task);

	ipipe_post_work_root(&wakework, work);
}

/**
 * @internal
 * @fn void xnthread_relax(int notify, int reason);
 * @brief Switch a shadow thread back to the Linux domain.
 *
 * This service yields the control of the running shadow back to
 * Linux. This is obtained by suspending the shadow and scheduling a
 * wake up call for the mated user task inside the Linux domain. The
 * Linux task will resume on return from xnthread_suspend() on behalf
 * of the root thread.
 *
 * @param notify A boolean flag indicating whether threads monitored
 * from secondary mode switches should be sent a SIGDEBUG signal. For
 * instance, some internal operations like task exit should not
 * trigger such signal.
 *
 * @param reason The reason to report along with the SIGDEBUG signal.
 *
 * @coretags{primary-only, might-switch}
 *
 * @note "current" is valid here since the shadow runs with the
 * properties of the Linux task.
 */
void xnthread_relax(int notify, int reason)
{
	struct xnthread *thread = xnthread_current();
	struct task_struct *p = current;
	int cpu __maybe_unused;
	siginfo_t si;

	primary_mode_only();

	/*
	 * Enqueue the request to move the running shadow from the Xenomai
	 * domain to the Linux domain.  This will cause the Linux task
	 * to resume using the register state of the shadow thread.
	 */
	trace_cobalt_shadow_gorelax(thread, reason);

	/*
	 * If you intend to change the following interrupt-free
	 * sequence, /first/ make sure to check the special handling
	 * of XNRELAX in xnthread_suspend() when switching out the
	 * current thread, not to break basic assumptions we make
	 * there.
	 *
	 * We disable interrupts during the migration sequence, but
	 * xnthread_suspend() has an interrupts-on section built in.
	 */
	splmax();
	post_wakeup(p);
	/*
	 * Grab the nklock to synchronize the Linux task state
	 * manipulation with handle_sigwake_event. This lock will be
	 * dropped by xnthread_suspend().
	 */
	xnlock_get(&nklock);
	set_current_state(p->state & ~TASK_NOWAKEUP);
	xnthread_run_handler_stack(thread, relax_thread);
	xnthread_suspend(thread, XNRELAX, XN_INFINITE, XN_RELATIVE, NULL);
	splnone();

	/*
	 * Basic sanity check after an expected transition to secondary
	 * mode.
	 */
	XENO_WARN(COBALT, !ipipe_root_p,
		  "xnthread_relax() failed for thread %s[%d]",
		  thread->name, xnthread_host_pid(thread));

	__ipipe_reenter_root();

	/* Account for secondary mode switch. */
	xnstat_counter_inc(&thread->stat.ssw);

	if (xnthread_test_state(thread, XNUSER) && notify) {
		xndebug_notify_relax(thread, reason);
		if (xnthread_test_state(thread, XNWARN)) {
			/* Help debugging spurious relaxes. */
			memset(&si, 0, sizeof(si));
			si.si_signo = SIGDEBUG;
			si.si_code = SI_QUEUE;
			si.si_int = reason | sigdebug_marker;
			send_sig_info(SIGDEBUG, &si, p);
		}
		xnsynch_detect_claimed_relax(thread);
	}

	/*
	 * "current" is now running into the Linux domain on behalf of
	 * the root thread.
	 */
	xnthread_sync_window(thread);

#ifdef CONFIG_SMP
	if (xnthread_test_localinfo(thread, XNMOVED)) {
		xnthread_clear_localinfo(thread, XNMOVED);
		cpu = xnsched_cpu(thread->sched);
		set_cpus_allowed_ptr(p, cpumask_of(cpu));
	}
#endif

	trace_cobalt_shadow_relaxed(thread);
}
EXPORT_SYMBOL_GPL(xnthread_relax);

struct lostage_signal {
	struct ipipe_work_header work; /* Must be first. */
	struct task_struct *task;
	int signo, sigval;
};

static inline void do_kthread_signal(struct task_struct *p,
				     struct xnthread *thread,
				     struct lostage_signal *rq)
{
	printk(XENO_WARNING
	       "kernel shadow %s received unhandled signal %d (action=0x%x)\n",
	       thread->name, rq->signo, rq->sigval);
}

static void lostage_task_signal(struct ipipe_work_header *work)
{
	struct lostage_signal *rq;
	struct xnthread *thread;
	struct task_struct *p;
	siginfo_t si;
	int signo;

	rq = container_of(work, struct lostage_signal, work);
	p = rq->task;

	thread = xnthread_from_task(p);
	if (thread && !xnthread_test_state(thread, XNUSER)) {
		do_kthread_signal(p, thread, rq);
		return;
	}

	signo = rq->signo;

	trace_cobalt_lostage_signal(p, signo);

	if (signo == SIGSHADOW || signo == SIGDEBUG) {
		memset(&si, '\0', sizeof(si));
		si.si_signo = signo;
		si.si_code = SI_QUEUE;
		si.si_int = rq->sigval;
		send_sig_info(signo, &si, p);
	} else
		send_sig(signo, p, 1);
}

static int force_wakeup(struct xnthread *thread) /* nklock locked, irqs off */
{
	int ret = 0;

	if (xnthread_test_info(thread, XNKICKED))
		return 1;

	if (xnthread_unblock(thread)) {
		xnthread_set_info(thread, XNKICKED);
		ret = 1;
	}

	/*
	 * CAUTION: we must NOT raise XNBREAK when clearing a forcible
	 * block state, such as XNSUSP, XNHELD. The caller of
	 * xnthread_suspend() we unblock shall proceed as for a normal
	 * return, until it traverses a cancellation point if
	 * XNCANCELD was raised earlier, or calls xnthread_suspend()
	 * which will detect XNKICKED and act accordingly.
	 *
	 * Rationale: callers of xnthread_suspend() may assume that
	 * receiving XNBREAK means that the process that motivated the
	 * blocking did not go to completion. E.g. the wait context
	 * (see. xnthread_prepare_wait()) was NOT posted before
	 * xnsynch_sleep_on() returned, leaving no useful data there.
	 * Therefore, in case only XNSUSP remains set for the thread
	 * on entry to force_wakeup(), after XNPEND was lifted earlier
	 * when the wait went to successful completion (i.e. no
	 * timeout), then we want the kicked thread to know that it
	 * did receive the requested resource, not finding XNBREAK in
	 * its state word.
	 *
	 * Callers of xnthread_suspend() may inquire for XNKICKED to
	 * detect forcible unblocks from XNSUSP, XNHELD, if they
	 * should act upon this case specifically.
	 */
	if (xnthread_test_state(thread, XNSUSP|XNHELD)) {
		xnthread_resume(thread, XNSUSP|XNHELD);
		xnthread_set_info(thread, XNKICKED);
	}

	/*
	 * Tricky cases:
	 *
	 * - a thread which was ready on entry wasn't actually
	 * running, but nevertheless waits for the CPU in primary
	 * mode, so we have to make sure that it will be notified of
	 * the pending break condition as soon as it enters
	 * xnthread_suspend() from a blocking Xenomai syscall.
	 *
	 * - a ready/readied thread on exit may be prevented from
	 * running by the scheduling policy module it belongs
	 * to. Typically, policies enforcing a runtime budget do not
	 * block threads with no budget, but rather keep them out of
	 * their runnable queue, so that ->sched_pick() won't elect
	 * them. We tell the policy handler about the fact that we do
	 * want such thread to run until it relaxes, whatever this
	 * means internally for the implementation.
	 */
	if (xnthread_test_state(thread, XNREADY))
		xnsched_kick(thread);

	return ret;
}

void __xnthread_kick(struct xnthread *thread) /* nklock locked, irqs off */
{
	struct task_struct *p = xnthread_host_task(thread);

	/* Thread is already relaxed -- nop. */
	if (xnthread_test_state(thread, XNRELAX))
		return;

	/*
	 * First, try to kick the thread out of any blocking syscall
	 * Xenomai-wise. If that succeeds, then the thread will relax
	 * on its return path to user-space.
	 */
	if (force_wakeup(thread))
		return;

	/*
	 * If that did not work out because the thread was not blocked
	 * (i.e. XNPEND/XNDELAY) in a syscall, then force a mayday
	 * trap. Note that we don't want to send that thread any linux
	 * signal, we only want to force it to switch to secondary
	 * mode asap.
	 *
	 * It could happen that a thread is relaxed on a syscall
	 * return path after it was resumed from self-suspension
	 * (e.g. XNSUSP) then also forced to run a mayday trap right
	 * after: this is still correct, at worst we would get a
	 * useless mayday syscall leading to a no-op, no big deal.
	 */
	xnthread_set_info(thread, XNKICKED);

	/*
	 * We may send mayday signals to userland threads only.
	 * However, no need to run a mayday trap if the current thread
	 * kicks itself out of primary mode: it will relax on its way
	 * back to userland via the current syscall
	 * epilogue. Otherwise, we want that thread to enter the
	 * mayday trap asap, to call us back for relaxing.
	 */
	if (thread != xnsched_current_thread() &&
	    xnthread_test_state(thread, XNUSER))
		ipipe_raise_mayday(p);
}

void xnthread_kick(struct xnthread *thread)
{
	spl_t s;

	xnlock_get_irqsave(&nklock, s);
	__xnthread_kick(thread);
	xnlock_put_irqrestore(&nklock, s);
}
EXPORT_SYMBOL_GPL(xnthread_kick);

void __xnthread_demote(struct xnthread *thread) /* nklock locked, irqs off */
{
	struct xnsched_class *sched_class;
	union xnsched_policy_param param;

	/*
	 * First we kick the thread out of primary mode, and have it
	 * resume execution immediately over the regular linux
	 * context.
	 */
	__xnthread_kick(thread);

	/*
	 * Then we demote it, turning that thread into a non real-time
	 * Xenomai shadow, which still has access to Xenomai
	 * resources, but won't compete for real-time scheduling
	 * anymore. In effect, moving the thread to a weak scheduling
	 * class/priority will prevent it from sticking back to
	 * primary mode.
	 */
#ifdef CONFIG_XENO_OPT_SCHED_WEAK
	param.weak.prio = 0;
	sched_class = &xnsched_class_weak;
#else
	param.rt.prio = 0;
	sched_class = &xnsched_class_rt;
#endif
	__xnthread_set_schedparam(thread, sched_class, &param);
}

void xnthread_demote(struct xnthread *thread)
{
	spl_t s;

	xnlock_get_irqsave(&nklock, s);
	__xnthread_demote(thread);
	xnlock_put_irqrestore(&nklock, s);
}
EXPORT_SYMBOL_GPL(xnthread_demote);

void xnthread_signal(struct xnthread *thread, int sig, int arg)
{
	struct lostage_signal sigwork = {
		.work = {
			.size = sizeof(sigwork),
			.handler = lostage_task_signal,
		},
		.task = xnthread_host_task(thread),
		.signo = sig,
		.sigval = sig == SIGDEBUG ? arg | sigdebug_marker : arg,
	};

	trace_cobalt_lostage_request("signal", sigwork.task);

	ipipe_post_work_root(&sigwork, work);
}
EXPORT_SYMBOL_GPL(xnthread_signal);

void xnthread_pin_initial(struct xnthread *thread)
{
	struct task_struct *p = current;
	struct xnsched *sched;
	int cpu;
	spl_t s;

	/*
	 * @thread is the Xenomai extension of the current kernel
	 * task. If the current CPU is part of the affinity mask of
	 * this thread, pin the latter on this CPU. Otherwise pin it
	 * to the first CPU of that mask.
	 */
	cpu = task_cpu(p);
	if (!cpumask_test_cpu(cpu, &thread->affinity))
		cpu = cpumask_first(&thread->affinity);

	set_cpus_allowed_ptr(p, cpumask_of(cpu));
	/*
	 * @thread is still unstarted Xenomai-wise, we are precisely
	 * in the process of mapping the current kernel task to
	 * it. Therefore xnthread_migrate_passive() is the right way
	 * to pin it on a real-time CPU.
	 */
	xnlock_get_irqsave(&nklock, s);
	sched = xnsched_struct(cpu);
	xnthread_migrate_passive(thread, sched);
	xnlock_put_irqrestore(&nklock, s);
}

struct parent_wakeup_request {
	struct ipipe_work_header work; /* Must be first. */
	struct completion *done;
};

static void do_parent_wakeup(struct ipipe_work_header *work)
{
	struct parent_wakeup_request *rq;

	rq = container_of(work, struct parent_wakeup_request, work);
	complete(rq->done);
}

static inline void wakeup_parent(struct completion *done)
{
	struct parent_wakeup_request wakework = {
		.work = {
			.size = sizeof(wakework),
			.handler = do_parent_wakeup,
		},
		.done = done,
	};

	trace_cobalt_lostage_request("wakeup", current);

	ipipe_post_work_root(&wakework, work);
}

static inline void init_kthread_info(struct xnthread *thread)
{
	struct ipipe_threadinfo *p;

	p = ipipe_current_threadinfo();
	p->thread = thread;
	p->process = NULL;
}

/**
 * @fn int xnthread_map(struct xnthread *thread, struct completion *done)
 * @internal
 * @brief Create a shadow thread context over a kernel task.
 *
 * This call maps a Cobalt core thread to the "current" Linux task
 * running in kernel space.  The priority and scheduling class of the
 * underlying Linux task are not affected; it is assumed that the
 * caller did set them appropriately before issuing the shadow mapping
 * request.
 *
 * This call immediately moves the calling kernel thread to the
 * Xenomai domain.
 *
 * @param thread The descriptor address of the new shadow thread to be
 * mapped to "current". This descriptor must have been previously
 * initialized by a call to xnthread_init().
 *
 * @param done A completion object to be signaled when @a thread is
 * fully mapped over the current Linux context, waiting for
 * xnthread_start().
 *
 * @return 0 is returned on success. Otherwise:
 *
 * - -ERESTARTSYS is returned if the current Linux task has received a
 * signal, thus preventing the final migration to the Xenomai domain
 * (i.e. in order to process the signal in the Linux domain). This
 * error should not be considered as fatal.
 *
 * - -EPERM is returned if the shadow thread has been killed before
 * the current task had a chance to return to the caller. In such a
 * case, the real-time mapping operation has failed globally, and no
 * Xenomai resource remains attached to it.
 *
 * - -EINVAL is returned if the thread control block bears the XNUSER
 * bit.
 *
 * - -EBUSY is returned if either the current Linux task or the
 * associated shadow thread is already involved in a shadow mapping.
 *
 * @coretags{secondary-only, might-switch}
 */
int xnthread_map(struct xnthread *thread, struct completion *done)
{
	struct task_struct *p = current;
	int ret;
	spl_t s;

	if (xnthread_test_state(thread, XNUSER))
		return -EINVAL;

	if (xnthread_current() || xnthread_test_state(thread, XNMAPPED))
		return -EBUSY;

	thread->u_window = NULL;
	xnthread_pin_initial(thread);

	trace_cobalt_shadow_map(thread);

	xnthread_init_shadow_tcb(thread);
	xnthread_suspend(thread, XNRELAX, XN_INFINITE, XN_RELATIVE, NULL);
	init_kthread_info(thread);
	xnthread_set_state(thread, XNMAPPED);
	xndebug_shadow_init(thread);
	xnthread_run_handler(thread, map_thread);
	ipipe_enable_notifier(p);

	/*
	 * CAUTION: Soon after xnthread_init() has returned,
	 * xnthread_start() is commonly invoked from the root domain,
	 * therefore the call site may expect the started kernel
	 * shadow to preempt immediately. As a result of such
	 * assumption, start attributes (struct xnthread_start_attr)
	 * are often laid on the caller's stack.
	 *
	 * For this reason, we raise the completion signal to wake up
	 * the xnthread_init() caller only once the emerging thread is
	 * hardened, and __never__ before that point. Since we run
	 * over the Xenomai domain upon return from xnthread_harden(),
	 * we schedule a virtual interrupt handler in the root domain
	 * to signal the completion object.
	 */
	xnthread_resume(thread, XNDORMANT);
	ret = xnthread_harden();
	wakeup_parent(done);

	xnlock_get_irqsave(&nklock, s);

	enlist_new_thread(thread);
	/*
	 * Make sure xnthread_start() did not slip in from another CPU
	 * while we were back from wakeup_parent().
	 */
	if (thread->entry == NULL)
		xnthread_suspend(thread, XNDORMANT,
				 XN_INFINITE, XN_RELATIVE, NULL);

	xnlock_put_irqrestore(&nklock, s);

	xnthread_test_cancel();

	xntrace_pid(xnthread_host_pid(thread),
		    xnthread_current_priority(thread));

	return ret;
}
EXPORT_SYMBOL_GPL(xnthread_map);

/* nklock locked, irqs off */
void xnthread_call_mayday(struct xnthread *thread, int reason)
{
	struct task_struct *p = xnthread_host_task(thread);

	/* Mayday traps are available to userland threads only. */
	XENO_BUG_ON(COBALT, !xnthread_test_state(thread, XNUSER));
	xnthread_set_info(thread, XNKICKED);
	xnthread_signal(thread, SIGDEBUG, reason);
	ipipe_raise_mayday(p);
}
EXPORT_SYMBOL_GPL(xnthread_call_mayday);

int xnthread_killall(int grace, int mask)
{
	struct xnthread *t, *curr = xnthread_current();
	int nrkilled = 0, nrthreads, count;
	long ret;
	spl_t s;

	secondary_mode_only();

	/*
	 * We may hold the core lock across calls to xnthread_cancel()
	 * provided that we won't self-cancel.
	 */
	xnlock_get_irqsave(&nklock, s);

	nrthreads = cobalt_nrthreads;
	
	xnsched_for_each_thread(t) {
		if (xnthread_test_state(t, XNROOT) ||
		    xnthread_test_state(t, mask) != mask ||
		    t == curr)
			continue;

		if (XENO_DEBUG(COBALT))
			printk(XENO_INFO "terminating %s[%d]\n",
			       t->name, xnthread_host_pid(t));
		nrkilled++;
		xnthread_cancel(t);
	}

	xnlock_put_irqrestore(&nklock, s);

	/*
	 * Cancel then join all existing threads during the grace
	 * period. It is the caller's responsibility to prevent more
	 * threads to bind to the system if required, we won't make
	 * any provision for this here.
	 */
	count = nrthreads - nrkilled;
	if (XENO_DEBUG(COBALT))
		printk(XENO_INFO "waiting for %d threads to exit\n",
		       nrkilled);

	if (grace > 0) {
		ret = wait_event_interruptible_timeout(join_all,
						       cobalt_nrthreads == count,
						       grace * HZ);
		if (ret == 0)
			return -EAGAIN;
	} else
		ret = wait_event_interruptible(join_all,
					       cobalt_nrthreads == count);

	/* Wait for a full RCU grace period to expire. */
	wait_for_rcu_grace_period(NULL);

	if (XENO_DEBUG(COBALT))
		printk(XENO_INFO "joined %d threads\n",
		       count + nrkilled - cobalt_nrthreads);

	return ret < 0 ? EINTR : 0;
}
EXPORT_SYMBOL_GPL(xnthread_killall);
		     
/* Xenomai's generic personality. */
struct xnthread_personality xenomai_personality = {
	.name = "core",
	.magic = -1
};
EXPORT_SYMBOL_GPL(xenomai_personality);

/** @} */
