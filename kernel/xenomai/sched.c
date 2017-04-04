/*
 * Copyright (C) 2001-2013 Philippe Gerum <rpm@xenomai.org>.
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
#include <linux/module.h>
#include <linux/signal.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <cobalt/kernel/sched.h>
#include <cobalt/kernel/thread.h>
#include <cobalt/kernel/timer.h>
#include <cobalt/kernel/intr.h>
#include <cobalt/kernel/heap.h>
#include <cobalt/kernel/arith.h>
#include <cobalt/uapi/signal.h>
#define CREATE_TRACE_POINTS
#include <trace/events/cobalt-core.h>

/**
 * @ingroup cobalt_core
 * @defgroup cobalt_core_sched Thread scheduling control
 * @{
 */

DEFINE_PER_CPU(struct xnsched, nksched);
EXPORT_PER_CPU_SYMBOL_GPL(nksched);

cpumask_t cobalt_cpu_affinity = CPU_MASK_ALL;
EXPORT_SYMBOL_GPL(cobalt_cpu_affinity);

LIST_HEAD(nkthreadq);

int cobalt_nrthreads;

#ifdef CONFIG_XENO_OPT_VFILE
struct xnvfile_rev_tag nkthreadlist_tag;
#endif

static struct xnsched_class *xnsched_class_highest;

#define for_each_xnsched_class(p) \
   for (p = xnsched_class_highest; p; p = p->next)

static void xnsched_register_class(struct xnsched_class *sched_class)
{
	sched_class->next = xnsched_class_highest;
	xnsched_class_highest = sched_class;

	/*
	 * Classes shall be registered by increasing priority order,
	 * idle first and up.
	 */
	XENO_BUG_ON(COBALT, sched_class->next &&
		   sched_class->next->weight > sched_class->weight);

	printk(XENO_INFO "scheduling class %s registered.\n", sched_class->name);
}

void xnsched_register_classes(void)
{
	xnsched_register_class(&xnsched_class_idle);
#ifdef CONFIG_XENO_OPT_SCHED_WEAK
	xnsched_register_class(&xnsched_class_weak);
#endif
#ifdef CONFIG_XENO_OPT_SCHED_TP
	xnsched_register_class(&xnsched_class_tp);
#endif
#ifdef CONFIG_XENO_OPT_SCHED_SPORADIC
	xnsched_register_class(&xnsched_class_sporadic);
#endif
#ifdef CONFIG_XENO_OPT_SCHED_QUOTA
	xnsched_register_class(&xnsched_class_quota);
#endif
	xnsched_register_class(&xnsched_class_rt);
}

#ifdef CONFIG_XENO_OPT_WATCHDOG

static unsigned long wd_timeout_arg = CONFIG_XENO_OPT_WATCHDOG_TIMEOUT;
module_param_named(watchdog_timeout, wd_timeout_arg, ulong, 0644);

/**
 * @internal
 * @fn void watchdog_handler(struct xntimer *timer)
 * @brief Process watchdog ticks.
 *
 * This internal routine handles incoming watchdog ticks to detect
 * software lockups. It kills any offending thread which is found to
 * monopolize the CPU so as to starve the Linux kernel for too long.
 *
 * @coretags{coreirq-only, atomic-entry}
 */
static void watchdog_handler(struct xntimer *timer)
{
	struct xnsched *sched = xnsched_current();
	struct xnthread *curr = sched->curr;

	if (likely(xnthread_test_state(curr, XNROOT))) {
		xnsched_reset_watchdog(sched);
		return;
	}

	if (likely(++sched->wdcount < wd_timeout_arg))
		return;

	trace_cobalt_watchdog_signal(curr);

	if (xnthread_test_state(curr, XNUSER)) {
		printk(XENO_WARNING "watchdog triggered on CPU #%d -- runaway thread "
		       "'%s' signaled\n", xnsched_cpu(sched), curr->name);
		xnthread_call_mayday(curr, SIGDEBUG_WATCHDOG);
	} else {
		printk(XENO_WARNING "watchdog triggered on CPU #%d -- runaway thread "
		       "'%s' canceled\n", xnsched_cpu(sched), curr->name);
		/*
		 * On behalf on an IRQ handler, xnthread_cancel()
		 * would go half way cancelling the preempted
		 * thread. Therefore we manually raise XNKICKED to
		 * cause the next call to xnthread_suspend() to return
		 * early in XNBREAK condition, and XNCANCELD so that
		 * @thread exits next time it invokes
		 * xnthread_test_cancel().
		 */
		xnthread_set_info(curr, XNKICKED|XNCANCELD);
	}

	xnsched_reset_watchdog(sched);
}

#endif /* CONFIG_XENO_OPT_WATCHDOG */

static void roundrobin_handler(struct xntimer *timer)
{
	struct xnsched *sched = container_of(timer, struct xnsched, rrbtimer);
	xnsched_tick(sched);
}

void xnsched_init(struct xnsched *sched, int cpu)
{
	char rrbtimer_name[XNOBJECT_NAME_LEN];
	char htimer_name[XNOBJECT_NAME_LEN];
	char root_name[XNOBJECT_NAME_LEN];
	union xnsched_policy_param param;
	struct xnthread_init_attr attr;
	struct xnsched_class *p;

#ifdef CONFIG_SMP
	sched->cpu = cpu;
	ksformat(htimer_name, sizeof(htimer_name), "[host-timer/%u]", cpu);
	ksformat(rrbtimer_name, sizeof(rrbtimer_name), "[rrb-timer/%u]", cpu);
	ksformat(root_name, sizeof(root_name), "ROOT/%u", cpu);
	cpumask_clear(&sched->resched);
#else
	strcpy(htimer_name, "[host-timer]");
	strcpy(rrbtimer_name, "[rrb-timer]");
	strcpy(root_name, "ROOT");
#endif
	for_each_xnsched_class(p) {
		if (p->sched_init)
			p->sched_init(sched);
	}

	sched->status = 0;
	sched->lflags = 0;
	sched->inesting = 0;
	sched->curr = &sched->rootcb;

	attr.flags = XNROOT | XNFPU;
	attr.name = root_name;
	attr.personality = &xenomai_personality;
	attr.affinity = *cpumask_of(cpu);
	param.idle.prio = XNSCHED_IDLE_PRIO;

	__xnthread_init(&sched->rootcb, &attr,
			sched, &xnsched_class_idle, &param);

	/*
	 * No direct handler here since the host timer processing is
	 * postponed to xnintr_irq_handler(), as part of the interrupt
	 * exit code.
	 */
	xntimer_init(&sched->htimer, &nkclock, NULL,
		     sched, XNTIMER_IGRAVITY);
	xntimer_set_priority(&sched->htimer, XNTIMER_LOPRIO);
	xntimer_set_name(&sched->htimer, htimer_name);
	xntimer_init(&sched->rrbtimer, &nkclock, roundrobin_handler,
		     sched, XNTIMER_IGRAVITY);
	xntimer_set_name(&sched->rrbtimer, rrbtimer_name);
	xntimer_set_priority(&sched->rrbtimer, XNTIMER_LOPRIO);

	xnstat_exectime_set_current(sched, &sched->rootcb.stat.account);
#ifdef CONFIG_XENO_ARCH_FPU
	sched->fpuholder = &sched->rootcb;
#endif /* CONFIG_XENO_ARCH_FPU */

	xnthread_init_root_tcb(&sched->rootcb);
	list_add_tail(&sched->rootcb.glink, &nkthreadq);
	cobalt_nrthreads++;

#ifdef CONFIG_XENO_OPT_WATCHDOG
	xntimer_init(&sched->wdtimer, &nkclock, watchdog_handler,
		     sched, XNTIMER_NOBLCK|XNTIMER_IGRAVITY);
	xntimer_set_name(&sched->wdtimer, "[watchdog]");
	xntimer_set_priority(&sched->wdtimer, XNTIMER_LOPRIO);
#endif /* CONFIG_XENO_OPT_WATCHDOG */
}

void xnsched_destroy(struct xnsched *sched)
{
	xntimer_destroy(&sched->htimer);
	xntimer_destroy(&sched->rrbtimer);
	xntimer_destroy(&sched->rootcb.ptimer);
	xntimer_destroy(&sched->rootcb.rtimer);
#ifdef CONFIG_XENO_OPT_WATCHDOG
	xntimer_destroy(&sched->wdtimer);
#endif /* CONFIG_XENO_OPT_WATCHDOG */
}

static inline void set_thread_running(struct xnsched *sched,
				      struct xnthread *thread)
{
	xnthread_clear_state(thread, XNREADY);
	if (xnthread_test_state(thread, XNRRB))
		xntimer_start(&sched->rrbtimer,
			      thread->rrperiod, XN_INFINITE, XN_RELATIVE);
	else
		xntimer_stop(&sched->rrbtimer);
}

/* Must be called with nklock locked, interrupts off. */
struct xnthread *xnsched_pick_next(struct xnsched *sched)
{
	struct xnsched_class *p __maybe_unused;
	struct xnthread *curr = sched->curr;
	struct xnthread *thread;

	if (!xnthread_test_state(curr, XNTHREAD_BLOCK_BITS | XNZOMBIE)) {
		/*
		 * Do not preempt the current thread if it holds the
		 * scheduler lock.
		 */
		if (curr->lock_count > 0) {
			xnsched_set_self_resched(sched);
			return curr;
		}
		/*
		 * Push the current thread back to the runnable queue
		 * of the scheduling class it belongs to, if not yet
		 * linked to it (XNREADY tells us if it is).
		 */
		if (!xnthread_test_state(curr, XNREADY)) {
			xnsched_requeue(curr);
			xnthread_set_state(curr, XNREADY);
		}
	}

	/*
	 * Find the runnable thread having the highest priority among
	 * all scheduling classes, scanned by decreasing priority.
	 */
#ifdef CONFIG_XENO_OPT_SCHED_CLASSES
	for_each_xnsched_class(p) {
		thread = p->sched_pick(sched);
		if (thread) {
			set_thread_running(sched, thread);
			return thread;
		}
	}

	return NULL; /* Never executed because of the idle class. */
#else /* !CONFIG_XENO_OPT_SCHED_CLASSES */
	thread = xnsched_rt_pick(sched);
	if (unlikely(thread == NULL))
		thread = &sched->rootcb;

	set_thread_running(sched, thread);

	return thread;
#endif /* CONFIG_XENO_OPT_SCHED_CLASSES */
}

#ifdef CONFIG_XENO_ARCH_UNLOCKED_SWITCH

struct xnsched *xnsched_finish_unlocked_switch(struct xnsched *sched)
{
	struct xnthread *last;
	spl_t s;

	xnlock_get_irqsave(&nklock, s);

#ifdef CONFIG_SMP
	/* If current thread migrated while suspended */
	sched = xnsched_current();
#endif /* CONFIG_SMP */

	last = sched->last;
	sched->status &= ~XNINSW;

	/* Detect a thread which called xnthread_migrate() */
	if (last->sched != sched) {
		xnsched_putback(last);
		xnthread_clear_state(last, XNMIGRATE);
	}

	return sched;
}

#endif /* CONFIG_XENO_ARCH_UNLOCKED_SWITCH */

void xnsched_lock(void)
{
	struct xnsched *sched = xnsched_current();
	struct xnthread *curr = sched->curr;

	/*
	 * CAUTION: The fast xnthread_current() accessor carries the
	 * relevant lock nesting count only if current runs in primary
	 * mode. Otherwise, if the caller is unknown or relaxed
	 * Xenomai-wise, then we fall back to the root thread on the
	 * current scheduler, which must be done with IRQs off.
	 * Either way, we don't need to grab the super lock.
	 */
	if (unlikely(curr == NULL || xnthread_test_state(curr, XNRELAX))) {
		/*
		 * In IRQ: scheduler already locked, and we may have
		 * interrupted xnthread_relax() where the BUG_ON condition is
		 * temporarily false.
		 */
		if (sched->lflags & XNINIRQ)
			return;

		irqoff_only();
		curr = &sched->rootcb;
		XENO_BUG_ON(COBALT, xnsched_current()->curr != curr);
	}

	curr->lock_count++;
}
EXPORT_SYMBOL_GPL(xnsched_lock);

void xnsched_unlock(void)
{
	struct xnsched *sched = xnsched_current();
	struct xnthread *curr = sched->curr;

	if (unlikely(curr == NULL || xnthread_test_state(curr, XNRELAX))) {
		/*
		 * In IRQ
		 */
		if (sched->lflags & XNINIRQ)
			return;

		irqoff_only();
		curr = &xnsched_current()->rootcb;
	}

	if (!XENO_ASSERT(COBALT, curr->lock_count > 0))
		return;
	
	if (--curr->lock_count == 0) {
		xnthread_clear_localinfo(curr, XNLBALERT);
		xnsched_run();
	}
}
EXPORT_SYMBOL_GPL(xnsched_unlock);

/* Must be called with nklock locked, interrupts off. */
void xnsched_putback(struct xnthread *thread)
{
	if (xnthread_test_state(thread, XNREADY))
		xnsched_dequeue(thread);
	else
		xnthread_set_state(thread, XNREADY);

	xnsched_enqueue(thread);
	xnsched_set_resched(thread->sched);
}

/* Must be called with nklock locked, interrupts off. */
int xnsched_set_policy(struct xnthread *thread,
		       struct xnsched_class *sched_class,
		       const union xnsched_policy_param *p)
{
	int ret;

	/*
	 * Declaring a thread to a new scheduling class may fail, so
	 * we do that early, while the thread is still a member of the
	 * previous class. However, this also means that the
	 * declaration callback shall not do anything that might
	 * affect the previous class (such as touching thread->rlink
	 * for instance).
	 */
	if (sched_class != thread->base_class) {
		ret = xnsched_declare(sched_class, thread, p);
		if (ret)
			return ret;
	}

	/*
	 * As a special case, we may be called from __xnthread_init()
	 * with no previous scheduling class at all.
	 */
	if (likely(thread->base_class != NULL)) {
		if (xnthread_test_state(thread, XNREADY))
			xnsched_dequeue(thread);

		if (sched_class != thread->base_class)
			xnsched_forget(thread);
	}

	thread->sched_class = sched_class;
	thread->base_class = sched_class;
	xnsched_setparam(thread, p);
	thread->bprio = thread->cprio;
	thread->wprio = thread->cprio + sched_class->weight;

	if (xnthread_test_state(thread, XNREADY))
		xnsched_enqueue(thread);

	if (!xnthread_test_state(thread, XNDORMANT))
		xnsched_set_resched(thread->sched);

	return 0;
}
EXPORT_SYMBOL_GPL(xnsched_set_policy);

/* Must be called with nklock locked, interrupts off. */
void xnsched_track_policy(struct xnthread *thread,
			  struct xnthread *target)
{
	union xnsched_policy_param param;

	if (xnthread_test_state(thread, XNREADY))
		xnsched_dequeue(thread);
	/*
	 * Self-targeting means to reset the scheduling policy and
	 * parameters to the base ones. Otherwise, make thread inherit
	 * the scheduling data from target.
	 */
	if (target == thread) {
		thread->sched_class = thread->base_class;
		xnsched_trackprio(thread, NULL);
	} else {
		xnsched_getparam(target, &param);
		thread->sched_class = target->sched_class;
		xnsched_trackprio(thread, &param);
	}

	if (xnthread_test_state(thread, XNREADY))
		xnsched_enqueue(thread);

	xnsched_set_resched(thread->sched);
}

static void migrate_thread(struct xnthread *thread, struct xnsched *sched)
{
	struct xnsched_class *sched_class = thread->sched_class;

	if (xnthread_test_state(thread, XNREADY)) {
		xnsched_dequeue(thread);
		xnthread_clear_state(thread, XNREADY);
	}

	if (sched_class->sched_migrate)
		sched_class->sched_migrate(thread, sched);
	/*
	 * WARNING: the scheduling class may have just changed as a
	 * result of calling the per-class migration hook.
	 */
	thread->sched = sched;
}

/*
 * Must be called with nklock locked, interrupts off. thread must be
 * runnable.
 */
void xnsched_migrate(struct xnthread *thread, struct xnsched *sched)
{
	xnsched_set_resched(thread->sched);
	migrate_thread(thread, sched);

#ifdef CONFIG_XENO_ARCH_UNLOCKED_SWITCH
	/*
	 * Mark the thread in flight, xnsched_finish_unlocked_switch()
	 * will put the thread on the remote runqueue.
	 */
	xnthread_set_state(thread, XNMIGRATE);
#else /* !CONFIG_XENO_ARCH_UNLOCKED_SWITCH */
	/* Move thread to the remote runnable queue. */
	xnsched_putback(thread);
#endif /* !CONFIG_XENO_ARCH_UNLOCKED_SWITCH */
}

/*
 * Must be called with nklock locked, interrupts off. Thread may be
 * blocked.
 */
void xnsched_migrate_passive(struct xnthread *thread, struct xnsched *sched)
{
	struct xnsched *last_sched = thread->sched;

	migrate_thread(thread, sched);

	if (!xnthread_test_state(thread, XNTHREAD_BLOCK_BITS)) {
		xnsched_requeue(thread);
		xnthread_set_state(thread, XNREADY);
		xnsched_set_resched(last_sched);
	}
}

#ifdef CONFIG_XENO_OPT_SCALABLE_SCHED

void xnsched_initq(struct xnsched_mlq *q)
{
	int prio;

	q->elems = 0;
	bitmap_zero(q->prio_map, XNSCHED_MLQ_LEVELS);

	for (prio = 0; prio < XNSCHED_MLQ_LEVELS; prio++)
		INIT_LIST_HEAD(q->heads + prio);
}

static inline int get_qindex(struct xnsched_mlq *q, int prio)
{
	XENO_BUG_ON(COBALT, prio < 0 || prio >= XNSCHED_MLQ_LEVELS);
	/*
	 * BIG FAT WARNING: We need to rescale the priority level to a
	 * 0-based range. We use find_first_bit() to scan the bitmap
	 * which is a bit scan forward operation. Therefore, the lower
	 * the index value, the higher the priority (since least
	 * significant bits will be found first when scanning the
	 * bitmap).
	 */
	return XNSCHED_MLQ_LEVELS - prio - 1;
}

static struct list_head *add_q(struct xnsched_mlq *q, int prio)
{
	struct list_head *head;
	int idx;

	idx = get_qindex(q, prio);
	head = q->heads + idx;
	q->elems++;

	/* New item is not linked yet. */
	if (list_empty(head))
		__set_bit(idx, q->prio_map);

	return head;
}

void xnsched_addq(struct xnsched_mlq *q, struct xnthread *thread)
{
	struct list_head *head = add_q(q, thread->cprio);
	list_add(&thread->rlink, head);
}

void xnsched_addq_tail(struct xnsched_mlq *q, struct xnthread *thread)
{
	struct list_head *head = add_q(q, thread->cprio);
	list_add_tail(&thread->rlink, head);
}

static void del_q(struct xnsched_mlq *q,
		  struct list_head *entry, int idx)
{
	struct list_head *head = q->heads + idx;

	list_del(entry);
	q->elems--;

	if (list_empty(head))
		__clear_bit(idx, q->prio_map);
}

void xnsched_delq(struct xnsched_mlq *q, struct xnthread *thread)
{
	del_q(q, &thread->rlink, get_qindex(q, thread->cprio));
}

struct xnthread *xnsched_getq(struct xnsched_mlq *q)
{
	struct xnthread *thread;
	struct list_head *head;
	int idx;

	if (q->elems == 0)
		return NULL;

	idx = xnsched_weightq(q);
	head = q->heads + idx;
	XENO_BUG_ON(COBALT, list_empty(head));
	thread = list_first_entry(head, struct xnthread, rlink);
	del_q(q, &thread->rlink, idx);

	return thread;
}

struct xnthread *xnsched_findq(struct xnsched_mlq *q, int prio)
{
	struct list_head *head;
	int idx;

	idx = get_qindex(q, prio);
	head = q->heads + idx;
	if (list_empty(head))
		return NULL;

	return list_first_entry(head, struct xnthread, rlink);
}

#ifdef CONFIG_XENO_OPT_SCHED_CLASSES

struct xnthread *xnsched_rt_pick(struct xnsched *sched)
{
	struct xnsched_mlq *q = &sched->rt.runnable;
	struct xnthread *thread;
	struct list_head *head;
	int idx;

	if (q->elems == 0)
		return NULL;

	/*
	 * Some scheduling policies may be implemented as variants of
	 * the core SCHED_FIFO class, sharing its runqueue
	 * (e.g. SCHED_SPORADIC, SCHED_QUOTA). This means that we have
	 * to do some cascading to call the right pick handler
	 * eventually.
	 */
	idx = xnsched_weightq(q);
	head = q->heads + idx;
	XENO_BUG_ON(COBALT, list_empty(head));

	/*
	 * The active class (i.e. ->sched_class) is the one currently
	 * queuing the thread, reflecting any priority boost due to
	 * PIP.
	 */
	thread = list_first_entry(head, struct xnthread, rlink);
	if (unlikely(thread->sched_class != &xnsched_class_rt))
		return thread->sched_class->sched_pick(sched);

	del_q(q, &thread->rlink, idx);

	return thread;
}

#endif /* CONFIG_XENO_OPT_SCHED_CLASSES */

#else /* !CONFIG_XENO_OPT_SCALABLE_SCHED */

struct xnthread *xnsched_findq(struct list_head *q, int prio)
{
	struct xnthread *thread;

	if (list_empty(q))
		return NULL;

	/* Find thread leading a priority group. */
	list_for_each_entry(thread, q, rlink) {
		if (prio == thread->cprio)
			return thread;
	}

	return NULL;
}

#ifdef CONFIG_XENO_OPT_SCHED_CLASSES

struct xnthread *xnsched_rt_pick(struct xnsched *sched)
{
	struct list_head *q = &sched->rt.runnable;
	struct xnthread *thread;

	if (list_empty(q))
		return NULL;

	thread = list_first_entry(q, struct xnthread, rlink);
	if (unlikely(thread->sched_class != &xnsched_class_rt))
		return thread->sched_class->sched_pick(sched);

	list_del(&thread->rlink);

	return thread;
}

#endif /* CONFIG_XENO_OPT_SCHED_CLASSES */

#endif /* !CONFIG_XENO_OPT_SCALABLE_SCHED */

static inline void switch_context(struct xnsched *sched,
				  struct xnthread *prev, struct xnthread *next)
{
#ifdef CONFIG_XENO_ARCH_UNLOCKED_SWITCH
	sched->last = prev;
	sched->status |= XNINSW;
	xnlock_clear_irqon(&nklock);
#endif /* !CONFIG_XENO_ARCH_UNLOCKED_SWITCH */

	xnarch_switch_to(prev, next);
}

/**
 * @fn int xnsched_run(void)
 * @brief The rescheduling procedure.
 *
 * This is the central rescheduling routine which should be called to
 * validate and apply changes which have previously been made to the
 * nucleus scheduling state, such as suspending, resuming or changing
 * the priority of threads.  This call performs context switches as
 * needed. xnsched_run() schedules out the current thread if:
 *
 * - the current thread is about to block.
 * - a runnable thread from a higher priority scheduling class is
 * waiting for the CPU.
 * - the current thread does not lead the runnable threads from its
 * own scheduling class (i.e. round-robin).
 *
 * The Cobalt core implements a lazy rescheduling scheme so that most
 * of the services affecting the threads state MUST be followed by a
 * call to the rescheduling procedure for the new scheduling state to
 * be applied.
 *
 * In other words, multiple changes on the scheduler state can be done
 * in a row, waking threads up, blocking others, without being
 * immediately translated into the corresponding context switches.
 * When all changes have been applied, xnsched_run() should be called
 * for considering those changes, and possibly switching context.
 *
 * As a notable exception to the previous principle however, every
 * action which ends up suspending the current thread begets an
 * implicit call to the rescheduling procedure on behalf of the
 * blocking service.
 *
 * Typically, self-suspension or sleeping on a synchronization object
 * automatically leads to a call to the rescheduling procedure,
 * therefore the caller does not need to explicitly issue
 * xnsched_run() after such operations.
 *
 * The rescheduling procedure always leads to a null-effect if it is
 * called on behalf of an interrupt service routine. Any outstanding
 * scheduler lock held by the outgoing thread will be restored when
 * the thread is scheduled back in.
 *
 * Calling this procedure with no applicable context switch pending is
 * harmless and simply leads to a null-effect.
 *
 * @return Non-zero is returned if a context switch actually happened,
 * otherwise zero if the current thread was left running.
 *
 * @coretags{unrestricted}
 */
static inline int test_resched(struct xnsched *sched)
{
	int resched = xnsched_resched_p(sched);
#ifdef CONFIG_SMP
	/* Send resched IPI to remote CPU(s). */
	if (unlikely(!cpumask_empty(&sched->resched))) {
		smp_mb();
		ipipe_send_ipi(IPIPE_RESCHEDULE_IPI, sched->resched);
		cpumask_clear(&sched->resched);
	}
#endif
	sched->status &= ~XNRESCHED;

	return resched;
}

static inline void enter_root(struct xnthread *root)
{
	struct xnarchtcb *rootcb __maybe_unused = xnthread_archtcb(root);

#ifdef CONFIG_XENO_ARCH_UNLOCKED_SWITCH
	if (rootcb->core.mm == NULL)
		set_ti_thread_flag(rootcb->core.tip, TIF_MMSWITCH_INT);
#endif
	ipipe_unmute_pic();
}

static inline void leave_root(struct xnthread *root)
{
	struct xnarchtcb *rootcb = xnthread_archtcb(root);
	struct task_struct *p = current;

	ipipe_notify_root_preemption();
	ipipe_mute_pic();
	/* Remember the preempted Linux task pointer. */
	rootcb->core.host_task = p;
	rootcb->core.tsp = &p->thread;
	rootcb->core.mm = rootcb->core.active_mm = ipipe_get_active_mm();
#ifdef CONFIG_XENO_ARCH_WANT_TIP
	rootcb->core.tip = task_thread_info(p);
#endif
	xnarch_leave_root(root);
}

void __xnsched_run_handler(void) /* hw interrupts off. */
{
	trace_cobalt_schedule_remote(xnsched_current());
	xnsched_run();
}

int ___xnsched_run(struct xnsched *sched)
{
	struct xnthread *prev, *next, *curr;
	int switched, shadow;
	spl_t s;

	if (xnarch_escalate())
		return 0;

	trace_cobalt_schedule(sched);

	xnlock_get_irqsave(&nklock, s);

	curr = sched->curr;
	/*
	 * CAUTION: xnthread_host_task(curr) may be unsynced and even
	 * stale if curr = &rootcb, since the task logged by
	 * leave_root() may not still be the current one. Use
	 * "current" for disambiguating.
	 */
	xntrace_pid(task_pid_nr(current), xnthread_current_priority(curr));
reschedule:
	switched = 0;
	if (!test_resched(sched))
		goto out;

	next = xnsched_pick_next(sched);
	if (next == curr) {
		if (unlikely(xnthread_test_state(next, XNROOT))) {
			if (sched->lflags & XNHTICK)
				xnintr_host_tick(sched);
			if (sched->lflags & XNHDEFER)
				xnclock_program_shot(&nkclock, sched);
		}
		goto out;
	}

	prev = curr;

	trace_cobalt_switch_context(prev, next);

	if (xnthread_test_state(next, XNROOT))
		xnsched_reset_watchdog(sched);

	sched->curr = next;
	shadow = 1;

	if (xnthread_test_state(prev, XNROOT)) {
		leave_root(prev);
		shadow = 0;
	} else if (xnthread_test_state(next, XNROOT)) {
		if (sched->lflags & XNHTICK)
			xnintr_host_tick(sched);
		if (sched->lflags & XNHDEFER)
			xnclock_program_shot(&nkclock, sched);
		enter_root(next);
	}

	xnstat_exectime_switch(sched, &next->stat.account);
	xnstat_counter_inc(&next->stat.csw);

	switch_context(sched, prev, next);

	/*
	 * Test whether we transitioned from primary mode to secondary
	 * over a shadow thread, caused by a call to xnthread_relax().
	 * In such a case, we are running over the regular schedule()
	 * tail code, so we have to skip our tail code.
	 */
	if (shadow && ipipe_root_p)
		goto shadow_epilogue;

	switched = 1;
	sched = xnsched_finish_unlocked_switch(sched);
	/*
	 * Re-read the currently running thread, this is needed
	 * because of relaxed/hardened transitions.
	 */
	curr = sched->curr;
	xnthread_switch_fpu(sched);
	xntrace_pid(task_pid_nr(current), xnthread_current_priority(curr));
out:
	if (switched &&
	    xnsched_maybe_resched_after_unlocked_switch(sched))
		goto reschedule;

	xnlock_put_irqrestore(&nklock, s);

	return switched;

shadow_epilogue:
	__ipipe_complete_domain_migration();

	XENO_BUG_ON(COBALT, xnthread_current() == NULL);

	/*
	 * Interrupts must be disabled here (has to be done on entry
	 * of the Linux [__]switch_to function), but it is what
	 * callers expect, specifically the reschedule of an IRQ
	 * handler that hit before we call xnsched_run in
	 * xnthread_suspend() when relaxing a thread.
	 */
	XENO_BUG_ON(COBALT, !hard_irqs_disabled());

	return 1;
}
EXPORT_SYMBOL_GPL(___xnsched_run);

#ifdef CONFIG_XENO_OPT_VFILE

static struct xnvfile_directory sched_vfroot;

struct vfile_schedlist_priv {
	struct xnthread *curr;
	xnticks_t start_time;
};

struct vfile_schedlist_data {
	int cpu;
	pid_t pid;
	char name[XNOBJECT_NAME_LEN];
	char sched_class[XNOBJECT_NAME_LEN];
	char personality[XNOBJECT_NAME_LEN];
	int cprio;
	xnticks_t timeout;
	int state;
};

static struct xnvfile_snapshot_ops vfile_schedlist_ops;

static struct xnvfile_snapshot schedlist_vfile = {
	.privsz = sizeof(struct vfile_schedlist_priv),
	.datasz = sizeof(struct vfile_schedlist_data),
	.tag = &nkthreadlist_tag,
	.ops = &vfile_schedlist_ops,
};

static int vfile_schedlist_rewind(struct xnvfile_snapshot_iterator *it)
{
	struct vfile_schedlist_priv *priv = xnvfile_iterator_priv(it);

	/* &nkthreadq cannot be empty (root thread(s)). */
	priv->curr = list_first_entry(&nkthreadq, struct xnthread, glink);
	priv->start_time = xnclock_read_monotonic(&nkclock);

	return cobalt_nrthreads;
}

static int vfile_schedlist_next(struct xnvfile_snapshot_iterator *it,
				void *data)
{
	struct vfile_schedlist_priv *priv = xnvfile_iterator_priv(it);
	struct vfile_schedlist_data *p = data;
	xnticks_t timeout, period;
	struct xnthread *thread;
	xnticks_t base_time;

	if (priv->curr == NULL)
		return 0;	/* All done. */

	thread = priv->curr;
	if (list_is_last(&thread->glink, &nkthreadq))
		priv->curr = NULL;
	else
		priv->curr = list_next_entry(thread, glink);

	p->cpu = xnsched_cpu(thread->sched);
	p->pid = xnthread_host_pid(thread);
	memcpy(p->name, thread->name, sizeof(p->name));
	p->cprio = thread->cprio;
	p->state = xnthread_get_state(thread);
	if (thread->lock_count > 0)
		p->state |= XNLOCK;
	knamecpy(p->sched_class, thread->sched_class->name);
	knamecpy(p->personality, thread->personality->name);
	period = xnthread_get_period(thread);
	base_time = priv->start_time;
	if (xntimer_clock(&thread->ptimer) != &nkclock)
		base_time = xnclock_read_monotonic(xntimer_clock(&thread->ptimer));
	timeout = xnthread_get_timeout(thread, base_time);
	/*
	 * Here we cheat: thread is periodic and the sampling rate may
	 * be high, so it is indeed possible that the next tick date
	 * from the ptimer progresses fast enough while we are busy
	 * collecting output data in this loop, so that next_date -
	 * start_time > period. In such a case, we simply ceil the
	 * value to period to keep the result meaningful, even if not
	 * necessarily accurate. But what does accuracy mean when the
	 * sampling frequency is high, and the way to read it has to
	 * go through the vfile interface anyway?
	 */
	if (period > 0 && period < timeout &&
	    !xntimer_running_p(&thread->rtimer))
		timeout = period;

	p->timeout = timeout;

	return 1;
}

static int vfile_schedlist_show(struct xnvfile_snapshot_iterator *it,
				void *data)
{
	struct vfile_schedlist_data *p = data;
	char sbuf[64], pbuf[16], tbuf[16];

	if (p == NULL)
		xnvfile_printf(it,
			       "%-3s  %-6s %-5s  %-8s  %-5s %-12s  %-10s %s\n",
			       "CPU", "PID", "CLASS", "TYPE", "PRI", "TIMEOUT",
			       "STAT", "NAME");
	else {
		ksformat(pbuf, sizeof(pbuf), "%3d", p->cprio);
		xntimer_format_time(p->timeout, tbuf, sizeof(tbuf));
		xnthread_format_status(p->state, sbuf, sizeof(sbuf));

		xnvfile_printf(it,
			       "%3u  %-6d %-5s  %-8s  %-5s %-12s  %-10s %s%s%s\n",
			       p->cpu,
			       p->pid,
			       p->sched_class,
			       p->personality,
			       pbuf,
			       tbuf,
			       sbuf,
			       (p->state & XNUSER) ? "" : "[",
			       p->name,
			       (p->state & XNUSER) ? "" : "]");
	}

	return 0;
}

static struct xnvfile_snapshot_ops vfile_schedlist_ops = {
	.rewind = vfile_schedlist_rewind,
	.next = vfile_schedlist_next,
	.show = vfile_schedlist_show,
};

#ifdef CONFIG_XENO_OPT_STATS

static spl_t vfile_schedstat_lock_s;

static int vfile_schedstat_get_lock(struct xnvfile *vfile)
{
	int ret;

	ret = xnintr_get_query_lock();
	if (ret < 0)
		return ret;
	xnlock_get_irqsave(&nklock, vfile_schedstat_lock_s);
	return 0;
}

static void vfile_schedstat_put_lock(struct xnvfile *vfile)
{
	xnlock_put_irqrestore(&nklock, vfile_schedstat_lock_s);
	xnintr_put_query_lock();
}

static struct xnvfile_lock_ops vfile_schedstat_lockops = {
	.get = vfile_schedstat_get_lock,
	.put = vfile_schedstat_put_lock,
};

struct vfile_schedstat_priv {
	int irq;
	struct xnthread *curr;
	struct xnintr_iterator intr_it;
};

struct vfile_schedstat_data {
	int cpu;
	pid_t pid;
	int state;
	char name[XNOBJECT_NAME_LEN];
	unsigned long ssw;
	unsigned long csw;
	unsigned long xsc;
	unsigned long pf;
	xnticks_t exectime_period;
	xnticks_t account_period;
	xnticks_t exectime_total;
	struct xnsched_class *sched_class;
	xnticks_t period;
	int cprio;
};

static struct xnvfile_snapshot_ops vfile_schedstat_ops;

static struct xnvfile_snapshot schedstat_vfile = {
	.privsz = sizeof(struct vfile_schedstat_priv),
	.datasz = sizeof(struct vfile_schedstat_data),
	.tag = &nkthreadlist_tag,
	.ops = &vfile_schedstat_ops,
	.entry = { .lockops = &vfile_schedstat_lockops },
};

static int vfile_schedstat_rewind(struct xnvfile_snapshot_iterator *it)
{
	struct vfile_schedstat_priv *priv = xnvfile_iterator_priv(it);
	int irqnr;

	/*
	 * The activity numbers on each valid interrupt descriptor are
	 * grouped under a pseudo-thread.
	 */
	priv->curr = list_first_entry(&nkthreadq, struct xnthread, glink);
	priv->irq = 0;
	irqnr = xnintr_query_init(&priv->intr_it) * NR_CPUS;

	return irqnr + cobalt_nrthreads;
}

static int vfile_schedstat_next(struct xnvfile_snapshot_iterator *it,
				void *data)
{
	struct vfile_schedstat_priv *priv = xnvfile_iterator_priv(it);
	struct vfile_schedstat_data *p = data;
	struct xnthread *thread;
	struct xnsched *sched;
	xnticks_t period;
	int ret;

	if (priv->curr == NULL)
		/*
		 * We are done with actual threads, scan interrupt
		 * descriptors.
		 */
		goto scan_irqs;

	thread = priv->curr;
	if (list_is_last(&thread->glink, &nkthreadq))
		priv->curr = NULL;
	else
		priv->curr = list_next_entry(thread, glink);

	sched = thread->sched;
	p->cpu = xnsched_cpu(sched);
	p->pid = xnthread_host_pid(thread);
	memcpy(p->name, thread->name, sizeof(p->name));
	p->state = xnthread_get_state(thread);
	if (thread->lock_count > 0)
		p->state |= XNLOCK;
	p->ssw = xnstat_counter_get(&thread->stat.ssw);
	p->csw = xnstat_counter_get(&thread->stat.csw);
	p->xsc = xnstat_counter_get(&thread->stat.xsc);
	p->pf = xnstat_counter_get(&thread->stat.pf);
	p->sched_class = thread->sched_class;
	p->cprio = thread->cprio;
	p->period = xnthread_get_period(thread);

	period = sched->last_account_switch - thread->stat.lastperiod.start;
	if (period == 0 && thread == sched->curr) {
		p->exectime_period = 1;
		p->account_period = 1;
	} else {
		p->exectime_period = thread->stat.account.total -
			thread->stat.lastperiod.total;
		p->account_period = period;
	}
	p->exectime_total = thread->stat.account.total;
	thread->stat.lastperiod.total = thread->stat.account.total;
	thread->stat.lastperiod.start = sched->last_account_switch;

	return 1;

scan_irqs:
	if (priv->irq >= IPIPE_NR_IRQS)
		return 0;	/* All done. */

	ret = xnintr_query_next(priv->irq, &priv->intr_it, p->name);
	if (ret) {
		if (ret == -EAGAIN)
			xnvfile_touch(it->vfile); /* force rewind. */
		priv->irq++;
		return VFILE_SEQ_SKIP;
	}

	if (!xnsched_supported_cpu(priv->intr_it.cpu))
		return VFILE_SEQ_SKIP;

	p->cpu = priv->intr_it.cpu;
	p->csw = priv->intr_it.hits;
	p->exectime_period = priv->intr_it.exectime_period;
	p->account_period = priv->intr_it.account_period;
	p->exectime_total = priv->intr_it.exectime_total;
	p->pid = 0;
	p->state =  0;
	p->ssw = 0;
	p->xsc = 0;
	p->pf = 0;
	p->sched_class = &xnsched_class_idle;
	p->cprio = 0;
	p->period = 0;

	return 1;
}

static int vfile_schedstat_show(struct xnvfile_snapshot_iterator *it,
				void *data)
{
	struct vfile_schedstat_data *p = data;
	int usage = 0;

	if (p == NULL)
		xnvfile_printf(it,
			       "%-3s  %-6s %-10s %-10s %-10s %-4s  %-8s  %5s"
			       "  %s\n",
			       "CPU", "PID", "MSW", "CSW", "XSC", "PF", "STAT", "%CPU",
			       "NAME");
	else {
		if (p->account_period) {
			while (p->account_period > 0xffffffffUL) {
				p->exectime_period >>= 16;
				p->account_period >>= 16;
			}
			usage = xnarch_ulldiv(p->exectime_period * 1000LL +
					      (p->account_period >> 1),
					      p->account_period, NULL);
		}
		xnvfile_printf(it,
			       "%3u  %-6d %-10lu %-10lu %-10lu %-4lu  %.8x  %3u.%u"
			       "  %s%s%s\n",
			       p->cpu, p->pid, p->ssw, p->csw, p->xsc, p->pf, p->state,
			       usage / 10, usage % 10,
			       (p->state & XNUSER) ? "" : "[",
			       p->name,
			       (p->state & XNUSER) ? "" : "]");
	}

	return 0;
}

static int vfile_schedacct_show(struct xnvfile_snapshot_iterator *it,
				void *data)
{
	struct vfile_schedstat_data *p = data;

	if (p == NULL)
		return 0;

	xnvfile_printf(it, "%u %d %lu %lu %lu %lu %.8x %Lu %Lu %Lu %s %s %d %Lu\n",
		       p->cpu, p->pid, p->ssw, p->csw, p->xsc, p->pf, p->state,
		       xnclock_ticks_to_ns(&nkclock, p->account_period),
		       xnclock_ticks_to_ns(&nkclock, p->exectime_period),
		       xnclock_ticks_to_ns(&nkclock, p->exectime_total),
		       p->name,
		       p->sched_class->name,
		       p->cprio,
		       p->period);

	return 0;
}

static struct xnvfile_snapshot_ops vfile_schedstat_ops = {
	.rewind = vfile_schedstat_rewind,
	.next = vfile_schedstat_next,
	.show = vfile_schedstat_show,
};

/*
 * An accounting vfile is a thread statistics vfile in disguise with a
 * different output format, which is parser-friendly.
 */
static struct xnvfile_snapshot_ops vfile_schedacct_ops;

static struct xnvfile_snapshot schedacct_vfile = {
	.privsz = sizeof(struct vfile_schedstat_priv),
	.datasz = sizeof(struct vfile_schedstat_data),
	.tag = &nkthreadlist_tag,
	.ops = &vfile_schedacct_ops,
};

static struct xnvfile_snapshot_ops vfile_schedacct_ops = {
	.rewind = vfile_schedstat_rewind,
	.next = vfile_schedstat_next,
	.show = vfile_schedacct_show,
};

#endif /* CONFIG_XENO_OPT_STATS */

#ifdef CONFIG_SMP

static int affinity_vfile_show(struct xnvfile_regular_iterator *it,
			       void *data)
{
	unsigned long val = 0;
	int cpu;

	for (cpu = 0; cpu < BITS_PER_LONG; cpu++)
		if (cpumask_test_cpu(cpu, &cobalt_cpu_affinity))
			val |= (1UL << cpu);

	xnvfile_printf(it, "%08lx\n", val);

	return 0;
}

static ssize_t affinity_vfile_store(struct xnvfile_input *input)
{
	cpumask_t affinity, set;
	ssize_t ret;
	long val;
	int cpu;
	spl_t s;

	ret = xnvfile_get_integer(input, &val);
	if (ret < 0)
		return ret;

	if (val == 0)
		affinity = xnsched_realtime_cpus; /* Reset to default. */
	else {
		cpumask_clear(&affinity);
		for (cpu = 0; cpu < BITS_PER_LONG; cpu++, val >>= 1) {
			if (val & 1)
				cpumask_set_cpu(cpu, &affinity);
		}
	}

	cpumask_and(&set, &affinity, cpu_online_mask);
	if (cpumask_empty(&set))
		return -EINVAL;

	/*
	 * The new dynamic affinity must be a strict subset of the
	 * static set of supported CPUs.
	 */
	cpumask_or(&set, &affinity, &xnsched_realtime_cpus);
	if (!cpumask_equal(&set, &xnsched_realtime_cpus))
		return -EINVAL;

	xnlock_get_irqsave(&nklock, s);
	cobalt_cpu_affinity = affinity;
	xnlock_put_irqrestore(&nklock, s);

	return ret;
}

static struct xnvfile_regular_ops affinity_vfile_ops = {
	.show = affinity_vfile_show,
	.store = affinity_vfile_store,
};

static struct xnvfile_regular affinity_vfile = {
	.ops = &affinity_vfile_ops,
};

#endif /* CONFIG_SMP */

int xnsched_init_proc(void)
{
	struct xnsched_class *p;
	int ret;

	ret = xnvfile_init_dir("sched", &sched_vfroot, &cobalt_vfroot);
	if (ret)
		return ret;

	ret = xnvfile_init_snapshot("threads", &schedlist_vfile, &sched_vfroot);
	if (ret)
		return ret;

	for_each_xnsched_class(p) {
		if (p->sched_init_vfile) {
			ret = p->sched_init_vfile(p, &sched_vfroot);
			if (ret)
				return ret;
		}
	}

#ifdef CONFIG_XENO_OPT_STATS
	ret = xnvfile_init_snapshot("stat", &schedstat_vfile, &sched_vfroot);
	if (ret)
		return ret;
	ret = xnvfile_init_snapshot("acct", &schedacct_vfile, &sched_vfroot);
	if (ret)
		return ret;
#endif /* CONFIG_XENO_OPT_STATS */

#ifdef CONFIG_SMP
	xnvfile_init_regular("affinity", &affinity_vfile, &cobalt_vfroot);
#endif /* CONFIG_SMP */

	return 0;
}

void xnsched_cleanup_proc(void)
{
	struct xnsched_class *p;

	for_each_xnsched_class(p) {
		if (p->sched_cleanup_vfile)
			p->sched_cleanup_vfile(p);
	}

#ifdef CONFIG_SMP
	xnvfile_destroy_regular(&affinity_vfile);
#endif /* CONFIG_SMP */
#ifdef CONFIG_XENO_OPT_STATS
	xnvfile_destroy_snapshot(&schedacct_vfile);
	xnvfile_destroy_snapshot(&schedstat_vfile);
#endif /* CONFIG_XENO_OPT_STATS */
	xnvfile_destroy_snapshot(&schedlist_vfile);
	xnvfile_destroy_dir(&sched_vfroot);
}

#endif /* CONFIG_XENO_OPT_VFILE */

/** @} */
