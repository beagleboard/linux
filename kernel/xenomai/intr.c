/*
 * Copyright (C) 2001,2002,2003 Philippe Gerum <rpm@xenomai.org>.
 * Copyright (C) 2005,2006 Dmitry Adamushko <dmitry.adamushko@gmail.com>.
 * Copyright (C) 2007 Jan Kiszka <jan.kiszka@web.de>.
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
#include <linux/mutex.h>
#include <linux/ipipe.h>
#include <linux/ipipe_tickdev.h>
#include <cobalt/kernel/sched.h>
#include <cobalt/kernel/intr.h>
#include <cobalt/kernel/stat.h>
#include <cobalt/kernel/clock.h>
#include <cobalt/kernel/assert.h>
#include <trace/events/cobalt-core.h>

/**
 * @ingroup cobalt_core
 * @defgroup cobalt_core_irq Interrupt management
 * @{
 */
#define XNINTR_MAX_UNHANDLED	1000

static DEFINE_MUTEX(intrlock);

#ifdef CONFIG_XENO_OPT_STATS
struct xnintr nktimer;	     /* Only for statistics */
static int xnintr_count = 1; /* Number of attached xnintr objects + nktimer */
static int xnintr_list_rev;  /* Modification counter of xnintr list */

/* Both functions update xnintr_list_rev at the very end.
 * This guarantees that module.c::stat_seq_open() won't get
 * an up-to-date xnintr_list_rev and old xnintr_count. */

static inline void stat_counter_inc(void)
{
	xnintr_count++;
	smp_mb();
	xnintr_list_rev++;
}

static inline void stat_counter_dec(void)
{
	xnintr_count--;
	smp_mb();
	xnintr_list_rev++;
}

static inline void sync_stat_references(struct xnintr *intr)
{
	struct xnirqstat *statp;
	struct xnsched *sched;
	int cpu;

	for_each_realtime_cpu(cpu) {
		sched = xnsched_struct(cpu);
		statp = per_cpu_ptr(intr->stats, cpu);
		/* Synchronize on all dangling references to go away. */
		while (sched->current_account == &statp->account)
			cpu_relax();
	}
}

static void clear_irqstats(struct xnintr *intr)
{
	struct xnirqstat *p;
	int cpu;

	for_each_realtime_cpu(cpu) {
		p = per_cpu_ptr(intr->stats, cpu);
		memset(p, 0, sizeof(*p));
	}
}

static inline void alloc_irqstats(struct xnintr *intr)
{
	intr->stats = alloc_percpu(struct xnirqstat);
	clear_irqstats(intr);
}

static inline void free_irqstats(struct xnintr *intr)
{
	free_percpu(intr->stats);
}

static inline void query_irqstats(struct xnintr *intr, int cpu,
				  struct xnintr_iterator *iterator)
{
	struct xnirqstat *statp;
	xnticks_t last_switch;

	statp = per_cpu_ptr(intr->stats, cpu);
	iterator->hits = xnstat_counter_get(&statp->hits);
	last_switch = xnsched_struct(cpu)->last_account_switch;
	iterator->exectime_period = statp->account.total;
	iterator->account_period = last_switch - statp->account.start;
	statp->sum.total += iterator->exectime_period;
	iterator->exectime_total = statp->sum.total;
	statp->account.total = 0;
	statp->account.start = last_switch;
}

static void inc_irqstats(struct xnintr *intr, struct xnsched *sched, xnticks_t start)
{
	struct xnirqstat *statp;

	statp = raw_cpu_ptr(intr->stats);
	xnstat_counter_inc(&statp->hits);
	xnstat_exectime_lazy_switch(sched, &statp->account, start);
}

static inline void switch_irqstats(struct xnintr *intr, struct xnsched *sched)
{
	struct xnirqstat *statp;

	statp = raw_cpu_ptr(intr->stats);
	xnstat_exectime_switch(sched, &statp->account);
}

static inline xnstat_exectime_t *switch_core_irqstats(struct xnsched *sched)
{
	struct xnirqstat *statp;
	xnstat_exectime_t *prev;

	statp = xnstat_percpu_data;
	prev = xnstat_exectime_switch(sched, &statp->account);
	xnstat_counter_inc(&statp->hits);

	return prev;
}

#else  /* !CONFIG_XENO_OPT_STATS */

static inline void stat_counter_inc(void) {}

static inline void stat_counter_dec(void) {}

static inline void sync_stat_references(struct xnintr *intr) {}

static inline void alloc_irqstats(struct xnintr *intr) {}

static inline void free_irqstats(struct xnintr *intr) {}

static inline void clear_irqstats(struct xnintr *intr) {}

static inline void query_irqstats(struct xnintr *intr, int cpu,
				  struct xnintr_iterator *iterator) {}

static inline void inc_irqstats(struct xnintr *intr, struct xnsched *sched, xnticks_t start) {}

static inline void switch_irqstats(struct xnintr *intr, struct xnsched *sched) {}

static inline xnstat_exectime_t *switch_core_irqstats(struct xnsched *sched)
{
	return NULL;
}

#endif /* !CONFIG_XENO_OPT_STATS */

static void xnintr_irq_handler(unsigned int irq, void *cookie);

void xnintr_host_tick(struct xnsched *sched) /* Interrupts off. */
{
	sched->lflags &= ~XNHTICK;
#ifdef XNARCH_HOST_TICK_IRQ
	ipipe_post_irq_root(XNARCH_HOST_TICK_IRQ);
#endif
}

/*
 * Low-level core clock irq handler. This one forwards ticks from the
 * Xenomai platform timer to nkclock exclusively.
 */
void xnintr_core_clock_handler(void)
{
	struct xnsched *sched = xnsched_current();
	int cpu  __maybe_unused = xnsched_cpu(sched);
	xnstat_exectime_t *prev;

	if (!xnsched_supported_cpu(cpu)) {
#ifdef XNARCH_HOST_TICK_IRQ
		ipipe_post_irq_root(XNARCH_HOST_TICK_IRQ);
#endif
		return;
	}

	prev = switch_core_irqstats(sched);

	trace_cobalt_clock_entry(per_cpu(ipipe_percpu.hrtimer_irq, cpu));

	++sched->inesting;
	sched->lflags |= XNINIRQ;

	xnlock_get(&nklock);
	xnclock_tick(&nkclock);
	xnlock_put(&nklock);

	trace_cobalt_clock_exit(per_cpu(ipipe_percpu.hrtimer_irq, cpu));
	xnstat_exectime_switch(sched, prev);

	if (--sched->inesting == 0) {
		sched->lflags &= ~XNINIRQ;
		xnsched_run();
		sched = xnsched_current();
	}
	/*
	 * If the core clock interrupt preempted a real-time thread,
	 * any transition to the root thread has already triggered a
	 * host tick propagation from xnsched_run(), so at this point,
	 * we only need to propagate the host tick in case the
	 * interrupt preempted the root thread.
	 */
	if ((sched->lflags & XNHTICK) &&
	    xnthread_test_state(sched->curr, XNROOT))
		xnintr_host_tick(sched);
}

struct irqdisable_work {
	struct ipipe_work_header work; /* Must be first. */
	int irq;
};

static void lostage_irqdisable_line(struct ipipe_work_header *work)
{
	struct irqdisable_work *rq;

	rq = container_of(work, struct irqdisable_work, work);
	ipipe_disable_irq(rq->irq);
}

static void disable_irq_line(int irq)
{
	struct irqdisable_work diswork = {
		.work = {
			.size = sizeof(diswork),
			.handler = lostage_irqdisable_line,
		},
		.irq = irq,
	};

	ipipe_post_work_root(&diswork, work);
}

/* Optional support for shared interrupts. */

#ifdef CONFIG_XENO_OPT_SHIRQ

struct xnintr_vector {
	DECLARE_XNLOCK(lock);
	struct xnintr *handlers;
	int unhandled;
} ____cacheline_aligned_in_smp;

static struct xnintr_vector vectors[IPIPE_NR_IRQS];

static inline struct xnintr *xnintr_vec_first(unsigned int irq)
{
	return vectors[irq].handlers;
}

static inline struct xnintr *xnintr_vec_next(struct xnintr *prev)
{
	return prev->next;
}

static void disable_shared_irq_line(struct xnintr_vector *vec)
{
	int irq = vec - vectors;
	struct xnintr *intr;

	xnlock_get(&vec->lock);
	intr = vec->handlers;
	while (intr) {
		set_bit(XN_IRQSTAT_DISABLED, &intr->status);
		intr = intr->next;
	}
	xnlock_put(&vec->lock);
	disable_irq_line(irq);
}

/*
 * Low-level interrupt handler dispatching the user-defined ISRs for
 * shared interrupts -- Called with interrupts off.
 */
static void xnintr_vec_handler(unsigned int irq, void *cookie)
{
	struct xnsched *sched = xnsched_current();
	struct xnintr_vector *vec = vectors + irq;
	xnstat_exectime_t *prev;
	struct xnintr *intr;
	xnticks_t start;
	int s = 0, ret;

	prev  = xnstat_exectime_get_current(sched);
	start = xnstat_exectime_now();
	trace_cobalt_irq_entry(irq);

	++sched->inesting;
	sched->lflags |= XNINIRQ;

	xnlock_get(&vec->lock);
	intr = vec->handlers;
	if (unlikely(test_bit(XN_IRQSTAT_DISABLED, &intr->status))) {
		/* irqdisable_work is on its way, ignore. */
		xnlock_put(&vec->lock);
		goto out;
	}

	while (intr) {
		/*
		 * NOTE: We assume that no CPU migration can occur
		 * while running the interrupt service routine.
		 */
		ret = intr->isr(intr);
		XENO_WARN_ON_ONCE(USER, (ret & XN_IRQ_STATMASK) == 0);
		s |= ret;
		if (ret & XN_IRQ_HANDLED) {
			inc_irqstats(intr, sched, start);
			start = xnstat_exectime_now();
		}
		intr = intr->next;
	}

	xnlock_put(&vec->lock);

	if (unlikely(!(s & XN_IRQ_HANDLED))) {
		if (++vec->unhandled == XNINTR_MAX_UNHANDLED) {
			printk(XENO_ERR "%s: IRQ%d not handled. Disabling IRQ line\n",
			       __FUNCTION__, irq);
			s |= XN_IRQ_DISABLE;
		}
	} else
		vec->unhandled = 0;

	if (s & XN_IRQ_PROPAGATE)
		ipipe_post_irq_root(irq);
	else if (s & XN_IRQ_DISABLE)
		disable_shared_irq_line(vec);
	else
		ipipe_end_irq(irq);
out:
	xnstat_exectime_switch(sched, prev);

	if (--sched->inesting == 0) {
		sched->lflags &= ~XNINIRQ;
		xnsched_run();
	}

	trace_cobalt_irq_exit(irq);
}

/*
 * Low-level interrupt handler dispatching the user-defined ISRs for
 * shared edge-triggered interrupts -- Called with interrupts off.
 */
static void xnintr_edge_vec_handler(unsigned int irq, void *cookie)
{
	const int MAX_EDGEIRQ_COUNTER = 128;
	struct xnsched *sched = xnsched_current();
	struct xnintr_vector *vec = vectors + irq;
	struct xnintr *intr, *end = NULL;
	int s = 0, counter = 0, ret;
	xnstat_exectime_t *prev;
	xnticks_t start;

	prev  = xnstat_exectime_get_current(sched);
	start = xnstat_exectime_now();
	trace_cobalt_irq_entry(irq);

	++sched->inesting;
	sched->lflags |= XNINIRQ;

	xnlock_get(&vec->lock);
	intr = vec->handlers;
	if (unlikely(test_bit(XN_IRQSTAT_DISABLED, &intr->status))) {
		/* irqdisable_work is on its way, ignore. */
		xnlock_put(&vec->lock);
		goto out;
	}

	while (intr != end) {
		switch_irqstats(intr, sched);
		/*
		 * NOTE: We assume that no CPU migration will occur
		 * while running the interrupt service routine.
		 */
		ret = intr->isr(intr);
		XENO_WARN_ON_ONCE(USER, (ret & XN_IRQ_STATMASK) == 0);
		s |= ret;

		if (ret & XN_IRQ_HANDLED) {
			end = NULL;
			inc_irqstats(intr, sched, start);
			start = xnstat_exectime_now();
		} else if (end == NULL)
			end = intr;

		if (counter++ > MAX_EDGEIRQ_COUNTER)
			break;

		intr = intr->next;
		if (intr  == NULL)
			intr = vec->handlers;
	}

	xnlock_put(&vec->lock);

	if (counter > MAX_EDGEIRQ_COUNTER)
		printk(XENO_ERR "%s: failed to get the IRQ%d line free\n",
		       __FUNCTION__, irq);

	if (unlikely(!(s & XN_IRQ_HANDLED))) {
		if (++vec->unhandled == XNINTR_MAX_UNHANDLED) {
			printk(XENO_ERR "%s: IRQ%d not handled. Disabling IRQ line\n",
			       __FUNCTION__, irq);
			s |= XN_IRQ_DISABLE;
		}
	} else
		vec->unhandled = 0;

	if (s & XN_IRQ_PROPAGATE)
		ipipe_post_irq_root(irq);
	else if (s & XN_IRQ_DISABLE)
		disable_shared_irq_line(vec);
	else
		ipipe_end_irq(irq);
out:
	xnstat_exectime_switch(sched, prev);

	if (--sched->inesting == 0) {
		sched->lflags &= ~XNINIRQ;
		xnsched_run();
	}

	trace_cobalt_irq_exit(irq);
}

static inline int xnintr_irq_attach(struct xnintr *intr)
{
	struct xnintr_vector *vec = vectors + intr->irq;
	struct xnintr *prev, **p = &vec->handlers;
	int ret;

	prev = *p;
	if (prev) {
		/* Check on whether the shared mode is allowed. */
		if ((prev->flags & intr->flags & XN_IRQTYPE_SHARED) == 0 ||
		    (prev->iack != intr->iack)
		    || ((prev->flags & XN_IRQTYPE_EDGE) !=
			(intr->flags & XN_IRQTYPE_EDGE)))
			return -EBUSY;

		/*
		 * Get a position at the end of the list to insert the
		 * new element.
		 */
		while (prev) {
			p = &prev->next;
			prev = *p;
		}
	} else {
		/* Initialize the corresponding interrupt channel */
		void (*handler) (unsigned, void *) = xnintr_irq_handler;

		if (intr->flags & XN_IRQTYPE_SHARED) {
			if (intr->flags & XN_IRQTYPE_EDGE)
				handler = xnintr_edge_vec_handler;
			else
				handler = xnintr_vec_handler;

		}
		vec->unhandled = 0;

		ret = ipipe_request_irq(&xnsched_realtime_domain,
					intr->irq, handler, intr,
					(ipipe_irq_ackfn_t)intr->iack);
		if (ret)
			return ret;
	}

	intr->next = NULL;
	/*
	 * Add the given interrupt object. No need to synchronise with
	 * the IRQ handler, we are only extending the chain.
	 */
	*p = intr;

	return 0;
}

static inline void xnintr_irq_detach(struct xnintr *intr)
{
	struct xnintr_vector *vec = vectors + intr->irq;
	struct xnintr *e, **p = &vec->handlers;

	while ((e = *p) != NULL) {
		if (e == intr) {
			/* Remove the given interrupt object from the list. */
			xnlock_get(&vec->lock);
			*p = e->next;
			xnlock_put(&vec->lock);

			sync_stat_references(intr);

			/* Release the IRQ line if this was the last user */
			if (vec->handlers == NULL)
				ipipe_free_irq(&xnsched_realtime_domain, intr->irq);

			return;
		}
		p = &e->next;
	}

	printk(XENO_ERR "attempted to detach an unregistered interrupt descriptor\n");
}

#else /* !CONFIG_XENO_OPT_SHIRQ */

struct xnintr_vector {
#if defined(CONFIG_SMP) || XENO_DEBUG(LOCKING)
	DECLARE_XNLOCK(lock);
#endif /* CONFIG_SMP || XENO_DEBUG(LOCKING) */
} ____cacheline_aligned_in_smp;

static struct xnintr_vector vectors[IPIPE_NR_IRQS];

static inline struct xnintr *xnintr_vec_first(unsigned int irq)
{
	return __ipipe_irq_cookie(&xnsched_realtime_domain, irq);
}

static inline struct xnintr *xnintr_vec_next(struct xnintr *prev)
{
	return NULL;
}

static inline int xnintr_irq_attach(struct xnintr *intr)
{
	return ipipe_request_irq(&xnsched_realtime_domain,
				 intr->irq, xnintr_irq_handler, intr,
				 (ipipe_irq_ackfn_t)intr->iack);
}

static inline void xnintr_irq_detach(struct xnintr *intr)
{
	int irq = intr->irq;

	xnlock_get(&vectors[irq].lock);
	ipipe_free_irq(&xnsched_realtime_domain, irq);
	xnlock_put(&vectors[irq].lock);

	sync_stat_references(intr);
}

#endif /* !CONFIG_XENO_OPT_SHIRQ */

/*
 * Low-level interrupt handler dispatching non-shared ISRs -- Called
 * with interrupts off.
 */
static void xnintr_irq_handler(unsigned int irq, void *cookie)
{
	struct xnintr_vector __maybe_unused *vec = vectors + irq;
	struct xnsched *sched = xnsched_current();
	xnstat_exectime_t *prev;
	struct xnintr *intr;
	xnticks_t start;
	int s = 0;

	prev  = xnstat_exectime_get_current(sched);
	start = xnstat_exectime_now();
	trace_cobalt_irq_entry(irq);

	++sched->inesting;
	sched->lflags |= XNINIRQ;

	xnlock_get(&vec->lock);

#ifdef CONFIG_SMP
	/*
	 * In SMP case, we have to reload the cookie under the per-IRQ
	 * lock to avoid racing with xnintr_detach.  However, we
	 * assume that no CPU migration will occur while running the
	 * interrupt service routine, so the scheduler pointer will
	 * remain valid throughout this function.
	 */
	intr = __ipipe_irq_cookie(&xnsched_realtime_domain, irq);
	if (unlikely(intr == NULL))
		goto done;
#else
	intr = cookie;
#endif
	if (unlikely(test_bit(XN_IRQSTAT_DISABLED, &intr->status))) {
		/* irqdisable_work is on its way, ignore. */
		xnlock_put(&vec->lock);
		goto out;
	}

	s = intr->isr(intr);
	XENO_WARN_ON_ONCE(USER, (s & XN_IRQ_STATMASK) == 0);
	if (unlikely(!(s & XN_IRQ_HANDLED))) {
		if (++intr->unhandled == XNINTR_MAX_UNHANDLED) {
			printk(XENO_ERR "%s: IRQ%d not handled. Disabling IRQ line\n",
			       __FUNCTION__, irq);
			s |= XN_IRQ_DISABLE;
		}
	} else {
		inc_irqstats(intr, sched, start);
		intr->unhandled = 0;
	}

	if (s & XN_IRQ_DISABLE)
		set_bit(XN_IRQSTAT_DISABLED, &intr->status);
#ifdef CONFIG_SMP
done:
#endif
	xnlock_put(&vec->lock);

	if (s & XN_IRQ_DISABLE)
		disable_irq_line(irq);
	else if (s & XN_IRQ_PROPAGATE)
		ipipe_post_irq_root(irq);
	else
		ipipe_end_irq(irq);
out:
	xnstat_exectime_switch(sched, prev);

	if (--sched->inesting == 0) {
		sched->lflags &= ~XNINIRQ;
		xnsched_run();
	}

	trace_cobalt_irq_exit(irq);
}

int __init xnintr_mount(void)
{
	int i;
	for (i = 0; i < IPIPE_NR_IRQS; ++i)
		xnlock_init(&vectors[i].lock);
	return 0;
}

/**
 * @fn int xnintr_init(struct xnintr *intr,const char *name,unsigned int irq,xnisr_t isr,xniack_t iack,int flags)
 * @brief Initialize an interrupt descriptor.
 *
 * When an interrupt occurs on the given @a irq line, the interrupt
 * service routine @a isr is fired in order to deal with the hardware
 * event. The interrupt handler may call any non-blocking service from
 * the Cobalt core.
 *
 * Upon receipt of an IRQ, the interrupt handler @a isr is immediately
 * called on behalf of the interrupted stack context, the rescheduling
 * procedure is locked, and the interrupt line is masked in the system
 * interrupt controller chip.  Upon return, the status of the
 * interrupt handler is checked for the following bits:
 *
 * - XN_IRQ_HANDLED indicates that the interrupt request was
 * successfully handled.
 *
 * - XN_IRQ_NONE indicates the opposite to XN_IRQ_HANDLED, meaning
 * that no interrupt source could be identified for the ongoing
 * request by the handler.
 *
 * In addition, one of the following bits may be present in the
 * status:
 *
 * - XN_IRQ_DISABLE tells the Cobalt core to disable the interrupt
 * line before returning from the interrupt context.
 *
 * - XN_IRQ_PROPAGATE propagates the IRQ event down the interrupt
 * pipeline to Linux. Using this flag is strongly discouraged, unless
 * you fully understand the implications of such propagation.
 *
 * @warning The handler should not use these bits if it shares the
 * interrupt line with other handlers in the real-time domain. When
 * any of these bits is detected, the interrupt line is left masked.
 *
 * A count of interrupt receipts is tracked into the interrupt
 * descriptor, and reset to zero each time such descriptor is
 * attached. Since this count could wrap around, it should be used as
 * an indication of interrupt activity only.
 *
 * @param intr The address of a descriptor the Cobalt core will use to
 * store the interrupt-specific data.
 *
 * @param name An ASCII string standing for the symbolic name of the
 * interrupt or NULL.
 *
 * @param irq The IRQ line number associated with the interrupt
 * descriptor. This value is architecture-dependent. An interrupt
 * descriptor must be attached to the system by a call to
 * xnintr_attach() before @a irq events can be received.
 *
 * @param isr The address of an interrupt handler, which is passed the
 * address of the interrupt descriptor receiving the IRQ.
 *
 * @param iack The address of an optional interrupt acknowledge
 * routine, aimed at replacing the default one. Only very specific
 * situations actually require to override the default setting for
 * this parameter, like having to acknowledge non-standard PIC
 * hardware. @a iack should return a non-zero value to indicate that
 * the interrupt has been properly acknowledged. If @a iack is NULL,
 * the default routine will be used instead.
 *
 * @param flags A set of creation flags affecting the operation. The
 * valid flags are:
 *
 * - XN_IRQTYPE_SHARED enables IRQ-sharing with other interrupt
 * objects.
 *
 * - XN_IRQTYPE_EDGE is an additional flag need to be set together
 * with XN_IRQTYPE_SHARED to enable IRQ-sharing of edge-triggered
 * interrupts.
 *
 * @return 0 is returned on success. Otherwise, -EINVAL is returned if
 * @a irq is not a valid interrupt number.
 *
 * @coretags{secondary-only}
 */
int xnintr_init(struct xnintr *intr, const char *name,
		unsigned int irq, xnisr_t isr, xniack_t iack,
		int flags)
{
	secondary_mode_only();

	if (irq >= IPIPE_NR_IRQS)
		return -EINVAL;

	intr->irq = irq;
	intr->isr = isr;
	intr->iack = iack;
	intr->cookie = NULL;
	intr->name = name ? : "<unknown>";
	intr->flags = flags;
	intr->status = _XN_IRQSTAT_DISABLED;
	intr->unhandled = 0;
	raw_spin_lock_init(&intr->lock);
#ifdef CONFIG_XENO_OPT_SHIRQ
	intr->next = NULL;
#endif
	alloc_irqstats(intr);

	return 0;
}
EXPORT_SYMBOL_GPL(xnintr_init);

/**
 * @fn void xnintr_destroy(struct xnintr *intr)
 * @brief Destroy an interrupt descriptor.
 *
 * Destroys an interrupt descriptor previously initialized by
 * xnintr_init(). The descriptor is automatically detached by a call
 * to xnintr_detach(). No more IRQs will be received through this
 * descriptor after this service has returned.
 *
 * @param intr The address of the interrupt descriptor to destroy.
 *
 * @coretags{secondary-only}
 */
void xnintr_destroy(struct xnintr *intr)
{
	secondary_mode_only();
	xnintr_detach(intr);
	free_irqstats(intr);
}
EXPORT_SYMBOL_GPL(xnintr_destroy);

/**
 * @fn int xnintr_attach(struct xnintr *intr, void *cookie)
 * @brief Attach an interrupt descriptor.
 *
 * Attach an interrupt descriptor previously initialized by
 * xnintr_init(). This operation registers the descriptor at the
 * interrupt pipeline, but does not enable the interrupt line yet. A
 * call to xnintr_enable() is required to start receiving IRQs from
 * the interrupt line associated to the descriptor.
 *
 * @param intr The address of the interrupt descriptor to attach.
 *
 * @param cookie A user-defined opaque value which is stored into the
 * descriptor for further retrieval by the interrupt handler.
 *
 * @return 0 is returned on success. Otherwise:
 *
 * - -EINVAL is returned if an error occurred while attaching the
 * descriptor.
 *
 * - -EBUSY is returned if the descriptor was already attached.
 *
 * @note The caller <b>must not</b> hold nklock when invoking this service,
 * this would cause deadlocks.
 *
 * @coretags{secondary-only}
 *
 * @note Attaching an interrupt descriptor resets the tracked number
 * of IRQ receipts to zero.
 */
int xnintr_attach(struct xnintr *intr, void *cookie)
{
	int ret;

	secondary_mode_only();
	trace_cobalt_irq_attach(intr->irq);

	intr->cookie = cookie;
	clear_irqstats(intr);

#ifdef CONFIG_SMP
	ipipe_set_irq_affinity(intr->irq, cobalt_cpu_affinity);
#endif /* CONFIG_SMP */

	raw_spin_lock(&intr->lock);

	if (test_and_set_bit(XN_IRQSTAT_ATTACHED, &intr->status)) {
		ret = -EBUSY;
		goto out;
	}

	ret = xnintr_irq_attach(intr);
	if (ret) {
		clear_bit(XN_IRQSTAT_ATTACHED, &intr->status);
		goto out;
	}

	stat_counter_inc();
out:
	raw_spin_unlock(&intr->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(xnintr_attach);

/**
 * @fn int xnintr_detach(struct xnintr *intr)
 * @brief Detach an interrupt descriptor.
 *
 * This call unregisters an interrupt descriptor previously attached
 * by xnintr_attach() from the interrupt pipeline. Once detached, the
 * associated interrupt line is disabled, but the descriptor remains
 * valid. The descriptor can be attached anew by a call to
 * xnintr_attach().
 *
 * @param intr The address of the interrupt descriptor to detach.
 *
 * @note The caller <b>must not</b> hold nklock when invoking this
 * service, this would cause deadlocks.
 *
 * @coretags{secondary-only}
 */
void xnintr_detach(struct xnintr *intr)
{
	secondary_mode_only();
	trace_cobalt_irq_detach(intr->irq);

	raw_spin_lock(&intr->lock);

	if (test_and_clear_bit(XN_IRQSTAT_ATTACHED, &intr->status)) {
		xnintr_irq_detach(intr);
		stat_counter_dec();
	}

	raw_spin_unlock(&intr->lock);
}
EXPORT_SYMBOL_GPL(xnintr_detach);

/**
 * @fn void xnintr_enable(struct xnintr *intr)
 * @brief Enable an interrupt line.
 *
 * Enables the interrupt line associated with an interrupt descriptor.
 *
 * @param intr The address of the interrupt descriptor.
 *
 * @coretags{secondary-only}
 */
void xnintr_enable(struct xnintr *intr)
{
	unsigned long flags;

	secondary_mode_only();
	trace_cobalt_irq_enable(intr->irq);

	raw_spin_lock_irqsave(&intr->lock, flags);

	/*
	 * If disabled on entry, there is no way we could race with
	 * disable_irq_line().
	 */
	if (test_and_clear_bit(XN_IRQSTAT_DISABLED, &intr->status))
		ipipe_enable_irq(intr->irq);

	raw_spin_unlock_irqrestore(&intr->lock, flags);
}
EXPORT_SYMBOL_GPL(xnintr_enable);

/**
 * @fn void xnintr_disable(struct xnintr *intr)
 * @brief Disable an interrupt line.
 *
 * Disables the interrupt line associated with an interrupt
 * descriptor.
 *
 * @param intr The address of the interrupt descriptor.
 *
 * @coretags{secondary-only}
 */
void xnintr_disable(struct xnintr *intr)
{
	unsigned long flags;

	secondary_mode_only();
	trace_cobalt_irq_disable(intr->irq);

	/* We only need a virtual masking. */
	raw_spin_lock_irqsave(&intr->lock, flags);

	/*
	 * Racing with disable_irq_line() is innocuous, the pipeline
	 * would serialize calls to ipipe_disable_irq() across CPUs,
	 * and the descriptor status would still properly match the
	 * line status in the end.
	 */
	if (!test_and_set_bit(XN_IRQSTAT_DISABLED, &intr->status))
		ipipe_disable_irq(intr->irq);

	raw_spin_unlock_irqrestore(&intr->lock, flags);
}
EXPORT_SYMBOL_GPL(xnintr_disable);

/**
 * @fn void xnintr_affinity(struct xnintr *intr, cpumask_t cpumask)
 * @brief Set processor affinity of interrupt.
 *
 * Restricts the IRQ line associated with the interrupt descriptor @a
 * intr to be received only on processors which bits are set in @a
 * cpumask.
 *
 * @param intr The address of the interrupt descriptor.
 *
 * @param cpumask The new processor affinity.
 *
 * @note Depending on architectures, setting more than one bit in @a
 * cpumask could be meaningless.
 *
 * @coretags{secondary-only}
 */
void xnintr_affinity(struct xnintr *intr, cpumask_t cpumask)
{
	secondary_mode_only();
#ifdef CONFIG_SMP
	ipipe_set_irq_affinity(intr->irq, cpumask);
#endif
}
EXPORT_SYMBOL_GPL(xnintr_affinity);

static inline int xnintr_is_timer_irq(int irq)
{
	int cpu;

	for_each_realtime_cpu(cpu)
		if (irq == per_cpu(ipipe_percpu.hrtimer_irq, cpu))
			return 1;

	return 0;
}

#ifdef CONFIG_XENO_OPT_STATS

int xnintr_get_query_lock(void)
{
	return mutex_lock_interruptible(&intrlock) ? -ERESTARTSYS : 0;
}

void xnintr_put_query_lock(void)
{
	mutex_unlock(&intrlock);
}

int xnintr_query_init(struct xnintr_iterator *iterator)
{
	iterator->cpu = -1;
	iterator->prev = NULL;

	/* The order is important here: first xnintr_list_rev then
	 * xnintr_count.  On the other hand, xnintr_attach/detach()
	 * update xnintr_count first and then xnintr_list_rev.  This
	 * should guarantee that we can't get an up-to-date
	 * xnintr_list_rev and old xnintr_count here. The other way
	 * around is not a problem as xnintr_query() will notice this
	 * fact later.  Should xnintr_list_rev change later,
	 * xnintr_query() will trigger an appropriate error below.
	 */
	iterator->list_rev = xnintr_list_rev;
	smp_mb();

	return xnintr_count;
}

int xnintr_query_next(int irq, struct xnintr_iterator *iterator,
		      char *name_buf)
{
	int cpu, nr_cpus = num_present_cpus();
	struct xnintr *intr;

	for (cpu = iterator->cpu + 1; cpu < nr_cpus; ++cpu) {
		if (cpu_online(cpu))
			break;
	}
	if (cpu == nr_cpus)
		cpu = 0;
	iterator->cpu = cpu;

	if (iterator->list_rev != xnintr_list_rev)
		return -EAGAIN;

	if (!iterator->prev) {
		if (xnintr_is_timer_irq(irq))
			intr = &nktimer;
		else
			intr = xnintr_vec_first(irq);
	} else
		intr = xnintr_vec_next(iterator->prev);

	if (intr == NULL) {
		cpu = -1;
		iterator->prev = NULL;
		return -ENODEV;
	}

	ksformat(name_buf, XNOBJECT_NAME_LEN, "IRQ%d: %s", irq, intr->name);

	query_irqstats(intr, cpu, iterator);

	/*
	 * Proceed to next entry in shared IRQ chain when all CPUs
	 * have been visited for this one.
	 */
	if (cpu + 1 == nr_cpus)
		iterator->prev = intr;

	return 0;
}

#endif /* CONFIG_XENO_OPT_STATS */

#ifdef CONFIG_XENO_OPT_VFILE

#include <cobalt/kernel/vfile.h>

static inline int format_irq_proc(unsigned int irq,
				  struct xnvfile_regular_iterator *it)
{
	struct xnintr *intr;
	int cpu;

	for_each_realtime_cpu(cpu)
		if (xnintr_is_timer_irq(irq)) {
			xnvfile_printf(it, "         [timer/%d]", cpu);
			return 0;
		}

#ifdef CONFIG_SMP
	/*
	 * IPI numbers on ARM are not compile time constants, so do
	 * not use switch/case here.
	 */
	if (irq == IPIPE_HRTIMER_IPI) {
		xnvfile_puts(it, "         [timer-ipi]");
		return 0;
	}
	if (irq == IPIPE_RESCHEDULE_IPI) {
		xnvfile_puts(it, "         [reschedule]");
		return 0;
	}
	if (irq == IPIPE_CRITICAL_IPI) {
		xnvfile_puts(it, "         [sync]");
		return 0;
	}
#endif /* CONFIG_SMP */
	if (ipipe_virtual_irq_p(irq)) {
		xnvfile_puts(it, "         [virtual]");
		return 0;
	}

	mutex_lock(&intrlock);

	intr = xnintr_vec_first(irq);
	if (intr) {
		xnvfile_puts(it, "        ");

		do {
			xnvfile_putc(it, ' ');
			xnvfile_puts(it, intr->name);
			intr = xnintr_vec_next(intr);
		} while (intr);
	}

	mutex_unlock(&intrlock);

	return 0;
}

static int irq_vfile_show(struct xnvfile_regular_iterator *it,
			  void *data)
{
	int cpu, irq;

	/* FIXME: We assume the entire output fits in a single page. */

	xnvfile_puts(it, "  IRQ ");

	for_each_realtime_cpu(cpu)
		xnvfile_printf(it, "        CPU%d", cpu);

	for (irq = 0; irq < IPIPE_NR_IRQS; irq++) {
		if (__ipipe_irq_handler(&xnsched_realtime_domain, irq) == NULL)
			continue;

		xnvfile_printf(it, "\n%5d:", irq);

		for_each_realtime_cpu(cpu) {
			xnvfile_printf(it, "%12lu",
				       __ipipe_cpudata_irq_hits(&xnsched_realtime_domain, cpu,
								irq));
		}

		format_irq_proc(irq, it);
	}

	xnvfile_putc(it, '\n');

	return 0;
}

static struct xnvfile_regular_ops irq_vfile_ops = {
	.show = irq_vfile_show,
};

static struct xnvfile_regular irq_vfile = {
	.ops = &irq_vfile_ops,
};

void xnintr_init_proc(void)
{
	xnvfile_init_regular("irq", &irq_vfile, &cobalt_vfroot);
}

void xnintr_cleanup_proc(void)
{
	xnvfile_destroy_regular(&irq_vfile);
}

#endif /* CONFIG_XENO_OPT_VFILE */

/** @} */
