/*
 * Copyright (C) 2006-2011 Philippe Gerum <rpm@xenomai.org>.
 *
 * Xenomai is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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
#include <linux/percpu.h>
#include <linux/errno.h>
#include <linux/ipipe_tickdev.h>
#include <cobalt/kernel/sched.h>
#include <cobalt/kernel/timer.h>
#include <cobalt/kernel/clock.h>
#include <cobalt/kernel/arith.h>
#include <cobalt/kernel/vdso.h>
#include <cobalt/uapi/time.h>
#include <asm/xenomai/calibration.h>
#include <trace/events/cobalt-core.h>
/**
 * @ingroup cobalt_core
 * @defgroup cobalt_core_clock Clock services
 *
 * @{
 */
unsigned long nktimerlat;

static unsigned long long clockfreq;

#ifdef XNARCH_HAVE_LLMULSHFT

static unsigned int tsc_scale, tsc_shift;

#ifdef XNARCH_HAVE_NODIV_LLIMD

static struct xnarch_u32frac tsc_frac;
static struct xnarch_u32frac bln_frac;

long long xnclock_core_ns_to_ticks(long long ns)
{
	return xnarch_nodiv_llimd(ns, tsc_frac.frac, tsc_frac.integ);
}

unsigned long long xnclock_divrem_billion(unsigned long long value,
					  unsigned long *rem)
{
	unsigned long long q;
	unsigned r;

	q = xnarch_nodiv_ullimd(value, bln_frac.frac, bln_frac.integ);
	r = value - q * 1000000000;
	if (r >= 1000000000) {
		++q;
		r -= 1000000000;
	}
	*rem = r;
	return q;
}

#else /* !XNARCH_HAVE_NODIV_LLIMD */

long long xnclock_core_ns_to_ticks(long long ns)
{
	return xnarch_llimd(ns, 1 << tsc_shift, tsc_scale);
}

#endif /* !XNARCH_HAVE_NODIV_LLIMD */

xnsticks_t xnclock_core_ticks_to_ns(xnsticks_t ticks)
{
	return xnarch_llmulshft(ticks, tsc_scale, tsc_shift);
}

xnsticks_t xnclock_core_ticks_to_ns_rounded(xnsticks_t ticks)
{
	unsigned int shift = tsc_shift - 1;
	return (xnarch_llmulshft(ticks, tsc_scale, shift) + 1) / 2;
}

#else  /* !XNARCH_HAVE_LLMULSHFT */

xnsticks_t xnclock_core_ticks_to_ns(xnsticks_t ticks)
{
	return xnarch_llimd(ticks, 1000000000, clockfreq);
}

xnsticks_t xnclock_core_ticks_to_ns_rounded(xnsticks_t ticks)
{
	return (xnarch_llimd(ticks, 1000000000, clockfreq/2) + 1) / 2;
}

xnsticks_t xnclock_core_ns_to_ticks(xnsticks_t ns)
{
	return xnarch_llimd(ns, clockfreq, 1000000000);
}

#endif /* !XNARCH_HAVE_LLMULSHFT */

#ifndef XNARCH_HAVE_NODIV_LLIMD
unsigned long long xnclock_divrem_billion(unsigned long long value,
					  unsigned long *rem)
{
	return xnarch_ulldiv(value, 1000000000, rem);

}
#endif /* !XNARCH_HAVE_NODIV_LLIMD */

EXPORT_SYMBOL_GPL(xnclock_core_ticks_to_ns);
EXPORT_SYMBOL_GPL(xnclock_core_ticks_to_ns_rounded);
EXPORT_SYMBOL_GPL(xnclock_core_ns_to_ticks);
EXPORT_SYMBOL_GPL(xnclock_divrem_billion);

DEFINE_PRIVATE_XNLOCK(ratelimit_lock);

int __xnclock_ratelimit(struct xnclock_ratelimit_state *rs, const char *func)
{
	spl_t s;
	int ret;

	if (!rs->interval)
		return 1;

	xnlock_get_irqsave(&ratelimit_lock, s);

	if (!rs->begin)
		rs->begin = xnclock_read_realtime(&nkclock);
	if (xnclock_read_realtime(&nkclock) >= rs->begin + rs->interval) {
		if (rs->missed)
			printk(KERN_WARNING "%s: %d callbacks suppressed\n",
			       func, rs->missed);
		rs->begin   = 0;
		rs->printed = 0;
		rs->missed  = 0;
	}
	if (rs->burst && rs->burst > rs->printed) {
		rs->printed++;
		ret = 1;
	} else {
		rs->missed++;
		ret = 0;
	}
	xnlock_put_irqrestore(&ratelimit_lock, s);

	return ret;
}
EXPORT_SYMBOL_GPL(__xnclock_ratelimit);

void xnclock_core_local_shot(struct xnsched *sched)
{
	struct xntimerdata *tmd;
	struct xntimer *timer;
	xnsticks_t delay;
	xntimerh_t *h;

	/*
	 * Do not reprogram locally when inside the tick handler -
	 * will be done on exit anyway. Also exit if there is no
	 * pending timer.
	 */
	if (sched->status & XNINTCK)
		return;

	/*
	 * Assume the core clock device always has percpu semantics in
	 * SMP.
	 */
	tmd = xnclock_this_timerdata(&nkclock);
	h = xntimerq_head(&tmd->q);
	if (h == NULL) {
		sched->lflags |= XNIDLE;
		return;
	}

	/*
	 * Here we try to defer the host tick heading the timer queue,
	 * so that it does not preempt a real-time activity uselessly,
	 * in two cases:
	 *
	 * 1) a rescheduling is pending for the current CPU. We may
	 * assume that a real-time thread is about to resume, so we
	 * want to move the host tick out of the way until the host
	 * kernel resumes, unless there is no other outstanding
	 * timers.
	 *
	 * 2) the current thread is running in primary mode, in which
	 * case we may also defer the host tick until the host kernel
	 * resumes.
	 *
	 * The host tick deferral is cleared whenever Xenomai is about
	 * to yield control to the host kernel (see ___xnsched_run()),
	 * or a timer with an earlier timeout date is scheduled,
	 * whichever comes first.
	 */
	sched->lflags &= ~(XNHDEFER|XNIDLE);
	timer = container_of(h, struct xntimer, aplink);
	if (unlikely(timer == &sched->htimer)) {
		if (xnsched_resched_p(sched) ||
		    !xnthread_test_state(sched->curr, XNROOT)) {
			h = xntimerq_second(&tmd->q, h);
			if (h) {
				sched->lflags |= XNHDEFER;
				timer = container_of(h, struct xntimer, aplink);
			}
		}
	}

	delay = xntimerh_date(&timer->aplink) - xnclock_core_read_raw();
	if (delay < 0)
		delay = 0;
	else if (delay > ULONG_MAX)
		delay = ULONG_MAX;

	xntrace_tick((unsigned)delay);

	ipipe_timer_set(delay);
}

#ifdef CONFIG_SMP
void xnclock_core_remote_shot(struct xnsched *sched)
{
	ipipe_send_ipi(IPIPE_HRTIMER_IPI, *cpumask_of(xnsched_cpu(sched)));
}
#endif

static void adjust_timer(struct xntimer *timer, xntimerq_t *q,
			 xnsticks_t delta)
{
	struct xnclock *clock = xntimer_clock(timer);
	xnticks_t period, div;
	xnsticks_t diff;

	xntimerh_date(&timer->aplink) -= delta;

	if (xntimer_periodic_p(timer) == 0)
		goto enqueue;

	timer->start_date -= delta;
	period = xntimer_interval(timer);
	diff = xnclock_ticks_to_ns(clock,
		xnclock_read_raw(clock) - xntimer_expiry(timer));

	if ((xnsticks_t)(diff - period) >= 0) {
		/*
		 * Timer should tick several times before now, instead
		 * of calling timer->handler several times, we change
		 * the timer date without changing its pexpect, so
		 * that timer will tick only once and the lost ticks
		 * will be counted as overruns.
		 */
		div = xnarch_div64(diff, period);
		timer->periodic_ticks += div;
		xntimer_update_date(timer);
	} else if (delta < 0
		   && (timer->status & XNTIMER_FIRED)
		   && (xnsticks_t) (diff + period) <= 0) {
		/*
		 * Timer is periodic and NOT waiting for its first
		 * shot, so we make it tick sooner than its original
		 * date in order to avoid the case where by adjusting
		 * time to a sooner date, real-time periodic timers do
		 * not tick until the original date has passed.
		 */
		div = xnarch_div64(-diff, period);
		timer->periodic_ticks -= div;
		timer->pexpect_ticks -= div;
		xntimer_update_date(timer);
	}

enqueue:
	xntimer_enqueue(timer, q);
}

static void adjust_clock_timers(struct xnclock *clock, xnsticks_t delta)
{
	struct xntimer *timer, *tmp;
	struct list_head adjq;
	struct xnsched *sched;
	xntimerq_it_t it;
	unsigned int cpu;
	xntimerh_t *h;
	xntimerq_t *q;

	INIT_LIST_HEAD(&adjq);
	delta = xnclock_ns_to_ticks(clock, delta);

	for_each_online_cpu(cpu) {
		sched = xnsched_struct(cpu);
		q = &xnclock_percpu_timerdata(clock, cpu)->q;

		for (h = xntimerq_it_begin(q, &it); h;
		     h = xntimerq_it_next(q, &it, h)) {
			timer = container_of(h, struct xntimer, aplink);
			if (timer->status & XNTIMER_REALTIME)
				list_add_tail(&timer->adjlink, &adjq);
		}

		if (list_empty(&adjq))
			continue;

		list_for_each_entry_safe(timer, tmp, &adjq, adjlink) {
			list_del(&timer->adjlink);
			xntimer_dequeue(timer, q);
			adjust_timer(timer, q, delta);
		}

		if (sched != xnsched_current())
			xnclock_remote_shot(clock, sched);
		else
			xnclock_program_shot(clock, sched);
	}
}

/**
 * @fn void xnclock_adjust(struct xnclock *clock, xnsticks_t delta)
 * @brief Adjust a clock time.
 *
 * This service changes the epoch for the given clock by applying the
 * specified tick delta on its wallclock offset.
 *
 * @param clock The clock to adjust.
 *
 * @param delta The adjustment value expressed in nanoseconds.
 *
 * @coretags{task-unrestricted, atomic-entry}
 *
 * @note Xenomai tracks the system time in @a nkclock, as a
 * monotonously increasing count of ticks since the epoch. The epoch
 * is initially the same as the underlying machine time.
 */
void xnclock_adjust(struct xnclock *clock, xnsticks_t delta)
{
	xnticks_t now;

	nkclock.wallclock_offset += delta;
	nkvdso->wallclock_offset = nkclock.wallclock_offset;
	now = xnclock_read_monotonic(clock) + nkclock.wallclock_offset;
	adjust_clock_timers(clock, delta);
}
EXPORT_SYMBOL_GPL(xnclock_adjust);

xnticks_t xnclock_get_host_time(void)
{
	return ktime_to_ns(ktime_get_real());
}
EXPORT_SYMBOL_GPL(xnclock_get_host_time);

xnticks_t xnclock_core_read_monotonic(void)
{
	return xnclock_core_ticks_to_ns(xnclock_core_read_raw());
}
EXPORT_SYMBOL_GPL(xnclock_core_read_monotonic);

#ifdef CONFIG_XENO_OPT_STATS

static struct xnvfile_directory timerlist_vfroot;

static struct xnvfile_snapshot_ops timerlist_ops;

struct vfile_clock_priv {
	struct xntimer *curr;
};

struct vfile_clock_data {
	int cpu;
	unsigned int scheduled;
	unsigned int fired;
	xnticks_t timeout;
	xnticks_t interval;
	unsigned long status;
	char name[XNOBJECT_NAME_LEN];
};

static int timerlist_rewind(struct xnvfile_snapshot_iterator *it)
{
	struct vfile_clock_priv *priv = xnvfile_iterator_priv(it);
	struct xnclock *clock = xnvfile_priv(it->vfile);

	if (list_empty(&clock->timerq))
		return -ESRCH;

	priv->curr = list_first_entry(&clock->timerq, struct xntimer, next_stat);

	return clock->nrtimers;
}

static int timerlist_next(struct xnvfile_snapshot_iterator *it, void *data)
{
	struct vfile_clock_priv *priv = xnvfile_iterator_priv(it);
	struct xnclock *clock = xnvfile_priv(it->vfile);
	struct vfile_clock_data *p = data;
	struct xntimer *timer;

	if (priv->curr == NULL)
		return 0;

	timer = priv->curr;
	if (list_is_last(&timer->next_stat, &clock->timerq))
		priv->curr = NULL;
	else
		priv->curr = list_entry(timer->next_stat.next,
					struct xntimer, next_stat);

	if (clock == &nkclock && xnstat_counter_get(&timer->scheduled) == 0)
		return VFILE_SEQ_SKIP;

	p->cpu = xnsched_cpu(xntimer_sched(timer));
	p->scheduled = xnstat_counter_get(&timer->scheduled);
	p->fired = xnstat_counter_get(&timer->fired);
	p->timeout = xntimer_get_timeout(timer);
	p->interval = xntimer_interval(timer);
	p->status = timer->status;
	knamecpy(p->name, timer->name);

	return 1;
}

static int timerlist_show(struct xnvfile_snapshot_iterator *it, void *data)
{
	struct vfile_clock_data *p = data;
	char timeout_buf[]  = "-         ";
	char interval_buf[] = "-         ";
	char hit_buf[32];

	if (p == NULL)
		xnvfile_printf(it,
			       "%-3s  %-20s  %-10s  %-10s  %s\n",
			       "CPU", "SCHED/SHOT", "TIMEOUT",
			       "INTERVAL", "NAME");
	else {
		if (p->status & XNTIMER_RUNNING)
			xntimer_format_time(p->timeout, timeout_buf,
					    sizeof(timeout_buf));
		if (p->status & XNTIMER_PERIODIC)
			xntimer_format_time(p->interval, interval_buf,
					    sizeof(interval_buf));
		ksformat(hit_buf, sizeof(hit_buf), "%u/%u",
			 p->scheduled, p->fired);
		xnvfile_printf(it,
			       "%-3u  %-20s  %-10s  %-10s  %s\n",
			       p->cpu, hit_buf, timeout_buf,
			       interval_buf, p->name);
	}

	return 0;
}

static struct xnvfile_snapshot_ops timerlist_ops = {
	.rewind = timerlist_rewind,
	.next = timerlist_next,
	.show = timerlist_show,
};

static void init_timerlist_proc(struct xnclock *clock)
{
	memset(&clock->timer_vfile, 0, sizeof(clock->timer_vfile));
	clock->timer_vfile.privsz = sizeof(struct vfile_clock_priv);
	clock->timer_vfile.datasz = sizeof(struct vfile_clock_data);
	clock->timer_vfile.tag = &clock->timer_revtag;
	clock->timer_vfile.ops = &timerlist_ops;

	xnvfile_init_snapshot(clock->name, &clock->timer_vfile, &timerlist_vfroot);
	xnvfile_priv(&clock->timer_vfile) = clock;
}

static void cleanup_timerlist_proc(struct xnclock *clock)
{
	xnvfile_destroy_snapshot(&clock->timer_vfile);
}

void init_timerlist_root(void)
{
	xnvfile_init_dir("timer", &timerlist_vfroot, &cobalt_vfroot);
}

void cleanup_timerlist_root(void)
{
	xnvfile_destroy_dir(&timerlist_vfroot);
}

#else  /* !CONFIG_XENO_OPT_STATS */

static inline void init_timerlist_root(void) { }

static inline void cleanup_timerlist_root(void) { }

static inline void init_timerlist_proc(struct xnclock *clock) { }

static inline void cleanup_timerlist_proc(struct xnclock *clock) { }

#endif	/* !CONFIG_XENO_OPT_STATS */

#ifdef CONFIG_XENO_OPT_VFILE

static struct xnvfile_directory clock_vfroot;

void print_core_clock_status(struct xnclock *clock,
			     struct xnvfile_regular_iterator *it)
{
	const char *wd_status = "off";

#ifdef CONFIG_XENO_OPT_WATCHDOG
	wd_status = "on";
#endif /* CONFIG_XENO_OPT_WATCHDOG */

	xnvfile_printf(it, "%8s: timer=%s, clock=%s\n",
		       "devices", ipipe_timer_name(), ipipe_clock_name());
	xnvfile_printf(it, "%8s: %s\n", "watchdog", wd_status);
	xnvfile_printf(it, "%8s: %Lu\n", "setup",
		       xnclock_ticks_to_ns(&nkclock, nktimerlat));
}

static int clock_show(struct xnvfile_regular_iterator *it, void *data)
{
	struct xnclock *clock = xnvfile_priv(it->vfile);
	xnticks_t now = xnclock_read_raw(clock);

	if (clock->id >= 0)	/* External clock, print id. */
		xnvfile_printf(it, "%7s: %d\n", "id", __COBALT_CLOCK_EXT(clock->id));
		
	xnvfile_printf(it, "%7s: irq=%Ld kernel=%Ld user=%Ld\n", "gravity",
		       xnclock_ticks_to_ns(clock, xnclock_get_gravity(clock, irq)),
		       xnclock_ticks_to_ns(clock, xnclock_get_gravity(clock, kernel)),
		       xnclock_ticks_to_ns(clock, xnclock_get_gravity(clock, user)));

	xnclock_print_status(clock, it);

	xnvfile_printf(it, "%7s: %Lu (%.4Lx %.4x)\n", "ticks",
		       now, now >> 32, (u32)(now & -1U));

	return 0;
}

static ssize_t clock_store(struct xnvfile_input *input)
{
	char buf[128], *args = buf, *p;
	struct xnclock_gravity gravity;
	struct xnvfile_regular *vfile;
	unsigned long ns, ticks;
	struct xnclock *clock;
	ssize_t nbytes;
	int ret;

	nbytes = xnvfile_get_string(input, buf, sizeof(buf));
	if (nbytes < 0)
		return nbytes;

	vfile = container_of(input->vfile, struct xnvfile_regular, entry);
	clock = xnvfile_priv(vfile);
	gravity = clock->gravity;

	while ((p = strsep(&args, " \t:/,")) != NULL) {
		if (*p == '\0')
			continue;
		ns = simple_strtol(p, &p, 10);
		ticks = xnclock_ns_to_ticks(clock, ns);
		switch (*p) {
		case 'i':
			gravity.irq = ticks;
			break;
		case 'k':
			gravity.kernel = ticks;
			break;
		case 'u':
		case '\0':
			gravity.user = ticks;
			break;
		default:
			return -EINVAL;
		}
		ret = xnclock_set_gravity(clock, &gravity);
		if (ret)
			return ret;
	}

	return nbytes;
}

static struct xnvfile_regular_ops clock_ops = {
	.show = clock_show,
	.store = clock_store,
};

static void init_clock_proc(struct xnclock *clock)
{
	memset(&clock->vfile, 0, sizeof(clock->vfile));
	clock->vfile.ops = &clock_ops;
	xnvfile_init_regular(clock->name, &clock->vfile, &clock_vfroot);
	xnvfile_priv(&clock->vfile) = clock;
	init_timerlist_proc(clock);
}

static void cleanup_clock_proc(struct xnclock *clock)
{
	cleanup_timerlist_proc(clock);
	xnvfile_destroy_regular(&clock->vfile);
}

void xnclock_init_proc(void)
{
	xnvfile_init_dir("clock", &clock_vfroot, &cobalt_vfroot);
	init_timerlist_root();
}

void xnclock_cleanup_proc(void)
{
	xnvfile_destroy_dir(&clock_vfroot);
	cleanup_timerlist_root();
}

#else /* !CONFIG_XENO_OPT_VFILE */

static inline void init_clock_proc(struct xnclock *clock) { }

static inline void cleanup_clock_proc(struct xnclock *clock) { }

#endif	/* !CONFIG_XENO_OPT_VFILE */

/**
 * @brief Register a Xenomai clock.
 *
 * This service installs a new clock which may be used to drive
 * Xenomai timers.
 *
 * @param clock The new clock to register.
 *
 * @param affinity The set of CPUs we may expect the backing clock
 * device to tick on. As a special case, passing a NULL affinity mask
 * means that timer IRQs cannot be seen as percpu events, in which
 * case all outstanding timers will be maintained into a single global
 * queue instead of percpu timer queues.
 *
 * @coretags{secondary-only}
 */
int xnclock_register(struct xnclock *clock, const cpumask_t *affinity)
{
	struct xntimerdata *tmd;
	int cpu;

	secondary_mode_only();

#ifdef CONFIG_SMP
	/*
	 * A CPU affinity set may be defined for each clock,
	 * enumerating the CPUs which can receive ticks from the
	 * backing clock device.  When given, this set must be a
	 * subset of the real-time CPU set.
	 */
	if (affinity) {
		cpumask_and(&clock->affinity, affinity, &xnsched_realtime_cpus);
		if (cpumask_empty(&clock->affinity))
			return -EINVAL;
	} else	/* Device is global without particular IRQ affinity. */
		cpumask_clear(&clock->affinity);
#endif

	/* Allocate the percpu timer queue slot. */
	clock->timerdata = alloc_percpu(struct xntimerdata);
	if (clock->timerdata == NULL)
		return -ENOMEM;

	/*
	 * POLA: init all timer slots for the new clock, although some
	 * of them might remain unused depending on the CPU affinity
	 * of the event source(s). If the clock device is global
	 * without any particular IRQ affinity, all timers will be
	 * queued to CPU0.
	 */
	for_each_online_cpu(cpu) {
		tmd = xnclock_percpu_timerdata(clock, cpu);
		xntimerq_init(&tmd->q);
	}

#ifdef CONFIG_XENO_OPT_STATS
	INIT_LIST_HEAD(&clock->timerq);
#endif /* CONFIG_XENO_OPT_STATS */

	init_clock_proc(clock);

	return 0;
}
EXPORT_SYMBOL_GPL(xnclock_register);

/**
 * @fn void xnclock_deregister(struct xnclock *clock)
 * @brief Deregister a Xenomai clock.
 *
 * This service uninstalls a Xenomai clock previously registered with
 * xnclock_register().
 *
 * This service may be called once all timers driven by @a clock have
 * been stopped.
 *
 * @param clock The clock to deregister.
 *
 * @coretags{secondary-only}
 */
void xnclock_deregister(struct xnclock *clock)
{
	struct xntimerdata *tmd;
	int cpu;

	secondary_mode_only();

	cleanup_clock_proc(clock);

	for_each_online_cpu(cpu) {
		tmd = xnclock_percpu_timerdata(clock, cpu);
		XENO_BUG_ON(COBALT, !xntimerq_empty(&tmd->q));
		xntimerq_destroy(&tmd->q);
	}

	free_percpu(clock->timerdata);
}
EXPORT_SYMBOL_GPL(xnclock_deregister);

/**
 * @fn void xnclock_tick(struct xnclock *clock)
 * @brief Process a clock tick.
 *
 * This routine processes an incoming @a clock event, firing elapsed
 * timers as appropriate.
 *
 * @param clock The clock for which a new event was received.
 *
 * @coretags{coreirq-only, atomic-entry}
 *
 * @note The current CPU must be part of the real-time affinity set
 * unless the clock device has no particular IRQ affinity, otherwise
 * weird things may happen.
 */
void xnclock_tick(struct xnclock *clock)
{
	struct xnsched *sched = xnsched_current();
	struct xntimer *timer;
	xnsticks_t delta;
	xntimerq_t *tmq;
	xnticks_t now;
	xntimerh_t *h;

	atomic_only();

#ifdef CONFIG_SMP
	/*
	 * Some external clock devices may be global without any
	 * particular IRQ affinity, in which case the associated
	 * timers will be queued to CPU0.
	 */
	if (IS_ENABLED(CONFIG_XENO_OPT_EXTCLOCK) &&
	    clock != &nkclock &&
	    !cpumask_test_cpu(xnsched_cpu(sched), &clock->affinity))
		tmq = &xnclock_percpu_timerdata(clock, 0)->q;
	else
#endif
		tmq = &xnclock_this_timerdata(clock)->q;
	
	/*
	 * Optimisation: any local timer reprogramming triggered by
	 * invoked timer handlers can wait until we leave the tick
	 * handler. Use this status flag as hint to xntimer_start().
	 */
	sched->status |= XNINTCK;

	now = xnclock_read_raw(clock);
	while ((h = xntimerq_head(tmq)) != NULL) {
		timer = container_of(h, struct xntimer, aplink);
		delta = (xnsticks_t)(xntimerh_date(&timer->aplink) - now);
		if (delta > 0)
			break;

		trace_cobalt_timer_expire(timer);

		xntimer_dequeue(timer, tmq);
		xntimer_account_fired(timer);

		/*
		 * By postponing the propagation of the low-priority
		 * host tick to the interrupt epilogue (see
		 * xnintr_irq_handler()), we save some I-cache, which
		 * translates into precious microsecs on low-end hw.
		 */
		if (unlikely(timer == &sched->htimer)) {
			sched->lflags |= XNHTICK;
			sched->lflags &= ~XNHDEFER;
			if (timer->status & XNTIMER_PERIODIC)
				goto advance;
			continue;
		}

		timer->handler(timer);
		now = xnclock_read_raw(clock);
		timer->status |= XNTIMER_FIRED;
		/*
		 * Only requeue periodic timers which have not been
		 * requeued, stopped or killed.
		 */
		if ((timer->status &
		     (XNTIMER_PERIODIC|XNTIMER_DEQUEUED|XNTIMER_KILLED|XNTIMER_RUNNING)) !=
		    (XNTIMER_PERIODIC|XNTIMER_DEQUEUED|XNTIMER_RUNNING))
			continue;
	advance:
		do {
			timer->periodic_ticks++;
			xntimer_update_date(timer);
		} while (xntimerh_date(&timer->aplink) < now);

#ifdef CONFIG_SMP
		/*
		 * If the timer was migrated over its timeout handler,
		 * xntimer_migrate() re-queued it already.
		 */
		if (unlikely(timer->sched != sched))
			continue;
#endif
		xntimer_enqueue(timer, tmq);
	}

	sched->status &= ~XNINTCK;

	xnclock_program_shot(clock, sched);
}
EXPORT_SYMBOL_GPL(xnclock_tick);

void xnclock_update_freq(unsigned long long freq)
{
	spl_t s;

	xnlock_get_irqsave(&nklock, s);
	clockfreq = freq;
#ifdef XNARCH_HAVE_LLMULSHFT
	xnarch_init_llmulshft(1000000000, freq, &tsc_scale, &tsc_shift);
#ifdef XNARCH_HAVE_NODIV_LLIMD
	xnarch_init_u32frac(&tsc_frac, 1 << tsc_shift, tsc_scale);
	xnarch_init_u32frac(&bln_frac, 1, 1000000000);
#endif
#endif
	cobalt_pipeline.clock_freq = freq;
	xnlock_put_irqrestore(&nklock, s);
}

static int set_core_clock_gravity(struct xnclock *clock,
				  const struct xnclock_gravity *p)
{
	nkclock.gravity = *p;

	return 0;
}

static void reset_core_clock_gravity(struct xnclock *clock)
{
	struct xnclock_gravity gravity;

	xnarch_get_latencies(&gravity);
	gravity.user += nktimerlat;
	if (gravity.kernel == 0)
		gravity.kernel = gravity.user;
	if (gravity.irq == 0)
		gravity.irq = nktimerlat;
	set_core_clock_gravity(clock, &gravity);
}

struct xnclock nkclock = {
	.name = "coreclk",
	.resolution = 1,	/* nanosecond. */
	.ops = {
		.set_gravity = set_core_clock_gravity,
		.reset_gravity = reset_core_clock_gravity,
#ifdef CONFIG_XENO_OPT_VFILE
		.print_status = print_core_clock_status,
#endif
	},
	.id = -1,
};
EXPORT_SYMBOL_GPL(nkclock);

void xnclock_cleanup(void)
{
	xnclock_deregister(&nkclock);
}

int __init xnclock_init(unsigned long long freq)
{
	xnclock_update_freq(freq);
	nktimerlat = xnarch_timer_calibrate();
	xnclock_reset_gravity(&nkclock);
	xnclock_register(&nkclock, &xnsched_realtime_cpus);

	return 0;
}

/** @} */
