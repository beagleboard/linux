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

#ifndef _COBALT_KERNEL_TIMER_H
#define _COBALT_KERNEL_TIMER_H

#include <cobalt/kernel/clock.h>
#include <cobalt/kernel/stat.h>
#include <cobalt/kernel/list.h>
#include <cobalt/kernel/assert.h>
#include <cobalt/kernel/ancillaries.h>
#include <asm/xenomai/wrappers.h>

/**
 * @addtogroup cobalt_core_timer
 * @{
 */
#define XN_INFINITE   ((xnticks_t)0)
#define XN_NONBLOCK   ((xnticks_t)-1)

/* Timer modes */
typedef enum xntmode {
	XN_RELATIVE,
	XN_ABSOLUTE,
	XN_REALTIME
} xntmode_t;

/* Timer status */
#define XNTIMER_DEQUEUED  0x00000001
#define XNTIMER_KILLED    0x00000002
#define XNTIMER_PERIODIC  0x00000004
#define XNTIMER_REALTIME  0x00000008
#define XNTIMER_FIRED     0x00000010
#define XNTIMER_RUNNING   0x00000020
#define XNTIMER_KGRAVITY  0x00000040
#define XNTIMER_UGRAVITY  0x00000080
#define XNTIMER_IGRAVITY  0	     /* most conservative */

#define XNTIMER_GRAVITY_MASK	(XNTIMER_KGRAVITY|XNTIMER_UGRAVITY)
#define XNTIMER_INIT_MASK	XNTIMER_GRAVITY_MASK

/* These flags are available to the real-time interfaces */
#define XNTIMER_SPARE0  0x01000000
#define XNTIMER_SPARE1  0x02000000
#define XNTIMER_SPARE2  0x04000000
#define XNTIMER_SPARE3  0x08000000
#define XNTIMER_SPARE4  0x10000000
#define XNTIMER_SPARE5  0x20000000
#define XNTIMER_SPARE6  0x40000000
#define XNTIMER_SPARE7  0x80000000

/* Timer priorities */
#define XNTIMER_LOPRIO  (-999999999)
#define XNTIMER_STDPRIO 0
#define XNTIMER_HIPRIO  999999999

struct xntlholder {
	struct list_head link;
	xnticks_t key;
	int prio;
};

#define xntlholder_date(h)	((h)->key)
#define xntlholder_prio(h)	((h)->prio)
#define xntlist_init(q)		INIT_LIST_HEAD(q)
#define xntlist_empty(q)	list_empty(q)

static inline struct xntlholder *xntlist_head(struct list_head *q)
{
	if (list_empty(q))
		return NULL;

	return list_first_entry(q, struct xntlholder, link);
}

static inline struct xntlholder *xntlist_next(struct list_head *q,
					      struct xntlholder *h)
{
	if (list_is_last(&h->link, q))
		return NULL;

	return list_entry(h->link.next, struct xntlholder, link);
}

static inline struct xntlholder *xntlist_second(struct list_head *q,
	struct xntlholder *h)
{
	return xntlist_next(q, h);
}

static inline void xntlist_insert(struct list_head *q, struct xntlholder *holder)
{
	struct xntlholder *p;

	if (list_empty(q)) {
		list_add(&holder->link, q);
		return;
	}

	/*
	 * Insert the new timer at the proper place in the single
	 * queue. O(N) here, but this is the price for the increased
	 * flexibility...
	 */
	list_for_each_entry_reverse(p, q, link) {
		if ((xnsticks_t) (holder->key - p->key) > 0 ||
		    (holder->key == p->key && holder->prio <= p->prio))
		  break;
	}

	list_add(&holder->link, &p->link);
}

#define xntlist_remove(q, h)			\
	do {					\
		(void)(q);			\
		list_del(&(h)->link);		\
	} while (0)

#if defined(CONFIG_XENO_OPT_TIMER_RBTREE)

#include <linux/rbtree.h>

typedef struct {
	unsigned long long date;
	unsigned prio;
	struct rb_node link;
} xntimerh_t;

#define xntimerh_date(h) ((h)->date)
#define xntimerh_prio(h) ((h)->prio)
#define xntimerh_init(h) do { } while (0)

typedef struct {
	struct rb_root root;
	xntimerh_t *head;
} xntimerq_t;

#define xntimerq_init(q)			\
	({					\
		xntimerq_t *_q = (q);		\
		_q->root = RB_ROOT;		\
		_q->head = NULL;		\
	})

#define xntimerq_destroy(q) do { } while (0)
#define xntimerq_empty(q) ((q)->head == NULL)

#define xntimerq_head(q) ((q)->head)

#define xntimerq_next(q, h)						\
	({								\
		struct rb_node *_node = rb_next(&(h)->link);		\
		_node ? (container_of(_node, xntimerh_t, link)) : NULL; \
	})

#define xntimerq_second(q, h) xntimerq_next(q, h)

void xntimerq_insert(xntimerq_t *q, xntimerh_t *holder);

static inline void xntimerq_remove(xntimerq_t *q, xntimerh_t *holder)
{
	if (holder == q->head)
		q->head = xntimerq_second(q, holder);

	rb_erase(&holder->link, &q->root);
}

typedef struct { } xntimerq_it_t;

#define xntimerq_it_begin(q,i)	((void) (i), xntimerq_head(q))
#define xntimerq_it_next(q,i,h) ((void) (i), xntimerq_next((q),(h)))

#else /* CONFIG_XENO_OPT_TIMER_LIST */

typedef struct xntlholder xntimerh_t;

#define xntimerh_date(h)       xntlholder_date(h)
#define xntimerh_prio(h)       xntlholder_prio(h)
#define xntimerh_init(h)       do { } while (0)

typedef struct list_head xntimerq_t;

#define xntimerq_init(q)        xntlist_init(q)
#define xntimerq_destroy(q)     do { } while (0)
#define xntimerq_empty(q)       xntlist_empty(q)
#define xntimerq_head(q)        xntlist_head(q)
#define xntimerq_second(q, h)   xntlist_second((q),(h))
#define xntimerq_insert(q, h)   xntlist_insert((q),(h))
#define xntimerq_remove(q, h)   xntlist_remove((q),(h))

typedef struct { } xntimerq_it_t;

#define xntimerq_it_begin(q,i)  ((void) (i), xntlist_head(q))
#define xntimerq_it_next(q,i,h) ((void) (i), xntlist_next((q),(h)))

#endif /* CONFIG_XENO_OPT_TIMER_LIST */

struct xnsched;

struct xntimerdata {
	xntimerq_t q;
};

static inline struct xntimerdata *
xnclock_percpu_timerdata(struct xnclock *clock, int cpu)
{
	return per_cpu_ptr(clock->timerdata, cpu);
}

static inline struct xntimerdata *
xnclock_this_timerdata(struct xnclock *clock)
{
	return raw_cpu_ptr(clock->timerdata);
}

struct xntimer {
#ifdef CONFIG_XENO_OPT_EXTCLOCK
	struct xnclock *clock;
#endif
	/** Link in timers list. */
	xntimerh_t aplink;
	struct list_head adjlink;
	/** Timer status. */
	unsigned long status;
	/** Periodic interval (clock ticks, 0 == one shot). */
	xnticks_t interval;
	/** Periodic interval (nanoseconds, 0 == one shot). */
	xnticks_t interval_ns;
	/** Count of timer ticks in periodic mode. */
	xnticks_t periodic_ticks;
	/** First tick date in periodic mode. */
	xnticks_t start_date;
	/** Date of next periodic release point (timer ticks). */
	xnticks_t pexpect_ticks;
	/** Sched structure to which the timer is attached. */
	struct xnsched *sched;
	/** Timeout handler. */
	void (*handler)(struct xntimer *timer);
#ifdef CONFIG_XENO_OPT_STATS
#ifdef CONFIG_XENO_OPT_EXTCLOCK
	struct xnclock *tracker;
#endif
	/** Timer name to be displayed. */
	char name[XNOBJECT_NAME_LEN];
	/** Timer holder in timebase. */
	struct list_head next_stat;
	/** Number of timer schedules. */
	xnstat_counter_t scheduled;
	/** Number of timer events. */
	xnstat_counter_t fired;
#endif /* CONFIG_XENO_OPT_STATS */
};

#ifdef CONFIG_XENO_OPT_EXTCLOCK

static inline struct xnclock *xntimer_clock(struct xntimer *timer)
{
	return timer->clock;
}

void xntimer_set_clock(struct xntimer *timer,
		       struct xnclock *newclock);

#else /* !CONFIG_XENO_OPT_EXTCLOCK */

static inline struct xnclock *xntimer_clock(struct xntimer *timer)
{
	return &nkclock;
}

static inline void xntimer_set_clock(struct xntimer *timer,
				     struct xnclock *newclock)
{
	XENO_BUG_ON(COBALT, newclock != &nkclock);
}

#endif /* !CONFIG_XENO_OPT_EXTCLOCK */

#ifdef CONFIG_SMP
static inline struct xnsched *xntimer_sched(struct xntimer *timer)
{
	return timer->sched;
}
#else /* !CONFIG_SMP */
#define xntimer_sched(t)	xnsched_current()
#endif /* !CONFIG_SMP */

#define xntimer_percpu_queue(__timer)					\
	({								\
		struct xntimerdata *tmd;				\
		int cpu = xnsched_cpu((__timer)->sched);		\
		tmd = xnclock_percpu_timerdata(xntimer_clock(__timer), cpu); \
		&tmd->q;						\
	})

static inline unsigned long xntimer_gravity(struct xntimer *timer)
{
	struct xnclock *clock = xntimer_clock(timer);

	if (timer->status & XNTIMER_KGRAVITY)
		return clock->gravity.kernel;

	if (timer->status & XNTIMER_UGRAVITY)
		return clock->gravity.user;

	return clock->gravity.irq;
}

static inline void xntimer_update_date(struct xntimer *timer)
{
	xntimerh_date(&timer->aplink) = timer->start_date
		+ xnclock_ns_to_ticks(xntimer_clock(timer),
			timer->periodic_ticks * timer->interval_ns)
		- xntimer_gravity(timer);
}

static inline xnticks_t xntimer_pexpect(struct xntimer *timer)
{
	return timer->start_date +
		xnclock_ns_to_ticks(xntimer_clock(timer),
				timer->pexpect_ticks * timer->interval_ns);
}

static inline void xntimer_set_priority(struct xntimer *timer,
					int prio)
{
	xntimerh_prio(&timer->aplink) = prio;
}

static inline int xntimer_active_p(struct xntimer *timer)
{
	return timer->sched != NULL;
}

static inline int xntimer_running_p(struct xntimer *timer)
{
	return (timer->status & XNTIMER_RUNNING) != 0;
}

static inline int xntimer_fired_p(struct xntimer *timer)
{
	return (timer->status & XNTIMER_FIRED) != 0;
}

static inline int xntimer_periodic_p(struct xntimer *timer)
{
	return (timer->status & XNTIMER_PERIODIC) != 0;
}

void __xntimer_init(struct xntimer *timer,
		    struct xnclock *clock,
		    void (*handler)(struct xntimer *timer),
		    struct xnsched *sched,
		    int flags);

void xntimer_set_gravity(struct xntimer *timer,
			 int gravity);

#ifdef CONFIG_XENO_OPT_STATS

#define xntimer_init(__timer, __clock, __handler, __sched, __flags)	\
do {									\
	__xntimer_init(__timer, __clock, __handler, __sched, __flags);	\
	xntimer_set_name(__timer, #__handler);				\
} while (0)

static inline void xntimer_reset_stats(struct xntimer *timer)
{
	xnstat_counter_set(&timer->scheduled, 0);
	xnstat_counter_set(&timer->fired, 0);
}

static inline void xntimer_account_scheduled(struct xntimer *timer)
{
	xnstat_counter_inc(&timer->scheduled);
}

static inline void xntimer_account_fired(struct xntimer *timer)
{
	xnstat_counter_inc(&timer->fired);
}

static inline void xntimer_set_name(struct xntimer *timer, const char *name)
{
	knamecpy(timer->name, name);
}

#else /* !CONFIG_XENO_OPT_STATS */

#define xntimer_init	__xntimer_init

static inline void xntimer_reset_stats(struct xntimer *timer) { }

static inline void xntimer_account_scheduled(struct xntimer *timer) { }

static inline void xntimer_account_fired(struct xntimer *timer) { }

static inline void xntimer_set_name(struct xntimer *timer, const char *name) { }

#endif /* !CONFIG_XENO_OPT_STATS */

#if defined(CONFIG_XENO_OPT_EXTCLOCK) && defined(CONFIG_XENO_OPT_STATS)
void xntimer_switch_tracking(struct xntimer *timer,
			     struct xnclock *newclock);
#else
static inline
void xntimer_switch_tracking(struct xntimer *timer,
			     struct xnclock *newclock) { }
#endif

void xntimer_destroy(struct xntimer *timer);

/**
 * @fn xnticks_t xntimer_interval(struct xntimer *timer)
 *
 * @brief Return the timer interval value.
 *
 * Return the timer interval value in nanoseconds.
 *
 * @param timer The address of a valid timer descriptor.
 *
 * @return The duration of a period in nanoseconds. The special value
 * XN_INFINITE is returned if @a timer is currently disabled or
 * one shot.
 *
 * @coretags{unrestricted, atomic-entry}
 */
static inline xnticks_t xntimer_interval(struct xntimer *timer)
{
	return timer->interval_ns;
}

static inline xnticks_t xntimer_expiry(struct xntimer *timer)
{
	/* Real expiry date in ticks without anticipation (no gravity) */
	return xntimerh_date(&timer->aplink) + xntimer_gravity(timer);
}

int xntimer_start(struct xntimer *timer,
		xnticks_t value,
		xnticks_t interval,
		xntmode_t mode);

void __xntimer_stop(struct xntimer *timer);

xnticks_t xntimer_get_date(struct xntimer *timer);

xnticks_t __xntimer_get_timeout(struct xntimer *timer);

xnticks_t xntimer_get_interval(struct xntimer *timer);

int xntimer_heading_p(struct xntimer *timer);

static inline void xntimer_stop(struct xntimer *timer)
{
	if (timer->status & XNTIMER_RUNNING)
		__xntimer_stop(timer);
}

static inline xnticks_t xntimer_get_timeout(struct xntimer *timer)
{
	if (!xntimer_running_p(timer))
		return XN_INFINITE;

	return __xntimer_get_timeout(timer);
}

static inline xnticks_t xntimer_get_timeout_stopped(struct xntimer *timer)
{
	return __xntimer_get_timeout(timer);
}

static inline void xntimer_enqueue(struct xntimer *timer,
				   xntimerq_t *q)
{
	xntimerq_insert(q, &timer->aplink);
	timer->status &= ~XNTIMER_DEQUEUED;
	xntimer_account_scheduled(timer);
}

static inline void xntimer_dequeue(struct xntimer *timer,
				   xntimerq_t *q)
{
	xntimerq_remove(q, &timer->aplink);
	timer->status |= XNTIMER_DEQUEUED;
}

unsigned long long xntimer_get_overruns(struct xntimer *timer,
					struct xnthread *waiter,
					xnticks_t now);

#ifdef CONFIG_SMP

void __xntimer_migrate(struct xntimer *timer, struct xnsched *sched);

static inline
void xntimer_migrate(struct xntimer *timer, struct xnsched *sched)
{				/* nklocked, IRQs off */
	if (timer->sched != sched)
		__xntimer_migrate(timer, sched);
}

int xntimer_setup_ipi(void);

void xntimer_release_ipi(void);

void __xntimer_set_affinity(struct xntimer *timer,
			    struct xnsched *sched);

static inline void xntimer_set_affinity(struct xntimer *timer,
					struct xnsched *sched)
{
	if (sched != xntimer_sched(timer))
		__xntimer_set_affinity(timer, sched);
}

#else /* ! CONFIG_SMP */

static inline void xntimer_migrate(struct xntimer *timer,
				   struct xnsched *sched)
{
	timer->sched = sched;
}

static inline int xntimer_setup_ipi(void)
{
	return 0;
}

static inline void xntimer_release_ipi(void) { }

static inline void xntimer_set_affinity(struct xntimer *timer,
					struct xnsched *sched)
{
	xntimer_migrate(timer, sched);
}

#endif /* CONFIG_SMP */

char *xntimer_format_time(xnticks_t ns,
			  char *buf, size_t bufsz);

int xntimer_grab_hardware(void);

void xntimer_release_hardware(void);

/** @} */

#endif /* !_COBALT_KERNEL_TIMER_H */
