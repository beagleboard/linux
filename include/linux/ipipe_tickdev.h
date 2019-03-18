/* -*- linux-c -*-
 * include/linux/ipipe_tickdev.h
 *
 * Copyright (C) 2007 Philippe Gerum.
 * Copyright (C) 2012 Gilles Chanteperdrix
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, Inc., 675 Mass Ave, Cambridge MA 02139,
 * USA; either version 2 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __LINUX_IPIPE_TICKDEV_H
#define __LINUX_IPIPE_TICKDEV_H

#include <linux/list.h>
#include <linux/cpumask.h>
#include <linux/clockchips.h>
#include <linux/ipipe_domain.h>
#include <linux/clocksource.h>
#include <linux/timekeeper_internal.h>

#ifdef CONFIG_IPIPE

struct clock_event_device;

struct ipipe_hostrt_data {
	short live;
	seqcount_t seqcount;
	time_t wall_time_sec;
	u32 wall_time_nsec;
	struct timespec wall_to_monotonic;
	cycle_t cycle_last;
	cycle_t mask;
	u32 mult;
	u32 shift;
};

enum clock_event_mode {
	CLOCK_EVT_MODE_PERIODIC,
	CLOCK_EVT_MODE_ONESHOT,
	CLOCK_EVT_MODE_UNUSED,
	CLOCK_EVT_MODE_SHUTDOWN,
};

struct ipipe_timer {
	int irq;
	void (*request)(struct ipipe_timer *timer, int steal);
	int (*set)(unsigned long ticks, void *timer);
	void (*ack)(void);
	void (*release)(struct ipipe_timer *timer);

	/* Only if registering a timer directly */
	const char *name;
	unsigned rating;
	unsigned long freq;
	unsigned min_delay_ticks;
	const struct cpumask *cpumask;

	/* For internal use */
	void *timer_set;	/* pointer passed to ->set() callback */
	struct clock_event_device *host_timer;
	struct list_head link;

	/* Conversions between clock frequency and timer frequency */
	unsigned c2t_integ;
	unsigned c2t_frac;

	/* For clockevent interception */
	u32 real_mult;
	u32 real_shift;
	void (*mode_handler)(enum clock_event_mode mode,
			     struct clock_event_device *);
	int orig_mode;
	int (*orig_set_state_periodic)(struct clock_event_device *);
	int (*orig_set_state_oneshot)(struct clock_event_device *);
	int (*orig_set_state_oneshot_stopped)(struct clock_event_device *);
	int (*orig_set_state_shutdown)(struct clock_event_device *);
	int (*orig_set_next_event)(unsigned long evt,
				   struct clock_event_device *cdev);
	unsigned int (*refresh_freq)(void);
};

#define __ipipe_hrtimer_irq __ipipe_raw_cpu_read(ipipe_percpu.hrtimer_irq)

extern unsigned long __ipipe_hrtimer_freq;

/*
 * Called by clockevents_register_device, to register a piggybacked
 * ipipe timer, if there is one
 */
void ipipe_host_timer_register(struct clock_event_device *clkevt);

/*
 * Register a standalone ipipe timer
 */
void ipipe_timer_register(struct ipipe_timer *timer);

/*
 * Chooses the best timer for each cpu. Take over its handling.
 */
int ipipe_select_timers(const struct cpumask *mask);

/*
 * Release the per-cpu timers
 */
void ipipe_timers_release(void);

/*
 * Start handling the per-cpu timer irq, and intercepting the linux clockevent
 * device callbacks.
 */
int ipipe_timer_start(void (*tick_handler)(void),
		      void (*emumode)(enum clock_event_mode mode,
				      struct clock_event_device *cdev),
		      int (*emutick)(unsigned long evt,
				     struct clock_event_device *cdev),
		      unsigned cpu);

/*
 * Stop handling a per-cpu timer
 */
void ipipe_timer_stop(unsigned cpu);

/*
 * Program the timer
 */
void ipipe_timer_set(unsigned long delay);

const char *ipipe_timer_name(void);

unsigned ipipe_timer_ns2ticks(struct ipipe_timer *timer, unsigned ns);

void __ipipe_timer_refresh_freq(unsigned int hrclock_freq);

#else /* !CONFIG_IPIPE */

#define ipipe_host_timer_register(clkevt) do { } while (0)

#endif /* !CONFIG_IPIPE */

#ifdef CONFIG_IPIPE_HAVE_HOSTRT
void ipipe_update_hostrt(struct timekeeper *tk);
#else
static inline void
ipipe_update_hostrt(struct timekeeper *tk) {}
#endif

#endif /* __LINUX_IPIPE_TICKDEV_H */
