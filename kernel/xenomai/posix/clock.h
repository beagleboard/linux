/*
 * Written by Gilles Chanteperdrix <gilles.chanteperdrix@xenomai.org>.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#ifndef _COBALT_POSIX_CLOCK_H
#define _COBALT_POSIX_CLOCK_H

#include <linux/types.h>
#include <linux/time.h>
#include <linux/cpumask.h>
#include <cobalt/uapi/time.h>
#include <xenomai/posix/syscall.h>

#define ONE_BILLION             1000000000

struct xnclock;

static inline void ns2ts(struct timespec *ts, xnticks_t nsecs)
{
	ts->tv_sec = xnclock_divrem_billion(nsecs, &ts->tv_nsec);
}

static inline xnticks_t ts2ns(const struct timespec *ts)
{
	xnticks_t nsecs = ts->tv_nsec;

	if (ts->tv_sec)
		nsecs += (xnticks_t)ts->tv_sec * ONE_BILLION;

	return nsecs;
}

static inline xnticks_t tv2ns(const struct timeval *tv)
{
	xnticks_t nsecs = tv->tv_usec * 1000;

	if (tv->tv_sec)
		nsecs += (xnticks_t)tv->tv_sec * ONE_BILLION;

	return nsecs;
}

static inline void ticks2tv(struct timeval *tv, xnticks_t ticks)
{
	unsigned long nsecs;

	tv->tv_sec = xnclock_divrem_billion(ticks, &nsecs);
	tv->tv_usec = nsecs / 1000;
}

static inline xnticks_t clock_get_ticks(clockid_t clock_id)
{
	return clock_id == CLOCK_REALTIME ?
		xnclock_read_realtime(&nkclock) :
		xnclock_read_monotonic(&nkclock);
}

static inline int clock_flag(int flag, clockid_t clock_id)
{
	if ((flag & TIMER_ABSTIME) == 0)
		return XN_RELATIVE;

	if (clock_id == CLOCK_REALTIME)
		return XN_REALTIME;

	return XN_ABSOLUTE;
}

int __cobalt_clock_getres(clockid_t clock_id,
			  struct timespec *ts);

int __cobalt_clock_gettime(clockid_t clock_id,
			   struct timespec *ts);

int __cobalt_clock_settime(clockid_t clock_id,
			   const struct timespec *ts);

int __cobalt_clock_adjtime(clockid_t clock_id,
			   struct timex *tx);

int __cobalt_clock_nanosleep(clockid_t clock_id, int flags,
			     const struct timespec *rqt,
			     struct timespec *rmt);

COBALT_SYSCALL_DECL(clock_getres,
		    (clockid_t clock_id, struct timespec __user *u_ts));

COBALT_SYSCALL_DECL(clock_gettime,
		    (clockid_t clock_id, struct timespec __user *u_ts));

COBALT_SYSCALL_DECL(clock_settime,
		    (clockid_t clock_id, const struct timespec __user *u_ts));

COBALT_SYSCALL_DECL(clock_adjtime,
		    (clockid_t clock_id, struct timex __user *u_tx));

COBALT_SYSCALL_DECL(clock_nanosleep,
		    (clockid_t clock_id, int flags,
		     const struct timespec __user *u_rqt,
		     struct timespec __user *u_rmt));

int cobalt_clock_register(struct xnclock *clock,
			  const cpumask_t *affinity,
			  clockid_t *clk_id);

void cobalt_clock_deregister(struct xnclock *clock);

struct xnclock *cobalt_clock_find(clockid_t clock_id);

extern DECLARE_BITMAP(cobalt_clock_extids, COBALT_MAX_EXTCLOCKS);

#endif /* !_COBALT_POSIX_CLOCK_H */
