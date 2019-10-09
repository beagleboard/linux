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

#include <linux/clocksource.h>
#include <linux/bitmap.h>
#include <cobalt/kernel/vdso.h>
#include <cobalt/kernel/clock.h>
#include "internal.h"
#include "thread.h"
#include "clock.h"
#include <trace/events/cobalt-posix.h>

static struct xnclock *external_clocks[COBALT_MAX_EXTCLOCKS];

DECLARE_BITMAP(cobalt_clock_extids, COBALT_MAX_EXTCLOCKS);

static int do_clock_host_realtime(struct timespec *tp)
{
#ifdef CONFIG_XENO_OPT_HOSTRT
	struct xnvdso_hostrt_data *hostrt_data;
	u64 now, base, mask, cycle_delta;
	__u32 mult, shift;
	unsigned long rem;
	urwstate_t tmp;
	__u64 nsec;

	hostrt_data = get_hostrt_data();
	BUG_ON(!hostrt_data);

	if (unlikely(!hostrt_data->live))
		return -1;

	/*
	 * Note: Disabling HW interrupts around writes to hostrt_data
	 * ensures that a reader (on the Xenomai side) cannot
	 * interrupt a writer (on the Linux kernel side) on the same
	 * CPU.  The urw block is required when a reader is
	 * interleaved by a writer on a different CPU. This follows
	 * the approach from userland, where taking the spinlock is
	 * not possible.
	 */
	unsynced_read_block(&tmp, &hostrt_data->lock) {
		now = xnclock_read_raw(&nkclock);
		base = hostrt_data->cycle_last;
		mask = hostrt_data->mask;
		mult = hostrt_data->mult;
		shift = hostrt_data->shift;
		tp->tv_sec = hostrt_data->wall_sec;
		nsec = hostrt_data->wall_nsec;
	}

	/*
	 * At this point, we have a consistent copy of the fundamental
	 * data structure - calculate the interval between the current
	 * and base time stamp cycles, and convert the difference
	 * to nanoseconds.
	 */
	cycle_delta = (now - base) & mask;
	nsec += (cycle_delta * mult) >> shift;

	/* Convert to the desired sec, usec representation */
	tp->tv_sec += xnclock_divrem_billion(nsec, &rem);
	tp->tv_nsec = rem;

	return 0;
#else /* CONFIG_XENO_OPT_HOSTRT */
	return -EINVAL;
#endif
}

#define do_ext_clock(__clock_id, __handler, __ret, __args...)	\
({								\
	struct xnclock *__clock;				\
	int __val = 0, __nr;					\
	spl_t __s;						\
								\
	if (!__COBALT_CLOCK_EXT_P(__clock_id))			\
		__val = -EINVAL;				\
	else {							\
		__nr = __COBALT_CLOCK_EXT_INDEX(__clock_id);	\
		xnlock_get_irqsave(&nklock, __s);		\
		if (!test_bit(__nr, cobalt_clock_extids)) {	\
			xnlock_put_irqrestore(&nklock, __s);	\
			__val = -EINVAL;			\
		} else {					\
			__clock = external_clocks[__nr];	\
			(__ret) = xnclock_ ## __handler(__clock, ##__args); \
			xnlock_put_irqrestore(&nklock, __s);	\
		}						\
	}							\
	__val;							\
})

int __cobalt_clock_getres(clockid_t clock_id, struct timespec *ts)
{
	xnticks_t ns;
	int ret;

	switch (clock_id) {
	case CLOCK_REALTIME:
	case CLOCK_MONOTONIC:
	case CLOCK_MONOTONIC_RAW:
		ns2ts(ts, 1);
		break;
	default:
		ret = do_ext_clock(clock_id, get_resolution, ns);
		if (ret)
			return ret;
		ns2ts(ts, ns);
	}

	trace_cobalt_clock_getres(clock_id, ts);

	return 0;
}

COBALT_SYSCALL(clock_getres, current,
	       (clockid_t clock_id, struct timespec __user *u_ts))
{
	struct timespec ts;
	int ret;

	ret = __cobalt_clock_getres(clock_id, &ts);
	if (ret)
		return ret;

	if (u_ts && cobalt_copy_to_user(u_ts, &ts, sizeof(ts)))
		return -EFAULT;

	trace_cobalt_clock_getres(clock_id, &ts);

	return 0;
}

int __cobalt_clock_gettime(clockid_t clock_id, struct timespec *ts)
{
	xnticks_t ns;
	int ret;

	switch (clock_id) {
	case CLOCK_REALTIME:
		ns2ts(ts, xnclock_read_realtime(&nkclock));
		break;
	case CLOCK_MONOTONIC:
	case CLOCK_MONOTONIC_RAW:
		ns2ts(ts, xnclock_read_monotonic(&nkclock));
		break;
	case CLOCK_HOST_REALTIME:
		if (do_clock_host_realtime(ts) != 0)
			return -EINVAL;
		break;
	default:
		ret = do_ext_clock(clock_id, read_monotonic, ns);
		if (ret)
			return ret;
		ns2ts(ts, ns);
	}

	trace_cobalt_clock_gettime(clock_id, ts);

	return 0;
}

COBALT_SYSCALL(clock_gettime, current,
	       (clockid_t clock_id, struct timespec __user *u_ts))
{
	struct timespec ts;
	int ret;

	ret = __cobalt_clock_gettime(clock_id, &ts);
	if (ret)
		return ret;

	if (cobalt_copy_to_user(u_ts, &ts, sizeof(*u_ts)))
		return -EFAULT;

	trace_cobalt_clock_gettime(clock_id, &ts);

	return 0;
}

int __cobalt_clock_settime(clockid_t clock_id, const struct timespec *ts)
{
	int _ret, ret = 0;
	xnticks_t now;
	spl_t s;

	if ((unsigned long)ts->tv_nsec >= ONE_BILLION)
		return -EINVAL;

	switch (clock_id) {
	case CLOCK_REALTIME:
		xnlock_get_irqsave(&nklock, s);
		now = xnclock_read_realtime(&nkclock);
		xnclock_adjust(&nkclock, (xnsticks_t) (ts2ns(ts) - now));
		xnlock_put_irqrestore(&nklock, s);
		break;
	default:
		_ret = do_ext_clock(clock_id, set_time, ret, ts);
		if (_ret || ret)
			return _ret ?: ret;
	}

	trace_cobalt_clock_settime(clock_id, ts);

	return 0;
}

COBALT_SYSCALL(clock_settime, current,
	       (clockid_t clock_id, const struct timespec __user *u_ts))
{
	struct timespec ts;

	if (cobalt_copy_from_user(&ts, u_ts, sizeof(ts)))
		return -EFAULT;

	return __cobalt_clock_settime(clock_id, &ts);
}

int __cobalt_clock_nanosleep(clockid_t clock_id, int flags,
			     const struct timespec *rqt,
			     struct timespec *rmt)
{
	struct restart_block *restart;
	struct xnthread *cur;
	xnsticks_t timeout, rem;
	int ret = 0;
	spl_t s;

	trace_cobalt_clock_nanosleep(clock_id, flags, rqt);

	if (clock_id != CLOCK_MONOTONIC &&
	    clock_id != CLOCK_MONOTONIC_RAW &&
	    clock_id != CLOCK_REALTIME)
		return -EOPNOTSUPP;

	if (rqt->tv_sec < 0)
		return -EINVAL;

	if ((unsigned long)rqt->tv_nsec >= ONE_BILLION)
		return -EINVAL;

	if (flags & ~TIMER_ABSTIME)
		return -EINVAL;

	cur = xnthread_current();

	if (xnthread_test_localinfo(cur, XNSYSRST)) {
		xnthread_clear_localinfo(cur, XNSYSRST);

		restart = cobalt_get_restart_block(current);

		if (restart->fn != cobalt_restart_syscall_placeholder) {
			if (rmt) {
				xnlock_get_irqsave(&nklock, s);
				rem = xntimer_get_timeout_stopped(&cur->rtimer);
				xnlock_put_irqrestore(&nklock, s);
				ns2ts(rmt, rem > 1 ? rem : 0);
			}
			return -EINTR;
		}

		timeout = restart->nanosleep.expires;
	} else
		timeout = ts2ns(rqt);

	xnlock_get_irqsave(&nklock, s);

	xnthread_suspend(cur, XNDELAY, timeout + 1,
			 clock_flag(flags, clock_id), NULL);

	if (xnthread_test_info(cur, XNBREAK)) {
		if (signal_pending(current)) {
			restart = cobalt_get_restart_block(current);
			restart->nanosleep.expires =
				(flags & TIMER_ABSTIME) ? timeout :
				    xntimer_get_timeout_stopped(&cur->rtimer);
			xnlock_put_irqrestore(&nklock, s);
			restart->fn = cobalt_restart_syscall_placeholder;

			xnthread_set_localinfo(cur, XNSYSRST);

			return -ERESTARTSYS;
		}

		if (flags == 0 && rmt) {
			rem = xntimer_get_timeout_stopped(&cur->rtimer);
			xnlock_put_irqrestore(&nklock, s);
			ns2ts(rmt, rem > 1 ? rem : 0);
		} else
			xnlock_put_irqrestore(&nklock, s);

		return -EINTR;
	}

	xnlock_put_irqrestore(&nklock, s);

	return ret;
}

COBALT_SYSCALL(clock_nanosleep, primary,
	       (clockid_t clock_id, int flags,
		const struct timespec __user *u_rqt,
		struct timespec __user *u_rmt))
{
	struct timespec rqt, rmt, *rmtp = NULL;
	int ret;

	if (u_rmt)
		rmtp = &rmt;

	if (cobalt_copy_from_user(&rqt, u_rqt, sizeof(rqt)))
		return -EFAULT;

	ret = __cobalt_clock_nanosleep(clock_id, flags, &rqt, rmtp);
	if (ret == -EINTR && flags == 0 && rmtp) {
		if (cobalt_copy_to_user(u_rmt, rmtp, sizeof(*u_rmt)))
			return -EFAULT;
	}

	return ret;
}

int cobalt_clock_register(struct xnclock *clock, const cpumask_t *affinity,
			  clockid_t *clk_id)
{
	int ret, nr;
	spl_t s;

	xnlock_get_irqsave(&nklock, s);

	nr = find_first_zero_bit(cobalt_clock_extids, COBALT_MAX_EXTCLOCKS);
	if (nr >= COBALT_MAX_EXTCLOCKS) {
		xnlock_put_irqrestore(&nklock, s);
		return -EAGAIN;
	}

	/*
	 * CAUTION: a bit raised in cobalt_clock_extids means that the
	 * corresponding entry in external_clocks[] is valid. The
	 * converse assumption is NOT true.
	 */
	__set_bit(nr, cobalt_clock_extids);
	external_clocks[nr] = clock;

	xnlock_put_irqrestore(&nklock, s);

	ret = xnclock_register(clock, affinity);
	if (ret)
		return ret;

	clock->id = nr;
	*clk_id = __COBALT_CLOCK_EXT(clock->id);

	trace_cobalt_clock_register(clock->name, *clk_id);

	return 0;
}
EXPORT_SYMBOL_GPL(cobalt_clock_register);

void cobalt_clock_deregister(struct xnclock *clock)
{
	trace_cobalt_clock_deregister(clock->name, clock->id);
	clear_bit(clock->id, cobalt_clock_extids);
	smp_mb__after_atomic();
	external_clocks[clock->id] = NULL;
	xnclock_deregister(clock);
}
EXPORT_SYMBOL_GPL(cobalt_clock_deregister);

struct xnclock *cobalt_clock_find(clockid_t clock_id)
{
	struct xnclock *clock = ERR_PTR(-EINVAL);
	spl_t s;
	int nr;

	if (clock_id == CLOCK_MONOTONIC ||
	    clock_id == CLOCK_MONOTONIC_RAW ||
	    clock_id == CLOCK_REALTIME)
		return &nkclock;
	
	if (__COBALT_CLOCK_EXT_P(clock_id)) {
		nr = __COBALT_CLOCK_EXT_INDEX(clock_id);
		xnlock_get_irqsave(&nklock, s);
		if (test_bit(nr, cobalt_clock_extids))
			clock = external_clocks[nr];
		xnlock_put_irqrestore(&nklock, s);
	}

	return clock;
}
EXPORT_SYMBOL_GPL(cobalt_clock_find);
