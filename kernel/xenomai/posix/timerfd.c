/*
 * Copyright (C) 2013 Gilles Chanteperdrix <gilles.chanteperdrix@xenomai.org>.
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

#include <linux/timerfd.h>
#include <linux/err.h>
#include <cobalt/kernel/timer.h>
#include <cobalt/kernel/select.h>
#include <rtdm/fd.h>
#include "internal.h"
#include "clock.h"
#include "timer.h"
#include "timerfd.h"

struct cobalt_tfd {
	int flags;
	clockid_t clockid;
	struct rtdm_fd fd;
	struct xntimer timer;
	DECLARE_XNSELECT(read_select);
	struct itimerspec value;
	struct xnsynch readers;
	struct xnthread *target;
};

#define COBALT_TFD_TICKED	(1 << 2)

#define COBALT_TFD_SETTIME_FLAGS (TFD_TIMER_ABSTIME | TFD_WAKEUP)

static ssize_t timerfd_read(struct rtdm_fd *fd, void __user *buf, size_t size)
{
	struct cobalt_tfd *tfd;
	__u64 __user *u_ticks;
	__u64 ticks = 0;
	bool aligned;
	spl_t s;
	int err;

	if (size < sizeof(ticks))
		return -EINVAL;

	u_ticks = buf;
	if (!access_wok(u_ticks, sizeof(*u_ticks)))
		return -EFAULT;

	aligned = (((unsigned long)buf) & (sizeof(ticks) - 1)) == 0;

	tfd = container_of(fd, struct cobalt_tfd, fd);

	xnlock_get_irqsave(&nklock, s);
	if (tfd->flags & COBALT_TFD_TICKED) {
		err = 0;
		goto out;
	}
	if (rtdm_fd_flags(fd) & O_NONBLOCK) {
		err = -EAGAIN;
		goto out;
	}

	do {
		err = xnsynch_sleep_on(&tfd->readers, XN_INFINITE, XN_RELATIVE);
	} while (err == 0 && (tfd->flags & COBALT_TFD_TICKED) == 0);

	if (err & XNBREAK)
		err = -EINTR;
  out:
	if (err == 0) {
		xnticks_t now;

		if (xntimer_periodic_p(&tfd->timer)) {
			now = xnclock_read_raw(xntimer_clock(&tfd->timer));
			ticks = 1 + xntimer_get_overruns(&tfd->timer,
					 xnthread_current(), now);
		} else
			ticks = 1;

		tfd->flags &= ~COBALT_TFD_TICKED;
		xnselect_signal(&tfd->read_select, 0);
	}
	xnlock_put_irqrestore(&nklock, s);

	if (err == 0) {
		err = aligned ? __xn_put_user(ticks, u_ticks) :
			__xn_copy_to_user(buf, &ticks, sizeof(ticks));
		if (err)
			err =-EFAULT;
	}

	return err ?: sizeof(ticks);
}

static int
timerfd_select(struct rtdm_fd *fd, struct xnselector *selector,
	       unsigned type, unsigned index)
{
	struct cobalt_tfd *tfd = container_of(fd, struct cobalt_tfd, fd);
	struct xnselect_binding *binding;
	spl_t s;
	int err;

	if (type != XNSELECT_READ)
		return -EBADF;

	binding = xnmalloc(sizeof(*binding));
	if (binding == NULL)
		return -ENOMEM;

	xnlock_get_irqsave(&nklock, s);
	xntimer_set_affinity(&tfd->timer, xnthread_current()->sched);
	err = xnselect_bind(&tfd->read_select, binding, selector, type,
			index, tfd->flags & COBALT_TFD_TICKED);
	xnlock_put_irqrestore(&nklock, s);

	return err;
}

static void timerfd_close(struct rtdm_fd *fd)
{
	struct cobalt_tfd *tfd = container_of(fd, struct cobalt_tfd, fd);
	spl_t s;

	xnlock_get_irqsave(&nklock, s);
	xntimer_destroy(&tfd->timer);
	xnsynch_destroy(&tfd->readers);
	xnsched_run();
	xnlock_put_irqrestore(&nklock, s);
	xnselect_destroy(&tfd->read_select); /* Reschedules. */
	xnfree(tfd);
}

static struct rtdm_fd_ops timerfd_ops = {
	.read_rt = timerfd_read,
	.select = timerfd_select,
	.close = timerfd_close,
};

static void timerfd_handler(struct xntimer *xntimer)
{
	struct cobalt_tfd *tfd;

	tfd = container_of(xntimer, struct cobalt_tfd, timer);
	tfd->flags |= COBALT_TFD_TICKED;
	xnselect_signal(&tfd->read_select, 1);
	xnsynch_wakeup_one_sleeper(&tfd->readers);
	if (tfd->target)
		xnthread_unblock(tfd->target);
}

COBALT_SYSCALL(timerfd_create, lostage, (int clockid, int flags))
{
	struct cobalt_tfd *tfd;
	struct xnthread *curr;
	struct xnclock *clock;
	int ret, ufd;

	if (flags & ~TFD_CREATE_FLAGS)
		return -EINVAL;

	clock = cobalt_clock_find(clockid);
	if (IS_ERR(clock))
		return PTR_ERR(clock);

	tfd = xnmalloc(sizeof(*tfd));
	if (tfd == NULL)
		return -ENOMEM;

	ufd = __rtdm_anon_getfd("[cobalt-timerfd]",
				O_RDWR | (flags & TFD_SHARED_FCNTL_FLAGS));
	if (ufd < 0) {
		ret = ufd;
		goto fail_getfd;
	}

	tfd->flags = flags & ~TFD_NONBLOCK;
	tfd->fd.oflags = (flags & TFD_NONBLOCK) ? O_NONBLOCK : 0;
	tfd->clockid = clockid;
	curr = xnthread_current();
	xntimer_init(&tfd->timer, clock, timerfd_handler,
		     curr ? curr->sched : NULL, XNTIMER_UGRAVITY);
	xnsynch_init(&tfd->readers, XNSYNCH_PRIO, NULL);
	xnselect_init(&tfd->read_select);
	tfd->target = NULL;

	ret = rtdm_fd_enter(&tfd->fd, ufd, COBALT_TIMERFD_MAGIC, &timerfd_ops);
	if (ret < 0)
		goto fail;

	ret = rtdm_fd_register(&tfd->fd, ufd);
	if (ret < 0)
		goto fail;

	return ufd;
fail:
	xnselect_destroy(&tfd->read_select);
	xnsynch_destroy(&tfd->readers);
	xntimer_destroy(&tfd->timer);
	__rtdm_anon_putfd(ufd);
fail_getfd:
	xnfree(tfd);

	return ret;
}

static inline struct cobalt_tfd *tfd_get(int ufd)
{
	struct rtdm_fd *fd;

	fd = rtdm_fd_get(ufd, COBALT_TIMERFD_MAGIC);
	if (IS_ERR(fd)) {
		int err = PTR_ERR(fd);
		if (err == -EBADF && cobalt_current_process() == NULL)
			err = -EPERM;
		return ERR_PTR(err);
	}

	return container_of(fd, struct cobalt_tfd, fd);
}

static inline void tfd_put(struct cobalt_tfd *tfd)
{
	rtdm_fd_put(&tfd->fd);
}

int __cobalt_timerfd_settime(int fd, int flags,
			     const struct itimerspec *value,
			     struct itimerspec *ovalue)
{
	struct cobalt_tfd *tfd;
	int cflag, ret;
	spl_t s;

	if (flags & ~COBALT_TFD_SETTIME_FLAGS)
		return -EINVAL;

	tfd = tfd_get(fd);
	if (IS_ERR(tfd))
		return PTR_ERR(tfd);

	cflag = (flags & TFD_TIMER_ABSTIME) ? TIMER_ABSTIME : 0;

	xnlock_get_irqsave(&nklock, s);

	tfd->target = NULL;
	if (flags & TFD_WAKEUP) {
		tfd->target = xnthread_current();
		if (tfd->target == NULL) {
			ret = -EPERM;
			goto out;
		}
	}

	if (ovalue)
		__cobalt_timer_getval(&tfd->timer, ovalue);

	xntimer_set_affinity(&tfd->timer, xnthread_current()->sched);

	ret = __cobalt_timer_setval(&tfd->timer,
				    clock_flag(cflag, tfd->clockid), value);
out:
	xnlock_put_irqrestore(&nklock, s);

	tfd_put(tfd);

	return ret;
}

COBALT_SYSCALL(timerfd_settime, primary,
	       (int fd, int flags,
		const struct itimerspec __user *new_value,
		struct itimerspec __user *old_value))
{
	struct itimerspec ovalue, value;
	int ret;

	ret = cobalt_copy_from_user(&value, new_value, sizeof(value));
	if (ret)
		return ret;

	ret = __cobalt_timerfd_settime(fd, flags, &value, &ovalue);
	if (ret)
		return ret;

	if (old_value) {
		ret = cobalt_copy_to_user(old_value, &ovalue, sizeof(ovalue));
		value.it_value.tv_sec = 0;
		value.it_value.tv_nsec = 0;
		__cobalt_timerfd_settime(fd, flags, &value, NULL);
	}

	return ret;
}

int __cobalt_timerfd_gettime(int fd, struct itimerspec *value)
{
	struct cobalt_tfd *tfd;
	spl_t s;

	tfd = tfd_get(fd);
	if (IS_ERR(tfd))
		return PTR_ERR(tfd);

	xnlock_get_irqsave(&nklock, s);
	__cobalt_timer_getval(&tfd->timer, value);
	xnlock_put_irqrestore(&nklock, s);

	tfd_put(tfd);

	return 0;
}

COBALT_SYSCALL(timerfd_gettime, current,
	       (int fd, struct itimerspec __user *curr_value))
{
	struct itimerspec value;
	int ret;

	ret = __cobalt_timerfd_gettime(fd, &value);

	return ret ?: cobalt_copy_to_user(curr_value, &value, sizeof(value));
}
