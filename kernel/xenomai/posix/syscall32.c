/*
 * Copyright (C) 2014 Philippe Gerum <rpm@xenomai.org>
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
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#include <linux/types.h>
#include <linux/err.h>
#include <cobalt/uapi/syscall.h>
#include <xenomai/rtdm/internal.h>
#include "internal.h"
#include "syscall32.h"
#include "thread.h"
#include "mutex.h"
#include "cond.h"
#include "sem.h"
#include "sched.h"
#include "clock.h"
#include "timer.h"
#include "timerfd.h"
#include "signal.h"
#include "monitor.h"
#include "event.h"
#include "mqueue.h"
#include "io.h"
#include "../debug.h"

COBALT_SYSCALL32emu(thread_create, init,
		    (compat_ulong_t pth,
		     int policy,
		     const struct compat_sched_param_ex __user *u_param_ex,
		     int xid,
		     __u32 __user *u_winoff))
{
	struct sched_param_ex param_ex;
	int ret;

	ret = sys32_get_param_ex(policy, &param_ex, u_param_ex);
	if (ret)
		return ret;

	return __cobalt_thread_create(pth, policy, &param_ex, xid, u_winoff);
}

COBALT_SYSCALL32emu(thread_setschedparam_ex, conforming,
		    (compat_ulong_t pth,
		     int policy,
		     const struct compat_sched_param_ex __user *u_param_ex,
		     __u32 __user *u_winoff,
		     int __user *u_promoted))
{
	struct sched_param_ex param_ex;
	int ret;

	ret = sys32_get_param_ex(policy, &param_ex, u_param_ex);
	if (ret)
		return ret;

	return cobalt_thread_setschedparam_ex(pth, policy, &param_ex,
					      u_winoff, u_promoted);
}

COBALT_SYSCALL32emu(thread_getschedparam_ex, current,
		    (compat_ulong_t pth,
		     int __user *u_policy,
		     struct compat_sched_param_ex __user *u_param))
{
	struct sched_param_ex param_ex;
	int ret, policy;

	ret = cobalt_thread_getschedparam_ex(pth, &policy, &param_ex);
	if (ret)
		return ret;

	ret = cobalt_copy_to_user(u_policy, &policy, sizeof(policy));

	return ret ?: sys32_put_param_ex(policy, u_param, &param_ex);
}

COBALT_SYSCALL32emu(thread_setschedprio, conforming,
		    (compat_ulong_t pth,
		     int prio,
		     __u32 __user *u_winoff,
		     int __user *u_promoted))
{
	return cobalt_thread_setschedprio(pth, prio, u_winoff, u_promoted);
}

static inline int sys32_fetch_timeout(struct timespec *ts,
				      const void __user *u_ts)
{
	return u_ts == NULL ? -EFAULT :
		sys32_get_timespec(ts, u_ts);
}

COBALT_SYSCALL32emu(sem_open, lostage,
		    (compat_uptr_t __user *u_addrp,
		     const char __user *u_name,
		     int oflags, mode_t mode, unsigned int value))
{
	struct cobalt_sem_shadow __user *usm;
	compat_uptr_t cusm;

	if (__xn_get_user(cusm, u_addrp))
		return -EFAULT;

	usm = __cobalt_sem_open(compat_ptr(cusm), u_name, oflags, mode, value);
	if (IS_ERR(usm))
		return PTR_ERR(usm);

	return __xn_put_user(ptr_to_compat(usm), u_addrp) ? -EFAULT : 0;
}

COBALT_SYSCALL32emu(sem_timedwait, primary,
		    (struct cobalt_sem_shadow __user *u_sem,
		     struct compat_timespec __user *u_ts))
{
	return __cobalt_sem_timedwait(u_sem, u_ts, sys32_fetch_timeout);
}

COBALT_SYSCALL32emu(clock_getres, current,
		    (clockid_t clock_id,
		     struct compat_timespec __user *u_ts))
{
	struct timespec ts;
	int ret;

	ret = __cobalt_clock_getres(clock_id, &ts);
	if (ret)
		return ret;

	return u_ts ? sys32_put_timespec(u_ts, &ts) : 0;
}

COBALT_SYSCALL32emu(clock_gettime, current,
		    (clockid_t clock_id,
		     struct compat_timespec __user *u_ts))
{
	struct timespec ts;
	int ret;

	ret = __cobalt_clock_gettime(clock_id, &ts);
	if (ret)
		return ret;

	return sys32_put_timespec(u_ts, &ts);
}

COBALT_SYSCALL32emu(clock_settime, current,
		    (clockid_t clock_id,
		     const struct compat_timespec __user *u_ts))
{
	struct timespec ts;
	int ret;

	ret = sys32_get_timespec(&ts, u_ts);
	if (ret)
		return ret;

	return __cobalt_clock_settime(clock_id, &ts);
}

COBALT_SYSCALL32emu(clock_adjtime, current,
		    (clockid_t clock_id, struct old_timex32 __user *u_tx))
{
	struct timex tx;
	int ret;

	ret = sys32_get_timex(&tx, u_tx);
	if (ret)
		return ret;

	ret = __cobalt_clock_adjtime(clock_id, &tx);
	if (ret)
		return ret;

	return sys32_put_timex(u_tx, &tx);
}

COBALT_SYSCALL32emu(clock_nanosleep, nonrestartable,
		    (clockid_t clock_id, int flags,
		     const struct compat_timespec __user *u_rqt,
		     struct compat_timespec __user *u_rmt))
{
	struct timespec rqt, rmt, *rmtp = NULL;
	int ret;

	if (u_rmt)
		rmtp = &rmt;

	ret = sys32_get_timespec(&rqt, u_rqt);
	if (ret)
		return ret;

	ret = __cobalt_clock_nanosleep(clock_id, flags, &rqt, rmtp);
	if (ret == -EINTR && flags == 0 && rmtp)
		ret = sys32_put_timespec(u_rmt, rmtp);

	return ret;
}

COBALT_SYSCALL32emu(mutex_timedlock, primary,
		    (struct cobalt_mutex_shadow __user *u_mx,
		     const struct compat_timespec __user *u_ts))
{
	return __cobalt_mutex_timedlock_break(u_mx, u_ts, sys32_fetch_timeout);
}

COBALT_SYSCALL32emu(cond_wait_prologue, nonrestartable,
		    (struct cobalt_cond_shadow __user *u_cnd,
		     struct cobalt_mutex_shadow __user *u_mx,
		     int *u_err,
		     unsigned int timed,
		     struct compat_timespec __user *u_ts))
{
	return __cobalt_cond_wait_prologue(u_cnd, u_mx, u_err, u_ts,
					   timed ? sys32_fetch_timeout : NULL);
}

COBALT_SYSCALL32emu(mq_open, lostage,
		    (const char __user *u_name, int oflags,
		     mode_t mode, struct compat_mq_attr __user *u_attr))
{
	struct mq_attr _attr, *attr = &_attr;
	int ret;

	if ((oflags & O_CREAT) && u_attr) {
		ret = sys32_get_mqattr(&_attr, u_attr);
		if (ret)
			return ret;
	} else
		attr = NULL;

	return __cobalt_mq_open(u_name, oflags, mode, attr);
}

COBALT_SYSCALL32emu(mq_getattr, current,
		    (mqd_t uqd, struct compat_mq_attr __user *u_attr))
{
	struct mq_attr attr;
	int ret;

	ret = __cobalt_mq_getattr(uqd, &attr);
	if (ret)
		return ret;

	return sys32_put_mqattr(u_attr, &attr);
}

COBALT_SYSCALL32emu(mq_timedsend, primary,
		    (mqd_t uqd, const void __user *u_buf, size_t len,
		     unsigned int prio,
		     const struct compat_timespec __user *u_ts))
{
	return __cobalt_mq_timedsend(uqd, u_buf, len, prio,
				     u_ts, u_ts ? sys32_fetch_timeout : NULL);
}

COBALT_SYSCALL32emu(mq_timedreceive, primary,
		    (mqd_t uqd, void __user *u_buf,
		     compat_ssize_t __user *u_len,
		     unsigned int __user *u_prio,
		     const struct compat_timespec __user *u_ts))
{
	compat_ssize_t clen;
	ssize_t len;
	int ret;

	ret = cobalt_copy_from_user(&clen, u_len, sizeof(*u_len));
	if (ret)
		return ret;

	len = clen;
	ret = __cobalt_mq_timedreceive(uqd, u_buf, &len, u_prio,
				       u_ts, u_ts ? sys32_fetch_timeout : NULL);
	clen = len;

	return ret ?: cobalt_copy_to_user(u_len, &clen, sizeof(*u_len));
}

static inline int mq_fetch_timeout(struct timespec *ts,
				   const void __user *u_ts)
{
	return u_ts == NULL ? -EFAULT :
		cobalt_copy_from_user(ts, u_ts, sizeof(*ts));

}

COBALT_SYSCALL32emu(mq_notify, primary,
		    (mqd_t fd, const struct compat_sigevent *__user u_cev))
{
	struct sigevent sev;
	int ret;

	if (u_cev) {
		ret = sys32_get_sigevent(&sev, u_cev);
		if (ret)
			return ret;
	}

	return __cobalt_mq_notify(fd, u_cev ? &sev : NULL);
}

COBALT_SYSCALL32emu(sched_weightprio, current,
		    (int policy,
		     const struct compat_sched_param_ex __user *u_param))
{
	struct sched_param_ex param_ex;
	int ret;

	ret = sys32_get_param_ex(policy, &param_ex, u_param);
	if (ret)
		return ret;

	return __cobalt_sched_weightprio(policy, &param_ex);
}

static union sched_config *
sys32_fetch_config(int policy, const void __user *u_config, size_t *len)
{
	union compat_sched_config *cbuf;
	union sched_config *buf;
	int ret, n;

	if (u_config == NULL)
		return ERR_PTR(-EFAULT);

	if (policy == SCHED_QUOTA && *len < sizeof(cbuf->quota))
		return ERR_PTR(-EINVAL);

	cbuf = xnmalloc(*len);
	if (cbuf == NULL)
		return ERR_PTR(-ENOMEM);

	ret = cobalt_copy_from_user(cbuf, u_config, *len);
	if (ret) {
		buf = ERR_PTR(ret);
		goto out;
	}

	switch (policy) {
	case SCHED_TP:
		*len = sched_tp_confsz(cbuf->tp.nr_windows);
		break;
	case SCHED_QUOTA:
		break;
	default:
		buf = ERR_PTR(-EINVAL);
		goto out;
	}

	buf = xnmalloc(*len);
	if (buf == NULL) {
		buf = ERR_PTR(-ENOMEM);
		goto out;
	}

	if (policy == SCHED_QUOTA)
		memcpy(&buf->quota, &cbuf->quota, sizeof(cbuf->quota));
	else {
		buf->tp.op = cbuf->tp.op;
		buf->tp.nr_windows = cbuf->tp.nr_windows;
		for (n = 0; n < buf->tp.nr_windows; n++) {
			buf->tp.windows[n].ptid = cbuf->tp.windows[n].ptid;
			buf->tp.windows[n].offset.tv_sec = cbuf->tp.windows[n].offset.tv_sec;
			buf->tp.windows[n].offset.tv_nsec = cbuf->tp.windows[n].offset.tv_nsec;
			buf->tp.windows[n].duration.tv_sec = cbuf->tp.windows[n].duration.tv_sec;
			buf->tp.windows[n].duration.tv_nsec = cbuf->tp.windows[n].duration.tv_nsec;
		}
	}
out:
	xnfree(cbuf);

	return buf;
}

static int sys32_ack_config(int policy, const union sched_config *config,
			    void __user *u_config)
{
	union compat_sched_config __user *u_p = u_config;

	if (policy != SCHED_QUOTA)
		return 0;

	return u_config == NULL ? -EFAULT :
		cobalt_copy_to_user(&u_p->quota.info, &config->quota.info,
				       sizeof(u_p->quota.info));
}

static ssize_t sys32_put_config(int policy,
				void __user *u_config, size_t u_len,
				const union sched_config *config, size_t len)
{
	union compat_sched_config __user *u_p = u_config;
	int n, ret;

	if (u_config == NULL)
		return -EFAULT;

	if (policy == SCHED_QUOTA) {
		if (u_len < sizeof(u_p->quota))
			return -EINVAL;
		return cobalt_copy_to_user(&u_p->quota.info, &config->quota.info,
					      sizeof(u_p->quota.info)) ?:
			sizeof(u_p->quota.info);
	}

	/* SCHED_TP */

	if (u_len < compat_sched_tp_confsz(config->tp.nr_windows))
		return -ENOSPC;

	__xn_put_user(config->tp.op, &u_p->tp.op);
	__xn_put_user(config->tp.nr_windows, &u_p->tp.nr_windows);

	for (n = 0, ret = 0; n < config->tp.nr_windows; n++) {
		ret |= __xn_put_user(config->tp.windows[n].ptid,
				     &u_p->tp.windows[n].ptid);
		ret |= __xn_put_user(config->tp.windows[n].offset.tv_sec,
				     &u_p->tp.windows[n].offset.tv_sec);
		ret |= __xn_put_user(config->tp.windows[n].offset.tv_nsec,
				     &u_p->tp.windows[n].offset.tv_nsec);
		ret |= __xn_put_user(config->tp.windows[n].duration.tv_sec,
				     &u_p->tp.windows[n].duration.tv_sec);
		ret |= __xn_put_user(config->tp.windows[n].duration.tv_nsec,
				     &u_p->tp.windows[n].duration.tv_nsec);
	}

	return ret ?: u_len;
}

COBALT_SYSCALL32emu(sched_setconfig_np, conforming,
		    (int cpu, int policy,
		     union compat_sched_config __user *u_config,
		     size_t len))
{
	return __cobalt_sched_setconfig_np(cpu, policy, u_config, len,
					   sys32_fetch_config, sys32_ack_config);
}

COBALT_SYSCALL32emu(sched_getconfig_np, conformin,
		    (int cpu, int policy,
		     union compat_sched_config __user *u_config,
		     size_t len))
{
	return __cobalt_sched_getconfig_np(cpu, policy, u_config, len,
					   sys32_fetch_config, sys32_put_config);
}

COBALT_SYSCALL32emu(sched_setscheduler_ex, conforming,
		    (compat_pid_t pid,
		     int policy,
		     const struct compat_sched_param_ex __user *u_param_ex,
		     __u32 __user *u_winoff,
		     int __user *u_promoted))
{
	struct sched_param_ex param_ex;
	int ret;

	ret = sys32_get_param_ex(policy, &param_ex, u_param_ex);
	if (ret)
		return ret;

	return cobalt_sched_setscheduler_ex(pid, policy, &param_ex,
					    u_winoff, u_promoted);
}

COBALT_SYSCALL32emu(sched_getscheduler_ex, current,
		    (compat_pid_t pid,
		     int __user *u_policy,
		     struct compat_sched_param_ex __user *u_param))
{
	struct sched_param_ex param_ex;
	int ret, policy;

	ret = cobalt_sched_getscheduler_ex(pid, &policy, &param_ex);
	if (ret)
		return ret;

	ret = cobalt_copy_to_user(u_policy, &policy, sizeof(policy));

	return ret ?: sys32_put_param_ex(policy, u_param, &param_ex);
}

COBALT_SYSCALL32emu(timer_create, current,
		    (clockid_t clock,
		     const struct compat_sigevent __user *u_sev,
		     timer_t __user *u_tm))
{
	struct sigevent sev, *evp = NULL;
	int ret;

	if (u_sev) {
		evp = &sev;
		ret = sys32_get_sigevent(&sev, u_sev);
		if (ret)
			return ret;
	}

	return __cobalt_timer_create(clock, evp, u_tm);
}

COBALT_SYSCALL32emu(timer_settime, primary,
		    (timer_t tm, int flags,
		     const struct compat_itimerspec __user *u_newval,
		     struct compat_itimerspec __user *u_oldval))
{
	struct itimerspec newv, oldv, *oldvp = &oldv;
	int ret;

	if (u_oldval == NULL)
		oldvp = NULL;

	ret = sys32_get_itimerspec(&newv, u_newval);
	if (ret)
		return ret;

	ret = __cobalt_timer_settime(tm, flags, &newv, oldvp);
	if (ret)
		return ret;

	if (oldvp) {
		ret = sys32_put_itimerspec(u_oldval, oldvp);
		if (ret)
			__cobalt_timer_settime(tm, flags, oldvp, NULL);
	}

	return ret;
}

COBALT_SYSCALL32emu(timer_gettime, current,
		    (timer_t tm, struct compat_itimerspec __user *u_val))
{
	struct itimerspec val;
	int ret;

	ret = __cobalt_timer_gettime(tm, &val);

	return ret ?: sys32_put_itimerspec(u_val, &val);
}

COBALT_SYSCALL32emu(timerfd_settime, primary,
		    (int fd, int flags,
		     const struct compat_itimerspec __user *new_value,
		     struct compat_itimerspec __user *old_value))
{
	struct itimerspec ovalue, value;
	int ret;

	ret = sys32_get_itimerspec(&value, new_value);
	if (ret)
		return ret;

	ret = __cobalt_timerfd_settime(fd, flags, &value, &ovalue);
	if (ret)
		return ret;

	if (old_value) {
		ret = sys32_put_itimerspec(old_value, &ovalue);
		value.it_value.tv_sec = 0;
		value.it_value.tv_nsec = 0;
		__cobalt_timerfd_settime(fd, flags, &value, NULL);
	}

	return ret;
}

COBALT_SYSCALL32emu(timerfd_gettime, current,
		    (int fd, struct compat_itimerspec __user *curr_value))
{
	struct itimerspec value;
	int ret;

	ret = __cobalt_timerfd_gettime(fd, &value);

	return ret ?: sys32_put_itimerspec(curr_value, &value);
}

COBALT_SYSCALL32emu(sigwait, primary,
		    (const compat_sigset_t __user *u_set,
		     int __user *u_sig))
{
	sigset_t set;
	int ret, sig;

	ret = sys32_get_sigset(&set, u_set);
	if (ret)
		return ret;

	sig = __cobalt_sigwait(&set);
	if (sig < 0)
		return sig;

	return cobalt_copy_to_user(u_sig, &sig, sizeof(*u_sig));
}

COBALT_SYSCALL32emu(sigtimedwait, nonrestartable,
		    (const compat_sigset_t __user *u_set,
		     struct compat_siginfo __user *u_si,
		     const struct compat_timespec __user *u_timeout))
{
	struct timespec timeout;
	sigset_t set;
	int ret;

	ret = sys32_get_sigset(&set, u_set);
	if (ret)
		return ret;

	ret = sys32_get_timespec(&timeout, u_timeout);
	if (ret)
		return ret;

	return __cobalt_sigtimedwait(&set, &timeout, u_si, true);
}

COBALT_SYSCALL32emu(sigwaitinfo, nonrestartable,
		    (const compat_sigset_t __user *u_set,
		     struct compat_siginfo __user *u_si))
{
	sigset_t set;
	int ret;

	ret = sys32_get_sigset(&set, u_set);
	if (ret)
		return ret;

	return __cobalt_sigwaitinfo(&set, u_si, true);
}

COBALT_SYSCALL32emu(sigpending, primary, (compat_old_sigset_t __user *u_set))
{
	struct cobalt_thread *curr = cobalt_current_thread();

	return sys32_put_sigset((compat_sigset_t *)u_set, &curr->sigpending);
}

COBALT_SYSCALL32emu(sigqueue, conforming,
		    (pid_t pid, int sig,
		     const union compat_sigval __user *u_value))
{
	union sigval val;
	int ret;

	ret = sys32_get_sigval(&val, u_value);

	return ret ?: __cobalt_sigqueue(pid, sig, &val);
}

COBALT_SYSCALL32emu(monitor_wait, nonrestartable,
		    (struct cobalt_monitor_shadow __user *u_mon,
		     int event, const struct compat_timespec __user *u_ts,
		     int __user *u_ret))
{
	struct timespec ts, *tsp = NULL;
	int ret;

	if (u_ts) {
		tsp = &ts;
		ret = sys32_get_timespec(&ts, u_ts);
		if (ret)
			return ret;
	}

	return __cobalt_monitor_wait(u_mon, event, tsp, u_ret);
}

COBALT_SYSCALL32emu(event_wait, primary,
		    (struct cobalt_event_shadow __user *u_event,
		     unsigned int bits,
		     unsigned int __user *u_bits_r,
		     int mode, const struct compat_timespec __user *u_ts))
{
	struct timespec ts, *tsp = NULL;
	int ret;

	if (u_ts) {
		tsp = &ts;
		ret = sys32_get_timespec(&ts, u_ts);
		if (ret)
			return ret;
	}

	return __cobalt_event_wait(u_event, bits, u_bits_r, mode, tsp);
}

COBALT_SYSCALL32emu(select, nonrestartable,
		    (int nfds,
		     compat_fd_set __user *u_rfds,
		     compat_fd_set __user *u_wfds,
		     compat_fd_set __user *u_xfds,
		     struct compat_timeval __user *u_tv))
{
	compat_fd_set __user *ufd_sets[XNSELECT_MAX_TYPES] = {
		[XNSELECT_READ] = u_rfds,
		[XNSELECT_WRITE] = u_wfds,
		[XNSELECT_EXCEPT] = u_xfds
	};
	fd_set *in_fds[XNSELECT_MAX_TYPES] = {NULL, NULL, NULL};
	fd_set *out_fds[XNSELECT_MAX_TYPES] = {NULL, NULL, NULL};
	fd_set in_fds_storage[XNSELECT_MAX_TYPES],
		out_fds_storage[XNSELECT_MAX_TYPES];
	xnticks_t timeout = XN_INFINITE;
	xntmode_t mode = XN_RELATIVE;
	struct xnselector *selector;
	struct xnthread *curr;
	struct timeval tv;
	xnsticks_t diff;
	size_t fds_size;
	int i, err;

	curr = xnthread_current();

	if (u_tv) {
		err = sys32_get_timeval(&tv, u_tv);
		if (err)
			return err;

		if (tv.tv_usec >= 1000000)
			return -EINVAL;

		timeout = clock_get_ticks(CLOCK_MONOTONIC) + tv2ns(&tv);
		mode = XN_ABSOLUTE;
	}

	fds_size = __FDELT__(nfds + __NFDBITS__ - 1) * sizeof(compat_ulong_t);

	for (i = 0; i < XNSELECT_MAX_TYPES; i++)
		if (ufd_sets[i]) {
			in_fds[i] = &in_fds_storage[i];
			out_fds[i] = & out_fds_storage[i];
			if (sys32_get_fdset(in_fds[i], ufd_sets[i], fds_size) < 0)
				return -EFAULT;
		}

	selector = curr->selector;
	if (selector == NULL) {
		/* Bail out if non-RTDM fildes is found. */
		if (!__cobalt_first_fd_valid_p(in_fds, nfds))
			return -EBADF;

		selector = xnmalloc(sizeof(*curr->selector));
		if (selector == NULL)
			return -ENOMEM;
		xnselector_init(selector);
		curr->selector = selector;

		/* Bind directly the file descriptors, we do not need to go
		   through xnselect returning -ECHRNG */
		err = __cobalt_select_bind_all(selector, in_fds, nfds);
		if (err)
			return err;
	}

	do {
		err = xnselect(selector, out_fds, in_fds, nfds, timeout, mode);
		if (err == -ECHRNG) {
			int err = __cobalt_select_bind_all(selector, out_fds, nfds);
			if (err)
				return err;
		}
	} while (err == -ECHRNG);

	if (u_tv && (err > 0 || err == -EINTR)) {
		diff = timeout - clock_get_ticks(CLOCK_MONOTONIC);
		if (diff > 0)
			ticks2tv(&tv, diff);
		else
			tv.tv_sec = tv.tv_usec = 0;

		if (sys32_put_timeval(u_tv, &tv))
			return -EFAULT;
	}

	if (err >= 0)
		for (i = 0; i < XNSELECT_MAX_TYPES; i++)
			if (ufd_sets[i] &&
			    sys32_put_fdset(ufd_sets[i], out_fds[i],
					    sizeof(fd_set)) < 0)
				return -EFAULT;
	return err;
}

COBALT_SYSCALL32emu(recvmsg, handover,
		    (int fd, struct compat_msghdr __user *umsg,
		     int flags))
{
	struct user_msghdr m;
	ssize_t ret;

	ret = sys32_get_msghdr(&m, umsg);
	if (ret)
		return ret;

	ret = rtdm_fd_recvmsg(fd, &m, flags);
	if (ret < 0)
		return ret;

	return sys32_put_msghdr(umsg, &m) ?: ret;
}

static int get_timespec32(struct timespec *ts,
			  const void __user *u_ts)
{
	return sys32_get_timespec(ts, u_ts);
}

static int get_mmsg32(struct mmsghdr *mmsg, void __user *u_mmsg)
{
	return sys32_get_mmsghdr(mmsg, u_mmsg);
}

static int put_mmsg32(void __user **u_mmsg_p, const struct mmsghdr *mmsg)
{
	struct compat_mmsghdr __user **p = (struct compat_mmsghdr **)u_mmsg_p,
		*q __user = (*p)++;

	return sys32_put_mmsghdr(q, mmsg);
}

COBALT_SYSCALL32emu(recvmmsg, primary,
	       (int ufd, struct compat_mmsghdr __user *u_msgvec, unsigned int vlen,
		unsigned int flags, struct compat_timespec *u_timeout))
{
	return __rtdm_fd_recvmmsg(ufd, u_msgvec, vlen, flags, u_timeout,
				  get_mmsg32, put_mmsg32,
				  get_timespec32);
}

COBALT_SYSCALL32emu(sendmsg, handover,
		    (int fd, struct compat_msghdr __user *umsg, int flags))
{
	struct user_msghdr m;
	int ret;

	ret = sys32_get_msghdr(&m, umsg);

	return ret ?: rtdm_fd_sendmsg(fd, &m, flags);
}

static int put_mmsglen32(void __user **u_mmsg_p, const struct mmsghdr *mmsg)
{
	struct compat_mmsghdr __user **p = (struct compat_mmsghdr **)u_mmsg_p,
		*q __user = (*p)++;

	return __xn_put_user(mmsg->msg_len, &q->msg_len);
}

COBALT_SYSCALL32emu(sendmmsg, primary,
		    (int fd, struct compat_mmsghdr __user *u_msgvec, unsigned int vlen,
		     unsigned int flags))
{
	return __rtdm_fd_sendmmsg(fd, u_msgvec, vlen, flags,
				  get_mmsg32, put_mmsglen32);
}

COBALT_SYSCALL32emu(mmap, lostage,
		    (int fd, struct compat_rtdm_mmap_request __user *u_crma,
		     compat_uptr_t __user *u_caddrp))
{
	struct _rtdm_mmap_request rma;
	compat_uptr_t u_caddr;
	void *u_addr = NULL;
	int ret;

	if (u_crma == NULL ||
	    !access_rok(u_crma, sizeof(*u_crma)) ||
	    __xn_get_user(rma.length, &u_crma->length) ||
	    __xn_get_user(rma.offset, &u_crma->offset) ||
	    __xn_get_user(rma.prot, &u_crma->prot) ||
	    __xn_get_user(rma.flags, &u_crma->flags))
	  return -EFAULT;

	ret = rtdm_fd_mmap(fd, &rma, &u_addr);
	if (ret)
		return ret;

	u_caddr = ptr_to_compat(u_addr);

	return cobalt_copy_to_user(u_caddrp, &u_caddr, sizeof(u_caddr));
}

COBALT_SYSCALL32emu(backtrace, current,
		    (int nr, compat_ulong_t __user *u_backtrace,
		     int reason))
{
	compat_ulong_t cbacktrace[SIGSHADOW_BACKTRACE_DEPTH];
	unsigned long backtrace[SIGSHADOW_BACKTRACE_DEPTH];
	int ret, n;

	if (nr <= 0)
		return 0;

	if (nr > SIGSHADOW_BACKTRACE_DEPTH)
		nr = SIGSHADOW_BACKTRACE_DEPTH;

	ret = cobalt_copy_from_user(cbacktrace, u_backtrace,
				       nr * sizeof(compat_ulong_t));
	if (ret)
		return ret;

	for (n = 0; n < nr; n++)
		backtrace [n] = cbacktrace[n];

	xndebug_trace_relax(nr, backtrace, reason);

	return 0;
}

#ifdef COBALT_SYSCALL32x

COBALT_SYSCALL32x(mq_timedreceive, primary,
		  (mqd_t uqd, void __user *u_buf,
		   compat_ssize_t __user *u_len,
		   unsigned int __user *u_prio,
		   const struct timespec __user *u_ts))
{
	compat_ssize_t clen;
	ssize_t len;
	int ret;

	ret = cobalt_copy_from_user(&clen, u_len, sizeof(*u_len));
	if (ret)
		return ret;

	len = clen;
	ret = __cobalt_mq_timedreceive(uqd, u_buf, &len, u_prio,
				       u_ts, u_ts ? mq_fetch_timeout : NULL);
	clen = len;

	return ret ?: cobalt_copy_to_user(u_len, &clen, sizeof(*u_len));
}

#endif /* COBALT_SYSCALL32x */
