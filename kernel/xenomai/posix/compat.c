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
#include <linux/err.h>
#include <linux/module.h>
#include <cobalt/kernel/compat.h>
#include <asm/xenomai/syscall.h>
#include <xenomai/posix/mqueue.h>

int sys32_get_timespec(struct timespec *ts,
		       const struct compat_timespec __user *cts)
{
	return (cts == NULL ||
		!access_rok(cts, sizeof(*cts)) ||
		__xn_get_user(ts->tv_sec, &cts->tv_sec) ||
		__xn_get_user(ts->tv_nsec, &cts->tv_nsec)) ? -EFAULT : 0;
}
EXPORT_SYMBOL_GPL(sys32_get_timespec);

int sys32_put_timespec(struct compat_timespec __user *cts,
		       const struct timespec *ts)
{
	return (cts == NULL ||
		!access_wok(cts, sizeof(*cts)) ||
		__xn_put_user(ts->tv_sec, &cts->tv_sec) ||
		__xn_put_user(ts->tv_nsec, &cts->tv_nsec)) ? -EFAULT : 0;
}
EXPORT_SYMBOL_GPL(sys32_put_timespec);

int sys32_get_itimerspec(struct itimerspec *its,
			 const struct compat_itimerspec __user *cits)
{
	int ret = sys32_get_timespec(&its->it_value, &cits->it_value);

	return ret ?: sys32_get_timespec(&its->it_interval, &cits->it_interval);
}
EXPORT_SYMBOL_GPL(sys32_get_itimerspec);

int sys32_put_itimerspec(struct compat_itimerspec __user *cits,
			 const struct itimerspec *its)
{
	int ret = sys32_put_timespec(&cits->it_value, &its->it_value);

	return ret ?: sys32_put_timespec(&cits->it_interval, &its->it_interval);
}
EXPORT_SYMBOL_GPL(sys32_put_itimerspec);

int sys32_get_timeval(struct timeval *tv,
		      const struct compat_timeval __user *ctv)
{
	return (ctv == NULL ||
		!access_rok(ctv, sizeof(*ctv)) ||
		__xn_get_user(tv->tv_sec, &ctv->tv_sec) ||
		__xn_get_user(tv->tv_usec, &ctv->tv_usec)) ? -EFAULT : 0;
}
EXPORT_SYMBOL_GPL(sys32_get_timeval);

int sys32_put_timeval(struct compat_timeval __user *ctv,
		      const struct timeval *tv)
{
	return (ctv == NULL ||
		!access_wok(ctv, sizeof(*ctv)) ||
		__xn_put_user(tv->tv_sec, &ctv->tv_sec) ||
		__xn_put_user(tv->tv_usec, &ctv->tv_usec)) ? -EFAULT : 0;
}
EXPORT_SYMBOL_GPL(sys32_put_timeval);

ssize_t sys32_get_fdset(fd_set *fds, const compat_fd_set __user *cfds,
			size_t cfdsize)
{
	int rdpos, wrpos, rdlim = cfdsize / sizeof(compat_ulong_t);

	if (cfds == NULL || !access_rok(cfds, cfdsize))
		return -EFAULT;

	for (rdpos = 0, wrpos = 0; rdpos < rdlim; rdpos++, wrpos++)
		if (__xn_get_user(fds->fds_bits[wrpos], cfds->fds_bits + rdpos))
			return -EFAULT;

	return (ssize_t)rdlim * sizeof(long);
}
EXPORT_SYMBOL_GPL(sys32_get_fdset);

ssize_t sys32_put_fdset(compat_fd_set __user *cfds, const fd_set *fds,
			size_t fdsize)
{
	int rdpos, wrpos, wrlim = fdsize / sizeof(long);

	if (cfds == NULL || !access_wok(cfds, wrlim * sizeof(compat_ulong_t)))
		return -EFAULT;

	for (rdpos = 0, wrpos = 0; wrpos < wrlim; rdpos++, wrpos++)
		if (__xn_put_user(fds->fds_bits[rdpos], cfds->fds_bits + wrpos))
			return -EFAULT;

	return (ssize_t)wrlim * sizeof(compat_ulong_t);
}
EXPORT_SYMBOL_GPL(sys32_put_fdset);

int sys32_get_param_ex(int policy,
		       struct sched_param_ex *p,
		       const struct compat_sched_param_ex __user *u_cp)
{
	struct compat_sched_param_ex cpex;

	if (u_cp == NULL || cobalt_copy_from_user(&cpex, u_cp, sizeof(cpex)))
		return -EFAULT;

	p->sched_priority = cpex.sched_priority;

	switch (policy) {
	case SCHED_SPORADIC:
		p->sched_ss_low_priority = cpex.sched_ss_low_priority;
		p->sched_ss_max_repl = cpex.sched_ss_max_repl;
		p->sched_ss_repl_period.tv_sec = cpex.sched_ss_repl_period.tv_sec;
		p->sched_ss_repl_period.tv_nsec = cpex.sched_ss_repl_period.tv_nsec;
		p->sched_ss_init_budget.tv_sec = cpex.sched_ss_init_budget.tv_sec;
		p->sched_ss_init_budget.tv_nsec = cpex.sched_ss_init_budget.tv_nsec;
		break;
	case SCHED_RR:
		p->sched_rr_quantum.tv_sec = cpex.sched_rr_quantum.tv_sec;
		p->sched_rr_quantum.tv_nsec = cpex.sched_rr_quantum.tv_nsec;
		break;
	case SCHED_TP:
		p->sched_tp_partition = cpex.sched_tp_partition;
		break;
	case SCHED_QUOTA:
		p->sched_quota_group = cpex.sched_quota_group;
		break;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(sys32_get_param_ex);

int sys32_put_param_ex(int policy,
		       struct compat_sched_param_ex __user *u_cp,
		       const struct sched_param_ex *p)
{
	struct compat_sched_param_ex cpex;

	if (u_cp == NULL)
		return -EFAULT;

	cpex.sched_priority = p->sched_priority;

	switch (policy) {
	case SCHED_SPORADIC:
		cpex.sched_ss_low_priority = p->sched_ss_low_priority;
		cpex.sched_ss_max_repl = p->sched_ss_max_repl;
		cpex.sched_ss_repl_period.tv_sec = p->sched_ss_repl_period.tv_sec;
		cpex.sched_ss_repl_period.tv_nsec = p->sched_ss_repl_period.tv_nsec;
		cpex.sched_ss_init_budget.tv_sec = p->sched_ss_init_budget.tv_sec;
		cpex.sched_ss_init_budget.tv_nsec = p->sched_ss_init_budget.tv_nsec;
		break;
	case SCHED_RR:
		cpex.sched_rr_quantum.tv_sec = p->sched_rr_quantum.tv_sec;
		cpex.sched_rr_quantum.tv_nsec = p->sched_rr_quantum.tv_nsec;
		break;
	case SCHED_TP:
		cpex.sched_tp_partition = p->sched_tp_partition;
		break;
	case SCHED_QUOTA:
		cpex.sched_quota_group = p->sched_quota_group;
		break;
	}

	return cobalt_copy_to_user(u_cp, &cpex, sizeof(cpex));
}
EXPORT_SYMBOL_GPL(sys32_put_param_ex);

int sys32_get_mqattr(struct mq_attr *ap,
		     const struct compat_mq_attr __user *u_cap)
{
	struct compat_mq_attr cattr;

	if (u_cap == NULL ||
	    cobalt_copy_from_user(&cattr, u_cap, sizeof(cattr)))
		return -EFAULT;

	ap->mq_flags = cattr.mq_flags;
	ap->mq_maxmsg = cattr.mq_maxmsg;
	ap->mq_msgsize = cattr.mq_msgsize;
	ap->mq_curmsgs = cattr.mq_curmsgs;

	return 0;
}
EXPORT_SYMBOL_GPL(sys32_get_mqattr);

int sys32_put_mqattr(struct compat_mq_attr __user *u_cap,
		     const struct mq_attr *ap)
{
	struct compat_mq_attr cattr;

	cattr.mq_flags = ap->mq_flags;
	cattr.mq_maxmsg = ap->mq_maxmsg;
	cattr.mq_msgsize = ap->mq_msgsize;
	cattr.mq_curmsgs = ap->mq_curmsgs;

	return u_cap == NULL ? -EFAULT :
		cobalt_copy_to_user(u_cap, &cattr, sizeof(cattr));
}
EXPORT_SYMBOL_GPL(sys32_put_mqattr);

int sys32_get_sigevent(struct sigevent *ev,
		       const struct compat_sigevent *__user u_cev)
{
	struct compat_sigevent cev;
	compat_int_t *cp;
	int ret, *p;

	if (u_cev == NULL)
		return -EFAULT;

	ret = cobalt_copy_from_user(&cev, u_cev, sizeof(cev));
	if (ret)
		return ret;

	memset(ev, 0, sizeof(*ev));
	ev->sigev_value.sival_ptr = compat_ptr(cev.sigev_value.sival_ptr);
	ev->sigev_signo = cev.sigev_signo;
	ev->sigev_notify = cev.sigev_notify;
	/*
	 * Extensions may define extra fields we don't know about in
	 * the padding area, so we have to load it entirely.
	 */
	p = ev->_sigev_un._pad;
	cp = cev._sigev_un._pad;
	while (p < &ev->_sigev_un._pad[ARRAY_SIZE(ev->_sigev_un._pad)] &&
	       cp < &cev._sigev_un._pad[ARRAY_SIZE(cev._sigev_un._pad)])
		*p++ = *cp++;

	return 0;
}
EXPORT_SYMBOL_GPL(sys32_get_sigevent);

int sys32_get_sigset(sigset_t *set, const compat_sigset_t *u_cset)
{
	compat_sigset_t cset;
	int ret;

	if (u_cset == NULL)
		return -EFAULT;

	ret = cobalt_copy_from_user(&cset, u_cset, sizeof(cset));
	if (ret)
		return ret;

	sigset_from_compat(set, &cset);

	return 0;
}
EXPORT_SYMBOL_GPL(sys32_get_sigset);

int sys32_put_sigset(compat_sigset_t *u_cset, const sigset_t *set)
{
	compat_sigset_t cset;

	if (u_cset == NULL)
		return -EFAULT;

	sigset_to_compat(&cset, set);

	return cobalt_copy_to_user(u_cset, &cset, sizeof(cset));
}
EXPORT_SYMBOL_GPL(sys32_put_sigset);

int sys32_get_sigval(union sigval *val, const union compat_sigval *u_cval)
{
	union compat_sigval cval;
	int ret;

	if (u_cval == NULL)
		return -EFAULT;

	ret = cobalt_copy_from_user(&cval, u_cval, sizeof(cval));
	if (ret)
		return ret;

	val->sival_ptr = compat_ptr(cval.sival_ptr);

	return 0;
}
EXPORT_SYMBOL_GPL(sys32_get_sigval);

int sys32_put_siginfo(void __user *u_si, const struct siginfo *si,
		      int overrun)
{
	struct compat_siginfo __user *u_p = u_si;
	int code, ret;

	if (u_p == NULL)
		return -EFAULT;

	/* Translate kernel codes for userland. */
	code = si->si_code;
	if (code & __SI_MASK)
		code |= __SI_MASK;

	ret = __xn_put_user(si->si_signo, &u_p->si_signo);
	ret |= __xn_put_user(si->si_errno, &u_p->si_errno);
	ret |= __xn_put_user(code, &u_p->si_code);

	/*
	 * Copy the generic/standard siginfo bits to userland.
	 */
	switch (si->si_code) {
	case SI_TIMER:
		ret |= __xn_put_user(si->si_tid, &u_p->si_tid);
		ret |= __xn_put_user(ptr_to_compat(si->si_ptr), &u_p->si_ptr);
		ret |= __xn_put_user(overrun, &u_p->si_overrun);
		break;
	case SI_QUEUE:
	case SI_MESGQ:
		ret |= __xn_put_user(ptr_to_compat(si->si_ptr), &u_p->si_ptr);
		/* falldown wanted. */
	case SI_USER:
		ret |= __xn_put_user(si->si_pid, &u_p->si_pid);
		ret |= __xn_put_user(si->si_uid, &u_p->si_uid);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(sys32_put_siginfo);

int sys32_get_msghdr(struct user_msghdr *msg,
		     const struct compat_msghdr __user *u_cmsg)
{
	compat_uptr_t tmp1, tmp2, tmp3;

	if (u_cmsg == NULL ||
	    !access_rok(u_cmsg, sizeof(*u_cmsg)) ||
	    __xn_get_user(tmp1, &u_cmsg->msg_name) ||
	    __xn_get_user(msg->msg_namelen, &u_cmsg->msg_namelen) ||
	    __xn_get_user(tmp2, &u_cmsg->msg_iov) ||
	    __xn_get_user(msg->msg_iovlen, &u_cmsg->msg_iovlen) ||
	    __xn_get_user(tmp3, &u_cmsg->msg_control) ||
	    __xn_get_user(msg->msg_controllen, &u_cmsg->msg_controllen) ||
	    __xn_get_user(msg->msg_flags, &u_cmsg->msg_flags))
		return -EFAULT;

	if (msg->msg_namelen > sizeof(struct sockaddr_storage))
		msg->msg_namelen = sizeof(struct sockaddr_storage);

	msg->msg_name = compat_ptr(tmp1);
	msg->msg_iov = compat_ptr(tmp2);
	msg->msg_control = compat_ptr(tmp3);

	return 0;
}
EXPORT_SYMBOL_GPL(sys32_get_msghdr);

int sys32_put_msghdr(struct compat_msghdr __user *u_cmsg,
		     const struct user_msghdr *msg)
{
	if (u_cmsg == NULL ||
	    !access_wok(u_cmsg, sizeof(*u_cmsg)) ||
	    __xn_put_user(ptr_to_compat(msg->msg_name), &u_cmsg->msg_name) ||
	    __xn_put_user(msg->msg_namelen, &u_cmsg->msg_namelen) ||
	    __xn_put_user(ptr_to_compat(msg->msg_iov), &u_cmsg->msg_iov) ||
	    __xn_put_user(msg->msg_iovlen, &u_cmsg->msg_iovlen) ||
	    __xn_put_user(ptr_to_compat(msg->msg_control), &u_cmsg->msg_control) ||
	    __xn_put_user(msg->msg_controllen, &u_cmsg->msg_controllen) ||
	    __xn_put_user(msg->msg_flags, &u_cmsg->msg_flags))
		return -EFAULT;

	return 0;
}
EXPORT_SYMBOL_GPL(sys32_put_msghdr);

int sys32_get_iovec(struct iovec *iov,
		    const struct compat_iovec __user *u_ciov,
		    int ciovlen)
{
	const struct compat_iovec __user *p;
	struct compat_iovec ciov;
	int ret, n;
	
	for (n = 0, p = u_ciov; n < ciovlen; n++, p++) {
		ret = cobalt_copy_from_user(&ciov, p, sizeof(ciov));
		if (ret)
			return ret;
		iov[n].iov_base = compat_ptr(ciov.iov_base);
		iov[n].iov_len = ciov.iov_len;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(sys32_get_iovec);

int sys32_put_iovec(struct compat_iovec __user *u_ciov,
		    const struct iovec *iov,
		    int iovlen)
{
	struct compat_iovec __user *p;
	struct compat_iovec ciov;
	int ret, n;
	
	for (n = 0, p = u_ciov; n < iovlen; n++, p++) {
		ciov.iov_base = ptr_to_compat(iov[n].iov_base);
		ciov.iov_len = iov[n].iov_len;
		ret = cobalt_copy_to_user(p, &ciov, sizeof(*p));
		if (ret)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(sys32_put_iovec);
