/*
 * Copyright (C) 2013 Philippe Gerum <rpm@xenomai.org>.
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
#ifndef _COBALT_POSIX_SIGNAL_H
#define _COBALT_POSIX_SIGNAL_H

#include <linux/signal.h>
#include <cobalt/kernel/timer.h>
#include <cobalt/kernel/list.h>
#include <cobalt/uapi/signal.h>
#include <xenomai/posix/syscall.h>

struct cobalt_thread;

struct cobalt_sigpending {
	struct siginfo si;
	struct list_head next;
};

static inline
void cobalt_copy_siginfo(int code,
			 struct siginfo *__restrict__ dst,
			 const struct siginfo *__restrict__ src)
{
	dst->si_signo = src->si_signo;
	dst->si_errno = src->si_errno;
	dst->si_code = code;

	switch (code) {
	case SI_TIMER:
		dst->si_tid = src->si_tid;
		dst->si_overrun = src->si_overrun;
		dst->si_value = src->si_value;
		break;
	case SI_QUEUE:
	case SI_MESGQ:
		dst->si_value = src->si_value;
		/* falldown wanted. */
	case SI_USER:
		dst->si_pid = src->si_pid;
		dst->si_uid = src->si_uid;
	}
}

int __cobalt_sigwait(sigset_t *set);

int __cobalt_sigtimedwait(sigset_t *set,
			  const struct timespec *timeout,
			  void __user *u_si,
			  int (*put_siginfo)(void __user *u_si,
					     const struct siginfo *si,
					     int overrun));

int __cobalt_sigwaitinfo(sigset_t *set,
			 void __user *u_si,
			 int (*put_siginfo)(void __user *u_si,
					    const struct siginfo *si,
					    int overrun));

int __cobalt_sigqueue(pid_t pid, int sig, const union sigval *value);

int cobalt_signal_send(struct cobalt_thread *thread,
		       struct cobalt_sigpending *sigp,
		       int group);

int cobalt_signal_send_pid(pid_t pid,
			   struct cobalt_sigpending *sigp);

struct cobalt_sigpending *cobalt_signal_alloc(void);

void cobalt_signal_free(struct cobalt_sigpending *sigp);

void cobalt_signal_flush(struct cobalt_thread *thread);

int cobalt_signal_wait(sigset_t *set, struct siginfo *si,
		       xnticks_t timeout, xntmode_t tmode);

int __cobalt_kill(struct cobalt_thread *thread,
		  int sig, int group);

COBALT_SYSCALL_DECL(sigwait,
		    (const sigset_t __user *u_set, int __user *u_sig));

COBALT_SYSCALL_DECL(sigtimedwait,
		    (const sigset_t __user *u_set,
		     struct siginfo __user *u_si,
		     const struct timespec __user *u_timeout));

COBALT_SYSCALL_DECL(sigwaitinfo,
		    (const sigset_t __user *u_set,
		     struct siginfo __user *u_si));

COBALT_SYSCALL_DECL(sigpending,
		    (old_sigset_t __user *u_set));

COBALT_SYSCALL_DECL(kill, (pid_t pid, int sig));

COBALT_SYSCALL_DECL(sigqueue,
		    (pid_t pid, int sig, const union sigval __user *u_value));

int cobalt_signal_init(void);

#endif /* !_COBALT_POSIX_SIGNAL_H */
