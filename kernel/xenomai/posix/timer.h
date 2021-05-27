/*
 * Copyright (C) 2005 Philippe Gerum <rpm@xenomai.org>.
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
#ifndef _COBALT_POSIX_TIMER_H
#define _COBALT_POSIX_TIMER_H

#include <linux/types.h>
#include <linux/time.h>
#include <linux/list.h>
#include <cobalt/kernel/timer.h>
#include <xenomai/posix/signal.h>
#include <xenomai/posix/syscall.h>

struct cobalt_timer {
	struct xntimer timerbase;
	timer_t id;
	int overruns;
	clockid_t clockid;
	pid_t target;
	struct cobalt_sigpending sigp;
	struct cobalt_extref extref;
};

int cobalt_timer_deliver(timer_t timerid);

void cobalt_timer_reclaim(struct cobalt_process *p);

static inline timer_t cobalt_timer_id(const struct cobalt_timer *timer)
{
	return timer->id;
}

struct cobalt_timer *
cobalt_timer_by_id(struct cobalt_process *p, timer_t timer_id);

void cobalt_timer_handler(struct xntimer *xntimer);

void __cobalt_timer_getval(struct xntimer *__restrict__ timer, 
			   struct itimerspec *__restrict__ value);

int __cobalt_timer_setval(struct xntimer *__restrict__ timer, int clock_flag, 
			  const struct itimerspec *__restrict__ value);

int __cobalt_timer_create(clockid_t clock,
			  const struct sigevent *sev,
			  timer_t __user *u_tm);

int __cobalt_timer_settime(timer_t timerid, int flags,
			   const struct itimerspec *__restrict__ value,
			   struct itimerspec *__restrict__ ovalue);

int __cobalt_timer_gettime(timer_t timerid, struct itimerspec *value);

COBALT_SYSCALL_DECL(timer_create,
		    (clockid_t clock,
		     const struct sigevent __user *u_sev,
		     timer_t __user *u_tm));

COBALT_SYSCALL_DECL(timer_delete, (timer_t tm));

COBALT_SYSCALL_DECL(timer_settime,
		    (timer_t tm, int flags,
		     const struct itimerspec __user *u_newval,
		     struct itimerspec __user *u_oldval));

COBALT_SYSCALL_DECL(timer_gettime,
		    (timer_t tm, struct itimerspec __user *u_val));

COBALT_SYSCALL_DECL(timer_getoverrun, (timer_t tm));

#endif /* !_COBALT_POSIX_TIMER_H */
