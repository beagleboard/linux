/*
 * Copyright (C) 2014 Gilles Chanteperdrix <gilles.chanteperdrix@xenomai.org>.
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
#ifndef TIMERFD_H
#define TIMERFD_H

#include <linux/time.h>
#include <xenomai/posix/syscall.h>

int __cobalt_timerfd_settime(int fd, int flags,
			     const struct itimerspec *new_value,
			     struct itimerspec *old_value);

int __cobalt_timerfd_gettime(int fd,
			     struct itimerspec *value);

COBALT_SYSCALL_DECL(timerfd_create,
		    (int clockid, int flags));

COBALT_SYSCALL_DECL(timerfd_settime,
		    (int fd, int flags,
		     const struct itimerspec __user *new_value,
		     struct itimerspec __user *old_value));

COBALT_SYSCALL_DECL(timerfd_gettime,
		    (int fd, struct itimerspec __user *curr_value));

#endif /* TIMERFD_H */
