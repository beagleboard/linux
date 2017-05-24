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

#ifndef _COBALT_POSIX_MUTEX_H
#define _COBALT_POSIX_MUTEX_H

#include "thread.h"
#include <cobalt/uapi/mutex.h>
#include <xenomai/posix/syscall.h>
#include <xenomai/posix/process.h>

struct cobalt_process;

struct cobalt_mutex {
	unsigned int magic;
	struct xnsynch synchbase;
	/** cobalt_mutexq */
	struct list_head conds;
	struct cobalt_mutexattr attr;
	struct cobalt_resnode resnode;
};

int __cobalt_mutex_timedlock_break(struct cobalt_mutex_shadow __user *u_mx,
				   const void __user *u_ts,
				   int (*fetch_timeout)(struct timespec *ts,
							const void __user *u_ts));

int __cobalt_mutex_acquire_unchecked(struct xnthread *cur,
				     struct cobalt_mutex *mutex,
				     const struct timespec *ts);

COBALT_SYSCALL_DECL(mutex_check_init,
		    (struct cobalt_mutex_shadow __user *u_mx));

COBALT_SYSCALL_DECL(mutex_init,
		    (struct cobalt_mutex_shadow __user *u_mx,
		     const struct cobalt_mutexattr __user *u_attr));

COBALT_SYSCALL_DECL(mutex_destroy,
		    (struct cobalt_mutex_shadow __user *u_mx));

COBALT_SYSCALL_DECL(mutex_trylock,
		    (struct cobalt_mutex_shadow __user *u_mx));

COBALT_SYSCALL_DECL(mutex_lock,
		    (struct cobalt_mutex_shadow __user *u_mx));

COBALT_SYSCALL_DECL(mutex_timedlock,
		    (struct cobalt_mutex_shadow __user *u_mx,
		     const struct timespec __user *u_ts));

COBALT_SYSCALL_DECL(mutex_unlock,
		    (struct cobalt_mutex_shadow __user *u_mx));

int cobalt_mutex_release(struct xnthread *cur,
			 struct cobalt_mutex *mutex);

void cobalt_mutex_reclaim(struct cobalt_resnode *node,
			  spl_t s);

#endif /* !_COBALT_POSIX_MUTEX_H */
