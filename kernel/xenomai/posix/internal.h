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
#ifndef _COBALT_POSIX_INTERNAL_H
#define _COBALT_POSIX_INTERNAL_H

#include <cobalt/kernel/sched.h>
#include <cobalt/kernel/heap.h>
#include <cobalt/kernel/ppd.h>
#include <cobalt/kernel/assert.h>
#include <cobalt/kernel/list.h>
#include <cobalt/kernel/arith.h>
#include <asm/xenomai/syscall.h>
#include "process.h"
#include "extension.h"
#include "syscall.h"
#include "memory.h"

#define COBALT_MAXNAME		64
#define COBALT_PERMS_MASK	(O_RDONLY | O_WRONLY | O_RDWR)

#define COBALT_MAGIC(n)		(0x8686##n##n)
#define COBALT_ANY_MAGIC	COBALT_MAGIC(00)
#define COBALT_THREAD_MAGIC	COBALT_MAGIC(01)
#define COBALT_MQ_MAGIC		COBALT_MAGIC(0A)
#define COBALT_MQD_MAGIC	COBALT_MAGIC(0B)
#define COBALT_EVENT_MAGIC	COBALT_MAGIC(0F)
#define COBALT_MONITOR_MAGIC	COBALT_MAGIC(10)
#define COBALT_TIMERFD_MAGIC	COBALT_MAGIC(11)

#define cobalt_obj_active(h,m,t)	\
	((h) && ((t *)(h))->magic == (m))

#define cobalt_mark_deleted(t) ((t)->magic = ~(t)->magic)

static inline xnhandle_t cobalt_get_handle_from_user(xnhandle_t *u_h)
{
	xnhandle_t handle;
	return __xn_get_user(handle, u_h) ? 0 : handle;
}

int cobalt_init(void);

long cobalt_restart_syscall_placeholder(struct restart_block *param);

#endif /* !_COBALT_POSIX_INTERNAL_H */
