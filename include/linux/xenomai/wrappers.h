/*
 * Copyright (C) 2017 Philippe Gerum <rpm@xenomai.org>.
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
 * along with Xenomai; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 * 02111-1307, USA.
 */
#ifndef _COBALT_LINUX_WRAPPERS_H
#define _COBALT_LINUX_WRAPPERS_H

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,11,0)
#include <linux/sched.h>
#include <linux/sched/rt.h>

#define cobalt_set_task_state(tsk, state_value)	\
	set_task_state(tsk, state_value)
#else
#include <linux/sched.h>
#include <linux/sched/signal.h>
#include <linux/sched/rt.h>
#include <linux/sched/mm.h>
#include <linux/sched/debug.h>
#include <linux/sched/task_stack.h>
#include <uapi/linux/sched/types.h>
/*
 * The co-kernel can still do this sanely for a thread which is
 * currently active on the head stage.
 */
#define cobalt_set_task_state(tsk, state_value)	\
		smp_store_mb((tsk)->state, (state_value))
#endif

#include <linux/ipipe.h>

#ifndef ipipe_root_nr_syscalls
#define ipipe_root_nr_syscalls(ti)	NR_syscalls
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 20, 0)
typedef siginfo_t kernel_siginfo_t;
#endif

#endif /* !_COBALT_LINUX_WRAPPERS_H */
