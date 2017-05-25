/*
 * Copyright (C) 2013 Philippe Gerum <rpm@xenomai.org>.
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
#ifndef _COBALT_ASM_GENERIC_THREAD_H
#define _COBALT_ASM_GENERIC_THREAD_H

#include <asm/ptrace.h>
#include <asm/processor.h>

struct task_struct;

struct xnthread;
struct xnarchtcb;

struct xntcb {
	struct task_struct *host_task;
	struct thread_struct *tsp;
	struct mm_struct *mm;
	struct mm_struct *active_mm;
	struct thread_struct ts;
#ifdef CONFIG_XENO_ARCH_WANT_TIP
	struct thread_info *tip;
#endif
#ifdef CONFIG_XENO_ARCH_FPU
	struct task_struct *user_fpu_owner;
#endif
};

#endif /* !_COBALT_ASM_GENERIC_THREAD_H */
