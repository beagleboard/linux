/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2012 Regents of the University of California
 * Copyright (C) 2017 SiFive
 */

#ifndef _ASM_RISCV_MMU_CONTEXT_H
#define _ASM_RISCV_MMU_CONTEXT_H

#include <linux/mm_types.h>
#include <asm-generic/mm_hooks.h>

#include <linux/mm.h>
#include <linux/sched.h>
#include <asm/tlbflush.h>
#include <asm/cacheflush.h>
#include <asm/asid.h>

#define ASID_MASK		((1 << SATP_ASID_BITS) - 1)
#define cpu_asid(mm)		(atomic64_read(&mm->context.asid) & ASID_MASK)

#define init_new_context(tsk,mm)	({ atomic64_set(&(mm)->context.asid, 0); 0; })

static inline void enter_lazy_tlb(struct mm_struct *mm,
	struct task_struct *task)
{
}

static inline void destroy_context(struct mm_struct *mm)
{
}

void switch_mm(struct mm_struct *prev, struct mm_struct *next,
	struct task_struct *task);

void check_and_switch_context(struct mm_struct *mm, unsigned int cpu);

static inline void activate_mm(struct mm_struct *prev,
			       struct mm_struct *next)
{
	switch_mm(prev, next, NULL);
}

static inline void deactivate_mm(struct task_struct *task,
	struct mm_struct *mm)
{
}

#endif /* _ASM_RISCV_MMU_CONTEXT_H */
