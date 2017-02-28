/*
 * arch/arm/include/asm/fcse.h
 *
 * Helper header for using the ARM Fast Context Switch Extension with
 * processors supporting it, lifted from the Fast Address Space
 * Switching (FASS) patch for ARM Linux.
 *
 * Copyright (C) 2001, 2002 Adam Wiggins <awiggins@cse.unsw.edu.au>
 * Copyright (C) 2007 Sebastian Smolorz <ssm@emlix.com>
 * Copyright (C) 2008 Richard Cochran
 * Copyright (C) 2009-2011 Gilles Chanteperdrix <gch@xenomai.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __ASM_ARM_FCSE_H
#define __ASM_ARM_FCSE_H

#ifdef CONFIG_ARM_FCSE_DEBUG
#define FCSE_BUG_ON(expr) BUG_ON(expr)
#else /* !CONFIG_ARM_FCSE_DEBUG */
#define FCSE_BUG_ON(expr) do { } while(0)
#endif /* !CONFIG_ARM_FCSE_DEBUG */

#ifdef CONFIG_ARM_FCSE

#include <linux/mm_types.h>	/* For struct mm_struct */
#include <linux/sched.h>
#include <linux/hardirq.h>

#include <asm/bitops.h>
#include <asm/cachetype.h>

#define FCSE_PID_SHIFT 25

/* Size of PID relocation area */
#define FCSE_PID_TASK_SIZE (1UL << FCSE_PID_SHIFT)

/* Mask to get rid of PID from relocated address */
#define FCSE_PID_MASK (FCSE_PID_TASK_SIZE - 1)

#define FCSE_PID_INVALID (~0 << FCSE_PID_SHIFT)

#define FCSE_NR_PIDS (TASK_SIZE / FCSE_PID_TASK_SIZE)
#define FCSE_PID_MAX (FCSE_NR_PIDS - 1)

struct vm_unmapped_area_info;

extern unsigned long fcse_pids_cache_dirty[];
#ifdef CONFIG_ARM_FCSE_BEST_EFFORT
extern struct mm_struct *fcse_large_process;
#endif

int fcse_pid_alloc(struct mm_struct *mm);
void fcse_pid_free(struct mm_struct *mm);
unsigned fcse_flush_all_start(void);
void fcse_flush_all_done(unsigned seq, unsigned dirty);
unsigned long
fcse_check_mmap_inner(struct mm_struct *mm,
		      struct vm_unmapped_area_info *info,
		      unsigned long addr, unsigned long flags);

/* Sets the CPU's PID Register */
static inline void fcse_pid_set(unsigned long pid)
{
	__asm__ __volatile__ ("mcr p15, 0, %0, c13, c0, 0"
			      : /* */: "r" (pid) : "cc", "memory");
}

static inline unsigned long fcse_pid_get(void)
{
	unsigned long pid;
	__asm__ __volatile__ ("mrc p15, 0, %0, c13, c0, 0"
			      : "=r"(pid) : /* */ : "cc", "memory");
	return pid;
}

static inline unsigned long fcse_mva_to_va(unsigned long mva)
{
	unsigned long va;

	if (!cache_is_vivt())
		return mva;

	va = fcse_pid_get() ^ mva;
	return (va & 0xfe000000) ? mva : va;
}

static inline unsigned long
fcse_va_to_mva(struct mm_struct *mm, unsigned long va)
{
	if (cache_is_vivt() && va < FCSE_PID_TASK_SIZE) {
		return mm->context.fcse.pid | va;
	}
	return va;
}


static inline unsigned long
fcse_check_mmap_addr(struct mm_struct *mm,
		     unsigned long addr, unsigned long len,
		     struct vm_unmapped_area_info *info, unsigned long flags)
{
	if ((addr & ~PAGE_MASK) == 0 && addr + len <= FCSE_TASK_SIZE)
		return addr;

	return fcse_check_mmap_inner(mm, info, addr, flags);
}

static inline void __fcse_mark_dirty(struct mm_struct *mm)
{
	__set_bit(FCSE_PID_MAX - (mm->context.fcse.pid >> FCSE_PID_SHIFT),
		fcse_pids_cache_dirty);
#ifdef CONFIG_ARM_FCSE_BEST_EFFORT
	if (mm->context.fcse.large)
		fcse_large_process = mm;
#endif
}

static inline void fcse_mark_dirty(struct mm_struct *mm)
{
	if (cache_is_vivt()) {
		set_bit(FCSE_PID_MAX - (mm->context.fcse.pid >> FCSE_PID_SHIFT),
			fcse_pids_cache_dirty);
#ifdef CONFIG_ARM_FCSE_BEST_EFFORT
		if (mm->context.fcse.large)
			fcse_large_process = mm;
#endif
	}
}

#ifdef CONFIG_ARM_FCSE_BEST_EFFORT
struct fcse_user {
	struct mm_struct *mm;
	unsigned count;
};
extern struct fcse_user fcse_pids_user[];
int fcse_switch_mm_start_inner(struct mm_struct *next);
void fcse_switch_mm_end_inner(struct mm_struct *next);
void fcse_pid_reference(struct mm_struct *mm);

static inline int fcse_switch_mm_start(struct mm_struct *next)
{
	if (!cache_is_vivt())
		return 0;

	return fcse_switch_mm_start_inner(next);
}

static inline void fcse_switch_mm_end(struct mm_struct *next)
{
	if (!cache_is_vivt())
		return;

	fcse_switch_mm_end_inner(next);
}

static inline int fcse_mm_in_cache(struct mm_struct *mm)
{
	unsigned fcse_pid = mm->context.fcse.pid >> FCSE_PID_SHIFT;
	int res;
	res = test_bit(FCSE_PID_MAX - fcse_pid, fcse_pids_cache_dirty)
		&& fcse_pids_user[fcse_pid].mm == mm;
	return res;
}
#else /* CONFIG_ARM_FCSE_GUARANTEED */
static inline int fcse_switch_mm_start(struct mm_struct *next)
{
	return 0;
}

static inline void fcse_switch_mm_end(struct mm_struct *next)
{
	if (!cache_is_vivt())
		return;

	fcse_mark_dirty(next);
	fcse_pid_set(next->context.fcse.pid);
}

static inline int fcse_mm_in_cache(struct mm_struct *mm)
{
	unsigned fcse_pid = mm->context.fcse.pid >> FCSE_PID_SHIFT;
	return test_bit(FCSE_PID_MAX - fcse_pid, fcse_pids_cache_dirty);
}
#endif /* CONFIG_ARM_FCSE_GUARANTEED */

#define fcse() (cache_is_vivt())
#else /* ! CONFIG_ARM_FCSE */
#define fcse_switch_mm_start(next) 1
#define fcse_switch_mm_end(next) do { (void)(next); } while(0)
#define fcse_mva_to_va(mva) (mva)
#define fcse_va_to_mva(mm, x) ({ (void)(mm); (x); })
#define fcse_mark_dirty(mm) do { (void)(mm); } while(0)
#define fcse_flush_all_start() (0)
#define fcse_flush_all_done(seq, dirty) do { (void)(seq); } while (0)
#define fcse_mm_in_cache(mm) \
		(cpumask_test_cpu(smp_processor_id(), mm_cpumask(mm)))
#define fcse_check_mmap_addr(mm, addr, len, info, flags) (addr)
#define fcse() (0)
#endif /* ! CONFIG_ARM_FCSE */

#ifdef CONFIG_ARM_FCSE_MESSAGES
void fcse_notify_segv(struct mm_struct *mm,
		      unsigned long addr, struct pt_regs *regs);
#else /* !FCSE_MESSAGES */
#define fcse_notify_segv(mm, addr, regs) do { } while(0)
#endif /* !FCSE_MESSAGES */

#endif /* __ASM_ARM_FCSE_H */
