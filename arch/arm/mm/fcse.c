/*
 * arch/arm/kernel/fcse.c
 *
 * Helper functions for using the ARM Fast Context Switch Extension with
 * processors supporting it.
 *
 * Copyright (C) 2008 Richard Cochran
 * Copyright (C) 2009-2011 Gilles Chanteperdrix <gch@xenomai.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/bitops.h>
#include <linux/memory.h>
#include <linux/spinlock.h>
#include <linux/mm.h>
#include <linux/kernel_stat.h>
#include <linux/mman.h>
#include <linux/dcache.h>
#include <linux/fs.h>
#include <linux/hardirq.h>
#include <linux/export.h>

#include <asm/fcse.h>
#include <asm/cacheflush.h>
#include <asm/tlbflush.h>
#include <asm/system_misc.h>	/* For user_debug, UDBG_SEGV */

#define PIDS_LONGS ((FCSE_NR_PIDS + BITS_PER_LONG - 1) / BITS_PER_LONG)

static IPIPE_DEFINE_RAW_SPINLOCK(fcse_lock);
static unsigned long fcse_pids_bits[PIDS_LONGS];
unsigned long fcse_pids_cache_dirty[PIDS_LONGS];
EXPORT_SYMBOL(fcse_pids_cache_dirty);

#ifdef CONFIG_ARM_FCSE_BEST_EFFORT
static unsigned random_pid;
struct mm_struct *fcse_large_process;
struct fcse_user fcse_pids_user[FCSE_NR_PIDS];
static struct mm_struct *fcse_cur_mm = &init_mm;
#endif /* CONFIG_ARM_FCSE_BEST_EFFORT */

static inline void fcse_pid_reference_inner(struct mm_struct *mm)
{
	unsigned fcse_pid = mm->context.fcse.pid >> FCSE_PID_SHIFT;

#ifdef CONFIG_ARM_FCSE_BEST_EFFORT
	if (++fcse_pids_user[fcse_pid].count == 1)
#endif /* CONFIG_ARM_FCSE_BEST_EFFORT */
		__set_bit(FCSE_PID_MAX - fcse_pid, fcse_pids_bits);
}

static inline void fcse_pid_dereference(struct mm_struct *mm)
{
	unsigned fcse_pid = mm->context.fcse.pid >> FCSE_PID_SHIFT;

#ifdef CONFIG_ARM_FCSE_BEST_EFFORT
	if (--fcse_pids_user[fcse_pid].count == 0)
		__clear_bit(FCSE_PID_MAX - fcse_pid, fcse_pids_bits);

	/*
	 * The following means we suppose that by the time this
	 * function is called, this mm is out of cache:
	 * - when the caller is destroy_context, exit_mmap is called
	 * by mmput before, which flushes the cache;
	 * - when the caller is fcse_relocate_mm_to_pid from
	 * fcse_switch_mm_inner, we only relocate when the mm is out
	 * of cache;
	 * - when the caller is fcse_relocate_mm_to_pid from
	 * fcse_relocate_mm_to_null_pid, we flush the cache in this
	 * function.
	 */
	if (fcse_pids_user[fcse_pid].mm == mm) {
		fcse_pids_user[fcse_pid].mm = NULL;
		__clear_bit(FCSE_PID_MAX - fcse_pid, fcse_pids_cache_dirty);
	}
	if (fcse_large_process == mm)
		fcse_large_process = NULL;
#else /* CONFIG_ARM_FCSE_BEST_EFFORT */
	__clear_bit(FCSE_PID_MAX - fcse_pid, fcse_pids_bits);
	__clear_bit(FCSE_PID_MAX - fcse_pid, fcse_pids_cache_dirty);
#endif /* CONFIG_ARM_FCSE_BEST_EFFORT */
}

static inline long find_free_pid(unsigned long bits[])
{
	return FCSE_PID_MAX - find_first_zero_bit(bits, FCSE_NR_PIDS);
}

void fcse_pid_free(struct mm_struct *mm)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&fcse_lock, flags);
	fcse_pid_dereference(mm);
	raw_spin_unlock_irqrestore(&fcse_lock, flags);
}

int fcse_pid_alloc(struct mm_struct *mm)
{
	unsigned long flags;
	unsigned fcse_pid;

	raw_spin_lock_irqsave(&fcse_lock, flags);
	fcse_pid = find_free_pid(fcse_pids_bits);
	if (fcse_pid == -1) {
		/* Allocate zero pid last, since zero pid is also used by
		   processes with address space larger than 32MB in
		   best-effort mode. */
#ifdef CONFIG_ARM_FCSE_BEST_EFFORT
		if(++random_pid == FCSE_NR_PIDS) {
			if (fcse_large_process) {
				random_pid =
					fcse_large_process->context.fcse.highest_pid + 1;
				if (random_pid == FCSE_NR_PIDS)
					random_pid = 0;
			} else
				random_pid = 0;
		}
		fcse_pid = random_pid;
#else /* CONFIG_ARM_FCSE_GUARANTEED */
		raw_spin_unlock_irqrestore(&fcse_lock, flags);
#ifdef CONFIG_ARM_FCSE_MESSAGES
		printk(KERN_WARNING "FCSE: %s[%d] would exceed the %lu processes limit.\n",
		       current->comm, current->pid, FCSE_NR_PIDS);
#endif /* CONFIG_ARM_FCSE_MESSAGES */
		/*
		 * Set mm pid to FCSE_PID_INVALID, as even when
		 * init_new_context fails, destroy_context is called.
		 */
		mm->context.fcse.pid = FCSE_PID_INVALID;
		return -EAGAIN;
#endif /* CONFIG_ARM_FCSE_GUARANTEED */
	}
	mm->context.fcse.pid = fcse_pid << FCSE_PID_SHIFT;
	fcse_pid_reference_inner(mm);
	raw_spin_unlock_irqrestore(&fcse_lock, flags);

	return 0;
}

static inline void fcse_clear_dirty_all(void)
{
	switch(ARRAY_SIZE(fcse_pids_cache_dirty)) {
	case 3:
		fcse_pids_cache_dirty[2] = 0UL;
	case 2:
		fcse_pids_cache_dirty[1] = 0UL;
	case 1:
		fcse_pids_cache_dirty[0] = 0UL;
	}
#ifdef CONFIG_ARM_FCSE_BEST_EFFORT
	fcse_large_process = NULL;
#endif
}

unsigned fcse_flush_all_start(void)
{
	if (!cache_is_vivt())
		return 0;

#ifndef CONFIG_ARM_FCSE_PREEMPT_FLUSH
	preempt_disable();
#endif /* CONFIG_ARM_FCSE_PREEMPT_FLUSH */

#if defined(CONFIG_IPIPE)
	clear_ti_thread_flag(current_thread_info(), TIF_SWITCHED);
#elif defined(CONFIG_ARM_FCSE_PREEMPT_FLUSH)
	return nr_context_switches();
#endif /* CONFIG_ARM_FCSE_PREEMPT_FLUSH */

	return 0;
}

noinline void
fcse_flush_all_done(unsigned seq, unsigned dirty)
{
	struct mm_struct *mm;
	unsigned long flags;

	if (!cache_is_vivt())
		return;

	mm = current->mm;

	raw_spin_lock_irqsave(&fcse_lock, flags);
#if defined(CONFIG_IPIPE)
	if (!test_ti_thread_flag(current_thread_info(), TIF_SWITCHED))
#elif defined(CONFIG_ARM_FCSE_PREEMPT_FLUSH)
	if (seq == nr_context_switches())
#endif /* CONFIG_ARM_FCSE_PREEMPT_FLUSH */
		fcse_clear_dirty_all();

	if (dirty && mm != &init_mm && mm)
		__fcse_mark_dirty(mm);
	raw_spin_unlock_irqrestore(&fcse_lock, flags);
#ifndef CONFIG_ARM_FCSE_PREEMPT_FLUSH
	preempt_enable();
#endif /* CONFIG_ARM_FCSE_PREEMPT_FLUSH */
}

#ifdef CONFIG_ARM_FCSE_BEST_EFFORT
/* Called with preemption disabled, mm->mmap_sem being held for writing. */
static noinline int fcse_relocate_mm_to_pid(struct mm_struct *mm, int fcse_pid)
{
	const unsigned len = pgd_index(FCSE_TASK_SIZE) * sizeof(pgd_t);
	unsigned long flags;
	pgd_t *from, *to;

	raw_spin_lock_irqsave(&fcse_lock, flags);
	fcse_pid_dereference(mm);
	from = pgd_offset(mm, 0);
	mm->context.fcse.pid = fcse_pid << FCSE_PID_SHIFT;
	to = pgd_offset(mm, 0);
	fcse_pid_reference_inner(mm);
	fcse_pids_user[fcse_pid].mm = mm;
	__fcse_mark_dirty(mm);
	raw_spin_unlock_irqrestore(&fcse_lock, flags);

	memcpy(to, from, len);
	memset(from, '\0', len);
	barrier();
	clean_dcache_area(from, len);
	clean_dcache_area(to, len);

	return fcse_pid;
}

static int fcse_flush_needed_p(struct mm_struct *next)
{
	unsigned fcse_pid = next->context.fcse.pid >> FCSE_PID_SHIFT;
	unsigned flush_needed = 0;

	if (next == &init_mm)
		goto check_cur;

	if (fcse_pids_user[fcse_pid].mm != next)
		if (fcse_pids_user[fcse_pid].mm)
			flush_needed = test_bit(FCSE_PID_MAX - fcse_pid,
						fcse_pids_cache_dirty);

	if (flush_needed == 0
	    && fcse_large_process
	    && fcse_large_process != next
	    && fcse_pid <= fcse_large_process->context.fcse.highest_pid)
		flush_needed = 1;

  check_cur:
	if (flush_needed == 0 && fcse_cur_mm->context.fcse.shared_dirty_pages)
		flush_needed = 1;

	return flush_needed;
}

int fcse_switch_mm_start_inner(struct mm_struct *next)
{
	unsigned flush_needed;
	unsigned long flags;

	raw_spin_lock_irqsave(&fcse_lock, flags);
	flush_needed = fcse_flush_needed_p(next);
	raw_spin_unlock_irqrestore(&fcse_lock, flags);

	return flush_needed;
}
EXPORT_SYMBOL_GPL(fcse_switch_mm_start_inner);

void fcse_switch_mm_end_inner(struct mm_struct *next)
{
	unsigned fcse_pid = next->context.fcse.pid >> FCSE_PID_SHIFT;
	unsigned long flags;

	raw_spin_lock_irqsave(&fcse_lock, flags);
	if (fcse_flush_needed_p(next))
		fcse_clear_dirty_all();

	fcse_pid_set(fcse_pid << FCSE_PID_SHIFT);
	if (next != &init_mm) {
		__fcse_mark_dirty(next);
		if (fcse_pids_user[fcse_pid].mm != next)
			fcse_pids_user[fcse_pid].mm = next;
	}
	fcse_cur_mm = next;
	raw_spin_unlock_irqrestore(&fcse_lock, flags);
}
EXPORT_SYMBOL_GPL(fcse_switch_mm_end_inner);

void fcse_pid_reference(struct mm_struct *mm)
{
	unsigned long flags;

	raw_spin_lock_irqsave(&fcse_lock, flags);
	fcse_pid_reference_inner(mm);
	raw_spin_unlock_irqrestore(&fcse_lock, flags);
}

/* Called with mm->mmap_sem write-locked. */
static noinline void fcse_relocate_mm_to_null_pid(struct mm_struct *mm)
{
	if (!cache_is_vivt())
		return;

	preempt_disable();
	while (fcse_mm_in_cache(mm)) {
		unsigned seq;

		preempt_enable();

		seq = fcse_flush_all_start();
		flush_cache_all();

		preempt_disable();
		fcse_flush_all_done(seq, 0);
	}

	fcse_relocate_mm_to_pid(mm, 0);
	barrier();
	flush_tlb_mm(mm);
	fcse_pid_set(0);

	preempt_enable();
}
#endif /* CONFIG_ARM_FCSE_BEST_EFFORT */

unsigned long
fcse_check_mmap_inner(struct mm_struct *mm,
		      struct vm_unmapped_area_info *info,
		      unsigned long addr, unsigned long flags)
{
	if (flags & MAP_FIXED)
		goto skip_retry;

	/* Try again the mmap, allowing addresses above 32 MB */
	info->flags = 0;
	info->low_limit = PAGE_ALIGN(mm->start_stack);
#ifdef CONFIG_ARM_FCSE_BEST_EFFORT
	info->high_limit = TASK_SIZE;
#else /* CONFIG_ARM_FCSE_GUARANTEED */
	info->high_limit = FCSE_TASK_SIZE;
#endif /* CONFIG_ARM_FCSE_GUARANTEED */
	addr = vm_unmapped_area(info);

	if ((addr & ~PAGE_MASK) == 0 && addr + info->length <= FCSE_TASK_SIZE)
		return addr;

	/* Could not find an address */
  skip_retry:
#ifdef CONFIG_ARM_FCSE_BEST_EFFORT
	/* Could not find an address */
	if (addr & ~PAGE_MASK)
		return addr;

	/* It is not the first time this process gets addresses above 32MB */
	if (mm->context.fcse.large)
		return addr;

#ifdef CONFIG_ARM_FCSE_MESSAGES
	printk(KERN_INFO "FCSE: process %u(%s) VM exceeds 32MB.\n",
	       current->pid, current->comm);
#endif /* CONFIG_ARM_FCSE_MESSAGES */
	mm->context.fcse.large = 1;
	if (mm->context.fcse.pid)
		fcse_relocate_mm_to_null_pid(mm);

	return addr;

#else /* CONFIG_ARM_FCSE_GUARANTEED */
	/* Address above 32MB, no 32MB processes in guaranteed mode. */
#ifdef CONFIG_ARM_FCSE_MESSAGES
	if ((flags & MAP_BRK) == 0)
		printk(KERN_WARNING
		       "FCSE: process %u(%s) VM would exceed the 32MB limit.\n",
		       current->pid, current->comm);
#endif /* CONFIG_ARM_FCSE_MESSAGES */
	return -ENOMEM;
#endif /* CONFIG_ARM_FCSE_GUARANTEED */
}

#ifdef CONFIG_ARM_FCSE_MESSAGES
#define addr_in_vma(vma, addr)						\
	({								\
		struct vm_area_struct *_vma = (vma);			\
		((unsigned long)((addr) - _vma->vm_start)		\
		 < (unsigned long)((_vma->vm_end - _vma->vm_start)));	\
	})

#ifdef CONFIG_DEBUG_USER
static noinline void
dump_vmas(struct mm_struct *mm, unsigned long addr, struct pt_regs *regs)
{
	struct vm_area_struct *vma;
	char path[128];
	int locked = 0;

	printk("mappings:\n");
	if (!in_atomic())
		locked = down_read_trylock(&mm->mmap_sem);
	for(vma = mm->mmap; vma; vma = vma->vm_next) {
		struct file *file = vma->vm_file;
		int flags = vma->vm_flags;
		const char *name;

		printk("0x%08lx-0x%08lx %c%c%c%c 0x%08llx ",
		       vma->vm_start,
		       vma->vm_end,
		       flags & VM_READ ? 'r' : '-',
		       flags & VM_WRITE ? 'w' : '-',
		       flags & VM_EXEC ? 'x' : '-',
		       flags & VM_MAYSHARE ? 's' : 'p',
		       ((loff_t)vma->vm_pgoff) << PAGE_SHIFT);

		if (file)
			name = d_path(&file->f_path, path, sizeof(path));
		else if ((name = arch_vma_name(vma)))
			;
		else if (!vma->vm_mm)
			name = "[vdso]";
		else if (vma->vm_start <= mm->start_brk
			 && vma->vm_end >= mm->brk)
			name = "[heap]";
		else if (vma->vm_start <= mm->start_stack &&
			 vma->vm_end >= mm->start_stack)
			name = "[stack]";
		else
			name = "";
		printk("%s", name);
		if (addr_in_vma(vma, regs->ARM_pc))
			printk(" <- PC");
		if (addr_in_vma(vma, regs->ARM_sp))
			printk(" <- SP");
		if (addr_in_vma(vma, addr))
			printk("%s fault",
			       (addr_in_vma(vma, regs->ARM_pc)
				|| addr_in_vma(vma, regs->ARM_sp)
				? "," : " <-"));
		printk("\n");
	}
	if (locked)
		up_read(&mm->mmap_sem);
}
#endif /* CONFIG_DEBUG_USER */

void fcse_notify_segv(struct mm_struct *mm,
		       unsigned long addr, struct pt_regs *regs)
{
	int locked = 0;

#if defined(CONFIG_DEBUG_USER)
	if (user_debug & UDBG_SEGV)
		dump_vmas(mm, addr, regs);
#endif /* CONFIG_DEBUG_USER */

	if (!in_atomic())
		locked = down_read_trylock(&mm->mmap_sem);
	if (find_vma(mm, addr) == find_vma(mm, regs->ARM_sp))
		printk(KERN_INFO "FCSE: process %u(%s) probably overflowed stack at 0x%08lx.\n",
		       current->pid, current->comm, regs->ARM_pc);
	if (locked)
		up_read(&mm->mmap_sem);
}
#endif /* CONFIG_ARM_FCSE_MESSAGES */
