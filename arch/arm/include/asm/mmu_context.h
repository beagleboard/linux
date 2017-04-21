/*
 *  arch/arm/include/asm/mmu_context.h
 *
 *  Copyright (C) 1996 Russell King.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Changelog:
 *   27-06-1996	RMK	Created
 */
#ifndef __ASM_ARM_MMU_CONTEXT_H
#define __ASM_ARM_MMU_CONTEXT_H

#include <linux/compiler.h>
#include <linux/sched.h>
#include <asm/cacheflush.h>
#include <asm/cachetype.h>
#include <asm/proc-fns.h>
#include <asm/smp_plat.h>
#include <asm-generic/mm_hooks.h>
#include <asm/fcse.h>

void __check_vmalloc_seq(struct mm_struct *mm);

#ifdef CONFIG_CPU_HAS_ASID

int check_and_switch_context(struct mm_struct *mm,
			     struct task_struct *tsk, bool may_defer);
#define init_new_context(tsk,mm)	({ atomic64_set(&mm->context.id, 0); 0; })

#ifdef CONFIG_ARM_ERRATA_798181
void a15_erratum_get_cpumask(int this_cpu, struct mm_struct *mm,
			     cpumask_t *mask);
#else  /* !CONFIG_ARM_ERRATA_798181 */
static inline void a15_erratum_get_cpumask(int this_cpu, struct mm_struct *mm,
					   cpumask_t *mask)
{
}
#endif /* CONFIG_ARM_ERRATA_798181 */

#else	/* !CONFIG_CPU_HAS_ASID */

#ifdef CONFIG_MMU

static inline int
check_and_switch_context(struct mm_struct *mm,
			 struct task_struct *tsk, bool may_defer)
{
	if (unlikely(mm->context.vmalloc_seq != init_mm.context.vmalloc_seq))
		__check_vmalloc_seq(mm);

	if (may_defer && irqs_disabled()) {
		/*
		 * cpu_switch_mm() needs to flush the VIVT caches. To avoid
		 * high interrupt latencies, defer the call and continue
		 * running with the old mm. Since we only support UP systems
		 * on non-ASID CPUs, the old mm will remain valid until the
		 * finish_arch_post_lock_switch() call.
		 */
		mm->context.switch_pending = 1;
		return -EAGAIN;
	} else {
		cpu_switch_mm(mm->pgd, mm, fcse_switch_mm_start(mm));
	}

	return 0;
}

#ifdef CONFIG_IPIPE
extern void deferred_switch_mm(struct mm_struct *mm);
#else /* !I-pipe */
static inline void deferred_switch_mm(struct mm_struct *next)
{
	cpu_switch_mm(next->pgd, next, fcse_switch_mm_start(next));
	fcse_switch_mm_end(next);
}
#endif /* !I-pipe */

#define finish_arch_post_lock_switch \
	finish_arch_post_lock_switch
static inline void finish_arch_post_lock_switch(void)
{
	struct mm_struct *mm = current->mm;

	if (mm && mm->context.switch_pending) {
		/*
		 * Preemption must be disabled during cpu_switch_mm() as we
		 * have some stateful cache flush implementations. Check
		 * switch_pending again in case we were preempted and the
		 * switch to this mm was already done.
		 */
		preempt_disable();
		if (mm->context.switch_pending) {
			unsigned long flags;
			mm->context.switch_pending = 0;
			ipipe_mm_switch_protect(flags);
			deferred_switch_mm(mm);
			ipipe_mm_switch_unprotect(flags);
		}
		preempt_enable_no_resched();
	}
}
#endif	/* CONFIG_MMU */

static inline int
init_new_context(struct task_struct *tsk, struct mm_struct *mm)
{
#ifdef CONFIG_ARM_FCSE_BEST_EFFORT
	if (!mm->context.fcse.large)
		fcse_pid_alloc(mm);
	else {
		/* We are normally forking a process vith a virtual address
		   space larger than 32 MB, so its pid should be 0. */
		FCSE_BUG_ON(mm->context.fcse.pid);
		fcse_pid_reference(mm);
	}
	/* If we are forking, set_pte_at will restore the correct high pages
	   count, and shared writable pages are write-protected again. */
	mm->context.fcse.high_pages = 0;
	mm->context.fcse.highest_pid = 0;
	mm->context.fcse.shared_dirty_pages = 0;
#elif defined(CONFIG_ARM_FCSE_GUARANTEED)
	int err = fcse_pid_alloc(mm);
	if (err < 0)
		return err;
#endif /* CONFIG_ARM_FCSE_GUARANTEED */
	FCSE_BUG_ON(fcse_mm_in_cache(mm));

	return 0;
}

#endif	/* !CONFIG_CPU_HAS_ASID */

/*
 * This is called when "tsk" is about to enter lazy TLB mode.
 *
 * mm:  describes the currently active mm context
 * tsk: task which is entering lazy tlb
 * cpu: cpu number which is entering lazy tlb
 *
 * tsk->mm will be NULL
 */
static inline void
enter_lazy_tlb(struct mm_struct *mm, struct task_struct *tsk)
{
}

/*
 * This is the actual mm switch as far as the scheduler
 * is concerned.  No registers are touched.  We avoid
 * calling the CPU specific function when the mm hasn't
 * actually changed.
 */
static inline int
__do_switch_mm(struct mm_struct *prev, struct mm_struct *next,
	       struct task_struct *tsk, bool may_defer)
{
#ifdef CONFIG_MMU
	const unsigned int cpu = ipipe_processor_id();

	/*
	 * __sync_icache_dcache doesn't broadcast the I-cache invalidation,
	 * so check for possible thread migration and invalidate the I-cache
	 * if we're new to this CPU.
	 */
	if (cache_ops_need_broadcast() &&
	    !cpumask_empty(mm_cpumask(next)) &&
	    !cpumask_test_cpu(cpu, mm_cpumask(next)))
		__flush_icache_all();

	if (!cpumask_test_and_set_cpu(cpu, mm_cpumask(next)) || prev != next) {
		int rc = check_and_switch_context(next, tsk, may_defer);
#ifdef CONFIG_IPIPE
		if (rc < 0) {
			cpumask_clear_cpu(cpu, mm_cpumask(next));
			return rc;
		}
#ifdef CONFIG_ARM_FCSE
		if (tsk)
			set_tsk_thread_flag(tsk, TIF_SWITCHED);
#endif /* CONFIG_ARM_FCSE */
#else /* !CONFIG_IPIPE */
		if (rc == 0)
			fcse_switch_mm_end(next);
#endif /* CONFIG_IPIPE */
		if (cache_is_vivt() && prev)
			cpumask_clear_cpu(cpu, mm_cpumask(prev));
	} else
		fcse_mark_dirty(next);
#endif /* CONFIG_MMU */
	return 0;
}

#if defined(CONFIG_IPIPE) && defined(CONFIG_MMU)
extern void __switch_mm_inner(struct mm_struct *prev, struct mm_struct *next,
			      struct task_struct *tsk);
#else /* !I-pipe || !MMU */
#define __switch_mm_inner(prev, next, tsk) \
	__do_switch_mm(prev, next, tsk, true)
#endif /* !I-pipe  || !MMU */

static inline void
ipipe_switch_mm_head(struct mm_struct *prev, struct mm_struct *next,
			   struct task_struct *tsk)
{
	__do_switch_mm(prev, next, tsk, false);
	fcse_switch_mm_end(next);
}

static inline void
__switch_mm(struct mm_struct *prev, struct mm_struct *next,
	    struct task_struct *tsk)
{
	__switch_mm_inner(prev, next, tsk);
}

static inline void
switch_mm(struct mm_struct *prev, struct mm_struct *next,
	  struct task_struct *tsk)
{
#ifdef CONFIG_MMU
	unsigned long flags;
	ipipe_mm_switch_protect(flags);
	__switch_mm(prev, next, tsk);
	ipipe_mm_switch_unprotect(flags);
#endif /* CONFIG_MMU */
}

#define deactivate_mm(tsk,mm)	do { } while (0)

#ifndef CONFIG_ARM_FCSE_BEST_EFFORT
#define activate_mm(prev,next) __switch_mm(prev, next, NULL)
#else /* CONFIG_ARM_FCSE_BEST_EFFORT */
#define activate_mm(prev,next)                                         \
       ({                                                              \
       __switch_mm(prev, next, NULL);                                    \
       FCSE_BUG_ON(current->mm == next && next->context.switch_pending == 0 && !fcse_mm_in_cache(next));    \
       })
#endif /* CONFIG_ARM_FCSE_BEST_EFFORT */
static inline void destroy_context(struct mm_struct *mm)
{
#ifdef CONFIG_ARM_FCSE
#ifdef CONFIG_ARM_FCSE_BEST_EFFORT
	FCSE_BUG_ON(mm->context.fcse.shared_dirty_pages);
	FCSE_BUG_ON(mm->context.fcse.high_pages);
#endif /* CONFIG_ARM_FCSE_BEST_EFFORT */
	if (mm->context.fcse.pid != FCSE_PID_INVALID)
		fcse_pid_free(mm);
#endif /* CONFIG_ARM_FCSE */
}

#endif
