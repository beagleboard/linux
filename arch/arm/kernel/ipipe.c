/* -*- linux-c -*-
 * linux/arch/arm/kernel/ipipe.c
 *
 * Copyright (C) 2002-2005 Philippe Gerum.
 * Copyright (C) 2004 Wolfgang Grandegger (Adeos/arm port over 2.4).
 * Copyright (C) 2005 Heikki Lindholm (PowerPC 970 fixes).
 * Copyright (C) 2005 Stelian Pop.
 * Copyright (C) 2006-2008 Gilles Chanteperdrix.
 * Copyright (C) 2010 Philippe Gerum (SMP port).
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, Inc., 675 Mass Ave, Cambridge MA 02139,
 * USA; either version 2 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * Architecture-dependent I-PIPE support for ARM.
 */

#include <linux/kernel.h>
#include <linux/smp.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/bitops.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kallsyms.h>
#include <linux/kprobes.h>
#include <linux/ipipe_trace.h>
#include <linux/irq.h>
#include <linux/irqnr.h>
#include <linux/prefetch.h>
#include <linux/cpu.h>
#include <linux/ipipe_domain.h>
#include <linux/ipipe_tickdev.h>
#include <asm/system_info.h>
#include <asm/atomic.h>
#include <asm/hardirq.h>
#include <asm/io.h>
#include <asm/unistd.h>
#include <asm/mach/irq.h>
#include <asm/mmu_context.h>
#include <asm/exception.h>

static void __ipipe_do_IRQ(unsigned irq, void *cookie);

#ifdef CONFIG_IPIPE_DEBUG_INTERNAL
void (*__ipipe_mach_hrtimer_debug)(unsigned irq);
#endif

#ifdef CONFIG_SMP

struct __ipipe_vnmidata {
	void (*fn)(void *);
	void *arg;
	cpumask_t cpumask;
};

static struct __ipipe_vnmislot {
	ipipe_spinlock_t lock;
	struct __ipipe_vnmidata *data;
	ipipe_rwlock_t data_lock;
} __ipipe_vnmi __cacheline_aligned_in_smp = {
	.lock		= IPIPE_SPIN_LOCK_UNLOCKED,
	.data		= NULL,
	.data_lock	= IPIPE_RW_LOCK_UNLOCKED,
};

void __ipipe_early_core_setup(void)
{
	__ipipe_mach_init_platform();
}

void ipipe_stall_root(void)
{
	unsigned long flags;

	ipipe_root_only();
	flags = hard_smp_local_irq_save();
	__set_bit(IPIPE_STALL_FLAG, &__ipipe_root_status);
	hard_smp_local_irq_restore(flags);
}
EXPORT_SYMBOL_GPL(ipipe_stall_root);

unsigned long ipipe_test_and_stall_root(void)
{
	unsigned long flags;
	int x;

	ipipe_root_only();
	flags = hard_smp_local_irq_save();
	x = __test_and_set_bit(IPIPE_STALL_FLAG, &__ipipe_root_status);
	hard_smp_local_irq_restore(flags);

	return x;
}
EXPORT_SYMBOL_GPL(ipipe_test_and_stall_root);

unsigned long ipipe_test_root(void)
{
	unsigned long flags;
	int x;

	flags = hard_smp_local_irq_save();
	x = test_bit(IPIPE_STALL_FLAG, &__ipipe_root_status);
	hard_smp_local_irq_restore(flags);

	return x;
}
EXPORT_SYMBOL_GPL(ipipe_test_root);

void __ipipe_do_vnmi(unsigned int irq, void *cookie)
{
	int cpu = ipipe_processor_id();
	struct __ipipe_vnmidata *data;

	read_lock(&__ipipe_vnmi.data_lock);

	data = __ipipe_vnmi.data;
	if (likely(data && cpumask_test_cpu(cpu, &data->cpumask))) {
		data->fn(data->arg);
		cpumask_clear_cpu(cpu, &data->cpumask);
	}

	read_unlock(&__ipipe_vnmi.data_lock);
}

static inline void
hook_internal_ipi(struct ipipe_domain *ipd, int virq,
		  void (*handler)(unsigned int irq, void *cookie))
{
	ipd->irqs[virq].ackfn = NULL;
	ipd->irqs[virq].handler = handler;
	ipd->irqs[virq].cookie = NULL;
	/* Immediately handle in the current domain but *never* pass */
	ipd->irqs[virq].control = IPIPE_HANDLE_MASK|IPIPE_STICKY_MASK;
}

void __ipipe_hook_critical_ipi(struct ipipe_domain *ipd)
{
	__ipipe_ipis_alloc();
	hook_internal_ipi(ipd, IPIPE_CRITICAL_IPI, __ipipe_do_critical_sync);
	hook_internal_ipi(ipd, IPIPE_SERVICE_VNMI, __ipipe_do_vnmi);
}

void ipipe_set_irq_affinity(unsigned int irq, cpumask_t cpumask)
{
	if (ipipe_virtual_irq_p(irq) ||
	    irq_get_chip(irq)->irq_set_affinity == NULL)
		return;

	cpumask_and(&cpumask, &cpumask, cpu_online_mask);
	if (WARN_ON_ONCE(cpumask_empty(&cpumask)))
		return;

	irq_get_chip(irq)->irq_set_affinity(irq_get_irq_data(irq), &cpumask, true);
}
EXPORT_SYMBOL_GPL(ipipe_set_irq_affinity);

void __ipipe_send_vnmi(void (*fn)(void *), cpumask_t cpumask, void *arg)
{
	struct __ipipe_vnmidata data;
	unsigned long flags;
	int cpu;

	data.fn = fn;
	data.arg = arg;
	data.cpumask = cpumask;

	while (!spin_trylock_irqsave(&__ipipe_vnmi.lock, flags)) {
		if (hard_irqs_disabled())
			__ipipe_do_vnmi(IPIPE_SERVICE_VNMI, NULL);
		cpu_relax();
	}

	cpu = ipipe_processor_id();
	cpumask_clear_cpu(cpu, &data.cpumask);
	if (cpumask_empty(&data.cpumask)) {
		spin_unlock_irqrestore(&__ipipe_vnmi.lock, flags);
		return;
	}

	write_lock(&__ipipe_vnmi.data_lock);
	__ipipe_vnmi.data = &data;
	write_unlock(&__ipipe_vnmi.data_lock);

	ipipe_send_ipi(IPIPE_SERVICE_VNMI, data.cpumask);
	while (!cpumask_empty(&data.cpumask))
		cpu_relax();

	write_lock(&__ipipe_vnmi.data_lock);
	__ipipe_vnmi.data = NULL;
	write_unlock(&__ipipe_vnmi.data_lock);

	spin_unlock_irqrestore(&__ipipe_vnmi.lock, flags);
}
EXPORT_SYMBOL_GPL(__ipipe_send_vnmi);
#endif	/* CONFIG_SMP */

#ifdef CONFIG_SMP_ON_UP
struct static_key __ipipe_smp_key = STATIC_KEY_INIT_TRUE;
EXPORT_SYMBOL_GPL(__ipipe_smp_key);

unsigned notrace __ipipe_processor_id(void)
{
	return raw_smp_processor_id();
}
EXPORT_SYMBOL_GPL(__ipipe_processor_id);

static int ipipe_disable_smp(void)
{
	if (num_online_cpus() == 1) {
		unsigned long flags;

		printk("I-pipe: disabling SMP code\n");

		flags = hard_local_irq_save();
		static_key_slow_dec(&__ipipe_smp_key);
		hard_local_irq_restore(flags);
	}
	return 0;
}
arch_initcall(ipipe_disable_smp);

extern unsigned int smp_on_up;
EXPORT_SYMBOL_GPL(smp_on_up);
#endif /* SMP_ON_UP */

int ipipe_get_sysinfo(struct ipipe_sysinfo *info)
{
	info->sys_nr_cpus = num_online_cpus();
	info->sys_cpu_freq = __ipipe_hrclock_freq;
	info->sys_hrtimer_irq = per_cpu(ipipe_percpu.hrtimer_irq, 0);
	info->sys_hrtimer_freq = __ipipe_hrtimer_freq;
	info->sys_hrclock_freq = __ipipe_hrclock_freq;
	__ipipe_mach_get_tscinfo(&info->arch.tsc);

	return 0;
}
EXPORT_SYMBOL_GPL(ipipe_get_sysinfo);

struct ipipe_mach_pic_muter ipipe_pic_muter;
EXPORT_SYMBOL_GPL(ipipe_pic_muter);

void ipipe_pic_muter_register(struct ipipe_mach_pic_muter *muter)
{
	ipipe_pic_muter = *muter;
}

void __ipipe_enable_irqdesc(struct ipipe_domain *ipd, unsigned irq)
{
	/* With sparse IRQs, some irqs may not have a descriptor */
	if (irq_to_desc(irq) == NULL)
		return;

	if (ipipe_pic_muter.enable_irqdesc)
		ipipe_pic_muter.enable_irqdesc(ipd, irq);
}
EXPORT_SYMBOL_GPL(__ipipe_enable_irqdesc);

void __ipipe_disable_irqdesc(struct ipipe_domain *ipd, unsigned irq)
{
	if (ipipe_pic_muter.disable_irqdesc)
		ipipe_pic_muter.disable_irqdesc(ipd, irq);
}
EXPORT_SYMBOL_GPL(__ipipe_disable_irqdesc);

/*
 * __ipipe_enable_pipeline() -- We are running on the boot CPU, hw
 * interrupts are off, and secondary CPUs are still lost in space.
 */
void __ipipe_enable_pipeline(void)
{
	unsigned long flags;
	unsigned int irq;

#ifdef CONFIG_CPU_ARM926T
	/*
	 * We do not want "wfi" to be called in arm926ejs based
	 * processor, as this causes Linux to disable the I-cache
	 * when idle.
	 */
	extern void cpu_arm926_proc_init(void);
	if (likely(cpu_proc_init == &cpu_arm926_proc_init)) {
		printk("I-pipe: ARM926EJ-S detected, disabling wfi instruction"
		       " in idle loop\n");
		cpu_idle_poll_ctrl(true);
	}
#endif
	flags = ipipe_critical_enter(NULL);

	/* virtualize all interrupts from the root domain. */
	for (irq = 0; irq < IPIPE_NR_ROOT_IRQS; irq++)
		ipipe_request_irq(ipipe_root_domain,
				  irq,
				  (ipipe_irq_handler_t)__ipipe_do_IRQ,
				  NULL, NULL);

#ifdef CONFIG_SMP
	__ipipe_ipis_request();
#endif /* CONFIG_SMP */

	ipipe_critical_exit(flags);
}

#ifdef CONFIG_IPIPE_DEBUG_INTERNAL
unsigned asmlinkage __ipipe_bugon_irqs_enabled(unsigned x)
{
	BUG_ON(!hard_irqs_disabled());
	return x;		/* Preserve r0 */
}
#endif

asmlinkage int __ipipe_check_root_interruptible(void)
{
	return __ipipe_root_p && !irqs_disabled();
}

__kprobes int
__ipipe_switch_to_notifier_call_chain(struct atomic_notifier_head *nh,
				      unsigned long val, void *v)
{
	unsigned long flags;
	int ret;

	local_irq_save(flags);
	ret = atomic_notifier_call_chain(nh, val, v);
	__ipipe_restore_root_nosync(flags);

	return ret;
}

void __ipipe_exit_irq(struct pt_regs *regs)
{
	/*
	 * Testing for user_regs() eliminates foreign stack contexts,
	 * including from legacy domains which did not set the foreign
	 * stack bit (foreign stacks are always kernel-based).
	 */
	if (user_mode(regs) &&
	    ipipe_test_thread_flag(TIP_MAYDAY)) {
		/*
		 * MAYDAY is never raised under normal circumstances,
		 * so prefer test then maybe clear over
		 * test_and_clear.
		 */
		ipipe_clear_thread_flag(TIP_MAYDAY);
		__ipipe_notify_trap(IPIPE_TRAP_MAYDAY, regs);
	}
}

/* hw irqs off */
asmlinkage void __exception __ipipe_grab_irq(int irq, struct pt_regs *regs)
{
	struct ipipe_percpu_data *p = __ipipe_raw_cpu_ptr(&ipipe_percpu);

	ipipe_trace_irq_entry(irq);

	if (p->hrtimer_irq == -1)
		goto copy_regs;

	if (irq == p->hrtimer_irq) {
		/*
		 * Given our deferred dispatching model for regular IRQs, we
		 * only record CPU regs for the last timer interrupt, so that
		 * the timer handler charges CPU times properly. It is assumed
		 * that other interrupt handlers don't actually care for such
		 * information.
		 */
#ifdef CONFIG_IPIPE_DEBUG_INTERNAL
		if (__ipipe_mach_hrtimer_debug)
			__ipipe_mach_hrtimer_debug(irq);
#endif /* CONFIG_IPIPE_DEBUG_INTERNAL */
	  copy_regs:
		p->tick_regs.ARM_cpsr =
			(p->curr == &p->root
			 ? regs->ARM_cpsr
			 : regs->ARM_cpsr | PSR_I_BIT);
		p->tick_regs.ARM_pc = regs->ARM_pc;
	}

	__ipipe_dispatch_irq(irq, 0);

	ipipe_trace_irq_exit(irq);

	__ipipe_exit_irq(regs);
}

static void __ipipe_do_IRQ(unsigned irq, void *cookie)
{
	handle_IRQ(irq, raw_cpu_ptr(&ipipe_percpu.tick_regs));
}

#ifdef CONFIG_MMU
void __switch_mm_inner(struct mm_struct *prev, struct mm_struct *next,
		       struct task_struct *tsk)
{
	struct mm_struct ** const active_mm =
		raw_cpu_ptr(&ipipe_percpu.active_mm);
#ifdef CONFIG_IPIPE_WANT_PREEMPTIBLE_SWITCH
	struct thread_info *const tip = current_thread_info();
	prev = *active_mm;
	clear_bit(TIF_MMSWITCH_INT, &tip->flags);
	barrier();
	*active_mm = NULL;
	barrier();
	for (;;) {
		unsigned long flags;
#endif /* CONFIG_IPIPE_WANT_PREEMPTIBLE_SWITCH */

		int rc __maybe_unused = __do_switch_mm(prev, next, tsk, true);

#ifdef CONFIG_IPIPE_WANT_PREEMPTIBLE_SWITCH
		/*
		 * Reading thread_info flags and setting active_mm
		 * must be done atomically.
		 */
		flags = hard_local_irq_save();
		if (__test_and_clear_bit(TIF_MMSWITCH_INT, &tip->flags) == 0) {
			if (rc < 0)
				*active_mm = prev;
			else {
				*active_mm = next;
				fcse_switch_mm_end(next);
			}
			hard_local_irq_restore(flags);
			return;
		}
		hard_local_irq_restore(flags);

		if (rc < 0)
			/*
			 * We were interrupted by head domain, which
			 * may have changed the mm context, mm context
			 * is now unknown, but will be switched in
			 * deferred_switch_mm
			 */
			return;

		prev = NULL;
	}
#else
	if (rc < 0)
		*active_mm = prev;
	else {
		*active_mm = next;
		fcse_switch_mm_end(next);
	}
#endif /* !IPIPE_WANT_PREEMPTIBLE_SWITCH */
}

#ifdef finish_arch_post_lock_switch
void deferred_switch_mm(struct mm_struct *next)
{
	struct mm_struct ** const active_mm =
		raw_cpu_ptr(&ipipe_percpu.active_mm);
	struct mm_struct *prev = *active_mm;
#ifdef CONFIG_IPIPE_WANT_PREEMPTIBLE_SWITCH
	struct thread_info *const tip = current_thread_info();
	clear_bit(TIF_MMSWITCH_INT, &tip->flags);
	barrier();
	*active_mm = NULL;
	barrier();
	for (;;) {
		unsigned long flags;
#endif /* CONFIG_IPIPE_WANT_PREEMPTIBLE_SWITCH */

		__do_switch_mm(prev, next, NULL, false);

#ifdef CONFIG_IPIPE_WANT_PREEMPTIBLE_SWITCH
		/*
		 * Reading thread_info flags and setting active_mm
		 * must be done atomically.
		 */
		flags = hard_local_irq_save();
		if (__test_and_clear_bit(TIF_MMSWITCH_INT, &tip->flags) == 0) {
			*active_mm = next;
			fcse_switch_mm_end(next);
			hard_local_irq_restore(flags);
			return;
		}
		hard_local_irq_restore(flags);
		prev = NULL;
	}
#else
	*active_mm = next;
	fcse_switch_mm_end(next);
#endif /* CONFIG_IPIPE_WANT_PREEMPTIBLE_SWITCH */
}
#endif
#endif /* CONFIG_MMU */

EXPORT_SYMBOL_GPL(do_munmap);
EXPORT_SYMBOL_GPL(show_stack);
EXPORT_SYMBOL_GPL(init_mm);
#ifndef MULTI_CPU
EXPORT_SYMBOL_GPL(cpu_do_switch_mm);
#endif
EXPORT_SYMBOL_GPL(__check_vmalloc_seq);
#if defined(CONFIG_SMP) || defined(CONFIG_DEBUG_SPINLOCK)
EXPORT_SYMBOL_GPL(tasklist_lock);
#endif /* CONFIG_SMP || CONFIG_DEBUG_SPINLOCK */

#ifndef CONFIG_SPARSE_IRQ
EXPORT_SYMBOL_GPL(irq_desc);
#endif

#ifdef CONFIG_CPU_HAS_ASID
EXPORT_SYMBOL_GPL(check_and_switch_context);
#endif /* CONFIG_CPU_HAS_ASID */

#if defined(CONFIG_SMP) && defined(CONFIG_IPIPE_LEGACY)
EXPORT_SYMBOL_GPL(__cpu_logical_map);
#endif /* CONFIG_IPIPE */

EXPORT_SYMBOL_GPL(cpu_architecture);
