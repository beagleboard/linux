/* -*- linux-c -*-
 * linux/arch/powerpc/kernel/ipipe.c
 *
 * Copyright (C) 2005 Heikki Lindholm (PPC64 port).
 * Copyright (C) 2004 Wolfgang Grandegger (Adeos/ppc port over 2.4).
 * Copyright (C) 2002-2012 Philippe Gerum.
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
 * Architecture-dependent I-PIPE core support for PowerPC 32/64bit.
 */

#include <linux/kernel.h>
#include <linux/smp.h>
#include <linux/sched.h>
#include <linux/ipipe.h>
#include <linux/slab.h>
#include <linux/bitops.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/kernel_stat.h>
#include <linux/ipipe_tickdev.h>
#include <asm/reg.h>
#include <asm/switch_to.h>
#include <asm/mmu_context.h>
#include <asm/unistd.h>
#include <asm/machdep.h>
#include <asm/atomic.h>
#include <asm/hardirq.h>
#include <asm/io.h>
#include <asm/time.h>
#include <asm/runlatch.h>
#include <asm/debug.h>
#include <asm/dbell.h>

static void __ipipe_do_IRQ(unsigned int irq, void *cookie);

static void __ipipe_do_timer(unsigned int irq, void *cookie);

#ifdef CONFIG_PPC_DOORBELL
static void __ipipe_do_doorbell(unsigned int irq, void *cookie);
#endif

#define DECREMENTER_MAX	0x7fffffff

#ifdef CONFIG_SMP

static DEFINE_PER_CPU(struct ipipe_ipi_struct, ipipe_ipi_message);

#if defined(CONFIG_DEBUGGER) || defined(CONFIG_KEXEC)
cpumask_t __ipipe_dbrk_pending;	/* pending debugger break IPIs */
#endif

static unsigned int mux_ipi;

void __ipipe_register_mux_ipi(unsigned int irq)
{
	mux_ipi = irq;
}

void __ipipe_hook_critical_ipi(struct ipipe_domain *ipd)
{
	unsigned int ipi = IPIPE_CRITICAL_IPI;

	ipd->irqs[ipi].ackfn = NULL;
	ipd->irqs[ipi].handler = __ipipe_do_critical_sync;
	ipd->irqs[ipi].cookie = NULL;
	ipd->irqs[ipi].control = IPIPE_HANDLE_MASK|IPIPE_STICKY_MASK;
}

static void do_ipi_demux(int irq, struct pt_regs *regs)
{
	int cpu __maybe_unused = ipipe_processor_id(), ipi;

	while (this_cpu_ptr(&ipipe_ipi_message)->value & IPIPE_MSG_IPI_MASK) {
		for (ipi = IPIPE_MSG_CRITICAL_IPI;
		     ipi <= IPIPE_MSG_RESCHEDULE_IPI; ++ipi) {
			if (test_and_clear_bit(ipi,
			       &this_cpu_ptr(&ipipe_ipi_message)->value)) {
				mb();
				__ipipe_handle_irq(ipi + IPIPE_BASE_IPI_OFFSET, NULL);
			}
		}
	}

#if defined(CONFIG_DEBUGGER) || defined(CONFIG_KEXEC)
	/*
	 * The debugger IPI handler should be NMI-safe, so let's call
	 * it immediately in case the IPI is pending.
	 */
	if (cpumask_test_cpu(cpu, &__ipipe_dbrk_pending)) {
		cpumask_clear_cpu(cpu, &__ipipe_dbrk_pending);
		debugger_ipi(regs);
	}
#endif /* CONFIG_DEBUGGER || CONFIG_KEXEC */

	__ipipe_finish_ipi_demux(irq);
}

void ipipe_set_irq_affinity(unsigned int irq, cpumask_t cpumask)
{
	if (ipipe_virtual_irq_p(irq) ||
	    irq_get_chip(irq)->irq_set_affinity == NULL)
		return;

	if (WARN_ON_ONCE(cpumask_any_and(&cpumask, cpu_online_mask) >= nr_cpu_ids))
		return;

	irq_get_chip(irq)->irq_set_affinity(irq_get_irq_data(irq), &cpumask, true);
}
EXPORT_SYMBOL_GPL(ipipe_set_irq_affinity);

void ipipe_send_ipi(unsigned int ipi, cpumask_t cpumask)
{
	unsigned long flags;
	int cpu, me;

	flags = hard_local_irq_save();

	me = ipipe_processor_id();
	ipi -= IPIPE_BASE_IPI_OFFSET;
	for_each_cpu(cpu, &cpumask) {
		if (cpu == me)
			continue;
		set_bit(ipi, &per_cpu(ipipe_ipi_message, cpu).value);
		if (smp_ops->message_pass)
			smp_ops->message_pass(cpu, PPC_MSG_IPIPE_DEMUX);
#ifdef CONFIG_PPC_SMP_MUXED_IPI
		else
			smp_muxed_ipi_message_pass(cpu, PPC_MSG_IPIPE_DEMUX);
#endif
	}

	hard_local_irq_restore(flags);
}
EXPORT_SYMBOL_GPL(ipipe_send_ipi);

void ipipe_stall_root(void)
{
	unsigned long flags;

	ipipe_root_only();
	flags = hard_local_irq_save();
	set_bit(IPIPE_STALL_FLAG, &__ipipe_root_status);
	hard_local_irq_restore(flags);
}
EXPORT_SYMBOL_GPL(ipipe_stall_root);

unsigned long ipipe_test_and_stall_root(void)
{
	unsigned long flags;
	int x;

	ipipe_root_only();
	flags = hard_local_irq_save();
	x = test_and_set_bit(IPIPE_STALL_FLAG, &__ipipe_root_status);
	hard_local_irq_restore(flags);

	return x;
}
EXPORT_SYMBOL(ipipe_test_and_stall_root);

unsigned long ipipe_test_root(void)
{
	unsigned long flags;
	int x;

	flags = hard_local_irq_save();
	x = test_bit(IPIPE_STALL_FLAG, &__ipipe_root_status);
	hard_local_irq_restore(flags);

	return x;
}
EXPORT_SYMBOL_GPL(ipipe_test_root);

#endif	/* !CONFIG_SMP */

void __ipipe_early_core_setup(void)
{
	unsigned int virq;
	/*
	 * Allocate all the virtual IRQs we need. We expect fixed virq
	 * numbers starting at IPIPE_VIRQ_BASE, so we request them
	 * early.
	 */
	virq = ipipe_alloc_virq();
	BUG_ON(virq != IPIPE_TIMER_VIRQ);
	/*
	 * Although not all CPUs define the doorbell event, we always
	 * allocate the corresponding VIRQ, so that we can keep fixed
	 * values for all VIRQ numbers.
	 */
	virq = ipipe_alloc_virq();
	BUG_ON(virq != IPIPE_DOORBELL_VIRQ);
#ifdef CONFIG_SMP
	virq = ipipe_alloc_virq();
	BUG_ON(virq != IPIPE_CRITICAL_IPI);
	virq = ipipe_alloc_virq();
	BUG_ON(virq != IPIPE_HRTIMER_IPI);
	virq = ipipe_alloc_virq();
	BUG_ON(virq != IPIPE_RESCHEDULE_IPI);
#endif
}

/*
 * __ipipe_enable_pipeline() -- We are running on the boot CPU, hw
 * interrupts are off, and secondary CPUs are still lost in space.
 */
void __ipipe_enable_pipeline(void)
{
	unsigned long flags;
	unsigned int irq;

	flags = ipipe_critical_enter(NULL);

	/* First, intercept all interrupts from the root
	 * domain. Regular Linux interrupt handlers will receive
	 * raw_cpu_ptr(&ipipe_percpu.tick_regs) for external IRQs,
	 * whatever cookie is passed here.
	 */
	for (irq = 0; irq < NR_IRQS; irq++)
		ipipe_request_irq(ipipe_root_domain,
				  irq,
				  __ipipe_do_IRQ, NULL,
				  NULL);
	/*
	 * We use a virtual IRQ to handle the timer irq (decrementer
	 * trap) which was allocated early in
	 * __ipipe_early_core_setup().
	 */
	ipipe_request_irq(ipipe_root_domain,
			  IPIPE_TIMER_VIRQ,
			  __ipipe_do_timer, NULL,
			  NULL);

#ifdef CONFIG_PPC_DOORBELL
	ipipe_request_irq(ipipe_root_domain,
			  IPIPE_DOORBELL_VIRQ,
			  __ipipe_do_doorbell, NULL,
			  NULL);
#endif

	ipipe_critical_exit(flags);
}

int ipipe_get_sysinfo(struct ipipe_sysinfo *info)
{
	info->sys_nr_cpus = num_online_cpus();
	info->sys_cpu_freq = __ipipe_cpu_freq;
	info->sys_hrtimer_irq = per_cpu(ipipe_percpu.hrtimer_irq, 0);
	info->sys_hrtimer_freq = __ipipe_hrtimer_freq;
	info->sys_hrclock_freq = __ipipe_hrclock_freq;

	return 0;
}
EXPORT_SYMBOL_GPL(ipipe_get_sysinfo);

static int __ipipe_exit_irq(struct pt_regs *regs)
{
	int root = __ipipe_root_p;

	if (root) {
#ifdef CONFIG_PPC_970_NAP
		struct thread_info *ti = current_thread_info();
		/* Emulate the napping check when 100% sure we do run
		 * over the root context. */
		if (test_and_clear_bit(TLF_NAPPING, &ti->local_flags))
			regs->nip = regs->link;
#endif
#ifdef CONFIG_PPC64
		ppc64_runlatch_on();
#endif
	}

	/*
	 * Testing for user_regs() eliminates foreign stack contexts,
	 * including from legacy domains (CONFIG_IPIPE_LEGACY) which
	 * did not set the foreign stack bit (foreign stacks are
	 * always kernel-based).
	 */
	if (user_mode(regs) && ipipe_test_thread_flag(TIP_MAYDAY))
		__ipipe_call_mayday(regs);

	if (root && !test_bit(IPIPE_STALL_FLAG, &__ipipe_root_status))
		return 1;

	return 0;
}

int __ipipe_grab_irq(struct pt_regs *regs)
{
	int irq;

	irq = ppc_md.get_irq();
	if (unlikely(irq == NO_IRQ)) {
		__this_cpu_add(irq_stat.spurious_irqs, 1);
		return __ipipe_exit_irq(regs);
	}

	if (likely(irq != NO_IRQ)) {
		ipipe_trace_irq_entry(irq);
#ifdef CONFIG_SMP
		if (irq == mux_ipi) {
			struct irq_desc *desc = irq_to_desc(irq);
			desc->ipipe_ack(desc);
			kstat_incr_irq_this_cpu(irq);
			do_ipi_demux(irq, regs);
			ipipe_end_irq(irq);
		} else
#endif /* CONFIG_SMP */
			__ipipe_handle_irq(irq, regs);
	}

	ipipe_trace_irq_exit(irq);

	return __ipipe_exit_irq(regs);
}

static void __ipipe_do_IRQ(unsigned int irq, void *cookie)
{
	struct pt_regs *regs, *old_regs;

	/* Any sensible register frame will do for non-timer IRQs. */
	regs = raw_cpu_ptr(&ipipe_percpu.tick_regs);
	old_regs = set_irq_regs(regs);
	___do_irq(irq, regs);
	set_irq_regs(old_regs);
}

static void __ipipe_do_timer(unsigned int irq, void *cookie)
{
	check_stack_overflow();
	timer_interrupt(raw_cpu_ptr(&ipipe_percpu.tick_regs));
}

#ifdef CONFIG_PPC_DOORBELL

int __ipipe_grab_doorbell(struct pt_regs *regs)
{
#ifdef CONFIG_SMP
	do_ipi_demux(IPIPE_DOORBELL_VIRQ, regs);
#endif
	return __ipipe_exit_irq(regs);
}

static void __ipipe_do_doorbell(unsigned int irq, void *cookie)
{
	doorbell_exception(raw_cpu_ptr(&ipipe_percpu.tick_regs));
}

#endif

int __ipipe_grab_timer(struct pt_regs *regs)
{
	struct pt_regs *tick_regs;
	struct ipipe_domain *ipd;

	ipd = __ipipe_current_domain;

	set_dec(DECREMENTER_MAX);

	ipipe_trace_irq_entry(IPIPE_TIMER_VIRQ);

	tick_regs = raw_cpu_ptr(&ipipe_percpu.tick_regs);
	tick_regs->msr = regs->msr;
	tick_regs->nip = regs->nip;
	if (ipd != &ipipe_root)
		/* Tick should not be charged to Linux. */
		tick_regs->msr &= ~MSR_EE;

	__ipipe_handle_irq(IPIPE_TIMER_VIRQ, NULL);

	ipipe_trace_irq_exit(IPIPE_TIMER_VIRQ);

	return __ipipe_exit_irq(regs);
}

EXPORT_SYMBOL_GPL(show_stack);
EXPORT_SYMBOL_GPL(_switch);
#ifdef CONFIG_IPIPE_LEGACY
#ifdef CONFIG_PPC64
EXPORT_PER_CPU_SYMBOL(ppc64_tlb_batch);
EXPORT_SYMBOL_GPL(switch_slb);
EXPORT_SYMBOL_GPL(__flush_tlb_pending);
#else  /* !CONFIG_PPC64 */
void atomic_set_mask(unsigned long mask, unsigned long *ptr);
void atomic_clear_mask(unsigned long mask, unsigned long *ptr);
EXPORT_SYMBOL_GPL(atomic_set_mask);
EXPORT_SYMBOL_GPL(atomic_clear_mask);
#endif	/* !CONFIG_PPC64 */
#endif /* !CONFIG_IPIPE_LEGACY */
