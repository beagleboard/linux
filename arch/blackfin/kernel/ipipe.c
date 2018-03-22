/* -*- linux-c -*-
 * linux/arch/blackfin/kernel/ipipe.c
 *
 * Copyright (C) 2005-2007 Philippe Gerum.
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
 * Architecture-dependent I-pipe support for the Blackfin.
 */

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/percpu.h>
#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/kthread.h>
#include <linux/unistd.h>
#include <linux/io.h>
#include <linux/atomic.h>
#include <linux/ipipe_tickdev.h>
#include <asm/irq_handler.h>
#include <asm/blackfin.h>
#include <asm/time.h>

asmlinkage void asm_do_IRQ(unsigned int irq, struct pt_regs *regs);

static void __ipipe_do_IRQ(unsigned int irq, void *cookie);

static void __ipipe_no_irqtail(void);

static atomic_t __ipipe_irq_lvdepth[IVG15 + 1];

static unsigned long __ipipe_irq_lvmask = bfin_no_irqs;

unsigned long __ipipe_irq_tail_hook = (unsigned long)__ipipe_no_irqtail;
EXPORT_SYMBOL_GPL(__ipipe_irq_tail_hook);

unsigned long __ipipe_core_clock;
EXPORT_SYMBOL_GPL(__ipipe_core_clock);

unsigned long __ipipe_freq_scale;
EXPORT_SYMBOL_GPL(__ipipe_freq_scale);

/*
 * __ipipe_enable_pipeline() -- We are running on the boot CPU, hw
 * interrupts are off, and secondary CPUs are still lost in space.
 */
void __ipipe_enable_pipeline(void)
{
	unsigned irq;

	__ipipe_core_clock = get_cclk(); /* Fetch this once. */
	__ipipe_freq_scale = 1000000000UL / __ipipe_core_clock;

	for (irq = 0; irq < NR_IRQS; ++irq)
		ipipe_request_irq(ipipe_root_domain, irq,
				  __ipipe_do_IRQ, NULL,
				  NULL);
}

void __ipipe_handle_irq(unsigned int irq, struct pt_regs *regs) /* hw IRQs off */
{
	struct ipipe_percpu_domain_data *p = ipipe_this_cpu_root_context();
	int flags, s = -1;

	if (test_bit(IPIPE_SYNCDEFER_FLAG, &p->status))
		s = __test_and_set_bit(IPIPE_STALL_FLAG, &p->status);

	flags = (regs && irq != IRQ_SYSTMR && irq != IRQ_CORETMR) ?
		0 : IPIPE_IRQF_NOACK;
	__ipipe_dispatch_irq(irq, flags);

	if (s == 0)
		__clear_bit(IPIPE_STALL_FLAG, &p->status);
}

void __ipipe_enable_irqdesc(struct ipipe_domain *ipd, unsigned int irq)
{
	struct irq_desc *desc = irq_to_desc(irq);
	int prio = __ipipe_get_irq_priority(irq);

	desc->depth = 0;
	if (ipd != ipipe_root_domain &&
	    atomic_inc_return(&__ipipe_irq_lvdepth[prio]) == 1)
		__set_bit(prio, &__ipipe_irq_lvmask);
}

void __ipipe_disable_irqdesc(struct ipipe_domain *ipd, unsigned int irq)
{
	int prio = __ipipe_get_irq_priority(irq);

	if (ipd != ipipe_root_domain &&
	    atomic_dec_and_test(&__ipipe_irq_lvdepth[prio]))
		__clear_bit(prio, &__ipipe_irq_lvmask);
}

static void __ipipe_do_IRQ(unsigned int irq, void *cookie)
{
	struct pt_regs *regs = raw_cpu_ptr(&ipipe_percpu.tick_regs);
	asm_do_IRQ(irq, regs);
}

static void __ipipe_no_irqtail(void)
{
}

int __ipipe_do_sync_check(void)
{
	return !(ipipe_root_p &&
		 test_bit(IPIPE_SYNCDEFER_FLAG, &__ipipe_root_status));
}

int ipipe_get_sysinfo(struct ipipe_sysinfo *info)
{
	info->sys_nr_cpus = num_online_cpus();
	info->sys_cpu_freq = ipipe_cpu_freq();
	info->sys_hrtimer_irq = per_cpu(ipipe_percpu.hrtimer_irq, 0);
	info->sys_hrtimer_freq = __ipipe_hrtimer_freq;
	info->sys_hrclock_freq = __ipipe_core_clock;

	return 0;
}
EXPORT_SYMBOL_GPL(ipipe_get_sysinfo);

void __ipipe_sync_root(void)
{
	void (*irq_tail_hook)(void) = (void (*)(void))__ipipe_irq_tail_hook;
	struct ipipe_percpu_domain_data *p;
	unsigned long flags;

	BUG_ON(irqs_disabled());

	flags = hard_local_irq_save();

	if (irq_tail_hook)
		irq_tail_hook();

	clear_thread_flag(TIF_IRQ_SYNC);

	p = ipipe_this_cpu_root_context();
	if (__ipipe_ipending_p(p))
		__ipipe_sync_stage();

	hard_local_irq_restore(flags);
}

unsigned long __ipipe_hard_save_root_irqs(void)
{
	/*
	 * This code is called by the ins{bwl} routines (see
	 * arch/blackfin/lib/ins.S), which are heavily used by the
	 * network stack. It masks all interrupts but those handled by
	 * non-root domains, so that we keep decent network transfer
	 * rates for Linux without inducing pathological jitter for
	 * the real-time domain.
	 */
	bfin_sti(__ipipe_irq_lvmask);
	return __test_and_set_bit(IPIPE_STALL_FLAG, &__ipipe_root_status) ?
		bfin_no_irqs : bfin_irq_flags;
}

void __ipipe_hard_restore_root_irqs(unsigned long flags)
{
	if (flags != bfin_no_irqs)
		__clear_bit(IPIPE_STALL_FLAG, &__ipipe_root_status);

	bfin_sti(bfin_irq_flags);
}

/*
 * We could use standard atomic bitops in the following root status
 * manipulation routines, but let's prepare for SMP support in the
 * same move, preventing CPU migration as required.
 */
void ipipe_stall_root(void)
{
	unsigned long *p, flags;

	ipipe_root_only();
	flags = hard_smp_local_irq_save();
	p = &__ipipe_root_status;
	__set_bit(IPIPE_STALL_FLAG, p);
	hard_smp_local_irq_restore(flags);
}
EXPORT_SYMBOL_GPL(ipipe_stall_root);

unsigned long ipipe_test_and_stall_root(void)
{
	unsigned long *p, flags;
	int x;

	ipipe_root_only();
	flags = hard_smp_local_irq_save();
	p = &__ipipe_root_status;
	x = __test_and_set_bit(IPIPE_STALL_FLAG, p);
	hard_smp_local_irq_restore(flags);

	return x;
}
EXPORT_SYMBOL_GPL(ipipe_test_and_stall_root);

unsigned long ipipe_test_root(void)
{
	const unsigned long *p;
	unsigned long flags;
	int x;

	ipipe_root_only();
	flags = hard_smp_local_irq_save();
	p = &__ipipe_root_status;
	x = test_bit(IPIPE_STALL_FLAG, p);
	hard_smp_local_irq_restore(flags);

	return x;
}
EXPORT_SYMBOL_GPL(ipipe_test_root);

void __ipipe_lock_root(void)
{
	unsigned long *p, flags;

	flags = hard_smp_local_irq_save();
	p = &__ipipe_root_status;
	__set_bit(IPIPE_SYNCDEFER_FLAG, p);
	hard_smp_local_irq_restore(flags);
}
EXPORT_SYMBOL_GPL(__ipipe_lock_root);

void __ipipe_unlock_root(void)
{
	unsigned long *p, flags;

	flags = hard_smp_local_irq_save();
	p = &__ipipe_root_status;
	__clear_bit(IPIPE_SYNCDEFER_FLAG, p);
	hard_smp_local_irq_restore(flags);
}
EXPORT_SYMBOL_GPL(__ipipe_unlock_root);

/*
 * We have two main options on the Blackfin for dealing with the clock
 * event sources for both domains:
 *
 * - If CONFIG_GENERIC_CLOCKEVENTS is disabled (old cranky stuff), we
 * force the system timer to GPT0.  This gives the head domain
 * exclusive control over the Blackfin core timer.  Therefore, we have
 * to flesh out the core timer request and release handlers since the
 * regular kernel won't have set it up at boot.
 *
 * - If CONFIG_GENERIC_CLOCKEVENTS is enabled, then Linux may pick
 * either GPT0 (CONFIG_TICKSOURCE_GPTMR0), or the core timer
 * (CONFIG_TICKSOURCE_CORETMR) as its own tick source. Depending on
 * what Kconfig says regarding this setting, we may have in turn:
 *
 * - CONFIG_TICKSOURCE_CORETMR is set, which means that both root
 * (linux) and the head domain will have to share the core timer for
 * timing duties. In this case, we don't register the core timer with
 * the pipeline, we only connect the regular linux clock event
 * structure to our ipipe_time timer structure via the ipipe_timer
 * field in struct clock_event_device.
 *
 * - CONFIG_TICKSOURCE_GPTMR0 is set, in which case we reserve the
 * core timer to the head domain, just like in the
 * CONFIG_GENERIC_CLOCKEVENTS disabled case. We have to register the
 * core timer with the pipeline, so that ipipe_select_timers() may
 * find it.
 */
#if defined(CONFIG_GENERIC_CLOCKEVENTS) && defined(CONFIG_TICKSOURCE_CORETMR)

static inline void icoretmr_request(struct ipipe_timer *timer, int steal)
{
}

static inline void icoretmr_release(struct ipipe_timer *timer)
{
}

#else /* !(CONFIG_GENERIC_CLOCKEVENTS && CONFIG_TICKSOURCE_CORETMR) */

static void icoretmr_request(struct ipipe_timer *timer, int steal)
{
	bfin_write_TCNTL(TMPWR);
	CSYNC();
	bfin_write_TSCALE(TIME_SCALE - 1);
	bfin_write_TPERIOD(0);
	bfin_write_TCOUNT(0);
	CSYNC();
}

static void icoretmr_release(struct ipipe_timer *timer)
{
	/* Power down the core timer */
	bfin_write_TCNTL(0);
}

#endif /* !(CONFIG_GENERIC_CLOCKEVENTS && CONFIG_TICKSOURCE_CORETMR) */

static int icoretmr_set(unsigned long evt, void *timer)
{
	bfin_write_TCNTL(TMPWR);
	CSYNC();
	bfin_write_TCOUNT(evt);
	CSYNC();
	bfin_write_TCNTL(TMPWR | TMREN);

	return 0;
}

struct ipipe_timer bfin_coretmr_itimer = {
	.irq			= IRQ_CORETMR,
	.request		= icoretmr_request,
	.set			= icoretmr_set,
	.ack			= NULL,
	.release		= icoretmr_release,
	.name			= "bfin_coretmr",
	.rating			= 500,
	.min_delay_ticks	= 2,
};

void bfin_ipipe_coretmr_register(void)
{
	bfin_coretmr_itimer.freq = get_cclk() / TIME_SCALE;
#if !(defined(CONFIG_GENERIC_CLOCKEVENTS) && defined(CONFIG_TICKSOURCE_CORETMR))
	ipipe_timer_register(&bfin_coretmr_itimer);
#endif
}
