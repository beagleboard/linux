/*   -*- linux-c -*-
 *   include/asm-blackfin/ipipe.h
 *
 *   Copyright (C) 2002-2007 Philippe Gerum.
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, Inc., 675 Mass Ave, Cambridge MA 02139,
 *   USA; either version 2 of the License, or (at your option) any later
 *   version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __ASM_BLACKFIN_IPIPE_H
#define __ASM_BLACKFIN_IPIPE_H

#ifdef CONFIG_IPIPE

#include <linux/cpumask.h>
#include <linux/list.h>
#include <linux/threads.h>
#include <linux/irq.h>
#include <linux/ipipe_domain.h>
#include <asm/ptrace.h>
#include <asm/irq.h>
#include <asm/bitops.h>
#include <linux/atomic.h>
#include <asm/traps.h>
#include <asm/bitsperlong.h>

#define IPIPE_CORE_RELEASE	5

#ifdef CONFIG_SMP
#error "I-pipe/blackfin: SMP not implemented"
#else /* !CONFIG_SMP */
#define ipipe_processor_id()	0
#endif	/* CONFIG_SMP */

struct ipipe_domain;

struct ipipe_arch_sysinfo {
};

#define ipipe_read_tsc(t)					\
	({							\
	unsigned long __cy2;					\
	__asm__ __volatile__ ("1: %0 = CYCLES2\n"		\
				"%1 = CYCLES\n"			\
				"%2 = CYCLES2\n"		\
				"CC = %2 == %0\n"		\
				"if ! CC jump 1b\n"		\
				: "=d,a" (((unsigned long *)&t)[1]),	\
				  "=d,a" (((unsigned long *)&t)[0]),	\
				  "=d,a" (__cy2)				\
				: /*no input*/ : "CC");			\
	t;								\
	})

#define ipipe_cpu_freq()	__ipipe_core_clock

#define __ipipe_hrclock_freq	__ipipe_core_clock

#define ipipe_tsc2ns(_t)	(((unsigned long)(_t)) * __ipipe_freq_scale)
#define ipipe_tsc2us(_t)	(ipipe_tsc2ns(_t) / 1000 + 1)

static inline const char *ipipe_clock_name(void)
{
	return "cyclectr";
}

/* Private interface -- Internal use only */

#define __ipipe_early_core_setup()	do { } while (0)

/* enable/disable_irqdesc _must_ be used in pairs. */

void __ipipe_enable_irqdesc(struct ipipe_domain *ipd,
			    unsigned int irq);

void __ipipe_disable_irqdesc(struct ipipe_domain *ipd,
			     unsigned int irq);

void __ipipe_enable_pipeline(void);

#define __ipipe_hook_critical_ipi(ipd) do { } while (0)

int __ipipe_get_irq_priority(unsigned int irq);

void __ipipe_handle_irq(unsigned int irq, struct pt_regs *regs);

void __ipipe_call_irqtail(unsigned long addr);

extern unsigned long __ipipe_core_clock;

extern unsigned long __ipipe_freq_scale;

extern unsigned long __ipipe_irq_tail_hook;

#ifdef CONFIG_BF561
#define bfin_write_TIMER_DISABLE(val)	bfin_write_TMRS8_DISABLE(val)
#define bfin_write_TIMER_ENABLE(val)	bfin_write_TMRS8_ENABLE(val)
#define bfin_write_TIMER_STATUS(val)	bfin_write_TMRS8_STATUS(val)
#define bfin_read_TIMER_STATUS()	bfin_read_TMRS8_STATUS()
#elif defined(CONFIG_BF54x)
#define bfin_write_TIMER_DISABLE(val)	bfin_write_TIMER_DISABLE0(val)
#define bfin_write_TIMER_ENABLE(val)	bfin_write_TIMER_ENABLE0(val)
#define bfin_write_TIMER_STATUS(val)	bfin_write_TIMER_STATUS0(val)
#define bfin_read_TIMER_STATUS(val)	bfin_read_TIMER_STATUS0(val)
#endif

#define __ipipe_root_tick_p(regs)	((regs->ipend & 0x10) != 0)

static inline void ipipe_mute_pic(void) { }

static inline void ipipe_unmute_pic(void) { }

static inline void ipipe_notify_root_preemption(void) { }

#endif /* CONFIG_IPIPE */

#ifdef CONFIG_TICKSOURCE_CORETMR
#define IRQ_SYSTMR		IRQ_CORETMR
#define IRQ_PRIOTMR		IRQ_CORETMR
#else
#define IRQ_SYSTMR		IRQ_TIMER0
#define IRQ_PRIOTMR		CONFIG_IRQ_TIMER0
#endif

#endif	/* !__ASM_BLACKFIN_IPIPE_H */
