/* -*- linux-c -*-
 * arch/arm/include/asm/ipipe_hwirq.h
 *
 * Copyright (C) 2002-2005 Philippe Gerum.
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
 */

#ifndef _ASM_ARM_IPIPE_HWIRQ_H
#define _ASM_ARM_IPIPE_HWIRQ_H

#define hard_local_irq_restore_notrace(x)				\
	__asm__ __volatile__(						\
	"msr	daif, %0"	\
	:								\
	: "r" (x)							\
	: "memory", "cc")

static inline void hard_local_irq_disable_notrace(void)
{
	__asm__ __volatile__("msr daifset, #2" : : : "memory", "cc");
}

static inline void hard_local_irq_enable_notrace(void)
{
	__asm__ __volatile__("msr daifclr, #2" : : : "memory", "cc");
}

static inline void hard_local_fiq_disable_notrace(void)
{
	__asm__ __volatile__("msr daifset, #1" : : : "memory", "cc");
}

static inline void hard_local_fiq_enable_notrace(void)
{
	__asm__ __volatile__("msr daifclr, #1" : : : "memory", "cc");
}

static inline unsigned long hard_local_irq_save_notrace(void)
{
	unsigned long res;
	__asm__ __volatile__(
		"mrs	%0, daif\n"
		"msr daifset, #2"
		: "=r" (res) : : "memory", "cc");
	return res;
}

#ifdef CONFIG_IPIPE

#include <linux/ipipe_trace.h>

static inline int arch_irqs_disabled_flags(unsigned long flags)
{
	return (int)((flags) & PSR_I_BIT);
}

static inline unsigned long hard_local_save_flags(void)
{
	unsigned long flags;
	__asm__ __volatile__(
		"mrs	%0, daif"
		: "=r" (flags) : : "memory", "cc");
	return flags;
}

#define hard_irqs_disabled_flags(flags) arch_irqs_disabled_flags(flags)

static inline int hard_irqs_disabled(void)
{
	return hard_irqs_disabled_flags(hard_local_save_flags());
}

#ifdef CONFIG_IPIPE_TRACE_IRQSOFF

static inline void hard_local_irq_disable(void)
{
	if (!hard_irqs_disabled()) {
		hard_local_irq_disable_notrace();
		ipipe_trace_begin(0x80000000);
	}
}

static inline void hard_local_irq_enable(void)
{
	if (hard_irqs_disabled()) {
		ipipe_trace_end(0x80000000);
		hard_local_irq_enable_notrace();
	}
}

static inline unsigned long hard_local_irq_save(void)
{
	unsigned long flags;

	flags = hard_local_irq_save_notrace();
	if (!arch_irqs_disabled_flags(flags))
		ipipe_trace_begin(0x80000001);

	return flags;
}

static inline void hard_local_irq_restore(unsigned long x)
{
	if (!arch_irqs_disabled_flags(x))
		ipipe_trace_end(0x80000001);

	hard_local_irq_restore_notrace(x);
}

#else /* !CONFIG_IPIPE_TRACE_IRQSOFF */

#define hard_local_irq_disable    hard_local_irq_disable_notrace
#define hard_local_irq_enable     hard_local_irq_enable_notrace
#define hard_local_irq_save       hard_local_irq_save_notrace
#define hard_local_irq_restore    hard_local_irq_restore_notrace

#endif /* CONFIG_IPIPE_TRACE_IRQSOFF */

#define arch_local_irq_disable()		\
	({					\
		ipipe_stall_root();		\
		barrier();			\
	})

#define arch_local_irq_enable()				\
	do {						\
		barrier();				\
		ipipe_unstall_root();			\
	} while (0)

#define local_fiq_enable() hard_local_fiq_enable_notrace()

#define local_fiq_disable() hard_local_fiq_disable_notrace()

#define arch_local_irq_restore(flags)			\
	do {						\
		if (!arch_irqs_disabled_flags(flags))	\
			arch_local_irq_enable();	\
	} while (0)

#define arch_local_irq_save()						\
	({								\
		unsigned long _flags;					\
		_flags = ipipe_test_and_stall_root() << 7;		\
		barrier();						\
		_flags;							\
	})

#define arch_local_save_flags()						\
	({								\
		unsigned long _flags;					\
		_flags = ipipe_test_root() << 7;			\
		barrier();						\
		_flags;							\
	})

#define arch_irqs_disabled()		ipipe_test_root()
#define hard_irq_disable()		hard_local_irq_disable()

static inline unsigned long arch_mangle_irq_bits(int virt, unsigned long real)
{
	/* Merge virtual and real interrupt mask bits into a single
	   32bit word. */
	return (real & ~(1L << 8)) | ((virt != 0) << 8);
}

static inline int arch_demangle_irq_bits(unsigned long *x)
{
	int virt = (*x & (1 << 8)) != 0;
	*x &= ~(1L << 8);
	return virt;
}

#else /* !CONFIG_IPIPE */

#define hard_local_irq_save()		arch_local_irq_save()
#define hard_local_irq_restore(x)	arch_local_irq_restore(x)
#define hard_local_irq_enable()		arch_local_irq_enable()
#define hard_local_irq_disable()	arch_local_irq_disable()
#define hard_irqs_disabled()		irqs_disabled()

#define hard_cond_local_irq_enable()		do { } while(0)
#define hard_cond_local_irq_disable()		do { } while(0)
#define hard_cond_local_irq_save()		0
#define hard_cond_local_irq_restore(flags)	do { (void)(flags); } while(0)

#endif /* !CONFIG_IPIPE */

#if defined(CONFIG_SMP) && defined(CONFIG_IPIPE)
#define hard_smp_local_irq_save()		hard_local_irq_save()
#define hard_smp_local_irq_restore(flags)	hard_local_irq_restore(flags)
#else /* !CONFIG_SMP */
#define hard_smp_local_irq_save()		0
#define hard_smp_local_irq_restore(flags)	do { (void)(flags); } while(0)
#endif /* CONFIG_SMP */

#endif /* _ASM_ARM_IPIPE_HWIRQ_H */
