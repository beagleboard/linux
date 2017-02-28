/*   -*- linux-c -*-
 *   arch/x86/include/asm/ipipe_base.h
 *
 *   Copyright (C) 2007-2012 Philippe Gerum.
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

#ifndef __X86_IPIPE_BASE_H
#define __X86_IPIPE_BASE_H

#include <asm/irq_vectors.h>
#include <asm/bitsperlong.h>

#ifdef CONFIG_X86_32
/* 32 from IDT + iret_error + mayday trap */
#define IPIPE_TRAP_MAYDAY	33	/* Internal recovery trap */
#define IPIPE_NR_FAULTS		34
#else
/* 32 from IDT + mayday trap */
#define IPIPE_TRAP_MAYDAY	32	/* Internal recovery trap */
#define IPIPE_NR_FAULTS		33
#endif

#if defined(CONFIG_X86_64) || defined(CONFIG_X86_LOCAL_APIC)
/*
 * Special APIC interrupts are mapped above the last defined external
 * IRQ number.
 */
#define nr_apic_vectors	        (NR_VECTORS - FIRST_SYSTEM_VECTOR)
#define IPIPE_FIRST_APIC_IRQ	NR_IRQS
#define IPIPE_HRTIMER_IPI	ipipe_apic_vector_irq(IPIPE_HRTIMER_VECTOR)
#ifdef CONFIG_SMP
#define IPIPE_RESCHEDULE_IPI	ipipe_apic_vector_irq(IPIPE_RESCHEDULE_VECTOR)
#define IPIPE_CRITICAL_IPI	ipipe_apic_vector_irq(IPIPE_CRITICAL_VECTOR)
#endif /* CONFIG_SMP */
#define IPIPE_NR_XIRQS		(NR_IRQS + nr_apic_vectors)
#define ipipe_apic_irq_vector(irq)  ((irq) - IPIPE_FIRST_APIC_IRQ + FIRST_SYSTEM_VECTOR)
#define ipipe_apic_vector_irq(vec)  ((vec) - FIRST_SYSTEM_VECTOR + IPIPE_FIRST_APIC_IRQ)
#else /* !(CONFIG_X86_64 || CONFIG_X86_LOCAL_APIC) */
#define IPIPE_NR_XIRQS		NR_IRQS
#endif /* !(CONFIG_X86_64 || CONFIG_X86_LOCAL_APIC) */

#ifndef __ASSEMBLY__

#include <asm/apicdef.h>

#ifdef CONFIG_X86_32
# include "ipipe_32.h"
#else
# include "ipipe_64.h"
#endif

struct pt_regs;
struct irq_desc;
struct ipipe_vm_notifier;

static inline unsigned __ipipe_get_irq_vector(int irq)
{
#ifdef CONFIG_X86_IO_APIC
	unsigned int __ipipe_get_ioapic_irq_vector(int irq);
	return __ipipe_get_ioapic_irq_vector(irq);
#elif defined(CONFIG_X86_LOCAL_APIC)
	return irq >= IPIPE_FIRST_APIC_IRQ && irq < IPIPE_NR_XIRQS ?
		ipipe_apic_irq_vector(irq) : irq + IRQ0_VECTOR;
#else
	return irq + IRQ0_VECTOR;
#endif
}

void __ipipe_halt_root(int use_mwait);

void ipipe_hrtimer_interrupt(void);

void ipipe_reschedule_interrupt(void);

void ipipe_critical_interrupt(void);

int __ipipe_handle_irq(struct pt_regs *regs);

void __ipipe_handle_vm_preemption(struct ipipe_vm_notifier *nfy);

extern int __ipipe_hrtimer_irq;

#ifdef CONFIG_SMP

#ifdef CONFIG_X86_32
#define GET_ROOT_STATUS_ADDR			\
	"pushfl; cli;"				\
	"movl %%fs:this_cpu_off, %%eax;"	\
	"lea ipipe_percpu(%%eax), %%eax;"
#define PUT_ROOT_STATUS_ADDR	"popfl;"
#define TEST_AND_SET_ROOT_STATUS		\
	"btsl $0,(%%eax);"
#define TEST_ROOT_STATUS			\
	"btl $0,(%%eax);"
#define ROOT_TEST_CLOBBER_LIST  "eax"
#else /* CONFIG_X86_64 */
#define GET_ROOT_STATUS_ADDR			\
	"pushfq; cli;"				\
	"movq %%gs:this_cpu_off, %%rax;"	\
	"lea ipipe_percpu(%%rax), %%rax;"
#define PUT_ROOT_STATUS_ADDR	"popfq;"
#define TEST_AND_SET_ROOT_STATUS		\
	"btsl $0,(%%rax);"
#define TEST_ROOT_STATUS			\
	"btl $0,(%%rax);"
#define ROOT_TEST_CLOBBER_LIST  "rax"
#endif /* CONFIG_X86_64 */

static inline void ipipe_stall_root(void)
{
	__asm__ __volatile__(GET_ROOT_STATUS_ADDR
			     TEST_AND_SET_ROOT_STATUS
			     PUT_ROOT_STATUS_ADDR
			     : : : ROOT_TEST_CLOBBER_LIST, "memory");
}

static inline unsigned long ipipe_test_and_stall_root(void)
{
	int oldbit;

	__asm__ __volatile__(GET_ROOT_STATUS_ADDR
			     TEST_AND_SET_ROOT_STATUS
			     "sbbl %0,%0;"
			     PUT_ROOT_STATUS_ADDR
			     :"=r" (oldbit)
			     : : ROOT_TEST_CLOBBER_LIST, "memory");
	return oldbit;
}

static inline unsigned long ipipe_test_root(void)
{
	int oldbit;

	__asm__ __volatile__(GET_ROOT_STATUS_ADDR
			     TEST_ROOT_STATUS
			     "sbbl %0,%0;"
			     PUT_ROOT_STATUS_ADDR
			     :"=r" (oldbit)
			     : : ROOT_TEST_CLOBBER_LIST);
	return oldbit;
}

#else /* !CONFIG_SMP */

extern unsigned long __ipipe_root_status;

static inline void ipipe_stall_root(void)
{
	volatile unsigned long *p = &__ipipe_root_status;
	__asm__ __volatile__("btsl $0,%0;"
			     :"+m" (*p) : : "memory");
}

static inline unsigned long ipipe_test_and_stall_root(void)
{
	volatile unsigned long *p = &__ipipe_root_status;
	int oldbit;

	__asm__ __volatile__("btsl $0,%1;"
			     "sbbl %0,%0;"
			     :"=r" (oldbit), "+m" (*p)
			     : : "memory");
	return oldbit;
}

static inline unsigned long ipipe_test_root(void)
{
	volatile unsigned long *p = &__ipipe_root_status;
	int oldbit;

	__asm__ __volatile__("btl $0,%1;"
			     "sbbl %0,%0;"
			     :"=r" (oldbit)
			     :"m" (*p));
	return oldbit;
}

#endif /* !CONFIG_SMP */

#ifdef CONFIG_IPIPE_LEGACY
#define __ipipe_tick_irq	__ipipe_hrtimer_irq
/*
 * The current Linux task is read from the PDA on x86, so this is
 * always safe, regardless of the active stack.
 */
#define ipipe_safe_current()	current
#endif

#endif	/* !__ASSEMBLY__ */

#endif	/* !__X86_IPIPE_BASE_H */
