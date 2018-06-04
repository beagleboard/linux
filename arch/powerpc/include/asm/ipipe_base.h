/* -*- linux-c -*-
 * include/asm-powerpc/ipipe_base.h
 *
 * Copyright (C) 2007-2012 Philippe Gerum.
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

#ifndef __ASM_POWERPC_IPIPE_BASE_H
#define __ASM_POWERPC_IPIPE_BASE_H

#ifdef CONFIG_IPIPE

#define IPIPE_NR_XIRQS		CONFIG_NR_IRQS
#ifdef CONFIG_PPC64
#define IPIPE_IRQ_ISHIFT	6		/* 64-bit arch. */
#else
#define IPIPE_IRQ_ISHIFT	5		/* 32-bit arch. */
#endif

/*
 * The first virtual interrupt is reserved for the timer (see
 * __ipipe_early_core_setup).
 */
#define IPIPE_TIMER_VIRQ	(IPIPE_VIRQ_BASE + 0)
#define IPIPE_DOORBELL_VIRQ	(IPIPE_VIRQ_BASE + 1)

#ifdef CONFIG_SMP
/*
 * These are virtual IPI numbers. The OpenPIC supports only 4 IPIs and
 * all are already used by Linux. The virtualization layer is
 * implemented by piggybacking the debugger break IPI 0x3,
 * which is demultiplexed in __ipipe_ipi_demux().
 */
#define IPIPE_CRITICAL_IPI	(IPIPE_VIRQ_BASE + 2)
#define IPIPE_HRTIMER_IPI	(IPIPE_VIRQ_BASE + 3)
#define IPIPE_RESCHEDULE_IPI	(IPIPE_VIRQ_BASE + 4)
#define IPIPE_BASE_IPI_OFFSET	IPIPE_CRITICAL_IPI

/* these are bit numbers in practice */
#define IPIPE_MSG_CRITICAL_IPI		0
#define IPIPE_MSG_HRTIMER_IPI		(IPIPE_MSG_CRITICAL_IPI + 1)
#define IPIPE_MSG_RESCHEDULE_IPI	(IPIPE_MSG_CRITICAL_IPI + 2)
#define IPIPE_MSG_IPI_MASK	((1UL << IPIPE_MSG_CRITICAL_IPI) |	\
				 (1UL << IPIPE_MSG_HRTIMER_IPI) |	\
				 (1UL << IPIPE_MSG_RESCHEDULE_IPI))

#define ipipe_processor_id()	raw_smp_processor_id()

#else  /* !CONFIG_SMP */
#define ipipe_processor_id()	0
#endif /* CONFIG_SMP */

/* traps */
#define IPIPE_TRAP_ACCESS	 0	/* Data or instruction access exception */
#define IPIPE_TRAP_ALIGNMENT	 1	/* Alignment exception */
#define IPIPE_TRAP_ALTUNAVAIL	 2	/* Altivec unavailable */
#define IPIPE_TRAP_PCE		 3	/* Program check exception */
#define IPIPE_TRAP_MCE		 4	/* Machine check exception */
#define IPIPE_TRAP_UNKNOWN	 5	/* Unknown exception */
#define IPIPE_TRAP_IABR		 6	/* Instruction breakpoint */
#define IPIPE_TRAP_RM		 7	/* Run mode exception */
#define IPIPE_TRAP_SSTEP	 8	/* Single-step exception */
#define IPIPE_TRAP_NREC		 9	/* Non-recoverable exception */
#define IPIPE_TRAP_SOFTEMU	10	/* Software emulation */
#define IPIPE_TRAP_DEBUG	11	/* Debug exception */
#define IPIPE_TRAP_SPE		12	/* SPE exception */
#define IPIPE_TRAP_ALTASSIST	13	/* Altivec assist exception */
#define IPIPE_TRAP_CACHE	14	/* Cache-locking exception (FSL) */
#define IPIPE_TRAP_KFPUNAVAIL	15	/* FP unavailable exception */
#define IPIPE_TRAP_MAYDAY	16	/* Internal recovery trap */
#define IPIPE_NR_FAULTS		17

#ifndef __ASSEMBLY__

#ifdef CONFIG_SMP

void ipipe_stall_root(void);

unsigned long ipipe_test_and_stall_root(void);

unsigned long ipipe_test_root(void);

#else /* !CONFIG_SMP */

#include <linux/bitops.h>

extern unsigned long __ipipe_root_status;

static __inline__ void ipipe_stall_root(void)
{
	volatile unsigned long *p = &__ipipe_root_status;
	ipipe_root_only();
	set_bit(0, p);
}

static __inline__ unsigned long ipipe_test_and_stall_root(void)
{
	volatile unsigned long *p = &__ipipe_root_status;
	ipipe_root_only();
	return test_and_set_bit(0, p);
}

static __inline__ unsigned long ipipe_test_root(void)
{
	volatile unsigned long *p = &__ipipe_root_status;
	return test_bit(0, p);
}

#endif /* !CONFIG_SMP */

#endif /* !__ASSEMBLY__ */

#ifdef CONFIG_IPIPE_LEGACY
#define __IPIPE_FEATURE_PREEMPTIBLE_SWITCH	1
#define __IPIPE_FEATURE_HARDENED_SWITCHMM	1
#endif

#endif /* !CONFIG_IPIPE */

#endif	/* !__ASM_POWERPC_IPIPE_BASE_H */
