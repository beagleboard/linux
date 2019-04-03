/* -*- linux-c -*-
 * arch/arm/include/asm/ipipe_base.h
 *
 * Copyright (C) 2007 Gilles Chanteperdrix.
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

#ifndef __ASM_ARM_IPIPE_BASE_H
#define __ASM_ARM_IPIPE_BASE_H

#ifndef __ASSEMBLY__
#include <asm-generic/ipipe.h>
#endif

#ifdef CONFIG_IPIPE

#ifndef __ASSEMBLY__

#ifdef CONFIG_SMP

extern unsigned __ipipe_first_ipi;

#define IPIPE_CRITICAL_IPI	__ipipe_first_ipi
#define IPIPE_HRTIMER_IPI	(IPIPE_CRITICAL_IPI + 1)
#define IPIPE_RESCHEDULE_IPI	(IPIPE_CRITICAL_IPI + 2)
#define IPIPE_SERVICE_VNMI	(IPIPE_CRITICAL_IPI + 3)

#define IPIPE_LAST_IPI		IPIPE_SERVICE_VNMI

#define hard_smp_processor_id()	raw_smp_processor_id()

#ifdef CONFIG_SMP_ON_UP
unsigned __ipipe_processor_id(void);

#define ipipe_processor_id()						\
	({								\
		register unsigned int cpunum __asm__ ("r0");		\
		register unsigned int r1 __asm__ ("r1");		\
		register unsigned int r2 __asm__ ("r2");		\
		register unsigned int r3 __asm__ ("r3");		\
		register unsigned int ip __asm__ ("ip");		\
		register unsigned int lr __asm__ ("lr");		\
		__asm__ __volatile__ ("\n"				\
			"1:	bl __ipipe_processor_id\n"		\
			"	.pushsection \".alt.smp.init\", \"a\"\n" \
			"	.long	1b\n"				\
			"	mov	%0, #0\n"			\
			"	.popsection"				\
				: "=r"(cpunum),	"=r"(r1), "=r"(r2), "=r"(r3), \
				  "=r"(ip), "=r"(lr)			\
				: /* */ : "cc");			\
		cpunum;						\
	})
#else /* !SMP_ON_UP */
#define ipipe_processor_id() raw_smp_processor_id()
#endif /* !SMP_ON_UP */

#define IPIPE_ARCH_HAVE_VIRQ_IPI

#else /* !CONFIG_SMP */
#define ipipe_processor_id()  (0)
#endif /* !CONFIG_IPIPE */

#endif /* !__ASSEMBLY__ */

#define IPIPE_TRAP_MAYDAY        0	/* Internal recovery trap */

/* ARM traps */
#define IPIPE_TRAP_ACCESS	 1	/* Data or instruction access exception */
#define IPIPE_TRAP_SECTION	 2	/* Section fault */
#define IPIPE_TRAP_DABT		 3	/* Generic data abort */
#define IPIPE_TRAP_UNKNOWN	 4	/* Unknown exception */
#define IPIPE_TRAP_BREAK	 5	/* Instruction breakpoint */
#define IPIPE_TRAP_FPU		 6	/* Floating point exception */
#define IPIPE_TRAP_VFP		 7	/* VFP floating point exception */
#define IPIPE_TRAP_UNDEFINSTR	 8	/* Undefined instruction */
#define IPIPE_TRAP_ALIGNMENT	 9	/* Unaligned access exception */
#define IPIPE_NR_FAULTS         10

#endif /* CONFIG_IPIPE */

#endif /* __ASM_ARM_IPIPE_BASE_H */
