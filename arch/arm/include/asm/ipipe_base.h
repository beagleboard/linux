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

#include <asm/irq.h>		/* For NR_IRQS */

#ifdef CONFIG_IPIPE

#define IPIPE_NR_ROOT_IRQS	1024

#define IPIPE_NR_XIRQS		IPIPE_NR_ROOT_IRQS

#ifdef CONFIG_SMP

extern unsigned __ipipe_first_ipi;

#define IPIPE_CRITICAL_IPI	__ipipe_first_ipi
#define IPIPE_HRTIMER_IPI	(IPIPE_CRITICAL_IPI + 1)
#define IPIPE_RESCHEDULE_IPI	(IPIPE_CRITICAL_IPI + 2)
#define IPIPE_SERVICE_VNMI	(IPIPE_CRITICAL_IPI + 3)

#define IPIPE_LAST_IPI		IPIPE_SERVICE_VNMI

#ifdef CONFIG_IPIPE_LEGACY
#define hard_smp_processor_id()						\
	({								\
		unsigned int cpunum;					\
		__asm__ __volatile__ ("\n"				\
			"1:	mrc p15, 0, %0, c0, c0, 5\n"		\
			"	.pushsection \".alt.smp.init\", \"a\"\n" \
			"	.long	1b\n"				\
			"	mov	%0, #0\n"			\
			"	.popsection"				\
				      : "=r" (cpunum));			\
		cpunum &= 0xFF;						\
	})
extern u32 __cpu_reverse_map[];
#define ipipe_processor_id()  (__cpu_reverse_map[hard_smp_processor_id()])

#else /* !legacy */
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
#endif /* !legacy */

#define IPIPE_ARCH_HAVE_VIRQ_IPI

#else /* !CONFIG_SMP */
#define ipipe_processor_id()  (0)
#endif /* !CONFIG_IPIPE */

/* ARM traps */
#define IPIPE_TRAP_ACCESS	 0	/* Data or instruction access exception */
#define IPIPE_TRAP_SECTION	 1	/* Section fault */
#define IPIPE_TRAP_DABT		 2	/* Generic data abort */
#define IPIPE_TRAP_UNKNOWN	 3	/* Unknown exception */
#define IPIPE_TRAP_BREAK	 4	/* Instruction breakpoint */
#define IPIPE_TRAP_FPU		 5	/* Floating point exception */
#define IPIPE_TRAP_VFP		 6	/* VFP floating point exception */
#define IPIPE_TRAP_UNDEFINSTR	 7	/* Undefined instruction */
#define IPIPE_TRAP_ALIGNMENT	 8	/* Unaligned access exception */
#define IPIPE_TRAP_MAYDAY        9	/* Internal recovery trap */
#define IPIPE_NR_FAULTS         10

#ifndef __ASSEMBLY__

#ifdef CONFIG_SMP

void ipipe_stall_root(void);

unsigned long ipipe_test_and_stall_root(void);

unsigned long ipipe_test_root(void);

#else /* !CONFIG_SMP */

#include <asm/irqflags.h>

#if __GNUC__ >= 4
/* Alias to ipipe_root_cpudom_var(status) */
extern unsigned long __ipipe_root_status;
#else
extern unsigned long *const __ipipe_root_status_addr;
#define __ipipe_root_status	(*__ipipe_root_status_addr)
#endif

static inline void ipipe_stall_root(void)
{
	unsigned long flags;

	flags = hard_local_irq_save();
	__ipipe_root_status |= 1;
	hard_local_irq_restore(flags);
}

static inline unsigned ipipe_test_root(void)
{
	return __ipipe_root_status & 1;
}

static inline unsigned ipipe_test_and_stall_root(void)
{
	unsigned long flags, res;

	flags = hard_local_irq_save();
	res = __ipipe_root_status;
	__ipipe_root_status = res | 1;
	hard_local_irq_restore(flags);

	return res & 1;
}

#endif	/* !CONFIG_SMP */

#endif /* !__ASSEMBLY__ */

#ifdef CONFIG_IPIPE_LEGACY
#define __IPIPE_FEATURE_PREEMPTIBLE_SWITCH	1
#define __IPIPE_FEATURE_SYSINFO_V2		1

#ifdef CONFIG_VFP
#define __IPIPE_FEATURE_VFP_SAFE		1
#endif

#ifdef CONFIG_IPIPE_ARM_KUSER_TSC
#define __IPIPE_FEATURE_KUSER_TSC		1
#endif
#endif /* CONFIG_IPIPE_LEGACY */

#endif /* CONFIG_IPIPE */

#endif /* __ASM_ARM_IPIPE_BASE_H */
