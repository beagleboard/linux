/*
 * Copyright (C) 2008 Gilles Chanteperdrix <gilles.chanteperdrix@xenomai.org>.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA.
 */
#ifndef _COBALT_ARM_ASM_UAPI_ARITH_H
#define _COBALT_ARM_ASM_UAPI_ARITH_H

#include <asm/xenomai/uapi/features.h>

#if __LINUX_ARM_ARCH__ >= 4 && (!defined(CONFIG_THUMB2_KERNEL) || !defined(CONFIG_FTRACE))
static inline __attribute__((__const__)) unsigned long long
mach_arm_nodiv_ullimd(const unsigned long long op,
		       const unsigned long long frac,
		       const unsigned rhs_integ);

#define xnarch_nodiv_ullimd(op, frac, integ) \
	mach_arm_nodiv_ullimd((op), (frac), (integ))

static inline __attribute__((__const__)) long long
mach_arm_nodiv_llimd(const long long op,
		       const unsigned long long frac,
		       const unsigned rhs_integ);

#define xnarch_nodiv_llimd(op, frac, integ) \
	mach_arm_nodiv_llimd((op), (frac), (integ))
#else /* arm <= v3 */
#define xnarch_add96and64(l0, l1, l2, s0, s1)		\
	do {						\
		__asm__ ("adds %2, %2, %4\n\t"		\
			 "adcs %1, %1, %3\n\t"		\
			 "adc %0, %0, #0\n\t"		\
			 : "+r"(l0), "+r"(l1), "+r"(l2)	\
			 : "r"(s0), "r"(s1): "cc");	\
	} while (0)
#endif /* arm <= v3 */

#include <cobalt/uapi/asm-generic/arith.h>

#if __LINUX_ARM_ARCH__ >= 4 && (!defined(CONFIG_THUMB2_KERNEL) || !defined(CONFIG_FTRACE))
#define mach_arm_nodiv_ullimd_str			\
	"umull %[tl], %[rl], %[opl], %[fracl]\n\t"	\
	"umull %[rm], %[rh], %[oph], %[frach]\n\t"	\
	"adds %[rl], %[rl], %[tl], lsr #31\n\t"		\
	"adcs %[rm], %[rm], #0\n\t"			\
	"adc %[rh], %[rh], #0\n\t"			\
	"umull %[tl], %[th], %[oph], %[fracl]\n\t"	\
	"adds %[rl], %[rl], %[tl]\n\t"			\
	"adcs %[rm], %[rm], %[th]\n\t"			\
	"adc %[rh], %[rh], #0\n\t"			\
	"umull %[tl], %[th], %[opl], %[frach]\n\t"	\
	"adds %[rl], %[rl], %[tl]\n\t"			\
	"adcs %[rm], %[rm], %[th]\n\t"			\
	"adc %[rh], %[rh], #0\n\t"			\
	"umlal %[rm], %[rh], %[opl], %[integ]\n\t"	\
	"mla %[rh], %[oph], %[integ], %[rh]\n\t"

static inline __attribute__((__const__)) unsigned long long
mach_arm_nodiv_ullimd(const unsigned long long op,
		       const unsigned long long frac,
		       const unsigned rhs_integ)
{
	register unsigned rl __asm__("r5");
	register unsigned rm __asm__("r0");
	register unsigned rh __asm__("r1");
	register unsigned fracl __asm__ ("r2");
	register unsigned frach __asm__ ("r3");
	register unsigned integ __asm__("r4") = rhs_integ;
	register unsigned opl __asm__ ("r6");
	register unsigned oph __asm__ ("r7");
	register unsigned tl __asm__("r8");
	register unsigned th __asm__("r9");

	xnarch_u64tou32(op, oph, opl);
	xnarch_u64tou32(frac, frach, fracl);

	__asm__ (mach_arm_nodiv_ullimd_str
		 : [rl]"=r"(rl), [rm]"=r"(rm), [rh]"=r"(rh),
		   [tl]"=r"(tl), [th]"=r"(th)
		 : [opl]"r"(opl), [oph]"r"(oph),
		   [fracl]"r"(fracl), [frach]"r"(frach),
		   [integ]"r"(integ)
		 : "cc");

	return xnarch_u64fromu32(rh, rm);
}

static inline __attribute__((__const__)) long long
mach_arm_nodiv_llimd(const long long op,
		       const unsigned long long frac,
		       const unsigned rhs_integ)
{
	register unsigned rl __asm__("r5");
	register unsigned rm __asm__("r0");
	register unsigned rh __asm__("r1");
	register unsigned fracl __asm__ ("r2");
	register unsigned frach __asm__ ("r3");
	register unsigned integ __asm__("r4") = rhs_integ;
	register unsigned opl __asm__ ("r6");
	register unsigned oph __asm__ ("r7");
	register unsigned tl __asm__("r8");
	register unsigned th __asm__("r9");
	register unsigned s __asm__("r10");

	xnarch_u64tou32(op, oph, opl);
	xnarch_u64tou32(frac, frach, fracl);

	__asm__ ("movs %[s], %[oph], lsr #30\n\t"
		 "beq 1f\n\t"
		 "rsbs  %[opl], %[opl], #0\n\t"
		 "sbc  %[oph], %[oph], %[oph], lsl #1\n"
		 "1:\t"
		 mach_arm_nodiv_ullimd_str
		 "teq %[s], #0\n\t"
		 "beq 2f\n\t"
		 "rsbs  %[rm], %[rm], #0\n\t"
		 "sbc  %[rh], %[rh], %[rh], lsl #1\n"
		 "2:\t"
		 : [rl]"=r"(rl), [rm]"=r"(rm), [rh]"=r"(rh),
		   [tl]"=r"(tl), [th]"=r"(th), [s]"=r"(s)
		 : [opl]"r"(opl), [oph]"r"(oph),
		   [fracl]"r"(fracl), [frach]"r"(frach),
		   [integ]"r"(integ)
		 : "cc");

	return xnarch_u64fromu32(rh, rm);
}
#endif /* arm >= v4 */

#endif /* _COBALT_ARM_ASM_UAPI_ARITH_H */
