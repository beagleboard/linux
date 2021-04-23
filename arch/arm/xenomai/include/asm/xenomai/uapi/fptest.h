/*
 * Copyright (C) 2006 Gilles Chanteperdrix <gilles.chanteperdrix@xenomai.org>.
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
#ifndef _COBALT_ARM_ASM_UAPI_FPTEST_H
#define _COBALT_ARM_ASM_UAPI_FPTEST_H

#define __COBALT_HAVE_VFP  0x1

static inline void fp_regs_set(int features, unsigned int val)
{
	unsigned long long e[16];
	unsigned int i;

	if (features & __COBALT_HAVE_VFP) {
		for (i = 0; i < 16; i++)
			e[i] = val;

		/* vldm %0!, {d0-d15},
		   AKA fldmiax %0!, {d0-d15} */
		__asm__ __volatile__("ldc p11, cr0, [%0],#32*4":
				     "=r"(i): "0"(&e[0]): "memory");
	}
}

static inline unsigned int fp_regs_check(int features, unsigned int val,
					 int (*report)(const char *fmt, ...))
{
	unsigned int result = val, i;
	unsigned long long e[16];

	if (features & __COBALT_HAVE_VFP) {
		/* vstm %0!, {d0-d15},
		   AKA fstmiax %0!, {d0-d15} */
		__asm__ __volatile__("stc p11, cr0, [%0],#32*4":
				     "=r"(i): "0"(&e[0]): "memory");

		for (i = 0; i < 16; i++)
			if (e[i] != val) {
				report("d%d: %llu != %u\n", i, e[i], val);
				result = e[i];
			}
	}

	return result;
}

#endif /* !_COBALT_ARM_ASM_UAPI_FPTEST_H */
