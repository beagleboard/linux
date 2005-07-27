/*
 * linux/arch/arm/mach-omap/dsp/uaccess_dsp.S
 *
 * user memory access functions for DSP driver
 *
 * Copyright (C) 2004,2005 Nokia Corporation
 *
 * Written by Toshihiro Kobayashi <toshihiro.kobayashi@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * 2004/06/29:  DSP Gateway version 3.3
 */

#include <linux/linkage.h>
#include <asm/assembler.h>

		.text

/* Prototype: int __arch_copy_to_user_dsp_2b(void *to, const char *from)
 * Purpose  : copy 2 bytes to user memory from kernel(DSP) memory
 *            escaping from unexpected byte swap using __arch_copy_to_user()
 *            in OMAP architecture.
 * Params   : to   - user memory
 *          : from - kernel(DSP) memory
 * Returns  : success = 0, failure = 2
 */

ENTRY(__arch_copy_to_user_dsp_2b)
		stmfd	sp!, {r4, lr}
		ldrb	r3, [r1], #1
		ldrb	r4, [r1], #1
USER(		strbt	r4, [r0], #1)			@ May fault
USER(		strbt	r3, [r0], #1)			@ May fault
		mov	r0, #0
		LOADREGS(fd,sp!,{r4, pc})

		.section .fixup,"ax"
		.align	0
9001:		mov	r0, #2
		LOADREGS(fd,sp!, {r4, pc})
		.previous

/* Prototype: unsigned long __arch_copy_from_user_dsp_2b(void *to, const void *from);
 * Purpose  : copy 2 bytes from user memory to kernel(DSP) memory
 *            escaping from unexpected byte swap using __arch_copy_to_user()
 *            in OMAP architecture.
 * Params   : to   - kernel (DSP) memory
 *          : from - user memory
 * Returns  : success = 0, failure = 2
 */

ENTRY(__arch_copy_from_user_dsp_2b)
		stmfd	sp!, {r4, lr}
USER(		ldrbt	r3, [r1], #1)			@ May fault
USER(		ldrbt	r4, [r1], #1)			@ May fault
		strb	r4, [r0], #1
		strb	r3, [r0], #1
		mov	r0, #0
		LOADREGS(fd,sp!,{r4, pc})

		.section .fixup,"ax"
		.align	0
9001:		mov	r3, #0
		strh	r3, [r0], #2
		mov	r0, #2
		LOADREGS(fd,sp!, {r4, pc})
		.previous
