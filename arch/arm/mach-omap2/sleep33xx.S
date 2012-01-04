/*
 * Low level suspend code for AM33XX SoCs
 *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/linkage.h>
#include <linux/init.h>
#include <asm/memory.h>
#include <asm/assembler.h>
#include <plat/sram.h>
#include <mach/io.h>

	.align 3
ENTRY(am33xx_do_wfi)
	wfi
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop

	/* We are back. Branch to the common CPU resume routine */
ENTRY(am33xx_resume_vector)
	ldr	pc, resume_addr

resume_addr:
	.word	cpu_resume - PAGE_OFFSET + 0x80000000

ENTRY(am33xx_do_wfi_sz)
	.word	. - am33xx_do_wfi
