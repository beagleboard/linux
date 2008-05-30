/*
 * linux/arch/arm/mach-omap2/sleep.S
 *
 * (C) Copyright 2004
 * Texas Instruments, <www.ti.com>
 * Richard Woodruff <r-woodruff2@ti.com>
 *
 * (C) Copyright 2006 Nokia Corporation
 * Fixed idle loop sleep
 * Igor Stoppa <igor.stoppa@nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR /PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <linux/linkage.h>
#include <asm/assembler.h>
#include <asm/arch/io.h>
#include <asm/arch/pm.h>

#include <asm/arch/omap24xx.h>

#include "sdrc.h"

/* First address of reserved address space?  apparently valid for OMAP2 & 3 */
#define A_SDRC0_V		(0xC0000000)

	.text

/*
 * Forces OMAP into idle state
 *
 * omap242x_idle_loop_suspend() - This bit of code just executes the WFI
 * for normal idles.
 *
 * Note: This code get's copied to internal SRAM at boot. When the OMAP
 *	 wakes up it continues execution at the point it went to sleep.
 */
ENTRY(omap242x_idle_loop_suspend)
	stmfd	sp!, {r0, lr}		@ save registers on stack
	mov	r0, #0x0		@ clear for mrc call
	mcr	p15, 0, r0, c7, c0, 4	@ wait for interrupt
	ldmfd	sp!, {r0, pc}		@ restore regs and return

ENTRY(omap242x_idle_loop_suspend_sz)
	.word	. - omap242x_idle_loop_suspend

/*
 * omap242x_cpu_suspend() - Forces OMAP into deep sleep state by completing
 * SDRC shutdown then ARM shutdown.  Upon wake MPU is back on so just restore
 * SDRC.
 *
 * Input:
 * R0 :	DLL ctrl value pre-Sleep
 *
 * The if the DPLL is going to AutoIdle. It seems like the DPLL may be back on
 * when we get called, but the DLL probably isn't.  We will wait a bit more in
 * case the DPLL isn't quite there yet. The code will wait on DLL for DDR even
 * if in unlocked mode.
 *
 * For less than 242x-ES2.2 upon wake from a sleep mode where the external
 * oscillator was stopped, a timing bug exists where a non-stabilized 12MHz
 * clock can pass into the PRCM can cause problems at DSP and IVA.
 * To work around this the code will switch to the 32kHz source prior to sleep.
 * Post sleep we will shift back to using the DPLL.  Apparently,
 * CM_IDLEST_CLKGEN does not reflect the full clock change so you need to wait
 * 3x12MHz + 3x32kHz clocks for a full switch.
 *
 * The DLL load value is not kept in RETENTION or OFF.	It needs to be restored
 * at wake
 */
ENTRY(omap242x_cpu_suspend)
	stmfd	sp!, {r0 - r12, lr}	@ save registers on stack
	mov	r3, #0x0		@ clear for mrc call
	mcr	p15, 0, r3, c7, c10, 4	@ memory barrier, hope SDR/DDR finished
	nop
	nop
	ldr	r3, omap2_ocs_sdrc_power	@ addr of sdrc power
	ldr	r4, [r3]		@ value of sdrc power
	orr	r4, r4, #0x40		@ enable self refresh on idle req
	mov	r5, #0x2000		@ set delay (DPLL relock + DLL relock)
	str	r4, [r3]		@ make it so
	mov	r2, #0
	nop
	mcr	p15, 0, r2, c7, c0, 4	@ wait for interrupt
	nop
loop:
	subs	r5, r5, #0x1		@ awake, wait just a bit
	bne	loop

	/* The DPLL has on before we take the DDR out of self refresh */
	bic	r4, r4, #0x40		@ now clear self refresh bit.
	str	r4, [r3]		@ put vlaue back.
	ldr	r4, A_SDRC0		@ make a clock happen
	ldr	r4, [r4]
	nop				@ start auto refresh only after clk ok
	movs	r0, r0			@ see if DDR or SDR
	ldrne	r1, omap2_ocs_sdrc_dlla_ctrl	@ get addr of DLL ctrl
	strne	r0, [r1]		@ rewrite DLLA to force DLL reload
	addne	r1, r1, #0x8		@ move to DLLB
	strne	r0, [r1]		@ rewrite DLLB to force DLL reload

	mov	r5, #0x1000
loop2:
	subs	r5, r5, #0x1
	bne	loop2
	/* resume*/
	ldmfd	sp!, {r0 - r12, pc}	@ restore regs and return

omap2_ocs_sdrc_power:
	.word OMAP242X_SDRC_REGADDR(SDRC_POWER)
A_SDRC0:
	.word A_SDRC0_V
omap2_ocs_sdrc_dlla_ctrl:
	.word OMAP242X_SDRC_REGADDR(SDRC_DLLA_CTRL)

ENTRY(omap242x_cpu_suspend_sz)
	.word	. - omap242x_cpu_suspend

