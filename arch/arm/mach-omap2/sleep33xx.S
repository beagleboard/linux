/*
 * Low level suspend code for AM33XX SoCs
 *
 * Copyright (C) 2012-2015 Texas Instruments Incorporated - http://www.ti.com/
 *	Vaibhav Bedia, Dave Gerlach
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
#include <asm/memory.h>
#include <asm/assembler.h>
#include <linux/platform_data/pm33xx.h>

#include "iomap.h"
#include "cm33xx.h"

/* replicated define because linux/bitops.h cannot be included in assembly */
#define BIT(nr)			(1 << (nr))

#define AM33XX_CM_CLKCTRL_MODULEMODE_DISABLE			0x0003
#define AM33XX_CM_CLKCTRL_MODULEMODE_ENABLE			0x0002

	.text
	.align 3

ENTRY(am33xx_do_wfi)
	stmfd	sp!, {r4 - r11, lr}	@ save registers on stack

	/* Save wfi_flags argument and move to a preserved register */
	str	r0, wfi_flags
	mov	r4, r0

	/* Only flush cache is we know we are losing MPU context */
	tst	r4, #WFI_FLAG_FLUSH_CACHE
	beq	cache_skip_flush

	/*
	 * Flush all data from the L1 and L2 data cache before disabling
	 * SCTLR.C bit.
	 */
	ldr	r1, kernel_flush
	blx	r1

	/*
	 * Clear the SCTLR.C bit to prevent further data cache
	 * allocation. Clearing SCTLR.C would make all the data accesses
	 * strongly ordered and would not hit the cache.
	 */
	mrc	p15, 0, r0, c1, c0, 0
	bic	r0, r0, #(1 << 2)	@ Disable the C bit
	mcr	p15, 0, r0, c1, c0, 0
	isb

	/*
	 * Invalidate L1 and L2 data cache.
	 */
	ldr	r1, kernel_flush
	blx	r1

	/* v7_flush_dcache_all doesn't preserve registers so reload wfi_flags*/
	ldr	r4, wfi_flags

cache_skip_flush:
	/* Only necessary if PER is losing context */
	tst	r4, #WFI_FLAG_SAVE_EMIF
	beq	emif_skip_save

	ldr	r1, ti_emif_save_context
	blx	r1

emif_skip_save:
	/* Check if we want self refresh, if so enter SR and disable EMIF */
	tst	r4, #WFI_FLAG_SELF_REFRESH
	beq	emif_skip_enter_sr

	ldr	r1, ti_emif_enter_sr
	blx	r1

	/* Disable EMIF */
	ldr     r1, virt_emif_clkctrl
	ldr     r2, [r1]
	bic     r2, r2, #AM33XX_CM_CLKCTRL_MODULEMODE_DISABLE
	str     r2, [r1]

	ldr	r1, virt_emif_clkctrl
wait_emif_disable:
	ldr	r2, [r1]
	ldr	r3, module_disabled_val
	cmp	r2, r3
	bne	wait_emif_disable

emif_skip_enter_sr:
	tst	r4, #WFI_FLAG_WAKE_M3
	beq	wkup_m3_skip

	/*
	 * For the MPU WFI to be registered as an interrupt
	 * to WKUP_M3, MPU_CLKCTRL.MODULEMODE needs to be set
	 * to DISABLED
	 */
	ldr	r1, virt_mpu_clkctrl
	ldr	r2, [r1]
	bic	r2, r2, #AM33XX_CM_CLKCTRL_MODULEMODE_DISABLE
	str	r2, [r1]

wkup_m3_skip:
	/*
	 * Execute an ISB instruction to ensure that all of the
	 * CP15 register changes have been committed.
	 */
	isb

	/*
	 * Execute a barrier instruction to ensure that all cache,
	 * TLB and branch predictor maintenance operations issued
	 * have completed.
	 */
	dsb
	dmb

	/*
	 * Execute a WFI instruction and wait until the
	 * STANDBYWFI output is asserted to indicate that the
	 * CPU is in idle and low power state. CPU can specualatively
	 * prefetch the instructions so add NOPs after WFI. Thirteen
	 * NOPs as per Cortex-A8 pipeline.
	 */
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

	/* We come here in case of an abort due to a late interrupt */

	/* Set MPU_CLKCTRL.MODULEMODE back to ENABLE */
	ldr	r1, virt_mpu_clkctrl
	mov	r2, #AM33XX_CM_CLKCTRL_MODULEMODE_ENABLE
	str	r2, [r1]

	/* Only necessary if PER is losing context */
	tst	r4, #WFI_FLAG_SELF_REFRESH
	beq	emif_skip_exit_sr_abt

	/* Re-enable EMIF */
	ldr	r1, virt_emif_clkctrl
	mov	r2, #AM33XX_CM_CLKCTRL_MODULEMODE_ENABLE
	str	r2, [r1]
wait_emif_enable:
	ldr	r3, [r1]
	cmp	r2, r3
	bne	wait_emif_enable

	ldr	r1, ti_emif_abort_sr
	blx	r1

emif_skip_exit_sr_abt:
	tst	r4, #WFI_FLAG_FLUSH_CACHE
	beq	cache_skip_restore

	/*
	 * Set SCTLR.C bit to allow data cache allocation
	 */
	mrc	p15, 0, r0, c1, c0, 0
	orr	r0, r0, #(1 << 2)	@ Enable the C bit
	mcr	p15, 0, r0, c1, c0, 0
	isb

cache_skip_restore:
	/* Let the suspend code know about the abort */
	mov	r0, #1
	ldmfd	sp!, {r4 - r11, pc}	@ restore regs and return
ENDPROC(am33xx_do_wfi)

	.align
ENTRY(am33xx_resume_offset)
	.word . - am33xx_do_wfi

ENTRY(am33xx_resume_from_deep_sleep)
	/* Re-enable EMIF */
	ldr	r0, phys_emif_clkctrl
	mov	r1, #AM33XX_CM_CLKCTRL_MODULEMODE_ENABLE
	str	r1, [r0]
wait_emif_enable1:
	ldr	r2, [r0]
	cmp	r1, r2
	bne	wait_emif_enable1

	adr	sp, temp_stack

	ldr	r1, ti_emif_restore_context
	blx	r1

	ldr	r1, ti_emif_exit_sr
	blx	r1

resume_to_ddr:
	/* We are back. Branch to the common CPU resume routine */
	mov	r0, #0
	ldr	pc, resume_addr
ENDPROC(am33xx_resume_from_deep_sleep)

/*
 * Local variables
 */
	.align
kernel_flush:
	.word   v7_flush_dcache_all
virt_mpu_clkctrl:
	.word	AM33XX_CM_MPU_MPU_CLKCTRL
virt_emif_clkctrl:
	.word	AM33XX_CM_PER_EMIF_CLKCTRL
phys_emif_clkctrl:
	.word	(AM33XX_CM_BASE + AM33XX_CM_PER_MOD + \
		AM33XX_CM_PER_EMIF_CLKCTRL_OFFSET)
module_disabled_val:
	.word	0x30000
wfi_flags:
	.word	0x00000000

/* DDR related defines */
am33xx_emif_sram_table:
ti_emif_save_context:
	.word	0x00000000
ti_emif_restore_context:
	.word	0x00000000
ti_emif_enter_sr:
	.word	0x00000000
ti_emif_exit_sr:
	.word	0x00000000
ti_emif_abort_sr:
	.word	0x00000000
	.align 3
	.space 64
temp_stack:

ENTRY(am33xx_pm_sram)
.word am33xx_do_wfi
.word am33xx_do_wfi_sz
.word am33xx_resume_offset
.word am33xx_emif_sram_table
rtc_base_virt:
.word 0xdeadbeef
resume_addr:
.word cpu_resume - PAGE_OFFSET + 0x80000000

ENTRY(am33xx_do_wfi_sz)
	.word	. - am33xx_do_wfi
