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
#include <mach/io.h>
#include <plat/emif.h>
#include "cm33xx.h"

#include <plat/emif.h>
#include <plat/sram.h>

#include "cm33xx.h"
#include "pm33xx.h"
#include "prm33xx.h"
#include "control.h"

/* We should probably pass in the virtual address of PRCM, Control and EMIF
 * along with the physical addresses
 * load it into the registers and then continue
 */
	.align 3
ENTRY(am33xx_do_wfi)
	stmfd	sp!, {r4 - r11, lr}	@ save registers on stack

	.macro	pll_bypass, name, clk_mode_addr, idlest_addr
pll_bypass_\name:
	ldr	r0, \clk_mode_addr
	ldr	r1, [r0]
	bic	r1, r1, #(7 << 0)
	orr	r1, r1, #0x5
	str	r1, [r0]
	ldr	r0, \idlest_addr
wait_pll_bypass_\name:
	ldr	r1, [r0]
	tst	r1, #0x0
	bne	wait_pll_bypass_\name
	.endm

	.macro	pll_lock, name, clk_mode_addr, idlest_addr
pll_lock_\name:
	ldr	r0, \clk_mode_addr
	ldr	r1, [r0]
	bic	r1, r1, #(7 << 0)
	orr	r1, r1, #0x7
	str	r1, [r0]
	ldr	r0, \idlest_addr
wait_pll_lock_\name:
	ldr	r1, [r0]
	tst	r1, #0x1
	bne	wait_pll_lock_\name
	.endm

	/* EMIF config for low power mode */
	ldr	r0, emif_addr_func
	blx	r0

	str	r0, emif_addr_virt

	/* Ensure that all the writes to DDR leave the A8 */
	dsb
	dmb
	isb

	add	r1, r0, #EMIF4_0_SDRAM_MGMT_CTRL
	ldr	r2, [r1]
	orr	r2, r2, #0xa0		@ a reasonable delay for entering SR
	str	r2, [r1, #0]

	ldr	r2, ddr_start		@ do a dummy access to DDR
	ldr	r3, [r2, #0]
	ldr	r3, [r1, #0]
	orr	r3, r3, #0x200		@ now set the LP MODE to Self-Refresh
	str	r3, [r1, #0]
	str	r2, [r1, #4]		@ write to shadow register also

	mov	r1, #0x1000		@ Give some time for the system to enter SR
wait_sr:
	subs	r1, r1, #1
	bne	wait_sr

	/* Disable EMIF at this point */
	ldr	r1, virt_emif_clkctrl
	ldr	r2, [r1]
	bic	r2, r2, #(3 << 0)
	str	r2, [r1]

	ldr	r1, virt_emif_clkctrl
wait_emif_disable:
	ldr	r2, [r1]
	ldr	r3, module_disabled_val
	cmp	r2, r3
	bne	wait_emif_disable

	/* Weak pull down for DQ, DM */
	ldr	r1, virt_ddr_io_pull1
	ldr	r2, susp_io_pull
	str	r2, [r1]

	ldr	r1, virt_ddr_io_pull2
	ldr	r2, susp_io_pull
	str	r2, [r1]

	/* Disable VTP with N & P = 0x1 */
	ldr	r1, virt_ddr_vtp_ctrl
	ldr	r2, susp_vtp_ctrl_val
	str	r2, [r1]

	/* IO to work in mDDR mode */
	ldr	r0, virt_ddr_io_ctrl
	ldr	r1, [r0]
	mov	r2, #1
	mov	r3, r2, lsl #28
	str	r3, [r0]

	/* Enable SRAM LDO ret mode */
	ldr	r0, virt_sram_ldo_addr
	ldr	r1, [r0]
	orr	r1, #1
	str	r1, [r0]

	/* Put the PLLs in bypass mode */
	pll_bypass	core, virt_core_clk_mode, virt_core_idlest
	pll_bypass	ddr, virt_ddr_clk_mode, virt_ddr_idlest
	pll_bypass	disp, virt_disp_clk_mode, virt_disp_idlest
	pll_bypass	per, virt_per_clk_mode, virt_per_idlest
	pll_bypass	mpu, virt_mpu_clk_mode, virt_mpu_idlest

	dsb
	dmb
	isb

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

	/* We come here in case of an abort */

	/* Relock the PLLs */
	pll_lock	mpu_abt, virt_mpu_clk_mode, virt_mpu_idlest
	pll_lock	per_abt, virt_per_clk_mode, virt_per_idlest
	pll_lock	disp_abt, virt_disp_clk_mode, virt_disp_idlest
	pll_lock	ddr_abt, virt_ddr_clk_mode, virt_ddr_idlest
	pll_lock	core_abt, virt_core_clk_mode, virt_core_idlest

	/* Disable SRAM LDO ret mode */
	ldr	r0, virt_sram_ldo_addr
	ldr	r1, [r0]
	bic	r1, #1
	str	r1, [r0]

	/* IO to work in DDR mode */
	ldr	r0, virt_ddr_io_ctrl
	ldr	r1, [r0]
	mov	r2, #0x0
	mov	r3, r2, lsl #28
	str	r3, [r0]

	/* Restore the pull for DQ, DM */
	ldr	r1, virt_ddr_io_pull1
	ldr	r2, resume_io_pull1
	str	r2, [r1]

	ldr	r1, virt_ddr_io_pull2
	ldr	r2, resume_io_pull2
	str	r2, [r1]

	/* Enable VTP */
config_vtp_abt:
	ldr	r0, virt_ddr_vtp_ctrl
	ldr	r1, [r0]
	mov	r2, #0x0	@ clear the register
	str	r2, [r0]
	mov	r2, #0x6	@ write the filter value
	str	r2, [r0]

	ldr	r1, [r0]
	ldr	r2, vtp_enable	@ set the enable bit
	orr	r2, r2, r1
	str	r2, [r0]

	ldr	r1, [r0]	@ toggle the CLRZ bit
	bic	r1, #1
	str	r1, [r0]

	ldr	r1, [r0]
	orr	r1, #1
	str	r1, [r0]

poll_vtp_ready_abt:
	ldr	r1, [r0]	@ poll for VTP ready
	tst	r1, #(1 << 5)
	beq	poll_vtp_ready_abt

	/* Enable EMIF */
	ldr	r1, virt_emif_clkctrl
	mov	r2, #0x2
	str	r2, [r1]
wait_emif_enable:
	ldr	r3, [r1]
	cmp	r2, r3
	bne	wait_emif_enable

	/* Disable EMIF self-refresh */
	ldr	r0, emif_addr_virt
	add	r0, r0, #EMIF4_0_SDRAM_MGMT_CTRL
	ldr	r1, [r0]
	bic	r1, r1, #(0x7 << 7)
	str	r1, [r0]

	mov	r0, #7
	ldmfd	sp!, {r4 - r11, pc}	@ restore regs and return

ENTRY(am33xx_resume_offset)
	.word . - am33xx_do_wfi

ENTRY(am33xx_resume_from_deep_sleep)
	/* Take the PLLs out of LP_BYPASS */
	pll_lock	mpu, phys_mpu_clk_mode, phys_mpu_idlest
	pll_lock	per, phys_per_clk_mode, phys_per_idlest
	pll_lock	disp, phys_disp_clk_mode, phys_disp_idlest
	pll_lock	ddr, phys_ddr_clk_mode, phys_ddr_idlest
	pll_lock	core, phys_core_clk_mode, phys_core_idlest

	/* Disable SRAM LDO ret mode */
	ldr	r0, phys_sram_ldo_addr
	ldr	r1, [r0]
	bic	r1, #1
	str	r1, [r0]

	/* Restore the pull for DQ, DM */
	ldr	r1, phys_ddr_io_pull1
	ldr	r2, resume_io_pull1
	str	r2, [r1]

	ldr	r1, phys_ddr_io_pull2
	ldr	r2, resume_io_pull2
	str	r2, [r1]

	/* Disable EMIF self-refresh */
	ldr	r0, emif_phys_addr
	add	r0, r0, #EMIF4_0_SDRAM_MGMT_CTRL
	ldr	r1, [r0]
	bic	r1, r1, #(0x7 << 7)
	str	r1, [r0]

	/* Take out IO of mDDR mode */
	ldr	r0, phys_ddr_io_ctrl
	ldr	r1, [r0]
	bic	r1, r1, #28
	str	r1, [r0]

/*
 * Instead of harcoding the EMIF and DDR PHY related settings
 * in this file, the sane thing to do would have been to backup
 * the register contents during suspend and restore it back in
 * the resume path. However, due to the Si errata related to
 * DDR PHY registers, these registers are read-only. So, we'll
 * need to hardcode atleast the DDR PHY configuration over here.
 * We _could_ back up the EMIF registers but in order to be
 * consistent with the DDR setup procedure we skip this for now.
 * The person updating the DDR PHY config values is expected
 * to update the EMIF config values also.
 */

config_vtp:
	ldr	r0, vtp0_addr
	ldr	r1, [r0]
	mov	r2, #0x0	@ clear the register
	str	r2, [r0]
	mov	r2, #0x6	@ write the filter value
	str	r2, [r0]

	ldr	r1, [r0]
	ldr	r2, vtp_enable	@ set the enable bit
	orr	r2, r2, r1
	str	r2, [r0]

	ldr	r1, [r0]	@ toggle the CLRZ bit
	bic	r1, #1
	str	r1, [r0]

	ldr	r1, [r0]
	orr	r1, #1
	str	r1, [r0]

poll_vtp_ready:
	ldr	r1, [r0]	@ poll for VTP ready
	tst	r1, #(1 << 5)
	beq	poll_vtp_ready

cmd_macro_config:
	ldr	r0, ddr_phy_base
	ldr	r1, [r0]
	ldr	r2, ddr2_ratio_val
	mov	r3, r2
	@ TODO: Need to use proper variable here
	mov	r4, #0
	str	r3, [r0, #28]	@cmd0
	str	r4, [r0, #32]
	str	r4, [r0, #36]
	str	r4, [r0, #40]
	str	r4, [r0, #44]
	str	r3, [r0, #80]	@cmd1
	str	r4, [r0, #84]
	str	r4, [r0, #88]
	str	r4, [r0, #92]
	str	r4, [r0, #96]
	str	r3, [r0, #132]	@cmd2
	str	r4, [r0, #136]
	str	r4, [r0, #140]
	str	r4, [r0, #144]
	str	r4, [r0, #148]

	mov	r3, #0x0
	bl	data_macro_config
	mov	r3, #0xa4
	bl	data_macro_config
	b	setup_rank_delays

data_macro_config:
	ldr	r0, ddr_phy_base
	add	r0, r0, r3
rd_dqs:
	ldr	r1, data0_rd_dqs_slave_ratio0_val
	mov	r2, r1
	/* shift by 30, 20, 10 and orr */
	mov	r5, r2, lsl #10
	mov	r6, r2, lsl #20
	mov	r7, r2, lsl #30
	orr	r2, r2, r5
	orr	r2, r2, r6
	orr	r2, r2, r7
	/* Done with crazy bit ops. store it now */
	str	r2, [r0, #200]
	ldr	r1, data0_rd_dqs_slave_ratio1_val
	mov	r2, r1
	mov	r5, r2, lsr #2
	mov	r2, r5
	str	r2, [r0, #204]
wr_dqs:
	ldr	r1, data0_wr_dqs_slave_ratio0_val
	mov	r2, r1
	/* shift by 30, 20, 10 and orr */
	mov	r5, r2, lsl #10
	mov	r6, r2, lsl #20
	mov	r7, r2, lsl #30
	orr	r2, r2, r5
	orr	r2, r2, r6
	orr	r2, r2, r7
	/* Done with crazy bit ops. store it now */
	str	r2, [r0, #220]
	ldr	r1, data0_wr_dqs_slave_ratio1_val
	mov	r2, r1
	mov	r5, r2, lsr #2
	mov	r2, r5
	str	r2, [r0, #224]
wr_lvl:
	ldr	r1, data0_wr_lvl_init_ratio0_val
	mov	r2, r1
	/* shift by 30, 20, 10 and orr */
	mov	r5, r2, lsl #10
	mov	r6, r2, lsl #20
	mov	r7, r2, lsl #30
	orr	r2, r2, r5
	orr	r2, r2, r6
	orr	r2, r2, r7
	/* Done with crazy bit ops. store it now */
	str	r2, [r0, #240]
	ldr	r1, data0_wr_lvl_init_ratio1_val
	mov	r2, r1
	mov	r5, r2, lsr #2
	mov	r2, r5
	str	r2, [r0, #244]
gate_lvl:
	ldr	r1, data0_gate_lvl_init_ratio0_val
	mov	r2, r1
	/* shift by 30, 20, 10 and orr */
	mov	r5, r2, lsl #10
	mov	r6, r2, lsl #20
	mov	r7, r2, lsl #30
	orr	r2, r2, r5
	orr	r2, r2, r6
	orr	r2, r2, r7
	/* Done with crazy bit ops. store it now */
	str	r2, [r0, #248]
	ldr	r1, data0_gate_lvl_init_ratio1_val
	mov	r2, r1
	mov	r5, r2, lsr #2
	mov	r2, r5
	str	r2, [r0, #256]
we_slv:
	ldr	r1, data0_wr_lvl_slave_ratio0_val
	mov	r2, r1
	/* shift by 30, 20, 10 and orr */
	mov	r5, r2, lsl #10
	mov	r6, r2, lsl #20
	mov	r7, r2, lsl #30
	orr	r2, r2, r5
	orr	r2, r2, r6
	orr	r2, r2, r7
	/* Done with crazy bit ops. store it now */
	str	r2, [r0, #264]
	ldr	r1, data0_wr_lvl_slave_ratio1_val
	mov	r2, r1
	mov	r5, r2, lsr #2
	mov	r2, r5
	str	r2, [r0, #268]
wr_data:
	ldr	r1, data0_wr_data_slave_ratio0_val
	mov	r2, r1
	/* shift by 30, 20, 10 and orr */
	mov	r5, r2, lsl #10
	mov	r6, r2, lsl #20
	mov	r7, r2, lsl #30
	orr	r2, r2, r5
	orr	r2, r2, r6
	orr	r2, r2, r7
	/* Done with crazy bit ops. store it now */
	str	r2, [r0, #288]
	ldr	r1, data0_wr_data_slave_ratio1_val
	mov	r2, r1
	mov	r5, r2, lsr #2
	mov	r2, r5
	str	r2, [r0, #292]
dll_lock:
	ldr	r1, data0_dll_lock_diff_val
	mov	r2, r1
	str	r2, [r0, #312]

setup_rank_delays:
	ldr	r1, data0_rank0_delay0_val
	mov	r2, r1
	str	r2, [r0, #308]
	ldr	r1, data1_rank0_delay1_val
	mov	r2, r1
	str	r2, [r0, #472]

setup_io_ctrl:
	ldr	r0, control_base
	ldr	r1, ddr_ioctrl_val
	mov	r2, r1
	ldr	r4, ddr_cmd_offset
	mov	r3, r4
	str	r2, [r0, r3]	@cmd0 0x1404
	add	r3, r3, #4
	str	r2, [r0, r3]	@cmd1 0x1408
	add	r3, r3, #4
	str	r2, [r0, r3]	@cmd2 0x140c
	ldr	r4, ddr_data_offset
	mov	r3, r4
	str	r2, [r0, r3]	@data0 0x1440
	add	r3, r3, #4
	str	r2, [r0, r3]	@data1 0x1444

misc_config:
	ldr	r1, ddr_io_ctrl_addr
	ldr	r2, [r1]
	and	r2, #0xefffffff
	str	r2, [r1]
	ldr	r1, ddr_cke_addr
	ldr	r2, [r1]
	orr	r2, #0x00000001
	str	r2, [r1]

config_emif_timings:
	mov	r3, #1275068416	@ 0x4c000000
disable_sr:
	mov	r4, #0
	str	r4, [r3, #56]	@ 0x38
	ldr	r4, emif_rd_lat_val
	mov	r2, r4
rd_lat:
	str	r2, [r3, #228]	@ 0xe4
	str	r2, [r3, #232]	@ 0xe8
	str	r2, [r3, #236]	@ 0xec
timing1:
	ldr	r4, emif_timing1_val
	mov	r2, r4
	str	r2, [r3, #24]
	str	r2, [r3, #28]
timing2:
	ldr	r4, emif_timing2_val
	mov	r2, r4
	str	r2, [r3, #32]
	str	r2, [r3, #36]	@ 0x24
timing3:
	ldr	r4, emif_timing3_val
	mov	r2, r4
	str	r2, [r3, #40]	@ 0x28
	str	r2, [r3, #44]	@ 0x2c
sdcfg1:
	ldr	r4, emif_sdcfg_val
	mov	r2, r4
	str	r2, [r3, #8]
	str	r2, [r3, #12]
ref_ctrl_const:
	ldr	r4, emif_ref_ctrl_const_val
	mov	r2, r4
	str	r2, [r3, #16]
	str	r2, [r3, #20]

	/* GEL had a loop with init value of 5000 */
	mov	r0, #0x1000
wait_loop1:
	subs	r0, r0, #1
	bne	wait_loop1

ref_ctrl_actual:
	ldr	r4, emif_ref_ctrl_val
	mov	r2, r4
	str	r2, [r3, #16]
	str	r2, [r3, #20]
sdcfg2:
	ldr	r4, emif_sdcfg_val
	mov	r2, r4
	str	r2, [r3, #8]
	str	r2, [r3, #12]

	/* Back from la-la-land. Kill some time for sanity to settle in */
	mov	r0, #0x1000
wait_loop2:
	subs	r0, r0, #1
	bne	wait_loop2

	/* We are back. Branch to the common CPU resume routine */
ENTRY(am33xx_resume_vector)
	ldr	pc, resume_addr

/*
 * Local variables
 */

resume_addr:
	.word	cpu_resume - PAGE_OFFSET + 0x80000000

emif_addr_func:
	.word	am33xx_get_ram_base
emif_phys_addr:
	.word	AM33XX_EMIF0_BASE

emif_pm_ctrl:
	.word EMIF4_0_SDRAM_MGMT_CTRL
ddr_start:
	.word PAGE_OFFSET

virt_mpu_idlest:
	.word	AM33XX_CM_IDLEST_DPLL_MPU
virt_mpu_clk_mode:
	.word	AM33XX_CM_CLKMODE_DPLL_MPU

phys_pll_mod:
	.word	AM33XX_CM_BASE + AM33XX_CM_WKUP_MOD
phys_mpu_clk_mode:
	.word	AM33XX_CM_BASE + AM33XX_CM_WKUP_MOD + AM33XX_CM_CLKMODE_DPLL_MPU_OFFSET
phys_mpu_idlest:
	.word	AM33XX_CM_BASE + AM33XX_CM_WKUP_MOD + AM33XX_CM_IDLEST_DPLL_MPU_OFFSET

virt_core_idlest:
	.word	AM33XX_CM_IDLEST_DPLL_CORE
virt_core_clk_mode:
	.word	AM33XX_CM_CLKMODE_DPLL_CORE
phys_core_clk_mode:
	.word	AM33XX_CM_BASE + AM33XX_CM_WKUP_MOD + AM33XX_CM_CLKMODE_DPLL_CORE_OFFSET
phys_core_idlest:
	.word	AM33XX_CM_BASE + AM33XX_CM_WKUP_MOD + AM33XX_CM_IDLEST_DPLL_CORE_OFFSET

virt_per_idlest:
	.word	AM33XX_CM_IDLEST_DPLL_PER
virt_per_clk_mode:
	.word	AM33XX_CM_CLKMODE_DPLL_PER
phys_per_clk_mode:
	.word	AM33XX_CM_BASE + AM33XX_CM_WKUP_MOD + AM33XX_CM_CLKMODE_DPLL_PER_OFFSET
phys_per_idlest:
	.word	AM33XX_CM_BASE + AM33XX_CM_WKUP_MOD + AM33XX_CM_IDLEST_DPLL_PER_OFFSET

virt_disp_idlest:
	.word	AM33XX_CM_IDLEST_DPLL_DISP
virt_disp_clk_mode:
	.word	AM33XX_CM_CLKMODE_DPLL_DISP
phys_disp_clk_mode:
	.word	AM33XX_CM_BASE + AM33XX_CM_WKUP_MOD + AM33XX_CM_CLKMODE_DPLL_DISP_OFFSET
phys_disp_idlest:
	.word	AM33XX_CM_BASE + AM33XX_CM_WKUP_MOD + AM33XX_CM_IDLEST_DPLL_DISP_OFFSET

virt_ddr_idlest:
	.word	AM33XX_CM_IDLEST_DPLL_DDR
virt_ddr_clk_mode:
	.word	AM33XX_CM_CLKMODE_DPLL_DDR
phys_ddr_clk_mode:
	.word	AM33XX_CM_BASE + AM33XX_CM_WKUP_MOD + AM33XX_CM_CLKMODE_DPLL_DDR_OFFSET
phys_ddr_idlest:
	.word	AM33XX_CM_BASE + AM33XX_CM_WKUP_MOD + AM33XX_CM_IDLEST_DPLL_DDR_OFFSET

virt_sram_ldo_addr:
	.word	AM33XX_PRM_LDO_SRAM_MPU_CTRL
phys_sram_ldo_addr:
	.word	AM33XX_PRM_BASE + AM33XX_PRM_DEVICE_MOD + AM33XX_PRM_LDO_SRAM_MPU_CTRL_OFFSET

virt_emif_clkctrl:
	.word	AM33XX_CM_PER_EMIF_CLKCTRL
phys_emif_clkctrl:
	.word	AM33XX_CM_BASE + AM33XX_CM_PER_MOD + AM33XX_CM_PER_EMIF_CLKCTRL_OFFSET
module_disabled_val:
	.word	0x30000

/* DDR related stuff */
vtp0_addr:
	.word	VTP0_CTRL_REG
vtp_enable:
	.word	VTP_CTRL_ENABLE
vtp_start_en:
	.word	VTP_CTRL_START_EN
vtp_ready:
	.word	VTP_CTRL_READY

ddr_phy_base:
	.word	DDR_PHY_BASE_ADDR
ddr2_ratio_val:
	.word	DDR2_RATIO
data0_rd_dqs_slave_ratio0_val:
	.word	DDR2_RD_DQS
data0_rd_dqs_slave_ratio1_val:
	.word	DDR2_RD_DQS
data0_wr_dqs_slave_ratio0_val:
	.word	DDR2_WR_DQS
data0_wr_dqs_slave_ratio1_val:
	.word	DDR2_WR_DQS
data0_wr_lvl_init_ratio0_val:
	.word	DDR2_PHY_WRLVL
data0_wr_lvl_init_ratio1_val:
	.word	DDR2_PHY_WRLVL
data0_gate_lvl_init_ratio0_val:
	.word	DDR2_PHY_GATELVL
data0_gate_lvl_init_ratio1_val:
	.word	DDR2_PHY_GATELVL
data0_wr_lvl_slave_ratio0_val:
	.word	DDR2_PHY_FIFO_WE
data0_wr_lvl_slave_ratio1_val:
	.word	DDR2_PHY_FIFO_WE
data0_wr_data_slave_ratio0_val:
	.word	DDR2_PHY_WR_DATA
data0_wr_data_slave_ratio1_val:
	.word	DDR2_PHY_WR_DATA
data0_dll_lock_diff_val:
	.word	PHY_DLL_LOCK_DIFF

data0_rank0_delay0_val:
	.word	PHY_RANK0_DELAY
data1_rank0_delay1_val:
	.word	PHY_RANK0_DELAY

control_base:
	.word	AM33XX_CTRL_BASE
ddr_io_ctrl_addr:
	.word	DDR_IO_CTRL
ddr_ioctrl_val:
	.word	0x18B
ddr_cmd_offset:
	.word	0x1404
ddr_data_offset:
	.word	0x1440
virt_ddr_io_ctrl:
	.word	AM33XX_CTRL_REGADDR(0x0E04)
phys_ddr_io_ctrl:
	.word	DDR_IO_CTRL
virt_ddr_vtp_ctrl:
	.word	AM33XX_CTRL_REGADDR(0x0E0C)
phys_ddr_vtp_ctrl:
	.word	VTP0_CTRL_REG
virt_ddr_io_pull1:
	.word	AM33XX_CTRL_REGADDR(0x1440)
phys_ddr_io_pull1:
	.word	AM33XX_CTRL_BASE + (0x1440)
virt_ddr_io_pull2:
	.word	AM33XX_CTRL_REGADDR(0x1444)
phys_ddr_io_pull2:
	.word	AM33XX_CTRL_BASE + (0x1444)
virt_ddr_io_pull3:
	.word	AM33XX_CTRL_REGADDR(0x1448)
phys_ddr_io_pull3:
	.word	AM33XX_CTRL_BASE + (0x1448)

ddr_cke_addr:
	.word	DDR_CKE_CTRL
emif_rd_lat_val:
	.word	EMIF_READ_LATENCY
emif_timing1_val:
	.word	EMIF_TIM1
emif_timing2_val:
	.word	EMIF_TIM2
emif_timing3_val:
	.word	EMIF_TIM3
emif_sdcfg_val:
	.word	EMIF_SDCFG
emif_ref_ctrl_const_val:
	.word	0x4650
emif_ref_ctrl_val:
	.word	EMIF_SDREF

susp_io_pull:
	.word	0x3FF00003
resume_io_pull1:
	.word	0x18B
resume_io_pull2:
	.word	0x18B
dyn_pd_val:
	.word	0x100000
susp_sdram_config:
	.word	0x40805332
susp_vtp_ctrl_val:
	.word	0x10117
emif_addr_virt:
	.word	0xDEADBEEF


ENTRY(am33xx_do_wfi_sz)
	.word	. - am33xx_do_wfi
