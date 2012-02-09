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

	.align 3
ENTRY(am33xx_do_wfi)
	stmfd	sp!, {r4 - r11, lr}	@ save registers on stack

	/* Put DDR in Self-Refresh */
	ldr	r0, emif_addr_func
	blx	r0
	add	r0, r0, #EMIF4_0_SDRAM_MGMT_CTRL
	ldr	r3, [r0]
	orr	r3, r3, #0xa0		@ a reasonable delay for entering SR
	str	r3, [r0, #0]

	ldr	r2, ddr_start
	ldr	r1, [r2, #0]
	ldr	r1, [r0, #0]
	orr	r1, r1, #0x200
	str	r1, [r0, #0]

	mov	r0, #0x1000
wait_sr:
	subs	r0, r0, #1
	bne	wait_sr

	/* Put the PLLs to LP bypass */
#if 0
	ldr	r0, virt_mpu_pll_clk_mode_addr
	ldr	r1, [r0]
	bic	r1, r1, #(7 << 0)
	orr	r1, r1, #0x5
	str	r1, [r0]
	ldr	r0, virt_mpu_pll_idlest_addr
wait_pll_by1:
	ldr	r1, [r0]
	tst	r1, #0x0
	bne	wait_pll_by1

	ldr	r0, virt_per_pll_clk_mode_addr
	ldr	r1, [r0]
	bic	r1, r1, #(7 << 0)
	orr	r1, r1, #0x5
	str	r1, [r0]
	ldr	r0, virt_per_pll_idlest_addr
wait_pll_by2:
	ldr	r1, [r0]
	tst	r1, #0x0
	bne	wait_pll_by2

	ldr	r0, virt_disp_pll_clk_mode_addr
	ldr	r1, [r0]
	bic	r1, r1, #(7 << 0)
	orr	r1, r1, #0x5
	str	r1, [r0]
	ldr	r0, virt_disp_pll_idlest_addr
wait_pll_by3:
	ldr	r1, [r0]
	tst	r1, #0x0
	bne	wait_pll_by3

	ldr	r0, virt_ddr_pll_clk_mode_addr
	ldr	r1, [r0]
	bic	r1, r1, #(7 << 0)
	orr	r1, r1, #0x5
	str	r1, [r0]
	ldr	r0, virt_ddr_pll_idlest_addr
wait_pll_by4:
	ldr	r1, [r0]
	tst	r1, #0x0
	bne	wait_pll_by4

	ldr	r0, virt_core_pll_clk_mode_addr
	ldr	r1, [r0]
	bic	r1, r1, #(7 << 0)
	orr	r1, r1, #0x5
	str	r1, [r0]
	ldr	r0, virt_core_pll_idlest_addr
wait_pll_by5:
	ldr	r1, [r0]
	tst	r1, #0x0
	bne	wait_pll_by5
#endif

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
	ldmfd	sp!, {r4 - r11, pc}	@ restore regs and return


	/* Take the PLLs out of LP_BYPASS */
#if 0
	ldr	r1, phys_mpu_pll_clk_mode_addr
	ldr	r2, [r1]
	bic	r2, r2, #(7 << 0)
	orr	r2, r2, #0x7
	str	r2, [r1]
	ldr	r1, phys_mpu_pll_idlest_addr
wait_pll_lock1:
	ldr	r2, [r1]
	tst	r2, #0x1
	bne	wait_pll_lock1

	ldr	r1, phys_core_pll_clk_mode_addr
	ldr	r2, [r1]
	bic	r2, r2, #(7 << 0)
	orr	r2, r2, #0x7
	str	r2, [r1]
	ldr	r1, phys_core_pll_idlest_addr
wait_pll_lock2:
	ldr	r2, [r1]
	tst	r2, #0x1
	bne	wait_pll_lock2

	ldr	r1, phys_per_pll_clk_mode_addr
	ldr	r2, [r1]
	bic	r2, r2, #(7 << 0)
	orr	r2, r2, #0x7
	str	r2, [r1]
	ldr	r1, phys_per_pll_idlest_addr
wait_pll_lock3:
	ldr	r2, [r1]
	tst	r2, #0x1
	bne	wait_pll_lock3

	ldr	r1, phys_disp_pll_clk_mode_addr
	ldr	r2, [r1]
	bic	r2, r2, #(7 << 0)
	orr	r2, r2, #0x7
	str	r2, [r1]
	ldr	r1, phys_disp_pll_idlest_addr
wait_pll_lock4:
	ldr	r2, [r1]
	tst	r2, #0x1
	bne	wait_pll_lock4

	ldr	r1, phys_ddr_pll_clk_mode_addr
	ldr	r2, [r1]
	bic	r2, r2, #(7 << 0)
	orr	r2, r2, #0x7
	str	r2, [r1]
	ldr	r1, phys_ddr_pll_idlest_addr
wait_pll_lock5:
	ldr	r2, [r1]
	tst	r2, #0x1
	bne	wait_pll_lock5

	ldr	r0, emif_phys_addr
	add	r0, r0, #EMIF4_0_SDRAM_MGMT_CTRL
	ldr	r1, [r0]
	bic	r1, r1, #(0x7 << 7)
	str	r1, [r0]
#endif

	nop
	nop
	nop
	nop
	nop
	nop
	nop
	/* Assume the PLL is locked at this point */
restore_emif:
	nop
	nop

config_vtp:
	ldr	r0, vtp0_addr
	ldr	r1, [r0]
	ldr	r2, vtp_enable
	orr	r1, r2
	str	r1, [r0]

	ldr	r1, [r0]
	bic	r1, #1
	str	r1, [r0]

	ldr	r1, [r0]
	orr	r1, #1
	str	r1, [r0]

poll_vtp_ready:
	ldr	r1, [r0]
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

#define DPLL_LP_BYP_MODE	0x5
#define DPLL_LOCK_MODE		0x7

emif_addr_func:
	.word	am33xx_get_ram_base
emif_phys_addr:
	.word	AM33XX_EMIF0_BASE

emif_pm_ctrl:
	.word EMIF4_0_SDRAM_MGMT_CTRL
ddr_start:
	.word PAGE_OFFSET

mpu_pll_n:
	.word	0xDEADBEEF
mpu_pll_m:
	.word	0xDEADBEEF
mpu_pll_m2:
	.word	0xDEADBEEF
ddr_pll_m2:
	.word	0xDEADBEEF
ddr_pll_n:
	.word	0xDEADBEEF
ddr_pll_m:
	.word	0xDEADBEEF
per_pll_n:
	.word	0xDEADBEEF
per_pll_m:
	.word	0xDEADBEEF
per_pll_m2:
	.word	0xDEADBEEF

virt_mpu_pll_idlest_addr:
	.word	AM33XX_CM_IDLEST_DPLL_MPU
virt_mpu_pll_clk_mode_addr:
	.word	AM33XX_CM_CLKMODE_DPLL_MPU

phys_pll_mod:
	.word	AM33XX_CM_BASE + AM33XX_CM_WKUP_MOD
phys_mpu_pll_idlest_addr:
	.word	AM33XX_CM_CLKMODE_DPLL_MPU_OFFSET
phys_mpu_pll_clk_mode_addr:
	.word	AM33XX_CM_IDLEST_DPLL_MPU_OFFSET

virt_core_pll_idlest_addr:
	.word	AM33XX_CM_IDLEST_DPLL_CORE
virt_core_pll_clk_mode_addr:
	.word	AM33XX_CM_CLKMODE_DPLL_CORE
phys_core_pll_idlest_addr:
	.word	AM33XX_CM_CLKMODE_DPLL_CORE_OFFSET
phys_core_pll_clk_mode_addr:
	.word	AM33XX_CM_IDLEST_DPLL_CORE_OFFSET

virt_per_pll_idlest_addr:
	.word	AM33XX_CM_IDLEST_DPLL_PER
virt_per_pll_clk_mode_addr:
	.word	AM33XX_CM_CLKMODE_DPLL_PER
phys_per_pll_idlest_addr:
	.word	AM33XX_CM_CLKMODE_DPLL_PER_OFFSET
phys_per_pll_clk_mode_addr:
	.word	AM33XX_CM_IDLEST_DPLL_PER_OFFSET

virt_disp_pll_idlest_addr:
	.word	AM33XX_CM_IDLEST_DPLL_DISP
virt_disp_pll_clk_mode_addr:
	.word	AM33XX_CM_CLKMODE_DPLL_DISP
phys_disp_pll_idlest_addr:
	.word	AM33XX_CM_CLKMODE_DPLL_DISP_OFFSET
phys_disp_pll_clk_mode_addr:
	.word	AM33XX_CM_IDLEST_DPLL_DISP_OFFSET

virt_ddr_pll_idlest_addr:
	.word	AM33XX_CM_IDLEST_DPLL_DDR
virt_ddr_pll_clk_mode_addr:
	.word	AM33XX_CM_CLKMODE_DPLL_DDR
phys_ddr_pll_idlest_addr:
	.word	AM33XX_CM_CLKMODE_DPLL_DDR_OFFSET
phys_ddr_pll_clk_mode_addr:
	.word	AM33XX_CM_IDLEST_DPLL_DDR_OFFSET


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
Rddr_ioctrl_val:
	.word	DDR_IOCTRL_VALUE
ddr_io_ctrl_addr:
	.word	DDR_IO_CTRL
ddr_ioctrl_val:
	.word	0x18B
ddr_cmd_offset:
	.word	0x1404
ddr_data_offset:
	.word	0x1440

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

ENTRY(am33xx_do_wfi_sz)
	.word	. - am33xx_do_wfi
