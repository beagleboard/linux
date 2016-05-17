/*
 * Low level PM code for TI EMIF
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
 *	Dave Gerlach
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

#include "emif.h"

#define EMIF_POWER_MGMT_WAIT_SELF_REFRESH_8192_CYCLES	0x00a0
#define EMIF_POWER_MGMT_SR_TIMER_MASK			0x00f0
#define EMIF_POWER_MGMT_SELF_REFRESH_MODE		0x0200
#define EMIF_POWER_MGMT_SELF_REFRESH_MODE_MASK		0x0700

#define EMIF_SDCFG_TYPE_DDR2				0x2 << SDRAM_TYPE_SHIFT
#define EMIF_STATUS_READY				0x4

#define AM43XX_EMIF_PHY_CTRL_REG_COUNT                  0x120

#define EMIF_AM437X_REGISTERS				0x1

	.text
	.align 3
	.arm

ENTRY(ti_emif_sram)

/*
 * void ti_emif_save_context(void)
 *
 * Used during suspend to save the context of all required EMIF registers
 * to local memory if the EMIF is going to lose context during the sleep
 * transition. Operates on the VIRTUAL address of the EMIF.
 */
ENTRY(ti_emif_save_context)
	stmfd   sp!, {r4 - r11, lr}     @ save registers on stack

	ldr	r0, ti_emif_base_addr_virt

	/* Save EMIF configuration */
	ldr	r1, [r0, #EMIF_SDRAM_CONFIG]
	str	r1, emif_sdcfg_val
	ldr	r1, [r0, #EMIF_SDRAM_REFRESH_CONTROL]
	str	r1, emif_ref_ctrl_val
	ldr	r1, [r0, #EMIF_SDRAM_TIMING_1]
	str	r1, emif_timing1_val
	ldr	r1, [r0, #EMIF_SDRAM_TIMING_2]
	str	r1, emif_timing2_val
	ldr	r1, [r0, #EMIF_SDRAM_TIMING_3]
	str	r1, emif_timing3_val
	ldr	r1, [r0, #EMIF_POWER_MANAGEMENT_CONTROL]
	str	r1, emif_pmcr_val
	ldr	r1, [r0, #EMIF_POWER_MANAGEMENT_CTRL_SHDW]
	str	r1, emif_pmcr_shdw_val
	ldr	r1, [r0, #EMIF_SDRAM_OUTPUT_IMPEDANCE_CALIBRATION_CONFIG]
	str	r1, emif_zqcfg_val
	ldr	r1, [r0, #EMIF_DDR_PHY_CTRL_1]
	str	r1, emif_ddr_phy_ctlr_1
	ldr	r1, [r0, #EMIF_COS_CONFIG]
	str	r1, emif_cos_config
	ldr	r1, [r0, #EMIF_PRIORITY_TO_CLASS_OF_SERVICE_MAPPING]
	str	r1, emif_priority_to_cos_mapping
	ldr	r1, [r0, #EMIF_CONNECTION_ID_TO_CLASS_OF_SERVICE_1_MAPPING]
	str	r1, emif_connect_id_serv_1_map
	ldr	r1, [r0, #EMIF_CONNECTION_ID_TO_CLASS_OF_SERVICE_2_MAPPING]
	str	r1, emif_connect_id_serv_2_map
	ldr	r1, [r0, #EMIF_OCP_CONFIG]
	str	r1, emif_ocp_config_val

	ldr	r2, ti_emif_sram_config
	cmp	r2, #EMIF_SRAM_AM43_REG_LAYOUT
	bne	emif_skip_save_extra_regs

	ldr	r1, [r0, #EMIF_READ_WRITE_LEVELING_RAMP_CONTROL]
	str	r1, emif_rd_wr_level_ramp_ctrl
	ldr	r1, [r0, #EMIF_READ_WRITE_EXECUTION_THRESHOLD]
	str	r1, emif_rd_wr_exec_thresh
	ldr	r1, [r0, #EMIF_LPDDR2_NVM_TIMING]
	str	r1, emif_lpddr2_nvm_tim
	ldr	r1, [r0, #EMIF_LPDDR2_NVM_TIMING_SHDW]
	str	r1, emif_lpddr2_nvm_tim_shdw
	ldr	r1, [r0, #EMIF_DLL_CALIB_CTRL]
	str	r1, emif_dll_calib_ctrl_val
	ldr	r1, [r0, #EMIF_DLL_CALIB_CTRL_SHDW]
	str	r1, emif_dll_calib_ctrl_val_shdw

	/* Loop and save entire block of emif phy regs */
	mov	r2, #0x0
	adrl	r4, emif_ext_phy_ctrl_vals
	add	r3, r0, #EMIF_EXT_PHY_CTRL_1
ddr_phy_ctrl_save:
	ldr	r1, [r3, r2]
	str	r1, [r4, r2]
	add	r2,r2,#0x4
	cmp	r2, #AM43XX_EMIF_PHY_CTRL_REG_COUNT
	bne	ddr_phy_ctrl_save

emif_skip_save_extra_regs:
	ldmfd	sp!, {r4 - r11, pc}	@ restore regs and return
ENDPROC(ti_emif_save_context)

/*
 * void ti_emif_restore_context(void)
 *
 * Used during resume to restore the context of all required EMIF registers
 * from local memory after the EMIF has lost context during a sleep transition.
 * Operates on the PHYSICAL address of the EMIF.
 */
ENTRY(ti_emif_restore_context)
	stmfd   sp!, {r4 - r11, lr}     @ save registers on stack

	ldr     r0, ti_emif_base_addr_phys

	/* Config EMIF Timings */
	ldr	r1, emif_ddr_phy_ctlr_1
	str	r1, [r0, #EMIF_DDR_PHY_CTRL_1]
	str	r1, [r0, #EMIF_DDR_PHY_CTRL_1_SHDW]
	ldr	r1, emif_timing1_val
	str	r1, [r0, #EMIF_SDRAM_TIMING_1]
	str	r1, [r0, #EMIF_SDRAM_TIMING_1_SHDW]
	ldr	r1, emif_timing2_val
	str	r1, [r0, #EMIF_SDRAM_TIMING_2]
	str	r1, [r0, #EMIF_SDRAM_TIMING_2_SHDW]
	ldr	r1, emif_timing3_val
	str	r1, [r0, #EMIF_SDRAM_TIMING_3]
	str	r1, [r0, #EMIF_SDRAM_TIMING_3_SHDW]
	ldr	r1, emif_ref_ctrl_val
	str	r1, [r0, #EMIF_SDRAM_REFRESH_CONTROL]
	str	r1, [r0, #EMIF_SDRAM_REFRESH_CTRL_SHDW]
	ldr	r1, emif_pmcr_val
	str	r1, [r0, #EMIF_POWER_MANAGEMENT_CTRL_SHDW]
	ldr	r1, emif_pmcr_shdw_val
	str	r1, [r0, #EMIF_POWER_MANAGEMENT_CTRL_SHDW]
	ldr	r1, emif_cos_config
	str	r1, [r0, #EMIF_COS_CONFIG]
	ldr	r1, emif_priority_to_cos_mapping
	str	r1, [r0, #EMIF_PRIORITY_TO_CLASS_OF_SERVICE_MAPPING]
	ldr	r1, emif_connect_id_serv_1_map
	str	r1, [r0, #EMIF_CONNECTION_ID_TO_CLASS_OF_SERVICE_1_MAPPING]
	ldr	r1, emif_connect_id_serv_2_map
	str	r1, [r0, #EMIF_CONNECTION_ID_TO_CLASS_OF_SERVICE_2_MAPPING]
	ldr	r1, emif_ocp_config_val
	str	r1, [r0, #EMIF_OCP_CONFIG]

	ldr	r2, ti_emif_sram_config
	cmp	r2, #EMIF_SRAM_AM43_REG_LAYOUT
	bne	emif_skip_restore_extra_regs

	ldr     r1, emif_rd_wr_level_ramp_ctrl
	str	r1, [r0, #EMIF_READ_WRITE_LEVELING_RAMP_CONTROL]
	ldr	r1, emif_rd_wr_exec_thresh
	str	r1, [r0, #EMIF_READ_WRITE_EXECUTION_THRESHOLD]
	ldr	r1, emif_lpddr2_nvm_tim
	str	r1, [r0, #EMIF_LPDDR2_NVM_TIMING]
	ldr	r1, emif_lpddr2_nvm_tim_shdw
	str	r1, [r0, #EMIF_LPDDR2_NVM_TIMING_SHDW]
	ldr	r1, emif_dll_calib_ctrl_val
	str	r1, [r0, #EMIF_DLL_CALIB_CTRL]
	ldr	r1, emif_dll_calib_ctrl_val_shdw
	str	r1, [r0, #EMIF_DLL_CALIB_CTRL_SHDW]
	ldr	r1, emif_zqcfg_val
	str	r1, [r0, #EMIF_SDRAM_OUTPUT_IMPEDANCE_CALIBRATION_CONFIG]

	/* Loop and restore entire block of emif phy regs */
	mov	r2, #0x0
	adrl	r3, emif_ext_phy_ctrl_vals
	add	r4, r0, #EMIF_EXT_PHY_CTRL_1
ddr_phy_ctrl_restore:
	ldr	r1, [r3, r2]
	str	r1, [r4, r2]
	add	r2, r2, #0x4
	cmp	r2, #AM43XX_EMIF_PHY_CTRL_REG_COUNT
	bne	ddr_phy_ctrl_restore

emif_skip_restore_extra_regs:
	/*
	 * Output impedence calib needed only for DDR3
	 * but since the initial state of this will be
	 * disabled for DDR2 no harm in restoring the
	 * old configuration
	 */
	ldr	r1, emif_zqcfg_val
	str	r1, [r0, #EMIF_SDRAM_OUTPUT_IMPEDANCE_CALIBRATION_CONFIG]

	/* Write to sdcfg last for DDR2 only */
	ldr	r1, emif_sdcfg_val
	and	r2, r1, #SDRAM_TYPE_MASK
	cmp	r2, #EMIF_SDCFG_TYPE_DDR2
	streq	r1, [r0, #EMIF_SDRAM_CONFIG]

	ldmfd	sp!, {r4 - r11, pc}	@ restore regs and return
ENDPROC(ti_emif_restore_context)

/*
 * void ti_emif_enter_sr(void)
 *
 * Programs the EMIF to tell the SDRAM to enter into self-refresh
 * mode during a sleep transition. Operates on the VIRTUAL address
 * of the EMIF.
 */
ENTRY(ti_emif_enter_sr)
	stmfd   sp!, {r4 - r11, lr}     @ save registers on stack

	ldr	r0, ti_emif_base_addr_virt

	ldr	r1, [r0, #EMIF_POWER_MANAGEMENT_CONTROL]
	bic	r1, r1, #EMIF_POWER_MGMT_SELF_REFRESH_MODE_MASK
	orr	r1, r1, #EMIF_POWER_MGMT_SELF_REFRESH_MODE
	str	r1, [r0, #EMIF_POWER_MANAGEMENT_CONTROL]

	ldmfd	sp!, {r4 - r11, pc}	@ restore regs and return
ENDPROC(ti_emif_enter_sr)

/*
 * void ti_emif_exit_sr(void)
 *
 * Programs the EMIF to tell the SDRAM to exit self-refresh mode
 * after a sleep transition. Operates on the PHYSICAL address of
 * the EMIF.
 */
ENTRY(ti_emif_exit_sr)
	stmfd   sp!, {r4 - r11, lr}     @ save registers on stack

	ldr	r0, ti_emif_base_addr_phys

	/*
	 * Toggle EMIF to exit refresh mode:
	 * if EMIF lost context, PWR_MGT_CTRL is currently 0, writing disable
	 *   (0x0), wont do diddly squat! so do a toggle from SR(0x2) to disable
	 *   (0x0) here.
	 * *If* EMIF did not lose context, nothing broken as we write the same
	 *   value(0x2) to reg before we write a disable (0x0).
	 */
	ldr	r1, emif_pmcr_val
	bic	r1, r1, #EMIF_POWER_MGMT_SELF_REFRESH_MODE_MASK
	orr	r1, r1, #EMIF_POWER_MGMT_SELF_REFRESH_MODE
	str	r1, [r0, #EMIF_POWER_MANAGEMENT_CONTROL]
	bic	r1, r1, #EMIF_POWER_MGMT_SELF_REFRESH_MODE_MASK
	str	r1, [r0, #EMIF_POWER_MANAGEMENT_CONTROL]

        /* Wait for EMIF to become ready */
1:	ldr     r1, [r0, #EMIF_STATUS]
	tst     r1, #EMIF_STATUS_READY
	beq     1b

	ldmfd	sp!, {r4 - r11, pc}	@ restore regs and return
ENDPROC(ti_emif_exit_sr)

/*
 * void ti_emif_abort_sr(void)
 *
 * Disables self-refresh after a failed transition to a low-power
 * state so the kernel can jump back to DDR and follow abort path.
 * Operates on the VIRTUAL address of the EMIF.
 */
ENTRY(ti_emif_abort_sr)
	stmfd   sp!, {r4 - r11, lr}     @ save registers on stack

	ldr	r0, ti_emif_base_addr_virt

	ldr	r1, emif_pmcr_val
	bic	r1, r1, #EMIF_POWER_MGMT_SELF_REFRESH_MODE_MASK
	str	r1, [r0, #EMIF_POWER_MANAGEMENT_CONTROL]

	/* Wait for EMIF to become ready */
1:	ldr     r1, [r0, #EMIF_STATUS]
	tst     r1, #EMIF_STATUS_READY
	beq     1b

	ldmfd	sp!, {r4 - r11, pc}	@ restore regs and return
ENDPROC(ti_emif_abort_sr)

	.align 3
/* DDR related defines */
emif_rd_lat_val:
	.word	0xDEADBEEF
emif_timing1_val:
	.word	0xDEADBEEF
emif_timing2_val:
	.word	0xDEADBEEF
emif_timing3_val:
	.word	0xDEADBEEF
emif_sdcfg_val:
	.word	0xDEADBEEF
emif_ref_ctrl_val:
	.word	0xDEADBEEF
emif_zqcfg_val:
	.word	0xDEADBEEF
emif_pmcr_val:
	.word	0xDEADBEEF
emif_pmcr_shdw_val:
	.word	0xDEADBEEF
emif_rd_wr_level_ramp_ctrl:
	.word	0xDEADBEEF
emif_rd_wr_exec_thresh:
	.word	0xDEADBEEF
emif_cos_config:
	.word	0xDEADBEEF
emif_priority_to_cos_mapping:
	.word	0xDEADBEEF
emif_connect_id_serv_1_map:
	.word	0xDEADBEEF
emif_connect_id_serv_2_map:
	.word	0xDEADBEEF
emif_ocp_config_val:
	.word	0xDEADBEEF
emif_lpddr2_nvm_tim:
	.word	0xDEADBEEF
emif_lpddr2_nvm_tim_shdw:
	.word	0xDEADBEEF
emif_dll_calib_ctrl_val:
	.word	0xDEADBEEF
emif_dll_calib_ctrl_val_shdw:
	.word	0xDEADBEEF
emif_ddr_phy_ctlr_1:
	.word	0xDEADBEEF
emif_ext_phy_ctrl_vals:
	.space	0x120

ENTRY(ti_emif_base_addr_virt)
        .word   0x00000000
ENTRY(ti_emif_base_addr_phys)
        .word   0x00000000
ENTRY(ti_emif_pm)
	.word	ti_emif_save_context - ti_emif_sram
	.word	ti_emif_restore_context - ti_emif_sram
	.word	ti_emif_enter_sr - ti_emif_sram
	.word	ti_emif_exit_sr - ti_emif_sram
	.word	ti_emif_abort_sr - ti_emif_sram
ENTRY(ti_emif_sram_config)
	.word	0x00000000
ENTRY(ti_emif_sram_sz)
        .word   . - ti_emif_save_context
