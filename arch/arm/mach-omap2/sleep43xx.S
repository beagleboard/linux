/*
 * Low level suspend code for AM43XX SoCs
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
 * Dave Gerlach <d-gerlach@ti.com>
 * Vaibhav Bedia <vaibhav.bedia@ti.com>
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
#include <linux/ti_emif.h>
#include <asm/memory.h>
#include <asm/assembler.h>

#include "iomap.h"
#include "cm33xx.h"
#include "pm33xx.h"
#include "prm33xx.h"
#include "prcm43xx.h"

#include "common.h"
#include "omap-secure.h"
#include "omap44xx.h"
#include <asm/hardware/cache-l2x0.h>

#define EMIF_POWER_MGMT_SR_TIMER_MASK			0x00f0

#define EMIF_POWER_MGMT_SELF_REFRESH_MODE		0x0200
#define EMIF_POWER_MGMT_SELF_REFRESH_MODE_MASK		0x0700
#define EMIF_POWER_MGMT_DELAY_PERIOD			0x1000

#define EMIF_LPDDR2_AM43XX_REFRESH_EN_CS0		0x40000002
#define EMIF_LPDDR2_AM43XX_REFRESH_EN_CS1		0xC0000002
#define AM43XX_EMIF_PHY_CTRL_REG_COUNT			0x120

#define AM33XX_CM_CLKCTRL_MODULEMODE_DISABLE		0x0003
#define AM33XX_CM_CLKCTRL_MODULEMODE_ENABLE		0x0002

#define AM43XX_EMIF_POWEROFF_ENABLE			0x1
#define AM43XX_EMIF_POWEROFF_DISABLE		0x0

#define AM43XX_CM_CLKSTCTRL_CLKTRCTRL_SW_SLEEP		0x1
#define AM43XX_CM_CLKSTCTRL_CLKTRCTRL_HW_AUTO		0x3

#define RTC_SECONDS_REG					0x0
#define RTC_PMIC_REG					0x98
#define RTC_PMIC_POWER_EN				(1 << 16)

	.text
	.align 3

/*
 * This routine is executed from internal RAM and expects some
 * parameters to be passed in r0 _strictly_ in following order:
 * 1) emif_addr_virt - ioremapped EMIF address
 * 2) wfi_flags - 2 -> DDR2, 3-> DDR3
 * 3) dram_sync_word - uncached word in SDRAM
 * 4) rtc_base_addr - ioremapped RTC base address
 * 5) l2_virt_base - L2CC ioremapped address for AM437x
 * 6) cke_override_virt - CKE override virtual address
 *
 * The code loads these values taking r0 value as reference to
 * the array in registers starting from r0, i.e emif_addr_virt
 * goes to r1, wfi_flags goes to r2 and and so on. These are
 * then saved into memory locations before proceeding with the
 * sleep sequence and hence registers r0, r1 etc can still be
 * used in the rest of the sleep code.
 */

ENTRY(am43xx_do_wfi)
	stmfd	sp!, {r4 - r11, lr}	@ save registers on stack

	ldm	r0, {r1-r6}		@ gather values passed

	/* Save the values passed */
	str	r1, emif_addr_virt
	str	r2, wfi_flags
	str	r3, dram_sync_word
	str	r4, rtc_base_addr
	str	r5, l2_base_virt
	str	r6, cke_override_virt

	ldr	r0, emif_addr_virt
	/* Save EMIF configuration */
	ldr	r1, [r0, #EMIF_SDRAM_CONFIG]
	str	r1, emif_sdcfg_val
	ldr	r1, [r0, #EMIF_SDRAM_REFRESH_CONTROL]
	str	r1, emif_ref_ctrl_val
	ldr	r1, [r0, #EMIF_SDRAM_REFRESH_CTRL_SHDW]
	str	r1, emif_ref_ctrl_val_shdw
	ldr	r1, [r0, #EMIF_SDRAM_TIMING_1]
	str	r1, emif_timing1_val
	ldr	r1, [r0, #EMIF_SDRAM_TIMING_1_SHDW]
	str	r1, emif_timing1_val_shdw
	ldr	r1, [r0, #EMIF_SDRAM_TIMING_2]
	str	r1, emif_timing2_val
	ldr	r1, [r0, #EMIF_SDRAM_TIMING_2_SHDW]
	str	r1, emif_timing2_val_shdw
	ldr	r1, [r0, #EMIF_SDRAM_TIMING_3]
	str	r1, emif_timing3_val
	ldr	r1, [r0, #EMIF_SDRAM_TIMING_3_SHDW]
	str	r1, emif_timing3_val_shdw
	ldr	r1, [r0, #EMIF_POWER_MANAGEMENT_CONTROL]
	str	r1, emif_pmcr_val
	ldr	r1, [r0, #EMIF_POWER_MANAGEMENT_CTRL_SHDW]
	str	r1, emif_pmcr_shdw_val
	ldr	r1, [r0, #EMIF_SDRAM_OUTPUT_IMPEDANCE_CALIBRATION_CONFIG]
	str	r1, emif_zqcfg_val
	ldr	r1, [r0, #EMIF_DDR_PHY_CTRL_1]
	str	r1, emif_ddr_phy_ctlr_1
	ldr	r1, [r0, #EMIF_READ_WRITE_LEVELING_RAMP_CONTROL]
	str	r1, emif_rd_wr_level_ramp_ctrl
	ldr	r1, [r0, #EMIF_READ_WRITE_EXECUTION_THRESHOLD]
	str	r1, emif_rd_wr_exec_thresh
	ldr	r1, [r0, #EMIF_COS_CONFIG]
	str	r1, emif_cos_config
	ldr	r1, [r0, #EMIF_PRIORITY_TO_CLASS_OF_SERVICE_MAPPING]
	str	r1, emif_priority_to_cos_mapping
	ldr	r1, [r0, #EMIF_CONNECTION_ID_TO_CLASS_OF_SERVICE_1_MAPPING]
	str	r1, emif_connect_id_serv_1_map
	ldr	r1, [r0, #EMIF_CONNECTION_ID_TO_CLASS_OF_SERVICE_2_MAPPING]
	str	r1, emif_connect_id_serv_2_map
	ldr	r1, [r0, #EMIF_LPDDR2_NVM_TIMING]
	str	r1, emif_lpddr2_nvm_tim
	ldr	r1, [r0, #EMIF_LPDDR2_NVM_TIMING_SHDW]
	str	r1, emif_lpddr2_nvm_tim_shdw
	ldr	r1, [r0, #EMIF_OCP_CONFIG]
	str	r1, emif_ocp_config_val
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
	dsb

	/*
	 * Invalidate L1 and L2 data cache.
	 */
	ldr	r1, kernel_flush
	blx	r1

	/*
	 * The kernel doesn't interwork: v7_flush_dcache_all in particluar will
	 * always return in Thumb state when CONFIG_THUMB2_KERNEL is enabled.
	 * This sequence switches back to ARM.  Note that .align may insert a
	 * nop: bx pc needs to be word-aligned in order to work.
	 */
 THUMB(	.thumb		)
 THUMB(	.align		)
 THUMB(	bx	pc	)
 THUMB(	nop		)
	.arm

#ifdef CONFIG_CACHE_L2X0
	/*
	 * Clean and invalidate the L2 cache.
	 */
#ifdef CONFIG_PL310_ERRATA_727915
	mov	r0, #0x03
	mov	r12, #OMAP4_MON_L2X0_DBG_CTRL_INDEX
	dsb
	smc	#0
	dsb
#endif
	ldr	r0, l2_base_virt
	mov	r2, r0
	ldr	r0, [r2, #L2X0_AUX_CTRL]
	str	r0, l2_aux_ctrl_val
	ldr	r0, [r2, #L2X0_PREFETCH_CTRL]
	str	r0, l2_prefetch_ctrl_val

	ldr	r0, l2_val
	str	r0, [r2, #L2X0_CLEAN_INV_WAY]
wait:
	ldr	r0, [r2, #L2X0_CLEAN_INV_WAY]
	ldr	r1, l2_val
	ands	r0, r0, r1
	bne	wait
#ifdef CONFIG_PL310_ERRATA_727915
	mov	r0, #0x00
	mov	r12, #OMAP4_MON_L2X0_DBG_CTRL_INDEX
	dsb
	smc	#0
	dsb
#endif
l2x_sync:
	ldr	r0, l2_base_virt
	mov	r2, r0
	mov	r0, #0x0
	str	r0, [r2, #L2X0_CACHE_SYNC]
sync:
	ldr	r0, [r2, #L2X0_CACHE_SYNC]
	ands	r0, r0, #0x1
	bne	sync
#endif

	ldr	r1, am43xx_virt_mpu_clkctrl
	ldr	r2, [r1]

	ldr	r1, am43xx_virt_mpu_clkstctrl
	ldr	r2, [r1]

	ldr	r1, am43xx_virt_emif_clkctrl
	ldr	r2, [r1]

	ldr	r0, emif_addr_virt
	/* Put SDRAM in self-refresh */
	ldr	r1, [r0, #EMIF_POWER_MANAGEMENT_CONTROL]
	bic	r1, r1, #EMIF_POWER_MGMT_SELF_REFRESH_MODE_MASK
	orr	r1, r1, #EMIF_POWER_MGMT_SELF_REFRESH_MODE
	str	r1, [r0, #EMIF_POWER_MANAGEMENT_CONTROL]

	ldr	r1, dram_sync_word	@ access DRAM
	ldr	r2, [r1, #0]
	str	r2, [r1, #0]
	isb

	mov	r1, #EMIF_POWER_MGMT_DELAY_PERIOD	@ Wait for system
wait_self_refresh:					@ to enter SR
	subs	r1, r1, #1
	bne	wait_self_refresh

	/* Disable EMIF */
	ldr	r1, am43xx_virt_emif_clkctrl
	ldr	r2, [r1]
	bic	r2, r2, #AM33XX_CM_CLKCTRL_MODULEMODE_DISABLE
	str	r2, [r1]

wait_emif_disable:
	ldr	r2, [r1]
	ldr	r3, module_disabled_val
	cmp	r2, r3
	bne	wait_emif_disable

	/*enable CKE override for both CKE0 and CKE1*/
	/*ensures CKE stays low throughout sleep*/
	ldr    r2, cke_override_virt
	mov    r1, #0
	str    r1, [r2]

	ldr	r1, wfi_flags
	tst	r1, #WFI_RTC_ONLY
	beq	am43xx_deep_sleep_suspend

	ldr	r1, rtc_base_addr
	ldr	r0, [r1, #RTC_PMIC_REG]
	orr	r0, r0, #RTC_PMIC_POWER_EN
	str	r0, [r1, #RTC_PMIC_REG]
	ldr	r0, [r1, #RTC_PMIC_REG]
	/* Wait for 2 seconds to lose power */
	mov	r3, #2
	ldr	r2, [r1, #RTC_SECONDS_REG]
rtc_loop:
	ldr	r0, [r1, #RTC_SECONDS_REG]
	cmp	r0, r2
	beq	rtc_loop
	mov	r2, r0
	subs	r3, r3, #1
	bne	rtc_loop

	b	re_enable_emif

am43xx_deep_sleep_suspend:
	/*
	 * For the MPU WFI to be registered as an interrupt
	 * to WKUP_M3, MPU_CLKCTRL.MODULEMODE needs to be set
	 * to DISABLED
	 */
	ldr	r1, am43xx_virt_mpu_clkctrl
	ldr	r2, [r1]
	bic	r2, r2, #AM33XX_CM_CLKCTRL_MODULEMODE_DISABLE
	str	r2, [r1]

	/*
	 * Put MPU CLKDM to SW_SLEEP
	 */
	ldr	r1, am43xx_virt_mpu_clkstctrl
	mov	r2, #AM43XX_CM_CLKSTCTRL_CLKTRCTRL_SW_SLEEP
	str	r2, [r1]
bus_sync:
	ldr	r1, ddr_start		@ access SRAM
	str	r1, ddr_start

	isb

	/*
	 * Execute a barrier instruction to ensure that all cache,
	 * TLB and branch predictor maintenance operations issued
	 * have completed.
	 */
	dsb
	dmb
	isb

	/*
	 * Execute a WFI instruction and wait until the
	 * STANDBYWFI output is asserted to indicate that the
	 * CPU is in idle and low power state. CPU can specualatively
	 * prefetch the instructions so add NOPs after WFI. Sixteen
	 * NOPs as per Cortex-A9 pipeline.
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
	nop
	nop
	nop

	/* We come here in case of an abort due to a late interrupt */
	ldr	r1, am43xx_virt_mpu_clkstctrl
	mov	r2, #AM43XX_CM_CLKSTCTRL_CLKTRCTRL_HW_AUTO
	str	r2, [r1]

	/* Set MPU_CLKCTRL.MODULEMODE back to ENABLE */
	ldr	r1, am43xx_virt_mpu_clkctrl
	mov	r2, #AM33XX_CM_CLKCTRL_MODULEMODE_ENABLE
	str	r2, [r1]

re_enable_emif:
	/* Re-enable EMIF */
	ldr	r1, am43xx_virt_emif_clkctrl
	mov	r2, #AM33XX_CM_CLKCTRL_MODULEMODE_ENABLE
	str	r2, [r1]
wait_emif_enable:
	ldr	r3, [r1]
	cmp	r2, r3
	bne	wait_emif_enable

	/* disable CKE override for both CKE0 and CKE1 */
	ldr    r2, cke_override_virt
	mov    r1, #3
	str    r1, [r2]

	/* Disable EMIF self-refresh */
	ldr	r0, emif_addr_virt
	ldr	r1, [r0, #EMIF_POWER_MANAGEMENT_CONTROL]
	bic	r1, r1, #LP_MODE_MASK
	str	r1, [r0, #EMIF_POWER_MANAGEMENT_CONTROL]
	str	r1, [r0, #EMIF_POWER_MANAGEMENT_CTRL_SHDW]

	/*
	 * Set SCTLR.C bit to allow data cache allocation
	 */
	mrc	p15, 0, r0, c1, c0, 0
	orr	r0, r0, #(1 << 2)	@ Enable the C bit
	mcr	p15, 0, r0, c1, c0, 0
	isb

	/* EMIF needs some time before read/write possible */
	mov r0, #EMIF_POWER_MGMT_DELAY_PERIOD
wait_abt:
	subs   r0, r0, #1
	bne wait_abt

	/* Let the suspend code know about the abort */
	mov	r0, #1
	ldmfd	sp!, {r4 - r11, pc}	@ restore regs and return
ENDPROC(am43xx_do_wfi)

	.align
ENTRY(am43xx_resume_offset)
	.word . - am43xx_do_wfi

ENTRY(am43xx_resume_from_deep_sleep)
	/* Set MPU CLKSTCTRL to HW AUTO so that CPUidle works properly */
	ldr	r1, am43xx_phys_mpu_clkstctrl
	mov	r2, #AM43XX_CM_CLKSTCTRL_CLKTRCTRL_HW_AUTO
	str	r2, [r1]

	/* For AM43xx, use EMIF power down until context is restored */
	ldr	r2, am43xx_phys_emif_poweroff
	mov	r1, #AM43XX_EMIF_POWEROFF_ENABLE
	str	r1, [r2, #0x0]

skip_emif_pwrdn_config:
	/* Re-enable EMIF */
	ldr	r1, am43xx_phys_emif_clkctrl
	mov	r2, #AM33XX_CM_CLKCTRL_MODULEMODE_ENABLE
	str	r2, [r1]
wait_emif_enable1:
	ldr	r3, [r1]
	cmp	r2, r3
	bne	wait_emif_enable1

	/* Config EMIF Timings */
	ldr	r0, emif_phys_addr
	ldr	r1, emif_ref_ctrl_val
	str	r1, [r0, #EMIF_SDRAM_REFRESH_CONTROL]
	ldr	r1, emif_ref_ctrl_val_shdw
	str	r1, [r0, #EMIF_SDRAM_REFRESH_CTRL_SHDW]
	ldr	r1, emif_timing1_val
	str	r1, [r0, #EMIF_SDRAM_TIMING_1]
	ldr	r1, emif_timing1_val_shdw
	str	r1, [r0, #EMIF_SDRAM_TIMING_1_SHDW]
	ldr	r1, emif_timing2_val
	str	r1, [r0, #EMIF_SDRAM_TIMING_2]
	ldr	r1, emif_timing2_val_shdw
	str	r1, [r0, #EMIF_SDRAM_TIMING_2_SHDW]
	ldr	r1, emif_timing3_val
	str	r1, [r0, #EMIF_SDRAM_TIMING_3]
	ldr	r1, emif_timing3_val_shdw
	str	r1, [r0, #EMIF_SDRAM_TIMING_3_SHDW]
	ldr	r1, emif_ddr_phy_ctlr_1
	str	r1, [r0, #EMIF_DDR_PHY_CTRL_1]
	str	r1, [r0, #EMIF_DDR_PHY_CTRL_1_SHDW]
	ldr	r1, emif_rd_wr_level_ramp_ctrl
	str	r1, [r0, #EMIF_READ_WRITE_LEVELING_RAMP_CONTROL]
	ldr	r1, emif_rd_wr_exec_thresh
	str	r1, [r0, #EMIF_READ_WRITE_EXECUTION_THRESHOLD]
	ldr	r1, emif_cos_config
	str	r1, [r0, #EMIF_COS_CONFIG]
	ldr	r1, emif_priority_to_cos_mapping
	str	r1, [r0, #EMIF_PRIORITY_TO_CLASS_OF_SERVICE_MAPPING]
	ldr	r1, emif_connect_id_serv_1_map
	str	r1, [r0, #EMIF_CONNECTION_ID_TO_CLASS_OF_SERVICE_1_MAPPING]
	ldr	r1, emif_connect_id_serv_2_map
	str	r1, [r0, #EMIF_CONNECTION_ID_TO_CLASS_OF_SERVICE_2_MAPPING]
	ldr	r1, emif_pmcr_val
	str	r1, [r0, #EMIF_POWER_MANAGEMENT_CONTROL]
	ldr	r1, emif_pmcr_shdw_val
	str	r1, [r0, #EMIF_POWER_MANAGEMENT_CTRL_SHDW]
	ldr	r1, emif_lpddr2_nvm_tim
	str	r1, [r0, #EMIF_LPDDR2_NVM_TIMING]
	ldr	r1, emif_lpddr2_nvm_tim_shdw
	str	r1, [r0, #EMIF_LPDDR2_NVM_TIMING_SHDW]
	ldr	r1, emif_ocp_config_val
	str	r1, [r0, #EMIF_OCP_CONFIG]
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

	/*disable CKE override for both CKE0 and CKE1*/
	ldr    r2, cke_override_phys
	mov    r1, #3
	str    r1, [r2]

	/*
	 * Toggle EMIF to exit refresh mode:
	 * if EMIF lost context, PWR_MGT_CTRL is currently 0, writing disable
	 *   (0x0), wont do diddly squat! so do a toggle from SR(0x2) to disable
	 *   (0x0) here.
	 * *If* EMIF did not loose context, nothing broken as we write the same
	 *   value(0x2) to reg before we write a disable (0x0).
	 */
	ldr	r1, emif_pmcr_val
	bic	r1, r1, #EMIF_POWER_MGMT_SELF_REFRESH_MODE_MASK
	orr	r1, r1, #EMIF_POWER_MGMT_SELF_REFRESH_MODE
	str	r1, [r0, #EMIF_POWER_MANAGEMENT_CONTROL]

	ldr	r2, am43xx_phys_emif_poweroff
	mov	r1, #AM43XX_EMIF_POWEROFF_DISABLE
	str	r1, [r2, #0x0]

	ldr	r1, emif_pmcr_val
	bic	r1, r1, #EMIF_POWER_MGMT_SELF_REFRESH_MODE_MASK
	str	r1, [r0, #EMIF_POWER_MANAGEMENT_CONTROL]

	/* EMIF needs some time before read/write possible */
	mov	r1, #EMIF_POWER_MGMT_DELAY_PERIOD
wait_resume:
	subs	r1, r1, #1
	bne	wait_resume

#ifdef CONFIG_CACHE_L2X0
	ldr	r2, l2_cache_base
	ldr	r0, [r2, #L2X0_CTRL]
	and	r0, #0x0f
	cmp	r0, #1
	beq	skip_l2en			@ Skip if already enabled
	ldr     r0, l2_prefetch_ctrl_val

	ldr	r12, l2_smc1
	dsb
	smc	#0
	dsb
set_aux_ctrl:
	ldr	r0, l2_aux_ctrl_val
	ldr	r12, l2_smc2
	dsb
	smc	#0
	dsb

	/* L2 invalidate on resume */
	ldr	r0, l2_val
	ldr	r2, l2_cache_base
	str	r0, [r2, #L2X0_INV_WAY]
wait2:
	ldr	r0, [r2, #L2X0_INV_WAY]
	ldr	r1, l2_val
	ands	r0, r0, r1
	bne	wait2
#ifdef CONFIG_PL310_ERRATA_727915
	mov	r0, #0x00
	mov	r12, #OMAP4_MON_L2X0_DBG_CTRL_INDEX
	dsb
	smc	#0
	dsb
#endif
l2x_sync2:
	ldr	r2, l2_cache_base
	mov	r0, #0x0
	str	r0, [r2, #L2X0_CACHE_SYNC]
sync2:
	ldr	r0, [r2, #L2X0_CACHE_SYNC]
	ands	r0, r0, #0x1
	bne	sync2

	mov	r0, #0x1
	ldr	r12, l2_smc3
	dsb
	smc	#0
	dsb
#endif

skip_l2en:

	/* We are back. Branch to the common CPU resume routine */
	mov	r0, #0
	ldr	pc, resume_addr
ENDPROC(am43xx_resume_from_deep_sleep)

/*
 * Local variables
 */
	.align
resume_addr:
	.word	cpu_resume - PAGE_OFFSET + 0x80000000
kernel_flush:
	.word   v7_flush_dcache_all
ddr_start:
	.word	PAGE_OFFSET
emif_phys_addr:
	.word	AM33XX_EMIF_BASE
wfi_flags:
	.word	0
rtc_base_addr:
	.word	(0xdeadbeef)

cke_override_virt:
	.word	0xDEADBEEF
cke_override_phys:
	.word	AM43XX_CTRL_CKE_OVERRIDE

am43xx_phys_emif_poweroff:
	.word   (AM43XX_CM_BASE + AM43XX_PRM_DEVICE_INST + \
		 AM43XX_PRM_EMIF_CTRL_OFFSET)
am43xx_virt_mpu_pwrstctrl:
	.word	(AM43XX_PM_MPU_PWRSTCTRL)
am43xx_virt_mpu_clkstctrl:
	.word	(AM43XX_CM_MPU_CLKSTCTRL)
am43xx_phys_mpu_clkstctrl:
	.word	(AM43XX_CM_BASE + AM43XX_CM_MPU_INST + \
		 AM43XX_CM_MPU_MPU_CDOFFS)
am43xx_virt_mpu_clkctrl:
	.word	(AM43XX_CM_MPU_MPU_CLKCTRL)
am43xx_virt_emif_clkctrl:
	.word	(AM43XX_CM_PER_EMIF_CLKCTRL)
am43xx_phys_emif_clkctrl:
	.word	(AM43XX_CM_BASE + AM43XX_CM_PER_INST + \
		 AM43XX_CM_PER_EMIF_CLKCTRL_OFFSET)
module_disabled_val:
	.word	0x30000

/* L2 cache related defines for AM437x */
l2_aux_ctrl_val:
	.word	0xDEADBEEF
l2_prefetch_ctrl_val:
	.word	0xDEADBEEF
l2_cache_base:
	.word	OMAP44XX_L2CACHE_BASE
l2_val:
	.word	0xffff
l2_smc1:
	.word	OMAP4_MON_L2X0_PREFETCH_INDEX
l2_smc2:
	.word	OMAP4_MON_L2X0_AUXCTRL_INDEX
l2_smc3:
	.word	OMAP4_MON_L2X0_CTRL_INDEX

/* DDR related defines */
dram_sync_word:
	.word	0xDEADBEEF
l2_base_virt:
	.word	0xDEADBEEF
emif_addr_virt:
	.word	0xDEADBEEF
emif_sdcfg_val:
	.word	0xDEADBEEF
emif_timing1_val:
	.word	0xDEADBEEF
emif_timing1_val_shdw:
	.word	0xDEADBEEF
emif_timing2_val:
	.word	0xDEADBEEF
emif_timing2_val_shdw:
	.word	0xDEADBEEF
emif_timing3_val:
	.word	0xDEADBEEF
emif_timing3_val_shdw:
	.word	0xDEADBEEF
emif_rd_lat_val:
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
emif_ref_ctrl_val:
	.word	0xDEADBEEF
emif_ref_ctrl_val_shdw:
	.word	0xDEADBEEF
emif_zqcfg_val:
	.word	0xDEADBEEF
emif_ocp_config_val:
	.word	0xDEADBEEF
emif_ocp_config_val_shdw:
	.word	0xDEADBEEF
emif_pmcr_val:
	.word	0xDEADBEEF
emif_pmcr_shdw_val:
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

	.align 3

ENTRY(am43xx_do_wfi_sz)
	.word	. - am43xx_do_wfi
