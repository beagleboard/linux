/*
 * linux/arch/arm/mach-omap2/pm.c
 *
 * OMAP2 Power Management Routines
 *
 * Copyright (C) 2005 Texas Instruments, Inc.
 * Copyright (C) 2006-2008 Nokia Corporation
 *
 * Written by:
 * Richard Woodruff <r-woodruff2@ti.com>
 * Tony Lindgren
 * Juha Yrjola
 * Amit Kucheria <amit.kucheria@nokia.com>
 * Igor Stoppa <igor.stoppa@nokia.com>
 *
 * Based on pm.c for omap1
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/suspend.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/irq.h>

#include <asm/mach/time.h>
#include <asm/mach/irq.h>
#include <asm/mach-types.h>

#include <asm/arch/irqs.h>
#include <asm/arch/clock.h>
#include <asm/arch/sram.h>
#include <asm/arch/control.h>
#include <asm/arch/gpio.h>
#include <asm/arch/pm.h>
#include <asm/arch/mux.h>
#include <asm/arch/dma.h>
#include <asm/arch/board.h>

#include "prm.h"
#include "prm-regbits-24xx.h"
#include "cm.h"
#include "cm-regbits-24xx.h"
#include "sdrc.h"
#include "pm.h"

/* These addrs are in assembly language code to be patched at runtime */
extern void *omap2_ocs_sdrc_power;
extern void *omap2_ocs_sdrc_dlla_ctrl;

static void (*omap2_sram_idle)(void);
static void (*omap2_sram_suspend)(void __iomem *dllctrl);
static void (*saved_idle)(void);

static struct clk *osc_ck, *emul_ck;

static int omap2_fclks_active(void)
{
	u32 f1, f2;

	f1 = cm_read_mod_reg(CORE_MOD, CM_FCLKEN1);
	f2 = cm_read_mod_reg(CORE_MOD, OMAP24XX_CM_FCLKEN2);
	serial_console_fclk_mask(&f1, &f2);
	if (f1 | f2)
		return 1;
	return 0;
}

static void omap2_enter_full_retention(void)
{
	u32 l, sleep_time = 0;

	/* There is 1 reference hold for all children of the oscillator
	 * clock, the following will remove it. If no one else uses the
	 * oscillator itself it will be disabled if/when we enter retention
	 * mode.
	 */
	clk_disable(osc_ck);

	/* Clear old wake-up events */
	/* REVISIT: These write to reserved bits? */
	prm_write_mod_reg(0xffffffff, CORE_MOD, PM_WKST1);
	prm_write_mod_reg(0xffffffff, CORE_MOD, OMAP24XX_PM_WKST2);
	prm_write_mod_reg(0xffffffff, WKUP_MOD, PM_WKST);

	/* Try to enter retention */
	prm_write_mod_reg((0x01 << OMAP_POWERSTATE_SHIFT) | OMAP_LOGICRETSTATE,
			  MPU_MOD, PM_PWSTCTRL);

	/* Workaround to kill USB */
	l = omap_ctrl_readl(OMAP2_CONTROL_DEVCONF0) | OMAP24XX_USBSTANDBYCTRL;
	omap_ctrl_writel(l, OMAP2_CONTROL_DEVCONF0);

	omap2_gpio_prepare_for_retention();

	if (omap2_pm_debug) {
		omap2_pm_dump(0, 0, 0);
		sleep_time = omap2_read_32k_sync_counter();
	}

	/* One last check for pending IRQs to avoid extra latency due
	 * to sleeping unnecessarily. */
	if (omap_irq_pending())
		goto no_sleep;

	serial_console_sleep(1);
	/* Jump to SRAM suspend code */
	omap2_sram_suspend(OMAP_SDRC_REGADDR(SDRC_DLLA_CTRL));
no_sleep:
	serial_console_sleep(0);

	if (omap2_pm_debug) {
		unsigned long long tmp;
		u32 resume_time;

		resume_time = omap2_read_32k_sync_counter();
		tmp = resume_time - sleep_time;
		tmp *= 1000000;
		omap2_pm_dump(0, 1, tmp / 32768);
	}
	omap2_gpio_resume_after_retention();

	clk_enable(osc_ck);

	/* clear CORE wake-up events */
	prm_write_mod_reg(0xffffffff, CORE_MOD, PM_WKST1);
	prm_write_mod_reg(0xffffffff, CORE_MOD, OMAP24XX_PM_WKST2);

	/* wakeup domain events - bit 1: GPT1, bit5 GPIO */
	prm_clear_mod_reg_bits(0x4 | 0x1, WKUP_MOD, PM_WKST);

	/* MPU domain wake events */
	l = __raw_readl(OMAP24XX_PRCM_IRQSTATUS_MPU);
	if (l & 0x01)
		__raw_writel(0x01, OMAP24XX_PRCM_IRQSTATUS_MPU);
	if (l & 0x20)
		__raw_writel(0x20, OMAP24XX_PRCM_IRQSTATUS_MPU);

	/* Mask future PRCM-to-MPU interrupts */
	__raw_writel(0x0, OMAP24XX_PRCM_IRQSTATUS_MPU);
}

static int omap2_i2c_active(void)
{
	u32 l;

	l = cm_read_mod_reg(CORE_MOD, CM_FCLKEN1);
	return l & (OMAP2420_EN_I2C2 | OMAP2420_EN_I2C1);
}

static int sti_console_enabled;

static int omap2_allow_mpu_retention(void)
{
	u32 l;

	if (atomic_read(&sleep_block))
		return 0;

	/* Check for MMC, UART2, UART1, McSPI2, McSPI1 and DSS1. */
	l = cm_read_mod_reg(CORE_MOD, CM_FCLKEN1);
	if (l & (OMAP2420_EN_MMC | OMAP24XX_EN_UART2 |
		 OMAP24XX_EN_UART1 | OMAP24XX_EN_MCSPI2 |
		 OMAP24XX_EN_MCSPI1 | OMAP24XX_EN_DSS1))
		return 0;
	/* Check for UART3. */
	l = cm_read_mod_reg(CORE_MOD, OMAP24XX_CM_FCLKEN2);
	if (l & OMAP24XX_EN_UART3)
		return 0;
	if (sti_console_enabled)
		return 0;

	return 1;
}

static void omap2_enter_mpu_retention(void)
{
	u32 sleep_time = 0;
	int only_idle = 0;

	/* Putting MPU into the WFI state while a transfer is active
	 * seems to cause the I2C block to timeout. Why? Good question. */
	if (omap2_i2c_active())
		return;

	/* The peripherals seem not to be able to wake up the MPU when
	 * it is in retention mode. */
	if (omap2_allow_mpu_retention()) {
		/* REVISIT: These write to reserved bits? */
		prm_write_mod_reg(0xffffffff, CORE_MOD, PM_WKST1);
		prm_write_mod_reg(0xffffffff, CORE_MOD, OMAP24XX_PM_WKST2);
		prm_write_mod_reg(0xffffffff, WKUP_MOD, PM_WKST);

		/* Try to enter MPU retention */
		prm_write_mod_reg((0x01 << OMAP_POWERSTATE_SHIFT) |
				  OMAP_LOGICRETSTATE,
				  MPU_MOD, PM_PWSTCTRL);
	} else {
		/* Block MPU retention */

		prm_write_mod_reg(OMAP_LOGICRETSTATE, MPU_MOD, PM_PWSTCTRL);
		only_idle = 1;
	}

	if (omap2_pm_debug) {
		omap2_pm_dump(only_idle ? 2 : 1, 0, 0);
		sleep_time = omap2_read_32k_sync_counter();
	}

	omap2_sram_idle();

	if (omap2_pm_debug) {
		unsigned long long tmp;
		u32 resume_time;

		resume_time = omap2_read_32k_sync_counter();
		tmp = resume_time - sleep_time;
		tmp *= 1000000;
		omap2_pm_dump(only_idle ? 2 : 1, 1, tmp / 32768);
	}
}

static int omap2_can_sleep(void)
{
	if (!enable_dyn_sleep)
		return 0;
	if (omap2_fclks_active())
		return 0;
	if (atomic_read(&sleep_block) > 0)
		return 0;
	if (clk_get_usecount(osc_ck) > 1)
		return 0;
	if (omap_dma_running())
		return 0;

	return 1;
}

static void omap2_pm_idle(void)
{
	local_irq_disable();
	local_fiq_disable();

	if (!omap2_can_sleep()) {
		/* timer_dyn_reprogram() takes about 100-200 us to complete.
		 * In some contexts (e.g. when waiting for a GPMC-SDRAM DMA
		 * transfer to complete), the increased latency is too much.
		 *
		 * omap2_block_sleep() and omap2_allow_sleep() can be used
		 * to indicate this.
		 */
		if (atomic_read(&sleep_block) == 0) {
			timer_dyn_reprogram();
			if (omap_irq_pending())
				goto out;
		}
		omap2_enter_mpu_retention();
		goto out;
	}

	/*
	 * Since an interrupt may set up a timer, we don't want to
	 * reprogram the hardware timer with interrupts enabled.
	 * Re-enable interrupts only after returning from idle.
	 */
	timer_dyn_reprogram();

	if (omap_irq_pending())
		goto out;

	omap2_enter_full_retention();

out:
	local_fiq_enable();
	local_irq_enable();
}

static int omap2_pm_prepare(void)
{
	/* We cannot sleep in idle until we have resumed */
	saved_idle = pm_idle;
	pm_idle = NULL;

	return 0;
}

static int omap2_pm_suspend(void)
{
	u32 wken_wkup, mir1;

	wken_wkup = prm_read_mod_reg(WKUP_MOD, PM_WKEN);
	prm_write_mod_reg(wken_wkup & ~OMAP24XX_EN_GPT1, WKUP_MOD, PM_WKEN);

	/* Mask GPT1 */
	mir1 = omap_readl(0x480fe0a4);
	omap_writel(1 << 5, 0x480fe0ac);

	omap2_enter_full_retention();

	omap_writel(mir1, 0x480fe0a4);
	prm_write_mod_reg(wken_wkup, WKUP_MOD, PM_WKEN);

	return 0;
}

static int omap2_pm_enter(suspend_state_t state)
{
	int ret = 0;

	switch (state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		ret = omap2_pm_suspend();
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static void omap2_pm_finish(void)
{
	pm_idle = saved_idle;
}

static struct platform_suspend_ops omap_pm_ops = {
	.prepare	= omap2_pm_prepare,
	.enter		= omap2_pm_enter,
	.finish		= omap2_pm_finish,
	.valid		= suspend_valid_only_mem,
};

static void __init prcm_setup_regs(void)
{
	u32 l;

	/* Enable autoidle */
	__raw_writel(OMAP24XX_AUTOIDLE, OMAP24XX_PRCM_SYSCONFIG);

	/* Set all domain wakeup dependencies */
	prm_write_mod_reg(OMAP_EN_WKUP_MASK, MPU_MOD, PM_WKDEP);
	prm_write_mod_reg(0, OMAP24XX_DSP_MOD, PM_WKDEP);
	prm_write_mod_reg(0, GFX_MOD, PM_WKDEP);
	prm_write_mod_reg(0, CORE_MOD, PM_WKDEP);
	if (cpu_is_omap2430())
		prm_write_mod_reg(0, OMAP2430_MDM_MOD, PM_WKDEP);

	l = prm_read_mod_reg(CORE_MOD, PM_PWSTCTRL);
	/* Enable retention for all memory blocks */
	l |= OMAP24XX_MEM3RETSTATE | OMAP24XX_MEM2RETSTATE |
		OMAP24XX_MEM1RETSTATE;

	/* Set power state to RETENTION */
	l &= ~OMAP_POWERSTATE_MASK;
	l |= 0x01 << OMAP_POWERSTATE_SHIFT;
	prm_write_mod_reg(l, CORE_MOD, PM_PWSTCTRL);

	prm_write_mod_reg((0x01 << OMAP_POWERSTATE_SHIFT) |
			  OMAP_LOGICRETSTATE,
			  MPU_MOD, PM_PWSTCTRL);

	/* Power down DSP and GFX */
	prm_write_mod_reg(OMAP24XX_FORCESTATE | (0x3 << OMAP_POWERSTATE_SHIFT),
			  OMAP24XX_DSP_MOD, PM_PWSTCTRL);
	prm_write_mod_reg(OMAP24XX_FORCESTATE | (0x3 << OMAP_POWERSTATE_SHIFT),
			  GFX_MOD, PM_PWSTCTRL);

	/* Enable clock auto control for all domains */
	cm_write_mod_reg(OMAP24XX_AUTOSTATE_MPU_MASK, MPU_MOD, CM_CLKSTCTRL);
	cm_write_mod_reg(OMAP24XX_AUTOSTATE_DSS_MASK |
			 OMAP24XX_AUTOSTATE_L4_MASK |
			 OMAP24XX_AUTOSTATE_L3_MASK,
			 CORE_MOD, CM_CLKSTCTRL);
	cm_write_mod_reg(OMAP24XX_AUTOSTATE_GFX_MASK, GFX_MOD, CM_CLKSTCTRL);
	cm_write_mod_reg(OMAP2420_AUTOSTATE_IVA_MASK |
			 OMAP24XX_AUTOSTATE_DSP_MASK,
			 OMAP24XX_DSP_MOD, CM_CLKSTCTRL);

	/* Enable clock autoidle for all domains */
	cm_write_mod_reg(OMAP24XX_AUTO_CAM |
			 OMAP24XX_AUTO_MAILBOXES |
			 OMAP24XX_AUTO_WDT4 |
			 OMAP2420_AUTO_WDT3 |
			 OMAP24XX_AUTO_MSPRO |
			 OMAP2420_AUTO_MMC |
			 OMAP24XX_AUTO_FAC |
			 OMAP2420_AUTO_EAC |
			 OMAP24XX_AUTO_HDQ |
			 OMAP24XX_AUTO_UART2 |
			 OMAP24XX_AUTO_UART1 |
			 OMAP24XX_AUTO_I2C2 |
			 OMAP24XX_AUTO_I2C1 |
			 OMAP24XX_AUTO_MCSPI2 |
			 OMAP24XX_AUTO_MCSPI1 |
			 OMAP24XX_AUTO_MCBSP2 |
			 OMAP24XX_AUTO_MCBSP1 |
			 OMAP24XX_AUTO_GPT12 |
			 OMAP24XX_AUTO_GPT11 |
			 OMAP24XX_AUTO_GPT10 |
			 OMAP24XX_AUTO_GPT9 |
			 OMAP24XX_AUTO_GPT8 |
			 OMAP24XX_AUTO_GPT7 |
			 OMAP24XX_AUTO_GPT6 |
			 OMAP24XX_AUTO_GPT5 |
			 OMAP24XX_AUTO_GPT4 |
			 OMAP24XX_AUTO_GPT3 |
			 OMAP24XX_AUTO_GPT2 |
			 OMAP2420_AUTO_VLYNQ |
			 OMAP24XX_AUTO_DSS,
			 CORE_MOD, CM_AUTOIDLE1);
	cm_write_mod_reg(OMAP24XX_AUTO_UART3 |
			 OMAP24XX_AUTO_SSI |
			 OMAP24XX_AUTO_USB,
			 CORE_MOD, CM_AUTOIDLE2);
	cm_write_mod_reg(OMAP24XX_AUTO_SDRC |
			 OMAP24XX_AUTO_GPMC |
			 OMAP24XX_AUTO_SDMA,
			 CORE_MOD, CM_AUTOIDLE3);
	cm_write_mod_reg(OMAP24XX_AUTO_PKA |
			 OMAP24XX_AUTO_AES |
			 OMAP24XX_AUTO_RNG |
			 OMAP24XX_AUTO_SHA |
			 OMAP24XX_AUTO_DES,
			 CORE_MOD, OMAP24XX_CM_AUTOIDLE4);

	cm_write_mod_reg(OMAP2420_AUTO_DSP_IPI, OMAP24XX_DSP_MOD, CM_AUTOIDLE);

	/* Put DPLL and both APLLs into autoidle mode */
	cm_write_mod_reg((0x03 << OMAP24XX_AUTO_DPLL_SHIFT) |
			 (0x03 << OMAP24XX_AUTO_96M_SHIFT) |
			 (0x03 << OMAP24XX_AUTO_54M_SHIFT),
			 PLL_MOD, CM_AUTOIDLE);

	cm_write_mod_reg(OMAP24XX_AUTO_OMAPCTRL |
			 OMAP24XX_AUTO_WDT1 |
			 OMAP24XX_AUTO_MPU_WDT |
			 OMAP24XX_AUTO_GPIOS |
			 OMAP24XX_AUTO_32KSYNC |
			 OMAP24XX_AUTO_GPT1,
			 WKUP_MOD, CM_AUTOIDLE);

	/* REVISIT: Configure number of 32 kHz clock cycles for sys_clk
	 * stabilisation */
	__raw_writel(15 << OMAP_SETUP_TIME_SHIFT, OMAP24XX_PRCM_CLKSSETUP);

	/* Configure automatic voltage transition */
	__raw_writel(2 << OMAP_SETUP_TIME_SHIFT, OMAP24XX_PRCM_VOLTSETUP);
	__raw_writel(OMAP24XX_AUTO_EXTVOLT |
		      (0x1 << OMAP24XX_SETOFF_LEVEL_SHIFT) |
		      OMAP24XX_MEMRETCTRL |
		      (0x1 << OMAP24XX_SETRET_LEVEL_SHIFT) |
		      (0x0 << OMAP24XX_VOLT_LEVEL_SHIFT),
		      OMAP24XX_PRCM_VOLTCTRL);

	/* Enable wake-up events */
	prm_write_mod_reg(OMAP24XX_EN_GPIOS | OMAP24XX_EN_GPT1,
			  WKUP_MOD, PM_WKEN);
}

int __init omap2_pm_init(void)
{
	u32 l;

	printk(KERN_INFO "Power Management for OMAP2 initializing\n");
	l = __raw_readl(OMAP24XX_PRCM_REVISION);
	printk(KERN_INFO "PRCM revision %d.%d\n", (l >> 4) & 0x0f, l & 0x0f);

	osc_ck = clk_get(NULL, "osc_ck");
	if (IS_ERR(osc_ck)) {
		printk(KERN_ERR "could not get osc_ck\n");
		return -ENODEV;
	}

	if (cpu_is_omap242x()) {
		emul_ck = clk_get(NULL, "emul_ck");
		if (IS_ERR(emul_ck)) {
			printk(KERN_ERR "could not get emul_ck\n");
			clk_put(osc_ck);
			return -ENODEV;
		}
	}

	prcm_setup_regs();

	pm_init_serial_console();

	/* Hack to prevent MPU retention when STI console is enabled. */
	{
		const struct omap_sti_console_config *sti;

		sti = omap_get_config(OMAP_TAG_STI_CONSOLE,
				      struct omap_sti_console_config);
		if (sti != NULL && sti->enable)
			sti_console_enabled = 1;
	}

	/*
	 * We copy the assembler sleep/wakeup routines to SRAM.
	 * These routines need to be in SRAM as that's the only
	 * memory the MPU can see when it wakes up.
	 */
	omap2_sram_idle = omap_sram_push(omap24xx_idle_loop_suspend,
					 omap24xx_idle_loop_suspend_sz);

	omap2_sram_suspend = omap_sram_push(omap24xx_cpu_suspend,
					    omap24xx_cpu_suspend_sz);

	/* Patch in the correct register addresses for multiboot */
	omap_sram_patch_va(omap24xx_cpu_suspend, &omap2_ocs_sdrc_power,
			   omap2_sram_suspend,
			   OMAP_SDRC_REGADDR(SDRC_POWER));
	omap_sram_patch_va(omap24xx_cpu_suspend, &omap2_ocs_sdrc_dlla_ctrl,
			   omap2_sram_suspend,
			   OMAP_SDRC_REGADDR(SDRC_DLLA_CTRL));

	suspend_set_ops(&omap_pm_ops);
	pm_idle = omap2_pm_idle;

	return 0;
}
