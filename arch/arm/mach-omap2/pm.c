/*
 * linux/arch/arm/mach-omap2/pm.c
 *
 * OMAP2 Power Management Routines
 *
 * Copyright (C) 2005 Texas Instruments, Inc.
 * Copyright (C) 2006 Nokia Corporation
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

#include <linux/pm.h>
#include <linux/sched.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/clk.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/atomic.h>
#include <asm/mach/time.h>
#include <asm/mach/irq.h>
#include <asm/mach-types.h>

#include <asm/arch/irqs.h>
#include <asm/arch/clock.h>
#include <asm/arch/sram.h>
#include <asm/arch/pm.h>
#include <asm/arch/mux.h>
#include <asm/arch/dma.h>
#include <asm/arch/board.h>
#include <asm/arch/gpio.h>

#include "prm.h"
#include "prm_regbits_24xx.h"
#include "cm.h"
#include "cm_regbits_24xx.h"
#include "sdrc.h"

static void (*omap2_sram_idle)(void);
static void (*omap2_sram_suspend)(void __iomem *dllctrl);
static void (*saved_idle)(void);

static u32 omap2_read_32k_sync_counter(void)
{
        return omap_readl(OMAP2_32KSYNCT_BASE + 0x0010);
}

#ifdef CONFIG_PM_DEBUG
int omap2_pm_debug = 0;

static int serial_console_clock_disabled;
static int serial_console_uart;
static unsigned int serial_console_next_disable;

static struct clk *console_iclk, *console_fclk;

static void serial_console_kick(void)
{
	serial_console_next_disable = omap2_read_32k_sync_counter();
	/* Keep the clocks on for 4 secs */
	serial_console_next_disable += 4 * 32768;
}

static void serial_wait_tx(void)
{
	static const unsigned long uart_bases[3] = {
		0x4806a000, 0x4806c000, 0x4806e000
	};
	unsigned long lsr_reg;
	int looped = 0;

	/* Wait for TX FIFO and THR to get empty */
	lsr_reg = IO_ADDRESS(uart_bases[serial_console_uart - 1] + (5 << 2));
	while ((__raw_readb(lsr_reg) & 0x60) != 0x60)
		looped = 1;
	if (looped)
		serial_console_kick();
}

static void serial_console_fclk_mask(u32 *f1, u32 *f2)
{
	switch (serial_console_uart)  {
	case 1:
		*f1 &= ~(1 << 21);
		break;
	case 2:
		*f1 &= ~(1 << 22);
		break;
	case 3:
		*f2 &= ~(1 << 2);
		break;
	}
}

static void serial_console_sleep(int enable)
{
	if (console_iclk == NULL || console_fclk == NULL)
		return;

	if (enable) {
		BUG_ON(serial_console_clock_disabled);
		if (clk_get_usecount(console_fclk) == 0)
			return;
		if ((int) serial_console_next_disable - (int) omap2_read_32k_sync_counter() >= 0)
			return;
		serial_wait_tx();
		clk_disable(console_iclk);
		clk_disable(console_fclk);
		serial_console_clock_disabled = 1;
	} else {
		int serial_wakeup = 0;
		u32 l;

		switch (serial_console_uart)  {
		case 1:
			l = prm_read_mod_reg(CORE_MOD, PM_WKST1);
			if (l & OMAP24XX_ST_UART1)
				serial_wakeup = 1;
			break;
		case 2:
			l = prm_read_mod_reg(CORE_MOD, PM_WKST1);
			if (l & OMAP24XX_ST_UART2)
				serial_wakeup = 1;
			break;
		case 3:
			l = prm_read_mod_reg(CORE_MOD, OMAP24XX_PM_WKST2);
			if (l & OMAP24XX_ST_UART3)
				serial_wakeup = 1;
			break;
		}
		if (serial_wakeup)
			serial_console_kick();
		if (!serial_console_clock_disabled)
			return;
		clk_enable(console_iclk);
		clk_enable(console_fclk);
		serial_console_clock_disabled = 0;
	}
}

static void pm_init_serial_console(void)
{
	const struct omap_serial_console_config *conf;
	char name[16];
	u32 l;

	conf = omap_get_config(OMAP_TAG_SERIAL_CONSOLE,
			       struct omap_serial_console_config);
	if (conf == NULL)
		return;
	if (conf->console_uart > 3 || conf->console_uart < 1)
		return;
	serial_console_uart = conf->console_uart;
	sprintf(name, "uart%d_fck", conf->console_uart);
	console_fclk = clk_get(NULL, name);
	if (IS_ERR(console_fclk))
		console_fclk = NULL;
	name[6] = 'i';
	console_iclk = clk_get(NULL, name);
	if (IS_ERR(console_fclk))
		console_iclk = NULL;
	if (console_fclk == NULL || console_iclk == NULL) {
		serial_console_uart = 0;
		return;
	}
	switch (serial_console_uart) {
	case 1:
		l = prm_read_mod_reg(CORE_MOD, PM_WKEN1);
		l |= OMAP24XX_ST_UART1;
		prm_write_mod_reg(l, CORE_MOD, PM_WKEN1);
		break;
	case 2:
		l = prm_read_mod_reg(CORE_MOD, PM_WKEN1);
		l |= OMAP24XX_ST_UART2;
		prm_write_mod_reg(l, CORE_MOD, PM_WKEN1);
		break;
	case 3:
		l = prm_read_mod_reg(CORE_MOD, OMAP24XX_PM_WKEN2);
		l |= OMAP24XX_ST_UART3;
		prm_write_mod_reg(l, CORE_MOD, OMAP24XX_PM_WKEN2);
		break;
	}
}

#define DUMP_PRM_MOD_REG(mod, reg)    \
	regs[reg_count].name = #mod "." #reg; \
	regs[reg_count++].val = prm_read_mod_reg(mod, reg)
#define DUMP_CM_MOD_REG(mod, reg)     \
	regs[reg_count].name = #mod "." #reg; \
	regs[reg_count++].val = cm_read_mod_reg(mod, reg)
#define DUMP_PRM_REG(reg) \
	regs[reg_count].name = #reg; \
	regs[reg_count++].val = prm_read_reg(reg)
#define DUMP_CM_REG(reg) \
	regs[reg_count].name = #reg; \
	regs[reg_count++].val = cm_read_reg(reg)
#define DUMP_INTC_REG(reg, off) \
	regs[reg_count].name = #reg; \
	regs[reg_count++].val = __raw_readl(IO_ADDRESS(0x480fe000 + (off)))

static void omap2_pm_dump(int mode, int resume, unsigned int us)
{
	struct reg {
		const char *name;
		u32 val;
	} regs[32];
	int reg_count = 0, i;
	const char *s1 = NULL, *s2 = NULL;

	if (!resume) {
#if 0
		/* MPU */
		DUMP_PRM_REG(OMAP24XX_PRCM_IRQENABLE_MPU);
		DUMP_CM_MOD_REG(MPU_MOD, CM_CLKSTCTRL);
		DUMP_PRM_MOD_REG(MPU_MOD, PM_PWSTCTRL);
		DUMP_PRM_MOD_REG(MPU_MOD, PM_PWSTST);
		DUMP_PRM_MOD_REG(MPU_MOD, PM_WKDEP);
#endif
#if 0
		/* INTC */
		DUMP_INTC_REG(INTC_MIR0, 0x0084);
		DUMP_INTC_REG(INTC_MIR1, 0x00a4);
		DUMP_INTC_REG(INTC_MIR2, 0x00c4);
#endif
#if 0
		DUMP_CM_MOD_REG(CORE_MOD, CM_FCLKEN1);
		DUMP_CM_MOD_REG(CORE_MOD, CM_FCLKEN2);
		DUMP_CM_MOD_REG(WKUP_MOD, CM_FCLKEN);
		DUMP_CM_MOD_REG(CORE_MOD, CM_ICLKEN1);
		DUMP_CM_MOD_REG(CORE_MOD, CM_ICLKEN2);
		DUMP_CM_MOD_REG(WKUP_MOD, CM_ICLKEN);
		DUMP_CM_MOD_REG(PLL_MOD, CM_CLKEN_PLL);
		DUMP_PRM_REG(OMAP24XX_PRCM_CLKEMUL_CTRL);
		DUMP_CM_MOD_REG(PLL_MOD, CM_AUTOIDLE);
		DUMP_PRM_MOD_REG(CORE_REG, PM_PWSTST);
		DUMP_PRM_REG(OMAP24XX_PRCM_CLKSRC_CTRL);
#endif
#if 0
		/* DSP */
		DUMP_CM_MOD_REG(OMAP24XX_DSP_MOD, CM_FCLKEN);
		DUMP_CM_MOD_REG(OMAP24XX_DSP_MOD, CM_ICLKEN);
		DUMP_CM_MOD_REG(OMAP24XX_DSP_MOD, CM_IDLEST);
		DUMP_CM_MOD_REG(OMAP24XX_DSP_MOD, CM_AUTOIDLE);
		DUMP_CM_MOD_REG(OMAP24XX_DSP_MOD, CM_CLKSEL);
		DUMP_CM_MOD_REG(OMAP24XX_DSP_MOD, CM_CLKSTCTRL);
		DUMP_PRM_MOD_REG(OMAP24XX_DSP_MOD, RM_RSTCTRL);
		DUMP_PRM_MOD_REG(OMAP24XX_DSP_MOD, RM_RSTST);
		DUMP_PRM_MOD_REG(OMAP24XX_DSP_MOD, PM_PWSTCTRL);
		DUMP_PRM_MOD_REG(OMAP24XX_DSP_MOD, PM_PWSTST);
#endif
	} else {
		DUMP_PRM_MOD_REG(CORE_MOD, PM_WKST1);
		DUMP_PRM_MOD_REG(CORE_MOD, OMAP24XX_PM_WKST2);
		DUMP_PRM_MOD_REG(WKUP_MOD, PM_WKST);
		DUMP_PRM_REG(OMAP24XX_PRCM_IRQSTATUS_MPU);
#if 1
		DUMP_INTC_REG(INTC_PENDING_IRQ0, 0x0098);
		DUMP_INTC_REG(INTC_PENDING_IRQ1, 0x00b8);
		DUMP_INTC_REG(INTC_PENDING_IRQ2, 0x00d8);
#endif
	}

	switch (mode) {
	case 0:
		s1 = "full";
		s2 = "retention";
		break;
	case 1:
		s1 = "MPU";
		s2 = "retention";
		break;
	case 2:
		s1 = "MPU";
		s2 = "idle";
		break;
	}

	if (!resume)
#if defined(CONFIG_NO_IDLE_HZ) || defined(CONFIG_NO_HZ)
		printk("--- Going to %s %s (next timer after %u ms)\n", s1, s2,
		       jiffies_to_msecs(get_next_timer_interrupt(jiffies) - 
					jiffies));
#else
		printk("--- Going to %s %s\n", s1, s2);
#endif
	else
		printk("--- Woke up (slept for %u.%03u ms)\n", us / 1000, us % 1000);
	for (i = 0; i < reg_count; i++)
		printk("%-20s: 0x%08x\n", regs[i].name, regs[i].val);
}

#else
static inline void serial_console_sleep(int enable) {}
static inline void pm_init_serial_console(void) {}
static inline void omap2_pm_dump(int mode, int resume, unsigned int us) {}
static inline void serial_console_fclk_mask(u32 *f1, u32 *f2) {}

#define omap2_pm_debug 0

#endif

static unsigned short enable_dyn_sleep = 0; /* disabled till drivers are fixed */

static ssize_t omap_pm_sleep_while_idle_show(struct kset * subsys, char *buf)
{
	return sprintf(buf, "%hu\n", enable_dyn_sleep);
}

static ssize_t omap_pm_sleep_while_idle_store(struct kset * subsys,
					      const char * buf,
					      size_t n)
{
	unsigned short value;
	if (sscanf(buf, "%hu", &value) != 1 ||
	    (value != 0 && value != 1)) {
		printk(KERN_ERR "idle_sleep_store: Invalid value\n");
		return -EINVAL;
	}
	enable_dyn_sleep = value;
	return n;
}

static struct subsys_attribute sleep_while_idle_attr = {
	.attr   = {
		.name = __stringify(sleep_while_idle),
		.mode = 0644,
	},
	.show   = omap_pm_sleep_while_idle_show,
	.store  = omap_pm_sleep_while_idle_store,
};

static struct clk *osc_ck, *emul_ck;

#define CONTROL_DEVCONF		__REG32(0x48000274)

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

static int omap2_irq_pending(void)
{
	u32 pending_reg = IO_ADDRESS(0x480fe098);
	int i;

	for (i = 0; i < 4; i++) {
		if (__raw_readl(pending_reg))
			return 1;
		pending_reg += 0x20;
	}
	return 0;
}

static atomic_t sleep_block = ATOMIC_INIT(0);

void omap2_block_sleep(void)
{
	atomic_inc(&sleep_block);
}

void omap2_allow_sleep(void)
{
	int i;

	i = atomic_dec_return(&sleep_block);
	BUG_ON(i < 0);
}

static void omap2_enter_full_retention(void)
{
	u32 sleep_time = 0;

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
	CONTROL_DEVCONF |= 0x00008000;

	omap2_gpio_prepare_for_retention();

	if (omap2_pm_debug) {
		omap2_pm_dump(0, 0, 0);
		sleep_time = omap2_read_32k_sync_counter();
	}

	/* One last check for pending IRQs to avoid extra latency due
	 * to sleeping unnecessarily. */
	if (omap2_irq_pending())
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
			if (omap2_irq_pending())
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

	if (omap2_irq_pending())
		goto out;

	omap2_enter_full_retention();

out:
	local_fiq_enable();
	local_irq_enable();
}

static int omap2_pm_prepare(suspend_state_t state)
{
	int error = 0;

	/* We cannot sleep in idle until we have resumed */
	saved_idle = pm_idle;
	pm_idle = NULL;

	switch (state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		break;
	default:
		return -EINVAL;
	}

	return error;
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

static int omap2_pm_finish(suspend_state_t state)
{
	pm_idle = saved_idle;
	return 0;
}

static struct pm_ops omap_pm_ops = {
	.prepare	= omap2_pm_prepare,
	.enter		= omap2_pm_enter,
	.finish		= omap2_pm_finish,
	.valid		= pm_valid_only_mem,
};

static void __init prcm_setup_regs(void)
{
	u32 l;

	/* Enable autoidle */
	prm_write_reg(OMAP24XX_AUTOIDLE, OMAP24XX_PRCM_SYSCONFIG);

	/* Set all domain wakeup dependencies */
	prm_write_mod_reg(OMAP_EN_WKUP, MPU_MOD, PM_WKDEP);
	prm_write_mod_reg(0, OMAP24XX_DSP_MOD, PM_WKDEP);
	prm_write_mod_reg(0, GFX_MOD, PM_WKDEP);

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
	cm_write_mod_reg(OMAP24XX_AUTOSTATE_MPU, MPU_MOD, CM_CLKSTCTRL);
	cm_write_mod_reg(OMAP24XX_AUTOSTATE_DSS | OMAP24XX_AUTOSTATE_L4 |
			 OMAP24XX_AUTOSTATE_L3,
			 CORE_MOD, CM_CLKSTCTRL);
	cm_write_mod_reg(OMAP24XX_AUTOSTATE_GFX, GFX_MOD, CM_CLKSTCTRL);
	cm_write_mod_reg(OMAP2420_AUTOSTATE_IVA | OMAP24XX_AUTOSTATE_DSP,
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
			 CORE_MOD, OMAP24XX_CM_AUTOIDLE3);
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
	prm_write_reg(15 << OMAP_SETUP_TIME_SHIFT, OMAP24XX_PRCM_CLKSSETUP);

	/* Configure automatic voltage transition */
	prm_write_reg(2 << OMAP_SETUP_TIME_SHIFT, OMAP24XX_PRCM_VOLTSETUP);
	prm_write_reg(OMAP24XX_AUTO_EXTVOLT |
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
	l = prm_read_reg(OMAP24XX_PRCM_REVISION);
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

	pm_set_ops(&omap_pm_ops);
	pm_idle = omap2_pm_idle;

	l = subsys_create_file(&power_subsys, &sleep_while_idle_attr);
	if (l)
		printk(KERN_ERR "subsys_create_file failed: %d\n", l);

	return 0;
}

late_initcall(omap2_pm_init);
