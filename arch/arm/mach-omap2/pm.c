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
#include <linux/pm.h>
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

#define PRCM_REVISION		0x000
#define PRCM_SYSCONFIG		0x010
#define PRCM_IRQSTATUS_MPU	0x018
#define PRCM_IRQENABLE_MPU	0x01c
#define PRCM_VOLTCTRL		0x050
#define		AUTO_EXTVOLT	(1 << 15)
#define		FORCE_EXTVOLT	(1 << 14)
#define		SETOFF_LEVEL(x)	(((x) & 0x3) << 12)
#define		MEMRETCTRL	(1 << 8)
#define		SETRET_LEVEL(x)	(((x) & 0x3) << 6)
#define		VOLT_LEVEL(x)	(((x) & 0x3) << 0)
#define PRCM_CLKSRC_CTRL	0x060
#define PRCM_CLKOUT_CTRL	0x070
#define PRCM_CLKEMUL_CTRL	0x078
#define PRCM_CLKCFG_CTRL	0x080
#define PRCM_VOLTSETUP		0x090
#define PRCM_CLKSSETUP		0x094


#define CM_CLKSEL_MPU		0x140
#define CM_CLKSTCTRL_MPU	0x148
#define		AUTOSTAT_MPU	(1 << 0)
#define PM_WKDEP_MPU		0x1c8
#define 	EN_WKUP		(1 << 4)
#define 	EN_GFX		(1 << 3)
#define 	EN_DSP		(1 << 2)
#define 	EN_MPU		(1 << 1)
#define 	EN_CORE		(1 << 0)
#define PM_PWSTCTRL_MPU		0x1e0
#define PM_PWSTST_MPU		0x1e4


#define CM_FCLKEN1_CORE		0x200
#define CM_FCLKEN2_CORE		0x204
#define CM_ICLKEN1_CORE		0x210
#define CM_ICLKEN2_CORE		0x214
#define CM_ICLKEN4_CORE		0x21c
#define CM_IDLEST1_CORE		0x220
#define CM_IDLEST2_CORE		0x224
#define CM_AUTOIDLE1_CORE	0x230
#define CM_AUTOIDLE2_CORE	0x234
#define CM_AUTOIDLE3_CORE	0x238
#define CM_AUTOIDLE4_CORE	0x23c
#define CM_CLKSEL1_CORE		0x240
#define CM_CLKSEL2_CORE		0x244
#define CM_CLKSTCTRL_CORE	0x248
#define		AUTOSTAT_DSS	(1 << 2)
#define		AUTOSTAT_L4	(1 << 1)
#define		AUTOSTAT_L3	(1 << 0)
#define PM_WKEN1_CORE		0x2a0
#define PM_WKEN2_CORE		0x2a4
#define PM_WKST1_CORE		0x2b0
#define PM_WKST2_CORE		0x2b4
#define PM_WKDEP_CORE		0x2c8
#define PM_PWSTCTRL_CORE	0x2e0
#define PM_PWSTST_CORE		0x2e4


#define CM_CLKSTCTRL_GFX	0x348
#define		AUTOSTAT_GFX	(1 << 0)
#define PM_WKDEP_GFX    	0x3c8
#define PM_PWSTCTRL_GFX		0x3e0


#define CM_FCLKEN_WKUP		0x400
#define CM_ICLKEN_WKUP		0x410
#define CM_AUTOIDLE_WKUP	0x430
#define PM_WKEN_WKUP		0x4a0
#define 	EN_GPIOS	(1 << 2)
#define 	EN_GPT1		(1 << 0)
#define PM_WKST_WKUP		0x4b0


#define CM_CLKEN_PLL		0x500
#define CM_IDLEST_CKGEN		0x520
#define CM_AUTOIDLE_PLL		0x530
#define CM_CLKSEL1_PLL		0x540
#define CM_CLKSEL2_PLL		0x544


#define CM_FCLKEN_DSP		0x800
#define CM_ICLKEN_DSP		0x810
#define CM_IDLEST_DSP		0x820
#define CM_AUTOIDLE_DSP		0x830
#define CM_CLKSEL_DSP		0x840
#define CM_CLKSTCTRL_DSP	0x848
#define		AUTOSTAT_IVA	(1 << 8)
#define		AUTOSTAT_DSP	(1 << 0)
#define RM_RSTCTRL_DSP		0x850
#define RM_RSTST_DSP		0x858
#define PM_WKDEP_DSP		0x8c8
#define PM_PWSTCTRL_DSP		0x8e0
#define PM_PWSTST_DSP		0x8e4

static void (*omap2_sram_idle)(void);
static void (*omap2_sram_suspend)(int dllctrl);
static void (*saved_idle)(void);

static u32 prcm_base = IO_ADDRESS(OMAP24XX_PRCM_BASE);

static inline void prcm_write_reg(int idx, u32 val)
{
	__raw_writel(val, prcm_base + idx);
}

static inline u32 prcm_read_reg(int idx)
{
	return __raw_readl(prcm_base + idx);
}

static u32 omap2_read_32k_sync_counter(void)
{
        return omap_readl(0x48004010);
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
			l = prcm_read_reg(PM_WKST1_CORE);
			if (l & (1 << 21))
				serial_wakeup = 1;
			break;
		case 2:
			l = prcm_read_reg(PM_WKST1_CORE);
			if (l & (1 << 22))
				serial_wakeup = 1;
			break;
		case 3:
			l = prcm_read_reg(PM_WKST2_CORE);
			if (l & (1 << 2))
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
		l = prcm_read_reg(PM_WKEN1_CORE);
		l |= 1 << 21;
		prcm_write_reg(PM_WKEN1_CORE, l);
		break;
	case 2:
		l = prcm_read_reg(PM_WKEN1_CORE);
		l |= 1 << 22;
		prcm_write_reg(PM_WKEN1_CORE, l);
		break;
	case 3:
		l = prcm_read_reg(PM_WKEN2_CORE);
		l |= 1 << 2;
		prcm_write_reg(PM_WKEN2_CORE, l);
		break;
	}
}

#define DUMP_REG(reg) \
	regs[reg_count].name = #reg; \
	regs[reg_count++].val = prcm_read_reg(reg)
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
		DUMP_REG(PRCM_IRQENABLE_MPU);
		DUMP_REG(CM_CLKSTCTRL_MPU);
		DUMP_REG(PM_PWSTCTRL_MPU);
		DUMP_REG(PM_PWSTST_MPU);
		DUMP_REG(PM_WKDEP_MPU);
#endif
#if 0
		/* INTC */
		DUMP_INTC_REG(INTC_MIR0, 0x0084);
		DUMP_INTC_REG(INTC_MIR1, 0x00a4);
		DUMP_INTC_REG(INTC_MIR2, 0x00c4);
#endif
#if 0
		DUMP_REG(CM_FCLKEN1_CORE);
		DUMP_REG(CM_FCLKEN2_CORE);
		DUMP_REG(CM_FCLKEN_WKUP);
		DUMP_REG(CM_ICLKEN1_CORE);
		DUMP_REG(CM_ICLKEN2_CORE);
		DUMP_REG(CM_ICLKEN_WKUP);
		DUMP_REG(CM_CLKEN_PLL);
		DUMP_REG(PRCM_CLKEMUL_CTRL);
		DUMP_REG(CM_AUTOIDLE_PLL);
		DUMP_REG(PM_PWSTST_CORE);
		DUMP_REG(PRCM_CLKSRC_CTRL);
#endif
#if 0
		/* DSP */
		DUMP_REG(CM_FCLKEN_DSP);
		DUMP_REG(CM_ICLKEN_DSP);
		DUMP_REG(CM_IDLEST_DSP);
		DUMP_REG(CM_AUTOIDLE_DSP);
		DUMP_REG(CM_CLKSEL_DSP);
		DUMP_REG(CM_CLKSTCTRL_DSP);
		DUMP_REG(RM_RSTCTRL_DSP);
		DUMP_REG(RM_RSTST_DSP);
		DUMP_REG(PM_PWSTCTRL_DSP);
		DUMP_REG(PM_PWSTST_DSP);
#endif
	} else {
		DUMP_REG(PM_WKST1_CORE);
		DUMP_REG(PM_WKST2_CORE);
		DUMP_REG(PM_WKST_WKUP);
		DUMP_REG(PRCM_IRQSTATUS_MPU);
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
		       jiffies_to_msecs(next_timer_interrupt() - jiffies));
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
#define SDRC_DLLA_CTRL		__REG32(0x68009060)

static int omap2_fclks_active(void)
{
	u32 f1, f2;

	f1 = prcm_read_reg(CM_FCLKEN1_CORE);
	f2 = prcm_read_reg(CM_FCLKEN2_CORE);
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
	prcm_write_reg(PM_WKST1_CORE, 0xffffffff);
	prcm_write_reg(PM_WKST2_CORE, 0xffffffff);
	prcm_write_reg(PM_WKST_WKUP, 0xffffffff);

	/* Try to enter retention */
	prcm_write_reg(PM_PWSTCTRL_MPU, (0x01 << 0) | (1 << 2));

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
	omap2_sram_suspend(SDRC_DLLA_CTRL);
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

	l = prcm_read_reg(CM_FCLKEN1_CORE);
	return l & ((1 << 19) | (1 << 20));
}

static int sti_console_enabled;

static int omap2_allow_mpu_retention(void)
{
	u32 l;

	if (atomic_read(&sleep_block))
		return 0;

	/* Check for UART2, UART1, McSPI2, McSPI1 and DSS1. */
	l = prcm_read_reg(CM_FCLKEN1_CORE);
	if (l & 0x04660001)
		return 0;
	/* Check for UART3. */
	l = prcm_read_reg(CM_FCLKEN2_CORE);
	if (l & (1 << 2))
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
		prcm_write_reg(PM_WKST1_CORE, 0xffffffff);
		prcm_write_reg(PM_WKST2_CORE, 0xffffffff);
		prcm_write_reg(PM_WKST_WKUP, 0xffffffff);

		/* Try to enter MPU retention */
		prcm_write_reg(PM_PWSTCTRL_MPU, (0x01 << 0) | (1 << 2));
	} else {
		/* Block MPU retention */
		prcm_write_reg(PM_PWSTCTRL_MPU, 1 << 2);
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
	case PM_SUSPEND_DISK:
		return -ENOTSUPP;
	default:
		return -EINVAL;
	}

	return error;
}

static int omap2_pm_suspend(void)
{
	u32 wken_wkup, mir1;

	wken_wkup = prcm_read_reg(PM_WKEN_WKUP);
	prcm_write_reg(PM_WKEN_WKUP, wken_wkup & ~EN_GPT1);

	/* Mask GPT1 */
	mir1 = omap_readl(0x480fe0a4);
	omap_writel(1 << 5, 0x480fe0ac);

	omap2_enter_full_retention();

	omap_writel(mir1, 0x480fe0a4);
	prcm_write_reg(PM_WKEN_WKUP, wken_wkup);

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
	case PM_SUSPEND_DISK:
		ret = -ENOTSUPP;
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
	prcm_write_reg(PRCM_SYSCONFIG, 1 << 0);

	/* Set all domain wakeup dependencies */
	prcm_write_reg(PM_WKDEP_MPU, EN_WKUP);
	prcm_write_reg(PM_WKDEP_DSP, 0);
	prcm_write_reg(PM_WKDEP_GFX, 0);

	l = prcm_read_reg(PM_PWSTCTRL_CORE);
	/* Enable retention for all memory blocks */
	l |= (1 << 3) | (1 << 4) | (1 << 5);
	/* Set power state to RETENTION */
	l &= ~0x03;
	l |= 0x01 << 0;
	prcm_write_reg(PM_PWSTCTRL_CORE, l);

	prcm_write_reg(PM_PWSTCTRL_MPU, (0x01 << 0) | (1 << 2));

	/* Power down DSP and GFX */
	prcm_write_reg(PM_PWSTCTRL_DSP, (1 << 18) | 0x03);
	prcm_write_reg(PM_PWSTCTRL_GFX, (1 << 18) | 0x03);

	/* Enable clock auto control for all domains */
	prcm_write_reg(CM_CLKSTCTRL_MPU, AUTOSTAT_MPU);
	prcm_write_reg(CM_CLKSTCTRL_CORE, AUTOSTAT_DSS | AUTOSTAT_L4 | AUTOSTAT_L3);
	prcm_write_reg(CM_CLKSTCTRL_GFX, AUTOSTAT_GFX);
	prcm_write_reg(CM_CLKSTCTRL_DSP, AUTOSTAT_IVA | AUTOSTAT_DSP);

	/* Enable clock autoidle for all domains */
	prcm_write_reg(CM_AUTOIDLE1_CORE, 0xfffffff9);
	prcm_write_reg(CM_AUTOIDLE2_CORE, 0x07);
	prcm_write_reg(CM_AUTOIDLE3_CORE, 0x07);
	prcm_write_reg(CM_AUTOIDLE4_CORE, 0x1f);

	prcm_write_reg(CM_AUTOIDLE_DSP, 0x02);

	/* Put DPLL and both APLLs into autoidle mode */
	prcm_write_reg(CM_AUTOIDLE_PLL, (0x03 << 0) | (0x03 << 2) | (0x03 << 6));

	prcm_write_reg(CM_AUTOIDLE_WKUP, 0x3f);

	/* REVISIT: Configure number of 32 kHz clock cycles for sys_clk
	 * stabilisation */
	prcm_write_reg(PRCM_CLKSSETUP, 15);

	/* Configure automatic voltage transition */
	prcm_write_reg(PRCM_VOLTSETUP, 2);
	l = AUTO_EXTVOLT | SETOFF_LEVEL(1) | MEMRETCTRL | \
		SETRET_LEVEL(1) | VOLT_LEVEL(0);
	prcm_write_reg(PRCM_VOLTCTRL, l);

	/* Enable wake-up events */
	prcm_write_reg(PM_WKEN_WKUP, EN_GPIOS | EN_GPT1);
}

int __init omap2_pm_init(void)
{
	u32 l;

	printk(KERN_INFO "Power Management for OMAP2 initializing\n");
	l = prcm_read_reg(PRCM_REVISION);
	printk(KERN_INFO "PRCM revision %d.%d\n", (l >> 4) & 0x0f, l & 0x0f);

	osc_ck = clk_get(NULL, "osc_ck");
	if (IS_ERR(osc_ck)) {
		printk(KERN_ERR "could not get osc_ck\n");
		return -ENODEV;
	}

	emul_ck = clk_get(NULL, "emul_ck");
	if (IS_ERR(emul_ck)) {
		printk(KERN_ERR "could not get emul_ck\n");
		clk_put(osc_ck);
		return -ENODEV;
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
