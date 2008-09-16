/*
 * linux/arch/arm/mach-omap2/pm_debug.c
 *
 * OMAP Power Management debug routines
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
 * Jouni Hogander
 *
 * Based on pm.c for omap2
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>

#include <mach/clock.h>
#include <mach/board.h>

#include "prm.h"
#include "cm.h"
#include "pm.h"

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

u32 omap2_read_32k_sync_counter(void)
{
        return omap_readl(OMAP2_32KSYNCT_BASE + 0x0010);
}

void serial_console_fclk_mask(u32 *f1, u32 *f2)
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

void serial_console_sleep(int enable)
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
			if (l & OMAP24XX_ST_UART1_MASK)
				serial_wakeup = 1;
			break;
		case 2:
			l = prm_read_mod_reg(CORE_MOD, PM_WKST1);
			if (l & OMAP24XX_ST_UART2_MASK)
				serial_wakeup = 1;
			break;
		case 3:
			l = prm_read_mod_reg(CORE_MOD, OMAP24XX_PM_WKST2);
			if (l & OMAP24XX_ST_UART3_MASK)
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

void pm_init_serial_console(void)
{
	const struct omap_serial_console_config *conf;
	char name[16];

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
		prm_set_mod_reg_bits(OMAP24XX_ST_UART1_MASK, CORE_MOD,
				     PM_WKEN1);
		break;
	case 2:
		prm_set_mod_reg_bits(OMAP24XX_ST_UART2_MASK, CORE_MOD,
				     PM_WKEN1);
		break;
	case 3:
		prm_set_mod_reg_bits(OMAP24XX_ST_UART3_MASK, CORE_MOD,
				     OMAP24XX_PM_WKEN2);
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
	regs[reg_count++].val = __raw_readl(reg)
#define DUMP_CM_REG(reg) \
	regs[reg_count].name = #reg; \
	regs[reg_count++].val = __raw_readl(reg)
#define DUMP_INTC_REG(reg, off) \
	regs[reg_count].name = #reg; \
	regs[reg_count++].val = __raw_readl(IO_ADDRESS(0x480fe000 + (off)))

void omap2_pm_dump(int mode, int resume, unsigned int us)
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
		DUMP_PRM_MOD_REG(OCP_MOD, OMAP2_PRM_IRQENABLE_MPU_OFFSET);
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
		if (cpu_is_omap24xx()) {
			DUMP_CM_MOD_REG(CORE_MOD, OMAP24XX_CM_FCLKEN2);
			DUMP_PRM_MOD_REG(OMAP24XX_GR_MOD,
					OMAP24XX_PRCM_CLKEMUL_CTRL_OFFSET);
			DUMP_PRM_MOD_REG(OMAP24XX_GR_MOD,
					OMAP24XX_PRCM_CLKSRC_CTRL_OFFSET);
		}
		DUMP_CM_MOD_REG(WKUP_MOD, CM_FCLKEN);
		DUMP_CM_MOD_REG(CORE_MOD, CM_ICLKEN1);
		DUMP_CM_MOD_REG(CORE_MOD, CM_ICLKEN2);
		DUMP_CM_MOD_REG(WKUP_MOD, CM_ICLKEN);
		DUMP_CM_MOD_REG(PLL_MOD, CM_CLKEN);
		DUMP_CM_MOD_REG(PLL_MOD, CM_AUTOIDLE);
		DUMP_PRM_MOD_REG(CORE_MOD, PM_PWSTST);
#endif
#if 0
		/* DSP */
		if (cpu_is_omap24xx()) {
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
		}
#endif
	} else {
		DUMP_PRM_MOD_REG(CORE_MOD, PM_WKST1);
		if (cpu_is_omap24xx())
			DUMP_PRM_MOD_REG(CORE_MOD, OMAP24XX_PM_WKST2);
		DUMP_PRM_MOD_REG(WKUP_MOD, PM_WKST);
		DUMP_PRM_MOD_REG(OCP_MOD, OMAP2_PRM_IRQSTATUS_MPU_OFFSET);
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
		printk("--- Woke up (slept for %u.%03u ms)\n",
			us / 1000, us % 1000);

	for (i = 0; i < reg_count; i++)
		printk("%-20s: 0x%08x\n", regs[i].name, regs[i].val);
}

#endif
