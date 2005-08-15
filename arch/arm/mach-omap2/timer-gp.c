/*
 * linux/arch/arm/mach-omap/omap2/timer-mpu.c
 *
 * OMAP2 MPU timer support.
 *
 * Copyright (C) 2005 Nokia Corporation
 * Author: Paul Mundt <paul.mundt@nokia.com>
 *         Juha Yrjölä <juha.yrjola@nokia.com>
 *
 * Some parts based off of TI's 24xx code:
 *
 *   Copyright (C) 2004 Texas Instruments, Inc.
 *
 * Roughly modelled after the OMAP1 MPU timer code.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file "COPYING" in the main directory of this archive
 * for more details.
 */
#include <linux/init.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <asm/mach/time.h>
#include <asm/delay.h>
#include <asm/io.h>

#define OMAP2_GP_TIMER1_BASE	0x48028000
#define OMAP2_GP_TIMER2_BASE	0x4802a000
#define OMAP2_GP_TIMER3_BASE	0x48078000
#define OMAP2_GP_TIMER4_BASE	0x4807a000

#define GP_TIMER_TIDR		0x00
#define GP_TIMER_TISR		0x18
#define GP_TIMER_TIER		0x1c
#define GP_TIMER_TCLR		0x24
#define GP_TIMER_TCRR		0x28
#define GP_TIMER_TLDR		0x2c
#define GP_TIMER_TSICR		0x40

#define OS_TIMER_NR		1  /* GP timer 2 */

static unsigned long timer_base[] = {
	IO_ADDRESS(OMAP2_GP_TIMER1_BASE),
	IO_ADDRESS(OMAP2_GP_TIMER2_BASE),
	IO_ADDRESS(OMAP2_GP_TIMER3_BASE),
	IO_ADDRESS(OMAP2_GP_TIMER4_BASE),
};

static inline unsigned int timer_read_reg(int nr, unsigned int reg)
{
	return __raw_readl(timer_base[nr] + reg);
}

static inline void timer_write_reg(int nr, unsigned int reg, unsigned int val)
{
	__raw_writel(val, timer_base[nr] + reg);
}

static inline void omap2_gp_timer_start(int nr, unsigned long load_val)
{
	unsigned int tmp;

	tmp = 0xffffffff - load_val;

	timer_write_reg(nr, GP_TIMER_TLDR, tmp);
	timer_write_reg(nr, GP_TIMER_TCRR, tmp);
	timer_write_reg(nr, GP_TIMER_TIER, 1 << 1);
	timer_write_reg(nr, GP_TIMER_TCLR, (1 << 5) | (1 << 1) | 1);
}

static irqreturn_t omap2_gp_timer_interrupt(int irq, void *dev_id,
					    struct pt_regs *regs)
{
	write_seqlock(&xtime_lock);

	timer_write_reg(OS_TIMER_NR, GP_TIMER_TISR, 1 << 1);
	timer_tick(regs);

	write_sequnlock(&xtime_lock);

	return IRQ_HANDLED;
}

static struct irqaction omap2_gp_timer_irq = {
	.name		= "gp timer",
	.flags		= SA_INTERRUPT,
	.handler	= omap2_gp_timer_interrupt,
};

#define MPU_TIMER_TICK_PERIOD	(192000 - 1)

static void __init omap2_gp_timer_init(void)
{
	u32 l;

	l = timer_read_reg(OS_TIMER_NR, GP_TIMER_TIDR);
	printk(KERN_INFO "OMAP2 GP timer (HW version %d.%d)\n",
	       (l >> 4) & 0x0f, l & 0x0f);

	setup_irq(38, &omap2_gp_timer_irq);

	omap2_gp_timer_start(OS_TIMER_NR, MPU_TIMER_TICK_PERIOD);
}

struct sys_timer omap_timer = {
	.init	= omap2_gp_timer_init,
};

