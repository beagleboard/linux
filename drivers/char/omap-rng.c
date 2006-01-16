/*
 * drivers/char/omap-rng.c
 *
 * Copyright (C) 2005 Nokia Corporation
 * Author: Juha Yrjölä <juha.yrjola@nokia.com>
 *
 * OMAP16xx and OMAP24xx Random Number Generator driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/random.h>
#include <linux/err.h>
#include <linux/clk.h>

#include <asm/io.h>

#if defined (CONFIG_ARCH_OMAP16XX)
#define RNG_BASE		0xfffe5000
#endif
#if defined (CONFIG_ARCH_OMAP24XX)
#define RNG_BASE		0x480A0000
#endif

#define RNG_OUT_REG		0x00		/* Output register */
#define RNG_STAT_REG		0x04		/* Status register
							[0] = STAT_BUSY */
#define RNG_ALARM_REG		0x24		/* Alarm register
							[7:0] = ALARM_COUNTER */
#define RNG_CONFIG_REG		0x28		/* Configuration register
							[11:6] = RESET_COUNT
							[5:3]  = RING2_DELAY 
							[2:0]  = RING1_DELAY */
#define RNG_REV_REG		0x3c		/* Revision register
							[7:0] = REV_NB */
#define RNG_MASK_REG		0x40		/* Mask and reset register
							[2] = IT_EN
							[1] = SOFTRESET
							[0] = AUTOIDLE */
#define RNG_SYSSTATUS		0x44		/* System status
							[0] = RESETDONE */

#define ENTROPY_WORD_COUNT	128

static u32 rng_base = io_p2v(RNG_BASE);

static struct clk *rng_ick = NULL;

static u32 rng_read_reg(int reg)
{
	return __raw_readl(rng_base + reg);
}

static void rng_write_reg(int reg, u32 val)
{
	__raw_writel(val, rng_base + reg);
}

static void rng_feed_entropy(int count)
{
	u32 l;

	while (count--) {
		while (rng_read_reg(RNG_STAT_REG));
		l = rng_read_reg(RNG_OUT_REG);
		add_input_randomness(0, 0, l);
	}
}

static int __init rng_init(void)
{
	if (!cpu_is_omap16xx() && !cpu_is_omap24xx())
		return -ENODEV;

	if (cpu_is_omap24xx()) {
		rng_ick = clk_get(NULL, "rng_ick");
		if (IS_ERR(rng_ick)) {
			printk(KERN_ERR "omap-rng.c: Could not get rng_ick\n");
			return PTR_ERR(rng_ick);
		}
		clk_enable(rng_ick);
	}

	printk("OMAP Random Number Generator ver. %02x\n",
	rng_read_reg(RNG_REV_REG));
	rng_write_reg(RNG_MASK_REG, 0x00000001);
	rng_feed_entropy(ENTROPY_WORD_COUNT);
	rng_write_reg(RNG_MASK_REG, 0x00000000);
	printk("%d words of entropy generated\n", ENTROPY_WORD_COUNT);

	return 0;
}
late_initcall(rng_init);
