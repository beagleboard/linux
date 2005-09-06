/*
 * drivers/char/omap16xx-rng.c
 *
 * Copyright (C) 2005 Nokia Corporation
 * Author: Juha Yrjölä <juha.yrjola@nokia.com>
 *
 * OMAP16xx Random Number Generator driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/random.h>
#include <asm/io.h>

#define RNG_BASE	0xfffe5000
#define RNG_OUT_REG	0x00
#define RNG_STAT_REG	0x04
#define RNG_REV_REG	0x3c
#define RNG_MASK_REG	0x40

#define ENTROPY_WORD_COUNT 128

static u32 rng_base = io_p2v(RNG_BASE);

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
	if (!cpu_is_omap16xx())
		return -ENODEV;

	printk("OMAP16xx Random Number Generator ver. %02x\n",
	rng_read_reg(RNG_REV_REG));
	rng_write_reg(RNG_MASK_REG, 0x00000001);
	rng_feed_entropy(ENTROPY_WORD_COUNT);
	rng_write_reg(RNG_MASK_REG, 0x00000000);
	printk("%d words of entropy generated\n", ENTROPY_WORD_COUNT);

	return 0;
}

static void __exit rng_cleanup(void)
{
}

MODULE_AUTHOR("Juha Yrjölä");
MODULE_DESCRIPTION("OMAP16xx H/W Random Number Generator (RNG) driver");
MODULE_LICENSE("GPL");

module_init(rng_init);
module_exit(rng_cleanup);
