/*
 * linux/arch/arm/mach-omap2/board-rx51-flash.c
 *
 * Copyright (C) 2008 Nokia
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>

#include "mmc-twl4030.h"

extern void __init n800_flash_init(void);

static struct twl4030_hsmmc_info mmc[] __initdata = {
	{
		.name		= "external",
		.mmc		= 1,
		.wires		= 4,
		.cover_only	= true,
		.gpio_cd	= 160,
		.gpio_wp	= -EINVAL,
	},
	{
		.name		= "internal",
		.mmc		= 2,
		.wires		= 8,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
	},
	{}	/* Terminator */
};

void __init rx51_flash_init(void)
{
	n800_flash_init();
	twl4030_mmc_init(mmc);
}

