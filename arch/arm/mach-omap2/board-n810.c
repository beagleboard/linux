/*
 * linux/arch/arm/mach-omap2/board-n810.c
 *
 * Copyright (C) 2007 Nokia
 * Author: Lauri Leukkunen <lauri.leukkunen@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/i2c/lm8323.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/board.h>
#include <mach/common.h>

#include "board-n800.h"

static void __init nokia_n810_init(void)
{
	nokia_n800_common_init();
}

MACHINE_START(NOKIA_N810, "Nokia N810")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= nokia_n800_map_io,
	.init_irq	= nokia_n800_init_irq,
	.init_machine	= nokia_n810_init,
	.timer		= &omap_timer,
MACHINE_END

MACHINE_START(NOKIA_N810_WIMAX, "Nokia N810 WiMAX")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= nokia_n800_map_io,
	.init_irq	= nokia_n800_init_irq,
	.init_machine	= nokia_n810_init,
	.timer		= &omap_timer,
MACHINE_END
