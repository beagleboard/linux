/*
 * linux/arch/arm/mach-omap2/io.c
 *
 * OMAP2 I/O mapping code
 *
 * Copyright (C) 2005 Nokia Corporation
 * Author: Juha Yrjölä <juha.yrjola@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

#include <asm/mach/map.h>
#include <asm/io.h>
#include <asm/arch/mux.h>

extern void omap_sram_init(void);

/*
 * The machine specific code may provide the extra mapping besides the
 * default mapping provided here.
 */
static struct map_desc omap2_io_desc[] __initdata = {
	{ L3_VIRT,	L3_PHYS,	L3_SIZE,	MT_DEVICE },
	{ L4_VIRT,	L4_PHYS,	L4_SIZE,	MT_DEVICE },
};

void __init omap_map_common_io(void)
{
	iotable_init(omap2_io_desc, ARRAY_SIZE(omap2_io_desc));
	omap_sram_init();
	omap2_mux_init();
}
