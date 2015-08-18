/*
 * linux/arch/arm/mach-omap2/common.c
 *
 * Code common to all OMAP2+ machines.
 *
 * Copyright (C) 2009 Texas Instruments
 * Copyright (C) 2010 Nokia Corporation
 * Tony Lindgren <tony@atomide.com>
 * Added OMAP4 support - Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_data/dsp-omap.h>
#include <asm/memblock.h>
#include <asm/mach/map.h>

#include "common.h"
#include "omap-secure.h"

#define AM33XX_DRAM_SYNC_VA 0xfe600000

/*
 * Stub function for OMAP2 so that common files
 * continue to build when custom builds are used
 */
int __weak omap_secure_ram_reserve_memblock(void)
{
	return 0;
}

void __init omap_reserve(void)
{
	omap_dsp_reserve_sdram_memblock();
	omap_secure_ram_reserve_memblock();
	omap_barrier_reserve_memblock();
}

static phys_addr_t am33xx_paddr;
static u32 am33xx_size;

/* Steal one page physical memory for uncached read DeepSleep */
void __init am33xx_reserve(void)
{
	am33xx_size = ALIGN(PAGE_SIZE, SZ_1M);
	am33xx_paddr = arm_memblock_steal(am33xx_size, SZ_1M);

	omap_reserve();
}

void __iomem *am33xx_dram_sync;

void __init am33xx_dram_sync_init(void)
{
	struct map_desc dram_io_desc[1];

	dram_io_desc[0].virtual = AM33XX_DRAM_SYNC_VA;
	dram_io_desc[0].pfn = __phys_to_pfn(am33xx_paddr);
	dram_io_desc[0].length = am33xx_size;
	dram_io_desc[0].type = MT_MEMORY_RW_SO;

	iotable_init(dram_io_desc, ARRAY_SIZE(dram_io_desc));

	am33xx_dram_sync = (void __iomem *)dram_io_desc[0].virtual;
}
