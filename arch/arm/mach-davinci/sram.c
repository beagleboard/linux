/*
 * mach-davinci/sram.c - DaVinci simple SRAM allocator
 *
 * Copyright (C) 2009 David Brownell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#include <linux/module.h>
#include <linux/init.h>

#include <mach/common.h>
#include <mach/sram.h>

struct gen_pool *davinci_gen_pool;
EXPORT_SYMBOL_GPL(davinci_gen_pool);

/*
 * REVISIT This supports CPU and DMA access to/from SRAM, but it
 * doesn't (yet?) support some other notable uses of SRAM:  as TCM
 * for data and/or instructions; and holding code needed to enter
 * and exit suspend states (while DRAM can't be used).
 */
static int __init sram_init(void)
{
	unsigned len = davinci_soc_info.sram_len;

	if (!len)
		return 0;

	len = min_t(unsigned, len, SRAM_SIZE);
	davinci_gen_pool = gen_pool_create(ilog2(SRAM_GRANULARITY), -1);

	if (!davinci_gen_pool)
		return -ENOMEM;

	WARN_ON(gen_pool_add_virt(davinci_gen_pool, SRAM_VIRT,
				  davinci_soc_info.sram_phys, len, -1));

	return 0;
}
core_initcall(sram_init);
