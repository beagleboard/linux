#ifndef __ARCH_ARM_MACH_OMAP2_SDRC_H
#define __ARCH_ARM_MACH_OMAP2_SDRC_H

/*
 * OMAP2 SDRC register definitions
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Copyright (C) 2007 Nokia Corporation
 *
 * Written by Paul Walmsley
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <asm/arch/io.h>


#define OMAP_SDRC_REGADDR(reg)	(void __iomem *)IO_ADDRESS(OMAP2_SDRC_BASE + reg)

/* SDRC register offsets - read/write with sdrc_{read,write}_reg() */

#define SDRC_SYSCONFIG		0x010
#define SDRC_DLLA_CTRL		0x060
#define SDRC_DLLA_STATUS	0x064
#define SDRC_DLLB_CTRL		0x068
#define SDRC_DLLB_STATUS	0x06C
#define SDRC_POWER		0x070
#define SDRC_MR_0		0x084


/* SDRC global register get/set */

static void __attribute__((unused)) sdrc_write_reg(u32 val, u16 reg)
{
	pr_debug("sdrc_write_reg: writing 0x%0x to 0x%0x\n", val,
		 (u32)OMAP_SDRC_REGADDR(reg));

	__raw_writel(val, OMAP_SDRC_REGADDR(reg));
}

static u32 __attribute__((unused)) sdrc_read_reg(u16 reg)
{
	return __raw_readl(OMAP_SDRC_REGADDR(reg));
}

#endif
