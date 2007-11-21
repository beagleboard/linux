#ifndef __ARCH_ARM_MACH_OMAP2_CONTROL_H
#define __ARCH_ARM_MACH_OMAP2_CONTROL_H

/*
 * OMAP2/3 System Control Module register definitions
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
#undef DEBUG

#include <linux/kernel.h>
#include <asm/arch/control.h>

extern unsigned long omap2_ctrl_base;

#define OMAP_CTRL_REGADDR(reg)	(void __iomem *)IO_ADDRESS(omap2_ctrl_base + reg)


/* Control global register get/set */

static void __attribute__((unused)) ctrl_write_reg(u32 val, u16 reg)
{
	pr_debug("ctrl_write_reg: writing 0x%0x to 0x%0x\n", val,
		 (u32)OMAP_CTRL_REGADDR(reg));

	__raw_writel(val, OMAP_CTRL_REGADDR(reg));
}

static u32 __attribute__((unused)) ctrl_read_reg(u16 reg)
{
	return __raw_readl(OMAP_CTRL_REGADDR(reg));
}

#endif  /* __ARCH_ARM_MACH_OMAP2_CONTROL_H */
