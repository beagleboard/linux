#ifndef __ASM_ARCH_CONTROL_H
#define __ASM_ARCH_CONTROL_H

/*
 * include/asm-arm/arch-omap/control.h
 *
 * OMAP2/3 System Control Module definitions
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Copyright (C) 2007 Nokia Corporation
 *
 * Written by Paul Walmsley
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation.
 */

#include <asm/arch/io.h>

#define OMAP242X_CTRL_REGADDR(reg)	(void __iomem *)IO_ADDRESS(OMAP242X_CTRL_BASE + reg)
#define OMAP243X_CTRL_REGADDR(reg)	(void __iomem *)IO_ADDRESS(OMAP243X_CTRL_BASE + reg)
#define OMAP343X_CTRL_REGADDR(reg)	(void __iomem *)IO_ADDRESS(OMAP343X_CTRL_BASE + reg)

/* Control submodule offsets */

#define CONTROL_INTERFACE		0x000
#define CONTROL_PADCONFS		0x030
#define CONTROL_GENERAL			0x270
#define CONTROL_MEM_WKUP		0x600
#define CONTROL_PADCONFS_WKUP		0xa00
#define CONTROL_GENERAL_WKUP		0xa60

/* Control register offsets - read/write with ctrl_{read,write}_reg() */

#define CONTROL_SYSCONFIG		(CONTROL_INTERFACE + 0x10)

#define CONTROL_DEVCONF0		(CONTROL_GENERAL + 0x04)
#define CONTROL_DEVCONF1		(CONTROL_GENERAL + 0x68) /* > 242x */
#define CONTROL_STATUS			(CONTROL_GENERAL + 0x80)


/*
 * Control module register bit defines - these should eventually go into
 * their own regbits file
 */
/* CONTROL_DEVCONF0 bits */
#define OMAP2_MCBSP2_CLKS_MASK		    (1 << 6)
#define OMAP2_MCBSP1_CLKS_MASK		    (1 << 2)

/* CONTROL_DEVCONF1 bits */
#define OMAP2_MCBSP5_CLKS_MASK		    (1 << 4)
#define OMAP2_MCBSP4_CLKS_MASK		    (1 << 2)
#define OMAP2_MCBSP3_CLKS_MASK		    (1 << 0)


#endif /* __ASM_ARCH_CONTROL_H */

