/*
 *  arch/arm/plat-omap/include/mach/bci.h
 *
 *  Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef ASMARM_ARCH_BCI_H
#define ASMARM_ARCH_BCI_H
struct twl4030_bci_platform_data {
	int *battery_tmp_tbl;
	unsigned int tblsize;
};
#endif

