/*
 * linux/arch/arm/mach-omap2/board-n800.h
 *
 * Copyright (C) 2005-2007 Nokia Corporation
 * Author: Lauri Leukkunen <lauri.leukkunen@nokia.com>
 *
 * Modified from mach-omap2/board-n800.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ARCH_ARM_MACH_OMAP2_BOARD_N800_H
#define __ARCH_ARM_MACH_OMAP2_BOARD_N800_H

void __init nokia_n800_common_init(void);
void __init nokia_n800_map_io(void);
void __init nokia_n800_init_irq(void);

extern const struct tcm825x_platform_data n800_tcm825x_platform_data;

#endif
