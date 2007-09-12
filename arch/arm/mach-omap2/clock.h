/*
 *  linux/arch/arm/mach-omap2/clock.h
 *
 *  Copyright (C) 2005 Texas Instruments Inc.
 *  Richard Woodruff <r-woodruff2@ti.com>
 *  Created for OMAP2.
 *
 *  Copyright (C) 2004 Nokia corporation
 *  Written by Tuukka Tikkanen <tuukka.tikkanen@elektrobit.com>
 *  Based on clocks.h by Tony Lindgren, Gordon McNutt and RidgeRun, Inc
 *
 *  Copyright (C) 2007 Texas Instruments, Inc.
 *  Copyright (C) 2007 Nokia Corporation
 *  Paul Walmsley
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ARCH_ARM_MACH_OMAP2_CLOCK_H
#define __ARCH_ARM_MACH_OMAP2_CLOCK_H

int omap2_clk_enable(struct clk *clk);
void omap2_clk_disable(struct clk *clk);
long omap2_clk_round_rate(struct clk *clk, unsigned long rate);
int omap2_clk_set_rate(struct clk *clk, unsigned long rate);
int omap2_clk_set_parent(struct clk *clk, struct clk *new_parent);

#ifdef CONFIG_OMAP_RESET_CLOCKS
void __init omap2_clk_disable_unused(struct clk *clk);
#else
#define omap2_clk_disable_unused	NULL
#endif

void omap2_clksel_recalc(struct clk *clk);
void omap2_init_clksel_parent(struct clk *clk);
u32 omap2_clksel_get_divisor(struct clk *clk);
u32 omap2_clksel_round_rate_div(struct clk *clk, unsigned long target_rate,
				u32 *new_div);
u32 omap2_clksel_to_divisor(struct clk *clk, u32 field_val);
u32 omap2_divisor_to_clksel(struct clk *clk, u32 div);
void omap2_fixed_divisor_recalc(struct clk *clk);
long omap2_clksel_round_rate(struct clk *clk, unsigned long target_rate);
int omap2_clksel_set_rate(struct clk *clk, unsigned long rate);
u32 omap2_get_dpll_rate(struct clk *clk);
int omap2_wait_clock_ready(void __iomem *reg, u32 cval, const char *name);
u8 mask_to_shift(u32 mask);

extern u8 cpu_mask;

#endif
