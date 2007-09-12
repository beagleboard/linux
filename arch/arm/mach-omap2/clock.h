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

static void omap2_clksel_recalc(struct clk * clk);
static void omap2_table_mpu_recalc(struct clk *clk);
static int omap2_select_table_rate(struct clk * clk, unsigned long rate);
static long omap2_round_to_table_rate(struct clk * clk, unsigned long rate);
static void omap2_clk_disable(struct clk *clk);
static void omap2_sys_clk_recalc(struct clk * clk);
static void omap2_init_clksel_parent(struct clk *clk);
static u32 omap2_clksel_get_divisor(struct clk *clk);
static u32 omap2_clksel_to_divisor(struct clk *clk, u32 field_val);
static u32 omap2_divisor_to_clksel(struct clk *clk, u32 div);
static void omap2_osc_clk_recalc(struct clk *clk);
static void omap2_sys_clk_recalc(struct clk *clk);
static void omap2_dpll_recalc(struct clk *clk);
static void omap2_fixed_divisor_recalc(struct clk *clk);
static int omap2_clk_fixed_enable(struct clk *clk);
static void omap2_clk_fixed_disable(struct clk *clk);
static long omap2_clksel_round_rate(struct clk *clk, unsigned long target_rate);
static int omap2_clksel_set_rate(struct clk *clk, unsigned long rate);
static int omap2_reprogram_dpll(struct clk *clk, unsigned long rate);
static int omap2_enable_osc_ck(struct clk *clk);
static void omap2_disable_osc_ck(struct clk *clk);

#endif
