/*
 * AM33XX clock function prototypes and macros.
 *
 * Copyright (C) 2011 Texas Instruments, Inc. - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __ARCH_ARM_MACH_OMAP2_CLOCK33XX_H
#define __ARCH_ARM_MACH_OMAP2_CLOCK33XX_H

#define AM33XX_MAX_DPLL_MULT  2047
#define AM33XX_MAX_DPLL_DIV   128


int am33xx_clk_init(void);

/* TRM ERRATA: Timer 3 & 6 default parent (TCLKIN) may not be always
     physically present, in such a case HWMOD enabling of
     clock would be failure with default parent. And timer
     probe thinks clock is already enabled, this leads to
     crash upon accessing timer 3 & 6 registers in probe.
     Fix by setting parent of both these timers to master
     oscillator clock.
 */
static inline void am33xx_init_timer_parent(struct clk *clk)
{
	omap2_clksel_set_parent(clk, clk->parent);
}
#endif
