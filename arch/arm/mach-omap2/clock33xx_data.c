/*
 * AM33XX Clock data
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/clk.h>
#include <plat/clkdev_omap.h>

#include "control.h"
#include "clock.h"
#include "clock33xx.h"
#include "cm.h"
#include "cm33xx.h"
#include "cm-regbits-33xx.h"
#include "prm.h"

/* Modulemode control */
#define AM33XX_MODULEMODE_HWCTRL	0
#define AM33XX_MODULEMODE_SWCTRL	1

/* Root clocks */
static struct clk clk_32768_ck = {
	.name		= "clk_32768_ck",
	.rate		= 32768,
	.ops		= &clkops_null,
};

/* On-Chip 32KHz RC OSC */
static struct clk clk_rc32k_ck = {
	.name		= "clk_rc32k_ck",
	.rate		= 32000,
	.ops		= &clkops_null,
};

/* Crystal input clks */
static struct clk virt_19_2m_ck = {
	.name		= "virt_19_2m_ck",
	.rate		= 19200000,
	.ops		= &clkops_null,
};

static struct clk virt_24m_ck = {
	.name		= "virt_24m_ck",
	.rate		= 24000000,
	.ops		= &clkops_null,
};

static struct clk virt_25m_ck = {
	.name		= "virt_25m_ck",
	.rate		= 25000000,
	.ops		= &clkops_null,
};

static struct clk virt_26m_ck = {
	.name		= "virt_26m_ck",
	.rate		= 26000000,
	.ops		= &clkops_null,
};

static struct clk tclkin_ck = {
	.name		= "tclkin_ck",
	.rate		= 12000000,
	.ops		= &clkops_null,
};

static const struct clksel_rate div_1_0_rates[] = {
	{ .div = 1, .val = 0, .flags = RATE_IN_AM33XX },
	{ .div = 0 },
};

static const struct clksel_rate div_1_1_rates[] = {
	{ .div = 1, .val = 1, .flags = RATE_IN_AM33XX },
	{ .div = 0 },
};

static const struct clksel_rate div_1_2_rates[] = {
	{ .div = 1, .val = 2, .flags = RATE_IN_AM33XX },
	{ .div = 0 },
};

static const struct clksel_rate div_1_3_rates[] = {
	{ .div = 1, .val = 3, .flags = RATE_IN_AM33XX },
	{ .div = 0 },
};

static const struct clksel_rate div_1_4_rates[] = {
	{ .div = 1, .val = 4, .flags = RATE_IN_AM33XX },
	{ .div = 0 },
};

static const struct clksel_rate div31_1to31_rates[] = {
	{ .div = 1, .val = 1, .flags = RATE_IN_AM33XX },
	{ .div = 2, .val = 2, .flags = RATE_IN_AM33XX },
	{ .div = 3, .val = 3, .flags = RATE_IN_AM33XX },
	{ .div = 4, .val = 4, .flags = RATE_IN_AM33XX },
	{ .div = 5, .val = 5, .flags = RATE_IN_AM33XX },
	{ .div = 6, .val = 6, .flags = RATE_IN_AM33XX },
	{ .div = 7, .val = 7, .flags = RATE_IN_AM33XX },
	{ .div = 8, .val = 8, .flags = RATE_IN_AM33XX },
	{ .div = 9, .val = 9, .flags = RATE_IN_AM33XX },
	{ .div = 10, .val = 10, .flags = RATE_IN_AM33XX },
	{ .div = 11, .val = 11, .flags = RATE_IN_AM33XX },
	{ .div = 12, .val = 12, .flags = RATE_IN_AM33XX },
	{ .div = 13, .val = 13, .flags = RATE_IN_AM33XX },
	{ .div = 14, .val = 14, .flags = RATE_IN_AM33XX },
	{ .div = 15, .val = 15, .flags = RATE_IN_AM33XX },
	{ .div = 16, .val = 16, .flags = RATE_IN_AM33XX },
	{ .div = 17, .val = 17, .flags = RATE_IN_AM33XX },
	{ .div = 18, .val = 18, .flags = RATE_IN_AM33XX },
	{ .div = 19, .val = 19, .flags = RATE_IN_AM33XX },
	{ .div = 20, .val = 20, .flags = RATE_IN_AM33XX },
	{ .div = 21, .val = 21, .flags = RATE_IN_AM33XX },
	{ .div = 22, .val = 22, .flags = RATE_IN_AM33XX },
	{ .div = 23, .val = 23, .flags = RATE_IN_AM33XX },
	{ .div = 24, .val = 24, .flags = RATE_IN_AM33XX },
	{ .div = 25, .val = 25, .flags = RATE_IN_AM33XX },
	{ .div = 26, .val = 26, .flags = RATE_IN_AM33XX },
	{ .div = 27, .val = 27, .flags = RATE_IN_AM33XX },
	{ .div = 28, .val = 28, .flags = RATE_IN_AM33XX },
	{ .div = 29, .val = 29, .flags = RATE_IN_AM33XX },
	{ .div = 30, .val = 30, .flags = RATE_IN_AM33XX },
	{ .div = 31, .val = 31, .flags = RATE_IN_AM33XX },
	{ .div = 0 },
};

/* Oscillator clock */
/* 19.2, 24, 25 or 26 MHz */
static const struct clksel sys_clkin_sel[] = {
	{ .parent = &virt_19_2m_ck, .rates = div_1_0_rates },
	{ .parent = &virt_24m_ck, .rates = div_1_1_rates },
	{ .parent = &virt_25m_ck, .rates = div_1_2_rates },
	{ .parent = &virt_26m_ck, .rates = div_1_3_rates },
	{ .parent = NULL },
};

/* sys_clk_in */
static struct clk sys_clkin_ck = {
	.name		= "sys_clkin_ck",
	.parent		= &virt_24m_ck,
	.init		= &omap2_init_clksel_parent,
	.clksel_reg	= AM33XX_CTRL_REGADDR(0x40),	/* CONTROL_STATUS */
	.clksel_mask	= (0x3 << 22),
	.clksel		= sys_clkin_sel,
	.ops		= &clkops_null,
	.recalc		= &omap2_clksel_recalc,
};

/* DPLL_CORE */
static struct dpll_data dpll_core_dd = {
	.mult_div1_reg	= AM33XX_CM_CLKSEL_DPLL_CORE,
	.clk_bypass	= &sys_clkin_ck,
	.clk_ref	= &sys_clkin_ck,
	.control_reg	= AM33XX_CM_CLKMODE_DPLL_CORE,
	.modes		= (1 << DPLL_LOW_POWER_BYPASS) | (1 << DPLL_LOCKED),
	.idlest_reg	= AM33XX_CM_IDLEST_DPLL_CORE,
	.mult_mask	= AM33XX_DPLL_MULT_MASK,
	.div1_mask	= AM33XX_DPLL_DIV_MASK,
	.enable_mask	= AM33XX_DPLL_EN_MASK,
	.idlest_mask	= AM33XX_ST_DPLL_CLK_MASK,
	.max_multiplier	= AM33XX_MAX_DPLL_MULT,
	.max_divider	= AM33XX_MAX_DPLL_DIV,
	.min_divider	= 1,
};

/* CLKDCOLDO output */
static struct clk dpll_core_ck = {
	.name		= "dpll_core_ck",
	.parent		= &sys_clkin_ck,
	.dpll_data	= &dpll_core_dd,
	.init		= &omap2_init_dpll_parent,
	.ops		= &clkops_omap3_core_dpll_ops,
	.recalc		= &omap3_dpll_recalc,
};

static struct clk dpll_core_x2_ck = {
	.name		= "dpll_core_x2_ck",
	.parent		= &dpll_core_ck,
	.flags		= CLOCK_CLKOUTX2,
	.ops		= &clkops_null,
	.recalc		= &omap3_clkoutx2_recalc,
};


static const struct clksel dpll_core_m4_div[] = {
	{ .parent = &dpll_core_x2_ck, .rates = div31_1to31_rates },
	{ .parent = NULL },
};

static struct clk dpll_core_m4_ck = {
	.name		= "dpll_core_m4_ck",
	.parent		= &dpll_core_x2_ck,
	.clksel		= dpll_core_m4_div,
	.clksel_reg	= AM33XX_CM_DIV_M4_DPLL_CORE,
	.clksel_mask	= AM33XX_HSDIVIDER_CLKOUT1_DIV_MASK,
	.ops		= &clkops_null,
	.recalc		= &omap2_clksel_recalc,
	.round_rate	= &omap2_clksel_round_rate,
	.set_rate	= &omap2_clksel_set_rate,
};

static const struct clksel dpll_core_m5_div[] = {
	{ .parent = &dpll_core_x2_ck, .rates = div31_1to31_rates },
	{ .parent = NULL },
};

static struct clk dpll_core_m5_ck = {
	.name		= "dpll_core_m5_ck",
	.parent		= &dpll_core_x2_ck,
	.clksel		= dpll_core_m5_div,
	.clksel_reg	= AM33XX_CM_DIV_M5_DPLL_CORE,
	.clksel_mask	= AM33XX_HSDIVIDER_CLKOUT2_DIV_MASK,
	.ops		= &clkops_null,
	.recalc		= &omap2_clksel_recalc,
	.round_rate	= &omap2_clksel_round_rate,
	.set_rate	= &omap2_clksel_set_rate,
};

static const struct clksel dpll_core_m6_div[] = {
	{ .parent = &dpll_core_x2_ck, .rates = div31_1to31_rates },
	{ .parent = NULL },
};

static struct clk dpll_core_m6_ck = {
	.name		= "dpll_core_m6_ck",
	.parent		= &dpll_core_x2_ck,
	.clksel		= dpll_core_m6_div,
	.clksel_reg	= AM33XX_CM_DIV_M6_DPLL_CORE,
	.clksel_mask	= AM33XX_HSDIVIDER_CLKOUT3_DIV_MASK,
	.ops		= &clkops_null,
	.recalc		= &omap2_clksel_recalc,
	.round_rate	= &omap2_clksel_round_rate,
	.set_rate	= &omap2_clksel_set_rate,
};

static struct clk sysclk1_ck = {
	.name		= "sysclk1_ck",
	.parent		= &dpll_core_m4_ck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk sysclk2_ck = {
	.name		= "sysclk2_ck",
	.parent		= &dpll_core_m5_ck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk core_clk_out = {
	.name		= "core_clk_out",
	.parent		= &dpll_core_m4_ck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

/* DPLL_MPU */
static struct dpll_data dpll_mpu_dd = {
	.mult_div1_reg	= AM33XX_CM_CLKSEL_DPLL_MPU,
	.clk_bypass	= &sys_clkin_ck,
	.clk_ref	= &sys_clkin_ck,
	.control_reg	= AM33XX_CM_CLKMODE_DPLL_MPU,
	.modes		= (1 << DPLL_LOW_POWER_BYPASS) | (1 << DPLL_LOCKED),
	.idlest_reg	= AM33XX_CM_IDLEST_DPLL_MPU,
	.mult_mask	= AM33XX_DPLL_MULT_MASK,
	.div1_mask	= AM33XX_DPLL_DIV_MASK,
	.enable_mask	= AM33XX_DPLL_EN_MASK,
	.idlest_mask	= AM33XX_ST_DPLL_CLK_MASK,
	.max_multiplier	= AM33XX_MAX_DPLL_MULT,
	.max_divider	= AM33XX_MAX_DPLL_DIV,
	.min_divider	= 1,
};

/* CLKOUT: fdpll/M2 */
static struct clk dpll_mpu_ck = {
	.name		= "dpll_mpu_ck",
	.parent		= &sys_clkin_ck,
	.dpll_data	= &dpll_mpu_dd,
	.init		= &omap2_init_dpll_parent,
	.ops		= &clkops_omap3_noncore_dpll_ops,
	.recalc		= &omap3_dpll_recalc,
	.round_rate	= &omap2_dpll_round_rate,
	.set_rate	= &omap3_noncore_dpll_set_rate,
};

/*
 * TODO: Add clksel here (sys_clkin, CORE_CLKOUTM6, PER_CLKOUTM2
 * and ALT_CLK1/2)
 */
static const struct clksel dpll_mpu_m2_div[] = {
	{ .parent = &dpll_mpu_ck, .rates = div31_1to31_rates },
	{ .parent = NULL },
};

static struct clk dpll_mpu_m2_ck = {
	.name		= "dpll_mpu_m2_ck",
	.parent		= &dpll_mpu_ck,
	.clksel		= dpll_mpu_m2_div,
	.clksel_reg	= AM33XX_CM_DIV_M2_DPLL_MPU,
	.clksel_mask	= AM33XX_DPLL_CLKOUT_DIV_MASK,
	.ops		= &clkops_null,
	.recalc		= &omap2_clksel_recalc,
	.round_rate	= &omap2_clksel_round_rate,
	.set_rate	= &omap2_clksel_set_rate,
};

static struct clk mpu_fck = {
	.name		= "mpu_fck",
	.clkdm_name	= "mpu_clkdm",
	.parent		= &dpll_mpu_m2_ck,
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_MPU_MPU_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.recalc		= &followparent_recalc,
	.flags		= ENABLE_ON_INIT,
};

/* DPLL_DDR */
static struct dpll_data dpll_ddr_dd = {
	.mult_div1_reg	= AM33XX_CM_CLKSEL_DPLL_DDR,
	.clk_bypass	= &sys_clkin_ck,
	.clk_ref	= &sys_clkin_ck,
	.control_reg	= AM33XX_CM_CLKMODE_DPLL_DDR,
	.modes		= (1 << DPLL_LOW_POWER_BYPASS) | (1 << DPLL_LOCKED),
	.idlest_reg	= AM33XX_CM_IDLEST_DPLL_DDR,
	.mult_mask	= AM33XX_DPLL_MULT_MASK,
	.div1_mask	= AM33XX_DPLL_DIV_MASK,
	.enable_mask	= AM33XX_DPLL_EN_MASK,
	.idlest_mask	= AM33XX_ST_DPLL_CLK_MASK,
	.max_multiplier	= AM33XX_MAX_DPLL_MULT,
	.max_divider	= AM33XX_MAX_DPLL_DIV,
	.min_divider	= 1,
};

/* CLKOUT: fdpll/M2 */
static struct clk dpll_ddr_ck = {
	.name		= "dpll_ddr_ck",
	.parent		= &sys_clkin_ck,
	.dpll_data	= &dpll_ddr_dd,
	.init		= &omap2_init_dpll_parent,
	.ops		= &clkops_null,
	.recalc		= &omap3_dpll_recalc,
};

/*
 * TODO: Add clksel here (sys_clkin, CORE_CLKOUTM6, PER_CLKOUTM2
 * and ALT_CLK1/2)
 */
static const struct clksel dpll_ddr_m2_div[] = {
	{ .parent = &dpll_ddr_ck, .rates = div31_1to31_rates },
	{ .parent = NULL },
};

static struct clk dpll_ddr_m2_ck = {
	.name		= "dpll_ddr_m2_ck",
	.parent		= &dpll_ddr_ck,
	.clksel		= dpll_ddr_m2_div,
	.clksel_reg	= AM33XX_CM_DIV_M2_DPLL_DDR,
	.clksel_mask	= AM33XX_DPLL_CLKOUT_DIV_MASK,
	.ops		= &clkops_null,
	.recalc		= &omap2_clksel_recalc,
	.round_rate	= &omap2_clksel_round_rate,
	.set_rate	= &omap2_clksel_set_rate,
};

static struct clk ddr_pll_clk = {
	.name		= "ddr_pll_clk",
	.parent		= &dpll_ddr_m2_ck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk emif_fck = {
	.name		= "emif_fck",
	.clkdm_name	= "l3_clkdm",
	.parent		= &ddr_pll_clk,
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_EMIF_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.fixed_div	= 2,
	.recalc		= &omap_fixed_divisor_recalc,
	.flags		= ENABLE_ON_INIT,
};

/* DPLL_DISP */
static struct dpll_data dpll_disp_dd = {
	.mult_div1_reg	= AM33XX_CM_CLKSEL_DPLL_DISP,
	.clk_bypass	= &sys_clkin_ck,
	.clk_ref	= &sys_clkin_ck,
	.control_reg	= AM33XX_CM_CLKMODE_DPLL_DISP,
	.modes		= (1 << DPLL_LOW_POWER_BYPASS) | (1 << DPLL_LOCKED),
	.idlest_reg	= AM33XX_CM_IDLEST_DPLL_DISP,
	.mult_mask	= AM33XX_DPLL_MULT_MASK,
	.div1_mask	= AM33XX_DPLL_DIV_MASK,
	.enable_mask	= AM33XX_DPLL_EN_MASK,
	.idlest_mask	= AM33XX_ST_DPLL_CLK_MASK,
	.max_multiplier	= AM33XX_MAX_DPLL_MULT,
	.max_divider	= AM33XX_MAX_DPLL_DIV,
	.min_divider	= 1,
};

/* CLKOUT: fdpll/M2 */
static struct clk dpll_disp_ck = {
	.name		= "dpll_disp_ck",
	.parent		= &sys_clkin_ck,
	.dpll_data	= &dpll_disp_dd,
	.init		= &omap2_init_dpll_parent,
	.ops		= &clkops_null,
	.recalc		= &omap3_dpll_recalc,
	.round_rate	= &omap2_dpll_round_rate,
	.set_rate	= &omap3_noncore_dpll_set_rate,
};

/*
 * TODO: Add clksel here (sys_clkin, CORE_CLKOUTM6, PER_CLKOUTM2
 * and ALT_CLK1/2)
 */
static const struct clksel dpll_disp_m2_div[] = {
	{ .parent = &dpll_disp_ck, .rates = div31_1to31_rates },
	{ .parent = NULL },
};

static struct clk dpll_disp_m2_ck = {
	.name		= "dpll_disp_m2_ck",
	.parent		= &dpll_disp_ck,
	.clksel		= dpll_disp_m2_div,
	.clksel_reg	= AM33XX_CM_DIV_M2_DPLL_DISP,
	.clksel_mask	= AM33XX_DPLL_CLKOUT_DIV_MASK,
	.ops		= &clkops_null,
	.recalc		= &omap2_clksel_recalc,
	.round_rate	= &omap2_clksel_round_rate,
	.set_rate	= &omap2_clksel_set_rate,
};

static struct clk disp_pll_clk = {
	.name		= "disp_pll_clk",
	.parent		= &dpll_disp_m2_ck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

/* DPLL_PER */
static struct dpll_data dpll_per_dd = {
	.mult_div1_reg	= AM33XX_CM_CLKSEL_DPLL_PERIPH,
	.clk_bypass	= &sys_clkin_ck,
	.clk_ref	= &sys_clkin_ck,
	.control_reg	= AM33XX_CM_CLKMODE_DPLL_PER,
	.modes		= (1 << DPLL_LOW_POWER_BYPASS) | (1 << DPLL_LOCKED),
	.idlest_reg	= AM33XX_CM_IDLEST_DPLL_PER,
	.mult_mask	= AM33XX_DPLL_MULT_PERIPH_MASK,
	.div1_mask	= AM33XX_DPLL_PER_DIV_MASK,
	.enable_mask	= AM33XX_DPLL_EN_MASK,
	.idlest_mask	= AM33XX_ST_DPLL_CLK_MASK,
	.max_multiplier	= AM33XX_MAX_DPLL_MULT,
	.max_divider	= AM33XX_MAX_DPLL_DIV,
	.min_divider	= 1,
	.flags		= DPLL_J_TYPE,
};

/* CLKDCOLDO */
static struct clk dpll_per_ck = {
	.name		= "dpll_per_ck",
	.parent		= &sys_clkin_ck,
	.dpll_data	= &dpll_per_dd,
	.init		= &omap2_init_dpll_parent,
	.ops		= &clkops_null,
	.recalc		= &omap3_dpll_recalc,
	.round_rate	= &omap2_dpll_round_rate,
	.set_rate	= &omap3_noncore_dpll_set_rate,
};

/* CLKOUT: fdpll/M2 */
static const struct clksel dpll_per_m2_div[] = {
	{ .parent = &dpll_per_ck, .rates = div31_1to31_rates },
	{ .parent = NULL },
};

static struct clk dpll_per_m2_ck = {
	.name		= "dpll_per_m2_ck",
	.parent		= &dpll_per_ck,
	.clksel		= dpll_per_m2_div,
	.clksel_reg	= AM33XX_CM_DIV_M2_DPLL_PER,
	.clksel_mask	= AM33XX_DPLL_CLKOUT_DIV_MASK,
	.ops		= &clkops_null,
	.recalc		= &omap2_clksel_recalc,
	.round_rate	= &omap2_clksel_round_rate,
	.set_rate	= &omap2_clksel_set_rate,
};

static struct clk per_192mhz_clk = {
	.name		= "per_192mhz_clk",
	.parent		= &dpll_per_m2_ck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk usb_pll_clk = {
	.name		= "usb_pll_clk",
	.parent		= &dpll_per_ck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk core_100mhz_ck = {
	.name		= "core_100mhz_ck",
	.parent		= &sysclk1_ck,
	.ops		= &clkops_null,
	.fixed_div	= 2,
	.recalc		= &omap_fixed_divisor_recalc,
};

static struct clk l3_aon_gclk = {
	.name		= "l3_aon_gclk",
	.parent		= &sysclk1_ck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk l4_wkup_aon_gclk = {
	.name		= "l4_wkup_aon_gclk",
	.clkdm_name	= "l4_wkup_aon_clkdm",
	.parent		= &sysclk1_ck,
	.enable_reg	= AM33XX_CM_L4_WKUP_AON_CLKSTCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_null,
	.fixed_div	= 2,
	.recalc		= &followparent_recalc,
};

static struct clk l3_gclk = {
	.name		= "l3_gclk",
	.parent		= &sysclk1_ck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk l3_ick = {
	.name		= "l3_ick",
	.clkdm_name	= "l3_clkdm",
	.parent		= &l3_gclk,
	.enable_reg	= AM33XX_CM_PER_L3_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.flags		= ENABLE_ON_INIT,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk l3_instr_ick = {
	.name		= "l3_instr_ick",
	.clkdm_name	= "l3_clkdm",
	.parent		= &l3_gclk,
	.enable_reg	= AM33XX_CM_PER_L3_INSTR_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.flags		= ENABLE_ON_INIT,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk l4_wkup_gclk = {
	.name		= "l4_wkup_gclk",
	.parent		= &sysclk1_ck,
	.ops		= &clkops_null,
	.fixed_div	= 2,
	.recalc		= &omap_fixed_divisor_recalc,
};

static struct clk l4hs_gclk = {
	.name		= "l4hs_gclk",
	.parent		= &sysclk1_ck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk gfx_l3_gclk = {
	.name		= "gfx_l3_gclk",
	.clkdm_name	= "gfx_l3_clkdm",
	.parent		= &sysclk1_ck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk debug_clka_gclk = {
	.name		= "debug_clka_gclk",
	.parent		= &sysclk1_ck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk l4_rtc_gclk = {
	.name		= "l4_rtc_gclk",
	.parent		= &sysclk1_ck,
	.ops		= &clkops_null,
	.fixed_div	= 2,
	.recalc		= &omap_fixed_divisor_recalc,
};

static struct clk rtc_ick = {
	.name		= "rtc_ick",
	.parent		= &l4_rtc_gclk,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk l3s_gclk = {
	.name		= "l3s_gclk",
	.parent		= &core_100mhz_ck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk l4fw_gclk = {
	.name		= "l4fw_gclk",
	.parent		= &core_100mhz_ck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk l4ls_gclk = {
	.name		= "l4ls_gclk",
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &core_100mhz_ck,
	.enable_reg	= AM33XX_CM_PER_L4LS_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk clk_24mhz = {
	.name		= "clk_24mhz",
	.parent		= &per_192mhz_clk,
	.fixed_div	= 8,
	.ops		= &clkops_null,
	.recalc		= &omap_fixed_divisor_recalc,
};

static struct clk l4_cefuse_gclk = {
	.name		= "l4_cefsue_gclk",
	.parent		= &core_100mhz_ck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk cefuse_iclk = {
	.name		= "cefuse_iclk",
	.clkdm_name	= "l4_cefuse_clkdm",
	.parent		= &l4_cefuse_gclk,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk cefuse_fck = {
	.name		= "cefuse_fck",
	.clkdm_name	= "l4_cefuse_clkdm",
	.parent		= &sys_clkin_ck,
	.enable_reg	= AM33XX_CM_CEFUSE_CEFUSE_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk sysclk_div_ck = {
	.name		= "sysclk_div_ck",
	.parent		= &dpll_core_m4_ck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk adc_tsc_fck = {
	.name		= "adc_tsc_fck",
	.clkdm_name	= "l4_wkup_clkdm",
	.parent		= &sys_clkin_ck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk adc_tsc_ick = {
	.name		= "adc_tsc_ick",
	.clkdm_name	= "l4_wkup_clkdm",
	.parent		= &l4_wkup_gclk,
	.enable_reg	= AM33XX_CM_WKUP_ADC_TSC_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk aes0_fck = {
	.name		= "aes0_fck",
	.clkdm_name	= "l3_clkdm",
	.parent		= &l3_gclk,
	.enable_reg	= AM33XX_CM_PER_AES0_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

/*
 * clkdiv32 is generated from fixed division of 732.4219
 */
static struct clk clkdiv32k_ick = {
	.name		= "clkdiv32k_ick",
	.clkdm_name	= "clk_24mhz_clkdm",
	.rate		= 32768,
	.parent		= &clk_24mhz,
	.enable_reg	= AM33XX_CM_PER_CLKDIV32K_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
};

static struct clk clk_32khz_ck = {
	.name		= "clk_32khz_ck",
	.clkdm_name	= "clk_24mhz_clkdm",
	.parent		= &clkdiv32k_ick,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk control_fck = {
	.name		= "control_fck",
	.clkdm_name	= "l4_wkup_clkdm",
	.parent		= &l4_wkup_gclk,
	.enable_reg	= AM33XX_CM_WKUP_CONTROL_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk dcan0_ick = {
	.name		= "dcan0_ick",
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &l4ls_gclk,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk dcan0_fck = {
	.name		= "dcan0_fck",
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &sys_clkin_ck,
	.enable_reg	= AM33XX_CM_PER_DCAN0_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk dcan1_ick = {
	.name		= "dcan1_ick",
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &l4ls_gclk,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk dcan1_fck = {
	.name		= "dcan1_fck",
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &sys_clkin_ck,
	.enable_reg	= AM33XX_CM_PER_DCAN1_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk debugss_ick = {
	.name		= "debugss_ick",
	.clkdm_name	= "l3_aon_clkdm",
	.parent		= &l3_aon_gclk,
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_WKUP_DEBUGSS_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.recalc		= &followparent_recalc,
};

static struct clk elm_fck = {
	.name		= "elm_fck",
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &l4ls_gclk,
	.enable_reg	= AM33XX_CM_PER_ELM_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk emif_fw_fck = {
	.name		= "emif_fw_fck",
	.clkdm_name	= "l4fw_clkdm",
	.parent		= &l4fw_gclk,
	.enable_reg	= AM33XX_CM_PER_EMIF_FW_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk epwmss0_fck = {
	.name		= "epwmss0_fck",
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &l4ls_gclk,
	.enable_reg	= AM33XX_CM_PER_EPWMSS0_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk epwmss1_fck = {
	.name		= "epwmss1_fck",
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &l4ls_gclk,
	.enable_reg	= AM33XX_CM_PER_EPWMSS1_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk epwmss2_fck = {
	.name		= "epwmss2_fck",
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &l4ls_gclk,
	.enable_reg	= AM33XX_CM_PER_EPWMSS2_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk gpmc_fck = {
	.name		= "gpmc_fck",
	.clkdm_name	= "l3s_clkdm",
	.parent		= &l3s_gclk,
	.enable_reg	= AM33XX_CM_PER_GPMC_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk i2c1_ick = {
	.name		= "i2c1_ick",
	.clkdm_name	= "l4_wkup_clkdm",
	.parent		= &l4_wkup_gclk,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk i2c1_fck = {
	.name		= "i2c1_fck",
	.clkdm_name	= "l4_wkup_clkdm",
	.parent		= &per_192mhz_clk,
	.enable_reg	= AM33XX_CM_WKUP_I2C0_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.fixed_div	= 4,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &omap_fixed_divisor_recalc,
};

static struct clk i2c2_ick = {
	.name		= "i2c2_ick",
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &l4ls_gclk,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk i2c2_fck = {
	.name		= "i2c2_fck",
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &per_192mhz_clk,
	.enable_reg	= AM33XX_CM_PER_I2C1_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.fixed_div	= 4,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &omap_fixed_divisor_recalc,
};

static struct clk i2c3_ick = {
	.name		= "i2c3_ick",
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &l4ls_gclk,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk i2c3_fck = {
	.name		= "i2c3_fck",
	.parent		= &per_192mhz_clk,
	.enable_reg	= AM33XX_CM_PER_I2C2_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4ls_clkdm",
	.fixed_div	= 4,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &omap_fixed_divisor_recalc,
};

static struct clk ieee5000_fck = {
	.name		= "ieee5000_fck",
	.clkdm_name	= "l3s_clkdm",
	.parent		= &l3s_gclk,
	.enable_reg	= AM33XX_CM_PER_IEEE5000_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk l4hs_ick = {
	.name		= "l4hs_ick",
	.clkdm_name	= "l4hs_clkdm",
	.parent		= &l4hs_gclk,
	.enable_reg	= AM33XX_CM_PER_L4HS_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.flags		= ENABLE_ON_INIT,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk l4wkup_ick = {
	.name		= "l4wkup_ick",
	.clkdm_name	= "l4_wkup_aon_clkdm",
	.parent		= &l4_wkup_aon_gclk,
	.enable_reg	= AM33XX_CM_L4_WKUP_AON_CLKSTCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.flags		= ENABLE_ON_INIT,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk l4fw_ick = {
	.name		= "l4fw_ick",
	.clkdm_name	= "l4fw_clkdm",
	.parent		= &core_100mhz_ck,
	.enable_reg	= AM33XX_CM_PER_L4FW_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.flags		= ENABLE_ON_INIT,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk l4ls_ick = {
	.name		= "l4ls_ick",
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &l4ls_gclk,
	.enable_reg	= AM33XX_CM_PER_L4LS_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.flags		= ENABLE_ON_INIT,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk mailbox0_fck = {
	.name		= "mailbox0_fck",
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &l4ls_gclk,
	.enable_reg	= AM33XX_CM_PER_MAILBOX0_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk mcasp0_ick = {
	.name		= "mcasp0_ick",
	.parent		= &l3s_gclk,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk mcasp0_fck = {
	.name		= "mcasp0_fck",
	.clkdm_name	= "l3s_clkdm",
	.parent		= &sys_clkin_ck,
	.enable_reg	= AM33XX_CM_PER_MCASP0_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk mcasp1_ick = {
	.name		= "mcasp1_ick",
	.parent		= &l3s_gclk,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk mcasp1_fck = {
	.name		= "mcasp1_fck",
	.clkdm_name	= "l3s_clkdm",
	.parent		= &sys_clkin_ck,
	.enable_reg	= AM33XX_CM_PER_MCASP1_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk mlb_fck = {
	.name		= "mlb_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_MLB_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l3_clkdm",
	.parent		= &sysclk_div_ck,
	.recalc		= &followparent_recalc,
};

static struct clk mmu_fck = {
	.name		= "mmu_fck",
	.clkdm_name	= "gfx_l3_clkdm",
	.parent		= &gfx_l3_gclk,
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_GFX_MMUDATA_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.recalc		= &followparent_recalc,
};

static struct clk ocmcram_ick = {
	.name		= "ocmcram_ick",
	.clkdm_name	= "l3_clkdm",
	.parent		= &l3_gclk,
	.enable_reg	= AM33XX_CM_PER_OCMCRAM_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk ocpwp_fck = {
	.name		= "ocpwp_fck",
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &l4ls_gclk,
	.enable_reg	= AM33XX_CM_PER_OCPWP_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk pka_fck = {
	.name		= "pka_fck",
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &l4ls_gclk,
	.enable_reg	= AM33XX_CM_PER_PKA_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk rng_fck = {
	.name		= "rng_fck",
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &l4ls_gclk,
	.enable_reg	= AM33XX_CM_PER_RNG_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk rtc_fck = {
	.name		= "rtc_fck",
	.clkdm_name	= "l4_rtc_clkdm",
	.parent		= &clk_32768_ck,
	.enable_reg	= AM33XX_CM_RTC_RTC_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk sha0_fck = {
	.name		= "sha0_fck",
	.clkdm_name	= "l3_clkdm",
	.parent		= &l3_gclk,
	.enable_reg	= AM33XX_CM_PER_SHA0_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk smartreflex0_ick = {
	.name		= "smartreflex0_ick",
	.parent		= &l4_wkup_gclk,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk smartreflex0_fck = {
	.name		= "smartreflex0_fck",
	.clkdm_name	= "l4_wkup_clkdm",
	.parent		= &sys_clkin_ck,
	.enable_reg	= AM33XX_CM_WKUP_SMARTREFLEX0_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk smartreflex1_ick = {
	.name		= "smartreflex1_ick",
	.parent		= &l4_wkup_gclk,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk smartreflex1_fck = {
	.name		= "smartreflex1_fck",
	.clkdm_name	= "l4_wkup_clkdm",
	.parent		= &sys_clkin_ck,
	.enable_reg	= AM33XX_CM_WKUP_SMARTREFLEX1_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk spi0_ick = {
	.name		= "spi0_ick",
	.parent		= &l4ls_gclk,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk spi0_fck = {
	.name		= "spi0_fck",
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &per_192mhz_clk,
	.enable_reg	= AM33XX_CM_PER_SPI0_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.fixed_div	= 4,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &omap_fixed_divisor_recalc,
};

static struct clk spi1_ick = {
	.name		= "spi1_ick",
	.parent		= &l4ls_gclk,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk spi1_fck = {
	.name		= "spi1_fck",
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &per_192mhz_clk,
	.enable_reg	= AM33XX_CM_PER_SPI1_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.fixed_div	= 4,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &omap_fixed_divisor_recalc,
};

static struct clk spinlock_fck = {
	.name		= "spinlock_fck",
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &l4ls_gclk,
	.enable_reg	= AM33XX_CM_PER_SPINLOCK_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk clk_32khz_timer = {
	.name		= "clk_32khz_timer",
	.parent		= &clk_32khz_ck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

/* Timers */

/* Secure Timer: Used only to disable the clocks and for completeness */
static const struct clksel timer0_clkmux_sel[] = {
	{ .parent = &clk_rc32k_ck, .rates = div_1_0_rates },
	{ .parent = &clk_32khz_ck, .rates = div_1_1_rates },
	{ .parent = &sys_clkin_ck, .rates = div_1_2_rates },
	{ .parent = &tclkin_ck, .rates = div_1_3_rates },
	{ .parent = NULL },
};

static struct clk timer0_ick = {
	.name		= "timer0_ick",
	.parent		= &l4_wkup_gclk,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk timer0_fck = {
	.name		= "timer0_fck",
	.clkdm_name	= "l4_wkup_clkdm",
	.parent		= &clk_rc32k_ck,
	.clksel		= timer0_clkmux_sel,
	.enable_reg	= AM33XX_CM_WKUP_TIMER0_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clksel_reg	= AM33XX_CTRL_REGADDR(0x01BC),
	.clksel_mask	= (0x3 << 4),
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static const struct clksel timer1_clkmux_sel[] = {
	{ .parent = &sys_clkin_ck, .rates = div_1_0_rates },
	{ .parent = &clk_32khz_ck, .rates = div_1_1_rates },
	{ .parent = &tclkin_ck, .rates = div_1_2_rates },
	{ .parent = &clk_rc32k_ck, .rates = div_1_3_rates },
	{ .parent = &clk_32768_ck, .rates = div_1_4_rates },
	{ .parent = NULL },
};

static struct clk timer1_ick = {
	.name		= "timer1_ick",
	.parent		= &l4_wkup_gclk,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk timer1_fck = {
	.name		= "timer1_fck",
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &sys_clkin_ck,
	.init		= &omap2_init_clksel_parent,
	.clksel		= timer1_clkmux_sel,
	.enable_reg	= AM33XX_CM_WKUP_TIMER1_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clksel_reg	= AM33XX_CLKSEL_TIMER1MS_CLK,
	.clksel_mask	= AM33XX_CLKSEL_0_2_MASK,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &omap2_clksel_recalc,
};

static const struct clksel timer2_to_7_clk_sel[] = {
	{ .parent = &tclkin_ck, .rates = div_1_0_rates },
	{ .parent = &sys_clkin_ck, .rates = div_1_1_rates },
	{ .parent = &clk_32khz_timer, .rates = div_1_2_rates },
	{ .parent = NULL },
};

static struct clk timer2_ick = {
	.name		= "timer2_ick",
	.parent		= &l4ls_gclk,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk timer2_fck = {
	.name		= "timer2_fck",
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &sys_clkin_ck,
	.init		= &omap2_init_clksel_parent,
	.clksel		= timer2_to_7_clk_sel,
	.enable_reg	= AM33XX_CM_PER_TIMER2_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clksel_reg	= AM33XX_CLKSEL_TIMER2_CLK,
	.clksel_mask	= AM33XX_CLKSEL_0_1_MASK,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &omap2_clksel_recalc,
};

static struct clk timer3_ick = {
	.name		= "timer3_ick",
	.parent		= &l4ls_gclk,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk timer3_fck = {
	.name		= "timer3_fck",
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &sys_clkin_ck,
	.init		= &am33xx_init_timer_parent,
	.clksel		= timer2_to_7_clk_sel,
	.enable_reg	= AM33XX_CM_PER_TIMER3_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clksel_reg	= AM33XX_CLKSEL_TIMER3_CLK,
	.clksel_mask	= AM33XX_CLKSEL_0_1_MASK,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &omap2_clksel_recalc,
};

static struct clk timer4_ick = {
	.name		= "timer4_ick",
	.parent		= &l4ls_gclk,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk timer4_fck = {
	.name		= "timer4_fck",
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &sys_clkin_ck,
	.init		= &omap2_init_clksel_parent,
	.clksel		= timer2_to_7_clk_sel,
	.enable_reg	= AM33XX_CM_PER_TIMER4_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clksel_reg	= AM33XX_CLKSEL_TIMER4_CLK,
	.clksel_mask	= AM33XX_CLKSEL_0_1_MASK,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &omap2_clksel_recalc,
};

static struct clk timer5_ick = {
	.name		= "timer5_ick",
	.parent		= &l4ls_gclk,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk timer5_fck = {
	.name		= "timer5_fck",
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &sys_clkin_ck,
	.init		= &omap2_init_clksel_parent,
	.clksel		= timer2_to_7_clk_sel,
	.enable_reg	= AM33XX_CM_PER_TIMER5_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clksel_reg	= AM33XX_CLKSEL_TIMER5_CLK,
	.clksel_mask	= AM33XX_CLKSEL_0_1_MASK,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &omap2_clksel_recalc,
};

static struct clk timer6_ick = {
	.name		= "timer6_ick",
	.parent		= &l4ls_gclk,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk timer6_fck = {
	.name		= "timer6_fck",
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &sys_clkin_ck,
	.init		= &am33xx_init_timer_parent,
	.clksel		= timer2_to_7_clk_sel,
	.enable_reg	= AM33XX_CM_PER_TIMER6_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clksel_reg	= AM33XX_CLKSEL_TIMER6_CLK,
	.clksel_mask	= AM33XX_CLKSEL_0_1_MASK,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &omap2_clksel_recalc,
};

static struct clk timer7_ick = {
	.name		= "timer7_ick",
	.parent		= &l4ls_gclk,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk timer7_fck = {
	.name		= "timer7_fck",
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &sys_clkin_ck,
	.init		= &omap2_init_clksel_parent,
	.clksel		= timer2_to_7_clk_sel,
	.enable_reg	= AM33XX_CM_PER_TIMER7_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clksel_reg	= AM33XX_CLKSEL_TIMER7_CLK,
	.clksel_mask	= AM33XX_CLKSEL_0_1_MASK,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &omap2_clksel_recalc,
};

static struct clk tpcc_ick = {
	.name		= "tpcc_ick",
	.clkdm_name	= "l3_clkdm",
	.parent		= &l3_gclk,
	.enable_reg	= AM33XX_CM_PER_TPCC_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk tptc0_ick = {
	.name		= "tptc0_ick",
	.parent		= &l3_gclk,
	.clkdm_name	= "l3_clkdm",
	.enable_reg	= AM33XX_CM_PER_TPTC0_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk tptc1_ick = {
	.name		= "tptc1_ick",
	.clkdm_name	= "l3_clkdm",
	.parent		= &l3_gclk,
	.enable_reg	= AM33XX_CM_PER_TPTC1_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk tptc2_ick = {
	.name		= "tptc2_ick",
	.clkdm_name	= "l3_clkdm",
	.parent		= &l3_gclk,
	.enable_reg	= AM33XX_CM_PER_TPTC2_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk uart1_ick = {
	.name		= "uart1_ick",
	.parent		= &l4_wkup_gclk,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk uart1_fck = {
	.name		= "uart1_fck",
	.clkdm_name	= "l4_wkup_clkdm",
	.parent		= &per_192mhz_clk,
	.enable_reg	= AM33XX_CM_WKUP_UART0_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.fixed_div	= 4,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &omap_fixed_divisor_recalc,
};

static struct clk uart2_ick = {
	.name		= "uart2_ick",
	.parent		= &l4ls_gclk,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk uart2_fck = {
	.name		= "uart2_fck",
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &per_192mhz_clk,
	.enable_reg	= AM33XX_CM_PER_UART1_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.fixed_div	= 4,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &omap_fixed_divisor_recalc,
};

static struct clk uart3_ick = {
	.name		= "uart3_ick",
	.parent		= &l4ls_gclk,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk uart3_fck = {
	.name		= "uart3_fck",
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &per_192mhz_clk,
	.enable_reg	= AM33XX_CM_PER_UART2_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.fixed_div	= 4,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &omap_fixed_divisor_recalc,
};

static struct clk uart4_ick = {
	.name		= "uart4_ick",
	.parent		= &l4ls_gclk,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk uart4_fck = {
	.name		= "uart4_fck",
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &per_192mhz_clk,
	.enable_reg	= AM33XX_CM_PER_UART3_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.fixed_div	= 4,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &omap_fixed_divisor_recalc,
};

static struct clk uart5_ick = {
	.name		= "uart5_ick",
	.parent		= &l4ls_gclk,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk uart5_fck = {
	.name		= "uart5_fck",
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &per_192mhz_clk,
	.enable_reg	= AM33XX_CM_PER_UART4_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.fixed_div	= 4,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &omap_fixed_divisor_recalc,
};

static struct clk uart6_ick = {
	.name		= "uart6_ick",
	.parent		= &l4ls_gclk,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk uart6_fck = {
	.name		= "uart6_fck",
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &per_192mhz_clk,
	.enable_reg	= AM33XX_CM_PER_UART5_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.fixed_div	= 4,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &omap_fixed_divisor_recalc,
};

static struct clk wkup_m3_fck = {
	.name		= "wkup_m3_fck",
	.clkdm_name	= "l4_wkup_aon_clkdm",
	.parent		= &l4_wkup_aon_gclk,
	.enable_reg	= AM33XX_CM_WKUP_WKUP_M3_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk cpsw_250mhz_clk = {
	.name		= "cpsw_250mhz_clk",
	.clkdm_name	= "l4hs_clkdm",
	.parent		= &sysclk2_ck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk cpsw_125mhz_gclk = {
	.name		= "cpsw_125mhz_gclk",
	.clkdm_name	= "cpsw_125mhz_clkdm",
	.parent		= &sysclk2_ck,
	.ops		= &clkops_null,
	.fixed_div	= 2,
	.recalc		= &omap_fixed_divisor_recalc,
};

/*
 * TODO: As per clock tree @OPP50 /2 is used, but there is not register
 * to configure this. @ normal OPP, /5 is used - 250MHz/5 = 50MHz
 */
static struct clk cpsw_50mhz_clk = {
	.name		= "cpsw_50mhz_clk",
	.clkdm_name	= "l4hs_clkdm",
	.parent		= &sysclk2_ck,
	.ops		= &clkops_null,
	.fixed_div	= 5,
	.recalc		= &omap_fixed_divisor_recalc,
};

static struct clk cpsw_5mhz_clk = {
	.name		= "cpsw_5mhz_clk",
	.clkdm_name	= "l4hs_clkdm",
	.parent		= &cpsw_50mhz_clk,
	.ops		= &clkops_null,
	.fixed_div	= 10,
	.recalc		= &omap_fixed_divisor_recalc,
};

static struct clk cpgmac0_ick = {
	.name		= "cpgmac0_ick",
	.clkdm_name	= "cpsw_125mhz_clkdm",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_CPGMAC0_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.parent		= &cpsw_125mhz_gclk,
	.recalc		= &followparent_recalc,
};

static const struct clksel cpsw_cpts_rft_clkmux_sel[] = {
	{ .parent = &sysclk2_ck, .rates = div_1_0_rates },
	{ .parent = &sysclk1_ck, .rates = div_1_1_rates },
	{ .parent = NULL },
};

static struct clk cpsw_cpts_rft_clk = {
	.name		= "cpsw_cpts_rft_clk",
	.clkdm_name	= "l3_clkdm",
	.parent		= &dpll_core_m5_ck,
	.clksel		= cpsw_cpts_rft_clkmux_sel,
	.clksel_reg	= AM33XX_CM_CPTS_RFT_CLKSEL,
	.clksel_mask	= AM33XX_CLKSEL_0_0_MASK,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk usbotg_ick = {
	.name		= "usbotg_ick",
	.clkdm_name	= "l3s_clkdm",
	.parent		= &l3s_gclk,
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_USB0_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.recalc		= &followparent_recalc,
};

static struct clk usbotg_fck = {
	.name		= "usbotg_fck",
	.clkdm_name	= "l3s_clkdm",
	.parent		= &usb_pll_clk,
	.enable_reg	= AM33XX_CM_CLKDCOLDO_DPLL_PER,
	.enable_bit	= AM33XX_ST_DPLL_CLKDCOLDO_SHIFT,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

/* gpio */
static const struct clksel gpio0_dbclk_mux_sel[] = {
	{ .parent = &clk_rc32k_ck, .rates = div_1_0_rates },
	{ .parent = &clk_32768_ck, .rates = div_1_1_rates },
	{ .parent = &clk_32khz_timer, .rates = div_1_2_rates },
	{ .parent = NULL },
};

static struct clk gpio0_dbclk_mux_ck = {
	.name		= "gpio0_dbclk_mux_ck",
	.clkdm_name	= "l4_wkup_clkdm",
	.parent		= &clk_rc32k_ck,
	.init		= &omap2_init_clksel_parent,
	.clksel		= gpio0_dbclk_mux_sel,
	.clksel_reg	= AM33XX_CLKSEL_GPIO0_DBCLK,
	.clksel_mask	= AM33XX_CLKSEL_0_1_MASK,
	.ops		= &clkops_null,
	.recalc		= &omap2_clksel_recalc,
};

static struct clk gpio0_dbclk = {
	.name		= "gpio0_dbclk",
	.clkdm_name	= "l4_wkup_clkdm",
	.parent		= &gpio0_dbclk_mux_ck,
	.enable_reg	= AM33XX_CM_WKUP_GPIO0_CLKCTRL,
	.enable_bit	= AM33XX_OPTFCLKEN_GPIO0_GDBCLK_SHIFT,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk gpio0_ick = {
	.name		= "gpio0_ick",
	.clkdm_name	= "l4_wkup_clkdm",
	.parent		= &l4_wkup_gclk,
	.enable_reg	= AM33XX_CM_WKUP_GPIO0_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk gpio1_dbclk = {
	.name		= "gpio1_dbclk",
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &clkdiv32k_ick,
	.enable_reg	= AM33XX_CM_PER_GPIO1_CLKCTRL,
	.enable_bit	= AM33XX_OPTFCLKEN_GPIO_1_GDBCLK_SHIFT,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk gpio1_ick = {
	.name		= "gpio1_ick",
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &l4ls_gclk,
	.enable_reg	= AM33XX_CM_PER_GPIO1_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk gpio2_dbclk = {
	.name		= "gpio2_dbclk",
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &clkdiv32k_ick,
	.enable_reg	= AM33XX_CM_PER_GPIO2_CLKCTRL,
	.enable_bit	= AM33XX_OPTFCLKEN_GPIO_2_GDBCLK_SHIFT,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk gpio2_ick = {
	.name		= "gpio2_ick",
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &l4ls_gclk,
	.enable_reg	= AM33XX_CM_PER_GPIO2_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk gpio3_dbclk = {
	.name		= "gpio3_dbclk",
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &clkdiv32k_ick,
	.enable_reg	= AM33XX_CM_PER_GPIO3_CLKCTRL,
	.enable_bit	= AM33XX_OPTFCLKEN_GPIO_3_GDBCLK_SHIFT,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk gpio3_ick = {
	.name		= "gpio3_ick",
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &l4ls_gclk,
	.enable_reg	= AM33XX_CM_PER_GPIO3_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static const struct clksel pruss_ocp_clk_mux_sel[] = {
	{ .parent = &l3_gclk, .rates = div_1_0_rates },
	{ .parent = &disp_pll_clk, .rates = div_1_1_rates },
	{ .parent = NULL },
};

static struct clk pruss_ocp_gclk = {
	.name		= "pruss_ocp_gclk",
	.parent		= &l3_gclk,
	.init		= &omap2_init_clksel_parent,
	.clksel		= pruss_ocp_clk_mux_sel,
	.clksel_reg	= AM33XX_CLKSEL_PRUSS_OCP_CLK,
	.clksel_mask	= AM33XX_CLKSEL_0_0_MASK,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk pruss_iep_gclk = {
	.name		= "pruss_iep_gclk",
	.clkdm_name	= "pruss_ocp_clkdm",
	.parent		= &l3_gclk,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk pruss_uart_gclk = {
	.name		= "pruss_uart_gclk",
	.clkdm_name	= "pruss_ocp_clkdm",
	.parent		= &per_192mhz_clk,
	.enable_reg	= AM33XX_CM_PER_PRUSS_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk lcdc_ick = {
	.name		= "lcdc_ick",
	.clkdm_name	= "l3_clkdm",
	.parent		= &sysclk1_ck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static const struct clksel lcd_clk_mux_sel[] = {
	{ .parent = &disp_pll_clk, .rates = div_1_0_rates },
	{ .parent = &sysclk2_ck, .rates = div_1_1_rates },
	{ .parent = &per_192mhz_clk, .rates = div_1_2_rates },
	{ .parent = NULL },
};

static struct clk lcd_gclk = {
	.name		= "lcd_gclk",
	.parent		= &disp_pll_clk,
	.init		= &omap2_init_clksel_parent,
	.clksel		= lcd_clk_mux_sel,
	.clksel_reg	= AM33XX_CLKSEL_LCDC_PIXEL_CLK,
	.clksel_mask	= AM33XX_CLKSEL_0_1_MASK,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk lcdc_fck = {
	.name		= "lcdc_fck",
	.clkdm_name	= "lcdc_clkdm",
	.parent		= &lcd_gclk,
	.enable_reg	= AM33XX_CM_PER_LCDC_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk mmc_clk = {
	.name		= "mmc_clk",
	.parent		= &per_192mhz_clk,
	.ops		= &clkops_null,
	.fixed_div	= 2,
	.recalc		= &omap_fixed_divisor_recalc,
};

static struct clk mmc0_ick = {
	.name		= "mmc0_ick",
	.parent		= &l4ls_gclk,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk mmc0_fck = {
	.name		= "mmc0_fck",
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &mmc_clk,
	.enable_reg	= AM33XX_CM_PER_MMC0_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk mmc1_ick = {
	.name		= "mmc1_ick",
	.parent		= &l4ls_gclk,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk mmc1_fck = {
	.name		= "mmc1_fck",
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &mmc_clk,
	.enable_reg	= AM33XX_CM_PER_MMC1_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk mmc2_ick = {
	.name		= "mmc2_ick",
	.parent		= &l4ls_gclk,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk mmc2_fck = {
	.name		= "mmc2_fck",
	.clkdm_name	= "l3s_clkdm",
	.parent		= &mmc_clk,
	.enable_reg	= AM33XX_CM_PER_MMC2_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static const struct clksel gfx_clksel_sel[] = {
	{ .parent = &sysclk1_ck, .rates = div_1_0_rates },
	{ .parent = &per_192mhz_clk, .rates = div_1_1_rates },
	{ .parent = NULL },
};

static struct clk gfx_fclk_clksel_ck = {
	.name		= "gfx_fclk_clksel_ck",
	.parent		= &sysclk1_ck,
	.clksel		= gfx_clksel_sel,
	.ops		= &clkops_null,
	.clksel_reg	= AM33XX_CLKSEL_GFX_FCLK,
	.clksel_mask	= AM33XX_CLKSEL_GFX_FCLK_MASK,
	.recalc		= &omap2_clksel_recalc,
};

static const struct clksel_rate div_1_0_2_1_rates[] = {
	{ .div = 1, .val = 0, .flags = RATE_IN_AM33XX },
	{ .div = 2, .val = 1, .flags = RATE_IN_AM33XX },
	{ .div = 0 },
};

static const struct clksel gfx_div_sel[] = {
	{ .parent = &gfx_fclk_clksel_ck, .rates = div_1_0_2_1_rates },
	{ .parent = NULL },
};

static struct clk gfx_ick = {
	.name		= "gfx_ick",
	.clkdm_name	= "gfx_l3_clkdm",
	.parent		= &gfx_l3_gclk,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk gfx_fclk = {
	.name		= "gfx_fclk",
	.clkdm_name	= "gfx_l3_clkdm",
	.parent		= &gfx_fclk_clksel_ck,
	.clksel		= gfx_div_sel,
	.enable_reg	= AM33XX_CM_GFX_GFX_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clksel_reg	= AM33XX_CLKSEL_GFX_FCLK,
	.clksel_mask	= AM33XX_CLKSEL_0_0_MASK,
	.recalc		= &omap2_clksel_recalc,
	.round_rate	= &omap2_clksel_round_rate,
	.set_rate	= &omap2_clksel_set_rate,
	.ops		= &clkops_omap2_dflt,
};

static const struct clksel sysclkout_pre_sel[] = {
	{ .parent = &clk_32768_ck, .rates = div_1_0_rates },
	{ .parent = &l3_gclk, .rates = div_1_1_rates },
	{ .parent = &ddr_pll_clk, .rates = div_1_2_rates },
	{ .parent = &per_192mhz_clk, .rates = div_1_3_rates },
	{ .parent = &lcd_gclk, .rates = div_1_4_rates },
	{ .parent = NULL },
};

static struct clk sysclkout_pre_ck = {
	.name		= "sysclkout_pre_ck",
	.parent		= &clk_32768_ck,
	.init		= &omap2_init_clksel_parent,
	.clksel		= sysclkout_pre_sel,
	.clksel_reg	= AM33XX_CM_CLKOUT_CTRL,
	.clksel_mask	= AM33XX_CLKOUT2SOURCE_MASK,
	.ops		= &clkops_null,
	.recalc		= &omap2_clksel_recalc,
};

/* Divide by 8 clock rates with default clock is 1/1*/
static const struct clksel_rate div8_rates[] = {
	{ .div = 1, .val = 0, .flags = RATE_IN_AM33XX },
	{ .div = 2, .val = 1, .flags = RATE_IN_AM33XX },
	{ .div = 3, .val = 2, .flags = RATE_IN_AM33XX },
	{ .div = 4, .val = 3, .flags = RATE_IN_AM33XX },
	{ .div = 5, .val = 4, .flags = RATE_IN_AM33XX },
	{ .div = 6, .val = 5, .flags = RATE_IN_AM33XX },
	{ .div = 7, .val = 6, .flags = RATE_IN_AM33XX },
	{ .div = 8, .val = 7, .flags = RATE_IN_AM33XX },
	{ .div = 0 },
};

static const struct clksel clkout2_div[] = {
	{ .parent = &sysclkout_pre_ck, .rates = div8_rates },
	{ .parent = NULL },
};

static struct clk clkout2_ck = {
	.name		= "clkout2_ck",
	.parent		= &sysclkout_pre_ck,
	.ops		= &clkops_omap2_dflt,
	.clksel		= clkout2_div,
	.clksel_reg	= AM33XX_CM_CLKOUT_CTRL,
	.clksel_mask	= AM33XX_CLKOUT2DIV_MASK,
	.enable_reg	= AM33XX_CM_CLKOUT_CTRL,
	.enable_bit	= AM33XX_CLKOUT2EN_SHIFT,
	.recalc		= &omap2_clksel_recalc,
	.round_rate	= &omap2_clksel_round_rate,
	.set_rate	= &omap2_clksel_set_rate,
};

static struct clk vtp_clk = {
	.name		= "vtp_clk",
	.parent		= &sys_clkin_ck,
	.ops		= &clkops_null,
	.fixed_div	= 2,
	.recalc		= &omap_fixed_divisor_recalc,
};

static const struct clksel wdt_clkmux_sel[] = {
	{ .parent = &clk_rc32k_ck, .rates = div_1_0_rates },
	{ .parent = &clk_32khz_ck, .rates = div_1_1_rates },
	{ .parent = NULL },
};

static struct clk wdt0_ick = {
	.name		= "wdt0_ick",
	.parent		= &l4_wkup_gclk,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk wdt0_fck = {
	.name		= "wdt0_fck",
	.clkdm_name	= "l4_wkup_clkdm",
	.parent		= &clk_rc32k_ck,
	.clksel		= wdt_clkmux_sel,
	.enable_reg	= AM33XX_CM_WKUP_WDT0_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &followparent_recalc,
};

static struct clk wdt1_ick = {
	.name		= "wdt1_ick",
	.parent		= &l4_wkup_gclk,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk wdt1_fck = {
	.name		= "wdt1_fck",
	.clkdm_name	= "l4_wkup_clkdm",
	.parent		= &clk_rc32k_ck,
	.init		= &omap2_init_clksel_parent,
	.clksel		= wdt_clkmux_sel,
	.enable_reg	= AM33XX_CM_WKUP_WDT1_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clksel_reg	= AM33XX_CLKSEL_WDT1_CLK,
	.clksel_mask	= AM33XX_CLKSEL_0_1_MASK,
	.ops		= &clkops_omap2_dflt,
	.recalc		= &omap2_clksel_recalc,
};

/*
 * Provides clock definitions for enabling bits for Time base module in
 * PWMSS ctrl register.
 */

static struct clk ehrpwm0_tbclk = {
	.name		= "ehrpwm0_tbclk",
	.enable_reg	= AM33XX_CONTROL_PWMSS_CTRL,
	.enable_bit	= AM33XX_PWMSS0_TBCLKEN,
	.ops		= &clkops_omap2_dflt,
	.flags		= ENABLE_ON_INIT,
};

static struct clk ehrpwm1_tbclk = {
	.name		= "ehrpwm1_tbclk",
	.enable_reg	= AM33XX_CONTROL_PWMSS_CTRL,
	.enable_bit	= AM33XX_PWMSS1_TBCLKEN,
	.ops		= &clkops_omap2_dflt,
	.flags		= ENABLE_ON_INIT,
};

static struct clk ehrpwm2_tbclk = {
	.name		= "ehrpwm2_tbclk",
	.enable_reg	= AM33XX_CONTROL_PWMSS_CTRL,
	.enable_bit	= AM33XX_PWMSS2_TBCLKEN,
	.ops		= &clkops_omap2_dflt,
	.flags		= ENABLE_ON_INIT,
};


/*
 * clkdev
 */
static struct omap_clk am33xx_clks[] = {
	CLK(NULL,	"clk_32768_ck",		&clk_32768_ck,	CK_AM33XX),
	CLK(NULL,	"clk_32khz_ck",		&clk_32khz_ck,	CK_AM33XX),
	CLK(NULL,	"clk_rc32k_ck",		&clk_rc32k_ck,	CK_AM33XX),
	CLK(NULL,	"virt_19_2m_ck",	&virt_19_2m_ck,	CK_AM33XX),
	CLK(NULL,	"virt_24m_ck",		&virt_24m_ck,	CK_AM33XX),
	CLK(NULL,	"virt_25m_ck",		&virt_25m_ck,	CK_AM33XX),
	CLK(NULL,	"virt_26m_ck",		&virt_26m_ck,	CK_AM33XX),
	CLK(NULL,	"sys_clkin_ck",		&sys_clkin_ck,	CK_AM33XX),
	CLK(NULL,	"tclkin_ck",		&tclkin_ck,	CK_AM33XX),
	CLK(NULL,	"dpll_core_ck",		&dpll_core_ck,		CK_AM33XX),
	CLK(NULL,	"dpll_core_x2_ck",	&dpll_core_x2_ck,	CK_AM33XX),
	CLK(NULL,	"dpll_core_m4_ck",	&dpll_core_m4_ck,	CK_AM33XX),
	CLK(NULL,	"dpll_core_m5_ck",	&dpll_core_m5_ck,	CK_AM33XX),
	CLK(NULL,	"dpll_core_m6_ck",	&dpll_core_m6_ck,	CK_AM33XX),
	CLK(NULL,	"sysclk1_ck",		&sysclk1_ck,	CK_AM33XX),
	CLK(NULL,	"sysclk2_ck",		&sysclk2_ck,	CK_AM33XX),
	CLK(NULL,	"core_clk_out",		&core_clk_out,	CK_AM33XX),
	CLK(NULL,	"clk_32khz_timer",	&clk_32khz_timer, CK_AM33XX),
	CLK(NULL,	"dpll_mpu_ck",		&dpll_mpu_ck,		CK_AM33XX),
	CLK(NULL,	"dpll_mpu_m2_ck",	&dpll_mpu_m2_ck,	CK_AM33XX),
	CLK(NULL,	"mpu_ck",		&mpu_fck,	CK_AM33XX),
	CLK(NULL,	"dpll_ddr_ck",		&dpll_ddr_ck,		CK_AM33XX),
	CLK(NULL,	"dpll_ddr_m2_ck",	&dpll_ddr_m2_ck,	CK_AM33XX),
	CLK(NULL,	"ddr_pll_clk",		&ddr_pll_clk,	CK_AM33XX),
	CLK(NULL,	"emif_fck",		&emif_fck,	CK_AM33XX),
	CLK(NULL,	"emif_fw_fck",		&emif_fw_fck,	CK_AM33XX),
	CLK(NULL,	"dpll_disp_ck",		&dpll_disp_ck,		CK_AM33XX),
	CLK(NULL,	"dpll_disp_m2_ck",	&dpll_disp_m2_ck,	CK_AM33XX),
	CLK(NULL,	"disp_pll_clk",		&disp_pll_clk,		CK_AM33XX),
	CLK(NULL,	"dpll_per_ck",		&dpll_per_ck,	CK_AM33XX),
	CLK(NULL,	"dpll_per_m2_ck",	&dpll_per_m2_ck,	CK_AM33XX),
	CLK(NULL,	"per_192mhz_clk",	&per_192mhz_clk,	CK_AM33XX),
	CLK(NULL,	"usb_pll_clk",		&usb_pll_clk,		CK_AM33XX),
	CLK(NULL,	"core_100mhz_ck",	&core_100mhz_ck,	CK_AM33XX),
	CLK(NULL,	"l3_ick",		&l3_ick,	CK_AM33XX),
	CLK(NULL,	"l3_instr_ick",		&l3_instr_ick,	CK_AM33XX),
	CLK(NULL,	"adc_tsc_fck",		&adc_tsc_fck,	CK_AM33XX),
	CLK(NULL,	"adc_tsc_ick",		&adc_tsc_ick,	CK_AM33XX),
	CLK(NULL,	"aes0_fck",		&aes0_fck,	CK_AM33XX),
	CLK(NULL,	"l4_cefuse_gclk",	&l4_cefuse_gclk, CK_AM33XX),
	CLK(NULL,	"cefuse_fck",		&cefuse_fck,	CK_AM33XX),
	CLK(NULL,	"cefuse_iclk",		&cefuse_iclk,	CK_AM33XX),
	CLK(NULL,	"clkdiv32k_ick",	&clkdiv32k_ick,	CK_AM33XX),
	CLK(NULL,	"control_fck",		&control_fck,	CK_AM33XX),
	CLK("cpsw.0",	NULL,			&cpgmac0_ick,	CK_AM33XX),
	CLK("d_can.0",	"fck",			&dcan0_fck,	CK_AM33XX),
	CLK("d_can.1",	"fck",			&dcan1_fck,	CK_AM33XX),
	CLK("d_can.0",	"ick",			&dcan0_ick,	CK_AM33XX),
	CLK("d_can.1",	"ick",			&dcan1_ick,	CK_AM33XX),
	CLK(NULL,	"debugss_ick",		&debugss_ick,	CK_AM33XX),
	CLK(NULL,	"elm_fck",		&elm_fck,	CK_AM33XX),
	CLK(NULL,	"epwmss0_fck",		&epwmss0_fck,	CK_AM33XX),
	CLK(NULL,	"epwmss1_fck",		&epwmss1_fck,	CK_AM33XX),
	CLK(NULL,	"epwmss2_fck",		&epwmss2_fck,	CK_AM33XX),
	CLK(NULL,	"gpio0_ick",		&gpio0_ick,	CK_AM33XX),
	CLK(NULL,	"gpio1_ick",		&gpio1_ick,	CK_AM33XX),
	CLK(NULL,	"gpio2_ick",		&gpio2_ick,	CK_AM33XX),
	CLK(NULL,	"gpio3_ick",		&gpio3_ick,	CK_AM33XX),
	CLK(NULL,	"gpmc_fck",		&gpmc_fck,	CK_AM33XX),
	CLK("omap_i2c.1",	"fck",		&i2c1_fck,	CK_AM33XX),
	CLK("omap_i2c.1",	"ick",		&i2c1_ick,	CK_AM33XX),
	CLK("omap_i2c.2",	"fck",		&i2c2_fck,	CK_AM33XX),
	CLK("omap_i2c.2",	"ick",		&i2c2_ick,	CK_AM33XX),
	CLK("omap_i2c.3",	"fck",		&i2c3_fck,	CK_AM33XX),
	CLK("omap_i2c.3",	"ick",		&i2c3_ick,	CK_AM33XX),
	CLK(NULL,	"pruss_ocp_gclk",	&pruss_ocp_gclk,	CK_AM33XX),
	CLK(NULL,	"pruss_uart_gclk",	&pruss_uart_gclk,	CK_AM33XX),
	CLK(NULL,	"pruss_iep_gclk",	&pruss_iep_gclk,	CK_AM33XX),
	CLK(NULL,	"ieee5000_fck",		&ieee5000_fck,	CK_AM33XX),
	CLK(NULL,	"l4hs_ick",		&l4hs_ick,	CK_AM33XX),
	CLK(NULL,	"l4wkup_ick",		&l4wkup_ick,	CK_AM33XX),
	CLK(NULL,	"l4fw_ick",		&l4fw_ick,	CK_AM33XX),
	CLK(NULL,	"l4ls_ick",		&l4ls_ick,	CK_AM33XX),
	CLK("da8xx_lcdc.0",	NULL,		&lcdc_fck,	CK_AM33XX),
	CLK(NULL,	"mailbox0_fck",		&mailbox0_fck,	CK_AM33XX),
	CLK(NULL,	"mcasp1_ick",		&mcasp0_ick,	CK_AM33XX),
	CLK("davinci-mcasp.0",	NULL,		&mcasp0_fck,	CK_AM33XX),
	CLK(NULL,	"mcasp2_ick",		&mcasp1_ick,	CK_AM33XX),
	CLK("davinci-mcasp.1",	NULL,		&mcasp1_fck,	CK_AM33XX),
	CLK(NULL,	"mlb_fck",		&mlb_fck,	CK_AM33XX),
	CLK("omap_hsmmc.0",	"ick",		&mmc0_ick,	CK_AM33XX),
	CLK("omap_hsmmc.1",	"ick",		&mmc1_ick,	CK_AM33XX),
	CLK("omap_hsmmc.2",	"ick",		&mmc2_ick,	CK_AM33XX),
	CLK("omap_hsmmc.0",	"fck",		&mmc0_fck,	CK_AM33XX),
	CLK("omap_hsmmc.1",	"fck",		&mmc1_fck,	CK_AM33XX),
	CLK("omap_hsmmc.2",	"fck",		&mmc2_fck,	CK_AM33XX),
	CLK(NULL,	"mmu_fck",		&mmu_fck,	CK_AM33XX),
	CLK(NULL,	"ocmcram_ick",		&ocmcram_ick,	CK_AM33XX),
	CLK(NULL,	"ocpwp_fck",		&ocpwp_fck,	CK_AM33XX),
	CLK(NULL,	"pka_fck",		&pka_fck,	CK_AM33XX),
	CLK(NULL,	"rng_fck",		&rng_fck,	CK_AM33XX),
	CLK(NULL,	"rtc_fck",		&rtc_fck,	CK_AM33XX),
	CLK(NULL,	"rtc_ick",		&rtc_ick,	CK_AM33XX),
	CLK(NULL,	"sha0_fck",		&sha0_fck,	CK_AM33XX),
	CLK(NULL,	"smartreflex0_fck",	&smartreflex0_fck,	CK_AM33XX),
	CLK(NULL,	"smartreflex0_ick",	&smartreflex0_ick,	CK_AM33XX),
	CLK(NULL,	"smartreflex1_fck",	&smartreflex1_fck,	CK_AM33XX),
	CLK(NULL,	"smartreflex1_ick",	&smartreflex1_ick,	CK_AM33XX),
	CLK("omap2_mcspi.1",	"fck",		&spi0_fck,	CK_AM33XX),
	CLK("omap2_mcspi.2",	"fck",		&spi1_fck,	CK_AM33XX),
	CLK("omap2_mcspi.1",	"ick",		&spi0_ick,	CK_AM33XX),
	CLK("omap2_mcspi.2",	"ick",		&spi1_ick,	CK_AM33XX),
	CLK(NULL,	"spinlock_fck",		&spinlock_fck,	CK_AM33XX),
	CLK(NULL,	"gpt0_fck",		&timer0_fck,	CK_AM33XX),
	CLK(NULL,	"gpt1_fck",		&timer1_fck,	CK_AM33XX),
	CLK(NULL,	"gpt2_fck",		&timer2_fck,	CK_AM33XX),
	CLK(NULL,	"gpt3_fck",		&timer3_fck,	CK_AM33XX),
	CLK(NULL,	"gpt4_fck",		&timer4_fck,	CK_AM33XX),
	CLK(NULL,	"gpt5_fck",		&timer5_fck,	CK_AM33XX),
	CLK(NULL,	"gpt6_fck",		&timer6_fck,	CK_AM33XX),
	CLK(NULL,	"gpt7_fck",		&timer7_fck,	CK_AM33XX),
	CLK("da8xx_lcdc.0",	"lcdc_ick",	&lcdc_ick,	CK_AM33XX),
	CLK(NULL,	"tpcc_ick",		&tpcc_ick,	CK_AM33XX),
	CLK(NULL,	"tptc0_ick",		&tptc0_ick,	CK_AM33XX),
	CLK(NULL,	"tptc1_ick",		&tptc1_ick,	CK_AM33XX),
	CLK(NULL,	"tptc2_ick",		&tptc2_ick,	CK_AM33XX),
	CLK(NULL,	"uart1_fck",		&uart1_fck,	CK_AM33XX),
	CLK(NULL,	"uart2_fck",		&uart2_fck,	CK_AM33XX),
	CLK(NULL,	"uart3_fck",		&uart3_fck,	CK_AM33XX),
	CLK(NULL,	"uart4_fck",		&uart4_fck,	CK_AM33XX),
	CLK(NULL,	"uart5_fck",		&uart5_fck,	CK_AM33XX),
	CLK(NULL,	"uart6_fck",		&uart6_fck,	CK_AM33XX),
	CLK(NULL,	"uart1_ick",		&uart1_ick,	CK_AM33XX),
	CLK(NULL,	"uart2_ick",		&uart2_ick,	CK_AM33XX),
	CLK(NULL,	"uart3_ick",		&uart3_ick,	CK_AM33XX),
	CLK(NULL,	"uart4_ick",		&uart4_ick,	CK_AM33XX),
	CLK(NULL,	"uart5_ick",		&uart5_ick,	CK_AM33XX),
	CLK(NULL,	"uart6_ick",		&uart6_ick,	CK_AM33XX),
	CLK(NULL,	"usbotg_ick",		&usbotg_ick,	CK_AM33XX),
	CLK(NULL,	"usbotg_fck",		&usbotg_fck,	CK_AM33XX),
	CLK(NULL,	"wdt0_ick",		&wdt0_ick,	CK_AM33XX),
	CLK(NULL,	"wdt0_fck",		&wdt0_fck,	CK_AM33XX),
	CLK(NULL,	"wdt1_ick",		&wdt1_ick,	CK_AM33XX),
	CLK(NULL,	"wdt1_fck",		&wdt1_fck,	CK_AM33XX),
	CLK(NULL,	"wkup_m3_fck",		&wkup_m3_fck,	CK_AM33XX),
	CLK(NULL,	"l3_aon_gclk",		&l3_aon_gclk,		CK_AM33XX),
	CLK(NULL,	"l4_wkup_aon_gclk",	&l4_wkup_aon_gclk,	CK_AM33XX),
	CLK(NULL,	"l4_rtc_gclk",		&l4_rtc_gclk,		CK_AM33XX),
	CLK(NULL,	"l3_gclk",		&l3_gclk,		CK_AM33XX),
	CLK(NULL,	"gfx_l3_gclk",		&gfx_l3_gclk,		CK_AM33XX),
	CLK(NULL,	"l4_wkup_gclk",		&l4_wkup_gclk,		CK_AM33XX),
	CLK(NULL,	"l4hs_gclk",		&l4hs_gclk,		CK_AM33XX),
	CLK(NULL,	"l3s_gclk",		&l3s_gclk,		CK_AM33XX),
	CLK(NULL,	"l4fw_gclk",		&l4fw_gclk,		CK_AM33XX),
	CLK(NULL,	"l4ls_gclk",		&l4ls_gclk,		CK_AM33XX),
	CLK(NULL,	"debug_clka_gclk",	&debug_clka_gclk,	CK_AM33XX),
	CLK(NULL,	"clk_24mhz",		&clk_24mhz,		CK_AM33XX),
	CLK(NULL,	"sysclk_div_ck",	&sysclk_div_ck,		CK_AM33XX),
	CLK(NULL,	"cpsw_250mhz_clk",	&cpsw_250mhz_clk,	CK_AM33XX),
	CLK(NULL,	"cpsw_125mhz_gclk",	&cpsw_125mhz_gclk,	CK_AM33XX),
	CLK(NULL,	"cpsw_50mhz_clk",	&cpsw_50mhz_clk,	CK_AM33XX),
	CLK(NULL,	"cpsw_5mhz_clk",	&cpsw_5mhz_clk,		CK_AM33XX),
	CLK(NULL,	"cpsw_cpts_rft_clk",	&cpsw_cpts_rft_clk,	CK_AM33XX),
	CLK(NULL,	"gpio0_dbclk_mux_ck",	&gpio0_dbclk_mux_ck,	CK_AM33XX),
	CLK(NULL,	"gpio0_dbclk",		&gpio0_dbclk,		CK_AM33XX),
	CLK(NULL,	"gpio1_dbclk",		&gpio1_dbclk,		CK_AM33XX),
	CLK(NULL,	"gpio2_dbclk",		&gpio2_dbclk,		CK_AM33XX),
	CLK(NULL,	"gpio3_dbclk",		&gpio3_dbclk,		CK_AM33XX),
	CLK(NULL,	"lcd_gclk",		&lcd_gclk,		CK_AM33XX),
	CLK(NULL,	"mmc_clk",		&mmc_clk,		CK_AM33XX),
	CLK(NULL,	"gfx_fclk_clksel_ck",	&gfx_fclk_clksel_ck,	CK_AM33XX),
	CLK(NULL,	"gfx_fclk",		&gfx_fclk,		CK_AM33XX),
	CLK(NULL,	"gfx_ick",		&gfx_ick,		CK_AM33XX),
	CLK(NULL,	"sysclkout_pre_ck",	&sysclkout_pre_ck,	CK_AM33XX),
	CLK(NULL,	"clkout2_ck",		&clkout2_ck,		CK_AM33XX),
	CLK(NULL,	"gpt0_ick",		&timer0_ick,		CK_AM33XX),
	CLK(NULL,	"gpt1_ick",		&timer1_ick,		CK_AM33XX),
	CLK(NULL,	"gpt2_ick",		&timer2_ick,		CK_AM33XX),
	CLK(NULL,	"gpt3_ick",		&timer3_ick,		CK_AM33XX),
	CLK(NULL,	"gpt4_ick",		&timer4_ick,		CK_AM33XX),
	CLK(NULL,	"gpt5_ick",		&timer5_ick,		CK_AM33XX),
	CLK(NULL,	"gpt6_ick",		&timer6_ick,		CK_AM33XX),
	CLK(NULL,	"gpt7_ick",		&timer7_ick,		CK_AM33XX),
	CLK(NULL,	"vtp_clk",		&vtp_clk,		CK_AM33XX),
	CLK(NULL,	"ehrpwm0_tbclk",	&ehrpwm0_tbclk,	CK_AM33XX),
	CLK(NULL,	"ehrpwm1_tbclk",	&ehrpwm1_tbclk,	CK_AM33XX),
	CLK(NULL,	"ehrpwm2_tbclk",	&ehrpwm2_tbclk,	CK_AM33XX),
};

int __init am33xx_clk_init(void)
{
	struct omap_clk *c;
	u32 cpu_clkflg;

	if (cpu_is_am33xx()) {
		cpu_mask = RATE_IN_AM33XX;
		cpu_clkflg = CK_AM33XX;
	}

	clk_init(&omap2_clk_functions);

	for (c = am33xx_clks; c < am33xx_clks + ARRAY_SIZE(am33xx_clks); c++)
		clk_preinit(c->lk.clk);

	for (c = am33xx_clks; c < am33xx_clks + ARRAY_SIZE(am33xx_clks); c++)
		if (c->cpu & cpu_clkflg) {
			clkdev_add(&c->lk);
			clk_register(c->lk.clk);
			omap2_init_clk_clkdm(c->lk.clk);
		}

	recalculate_root_clocks();

	/*
	 * Only enable those clocks we will need, let the drivers
	 * enable other clocks as necessary
	 */
	clk_enable_init_clocks();

	return 0;
}
