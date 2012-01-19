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

static struct clk clk_32khz_ck = {
	.name		= "clk_32khz_ck",
	.rate		= 32768,
	.ops		= &clkops_null,
};

/* On-Chip 32KHz RC OSC */
static struct clk clk_rc32k_ck = {
	.name		= "clk_rc32k_ck",
	.rate		= 32000,
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

static struct clk sys_clkin_ck = {
	.name		= "sys_clkin_ck",
	.rate		= 24000000,
	.ops		= &clkops_null,
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
	.autoidle_mask	= AM33XX_AUTO_DPLL_MODE_MASK,
	.idlest_mask	= AM33XX_ST_DPLL_CLK_MASK,
	.max_multiplier	= AM33XX_MAX_DPLL_MULT,
	.max_divider	= AM33XX_MAX_DPLL_DIV,
	.min_divider	= 1,
};

static struct clk dpll_per_ck = {
	.name		= "dpll_per_ck",
	.parent		= &sys_clkin_ck,
	.dpll_data	= &dpll_per_dd,
	.init		= &omap2_init_dpll_parent,
	.ops		= &clkops_null,
	.recalc		= &omap3_dpll_recalc,
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

static struct clk i2c_clk = {
	.name		= "i2c_clk",
	.parent		= &dpll_per_m2_ck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk clk_div_24_ck = {
	.name		= "clk_div_24_ck",
	.parent		= &i2c_clk,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
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
	.autoidle_mask	= AM33XX_AUTO_DPLL_MODE_MASK,
	.idlest_mask	= AM33XX_ST_DPLL_CLK_MASK,
	.max_multiplier	= AM33XX_MAX_DPLL_MULT,
	.max_divider	= AM33XX_MAX_DPLL_DIV,
	.min_divider	= 1,
};

static struct clk dpll_core_ck = {
	.name		= "dpll_core_ck",
	.parent		= &sys_clkin_ck,
	.dpll_data	= &dpll_core_dd,
	.init		= &omap2_init_dpll_parent,
	.ops		= &clkops_null,
	.recalc		= &omap3_dpll_recalc,
};

static struct clk dpll_core_x2_ck = {
	.name		= "dpll_core_x2_ck",
	.parent		= &dpll_core_ck,
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

static struct clk sysclk_div_ck = {
	.name		= "sysclk_div_ck",
	.parent		= &dpll_core_m4_ck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk div_l4_wkup_gclk_ck = {
	.name		= "div_l4_wkup_gclk_ck",
	.parent		= &dpll_core_m4_ck,
	.ops		= &clkops_null,
	.fixed_div	= 2,
	.recalc		= &omap_fixed_divisor_recalc,
};

static struct clk core_100m_ck = {
	.name		= "core_100m_ck",
	.parent		= &sysclk_div_ck,
	.ops		= &clkops_null,
	.fixed_div	= 2,
	.recalc		= &omap_fixed_divisor_recalc,
};

static struct clk l4ls_fck = {
	.name		= "l4ls_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_L4LS_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &core_100m_ck,
	.recalc		= &followparent_recalc,
	.flags		= ENABLE_ON_INIT,
};

static struct clk timer2_ick = {
	.name		= "timer2_ick",
	.parent		= &l4ls_fck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk timer3_ick = {
	.name		= "timer3_ick",
	.parent		= &l4ls_fck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk timer4_ick = {
	.name		= "timer4_ick",
	.parent		= &l4ls_fck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk timer5_ick = {
	.name		= "timer5_ick",
	.parent		= &l4ls_fck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk timer6_ick = {
	.name		= "timer6_ick",
	.parent		= &l4ls_fck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk timer7_ick = {
	.name		= "timer7_ick",
	.parent		= &l4ls_fck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk lcdc_l3ick = {
	.name		= "lcdc_ick_l3_clk",
	.enable_reg	= AM33XX_CM_PER_L3_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.parent		= &dpll_core_m4_ck,
	.ops		= &clkops_null,
	.clkdm_name	= "l3_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk lcdc_l4ick = {
	.name		= "lcdc_ick_l4_clk",
	.enable_reg	= AM33XX_CM_PER_L4LS_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.parent		= &dpll_core_m4_ck,
	.ops		= &clkops_null,
	.clkdm_name	= "l4ls_clkdm",
	.recalc		= &followparent_recalc,
};

/* Leaf clocks controlled by modules */
static struct clk adc_tsc_fck = {
	.name		= "adc_tsc_fck",
	.ops		= &clkops_null,
	.parent		= &sys_clkin_ck,
	.clkdm_name	= "l4_wkup_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk adc_tsc_ick = {
	.name		= "adc_tsc_ick",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_WKUP_ADC_TSC_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.parent		= &div_l4_wkup_gclk_ck,
	.recalc		= &followparent_recalc,
};

static struct clk aes0_fck = {
	.name		= "aes0_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_AES0_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l3_clkdm",
	.parent		= &sysclk_div_ck,
	.recalc		= &followparent_recalc,
};

static struct clk cefuse_fck = {
	.name		= "cefuse_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_CEFUSE_CEFUSE_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4_cefuse_clkdm",
	.parent		= &sys_clkin_ck,
	.recalc		= &followparent_recalc,
};

static struct clk clkdiv32k_fck = {
	.name		= "clkdiv32k_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_CLKDIV32K_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "clk_24mhz_clkdm",
	.parent		= &clk_div_24_ck,
	.recalc		= &followparent_recalc,
};

static struct clk control_fck = {
	.name		= "control_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_WKUP_CONTROL_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4_wkup_clkdm",
	.parent		= &div_l4_wkup_gclk_ck,
	.recalc		= &followparent_recalc,
	.flags		= ENABLE_ON_INIT,
};

static struct clk dcan0_fck = {
	.name		= "dcan0_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_DCAN0_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &sys_clkin_ck,
	.recalc		= &followparent_recalc,
};

static struct clk dcan1_fck = {
	.name		= "dcan1_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_DCAN1_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &sys_clkin_ck,
	.recalc		= &followparent_recalc,
};

static struct clk dcan0_ick = {
	.name		= "dcan0_ick",
	.parent		= &dpll_per_m2_ck ,
	.ops		= &clkops_null,
	.clkdm_name	= "l4ls_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk dcan1_ick = {
	.name		= "dcan1_ick",
	.parent		= &dpll_per_m2_ck ,
	.ops		= &clkops_null,
	.clkdm_name	= "l4ls_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk debugss_fck = {
	.name		= "debugss_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_WKUP_DEBUGSS_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l3_aon_clkdm",
	.parent		= &dpll_core_m4_ck,
	.recalc		= &followparent_recalc,
};

static struct clk elm_fck = {
	.name		= "elm_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_ELM_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &core_100m_ck,
	.recalc		= &followparent_recalc,
};

static struct clk emif_fw_fck = {
	.name		= "emif_fw_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_EMIF_FW_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4fw_clkdm",
	.parent		= &core_100m_ck,
	.recalc		= &followparent_recalc,
	.flags		= ENABLE_ON_INIT,
};

static struct clk epwmss0_fck = {
	.name		= "epwmss0_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_EPWMSS0_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &core_100m_ck,
	.recalc		= &followparent_recalc,
};

static struct clk epwmss1_fck = {
	.name		= "epwmss1_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_EPWMSS1_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &core_100m_ck,
	.recalc		= &followparent_recalc,
};

static struct clk epwmss2_fck = {
	.name		= "epwmss2_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_EPWMSS2_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &core_100m_ck,
	.recalc		= &followparent_recalc,
};

static struct clk gpio0_fck = {
	.name		= "gpio0_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_WKUP_GPIO0_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4_wkup_clkdm",
	.parent		= &div_l4_wkup_gclk_ck,
	.recalc		= &followparent_recalc,
};

static struct clk gpio1_fck = {
	.name		= "gpio1_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_GPIO1_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &core_100m_ck,
	.recalc		= &followparent_recalc,
};

static struct clk gpio2_fck = {
	.name		= "gpio2_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_GPIO2_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &core_100m_ck,
	.recalc		= &followparent_recalc,
};

static struct clk gpio3_fck = {
	.name		= "gpio3_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_GPIO3_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &core_100m_ck,
	.recalc		= &followparent_recalc,
};

static struct clk gpmc_fck = {
	.name		= "gpmc_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_GPMC_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l3s_clkdm",
	.parent		= &core_100m_ck,
	.recalc		= &followparent_recalc,
};

static struct clk i2c1_fck = {
	.name		= "i2c1_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_WKUP_I2C0_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4_wkup_clkdm",
	.parent		= &dpll_per_m2_ck,
	.fixed_div	= 4,
	.recalc		= &omap_fixed_divisor_recalc,
};

static struct clk i2c2_fck = {
	.name		= "i2c2_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_I2C1_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &i2c_clk,
	.recalc		= &followparent_recalc,
};

static struct clk i2c3_fck = {
	.name		= "i2c3_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_I2C2_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &i2c_clk,
	.recalc		= &followparent_recalc,
};

static struct clk icss_fck = {
	.name		= "icss_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_ICSS_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "icss_ocp_clkdm",
	.parent		= &dpll_per_m2_ck,
	.recalc		= &followparent_recalc,
};

static struct clk ieee5000_fck = {
	.name		= "ieee5000_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_IEEE5000_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l3s_clkdm",
	.parent		= &core_100m_ck,
	.recalc		= &followparent_recalc,
	.flags		= ENABLE_ON_INIT,
};

static struct clk l3_instr_fck = {
	.name		= "l3_instr_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_L3_INSTR_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l3_clkdm",
	.parent		= &sysclk_div_ck,
	.recalc		= &followparent_recalc,
	.flags		= ENABLE_ON_INIT,
};

static struct clk l3_main_fck = {
	.name		= "l3_main_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_L3_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l3_clkdm",
	.parent		= &sysclk_div_ck,
	.recalc		= &followparent_recalc,
	.flags		= ENABLE_ON_INIT,
};

static struct clk l4_hs_fck = {
	.name		= "l4_hs_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_L4HS_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4hs_clkdm",
	.parent		= &sysclk_div_ck,
	.recalc		= &followparent_recalc,
	.flags		= ENABLE_ON_INIT,
};

static struct clk l4fw_fck = {
	.name		= "l4fw_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_L4FW_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4fw_clkdm",
	.parent		= &core_100m_ck,
	.recalc		= &followparent_recalc,
	.flags		= ENABLE_ON_INIT,
};

static struct clk l4wkup_fck = {
	.name		= "l4wkup_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_L4_WKUP_AON_CLKSTCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4_wkup_aon_clkdm",
	.parent		= &div_l4_wkup_gclk_ck,
	.recalc		= &followparent_recalc,
	.flags		= ENABLE_ON_INIT,
};

static struct clk mailbox0_fck = {
	.name		= "mailbox0_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_MAILBOX0_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &core_100m_ck,
	.recalc		= &followparent_recalc,
};

static struct clk mcasp0_ick = {
	.name		= "mcasp0_ick",
	.parent		= &l3_main_fck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk mcasp1_ick = {
	.name		= "mcasp1_ick",
	.parent		= &l3_main_fck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk mcasp0_fck = {
	.name		= "mcasp0_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_MCASP0_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l3s_clkdm",
	.parent		= &sys_clkin_ck,
	.recalc		= &followparent_recalc,
};

static struct clk mcasp1_fck = {
	.name		= "mcasp1_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_MCASP1_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l3s_clkdm",
	.parent		= &sys_clkin_ck,
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
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_GFX_MMUDATA_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "gfx_l3_clkdm",
	.parent		= &dpll_core_m4_ck,
	.recalc		= &followparent_recalc,
};


static struct clk mstr_exps_fck = {
	.name		= "mstr_exps_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_MSTR_EXPS_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l3_clkdm",
	.parent		= &sysclk_div_ck,
	.recalc		= &followparent_recalc,
};

static struct clk ocmcram_fck = {
	.name		= "ocmcram_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_OCMCRAM_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l3_clkdm",
	.parent		= &sysclk_div_ck,
	.recalc		= &followparent_recalc,
};

static struct clk ocpwp_fck = {
	.name		= "ocpwp_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_OCPWP_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &core_100m_ck,
	.recalc		= &followparent_recalc,
};

static struct clk pka_fck = {
	.name		= "pka_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_PKA_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &core_100m_ck,
	.recalc		= &followparent_recalc,
};

static struct clk rng_fck = {
	.name		= "rng_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_RNG_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &core_100m_ck,
	.recalc		= &followparent_recalc,
};

static struct clk rtc_fck = {
	.name		= "rtc_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_RTC_RTC_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4_rtc_clkdm",
	.parent		= &clk_32khz_ck,
	.recalc		= &followparent_recalc,
};

static struct clk sha0_fck = {
	.name		= "sha0_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_SHA0_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l3_clkdm",
	.parent		= &sysclk_div_ck,
	.recalc		= &followparent_recalc,
};

static struct clk slv_exps_fck = {
	.name		= "slv_exps_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_SLV_EXPS_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l3_clkdm",
	.parent		= &sysclk_div_ck,
	.recalc		= &followparent_recalc,
};

static struct clk smartreflex0_fck = {
	.name		= "smartreflex0_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_WKUP_SMARTREFLEX0_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4_wkup_clkdm",
	.parent		= &sys_clkin_ck,
	.recalc		= &followparent_recalc,
};

static struct clk smartreflex1_fck = {
	.name		= "smartreflex1_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_WKUP_SMARTREFLEX1_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4_wkup_clkdm",
	.parent		= &sys_clkin_ck,
	.recalc		= &followparent_recalc,
};

static struct clk spare0_fck = {
	.name		= "spare0_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_SPARE0_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &core_100m_ck,
	.recalc		= &followparent_recalc,
};

static struct clk spare1_fck = {
	.name		= "spare1_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_SPARE1_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &core_100m_ck,
	.recalc		= &followparent_recalc,
};

static struct clk spi0_fck = {
	.name		= "spi0_fck",
	.parent		= &dpll_per_m2_ck ,
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_SPI0_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4ls_clkdm",
	.fixed_div	= 4,
	.recalc		= &omap_fixed_divisor_recalc,
};

static struct clk spi1_fck = {
	.name		= "spi1_fck",
	.parent		= &dpll_per_m2_ck ,
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_SPI1_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4ls_clkdm",
	.fixed_div	= 4,
	.recalc		= &omap_fixed_divisor_recalc,
};

static struct clk spi0_ick = {
	.name		= "spi0_ick",
	.parent		= &l4ls_fck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk spi1_ick = {
	.name		= "spi1_ick",
	.parent		= &l4ls_fck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk spinlock_fck = {
	.name		= "spinlock_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_SPINLOCK_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &core_100m_ck,
	.recalc		= &followparent_recalc,
};

static const struct clksel timer2_to_7_clk_sel[] = {
	{ .parent = &tclkin_ck, .rates = div_1_0_rates },
	{ .parent = &sys_clkin_ck, .rates = div_1_1_rates },
	{ .parent = &clk_32khz_ck, .rates = div_1_2_rates },
	{ .parent = NULL },
};

static struct clk timer2_fck = {
	.name		= "timer2_fck",
	.parent		= &sys_clkin_ck,
	.init		= &omap2_init_clksel_parent,
	.clksel		= timer2_to_7_clk_sel,
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_TIMER2_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clksel_reg	= AM33XX_CLKSEL_TIMER2_CLK,
	.clksel_mask	= AM33XX_CLKSEL_0_1_MASK,
	.clkdm_name	= "l4ls_clkdm",
	.recalc		= &omap2_clksel_recalc,
};

static struct clk timer3_fck = {
	.name		= "timer3_fck",
	.parent		= &sys_clkin_ck,
	.init		= &am33xx_init_timer_parent,
	.clksel		= timer2_to_7_clk_sel,
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_TIMER3_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clksel_reg	= AM33XX_CLKSEL_TIMER3_CLK,
	.clksel_mask	= AM33XX_CLKSEL_0_1_MASK,
	.clkdm_name	= "l4ls_clkdm",
	.recalc		= &omap2_clksel_recalc,
};

static struct clk timer4_fck = {
	.name		= "timer4_fck",
	.parent		= &sys_clkin_ck,
	.init		= &omap2_init_clksel_parent,
	.clksel		= timer2_to_7_clk_sel,
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_TIMER4_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clksel_reg	= AM33XX_CLKSEL_TIMER4_CLK,
	.clksel_mask	= AM33XX_CLKSEL_0_1_MASK,
	.clkdm_name	= "l4ls_clkdm",
	.recalc		= &omap2_clksel_recalc,
};

static struct clk timer5_fck = {
	.name		= "timer5_fck",
	.parent		= &sys_clkin_ck,
	.init		= &omap2_init_clksel_parent,
	.clksel		= timer2_to_7_clk_sel,
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_TIMER5_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clksel_reg	= AM33XX_CLKSEL_TIMER5_CLK,
	.clksel_mask	= AM33XX_CLKSEL_0_1_MASK,
	.clkdm_name	= "l4ls_clkdm",
	.recalc		= &omap2_clksel_recalc,
};

static struct clk timer6_fck = {
	.name		= "timer6_fck",
	.parent		= &sys_clkin_ck,
	.init		= &am33xx_init_timer_parent,
	.clksel		= timer2_to_7_clk_sel,
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_TIMER6_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clksel_reg	= AM33XX_CLKSEL_TIMER6_CLK,
	.clksel_mask	= AM33XX_CLKSEL_0_1_MASK,
	.clkdm_name	= "l4ls_clkdm",
	.recalc		= &omap2_clksel_recalc,
};

static struct clk timer7_fck = {
	.name		= "timer7_fck",
	.parent		= &sys_clkin_ck,
	.init		= &omap2_init_clksel_parent,
	.clksel		= timer2_to_7_clk_sel,
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_TIMER7_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clksel_reg	= AM33XX_CLKSEL_TIMER7_CLK,
	.clksel_mask	= AM33XX_CLKSEL_0_1_MASK,
	.clkdm_name	= "l4ls_clkdm",
	.recalc		= &omap2_clksel_recalc,
};

static struct clk tpcc_ick = {
	.name		= "tpcc_ick",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_TPCC_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l3_clkdm",
	.parent		= &l3_main_fck,
	.recalc		= &followparent_recalc,
};

static struct clk tptc0_ick = {
	.name		= "tptc0_ick",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_TPTC0_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l3_clkdm",
	.parent		= &l3_main_fck,
	.recalc		= &followparent_recalc,
};

static struct clk tptc1_ick = {
	.name		= "tptc1_ick",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_TPTC1_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l3_clkdm",
	.parent		= &l3_main_fck,
	.recalc		= &followparent_recalc,
};

static struct clk tptc2_ick = {
	.name		= "tptc2_ick",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_TPTC2_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l3_clkdm",
	.parent		= &l3_main_fck,
	.recalc		= &followparent_recalc,
};

static struct clk uart1_fck = {
	.name		= "uart1_fck",
	.parent		= &dpll_per_m2_ck ,
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_WKUP_UART0_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4_wkup_clkdm",
	.fixed_div	= 4,
	.recalc		= &omap_fixed_divisor_recalc,
};

static struct clk uart2_fck = {
	.name		= "uart2_fck",
	.parent		= &dpll_per_m2_ck ,
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_UART1_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4ls_clkdm",
	.fixed_div	= 4,
	.recalc		= &omap_fixed_divisor_recalc,
};

static struct clk uart3_fck = {
	.name		= "uart3_fck",
	.parent		= &dpll_per_m2_ck ,
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_UART2_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4ls_clkdm",
	.fixed_div	= 4,
	.recalc		= &omap_fixed_divisor_recalc,
};

static struct clk uart4_fck = {
	.name		= "uart4_fck",
	.parent		= &dpll_per_m2_ck ,
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_UART3_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4ls_clkdm",
	.fixed_div	= 4,
	.recalc		= &omap_fixed_divisor_recalc,
};

static struct clk uart5_fck = {
	.name		= "uart5_fck",
	.parent		= &dpll_per_m2_ck ,
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_UART4_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4ls_clkdm",
	.fixed_div	= 4,
	.recalc		= &omap_fixed_divisor_recalc,
};

static struct clk uart6_fck = {
	.name		= "uart6_fck",
	.parent		= &dpll_per_m2_ck ,
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_UART5_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4ls_clkdm",
	.fixed_div	= 4,
	.recalc		= &omap_fixed_divisor_recalc,
};

static struct clk uart1_ick = {
	.name		= "uart1_ick",
	.parent		= &div_l4_wkup_gclk_ck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk uart2_ick = {
	.name		= "uart2_ick",
	.parent		= &l4ls_fck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk uart3_ick = {
	.name		= "uart3_ick",
	.parent		= &l4ls_fck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk uart4_ick = {
	.name		= "uart4_ick",
	.parent		= &l4ls_fck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk uart5_ick = {
	.name		= "uart5_ick",
	.parent		= &l4ls_fck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk uart6_ick = {
	.name		= "uart6_ick",
	.parent		= &l4ls_fck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk wkup_m3_fck = {
	.name		= "wkup_m3_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_WKUP_WKUP_M3_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4_wkup_aon_clkdm",
	.parent		= &div_l4_wkup_gclk_ck,
	.recalc		= &followparent_recalc,
};

static struct clk dpll_core_m5_ck = {
	.name		= "dpll_core_m5_ck",
	.parent		= &dpll_core_x2_ck,
	.clksel		= dpll_core_m4_div,
	.clksel_reg	= AM33XX_CM_DIV_M5_DPLL_CORE,
	.clksel_mask	= AM33XX_HSDIVIDER_CLKOUT2_DIV_MASK,
	.ops		= &clkops_null,
	.recalc		= &omap2_clksel_recalc,
	.round_rate	= &omap2_clksel_round_rate,
	.set_rate	= &omap2_clksel_set_rate,
};

static struct clk cpsw_250m_clkdiv_ck = {
	.name		= "cpsw_250m_clkdiv_ck",
	.parent		= &dpll_core_m5_ck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk cpsw_125mhz_ocp_ck = {
	.name		= "cpsw_125mhz_ocp_ck",
	.parent		= &dpll_core_m5_ck,
	.ops		= &clkops_null,
	.fixed_div	= 2,
	.recalc		= &omap_fixed_divisor_recalc,
};

static struct clk cpsw_50m_clkdiv_ck = {
	.name		= "cpsw_50m_clkdiv_ck",
	.parent		= &dpll_core_m5_ck,
	.ops		= &clkops_null,
	.fixed_div	= 5,
	.recalc		= &omap_fixed_divisor_recalc,
};

static struct clk cpgmac0_fck = {
	.name		= "cpgmac0_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_CPGMAC0_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "cpsw_125mhz_clkdm",
	.parent		= &cpsw_125mhz_ocp_ck,
	.recalc		= &followparent_recalc,
};

static struct clk cpsw_5m_clkdiv_ck = {
	.name		= "cpsw_5m_clkdiv_ck",
	.parent		= &cpsw_50m_clkdiv_ck,
	.ops		= &clkops_null,
	.fixed_div	= 10,
	.recalc		= &omap_fixed_divisor_recalc,
};


static const struct clksel cpts_rft_clkmux_sel[] = {
	{ .parent = &dpll_core_m5_ck, .rates = div_1_0_rates },
	{ .parent = &dpll_core_m4_ck, .rates = div_1_1_rates },
	{ .parent = NULL },
};

static struct clk cpts_rft_clkmux_ck = {
	.name		= "cpts_rft_clkmux_ck",
	.parent		= &dpll_core_m5_ck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
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
	.autoidle_mask	= AM33XX_AUTO_DPLL_MODE_MASK,
	.idlest_mask	= AM33XX_ST_DPLL_CLK_MASK,
	.max_multiplier	= AM33XX_MAX_DPLL_MULT,
	.max_divider	= AM33XX_MAX_DPLL_DIV,
	.min_divider	= 1,
};


static struct clk dpll_ddr_ck = {
	.name		= "dpll_ddr_ck",
	.parent		= &sys_clkin_ck,
	.dpll_data	= &dpll_ddr_dd,
	.init		= &omap2_init_dpll_parent,
	.ops		= &clkops_null,
	.recalc		= &omap3_dpll_recalc,
};

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

static struct clk ddr_pll_div_clk = {
	.name		= "ddr_pll_div_clk",
	.parent		= &dpll_ddr_m2_ck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk emif_fck = {
	.name		= "emif_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_EMIF_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l3_clkdm",
	.parent		= &ddr_pll_div_clk,
	.recalc		= &followparent_recalc,
	.flags		= ENABLE_ON_INIT,
};

static struct clk div_l4_rtc_gclk_ck = {
	.name		= "div_l4_rtc_gclk_ck",
	.parent		= &dpll_core_m4_ck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
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
	.autoidle_mask	= AM33XX_AUTO_DPLL_MODE_MASK,
	.idlest_mask	= AM33XX_ST_DPLL_CLK_MASK,
	.max_multiplier	= AM33XX_MAX_DPLL_MULT,
	.max_divider	= AM33XX_MAX_DPLL_DIV,
	.min_divider	= 1,
};

static struct clk dpll_disp_ck = {
	.name		= "dpll_disp_ck",
	.parent		= &sys_clkin_ck,
	.dpll_data	= &dpll_disp_dd,
	.init		= &omap2_init_dpll_parent,
	.ops		= &clkops_omap3_noncore_dpll_ops,
	.recalc		= &omap3_dpll_recalc,
	.round_rate	= &omap2_dpll_round_rate,
	.set_rate	= &omap3_noncore_dpll_set_rate,
};

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
	.autoidle_mask	= AM33XX_AUTO_DPLL_MODE_MASK,
	.idlest_mask	= AM33XX_ST_DPLL_CLK_MASK,
	.max_multiplier	= AM33XX_MAX_DPLL_MULT,
	.max_divider	= AM33XX_MAX_DPLL_DIV,
	.min_divider	= 1,
};

static struct clk dpll_mpu_ck = {
	.name		= "dpll_mpu_ck",
	.parent		= &sys_clkin_ck,
	.dpll_data	= &dpll_mpu_dd,
	.init		= &omap2_init_dpll_parent,
	.ops		= &clkops_omap3_noncore_dpll_ops,
	.recalc		= &omap3_dpll_recalc,
	.round_rate     = &omap2_dpll_round_rate,
	.set_rate       = &omap3_noncore_dpll_set_rate,
};


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
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_MPU_MPU_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "mpu_clkdm",
	.parent		= &dpll_mpu_m2_ck,
	.recalc		= &followparent_recalc,
};

static struct clk dpll_per_clkdcoldo_ck = {
	.name		= "dpll_per_clkdcoldo_ck",
	.parent		= &dpll_per_ck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};


static const struct clksel gpio_dbclk_mux_sel[] = {
	{ .parent = &clk_rc32k_ck, .rates = div_1_0_rates },
	{ .parent = &clk_32768_ck, .rates = div_1_1_rates },
	{ .parent = &clk_32khz_ck, .rates = div_1_2_rates },
	{ .parent = NULL },
};

static struct clk usbotg_ick = {
	.name		= "usbotg_ick",
	.parent		= &core_100m_ck,
	.ops		= &clkops_omap2_dflt,
	.clkdm_name	= "l3s_clkdm",
	.enable_reg	= AM33XX_CM_PER_USB0_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.recalc		= &followparent_recalc,
};

static struct clk usbotg_fck = {
	.name		= "usbotg_fck",
	.ops		= &clkops_omap2_dflt,
	.clkdm_name	= "wkup_usb_clkdm",
	.enable_reg	= AM33XX_CM_CLKDCOLDO_DPLL_PER,
	.enable_bit	= AM33XX_ST_DPLL_CLKDCOLDO_SHIFT,
	.parent		= &dpll_per_clkdcoldo_ck,
	.recalc		= &followparent_recalc,
};

static struct clk gpio_dbclk_mux_ck = {
	.name		= "gpio_dbclk_mux_ck",
	.parent		= &sys_clkin_ck,
	.init		= &omap2_init_clksel_parent,
	.clksel		= gpio_dbclk_mux_sel,
	.ops		= &clkops_null,
	.clksel_reg	= AM33XX_CLKSEL_GPIO0_DBCLK,
	.clksel_mask	= (3 << 0),
	.clkdm_name	= "l4_wkup_clkdm",
	.recalc		= &omap2_clksel_recalc,
};

static struct clk gpio0_dbclk = {
	.name		= "gpio0_dbclk",
	.parent		= &gpio_dbclk_mux_ck,
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_WKUP_GPIO0_CLKCTRL,
	.enable_bit	= AM33XX_OPTFCLKEN_GPIO0_GDBCLK_SHIFT,
	.clkdm_name	= "l4_wkup_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk gpio1_dbclk = {
	.name		= "gpio1_dbclk",
	.parent		= &clkdiv32k_fck,
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_GPIO1_CLKCTRL,
	.enable_bit	= AM33XX_OPTFCLKEN_GPIO_1_GDBCLK_SHIFT,
	.clkdm_name	= "l4ls_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk gpio2_dbclk = {
	.name		= "gpio2_dbclk",
	.parent		= &clkdiv32k_fck,
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_GPIO2_CLKCTRL,
	.enable_bit	= AM33XX_OPTFCLKEN_GPIO_2_GDBCLK_SHIFT,
	.clkdm_name	= "l4ls_clkdm",
	.recalc		= &followparent_recalc,
};

static struct clk gpio3_dbclk = {
	.name		= "gpio3_dbclk",
	.parent		= &clkdiv32k_fck,
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_GPIO3_CLKCTRL,
	.enable_bit	= AM33XX_OPTFCLKEN_GPIO_3_GDBCLK_SHIFT,
	.clkdm_name	= "l4ls_clkdm",
	.recalc		= &followparent_recalc,
};

static const struct clksel icss_ocp_clk_mux_sel[] = {
	{ .parent = &sysclk_div_ck, .rates = div_1_0_rates },
	{ .parent = &dpll_disp_m2_ck, .rates = div_1_1_rates },
	{ .parent = NULL },
};

static struct clk icss_ocp_clk_mux_ck = {
	.name		= "icss_ocp_clk_mux_ck",
	.parent		= &sysclk_div_ck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};


static const struct clksel lcd_clk_mux_sel[] = {
	{ .parent = &dpll_disp_m2_ck, .rates = div_1_0_rates },
	{ .parent = &dpll_core_m5_ck, .rates = div_1_1_rates },
	{ .parent = &dpll_per_m2_ck, .rates = div_1_2_rates },
	{ .parent = NULL },
};

static struct clk lcd_clk_mux_ck = {
	.name		= "lcd_clk_mux_ck",
	.parent		= &dpll_disp_m2_ck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk lcdc_fck = {
	.name		= "lcdc_fck",
	.ops		= &clkops_omap2_dflt,
	.init		= &omap2_init_clksel_parent,
	.clksel		= lcd_clk_mux_sel,
	.enable_reg	= AM33XX_CM_PER_LCDC_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clksel_reg	= AM33XX_CLKSEL_LCDC_PIXEL_CLK,
	.clksel_mask	= AM33XX_CLKSEL_0_1_MASK,
	.clkdm_name	= "lcdc_clkdm",
	.parent		= &dpll_disp_m2_ck,
	.recalc		= &followparent_recalc,
};

static struct clk mmc0_ick = {
	.name		= "mmc0_ick",
	.parent		= &l4ls_fck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk mmc1_ick = {
	.name		= "mmc1_ick",
	.parent		= &l4ls_fck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk mmc2_ick = {
	.name		= "mmc2_ick",
	.parent		= &l3_main_fck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk mmc_clk = {
	.name		= "mmc_clk",
	.parent		= &dpll_per_m2_ck,
	.ops		= &clkops_null,
	.fixed_div	= 2,
	.recalc		= &omap_fixed_divisor_recalc,
};

static struct clk mmc0_fck = {
	.name		= "mmc0_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_MMC0_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &mmc_clk,
	.recalc		= &followparent_recalc,
};

static struct clk mmc1_fck = {
	.name		= "mmc1_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_MMC1_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4ls_clkdm",
	.parent		= &mmc_clk,
	.recalc		= &followparent_recalc,
};

static struct clk mmc2_fck = {
	.name		= "mmc2_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_PER_MMC2_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l3s_clkdm",
	.parent		= &mmc_clk,
	.recalc		= &followparent_recalc,
};

static const struct clksel sgx_clksel_sel[] = {
	{ .parent = &dpll_core_m4_ck, .rates = div_1_0_rates },
	{ .parent = &dpll_per_m2_ck, .rates = div_1_1_rates },
	{ .parent = NULL },
};

static struct clk sgx_clksel_ck = {
	.name		= "sgx_clksel_ck",
	.parent		= &dpll_core_m4_ck,
	.clksel		= sgx_clksel_sel,
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

static const struct clksel sgx_div_sel[] = {
	{ .parent = &sgx_clksel_ck, .rates = div_1_0_2_1_rates },
	{ .parent = NULL },
};

static struct clk sgx_ck = {
	.name		= "sgx_ck",
	.parent		= &sgx_clksel_ck,
	.clksel		= sgx_div_sel,
	.ops		= &clkops_null,
	.enable_reg	= AM33XX_CM_GFX_GFX_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clksel_reg	= AM33XX_CLKSEL_GFX_FCLK,
	.clksel_mask	= AM33XX_CLKSEL_0_0_MASK,
	.clkdm_name	= "gfx_l3_clkdm",
	.recalc		= &omap2_clksel_recalc,
	.round_rate	= &omap2_clksel_round_rate,
	.set_rate	= &omap2_clksel_set_rate,
};

static const struct clksel sysclkout_pre_sel[] = {
	{ .parent = &clk_32768_ck, .rates = div_1_0_rates },
	{ .parent = &sysclk_div_ck, .rates = div_1_1_rates },
	{ .parent = &dpll_ddr_m2_ck, .rates = div_1_2_rates },
	{ .parent = &dpll_per_m2_ck, .rates = div_1_3_rates },
	{ .parent = &lcd_clk_mux_ck, .rates = div_1_4_rates },
	{ .parent = NULL },
};

static struct clk sysclkout_pre_ck = {
	.name		= "sysclkout_pre_ck",
	.init		= &omap2_init_clksel_parent,
	.ops		= &clkops_null,
	.clksel		= sysclkout_pre_sel,
	.clksel_reg	= AM33XX_CM_CLKOUT_CTRL,
	.clksel_mask	= AM33XX_CLKOUT2SOURCE_MASK,
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

static const struct clksel timer0_clkmux_sel[] = {
	{ .parent = &clk_rc32k_ck, .rates = div_1_0_rates },
	{ .parent = &clk_32khz_ck, .rates = div_1_1_rates },
	{ .parent = &sys_clkin_ck, .rates = div_1_2_rates },
	{ .parent = &tclkin_ck, .rates = div_1_3_rates },
	{ .parent = NULL },
};

static struct clk timer0_clkmux_ck = {
	.name		= "timer0_clkmux_ck",
	.parent		= &clk_rc32k_ck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk timer0_ick = {
	.name		= "timer0_ick",
	.parent		= &div_l4_wkup_gclk_ck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};


static struct clk timer0_fck = {
	.name		= "timer0_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_WKUP_TIMER0_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4_wkup_clkdm",
	.parent		= &timer0_clkmux_ck,
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
	.parent		= &div_l4_wkup_gclk_ck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
};

static struct clk timer1_fck = {
	.name		= "timer1_fck",
	.parent		= &sys_clkin_ck,
	.init		= &omap2_init_clksel_parent,
	.clksel		= timer1_clkmux_sel,
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_WKUP_TIMER1_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clksel_reg	= AM33XX_CLKSEL_TIMER1MS_CLK,
	.clksel_mask	= AM33XX_CLKSEL_0_2_MASK,
	.clkdm_name	= "l4ls_clkdm",
	.recalc		= &omap2_clksel_recalc,
};

static struct clk vtp_clk_div_ck = {
	.name		= "vtp_clk_div_ck",
	.parent		= &sys_clkin_ck,
	.ops		= &clkops_null,
	.recalc		= &followparent_recalc,
	.flags		= ENABLE_ON_INIT,
};



static const struct clksel wdt0_clkmux_sel[] = {
	{ .parent = &clk_rc32k_ck, .rates = div_1_0_rates },
	{ .parent = &clk_32khz_ck, .rates = div_1_1_rates },
	{ .parent = NULL },
};

static struct clk wdt0_clkmux_ck = {
	.name		= "wdt0_clkmux_ck",
	.parent		= &clk_32khz_ck,
	.ops		= &clkops_null,
	.clksel_reg	= AM33XX_CM_DIV_M2_DPLL_PER,
	.clksel_mask	= AM33XX_DPLL_CLKOUT_DIV_MASK,
	.recalc		= &followparent_recalc,
};

static struct clk wd_timer1_fck = {
	.name		= "wd_timer1_fck",
	.init		= &omap2_init_clksel_parent,
	.clksel		= wdt0_clkmux_sel,
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_WKUP_WDT1_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clksel_reg	= AM33XX_CLKSEL_WDT1_CLK,
	.clksel_mask	= AM33XX_CLKSEL_0_1_MASK,
	.clkdm_name	= "l4_wkup_clkdm",
	.recalc		= &omap2_clksel_recalc,
};

static struct clk wdt0_fck = {
	.name		= "wdt0_fck",
	.ops		= &clkops_omap2_dflt,
	.enable_reg	= AM33XX_CM_WKUP_WDT0_CLKCTRL,
	.enable_bit	= AM33XX_MODULEMODE_SWCTRL,
	.clkdm_name	= "l4_wkup_clkdm",
	.parent		= &wdt0_clkmux_ck,
	.recalc		= &followparent_recalc,
};

/*
 * clkdev
 */
static struct omap_clk am33xx_clks[] = {
	CLK(NULL,	"clk_32768_ck",		&clk_32768_ck,	CK_AM33XX),
	CLK(NULL,	"clk_32khz_ck",		&clk_32khz_ck,	CK_AM33XX),
	CLK(NULL,	"clk_rc32k_ck",		&clk_rc32k_ck,	CK_AM33XX),
	CLK(NULL,	"sys_clkin_ck",		&sys_clkin_ck,	CK_AM33XX),
	CLK(NULL,	"tclkin_ck",		&tclkin_ck,	CK_AM33XX),
	CLK(NULL,	"adc_tsc_fck",		&adc_tsc_fck,	CK_AM33XX),
	CLK(NULL,	"adc_tsc_ick",		&adc_tsc_ick,	CK_AM33XX),
	CLK(NULL,	"aes0_fck",		&aes0_fck,	CK_AM33XX),
	CLK(NULL,	"cefuse_fck",		&cefuse_fck,	CK_AM33XX),
	CLK(NULL,	"clkdiv32k_fck",	&clkdiv32k_fck,	CK_AM33XX),
	CLK(NULL,	"control_fck",		&control_fck,	CK_AM33XX),
	CLK("cpsw.0",	NULL,			&cpgmac0_fck,	CK_AM33XX),
	CLK(NULL,	"dcan0_fck",		&dcan0_fck,	CK_AM33XX),
	CLK(NULL,	"dcan1_fck",		&dcan1_fck,	CK_AM33XX),
	CLK(NULL,	"dcan0_ick",		&dcan0_ick,	CK_AM33XX),
	CLK(NULL,	"dcan1_ick",		&dcan1_ick,	CK_AM33XX),
	CLK(NULL,	"debugss_fck",		&debugss_fck,	CK_AM33XX),
	CLK(NULL,	"elm_fck",		&elm_fck,	CK_AM33XX),
	CLK(NULL,	"emif_fck",		&emif_fck,	CK_AM33XX),
	CLK(NULL,	"emif_fw_fck",		&emif_fw_fck,	CK_AM33XX),
	CLK(NULL,	"epwmss0_fck",		&epwmss0_fck,	CK_AM33XX),
	CLK(NULL,	"epwmss1_fck",		&epwmss1_fck,	CK_AM33XX),
	CLK(NULL,	"epwmss2_fck",		&epwmss2_fck,	CK_AM33XX),
	CLK(NULL,	"gpio0_fck",		&gpio0_fck,	CK_AM33XX),
	CLK(NULL,	"gpio1_fck",		&gpio1_fck,	CK_AM33XX),
	CLK(NULL,	"gpio2_fck",		&gpio2_fck,	CK_AM33XX),
	CLK(NULL,	"gpio3_fck",		&gpio3_fck,	CK_AM33XX),
	CLK(NULL,	"gpmc_fck",		&gpmc_fck,	CK_AM33XX),
	CLK("omap_i2c.1",	"fck",		&i2c1_fck,	CK_AM33XX),
	CLK("omap_i2c.2",	"fck",		&i2c2_fck,	CK_AM33XX),
	CLK("omap_i2c.3",	"fck",		&i2c3_fck,	CK_AM33XX),
	CLK(NULL,	"icss_fck",		&icss_fck,	CK_AM33XX),
	CLK(NULL,	"ieee5000_fck",		&ieee5000_fck,	CK_AM33XX),
	CLK(NULL,	"l3_instr_fck",		&l3_instr_fck,	CK_AM33XX),
	CLK(NULL,	"l3_main_fck",		&l3_main_fck,	CK_AM33XX),
	CLK(NULL,	"l4_hs_fck",		&l4_hs_fck,	CK_AM33XX),
	CLK(NULL,	"l4fw_fck",		&l4fw_fck,	CK_AM33XX),
	CLK(NULL,	"l4ls_fck",		&l4ls_fck,	CK_AM33XX),
	CLK(NULL,	"l4wkup_fck",		&l4wkup_fck,	CK_AM33XX),
	CLK("da8xx_lcdc.0",	NULL,		&lcdc_fck,	CK_AM33XX),
	CLK(NULL,	"mailbox0_fck",		&mailbox0_fck,	CK_AM33XX),
	CLK(NULL,	"mcasp1_ick",		&mcasp0_ick,	CK_AM33XX),
	CLK(NULL,	"mcasp2_ick",		&mcasp1_ick,	CK_AM33XX),
	CLK("davinci-mcasp.0",	NULL,		&mcasp0_fck,	CK_AM33XX),
	CLK("davinci-mcasp.1",	NULL,		&mcasp1_fck,	CK_AM33XX),
	CLK(NULL,	"mlb_fck",		&mlb_fck,	CK_AM33XX),
	CLK("omap_hsmmc.0",	"ick",		&mmc0_ick,	CK_AM33XX),
	CLK("omap_hsmmc.1",	"ick",		&mmc1_ick,	CK_AM33XX),
	CLK("omap_hsmmc.2",	"ick",		&mmc2_ick,	CK_AM33XX),
	CLK("omap_hsmmc.0",	"fck",		&mmc0_fck,	CK_AM33XX),
	CLK("omap_hsmmc.1",	"fck",		&mmc1_fck,	CK_AM33XX),
	CLK("omap_hsmmc.2",	"fck",		&mmc2_fck,	CK_AM33XX),
	CLK(NULL,	"mmu_fck",		&mmu_fck,	CK_AM33XX),
	CLK(NULL,	"mpu_ck",		&mpu_fck,	CK_AM33XX),
	CLK(NULL,	"mstr_exps_fck",	&mstr_exps_fck,	CK_AM33XX),
	CLK(NULL,	"ocmcram_fck",		&ocmcram_fck,	CK_AM33XX),
	CLK(NULL,	"ocpwp_fck",		&ocpwp_fck,	CK_AM33XX),
	CLK(NULL,	"pka_fck",		&pka_fck,	CK_AM33XX),
	CLK(NULL,	"rng_fck",		&rng_fck,	CK_AM33XX),
	CLK(NULL,	"rtc_fck",		&rtc_fck,	CK_AM33XX),
	CLK(NULL,	"sha0_fck",		&sha0_fck,	CK_AM33XX),
	CLK(NULL,	"slv_exps_fck",		&slv_exps_fck,	CK_AM33XX),
	CLK(NULL,	"smartreflex0_fck",	&smartreflex0_fck,	CK_AM33XX),
	CLK(NULL,	"smartreflex1_fck",	&smartreflex1_fck,	CK_AM33XX),
	CLK(NULL,	"spare0_fck",		&spare0_fck,	CK_AM33XX),
	CLK(NULL,	"spare1_fck",		&spare1_fck,	CK_AM33XX),
	CLK("omap2_mcspi.1",	"fck",		&spi0_fck,	CK_AM33XX),
	CLK("omap2_mcspi.2",	"fck",		&spi1_fck,	CK_AM33XX),
	CLK("omap2_mcspi.1",	"ick",		&spi0_ick,	CK_AM33XX),
	CLK("omap2_mcspi.2",	"ick",		&spi1_ick,	CK_AM33XX),
	CLK(NULL,	"spinlock_fck",		&spinlock_fck,	CK_AM33XX),
	CLK(NULL,	"timer0_fck",		&timer0_fck,	CK_AM33XX),
	CLK(NULL,	"gpt1_fck",		&timer1_fck,	CK_AM33XX),
	CLK(NULL,	"gpt2_fck",		&timer2_fck,	CK_AM33XX),
	CLK(NULL,	"gpt3_fck",		&timer3_fck,	CK_AM33XX),
	CLK(NULL,	"gpt4_fck",		&timer4_fck,	CK_AM33XX),
	CLK(NULL,	"gpt5_fck",		&timer5_fck,	CK_AM33XX),
	CLK(NULL,	"gpt6_fck",		&timer6_fck,	CK_AM33XX),
	CLK(NULL,	"gpt7_fck",		&timer7_fck,	CK_AM33XX),
	CLK(NULL,	"lcdc_ick_l3_clk",	&lcdc_l3ick,	CK_AM33XX),
	CLK(NULL,	"lcdc_ick_l4_clk",	&lcdc_l4ick,	CK_AM33XX),
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
	CLK(NULL,	"wd_timer1_fck",	&wd_timer1_fck,	CK_AM33XX),
	CLK(NULL,	"wdt0_fck",		&wdt0_fck,	CK_AM33XX),
	CLK(NULL,	"wkup_m3_fck",		&wkup_m3_fck,	CK_AM33XX),
	CLK(NULL,	"dpll_per_ck",		&dpll_per_ck,	CK_AM33XX),
	CLK(NULL,	"dpll_per_m2_ck",	&dpll_per_m2_ck,	CK_AM33XX),
	CLK(NULL,	"i2c_clk",		&i2c_clk,		CK_AM33XX),
	CLK(NULL,	"clk_div_24_ck",	&clk_div_24_ck,		CK_AM33XX),
	CLK(NULL,	"dpll_core_ck",		&dpll_core_ck,		CK_AM33XX),
	CLK(NULL,	"dpll_core_x2_ck",	&dpll_core_x2_ck,	CK_AM33XX),
	CLK(NULL,	"dpll_core_m4_ck",	&dpll_core_m4_ck,	CK_AM33XX),
	CLK(NULL,	"sysclk_div_ck",	&sysclk_div_ck,		CK_AM33XX),
	CLK(NULL,	"core_100m_ck",		&core_100m_ck,		CK_AM33XX),
	CLK(NULL,	"dpll_core_m5_ck",	&dpll_core_m5_ck,	CK_AM33XX),
	CLK(NULL,	"cpsw_250m_clkdiv_ck",	&cpsw_250m_clkdiv_ck,	CK_AM33XX),
	CLK(NULL,	"cpsw_125mhz_ocp_ck",	&cpsw_125mhz_ocp_ck,	CK_AM33XX),
	CLK(NULL,	"cpsw_50m_clkdiv_ck",	&cpsw_50m_clkdiv_ck,	CK_AM33XX),
	CLK(NULL,	"cpsw_5m_clkdiv_ck",	&cpsw_5m_clkdiv_ck,	CK_AM33XX),
	CLK(NULL,	"cpts_rft_clkmux_ck",	&cpts_rft_clkmux_ck,	CK_AM33XX),
	CLK(NULL,	"dpll_ddr_ck",		&dpll_ddr_ck,		CK_AM33XX),
	CLK(NULL,	"dpll_ddr_m2_ck",	&dpll_ddr_m2_ck,	CK_AM33XX),
	CLK(NULL,	"ddr_pll_div_clk",	&ddr_pll_div_clk,	CK_AM33XX),
	CLK(NULL,	"div_l4_rtc_gclk_ck",	&div_l4_rtc_gclk_ck,	CK_AM33XX),
	CLK(NULL,	"div_l4_wkup_gclk_ck",	&div_l4_wkup_gclk_ck,	CK_AM33XX),
	CLK(NULL,	"dpll_disp_ck",		&dpll_disp_ck,		CK_AM33XX),
	CLK(NULL,	"dpll_disp_m2_ck",	&dpll_disp_m2_ck,	CK_AM33XX),
	CLK(NULL,	"dpll_mpu_ck",		&dpll_mpu_ck,		CK_AM33XX),
	CLK(NULL,	"dpll_mpu_m2_ck",	&dpll_mpu_m2_ck,	CK_AM33XX),
	CLK(NULL,	"dpll_per_clkdcoldo_ck", &dpll_per_clkdcoldo_ck,	CK_AM33XX),
	CLK(NULL,	"gpio_dbclk_mux_ck",	&gpio_dbclk_mux_ck,	CK_AM33XX),
	CLK(NULL,	"gpio0_dbclk",		&gpio0_dbclk,		CK_AM33XX),
	CLK(NULL,	"gpio1_dbclk",		&gpio1_dbclk,		CK_AM33XX),
	CLK(NULL,	"gpio2_dbclk",		&gpio2_dbclk,		CK_AM33XX),
	CLK(NULL,	"gpio3_dbclk",		&gpio3_dbclk,		CK_AM33XX),
	CLK(NULL,	"icss_ocp_clk_mux_ck",	&icss_ocp_clk_mux_ck,	CK_AM33XX),
	CLK(NULL,	"lcd_clk_mux_ck",	&lcd_clk_mux_ck,	CK_AM33XX),
	CLK(NULL,	"mmc_clk",		&mmc_clk,		CK_AM33XX),
	CLK(NULL,	"sgx_clksel_ck",	&sgx_clksel_ck,		CK_AM33XX),
	CLK(NULL,	"sgx_ck",		&sgx_ck,		CK_AM33XX),
	CLK(NULL,	"sysclkout_pre_ck",	&sysclkout_pre_ck,	CK_AM33XX),
	CLK(NULL,	"clkout2_ck",		&clkout2_ck,		CK_AM33XX),
	CLK(NULL,	"timer0_clkmux_ck",	&timer0_clkmux_ck,	CK_AM33XX),
	CLK(NULL,	"gpt0_ick",		&timer0_ick,		CK_AM33XX),
	CLK(NULL,	"gpt1_ick",		&timer1_ick,		CK_AM33XX),
	CLK(NULL,	"gpt2_ick",		&timer2_ick,		CK_AM33XX),
	CLK(NULL,	"gpt3_ick",		&timer3_ick,		CK_AM33XX),
	CLK(NULL,	"gpt4_ick",		&timer4_ick,		CK_AM33XX),
	CLK(NULL,	"gpt5_ick",		&timer5_ick,		CK_AM33XX),
	CLK(NULL,	"gpt6_ick",		&timer6_ick,		CK_AM33XX),
	CLK(NULL,	"gpt7_ick",		&timer7_ick,		CK_AM33XX),
	CLK(NULL,	"vtp_clk_div_ck",	&vtp_clk_div_ck,	CK_AM33XX),
	CLK(NULL,	"wdt0_clkmux_ck",	&wdt0_clkmux_ck,	CK_AM33XX),
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
