/*
 *  linux/arch/arm/mach-omap2/clock.c
 *
 *  Copyright (C) 2005 Texas Instruments Inc.
 *  Richard Woodruff <r-woodruff2@ti.com>
 *  Created for OMAP2.
 *
 *  Cleaned up and modified to use omap shared clock framework by
 *  Tony Lindgren <tony@atomide.com>
 *
 *  Based on omap1 clock.c, Copyright (C) 2004 - 2005 Nokia corporation
 *  Written by Tuukka Tikkanen <tuukka.tikkanen@elektrobit.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/clk.h>

#include <asm/io.h>

#include <asm/arch/clock.h>
#include <asm/arch/sram.h>
#include <asm/div64.h>

#include "memory.h"
#include "clock.h"
#include "prm.h"
#include "prm_regbits_24xx.h"
#include "cm.h"
#include "cm_regbits_24xx.h"

#undef DEBUG

/* CM_CLKSEL1_CORE.CLKSEL_VLYNQ options (2420) */
#define CLKSEL_VLYNQ_96MHZ		0
#define CLKSEL_VLYNQ_CORECLK_16		0x10

/* CM_CLKEN_PLL.EN_{54,96}M_PLL options (24XX) */
#define EN_APLL_STOPPED			0
#define EN_APLL_LOCKED			3

/* CM_{CLKSEL2_CORE,CLKSEL_WKUP}.CLKSEL_GPT* options (24XX) */
#define CLKSEL_GPT_32K			0
#define CLKSEL_GPT_SYSCLK		1
#define CLKSEL_GPT_EXTALTCLK		2

/* CM_CLKSEL1_CORE.CLKSEL_DSS1 options (24XX) */
#define CLKSEL_DSS1_SYSCLK		0
#define CLKSEL_DSS1_CORECLK_16		0x10

/* CM_CLKSEL1_CORE.CLKSEL_DSS2 options (24XX) */
#define CLKSEL_DSS2_SYSCLK		0
#define CLKSEL_DSS2_48MHZ		1

/* CM_CLKSEL1_PLL.APLLS_CLKIN options (24XX) */
#define APLLS_CLKIN_19_2MHZ		0
#define APLLS_CLKIN_13MHZ		2
#define APLLS_CLKIN_12MHZ		3

/* CM_CLKSEL1_PLL.54M_SOURCE options (24XX) */
#define CLK_54M_SOURCE_APLL		0
#define CLK_54M_SOURCE_EXTALTCLK	1

/* CM_CLKSEL1_PLL.48M_SOURCE options (24XX) */
#define CLK_48M_SOURCE_APLL		0
#define CLK_48M_SOURCE_EXTALTCLK	1

/* PRCM_CLKOUT_CTRL.CLKOUT_SOURCE options (2420) */
#define CLKOUT_SOURCE_CORE_CLK		0
#define CLKOUT_SOURCE_SYS_CLK		1
#define CLKOUT_SOURCE_96M_CLK		2
#define CLKOUT_SOURCE_54M_CLK		3

#define MAX_PLL_LOCK_WAIT		100000

//#define DOWN_VARIABLE_DPLL 1			/* Experimental */

static struct prcm_config *curr_prcm_set;
static struct clk *vclk;
static struct clk *sclk;
static u8 cpu_mask;

static u32 sysclkout_div[] = {1, 2, 4, 8, 16};

/*-------------------------------------------------------------------------
 * Omap2 specific clock functions
 *-------------------------------------------------------------------------*/

/* Recalculate SYST_CLK */
static void omap2_sys_clk_recalc(struct clk * clk)
{
	u32 div;

	if (!cpu_is_omap34xx()) {
		div = prm_read_reg(OMAP24XX_PRCM_CLKSRC_CTRL);
		/* Test if ext clk divided by 1 or 2 */
		div &= OMAP_SYSCLKDIV_MASK;
		div >>= clk->rate_offset;
		clk->rate = (clk->parent->rate / div);
	}
	propagate_rate(clk);
}

static u32 omap2_get_dpll_rate(struct clk * tclk)
{
	long long dpll_clk;
	int dpll_mult, dpll_div, amult;
	u32 dpll;

	dpll = cm_read_mod_reg(PLL_MOD, CM_CLKSEL1);

	dpll_mult = dpll & OMAP24XX_DPLL_MULT_MASK;
	dpll_mult >>= OMAP24XX_DPLL_MULT_SHIFT;		/* 10 bits */
	dpll_div = dpll & OMAP24XX_DPLL_DIV_MASK;
	dpll_div >>= OMAP24XX_DPLL_DIV_SHIFT;		/* 4 bits */
	dpll_clk = (long long)tclk->parent->rate * dpll_mult;
	do_div(dpll_clk, dpll_div + 1);
	amult = cm_read_mod_reg(PLL_MOD, CM_CLKSEL2);
	amult &= OMAP24XX_CORE_CLK_SRC_MASK;
	dpll_clk *= amult;

	return dpll_clk;
}

/*
 * Used for clocks that have the same value as the parent clock,
 * divided by some factor
 */
static void omap2_fixed_divisor_recalc(struct clk *clk)
{
	WARN_ON(!clk->fixed_div);

	clk->rate = clk->parent->rate / clk->fixed_div;

	if (clk->flags & RATE_PROPAGATES)
		propagate_rate(clk);
}

static void omap2_propagate_rate(struct clk * clk)
{
	if (!(clk->flags & RATE_FIXED))
		clk->rate = clk->parent->rate;

	propagate_rate(clk);
}

static void omap2_set_osc_ck(int enable)
{
	u32 pcc;

	pcc = prm_read_reg(OMAP24XX_PRCM_CLKSRC_CTRL);

	if (enable)
		prm_write_reg(pcc & ~OMAP_AUTOEXTCLKMODE_MASK,
			      OMAP24XX_PRCM_CLKSRC_CTRL);
	else
		prm_write_reg(pcc | OMAP_AUTOEXTCLKMODE_MASK,
			      OMAP24XX_PRCM_CLKSRC_CTRL);
}

/*
 * omap2_wait_clock_ready - wait for PLL to lock
 *
 * Returns 1 if the PLL locked, 0 if it failed to lock.
 */
static int omap2_wait_clock_ready(void __iomem *reg, u32 cval, const char *name)
{
	int i = 0;

	/* Wait for lock */
	while (!(cm_read_reg(reg) & cval)) {
		++i;
		udelay(1);
		if (i == MAX_PLL_LOCK_WAIT) {
			printk(KERN_ERR "Clock %s didn't lock in %d tries\n",
			       name, MAX_PLL_LOCK_WAIT);
			break;
		}
	}

	if (i)
		pr_debug("Clock %s stable after %d loops\n", name, i);

	return (i < MAX_PLL_LOCK_WAIT) ? 1 : 0;
};


/* Enable an APLL if off */
static void omap2_clk_fixed_enable(struct clk *clk)
{
	u32 cval, apll_mask;

	apll_mask = EN_APLL_LOCKED << clk->enable_bit;

	cval = cm_read_mod_reg(PLL_MOD, CM_CLKEN);

	if ((cval & apll_mask) == apll_mask)
		return;   /* apll already enabled */

	cval &= ~apll_mask;
	cval |= apll_mask;
	cm_write_mod_reg(cval, PLL_MOD, CM_CLKEN);

	if (clk == &apll96_ck)
		cval = OMAP24XX_ST_96M_APLL;
	else if (clk == &apll54_ck)
		cval = OMAP24XX_ST_54M_CLK;

	omap2_wait_clock_ready(OMAP_CM_REGADDR(PLL_MOD, CM_IDLEST), cval,
			    clk->name);
}

static void omap2_clk_wait_ready(struct clk *clk)
{
	void __iomem *reg, *other_reg, *st_reg;
	u32 bit;

	reg = clk->enable_reg;
	if (reg == OMAP_CM_REGADDR(CORE_MOD, CM_FCLKEN1) ||
	    reg == OMAP_CM_REGADDR(CORE_MOD, OMAP24XX_CM_FCLKEN2))
		other_reg = (void __iomem *)(((u32)reg & ~0xf0) | 0x10); /* CM_ICLKEN* */
	else if (reg == OMAP_CM_REGADDR(CORE_MOD, CM_ICLKEN1) ||
		 reg == OMAP_CM_REGADDR(CORE_MOD, CM_ICLKEN2))
		other_reg = (void __iomem *)(((u32)reg & ~0xf0) | 0x00); /* CM_FCLKEN* */
	else
		return;

	/* No check for DSS or cam clocks */
	if (((u32)reg & 0x0f) == 0) { /* CM_{F,I}CLKEN1 */
		if (clk->enable_bit == OMAP24XX_EN_DSS2_SHIFT ||
		    clk->enable_bit == OMAP24XX_EN_DSS1_SHIFT ||
		    clk->enable_bit == OMAP24XX_EN_CAM_SHIFT)
			return;
	}

	/* Check if both functional and interface clocks
	 * are running. */
	bit = 1 << clk->enable_bit;
	if (!(cm_read_reg(other_reg) & bit))
		return;
	st_reg = (void __iomem *)(((u32)other_reg & ~0xf0) | 0x20); /* CM_IDLEST* */

	omap2_wait_clock_ready(st_reg, bit, clk->name);
}

/* Enables clock without considering parent dependencies or use count
 * REVISIT: Maybe change this to use clk->enable like on omap1?
 */
static int _omap2_clk_enable(struct clk * clk)
{
	u32 regval32;

	if (clk->flags & (ALWAYS_ENABLED | PARENT_CONTROLS_CLOCK))
		return 0;

	if (unlikely(clk == &osc_ck)) {
		omap2_set_osc_ck(1);
		return 0;
	}

	if (unlikely(clk->enable_reg == 0)) {
		printk(KERN_ERR "clock.c: Enable for %s without enable code\n",
		       clk->name);
		return -EINVAL;
	}

	if (clk->enable_reg == OMAP_CM_REGADDR(PLL_MOD, CM_CLKEN)) {
		omap2_clk_fixed_enable(clk);
		return 0;
	}

	regval32 = cm_read_reg(clk->enable_reg);
	regval32 |= (1 << clk->enable_bit);
	cm_write_reg(regval32, clk->enable_reg);
	wmb();

	omap2_clk_wait_ready(clk);

	return 0;
}

/* Stop APLL */
static void omap2_clk_fixed_disable(struct clk *clk)
{
	u32 cval;

	cval = cm_read_mod_reg(PLL_MOD, CM_CLKEN);
	cval &= ~(EN_APLL_LOCKED << clk->enable_bit);
	cm_write_mod_reg(cval, PLL_MOD, CM_CLKEN);
}

/* Disables clock without considering parent dependencies or use count */
static void _omap2_clk_disable(struct clk *clk)
{
	u32 regval32;

	if (clk->flags & (ALWAYS_ENABLED | PARENT_CONTROLS_CLOCK))
		return;

	if (unlikely(clk == &osc_ck)) {
		omap2_set_osc_ck(0);
		return;
	}

	if (clk->enable_reg == 0) {
		/*
		 * 'Independent' here refers to a clock which is not
		 * controlled by its parent.
		 */
		printk(KERN_ERR "clock: clk_disable called on independent "
		       "clock %s which has no enable_reg\n", clk->name);
		return;
	}

	if (clk->enable_reg == OMAP_CM_REGADDR(PLL_MOD, CM_CLKEN)) {
		omap2_clk_fixed_disable(clk);
		return;
	}

	regval32 = cm_read_reg(clk->enable_reg);
	regval32 &= ~(1 << clk->enable_bit);
	cm_write_reg(regval32, clk->enable_reg);
	wmb();
}

static int omap2_clk_enable(struct clk *clk)
{
	int ret = 0;

	if (clk->usecount++ == 0) {
		if (likely((u32)clk->parent))
			ret = omap2_clk_enable(clk->parent);

		if (unlikely(ret != 0)) {
			clk->usecount--;
			return ret;
		}

		ret = _omap2_clk_enable(clk);

		if (unlikely(ret != 0) && clk->parent) {
			omap2_clk_disable(clk->parent);
			clk->usecount--;
		}
	}

	return ret;
}

static void omap2_clk_disable(struct clk *clk)
{
	if (clk->usecount > 0 && !(--clk->usecount)) {
		_omap2_clk_disable(clk);
		if (likely((u32)clk->parent))
			omap2_clk_disable(clk->parent);
	}
}

/*
 * Uses the current prcm set to tell if a rate is valid.
 * You can go slower, but not faster within a given rate set.
 */
static u32 omap2_dpll_round_rate(unsigned long target_rate)
{
	u32 high, low, core_clk_src;

	core_clk_src = cm_read_mod_reg(PLL_MOD, CM_CLKSEL2);
	core_clk_src &= OMAP24XX_CORE_CLK_SRC_MASK;

	if (core_clk_src == CORE_CLK_SRC_DPLL) {	/* DPLL clockout */
		high = curr_prcm_set->dpll_speed * 2;
		low = curr_prcm_set->dpll_speed;
	} else {				/* DPLL clockout x 2 */
		high = curr_prcm_set->dpll_speed;
		low = curr_prcm_set->dpll_speed / 2;
	}

#ifdef DOWN_VARIABLE_DPLL
	if (target_rate > high)
		return high;
	else
		return target_rate;
#else
	if (target_rate > low)
		return high;
	else
		return low;
#endif

}

static void omap2_dpll_recalc(struct clk *clk)
{
	clk->rate = omap2_get_dpll_rate(clk);

	propagate_rate(clk);
}

/*
 * Used for clocks that are part of CLKSEL_xyz governed clocks.
 * REVISIT: Maybe change to use clk->enable() functions like on omap1?
 */
static void omap2_clksel_recalc(struct clk * clk)
{
	u32 clksel1_core, div = 0;

	clksel1_core = cm_read_mod_reg(CORE_MOD, CM_CLKSEL1);

	if ((clk == &dss1_fck) &&
	    (clksel1_core & OMAP24XX_CLKSEL_DSS1_MASK) == 0) {
		div = 1;
	}

	if ((clk == &vlynq_fck) && cpu_is_omap2420() &&
	    (clksel1_core & OMAP2420_CLKSEL_VLYNQ_MASK) == CLKSEL_VLYNQ_96MHZ) {
		div = 1;
	}

	div = omap2_clksel_get_divisor(clk);
	if (div == 0)
		return;

	if (unlikely(clk->rate == clk->parent->rate / div))
		return;
	clk->rate = clk->parent->rate / div;

	if (unlikely(clk->flags & RATE_PROPAGATES))
		propagate_rate(clk);
}

/*
 * Finds best divider value in an array based on the source and target
 * rates. The divider array must be sorted with smallest divider first.
 */
static inline u32 omap2_divider_from_table(u32 size, u32 *div_array,
					   u32 src_rate, u32 tgt_rate)
{
	int i, test_rate;

	if (div_array == NULL)
		return ~1;

	for (i = 0; i < size; i++) {
		test_rate = src_rate / *div_array;
		if (test_rate <= tgt_rate)
			return *div_array;
		++div_array;
	}

	return ~0;	/* No acceptable divider */
}

/*
 * Find divisor for the given clock and target rate.
 *
 * Note that this will not work for clocks which are part of CONFIG_PARTICIPANT,
 * they are only settable as part of virtual_prcm set.
 */
static u32 omap2_clksel_round_rate(struct clk *tclk, u32 target_rate,
	u32 *new_div)
{
	u32 gfx_div[] = {2, 3, 4};
	u32 dss1_div[] = {1, 2, 3, 4, 5, 6, 8, 9, 12, 16};
	u32 vlynq_div[] = {1, 2, 3, 4, 6, 8, 9, 12, 16, 18};
	u32 best_div = ~0, asize = 0;
	u32 *div_array = NULL;

	switch (tclk->flags & SRC_RATE_SEL_MASK) {
	case CM_GFX_SEL1:
		asize = ARRAY_SIZE(gfx_div);
		div_array = gfx_div;
		break;
	case CM_PLL_SEL1:
		return omap2_dpll_round_rate(target_rate);
	case CM_SYSCLKOUT_SEL1:
		asize = ARRAY_SIZE(sysclkout_div);
		div_array = sysclkout_div;
		break;
	case CM_CORE_SEL1:
		if (tclk == &dss1_fck) {
			if (tclk->parent == &core_ck) {
				asize = ARRAY_SIZE(dss1_div);
				div_array = dss1_div;
			} else {
				*new_div = 0; /* fixed clk */
				return(tclk->parent->rate);
			}
		} else if ((tclk == &vlynq_fck) && cpu_is_omap2420()) {
			if (tclk->parent == &core_ck) {
				asize = ARRAY_SIZE(vlynq_div);
				div_array = vlynq_div;
			} else {
				*new_div = 0; /* fixed clk */
				return (tclk->parent->rate);
			}
		}
		break;
	}

	best_div = omap2_divider_from_table(asize, div_array,
					    tclk->parent->rate, target_rate);
	if (best_div == ~0) {
		*new_div = 1;
		return best_div; /* signal error */
	}

	*new_div = best_div;
	return (tclk->parent->rate / best_div);
}

/* Given a clock and a rate apply a clock specific rounding function */
static long omap2_clk_round_rate(struct clk *clk, unsigned long rate)
{
	u32 new_div = 0;

	if (clk->flags & RATE_FIXED)
		return clk->rate;

	if (clk->flags & RATE_CKCTL)
		return omap2_clksel_round_rate(clk, rate, &new_div);

	if (clk->round_rate != 0)
		return clk->round_rate(clk, rate);

	return clk->rate;
}

static int omap2_reprogram_dpll(struct clk * clk, unsigned long rate)
{
	u32 flags, cur_rate, low, mult, div, valid_rate, done_rate;
	u32 bypass = 0;
	struct prcm_config tmpset;
	int ret = -EINVAL;

	local_irq_save(flags);
	cur_rate = omap2_get_dpll_rate(&dpll_ck);
	mult = cm_read_mod_reg(PLL_MOD, CM_CLKSEL2);
	mult &= OMAP24XX_CORE_CLK_SRC_MASK;

	if ((rate == (cur_rate / 2)) && (mult == 2)) {
		omap2_reprogram_sdrc(CORE_CLK_SRC_DPLL, 1);
	} else if ((rate == (cur_rate * 2)) && (mult == 1)) {
		omap2_reprogram_sdrc(CORE_CLK_SRC_DPLL_X2, 1);
	} else if (rate != cur_rate) {
		valid_rate = omap2_dpll_round_rate(rate);
		if (valid_rate != rate)
			goto dpll_exit;

		if (mult == 1)
			low = curr_prcm_set->dpll_speed;
		else
			low = curr_prcm_set->dpll_speed / 2;

		tmpset.cm_clksel1_pll = cm_read_mod_reg(PLL_MOD, CM_CLKSEL1);
		tmpset.cm_clksel1_pll &= ~(OMAP24XX_DPLL_MULT_MASK |
					   OMAP24XX_DPLL_DIV_MASK);
		div = ((curr_prcm_set->xtal_speed / 1000000) - 1);
		tmpset.cm_clksel2_pll = cm_read_mod_reg(PLL_MOD, CM_CLKSEL2);
		tmpset.cm_clksel2_pll &= ~OMAP24XX_CORE_CLK_SRC_MASK;
		if (rate > low) {
			tmpset.cm_clksel2_pll |= CORE_CLK_SRC_DPLL_X2;
			mult = ((rate / 2) / 1000000);
			done_rate = CORE_CLK_SRC_DPLL_X2;
		} else {
			tmpset.cm_clksel2_pll |= CORE_CLK_SRC_DPLL;
			mult = (rate / 1000000);
			done_rate = CORE_CLK_SRC_DPLL;
		}
		tmpset.cm_clksel1_pll |= (div << OMAP24XX_DPLL_DIV_SHIFT);
		tmpset.cm_clksel1_pll |= (mult << OMAP24XX_DPLL_MULT_SHIFT);

		/* Worst case */
		tmpset.base_sdrc_rfr = V24XX_SDRC_RFR_CTRL_BYPASS;

		if (rate == curr_prcm_set->xtal_speed)	/* If asking for 1-1 */
			bypass = 1;

		omap2_reprogram_sdrc(CORE_CLK_SRC_DPLL_X2, 1); /* For init_mem */

		/* Force dll lock mode */
		omap2_set_prcm(tmpset.cm_clksel1_pll, tmpset.base_sdrc_rfr,
			       bypass);

		/* Errata: ret dll entry state */
		omap2_init_memory_params(omap2_dll_force_needed());
		omap2_reprogram_sdrc(done_rate, 0);
	}
	omap2_dpll_recalc(&dpll_ck);
	ret = 0;

dpll_exit:
	local_irq_restore(flags);
	return(ret);
}

/* Just return the MPU speed */
static void omap2_mpu_recalc(struct clk * clk)
{
	clk->rate = curr_prcm_set->mpu_speed;
}

/*
 * Look for a rate equal or less than the target rate given a configuration set.
 *
 * What's not entirely clear is "which" field represents the key field.
 * Some might argue L3-DDR, others ARM, others IVA. This code is simple and
 * just uses the ARM rates.
 */
static long omap2_round_to_table_rate(struct clk * clk, unsigned long rate)
{
	struct prcm_config * ptr;
	long highest_rate;

	if (clk != &virt_prcm_set)
		return -EINVAL;

	highest_rate = -EINVAL;

	for (ptr = rate_table; ptr->mpu_speed; ptr++) {
		if (!(ptr->flags & cpu_mask))
			continue;
		if (ptr->xtal_speed != sys_ck.rate)
			continue;

		highest_rate = ptr->mpu_speed;

		/* Can check only after xtal frequency check */
		if (ptr->mpu_speed <= rate)
			break;
	}
	return highest_rate;
}

/*
 * omap2_clksel_to_divisor() - turn field value into integer divider
 */
static u32 omap2_clksel_to_divisor(u32 div_sel, u32 field_val)
{
	u32 i;

	if ((div_sel & SRC_RATE_SEL_MASK) == CM_SYSCLKOUT_SEL1) {
		for (i = 0; i < ARRAY_SIZE(sysclkout_div); i++) {
			if (field_val == i)
				return sysclkout_div[i];
		}
		return 0;
	} else
		return field_val;
}

/*
 * omap2_divisor_to_clksel() - turn integer divider into field value
 */
static u32 omap2_divisor_to_clksel(u32 div_sel, u32 div)
{
	u32 i;

	if ((div_sel & SRC_RATE_SEL_MASK) == CM_SYSCLKOUT_SEL1) {
		for (i = 0; i < ARRAY_SIZE(sysclkout_div); i++) {
			if (div == sysclkout_div[i])
				return i;
		}
		return ~0;
	} else
		return div;
}

/*
 * Returns the CLKSEL divider register value
 */
static void __iomem *omap2_get_clksel(u32 *field_mask, struct clk *clk)
{
	u32 div_off, mask = ~0;
	void __iomem *div_addr = 0;

	div_off = clk->rate_offset;

	switch (clk->flags & SRC_RATE_SEL_MASK) {
	case CM_MPU_SEL1:
		div_addr = OMAP_CM_REGADDR(MPU_MOD, CM_CLKSEL);
		mask = OMAP24XX_CLKSEL_MPU_MASK;
		break;
	case CM_DSP_SEL1:
		div_addr = OMAP_CM_REGADDR(OMAP24XX_DSP_MOD, CM_CLKSEL);
		if (cpu_is_omap2420()) {
			if (div_off == OMAP24XX_CLKSEL_DSP_SHIFT)
				mask = OMAP24XX_CLKSEL_DSP_MASK;
			else if (div_off == OMAP2420_CLKSEL_IVA_SHIFT)
				mask = OMAP2420_CLKSEL_IVA_MASK;
			else if (div_off == OMAP24XX_CLKSEL_DSP_IF_SHIFT)
				mask = OMAP24XX_CLKSEL_DSP_IF_MASK;
		} else if (cpu_is_omap2430()) {
			if (div_off == OMAP24XX_CLKSEL_DSP_SHIFT)
				mask = OMAP24XX_CLKSEL_DSP_MASK;
			else if (div_off == OMAP24XX_CLKSEL_DSP_IF_SHIFT)
				mask = OMAP24XX_CLKSEL_DSP_IF_MASK;
		}
	case CM_GFX_SEL1:
		div_addr = OMAP_CM_REGADDR(GFX_MOD, CM_CLKSEL);
		if (div_off == OMAP_CLKSEL_GFX_SHIFT)
			mask = OMAP_CLKSEL_GFX_MASK;
		break;
	case CM_MODEM_SEL1:
		div_addr = OMAP_CM_REGADDR(OMAP2430_MDM_MOD, CM_CLKSEL);
		if (div_off == OMAP2430_CLKSEL_MDM_SHIFT)
			mask = OMAP2430_CLKSEL_MDM_MASK;
		break;
	case CM_SYSCLKOUT_SEL1:
		div_addr = OMAP24XX_PRCM_CLKOUT_CTRL;
		if (div_off == OMAP24XX_CLKOUT_DIV_SHIFT)
			mask = OMAP24XX_CLKOUT_DIV_MASK;
		else if (div_off == OMAP2420_CLKOUT2_DIV_SHIFT)
			mask = OMAP2420_CLKOUT2_DIV_MASK;
		break;
	case CM_CORE_SEL1:
		div_addr = OMAP_CM_REGADDR(CORE_MOD, CM_CLKSEL1);
		switch (div_off) {
		case OMAP24XX_CLKSEL_L3_SHIFT:
			mask = OMAP24XX_CLKSEL_L3_MASK;
			break;
		case OMAP24XX_CLKSEL_L4_SHIFT:
			mask = OMAP24XX_CLKSEL_L4_MASK;
			break;
		case OMAP24XX_CLKSEL_DSS1_SHIFT:
			mask = OMAP24XX_CLKSEL_DSS1_MASK;
			break;
		case OMAP24XX_CLKSEL_DSS2_SHIFT:
			mask = OMAP24XX_CLKSEL_DSS2_MASK;
			break;
		case OMAP2420_CLKSEL_VLYNQ_SHIFT:
			mask = OMAP2420_CLKSEL_VLYNQ_MASK;
			break;
		case OMAP24XX_CLKSEL_SSI_SHIFT:
			mask = OMAP24XX_CLKSEL_SSI_MASK;
			break;
		case OMAP24XX_CLKSEL_USB_SHIFT:
			mask = OMAP24XX_CLKSEL_USB_MASK;
			break;
		}
	}

	if (unlikely((mask == ~0) || (div_addr == 0)))
		return 0;

	*field_mask = mask;

	return div_addr;
}


/*
 * Return divider to be applied to parent clock.
 * Return 0 on error.
 */
static u32 omap2_clksel_get_divisor(struct clk *clk)
{
	u32 div, field_mask, field_val;
	void __iomem *div_addr;

	div_addr = omap2_get_clksel(&field_mask, clk);
	if (div_addr == 0)
		return 0;

	field_val = cm_read_reg(div_addr) & field_mask;
	field_val >>= clk->rate_offset;

	div = omap2_clksel_to_divisor(clk->flags, field_val);

	return div;
}

/* Set the clock rate for a clock source */
static int omap2_clk_set_rate(struct clk *clk, unsigned long rate)
{
	int ret = -EINVAL;
	u32 div_off, field_mask, field_val, reg_val, validrate;
	u32 new_div = 0;
	void __iomem *div_addr;

	if (!(clk->flags & CONFIG_PARTICIPANT) && (clk->flags & RATE_CKCTL)) {
		if (clk == &dpll_ck)
			return omap2_reprogram_dpll(clk, rate);

		/* Isolate control register */
		div_off = clk->rate_offset;

		validrate = omap2_clksel_round_rate(clk, rate, &new_div);
		if (validrate != rate)
			return ret;

		div_addr = omap2_get_clksel(&field_mask, clk);
		if (div_addr == 0)
			return ret;

		field_val = omap2_divisor_to_clksel(clk->flags, new_div);
		if (field_val == ~0)
			return ret;

		reg_val = cm_read_reg(div_addr);
		reg_val &= ~field_mask;
		reg_val |= (field_val << div_off);
		cm_write_reg(reg_val, div_addr);
		wmb();
		clk->rate = clk->parent->rate / new_div;

		if (clk->flags & DELAYED_APP) {
			prm_write_reg(OMAP24XX_VALID_CONFIG,
				      OMAP24XX_PRCM_CLKCFG_CTRL);
			wmb();
		}
		ret = 0;
	} else if (clk->set_rate != 0) {
		ret = clk->set_rate(clk, rate);
	}

	if (unlikely(ret == 0 && (clk->flags & RATE_PROPAGATES)))
		propagate_rate(clk);

	return ret;
}

/* Converts encoded control register address into a full address */
static u32 omap2_clksel_get_src_field(void __iomem **src_addr,
				      struct clk *src_clk, u32 *field_mask,
				      struct clk *clk, u32 *parent_div)
{
	u32 val = ~0, mask = 0;
	void __iomem *src_reg_addr = 0;
	u32 reg_offset;

	*parent_div = 0;
	reg_offset = clk->src_offset;

	/* Find target control register.*/
	switch (clk->flags & SRC_RATE_SEL_MASK) {
	case CM_CORE_SEL1:
		src_reg_addr = OMAP_CM_REGADDR(CORE_MOD, CM_CLKSEL1);
		if (reg_offset == OMAP24XX_CLKSEL_DSS2_SHIFT) {
			mask = OMAP24XX_CLKSEL_DSS2_MASK;
			if (src_clk == &sys_ck)
				val = CLKSEL_DSS2_SYSCLK;
			else if (src_clk == &func_48m_ck)
				val = CLKSEL_DSS2_48MHZ;
			else
				WARN_ON(1); /* unknown src_clk */
		} else if (reg_offset == OMAP24XX_CLKSEL_DSS1_SHIFT) {
			mask = OMAP24XX_CLKSEL_DSS1_MASK;
			if (src_clk == &sys_ck) {
				val = CLKSEL_DSS1_SYSCLK;
			} else if (src_clk == &core_ck) {
				val = CLKSEL_DSS1_CORECLK_16;
				*parent_div = 16;
			} else {
				WARN_ON(1); /* unknown src clk */
			}
		} else if ((reg_offset == OMAP2420_CLKSEL_VLYNQ_SHIFT) &&
			   cpu_is_omap2420()) {
			mask = OMAP2420_CLKSEL_VLYNQ_MASK;
			if (src_clk == &func_96m_ck) {
				val = CLKSEL_VLYNQ_96MHZ;
			} else if (src_clk == &core_ck) {
				val = CLKSEL_VLYNQ_CORECLK_16;
				*parent_div = 16;
			} else {
				WARN_ON(1); /* unknown src_clk */
			}
		} else {
			WARN_ON(1); /* unknown reg_offset */
		}
		break;
	case CM_CORE_SEL2:
		WARN_ON(reg_offset < OMAP24XX_CLKSEL_GPT2_SHIFT ||
			reg_offset > OMAP24XX_CLKSEL_GPT12_SHIFT);
		src_reg_addr = OMAP_CM_REGADDR(CORE_MOD, CM_CLKSEL2);
		mask = OMAP24XX_CLKSEL_GPT2_MASK;
		mask <<= (reg_offset - OMAP24XX_CLKSEL_GPT2_SHIFT);
		if (src_clk == &func_32k_ck)
			val = CLKSEL_GPT_32K;
		else if (src_clk == &sys_ck)
			val = CLKSEL_GPT_SYSCLK;
		else if (src_clk == &alt_ck)
			val = CLKSEL_GPT_EXTALTCLK;
		else
			WARN_ON(1);  /* unknown src_clk */
		break;
	case CM_WKUP_SEL1:
		WARN_ON(reg_offset != 0); /* unknown reg_offset */
		src_reg_addr = OMAP_CM_REGADDR(WKUP_MOD, CM_CLKSEL);
		mask = OMAP24XX_CLKSEL_GPT1_MASK;
		if (src_clk == &func_32k_ck)
			val = CLKSEL_GPT_32K;
		else if (src_clk == &sys_ck)
			val = CLKSEL_GPT_SYSCLK;
		else if (src_clk == &alt_ck)
			val = CLKSEL_GPT_EXTALTCLK;
		else
			WARN_ON(1); /* unknown src_clk */
		break;
	case CM_PLL_SEL1:
		src_reg_addr = OMAP_CM_REGADDR(PLL_MOD, CM_CLKSEL1);
		if (reg_offset == 0x3) {
			mask = OMAP24XX_48M_SOURCE;
			if (src_clk == &apll96_ck)
				val = CLK_48M_SOURCE_APLL;
			else if (src_clk == &alt_ck)
				val = CLK_48M_SOURCE_EXTALTCLK;
			else
				WARN_ON(1); /* unknown src_clk */
		}
		else if (reg_offset == 0x5) {
			mask = OMAP24XX_54M_SOURCE;
			if (src_clk == &apll54_ck)
				val = CLK_54M_SOURCE_APLL;
			else if (src_clk == &alt_ck)
				val = CLK_54M_SOURCE_EXTALTCLK;
			else
				WARN_ON(1); /* unknown src_clk */
		} else {
			WARN_ON(1); /* unknown reg_offset */
		}
		break;
	case CM_PLL_SEL2:
		WARN_ON(reg_offset != 0);
		src_reg_addr = OMAP_CM_REGADDR(PLL_MOD, CM_CLKSEL2);
		mask = OMAP24XX_CORE_CLK_SRC_MASK;
		if (src_clk == &func_32k_ck)
			val = CORE_CLK_SRC_32K;
		else if (src_clk == &dpll_ck)
			val = CORE_CLK_SRC_DPLL_X2;
		else
			WARN_ON(1); /* unknown src_clk */
		break;
	case CM_SYSCLKOUT_SEL1:
		src_reg_addr = OMAP24XX_PRCM_CLKOUT_CTRL;

		if (reg_offset == OMAP24XX_CLKOUT_SOURCE_SHIFT) {
			mask = OMAP24XX_CLKOUT_SOURCE_MASK;
		} else if (reg_offset == OMAP2420_CLKOUT2_SOURCE_SHIFT) {
			mask = OMAP2420_CLKOUT2_SOURCE_MASK;
		} else {
			WARN_ON(1); /* unknown reg_offset */
		}

		if (src_clk == &dpll_ck)
			val = 0;
		else if (src_clk == &sys_ck)
			val = 1;
		else if (src_clk == &func_96m_ck)
			val = 2;
		else if (src_clk == &func_54m_ck)
			val = 3;
		else
			WARN_ON(1); /* unknown src_clk */
		break;
	}

	if (val == ~0)			/* Catch errors in offset */
		*src_addr = 0;
	else
		*src_addr = src_reg_addr;

	WARN_ON(mask == 0);

	*field_mask = mask;

	return val;
}

static int omap2_clk_set_parent(struct clk *clk, struct clk *new_parent)
{
	void __iomem *src_addr;
	u32 field_val, field_mask, reg_val, parent_div;

	if (unlikely(clk->flags & CONFIG_PARTICIPANT))
		return -EINVAL;

	if (unlikely(!(clk->flags & SRC_SEL_MASK)))
		return -EINVAL;

	field_val = omap2_clksel_get_src_field(&src_addr, new_parent,
					       &field_mask, clk, &parent_div);
	if (src_addr == 0)
		return -EINVAL;

	if (clk->usecount > 0)
		_omap2_clk_disable(clk);

	/* Set new source value (previous dividers if any in effect) */
	reg_val = __raw_readl(src_addr) & ~field_mask;
	reg_val |= (field_val << clk->src_offset);
	__raw_writel(reg_val, src_addr);
	wmb();

	if (clk->flags & DELAYED_APP) {
		prm_write_reg(OMAP24XX_VALID_CONFIG,
			      OMAP24XX_PRCM_CLKCFG_CTRL);
		wmb();
	}

	if (clk->usecount > 0)
		_omap2_clk_enable(clk);

	clk->parent = new_parent;

	/* SRC_RATE_SEL_MASK clocks follow their parents rates.*/
	clk->rate = new_parent->rate;

	if (parent_div > 0)
		clk->rate /= parent_div;

	if (unlikely(clk->flags & RATE_PROPAGATES))
		propagate_rate(clk);

	return 0;
}

/* Sets basic clocks based on the specified rate */
static int omap2_select_table_rate(struct clk * clk, unsigned long rate)
{
	u32 flags, cur_rate, done_rate, bypass = 0, tmp;
	struct prcm_config *prcm;
	unsigned long found_speed = 0;

	if (clk != &virt_prcm_set)
		return -EINVAL;

	for (prcm = rate_table; prcm->mpu_speed; prcm++) {
		if (!(prcm->flags & cpu_mask))
			continue;

		if (prcm->xtal_speed != sys_ck.rate)
			continue;

		if (prcm->mpu_speed <= rate) {
			found_speed = prcm->mpu_speed;
			break;
		}
	}

	if (!found_speed) {
		printk(KERN_INFO "Could not set MPU rate to %luMHz\n",
		       rate / 1000000);
		return -EINVAL;
	}

	curr_prcm_set = prcm;
	cur_rate = omap2_get_dpll_rate(&dpll_ck);

	if (prcm->dpll_speed == cur_rate / 2) {
		omap2_reprogram_sdrc(CORE_CLK_SRC_DPLL, 1);
	} else if (prcm->dpll_speed == cur_rate * 2) {
		omap2_reprogram_sdrc(CORE_CLK_SRC_DPLL_X2, 1);
	} else if (prcm->dpll_speed != cur_rate) {
		local_irq_save(flags);

		if (prcm->dpll_speed == prcm->xtal_speed)
			bypass = 1;

		if ((prcm->cm_clksel2_pll & OMAP24XX_CORE_CLK_SRC_MASK) ==
		    CORE_CLK_SRC_DPLL_X2)
			done_rate = CORE_CLK_SRC_DPLL_X2;
		else
			done_rate = CORE_CLK_SRC_DPLL;

		/* MPU divider */
		cm_write_mod_reg(prcm->cm_clksel_mpu, MPU_MOD, CM_CLKSEL);

		/* dsp + iva1 div(2420), iva2.1(2430) */
		cm_write_mod_reg(prcm->cm_clksel_dsp,
				 OMAP24XX_DSP_MOD, CM_CLKSEL);

		cm_write_mod_reg(prcm->cm_clksel_gfx, GFX_MOD, CM_CLKSEL);

		/* Major subsystem dividers */
		tmp = cm_read_mod_reg(CORE_MOD, CM_CLKSEL1) & OMAP24XX_CLKSEL_DSS2_MASK;
		cm_write_mod_reg(prcm->cm_clksel1_core | tmp, CORE_MOD, CM_CLKSEL1);
		if (cpu_is_omap2430())
			cm_write_mod_reg(prcm->cm_clksel_mdm,
					 OMAP2430_MDM_MOD, CM_CLKSEL);

		/* x2 to enter init_mem */
		omap2_reprogram_sdrc(CORE_CLK_SRC_DPLL_X2, 1);

		omap2_set_prcm(prcm->cm_clksel1_pll, prcm->base_sdrc_rfr,
			       bypass);

		omap2_init_memory_params(omap2_dll_force_needed());
		omap2_reprogram_sdrc(done_rate, 0);

		local_irq_restore(flags);
	}
	omap2_dpll_recalc(&dpll_ck);

	return 0;
}

/*-------------------------------------------------------------------------
 * Omap2 clock reset and init functions
 *-------------------------------------------------------------------------*/

#ifdef CONFIG_OMAP_RESET_CLOCKS
static void __init omap2_clk_disable_unused(struct clk *clk)
{
	u32 regval32;

	regval32 = cm_read_reg(clk->enable_reg);
	if ((regval32 & (1 << clk->enable_bit)) == 0)
		return;

	printk(KERN_INFO "Disabling unused clock \"%s\"\n", clk->name);
	_omap2_clk_disable(clk);
}
#else
#define omap2_clk_disable_unused	NULL
#endif

static struct clk_functions omap2_clk_functions = {
	.clk_enable		= omap2_clk_enable,
	.clk_disable		= omap2_clk_disable,
	.clk_round_rate		= omap2_clk_round_rate,
	.clk_set_rate		= omap2_clk_set_rate,
	.clk_set_parent		= omap2_clk_set_parent,
	.clk_disable_unused	= omap2_clk_disable_unused,
};

static void __init omap2_get_crystal_rate(struct clk *osc, struct clk *sys)
{
	u32 div, aplls, sclk = 13000000;

	aplls = cm_read_mod_reg(PLL_MOD, CM_CLKSEL1);
	aplls &= OMAP24XX_APLLS_CLKIN_MASK;
	aplls >>= OMAP24XX_APLLS_CLKIN_SHIFT;

	if (aplls == APLLS_CLKIN_19_2MHZ)
		sclk = 19200000;
	else if (aplls == APLLS_CLKIN_13MHZ)
		sclk = 13000000;
	else if (aplls == APLLS_CLKIN_12MHZ)
		sclk = 12000000;

	div = prm_read_reg(OMAP24XX_PRCM_CLKSRC_CTRL);
	div &= OMAP_SYSCLKDIV_MASK;
	div >>= sys->rate_offset;

	osc->rate = sclk * div;
	sys->rate = sclk;
}

/*
 * Set clocks for bypass mode for reboot to work.
 */
void omap2_clk_prepare_for_reboot(void)
{
	u32 rate;

	if (vclk == NULL || sclk == NULL)
		return;

	rate = clk_get_rate(sclk);
	clk_set_rate(vclk, rate);
}

/*
 * Switch the MPU rate if specified on cmdline.
 * We cannot do this early until cmdline is parsed.
 */
static int __init omap2_clk_arch_init(void)
{
	if (!mpurate)
		return -EINVAL;

	if (omap2_select_table_rate(&virt_prcm_set, mpurate))
		printk(KERN_ERR "Could not find matching MPU rate\n");

	propagate_rate(&osc_ck);		/* update main root fast */
	propagate_rate(&func_32k_ck);		/* update main root slow */

	printk(KERN_INFO "Switched to new clocking rate (Crystal/DPLL/MPU): "
	       "%ld.%01ld/%ld/%ld MHz\n",
	       (sys_ck.rate / 1000000), (sys_ck.rate / 100000) % 10,
	       (dpll_ck.rate / 1000000), (mpu_ck.rate / 1000000)) ;

	return 0;
}
arch_initcall(omap2_clk_arch_init);

int __init omap2_clk_init(void)
{
	struct prcm_config *prcm;
	struct clk ** clkp;
	u32 clkrate;

	clk_init(&omap2_clk_functions);
	omap2_get_crystal_rate(&osc_ck, &sys_ck);

	for (clkp = onchip_clks; clkp < onchip_clks + ARRAY_SIZE(onchip_clks);
	     clkp++) {

		if ((*clkp)->flags & CLOCK_IN_OMAP242X && cpu_is_omap2420()) {
			clk_register(*clkp);
			continue;
		}

		if ((*clkp)->flags & CLOCK_IN_OMAP243X && (cpu_is_omap2430() || cpu_is_omap34xx())) {
			clk_register(*clkp);
			continue;
		}
	}

	if (cpu_is_omap242x())
		cpu_mask = RATE_IN_242X;
	else if (cpu_is_omap2430())
		cpu_mask = RATE_IN_243X;

	/* Check the MPU rate set by bootloader */
	clkrate = omap2_get_dpll_rate(&dpll_ck);
	for (prcm = rate_table; prcm->mpu_speed; prcm++) {
		if (!(prcm->flags & cpu_mask))
			continue;
		if (prcm->xtal_speed != sys_ck.rate)
			continue;
		if (prcm->dpll_speed <= clkrate)
			 break;
	}
	curr_prcm_set = prcm;

	propagate_rate(&osc_ck);		/* update main root fast */
	propagate_rate(&func_32k_ck);		/* update main root slow */

	printk(KERN_INFO "Clocking rate (Crystal/DPLL/MPU): "
	       "%ld.%01ld/%ld/%ld MHz\n",
	       (sys_ck.rate / 1000000), (sys_ck.rate / 100000) % 10,
	       (dpll_ck.rate / 1000000), (mpu_ck.rate / 1000000)) ;

	/*
	 * Only enable those clocks we will need, let the drivers
	 * enable other clocks as necessary
	 */
	clk_enable(&sync_32k_ick);
	clk_enable(&omapctrl_ick);

	/* Force the APLLs always active. The clocks are idled
	 * automatically by hardware. */
	clk_enable(&apll96_ck);
	clk_enable(&apll54_ck);

	if (cpu_is_omap2430())
		clk_enable(&sdrc_ick);

	/* Avoid sleeping sleeping during omap2_clk_prepare_for_reboot() */
	vclk = clk_get(NULL, "virt_prcm_set");
	sclk = clk_get(NULL, "sys_ck");

	return 0;
}
