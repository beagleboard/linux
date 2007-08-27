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
 *  Copyright (C) 2007 Texas Instruments, Inc.
 *  Copyright (C) 2007 Nokia Corporation
 *  Paul Walmsley
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

/* CM_CLKEN_PLL.EN_{54,96}M_PLL options (24XX) */
#define EN_APLL_STOPPED			0
#define EN_APLL_LOCKED			3

/* CM_CLKSEL1_PLL.APLLS_CLKIN options (24XX) */
#define APLLS_CLKIN_19_2MHZ		0
#define APLLS_CLKIN_13MHZ		2
#define APLLS_CLKIN_12MHZ		3

#define MAX_PLL_LOCK_WAIT		100000

//#define DOWN_VARIABLE_DPLL 1			/* Experimental */

static struct prcm_config *curr_prcm_set;
static struct clk *vclk;
static struct clk *sclk;
static u8 cpu_mask;

/*-------------------------------------------------------------------------
 * Omap2 specific clock functions
 *-------------------------------------------------------------------------*/

static inline u8 mask_to_shift(u32 mask)
{
	return ffs(mask) - 1;
}

/**
 * omap2_init_clksel_parent - set a clksel clk's parent field from the hardware
 * @clk: OMAP clock struct ptr to use
 *
 * Given a pointer to a source-selectable struct clk, read the hardware
 * register and determine what its parent is currently set to.  Update the
 * clk->parent field with the appropriate clk ptr.
 */
static void omap2_init_clksel_parent(struct clk *clk)
{
	const struct clksel *clks;
	const struct clksel_rate *clkr;
	u32 r, found = 0;

	if (!clk->clksel)
		return;

	/* XXX Should be __raw_readl for non-CM 3430 clocks ? */
	r = cm_read_reg(clk->clksel_reg) & clk->clksel_mask;
	r >>= mask_to_shift(clk->clksel_mask);

	for (clks = clk->clksel; clks->parent && !found; clks++) {
		for (clkr = clks->rates; clkr->div && !found; clkr++) {
			if ((clkr->flags & cpu_mask) && (clkr->val == r)) {
				if (clk->parent != clks->parent) {
					pr_debug("clock: inited %s parent "
						 "to %s (was %s)\n",
						 clk->name, clks->parent->name,
						 ((clk->parent->name) ?
						  clk->parent->name : "NULL"));
					clk->parent = clks->parent;
				};
				found = 1;
			}
		}
	}

	if (!found)
		printk(KERN_ERR "clock: init parent: could not find "
		       "regval %0x for clock %s\n", r,  clk->name);

	return;
}

/* Returns the DPLL rate */
static u32 omap2_get_dpll_rate(struct clk *clk)
{
	long long dpll_clk;
	u32 dpll_mult, dpll_div, dpll;
	const struct dpll_data *dd;

	dd = clk->dpll_data;
	/* REVISIT: What do we return on error? */
	if (!dd)
		return 0;

	dpll = cm_read_reg(dd->mult_div1_reg);
	dpll_mult = dpll & dd->mult_mask;
	dpll_mult >>= mask_to_shift(dd->mult_mask);
	dpll_div = dpll & dd->div1_mask;
	dpll_div >>= mask_to_shift(dd->div1_mask);

	dpll_clk = (long long)clk->parent->rate * dpll_mult;
	do_div(dpll_clk, dpll_div + 1);

	return dpll_clk;
}

/* This actually returns the rate of core_ck, not dpll_ck. */
static u32 omap2_get_dpll_rate_24xx(struct clk *tclk)
{
	long long dpll_clk;
	u8 amult;

	dpll_clk = omap2_get_dpll_rate(tclk);

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

static int omap2_enable_osc_ck(struct clk *clk)
{
	u32 pcc;

	pcc = prm_read_reg(OMAP24XX_PRCM_CLKSRC_CTRL);

	prm_write_reg(pcc & ~OMAP_AUTOEXTCLKMODE_MASK,
		      OMAP24XX_PRCM_CLKSRC_CTRL);

	return 0;
}

static void omap2_disable_osc_ck(struct clk *clk)
{
	u32 pcc;

	pcc = prm_read_reg(OMAP24XX_PRCM_CLKSRC_CTRL);

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
static int omap2_clk_fixed_enable(struct clk *clk)
{
	u32 cval, apll_mask;

	apll_mask = EN_APLL_LOCKED << clk->enable_bit;

	cval = cm_read_mod_reg(PLL_MOD, CM_CLKEN);

	if ((cval & apll_mask) == apll_mask)
		return 0;   /* apll already enabled */

	cval &= ~apll_mask;
	cval |= apll_mask;
	cm_write_mod_reg(cval, PLL_MOD, CM_CLKEN);

	if (clk == &apll96_ck)
		cval = OMAP24XX_ST_96M_APLL;
	else if (clk == &apll54_ck)
		cval = OMAP24XX_ST_54M_CLK;

	omap2_wait_clock_ready(OMAP_CM_REGADDR(PLL_MOD, CM_IDLEST), cval,
			    clk->name);

	/*
	 * REVISIT: Should we return an error code if omap2_wait_clock_ready()
	 * fails?
	 */
	return 0;
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

	if (clk->enable)
		return clk->enable(clk);

	if (unlikely(clk->enable_reg == 0)) {
		printk(KERN_ERR "clock.c: Enable for %s without enable code\n",
		       clk->name);
		return -EINVAL;
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

	if (clk->disable) {
		clk->disable(clk);
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
	clk->rate = omap2_get_dpll_rate_24xx(clk);

	propagate_rate(clk);
}

/*
 * Used for clocks that are part of CLKSEL_xyz governed clocks.
 * REVISIT: Maybe change to use clk->enable() functions like on omap1?
 */
static void omap2_clksel_recalc(struct clk * clk)
{
	u32 div = 0;

	pr_debug("clock: recalc'ing clksel clk %s\n", clk->name);

	div = omap2_clksel_get_divisor(clk);
	if (div == 0)
		return;

	if (unlikely(clk->rate == clk->parent->rate / div))
		return;
	clk->rate = clk->parent->rate / div;

	pr_debug("clock: new clock rate is %ld (div %d)\n", clk->rate, div);

	if (unlikely(clk->flags & RATE_PROPAGATES))
		propagate_rate(clk);
}

/**
 * omap2_get_clksel_by_parent - return clksel struct for a given clk & parent
 * @clk: OMAP struct clk ptr to inspect
 * @src_clk: OMAP struct clk ptr of the parent clk to search for
 *
 * Scan the struct clksel array associated with the clock to find
 * the element associated with the supplied parent clock address.
 * Returns a pointer to the struct clksel on success or NULL on error.
 */
const static struct clksel *omap2_get_clksel_by_parent(struct clk *clk,
						       struct clk *src_clk)
{
	const struct clksel *clks;

	if (!clk->clksel)
		return NULL;

	for (clks = clk->clksel; clks->parent; clks++) {
		if (clks->parent == src_clk)
			break; /* Found the requested parent */
	}

	if (!clks->parent) {
		printk(KERN_ERR "clock: Could not find parent clock %s in "
		       "clksel array of clock %s\n", src_clk->name,
		       clk->name);
		return NULL;
	}

	return clks;
}

/**
 * omap2_clksel_round_rate_div - find divisor for the given clock and rate
 * @clk: OMAP struct clk to use
 * @target_rate: desired clock rate
 * @new_div: ptr to where we should store the divisor
 *
 * Finds 'best' divider value in an array based on the source and target
 * rates.  The divider array must be sorted with smallest divider first.
 * Note that this will not work for clocks which are part of CONFIG_PARTICIPANT,
 * they are only settable as part of virtual_prcm set.
 *
 * Returns the rounded clock rate or returns 0xffffffff on error.
 */
static u32 omap2_clksel_round_rate_div(struct clk *clk,
				       unsigned long target_rate,
				       u32 *new_div)
{
	unsigned long test_rate;
	const struct clksel *clks;
	const struct clksel_rate *clkr;
	u32 last_div = 0;

	printk(KERN_INFO "clock: clksel_round_rate_div: %s target_rate %ld\n",
	       clk->name, target_rate);

	*new_div = 1;

	clks = omap2_get_clksel_by_parent(clk, clk->parent);
	if (clks == NULL)
		return ~0;

	for (clkr = clks->rates; clkr->div; clkr++) {
		if (!(clkr->flags & cpu_mask))
		    continue;

		/* Sanity check */
		if (clkr->div <= last_div)
			printk(KERN_ERR "clock: clksel_rate table not sorted "
			       "for clock %s", clk->name);

		last_div = clkr->div;

		test_rate = clk->parent->rate / clkr->div;

		if (test_rate <= target_rate)
			break; /* found it */
	}

	if (!clkr->div) {
		printk(KERN_ERR "clock: Could not find divisor for target "
		       "rate %ld for clock %s parent %s\n", target_rate,
		       clk->name, clk->parent->name);
		return ~0;
	}

	*new_div = clkr->div;

	printk(KERN_INFO "clock: new_div = %d, new_rate = %ld\n", *new_div,
	       (clk->parent->rate / clkr->div));

	return (clk->parent->rate / clkr->div);
}

/**
 * omap2_clksel_round_rate - find rounded rate for the given clock and rate
 * @clk: OMAP struct clk to use
 * @target_rate: desired clock rate
 *
 * Compatibility wrapper for OMAP clock framework
 * Finds best target rate based on the source clock and possible dividers.
 * rates. The divider array must be sorted with smallest divider first.
 * Note that this will not work for clocks which are part of CONFIG_PARTICIPANT,
 * they are only settable as part of virtual_prcm set.
 *
 * Returns the rounded clock rate or returns 0xffffffff on error.
 */
static long omap2_clksel_round_rate(struct clk *clk, unsigned long target_rate)
{
	u32 new_div;

	return omap2_clksel_round_rate_div(clk, target_rate, &new_div);
}


/* Given a clock and a rate apply a clock specific rounding function */
static long omap2_clk_round_rate(struct clk *clk, unsigned long rate)
{
	if (clk->round_rate != 0)
		return clk->round_rate(clk, rate);

	if (clk->flags & RATE_FIXED)
		printk(KERN_ERR "clock: generic omap2_clk_round_rate called "
		       "on fixed-rate clock %s\n", clk->name);

	return clk->rate;
}

static int omap2_reprogram_dpll(struct clk * clk, unsigned long rate)
{
	u32 flags, cur_rate, low, mult, div, valid_rate, done_rate;
	u32 bypass = 0;
	struct prcm_config tmpset;
	const struct dpll_data *dd;
	int ret = -EINVAL;

	local_irq_save(flags);
	cur_rate = omap2_get_dpll_rate_24xx(&dpll_ck);
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

		dd = clk->dpll_data;
		if (!dd)
			goto dpll_exit;

		tmpset.cm_clksel1_pll = cm_read_reg(dd->mult_div1_reg);
		tmpset.cm_clksel1_pll &= ~(dd->mult_mask |
					   dd->div1_mask);
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
		tmpset.cm_clksel1_pll |= (div << mask_to_shift(dd->mult_mask));
		tmpset.cm_clksel1_pll |= (mult << mask_to_shift(dd->div1_mask));

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

/**
 * omap2_table_mpu_recalc - just return the MPU speed
 * @clk: virt_prcm_set struct clk
 *
 * Set virt_prcm_set's rate to the mpu_speed field of the current PRCM set.
 */
static void omap2_table_mpu_recalc(struct clk *clk)
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

/**
 * omap2_clksel_to_divisor() - turn clksel field value into integer divider
 * @clk: OMAP struct clk to use
 * @field_val: register field value to find
 *
 * Given a struct clk of a rate-selectable clksel clock, and a register field
 * value to search for, find the corresponding clock divisor.  The register
 * field value should be pre-masked and shifted down so the LSB is at bit 0
 * before calling.  Returns 0 on error
 */
static u32 omap2_clksel_to_divisor(struct clk *clk, u32 field_val)
{
	const struct clksel *clks;
	const struct clksel_rate *clkr;

	clks = omap2_get_clksel_by_parent(clk, clk->parent);
	if (clks == NULL)
		return 0;

	for (clkr = clks->rates; clkr->div; clkr++) {
		if ((clkr->flags & cpu_mask) && (clkr->val == field_val))
			break;
	}

	if (!clkr->div) {
		printk(KERN_ERR "clock: Could not find fieldval %d for "
		       "clock %s parent %s\n", field_val, clk->name,
		       clk->parent->name);
		return 0;
	}

	return clkr->div;
}

/**
 * omap2_divisor_to_clksel() - turn clksel integer divisor into a field value
 * @clk: OMAP struct clk to use
 * @div: integer divisor to search for
 *
 * Given a struct clk of a rate-selectable clksel clock, and a clock divisor,
 * find the corresponding register field value.  The return register value is
 * the value before left-shifting.  Returns 0xffffffff on error
 */
static u32 omap2_divisor_to_clksel(struct clk *clk, u32 div)
{
	const struct clksel *clks;
	const struct clksel_rate *clkr;

	/* should never happen */
	WARN_ON(div == 0);

	clks = omap2_get_clksel_by_parent(clk, clk->parent);
	if (clks == NULL)
		return 0;

	for (clkr = clks->rates; clkr->div; clkr++) {
		if ((clkr->flags & cpu_mask) && (clkr->div == div))
			break;
	}

	if (!clkr->div) {
		printk(KERN_ERR "clock: Could not find divisor %d for "
		       "clock %s parent %s\n", div, clk->name,
		       clk->parent->name);
		return 0;
	}

	return clkr->val;
}

/**
 * omap2_get_clksel - find clksel register addr & field mask for a clk
 * @clk: struct clk to use
 * @field_mask: ptr to u32 to store the register field mask
 *
 * Returns the address of the clksel register upon success or NULL on error.
 */
static void __iomem *omap2_get_clksel(struct clk *clk, u32 *field_mask)
{
	if (unlikely((clk->clksel_reg == 0) || (clk->clksel_mask == 0)))
		return NULL;

	*field_mask = clk->clksel_mask;

	return clk->clksel_reg;
}

/**
 * omap2_clksel_get_divisor - get current divider applied to parent clock.
 * @clk: OMAP struct clk to use.
 *
 * Returns the integer divisor upon success or 0 on error.
 */
static u32 omap2_clksel_get_divisor(struct clk *clk)
{
	u32 field_mask, field_val;
	void __iomem *div_addr;

	div_addr = omap2_get_clksel(clk, &field_mask);
	if (div_addr == 0)
		return 0;

	field_val = cm_read_reg(div_addr) & field_mask;
	field_val >>= mask_to_shift(field_mask);

	return omap2_clksel_to_divisor(clk, field_val);
}

static int omap2_clksel_set_rate(struct clk *clk, unsigned long rate)
{
	u32 field_mask, field_val, reg_val, validrate, new_div = 0;
	void __iomem *div_addr;

	validrate = omap2_clksel_round_rate_div(clk, rate, &new_div);
	if (validrate != rate)
		return -EINVAL;

	div_addr = omap2_get_clksel(clk, &field_mask);
	if (div_addr == 0)
		return -EINVAL;

	field_val = omap2_divisor_to_clksel(clk, new_div);
	if (field_val == ~0)
		return -EINVAL;

	reg_val = cm_read_reg(div_addr);
	reg_val &= ~field_mask;
	reg_val |= (field_val << mask_to_shift(field_mask));
	cm_write_reg(reg_val, div_addr);
	wmb();

	clk->rate = clk->parent->rate / new_div;

	if (clk->flags & DELAYED_APP) {
		prm_write_reg(OMAP24XX_VALID_CONFIG, OMAP24XX_PRCM_CLKCFG_CTRL);
		wmb();
	}

	return 0;
}


/* Set the clock rate for a clock source */
static int omap2_clk_set_rate(struct clk *clk, unsigned long rate)
{
	int ret = -EINVAL;

	pr_debug("clock: set_rate for clock %s to rate %ld\n", clk->name, rate);

	/* CONFIG_PARTICIPANT clocks are changed only in sets via the
	   rate table mechanism, driven by mpu_speed  */
	if (clk->flags & CONFIG_PARTICIPANT)
		return -EINVAL;

	/* dpll_ck, core_ck, virt_prcm_set; plus all clksel clocks */
	if (clk->set_rate != 0)
		ret = clk->set_rate(clk, rate);

	if (unlikely(ret == 0 && (clk->flags & RATE_PROPAGATES)))
		propagate_rate(clk);

	return ret;
}

/*
 * Converts encoded control register address into a full address
 * On error, *src_addr will be returned as 0.
 */
static u32 omap2_clksel_get_src_field(void __iomem **src_addr,
				      struct clk *src_clk, u32 *field_mask,
				      struct clk *clk, u32 *parent_div)
{
	const struct clksel *clks;
	const struct clksel_rate *clkr;

	*parent_div = 0;
	*src_addr = 0;

	clks = omap2_get_clksel_by_parent(clk, src_clk);
	if (clks == NULL)
		return 0;

	for (clkr = clks->rates; clkr->div; clkr++) {
		if (clkr->flags & (cpu_mask | DEFAULT_RATE))
			break; /* Found the default rate for this platform */
	}

	if (!clkr->div) {
		printk(KERN_ERR "clock: Could not find default rate for "
		       "clock %s parent %s\n", clk->name,
		       src_clk->parent->name);
		return 0;
	}

	/* Should never happen.  Add a clksel mask to the struct clk. */
	WARN_ON(clk->clksel_mask == 0);

	*field_mask = clk->clksel_mask;
	*src_addr = clk->clksel_reg;
	*parent_div = clkr->div;

	return clkr->val;
}

static int omap2_clk_set_parent(struct clk *clk, struct clk *new_parent)
{
	void __iomem *src_addr;
	u32 field_val, field_mask, reg_val, parent_div;

	if (unlikely(clk->flags & CONFIG_PARTICIPANT))
		return -EINVAL;

	if (!clk->clksel)
		return -EINVAL;

	field_val = omap2_clksel_get_src_field(&src_addr, new_parent,
					       &field_mask, clk, &parent_div);
	if (src_addr == 0)
		return -EINVAL;

	if (clk->usecount > 0)
		_omap2_clk_disable(clk);

	/* Set new source value (previous dividers if any in effect) */
	reg_val = __raw_readl(src_addr) & ~field_mask;
	reg_val |= (field_val << mask_to_shift(field_mask));
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

	/* CLKSEL clocks follow their parents' rates, divided by a divisor */
	clk->rate = new_parent->rate;

	if (parent_div > 0)
		clk->rate /= parent_div;

	pr_debug("clock: set parent of %s to %s (new rate %ld)\n",
		 clk->name, clk->parent->name, clk->rate);

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
	cur_rate = omap2_get_dpll_rate_24xx(&dpll_ck);

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

static u32 omap2_get_apll_clkin(void)
{
	u32 aplls, sclk = 0;

	aplls = cm_read_mod_reg(PLL_MOD, CM_CLKSEL1);
	aplls &= OMAP24XX_APLLS_CLKIN_MASK;
	aplls >>= OMAP24XX_APLLS_CLKIN_SHIFT;

	if (aplls == APLLS_CLKIN_19_2MHZ)
		sclk = 19200000;
	else if (aplls == APLLS_CLKIN_13MHZ)
		sclk = 13000000;
	else if (aplls == APLLS_CLKIN_12MHZ)
		sclk = 12000000;

	return sclk;
}

static u32 omap2_get_sysclkdiv(void)
{
	u32 div;

	div = prm_read_reg(OMAP24XX_PRCM_CLKSRC_CTRL);
	div &= OMAP_SYSCLKDIV_MASK;
	div >>= OMAP_SYSCLKDIV_SHIFT;

	return div;
}

static void omap2_osc_clk_recalc(struct clk *clk)
{
	clk->rate = omap2_get_apll_clkin() * omap2_get_sysclkdiv();
	propagate_rate(clk);
}

static void omap2_sys_clk_recalc(struct clk *clk)
{
	clk->rate = clk->parent->rate / omap2_get_sysclkdiv();
	propagate_rate(clk);
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

	recalculate_root_clocks();

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

	if (cpu_is_omap242x())
		cpu_mask = RATE_IN_242X;
	else if (cpu_is_omap2430())
		cpu_mask = RATE_IN_243X;

	clk_init(&omap2_clk_functions);

	omap2_osc_clk_recalc(&osc_ck);

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

	/* Check the MPU rate set by bootloader */
	clkrate = omap2_get_dpll_rate_24xx(&dpll_ck);
	for (prcm = rate_table; prcm->mpu_speed; prcm++) {
		if (!(prcm->flags & cpu_mask))
			continue;
		if (prcm->xtal_speed != sys_ck.rate)
			continue;
		if (prcm->dpll_speed <= clkrate)
			 break;
	}
	curr_prcm_set = prcm;

	recalculate_root_clocks();

	printk(KERN_INFO "Clocking rate (Crystal/DPLL/MPU): "
	       "%ld.%01ld/%ld/%ld MHz\n",
	       (sys_ck.rate / 1000000), (sys_ck.rate / 100000) % 10,
	       (dpll_ck.rate / 1000000), (mpu_ck.rate / 1000000)) ;

	/*
	 * Only enable those clocks we will need, let the drivers
	 * enable other clocks as necessary
	 */
	clk_enable_init_clocks();

	/* Avoid sleeping sleeping during omap2_clk_prepare_for_reboot() */
	vclk = clk_get(NULL, "virt_prcm_set");
	sclk = clk_get(NULL, "sys_ck");

	return 0;
}
