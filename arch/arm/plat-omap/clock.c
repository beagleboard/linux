/*
 *  linux/arch/arm/plat-omap/clock.c
 *
 *  Copyright (C) 2004 - 2005 Nokia corporation
 *  Written by Tuukka Tikkanen <tuukka.tikkanen@elektrobit.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/err.h>

#include <asm/io.h>
#include <asm/semaphore.h>
#include <asm/hardware/clock.h>

#include <asm/arch/clock.h>

static LIST_HEAD(clocks);
static DECLARE_MUTEX(clocks_sem);
static DEFINE_SPINLOCK(clockfw_lock);

static struct clk_functions *arch_clock;

/*-------------------------------------------------------------------------
 * Standard clock functions defined in asm/hardware/clock.h
 *-------------------------------------------------------------------------*/

struct clk * clk_get(struct device *dev, const char *id)
{
	struct clk *p, *clk = ERR_PTR(-ENOENT);

	down(&clocks_sem);
	list_for_each_entry(p, &clocks, node) {
		if (strcmp(id, p->name) == 0 && try_module_get(p->owner)) {
			clk = p;
			break;
		}
	}
	up(&clocks_sem);

	return clk;
}
EXPORT_SYMBOL(clk_get);

int clk_enable(struct clk *clk)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&clockfw_lock, flags);
	ret = clk->enable(clk);
	spin_unlock_irqrestore(&clockfw_lock, flags);
	return ret;
}
EXPORT_SYMBOL(clk_enable);

void clk_disable(struct clk *clk)
{
	unsigned long flags;

	spin_lock_irqsave(&clockfw_lock, flags);
	clk->disable(clk);
	spin_unlock_irqrestore(&clockfw_lock, flags);
}
EXPORT_SYMBOL(clk_disable);

int clk_use(struct clk *clk)
{
	unsigned long flags;
	int ret = 0;

	spin_lock_irqsave(&clockfw_lock, flags);
	if (arch_clock->clk_use)
		ret = arch_clock->clk_use(clk);
	spin_unlock_irqrestore(&clockfw_lock, flags);
	return ret;
}
EXPORT_SYMBOL(clk_use);

void clk_unuse(struct clk *clk)
{
	unsigned long flags;

	spin_lock_irqsave(&clockfw_lock, flags);
	if (arch_clock->clk_unuse)
		arch_clock->clk_unuse(clk);
	spin_unlock_irqrestore(&clockfw_lock, flags);
}
EXPORT_SYMBOL(clk_unuse);

int clk_get_usecount(struct clk *clk)
{
        return clk->usecount;
}
EXPORT_SYMBOL(clk_get_usecount);

unsigned long clk_get_rate(struct clk *clk)
{
	return clk->rate;
}
EXPORT_SYMBOL(clk_get_rate);

void clk_put(struct clk *clk)
{
	if (clk && !IS_ERR(clk))
		module_put(clk->owner);
}
EXPORT_SYMBOL(clk_put);

/*-------------------------------------------------------------------------
 * Optional clock functions defined in asm/hardware/clock.h
 *-------------------------------------------------------------------------*/

long clk_round_rate(struct clk *clk, unsigned long rate)
{
	if (arch_clock->clk_round_rate)
		return arch_clock->clk_round_rate(clk, rate);

	return 0;
}
EXPORT_SYMBOL(clk_round_rate);

int clk_set_rate(struct clk *clk, unsigned long rate)
{
	if (arch_clock->clk_set_rate)
		return arch_clock->clk_set_rate(clk, rate);

	return 0;
}
EXPORT_SYMBOL(clk_set_rate);

int clk_set_parent(struct clk *clk, struct clk *parent)
{
	if (arch_clock->clk_set_parent)
		return arch_clock->clk_set_parent(clk, parent);

	return 0;
}
EXPORT_SYMBOL(clk_set_parent);

struct clk *clk_get_parent(struct clk *clk)
{
	if (arch_clock->clk_get_parent)
		return arch_clock->clk_get_parent(clk);

	return 0;
}
EXPORT_SYMBOL(clk_get_parent);

/*-------------------------------------------------------------------------
 * OMAP specific clock functions shared between omap1 and omap2
 *-------------------------------------------------------------------------*/

/* Used for clocks that always have same value as the parent clock */
void followparent_recalc(struct clk *  clk)
{
	clk->rate = clk->parent->rate;
}

void propagate_rate(struct clk * tclk)
{
	struct clk *clkp;

	list_for_each_entry(clkp, &clocks, node) {
		if (likely(clkp->parent != tclk))
			continue;
		if (likely((u32)clkp->recalc))
			clkp->recalc(clkp);
	}
}

int clk_register(struct clk *clk)
{
	down(&clocks_sem);
	list_add(&clk->node, &clocks);
	if (clk->init)
		clk->init(clk);
	up(&clocks_sem);
	return 0;
}
EXPORT_SYMBOL(clk_register);

void clk_unregister(struct clk *clk)
{
	down(&clocks_sem);
	list_del(&clk->node);
	up(&clocks_sem);
}
EXPORT_SYMBOL(clk_unregister);

void clk_deny_idle(struct clk *clk)
{
	unsigned long flags;

	spin_lock_irqsave(&clockfw_lock, flags);
	if (arch_clock->clk_deny_idle)
		arch_clock->clk_deny_idle(clk);
	spin_unlock_irqrestore(&clockfw_lock, flags);
}
EXPORT_SYMBOL(clk_deny_idle);

void clk_allow_idle(struct clk *clk)
{
	unsigned long flags;

	spin_lock_irqsave(&clockfw_lock, flags);
	if (arch_clock->clk_allow_idle)
		arch_clock->clk_allow_idle(clk);
	spin_unlock_irqrestore(&clockfw_lock, flags);
}
EXPORT_SYMBOL(clk_allow_idle);

/*-------------------------------------------------------------------------*/

int __init clk_init(struct clk_functions * custom_clocks)
{
	if (!custom_clocks) {
		printk(KERN_ERR "No custom clock functions registered\n");
		BUG();
	}

	arch_clock = custom_clocks;

	return 0;
}
