// SPDX-License-Identifier: GPL-2.0-only
/*
 * OMAP IOMMU quirks for various TI SoCs
 *
 * Copyright (C) 2015-2019 Texas Instruments Incorporated - http://www.ti.com/
 *      Suman Anna <s-anna@ti.com>
 */

#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/list.h>

#include "clockdomain.h"
#include "powerdomain.h"

struct pwrdm_link {
	struct device *dev;
	struct powerdomain *pwrdm;
	struct list_head node;
};

static DEFINE_SPINLOCK(iommu_lock);

static struct powerdomain *_get_pwrdm(struct device *dev)
{
	struct clk *clk;
	struct clk_hw_omap *hwclk;
	struct clockdomain *clkdm;
	struct powerdomain *pwrdm = NULL;
	struct pwrdm_link *entry;
	unsigned long flags;
	static LIST_HEAD(cache);

	spin_lock_irqsave(&iommu_lock, flags);

	list_for_each_entry(entry, &cache, node) {
		if (entry->dev == dev) {
			pwrdm = entry->pwrdm;
			break;
		}
	}

	spin_unlock_irqrestore(&iommu_lock, flags);

	if (pwrdm)
		return pwrdm;

	clk = of_clk_get(dev->of_node->parent, 0);
	if (!clk) {
		dev_err(dev, "no fck found\n");
		return NULL;
	}

	hwclk = to_clk_hw_omap(__clk_get_hw(clk));
	clk_put(clk);
	if (!hwclk || !hwclk->clkdm_name) {
		dev_err(dev, "no hwclk data\n");
		return NULL;
	}

	clkdm = clkdm_lookup(hwclk->clkdm_name);
	if (!clkdm) {
		dev_err(dev, "clkdm not found: %s\n", hwclk->clkdm_name);
		return NULL;
	}

	pwrdm = clkdm_get_pwrdm(clkdm);
	if (!pwrdm) {
		dev_err(dev, "pwrdm not found: %s\n", clkdm->name);
		return NULL;
	}

	entry = kmalloc(sizeof(*entry), GFP_KERNEL);
	if (entry) {
		entry->dev = dev;
		entry->pwrdm = pwrdm;
		spin_lock_irqsave(&iommu_lock, flags);
		list_add(&entry->node, &cache);
		spin_unlock_irqrestore(&iommu_lock, flags);
	}

	return pwrdm;
}

int omap_iommu_set_pwrdm_constraint(struct platform_device *pdev, bool request,
				    u8 *pwrst)
{
	struct powerdomain *pwrdm;
	u8 next_pwrst;

	pwrdm = _get_pwrdm(&pdev->dev);
	if (!pwrdm)
		return -ENODEV;

	if (request)
		*pwrst = pwrdm_read_next_pwrst(pwrdm);

	if (*pwrst > PWRDM_POWER_RET)
		return 0;

	next_pwrst = request ? PWRDM_POWER_ON : *pwrst;

	return pwrdm_set_next_pwrst(pwrdm, next_pwrst);
}
