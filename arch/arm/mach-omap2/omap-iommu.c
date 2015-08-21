/*
 * omap iommu: omap device registration
 *
 * Copyright (C) 2008-2009 Nokia Corporation
 *
 * Written by Hiroshi DOYU <Hiroshi.DOYU@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/of.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/slab.h>

#include <linux/platform_data/iommu-omap.h>
#include "soc.h"
#include "omap_hwmod.h"
#include "omap_device.h"
#include "clockdomain.h"
#include "powerdomain.h"

static void omap_iommu_dra7_emu_swsup_config(struct platform_device *pdev,
					     bool enable)
{
	static struct clockdomain *emu_clkdm;
	static DEFINE_MUTEX(emu_lock);
	static atomic_t count;
	struct device_node *np = pdev->dev.of_node;
	struct resource *res;

	if (!of_device_is_compatible(np, "ti,dra7-iommu"))
		return;

	/* process this only for DRA7 DSP MDMA MMUs */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res->start != 0x40D01000 && res->start != 0x41501000)
		return;

	if (!emu_clkdm) {
		emu_clkdm = clkdm_lookup("emu_clkdm");
		if (WARN_ON_ONCE(!emu_clkdm))
			return;
	}

	mutex_lock(&emu_lock);

	if (enable && (atomic_inc_return(&count) == 1))
		clkdm_deny_idle(emu_clkdm);
	else if (!enable && (atomic_dec_return(&count) == 0))
		clkdm_allow_idle(emu_clkdm);

	mutex_unlock(&emu_lock);
}

int omap_iommu_set_pwrdm_constraint(struct platform_device *pdev, bool request,
				    u8 *pwrst)
{
	struct powerdomain *pwrdm;
	struct omap_device *od;
	u8 next_pwrst;
	int ret = 0;

	od = to_omap_device(pdev);
	if (!od)
		return -ENODEV;

	if (od->hwmods_cnt != 1)
		return -EINVAL;

	pwrdm = omap_hwmod_get_pwrdm(od->hwmods[0]);
	if (!pwrdm)
		return -EINVAL;

	if (request) {
		*pwrst = pwrdm_read_next_pwrst(pwrdm);
		omap_iommu_dra7_emu_swsup_config(pdev, true);
	}

	if (*pwrst > PWRDM_POWER_RET)
		goto out;

	next_pwrst = request ? PWRDM_POWER_ON : *pwrst;

	ret = pwrdm_set_next_pwrst(pwrdm, next_pwrst);

out:
	if (!request)
		omap_iommu_dra7_emu_swsup_config(pdev, false);

	return ret;
}

static int __init omap_iommu_dev_init(struct omap_hwmod *oh, void *unused)
{
	struct platform_device *pdev;
	struct iommu_platform_data *pdata;
	struct omap_mmu_dev_attr *a = (struct omap_mmu_dev_attr *)oh->dev_attr;
	static int i;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	pdata->name = oh->name;
	pdata->nr_tlb_entries = a->nr_tlb_entries;
	pdata->da_start = a->da_start;
	pdata->da_end = a->da_end;

	if (oh->rst_lines_cnt == 1) {
		pdata->reset_name = oh->rst_lines->name;
		pdata->assert_reset = omap_device_assert_hardreset;
		pdata->deassert_reset = omap_device_deassert_hardreset;
	}
	pdata->device_enable = omap_device_enable,
	pdata->device_idle = omap_device_idle,

	pdev = omap_device_build("omap-iommu", i, oh, pdata, sizeof(*pdata));

	kfree(pdata);

	if (IS_ERR(pdev)) {
		pr_err("%s: device build err: %ld\n", __func__, PTR_ERR(pdev));
		return PTR_ERR(pdev);
	}

	i++;

	return 0;
}

static int __init omap_iommu_init(void)
{
	/* If dtb is there, the devices will be created dynamically */
	if (of_have_populated_dt())
		return -ENODEV;

	return omap_hwmod_for_each_by_class("mmu", omap_iommu_dev_init, NULL);
}
/* must be ready before omap3isp is probed */
omap_subsys_initcall(omap_iommu_init);

static void __exit omap_iommu_exit(void)
{
	/* Do nothing */
}
module_exit(omap_iommu_exit);

MODULE_AUTHOR("Hiroshi DOYU");
MODULE_DESCRIPTION("omap iommu: omap device registration");
MODULE_LICENSE("GPL v2");
