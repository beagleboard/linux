/*
 * Nokia N800 PM code
 *
 * Copyright (C) 2006 Nokia Corporation
 * Author: Amit Kucheria <amit.kucheria@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/arch/menelaus.h>

#ifdef CONFIG_MENELAUS

static int n800_auto_sleep_regulators(void)
{
	u32 val;
	int ret;

	val = EN_VPLL_SLEEP | EN_VMMC_SLEEP    \
		| EN_VAUX_SLEEP | EN_VIO_SLEEP \
		| EN_VMEM_SLEEP | EN_DC3_SLEEP \
		| EN_VC_SLEEP | EN_DC2_SLEEP;

	ret = menelaus_set_regulator_sleep(1, val);
	if (ret < 0) {
		printk(KERN_ERR "Could not set regulators to sleep on "
			"menelaus: %u\n", ret);
		return ret;
	}
	return 0;
}

static int n800_auto_voltage_scale(void)
{
	int ret;

	ret = menelaus_set_vcore_hw(1400, 1050);
	if (ret < 0) {
		printk(KERN_ERR "Could not set VCORE voltage on "
			"menelaus: %u\n", ret);
		return ret;
	}
	return 0;
}

static int n800_menelaus_init(struct device *dev)
{
	int ret;

	ret = n800_auto_voltage_scale();
	if (ret < 0)
		return ret;
	ret = n800_auto_sleep_regulators();
	if (ret < 0)
		return ret;
	return 0;
}

static struct menelaus_platform_data n800_menelaus_platform_data = {
	.late_init = n800_menelaus_init,
};

void __init n800_pm_init(void)
{
	menelaus_set_platform_data(&n800_menelaus_platform_data);
}

#else

void __init n800_pm_init(void)
{
}

#endif

