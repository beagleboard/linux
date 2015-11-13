/*
 * Voltage Domain header for users of voltage domain interface
 *
 * Copyright (C) 2013 Linaro Ltd <mturquette@linaro.org>
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __PM_VOLTAGE_DOMAIN__
#define __PM_VOLTAGE_DOMAIN__

#include <linux/clk.h>

#if defined(CONFIG_VOLTAGE_DOMAIN)
struct notifier_block *of_pm_voltdm_notifier_register(struct device *dev,
						      struct device_node *np,
						      struct clk *clk,
						      const char *supply,
						      int *voltage_latency);
void of_pm_voltdm_notifier_unregister(struct notifier_block *nb);

#else
static inline struct notifier_block *of_pm_voltdm_notifier_register(
						      struct device *dev,
						      struct device_node *np,
						      struct clk *clk,
						      const char *supply,
						      int *voltage_latency);
{
	return ERR_PTR(-ENODEV);
}

static inline void of_pm_voltdm_notifier_unregister(struct notifier_block *nb)
{
}

#endif				/* VOLTAGE_DOMAIN */

#endif				/* __PM_VOLTAGE_DOMAIN__ */
