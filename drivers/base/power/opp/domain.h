/*
 * Generic OPP Domain Private
 *
 * Copyright (C) 2016 Texas Instruments Incorporated.
 *	Dave Gerlach
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __DRIVER_OPP_DOMAIN_H__
#define __DRIVER_OPP_DOMAIN_H__

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/pm_opp.h>

/**
 * struct pm_opp_domain - Structure to maintain opp domain info
 * @dev:        device for which we scale clk and supply
 * @clk:        clk which we must scale
 * @reg:        regulator, if any, which is used for scaling voltage
 * @data: opp_domain driver specific data
 */
struct pm_opp_domain {
	struct device *dev;
	struct clk *clk;
	struct regulator *reg;
	void *data;
};

#if defined(CONFIG_PM_OPP)
int dev_pm_opp_domain_set_rate(struct pm_opp_domain *pod,
			       unsigned long target_freq);

struct pm_opp_domain *dev_pm_opp_domain_get(struct device *dev);
int dev_pm_opp_domain_put(struct pm_opp_domain *pod);

int dev_pm_opp_domain_get_supply(struct pm_opp_domain *pod,
				 const char *supply);
void dev_pm_opp_domain_put_supply(struct pm_opp_domain *pod);

int dev_pm_opp_domain_get_latency(struct pm_opp_domain *pod, int old_uV,
				  int old_uV_min, int old_uV_max, int new_uV,
				  int new_uV_min, int new_uV_max);
bool dev_pm_opp_domain_opp_supported_by_supply(struct pm_opp_domain *pod,
					       unsigned long uV_min,
					       unsigned long uV_max);
#else
static inline int dev_pm_opp_domain_set_rate(struct pm_opp_domain *pod,
					     unsigned long target_freq)
{
	return -ENODEV;
}

static inline struct pm_opp_domain *dev_pm_opp_domain_get(struct device *dev,
							  const char *supply)
{
	return NULL;
}

static inline void dev_pm_opp_domain_put(struct pm_opp_domain *pod)
{
}

static inline int dev_pm_opp_domain_get_supply(struct pm_opp_domain *pod,
					       const char *supply)
{
	return -ENODEV;
}

static inline void dev_pm_opp_domain_put_supply(struct pm_opp_domain *pod)
{
}

static inline int dev_pm_opp_domain_get_latency(struct pm_opp_domain *pod,
						int old_uV, int old_uV_min,
						int old_uV_max, int new_uV,
						int new_uV_min, int new_uV_max)

{
	return -ENODEV;
}

static inline
bool dev_pm_opp_domain_opp_supported_by_supply(struct pm_opp_domain *pod,
					       unsigned long uV_min,
					       unsigned long uV_max)
{
	return false;
}
#endif /* CONFIG_PM_OPP */
#endif /* __DRIVER_OPP_DOMAIN_H__ */
