/*
 * OPP Domain Core interface
 *
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * OPP domain handles scaling of clocks and regulators for a device when
 * changing OPPs. Default configuration is one clock and one regulator per
 * device.
 *
 */
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

#include "opp.h"
#include "domain.h"

static int _set_opp_domain_voltage(struct device *dev, struct regulator *reg,
				   unsigned long u_volt,
				   unsigned long u_volt_min,
				   unsigned long u_volt_max)
{
	int ret;

	/* Regulator not available for device */
	if (IS_ERR(reg)) {
		dev_dbg(dev, "%s: regulator not available: %ld\n", __func__,
			PTR_ERR(reg));
		return 0;
	}

	dev_dbg(dev, "%s: voltages (mV): %lu %lu %lu\n", __func__, u_volt_min,
		u_volt, u_volt_max);

	ret = regulator_set_voltage_triplet(reg, u_volt_min, u_volt,
					    u_volt_max);
	if (ret)
		dev_err(dev, "%s: failed to set voltage (%lu %lu %lu mV): %d\n",
			__func__, u_volt_min, u_volt, u_volt_max, ret);

	return ret;
}

/**
 * dev_pm_opp_domain_set_rate() - Set voltage and frequency for an opp domain
 * @pod: Pointer to pm_opp_domain obtained with dev_pm_opp_domain_get
 * @target_freq: Desired frequency to transition to in Hz
 *
 * Transition the clk and reg managed by opp_domain to the new operating point.
 */
int dev_pm_opp_domain_set_rate(struct pm_opp_domain *pod,
			       unsigned long target_freq)
{
	struct device *dev = pod->dev;
	struct dev_pm_opp *old_opp, *opp;
	struct regulator *reg;
	struct clk *clk;
	unsigned long freq, old_freq;
	unsigned long u_volt, u_volt_min, u_volt_max;
	unsigned long ou_volt, ou_volt_min, ou_volt_max;
	int ret;

	if (unlikely(!target_freq)) {
		dev_err(dev, "%s: Invalid target frequency %lu\n", __func__,
			target_freq);
		return -EINVAL;
	}

	clk = pod->clk;
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	freq = clk_round_rate(clk, target_freq);
	if ((long)freq <= 0)
		freq = target_freq;

	old_freq = clk_get_rate(clk);

	/* Return early if nothing to do */
	if (old_freq == freq) {
		dev_dbg(dev, "%s: old/new frequencies (%lu Hz) are same, nothing to do\n",
			__func__, freq);
		return 0;
	}

	rcu_read_lock();

	old_opp = dev_pm_opp_find_freq_ceil(dev, &old_freq);
	if (!IS_ERR(old_opp)) {
		ou_volt = old_opp->u_volt;
		ou_volt_min = old_opp->u_volt_min;
		ou_volt_max = old_opp->u_volt_max;
	} else {
		dev_err(dev, "%s: failed to find current OPP for freq %lu (%ld)\n",
			__func__, old_freq, PTR_ERR(old_opp));
	}

	opp = dev_pm_opp_find_freq_ceil(dev, &freq);
	if (IS_ERR(opp)) {
		ret = PTR_ERR(opp);
		dev_err(dev, "%s: failed to find OPP for freq %lu (%d)\n",
			__func__, freq, ret);
		rcu_read_unlock();
		return ret;
	}

	u_volt = opp->u_volt;
	u_volt_min = opp->u_volt_min;
	u_volt_max = opp->u_volt_max;

	reg = pod->reg;

	rcu_read_unlock();

	/* Scaling up? Scale voltage before frequency */
	if (freq > old_freq) {
		ret = _set_opp_domain_voltage(dev, reg, u_volt, u_volt_min,
					      u_volt_max);
		if (ret)
			goto restore_voltage;
	}

	/* Change frequency */

	dev_dbg(dev, "%s: switching OPP: %lu Hz --> %lu Hz\n",
		__func__, old_freq, freq);

	ret = clk_set_rate(clk, freq);
	if (ret) {
		dev_err(dev, "%s: failed to set clock rate: %d\n", __func__,
			ret);
		goto restore_voltage;
	}

	/* Scaling down? Scale voltage after frequency */
	if (freq < old_freq) {
		ret = _set_opp_domain_voltage(dev, reg, u_volt, u_volt_min,
					      u_volt_max);
		if (ret)
			goto restore_freq;
	}

	return 0;

restore_freq:
	if (clk_set_rate(clk, old_freq))
		dev_err(dev, "%s: failed to restore old-freq (%lu Hz)\n",
			__func__, old_freq);
restore_voltage:
	/* This shouldn't harm even if the voltages weren't updated earlier */
	if (!IS_ERR(old_opp))
		_set_opp_domain_voltage(dev, reg, ou_volt, ou_volt_min,
					ou_volt_max);

	return ret;
}

/**
 * dev_pm_opp_domain_get() - Get a handle to opp_domain for device
 * @dev: Pointer to the device for which opp_domain handle is needed
 *
 * This creates a pm_opp_domain and parses proper clock for the
 * device so it can be managed. Will defer if resources are
 * not yet available.
 */
struct pm_opp_domain *dev_pm_opp_domain_get(struct device *dev)
{
	struct pm_opp_domain *pod;
	int ret = 0;

	pod = kzalloc(sizeof(*pod), GFP_KERNEL);
	if (!pod)
		return ERR_PTR(-ENOMEM);

	pod->dev = dev;

	/* Find clk for the device */
	pod->clk = clk_get(dev, NULL);
	if (IS_ERR(pod->clk)) {
		ret = PTR_ERR(pod->clk);
		if (ret != -EPROBE_DEFER)
			dev_dbg(dev, "%s: Couldn't find clock: %d\n", __func__,
				ret);
		goto err;
	}

	return pod;
err:
	kfree(pod);
	return ERR_PTR(ret);
}

/**
 * dev_pm_opp_domain_put() - Release an opp_domain
 * @pod: Pointer to the opp_domain to be released
 *
 * Returns -EBUSY if regulator hasn't been released, otherwise 0
 */
int dev_pm_opp_domain_put(struct pm_opp_domain *pod)
{
	if (!IS_ERR_OR_NULL(pod->reg)) {
		dev_dbg(pod->dev, "%s called without putting regulator first\n",
			__func__);
		return -EBUSY;
	}

	if (!IS_ERR(pod->reg))
		regulator_put(pod->reg);

	if (!IS_ERR(pod->clk))
		clk_put(pod->clk);

	kfree(pod);
	return 0;
}

/**
 * dev_pm_opp_domain_get_supply() - Acquire supply for opp_domain
 * @pod: Pointer to valid pm_opp_domain acquired with dev_pm_opp_domain_get
 * @supply: Name of the supply.
 *
 * This parses proper regulator for the device so it can be managed. Will defer
 * if resource is not yet available.
 *
 * NOTE: This is called under mutex protection by the OPP core.
 */
int dev_pm_opp_domain_get_supply(struct pm_opp_domain *pod,
				 const char *supply)
{
	int ret = 0;

	if (!pod)
		return -EINVAL;

	pod->reg = regulator_get_optional(pod->dev, supply);
	if (IS_ERR(pod->reg)) {
		ret = PTR_ERR(pod->reg);
		/* Regulator is not mandatory */
		if (ret != -EPROBE_DEFER)
			dev_dbg(pod->dev, "%s: Couldn't find regulator: %d\n",
				__func__, ret);
	}

	return ret;
}

/**
 * dev_pm_opp_domain_put_supply() - Release an opp_domain supply
 * @pod: Pointer to the opp_domain to release supply for
 *
 * NOTE: This is called under mutex protection by the OPP core.
 */
void dev_pm_opp_domain_put_supply(struct pm_opp_domain *pod)
{
	if (!pod)
		return;

	if (!IS_ERR_OR_NULL(pod->reg)) {
		regulator_put(pod->reg);
		pod->reg = NULL;
	}
}

/**
 * dev_pm_opp_domain_get_latency() - provide the latency for the opp domain
 * @pod: Pointer to valid pm_opp_domain acquired with dev_pm_opp_domain_get
 * @old_uV: starting voltage in microvolts
 * @old_uV_min: minimum acceptable starting voltage in microvolts
 * @old_uV_max: maximum acceptable starting voltage in microvolts
 * @new_uV: target voltage in microvolts
 * @new_uV_min: minimum acceptable target voltage in microvolts
 * @new_uV_max: maximum acceptable target voltage in microvolts
 *
 * Return: If successful, the combined transition latency from min to max, else
 * returns error value
 */
int dev_pm_opp_domain_get_latency(struct pm_opp_domain *pod, int old_uV,
				  int old_uV_min, int old_uV_max, int new_uV,
				  int new_uV_min, int new_uV_max)
{
	int total_latency = 0;

	if (!IS_ERR(pod->reg)) {
		total_latency +=
			 regulator_set_voltage_time_triplet(pod->reg,
							    old_uV,
							    old_uV_min,
							    old_uV_max,
							    new_uV, new_uV_min,
							    new_uV_max);
	}

	return total_latency;
}

/**
 * dev_pm_opp_domain_opp_supported_by_supply() - opp domain supports voltage?
 * @pod: Pointer to valid pm_opp_domain acquired with dev_pm_opp_domain_get
 * @uV_min: minimum acceptable starting voltage in microvolts
 * @uV_max: maximum acceptable starting voltage in microvolts
 *
 * NOTE: This is called under mutex protection by the OPP core.
 *
 * Returns: If voltage is supported return true, else return false.
 */
bool dev_pm_opp_domain_opp_supported_by_supply(struct pm_opp_domain *pod,
					       unsigned long uV_min,
					       unsigned long uV_max)
{
	if (!IS_ERR(pod->reg) &&
	    !regulator_is_supported_voltage(pod->reg, uV_min,
					    uV_max))
		return false;

	return true;
}
