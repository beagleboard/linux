/*
 * Copyright (C) 2013 Linaro Ltd <mturquette@linaro.org>
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Helper functions for registering clock rate-change notifier handlers
 * that scale voltage when a clock changes its output frequency.
 */
#include <linux/device.h>
#include <linux/of.h>
#include <linux/pm_opp.h>
#include <linux/pm_voltage_domain.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

/**
 * struct voltdm_scale_data - Internal structure to maintain notifier information
 * @dev:	device on behalf of which we register the notifier
 * @clk:	clk on which we registered the notifier
 * @reg:	regulator if any which is used for scaling voltage
 * @tol:	voltage tolerance in %
 * @nb:		notifier block pointer
 */
struct voltdm_scale_data {
	struct device *dev;
	struct clk *clk;
	struct regulator *reg;
	int tol;
	struct notifier_block nb;
};

#define to_voltdm_scale_data(_nb) container_of(_nb, \
		struct voltdm_scale_data, nb)

static int clk_voltdm_notifier_handler(struct notifier_block *nb,
				       unsigned long flags, void *data)
{
	struct clk_notifier_data *cnd = data;
	struct voltdm_scale_data *vsd = to_voltdm_scale_data(nb);
	int ret, volt, tol;
	struct dev_pm_opp *opp;
	unsigned long old_rate = cnd->old_rate;
	unsigned long new_rate = cnd->new_rate;

	if ((new_rate < old_rate && flags == PRE_RATE_CHANGE) ||
	    (new_rate > old_rate && flags == POST_RATE_CHANGE))
		return NOTIFY_OK;

	rcu_read_lock();
	if (flags != ABORT_RATE_CHANGE)
		opp = dev_pm_opp_find_freq_ceil(vsd->dev, &new_rate);
	else
		opp = dev_pm_opp_find_freq_ceil(vsd->dev, &old_rate);
	if (IS_ERR(opp)) {
		rcu_read_unlock();
		dev_err(vsd->dev, "%s: Failed to find OPP for %lu\n",
			__func__, new_rate);
		return notifier_from_errno(PTR_ERR(opp));
	}

	volt = dev_pm_opp_get_voltage(opp);
	rcu_read_unlock();

	tol = volt * vsd->tol / 100;

	dev_dbg(vsd->dev, "%s: %lu -> %lu, V=%d, tol=%d, clk_flag=%lu\n",
		__func__, old_rate, new_rate, volt, tol, flags);

	ret = regulator_set_voltage_tol(vsd->reg, volt, tol);
	if (ret) {
		dev_err(vsd->dev,
			"%s: Failed to scale voltage(%u): %d\n", __func__,
			volt, ret);
		return notifier_from_errno(ret);
	}

	return NOTIFY_OK;
}

/**
 * of_pm_voltdm_notifier_register() - register voltage domain notifier
 * @dev:	device for which to register notifier for
 * @np:		node pointer of the device for which we register
 * @clk:	clk pointer around which the notifier is expected to trigger
 * @supply:	default regulator supply name(regulator id string)
 * @voltage_latency:	returns the latency for the voltage domain
 *
 * Return: notifier block which is registered with the common clock framework's
 * notifier for the clk node requested.
 */
struct notifier_block *of_pm_voltdm_notifier_register(struct device *dev,
						      struct device_node *np,
						      struct clk *clk,
						      const char *supply,
						      int *voltage_latency)
{
	struct voltdm_scale_data *vsd;
	struct dev_pm_opp *opp;
	unsigned long min, max, freq;
	int ret;

	vsd = kzalloc(sizeof(*vsd), GFP_KERNEL);
	if (!vsd)
		return ERR_PTR(-ENOMEM);

	vsd->dev = dev;
	vsd->clk = clk;
	vsd->nb.notifier_call = clk_voltdm_notifier_handler;
	vsd->reg = regulator_get_optional(dev, supply);
	ret = 0;
	if (IS_ERR(vsd->reg))
		ret = PTR_ERR(vsd->reg);
	/* regulator is not mandatory */
	if (ret != -EPROBE_DEFER) {
		dev_warn(dev, "%s: Failed to get %s regulator:%d\n",
			 __func__, supply, ret);
		ret = 0;
		goto err_free_vsd;
	}
	/* For devices that are not ready.... */
	if (ret)
		goto err_free_vsd;

	rcu_read_lock();
	freq = 0;
	opp = dev_pm_opp_find_freq_ceil(dev, &freq);
	if (IS_ERR(opp))
		goto err_bad_opp;
	min = dev_pm_opp_get_voltage(opp);

	freq = ULONG_MAX;
	opp = dev_pm_opp_find_freq_floor(dev, &freq);
	if (IS_ERR(opp))
		goto err_bad_opp;
	max = dev_pm_opp_get_voltage(opp);
	rcu_read_unlock();

	*voltage_latency = regulator_set_voltage_time(vsd->reg, min, max);
	if (*voltage_latency < 0) {
		dev_warn(dev,
			 "%s: Fail calculating voltage latency[%ld<->%ld]:%d\n",
			 __func__, min, max, *voltage_latency);
	}

	of_property_read_u32(np, "voltage-tolerance", &vsd->tol);

	ret = clk_notifier_register(clk, &vsd->nb);

	if (ret) {
		dev_err(dev, "%s: Failed to Register Notifier, %d\n", __func__,
			ret);
		goto err_free_reg;
	}

	return &vsd->nb;

err_bad_opp:
	rcu_read_unlock();
	ret = PTR_ERR(opp);
	dev_err(dev, "%s: failed to get OPP, %d\n", __func__, ret);

err_free_reg:
	regulator_put(vsd->reg);

err_free_vsd:
	kfree(vsd);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(of_pm_voltdm_notifier_register);

/**
 * of_pm_voltdm_notifier_unregister() - unregister notifier for volt domain
 * @nb:	notifier block returned by of_pm_voltdm_notifier_register
 */
void of_pm_voltdm_notifier_unregister(struct notifier_block *nb)
{
	struct voltdm_scale_data *vsd;
	struct clk *clk;

	/* if caller send us back error value */
	if (IS_ERR(nb))
		return;

	vsd = to_voltdm_scale_data(nb);
	clk = vsd->clk;
	clk_notifier_unregister(clk, nb);
	if (!IS_ERR(vsd->reg))
		regulator_put(vsd->reg);

	kfree(vsd);
}
EXPORT_SYMBOL_GPL(of_pm_voltdm_notifier_unregister);
