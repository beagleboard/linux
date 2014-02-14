/*
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Helper functions for registering clock rate-change notifier handlers
 * that scale voltage when a clock changes its output frequency.
 */
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include "voltage_domain_private.h"

/**
 * struct omap_voltdm_data - OMAP specific voltage domain data
 * @vdd_reg:	VDD regulator
 * @vbb_reg:	Body Bias regulator
 */
struct omap_voltdm_data {
	struct regulator *vdd_reg;
	struct regulator *vbb_reg;
};

/**
 * struct omap_voltdm_of_data - device tree match data
 * @desc:	voltage domain descriptor for voltage domain core
 */
struct omap_voltdm_of_data {
	const struct pm_voltdm_desc *desc;
};

/**
 * omap_voltdm_do_transition() - do the voltage domain transition
 * @dev:	voltage domain device for which we are doing the transition
 * @voltdm_data:	data specific to the device
 * @clk_notifier_flags:	clk notifier flags for direction of transition
 * @uv:		what voltage to transition to
 * @tol_uv:	voltage tolerance to use
 *
 * Return: If successful, 0, else appropriate error value.
 */
static int omap_voltdm_do_transition(struct device *dev,
				     void *voltdm_data,
				     unsigned long clk_notifier_flags, int uv,
				     int tol_uv)
{
	struct omap_voltdm_data *data = (struct omap_voltdm_data *)voltdm_data;
	int ret;
	bool do_abb_first;

	/* We dont expect voltdm layer to make mistakes.. but still */
	BUG_ON(!data);

	do_abb_first = clk_notifier_flags == ABORT_RATE_CHANGE ||
	    clk_notifier_flags == POST_RATE_CHANGE;

	if (do_abb_first) {
		dev_dbg(dev, "vbb pre %duV[tol %duV]\n", uv, tol_uv);
		ret = regulator_set_voltage_tol(data->vbb_reg, uv, tol_uv);
		if (ret) {
			dev_err(dev,
				"vbb failed for voltage %duV[tol %duV]:%d\n",
				uv, tol_uv, ret);
			return ret;
		}
	}

	dev_dbg(dev, "vdd for voltage %duV[tol %duV]\n", uv, tol_uv);
	ret = regulator_set_voltage_tol(data->vdd_reg, uv, tol_uv);
	if (ret) {
		dev_err(dev,
			"vdd failed for voltage %duV[tol %duV]:%d\n",
			uv, tol_uv, ret);
		return ret;
	}

	if (!do_abb_first) {
		dev_dbg(dev, "vbb post %duV[tol %duV]\n", uv, tol_uv);
		ret = regulator_set_voltage_tol(data->vbb_reg, uv, tol_uv);
		if (ret) {
			dev_err(dev,
				"vbb failed for voltage %duV[tol %duV]:%d\n",
				uv, tol_uv, ret);
			return ret;
		}
	}

	return 0;
}

/**
 * omap_voltdm_latency() - provide the transition latency for the voltage domain
 * @dev:	voltage domain device for which we are doing the transition
 * @voltdm_data: data specific to the device
 * @min:	minimum voltage in uV
 * @max:	maximum voltage in uV
 *
 * Return: If successful, the combined transition latency from min to max, else
 * returns error value
 */
static int omap_voltdm_latency(struct device *dev, void *voltdm_data,
			       unsigned long min, unsigned long max)
{
	struct omap_voltdm_data *data = (struct omap_voltdm_data *)voltdm_data;
	int ret, tot_latency = 0;

	/* We dont expect voltdm layer to make mistakes.. but still */
	BUG_ON(!data);

	/*
	 * NOTE: latency computations can depend on regulator involved
	 * We just provide the best of the information we can find.
	 */
	ret = regulator_set_voltage_time(data->vdd_reg, min, max);
	if (ret < 0) {
		dev_dbg(dev, "vdd failed voltage latency: %d\n", ret);
		goto skip_vdd;
	}
	tot_latency += ret;

skip_vdd:
	ret = regulator_set_voltage_time(data->vbb_reg, min, max);
	if (ret < 0) {
		dev_dbg(dev, "vbb failed voltage latency: %d\n", ret);
		goto skip_vbb;
	}
	tot_latency += ret;

skip_vbb:
	/* If we have No data at all.. return last error result */
	if (!tot_latency)
		return ret;

	return tot_latency;
}

static inline void omap_voltdm_cleanup(struct omap_voltdm_data *data)
{
	if (!IS_ERR(data->vbb_reg))
		regulator_put(data->vbb_reg);
	if (!IS_ERR(data->vdd_reg))
		regulator_put(data->vdd_reg);
	kfree(data);
}

/**
 * omap_voltdm_get() - get the voltage domain resources specific to request
 * @voltdm_dev:	voltage domain device
 * @request_dev:	device for which we have been requested to get
 * @np:			unused
 * @np_args:		unused
 * @supply:		unused
 * @voltdm_data:	returns data for the current request (freed in put)
 *
 * Return: 0 if everything went OK, else return appropriate error value.
 */
static int omap_voltdm_get(struct device *voltdm_dev,
			   struct device *request_dev,
			   struct device_node *np,
			   struct of_phandle_args *np_args,
			   const char *supply,
			   void **voltdm_data)
{
	struct omap_voltdm_data *data;
	int ret = 0;

	BUG_ON(!voltdm_dev || !request_dev || !voltdm_data);

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	/*
	 * Setup aliases for request device supply to let regulator framework
	 * do multi-consumer scenario
	 */
	ret = regulator_register_supply_alias(request_dev, "vdd",
					      voltdm_dev, "vdd");
	if (ret)
		goto out_free;
	ret = regulator_register_supply_alias(request_dev, "vbb",
					      voltdm_dev, "vbb");
	if (ret)
		goto out_unreg_vdd;

	data->vdd_reg = regulator_get(request_dev, "vdd");
	if (IS_ERR(data->vdd_reg)) {
		ret = PTR_ERR(data->vdd_reg);
		dev_err(voltdm_dev, "Unable to get vdd regulator:%d\n", ret);
		goto out_unreg;
	}

	data->vbb_reg = regulator_get(request_dev, "vbb");
	if (IS_ERR(data->vbb_reg)) {
		ret = PTR_ERR(data->vbb_reg);
		dev_err(voltdm_dev, "Unable to get vbb regulator:%d\n", ret);
		goto out_vdd_reg;
	}

	*voltdm_data = data;
	return 0;

out_vdd_reg:
	regulator_put(data->vdd_reg);
out_unreg:
	regulator_unregister_supply_alias(request_dev, "vbb");
out_unreg_vdd:
	regulator_unregister_supply_alias(request_dev, "vdd");
out_free:
	kfree(data);

	return ret;
}

/**
 * omap_voltdm_put() - release the resources reserved by omap_voltdm_get()
 * @voltdm_dev:		voltage domain device for which we reserved resources
 * @request_dev:	device for which we have been requested to put
 * @voltdm_data:	data for the request provided by omap_voltdm_get()
 */
static void omap_voltdm_put(struct device *voltdm_dev,
			    struct device *request_dev, void *voltdm_data)
{
	struct omap_voltdm_data *data = (struct omap_voltdm_data *)voltdm_data;

	/* We dont expect voltdm layer to make mistakes.. but still */
	BUG_ON(!data || !voltdm_dev || !request_dev);

	regulator_put(data->vbb_reg);
	regulator_put(data->vdd_reg);
	regulator_unregister_supply_alias(request_dev, "vbb");
	regulator_unregister_supply_alias(request_dev, "vdd");
	kfree(data);

	return;
}

static const struct pm_voltdm_ops omap_voltdm_ops = {
	.voltdm_get = omap_voltdm_get,
	.voltdm_put = omap_voltdm_put,
	.voltdm_latency = omap_voltdm_latency,
	.voltdm_do_transition = omap_voltdm_do_transition,
};

static const struct pm_voltdm_desc omap_voltdm_desc = {
	.ops = &omap_voltdm_ops,
};

static const struct omap_voltdm_of_data omap_generic_of_data = {
	.desc = &omap_voltdm_desc,
};

static const struct of_device_id omap_voltdm_of_match[] = {
	{.compatible = "ti,omap-voltdm", .data = &omap_generic_of_data},
	{},
};

static int omap_voltdm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_device_id *match;
	struct pm_voltdm_dev *vdev;
	int ret = 0;
	const struct omap_voltdm_of_data *of_data;

	match = of_match_device(omap_voltdm_of_match, dev);
	if (!match) {
		/* We do not expect this to happen */
		dev_err(dev, "%s: Unable to match device\n", __func__);
		return -ENODEV;
	}
	if (!match->data) {
		/* Again, unlikely.. but mistakes do happen */
		dev_err(dev, "%s: Bad data in match\n", __func__);
		return -EINVAL;
	}
	of_data = match->data;

	dev_set_drvdata(dev, (void *)of_data);
	vdev = devm_voltdm_register(dev, of_data->desc);
	if (IS_ERR(vdev)) {
		ret = PTR_ERR(vdev);
		dev_err(dev, "Failed to register voltage domain %d\n", ret);
	}

	return ret;
}

MODULE_ALIAS("platform:omap_voltdm");

static struct platform_driver omap_voltdm_driver = {
	.probe = omap_voltdm_probe,
	.driver = {
		   .name = "omap_voltdm",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(omap_voltdm_of_match),
		   },
};
module_platform_driver(omap_voltdm_driver);

MODULE_DESCRIPTION("Texas Instruments OMAP Voltage Domain driver");
MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_LICENSE("GPL v2");
