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
#include <linux/io.h>
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
 * struct omap_voltdm_optimium_voltage_table - optimized voltage table
 * @reference_uv:	reference voltage (usually Nominal voltage)
 * @optimized_uv:	Optimized voltage from efuse
 */
struct omap_voltdm_optimium_voltage_table {
	unsigned int reference_uv;
	unsigned int optimized_uv;
};

/**
 * struct omap_voltdm_data - OMAP specific voltage domain data
 * @vdd_reg:	VDD regulator
 * @vbb_reg:	Body Bias regulator
 * @vdd_table:	Optimized voltage mapping table
 * @num_vdd_table: number of entries in vdd_table
 * @vdd_absolute_max_voltage_uv: absolute maximum voltage in UV for the domain
 */
struct omap_voltdm_data {
	struct regulator *vdd_reg;
	struct regulator *vbb_reg;
	struct omap_voltdm_optimium_voltage_table *vdd_table;
	u32 num_vdd_table;
	u32 vdd_absolute_max_voltage_uv;
};

/**
 * struct omap_voltdm_of_data - device tree match data
 * @desc:	voltage domain descriptor for voltage domain core
 * @flags:	specific type of voltage domain
 * @efuse_voltage_mask: mask required for efuse register representing voltage
 * @efuse_voltage_uv: Are the efuse entries in micro-volts? if not, assume
 *		milli-volts.
 */
struct omap_voltdm_of_data {
	const struct pm_voltdm_desc *desc;
#define VOLTDM_EFUSE_CLASS0_OPTIMIZED_VOLTAGE	BIT(1)
#define VOLTDM_HAS_NO_ABB			BIT(2)
	const u8 flags;
	const u32 efuse_voltage_mask;
	const bool efuse_voltage_uv;
};

/**
 * voltdm_store_optimized_voltages() - store optimized voltages
 * @dev:	voltage domain device for which we need to store info
 * @data:	data specific to the device
 *
 * Picks up efuse based optimized voltages for VDD unique per device and
 * stores it in internal data structure for use during transition requests.
 *
 * Return: If successful, 0, else appropriate error value.
 */
static int voltdm_store_optimized_voltages(struct device *dev,
					   struct omap_voltdm_data *data)
{
	void __iomem *base;
	struct property *prop;
	struct resource *res;
	const __be32 *val;
	int proplen, i;
	int ret = 0;
	struct omap_voltdm_optimium_voltage_table *table;
	const struct omap_voltdm_of_data *of_data = dev_get_drvdata(dev);

	/* pick up Efuse based voltages */
	res = platform_get_resource(to_platform_device(dev), IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "Unable to get IO resource\n");
		ret = -ENODEV;
		goto out_map;
	}

	base = ioremap_nocache(res->start, resource_size(res));
	if (!base) {
		dev_err(dev, "Unable to map Efuse registers\n");
		ret = -ENOMEM;
		goto out_map;
	}

	/* Fetch efuse-settings. */
	prop = of_find_property(dev->of_node, "ti,efuse-settings", NULL);
	if (!prop) {
		dev_err(dev, "No 'ti,efuse-settings' property found\n");
		ret = -EINVAL;
		goto out;
	}

	proplen = prop->length / sizeof(int);
	data->num_vdd_table = proplen / 2;
	/* Verify for corrupted OPP entries in dt */
	if (data->num_vdd_table * 2 * sizeof(int) != prop->length) {
		dev_err(dev, "Invalid 'ti,efuse-settings'\n");
		ret = -EINVAL;
		goto out;
	}

	table = kzalloc(sizeof(*data->vdd_table) *
				  data->num_vdd_table, GFP_KERNEL);
	if (!table) {
		dev_err(dev, "Unable to Allocate voltage set table\n");
		ret = -ENOMEM;
		goto out;
	}
	data->vdd_table = table;

	val = prop->value;
	for (i = 0; i < data->num_vdd_table; i++, table++) {
		u32 efuse_offset;
		u32 tmp;

		table->reference_uv = be32_to_cpup(val++);
		efuse_offset = be32_to_cpup(val++);

		tmp = readl(base + efuse_offset);
		tmp &= of_data->efuse_voltage_mask;
		tmp >>= __ffs(of_data->efuse_voltage_mask);

		table->optimized_uv = of_data->efuse_voltage_uv ? tmp :
					tmp * 1000;

		dev_dbg(dev, "[%d] efuse=0x%08x volt_table=%d vset=%d\n",
			i, efuse_offset, table->reference_uv,
			table->optimized_uv);

		/*
		 * Some older samples might not have optimized efuse
		 * Use reference voltage for those - just add debug message
		 * for them.
		 */
		if (!table->optimized_uv) {
			dev_dbg(dev, "[%d] efuse=0x%08x volt_table=%d:vset0\n",
				i, efuse_offset, table->reference_uv);
			table->optimized_uv = table->reference_uv;
		}
	}

	of_property_read_u32(dev->of_node, "ti,absolute-max-voltage-uv",
			     &data->vdd_absolute_max_voltage_uv);

out:
	iounmap(base);
out_map:
	return ret;
}

/**
 * voltdm_free_optimized_voltages() - free resources for optimized voltages
 * @dev:	voltage domain device for which we need to free info
 * @data:	data specific to the device
 */
static void voltdm_free_optimized_voltages(struct device *dev,
					   struct omap_voltdm_data *data)
{
	kfree(data->vdd_table);
	data->vdd_table = NULL;
	data->num_vdd_table = 0;
}

/**
 * voltdm_get_optimal_vdd_voltage() - Finds optimal voltage for the domain
 * @dev:	voltage domain device for which we need to find info
 * @data:	data specific to the device
 * @reference_uv:	reference voltage (OPP voltage) for which we need value
 *
 * Return: if a match is found, return optimized voltage, else return
 * reference_uv, also return reference_uv if no optimization is needed.
 */
static int voltdm_get_optimal_vdd_voltage(struct device *dev,
					  struct omap_voltdm_data *data,
					  int reference_uv)
{
	int i;
	struct omap_voltdm_optimium_voltage_table *table;

	if (!data->num_vdd_table)
		return reference_uv;

	table = data->vdd_table;
	BUG_ON(!table);

	/* Find a exact match - this list is usually very small */
	for (i = 0; i < data->num_vdd_table; i++, table++)
		if (table->reference_uv == reference_uv)
			return table->optimized_uv;

	/* IF things are screwed up, we'd make a mess on console.. ratelimit */
	dev_err_ratelimited(dev, "%s: Failed optimized voltage match for %d\n",
			    __func__, reference_uv);
	return reference_uv;
}

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
	int vdd_uv;

	/* We dont expect voltdm layer to make mistakes.. but still */
	BUG_ON(!data);

	do_abb_first = clk_notifier_flags == ABORT_RATE_CHANGE ||
	    clk_notifier_flags == POST_RATE_CHANGE;

	if (do_abb_first && !IS_ERR(data->vbb_reg)) {
		dev_dbg(dev, "vbb pre %duV[tol %duV]\n", uv, tol_uv);
		ret = regulator_set_voltage_tol(data->vbb_reg, uv, tol_uv);
		if (ret) {
			dev_err(dev,
				"vbb failed for voltage %duV[tol %duV]:%d\n",
				uv, tol_uv, ret);
			return ret;
		}
	}

	vdd_uv = voltdm_get_optimal_vdd_voltage(dev, data, uv);
	dev_dbg(dev, "vdd for voltage %duV(ref=%duV)[tol %duV] MAX=%duV\n",
		vdd_uv, uv, tol_uv, data->vdd_absolute_max_voltage_uv);
	if (data->vdd_absolute_max_voltage_uv) {
		ret = regulator_set_voltage(data->vdd_reg,
					    vdd_uv,
					    data->vdd_absolute_max_voltage_uv);
		/* Try a lower range */
		if (ret)
			ret = regulator_set_voltage(data->vdd_reg,
					    vdd_uv - tol_uv,
					    data->vdd_absolute_max_voltage_uv);
	} else {
		ret = regulator_set_voltage_tol(data->vdd_reg, vdd_uv, tol_uv);
	}
	if (ret) {
		dev_err(dev,
			"vdd failed for voltage %duV(ref=%duV)[tol %duV]:%d\n",
			vdd_uv, uv, tol_uv, ret);
		return ret;
	}

	if (!do_abb_first && !IS_ERR(data->vbb_reg)) {
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
	if (IS_ERR(data->vbb_reg))
		goto skip_vbb;

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
	const struct omap_voltdm_of_data *of_data = dev_get_drvdata(voltdm_dev);

	BUG_ON(!voltdm_dev || !request_dev || !voltdm_data || !of_data);

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	/* If we need optimized voltage */
	if (of_data->flags & VOLTDM_EFUSE_CLASS0_OPTIMIZED_VOLTAGE) {
		ret = voltdm_store_optimized_voltages(voltdm_dev, data);
		if (ret)
			goto out_free;
	}

	/*
	 * Setup aliases for request device supply to let regulator framework
	 * do multi-consumer scenario
	 */
	ret = regulator_register_supply_alias(request_dev, "vdd",
					      voltdm_dev, "vdd");
	if (ret)
		goto out_release_optimized_voltages;
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

	if (of_data->flags & VOLTDM_HAS_NO_ABB) {
		data->vbb_reg = ERR_PTR(-ENODEV);
	} else {
		data->vbb_reg = regulator_get(request_dev, "vbb");
		if (IS_ERR(data->vbb_reg)) {
			ret = PTR_ERR(data->vbb_reg);
			dev_err(voltdm_dev, "Unable to get vbb regulator:%d\n",
				ret);
			goto out_vdd_reg;
		}
	}

	*voltdm_data = data;
	return 0;

out_vdd_reg:
	regulator_put(data->vdd_reg);
out_unreg:
	regulator_unregister_supply_alias(request_dev, "vbb");
out_unreg_vdd:
	regulator_unregister_supply_alias(request_dev, "vdd");
out_release_optimized_voltages:
	if (of_data->flags & VOLTDM_EFUSE_CLASS0_OPTIMIZED_VOLTAGE)
		voltdm_free_optimized_voltages(voltdm_dev, data);
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

	if (!IS_ERR(data->vbb_reg))
		regulator_put(data->vbb_reg);
	regulator_put(data->vdd_reg);
	regulator_unregister_supply_alias(request_dev, "vbb");
	regulator_unregister_supply_alias(request_dev, "vdd");
	kfree(data);
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

static const struct omap_voltdm_of_data omap_omap5_of_data = {
	.desc = &omap_voltdm_desc,
	.flags = VOLTDM_EFUSE_CLASS0_OPTIMIZED_VOLTAGE,
	.efuse_voltage_mask = 0xFFF,
	.efuse_voltage_uv = false,
};

static const struct omap_voltdm_of_data omap_omap5core_of_data = {
	.desc = &omap_voltdm_desc,
	.flags = VOLTDM_EFUSE_CLASS0_OPTIMIZED_VOLTAGE | VOLTDM_HAS_NO_ABB,
	.efuse_voltage_mask = 0xFFF,
	.efuse_voltage_uv = false,
};

static const struct of_device_id omap_voltdm_of_match[] = {
	{.compatible = "ti,omap-voltdm", .data = &omap_generic_of_data},
	{.compatible = "ti,omap5-voltdm", .data = &omap_omap5_of_data},
	{.compatible = "ti,omap5-core-voltdm", .data = &omap_omap5core_of_data},
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
