/*
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 *	Nishanth Menon <nm@ti.com>
 *	Dave Gerlach <d-gerlach@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * TI OPP Domain driver that provides overrides into the regulator control
 * for generic opp domains to handle devices with ABB regulator and/or
 * SmartReflex Class0.
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
#include <linux/pm_opp_domain.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

/**
 * struct ti_oppdm_optimum_voltage_table - optimized voltage table
 * @reference_uv:	reference voltage (usually Nominal voltage)
 * @optimized_uv:	Optimized voltage from efuse
 */
struct ti_oppdm_optimum_voltage_table {
	unsigned int reference_uv;
	unsigned int optimized_uv;
};

/**
 * struct ti_oppdm_data - OMAP specific opp domain data
 * @vdd_reg:	VDD regulator
 * @vbb_reg:	Body Bias regulator
 * @vdd_table:	Optimized voltage mapping table
 * @num_vdd_table: number of entries in vdd_table
 * @vdd_absolute_max_voltage_uv: absolute maximum voltage in UV for the domain
 */
struct ti_oppdm_data {
	struct regulator *vdd_reg;
	struct regulator *vbb_reg;
	struct ti_oppdm_optimum_voltage_table *vdd_table;
	u32 num_vdd_table;
	u32 vdd_absolute_max_voltage_uv;
};

/**
 * struct ti_oppdm_of_data - device tree match data
 * @desc:	opp domain descriptor for opp domain core
 * @flags:	specific type of opp domain
 * @efuse_voltage_mask: mask required for efuse register representing voltage
 * @efuse_voltage_uv: Are the efuse entries in micro-volts? if not, assume
 *		milli-volts.
 */
struct ti_oppdm_of_data {
	const struct pm_opp_domain_desc *desc;
#define OPPDM_EFUSE_CLASS0_OPTIMIZED_VOLTAGE	BIT(1)
#define OPPDM_HAS_NO_ABB			BIT(2)
	const u8 flags;
	const u32 efuse_voltage_mask;
	const bool efuse_voltage_uv;
};

/**
 * oppdm_store_optimized_voltages() - store optimized voltages
 * @dev:	opp domain device for which we need to store info
 * @data:	data specific to the device
 *
 * Picks up efuse based optimized voltages for VDD unique per device and
 * stores it in internal data structure for use during transition requests.
 *
 * Return: If successful, 0, else appropriate error value.
 */
static int oppdm_store_optimized_voltages(struct device *dev,
					  struct ti_oppdm_data *data)
{
	void __iomem *base;
	struct property *prop;
	struct resource *res;
	const __be32 *val;
	int proplen, i;
	int ret = 0;
	struct ti_oppdm_optimum_voltage_table *table;
	const struct ti_oppdm_of_data *of_data = dev_get_drvdata(dev);

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

	ret = of_property_read_u32(dev->of_node, "ti,absolute-max-voltage-uv",
				   &data->vdd_absolute_max_voltage_uv);
	if (ret) {
		dev_err(dev, "ti,absolute-max-voltage-uv is missing\n");
		ret = -EINVAL;
		goto out;
	}

	table = kzalloc(sizeof(*data->vdd_table) *
				  data->num_vdd_table, GFP_KERNEL);
	if (!table) {
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
out:
	iounmap(base);
out_map:
	return ret;
}

/**
 * oppdm_free_optimized_voltages() - free resources for optimized voltages
 * @dev:	opp domain device for which we need to free info
 * @data:	data specific to the device
 */
static void oppdm_free_optimized_voltages(struct device *dev,
					  struct ti_oppdm_data *data)
{
	kfree(data->vdd_table);
	data->vdd_table = NULL;
	data->num_vdd_table = 0;
}

/**
 * oppdm_get_optimal_vdd_voltage() - Finds optimal voltage for the domain
 * @dev:	opp domain device for which we need to find info
 * @data:	data specific to the device
 * @reference_uv:	reference voltage (OPP voltage) for which we need value
 *
 * Return: if a match is found, return optimized voltage, else return
 * reference_uv, also return reference_uv if no optimization is needed.
 */
static int oppdm_get_optimal_vdd_voltage(struct device *dev,
					 struct ti_oppdm_data *data,
					 int reference_uv)
{
	int i;
	struct ti_oppdm_optimum_voltage_table *table;

	if (!data->num_vdd_table)
		return reference_uv;

	table = data->vdd_table;
	if (!table)
		return -EINVAL;

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
 * ti_oppdm_do_transition() - do the opp domain transition
 * @dev:	opp domain device for which we are doing the transition
 * @oppdm_data:	data specific to the device
 * @clk_notifier_flags:	clk notifier flags for direction of transition
 * @uv:		what voltage to transition to
 * @uv_min:	minimum acceptable voltage for transition
 * @uv_max:	maximum	acceptable voltage for transition
 *
 * Return: If successful, 0, else appropriate error value.
 */
static int ti_oppdm_do_transition(struct device *dev,
				  void *oppdm_data,
				  unsigned long clk_notifier_flags, int uv,
				  int uv_min, int uv_max)
{
	struct ti_oppdm_data *data = (struct ti_oppdm_data *)oppdm_data;
	int ret;
	bool do_abb_first;
	int vdd_uv;

	if (!data)
		return -EINVAL;

	if (uv_max > data->vdd_absolute_max_voltage_uv) {
		dev_warn(dev,
			 "exceeding vdd_absolute_max_voltage_uv, using %d for uv_max\n",
			 data->vdd_absolute_max_voltage_uv);
		uv_max = data->vdd_absolute_max_voltage_uv;
	}
	/*
	 * If we do have an absolute max voltage specified, then we should
	 * use that voltage instead to allow for cases where the voltage rails
	 * are ganged (example if we set the max for an opp as 1.12v, and
	 * the absolute max is 1.5v, for another rail to get 1.25v, it cannot
	 * be achieved if the regulator is constrainted to max of 1.12v, even
	 * if it can function at 1.25v
	 */
	if (data->vdd_absolute_max_voltage_uv)
		uv_max = data->vdd_absolute_max_voltage_uv;

	do_abb_first = clk_notifier_flags == PM_OPPDM_VOLT_ABORTRATE ||
	    clk_notifier_flags == PM_OPPDM_VOLT_POSTRATE;

	vdd_uv = oppdm_get_optimal_vdd_voltage(dev, data, uv);

	if (vdd_uv > uv_max || vdd_uv < uv_min || uv_min > uv_max) {
		dev_warn(dev,
			 "Invalid range voltages [Min:%d target:%d Max:%d]\n",
			 uv_min, vdd_uv, uv_max);
		return -EINVAL;
	}

	if (do_abb_first && !IS_ERR(data->vbb_reg)) {
		dev_dbg(dev, "vbb pre %duV[min %duV max %duV]\n", uv, uv_min,
			uv_max);
		ret = regulator_set_voltage_triplet(data->vbb_reg, uv_min,
						    uv, uv_max);
		if (ret) {
			dev_err(dev,
				"vbb failed for voltage %duV[min %duV max %duV]:%d\n",
				uv, uv_min, uv_max, ret);
			return ret;
		}
	}

	dev_dbg(dev, "vdd for voltage %duV(ref=%duV)[min %duV max %duV] MAX=%duV\n",
		vdd_uv, uv, uv_min, uv_max, data->vdd_absolute_max_voltage_uv);

	ret = regulator_set_voltage_triplet(data->vdd_reg, uv_min,
					    vdd_uv, uv_max);
	if (ret) {
		dev_err(dev,
			"vdd failed for voltage %duV(ref=%duV)[min %duV max %duV]:%d\n",
			vdd_uv, uv, uv_min, uv_max, ret);
		return ret;
	}

	if (!do_abb_first && !IS_ERR(data->vbb_reg)) {
		dev_dbg(dev, "vbb post %duV[min %duV max %duV]\n", uv, uv_min,
			uv_max);
		ret = regulator_set_voltage_triplet(data->vbb_reg, uv_min,
						    uv, uv_max);
		if (ret) {
			dev_err(dev,
				"vbb failed for voltage %duV[min %duV max %duV]:%d\n",
				uv, uv_min, uv_max, ret);
			return ret;
		}
	}

	return 0;
}

/**
 * ti_oppdm_latency() - provide the transition latency for the opp domain
 * @dev:	opp domain device for which we are doing the transition
 * @oppdm_data: data specific to the device
 * @old_uv: starting voltage in microvolts
 * @old_uv_min: minimum acceptable starting voltage in microvolts
 * @old_uv_max: maximum acceptable starting voltage in microvolts
 * @new_uv: target voltage in microvolts
 * @new_uv_min: minimum acceptable target voltage in microvolts
 * @new_uv_max: maximum acceptable target voltage in microvolts
 *
 * Return: If successful, the combined transition latency from min to max, else
 * returns error value
 */
static int ti_oppdm_latency(struct device *dev, void *oppdm_data,
			    unsigned long old_uv,
			    unsigned long old_uv_min,
			    unsigned long old_uv_max,
			    unsigned long new_uv,
			    unsigned long new_uv_min,
			    unsigned long new_uv_max)
{
	struct ti_oppdm_data *data = (struct ti_oppdm_data *)oppdm_data;
	int ret, tot_latency = 0;

	if (!data)
		return -EINVAL;

	/*
	 * NOTE: latency computations can depend on regulator involved
	 * We just provide the best of the information we can find.
	 */
	ret = regulator_set_voltage_time_triplet(data->vdd_reg, old_uv,
						 old_uv_min, old_uv_max,
						 new_uv, new_uv_min,
						 new_uv_max);
	if (ret < 0) {
		dev_dbg(dev, "vdd failed voltage latency: %d\n", ret);
		goto skip_vdd;
	}
	tot_latency += ret;

skip_vdd:
	if (IS_ERR(data->vbb_reg))
		goto skip_vbb;

	ret = regulator_set_voltage_time_triplet(data->vbb_reg, old_uv,
						 old_uv_min, old_uv_max,
						 new_uv, new_uv_min,
						 new_uv_max);
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

static inline void ti_oppdm_cleanup(struct ti_oppdm_data *data)
{
	if (!IS_ERR(data->vbb_reg))
		regulator_put(data->vbb_reg);
	if (!IS_ERR(data->vdd_reg))
		regulator_put(data->vdd_reg);
	kfree(data);
}

/**
 * ti_oppdm_get() - get the opp domain resources specific to request
 * @oppdm_dev:		opp domain device
 * @request_dev:	device for which we have been requested to get
 * @np:			unused
 * @supply:		unused
 * @oppdm_data:		returns data for the current request (freed in put)
 *
 * Return: 0 if everything went OK, else return appropriate error value.
 */
static int ti_oppdm_get(struct device *oppdm_dev,
			struct device *request_dev,
			struct device_node *np,
			const char *supply,
			void **oppdm_data)
{
	struct ti_oppdm_data *data;
	int ret = 0;
	const struct ti_oppdm_of_data *of_data = dev_get_drvdata(oppdm_dev);

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	/* If we need optimized voltage */
	if (of_data->flags & OPPDM_EFUSE_CLASS0_OPTIMIZED_VOLTAGE) {
		ret = oppdm_store_optimized_voltages(oppdm_dev, data);
		if (ret)
			goto out_free;
	}

	/*
	 * Setup aliases for request device supply to let regulator framework
	 * do multi-consumer scenario
	 */
	ret = regulator_register_supply_alias(request_dev, "vdd",
					      oppdm_dev, "vdd");
	if (ret)
		goto out_release_optimized_voltages;
	ret = regulator_register_supply_alias(request_dev, "vbb",
					      oppdm_dev, "vbb");
	if (ret)
		goto out_unreg_vdd;

	data->vdd_reg = regulator_get(request_dev, "vdd");
	if (IS_ERR(data->vdd_reg)) {
		ret = PTR_ERR(data->vdd_reg);
		dev_err(oppdm_dev, "Unable to get vdd regulator:%d\n", ret);
		goto out_unreg;
	}

	if (of_data->flags & OPPDM_HAS_NO_ABB) {
		data->vbb_reg = ERR_PTR(-ENODEV);
	} else {
		data->vbb_reg = regulator_get(request_dev, "vbb");
		if (IS_ERR(data->vbb_reg)) {
			ret = PTR_ERR(data->vbb_reg);
			dev_err(oppdm_dev, "Unable to get vbb regulator:%d\n",
				ret);
			goto out_vdd_reg;
		}
	}

	*oppdm_data = data;
	return 0;

out_vdd_reg:
	regulator_put(data->vdd_reg);
out_unreg:
	regulator_unregister_supply_alias(request_dev, "vbb");
out_unreg_vdd:
	regulator_unregister_supply_alias(request_dev, "vdd");
out_release_optimized_voltages:
	if (of_data->flags & OPPDM_EFUSE_CLASS0_OPTIMIZED_VOLTAGE)
		oppdm_free_optimized_voltages(oppdm_dev, data);
out_free:
	kfree(data);

	return ret;
}

/**
 * ti_oppdm_put() - release the resources reserved by ti_oppdm_get()
 * @oppdm_dev:		opp domain device for which we reserved resources
 * @request_dev:	device for which we have been requested to put
 * @oppdm_data:		data for the request provided by ti_oppdm_get()
 */
static void ti_oppdm_put(struct device *oppdm_dev,
			 struct device *request_dev, void *oppdm_data)
{
	struct ti_oppdm_data *data = (struct ti_oppdm_data *)oppdm_data;

	if (!IS_ERR(data->vbb_reg))
		regulator_put(data->vbb_reg);
	regulator_put(data->vdd_reg);
	regulator_unregister_supply_alias(request_dev, "vbb");
	regulator_unregister_supply_alias(request_dev, "vdd");
	kfree(data);
}

/**
 * ti_oppdm_is_supported_voltage() - return if provided voltage is supported
 * @oppdm_dev:		opp domain device for which we reserved resources
 * @oppdm_data:		data for the request provided by ti_oppdm_get()
 * @uv_min:		minimum voltage range to check if supported
 * @uv_max:		maximum voltage range to check if supported
 *
 * Checks that voltage is supported by both vdd and vbb regulators if present
 */
static bool ti_oppdm_is_supported_voltage(struct device *oppdm_dev,
					  void *oppdm_data,
					  unsigned long uv_min,
					  unsigned long uv_max)
{
	struct ti_oppdm_data *data = (struct ti_oppdm_data *)oppdm_data;

	if (!IS_ERR(data->vdd_reg) &&
	    !regulator_is_supported_voltage(data->vdd_reg, uv_min,
					    uv_max))
		return false;

	if (!IS_ERR(data->vbb_reg) &&
	    !regulator_is_supported_voltage(data->vbb_reg, uv_min,
					    uv_max))
		return false;

	return true;
}

static const struct pm_opp_domain_ops ti_oppdm_ops = {
	.oppdm_get = ti_oppdm_get,
	.oppdm_put = ti_oppdm_put,
	.oppdm_get_latency = ti_oppdm_latency,
	.oppdm_do_transition = ti_oppdm_do_transition,
	.oppdm_is_supported_voltage = ti_oppdm_is_supported_voltage,
};

static const struct pm_opp_domain_desc ti_oppdm_desc = {
	.ops = &ti_oppdm_ops,
};

static const struct ti_oppdm_of_data omap_generic_of_data = {
	.desc = &ti_oppdm_desc,
};

static const struct ti_oppdm_of_data omap_omap5_of_data = {
	.desc = &ti_oppdm_desc,
	.flags = OPPDM_EFUSE_CLASS0_OPTIMIZED_VOLTAGE,
	.efuse_voltage_mask = 0xFFF,
	.efuse_voltage_uv = false,
};

static const struct ti_oppdm_of_data omap_omap5core_of_data = {
	.desc = &ti_oppdm_desc,
	.flags = OPPDM_EFUSE_CLASS0_OPTIMIZED_VOLTAGE | OPPDM_HAS_NO_ABB,
	.efuse_voltage_mask = 0xFFF,
	.efuse_voltage_uv = false,
};

static const struct of_device_id ti_oppdm_of_match[] = {
	{.compatible = "ti,omap-oppdm", .data = &omap_generic_of_data},
	{.compatible = "ti,omap5-oppdm", .data = &omap_omap5_of_data},
	{.compatible = "ti,omap5-core-oppdm", .data = &omap_omap5core_of_data},
	{},
};

static int ti_oppdm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_device_id *match;
	struct pm_opp_domain_dev *oppdm_dev;
	int ret = 0;
	const struct ti_oppdm_of_data *of_data;

	match = of_match_device(ti_oppdm_of_match, dev);
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
	oppdm_dev = devm_opp_domain_register(dev, of_data->desc);
	if (IS_ERR(oppdm_dev)) {
		ret = PTR_ERR(oppdm_dev);
		dev_err(dev, "Failed to register opp domain %d\n", ret);
	}

	return ret;
}

MODULE_DEVICE_TABLE(of, ti_oppdm_of_match);

static struct platform_driver ti_oppdm_driver = {
	.probe = ti_oppdm_probe,
	.driver = {
		   .name = "ti_oppdm",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(ti_oppdm_of_match),
		   },
};
module_platform_driver(ti_oppdm_driver);

MODULE_DESCRIPTION("Texas Instruments OMAP OPP Domain driver");
MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_LICENSE("GPL v2");
