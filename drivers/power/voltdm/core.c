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
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_opp.h>
#include <linux/pm_voltage_domain.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

#include "voltage_domain_private.h"

/**
 * struct pm_voltdm_dev - internal representation of voltage domain devices
 * @desc:	voltage domain description
 * @dev:	voltage domain device
 * @list:	list to remaining voltage domain devices
 * @lock:	mutex to control data structure modifications and serialize ops
 * @notifier_list:	list of notifiers registered for this device
 */
struct pm_voltdm_dev {
	const struct pm_voltdm_desc *desc;
	struct device *dev;
	struct list_head list;
	/* list lock */
	struct mutex lock;
	struct list_head notifier_list;
};

/**
 * struct voltdm_scale_data - Internal structure to maintain notifier information
 * @dev:	device on behalf of which we register the notifier
 * @clk:	clk on which we registered the notifier
 * @reg:	regulator if any which is used for scaling voltage
 * @tol:	voltage tolerance in %
 * @nb:		notifier block pointer
 * @list:	list head for the notifier
 * @vdev:	pointer to voltage domain device for this notifier
 * @voltdm_data: voltdm driver specific data
 */
struct voltdm_scale_data {
	struct device *dev;
	struct clk *clk;
	struct regulator *reg;
	int tol;
	struct notifier_block nb;
	struct list_head list;

	struct pm_voltdm_dev *vdev;
	void *voltdm_data;
};

#define to_voltdm_scale_data(_nb) container_of(_nb, \
		struct voltdm_scale_data, nb)

static DEFINE_MUTEX(pm_voltdm_list_lock);
static LIST_HEAD(pm_voltdm_list);

static inline bool voltdm_skip_check(struct pm_voltdm_dev *vdev)
{
	bool ret = false;

	if (vdev) {
		const struct pm_voltdm_desc *desc;

		if (IS_ERR(vdev))
			return false;

		mutex_lock(&vdev->lock);
		desc = vdev->desc;

		if (desc->flags & VOLTAGE_DOMAIN_FLAG_NOTIFY_ALL)
			ret = true;
		mutex_unlock(&vdev->lock);
	}

	return ret;
}

static inline int voltdm_scale_voltage(struct voltdm_scale_data *vsd,
				       unsigned long flags, int volt, int tol)
{
	int ret;
	struct pm_voltdm_dev *vdev = vsd->vdev;

	if (vdev) {
		const struct pm_voltdm_ops *ops;

		if (IS_ERR(vdev))
			return PTR_ERR(vdev);

		mutex_lock(&vdev->lock);
		ops = vdev->desc->ops;

		ret = ops->voltdm_do_transition(vdev->dev,
						vsd->voltdm_data,
						flags, volt, tol);
		mutex_unlock(&vdev->lock);
	} else {
		ret = regulator_set_voltage_tol(vsd->reg, volt, tol);
	}

	return ret;
}

static struct pm_voltdm_dev *voltdm_parse_of(struct device_node *np,
					     const char *supply,
					     struct of_phandle_args *args)
{
	char prop_name[32];	/* 32 is max size of property name */
	bool found = false;
	struct device_node *voltdm_np;
	struct pm_voltdm_dev *vdev = NULL;
	int ret;

	snprintf(prop_name, sizeof(prop_name), "%s-voltdm", supply);
	voltdm_np = of_parse_phandle(np, prop_name, 0);
	if (voltdm_np) {
		ret = of_parse_phandle_with_args(np, prop_name, "#voltdm-cells",
						 0, args);
		if (ret)
			return ERR_PTR(ret);

		mutex_lock(&pm_voltdm_list_lock);
		list_for_each_entry(vdev, &pm_voltdm_list, list)
			if (vdev->dev->parent && voltdm_np ==
			    vdev->dev->of_node) {
				found = true;
				break;
			}
		mutex_unlock(&pm_voltdm_list_lock);

		/* if node is present and not ready, then defer */
		if (!found)
			return ERR_PTR(-EPROBE_DEFER);
	} else {
		return NULL;
	}

	return vdev;
}

static int voltdm_get(struct voltdm_scale_data *vsd, struct device_node *np,
		      const char *supply, struct of_phandle_args *args,
		      bool *skip_reg)
{
	struct pm_voltdm_dev *vdev = vsd->vdev;
	struct device *dev = vsd->dev;
	int ret = 0;

	if (vdev) {
		const struct pm_voltdm_ops *ops;

		if (IS_ERR(vdev))
			return PTR_ERR(vdev);

		mutex_lock(&vdev->lock);
		if (!try_module_get(vdev->dev->driver->owner)) {
			ret = -ENODEV;
		} else {
			ops = vdev->desc->ops;
			if (ops->voltdm_get)
				ret = ops->voltdm_get(vdev->dev, dev, np,
						      args, supply,
						      &vsd->voltdm_data);
			if (ret)
				module_put(vdev->dev->driver->owner);
		}
		if (!ret)
			list_add(&vsd->list, &vdev->notifier_list);

		mutex_unlock(&vdev->lock);
	} else {
		vsd->reg = regulator_get_optional(dev, supply);
		if (IS_ERR(vsd->reg))
			ret = PTR_ERR(vsd->reg);
		/* Regulator is not mandatory */
		if (ret && ret != -EPROBE_DEFER) {
			ret = 0;
			*skip_reg = true;
			dev_dbg(dev, "%s: Failed to get %s regulator:%d\n",
				__func__, supply, ret);
		}
	}

	return ret;
}

static void voltdm_put(struct voltdm_scale_data *vsd)
{
	struct pm_voltdm_dev *vdev = vsd->vdev;

	if (vdev) {
		const struct pm_voltdm_ops *ops;

		if (IS_ERR(vdev))
			return;

		mutex_lock(&vdev->lock);
		ops = vdev->desc->ops;
		if (ops->voltdm_put)
			ops->voltdm_put(vdev->dev, vsd->dev, vsd->voltdm_data);
		list_del(&vsd->list);
		module_put(vdev->dev->driver->owner);
		mutex_unlock(&vdev->lock);
	} else {
		if (!IS_ERR(vsd->reg))
			regulator_put(vsd->reg);
	}
}

static int voltdm_get_latency(struct voltdm_scale_data *vsd, int min, int max)
{
	struct pm_voltdm_dev *vdev = vsd->vdev;
	const struct pm_voltdm_ops *ops;
	int ret;

	if (!vdev)
		return regulator_set_voltage_time(vsd->reg, min, max);

	if (IS_ERR(vdev))
		return PTR_ERR(vdev);

	mutex_lock(&vdev->lock);
	ops = vdev->desc->ops;

	if (!ops->voltdm_latency)
		ret = -ENXIO;
	else
		ret = ops->voltdm_latency(vdev->dev, vsd->voltdm_data,
					  min, max);
	mutex_unlock(&vdev->lock);

	return ret;
}

static int clk_voltdm_notifier_handler(struct notifier_block *nb,
				       unsigned long flags, void *data)
{
	struct clk_notifier_data *cnd = data;
	struct voltdm_scale_data *vsd = to_voltdm_scale_data(nb);
	struct pm_voltdm_dev *vdev = vsd->vdev;
	int ret, volt, tol;
	struct dev_pm_opp *opp;
	unsigned long old_rate = cnd->old_rate;
	unsigned long new_rate = cnd->new_rate;

	if (!voltdm_skip_check(vdev) &&
	    ((new_rate < old_rate && flags == PRE_RATE_CHANGE) ||
	     (new_rate > old_rate && flags == POST_RATE_CHANGE)))
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

	ret = voltdm_scale_voltage(vsd, flags, volt, tol);
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
	struct of_phandle_args voltdm_args = {NULL};
	struct pm_voltdm_dev *vdev = NULL;
	bool skip_reg = false;

	/* First look for voltdm of node */
	vdev = voltdm_parse_of(np, supply, &voltdm_args);
	if (IS_ERR(vdev))
		return (struct notifier_block *)vdev;

	vsd = kzalloc(sizeof(*vsd), GFP_KERNEL);
	if (!vsd)
		return ERR_PTR(-ENOMEM);

	vsd->dev = dev;
	vsd->clk = clk;
	vsd->nb.notifier_call = clk_voltdm_notifier_handler;
	vsd->vdev = vdev;

	ret = voltdm_get(vsd, np, supply, &voltdm_args, &skip_reg);
	if (ret) {
		dev_warn(dev, "%s: Failed to get %s regulator/voltdm: %d\n",
			 __func__, supply, ret);
		goto err_free_vsd;
	}
	/* if not mandatory... */
	if (skip_reg)
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

	*voltage_latency = voltdm_get_latency(vsd, min, max);
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
	voltdm_put(vsd);

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
	voltdm_put(vsd);

	kfree(vsd);
}
EXPORT_SYMBOL_GPL(of_pm_voltdm_notifier_unregister);

static void devm_voltdm_release(struct device *dev, void *res)
{
	struct pm_voltdm_dev *vdev = *(struct pm_voltdm_dev **)res;
	struct voltdm_scale_data *vsd;

	mutex_lock(&pm_voltdm_list_lock);
	mutex_lock(&vdev->lock);
	list_for_each_entry(vsd, &vdev->notifier_list, list) {
		dev_warn(dev, "%s: pending notifier from device %s!\n",
			 __func__, dev_name(vsd->dev));
		vsd->vdev = ERR_PTR(-EINVAL);
	}
	mutex_unlock(&vdev->lock);

	list_del(&vdev->list);
	mutex_unlock(&pm_voltdm_list_lock);

	kfree(vdev);
}

/**
 * devm_voltdm_register - Resource managed voltage domain registration
 * @dev: pointer to the device representing the voltage domain
 * @desc: voltage domain descriptor
 *
 * Called by voltage domain drivers to register a voltagedomain.  Returns a
 * valid pointer to struct pm_voltdm_dev on success or an ERR_PTR() on
 * error.  The voltagedomain will automatically be released when the device
 * is unbound.
 */
struct pm_voltdm_dev *devm_voltdm_register(struct device *dev,
					   const struct pm_voltdm_desc *desc)
{
	struct pm_voltdm_dev **ptr, *vdev;

	if (!dev || !desc)
		return ERR_PTR(-EINVAL);

	if (!desc->ops)
		return ERR_PTR(-EINVAL);

	/* Mandatory to have notify transition */
	if (!desc->ops->voltdm_do_transition) {
		dev_err(dev, "%s: Bad desc -do_transition missing\n", __func__);
		return ERR_PTR(-EINVAL);
	}

	vdev = kzalloc(sizeof(*vdev), GFP_KERNEL);
	if (!vdev)
		return ERR_PTR(-ENOMEM);

	ptr = devres_alloc(devm_voltdm_release, sizeof(*ptr), GFP_KERNEL);
	if (!ptr) {
		kfree(vdev);
		return ERR_PTR(-ENOMEM);
	}

	vdev->desc = desc;
	vdev->dev = dev;
	mutex_init(&vdev->lock);
	INIT_LIST_HEAD(&vdev->notifier_list);
	mutex_lock(&pm_voltdm_list_lock);
	list_add(&vdev->list, &pm_voltdm_list);
	mutex_unlock(&pm_voltdm_list_lock);

	*ptr = vdev;
	devres_add(dev, ptr);

	return vdev;
}
EXPORT_SYMBOL_GPL(devm_voltdm_register);

static int devm_vdev_match(struct device *dev, void *res, void *data)
{
	struct pm_voltdm_dev **r = res;

	if (!r || !*r) {
		WARN_ON(!r || !*r);
		return 0;
	}
	return *r == data;
}

/**
 * devm_voltdm_unregister - Resource managed voltagedomain unregister
 * @vdev: voltage domain device returned by devm_voltdm_register()
 *
 * Unregister a voltdm registered with devm_voltdm_register().
 * Normally this function will not need to be called and the resource
 * management code will ensure that the resource is freed.
 */
void devm_voltdm_unregister(struct pm_voltdm_dev *vdev)
{
	int rc;
	struct device *dev = vdev->dev;

	rc = devres_release(dev, devm_voltdm_release, devm_vdev_match, vdev);
	if (rc != 0)
		WARN_ON(rc);
}
EXPORT_SYMBOL_GPL(devm_voltdm_unregister);
