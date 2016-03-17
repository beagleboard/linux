/*
 * TI SCI Generic Power Domain Driver
 *
 * Copyright (C) 2015-2016 Texas Instruments Incorporated - http://www.ti.com/
 *	J Keerthy <j-keerthy@ti.com>
 *	Dave Gerlach <d-gerlach@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/err.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/slab.h>
#include <linux/soc/ti/ti_sci_protocol.h>

#define TI_GENPD_NAME_LENGTH	16

/**
 * struct ti_sci_genpd_data: holds data needed for every power domain
 * @ti_sci: handle to TI SCI protocol driver that provides ops to
 *	    communicate with system control processor.
 * @dev: pointer to dev for the driver for devm allocs
 * @pd_list_mutex: Mutex for protecting the global list of power domains
 * @pd_list: list that hols all power domains as they are allocated
 */
struct ti_sci_genpd_data {
	const struct ti_sci_handle *ti_sci;
	struct device *dev;
	struct mutex pd_list_mutex; /* Protect master list of domains */
	struct list_head pd_list;
};

/**
 * struct ti_sci_pm_domain: TI specific data needed for each power domain
 * @idx: index of the power domain that identifies it with the system
 *	 control processor.
 * @pd: generic_pm_domain for use with the genpd framework
 * @node: list_head for tracking all domains in a list
 * @parent: pointer to the global data defined for all domains
 */
struct ti_sci_pm_domain {
	int idx;
	struct generic_pm_domain pd;
	struct list_head node;
	struct ti_sci_genpd_data *parent;
};

#define genpd_to_ti_sci_pd(gpd) container_of(gpd, struct ti_sci_pm_domain, pd)

/**
 * pd_power_off(): generic_pm_domain power_off hook
 * @domain: generic_pm_domain struct provided by genpd framework of the
 *	    pd to be shut off
 *
 * This hook uses the put_device dev_op provided by ti_sci to power off the
 * device associated to the power domain provided.
 */
static int pd_power_off(struct generic_pm_domain *domain)
{
	struct ti_sci_pm_domain *ti_sci_pd = genpd_to_ti_sci_pd(domain);
	const struct ti_sci_handle *ti_sci = ti_sci_pd->parent->ti_sci;

	return ti_sci->ops.dev_ops.put_device(ti_sci,
					      ti_sci_pd->idx);
}

/**
 * pd_power_on(): generic_pm_domain power_on hook
 * @domain: generic_pm_domain struct provided by genpd framework of the
 *	    pd to be powered on
 *
 * This hook uses the get_device dev_op provided by ti_sci to power on the
 * device associated to the power domain provided.
 */
static int pd_power_on(struct generic_pm_domain *domain)
{
	struct ti_sci_pm_domain *ti_sci_pd = genpd_to_ti_sci_pd(domain);
	const struct ti_sci_handle *ti_sci = ti_sci_pd->parent->ti_sci;

	return ti_sci->ops.dev_ops.get_device(ti_sci,
					      ti_sci_pd->idx);
}

/**
 * of_ti_sci_genpd_xlate_onecell() - Xlate function using a single index.
 * @genpdspec: OF phandle args to define an index to be communicated over
 *	       TI SCI to the system control processor to identify it
 * @data: xlate function private data - pointer to the ti_sci_genpd_data
 *	  struct containing global sci pm domain data
 *
 * This is xlate function takes a single cell as an index representing the
 * id to be passed to the system control processor. As each device in the
 * device tree probes a single pm domain will be created specifically for it
 * based on the index passed in the pweor-domains property. If no pm domain
 * yet exists for that index it is created, otherwise it is looked up and
 * returned. Through this 1 to 1 association between power domains and
 * devices the genpd framework will work to directly control devices
 * through the pm_runtime framework.
 */
static struct generic_pm_domain *of_ti_sci_genpd_xlate_onecell(
					struct of_phandle_args *genpdspec,
					void *data)
{
	struct ti_sci_genpd_data *ti_sci_genpd = data;
	const struct ti_sci_handle *ti_sci = ti_sci_genpd->ti_sci;
	struct device *dev = ti_sci_genpd->dev;
	unsigned int idx = genpdspec->args[0];
	struct ti_sci_pm_domain *ti_sci_pd = NULL, *pd;
	char *name;
	int ret = 0;

	if (genpdspec->args_count != 1)
		return ERR_PTR(-EINVAL);

	/*
	 * Check the validity of the requested idx, if the index is not valid
	 * the PMMC will return a NAK here and we will not allocate it.
	 */
	ret = ti_sci->ops.dev_ops.is_valid(ti_sci, idx);
	if (ret)
		return ERR_PTR(-EINVAL);

	mutex_lock(&ti_sci_genpd->pd_list_mutex);
	list_for_each_entry(pd, &ti_sci_genpd->pd_list, node) {
		if (pd->idx == idx) {
			ti_sci_pd = pd;
			goto unlock_and_return;
		}
	}

	ti_sci_pd = devm_kzalloc(dev,
				 sizeof(*ti_sci_pd),
				 GFP_KERNEL);
	if (!ti_sci_pd) {
		ret = -ENOMEM;
		goto unlock_and_return;
	}

	ti_sci_pd->idx = idx;
	ti_sci_pd->pd.power_off = pd_power_off;
	ti_sci_pd->pd.power_on = pd_power_on;

	name = devm_kzalloc(dev, TI_GENPD_NAME_LENGTH, GFP_KERNEL);
	if (!name) {
		devm_kfree(dev, ti_sci_pd);
		ret = -ENOMEM;
		goto unlock_and_return;
	}

	snprintf(name, TI_GENPD_NAME_LENGTH, "pd-%d", idx);
	ti_sci_pd->pd.name = name;

	ti_sci_pd->parent = ti_sci_genpd;
	/*
	 * Init each pd as is_off so we always call pd_power_on
	 * to make sure reference counting is properly maintained
	 * on the SCI side
	 */
	pm_genpd_init(&ti_sci_pd->pd, NULL, true);

	list_add(&ti_sci_pd->node, &ti_sci_genpd->pd_list);

unlock_and_return:
	mutex_unlock(&ti_sci_genpd->pd_list_mutex);

	if (ret)
		return ERR_PTR(ret);
	else
		return &ti_sci_pd->pd;
}

static const struct of_device_id ti_sci_pm_domain_matches[] = {
	{ .compatible = "ti,sci-pm-domains", },
	{ },
};
MODULE_DEVICE_TABLE(of, ti_sci_pm_domain_matches);

static int ti_sci_pm_domains_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct ti_sci_genpd_data *ti_sci_genpd;

	ti_sci_genpd = devm_kzalloc(dev, sizeof(*ti_sci_genpd), GFP_KERNEL);
	if (!ti_sci_genpd)
		return -ENOMEM;

	ti_sci_genpd->ti_sci = devm_ti_sci_get_handle(dev);
	if (IS_ERR(ti_sci_genpd->ti_sci))
		return PTR_ERR(ti_sci_genpd->ti_sci);

	ti_sci_genpd->dev = dev;

	INIT_LIST_HEAD(&ti_sci_genpd->pd_list);
	mutex_init(&ti_sci_genpd->pd_list_mutex);

	return __of_genpd_add_provider(np, of_ti_sci_genpd_xlate_onecell,
				       ti_sci_genpd);
}

static struct platform_driver ti_sci_pm_domains_driver = {
	.probe = ti_sci_pm_domains_probe,
	.driver = {
		.name = "ti_sci_pm_domains",
		.of_match_table = ti_sci_pm_domain_matches,
	},
};
module_platform_driver(ti_sci_pm_domains_driver);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("TI System Control Interface(SCI) Power Domain driver");
MODULE_AUTHOR("Dave Gerlach");
