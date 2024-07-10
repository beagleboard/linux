// SPDX-License-Identifier: GPL-2.0-only
/*
 * TI SCI Generic Power Domain Driver
 *
 * Copyright (C) 2015-2017 Texas Instruments Incorporated - http://www.ti.com/
 *	J Keerthy <j-keerthy@ti.com>
 *	Dave Gerlach <d-gerlach@ti.com>
 */

#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/slab.h>
#include <linux/pm_qos.h>
#include <linux/pm_runtime.h>
#include <linux/soc/ti/ti_sci_protocol.h>
#include <dt-bindings/soc/ti,sci_pm_domain.h>

/**
 * struct ti_sci_genpd_provider: holds common TI SCI genpd provider data
 * @ti_sci: handle to TI SCI protocol driver that provides ops to
 *	    communicate with system control processor.
 * @dev: pointer to dev for the driver for devm allocs
 * @pd_list: list of all the power domains on the device
 * @data: onecell data for genpd core
 */
struct ti_sci_genpd_provider {
	const struct ti_sci_handle *ti_sci;
	struct device *dev;
	struct list_head pd_list;
	struct genpd_onecell_data data;
};

/**
 * struct ti_sci_pm_domain: TI specific data needed for power domain
 * @idx: index of the device that identifies it with the system
 *	 control processor.
 * @exclusive: Permissions for exclusive request or shared request of the
 *	       device.
 * @pd: generic_pm_domain for use with the genpd framework
 * @node: link for the genpd list
 * @parent: link to the parent TI SCI genpd provider
 */
struct ti_sci_pm_domain {
	int idx;
	u8 exclusive;
	struct generic_pm_domain pd;
	struct list_head node;
	struct ti_sci_genpd_provider *parent;
	s32 lat_constraint;
	bool constraint_sent;
	bool wkup_constraint;
};

#define genpd_to_ti_sci_pd(gpd) container_of(gpd, struct ti_sci_pm_domain, pd)

static inline bool ti_sci_pd_is_valid_constraint(s32 val)
{
	return val != PM_QOS_RESUME_LATENCY_NO_CONSTRAINT;
}

static int ti_sci_pd_send_constraint(struct device *dev, s32 val)
{
	struct generic_pm_domain *genpd = pd_to_genpd(dev->pm_domain);
	struct ti_sci_pm_domain *pd = genpd_to_ti_sci_pd(genpd);
	const struct ti_sci_handle *ti_sci = pd->parent->ti_sci;
	int ret;

	ret = ti_sci->ops.pm_ops.set_latency_constraint(ti_sci, val, TISCI_MSG_CONSTRAINT_SET);
	if (!ret)
		pd->constraint_sent = true;

	WARN_ON(ret != 0);
	return ret;
}

static inline void ti_sci_pd_clear_constraints(struct device *dev)
{
	struct generic_pm_domain *genpd = pd_to_genpd(dev->pm_domain);
	struct ti_sci_pm_domain *pd = genpd_to_ti_sci_pd(genpd);

	pd->lat_constraint = PM_QOS_RESUME_LATENCY_NO_CONSTRAINT;
	pd->constraint_sent = false;
	pd->wkup_constraint = false;
}

static inline bool ti_sci_pd_check_wkup_constraint(struct device *dev)
{
	struct generic_pm_domain *genpd = pd_to_genpd(dev->pm_domain);
	struct ti_sci_pm_domain *pd = genpd_to_ti_sci_pd(genpd);
	const struct ti_sci_handle *ti_sci = pd->parent->ti_sci;
	int ret;

	if (device_may_wakeup(dev)) {
		/*
		 * If device can wakeup using IO daisy chain wakeups,
		 * we do not want to set a constraint.
		 */
		if (dev->power.wakeirq) {
			dev_info(dev, "%s: has wake IRQ, not setting constraints\n", __func__);
			return false;
		}

		ret = ti_sci->ops.pm_ops.set_device_constraint(ti_sci, pd->idx,
							       TISCI_MSG_CONSTRAINT_SET);
		if (!ret)
			pd->wkup_constraint = true;
	}

	return pd->wkup_constraint;
}

/*
 * ti_sci_pd_power_off(): genpd power down hook
 * @domain: pointer to the powerdomain to power off
 */
static int ti_sci_pd_power_off(struct generic_pm_domain *domain)
{
	struct ti_sci_pm_domain *pd = genpd_to_ti_sci_pd(domain);
	const struct ti_sci_handle *ti_sci = pd->parent->ti_sci;
	struct pm_domain_data *pdd;

	list_for_each_entry(pdd, &domain->dev_list, list_node) {
		struct device *dev = pdd->dev;
		s32 val;

		/* If device has any resume latency constraints, send 'em */
		val = dev_pm_qos_read_value(dev, DEV_PM_QOS_RESUME_LATENCY);
		if (ti_sci_pd_is_valid_constraint(val) && !pd->constraint_sent)
			ti_sci_pd_send_constraint(dev, val);
		pd->lat_constraint = val;
	}

	return ti_sci->ops.dev_ops.put_device(ti_sci, pd->idx);
}

/*
 * ti_sci_pd_power_on(): genpd power up hook
 * @domain: pointer to the powerdomain to power on
 */
static int ti_sci_pd_power_on(struct generic_pm_domain *domain)
{
	struct ti_sci_pm_domain *pd = genpd_to_ti_sci_pd(domain);
	const struct ti_sci_handle *ti_sci = pd->parent->ti_sci;
	struct pm_domain_data *pdd;

	list_for_each_entry(pdd, &domain->dev_list, list_node) {
		ti_sci_pd_clear_constraints(pdd->dev);
	}

	if (pd->exclusive)
		return ti_sci->ops.dev_ops.get_device_exclusive(ti_sci,
								pd->idx);
	else
		return ti_sci->ops.dev_ops.get_device(ti_sci, pd->idx);
}

#if IS_ENABLED(CONFIG_SUSPEND)
static int ti_sci_pd_resume(struct device *dev)
{
	ti_sci_pd_clear_constraints(dev);
	return pm_generic_resume(dev);
}

static int ti_sci_pd_suspend(struct device *dev)
{
	struct generic_pm_domain *genpd = pd_to_genpd(dev->pm_domain);
	struct ti_sci_pm_domain *pd = genpd_to_ti_sci_pd(genpd);
	s32 val;

	/* Check if device has any resume latency constraints */
	val = dev_pm_qos_read_value(dev, DEV_PM_QOS_RESUME_LATENCY);
	if (ti_sci_pd_is_valid_constraint(val) && !pd->constraint_sent) {
		if (genpd && genpd->status == GENPD_STATE_OFF)
			dev_warn(dev, "%s: %s: already off.\n", genpd->name, __func__);
		else if (pm_runtime_suspended(dev))
			dev_warn(dev, "%s: %s: already RPM suspended.\n", genpd->name, __func__);
		else
			ti_sci_pd_send_constraint(dev, val);
	}
	pd->lat_constraint = val;

	ti_sci_pd_check_wkup_constraint(dev);

	return pm_generic_suspend(dev);
}

static int ti_sci_pd_suspend_late(struct device *dev)
{
	struct generic_pm_domain *genpd = pd_to_genpd(dev->pm_domain);
	struct ti_sci_pm_domain *pd = genpd_to_ti_sci_pd(genpd);

	if (pm_runtime_suspended(dev)) {
		if (genpd && genpd->status == GENPD_STATE_OFF)
			dev_warn(dev, "%s: RPM suspended but genpd %s still on.\n",
				 __func__, genpd->name);
	}

	if (ti_sci_pd_is_valid_constraint(pd->lat_constraint) &&
	    !pd->constraint_sent)
		dev_warn(dev, "%s: %s: valid constraint (%d), but NOT sent!\n",
			 genpd->name, __func__, pd->lat_constraint);

	return pm_generic_suspend_late(dev);
}
#endif /* CONFIG_SUSPEND */

/*
 * ti_sci_pd_xlate(): translation service for TI SCI genpds
 * @genpdspec: DT identification data for the genpd
 * @data: genpd core data for all the powerdomains on the device
 */
static struct generic_pm_domain *ti_sci_pd_xlate(
					struct of_phandle_args *genpdspec,
					void *data)
{
	struct genpd_onecell_data *genpd_data = data;
	unsigned int idx = genpdspec->args[0];

	if (genpdspec->args_count != 1 && genpdspec->args_count != 2)
		return ERR_PTR(-EINVAL);

	if (idx >= genpd_data->num_domains) {
		pr_err("%s: invalid domain index %u\n", __func__, idx);
		return ERR_PTR(-EINVAL);
	}

	if (!genpd_data->domains[idx])
		return ERR_PTR(-ENOENT);

	genpd_to_ti_sci_pd(genpd_data->domains[idx])->exclusive =
		genpdspec->args[1];

	return genpd_data->domains[idx];
}

static const struct of_device_id ti_sci_pm_domain_matches[] = {
	{ .compatible = "ti,sci-pm-domain", },
	{ },
};
MODULE_DEVICE_TABLE(of, ti_sci_pm_domain_matches);

static int ti_sci_pm_domain_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ti_sci_genpd_provider *pd_provider;
	struct ti_sci_pm_domain *pd;
	struct device_node *np;
	struct of_phandle_args args;
	int ret;
	u32 max_id = 0;
	int index;

	pd_provider = devm_kzalloc(dev, sizeof(*pd_provider), GFP_KERNEL);
	if (!pd_provider)
		return -ENOMEM;

	pd_provider->ti_sci = devm_ti_sci_get_handle(dev);
	if (IS_ERR(pd_provider->ti_sci))
		return PTR_ERR(pd_provider->ti_sci);

	pd_provider->dev = dev;

	INIT_LIST_HEAD(&pd_provider->pd_list);

	/* Find highest device ID used for power domains */
	for_each_node_with_property(np, "power-domains") {
		index = 0;

		while (1) {
			ret = of_parse_phandle_with_args(np, "power-domains",
							 "#power-domain-cells",
							 index, &args);
			if (ret)
				break;

			if (args.args_count >= 1 && args.np == dev->of_node) {
				if (args.args[0] > max_id)
					max_id = args.args[0];

				pd = devm_kzalloc(dev, sizeof(*pd), GFP_KERNEL);
				if (!pd)
					return -ENOMEM;

				pd->pd.name = devm_kasprintf(dev, GFP_KERNEL,
							     "pd:%d",
							     args.args[0]);
				if (!pd->pd.name)
					return -ENOMEM;

				pd->pd.power_off = ti_sci_pd_power_off;
				pd->pd.power_on = ti_sci_pd_power_on;
				pd->idx = args.args[0];
				pd->parent = pd_provider;
				pd->lat_constraint = PM_QOS_RESUME_LATENCY_NO_CONSTRAINT;
				/*
				 * If SCI constraint functions are present, then firmware
				 * supports the constraints API.
				 */
#if IS_ENABLED(CONFIG_SUSPEND)
				if (pd_provider->ti_sci->ops.pm_ops.set_device_constraint) {
					pd->pd.domain.ops.resume = ti_sci_pd_resume;
					pd->pd.domain.ops.suspend = ti_sci_pd_suspend;
					pd->pd.domain.ops.suspend_late = ti_sci_pd_suspend_late;
				}
#endif
				pm_genpd_init(&pd->pd, NULL, true);

				list_add(&pd->node, &pd_provider->pd_list);
			}
			index++;
		}
	}

	pd_provider->data.domains =
		devm_kcalloc(dev, max_id + 1,
			     sizeof(*pd_provider->data.domains),
			     GFP_KERNEL);
	if (!pd_provider->data.domains)
		return -ENOMEM;

	pd_provider->data.num_domains = max_id + 1;
	pd_provider->data.xlate = ti_sci_pd_xlate;

	list_for_each_entry(pd, &pd_provider->pd_list, node)
		pd_provider->data.domains[pd->idx] = &pd->pd;

	return of_genpd_add_provider_onecell(dev->of_node, &pd_provider->data);
}

static struct platform_driver ti_sci_pm_domains_driver = {
	.probe = ti_sci_pm_domain_probe,
	.driver = {
		.name = "ti_sci_pm_domains",
		.of_match_table = ti_sci_pm_domain_matches,
	},
};
module_platform_driver(ti_sci_pm_domains_driver);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("TI System Control Interface (SCI) Power Domain driver");
MODULE_AUTHOR("Dave Gerlach");
