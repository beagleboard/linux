/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * Copyright (C) 2014 Linaro.
 * Viresh Kumar <viresh.kumar@linaro.org>
 *
 * The OPP code in function set_target() is reused from
 * drivers/cpufreq/omap-cpufreq.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt)	KBUILD_MODNAME ": " fmt

#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/cpu_cooling.h>
#include <linux/cpufreq.h>
#include <linux/cpufreq-dt.h>
#include <linux/cpumask.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_opp.h>
#include <linux/pm_voltage_domain.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/thermal.h>

struct private_data {
	struct device *cpu_dev;
	struct thermal_cooling_device *cdev;
	struct notifier_block *clk_nb;
};

static int set_target(struct cpufreq_policy *policy, unsigned int index)
{
	struct cpufreq_frequency_table *freq_table = policy->freq_table;
	struct clk *cpu_clk = policy->clk;
	struct private_data *priv = policy->driver_data;
	struct device *cpu_dev = priv->cpu_dev;
	unsigned int old_freq, new_freq;
	long freq_Hz, freq_exact;
	int ret;

	freq_Hz = clk_round_rate(cpu_clk, freq_table[index].frequency * 1000);
	if (freq_Hz <= 0)
		freq_Hz = freq_table[index].frequency * 1000;

	freq_exact = freq_Hz;
	new_freq = freq_Hz / 1000;
	old_freq = clk_get_rate(cpu_clk) / 1000;

	pr_debug("%u MHz --> %u MHz\n", old_freq / 1000, new_freq / 1000);

	ret = clk_set_rate(cpu_clk, freq_exact);
	if (ret) {
		dev_err(cpu_dev, "failed to set clock rate: %d\n", ret);
		return ret;
	}

	return ret;
}

static int allocate_resources(int cpu, struct device **cdev,
			      struct clk **cclk)
{
	struct device *cpu_dev;
	struct clk *cpu_clk;
	int ret = 0;

	cpu_dev = get_cpu_device(cpu);
	if (!cpu_dev) {
		pr_err("failed to get cpu%d device\n", cpu);
		return -ENODEV;
	}

	cpu_clk = clk_get(cpu_dev, NULL);
	if (IS_ERR(cpu_clk)) {
		ret = PTR_ERR(cpu_clk);

		/*
		 * If cpu's clk node is present, but clock is not yet
		 * registered, we should try defering probe.
		 */
		if (ret == -EPROBE_DEFER)
			dev_dbg(cpu_dev, "cpu%d clock not ready, retry\n", cpu);
		else
			dev_err(cpu_dev, "failed to get cpu%d clock: %d\n", cpu,
				ret);
	} else {
		*cdev = cpu_dev;
		*cclk = cpu_clk;
	}

	return ret;
}

static int cpufreq_init(struct cpufreq_policy *policy)
{
	struct cpufreq_dt_platform_data *pd;
	struct cpufreq_frequency_table *freq_table;
	struct device_node *np;
	struct private_data *priv;
	struct device *cpu_dev;
	struct clk *cpu_clk;
	unsigned int transition_latency, voltage_latency;
	int ret;

	ret = allocate_resources(policy->cpu, &cpu_dev, &cpu_clk);
	if (ret) {
		pr_err("%s: Failed to allocate resources: %d\n", __func__, ret);
		return ret;
	}

	np = of_node_get(cpu_dev->of_node);
	if (!np) {
		dev_err(cpu_dev, "failed to find cpu%d node\n", policy->cpu);
		ret = -ENOENT;
		goto out_put_clk;
	}

	/* OPPs might be populated at runtime, don't check for error here */
	dev_pm_opp_of_add_table(cpu_dev);

	/*
	 * But we need OPP table to function so if it is not there let's
	 * give platform code chance to provide it for us.
	 */
	ret = dev_pm_opp_get_opp_count(cpu_dev);
	if (ret <= 0) {
		pr_debug("OPP table is not ready, deferring probe\n");
		ret = -EPROBE_DEFER;
		goto out_free_opp;
	}

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		ret = -ENOMEM;
		goto out_free_opp;
	}

	if (of_property_read_u32(np, "clock-latency", &transition_latency))
		transition_latency = CPUFREQ_ETERNAL;

	ret = dev_pm_opp_init_cpufreq_table(cpu_dev, &freq_table);
	if (ret) {
		pr_err("failed to init cpufreq table: %d\n", ret);
		goto out_free_priv;
	}

	priv->cpu_dev = cpu_dev;
	policy->driver_data = priv;

	policy->clk = cpu_clk;
	ret = cpufreq_table_validate_and_show(policy, freq_table);
	if (ret) {
		dev_err(cpu_dev, "%s: invalid frequency table: %d\n", __func__,
			ret);
		goto out_free_cpufreq_table;
	}

	priv->clk_nb = of_pm_voltdm_notifier_register(cpu_dev, np,
						      cpu_clk, "cpu0",
						      &voltage_latency);

	if (IS_ERR(priv->clk_nb)) {
		ret = PTR_ERR(priv->clk_nb);
		/* defer probe if regulator is not yet registered */
		if (ret == -EPROBE_DEFER) {
			dev_err(cpu_dev,
				"cpu0 clock notifier not ready, retry\n");
		} else {
			dev_err(cpu_dev,
				"Failed to register cpu0 clock notifier: %d\n",
				ret);
		}

		goto out_free_cpufreq_table;
	}

	if (voltage_latency > 0)
		transition_latency += voltage_latency;

	policy->cpuinfo.transition_latency = transition_latency;

	pd = cpufreq_get_driver_data();
	if (!pd || !pd->independent_clocks)
		cpumask_setall(policy->cpus);

	of_node_put(np);

	return 0;

out_free_cpufreq_table:
	dev_pm_opp_free_cpufreq_table(cpu_dev, &freq_table);
out_free_priv:
	kfree(priv);
out_free_opp:
	dev_pm_opp_of_remove_table(cpu_dev);
	of_node_put(np);
out_put_clk:
	clk_put(cpu_clk);

	return ret;
}

static int cpufreq_exit(struct cpufreq_policy *policy)
{
	struct private_data *priv = policy->driver_data;

	cpufreq_cooling_unregister(priv->cdev);
	dev_pm_opp_free_cpufreq_table(priv->cpu_dev, &policy->freq_table);
	of_pm_voltdm_notifier_unregister(priv->clk_nb);
	dev_pm_opp_of_remove_table(priv->cpu_dev);
	clk_put(policy->clk);
	kfree(priv);

	return 0;
}

static void cpufreq_ready(struct cpufreq_policy *policy)
{
	struct private_data *priv = policy->driver_data;
	struct device_node *np = of_node_get(priv->cpu_dev->of_node);

	if (WARN_ON(!np))
		return;

	/*
	 * For now, just loading the cooling device;
	 * thermal DT code takes care of matching them.
	 */
	if (of_find_property(np, "#cooling-cells", NULL)) {
		priv->cdev = of_cpufreq_cooling_register(np,
							 policy->related_cpus);
		if (IS_ERR(priv->cdev)) {
			dev_err(priv->cpu_dev,
				"running cpufreq without cooling device: %ld\n",
				PTR_ERR(priv->cdev));

			priv->cdev = NULL;
		}
	}

	of_node_put(np);
}

static struct cpufreq_driver voltdm_cpufreq_driver = {
	.flags = CPUFREQ_STICKY | CPUFREQ_NEED_INITIAL_FREQ_CHECK,
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = set_target,
	.get = cpufreq_generic_get,
	.init = cpufreq_init,
	.exit = cpufreq_exit,
	.ready = cpufreq_ready,
	.name = "cpufreq-voltdm",
	.attr = cpufreq_generic_attr,
};

static int voltdm_cpufreq_probe(struct platform_device *pdev)
{
	struct device *cpu_dev;
	struct clk *cpu_clk;
	int ret, tmp;
	struct notifier_block *clk_nb;
	struct device_node *np;

	/*
	 * All per-cluster (CPUs sharing clock/voltages) initialization is done
	 * from ->init(). In probe(), we just need to make sure that clk is
	 * available. Else defer probe and retry.
	 *
	 * FIXME: Is checking this only for CPU0 sufficient ?
	 */
	ret = allocate_resources(0, &cpu_dev, &cpu_clk);
	if (ret)
		return ret;

	np = cpu_dev->of_node;

	/* OPPs might be populated at runtime, This is just a dummy setup */
	dev_pm_opp_of_add_table(cpu_dev);

	clk_nb = of_pm_voltdm_notifier_register(cpu_dev, np,
						cpu_clk, "cpu0", &tmp);
	dev_pm_opp_of_remove_table(cpu_dev);

	if (IS_ERR(clk_nb)) {
		ret = PTR_ERR(clk_nb);
		/* defer probe if regulator is not yet registered */
		if (ret == -EPROBE_DEFER) {
			dev_err(cpu_dev,
				"cpu0 clock notifier not ready, retry\n");
		} else {
			dev_err(cpu_dev,
				"Failed to register cpu0 clock notifier: %d\n",
				ret);
		}

		clk_put(cpu_clk);
		return ret;
	}

	if (clk_nb)
		of_pm_voltdm_notifier_unregister(clk_nb);

	clk_put(cpu_clk);

	voltdm_cpufreq_driver.driver_data = dev_get_platdata(&pdev->dev);

	ret = cpufreq_register_driver(&voltdm_cpufreq_driver);
	if (ret)
		dev_err(cpu_dev, "failed register driver: %d\n", ret);

	return ret;
}

static int voltdm_cpufreq_remove(struct platform_device *pdev)
{
	cpufreq_unregister_driver(&voltdm_cpufreq_driver);
	return 0;
}

static struct platform_driver voltdm_cpufreq_platdrv = {
	.driver = {
		.name	= "cpufreq-voltdm",
	},
	.probe		= voltdm_cpufreq_probe,
	.remove		= voltdm_cpufreq_remove,
};
module_platform_driver(voltdm_cpufreq_platdrv);

MODULE_AUTHOR("Viresh Kumar <viresh.kumar@linaro.org>");
MODULE_AUTHOR("Shawn Guo <shawn.guo@linaro.org>");
MODULE_DESCRIPTION("Generic cpufreq driver with clk notifier support");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:cpufreq-voltdm");
