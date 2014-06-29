/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * The OPP code in function cpu0_set_target() is reused from
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
#include <linux/cpumask.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_opp.h>
#include <linux/platform_device.h>
#include <linux/pm_voltage_domain.h>
#include <linux/slab.h>
#include <linux/thermal.h>

static unsigned int transition_latency;

static struct device *cpu_dev;
static struct clk *cpu_clk;
static struct cpufreq_frequency_table *freq_table;
static struct thermal_cooling_device *cdev;
static struct notifier_block *clk_nb;

static int cpu0_set_target(struct cpufreq_policy *policy, unsigned int index)
{
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
		pr_err("failed to set clock rate: %d\n", ret);
		return ret;
	}

	return ret;
}

static int cpu0_cpufreq_init(struct cpufreq_policy *policy)
{
	policy->clk = cpu_clk;
	return cpufreq_generic_init(policy, freq_table, transition_latency);
}

static struct cpufreq_driver cpu0_cpufreq_driver = {
	.flags = CPUFREQ_STICKY,
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = cpu0_set_target,
	.get = cpufreq_generic_get,
	.init = cpu0_cpufreq_init,
	.exit = cpufreq_generic_exit,
	.name = "generic_cpu0",
	.attr = cpufreq_generic_attr,
};

static int cpu0_cpufreq_probe(struct platform_device *pdev)
{
	struct device_node *np;
	unsigned int voltage_latency;
	int ret;

	if (!cpu_dev) {
		cpu_dev = get_cpu_device(0);
		if (!cpu_dev) {
			pr_err("failed to get cpu0 device\n");
			return -ENODEV;
		}

		np = of_node_get(cpu_dev->of_node);
		ret = of_init_opp_table(cpu_dev);
		if (ret) {
			pr_err("failed to init OPP table: %d\n", ret);
			goto out_put_node;
		}
	} else {
		np = of_node_get(cpu_dev->of_node);
	}

	cpu_clk = clk_get(cpu_dev, NULL);
	if (IS_ERR(cpu_clk)) {
		ret = PTR_ERR(cpu_clk);
		pr_err("failed to get cpu0 clock: %d\n", ret);
		goto out_put_node;
	}

	if (of_property_read_u32(np, "clock-latency", &transition_latency))
		transition_latency = CPUFREQ_ETERNAL;

	ret = dev_pm_opp_init_cpufreq_table(cpu_dev, &freq_table);
	if (ret) {
		pr_err("failed to init cpufreq table: %d\n", ret);
		goto out_put_clk;
	}

	clk_nb = of_pm_voltdm_notifier_register(cpu_dev, np, cpu_clk, "cpu0",
						&voltage_latency);

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
		goto out_freq_table_free;
	}

	if (voltage_latency > 0)
		transition_latency += voltage_latency;

	ret = cpufreq_register_driver(&cpu0_cpufreq_driver);
	if (ret) {
		pr_err("failed register driver: %d\n", ret);
		goto out_notifier_unregister;
	}

	/*
	 * For now, just loading the cooling device;
	 * thermal DT code takes care of matching them.
	 */
	if (of_find_property(np, "#cooling-cells", NULL)) {
		cdev = of_cpufreq_cooling_register(np, cpu_present_mask);
		if (IS_ERR(cdev))
			pr_err("running cpufreq without cooling device: %ld\n",
			       PTR_ERR(cdev));
	}

	of_node_put(np);
	return 0;

out_notifier_unregister:
	of_pm_voltdm_notifier_unregister(clk_nb);
out_freq_table_free:
	dev_pm_opp_free_cpufreq_table(cpu_dev, &freq_table);
out_put_clk:
	if (!IS_ERR(cpu_clk))
		clk_put(cpu_clk);
out_put_node:
	of_node_put(np);
	return ret;
}

static int cpu0_cpufreq_remove(struct platform_device *pdev)
{
	cpufreq_cooling_unregister(cdev);
	cpufreq_unregister_driver(&cpu0_cpufreq_driver);
	of_pm_voltdm_notifier_unregister(clk_nb);
	dev_pm_opp_free_cpufreq_table(cpu_dev, &freq_table);

	return 0;
}

static struct platform_driver cpu0_cpufreq_platdrv = {
	.driver = {
		.name	= "cpufreq-cpu0",
		.owner	= THIS_MODULE,
	},
	.probe		= cpu0_cpufreq_probe,
	.remove		= cpu0_cpufreq_remove,
};
module_platform_driver(cpu0_cpufreq_platdrv);

MODULE_AUTHOR("Shawn Guo <shawn.guo@linaro.org>");
MODULE_DESCRIPTION("Generic CPU0 cpufreq driver");
MODULE_LICENSE("GPL");
