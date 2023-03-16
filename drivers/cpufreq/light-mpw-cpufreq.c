/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2021 Alibaba Group Holding Limited.
 */

#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/pm_opp.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/regulator/consumer.h>
#include <linux/suspend.h>
#include <linux/clk-provider.h>
#include <linux/smp.h>

static DEFINE_MUTEX(cpufreq_lock);

bool cpufreq_denied = false;

struct regulator *dvdd_cpu_reg;
struct regulator *dvddm_cpu_reg;

enum LIGHT_MPW_CPUFREQ_CLKS {
	LIGHT_C910_CCLK,
	LIGHT_C910_CCLK_I0,
	LIGHT_CPU_PLL1_FOUTPOSTDIV,
	LIGHT_CPU_PLL0_FOUTPOSTDIV,
};

#define LIGHT_MPW_CPUFREQ_CLK_NUM	4
#define LIGHT_CPUFREQ_THRE		2000000
#define LIGHT_C910_BUS_CLK_SYNC		BIT(11)
#define LIGHT_C910_BUS_CLK_RATIO_MASK	0x700
#define LIGHT_C910_BUS_CLK_DIV_RATIO_2	0x100
#define LIGHT_C910_BUS_CLK_DIV_RATIO_3	0x200

static int num_clks;
static struct clk_bulk_data clks[] = {
	{ .id = "c910_cclk" },
	{ .id = "c910_cclk_i0" },
	{ .id = "cpu_pll1_foutpostdiv" },
	{ .id = "cpu_pll0_foutpostdiv" },
};

static struct device *cpu_dev;
static struct cpufreq_frequency_table *freq_table;
static unsigned int max_freq;
static unsigned int transition_latency;
static void __iomem *ap_sys_reg;
static bool light_dvfs_sv = false;

static u32 *light_dvddm_volt;
static u32 soc_opp_count = 0;

static int light_set_target(struct cpufreq_policy *policy, unsigned int index)
{
	struct dev_pm_opp *opp;
	unsigned long freq_hz;
	int volt, volt_old;
	unsigned int old_freq, new_freq;
	int ret;
	u32 val;
	u32 re_modify_bus_freq = 0;

	mutex_lock(&cpufreq_lock);

	if (cpufreq_denied) {
		dev_emerg(cpu_dev, "Denied to set cpu frequency temporarily on reboot\n");
		mutex_unlock(&cpufreq_lock);
		return 0;
	}
	new_freq = freq_table[index].frequency;
	freq_hz = new_freq * 1000;
	old_freq = policy->cur;

	opp = dev_pm_opp_find_freq_ceil(cpu_dev, &freq_hz);
	if (IS_ERR(opp)) {
		dev_err(cpu_dev, "failed to find OPP for %ld\n", freq_hz);
		mutex_unlock(&cpufreq_lock);
		return PTR_ERR(opp);
	}

	volt = dev_pm_opp_get_voltage(opp);
	dev_pm_opp_put(opp);

	volt_old = regulator_get_voltage(dvdd_cpu_reg);
	if (volt_old < 0) {
		dev_err(cpu_dev, "failed to get cpu voltage\n");
		mutex_unlock(&cpufreq_lock);
		return volt_old;
	}

	dev_dbg(cpu_dev, "%u MHz, %d mV --> %u MHz, %d mV\n",
		old_freq / 1000, volt_old / 1000,
		new_freq / 1000, volt / 1000);

	/* change AXI bus clock ratio to match: BUS_CLK = CPU_CCLK/ratio <= 750MHz */
	val = readl(ap_sys_reg);
	if (new_freq > LIGHT_CPUFREQ_THRE) {
		val &= ~LIGHT_C910_BUS_CLK_RATIO_MASK;
		val |= LIGHT_C910_BUS_CLK_DIV_RATIO_3;
	} else {
		val &= ~LIGHT_C910_BUS_CLK_RATIO_MASK;

		if (old_freq > LIGHT_CPUFREQ_THRE) {
			re_modify_bus_freq = 1;
			val |= LIGHT_C910_BUS_CLK_DIV_RATIO_3;
		}else
			val |= LIGHT_C910_BUS_CLK_DIV_RATIO_2;
	}

	writel(val, ap_sys_reg);
	val &= ~LIGHT_C910_BUS_CLK_SYNC;
	writel(val, ap_sys_reg);
	udelay(1);
	val |= LIGHT_C910_BUS_CLK_SYNC;
	writel(val, ap_sys_reg);
	udelay(1);

	/* scaling up?  scale voltage before frequency */
	if (new_freq > old_freq && !light_dvfs_sv) {
		ret = regulator_set_voltage_tol(dvddm_cpu_reg, light_dvddm_volt[index], 0);
		if (ret) {
			dev_err(cpu_dev, "failed to scale vddsoc up: %d\n", ret);
			mutex_unlock(&cpufreq_lock);
			return ret;
		}
		ret = regulator_set_voltage_tol(dvdd_cpu_reg, volt, 0);
		if (ret) {
			dev_err(cpu_dev,
				"failed to scale vddarm up: %d\n", ret);
			mutex_unlock(&cpufreq_lock);
			return ret;
		}
	}

	if (!strcmp(__clk_get_name(clk_get_parent(clks[LIGHT_C910_CCLK].clk)),
	    __clk_get_name(clks[LIGHT_C910_CCLK_I0].clk))) {
		clk_prepare_enable(clks[LIGHT_CPU_PLL1_FOUTPOSTDIV].clk);
		clk_set_rate(clks[LIGHT_CPU_PLL1_FOUTPOSTDIV].clk, new_freq * 1000);
		ret = clk_set_parent(clks[LIGHT_C910_CCLK].clk, clks[LIGHT_CPU_PLL1_FOUTPOSTDIV].clk);
		udelay(1);
		clk_disable_unprepare(clks[LIGHT_CPU_PLL0_FOUTPOSTDIV].clk);
	} else {
		clk_prepare_enable(clks[LIGHT_CPU_PLL0_FOUTPOSTDIV].clk);
		clk_set_rate(clks[LIGHT_CPU_PLL0_FOUTPOSTDIV].clk, new_freq * 1000);
		ret  = clk_set_parent(clks[LIGHT_C910_CCLK].clk, clks[LIGHT_C910_CCLK_I0].clk);
		udelay(1);
		clk_disable_unprepare(clks[LIGHT_CPU_PLL1_FOUTPOSTDIV].clk);
	}

	/*add delay for clk-switch*/
	udelay(1);

	/* Ensure the c910_cclk clock divider is what we expect */
	ret = clk_set_rate(clks[LIGHT_C910_CCLK].clk, new_freq * 1000);
	if (ret) {
		int ret1;

		dev_err(cpu_dev, "failed to set clock rate: %d\n", ret);
		ret1 = regulator_set_voltage_tol(dvdd_cpu_reg, volt_old, 0);
		if (ret1)
			dev_err(cpu_dev, "failed to restore dvdd_cpu voltage: %d\n", ret1);
		mutex_unlock(&cpufreq_lock);
		return ret;
	}

	/* scaling down?  scale voltage after frequency */
	if (new_freq < old_freq && !light_dvfs_sv) {
		ret = regulator_set_voltage_tol(dvddm_cpu_reg, light_dvddm_volt[index], 0);
		if (ret)
			dev_err(cpu_dev, "failed to scale dvddm down: %d\n", ret);
		ret = regulator_set_voltage_tol(dvdd_cpu_reg, volt, 0);
		if (ret)
			dev_err(cpu_dev, "failed to scale dvdd_cpu down: %d\n", ret);
	}

	val = readl(ap_sys_reg);
	if (re_modify_bus_freq) {
		val &= ~LIGHT_C910_BUS_CLK_RATIO_MASK;
		val |= LIGHT_C910_BUS_CLK_DIV_RATIO_2;

		writel(val, ap_sys_reg);
		val &= ~LIGHT_C910_BUS_CLK_SYNC;
		writel(val, ap_sys_reg);
		udelay(1);
		val |= LIGHT_C910_BUS_CLK_SYNC;
		writel(val, ap_sys_reg);
		udelay(1);
	}

	mutex_unlock(&cpufreq_lock);

	return 0;
}

static int light_cpufreq_init(struct cpufreq_policy *policy)
{
	policy->clk = clks[LIGHT_C910_CCLK].clk;
	policy->cur = clk_get_rate(policy->clk) / 1000;
	cpufreq_generic_init(policy, freq_table, transition_latency);
	policy->suspend_freq = max_freq;
	dev_pm_opp_of_register_em(cpu_dev, policy->cpus);

	return 0;
}

static int light_cpufreq_reboot_notifier(struct notifier_block *this,
					 unsigned long event, void *ptr)
{
	mutex_lock(&cpufreq_lock);
	cpufreq_denied = true;
	mutex_unlock(&cpufreq_lock);

	return NOTIFY_DONE;
}

static struct notifier_block cpufreq_reboot_notifier = {
	.notifier_call = light_cpufreq_reboot_notifier,
};

static struct cpufreq_driver light_cpufreq_driver = {
	.flags = CPUFREQ_NEED_INITIAL_FREQ_CHECK |
		 CPUFREQ_IS_COOLING_DEV,
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = light_set_target,
	.get = cpufreq_generic_get,
	.init = light_cpufreq_init,
	.name = "light-cpufreq",
	.attr = cpufreq_generic_attr,
	.suspend = cpufreq_generic_suspend,
};

static int light_cpufreq_pm_notify(struct notifier_block *nb,
	unsigned long event, void *dummy)
{
	switch (event) {
	case PM_SUSPEND_PREPARE:
		/* TBD */
		break;
	case PM_POST_SUSPEND:
		/* TBD */
		break;
	default:
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block light_cpufreq_pm_notifier = {
	.notifier_call = light_cpufreq_pm_notify,
};

/*
 * Set CPU PLL1's frequency as minimum on panic
 */
static int panic_cpufreq_notifier_call(struct notifier_block *nb,
				 unsigned long action, void *data)
{
	int cpu = smp_processor_id();
	struct cpufreq_policy *policy = cpufreq_cpu_get(cpu);
	u32 val = readl(ap_sys_reg);

	pr_info("enter panic_cpufreq_notifier_call\n");

	/*
	 * set CPU PLL1's frequency as minimum to compatible voltage
	 * becarefull if the PLL1 is serving the cpu core, swith to PLL0 first
	 */
	if (strcmp(__clk_get_name(clk_get_parent(clks[LIGHT_C910_CCLK].clk)),
		__clk_get_name(clks[LIGHT_C910_CCLK_I0].clk))) {
		pr_debug("[%s,%d]\n", __func__, __LINE__);
		clk_prepare_enable(clks[LIGHT_CPU_PLL0_FOUTPOSTDIV].clk);
		clk_set_rate(clks[LIGHT_CPU_PLL0_FOUTPOSTDIV].clk, policy->min * 1000);
		udelay(1);
		clk_set_parent(clks[LIGHT_C910_CCLK].clk, clks[LIGHT_C910_CCLK_I0].clk);

		pr_debug("[%s,%d]\n", __func__, __LINE__);
	}

	pr_debug("[%s,%d]\n", __func__, __LINE__);
	/*
	 * since the clk driver will use PLL1 as the default clock source,
	 * in order to compatible voltage which is unpredictable we should
	 * set the CPU PLL1's frequency as minimum in advance, otherwise the
	 * system may crash in crash kernel stage.
	 */
	clk_prepare_enable(clks[LIGHT_CPU_PLL1_FOUTPOSTDIV].clk);
	clk_set_rate(clks[LIGHT_CPU_PLL1_FOUTPOSTDIV].clk, policy->min * 1000);
	udelay(1);

	pr_info("finish to execute cpufreq notifier callback on panic\n");

	return 0;
}

static struct notifier_block panic_cpufreq_notifier = {
	.notifier_call = panic_cpufreq_notifier_call,
};

static int light_cpufreq_probe(struct platform_device *pdev)
{
	struct device_node *np;
	int num, ret;
	const struct property *prop;
	const __be32 *val;
	u32 nr, i, j;

        np = of_find_compatible_node(NULL, NULL, "thead,light_sys_reg");
	if (!np)
		return -ENOENT;
	ap_sys_reg = of_iomap(np, 0);
	WARN_ON(!ap_sys_reg);

	cpu_dev = get_cpu_device(0);
	if (!cpu_dev) {
		pr_err("failed to get cpu0 device\n");
		return -ENODEV;
	}

	np = of_node_get(cpu_dev->of_node);
	if (!np) {
		dev_err(cpu_dev, "failed to find cpu0 node\n");
		return -ENOENT;
	}

	num_clks = LIGHT_MPW_CPUFREQ_CLK_NUM;
	ret = clk_bulk_get(cpu_dev, num_clks, clks);
	if (ret)
		goto put_node;

	dvdd_cpu_reg = regulator_get(cpu_dev, "dvdd");
	dvddm_cpu_reg = regulator_get(cpu_dev, "dvddm");
	if (PTR_ERR(dvdd_cpu_reg) == -EPROBE_DEFER ||
	    PTR_ERR(dvddm_cpu_reg) == -EPROBE_DEFER) {
		ret = -EPROBE_DEFER;
		dev_dbg(cpu_dev, "regulators not ready, defer\n");
		goto put_reg;
	}

	if (IS_ERR(dvdd_cpu_reg) || IS_ERR(dvddm_cpu_reg)) {
		dev_err(cpu_dev, "failed to get regulators\n");
		ret = -ENOENT;
		goto put_reg;
	}

	ret = dev_pm_opp_of_add_table(cpu_dev);
	if (ret < 0) {
		dev_err(cpu_dev, "failed to init OPP table: %d\n", ret);
		goto put_reg;
	}

	num = dev_pm_opp_get_opp_count(cpu_dev);
	if (num < 0) {
		ret = num;
		dev_err(cpu_dev, "no OPP table is found: %d\n", ret);
		goto out_free_opp;
	}

	ret = dev_pm_opp_init_cpufreq_table(cpu_dev, &freq_table);
	if (ret) {
		dev_err(cpu_dev, "failed to init cpufreq table: %d\n", ret);
		goto out_free_opp;
	}

	/* Make light_dvddm_volt array's size same as dvdd opp number */
	light_dvddm_volt = devm_kcalloc(cpu_dev, num, sizeof(*light_dvddm_volt),
				     GFP_KERNEL);
	if (light_dvddm_volt == NULL) {
		ret = -ENOMEM;
		goto free_freq_table;
	}

	if (of_get_property(np, "dvfs_sv", NULL))
		light_dvfs_sv = true;
	else
		light_dvfs_sv = false;

	prop = of_find_property(np, "light,dvddm-operating-points", NULL);
	if (!prop || !prop->value)
		goto soc_opp_out;

	nr = prop->length / sizeof(u32);
	if (nr % 2 || (nr / 2) < num)
		goto soc_opp_out;

	for (j = 0; j < num; j++) {
		val = prop->value;
		for (i = 0; i < nr / 2; i++) {
			unsigned long freq = be32_to_cpup(val++);
			unsigned long volt = be32_to_cpup(val++);
			if (freq_table[j].frequency == freq) {
				light_dvddm_volt[soc_opp_count++] = volt;
				break;
			}
		}
	}

soc_opp_out:
	if (soc_opp_count != num)
		dev_warn(cpu_dev, "Not find valid light,dvddm-operating-points property\n");

	if (of_property_read_u32(np, "clock-latency", &transition_latency))
		transition_latency = CPUFREQ_ETERNAL;

	max_freq = freq_table[--num].frequency;

	ret = cpufreq_register_driver(&light_cpufreq_driver);
	if (ret) {
		dev_err(cpu_dev, "failed register driver: %d\n", ret);
		goto free_freq_table;
	}

	register_pm_notifier(&light_cpufreq_pm_notifier);

	of_node_put(np);

	ret = atomic_notifier_chain_register(&panic_notifier_list,
					     &panic_cpufreq_notifier);
	if (ret) {
		pr_err("unable to register notifier(%d)\n", ret);
		goto free_freq_table;
	}

	register_reboot_notifier(&cpufreq_reboot_notifier);

	dev_info(cpu_dev, "finish to register cpufreq driver\n");

	return 0;

free_freq_table:
	dev_pm_opp_free_cpufreq_table(cpu_dev, &freq_table);
out_free_opp:
	dev_pm_opp_of_remove_table(cpu_dev);
put_reg:
	if (!IS_ERR(dvdd_cpu_reg))
		regulator_put(dvdd_cpu_reg);
	if (!IS_ERR(dvddm_cpu_reg))
		regulator_put(dvddm_cpu_reg);

	clk_bulk_put(num_clks, clks);
put_node:
	of_node_put(np);

	return ret;
}

static int light_cpufreq_remove(struct platform_device *pdev)
{
	cpufreq_unregister_driver(&light_cpufreq_driver);
	dev_pm_opp_free_cpufreq_table(cpu_dev, &freq_table);
	dev_pm_opp_of_remove_table(cpu_dev);
	regulator_put(dvdd_cpu_reg);
	regulator_put(dvddm_cpu_reg);

	clk_bulk_put(num_clks, clks);

	return 0;
}

static const struct of_device_id light_cpufreq_match[] = {
	{ .compatible = "thead,light-mpw-cpufreq" },
	{},
};

static struct platform_driver light_cpufreq_platdrv = {
	.driver = {
		.name	= "thead,light-mpw-cpufreq",
		.of_match_table = light_cpufreq_match,
	},
	.probe		= light_cpufreq_probe,
	.remove		= light_cpufreq_remove,
};
module_platform_driver(light_cpufreq_platdrv);

MODULE_ALIAS("platform:light-cpufreq");
MODULE_AUTHOR("fugang.duan <duanfugang.dfg@linux.alibaba.com>");
MODULE_DESCRIPTION("Thead Light cpufreq driver");
MODULE_LICENSE("GPL v2");
