/*
 *  CPU frequency scaling for OMAP using OPP information
 *
 *  Copyright (C) 2005 Nokia Corporation
 *  Written by Tony Lindgren <tony@atomide.com>
 *
 *  Based on cpu-sa1110.c, Copyright (C) 2001 Russell King
 *
 * Copyright (C) 2007-2011 Texas Instruments, Inc.
 * - OMAP3/4 support by Rajendra Nayak, Santosh Shilimkar
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/opp.h>
#include <linux/cpu.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/suspend.h>

#include <asm/system.h>
#include <asm/smp_plat.h>
#include <asm/cpu.h>

#include <plat/clock.h>
#include <plat/omap-pm.h>
#include <plat/common.h>
#include <plat/omap_device.h>

#include <mach/hardware.h>

/* Tolerance for MPU voltage is 4%, we have to pass +4% as a
 * maximum voltage while setting the MPU regulator voltage.
 * Which is taken from AM33XX datasheet */
#define MPU_TOLERANCE	4
#define PER_ROUND_VAL	100

/* Use 275MHz when entering suspend */
#define SLEEP_FREQ	(275 * 1000)


#ifdef CONFIG_SMP
struct lpj_info {
	unsigned long	ref;
	unsigned int	freq;
};

static DEFINE_PER_CPU(struct lpj_info, lpj_ref);
static struct lpj_info global_lpj_ref;
#endif

static struct cpufreq_frequency_table *freq_table;
static atomic_t freq_table_users = ATOMIC_INIT(0);
static struct clk *mpu_clk;
static char *mpu_clk_name;
static struct device *mpu_dev;
static struct regulator *mpu_reg;
static DEFINE_MUTEX(omap_cpu_lock);
static bool is_suspended;

static int omap_verify_speed(struct cpufreq_policy *policy)
{
	if (!freq_table)
		return -EINVAL;
	return cpufreq_frequency_table_verify(policy, freq_table);
}

static unsigned int omap_getspeed(unsigned int cpu)
{
	unsigned long rate;

	if (cpu >= NR_CPUS)
		return 0;

	rate = clk_get_rate(mpu_clk) / 1000;
	return rate;
}

static int omap_target(struct cpufreq_policy *policy,
		       unsigned int target_freq,
		       unsigned int relation)
{
	unsigned int i;
	int ret = 0;
	struct cpufreq_freqs freqs;
	struct opp *opp;
	int volt_old = 0, volt_new = 0;

	if (is_suspended)
		return -EBUSY;

	if (!freq_table) {
		dev_err(mpu_dev, "%s: cpu%d: no freq table!\n", __func__,
				policy->cpu);
		return -EINVAL;
	}

	ret = cpufreq_frequency_table_target(policy, freq_table, target_freq,
			relation, &i);
	if (ret) {
		dev_dbg(mpu_dev, "%s: cpu%d: no freq match for %d(ret=%d)\n",
			__func__, policy->cpu, target_freq, ret);
		return ret;
	}
	freqs.new = freq_table[i].frequency;
	if (!freqs.new) {
		dev_err(mpu_dev, "%s: cpu%d: no match for freq %d\n", __func__,
			policy->cpu, target_freq);
		return -EINVAL;
	}

	freqs.old = omap_getspeed(policy->cpu);
	freqs.cpu = policy->cpu;

	if (freqs.old == freqs.new && policy->cur == freqs.new)
		return ret;

	opp = opp_find_freq_exact(mpu_dev, freqs.new * 1000, true);
	if (IS_ERR(opp)) {
		dev_err(mpu_dev, "%s: cpu%d: no opp match for freq %d\n",
			__func__, policy->cpu, target_freq);
		return -EINVAL;
	}

	volt_new = opp_get_voltage(opp);
	if (!volt_new) {
		dev_err(mpu_dev, "%s: cpu%d: no opp voltage for freq %d\n",
			__func__, policy->cpu, target_freq);
		return -EINVAL;
	}

	volt_old = regulator_get_voltage(mpu_reg);

#ifdef CONFIG_CPU_FREQ_DEBUG
	pr_info("cpufreq-omap: frequency transition: %u --> %u\n",
			freqs.old, freqs.new);
	pr_info("cpufreq-omap: voltage transition: %d --> %d\n",
			volt_old, volt_new);
#endif

	if (freqs.new > freqs.old) {
		ret = regulator_set_voltage(mpu_reg, volt_new,
			volt_new + (volt_new * MPU_TOLERANCE) / PER_ROUND_VAL);
		if (ret) {
			dev_err(mpu_dev, "%s: unable to set voltage to %d uV (for %u MHz)\n",
				__func__, volt_new, freqs.new/1000);
			return ret;
		}
	}

	/* notifiers */
	for_each_cpu(i, policy->cpus) {
		freqs.cpu = i;
		cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);
	}

	ret = clk_set_rate(mpu_clk, freqs.new * 1000);
	freqs.new = omap_getspeed(policy->cpu);

#ifdef CONFIG_SMP
	/*
	 * Note that loops_per_jiffy is not updated on SMP systems in
	 * cpufreq driver. So, update the per-CPU loops_per_jiffy value
	 * on frequency transition. We need to update all dependent CPUs.
	 */
	for_each_cpu(i, policy->cpus) {
		struct lpj_info *lpj = &per_cpu(lpj_ref, i);
		if (!lpj->freq) {
			lpj->ref = per_cpu(cpu_data, i).loops_per_jiffy;
			lpj->freq = freqs.old;
		}

		per_cpu(cpu_data, i).loops_per_jiffy =
			cpufreq_scale(lpj->ref, lpj->freq, freqs.new);
	}

	/* And don't forget to adjust the global one */
	if (!global_lpj_ref.freq) {
		global_lpj_ref.ref = loops_per_jiffy;
		global_lpj_ref.freq = freqs.old;
	}
	loops_per_jiffy = cpufreq_scale(global_lpj_ref.ref, global_lpj_ref.freq,
					freqs.new);
#endif

	/* notifiers */
	for_each_cpu(i, policy->cpus) {
		freqs.cpu = i;
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);
	}

	if (freqs.new < freqs.old) {
		ret = regulator_set_voltage(mpu_reg, volt_new,
			volt_new + (volt_new * MPU_TOLERANCE) / PER_ROUND_VAL);
		if (ret) {
			unsigned int temp;

			dev_err(mpu_dev, "%s: unable to set voltage to %d uV (for %u MHz)\n",
				__func__, volt_new, freqs.new/1000);

			if (clk_set_rate(mpu_clk, freqs.old * 1000)) {
				dev_err(mpu_dev,
					"%s: failed restoring clock rate to %u MHz, clock rate is %u MHz",
					__func__,
					freqs.old/1000, freqs.new/1000);
				return ret;
			}

			temp = freqs.new;
			freqs.new = freqs.old;
			freqs.old = temp;

			for_each_cpu(i, policy->cpus) {
				freqs.cpu = i;
				cpufreq_notify_transition(&freqs,
					CPUFREQ_PRECHANGE);
				cpufreq_notify_transition(&freqs,
					CPUFREQ_POSTCHANGE);
			}
			return ret;
		}
	}

	return ret;
}

static inline void freq_table_free(void)
{
	if (atomic_dec_and_test(&freq_table_users))
		opp_free_cpufreq_table(mpu_dev, &freq_table);
}

static int omap_pm_notify(struct notifier_block *nb, unsigned long event,
                                void *dummy)
{
	struct cpufreq_policy *policy = cpufreq_cpu_get(0);
	static unsigned int saved_frequency;

	mutex_lock(&omap_cpu_lock);
	switch (event) {
	case PM_SUSPEND_PREPARE:
		if (is_suspended)
			goto out;

                saved_frequency = omap_getspeed(0);

                mutex_unlock(&omap_cpu_lock);
		omap_target(policy, SLEEP_FREQ, CPUFREQ_RELATION_H);
		mutex_lock(&omap_cpu_lock);
		is_suspended = true;
                break;

        case PM_POST_SUSPEND:
                is_suspended = false;
                mutex_unlock(&omap_cpu_lock);
		omap_target(policy, saved_frequency, CPUFREQ_RELATION_H);
		mutex_lock(&omap_cpu_lock);
                break;
	}
out:
	mutex_unlock(&omap_cpu_lock);

	return NOTIFY_OK;
}

static struct notifier_block omap_cpu_pm_notifier = {
	.notifier_call = omap_pm_notify,
};

static int __cpuinit omap_cpu_init(struct cpufreq_policy *policy)
{
	int result = 0;

	mpu_clk = clk_get(NULL, mpu_clk_name);
	if (IS_ERR(mpu_clk))
		return PTR_ERR(mpu_clk);

	mpu_reg = regulator_get(NULL, "vdd_mpu");
	if (IS_ERR(mpu_reg)) {
		result = -EINVAL;
		goto fail_ck;
	}

	/* success of regulator_get doesn't gurantee presence of driver for
	   physical regulator and presence of physical regulator (this
	   situation arises if dummy regulator is enabled),so check voltage
	   to verify that physical regulator and it's driver is present
	 */
	if (regulator_get_voltage(mpu_reg) < 0) {
		result = -EINVAL;
		goto fail_reg;
	}

	if (policy->cpu >= NR_CPUS) {
		result = -EINVAL;
		goto fail_reg;
	}

	policy->cur = policy->min = policy->max = omap_getspeed(policy->cpu);

	if (atomic_inc_return(&freq_table_users) == 1)
		result = opp_init_cpufreq_table(mpu_dev, &freq_table);

	if (result) {
		dev_err(mpu_dev, "%s: cpu%d: failed creating freq table[%d]\n",
				__func__, policy->cpu, result);
		goto fail_reg;
	}

	result = cpufreq_frequency_table_cpuinfo(policy, freq_table);
	if (result)
		goto fail_table;

	cpufreq_frequency_table_get_attr(freq_table, policy->cpu);

	policy->min = policy->cpuinfo.min_freq;
	policy->max = policy->cpuinfo.max_freq;
	policy->cur = omap_getspeed(policy->cpu);

	/*
	 * On OMAP SMP configuartion, both processors share the voltage
	 * and clock. So both CPUs needs to be scaled together and hence
	 * needs software co-ordination. Use cpufreq affected_cpus
	 * interface to handle this scenario. Additional is_smp() check
	 * is to keep SMP_ON_UP build working.
	 */
	if (is_smp()) {
		policy->shared_type = CPUFREQ_SHARED_TYPE_ANY;
		cpumask_setall(policy->cpus);
	}

	/* FIXME: what's the actual transition time? */
	policy->cpuinfo.transition_latency = 300 * 1000;

	register_pm_notifier(&omap_cpu_pm_notifier);

	return 0;

fail_table:
	freq_table_free();
fail_reg:
	regulator_put(mpu_reg);
fail_ck:
	clk_put(mpu_clk);
	return result;
}

static int omap_cpu_exit(struct cpufreq_policy *policy)
{
	freq_table_free();
	clk_put(mpu_clk);
	return 0;
}

static struct freq_attr *omap_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver omap_driver = {
	.flags		= CPUFREQ_STICKY,
	.verify		= omap_verify_speed,
	.target		= omap_target,
	.get		= omap_getspeed,
	.init		= omap_cpu_init,
	.exit		= omap_cpu_exit,
	.name		= "omap",
	.attr		= omap_cpufreq_attr,
};

static int __init omap_cpufreq_init(void)
{
	if (cpu_is_omap24xx())
		mpu_clk_name = "virt_prcm_set";
	else if (cpu_is_omap34xx() && !cpu_is_am33xx())
		mpu_clk_name = "dpll1_ck";
	else if (cpu_is_omap44xx() || cpu_is_am33xx())
		mpu_clk_name = "dpll_mpu_ck";

	if (!mpu_clk_name) {
		pr_err("%s: unsupported Silicon?\n", __func__);
		return -EINVAL;
	}

	mpu_dev = omap_device_get_by_hwmod_name("mpu");
	if (!mpu_dev) {
		pr_warning("%s: unable to get the mpu device\n", __func__);
		return -EINVAL;
	}

	return cpufreq_register_driver(&omap_driver);
}

static void __exit omap_cpufreq_exit(void)
{
	cpufreq_unregister_driver(&omap_driver);
}

MODULE_DESCRIPTION("cpufreq driver for OMAP SoCs");
MODULE_LICENSE("GPL");
module_init(omap_cpufreq_init);
module_exit(omap_cpufreq_exit);
