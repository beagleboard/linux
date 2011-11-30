/*
 * CPU idle for AM33XX SoCs
 *
 * Copyright (C) 2011 Texas Instruments Incorporated. http://www.ti.com/
 *
 * Derived from Davinci CPU idle code
 * (arch/arm/mach-davinci/cpuidle.c)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/cpuidle.h>
#include <linux/sched.h>
#include <asm/proc-fns.h>

#include <plat/emif.h>

#include "cpuidle33xx.h"

#define AM33XX_CPUIDLE_MAX_STATES	2

struct am33xx_ops {
	void (*enter) (u32 flags);
	void (*exit) (u32 flags);
	u32 flags;
};

/* fields in am33xx_ops.flags */
#define AM33XX_CPUIDLE_FLAGS_DDR2_PWDN	BIT(0)

static struct cpuidle_driver am33xx_idle_driver = {
	.name	= "cpuidle-am33xx",
	.owner	= THIS_MODULE,
};

static DEFINE_PER_CPU(struct cpuidle_device, am33xx_cpuidle_device);
static void __iomem *emif_base;

static void am33xx_save_ddr_power(int enter, bool pdown)
{
	u32 val;

	val = __raw_readl(emif_base + EMIF4_0_SDRAM_MGMT_CTRL);

	/* TODO: Choose the mode based on memory type */
	if (enter)
		val = SELF_REFRESH_ENABLE(64);
	else
		val = SELF_REFRESH_DISABLE;

	__raw_writel(val, emif_base + EMIF4_0_SDRAM_MGMT_CTRL);
}

static void am33xx_c2state_enter(u32 flags)
{
	am33xx_save_ddr_power(1, !!(flags & AM33XX_CPUIDLE_FLAGS_DDR2_PWDN));
}

static void am33xx_c2state_exit(u32 flags)
{
	am33xx_save_ddr_power(0, !!(flags & AM33XX_CPUIDLE_FLAGS_DDR2_PWDN));
}

static struct am33xx_ops am33xx_states[AM33XX_CPUIDLE_MAX_STATES] = {
	[1] = {
		.enter	= am33xx_c2state_enter,
		.exit	= am33xx_c2state_exit,
	},
};

/* Actual code that puts the SoC in different idle states */
static int am33xx_enter_idle(struct cpuidle_device *dev,
				struct cpuidle_driver *drv, int index)
{
	struct cpuidle_state_usage *state_usage = &dev->states_usage[index];
	struct am33xx_ops *ops = cpuidle_get_statedata(state_usage);
	struct timeval before, after;
	int idle_time;

	local_irq_disable();
	do_gettimeofday(&before);

	if (ops && ops->enter)
		ops->enter(ops->flags);

	/* Wait for interrupt state */
	cpu_do_idle();
	if (ops && ops->exit)
		ops->exit(ops->flags);

	do_gettimeofday(&after);
	local_irq_enable();
	idle_time = (after.tv_sec - before.tv_sec) * USEC_PER_SEC +
			(after.tv_usec - before.tv_usec);

	dev->last_residency = idle_time;

	return index;
}

static int __init am33xx_cpuidle_probe(struct platform_device *pdev)
{
	int ret;
	struct cpuidle_device *device;
	struct cpuidle_driver *driver = &am33xx_idle_driver;
	struct am33xx_cpuidle_config *pdata = pdev->dev.platform_data;

	device = &per_cpu(am33xx_cpuidle_device, smp_processor_id());

	if (!pdata) {
		dev_err(&pdev->dev, "cannot get platform data\n");
		return -ENOENT;
	}

	emif_base = pdata->emif_base;

	/* Wait for interrupt state */
	driver->states[0].enter = am33xx_enter_idle;
	driver->states[0].exit_latency = 1;
	driver->states[0].target_residency = 10000;
	driver->states[0].flags = CPUIDLE_FLAG_TIME_VALID;
	strcpy(driver->states[0].name, "WFI");
	strcpy(driver->states[0].desc, "Wait for interrupt");

	/* Wait for interrupt and DDR self refresh state */
	driver->states[1].enter = am33xx_enter_idle;
	driver->states[1].exit_latency = 100;
	driver->states[1].target_residency = 10000;
	driver->states[1].flags = CPUIDLE_FLAG_TIME_VALID;
	strcpy(driver->states[1].name, "DDR SR");
	strcpy(driver->states[1].desc, "WFI and DDR Self Refresh");
	if (pdata->ddr2_pdown)
		am33xx_states[1].flags |= AM33XX_CPUIDLE_FLAGS_DDR2_PWDN;
	cpuidle_set_statedata(&device->states_usage[1], &am33xx_states[1]);

	device->state_count = AM33XX_CPUIDLE_MAX_STATES;
	driver->state_count = AM33XX_CPUIDLE_MAX_STATES;

	ret = cpuidle_register_driver(&am33xx_idle_driver);
	if (ret) {
		dev_err(&pdev->dev, "failed to register driver\n");
		return ret;
	}

	ret = cpuidle_register_device(device);
	if (ret) {
		dev_err(&pdev->dev, "failed to register device\n");
		cpuidle_unregister_driver(&am33xx_idle_driver);
		return ret;
	}

	return 0;
}

static struct platform_driver am33xx_cpuidle_driver = {
	.driver = {
		.name	= "cpuidle-am33xx",
		.owner	= THIS_MODULE,
	},
};

static int __init am33xx_cpuidle_init(void)
{
	return platform_driver_probe(&am33xx_cpuidle_driver,
						am33xx_cpuidle_probe);
}
device_initcall(am33xx_cpuidle_init);
