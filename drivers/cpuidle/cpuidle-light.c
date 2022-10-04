/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2021 Alibaba Group Holding Limited.
 * CPU idle support for Thead LIGHT SoC
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/cpuidle.h>
#include <linux/io.h>
#include <linux/export.h>
#include <asm/cpuidle.h>

#define LIGHT_MAX_STATES	1

extern void arch_cpu_idle(void);

/* Actual code that puts the SoC in different idle states */
static int light_enter_idle(struct cpuidle_device *dev,
			struct cpuidle_driver *drv,
			       int index)
{
	arch_cpu_idle();
	return index;
}

static struct cpuidle_driver light_idle_driver = {
	.name			= "light_idle",
	.owner			= THIS_MODULE,
	.states[0]		= {
		.enter			= light_enter_idle,
		.exit_latency		= 1,
		.power_usage		= UINT_MAX,
		.target_residency	= 1,
		.name			= "WFI",
		.desc			= "RISC-V WFI",
	},
	.state_count = LIGHT_MAX_STATES,
};

/* Initialize CPU idle by registering the idle states */
static int light_cpuidle_probe(struct platform_device *dev)
{
	return cpuidle_register(&light_idle_driver, NULL);
}

static struct platform_driver light_cpuidle_driver = {
	.driver = {
		.name = "cpuidle-light",
	},
	.probe = light_cpuidle_probe,
};

static int __init light_cpuidle_init(void)
{
	int ret;
	struct platform_device *pdev;

	ret = platform_driver_register(&light_cpuidle_driver);
	if (ret)
		return ret;

	pdev = platform_device_register_simple("cpuidle-light",
						-1, NULL, 0);
	if (IS_ERR(pdev)) {
		platform_driver_unregister(&light_cpuidle_driver);
		return PTR_ERR(pdev);
	}

	return 0;
}
device_initcall(light_cpuidle_init);
