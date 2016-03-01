/*
 * AM43XX CPU idle Routines
 *
 * Copyright (C) 2014 Texas Instruments, Inc.
 * Russ Dill <russ.dill@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/cpuidle.h>
#include <linux/io.h>
#include <asm/smp_scu.h>
#include <asm/cpuidle.h>

static void __iomem *scu_base;

static int am43xx_enter_idle(struct cpuidle_device *dev,
			     struct cpuidle_driver *drv, int index)
{
	scu_power_mode(scu_base, SCU_PM_DORMANT);
	wfi();
	scu_power_mode(scu_base, SCU_PM_NORMAL);

	return index;
}

static struct cpuidle_driver am437x_idle_driver = {
	.name		= "am437x_idle",
	.states		= {
		ARM_CPUIDLE_WFI_STATE,
		{
			.exit_latency = 100,
			.target_residency = 200,
			.power_usage = 500,
			.enter = am43xx_enter_idle,
			.name = "C1",
			.desc = "Bypass MPU PLL",
			.flags = CPUIDLE_FLAG_TIMER_STOP,
		},
	},
	.state_count	= 2,
};

int am437x_idle_init(void)
{
	scu_base = ioremap(scu_a9_get_base(), SZ_256);
	if (!scu_base)
		return -ENOMEM;

	return cpuidle_register(&am437x_idle_driver, NULL);
}
