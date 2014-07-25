/*
 * AM33XX CPU idle Routines
 *
 * Copyright (C) 2011-2013 Texas Instruments, Inc.
 * Santosh Shilimkar <santosh.shilimkar@ti.com>
 * Rajendra Nayak <rnayak@ti.com>
 * Russ Dill <russ.dill@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/sched.h>
#include <linux/cpuidle.h>
#include <linux/cpu_pm.h>

#include "common.h"
#include "pm33xx.h"
#include "powerdomain.h"

#define AM33XX_FLAG_MPU_PLL		BIT(16)
#define AM33XX_FLAG_SELF_REFRESH	BIT(17)
#define AM33XX_FLAG_DISABLE_EMIF	BIT(18)

static int am33xx_enter_idle(struct cpuidle_device *dev,
			     struct cpuidle_driver *drv, int index)
{
	struct cpuidle_state *state;
	u32 wfi_flags = 0;
	u32 m3_flags = 0;

	if (omap_irq_pending() || need_resched())
		return index;

	state = &drv->states[index];

	if (state->flags & AM33XX_FLAG_SELF_REFRESH)
		wfi_flags |= WFI_SELF_REFRESH;

	if (state->flags & AM33XX_FLAG_DISABLE_EMIF)
		wfi_flags |= WFI_DISABLE_EMIF;

	if (state->flags & AM33XX_FLAG_MPU_PLL) {
		wfi_flags |= WFI_WAKE_M3;
		m3_flags = PWRDM_POWER_ON << M3_PARAM2_MPU_STATE_SHIFT |
			MEM_BANK_RET_ST_OFF << M3_PARAM2_MPU_RAM_RET_SHIFT |
			MEM_BANK_RET_ST_RET << M3_PARAM2_MPU_L1_RET_SHIFT |
			MEM_BANK_RET_ST_RET << M3_PARAM2_MPU_L2_RET_SHIFT |
			PWRDM_POWER_ON << M3_PARAM2_PER_STATE_SHIFT |
			MPU_WAKE << M3_PARAM2_WAKE_SOURCES_SHIFT;
	}

	am33xx_do_sram_cpuidle(wfi_flags, m3_flags);

	return index;
}

/* Power usage measured as a combination of CPU and DDR power rails */
struct cpuidle_state am33xx_ddr2_states[] = {
	{
		.exit_latency = 72,
		.target_residency = 150,
		.power_usage = 625,
		.flags = CPUIDLE_FLAG_TIME_VALID,
		.enter = am33xx_enter_idle,
		.name = "C0",
		.desc = "WFI",
	},
	{
		.exit_latency = 176,
		.target_residency = 300,
		.power_usage = 562,
		.flags = CPUIDLE_FLAG_TIME_VALID | AM33XX_FLAG_MPU_PLL,
		.enter = am33xx_enter_idle,
		.name = "C1",
		.desc = "Bypass MPU PLL",
	},
	{
		.exit_latency = 390,
		.target_residency = 500,
		.power_usage = 529,
		.flags = CPUIDLE_FLAG_TIME_VALID | AM33XX_FLAG_MPU_PLL |
				AM33XX_FLAG_SELF_REFRESH,
		.enter = am33xx_enter_idle,
		.name = "C1+SR",
		.desc = "Bypass MPU PLL + DDR SR",
	},
};

struct cpuidle_state am33xx_ddr3_states[] = {
	{
		.exit_latency = 68,
		.target_residency = 150,
		.power_usage = 557,
		.flags = CPUIDLE_FLAG_TIME_VALID,
		.enter = am33xx_enter_idle,
		.name = "C0",
		.desc = "WFI",
	},
	{
		.exit_latency = 130,
		.target_residency = 200,
		.power_usage = 497,
		.flags = CPUIDLE_FLAG_TIME_VALID | AM33XX_FLAG_MPU_PLL,
		.enter = am33xx_enter_idle,
		.name = "C1",
		.desc = "Bypass MPU PLL",
	},
};

static struct cpuidle_driver am33xx_idle_driver = {
	.name		= "am33xx_idle",
	.owner		= THIS_MODULE,
};

/**
 * am33xx_idle_init - Init routine for am33xx idle
 *
 * Registers the am33xx specific cpuidle driver to the cpuidle
 * framework with the valid set of states.
 */
int am33xx_idle_init(bool ddr3)
{
	if (ddr3) {
		BUILD_BUG_ON(ARRAY_SIZE(am33xx_ddr3_states) >
					ARRAY_SIZE(am33xx_idle_driver.states));
		memcpy(am33xx_idle_driver.states, am33xx_ddr3_states,
		       sizeof(am33xx_ddr3_states));
		am33xx_idle_driver.state_count =
						ARRAY_SIZE(am33xx_ddr3_states);
	} else {
		BUILD_BUG_ON(ARRAY_SIZE(am33xx_ddr2_states) >
					ARRAY_SIZE(am33xx_idle_driver.states));
		memcpy(am33xx_idle_driver.states, am33xx_ddr2_states,
		       sizeof(am33xx_ddr2_states));
		am33xx_idle_driver.state_count =
						ARRAY_SIZE(am33xx_ddr2_states);
	}
	return cpuidle_register(&am33xx_idle_driver, NULL);
}
