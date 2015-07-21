/*
 * AM33XX Power Management Routines
 *
 * Copyright (C) 2012-2014 Texas Instruments Incorporated - http://www.ti.com/
 *	Vaibhav Bedia, Dave Gerlach
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

#include <linux/cpu.h>
#include <linux/err.h>
#include <linux/genalloc.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/sizes.h>
#include <linux/suspend.h>
#include <linux/ti-emif-sram.h>
#include <linux/wkup_m3_ipc.h>

#include <asm/fncpy.h>
#include <asm/proc-fns.h>
#include <asm/suspend.h>
#include <asm/system_misc.h>

#include "pm.h"

static int (*am33xx_do_wfi_sram)(unsigned long unused);
static phys_addr_t am33xx_do_wfi_sram_phys;

static struct gen_pool *sram_pool;
static phys_addr_t ocmcram_location;

static struct am33xx_pm_ops *pm_ops;
static struct am33xx_pm_sram_addr *pm_sram;
static unsigned long suspend_wfi_flags;

static void am33xx_do_sram_idle(u32 wfi_flags)
{
	int ret = 0;

	ret = wkup_m3_prepare_low_power(WKUP_M3_IDLE);

	if (!ret)
		pm_ops->cpu_suspend(am33xx_do_wfi_sram, wfi_flags);
}

#ifdef CONFIG_SUSPEND
static int am33xx_pm_suspend(suspend_state_t suspend_state)
{
	int i, ret = 0;

	ret = pm_ops->soc_suspend(suspend_state, am33xx_do_wfi_sram,
				  suspend_wfi_flags);

	if (ret) {
		pr_err("PM: Kernel suspend failure\n");
	} else {
		i = wkup_m3_request_pm_status();

		switch (i) {
		case 0:
			pr_info("PM: Successfully put all powerdomains to target state\n");
			break;
		case 1:
			pr_err("PM: Could not transition all powerdomains to target state\n");
			ret = -1;
			break;
		default:
			pr_err("PM: CM3 returned unknown result = %d\n", i);
			ret = -1;
		}

		pr_info("PM: Wakeup source %s\n", wkup_m3_request_wake_src());
	}

	return ret;
}

static int am33xx_pm_enter(suspend_state_t suspend_state)
{
	int ret = 0;

	switch (suspend_state) {
	case PM_SUSPEND_MEM:
	case PM_SUSPEND_STANDBY:
		ret = am33xx_pm_suspend(suspend_state);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int am33xx_pm_begin(suspend_state_t state)
{
	int ret = -EINVAL;

	cpu_idle_poll_ctrl(true);

	switch (state) {
	case PM_SUSPEND_MEM:
	case PM_SUSPEND_STANDBY:
		ret = wkup_m3_prepare_low_power(state);
		break;
	}

	return ret;
}

static void am33xx_pm_end(void)
{
	wkup_m3_finish_low_power();
	cpu_idle_poll_ctrl(false);
}

static int am33xx_pm_valid(suspend_state_t state)
{
	switch (state) {
	case PM_SUSPEND_STANDBY:
	case PM_SUSPEND_MEM:
		return 1;
	default:
		return 0;
	}
}

static const struct platform_suspend_ops am33xx_pm_ops = {
	.begin		= am33xx_pm_begin,
	.end		= am33xx_pm_end,
	.enter		= am33xx_pm_enter,
	.valid		= am33xx_pm_valid,
};
#endif /* CONFIG_SUSPEND */

static void am33xx_pm_set_ipc_ops(void)
{
	void *resume_address;
	u32 temp;

	temp = ti_emif_get_mem_type();
	if (temp < 0) {
		pr_err("PM: Cannot determine memory type, no PM available\n");
		return;
	}
	wkup_m3_set_mem_type(temp);

	/* Physical resume address to be used by ROM code */
	resume_address = (void *)am33xx_do_wfi_sram_phys +
			 *pm_sram->resume_offset + 0x4;

	wkup_m3_set_resume_address(resume_address);
}

/*
 * Push the minimal suspend-resume code to SRAM
 */
static int am33xx_push_sram_idle(void)
{
	struct device_node *np;
	int ret;

	ret = ti_emif_copy_pm_function_table(pm_sram->emif_sram_table);
	if (ret) {
		pr_err("PM: %s: EMIF function copy failed\n", __func__);
		return -EPROBE_DEFER;
	}

	np = of_find_compatible_node(NULL, NULL, "ti,omap3-mpu");

	if (!np) {
		np = of_find_compatible_node(NULL, NULL, "ti,omap4-mpu");
		if (!np) {
			pr_warn("PM: %s: Unable to find device node for mpu\n",
				__func__);
			return -ENODEV;
		}
	}

	sram_pool = of_get_named_gen_pool(np, "sram", 0);

	if (!sram_pool) {
		pr_warn("PM: %s: Unable to get sram pool for ocmcram\n",
			__func__);
		return -ENODEV;
	}

	ocmcram_location = gen_pool_alloc(sram_pool, *pm_sram->do_wfi_sz);
	if (!ocmcram_location) {
		pr_warn("PM: %s: Unable to allocate memory from ocmcram\n",
			__func__);
		return -EINVAL;
	}

	/* Save physical address to calculate resume offset during pm init */
	am33xx_do_wfi_sram_phys = gen_pool_virt_to_phys(sram_pool,
							ocmcram_location);
	am33xx_do_wfi_sram = (void *)fncpy((void *)ocmcram_location,
					   pm_sram->do_wfi,
					   *pm_sram->do_wfi_sz);

	return 0;
}

int am33xx_pm_init(void)
{
	int ret;

	if (!of_machine_is_compatible("ti,am33xx") &&
	    !of_machine_is_compatible("ti,am43"))
		return -ENODEV;

	pm_sram = amx3_get_sram_addrs();
	if (!pm_sram) {
		pr_err("PM: Cannot get PM asm function addresses!!\n");
		return -ENODEV;
	}

	ret = am33xx_push_sram_idle();
	if (ret)
		return ret;

	am33xx_pm_set_ipc_ops();

#ifdef CONFIG_SUSPEND
	suspend_set_ops(&am33xx_pm_ops);
#endif /* CONFIG_SUSPEND */

	pm_ops = amx3_get_pm_ops();
	if (!pm_ops) {
		pr_err("PM: Cannot get core PM ops!\n");
		return -ENODEV;
	}

	suspend_wfi_flags = 0;
	suspend_wfi_flags |= WFI_FLAG_SELF_REFRESH;
	suspend_wfi_flags |= WFI_FLAG_SAVE_EMIF;
	suspend_wfi_flags |= WFI_FLAG_DISABLE_EMIF;
	suspend_wfi_flags |= WFI_FLAG_WAKE_M3;

	ret = pm_ops->init(am33xx_do_sram_idle);
	if (ret) {
		pr_err("Unable to call core pm init!\n");
		return -ENODEV;
	}

	return 0;
}

void am33xx_pm_exit(void)
{
	suspend_set_ops(NULL);
	gen_pool_free(sram_pool, ocmcram_location, *pm_sram->do_wfi_sz);
}

late_initcall(am33xx_pm_init);
module_exit(am33xx_pm_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("am33xx power management driver");
