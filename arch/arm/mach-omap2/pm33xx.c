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
#include <linux/sizes.h>
#include <linux/suspend.h>
#include <linux/ti-emif-sram.h>

#include <linux/platform_data/wkup_m3_ipc.h>

#include <asm/fncpy.h>
#include <asm/proc-fns.h>
#include <asm/suspend.h>
#include <asm/system_misc.h>

#include "clockdomain.h"
#include "cm33xx.h"
#include "common.h"
#include "pm.h"
#include "powerdomain.h"
#include "soc.h"

static struct powerdomain *cefuse_pwrdm, *gfx_pwrdm, *per_pwrdm, *mpu_pwrdm;
static struct clockdomain *gfx_l4ls_clkdm;

static int (*am33xx_do_wfi_sram)(unsigned long unused);
static phys_addr_t am33xx_do_wfi_sram_phys;

static struct wkup_m3_pm_ipc_ops *m3_ops;

#ifdef CONFIG_SUSPEND
static int am33xx_pm_suspend(void)
{
	int i, ret = 0;
	int status = 0;

	/* Try to put GFX to sleep */
	omap_set_pwrdm_state(gfx_pwrdm, PWRDM_POWER_OFF);

	ret = cpu_suspend(0, am33xx_do_wfi_sram);

	status = pwrdm_read_pwrst(gfx_pwrdm);
	if (status != PWRDM_POWER_OFF)
		pr_err("PM: GFX domain did not transition\n");

	/*
	 * BUG: GFX_L4LS clock domain needs to be woken up to
	 * ensure thet L4LS clock domain does not get stuck in transition
	 * If that happens L3 module does not get disabled, thereby leading
	 * to PER power domain transition failing
	 */
	clkdm_wakeup(gfx_l4ls_clkdm);
	clkdm_sleep(gfx_l4ls_clkdm);

	if (ret) {
		pr_err("PM: Kernel suspend failure\n");
	} else {
		i = m3_ops->request_pm_status();

		switch (i) {
		case 0:
			pr_info("PM: Successfully put all powerdomains to target state\n");

			/*
			 * The PRCM registers on AM335x do not contain
			 * previous state information like those present on
			 * OMAP4 so we must manually indicate transition so
			 * state counters are properly incremented
			 */
			pwrdm_post_transition(mpu_pwrdm);
			pwrdm_post_transition(per_pwrdm);
			break;
		case 1:
			pr_err("PM: Could not transition all powerdomains to target state\n");
			ret = -1;
			break;
		default:
			pr_err("PM: CM3 returned unknown result = %d\n", i);
			ret = -1;
		}
	}

	return ret;
}

static int am33xx_pm_enter(suspend_state_t suspend_state)
{
	int ret = 0;

	switch (suspend_state) {
	case PM_SUSPEND_MEM:
		ret = am33xx_pm_suspend();
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int am33xx_pm_begin(suspend_state_t state)
{
	int ret = -EINVAL;

	if (!m3_ops) {
		pr_err("PM: No wkup_m3 ops passed, no PM available.\n");
		return ret;
	}

	switch (state) {
	case PM_SUSPEND_MEM:
		ret = m3_ops->prepare_low_power(state);
		break;
	}

	return ret;
}

static void am33xx_pm_end(void)
{
	if (m3_ops)
		m3_ops->finish_low_power();
}

static int am33xx_pm_valid(suspend_state_t state)
{
	switch (state) {
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

void am33xx_pm_set_ipc_ops(struct wkup_m3_pm_ipc_ops *ops)
{
	void *resume_address;
	u32 temp;

	m3_ops = ops;

	if (!m3_ops)
		return;

	temp = ti_emif_get_mem_type();
	if (temp < 0) {
		pr_err("PM: Cannot determine memory type, no PM available\n");
		m3_ops = NULL;
		return;
	}
	m3_ops->set_mem_type(temp);

	/* Physical resume address to be used by ROM code */
	resume_address = (void *)am33xx_do_wfi_sram_phys +
			 am33xx_resume_offset + 0x4;

	m3_ops->set_resume_address(resume_address);
}

/*
 * Push the minimal suspend-resume code to SRAM
 */
static int am33xx_push_sram_idle(void)
{
	struct device_node *np;
	struct gen_pool *sram_pool;
	phys_addr_t ocmcram_location;
	int ret;

	ret = ti_emif_copy_pm_function_table(&am33xx_emif_sram_table);
	if (ret) {
		pr_err("PM: Cannot copy emif functions to sram, no PM available\n");
		return -ENODEV;
	}

	np = of_find_compatible_node(NULL, NULL, "ti,omap3-mpu");

	if (!np) {
		pr_warn("PM: %s: Unable to find device node for mpu\n",
			__func__);
		return -ENODEV;
	}

	sram_pool = of_get_named_gen_pool(np, "sram", 0);

	if (!sram_pool) {
		pr_warn("PM: %s: Unable to allocate sram pool for ocmcram\n",
			__func__);
		return -ENODEV;
	}

	ocmcram_location = gen_pool_alloc(sram_pool, am33xx_do_wfi_sz);
	if (!ocmcram_location)
		return -EINVAL;

	/* Save physical address to calculate resume offset during pm init */
	am33xx_do_wfi_sram_phys = gen_pool_virt_to_phys(sram_pool,
							ocmcram_location);
	am33xx_do_wfi_sram = (void *)fncpy((void *)ocmcram_location,
					   &am33xx_do_wfi,
					   am33xx_do_wfi_sz);

	return 0;
}

int __init am33xx_pm_init(void)
{
	int ret;

	if (!soc_is_am33xx())
		return -ENODEV;

	gfx_pwrdm = pwrdm_lookup("gfx_pwrdm");
	per_pwrdm = pwrdm_lookup("per_pwrdm");
	mpu_pwrdm = pwrdm_lookup("mpu_pwrdm");

	gfx_l4ls_clkdm = clkdm_lookup("gfx_l4ls_gfx_clkdm");

	if ((!gfx_pwrdm) || (!per_pwrdm) || (!mpu_pwrdm) || (!gfx_l4ls_clkdm)) {
		pr_err("PM: Could not lookup clock and power domains\n");
		return -ENODEV;
	}

	(void)clkdm_for_each(omap_pm_clkdms_setup, NULL);

	/* CEFUSE domain can be turned off post bootup */
	cefuse_pwrdm = pwrdm_lookup("cefuse_pwrdm");
	if (cefuse_pwrdm)
		omap_set_pwrdm_state(cefuse_pwrdm, PWRDM_POWER_OFF);
	else
		pr_warn("PM: Failed to get cefuse_pwrdm\n");

	ret = am33xx_push_sram_idle();
	if (ret)
		return ret;

#ifdef CONFIG_SUSPEND
	suspend_set_ops(&am33xx_pm_ops);
#endif /* CONFIG_SUSPEND */

	return 0;
}
