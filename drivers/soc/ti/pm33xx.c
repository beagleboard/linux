/*
 * AM33XX Power Management Routines
 *
 * Copyright (C) 2012-2016 Texas Instruments Incorporated - http://www.ti.com/
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
#include <linux/platform_data/pm33xx.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/sram.h>
#include <linux/suspend.h>
#include <linux/ti-emif-sram.h>
#include <linux/wkup_m3_ipc.h>

#include <asm/fncpy.h>
#include <asm/proc-fns.h>
#include <asm/suspend.h>
#include <asm/system_misc.h>

#define AMX3_PM_SRAM_SYMBOL_OFFSET(sym) ((unsigned long)(sym) - \
					 (unsigned long)pm_sram->do_wfi)

static int (*am33xx_do_wfi_sram)(unsigned long unused);
static phys_addr_t am33xx_do_wfi_sram_phys;

static struct gen_pool *sram_pool, *sram_pool_data;
static phys_addr_t ocmcram_location, ocmcram_location_data;

static struct am33xx_pm_platform_data *pm_ops;
static struct am33xx_pm_sram_addr *pm_sram;

static struct wkup_m3_ipc *m3_ipc;

static u32 sram_suspend_address(unsigned long addr)
{
	return ((unsigned long)am33xx_do_wfi_sram +
		AMX3_PM_SRAM_SYMBOL_OFFSET(addr));
}

#ifdef CONFIG_SUSPEND
static int am33xx_pm_suspend(suspend_state_t suspend_state)
{
	int i, ret = 0;

	ret = pm_ops->soc_suspend(suspend_state, am33xx_do_wfi_sram);

	if (ret) {
		pr_err("PM: Kernel suspend failure\n");
	} else {
		i = m3_ipc->ops->request_pm_status(m3_ipc);

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

	switch (state) {
	case PM_SUSPEND_MEM:
		ret = m3_ipc->ops->prepare_low_power(m3_ipc, WKUP_M3_DEEPSLEEP);
		break;
	case PM_SUSPEND_STANDBY:
		ret = m3_ipc->ops->prepare_low_power(m3_ipc, WKUP_M3_STANDBY);
		break;
	}

	return ret;
}

static void am33xx_pm_end(void)
{
	m3_ipc->ops->finish_low_power(m3_ipc);
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
	int temp;

	temp = ti_emif_get_mem_type();
	if (temp < 0) {
		pr_err("PM: Cannot determine memory type, no PM available\n");
		return;
	}
	m3_ipc->ops->set_mem_type(m3_ipc, temp);

	/* Physical resume address to be used by ROM code */
	resume_address = (void *)am33xx_do_wfi_sram_phys +
			 *pm_sram->resume_offset + 0x4;

	m3_ipc->ops->set_resume_address(m3_ipc, resume_address);
}

static void am33xx_pm_free_sram(void)
{
	gen_pool_free(sram_pool, ocmcram_location, *pm_sram->do_wfi_sz);
	gen_pool_free(sram_pool_data, ocmcram_location_data,
		      sizeof(struct am33xx_pm_ro_sram_data));
}

/*
 * Push the minimal suspend-resume code to SRAM
 */
static int am33xx_prepare_push_sram_idle(void)
{
	struct device_node *np;

	np = of_find_compatible_node(NULL, NULL, "ti,omap3-mpu");

	if (!np) {
		np = of_find_compatible_node(NULL, NULL, "ti,omap4-mpu");
		if (!np) {
			pr_warn("PM: %s: Unable to find device node for mpu\n",
				__func__);
			return -ENODEV;
		}
	}

	sram_pool = of_gen_pool_get(np, "pm-sram", 0);
	if (!sram_pool) {
		pr_warn("PM: %s: Unable to get sram pool for ocmcram\n",
			__func__);
		return -ENODEV;
	}

	sram_pool_data = of_gen_pool_get(np, "pm-sram", 1);
	if (!sram_pool_data) {
		pr_warn("PM: %s: Unable to get sram data pool for ocmcram\n",
			__func__);
		return -ENODEV;
	}

	ocmcram_location = gen_pool_alloc(sram_pool, *pm_sram->do_wfi_sz);
	if (!ocmcram_location) {
		pr_warn("PM: %s: Unable to allocate memory from ocmcram\n",
			__func__);
		return -ENOMEM;
	}

	ocmcram_location_data = gen_pool_alloc(sram_pool_data,
					       sizeof(struct emif_regs_amx3));
	if (!ocmcram_location_data) {
		pr_err("PM: Unable to allocate memory from ocmcram\n");
		gen_pool_free(sram_pool, ocmcram_location, *pm_sram->do_wfi_sz);
		return -ENOMEM;
	}

	return 0;
}

static int am33xx_push_sram_idle(void)
{
	struct am33xx_pm_ro_sram_data ro_sram_data;
	int ret;

	ro_sram_data.amx3_pm_sram_data_virt = ocmcram_location_data;
	ro_sram_data.amx3_pm_sram_data_phys =
		gen_pool_virt_to_phys(sram_pool_data, ocmcram_location_data);

	/* Save physical address to calculate resume offset during pm init */
	am33xx_do_wfi_sram = (void *)ocmcram_location;
	am33xx_do_wfi_sram_phys = gen_pool_virt_to_phys(sram_pool,
							ocmcram_location);

	ret = sram_exec_copy(sram_pool, am33xx_do_wfi_sram, pm_sram->do_wfi,
			     *pm_sram->do_wfi_sz);
	if (ret) {
		pr_err("PM: %s: am33xx_do_wfi copy to sram failed\n", __func__);
		return ret;
	}

	ret = ti_emif_copy_pm_function_table(sram_pool,
			(void *)sram_suspend_address((unsigned long)pm_sram->emif_sram_table));
	if (ret) {
		pr_warn("PM: %s: EMIF function copy failed\n", __func__);
		ret =  -EPROBE_DEFER;
		return ret;
	}

	ret = sram_exec_copy(sram_pool,
			     (void *)sram_suspend_address((unsigned long)pm_sram->ro_sram_data),
			     &ro_sram_data,
			     sizeof(ro_sram_data));
	if (ret) {
		pr_err("PM: %s: ro_sram_data copy to sram failed\n", __func__);
		return ret;
	}

	return 0;
}

static int am33xx_pm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret;

	if (!of_machine_is_compatible("ti,am33xx") &&
	    !of_machine_is_compatible("ti,am43"))
		return -ENODEV;

	pm_ops = dev->platform_data;
	if (!pm_ops) {
		pr_err("PM: Cannot get core PM ops!\n");
		return -ENODEV;
	}

	pm_sram = pm_ops->get_sram_addrs();
	if (!pm_sram) {
		pr_err("PM: Cannot get PM asm function addresses!!\n");
		return -ENODEV;
	}

	ret = am33xx_prepare_push_sram_idle();
	if (ret)
		return ret;

	ret = am33xx_push_sram_idle();
	if (ret)
		goto err_free_sram;

	m3_ipc = wkup_m3_ipc_get();
	if (!m3_ipc) {
		pr_err("PM: Cannot get wkup_m3_ipc handle\n");
		ret = -EPROBE_DEFER;
		goto err_free_sram;
	}

	am33xx_pm_set_ipc_ops();

#ifdef CONFIG_SUSPEND
	suspend_set_ops(&am33xx_pm_ops);
#endif /* CONFIG_SUSPEND */

	ret = pm_ops->init();
	if (ret) {
		pr_err("Unable to call core pm init!\n");
		ret = -ENODEV;
		goto err_put_wkup_m3_ipc;
	}

	return 0;

err_put_wkup_m3_ipc:
	wkup_m3_ipc_put(m3_ipc);
err_free_sram:
	am33xx_pm_free_sram();
	return ret;
}

static int am33xx_pm_remove(struct platform_device *pdev)
{
	suspend_set_ops(NULL);
	wkup_m3_ipc_put(m3_ipc);
	am33xx_pm_free_sram();
	return 0;
}

static struct platform_driver am33xx_pm_driver = {
	.driver = {
		.name   = "pm33xx",
	},
	.probe = am33xx_pm_probe,
	.remove = am33xx_pm_remove,
};

module_platform_driver(am33xx_pm_driver);

MODULE_ALIAS("platform:pm33xx");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("am33xx power management driver");
