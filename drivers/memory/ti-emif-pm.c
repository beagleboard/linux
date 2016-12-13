/*
 * TI AM33XX SRAM EMIF Driver
 *
 * Copyright (C) 2016 Texas Instruments Inc.
 *	Dave Gerlach
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/err.h>
#include <linux/genalloc.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/sram.h>
#include <linux/ti-emif-sram.h>

#include <asm/fncpy.h>

#include "emif.h"

#define TI_EMIF_SRAM_SYMBOL_OFFSET(sym) ((unsigned long)(sym) - \
					 (unsigned long)&ti_emif_sram)

#define EMIF_POWER_MGMT_WAIT_SELF_REFRESH_8192_CYCLES		0x00a0

static phys_addr_t ti_emif_sram_phys, ti_emif_sram_data_phys;
static unsigned long ti_emif_sram_virt, ti_emif_sram_data_virt;
static struct gen_pool *sram_pool_code, *sram_pool_data;
static struct ti_emif_pm_data pm_data;
static struct ti_emif_pm_functions pm_functions;

static u32 sram_suspend_address(unsigned long addr)
{
	return (ti_emif_sram_virt + TI_EMIF_SRAM_SYMBOL_OFFSET(addr));
}

static phys_addr_t sram_resume_address(unsigned long addr)
{
	return ((unsigned long)ti_emif_sram_phys +
		TI_EMIF_SRAM_SYMBOL_OFFSET(addr));
}

static void ti_emif_free_sram(void)
{
	gen_pool_free(sram_pool_code, ti_emif_sram_virt, ti_emif_sram_sz);
	gen_pool_free(sram_pool_data, ti_emif_sram_data_virt,
		      sizeof(struct emif_regs_amx3));
}

static int ti_emif_prepare_push_sram(struct device *dev)
{
	struct device_node *np = dev->of_node;
	int ret;

	sram_pool_code = of_gen_pool_get(np, "sram", 0);
	if (!sram_pool_code) {
		dev_err(dev, "Unable to get sram pool for ocmcram code\n");
		return -ENODEV;
	}

	ti_emif_sram_virt = gen_pool_alloc(sram_pool_code, ti_emif_sram_sz);
	if (!ti_emif_sram_virt) {
		dev_err(dev, "Unable to allocate code memory from ocmcram\n");
		return -ENOMEM;
	}

	/* Save physical address to calculate resume offset during pm init */
	ti_emif_sram_phys = gen_pool_virt_to_phys(sram_pool_code,
						  ti_emif_sram_virt);

	/* Get sram pool for data section and allocate space */
	sram_pool_data = of_gen_pool_get(np, "sram", 1);
	if (!sram_pool_data) {
		dev_err(dev, "Unable to get sram pool for ocmcram data\n");
		ret = -ENODEV;
		goto err_free_sram_code;
	}

	ti_emif_sram_data_virt = gen_pool_alloc(sram_pool_data,
						sizeof(struct emif_regs_amx3));
	if (!ti_emif_sram_data_virt) {
		dev_err(dev, "Unable to allocate data memory from ocmcram\n");
		return -ENOMEM;
		goto err_free_sram_code;
	}

	/* Save physical address to calculate resume offset during pm init */
	ti_emif_sram_data_phys = gen_pool_virt_to_phys(sram_pool_data,
						       ti_emif_sram_data_virt);
	/*
	 * These functions are called during suspend path while MMU is
	 * still on so add virtual base to offset for absolute address
	 */
	pm_functions.save_context =
		sram_suspend_address((unsigned long)ti_emif_save_context);
	pm_functions.enter_sr =
		sram_suspend_address((unsigned long)ti_emif_enter_sr);
	pm_functions.abort_sr =
		sram_suspend_address((unsigned long)ti_emif_abort_sr);

	/*
	 * These are called during resume path when MMU is not enabled
	 * so physical address is used instead
	 */
	pm_functions.restore_context =
		sram_resume_address((unsigned long)ti_emif_restore_context);
	pm_functions.exit_sr =
		sram_resume_address((unsigned long)ti_emif_exit_sr);

	pm_data.regs_virt = (struct emif_regs_amx3 *)ti_emif_sram_data_virt;
	pm_data.regs_phys = (struct emif_regs_amx3 *)ti_emif_sram_data_phys;

	return 0;

err_free_sram_code:
	gen_pool_free(sram_pool_code, ti_emif_sram_virt, ti_emif_sram_sz);
	return ret;
}

static int ti_emif_push_sram(struct device *dev)
{
	int ret;

	ret = sram_exec_copy(sram_pool_code, (void *)ti_emif_sram_virt,
			     &ti_emif_sram, ti_emif_sram_sz);
	if (ret) {
		dev_err(dev, "Cannot copy emif code to sram\n");
		return ret;
	}

	ret = sram_exec_copy(sram_pool_code,
			     (void *)sram_suspend_address((unsigned long)&ti_emif_pm_sram_data),
			     &pm_data, sizeof(pm_data));
	if (ret) {
		dev_err(dev, "Cannot copy emif data to code sram\n");
		return ret;
	}

	return 0;
}

/*
 * Due to Usage Note 3.1.2 "DDR3: JEDEC Compliance for Maximum
 * Self-Refresh Command Limit" found in AM335x Silicon Errata
 * (Document SPRZ360F Revised November 2013) we must configure
 * the self refresh delay timer to 0xA (8192 cycles) to avoid
 * generating too many refresh command from the EMIF.
 */
static void ti_emif_configure_sr_delay(void)
{
	writel(EMIF_POWER_MGMT_WAIT_SELF_REFRESH_8192_CYCLES,
	       (pm_data.ti_emif_base_addr_virt +
		EMIF_POWER_MANAGEMENT_CONTROL));

	writel(EMIF_POWER_MGMT_WAIT_SELF_REFRESH_8192_CYCLES,
	       (pm_data.ti_emif_base_addr_virt +
		EMIF_POWER_MANAGEMENT_CTRL_SHDW));
}

/**
 * ti_emif_copy_pm_function_table - copy mapping of pm funcs in sram
 * @sram_pool: pointer to struct gen_pool where dst resides
 * @dst: void * to address that table should be copied
 *
 * Returns 0 if success other error code if table is not available
 */
int ti_emif_copy_pm_function_table(struct gen_pool *sram_pool, void *dst)
{
	int ret;

	if (!ti_emif_sram_virt)
		return -EINVAL;

	ret = sram_exec_copy(sram_pool, dst, &pm_functions,
			     sizeof(pm_functions));

	return ret;
}
EXPORT_SYMBOL_GPL(ti_emif_copy_pm_function_table);

/**
 * ti_emif_get_mem_type - return type for memory type in use
 *
 * Returns memory type value read from EMIF or error code if fails
 */
int ti_emif_get_mem_type(void)
{
	unsigned long temp;

	if (!pm_data.ti_emif_base_addr_virt ||
	    IS_ERR(pm_data.ti_emif_base_addr_virt))
		return -ENODEV;

	temp = readl(pm_data.ti_emif_base_addr_virt +
		     EMIF_SDRAM_CONFIG);

	temp = (temp & SDRAM_TYPE_MASK) >> SDRAM_TYPE_SHIFT;
	return temp;
}
EXPORT_SYMBOL_GPL(ti_emif_get_mem_type);

static const struct of_device_id ti_emif_of_match[] = {
	{ .compatible = "ti,emif-am3352", .data =
					(void *)EMIF_SRAM_AM33_REG_LAYOUT, },
	{ .compatible = "ti,emif-am4372", .data =
					(void *)EMIF_SRAM_AM43_REG_LAYOUT, },
	{},
};
MODULE_DEVICE_TABLE(of, ti_emif_of_match);

static int ti_emif_probe(struct platform_device *pdev)
{
	int ret = -ENODEV;
	struct resource *res;
	struct device *dev = &pdev->dev;
	const struct of_device_id *match;

	match = of_match_device(ti_emif_of_match, &pdev->dev);
	if (!match)
		return -ENODEV;

	pm_data.ti_emif_sram_config = (u32)match->data;

	pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);

	if (IS_ERR_VALUE(ret)) {
		dev_err(dev, "pm_runtime_get_sync() failed\n");
		goto fail_runtime_put;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pm_data.ti_emif_base_addr_virt = devm_ioremap_resource(dev, res);
	if (IS_ERR(pm_data.ti_emif_base_addr_virt)) {
		dev_err(dev, "could not ioremap emif mem\n");
		ret =  PTR_ERR(pm_data.ti_emif_base_addr_virt);
		goto fail_runtime_put;
	}

	pm_data.ti_emif_base_addr_phys = res->start;

	ti_emif_configure_sr_delay();

	ret = ti_emif_prepare_push_sram(dev);
	if (ret)
		goto fail_runtime_put;

	ret = ti_emif_push_sram(dev);
	if (ret)
		goto fail_free_sram;

	return 0;

fail_free_sram:
	ti_emif_free_sram();
fail_runtime_put:
	pm_runtime_put_sync(dev);
	return ret;
}

static int ti_emif_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	pm_runtime_put_sync(dev);
	pm_runtime_disable(dev);

	ti_emif_free_sram();

	return 0;
}

static struct platform_driver ti_emif_driver = {
	.probe = ti_emif_probe,
	.remove = ti_emif_remove,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = of_match_ptr(ti_emif_of_match),
	},
};
module_platform_driver(ti_emif_driver);

MODULE_AUTHOR("Dave Gerlach <d-gerlach@ti.com>");
MODULE_DESCRIPTION("Texas Instruments SRAM EMIF driver");
MODULE_LICENSE("GPL v2");
