/*
 * TI AM33XX SRAM EMIF Driver
 *
 * Copyright (C) 2015 Texas Instruments Inc.
 *          Dave Gerlach <d-gerlach@ti.com>
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

#include <asm/fncpy.h>

#include "emif.h"

#define EMIF_POWER_MGMT_WAIT_SELF_REFRESH_8192_CYCLES		0x00a0

struct ti_emif_pm_functions {
	u32 save_context;
	u32 restore_context;
	u32 enter_sr;
	u32 exit_sr;
	u32 abort_sr;
} __packed;

static struct gen_pool *sram_pool;
static phys_addr_t ti_emif_sram_phys;
static void __iomem *ti_emif_sram_virt;
static unsigned long ocmcram_location;

static u32 sram_suspend_address(unsigned long fn_offset)
{
	return (unsigned long)ti_emif_sram_virt + fn_offset;
}

static phys_addr_t sram_resume_address(unsigned long fn_offset)
{
	return ti_emif_sram_phys + fn_offset;
}

static int ti_emif_push_sram(struct device *dev)
{
	struct device_node *np = dev->of_node;

	sram_pool = of_gen_pool_get(np, "sram", 0);

	if (!sram_pool) {
		dev_err(dev, "Unable to get sram pool for ocmcram\n");
		return -ENODEV;
	}

	ocmcram_location = gen_pool_alloc(sram_pool, ti_emif_sram_sz);
	if (!ocmcram_location) {
		dev_err(dev, "Unable to allocate memory from ocmcram\n");
		return -EINVAL;
	}

	/* Save physical address to calculate resume offset during pm init */
	ti_emif_sram_phys = gen_pool_virt_to_phys(sram_pool,
						  ocmcram_location);
	ti_emif_sram_virt = fncpy((void *)ocmcram_location,
				  &ti_emif_sram,
				  ti_emif_sram_sz);

	/*
	 * These functions are called during suspend path while MMU is
	 * still on so add virtual base to offset for absolute address
	 */
	ti_emif_pm.save_context = sram_suspend_address(ti_emif_pm.save_context);
	ti_emif_pm.enter_sr = sram_suspend_address(ti_emif_pm.enter_sr);
	ti_emif_pm.abort_sr = sram_suspend_address(ti_emif_pm.abort_sr);

	/*
	 * These are called during resume path when MMU is not enabled
	 * so physical address is used instead
	 */
	ti_emif_pm.restore_context =
		sram_resume_address(ti_emif_pm.restore_context);
	ti_emif_pm.exit_sr = sram_resume_address(ti_emif_pm.exit_sr);

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
	       (ti_emif_base_addr_virt +
		EMIF_POWER_MANAGEMENT_CONTROL));

	writel(EMIF_POWER_MGMT_WAIT_SELF_REFRESH_8192_CYCLES,
	       (ti_emif_base_addr_virt +
		EMIF_POWER_MANAGEMENT_CTRL_SHDW));
}

/**
 * ti_emif_copy_pm_function_table - copy mapping of pm funcs in sram
 * @dst: void * to address that table should be copied
 *
 * Returns 0 if success other error code if table is not available
 */
int ti_emif_copy_pm_function_table(void *dst)
{
	if (!ti_emif_sram_virt)
		return -EINVAL;

	memcpy_toio(dst, &ti_emif_pm, sizeof(ti_emif_pm));

	return 0;
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

	if (!ti_emif_base_addr_virt || IS_ERR(ti_emif_base_addr_virt))
		return -ENODEV;

	temp = readl(ti_emif_base_addr_virt + EMIF_SDRAM_CONFIG);

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

#ifdef CONFIG_PM_SLEEP
static int ti_emif_resume(struct device *dev)
{
	unsigned long tmp =  __raw_readl(ti_emif_sram_virt);

	/*
	 * Check to see if what we are copying is already present in the
	 * first byte at the destination, only copy if it is not which
	 * indicates we have lost context and sram no longer contains
	 * the PM code
	 */
	if (tmp != ti_emif_sram)
		fncpy((void *)ocmcram_location,
		      &ti_emif_sram,
		      ti_emif_sram_sz);

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static int ti_emif_probe(struct platform_device *pdev)
{
	int ret = -ENODEV;
	struct resource *res;
	struct device *dev = &pdev->dev;
	const struct of_device_id *match;

	match = of_match_device(ti_emif_of_match, &pdev->dev);
	if (!match)
		return -ENODEV;

	ti_emif_sram_config = (u32)match->data;

	pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);

	if (IS_ERR_VALUE(ret)) {
		dev_err(dev, "pm_runtime_get_sync() failed\n");
		goto fail_runtime_put;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ti_emif_base_addr_virt = devm_ioremap_resource(dev, res);
	if (IS_ERR(ti_emif_base_addr_virt)) {
		dev_err(dev, "could not ioremap emif mem\n");
		ret =  PTR_ERR(ti_emif_base_addr_virt);
		goto fail_runtime_put;
	}

	ti_emif_base_addr_phys = res->start;

	ti_emif_configure_sr_delay();

	ret = ti_emif_push_sram(dev);
	if (ret)
		goto fail_runtime_put;

	return 0;

fail_runtime_put:
	pm_runtime_put_sync(dev);
	return ret;
}

static int ti_emif_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	pm_runtime_put_sync(dev);
	pm_runtime_disable(dev);

	gen_pool_free(sram_pool, ocmcram_location,
		      ti_emif_sram_sz);

	return 0;
}

static const struct dev_pm_ops ti_emif_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(NULL, ti_emif_resume)
};

static struct platform_driver ti_emif_driver = {
	.probe = ti_emif_probe,
	.remove = ti_emif_remove,
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = of_match_ptr(ti_emif_of_match),
		.pm = &ti_emif_pm_ops,
	},
};

module_platform_driver(ti_emif_driver);

MODULE_AUTHOR("Dave Gerlach <d-gerlach@ti.com>");
MODULE_DESCRIPTION("Texas Instruments SRAM EMIF driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" KBUILD_MODNAME);
