/*
 * TI AM33XX SRAM EMIF Driver
 *
 * Copyright (C) 2014 Texas Instruments Inc.
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

static void __iomem *ti_emif_sram_phys;
static void __iomem *ti_emif_sram_virt;

static u32 sram_suspend_address(unsigned long fn_offset)
{
	return (unsigned long)ti_emif_sram_virt + fn_offset;
}

static u32 sram_resume_address(unsigned long fn_offset)
{
	return (unsigned long)ti_emif_sram_phys + fn_offset;
}

static int ti_emif_push_sram(struct device_node *np)
{
	struct gen_pool *sram_pool;
	phys_addr_t ocmcram_location;

	sram_pool = of_get_named_gen_pool(np, "sram", 0);

	if (!sram_pool)
		pr_warn("PM: %s: Unable to allocate sram pool for ocmcram\n",
			__func__);

	ocmcram_location = gen_pool_alloc(sram_pool, ti_emif_sram_sz);
	if (!ocmcram_location)
		return -EINVAL;

	/* Save physical address to calculate resume offset during pm init */
	ti_emif_sram_phys = (void *)gen_pool_virt_to_phys(sram_pool,
							  ocmcram_location);
	ti_emif_sram_virt = (void *)fncpy((void *)ocmcram_location,
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
	       (void *)(ti_emif_base_addr_virt +
			EMIF_POWER_MANAGEMENT_CONTROL));

	writel(EMIF_POWER_MGMT_WAIT_SELF_REFRESH_8192_CYCLES,
	       (void *)(ti_emif_base_addr_virt +
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
		return -ENODEV;

	memcpy_toio(dst, &ti_emif_pm, sizeof(ti_emif_pm));

	return 0;
}

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

	temp = readl((void *)ti_emif_base_addr_virt + EMIF_SDRAM_CONFIG);

	temp = (temp & SDRAM_TYPE_MASK) >> SDRAM_TYPE_SHIFT;
	return temp;
}

static const struct of_device_id ti_emif_of_match[] = {
	{ .compatible = "ti,am3352-emif", },
	{},
};

static int ti_emif_probe(struct platform_device *pdev)
{
	int ret = -ENODEV;
	struct resource *res;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ti_emif_base_addr_virt = devm_ioremap_resource(dev, res);
	if (IS_ERR(ti_emif_base_addr_virt)) {
		dev_err(dev, "could not ioremap emif mem\n");
		return PTR_ERR(ti_emif_base_addr_virt);
	}

	ti_emif_base_addr_phys = (void *)res->start;

	ti_emif_configure_sr_delay();

	ret = ti_emif_push_sram(np);
	if (ret)
		return ret;

	return 0;
}

static int ti_emif_remove(struct platform_device *pdev)
{
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
MODULE_ALIAS("platform:" KBUILD_MODNAME);
