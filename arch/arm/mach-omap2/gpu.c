/*
 * Deassert reset for AM33xx graphics device(SGX) hwmod
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
 * Prathap MS <msprathap@ti.com>
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
#include <linux/of_platform.h>
#include "omap_device.h"

void __init omap_sgx_init_of(void)
{
	struct device_node *node;
	struct platform_device *pdev;
	int ret = 0;
	node = of_find_compatible_node(NULL, NULL, "ti,sgx");
	if (!node)
		return;
	pdev = of_find_device_by_node(node);
	if (!pdev) {
		pr_warn("of_find_device_by_node() failed for sgx\n");
		return;
	}
	ret = omap_device_deassert_hardreset(pdev, "gfx");
	if (ret != 0)
		pr_warn("omap_device_deassert_hardreset() failed for sgx(gfx hwmod)\n");

	node = of_find_compatible_node(NULL, NULL, "ti,am335x-timer");
	if (!node)
		return;
	pdev = of_find_device_by_node(node);
	if (!pdev) {
		pr_warn("of_find_device_by_node() failed for sgx\n");
		return;
	}
	ret = omap_device_deassert_hardreset(pdev, "timer7");
	if (ret != 0)
		pr_warn("omap_device_deassert_hardreset() failed for sgx(gfx hwmod)\n");
}
