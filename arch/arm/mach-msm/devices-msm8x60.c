/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <mach/board.h>
#include <mach/msm_iomap.h>
#include <mach/dma.h>

static struct resource resources_dmov_adm0[] = {
	{
		.start = MSM8X60_DMOV_ADM0_PHYS,
		.end = MSM8X60_DMOV_ADM0_PHYS + MSM8X60_DMOV_ADM0_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = INT_ADM0_AARM,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource resources_dmov_adm1[] = {
	{
		.start = MSM8X60_DMOV_ADM1_PHYS,
		.end = MSM8X60_DMOV_ADM1_PHYS + MSM8X60_DMOV_ADM1_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.start = INT_ADM1_AARM,
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_dmov_adm0 = {
	.name	= "msm_dmov",
	.id	= 0,
	.num_resources = ARRAY_SIZE(resources_dmov_adm0),
	.resource = resources_dmov_adm0,
};

struct platform_device msm_device_dmov_adm1 = {
	.name	= "msm_dmov",
	.id	= 1,
	.num_resources = ARRAY_SIZE(resources_dmov_adm1),
	.resource = resources_dmov_adm1,
};
