/*
 * linux/arch/arm/mach-omap2/board-rx51-flash.c
 *
 * Copyright (C) 2008 Nokia
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <asm/mach/flash.h>

#include <mach/onenand.h>

#include "mmc-twl4030.h"

#define	RX51_FLASH_CS	0

extern struct mtd_partition n800_partitions[ONENAND_MAX_PARTITIONS];
extern int n800_onenand_setup(void __iomem *onenand_base, int freq);
extern void __init n800_flash_init(void);

static struct flash_platform_data rx51_flash_data = {
	.map_name	= "cfi_probe",
	.width		= 2,
	.parts		= n800_partitions,
	.nr_parts	= ARRAY_SIZE(n800_partitions),
};

static struct resource rx51_flash_resource = {
	.flags		= IORESOURCE_MEM,
};

static struct platform_device rx51_flash_device = {
	.name		= "omapflash",
	.id		= 0,
	.dev		= {
		.platform_data	= &rx51_flash_data,
	},
	.num_resources	= 1,
	.resource	= &rx51_flash_resource,
};

static struct platform_device *rx51_flash_devices[] = {
	&rx51_flash_device,
};

static struct twl4030_hsmmc_info mmc[] __initdata = {
	{
		.mmc		= 1,
		.wires		= 8,
		.gpio_cd	= 160,
		.gpio_wp	= -EINVAL,
	},
	{
		.mmc		= 2,
		.wires		= 8,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
	},
	{}	/* Terminator */
};

void __init rx51_flash_init(void)
{
	platform_add_devices(rx51_flash_devices, ARRAY_SIZE(rx51_flash_devices));
	n800_flash_init();
	twl4030_mmc_init(mmc);
}

