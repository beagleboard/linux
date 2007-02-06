/*
 * linux/arch/arm/mach-omap2/board-n800-flash.c
 *
 * Copyright (C) 2006 Nokia Corporation
 * Author: Juha Yrjola
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <asm/mach/flash.h>
#include <asm/arch/onenand.h>
#include <asm/arch/board.h>

static struct mtd_partition n800_partitions[8];

static struct omap_onenand_platform_data n800_onenand_data = {
	.cs = 0,
	.gpio_irq = 26,
	.parts = n800_partitions,
	.nr_parts = 0 /* filled later */
};

static struct platform_device n800_onenand_device = {
	.name		= "omap2-onenand",
	.id		= -1,
	.dev = {
		.platform_data = &n800_onenand_data,
	},
};


void __init n800_flash_init(void)
{
	const struct omap_partition_config *part;
	int i = 0;

	while ((part = omap_get_nr_config(OMAP_TAG_PARTITION,
					  struct omap_partition_config, i)) != NULL) {
		struct mtd_partition *mpart;

		mpart = n800_partitions + i;
		mpart->name = (char *) part->name;
		mpart->size = part->size;
		mpart->offset = part->offset;
		mpart->mask_flags = part->mask_flags;
		i++;
		if (i == ARRAY_SIZE(n800_partitions)) {
			printk(KERN_ERR "Too many partitions supplied\n");
			return;
		}
	}
	n800_onenand_data.nr_parts = i;
	if (platform_device_register(&n800_onenand_device) < 0) {
		printk(KERN_ERR "Unable to register OneNAND device\n");
		return;
	}
}
