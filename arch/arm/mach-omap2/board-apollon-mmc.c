/*
 * linux/arch/arm/mach-omap2/board-apollon-mmc.c
 *
 * Copyright (C) 2005-2007 Samsung Electronics
 * Author: Kyungmin Park <kyungmin.park@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>

#include <mach/gpio.h>
#include <mach/mmc.h>

#ifdef CONFIG_MMC_OMAP

static struct device *mmc_device;

static int apollon_mmc_set_power(struct device *dev, int slot, int power_on,
					int vdd)
{
#ifdef CONFIG_MMC_DEBUG
	dev_dbg(dev, "Set slot %d power: %s (vdd %d)\n", slot + 1,
		power_on ? "on" : "off", vdd);
#endif
	if (slot != 0) {
		dev_err(dev, "No such slot %d\n", slot + 1);
		return -ENODEV;
	}

	return 0;
}

static int apollon_mmc_set_bus_mode(struct device *dev, int slot, int bus_mode)
{
#ifdef CONFIG_MMC_DEBUG
	dev_dbg(dev, "Set slot %d bus_mode %s\n", slot + 1,
		bus_mode == MMC_BUSMODE_OPENDRAIN ? "open-drain" : "push-pull");
#endif
	if (slot != 0) {
		dev_err(dev, "No such slot %d\n", slot + 1);
		return -ENODEV;
	}

	return 0;
}

static int apollon_mmc_late_init(struct device *dev)
{
	mmc_device = dev;

	return 0;
}

static void apollon_mmc_cleanup(struct device *dev)
{
}

static struct omap_mmc_platform_data apollon_mmc_data = {
	.nr_slots			= 1,
	.switch_slot			= NULL,
	.init				= apollon_mmc_late_init,
	.cleanup			= apollon_mmc_cleanup,
	.slots[0]	= {
		.set_power		= apollon_mmc_set_power,
		.set_bus_mode		= apollon_mmc_set_bus_mode,
		.get_ro			= NULL,
		.get_cover_state	= NULL,
		.ocr_mask		= MMC_VDD_30_31 | MMC_VDD_31_32 |
					  MMC_VDD_32_33 | MMC_VDD_33_34,
		.name			= "mmcblk",
	},
};

void __init apollon_mmc_init(void)
{
	omap_set_mmc_info(1, &apollon_mmc_data);
}

#else	/* !CONFIG_MMC_OMAP */

void __init apollon_mmc_init(void)
{
}

#endif	/* CONFIG_MMC_OMAP */
