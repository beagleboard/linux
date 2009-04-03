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
#include <linux/platform_device.h>

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

/*
 * Note: If you want to detect card feature, please assign GPIO 37
 */
static struct omap_mmc_platform_data mmc1_data = {
	.nr_slots			= 1,
	.init				= apollon_mmc_late_init,
	.cleanup			= apollon_mmc_cleanup,
	.dma_mask			= 0xffffffff,
	.slots[0]	= {
		.wires			= 4,

		/*
		 * Use internal loop-back in MMC/SDIO Module Input Clock
		 * selection
		 */
		.internal_clock		= 1,

		.set_power		= apollon_mmc_set_power,
		.set_bus_mode		= apollon_mmc_set_bus_mode,
		.ocr_mask		= MMC_VDD_30_31 | MMC_VDD_31_32 |
					  MMC_VDD_32_33 | MMC_VDD_33_34,
		.name			= "mmcblk",
	},
};

static struct omap_mmc_platform_data *mmc_data[OMAP24XX_NR_MMC];

void __init apollon_mmc_init(void)
{
	mmc_data[0] = &mmc1_data;
	omap2_init_mmc(mmc_data, OMAP24XX_NR_MMC);
}

#else	/* !CONFIG_MMC_OMAP */

void __init apollon_mmc_init(void)
{
}

#endif	/* CONFIG_MMC_OMAP */
