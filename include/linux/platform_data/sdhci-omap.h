// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2018 Texas Instruments
// SDHCI Controller Platform Data for TI's OMAP SoCs
// Author: Kishon Vijay Abraham I <kishon@ti.com>

#ifndef __SDHCI_OMAP_PDATA_H__
#define __SDHCI_OMAP_PDATA_H__

struct sdhci_omap_platform_data {
	const char *name;

	/*
	 * set if your board has components or wiring that limits the
	 * maximum frequency on the MMC bus
	 */
	unsigned int max_freq;

	/* string specifying a particular variant of hardware */
	char *version;
};

#endif
