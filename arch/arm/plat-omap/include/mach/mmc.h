/*
 * MMC definitions for OMAP2
 *
 * Copyright (C) 2006 Nokia Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __OMAP2_MMC_H
#define __OMAP2_MMC_H

#include <linux/types.h>
#include <linux/device.h>
#include <linux/mmc/host.h>

#include <mach/board.h>

#define OMAP_MMC_MAX_SLOTS	2

struct omap_mmc_platform_data {

	/* number of slots on board */
	unsigned nr_slots:2;

	/* set if your board has components or wiring that limits the
	 * maximum frequency on the MMC bus */
	unsigned int max_freq;

	/* switch the bus to a new slot */
	int (* switch_slot)(struct device *dev, int slot);
	/* initialize board-specific MMC functionality, can be NULL if
	 * not supported */
	int (* init)(struct device *dev);
	void (* cleanup)(struct device *dev);
	void (* shutdown)(struct device *dev);

	/* To handle board related suspend/resume functionality for MMC */
	int (*suspend)(struct device *dev, int slot);
	int (*resume)(struct device *dev, int slot);

	struct omap_mmc_slot_data {

		unsigned enabled:1;

		/*
		 * nomux means "standard" muxing is wrong on this board, and
		 * that board-specific code handled it before common init logic.
		 */
		unsigned nomux:1;

		/* switch pin can be for card detect (default) or card cover */
		unsigned cover:1;

		/* 4 wire signaling is optional, and is only used for SD/SDIO */
		unsigned wire4:1;

		/* use the internal clock */
		unsigned internal_clock:1;
		s16 power_pin;
		s16 switch_pin;
		s16 wp_pin;

		int (* set_bus_mode)(struct device *dev, int slot, int bus_mode);
		int (* set_power)(struct device *dev, int slot, int power_on, int vdd);
		int (* get_ro)(struct device *dev, int slot);

		/* return MMC cover switch state, can be NULL if not supported.
		 *
		 * possible return values:
		 *   0 - open
		 *   1 - closed
		 */
		int (* get_cover_state)(struct device *dev, int slot);

		const char *name;
		u32 ocr_mask;

		/* Card detection IRQs */
		int card_detect_irq;
		int (* card_detect)(int irq);

		unsigned int ban_openended:1;

	} slots[OMAP_MMC_MAX_SLOTS];
};

/* called from board-specific card detection service routine */
extern void omap_mmc_notify_cover_event(struct device *dev, int slot, int is_closed);

#if	defined(CONFIG_MMC_OMAP) || defined(CONFIG_MMC_OMAP_MODULE) || \
	defined(CONFIG_MMC_OMAP_HS) || defined(CONFIG_MMC_OMAP_HS_MODULE)
void omap1_init_mmc(struct omap_mmc_platform_data *info);
void omap2_init_mmc(struct omap_mmc_platform_data *info);
void omap_init_mmc(struct omap_mmc_platform_data *info,
		struct platform_device *pdev1, struct platform_device *pdev2);
#else
static inline void omap1_init_mmc(struct omap_mmc_platform_data *info)
{
}
static inline void omap2_init_mmc(struct omap_mmc_platform_data *info)
{
}
static inline void omap_init_mmc(struct omap_mmc_platform_data *info,
		struct platform_device *pdev1, struct platform_device *pdev2)
{
}
#endif

#if defined(CONFIG_MMC_OMAP_HS) || defined(CONFIG_MMC_OMAP_HS_MODULE)
void __init hsmmc_init(void);
#endif

#endif
