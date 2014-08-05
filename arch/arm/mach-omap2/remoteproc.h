/*
 * Remote processor machine-specific quirks for OMAP4+ SoCs
 *
 * Copyright (C) 2014-2016 Texas Instruments Incorporated - http://www.ti.com/
 *      Suman Anna <s-anna@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ARCH_ARM_MACH_OMAP2_REMOTEPROC_H
#define __ARCH_ARM_MACH_OMAP2_REMOTEPROC_H

#include "linux/platform_device.h"

struct omap_dm_timer;

#if IS_ENABLED(CONFIG_OMAP_REMOTEPROC)
int omap_rproc_device_enable(struct platform_device *pdev);
int omap_rproc_device_shutdown(struct platform_device *pdev);
struct omap_dm_timer *omap_rproc_request_timer(struct device_node *np);
int omap_rproc_release_timer(struct omap_dm_timer *timer);
int omap_rproc_start_timer(struct omap_dm_timer *timer);
int omap_rproc_stop_timer(struct omap_dm_timer *timer);
int omap_rproc_get_timer_irq(struct omap_dm_timer *timer);
void omap_rproc_ack_timer_irq(struct omap_dm_timer *timer);
#else
static inline int omap_rproc_device_enable(struct platform_device *pdev)
{
	return 0;
}

static inline int omap_rproc_device_shutdown(struct platform_device *pdev)
{
	return 0;
}

static inline
struct omap_dm_timer *omap_rproc_request_timer(struct device_node *np)
{
	return ERR_PTR(-ENODEV);
}

static inline int omap_rproc_release_timer(struct omap_dm_timer *timer)
{
	return -ENODEV;
}

static inline int omap_rproc_start_timer(struct omap_dm_timer *timer)
{
	return -ENODEV;
}

static inline int omap_rproc_stop_timer(struct omap_dm_timer *timer)
{
	return -ENODEV;
}

static inline int omap_rproc_get_timer_irq(struct omap_dm_timer *timer)
{
	return -1;
}

static inline void omap_rproc_ack_timer_irq(struct omap_dm_timer *timer) { }
#endif

#endif
