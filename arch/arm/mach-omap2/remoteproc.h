/*
 * Remote processor machine-specific quirks for OMAP4+ SoCs
 *
 * Copyright (C) 2014-2017 Texas Instruments Incorporated - http://www.ti.com/
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

#if IS_ENABLED(CONFIG_OMAP_REMOTEPROC)
int omap_rproc_device_enable(struct platform_device *pdev);
int omap_rproc_device_shutdown(struct platform_device *pdev);
#else
static inline int omap_rproc_device_enable(struct platform_device *pdev)
{
	return 0;
}

static inline int omap_rproc_device_shutdown(struct platform_device *pdev)
{
	return 0;
}
#endif

#endif
