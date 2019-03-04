/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Remote processor machine-specific quirks for OMAP4+ SoCs
 *
 * Copyright (C) 2014-2019 Texas Instruments Incorporated - http://www.ti.com/
 *      Suman Anna <s-anna@ti.com>
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
