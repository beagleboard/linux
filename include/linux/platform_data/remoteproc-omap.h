/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Remote Processor - omap-specific bits
 *
 * Copyright (C) 2011-2019 Texas Instruments Incorporated - http://www.ti.com/
 * Copyright (C) 2011 Google, Inc.
 */

#ifndef _PLAT_REMOTEPROC_H
#define _PLAT_REMOTEPROC_H

struct platform_device;

/*
 * struct omap_rproc_pdata - omap remoteproc's platform data
 * @device_enable: omap-specific handler for enabling a device
 * @device_shutdown: omap-specific handler for shutting down a device
 */
struct omap_rproc_pdata {
	int (*device_enable)(struct platform_device *pdev);
	int (*device_shutdown)(struct platform_device *pdev);
};

#if defined(CONFIG_OMAP_REMOTEPROC) || defined(CONFIG_OMAP_REMOTEPROC_MODULE)

void __init omap_rproc_reserve_cma(void);

#else

static inline void __init omap_rproc_reserve_cma(void)
{
}

#endif

#endif /* _PLAT_REMOTEPROC_H */
