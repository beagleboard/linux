/*
 * omap wkup_m3: platform data
 *
 * Copyright (C) 2014 Texas Instruments, Inc.
 *
 * Dave Gerlach <d-gerlach@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _LINUX_PLATFORM_DATA_WKUP_M3_H
#define _LINUX_PLATFORM_DATA_WKUP_M3_H

struct wkup_m3_platform_data {
	const char *reset_name;

	int (*assert_reset)(struct platform_device *pdev, const char *name);
	int (*deassert_reset)(struct platform_device *pdev, const char *name);
};

#endif /* _LINUX_PLATFORM_DATA_WKUP_M3_H */
