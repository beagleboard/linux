/*
 * omap iommu: main structures
 *
 * Copyright (C) 2008-2009 Nokia Corporation
 *
 * Written by Hiroshi DOYU <Hiroshi.DOYU@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/platform_device.h>

struct iommu_platform_data {
	const char *reset_name;
	int (*assert_reset)(struct platform_device *pdev, const char *name);
	int (*deassert_reset)(struct platform_device *pdev, const char *name);
	int (*device_enable)(struct platform_device *pdev);
	bool (*device_is_enabled)(struct platform_device *pdev);
	int (*device_idle)(struct platform_device *pdev);
	int (*set_pwrdm_constraint)(struct platform_device *pdev, bool request,
				    u8 *pwrst);
};
