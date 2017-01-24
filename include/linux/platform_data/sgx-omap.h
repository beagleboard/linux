/*
 * SGX Graphics Driver Platform Data
 *
 * Copyright (C) 2014-2015 Texas Instruments Incorporated - http://www.ti.com/
 *	Darren Etheridge <detheridge@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __SGX_OMAP_H__
#define __SGX_OMAP_H__

#include <linux/platform_device.h>

struct gfx_sgx_platform_data {
	const char *reset_name;

	int (*assert_reset)(struct platform_device *pdev, const char *name);
	int (*deassert_reset)(struct platform_device *pdev, const char *name);
};

#endif
