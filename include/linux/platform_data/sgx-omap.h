/* SPDX-License-Identifier: GPL-2.0 */
/*
 * SGX Graphics Driver Platform Data
 *
 * Copyright (C) 2014-2019 Texas Instruments Incorporated - http://www.ti.com/
 *	Darren Etheridge <detheridge@ti.com>
 *
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
