/*
 * Platform data for PRUSS on TI SoCs
 *
 * Copyright (C) 2014-2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _PLAT_REMOTEPROC_PRUSS_H
#define _PLAT_REMOTEPROC_PRUSS_H

struct platform_device;

/**
 * struct pruss_platform_data - PRUSS remoteproc's platform data
 * @reset_name: name of the reset
 * @assert_reset: PRU-specific handler for putting the device in reset
 * @deassert_reset: PRU-specific handler for releasing the device from reset
 */
struct pruss_platform_data {
	const char *reset_name;
	int (*assert_reset)(struct platform_device *pdev, const char *name);
	int (*deassert_reset)(struct platform_device *pdev, const char *name);
};

#endif /* _PLAT_REMOTEPROC_PRUSS_H */
