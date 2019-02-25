/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Platform data for PRUSS on TI SoCs
 *
 * Copyright (C) 2014-2019 Texas Instruments Incorporated - http://www.ti.com/
 */

#ifndef _PLAT_TI_PRUSS_H
#define _PLAT_TI_PRUSS_H

struct platform_device;

/**
 * struct pruss_platform_data - PRUSS platform data
 * @reset_name: name of the reset
 * @assert_reset: PRU-specific handler for putting the device in reset
 * @deassert_reset: PRU-specific handler for releasing the device from reset
 */
struct pruss_platform_data {
	const char *reset_name;
	int (*assert_reset)(struct platform_device *pdev, const char *name);
	int (*deassert_reset)(struct platform_device *pdev, const char *name);
};

#endif /* _PLAT_TI_PRUSS_H */
