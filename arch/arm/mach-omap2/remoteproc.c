/*
 * Remote processor machine-specific module for OMAP4+ SoCs
 *
 * Copyright (C) 2011-2017 Texas Instruments Incorporated - http://www.ti.com/
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
 */

#define pr_fmt(fmt)    "%s: " fmt, __func__

#include <linux/kernel.h>

#include "omap_device.h"
#include "remoteproc.h"

/**
 * omap_rproc_device_enable - enable the remoteproc device
 * @pdev: the rproc platform device
 *
 * This function performs the necessary low-level functions to enable
 * a remoteproc device to start executing. This typically includes
 * releasing the reset lines, and enabling the clocks for the device.
 * We do not usually expect this function to fail.
 *
 * Return: 0 on success, or the return code from the failed function
 */
int omap_rproc_device_enable(struct platform_device *pdev)
{
	int ret = -EINVAL;

	/*
	 * This reset management follows a device name check to differentiate
	 * DSP and IPU processor subsystems. This check is weak and is ok for
	 * now because of the dependencies against the pdata-quirks, where
	 * the devices are given specific device names that satisfy the
	 * criteria for the check. It can easily be replaced with a stronger
	 * check like device node compatibility check, if needed.
	 */
	if (strstr(dev_name(&pdev->dev), "dsp")) {
		ret = omap_device_deassert_hardreset(pdev, "dsp");
		if (ret)
			goto out;
	} else if (strstr(dev_name(&pdev->dev), "ipu")) {
		ret = omap_device_deassert_hardreset(pdev, "cpu0");
		if (ret)
			goto out;

		ret = omap_device_deassert_hardreset(pdev, "cpu1");
		if (ret)
			goto out;
	} else {
		pr_err("unsupported remoteproc\n");
		goto out;
	}

	ret = omap_device_enable(pdev);

out:
	if (ret)
		pr_err("failed for proc %s\n", dev_name(&pdev->dev));
	return ret;
}

/**
 * omap_rproc_device_shutdown - shutdown the remoteproc device
 * @pdev: the rproc platform device
 *
 * This function performs the necessary low-level functions to shutdown
 * a remoteproc device. This typically includes disabling the clocks
 * for the device and asserting the associated reset lines. We do not
 * usually expect this function to fail.
 *
 * Return: 0 on success, or the return code from the failed function
 */
int omap_rproc_device_shutdown(struct platform_device *pdev)
{
	int ret = -EINVAL;

	ret = omap_device_idle(pdev);
	if (ret)
		goto out;

	/*
	 * This reset management follows a device name check to differentiate
	 * DSP and IPU processor subsystems. This check is weak and is ok for
	 * now because of the dependencies against the pdata-quirks, where
	 * the devices are given specific device names that satisfy the
	 * criteria for the check. It can easily be replaced with a stronger
	 * check like device node compatibility check, if needed.
	 */
	if (strstr(dev_name(&pdev->dev), "dsp")) {
		ret = omap_device_assert_hardreset(pdev, "dsp");
	} else if (strstr(dev_name(&pdev->dev), "ipu")) {
		ret = omap_device_assert_hardreset(pdev, "cpu1");
		if (ret)
			goto out;

		ret = omap_device_assert_hardreset(pdev, "cpu0");
		if (ret)
			goto out;
	} else {
		pr_err("unsupported remoteproc\n");
	}

out:
	if (ret)
		pr_err("failed for proc %s\n", dev_name(&pdev->dev));
	return ret;
}
