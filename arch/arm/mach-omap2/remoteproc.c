// SPDX-License-Identifier: GPL-2.0
/*
 * Remote processor machine-specific module for OMAP4+ SoCs
 *
 * Copyright (C) 2011-2019 Texas Instruments Incorporated - http://www.ti.com/
 *      Suman Anna <s-anna@ti.com>
 */

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
	struct omap_device *od = to_omap_device(pdev);

	if (!od) {
		dev_err(&pdev->dev, "device does not have a backing omap_device\n");
		goto out;
	}

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
		dev_err(&pdev->dev, "unsupported remoteproc\n");
		goto out;
	}

	ret = omap_device_enable(pdev);

out:
	if (ret)
		dev_err(&pdev->dev, "%s failed, ret = %d\n", __func__, ret);
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
	struct omap_device *od = to_omap_device(pdev);

	if (!od) {
		dev_err(&pdev->dev, "device does not have a backing omap_device\n");
		goto out;
	}

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
		dev_err(&pdev->dev, "unsupported remoteproc\n");
	}

out:
	if (ret)
		dev_err(&pdev->dev, "%s failed, ret = %d\n", __func__, ret);
	return ret;
}
