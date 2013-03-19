/*
 * Industrial IO helper driver
 *
 * Copyright (C) 2013 Matt Ranostay <mranostay@gmail.com>
 *
 * Based on original work by
 *  Copyright (C) 2012 Pantelis Antoniou <panto@antoniou-consulting.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/math64.h>
#include <linux/input/ti_am335x_tsc.h>
#include <linux/platform_data/ti_am335x_adc.h>
#include <linux/mfd/ti_am335x_tscadc.h>
#include <linux/iio/iio.h>
#include <linux/iio/machine.h>
#include <linux/iio/consumer.h>

static const struct of_device_id bone_iio_helper_of_match[] = {
	{
		.compatible = "bone-iio-helper",
	},
	{ },
};
MODULE_DEVICE_TABLE(of, bone_iio_helper_of_match);

struct iio_helper_info {
	struct platform_device *pdev;
	const char *vsense_name;
	unsigned int vsense_scale;
	struct iio_channel *vsense_channel;
};

static ssize_t iio_helper_show_mvolts(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct iio_helper_info *info = platform_get_drvdata(pdev);
	int ret, val;
	u32 mvolts;

	ret = iio_read_channel_raw(info->vsense_channel, &val);
	if (ret < 0)
		return ret;

	/* volts = ((1800 / 4096) * val * scale) = (1.8 * val * scale /4096) */
	mvolts = div_u64(1800 * info->vsense_scale * (u64)val, 4096 * 100);

	return sprintf(buf, "%d\n", mvolts);
}

static DEVICE_ATTR(mvolts, S_IRUGO,
		iio_helper_show_mvolts, NULL);

static int iio_helper_sysfs_register(struct platform_device *pdev)
{
	return device_create_file(&pdev->dev, &dev_attr_mvolts);
}

static int bone_iio_helper_probe(struct platform_device *pdev)
{
	struct iio_helper_info *info;
	struct device_node *pnode = pdev->dev.of_node;
	int err;

	/* we only support OF */
	if (pnode == NULL) {
		dev_err(&pdev->dev, "No platform of_node!\n");
		return -ENODEV;
	}

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (info == NULL) {
		dev_err(&pdev->dev, "Failed to allocate info\n");
		err = -ENOMEM;
		goto err_no_mem;
	}
	platform_set_drvdata(pdev, info);
	info->pdev = pdev;

	if (of_property_read_string(pnode, "vsense-name", &info->vsense_name) != 0) {
		dev_err(&pdev->dev, "Could not read vsense-name property.");
		err = -EINVAL;
		goto err_no_vsense;
	}

	if (of_property_read_u32(pnode, "vsense-scale", &info->vsense_scale) != 0) {
		dev_err(&pdev->dev, "Could not read vsense-scale property");
		err = -EINVAL;
		goto err_no_vsense;
	}

	if (info->vsense_scale == 0) {
		dev_err(&pdev->dev, "Invalid vsense-scale property");
		err = -EINVAL;
		goto err_no_vsense;
	}

	info->vsense_channel = iio_channel_get(NULL, info->vsense_name);
	if (IS_ERR(info->vsense_channel)) {
		dev_err(&pdev->dev, "Could not get %s analog input\n",
					info->vsense_name);
		err = PTR_ERR(info->vsense_channel);
		goto err_no_vsense;
	}

	err = iio_helper_sysfs_register(pdev);
	if (err != 0) {
		dev_err(&pdev->dev, "unable to register sysfs\n");
		goto err_no_sysfs;
	}

	dev_info(&pdev->dev, "ready\n");

	return 0;

err_no_sysfs:
	/* fall-through */
err_no_vsense:
	kfree(info);
err_no_mem:
	return err;
}


static int bone_iio_helper_remove(struct platform_device *pdev)
{
	struct iio_helper_info *info = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	iio_channel_release(info->vsense_channel);
	device_remove_file(dev, &dev_attr_mvolts);
	kfree(info->vsense_name);
	devm_kfree(dev, info);

	return 0;
}

struct platform_driver bone_iio_helper_driver = {
	.probe		= bone_iio_helper_probe,
	.remove		= bone_iio_helper_remove,
	.driver	= {
		.name		= "bone-iio-helper",
		.owner		= THIS_MODULE,
		.of_match_table	= bone_iio_helper_of_match,
	},
};

module_platform_driver(bone_iio_helper_driver);

MODULE_AUTHOR("Matt Ranostay");
MODULE_DESCRIPTION("Beaglebone IIO helper driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:bone-iio-helper");
