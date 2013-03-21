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

struct iio_helper_info;

struct iio_helper_chan {
	struct iio_helper_info *info;
	int index;
	const char *vsense_name;
	unsigned int vsense_scale;
	struct iio_channel *vsense_channel;
	struct device_attribute attr;
	unsigned int attr_created : 1;
};

#define iio_helper_attr_to_chan(attr) \
	container_of((attr), struct iio_helper_chan, attr)

struct iio_helper_info {
	struct platform_device *pdev;
	int channel_count;
	struct iio_helper_chan *channel;
	u32 *scale;
};

static ssize_t iio_helper_show_mvolts(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iio_helper_chan *chan = iio_helper_attr_to_chan(attr);
	int ret, val;
	u32 mvolts;

	ret = iio_read_channel_raw(chan->vsense_channel, &val);
	if (ret < 0)
		return ret;

	/* volts = ((1800 / 4096) * val * scale) = (1.8 * val * scale /4096) */
	mvolts = div_u64(1800 * chan->vsense_scale * (u64)val, 4096 * 100);

	return sprintf(buf, "%d\n", mvolts);
}

static int bone_iio_helper_probe(struct platform_device *pdev)
{
	struct iio_helper_info *info;
	struct iio_helper_chan *chan;
	struct device_node *pnode = pdev->dev.of_node;
	int i, err;

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

	err = of_property_count_strings(pnode, "vsense-name");
	if (err < 0) {
		dev_err(&pdev->dev, "Failed on vsense-name string property\n");
		goto err_no_vsense;
	}
	if (err == 0) {
		dev_err(&pdev->dev, "vsense-name property is empty\n");
		err = -EINVAL;
		goto err_no_vsense;
	}
	info->channel_count = err;
	info->channel = devm_kzalloc(&pdev->dev,
		info->channel_count * sizeof(*info->channel), GFP_KERNEL);
	if (info->channel == NULL) {
		dev_err(&pdev->dev, "Failed to allocate channel array\n");
		err = -ENOMEM;
		goto err_no_chan;
	}

	info->scale = devm_kzalloc(&pdev->dev,
		info->channel_count * sizeof(*info->scale), GFP_KERNEL);
	if (info->scale == NULL) {
		dev_err(&pdev->dev, "Failed to allocate scale array\n");
		err = -ENOMEM;
		goto err_no_scale;
	}

	err = of_property_read_u32_array(pnode, "vsense-scale", info->scale,
			info->channel_count);
	if (err != 0) {
		dev_err(&pdev->dev, "Failed to read vsense-scale array\n");
		goto err_bad_scale;
	}

	for (i = 0; i < info->channel_count; i++) {

		chan = &info->channel[i];

		err = of_property_read_string_index(pnode, "vsense-name", i,
				&chan->vsense_name);
		if (err != 0) {
			dev_err(&pdev->dev, "Error on vsense-name #%d\n", i);
			goto err_bad_chan;
		}

		chan->vsense_scale = info->scale[i];
		if (chan->vsense_scale == 0) {
			dev_err(&pdev->dev, "vsense-sense #%d zero\n", i);
			err = -EINVAL;
			goto err_bad_chan;
		}

		chan->vsense_channel = iio_channel_get(NULL, chan->vsense_name);
		if (IS_ERR(chan->vsense_channel)) {
			dev_err(&pdev->dev, "Could not get %s analog input\n",
						chan->vsense_name);
			err = PTR_ERR(chan->vsense_channel);
			chan->vsense_channel = NULL;
			goto err_bad_chan;
		}

		chan->attr.attr.name = chan->vsense_name;
		chan->attr.attr.mode = S_IRUGO;
		chan->attr.show = iio_helper_show_mvolts;
		chan->attr.store = NULL;

		err = device_create_file(&pdev->dev, &chan->attr);
		if (err != 0) {
			dev_err(&pdev->dev, "Could not create %s attr\n",
						chan->vsense_name);
			goto err_bad_chan;
		}
		chan->attr_created = 1;
	}

	dev_info(&pdev->dev, "ready\n");

	return 0;
err_bad_chan:
	/* cleanup possible channels allocated */
	for (i = info->channel_count - 1; i >= 0; i--) {
		chan = &info->channel[i];
		if (chan->attr_created)
			device_remove_file(&pdev->dev, &chan->attr);
		if (chan->vsense_channel != NULL)
			iio_channel_release(chan->vsense_channel);
	}
err_bad_scale:
	/* nothing */
err_no_scale:
	/* nothing */
err_no_vsense:
	/* nothing */
err_no_chan:
	/* nothing */
err_no_mem:
	return err;
}


static int bone_iio_helper_remove(struct platform_device *pdev)
{
	struct iio_helper_info *info = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	struct iio_helper_chan *chan;
	int i;

	for (i = info->channel_count - 1; i >= 0; i--) {
		chan = &info->channel[i];
		device_remove_file(&pdev->dev, &chan->attr);
		iio_channel_release(chan->vsense_channel);
	}
	devm_kfree(dev, info->scale);
	devm_kfree(dev, info->channel);
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
