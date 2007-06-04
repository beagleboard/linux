/*
 * input/touchscreen/tsc210x_ts.c
 *
 * Touchscreen input device driver for the TSC 2101/2102 chips.
 *
 * Copyright (c) 2006-2007 Andrzej Zaborowski  <balrog@zabor.org>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This package is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/platform_device.h>

#include <linux/spi/tsc210x.h>

static void tsc210x_touch(struct input_dev *dev, int touching)
{
	if (!touching) {
		input_report_abs(dev, ABS_X, 0);
		input_report_abs(dev, ABS_Y, 0);
		input_report_abs(dev, ABS_PRESSURE, 0);
		input_sync(dev);
	}

	input_report_key(dev, BTN_TOUCH, touching);
}

static void tsc210x_coords(struct input_dev *dev, int x, int y, int z1, int z2)
{
	int p;

	/* Calculate the touch resistance a la equation #1 */
	if (z1 != 0)
		p = x * (z2 - z1) / (z1 << 4);
	else
		p = 1;

	input_report_abs(dev, ABS_X, x);
	input_report_abs(dev, ABS_Y, y);
	input_report_abs(dev, ABS_PRESSURE, p);
	input_sync(dev);
}

static int tsc210x_ts_probe(struct platform_device *pdev)
{
	int status;
	struct input_dev *dev;

	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;

	status = tsc210x_touch_cb(pdev->dev.parent,
			(tsc210x_touch_t) tsc210x_touch, dev);
	if (status) {
		input_free_device(dev);
		return status;
	}

	status = tsc210x_coords_cb(pdev->dev.parent,
			(tsc210x_coords_t) tsc210x_coords, dev);
	if (status) {
		tsc210x_touch_cb(pdev->dev.parent, 0, 0);
		input_free_device(dev);
		return status;
	}

	dev->name = "TSC210x Touchscreen";
	dev->cdev.dev = &pdev->dev;
	dev->evbit[0] = BIT(EV_KEY) | BIT(EV_ABS);
	dev->keybit[LONG(BTN_TOUCH)] |= BIT(BTN_TOUCH);
	dev->absbit[0] = BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE);
	dev->phys = "tsc210x/input0";
	dev->id.bustype = BUS_HOST;
	dev->id.vendor = 0x0001;
	dev->id.product = 0x2100;
	dev->id.version = 0x0001;

	status = input_register_device(dev);
	if (status) {
		tsc210x_coords_cb(pdev->dev.parent, 0, 0);
		tsc210x_touch_cb(pdev->dev.parent, 0, 0);
		input_free_device(dev);
		return status;
	}

	platform_set_drvdata(pdev, dev);
	printk(KERN_INFO "TSC210x touchscreen initialised\n");
	return 0;
}

static int tsc210x_ts_remove(struct platform_device *pdev)
{
	struct input_dev *dev = (struct input_dev *)
		platform_get_drvdata(pdev);

	tsc210x_touch_cb(pdev->dev.parent, 0, 0);
	tsc210x_coords_cb(pdev->dev.parent, 0, 0);
	platform_set_drvdata(pdev, 0);
	input_unregister_device(dev);
	input_free_device(dev);

	return 0;
}

static struct platform_driver tsc210x_ts_driver = {
	.probe 		= tsc210x_ts_probe,
	.remove 	= tsc210x_ts_remove,
	/* Nothing to do on suspend/resume */
	.driver		= {
		.name	= "tsc210x-ts",
		.owner	= THIS_MODULE,
	},
};

static int __init tsc210x_ts_init(void)
{
	int ret;

	ret = platform_driver_register(&tsc210x_ts_driver);
	if (ret)
		return -ENODEV;

	return 0;
}

static void __exit tsc210x_ts_exit(void)
{
	platform_driver_unregister(&tsc210x_ts_driver);
}

module_init(tsc210x_ts_init);
module_exit(tsc210x_ts_exit);

MODULE_AUTHOR("Andrzej Zaborowski");
MODULE_DESCRIPTION("Touchscreen input driver for TI TSC2101/2102.");
MODULE_LICENSE("GPL");
