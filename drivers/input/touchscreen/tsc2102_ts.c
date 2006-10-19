/*
 * input/touchscreen/tsc2102_ts.c
 *
 * Touchscreen input device driver for the TSC 2102 chip.
 *
 * Copyright (c) 2006 Andrzej Zaborowski  <balrog@zabor.org>
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

#include <linux/spi/tsc2102.h>

struct input_dev *dev;

static void tsc2102_touch(int touching)
{
	if (!touching) {
		input_report_abs(dev, ABS_X, 0);
		input_report_abs(dev, ABS_Y, 0);
		input_report_abs(dev, ABS_PRESSURE, 0);
		input_sync(dev);
	}

	input_report_key(dev, BTN_TOUCH, touching);
}

static void tsc2102_coords(int x, int y, int z1, int z2)
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

static int tsc2102_ts_probe(struct platform_device *pdev)
{
	int status;

	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;

	status = tsc2102_touch_cb(tsc2102_touch);
	if (status) {
		input_free_device(dev);
		return status;
	}

	status = tsc2102_coords_cb(tsc2102_coords);
	if (status) {
		tsc2102_touch_cb(0);
		input_free_device(dev);
		return status;
	}

	dev->name = "TSC2102 Touchscreen";
	dev->dev = &pdev->dev;
	dev->cdev.dev = &pdev->dev;
	dev->evbit[0] = BIT(EV_KEY) | BIT(EV_ABS);
	dev->keybit[LONG(BTN_TOUCH)] |= BIT(BTN_TOUCH);
	dev->absbit[0] = BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE);
	dev->phys = "tsc2102/input0";
	dev->id.bustype = BUS_HOST;
	dev->id.vendor = 0x0001;
	dev->id.product = 0x2102;
	dev->id.version = 0x0001;

	status = input_register_device(dev);
	if (status) {
		tsc2102_coords_cb(0);
		tsc2102_touch_cb(0);
		input_free_device(dev);
		return status;
	}

	printk(KERN_INFO "TSC2102 touchscreen driver initialized\n");
	return 0;
}

static int tsc2102_ts_remove(struct platform_device *pdev)
{
	tsc2102_touch_cb(0);
	tsc2102_coords_cb(0);
	input_unregister_device(dev);
	input_free_device(dev);
	return 0;
}

#ifdef CONFIG_PM
static int
tsc2102_ts_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int tsc2102_ts_resume(struct platform_device *pdev)
{
	return 0;
}
#else
#define tsc2102_ts_suspend	NULL
#define tsc2102_ts_resume	NULL
#endif

static struct platform_driver tsc2102_ts_driver = {
	.probe 		= tsc2102_ts_probe,
	.remove 	= tsc2102_ts_remove,
	.suspend 	= tsc2102_ts_suspend,
	.resume 	= tsc2102_ts_resume,
	.driver		= {
		.name	= "tsc2102-ts",
		.owner	= THIS_MODULE,
	},
};

static int __init tsc2102_ts_init(void)
{
	int ret;

	ret = platform_driver_register(&tsc2102_ts_driver);
	if (ret)
		return -ENODEV;

	return 0;
}

static void __exit tsc2102_ts_exit(void)
{
	platform_driver_unregister(&tsc2102_ts_driver);
}

module_init(tsc2102_ts_init);
module_exit(tsc2102_ts_exit);

MODULE_AUTHOR("Andrzej Zaborowski");
MODULE_DESCRIPTION("Touchscreen input driver for TI TSC2102.");
MODULE_LICENSE("GPL");
