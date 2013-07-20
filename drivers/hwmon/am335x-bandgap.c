/*
 * Copyright (c) 2013 Jan Luebbe <j.luebbe@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 */

#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

#define DRV_NAME	"am335x-bandgap"

#define BANDGAP_CTRL			0x0
#define BANDGAP_CTRL_DTEMP_MASK		0x0000FF00
#define BANDGAP_CTRL_DTEMP_OFF		8
#define BANDGAP_CTRL_BGROFF		BIT(6)
#define BANDGAP_CTRL_SOC		BIT(4)
#define BANDGAP_CTRL_CLRZ		BIT(3) /* 0 = clear */
#define BANDGAP_CTRL_CONTCONV		BIT(2)
#define BANDGAP_CTRL_ECOZ		BIT(1)
#define BANDGAP_CTRL_TSHUT		BIT(0)

#define BANDGAP_TRIM			0x4
#define BANDGAP_TRIM_DTRBGAPC_MASK	0xFF000000
#define BANDGAP_TRIM_DTRBGAPC_OFF	24
#define BANDGAP_TRIM_DTRBGAPV_MASK	0x00FF0000
#define BANDGAP_TRIM_DTRBGAPV_OFF	16
#define BANDGAP_TRIM_DTRTEMPS_MASK	0x0000FF00
#define BANDGAP_TRIM_DTRTEMPS_OFF	8
#define BANDGAP_TRIM_DTRTEMPSC_MASK	0x000000FF
#define BANDGAP_TRIM_DTRTEMPSC_OFF	0

struct am335x_bandgap {
	u32 __iomem *regs;
	struct device *hwmon_dev;
};

static ssize_t show_name(struct device *dev, struct device_attribute
			 *devattr, char *buf)
{
	return sprintf(buf, "%s\n", DRV_NAME);
}

static ssize_t show_input(struct device *dev,
			  struct device_attribute *devattr, char *buf)
{
	struct am335x_bandgap *data = dev_get_drvdata(dev);
	u32 val, temp;

	/* read measurement */
	val = readl(data->regs + BANDGAP_CTRL);

	/* compute temperature */
	val = (val & BANDGAP_CTRL_DTEMP_MASK) >> BANDGAP_CTRL_DTEMP_OFF;
	temp = val * 1000;

	return sprintf(buf, "%d\n", temp);
}

static SENSOR_DEVICE_ATTR(name, S_IRUGO, show_name, NULL, 0);
static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, show_input, NULL, 0);

struct attribute *am335x_bandgap_attributes[] = {
	&sensor_dev_attr_name.dev_attr.attr,
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	NULL
};

static const struct attribute_group am335x_bandgap_group = {
	.attrs = am335x_bandgap_attributes,
};

static int am335x_bandgap_probe(struct platform_device *pdev)
{
	struct am335x_bandgap *data;
	struct resource *res;
	int err;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	data->regs = devm_request_and_ioremap(&pdev->dev, res);
	if (!data->regs)
		return -ENODEV;

	platform_set_drvdata(pdev, data);

	err = sysfs_create_group(&pdev->dev.kobj, &am335x_bandgap_group);
	if (err < 0) {
		dev_err(&pdev->dev, "Create sysfs group failed (%d)\n", err);
		return err;
	}

	data->hwmon_dev = hwmon_device_register(&pdev->dev);
	if (IS_ERR(data->hwmon_dev)) {
		err = PTR_ERR(data->hwmon_dev);
		dev_err(&pdev->dev, "Class registration failed (%d)\n", err);
		goto exit_sysfs_group;
	}

	/* enable HW sensor */
	writel(BANDGAP_CTRL_SOC | BANDGAP_CTRL_CLRZ | BANDGAP_CTRL_CONTCONV,
		data->regs + BANDGAP_CTRL);

	return 0;

exit_sysfs_group:
	sysfs_remove_group(&pdev->dev.kobj, &am335x_bandgap_group);
	return err;
}

static int am335x_bandgap_remove(struct platform_device *pdev)
{
	struct am335x_bandgap *data = platform_get_drvdata(pdev);

	/* disable HW sensor */
	writel(0x0, data->regs + BANDGAP_CTRL);

	hwmon_device_unregister(data->hwmon_dev);
	sysfs_remove_group(&pdev->dev.kobj, &am335x_bandgap_group);

	return 0;
}

static const struct of_device_id am335x_bandgap_match[] = {
	{ .compatible = "ti,am335x-bandgap" },
	{},
};

static struct platform_driver am335x_bandgap_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = DRV_NAME,
		.of_match_table = of_match_ptr(am335x_bandgap_match),
	},
	.probe	= am335x_bandgap_probe,
	.remove	= am335x_bandgap_remove,
};

module_platform_driver(am335x_bandgap_driver);

MODULE_AUTHOR("Jan Luebbe <j.luebbe@pengutronix.de>");
MODULE_DESCRIPTION("AM335x temperature sensor driver");
MODULE_LICENSE("GPL");
