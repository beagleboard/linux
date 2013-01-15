/*
 * omap-control-usb.c - The USB part of control module.
 *
 * Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Author: Kishon Vijay Abraham I <kishon@ti.com>
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/usb/omap_control_usb.h>

static struct omap_control_usb *control_usb;

/**
 * get_omap_control_dev - returns the device pointer for this control device
 *
 * This API should be called to get the device pointer for this control
 * module device. This device pointer should be passed to all other API's
 * in this driver.
 *
 * To be used by PHY driver and glue driver
 */
struct device *get_omap_control_dev(void)
{
	if (!control_usb)
		return ERR_PTR(-ENODEV);

	return control_usb->dev;
}
EXPORT_SYMBOL_GPL(get_omap_control_dev);

/**
 * omap_control_usb_phy_power - power on/off the phy using control module reg
 * @dev: the control module device
 * @on: 0 or 1, based on powering on or off the PHY
 */
void omap_control_usb_phy_power(struct device *dev, int on)
{
	u32 val;
	struct omap_control_usb	*control_usb = dev_get_drvdata(dev);

	if (on) {
		val = readl(control_usb->dev_conf);
		if (val & PHY_PD)
			writel(~PHY_PD, control_usb->dev_conf);
	} else {
		writel(PHY_PD, control_usb->dev_conf);
	}
}
EXPORT_SYMBOL_GPL(omap_control_usb_phy_power);

/**
 * omap_control_usb_host_mode - set AVALID, VBUSVALID and ID pin in grounded
 * @dev: struct device *
 *
 * This is an API to write to the mailbox register to notify the usb core that
 * a usb device has been connected.
 */
void omap_control_usb_host_mode(struct device *dev)
{
	u32 val;
	struct omap_control_usb	*control_usb = dev_get_drvdata(dev);

	val = AVALID | VBUSVALID;

	writel(val, control_usb->otghs_control);
}
EXPORT_SYMBOL_GPL(omap_control_usb_host_mode);

/**
 * omap_control_usb_device_mode - set AVALID, VBUSVALID and ID pin in high
 * impedance
 * @dev: struct device *
 *
 * This is an API to write to the mailbox register to notify the usb core that
 * it has been connected to a usb host.
 */
void omap_control_usb_device_mode(struct device *dev)
{
	u32 val;
	struct omap_control_usb	*control_usb = dev_get_drvdata(dev);

	val = IDDIG | AVALID | VBUSVALID;

	writel(val, control_usb->otghs_control);
}
EXPORT_SYMBOL_GPL(omap_control_usb_device_mode);

/**
 * omap_control_usb_set_sessionend - Enable SESSIONEND and IDIG to high
 * impedance
 * @dev: struct device *
 *
 * This is an API to write to the mailbox register to notify the usb core
 * it's now in disconnected state.
 */
void omap_control_usb_set_sessionend(struct device *dev)
{
	u32 val;
	struct omap_control_usb	*control_usb = dev_get_drvdata(dev);

	val = SESSEND | IDDIG;

	writel(val, control_usb->otghs_control);
}
EXPORT_SYMBOL_GPL(omap_control_usb_set_sessionend);

static int omap_control_usb_probe(struct platform_device *pdev)
{
	struct resource	*res;
	struct device_node *np = pdev->dev.of_node;
	struct omap_control_usb_platform_data *pdata = pdev->dev.platform_data;

	control_usb = devm_kzalloc(&pdev->dev, sizeof(*control_usb),
	    GFP_KERNEL);
	if (!control_usb) {
		dev_err(&pdev->dev, "unable to alloc memory for control usb\n");
		return -ENOMEM;
	}

	if (np) {
		control_usb->has_mailbox = of_property_read_bool(np,
		    "ti,has_mailbox");
	} else if (pdata) {
		control_usb->has_mailbox = pdata->has_mailbox;
	} else {
		dev_err(&pdev->dev, "no pdata present\n");
		return -EINVAL;
	}

	control_usb->dev	= &pdev->dev;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
	    "control_dev_conf");
	control_usb->dev_conf = devm_request_and_ioremap(&pdev->dev, res);
	if (control_usb->dev_conf == NULL) {
		dev_err(&pdev->dev, "Failed to obtain io memory\n");
		return -ENXIO;
	}

	if (control_usb->has_mailbox) {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
		    "otghs_control");
		control_usb->otghs_control = devm_request_and_ioremap(
		    &pdev->dev, res);
		if (control_usb->otghs_control == NULL) {
			dev_err(&pdev->dev, "Failed to obtain io memory\n");
			return -ENXIO;
		}
	}

	dev_set_drvdata(control_usb->dev, control_usb);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id omap_control_usb_id_table[] = {
	{ .compatible = "ti,omap-control-usb" },
	{}
};
MODULE_DEVICE_TABLE(of, omap_control_usb_id_table);
#endif

static struct platform_driver omap_control_usb_driver = {
	.probe		= omap_control_usb_probe,
	.driver		= {
		.name	= "omap-control-usb",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(omap_control_usb_id_table),
	},
};

static int __init omap_control_usb_init(void)
{
	return platform_driver_register(&omap_control_usb_driver);
}
subsys_initcall(omap_control_usb_init);

static void __exit omap_control_usb_exit(void)
{
	platform_driver_unregister(&omap_control_usb_driver);
}
module_exit(omap_control_usb_exit);

MODULE_ALIAS("platform: omap_control_usb");
MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_DESCRIPTION("OMAP CONTROL USB DRIVER");
MODULE_LICENSE("GPL v2");
