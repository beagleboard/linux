// SPDX-License-Identifier: GPL-2.0
/*
 * mikroBUS driver for adding mikrobus port from device tree
 *
 * Copyright 2020 Vaishnav M A, BeagleBoard.org Foundation.
 */
#define pr_fmt(fmt) "mikrobus_port:%s: " fmt, __func__

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/jump_label.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/pwm.h>
#include <linux/spi/spi.h>
#include <linux/serdev.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/consumer.h>

#include "mikrobus_core.h"

static int mikrobus_port_probe_pinctrl_setup(struct mikrobus_port *port)
{
	struct pinctrl_state *state;
	struct device *dev = port->dev.parent;
	int retval, i;

	state = pinctrl_lookup_state(port->pinctrl, PINCTRL_STATE_DEFAULT);
	if (!IS_ERR(state)) {
		retval = pinctrl_select_state(port->pinctrl, state);
		if (retval != 0) {
			dev_err(dev, "Failed to select state %s\n",
				PINCTRL_STATE_DEFAULT);
			return retval;
		}
	} else {
		dev_err(dev, "failed to find state %s\n",
			PINCTRL_STATE_DEFAULT);
		return PTR_ERR(state);
	}

	for (i = 0; i < MIKROBUS_NUM_PINCTRL_STATE; i++) {
		port->pinctrl_selected[i] =
				kmalloc(MIKROBUS_PINCTRL_NAME_SIZE, GFP_KERNEL);
		sprintf(port->pinctrl_selected[i], "%s_%s",
			MIKROBUS_PINCTRL_STR[i], PINCTRL_STATE_DEFAULT);
	}

	retval = mikrobus_port_pinctrl_select(port);
	if (retval)
		dev_err(dev, "failed to select pinctrl states [%d]", retval);
	return retval;
}

static int mikrobus_port_probe(struct platform_device *pdev)
{
	struct mikrobus_port *port;
	struct device *dev = &pdev->dev;
	struct device_node *i2c_adap_np;
	int retval;
	u32 val;

	port = kzalloc(sizeof(*port), GFP_KERNEL);
	if (!port)
		return -ENOMEM;

	i2c_adap_np = of_parse_phandle(dev->of_node, "i2c-adapter", 0);
	if (!i2c_adap_np) {
		dev_err(dev, "cannot parse i2c-adapter\n");
		retval = -ENODEV;
		goto err_port;
	}
	port->i2c_adap = of_find_i2c_adapter_by_node(i2c_adap_np);
	of_node_put(i2c_adap_np);
	retval = device_property_read_u32(dev, "spi-master", &val);
	if (retval) {
		dev_err(dev, "failed to get spi-master [%d]\n", retval);
		goto err_port;
	}
	port->spi_mstr = spi_busnum_to_master(val);
	retval = device_property_read_u32_array(dev, "spi-cs",
						port->chip_select, 2);
	if (retval) {
		dev_err(dev, "failed to get spi-cs [%d]\n", retval);
		goto err_port;
	}
	//port->pwm = devm_pwm_get(dev, NULL);
	//if (IS_ERR(port->pwm)) {
	//	retval = PTR_ERR(port->pwm);
	//	if (retval != -EPROBE_DEFER)
	//		dev_err(dev, "failed to request PWM device [%d]\n",
	//			retval);
	//	goto err_port;
	//}
	port->gpios = gpiod_get_array(dev, "mikrobus", GPIOD_OUT_LOW);
	if (IS_ERR(port->gpios)) {
		retval = PTR_ERR(port->gpios);
		dev_err(dev, "failed to get gpio array [%d]\n", retval);
		goto err_port;
	}
	port->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(port->pinctrl)) {
		retval = PTR_ERR(port->pinctrl);
		dev_err(dev, "failed to get pinctrl [%d]\n", retval);
		goto err_port;
	}
	port->dev.parent = dev;
	port->dev.of_node = pdev->dev.of_node;

	retval = mikrobus_port_probe_pinctrl_setup(port);
	if (retval) {
		dev_err(dev, "failed to setup pinctrl [%d]\n", retval);
		goto err_port;
	}

	retval = mikrobus_port_register(port);
	if (retval) {
		pr_err("port : can't register port [%d]\n", retval);
		goto err_port;
	}
	platform_set_drvdata(pdev, port);
	return 0;
err_port:
	kfree(port);
	return retval;
}

static int mikrobus_port_remove(struct platform_device *pdev)
{
	struct mikrobus_port	*port = platform_get_drvdata(pdev);

	mikrobus_port_delete(port);
	return 0;
}

static const struct of_device_id mikrobus_port_of_match[] = {
	{.compatible = "linux,mikrobus"},
	{},
};
MODULE_DEVICE_TABLE(of, mikrobus_port_of_match);

static struct platform_driver mikrobus_port_driver = {
	.probe = mikrobus_port_probe,
	.remove = mikrobus_port_remove,
	.driver = {
		.name = "mikrobus",
		.of_match_table = of_match_ptr(mikrobus_port_of_match),
	},
};

static int __init
mikrobus_port_init_driver(void)
{
	int retval;

	retval = platform_driver_register(&mikrobus_port_driver);
	if (retval)
		pr_err("driver register failed [%d]\n", retval);
	return retval;
}
subsys_initcall(mikrobus_port_init_driver);

static void __exit mikrobus_port_exit_driver(void)
{
	platform_driver_unregister(&mikrobus_port_driver);
}
module_exit(mikrobus_port_exit_driver);

MODULE_AUTHOR("Vaishnav M A <vaishnav@beagleboard.org>");
MODULE_DESCRIPTION("mikroBUS port module");
MODULE_LICENSE("GPL");
