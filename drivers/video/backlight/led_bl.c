// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2015-2018 Texas Instruments Incorporated -  http://www.ti.com/
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
 *
 * Based on pwm_bl.c
 */

#include <linux/backlight.h>
#include <linux/gpio/consumer.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

struct led_bl_data {
	struct device		*dev;
	struct backlight_device	*bl_dev;

	unsigned int		*levels;
	bool			enabled;
	struct regulator	*power_supply;
	struct gpio_desc	*enable_gpio;

	struct led_classdev *led_cdev;

	unsigned int max_brightness;
	unsigned int default_brightness;
};

static void led_bl_set_brightness(struct led_bl_data *priv, int brightness)
{
	int err;

	if (!priv->enabled) {
		err = regulator_enable(priv->power_supply);
		if (err < 0)
			dev_err(priv->dev, "failed to enable power supply\n");

		if (priv->enable_gpio)
			gpiod_set_value_cansleep(priv->enable_gpio, 1);
	}

	led_set_brightness(priv->led_cdev, priv->levels[brightness]);

	priv->enabled = true;
}

static void led_bl_power_off(struct led_bl_data *priv)
{
	if (!priv->enabled)
		return;

	led_set_brightness(priv->led_cdev, LED_OFF);

	if (priv->enable_gpio)
		gpiod_set_value_cansleep(priv->enable_gpio, 0);

	regulator_disable(priv->power_supply);

	priv->enabled = false;
}

static int led_bl_update_status(struct backlight_device *bl)
{
	struct led_bl_data *priv = bl_get_data(bl);
	int brightness = bl->props.brightness;

	if (bl->props.power != FB_BLANK_UNBLANK ||
	    bl->props.fb_blank != FB_BLANK_UNBLANK ||
	    bl->props.state & BL_CORE_FBBLANK)
		brightness = 0;

	if (brightness > 0)
		led_bl_set_brightness(priv, brightness);
	else
		led_bl_power_off(priv);

	return 0;
}

static const struct backlight_ops led_bl_ops = {
	.update_status	= led_bl_update_status,
};

static int led_bl_parse_dt(struct device *dev,
			   struct led_bl_data *priv)
{
	struct device_node *node = dev->of_node;
	int num_levels;
	u32 *levels;
	u32 value;
	int ret;

	if (!node)
		return -ENODEV;

	num_levels = of_property_count_u32_elems(node, "brightness-levels");
	if (num_levels < 0)
		return num_levels;

	levels = devm_kzalloc(dev, sizeof(u32) * num_levels, GFP_KERNEL);
	if (!levels)
		return -ENOMEM;

	ret = of_property_read_u32_array(node, "brightness-levels",
					 levels,
					 num_levels);
	if (ret < 0)
		return ret;

	ret = of_property_read_u32(node, "default-brightness-level", &value);
	if (ret < 0)
		return ret;

	if (value >= num_levels) {
		dev_err(dev, "invalid default-brightness-level\n");
		return -EINVAL;
	}

	priv->levels = levels;
	priv->max_brightness = num_levels - 1;
	priv->default_brightness = value;

	priv->led_cdev = of_led_get(node);
	if (IS_ERR(priv->led_cdev))
		return PTR_ERR(priv->led_cdev);

	return 0;
}

static int led_bl_probe(struct platform_device *pdev)
{
	struct backlight_properties props;
	struct led_bl_data *priv;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);

	priv->dev = &pdev->dev;

	ret = led_bl_parse_dt(&pdev->dev, priv);
	if (ret < 0) {
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "failed to parse DT data\n");
		return ret;
	}

	priv->enable_gpio = devm_gpiod_get_optional(&pdev->dev, "enable",
			    GPIOD_OUT_LOW);
	if (IS_ERR(priv->enable_gpio)) {
		ret = PTR_ERR(priv->enable_gpio);
		goto err;
	}

	priv->power_supply = devm_regulator_get(&pdev->dev, "power");
	if (IS_ERR(priv->power_supply)) {
		ret = PTR_ERR(priv->power_supply);
		goto err;
	}

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = priv->max_brightness;
	priv->bl_dev = backlight_device_register(dev_name(&pdev->dev),
			&pdev->dev, priv, &led_bl_ops, &props);
	if (IS_ERR(priv->bl_dev)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		ret = PTR_ERR(priv->bl_dev);
		goto err;
	}

	priv->bl_dev->props.brightness = priv->default_brightness;
	backlight_update_status(priv->bl_dev);

	return 0;

err:
	if (priv->led_cdev)
		led_put(priv->led_cdev);

	return ret;
}

static int led_bl_remove(struct platform_device *pdev)
{
	struct led_bl_data *priv = platform_get_drvdata(pdev);
	struct backlight_device *bl = priv->bl_dev;

	backlight_device_unregister(bl);

	led_bl_power_off(priv);

	led_put(priv->led_cdev);

	return 0;
}

static const struct of_device_id led_bl_of_match[] = {
	{ .compatible = "led-backlight" },
	{ }
};

MODULE_DEVICE_TABLE(of, led_bl_of_match);

static struct platform_driver led_bl_driver = {
	.driver		= {
		.name		= "led-backlight",
		.of_match_table	= of_match_ptr(led_bl_of_match),
	},
	.probe		= led_bl_probe,
	.remove		= led_bl_remove,
};

module_platform_driver(led_bl_driver);

MODULE_DESCRIPTION("LED based Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:led-backlight");
