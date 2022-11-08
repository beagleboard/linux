// SPDX-License-Identifier: GPL-2.0
/*
 * GPIO driver for TI TPS6594x PMICs
 *
 * Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com/
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <linux/gpio/driver.h>
#include <linux/mfd/tps6594x.h>

#define GPIO_CFG_MASK	BIT(0)
#define NGPIOS_PER_REG	8

struct tps6594x_gpio {
	struct gpio_chip gpio_chip;
	struct tps6594x *tps;
};

static int tps6594x_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	struct tps6594x_gpio *gpio = gpiochip_get_data(gc);
	int ret, val;

	ret = regmap_read(gpio->tps->regmap, TPS6594X_GPIO1_CONF + offset, &val);
	if (ret)
		return ret;

	if (val & GPIO_CFG_MASK)
		return GPIO_LINE_DIRECTION_OUT;

	return GPIO_LINE_DIRECTION_IN;
}

static int tps6594x_gpio_direction_input(struct gpio_chip *gc, unsigned int offset)
{
	struct tps6594x_gpio *gpio = gpiochip_get_data(gc);

	return regmap_update_bits(gpio->tps->regmap, TPS6594X_GPIO1_CONF + offset,
				  GPIO_CFG_MASK, 0);
}

static int tps6594x_gpio_direction_output(struct gpio_chip *gc, unsigned int offset, int value)
{
	struct tps6594x_gpio *gpio = gpiochip_get_data(gc);
	unsigned int reg = TPS6594X_GPIO_OUT_1, shift = offset;

	if (shift >= NGPIOS_PER_REG) {
		reg = TPS6594X_GPIO_OUT_2;
		shift -= NGPIOS_PER_REG;
	}

	regmap_update_bits(gpio->tps->regmap, reg, BIT(shift), value ? BIT(shift) : 0);

	return regmap_update_bits(gpio->tps->regmap, TPS6594X_GPIO1_CONF + offset,
				  GPIO_CFG_MASK, GPIO_CFG_MASK);
}

static int tps6594x_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct tps6594x_gpio *gpio = gpiochip_get_data(gc);
	unsigned int reg = TPS6594X_GPIO_IN_1;
	int ret, val;

	if (offset >= NGPIOS_PER_REG) {
		reg = TPS6594X_GPIO_IN_2;
		offset -= NGPIOS_PER_REG;
	}

	ret = regmap_read(gpio->tps->regmap, reg, &val);
	if (ret)
		return ret;

	return !!(val & BIT(offset));
}

static void tps6594x_gpio_set(struct gpio_chip *gc, unsigned int offset,
			      int value)
{
	struct tps6594x_gpio *gpio = gpiochip_get_data(gc);
	unsigned int reg = TPS6594X_GPIO_OUT_1;

	if (offset >= NGPIOS_PER_REG) {
		reg = TPS6594X_GPIO_OUT_2;
		offset -= NGPIOS_PER_REG;
	}

	regmap_update_bits(gpio->tps->regmap, reg, BIT(offset), value ? BIT(offset) : 0);
}

static const struct gpio_chip template_chip = {
	.label			= "tps6594x-gpio",
	.owner			= THIS_MODULE,
	.get_direction		= tps6594x_gpio_get_direction,
	.direction_input	= tps6594x_gpio_direction_input,
	.direction_output	= tps6594x_gpio_direction_output,
	.get			= tps6594x_gpio_get,
	.set			= tps6594x_gpio_set,
	.base			= -1,
	.ngpio			= 11,
	.can_sleep		= true,
};

static int tps6594x_gpio_probe(struct platform_device *pdev)
{
	struct tps6594x *tps = dev_get_drvdata(pdev->dev.parent);
	struct tps6594x_gpio *gpio;

	gpio = devm_kzalloc(&pdev->dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	gpio->tps = dev_get_drvdata(pdev->dev.parent);
	gpio->gpio_chip = template_chip;
	gpio->gpio_chip.parent = tps->dev;

	return devm_gpiochip_add_data(&pdev->dev, &gpio->gpio_chip, gpio);
}

static const struct of_device_id of_tps6594x_gpio_match[] = {
	{ .compatible = "ti,tps6594x-gpio", },
	{},
};
MODULE_DEVICE_TABLE(of, of_tps6594x_gpio_match);

static struct platform_driver tps6594x_gpio_driver = {
	.driver = {
		.name = "tps6594x-gpio",
		.of_match_table = of_match_ptr(of_tps6594x_gpio_match),
	},
	.probe = tps6594x_gpio_probe,
};
module_platform_driver(tps6594x_gpio_driver);

MODULE_ALIAS("platform:tps6594x-gpio");
MODULE_AUTHOR("Matt Ranostay <mranostay@ti.com>");
MODULE_DESCRIPTION("TPS6594X GPIO driver");
MODULE_LICENSE("GPL");
