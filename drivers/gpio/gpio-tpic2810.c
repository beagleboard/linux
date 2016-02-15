/*
 * Copyright (C) 2015-2016 Texas Instruments Incorporated - http://www.ti.com/
 *	Andrew F. Davis <afd@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether expressed or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License version 2 for more details.
 */

#include <linux/gpio/driver.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>

#define TPIC2810_WS_COMMAND 0x44

/**
 * struct tpic2810 - GPIO driver data
 * @chip: GPIO controller chip
 * @client: I2C device pointer
 * @buffer: Buffer for device register
 * @lock: Protects write sequences
 */
struct tpic2810 {
	struct gpio_chip chip;
	struct regmap *regmap;
};

static inline struct tpic2810 *to_tpic2810(struct gpio_chip *chip)
{
	return container_of(chip, struct tpic2810, chip);
}

static int tpic2810_get_direction(struct gpio_chip *chip,
				  unsigned offset)
{
	/* This device always output */
	return 0;
}

static int tpic2810_direction_input(struct gpio_chip *chip,
				    unsigned offset)
{
	/* This device is output only */
	return -EINVAL;
}

static int tpic2810_direction_output(struct gpio_chip *chip,
				     unsigned offset, int value)
{
	/* This device always output */
	return 0;
}

static void tpic2810_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct tpic2810 *gpio = to_tpic2810(chip);
	int ret;

	ret = regmap_update_bits(gpio->regmap, TPIC2810_WS_COMMAND, BIT(offset),
				 value ? BIT(offset) : 0x0);
	if (ret)
		dev_err(chip->dev, "Unable to set pin\n");
}

static void tpic2810_set_multiple(struct gpio_chip *chip, unsigned long *mask,
				  unsigned long *bits)
{
	struct tpic2810 *gpio = to_tpic2810(chip);
	int ret;

	ret = regmap_update_bits(gpio->regmap, TPIC2810_WS_COMMAND,
				 *mask, *bits);
	if (ret)
		dev_err(chip->dev, "Unable to set pins\n");
}

static struct gpio_chip template_chip = {
	.label			= "tpic2810",
	.owner			= THIS_MODULE,
	.get_direction		= tpic2810_get_direction,
	.direction_input	= tpic2810_direction_input,
	.direction_output	= tpic2810_direction_output,
	.set			= tpic2810_set,
	.set_multiple		= tpic2810_set_multiple,
	.base			= -1,
	.ngpio			= 8,
	.can_sleep		= true,
};

static const struct reg_default tpic2810_reg_defaults[] = {
	{ TPIC2810_WS_COMMAND, 0x0},
};

static const struct regmap_config tpic2810_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	/* set defaults and cache the register */
	.reg_defaults = tpic2810_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(tpic2810_reg_defaults),
	.cache_type = REGCACHE_RBTREE,
};

static const struct of_device_id tpic2810_of_match_table[] = {
	{ .compatible = "ti,tpic2810" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, tpic2810_of_match_table);

static int tpic2810_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct tpic2810 *gpio;
	int ret;

	gpio = devm_kzalloc(&client->dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	i2c_set_clientdata(client, gpio);

	gpio->chip = template_chip;
	gpio->chip.dev = &client->dev;

	gpio->regmap = devm_regmap_init_i2c(client, &tpic2810_regmap_config);
	if (IS_ERR(gpio->regmap)) {
		dev_err(&client->dev, "Unable to initialize regmap\n");
		return PTR_ERR(gpio->regmap);
	}

	ret = gpiochip_add(&gpio->chip);
	if (ret < 0) {
		dev_err(&client->dev, "Unable to register gpiochip\n");
		return ret;
	}

	return 0;
}

static int tpic2810_remove(struct i2c_client *client)
{
	struct tpic2810 *gpio = i2c_get_clientdata(client);

	gpiochip_remove(&gpio->chip);

	return 0;
}

static const struct i2c_device_id tpic2810_id_table[] = {
	{ "tpic2810", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, tpic2810_id_table);

static struct i2c_driver tpic2810_driver = {
	.driver = {
		.name = "tpic2810",
		.of_match_table = tpic2810_of_match_table,
	},
	.probe = tpic2810_probe,
	.remove = tpic2810_remove,
	.id_table = tpic2810_id_table,
};
module_i2c_driver(tpic2810_driver);

MODULE_AUTHOR("Andrew F. Davis <afd@ti.com>");
MODULE_DESCRIPTION("TPIC2810 8-Bit LED Driver GPIO Driver");
MODULE_LICENSE("GPL v2");
