/*
 * ti81xxhdmi_tlc59108.c
 *
 * Copyright (C) 2011 Texas Instruments
 * Author: Senthil Natarajan
 *
 * tlc59108 HDMI Driver
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 * History:
 *
 * Senthil Natarajan<senthil.n@ti.com> July 2011 I2C driver for tlc59108
 *						 backlight control
 */

#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/backlight.h>
#include <linux/fb.h>

#define TLC59108_MODE1   0x00
#define TLC59108_MODE2   0x01
#define TLC59108_PWM0 0x02
#define TLC59108_PWM1 0x03
#define TLC59108_PWM2 0x04
#define TLC59108_PWM3 0x05
#define TLC59108_PWM4 0x06
#define TLC59108_PWM5 0x07
#define TLC59108_PWM6 0x08
#define TLC59108_PWM7 0x09
#define TLC59108_LEDOUT0 0x0c
#define TLC59108_LEDOUT1 0x0d
#define TLC59108_MAX_BRIGHTNESS 0xFF

struct tlc59108_bl {
	struct i2c_client *client;
	struct backlight_device *bl;
};

static void tlc59108_bl_set_backlight(struct tlc59108_bl *data, int brightness)
{
	/* Set Backlight Duty Cycle*/
	i2c_smbus_write_byte_data(data->client, TLC59108_PWM2,
				  0xff - brightness );
}

static int tlc59108_bl_get_brightness(struct backlight_device *dev)
{
	struct backlight_properties *props = &dev->props;

	return props->brightness;
}

static int tlc59108_bl_update_status(struct backlight_device *dev)
{
	struct backlight_properties *props = &dev->props;
	struct tlc59108_bl *data = dev_get_drvdata(&dev->dev);

	int brightness = props->brightness;

	if (dev->props.state & BL_CORE_FBBLANK) {
		brightness = 0;
		/* Set LEDOUT0 Register */
		i2c_smbus_write_byte_data(data->client, TLC59108_LEDOUT0, 0x10);
	} else {
		/* Set LEDOUT0 Register */
		i2c_smbus_write_byte_data(data->client, TLC59108_LEDOUT0, 0x30);
	}

	tlc59108_bl_set_backlight(data, brightness);

	return 0;
}

static const struct backlight_ops bl_ops = {
	.get_brightness		= tlc59108_bl_get_brightness,
	.update_status		= tlc59108_bl_update_status,
};

static int __devinit tlc59108_probe(struct i2c_client *c, const struct i2c_device_id *id)
{
	struct backlight_properties props;
	struct tlc59108_bl *data = kzalloc(sizeof(struct tlc59108_bl),
					   GFP_KERNEL);
	int ret = 0;

	if (!data)
		return -ENOMEM;

	i2c_set_clientdata(c, data);
	data->client = c;

	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = TLC59108_MAX_BRIGHTNESS;
	props.type = BACKLIGHT_RAW;
	data->bl = backlight_device_register("tlc59108-bl", &c->dev, data,
					     &bl_ops, &props);
	if (IS_ERR(data->bl)) {
		ret = PTR_ERR(data->bl);
		goto err_reg;
	}

	data->bl->props.brightness = TLC59108_MAX_BRIGHTNESS;

	backlight_update_status(data->bl);

	i2c_smbus_write_byte_data(data->client, TLC59108_MODE1, 0x00);
	i2c_smbus_write_byte_data(data->client, TLC59108_PWM2, 0x80);
	i2c_smbus_write_byte_data(data->client, TLC59108_LEDOUT1, 0x05);
	i2c_smbus_write_byte_data(data->client, TLC59108_LEDOUT1, 0x15);

	return 0;

err_reg:
	data->bl = NULL;
	kfree(data);
	return ret;
}

static int tlc59108_remove(struct i2c_client *c)
{
	struct tlc59108_bl *data = i2c_get_clientdata(c);

	backlight_device_unregister(data->bl);
	data->bl = NULL;

	kfree(data);

	return 0;
}

/* I2C Device ID table */
static struct i2c_device_id tlc59108_id[] = {
	{ "tlc59108", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tlc59108_id);

/* I2C driver data */
static struct i2c_driver tlc59108_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "tlc59108"
	},
	.id_table = tlc59108_id,
	.probe = tlc59108_probe,
	.remove = tlc59108_remove,
};

static int __init tlc59108_init(void)
{
	return i2c_add_driver(&tlc59108_driver);
}

static void __exit tlc59108_exit(void)
{
	i2c_del_driver(&tlc59108_driver);
}

module_init(tlc59108_init);
module_exit(tlc59108_exit);

MODULE_DESCRIPTION("LCD/Backlight control for TLC59108");
MODULE_AUTHOR("Senthil Natarajan <senthil.n@ti.com>");
MODULE_LICENSE("GPL");
