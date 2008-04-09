/*
 * drivers/i2c/chips/lp5521.c
 *
 * Copyright (C) 2007 Nokia Corporation
 *
 * Written by Mathias Nyman <mathias.nyman@nokia.com>
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <asm/arch/gpio.h>

#define LP5521_DRIVER_NAME		"lp5521"

#ifdef LED_CONNECTED_WRONG
#define LP5521_REG_R_PWM		0x04
#define LP5521_REG_B_PWM		0x02
#else
#define LP5521_REG_R_PWM		0x02
#define LP5521_REG_B_PWM		0x04
#endif
#define LP5521_REG_ENABLE		0x00
#define LP5521_REG_OP_MODE		0x01
#define LP5521_REG_G_PWM		0x03
#define LP5521_REG_R_CNTRL		0x05
#define LP5521_REG_G_CNTRL		0x06
#define LP5521_REG_B_CNTRL		0x07
#define LP5521_REG_MISC			0x08
#define LP5521_REG_R_CHANNEL_PC		0x09
#define LP5521_REG_G_CHANNEL_PC		0x0a
#define LP5521_REG_B_CHANNEL_PC		0x0b
#define LP5521_REG_STATUS		0x0c
#define LP5521_REG_RESET		0x0d
#define LP5521_REG_GPO			0x0e
#define LP5521_REG_R_PROG_MEM		0x10
#define LP5521_REG_G_PROG_MEM		0x30
#define LP5521_REG_B_PROG_MEM		0x50

#define LP5521_MODE_LOAD		"load"
#define LP5521_MODE_RUN			"run"
#define LP5521_MODE_DIRECT_CONTROL	"direct"

#define LP5521_CURRENT_1m5		0x0f
#define LP5521_CURRENT_3m1		0x1f
#define LP5521_CURRENT_4m7		0x2f
#define LP5521_CURRENT_6m3		0x3f
#define LP5521_CURRENT_7m9		0x4f
#define LP5521_CURRENT_9m5		0x5f
#define LP5521_CURRENT_11m1		0x6f
#define LP5521_CURRENT_12m7		0x7f
#define LP5521_CURRENT_14m3		0x8f
#define LP5521_CURRENT_15m9		0x9f
#define LP5521_CURRENT_17m5		0xaf
#define LP5521_CURRENT_19m1		0xbf
#define LP5521_CURRENT_20m7		0xcf
#define LP5521_CURRENT_22m3		0xdf
#define LP5521_CURRENT_23m9		0xef
#define LP5521_CURRENT_25m5		0xff

#define LP5521_PROGRAM_LENGTH		32	/* in bytes */

struct lp5521_chip {
	struct mutex		lock;
	struct i2c_client	*client;
	char			*mode;
	int			red;
	int			green;
	int			blue;
};

static int lp5521_set_mode(struct lp5521_chip *chip, char *mode);

static int lp5521_write(struct i2c_client *client, u8 reg, u8 value)
{
	return i2c_smbus_write_byte_data(client, reg, value);
}

static int lp5521_read(struct i2c_client *client, u8 reg, u8 *buf)
{
	s32 ret = i2c_smbus_read_byte_data(client, reg);

	if (ret < 0)
		return -EIO;

	*buf = ret;
	return 0;
}

static int lp5521_configure(struct i2c_client *client)
{
	int ret = 0;

	/* Enable chip and set light to logarithmic mode*/
	ret |= lp5521_write(client, LP5521_REG_ENABLE, 0xc0);

	/* setting all color pwms to direct control mode */
	ret |= lp5521_write(client, LP5521_REG_OP_MODE, 0x3f);

	/* setting current to 4.7 mA for all channels */
	ret |= lp5521_write(client, LP5521_REG_R_CNTRL, LP5521_CURRENT_4m7);
	ret |= lp5521_write(client, LP5521_REG_G_CNTRL, LP5521_CURRENT_4m7);
	ret |= lp5521_write(client, LP5521_REG_B_CNTRL, LP5521_CURRENT_4m7);

	/* Enable auto-powersave, set charge pump to auto, red to battery */
	ret |= lp5521_write(client, LP5521_REG_MISC, 0x3c);

	/* initialize all channels pwm to zero */
	ret |= lp5521_write(client, LP5521_REG_R_PWM, 0);
	ret |= lp5521_write(client, LP5521_REG_G_PWM, 0);
	ret |= lp5521_write(client, LP5521_REG_B_PWM, 0);

	/* Not much can be done about errors at this point */
	return ret;
}

static int lp5521_load_program(struct lp5521_chip *chip, u8 *pattern)
{
	struct i2c_client *client = chip->client;
	int ret = 0;

	/* Enter load program mode for all led channels */
	ret |= lp5521_write(client, LP5521_REG_OP_MODE, 0x15); /* 0001 0101 */
	if (ret)
		return ret;

	if (chip->red)
		ret |= i2c_smbus_write_i2c_block_data(client,
						      LP5521_REG_R_PROG_MEM,
						      LP5521_PROGRAM_LENGTH,
						      pattern);
	if (chip->green)
		ret |= i2c_smbus_write_i2c_block_data(client,
						      LP5521_REG_G_PROG_MEM,
						      LP5521_PROGRAM_LENGTH,
						      pattern);
	if (chip->blue)
		ret |= i2c_smbus_write_i2c_block_data(client,
						      LP5521_REG_B_PROG_MEM,
						      LP5521_PROGRAM_LENGTH,
						      pattern);

	return ret;
}

static int lp5521_run_program(struct lp5521_chip *chip)
{
	struct i2c_client *client = chip->client;
	int ret;
	u8 mask = 0xc0;
	u8 exec_state = 0;
	u8 enable_reg;

	ret = lp5521_read(client, LP5521_REG_ENABLE, &enable_reg);
	if (ret)
		goto fail;

	enable_reg &= mask;

	/* set all active channels exec state to countinous run*/
	exec_state |= (chip->red   << 5);
	exec_state |= (chip->green << 3);
	exec_state |= (chip->blue  << 1);

	enable_reg |= exec_state;

	ret |= lp5521_write(client, LP5521_REG_ENABLE, enable_reg);

	/* set op-mode to run for active channels, disabled for others */
	ret |= lp5521_write(client, LP5521_REG_OP_MODE, exec_state);

fail:
	return ret;
}

/*--------------------------------------------------------------*/
/*			Sysfs interface				*/
/*--------------------------------------------------------------*/

static ssize_t show_active_channels(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct lp5521_chip *chip = dev_get_drvdata(dev);
	char channels[4];
	int pos = 0;

#ifdef LED_CONNECTED_WRONG
	if (chip->blue)
		pos += sprintf(channels + pos, "r");
	if (chip->green)
		pos += sprintf(channels + pos, "g");
	if (chip->red)
		pos += sprintf(channels + pos, "b");

#else
	if (chip->red)
		pos += sprintf(channels + pos, "r");
	if (chip->green)
		pos += sprintf(channels + pos, "g");
	if (chip->blue)
		pos += sprintf(channels + pos, "b");
#endif

	channels[pos] = '\0';

	return sprintf(buf, "%s\n", channels);
}

static ssize_t store_active_channels(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t len)
{
	struct lp5521_chip *chip = dev_get_drvdata(dev);

	chip->red = 0;
	chip->green = 0;
	chip->blue = 0;

#ifdef LED_CONNECTED_WRONG
	if (strchr(buf, 'r') != NULL)
		chip->blue = 1;
	if (strchr(buf, 'b') != NULL)
		chip->red = 1;
#else
	if (strchr(buf, 'r') != NULL)
		chip->red = 1;
	if (strchr(buf, 'b') != NULL)
		chip->blue = 1;
#endif
	if (strchr(buf, 'g') != NULL)
		chip->green = 1;

	return len;
}

static ssize_t show_color(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret = 0;
	u8 r, g, b;

	ret |= lp5521_read(client, LP5521_REG_R_PWM, &r);
	ret |= lp5521_read(client, LP5521_REG_G_PWM, &g);
	ret |= lp5521_read(client, LP5521_REG_B_PWM, &b);

	if (ret)
		return ret;

	return sprintf(buf, "%.2x:%.2x:%.2x\n", r, g, b);
}

static ssize_t store_color(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t len)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lp5521_chip *chip = i2c_get_clientdata(client);
	int ret;
	unsigned r, g, b;


	ret = sscanf(buf, "%2x:%2x:%2x", &r, &g, &b);
	if (ret != 3)
		return  -EINVAL;

	mutex_lock(&chip->lock);

	ret = lp5521_write(client, LP5521_REG_R_PWM, (u8)r);
	ret = lp5521_write(client, LP5521_REG_G_PWM, (u8)g);
	ret = lp5521_write(client, LP5521_REG_B_PWM, (u8)b);

	mutex_unlock(&chip->lock);

	return len;
}

static ssize_t store_load(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t len)
{
	struct lp5521_chip *chip = dev_get_drvdata(dev);
	int  ret, nrchars, offset = 0, i = 0;
	char c[3];
	unsigned cmd;
	u8 pattern[LP5521_PROGRAM_LENGTH] = {0};

	while ((offset < len - 1) && (i < LP5521_PROGRAM_LENGTH)) {

		/* separate sscanfs because length is working only for %s */
		ret = sscanf(buf + offset, "%2s%n ", c, &nrchars);
		ret = sscanf(c, "%2x", &cmd);
		if (ret != 1)
			goto fail;
		pattern[i] = (u8)cmd;

		offset += nrchars;
		i++;
	}

	/* pattern commands are always two bytes long */
	if (i % 2)
		goto fail;

	mutex_lock(&chip->lock);

	ret = lp5521_load_program(chip, pattern);
	mutex_unlock(&chip->lock);

	if (ret) {
		dev_err(dev, "lp5521 failed loading pattern\n");
		return ret;
	}

	return len;
fail:
	dev_err(dev, "lp5521 wrong pattern format\n");
	return -EINVAL;
}

static ssize_t show_mode(struct device *dev,
			 struct device_attribute *attr,
			 char *buf)
{
	struct lp5521_chip *chip = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", chip->mode);
}

static ssize_t store_mode(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf, size_t len)
{
	struct lp5521_chip *chip = dev_get_drvdata(dev);

	mutex_lock(&chip->lock);

	if (!strncmp(buf, "run", 3))
		lp5521_set_mode(chip, LP5521_MODE_RUN);
	else if (!strncmp(buf, "load", 4))
		lp5521_set_mode(chip, LP5521_MODE_LOAD);
	else if (!strncmp(buf, "direct", 6))
		lp5521_set_mode(chip, LP5521_MODE_DIRECT_CONTROL);

	mutex_unlock(&chip->lock);

	return len;
}

static ssize_t show_current(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int ret = 0;
	u8 r_curr, g_curr, b_curr;

	ret |= lp5521_read(client, LP5521_REG_R_CNTRL, &r_curr);
	ret |= lp5521_read(client, LP5521_REG_G_CNTRL, &g_curr);
	ret |= lp5521_read(client, LP5521_REG_B_CNTRL, &b_curr);

	if (ret)
		return ret;

	r_curr = r_curr >> 4;
	g_curr = g_curr >> 4;
	b_curr = b_curr >> 4;

	if (r_curr == g_curr && g_curr == b_curr)
		return sprintf(buf, "%x\n", r_curr);
	else
		return sprintf(buf, "%x %x %x\n", r_curr, g_curr, b_curr);
}

static ssize_t store_current(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t len)
{
	struct lp5521_chip *chip = dev_get_drvdata(dev);
	struct i2c_client *client = chip->client;
	int ret;
	unsigned curr;

	ret = sscanf(buf, "%1x", &curr);
	if (ret != 1)
		return  -EINVAL;

	/* current level is determined by the 4 upper bits, rest is ones */
	curr = (curr << 4) | 0x0f;

	mutex_lock(&chip->lock);

	ret |= lp5521_write(client, LP5521_REG_R_CNTRL, (u8)curr);
	ret |= lp5521_write(client, LP5521_REG_G_CNTRL, (u8)curr);
	ret |= lp5521_write(client, LP5521_REG_B_CNTRL, (u8)curr);

	mutex_unlock(&chip->lock);

	return len;
}

static DEVICE_ATTR(color, S_IRUGO | S_IWUGO, show_color, store_color);
static DEVICE_ATTR(load, S_IWUGO, NULL, store_load);
static DEVICE_ATTR(mode, S_IRUGO | S_IWUGO, show_mode, store_mode);
static DEVICE_ATTR(active_channels, S_IRUGO | S_IWUGO,
		   show_active_channels, store_active_channels);
static DEVICE_ATTR(led_current, S_IRUGO | S_IWUGO, show_current, store_current);

static int lp5521_register_sysfs(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	int ret;

	ret = device_create_file(dev, &dev_attr_color);
	if (ret)
		goto fail1;
	ret = device_create_file(dev, &dev_attr_load);
	if (ret)
		goto fail2;
	ret = device_create_file(dev, &dev_attr_active_channels);
	if (ret)
		goto fail3;
	ret = device_create_file(dev, &dev_attr_mode);
	if (ret)
		goto fail4;
	ret = device_create_file(dev, &dev_attr_led_current);
	if (ret)
		goto fail5;
	return 0;

fail5:
	device_remove_file(dev, &dev_attr_mode);
fail4:
	device_remove_file(dev, &dev_attr_active_channels);
fail3:
	device_remove_file(dev, &dev_attr_load);
fail2:
	device_remove_file(dev, &dev_attr_color);
fail1:
	return ret;
}

static void lp5521_unregister_sysfs(struct i2c_client *client)
{
	struct lp5521_chip *chip = i2c_get_clientdata(client);
	struct device *dev = &client->dev;

	device_remove_file(dev, &dev_attr_led_current);
	device_remove_file(dev, &dev_attr_mode);
	device_remove_file(dev, &dev_attr_active_channels);
	device_remove_file(dev, &dev_attr_color);

	if (!strcmp(chip->mode, LP5521_MODE_LOAD))
		device_remove_file(dev, &dev_attr_load);
}

/*--------------------------------------------------------------*/
/*			Set chip operating mode			*/
/*--------------------------------------------------------------*/

static int lp5521_set_mode(struct lp5521_chip *chip, char *mode)
{
	struct i2c_client *client = chip->client ;
	int ret = 0;

	/* if in that mode already do nothing, except for run */
	if (!strcmp(mode, chip->mode) && strcmp(mode, LP5521_MODE_RUN))
		return 0;

	if (!strcmp(mode, LP5521_MODE_RUN))
		ret = lp5521_run_program(chip);

	if (!strcmp(mode, LP5521_MODE_LOAD))
		ret |= lp5521_write(client, LP5521_REG_OP_MODE, 0x15);

	if (!strcmp(mode, LP5521_MODE_DIRECT_CONTROL))
		ret |= lp5521_write(client, LP5521_REG_OP_MODE, 0x3F);

	chip->mode = mode;

	return ret;
}

/*--------------------------------------------------------------*/
/*			Probe, Attach, Remove			*/
/*--------------------------------------------------------------*/
static struct i2c_driver lp5521_driver;

static int lp5521_probe(struct i2c_client *client)
{
	struct lp5521_chip *chip;
	int ret = 0;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client	= client;
	strncpy(client->name, LP5521_DRIVER_NAME, I2C_NAME_SIZE);
	i2c_set_clientdata(client, chip);

	mutex_init(&chip->lock);

	ret = lp5521_configure(client);
	if (ret < 0) {
		dev_err(&client->dev, "lp5521 error configuring chip \n");
		goto fail1;
	}

	/* Set default values */
	chip->mode	= LP5521_MODE_DIRECT_CONTROL;
	chip->red	= 1;
	chip->green	= 1;
	chip->blue	= 1;

	ret = lp5521_register_sysfs(client);
	if (ret)
		dev_err(&client->dev, "lp5521 registering sysfs failed \n");

	return ret;

fail1:
	kfree(chip);
	return ret;
}

static int lp5521_remove(struct i2c_client *client)
{
	struct lp5521_chip *chip = i2c_get_clientdata(client);

	lp5521_unregister_sysfs(client);
	kfree(chip);

	return 0;
}

static struct i2c_driver lp5521_driver = {
	.driver = {
		.name	= LP5521_DRIVER_NAME,
	},
	.probe		= lp5521_probe,
	.remove		= __exit_p(lp5521_remove),
};

static int __init lp5521_init(void)
{
	return i2c_add_driver(&lp5521_driver);
}

static void __exit lp5521_exit(void)
{
	i2c_del_driver(&lp5521_driver);
}

MODULE_AUTHOR("Mathias Nyman <mathias.nyman@nokia.com>");
MODULE_DESCRIPTION("lp5521 LED driver");
MODULE_LICENSE("GPL");

module_init(lp5521_init);
module_exit(lp5521_exit);
