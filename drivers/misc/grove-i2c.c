/*
 *  grove-i2c.c - I2C driver module for the Grove motor drivers
 *
 *  Copyright (C) 2013 Pantelis Antoniou <panto@antoniou-consulting.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>

struct grove_data {
	struct i2c_client *client;
	struct mutex lock;

	int state[2];
#define GS_NONE			0
#define GS_SOLENOID_FORWARD	1
#define GS_SOLENOID_BACKWARD	2

};

static void grove_send_word(struct grove_data *data, u8 cmd, u16 val)
{
	struct i2c_client *client = data->client;

	i2c_smbus_write_word_data(data->client, cmd, val);
}

static int grove_set_state(struct grove_data *data, unsigned int which,
		int state)
{
	static const u8 motor_addr[2] = { 0xa1, 0xa5 };

	if (which > 1)
		return -EINVAL;

	switch (state) {

		case GS_NONE:
			/* nothing */
			break;

		case GS_SOLENOID_FORWARD:
			grove_send_word(data, motor_addr[which], 0xff01);
			break;

		case GS_SOLENOID_BACKWARD:
			grove_send_word(data, motor_addr[which], 0x0000);
			break;

		default:
			/* unknown state */
			return -EINVAL;
	}

	data->state[which] = state;

	return 0;
}

static ssize_t grove_show_state(struct device *dev,
		struct device_attribute *attr, char *buf, unsigned int which)
{
	struct grove_data *data = i2c_get_clientdata(to_i2c_client(dev));

	if (which > 1)
		return -EINVAL;

	return sprintf(buf, "%u\n", data->state[which]);
}

static ssize_t grove_store_state(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count,
		unsigned int which)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct grove_data *data = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int ret;

	if (which > 1)
		return -EINVAL;

	if (val < 0 || val > 2)
		return -EINVAL;

	mutex_lock(&data->lock);
	ret = grove_set_state(data, which, val);
	mutex_unlock(&data->lock);

	if (ret < 0)
		return ret;

	return count;
}

static ssize_t grove_show_stateA(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return grove_show_state(dev, attr, buf, 0);
}

static ssize_t grove_store_stateA(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return grove_store_state(dev, attr, buf, count, 0);
}

static DEVICE_ATTR(stateA, S_IWUSR | S_IRUGO,
		   grove_show_stateA, grove_store_stateA);

static ssize_t grove_show_stateB(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return grove_show_state(dev, attr, buf, 1);
}

static ssize_t grove_store_stateB(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return grove_store_state(dev, attr, buf, count, 1);
}


static DEVICE_ATTR(stateB, S_IWUSR | S_IRUGO,
		   grove_show_stateB, grove_store_stateB);

static struct attribute *grove_attributes[] = {
	&dev_attr_stateA.attr,
	&dev_attr_stateB.attr,
	NULL
};

static const struct attribute_group grove_attr_group = {
	.attrs = grove_attributes,
};

static int grove_init_client(struct i2c_client *client)
{
	struct grove_data *data = i2c_get_clientdata(client);

	data->state[0] = GS_NONE;
	data->state[1] = GS_NONE;

	return 0;
}

static int grove_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct grove_data *data;
	int err = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WRITE_BYTE
					    | I2C_FUNC_SMBUS_READ_BYTE_DATA))
		return -EIO;

	data = devm_kzalloc(dev, sizeof(struct grove_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	i2c_set_clientdata(client, data);

	mutex_init(&data->lock);

	/* Initialize the TSL2550 chip */
	err = grove_init_client(client);
	if (err) {
		dev_err(dev, "failed to init client\n");
		goto exit_kfree;
	}

	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &grove_attr_group);
	if (err) {
		dev_err(dev, "failed to create sysfs group\n");
		goto exit_kfree;
	}

	dev_info(dev, "Initialized OK\n");

	return 0;

exit_kfree:
	devm_kfree(dev, data);
	return err;
}

static int grove_remove(struct i2c_client *client)
{
	struct grove_data *data = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &grove_attr_group);

	devm_kfree(&client->dev, data);

	return 0;
}

static const struct i2c_device_id grove_id[] = {
	{ "grove", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, grove_id);

static struct i2c_driver grove_driver = {
	.driver = {
		.name	= "grove-i2c",
		.owner	= THIS_MODULE,
	},
	.probe	= grove_probe,
	.remove	= grove_remove,
	.id_table = grove_id,
};

module_i2c_driver(grove_driver);

MODULE_AUTHOR("Pantelis Antoniou <panto@antoniou-consulting.com>");
MODULE_DESCRIPTION("Grove I2C Motor control driver");
MODULE_LICENSE("GPL");
