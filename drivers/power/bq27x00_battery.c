/*
 * linux/drivers/power/bq27x00_battery.c
 *
 * BQ27000/BQ27200 battery driver
 *
 * Copyright (C) 2008 Texas Instruments, Inc.
 *
 * Author: Texas Instruments
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>

#ifdef CONFIG_BATTERY_BQ27000
#include "../w1/w1.h"
#endif
#ifdef CONFIG_BATTERY_BQ27200
#include <linux/i2c.h>
#endif

#define BQ27x00_REG_TEMP		0x06
#define BQ27x00_REG_VOLT		0x08
#define BQ27x00_REG_RSOC		0x0B /* Relative State-of-Charge */
#define BQ27x00_REG_AI			0x14
#define BQ27x00_REG_FLAGS		0x0A
#define HIGH_BYTE(A)			((A) << 8)

#ifdef CONFIG_BATTERY_BQ27000
extern int w1_bq27000_read(struct device *dev, u8 reg);
#endif

struct bq27x00_device_info;
struct bq27x00_access_methods {
	int (*read)(u8 reg, int *rt_value, int b_single,
		struct bq27x00_device_info *di);
};

struct bq27x00_device_info {
	struct device 		*dev;
#ifdef CONFIG_BATTERY_BQ27000
	struct device		*w1_dev;
#endif
#ifdef CONFIG_BATTERY_BQ27200
	struct i2c_client *client;
#endif
	unsigned long		update_time;
	int			voltage_uV;
	int			current_uA;
	int			temp_C;
	int			charge_rsoc;
	struct bq27x00_access_methods	*bus;
	struct power_supply	bat;
	struct delayed_work	monitor_work;
};

static unsigned int cache_time = 60000;
module_param(cache_time, uint, 0644);
MODULE_PARM_DESC(cache_time, "cache time in milliseconds");

static enum power_supply_property bq27x00_battery_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
};

static int bq27x00_read(u8 reg, int *rt_value, int b_single,
	struct bq27x00_device_info *di);

#ifdef CONFIG_BATTERY_BQ27000
static int bq27000_battery_probe(struct platform_device *dev);
static int bq27000_battery_remove(struct platform_device *dev);
#ifdef CONFIG_PM
static int bq27000_battery_suspend(struct platform_device *dev,
	pm_message_t state);
static int bq27000_battery_resume(struct platform_device *dev);
#endif /* CONFIG_PM */

static struct platform_driver bq27000_battery_driver = {
	.probe = bq27000_battery_probe,
	.remove = bq27000_battery_remove,
#ifdef CONFIG_PM
	.suspend = bq27000_battery_suspend,
	.resume = bq27000_battery_resume,
#endif /* CONFIG_PM */
	.driver = {
		.name = "bq27000-battery",
	},
};
#endif /* CONFIG_BATTERY_BQ27000 */

#ifdef CONFIG_BATTERY_BQ27200
static int bq27200_battery_probe(struct i2c_client *client);
static int bq27200_battery_remove(struct i2c_client *client);
#ifdef CONFIG_PM
static int bq27200_battery_suspend(struct i2c_client *client,
		pm_message_t mesg);
static int bq27200_battery_resume(struct i2c_client *client);
#endif /* CONFIG_PM */
static struct i2c_driver bq27200_battery_driver = {
	.driver = {
		.name   = "bq27200-bat",
	},
	.probe  = bq27200_battery_probe,
	.remove = bq27200_battery_remove,
#ifdef CONFIG_PM
	.suspend = bq27200_battery_suspend,
	.resume = bq27200_battery_resume,
#endif /* CONFIG_PM */
};
#endif /* CONFIG_BATTERY_BQ27200 */

/*
 * Return the battery temperature in Celcius degrees
 * Or < 0 if something fails.
 */
static int bq27x00_battery_temperature(struct bq27x00_device_info *di)
{
	int ret, temp = 0;

	ret = bq27x00_read(BQ27x00_REG_TEMP, &temp, 0, di);
	if (ret) {
		pr_err("BQ27x00 battery driver:"
			"Error reading temperature from the battery\n");
		return ret;
	}

	return (temp >> 2) - 273;
}

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
static int bq27x00_battery_voltage(struct bq27x00_device_info *di)
{
	int ret, volt = 0;

	ret = bq27x00_read(BQ27x00_REG_VOLT, &volt, 0, di);
	if (ret) {
		pr_err("BQ27x00 battery driver:"
			"Error reading battery voltage from the battery\n");
		return ret;
	}

	return volt;
}

/*
 * Return the battery average current
 * Note that current can be negative signed as well
 * Or 0 if something fails.
 */
static int bq27x00_battery_current(struct bq27x00_device_info *di)
{
	int ret, curr = 0, flags = 0;

	ret = bq27x00_read(BQ27x00_REG_AI, &curr, 0, di);
	if (ret) {
		pr_err("BQ27x00 battery driver:"
			"Error reading current from the battery\n");
		return 0;
	}
	ret = bq27x00_read(BQ27x00_REG_FLAGS, &flags, 0, di);
	if (ret < 0) {
		pr_err("BQ27x00 battery driver:"
			"Error reading battery flags\n");
		return 0;
	}
	if ((flags & (1 << 7)) != 0) {
		pr_debug("Negative current\n");
		return -curr;
	} else {
		return curr;
	}
}

/*
 * Return the battery Relative State-of-Charge
 * Or < 0 if something fails.
 */
static int bq27x00_battery_rsoc(struct bq27x00_device_info *di)
{
	int ret, rsoc = 0;

	ret = bq27x00_read(BQ27x00_REG_RSOC, &rsoc, 1, di);
	if (ret) {
		pr_err("BQ27x00 battery driver:"
			"Error reading battery Relative"
			"State-of-Charge\n");
		return ret;
	}
	return rsoc;
}

#ifdef CONFIG_BATTERY_BQ27000
static inline int bq27000_read(u8 reg, int *rt_value, int b_single,
	struct bq27x00_device_info *di)
{
	u8 val;

	val = w1_bq27000_read(di->w1_dev, reg);
	*rt_value = val;

	if (!b_single) {
		val = 0;
		val = w1_bq27000_read(di->w1_dev, reg + 1);
		*rt_value +=  HIGH_BYTE((int) val);
	}

	return 0;
}
#else
static inline int bq27000_read(u8 reg, int *rt_value, int b_single,
	struct bq27x00_device_info *di)
{
	return 0;
}
#endif

#ifdef CONFIG_BATTERY_BQ27200
static inline int bq27200_read(u8 reg, int *rt_value, int b_single,
	struct bq27x00_device_info *di)
{
	struct i2c_client *client = di->client;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int err;

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 1;
	msg->buf = data;

	data[0] = reg;
	err = i2c_transfer(client->adapter, msg, 1);

	if (err >= 0) {
		if (!b_single)
			msg->len = 2;
		else
			msg->len = 1;

		msg->flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 1);
		if (err >= 0) {
			if (!b_single)
				*rt_value = data[1] | HIGH_BYTE(data[0]);
			else
				*rt_value = data[0];

			return 0;
		} else {
			pr_err("BQ27200 I2C read failed\n");
			return err;
		}
	} else {
		pr_err("BQ27200 I2C write failed\n");
		return err;
	}
}
#else
static inline int bq27200_read(u8 reg, int *rt_value, int b_single,
	struct bq27x00_device_info *di)
{
	return 0;
}
#endif

static int bq27x00_read(u8 reg, int *rt_value, int b_single,
	struct bq27x00_device_info *di)
{
	int ret;

	ret = di->bus->read(reg, rt_value, b_single, di);
	return ret;
}

/*
 * Read the battery temp, voltage, current and relative state of charge.
 */
static void bq27x00_battery_read_status(struct bq27x00_device_info *di)
{
	if (di->update_time && time_before(jiffies, di->update_time +
					msecs_to_jiffies(cache_time)))
		return;

	di->temp_C = bq27x00_battery_temperature(di);
	di->voltage_uV = bq27x00_battery_voltage(di);
	di->current_uA = bq27x00_battery_current(di);
	di->charge_rsoc = bq27x00_battery_rsoc(di);

	di->update_time = jiffies;

	return;
}

static void bq27x00_battery_work(struct delayed_work *work)
{
	struct bq27x00_device_info *di = container_of(work,
		struct bq27x00_device_info, monitor_work);

	bq27x00_battery_read_status(di);
	schedule_delayed_work(&di->monitor_work, 100);
	return;
}

#define to_bq27x00_device_info(x) container_of((x), \
				struct bq27x00_device_info, bat);

static int bq27x00_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct bq27x00_device_info *di = to_bq27x00_device_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = di->voltage_uV;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = di->current_uA;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = di->charge_rsoc;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = di->temp_C;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = di->charge_rsoc;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		if (di->voltage_uV == 0)
			val->intval = 0;
		else
			val->intval = 1;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void bq27x00_powersupply_init(struct bq27x00_device_info *di)
{
	di->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	di->bat.properties = bq27x00_battery_props;
	di->bat.num_properties = ARRAY_SIZE(bq27x00_battery_props);
	di->bat.get_property = bq27x00_battery_get_property;
	di->bat.external_power_changed = NULL;
	return;
}

#ifdef CONFIG_BATTERY_BQ27200
static int bq27200_battery_probe(struct i2c_client *client)
{
	struct bq27x00_device_info *di;
	struct bq27x00_access_methods *bus;
	int retval = 0;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		pr_err("BQ27000 battery driver:"
			"Failed to allocate device info structure\n");
		return -ENOMEM;
	}

	bus = kzalloc(sizeof(*bus), GFP_KERNEL);
	if (!bus) {
		pr_err("BQ27000 battery driver:"
			"Failed to allocate access method structure\n");
		kfree(di);
		return -ENOMEM;
	}

	i2c_set_clientdata(client, di);
	di->dev = &client->dev;
	di->bat.name = "bq27200";
	bus->read = &bq27200_read;
	di->bus = bus;
	di->client = client;

	bq27x00_powersupply_init(di);

	retval = power_supply_register(&client->dev, &di->bat);
	if (retval) {
		pr_err("BQ27200 battery driver: Failed to register battery\n");
		goto batt_failed;
	}

	INIT_DELAYED_WORK(&di->monitor_work, bq27x00_battery_work);
	schedule_delayed_work(&di->monitor_work, 100);

	return 0;

batt_failed:
	kfree(bus);
	kfree(di);
	return retval;
}

static int bq27200_battery_remove(struct i2c_client *client)
{
	struct bq27x00_device_info *di  = i2c_get_clientdata(client);

	flush_scheduled_work();
	power_supply_unregister(&di->bat);
	kfree(di);

	return 0;
}

#ifdef CONFIG_PM
static int bq27200_battery_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct bq27x00_device_info *di  = i2c_get_clientdata(client);

	cancel_delayed_work(&di->monitor_work);
	return 0;
}

static int bq27200_battery_resume(struct i2c_client *client)
{
	struct bq27x00_device_info *di  = i2c_get_clientdata(client);

	schedule_delayed_work(&di->monitor_work, 0);
	return 0;
}
#endif /* CONFIG_PM */
#endif /* CONFIG_BATTERY_BQ27200 */

#ifdef CONFIG_BATTERY_BQ27000
static int bq27000_battery_probe(struct  platform_device *pdev)
{
	struct bq27x00_device_info *di;
	struct bq27x00_access_methods *bus;
	int retval = 0;

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
		pr_err("BQ27000 battery driver:"
			"Failed to allocate device info structure\n");
		return -ENOMEM;
	}

	bus = kzalloc(sizeof(*bus), GFP_KERNEL);
	if (!bus) {
		pr_err("BQ27000 battery driver:"
			"Failed to allocate access method structure\n");
		kfree(di);
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, di);

	di->dev = &pdev->dev;
	di->w1_dev = pdev->dev.parent;
	di->bat.name = "bq27000";
	bus->read = &bq27000_read;
	di->bus = bus;

	bq27x00_powersupply_init(di);

	retval = power_supply_register(&pdev->dev, &di->bat);
	if (retval) {
		pr_err("BQ27000 battery driver: Failed to register battery\n");
		goto batt_failed;
	}

	INIT_DELAYED_WORK(&di->monitor_work, bq27x00_battery_work);
	schedule_delayed_work(&di->monitor_work, 50);

	return 0;

batt_failed:
	kfree(bus);
	kfree(di);
	return retval;
}

static int bq27000_battery_remove(struct  platform_device *pdev)
{
	struct bq27x00_device_info *di = platform_get_drvdata(pdev);

	flush_scheduled_work();
	power_supply_unregister(&di->bat);
	platform_set_drvdata(pdev, NULL);
	kfree(di);

	return 0;
}

#ifdef CONFIG_PM
static int bq27000_battery_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	struct bq27x00_device_info *di = platform_get_drvdata(pdev);

	cancel_delayed_work(&di->monitor_work);
	return 0;
}

static int bq27000_battery_resume(struct platform_device *pdev)
{
	struct bq27x00_device_info *di = platform_get_drvdata(pdev);

	schedule_delayed_work(&di->monitor_work, 0);
	return 0;
}
#endif /* CONFIG_PM */
#endif /* CONFIG_BATTERY_BQ27000 */

static int __init bq27x00_battery_init(void)
{
	int status = 0;

#ifdef CONFIG_BATTERY_BQ27000
	status = platform_driver_register(&bq27000_battery_driver);
	if (status)
		printk(KERN_ERR "Unable to register BQ27000 driver\n");
#endif
#ifdef CONFIG_BATTERY_BQ27200
	status = i2c_add_driver(&bq27200_battery_driver);
		printk(KERN_ERR "Unable to register BQ27200 driver\n");
#endif
	return status;
}

static void __exit bq27x00_battery_exit(void)
{
#ifdef CONFIG_BATTERY_BQ27000
	platform_driver_unregister(&bq27000_battery_driver);
#endif
#ifdef CONFIG_BATTERY_BQ27200
	i2c_del_driver(&bq27200_battery_driver);
#endif
}

module_init(bq27x00_battery_init);
module_exit(bq27x00_battery_exit);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("BQ27x00 battery moniter driver");
MODULE_LICENSE("GPL");
