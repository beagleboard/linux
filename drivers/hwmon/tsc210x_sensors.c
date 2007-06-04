/*
 * drivers/hwmon/tsc210x_sensors.c
 *
 * hwmon interface for TSC210X sensors
 *
 * Copyright (c) 2005-2007 Andrzej Zaborowski  <balrog@zabor.org>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This package is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */
#include <linux/init.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/autoconf.h>
#ifdef CONFIG_APM
# include <linux/apm-emulation.h>
#endif

#include <linux/spi/tsc210x.h>

struct tsc210x_hwmon {
	int bat[2], aux[2], temp[2];

	struct class_device *dev;
	struct tsc210x_config *pdata;
#ifdef CONFIG_APM
	spinlock_t apm_lock;
#endif
};

#ifdef CONFIG_APM
# define apm_lock()	spin_lock(&hwmon->apm_lock)
# define apm_unlock()	spin_unlock(&hwmon->apm_lock)
#else
# define apm_lock()
# define apm_unlock()
#endif

static void tsc210x_ports(struct tsc210x_hwmon *hwmon, int bat[], int aux[])
{
	apm_lock();
	hwmon->bat[0] = bat[0];
	hwmon->bat[1] = bat[1];
	hwmon->aux[0] = aux[0];
	hwmon->aux[1] = aux[1];
	apm_unlock();
}

static void tsc210x_temp1(struct tsc210x_hwmon *hwmon, int temp)
{
	apm_lock();
	hwmon->temp[0] = temp;
	apm_unlock();
}

static void tsc210x_temp2(struct tsc210x_hwmon *hwmon, int temp)
{
	apm_lock();
	hwmon->temp[1] = temp;
	apm_unlock();
}

#define TSC210X_INPUT(devname, field)	\
static ssize_t tsc_show_ ## devname(struct device *dev,	\
		struct device_attribute *devattr, char *buf)	\
{	\
	struct tsc210x_hwmon *hwmon = (struct tsc210x_hwmon *)	\
		platform_get_drvdata(to_platform_device(dev));	\
	return sprintf(buf, "%i\n", hwmon->field);	\
}	\
static DEVICE_ATTR(devname ## _input, S_IRUGO, tsc_show_ ## devname, NULL);

TSC210X_INPUT(in0, bat[0])
TSC210X_INPUT(in1, bat[1])
TSC210X_INPUT(in2, aux[0])
TSC210X_INPUT(in3, aux[1])
TSC210X_INPUT(in4, temp[0])
TSC210X_INPUT(in5, temp[1])

static ssize_t tsc_show_temp1(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	struct tsc210x_hwmon *hwmon = (struct tsc210x_hwmon *)
		platform_get_drvdata(to_platform_device(dev));
	int t1, t2;
	int diff, value;

	t1 = hwmon->temp[0];
	t2 = hwmon->temp[1];

	/*
	 * Use method #2 (differential) to calculate current temperature.
	 * The difference between TEMP2 and TEMP1 input values is
	 * multiplied by a constant to obtain current temperature.
	 * To find this constant we use the values measured at 25 C as
	 * thermometer calibration data.
	 *
	 * 298150 is 25 degrees Celcius represented in Kelvins and
	 * multiplied by 1000 for fixed point precision (273.15 + 25).
	 * 273150 is zero degrees Celcius.
	 */
	diff = hwmon->pdata->temp_at25c[1] - hwmon->pdata->temp_at25c[0];
	BUG_ON(diff == 0);
	value = (t2 - t1) * 298150 / diff;	/* This is in Kelvins now */

	value -= 273150;			/* Celcius millidegree */
	return sprintf(buf, "%i\n", value);
}
static DEVICE_ATTR(temp1_input, S_IRUGO, tsc_show_temp1, NULL);

#ifdef CONFIG_APM
static struct tsc210x_hwmon *apm_hwmon;

static void tsc210x_get_power_status(struct apm_power_info *info)
{
	struct tsc210x_hwmon *hwmon = apm_hwmon;
	apm_lock();
	hwmon->pdata->apm_report(info, hwmon->bat);
	apm_unlock();
}
#endif

static int tsc210x_hwmon_probe(struct platform_device *pdev)
{
	struct tsc210x_hwmon *hwmon;
	struct tsc210x_config *pdata = pdev->dev.platform_data;
	int status = 0;

	hwmon = (struct tsc210x_hwmon *)
		kzalloc(sizeof(struct tsc210x_hwmon), GFP_KERNEL);
	if (!hwmon) {
		printk(KERN_ERR "%s: allocation failed\n", __FUNCTION__);
		return -ENOMEM;
	}

	hwmon->dev = hwmon_device_register(&pdev->dev);
	if (IS_ERR(hwmon->dev)) {
		kfree(hwmon);
		printk(KERN_ERR "%s: Class registration failed\n",
				__FUNCTION__);
		return PTR_ERR(hwmon->dev);
	}

	hwmon->pdata = pdata;

#ifdef CONFIG_APM
	spin_lock_init(&hwmon->apm_lock);

	if (pdata->apm_report) {
		apm_hwmon = hwmon;
		apm_get_power_status = tsc210x_get_power_status;
	}
#endif

	platform_set_drvdata(pdev, hwmon);

	if (pdata->monitor & (TSC_BAT1 | TSC_BAT2 | TSC_AUX1 | TSC_AUX2))
		status |= tsc210x_ports_cb(pdev->dev.parent,
				(tsc210x_ports_t) tsc210x_ports, hwmon);
	if (pdata->monitor & TSC_TEMP) {
		status |= tsc210x_temp1_cb(pdev->dev.parent,
				(tsc210x_temp_t) tsc210x_temp1, hwmon);
		status |= tsc210x_temp2_cb(pdev->dev.parent,
				(tsc210x_temp_t) tsc210x_temp2, hwmon);
	}

	if (status) {
		tsc210x_ports_cb(pdev->dev.parent, 0, 0);
		tsc210x_temp1_cb(pdev->dev.parent, 0, 0);
		tsc210x_temp2_cb(pdev->dev.parent, 0, 0);
		platform_set_drvdata(pdev, 0);
#ifdef CONFIG_APM
		if (pdata->apm_report)
			apm_get_power_status = 0;
#endif
		hwmon_device_unregister(hwmon->dev);
		kfree(hwmon);
		return status;
	}

	if (pdata->monitor & TSC_BAT1)
		status |= device_create_file(&pdev->dev, &dev_attr_in0_input);
	if (pdata->monitor & TSC_BAT2)
		status |= device_create_file(&pdev->dev, &dev_attr_in1_input);
	if (pdata->monitor & TSC_AUX1)
		status |= device_create_file(&pdev->dev, &dev_attr_in2_input);
	if (pdata->monitor & TSC_AUX2)
		status |= device_create_file(&pdev->dev, &dev_attr_in3_input);
	if (pdata->monitor & TSC_TEMP) {
		status |= device_create_file(&pdev->dev, &dev_attr_in4_input);
		status |= device_create_file(&pdev->dev, &dev_attr_in5_input);
		status |= device_create_file(&pdev->dev, &dev_attr_temp1_input);
	}
	if (status)	/* Not fatal */
		printk(KERN_ERR "%s: Creating one or more "
				"attribute files failed\n", __FUNCTION__);

	return 0;
}

static int tsc210x_hwmon_remove(struct platform_device *pdev)
{
	struct tsc210x_hwmon *dev = platform_get_drvdata(pdev);

	tsc210x_ports_cb(pdev->dev.parent, 0, 0);
	tsc210x_temp1_cb(pdev->dev.parent, 0, 0);
	tsc210x_temp2_cb(pdev->dev.parent, 0, 0);
	platform_set_drvdata(pdev, 0);
#ifdef CONFIG_APM
	if (dev->pdata->apm_report)
		apm_get_power_status = 0;
#endif
	hwmon_device_unregister(dev->dev);
	kfree(dev);
	return 0;
}

static struct platform_driver tsc210x_hwmon_driver = {
	.probe 		= tsc210x_hwmon_probe,
	.remove 	= tsc210x_hwmon_remove,
	/* Nothing to do on suspend/resume */
	.driver		= {
		.name	= "tsc210x-hwmon",
	},
};

static int __init tsc210x_hwmon_init(void)
{
	return platform_driver_register(&tsc210x_hwmon_driver);
}

static void __exit tsc210x_hwmon_exit(void)
{
	platform_driver_unregister(&tsc210x_hwmon_driver);
}

module_init(tsc210x_hwmon_init);
module_exit(tsc210x_hwmon_exit);

MODULE_AUTHOR("Andrzej Zaborowski");
MODULE_DESCRIPTION("hwmon driver for TI TSC210x-connected sensors.");
MODULE_LICENSE("GPL");
