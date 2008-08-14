/*
 * omap34xx_temp.c - Linux kernel module for OMAP34xx hardware monitoring
 *
 * Copyright (C) 2008 Nokia Corporation
 *
 * Written by Peter De Schrijver <peter.de-schrijver@nokia.com>
 *
 * Inspired by k8temp.c
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/clk.h>
#include <linux/hrtimer.h>
#include <linux/module.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <mach/omap34xx.h>
#include <mach/control.h>

#define TEMP_SENSOR_SOC BIT(8)
#define TEMP_SENSOR_EOCZ BIT(7)

/* minimum delay for EOCZ rise after SOC rise is
 * 11 cycles of the 32.768Khz clock */
#define EOCZ_MIN_RISING_DELAY (11 * 30518)

/* maximum delay for EOCZ rise after SOC rise is
 * 14 cycles of the 32.768Khz clock */
#define EOCZ_MAX_RISING_DELAY (14 * 30518)

/* minimum delay for EOCZ falling is
 * 36 cycles of the 32.768Khz clock */
#define EOCZ_MIN_FALLING_DELAY (36 * 30518)

/* maximum delay for EOCZ falling is
 * 40 cycles of the 32.768Khz clock */
#define EOCZ_MAX_FALLING_DELAY (40 * 30518)

struct omap34xx_data {
	struct device *hwmon_dev;
	struct clk *clk_32k;
	struct mutex update_lock;
	const char *name;
	char valid;
	unsigned long last_updated;
	u32 temp;
};

static struct platform_device omap34xx_temp_device = {
	.name 	= "omap34xx_temp",
	.id	= -1,
};

static int adc_to_temp[] = {
	-40, -40, -40, -40, -40, -39, -38, -36, -34, -32, -31, -29, -28, -26,
	-25, -24, -22, -21, -19, -18, -17, -15, -14, -12, -11, -9, -8, -7, -5,
	-4, -2, -1, 0, 1, 3, 4, 5, 7, 8, 10, 11, 13, 14, 15, 17, 18, 20, 21,
	22, 24, 25, 27, 28, 30, 31, 32, 34, 35, 37, 38, 39, 41, 42, 44, 45,
	47, 48, 49, 51, 52, 53, 55, 56, 58, 59, 60, 62, 63, 65, 66, 67, 69,
	70, 72, 73, 74, 76, 77, 79, 80, 81, 83, 84, 85, 87, 88, 89, 91, 92,
	94, 95, 96, 98, 99, 100, 102, 103, 105, 106, 107, 109, 110, 111, 113,
	114, 116, 117, 118, 120, 121, 122, 124, 124, 125, 125, 125, 125, 125};

static inline u32 wait_for_eocz(int min_delay, int max_delay, u32 level)
{
	struct timespec timeout;
	ktime_t expire;
	u32 temp_sensor_reg;

	level &= 1;
	level *= TEMP_SENSOR_EOCZ;

	expire = ktime_add_ns(ktime_get(), max_delay);
	timeout = ns_to_timespec(min_delay);
	hrtimer_nanosleep(&timeout, NULL, HRTIMER_MODE_REL, CLOCK_MONOTONIC);
	do {
		temp_sensor_reg = omap_ctrl_readl(OMAP343X_CONTROL_TEMP_SENSOR);
		if ((temp_sensor_reg & TEMP_SENSOR_EOCZ) == level)
			break;
	} while (ktime_us_delta(expire, ktime_get()) > 0);

	return (temp_sensor_reg & TEMP_SENSOR_EOCZ) == level;
}

static void omap34xx_update(struct omap34xx_data *data)
{
	u32 temp_sensor_reg;

	mutex_lock(&data->update_lock);

	if (!data->valid
	    || time_after(jiffies, data->last_updated + HZ)) {

		clk_enable(data->clk_32k);

		temp_sensor_reg = omap_ctrl_readl(OMAP343X_CONTROL_TEMP_SENSOR);
		temp_sensor_reg |= TEMP_SENSOR_SOC;
		omap_ctrl_writel(temp_sensor_reg, OMAP343X_CONTROL_TEMP_SENSOR);

		if (!wait_for_eocz(EOCZ_MIN_RISING_DELAY,
					EOCZ_MAX_RISING_DELAY, 1))
			goto err;

		temp_sensor_reg = omap_ctrl_readl(OMAP343X_CONTROL_TEMP_SENSOR);
		temp_sensor_reg &= ~TEMP_SENSOR_SOC;
		omap_ctrl_writel(temp_sensor_reg, OMAP343X_CONTROL_TEMP_SENSOR);

		if (!wait_for_eocz(EOCZ_MIN_FALLING_DELAY,
					EOCZ_MAX_FALLING_DELAY, 0))
			goto err;

		data->temp = omap_ctrl_readl(OMAP343X_CONTROL_TEMP_SENSOR) &
						((1<<7) - 1);
		data->last_updated = jiffies;
		data->valid = 1;

err:
		clk_disable(data->clk_32k);
	}

	mutex_unlock(&data->update_lock);
}

static ssize_t show_name(struct device *dev,
			struct device_attribute *devattr, char *buf)
{
	struct omap34xx_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", data->name);
}

static ssize_t show_temp_raw(struct device *dev,
			 struct device_attribute *devattr, char *buf)
{
	struct omap34xx_data *data = dev_get_drvdata(dev);

	omap34xx_update(data);

	return sprintf(buf, "%d\n", data->temp);
}

static ssize_t show_temp(struct device *dev,
			 struct device_attribute *devattr, char *buf)
{
	struct omap34xx_data *data = dev_get_drvdata(dev);

	omap34xx_update(data);

	return sprintf(buf, "%d\n", adc_to_temp[data->temp]);
}

static SENSOR_DEVICE_ATTR_2(temp1_input, S_IRUGO, show_temp, NULL, 0, 0);
static SENSOR_DEVICE_ATTR_2(temp1_input_raw, S_IRUGO, show_temp_raw,
				NULL, 0, 0);
static DEVICE_ATTR(name, S_IRUGO, show_name, NULL);

static int __devinit omap34xx_temp_probe(void)
{
	int err;
	struct omap34xx_data *data;

	err = platform_device_register(&omap34xx_temp_device);
	if (err) {
		printk(KERN_ERR
			"Unable to register omap34xx temperature device\n");
		goto exit;
	}

	data = kzalloc(sizeof(struct omap34xx_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit_platform;
	}

	dev_set_drvdata(&omap34xx_temp_device.dev, data);
	mutex_init(&data->update_lock);
	data->name = "omap34xx_temp";

	data->clk_32k = clk_get(&omap34xx_temp_device.dev, "ts_fck");
	if (IS_ERR(data->clk_32k)) {
		err = PTR_ERR(data->clk_32k);
		goto exit_free;
	}

	err = device_create_file(&omap34xx_temp_device.dev,
				 &sensor_dev_attr_temp1_input.dev_attr);
	if (err)
		goto clock_free;

	err = device_create_file(&omap34xx_temp_device.dev,
				 &sensor_dev_attr_temp1_input_raw.dev_attr);
	if (err)
		goto exit_remove;

	err = device_create_file(&omap34xx_temp_device.dev, &dev_attr_name);
	if (err)
		goto exit_remove_raw;

	data->hwmon_dev = hwmon_device_register(&omap34xx_temp_device.dev);

	if (IS_ERR(data->hwmon_dev)) {
		err = PTR_ERR(data->hwmon_dev);
		goto exit_remove_all;
	}

	return 0;

exit_remove_all:
	device_remove_file(&omap34xx_temp_device.dev,
			   &dev_attr_name);
exit_remove_raw:
	device_remove_file(&omap34xx_temp_device.dev,
			   &sensor_dev_attr_temp1_input_raw.dev_attr);
exit_remove:
	device_remove_file(&omap34xx_temp_device.dev,
			   &sensor_dev_attr_temp1_input.dev_attr);
clock_free:
	clk_put(data->clk_32k);

exit_free:
	kfree(data);
exit_platform:
	platform_device_unregister(&omap34xx_temp_device);
exit:
	return err;
}

static int __init omap34xx_temp_init(void)
{
	return omap34xx_temp_probe();
}

static void __exit omap34xx_temp_exit(void)
{
	struct omap34xx_data *data =
			dev_get_drvdata(&omap34xx_temp_device.dev);

	clk_put(data->clk_32k);
	hwmon_device_unregister(data->hwmon_dev);
	device_remove_file(&omap34xx_temp_device.dev,
			   &sensor_dev_attr_temp1_input.dev_attr);
	device_remove_file(&omap34xx_temp_device.dev, &dev_attr_name);
	kfree(data);
	platform_device_unregister(&omap34xx_temp_device);
}

MODULE_AUTHOR("Peter De Schrijver");
MODULE_DESCRIPTION("Omap34xx temperature sensor");
MODULE_LICENSE("GPL");

module_init(omap34xx_temp_init)
module_exit(omap34xx_temp_exit)

