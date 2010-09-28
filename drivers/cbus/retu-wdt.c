/**
 * drivers/cbus/retu-wdt.c
 *
 * Driver for Retu watchdog
 *
 * Copyright (C) 2004, 2005 Nokia Corporation
 *
 * Written by Amit Kucheria <amit.kucheria@nokia.com>
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/init.h>

#include <linux/completion.h>
#include <linux/errno.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>

#include "cbus.h"
#include "retu.h"

/* Watchdog timeout in seconds */
#define RETU_WDT_MIN_TIMER 0
#define RETU_WDT_DEFAULT_TIMER 32
#define RETU_WDT_MAX_TIMER 63

static struct completion retu_wdt_completion;
static DEFINE_MUTEX(retu_wdt_mutex);

/* Current period of watchdog */
static unsigned int period_val = RETU_WDT_DEFAULT_TIMER;
static int counter_param = RETU_WDT_MAX_TIMER;

static int retu_modify_counter(unsigned int new)
{
	int ret = 0;

	if (new < RETU_WDT_MIN_TIMER || new > RETU_WDT_MAX_TIMER)
		return -EINVAL;

	mutex_lock(&retu_wdt_mutex);

	period_val = new;
	retu_write_reg(RETU_REG_WATCHDOG, (u16)period_val);

	mutex_unlock(&retu_wdt_mutex);
	return ret;
}

static ssize_t retu_wdt_period_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	/* Show current max counter */
	return sprintf(buf, "%u\n", (u16)period_val);
}

static ssize_t retu_wdt_period_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned int new_period;
	int ret;

	if (sscanf(buf, "%u", &new_period) != 1) {
		printk(KERN_ALERT "retu_wdt_period_store: Invalid input\n");
		return -EINVAL;
	}

	ret = retu_modify_counter(new_period);
	if (ret < 0)
		return ret;

	return strnlen(buf, count);
}

static ssize_t retu_wdt_counter_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	u16 counter;

	/* Show current value in watchdog counter */
	counter = retu_read_reg(RETU_REG_WATCHDOG);

	/* Only the 5 LSB are important */
	return snprintf(buf, PAGE_SIZE, "%u\n", (counter & 0x3F));
}

static DEVICE_ATTR(period, S_IRUGO | S_IWUSR, retu_wdt_period_show, \
			retu_wdt_period_store);
static DEVICE_ATTR(counter, S_IRUGO, retu_wdt_counter_show, NULL);

static int __devinit retu_wdt_probe(struct device *dev)
{
	int ret;

	ret = device_create_file(dev, &dev_attr_period);
	if (ret) {
		printk(KERN_ERR "retu_wdt_probe: Error creating "
					"sys device file: period\n");
		return ret;
	}

	ret = device_create_file(dev, &dev_attr_counter);
	if (ret) {
		device_remove_file(dev, &dev_attr_period);
		printk(KERN_ERR "retu_wdt_probe: Error creating "
					"sys device file: counter\n");
	}

	return ret;
}

static int __devexit retu_wdt_remove(struct device *dev)
{
	device_remove_file(dev, &dev_attr_period);
	device_remove_file(dev, &dev_attr_counter);
	return 0;
}

static void retu_wdt_device_release(struct device *dev)
{
	complete(&retu_wdt_completion);
}

static struct platform_device retu_wdt_device = {
	.name = "retu-watchdog",
	.id = -1,
	.dev = {
		.release = retu_wdt_device_release,
	},
};

static struct device_driver retu_wdt_driver = {
	.name = "retu-watchdog",
	.bus = &platform_bus_type,
	.probe = retu_wdt_probe,
	.remove = __devexit_p(retu_wdt_remove),
};

static int __init retu_wdt_init(void)
{
	int ret;

	init_completion(&retu_wdt_completion);

	ret = driver_register(&retu_wdt_driver);
	if (ret)
		return ret;

	ret = platform_device_register(&retu_wdt_device);
	if (ret)
		goto exit1;

	/* passed as module parameter? */
	ret = retu_modify_counter(counter_param);
	if (ret == -EINVAL) {
		ret = retu_modify_counter(RETU_WDT_DEFAULT_TIMER);
		printk(KERN_INFO
		       "retu_wdt_init: Intializing to default value\n");
	}

	printk(KERN_INFO "Retu watchdog driver initialized\n");
	return ret;

exit1:
	driver_unregister(&retu_wdt_driver);
	wait_for_completion(&retu_wdt_completion);

	return ret;
}

static void __exit retu_wdt_exit(void)
{
	platform_device_unregister(&retu_wdt_device);
	driver_unregister(&retu_wdt_driver);

	wait_for_completion(&retu_wdt_completion);
}

module_init(retu_wdt_init);
module_exit(retu_wdt_exit);
module_param(counter_param, int, 0);

MODULE_DESCRIPTION("Retu WatchDog");
MODULE_AUTHOR("Amit Kucheria");
MODULE_LICENSE("GPL");

