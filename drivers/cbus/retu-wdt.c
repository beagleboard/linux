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
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/platform_device.h>

#include <linux/completion.h>
#include <linux/errno.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>

#include <asm/uaccess.h>

#include <plat/prcm.h>

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

struct retu_wdt_dev {
	struct device		*dev;
	int			users;
	struct miscdevice	retu_wdt_miscdev;
	struct timer_list	ping_timer;
};

static struct retu_wdt_dev *retu_wdt;

static void retu_wdt_set_ping_timer(unsigned long enable);

static int _retu_modify_counter(unsigned int new)
{
	retu_write_reg(RETU_REG_WATCHDOG, (u16)new);

	return 0;
}

static int retu_modify_counter(unsigned int new)
{
	if (new < RETU_WDT_MIN_TIMER || new > RETU_WDT_MAX_TIMER)
		return -EINVAL;

	mutex_lock(&retu_wdt_mutex);
	period_val = new;
	_retu_modify_counter(period_val);
	mutex_unlock(&retu_wdt_mutex);

	return 0;
}

static ssize_t retu_wdt_period_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	/* Show current max counter */
	return sprintf(buf, "%u\n", (u16)period_val);
}

/*
 * Note: This inteface is non-standard and likely to disappear!
 * Use /dev/watchdog instead, that's the standard.
 */
static ssize_t retu_wdt_period_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned int new_period;
	int ret;

#ifdef CONFIG_WATCHDOG_NOWAYOUT
	retu_wdt_set_ping_timer(0);
#endif

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

/*----------------------------------------------------------------------------*/

/*
 * Since retu watchdog cannot be disabled in hardware, we must kick it
 * with a timer until userspace watchdog software takes over. Do this
 * unless /dev/watchdog is open or CONFIG_WATCHDOG_NOWAYOUT is set.
 */
static void retu_wdt_set_ping_timer(unsigned long enable)
{
	_retu_modify_counter(RETU_WDT_MAX_TIMER);
	if (enable)
		mod_timer(&retu_wdt->ping_timer,
				jiffies + RETU_WDT_DEFAULT_TIMER * HZ);
	else
		del_timer_sync(&retu_wdt->ping_timer);
}

static int retu_wdt_open(struct inode *inode, struct file *file)
{
	if (test_and_set_bit(1, (unsigned long *)&(retu_wdt->users)))
		return -EBUSY;

	file->private_data = (void *)retu_wdt;
	retu_wdt_set_ping_timer(0);

	return nonseekable_open(inode, file);
}

static int retu_wdt_release(struct inode *inode, struct file *file)
{
	struct retu_wdt_dev *wdev = file->private_data;

#ifndef CONFIG_WATCHDOG_NOWAYOUT
	retu_wdt_set_ping_timer(1);
#endif
	wdev->users = 0;

	return 0;
}

static ssize_t retu_wdt_write(struct file *file, const char __user *data,
						size_t len, loff_t *ppos)
{
	if (len)
		retu_modify_counter(RETU_WDT_MAX_TIMER);

	return len;
}

static int retu_wdt_ioctl(struct inode *inode, struct file *file,
					unsigned int cmd, unsigned long arg)
{
	int new_margin;

	static struct watchdog_info ident = {
		.identity = "Retu Watchdog",
		.options = WDIOF_SETTIMEOUT,
		.firmware_version = 0,
	};

	switch (cmd) {
	default:
		return -ENOTTY;
	case WDIOC_GETSUPPORT:
		return copy_to_user((struct watchdog_info __user *)arg, &ident,
							sizeof(ident));
	case WDIOC_GETSTATUS:
		return put_user(0, (int __user *)arg);
	case WDIOC_GETBOOTSTATUS:
		if (cpu_is_omap16xx())
			return put_user(omap_readw(ARM_SYSST),
					(int __user *)arg);
		if (cpu_is_omap24xx())
			return put_user(omap_prcm_get_reset_sources(),
					(int __user *)arg);
	case WDIOC_KEEPALIVE:
		retu_modify_counter(RETU_WDT_MAX_TIMER);
		break;
	case WDIOC_SETTIMEOUT:
		if (get_user(new_margin, (int __user *)arg))
			return -EFAULT;
		retu_modify_counter(new_margin);
		/* Fall through */
	case WDIOC_GETTIMEOUT:
		return put_user(period_val, (int __user *)arg);
	}

	return 0;
}

/* Start kicking retu watchdog until user space starts doing the kicking */
static int __init retu_wdt_ping(void)
{
	int r;

	r = retu_get_status();
	if (!r)
		return -ENODEV;

#ifdef CONFIG_WATCHDOG_NOWAYOUT
	retu_modify_counter(RETU_WDT_MAX_TIMER);
#else
	retu_wdt_set_ping_timer(1);
#endif

	return 0;
}
late_initcall(retu_wdt_ping);

static const struct file_operations retu_wdt_fops = {
	.owner = THIS_MODULE,
	.write = retu_wdt_write,
	.ioctl = retu_wdt_ioctl,
	.open = retu_wdt_open,
	.release = retu_wdt_release,
};

/*----------------------------------------------------------------------------*/

static int __devinit retu_wdt_probe(struct device *dev)
{
	struct retu_wdt_dev *wdev;
	int ret;

	wdev = kzalloc(sizeof(struct retu_wdt_dev), GFP_KERNEL);
	if (!wdev)
		return -ENOMEM;

	wdev->users = 0;

	ret = device_create_file(dev, &dev_attr_period);
	if (ret) {
		printk(KERN_ERR "retu_wdt_probe: Error creating "
					"sys device file: period\n");
		goto free1;
	}

	ret = device_create_file(dev, &dev_attr_counter);
	if (ret) {
		printk(KERN_ERR "retu_wdt_probe: Error creating "
					"sys device file: counter\n");
		goto free2;
	}

	dev_set_drvdata(dev, wdev);
	retu_wdt = wdev;
	wdev->retu_wdt_miscdev.parent = dev;
	wdev->retu_wdt_miscdev.minor = WATCHDOG_MINOR;
	wdev->retu_wdt_miscdev.name = "watchdog";
	wdev->retu_wdt_miscdev.fops = &retu_wdt_fops;

	ret = misc_register(&(wdev->retu_wdt_miscdev));
	if (ret)
		goto free3;

	setup_timer(&wdev->ping_timer, retu_wdt_set_ping_timer, 1);

	/* Kick the watchdog for kernel booting to finish */
	retu_modify_counter(RETU_WDT_MAX_TIMER);

	return 0;

free3:
	device_remove_file(dev, &dev_attr_counter);

free2:
	device_remove_file(dev, &dev_attr_period);
free1:
	kfree(wdev);

	return ret;
}

static int __devexit retu_wdt_remove(struct device *dev)
{
	struct retu_wdt_dev *wdev;

	wdev = dev_get_drvdata(dev);
	misc_deregister(&(wdev->retu_wdt_miscdev));
	device_remove_file(dev, &dev_attr_period);
	device_remove_file(dev, &dev_attr_counter);
	kfree(wdev);

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

	ret = retu_get_status();
	if (!ret)
		return -ENODEV;

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

