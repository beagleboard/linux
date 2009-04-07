/**
 * drivers/cbus/retu-rtc.c
 *
 * Support for Retu RTC
 *
 * Copyright (C) 2004, 2005 Nokia Corporation
 *
 * Written by Paul Mundt <paul.mundt@nokia.com> and
 *            Igor Stoppa <igor.stoppa@nokia.com>
 *
 * The Retu RTC is essentially a partial read-only RTC that gives us Retu's
 * idea of what time actually is. It's left as a userspace excercise to map
 * this back to time in the real world and ensure that calibration settings
 * are sane to compensate for any horrible drift (on account of not being able
 * to set the clock to anything).
 *
 * Days are semi-writeable. Namely, Retu will only track 255 days for us
 * consecutively, after which the counter is explicitly stuck at 255 until
 * someone comes along and clears it with a write. In the event that no one
 * comes along and clears it, we no longer have any idea what day it is.
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

#include <linux/device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/completion.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>

#include "cbus.h"
#include "retu.h"

static struct mutex retu_rtc_mutex;
static u16 retu_rtc_alarm_expired;
static u16 retu_rtc_reset_occurred;

static DECLARE_COMPLETION(retu_rtc_exited);
static DECLARE_COMPLETION(retu_rtc_sync);

static void retu_rtc_barrier(void);

static void retu_rtc_device_release(struct device *dev)
{
	complete(&retu_rtc_exited);
}

static ssize_t retu_rtc_time_show(struct device *dev, struct device_attribute *attr,
				  char *buf)
{
	u16 dsr, hmr, dsr2;

	mutex_lock(&retu_rtc_mutex);

	do {
		u16 dummy;

		/*
		 * Not being in_interrupt() for a retu rtc IRQ, we need to
		 * read twice for consistency..
		 */
		dummy	= retu_read_reg(RETU_REG_RTCDSR);
		dsr	= retu_read_reg(RETU_REG_RTCDSR);

		dummy	= retu_read_reg(RETU_REG_RTCHMR);
		hmr	= retu_read_reg(RETU_REG_RTCHMR);

		dummy	= retu_read_reg(RETU_REG_RTCDSR);
		dsr2	= retu_read_reg(RETU_REG_RTCDSR);
	} while ((dsr != dsr2));

	mutex_unlock(&retu_rtc_mutex);

	/*
	 * Format a 32-bit date-string for userspace
	 *
	 * days | hours | minutes | seconds
	 *
	 * 8 bits for each.
	 *
	 * This mostly sucks because days and seconds are tracked in RTCDSR
	 * while hours and minutes are tracked in RTCHMR. And yes, there
	 * really are no words that can describe an 8 bit day register (or
	 * rather, none that will be reprinted here).
	 */
	return sprintf(buf, "0x%08x\n", (((dsr >> 8) & 0xff) << 24) |
				        (((hmr >> 8) & 0x1f) << 16) |
					 ((hmr & 0x3f) << 8) | (dsr & 0x3f));
}

static ssize_t retu_rtc_time_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	mutex_lock(&retu_rtc_mutex);
	/*
	 * Writing anything to the day counter forces it to 0
	 * The seconds counter would be cleared by resetting the minutes counter,
	 * however this won't happen, since we are using the hh:mm counters as
	 * a set of free running counters and the day counter as a multiple
	 * overflow holder.
	 */

	/* Reset day counter, but keep Temperature Shutdown state */
	retu_write_reg(RETU_REG_RTCDSR,
		       retu_read_reg(RETU_REG_RTCDSR) & (1 << 6));

	mutex_unlock(&retu_rtc_mutex);

	return count;
}

static DEVICE_ATTR(time, S_IRUGO | S_IWUSR, retu_rtc_time_show,
		   retu_rtc_time_store);


static ssize_t retu_rtc_reset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/*
	 * Returns the status of the rtc
	 *
	 * 0: no reset has occurred or the status has been cleared
	 * 1: a reset has occurred
	 *
	 * RTC needs to be reset only when both main battery
	 * _AND_ backup battery are discharged
	 */
	return sprintf(buf, "%u\n", retu_rtc_reset_occurred);
}

static void retu_rtc_do_reset(void)
{
	u16 ccr1;

	ccr1 = retu_read_reg(RETU_REG_CC1);
	/* RTC in reset */
	retu_write_reg(RETU_REG_CC1, ccr1 | 0x0001);
	/* RTC in normal operating mode */
	retu_write_reg(RETU_REG_CC1, ccr1 & ~0x0001);

	retu_rtc_barrier();
	/* Disable alarm and RTC WD */
	retu_write_reg(RETU_REG_RTCHMAR, 0x7f3f);
	/* Set Calibration register to default value */
	retu_write_reg(RETU_REG_RTCCALR, 0x00c0);

	retu_rtc_alarm_expired = 0;
	retu_rtc_reset_occurred = 1;
}

static ssize_t retu_rtc_reset_store(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t count)
{
	unsigned choice;

	if(sscanf(buf, "%u", &choice) != 1)
		return count;
	mutex_lock(&retu_rtc_mutex);
	if (choice == 0)
		retu_rtc_reset_occurred = 0;
	else if (choice == 1)
		retu_rtc_do_reset();
	mutex_unlock(&retu_rtc_mutex);
	return count;
}

static DEVICE_ATTR(reset, S_IRUGO | S_IWUSR, retu_rtc_reset_show,
		   retu_rtc_reset_store);

static ssize_t retu_rtc_alarm_show(struct device *dev, struct device_attribute *attr,
				   char *buf)
{
	u16 chmar;
	ssize_t retval;

	mutex_lock(&retu_rtc_mutex);
	/*
	 * Format a 16-bit date-string for userspace
	 *
	 * hours | minutes
	 * 8 bits for each.
	 */
	chmar = retu_read_reg(RETU_REG_RTCHMAR);
	/* No shifting needed, only masking unrelated bits */
	retval = sprintf(buf, "0x%04x\n", chmar & 0x1f3f);
	mutex_unlock(&retu_rtc_mutex);

	return retval;
}

static ssize_t retu_rtc_alarm_store(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t count)
{
	u16 chmar;
	unsigned alrm;
	unsigned hours;
	unsigned minutes;

	mutex_lock(&retu_rtc_mutex);

	if(sscanf(buf, "%x", &alrm) != 1)
		return count;
	hours = (alrm >> 8) & 0x001f;
	minutes = (alrm >> 0) & 0x003f;
	if ((hours < 24 && minutes < 60) || (hours == 24 && minutes == 60)) {
		/*
		 * OK, the time format for the alarm is valid (including the
		 * disabling values)
		 */
		/* Keeps the RTC watchdog status */
		chmar = retu_read_reg(RETU_REG_RTCHMAR) & 0x6000;
		chmar |= alrm & 0x1f3f;	/* Stores the requested alarm */
		retu_rtc_barrier();
		retu_write_reg(RETU_REG_RTCHMAR, chmar);
		/* If the alarm is being disabled */
		if (hours == 24 && minutes == 60) {
			/* disable the interrupt */
			retu_disable_irq(RETU_INT_RTCA);
			retu_rtc_alarm_expired = 0;
		} else
			/* enable the interrupt */
			retu_enable_irq(RETU_INT_RTCA);
	}
	mutex_unlock(&retu_rtc_mutex);

	return count;
}

static DEVICE_ATTR(alarm, S_IRUGO | S_IWUSR, retu_rtc_alarm_show,
		   retu_rtc_alarm_store);

static ssize_t retu_rtc_alarm_expired_show(struct device *dev, struct device_attribute *attr,
					   char *buf)
{
	ssize_t retval;

	retval = sprintf(buf, "%u\n", retu_rtc_alarm_expired);

	return retval;
}

static ssize_t retu_rtc_alarm_expired_store(struct device *dev, struct device_attribute *attr,
					    const char *buf, size_t count)
{
	retu_rtc_alarm_expired = 0;

	return count;
}

static DEVICE_ATTR(alarm_expired, S_IRUGO | S_IWUSR, retu_rtc_alarm_expired_show,
		   retu_rtc_alarm_expired_store);


static ssize_t retu_rtc_cal_show(struct device *dev, struct device_attribute *attr,
				 char *buf)
{
	u16 rtccalr1;

	mutex_lock(&retu_rtc_mutex);
	rtccalr1 = retu_read_reg(RETU_REG_RTCCALR);
	mutex_unlock(&retu_rtc_mutex);

	/*
	 * Shows the status of the Calibration Register.
	 *
	 * Default, after power loss: 0x0000
	 * Default, for R&D: 0x00C0
	 * Default, for factory: 0x00??
	 *
	 */
	return sprintf(buf, "0x%04x\n", rtccalr1 & 0x00ff);
}

static ssize_t retu_rtc_cal_store(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t count)
{
	unsigned calibration_value;

	if (sscanf(buf, "%x", &calibration_value) != 1)
		return count;

	mutex_lock(&retu_rtc_mutex);
	retu_rtc_barrier();
	retu_write_reg(RETU_REG_RTCCALR, calibration_value & 0x00ff);
	mutex_unlock(&retu_rtc_mutex);

	return count;
}

static DEVICE_ATTR(cal, S_IRUGO | S_IWUSR, retu_rtc_cal_show,
		   retu_rtc_cal_store);

static struct platform_device retu_rtc_device;

static void retu_rtca_disable(void)
{
	retu_disable_irq(RETU_INT_RTCA);
	retu_rtc_alarm_expired = 1;
	retu_rtc_barrier();
	retu_write_reg(RETU_REG_RTCHMAR, (24 << 8) | 60);
}

static void retu_rtca_expired(struct work_struct *unused)
{
	retu_rtca_disable();
	sysfs_notify(&retu_rtc_device.dev.kobj, NULL, "alarm_expired");
}

DECLARE_WORK(retu_rtca_work, retu_rtca_expired);

/*
 * RTCHMR RTCHMAR RTCCAL must be accessed within 0.9 s since the seconds
 * interrupt has been signaled in the IDR register
 */
static void retu_rtcs_interrupt(unsigned long unused)
{
	retu_ack_irq(RETU_INT_RTCS);
	complete_all(&retu_rtc_sync);
}

static void retu_rtca_interrupt(unsigned long unused)
{
	retu_ack_irq(RETU_INT_RTCA);
	schedule_work(&retu_rtca_work);
}

static int retu_rtc_init_irq(void)
{
	int ret;

	ret = retu_request_irq(RETU_INT_RTCS, retu_rtcs_interrupt, 0, "RTCS");
	if (ret != 0)
		return ret;
	/*
	 * We will take care of enabling and disabling the interrupt
	 * elsewhere, so leave it off by default..
	 */
	retu_disable_irq(RETU_INT_RTCS);

	ret = retu_request_irq(RETU_INT_RTCA, retu_rtca_interrupt, 0, "RTCA");
	if (ret != 0) {
		retu_free_irq(RETU_INT_RTCS);
		return ret;
	}
	retu_disable_irq(RETU_INT_RTCA);

	return 0;
}


static int __devinit retu_rtc_probe(struct device *dev)
{
	int r;

	retu_rtc_alarm_expired = retu_read_reg(RETU_REG_IDR) &
					       (0x1 << RETU_INT_RTCA);

	if ((r = retu_rtc_init_irq()) != 0)
		return r;

	mutex_init(&retu_rtc_mutex);

	/* If the calibration register is zero, we've probably lost
	 * power */
	if (retu_read_reg(RETU_REG_RTCCALR) & 0x00ff)
		retu_rtc_reset_occurred = 0;
	else
		retu_rtc_do_reset();

	if ((r = device_create_file(dev, &dev_attr_time)) != 0)
		return r;
	else if ((r = device_create_file(dev, &dev_attr_reset)) != 0)
		goto err_unregister_time;
	else if ((r = device_create_file(dev, &dev_attr_alarm)) != 0)
		goto err_unregister_reset;
	else if ((r = device_create_file(dev, &dev_attr_alarm_expired)) != 0)
		goto err_unregister_alarm;
	else if ((r = device_create_file(dev, &dev_attr_cal)) != 0)
		goto err_unregister_alarm_expired;
	else
		return r;

err_unregister_alarm_expired:
	device_remove_file(dev, &dev_attr_alarm_expired);
err_unregister_alarm:
	device_remove_file(dev, &dev_attr_alarm);
err_unregister_reset:
	device_remove_file(dev, &dev_attr_reset);
err_unregister_time:
	device_remove_file(dev, &dev_attr_time);
	return r;
}

static int __devexit retu_rtc_remove(struct device *dev)
{
	retu_disable_irq(RETU_INT_RTCS);
	retu_free_irq(RETU_INT_RTCS);
	retu_free_irq(RETU_INT_RTCA);
	device_remove_file(dev, &dev_attr_cal);
	device_remove_file(dev, &dev_attr_alarm_expired);
	device_remove_file(dev, &dev_attr_alarm);
	device_remove_file(dev, &dev_attr_reset);
	device_remove_file(dev, &dev_attr_time);
	return 0;
}

static struct device_driver retu_rtc_driver = {
	.name		= "retu-rtc",
	.bus		= &platform_bus_type,
	.probe		= retu_rtc_probe,
	.remove		= __devexit_p(retu_rtc_remove),
};

static struct platform_device retu_rtc_device = {
	.name		= "retu-rtc",
	.id		= -1,
	.dev		= {
		.release	= retu_rtc_device_release,
	},
};

/* This function provides syncronization with the RTCS interrupt handler */
static void retu_rtc_barrier(void)
{
	INIT_COMPLETION(retu_rtc_sync);
	retu_ack_irq(RETU_INT_RTCS);
	retu_enable_irq(RETU_INT_RTCS);
	wait_for_completion(&retu_rtc_sync);
	retu_disable_irq(RETU_INT_RTCS);
}

static int __init retu_rtc_init(void)
{
	int ret;

	init_completion(&retu_rtc_exited);

	if ((ret = driver_register(&retu_rtc_driver)) != 0)
		return ret;

	if ((ret = platform_device_register(&retu_rtc_device)) != 0)
		goto err_unregister_driver;

	return 0;

err_unregister_driver:
	driver_unregister(&retu_rtc_driver);
	return ret;
}

static void __exit retu_rtc_exit(void)
{
	platform_device_unregister(&retu_rtc_device);
	driver_unregister(&retu_rtc_driver);

	wait_for_completion(&retu_rtc_exited);
}

module_init(retu_rtc_init);
module_exit(retu_rtc_exit);

MODULE_DESCRIPTION("Retu RTC");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Paul Mundt and Igor Stoppa");
