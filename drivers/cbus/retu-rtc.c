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
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/completion.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/rtc.h>
#include <linux/workqueue.h>

#include "cbus.h"
#include "retu.h"

struct retu_rtc {
	/* device lock */
	struct mutex		mutex;
	struct completion	sync;
	struct work_struct	work;
	struct device		*dev;
	struct rtc_device	*rtc;

	u16			alarm_expired;
	u16			reset_occurred;
};

#define work_to_rtc(r)	(container_of(r, struct retu_rtc, work))

/* This function provides syncronization with the RTCS interrupt handler */
static void retu_rtc_barrier(struct retu_rtc *rtc)
{
	INIT_COMPLETION(rtc->sync);
	retu_ack_irq(RETU_INT_RTCS);
	retu_enable_irq(RETU_INT_RTCS);
	wait_for_completion(&rtc->sync);
	retu_disable_irq(RETU_INT_RTCS);
}

static void retu_rtc_do_reset(struct retu_rtc *rtc)
{
	u16 ccr1;

	ccr1 = retu_read_reg(RETU_REG_CC1);
	/* RTC in reset */
	retu_write_reg(RETU_REG_CC1, ccr1 | 0x0001);
	/* RTC in normal operating mode */
	retu_write_reg(RETU_REG_CC1, ccr1 & ~0x0001);

	retu_rtc_barrier(rtc);
	/* Disable alarm and RTC WD */
	retu_write_reg(RETU_REG_RTCHMAR, 0x7f3f);
	/* Set Calibration register to default value */
	retu_write_reg(RETU_REG_RTCCALR, 0x00c0);

	rtc->alarm_expired = 0;
	rtc->reset_occurred = 1;
}

static void retu_rtca_disable(struct retu_rtc *rtc)
{
	retu_disable_irq(RETU_INT_RTCA);
	rtc->alarm_expired = 1;
	retu_rtc_barrier(rtc);
	retu_write_reg(RETU_REG_RTCHMAR, (24 << 8) | 60);
}

static void retu_rtca_expired(struct work_struct *work)
{
	struct retu_rtc		*rtc = work_to_rtc(work);

	retu_rtca_disable(rtc);
}

/*
 * RTCHMR RTCHMAR RTCCAL must be accessed within 0.9 s since the seconds
 * interrupt has been signaled in the IDR register
 */
static void retu_rtcs_interrupt(unsigned long _rtc)
{
	struct retu_rtc		*rtc = (struct retu_rtc *) _rtc;

	retu_ack_irq(RETU_INT_RTCS);
	complete_all(&rtc->sync);
}

static void retu_rtca_interrupt(unsigned long _rtc)
{
	struct retu_rtc		*rtc = (struct retu_rtc *) _rtc;

	retu_ack_irq(RETU_INT_RTCA);
	schedule_work(&rtc->work);
}

static int retu_rtc_init_irq(struct retu_rtc *rtc)
{
	int ret;

	ret = retu_request_irq(RETU_INT_RTCS, retu_rtcs_interrupt,
			(unsigned long) rtc, "RTCS");
	if (ret != 0)
		return ret;
	/*
	 * We will take care of enabling and disabling the interrupt
	 * elsewhere, so leave it off by default..
	 */
	retu_disable_irq(RETU_INT_RTCS);

	ret = retu_request_irq(RETU_INT_RTCA, retu_rtca_interrupt,
			(unsigned long) rtc, "RTCA");
	if (ret != 0) {
		retu_free_irq(RETU_INT_RTCS);
		return ret;
	}
	retu_disable_irq(RETU_INT_RTCA);

	return 0;
}

static int retu_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	struct retu_rtc		*rtc = dev_get_drvdata(dev);
	u16			chmar;

	mutex_lock(&rtc->mutex);

	chmar = ((alm->time.tm_hour & 0x1f) << 8) | (alm->time.tm_min & 0x3f);
	retu_write_reg(RETU_REG_RTCHMAR, chmar);

	mutex_unlock(&rtc->mutex);

	return 0;
}

static int retu_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	struct retu_rtc		*rtc = dev_get_drvdata(dev);
	u16			chmar;

	mutex_lock(&rtc->mutex);

	chmar = retu_read_reg(RETU_REG_RTCHMAR);

	alm->time.tm_hour	= (chmar >> 8) & 0x1f;
	alm->time.tm_min	= chmar & 0x3f;
	alm->enabled		= !!rtc->alarm_expired;

	mutex_unlock(&rtc->mutex);

	return 0;
}

static int retu_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct retu_rtc		*rtc = dev_get_drvdata(dev);
	u16			dsr;
	u16			hmr;

	dsr = ((tm->tm_mday & 0xff) << 8) | (tm->tm_hour & 0xff);
	hmr = ((tm->tm_min & 0xff) << 8) | (tm->tm_sec & 0xff);

	mutex_lock(&rtc->mutex);

	retu_write_reg(RETU_REG_RTCDSR, dsr);
	retu_write_reg(RETU_REG_RTCHMR, hmr);

	mutex_unlock(&rtc->mutex);

	return 0;
}

static int retu_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct retu_rtc		*rtc = dev_get_drvdata(dev);
	u16			dsr;
	u16			hmr;

	/*
	 * DSR holds days and hours
	 * HMR hols minutes and seconds
	 *
	 * both are 16 bit registers with 8-bit for each field.
	 */

	mutex_lock(&rtc->mutex);

	dsr	= retu_read_reg(RETU_REG_RTCDSR);
	hmr	= retu_read_reg(RETU_REG_RTCHMR);

	tm->tm_sec	= hmr & 0xff;
	tm->tm_min	= hmr >> 8;
	tm->tm_hour	= dsr & 0xff;
	tm->tm_mday	= dsr >> 8;

	mutex_unlock(&rtc->mutex);

	return 0;
}

#ifdef CONFIG_RTC_INTF_DEV

static int retu_rtc_ioctl(struct device *dev, unsigned int cmd,
		unsigned long arg)
{
	struct retu_rtc		*rtc = dev_get_drvdata(dev);

	mutex_lock(&rtc->mutex);

	switch (cmd) {
	case RTC_AIE_OFF:
		retu_disable_irq(RETU_INT_RTCA);
		break;
	case RTC_AIE_ON:
		retu_enable_irq(RETU_INT_RTCA);
		break;
	case RTC_UIE_OFF:
		retu_disable_irq(RETU_INT_RTCS);
		break;
	case RTC_UIE_ON:
		retu_enable_irq(RETU_INT_RTCS);
		break;
	default:
		return -ENOIOCTLCMD;
	}

	mutex_unlock(&rtc->mutex);

	return 0;
}
#else
#define retu_rtc_ioctl	NULL
#endif

static struct rtc_class_ops retu_rtc_ops = {
	.ioctl			= retu_rtc_ioctl,
	.read_time		= retu_rtc_read_time,
	.set_time		= retu_rtc_set_time,
	.read_alarm		= retu_rtc_read_alarm,
	.set_alarm		= retu_rtc_set_alarm,
};

static int __init retu_rtc_probe(struct platform_device *pdev)
{
	struct retu_rtc		*rtc;
	int			r;

	rtc = kzalloc(sizeof(*rtc), GFP_KERNEL);
	if (!rtc) {
		dev_err(&pdev->dev, "not enough memory\n");
		r = -ENOMEM;
		goto err0;
	}

	rtc->dev = &pdev->dev;
	platform_set_drvdata(pdev, rtc);
	INIT_WORK(&rtc->work, retu_rtca_expired);
	mutex_init(&rtc->mutex);

	r = retu_get_status();
	if (!r) {
		dev_err(&pdev->dev, "retu not initialized\n");
		goto err1;
	}

	rtc->alarm_expired = retu_read_reg(RETU_REG_IDR) &
		(0x1 << RETU_INT_RTCA);

	r = retu_rtc_init_irq(rtc);
	if (r < 0) {
		dev_err(&pdev->dev, "failed to request retu irq\n");
		goto err1;
	}

	/* If the calibration register is zero, we've probably lost
	 * power */
	if (retu_read_reg(RETU_REG_RTCCALR) & 0x00ff)
		rtc->reset_occurred = 0;
	else
		retu_rtc_do_reset(rtc);


	rtc->rtc = rtc_device_register(pdev->name, &pdev->dev, &
			retu_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc->rtc)) {
		dev_err(&pdev->dev, "can't register RTC device\n");
		goto err2;
	}

	return 0;

err2:
	retu_disable_irq(RETU_INT_RTCS);
	retu_disable_irq(RETU_INT_RTCA);
	retu_free_irq(RETU_INT_RTCS);
	retu_free_irq(RETU_INT_RTCA);

err1:
	kfree(rtc);

err0:
	return r;
}

static int __devexit retu_rtc_remove(struct platform_device *pdev)
{
	struct retu_rtc		*rtc = platform_get_drvdata(pdev);

	retu_disable_irq(RETU_INT_RTCS);
	retu_disable_irq(RETU_INT_RTCA);
	retu_free_irq(RETU_INT_RTCS);
	retu_free_irq(RETU_INT_RTCA);
	rtc_device_unregister(rtc->rtc);
	kfree(rtc);

	return 0;
}

static struct platform_driver retu_rtc_driver = {
	.remove		= __exit_p(retu_rtc_remove),
	.driver		= {
		.name	= "retu-rtc",
	},
};

static int __init retu_rtc_init(void)
{
	return platform_driver_probe(&retu_rtc_driver, retu_rtc_probe);
}

static void __exit retu_rtc_exit(void)
{
	platform_driver_unregister(&retu_rtc_driver);
}

module_init(retu_rtc_init);
module_exit(retu_rtc_exit);

MODULE_DESCRIPTION("Retu RTC");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Paul Mundt");
MODULE_AUTHOR("Igor Stoppa");
MODULE_AUTHOR("Felipe Balbi <felipe.balbi@nokia.com>");

