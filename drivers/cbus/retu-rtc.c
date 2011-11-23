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
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/rtc.h>

#include "cbus.h"
#include "retu.h"

struct retu_rtc {
	/* device lock */
	struct mutex		mutex;
	struct device		*dev;
	struct rtc_device	*rtc;

	u16			alarm_expired;
	int			irq_rtcs;
	int			irq_rtca;
};

static void retu_rtc_do_reset(struct retu_rtc *rtc)
{
	u16 ccr1;

	ccr1 = retu_read_reg(rtc->dev, RETU_REG_CC1);
	/* RTC in reset */
	retu_write_reg(rtc->dev, RETU_REG_CC1, ccr1 | 0x0001);
	/* RTC in normal operating mode */
	retu_write_reg(rtc->dev, RETU_REG_CC1, ccr1 & ~0x0001);

	/* Disable alarm and RTC WD */
	retu_write_reg(rtc->dev, RETU_REG_RTCHMAR, 0x7f3f);
	/* Set Calibration register to default value */
	retu_write_reg(rtc->dev, RETU_REG_RTCCALR, 0x00c0);

	rtc->alarm_expired = 0;
}

static irqreturn_t retu_rtc_interrupt(int irq, void *_rtc)
{
	struct retu_rtc		*rtc = _rtc;

	mutex_lock(&rtc->mutex);
	rtc->alarm_expired = 1;
	retu_write_reg(rtc->dev, RETU_REG_RTCHMAR, (24 << 8) | 60);
	mutex_unlock(&rtc->mutex);

	return IRQ_HANDLED;
}

static int retu_rtc_init_irq(struct retu_rtc *rtc)
{
	int irq;
	int ret;

	irq = platform_get_irq(to_platform_device(rtc->dev), 0);
	rtc->irq_rtcs = irq;

	irq = platform_get_irq(to_platform_device(rtc->dev), 1);
	rtc->irq_rtca = irq;

	ret = request_threaded_irq(rtc->irq_rtcs, NULL, retu_rtc_interrupt,
			0, "RTCS", rtc);
	if (ret != 0)
		return ret;

	ret = request_threaded_irq(rtc->irq_rtca, NULL, retu_rtc_interrupt,
			0, "RTCA", rtc);
	if (ret != 0) {
		free_irq(rtc->irq_rtcs, rtc);
		return ret;
	}

	return 0;
}

static int retu_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	struct retu_rtc		*rtc = dev_get_drvdata(dev);
	u16			chmar;

	mutex_lock(&rtc->mutex);

	chmar = ((alm->time.tm_hour & 0x1f) << 8) | (alm->time.tm_min & 0x3f);
	retu_write_reg(rtc->dev, RETU_REG_RTCHMAR, chmar);

	mutex_unlock(&rtc->mutex);

	return 0;
}

static int retu_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	struct retu_rtc		*rtc = dev_get_drvdata(dev);
	u16			chmar;

	mutex_lock(&rtc->mutex);

	chmar = retu_read_reg(rtc->dev, RETU_REG_RTCHMAR);

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

	retu_write_reg(rtc->dev, RETU_REG_RTCDSR, dsr);
	retu_write_reg(rtc->dev, RETU_REG_RTCHMR, hmr);

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

	dsr	= retu_read_reg(rtc->dev, RETU_REG_RTCDSR);
	hmr	= retu_read_reg(rtc->dev, RETU_REG_RTCHMR);

	tm->tm_sec	= hmr & 0xff;
	tm->tm_min	= hmr >> 8;
	tm->tm_hour	= dsr & 0xff;
	tm->tm_mday	= dsr >> 8;

	mutex_unlock(&rtc->mutex);

	return 0;
}

static struct rtc_class_ops retu_rtc_ops = {
	.read_time		= retu_rtc_read_time,
	.set_time		= retu_rtc_set_time,
	.read_alarm		= retu_rtc_read_alarm,
	.set_alarm		= retu_rtc_set_alarm,
};

static int __devinit retu_rtc_probe(struct platform_device *pdev)
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
	mutex_init(&rtc->mutex);

	rtc->alarm_expired = retu_read_reg(rtc->dev, RETU_REG_IDR) &
		(0x1 << RETU_INT_RTCA);

	r = retu_rtc_init_irq(rtc);
	if (r < 0) {
		dev_err(&pdev->dev, "failed to request retu irq\n");
		goto err1;
	}

	/* If the calibration register is zero, we've probably lost power */
	if (!(retu_read_reg(rtc->dev, RETU_REG_RTCCALR) & 0x00ff))
		retu_rtc_do_reset(rtc);

	rtc->rtc = rtc_device_register(pdev->name, &pdev->dev, &
			retu_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc->rtc)) {
		dev_err(&pdev->dev, "can't register RTC device\n");
		goto err2;
	}

	return 0;

err2:
	free_irq(rtc->irq_rtcs, rtc);
	free_irq(rtc->irq_rtca, rtc);

err1:
	kfree(rtc);

err0:
	return r;
}

static int __devexit retu_rtc_remove(struct platform_device *pdev)
{
	struct retu_rtc		*rtc = platform_get_drvdata(pdev);

	free_irq(rtc->irq_rtcs, rtc);
	free_irq(rtc->irq_rtca, rtc);
	rtc_device_unregister(rtc->rtc);
	kfree(rtc);

	return 0;
}

static struct platform_driver retu_rtc_driver = {
	.probe		= retu_rtc_probe,
	.remove		= __devexit_p(retu_rtc_remove),
	.driver		= {
		.name	= "retu-rtc",
	},
};

module_platform_driver(retu_rtc_driver);

MODULE_ALIAS("platform:retu-rtc");
MODULE_DESCRIPTION("Retu RTC");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Paul Mundt");
MODULE_AUTHOR("Igor Stoppa");
MODULE_AUTHOR("Felipe Balbi <felipe.balbi@nokia.com>");

