/**
 * Retu/Vilma headset detection
 *
 * Copyright (C) 2006 Nokia Corporation
 *
 * Written by Juha Yrjölä
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/platform_device.h>

#include "retu.h"

#define RETU_ADC_CHANNEL_HOOKDET	0x05

#define RETU_HEADSET_KEY		KEY_PHONE

struct retu_headset {
	spinlock_t			lock;
	struct mutex			mutex;
	struct device			*dev;
	struct input_dev		*idev;
	unsigned			bias_enabled;
	unsigned			detection_enabled;
	unsigned			pressed;
	struct timer_list		enable_timer;
	struct timer_list		detect_timer;
	int				irq;
};

static void retu_headset_set_bias(struct retu_headset *hs, int enable)
{
	if (enable) {
		retu_set_clear_reg_bits(hs->dev, RETU_REG_AUDTXR,
					(1 << 0) | (1 << 1), 0);
		msleep(2);
		retu_set_clear_reg_bits(hs->dev, RETU_REG_AUDTXR,
				1 << 3, 0);
	} else {
		retu_set_clear_reg_bits(hs->dev, RETU_REG_AUDTXR, 0,
					(1 << 0) | (1 << 1) | (1 << 3));
	}
}

static void retu_headset_enable(struct retu_headset *hs)
{
	mutex_lock(&hs->mutex);
	if (!hs->bias_enabled) {
		hs->bias_enabled = 1;
		retu_headset_set_bias(hs, 1);
	}
	mutex_unlock(&hs->mutex);
}

static void retu_headset_disable(struct retu_headset *hs)
{
	mutex_lock(&hs->mutex);
	if (hs->bias_enabled) {
		hs->bias_enabled = 0;
		retu_headset_set_bias(hs, 0);
	}
	mutex_unlock(&hs->mutex);
}

static void retu_headset_det_enable(struct retu_headset *hs)
{
	mutex_lock(&hs->mutex);
	if (!hs->detection_enabled) {
		hs->detection_enabled = 1;
		retu_set_clear_reg_bits(hs->dev, RETU_REG_CC1,
				(1 << 10) | (1 << 8), 0);
	}
	mutex_unlock(&hs->mutex);
}

static void retu_headset_det_disable(struct retu_headset *hs)
{
	unsigned long flags;

	mutex_lock(&hs->mutex);
	if (hs->detection_enabled) {
		hs->detection_enabled = 0;
		del_timer_sync(&hs->enable_timer);
		del_timer_sync(&hs->detect_timer);
		spin_lock_irqsave(&hs->lock, flags);
		if (hs->pressed)
			input_report_key(hs->idev, RETU_HEADSET_KEY, 0);
		spin_unlock_irqrestore(&hs->lock, flags);
		retu_set_clear_reg_bits(hs->dev, RETU_REG_CC1, 0,
				(1 << 10) | (1 << 8));
	}
	mutex_unlock(&hs->mutex);
}

static ssize_t retu_headset_hookdet_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	int val;

	val = retu_read_adc(dev, RETU_ADC_CHANNEL_HOOKDET);
	return sprintf(buf, "%d\n", val);
}

static DEVICE_ATTR(hookdet, S_IRUGO, retu_headset_hookdet_show, NULL);

static ssize_t retu_headset_enable_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct retu_headset *hs = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", hs->bias_enabled);
}

static ssize_t retu_headset_enable_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct retu_headset *hs = dev_get_drvdata(dev);
	int enable;

	if (sscanf(buf, "%u", &enable) != 1)
		return -EINVAL;
	if (enable)
		retu_headset_enable(hs);
	else
	        retu_headset_disable(hs);
	return count;
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
		   retu_headset_enable_show, retu_headset_enable_store);

static ssize_t retu_headset_enable_det_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct retu_headset *hs = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", hs->detection_enabled);
}

static ssize_t retu_headset_enable_det_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	struct retu_headset *hs = dev_get_drvdata(dev);
	int enable;

	if (sscanf(buf, "%u", &enable) != 1)
		return -EINVAL;
	if (enable)
		retu_headset_det_enable(hs);
	else
	        retu_headset_det_disable(hs);
	return count;
}

static DEVICE_ATTR(enable_det, S_IRUGO | S_IWUSR | S_IWGRP,
		   retu_headset_enable_det_show,
		   retu_headset_enable_det_store);

static irqreturn_t retu_headset_hook_interrupt(int irq, void *_hs)
{
	struct retu_headset	*hs = _hs;
	unsigned long		flags;

	spin_lock_irqsave(&hs->lock, flags);
	if (!hs->pressed) {
		/* Headset button was just pressed down. */
		hs->pressed = 1;
		input_report_key(hs->idev, RETU_HEADSET_KEY, 1);
	}
	spin_unlock_irqrestore(&hs->lock, flags);
	retu_set_clear_reg_bits(hs->dev, RETU_REG_CC1, 0,
			(1 << 10) | (1 << 8));
	mod_timer(&hs->enable_timer, jiffies + msecs_to_jiffies(50));

	return IRQ_HANDLED;
}

static void retu_headset_enable_timer(unsigned long arg)
{
	struct retu_headset *hs = (struct retu_headset *) arg;

	retu_set_clear_reg_bits(hs->dev, RETU_REG_CC1,
			(1 << 10) | (1 << 8), 0);
	mod_timer(&hs->detect_timer, jiffies + msecs_to_jiffies(350));
}

static void retu_headset_detect_timer(unsigned long arg)
{
	struct retu_headset *hs = (struct retu_headset *) arg;
	unsigned long flags;

	spin_lock_irqsave(&hs->lock, flags);
	if (hs->pressed) {
		hs->pressed = 0;
		input_report_key(hs->idev, RETU_HEADSET_KEY, 0);
	}
	spin_unlock_irqrestore(&hs->lock, flags);
}

static int __init retu_headset_probe(struct platform_device *pdev)
{
	struct retu_headset *hs;
	int irq;
	int r;

	hs = kzalloc(sizeof(*hs), GFP_KERNEL);
	if (hs == NULL)
		return -ENOMEM;

	hs->dev = &pdev->dev;

	hs->idev = input_allocate_device();
	if (hs->idev == NULL) {
		r = -ENOMEM;
		goto err1;
	}
	hs->idev->name = "retu-headset";
	hs->idev->dev.parent = &pdev->dev;
	set_bit(EV_KEY, hs->idev->evbit);
	set_bit(RETU_HEADSET_KEY, hs->idev->keybit);
	r = input_register_device(hs->idev);
	if (r < 0)
		goto err2;

	r = device_create_file(&pdev->dev, &dev_attr_hookdet);
	if (r < 0)
		goto err3;
	r = device_create_file(&pdev->dev, &dev_attr_enable);
	if (r < 0)
		goto err4;
	r = device_create_file(&pdev->dev, &dev_attr_enable_det);
	if (r < 0)
		goto err5;
	platform_set_drvdata(pdev, hs);

	spin_lock_init(&hs->lock);
	mutex_init(&hs->mutex);
	setup_timer(&hs->enable_timer, retu_headset_enable_timer,
		    (unsigned long) hs);
	setup_timer(&hs->detect_timer, retu_headset_detect_timer,
		    (unsigned long) hs);

	irq = platform_get_irq(pdev, 0);
	hs->irq = irq;

	r = request_threaded_irq(irq, NULL, retu_headset_hook_interrupt, 0,
			"hookdet", hs);
	if (r != 0) {
		dev_err(&pdev->dev, "hookdet IRQ not available\n");
		goto err6;
	}

	return 0;
err6:
	device_remove_file(&pdev->dev, &dev_attr_enable_det);
err5:
	device_remove_file(&pdev->dev, &dev_attr_enable);
err4:
	device_remove_file(&pdev->dev, &dev_attr_hookdet);
err3:
	input_unregister_device(hs->idev);
err2:
	input_free_device(hs->idev);
err1:
	kfree(hs);
	return r;
}

static int retu_headset_remove(struct platform_device *pdev)
{
	struct retu_headset *hs = platform_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_hookdet);
	device_remove_file(&pdev->dev, &dev_attr_enable);
	device_remove_file(&pdev->dev, &dev_attr_enable_det);
	retu_headset_disable(hs);
	retu_headset_det_disable(hs);
	free_irq(hs->irq, hs);
	input_unregister_device(hs->idev);
	input_free_device(hs->idev);

	return 0;
}

static int retu_headset_suspend(struct platform_device *pdev,
				pm_message_t mesg)
{
	struct retu_headset *hs = platform_get_drvdata(pdev);

	mutex_lock(&hs->mutex);
	if (hs->bias_enabled)
		retu_headset_set_bias(hs, 0);
	mutex_unlock(&hs->mutex);

	return 0;
}

static int retu_headset_resume(struct platform_device *pdev)
{
	struct retu_headset *hs = platform_get_drvdata(pdev);

	mutex_lock(&hs->mutex);
	if (hs->bias_enabled)
		retu_headset_set_bias(hs, 1);
	mutex_unlock(&hs->mutex);

	return 0;
}

static struct platform_driver retu_headset_driver = {
	.remove		= retu_headset_remove,
	.suspend	= retu_headset_suspend,
	.resume		= retu_headset_resume,
	.driver		= {
		.name	= "retu-headset",
	},
};

static int __init retu_headset_init(void)
{
	return platform_driver_probe(&retu_headset_driver, retu_headset_probe);
}

static void __exit retu_headset_exit(void)
{
	platform_driver_unregister(&retu_headset_driver);
}

module_init(retu_headset_init);
module_exit(retu_headset_exit);

MODULE_DESCRIPTION("Retu/Vilma headset detection");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Juha Yrjölä");
