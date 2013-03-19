/*
 * Driver for beaglebone Geiger cape
 *
 * Copyright (C) 2012 Pantelis Antoniou <panto@antoniou-consulting.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/consumer.h>
#include <linux/atomic.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/clkdev.h>
#include <linux/pwm.h>
#include <linux/math64.h>
#include <linux/atomic.h>
#include <linux/leds.h>
#include <linux/input/ti_am335x_tsc.h>
#include <linux/platform_data/ti_am335x_adc.h>
#include <linux/mfd/ti_am335x_tscadc.h>
#include <linux/iio/iio.h>
#include <linux/iio/machine.h>
#include <linux/iio/consumer.h>

/* fwd decl. */
struct bone_geiger_info {
	struct platform_device *pdev;
	struct pwm_device *pwm_dev;
	int pwm_frequency;
	int pwm_duty_cycle;
	int run;
	atomic64_t counter;
	int event_gpio;
	int event_irq;
	struct led_trigger *event_led;		/* event detect */
	struct led_trigger *run_led;		/* running      */
	unsigned long event_blink_delay;
	struct sysfs_dirent *counter_sd;	/* notifier */
	const char *vsense_name;
	unsigned int vsense_scale;
	struct iio_channel *vsense_channel;
};

static const struct of_device_id bonegeiger_of_match[] = {
	{
		.compatible = "bone-cape-geiger",
	},
	{ },
};
MODULE_DEVICE_TABLE(of, bonegeiger_of_match);


#define	DEFAULT_VSENSE_SCALE	37325		/* 373.25 */

static int bonegeiger_start(struct platform_device *pdev)
{
	struct bone_geiger_info *info = platform_get_drvdata(pdev);
	int duty, period;

	if (info->run != 0)
		return 0;

	/* checks */
	if (info->pwm_frequency < 1000 || info->pwm_frequency > 50000) {
		dev_err(&pdev->dev, "Cowardly refusing to use a "
				"frequency of %d\n",
				info->pwm_frequency);
		return -EINVAL;
	}
	if (info->pwm_duty_cycle > 80) {
		dev_err(&pdev->dev, "Cowardly refusing to use a "
				"duty cycle of %d\n",
				info->pwm_duty_cycle);
		return -EINVAL;
	}

	period = div_u64(1000000000LLU, info->pwm_frequency);
	duty = (period * info->pwm_duty_cycle) / 100;

	dev_info(&pdev->dev, "starting geiger tube with "
			"duty=%duns period=%dus\n",
			duty, period);

	pwm_config(info->pwm_dev, duty, period);
	pwm_enable(info->pwm_dev);

	info->run = 1;
	led_trigger_event(info->run_led, LED_FULL);

	return 0;
}

static int bonegeiger_stop(struct platform_device *pdev)
{
	struct bone_geiger_info *info = platform_get_drvdata(pdev);

	if (info->run == 0)
		return 0;

	dev_info(&pdev->dev, "disabling geiger tube\n");
	pwm_config(info->pwm_dev, 0, 50000);	/* 0% duty cycle, 20KHz */
	pwm_disable(info->pwm_dev);

	info->run = 0;
	led_trigger_event(info->run_led, LED_OFF);

	return 0;
}

static ssize_t bonegeiger_show_run(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bone_geiger_info *info = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", info->run);
}

static ssize_t bonegeiger_store_run(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	int run, err;

	if (sscanf(buf, "%i", &run) != 1)
		return -EINVAL;

	if (run)
		err = bonegeiger_start(pdev);
	else
		err = bonegeiger_stop(pdev);

	return err ? err : count;
}

static ssize_t bonegeiger_show_counter(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bone_geiger_info *info = platform_get_drvdata(pdev);

	return sprintf(buf, "%llu\n", atomic64_read(&info->counter));
}

static ssize_t bonegeiger_store_counter(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bone_geiger_info *info = platform_get_drvdata(pdev);

	atomic64_set(&info->counter, 0);	/* just reset */
	return count;
}

static ssize_t bonegeiger_show_vsense(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bone_geiger_info *info = platform_get_drvdata(pdev);
	int ret, val;
	u32 mvolts;

	ret = iio_read_channel_raw(info->vsense_channel, &val);
	if (ret < 0)
		return ret;

	/* V = (1800 / 4096) * val * scale) = (1.8 * val * scale / 4096) */
	mvolts = div_u64(1800 * info->vsense_scale * (u64)val, 4096 * 100);

	return sprintf(buf, "%d\n", mvolts);
}

static DEVICE_ATTR(run, S_IRUGO | S_IWUSR,
		bonegeiger_show_run, bonegeiger_store_run);
static DEVICE_ATTR(counter, S_IRUGO | S_IWUSR,
		bonegeiger_show_counter, bonegeiger_store_counter);
static DEVICE_ATTR(vsense, S_IRUGO,
		bonegeiger_show_vsense, NULL);

static int bonegeiger_sysfs_register(struct platform_device *pdev)
{
	int err;

	err = device_create_file(&pdev->dev, &dev_attr_run);
	if (err != 0)
		goto err_no_run;

	err = device_create_file(&pdev->dev, &dev_attr_counter);
	if (err != 0)
		goto err_no_counter;

	err = device_create_file(&pdev->dev, &dev_attr_vsense);
	if (err != 0)
		goto err_no_vsense;

	return 0;

err_no_vsense:
	device_remove_file(&pdev->dev, &dev_attr_counter);
err_no_counter:
	device_remove_file(&pdev->dev, &dev_attr_run);
err_no_run:
	return err;
}

static void bonegeiger_sysfs_unregister(struct platform_device *pdev)
{
	device_remove_file(&pdev->dev, &dev_attr_vsense);
	device_remove_file(&pdev->dev, &dev_attr_counter);
	device_remove_file(&pdev->dev, &dev_attr_run);
}

static irqreturn_t bonegeiger_irq_handler(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct bone_geiger_info *info = platform_get_drvdata(pdev);

	atomic64_inc(&info->counter);

	led_trigger_blink_oneshot(info->event_led,
		  &info->event_blink_delay, &info->event_blink_delay, 0);

	sysfs_notify_dirent(info->counter_sd);

	return IRQ_HANDLED;
}

static int bonegeiger_probe(struct platform_device *pdev)
{
	struct bone_geiger_info *info;
	struct pinctrl *pinctrl;
	struct device_node *pnode = pdev->dev.of_node;
	phandle phandle;
	u32 val;
	int err;

	/* we only support OF */
	if (pnode == NULL) {
		dev_err(&pdev->dev, "No platform of_node!\n");
		return -ENODEV;
	}

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (info == NULL) {
		dev_err(&pdev->dev, "Failed to allocate info\n");
		err = -ENOMEM;
		goto err_no_mem;
	}
	platform_set_drvdata(pdev, info);
	info->pdev = pdev;

	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(pinctrl))
		dev_warn(&pdev->dev,
			"pins are not configured from the driver\n");

	err = of_property_read_u32(pnode, "pwms", &val);
	if (err != 0) {
		dev_err(&pdev->dev, "unable to read pwm handle\n");
		goto err_no_pwm;
	}
	phandle = val;

	info->pwm_dev = of_pwm_request(pnode, NULL);
	if (IS_ERR(info->pwm_dev)) {
		dev_err(&pdev->dev, "unable to request PWM\n");
		err = PTR_ERR(info->pwm_dev);
		goto err_no_pwm;
	}

	if (of_property_read_u32(pnode, "pwm-frequency", &val) != 0) {
		val = 20000;
		dev_warn(&pdev->dev, "Could not read pwm-frequency property; "
				"using default %u\n",
				val);
	}
	info->pwm_frequency = val;

	if (of_property_read_u32(pnode, "pwm-duty-cycle", &val) != 0) {
		val = 60;
		dev_warn(&pdev->dev, "Could not read pwm-duty-cycle property; "
				"using default %u\n",
				val);
	}
	info->pwm_duty_cycle = val;

	info->event_gpio = of_get_gpio_flags(pnode, 0, NULL);
	if (IS_ERR_VALUE(info->event_gpio)) {
		dev_err(&pdev->dev, "unable to get event GPIO\n");
		err = info->event_gpio;
		goto err_no_gpio;
	}

	err = gpio_request_one(info->event_gpio,
			GPIOF_DIR_IN | GPIOF_EXPORT,
			"bone-geiger-cape-event");
	if (err != 0) {
		dev_err(&pdev->dev, "failed to request event GPIO\n");
		goto err_no_gpio;
	}

	atomic64_set(&info->counter, 0);

	info->event_irq = gpio_to_irq(info->event_gpio);
	if (IS_ERR_VALUE(info->event_irq)) {
		dev_err(&pdev->dev, "unable to get event GPIO IRQ\n");
		err = info->event_irq;
		goto err_no_irq;
	}

	err = request_irq(info->event_irq, bonegeiger_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_SHARED,
			"bone-geiger-irq", pdev);
	if (err != 0) {
		dev_err(&pdev->dev, "unable to request irq\n");
		goto err_no_irq;
	}

	err = bonegeiger_sysfs_register(pdev);
	if (err != 0) {
		dev_err(&pdev->dev, "unable to register sysfs\n");
		goto err_no_sysfs;
	}

	info->counter_sd = sysfs_get_dirent(pdev->dev.kobj.sd, NULL, "counter");
	if (info->counter_sd == NULL) {
		dev_err(&pdev->dev, "unable to get dirent of counter\n");
		err = -ENODEV;
		goto err_no_counter_dirent;
	}

	led_trigger_register_simple("geiger-event", &info->event_led);
	led_trigger_register_simple("geiger-run", &info->run_led);

	led_trigger_event(info->run_led, LED_OFF);

	/* default */
	if (of_property_read_u32(pnode, "event-blink-delay", &val) != 0) {
		val = 30;
		dev_warn(&pdev->dev, "Could not read event-blink-delay "
				"property; using default %u\n",
					val);
	}
	info->event_blink_delay = val;

	/* default */
	if (of_property_read_string(pnode, "vsense-name", &info->vsense_name) != 0) {
		info->vsense_name = "AIN5";
		dev_warn(&pdev->dev, "Could not read vsense-name property; "
				"using default '%s'\n",
					info->vsense_name);
	}

	if (of_property_read_u32(pnode, "vsense-scale", &info->vsense_scale) != 0) {
		info->vsense_scale = DEFAULT_VSENSE_SCALE;
		dev_warn(&pdev->dev, "Could not read vsense-scale property; "
				"using default %u\n",
					info->vsense_scale);
	}

	if (info->vsense_scale == 0) {
		info->vsense_scale = DEFAULT_VSENSE_SCALE;
		dev_warn(&pdev->dev, "Invalid vsense-scale property; "
				"using default %u\n",
					info->vsense_scale);
	}

	info->vsense_channel = iio_channel_get(NULL, info->vsense_name);
	if (IS_ERR(info->vsense_channel)) {
		dev_err(&pdev->dev, "Could not get %s analog input\n",
					info->vsense_name);
		err = PTR_ERR(info->vsense_channel);
		goto err_no_vsense;
	}

	dev_info(&pdev->dev, "ready\n");

	err = bonegeiger_start(pdev);
	if (err != 0) {
		dev_err(&pdev->dev, "Could not start geiger device\n");
		goto err_no_start;
	}

	return 0;

err_no_start:
	iio_channel_release(info->vsense_channel);
err_no_vsense:
	led_trigger_unregister_simple(info->run_led);
	led_trigger_unregister_simple(info->event_led);
	sysfs_put(info->counter_sd);
err_no_counter_dirent:
	bonegeiger_sysfs_unregister(pdev);
err_no_sysfs:
	free_irq(info->event_irq, pdev);
err_no_irq:
	gpio_free(info->event_gpio);
err_no_gpio:
	pwm_put(info->pwm_dev);
err_no_pwm:
	devm_kfree(&pdev->dev, info);
err_no_mem:
	return err;
}

static int bonegeiger_remove(struct platform_device *pdev)
{
	struct bone_geiger_info *info = platform_get_drvdata(pdev);

	dev_info(&pdev->dev, "Removing geiger cape driver...\n");

	bonegeiger_stop(pdev);

	iio_channel_release(info->vsense_channel);
	led_trigger_unregister_simple(info->run_led);
	led_trigger_unregister_simple(info->event_led);
	sysfs_put(info->counter_sd);
	bonegeiger_sysfs_unregister(pdev);
	free_irq(info->event_irq, pdev);
	gpio_free(info->event_gpio);
	pwm_put(info->pwm_dev);

	return 0;
}

#ifdef CONFIG_PM
#ifdef CONFIG_PM_RUNTIME
static int bonegeiger_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bonegeiger_info *_dev = platform_get_drvdata(pdev);

	(void)_dev;
	return 0;
}

static int bonegeiger_runtime_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bonegeiger_info *_dev = platform_get_drvdata(pdev);

	(void)_dev;
	return 0;
}
#endif /* CONFIG_PM_RUNTIME */

static struct dev_pm_ops bonegeiger_pm_ops = {
	SET_RUNTIME_PM_OPS(bonegeiger_runtime_suspend,
			   bonegeiger_runtime_resume, NULL)
};
#define BONEGEIGER_PM_OPS (&bonegeiger_pm_ops)
#else
#define BONEGEIGER_PM_OPS NULL
#endif /* CONFIG_PM */

struct platform_driver bonegeiger_driver = {
	.probe		= bonegeiger_probe,
	.remove		= bonegeiger_remove,
	.driver = {
		.name		= "bone-cape-geiger",
		.owner		= THIS_MODULE,
		.pm		= BONEGEIGER_PM_OPS,
		.of_match_table	= bonegeiger_of_match,
	},
};

module_platform_driver(bonegeiger_driver);

MODULE_AUTHOR("Pantelis Antoniou");
MODULE_DESCRIPTION("Beaglebone geiger cape");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:bone-cape-geiger");
