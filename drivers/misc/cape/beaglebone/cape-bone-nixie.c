/*
 * Driver for beaglebone Nixie cape
 *
 *  Copyright (C) 2013 Matt Ranostay <mranostay@gmail.com>
 *
 * Based on original work by
 *  Copyright (C) 2012 Pantelis Antoniou <panto@antoniou-consulting.com>
 *  Copyright (C) 2012 Texas Instruments Inc.
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
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/consumer.h>
#include <asm/barrier.h>
#include <linux/pwm.h>
#include <linux/leds.h>

enum {
	BLANK_PWM_CHAN	= 0,
	BOOST_PWM_CHAN	= 1,
};

struct bone_nixie_info {
	struct platform_device *pdev;
	struct pwm_device *pwm_dev;
	struct led_trigger *run_led;		/* running */

	int pwm_frequency;
	int pwm_duty_cycle;
	int pwm_period;
	int brightness;
	int run;
};

static const struct of_device_id bonenixie_of_match[] = {
	{
		.compatible = "bone-cape-nixie",
	},
	{ },
};
MODULE_DEVICE_TABLE(of, bonenixie_of_match);

/*
 * XXX: PWM subsystem doesn't allow easily selecting multiple channels on
 *	the same chip... so we hack it.. this shouldn't be here long term
 */

static void nixie_pwm_config(struct bone_nixie_info *info,
			     int duty, int chan)
{
	struct pwm_device *pwm = info->pwm_dev;

	duty = (info->pwm_period * duty) / 100;

	pwm->hwpwm = chan;
	pwm_config(pwm, duty, info->pwm_period);

	pwm->chip->ops->enable(pwm->chip, pwm);
}

static void nixie_pwm_disable(struct pwm_device *pwm, int chan)
{
	pwm->hwpwm = chan;
	pwm->chip->ops->disable(pwm->chip, pwm);
}

static int bonenixie_start(struct platform_device *pdev)
{
	struct bone_nixie_info *info = platform_get_drvdata(pdev);
	int duty;

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

	info->pwm_period = div_u64(1000000000LLU, info->pwm_frequency);
	duty = (info->pwm_period * info->pwm_duty_cycle) / 100;

	dev_info(&pdev->dev, "starting nixie tube with "
			" boost converter duty=%duns period=%dus"
			" default brightness percent=%d\n",
			duty, info->pwm_period, info->brightness);

	nixie_pwm_config(info, 100 - info->brightness, BLANK_PWM_CHAN);
	nixie_pwm_config(info, info->pwm_duty_cycle, BOOST_PWM_CHAN);

	info->run = 1;
	led_trigger_event(info->run_led, LED_FULL);

	return 0;
}

static int bonenixie_stop(struct platform_device *pdev)
{
	struct bone_nixie_info *info = platform_get_drvdata(pdev);

	if (info->run == 0)
		return 0;

	dev_info(&pdev->dev, "disabling nixie tube\n");

	nixie_pwm_config(info, 0, BLANK_PWM_CHAN);	/* 0% duty cycle */
	nixie_pwm_disable(info->pwm_dev, BLANK_PWM_CHAN);

	nixie_pwm_config(info, 0, BOOST_PWM_CHAN);	/* 0% duty cycle */
	nixie_pwm_disable(info->pwm_dev, BOOST_PWM_CHAN);

	info->run = 0;
	led_trigger_event(info->run_led, LED_OFF);

	return 0;
}

static ssize_t bonenixie_show_brightness(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bone_nixie_info *info = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", info->brightness);
}


static ssize_t bonenixie_store_brightness(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bone_nixie_info *info = platform_get_drvdata(pdev);
	int brightness;

	if (sscanf(buf, "%d", &brightness) != 1)
		return -EINVAL;

	if (brightness < 0 || brightness > 100)
		return -EINVAL;

	info->brightness = brightness;
	nixie_pwm_config(info, 100 - brightness, BLANK_PWM_CHAN);

	return count;
}


static ssize_t bonenixie_show_run(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bone_nixie_info *info = platform_get_drvdata(pdev);

	return sprintf(buf, "%d\n", info->run);
}


static ssize_t bonenixie_store_run(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	int run, err;

	if (sscanf(buf, "%i", &run) != 1)
		return -EINVAL;

	if (run)
		err = bonenixie_start(pdev);
	else
		err = bonenixie_stop(pdev);

	return err ? err : count;
}

static DEVICE_ATTR(brightness, S_IRUGO | S_IWUSR,
		bonenixie_show_brightness, bonenixie_store_brightness);

static DEVICE_ATTR(run, S_IRUGO | S_IWUSR,
		bonenixie_show_run, bonenixie_store_run);

static int bonenixie_sysfs_register(struct platform_device *pdev)
{
	int err;

	err = device_create_file(&pdev->dev, &dev_attr_run);
	if (err != 0)
		goto err_no_run;

	err = device_create_file(&pdev->dev, &dev_attr_brightness);
	if (err != 0)
		goto err_no_brightness;

	return 0;

err_no_brightness:
	device_remove_file(&pdev->dev, &dev_attr_run);
err_no_run:
	return err;
}

static void bonenixie_sysfs_unregister(struct platform_device *pdev)
{
	device_remove_file(&pdev->dev, &dev_attr_brightness);
	device_remove_file(&pdev->dev, &dev_attr_run);
}

static int bonenixie_probe(struct platform_device *pdev)
{
	struct bone_nixie_info *info;
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
		val = 9250;
		dev_warn(&pdev->dev, "Could not read pwm-frequency property; "
				"using default %u\n",
				val);
	}
	info->pwm_frequency = val;

	if (of_property_read_u32(pnode, "pwm-duty-cycle", &val) != 0) {
		val = 35;
		dev_warn(&pdev->dev, "Could not read pwm-duty-cycle property; "
				"using default %u\n",
				val);
	}
	info->pwm_duty_cycle = val;

	if (of_property_read_u32(pnode, "default-brightness", &val) != 0) {
		val = 80;
		dev_warn(&pdev->dev, "Could not read default-brightness property; "
				"using default %u\n",
				val);
	}
	info->brightness = val;

	err = bonenixie_sysfs_register(pdev);
	if (err != 0) {
		dev_err(&pdev->dev, "unable to register sysfs\n");
		goto err_no_sysfs;
	}

	led_trigger_register_simple("nixie-run", &info->run_led);
	led_trigger_event(info->run_led, LED_OFF);

	dev_info(&pdev->dev, "ready\n");

	err = bonenixie_start(pdev);
	if (err != 0) {
		dev_err(&pdev->dev, "Could not start nixie device\n");
		goto err_no_start;
	}

	return 0;

err_no_start:
	led_trigger_unregister_simple(info->run_led);
	bonenixie_sysfs_unregister(pdev);
err_no_sysfs:
	/* fall-through */
err_no_pwm:
	devm_kfree(&pdev->dev, info);
err_no_mem:
	return err;
}

static int bonenixie_remove(struct platform_device *pdev)
{
	struct bone_nixie_info *info = platform_get_drvdata(pdev);

	dev_info(&pdev->dev, "Remove nixie cape driver...\n");

	bonenixie_stop(pdev);

	led_trigger_unregister_simple(info->run_led);
	bonenixie_sysfs_unregister(pdev);

	return 0;
}

#ifdef CONFIG_PM
#ifdef CONFIG_PM_RUNTIME
static int bonenixie_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bone_nixie_info *_dev = platform_get_drvdata(pdev);

	(void)_dev;
	return 0;
}

static int bonenixie_runtime_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bone_nixie_info *_dev = platform_get_drvdata(pdev);

	(void)_dev;
	return 0;
}
#endif /* CONFIG_PM_RUNTIME */

static struct dev_pm_ops bonenixie_pm_ops = {
	SET_RUNTIME_PM_OPS(bonenixie_runtime_suspend,
			   bonenixie_runtime_resume, NULL)
};
#define BONENIXIE_PM_OPS (&bonenixie_pm_ops)
#else
#define BONENIXIE_PM_OPS NULL
#endif /* CONFIG_PM */


struct platform_driver bonenixie_driver = {
	.probe		= bonenixie_probe,
	.remove		= bonenixie_remove,
	.driver = {
		.name		= "bone-cape-nixie",
		.owner		= THIS_MODULE,
		.pm		= BONENIXIE_PM_OPS,
		.of_match_table = bonenixie_of_match,
	},
};

module_platform_driver(bonenixie_driver);

MODULE_AUTHOR("Matt Ranostay");
MODULE_DESCRIPTION("Beaglebone nixie cape");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:bone-nixie-cape");
