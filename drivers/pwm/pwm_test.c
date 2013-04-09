/*
 * PWM Test driver
 *
 * Copyright (C) 2012 Texas Instruments.
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
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/pinctrl/consumer.h>

struct pwm_test {
	struct pwm_device *pwm;
	struct class *pwm_test_class;
	int period;
	int duty;
	enum pwm_polarity polarity;
	int run;
};

static ssize_t pwm_test_show_duty(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct pwm_test *pwm_test = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", pwm_test->duty);
}

static ssize_t pwm_test_store_duty(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct pwm_test *pwm_test = dev_get_drvdata(dev);
	int duty;

	rc = kstrtoint(buf, 0, &duty);
	if (rc)
		return rc;

	if (duty < 0)
		return -EINVAL;

	rc = pwm_config(pwm_test->pwm, duty, pwm_test->period);
	if (rc) {
		dev_err(dev, "pwm_config() failed\n");
		return rc;
	}

	pwm_test->duty = duty;

	return count;
}

static ssize_t pwm_test_show_period(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct pwm_test *pwm_test = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", pwm_test->period);
}

static ssize_t pwm_test_store_period(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct pwm_test *pwm_test = dev_get_drvdata(dev);
	int period;

	rc = kstrtoint(buf, 0, &period);
	if (rc)
		return rc;

	if (period < 0)
		return -EINVAL;

	/* same period? just return */
	if (pwm_test->period == period)
		return count;

	rc = pwm_config(pwm_test->pwm, pwm_test->duty, period);
	if (rc) {
		dev_err(dev, "pwm_config() failed\n");
		return rc;
	}

	pwm_test->period = period;

	return count;
}

static ssize_t pwm_test_show_run(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct pwm_test *pwm_test = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", pwm_test->run);
}

static ssize_t pwm_test_store_run(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct pwm_test *pwm_test = dev_get_drvdata(dev);
	int run;

	rc = kstrtoint(buf, 0, &run);
	if (rc)
		return rc;

	/* only 0 & 1 allowed */
	if (run != 0 && run != 1)
		return -EINVAL;

	/* same state, don't bother */
	if (run == pwm_test->run)
		return count;

	if (run) {
		rc = pwm_enable(pwm_test->pwm);
		if (rc != 0) {
			dev_err(dev, "pwm_enable failed\n");
			return rc;
		}
	} else
		pwm_disable(pwm_test->pwm);

	pwm_test->run = run;

	return count;
}

static ssize_t pwm_test_show_polarity(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct pwm_test *pwm_test = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", pwm_test->polarity);
}

static ssize_t pwm_test_store_polarity(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct pwm_test *pwm_test = dev_get_drvdata(dev);
	int val;
	enum pwm_polarity polarity;

	rc = kstrtoint(buf, 0, &val);
	if (rc)
		return rc;

	/* only zero and one allowed */
	if (val != 0 && val != 1)
		return -EINVAL;

	polarity = val ? PWM_POLARITY_INVERSED : PWM_POLARITY_NORMAL;

	/* same? don't do anything */
	if (polarity == pwm_test->polarity)
		return count;

	/* polarity can only change when we stop the pwm */
	if (pwm_test->run)
		pwm_disable(pwm_test->pwm);

	rc = pwm_set_polarity(pwm_test->pwm, polarity);
	if (rc) {
		dev_err(dev, "pwm_set_polarity failed\n");
		if (pwm_test->run)
			pwm_enable(pwm_test->pwm);
		return rc;
	}

	if (pwm_test->run)
		pwm_enable(pwm_test->pwm);

	pwm_test->polarity = polarity;

	return count;
}

static DEVICE_ATTR(duty, S_IRUSR | S_IWUSR, pwm_test_show_duty, pwm_test_store_duty);
static DEVICE_ATTR(period, S_IRUSR | S_IWUSR, pwm_test_show_period, pwm_test_store_period);
static DEVICE_ATTR(polarity, S_IRUSR | S_IWUSR, pwm_test_show_polarity,
		pwm_test_store_polarity);
static DEVICE_ATTR(run, S_IRUSR | S_IWUSR , pwm_test_show_run, pwm_test_store_run);

static const struct attribute *pwm_attrs[] = {
	&dev_attr_duty.attr,
	&dev_attr_period.attr,
	&dev_attr_run.attr,
	&dev_attr_polarity.attr,
	NULL,
};

static const struct attribute_group pwm_device_attr_group = {
	.attrs = (struct attribute **) pwm_attrs,
};

static int pwm_test_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct pwm_test *pwm_test;
	struct of_phandle_args args;
	struct pinctrl *pinctrl;
	u32 val;
	int rc;

	if (node == NULL) {
		dev_err(dev, "Non DT platforms not supported\n");
		return -EINVAL;
	}

	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(pinctrl))
		dev_warn(&pdev->dev, "Unable to select pin group\n");

	pwm_test = devm_kzalloc(&pdev->dev, sizeof(*pwm_test), GFP_KERNEL);
	if (!pwm_test) {
		dev_err(&pdev->dev, "memory error\n");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, pwm_test);

	/* now do the probe time config */
	pwm_test->pwm = devm_pwm_get(&pdev->dev, NULL);
	if (IS_ERR(pwm_test->pwm)) {
		dev_err(dev, "unable to request PWM\n");
		return PTR_ERR(pwm_test->pwm);
	}

	rc = of_parse_phandle_with_args(node, "pwms", "#pwm-cells", 0, &args);
	if (rc != 0) {
		dev_err(dev, "of_parse_phandle_with_args() failed\n");
		return rc;
	}

	/* read the period */
	pwm_test->period = args.args[1];

	/* should be at least 2, but 3 is possible to store polarity */
	pwm_test->polarity = PWM_POLARITY_NORMAL;
	/* PWM_SPEC_POLARITY is (1 << 0) */
	if (args.args_count >= 3 && (args.args[2] & (1 << 0)) != 0)
		pwm_test->polarity = PWM_POLARITY_INVERSED;

	/* Determine the duty from the device tree */
	rc = of_property_read_u32(node, "duty", &val);
	if (rc != 0)
		val = 0;	/* default is 0 */
	pwm_test->duty = val;

	/* polarity is already set */
	rc = pwm_config(pwm_test->pwm, pwm_test->duty, pwm_test->period);
	if (rc) {
		dev_err(dev, "pwm_config() failed\n");
		return rc;
	}

	/* Determine running or not from the device tree */
	rc = of_property_read_u32(node, "enabled", &val);
	if (rc < 0)
		val = 0;	/* default is disabled */

	/* single bit */
	pwm_test->run = !!val;

	if (pwm_test->run) {
		rc = pwm_enable(pwm_test->pwm);
		if (rc < 0) {
			dev_err(dev, "pwm_enable failed\n");
			return rc;
		}
	}

	rc = sysfs_create_group(&dev->kobj, &pwm_device_attr_group);
	if (rc != 0) {
		dev_err(dev, "Unable to create sysfs entries\n");
		return rc;
	}

	return 0;
}

static int pwm_test_remove(struct platform_device *pdev)
{
	struct pwm_test *pwm_test = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &pwm_device_attr_group);

	if (pwm_test->run) {
		pwm_config(pwm_test->pwm, 0, 0x1000);
		pwm_disable(pwm_test->pwm);
	}

	devm_pwm_put(&pdev->dev, pwm_test->pwm);
	return 0;
}

static struct of_device_id pwm_test_of_match[] = {
	{ .compatible = "pwm_test" },
	{ }
};

MODULE_DEVICE_TABLE(of, pwm_test_of_match);

static struct platform_driver pwm_test_driver = {
	.driver		= {
		.name		= "pwm_test",
		.owner		= THIS_MODULE,
		.of_match_table	= of_match_ptr(pwm_test_of_match),
	},
	.probe		= pwm_test_probe,
	.remove		= pwm_test_remove,
};

module_platform_driver(pwm_test_driver);

MODULE_DESCRIPTION("pwm_test Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pwm_test");
