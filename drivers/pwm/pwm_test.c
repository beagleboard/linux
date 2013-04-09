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

struct pwm_test {
	struct pwm_device *pwm;
	int ret;
	struct class *pwm_test_class;
	unsigned long period, duty, run, polarity, config, requested;
	unsigned long period_s, duty_s, run_s, polarity_s, config_s, requested_s;
	struct device *dev;
};

static ssize_t pwm_test_show_duty(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct pwm_test *pwm_test = dev_get_drvdata(dev);
	return sprintf(buf, "%lu\n", pwm_test->duty);
}

static ssize_t pwm_test_store_duty(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct pwm_test *pwm_test = dev_get_drvdata(dev);

	rc = kstrtoul(buf, 0, &pwm_test->duty_s);
	if (rc)
		return rc;
	return count;
}

static ssize_t pwm_test_show_period(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct pwm_test *pwm_test = dev_get_drvdata(dev);

	return sprintf(buf, "%lu\n", pwm_test->period);
}

static ssize_t pwm_test_store_period(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct pwm_test *pwm_test = dev_get_drvdata(dev);

	rc = kstrtoul(buf, 0, &pwm_test->period_s);
	if (rc)
		return rc;

	return count;
}

static ssize_t pwm_test_show_config(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct pwm_test *pwm_test = dev_get_drvdata(dev);
	if (pwm_test->config)
		return sprintf(buf, "config Done\n");
	else
		return sprintf(buf, "config Failed\n");
}
static ssize_t pwm_test_store_config(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct pwm_test *pwm_test = dev_get_drvdata(dev);
	int ret;

  if (pwm_test->pwm == NULL) {
    pr_err("PWM Device has not been requested\n");
    return -ENODEV;
  }

	if (pwm_test->duty_s == 0) {
		ret = pwm_config(pwm_test->pwm, 0, pwm_test->period_s);
		if (ret) {
			pwm_test->config = 0;
			pr_err("operation failed %d\n", ret);
			pwm_test->duty_s = pwm_test->duty;
			pwm_test->period_s = pwm_test->period;
			return ret;
		}
		pwm_test->duty = pwm_test->duty_s;
		pwm_test->period = pwm_test->period_s;
		pwm_test->config = 1;
	} else {
		ret = pwm_config(pwm_test->pwm, pwm_test->duty_s,
				pwm_test->period_s);
		if (ret) {
			pwm_test->config = 0;
			pr_err("operation failed %d\n", ret);
			pwm_test->duty_s = pwm_test->duty;
			pwm_test->period_s = pwm_test->period;
			return ret;
		}
		pwm_test->duty = pwm_test->duty_s;
		pwm_test->period = pwm_test->period_s;
		pwm_test->config = 1;
	}
	return count;
}

static ssize_t pwm_test_show_run(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct pwm_test *pwm_test = dev_get_drvdata(dev);
	return sprintf(buf, "%s\n", pwm_test->run ? "Enabled" : "Disabled");
}

static ssize_t pwm_test_store_run(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct pwm_test *pwm_test = dev_get_drvdata(dev);

	rc = kstrtoul(buf, 0, &pwm_test->run_s);
	if (rc)
		return rc;

	if (pwm_test->run_s)
		rc = pwm_enable(pwm_test->pwm);
	else
		pwm_disable(pwm_test->pwm);

	if (rc) {
		pr_err("operation failed %d\n", rc);
		pwm_test->run_s = pwm_test->run;
		return rc;
	}

	pwm_test->run = pwm_test->run_s;
	return count;
}

static ssize_t pwm_test_show_polarity(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct pwm_test *pwm_test = dev_get_drvdata(dev);
	return sprintf(buf, "%s\n",
			pwm_test->polarity ? "Polarity Inversed" :
			"Polarity Normal");
}

static ssize_t pwm_test_store_polarity(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct pwm_test *pwm_test = dev_get_drvdata(dev);

	rc = kstrtoul(buf, 0, &pwm_test->polarity_s);
	if (rc)
		return rc;

	rc = pwm_set_polarity(pwm_test->pwm, pwm_test->polarity_s);
	if (rc) {
		pr_err("operation failed %d\n", rc);
		return rc;
	}

	pwm_test->polarity = pwm_test->polarity_s;
	return count;
}

static ssize_t pwm_test_show_request(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct pwm_test *pwm_test = dev_get_drvdata(dev);
	return sprintf(buf, "%s\n", pwm_test->requested ? "Requested" : "Freed");
}

static ssize_t pwm_test_store_request(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct pwm_test *pwm_test = dev_get_drvdata(dev);

	rc = kstrtoul(buf, 0, &pwm_test->requested_s);
	if (rc)
		return rc;

	if (pwm_test->requested_s) {

    if (pwm_test->pwm) {
      pr_info("PWM already requested!\n");
      return count;
    }

		pwm_test->pwm = pwm_get(dev, NULL);

		if (IS_ERR(pwm_test->pwm)) {
			dev_err(dev, "%s() %d %ld\n", __func__, __LINE__,
				PTR_ERR(pwm_test->pwm));
			rc = -EINVAL;
		}
	} else if (pwm_test->pwm != NULL) {

    pwm_test->polarity = 0;
		pwm_test->run = 0;
		pwm_test->duty = 0;
		pwm_test->period = 0;
		pwm_test->config = 0;
		pwm_test->polarity_s = 0;
		pwm_test->run_s = 0;
		pwm_test->duty_s = 0;
		pwm_test->period_s = 0;
		pwm_test->config_s = 0;

		pwm_config(pwm_test->pwm, pwm_test->duty_s,
      pwm_test->period_s);

    pwm_disable(pwm_test->pwm);

		rc = 0;
	}
  else {
    pr_info("PWM was already unrequested\n");
  }


	if (rc) {
		pr_err("operation failed %d\n", rc);
		pwm_test->requested_s = pwm_test->requested;
		return rc;
	}

	pwm_test->requested = pwm_test->requested_s;
	return count;
}

static DEVICE_ATTR(duty, S_IRUSR | S_IWUSR, pwm_test_show_duty, pwm_test_store_duty);
static DEVICE_ATTR(period, S_IRUSR | S_IWUSR, pwm_test_show_period, pwm_test_store_period);
static DEVICE_ATTR(polarity, S_IRUSR | S_IWUSR, pwm_test_show_polarity,
		pwm_test_store_polarity);
static DEVICE_ATTR(config, S_IRUSR | S_IWUSR , pwm_test_show_config, pwm_test_store_config);
static DEVICE_ATTR(run, S_IRUSR | S_IWUSR , pwm_test_show_run, pwm_test_store_run);
static DEVICE_ATTR(request, S_IRUSR | S_IWUSR , pwm_test_show_request, pwm_test_store_request);

static const struct attribute *pwm_attrs[] = {
	&dev_attr_duty.attr,
	&dev_attr_period.attr,
	&dev_attr_config.attr,
	&dev_attr_run.attr,
	&dev_attr_request.attr,
	&dev_attr_polarity.attr,
	NULL,
};

static const struct attribute_group pwm_device_attr_group = {
	.attrs = (struct attribute **) pwm_attrs,
};

static int __init pwm_test_class_init(struct device *dev)
{
	if (sysfs_create_group(&dev->kobj, &pwm_device_attr_group))
		return 1;
	return 0;
}

static int pwm_test_probe(struct platform_device *pdev)
{
	struct pwm_test *pwm_test;

	pwm_test = devm_kzalloc(&pdev->dev, sizeof(*pwm_test), GFP_KERNEL);

	if (!pwm_test) {
		dev_err(&pdev->dev, "memory error\n");
		return -ENOMEM;
	}

	if (pwm_test_class_init(&pdev->dev)) {
		dev_err(&pdev->dev, "sysfs creation failed\n");
		return -EINVAL;
	}
	dev_set_drvdata(&pdev->dev, pwm_test);
	platform_set_drvdata(pdev, pwm_test);
	return 0;
}

static int pwm_test_remove(struct platform_device *pdev)
{
	struct pwm_test *pwm_test = platform_get_drvdata(pdev);

	if (!pwm_test->ret) {
		pwm_config(pwm_test->pwm, 0, 0x1000);
		pwm_disable(pwm_test->pwm);
		pr_emerg("PWM Device Disabled %s\n", dev_name(&pdev->dev));
	}
	if (pwm_test->requested)
		pwm_free(pwm_test->pwm);

	pr_emerg("PWM Device Freed %s\n", dev_name(&pdev->dev));
	pwm_test->run = 0;
	sysfs_remove_group(&pdev->dev.kobj, &pwm_device_attr_group);
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
