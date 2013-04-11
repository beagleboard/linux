/*
 * Test driver for rstctl
 *
 * Author: Pantelis Antoniou <panto@antoniou-consulting.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rstctl.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/pinctrl/consumer.h>

struct gpio_data {
	int gpio;
	enum of_gpio_flags flags;
	int hold_ns;
};

struct gpio_rctrl_info {
	struct rstctl_dev *rdev;
	struct rstctl_desc desc;
	struct gpio_data *gpio_data;
};

int gpio_rctrl_request(struct rstctl_dev *rdev,
		const struct rstctl_line *line)
{
	struct device *dev = rdev->dev;
	const struct gpio_data *gd = line->data;
	unsigned long flags;
	int err;

	dev_info(dev, "%s %s\n", __func__, line->name);

	flags = GPIOF_DIR_OUT;
	if (gd->flags & OF_GPIO_ACTIVE_LOW)
		flags |= GPIOF_INIT_LOW;
	else
		flags |= GPIOF_INIT_HIGH;
	/* XXX more flags ? */

	err = devm_gpio_request_one(dev, gd->gpio, flags, line->name);
	if (err != 0) {
		dev_err(dev, "Failed to gpio_request\n");
		return err;
	}
	return 0;
}

int gpio_rctrl_release(struct rstctl_dev *rdev,
		const struct rstctl_line *line)
{
	struct device *dev = rdev->dev;
	const struct gpio_data *gd = line->data;

	dev_info(dev, "%s %s\n", __func__, line->name);
	devm_gpio_free(dev, gd->gpio);
	return 0;
}

int gpio_rctrl_assert(struct rstctl_dev *rdev,
		const struct rstctl_line *line)
{
	struct device *dev = rdev->dev;
	const struct gpio_data *gd = line->data;

	dev_info(dev, "%s %s\n", __func__, line->name);
	gpio_set_value(gd->gpio,
			(gd->flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1);
	return 0;
}

int gpio_rctrl_deassert(struct rstctl_dev *rdev,
		const struct rstctl_line *line)
{
	struct device *dev = rdev->dev;
	const struct gpio_data *gd = line->data;

	dev_info(dev, "%s %s\n", __func__, line->name);
	gpio_set_value(gd->gpio,
			(gd->flags & OF_GPIO_ACTIVE_LOW) ? 1 : 0);
	return 0;
}

int gpio_rctrl_pulse(struct rstctl_dev *rdev,
		const struct rstctl_line *line,
		unsigned long hold_ns)
{
	struct device *dev = rdev->dev;
	const struct gpio_data *gd = line->data;

	dev_info(dev, "%s %s\n", __func__, line->name);

	gpio_set_value(gd->gpio,
			(gd->flags & OF_GPIO_ACTIVE_LOW) ? 0 : 1);

	if (hold_ns == 0)
		hold_ns = gd->hold_ns;

	if (hold_ns < 1000000)
		ndelay(hold_ns);
	else
		mdelay(hold_ns / 1000000);

	gpio_set_value(gd->gpio,
			(gd->flags & OF_GPIO_ACTIVE_LOW) ? 1 : 0);
	return 0;
}

static const struct rstctl_ops gpio_rctrl_ops = {
	.request	= gpio_rctrl_request,
	.release	= gpio_rctrl_release,
	.assert		= gpio_rctrl_assert,
	.deassert	= gpio_rctrl_deassert,
	.pulse		= gpio_rctrl_pulse,
};

static int gpio_rctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct gpio_rctrl_info *info;
	struct rstctl_line *line, *lines;
	struct gpio_data *gdata, *gd;
	struct pinctrl *pinctrl;
	int count, i, err;

	/* we require OF */
	if (!IS_ENABLED(CONFIG_OF) || np == NULL) {
		dev_err(dev, "GPIO rstctl requires DT\n");
		return -ENODEV;
	}

	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl))
		dev_warn(dev, "Unable to select pin group\n");

	count = of_gpio_named_count(np, "gpios");
	if (count == 0) {
		dev_err(dev, "GPIO rstctl found no GPIO resources\n");
		return -ENODEV;
	}

	if (of_property_count_strings(np, "gpio-names") != count) {
		dev_err(dev, "GPIO rstctl gpio-names property is invalid\n");
		return -ENODEV;
	}

	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info) {
		dev_err(dev, "GPIO rstctl kzalloc failed\n");
		return -ENOMEM;
	}
	lines = devm_kzalloc(dev, sizeof(*line) * count, GFP_KERNEL);
	if (!lines) {
		dev_err(dev, "GPIO rstctl kzalloc failed\n");
		return -ENOMEM;
	}
	gdata = devm_kzalloc(dev, sizeof(*gdata) * count, GFP_KERNEL);
	if (!gdata) {
		dev_err(dev, "GPIO rstctl kzalloc failed\n");
		return -ENOMEM;
	}

	info->desc.name = "gpio";
	info->desc.ops = &gpio_rctrl_ops;
	info->desc.nlines = count;
	info->desc.lines = lines;
	info->gpio_data = gdata;

	for (i = 0; i < count; i++) {
		line = &lines[i];
		err = of_property_read_string_index(np, "gpio-names",
				i, &line->name);
		if (err != 0) {
			dev_err(dev, "Failed to get string property\n");
			return err;
		}

		gd = &gdata[i];
		err = of_get_named_gpio_flags(np, "gpios", i, &gd->flags);
		if (IS_ERR_VALUE(err)) {
			dev_err(dev, "Failed to get named gpio\n");
			return err;
		}
		gd->gpio = err;
		gd->hold_ns = 1000;	/* 1us reset */
		line->data = gd;
	}

	info->rdev = rstctl_register(&pdev->dev, &info->desc);
	if (IS_ERR(info->rdev)) {
		dev_err(&pdev->dev, "failed to register\n");
		return PTR_ERR(info->rdev);
	}
	platform_set_drvdata(pdev, info);

	dev_info(&pdev->dev, "loaded OK\n");

	return 0;
}

static int gpio_rctrl_remove(struct platform_device *pdev)
{
	struct gpio_rctrl_info *info = platform_get_drvdata(pdev);
	int err;

	err = rstctl_unregister(info->rdev);
	if (err == 0)
		dev_info(&pdev->dev, "removed OK\n");

	return err;
}

#ifdef CONFIG_OF
static struct of_device_id gpio_rctrl_of_match[] = {
	{ .compatible = "gpio-rctrl" },
	{ },
};
MODULE_DEVICE_TABLE(of, gpio_rctrl_of_match);
#endif

static struct platform_driver gpio_rctrl_driver = {
	.driver = {
		.name = "gpio-rctrl",
		.of_match_table = of_match_ptr(gpio_rctrl_of_match),
	},
	.probe = gpio_rctrl_probe,
	.remove = gpio_rctrl_remove,
};

/*
 * The reset control driver must be done very early.
 */
static int __init gpio_rctrl_drv_reg(void)
{
	return platform_driver_register(&gpio_rctrl_driver);
}
postcore_initcall(gpio_rctrl_drv_reg);

MODULE_AUTHOR("Pantelis Antoniou <panto@antoniou-consulting.com>");
MODULE_DESCRIPTION("rstctl GPIO driver");
MODULE_LICENSE("GPL");
