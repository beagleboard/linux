/*
 * Pinmux helper driver
 *
 * Copyright (C) 2013 Pantelis Antoniou <panto@antoniou-consulting.com>
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
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/consumer.h>

static const struct of_device_id bone_pinmux_helper_of_match[] = {
	{
		.compatible = "bone-pinmux-helper",
	},
	{ },
};
MODULE_DEVICE_TABLE(of, bone_pinmux_helper_of_match);

struct pinmux_helper_data {
	struct pinctrl *pinctrl;
	char *selected_state_name;
};

static ssize_t pinmux_helper_show_state(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pinmux_helper_data *data = platform_get_drvdata(pdev);
	const char *name;

	name = data->selected_state_name;
	if (name == NULL || strlen(name) == 0)
		name = "none";
	return sprintf(buf, "%s\n", name);
}

static ssize_t pinmux_helper_store_state(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pinmux_helper_data *data = platform_get_drvdata(pdev);
	struct pinctrl_state *state;
	char *state_name;
	char *s;
	int err;

	/* duplicate (as a null terminated string) */
	state_name = kmalloc(count + 1, GFP_KERNEL);
	if (state_name == NULL)
		return -ENOMEM;
	memcpy(state_name, buf, count);
	state_name[count] = '\0';

	/* and chop off newline */
	s = strchr(state_name, '\n');
	if (s != NULL)
		*s = '\0';

	/* try to select default state at first (if it exists) */
	state = pinctrl_lookup_state(data->pinctrl, state_name);
	if (!IS_ERR(state)) {
		err = pinctrl_select_state(data->pinctrl, state);
		if (err != 0)
			dev_err(dev, "Failed to select state %s\n",
					state_name);
	} else {
		dev_err(dev, "Failed to find state %s\n", state_name);
		err = PTR_RET(state);
	}

	if (err == 0) {
		kfree(data->selected_state_name);
		data->selected_state_name = state_name;
	}

	return err ? err : count;
}

static DEVICE_ATTR(state, S_IWUSR | S_IRUGO,
		   pinmux_helper_show_state, pinmux_helper_store_state);

static struct attribute *pinmux_helper_attributes[] = {
	&dev_attr_state.attr,
	NULL
};

static const struct attribute_group pinmux_helper_attr_group = {
	.attrs = pinmux_helper_attributes,
};

static int bone_pinmux_helper_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pinmux_helper_data *data;
	struct pinctrl_state *state;
	char *state_name;
	int err;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		dev_err(dev, "Failed to allocate data\n");
		err = -ENOMEM;
		goto err_no_mem;
	}
	state_name = kmalloc(strlen(PINCTRL_STATE_DEFAULT) + 1,
			GFP_KERNEL);
	if (state_name == NULL) {
		dev_err(dev, "Failed to allocate state name\n");
		err = -ENOMEM;
		goto err_no_mem;
	}
	data->selected_state_name = state_name;
	strcpy(data->selected_state_name, PINCTRL_STATE_DEFAULT);

	platform_set_drvdata(pdev, data);

	data->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(data->pinctrl)) {
		dev_err(dev, "Failed to get pinctrl\n");
		err = PTR_RET(data->pinctrl);
		goto err_no_pinctrl;
	}

	/* try to select default state at first (if it exists) */
	state = pinctrl_lookup_state(data->pinctrl,
			data->selected_state_name);
	if (!IS_ERR(state)) {
		err = pinctrl_select_state(data->pinctrl, state);
		if (err != 0) {
			dev_err(dev, "Failed to select default state\n");
			goto err_no_state;
		}
	} else {
		data->selected_state_name = '\0';
	}

	/* Register sysfs hooks */
	err = sysfs_create_group(&dev->kobj, &pinmux_helper_attr_group);
	if (err) {
		dev_err(dev, "Failed to create sysfs group\n");
		goto err_no_sysfs;
	}

	return 0;

err_no_sysfs:
err_no_state:
	devm_pinctrl_put(data->pinctrl);
err_no_pinctrl:
	devm_kfree(dev, data);
err_no_mem:
	return err;
}

static int bone_pinmux_helper_remove(struct platform_device *pdev)
{
	struct pinmux_helper_data *data = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	sysfs_remove_group(&dev->kobj, &pinmux_helper_attr_group);
	kfree(data->selected_state_name);
	devm_pinctrl_put(data->pinctrl);
	devm_kfree(dev, data);

	return 0;
}

struct platform_driver bone_pinmux_helper_driver = {
	.probe		= bone_pinmux_helper_probe,
	.remove		= bone_pinmux_helper_remove,
	.driver = {
		.name		= "bone-pinmux-helper",
		.owner		= THIS_MODULE,
		.of_match_table	= bone_pinmux_helper_of_match,
	},
};

module_platform_driver(bone_pinmux_helper_driver);

MODULE_AUTHOR("Pantelis Antoniou");
MODULE_DESCRIPTION("Beaglebone pinmux helper driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:bone-pinmux-helper");
