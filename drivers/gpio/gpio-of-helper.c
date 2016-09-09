/*
 * GPIO OF based helper
 *
 * A simple DT based driver to provide access to GPIO functionality
 * to user-space via sysfs.
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
#include <linux/math64.h>
#include <linux/atomic.h>
#include <linux/idr.h>

/* fwd decl. */
struct gpio_of_helper_info;

enum gpio_type {
	GPIO_TYPE_INPUT = 0,
	GPIO_TYPE_OUTPUT = 1,
};

struct gpio_of_entry {
	int id;
	struct gpio_of_helper_info *info;
	struct device_node *node;
	enum gpio_type type;
	int gpio;
	enum of_gpio_flags gpio_flags;
	int irq;
	const char *name;
	atomic64_t counter;
	unsigned int count_flags;
#define COUNT_RISING_EDGE	(1 << 0)
#define COUNT_FALLING_EDGE	(1 << 1)
};

struct gpio_of_helper_info {
	struct platform_device *pdev;
	struct idr idr;
};

static const struct of_device_id gpio_of_helper_of_match[] = {
	{
		.compatible = "gpio-of-helper",
	},
	{ },
};
MODULE_DEVICE_TABLE(of, gpio_of_helper_of_match);

static ssize_t gpio_of_helper_show_status(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gpio_of_helper_info *info = platform_get_drvdata(pdev);
	struct gpio_of_entry *entry;
	char *p, *e;
	int id, n;

	p = buf;
	e = p + PAGE_SIZE;
	n = 0;
	idr_for_each_entry(&info->idr, entry, id) {
		switch (entry->type) {
		case GPIO_TYPE_INPUT:
			n = snprintf(p, e - p, "%2d %-24s %3d %-3s %llu\n",
				entry->id, entry->name, entry->gpio, "IN",
				(unsigned long long)
					atomic64_read(&entry->counter));
			break;
		case GPIO_TYPE_OUTPUT:
			n = snprintf(p, e - p, "%2d %-24s %3d %-3s\n",
				entry->id, entry->name, entry->gpio, "OUT");
			break;
		}
		p += n;
	}

	return p - buf;
}

static DEVICE_ATTR(status, S_IRUGO,
		gpio_of_helper_show_status, NULL);

static irqreturn_t gpio_of_helper_handler(int irq, void *ptr)
{
	struct gpio_of_entry *entry = ptr;

	/* caution - low speed interfaces only! */
	atomic64_inc(&entry->counter);

	return IRQ_HANDLED;
}

static struct gpio_of_entry *
gpio_of_entry_create(struct gpio_of_helper_info *info,
		struct device_node *node)
{
	struct platform_device *pdev = info->pdev;
	struct device *dev = &pdev->dev;
	struct gpio_of_entry *entry;
	int err, gpio, irq;
	unsigned int req_flags, count_flags, irq_flags;
	enum gpio_type type;
	enum of_gpio_flags gpio_flags;
	const char *name;

	/* get the type of the node first */
	if (of_property_read_bool(node, "input"))
		type = GPIO_TYPE_INPUT;
	else if (of_property_read_bool(node, "output")
			|| of_property_read_bool(node, "init-low")
			|| of_property_read_bool(node, "init-high"))
		type = GPIO_TYPE_OUTPUT;
	else {
		dev_err(dev, "Not valid gpio node type\n");
		err = -EINVAL;
		goto err_bad_node;
	}

	/* get the name */
	if (of_property_read_string(node, "line-name", &name))
		if (of_property_read_string(node, "gpio-name", &name))
			name = node->name;

	err = of_get_named_gpio_flags(node, "gpio", 0, &gpio_flags);
	if (IS_ERR_VALUE(err)) {
		dev_err(dev, "Failed to get gpio property of '%s'\n", name);
		goto err_bad_node;
	}
	gpio = err;

	req_flags = 0;
	count_flags = 0;

	/* set the request flags */
	switch (type) {
		case GPIO_TYPE_INPUT:
			req_flags = GPIOF_DIR_IN | GPIOF_EXPORT;
			if (of_property_read_bool(node, "count-falling-edge"))
				count_flags |= COUNT_FALLING_EDGE;
			if (of_property_read_bool(node, "count-rising-edge"))
				count_flags |= COUNT_RISING_EDGE;
			break;
		case GPIO_TYPE_OUTPUT:
			req_flags = GPIOF_DIR_OUT | GPIOF_EXPORT;
			if (of_property_read_bool(node, "init-high"))
				req_flags |= GPIOF_OUT_INIT_HIGH;
			else if (of_property_read_bool(node, "init-low"))
				req_flags |= GPIOF_OUT_INIT_LOW;
			break;
	}
	if (of_property_read_bool(node, "dir-changeable"))
		req_flags |= GPIOF_EXPORT_CHANGEABLE;

	/* request the gpio */
	err = devm_gpio_request_one(dev, gpio, req_flags, name);
	if (err != 0) {
		dev_err(dev, "Failed to request gpio '%s'\n", name);
		goto err_bad_node;
	}

	irq = -1;
	irq_flags = 0;

	/* counter mode requested - need an interrupt */
	if (count_flags != 0) {
		irq = gpio_to_irq(gpio);
		if (IS_ERR_VALUE(irq)) {
			dev_err(dev, "Failed to request gpio '%s'\n", name);
			goto err_bad_node;
		}

		if (count_flags & COUNT_RISING_EDGE)
			irq_flags |= IRQF_TRIGGER_RISING;
		if (count_flags & COUNT_FALLING_EDGE)
			irq_flags |= IRQF_TRIGGER_FALLING;
	}

//	if (!idr_pre_get(&info->idr, GFP_KERNEL)) {
//		dev_err(dev, "Failed on idr_pre_get of '%s'\n", name);
//		err = -ENOMEM;
//		goto err_no_mem;
//	}

	idr_preload(GFP_KERNEL);

	entry = devm_kzalloc(dev, sizeof(*entry), GFP_KERNEL);
	if (entry == NULL) {
		dev_err(dev, "Failed to allocate gpio entry of '%s'\n", name);
		err = -ENOMEM;
		goto err_no_mem;
	}

	entry->id = -1;
	entry->info = info;
	entry->node = of_node_get(node);	/* get node reference */
	entry->type = type;
	entry->gpio = gpio;
	entry->gpio_flags = gpio_flags;
	entry->irq = irq;
	entry->name = name;

	/* interrupt enable is last thing done */
	if (irq >= 0) {
		atomic64_set(&entry->counter, 0);
		entry->count_flags = count_flags;
		err = devm_request_irq(dev, irq, gpio_of_helper_handler,
				irq_flags, name, entry);
		if (err != 0) {
			dev_err(dev, "Failed to request irq of '%s'\n", name);
			goto err_no_irq;
		}
	}

	/* all done; insert */
//	err = idr_get_new(&info->idr, entry, &entry->id);
//	if (IS_ERR_VALUE(err)) {
//		dev_err(dev, "Failed to idr_get_new  of '%s'\n", name);
//		goto err_fail_idr;
//	}

	err = idr_alloc(&info->idr, entry, 0, 0, GFP_NOWAIT);
	if (err >= 0)
		entry->id = err;

	idr_preload_end();

	if (err < 0) {
		dev_err(dev, "Failed to idr_get_new  of '%s'\n", name);
		goto err_fail_idr;
	}

	dev_dbg(dev, "Allocated GPIO id=%d name='%s'\n", entry->id, name);

	return entry;

err_fail_idr:
	/* nothing to do */
err_no_irq:
	/* release node ref */
	of_node_put(node);
	/* nothing else needs to be done, devres handles it */
err_no_mem:
err_bad_node:
	return ERR_PTR(err);
}

static int gpio_of_entry_destroy(struct gpio_of_entry *entry)
{
	struct gpio_of_helper_info *info = entry->info;
	struct platform_device *pdev = info->pdev;
	struct device *dev = &pdev->dev;

	dev_dbg(dev, "Destroying GPIO id=%d\n", entry->id);

	/* remove from the IDR */
	idr_remove(&info->idr, entry->id);

	/* remove node ref */
	of_node_put(entry->node);

	/* free gpio */
	devm_gpio_free(dev, entry->gpio);

	/* gree irq */
	if (entry->irq >= 0)
		devm_free_irq(dev, entry->irq, entry);

	/* and free */
	devm_kfree(dev, entry);

	return 0;
}

static int gpio_of_helper_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gpio_of_helper_info *info;
	struct gpio_of_entry *entry;
	struct device_node *pnode = pdev->dev.of_node;
	struct device_node *cnode;
	struct pinctrl *pinctrl;
	int err;

	/* we only support OF */
	if (pnode == NULL) {
		dev_err(&pdev->dev, "No platform of_node!\n");
		return -ENODEV;
	}

	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(pinctrl)) {
		/* special handling for probe defer */
		if (PTR_ERR(pinctrl) == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		dev_warn(&pdev->dev,
			"pins are not configured from the driver\n");
	}

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (info == NULL) {
		dev_err(&pdev->dev, "Failed to allocate info\n");
		err = -ENOMEM;
		goto err_no_mem;
	}
	platform_set_drvdata(pdev, info);
	info->pdev = pdev;

	idr_init(&info->idr);

	err = device_create_file(dev, &dev_attr_status);
	if (err != 0) {
		dev_err(dev, "Failed to create status sysfs attribute\n");
		goto err_no_sysfs;
	}

	for_each_child_of_node(pnode, cnode) {

		entry = gpio_of_entry_create(info, cnode);
		if (IS_ERR_OR_NULL(entry)) {
			dev_err(dev, "Failed to create gpio entry\n");
			err = PTR_ERR(entry);
			goto err_fail_entry;
		}
	}

	dev_info(&pdev->dev, "ready\n");

	return 0;
err_fail_entry:
	device_remove_file(&pdev->dev, &dev_attr_status);
err_no_sysfs:
err_no_mem:
	return err;
}

static int gpio_of_helper_remove(struct platform_device *pdev)
{
	struct gpio_of_helper_info *info = platform_get_drvdata(pdev);
	struct gpio_of_entry *entry;
	int id;

	dev_info(&pdev->dev, "removing\n");

	device_remove_file(&pdev->dev, &dev_attr_status);

	id = 0;
	idr_for_each_entry(&info->idr, entry, id) {
		/* destroy each and every one */
		gpio_of_entry_destroy(entry);
	}

	return 0;
}

#ifdef CONFIG_PM
//#ifdef CONFIG_PM_RUNTIME
static int gpio_of_helper_runtime_suspend(struct device *dev)
{
	/* place holder */
	return 0;
}

static int gpio_of_helper_runtime_resume(struct device *dev)
{
	/* place holder */
	return 0;
}
//#endif /* CONFIG_PM_RUNTIME */

static struct dev_pm_ops gpio_of_helper_pm_ops = {
	SET_RUNTIME_PM_OPS(gpio_of_helper_runtime_suspend,
			   gpio_of_helper_runtime_resume, NULL)
};
#define GPIO_OF_HELPER_PM_OPS (&gpio_of_helper_pm_ops)
#else
#define GPIO_OF_HELPER_PM_OPS NULL
#endif /* CONFIG_PM */

struct platform_driver gpio_of_helper_driver = {
	.probe		= gpio_of_helper_probe,
	.remove		= gpio_of_helper_remove,
	.driver = {
		.name		= "gpio-of-helper",
		.owner		= THIS_MODULE,
		.pm		= GPIO_OF_HELPER_PM_OPS,
		.of_match_table	= gpio_of_helper_of_match,
	},
};

module_platform_driver(gpio_of_helper_driver);

MODULE_AUTHOR("Pantelis Antoniou <panto@antoniou-consulting.com>");
MODULE_DESCRIPTION("GPIO OF Helper driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:gpio-of-helper");
