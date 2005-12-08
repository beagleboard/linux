/*
 *  linux/arch/arm/plat-omap/gpio-switch.c
 *
 *  Copyright (C) 2004, 2005 Nokia Corporation
 *  Written by Juha Yrjölä <juha.yrjola@nokia.com>
 *         and Paul Mundt <paul.mundt@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/sched.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/timer.h>
#include <linux/err.h>
#include <asm/arch/hardware.h>
#include <asm/arch/gpio.h>
#include <asm/arch/irqs.h>
#include <asm/arch/mux.h>
#include <asm/arch/board.h>

struct gpio_switch {
	char		name[14];
	u16		gpio;
	int		flags;
	int		type;
	int		state;

	struct work_struct	work;
	struct timer_list	timer;
	struct platform_device	pdev;

	struct list_head	node;
};

static LIST_HEAD(gpio_switches);
static struct platform_device *gpio_sw_platform_dev;
static struct device_driver gpio_sw_driver;

static const char *cover_str[2] = { "open", "closed" };
static const char *connection_str[2] = { "disconnected", "connected" };

/*
 * GPIO switch state poll delay in ms
 */
#define OMAP_GPIO_SW_POLL_DELAY	10

static void print_sw_state(struct gpio_switch *sw, int state)
{
	const char **str;

	switch (sw->type) {
	case OMAP_GPIO_SWITCH_TYPE_COVER:
		str = cover_str;
		break;
	case OMAP_GPIO_SWITCH_TYPE_CONNECTION:
		str = connection_str;
		break;
	default:
		str = NULL;
	}
	if (str != NULL)
		printk(KERN_INFO "%s (GPIO %d) is now %s\n", sw->name, sw->gpio, str[state]);
}

static int gpio_sw_get_state(struct gpio_switch *sw)
{
	int state;

	state = omap_get_gpio_datain(sw->gpio);
	if (sw->flags & OMAP_GPIO_SWITCH_FLAG_INVERTED)
		state = !state;

	return state;
}

static ssize_t gpio_sw_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf,
			     size_t count)
{
	struct gpio_switch *sw = dev_get_drvdata(dev);
	int enable = (int)simple_strtoul(buf, NULL, 10);
	omap_set_gpio_dataout(sw->gpio, enable);
	return count;
}

#define gpio_sw_switch_attr(name)					\
static ssize_t gpio_sw_show_##name(struct device *dev,			\
					struct device_attribute *attr,	\
					char *buf)			\
{									\
	struct gpio_switch *sw = dev_get_drvdata(dev);			\
	return sprintf(buf, "%s\n", name##_str[gpio_sw_get_state(sw)]);	\
}									\
static DEVICE_ATTR(name##_switch, S_IRUGO | S_IWUSR,			\
		   gpio_sw_show_##name, gpio_sw_store)

gpio_sw_switch_attr(cover);
gpio_sw_switch_attr(connection);

static irqreturn_t gpio_sw_irq_handler(int irq, void *arg, struct pt_regs *regs)
{
	struct gpio_switch *sw = arg;

	mod_timer(&sw->timer, jiffies + OMAP_GPIO_SW_POLL_DELAY / (1000 / HZ));

	return IRQ_HANDLED;
}

static void gpio_sw_timer(unsigned long arg)
{
	struct gpio_switch *sw = (struct gpio_switch *)arg;

	schedule_work(&sw->work);
}

static void gpio_sw_handler(void *data)
{
	struct gpio_switch *sw = data;
	int state = gpio_sw_get_state(sw);

	if (sw->state == state)
		return;

	if (sw->type == OMAP_GPIO_SWITCH_TYPE_CONNECTION)
		kobject_uevent(&sw->pdev.dev.kobj, KOBJ_CHANGE,
			       &dev_attr_connection_switch.attr);
	else
		kobject_uevent(&sw->pdev.dev.kobj, KOBJ_CHANGE,
			       &dev_attr_cover_switch.attr);
	sw->state = state;
	if (omap_get_gpio_datain(sw->gpio))
		set_irq_type(OMAP_GPIO_IRQ(sw->gpio), IRQT_FALLING);
	else
		set_irq_type(OMAP_GPIO_IRQ(sw->gpio), IRQT_RISING);
	print_sw_state(sw, state);
}

static int __init new_switch(struct gpio_switch *sw)
{
	int r, direction;

	sw->pdev.name	= sw->name;
	sw->pdev.id	= -1;

	sw->pdev.dev.parent = &gpio_sw_platform_dev->dev;
	sw->pdev.dev.driver = &gpio_sw_driver;

	r = platform_device_register(&sw->pdev);
	if (r)
		return r;

	dev_set_drvdata(&sw->pdev.dev, sw);

	r = omap_request_gpio(sw->gpio);
	if (r < 0) {
		platform_device_unregister(&sw->pdev);
		return r;
	}

	/* input: 1, output: 0 */
	direction = !(sw->flags & OMAP_GPIO_SWITCH_FLAG_OUTPUT);
	omap_set_gpio_direction(sw->gpio, direction);

	if (omap_get_gpio_datain(sw->gpio))
		set_irq_type(OMAP_GPIO_IRQ(sw->gpio), IRQT_FALLING);
	else
		set_irq_type(OMAP_GPIO_IRQ(sw->gpio), IRQT_RISING);

	switch (sw->type) {
	case OMAP_GPIO_SWITCH_TYPE_COVER:
		device_create_file(&sw->pdev.dev, &dev_attr_cover_switch);
		break;
	case OMAP_GPIO_SWITCH_TYPE_CONNECTION:
		device_create_file(&sw->pdev.dev, &dev_attr_connection_switch);
		break;
	}

	list_add(&sw->node, &gpio_switches);

	if (!direction)
		return 0;

	r = request_irq(OMAP_GPIO_IRQ(sw->gpio), gpio_sw_irq_handler, SA_SHIRQ,
			sw->name, sw);
	if (r < 0) {
		printk(KERN_ERR "gpio-switch: request_irq() failed "
				"for GPIO %d\n", sw->gpio);
		platform_device_unregister(&sw->pdev);
		omap_free_gpio(sw->gpio);
		return r;
	}

	INIT_WORK(&sw->work, gpio_sw_handler, sw);
	init_timer(&sw->timer);

	sw->timer.function = gpio_sw_timer;
	sw->timer.data = (unsigned long)sw;

	return 0;
}

static int __init add_atag_switches(void)
{
	const struct omap_gpio_switch_config *cfg;
	struct gpio_switch *sw;
	int i, r;

	for (i = 0; ; i++) {
		cfg = omap_get_nr_config(OMAP_TAG_GPIO_SWITCH,
					 struct omap_gpio_switch_config, i);
		if (cfg == NULL)
			break;
		sw = kmalloc(sizeof(*sw), GFP_KERNEL);
		if (sw == NULL) {
			printk(KERN_ERR "gpio-switch: kmalloc failed\n");
			return -ENOMEM;
		}
		memset(sw, 0, sizeof(*sw));
		strncpy(sw->name, cfg->name, sizeof(cfg->name));
		sw->gpio = cfg->gpio;
		sw->flags = cfg->flags;
		sw->type = cfg->type;
		sw->state = gpio_sw_get_state(sw);
		if ((r = new_switch(sw)) < 0) {
			kfree(sw);
			return r;
		}
	}
	return 0;
}

static void gpio_sw_cleanup(void)
{
	struct gpio_switch *sw = NULL, *old = NULL;

	list_for_each_entry(sw, &gpio_switches, node) {
		kfree(old);

		flush_scheduled_work();
		del_timer_sync(&sw->timer);

		free_irq(OMAP_GPIO_IRQ(sw->gpio), sw);

		if (sw->type == OMAP_GPIO_SWITCH_TYPE_CONNECTION)
			device_remove_file(&sw->pdev.dev,
					   &dev_attr_connection_switch);
		else
			device_remove_file(&sw->pdev.dev,
					   &dev_attr_cover_switch);

		platform_device_unregister(&sw->pdev);
		omap_free_gpio(sw->gpio);
		old = sw;
	}

	kfree(sw);
}

static void __init report_initial_state(void)
{
	struct gpio_switch *sw;

	list_for_each_entry(sw, &gpio_switches, node) {
		int state;

		state = omap_get_gpio_datain(sw->gpio);
		if (sw->flags & OMAP_GPIO_SWITCH_FLAG_INVERTED)
			state = !state;
		print_sw_state(sw, state);
	}
}

static void gpio_sw_shutdown(struct device *dev)
{
}

static struct device_driver gpio_sw_driver = {
	.name		= "gpio-switch",
	.bus		= &platform_bus_type,
	.shutdown	= gpio_sw_shutdown,
};

static int __init gpio_sw_init(void)
{
	int r;

	printk(KERN_INFO "OMAP GPIO switch handler initializing\n");

	r = driver_register(&gpio_sw_driver);
	if (r)
		return r;

	gpio_sw_platform_dev = platform_device_register_simple("gpio-switch",
							       -1, NULL, 0);
	if (IS_ERR(gpio_sw_platform_dev)) {
		driver_unregister(&gpio_sw_driver);
		return PTR_ERR(gpio_sw_platform_dev);
	}

	r = add_atag_switches();
	if (r < 0) {
		platform_device_unregister(gpio_sw_platform_dev);
		driver_unregister(&gpio_sw_driver);
		gpio_sw_cleanup();
		return r;
	}

	report_initial_state();

	return 0;
}

static void __exit gpio_sw_exit(void)
{
	gpio_sw_cleanup();
	platform_device_unregister(gpio_sw_platform_dev);
	driver_unregister(&gpio_sw_driver);
}

#ifndef MODULE
late_initcall(gpio_sw_init);
#else
module_init(gpio_sw_init);
#endif
module_exit(gpio_sw_exit);

MODULE_AUTHOR("Juha Yrjölä <juha.yrjola@nokia.com>, Paul Mundt <paul.mundt@nokia.com");
MODULE_DESCRIPTION("GPIO switch driver");
MODULE_LICENSE("GPL");
