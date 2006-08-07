/*
 *  linux/arch/arm/plat-omap/gpio-switch.c
 *
 *  Copyright (C) 2004-2006 Nokia Corporation
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
#include <linux/irq.h>
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
#include <asm/arch/gpio-switch.h>

struct gpio_switch {
	char		name[14];
	u16		gpio;
	unsigned	flags:4;
	unsigned	type:4;
	unsigned	state:1;
	unsigned	both_edges:1;

	u16		debounce_rising;
	u16		debounce_falling;

	void (* notify)(void *data, int state);
	void *notify_data;

	struct work_struct	work;
	struct timer_list	timer;
	struct platform_device	pdev;

	struct list_head	node;
};

static LIST_HEAD(gpio_switches);
static struct platform_device *gpio_sw_platform_dev;
static struct device_driver gpio_sw_driver;

static const struct omap_gpio_switch *board_gpio_sw_table;
static int board_gpio_sw_count;

static const char *cover_str[2] = { "open", "closed" };
static const char *connection_str[2] = { "disconnected", "connected" };
static const char *activity_str[2] = { "inactive", "active" };

/*
 * GPIO switch state default debounce delay in ms
 */
#define OMAP_GPIO_SW_DEFAULT_DEBOUNCE		10

static const char **get_sw_str(struct gpio_switch *sw)
{
	switch (sw->type) {
	case OMAP_GPIO_SWITCH_TYPE_COVER:
		return cover_str;
	case OMAP_GPIO_SWITCH_TYPE_CONNECTION:
		return connection_str;
	case OMAP_GPIO_SWITCH_TYPE_ACTIVITY:
		return activity_str;
	default:
		BUG();
		return NULL;
	}
}

static const char *get_sw_type(struct gpio_switch *sw)
{
	switch (sw->type) {
	case OMAP_GPIO_SWITCH_TYPE_COVER:
		return "cover";
	case OMAP_GPIO_SWITCH_TYPE_CONNECTION:
		return "connection";
	case OMAP_GPIO_SWITCH_TYPE_ACTIVITY:
		return "activity";
	default:
		BUG();
		return NULL;
	}
}

static void print_sw_state(struct gpio_switch *sw, int state)
{
	const char **str;

	str = get_sw_str(sw);
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

static ssize_t gpio_sw_state_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf,
				   size_t count)
{
	struct gpio_switch *sw = dev_get_drvdata(dev);
	const char **str;
	char state[16];
	int enable;

	if (!(sw->flags & OMAP_GPIO_SWITCH_FLAG_OUTPUT))
		return -EPERM;

	if (sscanf(buf, "%15s", state) != 1)
		return -EINVAL;

	str = get_sw_str(sw);
	if (strcmp(state, str[0]) == 0)
		enable = 0;
	else if (strcmp(state, str[1]) == 0)
		enable = 1;
	else
		return -EINVAL;
	if (sw->flags & OMAP_GPIO_SWITCH_FLAG_INVERTED)
		enable = !enable;
	omap_set_gpio_dataout(sw->gpio, enable);

	return count;
}

static ssize_t gpio_sw_state_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	struct gpio_switch *sw = dev_get_drvdata(dev);
	const char **str;

	str = get_sw_str(sw);
	return sprintf(buf, "%s\n", str[sw->state]);
}

static DEVICE_ATTR(state, S_IRUGO | S_IWUSR, gpio_sw_state_show,
		   gpio_sw_state_store);

static ssize_t gpio_sw_type_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct gpio_switch *sw = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", get_sw_type(sw));
}

static DEVICE_ATTR(type, S_IRUGO, gpio_sw_type_show, NULL);

static ssize_t gpio_sw_direction_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct gpio_switch *sw = dev_get_drvdata(dev);
	int is_output;

	is_output = sw->flags & OMAP_GPIO_SWITCH_FLAG_OUTPUT;
	return sprintf(buf, "%s\n", is_output ? "output" : "input");
}

static DEVICE_ATTR(direction, S_IRUGO, gpio_sw_direction_show, NULL);


static irqreturn_t gpio_sw_irq_handler(int irq, void *arg, struct pt_regs *regs)
{
	struct gpio_switch *sw = arg;
	unsigned long timeout;
	int state;

	if (!sw->both_edges) {
		if (omap_get_gpio_datain(sw->gpio))
			set_irq_type(OMAP_GPIO_IRQ(sw->gpio), IRQT_FALLING);
		else
			set_irq_type(OMAP_GPIO_IRQ(sw->gpio), IRQT_RISING);
	}

	state = gpio_sw_get_state(sw);
	if (sw->state == state)
		return IRQ_HANDLED;

	if (state)
		timeout = sw->debounce_rising;
	else
		timeout = sw->debounce_falling;
	if (!timeout)
		schedule_work(&sw->work);
	else
		mod_timer(&sw->timer, jiffies + msecs_to_jiffies(timeout));

	return IRQ_HANDLED;
}

static void gpio_sw_timer(unsigned long arg)
{
	struct gpio_switch *sw = (struct gpio_switch *) arg;

	schedule_work(&sw->work);
}

static void gpio_sw_handler(void *data)
{
	struct gpio_switch *sw = data;
	int state;

	state = gpio_sw_get_state(sw);
	if (sw->state == state)
		return;

	sw->state = state;
	if (sw->notify != NULL)
		sw->notify(sw->notify_data, state);
	sysfs_notify(&sw->pdev.dev.kobj, NULL, "state");
	print_sw_state(sw, state);
}

static int __init can_do_both_edges(struct gpio_switch *sw)
{
	if (!cpu_class_is_omap1())
		return 1;
	if (OMAP_GPIO_IS_MPUIO(sw->gpio))
		return 0;
	else
		return 1;
}

static void gpio_sw_release(struct device *dev)
{
}

static int __init new_switch(struct gpio_switch *sw)
{
	int r, direction, trigger;

	switch (sw->type) {
	case OMAP_GPIO_SWITCH_TYPE_COVER:
	case OMAP_GPIO_SWITCH_TYPE_CONNECTION:
	case OMAP_GPIO_SWITCH_TYPE_ACTIVITY:
		break;
	default:
		printk(KERN_ERR "invalid GPIO switch type: %d\n", sw->type);
		return -EINVAL;
	}

	sw->pdev.name	= sw->name;
	sw->pdev.id	= -1;

	sw->pdev.dev.parent = &gpio_sw_platform_dev->dev;
	sw->pdev.dev.driver = &gpio_sw_driver;
	sw->pdev.dev.release = gpio_sw_release;

	r = platform_device_register(&sw->pdev);
	if (r) {
		printk(KERN_ERR "gpio-switch: platform device registration "
		       "failed for %s", sw->name);
		return r;
	}
	dev_set_drvdata(&sw->pdev.dev, sw);

	r = omap_request_gpio(sw->gpio);
	if (r < 0) {
		platform_device_unregister(&sw->pdev);
		return r;
	}

	/* input: 1, output: 0 */
	direction = !(sw->flags & OMAP_GPIO_SWITCH_FLAG_OUTPUT);
	omap_set_gpio_direction(sw->gpio, direction);

	sw->state = gpio_sw_get_state(sw);

	device_create_file(&sw->pdev.dev, &dev_attr_state);
	device_create_file(&sw->pdev.dev, &dev_attr_type);
	device_create_file(&sw->pdev.dev, &dev_attr_direction);

	if (!direction)
		return 0;

	if (can_do_both_edges(sw)) {
		trigger = SA_TRIGGER_FALLING | SA_TRIGGER_RISING;
		sw->both_edges = 1;
	} else {
		if (omap_get_gpio_datain(sw->gpio))
			trigger = SA_TRIGGER_FALLING;
		else
			trigger = SA_TRIGGER_RISING;
	}
	r = request_irq(OMAP_GPIO_IRQ(sw->gpio), gpio_sw_irq_handler,
			SA_SHIRQ | trigger, sw->name, sw);
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

	list_add(&sw->node, &gpio_switches);

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
		sw = kzalloc(sizeof(*sw), GFP_KERNEL);
		if (sw == NULL) {
			printk(KERN_ERR "gpio-switch: kmalloc failed\n");
			return -ENOMEM;
		}
		strncpy(sw->name, cfg->name, sizeof(cfg->name));
		sw->gpio = cfg->gpio;
		sw->flags = cfg->flags;
		sw->type = cfg->type;
		sw->debounce_rising = OMAP_GPIO_SW_DEFAULT_DEBOUNCE;
		sw->debounce_falling = OMAP_GPIO_SW_DEFAULT_DEBOUNCE;
		if ((r = new_switch(sw)) < 0) {
			kfree(sw);
			return r;
		}
	}
	return 0;
}

static struct gpio_switch * __init find_switch(int gpio, const char *name)
{
	struct gpio_switch *sw;

	list_for_each_entry(sw, &gpio_switches, node) {
		if ((gpio < 0 || sw->gpio != gpio) &&
		    (name == NULL || strcmp(sw->name, name) != 0))
			continue;

		if (gpio < 0 || name == NULL)
			goto no_check;

		if (strcmp(sw->name, name) != 0)
			printk("gpio-switch: name mismatch for %d (%s, %s)\n",
			       gpio, name, sw->name);
		else if (sw->gpio != gpio)
			printk("gpio-switch: GPIO mismatch for %s (%d, %d)\n",
			       name, gpio, sw->gpio);
no_check:
		return sw;
	}
	return NULL;
}

static int __init add_board_switches(void)
{
	int i;

	for (i = 0; i < board_gpio_sw_count; i++) {
		const struct omap_gpio_switch *cfg;
		struct gpio_switch *sw;
		int r;

		cfg = board_gpio_sw_table + i;
		if (strlen(cfg->name) > sizeof(sw->name) - 1)
			return -EINVAL;
		/* Check whether we only update an existing switch
		 * or add a new switch. */
		sw = find_switch(cfg->gpio, cfg->name);
		if (sw != NULL) {
			sw->debounce_rising = cfg->debounce_rising;
			sw->debounce_falling = cfg->debounce_falling;
			sw->notify = cfg->notify;
			sw->notify_data = cfg->notify_data;
			continue;
		} else {
			if (cfg->gpio < 0 || cfg->name == NULL) {
				printk("gpio-switch: required switch not "
				       "found (%d, %s)\n", cfg->gpio,
				       cfg->name);
				continue;
			}
		}
		sw = kzalloc(sizeof(*sw), GFP_KERNEL);
		if (sw == NULL) {
			printk(KERN_ERR "gpio-switch: kmalloc failed\n");
			return -ENOMEM;
		}
		strlcpy(sw->name, cfg->name, sizeof(sw->name));
		sw->gpio = cfg->gpio;
		sw->flags = cfg->flags;
		sw->type = cfg->type;
		sw->debounce_rising = cfg->debounce_rising;
		sw->debounce_falling = cfg->debounce_falling;
		sw->notify = cfg->notify;
		sw->notify_data = cfg->notify_data;
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
		if (old != NULL)
			kfree(old);
		flush_scheduled_work();
		del_timer_sync(&sw->timer);

		free_irq(OMAP_GPIO_IRQ(sw->gpio), sw);

		device_remove_file(&sw->pdev.dev, &dev_attr_state);
		device_remove_file(&sw->pdev.dev, &dev_attr_type);
		device_remove_file(&sw->pdev.dev, &dev_attr_direction);

		platform_device_unregister(&sw->pdev);
		omap_free_gpio(sw->gpio);
		old = sw;
	}
	kfree(old);
}

static void __init report_initial_state(void)
{
	struct gpio_switch *sw;

	list_for_each_entry(sw, &gpio_switches, node) {
		int state;

		state = omap_get_gpio_datain(sw->gpio);
		if (sw->flags & OMAP_GPIO_SWITCH_FLAG_INVERTED)
			state = !state;
		if (sw->notify != NULL)
			sw->notify(sw->notify_data, state);
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

void __init omap_register_gpio_switches(const struct omap_gpio_switch *tbl,
					int count)
{
	BUG_ON(board_gpio_sw_table != NULL);

	board_gpio_sw_table = tbl;
	board_gpio_sw_count = count;
}

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
		r = PTR_ERR(gpio_sw_platform_dev);
		goto err1;
	}

	r = add_atag_switches();
	if (r < 0)
		goto err2;

	r = add_board_switches();
	if (r < 0)
		goto err2;

	report_initial_state();

	return 0;
err2:
	gpio_sw_cleanup();
	platform_device_unregister(gpio_sw_platform_dev);
err1:
	driver_unregister(&gpio_sw_driver);
	return r;
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
