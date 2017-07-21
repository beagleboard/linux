/**
 * @note Copyright (C) 2016 Philippe Gerum <rpm@xenomai.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/err.h>
#include "gpio-core.h"

struct rtdm_gpio_pin {
	struct rtdm_device dev;
	struct list_head next;
	rtdm_irq_t irqh;
	rtdm_event_t event;
	char *name;
	struct gpio_desc *desc;
};

struct rtdm_gpio_chan {
	int requested : 1,
	    has_direction : 1,
	    is_output : 1 ;
};

static int gpio_pin_interrupt(rtdm_irq_t *irqh)
{
	struct rtdm_gpio_pin *pin;

	pin = rtdm_irq_get_arg(irqh, struct rtdm_gpio_pin);

	rtdm_event_signal(&pin->event);

	return RTDM_IRQ_HANDLED;
}

static int request_gpio_irq(unsigned int gpio, struct rtdm_gpio_pin *pin,
			    struct rtdm_gpio_chan *chan,
			    int trigger)
{
	int ret, irq_trigger;
	unsigned int irq;

	if (trigger & ~GPIO_TRIGGER_MASK)
		return -EINVAL;

	ret = gpio_request(gpio, pin->name);
	if (ret) {
		if (ret != -EPROBE_DEFER)
			printk(XENO_ERR "cannot request GPIO%d\n", gpio);
		return ret;
	}

	ret = gpio_direction_input(gpio);
	if (ret) {
		printk(XENO_ERR "cannot set GPIO%d as input\n", gpio);
		goto fail;
	}

	chan->has_direction = true;
	gpio_export(gpio, true);

	rtdm_event_clear(&pin->event);
	irq = gpio_to_irq(gpio);

	irq_trigger = 0;
	if (trigger & GPIO_TRIGGER_EDGE_RISING)
		irq_trigger |= IRQ_TYPE_EDGE_RISING;
	if (trigger & GPIO_TRIGGER_EDGE_FALLING)
		irq_trigger |= IRQ_TYPE_EDGE_FALLING;
	if (trigger & GPIO_TRIGGER_LEVEL_HIGH)
		irq_trigger |= IRQ_TYPE_LEVEL_HIGH;
	if (trigger & GPIO_TRIGGER_LEVEL_LOW)
		irq_trigger |= IRQ_TYPE_LEVEL_LOW;

	if (irq_trigger)
		irq_set_irq_type(irq, irq_trigger);
	
	ret = rtdm_irq_request(&pin->irqh, irq, gpio_pin_interrupt,
			       0, pin->name, pin);
	if (ret) {
		printk(XENO_ERR "cannot request GPIO%d interrupt\n", gpio);
		goto fail;
	}

	chan->requested = true;

	rtdm_irq_enable(&pin->irqh);

	return 0;
fail:
	gpio_free(gpio);

	return ret;
}

static void release_gpio_irq(unsigned int gpio, struct rtdm_gpio_pin *pin,
			     struct rtdm_gpio_chan *chan)
{
	rtdm_irq_free(&pin->irqh);
	gpio_free(gpio);
	chan->requested = false;
}

static int gpio_pin_ioctl_nrt(struct rtdm_fd *fd,
			      unsigned int request, void *arg)
{
	struct rtdm_gpio_chan *chan = rtdm_fd_to_private(fd);
	struct rtdm_device *dev = rtdm_fd_device(fd);
	unsigned int gpio = rtdm_fd_minor(fd);
	int ret = 0, val, trigger;
	struct rtdm_gpio_pin *pin;
	
	pin = container_of(dev, struct rtdm_gpio_pin, dev);

	switch (request) {
	case GPIO_RTIOC_DIR_OUT:
		ret = rtdm_safe_copy_from_user(fd, &val, arg, sizeof(val));
		if (ret)
			return ret;
		ret = gpio_direction_output(gpio, val);
		if (ret == 0) {
			chan->has_direction = true;
			chan->is_output = true;
		}
		break;
	case GPIO_RTIOC_DIR_IN:
		ret = gpio_direction_input(gpio);
		if (ret == 0)
			chan->has_direction = true;
		break;
	case GPIO_RTIOC_IRQEN:
		if (chan->requested)
			return -EBUSY;
		ret = rtdm_safe_copy_from_user(fd, &trigger,
				       arg, sizeof(trigger));
		if (ret)
			return ret;
		ret = request_gpio_irq(gpio, pin, chan, trigger);
		break;
	case GPIO_RTIOC_IRQDIS:
		release_gpio_irq(gpio, pin, chan);
		chan->requested = false;
		break;
	default:
		return -EINVAL;
	}
	
	return ret;
}

static ssize_t gpio_pin_read_rt(struct rtdm_fd *fd,
				void __user *buf, size_t len)
{
	struct rtdm_gpio_chan *chan = rtdm_fd_to_private(fd);
	struct rtdm_device *dev = rtdm_fd_device(fd);
	struct rtdm_gpio_pin *pin;
	int value, ret;

	if (len < sizeof(value))
		return -EINVAL;

	if (!chan->has_direction)
		return -EAGAIN;

	if (chan->is_output)
		return -EINVAL;

	pin = container_of(dev, struct rtdm_gpio_pin, dev);

	if (!(fd->oflags & O_NONBLOCK)) {
		ret = rtdm_event_wait(&pin->event);
		if (ret)
			return ret;
	}

	value = gpiod_get_raw_value(pin->desc);
	ret = rtdm_safe_copy_to_user(fd, buf, &value, sizeof(value));
	
	return ret ?: sizeof(value);
}

static ssize_t gpio_pin_write_rt(struct rtdm_fd *fd,
				 const void __user *buf, size_t len)
{
	struct rtdm_gpio_chan *chan = rtdm_fd_to_private(fd);
	struct rtdm_device *dev = rtdm_fd_device(fd);
	struct rtdm_gpio_pin *pin;
	int value, ret;

	if (len < sizeof(value))
		return -EINVAL;

	if (!chan->has_direction)
		return -EAGAIN;

	if (!chan->is_output)
		return -EINVAL;

	ret = rtdm_safe_copy_from_user(fd, &value, buf, sizeof(value));
	if (ret)
		return ret;

	pin = container_of(dev, struct rtdm_gpio_pin, dev);
	gpiod_set_raw_value(pin->desc, value);

	return sizeof(value);
}

static int gpio_pin_select(struct rtdm_fd *fd, struct xnselector *selector,
			   unsigned int type, unsigned int index)
{
	struct rtdm_gpio_chan *chan = rtdm_fd_to_private(fd);
	struct rtdm_device *dev = rtdm_fd_device(fd);
	struct rtdm_gpio_pin *pin;

	if (!chan->has_direction)
		return -EAGAIN;

	if (chan->is_output)
		return -EINVAL;

	pin = container_of(dev, struct rtdm_gpio_pin, dev);

	return rtdm_event_select(&pin->event, selector, type, index);
}

static void gpio_pin_close(struct rtdm_fd *fd)
{
	struct rtdm_gpio_chan *chan = rtdm_fd_to_private(fd);
	struct rtdm_device *dev = rtdm_fd_device(fd);
	unsigned int gpio = rtdm_fd_minor(fd);
	struct rtdm_gpio_pin *pin;

	if (chan->requested) {
		pin = container_of(dev, struct rtdm_gpio_pin, dev);
		release_gpio_irq(gpio, pin, chan);
	}
}

static void delete_pin_devices(struct rtdm_gpio_chip *rgc)
{
	struct rtdm_gpio_pin *pin, *n;
	struct rtdm_device *dev;
	rtdm_lockctx_t s;

	rtdm_lock_get_irqsave(&rgc->lock, s);
	
	list_for_each_entry_safe(pin, n, &rgc->pins, next) {
		list_del(&pin->next);
		rtdm_lock_put_irqrestore(&rgc->lock, s);
		dev = &pin->dev;
		rtdm_dev_unregister(dev);
		rtdm_event_destroy(&pin->event);
		kfree(dev->label);
		kfree(pin->name);
		kfree(pin);
		rtdm_lock_get_irqsave(&rgc->lock, s);
	}

	rtdm_lock_put_irqrestore(&rgc->lock, s);
}

static int create_pin_devices(struct rtdm_gpio_chip *rgc)
{
	struct gpio_chip *gc = rgc->gc;
	struct rtdm_gpio_pin *pin;
	struct rtdm_device *dev;
	rtdm_lockctx_t s;
	int n, ret;

	for (n = gc->base; n < gc->base + gc->ngpio - 1; n++) {
		ret = -ENOMEM;
		pin = kzalloc(sizeof(*pin), GFP_KERNEL);
		if (pin == NULL)
			goto fail;
		pin->name = kasprintf(GFP_KERNEL, "gpio%d", n);
		if (pin->name == NULL)
			goto fail_name;
		pin->desc = gpio_to_desc(n);
		if (pin->desc == NULL) {
			ret = -ENODEV;
			goto fail_desc;
		}
		dev = &pin->dev;
		dev->driver = &rgc->driver;
		dev->label = kasprintf(GFP_KERNEL, "%s/gpio%%d", gc->label);
		if (dev->label == NULL)
			goto fail_label;
		dev->minor = n;
		dev->device_data = rgc;
		ret = rtdm_dev_register(dev);
		if (ret)
			goto fail_register;
		rtdm_event_init(&pin->event, 0);
		rtdm_lock_get_irqsave(&rgc->lock, s);
		list_add_tail(&pin->next, &rgc->pins);
		rtdm_lock_put_irqrestore(&rgc->lock, s);
	}

	return 0;

fail_register:
	kfree(dev->label);
fail_desc:
fail_label:
	kfree(pin->name);
fail_name:
	kfree(pin);
fail:
	delete_pin_devices(rgc);

	return ret;
}

static char *gpio_pin_devnode(struct device *dev, umode_t *mode)
{
	return kasprintf(GFP_KERNEL, "rtdm/%s/%s",
			 dev->class->name,
			 dev_name(dev));
}

int rtdm_gpiochip_add(struct rtdm_gpio_chip *rgc,
		      struct gpio_chip *gc, int gpio_subclass)
{
	int ret;

	if (!realtime_core_enabled())
		return 0;

	rgc->devclass = class_create(gc->owner, gc->label);
	if (IS_ERR(rgc->devclass)) {
		printk(XENO_ERR "cannot create sysfs class\n");
		return PTR_ERR(rgc->devclass);
	}
	rgc->devclass->devnode = gpio_pin_devnode;

	rgc->driver.profile_info = (struct rtdm_profile_info)
		RTDM_PROFILE_INFO(rtdm_gpio_chip,
				  RTDM_CLASS_GPIO,
				  gpio_subclass,
				  0);
	rgc->driver.device_flags = RTDM_NAMED_DEVICE|RTDM_FIXED_MINOR;
	rgc->driver.base_minor = gc->base;
	rgc->driver.device_count = gc->ngpio;
	rgc->driver.context_size = sizeof(struct rtdm_gpio_chan);
	rgc->driver.ops = (struct rtdm_fd_ops){
		.close		=	gpio_pin_close,
		.ioctl_nrt	=	gpio_pin_ioctl_nrt,
		.read_rt	=	gpio_pin_read_rt,
		.write_rt	=	gpio_pin_write_rt,
		.select		=	gpio_pin_select,
	};
	
	rtdm_drv_set_sysclass(&rgc->driver, rgc->devclass);

	rgc->gc = gc;
	INIT_LIST_HEAD(&rgc->pins);
	rtdm_lock_init(&rgc->lock);

	ret = create_pin_devices(rgc);
	if (ret)
		class_destroy(rgc->devclass);
	
	return ret;
}
EXPORT_SYMBOL_GPL(rtdm_gpiochip_add);

void rtdm_gpiochip_remove(struct rtdm_gpio_chip *rgc)
{
	if (!realtime_core_enabled())
		return;

	delete_pin_devices(rgc);
	class_destroy(rgc->devclass);
}
EXPORT_SYMBOL_GPL(rtdm_gpiochip_remove);

static int gpiochip_match_name(struct gpio_chip *chip, void *data)
{
	const char *name = data;

	return !strcmp(chip->label, name);
}

static struct gpio_chip *find_chip_by_name(const char *name)
{
	return gpiochip_find((void *)name, gpiochip_match_name);
}

int rtdm_gpiochip_add_by_name(struct rtdm_gpio_chip *rgc,
			      const char *label, int gpio_subclass)
{
	struct gpio_chip *gc = find_chip_by_name(label);

	if (gc == NULL)
		return -EPROBE_DEFER;

	return rtdm_gpiochip_add(rgc, gc, gpio_subclass);
}
EXPORT_SYMBOL_GPL(rtdm_gpiochip_add_by_name);

#ifdef CONFIG_OF

#include <linux/of_platform.h>

LIST_HEAD(rtdm_gpio_chips);

static DEFINE_MUTEX(chip_lock);

static int match_gpio_chip(struct gpio_chip *gc, void *data)
{
	struct device *dev = data;

	return cobalt_gpiochip_dev(gc) == dev;
}

static int add_gpio_chip(struct gpio_chip *gc, int type)
{
	struct rtdm_gpio_chip *rgc;
	int ret;

	rgc = kzalloc(sizeof(*rgc), GFP_KERNEL);
	if (rgc == NULL)
		return -ENOMEM;

	ret = rtdm_gpiochip_add(rgc, gc, type);
	if (ret) {
		kfree(rgc);
		return ret;
	}

	mutex_lock(&chip_lock);
	list_add(&rgc->next, &rtdm_gpio_chips);
	mutex_unlock(&chip_lock);

	return 0;
}

int rtdm_gpiochip_scan_of(struct device_node *from, const char *compat,
			  int type)
{
	struct device_node *np = from;
	struct platform_device *pdev;
	struct gpio_chip *gc;
	int ret = -ENODEV;

	for (;;) {
		np = of_find_compatible_node(np, NULL, compat);
		if (np == NULL)
			break;
		pdev = of_find_device_by_node(np);
		of_node_put(np);
		if (pdev == NULL)
			break;
		gc = gpiochip_find(&pdev->dev, match_gpio_chip);
		if (gc) {
			ret = add_gpio_chip(gc, type);
			if (ret)
				return ret;
		}
	}

	return ret;
}
EXPORT_SYMBOL_GPL(rtdm_gpiochip_scan_of);

void rtdm_gpiochip_remove_of(int type)
{
	struct rtdm_gpio_chip *rgc, *n;

	list_for_each_entry_safe(rgc, n, &rtdm_gpio_chips, next) {
		if (rgc->driver.profile_info.subclass_id == type) {
			mutex_lock(&chip_lock);
			list_del(&rgc->next);
			mutex_unlock(&chip_lock);
			rtdm_gpiochip_remove(rgc);
			kfree(rgc);
		}
	}
}
EXPORT_SYMBOL_GPL(rtdm_gpiochip_remove_of);

#endif /* CONFIG_OF */
