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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
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
#include <rtdm/gpio.h>

struct rtdm_gpio_chan {
	int requested : 1,
		has_direction : 1,
		is_output : 1,
	        is_interrupt : 1,
		want_timestamp : 1;
};

static LIST_HEAD(rtdm_gpio_chips);

static DEFINE_MUTEX(chip_lock);

static int gpio_pin_interrupt(rtdm_irq_t *irqh)
{
	struct rtdm_gpio_pin *pin;

	pin = rtdm_irq_get_arg(irqh, struct rtdm_gpio_pin);

	pin->timestamp = rtdm_clock_read_monotonic();
	rtdm_event_signal(&pin->event);

	return RTDM_IRQ_HANDLED;
}

static int request_gpio_irq(unsigned int gpio, struct rtdm_gpio_pin *pin,
			    struct rtdm_gpio_chan *chan,
			    int trigger)
{
	int ret, irq_trigger, irq;

	if (trigger & ~GPIO_TRIGGER_MASK)
		return -EINVAL;

	if (!chan->requested) {
		ret = gpio_request(gpio, pin->name);
		if (ret) {
			if (ret != -EPROBE_DEFER)
				printk(XENO_ERR 
				       "can not request GPIO%d\n", gpio);
			return ret;
		}
		chan->requested = true;
	}

	ret = gpio_direction_input(gpio);
	if (ret) {
		printk(XENO_ERR "cannot set GPIO%d as input\n", gpio);
		goto fail;
	}

	chan->has_direction = true;
	gpio_export(gpio, true);

	rtdm_event_clear(&pin->event);

	/*
	 * Attempt to hook the interrupt associated to that pin. We
	 * might fail getting a valid IRQ number, in case the GPIO
	 * chip did not define any mapping handler (->to_irq). If so,
	 * just assume that either we have no IRQ indeed, or interrupt
	 * handling may be open coded elsewhere.
	 */
	irq = gpio_to_irq(gpio);
	if (irq < 0)
		goto done;

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


	rtdm_irq_enable(&pin->irqh);
done:
	chan->is_interrupt = true;

	return 0;
fail:
	gpio_free(gpio);
	chan->requested = false;

	return ret;
}

static void release_gpio_irq(unsigned int gpio, struct rtdm_gpio_pin *pin,
			     struct rtdm_gpio_chan *chan)
{
	if (chan->is_interrupt) {
		rtdm_irq_free(&pin->irqh);
		chan->is_interrupt = false;
	}
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
		if (chan->is_interrupt) {
			return -EBUSY;
		}
		ret = rtdm_safe_copy_from_user(fd, &trigger,
					       arg, sizeof(trigger));
		if (ret)
			return ret;
		ret = request_gpio_irq(gpio, pin, chan, trigger);
		break;
	case GPIO_RTIOC_IRQDIS:
		if (chan->is_interrupt) {
			release_gpio_irq(gpio, pin, chan);
			chan->requested = false;
			chan->is_interrupt = false;
		}
		break;
	case GPIO_RTIOC_REQS:
		ret = gpio_request(gpio, pin->name);
		if (ret)
			return ret;
		else
			chan->requested = true;
		break;
	case GPIO_RTIOC_RELS:
		gpio_free(gpio);
		chan->requested = false;
		break;
	case GPIO_RTIOC_TS:
		ret = rtdm_safe_copy_from_user(fd, &val, arg, sizeof(val));
		if (ret)
			return ret;
		chan->want_timestamp = !!val;
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
	struct rtdm_gpio_readout rdo;
	struct rtdm_gpio_pin *pin;
	int ret;

	if (!chan->has_direction)
		return -EAGAIN;

	if (chan->is_output)
		return -EINVAL;

	pin = container_of(dev, struct rtdm_gpio_pin, dev);

	if (chan->want_timestamp) {
		if (len < sizeof(rdo))
			return -EINVAL;

		if (!(fd->oflags & O_NONBLOCK)) {
			ret = rtdm_event_wait(&pin->event);
			if (ret)
				return ret;
			rdo.timestamp = pin->timestamp;
		} else
			rdo.timestamp = rtdm_clock_read_monotonic();

		len = sizeof(rdo);
		rdo.value = gpiod_get_raw_value(pin->desc);
		ret = rtdm_safe_copy_to_user(fd, buf, &rdo, len);
	} else {
		if (len < sizeof(rdo.value))
			return -EINVAL;

		if (!(fd->oflags & O_NONBLOCK)) {
			ret = rtdm_event_wait(&pin->event);
			if (ret)
				return ret;
		}

		len = sizeof(rdo.value);
		rdo.value = gpiod_get_raw_value(pin->desc);
		ret = rtdm_safe_copy_to_user(fd, buf, &rdo.value, len);
	}
	
	return ret ?: len;
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

int gpio_pin_open(struct rtdm_fd *fd, int oflags)
{
	struct rtdm_gpio_chan *chan = rtdm_fd_to_private(fd);
	struct rtdm_device *dev = rtdm_fd_device(fd);
	unsigned int gpio = rtdm_fd_minor(fd);
	int ret = 0;
	struct rtdm_gpio_pin *pin;

	pin = container_of(dev, struct rtdm_gpio_pin, dev);
	ret = gpio_request(gpio, pin->name);
	if (ret) {
		printk(XENO_ERR "failed to request pin %d : %d\n", gpio, ret);
		return ret;
	} else {
		chan->requested = true;
	}

	return 0;
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
	struct rtdm_gpio_pin *pin;
	struct rtdm_device *dev;
	int offset;

	for (offset = 0; offset < rgc->gc->ngpio; offset++) {
		pin = rgc->pins + offset;
		dev = &pin->dev;
		rtdm_dev_unregister(dev);
		rtdm_event_destroy(&pin->event);
		kfree(dev->label);
		kfree(pin->name);
	}
}

static int create_pin_devices(struct rtdm_gpio_chip *rgc)
{
	struct gpio_chip *gc = rgc->gc;
	struct rtdm_gpio_pin *pin;
	struct rtdm_device *dev;
	int offset, ret, gpio;

	for (offset = 0; offset < gc->ngpio; offset++) {
		ret = -ENOMEM;
		gpio = gc->base + offset;
		pin = rgc->pins + offset;
		pin->name = kasprintf(GFP_KERNEL, "gpio%d", gpio);
		if (pin->name == NULL)
			goto fail_name;
		pin->desc = gpio_to_desc(gpio);
		if (pin->desc == NULL) {
			ret = -ENODEV;
			goto fail_desc;
		}
		dev = &pin->dev;
		dev->driver = &rgc->driver;
		dev->label = kasprintf(GFP_KERNEL, "%s/gpio%%d", gc->label);
		if (dev->label == NULL)
			goto fail_label;
		dev->minor = gpio;
		dev->device_data = rgc;
		ret = rtdm_dev_register(dev);
		if (ret)
			goto fail_register;
		rtdm_event_init(&pin->event, 0);
	}

	return 0;

fail_register:
	kfree(dev->label);
fail_desc:
fail_label:
	kfree(pin->name);
fail_name:
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
		.open		=	gpio_pin_open,
		.close		=	gpio_pin_close,
		.ioctl_nrt	=	gpio_pin_ioctl_nrt,
		.read_rt	=	gpio_pin_read_rt,
		.write_rt	=	gpio_pin_write_rt,
		.select		=	gpio_pin_select,
	};
	
	rtdm_drv_set_sysclass(&rgc->driver, rgc->devclass);

	rgc->gc = gc;
	rtdm_lock_init(&rgc->lock);

	ret = create_pin_devices(rgc);
	if (ret)
		class_destroy(rgc->devclass);
	
	return ret;
}
EXPORT_SYMBOL_GPL(rtdm_gpiochip_add);

struct rtdm_gpio_chip *
rtdm_gpiochip_alloc(struct gpio_chip *gc, int gpio_subclass)
{
	struct rtdm_gpio_chip *rgc;
	size_t asize;
	int ret;

	if (gc->ngpio == 0)
		return ERR_PTR(-EINVAL);

	asize = sizeof(*rgc) + gc->ngpio * sizeof(struct rtdm_gpio_pin);
	rgc = kzalloc(asize, GFP_KERNEL);
	if (rgc == NULL)
		return ERR_PTR(-ENOMEM);

	ret = rtdm_gpiochip_add(rgc, gc, gpio_subclass);
	if (ret) {
		kfree(rgc);
		return ERR_PTR(ret);
	}

	mutex_lock(&chip_lock);
	list_add(&rgc->next, &rtdm_gpio_chips);
	mutex_unlock(&chip_lock);

	return rgc;
}
EXPORT_SYMBOL_GPL(rtdm_gpiochip_alloc);

void rtdm_gpiochip_remove(struct rtdm_gpio_chip *rgc)
{
	mutex_lock(&chip_lock);
	list_del(&rgc->next);
	mutex_unlock(&chip_lock);
	delete_pin_devices(rgc);
	class_destroy(rgc->devclass);
}
EXPORT_SYMBOL_GPL(rtdm_gpiochip_remove);

int rtdm_gpiochip_post_event(struct rtdm_gpio_chip *rgc,
			     unsigned int offset)
{
	struct rtdm_gpio_pin *pin;

	if (offset >= rgc->gc->ngpio)
		return -EINVAL;

	pin = rgc->pins + offset;
	pin->timestamp = rtdm_clock_read_monotonic();
	rtdm_event_signal(&pin->event);
	
	return 0;
}
EXPORT_SYMBOL_GPL(rtdm_gpiochip_post_event);

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

struct gpiochip_holder {
	struct gpio_chip *chip;
	struct list_head next;
};
	
struct gpiochip_match_data {
	struct device *parent;
	struct list_head list;
};

static int match_gpio_chip(struct gpio_chip *gc, void *data)
{
	struct gpiochip_match_data *d = data;
	struct gpiochip_holder *h;

	if (cobalt_gpiochip_dev(gc) == d->parent) {
		h = kmalloc(sizeof(*h), GFP_KERNEL);
		if (h) {
			h->chip = gc;
			list_add(&h->next, &d->list);
		}
	}

	/*
	 * Iterate over all existing GPIO chips, we may have several
	 * hosted by the same pin controller mapping different ranges.
	 */
	return 0;
}

int rtdm_gpiochip_scan_of(struct device_node *from, const char *compat,
			  int type)
{
	struct gpiochip_match_data match;
	struct gpiochip_holder *h, *n;
	struct device_node *np = from;
	struct platform_device *pdev;
	struct rtdm_gpio_chip *rgc;
	int ret = -ENODEV, _ret;

	if (!rtdm_available())
		return -ENOSYS;

	for (;;) {
		np = of_find_compatible_node(np, NULL, compat);
		if (np == NULL)
			break;
		pdev = of_find_device_by_node(np);
		of_node_put(np);
		if (pdev == NULL)
			break;
		match.parent = &pdev->dev;
		INIT_LIST_HEAD(&match.list);
		gpiochip_find(&match, match_gpio_chip);
		if (!list_empty(&match.list)) {
			ret = 0;
			list_for_each_entry_safe(h, n, &match.list, next) {
				list_del(&h->next);
				_ret = 0;
				rgc = rtdm_gpiochip_alloc(h->chip, type);
				if (IS_ERR(rgc))
					_ret = PTR_ERR(rgc);
				kfree(h);
				if (_ret && !ret)
					ret = _ret;
			}
			if (ret)
				break;
		}
	}

	return ret;
}
EXPORT_SYMBOL_GPL(rtdm_gpiochip_scan_of);

int rtdm_gpiochip_scan_array_of(struct device_node *from,
				const char *compat[],
				int nentries, int type)
{
	int ret = -ENODEV, _ret, n;

	for (n = 0; n < nentries; n++) {
		_ret = rtdm_gpiochip_scan_of(from, compat[n], type);
		if (_ret) {
			if (_ret != -ENODEV)
				return _ret;
		} else
			ret = 0;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(rtdm_gpiochip_scan_array_of);

void rtdm_gpiochip_remove_of(int type)
{
	struct rtdm_gpio_chip *rgc, *n;

	mutex_lock(&chip_lock);

	list_for_each_entry_safe(rgc, n, &rtdm_gpio_chips, next) {
		if (rgc->driver.profile_info.subclass_id == type) {
			mutex_unlock(&chip_lock);
			rtdm_gpiochip_remove(rgc);
			kfree(rgc);
			mutex_lock(&chip_lock);
		}
	}

	mutex_unlock(&chip_lock);
}
EXPORT_SYMBOL_GPL(rtdm_gpiochip_remove_of);

#endif /* CONFIG_OF */
