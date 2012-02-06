/*
 * PWM API implementation
 *
 * Copyright (C) 2011 Bill Gatliff <bgat@billgatliff.com>
 * Copyright (C) 2011 Arun Murthy <arun.murthy@stericsson.com>
 *
 * This program is free software; you may redistribute and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
 * USA
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/completion.h>
#include <linux/workqueue.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/platform_device.h>
#include <linux/cpufreq.h>
#include <linux/pwm/pwm.h>

static const char *REQUEST_SYSFS = "sysfs";
static LIST_HEAD(pwm_device_list);
static DEFINE_MUTEX(device_list_mutex);
static struct class pwm_class;
static struct workqueue_struct *pwm_handler_workqueue;

static int pwm_match_name(struct device *dev, void *name)
{
	return !strcmp(name, dev_name(dev));
}

static struct pwm_device *__pwm_request(struct pwm_device *p, const char *label)
{
	int ret;

	ret = test_and_set_bit(FLAG_REQUESTED, &p->flags);
	if (ret) {
		p = ERR_PTR(-EBUSY);
		goto done;
	}

	p->label = label;
	p->pid = current->pid;

	if (p->ops->request) {
		ret = p->ops->request(p);
		if (ret) {
			p = ERR_PTR(ret);
			clear_bit(FLAG_REQUESTED, &p->flags);
			goto done;
		}
	}

done:
	return p;
}

static struct pwm_device *__pwm_request_byname(const char *name,
					       const char *label)
{
	struct device *d;
	struct pwm_device *p;

	d = class_find_device(&pwm_class, NULL, (char *)name, pwm_match_name);
	if (!d) {
		p = ERR_PTR(-EINVAL);
		goto done;
	}
	if (IS_ERR(d)) {
		p = (struct pwm_device *)d;
		goto done;
	}

	p = __pwm_request(dev_get_drvdata(d), label);

done:
	return p;
}

struct pwm_device *pwm_request_byname(const char *name, const char *label)
{
	struct pwm_device *p;

	mutex_lock(&device_list_mutex);
	p = __pwm_request_byname(name, label);
	mutex_unlock(&device_list_mutex);
	return p;
}
EXPORT_SYMBOL(pwm_request_byname);

struct pwm_device *pwm_request(const char *bus_id, int id, const char *label)
{
	char name[256];
	int ret;

	if (id == -1)
		ret = scnprintf(name, sizeof name, "%s", bus_id);
	else
		ret = scnprintf(name, sizeof name, "%s:%d", bus_id, id);
	if (ret <= 0 || ret >= sizeof name)
		return ERR_PTR(-EINVAL);

	return pwm_request_byname(name, label);
}
EXPORT_SYMBOL(pwm_request);

void pwm_release(struct pwm_device *p)
{
	mutex_lock(&device_list_mutex);

	if (!test_and_clear_bit(FLAG_REQUESTED, &p->flags)) {
		pr_debug("%s pwm device is not requested!\n",
				dev_name(p->dev));
		goto done;
	}

	pwm_stop(p);
	pwm_unsynchronize(p, NULL);
	pwm_set_handler(p, NULL, NULL);

	p->label = NULL;
	p->pid = -1;

	if (p->ops->release)
		p->ops->release(p);
done:
	mutex_unlock(&device_list_mutex);
}
EXPORT_SYMBOL(pwm_release);

unsigned long pwm_ns_to_ticks(struct pwm_device *p, unsigned long nsecs)
{
	unsigned long long ticks;

	ticks = nsecs;
	ticks *= p->tick_hz;
	do_div(ticks, 1000000000);
	return ticks;
}
EXPORT_SYMBOL(pwm_ns_to_ticks);

unsigned long pwm_ticks_to_ns(struct pwm_device *p, unsigned long ticks)
{
	unsigned long long ns;

	if (!p->tick_hz) {
		pr_debug("%s: frequency is zero\n", dev_name(p->dev));
		return 0;
	}

	ns = ticks;
	ns *= 1000000000UL;
	do_div(ns, p->tick_hz);
	return ns;
}
EXPORT_SYMBOL(pwm_ticks_to_ns);

static void pwm_config_ns_to_ticks(struct pwm_device *p, struct pwm_config *c)
{
	if (test_bit(PWM_CONFIG_PERIOD_NS, &c->config_mask)) {
		c->period_ticks = pwm_ns_to_ticks(p, c->period_ns);
		clear_bit(PWM_CONFIG_PERIOD_NS, &c->config_mask);
		set_bit(PWM_CONFIG_PERIOD_TICKS, &c->config_mask);
	}

	if (test_bit(PWM_CONFIG_DUTY_NS, &c->config_mask)) {
		c->duty_ticks = pwm_ns_to_ticks(p, c->duty_ns);
		clear_bit(PWM_CONFIG_DUTY_NS, &c->config_mask);
		set_bit(PWM_CONFIG_DUTY_TICKS, &c->config_mask);
	}
}

static void pwm_config_percent_to_ticks(struct pwm_device *p,
					struct pwm_config *c)
{
	if (test_bit(PWM_CONFIG_DUTY_PERCENT, &c->config_mask)) {
		if (test_bit(PWM_CONFIG_PERIOD_TICKS, &c->config_mask))
			c->duty_ticks = c->period_ticks;
		else
			c->duty_ticks = p->period_ticks;

		c->duty_ticks *= c->duty_percent;
		c->duty_ticks /= 100;
		clear_bit(PWM_CONFIG_DUTY_PERCENT, &c->config_mask);
		set_bit(PWM_CONFIG_DUTY_TICKS, &c->config_mask);
	}
}

int pwm_config_nosleep(struct pwm_device *p, struct pwm_config *c)
{
	if (!p->ops->config_nosleep)
		return -EINVAL;

	pwm_config_ns_to_ticks(p, c);
	pwm_config_percent_to_ticks(p, c);

	return p->ops->config_nosleep(p, c);
}
EXPORT_SYMBOL(pwm_config_nosleep);

int pwm_config(struct pwm_device *p, struct pwm_config *c)
{
	int ret = 0;

	pwm_config_ns_to_ticks(p, c);
	pwm_config_percent_to_ticks(p, c);

	switch (c->config_mask & (BIT(PWM_CONFIG_PERIOD_TICKS)
				  | BIT(PWM_CONFIG_DUTY_TICKS))) {
	case BIT(PWM_CONFIG_PERIOD_TICKS):
		if (p->duty_ticks > c->period_ticks) {
			ret = -EINVAL;
			goto err;
		}
		break;
	case BIT(PWM_CONFIG_DUTY_TICKS):
		if (p->period_ticks < c->duty_ticks) {
			ret = -EINVAL;
			goto err;
		}
		break;
	case BIT(PWM_CONFIG_DUTY_TICKS) | BIT(PWM_CONFIG_PERIOD_TICKS):
		if (c->duty_ticks > c->period_ticks) {
			ret = -EINVAL;
			goto err;
		}
		break;
	default:
		break;
	}

err:
	dev_dbg(p->dev, "%s: config_mask %lu period_ticks %lu duty_ticks %lu"
		" polarity %d duty_ns %lu period_ns %lu duty_percent %d\n",
		__func__, c->config_mask, c->period_ticks, c->duty_ticks,
		c->polarity, c->duty_ns, c->period_ns, c->duty_percent);

	if (ret)
		return ret;
	spin_lock(&p->pwm_lock);
	ret = p->ops->config(p, c);
	spin_unlock(&p->pwm_lock);
	return ret;
}
EXPORT_SYMBOL(pwm_config);

int pwm_set_period_ns(struct pwm_device *p, unsigned long period_ns)
{
	struct pwm_config c = {
		.config_mask = BIT(PWM_CONFIG_PERIOD_TICKS),
		.period_ticks = pwm_ns_to_ticks(p, period_ns),
	};

	spin_lock(&p->pwm_lock);
	p->period_ns = period_ns;
	spin_unlock(&p->pwm_lock);
	return pwm_config(p, &c);
}
EXPORT_SYMBOL(pwm_set_period_ns);

unsigned long pwm_get_period_ns(struct pwm_device *p)
{
	return pwm_ticks_to_ns(p, p->period_ticks);
}
EXPORT_SYMBOL(pwm_get_period_ns);

int pwm_set_frequency(struct pwm_device *p, unsigned long freq)
{
	struct pwm_config c;

	if (!freq)
		return -EINVAL;

	c.config_mask = BIT(PWM_CONFIG_PERIOD_TICKS),
	c.period_ticks = pwm_ns_to_ticks(p, (NSEC_PER_SEC / freq)),
	spin_lock(&p->pwm_lock);
	p->period_ns = NSEC_PER_SEC / freq;
	spin_unlock(&p->pwm_lock);
	return pwm_config(p, &c);
}
EXPORT_SYMBOL(pwm_set_frequency);

unsigned long pwm_get_frequency(struct pwm_device *p)
{
	unsigned long period_ns;

	 period_ns = pwm_ticks_to_ns(p, p->period_ticks);

	if (!period_ns) {
		pr_debug("%s: frequency is zero\n", dev_name(p->dev));
		return 0;
	}

	return	NSEC_PER_SEC / period_ns;
}
EXPORT_SYMBOL(pwm_get_frequency);

int pwm_set_period_ticks(struct pwm_device *p,
					unsigned long ticks)
{
	struct pwm_config c = {
		.config_mask = BIT(PWM_CONFIG_PERIOD_TICKS),
		.period_ticks = ticks,
	};

	spin_lock(&p->pwm_lock);
	p->period_ns = pwm_ticks_to_ns(p, ticks);
	spin_unlock(&p->pwm_lock);
	return pwm_config(p, &c);
}
EXPORT_SYMBOL(pwm_set_period_ticks);

int pwm_set_duty_ns(struct pwm_device *p, unsigned long duty_ns)
{
	struct pwm_config c = {
		.config_mask = BIT(PWM_CONFIG_DUTY_TICKS),
		.duty_ticks = pwm_ns_to_ticks(p, duty_ns),
	};
	spin_lock(&p->pwm_lock);
	p->duty_ns = duty_ns;
	spin_unlock(&p->pwm_lock);
	return pwm_config(p, &c);
}
EXPORT_SYMBOL(pwm_set_duty_ns);

unsigned long pwm_get_duty_ns(struct pwm_device *p)
{
	return pwm_ticks_to_ns(p, p->duty_ticks);
}
EXPORT_SYMBOL(pwm_get_duty_ns);

int pwm_set_duty_percent(struct pwm_device *p, int percent)
{
	struct pwm_config c = {
		.config_mask = BIT(PWM_CONFIG_DUTY_PERCENT),
		.duty_percent = percent,
	};

	spin_lock(&p->pwm_lock);
	p->duty_ns = p->period_ns * percent;
	p->duty_ns /= 100;
	spin_unlock(&p->pwm_lock);
	return pwm_config(p, &c);
}
EXPORT_SYMBOL(pwm_set_duty_percent);

unsigned long pwm_get_duty_percent(struct pwm_device *p)
{
	unsigned long long duty_percent;

	if (!p->period_ns) {
		pr_debug("%s: frequency is zero\n", dev_name(p->dev));
		return 0;
	}

	duty_percent = pwm_ticks_to_ns(p, p->duty_ticks);
	duty_percent *= 100;
	do_div(duty_percent, p->period_ns);
	return duty_percent;
}
EXPORT_SYMBOL(pwm_get_duty_percent);

int pwm_set_duty_ticks(struct pwm_device *p,
					unsigned long ticks)
{
	struct pwm_config c = {
		.config_mask = BIT(PWM_CONFIG_DUTY_TICKS),
		.duty_ticks = ticks,
	};

	spin_lock(&p->pwm_lock);
	p->duty_ns = pwm_ticks_to_ns(p, ticks);
	spin_unlock(&p->pwm_lock);
	return pwm_config(p, &c);
}
EXPORT_SYMBOL(pwm_set_duty_ticks);

int pwm_set_polarity(struct pwm_device *p, int active_high)
{
	struct pwm_config c = {
		.config_mask = BIT(PWM_CONFIG_POLARITY),
		.polarity = active_high,
	};
	return pwm_config(p, &c);
}
EXPORT_SYMBOL(pwm_set_polarity);

int pwm_start(struct pwm_device *p)
{
	struct pwm_config c = {
		.config_mask = BIT(PWM_CONFIG_START),
	};
	return pwm_config(p, &c);
}
EXPORT_SYMBOL(pwm_start);

int pwm_stop(struct pwm_device *p)
{
	struct pwm_config c = {
		.config_mask = BIT(PWM_CONFIG_STOP),
	};
	return pwm_config(p, &c);
}
EXPORT_SYMBOL(pwm_stop);

int pwm_synchronize(struct pwm_device *p, struct pwm_device *to_p)
{
	if (!p->ops->synchronize)
		return -EINVAL;

	return p->ops->synchronize(p, to_p);
}
EXPORT_SYMBOL(pwm_synchronize);

int pwm_unsynchronize(struct pwm_device *p, struct pwm_device *from_p)
{
	if (!p->ops->unsynchronize)
		return -EINVAL;

	return p->ops->unsynchronize(p, from_p);
}
EXPORT_SYMBOL(pwm_unsynchronize);

static void pwm_handler(struct work_struct *w)
{
	struct pwm_device *p = container_of(w, struct pwm_device,
					    handler_work);
	if (p->handler && p->handler(p, p->handler_data))
		pwm_stop(p);
}

static void __pwm_callback(struct pwm_device *p)
{
	queue_work(pwm_handler_workqueue, &p->handler_work);
}

int pwm_set_handler(struct pwm_device *p, pwm_handler_t handler, void *data)
{
	if (p->ops->set_callback) {
		p->handler_data = data;
		p->handler = handler;
		INIT_WORK(&p->handler_work, pwm_handler);
		return p->ops->set_callback(p, handler ? __pwm_callback : NULL);
	}
	return -EINVAL;
}
EXPORT_SYMBOL(pwm_set_handler);

static ssize_t pwm_run_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct pwm_device *p = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", pwm_is_running(p));
}

static ssize_t pwm_run_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t len)
{
	struct pwm_device *p = dev_get_drvdata(dev);
	int ret;

	if (sysfs_streq(buf, "1"))
		ret = pwm_start(p);
	else if (sysfs_streq(buf, "0"))
		ret = pwm_stop(p);
	else
		ret = -EINVAL;

	if (ret < 0)
		return ret;
	return len;
}
static DEVICE_ATTR(run, S_IRUGO | S_IWUSR, pwm_run_show, pwm_run_store);

static ssize_t pwm_tick_hz_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct pwm_device *p = dev_get_drvdata(dev);
	return sprintf(buf, "%lu\n", p->tick_hz);
}
static DEVICE_ATTR(tick_hz, S_IRUGO, pwm_tick_hz_show, NULL);

static ssize_t pwm_duty_ns_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct pwm_device *p = dev_get_drvdata(dev);
	return sprintf(buf, "%lu\n", pwm_get_duty_ns(p));
}

static ssize_t pwm_duty_ns_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t len)
{
	unsigned long duty_ns;
	struct pwm_device *p = dev_get_drvdata(dev);
	int ret;

	if (!kstrtoul(buf, 10, &duty_ns)) {
		ret = pwm_set_duty_ns(p, duty_ns);

		if (ret < 0)
			return ret;
	}

	return len;
}
static DEVICE_ATTR(duty_ns, S_IRUGO | S_IWUSR, pwm_duty_ns_show,
	       pwm_duty_ns_store);

static ssize_t pwm_duty_percent_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct pwm_device *p = dev_get_drvdata(dev);
	return sprintf(buf, "%lu\n", pwm_get_duty_percent(p));
}

static ssize_t pwm_duty_percent_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf,
				 size_t len)
{
	unsigned long duty_ns;
	struct pwm_device *p = dev_get_drvdata(dev);
	int ret;

	if (!kstrtoul(buf, 10, &duty_ns)) {
		ret = pwm_set_duty_percent(p, duty_ns);

		if (ret < 0)
			return ret;
	}

	return len;
}

static DEVICE_ATTR(duty_percent, S_IRUGO | S_IWUSR, pwm_duty_percent_show,
	       pwm_duty_percent_store);

static ssize_t pwm_period_ns_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	struct pwm_device *p = dev_get_drvdata(dev);
	return sprintf(buf, "%lu\n", pwm_get_period_ns(p));
}

static ssize_t pwm_period_ns_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t len)
{
	unsigned long period_ns;
	struct pwm_device *p = dev_get_drvdata(dev);
	int ret;

	if (!kstrtoul(buf, 10, &period_ns)) {
		ret = pwm_set_period_ns(p, period_ns);

		if (ret < 0)
			return ret;
	}

	return len;
}
static DEVICE_ATTR(period_ns, S_IRUGO | S_IWUSR, pwm_period_ns_show,
	       pwm_period_ns_store);

static ssize_t pwm_period_freq_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	struct pwm_device *p = dev_get_drvdata(dev);
	return sprintf(buf, "%lu\n", pwm_get_frequency(p));
}

static ssize_t pwm_period_freq_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf,
				   size_t len)
{
	unsigned long freq_hz;
	int ret;

	struct pwm_device *p = dev_get_drvdata(dev);
	if (!kstrtoul(buf, 10, &freq_hz)) {
		ret = pwm_set_frequency(p, freq_hz);

		if (ret < 0)
			return ret;
	}
	return len;
}

static DEVICE_ATTR(period_freq, S_IRUGO | S_IWUSR, pwm_period_freq_show,
	       pwm_period_freq_store);

static ssize_t pwm_polarity_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct pwm_device *p = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", p->active_high ? 1 : 0);
}

static ssize_t pwm_polarity_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t len)
{
	unsigned long polarity;
	struct pwm_device *p = dev_get_drvdata(dev);
	int ret;

	if (!kstrtoul(buf, 10, &polarity)) {
		ret = pwm_set_polarity(p, polarity);

		if (ret < 0)
			return ret;
	}

	return len;
}
static DEVICE_ATTR(polarity, S_IRUGO | S_IWUSR, pwm_polarity_show,
	       pwm_polarity_store);

static ssize_t pwm_request_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct pwm_device *p = dev_get_drvdata(dev);
	int ret;

	ret = test_bit(FLAG_REQUESTED, &p->flags);

	if (ret)
		return sprintf(buf, "%s requested by %s\n",
				dev_name(p->dev), p->label);
	else
		return sprintf(buf, "%s is free\n",
				dev_name(p->dev));
}

static ssize_t pwm_request_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t len)
{
	struct pwm_device *p = dev_get_drvdata(dev);
	unsigned long request;
	struct pwm_device *ret;

	if (!kstrtoul(buf, 10, &request)) {
		if (request) {
			mutex_lock(&device_list_mutex);
			ret = __pwm_request(p, REQUEST_SYSFS);
			mutex_unlock(&device_list_mutex);

			if (IS_ERR(ret))
				return PTR_ERR(ret);
		} else
			pwm_release(p);
	}

	return len;
}
static DEVICE_ATTR(request, S_IRUGO | S_IWUSR, pwm_request_show,
	       pwm_request_store);

static const struct attribute *pwm_attrs[] = {
	&dev_attr_tick_hz.attr,
	&dev_attr_run.attr,
	&dev_attr_polarity.attr,
	&dev_attr_duty_ns.attr,
	&dev_attr_period_ns.attr,
	&dev_attr_request.attr,
	&dev_attr_duty_percent.attr,
	&dev_attr_period_freq.attr,
	NULL,
};

static const struct attribute_group pwm_device_attr_group = {
	.attrs = (struct attribute **) pwm_attrs,
};

static struct class_attribute pwm_class_attrs[] = {
	__ATTR_NULL,
};

static struct class pwm_class = {
	.name = "pwm",
	.owner = THIS_MODULE,

	.class_attrs = pwm_class_attrs,
};

static int pwm_freq_transition_notifier_cb(struct notifier_block *nb,
		unsigned long val, void *data)
{
	struct pwm_device *p;

	p = container_of(nb, struct pwm_device, freq_transition);

	if (val == CPUFREQ_POSTCHANGE && pwm_is_requested(p))
		p->ops->freq_transition_notifier_cb(p);

	return 0;
}

static inline int pwm_cpufreq_notifier_register(struct pwm_device *p)
{
	p->freq_transition.notifier_call = pwm_freq_transition_notifier_cb;

	return cpufreq_register_notifier(&p->freq_transition,
		       CPUFREQ_TRANSITION_NOTIFIER);
}

int pwm_register_byname(struct pwm_device *p, struct device *parent,
			const char *name)
{
	struct device *d;
	int ret;

	if (!p->ops || !p->ops->config)
		return -EINVAL;

	mutex_lock(&device_list_mutex);

	d = class_find_device(&pwm_class, NULL, (char *)name, pwm_match_name);
	if (d) {
		ret = -EEXIST;
		goto err_found_device;
	}

	p->dev = device_create(&pwm_class, parent, MKDEV(0, 0), NULL, name);
	if (IS_ERR(p->dev)) {
		ret = PTR_ERR(p->dev);
		goto err_device_create;
	}

	ret = sysfs_create_group(&p->dev->kobj, &pwm_device_attr_group);
	if (ret)
		goto err_create_group;

	dev_set_drvdata(p->dev, p);
	p->flags = BIT(FLAG_REGISTERED);

	ret = pwm_cpufreq_notifier_register(p);

	if (ret < 0)
		printk(KERN_ERR "Failed to add cpufreq notifier\n");

	spin_lock_init(&p->pwm_lock);
	goto done;

err_create_group:
	device_unregister(p->dev);
	p->flags = 0;

err_device_create:
err_found_device:
done:
	mutex_unlock(&device_list_mutex);

	return ret;
}
EXPORT_SYMBOL(pwm_register_byname);

int pwm_register(struct pwm_device *p, struct device *parent, int id)
{
	int ret;
	char name[256];

	if (IS_ERR_OR_NULL(parent))
		return -EINVAL;

	if (id == -1)
		ret = scnprintf(name, sizeof name, "%s", dev_name(parent));
	else
		ret = scnprintf(name, sizeof name, "%s:%d",
			       dev_name(parent), id);
	if (ret <= 0 || ret >= sizeof name)
		return -EINVAL;

	return pwm_register_byname(p, parent, name);
}
EXPORT_SYMBOL(pwm_register);

int pwm_unregister(struct pwm_device *p)
{
	int ret = 0;

	mutex_lock(&device_list_mutex);

	if (pwm_is_running(p) || pwm_is_requested(p)) {
		ret = -EBUSY;
		goto done;
	}

	sysfs_remove_group(&p->dev->kobj, &pwm_device_attr_group);
	device_unregister(p->dev);
	p->flags = 0;

done:
	mutex_unlock(&device_list_mutex);

	return ret;
}
EXPORT_SYMBOL(pwm_unregister);

static int __init pwm_init(void)
{
	return class_register(&pwm_class);
}

static void __exit pwm_exit(void)
{
	class_unregister(&pwm_class);
}

#ifdef MODULE
module_init(pwm_init);
module_exit(pwm_exit);
MODULE_LICENSE("GPL");
#else
postcore_initcall(pwm_init);
#endif
