/*
 * Copyright (C) 2011 Bill Gatliff < bgat@billgatliff.com>
 * Copyright (C) 2011 Arun Murthy <arun.murth@stericsson.com>
 *
 * This program is free software; you may redistribute and/or modify
 * it under the terms of the GNU General Public License version 2, as
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
#ifndef __LINUX_PWM_H
#define __LINUX_PWM_H

enum {
	FLAG_REGISTERED		= 0,
	FLAG_REQUESTED		= 1,
	FLAG_STOP		= 2,
	FLAG_RUNNING		= 3,
};

enum {
	PWM_CONFIG_DUTY_TICKS	= 0,
	PWM_CONFIG_PERIOD_TICKS	= 1,
	PWM_CONFIG_POLARITY	= 2,
	PWM_CONFIG_START	= 3,
	PWM_CONFIG_STOP		= 4,

	PWM_CONFIG_HANDLER	= 5,

	PWM_CONFIG_DUTY_NS	= 6,
	PWM_CONFIG_DUTY_PERCENT	= 7,
	PWM_CONFIG_PERIOD_NS	= 8,
};

struct pwm_config;
struct pwm_device;

typedef int (*pwm_handler_t)(struct pwm_device *p, void *data);
typedef void (*pwm_callback_t)(struct pwm_device *p);

struct pwm_device_ops {
	int	(*request)		(struct pwm_device *p);
	void	(*release)		(struct pwm_device *p);
	int	(*config)		(struct pwm_device *p,
					 struct pwm_config *c);
	int	(*config_nosleep)	(struct pwm_device *p,
					 struct pwm_config *c);
	int	(*synchronize)		(struct pwm_device *p,
					 struct pwm_device *to_p);
	int	(*unsynchronize)	(struct pwm_device *p,
					 struct pwm_device *from_p);
	int	(*set_callback)		(struct pwm_device *p,
					 pwm_callback_t callback);
	int	(*freq_transition_notifier_cb) (struct pwm_device *p);
};

struct pwm_config {
	unsigned long config_mask;
	unsigned long duty_ticks;
	unsigned long period_ticks;
	int polarity;

	pwm_handler_t handler;

	unsigned long duty_ns;
	unsigned long period_ns;
	int duty_percent;
};

struct pwm_device {
	struct list_head list;

	struct device *dev;
	struct pwm_device_ops *ops;

	void *data;

	const char *label;
	pid_t pid;

	volatile unsigned long flags;

	unsigned long tick_hz;

	pwm_callback_t callback;

	struct work_struct handler_work;
	pwm_handler_t handler;
	void *handler_data;

	int active_high;
	unsigned long period_ticks;
	unsigned long duty_ticks;
	unsigned long period_ns;
	unsigned long duty_ns;
	struct notifier_block freq_transition;
	unsigned long max_period_ticks;
	spinlock_t pwm_lock;
};

#include <linux/semaphore.h>
#include <linux/pwm/ehrpwm.h>

enum {
	PWM_VERSION_0,
	PWM_VERSION_1,
};

struct pwm_chan_attrib {
	int max_freq;
};

#define PWM_CHANNEL NCHAN

struct pwmss_platform_data {
	int channel_mask;
	u8 version;
	struct pwm_chan_attrib chan_attrib[PWM_CHANNEL];
};

struct pwm_device *pwm_request_byname(const char *name, const char *label);
struct pwm_device *pwm_request(const char *bus_id, int id, const char *label);
void pwm_release(struct pwm_device *p);

static inline int pwm_is_registered(struct pwm_device *p)
{
	return test_bit(FLAG_REGISTERED, &p->flags);
}

static inline int pwm_is_requested(struct pwm_device *p)
{
	return test_bit(FLAG_REQUESTED, &p->flags);
}

static inline int pwm_is_running(struct pwm_device *p)
{
	return test_bit(FLAG_RUNNING, &p->flags);
}

static inline void pwm_set_drvdata(struct pwm_device *p, void *data)
{
	p->data = data;
}

static inline void *pwm_get_drvdata(const struct pwm_device *p)
{
	return p->data;
}

unsigned long pwm_ns_to_ticks(struct pwm_device *p, unsigned long nsecs);
unsigned long pwm_ticks_to_ns(struct pwm_device *p, unsigned long ticks);

int pwm_config_nosleep(struct pwm_device *p, struct pwm_config *c);
int pwm_config(struct pwm_device *p, struct pwm_config *c);

int pwm_set_period_ns(struct pwm_device *p, unsigned long period_ns);
unsigned long pwm_get_period_ns(struct pwm_device *p);
int pwm_set_duty_ns(struct pwm_device *p, unsigned long duty_ns);
unsigned long pwm_get_duty_ns(struct pwm_device *p);
int pwm_set_duty_percent(struct pwm_device *p, int percent);
int pwm_set_polarity(struct pwm_device *p, int active_high);

int pwm_start(struct pwm_device *p);
int pwm_stop(struct pwm_device *p);

int pwm_synchronize(struct pwm_device *p, struct pwm_device *to_p);
int pwm_unsynchronize(struct pwm_device *p, struct pwm_device *from_p);
int pwm_set_handler(struct pwm_device *p, pwm_handler_t handler, void *data);

int pwm_register(struct pwm_device *p, struct device *parent, int id);
int pwm_register_byname(struct pwm_device *p, struct device *parent,
			const char *name);
int pwm_unregister(struct pwm_device *p);

#ifdef CONFIG_GPIO_PWM
struct pwm_device *gpio_pwm_create(int gpio);
int gpio_pwm_destroy(struct pwm_device *p);
#endif
int pwm_set_frequency(struct pwm_device *p, unsigned long freq);
unsigned long pwm_get_frequency(struct pwm_device *p);
int pwm_set_period_ticks(struct pwm_device *p,
					unsigned long ticks);
unsigned long pwm_get_duty_percent(struct pwm_device *p);
int pwm_set_duty_ticks(struct pwm_device *p,
					unsigned long ticks);
#endif
