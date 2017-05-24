/*
 * This file is part of the Xenomai project.
 *
 * Copyright (C) 2014 Philippe Gerum <rpm@xenomai.org>
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
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/sort.h>
#include <cobalt/kernel/arith.h>
#include <rtdm/driver.h>
#include <rtdm/autotune.h>

MODULE_DESCRIPTION("Xenomai/cobalt core clock autotuner");
MODULE_AUTHOR("Philippe Gerum <rpm@xenomai.org>");
MODULE_LICENSE("GPL");

/* Auto-tuning services for the Cobalt core clock. */

#define SAMPLING_TIME	500000000UL
#define WARMUP_STEPS	3
#define AUTOTUNE_STEPS  40

struct tuning_score {
	int pmean;
	int stddev;
	int minlat;
	unsigned int step;
	unsigned int gravity;
};

struct tuner_state {
	xnticks_t ideal;
	xnticks_t step;
	int min_lat;
	int max_lat;
	int prev_mean;
	int prev_sqs;
	int cur_sqs;
	unsigned int sum;
	unsigned int cur_samples;
	unsigned int max_samples;
};

struct gravity_tuner {
	const char *name;
	unsigned int (*get_gravity)(struct gravity_tuner *tuner);
	void (*set_gravity)(struct gravity_tuner *tuner, unsigned int gravity);
	unsigned int (*adjust_gravity)(struct gravity_tuner *tuner, int adjust);
	int (*init_tuner)(struct gravity_tuner *tuner);
	int (*start_tuner)(struct gravity_tuner *tuner, xnticks_t start_time,
			   xnticks_t interval);
	void (*destroy_tuner)(struct gravity_tuner *tuner);
	struct tuner_state state;
	rtdm_event_t done;
	int status;
	int quiet;
	struct tuning_score scores[AUTOTUNE_STEPS];
	int nscores;
};

struct irq_gravity_tuner {
	rtdm_timer_t timer;
	struct gravity_tuner tuner;
};

struct kthread_gravity_tuner {
	rtdm_task_t task;
	rtdm_event_t barrier;
	xnticks_t start_time;
	xnticks_t interval;
	struct gravity_tuner tuner;
};

struct uthread_gravity_tuner {
	rtdm_timer_t timer;
	rtdm_event_t pulse;
	struct gravity_tuner tuner;
};

struct autotune_context {
	struct gravity_tuner *tuner;
	struct autotune_setup setup;
};

static inline void init_tuner(struct gravity_tuner *tuner)
{
	rtdm_event_init(&tuner->done, 0);
	tuner->status = 0;
}

static inline void destroy_tuner(struct gravity_tuner *tuner)
{
	rtdm_event_destroy(&tuner->done);
}

static inline void done_sampling(struct gravity_tuner *tuner,
				 int status)
{
	tuner->status = status;
	rtdm_event_signal(&tuner->done);
}

static int add_sample(struct gravity_tuner *tuner, xnticks_t timestamp)
{
	struct tuner_state *state;
	int n, delta, cur_mean;

	state = &tuner->state;

	delta = (int)(timestamp - state->ideal);
	if (delta < state->min_lat)
		state->min_lat = delta;
	if (delta > state->max_lat)
		state->max_lat = delta;
	if (delta < 0)
		delta = 0;

	state->sum += delta;
	state->ideal += state->step;
	n = ++state->cur_samples;

	/*
	 * Knuth citing Welford in TAOCP (Vol 2), single-pass
	 * computation of variance using a recurrence relation.
	 */
	if (n == 1)
		state->prev_mean = delta;
	else {
		cur_mean = state->prev_mean + (delta - state->prev_mean) / n;
                state->cur_sqs = state->prev_sqs + (delta - state->prev_mean)
			* (delta - cur_mean);
                state->prev_mean = cur_mean; 
                state->prev_sqs = state->cur_sqs;
	}

	if (n >= state->max_samples) {
		done_sampling(tuner, 0);
		return 1;	/* Finished. */
	}

	return 0;	/* Keep going. */
}

static void timer_handler(rtdm_timer_t *timer)
{
	struct irq_gravity_tuner *irq_tuner;
	xnticks_t now;

	irq_tuner = container_of(timer, struct irq_gravity_tuner, timer);
	now = xnclock_read_raw(&nkclock);

	if (add_sample(&irq_tuner->tuner, now))
		rtdm_timer_stop_in_handler(timer);
}

static int init_irq_tuner(struct gravity_tuner *tuner)
{
	struct irq_gravity_tuner *irq_tuner;
	int ret;

	irq_tuner = container_of(tuner, struct irq_gravity_tuner, tuner);
	ret = rtdm_timer_init(&irq_tuner->timer, timer_handler, "autotune");
	if (ret)
		return ret;

	init_tuner(tuner);

	return 0;
}

static void destroy_irq_tuner(struct gravity_tuner *tuner)
{
	struct irq_gravity_tuner *irq_tuner;

	irq_tuner = container_of(tuner, struct irq_gravity_tuner, tuner);
	rtdm_timer_destroy(&irq_tuner->timer);
	destroy_tuner(tuner);
}

static unsigned int get_irq_gravity(struct gravity_tuner *tuner)
{
	return nkclock.gravity.irq;
}

static void set_irq_gravity(struct gravity_tuner *tuner, unsigned int gravity)
{
	nkclock.gravity.irq = gravity;
}

static unsigned int adjust_irq_gravity(struct gravity_tuner *tuner, int adjust)
{
	return nkclock.gravity.irq += adjust;
}

static int start_irq_tuner(struct gravity_tuner *tuner,
			   xnticks_t start_time, xnticks_t interval)
{
	struct irq_gravity_tuner *irq_tuner;

	irq_tuner = container_of(tuner, struct irq_gravity_tuner, tuner);

	return rtdm_timer_start(&irq_tuner->timer, start_time,
				interval, RTDM_TIMERMODE_ABSOLUTE);
}

struct irq_gravity_tuner irq_tuner = {
	.tuner = {
		.name = "irqhand",
		.init_tuner = init_irq_tuner,
		.destroy_tuner = destroy_irq_tuner,
		.get_gravity = get_irq_gravity,
		.set_gravity = set_irq_gravity,
		.adjust_gravity = adjust_irq_gravity,
		.start_tuner = start_irq_tuner,
	},
};

void task_handler(void *arg)
{
	struct kthread_gravity_tuner *k_tuner = arg;
	xnticks_t now;
	int ret = 0;

	for (;;) {
		if (rtdm_task_should_stop())
			break;

		ret = rtdm_event_wait(&k_tuner->barrier);
		if (ret)
			break;

		ret = rtdm_task_set_period(&k_tuner->task, k_tuner->start_time,
					   k_tuner->interval);
		if (ret)
			break;

		for (;;) {
			ret = rtdm_task_wait_period(NULL);
			if (ret && ret != -ETIMEDOUT)
				goto out;

			now = xnclock_read_raw(&nkclock);
			if (add_sample(&k_tuner->tuner, now)) {
				rtdm_task_set_period(&k_tuner->task, 0, 0);
				break;
			}
		}
	}
out:
	done_sampling(&k_tuner->tuner, ret);
	rtdm_task_destroy(&k_tuner->task);
}

static int init_kthread_tuner(struct gravity_tuner *tuner)
{
	struct kthread_gravity_tuner *k_tuner;

	init_tuner(tuner);
	k_tuner = container_of(tuner, struct kthread_gravity_tuner, tuner);
	rtdm_event_init(&k_tuner->barrier, 0);

	return rtdm_task_init(&k_tuner->task, "autotune",
			      task_handler, k_tuner,
			      RTDM_TASK_HIGHEST_PRIORITY, 0);
}

static void destroy_kthread_tuner(struct gravity_tuner *tuner)
{
	struct kthread_gravity_tuner *k_tuner;

	k_tuner = container_of(tuner, struct kthread_gravity_tuner, tuner);
	rtdm_task_destroy(&k_tuner->task);
	rtdm_event_destroy(&k_tuner->barrier);
}

static unsigned int get_kthread_gravity(struct gravity_tuner *tuner)
{
	return nkclock.gravity.kernel;
}

static void set_kthread_gravity(struct gravity_tuner *tuner, unsigned int gravity)
{
	nkclock.gravity.kernel = gravity;
}

static unsigned int adjust_kthread_gravity(struct gravity_tuner *tuner, int adjust)
{
	return nkclock.gravity.kernel += adjust;
}

static int start_kthread_tuner(struct gravity_tuner *tuner,
			       xnticks_t start_time, xnticks_t interval)
{
	struct kthread_gravity_tuner *k_tuner;

	k_tuner = container_of(tuner, struct kthread_gravity_tuner, tuner);

	k_tuner->start_time = start_time;
	k_tuner->interval = interval;
	rtdm_event_signal(&k_tuner->barrier);

	return 0;
}

struct kthread_gravity_tuner kthread_tuner = {
	.tuner = {
		.name = "kthread",
		.init_tuner = init_kthread_tuner,
		.destroy_tuner = destroy_kthread_tuner,
		.get_gravity = get_kthread_gravity,
		.set_gravity = set_kthread_gravity,
		.adjust_gravity = adjust_kthread_gravity,
		.start_tuner = start_kthread_tuner,
	},
};

static void pulse_handler(rtdm_timer_t *timer)
{
	struct uthread_gravity_tuner *u_tuner;

	u_tuner = container_of(timer, struct uthread_gravity_tuner, timer);
	rtdm_event_signal(&u_tuner->pulse);
}

static int init_uthread_tuner(struct gravity_tuner *tuner)
{
	struct uthread_gravity_tuner *u_tuner;
	int ret;

	u_tuner = container_of(tuner, struct uthread_gravity_tuner, tuner);
	ret = rtdm_timer_init(&u_tuner->timer, pulse_handler, "autotune");
	if (ret)
		return ret;

	xntimer_set_gravity(&u_tuner->timer, XNTIMER_UGRAVITY); /* gasp... */
	rtdm_event_init(&u_tuner->pulse, 0);
	init_tuner(tuner);

	return 0;
}

static void destroy_uthread_tuner(struct gravity_tuner *tuner)
{
	struct uthread_gravity_tuner *u_tuner;

	u_tuner = container_of(tuner, struct uthread_gravity_tuner, tuner);
	rtdm_timer_destroy(&u_tuner->timer);
	rtdm_event_destroy(&u_tuner->pulse);
}

static unsigned int get_uthread_gravity(struct gravity_tuner *tuner)
{
	return nkclock.gravity.user;
}

static void set_uthread_gravity(struct gravity_tuner *tuner, unsigned int gravity)
{
	nkclock.gravity.user = gravity;
}

static unsigned int adjust_uthread_gravity(struct gravity_tuner *tuner, int adjust)
{
	return nkclock.gravity.user += adjust;
}

static int start_uthread_tuner(struct gravity_tuner *tuner,
			       xnticks_t start_time, xnticks_t interval)
{
	struct uthread_gravity_tuner *u_tuner;

	u_tuner = container_of(tuner, struct uthread_gravity_tuner, tuner);

	return rtdm_timer_start(&u_tuner->timer, start_time,
				interval, RTDM_TIMERMODE_ABSOLUTE);
}

static int add_uthread_sample(struct gravity_tuner *tuner,
			      nanosecs_abs_t user_timestamp)
{
	struct uthread_gravity_tuner *u_tuner;
	int ret;

	u_tuner = container_of(tuner, struct uthread_gravity_tuner, tuner);

	if (user_timestamp &&
	    add_sample(tuner, xnclock_ns_to_ticks(&nkclock, user_timestamp))) {
		rtdm_timer_stop(&u_tuner->timer);
		/* Tell the caller to park until next round. */
		ret = -EPIPE;
	} else
		ret = rtdm_event_wait(&u_tuner->pulse);

	return ret;
}

struct uthread_gravity_tuner uthread_tuner = {
	.tuner = {
		.name = "uthread",
		.init_tuner = init_uthread_tuner,
		.destroy_tuner = destroy_uthread_tuner,
		.get_gravity = get_uthread_gravity,
		.set_gravity = set_uthread_gravity,
		.adjust_gravity = adjust_uthread_gravity,
		.start_tuner = start_uthread_tuner,
	},
};

static inline void build_score(struct gravity_tuner *tuner, int step)
{
	struct tuner_state *state = &tuner->state;
	unsigned int variance, n;

	n = state->cur_samples;
	tuner->scores[step].pmean = state->sum / n;
	variance = n > 1 ? state->cur_sqs / (n - 1) : 0;
	tuner->scores[step].stddev = int_sqrt(variance);
	tuner->scores[step].minlat = state->min_lat;
	tuner->scores[step].gravity = tuner->get_gravity(tuner);
	tuner->scores[step].step = step;
	tuner->nscores++;
}

#define progress(__tuner, __fmt, __args...)				\
	do {								\
		if (!(__tuner)->quiet)					\
			printk(XENO_INFO "autotune(%s) " __fmt "\n",	\
			       (__tuner)->name, ##__args);		\
	} while (0)

static int cmp_score_mean(const void *c, const void *r)
{
	const struct tuning_score *sc = c, *sr = r;
	return sc->pmean - sr->pmean;
}

static int cmp_score_stddev(const void *c, const void *r)
{
	const struct tuning_score *sc = c, *sr = r;
	return sc->stddev - sr->stddev;
}

static int cmp_score_minlat(const void *c, const void *r)
{
	const struct tuning_score *sc = c, *sr = r;
	return sc->minlat - sr->minlat;
}

static int cmp_score_gravity(const void *c, const void *r)
{
	const struct tuning_score *sc = c, *sr = r;
	return sc->gravity - sr->gravity;
}

static int filter_mean(struct gravity_tuner *tuner)
{
	sort(tuner->scores, tuner->nscores, sizeof(struct tuning_score),
	     cmp_score_mean, NULL);

	/* Top half of the best pondered means. */

	return (tuner->nscores + 1) / 2;
}

static int filter_stddev(struct gravity_tuner *tuner)
{
	sort(tuner->scores, tuner->nscores, sizeof(struct tuning_score),
	     cmp_score_stddev, NULL);

	/* Top half of the best standard deviations. */

	return (tuner->nscores + 1) / 2;
}

static int filter_minlat(struct gravity_tuner *tuner)
{
	sort(tuner->scores, tuner->nscores, sizeof(struct tuning_score),
	     cmp_score_minlat, NULL);

	/* Top half of the minimum latencies. */

	return (tuner->nscores + 1) / 2;
}

static int filter_gravity(struct gravity_tuner *tuner)
{
	sort(tuner->scores, tuner->nscores, sizeof(struct tuning_score),
	     cmp_score_gravity, NULL);

	/* Smallest gravity required among the shortest latencies. */

	return tuner->nscores;
}

static void dump_scores(struct gravity_tuner *tuner)
{
	int n;

	if (tuner->quiet)
		return;

	for (n = 0; n < tuner->nscores; n++)
		printk(KERN_INFO
		       ".. S%.2d pmean=%Ld stddev=%Lu minlat=%Lu gravity=%Lu\n",
		       tuner->scores[n].step,
		       xnclock_ticks_to_ns(&nkclock, tuner->scores[n].pmean),
		       xnclock_ticks_to_ns(&nkclock, tuner->scores[n].stddev),
		       xnclock_ticks_to_ns(&nkclock, tuner->scores[n].minlat),
		       xnclock_ticks_to_ns(&nkclock, tuner->scores[n].gravity));
}

static inline void filter_score(struct gravity_tuner *tuner,
				int (*filter)(struct gravity_tuner *tuner))
{
	tuner->nscores = filter(tuner);
	dump_scores(tuner);
}

static int tune_gravity(struct gravity_tuner *tuner, int period)
{
	struct tuner_state *state = &tuner->state;
	unsigned int orig_gravity, gravity_limit;
	int ret, step, adjust;

	state->step = xnclock_ns_to_ticks(&nkclock, period);
	state->max_samples = SAMPLING_TIME / (period ?: 1);
	orig_gravity = tuner->get_gravity(tuner);
	tuner->set_gravity(tuner, 0);
	tuner->nscores = 0;
	adjust = xnclock_ns_to_ticks(&nkclock, 500); /* Gravity adjustment step */
	gravity_limit = state->step;
	progress(tuner, "warming up...");

	for (step = 0; step < WARMUP_STEPS + AUTOTUNE_STEPS; step++) {
		state->ideal = xnclock_read_raw(&nkclock) + state->step * WARMUP_STEPS;
		state->min_lat = xnclock_ns_to_ticks(&nkclock, SAMPLING_TIME);
		state->max_lat = 0;
		state->prev_mean = 0;
		state->prev_sqs = 0;
		state->cur_sqs = 0;
		state->sum = 0;
		state->cur_samples = 0;

		ret = tuner->start_tuner(tuner,
					 xnclock_ticks_to_ns(&nkclock, state->ideal),
					 period);
		if (ret)
			goto fail;

		/* Tuner stops when posting. */
		ret = rtdm_event_wait(&tuner->done);
		if (ret)
			goto fail;

		ret = tuner->status;
		if (ret)
			goto fail;

		if (step < WARMUP_STEPS) {
			if (step == WARMUP_STEPS - 1 && state->min_lat >= 0) {
				gravity_limit = state->min_lat;
				progress(tuner, "gravity limit set to %Lu ns",
					 xnclock_ticks_to_ns(&nkclock, gravity_limit));
			}
			continue;
		}

		if (state->min_lat < 0) {
			if (tuner->get_gravity(tuner) == 0) {
				printk(XENO_WARNING
				       "autotune(%s) failed with early shot (%Ld ns)\n",
				       tuner->name,
				       xnclock_ticks_to_ns(&nkclock, state->min_lat));
				ret = -EAGAIN;
				goto fail;
			}
			break;
		}

		if (((step - WARMUP_STEPS) % 5) == 0)
			progress(tuner, "calibrating... (slice %d)",
				 (step - WARMUP_STEPS) / 5 + 1);

		build_score(tuner, step - WARMUP_STEPS);

		/*
		 * Anticipating more than the minimum latency detected
		 * at warmup would make no sense: cap the gravity we
		 * may try.
		 */
		if (tuner->adjust_gravity(tuner, adjust) > gravity_limit) {
			progress(tuner, "gravity limit reached at %Lu ns",
				 xnclock_ticks_to_ns(&nkclock,
						     tuner->get_gravity(tuner)));
			break;
		}
	}

	progress(tuner, "calibration scores");
	dump_scores(tuner);
	progress(tuner, "pondered mean filter");
	filter_score(tuner, filter_mean);
	progress(tuner, "standard deviation filter");
	filter_score(tuner, filter_stddev);
	progress(tuner, "minimum latency filter");
	filter_score(tuner, filter_minlat);
	progress(tuner, "gravity filter");
	filter_score(tuner, filter_gravity);
	tuner->set_gravity(tuner, tuner->scores[0].gravity);

	return 0;
fail:
	tuner->set_gravity(tuner, orig_gravity);

	return ret;
}

static int autotune_ioctl_nrt(struct rtdm_fd *fd, unsigned int request, void *arg)
{
	struct autotune_context *context;
	struct autotune_setup setup;
	struct gravity_tuner *tuner;
	int period, ret;

	if (request == AUTOTUNE_RTIOC_RESET) {
		xnclock_reset_gravity(&nkclock);
		return 0;
	}

	ret = rtdm_copy_from_user(fd, &setup, arg, sizeof(setup));
	if (ret)
		return ret;

	context = rtdm_fd_to_private(fd);

	/* Clear previous tuner. */
	tuner = context->tuner;
	if (tuner) {
		tuner->destroy_tuner(tuner);
		context->tuner = NULL;
	}

	switch (request) {
	case AUTOTUNE_RTIOC_IRQ:
		tuner = &irq_tuner.tuner;
		break;
	case AUTOTUNE_RTIOC_KERN:
		tuner = &kthread_tuner.tuner;
		break;
	case AUTOTUNE_RTIOC_USER:
		tuner = &uthread_tuner.tuner;
		break;
	default:
		return -EINVAL;
	}

	ret = rtdm_safe_copy_from_user(fd, &period, arg, sizeof(period));
	if (ret)
		return ret;

	ret = tuner->init_tuner(tuner);
	if (ret)
		return ret;

	context->tuner = tuner;
	context->setup = setup;

	if (setup.quiet <= 1)
		printk(XENO_INFO "autotune(%s) started\n", tuner->name);

	return ret;
}

static int autotune_ioctl_rt(struct rtdm_fd *fd, unsigned int request, void *arg)
{
	struct autotune_context *context;
	struct gravity_tuner *tuner;
	__u64 timestamp;
	__u32 gravity;
	int ret;

	context = rtdm_fd_to_private(fd);
	tuner = context->tuner;
	if (tuner == NULL)
		return -ENOSYS;

	switch (request) {
	case AUTOTUNE_RTIOC_RUN:
		tuner->quiet = context->setup.quiet;
		ret = tune_gravity(tuner, context->setup.period);
		if (ret)
			break;
		gravity = xnclock_ticks_to_ns(&nkclock,
					      tuner->get_gravity(tuner));
		ret = rtdm_safe_copy_to_user(fd, arg, &gravity,
					     sizeof(gravity));
		break;
	case AUTOTUNE_RTIOC_PULSE:
		if (tuner != &uthread_tuner.tuner)
			return -EINVAL;
		ret = rtdm_safe_copy_from_user(fd, &timestamp, arg,
					       sizeof(timestamp));
		if (ret)
			return ret;
		ret = add_uthread_sample(tuner, timestamp);
		break;
	default:
		ret = -ENOSYS;
	}

	return ret;
}

static int autotune_open(struct rtdm_fd *fd, int oflags)
{
	struct autotune_context *context;

	context = rtdm_fd_to_private(fd);
	context->tuner = NULL;

	return 0;
}

static void autotune_close(struct rtdm_fd *fd)
{
	struct autotune_context *context;
	struct gravity_tuner *tuner;

	context = rtdm_fd_to_private(fd);
	tuner = context->tuner;
	if (tuner) {
		if (context->setup.quiet <= 1)
			printk(XENO_INFO "autotune finished [%Lui/%Luk/%Luu]\n",
			       xnclock_ticks_to_ns(&nkclock,
						   xnclock_get_gravity(&nkclock, irq)),
			       xnclock_ticks_to_ns(&nkclock,
						   xnclock_get_gravity(&nkclock, kernel)),
			       xnclock_ticks_to_ns(&nkclock,
						   xnclock_get_gravity(&nkclock, user)));
		tuner->destroy_tuner(tuner);
	}
}

static struct rtdm_driver autotune_driver = {
	.profile_info		=	RTDM_PROFILE_INFO(autotune,
							  RTDM_CLASS_AUTOTUNE,
							  RTDM_SUBCLASS_AUTOTUNE,
							  0),
	.device_flags		=	RTDM_NAMED_DEVICE|RTDM_EXCLUSIVE,
	.device_count		=	1,
	.context_size		=	sizeof(struct autotune_context),
	.ops = {
		.open		=	autotune_open,
		.ioctl_rt	=	autotune_ioctl_rt,
		.ioctl_nrt	=	autotune_ioctl_nrt,
		.close		=	autotune_close,
	},
};

static struct rtdm_device device = {
	.driver = &autotune_driver,
	.label = "autotune",
};

static int __init autotune_init(void)
{
	if (!realtime_core_enabled())
		return 0;

	return rtdm_dev_register(&device);
}

static void __exit autotune_exit(void)
{
	if (!realtime_core_enabled())
		return;

	rtdm_dev_unregister(&device);
}

module_init(autotune_init);
module_exit(autotune_exit);
