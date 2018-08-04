/*
 * PC-Speaker driver for Linux
 *
 * Copyright (C) 1993-1997  Michael Beck
 * Copyright (C) 1997-2001  David Woodhouse
 * Copyright (C) 2001-2008  Stas Sergeev
 */

#ifndef __PCSP_H__
#define __PCSP_H__

#include <linux/hrtimer.h>
#include <linux/timex.h>

#define MAX_DUTY_NS	998
#define MIN_DUTY_NS	10
#define DELTA_DUTY	(MAX_DUTY_NS - MIN_DUTY_NS)

#define NSECS_PER_SEC		1000000000UL
#define HZ_TO_NANOSECONDS(x)	(NSECS_PER_SEC / (x))

#define PWM_FREQ		1000000UL /* Period is 1us */
#define SAMPLING_FREQ		44100

#define PWM_PERIOD_NS		HZ_TO_NANOSECONDS(PWM_FREQ)
#define SAMPLING_PERIOD_NS	HZ_TO_NANOSECONDS(SAMPLING_FREQ)

#define PWMSP_MAX_PERIOD_SIZE	(64*1024)
#define PWMSP_MAX_PERIODS	512
#define PWMSP_BUFFER_SIZE	(128*1024)

struct snd_pwmsp {
	struct snd_card *card;
	struct snd_pcm *pcm;
	struct hrtimer timer;
	spinlock_t substream_lock;
	struct snd_pcm_substream *playback_substream;
	unsigned int fmt_size;
	unsigned int is_signed;
	size_t playback_ptr;
	size_t period_ptr;
	atomic_t timer_active;
	int enable;

	struct pwm_device *pwm;
};

extern struct snd_pwmsp pwmsp_chip;

extern enum hrtimer_restart pwmsp_do_timer(struct hrtimer *handle);
extern void pwmsp_sync_stop(struct snd_pwmsp *chip);

extern int snd_pwmsp_new_pcm(struct snd_pwmsp *chip);
extern int snd_pwmsp_new_mixer(struct snd_pwmsp *chip, int nopcm);

void pwmsp_stop_sound(void);

#endif
