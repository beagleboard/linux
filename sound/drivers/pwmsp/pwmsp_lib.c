#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

/*
 * Based on PC-Speaker driver for Linux
 *
 * Copyright (C) 1993-1997  Michael Beck
 * Copyright (C) 1997-2001  David Woodhouse
 * Copyright (C) 2001-2008  Stas Sergeev
 */

#include <linux/module.h>
#include <linux/gfp.h>
#include <linux/moduleparam.h>
#include <linux/interrupt.h>
#include <linux/pwm.h>
#include <sound/pcm.h>
#include "pwmsp.h"

//#define PWMSP_DEBUG
#define DEBUG_TIME

static __maybe_unused ktime_t calltime, delta, rettime;
static __maybe_unused unsigned long duration;
static __maybe_unused unsigned long pwm_config_calls;

/*
 * Call snd_pcm_period_elapsed in a tasklet
 * This avoids spinlock messes and long-running irq contexts
 */
static void pwmsp_call_pcm_elapsed(unsigned long priv)
{
	if (atomic_read(&pwmsp_chip.timer_active)) {
		struct snd_pcm_substream *substream;
		substream = pwmsp_chip.playback_substream;
		if (substream)
			snd_pcm_period_elapsed(substream);
#ifdef DEBUG_TIME
		pr_debug("%lu calls, %lu ns, %lu ns per call\n",
			pwm_config_calls, duration,
			duration / pwm_config_calls);
		pwm_config_calls = 0;
		duration = 0;
#endif
	}
}

static DECLARE_TASKLET(pwmsp_pcm_tasklet, pwmsp_call_pcm_elapsed, 0);

/* write the port and returns the next expire time in ns;
 * called at the trigger-start and in hrtimer callback
 */
static u64 pwmsp_timer_update(struct snd_pwmsp *chip)
{
	struct snd_pcm_substream *substream;
	struct snd_pcm_runtime *runtime;
	int duty_ns;
	s16 val;

	substream = chip->playback_substream;
	if (!substream)
		return 0;

	runtime = substream->runtime;

	/* TODO: Revisit here */
	val = runtime->dma_area[chip->playback_ptr] |
	      runtime->dma_area[chip->playback_ptr + 1] << 8;
	duty_ns = (val * DELTA_DUTY) / 65535 + DELTA_DUTY / 2 + MIN_DUTY_NS;

#ifdef DEBUG_TIME
	calltime = ktime_get();
#endif

	if (duty_ns && chip->enable)
		pwm_config(chip->pwm, duty_ns, PWM_PERIOD_NS);

#ifdef DEBUG_TIME
	rettime = ktime_get();
	delta = ktime_sub(rettime, calltime);
	duration += (unsigned long long) ktime_to_ns(delta) >> 10;
	pwm_config_calls++;
#endif

	return SAMPLING_PERIOD_NS;
}

static void pwmsp_pointer_update(struct snd_pwmsp *chip)
{
	struct snd_pcm_substream *substream;
	size_t period_bytes, buffer_bytes;
	int periods_elapsed;
	unsigned long flags;

	/* update the playback position */
	substream = chip->playback_substream;
	if (!substream)
		return;

	period_bytes = snd_pcm_lib_period_bytes(substream);
	buffer_bytes = snd_pcm_lib_buffer_bytes(substream);

	spin_lock_irqsave(&chip->substream_lock, flags);
	chip->playback_ptr += chip->fmt_size;
	periods_elapsed = chip->playback_ptr - chip->period_ptr;
	if (periods_elapsed < 0) {
#ifdef PWMSP_DEBUG
		printk(KERN_INFO "PWMSP: buffer_bytes mod period_bytes != 0 ? "
			"(%zi %zi %zi)\n",
			chip->playback_ptr, period_bytes, buffer_bytes);
#endif
		periods_elapsed += buffer_bytes;
	}
	periods_elapsed /= period_bytes;
	/* wrap the pointer _before_ calling snd_pcm_period_elapsed(),
	 * or ALSA will BUG on us. */
	chip->playback_ptr %= buffer_bytes;

	if (periods_elapsed) {
		chip->period_ptr += periods_elapsed * period_bytes;
		chip->period_ptr %= buffer_bytes;
	}
	spin_unlock_irqrestore(&chip->substream_lock, flags);

	if (periods_elapsed)
		tasklet_schedule(&pwmsp_pcm_tasklet);
}

enum hrtimer_restart pwmsp_do_timer(struct hrtimer *handle)
{
	struct snd_pwmsp *chip = container_of(handle, struct snd_pwmsp, timer);
	u64 ns;

	if (!atomic_read(&chip->timer_active) || !chip->playback_substream)
		return HRTIMER_NORESTART;

	ns = pwmsp_timer_update(chip);
	if (!ns) {
		printk(KERN_WARNING "PWMSP: unexpected stop\n");
		return HRTIMER_NORESTART;
	}

	hrtimer_forward(handle, hrtimer_get_expires(handle), ns_to_ktime(ns));

	pwmsp_pointer_update(chip);

	return HRTIMER_RESTART;
}

static int pwmsp_start_playing(struct snd_pwmsp *chip)
{
#ifdef PWMSP_DEBUG
	printk(KERN_INFO "PWMSP: start_playing called\n");
#endif
	if (atomic_read(&chip->timer_active)) {
		printk(KERN_ERR "PWMSP: Timer already active\n");
		return -EIO;
	}

	/* Start the PWM */
	pwm_enable(chip->pwm);
	pwm_config(chip->pwm, 0, PWM_PERIOD_NS);
	atomic_set(&chip->timer_active, 1);

	hrtimer_start(&pwmsp_chip.timer, ktime_set(0, 0), HRTIMER_MODE_REL);
	return 0;
}

static void pwmsp_stop_playing(struct snd_pwmsp *chip)
{
#ifdef PWMSP_DEBUG
	printk(KERN_INFO "PWMSP: stop_playing called\n");
#endif
	if (!atomic_read(&chip->timer_active))
		return;

	atomic_set(&chip->timer_active, 0);
	pwm_disable(chip->pwm);
}

/*
 * Force to stop and sync the stream
 */
void pwmsp_sync_stop(struct snd_pwmsp *chip)
{
	local_irq_disable();
	pwmsp_stop_playing(chip);
	local_irq_enable();
	hrtimer_cancel(&chip->timer);
	tasklet_kill(&pwmsp_pcm_tasklet);
}

static int snd_pwmsp_playback_close(struct snd_pcm_substream *substream)
{
	struct snd_pwmsp *chip = snd_pcm_substream_chip(substream);
#ifdef PWMSP_DEBUG
	printk(KERN_INFO "PWMSP: close called\n");
#endif
	pwmsp_sync_stop(chip);
	chip->playback_substream = NULL;
	return 0;
}

static int snd_pwmsp_playback_hw_params(struct snd_pcm_substream *substream,
				       struct snd_pcm_hw_params *hw_params)
{
	struct snd_pwmsp *chip = snd_pcm_substream_chip(substream);
	int err;
	pwmsp_sync_stop(chip);
	err = snd_pcm_lib_malloc_pages(substream,
				      params_buffer_bytes(hw_params));
	if (err < 0)
		return err;
	return 0;
}

static int snd_pwmsp_playback_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_pwmsp *chip = snd_pcm_substream_chip(substream);
#ifdef PWMSP_DEBUG
	printk(KERN_INFO "PWMSP: hw_free called\n");
#endif
	pwmsp_sync_stop(chip);
	return snd_pcm_lib_free_pages(substream);
}

static int snd_pwmsp_playback_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pwmsp *chip = snd_pcm_substream_chip(substream);
	pwmsp_sync_stop(chip);
	chip->playback_ptr = 0;
	chip->period_ptr = 0;
	chip->fmt_size =
		snd_pcm_format_physical_width(substream->runtime->format) >> 3;
	chip->is_signed = snd_pcm_format_signed(substream->runtime->format);
#ifdef PWMSP_DEBUG
	printk(KERN_INFO "PWMSP: prepare called, "
			"size=%zi psize=%zi f=%zi f1=%i fsize=%i\n",
			snd_pcm_lib_buffer_bytes(substream),
			snd_pcm_lib_period_bytes(substream),
			snd_pcm_lib_buffer_bytes(substream) /
			snd_pcm_lib_period_bytes(substream),
			substream->runtime->periods,
			chip->fmt_size);
#endif
	return 0;
}

static int snd_pwmsp_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_pwmsp *chip = snd_pcm_substream_chip(substream);
#ifdef PWMSP_DEBUG
	printk(KERN_INFO "PWMSP: trigger called\n");
#endif
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		return pwmsp_start_playing(chip);
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		pwmsp_stop_playing(chip);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static snd_pcm_uframes_t snd_pwmsp_playback_pointer(struct snd_pcm_substream
						   *substream)
{
	struct snd_pwmsp *chip = snd_pcm_substream_chip(substream);
	unsigned int pos;
	spin_lock(&chip->substream_lock);
	pos = chip->playback_ptr;
	spin_unlock(&chip->substream_lock);
	return bytes_to_frames(substream->runtime, pos);
}

static struct snd_pcm_hardware snd_pwmsp_playback = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED |
		 SNDRV_PCM_INFO_HALF_DUPLEX |
		 SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID),
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rates = SNDRV_PCM_RATE_KNOT,
	.rate_min = SAMPLING_FREQ,
	.rate_max = SAMPLING_FREQ,
	.channels_min = 1,
	.channels_max = 1,
	.buffer_bytes_max = PWMSP_BUFFER_SIZE,
	.period_bytes_min = 64,
	.period_bytes_max = PWMSP_MAX_PERIOD_SIZE,
	.periods_min = 2,
	.periods_max = PWMSP_MAX_PERIODS,
	.fifo_size = 0,
};

static int snd_pwmsp_playback_open(struct snd_pcm_substream *substream)
{
	struct snd_pwmsp *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
#ifdef PWMSP_DEBUG
	printk(KERN_INFO "PWMSP: open called\n");
#endif
	if (atomic_read(&chip->timer_active)) {
		printk(KERN_ERR "PWMSP: still active!!\n");
		return -EBUSY;
	}
	runtime->hw = snd_pwmsp_playback;
	chip->playback_substream = substream;
	return 0;
}

static struct snd_pcm_ops snd_pwmsp_playback_ops = {
	.open = snd_pwmsp_playback_open,
	.close = snd_pwmsp_playback_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = snd_pwmsp_playback_hw_params,
	.hw_free = snd_pwmsp_playback_hw_free,
	.prepare = snd_pwmsp_playback_prepare,
	.trigger = snd_pwmsp_trigger,
	.pointer = snd_pwmsp_playback_pointer,
};

int snd_pwmsp_new_pcm(struct snd_pwmsp *chip)
{
	int err;

	err = snd_pcm_new(chip->card, "pwmspeaker", 0, 1, 0, &chip->pcm);
	if (err < 0)
		return err;

	snd_pcm_set_ops(chip->pcm, SNDRV_PCM_STREAM_PLAYBACK,
			&snd_pwmsp_playback_ops);

	chip->pcm->private_data = chip;
	chip->pcm->info_flags = SNDRV_PCM_INFO_HALF_DUPLEX;
	strcpy(chip->pcm->name, "pwmsp");

	snd_pcm_lib_preallocate_pages_for_all(chip->pcm,
					      SNDRV_DMA_TYPE_CONTINUOUS,
					      snd_dma_continuous_data
					      (GFP_KERNEL), PWMSP_BUFFER_SIZE,
					      PWMSP_BUFFER_SIZE);

	return 0;
}
