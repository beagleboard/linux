/*
 * Based on PC-Speaker driver for Linux
 *
 * Copyright (C) 1997-2001  David Woodhouse
 * Copyright (C) 2001-2008  Stas Sergeev
 *
 * sndpwm {
 *	compatible = "snd-pwm";
 *	pwms = <&ehrpwm2 1 0 0>;
 *	status = "okay";
 * };
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <linux/delay.h>
#include <asm/bitops.h>
#include <linux/pwm.h>
#include "pwmsp.h"


static int index = SNDRV_DEFAULT_IDX1;	/* Index 0-MAX */
static char *id = SNDRV_DEFAULT_STR1;	/* ID for this card */
static bool enable = SNDRV_DEFAULT_ENABLE1;	/* Enable this card */

module_param(index, int, 0444);
MODULE_PARM_DESC(index, "Index value for pwmsp soundcard.");
module_param(id, charp, 0444);
MODULE_PARM_DESC(id, "ID string for pwmsp soundcard.");
module_param(enable, bool, 0444);
MODULE_PARM_DESC(enable, "Enable PC-Speaker sound.");

struct snd_pwmsp pwmsp_chip;

static int snd_pwmsp_create(struct snd_card *card)
{
	static struct snd_device_ops ops = { };
	int err;

	pr_info("PCSP: Timer resolution is (%linS)\n", hrtimer_resolution);

	spin_lock_init(&pwmsp_chip.substream_lock);
	atomic_set(&pwmsp_chip.timer_active, 0);
	pwmsp_chip.playback_ptr = 0;
	pwmsp_chip.period_ptr = 0;
	pwmsp_chip.enable = 1;
	pwmsp_chip.card = card;

	/* Register device */
	err = snd_device_new(card, SNDRV_DEV_LOWLEVEL, &pwmsp_chip, &ops);
	if (err < 0)
		return err;

	return 0;
}

static int snd_card_pwmsp_probe(int devnum, struct device *dev)
{
	struct snd_card *card;
	int err;

	if (devnum != 0)
		return -EINVAL;

	hrtimer_init(&pwmsp_chip.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	pwmsp_chip.timer.function = pwmsp_do_timer;

	err = snd_card_new(dev, index, id, THIS_MODULE, 0, &card);
	if (err < 0) {
		printk(KERN_ERR "PWM PC-Speaker snd_card_new() call failed.\n");
		return err;
	}

	err = snd_pwmsp_create(card);
	if (err < 0) {
		printk(KERN_ERR "PWM PC-Speaker snd_pwmsp_create() call failed.\n");
		snd_card_free(card);
		return err;
	}
	err = snd_pwmsp_new_pcm(&pwmsp_chip);
	if (err < 0) {
		printk(KERN_ERR "PWM PC-Speaker snd_pwmsp_new_pcm() call failed.\n");
		snd_card_free(card);
		return err;
	}

	strcpy(card->driver, "PC-Speaker");
	strcpy(card->shortname, "pwmsp");
	sprintf(card->longname, "PWM PC-Speaker");

	err = snd_card_register(card);
	if (err < 0) {
		printk(KERN_ERR "PWM PC-Speaker snd_card_register() call failed.\n");
		snd_card_free(card);
		return err;
	}

	return 0;
}

static int alsa_card_pwmsp_init(struct device *dev)
{
	int err;

	err = snd_card_pwmsp_probe(0, dev);
	if (err) {
		printk(KERN_ERR "PC-Speaker initialization failed.\n");
		return err;
	}

#ifdef CONFIG_DEBUG_PAGEALLOC
	/* Well, CONFIG_DEBUG_PAGEALLOC makes the sound horrible. Lets alert */
	printk(KERN_WARNING "PCSP: CONFIG_DEBUG_PAGEALLOC is enabled, "
	       "which may make the sound noisy.\n");
#endif

	return 0;
}

static void alsa_card_pwmsp_exit(struct snd_pwmsp *chip)
{
	snd_card_free(chip->card);
}

static int pwmsp_probe(struct platform_device *dev)
{
	int err;

	pwmsp_chip.pwm = devm_pwm_get(&dev->dev, NULL);
	if (IS_ERR(pwmsp_chip.pwm)) {
		printk(KERN_ERR "PWM PC-Speaker devm_pwm_get() call failed.\n");
		dev_err(&dev->dev, "unable to request PWM\n");
		err = PTR_ERR(pwmsp_chip.pwm);
		return err;
	}

	err = alsa_card_pwmsp_init(&dev->dev);
	if (err < 0) {
		printk(KERN_ERR "PWM PC-Speaker alsa_card_pwmsp_init() call failed.\n");
		return err;
	}

	platform_set_drvdata(dev, &pwmsp_chip);
	return 0;
}

static int pwmsp_remove(struct platform_device *dev)
{
	struct snd_pwmsp *chip = platform_get_drvdata(dev);
	alsa_card_pwmsp_exit(chip);
	return 0;
}

static struct of_device_id pwmsp_of_match[] = {
	{ .compatible = "snd-pwmsp" },
	{ },
};
static struct platform_driver pwmsp_platform_driver = {
	.driver		= {
		.name	= "pwmspkr",
		.owner	= THIS_MODULE,
		.of_match_table = pwmsp_of_match,
	},
	.probe		= pwmsp_probe,
	.remove		= pwmsp_remove,
};
module_platform_driver(pwmsp_platform_driver);

MODULE_LICENSE("GPL");
