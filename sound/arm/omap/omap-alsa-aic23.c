/*
 * arch/arm/mach-omap1/omap-alsa-aic23.c
 * 
 * Alsa codec Driver for AIC23 chip on OSK5912 platform board
 *
 * Copyright (C) 2005 Instituto Nokia de Tecnologia - INdT - Manaus Brazil
 * Written by Daniel Petrini, David Cohen, Anderson Briglia
 *            {daniel.petrini, david.cohen, anderson.briglia}@indt.org.br
 *
 * Copyright (C) 2006 Mika Laitio <lamikr@cc.jyu.fi>
 * 
 * Based in former alsa driver for osk and oss driver
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/control.h>
#include <linux/clk.h>
#include <asm/arch/clock.h>
#include <asm/arch/aic23.h>

#include <asm/arch/omap-alsa.h>
#include "omap-alsa-aic23.h"

static struct clk *aic23_mclk = 0;

/* aic23 related */
static const struct aic23_samplerate_reg_info
 rate_reg_info[NUMBER_SAMPLE_RATES_SUPPORTED] = {
	{4000, 0x06, 1},		/*  4000 */
	{8000, 0x06, 0},		/*  8000 */
	{16000, 0x0C, 1},		/* 16000 */
	{22050, 0x11, 1},               /* 22050 */
	{24000, 0x00, 1},		/* 24000 */
	{32000, 0x0C, 0},		/* 32000 */
	{44100, 0x11, 0},		/* 44100 */
	{48000, 0x00, 0},		/* 48000 */
	{88200, 0x1F, 0},		/* 88200 */
	{96000, 0x0E, 0},		/* 96000 */
};

/*
 * Hardware capabilities
 */
 
 /*
 * DAC USB-mode sampling rates (MCLK = 12 MHz)
 * The rates and rate_reg_into MUST be in the same order
 */
static unsigned int rates[] = {
	4000, 8000, 16000, 22050,
	24000, 32000, 44100,
	48000, 88200, 96000,
};

static snd_pcm_hw_constraint_list_t aic23_hw_constraints_rates = {
	.count = ARRAY_SIZE(rates),
	.list = rates,
	.mask = 0,
};

static snd_pcm_hardware_t aic23_snd_omap_alsa_playback = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER |
		 SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID),	
	.formats = (SNDRV_PCM_FMTBIT_S16_LE),
	.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
		  SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 |
		  SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |
		  SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000 |
		  SNDRV_PCM_RATE_KNOT),
	.rate_min = 8000,
	.rate_max = 96000,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = 128 * 1024,
	.period_bytes_min = 32,
	.period_bytes_max = 8 * 1024,
	.periods_min = 16,
	.periods_max = 255,
	.fifo_size = 0,
};

static snd_pcm_hardware_t aic23_snd_omap_alsa_capture = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER |
		 SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID),
	.formats = (SNDRV_PCM_FMTBIT_S16_LE),
	.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
		  SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 |
		  SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |
		  SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000 |
		  SNDRV_PCM_RATE_KNOT),
	.rate_min = 8000,
	.rate_max = 96000,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = 128 * 1024,
	.period_bytes_min = 32,
	.period_bytes_max = 8 * 1024,
	.periods_min = 16,
	.periods_max = 255,
	.fifo_size = 0,
};

/*
 * Codec/mcbsp init and configuration section
 * codec dependent code.
 */

extern int aic23_write_value(u8 reg, u16 value);

/* TLV320AIC23 is a write only device */
void audio_aic23_write(u8 address, u16 data)
{
	aic23_write_value(address, data);
}
EXPORT_SYMBOL_GPL(audio_aic23_write);

/*
 * Sample rate changing
 */
void aic23_set_samplerate(long rate)
{
	u8 count = 0;
	u16 data = 0;

	/* Fix the rate if it has a wrong value */
	if (rate >= 96000)
		rate = 96000;
	else if (rate >= 88200)
		rate = 88200;
	else if (rate >= 48000)
		rate = 48000;
	else if (rate >= 44100)
		rate = 44100;
	else if (rate >= 32000)
		rate = 32000;
	else if (rate >= 24000)
		rate = 24000;
	else if (rate >= 22050)
		rate = 22050;
	else if (rate >= 16000)
		rate = 16000;
	else if (rate >= 8000)
		rate = 8000;
	else
		rate = 4000;

	/* Search for the right sample rate */
	/* Verify what happens if the rate is not supported
	 * now it goes to 96Khz */
	while ((rate_reg_info[count].sample_rate != rate) &&
	       (count < (NUMBER_SAMPLE_RATES_SUPPORTED - 1))) {
		count++;
	}

	data = (rate_reg_info[count].divider << CLKIN_SHIFT) |
	    (rate_reg_info[count].control << BOSR_SHIFT) | USB_CLK_ON;

	audio_aic23_write(SAMPLE_RATE_CONTROL_ADDR, data);
}

inline void aic23_configure(void)
{
	/* Reset codec */
	audio_aic23_write(RESET_CONTROL_ADDR, 0);

	/* Initialize the AIC23 internal state */

	/* Analog audio path control, DAC selected, delete INSEL_MIC for line in */
	audio_aic23_write(ANALOG_AUDIO_CONTROL_ADDR, DEFAULT_ANALOG_AUDIO_CONTROL);

	/* Digital audio path control, de-emphasis control 44.1kHz */
	audio_aic23_write(DIGITAL_AUDIO_CONTROL_ADDR, DEEMP_44K);

	/* Digital audio interface, master/slave mode, I2S, 16 bit */
#ifdef AIC23_MASTER
	audio_aic23_write(DIGITAL_AUDIO_FORMAT_ADDR,
			  MS_MASTER | IWL_16 | FOR_DSP);
#else
	audio_aic23_write(DIGITAL_AUDIO_FORMAT_ADDR, IWL_16 | FOR_DSP);
#endif

	/* Enable digital interface */
	audio_aic23_write(DIGITAL_INTERFACE_ACT_ADDR, ACT_ON);
}

/*
 *  Omap MCBSP clock configuration and Power Management
 *  
 *  Here we have some functions that allows clock to be enabled and
 *   disabled only when needed. Besides doing clock configuration 
 *   it allows turn on/turn off audio when necessary. 
 */
/*
 * Do clock framework mclk search
 */
void aic23_clock_setup(void)
{
	aic23_mclk = clk_get(0, "mclk");
}

/*
 * Do some sanity check, set clock rate, starts it and
 *  turn codec audio on 
 */
int aic23_clock_on(void)
{
	uint	curRate;

	if (clk_get_usecount(aic23_mclk) > 0) {
		/* MCLK is already in use */
		printk(KERN_WARNING
		       "MCLK in use at %d Hz. We change it to %d Hz\n",
		       (uint) clk_get_rate(aic23_mclk),
		       CODEC_CLOCK);
	}
	curRate	= (uint)clk_get_rate(aic23_mclk);
	if (curRate != CODEC_CLOCK) {
		if (clk_set_rate(aic23_mclk, CODEC_CLOCK)) {
			printk(KERN_ERR
			       "Cannot set MCLK for AIC23 CODEC\n");
			return -ECANCELED;
		}
	}
	clk_enable(aic23_mclk);

	printk(KERN_DEBUG
		"MCLK = %d [%d], usecount = %d\n",
	       (uint) clk_get_rate(aic23_mclk), CODEC_CLOCK,
	       clk_get_usecount(aic23_mclk));

	/* Now turn the audio on */
	audio_aic23_write(POWER_DOWN_CONTROL_ADDR, 
			  ~DEVICE_POWER_OFF & ~OUT_OFF & ~DAC_OFF &
			  ~ADC_OFF & ~MIC_OFF & ~LINE_OFF);	
	return 0;
}

/*
 * Do some sanity check, turn clock off and then turn
 *  codec audio off
 */
int aic23_clock_off(void)
{
	if  (clk_get_usecount(aic23_mclk) > 0) { 
		if (clk_get_rate(aic23_mclk) != CODEC_CLOCK) {
			printk(KERN_WARNING
			       "MCLK for audio should be %d Hz. But is %d Hz\n",
			       (uint) clk_get_rate(aic23_mclk),
			       CODEC_CLOCK);
		}

		clk_disable(aic23_mclk);
	}
	
	audio_aic23_write(POWER_DOWN_CONTROL_ADDR,
			  DEVICE_POWER_OFF | OUT_OFF | DAC_OFF |
			  ADC_OFF | MIC_OFF | LINE_OFF);	
	return 0;
}

int aic23_get_default_samplerate(void)
{
	return DEFAULT_SAMPLE_RATE;
}

static int __devinit snd_omap_alsa_aic23_probe(struct platform_device *pdev)
{
	int	ret;
	struct	omap_alsa_codec_config *codec_cfg;
	
	codec_cfg = pdev->dev.platform_data;
	if (codec_cfg != NULL) {
		codec_cfg->hw_constraints_rates	= &aic23_hw_constraints_rates;
		codec_cfg->snd_omap_alsa_playback  = &aic23_snd_omap_alsa_playback;
		codec_cfg->snd_omap_alsa_capture  = &aic23_snd_omap_alsa_capture;		
		codec_cfg->codec_configure_dev	= aic23_configure;
		codec_cfg->codec_set_samplerate	= aic23_set_samplerate;
		codec_cfg->codec_clock_setup	= aic23_clock_setup;
		codec_cfg->codec_clock_on	= aic23_clock_on;
		codec_cfg->codec_clock_off	= aic23_clock_off;
		codec_cfg->get_default_samplerate = aic23_get_default_samplerate;
		ret	= snd_omap_alsa_post_probe(pdev, codec_cfg);
	}
	else
		ret = -ENODEV;
	return ret;
}

static struct platform_driver omap_alsa_driver = {
	.probe		= snd_omap_alsa_aic23_probe,
	.remove 	= snd_omap_alsa_remove,
	.suspend	= snd_omap_alsa_suspend,
	.resume		= snd_omap_alsa_resume,
	.driver	= {
		.name =	"omap_alsa_mcbsp",
	},
};

static int __init omap_alsa_aic23_init(void)
{
	int err;
	
	ADEBUG();
	err = platform_driver_register(&omap_alsa_driver);

	return err;
}

static void __exit omap_alsa_aic23_exit(void)
{
	ADEBUG();
	
	platform_driver_unregister(&omap_alsa_driver);
}

module_init(omap_alsa_aic23_init);
module_exit(omap_alsa_aic23_exit);
