/*
 * sound/arm/omap/omap-alsa-tsc2102.c
 * 
 * Alsa codec driver for TSC2102 chip for OMAP platforms.
 *
 * Copyright (c) 2006 Andrzej Zaborowski  <balrog@zabor.org>
 * Code based on the TSC2101 ALSA driver.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include <linux/delay.h>
#include <linux/soundcard.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/spi/tsc2102.h>

#include <asm/arch/mcbsp.h>
#include <asm/arch/dma.h>
#include <asm/arch/clock.h>
#include <asm/arch/omap-alsa.h>

#include "omap-alsa-tsc2102.h"

static struct clk *tsc2102_bclk = 0;

/*
 * Hardware capabilities
 */

/* DAC sampling rates (BCLK = 12 MHz) */
static unsigned int rates[] = {
	7350, 8000, 8820, 9600, 11025, 12000, 14700,
	16000, 22050, 24000, 29400, 32000, 44100, 48000,
};

static snd_pcm_hw_constraint_list_t tsc2102_hw_constraints_rates = {
	.count = ARRAY_SIZE(rates),
	.list = rates,
	.mask = 0,
};

static snd_pcm_hardware_t tsc2102_snd_omap_alsa_playback = {
	.info			= SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_BLOCK_TRANSFER | SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE,
	.rates			= SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
		SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |
		SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
		SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_KNOT,
	.rate_min		= 7350,
	.rate_max		= 48000,
	.channels_min		= 2,
	.channels_max		= 2,
	.buffer_bytes_max	= 128 * 1024,
	.period_bytes_min	= 32,
	.period_bytes_max	= 8 * 1024,
	.periods_min		= 16,
	.periods_max		= 255,
	.fifo_size		= 0,
};

#ifdef DUMP_TSC2102_AUDIO_REGISTERS
static void dump_tsc2102_audio_regs(void) {
	printk("TSC2102_AUDIO1_CTRL = 0x%04x\n",
			tsc2102_read_sync(TSC2102_AUDIO1_CTRL));
	printk("TSC2102_DAC_GAIN_CTRL = 0x%04x\n",
			tsc2102_read_sync(TSC2102_DAC_GAIN_CTRL));
	printk("TSC2102_AUDIO2_CTRL = 0x%04x\n",
			tsc2102_read_sync(TSC2102_AUDIO2_CTRL));
	printk("TSC2102_DAC_POWER_CTRL = 0x%04x\n",
			tsc2102_read_sync(TSC2102_DAC_POWER_CTRL));
	printk("TSC2102_AUDIO3_CTRL = 0x%04x\n",
			tsc2102_read_sync(TSC2102_AUDIO_CTRL_3));
	printk("TSC2102_LCH_BASS_BOOST_N0 = 0x%04x\n",
			tsc2102_read_sync(TSC2102_LCH_BASS_BOOST_N0));
	printk("TSC2102_LCH_BASS_BOOST_N1 = 0x%04x\n",
			tsc2102_read_sync(TSC2102_LCH_BASS_BOOST_N1));
	printk("TSC2102_LCH_BASS_BOOST_N2 = 0x%04x\n",
			tsc2102_read_sync(TSC2102_LCH_BASS_BOOST_N2));
	printk("TSC2102_LCH_BASS_BOOST_N3 = 0x%04x\n",
			tsc2102_read_sync(TSC2102_LCH_BASS_BOOST_N3));
	printk("TSC2102_LCH_BASS_BOOST_N4 = 0x%04x\n",
			tsc2102_read_sync(TSC2102_LCH_BASS_BOOST_N4));
	printk("TSC2102_LCH_BASS_BOOST_N5 = 0x%04x\n",
			tsc2102_read_sync(TSC2102_LCH_BASS_BOOST_N5));
	printk("TSC2102_LCH_BASS_BOOST_D1 = 0x%04x\n",
			tsc2102_read_sync(TSC2102_LCH_BASS_BOOST_D1));
	printk("TSC2102_LCH_BASS_BOOST_D2 = 0x%04x\n",
			tsc2102_read_sync(TSC2102_LCH_BASS_BOOST_D2));
	printk("TSC2102_LCH_BASS_BOOST_D4 = 0x%04x\n",
			tsc2102_read_sync(TSC2102_LCH_BASS_BOOST_D4));
	printk("TSC2102_LCH_BASS_BOOST_D5 = 0x%04x\n",
			tsc2102_read_sync(TSC2102_LCH_BASS_BOOST_D5));
	printk("TSC2102_RCH_BASS_BOOST_N0 = 0x%04x\n",
			tsc2102_read_sync(TSC2102_RCH_BASS_BOOST_N0));
	printk("TSC2102_RCH_BASS_BOOST_N1 = 0x%04x\n",
			tsc2102_read_sync(TSC2102_RCH_BASS_BOOST_N1));
	printk("TSC2102_RCH_BASS_BOOST_N2 = 0x%04x\n",
			tsc2102_read_sync(TSC2102_RCH_BASS_BOOST_N2));
	printk("TSC2102_RCH_BASS_BOOST_N3 = 0x%04x\n",
			tsc2102_read_sync(TSC2102_RCH_BASS_BOOST_N3));
	printk("TSC2102_RCH_BASS_BOOST_N4 = 0x%04x\n",
			tsc2102_read_sync(TSC2102_RCH_BASS_BOOST_N4));
	printk("TSC2102_RCH_BASS_BOOST_N5 = 0x%04x\n",
			tsc2102_read_sync(TSC2102_RCH_BASS_BOOST_N5));
	printk("TSC2102_RCH_BASS_BOOST_D1 = 0x%04x\n",
			tsc2102_read_sync(TSC2102_RCH_BASS_BOOST_D1));
	printk("TSC2102_RCH_BASS_BOOST_D2 = 0x%04x\n",
			tsc2102_read_sync(TSC2102_RCH_BASS_BOOST_D2));
	printk("TSC2102_RCH_BASS_BOOST_D4 = 0x%04x\n",
			tsc2102_read_sync(TSC2102_RCH_BASS_BOOST_D4));
	printk("TSC2102_RCH_BASS_BOOST_D5 = 0x%04x\n",
			tsc2102_read_sync(TSC2102_RCH_BASS_BOOST_D5));
	printk("TSC2102_PLL1_CTRL = 0x%04x\n",
			tsc2102_read_sync(TSC2102_PLL1_CTRL));
	printk("TSC2102_PLL2_CTRL = 0x%04x\n",
			tsc2102_read_sync(TSC2102_PLL2_CTRL));
	printk("TSC2102_AUDIO4_CTRL = 0x%04x\n",
			tsc2102_read_sync(TSC2102_AUDIO4_CTRL));
}
#endif

/*
 * ALSA operations according to board file
 */

static long current_rate = 0;

/*
 * Sample rate changing
 */
static void tsc2102_set_samplerate(long sample_rate)
{
	int clkgdv = 0;
	u16 srgr1, srgr2;

	if (sample_rate == current_rate)
		return;
	current_rate = 0;

	if (tsc2102_set_rate(sample_rate))
		return;

	/* Set the sample rate */
#ifndef TSC_MASTER
	clkgdv = CODEC_CLOCK / (sample_rate * (DEFAULT_BITPERSAMPLE * 2 - 1));
	if (clkgdv)
		srgr1 = (FWID(DEFAULT_BITPERSAMPLE - 1) | CLKGDV(clkgdv));
	else
		return;

	/* Stereo Mode */
	srgr2 = CLKSM | FSGM | FPER(DEFAULT_BITPERSAMPLE * 2 - 1);
#else
	srgr1 = FWID(DEFAULT_BITPERSAMPLE - 1) | CLKGDV(clkgdv);
	srgr2 = GSYNC | CLKSP | FSGM | FPER(DEFAULT_BITPERSAMPLE * 2 - 1);
#endif
	OMAP_MCBSP_WRITE(OMAP1510_MCBSP1_BASE, SRGR2, srgr2);
	OMAP_MCBSP_WRITE(OMAP1510_MCBSP1_BASE, SRGR1, srgr1);
	current_rate = sample_rate;
}

static void tsc2102_configure(void)
{
	tsc2102_dac_power(1);

#ifdef TSC_MASTER
	tsc2102_set_i2s_master(1);
#else
	tsc2102_set_i2s_master(0);
#endif
}

/*
 * Omap McBSP clock and Power Management configuration
 *  
 * Here we have some functions that allow clock to be enabled and
 * disabled only when needed.  Besides doing clock configuration
 * they allow turn audio on and off when necessary.
 */

/*
 * Do clock framework bclk search
 */
static void tsc2102_clock_setup(void)
{
	tsc2102_bclk = clk_get(0, "bclk");
}

/*
 * Do some sanity checks, set clock rate, start it.
 */
static int tsc2102_clock_on(void)
{
	int err;

	if (clk_get_usecount(tsc2102_bclk) > 0 &&
			clk_get_rate(tsc2102_bclk) != CODEC_CLOCK) {
		/* BCLK is already in use */
		printk(KERN_WARNING
			"BCLK already in use at %d Hz. We change it to %d Hz\n",
			(uint) clk_get_rate(tsc2102_bclk), CODEC_CLOCK);

		err = clk_set_rate(tsc2102_bclk, CODEC_CLOCK);
		if (err)
			printk(KERN_WARNING "Cannot set BCLK clock rate "
				"for TSC2102 codec, error code = %d\n", err);
	}

	clk_enable(tsc2102_bclk);
	return 0;
}

/*
 * Turn off the audio codec and then stop the clock.
 */
static int tsc2102_clock_off(void)
{
	DPRINTK("clock use count = %d\n", clk_get_usecount(tsc2102_bclk));

	clk_disable(tsc2102_bclk);
	return 0;
}

static int tsc2102_get_default_samplerate(void)
{
	return DEFAULT_SAMPLE_RATE;
}

static int snd_omap_alsa_tsc2102_suspend(
		struct platform_device *pdev, pm_message_t state)
{
	tsc2102_dac_power(0);
	current_rate = 0;

	return snd_omap_alsa_suspend(pdev, state);
}

static int snd_omap_alsa_tsc2102_resume(struct platform_device *pdev)
{
	tsc2102_dac_power(1);

#ifdef TSC_MASTER
	tsc2102_set_i2s_master(1);
#else
	tsc2102_set_i2s_master(0);
#endif

	return snd_omap_alsa_resume(pdev);
}

static int __init snd_omap_alsa_tsc2102_probe(struct platform_device *pdev)
{
	int ret;
	struct omap_alsa_codec_config *codec_cfg = pdev->dev.platform_data;

	if (codec_cfg) {
		codec_cfg->hw_constraints_rates =
			&tsc2102_hw_constraints_rates;
		codec_cfg->snd_omap_alsa_playback =
			&tsc2102_snd_omap_alsa_playback;
		codec_cfg->codec_configure_dev = tsc2102_configure;
		codec_cfg->codec_set_samplerate = tsc2102_set_samplerate;
		codec_cfg->codec_clock_setup = tsc2102_clock_setup;
		codec_cfg->codec_clock_on = tsc2102_clock_on;
		codec_cfg->codec_clock_off = tsc2102_clock_off;
		codec_cfg->get_default_samplerate =
			tsc2102_get_default_samplerate;
		ret = snd_omap_alsa_post_probe(pdev, codec_cfg);
	} else
		ret = -ENODEV;

	return ret;
}

static int snd_omap_alsa_tsc2102_remove(struct platform_device *pdev)
{
	tsc2102_dac_power(0);

	return snd_omap_alsa_remove(pdev);
}

static struct platform_driver omap_alsa_driver = {
	.probe		= snd_omap_alsa_tsc2102_probe,
	.remove 	= snd_omap_alsa_tsc2102_remove,
	.suspend	= snd_omap_alsa_tsc2102_suspend,
	.resume		= snd_omap_alsa_tsc2102_resume,
	.driver		= {
		.name	= "tsc2102-alsa",
		.owner	= THIS_MODULE,
	},
};

static int __init omap_alsa_tsc2102_init(void)
{
	int err;

	ADEBUG();
	err = platform_driver_register(&omap_alsa_driver);

	return err;
}

static void __exit omap_alsa_tsc2102_exit(void)
{
	ADEBUG();
	platform_driver_unregister(&omap_alsa_driver);
}

module_init(omap_alsa_tsc2102_init);
module_exit(omap_alsa_tsc2102_exit);
