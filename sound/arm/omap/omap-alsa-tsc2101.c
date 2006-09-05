/*
 * arch/arm/mach-omap1/omap-alsa-tsc2101.c
 * 
 * Alsa codec Driver for TSC2101 chip for OMAP platform boards. 
 * Code obtained from oss omap drivers
 *
 * Copyright (C) 2004 Texas Instruments, Inc.
 * 	Written by Nishanth Menon and Sriram Kannan
 * 	
 * Copyright (C) 2006 Instituto Nokia de Tecnologia - INdT - Manaus Brazil
 * 	Alsa modularization by Daniel Petrini (d.pensator@gmail.com)
 * 
 * Copyright (C) 2006 Mika Laitio <lamikr@cc.jyu.fi>
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
#include <asm/io.h>
#include <asm/arch/mcbsp.h>

#include <linux/slab.h>
#ifdef CONFIG_PM
#include <linux/pm.h>
#endif
#include <asm/mach-types.h>
#include <asm/arch/dma.h>
#include <asm/arch/clock.h>

#include <asm/hardware/tsc2101.h>
#include <../drivers/ssi/omap-tsc2101.h>

#include <asm/arch/omap-alsa.h>
#include "omap-alsa-tsc2101.h"

static struct clk *tsc2101_mclk = 0;

//#define DUMP_TSC2101_AUDIO_REGISTERS
#undef DUMP_TSC2101_AUDIO_REGISTERS

/*
 * Hardware capabilities 
 */

/*
 * DAC USB-mode sampling rates (MCLK = 12 MHz)
 * The rates and rate_reg_into MUST be in the same order
 */
static unsigned int rates[] = {
	7350, 8000, 8018, 8727,
	8820, 9600, 11025, 12000,
	14700, 16000, 22050, 24000,
	29400, 32000, 44100, 48000,
};

static snd_pcm_hw_constraint_list_t tsc2101_hw_constraints_rates = {
	.count = ARRAY_SIZE(rates),
	.list = rates,
	.mask = 0,
};

static const struct tsc2101_samplerate_reg_info
    rate_reg_info[NUMBER_SAMPLE_RATES_SUPPORTED] = {
	/* Div 6 */
	{7350, 7, 1},
	{8000, 7, 0},
	/* Div 5.5 */
	{8018, 6, 1},
	{8727, 6, 0},
	/* Div 5 */
	{8820, 5, 1},
	{9600, 5, 0},	
	/* Div 4 */
	{11025, 4, 1},
	{12000, 4, 0},
	/* Div 3 */
	{14700, 3, 1},
	{16000, 3, 0},
	/* Div 2 */
	{22050, 2, 1},
	{24000, 2, 0},
	/* Div 1.5 */
	{29400, 1, 1},
	{32000, 1, 0},
	/* Div 1 */
	{44100, 0, 1},
	{48000, 0, 0},		
};

static snd_pcm_hardware_t tsc2101_snd_omap_alsa_playback = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER |
		 SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID),	
#ifdef CONFIG_MACH_OMAP_H6300
	.formats = (SNDRV_PCM_FMTBIT_S8),
#else
 	.formats = (SNDRV_PCM_FMTBIT_S16_LE),
#endif
	.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
		  SNDRV_PCM_RATE_16000 |
		  SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 |
		  SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |
		  SNDRV_PCM_RATE_KNOT),
	.rate_min = 7350,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = 128 * 1024,
	.period_bytes_min = 32,
	.period_bytes_max = 8 * 1024,
	.periods_min = 16,
	.periods_max = 255,
	.fifo_size = 0,
};

static snd_pcm_hardware_t tsc2101_snd_omap_alsa_capture = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER |
		 SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID),
	.formats = (SNDRV_PCM_FMTBIT_S16_LE),
	.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
		  SNDRV_PCM_RATE_16000 |
		  SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 |
		  SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |
		  SNDRV_PCM_RATE_KNOT),
	.rate_min = 7350,
	.rate_max = 48000,
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
 * Simplified write for tsc2101 audio registers.
 */
inline void tsc2101_audio_write(u8 address, u16 data)
{
	omap_tsc2101_write(PAGE2_AUDIO_CODEC_REGISTERS, address, data);
}

/* 
 * Simplified read for tsc2101 audio registers.
 */
inline u16 tsc2101_audio_read(u8 address)
{
	return (omap_tsc2101_read(PAGE2_AUDIO_CODEC_REGISTERS, address));
}

#ifdef DUMP_TSC2101_AUDIO_REGISTERS
void dump_tsc2101_audio_reg(void) {
	printk("TSC2101_AUDIO_CTRL_1 = 0x%04x\n",	tsc2101_audio_read(TSC2101_AUDIO_CTRL_1));
	printk("TSC2101_HEADSET_GAIN_CTRL = 0x%04x\n",	tsc2101_audio_read(TSC2101_HEADSET_GAIN_CTRL));
	printk("TSC2101_DAC_GAIN_CTRL = 0x%04x\n", tsc2101_audio_read(TSC2101_DAC_GAIN_CTRL));
	printk("TSC2101_MIXER_PGA_CTRL = 0x%04x\n",	tsc2101_audio_read(TSC2101_MIXER_PGA_CTRL));
	printk("TSC2101_AUDIO_CTRL_2 = 0x%04x\n",	tsc2101_audio_read(TSC2101_AUDIO_CTRL_2));
	printk("TSC2101_CODEC_POWER_CTRL = 0x%04x\n",	tsc2101_audio_read(TSC2101_CODEC_POWER_CTRL));
	printk("TSC2101_AUDIO_CTRL_3 = 0x%04x\n",	tsc2101_audio_read(TSC2101_AUDIO_CTRL_3));
	printk("TSC2101_LCH_BASS_BOOST_N0 = 0x%04x\n",	tsc2101_audio_read(TSC2101_LCH_BASS_BOOST_N0));
	printk("TSC2101_LCH_BASS_BOOST_N1 = 0x%04x\n",	tsc2101_audio_read(TSC2101_LCH_BASS_BOOST_N1));
	printk("TSC2101_LCH_BASS_BOOST_N2 = 0x%04x\n",	tsc2101_audio_read(TSC2101_LCH_BASS_BOOST_N2));
	printk("TSC2101_LCH_BASS_BOOST_N3 = 0x%04x\n",	tsc2101_audio_read(TSC2101_LCH_BASS_BOOST_N3));
	printk("TSC2101_LCH_BASS_BOOST_N4 = 0x%04x\n",	tsc2101_audio_read(TSC2101_LCH_BASS_BOOST_N4));
	printk("TSC2101_LCH_BASS_BOOST_N5 = 0x%04x\n",	tsc2101_audio_read(TSC2101_LCH_BASS_BOOST_N5));
	printk("TSC2101_LCH_BASS_BOOST_D1 = 0x%04x\n",	tsc2101_audio_read(TSC2101_LCH_BASS_BOOST_D1));
	printk("TSC2101_LCH_BASS_BOOST_D2 = 0x%04x\n",	tsc2101_audio_read(TSC2101_LCH_BASS_BOOST_D2));
	printk("TSC2101_LCH_BASS_BOOST_D4 = 0x%04x\n",	tsc2101_audio_read(TSC2101_LCH_BASS_BOOST_D4));
	printk("TSC2101_LCH_BASS_BOOST_D5 = 0x%04x\n",	tsc2101_audio_read(TSC2101_LCH_BASS_BOOST_D5));
	
	printk("TSC2101_RCH_BASS_BOOST_N0 = 0x%04x\n",	tsc2101_audio_read(TSC2101_RCH_BASS_BOOST_N0));
	printk("TSC2101_RCH_BASS_BOOST_N1 = 0x%04x\n",	tsc2101_audio_read(TSC2101_RCH_BASS_BOOST_N1));
	printk("TSC2101_RCH_BASS_BOOST_N2 = 0x%04x\n",	tsc2101_audio_read(TSC2101_RCH_BASS_BOOST_N2));
	printk("TSC2101_RCH_BASS_BOOST_N3 = 0x%04x\n",	tsc2101_audio_read(TSC2101_RCH_BASS_BOOST_N3));
	printk("TSC2101_RCH_BASS_BOOST_N4 = 0x%04x\n",	tsc2101_audio_read(TSC2101_RCH_BASS_BOOST_N4));
	printk("TSC2101_RCH_BASS_BOOST_N5 = 0x%04x\n",	tsc2101_audio_read(TSC2101_RCH_BASS_BOOST_N5));
	printk("TSC2101_RCH_BASS_BOOST_D1 = 0x%04x\n",	tsc2101_audio_read(TSC2101_RCH_BASS_BOOST_D1));
	printk("TSC2101_RCH_BASS_BOOST_D2 = 0x%04x\n",	tsc2101_audio_read(TSC2101_RCH_BASS_BOOST_D2));
	printk("TSC2101_RCH_BASS_BOOST_D4 = 0x%04x\n",	tsc2101_audio_read(TSC2101_RCH_BASS_BOOST_D4));
	printk("TSC2101_RCH_BASS_BOOST_D5 = 0x%04x\n",	tsc2101_audio_read(TSC2101_RCH_BASS_BOOST_D5));
					
	printk("TSC2101_PLL_PROG_1 = 0x%04x\n",	tsc2101_audio_read(TSC2101_PLL_PROG_1));
	printk("TSC2101_PLL_PROG_1 = 0x%04x\n",	tsc2101_audio_read(TSC2101_PLL_PROG_2));
	printk("TSC2101_AUDIO_CTRL_4 = 0x%04x\n",	tsc2101_audio_read(TSC2101_AUDIO_CTRL_4));
	printk("TSC2101_HANDSET_GAIN_CTRL = 0x%04x\n",	tsc2101_audio_read(TSC2101_HANDSET_GAIN_CTRL));
	printk("TSC2101_BUZZER_GAIN_CTRL = 0x%04x\n",	tsc2101_audio_read(TSC2101_BUZZER_GAIN_CTRL));
	printk("TSC2101_AUDIO_CTRL_5 = 0x%04x\n",	tsc2101_audio_read(TSC2101_AUDIO_CTRL_5));
	printk("TSC2101_AUDIO_CTRL_6 = 0x%04x\n",	tsc2101_audio_read(TSC2101_AUDIO_CTRL_6));
	printk("TSC2101_AUDIO_CTRL_7 = 0x%04x\n",	tsc2101_audio_read(TSC2101_AUDIO_CTRL_7));
	printk("TSC2101_GPIO_CTRL = 0x%04x\n",	tsc2101_audio_read(TSC2101_GPIO_CTRL));
	printk("TSC2101_AGC_CTRL = 0x%04x\n",	tsc2101_audio_read(TSC2101_AGC_CTRL));
	printk("TSC2101_POWERDOWN_STS = 0x%04x\n",	tsc2101_audio_read(TSC2101_POWERDOWN_STS));
	printk("TSC2101_MIC_AGC_CONTROL = 0x%04x\n",	tsc2101_audio_read(TSC2101_MIC_AGC_CONTROL));
	printk("TSC2101_CELL_AGC_CONTROL = 0x%04x\n",	tsc2101_audio_read(TSC2101_CELL_AGC_CONTROL));
}
#endif

/*
 * ALSA operations according to board file
 */

/*
 * Sample rate changing
 */
void tsc2101_set_samplerate(long sample_rate)
{
	u8 count = 0;
	u16 data = 0;
	int clkgdv = 0;

	u16 srgr1, srgr2;
	/* wait for any frame to complete */
	udelay(125);
	ADEBUG();

	sample_rate	= sample_rate;
	/* Search for the right sample rate */
	while ((rate_reg_info[count].sample_rate != sample_rate) &&
	       (count < NUMBER_SAMPLE_RATES_SUPPORTED)) {
		count++;
	}
	if (count == NUMBER_SAMPLE_RATES_SUPPORTED) {
		printk(KERN_ERR "Invalid Sample Rate %d requested\n",
		       (int) sample_rate);
		return;		// -EPERM;
	}

	/* Set AC1 */
	data	= tsc2101_audio_read(TSC2101_AUDIO_CTRL_1);
	/* Clear prev settings */
	data	&= ~(AC1_DACFS(0x07) | AC1_ADCFS(0x07));
	data	|= AC1_DACFS(rate_reg_info[count].divisor) | 
			AC1_ADCFS(rate_reg_info[count].divisor);
	tsc2101_audio_write(TSC2101_AUDIO_CTRL_1, data);

	/* Set the AC3 */
	data	= tsc2101_audio_read(TSC2101_AUDIO_CTRL_3);
	/*Clear prev settings */
	data	&= ~(AC3_REFFS | AC3_SLVMS);
	data	|= (rate_reg_info[count].fs_44kHz) ? AC3_REFFS : 0;
#ifdef TSC_MASTER
	data	|= AC3_SLVMS;
#endif				/* #ifdef TSC_MASTER */
	tsc2101_audio_write(TSC2101_AUDIO_CTRL_3, data);

	/* Program the PLLs. This code assumes that the 12 Mhz MCLK is in use.
         * If MCLK rate is something else, these values must be changed.
	 * See the tsc2101 specification for the details.
	 */
	if (rate_reg_info[count].fs_44kHz) {
		/* samplerate = (44.1kHZ / x), where x is int. */
		tsc2101_audio_write(TSC2101_PLL_PROG_1, PLL1_PLLSEL |
				PLL1_PVAL(1) | PLL1_I_VAL(7));	/* PVAL 1; I_VAL 7 */
		tsc2101_audio_write(TSC2101_PLL_PROG_2, PLL2_D_VAL(0x1490));	/* D_VAL 5264 */
	} else {
		/* samplerate = (48.kHZ / x), where x is int. */
		tsc2101_audio_write(TSC2101_PLL_PROG_1, PLL1_PLLSEL |
			       PLL1_PVAL(1) | PLL1_I_VAL(8));	/* PVAL 1; I_VAL 8 */
		tsc2101_audio_write(TSC2101_PLL_PROG_2, PLL2_D_VAL(0x780));	/* D_VAL 1920 */
	}

	/* Set the sample rate */
#ifndef TSC_MASTER
	clkgdv	= CODEC_CLOCK / (sample_rate * (DEFAULT_BITPERSAMPLE * 2 - 1));
	if (clkgdv)
		srgr1 = (FWID(DEFAULT_BITPERSAMPLE - 1) | CLKGDV(clkgdv));
	else
		return (1);

	/* Stereo Mode */
	srgr2 = (CLKSM | FSGM | FPER(DEFAULT_BITPERSAMPLE * 2 - 1));
#else
	srgr1 = (FWID(DEFAULT_BITPERSAMPLE - 1) | CLKGDV(clkgdv));
	srgr2 = ((GSYNC | CLKSP | FSGM | FPER(DEFAULT_BITPERSAMPLE * 2 - 1)));

#endif				/* end of #ifdef TSC_MASTER */
	OMAP_MCBSP_WRITE(OMAP1610_MCBSP1_BASE, SRGR2, srgr2);
	OMAP_MCBSP_WRITE(OMAP1610_MCBSP1_BASE, SRGR1, srgr1);
}

void tsc2101_configure(void)
{
}

/*
 *  Omap MCBSP clock and Power Management configuration
 *  
 *  Here we have some functions that allows clock to be enabled and
 *   disabled only when needed. Besides doing clock configuration 
 *   it allows turn on/turn off audio when necessary. 
 */
 
/*
 * Do clock framework mclk search
 */
void tsc2101_clock_setup(void)
{
	tsc2101_mclk = clk_get(0, "mclk");
}

/*
 * Do some sanity check, set clock rate, starts it and turn codec audio on
 */
int tsc2101_clock_on(void) 
{
	int	curUseCount;
	uint	curRate;
	int	err;

	curUseCount	= clk_get_usecount(tsc2101_mclk);
	DPRINTK("clock use count = %d\n", curUseCount);
	if (curUseCount > 0) {
		// MCLK is already in use
		printk(KERN_WARNING
		       "MCLK already in use at %d Hz. We change it to %d Hz\n",
		       (uint) clk_get_rate(tsc2101_mclk),
		       CODEC_CLOCK);
	}
	curRate	= (uint)clk_get_rate(tsc2101_mclk);
	if (curRate != CODEC_CLOCK) {
		err	= clk_set_rate(tsc2101_mclk, CODEC_CLOCK);
		if (err) {
			printk(KERN_WARNING
			       "Cannot set MCLK clock rate for TSC2101 CODEC, error code = %d\n", err);
			return -ECANCELED;
		}
	}
	err		= clk_enable(tsc2101_mclk);
	curRate		= (uint)clk_get_rate(tsc2101_mclk);
	curUseCount	= clk_get_usecount(tsc2101_mclk);
	DPRINTK("MCLK = %d [%d], usecount = %d, clk_enable retval = %d\n",
	       curRate, 
	       CODEC_CLOCK,
	       curUseCount,
	       err);

	// Now turn the audio on
	omap_tsc2101_write(PAGE2_AUDIO_CODEC_REGISTERS,
			TSC2101_CODEC_POWER_CTRL,
			0x0000);	
	return 0;	
}

/*
 * Do some sanity check, turn clock off and then turn codec audio off
 */
int tsc2101_clock_off(void) 
{
	int curUseCount;
	int curRate;

	curUseCount	= clk_get_usecount(tsc2101_mclk);
	DPRINTK("clock use count = %d\n", curUseCount);
	if  (curUseCount > 0) {
		curRate	= clk_get_rate(tsc2101_mclk);
		DPRINTK("clock rate = %d\n", curRate);
		if (curRate != CODEC_CLOCK) {
			printk(KERN_WARNING
			       "MCLK for audio should be %d Hz. But is %d Hz\n",
			       (uint) clk_get_rate(tsc2101_mclk),
			       CODEC_CLOCK);
		}
		clk_disable(tsc2101_mclk);
		DPRINTK("clock disabled\n");
	}
	tsc2101_audio_write(TSC2101_CODEC_POWER_CTRL,
			    ~(CPC_SP1PWDN | CPC_SP2PWDN | CPC_BASSBC));
	DPRINTK("audio codec off\n");
	return 0;	
}

int tsc2101_get_default_samplerate(void)
{
	return DEFAULT_SAMPLE_RATE;
}

static int __devinit snd_omap_alsa_tsc2101_probe(struct platform_device *pdev)
{
	int	ret;
	struct	omap_alsa_codec_config *codec_cfg;
	
	codec_cfg = pdev->dev.platform_data;
	if (codec_cfg != NULL) {
		codec_cfg->hw_constraints_rates	= &tsc2101_hw_constraints_rates;
		codec_cfg->snd_omap_alsa_playback  = &tsc2101_snd_omap_alsa_playback;
		codec_cfg->snd_omap_alsa_capture  = &tsc2101_snd_omap_alsa_capture;
		codec_cfg->codec_configure_dev	= tsc2101_configure;
		codec_cfg->codec_set_samplerate	= tsc2101_set_samplerate;
		codec_cfg->codec_clock_setup	= tsc2101_clock_setup;
		codec_cfg->codec_clock_on	= tsc2101_clock_on;
		codec_cfg->codec_clock_off	= tsc2101_clock_off;
		codec_cfg->get_default_samplerate = tsc2101_get_default_samplerate;
		ret	= snd_omap_alsa_post_probe(pdev, codec_cfg);
	}
	else
		ret = -ENODEV;
	return ret;
}

static struct platform_driver omap_alsa_driver = {
	.probe		= snd_omap_alsa_tsc2101_probe,
	.remove 	= snd_omap_alsa_remove,
	.suspend	= snd_omap_alsa_suspend,
	.resume		= snd_omap_alsa_resume,
	.driver	= {
		.name =	"omap_alsa_mcbsp",
	},
};

static int __init omap_alsa_tsc2101_init(void)
{	
	ADEBUG();
#ifdef DUMP_TSC2101_AUDIO_REGISTERS
	printk("omap_alsa_tsc2101_init()\n");
	dump_tsc2101_audio_reg();
#endif
	return platform_driver_register(&omap_alsa_driver);
}

static void __exit omap_alsa_tsc2101_exit(void)
{
	ADEBUG();
#ifdef DUMP_TSC2101_AUDIO_REGISTERS
	printk("omap_alsa_tsc2101_exit()\n");
	dump_tsc2101_audio_reg();
#endif
	platform_driver_unregister(&omap_alsa_driver);
}

module_init(omap_alsa_tsc2101_init);
module_exit(omap_alsa_tsc2101_exit);
