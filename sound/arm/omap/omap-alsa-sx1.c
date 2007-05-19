/*
 * Alsa codec Driver for Siemens SX1 board.
 * based on omap-alsa-tsc2101.c	and cn_test.c example by Evgeniy Polyakov
 *
 * Copyright (C) 2006 Vladimir Ananiev (vovan888 at gmail com)
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
#include <linux/pm.h>
#include <asm/arch/dma.h>
#include <asm/arch/clock.h>
#include <asm/arch/gpio.h>

#include <asm/arch/omap-alsa.h>
#include "omap-alsa-sx1.h"

#include <linux/connector.h>

/* Connector implementation */
static struct cb_id cn_sx1snd_id = { CN_IDX_SX1SND, CN_VAL_SX1SND };
static char cn_sx1snd_name[] = "cn_sx1snd";

static void cn_sx1snd_callback(void *data)
{
	struct cn_msg *msg = (struct cn_msg *)data;

	printk("%s: %lu: idx=%x, val=%x, seq=%u, ack=%u, len=%d: %s.\n",
			__func__, jiffies, msg->id.idx, msg->id.val,
			msg->seq, msg->ack, msg->len, (char *)msg->data);
}

/* Send IPC message to sound server */
int cn_sx1snd_send(unsigned int cmd, unsigned int arg1, unsigned int arg2)
{
	struct cn_msg *m;
	unsigned short data[3];
	int err;

	m = kzalloc(sizeof(*m) + sizeof(data), gfp_any());
	if (!m)
		return -1;

	memcpy(&m->id, &cn_sx1snd_id, sizeof(m->id));
	m->seq = 1;
	m->len = sizeof(data);

	data[0] = (unsigned short)cmd;
	data[1] = (unsigned short)arg1;
	data[2] = (unsigned short)arg2;

	memcpy(m + 1, data, m->len);

	err = cn_netlink_send(m, CN_IDX_SX1SND, gfp_any());
	snd_printd("sent= %02X %02X %02X, err=%d\n", cmd,arg1,arg2,err);
	kfree(m);

	if (err == -ESRCH)
		return -1;	/* there are no listeners on socket */
	return 0;
}

/* Hardware capabilities
 *
 * DAC USB-mode sampling rates (MCLK = 12 MHz)
 * The rates and rate_reg_into MUST be in the same order
 */
static unsigned int rates[] = {
	 8000, 11025, 12000,
	 16000, 22050, 24000,
	 32000, 44100, 48000,
};

static struct snd_pcm_hw_constraint_list egold_hw_constraints_rates = {
	.count	= ARRAY_SIZE(rates),
	.list	= rates,
	.mask	= 0,
};

static struct snd_pcm_hardware egold_snd_omap_alsa_playback = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER |
		 SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID),
	.formats = (SNDRV_PCM_FMTBIT_S16_LE),
	.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
		  SNDRV_PCM_RATE_16000 |
		  SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 |
		  SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |
		  SNDRV_PCM_RATE_KNOT),
	.rate_min		= 8000,
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

static struct snd_pcm_hardware egold_snd_omap_alsa_capture = {
	.info = (SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER |
		 SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID),
	.formats = (SNDRV_PCM_FMTBIT_S16_LE),
	.rates = (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |
		  SNDRV_PCM_RATE_16000 |
		  SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 |
		  SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |
		  SNDRV_PCM_RATE_KNOT),
	.rate_min		= 8000,
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

static long current_rate = -1; /* current rate in egold format 0..8 */
/*
 * ALSA operations according to board file
 */

/*
 * Sample rate changing
 */
static void egold_set_samplerate(long sample_rate)
{
	int egold_rate = 0;
	int clkgdv = 0;
	u16 srgr1, srgr2;

	/* Set the sample rate */
#if 0
	/* fw15: 5005E490 - divs are different !!! */
	clkgdv	= CODEC_CLOCK / (sample_rate * (DEFAULT_BITPERSAMPLE * 2 - 1));
#endif
	switch (sample_rate) {
		case 8000:	clkgdv = 71; egold_rate = FRQ_8000; break;
		case 11025:	clkgdv = 51; egold_rate = FRQ_11025; break;
		case 12000:	clkgdv = 47; egold_rate = FRQ_12000; break;
		case 16000:	clkgdv = 35; egold_rate = FRQ_16000; break;
		case 22050:	clkgdv = 25; egold_rate = FRQ_22050; break;
		case 24000:	clkgdv = 23; egold_rate = FRQ_24000; break;
		case 32000:	clkgdv = 17; egold_rate = FRQ_32000; break;
		case 44100:	clkgdv = 12; egold_rate = FRQ_44100; break;
		case 48000:	clkgdv = 11; egold_rate = FRQ_48000; break;
	}

	srgr1 = (FWID(DEFAULT_BITPERSAMPLE - 1) | CLKGDV(clkgdv));
	srgr2 = ((FSGM | FPER(DEFAULT_BITPERSAMPLE * 2 - 1)));

	OMAP_MCBSP_WRITE(OMAP1510_MCBSP1_BASE, SRGR2, srgr2);
	OMAP_MCBSP_WRITE(OMAP1510_MCBSP1_BASE, SRGR1, srgr1);
	current_rate = egold_rate;
	snd_printd("set samplerate=%ld\n", sample_rate);

}

static void egold_configure(void)
{
}

/*
 * Omap MCBSP clock and Power Management configuration
 *
 * Here we have some functions that allows clock to be enabled and
 * disabled only when needed. Besides doing clock configuration
 * it allows turn on/turn off audio when necessary.
 */

/*
 * Do clock framework mclk search
 */
static void egold_clock_setup(void)
{
	omap_request_gpio(OSC_EN);
	omap_set_gpio_direction(OSC_EN, 0); /* output */
	snd_printd("\n");
}

/*
 * Do some sanity check, set clock rate, starts it and turn codec audio on
 */
static int egold_clock_on(void)
{
	omap_set_gpio_dataout(OSC_EN, 1);
	egold_set_samplerate(44100); /* TODO */
	cn_sx1snd_send(DAC_SETAUDIODEVICE, SX1_DEVICE_SPEAKER, 0);
	cn_sx1snd_send(DAC_OPEN_DEFAULT, current_rate , 4);
	snd_printd("\n");
	return 0;
}

/*
 * Do some sanity check, turn clock off and then turn codec audio off
 */
static int egold_clock_off(void)
{
	cn_sx1snd_send(DAC_CLOSE, 0 , 0);
	cn_sx1snd_send(DAC_SETAUDIODEVICE, SX1_DEVICE_PHONE, 0);
	omap_set_gpio_dataout(OSC_EN, 0);
	snd_printd("\n");
	return 0;
}

static int egold_get_default_samplerate(void)
{
	snd_printd("\n");
	return DEFAULT_SAMPLE_RATE;
}

static int __init snd_omap_alsa_egold_probe(struct platform_device *pdev)
{
	int ret;
	struct omap_alsa_codec_config *codec_cfg;

	codec_cfg = pdev->dev.platform_data;
	if (!codec_cfg)
		return -ENODEV;

	codec_cfg->hw_constraints_rates	= &egold_hw_constraints_rates;
	codec_cfg->snd_omap_alsa_playback= &egold_snd_omap_alsa_playback;
	codec_cfg->snd_omap_alsa_capture  = &egold_snd_omap_alsa_capture;
	codec_cfg->codec_configure_dev	= egold_configure;
	codec_cfg->codec_set_samplerate	= egold_set_samplerate;
	codec_cfg->codec_clock_setup	= egold_clock_setup;
	codec_cfg->codec_clock_on	= egold_clock_on;
	codec_cfg->codec_clock_off	= egold_clock_off;
	codec_cfg->get_default_samplerate = egold_get_default_samplerate;
	ret = snd_omap_alsa_post_probe(pdev, codec_cfg);

	snd_printd("\n");
	return ret;
}

static struct platform_driver omap_alsa_driver = {
	.probe		= snd_omap_alsa_egold_probe,
	.remove		= snd_omap_alsa_remove,
	.suspend	= snd_omap_alsa_suspend,
	.resume		= snd_omap_alsa_resume,
	.driver	= {
		.name =	"omap_alsa_mcbsp",
	},
};

static int __init omap_alsa_egold_init(void)
{
	int retval;

	retval = cn_add_callback(&cn_sx1snd_id, cn_sx1snd_name, cn_sx1snd_callback);
	if (retval)
		printk(KERN_WARNING "cn_sx1snd failed to register\n");
	return platform_driver_register(&omap_alsa_driver);
}

static void __exit omap_alsa_egold_exit(void)
{
	cn_del_callback(&cn_sx1snd_id);
	platform_driver_unregister(&omap_alsa_driver);
}

module_init(omap_alsa_egold_init);
module_exit(omap_alsa_egold_exit);
