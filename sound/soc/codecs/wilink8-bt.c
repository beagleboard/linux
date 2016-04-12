/*
* ALSA SoC PCM5102a driver
*
* Author: 	 Josh Elliott, <jelliott@ti.com>
* Copyright:	 Copyright:   (C) 2014	Texas Instruments
*
* Based on sound/soc/codecs/spdif_transmitter.c by Steve Chen
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*/

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <linux/of.h>

#define DRV_NAME "wilink8_bt"

#define RATES		(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000)
#define FORMATS		SNDRV_PCM_FMTBIT_S16_LE


static struct snd_soc_codec_driver soc_codec_wilink8_bt = {
};

static struct snd_soc_dai_driver wilink8_bt_dai = {
	.name		= "wilink8_bt-hifi",
	.playback = {
		.stream_name	= "Playback",
		.channels_min	= 1,
		.channels_max	= 2,
		.rates		= RATES,
		.formats	= FORMATS,
	},
	.capture = {
		.stream_name	= "Capture",
		.channels_min	= 1,
		.channels_max	= 2,
		.rates		= RATES,
		.formats	= FORMATS,
	},
};

static int wilink8_bt_probe(struct platform_device *pdev)
{
	return snd_soc_register_codec(&pdev->dev, &soc_codec_wilink8_bt,
			&wilink8_bt_dai, 1);
}

static int wilink8_bt_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id wilink8_bt_dt_ids[] = {
	{ .compatible = "ti,wilink8_bt", },
	{ }
};
MODULE_DEVICE_TABLE(of, wilink8_bt_dt_ids);
#endif

static struct platform_driver wilink8_bt_driver = {
	.probe		= wilink8_bt_probe,
	.remove		= wilink8_bt_remove,
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(wilink8_bt_dt_ids),
	},
};

module_platform_driver(wilink8_bt_driver);

MODULE_AUTHOR("Baozhu <zuobaozhu@gmail.com>");
MODULE_DESCRIPTION("WILINK8_BT dummy codec driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
