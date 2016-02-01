/*
 * ALSA SoC generic TDM codec driver
 *
 * Author:      Matthijs van Duin <matthijsvanduin@gmail.com>
 * Copyright:   (C) 2016  Dutch & Dutch
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * TODO Allow customization via device tree.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <linux/of.h>

#define DRV_NAME "tdm-audio"

/* As far as I can tell the LE/3LE/BE/3BE suffix merely indicates how the data
 * was represented in memory, so why would the codec care?  On the other hand,
 * how do you indicate the bit-endianness on wire?  */

#define TDM_FORMATS	(SNDRV_PCM_FMTBIT_S8 | \
			SNDRV_PCM_FMTBIT_U8 | \
			SNDRV_PCM_FMTBIT_S16_LE | \
			SNDRV_PCM_FMTBIT_U16_LE | \
			SNDRV_PCM_FMTBIT_S20_3LE | \
			SNDRV_PCM_FMTBIT_U20_3LE | \
			SNDRV_PCM_FMTBIT_S24_3LE | \
			SNDRV_PCM_FMTBIT_U24_3LE | \
			SNDRV_PCM_FMTBIT_S24_LE | \
			SNDRV_PCM_FMTBIT_U24_LE | \
			SNDRV_PCM_FMTBIT_S32_LE | \
			SNDRV_PCM_FMTBIT_U32_LE)

static const struct snd_soc_dapm_widget tdm_audio_widgets[] = {
	SND_SOC_DAPM_OUTPUT("Sink"),
	SND_SOC_DAPM_INPUT("Source"),
};

static const struct snd_soc_dapm_route tdm_audio_routes[] = {
	{ "Sink", NULL, "Playback" },
	{ "Capture", NULL, "Source" },
};

static struct snd_soc_codec_driver soc_codec_tdm_audio = {
	.dapm_widgets = tdm_audio_widgets,
	.num_dapm_widgets = ARRAY_SIZE(tdm_audio_widgets),
	.dapm_routes = tdm_audio_routes,
	.num_dapm_routes = ARRAY_SIZE(tdm_audio_routes),
};

static struct snd_soc_dai_driver tdm_audio_dai = {
	.name		= "tdm_audio",
	.playback 	= {
		.stream_name	= "Playback",
		.channels_min	= 1,
		.channels_max	= 16,
		.rates		= SNDRV_PCM_RATE_CONTINUOUS,
		.formats	= TDM_FORMATS,
	},
	.capture 	= {
		.stream_name	= "Capture",
		.channels_min	= 1,
		.channels_max	= 16,
		.rates		= SNDRV_PCM_RATE_CONTINUOUS,
		.formats	= TDM_FORMATS,
	},
};

static int tdm_audio_probe(struct platform_device *pdev)
{
	return snd_soc_register_codec(&pdev->dev, &soc_codec_tdm_audio,
			&tdm_audio_dai, 1);
}

static int tdm_audio_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id tdm_audio_dt_ids[] = {
	{ .compatible = "linux,tdm-audio", },
	{ }
};
MODULE_DEVICE_TABLE(of, tdm_audio_dt_ids);
#endif

static struct platform_driver tdm_audio_driver = {
	.probe		= tdm_audio_probe,
	.remove		= tdm_audio_remove,
	.driver		= {
		.name	= DRV_NAME,
		.of_match_table = of_match_ptr(tdm_audio_dt_ids),
	},
};

module_platform_driver(tdm_audio_driver);

MODULE_AUTHOR("Matthijs van Duin <matthijs@dutchdutch.com>");
MODULE_DESCRIPTION("Generic TDM codec driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
