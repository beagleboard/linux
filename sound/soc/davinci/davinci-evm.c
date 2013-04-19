/*
 * ASoC driver for TI DAVINCI EVM platform
 *
 * Author:      Vladimir Barinov, <vbarinov@embeddedalley.com>
 * Copyright:   (C) 2007 MontaVista Software, Inc., <source@mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/platform_data/edma.h>
#include <linux/i2c.h>
#include <linux/of_platform.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>

#include <asm/dma.h>
#include <asm/mach-types.h>

#include <linux/edma.h>

#include "davinci-pcm.h"
#include "davinci-i2s.h"
#include "davinci-mcasp.h"

#include <linux/of_gpio.h>


#define AUDIO_FORMAT (SND_SOC_DAIFMT_DSP_B | \
		SND_SOC_DAIFMT_CBM_CFM | SND_SOC_DAIFMT_IB_NF)
static int evm_hw_params(struct snd_pcm_substream *substream,
			 struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *soc_card = codec->card;
	struct device_node *np = soc_card->dev->of_node;
	int ret = 0;
	unsigned sysclk;

	if (np) {
		ret = of_property_read_u32(np, "ti,codec-clock-rate", &sysclk);
		if (ret < 0)
			return ret;
	} else {
		/* ASP1 on DM355 EVM is clocked by an external oscillator */
		if (machine_is_davinci_dm355_evm() ||
			machine_is_davinci_dm6467_evm() ||
			machine_is_davinci_dm365_evm())
			sysclk = 27000000;

		/*
		 * ASP0 in DM6446 EVM is clocked by U55, as configured by
		 * board-dm644x-evm.c using GPIOs from U18.  There are six
		 * options; here we "know" we use a 48 KHz sample rate.
		 */
		else if (machine_is_davinci_evm())
			sysclk = 12288000;

		else if (machine_is_davinci_da830_evm() ||
					machine_is_davinci_da850_evm())
			sysclk = 24576000;

		else
			return -EINVAL;
	}

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, AUDIO_FORMAT);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, AUDIO_FORMAT);
	if (ret < 0)
		return ret;

	/* set the codec system clock */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, sysclk, SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;

	/* set the CPU system clock */
	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, sysclk, SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;

	return 0;
}

static int evm_spdif_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;

	/* set cpu DAI configuration */
	return snd_soc_dai_set_fmt(cpu_dai, AUDIO_FORMAT);
}

static struct snd_soc_ops evm_ops = {
	.hw_params = evm_hw_params,
};

static struct snd_soc_ops evm_spdif_ops = {
	.hw_params = evm_spdif_hw_params,
};

/* davinci-evm machine dapm widgets */
static const struct snd_soc_dapm_widget aic3x_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_LINE("Line Out", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_LINE("Line In", NULL),
};

/* davinci-evm machine audio_mapnections to the codec pins */
static const struct snd_soc_dapm_route audio_map[] = {
	/* Headphone connected to HPLOUT, HPROUT */
	{"Headphone Jack", NULL, "HPLOUT"},
	{"Headphone Jack", NULL, "HPROUT"},

	/* Line Out connected to LLOUT, RLOUT */
	{"Line Out", NULL, "LLOUT"},
	{"Line Out", NULL, "RLOUT"},

	/* Mic connected to (MIC3L | MIC3R) */
	{"MIC3L", NULL, "Mic Bias 2V"},
	{"MIC3R", NULL, "Mic Bias 2V"},
	{"Mic Bias 2V", NULL, "Mic Jack"},

	/* Line In connected to (LINE1L | LINE2L), (LINE1R | LINE2R) */
	{"LINE1L", NULL, "Line In"},
	{"LINE2L", NULL, "Line In"},
	{"LINE1R", NULL, "Line In"},
	{"LINE2R", NULL, "Line In"},
};

/* Logic for a aic3x as connected on a davinci-evm */
static int evm_aic3x_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct device_node *np = codec->card->dev->of_node;
	int ret;

	/* Add davinci-evm specific widgets */
	snd_soc_dapm_new_controls(dapm, aic3x_dapm_widgets,
				  ARRAY_SIZE(aic3x_dapm_widgets));

	if (np) {
		ret = snd_soc_of_parse_audio_routing(codec->card,
							"ti,audio-routing");
		if (ret)
			return ret;
	} else {
		/* Set up davinci-evm specific audio path audio_map */
		snd_soc_dapm_add_routes(dapm, audio_map, ARRAY_SIZE(audio_map));
	}

	/* not connected */
	snd_soc_dapm_disable_pin(dapm, "MONO_LOUT");
	snd_soc_dapm_disable_pin(dapm, "HPLCOM");
	snd_soc_dapm_disable_pin(dapm, "HPRCOM");

	/* always connected */
	snd_soc_dapm_enable_pin(dapm, "Headphone Jack");
	snd_soc_dapm_enable_pin(dapm, "Line Out");
	snd_soc_dapm_enable_pin(dapm, "Mic Jack");
	snd_soc_dapm_enable_pin(dapm, "Line In");

	return 0;
}

/* davinci-evm digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link dm6446_evm_dai = {
	.name = "TLV320AIC3X",
	.stream_name = "AIC3X",
	.cpu_dai_name = "davinci-mcbsp",
	.codec_dai_name = "tlv320aic3x-hifi",
	.codec_name = "tlv320aic3x-codec.1-001b",
	.platform_name = "davinci-mcbsp",
	.init = evm_aic3x_init,
	.ops = &evm_ops,
};

static struct snd_soc_dai_link dm355_evm_dai = {
	.name = "TLV320AIC3X",
	.stream_name = "AIC3X",
	.cpu_dai_name = "davinci-mcbsp.1",
	.codec_dai_name = "tlv320aic3x-hifi",
	.codec_name = "tlv320aic3x-codec.1-001b",
	.platform_name = "davinci-mcbsp.1",
	.init = evm_aic3x_init,
	.ops = &evm_ops,
};

static struct snd_soc_dai_link dm365_evm_dai = {
#ifdef CONFIG_SND_DM365_AIC3X_CODEC
	.name = "TLV320AIC3X",
	.stream_name = "AIC3X",
	.cpu_dai_name = "davinci-mcbsp",
	.codec_dai_name = "tlv320aic3x-hifi",
	.init = evm_aic3x_init,
	.codec_name = "tlv320aic3x-codec.1-0018",
	.ops = &evm_ops,
	.platform_name = "davinci-mcbsp",
#elif defined(CONFIG_SND_DM365_VOICE_CODEC)
	.name = "Voice Codec - CQ93VC",
	.stream_name = "CQ93",
	.cpu_dai_name = "davinci-vcif",
	.codec_dai_name = "cq93vc-hifi",
	.codec_name = "cq93vc-codec",
	.platform_name = "davinci-vcif",
#endif
};

static struct snd_soc_dai_link dm6467_evm_dai[] = {
	{
		.name = "TLV320AIC3X",
		.stream_name = "AIC3X",
		.cpu_dai_name= "davinci-mcasp.0",
		.codec_dai_name = "tlv320aic3x-hifi",
		.platform_name = "davinci-mcasp.0",
		.codec_name = "tlv320aic3x-codec.0-001a",
		.init = evm_aic3x_init,
		.ops = &evm_ops,
	},
	{
		.name = "McASP",
		.stream_name = "spdif",
		.cpu_dai_name= "davinci-mcasp.1",
		.codec_dai_name = "dit-hifi",
		.codec_name = "spdif_dit",
		.platform_name = "davinci-mcasp.1",
		.ops = &evm_spdif_ops,
	},
};

static struct snd_soc_dai_link da830_evm_dai = {
	.name = "TLV320AIC3X",
	.stream_name = "AIC3X",
	.cpu_dai_name = "davinci-mcasp.1",
	.codec_dai_name = "tlv320aic3x-hifi",
	.codec_name = "tlv320aic3x-codec.1-0018",
	.platform_name = "davinci-mcasp.1",
	.init = evm_aic3x_init,
	.ops = &evm_ops,
};

static struct snd_soc_dai_link da850_evm_dai = {
	.name = "TLV320AIC3X",
	.stream_name = "AIC3X",
	.cpu_dai_name= "davinci-mcasp.0",
	.codec_dai_name = "tlv320aic3x-hifi",
	.codec_name = "tlv320aic3x-codec.1-0018",
	.platform_name = "davinci-mcasp.0",
	.init = evm_aic3x_init,
	.ops = &evm_ops,
};

/* davinci dm6446 evm audio machine driver */
static struct snd_soc_card dm6446_snd_soc_card_evm = {
	.name = "DaVinci DM6446 EVM",
	.owner = THIS_MODULE,
	.dai_link = &dm6446_evm_dai,
	.num_links = 1,
};

/* davinci dm355 evm audio machine driver */
static struct snd_soc_card dm355_snd_soc_card_evm = {
	.name = "DaVinci DM355 EVM",
	.owner = THIS_MODULE,
	.dai_link = &dm355_evm_dai,
	.num_links = 1,
};

/* davinci dm365 evm audio machine driver */
static struct snd_soc_card dm365_snd_soc_card_evm = {
	.name = "DaVinci DM365 EVM",
	.owner = THIS_MODULE,
	.dai_link = &dm365_evm_dai,
	.num_links = 1,
};

/* davinci dm6467 evm audio machine driver */
static struct snd_soc_card dm6467_snd_soc_card_evm = {
	.name = "DaVinci DM6467 EVM",
	.owner = THIS_MODULE,
	.dai_link = dm6467_evm_dai,
	.num_links = ARRAY_SIZE(dm6467_evm_dai),
};

static struct snd_soc_card da830_snd_soc_card = {
	.name = "DA830/OMAP-L137 EVM",
	.owner = THIS_MODULE,
	.dai_link = &da830_evm_dai,
	.num_links = 1,
};

static struct snd_soc_card da850_snd_soc_card = {
	.name = "DA850/OMAP-L138 EVM",
	.owner = THIS_MODULE,
	.dai_link = &da850_evm_dai,
	.num_links = 1,
};


#if defined(CONFIG_OF)

enum {
	MACHINE_VERSION_1 = 0,	/* DM365 with Voice Codec */
	MACHINE_VERSION_2,	/* DM365/DA8xx/OMAPL1x/AM33xx */
	MACHINE_VERSION_3,	/* AM33xx BeagleBone Black */
};

static const struct of_device_id davinci_evm_dt_ids[] = {
	{
		.compatible = "ti,dm365-voice-codec-audio",
		.data = (void *)MACHINE_VERSION_1,
	},
	{
		.compatible = "ti,da830-evm-audio",
		.data = (void *)MACHINE_VERSION_2,
	},
	{
		.compatible = "ti,am33xx-beaglebone-black",
		.data = (void *)MACHINE_VERSION_3,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, davinci_mcasp_dt_ids);

/*
 * This struct is just used as place holder. It will be filled with
 * data from dt node
 */
static struct snd_soc_dai_link evm_dai = {
	.name		= "TLV320AIC3X",
	.stream_name	= "AIC3X",
	.codec_dai_name	= "tlv320aic3x-hifi",
};

/* davinci evm audio machine driver */
static struct snd_soc_card evm_soc_card = {
	.owner = THIS_MODULE,
	.dai_link = &evm_dai,
	.num_links = 1,
};

static int davinci_evm_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *match =
		of_match_device(of_match_ptr(davinci_evm_dt_ids), &pdev->dev);
	u32 machine_ver, clk_gpio;
	int ret = 0;

	machine_ver = (u32)match->data;
	switch (machine_ver) {
	case MACHINE_VERSION_1:
		evm_dai.name		= "Voice Codec - CQ93VC";
		evm_dai.stream_name	= "CQ93";
		evm_dai.codec_dai_name	= "cq93vc-hifi";
		break;
	case MACHINE_VERSION_2:
		evm_dai.ops = &evm_ops;
		evm_dai.init = evm_aic3x_init;
		break;
	case MACHINE_VERSION_3:
		evm_dai.name		= "NXP TDA HDMI Chip";
		evm_dai.stream_name	= "HDMI";
		evm_dai.codec_dai_name	= "nxp-hdmi-hifi";

		/*
		 * Move GPIO handling out of the probe, if probe gets
		 * deferred, the gpio will have been claimed on previous
		 * probe and will fail on the second and susequent probes
		 */
		clk_gpio = of_get_named_gpio(np, "mcasp_clock_enable", 0);
		if (clk_gpio < 0) {
		  dev_err(&pdev->dev, "failed to find mcasp_clock enable GPIO!\n");
		  return -EINVAL;
		}
		ret = gpio_request_one(clk_gpio, GPIOF_OUT_INIT_HIGH,
				       "McASP Clock Enable Pin");
		if (ret < 0) {
		  dev_err(&pdev->dev, "Failed to claim McASP Clock Enable pin\n");
		  return -EINVAL;
		}
		gpio_set_value(clk_gpio, 1);
		evm_dai.dai_fmt = SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_DSP_B | SND_SOC_DAIFMT_IB_NF;
		break;

	}


	evm_dai.codec_of_node = of_parse_phandle(np, "ti,audio-codec", 0);
	if (!evm_dai.codec_of_node)
		return -EINVAL;

	evm_dai.cpu_of_node = of_parse_phandle(np,
						"ti,mcasp-controller", 0);
	if (!evm_dai.cpu_of_node)
		return -EINVAL;

	evm_dai.platform_of_node = evm_dai.cpu_of_node;

	evm_soc_card.dev = &pdev->dev;
	ret = snd_soc_of_parse_card_name(&evm_soc_card, "ti,model");
	if (ret)
		return ret;

	ret = snd_soc_register_card(&evm_soc_card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
	}
	return ret;
}

static int davinci_evm_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);

	return 0;
}

static struct platform_driver davinci_evm_driver = {
	.probe		= davinci_evm_probe,
	.remove		= davinci_evm_remove,
	.driver		= {
		.name	= "davinci_evm",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(davinci_evm_dt_ids),
	},
};
#endif

static struct platform_device *evm_snd_device;

static int __init evm_init(void)
{
	struct snd_soc_card *evm_snd_dev_data;
	int index;
	int ret;

#if defined(CONFIG_OF)
	/*
	 * If dtb is there, the devices will be created dynamically.
	 * Only register platfrom driver structure.
	 */
	if (of_have_populated_dt()) {
	  return platform_driver_register(&davinci_evm_driver);
	}
#endif

	if (machine_is_davinci_evm()) {
		evm_snd_dev_data = &dm6446_snd_soc_card_evm;
		index = 0;
	} else if (machine_is_davinci_dm355_evm()) {
		evm_snd_dev_data = &dm355_snd_soc_card_evm;
		index = 1;
	} else if (machine_is_davinci_dm365_evm()) {
		evm_snd_dev_data = &dm365_snd_soc_card_evm;
		index = 0;
	} else if (machine_is_davinci_dm6467_evm()) {
		evm_snd_dev_data = &dm6467_snd_soc_card_evm;
		index = 0;
	} else if (machine_is_davinci_da830_evm()) {
		evm_snd_dev_data = &da830_snd_soc_card;
		index = 1;
	} else if (machine_is_davinci_da850_evm()) {
		evm_snd_dev_data = &da850_snd_soc_card;
		index = 0;
	} else {
		return -EINVAL;
	}

	evm_snd_device = platform_device_alloc("soc-audio", index);
	if (!evm_snd_device)
		return -ENOMEM;

	platform_set_drvdata(evm_snd_device, evm_snd_dev_data);
	ret = platform_device_add(evm_snd_device);
	if (ret)
		platform_device_put(evm_snd_device);

	return ret;
}

static void __exit evm_exit(void)
{
#if defined(CONFIG_OF)
	if (of_have_populated_dt()) {
		platform_driver_unregister(&davinci_evm_driver);
		return;
	}
#endif

	platform_device_unregister(evm_snd_device);
}

module_init(evm_init);
module_exit(evm_exit);

MODULE_AUTHOR("Vladimir Barinov");
MODULE_DESCRIPTION("TI DAVINCI EVM ASoC driver");
MODULE_LICENSE("GPL");
