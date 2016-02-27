/*
 * ASoC machine driver for CTAG face2|4 Audio Card
 *
 * Author:	Henrik Langer <henni19790@googlemail.com>
 *        	based on
 * 				ASoC driver for TI DAVINCI EVM platform by
 *				Vladimir Barinov <vbarinov@embeddedalley.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <asm/dma.h>
#include <asm/mach-types.h>

#include "../codecs/ad193x.h"

struct snd_soc_card_drvdata_davinci {
	struct clk *mclk;
	unsigned sysclk;
	unsigned codec_clock;
};

/*
	Define Dynamic Audio Power Management (DAPM) widgets
*/
static const struct snd_soc_dapm_widget ad193x_dapm_widgets[] = {
	SND_SOC_DAPM_LINE("Line Out", NULL),
	SND_SOC_DAPM_LINE("Line In", NULL),
};

static const struct snd_soc_dapm_route audio_map[] = {
	{"Line Out", NULL, "DAC1OUT"},
	{"Line Out", NULL, "DAC2OUT"},
	{"Line Out", NULL, "DAC3OUT"},
	{"Line Out", NULL, "DAC4OUT"},
	{"ADC1IN", NULL, "Line In"},
	{"ADC2IN", NULL, "Line In"},
};

/*
	Sound card init
*/
static int snd_davinci_audiocard_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	struct device_node *np = card->dev->of_node;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int ret;
	unsigned int tdm_mask = 0x00;
	u32 tdm_slots;

	/*
		Add davinci-evm specific DAPM widgets
	*/
	snd_soc_dapm_new_controls(&card->dapm, ad193x_dapm_widgets,
				  ARRAY_SIZE(ad193x_dapm_widgets));

	/*
		Get audio routing from device tree or use built-in routing
	*/
	if (np) {
		dev_dbg(card->dev, "Using configuration from dt overlay.\n");
		ret = snd_soc_of_parse_audio_routing(card, "audio-routing");
		if (ret)
			return ret;
		ret = of_property_read_u32(np, "audiocard-tdm-slots", &tdm_slots);
		if (tdm_slots > 8 || tdm_slots < 2 || ret){
			dev_dbg(card->dev, "Couldn't get device tree property for tdm slots. Using default (=2).\n");
			tdm_slots = 2;
			tdm_mask = 0x03; // lsb for slot 0, ...
		} else {
			tdm_mask = 0xFF;
			tdm_mask = tdm_mask >> (8 - tdm_slots);
		}
	} else {
		dev_dbg(card->dev, "Use builtin audio routing.\n");
		/* Set up davinci specific audio path audio_map */
		snd_soc_dapm_add_routes(&card->dapm, audio_map,
					ARRAY_SIZE(audio_map));
	}

	/*
		Configure TDM mode of CPU and audio codec interface
		(ad193x codec driver ignores TX / RX mask and width)
	*/
	ret = snd_soc_dai_set_tdm_slot(codec_dai, tdm_mask, tdm_mask, tdm_slots, 32);
	if (ret < 0){
		dev_err(codec_dai->dev, "Unable to set AD193x TDM slots.\n");
		return ret;
	}
	ret = snd_soc_dai_set_tdm_slot(cpu_dai, tdm_mask, tdm_mask, tdm_slots, 32);
	if (ret < 0){
		dev_err(codec_dai->dev, "Unable to set McASP TDM slots.\n");
		return ret;
	}

	return 0;
}

/*
	Set hw parameters
*/
static int snd_davinci_audiocard_hw_params(struct snd_pcm_substream *substream,
				       struct snd_pcm_hw_params *params)
{
	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *soc_card = rtd->card;
	unsigned cpu_clock = ((struct snd_soc_card_drvdata_davinci *)
		snd_soc_card_get_drvdata(soc_card))->sysclk;
	unsigned codec_clock = ((struct snd_soc_card_drvdata_davinci *)
		snd_soc_card_get_drvdata(soc_card))->codec_clock;

	/*
		Set master clock of CPU and audio codec interface
		(ad193x codec driver ignores clock ID and direction)
	*/
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, codec_clock, SND_SOC_CLOCK_IN);
	if (ret < 0){
		dev_err(codec->dev, "Unable to set AD193x system clock: %d.\n", ret);
		return ret;
	}
	dev_dbg(cpu_dai->dev, "Set codec DAI clock rate to %d.\n", codec_clock);

	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, cpu_clock, SND_SOC_CLOCK_OUT);
	if (ret < 0){
		dev_err(cpu_dai->dev, "Unable to set cpu dai sysclk: %d.\n", ret);
		return ret;
	}
	dev_dbg(cpu_dai->dev, "Set CPU DAI clock rate to %d.\n", cpu_clock);

	return 0;
}

/*
	Startup
*/
static int snd_davinci_audiocard_startup(struct snd_pcm_substream *substream) {
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *soc_card = rtd->card;
	struct snd_soc_card_drvdata_davinci *drvdata = snd_soc_card_get_drvdata(soc_card);

	if (drvdata->mclk)
		return clk_prepare_enable(drvdata->mclk);

	return 0;
}

/*
	Shutdown
*/
static void snd_davinci_audiocard_shutdown(struct snd_pcm_substream *substream) {
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *soc_card = rtd->card;
	struct snd_soc_card_drvdata_davinci *drvdata = snd_soc_card_get_drvdata(soc_card);

	if (drvdata->mclk)
		clk_disable_unprepare(drvdata->mclk);
}

/*
	Machine stream operations
*/
static struct snd_soc_ops snd_davinci_audiocard_ops = {
	.hw_params = snd_davinci_audiocard_hw_params,
	.startup = snd_davinci_audiocard_startup,
	.shutdown = snd_davinci_audiocard_shutdown,
};

/*
	Interface setup
	(rxclk and txclk are configured asynchronous in i2s mode (see mcasp platform driver))
*/
#define AUDIOCARD_AD193X_DAIFMT ( SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_IF | SND_SOC_DAIFMT_CBM_CFM )
/*
	Struct ist just a placeholder. Device tree will add cpu and codec nodes here
*/
static struct snd_soc_dai_link snd_davinci_audiocard_dai = {
	.name = "CTAG face-2-4",
	.stream_name = "TDM",
	.codec_dai_name ="ad193x-hifi",
	.dai_fmt = AUDIOCARD_AD193X_DAIFMT,
	.ops = &snd_davinci_audiocard_ops,
	.init = snd_davinci_audiocard_init,
};

/*
	Export device tree identifiers
*/
static const struct of_device_id snd_davinci_audiocard_dt_ids[] = {
	{
		.compatible = "ctag,face-2-4",
		.data = &snd_davinci_audiocard_dai,
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, snd_davinci_audiocard_dt_ids);

/*
	Audio machine driver
*/
static struct snd_soc_card snd_davinci_audiocard = {
	.owner = THIS_MODULE,
	.num_links = 1,
};

/*
	Sound card probe
*/
static int snd_davinci_audiocard_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *match =
		of_match_device(of_match_ptr(snd_davinci_audiocard_dt_ids), &pdev->dev);
	struct snd_soc_dai_link *dai = (struct snd_soc_dai_link *) match->data;
	struct snd_soc_card_drvdata_davinci *drvdata = NULL;
	struct clk *mclk;
	int ret = 0;

	snd_davinci_audiocard.dai_link = dai;

	/*
		Parse device tree properties and nodes of Bone Cape for AD1938 AudioCard
	*/
	dai->codec_of_node = of_parse_phandle(np, "audio-codec", 0);
	if (!dai->codec_of_node)
		return -EINVAL;

	dai->cpu_of_node = of_parse_phandle(np, "mcasp-controller", 0);
	if (!dai->cpu_of_node)
		return -EINVAL;

	dai->platform_of_node = dai->cpu_of_node;

	snd_davinci_audiocard.dev = &pdev->dev;
	ret = snd_soc_of_parse_card_name(&snd_davinci_audiocard, "model");
	if (ret)
		return ret;

	mclk = devm_clk_get(&pdev->dev, "mclk");
	if (PTR_ERR(mclk) == -EPROBE_DEFER) {
		return -EPROBE_DEFER;
	} else if (IS_ERR(mclk)) {
		dev_dbg(&pdev->dev, "mclk not found.\n");
		mclk = NULL;
	}

	drvdata = devm_kzalloc(&pdev->dev, sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	drvdata->mclk = mclk;

	ret = of_property_read_u32(np, "codec-clock-rate", &drvdata->codec_clock);
	if (ret < 0){
		dev_err(&pdev->dev, "No codec clock rate defined.\n");
		return -EINVAL;
	}

	/*
		Configure internal 24,576 MHz oscillator as master clock for McASP
	*/
	ret = of_property_read_u32(np, "cpu-clock-rate", &drvdata->sysclk);
	if (ret < 0) {
		if (!drvdata->mclk) {
			dev_err(&pdev->dev,
				"No clock or clock rate defined.\n");
			return -EINVAL;
		}
		drvdata->sysclk = clk_get_rate(drvdata->mclk);
	} else if (drvdata->mclk) {
		unsigned int requestd_rate = drvdata->sysclk;
		clk_set_rate(drvdata->mclk, drvdata->sysclk);
		drvdata->sysclk = clk_get_rate(drvdata->mclk);
		if (drvdata->sysclk != requestd_rate)
			dev_warn(&pdev->dev,
				 "Could not get requested rate %u using %u.\n",
				 requestd_rate, drvdata->sysclk);
	}

	/*
		Register AD1938 AudioCard
	*/
	snd_soc_card_set_drvdata(&snd_davinci_audiocard, drvdata);
	ret = devm_snd_soc_register_card(&pdev->dev, &snd_davinci_audiocard);
	if (ret)
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);

	return ret;
}

/* Sound card disconnect */
static int snd_davinci_audiocard_remove(struct platform_device *pdev)
{
	return snd_soc_unregister_card(&snd_davinci_audiocard);
}

/* Sound card platform driver */
static struct platform_driver snd_davinci_audiocard_driver = {
	.probe          = snd_davinci_audiocard_probe,
	.driver = {
		.name   = "snd_ctag_face_2_4",
		.pm = &snd_soc_pm_ops,
		.of_match_table = of_match_ptr(snd_davinci_audiocard_dt_ids),
	},
	.remove = snd_davinci_audiocard_remove,
};

module_platform_driver(snd_davinci_audiocard_driver);

/* Module information */
MODULE_AUTHOR("Henrik Langer");
MODULE_DESCRIPTION("ALSA SoC AD193X AudioCard driver");
MODULE_LICENSE("GPL");
