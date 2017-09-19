/*
 * ASoC Driver for JustBoom DAC Sound Card
 *
 * Author:	Milan Neskovic
 *		Copyright 2017
 *		based on code by Vladimir Barinov, <vbarinov@embeddedalley.com>
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
#include <linux/platform_device.h>

#include <linux/of_platform.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>

#include "../codecs/pcm512x.h"

#define JUSTBOOM_MCLK_SEL_NOCLOCK 			0
#define JUSTBOOM_MCLK_SEL_CLK44K1_RATE 		1
#define JUSTBOOM_MCLK_SEL_CLK48K_RATE 		2

#define JUSTBOOM_PCM512x_GPIO_MASK(n) 		(0x01<<(n-1))

struct justboom_dac_drvdata {
	struct clk *mclk_48k;
	struct clk *mclk_44k1;
	unsigned sysclk;
	unsigned char mclk_select;
	int clk_44k1_en_gpio;
	int led_gpio;
};

static bool digital_gain_0db_limit = true;

static int justboom_dac_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *soc_card = rtd->card;
	struct justboom_dac_drvdata *drvdata =
		snd_soc_card_get_drvdata(soc_card);
	int ret;
	
	unsigned char clk_44k1_en_gpio_mask = 0;
	unsigned char led_gpio_mask = 
			JUSTBOOM_PCM512x_GPIO_MASK(drvdata->led_gpio);
	
	if (drvdata->mclk_44k1) 
		clk_44k1_en_gpio_mask = 
				JUSTBOOM_PCM512x_GPIO_MASK(drvdata->clk_44k1_en_gpio);
	/* initialize to low */
	snd_soc_update_bits(codec, PCM512x_GPIO_CONTROL_1, 
			clk_44k1_en_gpio_mask | led_gpio_mask, 0x00); 
	/* enable led gpio, clock gpio */
	snd_soc_update_bits(codec, PCM512x_GPIO_EN, 
			clk_44k1_en_gpio_mask | led_gpio_mask, 
			clk_44k1_en_gpio_mask | led_gpio_mask); 
	/* set clock enable gpio signal to register output */
	snd_soc_update_bits(codec, 
			PCM512x_GPIO_OUTPUT_1 + drvdata->clk_44k1_en_gpio - 1, 
			0x1f, 0x02); 
	/* set led gpio signal to register output */
	snd_soc_update_bits(codec, 
			PCM512x_GPIO_OUTPUT_1 + drvdata->led_gpio - 1,
			0x1f, 0x02);
			   
	drvdata->mclk_select = JUSTBOOM_MCLK_SEL_NOCLOCK;

	if (digital_gain_0db_limit)
	{
		ret = snd_soc_limit_volume(soc_card, "Digital Playback Volume", 207);
		if (ret < 0)
			dev_warn(soc_card->dev, "Failed to set volume limit: %d\n", ret);
	}

	return 0;
}


static int justboom_dac_hw_params(struct snd_pcm_substream *substream,
				       struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *soc_card = rtd->card;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct justboom_dac_drvdata *drvdata =
		snd_soc_card_get_drvdata(soc_card);
	unsigned sysclk = ((struct justboom_dac_drvdata *)
			   snd_soc_card_get_drvdata(soc_card))->sysclk;
	struct snd_mask *fmt = constrs_mask(&runtime->hw_constraints,
					    SNDRV_PCM_HW_PARAM_FORMAT);
	int bclk_div, bclk_fs_div;
	unsigned int sample_bits =
		snd_pcm_format_physical_width(params_format(params));
	int rate = params_rate(params);
	int ret;

	snd_mask_none(fmt);
	switch (sample_bits) {
		case 16: 
			snd_mask_set(fmt, SNDRV_PCM_FORMAT_S16_LE);
			break;
		case 24:
			snd_mask_set(fmt, SNDRV_PCM_FORMAT_S24_LE);
			break;
		default:
			snd_mask_set(fmt, SNDRV_PCM_FORMAT_S32_LE);
			break;
	}
	
	if ((rate % 8000) == 0) {
		drvdata->mclk_select = JUSTBOOM_MCLK_SEL_CLK48K_RATE;
		sysclk = clk_get_rate(drvdata->mclk_48k);
	} else if ((rate % 11025) == 0) {
		drvdata->mclk_select = JUSTBOOM_MCLK_SEL_CLK44K1_RATE;
		sysclk = clk_get_rate(drvdata->mclk_44k1);
	} else {
		drvdata->mclk_select = JUSTBOOM_MCLK_SEL_NOCLOCK;
		sysclk = 0;
		dev_err(soc_card->dev, "Rate %d is not supported\n", rate);	
		return -EINVAL;
	}
	
	bclk_fs_div = 64;
	bclk_div = sysclk / (rate * bclk_fs_div);
	
	ret = snd_soc_dai_set_clkdiv(cpu_dai, 1, bclk_div); /* set bclk */
	if (ret < 0)
		return ret;
	
	/* set BCLK/FS ratio */
	ret = snd_soc_dai_set_clkdiv(cpu_dai, 2, bclk_fs_div);
	if (ret < 0)
		return ret;
		
	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, sysclk, SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;
		
	/* enable system clock */
	if (drvdata->mclk_select == JUSTBOOM_MCLK_SEL_CLK48K_RATE) {
		if (drvdata->mclk_48k) 
			ret = clk_prepare_enable(drvdata->mclk_48k);
	} else if (drvdata->mclk_select == JUSTBOOM_MCLK_SEL_CLK44K1_RATE) {
		/* set enable gpio high to enable 44100 rate oscillator */
		if (drvdata->mclk_44k1) {
			unsigned char clk_44k1_en_gpio_mask = 
					JUSTBOOM_PCM512x_GPIO_MASK(drvdata->clk_44k1_en_gpio);
			snd_soc_update_bits(codec, PCM512x_GPIO_CONTROL_1, 
					clk_44k1_en_gpio_mask, clk_44k1_en_gpio_mask); 
		}
	}

	return 0;

}

static int justboom_dac_startup(struct snd_pcm_substream *substream) {
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *soc_card = rtd->card;
	
	struct justboom_dac_drvdata *drvdata =
		snd_soc_card_get_drvdata(soc_card);
		
	/* turn on LED */
	snd_soc_update_bits(codec, PCM512x_GPIO_CONTROL_1, 
		JUSTBOOM_PCM512x_GPIO_MASK(drvdata->led_gpio), 
		JUSTBOOM_PCM512x_GPIO_MASK(drvdata->led_gpio));
		
	return 0;
}

static void justboom_dac_shutdown(struct snd_pcm_substream *substream) {
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_card *soc_card = rtd->card;
	struct justboom_dac_drvdata *drvdata =
		snd_soc_card_get_drvdata(soc_card);
		
	if (drvdata->mclk_select == JUSTBOOM_MCLK_SEL_CLK48K_RATE) {
		 /* disable 48000 rate oscillator */
		clk_disable_unprepare(drvdata->mclk_48k);
	} else if (drvdata->mclk_select == JUSTBOOM_MCLK_SEL_CLK44K1_RATE) {
		/* disable 44100 rate oscillator */
		if (drvdata->mclk_44k1) {
			snd_soc_update_bits(codec, PCM512x_GPIO_CONTROL_1, 
				   JUSTBOOM_PCM512x_GPIO_MASK(drvdata->clk_44k1_en_gpio), 
				   0x00);
		}
	}
	drvdata->mclk_select = JUSTBOOM_MCLK_SEL_NOCLOCK;
	
	/* turn off LED */
	snd_soc_update_bits(codec, PCM512x_GPIO_CONTROL_1, 
			JUSTBOOM_PCM512x_GPIO_MASK(drvdata->led_gpio), 0x00);
}

/* machine stream operations */
static struct snd_soc_ops justboom_dac_ops = {
	.hw_params = justboom_dac_hw_params,
	.startup = justboom_dac_startup,
	.shutdown = justboom_dac_shutdown,
};

static struct snd_soc_dai_link justboom_dac_dai[] = {
{
	.name		= "JustBoom DAC",
	.stream_name	= "JustBoom HiFi",
	.codec_dai_name	= "pcm512x-hifi",
	.dai_fmt	= SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
				SND_SOC_DAIFMT_CBS_CFS,
	.ops		= &justboom_dac_ops,
	.init		= justboom_dac_init,
},
};

static const struct of_device_id justboom_dac_of_match[] = {
	{
		.compatible = "justboom,justboom-dac",
		.data = (void *) &justboom_dac_dai,
	},
	{},
};

/* audio machine driver */
static struct snd_soc_card justboom_dac = {
	.name         = "justboom_dac",
	.driver_name  = "JustBoomDac",
	.owner        = THIS_MODULE,
	.dai_link     = justboom_dac_dai,
	.num_links    = ARRAY_SIZE(justboom_dac_dai),
};

static int justboom_dac_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *match =
		of_match_device(of_match_ptr(justboom_dac_of_match), &pdev->dev);
	struct snd_soc_dai_link *dai = (struct snd_soc_dai_link *)match->data;
	struct justboom_dac_drvdata *drvdata = NULL;
	struct clk *mclk_48k, *mclk_44k1;
	int ret = 0;

	justboom_dac.dai_link = dai;

	dai->codec_of_node = of_parse_phandle(np, "audio-codec", 0);
	if (!dai->codec_of_node) {
		dev_err(&pdev->dev,"No codec dt node.\n");
		return -EINVAL;
	} 

	dai->cpu_of_node = of_parse_phandle(np, "mcasp-controller", 0);
	if (!dai->cpu_of_node)
		return -EINVAL;

	dai->platform_of_node = dai->cpu_of_node;

	justboom_dac.dev = &pdev->dev;
	ret = snd_soc_of_parse_card_name(&justboom_dac, "model");
	if (ret)
		return ret;

	mclk_48k = devm_clk_get(&pdev->dev, "mclk_48k");
	if (PTR_ERR(mclk_48k) == -EPROBE_DEFER) {
		return -EPROBE_DEFER;
	} else if (IS_ERR(mclk_48k)) {
		dev_dbg(&pdev->dev, "mclk_48k not found.\n");
		mclk_48k = NULL;
	}
	
	mclk_44k1 = devm_clk_get(&pdev->dev, "mclk_44k1");
	if (PTR_ERR(mclk_44k1) == -EPROBE_DEFER) {
		return -EPROBE_DEFER;
	} else if (IS_ERR(mclk_44k1)) {
		dev_dbg(&pdev->dev, "mclk_44k1 not found.\n");
		mclk_44k1 = NULL;
	}

	drvdata = devm_kzalloc(&pdev->dev, sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;

	drvdata->mclk_48k = mclk_48k;
	drvdata->mclk_44k1 = mclk_44k1;

	ret = of_property_read_u32(np, "clk-44k1-rate-en-gpio", 
			&drvdata->clk_44k1_en_gpio);
	if (ret) {
		drvdata->clk_44k1_en_gpio = 5;
		dev_dbg(&pdev->dev, 
				"clk-44k1-rate-en-gpio not defined, using default.\n");
	} 
	
	ret = of_property_read_u32(np, "led-gpio", &drvdata->led_gpio);
	if (ret) {
		drvdata->clk_44k1_en_gpio = 4;
		dev_dbg(&pdev->dev, "led-gpio not defined, using default.\n");
	} 
	
	snd_soc_card_set_drvdata(&justboom_dac, drvdata);
	
	digital_gain_0db_limit = !of_property_read_bool(np, 
			"justboom,24db_digital_gain");
			
	ret = devm_snd_soc_register_card(&pdev->dev, &justboom_dac);
	if (ret)
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);

	return ret;
}

static int justboom_dac_remove(struct platform_device *pdev)
{
	return snd_soc_unregister_card(&justboom_dac);
}

MODULE_DEVICE_TABLE(of, justboom_dac_of_match);

static struct platform_driver justboom_dac_driver = {
	.driver = {
		.name   = "justboom-dac",
		.owner  = THIS_MODULE,
		.of_match_table = justboom_dac_of_match,
	},
	.probe          = justboom_dac_probe,
	.remove         = justboom_dac_remove,
};

module_platform_driver(justboom_dac_driver);

MODULE_AUTHOR("Milan Neskovic <info@justboom.co>");
MODULE_DESCRIPTION("ASoC Driver for JustBoom DAC Sound Card");
MODULE_LICENSE("GPL v2");
