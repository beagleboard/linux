// SPDX-License-Identifier: GPL-2.0
/*
 *  Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com
 *  Author: Peter Ujfalusi <peter.ujfalusi@ti.com>
 */

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/of.h>

#include <sound/pcm_params.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <dt-bindings/sound/ti-mcasp.h>

#define J721E_CPD_DAI_CNT	2

#define J721E_CLK_PARENT_48000	0
#define J721E_CLK_PARENT_44100	1

#define J721E_MAX_CLK_HSDIV	128
#define PCM1368A_MAX_SYSCLK	36864000

#define J721E_DAI_FMT		(SND_SOC_DAIFMT_RIGHT_J | \
				 SND_SOC_DAIFMT_NB_NF |   \
				 SND_SOC_DAIFMT_CBS_CFS)

static unsigned int ratios_for_pcm3168a[] = {
	256,
	512,
	768,
};

struct j721e_audio_clocks {
	struct clk *target;
	struct clk *parent[2];
};

struct j721e_priv {
	struct device *dev;
	struct snd_soc_card cpb_card;
	struct snd_soc_dai_link cpb_dai_links[J721E_CPD_DAI_CNT];
	struct snd_soc_codec_conf codec_conf;
	struct snd_interval rate_range;

	struct j721e_audio_clocks audio_refclk2;
	struct j721e_audio_clocks cpb_mcasp;
	u32 pll_rates[2];
	unsigned int current_cpb_ref_rate;
	int current_cpb_ref_parent;

	int active;
	unsigned int rate;
	struct mutex mutex;
};

static int j721e_configure_refclk(struct j721e_priv *priv, unsigned int rate)
{
	unsigned int scki;
	int ret = -EINVAL;
	int i, clk_id;

	if (!(rate % 8000))
		clk_id = J721E_CLK_PARENT_48000;
	else if (!(rate % 11025))
		clk_id = J721E_CLK_PARENT_44100;
	else
		return ret;

	for (i = 0; i < ARRAY_SIZE(ratios_for_pcm3168a); i++) {
		scki = ratios_for_pcm3168a[i] * rate;

		if (priv->pll_rates[clk_id] / scki <= J721E_MAX_CLK_HSDIV) {
			ret = 0;
			break;
		}
	}

	if (ret) {
		dev_err(priv->dev, "No valid clock configuration for %u Hz\n",
			rate);
		return ret;
	}

	if (priv->current_cpb_ref_rate != scki) {
		dev_dbg(priv->dev,
			"Configuration for %u Hz: %s, %dxFS (SCKI: %u Hz)\n",
			rate,
			clk_id == J721E_CLK_PARENT_48000 ? "PLL4" : "PLL15",
			ratios_for_pcm3168a[i], scki);

		if (priv->current_cpb_ref_parent != clk_id) {
			ret = clk_set_parent(priv->audio_refclk2.target,
					priv->audio_refclk2.parent[clk_id]);
			if (ret)
				return ret;

			ret = clk_set_parent(priv->cpb_mcasp.target,
					priv->cpb_mcasp.parent[clk_id]);
			if (ret)
				return ret;

			priv->current_cpb_ref_parent = clk_id;
		}

		ret = clk_set_rate(priv->audio_refclk2.target, scki);
		if (ret)
			return ret;

		ret = clk_set_rate(priv->cpb_mcasp.target, scki);
		if (!ret)
			priv->current_cpb_ref_rate = scki;
	}

	return ret;
}

static int j721e_rule_rate(struct snd_pcm_hw_params *params,
			   struct snd_pcm_hw_rule *rule)
{
	struct snd_interval *t = rule->private;

	return snd_interval_refine(hw_param_interval(params, rule->var), t);
}

static int j721e_audio_startup(struct snd_pcm_substream *substream)
{
	struct 	snd_soc_pcm_runtime *rtd = substream->private_data;
	struct j721e_priv *priv = snd_soc_card_get_drvdata(rtd->card);
	int ret = 0;

	mutex_lock(&priv->mutex);
	priv->active++;

	if (priv->rate)
		ret = snd_pcm_hw_constraint_single(substream->runtime,
						   SNDRV_PCM_HW_PARAM_RATE,
						   priv->rate);
	else
		ret = snd_pcm_hw_rule_add(substream->runtime, 0,
					  SNDRV_PCM_HW_PARAM_RATE,
					  j721e_rule_rate, &priv->rate_range,
					  SNDRV_PCM_HW_PARAM_RATE, -1);

	mutex_unlock(&priv->mutex);

	if (ret)
		return ret;

	/* Reset TDM slots to 32 */
	ret = snd_soc_dai_set_tdm_slot(rtd->cpu_dai, 0x3, 0x3, 2, 32);
	if (ret)
		return ret;

	ret = snd_soc_dai_set_tdm_slot(rtd->codec_dai, 0x3, 0x3, 2, 32);
	if (ret)
		return ret;

	return 0;
}

static int j721e_audio_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params)
{
	struct 	snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_card *card = rtd->card;
	struct j721e_priv *priv = snd_soc_card_get_drvdata(card);
	int slot_width = 32;
	int ret;

	mutex_lock(&priv->mutex);

	if (priv->rate && priv->rate != params_rate(params)) {
		ret = -EINVAL;
		goto out;
	}

	if (params_width(params) == 16)
		slot_width = 16;

	ret = snd_soc_dai_set_tdm_slot(rtd->cpu_dai, 0x3, 0x3, 2, slot_width);
	if (ret)
		goto out;

	ret = snd_soc_dai_set_tdm_slot(rtd->codec_dai, 0x3, 0x3, 2, slot_width);
	if (ret)
		goto out;

	ret = j721e_configure_refclk(priv, params_rate(params));
	if (ret)
		goto out;

	ret = snd_soc_dai_set_sysclk(rtd->codec_dai, 0,
				     priv->current_cpb_ref_rate,
				     SND_SOC_CLOCK_IN);
	if (ret)
		goto out;

	ret = snd_soc_dai_set_sysclk(rtd->cpu_dai, MCASP_CLK_HCLK_AUXCLK,
				     priv->current_cpb_ref_rate,
				     SND_SOC_CLOCK_IN);

	if (!ret)
		priv->rate = params_rate(params);

out:
	mutex_unlock(&priv->mutex);
	return ret;
}

static void j721e_audio_shutdown(struct snd_pcm_substream *substream)
{
	struct 	snd_soc_pcm_runtime *rtd = substream->private_data;
	struct j721e_priv *priv = snd_soc_card_get_drvdata(rtd->card);

	mutex_lock(&priv->mutex);

	priv->active--;
	if (!priv->active)
		priv->rate = 0;

	mutex_unlock(&priv->mutex);
}

static const struct snd_soc_ops j721e_audio_ops = {
	.startup = j721e_audio_startup,
	.hw_params = j721e_audio_hw_params,
	.shutdown = j721e_audio_shutdown,
};

static int j721e_audio_init(struct snd_soc_pcm_runtime *rtd)
{
	struct j721e_priv *priv = snd_soc_card_get_drvdata(rtd->card);
	int ret;

	/* Set up initial clock configuration */
	ret = j721e_configure_refclk(priv, 48000);
	if (ret)
		return ret;

	ret = snd_soc_dai_set_sysclk(rtd->codec_dai, 0,
				     priv->current_cpb_ref_rate,
				     SND_SOC_CLOCK_IN);
	if (ret)
		return ret;

	ret = snd_soc_dai_set_sysclk(rtd->cpu_dai, MCASP_CLK_HCLK_AUXCLK,
				     priv->current_cpb_ref_rate,
				     SND_SOC_CLOCK_IN);
	if (ret)
		return ret;

	/* Set initial tdm slots */
	ret = snd_soc_dai_set_tdm_slot(rtd->cpu_dai, 0x3, 0x3, 2, 32);
	if (ret)
		return ret;

	ret = snd_soc_dai_set_tdm_slot(rtd->codec_dai, 0x3, 0x3, 2, 32);

	return ret;
}

static const struct snd_soc_dapm_widget j721e_cpb_dapm_widgets[] = {
	SND_SOC_DAPM_HP("CPB Stereo HP 1", NULL),
	SND_SOC_DAPM_HP("CPB Stereo HP 2", NULL),
	SND_SOC_DAPM_HP("CPB Stereo HP 3", NULL),
	SND_SOC_DAPM_LINE("CPB Line Out", NULL),
	SND_SOC_DAPM_MIC("CPB Stereo Mic 1", NULL),
	SND_SOC_DAPM_MIC("CPB Stereo Mic 2", NULL),
	SND_SOC_DAPM_LINE("CPB Line In", NULL),
};

static const struct snd_soc_dapm_route j721e_cpb_dapm_routes[] = {
	{"CPB Stereo HP 1", NULL, "codec1 AOUT1L"},
	{"CPB Stereo HP 1", NULL, "codec1 AOUT1R"},
	{"CPB Stereo HP 2", NULL, "codec1 AOUT2L"},
	{"CPB Stereo HP 2", NULL, "codec1 AOUT2R"},
	{"CPB Stereo HP 3", NULL, "codec1 AOUT3L"},
	{"CPB Stereo HP 3", NULL, "codec1 AOUT3R"},
	{"CPB Line Out", NULL, "codec1 AOUT4L"},
	{"CPB Line Out", NULL, "codec1 AOUT4R"},

	{"codec1 AIN1L", NULL, "CPB Stereo Mic 1"},
	{"codec1 AIN1R", NULL, "CPB Stereo Mic 1"},
	{"codec1 AIN2L", NULL, "CPB Stereo Mic 2"},
	{"codec1 AIN2R", NULL, "CPB Stereo Mic 2"},
	{"codec1 AIN3L", NULL, "CPB Line In"},
	{"codec1 AIN3R", NULL, "CPB Line In"},
};

static int j721e_get_clocks(struct platform_device *pdev,
			    struct j721e_audio_clocks *clocks, char *prefix)
{
	struct clk *parent;
	char *clk_name;
	int ret;

	clocks->target = devm_clk_get(&pdev->dev, prefix);
	if (IS_ERR(clocks->target)) {
		ret = PTR_ERR(clocks->target);
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "failed to acquire %s': %d\n",
				prefix, ret);
		return ret;
	}

	clk_name = kasprintf(GFP_KERNEL, "%s-48000", prefix);
	if (clk_name) {
		parent = devm_clk_get(&pdev->dev, clk_name);
		kfree(clk_name);
		if (IS_ERR(parent)) {
			ret = PTR_ERR(parent);
			if (ret != -EPROBE_DEFER)
				dev_err(&pdev->dev, "failed to acquire %s': %d\n",
					prefix, ret);
			return ret;
		}
		clocks->parent[J721E_CLK_PARENT_48000] = parent;
	} else {
		return -ENOMEM;
	}

	clk_name = kasprintf(GFP_KERNEL, "%s-44100", prefix);
	if (clk_name) {
		parent = devm_clk_get(&pdev->dev, clk_name);
		kfree(clk_name);
		if (IS_ERR(parent)) {
			ret = PTR_ERR(parent);
			if (ret != -EPROBE_DEFER)
				dev_err(&pdev->dev, "failed to acquire %s': %d\n",
					prefix, ret);
			return ret;
		}
		clocks->parent[J721E_CLK_PARENT_44100] = parent;
	} else {
		return -ENOMEM;
	}

	return 0;
}

static void j721e_calculate_rate_range(struct j721e_priv *priv)
{
	unsigned int min_rate, max_rate, pll_rate;

	pll_rate = priv->pll_rates[J721E_CLK_PARENT_44100];
	min_rate = pll_rate / J721E_MAX_CLK_HSDIV;
	min_rate /= ratios_for_pcm3168a[ARRAY_SIZE(ratios_for_pcm3168a) - 1];

	pll_rate = priv->pll_rates[J721E_CLK_PARENT_48000];
	if (pll_rate > PCM1368A_MAX_SYSCLK)
		pll_rate = PCM1368A_MAX_SYSCLK;

	max_rate = pll_rate / ratios_for_pcm3168a[0];

	snd_interval_any(&priv->rate_range);
	priv->rate_range.min = min_rate;
	priv->rate_range.max = max_rate;
}

static int j721e_soc_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct snd_soc_card *card;
	struct device_node *cpb_dai_node, *cpb_codec_node;
	struct j721e_priv *priv;
	int ret;

	if (!node) {
		dev_err(&pdev->dev, "of node is missing.\n");
		return -ENODEV;
	}

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (priv == NULL)
		return -ENOMEM;

	priv->current_cpb_ref_parent = -1;
	priv->dev = &pdev->dev;
	card = &priv->cpb_card;
	card->dev = &pdev->dev;
	card->owner = THIS_MODULE;
	card->dapm_widgets = j721e_cpb_dapm_widgets;
	card->num_dapm_widgets = ARRAY_SIZE(j721e_cpb_dapm_widgets);
	card->dapm_routes = j721e_cpb_dapm_routes;
	card->num_dapm_routes = ARRAY_SIZE(j721e_cpb_dapm_routes);
	card->fully_routed = 1;

	if (snd_soc_of_parse_card_name(card, "ti,model")) {
		dev_err(&pdev->dev, "Card name is not provided\n");
		return -ENODEV;
	}

	cpb_dai_node = of_parse_phandle(node, "ti,cpb-mcasp", 0);
	if (!cpb_dai_node) {
		dev_err(&pdev->dev, "CPB McASP node is not provided\n");
		return -EINVAL;
	}

	cpb_codec_node = of_parse_phandle(node, "ti,cpb-codec", 0);
	if (!cpb_codec_node) {
		dev_err(&pdev->dev, "CPB codec node is not provided\n");
		return -EINVAL;
	}

	ret = j721e_get_clocks(pdev, &priv->audio_refclk2, "audio-refclk2");
	if (ret)
		return ret;

	ret = j721e_get_clocks(pdev, &priv->cpb_mcasp, "cpb-mcasp");
	if (ret)
		return ret;

	ret = of_property_read_u32(node, "pll4-rate",
				   &priv->pll_rates[J721E_CLK_PARENT_48000]);
	if (ret)
		return ret;

	ret = of_property_read_u32(node, "pll15-rate",
				   &priv->pll_rates[J721E_CLK_PARENT_44100]);
	if (ret)
		return ret;
	
	priv->cpb_dai_links[0].name = "CPB pcm3168a DAC";
	priv->cpb_dai_links[0].stream_name = "cpb pcm3168a Playback";
	priv->cpb_dai_links[0].cpu_of_node = cpb_dai_node;
	priv->cpb_dai_links[0].platform_of_node = cpb_dai_node;
	priv->cpb_dai_links[0].codec_of_node = cpb_codec_node;
	priv->cpb_dai_links[0].codec_dai_name = "pcm3168a-dac";
	priv->cpb_dai_links[0].playback_only = 1;
	priv->cpb_dai_links[0].dai_fmt = J721E_DAI_FMT;
	priv->cpb_dai_links[0].init = j721e_audio_init;
	priv->cpb_dai_links[0].ops = &j721e_audio_ops;

	priv->cpb_dai_links[1].name = "CPB pcm3168a ADC";
	priv->cpb_dai_links[1].stream_name = "cpb pcm3168a Capture";
	priv->cpb_dai_links[1].cpu_of_node = cpb_dai_node;
	priv->cpb_dai_links[1].platform_of_node = cpb_dai_node;
	priv->cpb_dai_links[1].codec_of_node = cpb_codec_node;
	priv->cpb_dai_links[1].codec_dai_name = "pcm3168a-adc";
	priv->cpb_dai_links[1].capture_only = 1;
	priv->cpb_dai_links[1].dai_fmt = J721E_DAI_FMT;
	priv->cpb_dai_links[1].init = j721e_audio_init;
	priv->cpb_dai_links[1].ops = &j721e_audio_ops;

	card->dai_link = priv->cpb_dai_links;
	card->num_links = ARRAY_SIZE(priv->cpb_dai_links);

	priv->codec_conf.of_node = cpb_codec_node;
	priv->codec_conf.name_prefix = "codec1";
	if (!priv->codec_conf.name_prefix)
		return -ENOMEM;

	card->codec_conf = &priv->codec_conf;
	card->num_configs = 1;

	j721e_calculate_rate_range(priv);

	snd_soc_card_set_drvdata(card, priv);

	mutex_init(&priv->mutex);
	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret)
		dev_err(&pdev->dev, "devm_snd_soc_register_card() failed: %d\n",
			ret);

	return ret;

	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id j721e_audio_of_match[] = {
	{ .compatible = "ti,j721e-cpb-audio", },
	{},
};
MODULE_DEVICE_TABLE(of, j721e_audio_of_match);
#endif

static struct platform_driver j721e_soc_driver = {
	.driver = {
		.name = "j721e-audio",
		.pm = &snd_soc_pm_ops,
		.of_match_table = of_match_ptr(j721e_audio_of_match),
	},
	.probe = j721e_soc_probe,
};

module_platform_driver(j721e_soc_driver);

MODULE_AUTHOR("Peter Ujfalusi <peter.ujfalusi@ti.com>");
MODULE_DESCRIPTION("ASoC machine driver for j721e Common Processor Board");
MODULE_LICENSE("GPL v2");
