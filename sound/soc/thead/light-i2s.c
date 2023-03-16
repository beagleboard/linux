/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2021 Alibaba Group Holding Limited.
 */
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/pm_runtime.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/scatterlist.h>
#include <linux/sh_dma.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <sound/sh_fsi.h>
#include "light-i2s.h"
#include "light-pcm.h"
#include <linux/dmaengine.h>
#include <linux/regmap.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/dmaengine_pcm.h>
#include <linux/mfd/syscon.h>
#include <dt-bindings/pinctrl/light-fm-aon-pinctrl.h>

#define IIS_SRC_CLK  294912000
#define AUDIO_IIS_SRC0_CLK  49152000
#define AUDIO_IIS_SRC1_CLK  135475200
#define IIS_MCLK_SEL 256
#define HDMI_DIV_VALUE    2
#define DIV_DEFAULT	  1
#define MONO_SOURCE	  1
#define STEREO_CHANNEL    2

#define AP_I2S		"ap_i2s"
#define AUDIO_I2S0	"i2s0"
#define AUDIO_I2S1	"i2s1"
#define AUDIO_I2S2	"i2s2"
#define AUDIO_I2S3	"i2s3"

#define LIGHT_I2S_DMABUF_SIZE     (64 * 1024)

#define LIGHT_RATES SNDRV_PCM_RATE_8000_384000
#define LIGHT_FMTS (SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S8)

#define LIGHT_AUDIO_PAD_CONFIG(idx)   (i2s_priv->cfg_off + ((idx-25) >> 1) * 4)

static u32 light_special_sample_rates[] = { 11025, 22050, 44100, 88200 };

static int light_audio_cpr_set(struct light_i2s_priv *chip, unsigned int cpr_off,
                               unsigned int mask, unsigned int val)
{
       return regmap_update_bits(chip->audio_cpr_regmap,
                               cpr_off, mask, val);
}

static void light_i2s_set_div_sclk(struct light_i2s_priv *chip, u32 sample_rate, unsigned int div_val)
{
	u32 div;
	u32 div0;
	int i;
	u32 i2s_src_clk = 0;

	if (!strcmp(chip->name, AP_I2S))
		div = IIS_SRC_CLK / IIS_MCLK_SEL;
	else {
		for (i = 0; i < ARRAY_SIZE(light_special_sample_rates); i++) {
			if (light_special_sample_rates[i] == sample_rate) {
				i2s_src_clk = 1;
				break;
			}
		}
		if (!strcmp(chip->name, AUDIO_I2S0)) {
			if (!i2s_src_clk) {
				light_audio_cpr_set(chip, CPR_PERI_CLK_SEL_REG, CPR_I2S0_SRC_SEL_MSK, CPR_I2S0_SRC_SEL(0));
				div = AUDIO_IIS_SRC0_CLK / IIS_MCLK_SEL;
			} else {
				light_audio_cpr_set(chip, CPR_PERI_CLK_SEL_REG, CPR_I2S0_SRC_SEL_MSK, CPR_I2S0_SRC_SEL(2));
				div = AUDIO_IIS_SRC1_CLK / IIS_MCLK_SEL;
			}
		} else if (!strcmp(chip->name, AUDIO_I2S1)) {
			if (!i2s_src_clk) {
				light_audio_cpr_set(chip, CPR_PERI_CLK_SEL_REG, CPR_I2S1_SRC_SEL_MSK, CPR_I2S1_SRC_SEL(0));
				div = AUDIO_IIS_SRC0_CLK / IIS_MCLK_SEL;
			} else {
				light_audio_cpr_set(chip, CPR_PERI_CLK_SEL_REG, CPR_I2S1_SRC_SEL_MSK, CPR_I2S1_SRC_SEL(2));
				div = AUDIO_IIS_SRC1_CLK / IIS_MCLK_SEL;
			}
		} else if (!strcmp(chip->name, AUDIO_I2S2)) {
			if (!i2s_src_clk) {
				light_audio_cpr_set(chip, CPR_PERI_CLK_SEL_REG, CPR_I2S2_SRC_SEL_MSK, CPR_I2S2_SRC_SEL(0));
				div = AUDIO_IIS_SRC0_CLK / IIS_MCLK_SEL;
			} else {
				light_audio_cpr_set(chip, CPR_PERI_CLK_SEL_REG, CPR_I2S2_SRC_SEL_MSK, CPR_I2S2_SRC_SEL(2));
				div = AUDIO_IIS_SRC1_CLK / IIS_MCLK_SEL;
			}
		} else if (!strcmp(chip->name, AUDIO_I2S3)) {
				div = AUDIO_IIS_SRC0_CLK / IIS_MCLK_SEL;
		}
	}

	div0 = (div + div % sample_rate) / sample_rate / div_val;
	writel(div0, chip->regs + I2S_DIV0_LEVEL);
}

static inline void light_snd_txctrl(struct light_i2s_priv *chip, bool on)
{
	u32 dma_en = 0;
	u32 i2s_en = 0;
	u32 i2s_status = 0;
	u32 i2s_imr = 0;

	if (on) {
		dma_en |= DMACR_TDMAE_EN;
		i2s_en |= IISEN_I2SEN;
		writel(dma_en, chip->regs + I2S_DMACR);
		writel(i2s_en, chip->regs + I2S_IISEN);
	} else {
		dma_en &= ~DMACR_TDMAE_EN;
		i2s_en &= ~IISEN_I2SEN;
		i2s_status  = readl(chip->regs + I2S_SR);
		while ((i2s_status & SR_TXBUSY_STATUS) || !(i2s_status & SR_TFNF_TX_FIFO_NOT_FULL)) {
			i2s_status  = readl(chip->regs + I2S_SR);
		}

		i2s_imr  = readl(chip->regs + I2S_IMR);

		i2s_imr &= ~(IMR_TXUIRM_INTR_MSK);
		i2s_imr &= ~(IMR_TXEIM_INTR_MSK);

		writel(i2s_imr, chip->regs + I2S_IMR);
		i2s_imr  = readl(chip->regs + I2S_IMR);

		writel(dma_en, chip->regs + I2S_DMACR);

		i2s_status  = readl(chip->regs + I2S_SR);
		while ((i2s_status & SR_TXBUSY_STATUS) || !(i2s_status & SR_TFE_TX_FIFO_EMPTY)) {
			i2s_status  = readl(chip->regs + I2S_SR);
		}

		mdelay(10);
		writel(i2s_en, chip->regs + I2S_IISEN);
	}
}

static inline void light_snd_rxctrl(struct light_i2s_priv *chip, bool on)
{
        u32 dma_en;
	u32 i2s_en;

	if (on) {
		dma_en |= DMACR_RDMAE_EN;
		i2s_en |= IISEN_I2SEN;
	} else {
		dma_en &= ~DMACR_RDMAE_EN;
		i2s_en &= ~IISEN_I2SEN;
	}

        writel(dma_en, chip->regs + I2S_DMACR);
        writel(i2s_en, chip->regs + I2S_IISEN);
}

static int light_i2s_dai_startup(struct snd_pcm_substream *substream,
			   struct snd_soc_dai *dai)
{
	return 0;
}

static void light_i2s_dai_shutdown(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct light_i2s_priv *i2s_private = snd_soc_dai_get_drvdata(dai);

       if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		light_snd_rxctrl(i2s_private, 0);

	clk_disable_unprepare(i2s_private->clk);
}

/**
 * light_i2s_dai_trigger: start and stop the DMA transfer.
 *
 * This function is called by ALSA to start, stop, pause, and resume the DMA
 * transfer of data.
 */
static int light_i2s_dai_trigger(struct snd_pcm_substream *substream, int cmd,
			   struct snd_soc_dai *dai)
{
	int ret = 0;

	struct light_i2s_priv *i2s_private = snd_soc_dai_get_drvdata(dai);
        bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (tx)
			light_snd_txctrl(i2s_private, 1);
		else
			light_snd_rxctrl(i2s_private, 1);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
        case SNDRV_PCM_TRIGGER_SUSPEND:
        case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (tx)
			light_snd_txctrl(i2s_private, 0);
                break;

        default:
                return -EINVAL;

	}

	return ret;
}

static int light_i2s_set_fmt_dai(struct snd_soc_dai *cpu_dai, unsigned int fmt)
{
	struct light_i2s_priv *i2s_private = snd_soc_dai_get_drvdata(cpu_dai);
	u32 cnfout = 0;
	u32 cnfin = 0;

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		cnfout |= IISCNFOUT_TSAFS_I2S;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		cnfout |= IISCNFOUT_TSAFS_RIGHT_JUSTIFIED;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		cnfout |= IISCNFOUT_TSAFS_LEFT_JUSTIFIED;
		break;
	default:
		pr_err("Unknown fmt dai\n");
		return -EINVAL;
	}

	regmap_update_bits(i2s_private->regmap, I2S_IISCNF_OUT,
			IISCNFOUT_TSAFS_MSK,
			cnfout);

	cnfin |= CNFIN_I2S_RXMODE_MASTER_MODE;
	regmap_update_bits(i2s_private->regmap, I2S_IISCNF_IN,
			CNFIN_I2S_RXMODE_Msk,
			cnfin);

	return 0;
}

static int light_i2s_dai_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct light_i2s_priv *i2s_private = snd_soc_dai_get_drvdata(dai);
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	u32 val;
	u32 len = 0;
	u32 sclk_sel = 0;
	u32 rate;
	u32 funcmode;
	u32 iiscnf_out;
	u32 iiscnf_in;
	u32 i2s_en;

	u32 channels = params_channels(params);

	rate = params_rate(params);

	iiscnf_out = readl(i2s_private->regs + I2S_IISCNF_OUT);
	iiscnf_in = readl(i2s_private->regs + I2S_IISCNF_IN);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		val |= I2S_DATA_8BIT_WIDTH_32BIT;
		len = 32;
                break;
	case SNDRV_PCM_FORMAT_S16_LE:
		val |= I2S_DATA_WIDTH_16BIT;
		len = 32;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		val |= I2S_DATA_WIDTH_24BIT;
		len = 32;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		val |= I2S_DATA_WIDTH_32BIT;
		len = 32;
		break;
	default:
		pr_err("Unknown data format\n");
		return -EINVAL;
	}

	sclk_sel = len*STEREO_CHANNEL;

	switch (sclk_sel) {
	case 16:
		val |= FSSTA_SCLK_SEL_16;
		break;
	case 32:
		val |= FSSTA_SCLK_SEL_32;
		break;
	case 48:
		val |= FSSTA_SCLK_SEL_48;
		break;
	case 64:
		val |= FSSTA_SCLK_SEL_64;
		break;
	default:
		pr_err("Not support channel num %d\n", channels);
		return -EINVAL;
	}

	i2s_en &= ~IISEN_I2SEN;
	writel(i2s_en, i2s_private->regs + I2S_IISEN);

	regmap_update_bits(i2s_private->regmap, I2S_FSSTA,
			FSSTA_DATAWTH_Msk | FSSTA_SCLK_SEL_Msk,
			val);
	funcmode = readl(i2s_private->regs + I2S_FUNCMODE);
	if (tx) {
		funcmode |= FUNCMODE_TMODE_WEN;
		funcmode &= ~FUNCMODE_CH1_ENABLE;
		funcmode |= FUNCMODE_RMODE_WEN;
		funcmode &= ~FUNCMODE_RMODE;
		funcmode &= ~FUNCMODE_TMODE;
		funcmode |= FUNCMODE_TMODE;
	} else {
		funcmode |= FUNCMODE_RMODE_WEN;
		funcmode |= FUNCMODE_CH0_ENABLE;
		funcmode |= FUNCMODE_CH1_ENABLE;
		funcmode |= FUNCMODE_CH2_ENABLE;
		funcmode |= FUNCMODE_TMODE_WEN;
		funcmode &= ~FUNCMODE_TMODE;
		funcmode &= ~FUNCMODE_RMODE;
		funcmode |= FUNCMODE_RMODE;
	}

	writel(funcmode, i2s_private->regs + I2S_FUNCMODE);

	if (channels == MONO_SOURCE) {
		iiscnf_out |= IISCNFOUT_TX_VOICE_EN_MONO;
		iiscnf_in |= CNFIN_RX_CH_SEL_LEFT;
		iiscnf_in |= CNFIN_RVOICEEN_MONO;
	} else {
		iiscnf_out &= ~IISCNFOUT_TX_VOICE_EN_MONO;
		iiscnf_in &= ~CNFIN_RX_CH_SEL_LEFT;
		iiscnf_in &= ~CNFIN_RVOICEEN_MONO;
	}

	if (tx)
		writel(iiscnf_out, i2s_private->regs + I2S_IISCNF_OUT);
	else
		writel(iiscnf_in, i2s_private->regs + I2S_IISCNF_IN);

	light_i2s_set_div_sclk(i2s_private, rate, DIV_DEFAULT);

	return 0;
}

static int light_hdmi_dai_hw_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct light_i2s_priv *i2s_private = snd_soc_dai_get_drvdata(dai);
	bool tx = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	u32 val;
	u32 len = 0;
	u32 rate;
	u32 funcmode;
	u32 iiscnf_out;
	u32 i2s_en;

	u32 channels = params_channels(params);

	rate = params_rate(params);

	switch (params_format(params)) {
		case SNDRV_PCM_FORMAT_S16_LE:
			val |= I2S_DATA_WIDTH_16BIT;
			len = 16;
			break;
		case SNDRV_PCM_FORMAT_S24_LE:
			val |= I2S_DATA_WIDTH_24BIT;
			len = 24;
			break;
		default:
			pr_err("Unknown data format\n");
			return -EINVAL;
	}

	val |= FSSTA_SCLK_SEL_64;

	i2s_en &= ~IISEN_I2SEN;
	writel(i2s_en, i2s_private->regs + I2S_IISEN);

	regmap_update_bits(i2s_private->regmap, I2S_FSSTA,
			FSSTA_DATAWTH_Msk | FSSTA_SCLK_SEL_Msk,
			val);
	funcmode = readl(i2s_private->regs + I2S_FUNCMODE);
	if (tx) {
		funcmode |= FUNCMODE_TMODE_WEN;
		funcmode &= ~FUNCMODE_TMODE;
		funcmode |= FUNCMODE_TMODE;
	} else {
		funcmode |= FUNCMODE_RMODE_WEN;
		funcmode &= ~FUNCMODE_RMODE;
		funcmode |= FUNCMODE_RMODE;
	}

	writel(funcmode, i2s_private->regs + I2S_FUNCMODE);

	iiscnf_out = readl(i2s_private->regs + I2S_IISCNF_OUT);
	if (channels == MONO_SOURCE)
		iiscnf_out |= IISCNFOUT_TX_VOICE_EN_MONO;
	else
		iiscnf_out &= ~IISCNFOUT_TX_VOICE_EN_MONO;

	writel(iiscnf_out, i2s_private->regs + I2S_IISCNF_OUT);

	light_i2s_set_div_sclk(i2s_private, rate, DIV_DEFAULT);

	return 0;
}

static int light_i2s_dai_probe(struct snd_soc_dai *dai)
{
	struct light_i2s_priv *i2s = snd_soc_dai_get_drvdata(dai);

	if(i2s)
		snd_soc_dai_init_dma_data(dai, &i2s->dma_params_tx,
				&i2s->dma_params_rx);

	return 0;
}

static const struct snd_soc_dai_ops light_i2s_dai_ops = {
	.startup	= light_i2s_dai_startup,
	.shutdown	= light_i2s_dai_shutdown,
	.trigger	= light_i2s_dai_trigger,
	.set_fmt	= light_i2s_set_fmt_dai,
	.hw_params	= light_i2s_dai_hw_params,
};

static const struct snd_soc_dai_ops light_hdmi_dai_ops = {
	.startup        = light_i2s_dai_startup,
	.shutdown       = light_i2s_dai_shutdown,
	.trigger        = light_i2s_dai_trigger,
	.set_fmt        = light_i2s_set_fmt_dai,
	.hw_params      = light_hdmi_dai_hw_params,
};

static struct snd_soc_dai_driver light_i2s_soc_dai[] = {
	{
		.probe = light_i2s_dai_probe,
		.name			= "light-i2s-dai",
		.playback = {
			.rates		= LIGHT_RATES,
			.formats	= LIGHT_FMTS,
			.channels_min	= 1,
			.channels_max	= 2,
		},
		.capture = {
			.rates		= LIGHT_RATES,
			.formats	= LIGHT_FMTS,
			.channels_min	= 1,
			.channels_max	= 2,
		},
		.ops = &light_i2s_dai_ops,
	},
	{
		.probe = light_i2s_dai_probe,
		.name			= "light-hdmi-dai",
		.playback = {
			.rates		= LIGHT_RATES,
			.formats	= LIGHT_FMTS,
			.channels_min	= 1,
			.channels_max	= 2,
		},
		.ops = &light_hdmi_dai_ops,
	},
};


static const struct snd_soc_component_driver light_i2s_soc_component = {
	.name		= "light_i2s",
};

static int light_pcm_probe(struct platform_device *pdev,struct light_i2s_priv *i2s)
{
	int ret;

	ret = light_pcm_dma_init(pdev, LIGHT_I2S_DMABUF_SIZE);

	if (ret) {
		pr_err("light_pcm_dma_init error\n");
		return 0;
	}
	return 0;
}

static bool light_i2s_wr_reg(struct device *dev, unsigned int reg)
{
        switch (reg) {
        case I2S_IISEN:
        case I2S_FUNCMODE:
        case I2S_IISCNF_IN:
        case I2S_FSSTA:
        case I2S_IISCNF_OUT:
        case I2S_DMACR:
                return true;
        default:
                return false;
        }
}

static bool light_i2s_rd_reg(struct device *dev, unsigned int reg)
{
        switch (reg) {
        case I2S_IISEN:
        case I2S_FUNCMODE:
        case I2S_IISCNF_IN:
        case I2S_FSSTA:
        case I2S_IISCNF_OUT:
        case I2S_DMACR:
                return true;
        default:
                return false;
        }
}

static const struct regmap_config light_i2s_regmap_config = {
        .reg_bits = 32,
        .reg_stride = 4,
        .val_bits = 32,
        .max_register = I2S_DR4,
        .writeable_reg = light_i2s_wr_reg,
        .readable_reg = light_i2s_rd_reg,
        .cache_type = REGCACHE_FLAT,
};

static int light_audio_pinconf_set(struct device *dev, unsigned int pin_id, unsigned int val)
{
	struct light_i2s_priv *i2s_priv = dev_get_drvdata(dev);
	unsigned int shift;
	unsigned int mask = 0;

	i2s_priv->cfg_off = 0xC;

	shift = (((pin_id-25) % 2) << 4);
	mask |= (0xFFFF << shift);
	val = (val << shift);

	return regmap_update_bits(i2s_priv->audio_pin_regmap,
				LIGHT_AUDIO_PAD_CONFIG(pin_id),mask, val);
}

static int light_audio_pinctrl(struct device *dev)
{
	struct light_i2s_priv *i2s_priv = dev_get_drvdata(dev);

	if (!strcmp(i2s_priv->name, AUDIO_I2S0)) {
		light_audio_pinconf_set(i2s_priv->dev, FM_AUDIO_CFG_PA6, 0x4);
		light_audio_pinconf_set(i2s_priv->dev, FM_AUDIO_CFG_PA7, 0x4);
		light_audio_pinconf_set(i2s_priv->dev, FM_AUDIO_CFG_PA9, 0x8);
		light_audio_pinconf_set(i2s_priv->dev, FM_AUDIO_CFG_PA10, 0x8);
		light_audio_pinconf_set(i2s_priv->dev, FM_AUDIO_CFG_PA11, 0x8);
		light_audio_pinconf_set(i2s_priv->dev, FM_AUDIO_CFG_PA12, 0x8);
	} else if (!strcmp(i2s_priv->name, AUDIO_I2S1)) {
		light_audio_pinconf_set(i2s_priv->dev, FM_AUDIO_CFG_PA29, 0x5);
		light_audio_pinconf_set(i2s_priv->dev, FM_AUDIO_CFG_PA30, 0x5);
		light_audio_pinconf_set(i2s_priv->dev, FM_AUDIO_CFG_PA14, 0x8);
		light_audio_pinconf_set(i2s_priv->dev, FM_AUDIO_CFG_PA15, 0x8);
		light_audio_pinconf_set(i2s_priv->dev, FM_AUDIO_CFG_PA16, 0x8);
		light_audio_pinconf_set(i2s_priv->dev, FM_AUDIO_CFG_PA17, 0x8);
	} else if (!strcmp(i2s_priv->name, AUDIO_I2S2)) {
		light_audio_pinconf_set(i2s_priv->dev, FM_AUDIO_CFG_PA6, 0x5);
		light_audio_pinconf_set(i2s_priv->dev, FM_AUDIO_CFG_PA7, 0x5);
		light_audio_pinconf_set(i2s_priv->dev, FM_AUDIO_CFG_PA18, 0x8);
		light_audio_pinconf_set(i2s_priv->dev, FM_AUDIO_CFG_PA19, 0x8);
		light_audio_pinconf_set(i2s_priv->dev, FM_AUDIO_CFG_PA21, 0x8);
		light_audio_pinconf_set(i2s_priv->dev, FM_AUDIO_CFG_PA22, 0x8);
	}

	return 0;
}

static int light_i2s_runtime_suspend(struct device *dev)
{
        struct light_i2s_priv *i2s_priv = dev_get_drvdata(dev);

        regcache_cache_only(i2s_priv->regmap, true);
        clk_disable_unprepare(i2s_priv->clk);

	return 0;
}

static int light_i2s_runtime_resume(struct device *dev)
{
        struct light_i2s_priv *i2s_priv = dev_get_drvdata(dev);
        int ret;

        ret = clk_prepare_enable(i2s_priv->clk);
        if (ret) {
                dev_err(i2s_priv->dev, "clock enable failed %d\n", ret);
                return ret;
        }

        regcache_cache_only(i2s_priv->regmap, false);

        return ret;
}

static const struct of_device_id light_i2s_of_match[] = {
	{ .compatible = "light,light-i2s"},
	{},
};
MODULE_DEVICE_TABLE(of, light_i2s_of_match);

static int light_audio_i2s_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;

	const char *sprop;
	const uint32_t *iprop;
	struct light_i2s_priv *i2s_priv;
	struct resource *res;
	struct device *dev = &pdev->dev;
	unsigned int irq;
	int ret;

	i2s_priv = devm_kzalloc(&pdev->dev, sizeof(*i2s_priv),
		GFP_KERNEL);

	if (!i2s_priv)
		return -ENOMEM;

	i2s_priv->dev = dev;

	sprop = of_get_property(np, "light,mode", NULL);
	if (sprop) {
		if (!strcmp(sprop, "i2s-master"))
			i2s_priv->dai_fmt = SND_SOC_DAIFMT_I2S;
		else
			printk("mode is not i2s-master");
	}

	sprop = of_get_property(np, "light,sel", NULL);
	if (sprop) {
		strcpy(i2s_priv->name, sprop);
	}

	iprop = of_get_property(np, "light,dma_maxburst", NULL);
	if (iprop)
		i2s_priv->dma_maxburst = be32_to_cpup(iprop);
	else
		i2s_priv->dma_maxburst = 8;

	dev_set_drvdata(&pdev->dev, i2s_priv);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	i2s_priv->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(i2s_priv->regs))
		return PTR_ERR(i2s_priv->regs);

        i2s_priv->regmap = devm_regmap_init_mmio(&pdev->dev, i2s_priv->regs,
                                            &light_i2s_regmap_config);
        if (IS_ERR(i2s_priv->regmap)) {
                dev_err(&pdev->dev,
                        "Failed to initialise managed register map\n");
                return PTR_ERR(i2s_priv->regmap);
        }

	if (!strcmp(i2s_priv->name, AUDIO_I2S0) || !strcmp(i2s_priv->name, AUDIO_I2S1) || !strcmp(i2s_priv->name, AUDIO_I2S2)) {
		i2s_priv->audio_pin_regmap = syscon_regmap_lookup_by_phandle(np, "audio-pin-regmap");
		if (IS_ERR(i2s_priv->audio_pin_regmap)) {
			dev_err(&pdev->dev, "cannot find regmap for audio system register\n");
		} else
			light_audio_pinctrl(&pdev->dev);

		i2s_priv->audio_cpr_regmap = syscon_regmap_lookup_by_phandle(np, "audio-cpr-regmap");
		if (IS_ERR(i2s_priv->audio_cpr_regmap)) {
			dev_err(&pdev->dev, "cannot find regmap for audio cpr register\n");
		} else
			light_audio_cpr_set(i2s_priv, CPR_PERI_DIV_SEL_REG, CPR_AUDIO_DIV1_SEL_MSK, CPR_AUDIO_DIV1_SEL(5));
	}

	pm_runtime_enable(&pdev->dev);
        if (!pm_runtime_enabled(&pdev->dev)) {
                ret = light_i2s_runtime_resume(&pdev->dev);
                if (ret)
                        goto err_pm_disable;
        }

	irq = platform_get_irq(pdev, 0);

	if (!res || (int)irq <= 0) {
		dev_err(&pdev->dev, "Not enough light platform resources.\n");
		return -ENODEV;
	}

	i2s_priv->clk = devm_clk_get(&pdev->dev, "pclk");
	if (IS_ERR(i2s_priv->clk))
                return PTR_ERR(i2s_priv->clk);

	ret = clk_prepare_enable(i2s_priv->clk);
        if (ret < 0)
                return ret;

	i2s_priv->dma_params_tx.addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	i2s_priv->dma_params_rx.addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	i2s_priv->dma_params_tx.maxburst = i2s_priv->dma_maxburst;
	i2s_priv->dma_params_rx.maxburst = i2s_priv->dma_maxburst;

	if (!strcmp(i2s_priv->name, AP_I2S)) {
		i2s_priv->dma_params_tx.addr = res->start + I2S_DR;
		i2s_priv->dma_params_rx.addr = res->start + I2S_DR1;
	} else if (!strcmp(i2s_priv->name, AUDIO_I2S0) || !strcmp(i2s_priv->name, AUDIO_I2S1) || !strcmp(i2s_priv->name, AUDIO_I2S2)) {
		i2s_priv->dma_params_tx.addr = res->start + I2S_DR;
		i2s_priv->dma_params_rx.addr = res->start + I2S_DR;
	} else if (!strcmp(i2s_priv->name, AUDIO_I2S3)) {
		i2s_priv->dma_params_tx.addr = res->start + I2S_DR;
		i2s_priv->dma_params_rx.addr = res->start + I2S_DR2;
	}

	light_pcm_probe(pdev, i2s_priv);

	ret = devm_snd_soc_register_component(&pdev->dev, &light_i2s_soc_component,
				    light_i2s_soc_dai, ARRAY_SIZE(light_i2s_soc_dai));
	if (ret < 0) {
		dev_err(&pdev->dev, "cannot snd component register\n");
		goto err_pm_disable;
	}

	return ret;

err_pm_disable:
	pm_runtime_disable(&pdev->dev);

	return ret;
}

static int light_i2s_remove(struct platform_device *pdev)
{
        struct light_i2s_priv *i2s_priv = dev_get_drvdata(&pdev->dev);

        pm_runtime_disable(&pdev->dev);
        if (!pm_runtime_status_suspended(&pdev->dev))
                light_i2s_runtime_suspend(&pdev->dev);

        clk_disable_unprepare(i2s_priv->clk);

        return 0;
}

static const struct dev_pm_ops light_i2s_pm_ops = {
        SET_RUNTIME_PM_OPS(light_i2s_runtime_suspend, light_i2s_runtime_resume,
                           NULL)
};

static struct platform_driver light_i2s_driver = {
	.driver 	= {
		.name	= "light-pcm-audio",
		.pm	= &light_i2s_pm_ops,
		.of_match_table = light_i2s_of_match,
	},
	.probe		= light_audio_i2s_probe,
	.remove	= light_i2s_remove,
};

module_platform_driver(light_i2s_driver);

MODULE_AUTHOR("shuofeng.ren <shuofeng.rsf@linux.alibaba.com>");
MODULE_DESCRIPTION("Thead Light audio driver");
MODULE_LICENSE("GPL v2");
