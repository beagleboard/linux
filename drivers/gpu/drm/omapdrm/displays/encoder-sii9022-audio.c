/*
 * Silicon Image SiI9022 Encoder Driver
 *
 * Copyright (C) 2014 Texas Instruments
 * Author: Jyri Sarha <jsarha@ti.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/regmap.h>

#include <drm/drm_edid.h>

#include <linux/hdmi.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <uapi/sound/asound.h>
#include <sound/asoundef.h>

#include <dt-bindings/sound/sii9022-audio.h>

#include "../dss/omapdss.h"
#include "encoder-sii9022.h"

struct sii9022_audio {
	unsigned int mclk;
	unsigned int fmt;
	unsigned int channels;
	u32 i2s_fifo_routing[4];
};

static int sii9022_mute(struct snd_soc_dai *dai, int mute)
{
	struct panel_drv_data *dd = dev_get_drvdata(dai->dev);
	u8 channel_layout = dd->audio->channels > 2 ?
		TPI_AUDIO_LAYOUT_8_CHANNELS :
		TPI_AUDIO_LAYOUT_2_CHANNELS;
	int ret;

	if (mute) {
		dev_dbg(dai->dev, "Muted (%d)\n", mute);
		ret = regmap_write(dd->regmap,
				   HDMI_TPI_AUDIO_CONFIG_BYTE2_REG,
				   TPI_AUDIO_INTERFACE_I2S |
				   channel_layout |
				   TPI_AUDIO_MUTE_ENABLE |
				   TPI_AUDIO_CODING_PCM);
	} else {
		dev_dbg(dai->dev, "Unmuted (%d)\n", mute);
		ret = regmap_write(dd->regmap,
				   HDMI_TPI_AUDIO_CONFIG_BYTE2_REG,
				   TPI_AUDIO_INTERFACE_I2S |
				   channel_layout |
				   TPI_AUDIO_MUTE_DISABLE |
				   TPI_AUDIO_CODING_PCM);
	}
	return ret;
}

static const unsigned int mclk_div_table[] = {
	128, 256, 384, 512, 768, 1024, 1152, 192 };

static int sii9022_select_mclk_div(u8 *i2s_config_reg,
				   unsigned int rate,
				   unsigned int mclk)
{
	int distance = 100000;
	u8 i, nearest = 0;

	for (i = 0; i < ARRAY_SIZE(mclk_div_table); i++) {
		unsigned int div = mclk/rate;

		if (abs(div - mclk_div_table[i]) < distance) {
			nearest = i;
			distance = abs(div - mclk_div_table[i]);
		}
		if (div == mclk_div_table[i])
			break;
	}

	*i2s_config_reg |= nearest << 4;

	if (i == ARRAY_SIZE(mclk_div_table))
		return mclk_div_table[nearest];

	return 0;
}

static int sii9022_audio_start(struct snd_soc_dai *dai,
			       struct snd_pcm_hw_params *params)
{
	struct panel_drv_data *dd = dev_get_drvdata(dai->dev);
	u8 i2s_config_reg = TPI_I2S_SD_DIRECTION_MSB_FIRST;
	u8 config_byte3_reg = TPI_AUDIO_CHANNEL_STREAM;
	u8 i2s_strm_hdr_reg[5] = {
		IEC958_AES0_CON_NOT_COPYRIGHT,
		IEC958_AES1_CON_GENERAL,
		IEC958_AES2_CON_SOURCE_UNSPEC,
		IEC958_AES3_CON_CLOCK_VARIABLE,
		0
	};
	u8 infoframe_buf[HDMI_INFOFRAME_SIZE(AUDIO)];
	struct hdmi_audio_infoframe infoframe;
	int ret, i;

	hdmi_audio_infoframe_init(&infoframe);

	dd->audio->channels = params_channels(params);

	switch (dd->audio->fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		dev_err(dai->dev, "Unsupported daifmt (0x%x) master\n",
			dd->audio->fmt);
		return -EINVAL;
	}

	switch (dd->audio->fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		i2s_config_reg |=
			TPI_I2S_FIRST_BIT_SHIFT_YES | TPI_I2S_SD_JUSTIFY_LEFT;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		i2s_config_reg |= TPI_I2S_SD_JUSTIFY_LEFT;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		i2s_config_reg |= TPI_I2S_SD_JUSTIFY_RIGHT;
		break;
	default:
		dev_err(dai->dev, "Unsupported daifmt (0x%x) format\n",
			dd->audio->fmt);
		return -EINVAL;
	}

	switch (dd->audio->fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		i2s_config_reg |=
			TPI_I2S_WS_POLARITY_HIGH | TPI_I2S_SCK_EDGE_RISING;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		i2s_config_reg |=
			TPI_I2S_WS_POLARITY_HIGH | TPI_I2S_SCK_EDGE_FALLING;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		i2s_config_reg |=
			TPI_I2S_WS_POLARITY_LOW | TPI_I2S_SCK_EDGE_RISING;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		i2s_config_reg |=
			TPI_I2S_WS_POLARITY_LOW | TPI_I2S_SCK_EDGE_FALLING;
		break;
	}

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		config_byte3_reg |= TPI_AUDIO_SAMPLE_SIZE_16;
		i2s_strm_hdr_reg[4] |= IEC958_AES4_CON_WORDLEN_20_16;
		infoframe.sample_size = HDMI_AUDIO_SAMPLE_SIZE_16;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		config_byte3_reg |= TPI_AUDIO_SAMPLE_SIZE_20;
		i2s_strm_hdr_reg[4] |= IEC958_AES4_CON_WORDLEN_24_20;
		infoframe.sample_size = HDMI_AUDIO_SAMPLE_SIZE_20;
		break;
	case SNDRV_PCM_FORMAT_S24_3LE:
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_S32_LE:
		config_byte3_reg |= TPI_AUDIO_SAMPLE_SIZE_24;
		i2s_strm_hdr_reg[4] |= IEC958_AES4_CON_MAX_WORDLEN_24 |
			IEC958_AES4_CON_WORDLEN_24_20;
		infoframe.sample_size = HDMI_AUDIO_SAMPLE_SIZE_24;
		break;
	default:
		dev_err(dai->dev, "Unsupported sample format %d\n",
			params_format(params));
		return -EINVAL;
	}

	ret = sii9022_select_mclk_div(&i2s_config_reg, params_rate(params),
				      dd->audio->mclk);
	if (ret)
		dev_dbg(dai->dev, "Inaccurate reference clock (%u/%d != %u)\n",
			dd->audio->mclk, ret, params_rate(params));

	switch (params_rate(params)) {
	case 32000:
		i2s_strm_hdr_reg[3] |= IEC958_AES3_CON_FS_32000;
		i2s_strm_hdr_reg[4] |= IEC958_AES4_CON_ORIGFS_32000;
		infoframe.sample_frequency = HDMI_AUDIO_SAMPLE_FREQUENCY_32000;
		break;
	case 44100:
		i2s_strm_hdr_reg[3] |= IEC958_AES3_CON_FS_44100;
		i2s_strm_hdr_reg[4] |= IEC958_AES4_CON_ORIGFS_44100;
		infoframe.sample_frequency = HDMI_AUDIO_SAMPLE_FREQUENCY_44100;
		break;
	case 48000:
		i2s_strm_hdr_reg[3] |= IEC958_AES3_CON_FS_48000;
		i2s_strm_hdr_reg[4] |= IEC958_AES4_CON_ORIGFS_48000;
		infoframe.sample_frequency = HDMI_AUDIO_SAMPLE_FREQUENCY_48000;
		break;
	case 88200:
		i2s_strm_hdr_reg[3] |= IEC958_AES3_CON_FS_88200;
		i2s_strm_hdr_reg[4] |= IEC958_AES4_CON_ORIGFS_88200;
		infoframe.sample_frequency = HDMI_AUDIO_SAMPLE_FREQUENCY_88200;
		break;
	case 96000:
		i2s_strm_hdr_reg[3] |= IEC958_AES3_CON_FS_96000;
		i2s_strm_hdr_reg[4] |= IEC958_AES4_CON_ORIGFS_96000;
		infoframe.sample_frequency = HDMI_AUDIO_SAMPLE_FREQUENCY_96000;
		break;
	case 176400:
		i2s_strm_hdr_reg[3] |= IEC958_AES3_CON_FS_176400;
		i2s_strm_hdr_reg[4] |= IEC958_AES4_CON_ORIGFS_176400;
		infoframe.sample_frequency = HDMI_AUDIO_SAMPLE_FREQUENCY_176400;
		break;
	case 192000:
		i2s_strm_hdr_reg[3] |= IEC958_AES3_CON_FS_192000;
		i2s_strm_hdr_reg[4] |= IEC958_AES4_CON_ORIGFS_192000;
		infoframe.sample_frequency = HDMI_AUDIO_SAMPLE_FREQUENCY_192000;
		break;
	default:
		dev_err(dai->dev, "Unsupported sample rate %d\n",
			params_rate(params));
		return -EINVAL;
	}

	ret = sii9022_mute(dai, true);
	if (ret < 0)
		return ret;

	regmap_write(dd->regmap, HDMI_TPI_I2S_INPUT_CONFIG_REG,
		     i2s_config_reg);

	for (i = 0; i < ARRAY_SIZE(dd->audio->i2s_fifo_routing); i++)
		regmap_write(dd->regmap, HDMI_TPI_I2S_ENABLE_MAPPING_REG,
			     dd->audio->i2s_fifo_routing[i]);

	regmap_write(dd->regmap, HDMI_TPI_AUDIO_CONFIG_BYTE3_REG,
		     config_byte3_reg);

	regmap_bulk_write(dd->regmap, HDMI_TPI_I2S_STRM_HDR_BASE,
			  i2s_strm_hdr_reg, ARRAY_SIZE(i2s_strm_hdr_reg));

	infoframe.channels = params_channels(params);
	infoframe.coding_type = HDMI_AUDIO_CODING_TYPE_PCM;
	ret = hdmi_audio_infoframe_pack(&infoframe, infoframe_buf,
					sizeof(infoframe_buf));
	if (ret < 0) {
		dev_err(dai->dev, "Failed to pack audio infoframe: %d\n",
			ret);
		return ret;
	}
	regmap_bulk_write(dd->regmap, HDMI_CPI_MISC_IF_SELECT_REG,
			  infoframe_buf, ret);

	/* Decode Level 0 Packets */
	regmap_write(dd->regmap, HDMI_IND_SET_PAGE, 0x02);
	regmap_write(dd->regmap, HDMI_IND_OFFSET, 0x24);
	regmap_write(dd->regmap, HDMI_IND_VALUE, 0x02);

	dev_dbg(dai->dev, "hdmi audio enabled\n");

	return 0;
}

static int sii9022_audio_stop(struct snd_soc_dai *dai)
{
	struct panel_drv_data *dd = dev_get_drvdata(dai->dev);
	int ret;

	ret = regmap_write(dd->regmap, HDMI_TPI_AUDIO_CONFIG_BYTE2_REG,
			   TPI_AUDIO_INTERFACE_DISABLE);

	dev_dbg(dai->dev, "hdmi audio disabled (%d)\n", ret);
	return ret;
}

static int sii9022_dai_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct panel_drv_data *dd = dev_get_drvdata(dai->dev);
	int r;

	dev_dbg(dai->dev, "%s: format %d rate %d channels %d\n", __func__,
		params_format(params),
		params_rate(params),
		params_channels(params));

	mutex_lock(&dd->lock);

	r = sii9022_audio_start(dai, params);

	mutex_unlock(&dd->lock);

	return r;
}

static int sii9022_dai_set_sysclk(struct snd_soc_dai *dai,
				  int clk_id, unsigned int freq, int dir)
{
	struct panel_drv_data *dd = dev_get_drvdata(dai->dev);

	dev_dbg(dai->dev, "%s: id %d freq %u\n", __func__, clk_id, freq);
	dd->audio->mclk = freq;

	return 0;
}

static int sii9022_dai_set_fmt(struct snd_soc_dai *dai,
			       unsigned int fmt)
{
	struct panel_drv_data *dd = dev_get_drvdata(dai->dev);

	dev_dbg(dai->dev, "%s: 0x%08x\n", __func__, fmt);
	dd->audio->fmt = fmt;

	return 0;
}

static int sii9022_dai_digital_mute(struct snd_soc_dai *dai, int mute)
{
	struct panel_drv_data *dd = dev_get_drvdata(dai->dev);
	int r;

	mutex_lock(&dd->lock);
	r = sii9022_mute(dai, mute);
	mutex_unlock(&dd->lock);

	return r;
}

static void sii9022_dai_shutdown(struct snd_pcm_substream *s,
				 struct snd_soc_dai *dai)
{
	struct panel_drv_data *dd = dev_get_drvdata(dai->dev);

	dev_dbg(dai->dev, "%s called\n", __func__);

	mutex_lock(&dd->lock);
	sii9022_audio_stop(dai);
	mutex_unlock(&dd->lock);
}

static struct snd_soc_dai_ops sii9022_dai_ops = {
	.hw_params	= sii9022_dai_hw_params,
	.set_sysclk	= sii9022_dai_set_sysclk,
	.set_fmt	= sii9022_dai_set_fmt,
	.digital_mute	= sii9022_dai_digital_mute,
	.shutdown	= sii9022_dai_shutdown,
};

static struct snd_soc_dai_driver sii9022_codec_dai = {
	.name = "hdmi-hifi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_32000 |
			SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |
			SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000 |
			SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_192000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |
			SNDRV_PCM_FMTBIT_S32_LE,
		.sig_bits = 24,
	},
	.ops = &sii9022_dai_ops,
};

static const struct snd_soc_dapm_widget sii9022_widgets[] = {
	SND_SOC_DAPM_OUTPUT("TX"),
};

static const struct snd_soc_dapm_route sii9022_routes[] = {
	{ "TX", NULL, "Playback" },
};

static struct snd_soc_codec_driver sii9022_codec = {
	.dapm_widgets = sii9022_widgets,
	.num_dapm_widgets = ARRAY_SIZE(sii9022_widgets),
	.dapm_routes = sii9022_routes,
	.num_dapm_routes = ARRAY_SIZE(sii9022_routes),
};

static const u8 i2s_fifo_defaults[] = {
	TPI_I2S_CONFIG_FIFO0,
	TPI_I2S_CONFIG_FIFO1,
	TPI_I2S_CONFIG_FIFO2,
	TPI_I2S_CONFIG_FIFO3,
};

int sii9022_hdmi_codec_register(struct device *dev)
{
	struct panel_drv_data *dd = dev_get_drvdata(dev);
	struct device_node *node = dev->of_node;
	int ret, i;
	int fifos_enabled = 0;

	dd->audio = devm_kzalloc(dev, sizeof(dd->audio), GFP_KERNEL);
	if (IS_ERR(dd->audio))
		return PTR_ERR(dd->audio);

	ret = of_property_read_u32_array(
		node, "i2s-fifo-routing", dd->audio->i2s_fifo_routing,
		ARRAY_SIZE(dd->audio->i2s_fifo_routing));
	if (ret != 0) {
		dev_err(dev,
			"Error %d getting \"i2s-fifo-routing\" DT property.\n",
			ret);
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(dd->audio->i2s_fifo_routing); i++) {
		dd->audio->i2s_fifo_routing[i] |= i2s_fifo_defaults[i];
		if (dd->audio->i2s_fifo_routing[i] & TPI_I2S_FIFO_ENABLE)
			fifos_enabled++;
	}

	dev_dbg(dev, "%d fifos enabled, setting max_channels to %d\n",
		fifos_enabled, 2 * fifos_enabled);

	sii9022_codec_dai.playback.channels_max = 2 * fifos_enabled;

	return snd_soc_register_codec(dev, &sii9022_codec,
				      &sii9022_codec_dai, 1);
}

void sii9022_hdmi_codec_unregister(struct device *dev)
{
	snd_soc_unregister_codec(dev);
}
