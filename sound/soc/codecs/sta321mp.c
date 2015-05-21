/*
 * ASoC codec driver for sta321mp
 *
 * sound/soc/codecs/sta321mp.c -- ALSA SoC sta321mp codec driver
 *
 * Copyright (C) 2014 Robin Scheibler <fakufaku@gmail.com>
 *
 * Based on sound/soc/codecs/sta529.c by Rajeev Kumar
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pm.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>

/* STA321MP Register offsets */
#define	 STA321MP_CONFA		0x00
#define	 STA321MP_CONFC		0x02
#define	 STA321MP_CONFD		0x03
#define	 STA321MP_CONFE		0x04
#define	 STA321MP_CONFF		0x05
#define	 STA321MP_CONFG		0x06
#define	 STA321MP_CONFH		0x07
#define	 STA321MP_CONFI		0x08

#define	 STA321MP_MMUTE		0x09
#define	 STA321MP_MVOL		0x0A
#define	 STA321MP_C1VOL		0x0B
#define	 STA321MP_C2VOL		0x0C
#define	 STA321MP_C3VOL		0x0D
#define	 STA321MP_C4VOL		0x0E
#define	 STA321MP_C5VOL		0x0F
#define	 STA321MP_C6VOL		0x10
#define	 STA321MP_C7VOL		0x11
#define	 STA321MP_C8VOL		0x12

#define  STA321MP_C1VTMB	0x13
#define  STA321MP_C2VTMB	0x14
#define  STA321MP_C3VTMB	0x15
#define  STA321MP_C4VTMB	0x16
#define  STA321MP_C5VTMB	0x17
#define  STA321MP_C6VTMB	0x18
#define  STA321MP_C7VTMB	0x19
#define  STA321MP_C8VTMB	0x1A

#define  STA321MP_C12IM		0x1B
#define  STA321MP_C34IM		0x1C
#define  STA321MP_C56IM		0x1D
#define  STA321MP_C78IM		0x1E

#define  STA321MP_BQIP		0x28
#define  STA321MP_MXIP		0x29
#define  STA321MP_EQBP		0x2A
#define  STA321MP_TONEBP	0x2B
#define  STA321MP_TONE		0x2C

#define  STA321MP_C12OT		0x33
#define  STA321MP_C34OT		0x34
#define  STA321MP_C56OT		0x35
#define  STA321MP_C78OT		0x36

#define  STA321MP_C12OM		0x37
#define  STA321MP_C34OM		0x38
#define  STA321MP_C56OM		0x39
#define  STA321MP_C78OM		0x3A

#define  STA321MP_MRBIST	0x5C
#define  STA321MP_RCTR1		0x5D
#define  STA321MP_PDMCT		0x5E
#define  STA321MP_RCTR2		0x5F
#define  STA321MP_RCTR3		0x60
#define  STA321MP_RCTR4		0x61
#define  STA321MP_RCTR5		0x62
#define  STA321MP_RCTR6		0x63
#define  STA321MP_RCTR7		0x64
#define  STA321MP_RCTR8		0x65
#define  STA321MP_RCTR9		0x66
#define  STA321MP_RCTR10	0x67
#define  STA321MP_RCTR11	0x68
#define  STA321MP_RCTR12	0x69
#define  STA321MP_RCTR13	0x6A

#define  STA321MP_DPT		0x80
#define  STA321MP_CFR129	0x81
#define  STA321MP_TSDLY1	0x82
#define  STA322MP_TSDLY1	0x83



#define STA321MP_MAX_REGISTER	0x83

#define STA321MP_RATES		(SNDRV_PCM_RATE_44100)

#define STA321MP_FORMAT		(SNDRV_PCM_FMTBIT_S24_LE)

#define PDM_I_EN		0x9B
#define FS_XTI_256		0x20
#define CH78_BIN		0xC0
#define RM_SOFT_VOL		0x7A
#define BRG_PWR_UP		0x80
#define MST_VOL_0DB		0x00
#define MIC_MODE		0x01
#define I2S_OUT			0x09

#define AUDIO_MUTE_MSK		0x01
#define DATA_FORMAT_MSK		0x0F

#define I2S_DATA_FORMAT		0x00
#define CODEC_MUTE_VAL    0x01

static const struct reg_default sta321mp_reg_defaults[] = {
	{ STA321MP_CONFA,  PDM_I_EN },       /* R0  0x9B - PDM interface enable */
	{ STA321MP_CONFC,  FS_XTI_256 },     /* R2  0x20 - FS = XTI/256 */
	{ STA321MP_CONFE,  CH78_BIN },       /* R4  0xC0 - Ch4/5 binary */
	{ STA321MP_CONFH,  RM_SOFT_VOL },    /* R7  0x7A - Remove soft volume */
	{ STA321MP_CONFI,  BRG_PWR_UP },     /* R8  0x80 - Bridge power-up */
	{ STA321MP_MVOL,   MST_VOL_0DB },    /* RA  0x00 - Master volume 0 dB */
	{ STA321MP_C1VOL,  0x36 },     	     /* RB  0x36 - Ch1: +30dB */
	{ STA321MP_C2VOL,  0x36 },     	     /* RC  0x36 - Ch2: +30dB */
	{ STA321MP_C3VOL,  0x36 },     	     /* RD  0x36 - Ch3: +30dB */
	{ STA321MP_C4VOL,  0x36 },     	     /* RE  0x36 - Ch4: +30dB */
	{ STA321MP_C5VOL,  0x36 },     	     /* RF  0x36 - Ch5: +30dB */
	{ STA321MP_C6VOL,  0x36 },     	     /* R10 0x36 - Ch6: +30dB */
	{ 0x3b, 0x01 },	 /* Setting channel 7, Mixer 1, channe 1 on   */
	{ 0x3c, 0xD0 },  /* |                                         */
	{ 0x3d, 0x7F },  /* |                                         */
	{ 0x3e, 0xFF },  /* |                                         */
	{ 0x3f, 0xFF },  /* |                                         */
	{ 0x4c, 0x01 },  /* =                                         */
	{ 0x3b, 0x01 },  /* Setting channel 7, Mixer 1, channel 7 off */
	{ 0x3c, 0xD6 },  /* |                                         */
	{ 0x3d, 0x00 },  /* |                                         */
	{ 0x3e, 0x00 },  /* |                                         */
	{ 0x3f, 0x00 },  /* |                                         */
	{ 0x4c, 0x01 },  /* =                                         */
	{ 0x3b, 0x01 },  /* Setting channel 8, Mixer 1, channel 2 on  */
	{ 0x3c, 0xD9 },  /* |                                         */
	{ 0x3d, 0x7F },  /* |                                         */
	{ 0x3e, 0xFF },  /* |                                         */
	{ 0x3f, 0xFF },  /* |                                         */
	{ 0x4c, 0x01 },  /* =                                         */
	{ 0x3b, 0x01 },  /* Setting channel 8, Mixer 1, channel 8 off */
	{ 0x3c, 0xDF },  /* |                                         */
	{ 0x3d, 0x00 },  /* |                                         */
	{ 0x3e, 0x00 },  /* |                                         */
	{ 0x3f, 0x00 },  /* |                                         */
	{ 0x4c, 0x01 },  /* =                                         */
	{ STA321MP_RCTR1,  MIC_MODE },       /* R5D 0x01 - Microphone mode */
	{ STA321MP_CFR129,  I2S_OUT },       /* R81 0x09 - Output I2S i/f pins set as output */
};

struct sta321mp {
	struct regmap *regmap;
};

static bool sta321mp_readable(struct device *dev, unsigned int reg)
{
	switch (reg) {

	case STA321MP_CONFA:
	case STA321MP_CONFC:
	case STA321MP_CONFE:
	case STA321MP_CONFH:
	case STA321MP_CONFI:
	case STA321MP_MVOL:
	case STA321MP_C1VOL:
	case STA321MP_C2VOL:
	case STA321MP_C3VOL:
	case STA321MP_C4VOL:
	case STA321MP_C5VOL:
	case STA321MP_C6VOL:
	case STA321MP_C78IM:
	case STA321MP_EQBP:
	case STA321MP_TONEBP:
	case STA321MP_RCTR1:
	case STA321MP_CFR129:
	case 0x3d:
	case 0x3e:
	case 0x3f:
	case 0x4c:
		return true;
	default:
		return false;
	}
}

static const struct snd_soc_dapm_widget sta321mp_dapm_widgets[] = {
	SND_SOC_DAPM_INPUT("MIC1"),
	SND_SOC_DAPM_INPUT("MIC2"),
	SND_SOC_DAPM_INPUT("MIC3"),
	SND_SOC_DAPM_INPUT("MIC4"),
	SND_SOC_DAPM_INPUT("MIC5"),
	SND_SOC_DAPM_INPUT("MIC6"),
};

static int sta321mp_add_widgets(struct snd_soc_codec *codec)
{
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	snd_soc_dapm_new_controls(dapm, sta321mp_dapm_widgets,
				  ARRAY_SIZE(sta321mp_dapm_widgets));

	/* set up audio path interconnects */
	//snd_soc_dapm_add_routes(dapm, intercon, ARRAY_SIZE(intercon));

	return 0;
}

static int sta321mp_set_bits(struct snd_soc_codec *codec)
{

  snd_soc_write(codec, 0x00, 0x9b);
  snd_soc_write(codec, 0x02, 0x20);
  snd_soc_write(codec, 0x04, 0x0c);
  snd_soc_write(codec, 0x07, 0x7a);
  snd_soc_write(codec, 0x08, 0x80);
  snd_soc_write(codec, 0x0a, 0x00);
  snd_soc_write(codec, 0x0b, 0x36);
  snd_soc_write(codec, 0x0c, 0x36);
  snd_soc_write(codec, 0x0d, 0x36);
  snd_soc_write(codec, 0x0e, 0x36);
  snd_soc_write(codec, 0x0f, 0x36);
  snd_soc_write(codec, 0x10, 0x36);
  snd_soc_write(codec, 0x11, 0x36);
  snd_soc_write(codec, 0x12, 0x36);
  snd_soc_write(codec, 0x2a, 0xFF);
  snd_soc_write(codec, 0x2b, 0xFF);
  snd_soc_write(codec, 0x5d, 0x01);
  snd_soc_write(codec, 0x81, 0x09);

  return 0;
}

static int sta321mp_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S24_LE:
		break;
	default:
		dev_err(codec->dev, "Unsupported format\n");
		return -EINVAL;
	}

	switch (params_rate(params)) {
	case 44100:
		break;
	default:
		dev_err(codec->dev, "Unsupported rate\n");
		return -EINVAL;
	}

  printk("sta321mp_hw_params: setting codec bits.\n");
  sta321mp_set_bits(codec);

	return 0;
}

static int sta321mp_mute(struct snd_soc_dai *dai, int mute)
{
	u8 val = 0;

	if (mute)
		val |= CODEC_MUTE_VAL;

	snd_soc_update_bits(dai->codec, STA321MP_MMUTE, AUDIO_MUTE_MSK, val);

	return 0;
}

static int sta321mp_set_dai_fmt(struct snd_soc_dai *codec_dai, u32 fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u8 mode = 0;

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		mode = I2S_DATA_FORMAT;
		break;
	default:
		return -EINVAL;
	}

	snd_soc_update_bits(codec, STA321MP_CONFC, DATA_FORMAT_MSK, mode);

	return 0;
}

static const struct snd_soc_dai_ops sta321mp_dai_ops = {
	.hw_params	=	sta321mp_hw_params,
	//.set_fmt	=	sta321mp_set_dai_fmt,
	//.digital_mute	=	sta321mp_mute,
};

static struct snd_soc_dai_driver sta321mp_dai = {
	.name = "sta321mp-audio",
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 6,
		.rates = STA321MP_RATES,
		.formats = STA321MP_FORMAT,
	},
	.ops	= &sta321mp_dai_ops,
};

static int sta321mp_probe(struct snd_soc_codec *codec)
{
	struct sta321mp *sta321mp = snd_soc_codec_get_drvdata(codec);
	int ret;

	codec->control_data = sta321mp->regmap;
	ret = snd_soc_codec_set_cache_io(codec, 8, 8, SND_SOC_REGMAP);

	if (ret < 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}
	//sta321mp_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

  /* set the update bits */
#if 1
	snd_soc_update_bits(codec, STA321MP_CONFA,  0xFF, PDM_I_EN );       /* R0  0x9B - PDM interface enable */
	snd_soc_update_bits(codec, STA321MP_CONFC,  0xFF, FS_XTI_256 );     /* R2  0x20 - FS = XTI/256 */
	snd_soc_update_bits(codec, STA321MP_CONFE,  0xFF, CH78_BIN );       /* R4  0xC0 - Ch4/5 binary */
	snd_soc_update_bits(codec, STA321MP_CONFH,  0xFF, RM_SOFT_VOL );    /* R7  0x7A - Remove soft volume */
	snd_soc_update_bits(codec, STA321MP_CONFI,  0xFF, BRG_PWR_UP );     /* R8  0x80 - Bridge power-up */
	snd_soc_update_bits(codec, STA321MP_MVOL,   0xFF, MST_VOL_0DB );    /* RA  0x00 - Master volume 0 dB */
	snd_soc_update_bits(codec, STA321MP_C1VOL,  0xFF, 0x36 );     	     /* RB  0x36 - Ch1: +30dB */
	snd_soc_update_bits(codec, STA321MP_C2VOL,  0xFF, 0x36 );     	     /* RC  0x36 - Ch2: +30dB */
	snd_soc_update_bits(codec, STA321MP_C3VOL,  0xFF, 0x36 );     	     /* RD  0x36 - Ch3: +30dB */
	snd_soc_update_bits(codec, STA321MP_C4VOL,  0xFF, 0x36 );     	     /* RE  0x36 - Ch4: +30dB */
	snd_soc_update_bits(codec, STA321MP_C5VOL,  0xFF, 0x36 );     	     /* RF  0x36 - Ch5: +30dB */
	snd_soc_update_bits(codec, STA321MP_C6VOL,  0xFF, 0x36 );     	     /* R10 0x36 - Ch6: +30dB */
	snd_soc_update_bits(codec, 0x3b, 0xFF, 0x01 );	/* Setting channel 7, Mixer 1, channel 1 on   */
	snd_soc_update_bits(codec, 0x3c, 0xFF, 0xD0 );  /* |                                         */
	snd_soc_update_bits(codec, 0x3d, 0xFF, 0x7F );  /* |                                         */
	snd_soc_update_bits(codec, 0x3e, 0xFF, 0xFF );  /* |                                         */
	snd_soc_update_bits(codec, 0x3f, 0xFF, 0xFF );  /* |                                         */
	snd_soc_update_bits(codec, 0x4c, 0xFF, 0x01 );  /* =                                         */
	snd_soc_update_bits(codec, 0x3b, 0xFF, 0x01 );  /* Setting channel 7, Mixer 1, channel 7 off */
	snd_soc_update_bits(codec, 0x3c, 0xFF, 0xD6 );  /* |                                         */
	snd_soc_update_bits(codec, 0x3d, 0xFF, 0x00 );  /* |                                         */
	snd_soc_update_bits(codec, 0x3e, 0xFF, 0x00 );  /* |                                         */
	snd_soc_update_bits(codec, 0x3f, 0xFF, 0x00 );  /* |                                         */
	snd_soc_update_bits(codec, 0x4c, 0xFF, 0x01 );  /* =                                         */
	snd_soc_update_bits(codec, 0x3b, 0xFF, 0x01 );  /* Setting channel 8, Mixer 1, channel 2 on  */
	snd_soc_update_bits(codec, 0x3c, 0xFF, 0xD9 );  /* |                                         */
	snd_soc_update_bits(codec, 0x3d, 0xFF, 0x7F );  /* |                                         */
	snd_soc_update_bits(codec, 0x3e, 0xFF, 0xFF );  /* |                                         */
	snd_soc_update_bits(codec, 0x3f, 0xFF, 0xFF );  /* |                                         */
	snd_soc_update_bits(codec, 0x4c, 0xFF, 0x01 );  /* =                                         */
	snd_soc_update_bits(codec, 0x3b, 0xFF, 0x01 );  /* Setting channel 8, Mixer 1, channel 8 off */
	snd_soc_update_bits(codec, 0x3c, 0xFF, 0xDF );  /* |                                         */
	snd_soc_update_bits(codec, 0x3d, 0xFF, 0x00 );  /* |                                         */
	snd_soc_update_bits(codec, 0x3e, 0xFF, 0x00 );  /* |                                         */
	snd_soc_update_bits(codec, 0x3f, 0xFF, 0x00 );  /* |                                         */
	snd_soc_update_bits(codec, 0x4c, 0xFF, 0x01 );  /* =                                         */
	snd_soc_update_bits(codec, STA321MP_RCTR1,  0xFF, MIC_MODE );       /* R5D 0x01 - Microphone mode */
	snd_soc_update_bits(codec, STA321MP_CFR129,  0xFF, I2S_OUT );       /* R81 0x09 - Output I2S i/f pins set as output */
#else
  snd_soc_write(codec, 0x00, 0x9b);
  snd_soc_write(codec, 0x02, 0x20);
  snd_soc_write(codec, 0x04, 0x0c);
  snd_soc_write(codec, 0x07, 0x7a);
  snd_soc_write(codec, 0x08, 0x80);
  snd_soc_write(codec, 0x0a, 0x00);
  snd_soc_write(codec, 0x0b, 0x36);
  snd_soc_write(codec, 0x0c, 0x36);
  snd_soc_write(codec, 0x0d, 0x36);
  snd_soc_write(codec, 0x0e, 0x36);
  snd_soc_write(codec, 0x0f, 0x36);
  snd_soc_write(codec, 0x10, 0x36);
  snd_soc_write(codec, 0x11, 0x36);
  snd_soc_write(codec, 0x12, 0x36);
  snd_soc_write(codec, 0x2a, 0xFF);
  snd_soc_write(codec, 0x2b, 0xFF);
  snd_soc_write(codec, 0x5d, 0x01);
  snd_soc_write(codec, 0x81, 0x09);
#endif


  sta321mp_add_widgets(codec);

	return 0;
}

/* power down chip */
static int sta321mp_remove(struct snd_soc_codec *codec)
{
	//sta321mp_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

static int sta321mp_suspend(struct snd_soc_codec *codec)
{
	//sta321mp_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

static int sta321mp_resume(struct snd_soc_codec *codec)
{
	//sta321mp_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	return 0;
}

static const struct snd_soc_codec_driver sta321mp_codec_driver = {
	.probe = sta321mp_probe,
	.remove = sta321mp_remove,
	//.set_bias_level = sta321mp_set_bias_level,
	.suspend = sta321mp_suspend,
	.resume = sta321mp_resume,
	//.controls = sta321mp_snd_controls,
	//.num_controls = ARRAY_SIZE(sta321mp_snd_controls),
};

static const struct regmap_config sta321mp_regmap = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = STA321MP_MAX_REGISTER,
	.readable_reg = sta321mp_readable,

	.cache_type = REGCACHE_RBTREE,
	.reg_defaults = sta321mp_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(sta321mp_reg_defaults),
};

static int sta321mp_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct sta321mp *sta321mp;
	int ret;

  printk("sta321mp: probing i2c\n");

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EINVAL;

	sta321mp = devm_kzalloc(&i2c->dev, sizeof(struct sta321mp), GFP_KERNEL);
	if (sta321mp == NULL) {
		dev_err(&i2c->dev, "Can not allocate memory\n");
		return -ENOMEM;
	}

  printk("sta321mp: Setting up device through i2c.\n");

	sta321mp->regmap = devm_regmap_init_i2c(i2c, &sta321mp_regmap);
	if (IS_ERR(sta321mp->regmap)) {
		ret = PTR_ERR(sta321mp->regmap);
		dev_err(&i2c->dev, "Failed to allocate regmap: %d\n", ret);
		return ret;
	}

	i2c_set_clientdata(i2c, sta321mp);

	ret = snd_soc_register_codec(&i2c->dev,
			&sta321mp_codec_driver, &sta321mp_dai, 1);
	if (ret != 0)
		dev_err(&i2c->dev, "Failed to register CODEC: %d\n", ret);

	return ret;
}

static int sta321mp_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);

	return 0;
}

static const struct i2c_device_id sta321mp_i2c_id[] = {
	{ "ti,sta321mp", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sta321mp_i2c_id);

#if defined(CONFIG_OF)
static const struct of_device_id sta321mp_of_match[] = {
	{ .compatible = "ti,sta321mp", },
	{},
};
MODULE_DEVICE_TABLE(of, sta321mp_of_match);
#endif


static struct i2c_driver sta321mp_i2c_driver = {
	.driver = {
		.name = "sta321mp-codec",
		.owner = THIS_MODULE,
    .of_match_table = of_match_ptr(sta321mp_of_match),
	},
	.probe		= sta321mp_i2c_probe,
	.remove		= sta321mp_i2c_remove,
	.id_table	= sta321mp_i2c_id,
};

module_i2c_driver(sta321mp_i2c_driver);

MODULE_DESCRIPTION("ASoC STA321MP codec driver");
MODULE_AUTHOR("Robin Scheibler <fakufaku@gmail.com>");
MODULE_LICENSE("GPL");
