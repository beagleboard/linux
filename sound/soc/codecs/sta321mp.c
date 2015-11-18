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

#include "sta321mp.h"

#define STA321MP_RATES		(SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_192000)
#define STA321MP_FORMAT		(SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S32_LE)

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
static const DECLARE_TLV_DB_SCALE(mvol_tlv, -12700, 50, 1);
static const DECLARE_TLV_DB_SCALE(chvol_tlv, -7950, 50, 1);

static const struct snd_kcontrol_new sta321mp_snd_controls[] = {
SOC_SINGLE("Master Switch", STA321MP_MMUTE, 0, 1, 1),
SOC_SINGLE_TLV("Master Volume", STA321MP_MVOL, 0, 0xff, 1, mvol_tlv),
/*
SOC_SINGLE("Ch1 Switch", STA321MP_C1VTMB, 7, 1, 1),
SOC_SINGLE("Ch2 Switch", STA321MP_C2VTMB, 7, 1, 1),
SOC_SINGLE("Ch3 Switch", STA321MP_C3VTMB, 7, 1, 1),
SOC_SINGLE("Ch4 Switch", STA321MP_C4VTMB, 7, 1, 1),
SOC_SINGLE("Ch5 Switch", STA321MP_C5VTMB, 7, 1, 1),
SOC_SINGLE("Ch6 Switch", STA321MP_C6VTMB, 7, 1, 1),
SOC_SINGLE_TLV("Ch1 Volume", STA321MP_C1VOL, 0, 0xff, 1, chvol_tlv),
SOC_SINGLE_TLV("Ch2 Volume", STA321MP_C2VOL, 0, 0xff, 1, chvol_tlv),
SOC_SINGLE_TLV("Ch3 Volume", STA321MP_C3VOL, 0, 0xff, 1, chvol_tlv),
SOC_SINGLE_TLV("Ch4 Volume", STA321MP_C4VOL, 0, 0xff, 1, chvol_tlv),
SOC_SINGLE_TLV("Ch5 Volume", STA321MP_C5VOL, 0, 0xff, 1, chvol_tlv),
SOC_SINGLE_TLV("Ch6 Volume", STA321MP_C6VOL, 0, 0xff, 1, chvol_tlv),
*/
SOC_DOUBLE_R("Ch1 Capture Switch", STA321MP_C1VTMB, STA321MP_C2VTMB, 7, 1, 1),
SOC_DOUBLE_R("Ch2 Capture Switch", STA321MP_C3VTMB, STA321MP_C4VTMB, 7, 1, 1),
SOC_DOUBLE_R("Ch3 Capture Switch", STA321MP_C5VTMB, STA321MP_C6VTMB, 7, 1, 1),
SOC_DOUBLE_R_TLV("Ch1 Capture Volume", STA321MP_C1VOL, STA321MP_C2VOL, 0, 0xff, 1, chvol_tlv),
SOC_DOUBLE_R_TLV("Ch2 Capture Volume", STA321MP_C3VOL, STA321MP_C4VOL, 0, 0xff, 1, chvol_tlv),
SOC_DOUBLE_R_TLV("Ch3 Capture Volume", STA321MP_C5VOL, STA321MP_C6VOL, 0, 0xff, 1, chvol_tlv),
};

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

static void sta321mp_write_ram(struct snd_soc_codec *codec, unsigned int address, unsigned int value)
{
	snd_soc_write(codec, 0x3B, (address >> 8) & 0x03 );	/* writing top 2 bits of address */
	snd_soc_write(codec, 0x3C, address & 0x0F );        /* writing bottom 8 bits of address */
	snd_soc_write(codec, 0x3D, 0x0F & (value >> 16) );
	snd_soc_write(codec, 0x3E, 0x0F & (value >> 8) );
	snd_soc_write(codec, 0x3F, 0x0F & value );
	snd_soc_write(codec, 0x4C, 0x01); /* Finished writing 1 value */
}

static void sta321mp_mixer(struct snd_soc_codec *codec, int mix, int ch_out, int ch_in, unsigned int value)
{
  unsigned int address = 0x1A0 + 64*(mix-1) + 8*(ch_out-1) + (ch_in-1);
  sta321mp_write_ram(codec, address, value);
}

static void sta321mp_prescale(struct snd_soc_codec *codec, int ch, unsigned int val)
{
  unsigned int address = 0x190 + (ch-1);
  sta321mp_write_ram(codec, address, val);
}

static void sta321mp_postscale(struct snd_soc_codec *codec, int ch, unsigned int val)
{
  unsigned int address = 0x198 + (ch-1);
  sta321mp_write_ram(codec, address, val);
}

/* set the biquad coefficients in RAM */
static void sta321mp_biquad(struct snd_soc_codec *codec, int channel, unsigned int biquad, 
                            unsigned int b1_2, unsigned int b2, unsigned int a1_2, unsigned int a2, unsigned int b0_2)
{
  unsigned int address = 50*(channel-1) + 5*(biquad-1);

	snd_soc_write(codec, 0x3b, (address >> 8) & 0x03 );	/* writing top 2 bits of address */
	snd_soc_write(codec, 0x3c, address & 0xFF );        /* writing bottom 8 bits of address */
	snd_soc_write(codec, 0x3d, 0xFF & (b1_2 >> 16) );
	snd_soc_write(codec, 0x3e, 0xFF & (b1_2 >> 8) );
	snd_soc_write(codec, 0x3f, 0xFF & b1_2 );
	snd_soc_write(codec, 0x40, 0xFF & (b2 >> 16) );
	snd_soc_write(codec, 0x41, 0xFF & (b2 >> 8) );
	snd_soc_write(codec, 0x42, 0xFF & b2 );
	snd_soc_write(codec, 0x43, 0xFF & (a1_2 >> 16) );
	snd_soc_write(codec, 0x44, 0xFF & (a1_2 >> 8) );
	snd_soc_write(codec, 0x45, 0xFF & a1_2 );
	snd_soc_write(codec, 0x46, 0xFF & (a2 >> 16) );
	snd_soc_write(codec, 0x47, 0xFF & (a2 >> 8) );
	snd_soc_write(codec, 0x48, 0xFF & a2 );
	snd_soc_write(codec, 0x49, 0xFF & (b0_2 >> 16) );
	snd_soc_write(codec, 0x4A, 0xFF & (b0_2 >> 8) );
	snd_soc_write(codec, 0x4B, 0xFF & b0_2 );
	snd_soc_write(codec, 0x4C, 0x02); /* Finished writing ALL values */

}

static int sta321mp_LP_192kHz(struct snd_soc_codec *codec)
{
  /*
   * This function sets the biquads as a 9th order butterworth
   * Low pass filter with cutoff at 40kHz
   */
  sta321mp_biquad(codec, 1, 2, 0x34417a, 0x34417a, 0x10e211, 0xfd899c,0x1a20bd);
  sta321mp_biquad(codec, 1, 3, 0x1be34f, 0x1be34f, 0x112371, 0xfb9073,0xdf1a7);
  sta321mp_biquad(codec, 1, 4, 0x1be34f, 0x1be34f, 0x11aa29, 0xf77f69,0xdf1a7);
  sta321mp_biquad(codec, 1, 5, 0x1be34f, 0x1be34f, 0x127eb2, 0xf1150f,0xdf1a7);
  sta321mp_biquad(codec, 1, 6, 0x1be34f, 0x1be34f, 0x13af2a, 0xe7e44f,0xdf1a7);
  sta321mp_biquad(codec, 1, 7, 0x1be34f, 0x1be34f, 0x155167, 0xdb446c,0xdf1a7);
  sta321mp_biquad(codec, 1, 8, 0x1be34f, 0x1be34f, 0x17867e, 0xca35c0,0xdf1a7);
  sta321mp_biquad(codec, 1, 9, 0x1be34f, 0x1be34f, 0x1a80c9, 0xb33334,0xdf1a7);
  sta321mp_biquad(codec, 1, 10, 0x1be34f, 0x1be34f, 0x1e8e71, 0x93e0cf,0xdf1a7);

  return 0;
}

static int sta321mp_LP_48kHz(struct snd_soc_codec *codec)
{

  sta321mp_biquad(codec, 1, 2, 0x2d7d5e, 0x3faa84, 0x6599c4, 0xad9ed1,0x1fd542);
  sta321mp_biquad(codec, 1, 3, 0xfcec16, 0x177b7b, 0x63c973, 0xa4b499,0xbbdbd);
  sta321mp_biquad(codec, 1, 4, 0xf4caac, 0x177b7b, 0x616997, 0x98a93a,0xbbdbd);
  sta321mp_biquad(codec, 1, 5, 0xf1c0e3, 0x177b7b, 0x5f9823, 0x8e691f,0xbbdbd);
  sta321mp_biquad(codec, 1, 6, 0xf083cb, 0x177b7b, 0x5ec1ab, 0x873641,0xbbdbd);
  sta321mp_biquad(codec, 1, 7, 0xf00d82, 0x177b7b, 0x5f0547, 0x822c2a,0xbbdbd);
  sta321mp_biquad(codec, 1, 8, 0x0, 0x0, 0x0, 0x0, 0x400000);
  sta321mp_biquad(codec, 1, 9, 0x0, 0x0, 0x0, 0x0, 0x400000);
  sta321mp_biquad(codec, 1, 10, 0x0, 0x0, 0x0, 0x0, 0x400000);

  return 0;
}

static int sta321mp_set_bits(struct snd_soc_codec *codec)
{

  snd_soc_write(codec, STA321MP_CONFA, 0x9b);

  snd_soc_write(codec, STA321MP_CONFE, 0x0c);
  snd_soc_write(codec, STA321MP_CONFF, 0x18); /* use all biquad identical to channel 1 */
  snd_soc_write(codec, STA321MP_CONFH, 0x7c);
  snd_soc_write(codec, STA321MP_CONFI, 0x80);
  snd_soc_write(codec, STA321MP_MVOL, 0x00);

  snd_soc_write(codec, STA321MP_C1VOL, 0x36);
  snd_soc_write(codec, STA321MP_C2VOL, 0x36);
  snd_soc_write(codec, STA321MP_C3VOL, 0x36);
  snd_soc_write(codec, STA321MP_C4VOL, 0x36);
  snd_soc_write(codec, STA321MP_C5VOL, 0x36);
  snd_soc_write(codec, STA321MP_C6VOL, 0x36);
  snd_soc_write(codec, STA321MP_C7VOL, 0x36);
  snd_soc_write(codec, STA321MP_C8VOL, 0x36);

  /*
  snd_soc_write(codec, STA321MP_C12IM, 0x10);
  snd_soc_write(codec, STA321MP_C34IM, 0x32);
  snd_soc_write(codec, STA321MP_C56IM, 0x54);
  snd_soc_write(codec, STA321MP_C78IM, 0x76);
  */

  snd_soc_write(codec, 0x2b, 0xFF); /* Tone bypass */

  snd_soc_write(codec, 0x5d, 0x01); /* microphone mode, sets PDM clock */
  snd_soc_write(codec, 0x81, 0x09); /* I2S out normal, no pop-up removal */

  // set pwm output
  sta321mp_mixer(codec, 1, 7, 1, 0x7FFFFF); /* Setting channel 7, Mixer 1, channel 1 on  */
  sta321mp_mixer(codec, 1, 7, 7, 0x000000); /* Setting channel 7, Mixer 1, channel 7 off */
  sta321mp_mixer(codec, 1, 8, 2, 0x7FFFFF); /* Setting channel 8, Mixer 1, channel 2 on  */
  sta321mp_mixer(codec, 1, 8, 8, 0x000000); /* Setting channel 8, Mixer 1, channel 8 off */

  return 0;
}

static int sta321mp_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params,
		struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
  int fmt = 0x00; // format register


  // configure the output format
	switch (params_format(params)) {
  case SNDRV_PCM_FORMAT_S32_LE:
  case SNDRV_PCM_FORMAT_S24_LE:
    fmt |= I2S_S24_LE;
		break;
  case SNDRV_PCM_FORMAT_S24_3LE:
    fmt |= I2S_S24_3LE;
		break;
  case SNDRV_PCM_FORMAT_S16_LE:
    fmt |= I2S_S16_LE;
    break;
	default:
		dev_err(codec->dev, "Unsupported format\n");
		return -EINVAL;
	}
  fmt |= I2S_MSB_1ST;

  // configure the output rate
	switch (params_rate(params)) {
  case 192000:
    sta321mp_LP_192kHz(codec);
    fmt |= I2S_DIV_1;
    break;
	case 48000:
    sta321mp_LP_48kHz(codec);
    fmt |= I2S_DIV_4;
		break;
	default:
		dev_err(codec->dev, "Unsupported rate\n");
		return -EINVAL;
	}

  // write configuration to sta321mp
  snd_soc_write(codec, STA321MP_CONFC, fmt);

  // Initialize sta321mp
  sta321mp_set_bits(codec);

  printk("sta321mp: Started at %d Hz.\n", params_rate(params));

	return 0;
}

static const struct snd_soc_dai_ops sta321mp_dai_ops = {
	.hw_params	=	sta321mp_hw_params,
};

static struct snd_soc_dai_driver sta321mp_dai = {
	.name = "sta321mp-audio",
	.capture = {
		.stream_name = "Capture",
		.channels_min = 2,
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

  sta321mp_add_widgets(codec);

	return 0;
}

/* power down chip */
static int sta321mp_remove(struct snd_soc_codec *codec)
{
	return 0;
}

static int sta321mp_suspend(struct snd_soc_codec *codec)
{
	return 0;
}

static int sta321mp_resume(struct snd_soc_codec *codec)
{
	return 0;
}

static const struct snd_soc_codec_driver sta321mp_codec_driver = {
	.probe = sta321mp_probe,
	.remove = sta321mp_remove,
	.suspend = sta321mp_suspend,
	.resume = sta321mp_resume,
	.controls = sta321mp_snd_controls,
	.num_controls = ARRAY_SIZE(sta321mp_snd_controls),
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
