/*
 * es8156.c -- es8156 ALSA SoC audio driver
 * Copyright Everest Semiconductor Co.,Ltd
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/tlv.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <linux/proc_fs.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/regmap.h>
#include "es8156.h"

#define INVALID_GPIO -1
#define GPIO_LOW  0
#define GPIO_HIGH 1
#define es8156_DEF_VOL			0xBF
/*
* If your system doesn't have MCLK, define this to 0 or the
* driver will crash.
*/
#define MCLK 0

static struct snd_soc_component *es8156_codec;

static const struct reg_default es8156_reg_defaults[] = {
	{0x00, 0x1c}, {0x01, 0x20}, {0x02, 0x00}, {0x03, 0x01},
	{0x04, 0x00}, {0x05, 0x04}, {0x06, 0x11}, {0x07, 0x00},
	{0x08, 0x06}, {0x09, 0x00}, {0x0a, 0x50}, {0x0b, 0x50},
	{0x0c, 0x00}, {0x0d, 0x10}, {0x10, 0x40}, {0x10, 0x40},
	{0x11, 0x00}, {0x12, 0x04}, {0x13, 0x11}, {0x14, 0xbf},
	{0x15, 0x00}, {0x16, 0x00}, {0x17, 0xf7}, {0x18, 0x00},
	{0x19, 0x20}, {0x1a, 0x00}, {0x20, 0x16}, {0x21, 0x7f},
	{0x22, 0x00}, {0x23, 0x86}, {0x24, 0x00}, {0x25, 0x07},
	{0xfc, 0x00}, {0xfd, 0x81}, {0xfe, 0x55}, {0xff, 0x10},
};

/* codec private data */
struct es8156_priv {
	struct regmap *regmap;
	unsigned int dmic_amic;
	unsigned int sysclk;
	struct snd_pcm_hw_constraint_list *sysclk_constraints;
	struct clk *mclk;
	int debounce_time;
	int hp_det_invert;
	struct delayed_work work;

	int spk_ctl_gpio;
	int hp_det_gpio;
	bool muted;
	bool hp_inserted;
	bool spk_active_level;

	int pwr_count;
};

/*
 * es8156_reset
 */
static int es8156_reset(struct snd_soc_component *codec)
{
	snd_soc_component_write(codec, ES8156_RESET_REG00, 0x1c);
	usleep_range(5000, 5500);
	return snd_soc_component_write(codec, ES8156_RESET_REG00, 0x03);
}

static void es8156_enable_spk(struct es8156_priv *es8156, bool enable)
{
	bool level;

	level = enable ? es8156->spk_active_level : !es8156->spk_active_level;
	gpio_set_value(es8156->spk_ctl_gpio, level);
}


static const char *es8156_DAC_SRC[] = { "Left to Left, Right to Right",
"Right to both Left and Right","Left to both Left & Right", "Left to Right, Right to Left" };

static const DECLARE_TLV_DB_SCALE(dac_vol_tlv, -9600, 50, 1);
static const DECLARE_TLV_DB_SCALE(alc_gain_tlv,-2800,400,1);
static SOC_ENUM_SINGLE_DECL(es8165_dac_enum, ES8156_MISC_CONTROL3_REG18, 4, es8156_DAC_SRC);

static const struct snd_kcontrol_new es8156_DAC_MUX =SOC_DAPM_ENUM("Route", es8165_dac_enum);

static const struct snd_kcontrol_new es8156_snd_controls[] = {
	SOC_SINGLE("Timer 1",ES8156_TIME_CONTROL1_REG0A,0,63,0),
	SOC_SINGLE("Timer 2",ES8156_TIME_CONTROL2_REG0B,0,63,0),
	SOC_SINGLE("DAC Automute Gate",ES8156_AUTOMUTE_SET_REG12,4,15,0),
	SOC_SINGLE_TLV("ALC Gain",ES8156_ALC_CONFIG1_REG15,1,7,1,alc_gain_tlv),
	SOC_SINGLE("ALC Ramp Rate",ES8156_ALC_CONFIG2_REG16,4,15,0),
	SOC_SINGLE("ALC Window Size",ES8156_ALC_CONFIG2_REG16,0,15,0),
	SOC_DOUBLE("ALC Maximum Minimum Volume",ES8156_ALC_CONFIG3_REG17,
	4,0,15,0),
	/* DAC Digital controls */
	SOC_SINGLE_TLV("DAC Playback Volume", ES8156_VOLUME_CONTROL_REG14,
			  0, 0xff, 0, dac_vol_tlv),
	SOC_SINGLE("HP Switch",ES8156_ANALOG_SYS3_REG22,3,1,0),


};


static const struct snd_soc_dapm_widget es8156_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_OUT("SDOUT", "I2S Capture", 0,
			     ES8156_P2S_CONTROL_REG0D, 2, 0),

	SND_SOC_DAPM_AIF_IN("SDIN", "I2S Playback", 0,
			    SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_MUX("Channel Select Mux", SND_SOC_NOPM, 0, 0,
			 &es8156_DAC_MUX),

	SND_SOC_DAPM_DAC("DACL", "Left Playback", ES8156_DAC_MUTE_REG13, 1, 1),
	SND_SOC_DAPM_DAC("DACR", "Right Playback", ES8156_DAC_MUTE_REG13, 2, 1),

	
	SND_SOC_DAPM_PGA("SDOUT TRISTATE", ES8156_P2S_CONTROL_REG0D,
			 0, 1, NULL, 0),

	SND_SOC_DAPM_OUTPUT("SDOUT"),
	SND_SOC_DAPM_OUTPUT("LOUT"),
	SND_SOC_DAPM_OUTPUT("ROUT"),
};

static const struct snd_soc_dapm_route es8156_dapm_routes[] = {
	{"SDOUT TRISTATE",NULL,"SDIN"},
	{"SDOUT",NULL,"SDOUT TRISTATE"},

	{"Channel Select Mux","Left to Left, Right to Right","SDIN"},
	{"Channel Select Mux","Right to both Left and Right","SDIN"},
	{"Channel Select Mux","Left to both Left & Right","SDIN"},
	{"Channel Select Mux","Left to Right, Right to Left","SDIN"},

	{"DACL",NULL,"Channel Select Mux"},
	{"DACR",NULL,"Channel Select Mux"},



	{ "LOUT", NULL, "DACL" },	
	{ "ROUT", NULL, "DACR" },
};

/*************** parameter define ***************/
#define STATEconfirm		0x0C
#define NORMAL_I2S			0x00
#define NORMAL_LJ			0x01
#define NORMAL_DSPA			0x03
#define NORMAL_DSPB			0x07
#define Format_Len24		0x00
#define Format_Len20		0x01
#define Format_Len18		0x02
#define Format_Len16		0x03
#define Format_Len32		0x04

#define VDDA_3V3			0x00
#define VDDA_1V8			0x01
#define MCLK_PIN			0x00
#define SCLK_PIN			0x01

/**************************************************/
#define MSMode_MasterSelOn	0				// SlaveMode:0, MasterMode:1
#define Ratio 				64				// Ratio = MCLK/LRCK on board
#define Format 				NORMAL_I2S
#define Format_Len			Format_Len16	// data format
#define SCLK_DIV			8				// SCLK_DIV = MCLK/SCLK
#define SCLK_INV			0
#define MCLK_SOURCE			SCLK_PIN		// select MCLK source, MCLK_PIN or SCLK_PIN
#define EQ7bandOn			0
#define VDDA_VOLTAGE		VDDA_3V3
#define DAC_Volume			191				// DAC digital gain
#define DACHPModeOn			0				// disable:0, enable:1

/**************************************************/

static int es8156_init_sequence(struct snd_soc_component *codec)
{
	pr_debug("%s\n", __func__);

	snd_soc_component_write(codec,0x02,(MCLK_SOURCE<<7) + (SCLK_INV<<4) +  (EQ7bandOn<<3) + 0x04 + MSMode_MasterSelOn);
	snd_soc_component_write(codec,0x19,0x20);

	if(DACHPModeOn == 0) // output from PA
		{
			snd_soc_component_write(codec,0x20,0x2A);
			snd_soc_component_write(codec,0x21,0x3C);
			snd_soc_component_write(codec,0x22,0x02);
			snd_soc_component_write(codec,0x24,0x07);
			snd_soc_component_write(codec,0x23,0x40 + (0x30*VDDA_VOLTAGE));
		}
	if(DACHPModeOn == 1) // output from headphone
		{
			snd_soc_component_write(codec,0x20,0x16);
			snd_soc_component_write(codec,0x21,0x3F);
			snd_soc_component_write(codec,0x22,0x0A);
			snd_soc_component_write(codec,0x24,0x01);
			snd_soc_component_write(codec,0x23,0xCA + (0x30*VDDA_VOLTAGE));
		}
	snd_soc_component_write(codec,0x0A,0x01);
	snd_soc_component_write(codec,0x0B,0x01);
	//snd_soc_component_write(codec,0x11,NORMAL_I2S + (Format_Len<<4));
	snd_soc_component_write(codec,0x14,DAC_Volume);
	if(Ratio == 1536) // Ratio=MCLK/LRCK=1536; 12M288/8K; 24M576/16K
		{
			snd_soc_component_write(codec,0x01,0x26 - (0x03*EQ7bandOn)); // 1536 Ratio(MCLK/LRCK)
			snd_soc_component_write(codec,0x09,0x00); // 1536 Ratio(MCLK/LRCK)
			snd_soc_component_write(codec,0x03,0x06); // LRCK H
			snd_soc_component_write(codec,0x04,0x00); // LRCK=MCLK/1536
			snd_soc_component_write(codec,0x05,SCLK_DIV); // BCLK=MCLK/4
		}
	if(Ratio == 1024) // Ratio=MCLK/LRCK=1024; 12M288/12K; 24M576/24K
		{
			snd_soc_component_write(codec,0x01,0x24 - (0x02*EQ7bandOn)); // 256 Ratio(MCLK/LRCK)
			snd_soc_component_write(codec,0x09,0x00); // 256 Ratio(MCLK/LRCK)
			snd_soc_component_write(codec,0x03,0x04); // LRCK H
			snd_soc_component_write(codec,0x04,0x00); // LRCK=MCLK/256
			snd_soc_component_write(codec,0x05,SCLK_DIV); // BCLK=MCLK/4
		}
	if(Ratio == 768) // Ratio=MCLK/LRCK=768; 12M288/16K; 24M576/32K
		{
			snd_soc_component_write(codec,0x01,0x23 + (0x40*EQ7bandOn)); // 768 Ratio(MCLK/LRCK)
			snd_soc_component_write(codec,0x09,0x00); // 768 Ratio(MCLK/LRCK)
			snd_soc_component_write(codec,0x03,0x03); // LRCK H
			snd_soc_component_write(codec,0x04,0x00); // LRCK=MCLK/768
			snd_soc_component_write(codec,0x05,SCLK_DIV); // BCLK=MCLK/4
		}
	if(Ratio == 512) // Ratio=MCLK/LRCK=512; 12M288/24K; 24M576/48K
		{
			snd_soc_component_write(codec,0x01,0xC0 + 0x22 - (0x01*EQ7bandOn)); // 512 Ratio(MCLK/LRCK)
			snd_soc_component_write(codec,0x09,0x00); // 512 Ratio(MCLK/LRCK)
			snd_soc_component_write(codec,0x03,0x02); // LRCK H
			snd_soc_component_write(codec,0x04,0x00); // LRCK=MCLK/512
			snd_soc_component_write(codec,0x05,SCLK_DIV); //BCLK=MCLK/4
		}
	if(Ratio == 400) // Ratio=MCLK/LRCK=400; 19M2/48K
		{	// DVDD must be 3.3V
			snd_soc_component_write(codec,0x01,0x21 + (0x40*EQ7bandOn)); // 384 Ratio(MCLK/LRCK)
			snd_soc_component_write(codec,0x09,0x00); // 400 Ratio(MCLK/LRCK)
			snd_soc_component_write(codec,0x10,0x64); // 400 OSR
			snd_soc_component_write(codec,0x03,0x01); // LRCK H
			snd_soc_component_write(codec,0x04,0x90); // LRCK=MCLK/400
			snd_soc_component_write(codec,0x05,SCLK_DIV); // BCLK=MCLK/4
		}
	if(Ratio == 384) // Ratio=MCLK/LRCK=384; 12M288/32K; 6M144/16K
		{
			snd_soc_component_write(codec,0x01,0x63 + (0x40*EQ7bandOn)); // 384 Ratio(MCLK/LRCK)
			snd_soc_component_write(codec,0x09,0x00); // 384 Ratio(MCLK/LRCK)
			snd_soc_component_write(codec,0x03,0x01); // LRCK H
			snd_soc_component_write(codec,0x04,0x80); // LRCK=MCLK/384
			snd_soc_component_write(codec,0x05,SCLK_DIV); // BCLK=MCLK/4
		}
	if(Ratio == 256) // Ratio=MCLK/LRCK=256; 12M288/48K
		{
			snd_soc_component_write(codec,0x01,0x21 + (0x40*EQ7bandOn)); // 256 Ratio(MCLK/LRCK)
			snd_soc_component_write(codec,0x09,0x00); // 256 Ratio(MCLK/LRCK)
			snd_soc_component_write(codec,0x03,0x01); // LRCK H
			snd_soc_component_write(codec,0x04,0x00); // LRCK=MCLK/256
			snd_soc_component_write(codec,0x05,SCLK_DIV); // BCLK=MCLK/4
		}
	if(Ratio == 128) // Ratio=MCLK/LRCK=128; 6M144/48K
		{
			snd_soc_component_write(codec,0x01,0x61 + (0x40*EQ7bandOn)); // 128 Ratio(MCLK/LRCK)
			snd_soc_component_write(codec,0x09,0x00); // 128 Ratio(MCLK/LRCK)
			snd_soc_component_write(codec,0x03,0x00); // LRCK H
			snd_soc_component_write(codec,0x04,0x80); // LRCK=MCLK/128
			snd_soc_component_write(codec,0x05,SCLK_DIV); // BCLK=MCLK/4
		}
	if(Ratio == 64) // Ratio=MCLK/LRCK=64; 3M072/48K
		{
			snd_soc_component_write(codec,0x01,0xE1); // 64 Ratio(MCLK/LRCK)
			snd_soc_component_write(codec,0x09,0x02); // 64 Ratio(MCLK/LRCK)
			snd_soc_component_write(codec,0x03,0x00); // LRCK H
			snd_soc_component_write(codec,0x04,0x40); // LRCK=MCLK/64
			snd_soc_component_write(codec,0x05,SCLK_DIV); // BCLK=MCLK/2
		}

	snd_soc_component_write(codec,0x0D,0x14);
	snd_soc_component_write(codec,0x18,0x00);
	snd_soc_component_write(codec,0x08,0x3F);
	snd_soc_component_write(codec,0x00,0x02);
	snd_soc_component_write(codec,0x00,0x03);
	snd_soc_component_write(codec,0x25,0x20);

	return 0;
}


static int es8156_set_dai_fmt(struct snd_soc_dai *codec_dai,
			      unsigned int fmt)
{
	struct snd_soc_component *codec = codec_dai->component;		
	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM://es8156 master
		snd_soc_component_update_bits(codec, ES8156_SCLK_MODE_REG02, 0x01,0x01);
		break;
	case SND_SOC_DAIFMT_CBS_CFS://es8156 slave
		snd_soc_component_update_bits(codec, ES8156_SCLK_MODE_REG02, 0x01,0x00);
		break;
	default:
		return -EINVAL;
	}
	/* interface format */

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		snd_soc_component_update_bits(codec, ES8156_DAC_SDP_REG11, 0x07,0x00);
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		return -EINVAL;
	case SND_SOC_DAIFMT_LEFT_J:		
		snd_soc_component_update_bits(codec, ES8156_DAC_SDP_REG11, 0x07,0x01);
		break;
	case SND_SOC_DAIFMT_DSP_A:
		snd_soc_component_update_bits(codec, ES8156_DAC_SDP_REG11, 0x07,0x03);
		break;
	case SND_SOC_DAIFMT_DSP_B:
		snd_soc_component_update_bits(codec, ES8156_DAC_SDP_REG11, 0x07,0x07);
		break;
	default:
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		snd_soc_component_update_bits(codec, ES8156_SCLK_MODE_REG02, 0x01,0x00);		
		break;
	case SND_SOC_DAIFMT_IB_IF:
		snd_soc_component_update_bits(codec, ES8156_SCLK_MODE_REG02, 0x01,0x01);	
		break;
	case SND_SOC_DAIFMT_IB_NF:
		snd_soc_component_update_bits(codec, ES8156_SCLK_MODE_REG02, 0x01,0x01);			
		break;
	case SND_SOC_DAIFMT_NB_IF:
		snd_soc_component_update_bits(codec, ES8156_SCLK_MODE_REG02, 0x01,0x00);			
		break;
	default:
		return -EINVAL;
	}	
	return 0;
}


static int es8156_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_component *codec = dai->component;	
	struct snd_pcm_runtime *runtime = substream->runtime;

	snd_pcm_hw_constraint_minmax(runtime, SNDRV_PCM_HW_PARAM_PERIOD_BYTES,
				     4096, UINT_MAX);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:		
		snd_soc_component_update_bits(codec, ES8156_DAC_SDP_REG11, 0x70,0x30);
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:		
		snd_soc_component_update_bits(codec, ES8156_DAC_SDP_REG11, 0x70,0x10);
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		snd_soc_component_update_bits(codec, ES8156_DAC_SDP_REG11, 0x70,0x00);
		break;
	case SNDRV_PCM_FORMAT_S32_LE:		
		snd_soc_component_update_bits(codec, ES8156_DAC_SDP_REG11, 0x70,0x40);
		break;
	}
	return 0;
}

static int es8156_set_bias_level(struct snd_soc_component *codec,
				 enum snd_soc_bias_level level)
{
	int ret;  
	struct es8156_priv *priv = snd_soc_component_get_drvdata(codec);

	switch (level) 
	{
		case SND_SOC_BIAS_ON:
			break;

		case SND_SOC_BIAS_PREPARE:		
			break;

		case SND_SOC_BIAS_STANDBY:
		/*
		*open i2s clock 
		*/
		if (snd_soc_component_get_bias_level(codec) == SND_SOC_BIAS_OFF) 
		{
			if (!IS_ERR(priv->mclk)) 
			{
				ret = clk_prepare_enable(priv->mclk);
				es8156_init_sequence(codec);
				if (ret) 
				{
					dev_err(codec->dev,
						"Failed to enable master clock: %d\n",
						ret);
					return ret;
				}
			}
		}
		break;

	case SND_SOC_BIAS_OFF:
		snd_soc_component_write(codec,0x14,0x00);
		snd_soc_component_write(codec,0x19,0x02);
		snd_soc_component_write(codec,0x22,0x02);
		snd_soc_component_write(codec,0x25,0x81);
		snd_soc_component_write(codec,0x18,0x01);
		snd_soc_component_write(codec,0x09,0x02);
		snd_soc_component_write(codec,0x09,0x01);
		snd_soc_component_write(codec,0x08,0x00);
		mdelay(500);
		snd_soc_component_write(codec,0x25,0x87);
		/*
		*close i2s clock
		*/
		if (!IS_ERR(priv->mclk)) {
			pr_info("%s codec_uninit_sequence\n", __func__);
			clk_disable_unprepare(priv->mclk);
		}
		break;
	}
	return 0;
}

#define es8156_RATES SNDRV_PCM_RATE_8000_96000

#define es8156_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
	SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FORMAT_S32_LE)

static struct snd_soc_dai_ops es8156_ops = {
    .startup = NULL,
	.hw_params = es8156_pcm_hw_params,
	.set_fmt = es8156_set_dai_fmt,
	.set_sysclk = NULL,
	.shutdown = NULL,
};

static struct snd_soc_dai_driver es8156_dai = {
	.name = "ES8156 HiFi",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = es8156_RATES,
		.formats = es8156_FORMATS,
	},
	.ops = &es8156_ops,
	.symmetric_rates = 1,
};


static int es8156_suspend(struct snd_soc_component *codec)
{
	es8156_set_bias_level(codec, SND_SOC_BIAS_OFF);
	printk("es8156_suspend\n");
	return 0;
}

static int es8156_resume(struct snd_soc_component *codec)
{	
	return 0;
}

#ifdef HP_DET_FUNTION
static irqreturn_t es8156_irq_handler(int irq, void *data)
{
	struct es8156_priv *es8156 = data;

	queue_delayed_work(system_power_efficient_wq, &es8156->work,
			   msecs_to_jiffies(es8156->debounce_time));

	return IRQ_HANDLED;
}
#endif

/*
 * Call from rk_headset_irq_hook_adc.c
 *
 * Enable micbias for HOOK detection and disable external Amplifier
 * when jack insertion.
 */
int es8156_headset_detect(int jack_insert)
{
	struct es8156_priv *es8156;

	if (!es8156_codec)
		return -1;

	es8156 = snd_soc_component_get_drvdata(es8156_codec);

	es8156->hp_inserted = jack_insert;

	/*enable micbias and disable PA*/
	if (jack_insert) {
		es8156_enable_spk(es8156, false);
	}

	return 0;
}
EXPORT_SYMBOL(es8156_headset_detect);

#ifdef HP_DET_FUNTION
static void hp_work(struct work_struct *work)
{
	struct es8156_priv *es8156;
	int enable;

	es8156 = container_of(work, struct es8156_priv, work.work);
	enable = gpio_get_value(es8156->hp_det_gpio);
	if (es8156->hp_det_invert)
		enable = !enable;

	es8156->hp_inserted = enable ? true : false;
	if (!es8156->muted) {
		if (es8156->hp_inserted)
			es8156_enable_spk(es8156, false);
		else
			es8156_enable_spk(es8156, true);
	}
}
#endif

static int es8156_probe(struct snd_soc_component *codec)
{
#if MCLK
	struct es8156_priv *es8156 = snd_soc_component_get_drvdata(codec);
#endif
	int ret = 0;

	es8156_codec = codec;
#if MCLK
	es8156->mclk = devm_clk_get(codec->dev, "mclk");
	if (PTR_ERR(es8156->mclk) == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	ret = clk_prepare_enable(es8156->mclk);
#endif	
	es8156_reset(codec);
	es8156_init_sequence(codec);
	return ret;
}

static void es8156_remove(struct snd_soc_component *codec)
{
	es8156_set_bias_level(codec, SND_SOC_BIAS_OFF);
}

const struct regmap_config es8156_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= 0xff,
	.cache_type	= REGCACHE_RBTREE,
	.reg_defaults = es8156_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(es8156_reg_defaults),
};

static struct snd_soc_component_driver soc_codec_dev_es8156 = {
	.probe =	es8156_probe,
	.remove =	es8156_remove,
	.suspend =	es8156_suspend,
	.resume =	es8156_resume,
	.set_bias_level = es8156_set_bias_level,
	.controls = es8156_snd_controls,
	.num_controls = ARRAY_SIZE(es8156_snd_controls),
	.dapm_widgets = es8156_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(es8156_dapm_widgets),
	.dapm_routes = es8156_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(es8156_dapm_routes),
};

/*
*es8156 7bit i2c address:CE pin:0 0x08 / CE pin:1 0x09
*/
static int es8156_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct es8156_priv *es8156;
	int ret = -1;
#ifdef HP_DET_FUNTION
	int hp_irq;
	enum of_gpio_flags flags;
	struct device_node *np = i2c->dev.of_node;
#endif
	es8156 = devm_kzalloc(&i2c->dev, sizeof(*es8156), GFP_KERNEL);
	if (!es8156)
		return -ENOMEM;

	es8156->debounce_time = 200;
	es8156->hp_det_invert = 0;
	es8156->pwr_count = 0;
	es8156->hp_inserted = false;
	es8156->muted = true;

	es8156->regmap = devm_regmap_init_i2c(i2c, &es8156_regmap_config);
	if (IS_ERR(es8156->regmap)) {
		ret = PTR_ERR(es8156->regmap);
		dev_err(&i2c->dev, "Failed to init regmap: %d\n", ret);
		return ret;
	}

	i2c_set_clientdata(i2c, es8156);
#ifdef HP_DET_FUNTION
	es8156->spk_ctl_gpio = of_get_named_gpio_flags(np,
						       "spk-con-gpio",
						       0,
						       &flags);
	if (es8156->spk_ctl_gpio < 0) {
		dev_info(&i2c->dev, "Can not read property spk_ctl_gpio\n");
		es8156->spk_ctl_gpio = INVALID_GPIO;
	} else {
		es8156->spk_active_level = !(flags & OF_GPIO_ACTIVE_LOW);
		ret = devm_gpio_request_one(&i2c->dev, es8156->spk_ctl_gpio,
					    GPIOF_DIR_OUT, NULL);
		if (ret) {
			dev_err(&i2c->dev, "Failed to request spk_ctl_gpio\n");
			return ret;
		}
		es8156_enable_spk(es8156, false);
	}

	es8156->hp_det_gpio = of_get_named_gpio_flags(np,
						      "hp-det-gpio",
						      0,
						      &flags);
	if (es8156->hp_det_gpio < 0) {
		dev_info(&i2c->dev, "Can not read property hp_det_gpio\n");
		es8156->hp_det_gpio = INVALID_GPIO;
	} else {
		INIT_DELAYED_WORK(&es8156->work, hp_work);
		es8156->hp_det_invert = !!(flags & OF_GPIO_ACTIVE_LOW);
		ret = devm_gpio_request_one(&i2c->dev, es8156->hp_det_gpio,
					    GPIOF_IN, "hp det");
		if (ret < 0)
			return ret;
		hp_irq = gpio_to_irq(es8156->hp_det_gpio);
		ret = devm_request_threaded_irq(&i2c->dev, hp_irq, NULL,
						es8156_irq_handler,
						IRQF_TRIGGER_FALLING |
						IRQF_TRIGGER_RISING |
						IRQF_ONESHOT,
						"es8156_interrupt", es8156);
		if (ret < 0) {
			dev_err(&i2c->dev, "request_irq failed: %d\n", ret);
			return ret;
		}

		schedule_delayed_work(&es8156->work,
				      msecs_to_jiffies(es8156->debounce_time));
	}
#endif
	ret = snd_soc_register_component(&i2c->dev,
				     &soc_codec_dev_es8156,
				     &es8156_dai, 1);
	
	return ret;
}

static  int es8156_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_component(&client->dev);
	return 0;
}

static void es8156_i2c_shutdown(struct i2c_client *client)
{
	struct es8156_priv *es8156 = i2c_get_clientdata(client);

	if (es8156_codec != NULL) {
		es8156_enable_spk(es8156, false);
		msleep(20);
		es8156_set_bias_level(es8156_codec, SND_SOC_BIAS_OFF);
	}
}

static const struct i2c_device_id es8156_i2c_id[] = {
	{"es8156", 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, es8156_i2c_id);


static const struct of_device_id es8156_of_match[] = {
	{ .compatible = "everest,es8156", },
	{ }
};
MODULE_DEVICE_TABLE(of, es8156_of_match);

static struct i2c_driver es8156_i2c_driver = {
	.driver = {
		.name		= "es8156",
		.of_match_table = es8156_of_match,
	},
	.probe    = es8156_i2c_probe,
	.remove   = es8156_i2c_remove,
	.shutdown = es8156_i2c_shutdown,
	.id_table = es8156_i2c_id,
};
module_i2c_driver(es8156_i2c_driver);

MODULE_DESCRIPTION("ASoC es8156 driver");
MODULE_LICENSE("GPL");
