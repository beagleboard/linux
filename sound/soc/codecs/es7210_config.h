/*
 * ALSA SoC ES7210 adc driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.^M
 *
 * Notes:
 *  ES7210 is a 4-ch ADC of Everest
 *  This is user configuration document. 
 *  You can update this file according to your application
 */

#define ENABLE          1
#define DISABLE         0

#define MIC_CHN_16      16
#define MIC_CHN_14      14
#define MIC_CHN_12      12
#define MIC_CHN_10      10
#define MIC_CHN_8       8
#define MIC_CHN_6       6
#define MIC_CHN_4       4
#define MIC_CHN_2       2

#define ES7210_TDM_ENABLE       ENABLE
#define ES7210_CHANNELS_MAX     MIC_CHN_2

#if ES7210_CHANNELS_MAX == MIC_CHN_2
#define ADC_DEV_MAXNUM  1
#endif
#if ES7210_CHANNELS_MAX == MIC_CHN_4
#define ADC_DEV_MAXNUM  1
#endif
#if ES7210_CHANNELS_MAX == MIC_CHN_6
#define ADC_DEV_MAXNUM  2
#endif
#if ES7210_CHANNELS_MAX == MIC_CHN_8
#define ADC_DEV_MAXNUM  2
#endif
#if ES7210_CHANNELS_MAX == MIC_CHN_10
#define ADC_DEV_MAXNUM  3
#endif
#if ES7210_CHANNELS_MAX == MIC_CHN_12
#define ADC_DEV_MAXNUM  3
#endif
#if ES7210_CHANNELS_MAX == MIC_CHN_14
#define ADC_DEV_MAXNUM  4
#endif
#if ES7210_CHANNELS_MAX == MIC_CHN_16
#define ADC_DEV_MAXNUM  4
#endif

#define ES7210_TDM_1LRCK_DSPA                 0
#define ES7210_TDM_1LRCK_DSPB                 1
#define ES7210_TDM_1LRCK_I2S                  2
#define ES7210_TDM_1LRCK_LJ                   3
#define ES7210_TDM_NLRCK_DSPA                 4
#define ES7210_TDM_NLRCK_DSPB                 5
#define ES7210_TDM_NLRCK_I2S                  6
#define ES7210_TDM_NLRCK_LJ                   7
#define ES7210_NORMAL_I2S                       8
#define ES7210_NORMAL_LJ                        9
#define ES7210_NORMAL_DSPA                      10
#define ES7210_NORMAL_DSPB                      11

#define ES7210_WORK_MODE    ES7210_NORMAL_I2S

#define RATIO_MCLK_LRCK_64   64
#define RATIO_MCLK_LRCK_128  128
#define RATIO_MCLK_LRCK_192  192
#define RATIO_MCLK_LRCK_256  256
#define RATIO_MCLK_LRCK_384  384
#define RATIO_MCLK_LRCK_512  512
#define RATIO_MCLK_LRCK_768  768
#define RATIO_MCLK_LRCK_896  896
#define RATIO_MCLK_LRCK_1024 1024
#define RATIO_MCLK_LRCK_1152  1152
#define RATIO_MCLK_LRCK_1280  1280
#define RATIO_MCLK_LRCK_1408  1408
#define RAITO_MCLK_LRCK_1536 1536
#define RATIO_MCLK_LRCK_1664  1664
#define RATIO_MCLK_LRCK_1792  1792
#define RATIO_MCLK_LRCK_1920  1920
#define RATIO_MCLK_LRCK_2048 2048
#define RATIO_MCLK_LRCK_3072  3072
#define RATIO_MCLK_LRCK_4096  4096

#define RATIO_MCLK_LRCK  RATIO_MCLK_LRCK_64

#define ES7210_I2C_BUS_NUM              1
#define ES7210_CODEC_RW_TEST_EN         0
#define ES7210_IDLE_RESET_EN            1	//reset ES7210 when in idle time
#define ES7210_MATCH_DTS_EN             1	//ES7210 match method select: 0: i2c_detect, 1:of_device_id

struct i2c_client *i2c_clt1[ADC_DEV_MAXNUM];
struct snd_soc_component *tron_codec1[ADC_DEV_MAXNUM];

int es7210_init_reg = 0;
static int es7210_codec_num = 0;
struct es7210_reg_config {
	unsigned char reg_addr;
	unsigned char reg_v;
};
static const struct es7210_reg_config es7210_tdm_reg_common_cfg1[] = {
	{0x00, 0xFF},
	{0x00, 0x32},
	{0x09, 0x30},
	{0x0A, 0x30},
	{0x23, 0x2a},
	{0x22, 0x0a},
	{0x21, 0x2a},
	{0x20, 0x0a},
};

static const struct es7210_reg_config es7210_tdm_reg_fmt_cfg[] = {
	{0x11, 0x63},
	{0x12, 0x01},
};

static const struct es7210_reg_config es7210_tdm_reg_common_cfg2[] = {
	{0x40, 0xC7},
	{0x41, 0x70},
	{0x42, 0x70},
	{0x43, 0x10},
	{0x44, 0x10},
	{0x45, 0x10},
	{0x46, 0x10},
	{0x47, 0x08},
	{0x48, 0x08},
	{0x49, 0x08},
	{0x4A, 0x08},
	{0x07, 0x20},
};

static const struct es7210_reg_config es7210_tdm_reg_mclk_cfg[] = {
	{0x02, 0xC1},
};

static const struct es7210_reg_config es7210_tdm_reg_common_cfg3[] = {
	{0x06, 0x00},
	{0x4B, 0x0F},
	{0x4C, 0x0F},
	{0x00, 0x71},
	{0x00, 0x41},
};

struct es7210_mclklrck_ratio_config {
	int ratio;
	unsigned char nfs;
	unsigned char channels;
	unsigned char reg02_v;
	unsigned char reg06_v;
};

static const struct es7210_mclklrck_ratio_config es7210_1fs_ratio_cfg[] = {
	//ratio, nfs, channels, reg02_v, reg06_v
	{64, 0, 0, 0x41, 0x00},
	{128, 0, 0, 0x01, 0x00},
	{192, 0, 0, 0x43, 0x00},
	{256, 0, 0, 0xc1, 0x04},
	{384, 0, 0, 0x03, 0x00},
	{512, 0, 0, 0x81, 0x04},
	{768, 0, 0, 0xc3, 0x04},
	{896, 0, 0, 0x07, 0x00},
	{1024, 0, 0, 0x82, 0x04},
	{1152, 0, 0, 0x09, 0x00},
	{1280, 0, 0, 0x0a, 0x00},
	{1408, 0, 0, 0x0b, 0x00},
	{1536, 0, 0, 0xc6, 0x04},
	{1664, 0, 0, 0x0d, 0x00},
	{1792, 0, 0, 0xc7, 0x04},
	{1920, 0, 0, 0x0f, 0x00},
	{2048, 0, 0, 0x84, 0x04},
	{3072, 0, 0, 0x86, 0x04},
	{4096, 0, 0, 0x88, 0x04},
};

static const struct es7210_mclklrck_ratio_config es7210_nfs_ratio_cfg[] = {
	//ratio, nfs, channels, reg02_v, reg06_v
	{32, 1, 4, 0x41, 0x00},
	{32, 1, 8, 0x01, 0x00},
	{32, 1, 12, 0x43, 0x00},
	{32, 1, 16, 0xC1, 0x04},

	{64, 1, 4, 0x01, 0x00},
	{64, 1, 6, 0x43, 0x00},
	{64, 1, 8, 0xC1, 0x04},
	{64, 1, 10, 0x45, 0x00},
	{64, 1, 12, 0x03, 0x00},
	{64, 1, 14, 0x47, 0x00},
	{64, 1, 16, 0x81, 0x04},

	{96, 1, 4, 0x43, 0x00},
	{96, 1, 8, 0x03, 0x00},
	{96, 1, 12, 0x49, 0x00},
	{96, 1, 16, 0xc3, 0x04},

	{128, 1, 4, 0xc1, 0x04},
	{128, 1, 6, 0x03, 0x00},
	{128, 1, 8, 0x81, 0x04},
	{128, 1, 10, 0x05, 0x00},
	{128, 1, 12, 0xc3, 0x04},
	{128, 1, 14, 0x07, 0x00},
	{128, 1, 16, 0x82, 0x04},

	{192, 1, 4, 0x03, 0x00},
	{192, 1, 6, 0x49, 0x00},
	{192, 1, 8, 0xc3, 0x04},
	{192, 1, 10, 0x4f, 0x00},
	{192, 1, 12, 0x09, 0x00},
	{192, 1, 14, 0x55, 0x00},
	{192, 1, 16, 0x83, 0x04},

	{256, 1, 4, 0x81, 0x04},
	{256, 1, 6, 0xc3, 0x04},
	{256, 1, 8, 0x82, 0x04},
	{256, 1, 10, 0xc5, 0x04},
	{256, 1, 12, 0x83, 0x04},
	{256, 1, 14, 0xc7, 0x04},
	{256, 1, 16, 0x84, 0x04},

	{384, 1, 4, 0xc3, 0x04},
	{384, 1, 6, 0x09, 0x00},
	{384, 1, 8, 0x83, 0x04},
	{384, 1, 10, 0x0f, 0x00},
	{384, 1, 12, 0xc9, 0x04},
	{384, 1, 14, 0x15, 0x00},
	{384, 1, 16, 0x86, 0x04},

	{512, 1, 4, 0x82, 0x04},
	{512, 1, 6, 0x83, 0x04},
	{512, 1, 8, 0x84, 0x04},
	{512, 1, 10, 0x85, 0x04},
	{512, 1, 12, 0x86, 0x04},
	{512, 1, 14, 0x87, 0x04},
	{512, 1, 16, 0x88, 0x04},

	{768, 1, 4, 0x83, 0x04},
	{768, 1, 6, 0xC9, 0x04},
	{768, 1, 8, 0x86, 0x04},
	{768, 1, 10, 0xCf, 0x04},
	{768, 1, 12, 0x89, 0x04},
	{768, 1, 14, 0xD5, 0x04},
	{768, 1, 16, 0x8C, 0x04},

	{1024, 1, 4, 0x84, 0x04},
	{1024, 1, 6, 0x86, 0x04},
	{1024, 1, 8, 0x88, 0x04},
	{1024, 1, 10, 0x8a, 0x04},
	{1024, 1, 12, 0x8c, 0x04},
	{1024, 1, 14, 0x8e, 0x04},
	{1024, 1, 16, 0x90, 0x04},

};
