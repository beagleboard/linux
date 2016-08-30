/*
 * Copyright (C) 2014 Texas Instruments
 * Author : Tomi Valkeinen <tomi.valkeinen@ti.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 */

#ifndef __ENCODER_SII9022_H_
#define __ENCODER_SII9022_H_

#define SII9022_ID_902xA		0xb0

#define HDMI_I2C_MONITOR_ADDRESS	0x50

#define SII9022_VIDEO_DATA_BASE_REG	0x00
#define SII9022_PIXEL_CLK_LSB_REG	(SII9022_VIDEO_DATA_BASE_REG + 0x00)
#define SII9022_PIXEL_CLK_MSB_REG	(SII9022_VIDEO_DATA_BASE_REG + 0x01)
#define SII9022_VFREQ_LSB_REG		(SII9022_VIDEO_DATA_BASE_REG + 0x02)
#define SII9022_VFREQ_MSB_REG		(SII9022_VIDEO_DATA_BASE_REG + 0x03)
#define SII9022_PIXELS_LSB_REG		(SII9022_VIDEO_DATA_BASE_REG + 0x04)
#define SII9022_PIXELS_MSB_REG		(SII9022_VIDEO_DATA_BASE_REG + 0x05)
#define SII9022_LINES_LSB_REG		(SII9022_VIDEO_DATA_BASE_REG + 0x06)
#define SII9022_LINES_MSB_REG		(SII9022_VIDEO_DATA_BASE_REG + 0x07)

#define SII9022_PIXEL_REPETITION_REG	0x08

#define SII9022_AVI_IN_FORMAT_REG	0x09
#define SII9022_AVI_OUT_FORMAT_REG	0x0a
#define SII9022_AVI_INFOFRAME_BASE_REG	0x0c

#define SII9022_SYS_CTRL_DATA_REG	0x1a
#define SII9022_DEVICE_ID_REG		0x1b
#define SII9022_DEVICE_REV_ID_REG	0x1c
#define SII9022_DEVICE_TPI_ID_REG	0x1d

#define SII9022_POWER_STATE_CTRL_REG	0x1e

#define SII9022_IRQ_ENABLE_REG		0x3c
#define SII9022_IRQ_STATUS_REG		0x3d

#define SII9022_TPI_RQB_REG		0xc7

/* Indirect internal register access */
#define HDMI_IND_SET_PAGE		0xbc
#define HDMI_IND_OFFSET			0xbd
#define HDMI_IND_VALUE			0xbe

/* AVI InfoFrame */
#define HDMI_CPI_MISC_IF_SELECT_REG     0xbf
#define HDMI_CPI_MISC_IF_OFFSET         0xC0

/* Audio  */
#define HDMI_TPI_I2S_ENABLE_MAPPING_REG 0x1f
#define HDMI_TPI_I2S_INPUT_CONFIG_REG   0x20
#define HDMI_TPI_I2S_STRM_HDR_BASE      0x21
#define HDMI_TPI_I2S_STRM_HDR_0_REG     (HDMI_TPI_I2S_STRM_HDR_BASE + 0)
#define HDMI_TPI_I2S_STRM_HDR_1_REG     (HDMI_TPI_I2S_STRM_HDR_BASE + 1)
#define HDMI_TPI_I2S_STRM_HDR_2_REG     (HDMI_TPI_I2S_STRM_HDR_BASE + 2)
#define HDMI_TPI_I2S_STRM_HDR_3_REG     (HDMI_TPI_I2S_STRM_HDR_BASE + 3)
#define HDMI_TPI_I2S_STRM_HDR_4_REG     (HDMI_TPI_I2S_STRM_HDR_BASE + 4)
#define HDMI_TPI_AUDIO_CONFIG_BYTE2_REG	0x26
#define HDMI_TPI_AUDIO_CONFIG_BYTE3_REG	0x27
#define HDMI_TPI_AUDIO_CONFIG_BYTE4_REG	0x28

/* SII9022_SYS_CTRL_DATA_REG */
#define SII9022_SYS_CTRL_DDC_BUS_GRANTED	BIT(1)
#define SII9022_SYS_CTRL_DDC_BUS_REQUEST	BIT(2)

/* HDMI_TPI_AUDIO_CONFIG_BYTE2_REG  */
#define TPI_AUDIO_CODING_STREAM_HEADER		(0 << 0)
#define TPI_AUDIO_CODING_PCM			(1 << 0)
#define TPI_AUDIO_CODING_AC3			(2 << 0)
#define TPI_AUDIO_CODING_MPEG1			(3 << 0)
#define TPI_AUDIO_CODING_MP3			(4 << 0)
#define TPI_AUDIO_CODING_MPEG2			(5 << 0)
#define TPI_AUDIO_CODING_AAC			(6 << 0)
#define TPI_AUDIO_CODING_DTS			(7 << 0)
#define TPI_AUDIO_CODING_ATRAC			(8 << 0)
#define TPI_AUDIO_MUTE_DISABLE			(0 << 4)
#define TPI_AUDIO_MUTE_ENABLE			(1 << 4)
#define TPI_AUDIO_LAYOUT_2_CHANNELS		(0 << 5)
#define TPI_AUDIO_LAYOUT_8_CHANNELS		(1 << 5)
#define TPI_AUDIO_INTERFACE_DISABLE		(0 << 6)
#define TPI_AUDIO_INTERFACE_SPDIF		(1 << 6)
#define TPI_AUDIO_INTERFACE_I2S			(2 << 6)

/* HDMI_TPI_AUDIO_CONFIG_BYTE3_REG  */
#define TPI_AUDIO_CHANNEL_STREAM		(0 << 0)
#define TPI_AUDIO_2_CHANNEL			(1 << 0)
#define TPI_AUDIO_8_CHANNEL			(7 << 0)
#define TPI_AUDIO_FREQ_STREAM			(0 << 3)
#define TPI_AUDIO_FREQ_32KHZ			(1 << 3)
#define TPI_AUDIO_FREQ_44KHZ			(2 << 3)
#define TPI_AUDIO_FREQ_48KHZ			(3 << 3)
#define TPI_AUDIO_FREQ_88KHZ			(4 << 3)
#define TPI_AUDIO_FREQ_96KHZ			(5 << 3)
#define TPI_AUDIO_FREQ_176KHZ			(6 << 3)
#define TPI_AUDIO_FREQ_192KHZ			(7 << 3)
#define TPI_AUDIO_SAMPLE_SIZE_STREAM		(0 << 6)
#define TPI_AUDIO_SAMPLE_SIZE_16		(1 << 6)
#define TPI_AUDIO_SAMPLE_SIZE_20		(2 << 6)
#define TPI_AUDIO_SAMPLE_SIZE_24		(3 << 6)

/* HDMI_TPI_I2S_ENABLE_MAPPING_REG  */
#define TPI_I2S_CONFIG_FIFO0			(0 << 0)
#define TPI_I2S_CONFIG_FIFO1			(1 << 0)
#define TPI_I2S_CONFIG_FIFO2			(2 << 0)
#define TPI_I2S_CONFIG_FIFO3			(3 << 0)
#define TPI_I2S_LEFT_RIGHT_SWAP			(1 << 2)
#define TPI_I2S_AUTO_DOWNSAMPLE			(1 << 3)
#define TPI_I2S_SELECT_SD0			(0 << 4)
#define TPI_I2S_SELECT_SD1			(1 << 4)
#define TPI_I2S_SELECT_SD2			(2 << 4)
#define TPI_I2S_SELECT_SD3			(3 << 4)
#define TPI_I2S_FIFO_ENABLE			(1 << 7)

/* HDMI_TPI_I2S_INPUT_CONFIG_REG  */
#define TPI_I2S_FIRST_BIT_SHIFT_YES		(0 << 0)
#define TPI_I2S_FIRST_BIT_SHIFT_NO		(1 << 0)
#define TPI_I2S_SD_DIRECTION_MSB_FIRST		(0 << 1)
#define TPI_I2S_SD_DIRECTION_LSB_FIRST		(1 << 1)
#define TPI_I2S_SD_JUSTIFY_LEFT			(0 << 2)
#define TPI_I2S_SD_JUSTIFY_RIGHT		(1 << 2)
#define TPI_I2S_WS_POLARITY_LOW			(0 << 3)
#define TPI_I2S_WS_POLARITY_HIGH		(1 << 3)
#define TPI_I2S_MCLK_MULTIPLIER_128		(0 << 4)
#define TPI_I2S_MCLK_MULTIPLIER_256		(1 << 4)
#define TPI_I2S_MCLK_MULTIPLIER_384		(2 << 4)
#define TPI_I2S_MCLK_MULTIPLIER_512		(3 << 4)
#define TPI_I2S_MCLK_MULTIPLIER_768		(4 << 4)
#define TPI_I2S_MCLK_MULTIPLIER_1024		(5 << 4)
#define TPI_I2S_MCLK_MULTIPLIER_1152		(6 << 4)
#define TPI_I2S_MCLK_MULTIPLIER_192		(7 << 4)
#define TPI_I2S_SCK_EDGE_FALLING		(0 << 7)
#define TPI_I2S_SCK_EDGE_RISING			(1 << 7)

/* SII9022_IRQ_ENABLE_REG / SII9022_IRQ_STATUS_REG */
#define SII9022_IRQ_HPE				(1 << 0)
#define SII9022_IRQ_RXSENSE			(1 << 1)
#define SII9022_IRQ_HP_STATE			(1 << 2)
#define SII9022_IRQ_RXSENSE_STATE		(1 << 3)
#define SII9022_IRQ_AUDIO_ERROR			(1 << 4)
#define SII9022_IRQ_SEC_STATUS_CHANGE		(1 << 5)
#define SII9022_IRQ_HDCP_AUTH_CHANGE		(1 << 7)

enum sii9022_power_state {
	SII9022_POWER_STATE_D0,
	SII9022_POWER_STATE_D2,
	SII9022_POWER_STATE_D3_HOT,
	SII9022_POWER_STATE_D3_COLD,
};

struct sii9022_audio;
struct panel_drv_data {
	struct omap_dss_device dssdev;
	struct omap_dss_device *in;
	struct i2c_client *i2c_client;
	struct gpio_desc *reset_gpio;
	struct regmap *regmap;
	struct omap_video_timings timings;
	struct delayed_work work;
	struct sii9022_audio *audio;
	struct mutex lock;

	int irq;
	bool use_polling;

	bool htplg_state;
	bool rxsense_state;

	bool hdmi_mode;
	struct hdmi_avi_infoframe frame;
};

#define to_panel_data(x) container_of(x, struct panel_drv_data, dssdev)

int sii9022_hdmi_codec_register(struct device *dev);
void sii9022_hdmi_codec_unregister(struct device *dev);

#endif
