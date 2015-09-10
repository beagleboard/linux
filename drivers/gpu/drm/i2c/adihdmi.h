/*
 * Analog Devices ADIHDMI HDMI transmitter driver
 *
 * Copyright 2012 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#ifndef __DRM_I2C_ADIHDMI_H__
#define __DRM_I2C_ADIHDMI_H__

#include <linux/hdmi.h>

#define ADIHDMI_REG_CHIP_REVISION		0x00
#define ADIHDMI_REG_N0				0x01
#define ADIHDMI_REG_N1				0x02
#define ADIHDMI_REG_N2				0x03
#define ADIHDMI_REG_SPDIF_FREQ			0x04
#define ADIHDMI_REG_CTS_AUTOMATIC1		0x05
#define ADIHDMI_REG_CTS_AUTOMATIC2		0x06
#define ADIHDMI_REG_CTS_MANUAL0			0x07
#define ADIHDMI_REG_CTS_MANUAL1			0x08
#define ADIHDMI_REG_CTS_MANUAL2			0x09
#define ADIHDMI_REG_AUDIO_SOURCE		0x0a
#define ADIHDMI_REG_AUDIO_CONFIG		0x0b
#define ADIHDMI_REG_I2S_CONFIG			0x0c
#define ADIHDMI_REG_I2S_WIDTH			0x0d
#define ADIHDMI_REG_AUDIO_SUB_SRC0		0x0e
#define ADIHDMI_REG_AUDIO_SUB_SRC1		0x0f
#define ADIHDMI_REG_AUDIO_SUB_SRC2		0x10
#define ADIHDMI_REG_AUDIO_SUB_SRC3		0x11
#define ADIHDMI_REG_AUDIO_CFG1			0x12
#define ADIHDMI_REG_AUDIO_CFG2			0x13
#define ADIHDMI_REG_AUDIO_CFG3			0x14
#define ADIHDMI_REG_I2C_FREQ_ID_CFG		0x15
#define ADIHDMI_REG_VIDEO_INPUT_CFG1		0x16
#define ADIHDMI_REG_CSC_UPPER(x)		(0x18 + (x) * 2)
#define ADIHDMI_REG_CSC_LOWER(x)		(0x19 + (x) * 2)
#define ADIHDMI_REG_SYNC_DECODER(x)		(0x30 + (x))
#define ADIHDMI_REG_DE_GENERATOR		(0x35 + (x))
#define ADIHDMI_REG_PIXEL_REPETITION		0x3b
#define ADIHDMI_REG_VIC_MANUAL			0x3c
#define ADIHDMI_REG_VIC_SEND			0x3d
#define ADIHDMI_REG_VIC_DETECTED		0x3e
#define ADIHDMI_REG_AUX_VIC_DETECTED		0x3f
#define ADIHDMI_REG_PACKET_ENABLE0		0x40
#define ADIHDMI_REG_POWER			0x41
#define ADIHDMI_REG_STATUS			0x42
#define ADIHDMI_REG_EDID_I2C_ADDR		0x43
#define ADIHDMI_REG_PACKET_ENABLE1		0x44
#define ADIHDMI_REG_PACKET_I2C_ADDR		0x45
#define ADIHDMI_REG_DSD_ENABLE			0x46
#define ADIHDMI_REG_VIDEO_INPUT_CFG2		0x48
#define ADIHDMI_REG_INFOFRAME_UPDATE		0x4a
#define ADIHDMI_REG_GC(x)			(0x4b + (x)) /* 0x4b - 0x51 */
#define ADIHDMI_REG_AVI_INFOFRAME_VERSION	0x52
#define ADIHDMI_REG_AVI_INFOFRAME_LENGTH	0x53
#define ADIHDMI_REG_AVI_INFOFRAME_CHECKSUM	0x54
#define ADIHDMI_REG_AVI_INFOFRAME(x)		(0x55 + (x)) /* 0x55 - 0x6f */
#define ADIHDMI_REG_AUDIO_INFOFRAME_VERSION	0x70
#define ADIHDMI_REG_AUDIO_INFOFRAME_LENGTH	0x71
#define ADIHDMI_REG_AUDIO_INFOFRAME_CHECKSUM	0x72
#define ADIHDMI_REG_AUDIO_INFOFRAME(x)		(0x73 + (x)) /* 0x73 - 0x7c */
#define ADIHDMI_REG_INT_ENABLE(x)		(0x94 + (x))
#define ADIHDMI_REG_INT(x)			(0x96 + (x))
#define ADIHDMI_REG_INPUT_CLK_DIV		0x9d
#define ADIHDMI_REG_PLL_STATUS			0x9e
#define ADIHDMI_REG_HDMI_POWER			0xa1
#define ADIHDMI_REG_HDCP_HDMI_CFG		0xaf
#define ADIHDMI_REG_AN(x)			(0xb0 + (x)) /* 0xb0 - 0xb7 */
#define ADIHDMI_REG_HDCP_STATUS			0xb8
#define ADIHDMI_REG_BCAPS			0xbe
#define ADIHDMI_REG_BKSV(x)			(0xc0 + (x)) /* 0xc0 - 0xc3 */
#define ADIHDMI_REG_EDID_SEGMENT		0xc4
#define ADIHDMI_REG_DDC_STATUS			0xc8
#define ADIHDMI_REG_EDID_READ_CTRL		0xc9
#define ADIHDMI_REG_BSTATUS(x)			(0xca + (x)) /* 0xca - 0xcb */
#define ADIHDMI_REG_TIMING_GEN_SEQ		0xd0
#define ADIHDMI_REG_POWER2			0xd6
#define ADIHDMI_REG_HSYNC_PLACEMENT_MSB		0xfa

#define ADIHDMI_REG_SYNC_ADJUSTMENT(x)		(0xd7 + (x)) /* 0xd7 - 0xdc */
#define ADIHDMI_REG_TMDS_CLOCK_INV		0xde
#define ADIHDMI_REG_ARC_CTRL			0xdf
#define ADIHDMI_REG_CEC_I2C_ADDR		0xe1
#define ADIHDMI_REG_CEC_CTRL			0xe2
#define ADIHDMI_REG_CHIP_ID_HIGH		0xf5
#define ADIHDMI_REG_CHIP_ID_LOW			0xf6

#define ADIHDMI_CSC_ENABLE			BIT(7)
#define ADIHDMI_CSC_UPDATE_MODE			BIT(5)

#define ADIHDMI_INT0_HDP			BIT(7)
#define ADIHDMI_INT0_VSYNC			BIT(5)
#define ADIHDMI_INT0_AUDIO_FIFO_FULL		BIT(4)
#define ADIHDMI_INT0_EDID_READY			BIT(2)
#define ADIHDMI_INT0_HDCP_AUTHENTICATED		BIT(1)

#define ADIHDMI_INT1_DDC_ERROR			BIT(7)
#define ADIHDMI_INT1_BKSV			BIT(6)
#define ADIHDMI_INT1_CEC_TX_READY		BIT(5)
#define ADIHDMI_INT1_CEC_TX_ARBIT_LOST		BIT(4)
#define ADIHDMI_INT1_CEC_TX_RETRY_TIMEOUT	BIT(3)
#define ADIHDMI_INT1_CEC_RX_READY3		BIT(2)
#define ADIHDMI_INT1_CEC_RX_READY2		BIT(1)
#define ADIHDMI_INT1_CEC_RX_READY1		BIT(0)

#define ADIHDMI_ARC_CTRL_POWER_DOWN		BIT(0)

#define ADIHDMI_CEC_CTRL_POWER_DOWN		BIT(0)

#define ADIHDMI_POWER_POWER_DOWN		BIT(6)

#define ADIHDMI_HDMI_CFG_MODE_MASK		0x2
#define ADIHDMI_HDMI_CFG_MODE_DVI		0x0
#define ADIHDMI_HDMI_CFG_MODE_HDMI		0x2

#define ADIHDMI_AUDIO_SELECT_I2C		0x0
#define ADIHDMI_AUDIO_SELECT_SPDIF		0x1
#define ADIHDMI_AUDIO_SELECT_DSD		0x2
#define ADIHDMI_AUDIO_SELECT_HBR		0x3
#define ADIHDMI_AUDIO_SELECT_DST		0x4

#define ADIHDMI_I2S_SAMPLE_LEN_16		0x2
#define ADIHDMI_I2S_SAMPLE_LEN_20		0x3
#define ADIHDMI_I2S_SAMPLE_LEN_18		0x4
#define ADIHDMI_I2S_SAMPLE_LEN_22		0x5
#define ADIHDMI_I2S_SAMPLE_LEN_19		0x8
#define ADIHDMI_I2S_SAMPLE_LEN_23		0x9
#define ADIHDMI_I2S_SAMPLE_LEN_24		0xb
#define ADIHDMI_I2S_SAMPLE_LEN_17		0xc
#define ADIHDMI_I2S_SAMPLE_LEN_21		0xd

#define ADIHDMI_SAMPLE_FREQ_44100		0x0
#define ADIHDMI_SAMPLE_FREQ_48000		0x2
#define ADIHDMI_SAMPLE_FREQ_32000		0x3
#define ADIHDMI_SAMPLE_FREQ_88200		0x8
#define ADIHDMI_SAMPLE_FREQ_96000		0xa
#define ADIHDMI_SAMPLE_FREQ_176400		0xc
#define ADIHDMI_SAMPLE_FREQ_192000		0xe

#define ADIHDMI_STATUS_POWER_DOWN_POLARITY	BIT(7)
#define ADIHDMI_STATUS_HPD			BIT(6)
#define ADIHDMI_STATUS_MONITOR_SENSE		BIT(5)
#define ADIHDMI_STATUS_I2S_32BIT_MODE		BIT(3)

#define ADIHDMI_PACKET_ENABLE_N_CTS		BIT(8+6)
#define ADIHDMI_PACKET_ENABLE_AUDIO_SAMPLE	BIT(8+5)
#define ADIHDMI_PACKET_ENABLE_AVI_INFOFRAME	BIT(8+4)
#define ADIHDMI_PACKET_ENABLE_AUDIO_INFOFRAME	BIT(8+3)
#define ADIHDMI_PACKET_ENABLE_GC		BIT(7)
#define ADIHDMI_PACKET_ENABLE_SPD		BIT(6)
#define ADIHDMI_PACKET_ENABLE_MPEG		BIT(5)
#define ADIHDMI_PACKET_ENABLE_ACP		BIT(4)
#define ADIHDMI_PACKET_ENABLE_ISRC		BIT(3)
#define ADIHDMI_PACKET_ENABLE_GM		BIT(2)
#define ADIHDMI_PACKET_ENABLE_SPARE2		BIT(1)
#define ADIHDMI_PACKET_ENABLE_SPARE1		BIT(0)

#define ADIHDMI_REG_POWER2_HDP_SRC_MASK		0xc0
#define ADIHDMI_REG_POWER2_HDP_SRC_BOTH		0x00
#define ADIHDMI_REG_POWER2_HDP_SRC_HDP		0x40
#define ADIHDMI_REG_POWER2_HDP_SRC_CEC		0x80
#define ADIHDMI_REG_POWER2_HDP_SRC_NONE		0xc0
#define ADIHDMI_REG_POWER2_TDMS_ENABLE		BIT(4)
#define ADIHDMI_REG_POWER2_GATE_INPUT_CLK	BIT(0)

#define ADIHDMI_LOW_REFRESH_RATE_NONE		0x0
#define ADIHDMI_LOW_REFRESH_RATE_24HZ		0x1
#define ADIHDMI_LOW_REFRESH_RATE_25HZ		0x2
#define ADIHDMI_LOW_REFRESH_RATE_30HZ		0x3

#define ADIHDMI_AUDIO_CFG3_LEN_MASK		0x0f
#define ADIHDMI_I2C_FREQ_ID_CFG_RATE_MASK	0xf0

#define ADIHDMI_AUDIO_SOURCE_I2S		0
#define ADIHDMI_AUDIO_SOURCE_SPDIF		1

#define ADIHDMI_I2S_FORMAT_I2S			0
#define ADIHDMI_I2S_FORMAT_RIGHT_J		1
#define ADIHDMI_I2S_FORMAT_LEFT_J		2

#define ADIHDMI_PACKET(p, x)	    ((p) * 0x20 + (x))
#define ADIHDMI_PACKET_SDP(x)	    ADIHDMI_PACKET(0, x)
#define ADIHDMI_PACKET_MPEG(x)	    ADIHDMI_PACKET(1, x)
#define ADIHDMI_PACKET_ACP(x)	    ADIHDMI_PACKET(2, x)
#define ADIHDMI_PACKET_ISRC1(x)	    ADIHDMI_PACKET(3, x)
#define ADIHDMI_PACKET_ISRC2(x)	    ADIHDMI_PACKET(4, x)
#define ADIHDMI_PACKET_GM(x)	    ADIHDMI_PACKET(5, x)
#define ADIHDMI_PACKET_SPARE(x)	    ADIHDMI_PACKET(6, x)

enum adihdmi_input_clock {
	ADIHDMI_INPUT_CLOCK_1X,
	ADIHDMI_INPUT_CLOCK_2X,
	ADIHDMI_INPUT_CLOCK_DDR,
};

enum adihdmi_input_justification {
	ADIHDMI_INPUT_JUSTIFICATION_EVENLY = 0,
	ADIHDMI_INPUT_JUSTIFICATION_RIGHT = 1,
	ADIHDMI_INPUT_JUSTIFICATION_LEFT = 2,
};

enum adihdmi_input_sync_pulse {
	ADIHDMI_INPUT_SYNC_PULSE_DE = 0,
	ADIHDMI_INPUT_SYNC_PULSE_HSYNC = 1,
	ADIHDMI_INPUT_SYNC_PULSE_VSYNC = 2,
	ADIHDMI_INPUT_SYNC_PULSE_NONE = 3,
};

/**
 * enum adihdmi_sync_polarity - Polarity for the input sync signals
 * @ADIHDMI_SYNC_POLARITY_PASSTHROUGH:  Sync polarity matches that of
 *				       the currently configured mode.
 * @ADIHDMI_SYNC_POLARITY_LOW:	    Sync polarity is low
 * @ADIHDMI_SYNC_POLARITY_HIGH:	    Sync polarity is high
 *
 * If the polarity is set to either LOW or HIGH the driver will configure the
 * ADIHDMI to internally invert the sync signal if required to match the sync
 * polarity setting for the currently selected output mode.
 *
 * If the polarity is set to PASSTHROUGH, the ADIHDMI will route the signal
 * unchanged. This is used when the upstream graphics core already generates
 * the sync signals with the correct polarity.
 */
enum adihdmi_sync_polarity {
	ADIHDMI_SYNC_POLARITY_PASSTHROUGH,
	ADIHDMI_SYNC_POLARITY_LOW,
	ADIHDMI_SYNC_POLARITY_HIGH,
};

/**
 * struct adihdmi_link_config - Describes adihdmi hardware configuration
 * @input_color_depth:		Number of bits per color component (8, 10 or 12)
 * @input_colorspace:		The input colorspace (RGB, YUV444, YUV422)
 * @input_clock:		The input video clock style (1x, 2x, DDR)
 * @input_style:		The input component arrangement variant
 * @input_justification:	Video input format bit justification
 * @clock_delay:		Clock delay for the input clock (in ps)
 * @embedded_sync:		Video input uses BT.656-style embedded sync
 * @sync_pulse:			Select the sync pulse
 * @vsync_polarity:		vsync input signal configuration
 * @hsync_polarity:		hsync input signal configuration
 */
struct adihdmi_link_config {
	unsigned int input_color_depth;
	enum hdmi_colorspace input_colorspace;
	enum adihdmi_input_clock input_clock;
	unsigned int input_style;
	enum adihdmi_input_justification input_justification;

	int clock_delay;

	bool embedded_sync;
	enum adihdmi_input_sync_pulse sync_pulse;
	enum adihdmi_sync_polarity vsync_polarity;
	enum adihdmi_sync_polarity hsync_polarity;
};

/**
 * enum adihdmi_csc_scaling - Scaling factor for the ADIHDMI CSC
 * @ADIHDMI_CSC_SCALING_1: CSC results are not scaled
 * @ADIHDMI_CSC_SCALING_2: CSC results are scaled by a factor of two
 * @ADIHDMI_CSC_SCALING_4: CSC results are scalled by a factor of four
 */
enum adihdmi_csc_scaling {
	ADIHDMI_CSC_SCALING_1 = 0,
	ADIHDMI_CSC_SCALING_2 = 1,
	ADIHDMI_CSC_SCALING_4 = 2,
};

/**
 * struct adihdmi_video_config - Describes adihdmi hardware configuration
 * @csc_enable:			Whether to enable color space conversion
 * @csc_scaling_factor:		Color space conversion scaling factor
 * @csc_coefficents:		Color space conversion coefficents
 * @hdmi_mode:			Whether to use HDMI or DVI output mode
 * @avi_infoframe:		HDMI infoframe
 */
struct adihdmi_video_config {
	bool csc_enable;
	enum adihdmi_csc_scaling csc_scaling_factor;
	const uint16_t *csc_coefficents;

	bool hdmi_mode;
	struct hdmi_avi_infoframe avi_infoframe;
};

#endif /* __DRM_I2C_ADIHDMI_H__ */
