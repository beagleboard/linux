/*
 * Analog Devices ADIHDMI HDMI transmitter driver
 *
 * Copyright 2012 Analog Devices Inc.
 * Copyright 2015 Konsulko Group
 *
 * Licensed under the GPL-2.
 */

#include <linux/component.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_encoder_slave.h>
#include <drm_of.h>

#include "adihdmi.h"

#define ADIHDMI_INFOFRAME_PACKETS   (0x7900)

struct adihdmi {
	struct i2c_client *i2c_main;
	struct i2c_client *i2c_edid;

	struct regmap *regmap;
	struct regmap *packet_memory_regmap;
	enum drm_connector_status status;
	bool powered;

	unsigned int f_tmds;

	unsigned int current_edid_segment;
	uint8_t edid_buf[256];
	bool edid_read;

	wait_queue_head_t wq;
	struct drm_encoder *encoder;

	bool embedded_sync;
	enum adihdmi_sync_polarity vsync_polarity;
	enum adihdmi_sync_polarity hsync_polarity;
	bool rgb;

	struct edid *edid;

	struct gpio_desc *gpio_pd;
};

struct adihdmi2 {
	struct adihdmi base;
	struct drm_encoder encoder;
	struct drm_connector connector;
};

/* ADI recommended values for proper operation. */
static const struct reg_default adihdmi_fixed_registers[] = {
	{ 0x98, 0x03 },
	{ 0x9a, 0xe0 },
	{ 0x9c, 0x30 },
	{ 0x9d, 0x61 },
	{ 0xa2, 0xa4 },
	{ 0xa3, 0xa4 },
	{ 0xe0, 0xd0 },
	{ 0xf9, 0x00 },
	{ 0x55, 0x02 },
};

/* -----------------------------------------------------------------------------
 * Register access
 */

static const uint8_t adihdmi_register_defaults[] = {
	0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 00 */
	0x00, 0x00, 0x01, 0x0e, 0xbc, 0x18, 0x01, 0x13,
	0x25, 0x37, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 10 */
	0x46, 0x62, 0x04, 0xa8, 0x00, 0x00, 0x1c, 0x84,
	0x1c, 0xbf, 0x04, 0xa8, 0x1e, 0x70, 0x02, 0x1e, /* 20 */
	0x00, 0x00, 0x04, 0xa8, 0x08, 0x12, 0x1b, 0xac,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 30 */
	0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0xb0,
	0x00, 0x50, 0x90, 0x7e, 0x79, 0x70, 0x00, 0x00, /* 40 */
	0x00, 0xa8, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x02, 0x0d, 0x00, 0x00, 0x00, 0x00, /* 50 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 60 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x01, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 70 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 80 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, /* 90 */
	0x0b, 0x02, 0x00, 0x18, 0x5a, 0x60, 0x00, 0x00,
	0x00, 0x00, 0x80, 0x80, 0x08, 0x04, 0x00, 0x00, /* a0 */
	0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x40, 0x14,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* b0 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* c0 */
	0x00, 0x03, 0x00, 0x00, 0x02, 0x00, 0x01, 0x04,
	0x30, 0xff, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, /* d0 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x01,
	0x80, 0x75, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, /* e0 */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x75, 0x11, 0x00, /* f0 */
	0x00, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static bool adihdmi_register_volatile(struct device *dev, unsigned int reg)
{
	switch (reg) {
		case ADIHDMI_REG_CHIP_REVISION:
		case ADIHDMI_REG_SPDIF_FREQ:
		case ADIHDMI_REG_CTS_AUTOMATIC1:
		case ADIHDMI_REG_CTS_AUTOMATIC2:
		case ADIHDMI_REG_VIC_DETECTED:
		case ADIHDMI_REG_VIC_SEND:
		case ADIHDMI_REG_AUX_VIC_DETECTED:
		case ADIHDMI_REG_STATUS:
		case ADIHDMI_REG_GC(1):
		case ADIHDMI_REG_INT(0):
		case ADIHDMI_REG_INT(1):
		case ADIHDMI_REG_PLL_STATUS:
		case ADIHDMI_REG_AN(0):
		case ADIHDMI_REG_AN(1):
		case ADIHDMI_REG_AN(2):
		case ADIHDMI_REG_AN(3):
		case ADIHDMI_REG_AN(4):
		case ADIHDMI_REG_AN(5):
		case ADIHDMI_REG_AN(6):
		case ADIHDMI_REG_AN(7):
		case ADIHDMI_REG_HDCP_STATUS:
		case ADIHDMI_REG_BCAPS:
		case ADIHDMI_REG_BKSV(0):
		case ADIHDMI_REG_BKSV(1):
		case ADIHDMI_REG_BKSV(2):
		case ADIHDMI_REG_BKSV(3):
		case ADIHDMI_REG_BKSV(4):
		case ADIHDMI_REG_DDC_STATUS:
		case ADIHDMI_REG_BSTATUS(0):
		case ADIHDMI_REG_BSTATUS(1):
		case ADIHDMI_REG_CHIP_ID_HIGH:
		case ADIHDMI_REG_CHIP_ID_LOW:
			return true;
	}

	return false;
}

static const struct regmap_config adihdmi_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = 0xff,
	.cache_type = REGCACHE_RBTREE,
	.reg_defaults_raw = adihdmi_register_defaults,
	.num_reg_defaults_raw = ARRAY_SIZE(adihdmi_register_defaults),

	.volatile_reg = adihdmi_register_volatile,
};

/* -----------------------------------------------------------------------------
 * Hardware configuration
 */

	static void adihdmi_audio_setup(struct adihdmi * adihdmi)
{
	/* Select I2S. */
	regmap_write(adihdmi->regmap, ADIHDMI_REG_AUDIO_SOURCE, 0x01);
	regmap_write(adihdmi->regmap, ADIHDMI_REG_I2S_CONFIG, 0x84);

	/* Setup clocks for 48KHz. */
	regmap_write(adihdmi->regmap, ADIHDMI_REG_N0, 0x00);
	regmap_write(adihdmi->regmap, ADIHDMI_REG_N1, 0x18);
	regmap_write(adihdmi->regmap, ADIHDMI_REG_N2, 0x00);
	regmap_update_bits(adihdmi->regmap, ADIHDMI_REG_I2C_FREQ_ID_CFG, 0xF0, 0x20);

	/* Set audio word length to 24 bits. */
	regmap_update_bits(adihdmi->regmap, ADIHDMI_REG_AUDIO_CFG3, 0x0F, 0x0B);

	/* Update audio infoframe. */
	regmap_update_bits(adihdmi->regmap, ADIHDMI_REG_INFOFRAME_UPDATE, 0x20, 0x20);
	regmap_update_bits(adihdmi->regmap, ADIHDMI_REG_AUDIO_INFOFRAME(0), 0x07, 0x01);
	regmap_update_bits(adihdmi->regmap, ADIHDMI_REG_AUDIO_INFOFRAME(3), 0x1F, 0x00);
	regmap_update_bits(adihdmi->regmap, ADIHDMI_REG_INFOFRAME_UPDATE, 0x20, 0x00);
}

static void adihdmi_set_colormap(struct adihdmi *adihdmi, bool enable,
		const uint16_t *coeff,
		unsigned int scaling_factor)
{
	unsigned int i;

	regmap_update_bits(adihdmi->regmap, ADIHDMI_REG_CSC_UPPER(1),
			ADIHDMI_CSC_UPDATE_MODE, ADIHDMI_CSC_UPDATE_MODE);

	if (enable) {
		for (i = 0; i < 12; ++i) {
			regmap_update_bits(adihdmi->regmap,
					ADIHDMI_REG_CSC_UPPER(i),
					0x1f, coeff[i] >> 8);
			regmap_write(adihdmi->regmap,
					ADIHDMI_REG_CSC_LOWER(i),
					coeff[i] & 0xff);
		}
	}

	if (enable)
		regmap_update_bits(adihdmi->regmap, ADIHDMI_REG_CSC_UPPER(0),
				0xe0, 0x80 | (scaling_factor << 5));
	else
		regmap_update_bits(adihdmi->regmap, ADIHDMI_REG_CSC_UPPER(0),
				0x80, 0x00);

	regmap_update_bits(adihdmi->regmap, ADIHDMI_REG_CSC_UPPER(1),
			ADIHDMI_CSC_UPDATE_MODE, 0);
}

static int adihdmi_packet_enable(struct adihdmi *adihdmi, unsigned int packet)
{
	if (packet & 0xff)
		regmap_update_bits(adihdmi->regmap, ADIHDMI_REG_PACKET_ENABLE0,
				packet, 0xff);

	if (packet & 0xff00) {
		packet >>= 8;
		regmap_update_bits(adihdmi->regmap, ADIHDMI_REG_PACKET_ENABLE1,
				packet, 0xff);
	}

	return 0;
}

static int adihdmi_packet_disable(struct adihdmi *adihdmi, unsigned int packet)
{
	if (packet & 0xff)
		regmap_update_bits(adihdmi->regmap, ADIHDMI_REG_PACKET_ENABLE0,
				packet, 0x00);

	if (packet & 0xff00) {
		packet >>= 8;
		regmap_update_bits(adihdmi->regmap, ADIHDMI_REG_PACKET_ENABLE1,
				packet, 0x00);
	}

	return 0;
}

/* Coefficients for adihdmi color space conversion */
static const uint16_t adihdmi_csc_ycbcr_to_rgb[] = {
	0x0734, 0x04ad, 0x0000, 0x1c1b,
	0x1ddc, 0x04ad, 0x1f24, 0x0135,
	0x0000, 0x04ad, 0x087c, 0x1b77,
};

static void adihdmi_set_config_csc(struct adihdmi *adihdmi,
		struct drm_connector *connector,
		bool rgb)
{
	struct adihdmi_video_config config;
	bool output_format_422, output_format_ycbcr;
	unsigned int mode;
	uint8_t infoframe[17];

	if (adihdmi->edid)
		config.hdmi_mode = drm_detect_hdmi_monitor(adihdmi->edid);
	else
		config.hdmi_mode = false;

	hdmi_avi_infoframe_init(&config.avi_infoframe);

	config.avi_infoframe.scan_mode = HDMI_SCAN_MODE_UNDERSCAN;

	if (rgb) {
		config.csc_enable = false;
		config.avi_infoframe.colorspace = HDMI_COLORSPACE_RGB;
	} else {
		config.csc_scaling_factor = ADIHDMI_CSC_SCALING_4;
		config.csc_coefficents = adihdmi_csc_ycbcr_to_rgb;

		if ((connector->display_info.color_formats &
					DRM_COLOR_FORMAT_YCRCB422) &&
				config.hdmi_mode) {
			config.csc_enable = false;
			config.avi_infoframe.colorspace =
				HDMI_COLORSPACE_YUV422;
		} else {
			config.csc_enable = true;
			config.avi_infoframe.colorspace = HDMI_COLORSPACE_RGB;
		}
	}

	if (config.hdmi_mode) {
		mode = ADIHDMI_HDMI_CFG_MODE_HDMI;

		switch (config.avi_infoframe.colorspace) {
			case HDMI_COLORSPACE_YUV444:
				output_format_422 = false;
				output_format_ycbcr = true;
				break;
			case HDMI_COLORSPACE_YUV422:
				output_format_422 = true;
				output_format_ycbcr = true;
				break;
			default:
				output_format_422 = false;
				output_format_ycbcr = false;
				break;
		}
	} else {
		mode = ADIHDMI_HDMI_CFG_MODE_DVI;
		output_format_422 = false;
		output_format_ycbcr = false;
	}

	adihdmi_packet_disable(adihdmi, ADIHDMI_INFOFRAME_PACKETS);

	adihdmi_set_colormap(adihdmi, config.csc_enable,
			config.csc_coefficents,
			config.csc_scaling_factor);

	regmap_update_bits(adihdmi->regmap, ADIHDMI_REG_VIDEO_INPUT_CFG1, 0x81,
			(output_format_422 << 7) | output_format_ycbcr);

	regmap_update_bits(adihdmi->regmap, ADIHDMI_REG_HDCP_HDMI_CFG,
			ADIHDMI_HDMI_CFG_MODE_MASK, mode);

	hdmi_avi_infoframe_pack(&config.avi_infoframe, infoframe,
			sizeof(infoframe));

	/* The AVI infoframe id is not configurable */
	regmap_bulk_write(adihdmi->regmap, ADIHDMI_REG_AVI_INFOFRAME_VERSION,
			infoframe + 1, sizeof(infoframe) - 1);

	adihdmi_packet_enable(adihdmi, ADIHDMI_INFOFRAME_PACKETS);
}

static void adihdmi_set_link_config(struct adihdmi *adihdmi,
		const struct adihdmi_link_config *config)
{
	/*
	 * The input style values documented in the datasheet don't match the
	 * hardware register field values :-(
	 */
	static const unsigned int input_styles[4] = { 0, 2, 1, 3 };

	unsigned int clock_delay;
	unsigned int color_depth;
	unsigned int input_id;

	clock_delay = (config->clock_delay + 1200) / 400;
	color_depth = config->input_color_depth == 8 ? 3
		: (config->input_color_depth == 10 ? 1 : 2);

	/* TODO Support input ID 6 */
	if (config->input_colorspace != HDMI_COLORSPACE_YUV422)
		input_id = config->input_clock == ADIHDMI_INPUT_CLOCK_DDR
			? 5 : 0;
	else if (config->input_clock == ADIHDMI_INPUT_CLOCK_DDR)
		input_id = config->embedded_sync ? 8 : 7;
	else if (config->input_clock == ADIHDMI_INPUT_CLOCK_2X)
		input_id = config->embedded_sync ? 4 : 3;
	else
		input_id = config->embedded_sync ? 2 : 1;

	regmap_update_bits(adihdmi->regmap, ADIHDMI_REG_I2C_FREQ_ID_CFG, 0xf,
			input_id);
	regmap_update_bits(adihdmi->regmap, ADIHDMI_REG_VIDEO_INPUT_CFG1, 0x7e,
			(color_depth << 4) |
			(input_styles[config->input_style] << 2));
	regmap_write(adihdmi->regmap, ADIHDMI_REG_VIDEO_INPUT_CFG2,
			config->input_justification << 3);
	regmap_write(adihdmi->regmap, ADIHDMI_REG_TIMING_GEN_SEQ,
			config->sync_pulse << 2);

	regmap_write(adihdmi->regmap, 0xba, clock_delay << 5);

	adihdmi->embedded_sync = config->embedded_sync;
	adihdmi->hsync_polarity = config->hsync_polarity;
	adihdmi->vsync_polarity = config->vsync_polarity;
	adihdmi->rgb = config->input_colorspace == HDMI_COLORSPACE_RGB;
}

static void adihdmi_power_on(struct adihdmi *adihdmi)
{
	adihdmi->current_edid_segment = -1;

	regmap_write(adihdmi->regmap, ADIHDMI_REG_INT(0),
			ADIHDMI_INT0_EDID_READY);
	regmap_write(adihdmi->regmap, ADIHDMI_REG_INT(1),
			ADIHDMI_INT1_DDC_ERROR);
	regmap_update_bits(adihdmi->regmap, ADIHDMI_REG_POWER,
			ADIHDMI_POWER_POWER_DOWN, 0);

	/*
	 * Per spec it is allowed to pulse the HDP signal to indicate that the
	 * EDID information has changed. Some monitors do this when they wakeup
	 * from standby or are enabled. When the HDP goes low the adihdmi is
	 * reset and the outputs are disabled which might cause the monitor to
	 * go to standby again. To avoid this we ignore the HDP pin for the
	 * first few seconds after enabling the output.
	 */
	regmap_update_bits(adihdmi->regmap, ADIHDMI_REG_POWER2,
			ADIHDMI_REG_POWER2_HDP_SRC_MASK,
			ADIHDMI_REG_POWER2_HDP_SRC_NONE);

	/*
	 * Most of the registers are reset during power down or when HPD is low.
	 */
	regcache_sync(adihdmi->regmap);

	adihdmi->powered = true;
}

static void adihdmi_power_off(struct adihdmi *adihdmi)
{
	/* TODO: setup additional power down modes */
	regmap_update_bits(adihdmi->regmap, ADIHDMI_REG_POWER,
			ADIHDMI_POWER_POWER_DOWN,
			ADIHDMI_POWER_POWER_DOWN);
	regcache_mark_dirty(adihdmi->regmap);

	adihdmi->powered = false;
}

/* -----------------------------------------------------------------------------
 * Interrupt and hotplug detection
 */

static bool adihdmi_hpd(struct adihdmi *adihdmi)
{
	unsigned int irq0;
	int ret;

	ret = regmap_read(adihdmi->regmap, ADIHDMI_REG_INT(0), &irq0);
	if (ret < 0)
		return false;

	if (irq0 & ADIHDMI_INT0_HDP) {
		regmap_write(adihdmi->regmap, ADIHDMI_REG_INT(0),
				ADIHDMI_INT0_HDP);
		return true;
	}

	return false;
}

static int adihdmi_irq_process(struct adihdmi *adihdmi)
{
	unsigned int irq0, irq1;
	int ret;

	ret = regmap_read(adihdmi->regmap, ADIHDMI_REG_INT(0), &irq0);
	if (ret < 0)
		return ret;

	ret = regmap_read(adihdmi->regmap, ADIHDMI_REG_INT(1), &irq1);
	if (ret < 0)
		return ret;

	regmap_write(adihdmi->regmap, ADIHDMI_REG_INT(0), irq0);
	regmap_write(adihdmi->regmap, ADIHDMI_REG_INT(1), irq1);

	if (irq0 & ADIHDMI_INT0_HDP)
		drm_helper_hpd_irq_event(adihdmi->encoder->dev);

	if (irq0 & ADIHDMI_INT0_EDID_READY || irq1 & ADIHDMI_INT1_DDC_ERROR) {
		adihdmi->edid_read = true;

		if (adihdmi->i2c_main->irq)
			wake_up_all(&adihdmi->wq);
	}

	return 0;
}

static irqreturn_t adihdmi_irq_handler(int irq, void *devid)
{
	struct adihdmi *adihdmi = devid;
	int ret;

	ret = adihdmi_irq_process(adihdmi);
	return ret < 0 ? IRQ_NONE : IRQ_HANDLED;
}

/* -----------------------------------------------------------------------------
 * EDID retrieval
 */

static int adihdmi_wait_for_edid(struct adihdmi *adihdmi, int timeout)
{
	int ret;

	if (adihdmi->i2c_main->irq) {
		ret = wait_event_interruptible_timeout(adihdmi->wq,
				adihdmi->edid_read, msecs_to_jiffies(timeout));
	} else {
		for (; timeout > 0; timeout -= 25) {
			ret = adihdmi_irq_process(adihdmi);
			if (ret < 0)
				break;

			if (adihdmi->edid_read)
				break;

			msleep(25);
		}
	}

	return adihdmi->edid_read ? 0 : -EIO;
}

static int adihdmi_get_edid_block(void *data, u8 *buf, unsigned int block,
		size_t len)
{
	struct adihdmi *adihdmi = data;
	struct i2c_msg xfer[2];
	uint8_t offset;
	unsigned int i;
	int ret;

	if (len > 128)
		return -EINVAL;

	if (adihdmi->current_edid_segment != block / 2) {
		unsigned int status;

		ret = regmap_read(adihdmi->regmap, ADIHDMI_REG_DDC_STATUS,
				&status);
		if (ret < 0)
			return ret;

		if (status != 2) {
			adihdmi->edid_read = false;
			regmap_write(adihdmi->regmap, ADIHDMI_REG_EDID_SEGMENT,
					block);
			ret = adihdmi_wait_for_edid(adihdmi, 200);
			if (ret < 0)
				return ret;
		}

		/* Break this apart, hopefully more I2C controllers will
		 * support 64 byte transfers than 256 byte transfers
		 */

		xfer[0].addr = adihdmi->i2c_edid->addr;
		xfer[0].flags = 0;
		xfer[0].len = 1;
		xfer[0].buf = &offset;
		xfer[1].addr = adihdmi->i2c_edid->addr;
		xfer[1].flags = I2C_M_RD;
		xfer[1].len = 64;
		xfer[1].buf = adihdmi->edid_buf;

		offset = 0;

		for (i = 0; i < 4; ++i) {
			ret = i2c_transfer(adihdmi->i2c_edid->adapter, xfer,
					ARRAY_SIZE(xfer));
			if (ret < 0)
				return ret;
			else if (ret != 2)
				return -EIO;

			xfer[1].buf += 64;
			offset += 64;
		}

		adihdmi->current_edid_segment = block / 2;
	}

	if (block % 2 == 0)
		memcpy(buf, adihdmi->edid_buf, len);
	else
		memcpy(buf, adihdmi->edid_buf + 128, len);

	return 0;
}

static int adihdmi_mode_valid(struct drm_display_mode *mode)
{
	if (mode->clock > 165000)
		return MODE_CLOCK_HIGH;

	return MODE_OK;
}

/* -----------------------------------------------------------------------------
 * DT and private structure operations
 */

#define conn_to_adihdmi2(x) \
	container_of(x, struct adihdmi2, connector);

#define enc_to_adihdmi2(x) \
	container_of(x, struct adihdmi2, encoder);

#define enc_to_adihdmi(x) \
	(&(container_of(x, struct adihdmi2, encoder)->base))

static int adihdmi_parse_dt(struct device_node *np,
		struct adihdmi_link_config *config)
{
	memset(config, 0, sizeof(*config));

	config->input_color_depth = 8;

	config->input_colorspace = HDMI_COLORSPACE_RGB;
	//config->input_colorspace = HDMI_COLORSPACE_YUV422;
	//config->input_colorspace = HDMI_COLORSPACE_YUV444;

	config->input_clock = ADIHDMI_INPUT_CLOCK_1X;
	//config->input_clock = ADIHDMI_INPUT_CLOCK_2X;
	//config->input_clock = ADIHDMI_INPUT_CLOCK_DDR;

	if (config->input_colorspace == HDMI_COLORSPACE_YUV422 ||
			config->input_clock != ADIHDMI_INPUT_CLOCK_1X) {

		config->input_style = 1;
		//config->input_justification = ADIHDMI_INPUT_JUSTIFICATION_LEFT;
		config->input_justification = ADIHDMI_INPUT_JUSTIFICATION_EVENLY;
		//config->input_justification = ADIHDMI_INPUT_JUSTIFICATION_RIGHT;

	} else {
		config->input_style = 1;
		config->input_justification = ADIHDMI_INPUT_JUSTIFICATION_LEFT;
	}

	config->clock_delay = 0;
	config->embedded_sync = 0;

	/* Hardcode the sync pulse configurations for now. */
	config->sync_pulse = ADIHDMI_INPUT_SYNC_PULSE_NONE;
	config->vsync_polarity = ADIHDMI_SYNC_POLARITY_PASSTHROUGH;
	config->hsync_polarity = ADIHDMI_SYNC_POLARITY_PASSTHROUGH;

	return 0;
}

static const int edid_i2c_addr = 0x7e;
static const int packet_i2c_addr = 0x70;
static const int cec_i2c_addr = 0x78;

static int adihdmi_create(struct i2c_client *i2c, struct adihdmi *adihdmi)
{
	struct adihdmi_link_config link_config;
	struct device *dev = &i2c->dev;
	unsigned int val;
	int ret;

	adihdmi->powered = false;
	adihdmi->status = connector_status_disconnected;

	ret = adihdmi_parse_dt(NULL, &link_config);
	if (ret)
	{
		pr_err("%s - %d - Bad parse\n", __FUNCTION__, __LINE__);
		return -EINVAL;
	}

	/*
	 * The power down GPIO is optional. If present, toggle it from active to
	 * inactive to wake up the encoder.
	 */
	adihdmi->gpio_pd = devm_gpiod_get_optional(dev, "pd", GPIOD_OUT_HIGH);
	if (IS_ERR(adihdmi->gpio_pd))
	{
		pr_err("%s - %d - Bad PD GPIO\n", __FUNCTION__, __LINE__);
		return PTR_ERR(adihdmi->gpio_pd);
	}

	if (adihdmi->gpio_pd) {
		mdelay(5);
		gpiod_set_value_cansleep(adihdmi->gpio_pd, 0);
	}

	adihdmi->regmap = devm_regmap_init_i2c(i2c, &adihdmi_regmap_config);
	if (IS_ERR(adihdmi->regmap))
	{
		pr_err("%s - %d - Bad reg map init\n", __FUNCTION__, __LINE__);
		return PTR_ERR(adihdmi->regmap);
	}

	ret = regmap_read(adihdmi->regmap, ADIHDMI_REG_CHIP_REVISION, &val);
	if (ret)
	{
		pr_err("%s - %d - Bad reg map read\n", __FUNCTION__, __LINE__);
		return ret;
	}
	dev_dbg(dev, "Rev. %d\n", val);

	ret = regmap_register_patch(adihdmi->regmap, adihdmi_fixed_registers,
			ARRAY_SIZE(adihdmi_fixed_registers));
	if (ret)
	{
		pr_err("%s - %d - Bad reg map patch\n", __FUNCTION__, __LINE__);
		return ret;
	}

	regmap_write(adihdmi->regmap, ADIHDMI_REG_EDID_I2C_ADDR, edid_i2c_addr);
	regmap_write(adihdmi->regmap, ADIHDMI_REG_PACKET_I2C_ADDR,
			packet_i2c_addr);
	regmap_write(adihdmi->regmap, ADIHDMI_REG_CEC_I2C_ADDR, cec_i2c_addr);
	adihdmi_packet_disable(adihdmi, 0xffff);

	adihdmi->i2c_main = i2c;
	adihdmi->i2c_edid = i2c_new_dummy(i2c->adapter, edid_i2c_addr >> 1);
	if (!adihdmi->i2c_edid)
	{
		pr_err("%s - %d - No mem for EDID\n", __FUNCTION__, __LINE__);
		return -ENOMEM;
	}

	if (i2c->irq) {
		init_waitqueue_head(&adihdmi->wq);

		ret = devm_request_threaded_irq(dev, i2c->irq, NULL,
				adihdmi_irq_handler,
				IRQF_ONESHOT, dev_name(dev),
				adihdmi);
		if (ret)
		{
			pr_err("%s - %d - Bad IRQ thread request\n", __FUNCTION__, __LINE__);
			goto err_i2c_unregister_device;
		}
	}

	/* CEC is unused for now */
	regmap_write(adihdmi->regmap, ADIHDMI_REG_CEC_CTRL,
			ADIHDMI_CEC_CTRL_POWER_DOWN);

	adihdmi_power_off(adihdmi);

	adihdmi_set_link_config(adihdmi, &link_config);

	adihdmi_audio_setup(adihdmi);

	return 0;

err_i2c_unregister_device:
	i2c_unregister_device(adihdmi->i2c_edid);

	return ret;
}

static void adihdmi_destroy(struct adihdmi *priv)
{
	i2c_unregister_device(priv->i2c_edid);
}

/* -----------------------------------------------------------------------------
 * Encoder operations
 */

static int adihdmi_encoder_get_modes(struct adihdmi *adihdmi,
		struct drm_connector *connector)
{
	struct edid *edid;
	unsigned int count;

	/* Reading the EDID only works if the device is powered */
	if (!adihdmi->powered) {
		regmap_write(adihdmi->regmap, ADIHDMI_REG_INT(0),
				ADIHDMI_INT0_EDID_READY);
		regmap_write(adihdmi->regmap, ADIHDMI_REG_INT(1),
				ADIHDMI_INT1_DDC_ERROR);
		regmap_update_bits(adihdmi->regmap, ADIHDMI_REG_POWER,
				ADIHDMI_POWER_POWER_DOWN, 0);
		adihdmi->current_edid_segment = -1;
	}

	edid = drm_do_get_edid(connector, adihdmi_get_edid_block, adihdmi);

	if (!adihdmi->powered)
		regmap_update_bits(adihdmi->regmap, ADIHDMI_REG_POWER,
				ADIHDMI_POWER_POWER_DOWN,
				ADIHDMI_POWER_POWER_DOWN);

	kfree(adihdmi->edid);
	adihdmi->edid = edid;
	if (!edid)
	{
		pr_err("%s - %d - No EDID\n", __FUNCTION__, __LINE__);
		return 0;
	}

	drm_mode_connector_update_edid_property(connector, edid);
	count = drm_add_edid_modes(connector, edid);

	adihdmi_set_config_csc(adihdmi, connector, adihdmi->rgb);

	return count;
}

static void adihdmi_encoder_dpms(struct drm_encoder *encoder, int mode)
{
	struct adihdmi2 *priv2 = enc_to_adihdmi2(encoder);

	if (mode == DRM_MODE_DPMS_ON)
		adihdmi_power_on(&priv2->base);
	else
		adihdmi_power_off(&priv2->base);
}

	static enum drm_connector_status
adihdmi_encoder_detect(struct adihdmi *adihdmi,
		struct drm_connector *connector)
{
	enum drm_connector_status status;
	unsigned int val;
	bool hpd;
	int ret;

	ret = regmap_read(adihdmi->regmap, ADIHDMI_REG_STATUS, &val);
	if (ret < 0)
	{
		pr_err("%s - %d - Disconnected\n", __FUNCTION__, __LINE__);
		return connector_status_disconnected;
	}

	if (val & ADIHDMI_STATUS_HPD)
		status = connector_status_connected;
	else
		status = connector_status_disconnected;

	hpd = adihdmi_hpd(adihdmi);

	/* The chip resets itself when the cable is disconnected, so in case
	 * there is a pending HPD interrupt and the cable is connected there was
	 * at least one transition from disconnected to connected and the chip
	 * has to be reinitialized. */
	if (status == connector_status_connected && hpd && adihdmi->powered) {
		regcache_mark_dirty(adihdmi->regmap);
		adihdmi_power_on(adihdmi);
		adihdmi_encoder_get_modes(adihdmi, connector);
		if (adihdmi->status == connector_status_connected)
			status = connector_status_disconnected;
	} else {
		/* Renable HDP sensing */
		regmap_update_bits(adihdmi->regmap, ADIHDMI_REG_POWER2,
				ADIHDMI_REG_POWER2_HDP_SRC_MASK,
				ADIHDMI_REG_POWER2_HDP_SRC_BOTH);
	}

	adihdmi->status = status;
	return status;
}

static bool adihdmi_encoder_mode_fixup(struct drm_encoder *encoder, const struct drm_display_mode *mode, struct drm_display_mode *adjusted_mode)
{
	return true;
}

static int adihdmi_encoder_mode_valid(struct drm_encoder *encoder, struct drm_display_mode *mode)
{
	return adihdmi_mode_valid(mode);
}

static void adihdmi_encoder_mode_set(struct drm_encoder *encoder,
		struct drm_display_mode *mode,
		struct drm_display_mode *adj_mode)
{
	unsigned int low_refresh_rate;
	unsigned int hsync_polarity = 0;
	unsigned int vsync_polarity = 0;
	struct adihdmi *adihdmi = enc_to_adihdmi(encoder);

	if (adihdmi->embedded_sync) {
		unsigned int hsync_offset, hsync_len;
		unsigned int vsync_offset, vsync_len;

		hsync_offset = adj_mode->crtc_hsync_start -
			adj_mode->crtc_hdisplay;
		vsync_offset = adj_mode->crtc_vsync_start -
			adj_mode->crtc_vdisplay;
		hsync_len = adj_mode->crtc_hsync_end -
			adj_mode->crtc_hsync_start;
		vsync_len = adj_mode->crtc_vsync_end -
			adj_mode->crtc_vsync_start;

		/* The hardware vsync generator has a off-by-one bug */
		vsync_offset += 1;

		regmap_write(adihdmi->regmap, ADIHDMI_REG_HSYNC_PLACEMENT_MSB,
				((hsync_offset >> 10) & 0x7) << 5);
		regmap_write(adihdmi->regmap, ADIHDMI_REG_SYNC_DECODER(0),
				(hsync_offset >> 2) & 0xff);
		regmap_write(adihdmi->regmap, ADIHDMI_REG_SYNC_DECODER(1),
				((hsync_offset & 0x3) << 6) |
				((hsync_len >> 4) & 0x3f));
		regmap_write(adihdmi->regmap, ADIHDMI_REG_SYNC_DECODER(2),
				((hsync_len & 0xf) << 4) |
				((vsync_offset >> 6) & 0xf));
		regmap_write(adihdmi->regmap, ADIHDMI_REG_SYNC_DECODER(3),
				((vsync_offset & 0x3f) << 2) |
				((vsync_len >> 8) & 0x3));
		regmap_write(adihdmi->regmap, ADIHDMI_REG_SYNC_DECODER(4),
				vsync_len & 0xff);

		hsync_polarity = !(adj_mode->flags & DRM_MODE_FLAG_PHSYNC);
		vsync_polarity = !(adj_mode->flags & DRM_MODE_FLAG_PVSYNC);
	} else {
		enum adihdmi_sync_polarity mode_hsync_polarity;
		enum adihdmi_sync_polarity mode_vsync_polarity;

		/**
		 * If the input signal is always low or always high we want to
		 * invert or let it passthrough depending on the polarity of the
		 * current mode.
		 **/
		if (adj_mode->flags & DRM_MODE_FLAG_NHSYNC)
			mode_hsync_polarity = ADIHDMI_SYNC_POLARITY_LOW;
		else
			mode_hsync_polarity = ADIHDMI_SYNC_POLARITY_HIGH;

		if (adj_mode->flags & DRM_MODE_FLAG_NVSYNC)
			mode_vsync_polarity = ADIHDMI_SYNC_POLARITY_LOW;
		else
			mode_vsync_polarity = ADIHDMI_SYNC_POLARITY_HIGH;

		if (adihdmi->hsync_polarity != mode_hsync_polarity &&
				adihdmi->hsync_polarity !=
				ADIHDMI_SYNC_POLARITY_PASSTHROUGH)
			hsync_polarity = 1;

		if (adihdmi->vsync_polarity != mode_vsync_polarity &&
				adihdmi->vsync_polarity !=
				ADIHDMI_SYNC_POLARITY_PASSTHROUGH)
			vsync_polarity = 1;
	}

	if (mode->vrefresh <= 24000)
		low_refresh_rate = ADIHDMI_LOW_REFRESH_RATE_24HZ;
	else if (mode->vrefresh <= 25000)
		low_refresh_rate = ADIHDMI_LOW_REFRESH_RATE_25HZ;
	else if (mode->vrefresh <= 30000)
		low_refresh_rate = ADIHDMI_LOW_REFRESH_RATE_30HZ;
	else
		low_refresh_rate = ADIHDMI_LOW_REFRESH_RATE_NONE;

	regmap_update_bits(adihdmi->regmap, 0xfb,
			0x6, low_refresh_rate << 1);
	regmap_update_bits(adihdmi->regmap, 0x17,
			0x60, (vsync_polarity << 6) | (hsync_polarity << 5));

	/*
	 * TODO Test first order 4:2:2 to 4:4:4 up conversion method, which is
	 * supposed to give better results.
	 */

			adihdmi->f_tmds = mode->clock;
}

static void adihdmi_encoder_restore(struct drm_encoder *encoder)
{
	pr_debug("%s - %d\n", __FUNCTION__, __LINE__);
}

static void adihdmi_encoder_save(struct drm_encoder *encoder)
{
	pr_debug("%s - %d\n", __FUNCTION__, __LINE__);
}

static void adihdmi_encoder_prepare(struct drm_encoder *encoder)
{
	adihdmi_encoder_dpms(encoder, DRM_MODE_DPMS_OFF);
}

static void adihdmi_encoder_commit(struct drm_encoder *encoder)
{
	adihdmi_encoder_dpms(encoder, DRM_MODE_DPMS_ON);
}

static struct drm_encoder_helper_funcs adihdmi_encoder_helper_funcs = {
	.dpms		= adihdmi_encoder_dpms,
	.save		= adihdmi_encoder_save,
	.restore	= adihdmi_encoder_restore,
	.mode_fixup	= adihdmi_encoder_mode_fixup,
	.prepare	= adihdmi_encoder_prepare,
	.commit		= adihdmi_encoder_commit,
	.mode_set	= adihdmi_encoder_mode_set,
};

static void adihdmi_encoder_destroy(struct drm_encoder *encoder)
{
	struct adihdmi2 *priv = enc_to_adihdmi2(encoder);

	adihdmi_destroy(&priv->base);
	drm_encoder_cleanup(encoder);
}

static const struct drm_encoder_funcs adihdmi_encoder_funcs = {
	.destroy = adihdmi_encoder_destroy,
};

/* -----------------------------------------------------------------------------
 * Slave operations
 */

static int adihdmi_encoder_slave_create_resources(struct drm_encoder *encoder, struct drm_connector *connector)
{
	pr_debug("%s - %d\n", __FUNCTION__, __LINE__);
	return 0;
}

static void adihdmi_encoder_slave_destroy(struct drm_encoder *encoder)
{
	pr_debug("%s - %d\n", __FUNCTION__, __LINE__);
}

	static enum drm_connector_status
adihdmi_encoder_slave_detect(struct drm_encoder *encoder,
		struct drm_connector *connector)
{
	return adihdmi_encoder_detect(enc_to_adihdmi(encoder),
			connector);
}

static int adihdmi_encoder_slave_get_modes(struct drm_encoder *encoder,
		struct drm_connector *connector)
{
	return adihdmi_encoder_get_modes(enc_to_adihdmi(encoder),
			connector);
}


static void adihdmi_encoder_slave_set_config(struct drm_encoder *encoder, void *params)
{
	pr_debug("%s - %d\n", __FUNCTION__, __LINE__);
}

static int adihdmi_encoder_set_property(struct drm_encoder *encoder, struct drm_connector *connector, struct drm_property *property, uint64_t val)
{
	pr_debug("%s - %d\n", __FUNCTION__, __LINE__);
	return 0;
}

static struct drm_encoder_slave_funcs adihdmi_encoder_slave_funcs = {
	.create_resources   = adihdmi_encoder_slave_create_resources,
	.destroy            = adihdmi_encoder_slave_destroy,
	.detect             = adihdmi_encoder_slave_detect,
	.dpms               = adihdmi_encoder_dpms,
	.get_modes          = adihdmi_encoder_slave_get_modes,
	.mode_fixup         = adihdmi_encoder_mode_fixup,
	.mode_set           = adihdmi_encoder_mode_set,
	.mode_valid         = adihdmi_encoder_mode_valid,
	.restore            = adihdmi_encoder_restore,
	.save               = adihdmi_encoder_save,
	.set_config         = adihdmi_encoder_slave_set_config,
	.set_property       = adihdmi_encoder_set_property,
};

/* -----------------------------------------------------------------------------
 * Connector operations
 */

static int adihdmi_connector_get_modes(struct drm_connector *connector)
{
	struct adihdmi2 *priv = conn_to_adihdmi2(connector);

	return adihdmi_encoder_get_modes(&priv->base, connector);
}

static int adihdmi_connector_mode_valid(struct drm_connector *connector,
		struct drm_display_mode *mode)
{
	return adihdmi_mode_valid(mode);
}

	static struct drm_encoder *
adihdmi_connector_best_encoder(struct drm_connector *connector)
{
	struct adihdmi2 *priv = conn_to_adihdmi2(connector);

	return &priv->encoder;
}

static struct drm_connector_helper_funcs adihdmi_connector_helper_funcs = {
	.get_modes          = adihdmi_connector_get_modes,
	.mode_valid         = adihdmi_connector_mode_valid,
	.best_encoder	    = adihdmi_connector_best_encoder,
};

	static enum drm_connector_status
adihdmi_connector_detect(struct drm_connector *connector, bool force)
{
	struct adihdmi2 *priv = conn_to_adihdmi2(connector);

	return adihdmi_encoder_detect(&priv->base, connector);
}

static void adihdmi_connector_destroy(struct drm_connector *connector)
{
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
}

static struct drm_connector_funcs adihdmi_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = adihdmi_connector_detect,
	.destroy = adihdmi_connector_destroy,
};

/* -----------------------------------------------------------------------------
 * Component operations
 */

static int adihdmi_bind(struct device *dev, struct device *master, void *data)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct drm_device *drm = data;
	struct adihdmi2 *priv;
	uint32_t crtcs = 0;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
	{
		pr_err("%s - %d - No memory for ADIHDMI\n", __FUNCTION__, __LINE__);
		return -ENOMEM;
	}

	dev_set_drvdata(dev, priv);

	if (dev->of_node)
		crtcs = drm_of_find_possible_crtcs(drm, dev->of_node);

	/* If no CRTCs were found, fall back to our old behaviour */
	if (crtcs == 0) {
		dev_warn(dev, "Falling back to first CRTC\n");
		crtcs = 1 << 0;
	}

	priv->base.encoder = &priv->encoder;
	priv->connector.interlace_allowed = 1;
	priv->encoder.possible_crtcs = crtcs;

	ret = adihdmi_create(client, &priv->base);
	if (ret)
		return ret;

	drm_encoder_helper_add(&priv->encoder, &adihdmi_encoder_helper_funcs);
	ret = drm_encoder_init(drm, &priv->encoder, &adihdmi_encoder_funcs,
			DRM_MODE_ENCODER_TMDS);
	if (ret)
		goto err_encoder;

	drm_connector_helper_add(&priv->connector,
			&adihdmi_connector_helper_funcs);
	ret = drm_connector_init(drm, &priv->connector,
			&adihdmi_connector_funcs,
			DRM_MODE_CONNECTOR_HDMIA);
	if (ret)
		goto err_connector;

	ret = drm_connector_register(&priv->connector);
	if (ret)
		goto err_sysfs;

	priv->connector.encoder = &priv->encoder;
	drm_mode_connector_attach_encoder(&priv->connector, &priv->encoder);

	return 0;

err_sysfs:
	drm_connector_cleanup(&priv->connector);
err_connector:
	drm_encoder_cleanup(&priv->encoder);
err_encoder:
	adihdmi_destroy(&priv->base);
	return ret;


}

static void adihdmi_unbind(struct device *dev, struct device *master, void *data)
{
	struct adihdmi2 *priv = dev_get_drvdata(dev);

	drm_connector_cleanup(&priv->connector);
	drm_encoder_cleanup(&priv->encoder);
	adihdmi_destroy(&priv->base);
}

static const struct component_ops adihdmi_ops =
{
	.bind = adihdmi_bind,
	.unbind = adihdmi_unbind,
};

/* -----------------------------------------------------------------------------
 * Init operations
 */

static int adihdmi_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	return component_add(&i2c->dev, &adihdmi_ops);
}

static int adihdmi_remove(struct i2c_client *i2c)
{
	component_del(&i2c->dev, &adihdmi_ops);

	return 0;
}

static int adihdmi_encoder_init(struct i2c_client *i2c, struct drm_device *dev,
		struct drm_encoder_slave *encoder_slave)
{

	struct adihdmi *adihdmi;
	int ret;

	adihdmi = kzalloc(sizeof(*adihdmi), GFP_KERNEL);
	if (!adihdmi)
		return -ENOMEM;

	adihdmi->encoder = &encoder_slave->base;

	ret = adihdmi_create(i2c, adihdmi);
	if (ret) {
		kfree(adihdmi);
		return ret;
	}

	encoder_slave->slave_priv = adihdmi;
	encoder_slave->slave_funcs = &adihdmi_encoder_slave_funcs;

	return 0;
}

static const struct i2c_device_id adihdmi_i2c_ids[] = {
	{ "adv7511", 0 },
	{ "adv7511w", 0 },
	{ "adv7513", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, adihdmi_i2c_ids);

static const struct of_device_id adihdmi_of_ids[] = {
	{ .compatible = "adi,adv7511", },
	{ .compatible = "adi,adv7511w", },
	{ .compatible = "adi,adv7513", },
	{ }
};
MODULE_DEVICE_TABLE(of, adihdmi_of_ids);

static struct drm_i2c_encoder_driver adihdmi_driver = {
	.i2c_driver = {
		.driver = {
			.name = "adihdmi",
			.of_match_table = adihdmi_of_ids,
		},
		.id_table = adihdmi_i2c_ids,
		.probe = adihdmi_probe,
		.remove = adihdmi_remove,
	},

	.encoder_init = adihdmi_encoder_init,
};

static int __init adihdmi_init(void)
{
	return drm_i2c_encoder_register(THIS_MODULE, &adihdmi_driver);
}
module_init(adihdmi_init);

static void __exit adihdmi_exit(void)
{
	drm_i2c_encoder_unregister(&adihdmi_driver);
}
module_exit(adihdmi_exit);

MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_DESCRIPTION("ADIHDMI HDMI transmitter driver");
MODULE_LICENSE("GPL");
