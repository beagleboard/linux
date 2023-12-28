// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020 BayLibre, SAS
 * Author: Phong LE <ple@baylibre.com>
 * Copyright (C) 2018-2019, Artem Mygaiev
 * Copyright (C) 2017, Fresco Logic, Incorporated.
 *
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/bitfield.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/of_graph.h>
#include <linux/gpio/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_modes.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>

#include <sound/hdmi-codec.h>

#define IT66121_VENDOR_ID0_REG			0x00
#define IT66121_VENDOR_ID1_REG			0x01
#define IT66121_DEVICE_ID0_REG			0x02
#define IT66121_DEVICE_ID1_REG			0x03

#define IT66121_VENDOR_ID0			0x54
#define IT66121_VENDOR_ID1			0x49
#define IT66121_DEVICE_ID0			0x12
#define IT66121_DEVICE_ID1			0x06
#define IT66121_REVISION_MASK			GENMASK(7, 4)
#define IT66121_DEVICE_ID1_MASK			GENMASK(3, 0)

#define IT66121_MASTER_SEL_REG			0x10
#define IT66121_MASTER_SEL_HOST			BIT(0)

#define IT66121_AFE_DRV_REG			0x61
#define IT66121_AFE_DRV_RST			BIT(4)
#define IT66121_AFE_DRV_PWD			BIT(5)

#define IT66121_INPUT_MODE_REG			0x70
#define IT66121_INPUT_MODE_RGB			(0 << 6)
#define IT66121_INPUT_MODE_YUV422		BIT(6)
#define IT66121_INPUT_MODE_YUV444		(2 << 6)
#define IT66121_INPUT_MODE_CCIR656		BIT(4)
#define IT66121_INPUT_MODE_SYNCEMB		BIT(3)
#define IT66121_INPUT_MODE_DDR			BIT(2)

#define IT66121_INPUT_CSC_REG			0x72
#define IT66121_INPUT_CSC_ENDITHER		BIT(7)
#define IT66121_INPUT_CSC_ENUDFILTER		BIT(6)
#define IT66121_INPUT_CSC_DNFREE_GO		BIT(5)
#define IT66121_INPUT_CSC_RGB_TO_YUV		0x02
#define IT66121_INPUT_CSC_YUV_TO_RGB		0x03
#define IT66121_INPUT_CSC_NO_CONV		0x00

#define IT66121_AFE_XP_REG			0x62
#define IT66121_AFE_XP_GAINBIT			BIT(7)
#define IT66121_AFE_XP_PWDPLL			BIT(6)
#define IT66121_AFE_XP_ENI			BIT(5)
#define IT66121_AFE_XP_ENO			BIT(4)
#define IT66121_AFE_XP_RESETB			BIT(3)
#define IT66121_AFE_XP_PWDI			BIT(2)

#define IT66121_AFE_IP_REG			0x64
#define IT66121_AFE_IP_GAINBIT			BIT(7)
#define IT66121_AFE_IP_PWDPLL			BIT(6)
#define IT66121_AFE_IP_CKSEL_05			(0 << 4)
#define IT66121_AFE_IP_CKSEL_1			BIT(4)
#define IT66121_AFE_IP_CKSEL_2			(2 << 4)
#define IT66121_AFE_IP_CKSEL_2OR4		(3 << 4)
#define IT66121_AFE_IP_ER0			BIT(3)
#define IT66121_AFE_IP_RESETB			BIT(2)
#define IT66121_AFE_IP_ENC			BIT(1)
#define IT66121_AFE_IP_EC1			BIT(0)

#define IT66121_AFE_XP_EC1_REG			0x68
#define IT66121_AFE_XP_EC1_LOWCLK		BIT(4)

#define IT66121_SW_RST_REG			0x04
#define IT66121_SW_RST_REF			BIT(5)
#define IT66121_SW_RST_AREF			BIT(4)
#define IT66121_SW_RST_VID			BIT(3)
#define IT66121_SW_RST_AUD			BIT(2)
#define IT66121_SW_RST_HDCP			BIT(0)

#define IT66121_DDC_COMMAND_REG			0x15
#define IT66121_DDC_COMMAND_BURST_READ		0x0
#define IT66121_DDC_COMMAND_EDID_READ		0x3
#define IT66121_DDC_COMMAND_FIFO_CLR		0x9
#define IT66121_DDC_COMMAND_SCL_PULSE		0xA
#define IT66121_DDC_COMMAND_ABORT		0xF

#define IT66121_HDCP_REG			0x20
#define IT66121_HDCP_CPDESIRED			BIT(0)
#define IT66121_HDCP_EN1P1FEAT			BIT(1)

#define IT66121_INT_STATUS1_REG			0x06
#define IT66121_INT_STATUS1_AUD_OVF		BIT(7)
#define IT66121_INT_STATUS1_DDC_NOACK		BIT(5)
#define IT66121_INT_STATUS1_DDC_FIFOERR		BIT(4)
#define IT66121_INT_STATUS1_DDC_BUSHANG		BIT(2)
#define IT66121_INT_STATUS1_RX_SENS_STATUS	BIT(1)
#define IT66121_INT_STATUS1_HPD_STATUS		BIT(0)

#define IT66121_DDC_HEADER_REG			0x11
#define IT66121_DDC_HEADER_HDCP			0x74
#define IT66121_DDC_HEADER_EDID			0xA0

#define IT66121_DDC_OFFSET_REG			0x12
#define IT66121_DDC_BYTE_REG			0x13
#define IT66121_DDC_SEGMENT_REG			0x14
#define IT66121_DDC_RD_FIFO_REG			0x17

#define IT66121_CLK_BANK_REG			0x0F
#define IT66121_CLK_BANK_PWROFF_RCLK		BIT(6)
#define IT66121_CLK_BANK_PWROFF_ACLK		BIT(5)
#define IT66121_CLK_BANK_PWROFF_TXCLK		BIT(4)
#define IT66121_CLK_BANK_PWROFF_CRCLK		BIT(3)
#define IT66121_CLK_BANK_0			0
#define IT66121_CLK_BANK_1			1

#define IT66121_INT_REG				0x05
#define IT66121_INT_ACTIVE_HIGH			BIT(7)
#define IT66121_INT_OPEN_DRAIN			BIT(6)
#define IT66121_INT_TX_CLK_OFF			BIT(0)

#define IT66121_INT_MASK1_REG			0x09
#define IT66121_INT_MASK1_AUD_OVF		BIT(7)
#define IT66121_INT_MASK1_DDC_NOACK		BIT(5)
#define IT66121_INT_MASK1_DDC_FIFOERR		BIT(4)
#define IT66121_INT_MASK1_DDC_BUSHANG		BIT(2)
#define IT66121_INT_MASK1_RX_SENS		BIT(1)
#define IT66121_INT_MASK1_HPD			BIT(0)

#define IT66121_INT_CLR1_REG			0x0C
#define IT66121_INT_CLR1_PKTACP			BIT(7)
#define IT66121_INT_CLR1_PKTNULL		BIT(6)
#define IT66121_INT_CLR1_PKTGEN			BIT(5)
#define IT66121_INT_CLR1_KSVLISTCHK		BIT(4)
#define IT66121_INT_CLR1_AUTHDONE		BIT(3)
#define IT66121_INT_CLR1_AUTHFAIL		BIT(2)
#define IT66121_INT_CLR1_RX_SENS		BIT(1)
#define IT66121_INT_CLR1_HPD			BIT(0)

#define IT66121_AV_MUTE_REG			0xC1
#define IT66121_AV_MUTE_ON			BIT(0)
#define IT66121_AV_MUTE_BLUESCR			BIT(1)

#define IT66121_PKT_CTS_CTRL_REG		0xC5
#define IT66121_PKT_CTS_CTRL_SEL		BIT(1)

#define IT66121_PKT_GEN_CTRL_REG		0xC6
#define IT66121_PKT_GEN_CTRL_ON			BIT(0)
#define IT66121_PKT_GEN_CTRL_RPT		BIT(1)

#define IT66121_AVIINFO_DB1_REG			0x158
#define IT66121_AVIINFO_DB2_REG			0x159
#define IT66121_AVIINFO_DB3_REG			0x15A
#define IT66121_AVIINFO_DB4_REG			0x15B
#define IT66121_AVIINFO_DB5_REG			0x15C
#define IT66121_AVIINFO_CSUM_REG		0x15D
#define IT66121_AVIINFO_DB6_REG			0x15E
#define IT66121_AVIINFO_DB7_REG			0x15F
#define IT66121_AVIINFO_DB8_REG			0x160
#define IT66121_AVIINFO_DB9_REG			0x161
#define IT66121_AVIINFO_DB10_REG		0x162
#define IT66121_AVIINFO_DB11_REG		0x163
#define IT66121_AVIINFO_DB12_REG		0x164
#define IT66121_AVIINFO_DB13_REG		0x165

#define IT66121_AVI_INFO_PKT_REG		0xCD
#define IT66121_AVI_INFO_PKT_ON			BIT(0)
#define IT66121_AVI_INFO_PKT_RPT		BIT(1)

#define IT66121_HDMI_MODE_REG			0xC0
#define IT66121_HDMI_MODE_HDMI			BIT(0)

#define IT66121_SYS_STATUS_REG			0x0E
#define IT66121_SYS_STATUS_ACTIVE_IRQ		BIT(7)
#define IT66121_SYS_STATUS_HPDETECT		BIT(6)
#define IT66121_SYS_STATUS_SENDECTECT		BIT(5)
#define IT66121_SYS_STATUS_VID_STABLE		BIT(4)
#define IT66121_SYS_STATUS_AUD_CTS_CLR		BIT(1)
#define IT66121_SYS_STATUS_CLEAR_IRQ		BIT(0)

#define IT66121_DDC_STATUS_REG			0x16
#define IT66121_DDC_STATUS_TX_DONE		BIT(7)
#define IT66121_DDC_STATUS_ACTIVE		BIT(6)
#define IT66121_DDC_STATUS_NOACK		BIT(5)
#define IT66121_DDC_STATUS_WAIT_BUS		BIT(4)
#define IT66121_DDC_STATUS_ARBI_LOSE		BIT(3)
#define IT66121_DDC_STATUS_FIFO_FULL		BIT(2)
#define IT66121_DDC_STATUS_FIFO_EMPTY		BIT(1)
#define IT66121_DDC_STATUS_FIFO_VALID		BIT(0)

#define IT66121_EDID_SLEEP_US			20000
#define IT66121_EDID_TIMEOUT_US			200000
#define IT66121_EDID_FIFO_SIZE			32

#define IT66121_CLK_CTRL0_REG			0x58
#define IT66121_CLK_CTRL0_AUTO_OVER_SAMPLING	BIT(4)
#define IT66121_CLK_CTRL0_EXT_MCLK_MASK		GENMASK(3, 2)
#define IT66121_CLK_CTRL0_EXT_MCLK_128FS	(0 << 2)
#define IT66121_CLK_CTRL0_EXT_MCLK_256FS	BIT(2)
#define IT66121_CLK_CTRL0_EXT_MCLK_512FS	(2 << 2)
#define IT66121_CLK_CTRL0_EXT_MCLK_1024FS	(3 << 2)
#define IT66121_CLK_CTRL0_AUTO_IPCLK		BIT(0)
#define IT66121_CLK_STATUS1_REG			0x5E
#define IT66121_CLK_STATUS2_REG			0x5F

#define IT66121_AUD_CTRL0_REG			0xE0
#define IT66121_AUD_SWL				(3 << 6)
#define IT66121_AUD_16BIT			(0 << 6)
#define IT66121_AUD_18BIT			BIT(6)
#define IT66121_AUD_20BIT			(2 << 6)
#define IT66121_AUD_24BIT			(3 << 6)
#define IT66121_AUD_SPDIFTC			BIT(5)
#define IT66121_AUD_SPDIF			BIT(4)
#define IT66121_AUD_I2S				(0 << 4)
#define IT66121_AUD_EN_I2S3			BIT(3)
#define IT66121_AUD_EN_I2S2			BIT(2)
#define IT66121_AUD_EN_I2S1			BIT(1)
#define IT66121_AUD_EN_I2S0			BIT(0)
#define IT66121_AUD_CTRL0_AUD_SEL		BIT(4)

#define IT66121_AUD_CTRL1_REG			0xE1
#define IT66121_AUD_FIFOMAP_REG			0xE2
#define IT66121_AUD_CTRL3_REG			0xE3
#define IT66121_AUD_SRCVALID_FLAT_REG		0xE4
#define IT66121_AUD_FLAT_SRC0			BIT(4)
#define IT66121_AUD_FLAT_SRC1			BIT(5)
#define IT66121_AUD_FLAT_SRC2			BIT(6)
#define IT66121_AUD_FLAT_SRC3			BIT(7)
#define IT66121_AUD_HDAUDIO_REG			0xE5

#define IT66121_AUD_PKT_CTS0_REG		0x130
#define IT66121_AUD_PKT_CTS1_REG		0x131
#define IT66121_AUD_PKT_CTS2_REG		0x132
#define IT66121_AUD_PKT_N0_REG			0x133
#define IT66121_AUD_PKT_N1_REG			0x134
#define IT66121_AUD_PKT_N2_REG			0x135

#define IT66121_AUD_CHST_MODE_REG		0x191
#define IT66121_AUD_CHST_CAT_REG		0x192
#define IT66121_AUD_CHST_SRCNUM_REG		0x193
#define IT66121_AUD_CHST_CHTNUM_REG		0x194
#define IT66121_AUD_CHST_CA_FS_REG		0x198
#define IT66121_AUD_CHST_OFS_WL_REG		0x199

#define IT66121_AUD_PKT_CTS_CNT0_REG		0x1A0
#define IT66121_AUD_PKT_CTS_CNT1_REG		0x1A1
#define IT66121_AUD_PKT_CTS_CNT2_REG		0x1A2

#define IT66121_AUD_FS_22P05K			0x4
#define IT66121_AUD_FS_44P1K			0x0
#define IT66121_AUD_FS_88P2K			0x8
#define IT66121_AUD_FS_176P4K			0xC
#define IT66121_AUD_FS_24K			0x6
#define IT66121_AUD_FS_48K			0x2
#define IT66121_AUD_FS_96K			0xA
#define IT66121_AUD_FS_192K			0xE
#define IT66121_AUD_FS_768K			0x9
#define IT66121_AUD_FS_32K			0x3
#define IT66121_AUD_FS_OTHER			0x1

#define IT66121_AUD_SWL_21BIT			0xD
#define IT66121_AUD_SWL_24BIT			0xB
#define IT66121_AUD_SWL_23BIT			0x9
#define IT66121_AUD_SWL_22BIT			0x5
#define IT66121_AUD_SWL_20BIT			0x3
#define IT66121_AUD_SWL_17BIT			0xC
#define IT66121_AUD_SWL_19BIT			0x8
#define IT66121_AUD_SWL_18BIT			0x4
#define IT66121_AUD_SWL_16BIT			0x2
#define IT66121_AUD_SWL_NOT_INDICATED		0x0

#define IT66121_VENDOR_ID0			0x54
#define IT66121_VENDOR_ID1			0x49
#define IT66121_DEVICE_ID0			0x12
#define IT66121_DEVICE_ID1			0x06
#define IT66121_DEVICE_MASK			0x0F
#define IT66121_AFE_CLK_HIGH			80000 /* Khz */

struct it66121_ctx {
	struct regmap *regmap;
	struct drm_bridge bridge;
	struct drm_bridge *next_bridge;
	struct drm_connector *connector;
	struct device *dev;
	struct gpio_desc *gpio_reset;
	struct i2c_client *client;
	struct regulator_bulk_data supplies[3];
	u32 bus_width;
	struct mutex lock; /* Protects fields below and device registers */
	struct hdmi_avi_infoframe hdmi_avi_infoframe;
	struct {
		struct platform_device *pdev;
		u8 ch_enable;
		u8 fs;
		u8 swl;
		bool auto_cts;
	} audio;
};

static const struct regmap_range_cfg it66121_regmap_banks[] = {
	{
		.name = "it66121",
		.range_min = 0x00,
		.range_max = 0x1FF,
		.selector_reg = IT66121_CLK_BANK_REG,
		.selector_mask = 0x1,
		.selector_shift = 0,
		.window_start = 0x00,
		.window_len = 0x100,
	},
};

static const struct regmap_config it66121_regmap_config = {
	.val_bits = 8,
	.reg_bits = 8,
	.max_register = 0x1FF,
	.ranges = it66121_regmap_banks,
	.num_ranges = ARRAY_SIZE(it66121_regmap_banks),
};

static void it66121_hw_reset(struct it66121_ctx *ctx)
{
	gpiod_set_value(ctx->gpio_reset, 1);
	msleep(20);
	gpiod_set_value(ctx->gpio_reset, 0);
}

static inline int ite66121_power_on(struct it66121_ctx *ctx)
{
	return regulator_bulk_enable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
}

static inline int ite66121_power_off(struct it66121_ctx *ctx)
{
	return regulator_bulk_disable(ARRAY_SIZE(ctx->supplies), ctx->supplies);
}

static inline int it66121_preamble_ddc(struct it66121_ctx *ctx)
{
	return regmap_write(ctx->regmap, IT66121_MASTER_SEL_REG, IT66121_MASTER_SEL_HOST);
}

static inline int it66121_fire_afe(struct it66121_ctx *ctx)
{
	return regmap_write(ctx->regmap, IT66121_AFE_DRV_REG, 0);
}

/* TOFIX: Handle YCbCr Input & Output */
static int it66121_configure_input(struct it66121_ctx *ctx)
{
	int ret;
	u8 mode = IT66121_INPUT_MODE_RGB;

	if (ctx->bus_width == 12)
		mode |= IT66121_INPUT_MODE_DDR;

	ret = regmap_write(ctx->regmap, IT66121_INPUT_MODE_REG, mode);
	if (ret)
		return ret;

	return regmap_write(ctx->regmap, IT66121_INPUT_CSC_REG, IT66121_INPUT_CSC_NO_CONV);
}

/**
 * it66121_configure_afe() - Configure the analog front end
 * @ctx: it66121_ctx object
 * @mode: mode to configure
 *
 * RETURNS:
 * zero if success, a negative error code otherwise.
 */
static int it66121_configure_afe(struct it66121_ctx *ctx,
				 const struct drm_display_mode *mode)
{
	int ret;

	ret = regmap_write(ctx->regmap, IT66121_AFE_DRV_REG,
			   IT66121_AFE_DRV_RST);
	if (ret)
		return ret;

	if (mode->clock > IT66121_AFE_CLK_HIGH) {
		ret = regmap_write_bits(ctx->regmap, IT66121_AFE_XP_REG,
					IT66121_AFE_XP_GAINBIT |
					IT66121_AFE_XP_ENO,
					IT66121_AFE_XP_GAINBIT);
		if (ret)
			return ret;

		ret = regmap_write_bits(ctx->regmap, IT66121_AFE_IP_REG,
					IT66121_AFE_IP_GAINBIT |
					IT66121_AFE_IP_ER0 |
					IT66121_AFE_IP_EC1,
					IT66121_AFE_IP_GAINBIT);
		if (ret)
			return ret;

		ret = regmap_write_bits(ctx->regmap, IT66121_AFE_XP_EC1_REG,
					IT66121_AFE_XP_EC1_LOWCLK, 0x80);
		if (ret)
			return ret;
	} else {
		ret = regmap_write_bits(ctx->regmap, IT66121_AFE_XP_REG,
					IT66121_AFE_XP_GAINBIT |
					IT66121_AFE_XP_ENO,
					IT66121_AFE_XP_ENO);
		if (ret)
			return ret;

		ret = regmap_write_bits(ctx->regmap, IT66121_AFE_IP_REG,
					IT66121_AFE_IP_GAINBIT |
					IT66121_AFE_IP_ER0 |
					IT66121_AFE_IP_EC1, IT66121_AFE_IP_ER0 |
					IT66121_AFE_IP_EC1);
		if (ret)
			return ret;

		ret = regmap_write_bits(ctx->regmap, IT66121_AFE_XP_EC1_REG,
					IT66121_AFE_XP_EC1_LOWCLK,
					IT66121_AFE_XP_EC1_LOWCLK);
		if (ret)
			return ret;
	}

	/* Clear reset flags */
	ret = regmap_write_bits(ctx->regmap, IT66121_SW_RST_REG,
				IT66121_SW_RST_REF | IT66121_SW_RST_VID, 0);
	if (ret)
		return ret;

	return it66121_fire_afe(ctx);
}

static inline int it66121_wait_ddc_ready(struct it66121_ctx *ctx)
{
	int ret, val;
	u32 busy = IT66121_DDC_STATUS_NOACK | IT66121_DDC_STATUS_WAIT_BUS |
		   IT66121_DDC_STATUS_ARBI_LOSE;

	ret = regmap_read_poll_timeout(ctx->regmap, IT66121_DDC_STATUS_REG, val, true,
				       IT66121_EDID_SLEEP_US, IT66121_EDID_TIMEOUT_US);
	if (ret)
		return ret;

	if (val & busy)
		return -EAGAIN;

	return 0;
}

static int it66121_clear_ddc_fifo(struct it66121_ctx *ctx)
{
	int ret;

	ret = it66121_preamble_ddc(ctx);
	if (ret)
		return ret;

	return regmap_write(ctx->regmap, IT66121_DDC_COMMAND_REG,
			    IT66121_DDC_COMMAND_FIFO_CLR);
}

static int it66121_abort_ddc_ops(struct it66121_ctx *ctx)
{
	int ret;
	unsigned int swreset, cpdesire;

	ret = regmap_read(ctx->regmap, IT66121_SW_RST_REG, &swreset);
	if (ret)
		return ret;

	ret = regmap_read(ctx->regmap, IT66121_HDCP_REG, &cpdesire);
	if (ret)
		return ret;

	ret = regmap_write(ctx->regmap, IT66121_HDCP_REG,
			   cpdesire & (~IT66121_HDCP_CPDESIRED & 0xFF));
	if (ret)
		return ret;

	ret = regmap_write(ctx->regmap, IT66121_SW_RST_REG,
			   (swreset | IT66121_SW_RST_HDCP));
	if (ret)
		return ret;

	ret = it66121_preamble_ddc(ctx);
	if (ret)
		return ret;

	ret = regmap_write(ctx->regmap, IT66121_DDC_COMMAND_REG,
			   IT66121_DDC_COMMAND_ABORT);
	if (ret)
		return ret;

	return it66121_wait_ddc_ready(ctx);
}

static int it66121_get_edid_block(void *context, u8 *buf,
				  unsigned int block, size_t len)
{
	struct it66121_ctx *ctx = context;
	unsigned int val;
	int remain = len;
	int offset = 0;
	int ret, cnt;

	offset = (block % 2) * len;
	block = block / 2;

	ret = regmap_read(ctx->regmap, IT66121_INT_STATUS1_REG, &val);
	if (ret)
		return ret;

	if (val & IT66121_INT_STATUS1_DDC_BUSHANG) {
		ret = it66121_abort_ddc_ops(ctx);
		if (ret)
			return ret;
	}

	ret = it66121_clear_ddc_fifo(ctx);
	if (ret)
		return ret;

	while (remain > 0) {
		cnt = (remain > IT66121_EDID_FIFO_SIZE) ?
				IT66121_EDID_FIFO_SIZE : remain;
		ret = it66121_preamble_ddc(ctx);
		if (ret)
			return ret;

		ret = regmap_write(ctx->regmap, IT66121_DDC_COMMAND_REG,
				   IT66121_DDC_COMMAND_FIFO_CLR);
		if (ret)
			return ret;

		ret = it66121_wait_ddc_ready(ctx);
		if (ret)
			return ret;

		ret = regmap_read(ctx->regmap, IT66121_INT_STATUS1_REG, &val);
		if (ret)
			return ret;

		if (val & IT66121_INT_STATUS1_DDC_BUSHANG) {
			ret = it66121_abort_ddc_ops(ctx);
			if (ret)
				return ret;
		}

		ret = it66121_preamble_ddc(ctx);
		if (ret)
			return ret;

		ret = regmap_write(ctx->regmap, IT66121_DDC_HEADER_REG,
				   IT66121_DDC_HEADER_EDID);
		if (ret)
			return ret;

		ret = regmap_write(ctx->regmap, IT66121_DDC_OFFSET_REG, offset);
		if (ret)
			return ret;

		ret = regmap_write(ctx->regmap, IT66121_DDC_BYTE_REG, cnt);
		if (ret)
			return ret;

		ret = regmap_write(ctx->regmap, IT66121_DDC_SEGMENT_REG, block);
		if (ret)
			return ret;

		ret = regmap_write(ctx->regmap, IT66121_DDC_COMMAND_REG,
				   IT66121_DDC_COMMAND_EDID_READ);
		if (ret)
			return ret;

		offset += cnt;
		remain -= cnt;

		/* Per programming manual, sleep here before emptying the FIFO */
		msleep(20);

		ret = it66121_wait_ddc_ready(ctx);
		if (ret)
			return ret;

		do {
			ret = regmap_read(ctx->regmap, IT66121_DDC_RD_FIFO_REG, &val);
			if (ret)
				return ret;
			*(buf++) = val;
			cnt--;
		} while (cnt > 0);
	}

	return 0;
}

static bool it66121_is_hpd_detect(struct it66121_ctx *ctx)
{
	int val;

	if (regmap_read(ctx->regmap, IT66121_SYS_STATUS_REG, &val))
		return false;

	return val & IT66121_SYS_STATUS_HPDETECT;
}

static int it66121_bridge_attach(struct drm_bridge *bridge,
				 enum drm_bridge_attach_flags flags)
{
	struct it66121_ctx *ctx = container_of(bridge, struct it66121_ctx, bridge);
	int ret;

	if (!(flags & DRM_BRIDGE_ATTACH_NO_CONNECTOR))
		return -EINVAL;

	ret = drm_bridge_attach(bridge->encoder, ctx->next_bridge, bridge, flags);
	if (ret)
		return ret;

	ret = regmap_write_bits(ctx->regmap, IT66121_CLK_BANK_REG,
				IT66121_CLK_BANK_PWROFF_RCLK, 0);
	if (ret)
		return ret;

	ret = regmap_write_bits(ctx->regmap, IT66121_INT_REG,
				IT66121_INT_TX_CLK_OFF, 0);
	if (ret)
		return ret;

	ret = regmap_write_bits(ctx->regmap, IT66121_AFE_DRV_REG,
				IT66121_AFE_DRV_PWD, 0);
	if (ret)
		return ret;

	ret = regmap_write_bits(ctx->regmap, IT66121_AFE_XP_REG,
				IT66121_AFE_XP_PWDI | IT66121_AFE_XP_PWDPLL, 0);
	if (ret)
		return ret;

	ret = regmap_write_bits(ctx->regmap, IT66121_AFE_IP_REG,
				IT66121_AFE_IP_PWDPLL, 0);
	if (ret)
		return ret;

	ret = regmap_write_bits(ctx->regmap, IT66121_AFE_DRV_REG,
				IT66121_AFE_DRV_RST, 0);
	if (ret)
		return ret;

	ret = regmap_write_bits(ctx->regmap, IT66121_AFE_XP_REG,
				IT66121_AFE_XP_RESETB, IT66121_AFE_XP_RESETB);
	if (ret)
		return ret;

	ret = regmap_write_bits(ctx->regmap, IT66121_AFE_IP_REG,
				IT66121_AFE_IP_RESETB, IT66121_AFE_IP_RESETB);
	if (ret)
		return ret;

	ret = regmap_write_bits(ctx->regmap, IT66121_SW_RST_REG,
				IT66121_SW_RST_REF,
				IT66121_SW_RST_REF);
	if (ret)
		return ret;

	/* Per programming manual, sleep here for bridge to settle */
	msleep(50);

	/* Start interrupts */
	return regmap_write_bits(ctx->regmap, IT66121_INT_MASK1_REG,
				 IT66121_INT_MASK1_DDC_NOACK |
				 IT66121_INT_MASK1_DDC_FIFOERR |
				 IT66121_INT_MASK1_DDC_BUSHANG, 0);
}

static int it66121_set_mute(struct it66121_ctx *ctx, bool mute)
{
	int ret;
	unsigned int val = 0;

	if (mute)
		val = IT66121_AV_MUTE_ON;

	ret = regmap_write_bits(ctx->regmap, IT66121_AV_MUTE_REG, IT66121_AV_MUTE_ON, val);
	if (ret)
		return ret;

	return regmap_write(ctx->regmap, IT66121_PKT_GEN_CTRL_REG,
			    IT66121_PKT_GEN_CTRL_ON | IT66121_PKT_GEN_CTRL_RPT);
}

#define MAX_OUTPUT_SEL_FORMATS	1

static u32 *it66121_bridge_atomic_get_output_bus_fmts(struct drm_bridge *bridge,
						      struct drm_bridge_state *bridge_state,
						      struct drm_crtc_state *crtc_state,
						      struct drm_connector_state *conn_state,
						      unsigned int *num_output_fmts)
{
	u32 *output_fmts;

	output_fmts = kcalloc(MAX_OUTPUT_SEL_FORMATS, sizeof(*output_fmts),
			      GFP_KERNEL);
	if (!output_fmts)
		return NULL;

	/* TOFIX handle more than MEDIA_BUS_FMT_RGB888_1X24 as output format */
	output_fmts[0] =  MEDIA_BUS_FMT_RGB888_1X24;
	*num_output_fmts = 1;

	return output_fmts;
}

#define MAX_INPUT_SEL_FORMATS	1

static u32 *it66121_bridge_atomic_get_input_bus_fmts(struct drm_bridge *bridge,
						     struct drm_bridge_state *bridge_state,
						     struct drm_crtc_state *crtc_state,
						     struct drm_connector_state *conn_state,
						     u32 output_fmt,
						     unsigned int *num_input_fmts)
{
	struct it66121_ctx *ctx = container_of(bridge, struct it66121_ctx, bridge);
	u32 *input_fmts;

	*num_input_fmts = 0;

	input_fmts = kcalloc(MAX_INPUT_SEL_FORMATS, sizeof(*input_fmts),
			     GFP_KERNEL);
	if (!input_fmts)
		return NULL;

	if (ctx->bus_width == 12)
		/* IT66121FN Datasheet specifies Little-Endian ordering */
		input_fmts[0] = MEDIA_BUS_FMT_RGB888_2X12_LE;
	else
		/* TOFIX support more input bus formats in 24bit width */
		input_fmts[0] = MEDIA_BUS_FMT_RGB888_1X24;
	*num_input_fmts = 1;

	return input_fmts;
}

static void it66121_bridge_enable(struct drm_bridge *bridge,
				  struct drm_bridge_state *bridge_state)
{
	struct it66121_ctx *ctx = container_of(bridge, struct it66121_ctx, bridge);
	struct drm_atomic_state *state = bridge_state->base.state;

	ctx->connector = drm_atomic_get_new_connector_for_encoder(state, bridge->encoder);

	it66121_set_mute(ctx, false);
}

static void it66121_bridge_disable(struct drm_bridge *bridge,
				   struct drm_bridge_state *bridge_state)
{
	struct it66121_ctx *ctx = container_of(bridge, struct it66121_ctx, bridge);

	it66121_set_mute(ctx, true);

	ctx->connector = NULL;
}

static
void it66121_bridge_mode_set(struct drm_bridge *bridge,
			     const struct drm_display_mode *mode,
			     const struct drm_display_mode *adjusted_mode)
{
	int ret, i;
	u8 buf[HDMI_INFOFRAME_SIZE(AVI)];
	struct it66121_ctx *ctx = container_of(bridge, struct it66121_ctx, bridge);
	const u16 aviinfo_reg[HDMI_AVI_INFOFRAME_SIZE] = {
		IT66121_AVIINFO_DB1_REG,
		IT66121_AVIINFO_DB2_REG,
		IT66121_AVIINFO_DB3_REG,
		IT66121_AVIINFO_DB4_REG,
		IT66121_AVIINFO_DB5_REG,
		IT66121_AVIINFO_DB6_REG,
		IT66121_AVIINFO_DB7_REG,
		IT66121_AVIINFO_DB8_REG,
		IT66121_AVIINFO_DB9_REG,
		IT66121_AVIINFO_DB10_REG,
		IT66121_AVIINFO_DB11_REG,
		IT66121_AVIINFO_DB12_REG,
		IT66121_AVIINFO_DB13_REG
	};

	mutex_lock(&ctx->lock);

	hdmi_avi_infoframe_init(&ctx->hdmi_avi_infoframe);

	ret = drm_hdmi_avi_infoframe_from_display_mode(&ctx->hdmi_avi_infoframe, ctx->connector,
						       adjusted_mode);
	if (ret) {
		DRM_ERROR("Failed to setup AVI infoframe: %d\n", ret);
		goto unlock;
	}

	ret = hdmi_avi_infoframe_pack(&ctx->hdmi_avi_infoframe, buf, sizeof(buf));
	if (ret < 0) {
		DRM_ERROR("Failed to pack infoframe: %d\n", ret);
		goto unlock;
	}

	/* Write new AVI infoframe packet */
	for (i = 0; i < HDMI_AVI_INFOFRAME_SIZE; i++) {
		if (regmap_write(ctx->regmap, aviinfo_reg[i], buf[i + HDMI_INFOFRAME_HEADER_SIZE]))
			goto unlock;
	}
	if (regmap_write(ctx->regmap, IT66121_AVIINFO_CSUM_REG, buf[3]))
		goto unlock;

	/* Enable AVI infoframe */
	if (regmap_write(ctx->regmap, IT66121_AVI_INFO_PKT_REG,
			 IT66121_AVI_INFO_PKT_ON | IT66121_AVI_INFO_PKT_RPT))
		goto unlock;

	/* Set TX mode to HDMI */
	if (regmap_write(ctx->regmap, IT66121_HDMI_MODE_REG, IT66121_HDMI_MODE_HDMI))
		goto unlock;

	if (regmap_write_bits(ctx->regmap, IT66121_CLK_BANK_REG,
			      IT66121_CLK_BANK_PWROFF_TXCLK, IT66121_CLK_BANK_PWROFF_TXCLK))
		goto unlock;

	if (it66121_configure_input(ctx))
		goto unlock;

	if (it66121_configure_afe(ctx, adjusted_mode))
		goto unlock;

	regmap_write_bits(ctx->regmap, IT66121_CLK_BANK_REG, IT66121_CLK_BANK_PWROFF_TXCLK, 0);

unlock:
	mutex_unlock(&ctx->lock);
}

static enum drm_mode_status it66121_bridge_mode_valid(struct drm_bridge *bridge,
						      const struct drm_display_info *info,
						      const struct drm_display_mode *mode)
{
	struct it66121_ctx *ctx = container_of(bridge, struct it66121_ctx, bridge);
	unsigned long max_clock;

	max_clock = (ctx->bus_width == 12) ? 74250 : 148500;

	if (mode->clock > max_clock)
		return MODE_CLOCK_HIGH;

	if (mode->clock < 25000)
		return MODE_CLOCK_LOW;

	return MODE_OK;
}

static enum drm_connector_status it66121_bridge_detect(struct drm_bridge *bridge)
{
	struct it66121_ctx *ctx = container_of(bridge, struct it66121_ctx, bridge);

	return it66121_is_hpd_detect(ctx) ? connector_status_connected
					  : connector_status_disconnected;
}

static void it66121_bridge_hpd_enable(struct drm_bridge *bridge)
{
	struct it66121_ctx *ctx = container_of(bridge, struct it66121_ctx, bridge);
	int ret;

	ret = regmap_write_bits(ctx->regmap, IT66121_INT_MASK1_REG, IT66121_INT_MASK1_HPD, 0);
	if (ret)
		dev_err(ctx->dev, "failed to enable HPD IRQ\n");
}

static void it66121_bridge_hpd_disable(struct drm_bridge *bridge)
{
	struct it66121_ctx *ctx = container_of(bridge, struct it66121_ctx, bridge);
	int ret;

	ret = regmap_write_bits(ctx->regmap, IT66121_INT_MASK1_REG,
				IT66121_INT_MASK1_HPD, IT66121_INT_MASK1_HPD);
	if (ret)
		dev_err(ctx->dev, "failed to disable HPD IRQ\n");
}

static struct edid *it66121_bridge_get_edid(struct drm_bridge *bridge,
					    struct drm_connector *connector)
{
	struct it66121_ctx *ctx = container_of(bridge, struct it66121_ctx, bridge);
	struct edid *edid;

	mutex_lock(&ctx->lock);
	edid = drm_do_get_edid(connector, it66121_get_edid_block, ctx);
	mutex_unlock(&ctx->lock);

	return edid;
}

static const struct drm_bridge_funcs it66121_bridge_funcs = {
	.atomic_duplicate_state = drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_bridge_destroy_state,
	.atomic_reset = drm_atomic_helper_bridge_reset,
	.attach = it66121_bridge_attach,
	.atomic_get_output_bus_fmts = it66121_bridge_atomic_get_output_bus_fmts,
	.atomic_get_input_bus_fmts = it66121_bridge_atomic_get_input_bus_fmts,
	.atomic_enable = it66121_bridge_enable,
	.atomic_disable = it66121_bridge_disable,
	.mode_set = it66121_bridge_mode_set,
	.mode_valid = it66121_bridge_mode_valid,
	.detect = it66121_bridge_detect,
	.get_edid = it66121_bridge_get_edid,
	.hpd_enable = it66121_bridge_hpd_enable,
	.hpd_disable = it66121_bridge_hpd_disable,
};

static irqreturn_t it66121_irq_threaded_handler(int irq, void *dev_id)
{
	int ret;
	unsigned int val;
	struct it66121_ctx *ctx = dev_id;
	struct device *dev = ctx->dev;
	enum drm_connector_status status;
	bool event = false;

	mutex_lock(&ctx->lock);

	ret = regmap_read(ctx->regmap, IT66121_SYS_STATUS_REG, &val);
	if (ret)
		goto unlock;

	if (!(val & IT66121_SYS_STATUS_ACTIVE_IRQ))
		goto unlock;

	ret = regmap_read(ctx->regmap, IT66121_INT_STATUS1_REG, &val);
	if (ret) {
		dev_err(dev, "Cannot read STATUS1_REG %d\n", ret);
	} else {
		if (val & IT66121_INT_STATUS1_DDC_FIFOERR)
			it66121_clear_ddc_fifo(ctx);
		if (val & (IT66121_INT_STATUS1_DDC_BUSHANG |
			   IT66121_INT_STATUS1_DDC_NOACK))
			it66121_abort_ddc_ops(ctx);
		if (val & IT66121_INT_STATUS1_HPD_STATUS) {
			regmap_write_bits(ctx->regmap, IT66121_INT_CLR1_REG,
					  IT66121_INT_CLR1_HPD, IT66121_INT_CLR1_HPD);

			status = it66121_is_hpd_detect(ctx) ? connector_status_connected
							    : connector_status_disconnected;

			event = true;
		}
	}

	regmap_write_bits(ctx->regmap, IT66121_SYS_STATUS_REG,
			  IT66121_SYS_STATUS_CLEAR_IRQ,
			  IT66121_SYS_STATUS_CLEAR_IRQ);

unlock:
	mutex_unlock(&ctx->lock);

	if (event)
		drm_bridge_hpd_notify(&ctx->bridge, status);

	return IRQ_HANDLED;
}

static int it661221_set_chstat(struct it66121_ctx *ctx, u8 iec60958_chstat[])
{
	int ret;

	ret = regmap_write(ctx->regmap, IT66121_AUD_CHST_MODE_REG, iec60958_chstat[0] & 0x7C);
	if (ret)
		return ret;

	ret = regmap_write(ctx->regmap, IT66121_AUD_CHST_CAT_REG, iec60958_chstat[1]);
	if (ret)
		return ret;

	ret = regmap_write(ctx->regmap, IT66121_AUD_CHST_SRCNUM_REG, iec60958_chstat[2] & 0x0F);
	if (ret)
		return ret;

	ret = regmap_write(ctx->regmap, IT66121_AUD_CHST_CHTNUM_REG,
			   (iec60958_chstat[2] >> 4) & 0x0F);
	if (ret)
		return ret;

	ret = regmap_write(ctx->regmap, IT66121_AUD_CHST_CA_FS_REG, iec60958_chstat[3]);
	if (ret)
		return ret;

	return regmap_write(ctx->regmap, IT66121_AUD_CHST_OFS_WL_REG, iec60958_chstat[4]);
}

static int it661221_set_lpcm_audio(struct it66121_ctx *ctx, u8 audio_src_num, u8 audio_swl)
{
	int ret;
	unsigned int audio_enable = 0;
	unsigned int audio_format = 0;

	switch (audio_swl) {
	case 16:
		audio_enable |= IT66121_AUD_16BIT;
		break;
	case 18:
		audio_enable |= IT66121_AUD_18BIT;
		break;
	case 20:
		audio_enable |= IT66121_AUD_20BIT;
		break;
	case 24:
	default:
		audio_enable |= IT66121_AUD_24BIT;
		break;
	}

	audio_format |= 0x40;
	switch (audio_src_num) {
	case 4:
		audio_enable |= IT66121_AUD_EN_I2S3 | IT66121_AUD_EN_I2S2 |
				IT66121_AUD_EN_I2S1 | IT66121_AUD_EN_I2S0;
		break;
	case 3:
		audio_enable |= IT66121_AUD_EN_I2S2 | IT66121_AUD_EN_I2S1 |
				IT66121_AUD_EN_I2S0;
		break;
	case 2:
		audio_enable |= IT66121_AUD_EN_I2S1 | IT66121_AUD_EN_I2S0;
		break;
	case 1:
	default:
		audio_format &= ~0x40;
		audio_enable |= IT66121_AUD_EN_I2S0;
		break;
	}

	audio_format |= 0x01;
	ctx->audio.ch_enable = audio_enable;

	ret = regmap_write(ctx->regmap, IT66121_AUD_CTRL0_REG, audio_enable & 0xF0);
	if (ret)
		return ret;

	ret = regmap_write(ctx->regmap, IT66121_AUD_CTRL1_REG, audio_format);
	if (ret)
		return ret;

	ret = regmap_write(ctx->regmap, IT66121_AUD_FIFOMAP_REG, 0xE4);
	if (ret)
		return ret;

	ret = regmap_write(ctx->regmap, IT66121_AUD_CTRL3_REG, 0x00);
	if (ret)
		return ret;

	ret = regmap_write(ctx->regmap, IT66121_AUD_SRCVALID_FLAT_REG, 0x00);
	if (ret)
		return ret;

	return regmap_write(ctx->regmap, IT66121_AUD_HDAUDIO_REG, 0x00);
}

static int it661221_set_ncts(struct it66121_ctx *ctx, u8 fs)
{
	int ret;
	unsigned int n;

	switch (fs) {
	case IT66121_AUD_FS_32K:
		n = 4096;
		break;
	case IT66121_AUD_FS_44P1K:
		n = 6272;
		break;
	case IT66121_AUD_FS_48K:
		n = 6144;
		break;
	case IT66121_AUD_FS_88P2K:
		n = 12544;
		break;
	case IT66121_AUD_FS_96K:
		n = 12288;
		break;
	case IT66121_AUD_FS_176P4K:
		n = 25088;
		break;
	case IT66121_AUD_FS_192K:
		n = 24576;
		break;
	case IT66121_AUD_FS_768K:
		n = 24576;
		break;
	default:
		n = 6144;
		break;
	}

	ret = regmap_write(ctx->regmap, IT66121_AUD_PKT_N0_REG, (u8)((n) & 0xFF));
	if (ret)
		return ret;

	ret = regmap_write(ctx->regmap, IT66121_AUD_PKT_N1_REG, (u8)((n >> 8) & 0xFF));
	if (ret)
		return ret;

	ret = regmap_write(ctx->regmap, IT66121_AUD_PKT_N2_REG, (u8)((n >> 16) & 0xF));
	if (ret)
		return ret;

	if (ctx->audio.auto_cts) {
		u8 loop_cnt = 255;
		u8 cts_stable_cnt = 0;
		unsigned int sum_cts = 0;
		unsigned int cts = 0;
		unsigned int last_cts = 0;
		unsigned int diff;
		unsigned int val;

		while (loop_cnt--) {
			msleep(30);
			regmap_read(ctx->regmap, IT66121_AUD_PKT_CTS_CNT2_REG, &val);
			cts = val << 12;
			regmap_read(ctx->regmap, IT66121_AUD_PKT_CTS_CNT1_REG, &val);
			cts |= val << 4;
			regmap_read(ctx->regmap, IT66121_AUD_PKT_CTS_CNT0_REG, &val);
			cts |= val >> 4;
			if (cts == 0) {
				continue;
			} else {
				if (last_cts > cts)
					diff = last_cts - cts;
				else
					diff = cts - last_cts;
				last_cts = cts;
				if (diff < 5) {
					cts_stable_cnt++;
					sum_cts += cts;
				} else {
					cts_stable_cnt = 0;
					sum_cts = 0;
					continue;
				}

				if (cts_stable_cnt >= 32) {
					last_cts = (sum_cts >> 5);
					break;
				}
			}
		}

		regmap_write(ctx->regmap, IT66121_AUD_PKT_CTS0_REG, (u8)((last_cts) & 0xFF));
		regmap_write(ctx->regmap, IT66121_AUD_PKT_CTS1_REG, (u8)((last_cts >> 8) & 0xFF));
		regmap_write(ctx->regmap, IT66121_AUD_PKT_CTS2_REG, (u8)((last_cts >> 16) & 0x0F));
	}

	ret = regmap_write(ctx->regmap, 0xF8, 0xC3);
	if (ret)
		return ret;

	ret = regmap_write(ctx->regmap, 0xF8, 0xA5);
	if (ret)
		return ret;

	if (ctx->audio.auto_cts) {
		ret = regmap_write_bits(ctx->regmap, IT66121_PKT_CTS_CTRL_REG,
					IT66121_PKT_CTS_CTRL_SEL,
					1);
	} else {
		ret = regmap_write_bits(ctx->regmap, IT66121_PKT_CTS_CTRL_REG,
					IT66121_PKT_CTS_CTRL_SEL,
					0);
	}

	if (ret)
		return ret;

	return regmap_write(ctx->regmap, 0xF8, 0xFF);
}

static int it661221_audio_output_enable(struct it66121_ctx *ctx, bool enable)
{
	int ret;

	if (enable) {
		ret = regmap_write_bits(ctx->regmap, IT66121_SW_RST_REG,
					IT66121_SW_RST_AUD | IT66121_SW_RST_AREF,
					0);
		if (ret)
			return ret;

		ret = regmap_write_bits(ctx->regmap, IT66121_AUD_CTRL0_REG,
					IT66121_AUD_EN_I2S3 | IT66121_AUD_EN_I2S2 |
					IT66121_AUD_EN_I2S1 | IT66121_AUD_EN_I2S0,
					ctx->audio.ch_enable);
	} else {
		ret = regmap_write_bits(ctx->regmap, IT66121_AUD_CTRL0_REG,
					IT66121_AUD_EN_I2S3 | IT66121_AUD_EN_I2S2 |
					IT66121_AUD_EN_I2S1 | IT66121_AUD_EN_I2S0,
					ctx->audio.ch_enable & 0xF0);
		if (ret)
			return ret;

		ret = regmap_write_bits(ctx->regmap, IT66121_SW_RST_REG,
					IT66121_SW_RST_AUD | IT66121_SW_RST_AREF,
					IT66121_SW_RST_AUD | IT66121_SW_RST_AREF);
	}

	return ret;
}

static int it661221_audio_ch_enable(struct it66121_ctx *ctx, bool enable)
{
	int ret;

	if (enable) {
		ret = regmap_write(ctx->regmap, IT66121_AUD_SRCVALID_FLAT_REG, 0);
		if (ret)
			return ret;

		ret = regmap_write(ctx->regmap, IT66121_AUD_CTRL0_REG, ctx->audio.ch_enable);
	} else {
		ret = regmap_write(ctx->regmap, IT66121_AUD_CTRL0_REG, ctx->audio.ch_enable & 0xF0);
	}

	return ret;
}

static int it66121_audio_hw_params(struct device *dev, void *data,
				   struct hdmi_codec_daifmt *daifmt,
				   struct hdmi_codec_params *params)
{
	u8 fs;
	u8 swl;
	int ret;
	struct it66121_ctx *ctx = dev_get_drvdata(dev);
	static u8 iec60958_chstat[5];
	unsigned int channels = params->channels;
	unsigned int sample_rate = params->sample_rate;
	unsigned int sample_width = params->sample_width;

	mutex_lock(&ctx->lock);
	dev_dbg(dev, "%s: %u, %u, %u, %u\n", __func__,
		daifmt->fmt, sample_rate, sample_width, channels);

	switch (daifmt->fmt) {
	case HDMI_I2S:
		dev_dbg(dev, "Using HDMI I2S\n");
		break;
	default:
		dev_err(dev, "Invalid or unsupported DAI format %d\n", daifmt->fmt);
		ret = -EINVAL;
		goto out;
	}

	// Set audio clock recovery (N/CTS)
	ret = regmap_write(ctx->regmap, IT66121_CLK_CTRL0_REG,
			   IT66121_CLK_CTRL0_AUTO_OVER_SAMPLING |
			   IT66121_CLK_CTRL0_EXT_MCLK_256FS |
			   IT66121_CLK_CTRL0_AUTO_IPCLK);
	if (ret)
		goto out;

	ret = regmap_write_bits(ctx->regmap, IT66121_AUD_CTRL0_REG,
				IT66121_AUD_CTRL0_AUD_SEL, 0); // remove spdif selection
	if (ret)
		goto out;

	switch (sample_rate) {
	case 44100L:
		fs = IT66121_AUD_FS_44P1K;
		break;
	case 88200L:
		fs = IT66121_AUD_FS_88P2K;
		break;
	case 176400L:
		fs = IT66121_AUD_FS_176P4K;
		break;
	case 32000L:
		fs = IT66121_AUD_FS_32K;
		break;
	case 48000L:
		fs = IT66121_AUD_FS_48K;
		break;
	case 96000L:
		fs = IT66121_AUD_FS_96K;
		break;
	case 192000L:
		fs = IT66121_AUD_FS_192K;
		break;
	case 768000L:
		fs = IT66121_AUD_FS_768K;
		break;
	default:
		fs = IT66121_AUD_FS_48K;
		break;
	}

	ctx->audio.fs = fs;
	ret = it661221_set_ncts(ctx, fs);
	if (ret) {
		dev_err(dev, "Failed to set N/CTS: %d\n", ret);
		goto out;
	}

	// Set audio format register (except audio channel enable)
	ret = it661221_set_lpcm_audio(ctx, (channels + 1) / 2, sample_width);
	if (ret) {
		dev_err(dev, "Failed to set LPCM audio: %d\n", ret);
		goto out;
	}

	// Set audio channel status
	iec60958_chstat[0] = 0;
	if ((channels + 1) / 2 == 1)
		iec60958_chstat[0] |= 0x1;
	iec60958_chstat[0] &= ~(1 << 1);
	iec60958_chstat[1] = 0;
	iec60958_chstat[2] = (channels + 1) / 2;
	iec60958_chstat[2] |= (channels << 4) & 0xF0;
	iec60958_chstat[3] = fs;

	switch (sample_width) {
	case 21L:
		swl = IT66121_AUD_SWL_21BIT;
		break;
	case 24L:
		swl = IT66121_AUD_SWL_24BIT;
		break;
	case 23L:
		swl = IT66121_AUD_SWL_23BIT;
		break;
	case 22L:
		swl = IT66121_AUD_SWL_22BIT;
		break;
	case 20L:
		swl = IT66121_AUD_SWL_20BIT;
		break;
	case 17L:
		swl = IT66121_AUD_SWL_17BIT;
		break;
	case 19L:
		swl = IT66121_AUD_SWL_19BIT;
		break;
	case 18L:
		swl = IT66121_AUD_SWL_18BIT;
		break;
	case 16L:
		swl = IT66121_AUD_SWL_16BIT;
		break;
	default:
		swl = IT66121_AUD_SWL_NOT_INDICATED;
		break;
	}

	iec60958_chstat[4] = (((~fs) << 4) & 0xF0) | swl;
	ret = it661221_set_chstat(ctx, iec60958_chstat);
	if (ret) {
		dev_err(dev, "Failed to set channel status: %d\n", ret);
		goto out;
	}

	// Enable audio channel enable while input clock stable (if SPDIF).
	ret = it661221_audio_ch_enable(ctx, true);
	if (ret) {
		dev_err(dev, "Failed to enable audio channel: %d\n", ret);
		goto out;
	}

	ret = regmap_write_bits(ctx->regmap, IT66121_INT_MASK1_REG,
				IT66121_INT_MASK1_AUD_OVF,
				0);
	if (ret)
		goto out;

	dev_dbg(dev, "HDMI audio enabled.\n");
out:
	mutex_unlock(&ctx->lock);

	return ret;
}

static int it66121_audio_startup(struct device *dev, void *data)
{
	int ret;
	struct it66121_ctx *ctx = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);

	mutex_lock(&ctx->lock);
	ret = it661221_audio_output_enable(ctx, true);
	if (ret)
		dev_err(dev, "Failed to enable audio output: %d\n", ret);

	mutex_unlock(&ctx->lock);

	return ret;
}

static void it66121_audio_shutdown(struct device *dev, void *data)
{
	int ret;
	struct it66121_ctx *ctx = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);

	mutex_lock(&ctx->lock);
	ret = it661221_audio_output_enable(ctx, false);
	if (ret)
		dev_err(dev, "Failed to disable audio output: %d\n", ret);

	mutex_unlock(&ctx->lock);
}

static int it66121_audio_mute(struct device *dev, void *data,
			      bool enable, int direction)
{
	int ret;
	struct it66121_ctx *ctx = dev_get_drvdata(dev);

	dev_dbg(dev, "%s: enable=%s, direction=%d\n",
		__func__, enable ? "true" : "false", direction);

	mutex_lock(&ctx->lock);

	if (enable) {
		ret = regmap_write_bits(ctx->regmap, IT66121_AUD_SRCVALID_FLAT_REG,
					IT66121_AUD_FLAT_SRC0 | IT66121_AUD_FLAT_SRC1 |
					IT66121_AUD_FLAT_SRC2 | IT66121_AUD_FLAT_SRC3,
					IT66121_AUD_FLAT_SRC0 | IT66121_AUD_FLAT_SRC1 |
					IT66121_AUD_FLAT_SRC2 | IT66121_AUD_FLAT_SRC3);
	} else {
		ret = regmap_write_bits(ctx->regmap, IT66121_AUD_SRCVALID_FLAT_REG,
					IT66121_AUD_FLAT_SRC0 | IT66121_AUD_FLAT_SRC1 |
					IT66121_AUD_FLAT_SRC2 | IT66121_AUD_FLAT_SRC3,
					0);
	}

	mutex_unlock(&ctx->lock);

	return ret;
}

static int it66121_audio_get_eld(struct device *dev, void *data,
				 u8 *buf, size_t len)
{
	struct it66121_ctx *ctx = dev_get_drvdata(dev);

	mutex_lock(&ctx->lock);

	memcpy(buf, ctx->connector->eld,
	       min(sizeof(ctx->connector->eld), len));

	mutex_unlock(&ctx->lock);

	return 0;
}

static const struct hdmi_codec_ops it66121_audio_codec_ops = {
	.hw_params = it66121_audio_hw_params,
	.audio_startup = it66121_audio_startup,
	.audio_shutdown = it66121_audio_shutdown,
	.mute_stream = it66121_audio_mute,
	.get_eld = it66121_audio_get_eld,
	.no_capture_mute = 1,
};

static int it66121_audio_codec_init(struct it66121_ctx *ctx, struct device *dev)
{
	struct hdmi_codec_pdata codec_data = {
		.ops = &it66121_audio_codec_ops,
		.i2s = 1, /* Only i2s support for now */
		.spdif = 0,
		.max_i2s_channels = 8,
	};

	dev_dbg(dev, "%s\n", __func__);

	if (!of_property_read_bool(dev->of_node, "#sound-dai-cells")) {
		dev_info(dev, "No \"#sound-dai-cells\", no audio\n");
		return 0;
	}

	ctx->audio.pdev = platform_device_register_data(dev,
							HDMI_CODEC_DRV_NAME,
							PLATFORM_DEVID_AUTO,
							&codec_data,
							sizeof(codec_data));

	if (IS_ERR(ctx->audio.pdev)) {
		dev_err(dev, "Failed to initialize HDMI audio codec: %d\n",
			PTR_ERR_OR_ZERO(ctx->audio.pdev));
	}

	return PTR_ERR_OR_ZERO(ctx->audio.pdev);
}

static int it66121_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	u32 revision_id, vendor_ids[2] = { 0 }, device_ids[2] = { 0 };
	struct device_node *ep;
	int ret;
	struct it66121_ctx *ctx;
	struct device *dev = &client->dev;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(dev, "I2C check functionality failed.\n");
		return -ENXIO;
	}

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ep = of_graph_get_endpoint_by_regs(dev->of_node, 0, 0);
	if (!ep)
		return -EINVAL;

	ctx->dev = dev;
	ctx->client = client;

	of_property_read_u32(ep, "bus-width", &ctx->bus_width);
	of_node_put(ep);

	if (ctx->bus_width != 12 && ctx->bus_width != 24)
		return -EINVAL;

	ep = of_graph_get_remote_node(dev->of_node, 1, -1);
	if (!ep) {
		dev_err(ctx->dev, "The endpoint is unconnected\n");
		return -EINVAL;
	}

	if (!of_device_is_available(ep)) {
		of_node_put(ep);
		dev_err(ctx->dev, "The remote device is disabled\n");
		return -ENODEV;
	}

	ctx->next_bridge = of_drm_find_bridge(ep);
	of_node_put(ep);
	if (!ctx->next_bridge) {
		dev_dbg(ctx->dev, "Next bridge not found, deferring probe\n");
		return -EPROBE_DEFER;
	}

	i2c_set_clientdata(client, ctx);
	mutex_init(&ctx->lock);

	ctx->supplies[0].supply = "vcn33";
	ctx->supplies[1].supply = "vcn18";
	ctx->supplies[2].supply = "vrf12";
	ret = devm_regulator_bulk_get(ctx->dev, 3, ctx->supplies);
	if (ret) {
		dev_err(ctx->dev, "regulator_bulk failed\n");
		return ret;
	}

	ret = ite66121_power_on(ctx);
	if (ret)
		return ret;

	it66121_hw_reset(ctx);

	ctx->regmap = devm_regmap_init_i2c(client, &it66121_regmap_config);
	if (IS_ERR(ctx->regmap)) {
		ite66121_power_off(ctx);
		return PTR_ERR(ctx->regmap);
	}

	regmap_read(ctx->regmap, IT66121_VENDOR_ID0_REG, &vendor_ids[0]);
	regmap_read(ctx->regmap, IT66121_VENDOR_ID1_REG, &vendor_ids[1]);
	regmap_read(ctx->regmap, IT66121_DEVICE_ID0_REG, &device_ids[0]);
	regmap_read(ctx->regmap, IT66121_DEVICE_ID1_REG, &device_ids[1]);

	/* Revision is shared with DEVICE_ID1 */
	revision_id = FIELD_GET(IT66121_REVISION_MASK, device_ids[1]);
	device_ids[1] &= IT66121_DEVICE_ID1_MASK;

	if (vendor_ids[0] != IT66121_VENDOR_ID0 || vendor_ids[1] != IT66121_VENDOR_ID1 ||
	    device_ids[0] != IT66121_DEVICE_ID0 || device_ids[1] != IT66121_DEVICE_ID1) {
		ite66121_power_off(ctx);
		return -ENODEV;
	}

	ctx->bridge.funcs = &it66121_bridge_funcs;
	ctx->bridge.of_node = dev->of_node;
	ctx->bridge.type = DRM_MODE_CONNECTOR_HDMIA;
	ctx->bridge.ops = DRM_BRIDGE_OP_DETECT | DRM_BRIDGE_OP_EDID | DRM_BRIDGE_OP_HPD;

	ret = devm_request_threaded_irq(dev, client->irq, NULL,	it66121_irq_threaded_handler,
					IRQF_ONESHOT, dev_name(dev), ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to request irq %d:%d\n", client->irq, ret);
		ite66121_power_off(ctx);
		return ret;
	}

	it66121_audio_codec_init(ctx, dev);

	drm_bridge_add(&ctx->bridge);

	dev_info(ctx->dev, "IT66121 revision %d probed\n", revision_id);

	return 0;
}

static int it66121_remove(struct i2c_client *client)
{
	struct it66121_ctx *ctx = i2c_get_clientdata(client);

	ite66121_power_off(ctx);
	drm_bridge_remove(&ctx->bridge);
	mutex_destroy(&ctx->lock);

	return 0;
}

static const struct of_device_id it66121_dt_match[] = {
	{ .compatible = "ite,it66121" },
	{ }
};
MODULE_DEVICE_TABLE(of, it66121_dt_match);

static const struct i2c_device_id it66121_id[] = {
	{ "it66121", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, it66121_id);

static struct i2c_driver it66121_driver = {
	.driver = {
		.name	= "it66121",
		.of_match_table = it66121_dt_match,
	},
	.probe = it66121_probe,
	.remove = it66121_remove,
	.id_table = it66121_id,
};

module_i2c_driver(it66121_driver);

MODULE_AUTHOR("Phong LE");
MODULE_DESCRIPTION("IT66121 HDMI transmitter driver");
MODULE_LICENSE("GPL v2");
