// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the Texas Instruments DS90UB960-Q1 video deserializer
 *
 * Copyright (c) 2019 Luca Ceresoli <luca@lucaceresoli.net>
 * Copyright (c) 2021 Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c-atr.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-subdev.h>

#define UB960_MAX_RX_NPORTS	4
#define UB960_MAX_TX_NPORTS	2
#define UB960_MAX_NPORTS	(UB960_MAX_RX_NPORTS + UB960_MAX_TX_NPORTS)

#define UB960_NUM_SLAVE_ALIASES	8
#define UB960_MAX_POOL_ALIASES	(UB960_MAX_RX_NPORTS * UB960_NUM_SLAVE_ALIASES)

#define UB960_MAX_VC		4

/*
 * Register map
 *
 * 0x00-0x32   Shared (UB960_SR)
 * 0x33-0x3A   CSI-2 TX (per-port paged on DS90UB960, shared on 954) (UB960_TR)
 * 0x4C        Shared (UB960_SR)
 * 0x4D-0x7F   FPD-Link RX, per-port paged (UB960_RR)
 * 0xB0-0xBF   Shared (UB960_SR)
 * 0xD0-0xDF   FPD-Link RX, per-port paged (UB960_RR)
 * 0xF0-0xF5   Shared (UB960_SR)
 * 0xF8-0xFB   Shared (UB960_SR)
 * All others  Reserved
 *
 * Register prefixes:
 * UB960_SR_* = Shared register
 * UB960_RR_* = FPD-Link RX, per-port paged register
 * UB960_TR_* = CSI-2 TX, per-port paged register
 * UB960_XR_* = Reserved register
 * UB960_IR_* = Indirect register
 */

#define UB960_SR_I2C_DEV_ID			0x00
#define UB960_SR_RESET				0x01
#define UB960_SR_GEN_CONFIG			0x02
#define UB960_SR_REV_MASK			0x03
#define UB960_SR_DEVICE_STS			0x04
#define UB960_SR_PAR_ERR_THOLD_HI		0x05
#define UB960_SR_PAR_ERR_THOLD_LO		0x06
#define UB960_SR_BCC_WDOG_CTL			0x07
#define UB960_SR_I2C_CTL1			0x08
#define UB960_SR_I2C_CTL2			0x09
#define UB960_SR_SCL_HIGH_TIME			0x0A
#define UB960_SR_SCL_LOW_TIME			0x0B
#define UB960_SR_RX_PORT_CTL			0x0C
#define UB960_SR_IO_CTL				0x0D
#define UB960_SR_GPIO_PIN_STS			0x0E
#define UB960_SR_GPIO_INPUT_CTL			0x0F
#define UB960_SR_GPIO_PIN_CTL(n)		(0x10 + (n)) /* n < UB960_NUM_GPIOS */
#define UB960_SR_FS_CTL				0x18
#define UB960_SR_FS_HIGH_TIME_1			0x19
#define UB960_SR_FS_HIGH_TIME_0			0x1A
#define UB960_SR_FS_LOW_TIME_1			0x1B
#define UB960_SR_FS_LOW_TIME_0			0x1C
#define UB960_SR_MAX_FRM_HI			0x1D
#define UB960_SR_MAX_FRM_LO			0x1E
#define UB960_SR_CSI_PLL_CTL			0x1F

#define UB960_SR_FWD_CTL1			0x20
#define UB960_SR_FWD_CTL1_PORT_DIS(n)		BIT((n) + 4)

#define UB960_SR_FWD_CTL2			0x21
#define UB960_SR_FWD_STS			0x22

#define UB960_SR_INTERRUPT_CTL			0x23
#define UB960_SR_INTERRUPT_CTL_INT_EN		BIT(7)
#define UB960_SR_INTERRUPT_CTL_IE_CSI_TX0	BIT(4)
#define UB960_SR_INTERRUPT_CTL_IE_RX(n)		BIT((n)) /* rxport[n] IRQ */
#define UB960_SR_INTERRUPT_CTL_ALL		0x83 /* TODO 0x93 to enable CSI */

#define UB960_SR_INTERRUPT_STS			0x24
#define UB960_SR_INTERRUPT_STS_INT		BIT(7)
#define UB960_SR_INTERRUPT_STS_IS_CSI_TX(n)	BIT(4 + (n)) /* txport[n] IRQ */
#define UB960_SR_INTERRUPT_STS_IS_RX(n)		BIT((n)) /* rxport[n] IRQ */

#define UB960_SR_TS_CONFIG			0x25
#define UB960_SR_TS_CONTROL			0x26
#define UB960_SR_TS_LINE_HI			0x27
#define UB960_SR_TS_LINE_LO			0x28
#define UB960_SR_TS_STATUS			0x29
#define UB960_SR_TIMESTAMP_P0_HI		0x2A
#define UB960_SR_TIMESTAMP_P0_LO		0x2B
#define UB960_SR_TIMESTAMP_P1_HI		0x2C
#define UB960_SR_TIMESTAMP_P1_LO		0x2D

#define UB960_SR_CSI_PORT_SEL			0x32

#define UB960_TR_CSI_CTL			0x33
#define UB960_TR_CSI_CTL_CSI_CAL_EN		BIT(6)
#define UB960_TR_CSI_CTL_CSI_ENABLE		BIT(0)

#define UB960_TR_CSI_CTL2			0x34
#define UB960_TR_CSI_STS			0x35
#define UB960_TR_CSI_TX_ICR			0x36

#define UB960_TR_CSI_TX_ISR			0x37
#define UB960_TR_CSI_TX_ISR_IS_CSI_SYNC_ERROR	BIT(3)
#define UB960_TR_CSI_TX_ISR_IS_CSI_PASS_ERROR	BIT(1)

#define UB960_TR_CSI_TEST_CTL			0x38
#define UB960_TR_CSI_TEST_PATT_HI		0x39
#define UB960_TR_CSI_TEST_PATT_LO		0x3A

#define UB960_XR_AEQ_CTL1			0x42
#define UB960_XR_AEQ_ERR_THOLD			0x43

#define UB960_RR_BCC_ERR_CTL			0x46
#define UB960_RR_BCC_STATUS			0x47

#define UB960_RR_FPD3_CAP			0x4A
#define UB960_RR_RAW_EMBED_DTYPE		0x4B

#define UB960_SR_FPD3_PORT_SEL			0x4C

#define UB960_RR_RX_PORT_STS1			0x4D
#define UB960_RR_RX_PORT_STS1_BCC_CRC_ERROR	BIT(5)
#define UB960_RR_RX_PORT_STS1_LOCK_STS_CHG	BIT(4)
#define UB960_RR_RX_PORT_STS1_BCC_SEQ_ERROR	BIT(3)
#define UB960_RR_RX_PORT_STS1_PARITY_ERROR	BIT(2)
#define UB960_RR_RX_PORT_STS1_PORT_PASS		BIT(1)
#define UB960_RR_RX_PORT_STS1_LOCK_STS		BIT(0)

#define UB960_RR_RX_PORT_STS2			0x4E
#define UB960_RR_RX_PORT_STS2_LINE_LEN_UNSTABLE	BIT(7)
#define UB960_RR_RX_PORT_STS2_LINE_LEN_CHG	BIT(6)
#define UB960_RR_RX_PORT_STS2_FPD3_ENCODE_ERROR	BIT(5)
#define UB960_RR_RX_PORT_STS2_BUFFER_ERROR	BIT(4)
#define UB960_RR_RX_PORT_STS2_CSI_ERROR		BIT(3)
#define UB960_RR_RX_PORT_STS2_FREQ_STABLE	BIT(2)
#define UB960_RR_RX_PORT_STS2_CABLE_FAULT	BIT(1)
#define UB960_RR_RX_PORT_STS2_LINE_CNT_CHG	BIT(0)

#define UB960_RR_RX_FREQ_HIGH			0x4F
#define UB960_RR_RX_FREQ_LOW			0x50
#define UB960_RR_SENSOR_STS_0			0x51
#define UB960_RR_SENSOR_STS_1			0x52
#define UB960_RR_SENSOR_STS_2			0x53
#define UB960_RR_SENSOR_STS_3			0x54
#define UB960_RR_RX_PAR_ERR_HI			0x55
#define UB960_RR_RX_PAR_ERR_LO			0x56
#define UB960_RR_BIST_ERR_COUNT			0x57

#define UB960_RR_BCC_CONFIG			0x58
#define UB960_RR_BCC_CONFIG_I2C_PASS_THROUGH	BIT(6)

#define UB960_RR_DATAPATH_CTL1			0x59
#define UB960_RR_DATAPATH_CTL2			0x5A
#define UB960_RR_SER_ID				0x5B
#define UB960_RR_SER_ALIAS_ID			0x5C

/* For these two register sets: n < UB960_NUM_SLAVE_ALIASES */
#define UB960_RR_SLAVE_ID(n)			(0x5D + (n))
#define UB960_RR_SLAVE_ALIAS(n)			(0x65 + (n))

#define UB960_RR_PORT_CONFIG			0x6D
#define UB960_RR_BC_GPIO_CTL(n)			(0x6E + (n)) /* n < 2 */
#define UB960_RR_RAW10_ID			0x70
#define UB960_RR_RAW12_ID			0x71
#define UB960_RR_CSI_VC_MAP			0x72
#define UB960_RR_LINE_COUNT_HI			0x73
#define UB960_RR_LINE_COUNT_LO			0x74
#define UB960_RR_LINE_LEN_1			0x75
#define UB960_RR_LINE_LEN_0			0x76
#define UB960_RR_FREQ_DET_CTL			0x77
#define UB960_RR_MAILBOX_1			0x78
#define UB960_RR_MAILBOX_2			0x79

#define UB960_RR_CSI_RX_STS			0x7A
#define UB960_RR_CSI_RX_STS_LENGTH_ERR		BIT(3)
#define UB960_RR_CSI_RX_STS_CKSUM_ERR		BIT(2)
#define UB960_RR_CSI_RX_STS_ECC2_ERR		BIT(1)
#define UB960_RR_CSI_RX_STS_ECC1_ERR		BIT(0)

#define UB960_RR_CSI_ERR_COUNTER		0x7B
#define UB960_RR_PORT_CONFIG2			0x7C
#define UB960_RR_PORT_PASS_CTL			0x7D
#define UB960_RR_SEN_INT_RISE_CTL		0x7E
#define UB960_RR_SEN_INT_FALL_CTL		0x7F

#define UB960_XR_REFCLK_FREQ			0xA5

#define UB960_SR_IND_ACC_CTL			0xB0
#define UB960_SR_IND_ACC_CTL_IA_AUTO_INC	BIT(1)

#define UB960_SR_IND_ACC_ADDR			0xB1
#define UB960_SR_IND_ACC_DATA			0xB2
#define UB960_SR_BIST_CONTROL			0xB3
#define UB960_SR_MODE_IDX_STS			0xB8
#define UB960_SR_LINK_ERROR_COUNT		0xB9
#define UB960_SR_FPD3_ENC_CTL			0xBA
#define UB960_SR_FV_MIN_TIME			0xBC
#define UB960_SR_GPIO_PD_CTL			0xBE

#define UB960_RR_PORT_DEBUG			0xD0
#define UB960_RR_AEQ_CTL2			0xD2
#define UB960_RR_AEQ_STATUS			0xD3
#define UB960_RR_AEQ_BYPASS			0xD4
#define UB960_RR_AEQ_MIN_MAX			0xD5
#define UB960_RR_PORT_ICR_HI			0xD8
#define UB960_RR_PORT_ICR_LO			0xD9
#define UB960_RR_PORT_ISR_HI			0xDA
#define UB960_RR_PORT_ISR_LO			0xDB
#define UB960_RR_FC_GPIO_STS			0xDC
#define UB960_RR_FC_GPIO_ICR			0xDD
#define UB960_RR_SEN_INT_RISE_STS		0xDE
#define UB960_RR_SEN_INT_FALL_STS		0xDF

#define UB960_SR_FPD3_RX_ID0			0xF0
#define UB960_SR_FPD3_RX_ID1			0xF1
#define UB960_SR_FPD3_RX_ID2			0xF2
#define UB960_SR_FPD3_RX_ID3			0xF3
#define UB960_SR_FPD3_RX_ID4			0xF4
#define UB960_SR_FPD3_RX_ID5			0xF5
#define UB960_SR_I2C_RX_ID(n)			(0xF8 + (n)) /* < UB960_FPD_RX_NPORTS */

/* UB960_IR_PGEN_*: Indirect Registers for Test Pattern Generator */

#define UB960_IR_PGEN_CTL			0x01
#define UB960_IR_PGEN_CTL_PGEN_ENABLE		BIT(0)

#define UB960_IR_PGEN_CFG			0x02
#define UB960_IR_PGEN_CSI_DI			0x03
#define UB960_IR_PGEN_LINE_SIZE1		0x04
#define UB960_IR_PGEN_LINE_SIZE0		0x05
#define UB960_IR_PGEN_BAR_SIZE1			0x06
#define UB960_IR_PGEN_BAR_SIZE0			0x07
#define UB960_IR_PGEN_ACT_LPF1			0x08
#define UB960_IR_PGEN_ACT_LPF0			0x09
#define UB960_IR_PGEN_TOT_LPF1			0x0A
#define UB960_IR_PGEN_TOT_LPF0			0x0B
#define UB960_IR_PGEN_LINE_PD1			0x0C
#define UB960_IR_PGEN_LINE_PD0			0x0D
#define UB960_IR_PGEN_VBP			0x0E
#define UB960_IR_PGEN_VFP			0x0F
#define UB960_IRT_PGEN_COLOR(n)			(0x10 + (n)) /* n < 15 */

struct ub960_hw_data {
	u8 num_rxports;
	u8 num_txports;
};

enum ub960_rxport_mode {
	RXPORT_MODE_RAW10 = 0,
	RXPORT_MODE_RAW12_HF = 1,
	RXPORT_MODE_RAW12_LF = 2,
	RXPORT_MODE_CSI2 = 3,
};

struct ub960_rxport {
	struct ub960_data      *priv;
	u8                      nport;	/* RX port number, and index in priv->rxport[] */

	struct v4l2_subdev     *sd;	/* Connected subdev */
	struct fwnode_handle   *fwnode;

	enum ub960_rxport_mode  mode;

	struct device_node     *remote_of_node;	/* "remote-chip" OF node */
	struct i2c_client      *ser_client;	/* remote serializer */
	unsigned short          ser_alias;	/* ser i2c alias (lower 7 bits) */
	bool                    locked;
};

struct ub960_asd {
	struct v4l2_async_subdev base;
	struct ub960_rxport *rxport;
};

static inline struct ub960_asd *to_ub960_asd(struct v4l2_async_subdev *asd)
{
	return container_of(asd, struct ub960_asd, base);
}

struct ub960_txport {
	u32 num_data_lanes;
};

struct ub960_vc_map {
	u8	vc_map[UB960_MAX_RX_NPORTS];
	bool	port_en[UB960_MAX_RX_NPORTS];
};

struct ub960_data {
	const struct ub960_hw_data	*hw_data;
	struct i2c_client	*client; /* for shared local registers */
	struct regmap		*regmap;
	struct gpio_desc	*pd_gpio;
	struct task_struct	*kthread;
	struct i2c_atr		*atr;
	struct ub960_rxport	*rxports[UB960_MAX_RX_NPORTS];
	struct ub960_txport	*txports[UB960_MAX_TX_NPORTS];
	struct ub960_vc_map	vc_map;

	struct v4l2_subdev	sd;
	struct media_pad	pads[UB960_MAX_NPORTS];

	struct v4l2_ctrl_handler   ctrl_handler;
	struct v4l2_async_notifier notifier;

	struct clk		*refclk;
	struct clk_hw		*line_clk_hw;

	u32 tx_data_rate;		/* Nominal data rate (Gb/s) */
	s64 tx_link_freq[1];

	/* Address Translator alias-to-slave map table */
	size_t	     atr_alias_num; /* Number of aliases configured */
	u16	     atr_alias_id[UB960_MAX_POOL_ALIASES]; /* 0 = no alias */
	u16	     atr_slave_id[UB960_MAX_POOL_ALIASES]; /* 0 = not in use */
	struct mutex alias_table_lock;

	u8 current_read_rxport;
	u8 current_write_rxport_mask;

	u8 current_read_csiport;
	u8 current_write_csiport_mask;

	bool streaming;
};

static inline struct ub960_data *sd_to_ub960(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ub960_data, sd);
}

enum {
	TEST_PATTERN_DISABLED = 0,
	TEST_PATTERN_V_COLOR_BARS_1,
	TEST_PATTERN_V_COLOR_BARS_2,
	TEST_PATTERN_V_COLOR_BARS_4,
	TEST_PATTERN_V_COLOR_BARS_8,
};

static const char * const ub960_tpg_qmenu[] = {
	"Disabled",
	"1 vertical color bar",
	"2 vertical color bars",
	"4 vertical color bars",
	"8 vertical color bars",
};

static inline bool ub960_pad_is_sink(struct ub960_data *priv, u32 pad)
{
	return pad < priv->hw_data->num_rxports;
}

static inline bool ub960_pad_is_source(struct ub960_data *priv, u32 pad)
{
	return pad >= priv->hw_data->num_rxports &&
	       pad < (priv->hw_data->num_rxports + priv->hw_data->num_txports);
}

struct ub960_format_info {
	u32 code;
	u32 bpp;
	u8 datatype;
	bool meta;
};

static const struct ub960_format_info ub960_formats[] = {
	{ .code = MEDIA_BUS_FMT_YUYV8_1X16, .bpp = 16, .datatype = 0x1e, },
	{ .code = MEDIA_BUS_FMT_UYVY8_1X16, .bpp = 16, .datatype = 0x1e, },
	{ .code = MEDIA_BUS_FMT_VYUY8_1X16, .bpp = 16, .datatype = 0x1e, },
	{ .code = MEDIA_BUS_FMT_YVYU8_1X16, .bpp = 16, .datatype = 0x1e, },

	/* Legacy */
	{ .code = MEDIA_BUS_FMT_YUYV8_2X8, .bpp = 16, .datatype = 0x1e, },
	{ .code = MEDIA_BUS_FMT_UYVY8_2X8, .bpp = 16, .datatype = 0x1e, },
	{ .code = MEDIA_BUS_FMT_VYUY8_2X8, .bpp = 16, .datatype = 0x1e, },
	{ .code = MEDIA_BUS_FMT_YVYU8_2X8, .bpp = 16, .datatype = 0x1e, },

	/* RAW */
	{ .code = MEDIA_BUS_FMT_SBGGR12_1X12, .bpp = 12, .datatype = 0x2c, },
	{ .code = MEDIA_BUS_FMT_SRGGB12_1X12, .bpp = 12, .datatype = 0x2c, },
};

static const struct ub960_format_info *ub960_find_format(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ub960_formats); ++i) {
		if (ub960_formats[i].code == code)
			return &ub960_formats[i];
	}

	return NULL;
}

/* -----------------------------------------------------------------------------
 * Basic device access
 */

static int ub960_read(const struct ub960_data *priv, u8 reg, u8 *val)
{
	struct device *dev = &priv->client->dev;
	unsigned int v;
	int ret;

	ret = regmap_read(priv->regmap, reg, &v);
	if (ret) {
		dev_err(dev, "%s: cannot read register 0x%02x (%d)!\n",
			__func__, reg, ret);
		return ret;
	}

	*val = v;

	return 0;
}

static int ub960_write(const struct ub960_data *priv, u8 reg, u8 val)
{
	struct device *dev = &priv->client->dev;
	int ret;

	ret = regmap_write(priv->regmap, reg, val);
	if (ret < 0)
		dev_err(dev, "%s: cannot write register 0x%02x (%d)!\n",
			__func__, reg, ret);

	return ret;
}

static int ub960_update_bits_shared(const struct ub960_data *priv, u8 reg,
				    u8 mask, u8 val)
{
	struct device *dev = &priv->client->dev;
	int ret;

	ret = regmap_update_bits(priv->regmap, reg, mask, val);
	if (ret < 0)
		dev_err(dev, "%s: cannot update register 0x%02x (%d)!\n",
			__func__, reg, ret);

	return ret;
}

static int ub960_rxport_select(struct ub960_data *priv, u8 nport)
{
	struct device *dev = &priv->client->dev;
	int ret;

	if (priv->current_read_rxport == nport &&
	    priv->current_write_rxport_mask == BIT(nport))
		return 0;

	ret = regmap_write(priv->regmap, UB960_SR_FPD3_PORT_SEL,
			   (nport << 4) | (1 << nport));
	if (ret) {
		dev_err(dev, "%s: cannot select rxport %d (%d)!\n", __func__,
			nport, ret);
		return ret;
	}

	priv->current_read_rxport = nport;
	priv->current_write_rxport_mask = BIT(nport);

	return 0;
}

static int ub960_rxport_read(struct ub960_data *priv, u8 nport, u8 reg,
			     u8 *val)
{
	struct device *dev = &priv->client->dev;
	unsigned int v;
	int ret;

	ub960_rxport_select(priv, nport);

	ret = regmap_read(priv->regmap, reg, &v);
	if (ret) {
		dev_err(dev, "%s: cannot read register 0x%02x (%d)!\n",
			__func__, reg, ret);
		return ret;
	}

	*val = v;

	return 0;
}

static int ub960_rxport_write(struct ub960_data *priv, u8 nport, u8 reg,
			      u8 val)
{
	struct device *dev = &priv->client->dev;
	int ret;

	ub960_rxport_select(priv, nport);

	ret = regmap_write(priv->regmap, reg, val);
	if (ret)
		dev_err(dev, "%s: cannot write register 0x%02x (%d)!\n",
			__func__, reg, ret);

	return ret;
}

static int ub960_rxport_update_bits(struct ub960_data *priv, u8 nport, u8 reg,
				    u8 mask, u8 val)
{
	struct device *dev = &priv->client->dev;
	int ret;

	ub960_rxport_select(priv, nport);

	ret = regmap_update_bits(priv->regmap, reg, mask, val);

	if (ret)
		dev_err(dev, "%s: cannot update register 0x%02x (%d)!\n",
			__func__, reg, ret);

	return ret;
}

static int ub960_csiport_select(struct ub960_data *priv, u8 nport)
{
	struct device *dev = &priv->client->dev;
	int ret;

	if (priv->current_read_csiport == nport &&
	    priv->current_write_csiport_mask == BIT(nport))
		return 0;

	ret = regmap_write(priv->regmap, UB960_SR_CSI_PORT_SEL,
			   (nport << 4) | (1 << nport));
	if (ret) {
		dev_err(dev, "%s: cannot select csi port %d (%d)!\n", __func__,
			nport, ret);
		return ret;
	}

	priv->current_read_csiport = nport;
	priv->current_write_csiport_mask = BIT(nport);

	return 0;
}

static int ub960_csiport_read(struct ub960_data *priv, u8 nport, u8 reg,
			      u8 *val)
{
	struct device *dev = &priv->client->dev;
	unsigned int v;
	int ret;

	ub960_csiport_select(priv, nport);

	ret = regmap_read(priv->regmap, reg, &v);
	if (ret) {
		dev_err(dev, "%s: cannot read register 0x%02x (%d)!\n",
			__func__, reg, ret);
		return ret;
	}

	*val = v;

	return 0;
}

static int ub960_csiport_write(struct ub960_data *priv, u8 nport, u8 reg,
			       u8 val)
{
	struct device *dev = &priv->client->dev;
	int ret;

	ub960_csiport_select(priv, nport);

	ret = regmap_write(priv->regmap, reg, val);
	if (ret)
		dev_err(dev, "%s: cannot write register 0x%02x (%d)!\n",
			__func__, reg, ret);

	return ret;
}

__maybe_unused
static int ub960_csiport_update_bits(struct ub960_data *priv, u8 nport, u8 reg,
				     u8 mask, u8 val)
{
	struct device *dev = &priv->client->dev;
	int ret;

	ub960_csiport_select(priv, nport);

	ret = regmap_update_bits(priv->regmap, reg, mask, val);

	if (ret)
		dev_err(dev, "%s: cannot update register 0x%02x (%d)!\n",
			__func__, reg, ret);

	return ret;
}

static int ub960_write_ind8(const struct ub960_data *priv, u8 reg, u8 val)
{
	int ret;

	ret = ub960_write(priv, UB960_SR_IND_ACC_ADDR, reg);
	if (!ret)
		ret = ub960_write(priv, UB960_SR_IND_ACC_DATA, val);
	return ret;
}

/* Assumes IA_AUTO_INC is set in UB960_SR_IND_ACC_CTL */
static int ub960_write_ind16(const struct ub960_data *priv, u8 reg, u16 val)
{
	int ret;

	ret = ub960_write(priv, UB960_SR_IND_ACC_ADDR, reg);
	if (!ret)
		ret = ub960_write(priv, UB960_SR_IND_ACC_DATA, val >> 8);
	if (!ret)
		ret = ub960_write(priv, UB960_SR_IND_ACC_DATA, val & 0xff);
	return ret;
}

/* -----------------------------------------------------------------------------
 * I2C-ATR (address translator)
 */

static int ub960_atr_attach_client(struct i2c_atr *atr, u32 chan_id,
				   const struct i2c_board_info *info,
				   const struct i2c_client *client,
				   u16 *alias_id)
{
	struct ub960_data *priv = i2c_atr_get_clientdata(atr);
	struct ub960_rxport *rxport = priv->rxports[chan_id];
	struct device *dev = &priv->client->dev;
	unsigned int reg_idx;
	unsigned int pool_idx;
	u16 alias = 0;
	int ret = 0;

	dev_dbg(dev, "rx%d: %s\n", chan_id, __func__);

	mutex_lock(&priv->alias_table_lock);

	/* Find unused alias in table */

	for (pool_idx = 0; pool_idx < priv->atr_alias_num; pool_idx++)
		if (priv->atr_slave_id[pool_idx] == 0)
			break;

	if (pool_idx == priv->atr_alias_num) {
		dev_warn(dev, "rx%d: alias pool exhausted\n", rxport->nport);
		ret = -EADDRNOTAVAIL;
		goto out;
	}

	alias = priv->atr_alias_id[pool_idx];

	/* Find first unused alias register */

	for (reg_idx = 0; reg_idx < UB960_NUM_SLAVE_ALIASES; reg_idx++) {
		u8 regval;

		ret = ub960_rxport_read(priv, chan_id,
					UB960_RR_SLAVE_ALIAS(reg_idx), &regval);
		if (!ret && regval == 0)
			break;
	}

	if (reg_idx == UB960_NUM_SLAVE_ALIASES) {
		dev_warn(dev, "rx%d: all aliases in use\n", rxport->nport);
		ret = -EADDRNOTAVAIL;
		goto out;
	}

	/* Map alias to slave */

	ub960_rxport_write(priv, chan_id, UB960_RR_SLAVE_ID(reg_idx),
			   client->addr << 1);
	ub960_rxport_write(priv, chan_id, UB960_RR_SLAVE_ALIAS(reg_idx),
			   alias << 1);

	priv->atr_slave_id[pool_idx] = client->addr;

	*alias_id = alias; /* tell the atr which alias we chose */

	dev_dbg(dev, "rx%d: client 0x%02x mapped at alias 0x%02x (%s)\n",
		rxport->nport, client->addr, alias, client->name);

out:
	mutex_unlock(&priv->alias_table_lock);
	return ret;
}

static void ub960_atr_detach_client(struct i2c_atr *atr, u32 chan_id,
				    const struct i2c_client *client)
{
	struct ub960_data *priv = i2c_atr_get_clientdata(atr);
	struct ub960_rxport *rxport = priv->rxports[chan_id];
	struct device *dev = &priv->client->dev;
	unsigned int reg_idx;
	unsigned int pool_idx;
	u16 alias = 0;

	mutex_lock(&priv->alias_table_lock);

	/* Find alias mapped to this client */

	for (pool_idx = 0; pool_idx < priv->atr_alias_num; pool_idx++)
		if (priv->atr_slave_id[pool_idx] == client->addr)
			break;

	if (pool_idx == priv->atr_alias_num) {
		dev_err(dev, "rx%d: client 0x%02x is not mapped!\n",
			rxport->nport, client->addr);
		goto out;
	}

	alias = priv->atr_alias_id[pool_idx];

	/* Find alias register used for this client */

	for (reg_idx = 0; reg_idx < UB960_NUM_SLAVE_ALIASES; reg_idx++) {
		u8 regval;
		int ret;

		ret = ub960_rxport_read(priv, chan_id,
					UB960_RR_SLAVE_ALIAS(reg_idx), &regval);
		if (!ret && regval == (alias << 1))
			break;
	}

	if (reg_idx == UB960_NUM_SLAVE_ALIASES) {
		dev_err(dev,
			"rx%d: cannot find alias 0x%02x reg (client 0x%02x)!\n",
			rxport->nport, alias, client->addr);
		goto out;
	}

	/* Unmap */

	ub960_rxport_write(priv, chan_id, UB960_RR_SLAVE_ALIAS(reg_idx), 0);
	priv->atr_slave_id[pool_idx] = 0;

	dev_dbg(dev, "rx%d: client 0x%02x unmapped from alias 0x%02x (%s)\n",
		rxport->nport, client->addr, alias, client->name);

out:
	mutex_unlock(&priv->alias_table_lock);
}

static const struct i2c_atr_ops ub960_atr_ops = {
	.attach_client = ub960_atr_attach_client,
	.detach_client = ub960_atr_detach_client,
};

/* -----------------------------------------------------------------------------
 * CSI ports
 */

static int ub960_csiport_probe_one(struct ub960_data *priv,
				   const struct device_node *np,
				   u8 nport)
{
	struct device *dev = &priv->client->dev;
	struct ub960_txport *txport;
	int ret;

	if (priv->txports[nport]) {
		dev_err(dev, "OF: %s: duplicate tx port\n",
			of_node_full_name(np));
		return -EADDRINUSE;
	}

	txport = kzalloc(sizeof(*txport), GFP_KERNEL);
	if (!txport)
		return -ENOMEM;

	priv->txports[nport] = txport;

	ret = of_property_count_u32_elems(np, "data-lanes");

	if (ret <= 0) {
		dev_err(dev, "OF: %s: failed to parse data-lanes: %d\n",
			of_node_full_name(np), ret);
		goto err_free_txport;
	}

	txport->num_data_lanes = ret;

	return 0;

err_free_txport:
	kfree(txport);

	return ret;
}

static void ub960_txport_remove_one(struct ub960_data *priv, u8 nport)
{
	struct ub960_txport *txport = priv->txports[nport];

	kfree(txport);
	priv->txports[nport] = NULL;
}

static void ub960_csi_handle_events(struct ub960_data *priv, u8 nport)
{
	struct device *dev = &priv->client->dev;
	u8 csi_tx_isr;
	int ret;

	ret = ub960_csiport_read(priv, nport, UB960_TR_CSI_TX_ISR, &csi_tx_isr);

	if (!ret) {
		if (csi_tx_isr & UB960_TR_CSI_TX_ISR_IS_CSI_SYNC_ERROR)
			dev_warn(dev, "TX%u: CSI_SYNC_ERROR\n", nport);

		if (csi_tx_isr & UB960_TR_CSI_TX_ISR_IS_CSI_PASS_ERROR)
			dev_warn(dev, "TX%u: CSI_PASS_ERROR\n", nport);
	}
}

/* -----------------------------------------------------------------------------
 * RX ports
 */

/*
 * Instantiate serializer and i2c adapter for a locked remote end.
 *
 * Must be called with priv->alias_table_lock not held! The added i2c adapter
 * will probe new slaves, which can request i2c transfers, ending up in
 * calling ub960_atr_attach_client() where the lock is taken.
 */
static int ub960_rxport_add_serializer(struct ub960_data *priv, u8 nport)
{
	struct ub960_rxport *rxport = priv->rxports[nport];
	struct device *dev = &priv->client->dev;
	struct i2c_board_info ser_info = {
		.of_node = rxport->remote_of_node,
	};

	/*
	 * Adding the serializer under rxport->adap would be cleaner, but it
	 * would need tweaks to bypass the alias table. Adding to the
	 * upstream adapter is way simpler.
	 */
	ser_info.addr = rxport->ser_alias;
	rxport->ser_client =
		i2c_new_client_device(priv->client->adapter, &ser_info);
	if (!rxport->ser_client) {
		dev_err(dev, "rx%d: cannot add %s i2c device", nport,
			ser_info.type);
		return -EIO;
	}

	dev_dbg(dev, "rx%d: remote serializer at alias 0x%02x\n", nport,
		rxport->ser_client->addr);

	return 0;
}

static void ub960_rxport_remove_serializer(struct ub960_data *priv, u8 nport)
{
	struct ub960_rxport *rxport = priv->rxports[nport];

	if (rxport->ser_client) {
		i2c_unregister_device(rxport->ser_client);
		rxport->ser_client = NULL;
	}
}

static int ub960_rxport_probe_serializers(struct ub960_data *priv)
{
	struct device *dev = &priv->client->dev;
	unsigned long timeout;
	u8 nport;
	unsigned int missing = 0;

	timeout = jiffies + msecs_to_jiffies(750);

	while (time_before(jiffies, timeout)) {
		missing = 0;

		for (nport = 0; nport < priv->hw_data->num_rxports; ++nport) {
			struct ub960_rxport *rxport = priv->rxports[nport];
			u8 rx_port_sts1;
			int ret;

			/* No serializer in DT? */
			if (!rxport)
				continue;

			/* Serializer already added? */
			if (rxport->ser_client)
				continue;

			ret = ub960_rxport_read(priv, nport,
						UB960_RR_RX_PORT_STS1,
						&rx_port_sts1);
			if (ret)
				return ret;

			/* Serializer not locked yet? */
			if (!(rx_port_sts1 & UB960_RR_RX_PORT_STS1_LOCK_STS)) {
				missing++;
				continue;
			}

			ret = ub960_rxport_add_serializer(priv, nport);
			if (ret)
				return ret;

			rxport->locked = true;
		}

		if (missing == 0)
			return 0;

		usleep_range(500, 5000);
	}

	dev_err(dev, "timeout, continuing with %u missing serializer(s)\n",
		missing);

	return 0;
}

/*
 * Return the local alias for a given remote serializer.
 */
static int ub960_of_get_reg(struct device_node *np, const char *serializer_name)
{
	u32 alias;
	int ret;
	int idx;

	if (!np)
		return -ENODEV;

	idx = of_property_match_string(np, "reg-names", serializer_name);
	if (idx < 0)
		return idx;

	ret = of_property_read_u32_index(np, "reg", idx, &alias);
	if (ret)
		return ret;

	return alias;
}

static int ub960_rxport_probe_one(struct ub960_data *priv,
				  const struct device_node *np,
				  u8 nport)
{
	const char *ser_names[UB960_MAX_RX_NPORTS] = { "ser0", "ser1", "ser2",
						       "ser3" };
	struct device *dev = &priv->client->dev;
	struct ub960_rxport *rxport;
	u32 bc_freq, bc_freq_val;
	int ret;
	u32 mode;

	if (priv->rxports[nport]) {
		dev_err(dev, "OF: %s: reg value %d is duplicated\n",
			of_node_full_name(np), nport);
		return -EADDRINUSE;
	}

	rxport = kzalloc(sizeof(*rxport), GFP_KERNEL);
	if (!rxport)
		return -ENOMEM;

	priv->rxports[nport] = rxport;

	rxport->nport = nport;
	rxport->priv = priv;

	ret = ub960_of_get_reg(priv->client->dev.of_node, ser_names[nport]);
	if (ret < 0)
		goto err_free_rxport;

	rxport->ser_alias = ret;

	ret = of_property_read_u32(np, "mode", &mode);
	if (ret < 0) {
		dev_err(dev, "Missing RX port mode: %d\n", ret);
		goto err_free_rxport;
	}

	if (mode > RXPORT_MODE_CSI2) {
		dev_err(dev, "Bad RX port mode %u\n", mode);
		goto err_free_rxport;
	}

	rxport->mode = mode;

	ret = of_property_read_u32(np, "bc-freq", &bc_freq);
	if (ret < 0) {
		dev_err(dev, "Failed to read BC freq for port %u: %d\n", nport,
			ret);
		goto err_free_rxport;
	}

	switch (bc_freq) {
	case 2500000:
		bc_freq_val = 0; break;
	case 10000000:
		bc_freq_val = 2; break;
	case 50000000:
		bc_freq_val = 6; break;
	default:
		dev_err(dev, "Bad BC freq %u\n", bc_freq);
		goto err_free_rxport;
	}

	rxport->remote_of_node = of_get_child_by_name(np, "remote-chip");
	if (!rxport->remote_of_node) {
		dev_err(dev, "OF: %s: missing remote-chip child\n",
			of_node_full_name(np));
		ret = -EINVAL;
		goto err_free_rxport;
	}

	rxport->fwnode = fwnode_graph_get_remote_endpoint(of_fwnode_handle(np));
	if (!rxport->fwnode) {
		dev_err(dev, "No remote endpoint for rxport%d\n", nport);
		ret = -ENODEV;
		goto err_node_put;
	}

	/*
	 * Back channel frequency select.
	 * Override FREQ_SELECT from the strap.
	 * 0 - 2.5 Mbps (DS90UB913A-Q1 / DS90UB933-Q1)
	 * 2 - 10 Mbps
	 * 6 - 50 Mbps (DS90UB953-Q1)
	 *
	 * Note that changing this setting will result in some errors on the back
	 * channel for a short period of time.
	 */
	ub960_rxport_update_bits(priv, nport, UB960_RR_BCC_CONFIG, 0x7,
				 bc_freq_val);

	switch (rxport->mode) {
	default:
		WARN_ON(true);
		fallthrough;

	case RXPORT_MODE_RAW10:
		/* FPD3_MODE = RAW10 Mode (DS90UB913A-Q1 / DS90UB933-Q1 compatible) */
		ub960_rxport_update_bits(priv, nport, UB960_RR_PORT_CONFIG, 0x3,
					 0x3);

		/*
		 * RAW10_8BIT_CTL = 0b11 : 8-bit processing using lower 8 bits
		 * 0b10 : 8-bit processing using upper 8 bits
		 */
		ub960_rxport_update_bits(priv, nport, UB960_RR_PORT_CONFIG2,
					 0x3 << 6, 0x2 << 6);

		break;

	case RXPORT_MODE_CSI2:
		/* CSI-2 Mode (DS90UB953-Q1 compatible) */
		ub960_rxport_update_bits(priv, nport, UB960_RR_PORT_CONFIG, 0x3,
					 0x0);

		break;
	}

	/* LV_POLARITY & FV_POLARITY */
	ub960_rxport_update_bits(priv, nport, UB960_RR_PORT_CONFIG2, 0x3, 0x1);

	/* Enable all interrupt sources from this port */
	ub960_rxport_write(priv, nport, UB960_RR_PORT_ICR_HI, 0x07);
	ub960_rxport_write(priv, nport, UB960_RR_PORT_ICR_LO, 0x7f);

	/* Enable I2C_PASS_THROUGH */
	ub960_rxport_update_bits(priv, nport, UB960_RR_BCC_CONFIG,
				 UB960_RR_BCC_CONFIG_I2C_PASS_THROUGH,
				 UB960_RR_BCC_CONFIG_I2C_PASS_THROUGH);

	/* Enable I2C communication to the serializer via the alias addr */
	ub960_rxport_write(priv, nport, UB960_RR_SER_ALIAS_ID,
			   rxport->ser_alias << 1);

	dev_dbg(dev, "ser%d: at alias 0x%02x\n", nport, rxport->ser_alias);

	ret = i2c_atr_add_adapter(priv->atr, nport);
	if (ret) {
		dev_err(dev, "rx%d: cannot add adapter", nport);
		goto err_node_put;
	}

	return 0;

err_node_put:
	of_node_put(rxport->remote_of_node);
err_free_rxport:
	priv->rxports[nport] = NULL;
	kfree(rxport);
	return ret;
}

static void ub960_rxport_remove_one(struct ub960_data *priv, u8 nport)
{
	struct ub960_rxport *rxport = priv->rxports[nport];

	i2c_atr_del_adapter(priv->atr, nport);
	ub960_rxport_remove_serializer(priv, nport);
	of_node_put(rxport->remote_of_node);
	kfree(rxport);
}

static int ub960_atr_probe(struct ub960_data *priv)
{
	struct i2c_adapter *parent_adap = priv->client->adapter;
	struct device *dev = &priv->client->dev;

	priv->atr = i2c_atr_new(parent_adap, dev, &ub960_atr_ops,
				priv->hw_data->num_rxports);
	if (IS_ERR(priv->atr))
		return PTR_ERR(priv->atr);

	i2c_atr_set_clientdata(priv->atr, priv);

	return 0;
}

static void ub960_atr_remove(struct ub960_data *priv)
{
	i2c_atr_delete(priv->atr);
	priv->atr = NULL;
}

static void ub960_rxport_handle_events(struct ub960_data *priv, u8 nport)
{
	struct device *dev = &priv->client->dev;
	u8 rx_port_sts1;
	u8 rx_port_sts2;
	u8 csi_rx_sts;
	u8 bcc_sts;
	int ret = 0;

	/* Read interrupts (also clears most of them) */
	if (!ret)
		ret = ub960_rxport_read(priv, nport, UB960_RR_RX_PORT_STS1,
					&rx_port_sts1);
	if (!ret)
		ret = ub960_rxport_read(priv, nport, UB960_RR_RX_PORT_STS2,
					&rx_port_sts2);
	if (!ret)
		ret = ub960_rxport_read(priv, nport, UB960_RR_CSI_RX_STS,
					&csi_rx_sts);
	if (!ret)
		ret = ub960_rxport_read(priv, nport, UB960_RR_BCC_STATUS,
					&bcc_sts);

	if (ret)
		return;

	dev_dbg(dev, "Handle RX%d events: STS: %x, %x, %x, BCC %x\n", nport,
		rx_port_sts1, rx_port_sts2, csi_rx_sts, bcc_sts);

	if (rx_port_sts1 & (UB960_RR_RX_PORT_STS1_BCC_CRC_ERROR |
			    UB960_RR_RX_PORT_STS1_BCC_SEQ_ERROR |
			    UB960_RR_RX_PORT_STS1_PARITY_ERROR))
		dev_err(dev, "RX%u STS1 error: %#02x\n", nport, rx_port_sts1);

	if (rx_port_sts2 & (UB960_RR_RX_PORT_STS2_FPD3_ENCODE_ERROR |
			    UB960_RR_RX_PORT_STS2_BUFFER_ERROR |
			    UB960_RR_RX_PORT_STS2_CSI_ERROR |
			    UB960_RR_RX_PORT_STS2_CABLE_FAULT))
		dev_err(dev, "RX%u STS2 error: %#02x\n", nport, rx_port_sts2);

	if (csi_rx_sts)
		dev_err(dev, "RX%u CSI error: %#02x\n", nport, csi_rx_sts);

	if (bcc_sts)
		dev_err(dev, "RX%u BCC error: %#02x\n", nport, bcc_sts);
}

/* -----------------------------------------------------------------------------
 * V4L2
 */

static int ub960_start_streaming(struct ub960_data *priv)
{
	const struct v4l2_subdev_krouting *routing;
	struct v4l2_subdev_state *state;
	unsigned int i;
	u8 nport;
	int ret;
	u32 csi_ctl;
	u32 speed_select;
	u32 fwd_ctl;
	struct {
		u32 num_streams;
		u8 pixel_dt;
		u8 meta_dt;
		u32 meta_lines;
		u32 tx_port;
	} rx_data[UB960_MAX_RX_NPORTS] = { 0 };

	ret = 0;

	state = v4l2_subdev_lock_active_state(&priv->sd);

	routing = &state->routing;

	for (i = 0; i < routing->num_routes; ++i) {
		struct v4l2_subdev_route *route = &routing->routes[i];
		u32 port = route->sink_pad;
		struct ub960_rxport *rxport = priv->rxports[port];
		struct v4l2_mbus_framefmt *fmt;
		const struct ub960_format_info *ub960_fmt;

		if (!rxport)
			continue;

		rx_data[port].tx_port =
			route->source_pad - priv->hw_data->num_rxports;

		/* For the rest, we are only interested in parallel busses */
		if (rxport->mode == RXPORT_MODE_CSI2)
			continue;

		rx_data[port].num_streams++;

		if (rx_data[port].num_streams > 2) {
			ret = -EPIPE;
			break;
		}

		fmt = v4l2_state_get_stream_format(state, port,
						   route->sink_stream);
		if (!fmt) {
			ret = -EPIPE;
			break;
		}

		ub960_fmt = ub960_find_format(fmt->code);
		if (!ub960_fmt) {
			ret = -EPIPE;
			break;
		}

		if (ub960_fmt->meta) {
			if (fmt->height > 3) {
				dev_err(&priv->client->dev,
					"Unsupported metadata height %u\n",
					fmt->height);
				ret = -EPIPE;
				break;
			}

			rx_data[port].meta_dt = ub960_fmt->datatype;
			rx_data[port].meta_lines = fmt->height;
		} else {
			rx_data[port].pixel_dt = ub960_fmt->datatype;
		}
	}

	v4l2_subdev_unlock_state(state);

	if (ret)
		return ret;

	switch (priv->tx_data_rate) {
	case 1600000000:
	default:
		speed_select = 0;
		break;
	case 1200000000:
		speed_select = 1;
		break;
	case 800000000:
		speed_select = 2;
		break;
	case 400000000:
		speed_select = 3;
		break;
	}

	ub960_write(priv, UB960_SR_CSI_PLL_CTL, speed_select);

	for (nport = 0; nport < priv->hw_data->num_txports; nport++) {
		struct ub960_txport *txport = priv->txports[nport];

		if (!txport)
			continue;

		csi_ctl = UB960_TR_CSI_CTL_CSI_ENABLE;

		/*
		 * From the datasheet: "initial CSI Skew-Calibration
		 * sequence [...] should be set when operating at 1.6 Gbps"
		 */
		if (speed_select == 0)
			csi_ctl |= UB960_TR_CSI_CTL_CSI_CAL_EN;

		csi_ctl |= (4 - txport->num_data_lanes) << 4;

		ub960_csiport_write(priv, nport, UB960_TR_CSI_CTL, csi_ctl);
	}

	for (nport = 0; nport < priv->hw_data->num_rxports; ++nport) {
		struct ub960_rxport *rxport = priv->rxports[nport];

		if (!rxport || !rxport->locked)
			continue;

		switch (rxport->mode) {
		default:
			WARN_ON(true);
			fallthrough;

		case RXPORT_MODE_RAW10:
			/* VC=nport */
			ub960_rxport_write(priv, nport, UB960_RR_RAW10_ID,
					   rx_data[nport].pixel_dt |
						   (nport << 6));

			ub960_rxport_write(priv, rxport->nport,
					   UB960_RR_RAW_EMBED_DTYPE,
					   (rx_data[nport].meta_lines << 6) |
						   rx_data[nport].meta_dt);

			break;

		case RXPORT_MODE_CSI2:
			if (priv->vc_map.port_en[nport]) {
				/* Map VCs from this port */
				ub960_rxport_write(priv, nport, UB960_RR_CSI_VC_MAP,
						   priv->vc_map.vc_map[nport]);
			} else {
				/* Disable port */
				ub960_update_bits_shared(priv, UB960_SR_RX_PORT_CTL,
							 BIT(nport), 0);
			}

			break;
		}
	}

	/* Start all cameras */

	priv->streaming = true;

	for (nport = 0; nport < priv->hw_data->num_rxports; ++nport) {
		struct ub960_rxport *rxport = priv->rxports[nport];

		if (!rxport || !rxport->locked)
			continue;

		ret = v4l2_subdev_call(rxport->sd, video, s_stream, 1);
		if (ret) {
			for (; nport > 0; --nport) {
				rxport = priv->rxports[nport - 1];
				if (!rxport)
					continue;

				v4l2_subdev_call(rxport->sd, video, s_stream,
						 0);
			}

			priv->streaming = false;

			return ret;
		}
	}

	/* Forwarding */

	fwd_ctl = 0;

	for (nport = 0; nport < priv->hw_data->num_rxports; ++nport) {
		struct ub960_rxport *rxport = priv->rxports[nport];

		if (!rxport || !rxport->locked) {
			fwd_ctl |= BIT(4 + nport); /* forward disable */
			continue;
		}

		if (rx_data[nport].tx_port == 1)
			fwd_ctl |= BIT(nport); /* forward to TX1 */
	}

	ub960_write(priv, UB960_SR_FWD_CTL1, fwd_ctl);

	return 0;
}

static int ub960_stop_streaming(struct ub960_data *priv)
{
	unsigned int i;

	/* Disable forwarding */
	ub960_write(priv, UB960_SR_FWD_CTL1,
		    (BIT(0) | BIT(1) | BIT(2) | BIT(3)) << 4);

	/* Stop all cameras */
	for (i = 0; i < priv->hw_data->num_rxports; ++i) {
		struct ub960_rxport *rxport = priv->rxports[i];

		if (!rxport || !rxport->locked)
			continue;

		v4l2_subdev_call(rxport->sd, video, s_stream, 0);
	}

	for (i = 0; i < priv->hw_data->num_txports; i++) {
		struct ub960_txport *txport = priv->txports[i];

		if (!txport)
			continue;

		ub960_csiport_write(priv, i, UB960_TR_CSI_CTL, 0);
	}

	priv->streaming = false;

	return 0;
}

static int ub960_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ub960_data *priv = sd_to_ub960(sd);

	if (enable)
		return ub960_start_streaming(priv);
	else
		return ub960_stop_streaming(priv);
}

static const struct v4l2_subdev_video_ops ub960_video_ops = {
	.s_stream = ub960_s_stream,
};

static int _ub960_set_routing(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     struct v4l2_subdev_krouting *routing)
{
	const struct v4l2_mbus_framefmt format = {
		.width = 640,
		.height = 480,
		.code = MEDIA_BUS_FMT_UYVY8_2X8,
		.field = V4L2_FIELD_NONE,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.ycbcr_enc = V4L2_YCBCR_ENC_601,
		.quantization = V4L2_QUANTIZATION_LIM_RANGE,
		.xfer_func = V4L2_XFER_FUNC_SRGB,
	};
	int ret;

	/*
	 * Note: we can only support up to V4L2_FRAME_DESC_ENTRY_MAX, until
	 * frame desc is made dynamically allocated.
	 */

	if (routing->num_routes > V4L2_FRAME_DESC_ENTRY_MAX)
		return -EINVAL;

	ret = v4l2_routing_simple_verify(routing);
	if (ret)
		return ret;

	v4l2_subdev_lock_state(state);

	ret = v4l2_subdev_set_routing_with_fmt(sd, state, routing, &format);

	v4l2_subdev_unlock_state(state);

	if (ret)
		return ret;

	return 0;
}

static int ub960_set_routing(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     enum v4l2_subdev_format_whence which,
			     struct v4l2_subdev_krouting *routing)
{
	struct ub960_data *priv = sd_to_ub960(sd);

	if (which == V4L2_SUBDEV_FORMAT_ACTIVE && priv->streaming)
		return -EBUSY;

	return _ub960_set_routing(sd, state, routing);
}

static int ub960_get_source_frame_desc(struct ub960_data *priv,
				       struct v4l2_mbus_frame_desc *desc,
				       u8 nport)
{
	struct v4l2_subdev *sd;
	struct media_pad *pad;
	int ret;

	pad = media_entity_remote_pad(&priv->pads[nport]);
	if (!pad)
		return -EPIPE;

	sd = priv->rxports[nport]->sd;

	ret = v4l2_subdev_call(sd, pad, get_frame_desc, pad->index,
			       desc);
	if (ret)
		return ret;

	return 0;
}

static inline u8 ub960_get_output_vc(u8 map, u8 input_vc) {
	return (map >> (2 * input_vc)) & 0x03;
}

static void ub960_map_virtual_channels(struct ub960_data *priv)
{
	struct device *dev = &priv->client->dev;
	struct ub960_vc_map vc_map = {.vc_map = {0x00}, .port_en = {false}};
	u8 nport, available_vc = 0;

	for (nport = 0; nport < priv->hw_data->num_rxports; ++nport) {
		struct v4l2_mbus_frame_desc source_fd;
		bool used_vc[UB960_MAX_VC] = {false};
		u8 vc, cur_vc = available_vc;
		int j, ret;
		u8 map;

		ret = ub960_get_source_frame_desc(priv, &source_fd, nport);
		/* Mark channels used in source in used_vc[] */
		if (!ret) {
			for (j = 0; j < source_fd.num_entries; ++j) {
				u8 source_vc = source_fd.entry[j].bus.csi2.vc;
				if (source_vc < UB960_MAX_VC) {
					used_vc[source_vc] = true;
				}
			}
		} else if (ret == -ENOIOCTLCMD) {
			/* assume VC=0 is used if sensor driver doesn't provide info */
			used_vc[0] = true;
		} else {
			continue;
		}

		/* Start with all channels mapped to first free output */
		map = (cur_vc << 6) | (cur_vc << 4) | (cur_vc << 2) |
			(cur_vc << 0);

		/* Map actually used to channels to distinct free outputs */
		for (vc = 0; vc < UB960_MAX_VC; ++vc) {
			if (used_vc[vc]) {
				map &= ~(0x03 << (2*vc));
				map |= (cur_vc << (2*vc));
				++cur_vc;
			}
		}

		/* Don't enable port if we ran out of available channels */
		if (cur_vc > UB960_MAX_VC) {
			dev_err(dev,
				"No VCs available, RX ports %d will be disabled\n",
				nport);
			continue;
		}

		/* Enable port and update map */
		vc_map.vc_map[nport] = map;
		vc_map.port_en[nport] = true;
		available_vc = cur_vc;
		dev_dbg(dev, "%s: VC map for port %d is 0x%02x",
			__func__, nport, map);
	}
	priv->vc_map = vc_map;
}

static int ub960_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				struct v4l2_mbus_frame_desc *fd)
{
	struct ub960_data *priv = sd_to_ub960(sd);
	const struct v4l2_subdev_krouting *routing;
	struct v4l2_subdev_state *state;
	int ret = 0;
	unsigned int i;
	struct device *dev = &priv->client->dev;

	dev_dbg(dev, "%s for pad %d\n", __func__, pad);

	if (!ub960_pad_is_source(priv, pad))
		return -EINVAL;

	state = v4l2_subdev_lock_active_state(&priv->sd);

	routing = &state->routing;

	memset(fd, 0, sizeof(*fd));

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;

	ub960_map_virtual_channels(priv);

	for (i = 0; i < routing->num_routes; ++i) {
		const struct v4l2_subdev_route *route = &routing->routes[i];
		struct v4l2_mbus_frame_desc_entry *source_entry = NULL;
		struct v4l2_mbus_frame_desc source_fd;
		unsigned int j;

		if (!(route->flags & V4L2_SUBDEV_ROUTE_FL_ACTIVE))
			continue;

		if (route->source_pad != pad)
			continue;

		ret = ub960_get_source_frame_desc(priv, &source_fd,
						  route->sink_pad);
		if (ret) {
			dev_err(dev,
				"Failed to get source frame desc for port %u\n",
				route->sink_pad);
			goto out;
		}

		for (j = 0; j < source_fd.num_entries; ++j)
			if (source_fd.entry[j].stream == route->sink_stream) {
				source_entry = &source_fd.entry[j];
				break;
			}

		if (!source_entry) {
			dev_err(dev,
				"Failed to find stream from source frame desc\n");
			ret = -EPIPE;
			goto out;
		}

		fd->entry[fd->num_entries].stream = route->source_stream;

		fd->entry[fd->num_entries].flags =
			V4L2_MBUS_FRAME_DESC_FL_LEN_MAX;
		fd->entry[fd->num_entries].length = source_entry->length;
		fd->entry[fd->num_entries].pixelcode =
			source_entry->pixelcode;

		fd->entry[fd->num_entries].bus.csi2.vc =
			ub960_get_output_vc(priv->vc_map.vc_map[route->sink_pad],
					    source_entry->bus.csi2.vc);
		dev_dbg(dev, "Mapping sink %d/%d to output VC %d",
			route->sink_pad, route->sink_stream,
			fd->entry[fd->num_entries].bus.csi2.vc);

		if (source_fd.type == V4L2_MBUS_FRAME_DESC_TYPE_CSI2) {
			fd->entry[fd->num_entries].bus.csi2.dt =
				source_entry->bus.csi2.dt;
		} else {
			const struct ub960_format_info *ub960_fmt;
			struct v4l2_mbus_framefmt *fmt;

			fmt = v4l2_state_get_stream_format(
				state, pad, route->source_stream);

			if (!fmt) {
				ret = -EINVAL;
				goto out;
			}

			ub960_fmt = ub960_find_format(fmt->code);
			if (!ub960_fmt) {
				dev_err(dev, "Unable to find format\n");
				ret = -EINVAL;
				goto out;
			}

			fd->entry[fd->num_entries].bus.csi2.dt =
				ub960_fmt->datatype;
		}

		fd->num_entries++;
	}

out:
	v4l2_subdev_unlock_state(state);

	return ret;
}

static int ub960_set_fmt(struct v4l2_subdev *sd,
			 struct v4l2_subdev_state *state,
			 struct v4l2_subdev_format *format)
{
	struct ub960_data *priv = sd_to_ub960(sd);
	struct v4l2_mbus_framefmt *fmt;
	int ret = 0;

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE && priv->streaming)
		return -EBUSY;

	/* No transcoding, source and sink formats must match. */
	if (ub960_pad_is_source(priv, format->pad))
		return v4l2_subdev_get_fmt(sd, state, format);

	/* TODO: implement fmt validation */

	v4l2_subdev_lock_state(state);

	fmt = v4l2_state_get_stream_format(state, format->pad, format->stream);
	if (!fmt) {
		ret = -EINVAL;
		goto out;
	}

	*fmt = format->format;

	fmt = v4l2_state_get_opposite_stream_format(state, format->pad,
						    format->stream);
	if (!fmt) {
		ret = -EINVAL;
		goto out;
	}

	*fmt = format->format;

out:
	v4l2_subdev_unlock_state(state);

	return ret;
}

static int ub960_init_cfg(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state)
{
	struct ub960_data *priv = sd_to_ub960(sd);

	struct v4l2_subdev_route routes[] = {
		{
			.sink_pad = 0,
			.sink_stream = 0,
			.source_pad = priv->hw_data->num_rxports,
			.source_stream = 0,
			.flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE,
		},
	};

	struct v4l2_subdev_krouting routing = {
		.num_routes = ARRAY_SIZE(routes),
		.routes = routes,
	};

	return _ub960_set_routing(sd, state, &routing);
}

static const struct v4l2_subdev_pad_ops ub960_pad_ops = {
	.set_routing	= ub960_set_routing,
	.get_frame_desc	= ub960_get_frame_desc,

	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = ub960_set_fmt,

	.init_cfg = ub960_init_cfg,
};

static const struct v4l2_subdev_core_ops ub960_subdev_core_ops = {
	.log_status		= v4l2_ctrl_subdev_log_status,
	.subscribe_event	= v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event	= v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_ops ub960_subdev_ops = {
	.core		= &ub960_subdev_core_ops,
	.video		= &ub960_video_ops,
	.pad		= &ub960_pad_ops,
};

static const struct media_entity_operations ub960_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
	.has_route = v4l2_subdev_has_route
};

static void ub960_enable_tpg(struct ub960_data *priv, int tpg_num)
{
	/*
	 * Note: no need to write UB960_REG_IND_ACC_CTL: the only indirect
	 * registers target we use is "CSI-2 Pattern Generator & Timing
	 * Registers", which is the default
	 */

	/*
	 * TPG can only provide a single stream per CSI TX port. If
	 * multiple streams are currently enabled, only the first
	 * one will use the TPG, other streams will be halted.
	 */

	struct v4l2_mbus_framefmt *fmt;
	u8 vbp, vfp;
	u16 blank_lines;
	u16 width;
	u16 height;

	u16 bytespp = 2; /* For MEDIA_BUS_FMT_UYVY8_1X16 */
	u8 cbars_idx = tpg_num - TEST_PATTERN_V_COLOR_BARS_1;
	u8 num_cbars = 1 << cbars_idx;

	u16 line_size;	/* Line size [bytes] */
	u16 bar_size;	/* cbar size [bytes] */
	u16 act_lpf;	/* active lines/frame */
	u16 tot_lpf;	/* tot lines/frame */
	u16 line_pd;	/* Line period in 10-ns units */

	struct v4l2_subdev_state *state;

	state = v4l2_subdev_lock_active_state(&priv->sd);

	vbp = 33;
	vfp = 10;
	blank_lines = vbp + vfp + 2; /* total blanking lines */

	fmt = v4l2_state_get_stream_format(state, 4, 0);

	width = fmt->width;
	height = fmt->height;

	line_size = width * bytespp;
	bar_size = line_size / num_cbars;
	act_lpf = height;
	tot_lpf = act_lpf + blank_lines;
	line_pd = 100000000 / 60 / tot_lpf;

	/* Disable forwarding from FPD-3 RX ports */
	ub960_write(priv, UB960_SR_FWD_CTL1,
		    UB960_SR_FWD_CTL1_PORT_DIS(0) |
			    UB960_SR_FWD_CTL1_PORT_DIS(1));

	/* Access Indirect Pattern Gen */
	ub960_write(priv, UB960_SR_IND_ACC_CTL,
		    UB960_SR_IND_ACC_CTL_IA_AUTO_INC | 0);

	ub960_write_ind8(priv, UB960_IR_PGEN_CTL,
			 UB960_IR_PGEN_CTL_PGEN_ENABLE);

	/* YUV422 8bit: 2 bytes/block, CSI-2 data type 0x1e */
	ub960_write_ind8(priv, UB960_IR_PGEN_CFG, cbars_idx << 4 | 0x2);
	ub960_write_ind8(priv, UB960_IR_PGEN_CSI_DI, 0x1e);

	ub960_write_ind16(priv, UB960_IR_PGEN_LINE_SIZE1, line_size);
	ub960_write_ind16(priv, UB960_IR_PGEN_BAR_SIZE1, bar_size);
	ub960_write_ind16(priv, UB960_IR_PGEN_ACT_LPF1, act_lpf);
	ub960_write_ind16(priv, UB960_IR_PGEN_TOT_LPF1, tot_lpf);
	ub960_write_ind16(priv, UB960_IR_PGEN_LINE_PD1, line_pd);
	ub960_write_ind8(priv, UB960_IR_PGEN_VBP, vbp);
	ub960_write_ind8(priv, UB960_IR_PGEN_VFP, vfp);

	v4l2_subdev_unlock_state(state);
}

static void ub960_disable_tpg(struct ub960_data *priv)
{
	/* TPG off, enable forwarding from FPD-3 RX ports */
	ub960_write(priv, UB960_SR_FWD_CTL1, 0x00);

	ub960_write_ind8(priv, UB960_IR_PGEN_CTL, 0x00);
}

static int ub960_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ub960_data *priv =
		container_of(ctrl->handler, struct ub960_data, ctrl_handler);

	switch (ctrl->id) {
	case V4L2_CID_TEST_PATTERN:
		if (ctrl->val == 0)
			ub960_disable_tpg(priv);
		else
			ub960_enable_tpg(priv, ctrl->val);
		break;
	}

	return 0;
}

static const struct v4l2_ctrl_ops ub960_ctrl_ops = {
	.s_ctrl = ub960_s_ctrl,
};

/* -----------------------------------------------------------------------------
 * Core
 */

static irqreturn_t ub960_handle_events(int irq, void *arg)
{
	struct ub960_data *priv = arg;
	unsigned int i;
	u8 int_sts;
	int ret;

	ret = ub960_read(priv, UB960_SR_INTERRUPT_STS, &int_sts);

	if (!ret && int_sts) {
		u8 fwd_sts;

		dev_dbg(&priv->client->dev, "INTERRUPT_STS %x\n", int_sts);

		ub960_read(priv, UB960_SR_FWD_STS, &fwd_sts);

		dev_dbg(&priv->client->dev, "FWD_STS %#x\n", fwd_sts);

		for (i = 0; i < priv->hw_data->num_txports; ++i) {
			if (int_sts & UB960_SR_INTERRUPT_STS_IS_CSI_TX(i))
				ub960_csi_handle_events(priv, i);
		}

		for (i = 0; i < priv->hw_data->num_rxports; i++) {
			if (!priv->rxports[i] || !priv->rxports[i]->locked)
				continue;

			if (int_sts & UB960_SR_INTERRUPT_STS_IS_RX(i))
				ub960_rxport_handle_events(priv, i);
		}
	}

	return IRQ_HANDLED;
}

static int ub960_run(void *arg)
{
	struct ub960_data *priv = arg;

	while (!kthread_should_stop()) {
		ub960_handle_events(0, priv);

		msleep(500);
	}

	return 0;
}

static void ub960_remove_ports(struct ub960_data *priv)
{
	unsigned int i;

	for (i = 0; i < priv->hw_data->num_rxports; i++)
		if (priv->rxports[i])
			ub960_rxport_remove_one(priv, i);

	for (i = 0; i < priv->hw_data->num_txports; i++)
		if (priv->txports[i])
			ub960_txport_remove_one(priv, i);
}

static int ub960_register_clocks(struct ub960_data *priv)
{
	struct device *dev = &priv->client->dev;
	const char *name;
	int err;

	/* Get our input clock (REFCLK, 23..26 MHz) */

	priv->refclk = devm_clk_get(dev, NULL);
	if (IS_ERR(priv->refclk))
		return dev_err_probe(dev, PTR_ERR(priv->refclk), "Cannot get REFCLK");

	dev_dbg(dev, "REFCLK: %lu Hz\n", clk_get_rate(priv->refclk));

	/* Provide FPD-Link III line rate (160 * REFCLK in Synchronous mode) */

	name = kasprintf(GFP_KERNEL, "%s.fpd_line_rate", dev_name(dev));
	priv->line_clk_hw =
		clk_hw_register_fixed_factor(dev, name,
					     __clk_get_name(priv->refclk),
					     0, 160, 1);
	kfree(name);
	if (IS_ERR(priv->line_clk_hw))
		return dev_err_probe(dev, PTR_ERR(priv->line_clk_hw),
				     "Cannot register clock HW\n");

	dev_dbg(dev, "line rate: %lu Hz\n", clk_hw_get_rate(priv->line_clk_hw));

	/* Expose the line rate to OF */

	err = devm_of_clk_add_hw_provider(dev, of_clk_hw_simple_get, priv->line_clk_hw);
	if (err) {
		clk_hw_unregister_fixed_factor(priv->line_clk_hw);
		return dev_err_probe(dev, err, "Cannot add OF clock provider\n");
	}

	return 0;
}

static void ub960_unregister_clocks(struct ub960_data *priv)
{
	clk_hw_unregister_fixed_factor(priv->line_clk_hw);
}

static int ub960_parse_dt(struct ub960_data *priv)
{
	struct device_node *np = priv->client->dev.of_node;
	struct device *dev = &priv->client->dev;
	int ret = 0;
	int n;

	if (!np) {
		dev_err(dev, "OF: no device tree node!\n");
		return -ENOENT;
	}

	n = of_property_read_variable_u16_array(np, "i2c-alias-pool",
						priv->atr_alias_id,
						2, UB960_MAX_POOL_ALIASES);
	if (n < 0)
		dev_warn(dev,
			 "OF: no i2c-alias-pool, can't access remote I2C slaves");

	priv->atr_alias_num = n;

	dev_dbg(dev, "i2c-alias-pool has %zu aliases", priv->atr_alias_num);

	if (of_property_read_u32(np, "data-rate", &priv->tx_data_rate) != 0) {
		dev_err(dev, "OF: %s: missing \"data-rate\" node\n",
			of_node_full_name(np));
		return -EINVAL;
	}

	if (priv->tx_data_rate != 1600000000 &&
	    priv->tx_data_rate != 1200000000 &&
	    priv->tx_data_rate != 800000000 &&
	    priv->tx_data_rate != 400000000) {
		dev_err(dev, "OF: %s: invalid \"data-rate\" node\n",
			of_node_full_name(np));
		return -EINVAL;
	}

	priv->tx_link_freq[0] = priv->tx_data_rate / 2;

	dev_dbg(dev, "Nominal data rate: %u", priv->tx_data_rate);

	for (n = 0; n < priv->hw_data->num_rxports + priv->hw_data->num_txports; ++n) {
		struct device_node *ep_np;

		ep_np = of_graph_get_endpoint_by_regs(np, n, 0);
		if (!ep_np)
			continue;

		if (n < priv->hw_data->num_rxports)
			ret = ub960_rxport_probe_one(priv, ep_np, n);
		else
			ret = ub960_csiport_probe_one(
				priv, ep_np, n - priv->hw_data->num_rxports);

		of_node_put(ep_np);

		if (ret)
			break;
	}

	if (ret)
		ub960_remove_ports(priv);

	return ret;
}

static int ub960_notify_bound(struct v4l2_async_notifier *notifier,
			      struct v4l2_subdev *subdev,
			      struct v4l2_async_subdev *asd)
{
	struct ub960_data *priv = sd_to_ub960(notifier->sd);
	struct ub960_rxport *rxport = to_ub960_asd(asd)->rxport;
	struct device *dev = &priv->client->dev;
	u8 nport = rxport->nport;
	unsigned int src_pad;
	unsigned int i;
	int ret;

	dev_dbg(dev, "Bind %s\n", subdev->name);

	ret = media_entity_get_fwnode_pad(&subdev->entity, rxport->fwnode,
					  MEDIA_PAD_FL_SOURCE);
	if (ret < 0) {
		dev_err(dev, "Failed to find pad for %s\n", subdev->name);
		return ret;
	}

	rxport->sd = subdev;
	src_pad = ret;

	ret = media_create_pad_link(&rxport->sd->entity, src_pad,
				    &priv->sd.entity, nport,
				    MEDIA_LNK_FL_ENABLED |
				    MEDIA_LNK_FL_IMMUTABLE);
	if (ret) {
		dev_err(dev, "Unable to link %s:%u -> %s:%u\n",
			rxport->sd->name, src_pad, priv->sd.name, nport);
		return ret;
	}

	dev_dbg(dev, "Bound %s pad: %u on index %u\n", subdev->name, src_pad,
		nport);

	for (i = 0; i < priv->hw_data->num_rxports; ++i) {
		if (priv->rxports[i] && rxport->locked && !priv->rxports[i]->sd) {
			dev_dbg(dev, "Waiting for more subdevs to be bound\n");
			return 0;
		}
	}

	dev_dbg(dev, "All subdevs bound\n");

	return 0;
}

static void ub960_notify_unbind(struct v4l2_async_notifier *notifier,
				struct v4l2_subdev *subdev,
				struct v4l2_async_subdev *asd)
{
	struct ub960_data *priv = sd_to_ub960(notifier->sd);
	struct ub960_rxport *rxport = to_ub960_asd(asd)->rxport;
	struct device *dev = &priv->client->dev;

	dev_dbg(dev, "Unbind %s\n", subdev->name);

	rxport->sd = NULL;
}

static const struct v4l2_async_notifier_operations ub960_notify_ops = {
	.bound = ub960_notify_bound,
	.unbind = ub960_notify_unbind,
};

static int ub960_v4l2_notifier_register(struct ub960_data *priv)
{
	struct device *dev = &priv->client->dev;
	unsigned int i;
	int ret;

	v4l2_async_notifier_init(&priv->notifier);

	for (i = 0; i < priv->hw_data->num_rxports; ++i) {
		struct ub960_rxport *rxport = priv->rxports[i];
		struct v4l2_async_subdev *asd;
		struct ub960_asd *ubasd;

		if (!rxport || !rxport->locked)
			continue;

		asd = v4l2_async_notifier_add_fwnode_subdev(&priv->notifier,
							    rxport->fwnode,
							    sizeof(*ubasd));
		if (IS_ERR(asd)) {
			dev_err(dev, "Failed to add subdev for source %u: %ld",
				i, PTR_ERR(asd));
			v4l2_async_notifier_cleanup(&priv->notifier);
			return PTR_ERR(asd);
		}

		ubasd = to_ub960_asd(asd);
		ubasd->rxport = rxport;
	}

	priv->notifier.ops = &ub960_notify_ops;

	ret = v4l2_async_subdev_notifier_register(&priv->sd, &priv->notifier);
	if (ret) {
		dev_err(dev, "Failed to register subdev_notifier");
		v4l2_async_notifier_cleanup(&priv->notifier);
		return ret;
	}

	return 0;
}

static void ub960_v4l2_notifier_unregister(struct ub960_data *priv)
{
	struct device *dev = &priv->client->dev;

	dev_dbg(dev, "Unregister async notif\n");

	v4l2_async_notifier_unregister(&priv->notifier);
	v4l2_async_notifier_cleanup(&priv->notifier);
}

static int ub960_create_subdev(struct ub960_data *priv)
{
	struct device *dev = &priv->client->dev;
	unsigned int i;
	int ret;

	v4l2_i2c_subdev_init(&priv->sd, priv->client, &ub960_subdev_ops);
	v4l2_ctrl_handler_init(&priv->ctrl_handler,
			       ARRAY_SIZE(ub960_tpg_qmenu) - 1);
	priv->sd.ctrl_handler = &priv->ctrl_handler;

	v4l2_ctrl_new_std_menu_items(&priv->ctrl_handler, &ub960_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(ub960_tpg_qmenu) - 1, 0, 0,
				     ub960_tpg_qmenu);

	v4l2_ctrl_new_int_menu(&priv->ctrl_handler, NULL, V4L2_CID_LINK_FREQ,
			       ARRAY_SIZE(priv->tx_link_freq) - 1, 0,
			       priv->tx_link_freq);

	if (priv->ctrl_handler.error) {
		ret = priv->ctrl_handler.error;
		goto err_free_ctrl;
	}

	priv->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS |
		V4L2_SUBDEV_FL_MULTIPLEXED;
	priv->sd.entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	priv->sd.entity.ops = &ub960_entity_ops;

	for (i = 0; i < priv->hw_data->num_rxports + priv->hw_data->num_txports; i++) {
		priv->pads[i].flags = ub960_pad_is_sink(priv, i) ?
			MEDIA_PAD_FL_SINK : MEDIA_PAD_FL_SOURCE;
	}

	ret = media_entity_pads_init(&priv->sd.entity,
				     priv->hw_data->num_rxports +
					     priv->hw_data->num_txports,
				     priv->pads);
	if (ret)
		goto err_free_ctrl;

	ret = v4l2_subdev_init_finalize(&priv->sd);
	if (ret)
		goto err_entity_cleanup;

	ret = ub960_v4l2_notifier_register(priv);
	if (ret) {
		dev_err(dev, "v4l2 subdev notifier register failed: %d\n", ret);
		goto err_free_state;
	}

	ret = v4l2_async_register_subdev(&priv->sd);
	if (ret) {
		dev_err(dev, "v4l2_async_register_subdev error: %d\n", ret);
		goto err_unreg_notif;
	}

	return 0;

err_unreg_notif:
	ub960_v4l2_notifier_unregister(priv);
err_free_state:
	v4l2_subdev_cleanup(&priv->sd);
err_entity_cleanup:
	media_entity_cleanup(&priv->sd.entity);
err_free_ctrl:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);

	return ret;
}

static void ub960_destroy_subdev(struct ub960_data *priv)
{
	ub960_v4l2_notifier_unregister(priv);
	v4l2_async_unregister_subdev(&priv->sd);

	v4l2_subdev_cleanup(&priv->sd);

	media_entity_cleanup(&priv->sd.entity);
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
}

static const struct regmap_config ub960_regmap_config = {
	.name = "ds90ub960",

	.reg_bits = 8,
	.val_bits = 8,

	.max_register = 0xff,
};

static void ub960_sw_reset(struct ub960_data *priv)
{
	unsigned int i;

	ub960_write(priv, UB960_SR_RESET, BIT(1));

	for (i = 0; i < 10; ++i) {
		int ret;
		u8 v;

		ret = ub960_read(priv, UB960_SR_RESET, &v);

		if (ret || v == 0)
			break;
	}
}

static int ub960_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ub960_data *priv;
	unsigned int nport;
	u8 rev_mask;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->client = client;

	priv->hw_data = of_device_get_match_data(dev);
	if (!priv->hw_data)
		return -ENODEV;

	mutex_init(&priv->alias_table_lock);

	priv->regmap = devm_regmap_init_i2c(client, &ub960_regmap_config);
	if (IS_ERR(priv->regmap))
		return PTR_ERR(priv->regmap);

	/* get power-down pin from DT */
	priv->pd_gpio = devm_gpiod_get_optional(dev, "powerdown", GPIOD_OUT_HIGH);
	if (IS_ERR(priv->pd_gpio)) {
		ret = PTR_ERR(priv->pd_gpio);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Cannot get powerdown GPIO (%d)", ret);
		return ret;
	}

	if (priv->pd_gpio) {
		gpiod_set_value_cansleep(priv->pd_gpio, 1);
		/* wait min 2 ms for reset to complete */
		usleep_range(2000, 5000);
		gpiod_set_value_cansleep(priv->pd_gpio, 0);
		/* wait min 2 ms for power up to finish */
		usleep_range(2000, 5000);
	} else {
		/* Use SW reset if we don't have PD gpio */
		ub960_sw_reset(priv);
	}

	ret = ub960_register_clocks(priv);
	if (ret)
		return ret;

	/* Runtime check register accessibility */
	ret = ub960_read(priv, UB960_SR_REV_MASK, &rev_mask);
	if (ret) {
		dev_err(dev, "Cannot read first register (%d), abort\n", ret);
		goto err_reg_read;
	}

	ret = ub960_atr_probe(priv);
	if (ret)
		goto err_atr_probe;

	ret = ub960_parse_dt(priv);
	if (ret)
		goto err_parse_dt;

	ret = ub960_rxport_probe_serializers(priv);
	if (ret)
		goto err_parse_dt;

	/*
	 * Clear any errors caused by switching the RX port settings while
	 * probing.
	 */
	for (nport = 0; nport < priv->hw_data->num_rxports; ++nport) {
		u8 dummy;

		ub960_rxport_read(priv, nport, UB960_RR_RX_PORT_STS1, &dummy);
		ub960_rxport_read(priv, nport, UB960_RR_RX_PORT_STS2, &dummy);
		ub960_rxport_read(priv, nport, UB960_RR_CSI_RX_STS, &dummy);
		ub960_rxport_read(priv, nport, UB960_RR_BCC_STATUS, &dummy);
	}

	ret = ub960_create_subdev(priv);
	if (ret)
		goto err_subdev;

	if (client->irq) {
		dev_dbg(dev, "using IRQ %d\n", client->irq);

		ret = devm_request_threaded_irq(dev, client->irq, NULL,
						ub960_handle_events,
						IRQF_ONESHOT, client->name,
						priv);
		if (ret) {
			dev_err(dev, "Cannot enable IRQ (%d)\n", ret);
			goto err_irq;
		}

		/* Disable GPIO3 as input */
		ub960_update_bits_shared(priv, UB960_SR_GPIO_INPUT_CTL, BIT(3),
					 0);
		/* Enable GPIO3 as output, active low interrupt */
		ub960_write(priv, UB960_SR_GPIO_PIN_CTL(3), 0xd1);

		ub960_write(priv, UB960_SR_INTERRUPT_CTL,
			    UB960_SR_INTERRUPT_CTL_ALL);
	} else {
		/* No IRQ, fallback to polling */

		priv->kthread = kthread_run(ub960_run, priv, dev_name(dev));
		if (IS_ERR(priv->kthread)) {
			ret = PTR_ERR(priv->kthread);
			dev_err(dev, "Cannot create kthread (%d)\n", ret);
			goto err_kthread;
		}
		dev_dbg(dev, "using polling mode\n");
	}

	dev_info(dev, "Successfully probed (rev/mask %02x)\n", rev_mask);

	return 0;

err_kthread:
err_irq:
	ub960_destroy_subdev(priv);
err_subdev:
	ub960_remove_ports(priv);
err_parse_dt:
	ub960_atr_remove(priv);
err_atr_probe:
err_reg_read:
	ub960_unregister_clocks(priv);
	if (priv->pd_gpio)
		gpiod_set_value_cansleep(priv->pd_gpio, 1);
	mutex_destroy(&priv->alias_table_lock);
	return ret;
}

static int ub960_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ub960_data *priv = sd_to_ub960(sd);

	dev_dbg(&client->dev, "Removing\n");

	if (priv->kthread)
		kthread_stop(priv->kthread);
	ub960_destroy_subdev(priv);
	ub960_remove_ports(priv);
	ub960_atr_remove(priv);
	ub960_unregister_clocks(priv);
	if (priv->pd_gpio)
		gpiod_set_value_cansleep(priv->pd_gpio, 1);
	mutex_destroy(&priv->alias_table_lock);

	dev_dbg(&client->dev, "Remove done\n");

	return 0;
}

static const struct ub960_hw_data ds90ub960_hw = {
	.num_rxports = 4,
	.num_txports = 2,
};

static const struct i2c_device_id ub960_id[] = {
	{ "ds90ub960-q1", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ub960_id);

#ifdef CONFIG_OF
static const struct of_device_id ub960_dt_ids[] = {
	{ .compatible = "ti,ds90ub960-q1", .data = &ds90ub960_hw },
	{ }
};
MODULE_DEVICE_TABLE(of, ub960_dt_ids);
#endif

static struct i2c_driver ds90ub960_driver = {
	.probe_new	= ub960_probe,
	.remove		= ub960_remove,
	.id_table	= ub960_id,
	.driver = {
		.name	= "ds90ub960",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ub960_dt_ids),
	},
};

module_i2c_driver(ds90ub960_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Texas Instruments DS90UB960-Q1 FPDLink-3 deserializer driver");
MODULE_AUTHOR("Luca Ceresoli <luca@lucaceresoli.net>");
MODULE_AUTHOR("Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>");
