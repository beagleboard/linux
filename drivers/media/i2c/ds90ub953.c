// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the Texas Instruments DS90UB953 video serializer
 *
 * Based on a driver from Luca Ceresoli <luca@lucaceresoli.net>
 *
 * Copyright (c) 2019 Luca Ceresoli <luca@lucaceresoli.net>
 * Copyright (c) 2021 Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/gpio/driver.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/rational.h>
#include <linux/regmap.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>

#define UB953_PAD_SINK			0
#define UB953_PAD_SOURCE		1

#define UB953_NUM_GPIOS			4
#define UB953_MAX_DATA_LANES		4

#define UB953_REG_RESET_CTL			0x01
#define UB953_REG_RESET_CTL_DIGITAL_RESET_1	BIT(1)
#define UB953_REG_RESET_CTL_DIGITAL_RESET_0	BIT(0)

#define UB953_REG_GENERAL_CFG			0x02
#define UB953_REG_MODE_SEL			0x03

#define UB953_REG_CLKOUT_CTRL0			0x06
#define UB953_REG_CLKOUT_CTRL1			0x07

#define UB953_REG_SCL_HIGH_TIME			0x0B
#define UB953_REG_SCL_LOW_TIME			0x0C

#define UB953_REG_LOCAL_GPIO_DATA		0x0d
#define UB953_REG_LOCAL_GPIO_DATA_GPIO_RMTEN(n)		BIT(4 + (n))
#define UB953_REG_LOCAL_GPIO_DATA_GPIO_OUT_SRC(n)	BIT(0 + (n))

#define UB953_REG_GPIO_INPUT_CTRL		0x0e
#define UB953_REG_GPIO_INPUT_CTRL_OUT_EN(n)	BIT(4 + (n))
#define UB953_REG_GPIO_INPUT_CTRL_INPUT_EN(n)	BIT(0 + (n))

#define UB953_REG_REV_MASK_ID			0x50

#define UB953_REG_GPIO_PIN_STS			0x53
#define UB953_REG_GPIO_PIN_STS_GPIO_STS(n)	BIT(0 + (n))

#define UB953_REG_IND_ACC_CTL			0xb0
#define UB953_REG_IND_ACC_CTL_IA_AUTO_INC	BIT(1)
#define UB953_REG_IND_ACC_CTL_IA_SEL_MASK	GENMASK(4, 2)
#define UB953_REG_IND_ACC_ADDR			0xb1
#define UB953_REG_IND_ACC_DATA			0xb2

#define UB953_IND_PGEN_CTL			0x01
#define UB953_IND_PGEN_CTL_PGEN_ENABLE		BIT(0)
#define UB953_IND_PGEN_CFG			0x02
#define UB953_IND_PGEN_CSI_DI			0x03
#define UB953_IND_PGEN_LINE_SIZE1		0x04
#define UB953_IND_PGEN_LINE_SIZE0		0x05
#define UB953_IND_PGEN_BAR_SIZE1		0x06
#define UB953_IND_PGEN_BAR_SIZE0		0x07
#define UB953_IND_PGEN_ACT_LPF1			0x08
#define UB953_IND_PGEN_ACT_LPF0			0x09
#define UB953_IND_PGEN_TOT_LPF1			0x0A
#define UB953_IND_PGEN_TOT_LPF0			0x0B
#define UB953_IND_PGEN_LINE_PD1			0x0C
#define UB953_IND_PGEN_LINE_PD0			0x0D
#define UB953_IND_PGEN_VBP			0x0E
#define UB953_IND_PGEN_VFP			0x0F
#define UB953_IND_PGEN_COLOR(n)			(0x10 + (n)) /* n <= 15 */

struct ub953_data {
	struct i2c_client	*client;
	struct regmap		*regmap;

	struct clk		*line_rate_clk;
	struct clk_hw		clk_out_hw;

	u32			gpio_func[UB953_NUM_GPIOS];

	struct gpio_chip	gpio_chip;
	char			gpio_chip_name[64];

	struct v4l2_subdev	sd;
	struct media_pad	pads[2];

	struct v4l2_async_notifier	notifier;

	struct v4l2_subdev	*source_sd;

	struct v4l2_ctrl_handler   ctrl_handler;

	bool			streaming;

	struct device_node	*rx_ep_np;
	struct device_node	*tx_ep_np;

	bool			use_1v8_i2c;

	u8			clkout_mul;
	u8			clkout_div;
	u8			clkout_ctrl0;
	u8			clkout_ctrl1;
};

static inline struct ub953_data *sd_to_ub953(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ub953_data, sd);
}

/*
 * HW Access
 */

static int ub953_read(const struct ub953_data *priv, u8 reg, u8 *val)
{
	unsigned int v;
	int ret;

	ret = regmap_read(priv->regmap, reg, &v);
	if (ret < 0) {
		dev_err(&priv->client->dev,
			"Cannot read register 0x%02x: %d!\n", reg, ret);
		return ret;
	}

	*val = v;
	return 0;
}

static int ub953_write(const struct ub953_data *priv, u8 reg, u8 val)
{
	int ret;

	ret = regmap_write(priv->regmap, reg, val);
	if (ret < 0)
		dev_err(&priv->client->dev,
			"Cannot write register 0x%02x: %d!\n", reg, ret);

	return ret;
}

static int ub953_write_ind8(const struct ub953_data *priv, u8 reg, u8 val)
{
	int ret;

	ret = ub953_write(priv, UB953_REG_IND_ACC_ADDR, reg);
	if (!ret)
		ret = ub953_write(priv, UB953_REG_IND_ACC_DATA, val);
	return ret;
}

/* Assumes IA_AUTO_INC is set in UB953_REG_IND_ACC_CTL */
static int ub953_write_ind16(const struct ub953_data *priv, u8 reg, u16 val)
{
	int ret;

	ret = ub953_write(priv, UB953_REG_IND_ACC_ADDR, reg);
	if (!ret)
		ret = ub953_write(priv, UB953_REG_IND_ACC_DATA, val >> 8);
	if (!ret)
		ret = ub953_write(priv, UB953_REG_IND_ACC_DATA, val & 0xff);
	return ret;
}

/*
 * Clock output
 */

/*
 * Assume mode 0 "CSI-2 Synchronous mode" (strap, reg 0x03) is always
 * used. In this mode all clocks are derived from the deserializer. Other
 * modes are not implemented.
 */

/*
 * We always use 4 as a pre-divider (HS_CLK_DIV = 2).
 *
 * According to the datasheet:
 * - "HS_CLK_DIV typically should be set to either 16, 8, or 4 (default)."
 * - "if it is not possible to have an integer ratio of N/M, it is best to
 *    select a smaller value for HS_CLK_DIV.
 *
 * For above reasons the default HS_CLK_DIV seems the best in the average
 * case. Use always that value to keep the code simple.
 */
static const unsigned long hs_clk_div = 2;
static const unsigned long prediv = (1 << hs_clk_div);

static unsigned long ub953_clkout_recalc_rate(struct clk_hw *hw,
					      unsigned long parent_rate)
{
	struct ub953_data *priv = container_of(hw, struct ub953_data, clk_out_hw);
	u8 ctrl0, ctrl1;
	unsigned long mul, div, ret;

	ub953_read(priv, UB953_REG_CLKOUT_CTRL0, &ctrl0);
	ub953_read(priv, UB953_REG_CLKOUT_CTRL1, &ctrl1);

	if (ctrl0 < 0 || ctrl1 < 0) {
		/* Perhaps link down, use cached values */
		ctrl0 = priv->clkout_ctrl0;
		ctrl1 = priv->clkout_ctrl1;
	}

	mul = ctrl0 & 0x1f;
	div = ctrl1 & 0xff;

	if (div == 0)
		return 0;

	ret = parent_rate / prediv * mul / div;

	return ret;
}

static long ub953_clkout_round_rate(struct clk_hw *hw, unsigned long rate,
				    unsigned long *parent_rate)
{
	struct ub953_data *priv = container_of(hw, struct ub953_data, clk_out_hw);
	struct device *dev = &priv->client->dev;
	unsigned long mul, div, res;

	rational_best_approximation(rate, *parent_rate / prediv,
				    (1 << 5) - 1, (1 << 8) - 1,
				    &mul, &div);
	priv->clkout_mul = mul;
	priv->clkout_div = div;

	res = *parent_rate / prediv * priv->clkout_mul / priv->clkout_div;

	dev_dbg(dev, "%lu / %lu * %lu / %lu = %lu (wanted %lu)",
		*parent_rate, prediv, mul, div, res, rate);

	return res;
}

static int ub953_clkout_set_rate(struct clk_hw *hw, unsigned long rate,
				 unsigned long parent_rate)
{
	struct ub953_data *priv = container_of(hw, struct ub953_data, clk_out_hw);

	priv->clkout_ctrl0 = (hs_clk_div << 5) | priv->clkout_mul;
	priv->clkout_ctrl1 = priv->clkout_div;

	ub953_write(priv, UB953_REG_CLKOUT_CTRL0, priv->clkout_ctrl0);
	ub953_write(priv, UB953_REG_CLKOUT_CTRL1, priv->clkout_ctrl1);

	return 0;
}

static const struct clk_ops ub953_clkout_ops = {
	.recalc_rate	= ub953_clkout_recalc_rate,
	.round_rate	= ub953_clkout_round_rate,
	.set_rate	= ub953_clkout_set_rate,
};

static int ub953_register_clkout(struct ub953_data *priv)
{
	struct device *dev = &priv->client->dev;
	const char *parent_names[1] = { __clk_get_name(priv->line_rate_clk) };
	const struct clk_init_data init = {
		.name         = kasprintf(GFP_KERNEL, "%s.clk_out", dev_name(dev)),
		.ops          = &ub953_clkout_ops,
		.parent_names = parent_names,
		.num_parents  = 1,
	};
	int err;

	priv->clk_out_hw.init = &init;

	err = devm_clk_hw_register(dev, &priv->clk_out_hw);
	kfree(init.name); /* clock framework made a copy of the name */
	if (err)
		return dev_err_probe(dev, err, "Cannot register clock HW\n");

	err = devm_of_clk_add_hw_provider(dev, of_clk_hw_simple_get, &priv->clk_out_hw);
	if (err)
		return dev_err_probe(dev, err, "Cannot add OF clock provider\n");

	return 0;
}

/*
 * GPIO chip
 */
static int ub953_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	struct ub953_data *priv = gpiochip_get_data(gc);
	int ret;
	u8 v;

	ret = ub953_read(priv, UB953_REG_GPIO_INPUT_CTRL, &v);
	if (ret)
		return ret;

	if (v & UB953_REG_GPIO_INPUT_CTRL_INPUT_EN(offset))
		return GPIO_LINE_DIRECTION_IN;
	else
		return GPIO_LINE_DIRECTION_OUT;
}

static int ub953_gpio_direction_in(struct gpio_chip *gc, unsigned int offset)
{
	struct ub953_data *priv = gpiochip_get_data(gc);

	return regmap_update_bits(
		priv->regmap, UB953_REG_GPIO_INPUT_CTRL,
		UB953_REG_GPIO_INPUT_CTRL_INPUT_EN(offset) |
			UB953_REG_GPIO_INPUT_CTRL_OUT_EN(offset),
		UB953_REG_GPIO_INPUT_CTRL_INPUT_EN(offset));
}

static int ub953_gpio_direction_out(struct gpio_chip *gc, unsigned int offset,
				    int value)
{
	struct ub953_data *priv = gpiochip_get_data(gc);
	int ret;

	ret = regmap_update_bits(
		priv->regmap, UB953_REG_LOCAL_GPIO_DATA,
		UB953_REG_LOCAL_GPIO_DATA_GPIO_OUT_SRC(offset),
		value ? UB953_REG_LOCAL_GPIO_DATA_GPIO_OUT_SRC(offset) : 0);

	if (ret)
		return ret;

	return regmap_update_bits(
		priv->regmap, UB953_REG_GPIO_INPUT_CTRL,
		UB953_REG_GPIO_INPUT_CTRL_INPUT_EN(offset) |
			UB953_REG_GPIO_INPUT_CTRL_OUT_EN(offset),
		UB953_REG_GPIO_INPUT_CTRL_OUT_EN(offset));
}

static int ub953_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct ub953_data *priv = gpiochip_get_data(gc);
	int ret;
	u8 v;

	ret = ub953_read(priv, UB953_REG_GPIO_PIN_STS, &v);
	if (ret)
		return ret;

	return !!(v & UB953_REG_GPIO_PIN_STS_GPIO_STS(offset));
}

static void ub953_gpio_set(struct gpio_chip *gc, unsigned int offset, int value)
{
	struct ub953_data *priv = gpiochip_get_data(gc);

	regmap_update_bits(
		priv->regmap, UB953_REG_LOCAL_GPIO_DATA,
		UB953_REG_LOCAL_GPIO_DATA_GPIO_OUT_SRC(offset),
		value ? UB953_REG_LOCAL_GPIO_DATA_GPIO_OUT_SRC(offset) : 0);
}

static int ub953_gpio_of_xlate(struct gpio_chip *gc,
			       const struct of_phandle_args *gpiospec,
			       u32 *flags)
{
	if (flags)
		*flags = gpiospec->args[1];

	return gpiospec->args[0];
}

static int ub953_gpiochip_probe(struct ub953_data *priv)
{
	struct device *dev = &priv->client->dev;
	struct gpio_chip *gc = &priv->gpio_chip;
	int ret;

	/* Set all GPIOs to local mode */
	ub953_write(priv, UB953_REG_LOCAL_GPIO_DATA, 0);

	scnprintf(priv->gpio_chip_name, sizeof(priv->gpio_chip_name), "%s",
		  dev_name(dev));

	gc->label = priv->gpio_chip_name;
	gc->parent = dev;
	gc->owner = THIS_MODULE;
	gc->base = -1;
	gc->can_sleep = 1;
	gc->ngpio = UB953_NUM_GPIOS;
	gc->get_direction = ub953_gpio_get_direction;
	gc->direction_input = ub953_gpio_direction_in;
	gc->direction_output = ub953_gpio_direction_out;
	gc->get = ub953_gpio_get;
	gc->set = ub953_gpio_set;
	gc->of_xlate = ub953_gpio_of_xlate;
	gc->of_node = priv->client->dev.of_node;
	gc->of_gpio_n_cells = 2;

	ret = gpiochip_add_data(gc, priv);
	if (ret) {
		dev_err(dev, "Failed to add GPIOs: %d\n", ret);
		return ret;
	}

	return 0;
}

static void ub953_gpiochip_remove(struct ub953_data *priv)
{
	gpiochip_remove(&priv->gpio_chip);
}

/*
 * V4L2
 */

static int ub953_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ub953_data *priv = sd_to_ub953(sd);
	int ret;

	priv->streaming = enable;

	ret = v4l2_subdev_call(priv->source_sd, video, s_stream, enable);
	if (ret && enable)
		priv->streaming = false;

	return ret;
}

static const struct v4l2_subdev_video_ops ub953_video_ops = {
	.s_stream = ub953_s_stream,
};

static int _ub953_set_routing(struct v4l2_subdev *sd,
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


static int ub953_set_routing(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state,
			     enum v4l2_subdev_format_whence which,
			     struct v4l2_subdev_krouting *routing)
{
	struct ub953_data *priv = sd_to_ub953(sd);

	if (which == V4L2_SUBDEV_FORMAT_ACTIVE && priv->streaming)
		return -EBUSY;

	return _ub953_set_routing(sd, state, routing);
}

static int ub953_get_source_frame_desc(struct ub953_data *priv,
				       struct v4l2_mbus_frame_desc *desc)
{
	struct media_pad *pad;
	int ret;

	pad = media_entity_remote_pad(&priv->pads[UB953_PAD_SINK]);
	if (!pad)
		return -EPIPE;

	ret = v4l2_subdev_call(priv->source_sd, pad, get_frame_desc, pad->index,
			       desc);
	if (ret)
		return ret;

	return 0;
}

static int ub953_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				struct v4l2_mbus_frame_desc *fd)
{
	struct ub953_data *priv = sd_to_ub953(sd);
	const struct v4l2_subdev_krouting *routing;
	struct v4l2_mbus_frame_desc source_fd;
	struct v4l2_subdev_state *state;
	unsigned int i;
	int ret = 0;

	if (pad != 1) /* first tx pad */
		return -EINVAL;

	ret = ub953_get_source_frame_desc(priv, &source_fd);
	if (ret)
		return ret;

	state = v4l2_subdev_lock_active_state(sd);

	routing = &state->routing;

	memset(fd, 0, sizeof(*fd));

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;

	for (i = 0; i < routing->num_routes; ++i) {
		const struct v4l2_subdev_route *route = &routing->routes[i];
		struct v4l2_mbus_frame_desc_entry *source_entry = NULL;
		unsigned int j;

		if (!(route->flags & V4L2_SUBDEV_ROUTE_FL_ACTIVE))
			continue;

		if (route->source_pad != pad)
			continue;

		for (j = 0; j < source_fd.num_entries; ++j)
			if (source_fd.entry[j].stream == route->sink_stream) {
				source_entry = &source_fd.entry[j];
				break;
			}

		if (!source_entry) {
			dev_err(&priv->client->dev,
				"Failed to find stream from source frame desc\n");
			ret = -EPIPE;
			goto out;
		}

		fd->entry[fd->num_entries].stream = route->source_stream;

		fd->entry[fd->num_entries].flags =
			V4L2_MBUS_FRAME_DESC_FL_LEN_MAX;
		fd->entry[fd->num_entries].length = source_entry->length;
		fd->entry[fd->num_entries].pixelcode = source_entry->pixelcode;
		fd->entry[fd->num_entries].bus.csi2.vc =
			source_entry->bus.csi2.vc;
		fd->entry[fd->num_entries].bus.csi2.dt =
			source_entry->bus.csi2.dt;

		fd->num_entries++;
	}

out:
	v4l2_subdev_unlock_state(state);

	return ret;
}

static int ub953_set_fmt(struct v4l2_subdev *sd,
			 struct v4l2_subdev_state *state,
			 struct v4l2_subdev_format *format)
{
	struct ub953_data *priv = sd_to_ub953(sd);
	struct v4l2_mbus_framefmt *fmt;
	int ret = 0;

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE && priv->streaming)
		return -EBUSY;

	/* No transcoding, source and sink formats must match. */
	if (format->pad == 1)
		return v4l2_subdev_get_fmt(sd, state, format);

	v4l2_subdev_lock_state(state);

	/* Set sink format */
	fmt = v4l2_state_get_stream_format(state, format->pad, format->stream);
	if (!fmt) {
		ret = -EINVAL;
		goto out;
	}

	*fmt = format->format;

	/* Propagate to source format */
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

static int ub953_init_cfg(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_route routes[] = {
		{
			.sink_pad = 0,
			.sink_stream = 0,
			.source_pad = 1,
			.source_stream = 0,
			.flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE,
		},
	};

	struct v4l2_subdev_krouting routing = {
		.num_routes = ARRAY_SIZE(routes),
		.routes = routes,
	};

	return _ub953_set_routing(sd, state, &routing);
}

static const struct v4l2_subdev_pad_ops ub953_pad_ops = {
	.set_routing = ub953_set_routing,
	.get_frame_desc = ub953_get_frame_desc,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = ub953_set_fmt,
	.init_cfg = ub953_init_cfg,
};

static const struct v4l2_subdev_ops ub953_subdev_ops = {
	.video = &ub953_video_ops,
	.pad = &ub953_pad_ops,
};

static const struct media_entity_operations ub953_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

enum {
	TEST_PATTERN_DISABLED = 0,
	TEST_PATTERN_V_COLOR_BARS_1,
	TEST_PATTERN_V_COLOR_BARS_2,
	TEST_PATTERN_V_COLOR_BARS_4,
	TEST_PATTERN_V_COLOR_BARS_8,
};

static const char *const ub953_tpg_qmenu[] = {
	"Disabled",
	"1 vertical color bar",
	"2 vertical color bars",
	"4 vertical color bars",
	"8 vertical color bars",
};

static void ub953_enable_tpg(struct ub953_data *priv, int tpg_num)
{
	struct v4l2_subdev *sd = &priv->sd;
	struct v4l2_subdev_state *state;
	struct v4l2_mbus_framefmt *fmt;
	u8 vbp, vfp;
	u16 blank_lines;
	u16 width;
	u16 height;

	u16 bytespp = 2; /* For MEDIA_BUS_FMT_UYVY8_1X16 */
	u8 cbars_idx = tpg_num - TEST_PATTERN_V_COLOR_BARS_1;
	u8 num_cbars = 1 << cbars_idx;

	u16 line_size; /* Line size [bytes] */
	u16 bar_size; /* cbar size [bytes] */
	u16 act_lpf; /* active lines/frame */
	u16 tot_lpf; /* tot lines/frame */
	u16 line_pd; /* Line period in 10-ns units */

	u16 fps = 30;

	vbp = 33;
	vfp = 10;
	blank_lines = vbp + vfp + 2; /* total blanking lines */

	state = v4l2_subdev_lock_active_state(sd);

	fmt = v4l2_state_get_stream_format(state, UB953_PAD_SOURCE, 0);

	width = fmt->width;
	height = fmt->height;

	line_size = width * bytespp;
	bar_size = line_size / num_cbars;
	act_lpf = height;
	tot_lpf = act_lpf + blank_lines;
	line_pd = 100000000 / fps / tot_lpf;

	/* Access Indirect Pattern Gen */
	ub953_write(priv, UB953_REG_IND_ACC_CTL,
		    UB953_REG_IND_ACC_CTL_IA_AUTO_INC | (0 << 2));

	ub953_write_ind8(priv, UB953_IND_PGEN_CTL,
			 UB953_IND_PGEN_CTL_PGEN_ENABLE);

	/* YUV422 8bit: 2 bytes/block, CSI-2 data type 0x1e */
	ub953_write_ind8(priv, UB953_IND_PGEN_CFG, cbars_idx << 4 | 0x2);
	ub953_write_ind8(priv, UB953_IND_PGEN_CSI_DI, 0x1e);

	ub953_write_ind16(priv, UB953_IND_PGEN_LINE_SIZE1, line_size);
	ub953_write_ind16(priv, UB953_IND_PGEN_BAR_SIZE1, bar_size);
	ub953_write_ind16(priv, UB953_IND_PGEN_ACT_LPF1, act_lpf);
	ub953_write_ind16(priv, UB953_IND_PGEN_TOT_LPF1, tot_lpf);
	ub953_write_ind16(priv, UB953_IND_PGEN_LINE_PD1, line_pd);
	ub953_write_ind8(priv, UB953_IND_PGEN_VBP, vbp);
	ub953_write_ind8(priv, UB953_IND_PGEN_VFP, vfp);

	v4l2_subdev_unlock_state(state);
}

static void ub953_disable_tpg(struct ub953_data *priv)
{
	ub953_write_ind8(priv, UB953_IND_PGEN_CTL, 0x00);
}

static int ub953_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ub953_data *priv =
		container_of(ctrl->handler, struct ub953_data, ctrl_handler);

	switch (ctrl->id) {
	case V4L2_CID_TEST_PATTERN:
		if (ctrl->val == 0)
			ub953_disable_tpg(priv);
		else
			ub953_enable_tpg(priv, ctrl->val);
		break;
	}

	return 0;
}

static const struct v4l2_ctrl_ops ub953_ctrl_ops = {
	.s_ctrl = ub953_s_ctrl,
};

static int ub953_notify_bound(struct v4l2_async_notifier *notifier,
			      struct v4l2_subdev *source_subdev,
			      struct v4l2_async_subdev *asd)
{
	struct ub953_data *priv = sd_to_ub953(notifier->sd);
	struct device *dev = &priv->client->dev;
	unsigned int src_pad;
	int ret;

	dev_dbg(dev, "Bind %s\n", source_subdev->name);

	ret = media_entity_get_fwnode_pad(&source_subdev->entity,
					  source_subdev->fwnode,
					  MEDIA_PAD_FL_SOURCE);
	if (ret < 0) {
		dev_err(dev, "Failed to find pad for %s\n",
			source_subdev->name);
		return ret;
	}

	priv->source_sd = source_subdev;
	src_pad = ret;

	ret = media_create_pad_link(
		&source_subdev->entity, src_pad, &priv->sd.entity, 0,
		MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE);
	if (ret) {
		dev_err(dev, "Unable to link %s:%u -> %s:0\n",
			source_subdev->name, src_pad, priv->sd.name);
		return ret;
	}

	dev_dbg(dev, "Bound %s:%u\n", source_subdev->name, src_pad);

	dev_dbg(dev, "All subdevs bound\n");

	return 0;
}

static void ub953_notify_unbind(struct v4l2_async_notifier *notifier,
				struct v4l2_subdev *source_subdev,
				struct v4l2_async_subdev *asd)
{
	struct ub953_data *priv = sd_to_ub953(notifier->sd);
	struct device *dev = &priv->client->dev;

	dev_dbg(dev, "Unbind %s\n", source_subdev->name);
}

static const struct v4l2_async_notifier_operations ub953_notify_ops = {
	.bound = ub953_notify_bound,
	.unbind = ub953_notify_unbind,
};

static int ub953_v4l2_notifier_register(struct ub953_data *priv)
{
	struct device *dev = &priv->client->dev;
	struct v4l2_async_subdev *asd;
	struct device_node *ep_node;
	int ret;

	dev_dbg(dev, "register async notif\n");

	ep_node = of_graph_get_endpoint_by_regs(dev->of_node, 0, 0);
	if (!ep_node) {
		dev_err(dev, "No graph endpoint\n");
		return -ENODEV;
	}

	v4l2_async_notifier_init(&priv->notifier);

	asd = v4l2_async_notifier_add_fwnode_remote_subdev(
		&priv->notifier, of_fwnode_handle(ep_node),
		sizeof(*asd));

	of_node_put(ep_node);

	if (IS_ERR(asd)) {
		dev_err(dev, "Failed to add subdev: %ld", PTR_ERR(asd));
		v4l2_async_notifier_cleanup(&priv->notifier);
		return PTR_ERR(asd);
	}

	priv->notifier.ops = &ub953_notify_ops;

	ret = v4l2_async_subdev_notifier_register(&priv->sd, &priv->notifier);
	if (ret) {
		dev_err(dev, "Failed to register subdev_notifier");
		v4l2_async_notifier_cleanup(&priv->notifier);
		return ret;
	}

	return 0;
}

static void ub953_v4l2_notifier_unregister(struct ub953_data *priv)
{
	struct device *dev = &priv->client->dev;

	dev_dbg(dev, "Unregister async notif\n");

	v4l2_async_notifier_unregister(&priv->notifier);
	v4l2_async_notifier_cleanup(&priv->notifier);
}

/*
 * Probing
 */

static void ub953_soft_reset(struct ub953_data *priv)
{
	struct device *dev = &priv->client->dev;
	int retries;

	ub953_write(priv, UB953_REG_RESET_CTL,
		    UB953_REG_RESET_CTL_DIGITAL_RESET_1);

	usleep_range(10000, 30000);

	retries = 10;
	while (retries-- > 0) {
		int ret;
		u8 v;

		ret = ub953_read(priv, UB953_REG_RESET_CTL, &v);

		if (ret >= 0 &&
		    (v & UB953_REG_RESET_CTL_DIGITAL_RESET_1) == 0) {
			dev_dbg(dev, "reset done\n");
			break;
		}

		usleep_range(1000, 3000);
	}

	if (retries == 0)
		dev_err(dev, "reset timeout\n");
}

static int ub953_i2c_init(struct ub953_data *priv)
{
	/* i2c fast mode */
	u32 scl_high = 915; /* ns */
	u32 scl_low = 1641; /* ns */
	u32 ref = 25000000; /* TODO: get refclock from deserializer */
	int ret = 0;

	scl_high = div64_u64((u64)scl_high * ref, 1000000000) - 5;
	scl_low = div64_u64((u64)scl_low * ref, 1000000000) - 5;

	ret = ub953_write(priv, UB953_REG_SCL_HIGH_TIME, scl_high);
	if (ret)
		return ret;

	ret = ub953_write(priv, UB953_REG_SCL_LOW_TIME, scl_low);
	if (ret)
		return ret;

	return 0;
}

static int ub953_general_cfg(struct ub953_data *priv)
{
	struct device *dev = &priv->client->dev;
	u32 num_data_lanes;
	bool clock_continuous;
	int ret;

	ret = of_property_count_u32_elems(priv->rx_ep_np, "data-lanes");
	if (ret < 1 || ret > UB953_MAX_DATA_LANES) {
		dev_err(dev, "DT: invalid data-lanes (%d), only 1-4 lanes supported\n", ret);
		return ret;
	} else {
		num_data_lanes = ret;
	}

	clock_continuous = !of_property_read_bool(priv->rx_ep_np, "clock-noncontinuous");

	return ub953_write(priv, UB953_REG_GENERAL_CFG,
			   ((clock_continuous) << 6) |
			   ((num_data_lanes - 1) << 4) |
			   (1 << 1) | /* CRC TX gen */
			   (priv->use_1v8_i2c << 0));
}

static int ub953_parse_dt(struct ub953_data *priv)
{
	struct device_node *np = priv->client->dev.of_node;
	struct device *dev = &priv->client->dev;
	int ret;

	if (!np) {
		dev_err(dev, "OF: no device tree node!\n");
		return -ENOENT;
	}

	/* optional, if absent all GPIO pins are unused */
	ret = of_property_read_u32_array(np, "gpio-functions", priv->gpio_func,
					 ARRAY_SIZE(priv->gpio_func));
	if (ret && ret != -EINVAL)
		dev_err(dev, "DT: invalid gpio-functions property (%d)", ret);

	/* read i2c voltage level */
	priv->use_1v8_i2c = of_property_read_bool(np, "i2c-1_8v");

	return 0;
}

static const struct regmap_config ub953_regmap_config = {
	.name = "ds90ub953",
	.reg_bits = 8,
	.val_bits = 8,
	.reg_format_endian = REGMAP_ENDIAN_DEFAULT,
	.val_format_endian = REGMAP_ENDIAN_DEFAULT,
};

static int ub953_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ub953_data *priv;
	int ret;
	u8 rev;

	dev_dbg(dev, "probing, addr 0x%02x\n", client->addr);

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->client = client;

	priv->regmap = devm_regmap_init_i2c(client, &ub953_regmap_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(dev, "Failed to init regmap\n");
		return PTR_ERR(priv->regmap);
	}

	priv->line_rate_clk = devm_clk_get(dev, NULL);
	if (IS_ERR(priv->line_rate_clk))
		return dev_err_probe(dev, PTR_ERR(priv->line_rate_clk),
				     "Cannot get line rate clock\n");
	dev_dbg(dev, "line rate: %lu Hz\n", clk_get_rate(priv->line_rate_clk));

	ret = ub953_parse_dt(priv);
	if (ret)
		return ret;

	ub953_soft_reset(priv);

	ret = ub953_read(priv, UB953_REG_REV_MASK_ID, &rev);
	if (ret) {
		dev_err(dev, "Failed to read revision: %d", ret);
		return ret;
	}

	dev_info(dev, "Found rev %u, mask %u\n", rev >> 4, rev & 0xf);

	ret = ub953_i2c_init(priv);
	if (ret) {
		dev_err(dev, "i2c init failed: %d\n", ret);
		return ret;
	}

	ret = ub953_gpiochip_probe(priv);
	if (ret) {
		dev_err(dev, "Failed to init gpiochip\n");
		return ret;
	}

	v4l2_i2c_subdev_init(&priv->sd, priv->client, &ub953_subdev_ops);

	v4l2_ctrl_handler_init(&priv->ctrl_handler,
			       ARRAY_SIZE(ub953_tpg_qmenu) - 1);
	priv->sd.ctrl_handler = &priv->ctrl_handler;

	v4l2_ctrl_new_std_menu_items(&priv->ctrl_handler, &ub953_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(ub953_tpg_qmenu) - 1, 0, 0,
				     ub953_tpg_qmenu);

	if (priv->ctrl_handler.error) {
		ret = priv->ctrl_handler.error;
		goto err_gpiochip_remove;
	}

	priv->sd.flags |=
		V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_MULTIPLEXED;
	priv->sd.entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	priv->sd.entity.ops = &ub953_entity_ops;

	priv->pads[0].flags = MEDIA_PAD_FL_SINK;
	priv->pads[1].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&priv->sd.entity, 2, priv->pads);
	if (ret) {
		dev_err(dev, "Failed to init pads\n");
		goto err_remove_ctrls;
	}

	priv->rx_ep_np = of_graph_get_endpoint_by_regs(dev->of_node, 0, 0);
	priv->tx_ep_np = of_graph_get_endpoint_by_regs(dev->of_node, 1, 0);
	priv->sd.fwnode = of_fwnode_handle(priv->tx_ep_np);

	ret = v4l2_subdev_init_finalize(&priv->sd);
	if (ret)
		goto err_entity_cleanup;

	ret = ub953_v4l2_notifier_register(priv);
	if (ret) {
		dev_err(dev, "v4l2 subdev notifier register failed: %d\n", ret);
		goto err_free_state;
	}

	ret = v4l2_async_register_subdev(&priv->sd);
	if (ret) {
		dev_err(dev, "v4l2_async_register_subdev error: %d\n", ret);
		goto err_unreg_notif;
	}

	/* Default values for clock multiplier and divider registers */
	priv->clkout_ctrl0 = 0x41;
	priv->clkout_ctrl1 = 0x28;
	ret = ub953_register_clkout(priv);
	if (ret) {
		goto err_unreg_notif;
	}

	ub953_general_cfg(priv);
	if (ret)
		goto err_unreg_notif;

	dev_dbg(dev, "Successfully probed\n");

	return 0;

err_unreg_notif:
	ub953_v4l2_notifier_unregister(priv);
err_free_state:
	v4l2_subdev_cleanup(&priv->sd);
err_entity_cleanup:
	if (priv->rx_ep_np)
		of_node_put(priv->rx_ep_np);
	if (priv->tx_ep_np)
		of_node_put(priv->tx_ep_np);

	media_entity_cleanup(&priv->sd.entity);
err_remove_ctrls:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
err_gpiochip_remove:
	ub953_gpiochip_remove(priv);

	return ret;
}

static int ub953_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ub953_data *priv = sd_to_ub953(sd);

	dev_dbg(&client->dev, "Removing\n");

	ub953_v4l2_notifier_unregister(priv);
	v4l2_async_unregister_subdev(&priv->sd);

	v4l2_subdev_cleanup(&priv->sd);

	of_node_put(priv->tx_ep_np);

	media_entity_cleanup(&priv->sd.entity);

	v4l2_ctrl_handler_free(&priv->ctrl_handler);

	ub953_gpiochip_remove(priv);

	return 0;
}

static const struct i2c_device_id ub953_id[] = { { "ds90ub953-q1", 0 }, {} };
MODULE_DEVICE_TABLE(i2c, ub953_id);

#ifdef CONFIG_OF
static const struct of_device_id ub953_dt_ids[] = {
	{ .compatible = "ti,ds90ub953-q1", },
	{}
};
MODULE_DEVICE_TABLE(of, ub953_dt_ids);
#endif

static struct i2c_driver ds90ub953_driver = {
	.probe_new	= ub953_probe,
	.remove		= ub953_remove,
	.id_table	= ub953_id,
	.driver = {
		.name	= "ds90ub953",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ub953_dt_ids),
	},
};

module_i2c_driver(ds90ub953_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Texas Instruments DS90UB953 serializer driver");
MODULE_AUTHOR("Luca Ceresoli <luca@lucaceresoli.net>");
MODULE_AUTHOR("Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>");
