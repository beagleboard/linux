// SPDX-License-Identifier: GPL-2.0
/*
 * Sony IMX390 CMOS Image Sensor Driver
 *
 * Copyright (c) 2021 Apurva Nandan <a-nandan@ti.com>
 *
 * Copyright (c) 2021 Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>
 */

#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/v4l2-mediabus.h>
#include <linux/videodev2.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>

#include "imx390.h"

static inline struct imx390 *to_imx390(struct v4l2_subdev *sd)
{
	return container_of(sd, struct imx390, subdev);
}

static int imx390_read(struct imx390 *imx390, u16 addr, u32 *val, size_t nbytes)
{
	int ret;
	__le32 val_le = 0;

	ret = regmap_bulk_read(imx390->regmap, addr, &val_le, nbytes);
	if (ret < 0) {
		dev_err(imx390->dev, "%s: failed to read reg 0x%04x: %d\n",
			__func__, addr, ret);
		return ret;
	}

	*val = le32_to_cpu(val_le);
	return 0;
}

static int imx390_write(struct imx390 *imx390, u16 addr, u32 val, size_t nbytes)
{
	int ret;
	__le32 val_le = cpu_to_le32(val);

	ret = regmap_bulk_write(imx390->regmap, addr, &val_le, nbytes);
	if (ret < 0)
		dev_err(imx390->dev, "%s: failed to write reg 0x%04x: %d\n",
			__func__, addr, ret);
	return ret;
}

static int imx390_update_bits(struct imx390 *imx390, u16 addr, u32 val,
			      u32 mask, size_t nbytes)
{
	int ret;
	u32 cfg;

	ret = imx390_read(imx390, addr, &cfg, nbytes);
	if (ret < 0)
		return ret;

	cfg = (val) ? (cfg | mask) : (cfg & (~mask));

	return imx390_write(imx390, addr, cfg, nbytes);
}

static int imx390_write_table(struct imx390 *imx390,
			      const struct reg_sequence *regs,
			      unsigned int nr_regs)
{
	int ret;

	ret = regmap_multi_reg_write(imx390->regmap, regs, nr_regs);
	if (ret < 0)
		dev_err(imx390->dev,
			"%s: failed to write reg table (%d)!\n", __func__, ret);
	return ret;
}

static void imx390_init_formats(struct v4l2_subdev_state *state)
{
	struct v4l2_mbus_framefmt *format;

	format = v4l2_state_get_stream_format(state, 0, 0);
	format->code = imx390_mbus_formats[0];
	format->width = imx390_framesizes[0].width;
	format->height = imx390_framesizes[0].height;
	format->field = V4L2_FIELD_NONE;
	format->colorspace = V4L2_COLORSPACE_SMPTE170M;
}

static int _imx390_set_routing(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_route routes[] = {
		{
			.source_pad = 0,
			.source_stream = 0,
			.flags = V4L2_SUBDEV_ROUTE_FL_IMMUTABLE |
				 V4L2_SUBDEV_ROUTE_FL_SOURCE |
				 V4L2_SUBDEV_ROUTE_FL_ACTIVE,
		},
		{
			.source_pad = 0,
			.source_stream = 1,
			.flags = V4L2_SUBDEV_ROUTE_FL_IMMUTABLE |
				 V4L2_SUBDEV_ROUTE_FL_SOURCE,
		}
	};

	struct v4l2_subdev_krouting routing = {
		.num_routes = ARRAY_SIZE(routes),
		.routes = routes,
	};

	int ret;

	ret = v4l2_subdev_set_routing(sd, state, &routing);
	if (ret < 0)
		return ret;

	imx390_init_formats(state);

	return 0;
}

static int imx390_init_cfg(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *state)
{
	int ret;

	v4l2_subdev_lock_state(state);

	ret = _imx390_set_routing(sd, state);

	v4l2_subdev_unlock_state(state);

	return ret;
}

static int imx390_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(imx390_mbus_formats))
		return -EINVAL;

	code->code = imx390_mbus_formats[code->index];

	return 0;
}

static int imx390_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(imx390_mbus_formats); ++i) {
		if (imx390_mbus_formats[i] == fse->code)
			break;
	}

	if (i == ARRAY_SIZE(imx390_mbus_formats))
		return -EINVAL;

	if (fse->index >= ARRAY_SIZE(imx390_framesizes))
		return -EINVAL;

	fse->min_width = imx390_framesizes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->max_height = imx390_framesizes[fse->index].height;
	fse->min_height = fse->max_height;

	return 0;
}

static int imx390_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	struct imx390 *imx390 = to_imx390(sd);
	struct v4l2_mbus_framefmt *format;
	const struct v4l2_area *fsize;
	unsigned int i;
	u32 code;
	int ret = 0;

	if (fmt->pad != 0)
		return -EINVAL;

	if (fmt->stream != 0)
		return -EINVAL;

	/*
	 * Validate the media bus code, defaulting to the first one if the
	 * requested code isn't supported.
	 */
	for (i = 0; i < ARRAY_SIZE(imx390_mbus_formats); ++i) {
		if (imx390_mbus_formats[i] == fmt->format.code) {
			code = fmt->format.code;
			break;
		}
	}

	if (i == ARRAY_SIZE(imx390_mbus_formats))
		code = imx390_mbus_formats[0];

	/* Find the nearest supported frame size. */
	fsize = v4l2_find_nearest_size(imx390_framesizes,
				       ARRAY_SIZE(imx390_framesizes), width,
				       height, fmt->format.width,
				       fmt->format.height);

	v4l2_subdev_lock_state(state);

	/* Update the stored format and return it. */
	format = v4l2_state_get_stream_format(state, fmt->pad, fmt->stream);

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE && imx390->streaming) {
		ret = -EBUSY;
		goto done;
	}

	format->code = code;
	format->width = fsize->width;
	format->height = fsize->height;

	fmt->format = *format;

done:
	v4l2_subdev_unlock_state(state);

	return ret;
}

static int imx390_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				 struct v4l2_mbus_frame_desc *fd)
{
	struct v4l2_subdev_state *state;
	struct v4l2_mbus_framefmt *fmt;
	u32 bpp;
	int ret = 0;

	if (pad != 0)
		return -EINVAL;

	state = v4l2_subdev_lock_active_state(sd);

	fmt = v4l2_state_get_stream_format(state, 0, 0);
	if (!fmt) {
		ret = -EPIPE;
		goto out;
	}

	memset(fd, 0, sizeof(*fd));

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;

	/* pixel stream */

	bpp = 12;

	fd->entry[fd->num_entries].stream = 0;

	fd->entry[fd->num_entries].flags = V4L2_MBUS_FRAME_DESC_FL_LEN_MAX;
	fd->entry[fd->num_entries].length = fmt->width * fmt->height * bpp / 8;
	fd->entry[fd->num_entries].pixelcode = fmt->code;
	fd->entry[fd->num_entries].bus.csi2.vc = 0;
	fd->entry[fd->num_entries].bus.csi2.dt = 0x2c; /* SRGGB12 */

	fd->num_entries++;

out:
	v4l2_subdev_unlock_state(state);

	return ret;
}

static int imx390_set_routing(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state,
			      enum v4l2_subdev_format_whence which,
			      struct v4l2_subdev_krouting *routing)
{
	int ret;

	if (routing->num_routes == 0 || routing->num_routes > 1)
		return -EINVAL;

	v4l2_subdev_lock_state(state);

	ret = _imx390_set_routing(sd, state);

	v4l2_subdev_unlock_state(state);

	return ret;
}

static int imx390_check_non_wdr_mode_fps(struct imx390 *imx390, bool enable)
{
	if (!enable && imx390->fps > IMX390_FRAMERATE_MAX_LINEAR) {
		dev_err(imx390->dev,
			"%s: failed, %dFPS unsupported in non-WDR mode\n",
			__func__, imx390->fps);
		return -EINVAL;
	}
	return 0;
}

static int imx390_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx390 *imx390 = container_of(ctrl->handler,
					     struct imx390, ctrl.handler);
	int ret;

	dev_dbg(imx390->dev,
		"%s: %s, value: %d\n", __func__, ctrl->name, ctrl->val);

	/* V4L2 controls values will be applied only when power is already up */
	if (!pm_runtime_get_if_in_use(imx390->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_WIDE_DYNAMIC_RANGE:
		ret = imx390_check_non_wdr_mode_fps(imx390, ctrl->val);
		break;

	case V4L2_CID_EXPOSURE:
		ret = imx390_write(imx390, IMX390_REG_CAT0_SHS1,
				   IMX390_EXPOSURE_SHS_VAL(ctrl->val,
							   imx390->fps), 3);
		break;

	case V4L2_CID_ANALOGUE_GAIN:
		ret = imx390_write(imx390, IMX390_REG_CAT0_AGAIN_SP1H,
				   ctrl->val, 2);
		if (ret < 0)
			break;

		ret = imx390_write(imx390, IMX390_REG_CAT0_AGAIN_SP1L,
				   ctrl->val / IMX390_AGAIN_CONV_GAIN_RATIO, 2);
		break;

	case V4L2_CID_DIGITAL_GAIN:
		ret = imx390_write(imx390, IMX390_REG_CAT0_PGA_GAIN,
				   ctrl->val, 2);
		break;

	case V4L2_CID_RED_BALANCE:
		ret = imx390_write(imx390, IMX390_REG_CAT0_WBGAIN_R,
				   ctrl->val, 2);
		break;

	case V4L2_CID_BLUE_BALANCE:
		ret = imx390_write(imx390, IMX390_REG_CAT0_WBGAIN_B,
				   ctrl->val, 2);
		break;

	case V4L2_CID_HFLIP:
		ret = imx390_update_bits(imx390, IMX390_REG_CAT0_V_H_REVERSE,
					 ctrl->val, IMX390_H_REV_MASK, 1);
		if (ret < 0)
			break;

		ret = imx390_update_bits(imx390, IMX390_REG_SM_CFG_REVERSE_APL,
					 ctrl->val, IMX390_H_REV_APL_MASK, 1);
		break;

	case V4L2_CID_VFLIP:
		ret = imx390_update_bits(imx390, IMX390_REG_CAT0_V_H_REVERSE,
					 ctrl->val, IMX390_V_REV_MASK, 1);
		if (ret < 0)
			break;

		ret = imx390_update_bits(imx390, IMX390_REG_SM_CFG_REVERSE_APL,
					 ctrl->val, IMX390_V_REV_APL_MASK, 1);
		break;

	case V4L2_CID_TEST_PATTERN:
		ret = imx390_write(imx390, IMX390_REG_CAT0_PGMODE_PGREGEN,
				   imx390_pg_mode_reg_val[ctrl->val], 1);
		if (ret < 0)
			break;

		ret = imx390_update_bits(imx390, IMX390_SM_CFG_SM_PGREGEN_APL,
					 ctrl->val, IMX390_SM_PG_APL_MASK, 1);
		break;

	default:
		ret = -EINVAL;
	}

	pm_runtime_put_noidle(imx390->dev);
	return ret;
}

static int imx390_detect(struct imx390 *imx390)
{
	int ret;
	u32 id;

	ret = imx390_read(imx390, IMX390_REG_VERSION_ROM_VERSION, &id, 2);
	if (ret < 0)
		return ret;

	if (id != IMX390_ROM_VERSION) {
		dev_err(imx390->dev,
			"%s: unknown chip ID 0x%04x\n", __func__, id);
		return -ENODEV;
	}

	dev_dbg(imx390->dev, "%s: detected chip ID 0x%04x\n", __func__, id);
	return 0;
}

static int imx390_power_on(struct imx390 *imx390)
{
	int ret;

	ret = clk_prepare_enable(imx390->clk);
	if (ret < 0)
		return ret;

	if (imx390->xclr_gpio) {
		gpiod_set_value_cansleep(imx390->xclr_gpio, 1);
		/* Keep the XCLR pin on Low for 100 us or longer */
		usleep_range(100, 1000);
		gpiod_set_value_cansleep(imx390->xclr_gpio, 0);
		/* It takes max 30 ms for the sensor to be ready */
		msleep(30);
	}
	return 0;
}

static void imx390_power_off(struct imx390 *imx390)
{
	if (imx390->xclr_gpio) {
		gpiod_set_value_cansleep(imx390->xclr_gpio, 1);
		/* Wait for the XCLR pin to be Low for atleast 1 us */
		usleep_range(1, 10);
	}

	clk_disable_unprepare(imx390->clk);
}

static int imx390_get_frame_interval(struct v4l2_subdev *sd,
				     struct v4l2_subdev_frame_interval *fi)
{
	struct imx390 *imx390 = to_imx390(sd);

	fi->interval.numerator = 1;
	fi->interval.denominator = imx390->fps;
	return 0;
}

static int imx390_set_frame_interval(struct v4l2_subdev *sd,
				     struct v4l2_subdev_frame_interval *fi)
{
	struct imx390 *imx390 = to_imx390(sd);
	struct v4l2_ctrl *ctrl = imx390->ctrl.exposure;
	u32 req_fps;
	int ret;

	mutex_lock(&imx390->lock);

	if (fi->interval.numerator == 0 || fi->interval.denominator == 0) {
		fi->interval.denominator = IMX390_FRAMERATE_DEFAULT;
		fi->interval.numerator = 1;
	}

	req_fps = clamp_val(DIV_ROUND_CLOSEST(fi->interval.denominator,
					      fi->interval.numerator),
			    IMX390_FRAMERATE_MIN, IMX390_FRAMERATE_MAX);

	fi->interval.numerator = 1;
	fi->interval.denominator = req_fps;

	imx390->fps = req_fps;

	ret = __v4l2_ctrl_modify_range(ctrl, 0, IMX390_EXPOSURE_MAX(req_fps), 1,
				       IMX390_EXPOSURE_DEFAULT);
	if (ret < 0) {
		dev_err(imx390->dev,
			"%s: exposure ctrl range update failed %d\n",
			__func__, ret);
	}

	mutex_unlock(&imx390->lock);
	dev_dbg(imx390->dev, "%s frame rate = %d\n", __func__, imx390->fps);

	return ret;
}

static int imx390_start_stream(struct imx390 *imx390)
{
	int ret;

	if (!imx390->ctrl.wdr->val &&
	    imx390->fps <= IMX390_FRAMERATE_MAX_LINEAR)
		ret = imx390_write_table(imx390, imx390_linear_1936x1096,
					 ARRAY_SIZE(imx390_linear_1936x1096));
	else
		ret = imx390_write_table(imx390, imx390_wdr_1936x1096,
					 ARRAY_SIZE(imx390_wdr_1936x1096));
	if (ret < 0)
		return ret;

	msleep(100);

	/* Restore the V4L2 control values into the registers */
	ret =  __v4l2_ctrl_handler_setup(imx390->subdev.ctrl_handler);
	if (ret < 0) {
		dev_err(imx390->dev,
			"%s: failed to apply v4l2 ctrls: %d\n", __func__, ret);
		return ret;
	}

	ret = imx390_write(imx390, IMX390_REG_MODE_HMAX,
			   (u32)IMX390_FPS_TO_MODE_HMAX(imx390->fps), 2);
	if (ret < 0)
		return ret;

	/* Set active */
	ret = imx390_write(imx390, IMX390_REG_CAT0_STANDBY, 0, 1);
	if (ret < 0)
		return ret;

	/* No communication is possible for a while after exiting standby */
	msleep(20);

	return 0;
}

static int imx390_stop_stream(struct imx390 *imx390)
{
	int ret;

	/* Set standby */
	ret = imx390_write(imx390, IMX390_REG_CAT0_STANDBY, 1, 1);
	if (ret < 0)
		return ret;

	/* No communication is possible for a while after entering standby */
	usleep_range(10000, 20000);
	return 0;
}

static int imx390_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct imx390 *imx390 = to_imx390(sd);
	int ret;

	mutex_lock(&imx390->lock);
	if (imx390->streaming == enable) {
		mutex_unlock(&imx390->lock);
		return 0;
	}

	if (enable) {
		ret = pm_runtime_get_sync(imx390->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(imx390->dev);
			goto err_unlock;
		}

		ret = imx390_start_stream(imx390);
		if (ret < 0)
			goto err_runtime_put;
	} else {
		ret = imx390_stop_stream(imx390);
		if (ret < 0)
			goto err_runtime_put;
		pm_runtime_mark_last_busy(imx390->dev);
		pm_runtime_put_autosuspend(imx390->dev);
	}

	imx390->streaming = enable;
	/* WDR, HFLIP, VFLIP, TEST PATTERN cannot change during streaming */
	__v4l2_ctrl_grab(imx390->ctrl.wdr, enable);
	__v4l2_ctrl_grab(imx390->ctrl.h_flip, enable);
	__v4l2_ctrl_grab(imx390->ctrl.v_flip, enable);
	__v4l2_ctrl_grab(imx390->ctrl.pg_mode, enable);

	mutex_unlock(&imx390->lock);
	return 0;

err_runtime_put:
	pm_runtime_put(imx390->dev);

err_unlock:
	mutex_unlock(&imx390->lock);
	dev_err(imx390->dev,
		"%s: failed to setup streaming %d\n", __func__, ret);
	return ret;
}

static const struct v4l2_subdev_video_ops imx390_subdev_video_ops = {
	.g_frame_interval = imx390_get_frame_interval,
	.s_frame_interval = imx390_set_frame_interval,
	.s_stream = imx390_set_stream,
};

static const struct v4l2_subdev_pad_ops imx390_subdev_pad_ops = {
	.init_cfg = imx390_init_cfg,
	.enum_mbus_code	= imx390_enum_mbus_code,
	.enum_frame_size = imx390_enum_frame_sizes,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = imx390_set_fmt,
	.set_routing = imx390_set_routing,
	.get_frame_desc	= imx390_get_frame_desc,
};

static const struct v4l2_subdev_ops imx390_subdev_ops = {
	.video	= &imx390_subdev_video_ops,
	.pad	= &imx390_subdev_pad_ops,
};

static const struct v4l2_ctrl_ops imx390_ctrl_ops = {
	.s_ctrl	= imx390_set_ctrl,
};

static int imx390_probe(struct i2c_client *client)
{
	struct imx390 *imx390;
	struct v4l2_subdev *sd;
	struct v4l2_ctrl_handler *ctrl_hdr;
	int ret;

	imx390 = devm_kzalloc(&client->dev, sizeof(*imx390), GFP_KERNEL);
	if (!imx390)
		return -ENOMEM;

	imx390->dev = &client->dev;

	imx390->regmap = devm_regmap_init_i2c(client, &imx390_regmap_config);
	if (IS_ERR(imx390->regmap))
		return PTR_ERR(imx390->regmap);

	imx390->xclr_gpio = devm_gpiod_get_optional(imx390->dev,
						    "xclr", GPIOD_OUT_LOW);
	if (IS_ERR(imx390->xclr_gpio))
		return PTR_ERR(imx390->xclr_gpio);

	imx390->clk = devm_clk_get(imx390->dev, "inck");
	if (IS_ERR(imx390->clk))
		return PTR_ERR(imx390->clk);

	imx390->clk_rate = clk_get_rate(imx390->clk);
	dev_info(imx390->dev, "inck rate: %lu Hz\n", imx390->clk_rate);

	if (imx390->clk_rate < 5900000 || imx390->clk_rate > 27100000)
		return -EINVAL;

	ret = imx390_power_on(imx390);
	if (ret < 0)
		return ret;

	ret = imx390_detect(imx390);
	if (ret < 0)
		return ret;

	/* Initialize the subdev and its controls. */
	sd = &imx390->subdev;
	v4l2_i2c_subdev_init(sd, client, &imx390_subdev_ops);

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS |
		     V4L2_SUBDEV_FL_MULTIPLEXED;

	/* Initialize the media entity. */
	imx390->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &imx390->pad);
	if (ret < 0) {
		dev_err(imx390->dev,
			"%s: media entity init failed %d\n", __func__, ret);
		return ret;
	}

	/* Initialize controls */
	ctrl_hdr = &imx390->ctrl.handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdr, 9);
	if (ret < 0) {
		dev_err(imx390->dev,
			"%s: ctrl handler init failed: %d\n", __func__, ret);
		goto err_media_cleanup;
	}

	mutex_init(&imx390->lock);
	imx390->ctrl.handler.lock = &imx390->lock;
	imx390->fps = IMX390_FRAMERATE_DEFAULT;

	/* Add new controls */
	imx390->ctrl.exposure = v4l2_ctrl_new_std(ctrl_hdr, &imx390_ctrl_ops,
						  V4L2_CID_EXPOSURE, 0,
						  IMX390_EXPOSURE_MAX(imx390->fps),
						  1, IMX390_EXPOSURE_DEFAULT);

	imx390->ctrl.again = v4l2_ctrl_new_std(ctrl_hdr, &imx390_ctrl_ops,
					       V4L2_CID_ANALOGUE_GAIN, 0,
					       IMX390_ANALOG_GAIN_MAX, 1,
					       IMX390_ANALOG_GAIN_DEFAULT);

	imx390->ctrl.dgain = v4l2_ctrl_new_std(ctrl_hdr, &imx390_ctrl_ops,
					       V4L2_CID_DIGITAL_GAIN, 0,
					       IMX390_DIGITAL_GAIN_MAX, 1,
					       IMX390_DIGITAL_GAIN_DEFAULT);

	imx390->ctrl.r_balance = v4l2_ctrl_new_std(ctrl_hdr, &imx390_ctrl_ops,
						   V4L2_CID_RED_BALANCE, 0,
						   IMX390_R_B_BALANCE_MAX, 1,
						   IMX390_R_B_BALANCE_DEFAULT);

	imx390->ctrl.b_balance = v4l2_ctrl_new_std(ctrl_hdr, &imx390_ctrl_ops,
						   V4L2_CID_BLUE_BALANCE, 0,
						   IMX390_R_B_BALANCE_MAX, 1,
						   IMX390_R_B_BALANCE_DEFAULT);

	imx390->ctrl.wdr = v4l2_ctrl_new_std(ctrl_hdr, &imx390_ctrl_ops,
					     V4L2_CID_WIDE_DYNAMIC_RANGE,
					     0, 1, 1, 1);

	imx390->ctrl.h_flip = v4l2_ctrl_new_std(ctrl_hdr, &imx390_ctrl_ops,
						V4L2_CID_HFLIP, 0, 1, 1, 0);

	imx390->ctrl.v_flip = v4l2_ctrl_new_std(ctrl_hdr, &imx390_ctrl_ops,
						V4L2_CID_VFLIP, 0, 1, 1, 0);

	imx390->ctrl.pg_mode = v4l2_ctrl_new_std_menu_items(ctrl_hdr,
					&imx390_ctrl_ops, V4L2_CID_TEST_PATTERN,
					ARRAY_SIZE(imx390_ctrl_pg_qmenu) - 1,
					0, 0, imx390_ctrl_pg_qmenu);

	imx390->subdev.ctrl_handler = ctrl_hdr;
	if (imx390->ctrl.handler.error) {
		ret = imx390->ctrl.handler.error;
		dev_err(imx390->dev,
			"%s: failed to add the ctrls: %d\n", __func__, ret);
		goto err_ctrl_free;
	}

	pm_runtime_set_active(imx390->dev);
	pm_runtime_enable(imx390->dev);
	pm_runtime_set_autosuspend_delay(imx390->dev, IMX390_PM_IDLE_TIMEOUT);
	pm_runtime_use_autosuspend(imx390->dev);
	pm_runtime_get_noresume(imx390->dev);

	ret = v4l2_subdev_init_finalize(sd);
	if (ret < 0)
		goto err_pm_disable;

	/* Finally, register the subdev. */
	ret = v4l2_async_register_subdev(sd);
	if (ret < 0) {
		dev_err(imx390->dev,
			"%s: v4l2 subdev register failed %d\n", __func__, ret);
		goto err_subdev_cleanup;
	}

	dev_info(imx390->dev, "imx390 probed\n");
	pm_runtime_mark_last_busy(imx390->dev);
	pm_runtime_put_autosuspend(imx390->dev);
	return 0;

err_subdev_cleanup:
	v4l2_subdev_cleanup(&imx390->subdev);

err_pm_disable:
	pm_runtime_dont_use_autosuspend(imx390->dev);
	pm_runtime_put_noidle(imx390->dev);
	pm_runtime_disable(imx390->dev);

err_ctrl_free:
	v4l2_ctrl_handler_free(ctrl_hdr);
	mutex_destroy(&imx390->lock);

err_media_cleanup:
	media_entity_cleanup(&imx390->subdev.entity);

	return ret;
}

static int __maybe_unused imx390_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx390 *imx390 = to_imx390(sd);
	int ret;

	ret = imx390_power_on(imx390);
	if (ret < 0)
		return ret;

	return 0;
}

static int __maybe_unused imx390_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx390 *imx390 = to_imx390(sd);

	imx390_power_off(imx390);

	return 0;
}

static int __maybe_unused imx390_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx390 *imx390 = to_imx390(sd);
	int ret;

	if (imx390->streaming)
		imx390_stop_stream(imx390);

	ret = pm_runtime_force_suspend(dev);
	if (ret < 0)
		dev_warn(dev, "%s: failed to suspend: %i\n", __func__, ret);

	return 0;
}

static int __maybe_unused imx390_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx390 *imx390 = to_imx390(sd);
	int ret;

	ret = pm_runtime_force_resume(dev);
	if (ret < 0)
		dev_warn(dev, "%s: failed to resume: %i\n", __func__, ret);

	if (imx390->streaming)
		ret = imx390_start_stream(imx390);

	if (ret < 0) {
		imx390_stop_stream(imx390);
		imx390->streaming = 0;
		return ret;
	}

	return 0;
}

static int imx390_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx390 *imx390 = to_imx390(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&imx390->ctrl.handler);
	v4l2_subdev_cleanup(&imx390->subdev);
	media_entity_cleanup(&sd->entity);
	mutex_destroy(&imx390->lock);

	pm_runtime_disable(imx390->dev);
	pm_runtime_dont_use_autosuspend(imx390->dev);
	pm_runtime_set_suspended(imx390->dev);

	return 0;
}

static const struct dev_pm_ops imx390_pm_ops = {
	SET_RUNTIME_PM_OPS(imx390_runtime_suspend,
			   imx390_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(imx390_suspend, imx390_resume)
};

static const struct of_device_id imx390_dt_id[] = {
	{ .compatible = "sony,imx390" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, imx390_dt_id);

static struct i2c_driver imx390_i2c_driver = {
	.driver = {
		.name = "imx390",
		.of_match_table = of_match_ptr(imx390_dt_id),
		.pm = &imx390_pm_ops,
	},
	.probe_new = imx390_probe,
	.remove = imx390_remove,
};

module_i2c_driver(imx390_i2c_driver);

MODULE_DESCRIPTION("Camera Sensor Driver for Sony IMX390");
MODULE_AUTHOR("Apurva Nandan <a-nandan@ti.com>");
MODULE_AUTHOR("Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>");
MODULE_LICENSE("GPL v2");
