// SPDX-License-Identifier: GPL-2.0
/*
 * Omnivision OV2312 RGB-IR Image Sensor driver
 *
 * Copyright (c) 2022 Jai Luthra <j-luthra@ti.com>
 */

#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/v4l2-mediabus.h>
#include <linux/videodev2.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>

#include "ov2312.h"

struct ov2312 {
	struct device *dev;

	struct clk *clk;
	unsigned long clk_rate;

	struct i2c_client *client;
	struct regmap *regmap;
	struct gpio_desc *reset_gpio;

	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *again;
	struct v4l2_ctrl *dgain;
	struct v4l2_ctrl *h_flip;
	struct v4l2_ctrl *v_flip;

	u32 fps;

	struct mutex lock;
	bool streaming;
};

static inline struct ov2312 *to_ov2312(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov2312, sd);
}

static int ov2312_read(struct ov2312 *ov2312, u16 addr, u32 *val, size_t nbytes)
{
	int ret;
	__le32 val_le = 0;

	ret = regmap_bulk_read(ov2312->regmap, addr, &val_le, nbytes);
	if (ret < 0) {
		dev_err(ov2312->dev, "%s: failed to read reg 0x%04x: %d\n",
			__func__, addr, ret);
		return ret;
	}

	*val = le32_to_cpu(val_le);
	return 0;
}

static int ov2312_write(struct ov2312 *ov2312, u16 addr, u32 val, size_t nbytes)
{
	int ret;
	__le32 val_le = cpu_to_le32(val);

	ret = regmap_bulk_write(ov2312->regmap, addr, &val_le, nbytes);
	if (ret < 0)
		dev_err(ov2312->dev, "%s: failed to write reg 0x%04x: %d\n",
			__func__, addr, ret);
	return ret;
}

static int ov2312_write_table(struct ov2312 *ov2312,
			      const struct reg_sequence *regs,
			      unsigned int nr_regs)
{
	int ret, i;

	for (i = 0; i < nr_regs; i++) {
		ret = regmap_write(ov2312->regmap, regs[i].reg, regs[i].def);
		if (ret < 0) {
			dev_err(ov2312->dev,
				"%s: failed to write reg[%d] 0x%04x = 0x%02x (%d)!\n",
				__func__, i, regs[i].reg, regs[i].def, ret);
			return ret;
		}
	}
	return 0;
}

static void ov2312_init_formats(struct v4l2_subdev_state *state)
{
	struct v4l2_mbus_framefmt *format;
	int i;

	for (i = 0; i < 2; ++i) {
		format = v4l2_state_get_stream_format(state, 0, i);
		format->code = ov2312_mbus_formats[0];
		format->width = ov2312_framesizes[0].width;
		format->height = ov2312_framesizes[0].height;
		format->field = V4L2_FIELD_NONE;
		format->colorspace = V4L2_COLORSPACE_DEFAULT;
	}
}


static int ov2312_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	struct ov2312 *ov2312 = to_ov2312(sd);
	struct v4l2_mbus_framefmt *format;
	const struct v4l2_area *fsize;
	u32 code;
	int ret = 0;

	if (fmt->pad != 0)
		return -EINVAL;

	if (fmt->stream != 0)
		return -EINVAL;

	/* Sensor only supports a single format. */
	code = ov2312_mbus_formats[0];

	/* Find the nearest supported frame size. */
	fsize = v4l2_find_nearest_size(ov2312_framesizes,
				       ARRAY_SIZE(ov2312_framesizes), width,
				       height, fmt->format.width,
				       fmt->format.height);

	v4l2_subdev_lock_state(state);

	/* Update the stored format and return it. */
	format = v4l2_state_get_stream_format(state, fmt->pad, fmt->stream);

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE && ov2312->streaming) {
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

static int _ov2312_set_routing(struct v4l2_subdev *sd,
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
				 V4L2_SUBDEV_ROUTE_FL_SOURCE |
				 V4L2_SUBDEV_ROUTE_FL_ACTIVE,
		},
	};

	struct v4l2_subdev_krouting routing = {
		.num_routes = ARRAY_SIZE(routes),
		.routes = routes,
	};

	int ret;

	ret = v4l2_subdev_set_routing(sd, state, &routing);
	if (ret < 0)
		return ret;

	ov2312_init_formats(state);

	return 0;
}

static int ov2312_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				 struct v4l2_mbus_frame_desc *fd)
{
	struct v4l2_subdev_state *state;
	struct v4l2_mbus_framefmt *fmt;
	const struct v4l2_subdev_krouting *routing;
	u32 bpp;
	int ret = 0;
	unsigned int i;

	if (pad != 0)
		return -EINVAL;

	state = v4l2_subdev_lock_active_state(sd);
	routing = &state->routing;

	fmt = v4l2_state_get_stream_format(state, 0, 0);
	if (!fmt) {
		ret = -EPIPE;
		goto out;
	}

	memset(fd, 0, sizeof(*fd));

	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;

	/* pixel stream - 2 virtual channels */

	bpp = 10;

	for (i = 0; i < 2; ++i) {
		fd->entry[fd->num_entries].stream = i;

		fd->entry[fd->num_entries].flags = V4L2_MBUS_FRAME_DESC_FL_LEN_MAX;
		fd->entry[fd->num_entries].length = fmt->width * fmt->height * bpp / 8;
		fd->entry[fd->num_entries].pixelcode = fmt->code;
		fd->entry[fd->num_entries].bus.csi2.vc = i;
		fd->entry[fd->num_entries].bus.csi2.dt = 0x2b; /* RAW10 */

		fd->num_entries++;
	}

out:
	v4l2_subdev_unlock_state(state);

	return ret;
}

static int ov2312_set_routing(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state,
			      enum v4l2_subdev_format_whence which,
			      struct v4l2_subdev_krouting *routing)
{
	int ret;

	if (routing->num_routes == 0 || routing->num_routes > 2)
		return -EINVAL;

	v4l2_subdev_lock_state(state);

	ret = _ov2312_set_routing(sd, state);

	v4l2_subdev_unlock_state(state);

	return ret;
}


static int ov2312_init_cfg(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *state)
{
	int ret;

	v4l2_subdev_lock_state(state);

	ret = _ov2312_set_routing(sd, state);

	v4l2_subdev_unlock_state(state);

	return ret;
}

static int ov2312_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(ov2312_mbus_formats))
		return -EINVAL;

	code->code = ov2312_mbus_formats[code->index];

	return 0;
}

static int ov2312_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ov2312_mbus_formats); ++i) {
		if (ov2312_mbus_formats[i] == fse->code)
			break;
	}

	if (i == ARRAY_SIZE(ov2312_mbus_formats))
		return -EINVAL;

	if (fse->index >= ARRAY_SIZE(ov2312_framesizes))
		return -EINVAL;

	fse->min_width = ov2312_framesizes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->max_height = ov2312_framesizes[fse->index].height;
	fse->min_height = fse->max_height;

	return 0;
}

static int ov2312_get_frame_interval(struct v4l2_subdev *sd,
				     struct v4l2_subdev_frame_interval *fi)
{
	struct ov2312 *ov2312 = to_ov2312(sd);

	fi->interval.numerator = 1;
	fi->interval.denominator = ov2312->fps/2;

	return 0;
}

static int ov2312_set_frame_interval(struct v4l2_subdev *sd,
				     struct v4l2_subdev_frame_interval *fi)
{
	struct ov2312 *ov2312 = to_ov2312(sd);

	dev_dbg(ov2312->dev, "%s: Set framerate %dfps\n", __func__,
		 fi->interval.denominator/fi->interval.numerator);
	if (fi->interval.denominator/fi->interval.numerator != ov2312->fps) {
		dev_err(ov2312->dev, "%s: Framerate can only be %dfps\n",
			__func__, ov2312->fps);
		return -EINVAL;
	}
	return 0;
}

static int ov2312_detect(struct ov2312 *ov2312)
{
	int ret;
	u32 id;

	ret = ov2312_read(ov2312, OV2312_SC_CHIP_ID_HI, &id, 2);
	if (ret < 0)
		return ret;

	id = cpu_to_be16(id);

	if (id != OV2312_CHIP_ID) {
		dev_err(ov2312->dev,
			"%s: unknown chip ID 0x%04x\n", __func__, id);
		return -ENODEV;
	}

	dev_dbg(ov2312->dev, "%s: detected chip ID 0x%04x\n", __func__, id);
	return 0;
}

static int ov2312_set_AB_mode(struct ov2312 *ov2312)
{
	int i, ret;
	u32 exposure = ov2312->exposure->val;
	u32 again = ov2312->again->val;
	u32 dgain = ov2312->dgain->val;
	struct reg_sequence ov2312_groupB[] = {
		{0x3208, 0x00},/* Group B (RGB Dominant VC1) */
		{OV2312_AEC_PK_EXPO_HI, (exposure >> 8) & 0xff},
		{OV2312_AEC_PK_EXPO_LO, exposure & 0xff},
		{OV2312_AEC_PK_AGAIN_HI, (again >> 4) & 0xff},
		{OV2312_AEC_PK_AGAIN_LO, (again & 0x0f) << 4},
		{OV2312_AEC_PK_DGAIN_HI, (dgain >> 8) & 0xff},
		{OV2312_AEC_PK_DGAIN_LO, dgain & 0xff},
		{0x3920, 0x00},
		{0x4813, 0x00},/* VC=0. This register takes effect from next frame */
		{0x3208, 0x10},
		{0x320D, 0x00},/* Auto mode switch between group0 and group1 ;setting to switch */
		{0x320D, 0x31},
		{0x3208, 0xA0},
	};

	for (i = 0; i < ARRAY_SIZE(ov2312_groupB); i++) {
		ret = regmap_write(ov2312->regmap, ov2312_groupB[i].reg, ov2312_groupB[i].def);
		if (ret < 0) {
			dev_err(ov2312->dev,
				"%s: failed to write reg[%d] 0x%04x = 0x%02x (%d)!\n",
				__func__, i, ov2312_groupB[i].reg, ov2312_groupB[i].def, ret);
			return ret;
		}
	}

	msleep(33);
	return 0;
}

static int ov2312_set_orientation(struct ov2312 *ov2312)
{
	bool v_flip = ov2312->v_flip->val;
	bool h_flip = ov2312->h_flip->val;
	u32 reg = (v_flip ? 0x4400 : 0) | (h_flip ? 0x0004 : 0);

	return ov2312_write(ov2312, OV2312_TIMING_VFLIP, be16_to_cpu(reg), 2);
}

static int ov2312_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov2312 *ov2312 = container_of(ctrl->handler,
					     struct ov2312, ctrls);
	int ret;

	dev_dbg(ov2312->dev, "%s: %s, value: %d\n", __func__,
		ctrl->name, ctrl->val);

	/*
	 * If the device is not powered up by the host driver do
	 * not apply any controls to H/W at this time. Instead
	 * the controls will be restored right after power-up.
	 */
	if (pm_runtime_suspended(ov2312->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
	case V4L2_CID_ANALOGUE_GAIN:
	case V4L2_CID_DIGITAL_GAIN:
		ret = ov2312_set_AB_mode(ov2312);
		break;

	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		ret = ov2312_set_orientation(ov2312);
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

static int ov2312_power_on(struct ov2312 *ov2312)
{
	int ret;
	ret = clk_prepare_enable(ov2312->clk);
	if (ret < 0)
		return ret;

	if (ov2312->reset_gpio) {
		gpiod_set_value_cansleep(ov2312->reset_gpio, 0);
		msleep(10);
		gpiod_set_value_cansleep(ov2312->reset_gpio, 1);
		msleep(30);
	}
	return 0;
}

static int ov2312_power_off(struct ov2312 *ov2312)
{
	if (ov2312->reset_gpio) {
		gpiod_set_value_cansleep(ov2312->reset_gpio, 0);
		msleep(10);
	}

	clk_disable_unprepare(ov2312->clk);

	return 0;
}

static int ov2312_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov2312 *ov2312 = to_ov2312(sd);

	return ov2312_power_on(ov2312);
}

static int ov2312_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov2312 *ov2312 = to_ov2312(sd);

	return ov2312_power_off(ov2312);
}

static int ov2312_start_stream(struct ov2312 *ov2312)
{
	int ret;
	ret = ov2312_write_table(ov2312, ov2312_1600x1300_60fps_AB,
				 ARRAY_SIZE(ov2312_1600x1300_60fps_AB));
	if (ret < 0)
		return ret;

	/* Update controls on wake up */
	ret = ov2312_set_orientation(ov2312);
	if (ret < 0)
		return ret;

	ret = ov2312_set_AB_mode(ov2312);
	if (ret < 0)
		return ret;

	msleep(100);

	/* Set active */
	ret = ov2312_write(ov2312, OV2312_SYS_MODE_SEL, 1, 1);
	if (ret < 0)
		return ret;

	/* No communication is possible for a while after exiting standby */
	msleep(20);
	return 0;
}

static int ov2312_stop_stream(struct ov2312 *ov2312)
{
	int ret;

	/* Set standby */
	ret = ov2312_write(ov2312, OV2312_SYS_MODE_SEL, 0, 1);
	if (ret < 0)
		return ret;

	/* No communication is possible for a while after entering standby */
	usleep_range(10000, 20000);
	return 0;
}

static int ov2312_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct ov2312 *ov2312 = to_ov2312(sd);
	int ret;

	mutex_lock(&ov2312->lock);
	if (ov2312->streaming == enable) {
		mutex_unlock(&ov2312->lock);
		return 0;
	}

	if (enable) {
		ret = pm_runtime_resume_and_get(ov2312->dev);
		if (ret < 0) {
			goto err_unlock;
		}

		ret = ov2312_start_stream(ov2312);
		if (ret < 0)
			goto err_runtime_put;
	} else {
		ret = ov2312_stop_stream(ov2312);
		if (ret < 0)
			goto err_runtime_put;
		pm_runtime_put(ov2312->dev);
	}

	ov2312->streaming = enable;
	mutex_unlock(&ov2312->lock);
	return 0;

err_runtime_put:
	pm_runtime_put(ov2312->dev);

err_unlock:
	mutex_unlock(&ov2312->lock);
	dev_err(ov2312->dev,
		"%s: failed to setup streaming %d\n", __func__, ret);
	return ret;
}

static struct v4l2_subdev_video_ops ov2312_subdev_video_ops = {
	.s_stream = ov2312_set_stream,
	.g_frame_interval = ov2312_get_frame_interval,
	.s_frame_interval = ov2312_set_frame_interval,
};

static struct v4l2_subdev_pad_ops ov2312_subdev_pad_ops = {
	.init_cfg = ov2312_init_cfg,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = ov2312_set_fmt,
	.enum_mbus_code = ov2312_enum_mbus_code,
	.enum_frame_size = ov2312_enum_frame_sizes,
	.set_routing = ov2312_set_routing,
	.get_frame_desc	= ov2312_get_frame_desc,
};

static struct v4l2_subdev_ops ov2312_subdev_ops = {
	.video	= &ov2312_subdev_video_ops,
	.pad	= &ov2312_subdev_pad_ops,
};

static const struct v4l2_ctrl_ops ov2312_ctrl_ops = {
	.s_ctrl	= ov2312_set_ctrl,
};

static const struct dev_pm_ops ov2312_pm_ops = {
	SET_RUNTIME_PM_OPS(ov2312_suspend, ov2312_resume, NULL)
};

static int ov2312_probe(struct i2c_client *client)
{
	struct ov2312 *ov2312;
	struct v4l2_subdev *sd;
	struct v4l2_ctrl_handler *ctrl_hdr;
	int ret;

	/* Allocate internal struct */
	ov2312 = devm_kzalloc(&client->dev, sizeof(*ov2312), GFP_KERNEL);
	if (!ov2312)
		return -ENOMEM;

	ov2312->dev = &client->dev;
	ov2312->client = client;

	/* Initialize I2C Regmap */
	ov2312->regmap = devm_regmap_init_i2c(client, &ov2312_regmap_config);
	if (IS_ERR(ov2312->regmap))
		return PTR_ERR(ov2312->regmap);

	/* Initialize Shutdown GPIO */
	ov2312->reset_gpio = devm_gpiod_get_optional(ov2312->dev,
							 "reset",
							 GPIOD_OUT_HIGH);
	if (IS_ERR(ov2312->reset_gpio))
		return PTR_ERR(ov2312->reset_gpio);

	ov2312->clk = devm_clk_get(ov2312->dev, "xvclk");
	if (IS_ERR(ov2312->clk))
		return PTR_ERR(ov2312->clk);

	ov2312->clk_rate = clk_get_rate(ov2312->clk);
	dev_info(ov2312->dev, "xvclk rate: %lu Hz\n", ov2312->clk_rate);

	if (ov2312->clk_rate < 6000000 || ov2312->clk_rate > 27000000)
		return -EINVAL;

	/* Power on */
	ret = ov2312_power_on(ov2312);
	if (ret < 0)
		return ret;

	/* Detect sensor */
	ret = ov2312_detect(ov2312);
	if (ret < 0)
		return ret;

	/* Initialize the subdev and its controls. */
	sd = &ov2312->sd;
	v4l2_i2c_subdev_init(sd, client, &ov2312_subdev_ops);

	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS | V4L2_SUBDEV_FL_MULTIPLEXED;

	/* Initialize the media entity. */
	ov2312->pad.flags = MEDIA_PAD_FL_SOURCE;
	ov2312->pad.stream_count = 2;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &ov2312->pad);
	if (ret < 0) {
		dev_err(ov2312->dev,
			"%s: media entity init failed %d\n", __func__, ret);
		return ret;
	}

	ov2312->fps = OV2312_FRAMERATE_DEFAULT;
	mutex_init(&ov2312->lock);

	/* Initialize controls */
	ctrl_hdr = &ov2312->ctrls;
	ret = v4l2_ctrl_handler_init(ctrl_hdr, 5);
	if (ret < 0) {
		dev_err(ov2312->dev,
			"%s: ctrl handler init failed: %d\n", __func__, ret);
		goto err_media_cleanup;
	}

	ov2312->ctrls.lock = &ov2312->lock;

	/* Add new controls */
	ov2312->exposure = v4l2_ctrl_new_std(ctrl_hdr, &ov2312_ctrl_ops,
					     V4L2_CID_EXPOSURE, 1,
					     OV2312_EXPOSURE_MAX,
					     1, OV2312_EXPOSURE_DEFAULT);

	ov2312->again = v4l2_ctrl_new_std(ctrl_hdr, &ov2312_ctrl_ops,
					  V4L2_CID_ANALOGUE_GAIN, 0,
					  OV2312_AGAIN_MAX, 1,
					  OV2312_AGAIN_DEFAULT);

	ov2312->dgain = v4l2_ctrl_new_std(ctrl_hdr, &ov2312_ctrl_ops,
					  V4L2_CID_DIGITAL_GAIN, 0,
					  OV2312_DGAIN_MAX, 1,
					  OV2312_DGAIN_DEFAULT);

	ov2312->h_flip = v4l2_ctrl_new_std(ctrl_hdr, &ov2312_ctrl_ops,
					   V4L2_CID_HFLIP, 0, 1, 1, 0);

	ov2312->v_flip = v4l2_ctrl_new_std(ctrl_hdr, &ov2312_ctrl_ops,
					   V4L2_CID_VFLIP, 0, 1, 1, 0);

	ov2312->sd.ctrl_handler = ctrl_hdr;
	if (ov2312->ctrls.error) {
		ret = ov2312->ctrls.error;
		dev_err(ov2312->dev,
			"%s: failed to add the ctrls: %d\n", __func__, ret);
		goto err_ctrl_free;
	}

	/* PM Runtime */
	pm_runtime_enable(ov2312->dev);
	pm_runtime_set_suspended(ov2312->dev);

	ret = v4l2_subdev_init_finalize(sd);
	if (ret < 0)
		goto err_pm_disable;

	/* Finally, register the subdev. */
	ret = v4l2_async_register_subdev(sd);
	if (ret < 0) {
		dev_err(ov2312->dev,
			"%s: v4l2 subdev register failed %d\n", __func__, ret);
		goto err_subdev_cleanup;
	}

	dev_info(ov2312->dev, "ov2312 probed\n");
	return 0;

err_subdev_cleanup:
	v4l2_subdev_cleanup(&ov2312->sd);

err_pm_disable:
	pm_runtime_disable(ov2312->dev);

err_ctrl_free:
	v4l2_ctrl_handler_free(ctrl_hdr);
	mutex_destroy(&ov2312->lock);

err_media_cleanup:
	media_entity_cleanup(&ov2312->sd.entity);

	return ret;
}

static int ov2312_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov2312 *ov2312 = to_ov2312(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&ov2312->ctrls);
	v4l2_subdev_cleanup(&ov2312->sd);
	media_entity_cleanup(&sd->entity);
	mutex_destroy(&ov2312->lock);

	pm_runtime_disable(ov2312->dev);

	return 0;
}

static const struct i2c_device_id ov2312_id[] = {
	{ "ov2312", 0 },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(i2c, ov2312_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id ov2312_of_match[] = {
	{ .compatible = "ovti,ov2312", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, ov2312_of_match);
#endif

static struct i2c_driver ov2312_i2c_driver = {
	.driver = {
		.name	= "ov2312",
		.pm	= &ov2312_pm_ops,
		.of_match_table = of_match_ptr(ov2312_of_match),
	},
	.probe_new	= ov2312_probe,
	.remove		= ov2312_remove,
	.id_table	= ov2312_id,
};

module_i2c_driver(ov2312_i2c_driver);

MODULE_AUTHOR("Jai Luthra <j-luthra@ti.com>");
MODULE_DESCRIPTION("OV2312 RGB-IR Image Sensor driver");
MODULE_LICENSE("GPL v2");
