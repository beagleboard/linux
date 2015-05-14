/*
 * OmniVision OV1063X Camera Driver
 *
 * Based on the original driver written by Phil Edworthy.
 * Copyright (C) 2013 Phil Edworthy
 * Copyright (C) 2013 Renesas Electronics
 *
 * This driver has been tested at QVGA, VGA and 720p, and 1280x800 at up to
 * 30fps and it should work at any resolution in between and any frame rate
 * up to 30fps.
 *
 * FIXME:
 *  Horizontal flip (mirroring) does not work correctly. The image is flipped,
 *  but the colors are wrong.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/v4l2-mediabus.h>
#include <linux/videodev2.h>

#include <media/soc_camera.h>
#include <media/v4l2-async.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>

#include "ov1063x_regs.h"

#include <linux/pm_runtime.h>

#include <linux/of_gpio.h>
#include <linux/of_device.h>


/* Register definitions */
#define	OV1063X_VFLIP			0x381c
#define	 OV1063X_VFLIP_ON		(0x3 << 6)
#define	 OV1063X_VFLIP_SUBSAMPLE	0x1
#define	OV1063X_HMIRROR			0x381d
#define	 OV1063X_HMIRROR_ON		0x3
#define OV1063X_PID			0x300a
#define OV1063X_VER			0x300b

#define OV1063X_FORMAT_CTRL00		0x4300
#define   OV1063X_FORMAT_YUYV		0x38
#define   OV1063X_FORMAT_YYYU		0x39
#define   OV1063X_FORMAT_UYVY		0x3A
#define   OV1063X_FORMAT_VYUY		0x3B


/* IDs */
#define OV10633_VERSION_REG		0xa630
#define OV10635_VERSION_REG		0xa635
#define OV1063X_VERSION(pid, ver)	(((pid) << 8) | ((ver) & 0xff))

enum ov1063x_model {
	SENSOR_OV10633,
	SENSOR_OV10635,
};

#define OV1063X_SENSOR_WIDTH		1312
#define OV1063X_SENSOR_HEIGHT		814

#define OV1063X_MAX_WIDTH		1280
#define OV1063X_MAX_HEIGHT		800

#define MAX_NUM_GPIOS			20

struct ov1063x_color_format {
	enum v4l2_mbus_pixelcode code;
	enum v4l2_colorspace colorspace;
};

struct ov1063x_priv {
	struct v4l2_subdev		subdev;
	struct v4l2_async_subdev	asd;
	struct v4l2_ctrl_handler	hdl;
	int				model;
	int				revision;
	int				xvclk;
	int				fps_numerator;
	int				fps_denominator;
	const struct ov1063x_color_format	*cfmt;
	int				width;
	int				height;

	struct gpio			mux_gpios[MAX_NUM_GPIOS];
	int				num_gpios;
};

static int ov1063x_init_gpios(struct i2c_client *client);

/*
 * supported color format list
 */
static const struct ov1063x_color_format ov1063x_cfmts[] = {
	{
		.code		= V4L2_MBUS_FMT_YUYV8_2X8,
		.colorspace	= V4L2_COLORSPACE_SMPTE170M,
	},
	{
		.code		= V4L2_MBUS_FMT_UYVY8_2X8,
		.colorspace	= V4L2_COLORSPACE_SMPTE170M,
	},
	{
		.code		= V4L2_MBUS_FMT_VYUY8_2X8,
		.colorspace	= V4L2_COLORSPACE_SMPTE170M,
	},
	{
		.code		= V4L2_MBUS_FMT_YVYU8_2X8,
		.colorspace	= V4L2_COLORSPACE_SMPTE170M,
	},
	{
		.code		= V4L2_MBUS_FMT_YUYV10_2X10,
		.colorspace	= V4L2_COLORSPACE_SMPTE170M,
	},
};

static struct ov1063x_priv *to_ov1063x(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct ov1063x_priv,
			subdev);
}

/* read a register */
static int ov1063x_reg_read(struct i2c_client *client, u16 reg, u8 *val)
{
	int ret;
	u8 data[2] = { reg >> 8, reg & 0xff };
	struct i2c_msg msg = {
		.addr	= client->addr,
		.flags	= 0,
		.len	= 2,
		.buf	= data,
	};

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
		goto err;

	msg.flags = I2C_M_RD;
	msg.len	= 1;
	msg.buf	= &data[0];

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
		goto err;

	*val = data[0];
	return 0;
err:
	dev_err(&client->dev, "Failed reading register 0x%04x!\n", reg);
	return ret;
}

/* write a register */
static int ov1063x_reg_write(struct i2c_client *client, u16 reg, u8 val)
{
	int ret;
	u8 data[3] = { reg >> 8, reg & 0xff, val };

	struct i2c_msg msg = {
		.addr	= client->addr,
		.flags	= 0,
		.len	= 3,
		.buf	= data,
	};

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "Failed writing register 0x%04x!\n", reg);
		return ret;
	}

	return 0;
}

static int ov1063x_reg_write16(struct i2c_client *client, u16 reg, u16 val)
{
	int ret;

	ret = ov1063x_reg_write(client, reg, val >> 8);
	if (ret)
		return ret;

	ret = ov1063x_reg_write(client, reg + 1, val & 0xff);
	if (ret)
		return ret;

	return 0;
}

/* Read a register, alter its bits, write it back */
static int ov1063x_reg_rmw(struct i2c_client *client, u16 reg, u8 set, u8 unset)
{
	u8 val;
	int ret;

	ret = ov1063x_reg_read(client, reg, &val);
	if (ret) {
		dev_err(&client->dev,
			"[Read]-Modify-Write of register %04x failed!\n", reg);
		return ret;
	}

	val |= set;
	val &= ~unset;

	ret = ov1063x_reg_write(client, reg, val);
	if (ret)
		dev_err(&client->dev,
			"Read-Modify-[Write] of register %04x failed!\n", reg);

	return ret;
}


/* Start/Stop streaming from the device */
static int ov1063x_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	ret = ov1063x_init_gpios(client);
	if (ret) {
		dev_err(&client->dev, "Failed to request gpios");
		return ret;
	}

	ov1063x_reg_write(client, 0x0100, enable);

	if (enable)
		ov1063x_reg_write(client, 0x301c, 0xf0);
	else
		ov1063x_reg_write(client, 0x301c, 0x70);

	return 0;
}

/* Set status of additional camera capabilities */
static int ov1063x_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov1063x_priv *priv = container_of(ctrl->handler,
					struct ov1063x_priv, hdl);
	struct i2c_client *client = v4l2_get_subdevdata(&priv->subdev);

	switch (ctrl->id) {
	case V4L2_CID_VFLIP:
		if (ctrl->val)
			return ov1063x_reg_rmw(client, OV1063X_VFLIP,
				OV1063X_VFLIP_ON, 0);
		else
			return ov1063x_reg_rmw(client, OV1063X_VFLIP,
				0, OV1063X_VFLIP_ON);
		break;
	case V4L2_CID_HFLIP:
		if (ctrl->val)
			return ov1063x_reg_rmw(client, OV1063X_HMIRROR,
				OV1063X_HMIRROR_ON, 0);
		else
			return ov1063x_reg_rmw(client, OV1063X_HMIRROR,
				0, OV1063X_HMIRROR_ON);
		break;
	}

	return -EINVAL;
}

/*
 * Get the best pixel clock (pclk) that meets minimum hts/vts requirements.
 * xvclk => pre-divider => clk1 => multiplier => clk2 => post-divider => pclk
 * We try all valid combinations of settings for the 3 blocks to get the pixel
 * clock, and from that calculate the actual hts/vts to use. The vts is
 * extended so as to achieve the required frame rate. The function also returns
 * the PLL register contents needed to set the pixel clock.
 */
static int ov1063x_get_pclk(int xvclk, int *htsmin, int *vtsmin,
			int fps_numerator, int fps_denominator,
			u8 *r3003, u8 *r3004)
{
	int pre_divs[] = { 2, 3, 4, 6, 8, 10, 12, 14 };
	int pclk;
	int best_pclk = INT_MAX;
	int best_hts = 0;
	int i, j, k;
	int best_i = 0, best_j = 0, best_k = 0;
	int clk1, clk2;
	int hts;

	/* Pre-div, reg 0x3004, bits 6:4 */
	for (i = 0; i < ARRAY_SIZE(pre_divs); i++) {
		clk1 = (xvclk / pre_divs[i]) * 2;

		if ((clk1 < 3000000) || (clk1 > 27000000))
			continue;

		/* Mult = reg 0x3003, bits 5:0 */
		for (j = 1; j < 32; j++) {
			clk2 = (clk1 * j);

			if ((clk2 < 200000000) || (clk2 > 500000000))
				continue;

			/* Post-div, reg 0x3004, bits 2:0 */
			for (k = 0; k < 8; k++) {
				pclk = clk2 / (2*(k+1));

				if (pclk > 96000000)
					continue;

				hts = *htsmin + 200 + pclk/300000;

				/* 2 clock cycles for every YUV422 pixel */
				if (pclk < (((hts * *vtsmin)/fps_denominator)
					* fps_numerator * 2))
					continue;

				if (pclk < best_pclk) {
					best_pclk = pclk;
					best_hts = hts;
					best_i = i;
					best_j = j;
					best_k = k;
				}
			}
		}
	}

	/* register contents */
	*r3003 = (u8)best_j;
	*r3004 = ((u8)best_i << 4) | (u8)best_k;

	/* Did we get a valid PCLK? */
	if (best_pclk == INT_MAX)
		return -1;

	*htsmin = best_hts;

	/* Adjust vts to get as close to the desired frame rate as we can */
	*vtsmin = best_pclk / ((best_hts/fps_denominator) * fps_numerator * 2);

	return best_pclk;
}

static int ov1063x_set_regs(struct i2c_client *client,
	const struct ov1063x_reg *regs, int nr_regs)
{
	int i, ret;
	u8 val;

	for (i = 0; i < nr_regs; i++) {
		if (regs[i].reg == 0x300c) {
			val = ((client->addr * 2) | 0x1);

			ret = ov1063x_reg_write(client, regs[i].reg, val);
			if (ret)
				return ret;
		} else {
			ret = ov1063x_reg_write(client, regs[i].reg,
					regs[i].val);
			if (ret)
				return ret;
		}
	}

	return 0;
}

/* Setup registers according to resolution and color encoding */
static int ov1063x_set_params(struct i2c_client *client, u32 *width,
			u32 *height, enum v4l2_mbus_pixelcode code)
{
	struct ov1063x_priv *priv = to_ov1063x(client);
	int ret = -EINVAL;
	int pclk;
	int hts, vts;
	u8 r3003, r3004, r4300;
	int tmp;
	u32 height_pre_subsample;
	u32 width_pre_subsample;
	u8 horiz_crop_mode;
	int i;
	int nr_isp_pixels;
	int vert_sub_sample = 0;
	int horiz_sub_sample = 0;
	int sensor_width;

	if ((*width > OV1063X_MAX_WIDTH) || (*height > OV1063X_MAX_HEIGHT))
		return ret;

	/* select format */
	priv->cfmt = NULL;
	for (i = 0; i < ARRAY_SIZE(ov1063x_cfmts); i++) {
		if (code == ov1063x_cfmts[i].code) {
			priv->cfmt = ov1063x_cfmts + i;
			break;
		}
	}
	if (!priv->cfmt)
		goto ov1063x_set_fmt_error;

	priv->width = *width;
	priv->height = *height;

	/* Vertical sub-sampling? */
	height_pre_subsample = priv->height;
	if (priv->height <= 400) {
		vert_sub_sample = 1;
		height_pre_subsample <<= 1;
	}

	/* Horizontal sub-sampling? */
	width_pre_subsample = priv->width;
	if (priv->width <= 640) {
		horiz_sub_sample = 1;
		width_pre_subsample <<= 1;
	}

	/* Horizontal cropping */
	if (width_pre_subsample > 768) {
		sensor_width = OV1063X_SENSOR_WIDTH;
		horiz_crop_mode = 0x63;
	} else if (width_pre_subsample > 656) {
		sensor_width = 768;
		horiz_crop_mode = 0x6b;
	} else {
		sensor_width = 656;
		horiz_crop_mode = 0x73;
	}

	/* minimum values for hts and vts */
	hts = sensor_width;
	vts = height_pre_subsample + 50;
	dev_dbg(&client->dev, "fps=(%d/%d), hts=%d, vts=%d\n",
		priv->fps_numerator, priv->fps_denominator, hts, vts);

	/* Get the best PCLK & adjust hts,vts accordingly */
	pclk = ov1063x_get_pclk(priv->xvclk, &hts, &vts, priv->fps_numerator,
				priv->fps_denominator, &r3003, &r3004);
	if (pclk < 0)
		return ret;
	dev_dbg(&client->dev, "pclk=%d, hts=%d, vts=%d\n", pclk, hts, vts);
	dev_dbg(&client->dev, "r3003=0x%X r3004=0x%X\n", r3003, r3004);

	dev_err(&client->dev, "r3003 = %x , r3004 = %x\n", r3003, r3004);

	/* Disable ISP & program all registers that we might modify */
	ret = ov1063x_set_regs(client, ov1063x_regs_change_mode,
		ARRAY_SIZE(ov1063x_regs_change_mode));
	if (ret)
		return ret;

	/* Set to 1280x720 */
	ret = ov1063x_reg_write(client, 0x380f, 0x80);
	if (ret)
		return ret;

	/* Set PLL */
	ret = ov1063x_reg_write(client, 0x3003, r3003);
	if (ret)
		return ret;
	ret = ov1063x_reg_write(client, 0x3004, r3004);
	if (ret)
		return ret;

	/* Set HSYNC */
	ret = ov1063x_reg_write(client, 0x4700, 0x00);
	if (ret)
		return ret;

	switch (priv->cfmt->code) {
	case V4L2_MBUS_FMT_UYVY8_2X8:
		r4300 = OV1063X_FORMAT_UYVY;
		break;
	case V4L2_MBUS_FMT_VYUY8_2X8:
		r4300 = OV1063X_FORMAT_VYUY;
		break;
	case V4L2_MBUS_FMT_YUYV8_2X8:
		r4300 = OV1063X_FORMAT_YUYV;
		break;
	case V4L2_MBUS_FMT_YVYU8_2X8:
		r4300 = OV1063X_FORMAT_YYYU;
		break;
	default:
		r4300 = OV1063X_FORMAT_UYVY;
		break;
	}

	/* Set format to UYVY */
	ret = ov1063x_reg_write(client, OV1063X_FORMAT_CTRL00, r4300);
	if (ret)
		return ret;

	dev_dbg(&client->dev, "r4300=0x%X\n", r4300);

	/* Set output to 8-bit yuv */
	ret = ov1063x_reg_write(client, 0x4605, 0x08);
	if (ret)
		return ret;

	/* Horizontal cropping */
	ret = ov1063x_reg_write(client, 0x3621, horiz_crop_mode);
	if (ret)
		return ret;

	ret = ov1063x_reg_write(client, 0x3702, (pclk+1500000)/3000000);
	if (ret)
		return ret;
	ret = ov1063x_reg_write(client, 0x3703, (pclk+666666)/1333333);
	if (ret)
		return ret;
	ret = ov1063x_reg_write(client, 0x3704, (pclk+961500)/1923000);
	if (ret)
		return ret;

	/* Vertical cropping */
	tmp = ((OV1063X_SENSOR_HEIGHT - height_pre_subsample) / 2) & ~0x1;
	ret = ov1063x_reg_write16(client, 0x3802, tmp);
	if (ret)
		return ret;
	tmp = tmp + height_pre_subsample + 3;
	ret = ov1063x_reg_write16(client, 0x3806, tmp);
	if (ret)
		return ret;

	dev_err(&client->dev, "width x height = %x x %x\n",
		priv->width, priv->height);
	/* Output size */
	ret = ov1063x_reg_write16(client, 0x3808, priv->width);
	if (ret)
		return ret;
	ret = ov1063x_reg_write16(client, 0x380a, priv->height);
	if (ret)
		return ret;

	dev_err(&client->dev, "hts x vts = %x x %x\n", hts, vts);

	ret = ov1063x_reg_write16(client, 0x380c, hts);
	if (ret)
		return ret;

	ret = ov1063x_reg_write16(client, 0x380e, vts);
	if (ret)
		return ret;

	if (vert_sub_sample) {
		ret = ov1063x_reg_rmw(client, OV1063X_VFLIP,
					OV1063X_VFLIP_SUBSAMPLE, 0);
		if (ret)
			return ret;

		ret = ov1063x_set_regs(client, ov1063x_regs_vert_sub_sample,
			ARRAY_SIZE(ov1063x_regs_vert_sub_sample));
		if (ret)
			return ret;
	}

	ret = ov1063x_reg_write16(client, 0x4606, 2*hts);
	if (ret)
		return ret;
	ret = ov1063x_reg_write16(client, 0x460a, 2*(hts-width_pre_subsample));
	if (ret)
		return ret;

	tmp = (vts - 8) * 16;
	ret = ov1063x_reg_write16(client, 0xc488, tmp);
	if (ret)
		return ret;
	ret = ov1063x_reg_write16(client, 0xc48a, tmp);
	if (ret)
		return ret;

	nr_isp_pixels = sensor_width * (priv->height + 4);
	ret = ov1063x_reg_write16(client, 0xc4cc, nr_isp_pixels/256);
	if (ret)
		return ret;
	ret = ov1063x_reg_write16(client, 0xc4ce, nr_isp_pixels/256);
	if (ret)
		return ret;
	ret = ov1063x_reg_write16(client, 0xc512, nr_isp_pixels/16);
	if (ret)
		return ret;

	/* Horizontal sub-sampling */
	if (horiz_sub_sample) {
		ret = ov1063x_reg_write(client, 0x5005, 0x9);
		if (ret)
			return ret;

		ret = ov1063x_reg_write(client, 0x3007, 0x2);
		if (ret)
			return ret;
	}

	ret = ov1063x_reg_write16(client, 0xc518, vts);
	if (ret)
		return ret;
	ret = ov1063x_reg_write16(client, 0xc51a, hts);
	if (ret)
		return ret;

	/* Enable ISP blocks */
	ret = ov1063x_set_regs(client, ov1063x_regs_enable,
		ARRAY_SIZE(ov1063x_regs_enable));
	if (ret)
		return ret;

	/* Wait for settings to take effect */
	mdelay(500);

	if (priv->cfmt->code == V4L2_MBUS_FMT_YUYV8_2X8)
		dev_dbg(&client->dev, "Using 8-bit BT.656\n");
	else
		dev_dbg(&client->dev, "Using 10-bit BT.656\n");

	return 0;

ov1063x_set_fmt_error:
	dev_err(&client->dev, "Unsupported settings (%dx%d@(%d/%d)fps)\n",
		*width, *height, priv->fps_numerator, priv->fps_denominator);
	priv = NULL;
	priv->cfmt = NULL;

	return ret;
}

static int ov1063x_g_fmt(struct v4l2_subdev *sd,
			struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov1063x_priv *priv = to_ov1063x(client);
	enum v4l2_mbus_pixelcode code;

	if (priv->cfmt)
		code = priv->cfmt->code;
	else
		code = V4L2_MBUS_FMT_YUYV8_2X8;

	mf->width	= priv->width;
	mf->height	= priv->height;
	mf->code	= priv->cfmt->code;
	mf->colorspace	= priv->cfmt->colorspace;
	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

/* set the format we will capture in */
static int ov1063x_s_fmt(struct v4l2_subdev *sd,
			struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov1063x_priv *priv = to_ov1063x(client);
	int ret;

	ret = ov1063x_set_params(client, &mf->width, &mf->height,
				mf->code);
	if (!ret)
		mf->colorspace = priv->cfmt->colorspace;

	return ret;
}

static int ov1063x_try_fmt(struct v4l2_subdev *sd,
			struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov1063x_priv *priv = to_ov1063x(client);
	int i;

	/* Note: All sizes are good */

	mf->field = V4L2_FIELD_NONE;

	for (i = 0; i < ARRAY_SIZE(ov1063x_cfmts); i++)
		if (mf->code == ov1063x_cfmts[i].code) {
			mf->colorspace	= ov1063x_cfmts[i].colorspace;
			break;
		}

	if (i == ARRAY_SIZE(ov1063x_cfmts)) {
		/* Unsupported format requested. Propose either */
		if (priv->cfmt) {
			/* the current one or */
			mf->colorspace = priv->cfmt->colorspace;
			mf->code = priv->cfmt->code;
		} else {
			/* the default one */
			mf->colorspace = ov1063x_cfmts[0].colorspace;
			mf->code = ov1063x_cfmts[0].code;
		}
	}

	return 0;
}

static int ov1063x_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
			enum v4l2_mbus_pixelcode *code)
{
	if (index >= ARRAY_SIZE(ov1063x_cfmts))
		return -EINVAL;

	*code = ov1063x_cfmts[index].code;

	return 0;
}

static int ov1063x_enum_framesizes(struct v4l2_subdev *sd,
			struct v4l2_frmsizeenum *f)
{
	/* For now, hard coded resolutions for OV10635 sensor */
	int cam_width[] =	{ 1280, 1280, 752, 640, 600, 352, 320 };
	int cam_height[] =	{  800,  720, 480, 480, 400, 288, 240 };

	if (f->index >= ARRAY_SIZE(cam_width))
		return -EINVAL;

	f->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	f->discrete.width = cam_width[f->index];
	f->discrete.height = cam_height[f->index];
	return 0;
}

static int ov1063x_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov1063x_priv *priv = to_ov1063x(client);
	struct v4l2_captureparm *cp = &parms->parm.capture;

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(cp, 0, sizeof(struct v4l2_captureparm));
	cp->capability = V4L2_CAP_TIMEPERFRAME;
	cp->timeperframe.denominator = priv->fps_numerator;
	cp->timeperframe.numerator = priv->fps_denominator;

	return 0;
}

static int ov1063x_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov1063x_priv *priv = to_ov1063x(client);
	struct v4l2_captureparm *cp = &parms->parm.capture;
	enum v4l2_mbus_pixelcode code;
	int ret;

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	if (cp->extendedmode != 0)
		return -EINVAL;

	/* FIXME Check we can handle the requested framerate */
	priv->fps_denominator = cp->timeperframe.numerator;
	priv->fps_numerator = cp->timeperframe.denominator;

	if (priv->cfmt)
		code = priv->cfmt->code;
	else
		code = V4L2_MBUS_FMT_YUYV8_2X8;

	ret = ov1063x_set_params(client, &priv->width, &priv->height, code);
	if (ret < 0)
		return ret;

	return 0;
}

static int ov1063x_s_crop(struct v4l2_subdev *sd, const struct v4l2_crop *a)
{
	struct v4l2_crop a_writable = *a;
	struct v4l2_rect *rect = &a_writable.c;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov1063x_priv *priv = to_ov1063x(client);
	int ret = -EINVAL;
	enum v4l2_mbus_pixelcode code;

	if (priv->cfmt)
		code = priv->cfmt->code;
	else
		code = V4L2_MBUS_FMT_YUYV8_2X8;

	ret = ov1063x_set_params(client, &rect->width, &rect->height, code);
	if (!ret)
		return -EINVAL;

	rect->width = priv->width;
	rect->height = priv->height;
	rect->left = 0;
	rect->top = 0;

	return ret;
}

static int ov1063x_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *a)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov1063x_priv *priv = to_ov1063x(client);

	if (priv) {
		a->c.width = priv->width;
		a->c.height = priv->height;
	} else {
		a->c.width = OV1063X_MAX_WIDTH;
		a->c.height = OV1063X_MAX_HEIGHT;
	}

	a->c.left = 0;
	a->c.top = 0;
	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	return 0;
}

static int ov1063x_cropcap(struct v4l2_subdev *sd, struct v4l2_cropcap *a)
{
	a->bounds.left = 0;
	a->bounds.top = 0;
	a->bounds.width = OV1063X_MAX_WIDTH;
	a->bounds.height = OV1063X_MAX_HEIGHT;
	a->defrect = a->bounds;
	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	a->pixelaspect.numerator = 1;
	a->pixelaspect.denominator = 1;

	return 0;
}

static int ov1063x_init_gpios(struct i2c_client *client)
{
	struct ov1063x_priv *priv = to_ov1063x(client);
	int ret = 0;

	ret = gpio_request_array(priv->mux_gpios, priv->num_gpios);
	if (ret)
		goto done;
	gpio_free_array(priv->mux_gpios, priv->num_gpios);

done:
	return ret;
}

static int ov1063x_video_probe(struct i2c_client *client)
{
	struct ov1063x_priv *priv = to_ov1063x(client);
	u8 pid, ver;
	u16 id;
	int ret;

	ret = ov1063x_set_regs(client, ov1063x_regs_default,
		ARRAY_SIZE(ov1063x_regs_default));
	if (ret)
		return ret;

	usleep_range(500, 510);

	/* check and show product ID and manufacturer ID */
	ret = ov1063x_reg_read(client, OV1063X_PID, &pid);
	if (ret)
		return ret;

	id = pid << 8;
	ret = ov1063x_reg_read(client, OV1063X_VER, &ver);
	if (ret)
		return ret;

	id |= ver;
	if (OV1063X_VERSION(pid, ver) == OV10635_VERSION_REG) {
		priv->model = SENSOR_OV10635;
		priv->revision = 1;
	} else if (OV1063X_VERSION(pid, ver) == OV10633_VERSION_REG) {
		priv->model = SENSOR_OV10633;
		priv->revision = 1;
	} else {
		dev_err(&client->dev, "Product ID error %x:%x\n", pid, ver);
				return -ENODEV;
	}

	dev_info(&client->dev, "ov1063x Product ID %x Manufacturer ID %x\n",
		 pid, ver);

	/* Program all the 'standard' registers */

	return v4l2_ctrl_handler_setup(&priv->hdl);
}

static const struct v4l2_ctrl_ops ov1063x_ctrl_ops = {
	.s_ctrl = ov1063x_s_ctrl,
};

static struct v4l2_subdev_video_ops ov1063x_video_ops = {
	.s_stream	= ov1063x_s_stream,
	.g_mbus_fmt	= ov1063x_g_fmt,
	.s_mbus_fmt	= ov1063x_s_fmt,
	.try_mbus_fmt	= ov1063x_try_fmt,
	.enum_mbus_fmt	= ov1063x_enum_fmt,
	.enum_framesizes = ov1063x_enum_framesizes,
	.cropcap	= ov1063x_cropcap,
	.g_crop		= ov1063x_g_crop,
	.s_crop		= ov1063x_s_crop,
	.g_parm		= ov1063x_g_parm,
	.s_parm		= ov1063x_s_parm,
};

static struct v4l2_subdev_ops ov1063x_subdev_ops = {
	.video	= &ov1063x_video_ops,
};

/*
 * i2c_driver function
 */

static int ov1063x_of_probe(struct i2c_client *client,
			struct device_node *node)
{
	struct ov1063x_priv *priv = to_ov1063x(client);
	struct gpio *gpios = &priv->mux_gpios[0];
	unsigned int flags;
	int i, gpio;

	/* Iterate over all the gpios in the device tree
	 * ENOENT is returned when trying to access last + 1 gpio */
	for (i = 0; i < MAX_NUM_GPIOS; i++) {
		gpio = of_get_named_gpio_flags(node, "mux-gpios", i, &flags);
		if (gpio_is_valid(gpio)) {
			gpios[i].gpio	= gpio;
			gpios[i].flags	= (flags & GPIO_ACTIVE_LOW) ?
				GPIOF_OUT_INIT_LOW : GPIOF_OUT_INIT_HIGH;
			gpios[i].label	= client->name;
		} else if (gpio == -ENOENT) {
			break;
		} else {
			return gpio;
		}
	}
	priv->num_gpios = i;
	return 0;
}

static int ov1063x_probe(struct i2c_client *client,
			 const struct i2c_device_id *did)
{
	struct device_node *node = client->dev.of_node;
	struct ov1063x_priv *priv;
	struct v4l2_subdev *sd;
	int ret = 0;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	i2c_set_clientdata(client, priv);

	ret = ov1063x_of_probe(client, node);
	if (ret)
		goto err;

	/* TODO External XVCLK is board specific */
	priv->xvclk = 24000000;
	/* Default framerate */
	priv->fps_numerator = 30;
	priv->fps_denominator = 1;
	priv->cfmt = &ov1063x_cfmts[0];
	priv->width = OV1063X_MAX_WIDTH;
	priv->height = OV1063X_MAX_HEIGHT;

	sd = &(priv->subdev);
	v4l2_i2c_subdev_init(sd, client, &ov1063x_subdev_ops);
	v4l2_ctrl_handler_init(&priv->hdl, 2);
	v4l2_ctrl_new_std(&priv->hdl, &ov1063x_ctrl_ops,
			  V4L2_CID_VFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&priv->hdl, &ov1063x_ctrl_ops,
			  V4L2_CID_HFLIP, 0, 1, 1, 0);
	priv->subdev.ctrl_handler = &priv->hdl;
	if (priv->hdl.error) {
		ret = priv->hdl.error;
		goto err;
	}

	ret = ov1063x_init_gpios(client);
	if (ret) {
		dev_err(&client->dev, "Failed to request gpios");
		goto err;
	}

	ret = ov1063x_video_probe(client);
	if (ret) {
		v4l2_ctrl_handler_free(&priv->hdl);
		goto err;
	}

	sd->dev = &client->dev;
	ret = v4l2_async_register_subdev(sd);

	pm_runtime_enable(&client->dev);

err:
	return ret;
}

static int ov1063x_remove(struct i2c_client *client)
{
	struct ov1063x_priv *priv = i2c_get_clientdata(client);

	v4l2_device_unregister_subdev(&priv->subdev);
	v4l2_ctrl_handler_free(&priv->hdl);

	return 0;
}

static const struct i2c_device_id ov1063x_id[] = {
	{ "ov10635", 0 },
	{ "ov10633", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov1063x_id);

#if defined(CONFIG_OF)
static const struct of_device_id ov1063x_dt_id[] = {
	{
		.compatible = "ovti,ov10635", .data = "ov10635"
	},
	{
		.compatible = "ovti,ov10633", .data = "ov10633"
	},
	{
	}
};

#else
#define ov1063x_dt_id NULL
#endif

static struct i2c_driver ov1063x_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "ov1063x",
		.of_match_table = ov1063x_dt_id,
	},
	.probe = ov1063x_probe,
	.remove = ov1063x_remove,
	.id_table = ov1063x_id,
};

module_i2c_driver(ov1063x_i2c_driver);

MODULE_DESCRIPTION("SoC Camera driver for OmniVision OV1063X");
MODULE_AUTHOR("Phil Edworthy <phil.edworthy@renesas.com>");
MODULE_LICENSE("GPL v2");
