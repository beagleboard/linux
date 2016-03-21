/*
 * Aptina mt9t11x CMOS Image Sensor driver
 *
 * This an adaptation of the existing soc_camera/mt9t112.c
 * to use the newer v4l2 subdevice framework.
 *
 * Copyright (C) 2015 Benoit Parrot <bparrot@ti.com>
 *
 * Register definitions and initial settings based on
 * soc_camera/mt9t112.c driver written by Kuninori Morimoto.
 * Copyright (C) 2009 Renesas Solutions Corp.
 * Kuninori Morimoto <morimoto.kuninori@renesas.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/media.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/ratelimit.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/videodev2.h>

#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-image-sizes.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-common.h>
#include <media/v4l2-of.h>

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-2)");

#define DRIVER_NAME "mt9t11x"

#define MT9T111_ID	0
#define MT9T112_ID	1

/* you can check PLL/clock info */
#define EXT_CLOCK 32000000
#define MT9T11x_DEF_PIXEL_CLOCK 96000000
#define MT9T11x_DEF_EXT_CLOCK 32000000

#define MT9T11x_MAX_EXT_CLK 54000000
#define MT9T11x_MAX_VCO_CLK 768000000
#define MT9T11x_MAX_PIXEL_CLK 96000000
#define MT9T11x_MAX_MIPI_CLK 768000000
#define MT9T11x_MAX_MCU_CLK 96000000
#define MT9T11x_MAX_SOC_CLK 54000000
#define MT9T11x_MAX_SENSOR_CLK 70000000
#define MT9T11x_MAX_PFD_CLK 24000000

#define MT9T11x_FLAG_VFLIP	BIT(2)
#define MT9T11x_FLAG_HFLIP	BIT(3)

struct mt9t11x_pll_divider {
	u8 m, n;
	u8 p1, p2, p3, p4, p5, p6, p7;
};

/*
 * mt9t11x camera info
 */
struct mt9t11x_camera_info {
	u32 flags;
	struct mt9t11x_pll_divider divider;
	unsigned int mclk;
	unsigned int pclk;
	struct v4l2_of_endpoint endpoint;
};

/************************************************************************
 *			macro
 ***********************************************************************/
/*
 * frame size
 */
#define MIN_WIDTH   32
#define MIN_HEIGHT  32
#define MAX_WIDTH   2048
#define MAX_HEIGHT  1536

#define VGA_WIDTH   640
#define VGA_HEIGHT  480

/*
 * Logical address
 */
#define _VAR(id, offset, base)	(base | (id & 0x1f) << 10 | (offset & 0x3ff))
#define VAR(id, offset)  _VAR(id, offset, 0x0000)
#define VAR8(id, offset) _VAR(id, offset, 0x8000)

/************************************************************************
 *			struct
 ***********************************************************************/
struct mt9t11x_format {
	u32 code;
	u32 colorspace;
	u16 fmt;
	u16 order;
};

struct mt9t11x_framesize {
	u16 width;
	u16 height;
};

struct mt9t11x_resolution_param {
	u16 col_strt;
	u16 row_end;
	u16 col_end;
	u16 read_mode;
	u16 fine_cor;
	u16 fine_min;
	u16 fine_max;
	u16 base_lines;
	u16 min_lin_len;
	u16 line_len;
	u16 con_width;
	u16 con_height;
	u16 s_f1_50;
	u16 s_f2_50;
	u16 s_f1_60;
	u16 s_f2_60;
	u16 per_50;
	u16 per_50_M;
	u16 per_60;
	u16 fd_w_height;
	u16 tx_water;
	u16 max_fd_50;
	u16 max_fd_60;
	u16 targ_fd;
};

struct mt9t11x_priv {
	struct v4l2_subdev		subdev;
#if defined(CONFIG_MEDIA_CONTROLLER)
	struct media_pad		pad;
#endif
	struct v4l2_ctrl_handler	handler;

	struct mt9t11x_camera_info	*info;
	struct i2c_client		*client;
	struct v4l2_rect		frame;
	struct v4l2_clk			*clk;
	const struct mt9t11x_format	*format;
	int				num_formats;
	u32				flags;

	/* GPIOs */
	struct gpio_desc		*reset_gpio;
	struct gpio_desc		*powerdown_gpio;
	struct gpio_desc		*oscen_gpio;
	struct gpio_desc		*bufen_gpio;
	struct gpio_desc		*camen_gpio;

	/* V4L2 Ctrl handle */
	struct v4l2_ctrl		*hflip;
	struct v4l2_ctrl		*vflip;

	/* Protects the struct fields below */
	struct mutex			lock;
	int				streaming;
	int				power;

	struct mt9t11x_resolution_param	resolution;
};

/************************************************************************
 *			supported frame sizes
 ***********************************************************************/

static const struct mt9t11x_framesize mt9t11x_framesizes[] = {
	{
		.width		= 2048,
		.height		= 1536,
	}, {
		.width		= 1920,
		.height		= 1200,
	}, {
		.width		= 1920,
		.height		= 1080,
	}, {
		.width		= 1600,
		.height		= 1200,
	}, {
		.width		= 1440,
		.height		= 900,
	}, {
		.width		= 1280,
		.height		= 800,
	}, {
		.width		= 1280,
		.height		= 720,
	}, {
		.width		= 800,
		.height		= 600,
	}, {
		.width		= 800,
		.height		= 480,
	}, {
		.width		= 640,
		.height		= 480,
	}, {
		.width		= 600,
		.height		= 400,
	}, {
		.width		= 352,
		.height		= 288,
	}, {
		.width		= 320,
		.height		= 240,
	}, {
		.width		= 160,
		.height		= 120,
	},
};

/************************************************************************
 *			supported format
 ***********************************************************************/

static const struct mt9t11x_format mt9t11x_cfmts[] = {
	{
		.code		= MEDIA_BUS_FMT_UYVY8_2X8,
		.colorspace	= V4L2_COLORSPACE_JPEG,
		.fmt		= 1,
		.order		= 0,
	}, {
		.code		= MEDIA_BUS_FMT_VYUY8_2X8,
		.colorspace	= V4L2_COLORSPACE_JPEG,
		.fmt		= 1,
		.order		= 1,
	}, {
		.code		= MEDIA_BUS_FMT_YUYV8_2X8,
		.colorspace	= V4L2_COLORSPACE_JPEG,
		.fmt		= 1,
		.order		= 2,
	}, {
		.code		= MEDIA_BUS_FMT_YVYU8_2X8,
		.colorspace	= V4L2_COLORSPACE_JPEG,
		.fmt		= 1,
		.order		= 3,
	}, {
		.code		= MEDIA_BUS_FMT_RGB555_2X8_PADHI_LE,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.fmt		= 8,
		.order		= 2,
	}, {
		.code		= MEDIA_BUS_FMT_RGB565_2X8_LE,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.fmt		= 4,
		.order		= 2,
	},
};

/************************************************************************
 *			general function
 ***********************************************************************/
static inline struct mt9t11x_priv *sd_to_mt9t11x(struct v4l2_subdev *sd)
{
	return container_of(sd, struct mt9t11x_priv, subdev);
}

static inline struct mt9t11x_priv *to_mt9t11x(const struct i2c_client *client)
{
	return sd_to_mt9t11x(i2c_get_clientdata(client));
}

static inline struct mt9t11x_priv *ctrl_to_mt9t11x(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct mt9t11x_priv, handler);
}

static int mt9t11x_reg_read(const struct i2c_client *client, u16 command)
{
	struct i2c_msg msg[2];
	u8 buf[2];
	int ret;

	command = swab16(command);

	msg[0].addr  = client->addr;
	msg[0].flags = 0;
	msg[0].len   = 2;
	msg[0].buf   = (u8 *)&command;

	msg[1].addr  = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len   = 2;
	msg[1].buf   = buf;

	/*
	 * if return value of this function is < 0,
	 * it mean error.
	 * else, under 16bit is valid data.
	 */
	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0)
		return ret;

	memcpy(&ret, buf, 2);
	return swab16(ret);
}

static int mt9t11x_reg_write(const struct i2c_client *client,
			     u16 command, u16 data)
{
	struct i2c_msg msg;
	u8 buf[4];
	int ret;

	command = swab16(command);
	data = swab16(data);

	memcpy(buf + 0, &command, 2);
	memcpy(buf + 2, &data,    2);

	msg.addr  = client->addr;
	msg.flags = 0;
	msg.len   = 4;
	msg.buf   = buf;

	/*
	 * i2c_transfer return message length,
	 * but this function should return 0 if correct case
	 */
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret >= 0)
		ret = 0;

	return ret;
}

static int mt9t11x_reg_mask_set(const struct i2c_client *client,
				u16  command,
				u16  mask,
				u16  set)
{
	int val = mt9t11x_reg_read(client, command);

	if (val < 0)
		return val;

	val &= ~mask;
	val |= set & mask;

	return mt9t11x_reg_write(client, command, val);
}

/* mcu access */
static int mt9t11x_mcu_read(const struct i2c_client *client, u16 command)
{
	int ret;

	ret = mt9t11x_reg_write(client, 0x098E, command);
	if (ret < 0)
		return ret;

	return mt9t11x_reg_read(client, 0x0990);
}

static int mt9t11x_mcu_write(const struct i2c_client *client,
			     u16 command, u16 data)
{
	int ret;

	ret = mt9t11x_reg_write(client, 0x098E, command);
	if (ret < 0)
		return ret;

	return mt9t11x_reg_write(client, 0x0990, data);
}

static int mt9t11x_mcu_mask_set(const struct i2c_client *client,
				u16  command,
				u16  mask,
				u16  set)
{
	int val = mt9t11x_mcu_read(client, command);

	if (val < 0)
		return val;

	val &= ~mask;
	val |= set & mask;

	return mt9t11x_mcu_write(client, command, val);
}

static int mt9t11x_reset(const struct i2c_client *client)
{
	int ret;

	dev_dbg(&client->dev, "%s:\n", __func__);

	ret = mt9t11x_reg_mask_set(client, 0x001a, 0x0001, 0x0001);
	if (ret < 0)
		return ret;
	usleep_range(1000, 2000);
	ret = mt9t11x_reg_mask_set(client, 0x001a, 0x0001, 0x0000);
	if (ret < 0)
		return ret;

	return ret;
}

#define CLOCK_INFO(a, b)				\
	do {						\
		if (debug > 1)				\
			mt9t11x_clock_info(a, b);	\
	} while (0)
static int mt9t11x_clock_info(const struct i2c_client *client, u32 ext)
{
	int m, n, p1, p2, p3, p4, p5, p6, p7;
	u32 vco, clk;
	char *enable;

	ext /= 1000; /* kbyte order */

	n = mt9t11x_reg_read(client, 0x0012);
	p1 = n & 0x000f;
	n = n >> 4;
	p2 = n & 0x000f;
	n = n >> 4;
	p3 = n & 0x000f;

	n = mt9t11x_reg_read(client, 0x002a);
	p4 = n & 0x000f;
	n = n >> 4;
	p5 = n & 0x000f;
	n = n >> 4;
	p6 = n & 0x000f;

	n = mt9t11x_reg_read(client, 0x002c);
	p7 = n & 0x000f;

	n = mt9t11x_reg_read(client, 0x0010);
	m = n & 0x00ff;
	n = (n >> 8) & 0x003f;

	enable = ((ext < 6000) || (ext > 54000)) ? "X" : "";
	dev_dbg(&client->dev, "EXTCLK          : %10u K %s\n", ext, enable);

	vco = 2 * m * ext / (n + 1);
	enable = ((vco < 384000) || (vco > 768000)) ? "X" : "";
	dev_dbg(&client->dev, "VCO             : %10u K %s\n", vco, enable);

	clk = vco / (p1 + 1) / (p2 + 1);
	enable = (clk > 96000) ? "X" : "";
	dev_dbg(&client->dev, "PIXCLK          : %10u K %s\n", clk, enable);

	clk = vco / (p3 + 1);
	enable = (clk > 768000) ? "X" : "";
	dev_dbg(&client->dev, "MIPICLK         : %10u K %s\n", clk, enable);

	clk = vco / (p6 + 1);
	enable = (clk > 96000) ? "X" : "";
	dev_dbg(&client->dev, "MCU CLK         : %10u K %s\n", clk, enable);

	clk = vco / (p5 + 1);
	enable = (clk > 54000) ? "X" : "";
	dev_dbg(&client->dev, "SOC CLK         : %10u K %s\n", clk, enable);

	clk = vco / (p4 + 1);
	enable = (clk > 70000) ? "X" : "";
	dev_dbg(&client->dev, "Sensor CLK      : %10u K %s\n", clk, enable);

	clk = vco / (p7 + 1);
	dev_dbg(&client->dev, "External sensor : %10u K\n", clk);

	clk = ext / (n + 1);
	enable = ((clk < 2000) || (clk > 24000)) ? "X" : "";
	dev_dbg(&client->dev, "PFD             : %10u K %s\n", clk, enable);

	return 0;
}

static void mt9t11x_frame_check(u32 *width, u32 *height, u32 *left, u32 *top)
{
	v4l_bound_align_image(width, MIN_WIDTH, MAX_WIDTH, 1,
			      height, MIN_HEIGHT, MAX_HEIGHT, 1, 0);
	*left = 0;
	*top = 0;
}

static int mt9t11x_set_a_frame_size(const struct i2c_client *client,
				    u16 width,
				    u16 height)
{
	int ret;
	u16 wstart = (MAX_WIDTH - width) / 2;
	u16 hstart = (MAX_HEIGHT - height) / 2;

	dev_dbg(&client->dev, "%s:\n", __func__);

	/* (Context A) Image Width/Height */
	ret = mt9t11x_mcu_write(client, VAR(26, 0), width);
	if (ret < 0)
		return ret;
	ret = mt9t11x_mcu_write(client, VAR(26, 2), height);
	if (ret < 0)
		return ret;

	/* (Context A) Output Width/Height */
	ret = mt9t11x_mcu_write(client, VAR(18, 43), 8 + width);
	if (ret < 0)
		return ret;
	ret = mt9t11x_mcu_write(client, VAR(18, 45), 8 + height);
	if (ret < 0)
		return ret;

	/* (Context A) Start Row/Column */
	ret = mt9t11x_mcu_write(client, VAR(18, 2), 4 + hstart);
	if (ret < 0)
		return ret;
	ret = mt9t11x_mcu_write(client, VAR(18, 4), 4 + wstart);
	if (ret < 0)
		return ret;

	/* (Context A) End Row/Column */
	ret = mt9t11x_mcu_write(client, VAR(18, 6), 11 + height + hstart);
	if (ret < 0)
		return ret;
	ret = mt9t11x_mcu_write(client, VAR(18, 8), 11 + width  + wstart);
	if (ret < 0)
		return ret;

	ret = mt9t11x_mcu_write(client, VAR8(1, 0), 0x06);
	if (ret < 0)
		return ret;

	return ret;
}

static int mt9t11x_set_pll_dividers(const struct i2c_client *client,
				    u8 m, u8 n,
				    u8 p1, u8 p2, u8 p3,
				    u8 p4, u8 p5, u8 p6,
				    u8 p7)
{
	int ret;
	u16 val;

	dev_dbg(&client->dev, "%s:\n", __func__);

	/* N/M */
	val = (n << 8) |
	      (m << 0);
	ret = mt9t11x_reg_mask_set(client, 0x0010, 0x3fff, val);
	if (ret < 0)
		return ret;

	/* P1/P2/P3 */
	val = ((p3 & 0x0F) << 8) |
	      ((p2 & 0x0F) << 4) |
	      ((p1 & 0x0F) << 0);
	ret = mt9t11x_reg_mask_set(client, 0x0012, 0x0fff, val);
	if (ret < 0)
		return ret;

	/* P4/P5/P6 */
	val = (0x7         << 12) |
	      ((p6 & 0x0F) <<  8) |
	      ((p5 & 0x0F) <<  4) |
	      ((p4 & 0x0F) <<  0);
	ret = mt9t11x_reg_mask_set(client, 0x002A, 0x7fff, val);
	if (ret < 0)
		return ret;

	/* P7 */
	val = (0x1         << 12) |
	      ((p7 & 0x0F) <<  0);
	ret = mt9t11x_reg_mask_set(client, 0x002C, 0x100f, val);
	if (ret < 0)
		return ret;

	return ret;
}

static int mt9t11x_set_resolution_params(const struct i2c_client *client)
{
	int ret = 1;
	struct mt9t11x_priv *priv = to_mt9t11x(client);
	struct mt9t11x_resolution_param *resolution = &priv->resolution;

	if ((priv->frame.width == 1280) && (priv->frame.height == 720)) {
		resolution->col_strt    = 0x0004;
		resolution->row_end     = 0x05AD;
		resolution->col_end     = 0x050B;
		resolution->read_mode   = 0x002C;
		resolution->fine_cor    = 0x008C;
		resolution->fine_min    = 0x01F1;
		resolution->fine_max    = 0x00FF;
		resolution->base_lines  = 0x032D;
		resolution->min_lin_len = 0x0378;
		resolution->line_len    = 0x091C;
		resolution->con_width   = 0x0508;
		resolution->con_height  = 0x02D8;
		resolution->s_f1_50     = 0x23;
		resolution->s_f2_50     = 0x25;
		resolution->s_f1_60     = 0x2B;
		resolution->s_f2_60     = 0x2D;
		resolution->per_50      = 0xDC;
		resolution->per_50_M    = 0x00;
		resolution->per_60      = 0xB7;
		resolution->fd_w_height = 0x05;
		resolution->tx_water    = 0x0210;
		resolution->max_fd_50   = 0x0004;
		resolution->max_fd_60   = 0x0004;
		resolution->targ_fd     = 0x0004;
	} else if ((priv->frame.width <= 1024) &&
		   (priv->frame.height <= 768) &&
		   (priv->frame.width != priv->frame.height)) {
		resolution->col_strt    = 0x000;
		resolution->row_end     = 0x60D;
		resolution->col_end     = 0x80D;
		resolution->read_mode   = 0x046C;
		resolution->fine_cor    = 0x00CC;
		resolution->fine_min    = 0x0381;
		resolution->fine_max    = 0x024F;
		resolution->base_lines  = 0x0364;
		resolution->min_lin_len = 0x05D0;
		resolution->line_len    = 0x07AC;
		resolution->con_width   = 0x0408;
		resolution->con_height  = 0x0308;
		resolution->s_f1_50     = 0x23;
		resolution->s_f2_50     = 0x25;
		resolution->s_f1_60     = 0x2A;
		resolution->s_f2_60     = 0x2C;
		resolution->per_50      = 0x05;
		resolution->per_50_M    = 0x01;
		resolution->per_60      = 0xD9;
		resolution->fd_w_height = 0x06;
		resolution->max_fd_50   = 0x0003;
		resolution->max_fd_60   = 0x0004;
		resolution->targ_fd     = 0x0003;
		if ((priv->frame.width == 1024) &&
		    (priv->frame.height == 768)) {
			resolution->tx_water = 0x0218;
		} else if ((priv->frame.width == 800) &&
			   (priv->frame.height == 480)) {
			resolution->tx_water = 0x02DA;
		} else {
			/*
			 * 640 x 480 but use it with everything else until
			 * we figure out how to calc it
			 */
			resolution->tx_water = 0x0352;
		}
	} else {
		ret = 0;
	}

	return ret;
}

static int mt9t11x_pll_setup_pll(const struct i2c_client *client)
{
	struct mt9t11x_priv *priv = to_mt9t11x(client);
	int data, i, ret;

	/* Bypass PLL */
	ret = mt9t11x_reg_mask_set(client, 0x14, 1, 1);
	if (ret < 0)
		return ret;
	/* Power-down PLL */
	ret = mt9t11x_reg_mask_set(client, 0X14, 2, 0);
	if (ret < 0)
		return ret;
	/* PLL control: BYPASS PLL = 8517 */
	ret = mt9t11x_reg_write(client, 0x0014, 0x2145);
	if (ret < 0)
		return ret;

	/* Replace these registers when new timing parameters are generated */
	ret = mt9t11x_set_pll_dividers(client,
				       priv->info->divider.m,
				       priv->info->divider.n,
				       priv->info->divider.p1,
				       priv->info->divider.p2,
				       priv->info->divider.p3,
				       priv->info->divider.p4,
				       priv->info->divider.p5,
				       priv->info->divider.p6,
				       priv->info->divider.p7);
	if (ret < 0)
		return ret;

	/* Reset Misc. Control = 536 */
	ret = mt9t11x_reg_write(client, 0x001A, 0x218);
	if (ret < 0)
		return ret;
	/* PLL control: TEST_BYPASS on = 9541 */
	ret = mt9t11x_reg_write(client, 0x0014, 0x2545);
	if (ret < 0)
		return ret;
	/* PLL control: PLL_ENABLE on = 9543 */
	ret = mt9t11x_reg_write(client, 0x0014, 0x2547);
	if (ret < 0)
		return ret;
	/* PLL control: SEL_LOCK_DET on = 9287 */
	ret = mt9t11x_reg_write(client, 0x0014, 0x2447);
	if (ret < 0)
		return ret;
	/* PLL control: TEST_BYPASS off = 8263 */
	ret = mt9t11x_reg_write(client, 0x0014, 0x2047);
	if (ret < 0)
		return ret;

	/*  Wait for the PLL to lock */
	for (i = 0; i < 1000; i++) {
		data = mt9t11x_reg_read(client, 0x0014);
		if (0x8000 & data)
			break;

		usleep_range(10000, 11000);
	}

	/* PLL control: PLL_BYPASS off = 8262 */
	ret = mt9t11x_reg_write(client, 0x0014, 0x2046);
	if (ret < 0)
		return ret;
	/* Reference clock count for 20 us = 640 */
	ret = mt9t11x_reg_write(client, 0x0022, 0x0280);
	if (ret < 0)
		return ret;
	/* Pad Slew Rate = 1911 */
	ret = mt9t11x_reg_write(client, 0x001E, 0x0777);
	if (ret < 0)
		return ret;
	/* JPEG Clock = 1024 */
	ret = mt9t11x_reg_write(client, 0x0016, 0x0400);

	return ret;
}

inline u32 calc_vco(u32 desired, u32 ext, u8 *_m, u8 *_n)
{
	u32 m, n;
	u32 delta, actual;
	long bestdelta = -1;

	n = *_n + 1;
	if (n == 0)
		n = 1;

	for (; n <= 64; n++)
		for (m = 1; m <= 256; m++) {
			actual = ext * 2;
			actual *= m;
			actual /= n;
			delta = (desired - actual);
			if (delta < 0)
				continue;
			if ((delta < bestdelta) || (bestdelta == -1)) {
				bestdelta = delta;
				*_m = (u8)(m);
				*_n = (u8)(n - 1);
			}
		}
	actual = ext * 2 * (*_m);
	actual /= (*_n + 1);

	return actual;
}

static inline u32 calc_pixclk(u32 desired, u32 vco, u8 *_p1, u8 *_p2)
{
	u32 p1, p2;
	u32 delta, actual;
	long bestdelta = -1;

	for (p1 = 1; p1 <= 16; p1++)
		for (p2 = 1; p2 <= 16; p2++) {
			actual = vco;
			actual /= p1;
			actual /= p2;
			delta = (desired - actual);
			if (delta < 0)
				continue;
			if ((delta < bestdelta) || (bestdelta == -1)) {
				bestdelta = delta;
				*_p1 = (u8)(p1 - 1);
				*_p2 = (u8)(p2 - 1);
			}
		}
	actual = vco / (*_p1 + 1);
	actual /= (*_p2 + 1);

	return actual;
}

static inline u32 calc_div(u32 desired, u32 src, u8 *_div)
{
	u32 div;

	if (src > desired) {
		div = src / desired;
		if ((src % desired) > 0)
			div++;
	} else {
		div = 1;
	}

	*_div = (u8)(div - 1);

	return 0;
}

static unsigned int mt9t11x_pll_calc_params(struct mt9t11x_priv *priv)
{
	struct i2c_client *client = priv->client;
	struct mt9t11x_camera_info *info = priv->info;
	u32 vco;

	dev_dbg(&client->dev, "%s:\n", __func__);

	calc_div(MT9T11x_MAX_PFD_CLK, info->mclk, &info->divider.n);
	vco = calc_vco(MT9T11x_MAX_VCO_CLK, info->mclk, &info->divider.m,
		       &info->divider.n);
	calc_pixclk(info->pclk, vco, &info->divider.p1, &info->divider.p2);
	calc_div(MT9T11x_MAX_MIPI_CLK, vco, &info->divider.p3);
	calc_div(MT9T11x_MAX_MCU_CLK, vco, &info->divider.p6);
	calc_div(MT9T11x_MAX_SOC_CLK, vco, &info->divider.p5);
	calc_div(MT9T11x_MAX_SENSOR_CLK, vco, &info->divider.p4);

	return 0;
}

static int mt9t11x_sysctl_startup(const struct i2c_client *client)
{
	int ret = 0;

	/* reset */
	mt9t11x_reset(client);

	/* Setup PLL */
	mt9t11x_pll_setup_pll(client);

	return ret;
}

static int mt9t11x_high_speed_overrides(const struct i2c_client *client)
{
	int ret;

	/*
	 * Use this section to apply settings that are specific to this
	 * revision of SOC or for any other specialized settings
	 * clear the "Output Buffer Enable Adaptive Clock" bit to enable
	 * the SYSCTL slew rate settings, change this in the variables
	 * and register
	 */

	/* PRI_A_CONFIG_JPEG_OB_TX_CONTROL_VAR */
	ret = mt9t11x_mcu_write(client, VAR(26, 160), 0x082E);
	if (ret < 0)
		return ret;
	/* PRI_B_CONFIG_JPEG_OB_TX_CONTROL_VAR */
	ret = mt9t11x_mcu_write(client, VAR(27, 160), 0x082E);
	if (ret < 0)
		return ret;
	/* SEC_A_CONFIG_JPEG_OB_TX_CONTROL_VAR */
	ret = mt9t11x_mcu_write(client, VAR(28, 160), 0x082E);
	if (ret < 0)
		return ret;
	/* SEC_B_CONFIG_JPEG_OB_TX_CONTROL_VAR */
	ret = mt9t11x_mcu_write(client, VAR(29, 160), 0x082E);
	if (ret < 0)
		return ret;
	ret = mt9t11x_reg_mask_set(client, 0x3C52, 0x0040, 0);
	if (ret < 0)
		return ret;

	/* Set correct values for Context B FIFO control */
	/* CAM1_CTX_B_RX_FIFO_TRIGGER_MARK */
	ret = mt9t11x_mcu_write(client, VAR(18, 142), 32);
	if (ret < 0)
		return ret;
	/* PRI_B_CONFIG_IO_OB_MANUAL_FLAG */
	ret = mt9t11x_mcu_write(client, VAR(27, 172), 0);

	return ret;
}

static int mt9t11x_go(const struct i2c_client *client)
{
	int data, i, ret;

	/* release MCU from standby */
	ret = mt9t11x_reg_mask_set(client, 0x0018, 0x0001, 0);
	if (ret < 0)
		return ret;

	/* wait for K26A to come out of standby */
	for (i = 0; i < 100; i++) {
		data = mt9t11x_reg_read(client, 0x0018);
		if (!(0x4000 & data))
			break;

		usleep_range(10000, 11000);
	}

	return ret;
}

static int mt9t11x_continue(const struct i2c_client *client)
{
	int data, i, ret;

	/* clear powerup stop bit */
	ret = mt9t11x_reg_mask_set(client, 0x0018, 0x0004, 0);
	if (ret < 0)
		return ret;

	/* wait for sequencer to enter preview state */
	for (i = 0; i < 100; i++) {
		data = mt9t11x_mcu_read(client, VAR8(1, 1));
		if (data == 3)
			break;

		usleep_range(10000, 11000);
	}

	return ret;
}

static int mt9t11x_mcu_powerup_stop_enable(const struct i2c_client *client)
{
	int ret;

	/* set powerup stop bit */
	ret = mt9t11x_reg_mask_set(client, 0x0018, 0x0004, 1);

	return ret;
}

static int mt9t11x_custom_setup(const struct i2c_client *client)
{
	struct mt9t11x_priv *priv = to_mt9t11x(client);
	struct mt9t11x_resolution_param *resolution = &priv->resolution;
	int ret;

	/* I2C Master Clock Divider */
	ret = mt9t11x_mcu_write(client, VAR(24, 6), 0x0100);
	if (ret < 0)
		return ret;
	/* Output Width (A) */
	ret = mt9t11x_mcu_write(client, VAR(26, 0), priv->frame.width);
	if (ret < 0)
		return ret;
	/* Output Height (A) */
	ret = mt9t11x_mcu_write(client, VAR(26, 2), priv->frame.height);
	if (ret < 0)
		return ret;
	/* JPEG (A) */
	ret = mt9t11x_mcu_write(client, VAR8(26, 142), 0x00);
	if (ret < 0)
		return ret;
	/* Adaptive Output Clock (A) */
	ret = mt9t11x_mcu_mask_set(client, VAR(26, 160), 0x0040, 0x0000);
	if (ret < 0)
		return ret;
	/* Row Start (A) */
	ret = mt9t11x_mcu_write(client, VAR(18, 2), 0x000);
	if (ret < 0)
		return ret;
	/* Column Start (A) */
	ret = mt9t11x_mcu_write(client, VAR(18, 4), resolution->col_strt);
	if (ret < 0)
		return ret;
	/* Row End (A) */
	ret = mt9t11x_mcu_write(client, VAR(18, 6), resolution->row_end);
	if (ret < 0)
		return ret;
	/* Column End (A) */
	ret = mt9t11x_mcu_write(client, VAR(18, 8), resolution->col_end);
	if (ret < 0)
		return ret;
	/* Row Speed (A) */
	ret = mt9t11x_mcu_write(client, VAR(18, 10), 0x0111);
	if (ret < 0)
		return ret;
	/* Read Mode (A) */
	ret = mt9t11x_mcu_write(client, VAR(18, 12), resolution->read_mode);
	if (ret < 0)
		return ret;
	/* Fine Correction (A) */
	ret = mt9t11x_mcu_write(client, VAR(18, 15), resolution->fine_cor);
	if (ret < 0)
		return ret;
	/* Fine IT Min (A) */
	ret = mt9t11x_mcu_write(client, VAR(18, 17), resolution->fine_min);
	if (ret < 0)
		return ret;
	/* Fine IT Max Margin (A) */
	ret = mt9t11x_mcu_write(client, VAR(18, 19), resolution->fine_max);
	if (ret < 0)
		return ret;
	/* Base Frame Lines (A) */
	ret = mt9t11x_mcu_write(client, VAR(18, 29), resolution->base_lines);
	if (ret < 0)
		return ret;
	/* Min Line Length (A) */
	ret = mt9t11x_mcu_write(client, VAR(18, 31), resolution->min_lin_len);
	if (ret < 0)
		return ret;
	/* Line Length (A) */
	ret = mt9t11x_mcu_write(client, VAR(18, 37), resolution->line_len);
	if (ret < 0)
		return ret;
	/* Contex Width (A) */
	ret = mt9t11x_mcu_write(client, VAR(18, 43), resolution->con_width);
	if (ret < 0)
		return ret;
	/* Context Height (A) */
	ret = mt9t11x_mcu_write(client, VAR(18, 45), resolution->con_height);
	if (ret < 0)
		return ret;
	/* Output Width (B) */
	ret = mt9t11x_mcu_write(client, VAR(27, 0), 0x0800);
	if (ret < 0)
		return ret;
	/* Output Height (B) */
	ret = mt9t11x_mcu_write(client, VAR(27, 2), 0x0600);
	if (ret < 0)
		return ret;
	/* JPEG (B) */
	ret = mt9t11x_mcu_write(client, VAR8(27, 142), 0x01);
	if (ret < 0)
		return ret;
	/* Adaptive Output Clock (B) */
	ret = mt9t11x_mcu_mask_set(client, VAR(27, 160), 0x0040, 0x0000);
	if (ret < 0)
		return ret;
	/* Row Start (B) */
	ret = mt9t11x_mcu_write(client, VAR(18, 74), 0x004);
	if (ret < 0)
		return ret;
	/* Column Start (B) */
	ret = mt9t11x_mcu_write(client, VAR(18, 76), 0x004);
	if (ret < 0)
		return ret;
	/* Row End (B) */
	ret = mt9t11x_mcu_write(client, VAR(18, 78), 0x60B);
	if (ret < 0)
		return ret;
	/* Column End (B) */
	ret = mt9t11x_mcu_write(client, VAR(18, 80), 0x80B);
	if (ret < 0)
		return ret;
	/* Row Speed (B) */
	ret = mt9t11x_mcu_write(client, VAR(18, 82), 0x0111);
	if (ret < 0)
		return ret;
	/* Read Mode (B) */
	ret = mt9t11x_mcu_write(client, VAR(18, 84), 0x0024);
	if (ret < 0)
		return ret;
	/* Fine Correction (B) */
	ret = mt9t11x_mcu_write(client, VAR(18, 87), 0x008C);
	if (ret < 0)
		return ret;
	/* Fine IT Min (B) */
	ret = mt9t11x_mcu_write(client, VAR(18, 89), 0x01F1);
	if (ret < 0)
		return ret;
	/* Fine IT Max Margin (B) */
	ret = mt9t11x_mcu_write(client, VAR(18, 91), 0x00FF);
	if (ret < 0)
		return ret;
	/* Base Frame Lines (B) */
	ret = mt9t11x_mcu_write(client, VAR(18, 101), 0x06AE);
	if (ret < 0)
		return ret;
	/* Min Line Length (B) */
	ret = mt9t11x_mcu_write(client, VAR(18, 103), 0x0378);
	if (ret < 0)
		return ret;
	/* Line Length (B) */
	ret = mt9t11x_mcu_write(client, VAR(18, 109), 0x0A3A);
	if (ret < 0)
		return ret;
	/* Contex Width (B) */
	ret = mt9t11x_mcu_write(client, VAR(18, 115), 0x0808);
	if (ret < 0)
		return ret;
	/* Context Height (B) */
	ret = mt9t11x_mcu_write(client, VAR(18, 117), 0x0608);
	if (ret < 0)
		return ret;
	/* search_f1_50 */
	ret = mt9t11x_mcu_write(client, VAR8(18, 165), resolution->s_f1_50);
	if (ret < 0)
		return ret;
	/* search_f2_50 */
	ret = mt9t11x_mcu_write(client, VAR8(18, 166), resolution->s_f2_50);
	if (ret < 0)
		return ret;
	/* search_f1_60 */
	ret = mt9t11x_mcu_write(client, VAR8(18, 167), resolution->s_f1_60);
	if (ret < 0)
		return ret;
	/* search_f2_60 */
	ret = mt9t11x_mcu_write(client, VAR8(18, 168), resolution->s_f2_60);
	if (ret < 0)
		return ret;
	/* period_50Hz (A) */
	ret = mt9t11x_mcu_write(client, VAR8(18, 68), resolution->per_50);
	if (ret < 0)
		return ret;
	/* period_50Hz (A MSB) */
	ret = mt9t11x_mcu_write(client, VAR8(18, 303), resolution->per_50_M);
	if (ret < 0)
		return ret;
	/* period_60Hz (A) */
	ret = mt9t11x_mcu_write(client, VAR8(18, 69), resolution->per_60);
	if (ret < 0)
		return ret;
	/* period_60Hz (A MSB) */
	ret = mt9t11x_mcu_write(client, VAR8(18, 301), 0x00);
	if (ret < 0)
		return ret;
	/* period_50Hz (B) */
	ret = mt9t11x_mcu_write(client, VAR8(18, 140), 0xD2);
	if (ret < 0)
		return ret;
	/* period_50Hz (B) MSB */
	ret = mt9t11x_mcu_write(client, VAR8(18, 304), 0x00);
	if (ret < 0)
		return ret;
	/* period_60Hz (B) */
	ret = mt9t11x_mcu_write(client, VAR8(18, 141), 0xAF);
	if (ret < 0)
		return ret;
	/* period_60Hz (B) MSB */
	ret = mt9t11x_mcu_write(client, VAR8(18, 302), 0x00);
	if (ret < 0)
		return ret;
	/* FD Window Height */
	ret = mt9t11x_mcu_write(client, VAR8(14, 37), resolution->fd_w_height);
	if (ret < 0)
		return ret;
	/* Stat_min */
	ret = mt9t11x_mcu_write(client, VAR8(8, 9), 0x02);
	if (ret < 0)
		return ret;
	/* Stat_max */
	ret = mt9t11x_mcu_write(client, VAR8(8, 10), 0x03);
	if (ret < 0)
		return ret;
	/* Min_amplitude */
	ret = mt9t11x_mcu_write(client, VAR8(8, 12), 0x0A);
	if (ret < 0)
		return ret;
	/* RX FIFO Watermark (A) */
	ret = mt9t11x_mcu_write(client, VAR(18, 70), 0x0080);
	if (ret < 0)
		return ret;
	/* TX FIFO Watermark (A) */
	ret = mt9t11x_mcu_write(client, VAR(26, 170), resolution->tx_water);
	if (ret < 0)
		return ret;
	/* Max FD Zone 50 Hz */
	ret = mt9t11x_mcu_write(client, VAR(26, 21), resolution->max_fd_50);
	if (ret < 0)
		return ret;
	/* Max FD Zone 60 Hz */
	ret = mt9t11x_mcu_write(client, VAR(26, 23), resolution->max_fd_60);
	if (ret < 0)
		return ret;
	/* AE Target FD Zone */
	ret = mt9t11x_mcu_write(client, VAR(26, 45), resolution->targ_fd);
	if (ret < 0)
		return ret;
	/* RX FIFO Watermark (B) */
	ret = mt9t11x_mcu_write(client, VAR(18, 142), 0x0080);
	if (ret < 0)
		return ret;
	/* TX FIFO Watermark (B) */
	ret = mt9t11x_mcu_write(client, VAR(27, 170), 0x01D0);
	if (ret < 0)
		return ret;
	/* Refresh Sequencer Mode */
	ret = mt9t11x_mcu_write(client, VAR8(1, 0), 0x06);
	if (ret < 0)
		return ret;
	/* Refresh Sequencer */
	ret = mt9t11x_mcu_write(client, VAR8(1, 0), 0x05);
	if (ret < 0)
		return ret;

#ifdef TEST_PATTERN
	ret = mt9t11x_mcu_write(client, VAR(24, 3), 0x100);
	if (ret < 0)
		return ret;
	ret = mt9t11x_mcu_write(client, VAR(24, 37), 0x0B);
#endif

	return ret;
}

static int mt9t11x_optimal_power_consumption(const struct i2c_client *client)
{
	int ret;

	/* Analog setting B */
	ret = mt9t11x_reg_write(client, 0x3084, 0x2409);
	if (ret < 0)
		return ret;
	ret = mt9t11x_reg_write(client, 0x3092, 0x0A49);
	if (ret < 0)
		return ret;
	ret = mt9t11x_reg_write(client, 0x3094, 0x4949);
	if (ret < 0)
		return ret;
	ret = mt9t11x_reg_write(client, 0x3096, 0x4950);

	return ret;
}

static int mt9t11x_blooming_row_pattern(const struct i2c_client *client)
{
	int ret;

	/* Improve high light image quality */

	/* [CAM1_CTX_A_COARSE_ITMIN] */
	ret = mt9t11x_mcu_write(client, VAR(18, 21), 0x0004);
	if (ret < 0)
		return ret;
	/* [CAM1_CTX_B_COARSE_ITMIN] */
	ret = mt9t11x_mcu_write(client, VAR(18, 93), 0x0004);

	return ret;
}

static int mt9t11x_set_orientation(const struct i2c_client *client,
				   u32 mask, u32 flip)
{
	int ret;

	flip &= mask;

	dev_dbg(&client->dev, "%s:\n", __func__);

	/* [CAM1_CTX_A_READ_MODE] */
	ret = mt9t11x_mcu_mask_set(client, VAR(18, 12), mask, flip);
	if (ret < 0)
		return ret;
	/* [CAM1_CTX_A_PIXEL_ORDER] */
	ret = mt9t11x_mcu_mask_set(client, VAR8(18, 14), mask, flip);
	if (ret < 0)
		return ret;

	/* [CAM1_CTX_B_READ_MODE] */
	ret = mt9t11x_mcu_mask_set(client, VAR(18, 84), mask, flip);
	if (ret < 0)
		return ret;
	/* [CAM1_CTX_B_PIXEL_ORDER] */
	ret = mt9t11x_mcu_mask_set(client, VAR8(18, 86), mask, flip);
	if (ret < 0)
		return ret;

	/* [SEQ_CMD] */
	ret = mt9t11x_mcu_write(client, VAR8(1, 0), 0x06);

	return ret;
}

static int mt9t11x_init_camera_optimized(const struct i2c_client *client)
{
	int ret;

	dev_dbg(&client->dev, "%s:\n", __func__);

	ret = mt9t11x_sysctl_startup(client);
	if (ret < 0)
		return ret;
	ret = mt9t11x_mcu_powerup_stop_enable(client);
	if (ret < 0)
		return ret;
	ret = mt9t11x_go(client);
	if (ret < 0)
		return ret;

	ret = mt9t11x_custom_setup(client);
	if (ret < 0)
		return ret;

	ret = mt9t11x_high_speed_overrides(client);
	if (ret < 0)
		return ret;

	ret = mt9t11x_optimal_power_consumption(client);
	if (ret < 0)
		return ret;

	ret = mt9t11x_blooming_row_pattern(client);
	if (ret < 0)
		return ret;

	ret = mt9t11x_continue(client);

	return ret;
}

static int mt9t11x_init_setting(const struct i2c_client *client)
{
	int ret;

	dev_dbg(&client->dev, "%s:\n", __func__);

	/* Adaptive Output Clock (A) */
	ret = mt9t11x_mcu_mask_set(client, VAR(26, 160), 0x0040, 0x0000);
	if (ret < 0)
		return ret;
	/* Read Mode (A) */
	ret = mt9t11x_mcu_write(client, VAR(18, 12), 0x0024);
	if (ret < 0)
		return ret;
	/* Fine Correction (A) */
	ret = mt9t11x_mcu_write(client, VAR(18, 15), 0x00CC);
	if (ret < 0)
		return ret;
	/* Fine IT Min (A) */
	ret = mt9t11x_mcu_write(client, VAR(18, 17), 0x01f1);
	if (ret < 0)
		return ret;
	/* Fine IT Max Margin (A) */
	ret = mt9t11x_mcu_write(client, VAR(18, 19), 0x00fF);
	if (ret < 0)
		return ret;
	/* Base Frame Lines (A) */
	ret = mt9t11x_mcu_write(client, VAR(18, 29), 0x032D);
	if (ret < 0)
		return ret;
	/* Min Line Length (A) */
	ret = mt9t11x_mcu_write(client, VAR(18, 31), 0x073a);
	if (ret < 0)
		return ret;
	/* Line Length (A) */
	ret = mt9t11x_mcu_write(client, VAR(18, 37), 0x07d0);
	if (ret < 0)
		return ret;
	/* Adaptive Output Clock (B) */
	ret = mt9t11x_mcu_mask_set(client, VAR(27, 160), 0x0040, 0x0000);
	if (ret < 0)
		return ret;
	/* Row Start (B) */
	ret = mt9t11x_mcu_write(client, VAR(18, 74), 0x004);
	if (ret < 0)
		return ret;
	/* Column Start (B) */
	ret = mt9t11x_mcu_write(client, VAR(18, 76), 0x004);
	if (ret < 0)
		return ret;
	/* Row End (B) */
	ret = mt9t11x_mcu_write(client, VAR(18, 78), 0x60B);
	if (ret < 0)
		return ret;
	/* Column End (B) */
	ret = mt9t11x_mcu_write(client, VAR(18, 80), 0x80B);
	if (ret < 0)
		return ret;
	/* Fine Correction (B) */
	ret = mt9t11x_mcu_write(client, VAR(18, 87), 0x008C);
	if (ret < 0)
		return ret;
	/* Fine IT Min (B) */
	ret = mt9t11x_mcu_write(client, VAR(18, 89), 0x01F1);
	if (ret < 0)
		return ret;
	/* Fine IT Max Margin (B) */
	ret = mt9t11x_mcu_write(client, VAR(18, 91), 0x00FF);
	if (ret < 0)
		return ret;
	/* Base Frame Lines (B) */
	ret = mt9t11x_mcu_write(client, VAR(18, 101), 0x0668);
	if (ret < 0)
		return ret;
	/* Min Line Length (B) */
	ret = mt9t11x_mcu_write(client, VAR(18, 103), 0x0AF0);
	if (ret < 0)
		return ret;
	/* Line Length (B) */
	ret = mt9t11x_mcu_write(client, VAR(18, 109), 0x0AF0);
	if (ret < 0)
		return ret;
	/*
	 * Flicker Dectection registers
	 * This section should be replaced whenever new Timing file is
	 * generated.
	 * All the following registers need to be replaced
	 * Following registers are generated from Register Wizard but user can
	 * modify them. For detail see auto flicker detection tuning
	 */
	/* FD_FDPERIOD_SELECT */
	ret = mt9t11x_mcu_write(client, VAR8(8, 5), 0x01);
	if (ret < 0)
		return ret;
	/* PRI_B_CONFIG_FD_ALGO_RUN */
	ret = mt9t11x_mcu_write(client, VAR(27, 17), 0x0003);
	if (ret < 0)
		return ret;
	/* PRI_A_CONFIG_FD_ALGO_RUN */
	ret = mt9t11x_mcu_write(client, VAR(26, 17), 0x0003);
	if (ret < 0)
		return ret;
	/*
	 * AFD range detection tuning registers
	 */
	/* search_f1_50 */
	ret = mt9t11x_mcu_write(client, VAR8(18, 165), 0x25);
	if (ret < 0)
		return ret;
	/* search_f2_50 */
	ret = mt9t11x_mcu_write(client, VAR8(18, 166), 0x28);
	if (ret < 0)
		return ret;
	/* search_f1_60 */
	ret = mt9t11x_mcu_write(client, VAR8(18, 167), 0x2C);
	if (ret < 0)
		return ret;
	/* search_f2_60 */
	ret = mt9t11x_mcu_write(client, VAR8(18, 168), 0x2F);
	if (ret < 0)
		return ret;
	/* period_50Hz (A) */
	ret = mt9t11x_mcu_write(client, VAR8(18, 68), 0xBA);
	if (ret < 0)
		return ret;
	/* period_50Hz (A MSB) */
	ret = mt9t11x_mcu_write(client, VAR8(18, 303), 0x00);
	if (ret < 0)
		return ret;
	/* period_60Hz (A) */
	ret = mt9t11x_mcu_write(client, VAR8(18, 69), 0x9B);
	if (ret < 0)
		return ret;
	/* period_60Hz (A MSB) */
	ret = mt9t11x_mcu_write(client, VAR8(18, 301), 0x00);
	if (ret < 0)
		return ret;
	/* period_50Hz (B) */
	ret = mt9t11x_mcu_write(client, VAR8(18, 140), 0x82);
	if (ret < 0)
		return ret;
	/* period_50Hz (B) MSB */
	ret = mt9t11x_mcu_write(client, VAR8(18, 304), 0x00);
	if (ret < 0)
		return ret;
	/* period_60Hz (B) */
	ret = mt9t11x_mcu_write(client, VAR8(18, 141), 0x6D);
	if (ret < 0)
		return ret;
	/* period_60Hz (B) MSB */
	ret = mt9t11x_mcu_write(client, VAR8(18, 302), 0x00);
	if (ret < 0)
		return ret;
	/* FD Mode */
	ret = mt9t11x_mcu_write(client, VAR8(8, 2), 0x10);
	if (ret < 0)
		return ret;
	/* Stat_min */
	ret = mt9t11x_mcu_write(client, VAR8(8, 9), 0x02);
	if (ret < 0)
		return ret;
	/* Stat_max */
	ret = mt9t11x_mcu_write(client, VAR8(8, 10), 0x03);
	if (ret < 0)
		return ret;
	/* Min_amplitude */
	ret = mt9t11x_mcu_write(client, VAR8(8, 12), 0x0A);
	if (ret < 0)
		return ret;
	/* RX FIFO Watermark (A) */
	ret = mt9t11x_mcu_write(client, VAR(18, 70), 0x0014);
	if (ret < 0)
		return ret;
	/* RX FIFO Watermark (B) */
	ret = mt9t11x_mcu_write(client, VAR(18, 142), 0x0014);

	return ret;
}

static int mt9t11x_init_camera(const struct i2c_client *client)
{
	struct mt9t11x_priv *priv = to_mt9t11x(client);
	int ret;

	dev_dbg(&client->dev, "%s:\n", __func__);

	ret = mt9t11x_sysctl_startup(client);
	if (ret < 0)
		return ret;
	ret = mt9t11x_mcu_powerup_stop_enable(client);
	if (ret < 0)
		return ret;
	ret = mt9t11x_go(client);
	if (ret < 0)
		return ret;

	ret = mt9t11x_init_setting(client);
	if (ret < 0)
		return ret;

	ret = mt9t11x_high_speed_overrides(client);
	if (ret < 0)
		return ret;

	ret = mt9t11x_optimal_power_consumption(client);
	if (ret < 0)
		return ret;

	ret = mt9t11x_blooming_row_pattern(client);
	if (ret < 0)
		return ret;

	ret = mt9t11x_continue(client);
	if (ret < 0)
		return ret;

	ret = mt9t11x_set_a_frame_size(client,
				       priv->frame.width,
				       priv->frame.height);

	return ret;
}

/************************************************************************
 *			v4l2_subdev_core_ops
 ***********************************************************************/

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int mt9t11x_g_register(struct v4l2_subdev *sd,
			      struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int                ret;

	reg->size = 2;
	ret = mt9t11x_reg_read(client, reg->reg);

	reg->val = (__u64)ret;

	return 0;
}

static int mt9t11x_s_register(struct v4l2_subdev *sd,
			      const struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	ret = mt9t11x_reg_write(client, reg->reg, reg->val);

	return ret;
}
#endif

static void __mt9t11x_set_power(struct mt9t11x_priv *priv, int on)
{
	struct i2c_client *client = priv->client;

	dev_dbg(&client->dev, "%s: on: %d\n", __func__, on);
	on = (on) ? 1 : 0;

	if (priv->power == on)
		return;

	if (on) {
		if (priv->powerdown_gpio)
			gpiod_set_value_cansleep(priv->powerdown_gpio, 0);
		if (priv->reset_gpio)
			gpiod_set_value_cansleep(priv->reset_gpio, 0);
		usleep_range(25000, 26000);
	} else {
		if (priv->powerdown_gpio)
			gpiod_set_value_cansleep(priv->powerdown_gpio, 1);
		if (priv->reset_gpio)
			gpiod_set_value_cansleep(priv->reset_gpio, 1);
	}

	priv->power = on;
}

static int mt9t11x_s_power(struct v4l2_subdev *sd, int on)
{
	struct mt9t11x_priv *priv = sd_to_mt9t11x(sd);
	struct i2c_client *client = priv->client;

	dev_dbg(&client->dev, "%s: on: %d\n", __func__, on);

	mutex_lock(&priv->lock);
	__mt9t11x_set_power(priv, on);
	mutex_unlock(&priv->lock);
	return 0;
}

static struct v4l2_subdev_core_ops mt9t11x_subdev_core_ops = {
	.log_status = v4l2_ctrl_subdev_log_status,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
	.s_power	= mt9t11x_s_power,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register	= mt9t11x_g_register,
	.s_register	= mt9t11x_s_register,
#endif
};

/*
 * V4L2 controls
 */
static int mt9t11x_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mt9t11x_priv *priv = ctrl_to_mt9t11x(ctrl);
	struct i2c_client *client = priv->client;
	int ret = -EINVAL;

	dev_dbg(&client->dev, "%s: ctrl_id:0x%x (%s), value: %d\n",
		__func__, ctrl->id, ctrl->name, ctrl->val);

	mutex_lock(&priv->lock);
	/*
	 * If the device is not powered up now, postpone applying control's
	 * value to the hardware, until it is ready to accept commands.
	 */
	if (priv->power == 0) {
		mutex_unlock(&priv->lock);
		return 0;
	}

	switch (ctrl->id) {
	case V4L2_CID_VFLIP:
		mt9t11x_set_orientation(client, 0x2, (ctrl->val) ? 2 : 0);
		ret = 0;
		break;
	case V4L2_CID_HFLIP:
		mt9t11x_set_orientation(client, 0x1, (ctrl->val) ? 1 : 0);
		ret = 0;
		break;
	}

	mutex_unlock(&priv->lock);
	return ret;
}

static const struct v4l2_ctrl_ops mt9t11x_ctrl_ops = {
	.s_ctrl	= mt9t11x_s_ctrl,
};

static int mt9t11x_initialize_controls(struct mt9t11x_priv *priv)
{
	const struct v4l2_ctrl_ops *ops = &mt9t11x_ctrl_ops;
	struct i2c_client *client = priv->client;
	struct v4l2_subdev *sd = &priv->subdev;
	struct v4l2_ctrl_handler *hdl = &priv->handler;
	int ret;

	dev_dbg(&client->dev, "%s:\n", __func__);

	ret = v4l2_ctrl_handler_init(hdl, 2);
	if (ret < 0)
		return ret;

	priv->vflip = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VFLIP, 0, 1, 1, 0);
	priv->hflip = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HFLIP, 0, 1, 1, 0);
	sd->ctrl_handler = hdl;
	if (priv->handler.error)
		return priv->handler.error;

	return 0;
}

/************************************************************************
 *			v4l2_subdev_video_ops
 ***********************************************************************/
static int mt9t11x_set_params(struct mt9t11x_priv *priv,
			      const struct v4l2_rect *rect,
			      u32 code)
{
	int i;

	/*
	 * get color format
	 */
	for (i = 0; i < priv->num_formats; i++)
		if (mt9t11x_cfmts[i].code == code)
			break;

	if (i == priv->num_formats)
		return -EINVAL;

	priv->frame  = *rect;

	/*
	 * frame size check
	 */
	mt9t11x_frame_check(&priv->frame.width, &priv->frame.height,
			    &priv->frame.left, &priv->frame.top);

	priv->format = mt9t11x_cfmts + i;

	return 0;
}

static int mt9t11x_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct mt9t11x_priv *priv = sd_to_mt9t11x(sd);
	struct i2c_client *client = priv->client;
	int ret = 0;
	int optimize = 0;
	struct v4l2_rect rect = {
		.width = priv->frame.width,
		.height = priv->frame.height,
		.left = priv->frame.left,
		.top = priv->frame.top,
	};

	dev_dbg(&client->dev, "%s: enable: %d\n", __func__, enable);

	mutex_lock(&priv->lock);

	if (priv->streaming == enable) {
		mutex_unlock(&priv->lock);
		return 0;
	}

	if (!enable) {
		/* Stop Streaming Sequence */
		__mt9t11x_set_power(priv, 0);
		priv->streaming = enable;
		mutex_unlock(&priv->lock);
		return 0;
	}

	mt9t11x_set_params(priv, &rect, priv->format->code);

	__mt9t11x_set_power(priv, 1);

	/* fill the structure with new resolution parameters */
	optimize = mt9t11x_set_resolution_params(client);

	if (optimize) {
		ret = mt9t11x_init_camera_optimized(client);
		if (ret < 0)
			return ret;
	} else {
		ret = mt9t11x_init_camera(client);
		if (ret < 0)
			return ret;
	}
	/*
	 * By default data is sampled on falling edge of pixclk.
	 * Change the default to be rising edge. i.e. Invert PCLK
	 */
	ret = mt9t11x_reg_write(client, 0x3C20, 0);
	if (ret < 0)
		return ret;
	usleep_range(5000, 6000);

	ret = mt9t11x_mcu_write(client, VAR(26, 7), priv->format->fmt);
	if (ret < 0)
		return ret;
	ret = mt9t11x_mcu_write(client, VAR(26, 9), priv->format->order);
	if (ret < 0)
		return ret;

	ret = mt9t11x_mcu_write(client, VAR8(1, 0), 0x06);
	if (ret < 0)
		return ret;

	if (priv->flags & MT9T11x_FLAG_VFLIP)
		v4l2_ctrl_s_ctrl(priv->vflip, 1);

	/* Make sure H/W is consistent with current control settings */
	ret = mt9t11x_set_orientation(client, 0x3,
				      (v4l2_ctrl_g_ctrl(priv->vflip) << 1 |
				       v4l2_ctrl_g_ctrl(priv->hflip)));
	if (ret < 0)
		return ret;

	dev_dbg(&client->dev, "format : %04x\n", priv->format->code);
	dev_dbg(&client->dev, "size   : %d x %d\n",
		priv->frame.width,
		priv->frame.height);

	priv->streaming = enable;

	CLOCK_INFO(client, EXT_CLOCK);

	mutex_unlock(&priv->lock);
	return ret;
}

static int mt9t11x_cropcap(struct v4l2_subdev *sd, struct v4l2_cropcap *a)
{
	struct mt9t11x_priv *priv = sd_to_mt9t11x(sd);
	struct i2c_client *client = priv->client;

	dev_dbg(&client->dev, "%s:\n", __func__);

	a->bounds.left			= 0;
	a->bounds.top			= 0;
	a->bounds.width			= MAX_WIDTH;
	a->bounds.height		= MAX_HEIGHT;
	a->defrect.left			= 0;
	a->defrect.top			= 0;
	a->defrect.width		= VGA_WIDTH;
	a->defrect.height		= VGA_HEIGHT;
	a->type				= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	a->pixelaspect.numerator	= 1;
	a->pixelaspect.denominator	= 1;

	return 0;
}

static int mt9t11x_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *a)
{
	struct mt9t11x_priv *priv = sd_to_mt9t11x(sd);
	struct i2c_client *client = priv->client;

	dev_dbg(&client->dev, "%s:\n", __func__);

	mutex_lock(&priv->lock);
	a->c	= priv->frame;
	a->type	= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	mutex_unlock(&priv->lock);

	return 0;
}

static int mt9t11x_s_crop(struct v4l2_subdev *sd, const struct v4l2_crop *a)
{
	struct mt9t11x_priv *priv = sd_to_mt9t11x(sd);
	struct i2c_client *client = priv->client;
	const struct v4l2_rect *rect = &a->c;
	int ret;

	dev_dbg(&client->dev, "%s:\n", __func__);

	mutex_lock(&priv->lock);
	ret = mt9t11x_set_params(priv, rect, priv->format->code);
	mutex_unlock(&priv->lock);

	return ret;
}

static void mt9t11x_get_default_format(struct mt9t11x_priv *priv,
				       struct v4l2_mbus_framefmt *mf)
{
	struct v4l2_rect rect = {
		.width = VGA_WIDTH,
		.height = VGA_HEIGHT,
		.left = (MAX_WIDTH - VGA_WIDTH) / 2,
		.top = (MAX_HEIGHT - VGA_HEIGHT) / 2,
	};

	mt9t11x_set_params(priv, &rect, MEDIA_BUS_FMT_UYVY8_2X8);

	/* Need fixing */
	mf->width = rect.width;
	mf->height = rect.height;
	mf->colorspace = mt9t11x_cfmts[0].colorspace;
	mf->code = mt9t11x_cfmts[0].code;

	mf->field = V4L2_FIELD_NONE;
}

static int mt9t11x_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct mt9t11x_priv *priv = sd_to_mt9t11x(sd);
	struct v4l2_mbus_framefmt *mf;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		mf = v4l2_subdev_get_try_format(sd, cfg, 0);
		mutex_lock(&priv->lock);
		fmt->format = *mf;
		mutex_unlock(&priv->lock);
		return 0;
	}

	mutex_lock(&priv->lock);
	mf = &fmt->format;
	mf->width	= priv->frame.width;
	mf->height	= priv->frame.height;
	mf->colorspace	= priv->format->colorspace;
	mf->code	= priv->format->code;
	mf->field	= V4L2_FIELD_NONE;
	mutex_unlock(&priv->lock);

	return 0;
}

static void __mt9t11x_try_frame_size(struct v4l2_mbus_framefmt *mf)
{
	const struct mt9t11x_framesize *fsize = &mt9t11x_framesizes[0];
	const struct mt9t11x_framesize *match = NULL;
	int i = ARRAY_SIZE(mt9t11x_framesizes);
	unsigned int min_err = UINT_MAX;

	while (i--) {
		int err = abs(fsize->width - mf->width)
				+ abs(fsize->height - mf->height);
		if (err < min_err) {
			min_err = err;
			match = fsize;
		}
		fsize++;
	}

	if (!match)
		match = &mt9t11x_framesizes[0];

	mf->width  = match->width;
	mf->height = match->height;
}

static int mt9t11x_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct mt9t11x_priv *priv = sd_to_mt9t11x(sd);
	unsigned int index = priv->num_formats;
	struct v4l2_mbus_framefmt *mf = &fmt->format;
	int ret = 0;
	struct v4l2_rect rect;

	__mt9t11x_try_frame_size(mf);
	rect.width = mf->width;
	rect.height = mf->height;
	rect.left = priv->frame.left;
	rect.top = priv->frame.top;

	while (--index >= 0)
		if (mt9t11x_cfmts[index].code == mf->code)
			break;

	if (index < 0)
		return -EINVAL;

	mf->colorspace = mt9t11x_cfmts[index].colorspace;
	mf->code = mt9t11x_cfmts[index].code;
	mf->field = V4L2_FIELD_NONE;

	mutex_lock(&priv->lock);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		mf = v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
		*mf = fmt->format;
	} else {
		ret = mt9t11x_set_params(priv, &rect, mf->code);
	}

	mutex_unlock(&priv->lock);

	return ret;
}

static int mt9t11x_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	struct mt9t11x_priv *priv = sd_to_mt9t11x(sd);

	if (code->index >= priv->num_formats)
		return -EINVAL;

	code->code = mt9t11x_cfmts[code->index].code;

	return 0;
}

static int mt9t11x_enum_frame_sizes(struct v4l2_subdev *sd,
				    struct v4l2_subdev_pad_config *cfg,
				    struct v4l2_subdev_frame_size_enum *fse)
{
	struct mt9t11x_priv *priv = sd_to_mt9t11x(sd);
	int i = priv->num_formats;

	if (fse->index >= ARRAY_SIZE(mt9t11x_framesizes))
		return -EINVAL;

	while (--i)
		if (mt9t11x_cfmts[i].code == fse->code)
			break;

	fse->code = mt9t11x_cfmts[i].code;

	fse->min_width  = mt9t11x_framesizes[fse->index].width;
	fse->max_width  = fse->min_width;
	fse->max_height = mt9t11x_framesizes[fse->index].height;
	fse->min_height = fse->max_height;

	return 0;
}

static struct v4l2_subdev_video_ops mt9t11x_subdev_video_ops = {
	.s_stream	= mt9t11x_s_stream,
	.cropcap	= mt9t11x_cropcap,
	.g_crop		= mt9t11x_g_crop,
	.s_crop		= mt9t11x_s_crop,
};

static const struct v4l2_subdev_pad_ops mt9t11x_subdev_pad_ops = {
	.enum_mbus_code		= mt9t11x_enum_mbus_code,
	.enum_frame_size	= mt9t11x_enum_frame_sizes,
	.get_fmt		= mt9t11x_get_fmt,
	.set_fmt		= mt9t11x_set_fmt,
};

/************************************************************************
 *			i2c driver
 ***********************************************************************/
static struct v4l2_subdev_ops mt9t11x_subdev_ops = {
	.core	= &mt9t11x_subdev_core_ops,
	.video	= &mt9t11x_subdev_video_ops,
	.pad   = &mt9t11x_subdev_pad_ops,
};

/*
 * V4L2 subdev internal operations
 */
static int mt9t11x_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct mt9t11x_priv *priv = sd_to_mt9t11x(sd);
	struct i2c_client *client = priv->client;
	struct v4l2_mbus_framefmt *mf;

	dev_dbg(&client->dev, "%s:\n", __func__);

	mf = v4l2_subdev_get_try_format(sd, fh->pad, 0);
	mt9t11x_get_default_format(priv, mf);

	return 0;
}

static const struct v4l2_subdev_internal_ops mt9t11x_subdev_internal_ops = {
	.open = mt9t11x_open,
};

static int mt9t11x_camera_probe(struct i2c_client *client)
{
	struct mt9t11x_priv *priv = to_mt9t11x(client);
	const char          *devname;
	int                  chipid;
	int		     ret = 0;

	__mt9t11x_set_power(priv, 1);

	/*
	 * check and show chip ID
	 */
	chipid = mt9t11x_reg_read(client, 0x0000);

	switch (chipid) {
	case 0x2680:
		devname = "mt9t111";
		/*
		 * Looks like only uyvy is supported
		 * so limiting available formats.
		 */
		priv->num_formats = 1;
		break;
	case 0x2682:
		devname = "mt9t112";
		priv->num_formats = ARRAY_SIZE(mt9t11x_cfmts);
		break;
	default:
		dev_err(&client->dev, "Product ID error %04x\n", chipid);
		ret = -ENODEV;
		goto done;
	}

	dev_info(&client->dev, "%s chip ID %04x\n", devname, chipid);

done:
	__mt9t11x_set_power(priv, 0);
	return ret;
}

static struct mt9t11x_camera_info *
mt9t11x_get_pdata(struct i2c_client *client)
{
	struct mt9t11x_camera_info *pdata;
	struct device_node *endpoint;
	struct v4l2_of_endpoint *v4l2_endpoint;

	dev_dbg(&client->dev, "_get_pdata invoked\n");

	if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node)
		return client->dev.platform_data;

	dev_dbg(&client->dev, "_get_pdata: DT Node found\n");

	endpoint = of_graph_get_next_endpoint(client->dev.of_node, NULL);
	if (!endpoint)
		return NULL;

	dev_dbg(&client->dev, "_get_pdata: endpoint found\n");

	pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		goto done;

	v4l2_endpoint = &pdata->endpoint;
	v4l2_of_parse_endpoint(endpoint, v4l2_endpoint);

	if (of_property_read_u32(endpoint, "input-clock-freq", &pdata->mclk)) {
		dev_err(&client->dev, "input-clock-freq property not found\n");
		goto err_out;
	} else if (pdata->mclk > MT9T11x_MAX_EXT_CLK) {
		dev_err(&client->dev, "input-clock-freq property exceed max value\n");
		goto err_out;
	}
	dev_info(&client->dev, "input-clock-freq: %d\n", pdata->mclk);

	if (of_property_read_u32(endpoint, "pixel-clock-freq", &pdata->pclk)) {
		dev_err(&client->dev, "pixel-clock-freq property not found\n");
		goto err_out;
	} else if (pdata->pclk > MT9T11x_MAX_PIXEL_CLK) {
		dev_err(&client->dev, "pixel-clock-freq property exceed max value\n");
		goto err_out;
	}
	dev_info(&client->dev, "pixel-clock-freq: %d\n", pdata->pclk);

	/* Just copy them for now */
	if (pdata->endpoint.bus_type == V4L2_MBUS_PARALLEL)
		pdata->flags = pdata->endpoint.bus.parallel.flags;
	else
		pdata->flags = 0;
done:
	of_node_put(endpoint);
	return pdata;
err_out:
	of_node_put(endpoint);
	kfree(pdata);
	return NULL;
}

static int mt9t11x_probe(struct i2c_client *client,
			 const struct i2c_device_id *did)
{
	struct mt9t11x_priv *priv;
	struct v4l2_subdev *sd;
	struct mt9t11x_camera_info *info = mt9t11x_get_pdata(client);
	struct v4l2_mbus_framefmt mf;

	int ret;
	struct gpio_desc *gpio;

	if (!info) {
		dev_err(&client->dev, "mt9t11x: missing platform data!\n");
		return -EINVAL;
	}

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->info = info;

	mutex_init(&priv->lock);
	priv->client = client;

	gpio = devm_gpiod_get(&client->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(gpio)) {
		if (PTR_ERR(gpio) != -ENOENT)
			return PTR_ERR(gpio);
		gpio = NULL;
	}
	priv->reset_gpio = gpio;

	gpio = devm_gpiod_get(&client->dev, "powerdown", GPIOD_OUT_LOW);
	if (IS_ERR(gpio)) {
		if (PTR_ERR(gpio) != -ENOENT)
			return PTR_ERR(gpio);
		gpio = NULL;
	}
	priv->powerdown_gpio = gpio;

	gpio = devm_gpiod_get(&client->dev, "oscen", GPIOD_OUT_LOW);
	if (IS_ERR(gpio)) {
		if (PTR_ERR(gpio) != -ENOENT)
			return PTR_ERR(gpio);
		gpio = NULL;
	}
	priv->oscen_gpio = gpio;

	gpio = devm_gpiod_get(&client->dev, "bufen", GPIOD_OUT_LOW);
	if (IS_ERR(gpio)) {
		if (PTR_ERR(gpio) != -ENOENT)
			return PTR_ERR(gpio);
		gpio = NULL;
	}
	priv->bufen_gpio = gpio;

	gpio = devm_gpiod_get(&client->dev, "camen", GPIOD_OUT_LOW);
	if (IS_ERR(gpio)) {
		if (PTR_ERR(gpio) != -ENOENT)
			return PTR_ERR(gpio);
		gpio = NULL;
	}
	priv->camen_gpio = gpio;

	/* Do these here for now */
	if (priv->bufen_gpio)
		gpiod_set_value_cansleep(priv->bufen_gpio, 1);

	if (priv->oscen_gpio)
		gpiod_set_value_cansleep(priv->oscen_gpio, 1);

	if (priv->camen_gpio)
		gpiod_set_value_cansleep(priv->camen_gpio, 1);

	sd = &priv->subdev;
	v4l2_i2c_subdev_init(sd, client, &mt9t11x_subdev_ops);
	strlcpy(sd->name, DRIVER_NAME, sizeof(sd->name));

	sd->internal_ops = &mt9t11x_subdev_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;

#if defined(CONFIG_MEDIA_CONTROLLER)
	priv->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	ret = media_entity_init(&sd->entity, 1, &priv->pad, 0);
	if (ret < 0)
		return ret;
#endif
	ret = mt9t11x_initialize_controls(priv);
	if (ret < 0)
		goto err_me;

	ret = mt9t11x_camera_probe(client);
	if (ret < 0)
		goto err_ctrls;

	mt9t11x_get_default_format(priv, &mf);

	/* Calculate the PLL register value needed */
	mt9t11x_pll_calc_params(priv);

	ret = v4l2_async_register_subdev(sd);
	if (ret)
		goto err_ctrls;

	dev_info(&client->dev, "%s sensor driver registered !!\n", sd->name);

	return 0;
err_ctrls:
	v4l2_ctrl_handler_free(sd->ctrl_handler);
err_me:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	return ret;
}

static int mt9t11x_remove(struct i2c_client *client)
{
	struct mt9t11x_priv *priv = to_mt9t11x(client);

	v4l2_async_unregister_subdev(&priv->subdev);
	v4l2_device_unregister_subdev(&priv->subdev);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&priv->subdev.entity);
#endif
	v4l2_ctrl_handler_free(priv->subdev.ctrl_handler);
	kfree(priv->info);
	kfree(priv);
	return 0;
}

static const struct i2c_device_id mt9t11x_id[] = {
	{ "mt9t111", 0 },
	{ "mt9t112", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, mt9t11x_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id mt9t11x_of_match[] = {
	{ .compatible = "aptina,mt9t111", .data = (void *)MT9T111_ID},
	{ .compatible = "aptina,mt9t112", .data = (void *)MT9T112_ID},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, mt9t11x_of_match);
#endif

static struct i2c_driver mt9t11x_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name	= DRIVER_NAME,
		.of_match_table = of_match_ptr(mt9t11x_of_match),
	},
	.probe		= mt9t11x_probe,
	.remove		= mt9t11x_remove,
	.id_table	= mt9t11x_id,
};

module_i2c_driver(mt9t11x_i2c_driver);

MODULE_AUTHOR("Benoit Parrot <bparrot@ti.com>");
MODULE_DESCRIPTION("MT9T11x CMOS Image Sensor driver");
MODULE_LICENSE("GPL v2");
