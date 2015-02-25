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

#define MT9T111_ID	1
#define MT9T112_ID	2
#define MT9T11X_ID	0

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

#define MT9T11x_FLAG_VFLIP		(1 << 2)
#define MT9T11x_FLAG_HFLIP		(1 << 3)

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
			macro
************************************************************************/
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
 * macro of read/write
 */
#define ECHECKER(ret, x)		\
	do {				\
		(ret) = (x);		\
		if ((ret) < 0)		\
			return ret;	\
	} while (0)

#define mt9t11x_reg_write(ret, client, a, b) \
	ECHECKER(ret, __mt9t11x_reg_write(client, a, b))
#define mt9t11x_mcu_write(ret, client, a, b) \
	ECHECKER(ret, __mt9t11x_mcu_write(client, a, b))

#define mt9t11x_reg_mask_set(ret, client, a, b, c) \
	ECHECKER(ret, __mt9t11x_reg_mask_set(client, a, b, c))
#define mt9t11x_mcu_mask_set(ret, client, a, b, c) \
	ECHECKER(ret, __mt9t11x_mcu_mask_set(client, a, b, c))

#define mt9t11x_reg_read(ret, client, a) \
	ECHECKER(ret, __mt9t11x_reg_read(client, a))
#define mt9t11x_mcu_read(ret, client, a) \
	ECHECKER(ret, __mt9t11x_mcu_read(client, a))

/*
 * Logical address
 */
#define _VAR(id, offset, base)	(base | (id & 0x1f) << 10 | (offset & 0x3ff))
#define VAR(id, offset)  _VAR(id, offset, 0x0000)
#define VAR8(id, offset) _VAR(id, offset, 0x8000)

/************************************************************************
			struct
************************************************************************/
struct mt9t11x_format {
	enum v4l2_mbus_pixelcode code;
	enum v4l2_colorspace colorspace;
	u16 fmt;
	u16 order;
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

	struct mutex			lock;
	int				streaming;
	int				power;

	struct mt9t11x_resolution_param	resolution;
};

/************************************************************************
			supported format
************************************************************************/

static const struct mt9t11x_format mt9t11x_cfmts[] = {
	{
		.code		= V4L2_MBUS_FMT_UYVY8_2X8,
		.colorspace	= V4L2_COLORSPACE_JPEG,
		.fmt		= 1,
		.order		= 0,
	}, {
		.code		= V4L2_MBUS_FMT_VYUY8_2X8,
		.colorspace	= V4L2_COLORSPACE_JPEG,
		.fmt		= 1,
		.order		= 1,
	}, {
		.code		= V4L2_MBUS_FMT_YUYV8_2X8,
		.colorspace	= V4L2_COLORSPACE_JPEG,
		.fmt		= 1,
		.order		= 2,
	}, {
		.code		= V4L2_MBUS_FMT_YVYU8_2X8,
		.colorspace	= V4L2_COLORSPACE_JPEG,
		.fmt		= 1,
		.order		= 3,
	}, {
		.code		= V4L2_MBUS_FMT_RGB555_2X8_PADHI_LE,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.fmt		= 8,
		.order		= 2,
	}, {
		.code		= V4L2_MBUS_FMT_RGB565_2X8_LE,
		.colorspace	= V4L2_COLORSPACE_SRGB,
		.fmt		= 4,
		.order		= 2,
	},
};

/************************************************************************
			general function
************************************************************************/
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

static int __mt9t11x_reg_read(const struct i2c_client *client, u16 command)
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

static int __mt9t11x_reg_write(const struct i2c_client *client,
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

static int __mt9t11x_reg_mask_set(const struct i2c_client *client,
				  u16  command,
				  u16  mask,
				  u16  set)
{
	int val = __mt9t11x_reg_read(client, command);
	if (val < 0)
		return val;

	val &= ~mask;
	val |= set & mask;

	return __mt9t11x_reg_write(client, command, val);
}

/* mcu access */
static int __mt9t11x_mcu_read(const struct i2c_client *client, u16 command)
{
	int ret;

	ret = __mt9t11x_reg_write(client, 0x098E, command);
	if (ret < 0)
		return ret;

	return __mt9t11x_reg_read(client, 0x0990);
}

static int __mt9t11x_mcu_write(const struct i2c_client *client,
			       u16 command, u16 data)
{
	int ret;

	ret = __mt9t11x_reg_write(client, 0x098E, command);
	if (ret < 0)
		return ret;

	return __mt9t11x_reg_write(client, 0x0990, data);
}

static int __mt9t11x_mcu_mask_set(const struct i2c_client *client,
				  u16  command,
				  u16  mask,
				  u16  set)
{
	int val = __mt9t11x_mcu_read(client, command);
	if (val < 0)
		return val;

	val &= ~mask;
	val |= set & mask;

	return __mt9t11x_mcu_write(client, command, val);
}

static int mt9t11x_reset(const struct i2c_client *client)
{
	int ret;

	dev_dbg(&client->dev, "%s:\n", __func__);

	mt9t11x_reg_mask_set(ret, client, 0x001a, 0x0001, 0x0001);
	usleep_range(1000, 2000);
	mt9t11x_reg_mask_set(ret, client, 0x001a, 0x0001, 0x0000);

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

	mt9t11x_reg_read(n, client, 0x0012);
	p1 = n & 0x000f;
	n = n >> 4;
	p2 = n & 0x000f;
	n = n >> 4;
	p3 = n & 0x000f;

	mt9t11x_reg_read(n, client, 0x002a);
	p4 = n & 0x000f;
	n = n >> 4;
	p5 = n & 0x000f;
	n = n >> 4;
	p6 = n & 0x000f;

	mt9t11x_reg_read(n, client, 0x002c);
	p7 = n & 0x000f;

	mt9t11x_reg_read(n, client, 0x0010);
	m = n & 0x00ff;
	n = (n >> 8) & 0x003f;

	enable = ((6000 > ext) || (54000 < ext)) ? "X" : "";
	dev_dbg(&client->dev, "EXTCLK          : %10u K %s\n", ext, enable);

	vco = 2 * m * ext / (n+1);
	enable = ((384000 > vco) || (768000 < vco)) ? "X" : "";
	dev_dbg(&client->dev, "VCO             : %10u K %s\n", vco, enable);

	clk = vco / (p1+1) / (p2+1);
	enable = (96000 < clk) ? "X" : "";
	dev_dbg(&client->dev, "PIXCLK          : %10u K %s\n", clk, enable);

	clk = vco / (p3+1);
	enable = (768000 < clk) ? "X" : "";
	dev_dbg(&client->dev, "MIPICLK         : %10u K %s\n", clk, enable);

	clk = vco / (p6+1);
	enable = (96000 < clk) ? "X" : "";
	dev_dbg(&client->dev, "MCU CLK         : %10u K %s\n", clk, enable);

	clk = vco / (p5+1);
	enable = (54000 < clk) ? "X" : "";
	dev_dbg(&client->dev, "SOC CLK         : %10u K %s\n", clk, enable);

	clk = vco / (p4+1);
	enable = (70000 < clk) ? "X" : "";
	dev_dbg(&client->dev, "Sensor CLK      : %10u K %s\n", clk, enable);

	clk = vco / (p7+1);
	dev_dbg(&client->dev, "External sensor : %10u K\n", clk);

	clk = ext / (n+1);
	enable = ((2000 > clk) || (24000 < clk)) ? "X" : "";
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
	mt9t11x_mcu_write(ret, client, VAR(26, 0), width);
	mt9t11x_mcu_write(ret, client, VAR(26, 2), height);

	/* (Context A) Output Width/Height */
	mt9t11x_mcu_write(ret, client, VAR(18, 43), 8 + width);
	mt9t11x_mcu_write(ret, client, VAR(18, 45), 8 + height);

	/* (Context A) Start Row/Column */
	mt9t11x_mcu_write(ret, client, VAR(18, 2), 4 + hstart);
	mt9t11x_mcu_write(ret, client, VAR(18, 4), 4 + wstart);

	/* (Context A) End Row/Column */
	mt9t11x_mcu_write(ret, client, VAR(18, 6), 11 + height + hstart);
	mt9t11x_mcu_write(ret, client, VAR(18, 8), 11 + width  + wstart);

	mt9t11x_mcu_write(ret, client, VAR8(1, 0), 0x06);

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
	mt9t11x_reg_mask_set(ret, client, 0x0010, 0x3fff, val);

	/* P1/P2/P3 */
	val = ((p3 & 0x0F) << 8) |
	      ((p2 & 0x0F) << 4) |
	      ((p1 & 0x0F) << 0);
	mt9t11x_reg_mask_set(ret, client, 0x0012, 0x0fff, val);

	/* P4/P5/P6 */
	val = (0x7         << 12) |
	      ((p6 & 0x0F) <<  8) |
	      ((p5 & 0x0F) <<  4) |
	      ((p4 & 0x0F) <<  0);
	mt9t11x_reg_mask_set(ret, client, 0x002A, 0x7fff, val);

	/* P7 */
	val = (0x1         << 12) |
	      ((p7 & 0x0F) <<  0);
	mt9t11x_reg_mask_set(ret, client, 0x002C, 0x100f, val);

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
	mt9t11x_reg_mask_set(ret, client, 0x14, 1, 1);
	/* Power-down PLL */
	mt9t11x_reg_mask_set(ret, client, 0X14, 2, 0);
	/* PLL control: BYPASS PLL = 8517 */
	mt9t11x_reg_write(ret, client, 0x0014, 0x2145);

	/* Replace these registers when new timing parameters are generated */
	mt9t11x_set_pll_dividers(client,
				 priv->info->divider.m,
				 priv->info->divider.n,
				 priv->info->divider.p1,
				 priv->info->divider.p2,
				 priv->info->divider.p3,
				 priv->info->divider.p4,
				 priv->info->divider.p5,
				 priv->info->divider.p6,
				 priv->info->divider.p7);

	/* Reset Misc. Control = 536 */
	mt9t11x_reg_write(ret, client, 0x001A, 0x218);
	/* PLL control: TEST_BYPASS on = 9541 */
	mt9t11x_reg_write(ret, client, 0x0014, 0x2545);
	/* PLL control: PLL_ENABLE on = 9543 */
	mt9t11x_reg_write(ret, client, 0x0014, 0x2547);
	/* PLL control: SEL_LOCK_DET on = 9287 */
	mt9t11x_reg_write(ret, client, 0x0014, 0x2447);
	/* PLL control: TEST_BYPASS off = 8263 */
	mt9t11x_reg_write(ret, client, 0x0014, 0x2047);

	/*  Wait for the PLL to lock */
	for (i = 0; i < 1000; i++) {
		mt9t11x_reg_read(data, client, 0x0014);
		if (0x8000 & data)
			break;

		usleep_range(10000, 11000);
	}

	/* PLL control: PLL_BYPASS off = 8262 */
	mt9t11x_reg_write(ret, client, 0x0014, 0x2046);
	/* Reference clock count for 20 us = 640 */
	mt9t11x_reg_write(ret, client, 0x0022, 0x0280);
	/* Pad Slew Rate = 1911 */
	mt9t11x_reg_write(ret, client, 0x001E, 0x0777);
	/* JPEG Clock = 1024 */
	mt9t11x_reg_write(ret, client, 0x0016, 0x0400);

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
	} else
		div = 1;

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
	mt9t11x_mcu_write(ret, client, VAR(26, 160), 0x082E);
	/* PRI_B_CONFIG_JPEG_OB_TX_CONTROL_VAR */
	mt9t11x_mcu_write(ret, client, VAR(27, 160), 0x082E);
	/* SEC_A_CONFIG_JPEG_OB_TX_CONTROL_VAR */
	mt9t11x_mcu_write(ret, client, VAR(28, 160), 0x082E);
	/* SEC_B_CONFIG_JPEG_OB_TX_CONTROL_VAR */
	mt9t11x_mcu_write(ret, client, VAR(29, 160), 0x082E);
	mt9t11x_reg_mask_set(ret, client, 0x3C52, 0x0040, 0);

	/* Set correct values for Context B FIFO control */
	/* CAM1_CTX_B_RX_FIFO_TRIGGER_MARK */
	mt9t11x_mcu_write(ret, client, VAR(18, 142), 32);
	/* PRI_B_CONFIG_IO_OB_MANUAL_FLAG */
	mt9t11x_mcu_write(ret, client, VAR(27, 172), 0);

	return ret;
}

static int mt9t11x_go(const struct i2c_client *client)
{
	int data, i, ret;

	/* release MCU from standby */
	mt9t11x_reg_mask_set(ret, client, 0x0018, 0x0001, 0);

	/* wait for K26A to come out of standby */
	for (i = 0; i < 100; i++) {
		mt9t11x_reg_read(data, client, 0x0018);
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
	mt9t11x_reg_mask_set(ret, client, 0x0018, 0x0004, 0);

	/* wait for sequencer to enter preview state */
	for (i = 0; i < 100; i++) {
		mt9t11x_mcu_read(data, client, VAR8(1, 1));
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
	mt9t11x_reg_mask_set(ret, client, 0x0018, 0x0004, 1);

	return ret;
}

static int mt9t11x_custom_setup(const struct i2c_client *client)
{
	struct mt9t11x_priv *priv = to_mt9t11x(client);
	struct mt9t11x_resolution_param *resolution = &priv->resolution;
	int ret;

	/* I2C Master Clock Divider */
	mt9t11x_mcu_write(ret, client, VAR(24, 6), 0x0100);
	/* Output Width (A) */
	mt9t11x_mcu_write(ret, client, VAR(26, 0), priv->frame.width);
	/* Output Height (A) */
	mt9t11x_mcu_write(ret, client, VAR(26, 2), priv->frame.height);
	/* JPEG (A) */
	mt9t11x_mcu_write(ret, client, VAR8(26, 142), 0x00);
	/* Adaptive Output Clock (A) */
	mt9t11x_mcu_mask_set(ret, client, VAR(26, 160), 0x0040, 0x0000);
	/* Row Start (A) */
	mt9t11x_mcu_write(ret, client, VAR(18, 2), 0x000);
	/* Column Start (A) */
	mt9t11x_mcu_write(ret, client, VAR(18, 4), resolution->col_strt);
	/* Row End (A) */
	mt9t11x_mcu_write(ret, client, VAR(18, 6), resolution->row_end);
	/* Column End (A) */
	mt9t11x_mcu_write(ret, client, VAR(18, 8), resolution->col_end);
	/* Row Speed (A) */
	mt9t11x_mcu_write(ret, client, VAR(18, 10), 0x0111);
	/* Read Mode (A) */
	mt9t11x_mcu_write(ret, client, VAR(18, 12), resolution->read_mode);
	/* Fine Correction (A) */
	mt9t11x_mcu_write(ret, client, VAR(18, 15), resolution->fine_cor);
	/* Fine IT Min (A) */
	mt9t11x_mcu_write(ret, client, VAR(18, 17), resolution->fine_min);
	/* Fine IT Max Margin (A) */
	mt9t11x_mcu_write(ret, client, VAR(18, 19), resolution->fine_max);
	/* Base Frame Lines (A) */
	mt9t11x_mcu_write(ret, client, VAR(18, 29), resolution->base_lines);
	/* Min Line Length (A) */
	mt9t11x_mcu_write(ret, client, VAR(18, 31), resolution->min_lin_len);
	/* Line Length (A) */
	mt9t11x_mcu_write(ret, client, VAR(18, 37), resolution->line_len);
	/* Contex Width (A) */
	mt9t11x_mcu_write(ret, client, VAR(18, 43), resolution->con_width);
	/* Context Height (A) */
	mt9t11x_mcu_write(ret, client, VAR(18, 45), resolution->con_height);
	/* Output Width (B) */
	mt9t11x_mcu_write(ret, client, VAR(27, 0), 0x0800);
	/* Output Height (B) */
	mt9t11x_mcu_write(ret, client, VAR(27, 2), 0x0600);
	/* JPEG (B) */
	mt9t11x_mcu_write(ret, client, VAR8(27, 142), 0x01);
	/* Adaptive Output Clock (B) */
	mt9t11x_mcu_mask_set(ret, client, VAR(27, 160), 0x0040, 0x0000);
	/* Row Start (B) */
	mt9t11x_mcu_write(ret, client, VAR(18, 74), 0x004);
	/* Column Start (B) */
	mt9t11x_mcu_write(ret, client, VAR(18, 76), 0x004);
	/* Row End (B) */
	mt9t11x_mcu_write(ret, client, VAR(18, 78), 0x60B);
	/* Column End (B) */
	mt9t11x_mcu_write(ret, client, VAR(18, 80), 0x80B);
	/* Row Speed (B) */
	mt9t11x_mcu_write(ret, client, VAR(18, 82), 0x0111);
	/* Read Mode (B) */
	mt9t11x_mcu_write(ret, client, VAR(18, 84), 0x0024);
	/* Fine Correction (B) */
	mt9t11x_mcu_write(ret, client, VAR(18, 87), 0x008C);
	/* Fine IT Min (B) */
	mt9t11x_mcu_write(ret, client, VAR(18, 89), 0x01F1);
	/* Fine IT Max Margin (B) */
	mt9t11x_mcu_write(ret, client, VAR(18, 91), 0x00FF);
	/* Base Frame Lines (B) */
	mt9t11x_mcu_write(ret, client, VAR(18, 101), 0x06AE);
	/* Min Line Length (B) */
	mt9t11x_mcu_write(ret, client, VAR(18, 103), 0x0378);
	/* Line Length (B) */
	mt9t11x_mcu_write(ret, client, VAR(18, 109), 0x0A3A);
	/* Contex Width (B) */
	mt9t11x_mcu_write(ret, client, VAR(18, 115), 0x0808);
	/* Context Height (B) */
	mt9t11x_mcu_write(ret, client, VAR(18, 117), 0x0608);
	/* search_f1_50 */
	mt9t11x_mcu_write(ret, client, VAR8(18, 165), resolution->s_f1_50);
	/* search_f2_50 */
	mt9t11x_mcu_write(ret, client, VAR8(18, 166), resolution->s_f2_50);
	/* search_f1_60 */
	mt9t11x_mcu_write(ret, client, VAR8(18, 167), resolution->s_f1_60);
	/* search_f2_60 */
	mt9t11x_mcu_write(ret, client, VAR8(18, 168), resolution->s_f2_60);
	/* period_50Hz (A) */
	mt9t11x_mcu_write(ret, client, VAR8(18, 68), resolution->per_50);
	/* period_50Hz (A MSB) */
	mt9t11x_mcu_write(ret, client, VAR8(18, 303), resolution->per_50_M);
	/* period_60Hz (A) */
	mt9t11x_mcu_write(ret, client, VAR8(18, 69), resolution->per_60);
	/* period_60Hz (A MSB) */
	mt9t11x_mcu_write(ret, client, VAR8(18, 301), 0x00);
	/* period_50Hz (B) */
	mt9t11x_mcu_write(ret, client, VAR8(18, 140), 0xD2);
	/* period_50Hz (B) MSB */
	mt9t11x_mcu_write(ret, client, VAR8(18, 304), 0x00);
	/* period_60Hz (B) */
	mt9t11x_mcu_write(ret, client, VAR8(18, 141), 0xAF);
	/* period_60Hz (B) MSB */
	mt9t11x_mcu_write(ret, client, VAR8(18, 302), 0x00);
	/* FD Window Height */
	mt9t11x_mcu_write(ret, client, VAR8(14, 37), resolution->fd_w_height);
	/* Stat_min */
	mt9t11x_mcu_write(ret, client, VAR8(8, 9), 0x02);
	/* Stat_max */
	mt9t11x_mcu_write(ret, client, VAR8(8, 10), 0x03);
	/* Min_amplitude */
	mt9t11x_mcu_write(ret, client, VAR8(8, 12), 0x0A);
	/* RX FIFO Watermark (A) */
	mt9t11x_mcu_write(ret, client, VAR(18, 70), 0x0080);
	/* TX FIFO Watermark (A) */
	mt9t11x_mcu_write(ret, client, VAR(26, 170), resolution->tx_water);
	/* Max FD Zone 50 Hz */
	mt9t11x_mcu_write(ret, client, VAR(26, 21), resolution->max_fd_50);
	/* Max FD Zone 60 Hz */
	mt9t11x_mcu_write(ret, client, VAR(26, 23), resolution->max_fd_60);
	/* AE Target FD Zone */
	mt9t11x_mcu_write(ret, client, VAR(26, 45), resolution->targ_fd);
	/* RX FIFO Watermark (B) */
	mt9t11x_mcu_write(ret, client, VAR(18, 142), 0x0080);
	/* TX FIFO Watermark (B) */
	mt9t11x_mcu_write(ret, client, VAR(27, 170), 0x01D0);
	/* Refresh Sequencer Mode */
	mt9t11x_mcu_write(ret, client, VAR8(1, 0), 0x06);
	/* Refresh Sequencer */
	mt9t11x_mcu_write(ret, client, VAR8(1, 0), 0x05);

#ifdef TEST_PATTERN
	mt9t11x_mcu_write(ret, client, VAR(24, 3), 0x100);
	mt9t11x_mcu_write(ret, client, VAR(24, 37), 0x0B);
#endif

	return ret;
}

static int mt9t11x_optimal_power_consumption(const struct i2c_client *client)
{
	int ret;

	/* Analog setting B */
	mt9t11x_reg_write(ret, client, 0x3084, 0x2409);
	mt9t11x_reg_write(ret, client, 0x3092, 0x0A49);
	mt9t11x_reg_write(ret, client, 0x3094, 0x4949);
	mt9t11x_reg_write(ret, client, 0x3096, 0x4950);

	return ret;
}

static int mt9t11x_blooming_row_pattern(const struct i2c_client *client)
{
	int ret;

	/* Improve high light image quality */

	/* [CAM1_CTX_A_COARSE_ITMIN] */
	mt9t11x_mcu_write(ret, client, VAR(18, 21), 0x0004);
	/* [CAM1_CTX_B_COARSE_ITMIN] */
	mt9t11x_mcu_write(ret, client, VAR(18, 93), 0x0004);

	return ret;
}

static int mt9t11x_set_orientation(const struct i2c_client *client,
				   u32 mask, u32 flip)
{
	int ret;

	flip &= mask;

	dev_dbg(&client->dev, "%s:\n", __func__);

	/* [CAM1_CTX_A_READ_MODE] */
	mt9t11x_mcu_mask_set(ret, client, VAR(18, 12), mask, flip);
	/* [CAM1_CTX_A_PIXEL_ORDER] */
	mt9t11x_mcu_mask_set(ret, client, VAR8(18, 14), mask, flip);

	/* [CAM1_CTX_B_READ_MODE] */
	mt9t11x_mcu_mask_set(ret, client, VAR(18, 84), mask, flip);
	/* [CAM1_CTX_B_PIXEL_ORDER] */
	mt9t11x_mcu_mask_set(ret, client, VAR8(18, 86), mask, flip);

	/* [SEQ_CMD] */
	mt9t11x_mcu_write(ret, client, VAR8(1, 0), 0x06);

	return ret;
}

static int mt9t11x_init_camera_optimized(const struct i2c_client *client)
{
	int ret;

	dev_dbg(&client->dev, "%s:\n", __func__);

	ECHECKER(ret, mt9t11x_sysctl_startup(client));
	ECHECKER(ret, mt9t11x_mcu_powerup_stop_enable(client));
	ECHECKER(ret, mt9t11x_go(client));

	ECHECKER(ret, mt9t11x_custom_setup(client));

	ECHECKER(ret, mt9t11x_high_speed_overrides(client));

	ECHECKER(ret, mt9t11x_optimal_power_consumption(client));

	ECHECKER(ret, mt9t11x_blooming_row_pattern(client));

	ECHECKER(ret, mt9t11x_continue(client));

	return ret;
}

static int mt9t11x_init_setting(const struct i2c_client *client)
{
	int ret;

	dev_dbg(&client->dev, "%s:\n", __func__);

	/* Adaptive Output Clock (A) */
	mt9t11x_mcu_mask_set(ret, client, VAR(26, 160), 0x0040, 0x0000);
	/* Read Mode (A) */
	mt9t11x_mcu_write(ret, client, VAR(18, 12), 0x0024);
	/* Fine Correction (A) */
	mt9t11x_mcu_write(ret, client, VAR(18, 15), 0x00CC);
	/* Fine IT Min (A) */
	mt9t11x_mcu_write(ret, client, VAR(18, 17), 0x01f1);
	/* Fine IT Max Margin (A) */
	mt9t11x_mcu_write(ret, client, VAR(18, 19), 0x00fF);
	/* Base Frame Lines (A) */
	mt9t11x_mcu_write(ret, client, VAR(18, 29), 0x032D);
	/* Min Line Length (A) */
	mt9t11x_mcu_write(ret, client, VAR(18, 31), 0x073a);
	/* Line Length (A) */
	mt9t11x_mcu_write(ret, client, VAR(18, 37), 0x07d0);
	/* Adaptive Output Clock (B) */
	mt9t11x_mcu_mask_set(ret, client, VAR(27, 160), 0x0040, 0x0000);
	/* Row Start (B) */
	mt9t11x_mcu_write(ret, client, VAR(18, 74), 0x004);
	/* Column Start (B) */
	mt9t11x_mcu_write(ret, client, VAR(18, 76), 0x004);
	/* Row End (B) */
	mt9t11x_mcu_write(ret, client, VAR(18, 78), 0x60B);
	/* Column End (B) */
	mt9t11x_mcu_write(ret, client, VAR(18, 80), 0x80B);
	/* Fine Correction (B) */
	mt9t11x_mcu_write(ret, client, VAR(18, 87), 0x008C);
	/* Fine IT Min (B) */
	mt9t11x_mcu_write(ret, client, VAR(18, 89), 0x01F1);
	/* Fine IT Max Margin (B) */
	mt9t11x_mcu_write(ret, client, VAR(18, 91), 0x00FF);
	/* Base Frame Lines (B) */
	mt9t11x_mcu_write(ret, client, VAR(18, 101), 0x0668);
	/* Min Line Length (B) */
	mt9t11x_mcu_write(ret, client, VAR(18, 103), 0x0AF0);
	/* Line Length (B) */
	mt9t11x_mcu_write(ret, client, VAR(18, 109), 0x0AF0);
	/*
	 * Flicker Dectection registers
	 * This section should be replaced whenever new Timing file is
	 * generated.
	 * All the following registers need to be replaced
	 * Following registers are generated from Register Wizard but user can
	 * modify them. For detail see auto flicker detection tuning
	 */
	/* FD_FDPERIOD_SELECT */
	mt9t11x_mcu_write(ret, client, VAR8(8, 5), 0x01);
	/* PRI_B_CONFIG_FD_ALGO_RUN */
	mt9t11x_mcu_write(ret, client, VAR(27, 17), 0x0003);
	/* PRI_A_CONFIG_FD_ALGO_RUN */
	mt9t11x_mcu_write(ret, client, VAR(26, 17), 0x0003);
	/*
	 * AFD range detection tuning registers
	 */
	/* search_f1_50 */
	mt9t11x_mcu_write(ret, client, VAR8(18, 165), 0x25);
	/* search_f2_50 */
	mt9t11x_mcu_write(ret, client, VAR8(18, 166), 0x28);
	/* search_f1_60 */
	mt9t11x_mcu_write(ret, client, VAR8(18, 167), 0x2C);
	/* search_f2_60 */
	mt9t11x_mcu_write(ret, client, VAR8(18, 168), 0x2F);
	/* period_50Hz (A) */
	mt9t11x_mcu_write(ret, client, VAR8(18, 68), 0xBA);
	/* period_50Hz (A MSB) */
	mt9t11x_mcu_write(ret, client, VAR8(18, 303), 0x00);
	/* period_60Hz (A) */
	mt9t11x_mcu_write(ret, client, VAR8(18, 69), 0x9B);
	/* period_60Hz (A MSB) */
	mt9t11x_mcu_write(ret, client, VAR8(18, 301), 0x00);
	/* period_50Hz (B) */
	mt9t11x_mcu_write(ret, client, VAR8(18, 140), 0x82);
	/* period_50Hz (B) MSB */
	mt9t11x_mcu_write(ret, client, VAR8(18, 304), 0x00);
	/* period_60Hz (B) */
	mt9t11x_mcu_write(ret, client, VAR8(18, 141), 0x6D);
	/* period_60Hz (B) MSB */
	mt9t11x_mcu_write(ret, client, VAR8(18, 302), 0x00);
	/* FD Mode */
	mt9t11x_mcu_write(ret, client, VAR8(8, 2), 0x10);
	/* Stat_min */
	mt9t11x_mcu_write(ret, client, VAR8(8, 9), 0x02);
	/* Stat_max */
	mt9t11x_mcu_write(ret, client, VAR8(8, 10), 0x03);
	/* Min_amplitude */
	mt9t11x_mcu_write(ret, client, VAR8(8, 12), 0x0A);
	/* RX FIFO Watermark (A) */
	mt9t11x_mcu_write(ret, client, VAR(18, 70), 0x0014);
	/* RX FIFO Watermark (B) */
	mt9t11x_mcu_write(ret, client, VAR(18, 142), 0x0014);

	return ret;
}

static int mt9t11x_init_camera(const struct i2c_client *client)
{
	struct mt9t11x_priv *priv = to_mt9t11x(client);
	int ret;

	dev_dbg(&client->dev, "%s:\n", __func__);

	ECHECKER(ret, mt9t11x_sysctl_startup(client));
	ECHECKER(ret, mt9t11x_mcu_powerup_stop_enable(client));
	ECHECKER(ret, mt9t11x_go(client));

	ECHECKER(ret, mt9t11x_init_setting(client));

	ECHECKER(ret, mt9t11x_high_speed_overrides(client));

	ECHECKER(ret, mt9t11x_optimal_power_consumption(client));

	ECHECKER(ret, mt9t11x_blooming_row_pattern(client));

	ECHECKER(ret, mt9t11x_continue(client));

	ECHECKER(ret, mt9t11x_set_a_frame_size(client,
				 priv->frame.width,
				 priv->frame.height));

	return ret;
}

/************************************************************************
			v4l2_subdev_core_ops
************************************************************************/

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int mt9t11x_g_register(struct v4l2_subdev *sd,
			      struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int                ret;

	reg->size = 2;
	mt9t11x_reg_read(ret, client, reg->reg);

	reg->val = (__u64)ret;

	return 0;
}

static int mt9t11x_s_register(struct v4l2_subdev *sd,
			      const struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	mt9t11x_reg_write(ret, client, reg->reg, reg->val);

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
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register	= mt9t11x_g_register,
	.s_register	= mt9t11x_s_register,
#endif
	.s_power	= mt9t11x_s_power,
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
		mt9t11x_set_orientation(client, 0x2, (ctrl->val)?2:0);
		ret = 0;
		break;
	case V4L2_CID_HFLIP:
		mt9t11x_set_orientation(client, 0x1, (ctrl->val)?1:0);
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
			v4l2_subdev_video_ops
************************************************************************/
static int mt9t11x_set_params(struct mt9t11x_priv *priv,
			      const struct v4l2_rect *rect,
			      enum v4l2_mbus_pixelcode code)
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

	if (optimize)
		ECHECKER(ret, mt9t11x_init_camera_optimized(client));
	else
		ECHECKER(ret, mt9t11x_init_camera(client));

	/*
	 * By default data is sampled on falling edge of pixclk.
	 * Change the default to be rising edge. i.e. Invert PCLK
	 */
	mt9t11x_reg_write(ret, client, 0x3C20, 0);
	usleep_range(5000, 6000);

	mt9t11x_mcu_write(ret, client, VAR(26, 7), priv->format->fmt);
	mt9t11x_mcu_write(ret, client, VAR(26, 9), priv->format->order);
	mt9t11x_mcu_write(ret, client, VAR8(1, 0), 0x06);

	if (priv->flags & MT9T11x_FLAG_VFLIP)
		v4l2_ctrl_s_ctrl(priv->vflip, 1);

	/* Make sure H/W is consistent with current control settings */
	ECHECKER(ret, mt9t11x_set_orientation(client, 0x3,
				(v4l2_ctrl_g_ctrl(priv->vflip) << 1 |
				 v4l2_ctrl_g_ctrl(priv->hflip))));

	dev_dbg(&client->dev, "format : %d\n", priv->format->code);
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

static int mt9t11x_g_fmt(struct v4l2_subdev *sd,
			 struct v4l2_mbus_framefmt *mf)
{
	struct mt9t11x_priv *priv = sd_to_mt9t11x(sd);
	struct i2c_client *client = priv->client;

	dev_dbg(&client->dev, "%s:\n", __func__);

	mutex_lock(&priv->lock);
	mf->width	= priv->frame.width;
	mf->height	= priv->frame.height;
	mf->colorspace	= priv->format->colorspace;
	mf->code	= priv->format->code;
	mf->field	= V4L2_FIELD_NONE;
	mutex_unlock(&priv->lock);

	return 0;
}

static int mt9t11x_s_fmt(struct v4l2_subdev *sd,
			 struct v4l2_mbus_framefmt *mf)
{
	struct mt9t11x_priv *priv = sd_to_mt9t11x(sd);
	struct i2c_client *client = priv->client;
	struct v4l2_rect rect = {
		.width = mf->width,
		.height = mf->height,
		.left = priv->frame.left,
		.top = priv->frame.top,
	};
	int ret;

	dev_dbg(&client->dev, "%s:\n", __func__);

	mutex_lock(&priv->lock);
	ret = mt9t11x_set_params(priv, &rect, mf->code);

	if (!ret)
		mf->colorspace = priv->format->colorspace;
	mutex_unlock(&priv->lock);

	return ret;
}

static int mt9t11x_try_fmt(struct v4l2_subdev *sd,
			   struct v4l2_mbus_framefmt *mf)
{
	struct mt9t11x_priv *priv = sd_to_mt9t11x(sd);
	struct i2c_client *client = priv->client;
	unsigned int top, left;
	int i;

	dev_dbg(&client->dev, "%s:\n", __func__);

	mutex_lock(&priv->lock);
	for (i = 0; i < priv->num_formats; i++)
		if (mt9t11x_cfmts[i].code == mf->code)
			break;

	if (i == priv->num_formats) {
		mf->code = V4L2_MBUS_FMT_UYVY8_2X8;
		mf->colorspace = V4L2_COLORSPACE_JPEG;
	} else {
		mf->colorspace	= mt9t11x_cfmts[i].colorspace;
	}

	mt9t11x_frame_check(&mf->width, &mf->height, &left, &top);

	mf->field = V4L2_FIELD_NONE;
	mutex_unlock(&priv->lock);

	return 0;
}

static int mt9t11x_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
			   enum v4l2_mbus_pixelcode *code)
{
	struct mt9t11x_priv *priv = sd_to_mt9t11x(sd);
	struct i2c_client *client = priv->client;

	dev_dbg(&client->dev, "%s: index:%d\n", __func__, index);

	if (index >= priv->num_formats)
		return -EINVAL;

	*code = mt9t11x_cfmts[index].code;

	return 0;
}

static int mt9t11x_enum_framesizes(struct v4l2_subdev *sd,
			   struct v4l2_frmsizeenum *f)
{
	struct mt9t11x_priv *priv = sd_to_mt9t11x(sd);
	struct i2c_client *client = priv->client;

	dev_dbg(&client->dev, "%s: index:%d\n", __func__, f->index);

	if (f->index > 0)
		return -EINVAL;

	f->type = V4L2_FRMSIZE_TYPE_CONTINUOUS;
	f->stepwise.min_width = MIN_WIDTH;
	f->stepwise.max_width = MAX_WIDTH;
	f->stepwise.step_width = 1;
	f->stepwise.min_height = MIN_HEIGHT;
	f->stepwise.max_height = MAX_HEIGHT;
	f->stepwise.step_height = 1;
	return 0;
}

static struct v4l2_subdev_video_ops mt9t11x_subdev_video_ops = {
	.s_stream	= mt9t11x_s_stream,
	.g_mbus_fmt	= mt9t11x_g_fmt,
	.s_mbus_fmt	= mt9t11x_s_fmt,
	.try_mbus_fmt	= mt9t11x_try_fmt,
	.cropcap	= mt9t11x_cropcap,
	.g_crop		= mt9t11x_g_crop,
	.s_crop		= mt9t11x_s_crop,
	.enum_mbus_fmt	= mt9t11x_enum_fmt,
	.enum_framesizes = mt9t11x_enum_framesizes,
};

/************************************************************************
			i2c driver
************************************************************************/
static struct v4l2_subdev_ops mt9t11x_subdev_ops = {
	.core	= &mt9t11x_subdev_core_ops,
	.video	= &mt9t11x_subdev_video_ops,
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
	mt9t11x_reg_read(chipid, client, 0x0000);

	switch (chipid) {
	case 0x2680:
		devname = "mt9t111";
		priv->num_formats = ARRAY_SIZE(mt9t11x_cfmts);
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
	struct v4l2_rect rect = {
		.width = VGA_WIDTH,
		.height = VGA_HEIGHT,
		.left = (MAX_WIDTH - VGA_WIDTH) / 2,
		.top = (MAX_HEIGHT - VGA_HEIGHT) / 2,
	};
	int ret;
	struct gpio_desc *gpio;

	if (info == NULL) {
		dev_err(&client->dev, "mt9t11x: missing platform data!\n");
		return -EINVAL;
	}

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->info = info;

	mutex_init(&priv->lock);
	priv->client = client;

	gpio = devm_gpiod_get(&client->dev, "reset");
	if (IS_ERR(gpio)) {
		if (PTR_ERR(gpio) != -ENOENT)
			return PTR_ERR(gpio);
		else
			gpio = NULL;
	} else {
		gpiod_direction_output(gpio, 0);
	}
	priv->reset_gpio = gpio;

	gpio = devm_gpiod_get(&client->dev, "powerdown");
	if (IS_ERR(gpio)) {
		if (PTR_ERR(gpio) != -ENOENT)
			return PTR_ERR(gpio);
		else
			gpio = NULL;
	} else {
		gpiod_direction_output(gpio, 0);
	}
	priv->powerdown_gpio = gpio;

	gpio = devm_gpiod_get(&client->dev, "oscen");
	if (IS_ERR(gpio)) {
		if (PTR_ERR(gpio) != -ENOENT)
			return PTR_ERR(gpio);
		else
			gpio = NULL;
	} else {
		gpiod_direction_output(gpio, 0);
	}
	priv->oscen_gpio = gpio;

	gpio = devm_gpiod_get(&client->dev, "bufen");
	if (IS_ERR(gpio)) {
		if (PTR_ERR(gpio) != -ENOENT)
			return PTR_ERR(gpio);
		else
			gpio = NULL;
	} else {
		gpiod_direction_output(gpio, 0);
	}
	priv->bufen_gpio = gpio;

	gpio = devm_gpiod_get(&client->dev, "camen");
	if (IS_ERR(gpio)) {
		if (PTR_ERR(gpio) != -ENOENT)
			return PTR_ERR(gpio);
		else
			gpio = NULL;
	} else {
		gpiod_direction_output(gpio, 0);
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

	mt9t11x_set_params(priv, &rect, V4L2_MBUS_FMT_UYVY8_2X8);

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
	{ "mt9t11x", 0 },
	{ "mt9t111", 0 },
	{ "mt9t112", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, mt9t11x_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id mt9t11x_of_match[] = {
	{ .compatible = "aptina,mt9t11x", .data = (void *) MT9T11X_ID},
	{ .compatible = "aptina,mt9t111", .data = (void *) MT9T111_ID},
	{ .compatible = "aptina,mt9t112", .data = (void *) MT9T112_ID},
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
