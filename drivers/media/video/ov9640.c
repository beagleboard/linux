/*
 * drivers/media/video/ov9640.c
 *
 * OV9640 sensor driver
 *
 * Author: Andy Lowe (source@mvista.com)
 * Contact: Trilok Soni <soni.trilok@gmail.com>
 *
 * Copyright (C) 2004 MontaVista Software, Inc.
 * Copyright (C) 2004 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <media/v4l2-int-device.h>

#include "ov9640.h"

#define DRIVER_NAME  "ov9640"

struct ov9640_sensor {
	const struct ov9640_platform_data *pdata;
	struct v4l2_int_device *v4l2_int_device;
	struct i2c_client *i2c_client;
	struct v4l2_pix_format pix;
	struct v4l2_fract timeperframe;
	int ver;				/*ov9640 chip version*/
};

static struct ov9640_sensor ov9640;
static struct i2c_driver ov9640sensor_i2c_driver;

/* list of image formats supported by OV9640 sensor */
const static struct v4l2_fmtdesc ov9640_formats[] = {
	{
		/* Note:  V4L2 defines RGB565 as:
		 *
		 *	Byte 0			  Byte 1
		 *	g2 g1 g0 r4 r3 r2 r1 r0	  b4 b3 b2 b1 b0 g5 g4 g3
		 *
		 * We interpret RGB565 as:
		 *
		 *	Byte 0			  Byte 1
		 *	g2 g1 g0 b4 b3 b2 b1 b0	  r4 r3 r2 r1 r0 g5 g4 g3
		 */
		.description	= "RGB565, le",
		.pixelformat	= V4L2_PIX_FMT_RGB565,
	},
	{
		/* Note:  V4L2 defines RGB565X as:
		 *
		 *	Byte 0			  Byte 1
		 *	b4 b3 b2 b1 b0 g5 g4 g3	  g2 g1 g0 r4 r3 r2 r1 r0
		 *
		 * We interpret RGB565X as:
		 *
		 *	Byte 0			  Byte 1
		 *	r4 r3 r2 r1 r0 g5 g4 g3	  g2 g1 g0 b4 b3 b2 b1 b0
		 */
		.description	= "RGB565, be",
		.pixelformat	= V4L2_PIX_FMT_RGB565X,
	},
	{
		.description	= "YUYV (YUV 4:2:2), packed",
		.pixelformat	= V4L2_PIX_FMT_YUYV,
	},
	{
		.description	= "UYVY, packed",
		.pixelformat	= V4L2_PIX_FMT_UYVY,
	},
	{
		/* Note:  V4L2 defines RGB555 as:
		 *
		 *	Byte 0			  Byte 1
		 *	g2 g1 g0 r4 r3 r2 r1 r0	  x  b4 b3 b2 b1 b0 g4 g3
		 *
		 * We interpret RGB555 as:
		 *
		 *	Byte 0			  Byte 1
		 *	g2 g1 g0 b4 b3 b2 b1 b0	  x  r4 r3 r2 r1 r0 g4 g3
		 */
		.description	= "RGB555, le",
		.pixelformat	= V4L2_PIX_FMT_RGB555,
	},
	{
		/* Note:  V4L2 defines RGB555X as:
		 *
		 *	Byte 0			  Byte 1
		 *	x  b4 b3 b2 b1 b0 g4 g3	  g2 g1 g0 r4 r3 r2 r1 r0
		 *
		 * We interpret RGB555X as:
		 *
		 *	Byte 0			  Byte 1
		 *	x  r4 r3 r2 r1 r0 g4 g3	  g2 g1 g0 b4 b3 b2 b1 b0
		 */
		.description	= "RGB555, be",
		.pixelformat	= V4L2_PIX_FMT_RGB555X,
	},
};

#define NUM_CAPTURE_FORMATS ARRAY_SIZE(ov9640_formats)

/*
 * OV9640 register configuration for all combinations of pixel format and
 * image size
 */
	/* YUV (YCbCr) QQCIF */
const static struct ov9640_reg qqcif_yuv[] = {
	{ 0x12, 0x08 }, { 0x3C, 0x46 }, { 0x40, 0xC0 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x24 }, { 0x0C, 0x00 }, { 0x0D, 0x40 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x50 }, { 0x50, 0x43 }, { 0x51, 0x0D },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x19 }, { 0x53, 0x4C }, { 0x54, 0x65 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x40 }, { 0x56, 0x40 }, { 0x57, 0x40 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x0F },					/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* YUV (YCbCr) QQVGA */
const static struct ov9640_reg qqvga_yuv[] = {
	{ 0x12, 0x10 }, { 0x3C, 0x46 }, { 0x40, 0xC0 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x24 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x50 }, { 0x50, 0x43 }, { 0x51, 0x0D },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x19 }, { 0x53, 0x4C }, { 0x54, 0x65 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x40 }, { 0x56, 0x40 }, { 0x57, 0x40 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x0F },					/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* YUV (YCbCr) QCIF */
const static struct ov9640_reg qcif_yuv[] = {
	{ 0x12, 0x08 }, { 0x3C, 0x46 }, { 0x40, 0xC0 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x50 }, { 0x50, 0x43 }, { 0x51, 0x0D },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x19 }, { 0x53, 0x4C }, { 0x54, 0x65 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x40 }, { 0x56, 0x40 }, { 0x57, 0x40 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x0F },					/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* YUV (YCbCr) QVGA */
const static struct ov9640_reg qvga_yuv[] = {
	{ 0x12, 0x10 }, { 0x3C, 0x46 }, { 0x40, 0xC0 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x50 }, { 0x50, 0x43 }, { 0x51, 0x0D },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x19 }, { 0x53, 0x4C }, { 0x54, 0x65 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x40 }, { 0x56, 0x40 }, { 0x57, 0x40 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x0F },					/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* YUV (YCbCr) CIF */
const static struct ov9640_reg cif_yuv[] = {
	{ 0x12, 0x20 }, { 0x3C, 0x46 }, { 0x40, 0xC0 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x50 }, { 0x50, 0x43 }, { 0x51, 0x0D },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x19 }, { 0x53, 0x4C }, { 0x54, 0x65 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x40 }, { 0x56, 0x40 }, { 0x57, 0x40 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x0F },					/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* YUV (YCbCr) VGA */
const static struct ov9640_reg vga_yuv[] = {
	{ 0x12, 0x40 }, { 0x3C, 0x46 }, { 0x40, 0xC0 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x50 }, { 0x50, 0x43 }, { 0x51, 0x0D },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x19 }, { 0x53, 0x4C }, { 0x54, 0x65 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x40 }, { 0x56, 0x40 }, { 0x57, 0x40 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x0F },					/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* YUV (YCbCr) SXGA */
const static struct ov9640_reg sxga_yuv[] = {
	{ 0x12, 0x00 }, { 0x3C, 0x46 }, { 0x40, 0xC0 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x00 }, { 0x0D, 0x40 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x50 }, { 0x50, 0x43 }, { 0x51, 0x0D },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x19 }, { 0x53, 0x4C }, { 0x54, 0x65 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x40 }, { 0x56, 0x40 }, { 0x57, 0x40 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x0F },					/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB565 QQCIF */
const static struct ov9640_reg qqcif_565[] = {
	{ 0x12, 0x0C }, { 0x3C, 0x40 }, { 0x40, 0x10 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x24 }, { 0x0C, 0x00 }, { 0x0D, 0x40 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 },					/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB565 QQVGA */
const static struct ov9640_reg qqvga_565[] = {
	{ 0x12, 0x14 }, { 0x3C, 0x40 }, { 0x40, 0x10 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x24 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 },					/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB565 QCIF */
const static struct ov9640_reg qcif_565[] = {
	{ 0x12, 0x0C }, { 0x3C, 0x40 }, { 0x40, 0x10 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 },					/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB565 QVGA */
const static struct ov9640_reg qvga_565[] = {
	{ 0x12, 0x14 }, { 0x3C, 0x40 }, { 0x40, 0x10 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 },					/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB565 CIF */
const static struct ov9640_reg cif_565[] = {
	{ 0x12, 0x24 }, { 0x3C, 0x40 }, { 0x40, 0x10 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 },					/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB565 VGA */
const static struct ov9640_reg vga_565[] = {
	{ 0x12, 0x44 }, { 0x3C, 0x40 }, { 0x40, 0x10 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 },					/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB565 SXGA */
const static struct ov9640_reg sxga_565[] = {
	{ 0x12, 0x04 }, { 0x3C, 0x40 }, { 0x40, 0x10 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x00 }, { 0x0D, 0x40 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 },					/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB555 QQCIF */
const static struct ov9640_reg qqcif_555[] = {
	{ 0x12, 0x0C }, { 0x3C, 0x40 }, { 0x40, 0x30 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x24 }, { 0x0C, 0x00 }, { 0x0D, 0x40 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 },					/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB555 QQVGA */
const static struct ov9640_reg qqvga_555[] = {
	{ 0x12, 0x14 }, { 0x3C, 0x40 }, { 0x40, 0x30 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x24 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 },					/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB555 QCIF */
const static struct ov9640_reg qcif_555[] = {
	{ 0x12, 0x0C }, { 0x3C, 0x40 }, { 0x40, 0x30 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 },					/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB555 QVGA */
const static struct ov9640_reg qvga_555[] = {
	{ 0x12, 0x14 }, { 0x3C, 0x40 }, { 0x40, 0x30 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 },					/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB555 CIF */
const static struct ov9640_reg cif_555[] = {
	{ 0x12, 0x24 }, { 0x3C, 0x40 }, { 0x40, 0x30 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 },					/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB555 VGA */
const static struct ov9640_reg vga_555[] = {
	{ 0x12, 0x44 }, { 0x3C, 0x40 }, { 0x40, 0x30 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 },					/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB555 SXGA */
const static struct ov9640_reg sxga_555[] = {
	{ 0x12, 0x04 }, { 0x3C, 0x40 }, { 0x40, 0x30 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x00 }, { 0x0D, 0x40 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 },					/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};


#define DEF_GAIN         31
#define DEF_AUTOGAIN      1
#define DEF_EXPOSURE    154
#define DEF_AEC           1
#define DEF_FREEZE_AGCAEC 0
#define DEF_BLUE        153
#define DEF_RED         (255 - DEF_BLUE)
#define DEF_AWB           1
#define DEF_HFLIP         0
#define DEF_VFLIP         0

/* Our own specific controls */
#define V4L2_CID_FREEZE_AGCAEC		V4L2_CID_PRIVATE_BASE
#define V4L2_CID_AUTOEXPOSURE		V4L2_CID_PRIVATE_BASE + 1
#define V4L2_CID_LAST_PRIV		V4L2_CID_AUTOEXPOSURE

/*  Video controls  */
static struct vcontrol {
	struct v4l2_queryctrl qc;
	int current_value;
	u8 reg;
	u8 mask;
	u8 start_bit;
} video_control[] = {
	{
		{
			.id = V4L2_CID_GAIN,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Gain",
			.minimum = 0,
			.maximum = 63,
			.step = 1,
			.default_value = DEF_GAIN,
		},
		.current_value	= 0,
		.reg		= OV9640_GAIN,
		.mask		= 0x3f,
		.start_bit	= 0,
	},
	{
		{
			.id = V4L2_CID_AUTOGAIN,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "Auto Gain",
			.minimum = 0,
			.maximum = 1,
			.step = 0,
			.default_value = DEF_AUTOGAIN,
		},
		.current_value	= 0,
		.reg		= OV9640_COM8,
		.mask		= 0x04,
		.start_bit	= 2,
	},
	{
		{
			.id = V4L2_CID_EXPOSURE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Exposure",
			.minimum = 0,
			.maximum = 255,
			.step = 1,
			.default_value = DEF_EXPOSURE,
		},
		.current_value	= 0,
		.reg		= OV9640_AECH,
		.mask		= 0xff,
		.start_bit	= 0,
	},
	{
		{
			.id = V4L2_CID_AUTOEXPOSURE,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "Auto Exposure",
			.minimum = 0,
			.maximum = 1,
			.step = 0,
			.default_value = DEF_AEC,
		},
		.current_value	= 0,
		.reg		= OV9640_COM8,
		.mask		= 0x01,
		.start_bit	= 0,
	},
	{
		{
			.id = V4L2_CID_FREEZE_AGCAEC,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "Freeze AGC/AEC",
			.minimum = 0,
			.maximum = 1,
			.step = 0,
			.default_value = DEF_FREEZE_AGCAEC,
		},
		.current_value	= 0,
		.reg		= OV9640_COM9,
		.mask		= 0x01,
		.start_bit	= 0,
	},
	{
		{
			.id = V4L2_CID_RED_BALANCE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Red Balance",
			.minimum = 0,
			.maximum = 255,
			.step = 1,
			.default_value = DEF_RED,
		},
		.current_value	= 0,
		.reg		= OV9640_RED,
		.mask		= 0xff,
		.start_bit	= 0,
	},
	{
		{
			.id = V4L2_CID_BLUE_BALANCE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Blue Balance",
			.minimum = 0,
			.maximum = 255,
			.step = 1,
			.default_value = DEF_BLUE,
		},
		.current_value	= 0,
		.reg		= OV9640_BLUE,
		.mask		= 0xff,
		.start_bit	= 0,
	},
	{
		{
			.id = V4L2_CID_AUTO_WHITE_BALANCE,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "Auto White Balance",
			.minimum = 0,
			.maximum = 1,
			.step = 0,
			.default_value = DEF_AWB,
		},
		.current_value	= 0,
		.reg		= OV9640_COM8,
		.mask		= 0x02,
		.start_bit	= 1,
	},
	{
		{
			.id = V4L2_CID_HFLIP,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "Mirror Image",
			.minimum = 0,
			.maximum = 1,
			.step = 0,
			.default_value = DEF_HFLIP,
		},
		.current_value	= 0,
		.reg		= OV9640_MVFP,
		.mask		= 0x20,
		.start_bit	= 5,
	},
	{
		{
			.id = V4L2_CID_VFLIP,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "Vertical Flip",
			.minimum = 0,
			.maximum = 1,
			.step = 0,
			.default_value = DEF_VFLIP,
		},
		.current_value	= 0,
		.reg		= OV9640_MVFP,
		.mask		= 0x10,
		.start_bit	= 4,
	},
};

const static struct ov9640_reg *
	ov9640_reg_init[NUM_PIXEL_FORMATS][NUM_IMAGE_SIZES] =
{
 { qqcif_yuv, qqvga_yuv, qcif_yuv, qvga_yuv, cif_yuv, vga_yuv, sxga_yuv },
 { qqcif_565, qqvga_565, qcif_565, qvga_565, cif_565, vga_565, sxga_565 },
 { qqcif_555, qqvga_555, qcif_555, qvga_555, cif_555, vga_555, sxga_555 },
};


/*
 * Read a value from a register in an OV9640 sensor device.
 * The value is returned in 'val'.
 * Returns zero if successful, or non-zero otherwise.
 */
static int
ov9640_read_reg(struct i2c_client *client, u8 reg, u8 *val)
{
	int err;
	struct i2c_msg msg[1];
	unsigned char data[1];

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 1;
	msg->buf = data;
	*data = reg;
	err = i2c_transfer(client->adapter, msg, 1);
	if (err >= 0) {
		msg->flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 1);
	}
	if (err >= 0) {
		*val = *data;
		return 0;
	}
	return err;
}

/*
 * Write a value to a register in an OV9640 sensor device.
 * Returns zero if successful, or non-zero otherwise.
 */
static int
ov9640_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int err;
	struct i2c_msg msg[1];
	unsigned char data[2];

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 2;
	msg->buf = data;
	data[0] = reg;
	data[1] = val;
	err = i2c_transfer(client->adapter, msg, 1);
	if (err >= 0)
		return 0;
	return err;
}

static int
ov9640_write_reg_mask(struct i2c_client *client, u8 reg, u8 *val, u8 mask)
{
	u8 oldval, newval;
	int rc;

	if (mask == 0xff)
		newval = *val;
	else {
		/* need to do read - modify - write */
		rc = ov9640_read_reg(client, reg, &oldval);
		if (rc)
			return rc;
		oldval &= (~mask);              /* Clear the masked bits */
		*val &= mask;                  /* Enforce mask on value */
		newval = oldval | *val;        /* Set the desired bits */
	}

	/* write the new value to the register */
	rc = ov9640_write_reg(client, reg, newval);
	if (rc)
		return rc;

	rc = ov9640_read_reg(client, reg, &newval);
	if (rc)
		return rc;

	*val = newval & mask;
	return 0;
}

static int
ov9640_read_reg_mask(struct i2c_client *client, u8 reg, u8 *val, u8 mask)
{
	int rc;

	rc = ov9640_read_reg(client, reg, val);
	if (rc)
		return rc;
	(*val) &= mask;

	return 0;
}

/*
 * Initialize a list of OV9640 registers.
 * The list of registers is terminated by the pair of values
 * { OV9640_REG_TERM, OV9640_VAL_TERM }.
 * Returns zero if successful, or non-zero otherwise.
 */
static int
ov9640_write_regs(struct i2c_client *client, const struct ov9640_reg reglist[])
{
	int err;
	const struct ov9640_reg *next = reglist;

	while (!((next->reg == OV9640_REG_TERM)
		&& (next->val == OV9640_VAL_TERM))) {
		err = ov9640_write_reg(client, next->reg, next->val);
		udelay(100);
		if (err)
			return err;
		next++;
	}
	return 0;
}

/* Returns the index of the requested ID from the control structure array */
static int
find_vctrl(int id)
{
	int i;

	if (id < V4L2_CID_BASE)
		return -EDOM;

	for (i = (ARRAY_SIZE(video_control) - 1); i >= 0; i--)
		if (video_control[i].qc.id == id)
			break;
	if (i < 0)
		i = -EINVAL;
	return i;
}

/*
 * Calculate the internal clock divisor (value of the CLKRC register) of the
 * OV9640 given the image size, the frequency (in Hz) of its XCLK input and a
 * desired frame period (in seconds).  The frame period 'fper' is expressed as
 * a fraction.  The frame period is an input/output parameter.
 * Returns the value of the OV9640 CLKRC register that will yield the frame
 * period returned in 'fper' at the specified xclk frequency.  The
 * returned period will be as close to the requested period as possible.
 */
static unsigned char
ov9640_clkrc(enum image_size isize, unsigned long xclk, struct v4l2_fract *fper)
{
	unsigned long fpm, fpm_max;	/* frames per minute */
	unsigned long divisor;
	const unsigned long divisor_max = 64;
	/* FIXME
	 * clks_per_frame should come from platform data
	 */
#ifdef CONFIG_ARCH_OMAP24XX
	const static unsigned long clks_per_frame[] =
		{ 200000, 400000, 200000, 400000, 400000, 800000, 3200000 };
      /*         QQCIF   QQVGA    QCIF    QVGA  CIF     VGA	SXGA
       *         199680,400000, 199680, 400000, 399360, 800000, 3200000
       */
#else
	const static unsigned long clks_per_frame[] =
		{ 200000, 200000, 200000, 200000, 400000, 800000, 3200000 };
#endif

	if (fper->numerator > 0)
		fpm = (fper->denominator*60)/fper->numerator;
	else
		fpm = 0xffffffff;
	fpm_max = (xclk*60)/clks_per_frame[isize];
	if (fpm_max == 0)
		fpm_max = 1;
	if (fpm > fpm_max)
		fpm = fpm_max;
	if (fpm == 0)
		fpm = 1;
	divisor = fpm_max/fpm;
	if (divisor > divisor_max)
		divisor = divisor_max;
	fper->numerator = divisor*60;
	fper->denominator = fpm_max;

	/* try to reduce the fraction */
	while (!(fper->denominator % 5) && !(fper->numerator % 5)) {
		fper->numerator /= 5;
		fper->denominator /= 5;
	}
	while (!(fper->denominator % 3) && !(fper->numerator % 3)) {
		fper->numerator /= 3;
		fper->denominator /= 3;
	}
	while (!(fper->denominator % 2) && !(fper->numerator % 2)) {
		fper->numerator /= 2;
		fper->denominator /= 2;
	}
	if (fper->numerator < fper->denominator) {
		if (!(fper->denominator % fper->numerator)) {
			fper->denominator /= fper->numerator;
			fper->numerator = 1;
		}
	} else {
		if (!(fper->numerator % fper->denominator)) {
			fper->numerator /= fper->denominator;
			fper->denominator = 1;
		}
	}

	/* we set bit 7 in CLKRC to enable the digital PLL */
	return (0x80 | (divisor - 1));
}

/*
 * Find the best match for a requested image capture size.  The best match
 * is chosen as the nearest match that has the same number or fewer pixels
 * as the requested size, or the smallest image size if the requested size
 * has fewer pixels than the smallest image.
 */
static enum image_size
ov9640_find_size(unsigned int width, unsigned int height)
{
	enum image_size isize;
	unsigned long pixels = width*height;

	for (isize = QQCIF; isize < SXGA; isize++) {
		if (ov9640_sizes[isize + 1].height *
			ov9640_sizes[isize + 1].width > pixels)
			return isize;
	}
	return SXGA;
}

/*
 * Given the image capture format in pix, the nominal frame period in
 * timeperframe, calculate the required xclk frequency
 */
static unsigned long
ov9640sensor_calc_xclk(struct i2c_client *c)
{
	struct ov9640_sensor *sensor = i2c_get_clientdata(c);
	struct v4l2_fract *timeperframe = &sensor->timeperframe;
	struct v4l2_pix_format *pix = &sensor->pix;

	unsigned long tgt_xclk;			/* target xclk */
	unsigned long tgt_fpm;			/* target frames per minute */
	enum image_size isize;

	/*
	 * We use arbitrary rules to select the xclk frequency.  If the
	 * capture size is VGA and the frame rate is greater than 900
	 * frames per minute, or if the capture size is SXGA and the
	 * frame rate is greater than 450 frames per minutes, then the
	 * xclk frequency will be set to 48MHz.  Otherwise, the xclk
	 * frequency will be set to 24MHz.  If the mclk frequency is such that
	 * the target xclk frequency is not achievable, then xclk will be set
	 * as close as to the target as possible.
	 */
	tgt_fpm = (timeperframe->denominator*60)
		/ timeperframe->numerator;
	tgt_xclk = OV9640_XCLK_NOM;
	isize = ov9640_find_size(pix->width, pix->height);
	switch (isize) {
	case SXGA:
		if (tgt_fpm > 450)
			tgt_xclk = OV9640_XCLK_MAX;
		break;
	case VGA:
		if (tgt_fpm > 900)
			tgt_xclk = OV9640_XCLK_MAX;
		break;
	default:
		break;
	}
	return tgt_xclk;
}

/*
 * Configure the OV9640 for a specified image size, pixel format, and frame
 * period.  xclk is the frequency (in Hz) of the xclk input to the OV9640.
 * fper is the frame period (in seconds) expressed as a fraction.
 * Returns zero if successful, or non-zero otherwise.
 * The actual frame period is returned in fper.
 */
static int ov9640_configure(struct v4l2_int_device *s)
{
	struct ov9640_sensor *sensor = s->priv;
	struct v4l2_pix_format *pix = &sensor->pix;
	struct v4l2_fract *fper = &sensor->timeperframe;
	struct i2c_client *client = sensor->i2c_client;
	enum image_size isize;
	unsigned long xclk;

	int err;
	unsigned char clkrc;
	enum pixel_format pfmt = YUV;

	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_RGB565X:
		pfmt = RGB565;
		break;
	case V4L2_PIX_FMT_RGB555:
	case V4L2_PIX_FMT_RGB555X:
		pfmt = RGB555;
		break;
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
	default:
		pfmt = YUV;
	}

	xclk = ov9640sensor_calc_xclk(client);

	isize = ov9640_find_size(pix->width, pix->height);

	/* common register initialization */
	err = ov9640_write_regs(client, sensor->pdata->default_regs);
	if (err)
		return err;

	/* configure image size and pixel format */
	err = ov9640_write_regs(client, ov9640_reg_init[pfmt][isize]);
	if (err)
		return err;

	/* configure frame rate */
	clkrc = ov9640_clkrc(isize, xclk, fper);
	err = ov9640_write_reg(client, OV9640_CLKRC, clkrc);
	if (err)
		return err;

	return 0;
}

/*
 * Detect if an OV9640 is present, and if so which revision.
 * A device is considered to be detected if the manufacturer ID (MIDH and MIDL)
 * and the product ID (PID) registers match the expected values.
 * Any value of the version ID (VER) register is accepted.
 * Here are the version numbers we know about:
 *	0x48 --> OV9640 Revision 1 or OV9640 Revision 2
 *	0x49 --> OV9640 Revision 3
 * Returns a negative error number if no device is detected, or the
 * non-negative value of the version ID register if a device is detected.
 */
static int
ov9640_detect(struct i2c_client *client)
{
	u8 midh, midl, pid, ver;

	if (!client)
		return -ENODEV;

	if (ov9640_read_reg(client, OV9640_MIDH, &midh))
		return -ENODEV;
	if (ov9640_read_reg(client, OV9640_MIDL, &midl))
		return -ENODEV;
	if (ov9640_read_reg(client, OV9640_PID, &pid))
		return -ENODEV;
	if (ov9640_read_reg(client, OV9640_VER, &ver))
		return -ENODEV;

	if ((midh != OV9640_MIDH_MAGIC)
		|| (midl != OV9640_MIDL_MAGIC)
		|| (pid != OV9640_PID_MAGIC))
		/*
		 * We didn't read the values we expected, so
		 * this must not be an OV9640.
		 */
		return -ENODEV;

	return ver;
}

/*
 * following are sensor interface functions implemented by
 * OV9640 sensor driver.
 */
static int ioctl_queryctrl(struct v4l2_int_device *s,
				struct v4l2_queryctrl *qc)
{
	int i;

	i = find_vctrl(qc->id);
	if (i == -EINVAL) {
		qc->flags = V4L2_CTRL_FLAG_DISABLED;
		return 0;
	}
	if (i < 0)
		return -EINVAL;

	*qc = video_control[i].qc;
	return 0;
}

static int ioctl_g_ctrl(struct v4l2_int_device *s,
			     struct v4l2_control *vc)
{
	struct ov9640_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;
	int i, val;
	struct vcontrol *lvc;

	i = find_vctrl(vc->id);
	if (i < 0)
		return -EINVAL;

	lvc = &video_control[i];
	if (ov9640_read_reg_mask(client, lvc->reg, (u8 *)&val, lvc->mask))
		return -EIO;

	val = val >> lvc->start_bit;
	if (val >= 0) {
		vc->value = lvc->current_value = val;
		return 0;
	} else
		return val;
}

static int ioctl_s_ctrl(struct v4l2_int_device *s,
			     struct v4l2_control *vc)
{
	struct ov9640_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;
	struct vcontrol *lvc;
	int val = vc->value;
	int i;

	i = find_vctrl(vc->id);
	if (i < 0)
		return -EINVAL;

	lvc = &video_control[i];
	val = val << lvc->start_bit;
	if (ov9640_write_reg_mask(client, lvc->reg, (u8 *)&val, (u8)lvc->mask))
		return -EIO;

	val = val >> lvc->start_bit;
	if (val >= 0) {
		lvc->current_value = val;
		return 0;
	} else
		return val;
}

/*
 * Implement the VIDIOC_ENUM_FMT ioctl for the CAPTURE buffer type.
 */
static int ioctl_enum_fmt_cap(struct v4l2_int_device *s,
				   struct v4l2_fmtdesc *fmt)
{
	int index = fmt->index;
	enum v4l2_buf_type type = fmt->type;

	memset(fmt, 0, sizeof(*fmt));
	fmt->index = index;
	fmt->type = type;

	switch (fmt->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		if (index >= NUM_CAPTURE_FORMATS)
			return -EINVAL;
	break;
	default:
		return -EINVAL;
	}

	fmt->flags = ov9640_formats[index].flags;
	strlcpy(fmt->description, ov9640_formats[index].description,
					sizeof(fmt->description));
	fmt->pixelformat = ov9640_formats[index].pixelformat;

	return 0;
}

/*
 * Implement the VIDIOC_TRY_FMT ioctl for the CAPTURE buffer type.  This
 * ioctl is used to negotiate the image capture size and pixel format
 * without actually making it take effect.
 */
static int ioctl_try_fmt_cap(struct v4l2_int_device *s,
			     struct v4l2_format *f)
{
	enum image_size isize;
	int ifmt;
	struct v4l2_pix_format *pix = &f->fmt.pix;

	isize = ov9640_find_size(pix->width, pix->height);
	pix->width = ov9640_sizes[isize].width;
	pix->height = ov9640_sizes[isize].height;
	for (ifmt = 0; ifmt < NUM_CAPTURE_FORMATS; ifmt++) {
		if (pix->pixelformat == ov9640_formats[ifmt].pixelformat)
			break;
	}
	if (ifmt == NUM_CAPTURE_FORMATS)
		ifmt = 0;
	pix->pixelformat = ov9640_formats[ifmt].pixelformat;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = pix->width*2;
	pix->sizeimage = pix->bytesperline*pix->height;
	pix->priv = 0;
	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
	default:
		pix->colorspace = V4L2_COLORSPACE_JPEG;
		break;
	case V4L2_PIX_FMT_RGB565:
	case V4L2_PIX_FMT_RGB565X:
	case V4L2_PIX_FMT_RGB555:
	case V4L2_PIX_FMT_RGB555X:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		break;
	}
	return 0;
}

static int ioctl_s_fmt_cap(struct v4l2_int_device *s,
				struct v4l2_format *f)
{
	struct ov9640_sensor *sensor = s->priv;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	int rval;

	rval = ioctl_try_fmt_cap(s, f);
	if (rval)
		return rval;

	rval = ov9640_configure(s);

	if (!rval)
		sensor->pix = *pix;

	return rval;
}

static int ioctl_g_fmt_cap(struct v4l2_int_device *s,
				struct v4l2_format *f)
{
	struct ov9640_sensor *sensor = s->priv;

	f->fmt.pix = sensor->pix;

	return 0;
}

static int ioctl_g_parm(struct v4l2_int_device *s,
			     struct v4l2_streamparm *a)
{
	struct ov9640_sensor *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(a, 0, sizeof(*a));
	a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	cparm->capability = V4L2_CAP_TIMEPERFRAME;
	cparm->timeperframe = sensor->timeperframe;

	return 0;
}

static int ioctl_s_parm(struct v4l2_int_device *s,
			     struct v4l2_streamparm *a)
{
	struct ov9640_sensor *sensor = s->priv;
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	struct v4l2_fract timeperframe_old;
	int rval;

	timeperframe_old = sensor->timeperframe;
	sensor->timeperframe = *timeperframe;

	rval = ov9640_configure(s);

	if (rval)
		sensor->timeperframe = timeperframe_old;
	else
		*timeperframe = sensor->timeperframe;

	return rval;
}

static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	struct ov9640_sensor *sensor = s->priv;
	struct i2c_client *client = sensor->i2c_client;
	u32 xclk;	/* target xclk */
	int rval;

	rval = sensor->pdata->ifparm(p);
	if (rval)
		return rval;

	xclk = ov9640sensor_calc_xclk(client);

	p->u.bt656.clock_curr = xclk;

	return 0;
}

static int ioctl_s_power(struct v4l2_int_device *s, int on)
{
	struct ov9640_sensor *sensor = s->priv;

	return sensor->pdata->power_set(on);
}

static int ioctl_init(struct v4l2_int_device *s)
{
	return ov9640_configure(s);
}

static int ioctl_dev_exit(struct v4l2_int_device *s)
{
	return 0;
}

static int ioctl_dev_init(struct v4l2_int_device *s)
{
	struct ov9640_sensor *sensor = s->priv;
	struct i2c_client *c = sensor->i2c_client;
	int err;

	err = ov9640_detect(c);
	if (err < 0) {
		dev_err(&c->dev, "Unable to detect " DRIVER_NAME " sensor\n");
		return err;
	}

	sensor->ver = err;
	pr_info(DRIVER_NAME " chip version 0x%02x detected\n", sensor->ver);

	return 0;
}

static struct v4l2_int_ioctl_desc ov9640_ioctl_desc[] = {
	{ vidioc_int_dev_init_num,
	  (v4l2_int_ioctl_func *)ioctl_dev_init },
	{ vidioc_int_dev_exit_num,
	  (v4l2_int_ioctl_func *)ioctl_dev_exit },
	{ vidioc_int_s_power_num,
	  (v4l2_int_ioctl_func *)ioctl_s_power },
	{ vidioc_int_g_ifparm_num,
	  (v4l2_int_ioctl_func *)ioctl_g_ifparm },
	{ vidioc_int_init_num,
	  (v4l2_int_ioctl_func *)ioctl_init },
	{ vidioc_int_enum_fmt_cap_num,
	  (v4l2_int_ioctl_func *)ioctl_enum_fmt_cap },
	{ vidioc_int_try_fmt_cap_num,
	  (v4l2_int_ioctl_func *)ioctl_try_fmt_cap },
	{ vidioc_int_g_fmt_cap_num,
	  (v4l2_int_ioctl_func *)ioctl_g_fmt_cap },
	{ vidioc_int_s_fmt_cap_num,
	  (v4l2_int_ioctl_func *)ioctl_s_fmt_cap },
	{ vidioc_int_g_parm_num,
	  (v4l2_int_ioctl_func *)ioctl_g_parm },
	{ vidioc_int_s_parm_num,
	  (v4l2_int_ioctl_func *)ioctl_s_parm },
	{ vidioc_int_queryctrl_num,
	  (v4l2_int_ioctl_func *)ioctl_queryctrl },
	{ vidioc_int_g_ctrl_num,
	  (v4l2_int_ioctl_func *)ioctl_g_ctrl },
	{ vidioc_int_s_ctrl_num,
	  (v4l2_int_ioctl_func *)ioctl_s_ctrl },
};

static struct v4l2_int_slave ov9640_slave = {
	.ioctls		= ov9640_ioctl_desc,
	.num_ioctls	= ARRAY_SIZE(ov9640_ioctl_desc),
};

static struct v4l2_int_device ov9640_int_device = {
	.module	= THIS_MODULE,
	.name	= DRIVER_NAME,
	.priv	= &ov9640,
	.type	= v4l2_int_type_slave,
	.u	= {
		.slave = &ov9640_slave,
	},
};

static int __init
ov9640_probe(struct i2c_client *client)
{
	struct ov9640_sensor *sensor = &ov9640;
	int err;

	if (i2c_get_clientdata(client))
		return -EBUSY;

	sensor->pdata = client->dev.platform_data;

	if (!sensor->pdata || !sensor->pdata->default_regs) {
		dev_err(&client->dev, "no platform data?\n");
		return -ENODEV;
	}

	sensor->v4l2_int_device = &ov9640_int_device;
	sensor->i2c_client = client;

	i2c_set_clientdata(client, sensor);

	/* Make the default capture format QCIF RGB565 */
	sensor->pix.width = ov9640_sizes[QCIF].width;
	sensor->pix.height = ov9640_sizes[QCIF].height;
	sensor->pix.pixelformat = V4L2_PIX_FMT_RGB565;

	err = v4l2_int_device_register(sensor->v4l2_int_device);
	if (err)
		i2c_set_clientdata(client, NULL);

	return 0;
}

static int __exit
ov9640_remove(struct i2c_client *client)
{
	struct ov9640_sensor *sensor = i2c_get_clientdata(client);

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	v4l2_int_device_unregister(sensor->v4l2_int_device);
	i2c_set_clientdata(client, NULL);

	return 0;
}

static struct i2c_driver ov9640sensor_i2c_driver = {
	.driver = {
		.name	= DRIVER_NAME,
	},
	.probe	= ov9640_probe,
	.remove	= __exit_p(ov9640_remove),
};

static struct ov9640_sensor ov9640 = {
	.timeperframe = {
		.numerator = 1,
		.denominator = 15,
	},
};

static int ov9640sensor_init(void)
{
	int err;

	err = i2c_add_driver(&ov9640sensor_i2c_driver);
	if (err) {
		printk(KERN_ERR "Failed to register" DRIVER_NAME ".\n");
		return err;
	}
	return 0;
}
module_init(ov9640sensor_init);

static void __exit ov9640sensor_cleanup(void)
{
	i2c_del_driver(&ov9640sensor_i2c_driver);
}
module_exit(ov9640sensor_cleanup);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("OV9640 camera sensor driver");
