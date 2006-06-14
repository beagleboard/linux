
/*
 * drivers/media/video/omap/sensor_ov9640.c
 *
 * Ov9640 Sensor driver for OMAP camera sensor interface
 *
 * Author: Andy Lowe (source@mvista.com)
 *
 * Copyright (C) 2004 MontaVista Software, Inc.
 * Copyright (C) 2004 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License 
 * version 2. This program is licensed "as is" without any warranty of any 
 * kind, whether express or implied.
 */

#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/videodev.h>
#include <media/video-buf.h>
#include <linux/delay.h>
#include <asm/mach-types.h>

#include "sensor_if.h"
#include "ov9640.h"
#include "h3sensorpower.h"
#include "h4sensorpower.h"

#define CAMERA_OV9640
#ifdef CAMERA_OV9640

struct ov9640_sensor {
	/* I2C parameters */
	struct i2c_client client;
	int ver; /* OV9640 version */
};

static struct ov9640_sensor ov9640;

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
	},{
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
	},{
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
	},{
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
	}
};

#define NUM_CAPTURE_FORMATS ARRAY_SIZE(ov9640_formats)
#ifdef CONFIG_ARCH_OMAP24XX
#define NUM_OVERLAY_FORMATS 4
#else
#define NUM_OVERLAY_FORMATS 2
#endif

/* register initialization tables for OV9640 */

#define OV9640_REG_TERM 0xFF	/* terminating list entry for reg */
#define OV9640_VAL_TERM 0xFF	/* terminating list entry for val */

/* common OV9640 register initialization for all image sizes, pixel formats, 
 * and frame rates
 */
const static struct ov9640_reg ov9640_common[] = {
#ifdef CONFIG_ARCH_OMAP24XX
	{ 0x12, 0x80 }, { 0x11, 0x80 }, { 0x13, 0x8F },	/* COM7, CLKRC, COM8 */
	{ 0x01, 0x80 }, { 0x02, 0x80 }, { 0x04, 0x00 },	/* BLUE, RED, COM1 */
	{ 0x0E, 0x81 }, { 0x0F, 0x4F }, { 0x14, 0x4A },	/* COM5, COM6, COM9 */
#else
	{ 0x12, 0x80 }, { 0x11, 0x80 }, { 0x13, 0x88 },	/* COM7, CLKRC, COM8 */
	{ 0x01, 0x58 }, { 0x02, 0x24 }, { 0x04, 0x00 },	/* BLUE, RED, COM1 */
	{ 0x0E, 0x81 }, { 0x0F, 0x4F }, { 0x14, 0xcA },	/* COM5, COM6, COM9 */
#endif
	{ 0x16, 0x02 }, { 0x1B, 0x01 }, { 0x24, 0x70 },	/* ?, PSHFT, AEW */
	{ 0x25, 0x68 }, { 0x26, 0xD3 }, { 0x27, 0x90 },	/* AEB, VPT, BBIAS */
	{ 0x2A, 0x00 }, { 0x2B, 0x00 }, { 0x32, 0x24 },	/* EXHCH, EXHCL, HREF */
	{ 0x33, 0x02 }, { 0x37, 0x02 }, { 0x38, 0x13 },	/* CHLF, ADC, ACOM */
	{ 0x39, 0xF0 }, { 0x3A, 0x00 }, { 0x3B, 0x01 },	/* OFON, TSLB, COM11 */
	{ 0x3D, 0x90 }, { 0x3E, 0x02 }, { 0x3F, 0xF2 },	/* COM13, COM14, EDGE */
	{ 0x41, 0x02 }, { 0x42, 0xC8 },		/* COM16, COM17 */
	{ 0x43, 0xF0 }, { 0x44, 0x10 }, { 0x45, 0x6C },	/* ?, ?, ? */
	{ 0x46, 0x6C }, { 0x47, 0x44 }, { 0x48, 0x44 },	/* ?, ?, ? */
	{ 0x49, 0x03 }, { 0x59, 0x49 }, { 0x5A, 0x94 },	/* ?, ?, ? */
	{ 0x5B, 0x46 }, { 0x5C, 0x84 }, { 0x5D, 0x5C },	/* ?, ?, ? */
	{ 0x5E, 0x08 }, { 0x5F, 0x00 }, { 0x60, 0x14 },	/* ?, ?, ? */
	{ 0x61, 0xCE },					/* ? */
	{ 0x62, 0x70 }, { 0x63, 0x00 }, { 0x64, 0x04 },	/* LCC1, LCC2, LCC3 */
	{ 0x65, 0x00 }, { 0x66, 0x00 },			/* LCC4, LCC5 */
	{ 0x69, 0x00 }, { 0x6A, 0x3E }, { 0x6B, 0x3F },	/* HV, MBD, DBLV */
	{ 0x6C, 0x40 }, { 0x6D, 0x30 }, { 0x6E, 0x4B },	/* GSP1, GSP2, GSP3 */
	{ 0x6F, 0x60 }, { 0x70, 0x70 }, { 0x71, 0x70 },	/* GSP4, GSP5, GSP6 */
	{ 0x72, 0x70 }, { 0x73, 0x70 }, { 0x74, 0x60 },	/* GSP7, GSP8, GSP9 */
	{ 0x75, 0x60 }, { 0x76, 0x50 }, { 0x77, 0x48 },	/* GSP10,GSP11,GSP12 */
	{ 0x78, 0x3A }, { 0x79, 0x2E }, { 0x7A, 0x28 },	/* GSP13,GSP14,GSP15 */
	{ 0x7B, 0x22 }, { 0x7C, 0x04 }, { 0x7D, 0x07 },	/* GSP16,GST1, GST2 */
	{ 0x7E, 0x10 }, { 0x7F, 0x28 }, { 0x80, 0x36 },	/* GST3, GST4, GST5 */
	{ 0x81, 0x44 }, { 0x82, 0x52 }, { 0x83, 0x60 },	/* GST6, GST7, GST8 */
	{ 0x84, 0x6C }, { 0x85, 0x78 }, { 0x86, 0x8C },	/* GST9, GST10,GST11 */
	{ 0x87, 0x9E }, { 0x88, 0xBB }, { 0x89, 0xD2 },	/* GST12,GST13,GST14 */
#ifdef CONFIG_ARCH_OMAP24XX
	{ 0x8A, 0xE6 }, { 0x13, 0x8F }, { 0x00, 0x7F },	/* GST15, COM8 */
#else
	{ 0x8A, 0xE6 }, { 0x13, 0xaF }, { 0x15, 0x02 },	/* GST15, COM8 */
	{ 0x22, 0x8a }, /* GROS */
#endif
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};

/* OV9640 register configuration for all combinations of pixel format and 
 * image size
 */
	/* YUV (YCbCr) QQCIF */
const static struct ov9640_reg qqcif_yuv[] = {
	{ 0x12, 0x08 }, { 0x3C, 0x46 }, { 0x40, 0xC0 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x24 }, { 0x0C, 0x00 }, { 0x0D, 0x40 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x50 }, { 0x50, 0x43 }, { 0x51, 0x0D },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x19 }, { 0x53, 0x4C }, { 0x54, 0x65 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x40 }, { 0x56, 0x40 }, { 0x57, 0x40 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x0F }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* YUV (YCbCr) QQVGA */
const static struct ov9640_reg qqvga_yuv[] = {
	{ 0x12, 0x10 }, { 0x3C, 0x46 }, { 0x40, 0xC0 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x24 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x50 }, { 0x50, 0x43 }, { 0x51, 0x0D },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x19 }, { 0x53, 0x4C }, { 0x54, 0x65 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x40 }, { 0x56, 0x40 }, { 0x57, 0x40 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x0F }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* YUV (YCbCr) QCIF */
const static struct ov9640_reg qcif_yuv[] = {
	{ 0x12, 0x08 }, { 0x3C, 0x46 }, { 0x40, 0xC0 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x50 }, { 0x50, 0x43 }, { 0x51, 0x0D },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x19 }, { 0x53, 0x4C }, { 0x54, 0x65 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x40 }, { 0x56, 0x40 }, { 0x57, 0x40 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x0F }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* YUV (YCbCr) QVGA */
const static struct ov9640_reg qvga_yuv[] = {
	{ 0x12, 0x10 }, { 0x3C, 0x46 }, { 0x40, 0xC0 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x50 }, { 0x50, 0x43 }, { 0x51, 0x0D },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x19 }, { 0x53, 0x4C }, { 0x54, 0x65 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x40 }, { 0x56, 0x40 }, { 0x57, 0x40 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x0F }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* YUV (YCbCr) CIF */
const static struct ov9640_reg cif_yuv[] = {
	{ 0x12, 0x20 }, { 0x3C, 0x46 }, { 0x40, 0xC0 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x50 }, { 0x50, 0x43 }, { 0x51, 0x0D },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x19 }, { 0x53, 0x4C }, { 0x54, 0x65 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x40 }, { 0x56, 0x40 }, { 0x57, 0x40 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x0F }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* YUV (YCbCr) VGA */
const static struct ov9640_reg vga_yuv[] = {
	{ 0x12, 0x40 }, { 0x3C, 0x46 }, { 0x40, 0xC0 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x50 }, { 0x50, 0x43 }, { 0x51, 0x0D },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x19 }, { 0x53, 0x4C }, { 0x54, 0x65 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x40 }, { 0x56, 0x40 }, { 0x57, 0x40 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x0F }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* YUV (YCbCr) SXGA */
const static struct ov9640_reg sxga_yuv[] = {
	{ 0x12, 0x00 }, { 0x3C, 0x46 }, { 0x40, 0xC0 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x00 }, { 0x0D, 0x40 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x50 }, { 0x50, 0x43 }, { 0x51, 0x0D },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x19 }, { 0x53, 0x4C }, { 0x54, 0x65 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x40 }, { 0x56, 0x40 }, { 0x57, 0x40 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x0F }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB565 QQCIF */
const static struct ov9640_reg qqcif_565[] = {
	{ 0x12, 0x0C }, { 0x3C, 0x40 }, { 0x40, 0x10 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x24 }, { 0x0C, 0x00 }, { 0x0D, 0x40 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB565 QQVGA */
const static struct ov9640_reg qqvga_565[] = {
	{ 0x12, 0x14 }, { 0x3C, 0x40 }, { 0x40, 0x10 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x24 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB565 QCIF */
const static struct ov9640_reg qcif_565[] = {
	{ 0x12, 0x0C }, { 0x3C, 0x40 }, { 0x40, 0x10 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB565 QVGA */
const static struct ov9640_reg qvga_565[] = {
	{ 0x12, 0x14 }, { 0x3C, 0x40 }, { 0x40, 0x10 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB565 CIF */
const static struct ov9640_reg cif_565[] = {
	{ 0x12, 0x24 }, { 0x3C, 0x40 }, { 0x40, 0x10 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB565 VGA */
const static struct ov9640_reg vga_565[] = {
	{ 0x12, 0x44 }, { 0x3C, 0x40 }, { 0x40, 0x10 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB565 SXGA */
const static struct ov9640_reg sxga_565[] = {
	{ 0x12, 0x04 }, { 0x3C, 0x40 }, { 0x40, 0x10 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x00 }, { 0x0D, 0x40 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB555 QQCIF */
const static struct ov9640_reg qqcif_555[] = {
	{ 0x12, 0x0C }, { 0x3C, 0x40 }, { 0x40, 0x30 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x24 }, { 0x0C, 0x00 }, { 0x0D, 0x40 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB555 QQVGA */
const static struct ov9640_reg qqvga_555[] = {
	{ 0x12, 0x14 }, { 0x3C, 0x40 }, { 0x40, 0x30 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x24 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB555 QCIF */
const static struct ov9640_reg qcif_555[] = {
	{ 0x12, 0x0C }, { 0x3C, 0x40 }, { 0x40, 0x30 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB555 QVGA */
const static struct ov9640_reg qvga_555[] = {
	{ 0x12, 0x14 }, { 0x3C, 0x40 }, { 0x40, 0x30 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB555 CIF */
const static struct ov9640_reg cif_555[] = {
	{ 0x12, 0x24 }, { 0x3C, 0x40 }, { 0x40, 0x30 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB555 VGA */
const static struct ov9640_reg vga_555[] = {
	{ 0x12, 0x44 }, { 0x3C, 0x40 }, { 0x40, 0x30 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x04 }, { 0x0D, 0xC0 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 }, 				/* MTXS */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};
	/* RGB555 SXGA */
const static struct ov9640_reg sxga_555[] = {
	{ 0x12, 0x04 }, { 0x3C, 0x40 }, { 0x40, 0x30 },	/* COM7, COM12, COM15 */
	{ 0x04, 0x00 }, { 0x0C, 0x00 }, { 0x0D, 0x40 },	/* COM1, COM3, COM4 */
	{ 0x4F, 0x71 }, { 0x50, 0x3E }, { 0x51, 0x0C },	/* MTX1, MTX2, MTX3 */
	{ 0x52, 0x33 }, { 0x53, 0x72 }, { 0x54, 0x00 },	/* MTX4, MTX5, MTX6 */
	{ 0x55, 0x2B }, { 0x56, 0x66 }, { 0x57, 0xD2 },	/* MTX7, MTX8, MTX9 */
	{ 0x58, 0x65 }, 				/* MTXS */
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
#define V4L2_CID_FREEZE_AGCAEC V4L2_CID_PRIVATE_BASE+0
#define V4L2_CID_AUTOEXPOSURE V4L2_CID_PRIVATE_BASE+1
#define V4L2_CID_LAST_PRIV  V4L2_CID_AUTOEXPOSURE

/*  Video controls  */
static struct vcontrol {
        struct v4l2_queryctrl qc;
        int current_value;
        u8 reg;
        u8 mask;
        u8 start_bit;
} control[] = {
        { { V4L2_CID_GAIN, V4L2_CTRL_TYPE_INTEGER, "Gain", 0, 63, 1,
            DEF_GAIN },
          0, OV9640_GAIN, 0x3f, 0 },
        { { V4L2_CID_AUTOGAIN, V4L2_CTRL_TYPE_BOOLEAN, "Auto Gain", 0, 1, 0,
            DEF_AUTOGAIN },
          0, OV9640_COM8, 0x04, 2 },
        { { V4L2_CID_EXPOSURE, V4L2_CTRL_TYPE_INTEGER, "Exposure", 0, 255, 1,
            DEF_EXPOSURE },
          0, OV9640_AECH, 0xff, 0 },
        { { V4L2_CID_AUTOEXPOSURE, V4L2_CTRL_TYPE_BOOLEAN, "Auto Exposure", 0, 1, 0,
            DEF_AEC },
          0, OV9640_COM8, 0x01, 0 },
        { { V4L2_CID_FREEZE_AGCAEC, V4L2_CTRL_TYPE_BOOLEAN, "Freeze AGC/AEC", 0,1,0,
            DEF_FREEZE_AGCAEC },
          0, OV9640_COM9, 0x01, 0 },
        { { V4L2_CID_RED_BALANCE, V4L2_CTRL_TYPE_INTEGER, "Red Balance", 0, 255, 1,
            DEF_RED },
          0, OV9640_RED, 0xff, 0 },
        { { V4L2_CID_BLUE_BALANCE, V4L2_CTRL_TYPE_INTEGER, "Blue Balance", 0, 255, 1,
            DEF_BLUE },
          0, OV9640_BLUE, 0xff, 0 },
        { { V4L2_CID_AUTO_WHITE_BALANCE, V4L2_CTRL_TYPE_BOOLEAN, "Auto White Balance", 0,1,0,
            DEF_AWB },
          0, OV9640_COM8, 0x02, 1 },
        { { V4L2_CID_HFLIP, V4L2_CTRL_TYPE_BOOLEAN, "Mirror Image", 0, 1, 0,
            DEF_HFLIP },
          0, OV9640_MVFP, 0x20, 5 },
        { { V4L2_CID_VFLIP, V4L2_CTRL_TYPE_BOOLEAN, "Vertical Flip", 0, 1, 0,
            DEF_VFLIP },
          0, OV9640_MVFP, 0x10, 4 },
};

#define NUM_CONTROLS ARRAY_SIZE(control)

const static struct ov9640_reg *
	ov9640_reg_init[NUM_PIXEL_FORMATS][NUM_IMAGE_SIZES] =
{
 { qqcif_yuv, qqvga_yuv, qcif_yuv, qvga_yuv, cif_yuv, vga_yuv, sxga_yuv },
 { qqcif_565, qqvga_565, qcif_565, qvga_565, cif_565, vga_565, sxga_565 },
 { qqcif_555, qqvga_555, qcif_555, qvga_555, cif_555, vga_555, sxga_555 },
};


/* 
 * Read a value from a register in an OV9640 sensor device.  The value is 
 * returned in 'val'.
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

/* Write a value to a register in an OV9640 sensor device.
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
		if ((rc = ov9640_read_reg(client, reg, &oldval)))
			return rc;
		oldval &= (~mask);              /* Clear the masked bits */
		*val &= mask;                  /* Enforce mask on value */
		newval = oldval | *val;        /* Set the desired bits */
	}

	/* write the new value to the register */
	if ((rc = ov9640_write_reg(client, reg, newval)))
		return rc;

	if ((rc = ov9640_read_reg(client, reg, &newval)))
		return rc;

	*val = newval & mask;
	return 0;
}

static int 
ov9640_read_reg_mask(struct i2c_client *client, u8 reg, u8 *val, u8 mask)
{
	int rc;

	if ((rc = ov9640_read_reg(client, reg, val)))
		return rc;
	(*val) &= mask;

	return 0;
}

/* Initialize a list of OV9640 registers.
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
		&& (next->val == OV9640_VAL_TERM)))
	{
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

	for (i = NUM_CONTROLS - 1; i >= 0; i--)
		if (control[i].qc.id == id)
			break;
	if (i < 0)
		i = -EINVAL;
	return i;
}

/* Calculate the internal clock divisor (value of the CLKRC register) of the 
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
#ifdef CONFIG_ARCH_OMAP24XX
	const static unsigned long clks_per_frame[] =
		{ 200000, 400000, 200000, 400000, 400000, 800000, 3200000 };
      /*         QQCIF   QQVGA    QCIF    QVGA  CIF     VGA   	SXGA
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
	}
	else {
		if (!(fper->numerator % fper->denominator)) {
			fper->numerator /= fper->denominator;
			fper->denominator = 1;
		}
	}

	/* we set bit 7 in CLKRC to enable the digital PLL */
	return (0x80 | (divisor - 1));
}

/* Configure the OV9640 for a specified image size, pixel format, and frame 
 * period.  xclk is the frequency (in Hz) of the xclk input to the OV9640.  
 * fper is the frame period (in seconds) expressed as a fraction.
 * Returns zero if successful, or non-zero otherwise.
 * The actual frame period is returned in fper.
 */
static int
ov9640_configure(struct i2c_client *client, 
	enum image_size isize, 
	enum pixel_format pfmt,
	unsigned long xclk,
	struct v4l2_fract *fper)
{
	int err;
	unsigned char clkrc;

	/* common register initialization */
	err = ov9640_write_regs(client, ov9640_common);
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

static int
ov9640_powerup(void)
{
	int err;

	if (machine_is_omap_h2())
		return 0;

	if (machine_is_omap_h3()) {
		err = h3_sensor_powerup();
		if (err)
			return err;
	}

	if (machine_is_omap_h4()) {
		err = h4_sensor_powerup();
		if (err)
			return err;
	}

	return 0;
}
static int
ov9640_powerdown(void)
{
	int err;

	if (machine_is_omap_h2())
		return 0;

	if (machine_is_omap_h3()) {
		err = h3_sensor_powerdown();
		if (err)
			return err;
	}

	if (machine_is_omap_h4()) {
		err = h4_sensor_powerdown();
		if (err)
			return err;
	}

	return 0;
}

static int
ov9640sensor_power_on(void *priv)
{
	return ov9640_powerup();
}

static int
ov9640sensor_power_off(void *priv)
{
	return ov9640_powerdown();
}

/* Detect if an OV9640 is present, and if so which revision. 
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
	{
		/* We didn't read the values we expected, so 
		 * this must not be an OV9640.
		 */
		return -ENODEV;
	}
	return ver;
}

static struct i2c_driver ov9640sensor_i2c_driver;

/* This function registers an I2C client via i2c_attach_client() for an OV9640 
 * sensor device.  If 'probe' is non-zero, then the I2C client is only 
 * registered if the device can be detected.  If 'probe' is zero, then no 
 * device detection is attempted and the I2C client is always registered.
 * Returns zero if an I2C client is successfully registered, or non-zero 
 * otherwise.
 */
static int 
ov9640_i2c_attach_client(struct i2c_adapter *adap, int addr, int probe)
{
	struct ov9640_sensor *sensor = &ov9640;
	struct i2c_client *client = &sensor->client;
	int err;

	if (client->adapter)
		return -EBUSY;	/* our client is already attached */

	client->addr = addr;
	client->driver = &ov9640sensor_i2c_driver;
	client->adapter = adap;

	err = i2c_attach_client(client);
	if (err) {
		client->adapter = NULL;
		return err;
	}

	if (probe) {
		err = ov9640_detect(client);
		if (err < 0) {
			i2c_detach_client(client);
			client->adapter = NULL;
			return err;
		}
		sensor->ver = err;
	}
	return 0;
}

/* This function is called by i2c_del_adapter() and i2c_del_driver() 
 * if the adapter or driver with which this I2C client is associated is 
 * removed.  This function unregisters the client via i2c_detach_client().
 * Returns zero if the client is successfully detached, or non-zero 
 * otherwise.
 */
static int 
ov9640_i2c_detach_client(struct i2c_client *client)
{
	int err;

	if (!client->adapter)
		return -ENODEV;	/* our client isn't attached */

	err = i2c_detach_client(client);
	client->adapter = NULL;

	return err;
}

/* This function will be called for each registered I2C bus adapter when our 
 * I2C driver is registered via i2c_add_driver().  It will also be called 
 * whenever a new I2C adapter is registered after our I2C driver is registered.
 * This function probes the specified I2C bus adapter to determine if an 
 * OV9640 sensor device is present.  If a device is detected, an I2C client 
 * is registered for it via ov9640_i2c_attach_client().  Note that we can't use 
 * the standard i2c_probe() function to look for the sensor because the OMAP 
 * I2C controller doesn't support probing.
 * Returns zero if an OV9640 device is detected and an I2C client successfully 
 * registered for it, or non-zero otherwise.
 */
static int 
ov9640_i2c_probe_adapter(struct i2c_adapter *adap)
{
	return ov9640_i2c_attach_client(adap, OV9640_I2C_ADDR, 1);
}

/* Find the best match for a requested image capture size.  The best match 
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
		if (ov9640_sizes[isize + 1].height*
			ov9640_sizes[isize + 1].width > pixels)
		{
			return isize;
		}
	}
	return SXGA;
}

/* following are sensor interface functions implemented by 
 * OV9640 sensor driver.
 */
static int
ov9640sensor_query_control(struct v4l2_queryctrl *qc, void *priv)
{
	int i;

	i = find_vctrl (qc->id);
	if (i == -EINVAL) {
		qc->flags = V4L2_CTRL_FLAG_DISABLED;
		return 0;
	}
	if (i < 0)
		return -EINVAL;

	*qc = control[i].qc;
	return 0;
}

static int
ov9640sensor_get_control(struct v4l2_control *vc, void *priv)
{
	struct ov9640_sensor *sensor = (struct ov9640_sensor *) priv;
	struct i2c_client *client = &sensor->client;
	int i, val;
	struct vcontrol * lvc;
	
	i = find_vctrl(vc->id);
	if (i < 0)
		return -EINVAL;

	lvc = &control[i];	
	if (ov9640_read_reg_mask(client, lvc->reg, (u8 *)&val, lvc->mask))
		return -EIO;
		
	val = val >> lvc->start_bit;	
	if (val >= 0) {
		vc->value = lvc->current_value = val;
		return 0;
	} else
		return val;
}

static int
ov9640sensor_set_control(struct v4l2_control *vc, void *priv)
{
	struct ov9640_sensor *sensor = (struct ov9640_sensor *) priv;
	struct i2c_client *client = &sensor->client;
	struct vcontrol *lvc;
	int val = vc->value;
	int i;

	i = find_vctrl(vc->id);
	if (i < 0)
		return -EINVAL;

	lvc = &control[i];
	val = val << lvc->start_bit;
	if (ov9640_write_reg_mask(client, lvc->reg, (u8 *)&val, (u8)lvc->mask))
		return -EIO;

	val = val>> lvc->start_bit;
	if (val >= 0) {
		lvc->current_value = val;
		return 0;
	} else
		return val;
}

/* Implement the VIDIOC_ENUM_FMT ioctl for the CAPTURE buffer type.
 */
static int
ov9640sensor_enum_pixformat(struct v4l2_fmtdesc *fmt, void *priv)
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

		case V4L2_BUF_TYPE_VIDEO_OVERLAY:
		if (index >= NUM_OVERLAY_FORMATS)
			return -EINVAL;
		break;

		default:
			return -EINVAL;
	}

	fmt->flags = ov9640_formats[index].flags;
	strlcpy(fmt->description, ov9640_formats[index].description, sizeof(fmt->description));
	fmt->pixelformat = ov9640_formats[index].pixelformat;

	return 0;
}

/* Implement the VIDIOC_TRY_FMT ioctl for the CAPTURE buffer type.  This 
 * ioctl is used to negotiate the image capture size and pixel format 
 * without actually making it take effect.
 */
static int
ov9640sensor_try_format(struct v4l2_pix_format *pix, void *priv)
{
	enum image_size isize;
	int ifmt;

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

/* Given the image capture format in pix, the nominal frame period in 
 * timeperframe, calculate the required xclk frequency 
 * The nominal xclk input frequency of the OV9640 is 24MHz, maximum 
 * frequency is 48MHz, and minimum frequency is 10MHz.
 */
static unsigned long
ov9640sensor_calc_xclk(struct v4l2_pix_format *pix,
			struct v4l2_fract *timeperframe, void *priv)
{
	unsigned long tgt_xclk;			/* target xclk */
	unsigned long tgt_fpm;			/* target frames per minute */
 	enum image_size isize;

	/* We use arbitrary rules to select the xclk frequency.  If the 
	 * capture size is VGA and the frame rate is greater than 900 
	 * frames per minute, or if the capture size is SXGA and the 
	 * frame rate is greater than 450 frames per minutes, then the 
	 * xclk frequency will be set to 48MHz.  Otherwise, the xclk 
	 * frequency will be set to 24MHz.  If the mclk frequency is such that 
	 * the target xclk frequency is not achievable, then xclk will be set 
	 * as close as to the target as possible.
	 */
	if ((timeperframe->numerator == 0) 
		|| (timeperframe->denominator == 0))
	{
		/* supply a default nominal_timeperframe of 15 fps */
		timeperframe->numerator = 1;
		timeperframe->denominator = 15;
	}
	tgt_fpm = (timeperframe->denominator*60)
		/ timeperframe->numerator;
	tgt_xclk = 24000000;
	isize = ov9640_find_size(pix->width, pix->height);
	switch (isize) {
		case SXGA:
			if (tgt_fpm > 450)
				tgt_xclk = 48000000;
			break;
		case VGA:
			if (tgt_fpm > 900)
				tgt_xclk = 48000000;
			break;
		default:
			break;
	}
	return tgt_xclk;
}

/* Given a capture format in pix, the frame period in timeperframe, and
 * the xclk frequency, set the capture format of the OV9640 sensor.
 * The actual frame period will be returned in timeperframe.
 */
static int
ov9640sensor_configure(struct v4l2_pix_format *pix, unsigned long xclk,
			struct v4l2_fract *timeperframe, void *priv)
{
	struct ov9640_sensor *sensor = (struct ov9640_sensor *) priv;
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

	return ov9640_configure(&sensor->client,
				ov9640_find_size(pix->width, pix->height),
				pfmt, xclk, timeperframe);
}

/* Prepare for the driver to exit.
 * Balances ov9640sensor_init().
 * This function must de-initialize the sensor and its associated data 
 * structures.
 */
static int
ov9640sensor_cleanup(void *priv)
{
	struct ov9640_sensor *sensor = (struct ov9640_sensor *) priv;

	if (sensor) {
		i2c_del_driver(&ov9640sensor_i2c_driver);
		ov9640_powerdown();
 	}
	return 0;
}


static struct i2c_driver ov9640sensor_i2c_driver = {
	.driver {
		.name		= "ov9640",
	},
	.id		= I2C_DRIVERID_MISC, /*FIXME:accroding to i2c-ids.h */
	.attach_adapter	= ov9640_i2c_probe_adapter,
	.detach_client	= ov9640_i2c_detach_client,
};


/* Initialize the OV9640 sensor.
 * This routine allocates and initializes the data structure for the sensor, 
 * powers up the sensor, registers the I2C driver, and sets a default image 
 * capture format in pix.  The capture format is not actually programmed 
 * into the OV9640 sensor by this routine.
 * This function must return a non-NULL value to indicate that 
 * initialization is successful.
 */
static void *
ov9640sensor_init(struct v4l2_pix_format *pix)
{
	struct ov9640_sensor *sensor = &ov9640;
 	int err;

	memset(sensor, 0, sizeof(*sensor));
 
	/* power-up the sensor */
	if (ov9640_powerup())
		return NULL;

	err = i2c_add_driver(&ov9640sensor_i2c_driver);
	if (err) {
		printk(KERN_ERR "Failed to register OV9640 I2C client.\n");
		return NULL;
	}
	if (!sensor->client.adapter) {
		printk(KERN_WARNING 
			"Failed to detect OV9640 sensor chip.\n");
		return NULL;
	}
	else
		printk(KERN_INFO 
			"OV9640 sensor chip version 0x%02x detected\n", sensor->ver);

	/* Make the default capture format QCIF RGB565 */
	pix->width = ov9640_sizes[QCIF].width;
	pix->height = ov9640_sizes[QCIF].height;
	pix->pixelformat = V4L2_PIX_FMT_RGB565;
	ov9640sensor_try_format(pix, NULL);

	return (void *)sensor;
}

struct omap_camera_sensor camera_sensor_if = {
	.version	= 0x01,
	.name		= "OV9640",
	.parallel_mode	= PAR_MODE_NOBT8,
	.hs_polarity	= SYNC_ACTIVE_HIGH,
	.vs_polarity	= SYNC_ACTIVE_LOW,
	.image_swap 	= 0,
	.init		= ov9640sensor_init,
	.cleanup	= ov9640sensor_cleanup,
	.enum_pixformat = ov9640sensor_enum_pixformat,
	.try_format	= ov9640sensor_try_format,
	.calc_xclk	= ov9640sensor_calc_xclk,
	.configure	= ov9640sensor_configure,
	.query_control	= ov9640sensor_query_control,
	.get_control	= ov9640sensor_get_control,
	.set_control	= ov9640sensor_set_control,
	.power_on	= ov9640sensor_power_on,
	.power_off	= ov9640sensor_power_off,
};

void print_ov9640_regs(void *priv)
{
	struct ov9640_sensor *sensor = (struct ov9640_sensor *) priv;
	u8 reg, val;
	for (reg=0x00; reg <=0x8A; reg++)
		if (ov9640_read_reg(&sensor->client,reg,&val))
			printk("error reading %x\n", reg);
		else
			printk("reg %x = %x\n", reg, val);	 
}

#endif	/* ifdef CAMERA_OV9640 */
