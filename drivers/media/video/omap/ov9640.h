/*
 * drivers/media/video/omap/ov9640.h
 *
 * Register definitions for the OmniVision OV9640 CameraChip.
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

#ifndef OV9640_H
#define OV9640_H

/* The OV9640 I2C sensor chip has a fixed slave address of 0x30. */
#ifdef CONFIG_OMAP24XX_VIRTIO
#define OV9640_I2C_ADDR		0x60
#else
#define OV9640_I2C_ADDR		0x30
#endif

/* define register offsets for the OV9640 sensor chip */
#define OV9640_GAIN		0x00
#define OV9640_BLUE		0x01
#define OV9640_RED		0x02
#define OV9640_VREF		0x03
#define OV9640_COM1		0x04
#define OV9640_BAVE		0x05
#define OV9640_GEAVE		0x06
#define OV9640_RAVE		0x08
#define OV9640_COM2		0x09
#define OV9640_PID		0x0A
#define OV9640_VER		0x0B
#define OV9640_COM3		0x0C
#define OV9640_COM4		0x0D
#define OV9640_COM5		0x0E
#define OV9640_COM6		0x0F
#define OV9640_AECH		0x10
#define OV9640_CLKRC		0x11
#define OV9640_COM7		0x12
#define OV9640_COM8		0x13
#define OV9640_COM9		0x14
#define OV9640_COM10		0x15
#define OV9640_HSTRT		0x17
#define OV9640_HSTOP		0x18
#define OV9640_VSTRT		0x19
#define OV9640_VSTOP		0x1A
#define OV9640_PSHFT		0x1B
#define OV9640_MIDH		0x1C
#define OV9640_MIDL		0x1D
#define OV9640_MVFP		0x1E
#define OV9640_LAEC		0x1F
#define OV9640_BOS		0x20
#define OV9640_GBOS		0x21
#define OV9640_GROS		0x22
#define OV9640_ROS		0x23
#define OV9640_AEW		0x24
#define OV9640_AEB		0x25
#define OV9640_VPT		0x26
#define OV9640_BBIAS		0x27
#define OV9640_GBBIAS		0x28
#define OV9640_EXHCH		0x2A
#define OV9640_EXHCL		0x2B
#define OV9640_RBIAS		0x2C
#define OV9640_ADVFL		0x2D
#define OV9640_ADVFH		0x2E
#define OV9640_YAVE		0x2F
#define OV9640_HSYST		0x30
#define OV9640_HSYEN		0x31
#define OV9640_HREF		0x32
#define OV9640_CHLF		0x33
#define OV9640_ARBLM		0x34
#define OV9640_ADC		0x37
#define OV9640_ACOM		0x38
#define OV9640_OFON		0x39
#define OV9640_TSLB		0x3A
#define OV9640_COM11		0x3B
#define OV9640_COM12		0x3C
#define OV9640_COM13		0x3D
#define OV9640_COM14		0x3E
#define OV9640_EDGE		0x3F
#define OV9640_COM15		0x40
#define OV9640_COM16		0x41
#define OV9640_COM17		0x42
#define OV9640_MTX1		0x4F
#define OV9640_MTX2		0x50
#define OV9640_MTX3		0x51
#define OV9640_MTX4		0x52
#define OV9640_MTX5		0x53
#define OV9640_MTX6		0x54
#define OV9640_MTX7		0x55
#define OV9640_MTX8		0x56
#define OV9640_MTX9		0x57
#define OV9640_MTXS		0x58
#define OV9640_LCC1		0x62
#define OV9640_LCC2		0x63
#define OV9640_LCC3		0x64
#define OV9640_LCC4		0x65
#define OV9640_LCC5		0x66
#define OV9640_MANU		0x67
#define OV9640_MANV		0x68
#define OV9640_HV		0x69
#define OV9640_MBD		0x6A
#define OV9640_DBLV		0x6B
#define OV9640_GSP1		0x6C
#define OV9640_GSP2		0x6D
#define OV9640_GSP3		0x6E
#define OV9640_GSP4		0x6F
#define OV9640_GSP5		0x70
#define OV9640_GSP6		0x71
#define OV9640_GSP7		0x72
#define OV9640_GSP8		0x73
#define OV9640_GSP9		0x74
#define OV9640_GSP10		0x75
#define OV9640_GSP11		0x76
#define OV9640_GSP12		0x77
#define OV9640_GSP13		0x78
#define OV9640_GSP14		0x79
#define OV9640_GSP15		0x7A
#define OV9640_GSP16		0x7B
#define OV9640_GST1		0x7C
#define OV9640_GST2		0x7D
#define OV9640_GST3		0x7E
#define OV9640_GST4		0x7F
#define OV9640_GST5		0x80
#define OV9640_GST6		0x81
#define OV9640_GST7		0x82
#define OV9640_GST8		0x83
#define OV9640_GST9		0x84
#define OV9640_GST10		0x85
#define OV9640_GST11		0x86
#define OV9640_GST12		0x87
#define OV9640_GST13		0x88
#define OV9640_GST14		0x89
#define OV9640_GST15		0x8A

#define OV9640_NUM_REGS		(OV9640_GST15 + 1)

#define OV9640_PID_MAGIC	0x96	/* high byte of product ID number */
#define OV9640_VER_REV2		0x48	/* low byte of product ID number */
#define OV9640_VER_REV3		0x49	/* low byte of product ID number */
#define OV9640_MIDH_MAGIC	0x7F	/* high byte of mfg ID */
#define OV9640_MIDL_MAGIC	0xA2	/* low byte of mfg ID */

/* define a structure for ov9640 register initialization values */
struct ov9640_reg {
	unsigned char reg;
	unsigned char val;
};

enum image_size { QQCIF, QQVGA, QCIF, QVGA, CIF, VGA, SXGA };
enum pixel_format { YUV, RGB565, RGB555 };
#define NUM_IMAGE_SIZES 7
#define NUM_PIXEL_FORMATS 3

struct capture_size {
	unsigned long width;
	unsigned long height;
};

/* Array of image sizes supported by OV9640.  These must be ordered from 
 * smallest image size to largest.
 */
const static struct capture_size ov9640_sizes[] = {
	{   88,  72 },	/* QQCIF */
	{  160, 120 },	/* QQVGA */
	{  176, 144 },	/* QCIF */
	{  320, 240 },	/* QVGA */
	{  352, 288 },	/* CIF */
	{  640, 480 },	/* VGA */
	{ 1280, 960 },	/* SXGA */
};

#endif /* ifndef OV9640_H */

