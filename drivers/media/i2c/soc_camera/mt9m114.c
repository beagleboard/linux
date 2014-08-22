/*
 * mt9m114.c Aptina MT9M114 sensor driver
 *
 * Copyright (c) 2013 VVDN Technologies
 * Copyright (c) 2012 Analog Devices Inc.
 *
 * refer to: SoC Camera driver by Andrew Chew <achew@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/videodev2.h>

#include <media/v4l2-chip-ident.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mediabus.h>

/* Sysctl registers */
#define MT9M114_CHIP_ID					0x0000
#define MT9M114_COMMAND_REGISTER			0x0080
#define MT9M114_COMMAND_REGISTER_APPLY_PATCH		(1 << 0)
#define MT9M114_COMMAND_REGISTER_SET_STATE		(1 << 1)
#define MT9M114_COMMAND_REGISTER_REFRESH		(1 << 2)
#define MT9M114_COMMAND_REGISTER_WAIT_FOR_EVENT		(1 << 3)
#define MT9M114_COMMAND_REGISTER_OK			(1 << 15)
#define MT9M114_SOFT_RESET				0x001a
#define MT9M114_PAD_SLEW				0x001e
#define MT9M114_PAD_CONTROL				0x0032

/* XDMA registers */
#define MT9M114_ACCESS_CTL_STAT				0x0982
#define MT9M114_PHYSICAL_ADDRESS_ACCESS			0x098a
#define MT9M114_LOGICAL_ADDRESS_ACCESS			0x098e

/* Core registers */
#define MT9M114_RESET_REGISTER				0x301a
#define MT9M114_FLASH					0x3046
#define MT9M114_CUSTOMER_REV				0x31fe

/* Camera Control registers */
#define MT9M114_CAM_SENSOR_CFG_Y_ADDR_START		0xc800
#define MT9M114_CAM_SENSOR_CFG_X_ADDR_START		0xc802
#define MT9M114_CAM_SENSOR_CFG_Y_ADDR_END		0xc804
#define MT9M114_CAM_SENSOR_CFG_X_ADDR_END		0xc806
#define MT9M114_CAM_SENSOR_CFG_PIXCLK			0xc808
#define MT9M114_CAM_SENSOR_CFG_ROW_SPEED		0xc80c
#define MT9M114_CAM_SENSOR_CFG_FINE_INTEG_TIME_MIN	0xc80e
#define MT9M114_CAM_SENSOR_CFG_FINE_INTEG_TIME_MAX	0xc810
#define MT9M114_CAM_SENSOR_CFG_FRAME_LENGTH_LINES	0xc812
#define MT9M114_CAM_SENSOR_CFG_LINE_LENGTH_PCK		0xc814
#define MT9M114_CAM_SENSOR_CFG_FINE_CORRECTION		0xc816
#define MT9M114_CAM_SENSOR_CFG_CPIPE_LAST_ROW		0xc818
#define MT9M114_CAM_SENSOR_CFG_REG_0_DATA		0xc826
#define MT9M114_CAM_SENSOR_CONTROL_READ_MODE		0xc834
#define MT9M114_CAM_CROP_WINDOW_XOFFSET			0xc854
#define MT9M114_CAM_CROP_WINDOW_YOFFSET			0xc856
#define MT9M114_CAM_CROP_WINDOW_WIDTH			0xc858
#define MT9M114_CAM_CROP_WINDOW_HEIGHT			0xc85a
#define MT9M114_CAM_CROP_CROPMODE			0xc85c
#define MT9M114_CAM_OUTPUT_WIDTH			0xc868
#define MT9M114_CAM_OUTPUT_HEIGHT			0xc86a
#define MT9M114_CAM_OUTPUT_FORMAT			0xc86c
#define MT9M114_CAM_AET_AEMODE				0xc878
#define MT9M114_CAM_AET_MAX_FRAME_RATE			0xc88c
#define MT9M114_CAM_AET_MIN_FRAME_RATE			0xc88e
#define MT9M114_CAM_AWB_AWB_XSCALE			0xc8f2
#define MT9M114_CAM_AWB_AWB_YSCALE			0xc8f3
#define MT9M114_CAM_AWB_AWB_XSHIFT_PRE_ADJ		0xc904
#define MT9M114_CAM_AWB_AWB_YSHIFT_PRE_ADJ		0xc906
#define MT9M114_CAM_STAT_AWB_CLIP_WINDOW_XSTART		0xc914
#define MT9M114_CAM_STAT_AWB_CLIP_WINDOW_YSTART		0xc916
#define MT9M114_CAM_STAT_AWB_CLIP_WINDOW_XEND		0xc918
#define MT9M114_CAM_STAT_AWB_CLIP_WINDOW_YEND		0xc91a
#define MT9M114_CAM_STAT_AE_INITIAL_WINDOW_XSTART	0xc91c
#define MT9M114_CAM_STAT_AE_INITIAL_WINDOW_YSTART	0xc91e
#define MT9M114_CAM_STAT_AE_INITIAL_WINDOW_XEND		0xc920
#define MT9M114_CAM_STAT_AE_INITIAL_WINDOW_YEND		0xc922
#define MT9M114_CAM_SYSCTL_PLL_ENABLE			0xc97e
#define MT9M114_CAM_SYSCTL_PLL_DIVIDER_M_N		0xc980
#define MT9M114_CAM_SYSCTL_PLL_DIVIDER_P		0xc982
#define MT9M114_CAM_PORT_OUTPUT_CONTROL			0xc984

/* System Manager registers */
#define MT9M114_SYSMGR_NEXT_STATE			0xdc00
#define MT9M114_SYSMGR_CURRENT_STATE			0xdc01
#define MT9M114_SYSMGR_CMD_STATUS			0xdc02

/* Patch Loader registers */
#define MT9M114_PATCHLDR_LOADER_ADDRESS			0xe000
#define MT9M114_PATCHLDR_PATCH_ID			0xe002
#define MT9M114_PATCHLDR_FIRMWARE_ID			0xe004
#define MT9M114_PATCHLDR_APPLY_STATUS			0xe008
#define MT9M114_PATCHLDR_NUM_PATCHES			0xe009
#define MT9M114_PATCHLDR_PATCH_ID_0			0xe00a
#define MT9M114_PATCHLDR_PATCH_ID_1			0xe00c
#define MT9M114_PATCHLDR_PATCH_ID_2			0xe00e
#define MT9M114_PATCHLDR_PATCH_ID_3			0xe010
#define MT9M114_PATCHLDR_PATCH_ID_4			0xe012
#define MT9M114_PATCHLDR_PATCH_ID_5			0xe014
#define MT9M114_PATCHLDR_PATCH_ID_6			0xe016
#define MT9M114_PATCHLDR_PATCH_ID_7			0xe018

/* SYS_STATE values (for SYSMGR_NEXT_STATE and SYSMGR_CURRENT_STATE) */
#define MT9M114_SYS_STATE_ENTER_CONFIG_CHANGE		0x28
#define MT9M114_SYS_STATE_STREAMING			0x31
#define MT9M114_SYS_STATE_START_STREAMING		0x34
#define MT9M114_SYS_STATE_ENTER_SUSPEND			0x40
#define MT9M114_SYS_STATE_SUSPENDED			0x41
#define MT9M114_SYS_STATE_ENTER_STANDBY			0x50
#define MT9M114_SYS_STATE_STANDBY			0x52
#define MT9M114_SYS_STATE_LEAVE_STANDBY			0x54

/* Result status of last SET_STATE comamnd */
#define MT9M114_SET_STATE_RESULT_ENOERR			0x00
#define MT9M114_SET_STATE_RESULT_EINVAL			0x0c
#define MT9M114_SET_STATE_RESULT_ENOSPC			0x0d

#define MAX_FRAME_RATE 30

struct mt9m114 {
	struct v4l2_subdev sd;
	struct v4l2_ctrl_handler hdl;
	struct v4l2_fract frame_rate;
	struct v4l2_mbus_framefmt fmt;
};

struct mt9m114_reg {
	u16 reg;
	u32 val;
	int width;
};

enum {
	MT9M114_QVGA,
	MT9M114_VGA,
	MT9M114_WVGA,
	MT9M114_720P,
};

struct mt9m114_resolution {
	unsigned int width;
	unsigned int height;
};

static const struct mt9m114_resolution mt9m114_resolutions[] = {
	[MT9M114_QVGA] = {
		.width  = 320,
		.height = 240,
	},
	[MT9M114_VGA] = {
		.width  = 640,
		.height = 480,
	},
	[MT9M114_WVGA] = {
		.width  = 800,
		.height = 480,
	},
	[MT9M114_720P] = {
		.width  = 1280,
		.height = 720,
	},
};

static const struct mt9m114_reg mt9m114_init[] = {
	{ MT9M114_RESET_REGISTER,                        0x0218, 2 },

	/* PLL settings */
	{ MT9M114_LOGICAL_ADDRESS_ACCESS,                0x0000, 2 },
	{ MT9M114_CAM_SYSCTL_PLL_ENABLE,                 0x01,   1 },
	{ MT9M114_CAM_SYSCTL_PLL_DIVIDER_M_N,            0x0118, 2 },
	{ MT9M114_CAM_SYSCTL_PLL_DIVIDER_P,              0x0700, 2 },
	{ MT9M114_CAM_SENSOR_CFG_PIXCLK,                 0x2DC6C00, 4 },

	/* Sensor optimization */
	{ 0x316A, 0x8270, 2 },
	{ 0x316C, 0x8270, 2 },
	{ 0x3ED0, 0x2305, 2 },
	{ 0x3ED2, 0x77CF, 2 },
	{ 0x316E, 0x8202, 2 },
	{ 0x3180, 0x87FF, 2 },
	{ 0x30D4, 0x6080, 2 },
	{ 0xA802, 0x0008, 2 },

	{ 0x3E14, 0xFF39, 2 },

	/* APGA */
	{ 0xC95E, 0x0000, 2 },

	/* Camera control module   */
	{ 0xC892, 0x0267, 2 },
	{ 0xC894, 0xFF1A, 2 },
	{ 0xC896, 0xFFB3, 2 },
	{ 0xC898, 0xFF80, 2 },
	{ 0xC89A, 0x0166, 2 },
	{ 0xC89C, 0x0003, 2 },
	{ 0xC89E, 0xFF9A, 2 },
	{ 0xC8A0, 0xFEB4, 2 },
	{ 0xC8A2, 0x024D, 2 },
	{ 0xC8A4, 0x01BF, 2 },
	{ 0xC8A6, 0xFF01, 2 },
	{ 0xC8A8, 0xFFF3, 2 },
	{ 0xC8AA, 0xFF75, 2 },
	{ 0xC8AC, 0x0198, 2 },
	{ 0xC8AE, 0xFFFD, 2 },
	{ 0xC8B0, 0xFF9A, 2 },
	{ 0xC8B2, 0xFEE7, 2 },
	{ 0xC8B4, 0x02A8, 2 },
	{ 0xC8B6, 0x01D9, 2 },
	{ 0xC8B8, 0xFF26, 2 },
	{ 0xC8BA, 0xFFF3, 2 },
	{ 0xC8BC, 0xFFB3, 2 },
	{ 0xC8BE, 0x0132, 2 },
	{ 0xC8C0, 0xFFE8, 2 },
	{ 0xC8C2, 0xFFDA, 2 },
	{ 0xC8C4, 0xFECD, 2 },
	{ 0xC8C6, 0x02C2, 2 },
	{ 0xC8C8, 0x0075, 2 },
	{ 0xC8CA, 0x011C, 2 },
	{ 0xC8CC, 0x009A, 2 },
	{ 0xC8CE, 0x0105, 2 },
	{ 0xC8D0, 0x00A4, 2 },
	{ 0xC8D2, 0x00AC, 2 },
	{ 0xC8D4, 0x0A8C, 2 },
	{ 0xC8D6, 0x0F0A, 2 },
	{ 0xC8D8, 0x1964, 2 },

	/* Automatic White balance */
	{ MT9M114_CAM_AWB_AWB_XSHIFT_PRE_ADJ,            0x0033, 2 },
	{ MT9M114_CAM_AWB_AWB_YSHIFT_PRE_ADJ,            0x003C, 2 },
	{ MT9M114_CAM_AWB_AWB_XSCALE,                    0x03,   1 },
	{ MT9M114_CAM_AWB_AWB_YSCALE,                    0x02,   1 },
	{ 0xC8F4, 0x0000, 2 },
	{ 0xC8F6, 0x0000, 2 },
	{ 0xC8F8, 0x0000, 2 },
	{ 0xC8FA, 0xE724, 2 },
	{ 0xC8FC, 0x1583, 2 },
	{ 0xC8FE, 0x2045, 2 },
	{ 0xC900, 0x03FF, 2 },
	{ 0xC902, 0x007C, 2 },
	{ 0xC90C, 0x80,   1 },
	{ 0xC90D, 0x80,   1 },
	{ 0xC90E, 0x80,   1 },
	{ 0xC90F, 0x88,   1 },
	{ 0xC910, 0x80,   1 },
	{ 0xC911, 0x80,   1 },

	/* CPIPE Preference */
	{ 0xC926, 0x0020, 2 },
	{ 0xC928, 0x009A, 2 },
	{ 0xC946, 0x0070, 2 },
	{ 0xC948, 0x00F3, 2 },
	{ 0xC944, 0x20,   1 },
	{ 0xC945, 0x9A,   1 },
	{ 0xC92A, 0x80,   1 },
	{ 0xC92B, 0x4B,   1 },
	{ 0xC92C, 0x00,   1 },
	{ 0xC92D, 0xFF,   1 },
	{ 0xC92E, 0x3C,   1 },
	{ 0xC92F, 0x02,   1 },
	{ 0xC930, 0x06,   1 },
	{ 0xC931, 0x64,   1 },
	{ 0xC932, 0x01,   1 },
	{ 0xC933, 0x0C,   1 },
	{ 0xC934, 0x3C,   1 },
	{ 0xC935, 0x3C,   1 },
	{ 0xC936, 0x3C,   1 },
	{ 0xC937, 0x0F,   1 },
	{ 0xC938, 0x64,   1 },
	{ 0xC939, 0x64,   1 },
	{ 0xC93A, 0x64,   1 },
	{ 0xC93B, 0x32,   1 },
	{ 0xC93C, 0x0020, 2 },
	{ 0xC93E, 0x009A, 2 },
	{ 0xC940, 0x00DC, 2 },
	{ 0xC942, 0x38,   1 },
	{ 0xC943, 0x30,   1 },
	{ 0xC944, 0x50,   1 },
	{ 0xC945, 0x19,   1 },
	{ 0xC94A, 0x0230, 2 },
	{ 0xC94C, 0x0010, 2 },
	{ 0xC94E, 0x01CD, 2 },
	{ 0xC950, 0x05,   1 },
	{ 0xC951, 0x40,   1 },
	{ 0xC87B, 0x1B,   1 },
	{ MT9M114_CAM_AET_AEMODE, 0x0E, 1 },
	{ 0xC890, 0x0080, 2 },
	{ 0xC886, 0x0100, 2 },
	{ 0xC87C, 0x005A, 2 },
	{ 0xB42A, 0x05,   1 },
	{ 0xA80A, 0x20,   1 },

	{ MT9M114_LOGICAL_ADDRESS_ACCESS,                0x0000, 2 },
	{ MT9M114_CAM_PORT_OUTPUT_CONTROL,               0x8040, 2 },
	{ MT9M114_PAD_SLEW,                              0x0777, 2 },
};

static const struct mt9m114_reg mt9m114_regs_qvga[] = {
	{ MT9M114_LOGICAL_ADDRESS_ACCESS,                0x1000, 2 },
	{ MT9M114_CAM_SENSOR_CFG_Y_ADDR_START,           0x0000, 2 },
	{ MT9M114_CAM_SENSOR_CFG_X_ADDR_START,           0x0000, 2 },
	{ MT9M114_CAM_SENSOR_CFG_Y_ADDR_END,             0x03CD, 2 },
	{ MT9M114_CAM_SENSOR_CFG_X_ADDR_END,             0x050D, 2 },
	{ MT9M114_CAM_SENSOR_CFG_ROW_SPEED,              0x0001, 2 },
	{ MT9M114_CAM_SENSOR_CFG_FINE_INTEG_TIME_MIN,    0x01C3, 2 },
	{ MT9M114_CAM_SENSOR_CFG_FINE_INTEG_TIME_MAX,    0x03F7, 2 },
	{ MT9M114_CAM_SENSOR_CFG_FRAME_LENGTH_LINES,     0x0500, 2 },
	{ MT9M114_CAM_SENSOR_CFG_LINE_LENGTH_PCK,        0x04E2, 2 },
	{ MT9M114_CAM_SENSOR_CFG_FINE_CORRECTION,        0x00E0, 2 },
	{ MT9M114_CAM_SENSOR_CFG_CPIPE_LAST_ROW,         0x01E3, 2 },
	{ MT9M114_CAM_SENSOR_CFG_REG_0_DATA,             0x0020, 2 },
	{ MT9M114_CAM_CROP_WINDOW_XOFFSET,               0x0000, 2 },
	{ MT9M114_CAM_CROP_WINDOW_YOFFSET,               0x0000, 2 },
	{ MT9M114_CAM_CROP_WINDOW_WIDTH,                 0x0280, 2 },
	{ MT9M114_CAM_CROP_WINDOW_HEIGHT,                0x01E0, 2 },
	{ MT9M114_CAM_CROP_CROPMODE,                     0x03,   1 },
	{ MT9M114_CAM_OUTPUT_WIDTH,                      0x0140, 2 },
	{ MT9M114_CAM_OUTPUT_HEIGHT,                     0x00F0, 2 },
	{ MT9M114_CAM_AET_AEMODE,                        0x00,   1 },
	{ MT9M114_CAM_STAT_AWB_CLIP_WINDOW_XSTART,       0x0000, 2 },
	{ MT9M114_CAM_STAT_AWB_CLIP_WINDOW_YSTART,       0x0000, 2 },
	{ MT9M114_CAM_STAT_AWB_CLIP_WINDOW_XEND,         0x013F, 2 },
	{ MT9M114_CAM_STAT_AWB_CLIP_WINDOW_YEND,         0x00EF, 2 },
	{ MT9M114_CAM_STAT_AE_INITIAL_WINDOW_XSTART,     0x0000, 2 },
	{ MT9M114_CAM_STAT_AE_INITIAL_WINDOW_YSTART,     0x0000, 2 },
	{ MT9M114_CAM_STAT_AE_INITIAL_WINDOW_XEND,       0x003F, 2 },
	{ MT9M114_CAM_STAT_AE_INITIAL_WINDOW_YEND,       0x002F, 2 },
};

static const struct mt9m114_reg mt9m114_regs_vga[] = {
	{ MT9M114_LOGICAL_ADDRESS_ACCESS,                0x1000, 2 },
	{ MT9M114_CAM_SENSOR_CFG_Y_ADDR_START,           0x0000, 2 },
	{ MT9M114_CAM_SENSOR_CFG_X_ADDR_START,           0x0000, 2 },
	{ MT9M114_CAM_SENSOR_CFG_Y_ADDR_END,             0x03CD, 2 },
	{ MT9M114_CAM_SENSOR_CFG_X_ADDR_END,             0x050D, 2 },
	{ MT9M114_CAM_SENSOR_CFG_ROW_SPEED,              0x0001, 2 },
	{ MT9M114_CAM_SENSOR_CFG_FINE_INTEG_TIME_MIN,    0x01C3, 2 },
	{ MT9M114_CAM_SENSOR_CFG_FINE_INTEG_TIME_MAX,    0x03F7, 2 },
	{ MT9M114_CAM_SENSOR_CFG_FRAME_LENGTH_LINES,     0x0500, 2 },
	{ MT9M114_CAM_SENSOR_CFG_LINE_LENGTH_PCK,        0x04E2, 2 },
	{ MT9M114_CAM_SENSOR_CFG_FINE_CORRECTION,        0x00E0, 2 },
	{ MT9M114_CAM_SENSOR_CFG_CPIPE_LAST_ROW,         0x01E3, 2 },
	{ MT9M114_CAM_SENSOR_CFG_REG_0_DATA,             0x0020, 2 },
	{ MT9M114_CAM_CROP_WINDOW_XOFFSET,               0x0000, 2 },
	{ MT9M114_CAM_CROP_WINDOW_YOFFSET,               0x0000, 2 },
	{ MT9M114_CAM_CROP_WINDOW_WIDTH,                 0x0280, 2 },
	{ MT9M114_CAM_CROP_WINDOW_HEIGHT,                0x01E0, 2 },
	{ MT9M114_CAM_CROP_CROPMODE,                     0x03,   1 },
	{ MT9M114_CAM_OUTPUT_WIDTH,                      0x0280, 2 },
	{ MT9M114_CAM_OUTPUT_HEIGHT,                     0x01E0, 2 },
	{ MT9M114_CAM_AET_AEMODE,                        0x00,   1 },
	{ MT9M114_CAM_STAT_AWB_CLIP_WINDOW_XSTART,       0x0000, 2 },
	{ MT9M114_CAM_STAT_AWB_CLIP_WINDOW_YSTART,       0x0000, 2 },
	{ MT9M114_CAM_STAT_AWB_CLIP_WINDOW_XEND,         0x027F, 2 },
	{ MT9M114_CAM_STAT_AWB_CLIP_WINDOW_YEND,         0x01DF, 2 },
	{ MT9M114_CAM_STAT_AE_INITIAL_WINDOW_XSTART,     0x0000, 2 },
	{ MT9M114_CAM_STAT_AE_INITIAL_WINDOW_YSTART,     0x0000, 2 },
	{ MT9M114_CAM_STAT_AE_INITIAL_WINDOW_XEND,       0x007F, 2 },
	{ MT9M114_CAM_STAT_AE_INITIAL_WINDOW_YEND,       0x005F, 2 },
};

static const struct mt9m114_reg mt9m114_regs_wvga[] = {
	{ MT9M114_LOGICAL_ADDRESS_ACCESS,                0x1000, 2 },
	{ MT9M114_CAM_SENSOR_CFG_Y_ADDR_START,           0x00F4, 2 },
	{ MT9M114_CAM_SENSOR_CFG_X_ADDR_START,           0x00F4, 2 },
	{ MT9M114_CAM_SENSOR_CFG_Y_ADDR_END,             0x02DB, 2 },
	{ MT9M114_CAM_SENSOR_CFG_X_ADDR_END,             0x041B, 2 },
	{ MT9M114_CAM_SENSOR_CFG_ROW_SPEED,              0x0001, 2 },
	{ MT9M114_CAM_SENSOR_CFG_FINE_INTEG_TIME_MIN,    0x00DB, 2 },
	{ MT9M114_CAM_SENSOR_CFG_FINE_INTEG_TIME_MAX,    0x045F, 2 },
	{ MT9M114_CAM_SENSOR_CFG_FRAME_LENGTH_LINES,     0x0500, 2 },
	{ MT9M114_CAM_SENSOR_CFG_LINE_LENGTH_PCK,        0x04E2, 2 },
	{ MT9M114_CAM_SENSOR_CFG_FINE_CORRECTION,        0x0060, 2 },
	{ MT9M114_CAM_SENSOR_CFG_CPIPE_LAST_ROW,         0x01E3, 2 },
	{ MT9M114_CAM_SENSOR_CFG_REG_0_DATA,             0x0020, 2 },
	{ MT9M114_CAM_CROP_WINDOW_XOFFSET,               0x0000, 2 },
	{ MT9M114_CAM_CROP_WINDOW_YOFFSET,               0x0000, 2 },
	{ MT9M114_CAM_CROP_WINDOW_WIDTH,                 0x0320, 2 },
	{ MT9M114_CAM_CROP_WINDOW_HEIGHT,                0x01E0, 2 },
	{ MT9M114_CAM_CROP_CROPMODE,                     0x03,   1 },
	{ MT9M114_CAM_OUTPUT_WIDTH,                      0x0320, 2 },
	{ MT9M114_CAM_OUTPUT_HEIGHT,                     0x01E0, 2 },
	{ MT9M114_CAM_AET_AEMODE,                        0x00,   1 },
	{ MT9M114_CAM_STAT_AWB_CLIP_WINDOW_XSTART,       0x0000, 2 },
	{ MT9M114_CAM_STAT_AWB_CLIP_WINDOW_YSTART,       0x0000, 2 },
	{ MT9M114_CAM_STAT_AWB_CLIP_WINDOW_XEND,         0x031F, 2 },
	{ MT9M114_CAM_STAT_AWB_CLIP_WINDOW_YEND,         0x01DF, 2 },
	{ MT9M114_CAM_STAT_AE_INITIAL_WINDOW_XSTART,     0x0000, 2 },
	{ MT9M114_CAM_STAT_AE_INITIAL_WINDOW_YSTART,     0x0000, 2 },
	{ MT9M114_CAM_STAT_AE_INITIAL_WINDOW_XEND,       0x009F, 2 },
	{ MT9M114_CAM_STAT_AE_INITIAL_WINDOW_YEND,       0x005F, 2 },
};

static const struct mt9m114_reg mt9m114_regs_720p[] = {
	{ MT9M114_LOGICAL_ADDRESS_ACCESS,                0x1000, 2 },
	{ MT9M114_CAM_SENSOR_CFG_Y_ADDR_START,           0x007C, 2 },
	{ MT9M114_CAM_SENSOR_CFG_X_ADDR_START,           0x0004, 2 },
	{ MT9M114_CAM_SENSOR_CFG_Y_ADDR_END,             0x0353, 2 },
	{ MT9M114_CAM_SENSOR_CFG_X_ADDR_END,             0x050B, 2 },
	{ MT9M114_CAM_SENSOR_CFG_ROW_SPEED,              0x0001, 2 },
	{ MT9M114_CAM_SENSOR_CFG_FINE_INTEG_TIME_MIN,    0x00A0, 2 },
	{ MT9M114_CAM_SENSOR_CFG_FINE_INTEG_TIME_MAX,    0x03F7, 2 },
	{ MT9M114_CAM_SENSOR_CFG_FRAME_LENGTH_LINES,     0x03EE, 2 },
	{ MT9M114_CAM_SENSOR_CFG_LINE_LENGTH_PCK,        0x060E, 2 },
	{ MT9M114_CAM_SENSOR_CFG_FINE_CORRECTION,        0x0060, 2 },
	{ MT9M114_CAM_SENSOR_CFG_CPIPE_LAST_ROW,         0x02D3, 2 },
	{ MT9M114_CAM_SENSOR_CFG_REG_0_DATA,             0x0020, 2 },
	{ MT9M114_CAM_CROP_WINDOW_XOFFSET,               0x0000, 2 },
	{ MT9M114_CAM_CROP_WINDOW_YOFFSET,               0x0000, 2 },
	{ MT9M114_CAM_CROP_WINDOW_WIDTH,                 0x0500, 2 },
	{ MT9M114_CAM_CROP_WINDOW_HEIGHT,                0x02D0, 2 },
	{ MT9M114_CAM_CROP_CROPMODE,                     0x03,   1 },
	{ MT9M114_CAM_OUTPUT_WIDTH,                      0x0500, 2 },
	{ MT9M114_CAM_OUTPUT_HEIGHT,                     0x02D0, 2 },
	{ MT9M114_CAM_AET_AEMODE,                        0x00,   1 },
	{ MT9M114_CAM_STAT_AWB_CLIP_WINDOW_XSTART,       0x0000, 2 },
	{ MT9M114_CAM_STAT_AWB_CLIP_WINDOW_YSTART,       0x0000, 2 },
	{ MT9M114_CAM_STAT_AWB_CLIP_WINDOW_XEND,         0x04FF, 2 },
	{ MT9M114_CAM_STAT_AWB_CLIP_WINDOW_YEND,         0x02CF, 2 },
	{ MT9M114_CAM_STAT_AE_INITIAL_WINDOW_XSTART,     0x0000, 2 },
	{ MT9M114_CAM_STAT_AE_INITIAL_WINDOW_YSTART,     0x0000, 2 },
	{ MT9M114_CAM_STAT_AE_INITIAL_WINDOW_XEND,       0x009F, 2 },
	{ MT9M114_CAM_STAT_AE_INITIAL_WINDOW_YEND,       0x005F, 2 },
};

static const struct mt9m114_format {
	enum v4l2_mbus_pixelcode mbus_code;
	enum v4l2_colorspace colorspace;
} mt9m114_formats[] = {
	{
		.mbus_code      = V4L2_MBUS_FMT_UYVY8_2X8,
		.colorspace     = V4L2_COLORSPACE_JPEG,
	},
	{
		.mbus_code      = V4L2_MBUS_FMT_YUYV8_2X8,
		.colorspace     = V4L2_COLORSPACE_JPEG,
	},
	{
		.mbus_code      = V4L2_MBUS_FMT_RGB565_2X8_LE,
		.colorspace     = V4L2_COLORSPACE_SRGB,
	},
};

static inline struct mt9m114 *to_mt9m114(struct v4l2_subdev *sd)
{
	return container_of(sd, struct mt9m114, sd);
}
static inline struct v4l2_subdev *to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct mt9m114, hdl)->sd;
}

static int mt9m114_write8(struct i2c_client *client, u16 reg, u8 val)
{
	int ret;
	struct {
		u16 reg;
		u8 val;
	} __packed buf;
	struct i2c_msg msg = {
		.addr   = client->addr,
		.flags  = 0,
		.len    = 3,
		.buf    = (u8 *)&buf,
	};
	buf.reg = swab16(reg);
	buf.val = val;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		v4l_err(client, "Failed to write register 0x%04x!\n", reg);
		return ret;
	}

	return 0;
}

static int mt9m114_read16(struct i2c_client *client, u16 reg, u16 *val)
{
	int ret;
	u16 rval;
	struct i2c_msg msg[] = {
		{
			.addr   = client->addr,
			.flags  = 0,
			.len    = 2,
			.buf    = (u8 *)&reg,
		},
		{
			.addr   = client->addr,
			.flags  = I2C_M_RD,
			.len    = 2,
			.buf    = (u8 *)&rval,
		},
	};

	reg = swab16(reg);

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		v4l_err(client, "Failed to read register 0x%04x!\n", reg);
		return ret;
	}
	*val = swab16(rval);

	return 0;
}

static int mt9m114_write16(struct i2c_client *client, u16 reg, u16 val)
{
	int ret;
	struct {
		u16 reg;
		u16 val;
	} __packed buf;
	struct i2c_msg msg = {
		.addr   = client->addr,
		.flags  = 0,
		.len    = 4,
		.buf    = (u8 *)&buf,
	};
	buf.reg = swab16(reg);
	buf.val = swab16(val);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		v4l_err(client, "Failed to write register 0x%04x!\n", reg);
		return ret;
	}

	return 0;
}

static int mt9m114_write32(struct i2c_client *client, u16 reg, u32 val)
{
	int ret;
	struct {
		u16 reg;
		u32 val;
	} __packed buf;
	struct i2c_msg msg = {
		.addr   = client->addr,
		.flags  = 0,
		.len    = 6,
		.buf    = (u8 *)&buf,
	};
	buf.reg = swab16(reg);
	buf.val = swab32(val);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		v4l_err(client, "Failed to write register 0x%04x!\n", reg);
		return ret;
	}

	return 0;
}

static int mt9m114_writeregs(struct i2c_client *client,
		const struct mt9m114_reg *regs, int len)
{
	int i, ret;

	for (i = 0; i < len; i++) {
		switch (regs[i].width) {
		case 1:
			ret = mt9m114_write8(client,
					regs[i].reg, regs[i].val);
			break;
		case 2:
			ret = mt9m114_write16(client,
					regs[i].reg, regs[i].val);
			break;
		case 4:
			ret = mt9m114_write32(client,
					regs[i].reg, regs[i].val);
			break;
		default:
			ret = -EINVAL;
			break;
		}
		if (ret < 0)
			return ret;
	}
	return 0;
}

static void mt9m114_res_roundup(u32 *width, u32 *height)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mt9m114_resolutions); i++)
		if ((mt9m114_resolutions[i].width >= *width) &&
				(mt9m114_resolutions[i].height >= *height)) {
			*width = mt9m114_resolutions[i].width;
			*height = mt9m114_resolutions[i].height;
			return;
		}
	*width = mt9m114_resolutions[MT9M114_720P].width;
	*height = mt9m114_resolutions[MT9M114_720P].height;
}

static int mt9m114_set_res(struct i2c_client *client, u32 width, u32 height)
{
	u16 read_mode;

	if ((width == mt9m114_resolutions[MT9M114_QVGA].width)
		&& (height == mt9m114_resolutions[MT9M114_QVGA].height)) {
		mt9m114_writeregs(client, mt9m114_regs_qvga,
				ARRAY_SIZE(mt9m114_regs_qvga));
		mt9m114_read16(client,
			MT9M114_CAM_SENSOR_CONTROL_READ_MODE, &read_mode);
		read_mode = (read_mode & 0xfccf) | 0x0330;
		mt9m114_write16(client,
			MT9M114_CAM_SENSOR_CONTROL_READ_MODE, read_mode);
	} else if ((width == mt9m114_resolutions[MT9M114_VGA].width)
		&& (height == mt9m114_resolutions[MT9M114_VGA].height)) {
		mt9m114_writeregs(client, mt9m114_regs_vga,
				ARRAY_SIZE(mt9m114_regs_vga));
		mt9m114_read16(client,
			MT9M114_CAM_SENSOR_CONTROL_READ_MODE, &read_mode);
		read_mode = (read_mode & 0xfccf) | 0x0330;
		mt9m114_write16(client,
			MT9M114_CAM_SENSOR_CONTROL_READ_MODE, read_mode);
	} else if ((width == mt9m114_resolutions[MT9M114_WVGA].width)
		&& (height == mt9m114_resolutions[MT9M114_WVGA].height)) {
		mt9m114_writeregs(client, mt9m114_regs_wvga,
				ARRAY_SIZE(mt9m114_regs_wvga));
		mt9m114_read16(client,
			MT9M114_CAM_SENSOR_CONTROL_READ_MODE, &read_mode);
		read_mode &= 0xfccf;
		mt9m114_write16(client,
			MT9M114_CAM_SENSOR_CONTROL_READ_MODE, read_mode);
	} else if ((width == mt9m114_resolutions[MT9M114_720P].width)
		&& (height == mt9m114_resolutions[MT9M114_720P].height)) {
		mt9m114_writeregs(client, mt9m114_regs_720p,
				ARRAY_SIZE(mt9m114_regs_720p));
		mt9m114_read16(client,
			MT9M114_CAM_SENSOR_CONTROL_READ_MODE, &read_mode);
		read_mode &= 0xfccf;
		mt9m114_write16(client,
			MT9M114_CAM_SENSOR_CONTROL_READ_MODE, read_mode);
	} else {
		v4l_err(client, "Failed to select resolution!\n");
		return -EINVAL;
	}
	return 0;
}

static int mt9m114_set_state(struct i2c_client *client, u8 next_state)
{
	int timeout = 100, ret;
	u16 command;

	/* set the next desired state */
	ret = mt9m114_write8(client, MT9M114_SYSMGR_NEXT_STATE, next_state);
	if (ret < 0)
		return ret;

	/* start state transition */
	ret = mt9m114_write16(client, MT9M114_COMMAND_REGISTER,
			(MT9M114_COMMAND_REGISTER_OK
			 | MT9M114_COMMAND_REGISTER_SET_STATE));
	if (ret < 0)
		return ret;

	/* wait for the state transition to complete */
	while (timeout) {
		ret = mt9m114_read16(client,
				MT9M114_COMMAND_REGISTER, &command);
		if (ret < 0)
			return ret;
		if (!(command & MT9M114_COMMAND_REGISTER_SET_STATE))
			break;
		msleep(10);
		timeout--;
	}
	if (!timeout) {
		v4l_err(client, "Failed to poll command register\n");
		return -ETIMEDOUT;
	}

	/* check if the command is successful */
	ret = mt9m114_read16(client,
			MT9M114_COMMAND_REGISTER, &command);
	if (ret < 0)
		return ret;
	if (command & MT9M114_COMMAND_REGISTER_OK)
		return 0;
	else
		return -EFAULT;
}

static int mt9m114_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	switch (ctrl->id) {
	case V4L2_CID_HFLIP:
	{
		u16 read_mode;
		mt9m114_read16(client,
			MT9M114_CAM_SENSOR_CONTROL_READ_MODE, &read_mode);
		read_mode = (read_mode & 0xfffe) | ctrl->val;
		mt9m114_write16(client,
			MT9M114_CAM_SENSOR_CONTROL_READ_MODE, read_mode);
		break;
	}
	case V4L2_CID_VFLIP:
	{
		u16 read_mode;
		mt9m114_read16(client,
			MT9M114_CAM_SENSOR_CONTROL_READ_MODE, &read_mode);
		read_mode = (read_mode & 0xfffd) | (ctrl->val << 1);
		mt9m114_write16(client,
			MT9M114_CAM_SENSOR_CONTROL_READ_MODE, read_mode);
		break;
	}
	default:
		return -EINVAL;
	}

	return 0;
}

static int mt9m114_enum_mbus_fmt(struct v4l2_subdev *sd, unsigned index,
				enum v4l2_mbus_pixelcode *code)
{
	if (index >= ARRAY_SIZE(mt9m114_formats))
		return -EINVAL;

	*code = mt9m114_formats[index].mbus_code;
	return 0;
}

static int mt9m114_try_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	int index;

	for (index = 0; index < ARRAY_SIZE(mt9m114_formats); index++)
		if (mt9m114_formats[index].mbus_code == fmt->code)
			break;
	if (index >= ARRAY_SIZE(mt9m114_formats)) {
		/* default to first format */
		index = 0;
		fmt->code = mt9m114_formats[0].mbus_code;
	}
	mt9m114_res_roundup(&fmt->width, &fmt->height);

	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = mt9m114_formats[index].colorspace;
	return 0;
}

static int mt9m114_s_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mt9m114 *sensor = to_mt9m114(sd);
	u16 output_fmt;
	int ret;

	mt9m114_try_mbus_fmt(sd, fmt);

	/* set image size */
	ret = mt9m114_set_res(client, fmt->width, fmt->height);
	if (ret < 0)
		return ret;

	/* set image format */
	ret = mt9m114_read16(client, MT9M114_CAM_OUTPUT_FORMAT, &output_fmt);
	if (ret < 0)
		return ret;
	output_fmt &= 0xc0fc;
	switch (fmt->code) {
	case V4L2_MBUS_FMT_UYVY8_2X8:
		output_fmt |= 0x0002;
		break;
	case V4L2_MBUS_FMT_YUYV8_2X8:
		break;
	case V4L2_MBUS_FMT_RGB565_2X8_LE:
		output_fmt |= 0x0102;
		break;
	case V4L2_MBUS_FMT_RGB565_2X8_BE:
		output_fmt |= 0x0100;
		break;
	default:
		return -EINVAL;
	}
	ret = mt9m114_write16(client, MT9M114_CAM_OUTPUT_FORMAT, output_fmt);
	if (ret < 0)
		return ret;

	sensor->fmt = *fmt;

	return 0;
}

static int mt9m114_g_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	struct mt9m114 *sensor = to_mt9m114(sd);

	*fmt = sensor->fmt;
	return 0;
}

static int mt9m114_g_parm(struct v4l2_subdev *sd,
				struct v4l2_streamparm *parms)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct v4l2_captureparm *cp = &parms->parm.capture;
	u16 frame_rate;

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(cp, 0, sizeof(*cp));
	cp->capability = V4L2_CAP_TIMEPERFRAME;
	cp->timeperframe.numerator = 1;
	mt9m114_read16(client, MT9M114_CAM_AET_MAX_FRAME_RATE, &frame_rate);
	cp->timeperframe.denominator = frame_rate >> 8;
	return 0;
}

static int mt9m114_s_parm(struct v4l2_subdev *sd,
				struct v4l2_streamparm *parms)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct v4l2_captureparm *cp = &parms->parm.capture;
	struct v4l2_fract *tpf = &cp->timeperframe;
	u16 frame_rate;

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	if (cp->extendedmode != 0)
		return -EINVAL;

	if (tpf->numerator == 0 || tpf->denominator == 0
		|| (tpf->denominator > tpf->numerator * MAX_FRAME_RATE)) {
		/* reset to max frame rate */
		tpf->numerator = 1;
		tpf->denominator = MAX_FRAME_RATE;
	}
	frame_rate = (tpf->denominator / tpf->numerator) << 8;
	mt9m114_write16(client, MT9M114_CAM_AET_MAX_FRAME_RATE, frame_rate);
	mt9m114_write16(client, MT9M114_CAM_AET_MIN_FRAME_RATE, frame_rate);
	return 0;
}

static int mt9m114_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	ret = mt9m114_set_state(client,
			MT9M114_SYS_STATE_ENTER_CONFIG_CHANGE);
	if (ret < 0)
		return ret;
	if (enable)
		ret = mt9m114_set_state(client,
				MT9M114_SYS_STATE_START_STREAMING);
	else
		ret = mt9m114_set_state(client,
				MT9M114_SYS_STATE_ENTER_SUSPEND);
	return ret;
}

static int mt9m114_g_chip_ident(struct v4l2_subdev *sd,
		struct v4l2_dbg_chip_ident *chip)
{
	u16 rev;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	mt9m114_read16(client, MT9M114_CUSTOMER_REV, &rev);

	return v4l2_chip_ident_i2c_client(client, chip,
			V4L2_IDENT_MT9M114, rev);
}

static const struct v4l2_ctrl_ops mt9m114_ctrl_ops = {
	.s_ctrl = mt9m114_s_ctrl,
};

static const struct v4l2_subdev_core_ops mt9m114_core_ops = {
	.g_chip_ident = mt9m114_g_chip_ident,
};

static const struct v4l2_subdev_video_ops mt9m114_video_ops = {
	.enum_mbus_fmt = mt9m114_enum_mbus_fmt,
	.try_mbus_fmt = mt9m114_try_mbus_fmt,
	.s_mbus_fmt = mt9m114_s_mbus_fmt,
	.g_mbus_fmt = mt9m114_g_mbus_fmt,
	.s_parm = mt9m114_s_parm,
	.g_parm = mt9m114_g_parm,
	.s_stream = mt9m114_s_stream,
};

static const struct v4l2_subdev_ops mt9m114_ops = {
	.core = &mt9m114_core_ops,
	.video = &mt9m114_video_ops,
};

static int mt9m114_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct mt9m114 *sensor;
	struct v4l2_subdev *sd;
	struct v4l2_ctrl_handler *hdl;
	u16 chip_id, command, output_control;
	struct v4l2_mbus_framefmt default_fmt;
	int ret;

	/* check if the adapter supports the needed features */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -EIO;

	ret = mt9m114_read16(client, MT9M114_CHIP_ID, &chip_id);
	if (ret < 0) {
		v4l_err(client, "Failed to get chip id\n");
		return -ENODEV;
	}
	if (chip_id != 0x2481) {
		v4l_err(client, "chip id 0x%04x mismatch\n", chip_id);
		return -ENODEV;
	}

	/* reset the sensor */
	ret = mt9m114_write16(client, MT9M114_SOFT_RESET, 0x0001);
	if (ret < 0) {
		v4l_err(client, "Failed to reset the sensor\n");
		return ret;
	}
	mt9m114_write16(client, MT9M114_SOFT_RESET, 0x0000);
	mdelay(50);

	do {
		ret = mt9m114_read16(client,
				MT9M114_COMMAND_REGISTER, &command);
		if (ret < 0)
			return ret;
	} while (command & MT9M114_COMMAND_REGISTER_SET_STATE);
	ret = mt9m114_writeregs(client, mt9m114_init,
			ARRAY_SIZE(mt9m114_init));
	if (ret < 0) {
		v4l_err(client, "Failed to initialize the sensor\n");
		return ret;
	}

	/* set the sensor in parallel data output mode */
	mt9m114_read16(client, MT9M114_CAM_PORT_OUTPUT_CONTROL,
			&output_control);
	output_control &= 0xfff8;
	mt9m114_write16(client, MT9M114_CAM_PORT_OUTPUT_CONTROL,
			output_control);
	mt9m114_set_state(client, MT9M114_SYS_STATE_ENTER_CONFIG_CHANGE);

	sensor = kzalloc(sizeof(*sensor), GFP_KERNEL);
	if (sensor == NULL)
		return -ENOMEM;

	sd = &sensor->sd;
	v4l2_i2c_subdev_init(sd, client, &mt9m114_ops);

	default_fmt.width = mt9m114_resolutions[MT9M114_VGA].width;
	default_fmt.height = mt9m114_resolutions[MT9M114_VGA].height;
	default_fmt.code = V4L2_MBUS_FMT_YUYV8_2X8;
	ret = mt9m114_s_mbus_fmt(sd, &default_fmt);
	if (ret < 0) {
		v4l_err(client, "Failed to set default format\n");
		kfree(sensor);
		return -EFAULT;
	}

	v4l_info(client, "chip found @ 0x%02x (%s)\n",
			client->addr << 1, client->adapter->name);

	hdl = &sensor->hdl;
	v4l2_ctrl_handler_init(hdl, 2);
	v4l2_ctrl_new_std(hdl, &mt9m114_ctrl_ops,
			V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(hdl, &mt9m114_ctrl_ops,
			V4L2_CID_VFLIP, 0, 1, 1, 0);
	/* hook the control handler into the driver */
	sd->ctrl_handler = hdl;
	if (hdl->error) {
		int err = hdl->error;

		v4l2_ctrl_handler_free(hdl);
		kfree(sensor);
		return err;
	}

	/* initialize the hardware to the default control values */
	ret = v4l2_ctrl_handler_setup(hdl);
	if (ret) {
		v4l2_ctrl_handler_free(hdl);
		kfree(sensor);
	}

	return ret;
}

static int mt9m114_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mt9m114 *sensor = to_mt9m114(sd);

	v4l2_device_unregister_subdev(sd);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	kfree(sensor);
	return 0;
}

static const struct i2c_device_id mt9m114_id[] = {
	{"mt9m114", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, mt9m114_id);

static struct i2c_driver mt9m114_driver = {
	.driver = {
		.owner  = THIS_MODULE,
		.name   = "mt9m114",
	},
	.probe          = mt9m114_probe,
	.remove         = mt9m114_remove,
	.id_table       = mt9m114_id,
};

module_i2c_driver(mt9m114_driver);

MODULE_DESCRIPTION("Aptina MT9M114 sensor driver");
MODULE_AUTHOR("Scott Jiang <Scott.Jiang.Linux@gmail.com>");
MODULE_LICENSE("GPL v2");
