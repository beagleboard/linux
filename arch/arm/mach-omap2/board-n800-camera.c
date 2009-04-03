/*
 * arch/arm/mach-omap2/board-n800-camera.c
 *
 * Copyright (C) 2007 Nokia Corporation
 *
 * Contact: Sakari Ailus <sakari.ailus@nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/gpio.h>
#include <linux/i2c/menelaus.h>

#include <media/v4l2-int-device.h>

#include <asm/mach-types.h>

#include <mach/board.h>

#include <../drivers/cbus/retu.h>
#include <../drivers/media/video/tcm825x.h>

#include "board-n800.h"

#if defined (CONFIG_VIDEO_TCM825X) || defined (CONFIG_VIDEO_TCM825X_MODULE)

#define OMAP24XX_CAMERA_JAM_HACK

#ifdef OMAP24XX_CAMERA_JAM_HACK
/*
 * We don't need to check every pixel to assume that the frame is
 * corrupt and the sensor is jammed. CHECK_X and CHECK_Y are the
 * number of u32s to check per line / row, plus there are two lines in
 * the bottom of the frame.
 */
#define CHECK_X 8
#define CHECK_Y 6
/*
 * Start checking after this many frames since resetting the sensor.
 * Sometimes the first frame(s) is(/are) black which could trigger
 * unwanted reset(s).
 */
#define JAM_CHECK_AFTER 3
/*
 * If the sensor is quickly brought into bright conditions from dark,
 * it may temporarily be saturated, leaving out the normal background
 * noise. This many saturated frames may go through before the sensor
 * is considered jammed.
 */
#define SATURATED_MAX 30
#endif

#define N800_CAM_SENSOR_RESET_GPIO	53

static int sensor_okay;
#ifdef OMAP24XX_CAMERA_JAM_HACK
static int frames_after_reset;
static int saturated_count;
#endif

const static struct tcm825x_reg tcm825x_regs_n800[] = {
	/* initial settings for 2.5 V */
	{0x00, 0x03}, {0x03, 0x29}, {0xaa, 0x2a}, {0xc0, 0x2b},
	{0x10, 0x2c}, {0x4c, 0x2d}, {0x9c, 0x3f},

	/* main settings */
	{0x00, 0x00}, {0x30, 0x01}, {0x0e, 0x02}, /* initial */
	{0x0f, 0x04}, {0x02, 0x05}, {0x0d, 0x06}, {0xc0, 0x07},
	{0x38, 0x08}, {0x50, 0x09}, {0x80, 0x0a}, {0x40, 0x0b},
	{0x40, 0x0c}, {0x00, 0x0d}, {0x04, 0x0e}, {0x04, 0x0f},
	{0x22, 0x10}, {0x96, 0x11}, {0xf0, 0x12}, {0x08, 0x13},
	{0x08, 0x14}, {0x30, 0x15}, {0x30, 0x16}, {0x01, 0x17},
	{0x40, 0x18}, {0x87, 0x19}, {0x2b, 0x1a}, {0x84, 0x1b},
	{0x52, 0x1c}, {0x44, 0x1d}, {0x68, 0x1e}, {0x00, 0x1f},
	{0x00, 0x20}, {0x01, 0x21}, {0x27, 0x22}, {0x40, 0x23},
	{0x27, 0x24}, {0x5f, 0x25}, {0x00, 0x26}, {0x16, 0x27},
	{0x23, 0x28}, /* initial */ /* initial */ /* initial */
	/* initial */ /* initial */ {0x00, 0x2e}, {0x00, 0x2f},
	{0x00, 0x30}, {0x00, 0x31}, {0x00, 0x32}, {0x00, 0x33},
	{0x00, 0x34}, {0x00, 0x35}, {0x00, 0x36}, {0x00, 0x37},
	{0x00, 0x38}, {0x8c, 0x39}, {0xc8, 0x3A}, {0x80, 0x3b},
	{0x00, 0x3c}, {0x17, 0x3d}, {0x85, 0x3e}, /* initial */
	{0xa0, 0x40}, {0x00, 0x41}, {0x00, 0x42}, {0x00, 0x43},
	{0x08, 0x44}, {0x12, 0x45}, {0x00, 0x46}, {0x20, 0x47},
	{0x30, 0x48}, {0x18, 0x49}, {0x20, 0x4a}, {0x4d, 0x4b},
	{0x0c, 0x4c}, {0xe0, 0x4d}, {0x20, 0x4e}, {0x89, 0x4f},
	{0x21, 0x50}, {0x80, 0x51}, {0x02, 0x52}, {0x00, 0x53},
	{0x30, 0x54}, {0x90, 0x55}, {0x40, 0x56}, {0x06, 0x57},
	{0x0f, 0x58}, {0x23, 0x59}, {0x08, 0x5A}, {0x04, 0x5b},
	{0x08, 0x5c}, {0x08, 0x5d}, {0x08, 0x5e}, {0x08, 0x5f},
	{TCM825X_VAL_TERM, TCM825X_REG_TERM}
};

const static struct tcm825x_reg tcm825x_regs_n810[] = {
	/* initial settings for 2.5 V */
	{0x00, 0x03}, {0x03, 0x29}, {0xaa, 0x2a}, {0xc0, 0x2b},
	{0x10, 0x2c}, {0x4c, 0x2d}, {0x9c, 0x3f},

	/* main settings */
	{0x00, 0x00}, {0x30, 0x01}, {0x0e, 0x02}, /* initial */
	{0xcf, 0x04}, {0x02, 0x05}, {0x0d, 0x06}, {0xc0, 0x07},
	{0x38, 0x08}, {0x50, 0x09}, {0x80, 0x0a}, {0x40, 0x0b},
	{0x40, 0x0c}, {0x00, 0x0d}, {0x04, 0x0e}, {0x04, 0x0f},
	{0x22, 0x10}, {0x96, 0x11}, {0xf0, 0x12}, {0x08, 0x13},
	{0x08, 0x14}, {0x30, 0x15}, {0x30, 0x16}, {0x01, 0x17},
	{0x40, 0x18}, {0x87, 0x19}, {0x2b, 0x1a}, {0x84, 0x1b},
	{0x52, 0x1c}, {0x44, 0x1d}, {0x68, 0x1e}, {0x00, 0x1f},
	{0x00, 0x20}, {0x01, 0x21}, {0x27, 0x22}, {0x40, 0x23},
	{0x27, 0x24}, {0x5f, 0x25}, {0x00, 0x26}, {0x16, 0x27},
	{0x23, 0x28}, /* initial */ /* initial */ /* initial */
	/* initial */ /* initial */ {0x00, 0x2e}, {0x00, 0x2f},
	{0x00, 0x30}, {0x00, 0x31}, {0x00, 0x32}, {0x00, 0x33},
	{0x00, 0x34}, {0x00, 0x35}, {0x00, 0x36}, {0x00, 0x37},
	{0x00, 0x38}, {0x8c, 0x39}, {0xc8, 0x3A}, {0x80, 0x3b},
	{0x00, 0x3c}, {0x17, 0x3d}, {0x85, 0x3e}, /* initial */
	{0xa0, 0x40}, {0x00, 0x41}, {0x00, 0x42}, {0x00, 0x43},
	{0x08, 0x44}, {0x12, 0x45}, {0x00, 0x46}, {0x20, 0x47},
	{0x30, 0x48}, {0x18, 0x49}, {0x20, 0x4a}, {0x4d, 0x4b},
	{0x0c, 0x4c}, {0xe0, 0x4d}, {0x20, 0x4e}, {0x89, 0x4f},
	{0x21, 0x50}, {0x80, 0x51}, {0x02, 0x52}, {0x00, 0x53},
	{0x30, 0x54}, {0x90, 0x55}, {0x40, 0x56}, {0x06, 0x57},
	{0x0f, 0x58}, {0x23, 0x59}, {0x08, 0x5A}, {0x04, 0x5b},
	{0x08, 0x5c}, {0x08, 0x5d}, {0x08, 0x5e}, {0x08, 0x5f},
	{TCM825X_VAL_TERM, TCM825X_REG_TERM}
};

static int tcm825x_is_okay(void)
{
	return sensor_okay;
}

/*
 * VSIM1	--> CAM_IOVDD	--> IOVDD (1.8 V)
 */
static int tcm825x_power_on(void)
{
	int ret;

	/* Set VMEM to 1.5V and VIO to 2.5V */
	ret = menelaus_set_vmem(1500);
	if (ret < 0) {
		/* Try once more, it seems the sensor power up causes
		 * some problems on the I2C bus. */
		ret = menelaus_set_vmem(1500);
		if (ret < 0)
			return ret;
	}
	msleep(1);

	ret = menelaus_set_vio(2500);
	if (ret < 0)
		return ret;

	/* Set VSim1 on */
	retu_write_reg(RETU_REG_CTRL_SET, 0x0080);
	msleep(1);

	gpio_set_value(N800_CAM_SENSOR_RESET_GPIO, 1);
	msleep(1);

	saturated_count = 0;
	frames_after_reset = 0;

	return 0;
}

static int tcm825x_power_off(void)
{
	int ret;

	gpio_set_value(N800_CAM_SENSOR_RESET_GPIO, 0);
	msleep(1);

	/* Set VSim1 off */
	retu_write_reg(RETU_REG_CTRL_CLR, 0x0080);
	msleep(1);

	/* Set VIO_MODE to off */
	ret = menelaus_set_vio(0);
	if (ret < 0)
		return ret;
	msleep(1);

	/* Set VMEM_MODE to off */
	ret = menelaus_set_vmem(0);
	if (ret < 0)
		return ret;
	msleep(1);

	return 0;
}

static int tcm825x_power_set(int power)
{
	BUG_ON(!sensor_okay);

	if (power)
		return tcm825x_power_on();
	else
		return tcm825x_power_off();
}

static const struct tcm825x_reg *tcm825x_default_regs(void)
{
	if (machine_is_nokia_n810())
		return tcm825x_regs_n810;

	return tcm825x_regs_n800;
}

#ifdef OMAP24XX_CAMERA_JAM_HACK
/*
 * Check for jammed sensor, in which case all horizontal lines are
 * equal. Handle also case where sensor could be saturated awhile in
 * case of rapid increase of brightness.
 */
static int tcm825x_needs_reset(struct v4l2_int_device *s, void *buf,
			       struct v4l2_pix_format *pix)
{
	int i, j;
	uint32_t xor, xor2;
	uint32_t offset;
	uint32_t dx_offset;
	uint32_t saturated_pattern;
	int is_saturated = 1;

	switch (pix->pixelformat) {
	default:
	case V4L2_PIX_FMT_RGB565:
		saturated_pattern = 0xffffffff; /* guess */
		break;
	case V4L2_PIX_FMT_UYVY:
		saturated_pattern = 0xe080e080;
		break;
	}

	/* This won't work for height under 2 at all. */
	if (pix->height < 2)
		return 0;
	/* Check that there is enough image data. */
	if (pix->width * TCM825X_BYTES_PER_PIXEL < sizeof(uint32_t))
		return 0;
	/*
	 * Don't check for jamming immediately. Sometimes frames
	 * immediately after reset are black.
	 */
	if (frames_after_reset < JAM_CHECK_AFTER) {
		frames_after_reset++;
		return 0;
	}

	dx_offset = ((pix->width - sizeof(uint32_t) / TCM825X_BYTES_PER_PIXEL)
		     * TCM825X_BYTES_PER_PIXEL) / (CHECK_X - 1);
	dx_offset = dx_offset - dx_offset % TCM825X_BYTES_PER_PIXEL;
	/*
	 * Check two lines in the bottom first. They're unlikely to be
	 * saturated and quick to check.
	 */
	offset = (pix->height - 2) * pix->bytesperline;
	xor = xor2 = 0;
	for (j = 0; j < CHECK_X; j++) {
		uint32_t *val = buf + offset;
		uint32_t *val2 = buf + offset + pix->bytesperline;
		xor ^= *val;
		if (*val != saturated_pattern)
			is_saturated = 0;
		xor2 ^= *val2;
		if (xor2 != xor) {
			saturated_count = 0;
			return 0;
		}
		offset += dx_offset;
	}
	/* Check the rest of the picture. */
	offset = 0;
	for (i = 0; i < CHECK_Y; i++) {
		uint32_t offset2 = offset;
		xor2 = 0;
		for (j = 0; j < CHECK_X; j++) {
			uint32_t *val = buf + offset2;
			xor2 ^= *val;
			offset2 += dx_offset;
		}
		if (xor2 != xor) {
			saturated_count = 0;
			return 0;
		}
		offset += pix->bytesperline * ((pix->height - 2) / CHECK_Y);
	}

	if (is_saturated && saturated_count++ < SATURATED_MAX)
		return 0;

	return -EIO;
}
#else
static int tcm825x_needs_reset(struct v4l2_int_device *s, void *buf,
			       struct v4l2_pix_format *pix)
{
	return 0;
}
#endif

static const struct v4l2_ifparm ifparm = {
	.if_type = V4L2_IF_TYPE_BT656,
	.u = {
		.bt656 = {
			 .frame_start_on_rising_vs = 1,
			 .latch_clk_inv = 1,
			 .mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT,
			 .clock_min = TCM825X_XCLK_MIN,
			 .clock_max = TCM825X_XCLK_MAX,
		 },
	},
};

static int tcm825x_ifparm(struct v4l2_ifparm *p)
{
	*p = ifparm;

	return 0;
}

static int tcm825x_is_upside_down(void)
{
	return machine_is_nokia_n810();
}

const struct tcm825x_platform_data n800_tcm825x_platform_data = {
	.is_okay	= tcm825x_is_okay,
	.power_set	= tcm825x_power_set,
	.default_regs	= tcm825x_default_regs,
	.needs_reset	= tcm825x_needs_reset,
	.ifparm		= tcm825x_ifparm,
	.is_upside_down	= tcm825x_is_upside_down,
};

void __init n800_cam_init(void)
{
	int r;

	r = gpio_request(N800_CAM_SENSOR_RESET_GPIO, "TCM825x reset");
	if (r < 0) {
		printk(KERN_WARNING "%s: failed to request gpio\n",
			__func__);
		return;
	}

	gpio_direction_output(N800_CAM_SENSOR_RESET_GPIO, 0);

	sensor_okay = 1;
}

#else
void __init n800_cam_init(void)
{
}

#endif
