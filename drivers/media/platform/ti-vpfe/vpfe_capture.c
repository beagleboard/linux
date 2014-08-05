/*
 * TI VPFE Multi instance Capture Driver
 *
 * Copyright (C) 2013 - 2014 Texas Instruments, Inc.
 *
 * Benoit Parrot <bparrot@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Detail description:
 *    VPFE Capture driver allows applications to capture and stream video
 *    frames on TI AM43xx SoCs from a YUV or Raw Bayer source such as
 *    decoder or image sensor device.
 *
 *    This SoC has a dual Video Processing Front End (VPFE) instances for
 *    capturing video/raw image data into memory buffers. The data can then
 *    be stored or passed on to a compatible display device.
 *    A typical EVM using these SoCs have following high level configuration.
 *
 *
 *    decoder(OV2659/		YUV/
 *	     TVP514x)   -->  Raw Bayer RGB ---> VPFE (CCDC/ISIF)
 *				data input        |      |
 *						  V      |
 *						SDRAM    |
 *						         V
 *						    Image Processor
 *						         |
 *						         V
 *						       SDRAM
 *
 *    The data flow happens from a sensor/decoder connected to the VPFE over a
 *    YUV embedded (BT.656/BT.1120) or separate sync or raw bayer rgb interface
 *    and to the input of VPFE. The input data is first passed through
 *    ISIF (Image Sensor Interface). The ISIF does very little or no
 *    processing on YUV data and does lightly pre-process Raw Bayer RGB
 *    data through data gain/offset etc. After this, data is written to SDRAM.
 *
 *    Features supported
 *		- MMAP IO
 *		- Capture using OV2659 in raw mode
 *		- Support for interfacing decoders/sensor using sub-device model
 *		- Support for asynchronous sub-device registration
 *		- Support media controller API
 *		- Support videobuf2 buffer API
 *		- Work with AM437x ISIF to do Raw Bayer RGB/YUV data capture
 *                to SDRAM.
 *    TODO list
 *		- Support for control ioctls
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <media/v4l2-common.h>
#include <media/v4l2-of.h>
#include <linux/io.h>
#include "vpfe_capture.h"
#include <media/tvp514x.h>

#include "am437x_isif_regs.h"

static int debug;
static u32 numbuffers = 3;
static u32 bufsize = (800 * 600 * 2);

module_param(numbuffers, uint, S_IRUGO);
module_param(bufsize, uint, S_IRUGO);
module_param(debug, int, 0644);

MODULE_PARM_DESC(numbuffers, "buffer count (default:3)");
MODULE_PARM_DESC(bufsize, "buffer size in bytes (default:720 x 576 x 2)");
MODULE_PARM_DESC(debug, "Debug level 0-1");

MODULE_DESCRIPTION("VPFE Video for Linux Capture Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Texas Instruments");

#define MAX_WIDTH	4096
#define MAX_HEIGHT	4096

/* standard information */
struct vpfe_standard {
	v4l2_std_id std_id;
	unsigned int width;
	unsigned int height;
	struct v4l2_fract pixelaspect;
	/* 0 - progressive, 1 - interlaced */
	int frame_format;
};

/* MULTI: Global default OK for multi-instance use */
/* data structures */
static struct vpfe_config_params config_params = {
	.min_numbuffers = 3,
	.numbuffers = 3,
	.min_bufsize = 800 * 600 * 2,
	.device_bufsize = 800 * 600 * 2,
};

const struct vpfe_standard vpfe_standards[] = {
	{V4L2_STD_525_60, 800, 600, {1, 1}, 0},
	{V4L2_STD_625_50, 720, 576, {54, 59}, 1},
};

/* map mbus_fmt to pixelformat */
void mbus_to_pix(struct vpfe_device *vpfe_dev,
		const struct v4l2_mbus_framefmt *mbus,
		struct v4l2_pix_format *pix, unsigned int *bpp)
{
	unsigned int bus_width =
			vpfe_dev->current_subdev->isif_if_params.bus_width;

	switch (mbus->code) {
	case V4L2_MBUS_FMT_UYVY8_2X8:
		pix->pixelformat = V4L2_PIX_FMT_UYVY;
		*bpp = (bus_width == 10) ? 4 : 2;
		break;

	case V4L2_MBUS_FMT_YUYV8_2X8:
		pix->pixelformat = V4L2_PIX_FMT_YUYV;
		*bpp = (bus_width == 10) ? 4 : 2;
		break;

	case V4L2_MBUS_FMT_YVYU8_2X8:
		pix->pixelformat = V4L2_PIX_FMT_YVYU;
		*bpp = (bus_width == 10) ? 4 : 2;
		break;

	case V4L2_MBUS_FMT_VYUY8_2X8:
		pix->pixelformat = V4L2_PIX_FMT_VYUY;
		*bpp = (bus_width == 10) ? 4 : 2;
		break;

	case V4L2_MBUS_FMT_YDYUYDYV8_1X16:
		pix->pixelformat = V4L2_PIX_FMT_NV12;
		*bpp = (bus_width == 10) ? 2 : 1;
		break;

	case V4L2_MBUS_FMT_Y12_1X12:
		pix->pixelformat = V4L2_PIX_FMT_YUV420;
		*bpp = (bus_width == 10) ? 2 : 1;
		break;

	case V4L2_MBUS_FMT_YUYV8_1_5X8:
		pix->pixelformat = V4L2_PIX_FMT_YUV420;
		*bpp = (bus_width == 10) ? 2 : 1;
		break;

	case V4L2_MBUS_FMT_SBGGR8_1X8:
		pix->pixelformat = V4L2_PIX_FMT_SBGGR8;
		*bpp = (bus_width == 10) ? 2 : 1;
		break;

	case V4L2_MBUS_FMT_SGBRG8_1X8:
		pix->pixelformat = V4L2_PIX_FMT_SGBRG8;
		*bpp = (bus_width == 10) ? 2 : 1;
		break;

	case V4L2_MBUS_FMT_SGRBG8_1X8:
		pix->pixelformat = V4L2_PIX_FMT_SGRBG8;
		*bpp = (bus_width == 10) ? 2 : 1;
		break;

	case V4L2_MBUS_FMT_SRGGB8_1X8:
		pix->pixelformat = V4L2_PIX_FMT_SRGGB8;
		*bpp = (bus_width == 10) ? 2 : 1;
		break;

	case V4L2_MBUS_FMT_BGR565_2X8_BE:
		pix->pixelformat = V4L2_PIX_FMT_RGB565;
		*bpp = (bus_width == 10) ? 4 : 2;
		break;

	case V4L2_MBUS_FMT_RGB565_2X8_BE:
		pix->pixelformat = V4L2_PIX_FMT_RGB565X;
		*bpp = (bus_width == 10) ? 4 : 2;
		break;

	default:
		pr_err("Invalid mbus code set\n");
		*bpp = 1;
	}

	pix->bytesperline = pix->width * *bpp;
	/* pitch should be 32 bytes aligned */
	pix->bytesperline = ALIGN(pix->bytesperline, 32);
	if (pix->pixelformat == V4L2_PIX_FMT_NV12)
		pix->sizeimage = pix->bytesperline * pix->height +
				((pix->bytesperline * pix->height) >> 1);
	else if (pix->pixelformat == V4L2_PIX_FMT_YUV420)
		pix->sizeimage = pix->bytesperline * pix->height +
				((pix->bytesperline * pix->height) >> 1);
	else
		pix->sizeimage = pix->bytesperline * pix->height;
}

/* map mbus_fmt to pixelformat */
void pix_to_mbus(struct vpfe_device *vpfe_dev,
		struct v4l2_pix_format *pix,
		struct v4l2_mbus_framefmt *mbus)
{
	memset(mbus, 0, sizeof(*mbus));
	mbus->width = pix->width;
	mbus->height = pix->height;

	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_UYVY:
		mbus->code = V4L2_MBUS_FMT_UYVY8_2X8;
		break;

	case V4L2_PIX_FMT_YUYV:
		mbus->code = V4L2_MBUS_FMT_YUYV8_2X8;
		break;

	case V4L2_PIX_FMT_YVYU:
		mbus->code = V4L2_MBUS_FMT_YVYU8_2X8;
		break;

	case V4L2_PIX_FMT_VYUY:
		mbus->code = V4L2_MBUS_FMT_VYUY8_2X8;
		break;

	case V4L2_PIX_FMT_NV12:
		mbus->code = V4L2_MBUS_FMT_YDYUYDYV8_1X16;
		break;

	case V4L2_PIX_FMT_YUV420:
		mbus->code = V4L2_MBUS_FMT_Y12_1X12;
		break;

	case V4L2_PIX_FMT_SBGGR8:
		mbus->code = V4L2_MBUS_FMT_SBGGR8_1X8;
		break;

	case V4L2_PIX_FMT_SGBRG8:
		mbus->code = V4L2_MBUS_FMT_SGBRG8_1X8;
		break;

	case V4L2_PIX_FMT_SGRBG8:
		mbus->code = V4L2_MBUS_FMT_SGRBG8_1X8;
		break;

	case V4L2_PIX_FMT_SRGGB8:
		mbus->code = V4L2_MBUS_FMT_SRGGB8_1X8;
		break;

	case V4L2_PIX_FMT_RGB565:
		mbus->code = V4L2_MBUS_FMT_BGR565_2X8_BE;
		break;

	case V4L2_PIX_FMT_RGB565X:
		mbus->code = V4L2_MBUS_FMT_RGB565_2X8_BE;
		break;

	default:
		pr_err("Invalid pixel code\n");
	}

	mbus->colorspace = pix->colorspace;
	mbus->field = pix->field;
}

/*  Print Four-character-code (FOURCC) */
static char *print_fourcc(u32 fmt)
{
	static char code[5];
	code[0] = (unsigned char)(fmt & 0xff);
	code[1] = (unsigned char)((fmt>>8) & 0xff);
	code[2] = (unsigned char)((fmt>>16) & 0xff);
	code[3] = (unsigned char)((fmt>>24) & 0xff);
	code[4] = '\0';

	return code;
}

static int cmp_v4l2_format(const struct v4l2_format *lhs,
	const struct v4l2_format *rhs)
{
	return lhs->type == rhs->type &&
		lhs->fmt.pix.width == rhs->fmt.pix.width &&
		lhs->fmt.pix.height == rhs->fmt.pix.height &&
		lhs->fmt.pix.pixelformat == rhs->fmt.pix.pixelformat &&
		lhs->fmt.pix.field == rhs->fmt.pix.field &&
		lhs->fmt.pix.colorspace == rhs->fmt.pix.colorspace;
}

/*
 * vpfe_get_isif_image_format - Get image parameters based on ISIF settings
 */
static int vpfe_get_isif_image_format(struct vpfe_device *vpfe_dev,
				 struct v4l2_format *f)
{
	struct v4l2_rect image_win;
	enum isif_buftype buf_type;
	enum isif_frmfmt frm_fmt;

	memset(f, 0, sizeof(*f));
	f->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	vpfe_dev->vpfe_isif.hw_ops.get_image_window(
			&vpfe_dev->vpfe_isif, &image_win);
	f->fmt.pix.width = image_win.width;
	f->fmt.pix.height = image_win.height;
	f->fmt.pix.bytesperline =
		vpfe_dev->vpfe_isif.hw_ops.get_line_length(
			&vpfe_dev->vpfe_isif);
	f->fmt.pix.sizeimage = f->fmt.pix.bytesperline *
				f->fmt.pix.height;
	buf_type = vpfe_dev->vpfe_isif.hw_ops.get_buftype(
				&vpfe_dev->vpfe_isif);
	f->fmt.pix.pixelformat =
		vpfe_dev->vpfe_isif.hw_ops.get_pixel_format(
				&vpfe_dev->vpfe_isif);
	frm_fmt =
		vpfe_dev->vpfe_isif.hw_ops.get_frame_format(
				&vpfe_dev->vpfe_isif);
	if (frm_fmt == ISIF_FRMFMT_PROGRESSIVE) {
		f->fmt.pix.field = V4L2_FIELD_NONE;
	} else if (frm_fmt == ISIF_FRMFMT_INTERLACED) {
		if (buf_type == ISIF_BUFTYPE_FLD_INTERLEAVED) {
			f->fmt.pix.field = V4L2_FIELD_INTERLACED;
		 } else if (buf_type == ISIF_BUFTYPE_FLD_SEPARATED) {
			f->fmt.pix.field = V4L2_FIELD_SEQ_TB;
		} else {
			dev_err(vpfe_dev->pdev, "Invalid buf_type\n");
			return -EINVAL;
		}
	} else {
		dev_err(vpfe_dev->pdev, "Invalid frm_fmt\n");
		return -EINVAL;
	}
	return 0;
}

/*
 * vpfe_config_isif_image_format()
 * For a pix format, configure isif to setup the capture
 */
static int vpfe_config_isif_image_format(struct vpfe_device *vpfe_dev)
{
	enum isif_frmfmt frm_fmt = ISIF_FRMFMT_INTERLACED;
	int ret = 0;

	dev_dbg(vpfe_dev->pdev, "vpfe_config_isif_image_format\n");

	dev_dbg(vpfe_dev->pdev, "pixelformat: %s\n",
		print_fourcc(vpfe_dev->fmt.fmt.pix.pixelformat));

	if (vpfe_dev->vpfe_isif.hw_ops.set_pixel_format(
			&vpfe_dev->vpfe_isif,
			vpfe_dev->fmt.fmt.pix.pixelformat) < 0) {
		dev_err(vpfe_dev->pdev,
			"couldn't set pix format in isif\n");
		return -EINVAL;
	}
	/* configure the image window */
	vpfe_dev->vpfe_isif.hw_ops.set_image_window(
				&vpfe_dev->vpfe_isif,
				&vpfe_dev->crop, vpfe_dev->bpp);

	switch (vpfe_dev->fmt.fmt.pix.field) {
	case V4L2_FIELD_INTERLACED:
		/* do nothing, since it is default */
		ret = vpfe_dev->vpfe_isif.hw_ops.set_buftype(
				&vpfe_dev->vpfe_isif,
				ISIF_BUFTYPE_FLD_INTERLEAVED);
		break;
	case V4L2_FIELD_NONE:
		frm_fmt = ISIF_FRMFMT_PROGRESSIVE;
		/* buffer type only applicable for interlaced scan */
		break;
	case V4L2_FIELD_SEQ_TB:
		ret = vpfe_dev->vpfe_isif.hw_ops.set_buftype(
				&vpfe_dev->vpfe_isif,
				ISIF_BUFTYPE_FLD_SEPARATED);
		break;
	default:
		return -EINVAL;
	}

	/* set the frame format */
	if (!ret)
		ret = vpfe_dev->vpfe_isif.hw_ops.set_frame_format(
				&vpfe_dev->vpfe_isif,
				frm_fmt);
	return ret;
}
/*
 * vpfe_config_image_format()
 * For a given standard, this functions sets up the default
 * pix format & crop values in the vpfe device and isif.  It first
 * starts with defaults based values from the standard table.
 * It then checks if sub device support g_mbus_fmt and then override the
 * values based on that.Sets crop values to match with scan resolution
 * starting at 0,0. It calls vpfe_config_isif_image_format() set the
 * values in isif
 */
static int vpfe_config_image_format(struct vpfe_device *vpfe_dev,
				    v4l2_std_id std_id)
{
	struct vpfe_subdev_info *sdinfo = vpfe_dev->current_subdev;
	struct v4l2_mbus_framefmt mbus_fmt;
	struct v4l2_pix_format *pix = &vpfe_dev->fmt.fmt.pix;
	struct v4l2_pix_format pix_test;
	struct v4l2_subdev_format sd_fmt;
	int i, ret = 0;

	for (i = 0; i < ARRAY_SIZE(vpfe_standards); i++) {
		if (vpfe_standards[i].std_id & std_id) {
			vpfe_dev->std_info.active_pixels =
					vpfe_standards[i].width;
			vpfe_dev->std_info.active_lines =
					vpfe_standards[i].height;
			vpfe_dev->std_info.frame_format =
					vpfe_standards[i].frame_format;
			vpfe_dev->std_index = i;
			break;
		}
	}

	if (i ==  ARRAY_SIZE(vpfe_standards)) {
		dev_err(vpfe_dev->pdev, "standard not supported\n");
		return -EINVAL;
	}

	vpfe_dev->crop.top = 0;
	vpfe_dev->crop.left = 0;
	vpfe_dev->crop.width = vpfe_dev->std_info.active_pixels;
	vpfe_dev->crop.height = vpfe_dev->std_info.active_lines;
	pix->width = vpfe_dev->crop.width;
	pix->height = vpfe_dev->crop.height;

	/* first field and frame format based on standard frame format */
	if (vpfe_dev->std_info.frame_format) {
		pix->field = V4L2_FIELD_INTERLACED;
		/* assume V4L2_PIX_FMT_UYVY as default */
		pix->pixelformat = V4L2_PIX_FMT_YUYV;
		v4l2_fill_mbus_format(&mbus_fmt, pix,
				      V4L2_MBUS_FMT_YUYV10_2X10);
	} else {
		pix->field = V4L2_FIELD_NONE;
		/* assume V4L2_PIX_FMT_SBGGR8 */
		pix->pixelformat = V4L2_PIX_FMT_YUYV;
		v4l2_fill_mbus_format(&mbus_fmt, pix,
				      V4L2_MBUS_FMT_YUYV10_2X10);
	}

	/* copy pix format for testing */
	pix_test = *pix;

	if (!v4l2_device_has_op(&vpfe_dev->v4l2_dev, video, g_mbus_fmt)) {
		dev_dbg(vpfe_dev->pdev,
			"v4l2_device_has_op: sub device '%s' does not support g_mbus_fmt\n",
			sdinfo->name);

		sd_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;

		ret = v4l2_device_call_until_err(&vpfe_dev->v4l2_dev,
				sdinfo->grp_id, pad, get_fmt, NULL, &sd_fmt);

		if (ret && ret != -ENOIOCTLCMD) {
			dev_err(vpfe_dev->pdev,
				"error in getting g_mbus_fmt from sub device\n");
			return ret;
		}
	} else {
		/* if sub device supports g_mbus_fmt, override the defaults */
		ret = v4l2_device_call_until_err(&vpfe_dev->v4l2_dev,
				sdinfo->grp_id, video, g_mbus_fmt, &mbus_fmt);

		if (ret && ret != -ENOIOCTLCMD) {
			dev_err(vpfe_dev->pdev,
				"error in getting g_mbus_fmt from sub device\n");
			return ret;
		} else if (ret == -ENOIOCTLCMD) {
			dev_dbg(vpfe_dev->pdev,
				"sub device '%s' does not support g_mbus_fmt\n",
				sdinfo->name);
		} else {
			dev_dbg(vpfe_dev->pdev,
				"v4l2_subdev_call g_mbus_fmt: %d\n", ret);
		}
	}

	/* convert mbus_format to v4l2_format */
	v4l2_fill_pix_format(pix, &sd_fmt.format);
	mbus_to_pix(vpfe_dev, &sd_fmt.format, pix, &vpfe_dev->bpp);

	dev_dbg(vpfe_dev->pdev, "vpfe_config_image_format pix 1: size %dx%d pixelformat: %s bytesperline = %d, sizeimage = %d\n",
		pix->width, pix->height,
		print_fourcc(pix->pixelformat),
		pix->bytesperline, pix->sizeimage);

	/* Update the crop window based on found values */
	vpfe_dev->crop.width = pix->width;
	vpfe_dev->crop.height = pix->height;

	/* Sets the values in ISIF */
	ret = vpfe_config_isif_image_format(vpfe_dev);

	return ret;
}

static int vpfe_initialize_device(struct vpfe_device *vpfe_dev)
{
	int ret = 0;

	/* set first input of current subdevice as the current input */
	vpfe_dev->current_input = 0;

	/* set default standard */
	vpfe_dev->std_index = 0;

	/* Configure the default format information */
	ret = vpfe_config_image_format(vpfe_dev,
				vpfe_standards[vpfe_dev->std_index].std_id);
	if (ret)
		return ret;

	/* now open the isif device to initialize it */
	ret = vpfe_dev->vpfe_isif.hw_ops.open(
				&vpfe_dev->vpfe_isif,
				vpfe_dev->pdev);
	if (!ret)
		vpfe_dev->initialized = 1;

	/* Clear all VPFE/ISIF interrupts */
	if (vpfe_dev->vpfe_isif.hw_ops.clear_intr)
		vpfe_dev->vpfe_isif.hw_ops.clear_intr(
				&vpfe_dev->vpfe_isif, -1);

	return ret;
}

/*
 * vpfe_open : It creates object of file handle structure and
 * stores it in private_data  member of filepointer
 */
static int vpfe_open(struct file *file)
{
	struct vpfe_device *vpfe_dev = video_drvdata(file);
	struct vpfe_fh *fh;

	dev_dbg(vpfe_dev->pdev, "vpfe_open\n");

	if (!vpfe_dev->cfg->num_subdevs) {
		dev_err(vpfe_dev->pdev, "No decoder/sensor registered\n");
		return -ENODEV;
	}

	/* Allocate memory for the file handle object */
	fh = kmalloc(sizeof(*fh), GFP_KERNEL);
	if (NULL == fh) {
		dev_err(vpfe_dev->pdev,
			"unable to allocate memory for file handle object\n");
		return -ENOMEM;
	}
	/* store pointer to fh in private_data member of file */
	file->private_data = fh;
	fh->vpfe_dev = vpfe_dev;
	mutex_lock(&vpfe_dev->lock);
	/* If decoder is not initialized. initialize it */
	if (!vpfe_dev->initialized) {
		if (vpfe_initialize_device(vpfe_dev)) {
			mutex_unlock(&vpfe_dev->lock);
			return -ENODEV;
		}
	}
	/* Increment device usrs counter */
	vpfe_dev->usrs++;
	/* Set io_allowed member to false */
	fh->io_allowed = 0;
	/* Initialize priority of this instance to default priority */
	fh->prio = V4L2_PRIORITY_UNSET;
	v4l2_prio_open(&vpfe_dev->prio, &fh->prio);
	mutex_unlock(&vpfe_dev->lock);
	return 0;
}

/**
 * vpfe_schedule_next_buffer: set next buffer address for capture
 * @vpfe_dev : ptr to device
 *
 * This function will get next buffer from the dma queue and
 * set the buffer address in the vpfe register for capture.
 * the buffer is marked active
 *
 * Assumes caller is holding vpfe_dev->dma_queue_lock already
 */
static void vpfe_schedule_next_buffer(struct vpfe_device *vpfe_dev)
{
	unsigned long addr;

	vpfe_dev->next_frm = list_entry(vpfe_dev->dma_queue.next,
					struct vpfe_cap_buffer, list);
	list_del(&vpfe_dev->next_frm->list);

	vpfe_dev->next_frm->vb.state = VB2_BUF_STATE_ACTIVE;
	addr = vb2_dma_contig_plane_dma_addr(&vpfe_dev->next_frm->vb, 0);

	vpfe_dev->vpfe_isif.hw_ops.setfbaddr(&vpfe_dev->vpfe_isif, addr);
}

static void vpfe_schedule_bottom_field(struct vpfe_device *vpfe_dev)
{
	unsigned long addr;

	addr = vb2_dma_contig_plane_dma_addr(&vpfe_dev->next_frm->vb, 0);
	addr += vpfe_dev->field_off;
	vpfe_dev->vpfe_isif.hw_ops.setfbaddr(&vpfe_dev->vpfe_isif, addr);
}

/*
 * vpfe_process_buffer_complete: process a completed buffer
 * @vpfe_dev : ptr to device
 *
 * This function time stamp the buffer and mark it as DONE. It also
 * wake up any process waiting on the QUEUE and set the next buffer
 * as current
 */
static void vpfe_process_buffer_complete(struct vpfe_device *vpfe_dev)
{
	v4l2_get_timestamp(&vpfe_dev->cur_frm->vb.v4l2_buf.timestamp);

	vb2_buffer_done(&vpfe_dev->cur_frm->vb, VB2_BUF_STATE_DONE);
	vpfe_dev->cur_frm = vpfe_dev->next_frm;
}

/*
 * vpfe_isr : ISR handler for vpfe capture (VINT0)
 * @irq: irq number
 * @dev_id: dev_id ptr
 *
 * It changes status of the captured buffer, takes next buffer from the queue
 * and sets its address in VPFE registers
 */
static irqreturn_t vpfe_isr(int irq, void *dev_id)
{
	struct vpfe_device *vpfe_dev = dev_id;
	int vpfe_intr_status;
	enum v4l2_field field;
	int fid;

	/* Which interrupt did we get */
	vpfe_intr_status = vpfe_dev->vpfe_isif.hw_ops.intr_status(
					&vpfe_dev->vpfe_isif);

	/* if streaming not started, don't do anything */
	if (!vpfe_dev->started)
		goto clear_intr;

	if (vpfe_intr_status & ISIF_VPFE_VDINT0) {
		field = vpfe_dev->fmt.fmt.pix.field;

		/* only for 6446 this will be applicable */
		if (NULL != vpfe_dev->vpfe_isif.hw_ops.reset)
			vpfe_dev->vpfe_isif.hw_ops.reset(
					&vpfe_dev->vpfe_isif);

		if (field == V4L2_FIELD_NONE) {
			/* handle progressive frame capture */
			if (vpfe_dev->cur_frm != vpfe_dev->next_frm)
				vpfe_process_buffer_complete(vpfe_dev);
			goto next_intr;
		}

		/* interlaced or TB capture check which field
		   we are in hardware */
		fid = vpfe_dev->vpfe_isif.hw_ops.getfid(
					&vpfe_dev->vpfe_isif);

		/* switch the software maintained field id */
		vpfe_dev->field_id ^= 1;
		if (fid == vpfe_dev->field_id) {
			/* we are in-sync here,continue */
			if (fid == 0) {
				/*
				 * One frame is just being captured. If the
				 * next frame is available, release the
				 * current frame and move on
				 */
				if (vpfe_dev->cur_frm != vpfe_dev->next_frm)
					vpfe_process_buffer_complete(vpfe_dev);
				/*
				 * based on whether the two fields are stored
				 * interleavely or separately in memory,
				 * reconfigure the ISIF memory address
				 */
				if (field == V4L2_FIELD_SEQ_TB)
					vpfe_schedule_bottom_field(vpfe_dev);

				goto next_intr;
			}
			/*
			 * if one field is just being captured configure
			 * the next frame get the next frame from the empty
			 * queue if no frame is available hold on to the
			 * current buffer
			 */
			spin_lock(&vpfe_dev->dma_queue_lock);
			if (!list_empty(&vpfe_dev->dma_queue) &&
			    vpfe_dev->cur_frm == vpfe_dev->next_frm)
				vpfe_schedule_next_buffer(vpfe_dev);
			spin_unlock(&vpfe_dev->dma_queue_lock);
		} else if (fid == 0) {
			/*
			 * out of sync. Recover from any hardware out-of-sync.
			 * May loose one frame
			 */
			vpfe_dev->field_id = fid;
		}
	}

next_intr:
	if (vpfe_intr_status & ISIF_VPFE_VDINT1) {
		spin_lock(&vpfe_dev->dma_queue_lock);
		if ((vpfe_dev->fmt.fmt.pix.field == V4L2_FIELD_NONE) &&
		    !list_empty(&vpfe_dev->dma_queue) &&
		    vpfe_dev->cur_frm == vpfe_dev->next_frm)
			vpfe_schedule_next_buffer(vpfe_dev);
		spin_unlock(&vpfe_dev->dma_queue_lock);
	}

clear_intr:
	if (vpfe_dev->vpfe_isif.hw_ops.clear_intr)
		vpfe_dev->vpfe_isif.hw_ops.clear_intr(
				&vpfe_dev->vpfe_isif, vpfe_intr_status);

	return IRQ_HANDLED;
}

static void vpfe_detach_irq(struct vpfe_device *vpfe_dev)
{
	enum isif_frmfmt frame_format;
	unsigned int intr = ISIF_VPFE_VDINT0;

	frame_format = vpfe_dev->vpfe_isif.hw_ops.get_frame_format(
					&vpfe_dev->vpfe_isif);
	if (frame_format == ISIF_FRMFMT_PROGRESSIVE)
		intr |= ISIF_VPFE_VDINT1;

	vpfe_dev->vpfe_isif.hw_ops.intr_disable(
			&vpfe_dev->vpfe_isif, intr);
}

static int vpfe_attach_irq(struct vpfe_device *vpfe_dev)
{
	enum isif_frmfmt frame_format;
	unsigned int intr = ISIF_VPFE_VDINT0;

	frame_format = vpfe_dev->vpfe_isif.hw_ops.get_frame_format(
					&vpfe_dev->vpfe_isif);
	if (frame_format == ISIF_FRMFMT_PROGRESSIVE)
		intr |= ISIF_VPFE_VDINT1;

	vpfe_dev->vpfe_isif.hw_ops.intr_enable(
			&vpfe_dev->vpfe_isif, intr);
	return 0;
}

/* vpfe_stop_isif_capture: stop streaming in ccdc/isif */
static void vpfe_stop_isif_capture(struct vpfe_device *vpfe_dev)
{
	vpfe_dev->started = 0;
	vpfe_dev->vpfe_isif.hw_ops.enable(&vpfe_dev->vpfe_isif, 0);
	if (vpfe_dev->vpfe_isif.hw_ops.enable_out_to_sdram)
		vpfe_dev->vpfe_isif.hw_ops.enable_out_to_sdram(
					&vpfe_dev->vpfe_isif, 0);
}

 /*
 * vpfe_release : function to clean up file close
 * @filep: file pointer
 *
 * This function deletes buffer queue, frees the
 * buffers and the vpfe file  handle
 */
static int vpfe_release(struct file *file)
{
	struct vpfe_device *vpfe_dev = video_drvdata(file);
	struct vpfe_fh *fh = file->private_data;
	struct vpfe_subdev_info *sdinfo;
	int ret;

	dev_dbg(vpfe_dev->pdev, "vpfe_release\n");

	/* Get the device lock */
	mutex_lock(&vpfe_dev->lock);
	/* if this instance is doing IO */
	if (fh->io_allowed) {
		if (vpfe_dev->started) {
			sdinfo = vpfe_dev->current_subdev;
			ret = v4l2_device_call_until_err(&vpfe_dev->v4l2_dev,
							 sdinfo->grp_id,
							 video, s_stream, 0);
			if (ret && (ret != -ENOIOCTLCMD))
				dev_err(vpfe_dev->pdev,
					"stream off failed in subdev\n");
			vpfe_stop_isif_capture(vpfe_dev);
			vpfe_detach_irq(vpfe_dev);
			vb2_queue_release(&vpfe_dev->buffer_queue);
			vb2_dma_contig_cleanup_ctx(vpfe_dev->alloc_ctx);
		}
		vpfe_dev->io_usrs = 0;
		vpfe_dev->numbuffers = config_params.numbuffers;
	}

	/* Decrement device usrs counter */
	vpfe_dev->usrs--;
	/* Close the priority */
	v4l2_prio_close(&vpfe_dev->prio, fh->prio);
	/* If this is the last file handle */
	if (!vpfe_dev->usrs) {
		vpfe_dev->initialized = 0;
		if (vpfe_dev->vpfe_isif.hw_ops.close)
			vpfe_dev->vpfe_isif.hw_ops.close(
				&vpfe_dev->vpfe_isif, vpfe_dev->pdev);
	}
	mutex_unlock(&vpfe_dev->lock);
	file->private_data = NULL;
	/* Free memory allocated to file handle object */
	kfree(fh);
	return 0;
}

/*
 * vpfe_mmap : It is used to map kernel space buffers into user spaces
 * @file: file pointer
 * @vma: ptr to vm_area_struct
 */
static int vpfe_mmap(struct file *file, struct vm_area_struct *vma)
{
	/* Get the device object and file handle object */
	struct vpfe_device *vpfe_dev = video_drvdata(file);
	int ret;

	dev_dbg(vpfe_dev->pdev, "vpfe_mmap\n");
	if (mutex_lock_interruptible(&vpfe_dev->lock))
		return -ERESTARTSYS;
	ret = vb2_mmap(&vpfe_dev->buffer_queue, vma);
	mutex_unlock(&vpfe_dev->lock);
	return ret;
}

/*
 * vpfe_poll: It is used for select/poll system call
 * @filep: file pointer
 * @wait: poll table to wait
 */
static unsigned int vpfe_poll(struct file *file, poll_table *wait)
{
	struct vpfe_device *vpfe_dev = video_drvdata(file);
	int ret = 0;

	if (vpfe_dev->started) {
		mutex_lock(&vpfe_dev->lock);
		ret = vb2_poll(&vpfe_dev->buffer_queue, file, wait);
		mutex_unlock(&vpfe_dev->lock);
	}

	return ret;
}

/* vpfe capture driver file operations */
static const struct v4l2_file_operations vpfe_fops = {
	.owner = THIS_MODULE,
	.open = vpfe_open,
	.release = vpfe_release,
	.unlocked_ioctl = video_ioctl2,
	.mmap = vpfe_mmap,
	.poll = vpfe_poll
};

static int vpfe_querycap(struct file *file, void  *priv,
			       struct v4l2_capability *cap)
{
	struct vpfe_device *vpfe_dev = video_drvdata(file);

	dev_dbg(vpfe_dev->pdev, "vpfe_querycap\n");

	cap->version = VPFE_CAPTURE_VERSION_CODE;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	strlcpy(cap->driver, CAPTURE_DRV_NAME, sizeof(cap->driver));
	strlcpy(cap->bus_info, "VPFE", sizeof(cap->bus_info));
	strlcpy(cap->card, vpfe_dev->cfg->card_name, sizeof(cap->card));
	return 0;
}

/* get the subdev which is connected to the output video node */
static struct v4l2_subdev *
vpfe_remote_subdev(struct vpfe_device *vpfe_dev, u32 *pad)
{
	struct media_pad *remote = media_entity_remote_pad(&vpfe_dev->pad);

	dev_dbg(vpfe_dev->pdev, "vpfe_remote_subdev: remote:%x\n",
		(unsigned int)remote);

	if (remote == NULL) {
		return NULL;
	} else if ((remote->entity->type & MEDIA_ENT_T_V4L2_SUBDEV) == 0) {
		dev_dbg(vpfe_dev->pdev,
			"vpfe_remote_subdev: remote->entity->type:%d was expecting %d\n",
			remote->entity->type, MEDIA_ENT_T_V4L2_SUBDEV);
		return NULL;
	}

	if (pad)
		*pad = remote->index;

	return media_entity_to_v4l2_subdev(remote->entity);
}

/* get the format set at output pad of the adjacent subdev */
static int __vpfe_get_format(struct vpfe_device *vpfe_dev,
			struct v4l2_format *format, unsigned int *bpp)
{
	struct v4l2_subdev_format fmt;
	struct v4l2_subdev *subdev;
	struct media_pad *remote;
	u32 pad;
	int ret;

	dev_dbg(vpfe_dev->pdev, "__vpfe_get_format\n");

	subdev = vpfe_remote_subdev(vpfe_dev, &pad);
	if (subdev == NULL)
		return -EINVAL;

	fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	remote = media_entity_remote_pad(&vpfe_dev->pad);
	fmt.pad = remote->index;

	ret = v4l2_subdev_call(subdev, pad, get_fmt, NULL, &fmt);
	if (ret == -ENOIOCTLCMD)
		return -EINVAL;

	format->type = vpfe_dev->fmt.type;

	/* convert mbus_format to v4l2_format */
	v4l2_fill_pix_format(&format->fmt.pix, &fmt.format);
	mbus_to_pix(vpfe_dev, &fmt.format, &format->fmt.pix, bpp);
	dev_dbg(vpfe_dev->pdev, "__vpfe_get_format size %dx%d (%s) bytesperline = %d, sizeimage = %d, bpp = %d\n",
		format->fmt.pix.width, format->fmt.pix.height,
		print_fourcc(format->fmt.pix.pixelformat),
		format->fmt.pix.bytesperline, format->fmt.pix.sizeimage, *bpp);

	return 0;
}

/* set the format at output pad of the adjacent subdev */
static int __vpfe_set_format(struct vpfe_device *vpfe_dev,
			struct v4l2_format *format, unsigned int *bpp)
{
	struct v4l2_subdev_format fmt;
	struct v4l2_subdev *subdev;
	struct media_pad *remote;
	u32 pad;
	int ret;

	dev_dbg(vpfe_dev->pdev, "__vpfe_set_format\n");

	subdev = vpfe_remote_subdev(vpfe_dev, &pad);
	if (subdev == NULL)
		return -EINVAL;

	fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	remote = media_entity_remote_pad(&vpfe_dev->pad);
	fmt.pad = remote->index;

	pix_to_mbus(vpfe_dev, &format->fmt.pix, &fmt.format);

	ret = v4l2_subdev_call(subdev, pad, set_fmt, NULL, &fmt);
	if (ret == -ENOIOCTLCMD)
		return -EINVAL;

	format->type = vpfe_dev->fmt.type;

	/* convert mbus_format to v4l2_format */
	v4l2_fill_pix_format(&format->fmt.pix, &fmt.format);
	mbus_to_pix(vpfe_dev, &fmt.format, &format->fmt.pix, bpp);
	dev_dbg(vpfe_dev->pdev, "__vpfe_set_format size %dx%d (%s) bytesperline = %d, sizeimage = %d, bpp = %d\n",
		format->fmt.pix.width, format->fmt.pix.height,
		print_fourcc(format->fmt.pix.pixelformat),
		format->fmt.pix.bytesperline, format->fmt.pix.sizeimage, *bpp);

	return 0;
}

static int vpfe_g_fmt(struct file *file, void *priv,
				struct v4l2_format *fmt)
{
	struct vpfe_device *vpfe_dev = video_drvdata(file);
	struct v4l2_format format;
	unsigned int bpp;
	int ret = 0;

	dev_dbg(vpfe_dev->pdev, "vpfe_g_fmt\n");

	ret = __vpfe_get_format(vpfe_dev, &format, &bpp);
	if (ret)
		return ret;

	/* Fill in the information about format */
	*fmt = vpfe_dev->fmt;
	return ret;
}

static int vpfe_enum_fmt(struct file *file, void  *priv,
				   struct v4l2_fmtdesc *fmt)
{
	struct vpfe_device *vpfe_dev = video_drvdata(file);
	struct v4l2_mbus_framefmt mbus;
	struct v4l2_subdev_mbus_code_enum mbus_code;
	struct v4l2_subdev *subdev;
	struct media_pad *remote;
	struct v4l2_pix_format pix;
	u32 pad, bpp;
	int ret;

	dev_dbg(vpfe_dev->pdev, "vpfe_enum_format\n");

	subdev = vpfe_remote_subdev(vpfe_dev, &pad);
	if (subdev == NULL)
		return -EINVAL;

	remote = media_entity_remote_pad(&vpfe_dev->pad);
	mbus_code.index = fmt->index;
	ret = v4l2_subdev_call(subdev, pad, enum_mbus_code, NULL, &mbus_code);
	if (ret)
		return -EINVAL;

	/* convert mbus_format to v4l2_format */
	/* just populate pix with dummy size value, those don't matter here */
	pix.width = 640;
	pix.height = 480;

	mbus.code = mbus_code.code;
	mbus_to_pix(vpfe_dev, &mbus, &pix, &bpp);

	dev_dbg(vpfe_dev->pdev, "vpfe_enum_format: mbus index: %d code: %x pixelformat: %s\n",
		mbus_code.index, mbus_code.code, print_fourcc(pix.pixelformat));

	/* Should be V4L2_BUF_TYPE_VIDEO_CAPTURE */
	fmt->type = vpfe_dev->fmt.type;
	fmt->pixelformat = pix.pixelformat;

	return 0;
}

static int vpfe_s_fmt(struct file *file, void *priv,
				struct v4l2_format *fmt)
{
	struct vpfe_device *vpfe_dev = video_drvdata(file);
	struct v4l2_format format;
	unsigned int bpp;
	int ret = 0;

	dev_dbg(vpfe_dev->pdev, "vpfe_s_fmt\n");

	/* If streaming is started, return error */
	if (vpfe_dev->started) {
		dev_err(vpfe_dev->pdev, "Streaming is started\n");
		return -EBUSY;
	}

	/* Check for valid frame format */
	ret = __vpfe_get_format(vpfe_dev, &format, &bpp);
	if (ret)
		return ret;
	if (!cmp_v4l2_format(fmt, &format)) {
		/* Sensor format is different from the requested format
		 * so we need to change it
		 */
		ret = __vpfe_set_format(vpfe_dev, fmt, &bpp);
		if (ret)
			return ret;
	} else /* Just make sure all of the fields are consistent */
		*fmt = format;

	/* store the pixel format in the device  object */
	ret = mutex_lock_interruptible(&vpfe_dev->lock);
	if (ret)
		return ret;

	/* First detach any IRQ if currently attached */
	vpfe_detach_irq(vpfe_dev);
	vpfe_dev->fmt = *fmt;
	vpfe_dev->bpp = bpp;

	/* Update the crop window based on found values */
	vpfe_dev->crop.width = fmt->fmt.pix.width;
	vpfe_dev->crop.height = fmt->fmt.pix.height;

	/* set image capture parameters in the isif */
	ret = vpfe_config_isif_image_format(vpfe_dev);
	mutex_unlock(&vpfe_dev->lock);
	return ret;
}

static int vpfe_try_fmt(struct file *file, void *priv,
				  struct v4l2_format *fmt)
{
	struct vpfe_device *vpfe_dev = video_drvdata(file);
	struct v4l2_format format;
	unsigned int bpp;
	int ret = 0;

	dev_dbg(vpfe_dev->pdev, "vpfe_try_fmt\n");

	ret = __vpfe_get_format(vpfe_dev, &format, &bpp);
	if (ret)
		return ret;

	*fmt = format;
	return 0;
}

static int vpfe_enum_size(struct file *file, void  *priv,
				   struct v4l2_frmsizeenum *fsize)
{
	struct vpfe_device *vpfe_dev = video_drvdata(file);
	struct v4l2_mbus_framefmt mbus;
	struct v4l2_subdev_frame_size_enum fse;
	struct v4l2_subdev *subdev;
	struct media_pad *remote;
	struct v4l2_pix_format pix;
	u32 pad;
	int ret;

	dev_dbg(vpfe_dev->pdev, "vpfe_enum_size\n");

	subdev = vpfe_remote_subdev(vpfe_dev, &pad);
	if (subdev == NULL)
		return -EINVAL;

	/* Construct pix from parameter and use defualt for the rest */
	pix.pixelformat = fsize->pixel_format;
	pix.width = 640;
	pix.height = 480;
	pix.colorspace = V4L2_COLORSPACE_JPEG;
	pix.field = V4L2_FIELD_NONE;
	pix_to_mbus(vpfe_dev, &pix, &mbus);
	remote = media_entity_remote_pad(&vpfe_dev->pad);
	fse.index = fsize->index;
	fse.pad = remote->index;
	fse.code = mbus.code;
	ret = v4l2_subdev_call(subdev, pad, enum_frame_size, NULL, &fse);
	if (ret)
		return -EINVAL;

	dev_dbg(vpfe_dev->pdev, "vpfe_enum_size: index: %d code: %x W:[%d,%d] H:[%d,%d]\n",
		fse.index, fse.code, fse.min_width, fse.max_width,
		fse.min_height, fse.max_height);

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = fse.max_width;
	fsize->discrete.height = fse.max_height;

	dev_dbg(vpfe_dev->pdev, "vpfe_enum_size: index: %d pixformat: %s size: %dx%d\n",
		fsize->index, print_fourcc(fsize->pixel_format),
		fsize->discrete.width, fsize->discrete.height);

	return 0;
}

/*
 * vpfe_get_subdev_input_index - Get subdev index and subdev input index for a
 * given app input index
 */
static int vpfe_get_subdev_input_index(struct vpfe_device *vpfe_dev,
					int *subdev_index,
					int *subdev_input_index,
					int app_input_index)
{
	struct vpfe_config *cfg = vpfe_dev->cfg;
	struct vpfe_subdev_info *sdinfo;
	int i, j = 0;

	for (i = 0; i < cfg->num_subdevs; i++) {
		sdinfo = &cfg->sub_devs[i];
		if (app_input_index < (j + sdinfo->num_inputs)) {
			*subdev_index = i;
			*subdev_input_index = app_input_index - j;
			return 0;
		}
		j += sdinfo->num_inputs;
	}
	return -EINVAL;
}

/*
 * vpfe_get_app_input - Get app input index for a given subdev input index
 * driver stores the input index of the current sub device and translate it
 * when application request the current input
 */
static int vpfe_get_app_input_index(struct vpfe_device *vpfe_dev,
				    int *app_input_index)
{
	struct vpfe_config *cfg = vpfe_dev->cfg;
	struct vpfe_subdev_info *sdinfo;
	int i, j = 0;

	for (i = 0; i < cfg->num_subdevs; i++) {
		sdinfo = &cfg->sub_devs[i];
		if (!strcmp(sdinfo->name, vpfe_dev->current_subdev->name)) {
			if (vpfe_dev->current_input >= sdinfo->num_inputs)
				return -1;
			*app_input_index = j + vpfe_dev->current_input;
			return 0;
		}
		j += sdinfo->num_inputs;
	}
	return -EINVAL;
}

static int vpfe_enum_input(struct file *file, void *priv,
				 struct v4l2_input *inp)
{
	struct vpfe_device *vpfe_dev = video_drvdata(file);
	struct vpfe_subdev_info *sdinfo;
	int subdev, index;

	dev_dbg(vpfe_dev->pdev, "vpfe_enum_input\n");

	if (vpfe_get_subdev_input_index(vpfe_dev,
					&subdev,
					&index,
					inp->index) < 0) {
		dev_dbg(vpfe_dev->pdev,
			"input information not found for the subdev\n");
		return -EINVAL;
	}
	sdinfo = &vpfe_dev->cfg->sub_devs[subdev];
	memcpy(inp, &sdinfo->inputs[index], sizeof(struct v4l2_input));
	return 0;
}

static int vpfe_g_input(struct file *file, void *priv, unsigned int *index)
{
	struct vpfe_device *vpfe_dev = video_drvdata(file);

	dev_dbg(vpfe_dev->pdev, "vpfe_g_input\n");

	return vpfe_get_app_input_index(vpfe_dev, index);
}

/* Assumes caller is holding vpfe_dev->lock */
static int vpfe_set_input(struct vpfe_device *vpfe_dev, unsigned int index)
{
	struct v4l2_subdev *sd;
	struct vpfe_subdev_info *sdinfo;
	int subdev_index = 0, inp_index = 0;
	struct vpfe_route *route;
	u32 input = 0, output = 0;
	int ret = -EINVAL;

	dev_dbg(vpfe_dev->pdev, "vpfe_set_input: index: %d\n", index);

	/*
	 * If streaming is started return device busy
	 * error
	 */
	if (vpfe_dev->started) {
		dev_err(vpfe_dev->pdev, "Streaming is on\n");
		ret = -EBUSY;
		goto get_out;
	}
	ret = vpfe_get_subdev_input_index(vpfe_dev,
					  &subdev_index,
					  &inp_index,
					  index);
	dev_dbg(vpfe_dev->pdev,
		"vpfe_set_input: after vpfe_get_subdev_input_index: ret %d\n",
		ret);

	if (ret < 0) {
		dev_err(vpfe_dev->pdev, "invalid input index\n");
		goto get_out;
	}

	sdinfo = &vpfe_dev->cfg->sub_devs[subdev_index];
	sd = vpfe_dev->sd[subdev_index];
	route = &sdinfo->routes[inp_index];
	if (route && sdinfo->can_route) {
		input = route->input;
		output = route->output;
		if (sd) {
			ret = v4l2_subdev_call(sd, video,
					s_routing, input, output, 0);
			dev_dbg(vpfe_dev->pdev,
				"vpfe_set_input: after s_routing: ret %d\n",
				ret);
		}

		if (ret) {
			dev_err(vpfe_dev->pdev,
				"vpfe_doioctl:error in setting input in sub device\n");
			ret = -EINVAL;
			goto get_out;
		}
	}

	vpfe_dev->current_subdev = sdinfo;
	if (sd)
		vpfe_dev->v4l2_dev.ctrl_handler = sd->ctrl_handler;
	vpfe_dev->current_input = index;
	vpfe_dev->std_index = 0;

	/* set the bus/interface parameter for the sub device in isif */
	ret = vpfe_dev->vpfe_isif.hw_ops.set_hw_if_params(
			&vpfe_dev->vpfe_isif, &sdinfo->isif_if_params);
	dev_dbg(vpfe_dev->pdev,
		"vpfe_set_input: after set_hw_if_params: ret %d\n", ret);
	if (ret)
		goto get_out;

	/* set the default image parameters in the device */
	ret = vpfe_config_image_format(vpfe_dev,
				vpfe_standards[vpfe_dev->std_index].std_id);
	dev_dbg(vpfe_dev->pdev,
		"vpfe_set_input: after vpfe_config_image_format: ret %d\n",
		ret);

get_out:
	return ret;
}

static int vpfe_s_input(struct file *file, void *priv, unsigned int index)
{
	struct vpfe_device *vpfe_dev = video_drvdata(file);
	int ret = -EINVAL;

	dev_dbg(vpfe_dev->pdev,
		"vpfe_s_input: index: %d\n", index);

	ret = mutex_lock_interruptible(&vpfe_dev->lock);
	if (ret)
		return ret;

	ret = vpfe_set_input(vpfe_dev, index);

	mutex_unlock(&vpfe_dev->lock);
	return ret;
}

static int vpfe_querystd(struct file *file, void *priv, v4l2_std_id *std_id)
{
	struct vpfe_device *vpfe_dev = video_drvdata(file);
	struct vpfe_subdev_info *sdinfo;
	int ret = 0;

	dev_dbg(vpfe_dev->pdev, "vpfe_querystd\n");

	ret = mutex_lock_interruptible(&vpfe_dev->lock);
	sdinfo = vpfe_dev->current_subdev;
	if (ret)
		return ret;
	/* Call querystd function of decoder device */
	ret = v4l2_device_call_until_err(&vpfe_dev->v4l2_dev, sdinfo->grp_id,
					 video, querystd, std_id);
	mutex_unlock(&vpfe_dev->lock);
	return ret;
}

static int vpfe_s_std(struct file *file, void *priv, v4l2_std_id std_id)
{
	struct vpfe_device *vpfe_dev = video_drvdata(file);
	struct vpfe_subdev_info *sdinfo;
	int ret = 0;

	dev_dbg(vpfe_dev->pdev, "vpfe_s_std\n");

	/* Call decoder driver function to set the standard */
	ret = mutex_lock_interruptible(&vpfe_dev->lock);
	if (ret)
		return ret;

	sdinfo = vpfe_dev->current_subdev;
	/* If streaming is started, return device busy error */
	if (vpfe_dev->started) {
		dev_err(vpfe_dev->pdev, "streaming is started\n");
		ret = -EBUSY;
		goto unlock_out;
	}

	ret = v4l2_device_call_until_err(&vpfe_dev->v4l2_dev, sdinfo->grp_id,
					 core, s_std, std_id);
	if (ret < 0) {
		dev_err(vpfe_dev->pdev, "Failed to set standard\n");
		goto unlock_out;
	}
	ret = vpfe_config_image_format(vpfe_dev, std_id);

unlock_out:
	mutex_unlock(&vpfe_dev->lock);
	return ret;
}

static int vpfe_g_std(struct file *file, void *priv, v4l2_std_id *std_id)
{
	struct vpfe_device *vpfe_dev = video_drvdata(file);

	dev_dbg(vpfe_dev->pdev, "vpfe_g_std\n");

	*std_id = vpfe_standards[vpfe_dev->std_index].std_id;
	return 0;
}
/*
 *  Videobuf operations
 */

/*
 * vpfe_buffer_queue_setup : Callback function for buffer setup.
 * @vq: vb2_queue ptr
 * @fmt: v4l2 format
 * @nbuffers: ptr to number of buffers requested by application
 * @nplanes:: contains number of distinct video planes needed to hold a frame
 * @sizes[]: contains the size (in bytes) of each plane.
 * @alloc_ctxs: ptr to allocation context
 *
 * This callback function is called when reqbuf() is called to adjust
 * the buffer count and buffer size
 */

static int vpfe_buffer_queue_setup(struct vb2_queue *vq,
				const struct v4l2_format *fmt,
				unsigned int *nbuffers, unsigned int *nplanes,
				unsigned int sizes[], void *alloc_ctxs[])
{
	struct vpfe_fh *fh = vb2_get_drv_priv(vq);
	struct vpfe_device *vpfe_dev = fh->vpfe_dev;
	unsigned long size;

	dev_dbg(vpfe_dev->pdev, "vpfe_buffer_queue_setup\n");
	size = vpfe_dev->fmt.fmt.pix.sizeimage;

	if (*nbuffers < config_params.min_numbuffers)
		*nbuffers = config_params.min_numbuffers;

	*nplanes = 1;
	sizes[0] = size;
	alloc_ctxs[0] = vpfe_dev->alloc_ctx;

	dev_dbg(vpfe_dev->pdev,
		"nbuffers=%d, size=%ld\n", *nbuffers, size);
	return 0;
}

/*
 * vpfe_buffer_prepare :  callback function for buffer prepare
 * @vb: ptr to vb2_buffer
 *
 * This is the callback function for buffer prepare when vb2_qbuf()
 * function is called. The buffer is prepared and user space virtual address
 * or user address is converted into  physical address
 */
static int vpfe_buffer_prepare(struct vb2_buffer *vb)
{
	struct vpfe_fh *fh = vb2_get_drv_priv(vb->vb2_queue);
	struct vpfe_device *vpfe_dev = fh->vpfe_dev;
	unsigned long addr;

	/* If buffer is not initialized, initialize it */
	if (vb->state != VB2_BUF_STATE_ACTIVE &&
	    vb->state != VB2_BUF_STATE_PREPARED) {
		vb2_set_plane_payload(vb, 0, vpfe_dev->fmt.fmt.pix.sizeimage);
		if (vb2_plane_vaddr(vb, 0) &&
		    vb2_get_plane_payload(vb, 0) > vb2_plane_size(vb, 0))
			goto exit;
		addr = vb2_dma_contig_plane_dma_addr(vb, 0);

		/* Make sure user addresses are aligned to 32 bytes */
		if (!ALIGN(addr, 32))
			goto exit;
	}
	return 0;
exit:
	dev_dbg(vpfe_dev->pdev, "buffer_prepare: offset is not aligned\n");
	return -EINVAL;
}

/*
 * vpfe_buffer_queue : Callback function to add buffer to DMA queue
 * @vb: ptr to vb2_buffer
 */
static void vpfe_buffer_queue(struct vb2_buffer *vb)
{
	/* Get the file handle object and device object */
	struct vpfe_fh *fh = vb2_get_drv_priv(vb->vb2_queue);
	struct vpfe_device *vpfe_dev = fh->vpfe_dev;
	struct vpfe_cap_buffer *buf = container_of(vb,
					struct vpfe_cap_buffer, vb);
	unsigned long flags;

	/* add the buffer to the DMA queue */
	spin_lock_irqsave(&vpfe_dev->dma_queue_lock, flags);
	list_add_tail(&buf->list, &vpfe_dev->dma_queue);
	spin_unlock_irqrestore(&vpfe_dev->dma_queue_lock, flags);
}

/*
 * vpfe_buffer_cleanup : Callback function to free buffer
 * @vb: ptr to vb2_buffer
 *
 * This function is called from the videobuf2 layer to free memory
 * allocated to  the buffers
 */
static void vpfe_buffer_cleanup(struct vb2_buffer *vb)
{
	/* Get the file handle object and device object */
	struct vpfe_fh *fh = vb2_get_drv_priv(vb->vb2_queue);
	struct vpfe_device *vpfe_dev = fh->vpfe_dev;
	struct vpfe_cap_buffer *buf = container_of(vb,
					struct vpfe_cap_buffer, vb);
	unsigned long flags;

	dev_dbg(vpfe_dev->pdev, "vpfe_buffer_cleanup\n");

	/*
	 * We need to flush the buffer from the dma queue since
	 * they are de-allocated
	 */
	spin_lock_irqsave(&vpfe_dev->dma_queue_lock, flags);
	if (vb->state == VB2_BUF_STATE_ACTIVE)
		list_del_init(&buf->list);
	spin_unlock_irqrestore(&vpfe_dev->dma_queue_lock, flags);
}

static void vpfe_wait_prepare(struct vb2_queue *vq)
{
	struct vpfe_fh *fh = vb2_get_drv_priv(vq);
	struct vpfe_device *vpfe_dev = fh->vpfe_dev;

	mutex_unlock(&vpfe_dev->lock);
}

static void vpfe_wait_finish(struct vb2_queue *vq)
{
	struct vpfe_fh *fh = vb2_get_drv_priv(vq);
	struct vpfe_device *vpfe_dev = fh->vpfe_dev;

	mutex_lock(&vpfe_dev->lock);
}

static int vpfe_buffer_init(struct vb2_buffer *vb)
{
	struct vpfe_cap_buffer *buf = container_of(vb,
					struct vpfe_cap_buffer, vb);

	INIT_LIST_HEAD(&buf->list);

	return 0;
}

static void vpfe_calculate_offsets(struct vpfe_device *vpfe_dev);
static void vpfe_start_isif_capture(struct vpfe_device *vpfe_dev);

static int vpfe_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct vpfe_fh *fh = vb2_get_drv_priv(vq);
	struct vpfe_device *vpfe_dev = fh->vpfe_dev;
	unsigned long addr = 0;
	unsigned long flags;
	int ret = 0;

	/* If buffer queue is empty, return error */
	spin_lock_irqsave(&vpfe_dev->dma_queue_lock, flags);
	if (list_empty(&vpfe_dev->dma_queue)) {
		spin_unlock_irqrestore(&vpfe_dev->irqlock, flags);
		dev_dbg(vpfe_dev->pdev, "buffer queue is empty\n");
		return -EIO;
	}

	/* Get the next frame from the buffer queue */
	vpfe_dev->next_frm = list_entry(vpfe_dev->dma_queue.next,
					struct vpfe_cap_buffer, list);
	vpfe_dev->cur_frm = vpfe_dev->next_frm;
	/* Remove buffer from the buffer queue */
	list_del(&vpfe_dev->cur_frm->list);
	spin_unlock_irqrestore(&vpfe_dev->dma_queue_lock, flags);

	/* Mark state of the current frame to active */
	vpfe_dev->cur_frm->vb.state = VB2_BUF_STATE_ACTIVE;
	/* Initialize field_id and started member */
	vpfe_dev->field_id = 0;
	addr = vb2_dma_contig_plane_dma_addr(&vpfe_dev->cur_frm->vb, 0);

	/* Calculate field offset */
	vpfe_calculate_offsets(vpfe_dev);

	if (vpfe_attach_irq(vpfe_dev) < 0) {
		dev_err(vpfe_dev->pdev,
			"Error in attaching interrupt handle\n");
		return -EFAULT;
	}
	if (vpfe_dev->vpfe_isif.hw_ops.configure(&vpfe_dev->vpfe_isif) < 0) {
		dev_err(vpfe_dev->pdev,
			"Error in configuring isif\n");
		return -EINVAL;
	}

	dev_dbg(vpfe_dev->pdev,
		"vpfe_start_streaming: initial buffer %lx\n", addr);
	vpfe_dev->vpfe_isif.hw_ops.setfbaddr(
			&vpfe_dev->vpfe_isif, (unsigned long)(addr));
	vpfe_start_isif_capture(vpfe_dev);
	return ret;
}

/* abort streaming and wait for last buffer */
static int vpfe_stop_streaming(struct vb2_queue *vq)
{
	struct vpfe_fh *fh = vb2_get_drv_priv(vq);
	struct vpfe_device *vpfe_dev = fh->vpfe_dev;
	unsigned long flags;

	if (!vb2_is_streaming(vq))
		return 0;

	/* release all active buffers */
	spin_lock_irqsave(&vpfe_dev->dma_queue_lock, flags);
	while (!list_empty(&vpfe_dev->dma_queue)) {
		vpfe_dev->next_frm = list_entry(vpfe_dev->dma_queue.next,
						struct vpfe_cap_buffer, list);
		list_del(&vpfe_dev->next_frm->list);
		vb2_buffer_done(&vpfe_dev->next_frm->vb, VB2_BUF_STATE_ERROR);
	}
	spin_unlock_irqrestore(&vpfe_dev->dma_queue_lock, flags);

	return 0;
}

static struct vb2_ops video_qops = {
	.queue_setup		= vpfe_buffer_queue_setup,
	.wait_prepare		= vpfe_wait_prepare,
	.wait_finish		= vpfe_wait_finish,
	.buf_init		= vpfe_buffer_init,
	.buf_prepare		= vpfe_buffer_prepare,
	.start_streaming	= vpfe_start_streaming,
	.stop_streaming		= vpfe_stop_streaming,
	.buf_cleanup		= vpfe_buffer_cleanup,
	.buf_queue		= vpfe_buffer_queue,
};

/*
 * vpfe_reqbufs() - request buffer handler
 * @file: file ptr
 * @priv: file handle
 * @reqbuf: request buffer structure ptr
 *
 * Currently support REQBUF only once opening
 * the device.
 */
static int vpfe_reqbufs(struct file *file, void *priv,
			struct v4l2_requestbuffers *req_buf)
{
	struct vpfe_device *vpfe_dev = video_drvdata(file);
	struct vpfe_fh *fh = file->private_data;
	struct vb2_queue *q;
	int ret = 0;

	dev_dbg(vpfe_dev->pdev, "vpfe_reqbufs\n");

	if (V4L2_BUF_TYPE_VIDEO_CAPTURE != req_buf->type) {
		dev_err(vpfe_dev->pdev, "Invalid buffer type\n");
		return -EINVAL;
	}

	ret = mutex_lock_interruptible(&vpfe_dev->lock);
	if (ret)
		return ret;

	if (vpfe_dev->io_usrs != 0) {
		dev_err(vpfe_dev->pdev, "Only one IO user allowed\n");
		ret = -EBUSY;
		goto unlock_out;
	}

	/* Initialize videobuf2 queue as per the buffer type */
	vpfe_dev->alloc_ctx = vb2_dma_contig_init_ctx(vpfe_dev->pdev);
	if (IS_ERR(vpfe_dev->alloc_ctx)) {
		dev_err(vpfe_dev->pdev, "Failed to get the context\n");
		ret = PTR_ERR(vpfe_dev->alloc_ctx);
		goto unlock_out;
	}
	q = &vpfe_dev->buffer_queue;
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_USERPTR;
	q->drv_priv = fh;
	q->ops = &video_qops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct vpfe_cap_buffer);
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;

	ret = vb2_queue_init(q);
	if (ret) {
		dev_err(vpfe_dev->pdev, "vb2_queue_init() failed\n");
		vb2_dma_contig_cleanup_ctx(vpfe_dev->alloc_ctx);
		goto unlock_out;
	}
	/* Set io allowed member of file handle to TRUE */
	fh->io_allowed = 1;
	/* Increment io usrs member of channel object to 1 */
	vpfe_dev->io_usrs = 1;
	/* Store type of memory requested in channel object */
	vpfe_dev->memory = req_buf->memory;
	INIT_LIST_HEAD(&vpfe_dev->dma_queue);

	/* Allocate buffers */
	ret = vb2_reqbufs(&vpfe_dev->buffer_queue, req_buf);
unlock_out:
	mutex_unlock(&vpfe_dev->lock);
	return ret;
}

/*
 * vpfe_querybuf() - query buffer handler
 * @file: file ptr
 * @priv: file handle
 * @buf: v4l2 buffer structure ptr
 */
static int vpfe_querybuf(struct file *file, void *priv,
			 struct v4l2_buffer *buf)
{
	struct vpfe_device *vpfe_dev = video_drvdata(file);

	dev_dbg(vpfe_dev->pdev, "vpfe_querybuf\n");

	if (V4L2_BUF_TYPE_VIDEO_CAPTURE != buf->type) {
		dev_err(vpfe_dev->pdev, "Invalid buf type\n");
		return  -EINVAL;
	}

	if (vpfe_dev->memory != V4L2_MEMORY_MMAP) {
		dev_err(vpfe_dev->pdev, "Invalid memory\n");
		return -EINVAL;
	}
	/* Call vb2_querybuf to get information */
	return vb2_querybuf(&vpfe_dev->buffer_queue, buf);
}

/*
 * vpfe_qbuf() - queue buffer handler
 * @file: file ptr
 * @priv: file handle
 * @buf: v4l2 buffer structure ptr
 */
static int vpfe_qbuf(struct file *file, void *priv,
		     struct v4l2_buffer *buf)
{
	struct vpfe_device *vpfe_dev = video_drvdata(file);
	struct vpfe_fh *fh = file->private_data;

	if (V4L2_BUF_TYPE_VIDEO_CAPTURE != buf->type) {
		dev_err(vpfe_dev->pdev, "Invalid buf type\n");
		return -EINVAL;
	}

	/*
	 * If this file handle is not allowed to do IO,
	 * return error
	 */
	if (!fh->io_allowed) {
		dev_err(vpfe_dev->pdev, "fh->io_allowed\n");
		return -EACCES;
	}
	return vb2_qbuf(&vpfe_dev->buffer_queue, buf);
}

/*
 * vpfe_dqbuf() - dequeue buffer handler
 * @file: file ptr
 * @priv: file handle
 * @buf: v4l2 buffer structure ptr
 */
static int vpfe_dqbuf(struct file *file, void *priv,
		      struct v4l2_buffer *buf)
{
	struct vpfe_device *vpfe_dev = video_drvdata(file);

	if (V4L2_BUF_TYPE_VIDEO_CAPTURE != buf->type) {
		dev_err(vpfe_dev->pdev, "Invalid buf type\n");
		return -EINVAL;
	}
	return vb2_dqbuf(&vpfe_dev->buffer_queue, buf,
			(file->f_flags & O_NONBLOCK));
}

/*
 * vpfe_calculate_offsets : This function calculates buffers offset
 * for top and bottom field
 */
static void vpfe_calculate_offsets(struct vpfe_device *vpfe_dev)
{
	struct v4l2_rect image_win;

	dev_dbg(vpfe_dev->pdev, "vpfe_calculate_offsets\n");

	vpfe_dev->vpfe_isif.hw_ops.get_image_window(
				&vpfe_dev->vpfe_isif, &image_win);
	vpfe_dev->field_off = image_win.height * image_win.width;
}

/* vpfe_start_isif_capture: start streaming in ccdc/isif */
static void vpfe_start_isif_capture(struct vpfe_device *vpfe_dev)
{
	vpfe_dev->vpfe_isif.hw_ops.enable(&vpfe_dev->vpfe_isif, 1);
	if (vpfe_dev->vpfe_isif.hw_ops.enable_out_to_sdram)
		vpfe_dev->vpfe_isif.hw_ops.enable_out_to_sdram(
				&vpfe_dev->vpfe_isif, 1);
	vpfe_dev->started = 1;
}

/*
 * vpfe_streamon() - streamon handler
 * @file: file ptr
 * @priv: file handle
 * @buftype: v4l2 buffer type

 * Assume the DMA queue is not empty.
 * application is expected to call QBUF before calling
 * this ioctl. If not, driver returns error
 */
static int vpfe_streamon(struct file *file, void *priv,
			 enum v4l2_buf_type buf_type)
{
	struct vpfe_device *vpfe_dev = video_drvdata(file);
	struct vpfe_fh *fh = file->private_data;
	struct vpfe_subdev_info *sdinfo;
	int ret = 0;

	dev_dbg(vpfe_dev->pdev, "vpfe_streamon\n");

	if (V4L2_BUF_TYPE_VIDEO_CAPTURE != buf_type) {
		dev_err(vpfe_dev->pdev, "Invalid buf type\n");
		return -EINVAL;
	}

	/* If file handle is not allowed IO, return error */
	if (!fh->io_allowed) {
		dev_err(vpfe_dev->pdev, "fh->io_allowed\n");
		return -EACCES;
	}

	/* If streaming is already started, return error */
	if (vpfe_dev->started) {
		dev_err(vpfe_dev->pdev, "device started\n");
		return -EBUSY;
	}

	sdinfo = vpfe_dev->current_subdev;
	ret = v4l2_device_call_until_err(&vpfe_dev->v4l2_dev, sdinfo->grp_id,
					video, s_stream, 1);

	if (ret && (ret != -ENOIOCTLCMD)) {
		dev_err(vpfe_dev->pdev, "stream on failed in subdev\n");
		return -EINVAL;
	}

	ret = mutex_lock_interruptible(&vpfe_dev->lock);
	if (ret)
		goto unlock_out;

	/* Call vb2_streamon to start streaming * in videobuf2 */
	ret = vb2_streamon(&vpfe_dev->buffer_queue, buf_type);
	if (ret) {
		dev_dbg(vpfe_dev->pdev, "vb2_streamon\n");
		goto unlock_out;
	}

unlock_out:
	mutex_unlock(&vpfe_dev->lock);
	return ret;
}

/*
 * vpfe_streamoff() - streamoff handler
 * @file: file ptr
 * @priv: file handle
 * @buftype: v4l2 buffer type
 */
static int vpfe_streamoff(struct file *file, void *priv,
			  enum v4l2_buf_type buf_type)
{
	struct vpfe_device *vpfe_dev = video_drvdata(file);
	struct vpfe_fh *fh = file->private_data;
	struct vpfe_subdev_info *sdinfo;
	int ret = 0;

	dev_dbg(vpfe_dev->pdev, "vpfe_streamoff\n");

	if (V4L2_BUF_TYPE_VIDEO_CAPTURE != buf_type) {
		dev_err(vpfe_dev->pdev, "Invalid buf type\n");
		return -EINVAL;
	}

	/* If io is allowed for this file handle, return error */
	if (!fh->io_allowed) {
		dev_err(vpfe_dev->pdev, "fh->io_allowed\n");
		return -EACCES;
	}

	/* If streaming is not started, return error */
	if (!vpfe_dev->started) {
		dev_err(vpfe_dev->pdev, "device started\n");
		return -EINVAL;
	}

	ret = mutex_lock_interruptible(&vpfe_dev->lock);
	if (ret)
		return ret;

	vpfe_stop_isif_capture(vpfe_dev);
	vpfe_detach_irq(vpfe_dev);

	sdinfo = vpfe_dev->current_subdev;
	ret = v4l2_device_call_until_err(&vpfe_dev->v4l2_dev, sdinfo->grp_id,
					video, s_stream, 0);

	if (ret && (ret != -ENOIOCTLCMD))
		dev_err(vpfe_dev->pdev, "stream off failed in subdev\n");
	ret = vb2_streamoff(&vpfe_dev->buffer_queue, buf_type);
	mutex_unlock(&vpfe_dev->lock);
	return ret;
}

static int vpfe_cropcap(struct file *file, void *priv,
			      struct v4l2_cropcap *crop)
{
	struct vpfe_device *vpfe_dev = video_drvdata(file);

	dev_dbg(vpfe_dev->pdev, "vpfe_cropcap\n");

	if (vpfe_dev->std_index >= ARRAY_SIZE(vpfe_standards))
		return -EINVAL;

	memset(crop, 0, sizeof(struct v4l2_cropcap));
	crop->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	crop->defrect.width = vpfe_standards[vpfe_dev->std_index].width;
	crop->bounds.width = crop->defrect.width;
	crop->defrect.height = vpfe_standards[vpfe_dev->std_index].height;
	crop->bounds.height = crop->defrect.height;
	crop->pixelaspect = vpfe_standards[vpfe_dev->std_index].pixelaspect;
	return 0;
}

static int vpfe_g_crop(struct file *file, void *priv,
			     struct v4l2_crop *crop)
{
	struct vpfe_device *vpfe_dev = video_drvdata(file);

	dev_dbg(vpfe_dev->pdev, "vpfe_g_crop\n");

	crop->c = vpfe_dev->crop;
	return 0;
}

static int vpfe_s_crop(struct file *file, void *priv,
			     const struct v4l2_crop *crop)
{
	struct vpfe_device *vpfe_dev = video_drvdata(file);
	struct v4l2_rect rect = crop->c;
	int ret = 0;

	dev_dbg(vpfe_dev->pdev, "vpfe_s_crop\n");

	if (vpfe_dev->started) {
		/* make sure streaming is not started */
		dev_err(vpfe_dev->pdev,
			"Cannot change crop when streaming is ON\n");
		return -EBUSY;
	}

	ret = mutex_lock_interruptible(&vpfe_dev->lock);
	if (ret)
		return ret;

	if (rect.top < 0 || rect.left < 0) {
		dev_err(vpfe_dev->pdev,
			"doesn't support negative values for top & left\n");
		ret = -EINVAL;
		goto unlock_out;
	}

	/* adjust the width to 16 pixel boundary */
	rect.width = ((rect.width + 15) & ~0xf);

	/* make sure parameters are valid */
	if ((rect.left + rect.width >
		vpfe_dev->std_info.active_pixels) ||
	    (rect.top + rect.height >
		vpfe_dev->std_info.active_lines)) {
		dev_err(vpfe_dev->pdev, "Error in S_CROP params\n");
		ret = -EINVAL;
		goto unlock_out;
	}
	vpfe_dev->vpfe_isif.hw_ops.set_image_window(
				&vpfe_dev->vpfe_isif, &rect, vpfe_dev->bpp);
	vpfe_dev->fmt.fmt.pix.width = rect.width;
	vpfe_dev->fmt.fmt.pix.height = rect.height;
	vpfe_dev->fmt.fmt.pix.bytesperline =
		vpfe_dev->vpfe_isif.hw_ops.get_line_length(
				&vpfe_dev->vpfe_isif);
	vpfe_dev->fmt.fmt.pix.sizeimage =
		vpfe_dev->fmt.fmt.pix.bytesperline *
		vpfe_dev->fmt.fmt.pix.height;
	vpfe_dev->crop = rect;
unlock_out:
	mutex_unlock(&vpfe_dev->lock);
	return ret;
}


static long vpfe_param_handler(struct file *file, void *priv,
		bool valid_prio, unsigned int cmd, void *param)
{
	struct vpfe_device *vpfe_dev = video_drvdata(file);
	int ret = 0;

	dev_dbg(vpfe_dev->pdev, "vpfe_param_handler\n");

	if (vpfe_dev->started) {
		/* only allowed if streaming is not started */
		dev_dbg(vpfe_dev->pdev,
			"device already started\n");
		return -EBUSY;
	}

	ret = mutex_lock_interruptible(&vpfe_dev->lock);
	if (ret)
		return ret;

	switch (cmd) {
	case VPFE_CMD_S_ISIF_RAW_PARAMS:
		dev_warn(vpfe_dev->pdev,
			 "VPFE_CMD_S_ISIF_RAW_PARAMS: experimental ioctl\n");
		if (vpfe_dev->vpfe_isif.hw_ops.set_params) {
			ret = vpfe_dev->vpfe_isif.hw_ops.set_params(
					&vpfe_dev->vpfe_isif, param);
			if (ret) {
				dev_dbg(vpfe_dev->pdev,
					"Error setting parameters in ISIF\n");
				goto unlock_out;
			}
			ret = vpfe_get_isif_image_format(vpfe_dev,
							 &vpfe_dev->fmt);
			if (ret < 0) {
				dev_dbg(vpfe_dev->pdev,
					"Invalid image format at ISIF\n");
				goto unlock_out;
			}
		} else {
			ret = -EINVAL;
			dev_dbg(vpfe_dev->pdev,
				"VPFE_CMD_S_ISIF_RAW_PARAMS not supported\n");
		}
		break;
	default:
		ret = -ENOTTY;
	}
unlock_out:
	mutex_unlock(&vpfe_dev->lock);
	return ret;
}


/* vpfe capture ioctl operations */
static const struct v4l2_ioctl_ops vpfe_ioctl_ops = {
	.vidioc_querycap	 = vpfe_querycap,
	.vidioc_g_fmt_vid_cap    = vpfe_g_fmt,
	.vidioc_enum_fmt_vid_cap = vpfe_enum_fmt,
	.vidioc_s_fmt_vid_cap    = vpfe_s_fmt,
	.vidioc_try_fmt_vid_cap  = vpfe_try_fmt,
	.vidioc_enum_framesizes  = vpfe_enum_size,
	.vidioc_enum_input	 = vpfe_enum_input,
	.vidioc_g_input		 = vpfe_g_input,
	.vidioc_s_input		 = vpfe_s_input,
	.vidioc_querystd	 = vpfe_querystd,
	.vidioc_s_std		 = vpfe_s_std,
	.vidioc_g_std		 = vpfe_g_std,
	.vidioc_reqbufs		 = vpfe_reqbufs,
	.vidioc_querybuf	 = vpfe_querybuf,
	.vidioc_qbuf		 = vpfe_qbuf,
	.vidioc_dqbuf		 = vpfe_dqbuf,
	.vidioc_streamon	 = vpfe_streamon,
	.vidioc_streamoff	 = vpfe_streamoff,
	.vidioc_cropcap		 = vpfe_cropcap,
	.vidioc_g_crop		 = vpfe_g_crop,
	.vidioc_s_crop		 = vpfe_s_crop,
	.vidioc_default		 = vpfe_param_handler,
};

static struct vpfe_device *vpfe_initialize(void)
{
	struct vpfe_device *vpfe_dev;

	/* Default number of buffers should be 3 */
	if ((numbuffers > 0) &&
	    (numbuffers < config_params.min_numbuffers))
		numbuffers = config_params.min_numbuffers;

	config_params.numbuffers = numbuffers;

	/* Allocate memory for device objects */
	vpfe_dev = kzalloc(sizeof(*vpfe_dev), GFP_KERNEL);

	return vpfe_dev;
}

static int vpfe_async_bound(struct v4l2_async_notifier *notifier,
			    struct v4l2_subdev *subdev,
			    struct v4l2_async_subdev *asd)
{
	int i, j;
	struct vpfe_subdev_info *sdinfo;
	struct vpfe_device *vpfe_dev = container_of(notifier->v4l2_dev,
					struct vpfe_device, v4l2_dev);

	dev_dbg(vpfe_dev->pdev, "vpfe_async_bound\n");

	for (i = 0; i < vpfe_dev->cfg->num_subdevs; i++) {
		sdinfo = &vpfe_dev->cfg->sub_devs[i];

		if (!strcmp(sdinfo->name, subdev->name)) {
			vpfe_dev->sd[i] = subdev;
			dev_info(vpfe_dev->pdev,
				 "v4l2 sub device %s registered\n",
				 subdev->name);
			vpfe_dev->sd[i]->grp_id =
					sdinfo->grp_id;
			/* update tvnorms from the sub devices */
			for (j = 0;
				 j < sdinfo->num_inputs;
				 j++) {
				vpfe_dev->video_dev->tvnorms |=
					sdinfo->inputs[j].std;
			}
			return 0;
		}
	}

	dev_warn(vpfe_dev->pdev, "vpfe_async_bound sub device (%s) not matched\n",
		 subdev->name);
	return -EINVAL;
}

static int vpfe_probe_complete(struct vpfe_device *vpfe_dev)
{
	int err;
	unsigned int flags;

	spin_lock_init(&vpfe_dev->irqlock);
	spin_lock_init(&vpfe_dev->dma_queue_lock);
	mutex_init(&vpfe_dev->lock);

	/* Initialize field of the device objects */
	vpfe_dev->numbuffers = config_params.numbuffers;

	/* Initialize prio member of device object */
	v4l2_prio_init(&vpfe_dev->prio);

	vpfe_dev->pad.flags = MEDIA_PAD_FL_SINK;
	err = media_entity_init(&vpfe_dev->video_dev->entity,
				1, &vpfe_dev->pad, 0);
	if (err < 0)
		return err;

	/* set video driver private data */
	video_set_drvdata(vpfe_dev->video_dev, vpfe_dev);
	/* register video device */
	dev_dbg(vpfe_dev->pdev, "trying to register vpfe device.\n");
	dev_dbg(vpfe_dev->pdev,
		"video_dev=%x\n", (int)&vpfe_dev->video_dev);
	vpfe_dev->fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	dev_info(vpfe_dev->pdev, "%s capture driver initialized\n",
		 vpfe_dev->cfg->card_name);

	/* set first sub device as current one */
	vpfe_dev->current_subdev = &vpfe_dev->cfg->sub_devs[0];
	vpfe_dev->v4l2_dev.ctrl_handler = vpfe_dev->sd[0]->ctrl_handler;

	/* select input 0 */
	err = vpfe_set_input(vpfe_dev, 0);
	if (err)
		goto probe_out;

	/* connect subdev to video node */
	dev_err(vpfe_dev->pdev, "sd[0]->entity is %x\n",
		(unsigned int)&vpfe_dev->sd[0]->entity);
	dev_err(vpfe_dev->pdev, "video_dev->entity is %x\n",
		(unsigned int)&vpfe_dev->video_dev->entity);

	dev_err(vpfe_dev->pdev, "source_pad:%d source->num_pads:%d\n",
		0, vpfe_dev->sd[0]->entity.num_pads);
	dev_err(vpfe_dev->pdev, "sink_pad:%d sink->num_pads:%d\n",
		0, vpfe_dev->video_dev->entity.num_pads);

	flags = MEDIA_LNK_FL_ENABLED;
	err = media_entity_create_link(&vpfe_dev->sd[0]->entity, 0,
				       &vpfe_dev->video_dev->entity,
				       0, flags);
	if (err < 0)
		goto probe_out;

	err = v4l2_device_register_subdev_nodes(&vpfe_dev->v4l2_dev);
	if (err < 0)
		goto probe_out;

	err = video_register_device(vpfe_dev->video_dev,
				    VFL_TYPE_GRABBER, -1);
	if (err) {
		dev_err(vpfe_dev->pdev,
			"Unable to register video device.\n");
		goto probe_out;
	}
	dev_info(vpfe_dev->pdev, "video device registered as %s\n",
		 video_device_node_name(vpfe_dev->video_dev));

	return 0;

probe_out:
	v4l2_device_unregister(&vpfe_dev->v4l2_dev);
	return err;
}

static int vpfe_async_complete(struct v4l2_async_notifier *notifier)
{
	struct vpfe_device *vpfe_dev = container_of(notifier->v4l2_dev,
					struct vpfe_device, v4l2_dev);
	return vpfe_probe_complete(vpfe_dev);
}

static struct vpfe_config *
vpfe_get_pdata(struct platform_device *pdev)
{
	struct vpfe_config *pdata;
	struct device_node *endpoint, *rem;
	struct vpfe_subdev_info *sdinfo;
	const char *instance_name;

	dev_dbg(&pdev->dev, "vpfe_get_pdata\n");

	if (!IS_ENABLED(CONFIG_OF) || !pdev->dev.of_node)
		return pdev->dev.platform_data;

	dev_dbg(&pdev->dev, "vpfe_get_pdata: DT Node found\n");

	dev_dbg(&pdev->dev, "dev.of_node->name: %s\n",
		pdev->dev.of_node->name);
	dev_dbg(&pdev->dev, "dev.id: %d\n",
		pdev->dev.id);
	/* the hwmods property basically provides us with the instance
	 * name and number. So use instead of creating some other means
	 */
	of_property_read_string(pdev->dev.of_node, "ti,hwmods", &instance_name);
	dev_dbg(&pdev->dev, "hwmods: %s\n", instance_name);

	endpoint = of_graph_get_next_endpoint(pdev->dev.of_node, NULL);
	if (!endpoint)
		return NULL;

	dev_dbg(&pdev->dev, "vpfe_get_pdata: endpoint found\n");

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		goto done;

	/* Populate vpfe_config with default if needed */
	pdata->card_name = (char *)instance_name;
	pdata->isif = "AM437x ISIF";

	/* We only support one sub_devive per port */
	/* Will need cleanup */
	pdata->num_subdevs = 1;
	sdinfo = &pdata->sub_devs[0];
	sdinfo->grp_id = 0;

	/* There is only one input on the camera */
	sdinfo->num_inputs = 1;
	sdinfo->inputs[0].index = 0;
	strcpy(sdinfo->inputs[0].name, "camera");
	sdinfo->inputs[0].type = V4L2_INPUT_TYPE_CAMERA;
	sdinfo->inputs[0].std = (V4L2_STD_NTSC | V4L2_STD_PAL);

	sdinfo->can_route = 0;
	sdinfo->routes = NULL;

	of_property_read_u32(endpoint, "if_type",
			     &sdinfo->isif_if_params.if_type);
	if (sdinfo->isif_if_params.if_type < 0)
		sdinfo->isif_if_params.if_type = VPFE_RAW_BAYER;
	of_property_read_u32(endpoint, "bus_width",
			     &sdinfo->isif_if_params.bus_width);
	if (sdinfo->isif_if_params.bus_width != 10)
		sdinfo->isif_if_params.bus_width = 8;
	of_property_read_u32(endpoint, "hdpol",
			     &sdinfo->isif_if_params.hdpol);
	if (sdinfo->isif_if_params.hdpol != 0)
		sdinfo->isif_if_params.hdpol = 1;
	of_property_read_u32(endpoint, "vdpol",
			     &sdinfo->isif_if_params.vdpol);
	if (sdinfo->isif_if_params.vdpol != 0)
		sdinfo->isif_if_params.vdpol = 1;

	rem = of_graph_get_remote_port_parent(endpoint);
	if (rem == NULL) {
		dev_err(&pdev->dev, "Remote device at %s not found\n",
			endpoint->full_name);
		goto done;
	} else {
		dev_dbg(&pdev->dev, "endpoint->full_name: %s\n",
			endpoint->full_name);
		dev_dbg(&pdev->dev, "remote->full_name: %s\n",
			rem->full_name);
		dev_dbg(&pdev->dev, "remote->name: %s\n",
			rem->name);
		strncpy(sdinfo->name, rem->name, sizeof(sdinfo->name));
	}

	sdinfo->asd.match_type = V4L2_ASYNC_MATCH_OF;
	sdinfo->asd.match.of.node = rem;
	pdata->asd[0] = &sdinfo->asd;
	pdata->asd_sizes = 1;
	of_node_put(rem);

done:
	of_node_put(endpoint);
	return pdata;
}

/*
 * vpfe_probe : This function creates device entries by register
 * itself to the V4L2 driver and initializes fields of each
 * device objects
 */
static int vpfe_probe(struct platform_device *pdev)
{
	struct vpfe_subdev_info *sdinfo;
	struct vpfe_config *vpfe_cfg = vpfe_get_pdata(pdev);
	struct vpfe_device *vpfe_dev;
	struct i2c_adapter *i2c_adap;
	struct video_device *vfd;
	int ret = -ENOMEM, i, j;
	int num_subdevs = 0;

	dev_dbg(&pdev->dev, "info vpfe_probe asynch\n");

	/* Get the pointer to the device object */
	vpfe_dev = vpfe_initialize();

	if (!vpfe_dev) {
		dev_err(vpfe_dev->pdev,
			"Failed to allocate memory for vpfe_dev\n");
		return ret;
	}

	vpfe_dev->pdev = &pdev->dev;

	if (vpfe_cfg == NULL) {
		dev_err(vpfe_dev->pdev, "Unable to get vpfe config\n");
		ret = -ENODEV;
		goto probe_free_dev_mem;
	}

	vpfe_dev->cfg = vpfe_cfg;
	if (NULL == vpfe_cfg->isif ||
	    NULL == vpfe_cfg->card_name ||
	    NULL == vpfe_cfg->sub_devs) {
		dev_err(vpfe_dev->pdev, "null ptr in vpfe_cfg\n");
		ret = -ENOENT;
		goto probe_free_dev_mem;
	}

	dev_dbg(vpfe_dev->pdev, "vpfe_cfg->isif: %s\n", vpfe_cfg->isif);

	/* Get VINT0 irq resource */
	vpfe_dev->isif_irq0 = platform_get_irq(pdev, 0);
	if (vpfe_dev->isif_irq0 < 0) {
		dev_err(vpfe_dev->pdev,
			"Unable to get interrupt for VINT0\n");
		ret = -ENODEV;
		goto probe_free_dev_mem;
	}

	dev_dbg(vpfe_dev->pdev, "Found ISIF IRQ: %d\n",
		vpfe_dev->isif_irq0);

	ret = request_irq(vpfe_dev->isif_irq0, vpfe_isr, IRQF_DISABLED,
			  "vpfe_capture0", vpfe_dev);
	if (0 != ret) {
		dev_err(vpfe_dev->pdev, "Unable to request interrupt\n");
		goto probe_free_dev_mem;
	}

	/* Allocate memory for video device */
	vfd = video_device_alloc();
	if (NULL == vfd) {
		ret = -ENOMEM;
		dev_err(vpfe_dev->pdev, "Unable to alloc video device\n");
		goto probe_out_release_irq;
	}

	/* Initialize field of video device */
	vfd->release		= video_device_release;
	vfd->fops		= &vpfe_fops;
	vfd->ioctl_ops		= &vpfe_ioctl_ops;
	vfd->tvnorms		= 0;
	vfd->v4l2_dev		= &vpfe_dev->v4l2_dev;
	snprintf(vfd->name, sizeof(vfd->name),
		 "%s_V%d.%d.%d",
		 CAPTURE_DRV_NAME,
		 (VPFE_CAPTURE_VERSION_CODE >> 16) & 0xff,
		 (VPFE_CAPTURE_VERSION_CODE >> 8) & 0xff,
		 (VPFE_CAPTURE_VERSION_CODE) & 0xff);
	/* Set video_dev to the video device */
	vpfe_dev->video_dev	= vfd;

	vpfe_dev->media_dev.dev = vpfe_dev->pdev;
	strcpy((char *)&vpfe_dev->media_dev.model, "ti-vpfe-media");

	ret = media_device_register(&vpfe_dev->media_dev);
	if (ret) {
		dev_err(vpfe_dev->pdev,
			"Unable to register media device.\n");
		goto probe_out_video_release;
	}

	vpfe_dev->v4l2_dev.mdev = &vpfe_dev->media_dev;

	ret = v4l2_device_register(&pdev->dev, &vpfe_dev->v4l2_dev);
	if (ret) {
		dev_err(vpfe_dev->pdev,
			"Unable to register v4l2 device.\n");
		goto probe_out_media_unregister;
	}
	dev_info(vpfe_dev->pdev, "v4l2 device registered\n");

	/* set the driver data in platform device */
	platform_set_drvdata(pdev, vpfe_dev);

	/* We have at least one sub device to work with */
	ret = vpfe_isif_init(&vpfe_dev->vpfe_isif, pdev);
	if (ret) {
		dev_err(vpfe_dev->pdev, "Error initializing isif\n");
		ret = -EINVAL;
		goto probe_out_v4l2_unregister;
	}

	num_subdevs = vpfe_cfg->num_subdevs;
	vpfe_dev->sd = kmalloc(sizeof(struct v4l2_subdev *) * num_subdevs,
				GFP_KERNEL);
	if (NULL == vpfe_dev->sd) {
		dev_err(vpfe_dev->pdev,
			"unable to allocate memory for subdevice pointers\n");
		ret = -ENOMEM;
		goto probe_out_v4l2_unregister;
	}

	dev_dbg(vpfe_dev->pdev, "vpfe_dev->cfg->asd_sizes = %x\n",
		vpfe_dev->cfg->asd_sizes);

	if (vpfe_dev->cfg->asd_sizes == 0) {
		dev_info(vpfe_dev->pdev, "Synchronous subdevice registration\n");

		i2c_adap = i2c_get_adapter(vpfe_cfg->i2c_adapter_id);

		for (i = 0; i < num_subdevs; i++) {
			struct v4l2_input *inps;

			sdinfo = &vpfe_cfg->sub_devs[i];

			/* Load up the subdevice */
			vpfe_dev->sd[i] =
				v4l2_i2c_new_subdev_board(&vpfe_dev->v4l2_dev,
							  i2c_adap,
							  &sdinfo->board_info,
							  NULL);
			if (vpfe_dev->sd[i]) {
				dev_info(vpfe_dev->pdev,
					 "v4l2 sub device %s registered\n",
					 sdinfo->name);
				vpfe_dev->sd[i]->grp_id = sdinfo->grp_id;
				/* update tvnorms from the sub devices */
				for (j = 0; j < sdinfo->num_inputs; j++) {
					inps = &sdinfo->inputs[j];
					vfd->tvnorms |= inps->std;
				}
			} else {
				dev_err(vpfe_dev->pdev,
					"v4l2 sub device %s register fails\n",
					sdinfo->name);
				goto probe_sd_out;
			}
		}
		vpfe_probe_complete(vpfe_dev);

	} else {
		dev_info(vpfe_dev->pdev, "Asynchronous subdevice registration\n");

		vpfe_dev->notifier.subdevs = vpfe_dev->cfg->asd;
		vpfe_dev->notifier.num_subdevs = vpfe_dev->cfg->asd_sizes;
		vpfe_dev->notifier.bound = vpfe_async_bound;
		vpfe_dev->notifier.complete = vpfe_async_complete;
		ret = v4l2_async_notifier_register(&vpfe_dev->v4l2_dev,
						   &vpfe_dev->notifier);
		if (ret) {
			dev_err(vpfe_dev->pdev, "Error registering async notifier\n");
			ret = -EINVAL;
			goto probe_sd_out;
		}
	}

	return 0;

probe_sd_out:
	kfree(vpfe_dev->sd);
probe_out_v4l2_unregister:
	v4l2_device_unregister(&vpfe_dev->v4l2_dev);
probe_out_media_unregister:
	media_device_unregister(&vpfe_dev->media_dev);
probe_out_video_release:
	if (!video_is_registered(vpfe_dev->video_dev))
		video_device_release(vpfe_dev->video_dev);
probe_out_release_irq:
	free_irq(vpfe_dev->isif_irq0, vpfe_dev);
probe_free_dev_mem:
	kfree(vpfe_dev);
	return ret;
}

/*
 * vpfe_remove : It un-register device from V4L2 driver
 */
static int vpfe_remove(struct platform_device *pdev)
{
	struct vpfe_device *vpfe_dev = platform_get_drvdata(pdev);

	dev_dbg(vpfe_dev->pdev, "vpfe_remove\n");

	isif_remove(&vpfe_dev->vpfe_isif, pdev);

	free_irq(vpfe_dev->isif_irq0, vpfe_dev);
	v4l2_async_notifier_unregister(&vpfe_dev->notifier);
	kfree(vpfe_dev->sd);
	v4l2_device_unregister(&vpfe_dev->v4l2_dev);
	media_device_unregister(&vpfe_dev->media_dev);
	video_unregister_device(vpfe_dev->video_dev);
	kfree(vpfe_dev);
	return 0;
}

static int vpfe_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct vpfe_device *vpfe_dev = platform_get_drvdata(pdev);
	isif_suspend(&vpfe_dev->vpfe_isif, dev);

	/* Select sleep pin state */
	pinctrl_pm_select_sleep_state(dev);

	return 0;
}

static int vpfe_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct vpfe_device *vpfe_dev = platform_get_drvdata(pdev);
	isif_resume(&vpfe_dev->vpfe_isif, dev);

	/* Select default pin state */
	pinctrl_pm_select_default_state(dev);

	return 0;
}

static const struct dev_pm_ops vpfe_dev_pm_ops = {
	.suspend = vpfe_suspend,
	.resume = vpfe_resume,
};

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id vpfe_of_match[] = {
	{ .compatible = "ti,am437x-vpfe", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, vpfe_of_match);
#endif

static struct platform_driver vpfe_driver = {
	.driver = {
		.name = CAPTURE_DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &vpfe_dev_pm_ops,
		.of_match_table = of_match_ptr(vpfe_of_match),
	},
	.probe = vpfe_probe,
	.remove = vpfe_remove,
};

module_platform_driver(vpfe_driver);
