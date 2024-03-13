// SPDX-License-Identifier: GPL-2.0
/*
 * Imagination E5010 JPEG Encoder driver.
 *
 * Copyright (c) 2023 Texas Instruments Inc.
 * Author: David Huang <d-huang@ti.com>
 * Author: Devarsh Thakkar <devarsht@ti.com>
 */

#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/printk.h>
#include <linux/ioctl.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/pm_runtime.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>
#include "e5010-jpeg-enc.h"
#include "e5010-jpeg-enc-hw.h"

static const struct of_device_id e5010_of_match[];

static const struct v4l2_file_operations e5010_fops;

static const struct v4l2_ioctl_ops e5010_ioctl_ops;

static const struct vb2_ops e5010_video_ops;

static const struct v4l2_m2m_ops e5010_m2m_ops;

static struct e5010_fmt e5010_formats[] = {
	{
		.fourcc = V4L2_PIX_FMT_NV12,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.subsampling = V4L2_JPEG_CHROMA_SUBSAMPLING_420,
		.chroma_order = CHROMA_ORDER_CB_CR,
		.h_align = 6,
		.v_align = 3,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV12M,
		.num_planes = 2,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.subsampling = V4L2_JPEG_CHROMA_SUBSAMPLING_420,
		.chroma_order = CHROMA_ORDER_CB_CR,
		.h_align = 6,
		.v_align = 3,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV21,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.subsampling = V4L2_JPEG_CHROMA_SUBSAMPLING_420,
		.chroma_order = CHROMA_ORDER_CR_CB,
		.h_align = 6,
		.v_align = 3,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV21M,
		.num_planes = 2,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.subsampling = V4L2_JPEG_CHROMA_SUBSAMPLING_420,
		.chroma_order = CHROMA_ORDER_CR_CB,
		.h_align = 6,
		.v_align = 3,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV16,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.subsampling = V4L2_JPEG_CHROMA_SUBSAMPLING_422,
		.chroma_order = CHROMA_ORDER_CB_CR,
		.h_align = 6,
		.v_align = 3,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV16M,
		.num_planes = 2,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.subsampling = V4L2_JPEG_CHROMA_SUBSAMPLING_422,
		.chroma_order = CHROMA_ORDER_CB_CR,
		.h_align = 6,
		.v_align = 3,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV61,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.subsampling = V4L2_JPEG_CHROMA_SUBSAMPLING_422,
		.chroma_order = CHROMA_ORDER_CR_CB,
		.h_align = 6,
		.v_align = 3,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV61M,
		.num_planes = 2,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.subsampling = V4L2_JPEG_CHROMA_SUBSAMPLING_422,
		.chroma_order = CHROMA_ORDER_CR_CB,
		.h_align = 6,
		.v_align = 3,
	},
	{
		.fourcc = V4L2_PIX_FMT_JPEG,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.subsampling = 0,
		.chroma_order = 0,
		.h_align = 0,
		.v_align = 0,
	},
};

/* Luma and chroma qp table to acheive 50% compression quality
 * This is as per example in Annex K.1 of IS0/IEC 10918-1:1994(E)
 */
static const u8 luma[64] = {
	16, 11, 10, 16, 24, 40, 51, 61,
	12, 12, 14, 19, 26, 58, 60, 55,
	14, 13, 16, 24, 40, 57, 69, 56,
	14, 17, 22, 29, 51, 87, 80, 62,
	18, 22, 37, 56, 68, 109, 103, 77,
	24, 35, 55, 64, 81, 104, 113, 92,
	49, 64, 78, 87, 103, 121, 120, 101,
	72, 92, 95, 98, 112, 100, 103, 99
};

static const u8 chroma[64] = {
	17, 18, 24, 47, 99, 99, 99, 99,
	18, 21, 26, 66, 99, 99, 99, 99,
	24, 26, 56, 99, 99, 99, 99, 99,
	47, 66, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99,
	99, 99, 99, 99, 99, 99, 99, 99
};

/* Zigzag scan pattern */
static const u8 zigzag[64] = {
	0,   1,  8, 16,  9,  2,  3, 10,
	17, 24, 32, 25, 18, 11,  4,  5,
	12, 19, 26, 33, 40, 48, 41, 34,
	27, 20, 13,  6,  7, 14, 21, 28,
	35, 42, 49, 56, 57, 50, 43, 36,
	29, 22, 15, 23, 30, 37, 44, 51,
	58, 59, 52, 45, 38, 31, 39, 46,
	53, 60, 61, 54, 47, 55, 62, 63
};

/*
 * Contains the data that needs to be sent in the marker segment of an interchange format JPEG
 * stream or an abbreviated format table specification data stream.
 * Specifies the huffman table used for encoding the luminance DC coefficient differences.
 * The table represents Table K.3 of IS0/IEC 10918-1:1994(E)
 */
static const u8 marker_luma_dc[] = {
	0x00, 0x00, 0x01, 0x05, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B
};

/*
 * Contains the data that needs to be sent in the marker segment of an interchange format JPEG
 * stream or an abbreviated format table specification data stream.
 * Specifies the huffman table used for encoding the luminance AC coefficients.
 * The table represents Table K.5 of IS0/IEC 10918-1:1994(E)
 */
static const u8 marker_luma_ac[] = {
	0x10, 0x00, 0x02, 0x01, 0x03, 0x03, 0x02, 0x04, 0x03,
	0x05, 0x05, 0x04, 0x04, 0x00, 0x00, 0x01, 0x7D,
	0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61,
	0x07, 0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xA1, 0x08, 0x23, 0x42, 0xB1, 0xC1, 0x15, 0x52,
	0xD1, 0xF0, 0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0A, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x25,
	0x26, 0x27, 0x28, 0x29, 0x2A, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x43, 0x44, 0x45,
	0x46, 0x47, 0x48, 0x49, 0x4A, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x63, 0x64,
	0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A, 0x83,
	0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99,
	0x9A, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6,
	0xB7, 0xB8, 0xB9, 0xBA, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xD2, 0xD3,
	0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8,
	0xE9, 0xEA, 0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8, 0xF9, 0xFA
};

/*
 * Contains the data that needs to be sent in the marker segment of an interchange format JPEG
 * stream or an abbreviated format table specification data stream.
 * Specifies the huffman table used for encoding the chrominance DC coefficient differences.
 * The table represents Table K.4 of IS0/IEC 10918-1:1994(E)
 */
static const u8 marker_chroma_dc[] = {
	0x01, 0x00, 0x03, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
	0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B
};

/*
 * Contains the data that needs to be sent in the marker segment of an interchange format JPEG
 * stream or an abbreviated format table specification data stream.
 * Specifies the huffman table used for encoding the chrominance AC coefficients.
 * The table represents Table K.6 of IS0/IEC 10918-1:1994(E)
 */
static const u8 marker_chroma_ac[] = {
	0x11, 0x00, 0x02, 0x01, 0x02, 0x04, 0x04, 0x03, 0x04,
	0x07, 0x05, 0x04, 0x04, 0x00, 0x01, 0x02, 0x77,
	0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21, 0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61,
	0x71, 0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91, 0xA1, 0xB1, 0xC1, 0x09, 0x23, 0x33,
	0x52, 0xF0, 0x15, 0x62, 0x72, 0xD1, 0x0A, 0x16, 0x24, 0x34, 0xE1, 0x25, 0xF1, 0x17, 0x18,
	0x19, 0x1A, 0x26, 0x27, 0x28, 0x29, 0x2A, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x43, 0x44,
	0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x63,
	0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A,
	0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97,
	0x98, 0x99, 0x9A, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xB2, 0xB3, 0xB4,
	0xB5, 0xB6, 0xB7, 0xB8, 0xB9, 0xBA, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA,
	0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7,
	0xE8, 0xE9, 0xEA, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8, 0xF9, 0xFA
};

static unsigned int debug;
module_param(debug, uint, 0644);
MODULE_PARM_DESC(debug, "debug level");

#define dprintk(dev, lvl, fmt, arg...) \
	v4l2_dbg(lvl, debug, &(dev)->v4l2_dev, "%s: " fmt, __func__, ## arg)

static const struct v4l2_event e5010_eos_event = {
	.type = V4L2_EVENT_EOS
};

static const char *type_name(enum v4l2_buf_type type)
{
	switch (type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
		return "Output";
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		return "Capture";
	default:
		return "Invalid";
	}
}

static struct e5010_q_data *get_queue(struct e5010_context *ctx, enum v4l2_buf_type type)
{
	switch (type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
		return &ctx->out_queue;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		return &ctx->cap_queue;
	default:
		return ERR_PTR(-EINVAL);
	}

	return ERR_PTR(-EINVAL);
}

static void calculate_qp_tables(struct e5010_context *ctx)
{
	long long luminosity, contrast;
	int quality, i;

	quality = 100 - ctx->quality;
	quality -= 50;

	luminosity = LUMINOSITY * quality / 50;
	contrast = CONTRAST * quality / 50;

	if (quality > 0) {
		luminosity *= INCREASE;
		contrast *= INCREASE;
	}

	for (i = 0; i < 64; i++) {
		long long delta = chroma[i] * contrast + luminosity;
		int val = (int)(chroma[i] + delta);

		if (val < 1)
			val = 1;
		if (val > 255)
			val = 255;
		ctx->chroma_qp[i] = quality == -50 ? 1 : val;

		delta = luma[i] * contrast + luminosity;
		val = (int)(luma[i] + delta);
		if (val < 1)
			val = 1;
		if (val > 255)
			val = 255;
		ctx->luma_qp[i] = quality == -50 ? 1 : val;
	}

	ctx->update_qp = true;
}

static int update_qp_tables(struct e5010_context *ctx)
{
	struct e5010_dev *dev = ctx->dev;
	int i, ret = 0;
	u32 lvalue, cvalue;

	lvalue = 0;
	cvalue = 0;

	for (i = 0; i < (QP_TABLE_SIZE); i++) {
		lvalue |= ctx->luma_qp[i] << (8 * (i % 4));
		cvalue |= ctx->chroma_qp[i] << (8 * (i % 4));
		if (i % 4 == 3) {
			ret |= e5010_hw_set_qpvalue(dev->jasper_base,
							JASPER_LUMA_QUANTIZATION_TABLE0_OFFSET
							+ QP_TABLE_FIELD_OFFSET * ((i - 3) / 4),
							lvalue);
			ret |= e5010_hw_set_qpvalue(dev->jasper_base,
							JASPER_CHROMA_QUANTIZATION_TABLE0_OFFSET
							+ QP_TABLE_FIELD_OFFSET * ((i - 3) / 4),
							cvalue);
			lvalue = 0;
			cvalue = 0;
		}
	}

	return ret;
}

static int e5010_set_input_subsampling(void __iomem *core_base, int subsampling)
{
	switch (subsampling) {
	case V4L2_JPEG_CHROMA_SUBSAMPLING_420:
		return e5010_hw_set_input_subsampling(core_base, SUBSAMPLING_420);
	case V4L2_JPEG_CHROMA_SUBSAMPLING_422:
		return e5010_hw_set_input_subsampling(core_base, SUBSAMPLING_422);
	default:
		return -EINVAL;
	};
}

static int e5010_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	strncpy(cap->driver, E5010_MODULE_NAME, sizeof(cap->driver));
	strncpy(cap->card, E5010_MODULE_NAME, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s", E5010_MODULE_NAME);
	cap->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static struct e5010_fmt *find_format(struct v4l2_format *f)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(e5010_formats); ++i) {
		if (e5010_formats[i].fourcc == f->fmt.pix_mp.pixelformat &&
		    e5010_formats[i].type == f->type)
			return &e5010_formats[i];
	}

	return NULL;
}

static int e5010_enum_fmt(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	int i, index = 0;
	struct e5010_fmt *fmt = NULL;
	struct e5010_context *ctx = file->private_data;

	if (!V4L2_TYPE_IS_MULTIPLANAR(f->type)) {
		dev_err(ctx->dev->dev, "ENUMFMT with Invalid type: %d\n", f->type);
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(e5010_formats); ++i) {
		if (e5010_formats[i].type == f->type) {
			if (index == f->index) {
				fmt = &e5010_formats[i];
				break;
			}
			index++;
		}
	}

	if (!fmt)
		return -EINVAL;

	f->pixelformat = fmt->fourcc;
	return 0;
}

static int e5010_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct e5010_context *ctx = file->private_data;
	struct e5010_q_data *queue;
	int i;
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	struct v4l2_plane_pix_format *plane_fmt = pix_mp->plane_fmt;

	if (!V4L2_TYPE_IS_MULTIPLANAR(f->type)) {
		dev_err(ctx->dev->dev, "G_FMT with Invalid type: %d\n", f->type);
		return -EINVAL;
	}

	queue = get_queue(ctx, f->type);
	if (IS_ERR(queue))
		return PTR_ERR(queue);

	pix_mp->flags = 0;
	pix_mp->field = V4L2_FIELD_NONE;
	pix_mp->pixelformat = queue->fmt->fourcc;
	pix_mp->width = queue->width_adjusted;
	pix_mp->height = queue->height_adjusted;
	pix_mp->num_planes = queue->fmt->num_planes;

	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		if (!pix_mp->colorspace)
			pix_mp->colorspace = V4L2_COLORSPACE_SRGB;

		for (i = 0; i < queue->fmt->num_planes; i++) {
			plane_fmt[i].sizeimage = queue->sizeimage[i];
			plane_fmt[i].bytesperline = queue->bytesperline[i];
		}

	} else {
		pix_mp->colorspace = V4L2_COLORSPACE_JPEG;
		plane_fmt[0].bytesperline = 0;
		plane_fmt[0].sizeimage = queue->sizeimage[0];
	}
	pix_mp->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	pix_mp->xfer_func = V4L2_XFER_FUNC_DEFAULT;
	pix_mp->quantization = V4L2_QUANTIZATION_DEFAULT;

	return 0;
}

static void e5010_queue_update_bytesperline(struct e5010_q_data *q)
{
	if (q->fmt->fourcc == V4L2_PIX_FMT_JPEG) {
		/* bytesperline unused for compressed formats */
		q->bytesperline[0] = 0;
		q->bytesperline[1] = 0;
	} else if (q->fmt->num_planes == 1) {
		q->bytesperline[0] = q->width_adjusted;
		q->bytesperline[1] = 0;
	} else {
		q->bytesperline[0] = q->width_adjusted;
		q->bytesperline[1] = q->bytesperline[0];
	}
}

static void e5010_queue_update_sizeimage(struct e5010_q_data *q, struct e5010_context *ctx)
{
	if (q->fmt->fourcc == V4L2_PIX_FMT_JPEG) {
		if (ctx->out_queue.fmt->subsampling == V4L2_JPEG_CHROMA_SUBSAMPLING_420)
			q->sizeimage[0] = q->width_adjusted * q->height_adjusted * 3 / 2;
		else
			q->sizeimage[0] = q->width_adjusted * q->height_adjusted * 2;
		q->sizeimage[0] += HEADER_SIZE;
		q->sizeimage[1] = 0;
	} else if (q->fmt->subsampling == V4L2_JPEG_CHROMA_SUBSAMPLING_420) {
		if (q->fmt->num_planes == 1)	{
			q->sizeimage[0] = q->width_adjusted * q->height_adjusted * 3 / 2;
			q->sizeimage[1] = 0;
		} else {
			q->sizeimage[0] = q->width_adjusted * q->height_adjusted;
			q->sizeimage[1] = q->sizeimage[0] / 2;
		}
	} else {
		if (q->fmt->num_planes == 1)	{
			q->sizeimage[0] = q->width_adjusted * q->height_adjusted * 2;
			q->sizeimage[1] = 0;
		} else {
			q->sizeimage[0] = q->width_adjusted * q->height_adjusted;
			q->sizeimage[1] = q->sizeimage[0];
		}
	}
}

static int e5010_jpeg_try_fmt(struct v4l2_format *f, struct e5010_context *ctx)
{
	struct e5010_fmt *fmt;
	struct e5010_q_data *queue;
	int i;
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	struct v4l2_plane_pix_format *plane_fmt = pix_mp->plane_fmt;

	if (!V4L2_TYPE_IS_MULTIPLANAR(f->type)) {
		dev_err(ctx->dev->dev, "G_FMT with Invalid type: %d\n", f->type);
		return -EINVAL;
	}

	fmt = find_format(f);
	if (!fmt) {
		if (V4L2_TYPE_IS_OUTPUT(f->type))
			pix_mp->pixelformat = V4L2_PIX_FMT_NV12;
		else
			pix_mp->pixelformat = V4L2_PIX_FMT_JPEG;
		fmt = find_format(f);
		if (!fmt)
			return -EINVAL;
	}

	queue = get_queue(ctx, f->type);
	if (IS_ERR(queue))
		return PTR_ERR(queue);

	queue->fmt = fmt;
	queue->width = pix_mp->width;
	queue->height = pix_mp->height;

	queue->width_adjusted = queue->width;
	queue->height_adjusted = queue->height;

	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		if (!pix_mp->colorspace)
			pix_mp->colorspace = V4L2_COLORSPACE_JPEG;
		if (!pix_mp->ycbcr_enc)
			pix_mp->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
		if (!pix_mp->quantization)
			pix_mp->quantization = V4L2_QUANTIZATION_DEFAULT;
		if (!pix_mp->xfer_func)
			pix_mp->xfer_func = V4L2_XFER_FUNC_DEFAULT;

		v4l_bound_align_image(&queue->width_adjusted,
				      MIN_DIMENSION,
				      MAX_DIMENSION,
				      fmt->h_align,
				      &queue->height_adjusted,
				      MIN_DIMENSION, /* adjust upwards*/
				      MAX_DIMENSION,
				      fmt->v_align,
				      0);
		e5010_queue_update_bytesperline(queue);
		e5010_queue_update_sizeimage(queue, ctx);
		for (i = 0; i < fmt->num_planes; i++) {
			memset(plane_fmt[i].reserved, 0, sizeof(plane_fmt[i].reserved));
			plane_fmt[i].bytesperline = queue->bytesperline[i];
			plane_fmt[i].sizeimage = queue->sizeimage[i];
		}
	} else {
		pix_mp->colorspace = V4L2_COLORSPACE_JPEG;
		pix_mp->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
		pix_mp->quantization = V4L2_QUANTIZATION_DEFAULT;
		pix_mp->xfer_func = V4L2_XFER_FUNC_DEFAULT;

		plane_fmt[0].bytesperline = 0;

		memset(plane_fmt[0].reserved, 0, sizeof(plane_fmt[0].reserved));
		v4l_bound_align_image(&queue->width_adjusted,
				      MIN_DIMENSION,
				      MAX_DIMENSION,
				      4,
				      &queue->height_adjusted,
				      MIN_DIMENSION, /* adjust upwards*/
				      MAX_DIMENSION,
				      ctx->out_queue.fmt->v_align,
				      0);
		e5010_queue_update_bytesperline(queue);
		e5010_queue_update_sizeimage(queue, ctx);
		plane_fmt[0].sizeimage = queue->sizeimage[0];
	}
	pix_mp->flags = 0;
	pix_mp->field = V4L2_FIELD_NONE;
	pix_mp->pixelformat = fmt->fourcc;
	pix_mp->width = queue->width_adjusted;
	pix_mp->height = queue->height_adjusted;
	pix_mp->num_planes = fmt->num_planes;
	memset(pix_mp->reserved, 0, sizeof(pix_mp->reserved));

	dprintk(ctx->dev, 2,
		"ctx: 0x%p: format type %s:, wxh: %dx%d (plane0 : %d bytes, plane1 : %d bytes),fmt: %c%c%c%c\n",
		ctx, type_name(f->type), queue->width_adjusted, queue->height_adjusted,
		queue->sizeimage[0], queue->sizeimage[1],
		(queue->fmt->fourcc & 0xff),
		(queue->fmt->fourcc >>  8) & 0xff,
		(queue->fmt->fourcc >> 16) & 0xff,
		(queue->fmt->fourcc >> 24) & 0xff);

	return 0;
}

static int e5010_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct e5010_context *ctx = file->private_data;

	return e5010_jpeg_try_fmt(f, ctx);
}

static int e5010_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct e5010_context *ctx = file->private_data;
	struct vb2_queue *vq;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	if (vb2_is_busy(vq)) {
		v4l2_err(&ctx->dev->v4l2_dev, "queue busy\n");
		return -EBUSY;
	}

	return e5010_jpeg_try_fmt(f, ctx);
}

static int e5010_enum_framesizes(struct file *file, void *priv, struct v4l2_frmsizeenum *fsize)
{
	struct v4l2_format f;

	if (fsize->index != 0)
		return -EINVAL;

	f.fmt.pix_mp.pixelformat = fsize->pixel_format;
	if (f.fmt.pix_mp.pixelformat ==  V4L2_PIX_FMT_JPEG)
		f.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	else
		f.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;

	if (!find_format(&f))
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_CONTINUOUS;
	fsize->stepwise.min_width = MIN_DIMENSION;
	fsize->stepwise.max_width = MAX_DIMENSION;
	fsize->stepwise.step_width = 1;
	fsize->stepwise.min_height = MIN_DIMENSION;
	fsize->stepwise.max_height = MAX_DIMENSION;
	fsize->stepwise.step_height = 1;

	fsize->reserved[0] = 0;
	fsize->reserved[1] = 0;

	return 0;
}

static int e5010_g_selection(struct file *file, void *fh, struct v4l2_selection *s)
{
	struct e5010_context *ctx = file->private_data;
	struct e5010_q_data *queue;

	if (s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return -EINVAL;

	queue = get_queue(ctx, s->type);
	if (IS_ERR(queue))
		return PTR_ERR(queue);

	switch (s->target) {
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		s->r.left = 0;
		s->r.top = 0;
		s->r.width = queue->width;
		s->r.height = queue->height;
		break;
	case V4L2_SEL_TGT_CROP:
		s->r = queue->crop;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int e5010_s_selection(struct file *file, void *fh, struct v4l2_selection *s)
{
	struct e5010_context *ctx = file->private_data;
	struct e5010_q_data *queue;

	if (s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return -EINVAL;

	queue = get_queue(ctx, s->type);
	if (IS_ERR(queue))
		return PTR_ERR(queue);

	queue->crop.left = 0;
	queue->crop.top = 0;
	queue->crop.width = s->r.width;
	queue->crop.height = s->r.height;

	return 0;
}

static int e5010_subscribe_event(struct v4l2_fh *fh, const struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_EOS:
		return v4l2_event_subscribe(fh, sub, 0, NULL);
	case V4L2_EVENT_CTRL:
		return v4l2_ctrl_subscribe_event(fh, sub);
	default:
		return -EINVAL;
	}

	return 0;
}

static int queue_init(void *priv, struct vb2_queue *src_vq, struct vb2_queue *dst_vq)
{
	struct e5010_context *ctx = priv;
	struct e5010_dev *dev = ctx->dev;
	int ret = 0;

	/* src_vq */
	memset(src_vq, 0, sizeof(*src_vq));
	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->dma_attrs = DMA_ATTR_FORCE_CONTIGUOUS;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct e5010_buffer);
	src_vq->ops = &e5010_video_ops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &dev->mutex;
	src_vq->dev = dev->v4l2_dev.dev;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	/* dst_vq */
	memset(dst_vq, 0, sizeof(*dst_vq));
	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	src_vq->dma_attrs = DMA_ATTR_FORCE_CONTIGUOUS;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct e5010_buffer);
	dst_vq->ops = &e5010_video_ops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &dev->mutex;
	dst_vq->dev = dev->v4l2_dev.dev;

	ret = vb2_queue_init(dst_vq);
	if (ret) {
		vb2_queue_release(src_vq);
		return ret;
	}

	return 0;
}

static int e5010_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct e5010_context *ctx =
		container_of(ctrl->handler, struct e5010_context, ctrl_handler);

	switch (ctrl->id) {
	case V4L2_CID_JPEG_COMPRESSION_QUALITY:
		ctx->quality = ctrl->val;
		calculate_qp_tables(ctx);
		break;
	default:
		dev_err(ctx->dev->dev, "Invalid control, id = %d, val = %d\n",
			ctrl->id, ctrl->val);
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_ctrl_ops e5010_ctrl_ops = {
	.s_ctrl = e5010_s_ctrl,
};

static void e5010_encode_ctrls(struct e5010_context *ctx)
{
	v4l2_ctrl_new_std(&ctx->ctrl_handler, &e5010_ctrl_ops,
			  V4L2_CID_JPEG_COMPRESSION_QUALITY, 1, 100, 1, 75);
}

static int e5010_ctrls_setup(struct e5010_context *ctx)
{
	int err;

	v4l2_ctrl_handler_init(&ctx->ctrl_handler, 1);

	e5010_encode_ctrls(ctx);

	if (ctx->ctrl_handler.error) {
		err = ctx->ctrl_handler.error;
		v4l2_ctrl_handler_free(&ctx->ctrl_handler);
		return err;
	}

	err = v4l2_ctrl_handler_setup(&ctx->ctrl_handler);
	if (err)
		v4l2_ctrl_handler_free(&ctx->ctrl_handler);

	return err;
}

static void e5010_jpeg_set_default_params(struct e5010_context *ctx)
{
	struct e5010_q_data *queue;
	struct v4l2_format f;
	struct e5010_fmt *fmt;

	f.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	f.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
	fmt = find_format(&f);
	queue = &ctx->out_queue;
	queue->fmt = fmt;
	queue->width = DEFAULT_WIDTH;
	queue->height = DEFAULT_HEIGHT;
	queue->width_adjusted = queue->width;
	queue->height_adjusted = queue->height;

	v4l_bound_align_image(&queue->width_adjusted,
			      MIN_DIMENSION,
			      MAX_DIMENSION,
			      fmt->h_align,
			      &queue->height_adjusted,
			      MIN_DIMENSION, /* adjust upwards*/
			      MAX_DIMENSION,
			      fmt->v_align,
			      0);

	e5010_queue_update_bytesperline(queue);
	e5010_queue_update_sizeimage(queue, ctx);
	queue->format_set = false;
	queue->streaming = false;

	f.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	f.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_JPEG;
	fmt = find_format(&f);
	queue = &ctx->cap_queue;
	queue->fmt = fmt;
	queue->width = DEFAULT_WIDTH;
	queue->height = DEFAULT_HEIGHT;
	queue->width_adjusted = queue->width;
	queue->height_adjusted = queue->height;
	v4l_bound_align_image(&queue->width_adjusted,
			      MIN_DIMENSION,
			      MAX_DIMENSION,
			      4,
			      &queue->height_adjusted,
			      MIN_DIMENSION, /* adjust upwards*/
			      MAX_DIMENSION,
			      ctx->out_queue.fmt->v_align,
			      0);

	e5010_queue_update_bytesperline(queue);
	e5010_queue_update_sizeimage(queue, ctx);
	queue->format_set = false;
	queue->streaming = false;
}

static int e5010_open(struct file *file)
{
	struct e5010_dev *dev = video_drvdata(file);
	struct video_device *vdev = video_devdata(file);
	struct e5010_context *ctx;
	int ret = 0;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	if (mutex_lock_interruptible(&dev->mutex)) {
		ret = -ERESTARTSYS;
		goto free;
	}

	v4l2_fh_init(&ctx->fh, vdev);
	file->private_data = ctx;
	v4l2_fh_add(&ctx->fh);

	ctx->dev = dev;
	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(dev->m2m_dev, ctx, queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		dev_err(dev->dev, "Failed to init m2m ctx\n");
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto exit;
	}

	ret = e5010_ctrls_setup(ctx);
	if (ret) {
		dev_err(ctx->dev->dev, "failed to setup e5010 jpeg controls\n");
		goto err_ctrls_setup;
	}
	ctx->fh.ctrl_handler = &ctx->ctrl_handler;

	e5010_jpeg_set_default_params(ctx);

	dprintk(dev, 1, "Created instance: 0x%p, m2m_ctx: 0x%p\n", ctx, ctx->fh.m2m_ctx);

	mutex_unlock(&dev->mutex);
	return 0;

err_ctrls_setup:
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
exit:
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	mutex_unlock(&dev->mutex);
free:
	kfree(ctx);
	return ret;
}

static int e5010_release(struct file *file)
{
	struct e5010_dev *dev = video_drvdata(file);
	struct e5010_context *ctx = file->private_data;

	dprintk(dev, 1, "Releasing instance: 0x%p, m2m_ctx: 0x%p\n", ctx, ctx->fh.m2m_ctx);
	mutex_lock(&dev->mutex);
	v4l2_ctrl_handler_free(&ctx->ctrl_handler);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	mutex_unlock(&dev->mutex);

	return 0;
}

static struct video_device e5010_videodev = {
	.name = E5010_MODULE_NAME,
	.fops = &e5010_fops,
	.ioctl_ops = &e5010_ioctl_ops,
	.minor = -1,
	.release = video_device_release_empty,
	.vfl_dir = VFL_DIR_M2M,
	.device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING,
};

static void header_write(struct e5010_context *ctx, u8 *addr, unsigned int *offset,
			 unsigned int no_bytes, unsigned long bits)
{
	u8 *w_addr = addr + *offset;
	int i;

	if ((*offset + no_bytes) > HEADER_SIZE) {
		dev_warn(ctx->dev->dev, "%s: %s: %d: Problem writing header. %d > HEADER_SIZE %d\n",
			 __FILE__, __func__, __LINE__, *offset + no_bytes, HEADER_SIZE);
		return;
	}

	for (i = no_bytes - 1; i >= 0; i--)
		*(w_addr++) = ((u8 *)&bits)[i];

	*offset += no_bytes;
}

static void encode_marker_segment(struct e5010_context *ctx, void *addr, unsigned int *offset)
{
	u8 *buffer = (u8 *)addr;
	int i;

	header_write(ctx, buffer, offset, 2, START_OF_IMAGE);
	header_write(ctx, buffer, offset, 2, DQT_MARKER);
	header_write(ctx, buffer, offset, 3, LQPQ << 4);
	for (i = 0; i < PELS_IN_BLOCK; i++)
		header_write(ctx, buffer, offset, 1, ctx->luma_qp[zigzag[i]]);

	header_write(ctx, buffer, offset, 2, DQT_MARKER);
	header_write(ctx, buffer, offset, 3, (LQPQ << 4) | 1);
	for (i = 0; i < PELS_IN_BLOCK; i++)
		header_write(ctx, buffer, offset, 1, ctx->chroma_qp[zigzag[i]]);

	/* Huffman tables */
	header_write(ctx, buffer, offset, 2, DHT_MARKER);
	header_write(ctx, buffer, offset, 2, LH_DC);
	for (i = 0 ; i < (LH_DC - 2); i++)
		header_write(ctx, buffer, offset, 1, marker_luma_dc[i]);

	header_write(ctx, buffer, offset, 2, DHT_MARKER);
	header_write(ctx, buffer, offset, 2, LH_AC);
	for (i = 0 ; i < (LH_AC - 2); i++)
		header_write(ctx, buffer, offset, 1, marker_luma_ac[i]);

	header_write(ctx, buffer, offset, 2, DHT_MARKER);
	header_write(ctx, buffer, offset, 2, LH_DC);
	for (i = 0 ; i < (LH_DC - 2); i++)
		header_write(ctx, buffer, offset, 1, marker_chroma_dc[i]);

	header_write(ctx, buffer, offset, 2, DHT_MARKER);
	header_write(ctx, buffer, offset, 2, LH_AC);
	for (i = 0 ; i < (LH_AC - 2); i++)
		header_write(ctx, buffer, offset, 1, marker_chroma_ac[i]);
}

static void encode_frame_header(struct e5010_context *ctx, void *addr, unsigned int *offset)
{
	u8 *buffer = (u8 *)addr;

	header_write(ctx, buffer, offset, 2, SOF_BASELINE_DCT);
	header_write(ctx, buffer, offset, 2, 8 + (3 * UC_NUM_COMP));
	header_write(ctx, buffer, offset, 1, PRECISION);
	header_write(ctx, buffer, offset, 2, ctx->out_queue.height);
	header_write(ctx, buffer, offset, 2, ctx->out_queue.width);
	header_write(ctx, buffer, offset, 1, UC_NUM_COMP);

	/* Luma details */
	header_write(ctx, buffer, offset, 1, 1);
	if (ctx->out_queue.fmt->subsampling == V4L2_JPEG_CHROMA_SUBSAMPLING_422)
		header_write(ctx, buffer, offset, 1,
			     HORZ_SAMPLING_FACTOR | (VERT_SAMPLING_FACTOR_422));
	else
		header_write(ctx, buffer, offset, 1,
			     HORZ_SAMPLING_FACTOR | (VERT_SAMPLING_FACTOR_420));
	header_write(ctx, buffer, offset, 1, 0);
	/* Chroma details */
	header_write(ctx, buffer, offset, 1, 2);
	header_write(ctx, buffer, offset, 1, (HORZ_SAMPLING_FACTOR >> 1) | 1);
	header_write(ctx, buffer, offset, 1, 1);
	header_write(ctx, buffer, offset, 1, 3);
	header_write(ctx, buffer, offset, 1, (HORZ_SAMPLING_FACTOR >> 1) | 1);
	header_write(ctx, buffer, offset, 1, 1);

	header_write(ctx, buffer, offset, 1, 0xFF);
}

static void jpg_encode_sos_header(struct e5010_context *ctx, void *addr, unsigned int *offset)
{
	u8 *buffer = (u8 *)addr;
	int i;

	header_write(ctx, buffer, offset, 2, START_OF_SCAN);
	header_write(ctx, buffer, offset, 2, 6 + (COMPONENTS_IN_SCAN << 1));
	header_write(ctx, buffer, offset, 1, COMPONENTS_IN_SCAN);

	for (i = 0; i < COMPONENTS_IN_SCAN; i++) {
		header_write(ctx, buffer, offset, 1, i + 1);
		if (i == 0)
			header_write(ctx, buffer, offset, 1, 0);
		else
			header_write(ctx, buffer, offset, 1, 17);
	}

	header_write(ctx, buffer, offset, 1, 0);
	header_write(ctx, buffer, offset, 1, 63);
	header_write(ctx, buffer, offset, 1, 0);
}

static void write_header(struct e5010_context *ctx, void *addr)
{
	unsigned int offset = 0;

	encode_marker_segment(ctx, addr, &offset);
	encode_frame_header(ctx, addr, &offset);
	jpg_encode_sos_header(ctx, addr, &offset);
}

static irqreturn_t e5010_irq(int irq, void *data)
{
	struct e5010_dev *dev = data;
	struct e5010_context *ctx;
	int output_size;
	struct vb2_v4l2_buffer *src_buf, *dst_buf;
	bool pic_done, out_addr_err;

	spin_lock(&dev->hw_lock);
	pic_done = e5010_hw_pic_done_irq(dev->jasper_base);
	out_addr_err = e5010_hw_output_address_irq(dev->jasper_base);

	if (!pic_done && !out_addr_err) {
		spin_unlock(&dev->hw_lock);
		return IRQ_NONE;
	}

	ctx = v4l2_m2m_get_curr_priv(dev->m2m_dev);
	if (WARN_ON(!ctx))
		goto job_unlock;

	dst_buf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
	src_buf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
	if (!dst_buf || !src_buf) {
		dev_err(dev->dev, "ctx: 0x%p No source or destination buffer\n", ctx);
		goto job_unlock;
	}

	if (out_addr_err) {
		e5010_hw_clear_output_error(dev->jasper_base, 1);
		dev_warn(dev->dev, "ctx: 0x%p Output bitstream size exceeded max size\n", ctx);
		v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_ERROR);
		vb2_set_plane_payload(&dst_buf->vb2_buf, 0, dst_buf->planes[0].length);
		v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_ERROR);
		if (v4l2_m2m_is_last_draining_src_buf(ctx->fh.m2m_ctx, src_buf)) {
			dst_buf->flags |= V4L2_BUF_FLAG_LAST;
			v4l2_m2m_mark_stopped(ctx->fh.m2m_ctx);
			v4l2_event_queue_fh(&ctx->fh, &e5010_eos_event);
			dprintk(dev, 2, "ctx: 0x%p Sending EOS\n", ctx);
		}
	}

	if (pic_done) {
		e5010_hw_clear_picture_done(dev->jasper_base, 1);
		dprintk(dev, 3, "ctx: 0x%p Got output bitstream of size %d bytes\n",
			ctx, readl(dev->jasper_base + JASPER_OUTPUT_SIZE_OFFSET));

		if (v4l2_m2m_is_last_draining_src_buf(ctx->fh.m2m_ctx, src_buf)) {
			dst_buf->flags |= V4L2_BUF_FLAG_LAST;
			v4l2_m2m_mark_stopped(ctx->fh.m2m_ctx);
			v4l2_event_queue_fh(&ctx->fh, &e5010_eos_event);
			dprintk(dev, 2, "ctx: 0x%p Sending EOS\n", ctx);
		}
		v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_DONE);
		output_size = e5010_hw_get_output_size(dev->jasper_base);
		vb2_set_plane_payload(&dst_buf->vb2_buf, 0, output_size + HEADER_SIZE);
		v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_DONE);
		dprintk(dev, 3,
			"ctx: 0x%p frame done for dst_buf->sequence: %d src_buf->sequence: %d\n",
			ctx, dst_buf->sequence, src_buf->sequence);
	}

	v4l2_m2m_job_finish(dev->m2m_dev, ctx->fh.m2m_ctx);
	dprintk(dev, 3, "ctx: 0x%p Finish job\n", ctx);

job_unlock:
	spin_unlock(&dev->hw_lock);
	return IRQ_HANDLED;
}

static int e5010_init_device(struct e5010_dev *dev)
{
	int ret = 0;

	/*TODO: Set MMU in bypass mode until support for the same is added in driver*/
	e5010_hw_bypass_mmu(dev->mmu_base, 1);

	if (e5010_hw_enable_auto_clock_gating(dev->jasper_base, 1))
		dev_warn(dev->dev, "Failed to enable auto clock gating\n");

	if (e5010_hw_enable_manual_clock_gating(dev->jasper_base, 0))
		dev_warn(dev->dev, "Failed to disable manual clock gating\n");

	if (e5010_hw_enable_crc_check(dev->jasper_base, 0))
		dev_warn(dev->dev, "Failed to disable CRC check\n");

	if (e5010_hw_enable_output_address_error_irq(dev->jasper_base, 1))
		dev_err(dev->dev, "Failed to enable Output Address Error interrupts\n");

	ret = e5010_hw_set_input_source_to_memory(dev->jasper_base, 1);
	if (ret) {
		dev_err(dev->dev, "Failed to set input source to memory\n");
		goto fail;
	}

	ret = e5010_hw_enable_picture_done_irq(dev->jasper_base, 1);
	if (ret)
		dev_err(dev->dev, "Failed to enable Picture Done interrupts\n");
fail:
	return ret;
}

static int e5010_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_dev_id;
	struct e5010_dev *dev;
	struct resource *res;
	int irq, ret = 0;

	of_dev_id = of_match_device(e5010_of_match, &pdev->dev);
	if (!of_dev_id) {
		dev_err(&pdev->dev, "%s: Unable to match device\n", __func__);
		return -ENODEV;
	}

	ret = dma_set_mask(&pdev->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&pdev->dev, "32-bit consistent DMA enable failed\n");
		return ret;
	}

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	platform_set_drvdata(pdev, dev);

	dev->dev = &pdev->dev;

	mutex_init(&dev->mutex);
	spin_lock_init(&dev->hw_lock);

	dev->vdev = &e5010_videodev;
	dev->vdev->v4l2_dev = &dev->v4l2_dev;
	dev->vdev->lock = &dev->mutex;
	dev->vdev->queue = NULL;
	dev->vdev->prio = NULL;
	dev->vdev->dev_parent = NULL;
	dev->vdev->minor = -1;

	ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
	if (ret) {
		dev_err(dev->dev, "Failed to register v4l2 device\n");
		return ret;
	}

	dev->m2m_dev = v4l2_m2m_init(&e5010_m2m_ops);
	if (!dev->m2m_dev) {
		dev_err(dev->dev, "Failed to initialize m2m device\n");
		ret = -ENOMEM;
		goto fail_after_v4l2_register;
	}

	video_set_drvdata(dev->vdev, dev);

	ret = video_register_device(dev->vdev, VFL_TYPE_VIDEO, 0);
	if (ret) {
		dev_err(dev->dev, "Failed to register video device\n");
		ret = -ENOMEM;
		goto fail_after_v4l2_register;
	}

	dev_info(dev->dev, "Device registered as /dev/video%d\n",
		 dev->vdev->num);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "regjasper");
	if (!res) {
		dev_err(dev->dev, "Missing 'regjasper' resources area\n");
		ret = -ENOMEM;
		goto fail_after_video_register_device;
	}
	dev->jasper_base = devm_ioremap_resource(&pdev->dev, res);
	if (!dev->jasper_base) {
		ret = -ENOMEM;
		goto fail_after_video_register_device;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "regmmu");
	if (!res) {
		dev_err(dev->dev, "Missing 'regmmu' resources area\n");
		ret = -ENOMEM;
		goto fail_after_video_register_device;
	}
	dev->mmu_base = devm_ioremap_resource(&pdev->dev, res);
	if (!dev->mmu_base) {
		ret = -ENOMEM;
		goto fail_after_video_register_device;
	}

	dev->last_context_run = NULL;

	irq = platform_get_irq(pdev, 0);
	ret = devm_request_irq(dev->dev, irq, e5010_irq, 0,
			       E5010_MODULE_NAME, dev);
	if (ret) {
		dev_err(dev->dev, "Failed to register IRQ %d\n", irq);
		goto fail_after_video_register_device;
	}

	dev->clk = devm_clk_get(&pdev->dev, "core_clk");
	if (IS_ERR(dev->clk)) {
		dev_err(dev->dev, "failed to get clock\n");
		ret = PTR_ERR(dev->clk);
		goto fail_after_video_register_device;
	}

	pm_runtime_enable(dev->dev);

	return 0;

fail_after_video_register_device:
	video_device_release(dev->vdev);
	v4l2_m2m_release(dev->m2m_dev);
fail_after_v4l2_register:
	v4l2_device_unregister(&dev->v4l2_dev);
	return ret;
}

static int e5010_remove(struct platform_device *pdev)
{
	struct e5010_dev *dev = platform_get_drvdata(pdev);

	pm_runtime_disable(dev->dev);
	video_unregister_device(dev->vdev);
	v4l2_m2m_release(dev->m2m_dev);
	v4l2_device_unregister(&dev->v4l2_dev);

	return 0;
}

static int e5010_queue_setup(struct vb2_queue *vq, unsigned int *nbuffers, unsigned int *nplanes,
			     unsigned int sizes[], struct device *alloc_devs[])
{
	struct e5010_context *ctx = vb2_get_drv_priv(vq);
	struct e5010_q_data *queue;
	int i;

	if (!V4L2_TYPE_IS_MULTIPLANAR(vq->type)) {
		dev_err(ctx->dev->dev, "queue setup with Invalid type: %d\n", vq->type);
		return -EINVAL;
	}

	queue = get_queue(ctx, vq->type);
	if (IS_ERR(queue))
		return PTR_ERR(queue);

	if (*nplanes) {
		if (*nplanes != queue->fmt->num_planes)
			return -EINVAL;
		for (i = 0; i < *nplanes; i++) {
			if (sizes[i] < queue->sizeimage[i])
				return -EINVAL;
		}
		return 0;
	}

	*nbuffers = max_t(unsigned int, *nbuffers, 1);
	*nplanes = queue->fmt->num_planes;
	for (i = 0; i < *nplanes; i++)
		sizes[i] = queue->sizeimage[i];

	dprintk(ctx->dev, 2,
		"ctx: 0x%p, type %s, buffer(s): %d, planes %d, plane1: bytes %d plane2: %d bytes\n",
		ctx, type_name(vq->type), *nbuffers, *nplanes, sizes[0], sizes[1]);

	return 0;
}

static void e5010_buf_finish(struct vb2_buffer *vb)
{
	struct e5010_context *ctx = vb2_get_drv_priv(vb->vb2_queue);
	void *d_addr;

	if (vb->state != VB2_BUF_STATE_DONE || V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type))
		return;

	d_addr = vb2_plane_vaddr(vb, 0);
	write_header(ctx, d_addr);
}

static int e5010_buf_out_validate(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct e5010_context *ctx = vb2_get_drv_priv(vb->vb2_queue);

	if (vbuf->field != V4L2_FIELD_NONE)
		dprintk(ctx->dev, 1, "ctx: 0x%p, field isn't supported\n", ctx);

	vbuf->field = V4L2_FIELD_NONE;

	return 0;
}

static int e5010_buf_prepare(struct vb2_buffer *vb)
{
	struct e5010_context *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct e5010_q_data *queue;
	int i;

	vbuf->field = V4L2_FIELD_NONE;

	queue = get_queue(ctx, vb->vb2_queue->type);
	if (IS_ERR(queue))
		return PTR_ERR(queue);

	for (i = 0; i < queue->fmt->num_planes; i++) {
		if (vb2_plane_size(vb, i) < (unsigned long)queue->sizeimage[i]) {
			dev_err(ctx->dev->dev, "plane %d too small (%lu < %lu)", i,
				vb2_plane_size(vb, i), (unsigned long)queue->sizeimage[i]);

			return -EINVAL;
		}
	}

	if (V4L2_TYPE_IS_CAPTURE(vb->vb2_queue->type)) {
		vb2_set_plane_payload(vb, 0, 0);
		vb2_set_plane_payload(vb, 1, 0);
	}

	return 0;
}

static void e5010_buf_queue(struct vb2_buffer *vb)
{
	struct e5010_context *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	if (V4L2_TYPE_IS_CAPTURE(vb->vb2_queue->type) &&
	    vb2_is_streaming(vb->vb2_queue) &&
	    v4l2_m2m_dst_buf_is_last(ctx->fh.m2m_ctx)) {
		struct e5010_q_data *queue = get_queue(ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);

		if (IS_ERR(queue))
			return;
		vbuf->sequence = queue->sequence++;
		v4l2_m2m_last_buffer_done(ctx->fh.m2m_ctx, vbuf);
		v4l2_event_queue_fh(&ctx->fh, &e5010_eos_event);
		return;
	}

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
}

static int e5010_encoder_cmd(struct file *file, void *priv,
			     struct v4l2_encoder_cmd *cmd)
{
	struct e5010_context *ctx = file->private_data;
	int ret;
	struct vb2_queue *cap_vq;

	cap_vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);

	ret = v4l2_m2m_ioctl_try_encoder_cmd(file, &ctx->fh, cmd);
	if (ret < 0)
		return ret;

	if (!vb2_is_streaming(v4l2_m2m_get_src_vq(ctx->fh.m2m_ctx)) ||
	    !vb2_is_streaming(v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx)))
		return 0;

	ret = v4l2_m2m_ioctl_encoder_cmd(file, &ctx->fh, cmd);
	if (ret < 0)
		return ret;

	if (cmd->cmd == V4L2_ENC_CMD_STOP &&
	    v4l2_m2m_has_stopped(ctx->fh.m2m_ctx))
		v4l2_event_queue_fh(&ctx->fh, &e5010_eos_event);

	if (cmd->cmd == V4L2_ENC_CMD_START &&
	    v4l2_m2m_has_stopped(ctx->fh.m2m_ctx))
		vb2_clear_last_buffer_dequeued(cap_vq);

	return 0;
}

static int e5010_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct e5010_context *ctx = vb2_get_drv_priv(q);
	int ret;

	struct e5010_q_data *queue = get_queue(ctx, q->type);

	if (IS_ERR(queue))
		return PTR_ERR(queue);
	queue->streaming = true;
	v4l2_m2m_update_start_streaming_state(ctx->fh.m2m_ctx, q);
	queue->sequence = 0;

	ret = pm_runtime_resume_and_get(ctx->dev->dev);
	if (ret < 0) {
		dev_err(ctx->dev->dev, "Failed to power up jpeg\n");
		return ret;
	}

	ret = e5010_init_device(ctx->dev);
	if (ret)
		dev_err(ctx->dev->dev, "Failed to Enable e5010 device\n");

	return ret;
}

static void e5010_stop_streaming(struct vb2_queue *q)
{
	struct e5010_context *ctx = vb2_get_drv_priv(q);
	struct vb2_v4l2_buffer *vbuf;
	struct e5010_q_data *queue;

	queue = get_queue(ctx, q->type);
	if (IS_ERR(queue))
		return;

	queue->streaming = false;

	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		while ((vbuf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx))) {
			dprintk(ctx->dev, 2, "ctx: 0x%p, buf type %s | index %d\n",
				ctx, type_name(vbuf->vb2_buf.type), vbuf->vb2_buf.index);
			v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
		}
	} else {
		while ((vbuf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx))) {
			dprintk(ctx->dev, 2, "ctx: 0x%p, buf type %s | index %d\n",
				ctx, type_name(vbuf->vb2_buf.type), vbuf->vb2_buf.index);
			vb2_set_plane_payload(&vbuf->vb2_buf, 0, 0);
			v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_ERROR);
		}
	}

	if (V4L2_TYPE_IS_OUTPUT(q->type))
		v4l2_m2m_update_stop_streaming_state(ctx->fh.m2m_ctx, q);

	if (V4L2_TYPE_IS_OUTPUT(q->type) &&
	    v4l2_m2m_has_stopped(ctx->fh.m2m_ctx)) {
		v4l2_event_queue_fh(&ctx->fh, &e5010_eos_event);
	}

	pm_runtime_put_sync(ctx->dev->dev);
}

static void e5010_device_run(void *priv)
{
	struct e5010_context *ctx = priv;
	struct e5010_dev *dev = ctx->dev;
	struct vb2_v4l2_buffer *s_vb, *d_vb;
	u32 reg = 0;
	int ret = 0;
	unsigned long flags;
	int num_planes = ctx->out_queue.fmt->num_planes;

	spin_lock_irqsave(&dev->hw_lock, flags);
	s_vb = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	WARN_ON(!s_vb);
	d_vb = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	WARN_ON(!d_vb);
	if (!s_vb || !d_vb)
		goto no_ready_buf_err;

	s_vb->sequence = ctx->out_queue.sequence++;
	d_vb->sequence = ctx->cap_queue.sequence++;

	v4l2_m2m_buf_copy_metadata(s_vb, d_vb, false);

	if (ctx != dev->last_context_run || ctx->update_qp) {
		dprintk(dev, 1, "ctx updated: 0x%p -> 0x%p, updating qp tables\n",
			dev->last_context_run, ctx);
		ret = update_qp_tables(ctx);
	}

	if (ret) {
		ctx->update_qp = true;
		dev_err(dev->dev, "Failed to update QP tables\n");
		goto device_busy_err;
	} else {
		dev->last_context_run = ctx;
		ctx->update_qp = false;
	}

	/* Set I/O Buffer addresses */
	reg = (u32)vb2_dma_contig_plane_dma_addr(&s_vb->vb2_buf, 0);
	ret = e5010_hw_set_input_luma_addr(dev->jasper_base, reg);
	if (ret || !reg) {
		dev_err(dev->dev, "Failed to set input luma address\n");
		goto device_busy_err;
	}

	if (num_planes == 1)
		reg += (ctx->out_queue.bytesperline[0]) * (ctx->out_queue.height);
	else
		reg = (u32)vb2_dma_contig_plane_dma_addr(&s_vb->vb2_buf, 1);

	dprintk(dev, 3,
		"ctx: 0x%p, luma_addr: 0x%x, chroma_addr: 0x%x, out_addr: 0x%x\n",
		ctx, (u32)vb2_dma_contig_plane_dma_addr(&s_vb->vb2_buf, 0), reg,
		(u32)vb2_dma_contig_plane_dma_addr(&d_vb->vb2_buf, 0));

	dprintk(dev, 3,
		"ctx: 0x%p, buf indices: src_index: %d, dst_index: %d\n",
		ctx, s_vb->vb2_buf.index, d_vb->vb2_buf.index);

	ret = e5010_hw_set_input_chroma_addr(dev->jasper_base, reg);
	if (ret || !reg) {
		dev_err(dev->dev, "Failed to set input chroma address\n");
		goto device_busy_err;
	}

	reg = (u32)vb2_dma_contig_plane_dma_addr(&d_vb->vb2_buf, 0);
	reg += HEADER_SIZE;
	ret = e5010_hw_set_output_base_addr(dev->jasper_base, reg);
	if (ret || !reg) {
		dev_err(dev->dev, "Failed to set output size\n");
		goto device_busy_err;
	}

	/* Set input settings */
	ret = e5010_hw_set_horizontal_size(dev->jasper_base, ctx->out_queue.width - 1);
	if (ret) {
		dev_err(dev->dev, "Failed to set input width\n");
		goto device_busy_err;
	}

	ret = e5010_hw_set_vertical_size(dev->jasper_base, ctx->out_queue.height - 1);
	if (ret) {
		dev_err(dev->dev, "Failed to set input width\n");
		goto device_busy_err;
	}

	ret = e5010_hw_set_luma_stride(dev->jasper_base, ctx->out_queue.bytesperline[0]);
	if (ret) {
		dev_err(dev->dev, "Failed to set luma stride\n");
		goto device_busy_err;
	}

	ret = e5010_hw_set_chroma_stride(dev->jasper_base, ctx->out_queue.bytesperline[0]);
	if (ret) {
		dev_err(dev->dev, "Failed to set chroma stride\n");
		goto device_busy_err;
	}

	ret = e5010_set_input_subsampling(dev->jasper_base, ctx->out_queue.fmt->subsampling);
	if (ret) {
		dev_err(dev->dev, "Failed to set input subsampling\n");
		goto device_busy_err;
	}

	ret = e5010_hw_set_chroma_order(dev->jasper_base, ctx->out_queue.fmt->chroma_order);
	if (ret) {
		dev_err(dev->dev, "Failed to set chroma order\n");
		goto device_busy_err;
	}

	e5010_hw_set_output_max_size(dev->jasper_base, d_vb->planes[0].length);
	e5010_hw_encode_start(dev->jasper_base, 1);

	spin_unlock_irqrestore(&dev->hw_lock, flags);

	return;

device_busy_err:
	e5010_reset(dev->dev, dev->jasper_base, dev->mmu_base);

no_ready_buf_err:
	if (s_vb) {
		v4l2_m2m_src_buf_remove_by_buf(ctx->fh.m2m_ctx, s_vb);
		v4l2_m2m_buf_done(s_vb, VB2_BUF_STATE_ERROR);
	}

	if (d_vb) {
		v4l2_m2m_dst_buf_remove_by_buf(ctx->fh.m2m_ctx, d_vb);
		/* Payload set to 1 since 0 payload can trigger EOS */
		vb2_set_plane_payload(&d_vb->vb2_buf, 0, 1);
		v4l2_m2m_buf_done(d_vb, VB2_BUF_STATE_ERROR);
	}
	v4l2_m2m_job_finish(dev->m2m_dev, ctx->fh.m2m_ctx);
	spin_unlock_irqrestore(&dev->hw_lock, flags);

	return;
}

#ifdef CONFIG_PM
static int e5010_runtime_resume(struct device *dev)
{
	struct e5010_dev *e5010_dev = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(e5010_dev->clk);
	if (ret < 0) {
		dev_err(dev, "failed to enable clock\n");
		return ret;
	}

	return 0;
}

static int e5010_runtime_suspend(struct device *dev)
{
	struct e5010_dev *e5010_dev = dev_get_drvdata(dev);

	clk_disable_unprepare(e5010_dev->clk);

	return 0;
}
#endif

#ifdef CONFIG_PM_SLEEP
static int e5010_suspend(struct device *dev)
{
	struct e5010_dev *e5010_dev = dev_get_drvdata(dev);

	v4l2_m2m_suspend(e5010_dev->m2m_dev);
	return pm_runtime_force_suspend(dev);
}

static int e5010_resume(struct device *dev)
{
	struct e5010_dev *e5010_dev = dev_get_drvdata(dev);
	int ret;

	ret = pm_runtime_force_resume(dev);
	if (ret < 0)
		return ret;

	ret = e5010_init_device(e5010_dev);
	if (ret) {
		dev_err(dev, "Failed to re-enable e5010 device\n");
		return ret;
	}

	v4l2_m2m_resume(e5010_dev->m2m_dev);
	return ret;
}
#endif

static const struct dev_pm_ops	e5010_pm_ops = {
	SET_RUNTIME_PM_OPS(e5010_runtime_suspend,
			   e5010_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(e5010_suspend, e5010_resume)
};

static const struct v4l2_ioctl_ops e5010_ioctl_ops = {
	.vidioc_querycap = e5010_querycap,

	.vidioc_enum_fmt_vid_cap = e5010_enum_fmt,
	.vidioc_g_fmt_vid_cap_mplane = e5010_g_fmt,
	.vidioc_try_fmt_vid_cap_mplane = e5010_try_fmt,
	.vidioc_s_fmt_vid_cap_mplane = e5010_s_fmt,

	.vidioc_enum_fmt_vid_out = e5010_enum_fmt,
	.vidioc_g_fmt_vid_out_mplane = e5010_g_fmt,
	.vidioc_try_fmt_vid_out_mplane = e5010_try_fmt,
	.vidioc_s_fmt_vid_out_mplane = e5010_s_fmt,

	.vidioc_g_selection = e5010_g_selection,
	.vidioc_s_selection = e5010_s_selection,

	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,

	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,
	.vidioc_log_status = v4l2_ctrl_log_status,

	.vidioc_subscribe_event = e5010_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
	.vidioc_try_encoder_cmd = v4l2_m2m_ioctl_try_encoder_cmd,
	.vidioc_encoder_cmd = e5010_encoder_cmd,

	.vidioc_enum_framesizes = e5010_enum_framesizes,
};

static const struct vb2_ops e5010_video_ops = {
	.queue_setup = e5010_queue_setup,
	.buf_queue = e5010_buf_queue,
	.buf_finish = e5010_buf_finish,
	.buf_prepare = e5010_buf_prepare,
	.buf_out_validate = e5010_buf_out_validate,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
	.start_streaming = e5010_start_streaming,
	.stop_streaming = e5010_stop_streaming,
};

static const struct v4l2_file_operations e5010_fops = {
	.owner = THIS_MODULE,
	.open = e5010_open,
	.release = e5010_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

static const struct v4l2_m2m_ops e5010_m2m_ops = {
	.device_run = e5010_device_run,
};

static const struct of_device_id e5010_of_match[] = {
	{.compatible = "img,e5010-jpeg-enc"},   { /* end */},
};
MODULE_DEVICE_TABLE(of, e5010_of_match);

static struct platform_driver e5010_driver = {
	.probe = e5010_probe,
	.remove = e5010_remove,
	.driver = {
		.name = E5010_MODULE_NAME,
		.of_match_table = e5010_of_match,
		.pm = &e5010_pm_ops,
	},
};
module_platform_driver(e5010_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Imagination E5010 JPEG encoder driver");
