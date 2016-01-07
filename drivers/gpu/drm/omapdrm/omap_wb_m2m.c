/*
 * TI OMAP WB mem2mem driver
 *
 * Copyright (c) 2015 Texas Instruments Inc.
 * Benoit Parrot, <bparrot@ti.com>
 *
 * Based on the virtual v4l2-mem2mem example device
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation
 */

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/log2.h>
#include <linux/sizes.h>
#include <video/omapdss.h>
#include <drm/drm_fourcc.h>

#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>

#include "omap_drv.h"

#define WBM2M_MODULE_NAME "omapwb-m2m"

#define WBM2M_VERSION "0.1.0"

MODULE_DESCRIPTION("TI OMAP WB M2M driver");
MODULE_AUTHOR("Benoit Parrot <bparrot@ti.com>");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(WBM2M_VERSION);

static unsigned wbdebug;
module_param(wbdebug, uint, 0644);
MODULE_PARM_DESC(wbdebug, "activates debug info");

/* minimum and maximum frame sizes */
#define MIN_W		32
#define MIN_H		32
#define MAX_W		2048
#define MAX_H		2048

/* required alignments */
#define S_ALIGN		0	/* multiple of 1 */
#define H_ALIGN		0	/* multiple of 2 */

/* used as plane indices */
#define MAX_PLANES	2
#define LUMA_PLANE	0
#define CHROMA_PLANE	1

#define log_dbg(dev, fmt, arg...)	\
		v4l2_dbg(1, wbdebug, &dev->v4l2_dev, "%s: " fmt, \
			 __func__, ## arg)
#define log_err(dev, fmt, arg...)	\
		v4l2_err(&dev->v4l2_dev, fmt, ## arg)
#define log_info(dev, fmt, arg...)	\
		v4l2_info(&dev->v4l2_dev, fmt, ## arg)

/* driver info for each of the supported video formats */
struct wbm2m_fmt {
	u32	fourcc;			/* standard format identifier */
	u8	coplanar;		/* set for unpacked Luma and Chroma */
	u8	depth[MAX_PLANES];	/* Bits per pixel per plane*/
};

static struct wbm2m_fmt wbm2m_formats[] = {
	{
		.fourcc		= V4L2_PIX_FMT_NV12,
		.coplanar	= 1,
		.depth		= {8, 4},
	},
	{
		.fourcc		= V4L2_PIX_FMT_YUYV,
		.coplanar	= 0,
		.depth		= {16, 0},
	},
	{
		.fourcc		= V4L2_PIX_FMT_UYVY,
		.coplanar	= 0,
		.depth		= {16, 0},
	},
	{
		/* "XR24", DRM_FORMAT_XRGB8888 */
		.fourcc		= V4L2_PIX_FMT_XBGR32,
		.coplanar	= 0,
		.depth		= {32, 0},
	},
};

/*  Print Four-character-code (FOURCC) */
static char *fourcc_to_str(u32 fmt)
{
	static char code[5];

	code[0] = (unsigned char)(fmt & 0xff);
	code[1] = (unsigned char)((fmt >> 8) & 0xff);
	code[2] = (unsigned char)((fmt >> 16) & 0xff);
	code[3] = (unsigned char)((fmt >> 24) & 0xff);
	code[4] = '\0';

	return code;
}

/*
 * per-queue, driver-specific private data.
 * there is one source queue and one destination queue for each m2m context.
 */
struct wbm2m_q_data {
	/* frame width */
	unsigned int		width;
	/* frame height */
	unsigned int		height;
	/* bytes per line in memory */
	unsigned int		bytesperline[MAX_PLANES];
	enum v4l2_colorspace	colorspace;
	/* supported field value */
	enum v4l2_field		field;
	/* image size in memory */
	unsigned int		sizeimage[MAX_PLANES];
	/* crop/compose rectangle */
	struct v4l2_rect	c_rect;
	/* format info */
	struct wbm2m_fmt	*fmt;
};

enum {
	Q_DATA_SRC = 0,
	Q_DATA_DST = 1,
};

/* find our format description corresponding to the passed v4l2_format */
static struct wbm2m_fmt *find_format(struct v4l2_format *f)
{
	struct wbm2m_fmt *fmt;
	unsigned int k;

	for (k = 0; k < ARRAY_SIZE(wbm2m_formats); k++) {
		fmt = &wbm2m_formats[k];
		if (fmt->fourcc == f->fmt.pix.pixelformat)
			return fmt;
	}

	return NULL;
}

/*
 * there is one wbm2m_dev structure in the driver, it is shared by
 * all instances.
 */
struct wbm2m_dev {
	struct v4l2_device	v4l2_dev;
	struct video_device	vfd;
	struct v4l2_m2m_dev	*m2m_dev;
	struct drm_device	*drm_dev;
	struct drm_plane	*plane;

	struct omap_drm_irq	wb_irq;

	/* count of driver instances */
	atomic_t		num_instances;
	/* v4l2_ioctl mutex */
	struct mutex		dev_mutex;
	/* v4l2 buffers lock */
	spinlock_t		lock;

	struct vb2_alloc_ctx	*alloc_ctx;
};

/*
 * There is one wbm2m_ctx structure for each m2m context.
 */
struct wbm2m_ctx {
	struct v4l2_fh		fh;
	struct wbm2m_dev	*dev;
	struct v4l2_ctrl_handler hdl;

	/* current frame seq */
	unsigned int		sequence;
	/* abort after next irq */
	unsigned int		aborting;
	/* bufs done in this batch */
	unsigned int		bufs_completed;

	/* src & dst queue data */
	struct wbm2m_q_data	q_data[2];
	struct vb2_buffer	*src_vb;
	struct vb2_buffer	*dst_vb;
};

/*
 * M2M devices get 2 queues.
 * Return the queue given the type.
 */
static struct wbm2m_q_data *get_q_data(struct wbm2m_ctx *ctx,
				       enum v4l2_buf_type type)
{
	switch (type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		return &ctx->q_data[Q_DATA_SRC];
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		return &ctx->q_data[Q_DATA_DST];
	default:
		return NULL;
	}
	return NULL;
}

static inline dma_addr_t vb2_dma_addr_plus_data_offset(struct vb2_buffer *vb,
						       unsigned int plane_no)
{
	return vb2_dma_contig_plane_dma_addr(vb, plane_no) +
		vb->v4l2_planes[plane_no].data_offset;
}

static enum omap_color_mode fourcc_to_dss(u32 fourcc)
{
	switch (fourcc) {
	case DRM_FORMAT_XRGB8888:
		return OMAP_DSS_COLOR_RGB24U;

	case DRM_FORMAT_NV12:
		return OMAP_DSS_COLOR_NV12;
	case DRM_FORMAT_YUYV:
		return OMAP_DSS_COLOR_YUV2;
	case DRM_FORMAT_UYVY:
		return OMAP_DSS_COLOR_UYVY;
	default:
		BUG();
	}
}

static bool wbm2m_convert(struct wbm2m_dev *dev, enum omap_plane src_plane,
			  const struct omap_overlay_info *src_info,
			  const struct omap_dss_writeback_info *wb_info)
{
	struct omap_drm_private *priv = dev->drm_dev->dev_private;
	enum dss_writeback_channel wb_channel;
	struct omap_video_timings t = { 0 };
	int r;

	t.x_res = src_info->out_width;
	t.y_res = src_info->out_height;

	/* configure input */

	log_dbg(dev, "WB IN plane %d, %dx%d -> %dx%d\n", src_plane,
		src_info->width, src_info->height,
		src_info->out_width, src_info->out_height);

	r = priv->dispc_ops->ovl_setup(src_plane, src_info, 0, &t, 1);
	if (r)
		return false;

	priv->dispc_ops->ovl_set_channel_out(src_plane, OMAP_DSS_CHANNEL_WB);
	priv->dispc_ops->ovl_enable(src_plane, true);

	/* configure output */

	r = priv->dispc_ops->wb_setup(wb_info, true, &t);
	if (r) {
		priv->dispc_ops->ovl_enable(src_plane, false);
		return false;
	}

	switch (src_plane) {
	case OMAP_DSS_GFX:
		wb_channel = DSS_WB_OVL0; break;
	case OMAP_DSS_VIDEO1:
		wb_channel = DSS_WB_OVL1; break;
	case OMAP_DSS_VIDEO2:
		wb_channel = DSS_WB_OVL2; break;
	case OMAP_DSS_VIDEO3:
		wb_channel = DSS_WB_OVL3; break;
	default:
		BUG();
	}

	priv->dispc_ops->wb_set_channel_in(wb_channel);

	priv->dispc_ops->ovl_enable(OMAP_DSS_WB, true);

	return true;
}

/*
 * Return the wbm2m_ctx structure for a given struct file
 */
static struct wbm2m_ctx *file2ctx(struct file *file)
{
	return container_of(file->private_data, struct wbm2m_ctx, fh);
}

/*
 * mem2mem callbacks
 */

/**
 * job_ready() - check whether an instance is ready to be scheduled to run
 */
static int job_ready(void *priv)
{
	struct wbm2m_ctx *ctx = priv;

	/*
	 * This check is needed as this might be called directly from driver
	 * When called by m2m framework, this will always satisy, but when
	 * called from wbm2m_irq, this might fail.
	 * (src stream with zero buffers)
	 */
	if (v4l2_m2m_num_src_bufs_ready(ctx->fh.m2m_ctx) <= 0 ||
	    v4l2_m2m_num_dst_bufs_ready(ctx->fh.m2m_ctx) <= 0)
		return 0;

	return 1;
}

static void job_abort(void *priv)
{
	struct wbm2m_ctx *ctx = priv;

	/* Will cancel the transaction in the next interrupt handler */
	ctx->aborting = 1;

	log_dbg(ctx->dev, "Aborting transaction\n");
	v4l2_m2m_job_finish(ctx->dev->m2m_dev, ctx->fh.m2m_ctx);
}

/*
 * Lock access to the device
 */
static void wbm2m_lock(void *priv)
{
	struct wbm2m_ctx *ctx = priv;
	struct wbm2m_dev *dev = ctx->dev;

	mutex_lock(&dev->dev_mutex);
}

static void wbm2m_unlock(void *priv)
{
	struct wbm2m_ctx *ctx = priv;
	struct wbm2m_dev *dev = ctx->dev;

	mutex_unlock(&dev->dev_mutex);
}

/* device_run() - prepares and starts the device
 *
 * This function is only called when both the source and destination
 * buffers are in place.
 */
static void device_run(void *priv)
{
	struct wbm2m_ctx *ctx = priv;
	struct wbm2m_dev *dev = ctx->dev;
	struct wbm2m_q_data *d_q_data = &ctx->q_data[Q_DATA_DST];
	struct wbm2m_q_data *s_q_data = &ctx->q_data[Q_DATA_SRC];
	dma_addr_t src_dma_addr[2] = {0, 0};
	dma_addr_t dst_dma_addr[2] = {0, 0};
	struct omap_overlay_info src_info = { 0 };
	struct omap_dss_writeback_info wb_info = { 0 };
	bool ok;

	ctx->src_vb = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
	WARN_ON(!ctx->src_vb);

	ctx->dst_vb = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
	WARN_ON(!ctx->dst_vb);

	src_dma_addr[0] = vb2_dma_addr_plus_data_offset(ctx->src_vb, 0);
	if (ctx->q_data[Q_DATA_SRC].fmt->coplanar)
		src_dma_addr[1] = vb2_dma_addr_plus_data_offset(ctx->src_vb, 1);
	if (!src_dma_addr[0]) {
		log_err(dev,
			"acquiring source buffer(%d) dma_addr failed\n",
			ctx->src_vb->v4l2_buf.index);
		return;
	}

	dst_dma_addr[0] = vb2_dma_addr_plus_data_offset(ctx->dst_vb, 0);
	if (ctx->q_data[Q_DATA_DST].fmt->coplanar)
		dst_dma_addr[1] = vb2_dma_addr_plus_data_offset(ctx->dst_vb, 1);
	if (!dst_dma_addr[0]) {
		log_err(dev,
			"acquiring destination buffer(%d) dma_addr failed\n",
			ctx->dst_vb->v4l2_buf.index);
		return;
	}

	/* fill source DSS info */
	src_info.paddr = (u32)src_dma_addr[0];
	src_info.p_uv_addr = (u32)src_dma_addr[1];

	src_info.screen_width = s_q_data->bytesperline[0] /
				(s_q_data->fmt->depth[0] / 8);

	src_info.width = s_q_data->width;
	src_info.height = s_q_data->height;

	src_info.pos_x = 0;
	src_info.pos_y = 0;
	src_info.out_width = s_q_data->width;
	src_info.out_height = s_q_data->height;

	src_info.color_mode = fourcc_to_dss(s_q_data->fmt->fourcc);
	src_info.global_alpha = 0xff;

	src_info.rotation = OMAP_DSS_ROT_0;
	src_info.rotation_type = OMAP_DSS_ROT_DMA;

	log_dbg(dev, "SRC: %dx%d, sw %d\n", src_info.width,
		src_info.height, src_info.screen_width);

	/* fill WB DSS info */
	wb_info.paddr = (u32)dst_dma_addr[0];
	wb_info.p_uv_addr = (u32)dst_dma_addr[1];

	wb_info.buf_width = d_q_data->bytesperline[0] /
			    (d_q_data->fmt->depth[0] / 8);

	wb_info.width = d_q_data->width;
	wb_info.height = d_q_data->height;
	wb_info.color_mode = fourcc_to_dss(d_q_data->fmt->fourcc);
	wb_info.pre_mult_alpha = 1;

	wb_info.rotation = OMAP_DSS_ROT_0;
	wb_info.rotation_type = OMAP_DSS_ROT_DMA;

	log_dbg(dev, "DST: %dx%d, sw %d\n", wb_info.width,
		wb_info.height, wb_info.buf_width);

	ok = wbm2m_convert(dev, omap_plane_id(dev->plane), &src_info, &wb_info);
	if (!ok) {
		log_err(dev,
			"Conversion setup failed, check source and destination parameters\n"
			);
		log_err(dev, "\tSRC: %dx%d, fmt: %s sw %d\n", src_info.width,
			src_info.height, fourcc_to_str(s_q_data->fmt->fourcc),
			src_info.screen_width);
		log_err(dev, "\tDST: %dx%d, fmr: %s sw %d\n", wb_info.width,
			wb_info.height, fourcc_to_str(d_q_data->fmt->fourcc),
			wb_info.buf_width);
		return;
	}
}

static void wbm2m_irq(struct omap_drm_irq *irq, uint32_t irqstatus)
{
	struct wbm2m_dev *dev =	container_of(irq, struct wbm2m_dev, wb_irq);
	struct wbm2m_ctx *ctx;
	struct wbm2m_q_data *d_q_data;
	struct wbm2m_q_data *s_q_data;
	struct vb2_buffer *s_vb, *d_vb;
	struct v4l2_buffer *s_buf, *d_buf;
	unsigned long flags;
	bool wb_done = false;

	ctx = v4l2_m2m_get_curr_priv(dev->m2m_dev);
	if (!ctx) {
		log_err(dev, "instance released before end of transaction\n");
		goto handled;
	}

	if (irqstatus & DISPC_IRQ_FRAMEDONEWB) {
		log_dbg(dev, "WB: FRAMEDONE\n");
		wb_done = true;
	}

	if (irqstatus & DISPC_IRQ_WBBUFFEROVERFLOW)
		log_err(dev, "WB: UNDERFLOW\n");

	if (irqstatus & DISPC_IRQ_WBUNCOMPLETEERROR)
		log_err(dev, "WB: DISPC_IRQ_WBUNCOMPLETEERROR\n");

	if (!wb_done)
		goto handled;

	if (ctx->aborting)
		goto finished;

	s_vb = ctx->src_vb;
	d_vb = ctx->dst_vb;
	s_buf = &s_vb->v4l2_buf;
	d_buf = &d_vb->v4l2_buf;

	d_buf->flags = s_buf->flags;

	d_buf->timestamp = s_buf->timestamp;
	if (s_buf->flags & V4L2_BUF_FLAG_TIMECODE)
		d_buf->timecode = s_buf->timecode;

	d_buf->sequence = ctx->sequence;

	d_q_data = &ctx->q_data[Q_DATA_DST];
	d_buf->field = V4L2_FIELD_NONE;
	ctx->sequence++;

	s_q_data = &ctx->q_data[Q_DATA_SRC];

	spin_lock_irqsave(&dev->lock, flags);

	if (s_vb)
		v4l2_m2m_buf_done(s_vb, VB2_BUF_STATE_DONE);

	v4l2_m2m_buf_done(d_vb, VB2_BUF_STATE_DONE);

	spin_unlock_irqrestore(&dev->lock, flags);

	/*
	 * Since the vb2_buf_done has already been called for therse
	 * buffer we can now NULL them out so that we won't try
	 * to clean out stray pointer later on.
	 */
	ctx->src_vb = NULL;
	ctx->dst_vb = NULL;

	ctx->bufs_completed++;

finished:
	log_dbg(ctx->dev, "finishing transaction\n");
	ctx->bufs_completed = 0;
	v4l2_m2m_job_finish(dev->m2m_dev, ctx->fh.m2m_ctx);
handled:
	return;
}

/*
 * video ioctls
 */
static int wbm2m_querycap(struct file *file, void *priv,
			  struct v4l2_capability *cap)
{
	struct wbm2m_ctx *ctx = file2ctx(file);

	strncpy(cap->driver, WBM2M_MODULE_NAME, sizeof(cap->driver) - 1);
	strncpy(cap->card, WBM2M_MODULE_NAME, sizeof(cap->card) - 1);
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 ctx->dev->v4l2_dev.name);
	cap->device_caps  = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}

static int wbm2m_enum_fmt(struct file *file, void *priv,
			  struct v4l2_fmtdesc *f)
{
	if (f->index >= ARRAY_SIZE(wbm2m_formats))
		return -EINVAL;

	f->pixelformat = wbm2m_formats[f->index].fourcc;
	return 0;
}

static int wbm2m_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	struct wbm2m_ctx *ctx = file2ctx(file);
	struct vb2_queue *vq;
	struct wbm2m_q_data *q_data;
	int i;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	q_data = get_q_data(ctx, f->type);

	pix->width = q_data->width;
	pix->height = q_data->height;
	pix->pixelformat = q_data->fmt->fourcc;
	pix->field = q_data->field;

	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		pix->colorspace = q_data->colorspace;
	} else {
		struct wbm2m_q_data *s_q_data;

		/* get colorspace from the source queue */
		s_q_data = get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);

		pix->colorspace = s_q_data->colorspace;
	}

	pix->num_planes = q_data->fmt->coplanar ? 2 : 1;

	for (i = 0; i < pix->num_planes; i++) {
		pix->plane_fmt[i].bytesperline = q_data->bytesperline[i];
		pix->plane_fmt[i].sizeimage = q_data->sizeimage[i];
	}

	return 0;
}

static int wbm2m_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct wbm2m_ctx *ctx = file2ctx(file);
	struct wbm2m_fmt *fmt = find_format(f);
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	struct v4l2_plane_pix_format *plane_fmt;
	unsigned int w_align;
	int i, depth, depth_bytes;

	if (!fmt) {
		log_err(ctx->dev, "Fourcc format (0x%08x) invalid.\n",
			pix->pixelformat);
		return -EINVAL;
	}

	/* we only allow V4L2_FIELD_NONE */
	if (pix->field != V4L2_FIELD_NONE)
		pix->field = V4L2_FIELD_NONE;

	depth = fmt->depth[LUMA_PLANE];

	/*
	 * The line stride needs to be even is even.
	 * Special case is with YUV422 interleaved format an even number
	 * of pixels is required also.
	 */
	depth_bytes = depth >> 3;

	w_align = 0;
	if ((depth_bytes == 3) || (depth_bytes == 1))
		w_align = 1;
	else if ((depth_bytes == 2) &&
		 (fmt->fourcc == V4L2_PIX_FMT_YUYV ||
		  fmt->fourcc == V4L2_PIX_FMT_UYVY))
		w_align = 1;

	v4l_bound_align_image(&pix->width, MIN_W, MAX_W, w_align,
			      &pix->height, MIN_H, MAX_H, H_ALIGN,
			      S_ALIGN);
	pix->num_planes = fmt->coplanar ? 2 : 1;
	pix->pixelformat = fmt->fourcc;

	/* Probably need something better here */
	if (!pix->colorspace) {
		if (fmt->fourcc == V4L2_PIX_FMT_RGB24 ||
		    fmt->fourcc == V4L2_PIX_FMT_BGR24 ||
		    fmt->fourcc == V4L2_PIX_FMT_RGB32 ||
		    fmt->fourcc == V4L2_PIX_FMT_BGR32) {
			pix->colorspace = V4L2_COLORSPACE_SRGB;
		} else {
			if (pix->height > 1280)	/* HD */
				pix->colorspace = V4L2_COLORSPACE_REC709;
			else			/* SD */
				pix->colorspace = V4L2_COLORSPACE_SMPTE170M;
		}
	}

	memset(pix->reserved, 0, sizeof(pix->reserved));
	for (i = 0; i < pix->num_planes; i++) {
		plane_fmt = &pix->plane_fmt[i];
		depth = fmt->depth[i];

		if (i == LUMA_PLANE)
			plane_fmt->bytesperline = (pix->width * depth) >> 3;
		else
			plane_fmt->bytesperline = pix->width;

		plane_fmt->sizeimage =
				(pix->height * pix->width * depth) >> 3;

		memset(plane_fmt->reserved, 0, sizeof(plane_fmt->reserved));
	}

	return 0;
}

static int __wbm2m_s_fmt(struct wbm2m_ctx *ctx, struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	struct v4l2_plane_pix_format *plane_fmt;
	struct wbm2m_q_data *q_data;
	struct vb2_queue *vq;
	int i;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	if (vb2_is_busy(vq)) {
		log_err(ctx->dev, "queue busy\n");
		return -EBUSY;
	}

	q_data = get_q_data(ctx, f->type);
	if (!q_data)
		return -EINVAL;

	q_data->fmt		= find_format(f);
	q_data->width		= pix->width;
	q_data->height		= pix->height;
	q_data->colorspace	= pix->colorspace;
	q_data->field		= pix->field;

	for (i = 0; i < pix->num_planes; i++) {
		plane_fmt = &pix->plane_fmt[i];

		q_data->bytesperline[i]	= plane_fmt->bytesperline;
		q_data->sizeimage[i]	= plane_fmt->sizeimage;
	}

	q_data->c_rect.left	= 0;
	q_data->c_rect.top	= 0;
	q_data->c_rect.width	= q_data->width;
	q_data->c_rect.height	= q_data->height;

	log_dbg(ctx->dev, "Setting format for type %d, %dx%d, fmt: %s bpl_y %d",
		f->type, q_data->width, q_data->height,
		fourcc_to_str(q_data->fmt->fourcc),
		q_data->bytesperline[LUMA_PLANE]);
	if (q_data->fmt->coplanar)
		log_dbg(ctx->dev, " bpl_uv %d\n",
			q_data->bytesperline[CHROMA_PLANE]);

	return 0;
}

static int wbm2m_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	int ret;
	struct wbm2m_ctx *ctx = file2ctx(file);

	ret = wbm2m_try_fmt(file, priv, f);
	if (ret)
		return ret;

	ret = __wbm2m_s_fmt(ctx, f);
	if (ret)
		return ret;

	ctx->sequence = 0;

	return 0;
}

static int __wbm2m_try_selection(struct wbm2m_ctx *ctx,
				 struct v4l2_selection *s)
{
	struct wbm2m_q_data *q_data;
	unsigned int w_align;
	int depth_bytes;

	if ((s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) &&
	    (s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT))
		return -EINVAL;

	q_data = get_q_data(ctx, s->type);
	if (!q_data)
		return -EINVAL;

	switch (s->target) {
	case V4L2_SEL_TGT_COMPOSE:
		/*
		 * COMPOSE target is only valid for capture buffer type, return
		 * error for output buffer type
		 */
		if (s->type == V4L2_BUF_TYPE_VIDEO_OUTPUT)
			return -EINVAL;
		break;
	case V4L2_SEL_TGT_CROP:
		/*
		 * CROP target is only valid for output buffer type, return
		 * error for capture buffer type
		 */
		if (s->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
			return -EINVAL;
		break;
	/*
	 * bound and default crop/compose targets are invalid targets to
	 * try/set
	 */
	default:
		return -EINVAL;
	}

	if (s->r.top < 0 || s->r.left < 0) {
		log_err(ctx->dev, "negative values for top and left\n");
		s->r.top = 0;
		s->r.left = 0;
	}

	depth_bytes = q_data->fmt->depth[LUMA_PLANE] >> 3;

	w_align = 0;
	if ((depth_bytes == 3) || (depth_bytes == 1))
		w_align = 1;
	else if ((depth_bytes == 2) &&
		 (q_data->fmt->fourcc == V4L2_PIX_FMT_YUYV ||
		  q_data->fmt->fourcc == V4L2_PIX_FMT_UYVY))
		w_align = 1;

	v4l_bound_align_image(&s->r.width, MIN_W, q_data->width, w_align,
			      &s->r.height, MIN_H, q_data->height,
			      H_ALIGN, S_ALIGN);

	/* adjust left/top if cropping rectangle is out of bounds */
	if (s->r.left + s->r.width > q_data->width)
		s->r.left = q_data->width - s->r.width;
	if (s->r.top + s->r.height > q_data->height)
		s->r.top = q_data->height - s->r.height;

	return 0;
}

static int wbm2m_g_selection(struct file *file, void *fh,
			     struct v4l2_selection *s)
{
	struct wbm2m_ctx *ctx = file2ctx(file);
	struct wbm2m_q_data *q_data;
	bool use_c_rect = false;

	if ((s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) &&
	    (s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT))
		return -EINVAL;

	q_data = get_q_data(ctx, s->type);
	if (!q_data)
		return -EINVAL;

	switch (s->target) {
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
		if (s->type == V4L2_BUF_TYPE_VIDEO_OUTPUT)
			return -EINVAL;
		break;
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		if (s->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
			return -EINVAL;
		break;
	case V4L2_SEL_TGT_COMPOSE:
		if (s->type == V4L2_BUF_TYPE_VIDEO_OUTPUT)
			return -EINVAL;
		use_c_rect = true;
		break;
	case V4L2_SEL_TGT_CROP:
		if (s->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
			return -EINVAL;
		use_c_rect = true;
		break;
	default:
		return -EINVAL;
	}

	if (use_c_rect) {
		/*
		 * for CROP/COMPOSE target type, return c_rect params from the
		 * respective buffer type
		 */
		s->r = q_data->c_rect;
	} else {
		/*
		 * for DEFAULT/BOUNDS target type, return width and height from
		 * S_FMT of the respective buffer type
		 */
		s->r.left = 0;
		s->r.top = 0;
		s->r.width = q_data->width;
		s->r.height = q_data->height;
	}

	return 0;
}

static int wbm2m_s_selection(struct file *file, void *fh,
			     struct v4l2_selection *s)
{
	struct wbm2m_ctx *ctx = file2ctx(file);
	struct wbm2m_q_data *q_data;
	struct v4l2_selection sel = *s;
	int ret;

	ret = __wbm2m_try_selection(ctx, &sel);
	if (ret)
		return ret;

	q_data = get_q_data(ctx, sel.type);
	if (!q_data)
		return -EINVAL;

	if ((q_data->c_rect.left == sel.r.left) &&
	    (q_data->c_rect.top == sel.r.top) &&
	    (q_data->c_rect.width == sel.r.width) &&
	    (q_data->c_rect.height == sel.r.height)) {
		log_dbg(ctx->dev,
			"requested crop/compose values are already set\n");
		return 0;
	}

	q_data->c_rect = sel.r;

	ctx->sequence = 0;

	return 0;
}

static const struct v4l2_ioctl_ops wbm2m_ioctl_ops = {
	.vidioc_querycap		= wbm2m_querycap,

	.vidioc_enum_fmt_vid_cap_mplane	= wbm2m_enum_fmt,
	.vidioc_g_fmt_vid_cap_mplane	= wbm2m_g_fmt,
	.vidioc_try_fmt_vid_cap_mplane	= wbm2m_try_fmt,
	.vidioc_s_fmt_vid_cap_mplane	= wbm2m_s_fmt,

	.vidioc_enum_fmt_vid_out_mplane	= wbm2m_enum_fmt,
	.vidioc_g_fmt_vid_out_mplane	= wbm2m_g_fmt,
	.vidioc_try_fmt_vid_out_mplane	= wbm2m_try_fmt,
	.vidioc_s_fmt_vid_out_mplane	= wbm2m_s_fmt,

	.vidioc_g_selection		= wbm2m_g_selection,
	.vidioc_s_selection		= wbm2m_s_selection,

	.vidioc_reqbufs			= v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf		= v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf			= v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf			= v4l2_m2m_ioctl_dqbuf,
	.vidioc_streamon		= v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff		= v4l2_m2m_ioctl_streamoff,

	.vidioc_subscribe_event		= v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,
};

/*
 * Queue operations
 */
static int wbm2m_queue_setup(struct vb2_queue *vq,
			     const struct v4l2_format *fmt,
			     unsigned int *nbuffers, unsigned int *nplanes,
			     unsigned int sizes[], void *alloc_ctxs[])
{
	int i;
	struct wbm2m_ctx *ctx = vb2_get_drv_priv(vq);
	struct wbm2m_q_data *q_data;

	q_data = get_q_data(ctx, vq->type);

	*nplanes = q_data->fmt->coplanar ? 2 : 1;

	for (i = 0; i < *nplanes; i++) {
		sizes[i] = q_data->sizeimage[i];
		alloc_ctxs[i] = ctx->dev->alloc_ctx;
	}

	log_dbg(ctx->dev, "get %d buffer(s) of size %d\n", *nbuffers,
		sizes[LUMA_PLANE]);
	if (q_data->fmt->coplanar)
		log_dbg(ctx->dev, " and %d\n", sizes[CHROMA_PLANE]);

	return 0;
}

static int wbm2m_buf_prepare(struct vb2_buffer *vb)
{
	struct wbm2m_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct wbm2m_q_data *q_data;
	int i, num_planes;

	log_dbg(ctx->dev, "type: %d\n", vb->vb2_queue->type);

	q_data = get_q_data(ctx, vb->vb2_queue->type);
	num_planes = q_data->fmt->coplanar ? 2 : 1;

	if (vb->vb2_queue->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		vb->v4l2_buf.field = V4L2_FIELD_NONE;

	for (i = 0; i < num_planes; i++) {
		if (vb2_plane_size(vb, i) < q_data->sizeimage[i]) {
			log_err(ctx->dev,
				"data will not fit into plane (%lu < %lu)\n",
				vb2_plane_size(vb, i),
				(long)q_data->sizeimage[i]);
			return -EINVAL;
		}
	}

	for (i = 0; i < num_planes; i++)
		vb2_set_plane_payload(vb, i, q_data->sizeimage[i]);

	if (num_planes) {
		if (vb->v4l2_planes[0].m.fd ==
		    vb->v4l2_planes[1].m.fd) {
			/*
			 * So it appears we are in a single memory buffer
			 * with 2 plane case. Then we need to also set the
			 * data_offset properly
			 */
			vb->v4l2_planes[1].data_offset =
				vb2_get_plane_payload(vb, 0);
		}
	}
	return 0;
}

static void wbm2m_buf_queue(struct vb2_buffer *vb)
{
	struct wbm2m_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vb);
}

static int wbm2m_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct wbm2m_ctx *ctx = vb2_get_drv_priv(q);
	struct omap_drm_private *priv = ctx->dev->drm_dev->dev_private;

	log_dbg(ctx->dev, "queue: %s\n",
		V4L2_TYPE_IS_OUTPUT(q->type) ? "OUTPUT" : "CAPTURE");

	ctx->sequence = 0;

	priv->dispc_ops->runtime_get();

	return 0;
}

static void wbm2m_stop_streaming(struct vb2_queue *q)
{
	struct wbm2m_ctx *ctx = vb2_get_drv_priv(q);
	struct omap_drm_private *priv = ctx->dev->drm_dev->dev_private;
	struct vb2_buffer *vb;
	unsigned long flags;

	log_dbg(ctx->dev, "queue: %s\n",
		V4L2_TYPE_IS_OUTPUT(q->type) ? "OUTPUT" : "CAPTURE");

	for (;;) {
		if (V4L2_TYPE_IS_OUTPUT(q->type))
			vb = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		else
			vb = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
		if (!vb)
			break;
		spin_lock_irqsave(&ctx->dev->lock, flags);
		v4l2_m2m_buf_done(vb, VB2_BUF_STATE_ERROR);
		spin_unlock_irqrestore(&ctx->dev->lock, flags);
	}

	/*
	 * Cleanup the in-transit vb2 buffers that have been
	 * removed from their respective queue already but for
	 * which procecessing has not been completed yet.
	 */
	if (V4L2_TYPE_IS_OUTPUT(q->type)) {
		/*
		 * Vayu errata i829 (Reusing Pipe Connected To Writeback
		 * Pipeline On The Fly To An Active Panel)
		 */
		priv->dispc_ops->ovl_enable(omap_plane_id(ctx->dev->plane),
					    false);
		priv->dispc_ops->ovl_enable(OMAP_DSS_WB, true);
		priv->dispc_ops->ovl_enable(OMAP_DSS_WB, false);

		if (ctx->src_vb) {
			spin_lock_irqsave(&ctx->dev->lock, flags);
			v4l2_m2m_buf_done(ctx->src_vb, VB2_BUF_STATE_ERROR);
			ctx->src_vb = NULL;
			spin_unlock_irqrestore(&ctx->dev->lock, flags);
		}
	} else {
		if (ctx->dst_vb) {
			spin_lock_irqsave(&ctx->dev->lock, flags);

			v4l2_m2m_buf_done(ctx->dst_vb, VB2_BUF_STATE_ERROR);
			ctx->dst_vb = NULL;
			spin_unlock_irqrestore(&ctx->dev->lock, flags);
		}
	}

	priv->dispc_ops->runtime_put();
}

static struct vb2_ops wbm2m_qops = {
	.queue_setup	 = wbm2m_queue_setup,
	.buf_prepare	 = wbm2m_buf_prepare,
	.buf_queue	 = wbm2m_buf_queue,
	.wait_prepare	 = vb2_ops_wait_prepare,
	.wait_finish	 = vb2_ops_wait_finish,
	.start_streaming = wbm2m_start_streaming,
	.stop_streaming  = wbm2m_stop_streaming,
};

static int queue_init(void *priv, struct vb2_queue *src_vq,
		      struct vb2_queue *dst_vq)
{
	struct wbm2m_ctx *ctx = priv;
	struct wbm2m_dev *dev = ctx->dev;
	int ret;

	memset(src_vq, 0, sizeof(*src_vq));
	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->ops = &wbm2m_qops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &dev->dev_mutex;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	memset(dst_vq, 0, sizeof(*dst_vq));
	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->ops = &wbm2m_qops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &dev->dev_mutex;

	return vb2_queue_init(dst_vq);
}

/*
 * File operations
 */
static int wbm2m_open(struct file *file)
{
	struct wbm2m_dev *dev = video_drvdata(file);
	struct wbm2m_q_data *s_q_data;
	struct v4l2_ctrl_handler *hdl;
	struct wbm2m_ctx *ctx;
	int ret;

	log_dbg(dev, "enter\n");

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dev = dev;

	if (mutex_lock_interruptible(&dev->dev_mutex)) {
		ret = -ERESTARTSYS;
		goto free_ctx;
	}

	v4l2_fh_init(&ctx->fh, video_devdata(file));
	file->private_data = &ctx->fh;

	hdl = &ctx->hdl;
	v4l2_ctrl_handler_init(hdl, 1);
	ctx->fh.ctrl_handler = hdl;

	s_q_data = &ctx->q_data[Q_DATA_SRC];
	s_q_data->fmt = &wbm2m_formats[1];
	s_q_data->width = 1920;
	s_q_data->height = 1080;
	s_q_data->bytesperline[LUMA_PLANE] = (s_q_data->width *
			s_q_data->fmt->depth[LUMA_PLANE]) >> 3;
	s_q_data->sizeimage[LUMA_PLANE] = (s_q_data->bytesperline[LUMA_PLANE] *
			s_q_data->height);
	s_q_data->colorspace = V4L2_COLORSPACE_REC709;
	s_q_data->field = V4L2_FIELD_NONE;
	s_q_data->c_rect.left = 0;
	s_q_data->c_rect.top = 0;
	s_q_data->c_rect.width = s_q_data->width;
	s_q_data->c_rect.height = s_q_data->height;

	ctx->q_data[Q_DATA_DST] = *s_q_data;

	ctx->sequence = 0;

	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(dev->m2m_dev, ctx, &queue_init);

	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto exit_fh;
	}

	v4l2_fh_add(&ctx->fh);

	if (atomic_inc_return(&dev->num_instances) == 1) {
		log_dbg(dev, "first instance created\n");

		drm_modeset_lock_all(dev->drm_dev);
		dev->plane = omap_plane_reserve_wb(dev->drm_dev);
		drm_modeset_unlock_all(dev->drm_dev);

		if (!dev->plane) {
			log_dbg(dev, "Could not reserve plane!\n");
			ret = -EBUSY;
			goto free_fh;
		}
	}

	log_dbg(dev, "created instance %pa, m2m_ctx: %pa\n",
		&ctx, &ctx->fh.m2m_ctx);

	mutex_unlock(&dev->dev_mutex);

	return 0;

free_fh:
	v4l2_fh_del(&ctx->fh);
exit_fh:
	v4l2_ctrl_handler_free(hdl);
	v4l2_fh_exit(&ctx->fh);
	mutex_unlock(&dev->dev_mutex);
free_ctx:
	kfree(ctx);
	return ret;
}

static int wbm2m_release(struct file *file)
{
	struct wbm2m_dev *dev = video_drvdata(file);
	struct wbm2m_ctx *ctx = file2ctx(file);

	log_dbg(dev, "releasing instance %pa\n", &ctx);

	mutex_lock(&dev->dev_mutex);

	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	v4l2_ctrl_handler_free(&ctx->hdl);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);

	kfree(ctx);

	if (atomic_dec_return(&dev->num_instances) == 0) {
		log_dbg(dev, "last instance released\n");

		drm_modeset_lock_all(dev->drm_dev);
		omap_plane_release_wb(dev->plane);
		drm_modeset_unlock_all(dev->drm_dev);
	}

	mutex_unlock(&dev->dev_mutex);

	return 0;
}

static const struct v4l2_file_operations wbm2m_fops = {
	.owner		= THIS_MODULE,
	.open		= wbm2m_open,
	.release	= wbm2m_release,
	.poll		= v4l2_m2m_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= v4l2_m2m_fop_mmap,
};

static struct video_device wbm2m_videodev = {
	.name		= WBM2M_MODULE_NAME,
	.fops		= &wbm2m_fops,
	.ioctl_ops	= &wbm2m_ioctl_ops,
	.minor		= -1,
	.release	= video_device_release_empty,
	.vfl_dir	= VFL_DIR_M2M,
};

static struct v4l2_m2m_ops m2m_ops = {
	.device_run	= device_run,
	.job_ready	= job_ready,
	.job_abort	= job_abort,
	.lock		= wbm2m_lock,
	.unlock		= wbm2m_unlock,
};

int wbm2m_init(struct drm_device *drmdev)
{
	struct omap_drm_private *priv = drmdev->dev_private;
	struct wbm2m_dev *dev;
	struct video_device *vfd;
	int ret, irq;

	irq = 0;

	dev = devm_kzalloc(drmdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->drm_dev = drmdev;

	spin_lock_init(&dev->lock);

	snprintf(dev->v4l2_dev.name, sizeof(dev->v4l2_dev.name),
		 "%s", WBM2M_MODULE_NAME);
	ret = v4l2_device_register(drmdev->dev, &dev->v4l2_dev);
	if (ret)
		return ret;

	priv->wb_private = dev;

	atomic_set(&dev->num_instances, 0);
	mutex_init(&dev->dev_mutex);

	dev->alloc_ctx = vb2_dma_contig_init_ctx(drmdev->dev);
	if (IS_ERR(dev->alloc_ctx)) {
		log_err(dev, "Failed to alloc vb2 context\n");
		ret = PTR_ERR(dev->alloc_ctx);
		goto v4l2_dev_unreg;
	}

	dev->m2m_dev = v4l2_m2m_init(&m2m_ops);
	if (IS_ERR(dev->m2m_dev)) {
		log_err(dev, "Failed to init mem2mem device\n");
		ret = PTR_ERR(dev->m2m_dev);
		goto rel_ctx;
	}

	dev->wb_irq.irqmask = DISPC_IRQ_FRAMEDONEWB |
			      DISPC_IRQ_WBBUFFEROVERFLOW |
			      DISPC_IRQ_WBUNCOMPLETEERROR;
	dev->wb_irq.irq = wbm2m_irq;
	omap_irq_register(drmdev, &dev->wb_irq);

	vfd = &dev->vfd;
	*vfd = wbm2m_videodev;
	vfd->lock = &dev->dev_mutex;
	vfd->v4l2_dev = &dev->v4l2_dev;

	ret = video_register_device(vfd, VFL_TYPE_GRABBER, 10);
	if (ret) {
		log_err(dev, "Failed to register video device\n");
		goto rel_m2m;
	}

	video_set_drvdata(vfd, dev);
	snprintf(vfd->name, sizeof(vfd->name), "%s", wbm2m_videodev.name);
	log_info(dev, "Device registered as %s\n",
		 video_device_node_name(vfd));

	return 0;

rel_m2m:
	omap_irq_unregister(drmdev, &dev->wb_irq);
	v4l2_m2m_release(dev->m2m_dev);
rel_ctx:
	vb2_dma_contig_cleanup_ctx(dev->alloc_ctx);
v4l2_dev_unreg:
	v4l2_device_unregister(&dev->v4l2_dev);

	return ret;
}

void wbm2m_cleanup(struct drm_device *drmdev)
{
	struct omap_drm_private *priv = drmdev->dev_private;
	struct wbm2m_dev *dev = priv->wb_private;

	log_dbg(dev, "Cleanup WB\n");

	omap_irq_unregister(drmdev, &dev->wb_irq);

	v4l2_m2m_release(dev->m2m_dev);
	video_unregister_device(&dev->vfd);
	v4l2_device_unregister(&dev->v4l2_dev);
	vb2_dma_contig_cleanup_ctx(dev->alloc_ctx);
}
