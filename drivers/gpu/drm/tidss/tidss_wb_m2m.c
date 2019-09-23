// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Texas Instruments Incorporated -  http://www.ti.com/
 * Benoit Parrot <bparrot@ti.com>
 *
 * Based on the virtual v4l2-mem2mem example device
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <media/v4l2-mem2mem.h>

#include "tidss_wb.h"

MODULE_DESCRIPTION("TI TIDSS WB M2M driver");
MODULE_AUTHOR("Benoit Parrot <bparrot@ti.com>");
MODULE_LICENSE("GPL v2");

/*
 * M2M devices get 2 queues.
 * Return the queue given the type.
 */
static struct wb_q_data *get_q_data(struct wbm2m_ctx *ctx,
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

static bool wbm2m_convert(struct wbm2m_dev *dev, struct drm_plane *plane,
			  const struct drm_plane_state *in_state,
			  const struct drm_plane_state *out_state)
{
	struct tidss_device *tidss = dev->dev->drm_dev->dev_private;
	struct tidss_plane *tplane = to_tidss_plane(plane);
	u32 hw_plane = tplane->hw_plane_id;
	u32 hw_videoport = tidss->dispc_ops->wb_get_reserved_ovr(tidss->dispc);
	int r;

	/* configure input */

	r = tidss->dispc_ops->plane_setup(tidss->dispc, hw_plane,
					  in_state, hw_videoport);

	if (r) {
		log_err(dev, "Failed to setup plane %d\n", hw_plane);
		tidss->dispc_ops->plane_enable(tidss->dispc, hw_plane,
					       false);
		return false;
	}

	tidss->dispc_ops->plane_enable(tidss->dispc, hw_plane, true);

	/* configure output */

	r = tidss->dispc_ops->wb_setup(tidss->dispc, out_state,
				       true, hw_plane, hw_videoport);
	if (r) {
		log_err(dev, "Failed to setup wb\n");
		tidss->dispc_ops->plane_enable(tidss->dispc, hw_plane,
					       false);
		return false;
	}

	tidss->dispc_ops->wb_enable(tidss->dispc, true);

	return true;
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
	 * When called by m2m framework, this will always satisfy, but when
	 * called from tidss_wbm2m_irq, this might fail.
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

static void prepare_plane_state(struct drm_plane_state *state,
				struct drm_framebuffer *fb,
				int v4l2_pixelformat,
				struct drm_gem_cma_object gem_obj[])
{
	int drm_fourcc = tidss_wb_fourcc_v4l2_to_drm(v4l2_pixelformat);

	memset(state, 0, sizeof(*state));
	memset(fb, 0, sizeof(*fb));
	memset(&gem_obj[0], 0, sizeof(gem_obj[0]));
	memset(&gem_obj[1], 0, sizeof(gem_obj[1]));

	state->fb = fb;
	state->fb->format = drm_format_info(drm_fourcc);
	state->fb->obj[0] = &gem_obj[0].base;
	state->fb->obj[1] = &gem_obj[1].base;
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
	struct wb_q_data *d_q_data = &ctx->q_data[Q_DATA_DST];
	struct wb_q_data *s_q_data = &ctx->q_data[Q_DATA_SRC];
	struct vb2_v4l2_buffer *s_vb, *d_vb;
	struct v4l2_pix_format_mplane *spix, *dpix;
	struct v4l2_rect *srect, *drect;
	bool ok;

	s_vb = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	d_vb = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);

	/* fill source info */
	srect = &s_q_data->c_rect;
	spix = &s_q_data->format.fmt.pix_mp;
	prepare_plane_state(&ctx->s_state, &ctx->s_fb,
			    spix->pixelformat, ctx->s_cma_gem_obj);

	ctx->s_state.src_w = srect->width << 16;
	ctx->s_state.src_h = srect->height << 16;
	ctx->s_state.src_x = srect->left << 16;
	ctx->s_state.src_y = srect->top << 16;
	ctx->s_state.crtc_w = srect->width;
	ctx->s_state.crtc_h = srect->height;
	ctx->s_state.fb->pitches[0] = spix->plane_fmt[0].bytesperline;
	ctx->s_state.alpha = DRM_BLEND_ALPHA_OPAQUE;
	ctx->s_state.color_encoding = DRM_COLOR_YCBCR_BT601;
	ctx->s_state.color_range = DRM_COLOR_YCBCR_FULL_RANGE;

	ctx->s_cma_gem_obj[0].paddr =
			vb2_dma_contig_plane_dma_addr(&s_vb->vb2_buf, 0);
	if (spix->num_planes == 2) {
		ctx->s_cma_gem_obj[1].paddr =
			vb2_dma_contig_plane_dma_addr(&s_vb->vb2_buf, 1);
		ctx->s_state.fb->pitches[1] = spix->plane_fmt[1].bytesperline;
	} else if (spix->pixelformat == V4L2_PIX_FMT_NV12) {
		ctx->s_cma_gem_obj[1].paddr = ctx->s_cma_gem_obj[0].paddr +
			(spix->plane_fmt[0].bytesperline * spix->height);
		ctx->s_state.fb->pitches[1] = spix->plane_fmt[0].bytesperline;
	}
	if (!ctx->s_cma_gem_obj[0].paddr) {
		log_err(dev,
			"acquiring source buffer(%d) dma_addr failed\n",
			(&s_vb->vb2_buf)->index);
		return;
	}

	log_dbg(dev, "SRC: ctx %pa buf_index %d %dx%d, pitches: %d, cpp: %d, sw %d\n",
		&ctx, (&s_vb->vb2_buf)->index,
		ctx->s_state.crtc_w, ctx->s_state.crtc_h,
		ctx->s_state.fb->pitches[0], ctx->s_state.fb->format->cpp[0],
		ctx->s_state.fb->pitches[0] / ctx->s_state.fb->format->cpp[0]);
	if (spix->num_planes == 2 || spix->pixelformat == V4L2_PIX_FMT_NV12) {
		log_dbg(dev, "SRC: pitches_uv: %d, cpp_uv: %d, sw_uv %d\n",
			ctx->s_state.fb->pitches[1],
			ctx->s_state.fb->format->cpp[1],
			ctx->s_state.fb->pitches[1] /
				ctx->s_state.fb->format->cpp[1]);
	}

	/* fill WB info */
	drect = &d_q_data->c_rect;
	dpix = &d_q_data->format.fmt.pix_mp;
	prepare_plane_state(&ctx->d_state, &ctx->d_fb,
			    dpix->pixelformat, ctx->d_cma_gem_obj);

	ctx->d_state.src_w = ctx->s_state.crtc_w << 16;
	ctx->d_state.src_h = ctx->s_state.crtc_h << 16;
	ctx->d_state.src_x = drect->left << 16;
	ctx->d_state.src_y = drect->top << 16;
	ctx->d_state.crtc_w = drect->width;
	ctx->d_state.crtc_h = drect->height;
	ctx->d_state.fb->pitches[0] = dpix->plane_fmt[0].bytesperline;
	ctx->d_state.alpha = DRM_BLEND_ALPHA_OPAQUE;
	ctx->d_state.color_encoding = DRM_COLOR_YCBCR_BT601;
	ctx->d_state.color_range = DRM_COLOR_YCBCR_FULL_RANGE;

	ctx->d_cma_gem_obj[0].paddr =
			vb2_dma_contig_plane_dma_addr(&d_vb->vb2_buf, 0);
	if (dpix->num_planes == 2) {
		ctx->d_cma_gem_obj[1].paddr =
			vb2_dma_contig_plane_dma_addr(&d_vb->vb2_buf, 1);
		ctx->d_state.fb->pitches[1] = dpix->plane_fmt[1].bytesperline;
	} else if (dpix->pixelformat == V4L2_PIX_FMT_NV12) {
		ctx->d_cma_gem_obj[1].paddr = ctx->d_cma_gem_obj[0].paddr +
			(dpix->plane_fmt[0].bytesperline * dpix->height);
		ctx->d_state.fb->pitches[1] = dpix->plane_fmt[0].bytesperline;
	}
	if (!ctx->d_cma_gem_obj[0].paddr) {
		log_err(dev,
			"acquiring destination buffer(%d) dma_addr failed\n",
			(&d_vb->vb2_buf)->index);
		return;
	}

	log_dbg(dev, "DST: ctx %pa buf_index %d %dx%d, pitches: %d, cpp: %d, sw %d\n",
		&ctx, (&d_vb->vb2_buf)->index,
		ctx->d_state.crtc_w, ctx->d_state.crtc_h,
		ctx->d_state.fb->pitches[0], ctx->d_state.fb->format->cpp[0],
		ctx->d_state.fb->pitches[0] / ctx->d_state.fb->format->cpp[0]);
	if (dpix->num_planes == 2 || dpix->pixelformat == V4L2_PIX_FMT_NV12) {
		log_dbg(dev, "DST: pitches_uv: %d, cpp_uv: %d, sw_uv %d\n",
			ctx->d_state.fb->pitches[1],
			ctx->d_state.fb->format->cpp[1],
			ctx->d_state.fb->pitches[1] /
				ctx->d_state.fb->format->cpp[1]);
	}

	ok = wbm2m_convert(dev, ctx->dev->plane,
			   (const struct drm_plane_state *)&ctx->s_state,
			   (const struct drm_plane_state *)&ctx->d_state);
	if (!ok) {
		log_err(dev,
			"Conversion setup failed, check source and destination parameters\n"
			);
		log_err(dev, "\tSRC: %dx%d, fmt: %4.4s sw %d\n",
			ctx->s_state.crtc_w, ctx->s_state.crtc_h,
			(char *)&spix->pixelformat,
			ctx->s_state.fb->pitches[0] /
				ctx->s_state.fb->format->cpp[0]);
		log_err(dev, "\tDST: %dx%d, fmt: %4.4s sw %d\n",
			ctx->d_state.crtc_w, ctx->d_state.crtc_h,
			(char *)&dpix->pixelformat,
			ctx->d_state.fb->pitches[0] /
				ctx->d_state.fb->format->cpp[0]);
		return;
	}
}

void tidss_wbm2m_irq(struct wbm2m_dev *wbm2m, u64 irqstatus)
{
	struct wbm2m_ctx *ctx;
	struct vb2_v4l2_buffer *s_vb, *d_vb;
	unsigned long flags;

	if (irqstatus & DSS_IRQ_DEVICE_WBBUFFEROVERFLOW)
		log_err(wbm2m, "WB: UNDERFLOW\n");

	if (irqstatus & DSS_IRQ_DEVICE_WBUNCOMPLETEERROR)
		log_err(wbm2m, "WB: UNCOMPLETEERROR\n");

	if (irqstatus & DSS_IRQ_DEVICE_WBSECURITYVIOLATION)
		log_err(wbm2m, "WB: SECURITYVIOLATION\n");

	if (irqstatus & DSS_IRQ_DEVICE_WBSYNC)
		log_dbg(wbm2m, "WB: SYNC\n");

	/* If DISPC_IRQ_FRAMEDONEWB is not set then we are done */
	if (!(irqstatus & DSS_IRQ_DEVICE_FRAMEDONEWB))
		goto handled;

	log_dbg(wbm2m, "WB: FRAMEDONE\n");

	ctx = v4l2_m2m_get_curr_priv(wbm2m->m2m_dev);
	if (!ctx) {
		log_err(wbm2m, "instance released before end of transaction\n");
		goto handled;
	}

	log_dbg(ctx->dev, "ctx %pa\n", &ctx);

	s_vb = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
	d_vb = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
	if (!s_vb || !d_vb) {
		log_err(wbm2m, "source or dest vb pointer is NULL!!");
		goto handled;
	}

	d_vb->flags = s_vb->flags;

	d_vb->vb2_buf.timestamp = s_vb->vb2_buf.timestamp;
	if (s_vb->flags & V4L2_BUF_FLAG_TIMECODE)
		d_vb->timecode = s_vb->timecode;

	d_vb->sequence = ctx->sequence;
	s_vb->sequence = ctx->sequence;
	log_dbg(wbm2m, "ctx %pa sequence %d\n",
		&ctx, ctx->sequence);

	d_vb->field = V4L2_FIELD_NONE;
	ctx->sequence++;

	spin_lock_irqsave(&wbm2m->lock, flags);

	v4l2_m2m_buf_done(s_vb, VB2_BUF_STATE_DONE);
	v4l2_m2m_buf_done(d_vb, VB2_BUF_STATE_DONE);

	spin_unlock_irqrestore(&wbm2m->lock, flags);

	v4l2_m2m_job_finish(wbm2m->m2m_dev, ctx->fh.m2m_ctx);
handled:
	return;
}

/*
 * video ioctls
 */
static int wbm2m_querycap(struct file *file, void *priv,
			  struct v4l2_capability *cap)
{
	struct wbm2m_ctx *ctx = file->private_data;

	strscpy(cap->driver, WBM2M_MODULE_NAME, sizeof(cap->driver));
	strscpy(cap->card, WBM2M_MODULE_NAME, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 ctx->dev->v4l2_dev.name);

	return 0;
}

static int wbm2m_enum_fmt(struct file *file, void *priv,
			  struct v4l2_fmtdesc *f)
{
	if (f->index >= tidss_num_wb_formats)
		return -EINVAL;

	f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	f->pixelformat = tidss_wb_formats[f->index].fourcc;
	return 0;
}

static int wbm2m_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct wbm2m_ctx *ctx = file->private_data;
	struct vb2_queue *vq;
	struct wb_q_data *q_data;
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	q_data = get_q_data(ctx, f->type);
	if (!q_data)
		return -EINVAL;

	*f = q_data->format;

	if (!V4L2_TYPE_IS_OUTPUT(f->type)) {
		struct wb_q_data *s_q_data;

		/* get colorspace from the source queue */
		s_q_data = get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);

		f->fmt.pix_mp.colorspace =
			s_q_data->format.fmt.pix_mp.colorspace;
	}

	log_dbg(ctx->dev, "ctx %pa type %d, %dx%d, fmt: %4.4s bpl_y %d",
		&ctx, f->type, pix->width, pix->height,
		(char *)&pix->pixelformat,
		pix->plane_fmt[LUMA_PLANE].bytesperline);
	if (pix->num_planes == 2)
		log_dbg(ctx->dev, " bpl_uv %d\n",
			pix->plane_fmt[CHROMA_PLANE].bytesperline);

	return 0;
}

static int wbm2m_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct wbm2m_ctx *ctx = file->private_data;
	const struct wb_fmt *fmt = tidss_wb_find_format(f);
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	struct v4l2_plane_pix_format *plane_fmt;
	unsigned int w_align;
	int i, depth, depth_bytes;

	if (!fmt) {
		log_dbg(ctx->dev, "Fourcc format (0x%08x) invalid.\n",
			pix->pixelformat);
		fmt = __tidss_wb_find_format(V4L2_PIX_FMT_YUYV);
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
			plane_fmt->bytesperline = pix->width * depth / 8;
		else
			plane_fmt->bytesperline = pix->width;

		plane_fmt->sizeimage = (pix->height * pix->width *
					depth) / 8;

		if (fmt->fourcc == V4L2_PIX_FMT_NV12) {
			/*
			 * Since we are using a single plane buffer
			 * we need to adjust the reported sizeimage
			 * to include the colocated UV part.
			 */
			plane_fmt->sizeimage += (pix->height / 2 *
				plane_fmt->bytesperline);
		}

		memset(plane_fmt->reserved, 0, sizeof(plane_fmt->reserved));
	}

	return 0;
}

static int __wbm2m_s_fmt(struct wbm2m_ctx *ctx, struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	struct wb_q_data *q_data;
	struct vb2_queue *vq;

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

	q_data->fmt = tidss_wb_find_format(f);
	q_data->format = *f;

	q_data->c_rect.left	= 0;
	q_data->c_rect.top	= 0;
	q_data->c_rect.width	= pix->width;
	q_data->c_rect.height	= pix->height;

	log_dbg(ctx->dev, "ctx %pa type %d, %dx%d, fmt: %4.4s bpl_y %d",
		&ctx, f->type, pix->width, pix->height,
		(char *)&pix->pixelformat,
		pix->plane_fmt[LUMA_PLANE].bytesperline);
	if (pix->num_planes == 2)
		log_dbg(ctx->dev, " bpl_uv %d\n",
			pix->plane_fmt[CHROMA_PLANE].bytesperline);

	return 0;
}

static int wbm2m_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	int ret;
	struct wbm2m_ctx *ctx = file->private_data;

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
	struct wb_q_data *q_data;
	struct v4l2_pix_format_mplane *pix;
	unsigned int w_align;
	int depth_bytes;

	if ((s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) &&
	    (s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT))
		return -EINVAL;

	q_data = get_q_data(ctx, s->type);
	if (!q_data)
		return -EINVAL;

	pix = &q_data->format.fmt.pix_mp;

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
		 (pix->pixelformat == V4L2_PIX_FMT_YUYV ||
		  pix->pixelformat == V4L2_PIX_FMT_UYVY))
		w_align = 1;

	v4l_bound_align_image(&s->r.width, MIN_W, pix->width, w_align,
			      &s->r.height, MIN_H, pix->height,
			      H_ALIGN, S_ALIGN);

	/* adjust left/top if cropping rectangle is out of bounds */
	if (s->r.left + s->r.width > pix->width)
		s->r.left = pix->width - s->r.width;
	if (s->r.top + s->r.height > pix->height)
		s->r.top = pix->height - s->r.height;

	return 0;
}

static int wbm2m_g_selection(struct file *file, void *fh,
			     struct v4l2_selection *s)
{
	struct wbm2m_ctx *ctx = file->private_data;
	struct wb_q_data *q_data;
	struct v4l2_pix_format_mplane *pix;
	bool use_c_rect = false;

	if ((s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) &&
	    (s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT))
		return -EINVAL;

	q_data = get_q_data(ctx, s->type);
	if (!q_data)
		return -EINVAL;

	pix = &q_data->format.fmt.pix_mp;

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
		s->r.width = pix->width;
		s->r.height = pix->height;
	}

	return 0;
}

static int wbm2m_s_selection(struct file *file, void *fh,
			     struct v4l2_selection *s)
{
	struct wbm2m_ctx *ctx = file->private_data;
	struct wb_q_data *q_data;
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
			"type: %d, requested crop/compose values are already set\n",
			sel.type);
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
	.vidioc_create_bufs		= v4l2_m2m_ioctl_create_bufs,
	.vidioc_prepare_buf		= v4l2_m2m_ioctl_prepare_buf,
	.vidioc_querybuf		= v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf			= v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf			= v4l2_m2m_ioctl_dqbuf,
	.vidioc_expbuf			= v4l2_m2m_ioctl_expbuf,
	.vidioc_streamon		= v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff		= v4l2_m2m_ioctl_streamoff,

	.vidioc_subscribe_event		= v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,
};

/*
 * Queue operations
 */
static int wbm2m_queue_setup(struct vb2_queue *vq,
			     unsigned int *nbuffers, unsigned int *nplanes,
			     unsigned int sizes[], struct device *alloc_devs[])
{
	int i;
	struct wbm2m_ctx *ctx = vb2_get_drv_priv(vq);
	struct wb_q_data *q_data;

	q_data = get_q_data(ctx, vq->type);
	if (!q_data)
		return -EINVAL;

	*nplanes = q_data->format.fmt.pix_mp.num_planes;

	for (i = 0; i < *nplanes; i++)
		sizes[i] = q_data->format.fmt.pix_mp.plane_fmt[i].sizeimage;

	log_dbg(ctx->dev, "get %d buffer(s) of size %d\n", *nbuffers,
		sizes[LUMA_PLANE]);
	if (*nplanes == 2)
		log_dbg(ctx->dev, " and %d\n", sizes[CHROMA_PLANE]);

	return 0;
}

static int wbm2m_buf_prepare(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct wbm2m_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct wb_q_data *q_data;
	struct v4l2_pix_format_mplane *mp;
	int i, num_planes;

	log_dbg(ctx->dev, "type: %d\n", vb->vb2_queue->type);

	q_data = get_q_data(ctx, vb->vb2_queue->type);
	if (!q_data)
		return -EINVAL;

	if (vb->vb2_queue->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		vbuf->field = V4L2_FIELD_NONE;

	num_planes = q_data->format.fmt.pix_mp.num_planes;

	for (i = 0; i < num_planes; i++) {
		mp = &q_data->format.fmt.pix_mp;

		if (vb->vb2_queue->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
			if (vb2_get_plane_payload(vb, i) <
			    mp->plane_fmt[i].sizeimage) {
				log_dbg(ctx->dev,
					"the payload is too small for plane plane (%lu < %lu)\n",
					vb2_get_plane_payload(vb, i),
					(long)mp->plane_fmt[i].sizeimage);
				return -EINVAL;
			}
		} else {
			if (vb2_plane_size(vb, i) <
			    mp->plane_fmt[i].sizeimage) {
				log_dbg(ctx->dev,
					"data will not fit into plane (%lu < %lu)\n",
					vb2_plane_size(vb, i),
					(long)mp->plane_fmt[i].sizeimage);
				return -EINVAL;
			}
			vb2_set_plane_payload(vb, i,
					      mp->plane_fmt[i].sizeimage);
		}
	}

	return 0;
}

static void wbm2m_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct wbm2m_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	log_dbg(ctx->dev, "queueing buffer: %s index %d\n",
		V4L2_TYPE_IS_OUTPUT(vb->type) ? "OUTPUT" : "CAPTURE",
		vb->index);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
}

static int wbm2m_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct wbm2m_ctx *ctx = vb2_get_drv_priv(q);
	struct tidss_device *tidss = ctx->dev->dev->drm_dev->dev_private;

	log_dbg(ctx->dev, "ctx %pa queue: %s\n", &ctx,
		V4L2_TYPE_IS_OUTPUT(q->type) ? "OUTPUT" : "CAPTURE");

	ctx->sequence = 0;

	tidss->dispc_ops->runtime_get(tidss->dispc);
	atomic_inc(&ctx->dev->dev->irq_enabled);

	return 0;
}

static void wbm2m_stop_streaming(struct vb2_queue *q)
{
	struct wbm2m_ctx *ctx = vb2_get_drv_priv(q);
	struct tidss_device *tidss = ctx->dev->dev->drm_dev->dev_private;
	struct vb2_v4l2_buffer *vb;
	unsigned long flags;

	log_dbg(ctx->dev, "ctx %pa queue: %s\n", &ctx,
		V4L2_TYPE_IS_OUTPUT(q->type) ? "OUTPUT" : "CAPTURE");

	atomic_dec(&ctx->dev->dev->irq_enabled);

	for (;;) {
		if (V4L2_TYPE_IS_OUTPUT(q->type))
			vb = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		else
			vb = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
		if (!vb)
			break;
		log_dbg(ctx->dev, "returning from queue: buffer index %d\n",
			vb->vb2_buf.index);
		spin_lock_irqsave(&ctx->dev->lock, flags);
		v4l2_m2m_buf_done(vb, VB2_BUF_STATE_ERROR);
		spin_unlock_irqrestore(&ctx->dev->lock, flags);
	}

	/*
	 * Cleanup the in-transit vb2 buffers that have been
	 * removed from their respective queue already but for
	 * which processing has not been completed yet.
	 */
	if (V4L2_TYPE_IS_OUTPUT(q->type)) {
		struct tidss_plane *tplane = to_tidss_plane(ctx->dev->plane);

		tidss->dispc_ops->plane_enable(tidss->dispc,
					       tplane->hw_plane_id, false);
		tidss->dispc_ops->wb_enable(tidss->dispc, false);
	}

	tidss->dispc_ops->runtime_put(tidss->dispc);
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
	src_vq->lock = &dev->dev->lock;
	src_vq->min_buffers_needed = 1;
	src_vq->dev = dev->v4l2_dev.dev;

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
	dst_vq->lock = &dev->dev->lock;
	dst_vq->min_buffers_needed = 1;
	dst_vq->dev = dev->v4l2_dev.dev;

	return vb2_queue_init(dst_vq);
}

/*
 * File operations
 */
static int wbm2m_open(struct file *file)
{
	struct wbm2m_dev *dev = video_drvdata(file);
	struct wb_q_data *s_q_data;
	struct wbm2m_ctx *ctx;
	struct v4l2_pix_format_mplane *pix;
	int ret;

	log_dbg(dev, "enter\n");

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dev = dev;

	if (mutex_lock_interruptible(&dev->dev->lock)) {
		ret = -ERESTARTSYS;
		goto free_ctx;
	}

	if ((dev->dev->mode != TIDSS_WB_NOT_CONFIGURED) &&
	    (dev->dev->mode != TIDSS_WB_MEM2MEM_OVL)) {
		/* WB is already open for other modes */
		ret = -EBUSY;
		goto unlock;
	}

	v4l2_fh_init(&ctx->fh, video_devdata(file));
	file->private_data = ctx;

	s_q_data = &ctx->q_data[Q_DATA_SRC];
	s_q_data->fmt = __tidss_wb_find_format(V4L2_PIX_FMT_YUYV);
	pix = &s_q_data->format.fmt.pix_mp;
	pix->pixelformat = s_q_data->fmt->fourcc;
	s_q_data->format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	pix->width = 1920;
	pix->height = 1080;
	pix->plane_fmt[LUMA_PLANE].bytesperline = (pix->width *
			s_q_data->fmt->depth[LUMA_PLANE]) >> 3;
	pix->plane_fmt[LUMA_PLANE].sizeimage =
			pix->plane_fmt[LUMA_PLANE].bytesperline *
			pix->height;
	pix->num_planes = s_q_data->fmt->coplanar ? 2 : 1;
	pix->colorspace = V4L2_COLORSPACE_REC709;
	pix->field = V4L2_FIELD_NONE;
	s_q_data->c_rect.left = 0;
	s_q_data->c_rect.top = 0;
	s_q_data->c_rect.width = pix->width;
	s_q_data->c_rect.height = pix->height;

	ctx->q_data[Q_DATA_DST] = *s_q_data;
	ctx->q_data[Q_DATA_DST].format.type =
			V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

	ctx->sequence = 0;

	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(dev->m2m_dev, ctx, &queue_init);

	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto exit_fh;
	}

	v4l2_fh_add(&ctx->fh);

	if (v4l2_fh_is_singular_file(file)) {
		log_dbg(dev, "first instance created\n");

		drm_modeset_lock_all(dev->dev->drm_dev);
		dev->plane = tidss_plane_reserve_wb(dev->dev->drm_dev);
		drm_modeset_unlock_all(dev->dev->drm_dev);

		if (!dev->plane) {
			log_dbg(dev, "Could not reserve plane!\n");
			ret = -EBUSY;
			goto free_fh;
		}

		dev->dev->mode = TIDSS_WB_MEM2MEM_OVL;
	}

	log_dbg(dev, "created instance %pa, m2m_ctx: %pa\n",
		&ctx, &ctx->fh.m2m_ctx);

	mutex_unlock(&dev->dev->lock);

	return 0;

free_fh:
	v4l2_fh_del(&ctx->fh);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
exit_fh:
	v4l2_fh_exit(&ctx->fh);
unlock:
	mutex_unlock(&dev->dev->lock);
free_ctx:
	kfree(ctx);
	return ret;
}

static int wbm2m_release(struct file *file)
{
	struct wbm2m_dev *dev = video_drvdata(file);
	struct wbm2m_ctx *ctx = file->private_data;
	bool fh_singular;

	log_dbg(dev, "releasing instance %pa\n", &ctx);

	mutex_lock(&dev->dev->lock);

	/* Save the singular status before we call the clean-up helper */
	fh_singular = v4l2_fh_is_singular_file(file);

	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);

	kfree(ctx);

	if (fh_singular) {
		log_dbg(dev, "last instance released\n");

		drm_modeset_lock_all(dev->dev->drm_dev);
		tidss_plane_release_wb(dev->plane);
		drm_modeset_unlock_all(dev->dev->drm_dev);
		dev->dev->mode = TIDSS_WB_NOT_CONFIGURED;
	}

	mutex_unlock(&dev->dev->lock);

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
	.device_caps	= V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING,
};

static struct v4l2_m2m_ops m2m_ops = {
	.device_run	= device_run,
	.job_ready	= job_ready,
	.job_abort	= job_abort,
};

int tidss_wbm2m_init(struct wb_dev *dev)
{
	struct wbm2m_dev *wbm2m;
	struct video_device *vfd;
	int ret;

	if (!dev)
		return -ENOMEM;

	/* Allocate a new instance */
	wbm2m = devm_kzalloc(dev->drm_dev->dev, sizeof(*wbm2m), GFP_KERNEL);
	if (!wbm2m)
		return -ENOMEM;

	dev->m2m = wbm2m;
	wbm2m->dev = dev;

	spin_lock_init(&wbm2m->lock);

	snprintf(wbm2m->v4l2_dev.name, sizeof(wbm2m->v4l2_dev.name),
		 "%s", WBM2M_MODULE_NAME);
	ret = v4l2_device_register(dev->drm_dev->dev, &wbm2m->v4l2_dev);
	if (ret)
		return ret;

	wbm2m->m2m_dev = v4l2_m2m_init(&m2m_ops);
	if (IS_ERR(wbm2m->m2m_dev)) {
		log_err(wbm2m, "Failed to init mem2mem device\n");
		ret = PTR_ERR(wbm2m->m2m_dev);
		goto v4l2_dev_unreg;
	}

	vb2_dma_contig_set_max_seg_size(dev->drm_dev->dev, DMA_BIT_MASK(32));

	vfd = &wbm2m->vfd;
	*vfd = wbm2m_videodev;
	vfd->lock = &dev->lock;
	vfd->v4l2_dev = &wbm2m->v4l2_dev;

	ret = video_register_device(vfd, VFL_TYPE_GRABBER, 10);
	if (ret) {
		log_err(wbm2m, "Failed to register video device\n");
		goto rel_m2m;
	}

	video_set_drvdata(vfd, wbm2m);
	snprintf(vfd->name, sizeof(vfd->name), "%s", wbm2m_videodev.name);
	log_dbg(wbm2m, "Device registered as %s\n",
		video_device_node_name(vfd));

	return 0;

rel_m2m:
	vb2_dma_contig_clear_max_seg_size(dev->drm_dev->dev);
	v4l2_m2m_release(wbm2m->m2m_dev);
v4l2_dev_unreg:
	v4l2_device_unregister(&wbm2m->v4l2_dev);

	return ret;
}

void tidss_wbm2m_cleanup(struct wb_dev *dev)
{
	log_dbg(dev->m2m, "Cleanup WB M2M\n");

	vb2_dma_contig_clear_max_seg_size(dev->drm_dev->dev);
	v4l2_m2m_release(dev->m2m->m2m_dev);
	video_unregister_device(&dev->m2m->vfd);
	v4l2_device_unregister(&dev->m2m->v4l2_dev);
}
