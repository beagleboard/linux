// SPDX-License-Identifier: GPL-2.0
/*
 * IMG Encoder v4l2 Driver Interface function implementations
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Sunita Nadampalli <sunitan@ti.com>
 *	David Huang <d-huang@ti.com>
 *
 * Re-written for upstreaming
 *	Sidraya Jayagond <sidraya.bj@pathpartnertech.com>
 */

#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/idr.h>
#include <linux/slab.h>
#include <linux/time64.h>
#include <linux/interrupt.h>
#include <linux/printk.h>
#include <linux/mutex.h>
#include <linux/dma-mapping.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-sg.h>
#include <media/v4l2-event.h>
#include <media/v4l2-mem2mem.h>

#include "fw_headers/vxe_common.h"
#include "img_mem_man.h"
#include "target_config.h"
#include "topaz_device.h"
#include "vxe_enc.h"
#include "vxe_v4l2.h"
#include "img_errors.h"

#define IMG_VXE_ENC_MODULE_NAME "vxe-enc"

static struct heap_config vxe_enc_heap_configs[] = {
	{
		.type = MEM_HEAP_TYPE_UNIFIED,
		.options.unified = {
			.gfp_type = __GFP_DMA32 | __GFP_ZERO,
		},
		.to_dev_addr = NULL,
	},
};

static struct vxe_enc_fmt vxe_enc_formats[] = {
	{
		.fourcc = V4L2_PIX_FMT_NV12,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.fmt = IMG_CODEC_420_PL12,
		.min_bufs = 2,
		.size_num[0] = 3,
		.size_den[0] = 2,
		.bytes_pp = 1,
		.csc_preset = IMG_CSC_NONE,
	},
	{
		.fourcc = V4L2_PIX_FMT_RGB32,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.fmt = IMG_CODEC_ABCX,
		.min_bufs = 2,
		.size_num[0] = 1,
		.size_den[0] = 1,
		.bytes_pp = 4,
		.csc_preset = IMG_CSC_RGB_TO_601_ANALOG,
	},
	{
		.fourcc = V4L2_PIX_FMT_H264,
		.num_planes = 1,
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		.std = IMG_STANDARD_H264,
		.min_bufs = 1,
		.size_num[0] = 1,
		.size_den[0] = 1,
		.bytes_pp = 1,
		.csc_preset = IMG_CSC_NONE,
	},
};

/* Note: Arrange in order of ascending CID # to simplify QUERYCTRL */
static struct vxe_ctrl controls[] = {
	{
		/*
		 * idr_period
		 *
		 * Period between IDR frames. Default to 60 * framerate.
		 * Since default framerate is 30fps, default to 1800 frames
		 * between IDR frames. IDR frames are a special I frame in
		 * H.264 that specifies no frame after the IDR frame can
		 * reference any frame before the IDR frame.
		 *
		 * This period is in number of frames.
		 * ex. Default: 1800
		 * Every 1800 frames is an IDR frame. At 30fps this means there
		 * is an IDR frame every 60 seconds.
		 */
		.cid = V4L2_CID_MPEG_VIDEO_GOP_SIZE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "GOP size",
		.minimum = 1,
		.maximum = 7200,
		.step = 1,
		.default_value = 1800,
		.compound = FALSE,
	},
	{
		/*
		 * bits_per_second
		 *
		 * Bits per second for the encode. This will be the final
		 * bitrate of the encoded stream. Warning, setting this too
		 * low results in extreme loss of quality and choppy output.
		 *
		 * This is specified in bits per second
		 */
		.cid = V4L2_CID_MPEG_VIDEO_BITRATE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Video Bitrate",
		.minimum = 50000,
		.maximum = 100000000,
		.step = 1,
		.default_value = 500000,
		.compound = FALSE,
	},
	{
		/*
		 * intra_freq
		 *
		 * Period between I-frames. I-frames are complete frames that
		 * do not need to reference any other frames to decode. Named
		 * intra_freq instead of intra_period due to naming in
		 * underlying topaz_api layers.
		 *
		 * This frequency is actually the period between I-frames.
		 * ex. Default: 30
		 * This means there is an I-frame every 30 frames. At 30fps
		 * this would mean one I-frame every second.
		 */
		.cid = V4L2_CID_MPEG_VIDEO_H264_I_PERIOD,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "H264 I period",
		.minimum = 1,
		.maximum = 600,
		.step = 1,
		.default_value = 30,
		.compound = FALSE,
	},
};

static struct v4l2_fract frmivals[] = {
	{
		.numerator = 1,
		.denominator = 960,
	},
	{
		.numerator = 1,
		.denominator = 1,
	},
};

static struct vxe_enc_ctx *file2ctx(struct file *file)
{
	return container_of(file->private_data, struct vxe_enc_ctx, fh);
}

static void vxe_eos(struct vxe_enc_ctx *ctx)
{
	struct v4l2_event event = {};
	struct vb2_v4l2_buffer *vb;

	event.type = V4L2_EVENT_EOS;
	v4l2_event_queue_fh(&ctx->fh, &event);
	/*
	 * If a capture buffer is available, dequeue with FLAG_LAST
	 * else, mark for next qbuf to handle
	 */
	if (v4l2_m2m_num_dst_bufs_ready(ctx->fh.m2m_ctx) > 0) {
		vb = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
		vb->flags |= V4L2_BUF_FLAG_LAST;
		vb2_set_plane_payload(&vb->vb2_buf, 0, 0);
		v4l2_m2m_buf_done(vb, VB2_BUF_STATE_DONE);
	} else {
		ctx->flag_last = TRUE;
	}

	topaz_flush_stream(ctx->topaz_str_context, ctx->last_frame_num);
}

static void vxe_return_resource(void *ctx_handle, enum vxe_cb_type type,
				void *img_buf_ref, unsigned int size,
	unsigned int coded_frm_cnt)
{
	struct vxe_enc_ctx *ctx = ctx_handle;
	struct device *dev = ctx->dev->dev;
	struct vxe_buffer *buf;
#ifdef ENABLE_PROFILING
	struct timespec64 time;
#endif

	switch (type) {
	case VXE_CB_CODED_BUFF_READY:
		if (!img_buf_ref)
			dev_err(dev, "VXE_CB_STRUNIT_PROCESSED had no buffer\n");

		buf = container_of((struct img_coded_buffer *)img_buf_ref,
				   struct vxe_buffer, coded_buffer);
		vb2_set_plane_payload(&buf->buffer.vb.vb2_buf, 0, size);
#ifdef ENABLE_PROFILING
		ktime_get_real_ts64(&time);
		ctx->drv_lat.end_time = timespec64_to_ns((const struct timespec64 *)&time);

		pr_err("driver encode time is %llu us\n", div_s64(ctx->drv_lat.end_time -
		       ctx->drv_lat.start_time, 1000));
#endif

		v4l2_m2m_buf_done(&buf->buffer.vb, VB2_BUF_STATE_DONE);

		if ((coded_frm_cnt == ctx->last_frame_num) && (coded_frm_cnt != 0)) {
			vxe_eos(ctx);
			ctx->eos = TRUE;
		}
		if (ctx->frames_encoding < 2)
			v4l2_m2m_job_finish(ctx->dev->m2m_dev, ctx->fh.m2m_ctx);
		break;
	case VXE_CB_SRC_FRAME_RELEASE:
		if (!img_buf_ref)
			dev_err(dev, "VXE_CB_PICT_RELEASE had no buffer\n");

		buf = container_of((struct img_frame *)img_buf_ref, struct vxe_buffer, src_frame);
		vb2_set_plane_payload(&buf->buffer.vb.vb2_buf, 0, size);
		v4l2_m2m_buf_done(&buf->buffer.vb, VB2_BUF_STATE_DONE);
		ctx->frames_encoding--;
		break;
	case VXE_CB_ERROR_FATAL:
		break;
	default:
		break;
	}
}

static void device_run(void *priv)
{
	struct vxe_enc_ctx *ctx = priv;
	struct device *dev = ctx->dev->dev;
	struct vb2_v4l2_buffer *dst_vbuf, *src_vbuf;
	struct vxe_buffer *buf;
	int ret = 0;
#ifdef ENABLE_PROFILING
	struct timespec64 time;
#endif

	mutex_lock_nested(ctx->mutex, SUBCLASS_VXE_V4L2);
	while (((topaz_query_empty_coded_slots(ctx->topaz_str_context) > 0) &&
		(v4l2_m2m_num_dst_bufs_ready(ctx->fh.m2m_ctx) > 0)) &&
		((topaz_query_empty_source_slots(ctx->topaz_str_context) > 0) &&
		(v4l2_m2m_num_src_bufs_ready(ctx->fh.m2m_ctx) > 0))) {
#ifdef ENABLE_PROFILING
		ktime_get_real_ts64(&time);
		ctx->drv_lat.start_time = timespec64_to_ns((const struct timespec64 *)&time);
#endif
		/*
		 * Submit src and dst buffers one to one
		 * Note: Will have to revisit for B frame support
		 */
		dst_vbuf = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
		if (!dst_vbuf)
			dev_err(dev, "Next src buffer is null\n");

		src_vbuf = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		if (!src_vbuf)
			dev_err(dev, "Next src buffer is null\n");

		/* Handle EOS */
		if (ctx->eos && (v4l2_m2m_num_src_bufs_ready(ctx->fh.m2m_ctx) == 0)) {
			pr_debug("%s eos found\n", __func__);
			ret = topaz_end_of_stream(ctx->topaz_str_context, ctx->frame_num + 1);
			if (ret)
				dev_err(dev, "Failed to send EOS to topaz %d\n",
					ret);
			ctx->last_frame_num = ctx->frame_num + 1;
		}

		/* Submit coded package */
		buf = container_of(dst_vbuf, struct vxe_buffer, buffer.vb);
		ret = topaz_reserve_coded_package_slot(ctx->topaz_str_context);
		if (ret)
			dev_err(dev, "Failed to reserve coded package slot %d\n", ret);
		ret = topaz_send_coded_package(ctx->topaz_str_context, &buf->coded_buffer);
		if (ret)
			dev_err(dev, "Failed to send coded package %d\n",
				ret);
		if (!ret)
			ctx->available_coded_packages++;

		/* Submit source frame */
		buf = container_of(src_vbuf, struct vxe_buffer, buffer.vb);
		ret = topaz_reserve_source_slot(ctx->topaz_str_context, &buf->src_slot_num);
		if (ret)
			dev_err(dev, "Failed to reserve source slot %d\n",
				ret);
		ret = topaz_send_source_frame(ctx->topaz_str_context, &buf->src_frame,
					      ctx->frame_num, (unsigned long)ctx);
		if (ret)
			dev_err(dev, "Failed to send source frame %d\n",
				ret);
		ctx->frame_num++;
		if (!ret)
			ctx->available_source_frames++;
	}

	while ((ctx->available_source_frames > 0) && (ctx->available_coded_packages > 0)) {
		pr_debug("Calling topaz_encode_frame #src=%d #coded=%d frames_encoding=%d\n",
			 ctx->available_source_frames,
			 ctx->available_coded_packages,
			 ctx->frames_encoding);
		ret = topaz_encode_frame(ctx->topaz_str_context);
		if (ret) {
			dev_err(dev, "Failed to send encode_frame command %d\n",
				ret);
		} else {
			/* TODO: Account for scenarios where these are not 1 */
			ctx->available_source_frames--;
			ctx->available_coded_packages--;
			ctx->frames_encoding++;
		}
	}

	mutex_unlock((struct mutex *)ctx->mutex);
}

static int job_ready(void *priv)
{
	struct vxe_enc_ctx *ctx = priv;

	/*
	 * In normal play, check if we can
	 * submit any source or coded buffers
	 */
	if (((topaz_query_empty_source_slots(ctx->topaz_str_context) > 0) &&
	     (v4l2_m2m_num_src_bufs_ready(ctx->fh.m2m_ctx) > 0)) &&
	    ((topaz_query_empty_coded_slots(ctx->topaz_str_context) > 0) &&
	    (v4l2_m2m_num_dst_bufs_ready(ctx->fh.m2m_ctx) > 0)) && ctx->core_streaming)
		return 1;

	/*
	 * In EOS state, we only need to know
	 * that coded buffers are available
	 */
	if (ctx->eos && (v4l2_m2m_num_dst_bufs_ready(ctx->fh.m2m_ctx) > 0) &&
	    (topaz_query_empty_coded_slots(ctx->topaz_str_context) > 0) && ctx->core_streaming)
		return 1;

	/*
	 * Since we're allowing device_run for both submissions and actual
	 * encodes, say job ready if buffers are ready in fw
	 */
	if (ctx->available_source_frames > 0 && ctx->available_coded_packages > 0
			&& ctx->core_streaming)
		return 1;

	return 0;
}

static void job_abort(void *priv)
{
	/* TODO: stub */
	struct vxe_enc_ctx *ctx = priv;

	ctx->core_streaming = FALSE;
}

static const struct v4l2_m2m_ops m2m_ops = {
	.device_run = device_run,
	.job_ready = job_ready,
	.job_abort = job_abort,
};

static struct vxe_enc_q_data *get_queue(struct vxe_enc_ctx *ctx,
					enum v4l2_buf_type type)
{
	switch (type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
		return &ctx->out_queue;
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		return &ctx->cap_queue;
	default:
		return NULL;
	}
	return NULL;
}

static int vxe_queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
			   unsigned int *nplanes, unsigned int sizes[],
			   struct device *alloc_devs[])
{
	int i;
	struct vxe_enc_ctx *ctx = vb2_get_drv_priv(vq);
	struct vxe_enc_q_data *queue;

	queue = get_queue(ctx, vq->type);
	if (!queue)
		return -EINVAL;

	if (*nplanes) {
		/* This is being called from CREATEBUFS, perform validation */
		if (*nplanes != queue->fmt->num_planes)
			return -EINVAL;

		for (i = 0; i < *nplanes; i++) {
			if (sizes[i] != queue->size_image[i])
				return -EINVAL;
		}

		return 0;
	}

	*nplanes = queue->fmt->num_planes;

	if (V4L2_TYPE_IS_OUTPUT(queue->fmt->type)) {
		*nbuffers = max(*nbuffers, queue->fmt->min_bufs);
	} else {
		*nbuffers = topaz_get_coded_package_max_num(ctx->topaz_str_context,
							    queue->fmt->std,
							    queue->width,
							    queue->height,
							    &ctx->rc);
		for (i = 0; i < *nplanes; i++) {
			queue->size_image[i] =
				topaz_get_coded_buffer_max_size(ctx->topaz_str_context,
								queue->fmt->std,
								queue->width,
								queue->height,
								&ctx->rc);
		}
	}

	for (i = 0; i < *nplanes; i++)
		sizes[i] = queue->size_image[i];

	return 0;
}

static int vxe_buf_init(struct vb2_buffer *vb)
{
	struct vxe_enc_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct device *dev = ctx->dev->dev;
	struct vxe_enc_q_data *queue;
	void *sgt;
	int i, num_planes, ret;
	struct vxe_buffer *buf = container_of(vb, struct vxe_buffer,
			buffer.vb.vb2_buf);

	queue = get_queue(ctx, vb->vb2_queue->type);
	if (!queue) {
		dev_err(dev, "Invalid queue type %d\n",
			vb->vb2_queue->type);
		return -EINVAL;
	}

	num_planes = queue->fmt->num_planes;

	for (i = 0; i < num_planes; i++) {
		if (vb2_plane_size(vb, i) < queue->size_image[i]) {
			dev_err(dev, "data will not fit into plane(%lu < %lu)\n",
				vb2_plane_size(vb, i),
				(long)queue->size_image[i]);
			return -EINVAL;
		}
	}

	buf->buf_info.cpu_virt = vb2_plane_vaddr(vb, 0);
	buf->buf_info.buf_size = vb2_plane_size(vb, 0);

	sgt = vb2_dma_sg_plane_desc(vb, 0);
	if (!sgt) {
		dev_err(dev, "Could not get sg_table from plane 0\n");
		return -EINVAL;
	}

	if (V4L2_TYPE_IS_OUTPUT(vb->type)) {
		ret = topaz_stream_map_buf_sg(ctx->topaz_str_context, VENC_BUFTYPE_PICTURE,
					      &buf->buf_info, sgt);
		if (ret) {
			dev_err(dev, "OUTPUT core_stream_map_buf_sg failed\n");
			return ret;
		}
		pr_debug("Picture buffer mapped successfully, buf_id[%d], dev_virt[%x]\n",
			 buf->buf_info.buff_id, buf->buf_info.dev_virt);

		vxe_fill_default_src_frame_params(buf);

		buf->y_buffer.mem_info = buf->buf_info;
		buf->y_buffer.lock = BUFFER_FREE;
		buf->y_buffer.size = 0; /* IMG has 0 */
		buf->y_buffer.bytes_written = 0;

		/* TODO Fill U/V img buffers if necessary */
		buf->src_frame.y_plane_buffer = &buf->y_buffer;
		buf->src_frame.u_plane_buffer = NULL;
		buf->src_frame.v_plane_buffer = NULL;
		buf->src_frame.y_component_offset = 0;
		buf->src_frame.u_component_offset = queue->bytesperline[0] * queue->height;
		buf->src_frame.v_component_offset = queue->bytesperline[0] * queue->height;

		buf->src_frame.width_bytes = queue->bytesperline[0];
		buf->src_frame.height = queue->height;
		buf->src_frame.src_y_stride_bytes = queue->bytesperline[0];
		buf->src_frame.src_uv_stride_bytes = queue->bytesperline[0];
	} else {
		ret = topaz_stream_map_buf_sg(ctx->topaz_str_context,
					      VENC_BUFTYPE_BITSTREAM,
					      &buf->buf_info, sgt);
		if (ret) {
			dev_err(dev, "CAPTURE core_stream_map_buf_sg failed\n");
			return ret;
		}
		pr_debug("Bit-stream buffer mapped successfully, buf_id[%d], dev_virt[%x]\n",
			 buf->buf_info.buff_id, buf->buf_info.dev_virt);

		buf->coded_buffer.mem_info = buf->buf_info;
		buf->coded_buffer.lock = BUFFER_FREE;
		buf->coded_buffer.size = vb2_plane_size(vb, 0);
		buf->coded_buffer.bytes_written = 0;
	}

	return 0;
}

static int vxe_buf_prepare(struct vb2_buffer *vb)
{
#ifdef DEBUG_ENCODER_DRIVER
	int i;
	struct vxe_buffer *buf = container_of(vb, struct vxe_buffer,
			buffer.vb.vb2_buf);

	pr_info("%s printing contents of buffer %d at 0x%p\n",
		__func__, vb->index, buf->buf_info.cpu_virt);
	for (i = 0; i < 1536; i = i + 8) {
		pr_info("[%d] 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x,\n",
			((i + 1) / 8),
			((char *)buf->buf_info.cpu_virt)[i + 0],
			((char *)buf->buf_info.cpu_virt)[i + 1],
			((char *)buf->buf_info.cpu_virt)[i + 2],
			((char *)buf->buf_info.cpu_virt)[i + 3],
			((char *)buf->buf_info.cpu_virt)[i + 4],
			((char *)buf->buf_info.cpu_virt)[i + 5],
			((char *)buf->buf_info.cpu_virt)[i + 6],
			((char *)buf->buf_info.cpu_virt)[i + 7]);
	}
#endif
	return 0;
}

static void vxe_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct vxe_enc_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	mutex_lock_nested(ctx->mutex, SUBCLASS_VXE_V4L2);
	if (ctx->flag_last && (!V4L2_TYPE_IS_OUTPUT(vb->type))) {
		/*
		 * If EOS came and we did not have a buffer ready
		 * to service it, service now that we have a buffer
		 */
		vbuf->flags |= V4L2_BUF_FLAG_LAST;
		vb2_set_plane_payload(&vbuf->vb2_buf, 0, 0);
		v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_DONE);
	} else {
		v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
	}
	mutex_unlock((struct mutex *)ctx->mutex);
}

static void vxe_buf_cleanup(struct vb2_buffer *vb)
{
	struct vxe_enc_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct vxe_buffer *buf = container_of(vb, struct vxe_buffer,
			buffer.vb.vb2_buf);

	pr_debug("%s Unmapping buffer %d\n", __func__, buf->index);
	topaz_stream_unmap_buf_sg(ctx->topaz_str_context, &buf->buf_info);
}

static int vxe_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct vxe_enc_ctx *ctx = vb2_get_drv_priv(vq);
	struct vxe_enc_q_data *queue;
	ctx->core_streaming = TRUE;

	queue = get_queue(ctx, vq->type);
	queue->streaming = TRUE;

	return 0;
}

static void vxe_stop_streaming(struct vb2_queue *vq)
{
	struct vxe_enc_ctx *ctx = vb2_get_drv_priv(vq);
	struct device *dev = ctx->dev->dev;
	struct vb2_v4l2_buffer *vb;
	struct vxe_enc_q_data *queue;

	queue = get_queue(ctx, vq->type);
	/* Unmap all buffers in v4l2 from mmu */
	mutex_lock_nested(ctx->mutex, SUBCLASS_VXE_V4L2);
	ctx->core_streaming = FALSE;
	if (!V4L2_TYPE_IS_OUTPUT(queue->fmt->type)) {
		while (v4l2_m2m_num_dst_bufs_ready(ctx->fh.m2m_ctx)) {
			vb = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
			if (!vb)
				dev_err(dev, "Next dst buffer is null\n");
			v4l2_m2m_buf_done(vb, VB2_BUF_STATE_ERROR);
		}
	} else {
		while (v4l2_m2m_num_src_bufs_ready(ctx->fh.m2m_ctx)) {
			vb = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
			if (!vb)
				dev_err(dev, "Next dst buffer is null\n");
			v4l2_m2m_buf_done(vb, VB2_BUF_STATE_ERROR);
		}
	}
	mutex_unlock(ctx->mutex);
}

static struct vb2_ops vxe_video_ops = {
	.queue_setup = vxe_queue_setup,
	.buf_init = vxe_buf_init,
	.buf_prepare = vxe_buf_prepare,
	.buf_queue = vxe_buf_queue,
	.buf_cleanup = vxe_buf_cleanup,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
	.start_streaming = vxe_start_streaming,
	.stop_streaming = vxe_stop_streaming,
};

static int queue_init(void *priv, struct vb2_queue *src_vq,
		      struct vb2_queue *dst_vq)
{
	struct vxe_enc_ctx *ctx = priv;
	struct vxe_dev *vxe = ctx->dev;
	int ret = 0;

	/* src_vq */
	memset(src_vq, 0, sizeof(*src_vq));
	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct vxe_buffer);
	src_vq->ops = &vxe_video_ops;
	src_vq->mem_ops = &vb2_dma_sg_memops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = vxe->mutex;
	src_vq->dev = vxe->ti_vxe_dev.dev;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	/* dst_vq */
	memset(dst_vq, 0, sizeof(*dst_vq));
	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct vxe_buffer);
	dst_vq->ops = &vxe_video_ops;
	dst_vq->mem_ops = &vb2_dma_sg_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = vxe->mutex;
	dst_vq->dev = vxe->ti_vxe_dev.dev;

	ret = vb2_queue_init(dst_vq);
	if (ret) {
		vb2_queue_release(src_vq);
		return ret;
	}

	return 0;
}

static int vxe_open(struct file *file)
{
	struct vxe_dev *vxe = video_drvdata(file);
	struct vxe_enc_ctx *ctx;
	int i, ret = 0;

	dev_dbg(vxe->dev, "%s:%d vxe %p\n", __func__, __LINE__, vxe);

	mutex_lock((struct mutex *)vxe->mutex);

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		mutex_unlock((struct mutex *)vxe->mutex);
		return -ENOMEM;
	}

	ctx->mutex = kzalloc(sizeof(*ctx->mutex), GFP_KERNEL);
	if (!ctx->mutex)
		return -ENOMEM;

	mutex_init(ctx->mutex);

	ctx->dev = vxe;
	ctx->s_fmt_flags = 0;
	ctx->eos = FALSE;
	ctx->flag_last = FALSE;
	ctx->available_coded_packages = 0;
	ctx->available_source_frames = 0;
	ctx->frames_encoding = 0;
	ctx->frame_num = 0;
	ctx->out_queue.streaming = FALSE;
	ctx->cap_queue.streaming = FALSE;

	for (i = 0; i < ARRAY_SIZE(vxe_enc_formats); i++) {
		if (vxe_enc_formats[i].type ==
			V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
			ctx->out_queue.fmt = &vxe_enc_formats[i];
			break;
		}
	}
	for (i = 0; i < ARRAY_SIZE(vxe_enc_formats); i++) {
		if (vxe_enc_formats[i].type ==
			V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
			ctx->cap_queue.fmt = &vxe_enc_formats[i];
			break;
		}
	}

	v4l2_fh_init(&ctx->fh, video_devdata(file));
	file->private_data = &ctx->fh;

	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(vxe->m2m_dev, ctx, &queue_init);
	if (IS_ERR_VALUE((unsigned long)ctx->fh.m2m_ctx)) {
		ret = (long)(ctx->fh.m2m_ctx);
		goto exit;
	}

	vxe_fill_default_params(ctx);

	v4l2_fh_add(&ctx->fh);

	vxe_create_ctx(vxe, ctx);

	/* TODO: Add stream id creation */
exit:
	mutex_unlock((struct mutex *)vxe->mutex);
	return ret;
}

static int vxe_release(struct file *file)
{
	struct vxe_dev *vxe = video_drvdata(file);
	struct vxe_enc_ctx *ctx = file2ctx(file);
	/* TODO Need correct API */

	mutex_lock((struct mutex *)vxe->mutex);

	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);

	if (ctx->s_fmt_flags & S_FMT_FLAG_STREAM_CREATED)
		topaz_stream_destroy(ctx->topaz_str_context);
	ctx->topaz_str_context = NULL;

	mutex_destroy(ctx->mutex);
	kfree(ctx->mutex);
	ctx->mutex = NULL;
	kfree(ctx);

	mutex_unlock((struct mutex *)vxe->mutex);

	return 0;
}

static const struct v4l2_file_operations vxe_enc_fops = {
	.owner = THIS_MODULE,
	.open = vxe_open,
	.release = vxe_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

static int vxe_querycap(struct file *file, void *priv,
			struct v4l2_capability *cap)
{
	strncpy(cap->driver, IMG_VXE_ENC_MODULE_NAME, sizeof(cap->driver) - 1);
	strncpy(cap->card, IMG_VXE_ENC_MODULE_NAME, sizeof(cap->card) - 1);
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s", IMG_VXE_ENC_MODULE_NAME);
	cap->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}

static struct vxe_enc_fmt *find_format(struct v4l2_format *f)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(vxe_enc_formats); ++i) {
		if (vxe_enc_formats[i].fourcc == f->fmt.pix_mp.pixelformat &&
		    vxe_enc_formats[i].type == f->type)
			return &vxe_enc_formats[i];
	}
	return NULL;
}

static int vxe_enum_fmt(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	int i, index = 0;
	struct vxe_enc_fmt *fmt = NULL;

	for (i = 0; i < ARRAY_SIZE(vxe_enc_formats); ++i) {
		if (vxe_enc_formats[i].type == f->type) {
			if (index == f->index) {
				fmt = &vxe_enc_formats[i];
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

static int vxe_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct vxe_enc_ctx *ctx = file2ctx(file);
	struct v4l2_pix_format_mplane *pix_mp;
	struct vxe_enc_q_data *queue;
	int i;

	pix_mp = &f->fmt.pix_mp;

	queue = get_queue(ctx, f->type);
	if (!queue)
		return -EINVAL;

	pix_mp->width = queue->width;
	pix_mp->height = queue->height;
	pix_mp->pixelformat = queue->fmt->fourcc;
	pix_mp->field = V4L2_FIELD_NONE;

	for (i = 0; i < queue->fmt->num_planes; i++) {
		pix_mp->plane_fmt[i].sizeimage = queue->size_image[i];
		pix_mp->plane_fmt[i].bytesperline = queue->bytesperline[i];
	}
	pix_mp->num_planes = queue->fmt->num_planes;

	return 0;
}

static int vxe_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct vxe_enc_ctx *ctx = file2ctx(file);
	struct vxe_enc_fmt *fmt;
	struct vxe_enc_q_data *queue;
	int i;
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	struct v4l2_plane_pix_format *plane_fmt = pix_mp->plane_fmt;
	struct img_rc_params rc;

	fmt = find_format(f);
	if (!fmt)
		return -EINVAL;

	queue = get_queue(ctx, f->type);
	if (!queue)
		return -EINVAL;

	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		pix_mp->num_planes = fmt->num_planes;
		pix_mp->flags = 0;
		for (i = 0; i < fmt->num_planes; i++) {
			plane_fmt[i].bytesperline = vxe_get_stride(pix_mp->width, fmt);
			plane_fmt[i].sizeimage = vxe_get_sizeimage(plane_fmt[i].bytesperline,
								   pix_mp->height, fmt, i);
		}
	} else {
		pix_mp->flags = 0;
		/* Worst case estimation of sizeimage
		 *plane_fmt[0].sizeimage = ALIGN(pix_mp->width, HW_ALIGN) *
		 *			 ALIGN(pix_mp->height, HW_ALIGN) * 2;
		 */
		/* TODO: This is the only thing that matters here, make sure this is correct */
		rc.initial_qp_i = 18;
		plane_fmt[0].bytesperline = 0;
		plane_fmt[0].sizeimage = topaz_get_coded_buffer_max_size(NULL, fmt->std,
									 pix_mp->width,
									 pix_mp->height,
									 &rc);
	}

	if (pix_mp->field == V4L2_FIELD_ANY)
		pix_mp->field = V4L2_FIELD_NONE;

	return 0;
}

static int vxe_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct vxe_enc_ctx *ctx = file2ctx(file);
	struct v4l2_pix_format_mplane *pix_mp;
	struct vxe_enc_fmt *fmt;
	struct vxe_enc_q_data *queue;
	int i, ret = 0;
	unsigned int level_h264;
	static int base_pipe;

	ret = vxe_try_fmt(file, priv, f);
	if (ret)
		return ret;

	fmt = find_format(f);
	if (!fmt)
		return -EINVAL;

	queue = get_queue(ctx, f->type);
	if (!queue)
		return -EINVAL;

	pix_mp = &f->fmt.pix_mp;

	queue->fmt = fmt;
	queue->width = pix_mp->width;
	queue->height = pix_mp->height;

	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		ctx->vparams.format = fmt->fmt;
		ctx->vparams.source_width = pix_mp->width;
		ctx->vparams.source_frame_height = pix_mp->height;
		ctx->vparams.csc_preset = fmt->csc_preset;
		if (ctx->vparams.csc_preset != IMG_CSC_NONE)
			ctx->vparams.enable_scaler = TRUE;

		pr_debug("img_video_params: format=%d\n", ctx->vparams.format);
		pr_debug("img_video_params: source_width=%d\n", ctx->vparams.source_width);
		pr_debug("img_video_params: source_frame_height=%d\n",
			 ctx->vparams.source_frame_height);
		pr_debug("img_video_params: csc_preset=%d\n", ctx->vparams.csc_preset);
		pr_debug("img_video_params: enable_scaler=%s\n",
			 ctx->vparams.enable_scaler ? "true" : "false");

		for (i = 0; i < fmt->num_planes; i++) {
			queue->bytesperline[i] = vxe_get_stride(queue->width, fmt);
			queue->size_image[i] = vxe_get_sizeimage(pix_mp->plane_fmt[i].bytesperline,
								 queue->height, fmt, i);
		}

		/* Rate Control parameters */
		ctx->rc.transfer_bits_per_second = ctx->rc.bits_per_second;
		ctx->rc.bu_size = -1414812757; /* Pretty sure uninitialized */
		ctx->rc.buffer_size = ctx->rc.transfer_bits_per_second;

		ctx->rc.initial_level = (3 * ctx->rc.buffer_size) >> 4;
		ctx->rc.initial_level = ((ctx->rc.initial_level +
			((ctx->rc.bits_per_second /
			ctx->rc.frame_rate) / 2)) /
			(ctx->rc.bits_per_second /
			ctx->rc.frame_rate)) *
			(ctx->rc.bits_per_second / ctx->rc.frame_rate);
		ctx->rc.initial_level = max((unsigned int)ctx->rc.initial_level,
					    (unsigned int)(ctx->rc.bits_per_second /
				ctx->rc.frame_rate));
		ctx->rc.initial_delay = ctx->rc.buffer_size - ctx->rc.initial_level;
		ctx->rc.bframes = 0;

		pr_debug("img_rc_params: initial_level=%d\n", ctx->rc.initial_level);
		pr_debug("img_rc_params: initial_delay=%d\n", ctx->rc.initial_delay);
		/* TODO Figure out which lossless to use */
		ctx->sh_params.profile = find_h264_profile
				(FALSE,
				 ctx->vparams.use_default_scaling_list,
				 FALSE,
				 ctx->vparams.h264_8x8,
				 ctx->vparams.enable_mvc,
				 ctx->rc.bframes,
				 ctx->vparams.is_interlaced,
				 ctx->vparams.cabac_enabled,
				 ctx->vparams.weighted_prediction,
				 ctx->vparams.vp_weighted_implicit_bi_pred);
		ctx->sh_params.max_num_ref_frames = 1; //TODO Need more logic

		level_h264 = calculate_h264_level(pix_mp->width, pix_mp->height,
						  ctx->rc.frame_rate,
						  ctx->rc.rc_enable,
						  ctx->rc.bits_per_second,
						  /* TODO Figure out which lossless to use */
						  FALSE,
						  ctx->sh_params.profile,
						  ctx->sh_params.max_num_ref_frames);
		pr_debug("level_h264=%d\n", level_h264);

		ctx->vparams.vert_mv_limit = 255;
		if (level_h264 >= 110)
			ctx->vparams.vert_mv_limit = 511;
		if (level_h264 >= 210)
			ctx->vparams.vert_mv_limit = 1023;
		if (level_h264 >= 310)
			ctx->vparams.vert_mv_limit = 2047;

		if (level_h264 >= 300)
			ctx->vparams.limit_num_vectors = TRUE;
		else
			ctx->vparams.limit_num_vectors = FALSE;

		pr_debug("ctx->vparams.vert_mv_limit=%d\n", ctx->vparams.vert_mv_limit);
		pr_debug("ctx->vparams.limit_num_vectors=%d\n", ctx->vparams.limit_num_vectors);

		/* VUI parameters */
		ctx->vui_params.time_scale = ctx->rc.frame_rate * 2;
		ctx->vui_params.bit_rate_value_minus1 = (ctx->rc.bits_per_second / 64)
			- 1;
		ctx->vui_params.cbp_size_value_minus1 = (ctx->rc.buffer_size / 64) - 1;
		ctx->vui_params.aspect_ratio_info_present_flag = FALSE; //unset
		ctx->vui_params.aspect_ratio_idc = 0; //unset
		ctx->vui_params.sar_width = 0; //unset
		ctx->vui_params.sar_height = 0; //unset
		ctx->vui_params.cbr = (ctx->rc.rc_mode == IMG_RCMODE_CBR) ?
			TRUE : FALSE;
		ctx->vui_params.initial_cpb_removal_delay_length_minus1 =
			BPH_SEI_NAL_INITIAL_CPB_REMOVAL_DELAY_SIZE - 1;
		ctx->vui_params.cpb_removal_delay_length_minus1 =
			PTH_SEI_NAL_CPB_REMOVAL_DELAY_SIZE - 1;
		ctx->vui_params.dpb_output_delay_length_minus1 =
			PTH_SEI_NAL_DPB_OUTPUT_DELAY_SIZE - 1;
		ctx->vui_params.time_offset_length = 24; //hard coded
		ctx->vui_params.num_reorder_frames = 0; //TODO
		ctx->vui_params.max_dec_frame_buffering = 0; //unset

		pr_debug("h264_vui_params: time_scale=%d\n", ctx->vui_params.time_scale);
		pr_debug("h264_vui_params: bit_rate_value_minus1=%d\n",
			 ctx->vui_params.bit_rate_value_minus1);
		pr_debug("h264_vui_params: cbp_size_value_minus1=%d\n",
			 ctx->vui_params.cbp_size_value_minus1);
		pr_debug("h264_vui_params: cbr=%d\n", ctx->vui_params.cbr);
		pr_debug("h264_vui_params: initial_cpb_removal_delay_length_minus1=%d\n",
			 ctx->vui_params.initial_cpb_removal_delay_length_minus1);
		pr_debug("h264_vui_params: cpb_removal_delay_length_minus1=%d\n",
			 ctx->vui_params.cpb_removal_delay_length_minus1);
		pr_debug("h264_vui_params: dpb_output_delay_length_minus1=%d\n",
			 ctx->vui_params.dpb_output_delay_length_minus1);

		/* Sequence Header parameters */
		switch (level_h264) {
		case  100:
			ctx->sh_params.level = SH_LEVEL_1;
			break;
		case 101:
			ctx->sh_params.level = SH_LEVEL_1B;
			break;
		case 110:
			ctx->sh_params.level = SH_LEVEL_11;
			break;
		case 120:
			ctx->sh_params.level = SH_LEVEL_12;
			break;
		case 130:
			ctx->sh_params.level = SH_LEVEL_13;
			break;
		case 200:
			ctx->sh_params.level = SH_LEVEL_2;
			break;
		case 210:
			ctx->sh_params.level = SH_LEVEL_21;
			break;
		case 220:
			ctx->sh_params.level = SH_LEVEL_22;
			break;
		case 300:
			ctx->sh_params.level = SH_LEVEL_3;
			break;
		case 310:
			ctx->sh_params.level = SH_LEVEL_31;
			break;
		case 320:
			ctx->sh_params.level = SH_LEVEL_32;
			break;
		case 400:
			ctx->sh_params.level = SH_LEVEL_4;
			break;
		case 410:
			ctx->sh_params.level = SH_LEVEL_41;
			break;
		case 420:
			ctx->sh_params.level = SH_LEVEL_42;
			break;
		case 500:
			ctx->sh_params.level = SH_LEVEL_5;
			break;
		case 510:
			ctx->sh_params.level = SH_LEVEL_51;
			break;
		case 520:
			ctx->sh_params.level = SH_LEVEL_52;
			break;
		default:
			pr_err("Error invalid h264 level %d\n", level_h264);
			return -EINVAL;
		}
		if (V4L2_TYPE_IS_OUTPUT(f->type)) {
			ctx->sh_params.width_in_mbs_minus1 = ((queue->width +
				(MB_SIZE - 1))/MB_SIZE)-1;
			ctx->sh_params.height_in_maps_units_minus1 = ((queue->height +
					(MB_SIZE - 1))/MB_SIZE) - 1;
			pr_debug("h264_sequence_header_params: width_in_mbs_minus1=%d\n",
				 ctx->sh_params.width_in_mbs_minus1);
			pr_debug("h264_sequence_header_params: height_in_maps_units_minus1=%d\n",
				 ctx->sh_params.height_in_maps_units_minus1);
		}
		ctx->sh_params.log2_max_pic_order_cnt = 6; //hard coded
		ctx->sh_params.gaps_in_frame_num_value = FALSE;
		ctx->sh_params.frame_mbs_only_flag = ctx->vparams.is_interlaced ?
			FALSE : TRUE;
		ctx->sh_params.vui_params_present = (ctx->rc.rc_mode == IMG_RCMODE_NONE)
			? FALSE : TRUE;
		ctx->sh_params.seq_scaling_matrix_present_flag = FALSE;
		ctx->sh_params.use_default_scaling_list = FALSE;
		ctx->sh_params.is_lossless = FALSE;
		ctx->sh_params.vui_params = ctx->vui_params;

		pr_debug("h264_sequence_header_params: frame_mbs_only_flag=%d\n",
			 ctx->sh_params.frame_mbs_only_flag);
		pr_debug("h264_sequence_header_params: vui_params_present=%d\n",
			 ctx->sh_params.vui_params_present);

		ctx->s_fmt_flags |= S_FMT_FLAG_OUT_RECV;
	} else {
		for (i = 0; i < fmt->num_planes; i++) {
			queue->bytesperline[i] = 0;
			queue->size_image[i] =
				topaz_get_coded_buffer_max_size(ctx->topaz_str_context,
								queue->fmt->std,
								queue->width,
								queue->height,
								&ctx->rc);
		}
		ctx->vparams.standard = fmt->std;
		ctx->vparams.width = pix_mp->width;
		/*
		 * Note: Do not halve height for interlaced.
		 * App should take care of this.
		 */
		ctx->vparams.frame_height = pix_mp->height;

		pr_debug("img_video_params: standard=%d\n", ctx->vparams.standard);
		pr_debug("img_video_params: width=%d\n", ctx->vparams.width);
		pr_debug("img_video_params: frame_height=%d\n", ctx->vparams.frame_height);

		ctx->s_fmt_flags |= S_FMT_FLAG_CAP_RECV;
	}
	ctx->vparams.is_interlaced = FALSE;

	ctx->vparams.intra_pred_modes = -1414812757; /* Pretty sure uninitialized */

	ctx->vparams.buffer_stride_bytes = 0;
	ctx->vparams.buffer_height = 0;

	ctx->vparams.crop_left = 0;
	ctx->vparams.crop_right = 0;
	ctx->vparams.crop_top = 0;
	ctx->vparams.crop_bottom = 0;

	ctx->vparams.slices_per_picture = 1;

	/* Crop parameters */
	ctx->crop_params.clip = FALSE;
	ctx->crop_params.left_crop_offset = 0;
	ctx->crop_params.right_crop_offset = (((ctx->sh_params.width_in_mbs_minus1 + 1)*MB_SIZE) -
			ctx->vparams.source_width)/2;
	ctx->crop_params.top_crop_offset = 0;
	ctx->crop_params.bottom_crop_offset = (((ctx->sh_params.height_in_maps_units_minus1 + 1)
				*MB_SIZE) - ctx->vparams.source_frame_height)/2;
	if (ctx->crop_params.right_crop_offset | ctx->crop_params.bottom_crop_offset)
		ctx->crop_params.clip = TRUE;

	pr_debug("s_fmt_flags=%#08x\n", ctx->s_fmt_flags);
	if ((ctx->s_fmt_flags & S_FMT_FLAG_OUT_RECV) &&
	    (ctx->s_fmt_flags & S_FMT_FLAG_CAP_RECV)) {
		pr_debug("Calling topaz_stream_create()\n");
		topaz_stream_create(ctx, &ctx->vparams, ((base_pipe++ % 2) ? 0 : 1), 2,
				&ctx->rc, &ctx->topaz_str_context);

		topaz_h264_prepare_sequence_header(ctx->topaz_str_context,
						   ctx->sh_params.width_in_mbs_minus1 + 1,
						   ctx->sh_params.height_in_maps_units_minus1 + 1,
						   TRUE, &ctx->vui_params,
						   &ctx->crop_params,
						   &ctx->sh_params, FALSE);
		/* Note: cqp_offset looks unset in img */
		topaz_h264_prepare_picture_header(ctx->topaz_str_context, 0);

		topaz_load_context(ctx->topaz_str_context);

		ctx->s_fmt_flags |= S_FMT_FLAG_STREAM_CREATED;
	}

	return 0;
}

static int vxe_subscribe_event(struct v4l2_fh *fh,
			       const struct v4l2_event_subscription *sub)
{
	if (sub->type != V4L2_EVENT_EOS)
		return -EINVAL;

	v4l2_event_subscribe(fh, sub, 0, NULL);
	return 0;
}

static int vxe_try_cmd(struct file *file, void *fh,
		       struct v4l2_encoder_cmd *cmd)
{
	if (cmd->cmd != V4L2_DEC_CMD_STOP)
		return -EINVAL;
	return 0;
}

static int vxe_cmd(struct file *file, void *fh, struct v4l2_encoder_cmd *cmd)
{
	struct vxe_enc_ctx *ctx = file2ctx(file);

	if (cmd->cmd != V4L2_DEC_CMD_STOP)
		return -EINVAL;

	mutex_lock_nested(ctx->mutex, SUBCLASS_VXE_V4L2);
	if (v4l2_m2m_num_src_bufs_ready(ctx->fh.m2m_ctx) > 0 ||
	    !ctx->out_queue.streaming || !ctx->cap_queue.streaming) {
		/* Buffers are still in queue for encode, set eos flag */
		ctx->eos = TRUE;
		mutex_unlock((struct mutex *)ctx->mutex);
	} else if ((ctx->available_source_frames > 0) ||
		(ctx->frames_encoding) > 0) {
		/*
		 * Buffers are still in firmware for encode. Tell topaz
		 * that last frame sent is last frame in stream
		 */
		topaz_end_of_stream(ctx->topaz_str_context, ctx->frame_num);
		ctx->last_frame_num = ctx->frame_num;
		mutex_unlock((struct mutex *)ctx->mutex);
	} else {
		/* All buffers are encoded, so issue dummy stream end */
		mutex_unlock((struct mutex *)ctx->mutex);
		vxe_eos(ctx);
	}
	return 0;
}

static int vxe_queryctrl(struct file *file, void *priv,
			 struct v4l2_queryctrl *query)
{
	int i;

	query->reserved[0] = 0;
	query->reserved[1] = 0;

	/* Enumerate controls */
	if (query->id & V4L2_CTRL_FLAG_NEXT_CTRL) {
		query->id &= ~V4L2_CTRL_FLAG_NEXT_CTRL;
		for (i = 0; i < ARRAY_SIZE(controls); i++) {
			if (!controls[i].compound && controls[i].cid > query->id) {
				query->id = controls[i].cid;
				query->type = controls[i].type;
				strncpy(query->name, controls[i].name, sizeof(query->name));
				query->minimum = controls[i].minimum;
				query->maximum = controls[i].maximum;
				query->step = controls[i].step;
				query->default_value = controls[i].default_value;
				query->flags = 0;
				return 0;
			}
		}
		return -EINVAL;
	}

	/* Return info on requested control */
	for (i = 0; i < ARRAY_SIZE(controls); i++) {
		if (controls[i].cid == query->id) {
			query->id = controls[i].cid;
			query->type = controls[i].type;
			strncpy(query->name, controls[i].name, sizeof(query->name));
			query->minimum = controls[i].minimum;
			query->maximum = controls[i].maximum;
			query->step = controls[i].step;
			query->default_value = controls[i].default_value;
			query->flags = 0;
			return 0;
		}
	}

	return -EINVAL;
}

static int vxe_query_ext_ctrl(struct file *file, void *priv,
			      struct v4l2_query_ext_ctrl *query)
{
	unsigned int queryid;
	int i, j;

	query->reserved[0] = 0;
	query->reserved[1] = 0;

	/* Enumerate controls */
	if ((query->id & V4L2_CTRL_FLAG_NEXT_CTRL) ||
	    (query->id & V4L2_CTRL_FLAG_NEXT_COMPOUND)) {
		queryid = query->id;
		queryid &= ~V4L2_CTRL_FLAG_NEXT_CTRL;
		queryid &= ~V4L2_CTRL_FLAG_NEXT_COMPOUND;
		for (i = 0; i < ARRAY_SIZE(controls); i++) {
			if (((!controls[i].compound && (query->id & V4L2_CTRL_FLAG_NEXT_CTRL)) ||
			     (controls[i].compound &&
				(query->id & V4L2_CTRL_FLAG_NEXT_COMPOUND))) &&
				controls[i].cid > queryid) {
				query->id = controls[i].cid;
				query->type = controls[i].type;
				strncpy(query->name, controls[i].name, sizeof(query->name));
				query->minimum = controls[i].minimum;
				query->maximum = controls[i].maximum;
				query->step = controls[i].step;
				query->default_value = controls[i].default_value;
				/* Our supported controls use int values */
				query->elem_size = 4;
				query->elems  = 1;
				query->nr_of_dims = 0;
				for (j = 0; j < V4L2_CTRL_MAX_DIMS; j++)
					query->dims[j] = 0;
				query->flags = 0;
				return 0;
			}
		}
		return -EINVAL;
	}

	/* Return info on requested control */
	for (i = 0; i < ARRAY_SIZE(controls); i++) {
		if (controls[i].cid == query->id) {
			query->id = controls[i].cid;
			query->type = controls[i].type;
			strncpy(query->name, controls[i].name, sizeof(query->name));
			query->minimum = controls[i].minimum;
			query->maximum = controls[i].maximum;
			query->step = controls[i].step;
			query->default_value = controls[i].default_value;
			/* Our supported controls use int values */
			query->elem_size = 4;
			query->elems  = 1;
			query->nr_of_dims = 0;
			for (j = 0; j < V4L2_CTRL_MAX_DIMS; j++)
				query->dims[j] = 0;
			query->flags = 0;
			return 0;
		}
	}

	return -EINVAL;
}

static int vxe_g_ext_ctrls(struct file *file, void *priv,
			   struct v4l2_ext_controls *ctrls)
{
	struct vxe_enc_ctx *ctx = file2ctx(file);
	struct device *dev = ctx->dev->dev;
	struct v4l2_queryctrl query;
	int i;

	ctrls->reserved[0] = 0;
	ctrls->reserved[1] = 0;

	if (ctrls->which == V4L2_CTRL_WHICH_DEF_VAL) {
		for (i = 0; i < ctrls->count; i++) {
			query.id = ctrls->controls[i].id;
			if (vxe_queryctrl(NULL, NULL, &query)) {
				dev_err(dev, "%s could not find default value for id=%#08x\n",
					__func__, ctrls->controls[i].id);
				return -EINVAL;
			}
			ctrls->controls[i].value = query.default_value;
		}
	}

	for (i = 0; i < ctrls->count; i++) {
		ctrls->controls[i].reserved2[0] = 0;

		switch (ctrls->controls[i].id) {
		case V4L2_CID_MPEG_VIDEO_GOP_SIZE:
			ctrls->controls[i].size = 0;
			ctrls->controls[i].value = ctx->vparams.idr_period;
			break;
		case V4L2_CID_MPEG_VIDEO_BITRATE:
			ctrls->controls[i].size = 0;
			ctrls->controls[i].value = ctx->rc.bits_per_second;
			break;
		case V4L2_CID_MPEG_VIDEO_H264_I_PERIOD:
			ctrls->controls[i].size = 0;
			ctrls->controls[i].value = ctx->rc.intra_freq;
			break;
		case V4L2_CID_MPEG_VIDEO_H264_PROFILE:
			ctrls->controls[i].size = 0;
			ctrls->controls[i].value = ctx->sh_params.profile;
			break;
		case V4L2_CID_MPEG_VIDEO_H264_LEVEL:
			ctrls->controls[i].size = 0;
			ctrls->controls[i].value = ctx->sh_params.level;
			break;
		default:
			dev_err(dev, "%s Invalid control id %#08x\n",
				__func__, ctrls->controls[i].id);
			ctrls->error_idx = ctrls->count;
			return -EINVAL;
		}
	}

	return 0;
}

static int vxe_try_ext_ctrls(struct file *file, void *priv,
			     struct v4l2_ext_controls *ctrls)
{
	struct vxe_enc_ctx *ctx = file2ctx(file);
	struct device *dev = ctx->dev->dev;
	struct v4l2_queryctrl query;
	int i;

	ctrls->reserved[0] = 0;
	ctrls->reserved[1] = 0;

	/* Can't write default values or support requests */
	if (ctrls->which != V4L2_CTRL_WHICH_CUR_VAL)
		return -EINVAL;

	/* Cannot change values once context is created */
	/* TODO: Handle controls after stream is created but before streamon */
	if (ctx->s_fmt_flags & S_FMT_FLAG_STREAM_CREATED)
		return -EBUSY;

	for (i = 0; i < ctrls->count; i++) {
		ctrls->controls[i].reserved2[0] = 0;

		query.id = ctrls->controls[i].id;
		if (vxe_queryctrl(NULL, NULL, &query)) {
			dev_err(dev, "%s could not find control id=%#08x\n",
				__func__, ctrls->controls[i].id);
			ctrls->error_idx = i;
			return -EINVAL;
		}
		if (ctrls->controls[i].value < query.minimum) {
			dev_err(dev, "%s control id=%#08x value=%d less than minimum=%d\n",
				__func__, ctrls->controls[i].id,
				ctrls->controls[i].value, query.minimum);
			ctrls->error_idx = i;
			return -ERANGE;
		}
		if (ctrls->controls[i].value > query.maximum) {
			dev_err(dev, "%s control id=%#08x value=%d greater than maximum=%d\n",
				__func__, ctrls->controls[i].id,
				ctrls->controls[i].value, query.maximum);
			ctrls->error_idx = i;
			return -ERANGE;
		}
	}

	return 0;
}

static int vxe_s_ext_ctrls(struct file *file, void *priv,
			   struct v4l2_ext_controls *ctrls)
{
	struct vxe_enc_ctx *ctx = file2ctx(file);
	struct device *dev = ctx->dev->dev;
	int i;
	int ret;

	ctrls->reserved[0] = 0;
	ctrls->reserved[1] = 0;

	if (ctrls->which != V4L2_CTRL_WHICH_CUR_VAL)
		return -EINVAL;

	/* Verify first with try_ext_ctrls */
	ret = vxe_try_ext_ctrls(file, priv, ctrls);
	if (ret) {
		/* Indicate verification stage error */
		ctrls->error_idx = ctrls->count;
		return ret;
	}

	/* Set all values in this set of commands */
	for (i = 0; i < ctrls->count; i++) {
		ctrls->controls[i].reserved2[0] = 0;

		switch (ctrls->controls[i].id) {
		case V4L2_CID_MPEG_VIDEO_GOP_SIZE:
			ctrls->controls[i].size = 0;
			ctx->vparams.idr_period = ctrls->controls[i].value;
			break;
		case V4L2_CID_MPEG_VIDEO_BITRATE:
			ctrls->controls[i].size = 0;
			ctx->rc.bits_per_second = ctrls->controls[i].value;
			break;
		case V4L2_CID_MPEG_VIDEO_H264_I_PERIOD:
			ctrls->controls[i].size = 0;
			ctx->rc.intra_freq = ctrls->controls[i].value;
			ctx->vparams.intra_cnt = ctrls->controls[i].value;
			break;
		default:
			dev_err(dev, "%s Invalid control id %#08x\n",
				__func__, ctrls->controls[i].id);
			ctrls->error_idx = i;
			return -EINVAL;
		}
	}

	return 0;
}

static int vxe_enum_framesizes(struct file *file, void *priv,
			       struct v4l2_frmsizeenum *fsize)
{
	if (fsize->index != 0)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_CONTINUOUS;
	fsize->stepwise.min_width = 1;
	fsize->stepwise.max_width = 1920;
	fsize->stepwise.step_width = 1;
	fsize->stepwise.min_height = 1;
	fsize->stepwise.max_height = 1080;
	fsize->stepwise.step_height = 1;

	fsize->reserved[0] = 0;
	fsize->reserved[1] = 0;

	return 0;
}

static int vxe_enum_frameintervals(struct file *file, void *priv,
				   struct v4l2_frmivalenum *fival)
{
	if (fival->index)
		return -EINVAL;

	fival->type = V4L2_FRMIVAL_TYPE_CONTINUOUS;
	fival->stepwise.min = frmivals[0];
	fival->stepwise.max = frmivals[1];
	fival->stepwise.step = frmivals[1];

	fival->reserved[0] = 0;
	fival->reserved[1] = 1;

	return 0;
}

static int vxe_g_parm(struct file *file, void *priv,
		      struct v4l2_streamparm *parm)
{
	struct vxe_enc_ctx *ctx = file2ctx(file);

	if (V4L2_TYPE_IS_OUTPUT(parm->type)) {
		parm->parm.output.capability = V4L2_CAP_TIMEPERFRAME;
		parm->parm.output.timeperframe.numerator = 1;
		parm->parm.output.timeperframe.denominator = ctx->rc.frame_rate;
	} else {
		parm->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
		parm->parm.capture.timeperframe.numerator = 1;
		parm->parm.capture.timeperframe.denominator = ctx->rc.frame_rate;
	}

	return 0;
}

static int vxe_s_parm(struct file *file, void *priv,
		      struct v4l2_streamparm *parm)
{
	struct vxe_enc_ctx *ctx = file2ctx(file);
	unsigned int num, den;

	/* Cannot change values once context is created */
	/* TODO: Handle controls after stream is created but before streamon */
	if (ctx->s_fmt_flags & S_FMT_FLAG_STREAM_CREATED)
		return -EBUSY;

	if (V4L2_TYPE_IS_OUTPUT(parm->type)) {
		parm->parm.output.capability = V4L2_CAP_TIMEPERFRAME;
		num = parm->parm.output.timeperframe.numerator;
		den = parm->parm.output.timeperframe.denominator;
	} else {
		parm->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
		num = parm->parm.capture.timeperframe.numerator;
		den = parm->parm.capture.timeperframe.denominator;
	}

	if (parm->parm.output.timeperframe.denominator &&
			parm->parm.output.timeperframe.numerator) {
		ctx->rc.frame_rate = den / num;
	}

	if (V4L2_TYPE_IS_OUTPUT(parm->type)) {
		parm->parm.output.capability = V4L2_CAP_TIMEPERFRAME;
		parm->parm.output.timeperframe.numerator = 1;
		parm->parm.output.timeperframe.denominator = ctx->rc.frame_rate;
	} else {
		parm->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
		parm->parm.capture.timeperframe.numerator = 1;
		parm->parm.capture.timeperframe.denominator = ctx->rc.frame_rate;
	}

	return 0;
}

static const struct v4l2_ioctl_ops vxe_enc_ioctl_ops = {
	.vidioc_querycap = vxe_querycap,

	.vidioc_enum_fmt_vid_cap = vxe_enum_fmt,
	.vidioc_g_fmt_vid_cap_mplane = vxe_g_fmt,
	.vidioc_try_fmt_vid_cap_mplane = vxe_try_fmt,
	.vidioc_s_fmt_vid_cap_mplane = vxe_s_fmt,

	.vidioc_enum_fmt_vid_out = vxe_enum_fmt,
	.vidioc_g_fmt_vid_out_mplane = vxe_g_fmt,
	.vidioc_try_fmt_vid_out_mplane = vxe_try_fmt,
	.vidioc_s_fmt_vid_out_mplane = vxe_s_fmt,

	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,

	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,
	.vidioc_log_status = v4l2_ctrl_log_status,

	.vidioc_subscribe_event = vxe_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
	.vidioc_try_encoder_cmd = vxe_try_cmd,
	.vidioc_encoder_cmd = vxe_cmd,

	.vidioc_queryctrl = vxe_queryctrl,
	.vidioc_query_ext_ctrl = vxe_query_ext_ctrl,
	.vidioc_g_ext_ctrls = vxe_g_ext_ctrls,
	.vidioc_s_ext_ctrls = vxe_s_ext_ctrls,
	.vidioc_try_ext_ctrls = vxe_try_ext_ctrls,

	.vidioc_enum_framesizes = vxe_enum_framesizes,
	.vidioc_enum_frameintervals = vxe_enum_frameintervals,

	.vidioc_g_parm = vxe_g_parm,
	.vidioc_s_parm = vxe_s_parm,
};

static const struct of_device_id vxe_enc_of_match[] = {
	{.compatible = "img,vxe384"},   { /* end */},
};
MODULE_DEVICE_TABLE(of, vxe_enc_of_match);

static irqreturn_t soft_thread_irq(int irq, void *dev_data)
{
	unsigned char handled;

	if (!dev_data)
		return IRQ_NONE;

	handled = topazdd_threaded_isr(dev_data);
	if (handled)
		return IRQ_HANDLED;

	return IRQ_NONE;
}

static irqreturn_t hard_isrcb(int irq, void *dev_data)
{
	if (!dev_data)
		return IRQ_NONE;

	return topazdd_isr(dev_data);
}

static int vxe_enc_probe(struct platform_device *pdev)
{
	struct vxe_dev *vxe;
	struct resource *res;
	const struct of_device_id *of_dev_id;
	struct video_device *vfd;
	int ret;
	int module_irq;
	struct vxe_enc_ctx *ctx;
	struct heap_config *heap_configs = vxe_enc_heap_configs;
	int num_heaps = ARRAY_SIZE(vxe_enc_heap_configs);
	unsigned int i;

	of_dev_id = of_match_device(vxe_enc_of_match, &pdev->dev);
	if (!of_dev_id) {
		dev_err(&pdev->dev, "%s: Unable to match device\n", __func__);
		return -ENODEV;
	}

	dma_set_mask(&pdev->dev, DMA_BIT_MASK(40));

	vxe = devm_kzalloc(&pdev->dev, sizeof(*vxe), GFP_KERNEL);
	if (!vxe)
		return -ENOMEM;

	vxe->dev = &pdev->dev;
	vxe->plat_dev = pdev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	vxe->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(vxe->reg_base))
		return PTR_ERR(vxe->reg_base);

	module_irq = platform_get_irq(pdev, 0);
	if (module_irq < 0)
		return -ENXIO;
	vxe->module_irq = module_irq;

	ret = img_mem_init(vxe->dev);
	if (ret) {
		dev_err(vxe->dev, "Failed to initialize memory\n");
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&vxe->drv_ctx.heaps);
	vxe->drv_ctx.internal_heap_id = VXE_INVALID_ID;

	/* Initialise memory management component */
	for (i = 0; i < num_heaps; i++) {
		struct vxe_heap *heap;
#ifdef DEBUG_ENCODER_DRIVER
		dev_info(vxe->dev, "%s: adding heap of type %d\n",
			 __func__, heap_configs[i].type);
#endif

		heap = kzalloc(sizeof(*heap), GFP_KERNEL);
		if (!heap) {
			ret = -ENOMEM;
			goto heap_add_failed;
		}

		ret = img_mem_add_heap(&heap_configs[i], &heap->id);
		if (ret < 0) {
			dev_err(vxe->dev, "%s: failed to init heap (type %d)!\n",
				__func__, heap_configs[i].type);
			kfree(heap);
			goto heap_add_failed;
		}
		list_add(&heap->list, &vxe->drv_ctx.heaps);

		/* Implicitly, first heap is used for internal allocations */
		if (vxe->drv_ctx.internal_heap_id < 0) {
			vxe->drv_ctx.internal_heap_id = heap->id;
			dev_err(vxe->dev, "%s: using heap %d for internal alloc\n",
				__func__, vxe->drv_ctx.internal_heap_id);
		}
	}

	/* Do not proceed if internal heap not defined */
	if (vxe->drv_ctx.internal_heap_id < 0) {
		dev_err(vxe->dev, "%s: failed to locate heap for internal alloc\n",
			__func__);
		ret = -EINVAL;
		/* Loop registered heaps just for sanity */
		goto heap_add_failed;
	}

	ret = vxe_init_mem(vxe);
	if (ret) {
		dev_err(&pdev->dev, "Failed to initialize memory\n");
		return -ENOMEM;
	}

	vxe->mutex = kzalloc(sizeof(*vxe->mutex), GFP_KERNEL);
	if (!vxe->mutex)
		return -ENOMEM;

	mutex_init(vxe->mutex);

	platform_set_drvdata(pdev, vxe);

	pm_runtime_enable(&pdev->dev);
	ret = pm_runtime_get_sync(&pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "%s: failed to enable clock, status = %d\n",
			__func__, ret);
		goto exit;
	}

	ret = devm_request_threaded_irq(&pdev->dev, module_irq, (irq_handler_t)hard_isrcb,
					(irq_handler_t)soft_thread_irq, IRQF_SHARED,
					IMG_VXE_ENC_MODULE_NAME, &vxe->topaz_dev_ctx);
	if (ret) {
		dev_err(&pdev->dev, "Failed to get IRQ\n");
		goto out_put_sync;
	}

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		free_irq(module_irq, &vxe->topaz_dev_ctx);
		return -ENOMEM;
	}
	ctx->dev = vxe;

	vxe_fill_default_params(ctx);

	ctx->mem_ctx = vxe->drv_ctx.mem_ctx;
	ctx->mmu_ctx = vxe->drv_ctx.mmu_ctx;

	vxe->ctx = ctx;

	ret = topazdd_init((unsigned long)vxe->reg_base, res->end - res->start + 1,
			   (MMU_USE_MMU_FLAG | MMU_EXTENDED_ADDR_FLAG),
			   ctx, vxe->drv_ctx.ptd, &vxe->topaz_dev_ctx);
	if (ret)
		goto out_free_irq;

	vxe->streams = kzalloc(sizeof(*vxe->streams), GFP_KERNEL);
	if (!vxe->streams) {
		ret = -ENOMEM;
		goto topazdd_deinit;
	}
	idr_init(vxe->streams);

	ret = init_topaz_core(vxe->topaz_dev_ctx, &vxe->num_pipes,
			      (MMU_USE_MMU_FLAG | MMU_EXTENDED_ADDR_FLAG),
			      vxe_return_resource);
	if (ret) {
		dev_err(&pdev->dev, "Failed to initialize topaz core\n");
		goto topazdd_deinit;
	}

	ret = v4l2_device_register(&pdev->dev, &vxe->ti_vxe_dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register v4l2 device\n");
		goto topaz_core_deinit;
	}

	vfd = video_device_alloc();
	if (!vfd) {
		dev_err(&pdev->dev, "Failed to allocate video device\n");
		ret = -ENOMEM;
		goto out_v4l2_device;
	}

	snprintf(vfd->name, sizeof(vfd->name), "%s", IMG_VXE_ENC_MODULE_NAME);
	vfd->fops = &vxe_enc_fops;
	vfd->ioctl_ops = &vxe_enc_ioctl_ops;
	vfd->minor = -1;
	vfd->release = video_device_release;
	vfd->vfl_dir = VFL_DIR_M2M;
	vfd->v4l2_dev = &vxe->ti_vxe_dev;
	vfd->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	vfd->lock = vxe->mutex;

	vxe->vfd = vfd;
	video_set_drvdata(vfd, vxe);

	vxe->m2m_dev = v4l2_m2m_init(&m2m_ops);
	if (IS_ERR_VALUE((unsigned long)vxe->m2m_dev)) {
		dev_err(&pdev->dev, "Failed to init mem2mem device\n");
		ret = -EINVAL;
		goto out_vid_dev;
	}

	ret = video_register_device(vfd, VFL_TYPE_VIDEO, 0);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register video device\n");
		goto out_vid_reg;
	}
	v4l2_info(&vxe->ti_vxe_dev, "encoder registered as /dev/video%d\n",
		  vfd->num);

	return 0;

out_vid_reg:
	v4l2_m2m_release(vxe->m2m_dev);
out_vid_dev:
	video_device_release(vfd);
out_v4l2_device:
	v4l2_device_unregister(&vxe->ti_vxe_dev);
topaz_core_deinit:
	deinit_topaz_core();
topazdd_deinit:
	topazdd_deinit(vxe->topaz_dev_ctx);
out_free_irq:
	kfree(vxe->ctx);
	free_irq(module_irq, &vxe->topaz_dev_ctx);
out_put_sync:
	pm_runtime_put_sync(&pdev->dev);
heap_add_failed:
	while (!list_empty(&vxe->drv_ctx.heaps)) {
		struct vxe_heap *heap;

		heap = list_first_entry(&vxe->drv_ctx.heaps, struct vxe_heap, list);
		__list_del_entry(&heap->list);
		img_mem_del_heap(heap->id);
		kfree(heap);
	}
	vxe->drv_ctx.internal_heap_id = VXE_INVALID_ID;

exit:
	pm_runtime_disable(&pdev->dev);
	vxe_deinit_mem(vxe);

	return ret;
}

static int vxe_enc_remove(struct platform_device *pdev)
{
	struct vxe_dev *vxe = platform_get_drvdata(pdev);

	topazdd_deinit(vxe->topaz_dev_ctx);

	kfree(vxe->ctx);
	vxe_deinit_mem(vxe);

	free_irq(vxe->module_irq, &vxe->topaz_dev_ctx);

	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

static struct platform_driver vxe_enc_driver = {
	.probe = vxe_enc_probe,
	.remove = vxe_enc_remove,
	.driver = {
		.name = "img_enc",
		.of_match_table = vxe_enc_of_match,
	},
};
module_platform_driver(vxe_enc_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("IMG VXE384 video encoder driver");
