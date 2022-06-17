// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * Wave5 series multi-standard codec IP - decoder interface
 *
 * Copyright (C) 2021 CHIPS&MEDIA INC
 */

#include "wave5-vpu.h"

#define VPU_DEC_DEV_NAME "C&M VPU decoder"
#define VPU_DEC_DRV_NAME "vpu-dec"
#define V4L2_CID_VPU_THUMBNAIL_MODE (V4L2_CID_USER_BASE + 0x1001)

static const struct vpu_format wave5_vpu_dec_fmt_list[2][6] = {
	[VPU_FMT_TYPE_CODEC] = {
		{
			.v4l2_pix_fmt = V4L2_PIX_FMT_HEVC,
			.max_width = 8192,
			.min_width = 8,
			.max_height = 8192,
			.min_height = 8,
		},
		{
			.v4l2_pix_fmt = V4L2_PIX_FMT_H264,
			.max_width = 8192,
			.min_width = 32,
			.max_height = 8192,
			.min_height = 32,
		},
	},
	[VPU_FMT_TYPE_RAW] = {
		{
			.v4l2_pix_fmt = V4L2_PIX_FMT_YUV420,
			.max_width = 8192,
			.min_width = 8,
			.max_height = 8192,
			.min_height = 8,
		},
		{
			.v4l2_pix_fmt = V4L2_PIX_FMT_NV12,
			.max_width = 8192,
			.min_width = 8,
			.max_height = 8192,
			.min_height = 8,
		},
		{
			.v4l2_pix_fmt = V4L2_PIX_FMT_NV21,
			.max_width = 8192,
			.min_width = 8,
			.max_height = 8192,
			.min_height = 8,
		},
		{
			.v4l2_pix_fmt = V4L2_PIX_FMT_YUV420M,
			.max_width = 8192,
			.min_width = 8,
			.max_height = 8192,
			.min_height = 8,
		},
		{
			.v4l2_pix_fmt = V4L2_PIX_FMT_NV12M,
			.max_width = 8192,
			.min_width = 8,
			.max_height = 8192,
			.min_height = 8,
		},
		{
			.v4l2_pix_fmt = V4L2_PIX_FMT_NV21M,
			.max_width = 8192,
			.min_width = 8,
			.max_height = 8192,
			.min_height = 8,
		},
	}
};

static enum wave_std wave5_to_vpu_codstd(unsigned int v4l2_pix_fmt)
{
	switch (v4l2_pix_fmt) {
	case V4L2_PIX_FMT_H264:
		return W_AVC_DEC;
	case V4L2_PIX_FMT_HEVC:
		return W_HEVC_DEC;
	default:
		return STD_UNKNOWN;
	}
}

static const struct vpu_format *wave5_get_vpu_fmt(unsigned int v4l2_pix_fmt, enum vpu_fmt_type type)
{
	unsigned int index;

	for (index = 0; index < ARRAY_SIZE(wave5_vpu_dec_fmt_list[type]); index++) {
		if (wave5_vpu_dec_fmt_list[type][index].v4l2_pix_fmt == v4l2_pix_fmt)
			return &wave5_vpu_dec_fmt_list[type][index];
	}

	return NULL;
}

static const struct vpu_format *wave5_find_vpu_fmt_by_idx(unsigned int idx, enum vpu_fmt_type type)
{
	if (idx >= ARRAY_SIZE(wave5_vpu_dec_fmt_list[type]))
		return NULL;

	if (!wave5_vpu_dec_fmt_list[type][idx].v4l2_pix_fmt)
		return NULL;

	return &wave5_vpu_dec_fmt_list[type][idx];
}

static void wave5_handle_bitstream_buffer(struct vpu_instance *inst)
{
	struct v4l2_m2m_buffer *v4l2_m2m_buf = NULL;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&inst->bitstream_lock, flags);

	v4l2_m2m_for_each_src_buf(inst->v4l2_fh.m2m_ctx, v4l2_m2m_buf) {
		struct vb2_v4l2_buffer *vbuf = &v4l2_m2m_buf->vb;
		struct vpu_buffer *vpu_buf = wave5_to_vpu_buf(vbuf);
		u32 src_size = vb2_get_plane_payload(&vbuf->vb2_buf, 0);
		void *src_buf = vb2_plane_vaddr(&vbuf->vb2_buf, 0);
		dma_addr_t bs_rd_ptr = 0;
		dma_addr_t bs_wr_ptr = 0;
		u32 bs_remain_size = 0;
		size_t offset;

		if (vpu_buf->consumed) {
			dev_dbg(inst->dev->dev, "already consumed buffer\n");
			continue;
		}

		wave5_vpu_dec_get_bitstream_buffer(inst, &bs_rd_ptr, &bs_wr_ptr, &bs_remain_size);

		if (bs_remain_size < src_size) {
			dev_dbg(inst->dev->dev, "fill next time : remained size < source size.\n");
			continue;
		}

		offset = bs_wr_ptr - inst->bitstream_vbuf.daddr;
		if (bs_wr_ptr + src_size > inst->bitstream_vbuf.daddr + inst->bitstream_vbuf.size) {
			int size;

			size = inst->bitstream_vbuf.daddr + inst->bitstream_vbuf.size - bs_wr_ptr;
			wave5_vdi_write_memory(inst->dev, &inst->bitstream_vbuf, offset, src_buf,
					       size, VDI_128BIT_LITTLE_ENDIAN);
			wave5_vdi_write_memory(inst->dev, &inst->bitstream_vbuf, 0,
					       src_buf + size, src_size - size,
					       VDI_128BIT_LITTLE_ENDIAN);
		} else {
			wave5_vdi_write_memory(inst->dev, &inst->bitstream_vbuf, offset, src_buf,
					       src_size, VDI_128BIT_LITTLE_ENDIAN);
		}

		ret = wave5_vpu_dec_update_bitstream_buffer(inst, src_size);
		if (ret) {
			dev_dbg(inst->dev->dev, "vpu_dec_update_bitstream_buffer fail: %d\n", ret);
			continue;
		}

		vpu_buf->consumed = true;

		if (inst->state == VPU_INST_STATE_WAIT_BUF)
			inst->state = VPU_INST_STATE_PIC_RUN;
	}

	spin_unlock_irqrestore(&inst->bitstream_lock, flags);
}

static void wave5_handle_src_buffer(struct vpu_instance *inst)
{
	struct vb2_v4l2_buffer *src_buf;

	unsigned long flags;

	spin_lock_irqsave(&inst->bitstream_lock, flags);

	src_buf = v4l2_m2m_next_src_buf(inst->v4l2_fh.m2m_ctx);
	if (src_buf) {
		struct vpu_buffer *vpu_buf = wave5_to_vpu_buf(src_buf);

		if (vpu_buf->consumed) {
			dev_dbg(inst->dev->dev, "already consumed buffer\n");
			src_buf = v4l2_m2m_src_buf_remove(inst->v4l2_fh.m2m_ctx);
			inst->timestamp = src_buf->vb2_buf.timestamp;
			v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_DONE);
		}
	}
	spin_unlock_irqrestore(&inst->bitstream_lock, flags);
}

static void wave5_update_pix_fmt(struct v4l2_pix_format_mplane *pix_mp, unsigned int width,
				 unsigned int height)
{
	switch (pix_mp->pixelformat) {
	case V4L2_PIX_FMT_YUV420:
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV21:
		pix_mp->width = round_up(width, 32);
		pix_mp->height = height;
		pix_mp->plane_fmt[0].bytesperline = round_up(width, 32);
		pix_mp->plane_fmt[0].sizeimage = width * height * 3 / 2;
		memset(&pix_mp->plane_fmt[0].reserved, 0, sizeof(pix_mp->plane_fmt[0].reserved));
		break;
	case V4L2_PIX_FMT_YUV420M:
		pix_mp->width = round_up(width, 32);
		pix_mp->height = height;
		pix_mp->plane_fmt[0].bytesperline = round_up(width, 32);
		pix_mp->plane_fmt[0].sizeimage = width * height;
		memset(&pix_mp->plane_fmt[0].reserved, 0, sizeof(pix_mp->plane_fmt[0].reserved));
		pix_mp->plane_fmt[1].bytesperline = round_up(width, 32) / 2;
		pix_mp->plane_fmt[1].sizeimage = width * height / 4;
		memset(&pix_mp->plane_fmt[1].reserved, 0, sizeof(pix_mp->plane_fmt[1].reserved));
		pix_mp->plane_fmt[2].bytesperline = round_up(width, 32) / 2;
		pix_mp->plane_fmt[2].sizeimage = width * height / 4;
		memset(&pix_mp->plane_fmt[2].reserved, 0, sizeof(pix_mp->plane_fmt[2].reserved));
		break;
	case V4L2_PIX_FMT_NV12M:
	case V4L2_PIX_FMT_NV21M:
		pix_mp->width = round_up(width, 32);
		pix_mp->height = height;
		pix_mp->plane_fmt[0].bytesperline = round_up(width, 32);
		pix_mp->plane_fmt[0].sizeimage = width * height;
		memset(&pix_mp->plane_fmt[0].reserved, 0, sizeof(pix_mp->plane_fmt[0].reserved));
		pix_mp->plane_fmt[1].bytesperline = round_up(width, 32);
		pix_mp->plane_fmt[1].sizeimage = width * height / 2;
		memset(&pix_mp->plane_fmt[1].reserved, 0, sizeof(pix_mp->plane_fmt[1].reserved));
		break;
	default:
		pix_mp->width = width;
		pix_mp->height = height;
		pix_mp->plane_fmt[0].bytesperline = 0;
		pix_mp->plane_fmt[0].sizeimage = width * height / 2;
		memset(&pix_mp->plane_fmt[0].reserved, 0, sizeof(pix_mp->plane_fmt[0].reserved));
		break;
	}
}

static void wave5_vpu_dec_start_decode(struct vpu_instance *inst)
{
	struct dec_param pic_param;
	u32 max_cmd_q = 0;
	int ret;

	memset(&pic_param, 0, sizeof(struct dec_param));

	if (inst->state == VPU_INST_STATE_STOP) {
		max_cmd_q = 1;
	} else {
		max_cmd_q = (inst->min_src_frame_buf_count < COMMAND_QUEUE_DEPTH) ?
			inst->min_src_frame_buf_count : COMMAND_QUEUE_DEPTH;
	}

	while (max_cmd_q) {
		u32 fail_res = 0;

		ret = wave5_vpu_dec_start_one_frame(inst, &pic_param, &fail_res);
		if (ret) {
			struct vb2_v4l2_buffer *src_buf;

			if (fail_res == WAVE5_SYSERR_QUEUEING_FAIL)
				break;

			src_buf = v4l2_m2m_src_buf_remove(inst->v4l2_fh.m2m_ctx);
			inst->state = VPU_INST_STATE_STOP;
			v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_ERROR);
			break;
		}
		max_cmd_q--;
	}
}

static void wave5_vpu_dec_stop_decode(struct vpu_instance *inst)
{
	unsigned int i;
	unsigned long flags;

	inst->state = VPU_INST_STATE_STOP;

	spin_lock_irqsave(&inst->bitstream_lock, flags);
	wave5_vpu_dec_update_bitstream_buffer(inst, 0);
	spin_unlock_irqrestore(&inst->bitstream_lock, flags);

	for (i = 0; i < inst->min_dst_frame_buf_count; i++)
		wave5_vpu_dec_clr_disp_flag(inst, i);

	v4l2_m2m_job_finish(inst->dev->v4l2_m2m_dev, inst->v4l2_fh.m2m_ctx);
}

static void wave5_vpu_dec_finish_decode(struct vpu_instance *inst)
{
	struct dec_output_info dec_output_info;
	int ret;
	int irq_status;

	if (kfifo_out(&inst->dev->irq_status, &irq_status, sizeof(int)))
		wave5_vpu_clear_interrupt_ex(inst, irq_status);

	if (irq_status & BIT(INT_WAVE5_BSBUF_EMPTY)) {
		dev_dbg(inst->dev->dev, "bitstream EMPTY!!!!\n");
		inst->state = VPU_INST_STATE_WAIT_BUF;
		wave5_handle_src_buffer(inst);
		wave5_handle_bitstream_buffer(inst);
	}

	if (!(irq_status & BIT(INT_WAVE5_DEC_PIC)))
		return;
	ret = wave5_vpu_dec_get_output_info(inst, &dec_output_info);
	if (ret) {
		v4l2_m2m_job_finish(inst->dev->v4l2_m2m_dev, inst->v4l2_fh.m2m_ctx);
		return;
	}
	if (dec_output_info.index_frame_decoded == DECODED_IDX_FLAG_NO_FB &&
	    dec_output_info.index_frame_display == DISPLAY_IDX_FLAG_NO_FB) {
		dev_dbg(inst->dev->dev, "no more frame buffer\n");
		if (inst->state != VPU_INST_STATE_STOP)
			inst->state = VPU_INST_STATE_WAIT_BUF;
	} else {
		wave5_handle_src_buffer(inst);

		if (dec_output_info.index_frame_display >= 0) {
			struct vb2_v4l2_buffer *dst_buf =
				v4l2_m2m_dst_buf_remove_by_idx(inst->v4l2_fh.m2m_ctx,
							       dec_output_info.index_frame_display);
			int stride = dec_output_info.disp_frame.stride;
			int height = dec_output_info.disp_pic_height;

			if (inst->dst_fmt.num_planes == 1) {
				vb2_set_plane_payload(&dst_buf->vb2_buf, 0,
						      (stride * height * 3 / 2));
			} else if (inst->dst_fmt.num_planes == 2) {
				vb2_set_plane_payload(&dst_buf->vb2_buf, 0,
						      (stride * height));
				vb2_set_plane_payload(&dst_buf->vb2_buf, 1,
						      ((stride / 2) * height));
			} else if (inst->dst_fmt.num_planes == 3) {
				vb2_set_plane_payload(&dst_buf->vb2_buf, 0,
						      (stride * height));
				vb2_set_plane_payload(&dst_buf->vb2_buf, 1,
						      ((stride / 2) * (height / 2)));
				vb2_set_plane_payload(&dst_buf->vb2_buf, 2,
						      ((stride / 2) * (height / 2)));
			}

			dst_buf->vb2_buf.timestamp = inst->timestamp;
			dst_buf->field = V4L2_FIELD_NONE;
			v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_DONE);
		} else if (dec_output_info.index_frame_display == DISPLAY_IDX_FLAG_SEQ_END) {
			static const struct v4l2_event vpu_event_eos = {
				.type = V4L2_EVENT_EOS
			};
			struct vb2_v4l2_buffer *dst_buf =
				v4l2_m2m_dst_buf_remove(inst->v4l2_fh.m2m_ctx);

			if (!dst_buf)
				return;

			if (inst->dst_fmt.num_planes == 1) {
				vb2_set_plane_payload(&dst_buf->vb2_buf, 0,
						      vb2_plane_size(&dst_buf->vb2_buf, 0));
			} else if (inst->dst_fmt.num_planes == 2) {
				vb2_set_plane_payload(&dst_buf->vb2_buf, 0,
						      vb2_plane_size(&dst_buf->vb2_buf, 0));
				vb2_set_plane_payload(&dst_buf->vb2_buf, 1,
						      vb2_plane_size(&dst_buf->vb2_buf, 1));
			} else if (inst->dst_fmt.num_planes == 3) {
				vb2_set_plane_payload(&dst_buf->vb2_buf, 0,
						      vb2_plane_size(&dst_buf->vb2_buf, 0));
				vb2_set_plane_payload(&dst_buf->vb2_buf, 1,
						      vb2_plane_size(&dst_buf->vb2_buf, 1));
				vb2_set_plane_payload(&dst_buf->vb2_buf, 2,
						      vb2_plane_size(&dst_buf->vb2_buf, 2));
			}

			dst_buf->vb2_buf.timestamp = inst->timestamp;
			dst_buf->flags |= V4L2_BUF_FLAG_LAST;
			dst_buf->field = V4L2_FIELD_NONE;
			v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_DONE);

			inst->state = VPU_INST_STATE_PIC_RUN;
			v4l2_event_queue_fh(&inst->v4l2_fh, &vpu_event_eos);

			v4l2_m2m_job_finish(inst->dev->v4l2_m2m_dev, inst->v4l2_fh.m2m_ctx);
		}
	}
}

static int wave5_vpu_dec_querycap(struct file *file, void *fh, struct v4l2_capability *cap)
{
	strscpy(cap->driver, VPU_DEC_DRV_NAME, sizeof(cap->driver));
	strscpy(cap->card, VPU_DEC_DRV_NAME, sizeof(cap->card));
	strscpy(cap->bus_info, "platform:" VPU_DEC_DRV_NAME, sizeof(cap->bus_info));

	return 0;
}

static int wave5_vpu_dec_enum_framesizes(struct file *f, void *fh, struct v4l2_frmsizeenum *fsize)
{
	const struct vpu_format *vpu_fmt;

	if (fsize->index)
		return -EINVAL;

	vpu_fmt = wave5_get_vpu_fmt(fsize->pixel_format, VPU_FMT_TYPE_CODEC);
	if (!vpu_fmt) {
		vpu_fmt = wave5_get_vpu_fmt(fsize->pixel_format, VPU_FMT_TYPE_RAW);
		if (!vpu_fmt)
			return -EINVAL;
	}

	fsize->type = V4L2_FRMSIZE_TYPE_CONTINUOUS;
	fsize->stepwise.min_width = vpu_fmt->min_width;
	fsize->stepwise.max_width = vpu_fmt->max_width;
	fsize->stepwise.step_width = 1;
	fsize->stepwise.min_height = vpu_fmt->min_height;
	fsize->stepwise.max_height = vpu_fmt->max_height;
	fsize->stepwise.step_height = 1;

	return 0;
}

static int wave5_vpu_dec_enum_fmt_cap(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	const struct vpu_format *vpu_fmt;

	vpu_fmt = wave5_find_vpu_fmt_by_idx(f->index, VPU_FMT_TYPE_RAW);
	if (!vpu_fmt)
		return -EINVAL;

	f->pixelformat = vpu_fmt->v4l2_pix_fmt;
	f->flags = 0;

	return 0;
}

static int wave5_vpu_dec_try_fmt_cap(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vpu_instance *inst = wave5_to_vpu_inst(fh);
	const struct vpu_format *vpu_fmt;

	dev_dbg(inst->dev->dev, "4cc %d width %d height %d nm planes %d colorspace %d field : %d\n",
		f->fmt.pix_mp.pixelformat, f->fmt.pix_mp.width, f->fmt.pix_mp.height,
		f->fmt.pix_mp.num_planes, f->fmt.pix_mp.colorspace, f->fmt.pix_mp.field);

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;

	vpu_fmt = wave5_get_vpu_fmt(f->fmt.pix_mp.pixelformat, VPU_FMT_TYPE_RAW);
	if (!vpu_fmt) {
		f->fmt.pix_mp.pixelformat = inst->dst_fmt.pixelformat;
		f->fmt.pix_mp.num_planes = inst->dst_fmt.num_planes;
		wave5_update_pix_fmt(&f->fmt.pix_mp, inst->dst_fmt.width, inst->dst_fmt.height);
	} else {
		int width = clamp(f->fmt.pix_mp.width, vpu_fmt->min_width, vpu_fmt->max_width);
		int height = clamp(f->fmt.pix_mp.height, vpu_fmt->min_height, vpu_fmt->max_height);
		const struct v4l2_format_info *info = v4l2_format_info(vpu_fmt->v4l2_pix_fmt);

		f->fmt.pix_mp.pixelformat = vpu_fmt->v4l2_pix_fmt;
		f->fmt.pix_mp.num_planes = info->mem_planes;
		wave5_update_pix_fmt(&f->fmt.pix_mp, width, height);
	}

	f->fmt.pix_mp.flags = 0;
	f->fmt.pix_mp.field = V4L2_FIELD_NONE;
	f->fmt.pix_mp.colorspace = inst->colorspace;
	f->fmt.pix_mp.ycbcr_enc = inst->ycbcr_enc;
	f->fmt.pix_mp.hsv_enc = inst->hsv_enc;
	f->fmt.pix_mp.quantization = inst->quantization;
	f->fmt.pix_mp.xfer_func = inst->xfer_func;
	memset(&f->fmt.pix_mp.reserved, 0, sizeof(f->fmt.pix_mp.reserved));

	return 0;
}

static int wave5_vpu_dec_s_fmt_cap(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vpu_instance *inst = wave5_to_vpu_inst(fh);
	int i, ret;

	dev_dbg(inst->dev->dev, "4cc %d width %d height %d num_planes %d colorspace %d field %d\n",
		f->fmt.pix_mp.pixelformat, f->fmt.pix_mp.width, f->fmt.pix_mp.height,
		f->fmt.pix_mp.num_planes, f->fmt.pix_mp.colorspace, f->fmt.pix_mp.field);

	ret = wave5_vpu_dec_try_fmt_cap(file, fh, f);
	if (ret)
		return ret;

	inst->dst_fmt.width = f->fmt.pix_mp.width;
	inst->dst_fmt.height = f->fmt.pix_mp.height;
	inst->dst_fmt.pixelformat = f->fmt.pix_mp.pixelformat;
	inst->dst_fmt.field = f->fmt.pix_mp.field;
	inst->dst_fmt.flags = f->fmt.pix_mp.flags;
	inst->dst_fmt.num_planes = f->fmt.pix_mp.num_planes;
	for (i = 0; i < inst->dst_fmt.num_planes; i++) {
		inst->dst_fmt.plane_fmt[i].bytesperline = f->fmt.pix_mp.plane_fmt[i].bytesperline;
		inst->dst_fmt.plane_fmt[i].sizeimage = f->fmt.pix_mp.plane_fmt[i].sizeimage;
	}

	if (inst->dst_fmt.pixelformat == V4L2_PIX_FMT_NV12 ||
	    inst->dst_fmt.pixelformat == V4L2_PIX_FMT_NV12M) {
		inst->cbcr_interleave = true;
		inst->nv21 = false;
	} else if (inst->dst_fmt.pixelformat == V4L2_PIX_FMT_NV21 ||
		   inst->dst_fmt.pixelformat == V4L2_PIX_FMT_NV21M) {
		inst->cbcr_interleave = true;
		inst->nv21 = true;
	} else {
		inst->cbcr_interleave = false;
		inst->nv21 = false;
	}

	return 0;
}

static int wave5_vpu_dec_g_fmt_cap(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vpu_instance *inst = wave5_to_vpu_inst(fh);
	int i;

	f->fmt.pix_mp.width = inst->dst_fmt.width;
	f->fmt.pix_mp.height = inst->dst_fmt.height;
	f->fmt.pix_mp.pixelformat = inst->dst_fmt.pixelformat;
	f->fmt.pix_mp.field = inst->dst_fmt.field;
	f->fmt.pix_mp.flags = inst->dst_fmt.flags;
	f->fmt.pix_mp.num_planes = inst->dst_fmt.num_planes;
	for (i = 0; i < f->fmt.pix_mp.num_planes; i++) {
		f->fmt.pix_mp.plane_fmt[i].bytesperline = inst->dst_fmt.plane_fmt[i].bytesperline;
		f->fmt.pix_mp.plane_fmt[i].sizeimage = inst->dst_fmt.plane_fmt[i].sizeimage;
	}

	f->fmt.pix_mp.colorspace = inst->colorspace;
	f->fmt.pix_mp.ycbcr_enc = inst->ycbcr_enc;
	f->fmt.pix_mp.hsv_enc = inst->hsv_enc;
	f->fmt.pix_mp.quantization = inst->quantization;
	f->fmt.pix_mp.xfer_func = inst->xfer_func;

	return 0;
}

static int wave5_vpu_dec_enum_fmt_out(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	struct vpu_instance *inst = wave5_to_vpu_inst(fh);
	const struct vpu_format *vpu_fmt;

	dev_dbg(inst->dev->dev, "index : %d\n", f->index);

	vpu_fmt = wave5_find_vpu_fmt_by_idx(f->index, VPU_FMT_TYPE_CODEC);
	if (!vpu_fmt)
		return -EINVAL;

	f->pixelformat = vpu_fmt->v4l2_pix_fmt;
	f->flags = 0;

	return 0;
}

static int wave5_vpu_dec_try_fmt_out(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vpu_instance *inst = wave5_to_vpu_inst(fh);
	const struct vpu_format *vpu_fmt;

	dev_dbg(inst->dev->dev, "4cc %d width %d height %d num_planes %d colorspace %d field %d\n",
		f->fmt.pix_mp.pixelformat, f->fmt.pix_mp.width, f->fmt.pix_mp.height,
		f->fmt.pix_mp.num_planes, f->fmt.pix_mp.colorspace, f->fmt.pix_mp.field);

	if (f->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return -EINVAL;

	vpu_fmt = wave5_get_vpu_fmt(f->fmt.pix_mp.pixelformat, VPU_FMT_TYPE_CODEC);
	if (!vpu_fmt) {
		f->fmt.pix_mp.pixelformat = inst->src_fmt.pixelformat;
		f->fmt.pix_mp.num_planes = inst->src_fmt.num_planes;
		wave5_update_pix_fmt(&f->fmt.pix_mp, inst->src_fmt.width, inst->src_fmt.height);
	} else {
		int width = clamp(f->fmt.pix_mp.width, vpu_fmt->min_width, vpu_fmt->max_width);
		int height = clamp(f->fmt.pix_mp.height, vpu_fmt->min_height, vpu_fmt->max_height);

		f->fmt.pix_mp.pixelformat = vpu_fmt->v4l2_pix_fmt;
		f->fmt.pix_mp.num_planes = 1;
		wave5_update_pix_fmt(&f->fmt.pix_mp, width, height);
	}

	f->fmt.pix_mp.flags = 0;
	f->fmt.pix_mp.field = V4L2_FIELD_NONE;
	memset(&f->fmt.pix_mp.reserved, 0, sizeof(f->fmt.pix_mp.reserved));

	return 0;
}

static int wave5_vpu_dec_s_fmt_out(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vpu_instance *inst = wave5_to_vpu_inst(fh);
	int i, ret;

	dev_dbg(inst->dev->dev, "pixelformat %d width %d height %d num_planes %d field : %d\n",
		f->fmt.pix_mp.pixelformat, f->fmt.pix_mp.width, f->fmt.pix_mp.height,
		f->fmt.pix_mp.num_planes, f->fmt.pix_mp.field);

	ret = wave5_vpu_dec_try_fmt_out(file, fh, f);
	if (ret)
		return ret;

	inst->src_fmt.width = f->fmt.pix_mp.width;
	inst->src_fmt.height = f->fmt.pix_mp.height;
	inst->src_fmt.pixelformat = f->fmt.pix_mp.pixelformat;
	inst->src_fmt.field = f->fmt.pix_mp.field;
	inst->src_fmt.flags = f->fmt.pix_mp.flags;
	inst->src_fmt.num_planes = f->fmt.pix_mp.num_planes;
	for (i = 0; i < inst->src_fmt.num_planes; i++) {
		inst->src_fmt.plane_fmt[i].bytesperline = f->fmt.pix_mp.plane_fmt[i].bytesperline;
		inst->src_fmt.plane_fmt[i].sizeimage = f->fmt.pix_mp.plane_fmt[i].sizeimage;
	}

	inst->colorspace = f->fmt.pix_mp.colorspace;
	inst->ycbcr_enc = f->fmt.pix_mp.ycbcr_enc;
	inst->hsv_enc = f->fmt.pix_mp.hsv_enc;
	inst->quantization = f->fmt.pix_mp.quantization;
	inst->xfer_func = f->fmt.pix_mp.xfer_func;

	wave5_update_pix_fmt(&inst->dst_fmt, f->fmt.pix_mp.width, f->fmt.pix_mp.height);

	return 0;
}

static int wave5_vpu_dec_g_fmt_out(struct file *file, void *fh, struct v4l2_format *f)
{
	struct vpu_instance *inst = wave5_to_vpu_inst(fh);
	int i;

	f->fmt.pix_mp.width = inst->src_fmt.width;
	f->fmt.pix_mp.height = inst->src_fmt.height;
	f->fmt.pix_mp.pixelformat = inst->src_fmt.pixelformat;
	f->fmt.pix_mp.field = inst->src_fmt.field;
	f->fmt.pix_mp.flags = inst->src_fmt.flags;
	f->fmt.pix_mp.num_planes = inst->src_fmt.num_planes;
	for (i = 0; i < f->fmt.pix_mp.num_planes; i++) {
		f->fmt.pix_mp.plane_fmt[i].bytesperline = inst->src_fmt.plane_fmt[i].bytesperline;
		f->fmt.pix_mp.plane_fmt[i].sizeimage = inst->src_fmt.plane_fmt[i].sizeimage;
	}

	f->fmt.pix_mp.colorspace = inst->colorspace;
	f->fmt.pix_mp.ycbcr_enc = inst->ycbcr_enc;
	f->fmt.pix_mp.hsv_enc = inst->hsv_enc;
	f->fmt.pix_mp.quantization = inst->quantization;
	f->fmt.pix_mp.xfer_func = inst->xfer_func;

	return 0;
}

static int wave5_vpu_dec_g_selection(struct file *file, void *fh, struct v4l2_selection *s)
{
	struct vpu_instance *inst = wave5_to_vpu_inst(fh);

	dev_dbg(inst->dev->dev, "type : %d | target : %d\n", s->type, s->target);

	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	switch (s->target) {
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
	case V4L2_SEL_TGT_COMPOSE_PADDED:
		s->r.left = 0;
		s->r.top = 0;
		s->r.width = inst->dst_fmt.width;
		s->r.height = inst->dst_fmt.height;
		break;
	case V4L2_SEL_TGT_COMPOSE:
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
		s->r.left = 0;
		s->r.top = 0;
		s->r.width = inst->src_fmt.width;
		s->r.height = inst->src_fmt.height;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int wave5_vpu_dec_s_selection(struct file *file, void *fh, struct v4l2_selection *s)
{
	struct vpu_instance *inst = wave5_to_vpu_inst(fh);

	dev_dbg(inst->dev->dev, "type : %d | target : %d\n", s->type, s->target);

	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	if (s->target != V4L2_SEL_TGT_COMPOSE)
		return -EINVAL;

	dev_dbg(inst->dev->dev, "V4L2_SEL_TGT_COMPOSE w: %d h: %d\n", s->r.width, s->r.height);
	inst->dst_fmt.width = s->r.width;
	inst->dst_fmt.height = s->r.height;

	return 0;
}

static int wave5_vpu_dec_try_decoder_cmd(struct file *file, void *fh, struct v4l2_decoder_cmd *dc)
{
	struct vpu_instance *inst = wave5_to_vpu_inst(fh);

	dev_dbg(inst->dev->dev, "decoder command : %d\n", dc->cmd);

	if (dc->cmd != V4L2_DEC_CMD_STOP && dc->cmd != V4L2_DEC_CMD_START)
		return -EINVAL;

	dc->flags = 0;

	if (dc->cmd == V4L2_DEC_CMD_STOP) {
		dc->stop.pts = 0;
	} else if (dc->cmd == V4L2_DEC_CMD_START) {
		dc->start.speed = 0;
		dc->start.format = V4L2_DEC_START_FMT_NONE;
	}

	return 0;
}

static int wave5_vpu_dec_decoder_cmd(struct file *file, void *fh, struct v4l2_decoder_cmd *dc)
{
	struct vpu_instance *inst = wave5_to_vpu_inst(fh);
	int ret;
	unsigned long flags;

	dev_dbg(inst->dev->dev, "decoder command : %d\n", dc->cmd);

	ret = wave5_vpu_dec_try_decoder_cmd(file, fh, dc);
	if (ret < 0)
		return ret;

	if (!wave5_vpu_both_queues_are_streaming(inst))
		return 0;

	switch (dc->cmd) {
	case V4L2_DEC_CMD_STOP:
		inst->state = VPU_INST_STATE_STOP;

		spin_lock_irqsave(&inst->bitstream_lock, flags);
		wave5_vpu_dec_update_bitstream_buffer(inst, 0);
		spin_unlock_irqrestore(&inst->bitstream_lock, flags);
		break;
	case V4L2_DEC_CMD_START:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int wave5_vpu_dec_subscribe_event(struct v4l2_fh *fh,
					 const struct v4l2_event_subscription *sub)
{
	struct vpu_instance *inst = wave5_to_vpu_inst(fh);

	dev_dbg(inst->dev->dev, "type : %d id : %d | flags : %d\n", sub->type, sub->id, sub->flags);

	switch (sub->type) {
	case V4L2_EVENT_EOS:
		return v4l2_event_subscribe(fh, sub, 0, NULL);
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subscribe(fh, sub);
	case V4L2_EVENT_CTRL:
		return v4l2_ctrl_subscribe_event(fh, sub);
	default:
		return -EINVAL;
	}
}

static const struct v4l2_ioctl_ops wave5_vpu_dec_ioctl_ops = {
	.vidioc_querycap = wave5_vpu_dec_querycap,
	.vidioc_enum_framesizes = wave5_vpu_dec_enum_framesizes,

	.vidioc_enum_fmt_vid_cap	= wave5_vpu_dec_enum_fmt_cap,
	.vidioc_s_fmt_vid_cap_mplane = wave5_vpu_dec_s_fmt_cap,
	.vidioc_g_fmt_vid_cap_mplane = wave5_vpu_dec_g_fmt_cap,
	.vidioc_try_fmt_vid_cap_mplane = wave5_vpu_dec_try_fmt_cap,

	.vidioc_enum_fmt_vid_out	= wave5_vpu_dec_enum_fmt_out,
	.vidioc_s_fmt_vid_out_mplane = wave5_vpu_dec_s_fmt_out,
	.vidioc_g_fmt_vid_out_mplane = wave5_vpu_dec_g_fmt_out,
	.vidioc_try_fmt_vid_out_mplane = wave5_vpu_dec_try_fmt_out,

	.vidioc_g_selection = wave5_vpu_dec_g_selection,
	.vidioc_s_selection = wave5_vpu_dec_s_selection,

	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,

	.vidioc_try_decoder_cmd = wave5_vpu_dec_try_decoder_cmd,
	.vidioc_decoder_cmd = wave5_vpu_dec_decoder_cmd,

	.vidioc_subscribe_event = wave5_vpu_dec_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static int wave5_vpu_dec_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct vpu_instance *inst = wave5_ctrl_to_vpu_inst(ctrl);

	dev_dbg(inst->dev->dev, "name : %s\n", ctrl->name);

	switch (ctrl->id) {
	case V4L2_CID_MIN_BUFFERS_FOR_CAPTURE:
		if (inst->state != VPU_INST_STATE_NONE && inst->state != VPU_INST_STATE_OPEN)
			ctrl->val = inst->min_dst_frame_buf_count;
		break;
	default:
		return -EINVAL;
	}

	dev_dbg(inst->dev->dev, "value : %d\n", ctrl->val);

	return 0;
}

static int wave5_vpu_dec_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct vpu_instance *inst = wave5_ctrl_to_vpu_inst(ctrl);

	dev_dbg(inst->dev->dev, "name : %s | value : %d\n", ctrl->name, ctrl->val);

	switch (ctrl->id) {
	case V4L2_CID_VPU_THUMBNAIL_MODE:
		inst->thumbnail_mode = ctrl->val;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_ctrl_ops wave5_vpu_dec_ctrl_ops = {
	.g_volatile_ctrl = wave5_vpu_dec_g_volatile_ctrl,
	.s_ctrl = wave5_vpu_dec_s_ctrl,
};

static const struct v4l2_ctrl_config wave5_vpu_thumbnail_mode = {
	.ops = &wave5_vpu_dec_ctrl_ops,
	.id = V4L2_CID_VPU_THUMBNAIL_MODE,
	.name = "thumbnail mode",
	.type = V4L2_CTRL_TYPE_BOOLEAN,
	.def = 0,
	.min = 0,
	.max = 1,
	.step = 1,
	.flags = V4L2_CTRL_FLAG_WRITE_ONLY,
};

static void wave5_set_default_dec_openparam(struct dec_open_param *open_param)
{
	open_param->bitstream_mode = BS_MODE_INTERRUPT;
	open_param->stream_endian = VPU_STREAM_ENDIAN;
	open_param->frame_endian = VPU_FRAME_ENDIAN;
}

static int wave5_vpu_dec_queue_setup(struct vb2_queue *q, unsigned int *num_buffers,
				     unsigned int *num_planes, unsigned int sizes[],
			 struct device *alloc_devs[])
{
	struct vpu_instance *inst = vb2_get_drv_priv(q);
	struct v4l2_pix_format_mplane inst_format =
		(q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) ? inst->src_fmt : inst->dst_fmt;
	unsigned int i;
	int ret;

	dev_dbg(inst->dev->dev, "num_buffers : %d | num_planes : %d | type : %d\n", *num_buffers,
		*num_planes, q->type);

	if (*num_planes) {
		if (inst_format.num_planes != *num_planes)
			return -EINVAL;

		for (i = 0; i < *num_planes; i++) {
			if (sizes[i] < inst_format.plane_fmt[i].sizeimage)
				return -EINVAL;
		}
	} else {
		*num_planes = inst_format.num_planes;

		if (*num_planes == 1) {
			sizes[0] = inst_format.width * inst_format.height * 3 / 2;
			if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
				sizes[0] = inst_format.plane_fmt[0].sizeimage;
			dev_dbg(inst->dev->dev, "size[0] : %d\n", sizes[0]);
		} else if (*num_planes == 2) {
			sizes[0] = inst_format.width * inst_format.height;
			sizes[1] = inst_format.width * inst_format.height / 2;
			dev_dbg(inst->dev->dev, "size[0] :%d | size[1] : %d\n", sizes[0], sizes[1]);
		} else if (*num_planes == 3) {
			sizes[0] = inst_format.width * inst_format.height;
			sizes[1] = inst_format.width * inst_format.height / 4;
			sizes[2] = inst_format.width * inst_format.height / 4;
			dev_dbg(inst->dev->dev, "size[0] : %d | size[1] : %d | size[2] : %d\n",
				sizes[0], sizes[1], sizes[2]);
		}
	}

	if (inst->state == VPU_INST_STATE_NONE && q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		struct dec_open_param open_param;

		memset(&open_param, 0, sizeof(struct dec_open_param));
		wave5_set_default_dec_openparam(&open_param);

		inst->bitstream_vbuf.size = ALIGN(inst->src_fmt.plane_fmt[0].sizeimage, 1024) * 4;
		ret = wave5_vdi_allocate_dma_memory(inst->dev, &inst->bitstream_vbuf);
		if (ret) {
			dev_dbg(inst->dev->dev, "alloc bitstream of size %zu failed\n",
				inst->bitstream_vbuf.size);
			return ret;
		}

		inst->std = wave5_to_vpu_codstd(inst->src_fmt.pixelformat);
		if (inst->std == STD_UNKNOWN) {
			dev_warn(inst->dev->dev, "unsupported pixelformat: %.4s\n",
				 (char *)&inst->src_fmt.pixelformat);
			return -EINVAL;
		}
		open_param.bitstream_buffer = inst->bitstream_vbuf.daddr;
		open_param.bitstream_buffer_size = inst->bitstream_vbuf.size;

		ret = wave5_vpu_dec_open(inst, &open_param);
		if (ret) {
			dev_dbg(inst->dev->dev, "wave5_vpu_dec_open failed: %d\n", ret);
			return ret;
		}

		inst->state = VPU_INST_STATE_OPEN;

		if (inst->thumbnail_mode)
			wave5_vpu_dec_give_command(inst, ENABLE_DEC_THUMBNAIL_MODE, NULL);

		inst->min_src_frame_buf_count = *num_buffers;
	} else if (inst->state == VPU_INST_STATE_INIT_SEQ &&
		q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		u32 non_linear_num = inst->min_dst_frame_buf_count;
		u32 fb_stride, fb_height;
		u32 luma_size, chroma_size;

		*num_buffers = inst->min_dst_frame_buf_count;

		for (i = 0; i < non_linear_num; i++) {
			struct frame_buffer *frame = &inst->frame_buf[i];
			struct vpu_buf *vframe = &inst->frame_vbuf[i];

			fb_stride = inst->dst_fmt.width;
			fb_height = ALIGN(inst->dst_fmt.height, 32);
			luma_size = fb_stride * fb_height;
			chroma_size = ALIGN(fb_stride / 2, 16) * fb_height;

			vframe->size = luma_size + chroma_size;
			ret = wave5_vdi_allocate_dma_memory(inst->dev, vframe);
			if (ret) {
				dev_dbg(inst->dev->dev, "alloc FBC buff of size %zu failed\n",
					vframe->size);
				return ret;
			}

			frame->buf_y = vframe->daddr;
			frame->buf_cb = vframe->daddr + luma_size;
			frame->buf_cr = (dma_addr_t)-1;
			frame->size = vframe->size;
			frame->width = inst->src_fmt.width;
			frame->stride = fb_stride;
			frame->map_type = COMPRESSED_FRAME_MAP;
			frame->update_fb_info = true;
		}
	}

	return 0;
}

static void wave5_vpu_dec_start_streaming_inst_open(struct vb2_queue *q, unsigned int count)
{
	struct dec_initial_info initial_info;
	struct vpu_instance *inst = vb2_get_drv_priv(q);
	int ret;

	ret = wave5_vpu_dec_issue_seq_init(inst);
	if (ret)
		dev_dbg(inst->dev->dev, "failed wave5_vpu_dec_issue_seq_init %d\n", ret);

	if (wave5_vpu_wait_interrupt(inst, VPU_DEC_TIMEOUT) < 0)
		dev_dbg(inst->dev->dev, "failed to call vpu_wait_interrupt()\n");

	ret = wave5_vpu_dec_complete_seq_init(inst, &initial_info);
	if (ret) {
		dev_dbg(inst->dev->dev, "vpu_dec_complete_seq_init: %d, reason : %d\n",
			ret, initial_info.seq_init_err_reason);
	} else {
		static const struct v4l2_event vpu_event_src_ch = {
			.type = V4L2_EVENT_SOURCE_CHANGE,
			.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
		};

		dev_dbg(inst->dev->dev, "width %d height %d profile %d | minbuffer : %u\n",
			initial_info.pic_width, initial_info.pic_height,
			initial_info.profile, initial_info.min_frame_buffer_count);

		inst->state = VPU_INST_STATE_INIT_SEQ;
		inst->min_dst_frame_buf_count = initial_info.min_frame_buffer_count + 1;

		if (initial_info.pic_width != inst->src_fmt.width ||
		    initial_info.pic_height != inst->src_fmt.height) {
			wave5_update_pix_fmt(&inst->src_fmt, initial_info.pic_width,
					     initial_info.pic_height);
			wave5_update_pix_fmt(&inst->dst_fmt, initial_info.pic_width,
					     initial_info.pic_height);
		}
		v4l2_event_queue_fh(&inst->v4l2_fh, &vpu_event_src_ch);
	}
}

static void wave5_vpu_dec_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct vpu_instance *inst = vb2_get_drv_priv(vb->vb2_queue);
	struct vpu_buffer *vpu_buf = wave5_to_vpu_buf(vbuf);

	dev_dbg(inst->dev->dev, "type %4d index %4d size[0] %4ld size[1] : %4ld | size[2] : %4ld\n",
		vb->type, vb->index, vb2_plane_size(&vbuf->vb2_buf, 0),
		vb2_plane_size(&vbuf->vb2_buf, 1), vb2_plane_size(&vbuf->vb2_buf, 2));

	v4l2_m2m_buf_queue(inst->v4l2_fh.m2m_ctx, vbuf);

	if (vb->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		vpu_buf->consumed = false;
		wave5_handle_bitstream_buffer(inst);
		vbuf->sequence = inst->queued_src_buf_num++;

		if (inst->state == VPU_INST_STATE_PIC_RUN)
			inst->ops->start_process(inst);
	}

	if (vb->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return;

	if (inst->state == VPU_INST_STATE_INIT_SEQ) {
		dma_addr_t buf_addr_y = 0, buf_addr_cb = 0, buf_addr_cr = 0;
		u32 buf_size = 0;
		u32 non_linear_num = inst->min_dst_frame_buf_count;
		u32 fb_stride = inst->dst_fmt.width;
		u32 luma_size = fb_stride * inst->dst_fmt.height;
		u32 chroma_size = (fb_stride / 2) * (inst->dst_fmt.height / 2);

		if (inst->dst_fmt.num_planes == 1) {
			buf_size = vb2_plane_size(&vbuf->vb2_buf, 0);
			buf_addr_y = vb2_dma_contig_plane_dma_addr(&vbuf->vb2_buf, 0);
			buf_addr_cb = buf_addr_y + luma_size;
			buf_addr_cr = buf_addr_cb + chroma_size;
		} else if (inst->dst_fmt.num_planes == 2) {
			buf_size = vb2_plane_size(&vbuf->vb2_buf, 0) +
				vb2_plane_size(&vbuf->vb2_buf, 1);
			buf_addr_y = vb2_dma_contig_plane_dma_addr(&vbuf->vb2_buf, 0);
			buf_addr_cb = vb2_dma_contig_plane_dma_addr(&vbuf->vb2_buf, 1);
			buf_addr_cr = buf_addr_cb + chroma_size;
		} else if (inst->dst_fmt.num_planes == 3) {
			buf_size = vb2_plane_size(&vbuf->vb2_buf, 0) +
				vb2_plane_size(&vbuf->vb2_buf, 1) +
				vb2_plane_size(&vbuf->vb2_buf, 2);
			buf_addr_y = vb2_dma_contig_plane_dma_addr(&vbuf->vb2_buf, 0);
			buf_addr_cb = vb2_dma_contig_plane_dma_addr(&vbuf->vb2_buf, 1);
			buf_addr_cr = vb2_dma_contig_plane_dma_addr(&vbuf->vb2_buf, 2);
		}
		inst->frame_buf[vb->index + non_linear_num].buf_y = buf_addr_y;
		inst->frame_buf[vb->index + non_linear_num].buf_cb = buf_addr_cb;
		inst->frame_buf[vb->index + non_linear_num].buf_cr = buf_addr_cr;
		inst->frame_buf[vb->index + non_linear_num].size = buf_size;
		inst->frame_buf[vb->index + non_linear_num].width = inst->src_fmt.width;
		inst->frame_buf[vb->index + non_linear_num].stride = fb_stride;
		inst->frame_buf[vb->index + non_linear_num].map_type = LINEAR_FRAME_MAP;
		inst->frame_buf[vb->index + non_linear_num].update_fb_info = true;
	}

	if (inst->state == VPU_INST_STATE_PIC_RUN ||
	    inst->state == VPU_INST_STATE_STOP ||
	    inst->state == VPU_INST_STATE_WAIT_BUF) {
		wave5_vpu_dec_clr_disp_flag(inst, vb->index);
		if (inst->state == VPU_INST_STATE_WAIT_BUF)
			inst->state = VPU_INST_STATE_PIC_RUN;
		if (inst->state == VPU_INST_STATE_STOP)
			inst->ops->start_process(inst);
	}
	vbuf->sequence = inst->queued_dst_buf_num++;
}

static int wave5_vpu_dec_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct vpu_instance *inst = vb2_get_drv_priv(q);
	int ret;

	dev_dbg(inst->dev->dev, "type : %d\n", q->type);

	if (inst->state == VPU_INST_STATE_OPEN)
		wave5_vpu_dec_start_streaming_inst_open(q, count);

	if (inst->state == VPU_INST_STATE_INIT_SEQ &&
	    q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		u32 non_linear_num = inst->min_dst_frame_buf_count;
		u32 linear_num = inst->min_dst_frame_buf_count;
		u32 stride = inst->dst_fmt.width;

		dev_dbg(inst->dev->dev, "stride %d dst height %d\n", stride, inst->dst_fmt.height);
		ret = wave5_vpu_dec_register_frame_buffer_ex(inst, non_linear_num, linear_num,
							     stride, inst->dst_fmt.height,
							    COMPRESSED_FRAME_MAP);
		if (ret)
			dev_dbg(inst->dev->dev, "fail vpu_dec_register_frame_buffer_ex %d", ret);

		inst->state = VPU_INST_STATE_PIC_RUN;
	}

	return 0;
}

static void wave5_vpu_dec_stop_streaming(struct vb2_queue *q)
{
	struct vpu_instance *inst = vb2_get_drv_priv(q);
	struct vb2_v4l2_buffer *buf;
	bool check_cmd = TRUE;

	dev_dbg(inst->dev->dev, "type : %d\n", q->type);

	if (wave5_vpu_both_queues_are_streaming(inst))
		inst->state = VPU_INST_STATE_STOP;

	while (check_cmd) {
		struct queue_status_info q_status;
		struct dec_output_info dec_output_info;
		int ret;

		wave5_vpu_dec_give_command(inst, DEC_GET_QUEUE_STATUS, &q_status);

		if (q_status.instance_queue_count + q_status.report_queue_count == 0)
			break;

		if (wave5_vpu_wait_interrupt(inst, VPU_DEC_TIMEOUT) < 0)
			break;

		ret = wave5_vpu_dec_get_output_info(inst, &dec_output_info);
	}

	if (q->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		while ((buf = v4l2_m2m_src_buf_remove(inst->v4l2_fh.m2m_ctx))) {
			dev_dbg(inst->dev->dev, "buf type : %4d | index : %4d\n", buf->vb2_buf.type,
				buf->vb2_buf.index);
			v4l2_m2m_buf_done(buf, VB2_BUF_STATE_ERROR);
		}
	} else {
		while ((buf = v4l2_m2m_dst_buf_remove(inst->v4l2_fh.m2m_ctx))) {
			u32 plane;

			dev_dbg(inst->dev->dev, "buf type : %4d | index : %4d\n", buf->vb2_buf.type,
				buf->vb2_buf.index);

			for (plane = 0; plane < inst->dst_fmt.num_planes; plane++)
				vb2_set_plane_payload(&buf->vb2_buf, plane, 0);

			v4l2_m2m_buf_done(buf, VB2_BUF_STATE_ERROR);
		}
	}
}

static const struct vb2_ops wave5_vpu_dec_vb2_ops = {
	.queue_setup = wave5_vpu_dec_queue_setup,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
	.buf_queue = wave5_vpu_dec_buf_queue,
	.start_streaming = wave5_vpu_dec_start_streaming,
	.stop_streaming = wave5_vpu_dec_stop_streaming,
};

static void wave5_set_default_format(struct v4l2_pix_format_mplane *src_fmt,
				     struct v4l2_pix_format_mplane *dst_fmt)
{
	unsigned int dst_pix_fmt = wave5_vpu_dec_fmt_list[VPU_FMT_TYPE_RAW][0].v4l2_pix_fmt;
	const struct v4l2_format_info *dst_fmt_info = v4l2_format_info(dst_pix_fmt);

	src_fmt->pixelformat = wave5_vpu_dec_fmt_list[VPU_FMT_TYPE_CODEC][0].v4l2_pix_fmt;
	src_fmt->field = V4L2_FIELD_NONE;
	src_fmt->flags = 0;
	src_fmt->num_planes = 1;
	wave5_update_pix_fmt(src_fmt, 720, 480);

	dst_fmt->pixelformat = dst_pix_fmt;
	dst_fmt->field = V4L2_FIELD_NONE;
	dst_fmt->flags = 0;
	dst_fmt->num_planes = dst_fmt_info->mem_planes;
	wave5_update_pix_fmt(dst_fmt, 736, 480);
}

static int wave5_vpu_dec_queue_init(void *priv, struct vb2_queue *src_vq, struct vb2_queue *dst_vq)
{
	struct vpu_instance *inst = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->ops = &wave5_vpu_dec_vb2_ops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->buf_struct_size = sizeof(struct vpu_buffer);
	src_vq->allow_zero_bytesused = 1;
	src_vq->min_buffers_needed = 0;
	src_vq->drv_priv = inst;
	src_vq->lock = &inst->dev->dev_lock;
	src_vq->dev = inst->dev->v4l2_dev.dev;
	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->ops = &wave5_vpu_dec_vb2_ops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->buf_struct_size = sizeof(struct vpu_buffer);
	dst_vq->allow_zero_bytesused = 1;
	dst_vq->min_buffers_needed = 0;
	dst_vq->drv_priv = inst;
	dst_vq->lock = &inst->dev->dev_lock;
	dst_vq->dev = inst->dev->v4l2_dev.dev;
	ret = vb2_queue_init(dst_vq);
	if (ret)
		return ret;

	return 0;
}

static const struct vpu_instance_ops wave5_vpu_dec_inst_ops = {
	.start_process = wave5_vpu_dec_start_decode,
	.stop_process = wave5_vpu_dec_stop_decode,
	.finish_process = wave5_vpu_dec_finish_decode,
};

static int wave5_vpu_open_dec(struct file *filp)
{
	struct video_device *vdev = video_devdata(filp);
	struct vpu_device *dev = video_drvdata(filp);
	struct vpu_instance *inst = NULL;
	struct v4l2_ctrl *ctrl;
	int ret;

	inst = kzalloc(sizeof(*inst), GFP_KERNEL);
	if (!inst)
		return -ENOMEM;

	inst->dev = dev;
	inst->type = VPU_INST_TYPE_DEC;
	inst->ops = &wave5_vpu_dec_inst_ops;

	spin_lock_init(&inst->bitstream_lock);

	v4l2_fh_init(&inst->v4l2_fh, vdev);
	filp->private_data = &inst->v4l2_fh;
	v4l2_fh_add(&inst->v4l2_fh);

	inst->v4l2_fh.m2m_ctx =
		v4l2_m2m_ctx_init(dev->v4l2_m2m_dev, inst, wave5_vpu_dec_queue_init);
	if (IS_ERR(inst->v4l2_fh.m2m_ctx)) {
		ret = PTR_ERR(inst->v4l2_fh.m2m_ctx);
		goto free_inst;
	}

	v4l2_ctrl_handler_init(&inst->v4l2_ctrl_hdl, 10);
	v4l2_ctrl_new_custom(&inst->v4l2_ctrl_hdl, &wave5_vpu_thumbnail_mode, NULL);
	ctrl = v4l2_ctrl_new_std(&inst->v4l2_ctrl_hdl, &wave5_vpu_dec_ctrl_ops,
				 V4L2_CID_MIN_BUFFERS_FOR_CAPTURE, 1, 32, 1, 1);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE;

	if (inst->v4l2_ctrl_hdl.error) {
		ret = -ENODEV;
		goto free_inst;
	}

	inst->v4l2_fh.ctrl_handler = &inst->v4l2_ctrl_hdl;
	v4l2_ctrl_handler_setup(&inst->v4l2_ctrl_hdl);

	wave5_set_default_format(&inst->src_fmt, &inst->dst_fmt);
	inst->colorspace = V4L2_COLORSPACE_REC709;
	inst->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	inst->hsv_enc = 0;
	inst->quantization = V4L2_QUANTIZATION_DEFAULT;
	inst->xfer_func = V4L2_XFER_FUNC_DEFAULT;

	return 0;

free_inst:
	kfree(inst);
	return ret;
}

static int wave5_vpu_dec_release(struct file *filp)
{
	struct vpu_instance *inst = wave5_to_vpu_inst(filp->private_data);
	u32 fail_res;
	int retry_count = 10;
	int i, ret;

	v4l2_m2m_ctx_release(inst->v4l2_fh.m2m_ctx);
	if (inst->state != VPU_INST_STATE_NONE) {
		do {
			fail_res = 0;
			ret = wave5_vpu_dec_close(inst, &fail_res);
			if (ret && ret != -EIO)
				break;
			if (fail_res != WAVE5_SYSERR_VPU_STILL_RUNNING)
				break;
			if (!wave5_vpu_wait_interrupt(inst, VPU_DEC_TIMEOUT))
				break;
		} while (--retry_count);

		if (fail_res == WAVE5_SYSERR_VPU_STILL_RUNNING)
			dev_warn(inst->dev->dev, "close fail res %u\n", fail_res);
		if (ret && ret != -EIO)
			dev_warn(inst->dev->dev, "close fail ret=%d\n", ret);
	}

	for (i = 0; i < inst->min_dst_frame_buf_count; i++)
		wave5_vdi_free_dma_memory(inst->dev, &inst->frame_vbuf[i]);

	wave5_vdi_free_dma_memory(inst->dev, &inst->bitstream_vbuf);
	v4l2_ctrl_handler_free(&inst->v4l2_ctrl_hdl);
	v4l2_fh_del(&inst->v4l2_fh);
	v4l2_fh_exit(&inst->v4l2_fh);
	kfree(inst);

	return 0;
}

static const struct v4l2_file_operations wave5_vpu_dec_fops = {
	.owner = THIS_MODULE,
	.open = wave5_vpu_open_dec,
	.release = wave5_vpu_dec_release,
	.unlocked_ioctl = video_ioctl2,
	.poll = v4l2_m2m_fop_poll,
	.mmap = v4l2_m2m_fop_mmap,
};

int wave5_vpu_dec_register_device(struct vpu_device *dev)
{
	struct video_device *vdev_dec;
	int ret;

	vdev_dec = devm_kzalloc(dev->v4l2_dev.dev, sizeof(*vdev_dec), GFP_KERNEL);
	if (!vdev_dec)
		return -ENOMEM;

	dev->video_dev_dec = vdev_dec;

	strscpy(vdev_dec->name, VPU_DEC_DEV_NAME, sizeof(vdev_dec->name));
	vdev_dec->fops = &wave5_vpu_dec_fops;
	vdev_dec->ioctl_ops = &wave5_vpu_dec_ioctl_ops;
	vdev_dec->release = video_device_release_empty;
	vdev_dec->v4l2_dev = &dev->v4l2_dev;
	vdev_dec->vfl_dir = VFL_DIR_M2M;
	vdev_dec->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	vdev_dec->lock = &dev->dev_lock;

	ret = video_register_device(vdev_dec, VFL_TYPE_VIDEO, -1);
	if (ret)
		return ret;

	video_set_drvdata(vdev_dec, dev);

	return 0;
}

void wave5_vpu_dec_unregister_device(struct vpu_device *dev)
{
	video_unregister_device(dev->video_dev_dec);
}

