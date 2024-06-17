// SPDX-License-Identifier: GPL-2.0
/*
 * IMG DEC V4L2 Interface function implementations
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Angela Stegmaier <angelabaker@ti.com>
 *	David Huang <d-huang@ti.com>
 *
 * Re-written for upstreaming
 *	Prashanth Kumar Amai <prashanth.ka@pathpartnertech.com>
 *	Sidraya Jayagond <sidraya.bj@pathpartnertech.com>
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/videodev2.h>
#include <linux/idr.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/spinlock.h>
#include <linux/printk.h>
#include <linux/mutex.h>

#ifdef ERROR_RECOVERY_SIMULATION
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/types.h>
#endif

#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-sg.h>
#ifdef CAPTURE_CONTIG_ALLOC
#include <media/videobuf2-dma-contig.h>
#endif

#include "core.h"
#include "h264fw_data.h"
#include "hevcfw_data.h"
#include "img_dec_common.h"
#include "vxd_pvdec_priv.h"
#include "vxd_dec.h"
#include "img_errors.h"

#define VXD_DEC_SPIN_LOCK_NAME  "vxd-dec"
#define IMG_VXD_DEC_MODULE_NAME "vxd-dec"

#ifdef ERROR_RECOVERY_SIMULATION
/* This code should be execute only in debug flag */
/*
 * vxd decoder kernel object to create sysfs to debug error recovery and firmware
 * watchdog timer. This kernel object will create a directory under /sys/kernel,
 * containing two files fw_error_value and disable_fw_irq.
 */
struct kobject *vxd_dec_kobject;

/* fw_error_value is the variable used to handle fw_error_attr */
int fw_error_value = VDEC_ERROR_MAX;

/* irq for the module, stored globally so can be accessed from sysfs */
int g_module_irq;

/*
 * fw_error_attr. Application can set the value of this attribute, based on the
 * firmware error that needs to be reproduced.
 */
struct kobj_attribute fw_error_attr =
	__ATTR(fw_error_value, 0660, vxd_sysfs_show, vxd_sysfs_store);

/* disable_fw_irq_value is variable to handle disable_fw_irq_attr */
int disable_fw_irq_value;

/*
 * disable_fw_irq_attr. Application can set the value of this attribute. 1 to
 * disable irq. 0 to enable irq.
 */
struct kobj_attribute disable_fw_irq_attr =
	__ATTR(disable_fw_irq_value, 0660, vxd_sysfs_show, vxd_sysfs_store);

/*
 * Group attribute so that we can create and destroy all of them at once.
 */
struct attribute *attrs[] = {
	&fw_error_attr.attr,
	&disable_fw_irq_attr.attr,
	NULL,         /* Terminate list of attributes with NULL */
};

/*
 * An unnamed attribute group will put all of the attributes directly in
 * the kobject directory.  If we specify a name, a sub directory will be
 * created for the attributes with the directory being the name of the
 * attribute group
 */
struct attribute_group attr_group = {
	.attrs = attrs,
};

#endif

static struct heap_config vxd_dec_heap_configs[] = {
	{
		.type = MEM_HEAP_TYPE_UNIFIED,
		.options.unified = {
			.gfp_type = __GFP_DMA32 | __GFP_ZERO,
		},
		.to_dev_addr = NULL,
	},
};

static struct vxd_dec_fmt vxd_dec_formats[] = {
	{
		.fourcc = V4L2_PIX_FMT_NV12,
		.num_planes = 1,
		.type = IMG_DEC_FMT_TYPE_CAPTURE,
		.std = VDEC_STD_UNDEFINED,
		.pixfmt = IMG_PIXFMT_420PL12YUV8,
		.interleave = PIXEL_UV_ORDER,
		.idc = PIXEL_FORMAT_420,
		.size_num = 3,
		.size_den = 2,
		.bytes_pp = 1,
	},
	{
		.fourcc = V4L2_PIX_FMT_NV16,
		.num_planes = 1,
		.type = IMG_DEC_FMT_TYPE_CAPTURE,
		.std = VDEC_STD_UNDEFINED,
		.pixfmt = IMG_PIXFMT_422PL12YUV8,
		.interleave = PIXEL_UV_ORDER,
		.idc = PIXEL_FORMAT_422,
		.size_num = 2,
		.size_den = 1,
		.bytes_pp = 1,
	},
	{
		.fourcc = V4L2_PIX_FMT_TI1210,
		.num_planes = 1,
		.type = IMG_DEC_FMT_TYPE_CAPTURE,
		.std = VDEC_STD_UNDEFINED,
		.pixfmt = IMG_PIXFMT_420PL12YUV10_MSB,
		.interleave = PIXEL_UV_ORDER,
		.idc = PIXEL_FORMAT_420,
		.size_num = 3,
		.size_den = 2,
		.bytes_pp = 2,
	},
	{
		.fourcc = V4L2_PIX_FMT_TI1610,
		.num_planes = 1,
		.type = IMG_DEC_FMT_TYPE_CAPTURE,
		.std = VDEC_STD_UNDEFINED,
		.pixfmt = IMG_PIXFMT_422PL12YUV10_MSB,
		.interleave = PIXEL_UV_ORDER,
		.idc = PIXEL_FORMAT_422,
		.size_num = 2,
		.size_den = 1,
		.bytes_pp = 2,
	},
	{
		.fourcc = V4L2_PIX_FMT_H264,
		.num_planes = 1,
		.type = IMG_DEC_FMT_TYPE_OUTPUT,
		.std = VDEC_STD_H264,
		.pixfmt = IMG_PIXFMT_UNDEFINED,
		.interleave = PIXEL_INVALID_CI,
		.idc = PIXEL_FORMAT_INVALID,
		.size_num = 1,
		.size_den = 1,
		.bytes_pp = 1,
	},
	{
		.fourcc = V4L2_PIX_FMT_HEVC,
		.num_planes = 1,
		.type = IMG_DEC_FMT_TYPE_OUTPUT,
		.std = VDEC_STD_HEVC,
		.pixfmt = IMG_PIXFMT_UNDEFINED,
		.interleave = PIXEL_INVALID_CI,
		.idc = PIXEL_FORMAT_INVALID,
		.size_num = 1,
		.size_den = 1,
		.bytes_pp = 1,
	},
	{
		.fourcc = V4L2_PIX_FMT_MJPEG,
		.num_planes = 1,
		.type = IMG_DEC_FMT_TYPE_OUTPUT,
		.std = VDEC_STD_JPEG,
		.pixfmt = IMG_PIXFMT_UNDEFINED,
		.interleave = PIXEL_INVALID_CI,
		.idc = PIXEL_FORMAT_INVALID,
		.size_num = 1,
		.size_den = 1,
		.bytes_pp = 1,
	},
	{
		.fourcc = V4L2_PIX_FMT_YUV420M,
		.num_planes = 3,
		.type = IMG_DEC_FMT_TYPE_CAPTURE,
		.std = VDEC_STD_UNDEFINED,
		.pixfmt = 86031,
		.interleave = PIXEL_UV_ORDER,
		.idc = PIXEL_FORMAT_420,
		.size_num = 2,
		.size_den = 1,
		.bytes_pp = 1,
	},
	{
		.fourcc = V4L2_PIX_FMT_YUV422M,
		.num_planes = 3,
		.type = IMG_DEC_FMT_TYPE_CAPTURE,
		.std = VDEC_STD_UNDEFINED,
		.pixfmt = 81935,
		.interleave = PIXEL_UV_ORDER,
		.idc = PIXEL_FORMAT_422,
		.size_num = 3,
		.size_den = 1,
		.bytes_pp = 1,
	},
};

#ifdef ERROR_RECOVERY_SIMULATION
ssize_t vxd_sysfs_show(struct kobject *vxd_dec_kobject,
		       struct kobj_attribute *attr, char *buf)

{
	int var = 0;

	if (strcmp(attr->attr.name, "fw_error_value") == 0)
		var = fw_error_value;

	else
		var = disable_fw_irq_value;

	return sprintf(buf, "%d\n", var);
}

ssize_t vxd_sysfs_store(struct kobject *vxd_dec_kobject,
			struct kobj_attribute *attr,
			const char *buf, unsigned long count)
{
	int var = 0, rv = 0;

	rv = sscanf(buf, "%du", &var);

	if (strcmp(attr->attr.name, "fw_error_value") == 0) {
		fw_error_value = var;
	} else {
		disable_fw_irq_value = var;
		/*
		 * if disable_fw_irq_value is not zero, disable the irq to reproduce
		 * firmware non responsiveness in vxd_worker.
		 */
		if (disable_fw_irq_value != 0) {
			/* just ignore the irq */
			disable_irq(g_module_irq);
		}
	}
	return sprintf((char *)buf, "%d\n", var);
}
#endif

static struct vxd_dec_ctx *file2ctx(struct file *file)
{
	return container_of(file->private_data, struct vxd_dec_ctx, fh);
}

static irqreturn_t soft_thread_irq(int irq, void *dev_id)
{
	struct platform_device *pdev = (struct platform_device *)dev_id;

	if (!pdev)
		return IRQ_NONE;

	return vxd_handle_thread_irq(&pdev->dev);
}

static irqreturn_t hard_isrcb(int irq, void *dev_id)
{
	struct platform_device *pdev = (struct platform_device *)dev_id;

	if (!pdev)
		return IRQ_NONE;

	return vxd_handle_irq(&pdev->dev);
}

static struct vxd_mapping *find_mapping(unsigned int buf_map_id, struct list_head *head)
{
	struct list_head *list;
	struct vxd_mapping *mapping = NULL;

	list_for_each(list, head) {
		mapping = list_entry(list, struct vxd_mapping, list);
		if (mapping->buf_map_id == buf_map_id)
			break;
		mapping = NULL;
	}
	return mapping;
}

static struct vxd_buffer *find_buffer(unsigned int buf_map_id, struct list_head *head)
{
	struct list_head *list;
	struct vxd_buffer *buf = NULL;

	list_for_each(list, head) {
		buf = list_entry(list, struct vxd_buffer, list);
		if (buf->buf_map_id == buf_map_id)
			break;
		buf = NULL;
	}
	return buf;
}

static void return_worker(void *work)
{
	struct vxd_dec_ctx *ctx;
	struct vxd_return *res;
	struct device *dev;
	struct timespec64 time;
	int loop;

	work = get_work_buff(work, TRUE);

	res = container_of(work, struct vxd_return, work);
	ctx = res->ctx;
	dev = ctx->dev->dev;
	switch (res->type) {
	case VXD_CB_PICT_DECODED:
		v4l2_m2m_job_finish(ctx->dev->m2m_dev, ctx->fh.m2m_ctx);
		ktime_get_real_ts64(&time);
		for (loop = 0; loop < ARRAY_SIZE(ctx->dev->time_drv); loop++) {
			if (ctx->dev->time_drv[loop].id == res->buf_map_id) {
				ctx->dev->time_drv[loop].end_time =
				timespec64_to_ns(&time);
#ifdef DEBUG_DECODER_DRIVER
				dev_info(dev, "picture buf decode time is %llu us for buf_map_id 0x%x\n",
					 div_s64(ctx->dev->time_drv[loop].end_time -
						 ctx->dev->time_drv[loop].start_time, 1000),
					 res->buf_map_id);
#endif
				break;
				}
			}

		if (loop == ARRAY_SIZE(ctx->dev->time_drv))
			dev_err(dev, "picture buf decode for buf_map_id x%0x is not measured\n",
				res->buf_map_id);
		break;

	default:
		break;
	}
	kfree(res->work);
	kfree(res);
}

static void vxd_error_recovery(struct vxd_dec_ctx *ctx)
{
	int ret = -1;

	/*
	 * In the previous frame decoding fatal error has been detected
	 * so we need to reload the firmware to make it alive.
	 */
	pr_debug("Reloading the firmware because of previous error\n");
	vxd_clean_fw_resources(ctx->dev);
	ret = vxd_prepare_fw(ctx->dev);
	if (ret)
		pr_err("Reloading the firmware failed!!");
}

static struct vxd_dec_q_data *get_q_data(struct vxd_dec_ctx *ctx,
					 enum v4l2_buf_type type)
{
	switch (type) {
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
		return &ctx->q_data[Q_DATA_SRC];
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		return &ctx->q_data[Q_DATA_DST];
	default:
		return NULL;
	}
	return NULL;
}

static void vxd_return_resource(void *ctx_handle, enum vxd_cb_type type,
				unsigned int buf_map_id)
{
	struct vxd_return *res;
	struct vxd_buffer *buf = NULL;
	struct vb2_v4l2_buffer *vb;
	struct vxd_mapping *mapping = NULL;
	struct vxd_dec_ctx *ctx = (struct vxd_dec_ctx *)ctx_handle;
	struct v4l2_event event = {};
	struct device *dev = ctx->dev->dev;
	int i;
	struct vxd_dec_q_data *q_data;

	switch (type) {
	case VXD_CB_STRUNIT_PROCESSED:

		buf = find_buffer(buf_map_id, &ctx->out_buffers);
		if (!buf) {
			dev_err(dev, "Could not locate buf_map_id=0x%x in OUTPUT buffers list\n",
				buf_map_id);
			break;
		}
		buf->buffer.vb.field = V4L2_FIELD_NONE;
		q_data = get_q_data(ctx, buf->buffer.vb.vb2_buf.vb2_queue->type);
		if (!q_data)
			return;

		for (i = 0; i < q_data->fmt->num_planes; i++)
			vb2_set_plane_payload(&buf->buffer.vb.vb2_buf, i,
					      ctx->pict_bufcfg.plane_size[i]);

		v4l2_m2m_buf_done(&buf->buffer.vb, VB2_BUF_STATE_DONE);
		break;
	case VXD_CB_SPS_RELEASE:
		break;
	case VXD_CB_PPS_RELEASE:
		break;
	case VXD_CB_PICT_DECODED:
		res = kzalloc(sizeof(*res), GFP_KERNEL);
		if (!res)
			return;
		res->ctx = ctx;
		res->type = type;
		res->buf_map_id = buf_map_id;

		init_work(&res->work, return_worker, HWA_DECODER);
		if (!res->work)
			return;

		schedule_work(res->work);

		break;
	case VXD_CB_PICT_DISPLAY:
		buf = find_buffer(buf_map_id, &ctx->cap_buffers);
		if (!buf) {
			dev_err(dev, "Could not locate buf_map_id=0x%x in CAPTURE buffers list\n",
				buf_map_id);
			break;
		}
		buf->mapping->reuse = FALSE;
		buf->buffer.vb.field = V4L2_FIELD_NONE;
		q_data = get_q_data(ctx, buf->buffer.vb.vb2_buf.vb2_queue->type);
		if (!q_data)
			return;

		for (i = 0; i < q_data->fmt->num_planes; i++)
			vb2_set_plane_payload(&buf->buffer.vb.vb2_buf, i,
					      ctx->pict_bufcfg.plane_size[i]);

		v4l2_m2m_buf_done(&buf->buffer.vb, VB2_BUF_STATE_DONE);
		break;
	case VXD_CB_PICT_RELEASE:
		buf = find_buffer(buf_map_id, &ctx->reuse_queue);
		if (buf) {
			buf->mapping->reuse = TRUE;
			list_move_tail(&buf->list, &ctx->cap_buffers);

			v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, &buf->buffer.vb);
			break;
		}
		mapping = find_mapping(buf_map_id, &ctx->cap_mappings);
		if (!mapping) {
			dev_err(dev, "Could not locate buf_map_id=0x%x in CAPTURE buffers list\n",
				buf_map_id);
			break;
		}
		mapping->reuse = TRUE;

		break;
	case VXD_CB_PICT_END:
		break;
	case VXD_CB_STR_END:
		event.type = V4L2_EVENT_EOS;
		v4l2_event_queue_fh(&ctx->fh, &event);
		if (v4l2_m2m_num_dst_bufs_ready(ctx->fh.m2m_ctx) > 0) {
			vb = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
			vb->flags |= V4L2_BUF_FLAG_LAST;

			q_data = get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE);
			if (!q_data)
				break;

			for (i = 0; i < q_data->fmt->num_planes; i++)
				vb2_set_plane_payload(&vb->vb2_buf, i, 0);

			v4l2_m2m_buf_done(vb, VB2_BUF_STATE_DONE);
		} else {
			ctx->flag_last = TRUE;
		}
		break;
	case VXD_CB_ERROR_FATAL:
		/*
		 * There has been FW error, so we need to reload the firmware.
		 */
		vxd_error_recovery(ctx);

		/*
		 * Just send zero size buffer to v4l2 application,
		 * informing the error condition.
		 */
		if (v4l2_m2m_num_dst_bufs_ready(ctx->fh.m2m_ctx) > 0) {
			vb = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);

			q_data = get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE);
			if (!q_data)
				break;

			for (i = 0; i < q_data->fmt->num_planes; i++)
				vb2_set_plane_payload(&vb->vb2_buf, i, 0);

			v4l2_m2m_buf_done(vb, VB2_BUF_STATE_DONE);
		} else {
			ctx->flag_last = TRUE;
		}
		break;
	default:
		break;
	}
}

static int vxd_dec_submit_opconfig(struct vxd_dec_ctx *ctx)
{
	int ret = 0;

	if (ctx->stream_created) {
		ret = core_stream_set_output_config(ctx->res_str_id,
						    &ctx->str_opcfg,
						    &ctx->pict_bufcfg);
		if (ret) {
			dev_err(ctx->dev->dev, "core_stream_set_output_config failed\n");
			ctx->opconfig_pending = TRUE;
			return ret;
		}
		ctx->opconfig_pending = FALSE;
		ctx->stream_configured = TRUE;
	} else {
		ctx->opconfig_pending = TRUE;
	}
	return ret;
}

static int vxd_dec_queue_setup(struct vb2_queue *vq,
			       unsigned int *nbuffers,
			       unsigned int *nplanes,
			       unsigned int sizes[],
			       struct device *alloc_devs[])
{
	struct vxd_dec_ctx *ctx = vb2_get_drv_priv(vq);
	struct vxd_dec_q_data *q_data;
	struct vxd_dec_q_data *src_q_data;
	int i;
	unsigned int hw_nbuffers = 0;

	q_data = get_q_data(ctx, vq->type);
	if (!q_data)
		return -EINVAL;

	if (*nplanes) {
		/* This is being called from CREATEBUFS, perform validation */
		if (*nplanes != q_data->fmt->num_planes)
			return -EINVAL;

		for (i = 0; i < *nplanes; i++) {
			if (sizes[i] != q_data->size_image[i])
				return -EINVAL;
		}

		return 0;
	}

	*nplanes = q_data->fmt->num_planes;

	if (!V4L2_TYPE_IS_OUTPUT(vq->type)) {
		src_q_data = &ctx->q_data[Q_DATA_SRC];
		if (src_q_data)
			hw_nbuffers = get_nbuffers(src_q_data->fmt->std,
						   q_data->width,
						   q_data->height,
						   ctx->max_num_ref_frames);
	}

	*nbuffers = max(*nbuffers, hw_nbuffers);

	for (i = 0; i < *nplanes; i++)
		sizes[i] = q_data->size_image[i];

	return 0;
}

static int vxd_dec_buf_prepare(struct vb2_buffer *vb)
{
	struct vxd_dec_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct device *dev = ctx->dev->dev;
	struct vxd_dec_q_data *q_data;
	void *sgt;
#ifdef CAPTURE_CONTIG_ALLOC
	struct page *new_page;
#else
	void *sgl;
#endif
	struct sg_table *sgt_new;
	void *sgl_new;
	int pages;
	int nents = 0;
	int size = 0;
	int plane, num_planes, ret = 0;
	unsigned long dma_addr;
	struct vxd_mapping *mapping;
	struct list_head *list;
	struct vxd_buffer *buf =
		container_of(vb, struct vxd_buffer, buffer.vb.vb2_buf);

	q_data = get_q_data(ctx, vb->vb2_queue->type);
	if (!q_data)
		return -EINVAL;

	num_planes = q_data->fmt->num_planes;

	for (plane = 0; plane < num_planes; plane++) {
		if (vb2_plane_size(vb, plane) < q_data->size_image[plane]) {
			dev_err(dev, "data will not fit into plane (%lu < %lu)\n",
				vb2_plane_size(vb, plane),
				(long)q_data->size_image[plane]);
			return -EINVAL;
		}
	}

	if (buf->mapped && !V4L2_TYPE_IS_OUTPUT(vb->type)) {
#ifdef CAPTURE_CONTIG_ALLOC
		dma_addr = vb2_dma_contig_plane_dma_addr(vb, 0);
#else
		sgt = vb2_dma_sg_plane_desc(vb, 0);
		if (!sgt)
			return -EFAULT;

		dma_addr = sg_phys(img_mmu_get_sgl(sgt));
#endif
		if (buf->buf_info.dma_addr != dma_addr) {
			list_for_each(list, &ctx->cap_mappings) {
				mapping = list_entry(list, struct vxd_mapping, list);
				if (dma_addr == mapping->dma_addr)
					break;

				mapping = NULL;
			}
			if (mapping) {
				/* skip the mapping, buf update buf_map_id */
				buf->buf_info.dma_addr = mapping->dma_addr;
				buf->buf_map_id = mapping->buf_map_id;
				buf->buf_info.cpu_linear_addr = vb2_plane_vaddr(vb, 0);
				buf->mapping = mapping;
				return 0;
			}
		} else {
			return 0;
		}
	} else if (buf->mapped) {
		return 0;
	}

	buf->buf_info.cpu_linear_addr = vb2_plane_vaddr(vb, 0);
	buf->buf_info.buf_size = vb2_plane_size(vb, 0);
	buf->buf_info.fd = -1;
	sgt = vb2_dma_sg_plane_desc(vb, 0);
	if (!sgt) {
		dev_err(dev, "Could not get sg_table from plane 0\n");
		return -EINVAL;
	}

	if (V4L2_TYPE_IS_OUTPUT(vb->type)) {
		buf->buf_info.dma_addr = sg_phys(img_mmu_get_sgl(sgt));
		ret = core_stream_map_buf_sg(ctx->res_str_id,
					     VDEC_BUFTYPE_BITSTREAM,
					     &buf->buf_info, sgt,
					     &buf->buf_map_id);
		if (ret) {
			dev_err(dev, "OUTPUT core_stream_map_buf_sg failed\n");
			return ret;
		}

		buf->bstr_info.buf_size = q_data->size_image[0];
		buf->bstr_info.cpu_virt_addr = buf->buf_info.cpu_linear_addr;
		buf->bstr_info.mem_attrib =
			SYS_MEMATTRIB_UNCACHED | SYS_MEMATTRIB_WRITECOMBINE |
			SYS_MEMATTRIB_INPUT | SYS_MEMATTRIB_CPU_WRITE;
		buf->bstr_info.bufmap_id = buf->buf_map_id;
		lst_init(&buf->seq_unit.bstr_seg_list);
		lst_init(&buf->pic_unit.bstr_seg_list);
		lst_init(&buf->end_unit.bstr_seg_list);

		list_add_tail(&buf->list, &ctx->out_buffers);
	} else {
#ifdef CAPTURE_CONTIG_ALLOC
		buf->buf_info.dma_addr = vb2_dma_contig_plane_dma_addr(vb, 0);
#else
		buf->buf_info.dma_addr = sg_phys(img_mmu_get_sgl(sgt));
#endif
		/* Create a single sgt from the plane(s) */
		sgt_new = kmalloc(sizeof(*sgt_new), GFP_KERNEL);
		if (!sgt_new)
			return -EINVAL;

		for (plane = 0; plane < num_planes; plane++) {
			size += ALIGN(vb2_plane_size(vb, plane), PAGE_SIZE);
			sgt = vb2_dma_sg_plane_desc(vb, plane);
			if (!sgt) {
				dev_err(dev, "Could not get sg_table from plane %d\n", plane);
				kfree(sgt_new);
				return -EINVAL;
			}
#ifdef CAPTURE_CONTIG_ALLOC
			nents += 1;
#else
			nents += sg_nents(img_mmu_get_sgl(sgt));
#endif
		}
		buf->buf_info.buf_size = size;

		pages = (size + PAGE_SIZE - 1) / PAGE_SIZE;
		ret = sg_alloc_table(sgt_new, nents, GFP_KERNEL);
		if (ret) {
			kfree(sgt_new);
			return -EINVAL;
		}
		sgl_new = img_mmu_get_sgl(sgt_new);

		for (plane = 0; plane < num_planes; plane++) {
			sgt = vb2_dma_sg_plane_desc(vb, plane);
			if (!sgt) {
				dev_err(dev, "Could not get sg_table from plane %d\n", plane);
				sg_free_table(sgt_new);
				kfree(sgt_new);
				return -EINVAL;
			}
#ifdef CAPTURE_CONTIG_ALLOC
			new_page = phys_to_page(vb2_dma_contig_plane_dma_addr(vb, plane));
			sg_set_page(sgl_new, new_page, ALIGN(vb2_plane_size(vb, plane),
							     PAGE_SIZE), 0);
			sgl_new = sg_next(sgl_new);
#else
			sgl = img_mmu_get_sgl(sgt);

			while (sgl) {
				sg_set_page(sgl_new, sg_page(sgl), img_mmu_get_sgl_length(sgl), 0);
				sgl = sg_next(sgl);
				sgl_new = sg_next(sgl_new);
			}
#endif
		}

		buf->buf_info.pictbuf_cfg = ctx->pict_bufcfg;
		ret = core_stream_map_buf_sg(ctx->res_str_id,
					     VDEC_BUFTYPE_PICTURE,
					     &buf->buf_info, sgt_new,
					     &buf->buf_map_id);
		sg_free_table(sgt_new);
		kfree(sgt_new);
		if (ret) {
			dev_err(dev, "CAPTURE core_stream_map_buf_sg failed\n");
			return ret;
		}
		if (buf->mapped == FALSE)
			list_add_tail(&buf->list, &ctx->cap_buffers);

		/* Add this to the mappings */
		mapping = kzalloc(sizeof(*mapping), GFP_KERNEL);

		mapping->reuse = TRUE;
		mapping->dma_addr = buf->buf_info.dma_addr;
		mapping->buf_map_id = buf->buf_map_id;
		list_add_tail(&mapping->list, &ctx->cap_mappings);
		buf->mapping = mapping;
	}
	buf->mapped = TRUE;

	return 0;
}

static void vxd_dec_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct vxd_dec_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct vxd_buffer *buf =
		container_of(vb, struct vxd_buffer, buffer.vb.vb2_buf);
	struct vxd_dec_q_data *q_data;
	int i;

	if (V4L2_TYPE_IS_OUTPUT(vb->type)) {
		v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
	} else {
		mutex_lock_nested(ctx->mutex, SUBCLASS_VXD_V4L2);
		if (buf->mapping->reuse) {
			mutex_unlock(ctx->mutex);
			if (ctx->flag_last) {
				q_data = get_q_data(ctx, V4L2_BUF_TYPE_VIDEO_CAPTURE);
				vbuf->flags |= V4L2_BUF_FLAG_LAST;

				for (i = 0; i < q_data->fmt->num_planes; i++)
					vb2_set_plane_payload(&vbuf->vb2_buf, i, 0);

				v4l2_m2m_buf_done(vbuf, VB2_BUF_STATE_DONE);
			} else {
				v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
			}
		} else {
			list_move_tail(&buf->list, &ctx->reuse_queue);
			mutex_unlock(ctx->mutex);
		}
	}
}

static void vxd_dec_return_all_buffers(struct vxd_dec_ctx *ctx,
				       struct vb2_queue *q,
				       enum vb2_buffer_state state)
{
	struct vb2_v4l2_buffer *vb;
	unsigned long flags;

	for (;;) {
		if (V4L2_TYPE_IS_OUTPUT(q->type))
			vb = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
		else
			vb = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);

		if (!vb)
			break;

		spin_lock_irqsave(ctx->dev->lock, flags);
		v4l2_m2m_buf_done(vb, state);
		spin_unlock_irqrestore(ctx->dev->lock, (unsigned long)flags);
	}
}

static int vxd_dec_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	int ret = 0;
	struct vxd_dec_ctx *ctx = vb2_get_drv_priv(vq);

	if (V4L2_TYPE_IS_OUTPUT(vq->type))
		ctx->dst_streaming = TRUE;
	else
		ctx->src_streaming = TRUE;

	if (ctx->dst_streaming && ctx->src_streaming && !ctx->core_streaming) {
		if (!ctx->stream_configured) {
			vxd_dec_return_all_buffers(ctx, vq, VB2_BUF_STATE_ERROR);
			return -EINVAL;
		}
		ctx->eos = FALSE;
		ctx->stop_initiated = FALSE;
		ctx->flag_last = FALSE;
		ret = core_stream_play(ctx->res_str_id);
		if (ret) {
			vxd_dec_return_all_buffers(ctx, vq, VB2_BUF_STATE_ERROR);
			return ret;
		}
		ctx->core_streaming = TRUE;
		ctx->aborting = 0;
	}

	return 0;
}

static void vxd_dec_stop_streaming(struct vb2_queue *vq)
{
	struct vxd_dec_ctx *ctx = vb2_get_drv_priv(vq);
	struct list_head *list;
	struct list_head *temp;
	struct vxd_buffer *buf = NULL;
	struct vxd_mapping *mapping = NULL;

	if (V4L2_TYPE_IS_OUTPUT(vq->type))
		ctx->dst_streaming = FALSE;
	else
		ctx->src_streaming = FALSE;

	if (!ctx->stream_created) {
		vxd_dec_return_all_buffers(ctx, vq, VB2_BUF_STATE_ERROR);
		return;
	}

	if (ctx->core_streaming) {
		core_stream_stop(ctx->res_str_id);
		ctx->core_streaming = FALSE;

		core_stream_flush(ctx->res_str_id, TRUE);
	}
	/* unmap all the output and capture plane buffers */
	if (V4L2_TYPE_IS_OUTPUT(vq->type)) {
		list_for_each(list, &ctx->out_buffers) {
			buf = list_entry(list, struct vxd_buffer, list);
			core_stream_unmap_buf_sg(buf->buf_map_id);
			buf->mapped = FALSE;
			__list_del_entry(&buf->list);
		}
	} else {
		list_for_each_safe(list, temp, &ctx->reuse_queue) {
			buf = list_entry(list, struct vxd_buffer, list);
			list_move_tail(&buf->list, &ctx->cap_buffers);
			v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, &buf->buffer.vb);
		}

		list_for_each(list, &ctx->cap_mappings) {
			mapping = list_entry(list, struct vxd_mapping, list);
			core_stream_unmap_buf_sg(mapping->buf_map_id);
			__list_del_entry(&mapping->list);
		}
	}

	ctx->flag_last = FALSE;
	vxd_dec_return_all_buffers(ctx, vq, VB2_BUF_STATE_ERROR);
}

static const struct vb2_ops vxd_dec_video_ops = {
	.queue_setup = vxd_dec_queue_setup,
	.buf_prepare = vxd_dec_buf_prepare,
	.buf_queue = vxd_dec_buf_queue,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
	.start_streaming = vxd_dec_start_streaming,
	.stop_streaming = vxd_dec_stop_streaming,
};

static int queue_init(void *priv, struct vb2_queue *src_vq, struct vb2_queue *dst_vq)
{
	struct vxd_dec_ctx *ctx = priv;
	struct vxd_dev *vxd = ctx->dev;
	int ret = 0;

	/* src_vq */
	memset(src_vq, 0, sizeof(*src_vq));
	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct vxd_buffer);
	src_vq->ops = &vxd_dec_video_ops;
	src_vq->mem_ops = &vb2_dma_sg_memops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = vxd->mutex_queue;
	src_vq->dev = vxd->v4l2_dev.dev;
	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	/* dst_vq */
	memset(dst_vq, 0, sizeof(*dst_vq));
	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct vxd_buffer);
	dst_vq->ops = &vxd_dec_video_ops;
#ifdef CAPTURE_CONTIG_ALLOC
	dst_vq->mem_ops = &vb2_dma_contig_memops;
#else
	dst_vq->mem_ops = &vb2_dma_sg_memops;
#endif
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = vxd->mutex_queue;
	dst_vq->dev = vxd->v4l2_dev.dev;
	ret = vb2_queue_init(dst_vq);
	if (ret) {
		vb2_queue_release(src_vq);
		return ret;
	}

	return ret;
}

static int vxd_dec_open(struct file *file)
{
	struct vxd_dev *vxd = video_drvdata(file);
	struct vxd_dec_ctx *ctx;
	struct vxd_dec_q_data *s_q_data;
	int i, ret = 0;

	dev_dbg(vxd->dev, "%s:%d vxd %p\n", __func__, __LINE__, vxd);

	if (vxd->no_fw) {
		dev_err(vxd->dev, "Error!! fw binary is not present");
		return -1;
	}

	mutex_lock_nested(vxd->mutex, SUBCLASS_BASE);

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		mutex_unlock(vxd->mutex);
		return -ENOMEM;
	}
	ctx->dev = vxd;

	v4l2_fh_init(&ctx->fh, video_devdata(file));
	file->private_data = &ctx->fh;

	s_q_data = &ctx->q_data[Q_DATA_SRC];
	s_q_data->fmt = &vxd_dec_formats[0];
	s_q_data->width = 1920;
	s_q_data->height = 1080;
	for (i = 0; i < s_q_data->fmt->num_planes; i++) {
		s_q_data->bytesperline[i] = s_q_data->width;
		s_q_data->size_image[i] = s_q_data->bytesperline[i] * s_q_data->height;
	}

	ctx->q_data[Q_DATA_DST] = *s_q_data;

	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(vxd->m2m_dev, ctx, &queue_init);
	if (IS_ERR_VALUE((unsigned long)ctx->fh.m2m_ctx)) {
		ret = (long)(ctx->fh.m2m_ctx);
		goto exit;
	}

	v4l2_fh_add(&ctx->fh);

	ret = idr_alloc_cyclic(vxd->streams, &ctx->stream, VXD_MIN_STREAM_ID, VXD_MAX_STREAM_ID,
			       GFP_KERNEL);
	if (ret < VXD_MIN_STREAM_ID || ret > VXD_MAX_STREAM_ID) {
		dev_err(vxd->dev, "%s: stream id creation failed!\n",
			__func__);
		ret = -EFAULT;
		goto exit;
	}

	ctx->stream.id = ret;
	ctx->stream.ctx = ctx;

	ctx->stream_created = FALSE;
	ctx->stream_configured = FALSE;
	ctx->src_streaming = FALSE;
	ctx->dst_streaming = FALSE;
	ctx->core_streaming = FALSE;
	ctx->eos = FALSE;
	ctx->stop_initiated = FALSE;
	ctx->flag_last = FALSE;

	lst_init(&ctx->seg_list);
	for (i = 0; i < MAX_SEGMENTS; i++)
		lst_add(&ctx->seg_list, &ctx->bstr_segments[i]);

	if (vxd_create_ctx(vxd, ctx))
		goto out_idr_remove;

	ctx->stream.mmu_ctx = ctx->mmu_ctx;
	ctx->stream.ptd = ctx->ptd;

	ctx->mutex = kzalloc(sizeof(*ctx->mutex), GFP_KERNEL);
	if (!ctx->mutex) {
		ret = -ENOMEM;
		goto out_idr_remove;
	}
	mutex_init(ctx->mutex);

	INIT_LIST_HEAD(&ctx->items_done);
	INIT_LIST_HEAD(&ctx->reuse_queue);
	INIT_LIST_HEAD(&ctx->return_queue);
	INIT_LIST_HEAD(&ctx->out_buffers);
	INIT_LIST_HEAD(&ctx->cap_buffers);
	INIT_LIST_HEAD(&ctx->cap_mappings);

	mutex_unlock(vxd->mutex);

	return 0;

out_idr_remove:
	idr_remove(vxd->streams, ctx->stream.id);

exit:
	v4l2_fh_exit(&ctx->fh);
	get_work_buff(ctx->work, TRUE);
	kfree(ctx->work);
	kfree(ctx);
	mutex_unlock(vxd->mutex);
	return ret;
}

static int vxd_dec_release(struct file *file)
{
	struct vxd_dev *vxd = video_drvdata(file);
	struct vxd_dec_ctx *ctx = file2ctx(file);
	struct bspp_ddbuf_array_info *fw_sequ = ctx->fw_sequ;
	struct bspp_ddbuf_array_info *fw_pps = ctx->fw_pps;
	int i, ret = 0;
	struct vxd_dec_q_data *s_q_data;
	struct list_head *list;
	struct list_head *temp;
	struct vxd_buffer *buf = NULL;
	struct vxd_mapping *mapping = NULL;

	s_q_data = &ctx->q_data[Q_DATA_SRC];
	if (ctx->core_streaming) {
		core_stream_stop(ctx->res_str_id);
		ctx->core_streaming = FALSE;

		core_stream_flush(ctx->res_str_id, TRUE);
	}

	list_for_each(list, &ctx->out_buffers) {
		buf = list_entry(list, struct vxd_buffer, list);
		core_stream_unmap_buf_sg(buf->buf_map_id);
		buf->mapped = FALSE;
		__list_del_entry(&buf->list);
	}

	list_for_each_safe(list, temp, &ctx->reuse_queue) {
		buf = list_entry(list, struct vxd_buffer, list);
		core_stream_unmap_buf_sg(buf->buf_map_id);
		buf->mapped = FALSE;
		__list_del_entry(&buf->list);
	}

	list_for_each(list, &ctx->cap_mappings) {
		mapping = list_entry(list, struct vxd_mapping, list);
		core_stream_unmap_buf_sg(mapping->buf_map_id);
		__list_del_entry(&mapping->list);
	}
	if (ctx->stream_created) {
		bspp_stream_destroy(ctx->bspp_context);

		for (i = 0; i < MAX_SEQUENCES; i++) {
			core_stream_unmap_buf(fw_sequ[i].ddbuf_info.bufmap_id);
			img_mem_free(ctx->mem_ctx, fw_sequ[i].ddbuf_info.buf_id);
		}

		if (s_q_data->fmt->std != VDEC_STD_JPEG) {
			for (i = 0; i < MAX_PPSS; i++) {
				core_stream_unmap_buf(fw_pps[i].ddbuf_info.bufmap_id);
				img_mem_free(ctx->mem_ctx, fw_pps[i].ddbuf_info.buf_id);
			}
		}
		core_stream_destroy(ctx->res_str_id);
		ctx->stream_created = FALSE;
	}

	mutex_lock_nested(vxd->mutex, SUBCLASS_BASE);

	vxd_destroy_ctx(vxd, ctx);

	idr_remove(vxd->streams, ctx->stream.id);

	v4l2_fh_del(&ctx->fh);

	v4l2_fh_exit(&ctx->fh);

	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);

	mutex_destroy(ctx->mutex);
	kfree(ctx->mutex);
	ctx->mutex = NULL;

	get_work_buff(ctx->work, TRUE);
	kfree(ctx->work);
	kfree(ctx);

	mutex_unlock(vxd->mutex);

	return ret;
}

static int vxd_dec_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	strncpy(cap->driver, IMG_VXD_DEC_MODULE_NAME, sizeof(cap->driver) - 1);
	strncpy(cap->card, IMG_VXD_DEC_MODULE_NAME, sizeof(cap->card) - 1);
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s", IMG_VXD_DEC_MODULE_NAME);
	cap->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}

static int __enum_fmt(struct v4l2_fmtdesc *f, unsigned int type)
{
	int i, index;
	struct vxd_dec_fmt *fmt = NULL;

	index = 0;
	for (i = 0; i < ARRAY_SIZE(vxd_dec_formats); ++i) {
		if (vxd_dec_formats[i].type & type) {
			if (index == f->index) {
				fmt = &vxd_dec_formats[i];
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

static int vxd_dec_enum_fmt(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	if (V4L2_TYPE_IS_OUTPUT(f->type))
		return __enum_fmt(f, IMG_DEC_FMT_TYPE_OUTPUT);

	return __enum_fmt(f, IMG_DEC_FMT_TYPE_CAPTURE);
}

static struct vxd_dec_fmt *find_format(struct v4l2_format *f, unsigned int type)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(vxd_dec_formats); ++i) {
		if (vxd_dec_formats[i].fourcc == f->fmt.pix_mp.pixelformat &&
		    vxd_dec_formats[i].type == type)
			return &vxd_dec_formats[i];
	}
	return NULL;
}

static unsigned int get_sizeimage(int w, int h, struct vxd_dec_fmt *fmt, int plane)
{
	switch (fmt->fourcc) {
	case V4L2_PIX_FMT_YUV420M:
		return ((plane == 0) ? (w * h) : (w * h / 2));
	case V4L2_PIX_FMT_YUV422M:
		return (w * h);
	default:
		return (w * h * fmt->size_num / fmt->size_den);
	}

	return 0;
}

static unsigned int get_stride(int w, struct vxd_dec_fmt *fmt)
{
	return (ALIGN(w, HW_ALIGN) * fmt->bytes_pp);
}

/*
 * @ Function vxd_get_header_info
 * Run bspp stream submit and preparse once before device_run
 * To retrieve header information
 */
static int vxd_get_header_info(void *priv)
{
	struct vxd_dec_ctx *ctx = priv;
	struct vxd_dev *vxd_dev = ctx->dev;
	struct device *dev = vxd_dev->v4l2_dev.dev;
	struct vb2_v4l2_buffer  *src_vb;
	struct vxd_buffer *src_vxdb;
	struct vxd_buffer *dst_vxdb;
	struct bspp_preparsed_data *preparsed_data;
	unsigned int data_size;
	int ret;

	/*
	 * Checking for queued buffer.
	 * If no next buffer present, do not get information from header.
	 * Else, get header information and store for later use.
	 */
	src_vb =  v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	if (!src_vb) {
		dev_warn(dev, "get_header_info Next src buffer is null\n");
		return IMG_ERROR_INVALID_PARAMETERS;
	}
	mutex_lock_nested(ctx->mutex, SUBCLASS_VXD_V4L2);

	src_vxdb = container_of(src_vb, struct vxd_buffer, buffer.vb);
	/* Setting dst_vxdb to arbitrary value (using src_vb) for now */
	dst_vxdb = container_of(src_vb, struct vxd_buffer, buffer.vb);

	preparsed_data = &dst_vxdb->preparsed_data;

	data_size = vb2_get_plane_payload(&src_vxdb->buffer.vb.vb2_buf, 0);

	ret = bspp_stream_submit_buffer(ctx->bspp_context,
					&src_vxdb->bstr_info,
					src_vxdb->buf_map_id,
					data_size, NULL,
					VDEC_BSTRELEMENT_UNSPECIFIED);
	if (ret) {
		dev_err(dev, "get_header_info bspp_stream_submit_buffer failed %d\n", ret);
		return ret;
	}
	mutex_unlock(ctx->mutex);

	ret = bspp_stream_preparse_buffers(ctx->bspp_context, NULL, 0,
					   &ctx->seg_list,
					   preparsed_data, ctx->eos);
	if (ret) {
		dev_err(dev, "get_header_info bspp_stream_preparse_buffers failed %d\n", ret);
		return ret;
	}

	if (preparsed_data->sequ_hdr_info.com_sequ_hdr_info.max_frame_size.height &&
	    preparsed_data->sequ_hdr_info.com_sequ_hdr_info.max_ref_frame_num) {
		ctx->height = preparsed_data->sequ_hdr_info.com_sequ_hdr_info.max_frame_size.height;
		ctx->max_num_ref_frames =
			preparsed_data->sequ_hdr_info.com_sequ_hdr_info.max_ref_frame_num;
	} else {
		dev_err(dev, "get_header_info preparsed data is null %d\n", ret);
		return ret;
	}

	return 0;
}

static int vxd_dec_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	struct vxd_dec_ctx *ctx = file2ctx(file);
	struct vxd_dec_q_data *q_data;
	struct vxd_dev *vxd_dev = ctx->dev;
	unsigned int i = 0;
	int ret = 0;

	q_data = get_q_data(ctx, f->type);
	if (!q_data)
		return -EINVAL;

	pix_mp->field = V4L2_FIELD_NONE;
	pix_mp->pixelformat = q_data->fmt->fourcc;
	pix_mp->num_planes = q_data->fmt->num_planes;

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		/* The buffer contains compressed image. */
		pix_mp->width = ctx->width;
		pix_mp->height = ctx->height;
		pix_mp->plane_fmt[0].bytesperline = 0;
		pix_mp->plane_fmt[0].sizeimage = q_data->size_image[0];
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		/* The buffer contains decoded YUV image. */
		pix_mp->width = ctx->width;
		pix_mp->height = ctx->height;
		for (i = 0; i < q_data->fmt->num_planes; i++) {
			pix_mp->plane_fmt[i].bytesperline = get_stride(pix_mp->width, q_data->fmt);
			pix_mp->plane_fmt[i].sizeimage = get_sizeimage
							(pix_mp->plane_fmt[i].bytesperline,
							 ctx->height, q_data->fmt, i);
		}
	} else {
		dev_err(vxd_dev->v4l2_dev.dev, "Wrong V4L2_format type\n");
		return -EINVAL;
	}

	return ret;
}

static int vxd_dec_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct vxd_dec_ctx *ctx = file2ctx(file);
	struct vxd_dev *vxd_dev = ctx->dev;
	struct vxd_dec_fmt *fmt;
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	struct v4l2_plane_pix_format *plane_fmt = pix_mp->plane_fmt;
	unsigned int i = 0;
	int ret = 0;

	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		fmt = find_format(f, IMG_DEC_FMT_TYPE_OUTPUT);
		if (!fmt) {
			dev_err(vxd_dev->v4l2_dev.dev, "Unsupported format for source.\n");
			return -EINVAL;
		}
		/*
		 * Allocation for worst case input frame size:
		 * I frame with full YUV size (YUV422)
		 */
		plane_fmt[0].sizeimage = ALIGN(pix_mp->width, HW_ALIGN) *
			ALIGN(pix_mp->height, HW_ALIGN) * 2;
	} else {
		fmt = find_format(f, IMG_DEC_FMT_TYPE_CAPTURE);
		if (!fmt) {
			dev_err(vxd_dev->v4l2_dev.dev, "Unsupported format for dest.\n");
			return -EINVAL;
		}
		for (i = 0; i < fmt->num_planes; i++) {
			plane_fmt[i].bytesperline = get_stride(pix_mp->width, fmt);
			plane_fmt[i].sizeimage = get_sizeimage(plane_fmt[i].bytesperline,
							       pix_mp->height, fmt, i);
		}
		pix_mp->num_planes = fmt->num_planes;
		pix_mp->flags = 0;
	}

	if (pix_mp->field == V4L2_FIELD_ANY)
		pix_mp->field = V4L2_FIELD_NONE;

	return ret;
}

static int vxd_dec_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix_mp;
	struct vxd_dec_ctx *ctx = file2ctx(file);
	struct vxd_dev *vxd_dev = ctx->dev;
	struct device *dev = vxd_dev->v4l2_dev.dev;
	struct vxd_dec_q_data *q_data;
	struct vb2_queue *vq;
	struct vdec_str_configdata strcfgdata;
	int ret = 0;
	unsigned char i = 0, j = 0;

	pix_mp = &f->fmt.pix_mp;

	if (!V4L2_TYPE_IS_OUTPUT(f->type)) {
		int res = vxd_get_header_info(ctx);

		if (res == 0)
			pix_mp->height = ctx->height;
	}

	ret = vxd_dec_try_fmt(file, priv, f);
	if (ret)
		return ret;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	if (vb2_is_busy(vq)) {
		dev_err(dev, "Queue is busy\n");
		return -EBUSY;
	}

	q_data = get_q_data(ctx, f->type);

	if (!q_data)
		return -EINVAL;

	/*
	 * saving the original dimensions to pass to gstreamer (to remove the green
	 * padding on kmsink)
	 */
	ctx->width_orig = pix_mp->width;
	ctx->height_orig = pix_mp->height;

	ctx->width = pix_mp->width;
	ctx->height = pix_mp->height;

	q_data->width = pix_mp->width;
	q_data->height = pix_mp->height;

	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		q_data->fmt = find_format(f, IMG_DEC_FMT_TYPE_OUTPUT);
		q_data->size_image[0] = pix_mp->plane_fmt[0].sizeimage;

		if (!ctx->stream_created) {
			strcfgdata.vid_std = q_data->fmt->std;

			if (strcfgdata.vid_std == VDEC_STD_UNDEFINED) {
				dev_err(dev, "Invalid input format\n");
				return -EINVAL;
			}
			strcfgdata.bstr_format = VDEC_BSTRFORMAT_ELEMENTARY;
			strcfgdata.user_str_id = ctx->stream.id;
			strcfgdata.update_yuv = FALSE;
			strcfgdata.bandwidth_efficient = FALSE;
			strcfgdata.disable_mvc = FALSE;
			strcfgdata.full_scan = FALSE;
			strcfgdata.immediate_decode = TRUE;
			strcfgdata.intra_frame_closed_gop = TRUE;

			ret = core_stream_create(ctx, &strcfgdata, &ctx->res_str_id);
			if (ret) {
				dev_err(dev, "Core stream create failed\n");
				return -EINVAL;
			}
			ctx->stream_created = TRUE;
			if (ctx->opconfig_pending) {
				ret = vxd_dec_submit_opconfig(ctx);
				if (ret) {
					dev_err(dev, "Output config failed\n");
					return -EINVAL;
				}
			}

			vxd_dec_alloc_bspp_resource(ctx, strcfgdata.vid_std);
			ret = bspp_stream_create(&strcfgdata,
						 &ctx->bspp_context,
						 ctx->fw_sequ,
						 ctx->fw_pps);
			if (ret) {
				dev_err(dev, "BSPP stream create failed %d\n", ret);
				return ret;
			}
		} else if (q_data->fmt !=
			find_format(f, IMG_DEC_FMT_TYPE_OUTPUT)) {
			dev_err(dev, "Input format already set\n");
			return -EBUSY;
		}
	} else {
		q_data->fmt = find_format(f, IMG_DEC_FMT_TYPE_CAPTURE);
		for (i = 0; i < q_data->fmt->num_planes; i++) {
			q_data->size_image[i] =
				get_sizeimage(get_stride(pix_mp->width, q_data->fmt),
					      ctx->height, q_data->fmt, i);
		}

		ctx->str_opcfg.pixel_info.pixfmt = q_data->fmt->pixfmt;
		ctx->str_opcfg.pixel_info.chroma_interleave = q_data->fmt->interleave;
		ctx->str_opcfg.pixel_info.chroma_fmt = TRUE;
		ctx->str_opcfg.pixel_info.chroma_fmt_idc = q_data->fmt->idc;

		if (q_data->fmt->pixfmt == IMG_PIXFMT_420PL12YUV10_MSB ||
		    q_data->fmt->pixfmt == IMG_PIXFMT_422PL12YUV10_MSB) {
			ctx->str_opcfg.pixel_info.mem_pkg = PIXEL_BIT10_MSB_MP;
			ctx->str_opcfg.pixel_info.bitdepth_y = 10;
			ctx->str_opcfg.pixel_info.bitdepth_c = 10;
		} else {
			ctx->str_opcfg.pixel_info.mem_pkg = PIXEL_BIT8_MP;
			ctx->str_opcfg.pixel_info.bitdepth_y = 8;
			ctx->str_opcfg.pixel_info.bitdepth_c = 8;
		}

		ctx->str_opcfg.force_oold = FALSE;

		ctx->pict_bufcfg.coded_width = pix_mp->width;
		ctx->pict_bufcfg.coded_height = pix_mp->height;
		ctx->pict_bufcfg.pixel_fmt = q_data->fmt->pixfmt;
		for (i = 0; i < pix_mp->num_planes; i++) {
			q_data->bytesperline[i] = get_stride(q_data->width, q_data->fmt);
			if (q_data->bytesperline[i] <
				pix_mp->plane_fmt[0].bytesperline)
				q_data->bytesperline[i] =
					ALIGN(pix_mp->plane_fmt[0].bytesperline, HW_ALIGN);
			pix_mp->plane_fmt[0].bytesperline =
				q_data->bytesperline[i];
			ctx->pict_bufcfg.stride[i] = q_data->bytesperline[i];
		}
		for (j = i; j < IMG_MAX_NUM_PLANES; j++) {
			if ((i - 1) < 0)
				i++;
			ctx->pict_bufcfg.stride[j] =
				q_data->bytesperline[i - 1];
		}
		ctx->pict_bufcfg.stride_alignment = HW_ALIGN;
		ctx->pict_bufcfg.byte_interleave = FALSE;
		for (i = 0; i < pix_mp->num_planes; i++) {
			unsigned int plane_size =
				get_sizeimage(ctx->pict_bufcfg.stride[i],
					      ctx->pict_bufcfg.coded_height,
					      q_data->fmt, i);
			ctx->pict_bufcfg.buf_size += ALIGN(plane_size, PAGE_SIZE);
			ctx->pict_bufcfg.plane_size[i] = plane_size;
			pix_mp->plane_fmt[i].sizeimage = plane_size;
		}
		if (q_data->fmt->pixfmt == 86031 ||
		    q_data->fmt->pixfmt == 81935) {
			/* Handle the v4l2 multi-planar formats */
			ctx->str_opcfg.pixel_info.num_planes = 3;
			ctx->pict_bufcfg.packed = FALSE;
			for (i = 0; i < pix_mp->num_planes; i++) {
				ctx->pict_bufcfg.chroma_offset[i] =
					ALIGN(pix_mp->plane_fmt[i].sizeimage, PAGE_SIZE);
				ctx->pict_bufcfg.chroma_offset[i] +=
					(i ? ctx->pict_bufcfg.chroma_offset[i - 1] : 0);
			}
		} else {
			/* IMG Decoders support only multi-planar formats */
			ctx->str_opcfg.pixel_info.num_planes = 2;
			ctx->pict_bufcfg.packed = TRUE;
			ctx->pict_bufcfg.chroma_offset[0] = 0;
			ctx->pict_bufcfg.chroma_offset[1] = 0;
		}

		vxd_dec_submit_opconfig(ctx);
	}

	return ret;
}

static int vxd_dec_subscribe_event(struct v4l2_fh *fh, const struct v4l2_event_subscription *sub)
{
	if (sub->type != V4L2_EVENT_EOS)
		return -EINVAL;

	v4l2_event_subscribe(fh, sub, 0, NULL);
	return 0;
}

static int vxd_dec_try_cmd(struct file *file, void *fh, struct v4l2_decoder_cmd *cmd)
{
	if (cmd->cmd != V4L2_DEC_CMD_STOP)
		return -EINVAL;

	return 0;
}

static int vxd_dec_cmd(struct file *file, void *fh, struct v4l2_decoder_cmd *cmd)
{
	struct vxd_dec_ctx *ctx = file2ctx(file);

	if (cmd->cmd != V4L2_DEC_CMD_STOP)
		return -EINVAL;

#ifdef DEBUG_DECODER_DRIVER
	pr_info("%s CMD_STOP\n", __func__);
#endif
	/*
	 * When stop command is received, notify device_run if it is
	 * scheduled to run, or tell the decoder that eos has
	 * happened.
	 */
	mutex_lock_nested(ctx->mutex, SUBCLASS_VXD_V4L2);
	if (v4l2_m2m_num_src_bufs_ready(ctx->fh.m2m_ctx) > 0) {
#ifdef DEBUG_DECODER_DRIVER
		pr_info("V4L2 src bufs not empty, set a flag to notify device_run\n");
#endif
		ctx->stop_initiated = TRUE;
		mutex_unlock(ctx->mutex);
	} else {
		if (ctx->num_decoding) {
#ifdef DEBUG_DECODER_DRIVER
			pr_info("buffers are still being decoded, so just set eos flag\n");
#endif
			ctx->eos = TRUE;
			mutex_unlock(ctx->mutex);
		} else {
			mutex_unlock(ctx->mutex);
#ifdef DEBUG_DECODER_DRIVER
			pr_info("All buffers are decoded, so issue dummy stream end\n");
#endif
			vxd_return_resource((void *)ctx, VXD_CB_STR_END, 0);
		}
	}

	return 0;
}

static int vxd_g_selection(struct file *file, void *fh, struct v4l2_selection *s)
{
	struct vxd_dec_ctx *ctx = file2ctx(file);
	bool def_bounds = true;

	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE &&
	    s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
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
		def_bounds = false;
		break;
	case V4L2_SEL_TGT_CROP:
		if (s->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
			return -EINVAL;
		def_bounds = false;
		break;
	default:
		return -EINVAL;
	}

	if (def_bounds) {
		s->r.left = 0;
		s->r.top = 0;
		s->r.width = ctx->width_orig;
		s->r.height = ctx->height_orig;
	}

	return 0;
}

static const struct v4l2_ioctl_ops vxd_dec_ioctl_ops = {
	.vidioc_querycap = vxd_dec_querycap,

	.vidioc_enum_fmt_vid_cap = vxd_dec_enum_fmt,
	.vidioc_g_fmt_vid_cap_mplane = vxd_dec_g_fmt,
	.vidioc_try_fmt_vid_cap_mplane = vxd_dec_try_fmt,
	.vidioc_s_fmt_vid_cap_mplane = vxd_dec_s_fmt,

	.vidioc_enum_fmt_vid_out = vxd_dec_enum_fmt,
	.vidioc_g_fmt_vid_out_mplane = vxd_dec_g_fmt,
	.vidioc_try_fmt_vid_out_mplane = vxd_dec_try_fmt,
	.vidioc_s_fmt_vid_out_mplane = vxd_dec_s_fmt,

	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,

	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,
	.vidioc_log_status = v4l2_ctrl_log_status,
	.vidioc_subscribe_event = vxd_dec_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
	.vidioc_try_decoder_cmd = vxd_dec_try_cmd,
	.vidioc_decoder_cmd = vxd_dec_cmd,

	.vidioc_g_selection = vxd_g_selection,
};

static const struct v4l2_file_operations vxd_dec_fops = {
	.owner = THIS_MODULE,
	.open = vxd_dec_open,
	.release = vxd_dec_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

static void device_run(void *priv)
{
	struct vxd_dec_ctx *ctx = priv;
	struct vxd_dev *vxd_dev = ctx->dev;
	struct device *dev = vxd_dev->v4l2_dev.dev;
	struct vb2_v4l2_buffer  *src_vb;
	struct vb2_v4l2_buffer  *dst_vb;
	struct vxd_buffer *src_vxdb;
	struct vxd_buffer *dst_vxdb;
	struct bspp_bitstr_seg *item = NULL, *next = NULL;
	struct bspp_preparsed_data *preparsed_data;
	unsigned int data_size;
	int ret;
	struct timespec64 time;
	static int cnt;
	int i;

	mutex_lock_nested(ctx->mutex, SUBCLASS_VXD_V4L2);
	ctx->num_decoding++;

	src_vb = v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
	if (!src_vb)
		dev_err(dev, "Next src buffer is null\n");

	dst_vb = v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
	if (!dst_vb)
		dev_err(dev, "Next dst buffer is null\n");

	src_vxdb = container_of(src_vb, struct vxd_buffer, buffer.vb);
	dst_vxdb = container_of(dst_vb, struct vxd_buffer, buffer.vb);

	preparsed_data = &dst_vxdb->preparsed_data;

	data_size = vb2_get_plane_payload(&src_vxdb->buffer.vb.vb2_buf, 0);

	ret = bspp_stream_submit_buffer(ctx->bspp_context,
					&src_vxdb->bstr_info,
					src_vxdb->buf_map_id,
					data_size, NULL,
					VDEC_BSTRELEMENT_UNSPECIFIED);
	if (ret)
		dev_err(dev, "bspp_stream_submit_buffer failed %d\n", ret);

	if (ctx->stop_initiated &&
	    (v4l2_m2m_num_src_bufs_ready(ctx->fh.m2m_ctx) == 0))
		ctx->eos = TRUE;

	mutex_unlock(ctx->mutex);

	ret = bspp_stream_preparse_buffers(ctx->bspp_context, NULL, 0, &ctx->seg_list,
					   preparsed_data, ctx->eos);
	if (ret)
		dev_err(dev, "bspp_stream_preparse_buffers failed %d\n", ret);

	ktime_get_real_ts64(&time);
	vxd_dev->time_drv[cnt].start_time = timespec64_to_ns(&time);
	vxd_dev->time_drv[cnt].id = dst_vxdb->buf_map_id;
	cnt++;

	if (cnt >= ARRAY_SIZE(vxd_dev->time_drv))
		cnt = 0;

	core_stream_fill_pictbuf(dst_vxdb->buf_map_id);

	if (preparsed_data->new_sequence) {
		src_vxdb->seq_unit.str_unit_type =
			VDECDD_STRUNIT_SEQUENCE_START;
		src_vxdb->seq_unit.str_unit_handle = ctx;
		src_vxdb->seq_unit.err_flags = 0;
		src_vxdb->seq_unit.dd_data = NULL;
		src_vxdb->seq_unit.seq_hdr_info =
			&preparsed_data->sequ_hdr_info;
		src_vxdb->seq_unit.seq_hdr_id = 0;
		src_vxdb->seq_unit.closed_gop = TRUE;
		src_vxdb->seq_unit.eop = FALSE;
		src_vxdb->seq_unit.pict_hdr_info = NULL;
		src_vxdb->seq_unit.dd_pict_data = NULL;
		src_vxdb->seq_unit.last_pict_in_seq = FALSE;
		src_vxdb->seq_unit.str_unit_tag = NULL;
		src_vxdb->seq_unit.decode = FALSE;
		src_vxdb->seq_unit.features = 0;
		core_stream_submit_unit(ctx->res_str_id, &src_vxdb->seq_unit);
	}

	src_vxdb->pic_unit.str_unit_type = VDECDD_STRUNIT_PICTURE_START;
	src_vxdb->pic_unit.str_unit_handle = ctx;
	src_vxdb->pic_unit.err_flags = 0;
	/* Move the processed segments to the submission buffer */
	for (i = 0; i < BSPP_MAX_PICTURES_PER_BUFFER; i++) {
		item = lst_first(&preparsed_data->picture_data.pre_pict_seg_list[i]);
		while (item) {
			next = lst_next(item);
			lst_remove(&preparsed_data->picture_data.pre_pict_seg_list[i], item);
			lst_add(&src_vxdb->pic_unit.bstr_seg_list, item);
			item = next;
		}
		/* Move the processed segments to the submission buffer */
		item = lst_first(&preparsed_data->picture_data.pict_seg_list[i]);
		while (item) {
			next = lst_next(item);
			lst_remove(&preparsed_data->picture_data.pict_seg_list[i], item);
			lst_add(&src_vxdb->pic_unit.bstr_seg_list, item);
			item = next;
		}
	}

	src_vxdb->pic_unit.dd_data = NULL;
	src_vxdb->pic_unit.seq_hdr_info = NULL;
	src_vxdb->pic_unit.seq_hdr_id = 0;
	if (preparsed_data->new_sequence)
		src_vxdb->pic_unit.closed_gop = TRUE;
	else
		src_vxdb->pic_unit.closed_gop = FALSE;
	src_vxdb->pic_unit.eop = TRUE;
	src_vxdb->pic_unit.eos = ctx->eos;
	src_vxdb->pic_unit.pict_hdr_info =
		&preparsed_data->picture_data.pict_hdr_info;
	src_vxdb->pic_unit.dd_pict_data = NULL;
	src_vxdb->pic_unit.last_pict_in_seq = FALSE;
	src_vxdb->pic_unit.str_unit_tag = NULL;
	src_vxdb->pic_unit.decode = FALSE;
	src_vxdb->pic_unit.features = 0;
	core_stream_submit_unit(ctx->res_str_id, &src_vxdb->pic_unit);

	src_vxdb->end_unit.str_unit_type = VDECDD_STRUNIT_PICTURE_END;
	src_vxdb->end_unit.str_unit_handle = ctx;
	src_vxdb->end_unit.err_flags = 0;
	src_vxdb->end_unit.dd_data = NULL;
	src_vxdb->end_unit.seq_hdr_info = NULL;
	src_vxdb->end_unit.seq_hdr_id = 0;
	src_vxdb->end_unit.closed_gop = FALSE;
	src_vxdb->end_unit.eop = FALSE;
	src_vxdb->end_unit.eos = ctx->eos;
	src_vxdb->end_unit.pict_hdr_info = NULL;
	src_vxdb->end_unit.dd_pict_data = NULL;
	src_vxdb->end_unit.last_pict_in_seq = FALSE;
	src_vxdb->end_unit.str_unit_tag = NULL;
	src_vxdb->end_unit.decode = FALSE;
	src_vxdb->end_unit.features = 0;
	core_stream_submit_unit(ctx->res_str_id, &src_vxdb->end_unit);
}

static int job_ready(void *priv)
{
	struct vxd_dec_ctx *ctx = priv;

	if (v4l2_m2m_num_src_bufs_ready(ctx->fh.m2m_ctx) < 1 ||
	    v4l2_m2m_num_dst_bufs_ready(ctx->fh.m2m_ctx) < 1 ||
	    !ctx->core_streaming)
		return 0;

	return 1;
}

static void job_abort(void *priv)
{
	struct vxd_dec_ctx *ctx = priv;

	/* Cancel the transaction at next callback */
	ctx->aborting = 1;
}

static const struct v4l2_m2m_ops m2m_ops = {
	.device_run = device_run,
	.job_ready = job_ready,
	.job_abort = job_abort,
};

static const struct of_device_id vxd_dec_of_match[] = {
	{.compatible = "img,d5500-vxd"},
	{ /* end */},
};
MODULE_DEVICE_TABLE(of, vxd_dec_of_match);

static int vxd_dec_probe(struct platform_device *pdev)
{
	struct vxd_dev *vxd;
	struct resource *res;
	const struct of_device_id *of_dev_id;
	int ret;
	int module_irq;
	struct video_device *vfd;

	struct heap_config *heap_configs;
	int num_heaps;
	unsigned int i_heap_id;
	/* Protect structure fields */
	spinlock_t **lock;

	of_dev_id = of_match_device(vxd_dec_of_match, &pdev->dev);
	if (!of_dev_id) {
		dev_err(&pdev->dev, "%s: Unable to match device\n", __func__);
		return -ENODEV;
	}

	dma_set_mask(&pdev->dev, DMA_BIT_MASK(40));

	vxd = devm_kzalloc(&pdev->dev, sizeof(*vxd), GFP_KERNEL);
	if (!vxd)
		return -ENOMEM;

	vxd->dev = &pdev->dev;
	vxd->plat_dev = pdev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	vxd->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR_VALUE((unsigned long)vxd->reg_base))
		return (long)(vxd->reg_base);

	module_irq = platform_get_irq(pdev, 0);
	if (module_irq < 0)
		return -ENXIO;
	vxd->module_irq = module_irq;
#ifdef ERROR_RECOVERY_SIMULATION
	g_module_irq = module_irq;
#endif

	heap_configs = vxd_dec_heap_configs;
	num_heaps = ARRAY_SIZE(vxd_dec_heap_configs);

	vxd->mutex = kzalloc(sizeof(*vxd->mutex), GFP_KERNEL);
	if (!vxd->mutex)
		return -ENOMEM;

	mutex_init(vxd->mutex);

	vxd->mutex_queue = kzalloc(sizeof(*vxd->mutex_queue), GFP_KERNEL);
	if (!vxd->mutex_queue)
		return -ENOMEM;

	mutex_init(vxd->mutex_queue);

	platform_set_drvdata(pdev, vxd);

	pm_runtime_enable(&pdev->dev);
	ret = pm_runtime_get_sync(&pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "%s: failed to enable clock, status = %d\n", __func__, ret);
		goto exit;
	}

	/* Read HW properties */
	ret = vxd_pvdec_get_props(vxd->dev, vxd->reg_base, &vxd->props);
	if (ret) {
		dev_err(&pdev->dev, "%s: failed to fetch core properties!\n", __func__);
		ret = -ENXIO;
		goto out_put_sync;
	}
	vxd->mmu_config_addr_width = VXD_EXTRN_ADDR_WIDTH(vxd->props);
#ifdef DEBUG_DECODER_DRIVER
	dev_info(&pdev->dev, "hw:%u.%u.%u, num_pix: %d, num_ent: %d, mmu: %d, MTX RAM: %d\n",
		 VXD_MAJ_REV(vxd->props),
		 VXD_MIN_REV(vxd->props),
		 VXD_MAINT_REV(vxd->props),
		 VXD_NUM_PIX_PIPES(vxd->props),
		 VXD_NUM_ENT_PIPES(vxd->props),
		 VXD_EXTRN_ADDR_WIDTH(vxd->props),
		 vxd->props.mtx_ram_size);
#endif

	INIT_LIST_HEAD(&vxd->msgs);
	INIT_LIST_HEAD(&vxd->pend);

	/* initialize memory manager */
	ret = img_mem_init(&pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to initialize memory\n");
		ret = -ENOMEM;
		goto out_put_sync;
	}
	vxd->streams = kzalloc(sizeof(*vxd->streams), GFP_KERNEL);
	if (!vxd->streams) {
		ret = -ENOMEM;
		goto out_init;
	}

	idr_init(vxd->streams);

	ret = vxd_init(&pdev->dev, vxd, heap_configs, num_heaps);
	if (ret) {
		dev_err(&pdev->dev, "%s: main component initialisation failed!\n", __func__);
		goto out_idr_init;
	}

	/* initialize core */
	i_heap_id = vxd_g_internal_heap_id();
	if (i_heap_id < 0) {
		dev_err(&pdev->dev, "%s: Invalid internal heap id", __func__);
		goto out_vxd_init;
	}
	ret = core_initialise(vxd, i_heap_id, vxd_return_resource);
	if (ret) {
		dev_err(&pdev->dev, "%s: core initialization failed!", __func__);
		goto out_vxd_init;
	}

	vxd->fw_refcnt = 0;
	vxd->hw_on = 0;

#ifdef DEBUG_DECODER_DRIVER
	vxd->hw_pm_delay = 10000;
	vxd->hw_dwr_period = 10000;
#else
	vxd->hw_pm_delay = 1000;
	vxd->hw_dwr_period = 1000;
#endif
	ret = vxd_prepare_fw(vxd);
	if (ret) {
		dev_err(&pdev->dev, "%s fw acquire failed!", __func__);
		goto out_core_init;
	}

	if (vxd->no_fw) {
		dev_err(&pdev->dev, "%s fw acquire failed!", __func__);
		goto out_core_init;
	}

	 lock = (spinlock_t **)&vxd->lock;
	*lock = kzalloc(sizeof(spinlock_t), GFP_KERNEL);

	if (!(*lock)) {
		pr_err("Memory allocation failed for spin-lock\n");
		ret = ENOMEM;
		goto out_core_init;
	}
	spin_lock_init(*lock);

	ret = v4l2_device_register(&pdev->dev, &vxd->v4l2_dev);
	if (ret)
		goto out_clean_fw;

#ifdef ERROR_RECOVERY_SIMULATION
	/*
	 * create a sysfs entry here, to debug firmware error recovery.
	 */
	vxd_dec_kobject = kobject_create_and_add("vxd_decoder", kernel_kobj);
	if (!vxd_dec_kobject) {
		dev_err(&pdev->dev, "Failed to create kernel object\n");
		goto out_clean_fw;
	}

	ret = sysfs_create_group(vxd_dec_kobject, &attr_group);
	if (ret) {
		dev_err(&pdev->dev, "Failed to create sysfs files\n");
		kobject_put(vxd_dec_kobject);
	}
#endif

	vfd = video_device_alloc();
	if (!vfd) {
		dev_err(&pdev->dev, "Failed to allocate video device\n");
		ret = -ENOMEM;
		goto out_v4l2_device;
	}

	snprintf(vfd->name, sizeof(vfd->name), "%s", IMG_VXD_DEC_MODULE_NAME);
	vfd->fops = &vxd_dec_fops;
	vfd->ioctl_ops = &vxd_dec_ioctl_ops;
	vfd->minor = -1;
	vfd->release = video_device_release;
	vfd->vfl_dir = VFL_DIR_M2M;
	vfd->v4l2_dev = &vxd->v4l2_dev;
	vfd->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	vfd->lock = vxd->mutex;

	vxd->vfd_dec = vfd;
	video_set_drvdata(vfd, vxd);

	ret = devm_request_threaded_irq(&pdev->dev, module_irq, (irq_handler_t)hard_isrcb,
					(irq_handler_t)soft_thread_irq, IRQF_SHARED,
					IMG_VXD_DEC_MODULE_NAME, pdev);
	if (ret) {
		dev_err(&pdev->dev, "failed to request irq\n");
		goto out_vid_dev;
	}

	vxd->m2m_dev = v4l2_m2m_init(&m2m_ops);
	if (IS_ERR_VALUE((unsigned long)vxd->m2m_dev)) {
		dev_err(&pdev->dev, "Failed to init mem2mem device\n");
		ret = -EINVAL;
		goto out_vid_dev;
	}

	ret = video_register_device(vfd, VFL_TYPE_VIDEO, 0);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register video device\n");
		goto out_vid_reg;
	}
	v4l2_info(&vxd->v4l2_dev, "decoder registered as /dev/video%d\n", vfd->num);

	return 0;

out_vid_reg:
	v4l2_m2m_release(vxd->m2m_dev);

out_vid_dev:
	video_device_release(vfd);

out_v4l2_device:
	v4l2_device_unregister(&vxd->v4l2_dev);

out_clean_fw:
	vxd_clean_fw_resources(vxd);

out_core_init:
	core_deinitialise();

out_vxd_init:
	vxd_deinit(vxd);

out_idr_init:
	idr_destroy(vxd->streams);
	kfree(vxd->streams);

out_init:
	img_mem_exit();

out_put_sync:
	pm_runtime_put_sync(&pdev->dev);

exit:
	pm_runtime_disable(&pdev->dev);
	mutex_destroy(vxd->mutex);
	kfree(vxd->mutex);
	vxd->mutex = NULL;

	return ret;
}

static int vxd_dec_remove(struct platform_device *pdev)
{
	struct vxd_dev *vxd = platform_get_drvdata(pdev);

	core_deinitialise();

	vxd_clean_fw_resources(vxd);
	vxd_deinit(vxd);
	idr_destroy(vxd->streams);
	kfree(vxd->streams);
	get_delayed_work_buff(&vxd->dwork, TRUE);
	kfree(&vxd->lock);
	img_mem_exit();

	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	kfree(vxd->dwork);
	mutex_destroy(vxd->mutex);
	mutex_destroy(vxd->mutex_queue);
	kfree(vxd->mutex);
	kfree(vxd->mutex_queue);
	vxd->mutex = NULL;
	vxd->mutex_queue = NULL;

	video_unregister_device(vxd->vfd_dec);
	v4l2_m2m_release(vxd->m2m_dev);
	v4l2_device_unregister(&vxd->v4l2_dev);

	return 0;
}

static int __maybe_unused vxd_dec_suspend(struct device *dev)
{
	int ret = 0;

	ret = vxd_suspend_dev(dev);
	if (ret)
		dev_err(dev, "failed to suspend core hw!\n");

	return ret;
}

static int __maybe_unused vxd_dec_resume(struct device *dev)
{
	int ret = 0;

	ret = vxd_resume_dev(dev);
	if (ret)
		dev_err(dev, "failed to resume core hw!\n");

	return ret;
}

static UNIVERSAL_DEV_PM_OPS(vxd_dec_pm_ops,
	vxd_dec_suspend, vxd_dec_resume, NULL);

static const struct platform_driver vxd_dec_driver = {
	.probe = vxd_dec_probe,
	.remove = vxd_dec_remove,
	.driver = {
		.name = "img_dec",
		.pm = &vxd_dec_pm_ops,
		.of_match_table = vxd_dec_of_match,
	},
};
module_platform_driver(vxd_dec_driver);

MODULE_AUTHOR("Prashanth Kumar Amai <prashanth.ka@pathpartnertech.com> Sidraya Jayagond <sidraya.bj@pathpartnertech.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("IMG D5520 video decoder driver");
