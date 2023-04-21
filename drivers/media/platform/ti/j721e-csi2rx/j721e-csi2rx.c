// SPDX-License-Identifier: GPL-2.0-only
/*
 * TI CSI2 RX driver.
 *
 * Copyright (C) 2021 Texas Instruments Incorporated - https://www.ti.com/
 *
 * Author: Pratyush Yadav <p.yadav@ti.com>
 */

#include <linux/bitfield.h>
#include <linux/dmaengine.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#include <media/mipi-csi2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mc.h>
#include <media/videobuf2-dma-contig.h>

#define TI_CSI2RX_MODULE_NAME		"j721e-csi2rx"

#define SHIM_CNTL			0x10
#define SHIM_CNTL_PIX_RST		BIT(0)

#define SHIM_DMACNTX			0x20
#define SHIM_DMACNTX_EN			BIT(31)
#define SHIM_DMACNTX_YUV422		GENMASK(27, 26)
#define SHIM_DMACNTX_FMT		GENMASK(5, 0)
#define SHIM_DMACNTX_UYVY		0
#define SHIM_DMACNTX_VYUY		1
#define SHIM_DMACNTX_YUYV		2
#define SHIM_DMACNTX_YVYU		3

#define SHIM_PSI_CFG0			0x24
#define SHIM_PSI_CFG0_SRC_TAG		GENMASK(15, 0)
#define SHIM_PSI_CFG0_DST_TAG		GENMASK(31, 16)

#define PSIL_WORD_SIZE_BYTES		16
#define TI_CSI2RX_NUM_CTX		1

/*
 * There are no hard limits on the width or height. The DMA engine can handle
 * all sizes. The max width and height are arbitrary numbers for this driver.
 * Use 16K * 16K as the arbitrary limit. It is large enough that it is unlikely
 * the limit will be hit in practice.
 */
#define MAX_WIDTH_BYTES			SZ_16K
#define MAX_HEIGHT_LINES		SZ_16K

struct ti_csi2rx_fmt {
	u32				fourcc;	/* Four character code. */
	u32				code;	/* Mbus code. */
	u32				csi_dt;	/* CSI Data type. */
	u8				bpp;	/* Bits per pixel. */
};

struct ti_csi2rx_buffer {
	/* Common v4l2 buffer. Must be first. */
	struct vb2_v4l2_buffer		vb;
	struct list_head		list;
	struct ti_csi2rx_ctx		*ctx;
};

enum ti_csi2rx_dma_state {
	TI_CSI2RX_DMA_STOPPED,	/* Streaming not started yet. */
	TI_CSI2RX_DMA_IDLE,	/* Streaming but no pending DMA operation. */
	TI_CSI2RX_DMA_ACTIVE,	/* Streaming and pending DMA operation. */
};

struct ti_csi2rx_dma {
	/* Protects all fields in this struct. */
	spinlock_t			lock;
	struct dma_chan			*chan;
	/* Buffers queued to the driver, waiting to be processed by DMA. */
	struct list_head		queue;
	enum ti_csi2rx_dma_state	state;
	/*
	 * Current buffer being processed by DMA. NULL if no buffer is being
	 * processed.
	 */
	struct ti_csi2rx_buffer		*curr;
};

struct ti_csi2rx_dev;

struct ti_csi2rx_ctx {
	struct ti_csi2rx_dev		*csi;
	struct video_device		vdev;
	struct vb2_queue		vidq;
	struct mutex			mutex; /* To serialize ioctls. */
	struct v4l2_format		v_fmt;
	struct ti_csi2rx_dma		dma;
	u32				sequence;
	u32				idx;
};

struct ti_csi2rx_dev {
	struct device			*dev;
	void __iomem			*shim;
	struct v4l2_async_notifier	notifier;
	struct media_device		mdev;
	struct media_pipeline		pipe;
	struct media_pad		pad;
	struct v4l2_device		v4l2_dev;
	struct v4l2_subdev		*subdev;
	struct ti_csi2rx_ctx		ctx[TI_CSI2RX_NUM_CTX];
};

static const struct ti_csi2rx_fmt formats[] = {
	{
		.fourcc			= V4L2_PIX_FMT_YUYV,
		.code			= MEDIA_BUS_FMT_YUYV8_1X16,
		.csi_dt			= MIPI_CSI2_DT_YUV422_8B,
		.bpp			= 16,
	}, {
		.fourcc			= V4L2_PIX_FMT_UYVY,
		.code			= MEDIA_BUS_FMT_UYVY8_1X16,
		.csi_dt			= MIPI_CSI2_DT_YUV422_8B,
		.bpp			= 16,
	}, {
		.fourcc			= V4L2_PIX_FMT_YVYU,
		.code			= MEDIA_BUS_FMT_YVYU8_1X16,
		.csi_dt			= MIPI_CSI2_DT_YUV422_8B,
		.bpp			= 16,
	}, {
		.fourcc			= V4L2_PIX_FMT_VYUY,
		.code			= MEDIA_BUS_FMT_VYUY8_1X16,
		.csi_dt			= MIPI_CSI2_DT_YUV422_8B,
		.bpp			= 16,
	},

	/* More formats can be supported but they are not listed for now. */
};

static const unsigned int num_formats = ARRAY_SIZE(formats);

/* Forward declaration needed by ti_csi2rx_dma_callback. */
static int ti_csi2rx_start_dma(struct ti_csi2rx_ctx *ctx,
			       struct ti_csi2rx_buffer *buf);

static const struct ti_csi2rx_fmt *find_format_by_pix(u32 pixelformat)
{
	unsigned int i;

	for (i = 0; i < num_formats; i++) {
		if (formats[i].fourcc == pixelformat)
			return &formats[i];
	}

	return NULL;
}

static const struct ti_csi2rx_fmt *find_format_by_code(u32 code)
{
	unsigned int i;

	for (i = 0; i < num_formats; i++) {
		if (formats[i].code == code)
			return &formats[i];
	}

	return NULL;
}

static void ti_csi2rx_fill_fmt(const struct ti_csi2rx_fmt *csi_fmt,
			       struct v4l2_format *v4l2_fmt)
{
	struct v4l2_pix_format *pix = &v4l2_fmt->fmt.pix;
	unsigned int pixels_in_word;
	u8 bpp = csi_fmt->bpp;
	u32 bpl;

	pixels_in_word = PSIL_WORD_SIZE_BYTES * 8 / bpp;

	pix->width = clamp_t(unsigned int, pix->width,
			     pixels_in_word,
			     MAX_WIDTH_BYTES * 8 / bpp);
	pix->width = rounddown(pix->width, pixels_in_word);

	pix->height = clamp_t(unsigned int, pix->height, 1, MAX_HEIGHT_LINES);

	v4l2_fmt->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	pix->pixelformat = csi_fmt->fourcc;
	pix->colorspace = V4L2_COLORSPACE_SRGB;
	pix->sizeimage = pix->height * pix->width * (bpp / 8);

	bpl = (pix->width * ALIGN(bpp, 8)) >> 3;
	pix->bytesperline = ALIGN(bpl, 16);
}

static int ti_csi2rx_querycap(struct file *file, void *priv,
			      struct v4l2_capability *cap)
{
	strscpy(cap->driver, TI_CSI2RX_MODULE_NAME, sizeof(cap->driver));
	strscpy(cap->card, TI_CSI2RX_MODULE_NAME, sizeof(cap->card));

	return 0;
}

static int ti_csi2rx_enum_fmt_vid_cap(struct file *file, void *priv,
				      struct v4l2_fmtdesc *f)
{
	if (f->index >= num_formats)
		return -EINVAL;

	memset(f->reserved, 0, sizeof(f->reserved));
	f->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	f->pixelformat = formats[f->index].fourcc;

	return 0;
}

static int ti_csi2rx_g_fmt_vid_cap(struct file *file, void *prov,
				   struct v4l2_format *f)
{
	struct ti_csi2rx_ctx *ctx = video_drvdata(file);

	*f = ctx->v_fmt;

	return 0;
}

static int ti_csi2rx_try_fmt_vid_cap(struct file *file, void *priv,
				     struct v4l2_format *f)
{
	const struct ti_csi2rx_fmt *fmt;

	/*
	 * Default to the first format if the requested pixel format code isn't
	 * supported.
	 */
	fmt = find_format_by_pix(f->fmt.pix.pixelformat);
	if (!fmt)
		fmt = &formats[0];

	if (f->fmt.pix.field == V4L2_FIELD_ANY)
		f->fmt.pix.field = V4L2_FIELD_NONE;

	if (f->fmt.pix.field != V4L2_FIELD_NONE)
		/* Interlaced formats are not supported. */
		f->fmt.pix.field = V4L2_FIELD_NONE;

	ti_csi2rx_fill_fmt(fmt, f);

	return 0;
}

static int ti_csi2rx_s_fmt_vid_cap(struct file *file, void *priv,
				   struct v4l2_format *f)
{
	struct ti_csi2rx_ctx *ctx = video_drvdata(file);
	struct vb2_queue *q = &ctx->vidq;
	int ret;

	if (vb2_is_busy(q))
		return -EBUSY;

	ret = ti_csi2rx_try_fmt_vid_cap(file, priv, f);
	if (ret < 0)
		return ret;

	ctx->v_fmt = *f;

	return 0;
}

static int ti_csi2rx_enum_framesizes(struct file *file, void *fh,
				     struct v4l2_frmsizeenum *fsize)
{
	const struct ti_csi2rx_fmt *fmt;
	unsigned int pixels_in_word;
	u8 bpp;

	fmt = find_format_by_pix(fsize->pixel_format);
	if (!fmt)
		return -EINVAL;

	bpp = ALIGN(fmt->bpp, 8);

	/*
	 * Number of pixels in one PSI-L word. The transfer happens in multiples
	 * of PSI-L word sizes.
	 */
	pixels_in_word = PSIL_WORD_SIZE_BYTES * 8 / bpp;

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise.min_width = pixels_in_word;
	fsize->stepwise.max_width = rounddown(MAX_WIDTH_BYTES * 8 / bpp,
					      pixels_in_word);
	fsize->stepwise.step_width = pixels_in_word;
	fsize->stepwise.min_height = 1;
	fsize->stepwise.max_height = MAX_HEIGHT_LINES;
	fsize->stepwise.step_height = 1;

	return 0;
}

static const struct v4l2_ioctl_ops csi_ioctl_ops = {
	.vidioc_querycap      = ti_csi2rx_querycap,
	.vidioc_enum_fmt_vid_cap = ti_csi2rx_enum_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap = ti_csi2rx_try_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap = ti_csi2rx_g_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap = ti_csi2rx_s_fmt_vid_cap,
	.vidioc_enum_framesizes = ti_csi2rx_enum_framesizes,
	.vidioc_reqbufs       = vb2_ioctl_reqbufs,
	.vidioc_create_bufs   = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf   = vb2_ioctl_prepare_buf,
	.vidioc_querybuf      = vb2_ioctl_querybuf,
	.vidioc_qbuf          = vb2_ioctl_qbuf,
	.vidioc_dqbuf         = vb2_ioctl_dqbuf,
	.vidioc_expbuf        = vb2_ioctl_expbuf,
	.vidioc_streamon      = vb2_ioctl_streamon,
	.vidioc_streamoff     = vb2_ioctl_streamoff,
};

static const struct v4l2_file_operations csi_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.read = vb2_fop_read,
	.poll = vb2_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = vb2_fop_mmap,
};

static inline int ti_csi2rx_video_register(struct ti_csi2rx_ctx *ctx)
{
	struct ti_csi2rx_dev *csi = ctx->csi;
	struct video_device *vdev = &ctx->vdev;
	int ret;

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		return ret;

	ret = v4l2_create_fwnode_links_to_pad(csi->subdev, &csi->pad,
					      MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED);

	if (ret) {
		video_unregister_device(vdev);
		return ret;
	}

	return 0;
}

static int csi_async_notifier_bound(struct v4l2_async_notifier *notifier,
				    struct v4l2_subdev *subdev,
				    struct v4l2_async_subdev *asd)
{
	struct ti_csi2rx_dev *csi = dev_get_drvdata(notifier->v4l2_dev->dev);

	csi->subdev = subdev;

	return 0;
}

static int csi_async_notifier_complete(struct v4l2_async_notifier *notifier)
{
	struct ti_csi2rx_dev *csi = dev_get_drvdata(notifier->v4l2_dev->dev);
	int ret, i;

	for (i = 0; i < TI_CSI2RX_NUM_CTX; i++) {
		ret = ti_csi2rx_video_register(&csi->ctx[i]);
		if (ret)
			return ret;
	}

	return v4l2_device_register_subdev_nodes(&csi->v4l2_dev);
}

static const struct v4l2_async_notifier_operations csi_async_notifier_ops = {
	.bound = csi_async_notifier_bound,
	.complete = csi_async_notifier_complete,
};

static int ti_csi2rx_init_subdev(struct ti_csi2rx_dev *csi)
{
	struct fwnode_handle *fwnode;
	struct v4l2_async_subdev *asd;
	int ret;

	fwnode = fwnode_get_named_child_node(csi->dev->fwnode, "csi-bridge");
	if (!fwnode)
		return -EINVAL;

	v4l2_async_nf_init(&csi->notifier);
	csi->notifier.ops = &csi_async_notifier_ops;

	asd = v4l2_async_nf_add_fwnode(&csi->notifier, fwnode,
				       struct v4l2_async_subdev);
	fwnode_handle_put(fwnode);
	if (IS_ERR(asd)) {
		v4l2_async_nf_cleanup(&csi->notifier);
		return PTR_ERR(asd);
	}

	ret = v4l2_async_nf_register(&csi->v4l2_dev, &csi->notifier);
	if (ret) {
		v4l2_async_nf_cleanup(&csi->notifier);
		return ret;
	}

	return 0;
}

static void ti_csi2rx_setup_shim(struct ti_csi2rx_ctx *ctx)
{
	struct ti_csi2rx_dev *csi = ctx->csi;
	const struct ti_csi2rx_fmt *fmt;
	unsigned int reg;

	fmt = find_format_by_pix(ctx->v_fmt.fmt.pix.pixelformat);
	if (!fmt) {
		dev_err(csi->dev, "Unknown format\n");
		return;
	}

	/* De-assert the pixel interface reset. */
	reg = SHIM_CNTL_PIX_RST;
	writel(reg, csi->shim + SHIM_CNTL);

	reg = SHIM_DMACNTX_EN;
	reg |= FIELD_PREP(SHIM_DMACNTX_FMT, fmt->csi_dt);

	/*
	 * Using the values from the documentation gives incorrect ordering for
	 * the luma and chroma components. In practice, the "reverse" format
	 * gives the correct image. So for example, if the image is in UYVY, the
	 * reverse would be YVYU.
	 */
	switch (fmt->fourcc) {
	case V4L2_PIX_FMT_UYVY:
		reg |= FIELD_PREP(SHIM_DMACNTX_YUV422,
					SHIM_DMACNTX_YVYU);
		break;
	case V4L2_PIX_FMT_VYUY:
		reg |= FIELD_PREP(SHIM_DMACNTX_YUV422,
					SHIM_DMACNTX_YUYV);
		break;
	case V4L2_PIX_FMT_YUYV:
		reg |= FIELD_PREP(SHIM_DMACNTX_YUV422,
					SHIM_DMACNTX_VYUY);
		break;
	case V4L2_PIX_FMT_YVYU:
		reg |= FIELD_PREP(SHIM_DMACNTX_YUV422,
					SHIM_DMACNTX_UYVY);
		break;
	default:
		/* Ignore if not YUV 4:2:2 */
		break;
	}

	writel(reg, csi->shim + SHIM_DMACNTX);

	reg = FIELD_PREP(SHIM_PSI_CFG0_SRC_TAG, 0) |
	      FIELD_PREP(SHIM_PSI_CFG0_DST_TAG, 0);
	writel(reg, csi->shim + SHIM_PSI_CFG0);
}

static void ti_csi2rx_dma_callback(void *param)
{
	struct ti_csi2rx_buffer *buf = param;
	struct ti_csi2rx_ctx *ctx = buf->ctx;
	struct ti_csi2rx_dma *dma = &ctx->dma;
	unsigned long flags = 0;

	/*
	 * TODO: Derive the sequence number from the CSI2RX frame number
	 * hardware monitor registers.
	 */
	buf->vb.vb2_buf.timestamp = ktime_get_ns();
	buf->vb.sequence = ctx->sequence++;

	spin_lock_irqsave(&dma->lock, flags);

	WARN_ON(dma->curr != buf);
	vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);

	/* If there are more buffers to process then start their transfer. */
	dma->curr = NULL;
	while (!list_empty(&dma->queue)) {
		buf = list_entry(dma->queue.next, struct ti_csi2rx_buffer, list);
		list_del(&buf->list);

		if (ti_csi2rx_start_dma(ctx, buf)) {
			dev_err(ctx->csi->dev,
				"Failed to queue the next buffer for DMA\n");
			vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
		} else {
			dma->curr = buf;
			break;
		}
	}

	if (!dma->curr)
		dma->state = TI_CSI2RX_DMA_IDLE;

	spin_unlock_irqrestore(&dma->lock, flags);
}

static int ti_csi2rx_start_dma(struct ti_csi2rx_ctx *ctx,
			       struct ti_csi2rx_buffer *buf)
{
	unsigned long addr;
	struct dma_async_tx_descriptor *desc;
	size_t len = ctx->v_fmt.fmt.pix.sizeimage;
	dma_cookie_t cookie;
	int ret = 0;

	addr = vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 0);
	desc = dmaengine_prep_slave_single(ctx->dma.chan, addr, len,
					   DMA_DEV_TO_MEM,
					   DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc)
		return -EIO;

	desc->callback = ti_csi2rx_dma_callback;
	desc->callback_param = buf;

	cookie = dmaengine_submit(desc);
	ret = dma_submit_error(cookie);
	if (ret)
		return ret;

	dma_async_issue_pending(ctx->dma.chan);

	return 0;
}

static void ti_csi2rx_cleanup_buffers(struct ti_csi2rx_ctx *ctx,
				      enum vb2_buffer_state state)
{
	struct ti_csi2rx_dma *dma = &ctx->dma;
	struct ti_csi2rx_buffer *buf = NULL, *tmp;
	unsigned long flags;

	spin_lock_irqsave(&dma->lock, flags);
	list_for_each_entry_safe(buf, tmp, &ctx->dma.queue, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}

	if (dma->curr)
		vb2_buffer_done(&dma->curr->vb.vb2_buf, state);

	dma->curr = NULL;
	dma->state = TI_CSI2RX_DMA_STOPPED;
	spin_unlock_irqrestore(&dma->lock, flags);
}

static int ti_csi2rx_queue_setup(struct vb2_queue *q, unsigned int *nbuffers,
				 unsigned int *nplanes, unsigned int sizes[],
				 struct device *alloc_devs[])
{
	struct ti_csi2rx_ctx *ctx = vb2_get_drv_priv(q);
	unsigned int size = ctx->v_fmt.fmt.pix.sizeimage;

	if (*nplanes) {
		if (sizes[0] < size)
			return -EINVAL;
		size = sizes[0];
	}

	*nplanes = 1;
	sizes[0] = size;

	return 0;
}

static int ti_csi2rx_buffer_prepare(struct vb2_buffer *vb)
{
	struct ti_csi2rx_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	unsigned long size = ctx->v_fmt.fmt.pix.sizeimage;

	if (vb2_plane_size(vb, 0) < size) {
		dev_err(ctx->csi->dev, "Data will not fit into plane\n");
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, size);
	return 0;
}

static void ti_csi2rx_buffer_queue(struct vb2_buffer *vb)
{
	struct ti_csi2rx_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct ti_csi2rx_buffer *buf;
	struct ti_csi2rx_dma *dma = &ctx->dma;
	unsigned long flags = 0;
	int ret;

	buf = container_of(vb, struct ti_csi2rx_buffer, vb.vb2_buf);
	buf->ctx = ctx;

	spin_lock_irqsave(&dma->lock, flags);
	/*
	 * Usually the DMA callback takes care of queueing the pending buffers.
	 * But if DMA has stalled due to lack of buffers, restart it now.
	 */
	if (dma->state == TI_CSI2RX_DMA_IDLE) {
		ret = ti_csi2rx_start_dma(ctx, buf);
		if (ret) {
			dev_err(ctx->csi->dev, "Failed to start DMA: %d\n", ret);
			vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_QUEUED);
			goto unlock;
		}

		dma->curr = buf;
		dma->state = TI_CSI2RX_DMA_ACTIVE;
	} else {
		list_add_tail(&buf->list, &dma->queue);
	}

unlock:
	spin_unlock_irqrestore(&dma->lock, flags);
}

static int ti_csi2rx_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct ti_csi2rx_ctx *ctx = vb2_get_drv_priv(vq);
	struct ti_csi2rx_dev *csi = ctx->csi;
	struct ti_csi2rx_dma *dma = &ctx->dma;
	struct ti_csi2rx_buffer *buf;
	unsigned long flags = 0;
	int ret = 0;

	spin_lock_irqsave(&dma->lock, flags);
	if (list_empty(&dma->queue))
		ret = -EIO;
	spin_unlock_irqrestore(&dma->lock, flags);
	if (ret)
		return ret;

	ret = video_device_pipeline_start(&ctx->vdev, &csi->pipe);
	if (ret)
		goto err;

	ti_csi2rx_setup_shim(ctx);

	ctx->sequence = 0;

	spin_lock_irqsave(&dma->lock, flags);
	buf = list_entry(dma->queue.next, struct ti_csi2rx_buffer, list);
	list_del(&buf->list);
	dma->state = TI_CSI2RX_DMA_ACTIVE;
	dma->curr = buf;

	ret = ti_csi2rx_start_dma(ctx, buf);
	if (ret) {
		dev_err(csi->dev, "Failed to start DMA: %d\n", ret);
		spin_unlock_irqrestore(&dma->lock, flags);
		goto err_pipeline;
	}

	spin_unlock_irqrestore(&dma->lock, flags);

	ret = v4l2_subdev_call(csi->subdev, video, s_stream, 1);
	if (ret)
		goto err_dma;

	return 0;

err_dma:
	dmaengine_terminate_sync(ctx->dma.chan);
	writel(0, csi->shim + SHIM_DMACNTX);
err_pipeline:
	video_device_pipeline_stop(&ctx->vdev);
err:
	ti_csi2rx_cleanup_buffers(ctx, VB2_BUF_STATE_QUEUED);
	return ret;
}

static void ti_csi2rx_stop_streaming(struct vb2_queue *vq)
{
	struct ti_csi2rx_ctx *ctx = vb2_get_drv_priv(vq);
	struct ti_csi2rx_dev *csi = ctx->csi;
	int ret;

	video_device_pipeline_stop(&ctx->vdev);

	ret = v4l2_subdev_call(csi->subdev, video, s_stream, 0);
	if (ret)
		dev_err(csi->dev, "Failed to stop subdev stream\n");

	writel(0, csi->shim + SHIM_CNTL);

	ret = dmaengine_terminate_sync(ctx->dma.chan);
	if (ret)
		dev_err(csi->dev, "Failed to stop DMA: %d\n", ret);

	writel(0, csi->shim + SHIM_DMACNTX);

	ti_csi2rx_cleanup_buffers(ctx, VB2_BUF_STATE_ERROR);
}

static const struct vb2_ops csi_vb2_qops = {
	.queue_setup = ti_csi2rx_queue_setup,
	.buf_prepare = ti_csi2rx_buffer_prepare,
	.buf_queue = ti_csi2rx_buffer_queue,
	.start_streaming = ti_csi2rx_start_streaming,
	.stop_streaming = ti_csi2rx_stop_streaming,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

static void ti_csi2rx_cleanup_dma(struct ti_csi2rx_ctx *ctx)
{
	dma_release_channel(ctx->dma.chan);
}

static void ti_csi2rx_cleanup_v4l2(struct ti_csi2rx_dev *csi)
{
	media_device_unregister(&csi->mdev);
	v4l2_device_unregister(&csi->v4l2_dev);
	media_device_cleanup(&csi->mdev);
}

static void ti_csi2rx_cleanup_subdev(struct ti_csi2rx_dev *csi)
{
	v4l2_async_nf_unregister(&csi->notifier);
	v4l2_async_nf_cleanup(&csi->notifier);
}

static void ti_csi2rx_cleanup_vb2q(struct ti_csi2rx_ctx *ctx)
{
	vb2_queue_release(&ctx->vidq);
}

static void ti_csi2rx_cleanup_ctx(struct ti_csi2rx_ctx *ctx)
{
	ti_csi2rx_cleanup_dma(ctx);
	ti_csi2rx_cleanup_vb2q(ctx);

	video_unregister_device(&ctx->vdev);

	mutex_destroy(&ctx->mutex);
}

static int ti_csi2rx_init_vb2q(struct ti_csi2rx_ctx *ctx)
{
	struct vb2_queue *q = &ctx->vidq;
	int ret;

	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_DMABUF;
	q->drv_priv = ctx;
	q->buf_struct_size = sizeof(struct ti_csi2rx_buffer);
	q->ops = &csi_vb2_qops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->dev = dmaengine_get_dma_device(ctx->dma.chan);
	q->lock = &ctx->mutex;
	q->min_buffers_needed = 1;

	ret = vb2_queue_init(q);
	if (ret)
		return ret;

	ctx->vdev.queue = q;

	return 0;
}

static int ti_csi2rx_link_validate_get_fmt(struct media_pad *pad,
					   struct v4l2_subdev_format *fmt)
{
	if (is_media_entity_v4l2_subdev(pad->entity)) {
		struct v4l2_subdev *sd =
			media_entity_to_v4l2_subdev(pad->entity);

		fmt->which = V4L2_SUBDEV_FORMAT_ACTIVE;
		fmt->pad = pad->index;
		return v4l2_subdev_call(sd, pad, get_fmt, NULL, fmt);
	}

	WARN(pad->entity->function != MEDIA_ENT_F_IO_V4L,
	     "Driver bug! Wrong media entity type 0x%08x, entity %s\n",
	     pad->entity->function, pad->entity->name);

	return -EINVAL;
}

static int ti_csi2rx_link_validate(struct media_link *link)
{
	struct media_entity *entity = link->sink->entity;
	struct video_device *vdev = media_entity_to_video_device(entity);
	struct ti_csi2rx_ctx *ctx = container_of(vdev, struct ti_csi2rx_ctx, vdev);
	struct ti_csi2rx_dev *csi = ctx->csi;
	struct v4l2_pix_format *csi_fmt = &ctx->v_fmt.fmt.pix;
	struct v4l2_subdev_format source_fmt;
	const struct ti_csi2rx_fmt *ti_fmt;
	int ret;

	ret = ti_csi2rx_link_validate_get_fmt(link->source, &source_fmt);
	if (ret)
		return ret;

	if (source_fmt.format.width != csi_fmt->width) {
		dev_dbg(csi->dev, "Width does not match (source %u, sink %u)\n",
			source_fmt.format.width, csi_fmt->width);
		return -EPIPE;
	}

	if (source_fmt.format.height != csi_fmt->height) {
		dev_dbg(csi->dev, "Height does not match (source %u, sink %u)\n",
			source_fmt.format.height, csi_fmt->height);
		return -EPIPE;
	}

	if (source_fmt.format.field != csi_fmt->field &&
	    csi_fmt->field != V4L2_FIELD_NONE) {
		dev_dbg(csi->dev, "Field does not match (source %u, sink %u)\n",
			source_fmt.format.field, csi_fmt->field);
		return -EPIPE;
	}

	ti_fmt = find_format_by_code(source_fmt.format.code);
	if (!ti_fmt) {
		dev_dbg(csi->dev, "Media bus format 0x%x not supported\n",
			source_fmt.format.code);
		return -EPIPE;
	}

	if (ctx->v_fmt.fmt.pix.pixelformat != ti_fmt->fourcc) {
		dev_dbg(csi->dev,
			"Cannot transform source fmt 0x%x to sink fmt 0x%x\n",
			ctx->v_fmt.fmt.pix.pixelformat, ti_fmt->fourcc);
		return -EPIPE;
	}

	return 0;
}

static const struct media_entity_operations ti_csi2rx_video_entity_ops = {
	.link_validate = ti_csi2rx_link_validate,
};

static int ti_csi2rx_init_dma(struct ti_csi2rx_ctx *ctx)
{
	struct dma_slave_config cfg = {
		.src_addr_width = DMA_SLAVE_BUSWIDTH_16_BYTES };
	int ret;

	INIT_LIST_HEAD(&ctx->dma.queue);
	spin_lock_init(&ctx->dma.lock);

	ctx->dma.state = TI_CSI2RX_DMA_STOPPED;

	ctx->dma.chan = dma_request_chan(ctx->csi->dev, "rx0");
	if (IS_ERR(ctx->dma.chan))
		return PTR_ERR(ctx->dma.chan);

	ret = dmaengine_slave_config(ctx->dma.chan, &cfg);
	if (ret) {
		dma_release_channel(ctx->dma.chan);
		return ret;
	}

	return 0;
}

static int ti_csi2rx_v4l2_init(struct ti_csi2rx_dev *csi)
{
	struct media_device *mdev = &csi->mdev;
	int ret;

	mdev->dev = csi->dev;
	mdev->hw_revision = 1;
	strscpy(mdev->model, "TI-CSI2RX", sizeof(mdev->model));

	media_device_init(mdev);

	csi->v4l2_dev.mdev = mdev;

	ret = v4l2_device_register(csi->dev, &csi->v4l2_dev);
	if (ret)
		return ret;

	ret = media_device_register(mdev);
	if (ret) {
		v4l2_device_unregister(&csi->v4l2_dev);
		media_device_cleanup(mdev);
		return ret;
	}

	return 0;
}

static int ti_csi2rx_init_ctx(struct ti_csi2rx_ctx *ctx)
{
	struct ti_csi2rx_dev *csi = ctx->csi;
	struct video_device *vdev = &ctx->vdev;
	const struct ti_csi2rx_fmt *fmt;
	struct v4l2_pix_format *pix_fmt = &ctx->v_fmt.fmt.pix;
	int ret;

	mutex_init(&ctx->mutex);

	fmt = find_format_by_pix(V4L2_PIX_FMT_UYVY);
	if (!fmt)
		return -EINVAL;

	pix_fmt->width = 640;
	pix_fmt->height = 480;

	ti_csi2rx_fill_fmt(fmt, &ctx->v_fmt);

	csi->pad.flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&ctx->vdev.entity, 1, &csi->pad);
	if (ret)
		return ret;

	snprintf(vdev->name, sizeof(vdev->name), "%s context %u",
		 dev_name(csi->dev), ctx->idx);
	vdev->v4l2_dev = &csi->v4l2_dev;
	vdev->vfl_dir = VFL_DIR_RX;
	vdev->fops = &csi_fops;
	vdev->ioctl_ops = &csi_ioctl_ops;
	vdev->release = video_device_release_empty;
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING |
			    V4L2_CAP_IO_MC;
	vdev->lock = &ctx->mutex;
	video_set_drvdata(vdev, ctx);

	ret = ti_csi2rx_init_dma(ctx);
	if (ret)
		return ret;

	ret = ti_csi2rx_init_vb2q(ctx);
	if (ret)
		goto cleanup_dma;

	return 0;

cleanup_dma:
	ti_csi2rx_cleanup_dma(ctx);
	return ret;
}

static int ti_csi2rx_probe(struct platform_device *pdev)
{
	struct ti_csi2rx_dev *csi;
	struct resource *res;
	int ret, i;

	csi = devm_kzalloc(&pdev->dev, sizeof(*csi), GFP_KERNEL);
	if (!csi)
		return -ENOMEM;

	csi->dev = &pdev->dev;
	platform_set_drvdata(pdev, csi);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	csi->shim = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(csi->shim)) {
		ret = PTR_ERR(csi->shim);
		return ret;
	}

	ret = ti_csi2rx_v4l2_init(csi);
	if (ret)
		return ret;

	for (i = 0; i < TI_CSI2RX_NUM_CTX; i++) {
		csi->ctx[i].idx = i;
		csi->ctx[i].csi = csi;
		ret = ti_csi2rx_init_ctx(&csi->ctx[i]);
		if (ret)
			goto cleanup_ctx;
	}

	ret = ti_csi2rx_init_subdev(csi);
	if (ret)
		goto cleanup_ctx;

	ret = of_platform_populate(csi->dev->of_node, NULL, NULL, csi->dev);
	if (ret) {
		dev_err(csi->dev, "Failed to create children: %d\n", ret);
		goto cleanup_subdev;
	}

	return 0;

cleanup_subdev:
	ti_csi2rx_cleanup_subdev(csi);
cleanup_ctx:

	i--;
	for (; i >= 0; i--)
		ti_csi2rx_cleanup_ctx(&csi->ctx[i]);

	ti_csi2rx_cleanup_v4l2(csi);
	return ret;
}

static int ti_csi2rx_remove(struct platform_device *pdev)
{
	struct ti_csi2rx_dev *csi = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < TI_CSI2RX_NUM_CTX; i++) {
		if (vb2_is_busy(&csi->ctx[i].vidq))
			return -EBUSY;
	}

	for (i = 0; i < TI_CSI2RX_NUM_CTX; i++)
		ti_csi2rx_cleanup_ctx(&csi->ctx[i]);

	ti_csi2rx_cleanup_subdev(csi);
	ti_csi2rx_cleanup_v4l2(csi);

	return 0;
}

static const struct of_device_id ti_csi2rx_of_match[] = {
	{ .compatible = "ti,j721e-csi2rx", },
	{ },
};
MODULE_DEVICE_TABLE(of, ti_csi2rx_of_match);

static struct platform_driver ti_csi2rx_pdrv = {
	.probe = ti_csi2rx_probe,
	.remove = ti_csi2rx_remove,
	.driver = {
		.name = TI_CSI2RX_MODULE_NAME,
		.of_match_table = ti_csi2rx_of_match,
	},
};

module_platform_driver(ti_csi2rx_pdrv);

MODULE_DESCRIPTION("TI J721E CSI2 RX Driver");
MODULE_AUTHOR("Pratyush Yadav <p.yadav@ti.com>");
MODULE_LICENSE("GPL");
