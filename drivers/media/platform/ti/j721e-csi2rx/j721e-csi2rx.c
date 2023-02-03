// SPDX-License-Identifier: GPL-2.0-only
/*
 * TI CSI2 RX driver.
 *
 * Copyright (C) 2021 Texas Instruments Incorporated - https://www.ti.com/
 *
 * Author: Pratyush Yadav <p.yadav@ti.com>
 */

#include <linux/bitfield.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/dmaengine.h>
#include <linux/of_platform.h>
#include <linux/pm.h>

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>

#define TI_CSI2RX_MODULE_NAME		"j721e-csi2rx"

#define SHIM_CNTL			0x10
#define SHIM_CNTL_PIX_RST		BIT(0)

#define SHIM_DMACNTX(i)			(0x20 + ((i) * 0x20))
#define SHIM_DMACNTX_EN			BIT(31)
#define SHIM_DMACNTX_YUV422		GENMASK(27, 26)
#define SHIM_DMACNTX_SIZE		GENMASK(21, 20)
#define SHIM_DMACNTX_VC			GENMASK(9, 6)
#define SHIM_DMACNTX_FMT		GENMASK(5, 0)
#define SHIM_DMACNTX_UYVY		0
#define SHIM_DMACNTX_VYUY		1
#define SHIM_DMACNTX_YUYV		2
#define SHIM_DMACNTX_YVYU		3
#define SHIM_DMACNTX_SIZE_8		0
#define SHIM_DMACNTX_SIZE_16		1
#define SHIM_DMACNTX_SIZE_32		2

#define SHIM_PSI_CFG0(i)		(0x24 + ((i) * 0x20))
#define SHIM_PSI_CFG0_SRC_TAG		GENMASK(15, 0)
#define SHIM_PSI_CFG0_DST_TAG		GENMASK(31, 16)

#define CSI_DF_YUV420			0x18
#define CSI_DF_YUV422			0x1e
#define CSI_DF_RGB444			0x20
#define CSI_DF_RGB888			0x24
#define CSI_DF_RAW8			0x2a
#define CSI_DF_RAW10			0x2b
#define CSI_DF_RAW12			0x2c

#define PSIL_WORD_SIZE_BYTES		16
#define TI_CSI2RX_MAX_CTX		32

/*
 * There are no hard limits on the width or height. The DMA engine can handle
 * all sizes. The max width and height are arbitrary numbers for this driver.
 * Use 16M * 16M as the arbitrary limit. It is large enough that it is unlikely
 * the limit will be hit in practice.
 */
#define MAX_WIDTH_BYTES			SZ_16M
#define MAX_HEIGHT_BYTES		SZ_16M

#define TI_CSI2RX_PAD_SINK		0
#define TI_CSI2RX_PAD_FIRST_SOURCE	1
#define TI_CSI2RX_MAX_SOURCE_PADS	TI_CSI2RX_MAX_CTX
#define TI_CSI2RX_MAX_PADS		(1 + TI_CSI2RX_MAX_SOURCE_PADS)

#define DRAIN_TIMEOUT_MS		50

struct ti_csi2rx_fmt {
	u32				fourcc;	/* Four character code. */
	u32				code;	/* Mbus code. */
	enum v4l2_colorspace		colorspace;
	u32				csi_df;	/* CSI Data format. */
	u8				bpp;	/* Bits per pixel. */
	u8				size;	/* Data size shift when unpacking. */
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
	struct media_pad		pad;
	u32				sequence;
	u32				idx;
	u32				vc;
	u32				stream;
};

struct ti_csi2rx_dev {
	struct device			*dev;
	void __iomem			*shim;
	/* To serialize core subdev ioctls. */
	struct mutex			mutex;
	unsigned int			enable_count;
	unsigned int			num_ctx;
	struct v4l2_async_notifier	notifier;
	struct media_device		mdev;
	struct media_pipeline		pipe;
	struct media_pad		pads[TI_CSI2RX_MAX_PADS];
	struct v4l2_device		v4l2_dev;
	struct v4l2_subdev		*source;
	struct v4l2_subdev		subdev;
	struct ti_csi2rx_ctx		ctx[TI_CSI2RX_MAX_CTX];
};

static const struct ti_csi2rx_fmt formats[] = {
	{
		.fourcc			= V4L2_PIX_FMT_YUYV,
		.code			= MEDIA_BUS_FMT_YUYV8_1X16,
		.colorspace		= V4L2_COLORSPACE_SRGB,
		.csi_df			= CSI_DF_YUV422,
		.bpp			= 16,
		.size			= SHIM_DMACNTX_SIZE_8,
	}, {
		.fourcc			= V4L2_PIX_FMT_UYVY,
		.code			= MEDIA_BUS_FMT_UYVY8_1X16,
		.colorspace		= V4L2_COLORSPACE_SRGB,
		.csi_df			= CSI_DF_YUV422,
		.bpp			= 16,
		.size			= SHIM_DMACNTX_SIZE_8,
	}, {
		.fourcc			= V4L2_PIX_FMT_YVYU,
		.code			= MEDIA_BUS_FMT_YVYU8_1X16,
		.colorspace		= V4L2_COLORSPACE_SRGB,
		.csi_df			= CSI_DF_YUV422,
		.bpp			= 16,
		.size			= SHIM_DMACNTX_SIZE_8,
	}, {
		.fourcc			= V4L2_PIX_FMT_VYUY,
		.code			= MEDIA_BUS_FMT_VYUY8_1X16,
		.colorspace		= V4L2_COLORSPACE_SRGB,
		.csi_df			= CSI_DF_YUV422,
		.bpp			= 16,
		.size			= SHIM_DMACNTX_SIZE_8,
	}, {
		.fourcc			= V4L2_PIX_FMT_SBGGR8,
		.code			= MEDIA_BUS_FMT_SBGGR8_1X8,
		.colorspace		= V4L2_COLORSPACE_SRGB,
		.csi_df			= CSI_DF_RAW8,
		.bpp			= 8,
		.size			= SHIM_DMACNTX_SIZE_8,
	}, {
		.fourcc			= V4L2_PIX_FMT_SGBRG8,
		.code			= MEDIA_BUS_FMT_SGBRG8_1X8,
		.colorspace		= V4L2_COLORSPACE_SRGB,
		.csi_df			= CSI_DF_RAW8,
		.bpp			= 8,
		.size			= SHIM_DMACNTX_SIZE_8,
	}, {
		.fourcc			= V4L2_PIX_FMT_SGRBG8,
		.code			= MEDIA_BUS_FMT_SGRBG8_1X8,
		.colorspace		= V4L2_COLORSPACE_SRGB,
		.csi_df			= CSI_DF_RAW8,
		.bpp			= 8,
		.size			= SHIM_DMACNTX_SIZE_8,
	}, {
		.fourcc			= V4L2_PIX_FMT_SRGGB8,
		.code			= MEDIA_BUS_FMT_SRGGB8_1X8,
		.colorspace		= V4L2_COLORSPACE_SRGB,
		.csi_df			= CSI_DF_RAW8,
		.bpp			= 8,
		.size			= SHIM_DMACNTX_SIZE_8,
	}, {
		.fourcc			= V4L2_PIX_FMT_SBGGR10,
		.code			= MEDIA_BUS_FMT_SBGGR10_1X10,
		.colorspace		= V4L2_COLORSPACE_SRGB,
		.csi_df			= CSI_DF_RAW10,
		.bpp			= 16,
		.size			= SHIM_DMACNTX_SIZE_16,
	}, {
		.fourcc			= V4L2_PIX_FMT_SGBRG10,
		.code			= MEDIA_BUS_FMT_SGBRG10_1X10,
		.colorspace		= V4L2_COLORSPACE_SRGB,
		.csi_df			= CSI_DF_RAW10,
		.bpp			= 16,
		.size			= SHIM_DMACNTX_SIZE_16,
	}, {
		.fourcc			= V4L2_PIX_FMT_SGRBG10,
		.code			= MEDIA_BUS_FMT_SGRBG10_1X10,
		.colorspace		= V4L2_COLORSPACE_SRGB,
		.csi_df			= CSI_DF_RAW10,
		.bpp			= 16,
		.size			= SHIM_DMACNTX_SIZE_16,
	}, {
		.fourcc			= V4L2_PIX_FMT_SRGGB10,
		.code			= MEDIA_BUS_FMT_SRGGB10_1X10,
		.colorspace		= V4L2_COLORSPACE_SRGB,
		.csi_df			= CSI_DF_RAW10,
		.bpp			= 16,
		.size			= SHIM_DMACNTX_SIZE_16,
	}, {
		.fourcc			= V4L2_PIX_FMT_SBGGR12,
		.code			= MEDIA_BUS_FMT_SBGGR12_1X12,
		.colorspace		= V4L2_COLORSPACE_SRGB,
		.csi_df			= CSI_DF_RAW12,
		.bpp			= 16,
		.size			= SHIM_DMACNTX_SIZE_16,
	}, {
		.fourcc			= V4L2_PIX_FMT_SGBRG12,
		.code			= MEDIA_BUS_FMT_SGBRG12_1X12,
		.colorspace		= V4L2_COLORSPACE_SRGB,
		.csi_df			= CSI_DF_RAW12,
		.bpp			= 16,
		.size			= SHIM_DMACNTX_SIZE_16,
	}, {
		.fourcc			= V4L2_PIX_FMT_SGRBG12,
		.code			= MEDIA_BUS_FMT_SGRBG12_1X12,
		.colorspace		= V4L2_COLORSPACE_SRGB,
		.csi_df			= CSI_DF_RAW12,
		.bpp			= 16,
		.size			= SHIM_DMACNTX_SIZE_16,
	}, {
		.fourcc			= V4L2_PIX_FMT_SRGGB12,
		.code			= MEDIA_BUS_FMT_SRGGB12_1X12,
		.colorspace		= V4L2_COLORSPACE_SRGB,
		.csi_df			= CSI_DF_RAW12,
		.bpp			= 16,
		.size			= SHIM_DMACNTX_SIZE_16,
	}, {
		.fourcc			= V4L2_PIX_FMT_SRGGI10,
		.code			= MEDIA_BUS_FMT_SRGGI10_1X10,
		.colorspace		= V4L2_COLORSPACE_SRGB,
		.csi_df			= CSI_DF_RAW10,
		.bpp			= 16,
		.size			= SHIM_DMACNTX_SIZE_16,
	}, {
		.fourcc			= V4L2_PIX_FMT_SGRIG10,
		.code			= MEDIA_BUS_FMT_SGRIG10_1X10,
		.colorspace		= V4L2_COLORSPACE_SRGB,
		.csi_df			= CSI_DF_RAW10,
		.bpp			= 16,
		.size			= SHIM_DMACNTX_SIZE_16,
	}, {
		.fourcc			= V4L2_PIX_FMT_SBGGI10,
		.code			= MEDIA_BUS_FMT_SBGGI10_1X10,
		.colorspace		= V4L2_COLORSPACE_SRGB,
		.csi_df			= CSI_DF_RAW10,
		.bpp			= 16,
		.size			= SHIM_DMACNTX_SIZE_16,
	}, {
		.fourcc			= V4L2_PIX_FMT_SGBIG10,
		.code			= MEDIA_BUS_FMT_SGBIG10_1X10,
		.colorspace		= V4L2_COLORSPACE_SRGB,
		.csi_df			= CSI_DF_RAW10,
		.bpp			= 16,
		.size			= SHIM_DMACNTX_SIZE_16,
	}, {
		.fourcc			= V4L2_PIX_FMT_SGIRG10,
		.code			= MEDIA_BUS_FMT_SGRIG10_1X10,
		.colorspace		= V4L2_COLORSPACE_SRGB,
		.csi_df			= CSI_DF_RAW10,
		.bpp			= 16,
		.size			= SHIM_DMACNTX_SIZE_16,
	}, {
		.fourcc			= V4L2_PIX_FMT_SIGGR10,
		.code			= MEDIA_BUS_FMT_SIGGR10_1X10,
		.colorspace		= V4L2_COLORSPACE_SRGB,
		.csi_df			= CSI_DF_RAW10,
		.bpp			= 16,
		.size			= SHIM_DMACNTX_SIZE_16,
	}, {
		.fourcc			= V4L2_PIX_FMT_SGIBG10,
		.code			= MEDIA_BUS_FMT_SGIBG10_1X10,
		.colorspace		= V4L2_COLORSPACE_SRGB,
		.csi_df			= CSI_DF_RAW10,
		.bpp			= 16,
		.size			= SHIM_DMACNTX_SIZE_16,
	}, {
		.fourcc			= V4L2_PIX_FMT_SIGGB10,
		.code			= MEDIA_BUS_FMT_SIGGB10_1X10,
		.colorspace		= V4L2_COLORSPACE_SRGB,
		.csi_df			= CSI_DF_RAW10,
		.bpp			= 16,
		.size			= SHIM_DMACNTX_SIZE_16,
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
	u32 bpl;

	v4l2_fmt->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	pix->pixelformat = csi_fmt->fourcc;
	pix->colorspace = csi_fmt->colorspace;
	pix->sizeimage = pix->height * pix->width * (csi_fmt->bpp / 8);

	bpl = (pix->width * ALIGN(csi_fmt->bpp, 8)) >> 3;
	pix->bytesperline = ALIGN(bpl, 16);
}

static int ti_csi2rx_querycap(struct file *file, void *priv,
			      struct v4l2_capability *cap)
{
	struct ti_csi2rx_ctx *ctx = video_drvdata(file);

	strscpy(cap->driver, TI_CSI2RX_MODULE_NAME, sizeof(cap->driver));
	strscpy(cap->card, TI_CSI2RX_MODULE_NAME, sizeof(cap->card));

	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 dev_name(ctx->csi->dev));

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
		return -EINVAL;

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
	if (!fmt || fsize->index != 0)
		return -EINVAL;

	bpp = ALIGN(fmt->bpp, 8);

	/*
	 * Number of pixels in one PSI-L word. The transfer happens in multiples
	 * of PSI-L word sizes.
	 */
	pixels_in_word = PSIL_WORD_SIZE_BYTES * 8 / bpp;

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise.min_width = pixels_in_word;
	fsize->stepwise.max_width = rounddown(MAX_WIDTH_BYTES, pixels_in_word);
	fsize->stepwise.step_width = pixels_in_word;
	fsize->stepwise.min_height = 1;
	fsize->stepwise.max_height = MAX_HEIGHT_BYTES;
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

static int ti_csi2rx_video_register(struct ti_csi2rx_ctx *ctx)
{
	struct ti_csi2rx_dev *csi = ctx->csi;
	struct video_device *vdev = &ctx->vdev;
	int ret;

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		return ret;

	ret = media_create_pad_link(&csi->subdev.entity,
				    TI_CSI2RX_PAD_FIRST_SOURCE + ctx->idx,
				    &vdev->entity, 0,
				    MEDIA_LNK_FL_IMMUTABLE |
				    MEDIA_LNK_FL_ENABLED);
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

	/* Should register only one source. */
	WARN_ON(csi->source);

	csi->source = subdev;

	return 0;
}

static int csi_async_notifier_complete(struct v4l2_async_notifier *notifier)
{
	struct ti_csi2rx_dev *csi = dev_get_drvdata(notifier->v4l2_dev->dev);
	int ret, i, src_pad;

	src_pad = media_entity_get_fwnode_pad(&csi->source->entity,
					      csi->source->fwnode,
					      MEDIA_PAD_FL_SOURCE);
	if (src_pad < 0) {
		dev_err(csi->dev, "Couldn't find source pad for subdev\n");
		return src_pad;
	}

	ret = media_create_pad_link(&csi->source->entity, src_pad,
				    &csi->subdev.entity, TI_CSI2RX_PAD_SINK,
				    MEDIA_LNK_FL_IMMUTABLE |
				    MEDIA_LNK_FL_ENABLED);
	if (ret)
		return ret;

	for (i = 0; i < csi->num_ctx; i++) {
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
	struct device_node *node;
	int ret;

	node = of_get_child_by_name(csi->dev->of_node, "csi-bridge");
	if (!node)
		return -EINVAL;

	fwnode = of_fwnode_handle(node);
	if (!fwnode) {
		of_node_put(node);
		return -EINVAL;
	}

	v4l2_async_notifier_init(&csi->notifier);
	csi->notifier.ops = &csi_async_notifier_ops;

	asd = v4l2_async_notifier_add_fwnode_subdev(&csi->notifier, fwnode,
						    sizeof(struct v4l2_async_subdev));
	of_node_put(node);
	if (IS_ERR(asd)) {
		v4l2_async_notifier_cleanup(&csi->notifier);
		return PTR_ERR(asd);
	}

	ret = v4l2_async_notifier_register(&csi->v4l2_dev, &csi->notifier);
	if (ret) {
		v4l2_async_notifier_cleanup(&csi->notifier);
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

	reg = SHIM_DMACNTX_EN;
	reg |= FIELD_PREP(SHIM_DMACNTX_FMT, fmt->csi_df);

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

	reg |= FIELD_PREP(SHIM_DMACNTX_SIZE, fmt->size);
	reg |= FIELD_PREP(SHIM_DMACNTX_VC, ctx->vc);

	writel(reg, csi->shim + SHIM_DMACNTX(ctx->idx));

	reg = FIELD_PREP(SHIM_PSI_CFG0_SRC_TAG, 0) |
	      FIELD_PREP(SHIM_PSI_CFG0_DST_TAG, 0);
	writel(reg, csi->shim + SHIM_PSI_CFG0(ctx->idx));
}

static void ti_csi2rx_drain_callback(void *param)
{
	struct completion *drain_complete = param;

	complete(drain_complete);
}

static int ti_csi2rx_drain_dma(struct ti_csi2rx_ctx *csi)
{
	struct dma_async_tx_descriptor *desc;
	struct device *dev = csi->dma.chan->device->dev;
	struct completion drain_complete;
	void *buf;
	size_t len = csi->v_fmt.fmt.pix.sizeimage;
	dma_addr_t addr;
	dma_cookie_t cookie;
	int ret;

	init_completion(&drain_complete);

	buf = dma_alloc_coherent(dev, len, &addr, GFP_KERNEL | GFP_ATOMIC);
	if (!buf)
		return -ENOMEM;

	desc = dmaengine_prep_slave_single(csi->dma.chan, addr, len,
					   DMA_DEV_TO_MEM,
					   DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc) {
		ret = -EIO;
		goto out;
	}

	desc->callback = ti_csi2rx_drain_callback;
	desc->callback_param = &drain_complete;

	cookie = dmaengine_submit(desc);
	ret = dma_submit_error(cookie);
	if (ret)
		goto out;

	dma_async_issue_pending(csi->dma.chan);

	if (!wait_for_completion_timeout(&drain_complete,
					 msecs_to_jiffies(DRAIN_TIMEOUT_MS))) {
		dmaengine_terminate_sync(csi->dma.chan);
		ret = -ETIMEDOUT;
		goto out;
	}
out:
	dma_free_coherent(dev, len, buf, addr);
	return ret;
}

static void ti_csi2rx_dma_callback(void *param)
{
	struct ti_csi2rx_buffer *buf = param;
	struct ti_csi2rx_ctx *ctx = buf->ctx;
	struct ti_csi2rx_dma *dma = &ctx->dma;
	unsigned long flags = 0;

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

static int ti_csi2rx_restart_dma(struct ti_csi2rx_ctx *ctx,
				 struct ti_csi2rx_buffer *buf)
{
	struct ti_csi2rx_dma *dma = &ctx->dma;
	unsigned long flags = 0;
	int ret = 0;

	ret = ti_csi2rx_drain_dma(ctx);
	if (ret)
		dev_warn(ctx->csi->dev,
			 "Failed to drain DMA. Next frame might be bogus\n");

	ret = ti_csi2rx_start_dma(ctx, buf);
	if (ret) {
		dev_err(ctx->csi->dev, "Failed to start DMA: %d\n", ret);
		spin_lock_irqsave(&dma->lock, flags);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
		dma->curr = NULL;
		dma->state = TI_CSI2RX_DMA_IDLE;
		spin_unlock_irqrestore(&dma->lock, flags);
	}

	return ret;
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
	bool restart_dma = false;
	unsigned long flags = 0;

	buf = container_of(vb, struct ti_csi2rx_buffer, vb.vb2_buf);
	buf->ctx = ctx;

	spin_lock_irqsave(&dma->lock, flags);
	/*
	 * Usually the DMA callback takes care of queueing the pending buffers.
	 * But if DMA has stalled due to lack of buffers, restart it now.
	 */
	if (dma->state == TI_CSI2RX_DMA_IDLE) {
		/*
		 * Do not restart DMA with the lock held because
		 * ti_csi2rx_drain_dma() might block when allocating a buffer.
		 * There won't be a race on queueing DMA anyway since the
		 * callback is not being fired.
		 */
		restart_dma = true;
		dma->curr = buf;
		dma->state = TI_CSI2RX_DMA_ACTIVE;
	} else {
		list_add_tail(&buf->list, &dma->queue);
	}
	spin_unlock_irqrestore(&dma->lock, flags);

	if (restart_dma) {
		/*
		 * Once frames start dropping, some data gets stuck in the DMA
		 * pipeline somewhere. So the first DMA transfer after frame
		 * drops gives a partial frame. This is obviously not useful to
		 * the application and will only confuse it. Issue a DMA
		 * transaction to drain that up.
		 */
		ti_csi2rx_restart_dma(ctx, buf);
	}
}

static int ti_csi2rx_get_vc(struct ti_csi2rx_ctx *ctx)
{
	struct ti_csi2rx_dev *csi = ctx->csi;
	struct v4l2_mbus_frame_desc fd;
	struct media_pad *pad;
	int ret, i;

	pad = media_entity_remote_pad(&csi->pads[TI_CSI2RX_PAD_SINK]);
	if (!pad)
		return -ENODEV;

	ret = v4l2_subdev_call(csi->source, pad, get_frame_desc, pad->index,
			       &fd);
	if (ret)
		return ret;

	if (fd.type != V4L2_MBUS_FRAME_DESC_TYPE_CSI2)
		return -EINVAL;

	for (i = 0; i < fd.num_entries; i++) {
		if (ctx->stream == fd.entry[i].stream)
			return fd.entry[i].bus.csi2.vc;
	}

	return -ENODEV;
}

static int ti_csi2rx_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct ti_csi2rx_ctx *ctx = vb2_get_drv_priv(vq);
	struct ti_csi2rx_dev *csi = ctx->csi;
	struct ti_csi2rx_dma *dma = &ctx->dma;
	struct ti_csi2rx_buffer *buf, *tmp;
	struct v4l2_subdev_krouting *routing;
	struct v4l2_subdev_route *route = NULL;
	struct media_pad *remote_pad;
	unsigned long flags = 0;
	int ret = 0, i;
	struct v4l2_subdev_state *state;

	spin_lock_irqsave(&dma->lock, flags);
	if (list_empty(&dma->queue))
		ret = -EIO;
	spin_unlock_irqrestore(&dma->lock, flags);
	if (ret)
		return ret;

	ret = media_pipeline_start(ctx->vdev.entity.pads, &csi->pipe);
	if (ret)
		goto err;

	remote_pad = media_entity_remote_pad(&ctx->pad);
	if (!remote_pad) {
		ret = -ENODEV;
		goto err_pipeline;
	}

	state = v4l2_subdev_lock_active_state(&csi->subdev);

	routing = &state->routing;

	/* Find the stream to process. */
	for (i = 0; i < routing->num_routes; i++) {
		struct v4l2_subdev_route *r = &routing->routes[i];

		if (!(r->flags & V4L2_SUBDEV_ROUTE_FL_ACTIVE))
			continue;

		if (r->source_pad != remote_pad->index)
			continue;

		route = r;
		break;
	}

	if (!route) {
		ret = -ENODEV;
		v4l2_subdev_unlock_state(state);
		goto err_pipeline;
	}

	ctx->stream = route->sink_stream;

	v4l2_subdev_unlock_state(state);

	ret = ti_csi2rx_get_vc(ctx);
	if (ret == -ENOIOCTLCMD)
		ctx->vc = 0;
	else if (ret < 0)
		goto err_pipeline;
	else
		ctx->vc = ret;

	ti_csi2rx_setup_shim(ctx);

	ret = v4l2_subdev_call(&csi->subdev, video, s_stream, 1);
	if (ret)
		goto err_pipeline;

	ctx->sequence = 0;

	spin_lock_irqsave(&dma->lock, flags);
	buf = list_entry(dma->queue.next, struct ti_csi2rx_buffer, list);
	list_del(&buf->list);
	dma->state = TI_CSI2RX_DMA_ACTIVE;

	ret = ti_csi2rx_start_dma(ctx, buf);
	if (ret) {
		dev_err(csi->dev, "Failed to start DMA: %d\n", ret);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_QUEUED);
		spin_unlock_irqrestore(&dma->lock, flags);
		goto err_stream;
	}

	dma->curr = buf;
	spin_unlock_irqrestore(&dma->lock, flags);

	return 0;

err_stream:
	v4l2_subdev_call(&csi->subdev, video, s_stream, 0);
err_pipeline:
	media_pipeline_stop(ctx->vdev.entity.pads);
err:
	spin_lock_irqsave(&dma->lock, flags);
	list_for_each_entry_safe(buf, tmp, &dma->queue, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_QUEUED);
	}
	ctx->dma.state = TI_CSI2RX_DMA_STOPPED;
	spin_unlock_irqrestore(&dma->lock, flags);

	return ret;
}

static void ti_csi2rx_stop_streaming(struct vb2_queue *vq)
{
	struct ti_csi2rx_ctx *ctx = vb2_get_drv_priv(vq);
	struct ti_csi2rx_dev *csi = ctx->csi;
	struct ti_csi2rx_buffer *buf = NULL, *tmp;
	struct ti_csi2rx_dma *dma = &ctx->dma;
	unsigned long flags = 0;
	enum ti_csi2rx_dma_state state;
	int ret;

	media_pipeline_stop(ctx->vdev.entity.pads);

	ret = v4l2_subdev_call(&csi->subdev, video, s_stream, 0);
	if (ret)
		dev_err(csi->dev, "Failed to stop subdev stream\n");

	ret = dmaengine_terminate_sync(ctx->dma.chan);
	if (ret)
		dev_err(csi->dev, "Failed to stop DMA\n");

	writel(0, csi->shim + SHIM_DMACNTX(ctx->idx));

	spin_lock_irqsave(&dma->lock, flags);
	list_for_each_entry_safe(buf, tmp, &ctx->dma.queue, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}

	if (dma->curr)
		vb2_buffer_done(&dma->curr->vb.vb2_buf, VB2_BUF_STATE_ERROR);

	state = dma->state;

	dma->curr = NULL;
	dma->state = TI_CSI2RX_DMA_STOPPED;
	spin_unlock_irqrestore(&dma->lock, flags);

	/*
	 * TODO: For some reason the first frame is wrong if we don't toggle
	 * the pixel reset. But at the same time, drain does not work either.
	 * Figure this one out.
	 */
	if (state != TI_CSI2RX_DMA_STOPPED) {
		ret = ti_csi2rx_drain_dma(ctx);
		if (ret)
			dev_dbg(csi->dev,
				"Failed to drain DMA. Next frame might be bogus\n");
	}
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

static inline struct ti_csi2rx_dev *to_csi2rx_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ti_csi2rx_dev, subdev);
}

static int ti_csi2rx_sd_set_fmt(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *fmt;
	int ret = 0;

	/* No transcoding, don't allow setting source fmt */
	if (format->pad >= TI_CSI2RX_PAD_FIRST_SOURCE)
		return v4l2_subdev_get_fmt(sd, state, format);

	if (!find_format_by_code(format->format.code))
		format->format.code = formats[0].code;

	v4l2_subdev_lock_state(state);

	fmt = v4l2_state_get_stream_format(state, format->pad, format->stream);
	if (!fmt) {
		ret = -EINVAL;
		goto out;
	}
	*fmt = format->format;

	fmt = v4l2_state_get_opposite_stream_format(state, format->pad,
						    format->stream);
	if (!fmt) {
		ret = -EINVAL;
		goto out;
	}
	*fmt = format->format;

out:
	v4l2_subdev_unlock_state(state);
	return ret;
}

static int _ti_csi2rx_sd_set_routing(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state,
				    struct v4l2_subdev_krouting *routing)
{
	int ret;

	const struct v4l2_mbus_framefmt format = {
		.width = 640,
		.height = 480,
		.code = MEDIA_BUS_FMT_UYVY8_1X16,
		.field = V4L2_FIELD_NONE,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.ycbcr_enc = V4L2_YCBCR_ENC_601,
		.quantization = V4L2_QUANTIZATION_LIM_RANGE,
		.xfer_func = V4L2_XFER_FUNC_SRGB,
	};

	v4l2_subdev_lock_state(state);

	ret = v4l2_subdev_set_routing_with_fmt(sd, state, routing, &format);

	v4l2_subdev_unlock_state(state);

	return ret;
}

static int ti_csi2rx_sd_set_routing(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *state,
				     enum v4l2_subdev_format_whence which,
				    struct v4l2_subdev_krouting *routing)
{
	return _ti_csi2rx_sd_set_routing(sd, state, routing);
}

static int ti_csi2rx_sd_init_cfg(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_route routes[] = { {
		.sink_pad = 0,
		.sink_stream = 0,
		.source_pad = TI_CSI2RX_PAD_FIRST_SOURCE,
		.source_stream = 0,
		.flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE,
	} };

	struct v4l2_subdev_krouting routing = {
		.num_routes = 1,
		.routes = routes,
	};

	/* Initialize routing to single route to the fist source pad */
	return _ti_csi2rx_sd_set_routing(sd, state, &routing);
}

static int ti_csi2rx_sd_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ti_csi2rx_dev *csi = to_csi2rx_dev(sd);
	int ret = 0;

	mutex_lock(&csi->mutex);

	if (enable) {
		if (csi->enable_count > 0) {
			csi->enable_count++;
			goto out;
		}

		ret = v4l2_subdev_call(csi->source, video, s_stream, 1);
		if (ret)
			goto out;

		csi->enable_count++;
	} else {
		if (csi->enable_count == 0) {
			ret = -EINVAL;
			goto out;
		}

		if (--csi->enable_count > 0)
			goto out;

		ret = v4l2_subdev_call(csi->source, video, s_stream, 0);
	}

out:
	mutex_unlock(&csi->mutex);
	return ret;
}

static const struct v4l2_subdev_video_ops ti_csi2rx_subdev_video_ops = {
	.s_stream = ti_csi2rx_sd_s_stream,
};

static const struct v4l2_subdev_pad_ops ti_csi2rx_subdev_pad_ops = {
	.init_cfg = ti_csi2rx_sd_init_cfg,
	.set_routing = ti_csi2rx_sd_set_routing,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = ti_csi2rx_sd_set_fmt,
};

static const struct v4l2_subdev_ops ti_csi2rx_subdev_ops = {
	.video = &ti_csi2rx_subdev_video_ops,
	.pad = &ti_csi2rx_subdev_pad_ops,
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
	v4l2_async_notifier_unregister(&csi->notifier);
	v4l2_async_notifier_cleanup(&csi->notifier);
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
	q->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF | VB2_READ;
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

static int ti_csi2rx_init_dma(struct ti_csi2rx_ctx *ctx)
{
	struct dma_slave_config cfg;
	char name[32];
	int ret;

	INIT_LIST_HEAD(&ctx->dma.queue);
	spin_lock_init(&ctx->dma.lock);

	ctx->dma.state = TI_CSI2RX_DMA_STOPPED;

	snprintf(name, sizeof(name), "rx%u", ctx->idx);
	ctx->dma.chan = dma_request_chan(ctx->csi->dev, name);
	if (IS_ERR(ctx->dma.chan))
		return PTR_ERR(ctx->dma.chan);

	memset(&cfg, 0, sizeof(cfg));

	cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_16_BYTES;

	ret = dmaengine_slave_config(ctx->dma.chan, &cfg);
	if (ret)
		return ret;

	return 0;
}

static int ti_csi2rx_v4l2_init(struct ti_csi2rx_dev *csi)
{
	struct media_device *mdev = &csi->mdev;
	struct v4l2_subdev *sd = &csi->subdev;
	int ret, i;

	mdev->dev = csi->dev;
	mdev->hw_revision = 1;
	strscpy(mdev->model, "TI-CSI2RX", sizeof(mdev->model));
	snprintf(mdev->bus_info, sizeof(mdev->bus_info), "platform:%s",
		 dev_name(mdev->dev));

	media_device_init(mdev);

	csi->v4l2_dev.mdev = mdev;

	ret = v4l2_device_register(csi->dev, &csi->v4l2_dev);
	if (ret)
		goto cleanup_media;

	ret = media_device_register(mdev);
	if (ret)
		goto unregister_v4l2;

	v4l2_subdev_init(sd, &ti_csi2rx_subdev_ops);
	sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	sd->flags = V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_MULTIPLEXED;
	strscpy(sd->name, dev_name(csi->dev), sizeof(sd->name));
	sd->dev = csi->dev;

	csi->pads[TI_CSI2RX_PAD_SINK].flags = MEDIA_PAD_FL_SINK;

	for (i = TI_CSI2RX_PAD_FIRST_SOURCE;
	     i < TI_CSI2RX_PAD_FIRST_SOURCE + csi->num_ctx; i++)
		csi->pads[i].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&sd->entity,
				     TI_CSI2RX_PAD_FIRST_SOURCE + csi->num_ctx,
				     csi->pads);
	if (ret)
		goto unregister_media;

	ret = v4l2_subdev_init_finalize(sd);
	if (ret)
		goto unregister_media;

	ret = v4l2_device_register_subdev(&csi->v4l2_dev, sd);
	if (ret)
		goto cleanup_subdev;

	return 0;

cleanup_subdev:
	v4l2_subdev_cleanup(sd);
unregister_media:
	media_device_unregister(mdev);
unregister_v4l2:
	v4l2_device_unregister(&csi->v4l2_dev);
cleanup_media:
	media_device_cleanup(mdev);

	return ret;
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

	ctx->pad.flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&ctx->vdev.entity, 1, &ctx->pad);
	if (ret)
		return ret;

	snprintf(vdev->name, sizeof(vdev->name), "%s context %u",
		 dev_name(csi->dev), ctx->idx);
	vdev->v4l2_dev = &csi->v4l2_dev;
	vdev->vfl_dir = VFL_DIR_RX;
	vdev->fops = &csi_fops;
	vdev->ioctl_ops = &csi_ioctl_ops;
	vdev->release = video_device_release_empty;
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_READWRITE |
			    V4L2_CAP_STREAMING | V4L2_CAP_IO_MC;
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

#ifdef CONFIG_PM
static int ti_csi2rx_suspend(struct device *dev)
{
	struct ti_csi2rx_dev *csi = dev_get_drvdata(dev);
	struct ti_csi2rx_ctx *ctx;
	struct ti_csi2rx_dma *dma;
	unsigned long flags = 0;
	int i, ret = 0;

	for (i = 0; i < csi->num_ctx; i++) {
		ctx = &csi->ctx[i];
		dma = &ctx->dma;

		spin_lock_irqsave(&dma->lock, flags);
		if (dma->state != TI_CSI2RX_DMA_STOPPED) {
			spin_unlock_irqrestore(&dma->lock, flags);
			ret = v4l2_subdev_call(&csi->subdev, video, s_stream, 0);
			if (ret)
				dev_err(csi->dev, "Failed to stop subdev stream\n");
			/* Terminate DMA */
			ret = dmaengine_terminate_sync(ctx->dma.chan);
			if (ret)
				dev_err(csi->dev, "Failed to stop DMA\n");
		} else {
			spin_unlock_irqrestore(&dma->lock, flags);
		}

		/* Stop any on-going streams */
		writel(0, csi->shim + SHIM_DMACNTX(ctx->idx));
	}

	/* Assert the pixel reset. */
	writel(0, csi->shim + SHIM_CNTL);

	return ret;
}

static int ti_csi2rx_resume(struct device *dev)
{
	struct ti_csi2rx_dev *csi = dev_get_drvdata(dev);
	struct ti_csi2rx_ctx *ctx;
	struct ti_csi2rx_dma *dma;
	struct ti_csi2rx_buffer *buf;
	unsigned long flags = 0;
	unsigned int reg;
	int i, ret = 0;

	reg = SHIM_CNTL_PIX_RST;
	writel(reg, csi->shim + SHIM_CNTL);

	for (i = 0; i < csi->num_ctx; i++) {
		ctx = &csi->ctx[i];
		dma = &ctx->dma;
		spin_lock_irqsave(&dma->lock, flags);
		if (dma->state != TI_CSI2RX_DMA_STOPPED) {
			buf = dma->curr;
			spin_unlock_irqrestore(&dma->lock, flags);

			/* Restore stream config */
			ti_csi2rx_setup_shim(ctx);

			ret = v4l2_subdev_call(&csi->subdev, video, s_stream, 1);
			if (ret)
				dev_err(ctx->csi->dev, "Failed to start subdev\n");

			/* Restart DMA */
			if (buf)
				ti_csi2rx_restart_dma(ctx, buf);
		} else {
			spin_unlock_irqrestore(&dma->lock, flags);
		}
	}

	return ret;
}

static const struct dev_pm_ops ti_csi2rx_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ti_csi2rx_suspend, ti_csi2rx_resume)
};
#endif /* CONFIG_PM */

static int ti_csi2rx_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct ti_csi2rx_dev *csi;
	struct resource *res;
	int ret, i, count;
	unsigned int reg;

	csi = devm_kzalloc(&pdev->dev, sizeof(*csi), GFP_KERNEL);
	if (!csi)
		return -ENOMEM;

	csi->dev = &pdev->dev;
	platform_set_drvdata(pdev, csi);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	csi->shim = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(csi->shim))
		return PTR_ERR(csi->shim);

	/* Only use as many contexts as the number of DMA channels allocated. */
	count = of_property_count_strings(np, "dma-names");
	if (count < 0) {
		dev_err(csi->dev, "Failed to get DMA channel count: %d\n",
			count);
		return count;
	}

	csi->num_ctx = count;
	if (csi->num_ctx > TI_CSI2RX_MAX_CTX) {
		dev_warn(csi->dev,
			 "%u DMA channels passed. Maximum is %u. Ignoring the rest.\n",
			 csi->num_ctx, TI_CSI2RX_MAX_CTX);
		csi->num_ctx = TI_CSI2RX_MAX_CTX;
	}

	mutex_init(&csi->mutex);

	ret = ti_csi2rx_v4l2_init(csi);
	if (ret)
		return ret;

	for (i = 0; i < csi->num_ctx; i++) {
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

	/* De-assert the pixel interface reset. */
	reg = SHIM_CNTL_PIX_RST;
	writel(reg, csi->shim + SHIM_CNTL);

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

	for (i = 0; i < csi->num_ctx; i++) {
		if (vb2_is_busy(&csi->ctx[i].vidq))
			return -EBUSY;
	}

	for (i = 0; i < csi->num_ctx; i++)
		ti_csi2rx_cleanup_ctx(&csi->ctx[i]);

	ti_csi2rx_cleanup_subdev(csi);
	ti_csi2rx_cleanup_v4l2(csi);

	/* Assert the pixel reset. */
	writel(0, csi->shim + SHIM_CNTL);

	mutex_destroy(&csi->mutex);

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
		.name		= TI_CSI2RX_MODULE_NAME,
		.of_match_table	= ti_csi2rx_of_match,
#ifdef CONFIG_PM
		.pm		= &ti_csi2rx_pm_ops,
#endif
	},
};

module_platform_driver(ti_csi2rx_pdrv);

MODULE_DESCRIPTION("TI J721E CSI2 RX Driver");
MODULE_AUTHOR("Pratyush Yadav <p.yadav@ti.com>");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");
