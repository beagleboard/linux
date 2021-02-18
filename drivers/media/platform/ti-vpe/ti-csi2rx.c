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

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>

#define TI_CSI2RX_MODULE_NAME		"ti-csi2rx"

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
#define SHIM_PSI_CFG0_DST_TAG		GENMASK(31, 15)

#define CSI_DF_YUV420			0x18
#define CSI_DF_YUV422			0x1e
#define CSI_DF_RGB444			0x20
#define CSI_DF_RGB888			0x24

struct ti_csi2rx_fmt {
	u32				fourcc;	/* Four character code. */
	u32				code;	/* Mbus code. */
	u32				csi_df;	/* CSI Data format. */
	u8				bpp;	/* Bits per pixel. */
};

struct ti_csi2rx_buffer {
	/* Common v4l2 buffer. Must be first. */
	struct vb2_v4l2_buffer		vb;
	struct list_head		list;
};

struct ti_csi2rx_dmaq {
	struct list_head		list;
};

struct ti_csi2rx_dev {
	struct device			*dev;
	void __iomem			*shim;
	struct v4l2_device		v4l2_dev;
	struct video_device		vdev;
	struct media_device		mdev;
	struct v4l2_async_notifier	notifier;
	struct v4l2_subdev		*subdev;
	struct vb2_queue		vidq;
	struct mutex			mutex; /* To serialize ioctls. */
	const struct ti_csi2rx_fmt	**supported_fmts;
	unsigned int			num_supported_fmts;
	struct v4l2_format		v_fmt;
	struct dma_chan			*dma;
	struct ti_csi2rx_dmaq		dmaq;
	u32				sequence;
};

static const struct ti_csi2rx_fmt formats[] = {
	{
		.fourcc			= V4L2_PIX_FMT_YUYV,
		.code			= MEDIA_BUS_FMT_YUYV8_2X8,
		.csi_df			= CSI_DF_YUV422,
		.bpp			= 16,
	}, {
		.fourcc			= V4L2_PIX_FMT_UYVY,
		.code			= MEDIA_BUS_FMT_UYVY8_2X8,
		.csi_df			= CSI_DF_YUV422,
		.bpp			= 16,
	}, {
		.fourcc			= V4L2_PIX_FMT_YVYU,
		.code			= MEDIA_BUS_FMT_YVYU8_2X8,
		.csi_df			= CSI_DF_YUV422,
		.bpp			= 16,
	}, {
		.fourcc			= V4L2_PIX_FMT_VYUY,
		.code			= MEDIA_BUS_FMT_VYUY8_2X8,
		.csi_df			= CSI_DF_YUV422,
		.bpp			= 16,
	},

	/* More formats can be supported but they are not listed for now. */
};

/* Forward declaration needed by ti_csi2rx_dma_callback. */
static int ti_csi2rx_start_dma(struct ti_csi2rx_dev *csi,
			       struct ti_csi2rx_buffer *buf);

static const struct ti_csi2rx_fmt *find_format_by_pix(struct ti_csi2rx_dev *csi,
						      u32 pixelformat)
{
	const struct ti_csi2rx_fmt *fmt;
	unsigned int i;

	for (i = 0; i < csi->num_supported_fmts; i++) {
		fmt = csi->supported_fmts[i];
		if (fmt->fourcc == pixelformat)
			return fmt;
	}

	return NULL;
}

static const struct ti_csi2rx_fmt *find_format_by_code(struct ti_csi2rx_dev *csi,
						       u32 code)
{
	const struct ti_csi2rx_fmt *fmt;
	unsigned int i;

	for (i = 0; i < csi->num_supported_fmts; i++) {
		fmt = csi->supported_fmts[i];
		if (fmt->code == code)
			return fmt;
	}

	return NULL;
}

static void ti_csi2rx_fill_fmt(struct v4l2_mbus_framefmt *mbus_fmt,
			       const struct ti_csi2rx_fmt *csi_fmt,
			       struct v4l2_format *v4l2_fmt)
{
	u32 bpl;

	v4l2_fill_pix_format(&v4l2_fmt->fmt.pix, mbus_fmt);
	v4l2_fmt->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	v4l2_fmt->fmt.pix.pixelformat = csi_fmt->fourcc;
	v4l2_fmt->fmt.pix.sizeimage = v4l2_fmt->fmt.pix.height *
				       v4l2_fmt->fmt.pix.width *
				       (csi_fmt->bpp / 8);

	bpl = (v4l2_fmt->fmt.pix.width * ALIGN(csi_fmt->bpp, 8)) >> 3;
	v4l2_fmt->fmt.pix.bytesperline = ALIGN(bpl, 16);
}

static int ti_csi2rx_init_formats(struct ti_csi2rx_dev *csi)
{
	struct v4l2_subdev_mbus_code_enum mbus_code;
	struct v4l2_mbus_framefmt mbus_fmt;
	struct v4l2_subdev_format sd_fmt;
	const struct ti_csi2rx_fmt *fmt;
	unsigned int i, j, k;
	int ret = 0;

	csi->supported_fmts = devm_kcalloc(csi->dev, ARRAY_SIZE(formats),
					   sizeof(*csi->supported_fmts),
					   GFP_KERNEL);
	if (!csi->supported_fmts)
		return -ENOMEM;

	csi->num_supported_fmts = 0;

	/* Find a set of formarts supported by both CSI2RX and camera. */
	for (j = 0, i = 0; ret != -EINVAL; j++) {
		memset(&mbus_code, 0, sizeof(mbus_code));
		mbus_code.index = j;
		mbus_code.which = V4L2_SUBDEV_FORMAT_ACTIVE;
		ret = v4l2_subdev_call(csi->subdev, pad, enum_mbus_code, NULL,
				       &mbus_code);
		if (ret)
			continue;

		for (k = 0; k < ARRAY_SIZE(formats); k++) {
			const struct ti_csi2rx_fmt *fmt = &formats[k];

			if (mbus_code.code == fmt->code) {
				csi->supported_fmts[i] = fmt;
				csi->num_supported_fmts = ++i;
			}
		}
	}

	if (i == 0)
		return -EINVAL; /* No format mutually supported. */

	/* Get the current format from the subdev. */
	sd_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	sd_fmt.pad = 0;

	ret = v4l2_subdev_call(csi->subdev, pad, get_fmt, NULL, &sd_fmt);
	if (ret)
		return ret;

	mbus_fmt = sd_fmt.format;

	fmt = find_format_by_code(csi, mbus_fmt.code);
	if (!fmt)
		return -EINVAL;

	/*
	 * YUV422 8-bit should always be sent by the camera in UYVY. Format
	 * conversions to other YUV422 formats will be done later by SHIM.
	 */
	if (fmt->csi_df == CSI_DF_YUV422 &&
	    fmt->code != MEDIA_BUS_FMT_UYVY8_2X8) {
		sd_fmt.format.code = MEDIA_BUS_FMT_UYVY8_2X8;
		ret = v4l2_subdev_call(csi->subdev, pad, set_fmt, NULL, &sd_fmt);
		if (ret)
			return ret;
	}

	ti_csi2rx_fill_fmt(&mbus_fmt, fmt, &csi->v_fmt);

	return 0;
}

static int ti_csi2rx_querycap(struct file *file, void *priv,
			      struct v4l2_capability *cap)
{
	struct ti_csi2rx_dev *csi = video_drvdata(file);

	strscpy(cap->driver, TI_CSI2RX_MODULE_NAME, sizeof(cap->driver));
	strscpy(cap->card, TI_CSI2RX_MODULE_NAME, sizeof(cap->card));

	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 dev_name(csi->dev));

	return 0;
}

static int ti_csi2rx_enum_fmt_vid_cap(struct file *file, void *priv,
				      struct v4l2_fmtdesc *f)
{
	struct ti_csi2rx_dev *csi = video_drvdata(file);
	const struct ti_csi2rx_fmt *fmt;

	if (f->index >= csi->num_supported_fmts)
		return -EINVAL;

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	fmt = csi->supported_fmts[f->index];

	f->pixelformat = fmt->fourcc;

	return 0;
}

static int ti_csi2rx_g_fmt_vid_cap(struct file *file, void *prov,
				   struct v4l2_format *f)
{
	struct ti_csi2rx_dev *csi = video_drvdata(file);

	*f = csi->v_fmt;

	return 0;
}

static int ti_csi2rx_try_fmt_vid_cap(struct file *file, void *priv,
				     struct v4l2_format *f)
{
	struct ti_csi2rx_dev *csi = video_drvdata(file);
	const struct ti_csi2rx_fmt *fmt;
	struct v4l2_subdev_frame_size_enum fse;
	int ret, found;

	fmt = find_format_by_pix(csi, f->fmt.pix.pixelformat);
	if (!fmt) {
		/* Just get the first one enumerated */
		fmt = csi->supported_fmts[0];
		f->fmt.pix.pixelformat = fmt->fourcc;
	}

	f->fmt.pix.field = csi->v_fmt.fmt.pix.field;

	/* Check for/find a valid width/height. */
	ret = 0;
	found = false;
	fse.pad = 0;
	fse.code = fmt->code;
	fse.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	for (fse.index = 0; ; fse.index++) {
		ret = v4l2_subdev_call(csi->subdev, pad, enum_frame_size, NULL,
				       &fse);
		if (ret)
			break;

		if (f->fmt.pix.width == fse.max_width &&
		    f->fmt.pix.height == fse.max_height) {
			found = true;
			break;
		} else if (f->fmt.pix.width >= fse.min_width &&
			   f->fmt.pix.width <= fse.max_width &&
			   f->fmt.pix.height >= fse.min_height &&
			   f->fmt.pix.height <= fse.max_height) {
			found = true;
			break;
		}
	}

	if (!found) {
		/* use existing values as default */
		f->fmt.pix.width = csi->v_fmt.fmt.pix.width;
		f->fmt.pix.height =  csi->v_fmt.fmt.pix.height;
	}

	/*
	 * Use current colorspace for now, it will get
	 * updated properly during s_fmt
	 */
	f->fmt.pix.colorspace = csi->v_fmt.fmt.pix.colorspace;
	f->fmt.pix.sizeimage = f->fmt.pix.width * f->fmt.pix.height *
			       (fmt->bpp / 8);

	return 0;
}

static int ti_csi2rx_s_fmt_vid_cap(struct file *file, void *priv,
				   struct v4l2_format *f)
{
	struct ti_csi2rx_dev *csi = video_drvdata(file);
	struct vb2_queue *q = &csi->vidq;
	const struct ti_csi2rx_fmt *fmt;
	struct v4l2_mbus_framefmt mbus_fmt;
	struct v4l2_subdev_format sd_fmt;
	int ret;

	if (vb2_is_busy(q))
		return -EBUSY;

	ret = ti_csi2rx_try_fmt_vid_cap(file, priv, f);
	if (ret < 0)
		return ret;

	fmt = find_format_by_pix(csi, f->fmt.pix.pixelformat);
	if (!fmt)
		return -EINVAL;

	v4l2_fill_mbus_format(&mbus_fmt, &f->fmt.pix, fmt->code);

	sd_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	sd_fmt.pad = 0;
	sd_fmt.format = mbus_fmt;

	/*
	 * The MIPI CSI-2 spec says that YUV422 8-bit data transmission is
	 * perfromed using UYVY sequence. So the hardware always expects UYVY
	 * for YUV422 streams. Ask the subdev to send UYVY data and then we can
	 * repack it later into the format requested by the application using
	 * the SHIM layer.
	 */
	if (fmt->csi_df == CSI_DF_YUV422)
		sd_fmt.format.code = MEDIA_BUS_FMT_UYVY8_2X8;
	else
		sd_fmt.format.code = fmt->code;

	ret = v4l2_subdev_call(csi->subdev, pad, set_fmt, NULL, &sd_fmt);
	if (ret)
		return ret;

	ti_csi2rx_fill_fmt(&mbus_fmt, fmt, &csi->v_fmt);
	*f = csi->v_fmt;

	return 0;
}

static int ti_csi2rx_enum_framesizes(struct file *file, void *fh,
				     struct v4l2_frmsizeenum *fsize)
{
	struct ti_csi2rx_dev *csi = video_drvdata(file);
	const struct ti_csi2rx_fmt *fmt;
	struct v4l2_subdev_frame_size_enum fse;
	int ret;

	fmt = find_format_by_pix(csi, fsize->pixel_format);
	if (!fmt)
		return -EINVAL;

	fse.index = fsize->index;
	fse.code = fmt->code;
	fse.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	fse.pad = 0;

	ret = v4l2_subdev_call(csi->subdev, pad, enum_frame_size, NULL, &fse);
	if (ret)
		return ret;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = fse.max_width;
	fsize->discrete.height = fse.max_height;

	return 0;
}

static int ti_csi2rx_enum_input(struct file *file, void *priv,
				struct v4l2_input *inp)
{
	if (inp->index > 0)
		return -EINVAL;

	inp->type = V4L2_INPUT_TYPE_CAMERA;
	snprintf(inp->name, sizeof(inp->name), "Camera %u", inp->index);

	return 0;
}

static int ti_csi2rx_g_input(struct file *file, void *priv, unsigned int *i)
{
	*i = 0;
	return 0;
}

static int ti_csi2rx_s_input(struct file *file, void *priv, unsigned int i)
{
	return i > 0 ? -EINVAL : 0;
}

static int ti_csi2rx_enum_frameintervals(struct file *file, void *priv,
					 struct v4l2_frmivalenum *fival)
{
	struct ti_csi2rx_dev *csi = video_drvdata(file);
	const struct ti_csi2rx_fmt *fmt;
	struct v4l2_subdev_frame_interval_enum fie = {
		.index = fival->index,
		.width = fival->width,
		.height = fival->height,
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	int ret;

	fmt = find_format_by_pix(csi, fival->pixel_format);
	if (!fmt)
		return -EINVAL;

	fie.code = fmt->code;
	ret = v4l2_subdev_call(csi->subdev, pad, enum_frame_interval,
			       NULL, &fie);
	if (ret)
		return ret;
	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete = fie.interval;

	return 0;
}

static int ti_csi2rx_g_parm(struct file *file, void *fh,
			    struct v4l2_streamparm *a)
{
	struct ti_csi2rx_dev *csi = video_drvdata(file);

	return v4l2_g_parm_cap(video_devdata(file), csi->subdev, a);
}

static int ti_csi2rx_s_parm(struct file *file, void *fh,
			    struct v4l2_streamparm *a)
{
	struct ti_csi2rx_dev *csi = video_drvdata(file);

	return v4l2_s_parm_cap(video_devdata(file), csi->subdev, a);
}

static const struct v4l2_ioctl_ops csi_ioctl_ops = {
	.vidioc_querycap      = ti_csi2rx_querycap,
	.vidioc_enum_fmt_vid_cap = ti_csi2rx_enum_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap = ti_csi2rx_try_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap = ti_csi2rx_g_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap = ti_csi2rx_s_fmt_vid_cap,
	.vidioc_enum_framesizes = ti_csi2rx_enum_framesizes,
	.vidioc_enum_input    = ti_csi2rx_enum_input,
	.vidioc_g_input       = ti_csi2rx_g_input,
	.vidioc_s_input       = ti_csi2rx_s_input,
	.vidioc_enum_frameintervals = ti_csi2rx_enum_frameintervals,
	.vidioc_g_parm        = ti_csi2rx_g_parm,
	.vidioc_s_parm        = ti_csi2rx_s_parm,
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

static int ti_csi2rx_init_video(struct ti_csi2rx_dev *csi)
{
	struct video_device *vdev = &csi->vdev;
	int ret;

	strscpy(vdev->name, TI_CSI2RX_MODULE_NAME, sizeof(vdev->name));
	vdev->v4l2_dev = &csi->v4l2_dev;
	vdev->vfl_dir = VFL_DIR_RX;
	vdev->fops = &csi_fops;
	vdev->ioctl_ops = &csi_ioctl_ops;
	vdev->release = video_device_release_empty;
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_READWRITE |
			    V4L2_CAP_STREAMING;
	vdev->lock = &csi->mutex;
	video_set_drvdata(vdev, csi);

	ret = video_register_device(vdev, VFL_TYPE_GRABBER, -1);
	if (ret)
		return ret;

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
	int ret;

	ret = ti_csi2rx_init_formats(csi);
	if (ret)
		return ret;

	ret = ti_csi2rx_init_video(csi);
	if (ret)
		return ret;

	return 0;
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

	v4l2_async_notifier_init(&csi->notifier);
	csi->notifier.ops = &csi_async_notifier_ops;

	fwnode = of_fwnode_handle(node);
	if (!fwnode)
		return -EINVAL;

	asd = v4l2_async_notifier_add_fwnode_subdev(&csi->notifier, fwnode,
						    sizeof(*asd));
	if (IS_ERR(asd))
		return PTR_ERR(asd);

	ret = v4l2_async_notifier_register(&csi->v4l2_dev, &csi->notifier);
	if (ret)
		return ret;

	return 0;
}

static void ti_csi2rx_setup_shim(struct ti_csi2rx_dev *csi)
{
	const struct ti_csi2rx_fmt *fmt;
	unsigned int reg;

	fmt = find_format_by_pix(csi, csi->v_fmt.fmt.pix.pixelformat);
	if (!fmt) {
		dev_err(csi->dev, "Unknown format\n");
		return;
	}

	/* De-assert the pixel interface reset. */
	reg = SHIM_CNTL_PIX_RST;
	writel(reg, csi->shim + SHIM_CNTL);

	reg = SHIM_DMACNTX_EN;
	reg |= FIELD_PREP(SHIM_DMACNTX_FMT, fmt->csi_df);

	/*
	 * FIXME: Using the values from the documentation gives incorrect
	 * ordering for the luma and chroma components. In practice, the
	 * "reverse" format gives the correct image. So for example, if the
	 * image is in UYVY, the reverse would be YVYU.
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
	      FIELD_PREP(SHIM_PSI_CFG0_DST_TAG, 1);
	writel(reg, csi->shim + SHIM_PSI_CFG0);
}

static void ti_csi2rx_dma_callback(void *param)
{
	struct ti_csi2rx_dev *csi = param;
	struct ti_csi2rx_buffer *buf;
	struct ti_csi2rx_dmaq *dmaq = &csi->dmaq;

	buf = list_entry(dmaq->list.next, struct ti_csi2rx_buffer, list);
	list_del(&buf->list);

	buf->vb.vb2_buf.timestamp = ktime_get_ns();
	buf->vb.sequence = csi->sequence++;

	vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);

	/* If there are more buffers to process then start their transfer. */
	if (list_empty(&dmaq->list))
		return;

	buf = list_entry(dmaq->list.next, struct ti_csi2rx_buffer, list);

	if (ti_csi2rx_start_dma(csi, buf))
		dev_err(csi->dev, "Failed to queue the next buffer for DMA\n");
}

static int ti_csi2rx_start_dma(struct ti_csi2rx_dev *csi,
			       struct ti_csi2rx_buffer *buf)
{
	unsigned long addr;
	struct dma_async_tx_descriptor *desc;
	size_t len = csi->v_fmt.fmt.pix.sizeimage;
	dma_cookie_t cookie;
	int ret = 0;

	addr = vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 0);
	desc = dmaengine_prep_slave_single(csi->dma, addr, len, DMA_DEV_TO_MEM,
					   DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!desc)
		return -EIO;

	desc->callback = ti_csi2rx_dma_callback;
	desc->callback_param = csi;

	cookie = dmaengine_submit(desc);
	ret = dma_submit_error(cookie);
	if (ret)
		return ret;

	dma_async_issue_pending(csi->dma);

	return 0;
}

static int ti_csi2rx_queue_setup(struct vb2_queue *q, unsigned int *nbuffers,
				 unsigned int *nplanes, unsigned int sizes[],
				 struct device *alloc_devs[])
{
	struct ti_csi2rx_dev *csi = vb2_get_drv_priv(q);
	unsigned int size = csi->v_fmt.fmt.pix.sizeimage;

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
	struct ti_csi2rx_dev *csi = vb2_get_drv_priv(vb->vb2_queue);
	unsigned long size = csi->v_fmt.fmt.pix.sizeimage;

	if (vb2_plane_size(vb, 0) < size) {
		dev_err(csi->dev, "Data will not fit into plane\n");
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, size);
	return 0;
}

static void ti_csi2rx_buffer_queue(struct vb2_buffer *vb)
{
	struct ti_csi2rx_dev *csi = vb2_get_drv_priv(vb->vb2_queue);
	struct ti_csi2rx_buffer *buf;
	struct ti_csi2rx_dmaq *dmaq = &csi->dmaq;

	buf = container_of(vb, struct ti_csi2rx_buffer, vb.vb2_buf);

	list_add_tail(&buf->list, &dmaq->list);
}

static int ti_csi2rx_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct ti_csi2rx_dev *csi = vb2_get_drv_priv(vq);
	struct ti_csi2rx_dmaq *dmaq = &csi->dmaq;
	struct ti_csi2rx_buffer *buf, *tmp;
	int ret;

	if (list_empty(&dmaq->list))
		return -EIO;

	ti_csi2rx_setup_shim(csi);

	ret = v4l2_subdev_call(csi->subdev, video, s_stream, 1);
	if (ret)
		goto err;

	csi->sequence = 0;

	buf = list_entry(dmaq->list.next, struct ti_csi2rx_buffer, list);
	ret = ti_csi2rx_start_dma(csi, buf);
	if (ret) {
		dev_err(csi->dev, "Failed to start DMA: %d\n", ret);
		goto err;
	}

	return 0;

err:
	v4l2_subdev_call(csi->subdev, video, s_stream, 0);
	list_for_each_entry_safe(buf, tmp, &dmaq->list, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_QUEUED);
	}

	return ret;
}

static void ti_csi2rx_stop_streaming(struct vb2_queue *vq)
{
	struct ti_csi2rx_dev *csi = vb2_get_drv_priv(vq);
	struct ti_csi2rx_buffer *buf = NULL, *tmp;
	int ret;

	ret = v4l2_subdev_call(csi->subdev, video, s_stream, 0);
	if (ret)
		dev_err(csi->dev, "Failed to stop subdev stream\n");

	writel(0, csi->shim + SHIM_CNTL);

	ret = dmaengine_terminate_sync(csi->dma);
	if (ret)
		dev_err(csi->dev, "Failed to stop DMA\n");

	writel(0, csi->shim + SHIM_DMACNTX);

	list_for_each_entry_safe(buf, tmp, &csi->dmaq.list, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
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

static int ti_csi2rx_init_vb2q(struct ti_csi2rx_dev *csi)
{
	struct vb2_queue *q = &csi->vidq;
	int ret;

	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF | VB2_READ;
	q->drv_priv = csi;
	q->buf_struct_size = sizeof(struct ti_csi2rx_buffer);
	q->ops = &csi_vb2_qops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->dev = csi->dma->device->dev;
	q->lock = &csi->mutex;

	ret = vb2_queue_init(q);
	if (ret)
		return ret;

	csi->vdev.queue = q;

	return 0;
}

static int ti_csi2rx_init_dma(struct ti_csi2rx_dev *csi)
{
	struct dma_slave_config cfg;
	int ret;

	INIT_LIST_HEAD(&csi->dmaq.list);

	csi->dma = NULL;

	csi->dma = dma_request_chan(csi->dev, "rx0");
	if (IS_ERR(csi->dma))
		return PTR_ERR(csi->dma);

	memset(&cfg, 0, sizeof(cfg));

	cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_16_BYTES;
	cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_16_BYTES;

	ret = dmaengine_slave_config(csi->dma, &cfg);
	if (ret)
		return ret;

	return 0;
}

static int ti_csi2rx_media_init(struct ti_csi2rx_dev *csi)
{
	struct media_device *mdev = &csi->mdev;

	mdev->dev = csi->dev;
	strscpy(mdev->model, "TI-CSI2RX", sizeof(mdev->model));
	snprintf(mdev->bus_info, sizeof(mdev->bus_info), "platform:%s",
		 dev_name(mdev->dev));

	media_device_init(mdev);

	return 0;
}

static int ti_csi2rx_probe(struct platform_device *pdev)
{
	struct ti_csi2rx_dev *csi;
	struct resource *res;
	int ret;

	csi = devm_kzalloc(&pdev->dev, sizeof(*csi), GFP_KERNEL);
	if (!csi)
		return -ENOMEM;

	csi->dev = &pdev->dev;
	platform_set_drvdata(pdev, csi);

	mutex_init(&csi->mutex);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	csi->shim = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(csi->shim))
		return PTR_ERR(csi->shim);

	ret = ti_csi2rx_media_init(csi);
	if (ret)
		return ret;

	csi->v4l2_dev.mdev = &csi->mdev;

	ret = v4l2_device_register(csi->dev, &csi->v4l2_dev);
	if (ret)
		return ret;

	ret = ti_csi2rx_init_dma(csi);
	if (ret)
		return ret;

	ret = ti_csi2rx_init_vb2q(csi);
	if (ret)
		return ret;

	ret = ti_csi2rx_init_subdev(csi);
	if (ret)
		return ret;

	ret = of_platform_populate(csi->dev->of_node, NULL, NULL, csi->dev);
	if (ret) {
		dev_err(csi->dev, "Failed to create children: %d\n", ret);
		return ret;
	}

	return 0;
}

static int ti_csi2rx_remove(struct platform_device *pdev)
{
	struct ti_csi2rx_dev *csi = platform_get_drvdata(pdev);

	if (vb2_is_busy(&csi->vidq))
		return -EBUSY;

	vb2_queue_release(&csi->vidq);
	v4l2_async_notifier_unregister(&csi->notifier);
	v4l2_async_notifier_cleanup(&csi->notifier);
	v4l2_device_unregister(&csi->v4l2_dev);
	video_unregister_device(&csi->vdev);
	dma_release_channel(csi->dma);

	return 0;
}

static const struct of_device_id ti_csi2rx_of_match[] = {
	{
		.compatible = "ti,csi2rx",
	},
	{},
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

MODULE_DESCRIPTION("TI CSI2 RX Driver");
MODULE_AUTHOR("Pratyush Yadav <p.yadav@ti.com>");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");
