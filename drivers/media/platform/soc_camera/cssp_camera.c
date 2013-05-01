/*
 * cssp-camera driver
 *
 * Based on Vivi driver
 *
 * Copyright (C) 2012 QuickLogic Corp.
 *
 * Developed for QuickLogic by:
 * Damian Eppel <damian.eppel@teleca.com>
 * Przemek Szewczyk <przemek.szewczyk@teleca.com>
 * Dan Aizenstros <daizenstros@quicklogic.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */


#include <linux/init.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/platform_data/edma.h>
#include <linux/clk.h>
// V4L2 Interface *********************
#include <media/soc_camera.h>
#include <media/v4l2-mediabus.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-event.h>
#include <media/mt9t112.h>
//*************************************
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_dma.h>
#include <linux/of_gpio.h>
#include <linux/of_i2c.h>

static unsigned video_nr = -1;
module_param(video_nr, uint, 0644);
MODULE_PARM_DESC(video_nr, "videoX start number, -1 is autodetect");

static unsigned int vid_limit = 6;
module_param(vid_limit, uint, 0644);
MODULE_PARM_DESC(vid_limit, "capture memory limit in megabytes");

#define VGA_WIDTH 640
#define VGA_HEIGHT 480

#define MAX_WIDTH 2048
#define MAX_HEIGHT 1536

#define VGA_RES (VGA_WIDTH * VGA_HEIGHT)
#define BYTES_PER_PIXEL 2

/* PaRAM.opt: */
#define TCC(v) (((v) & 0x3f) << 12)
/* PaRAM.a_b_cnt: */
#define ACNT(v) ((v) & 0xffff)
#define BCNT(v) (((v) & 0xffff) << 16)
/* PaRAM.src_dst_bidx: */
#define SRCBIDX(v) ((v) & 0xffff)
#define DSTBIDX(v) (((v) & 0xffff) << 16)
/* PaRAM.link_bcntrld: */
#define LINK(v) ((v) & 0xffff)
#define BCNTRLD(v) (((v) & 0xffff) << 16)
/* PaRAM.src_dst_cidx: */
#define SRCCIDX(v) ((v) & 0xffff)
#define DSTCIDX(v) (((v) & 0xffff) << 16)
/* PaRAM.ccnt: */
#define CCNT(v) ((v) & 0xffff)


struct cssp_cam_platform_data {
	struct i2c_board_info *cam_i2c_board_info;
	const char *cam_clk_name;
	u32 cam_clk_rate;
	int dma_ch;
	int gpio_reset_pin;
};

struct cssp_cam_platform_data_storage {
	struct cssp_cam_platform_data pdata;
	struct i2c_board_info i2c_camera;
	struct soc_camera_link camera_link;
	/* only support mt9t112 for now */
	struct mt9t112_camera_info mt9t111_cam_info;
};

#define to_cssp_platform_data_storage(_x) container_of(_x, \
		struct cssp_cam_platform_data_storage, pdata)


/* ------------------------------------------------------------------
	video Basic structures
   ------------------------------------------------------------------*/

struct cssp_cam_fmt {
	char	*name;
	u32	fourcc;          /* v4l2 format id */
	int	depth;
	enum v4l2_mbus_pixelcode code;
};

/* buffer for one video frame */
struct cssp_cam_buffer {
	/* common v4l buffer stuff -- must be first */
	struct vb2_buffer	vb;
	struct list_head	list;
	struct cssp_cam_fmt	*fmt;
};

struct cssp_cam_dmaqueue {
	struct list_head	active;
};

struct cssp_cam_dev {
	struct v4l2_device		v4l2_dev;
	struct v4l2_ctrl_handler	ctrl_handler;
	struct v4l2_subdev		*subdev;

	spinlock_t			slock;
	struct mutex			mutex;

	/* various device info */
	struct video_device		*vdev;
	struct platform_device		*pdev;

	struct cssp_cam_dmaqueue	vidq;
	void				*dma_cont_ctx;
	int				streaming_started;
	struct vb2_buffer		*current_vb;

	/* video capture */
	struct cssp_cam_fmt		*fmt;
	u32				width;
	u32				height;
	u32				bytesperline;
	u32				sizeimage;
	enum v4l2_colorspace		colorspace;
	struct vb2_queue		vb_vidq;
	enum v4l2_field			field;

	/* Camera Sensor */
	struct i2c_board_info		*camera_board_info;
	struct clk			*camera_clk;

	void __iomem			*reg_base_virt;
	unsigned int			reg_base_phys;
	resource_size_t			reg_size;
	u16				mode;

	int				dma_ch;
	struct edmacc_param		dma_tr_params;

	u64				dma_mask;
	int				dma_req_len;

	int				frame_cnt;

	int				reset_pin;

	int				rev;

	/* OF build platform data here */
	struct cssp_cam_platform_data_storage *pstore;
};

/*
 * ---------------------------------------------------------------------------
 *  QuickLoigc Camera Interface registers
 * ---------------------------------------------------------------------------
 */

#define REG_MODE		0x00000
#define REG_DATA		0x10000

/* MODE bit shifts */
#define FMT_2X8_EN		BIT(15) /* Enable 2 byte format on CAMIF bus (0 - 10 bit, 1 - 16 bit 2x8) */
#define PCLK_POL		BIT(14) /* PCLK polarity (0 - rising edge, 1 - falling edge */
#define HS_EN			BIT(13) /* High speed bus (0 =< 50 MHz, 1 > 50 MHz) */
#define ENABLE			BIT(12)
#define LDR_EN			BIT(11) /* Large DMA Request Support (0 - 32 bytes, 1 - 128 bytes) */
#define REV			0xFF	/* Chip Revision mask */


static struct cssp_cam_fmt formats[] = {
	{
		.name	= "4:2:2, packed, YUYV",
		.fourcc	= V4L2_PIX_FMT_YUYV,
		.depth	= 16,
		.code	= V4L2_MBUS_FMT_YUYV8_2X8,
	},
/*
 * UYVY doesn't work properly. VYUY and YVYU are not tested.
 * So disable the UYVY, VYUY and YVYU modes for now
 */
#if 0
	{
		.name	= "4:2:2, packed, UYVY",
		.fourcc	= V4L2_PIX_FMT_UYVY,
		.depth	= 16,
		.code	= V4L2_MBUS_FMT_UYVY8_2X8,
	},
	{
		.name	= "4:2:2, packed, VYUY",
		.fourcc	= V4L2_PIX_FMT_VYUY,
		.depth	= 16,
		.code	= V4L2_MBUS_FMT_VYUY8_2X8,
	},
	{
		.name	= "4:2:2, packed, YVYU",
		.fourcc	= V4L2_PIX_FMT_YVYU,
		.depth	= 16,
		.code	= V4L2_MBUS_FMT_YVYU8_2X8,
	},
#endif
	{
		.name	= "RGB565 (LE)",
		.fourcc	= V4L2_PIX_FMT_RGB565,
		.depth	= 16,
		.code	= V4L2_MBUS_FMT_RGB565_2X8_LE,
	},
	{
		.name	= "RGB555 (LE)",
		.fourcc	= V4L2_PIX_FMT_RGB555,
		.depth	= 16,
		.code	= V4L2_MBUS_FMT_RGB555_2X8_PADHI_LE,
	},
};


/***************************************************************************/


static int configure_gpio(int nr, int val, const char *name)
{
	unsigned long flags = val ? GPIOF_OUT_INIT_HIGH : GPIOF_OUT_INIT_LOW;
	int ret;
	if (!gpio_is_valid(nr))
		return 0;
	ret = gpio_request_one(nr, flags, name);
	if (!ret)
		gpio_export(nr, 0);
	return ret;
}

static int reset_cssp(struct cssp_cam_dev *cam)
{
	struct platform_device *pdev = cam->pdev;
	struct cssp_cam_platform_data *pdata = pdev->dev.platform_data;
	int err;

	cam->reset_pin = pdata->gpio_reset_pin;
	if (!gpio_is_valid(cam->reset_pin))
		return 0;

	err = configure_gpio(cam->reset_pin, 0, "cssp_reset");
	if (err) {
		dev_err(&pdev->dev, "failed to configure cssp reset pin\n");
		return -1;
	}

	mdelay(1);

	gpio_direction_output(cam->reset_pin, 1);

	return err;
}

static int trigger_dma_transfer_to_buf(struct cssp_cam_dev *dev, struct vb2_buffer *vb)
{
	dma_addr_t dma_buf = vb2_dma_contig_plane_dma_addr(vb, 0);

	if (!dma_buf) {
		/* Is this possible? Release the vb2_buffer with an error here, */
		vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
		dev->current_vb = NULL;
		return -ENOMEM;
	}

	dev->dma_tr_params.dst = dma_buf;

	// Enable DMA
	edma_write_slot(dev->dma_ch, &dev->dma_tr_params);

	dev->current_vb = vb;

	// Enable data capture
	dev->mode |= ENABLE;
	writew(dev->mode, dev->reg_base_virt + REG_MODE);
	readw(dev->reg_base_virt + REG_MODE);

	return 0;
}

static void dequeue_buffer_for_dma(struct cssp_cam_dev *dev)
{
	struct cssp_cam_dmaqueue *dma_q = &dev->vidq;
	unsigned long flags = 0;

	spin_lock_irqsave(&dev->slock, flags);
	if (!list_empty(&dma_q->active)) {
		struct cssp_cam_buffer *buf;

		buf = list_entry(dma_q->active.next, struct cssp_cam_buffer, list);
		list_del(&buf->list);
		spin_unlock_irqrestore(&dev->slock, flags);

		buf->fmt = dev->fmt;

		trigger_dma_transfer_to_buf(dev, &buf->vb);
	} else {
		spin_unlock_irqrestore(&dev->slock, flags);
	}
}

static void dma_callback(unsigned lch, u16 ch_status, void *data)
{
	struct cssp_cam_dev *dev = data;
	struct vb2_buffer *vb;
	struct edmacc_param dma_tr_params;

	// Disable data capture
	dev->mode &= ~ENABLE;
	writew(dev->mode, dev->reg_base_virt + REG_MODE);
	readw(dev->reg_base_virt + REG_MODE);

	if (ch_status == DMA_COMPLETE) {

		vb = dev->current_vb;

		edma_read_slot(dev->dma_ch, &dma_tr_params);
		if ((dma_tr_params.opt != 0) ||
			(dma_tr_params.src != 0) ||
			(dma_tr_params.a_b_cnt != 0) ||
			(dma_tr_params.dst != 0) ||
			(dma_tr_params.src_dst_bidx != 0) ||
			(dma_tr_params.link_bcntrld != 0xffff) ||
			(dma_tr_params.src_dst_cidx != 0) ||
			(dma_tr_params.ccnt != 0)) {

			trigger_dma_transfer_to_buf(dev, dev->current_vb);
			return;
		}

		vb->v4l2_buf.field = dev->field;
		vb->v4l2_buf.sequence = dev->frame_cnt++;
		do_gettimeofday(&vb->v4l2_buf.timestamp);
		vb2_buffer_done(vb, VB2_BUF_STATE_DONE);
		dev->current_vb = NULL;

		/* check if we have new buffer queued */
		dequeue_buffer_for_dma(dev);
	} else {
		/* we got a missed interrupt so just start a new DMA with the existing buffer */
		if (dev->current_vb != NULL) {
			if (trigger_dma_transfer_to_buf(dev, dev->current_vb))
				dev_err(&dev->pdev->dev, "No buffer allocated!\n");
		}
	}
}

static int configure_edma(struct cssp_cam_dev *cam)
{
	struct platform_device *pdev = cam->pdev;
	struct cssp_cam_platform_data *pdata = pdev->dev.platform_data;
	struct edmacc_param *param;
	int dma_channel;

	dma_channel = pdata->dma_ch;

	/* wtf? do we need to this here? */
	pdev->dev.dma_mask = &cam->dma_mask;
	pdev->dev.coherent_dma_mask = (u32)~0;
	if (dma_set_mask(&pdev->dev, (u32)~0)) {
		dev_err(&pdev->dev, "failed setting mask for DMA\n");
		return -EINVAL;
	}
	cam->dma_ch = edma_alloc_channel(dma_channel, dma_callback, cam, EVENTQ_0);
	if (cam->dma_ch < 0) {
		dev_err(&pdev->dev, "allocating channel for DMA failed\n");
		return -EBUSY;
	}

	cam->dma_req_len = cam->rev > 3 ? 128 : 32;

	param = &cam->dma_tr_params;
	param->opt = TCINTEN | TCC(cam->dma_ch);
	param->src = cam->reg_base_phys + REG_DATA;
	param->a_b_cnt = ACNT(cam->dma_req_len) |
		BCNT((VGA_WIDTH * BYTES_PER_PIXEL) / cam->dma_req_len);
	param->src_dst_bidx = SRCBIDX(0) | DSTBIDX(cam->dma_req_len);
	param->link_bcntrld = BCNTRLD((VGA_WIDTH * BYTES_PER_PIXEL) /
			cam->dma_req_len) | LINK(0xffff);
	param->src_dst_cidx = SRCCIDX(0) | DSTCIDX(cam->dma_req_len);
	param->ccnt = CCNT(VGA_HEIGHT);

	return 0;
}

static int configure_cssp(struct cssp_cam_dev *cam)
{
	struct platform_device *pdev = cam->pdev;
	int ret = 0;
	unsigned int val;
	struct resource *res;

	ret = reset_cssp(cam);
	if (ret)
		return ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "no mem resource\n");
		return -ENODEV;
	}

	/*
	 * Request the region.
	 */
	if (!request_mem_region(res->start, resource_size(res), pdev->name)) {
		dev_err(&pdev->dev, "request_mem_region failed\n");
		return -EBUSY;
	}

	cam->reg_base_phys = res->start;
	cam->reg_size = resource_size(res);

	cam->reg_base_virt = ioremap(cam->reg_base_phys, cam->reg_size);
	if (cam->reg_base_virt == NULL) {
		dev_err(&pdev->dev, "ioremap of registers region failed\n");
		release_mem_region(cam->reg_base_phys, cam->reg_size);
		return -ENOMEM;
	}

	/* double reads */
	readw(cam->reg_base_virt + REG_MODE);
	val = readw(cam->reg_base_virt + REG_MODE);
	cam->rev = val & REV;
	dev_info(&pdev->dev, "CSSP Revision %c%d\n",
			'A' + ((cam->rev & 0xf0) >> 4), cam->rev & 0x0f);

	cam->dma_req_len = cam->rev > 3 ? 128 : 32;

	return 0;
}

static int configure_camera_sensor(struct cssp_cam_dev *cam)
{
	struct i2c_board_info *info = cam->camera_board_info;
	struct i2c_client *client;
	struct i2c_adapter *adapter;
	struct v4l2_subdev *subdev;
	struct v4l2_mbus_framefmt f_format = {
			.width = VGA_WIDTH,
			.height = VGA_HEIGHT,
			.code = V4L2_MBUS_FMT_YUYV8_2X8,
			.colorspace = V4L2_COLORSPACE_JPEG,
	};

	/* Enable the clock just for the time of loading the camera driver and disable after that */
	/* It is going to be be re-enabled later, when camera will be in use */
	clk_enable(cam->camera_clk);
	udelay(5); // let the clock stabilize

	adapter	= i2c_get_adapter(((struct soc_camera_link *)(info->platform_data))->i2c_adapter_id);
	if (!adapter) {
		dev_err(&cam->pdev->dev, "failed to get i2c adapter...\n");
		return -ENODEV;
	}

	client = i2c_new_device(adapter, info);
	i2c_put_adapter(adapter);

	if (client == NULL) {
		return -ENODEV;
	}

	subdev = (struct v4l2_subdev *)i2c_get_clientdata(client);
	if (subdev == NULL) {
		i2c_unregister_device(client);
		return -ENODEV;
	}

	cam->subdev = subdev;

	v4l2_subdev_call(subdev, video, s_mbus_fmt, &f_format);

	clk_disable(cam->camera_clk);

	return 0;
}

static int start_camera_sensor(struct cssp_cam_dev *cam)
{
	clk_enable(cam->camera_clk);
	udelay(5); /* let the clock stabilize */

	v4l2_subdev_call(cam->subdev, video, s_stream, 1);

	return 0;
}

static void stop_camera_sensor(struct cssp_cam_dev *cam)
{
	v4l2_subdev_call(cam->subdev, video, s_stream, 0);

	clk_disable(cam->camera_clk);

	return;
}

static struct cssp_cam_fmt *get_format(struct v4l2_format *f)
{
	struct cssp_cam_fmt *fmt;
	unsigned int k;

	for (k = 0; k < ARRAY_SIZE(formats); k++) {
		fmt = &formats[k];
		if (fmt->fourcc == f->fmt.pix.pixelformat)
			break;
	}

	if (k == ARRAY_SIZE(formats))
		return NULL;

	return &formats[k];
}


/* ------------------------------------------------------------------
	Videobuf operations
   ------------------------------------------------------------------*/

static int queue_setup(struct vb2_queue *vq, const struct v4l2_format *fmt,
				unsigned int *nbuffers, unsigned int *nplanes,
				unsigned int sizes[], void *alloc_ctxs[])
{
	struct cssp_cam_dev *dev = vb2_get_drv_priv(vq);
	unsigned long size;

	size = dev->sizeimage;

	if (0 == *nbuffers)
		*nbuffers = 32;

	while (size * *nbuffers > vid_limit * 1024 * 1024)
		(*nbuffers)--;

	*nplanes = 1;

	sizes[0] = size;

	alloc_ctxs[0] = dev->dma_cont_ctx;

	dev->frame_cnt = 0;

	dev_dbg(&dev->pdev->dev, "%s, count=%d, size=%ld\n", __func__,
			*nbuffers, size);

	return 0;
}

static int buffer_init(struct vb2_buffer *vb)
{
	struct cssp_cam_dev *dev = vb2_get_drv_priv(vb->vb2_queue);

	BUG_ON(NULL == dev->fmt);

	/*
	 * This callback is called once per buffer, after its allocation.
	 *
	 * Vivi does not allow changing format during streaming, but it is
	 * possible to do so when streaming is paused (i.e. in streamoff state).
	 * Buffers however are not freed when going into streamoff and so
	 * buffer size verification has to be done in buffer_prepare, on each
	 * qbuf.
	 * It would be best to move verification code here to buf_init and
	 * s_fmt though.
	 */

	return 0;
}

static int buffer_prepare(struct vb2_buffer *vb)
{
	struct cssp_cam_dev *dev = vb2_get_drv_priv(vb->vb2_queue);
	struct cssp_cam_buffer *buf =
		container_of(vb, struct cssp_cam_buffer, vb);
	unsigned long size;

	dev_dbg(&dev->pdev->dev, "%s, field=%d\n", __func__,
			vb->v4l2_buf.field);

	BUG_ON(NULL == dev->fmt);

	/*
	 * Theses properties only change when queue is idle, see s_fmt.
	 * The below checks should not be performed here, on each
	 * buffer_prepare (i.e. on each qbuf). Most of the code in this function
	 * should thus be moved to buffer_init and s_fmt.
	 */
	if (dev->width  < 48 || dev->width  > MAX_WIDTH ||
	    dev->height < 32 || dev->height > MAX_HEIGHT)
		return -EINVAL;

	size = dev->sizeimage;
	if (vb2_plane_size(vb, 0) < size) {
		dev_err(&dev->pdev->dev, "%s data will not fit into "
				"plane (%lu < %lu)\n",
				__func__, vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}

	vb2_set_plane_payload(&buf->vb, 0, size);

	buf->fmt = dev->fmt;

	return 0;
}

static int buffer_finish(struct vb2_buffer *vb)
{
	struct cssp_cam_dev *dev = vb2_get_drv_priv(vb->vb2_queue);
	dev_dbg(&dev->pdev->dev, "%s\n", __func__);
	return 0;
}

static void buffer_cleanup(struct vb2_buffer *vb)
{
	struct cssp_cam_dev *dev = vb2_get_drv_priv(vb->vb2_queue);
	dev_dbg(&dev->pdev->dev, "%s\n", __func__);
}

static void buffer_queue(struct vb2_buffer *vb)
{
	struct cssp_cam_dev *dev = vb2_get_drv_priv(vb->vb2_queue);
	struct cssp_cam_buffer *buf = container_of(vb, struct cssp_cam_buffer, vb);
	struct cssp_cam_dmaqueue *vidq = &dev->vidq;
	unsigned long flags = 0;

	dev_dbg(&dev->pdev->dev, "%s\n", __func__);

	if (dev->streaming_started && !dev->current_vb) {
		trigger_dma_transfer_to_buf(dev, &buf->vb);
	} else {
		spin_lock_irqsave(&dev->slock, flags);
		list_add_tail(&buf->list, &vidq->active);
		spin_unlock_irqrestore(&dev->slock, flags);
	}
}

static int start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct cssp_cam_dev *dev = vb2_get_drv_priv(vq);
	struct platform_device *pdev = dev->pdev;
	int ret;

	dev_dbg(&dev->pdev->dev, "%s\n", __func__);

	ret = start_camera_sensor(dev);
	if (ret != 0) {
		dev_err(&pdev->dev, "start_camera_sensor failed\n");
		return ret;
	}

	// Enable DMA
	ret = edma_start(dev->dma_ch);
	if (ret != 0) {
		dev_err(&pdev->dev, "edma_start failed\n");
		return ret;
	}
	dev->streaming_started = 1;

	/* check if we have new buffer queued */
	dequeue_buffer_for_dma(dev);

	return 0;
}

/* abort streaming and wait for last buffer */
static int stop_streaming(struct vb2_queue *vq)
{
	struct cssp_cam_dev *dev = vb2_get_drv_priv(vq);
	struct cssp_cam_dmaqueue *dma_q = &dev->vidq;

	dev_dbg(&dev->pdev->dev, "%s\n", __func__);

	// Disable DMA
	edma_stop(dev->dma_ch);

	// Disable data capture
	dev->mode &= ~ENABLE;
	writew(dev->mode, dev->reg_base_virt + REG_MODE);

	stop_camera_sensor(dev);

	dev->streaming_started = 0;

	/* Release all active buffers */
	while (!list_empty(&dma_q->active)) {
		struct cssp_cam_buffer *buf;

		buf = list_entry(dma_q->active.next,
				struct cssp_cam_buffer, list);
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);

		dev_dbg(&dev->pdev->dev, "[%p/%d] done\n",
				buf, buf->vb.v4l2_buf.index);
	}

	dev->current_vb = NULL;

	return 0;
}

static void cssp_cam_lock(struct vb2_queue *vq)
{
	struct cssp_cam_dev *dev = vb2_get_drv_priv(vq);
	mutex_lock(&dev->mutex);
}

static void cssp_cam_unlock(struct vb2_queue *vq)
{
	struct cssp_cam_dev *dev = vb2_get_drv_priv(vq);
	mutex_unlock(&dev->mutex);
}

static struct vb2_ops cssp_cam_video_qops = {
	.queue_setup		= queue_setup,
	.buf_init		= buffer_init,
	.buf_prepare		= buffer_prepare,
	.buf_finish		= buffer_finish,
	.buf_cleanup		= buffer_cleanup,
	.buf_queue		= buffer_queue,
	.start_streaming	= start_streaming,
	.stop_streaming		= stop_streaming,
	.wait_prepare		= cssp_cam_unlock,
	.wait_finish		= cssp_cam_lock,
};

/* ------------------------------------------------------------------
	IOCTL vidioc handling
   ------------------------------------------------------------------*/

static int vidioc_querycap(struct file *file, void *priv,
					struct v4l2_capability *cap)
{
	struct cssp_cam_dev *dev = video_drvdata(file);

	strcpy(cap->driver, "cssp_camera");
	strcpy(cap->card, "cssp_camera");
	strlcpy(cap->bus_info, dev->v4l2_dev.name, sizeof(cap->bus_info));
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING |
			    V4L2_CAP_READWRITE;
	return 0;
}

static int vidioc_enum_fmt_vid_cap(struct file *file, void *priv,
					struct v4l2_fmtdesc *f)
{
	struct cssp_cam_fmt *fmt;

	if (f->index >= ARRAY_SIZE(formats))
		return -EINVAL;

	fmt = &formats[f->index];

	strlcpy(f->description, fmt->name, sizeof(f->description));
	f->pixelformat = fmt->fourcc;
	return 0;
}

static int vidioc_g_fmt_vid_cap(struct file *file, void *priv,
					struct v4l2_format *f)
{
	struct cssp_cam_dev *dev = video_drvdata(file);

	f->fmt.pix.width	= dev->width;
	f->fmt.pix.height	= dev->height;
	f->fmt.pix.field	= dev->field;
	f->fmt.pix.pixelformat	= dev->fmt->fourcc;
	f->fmt.pix.bytesperline	= dev->bytesperline;
	f->fmt.pix.sizeimage	= dev->sizeimage;
	f->fmt.pix.colorspace	= dev->colorspace;

	return 0;
}

static int vidioc_try_fmt_vid_cap(struct file *file, void *priv,
			struct v4l2_format *f)
{
	struct cssp_cam_dev *dev = video_drvdata(file);
	struct cssp_cam_fmt *fmt;
	struct v4l2_mbus_framefmt mbus_fmt;
	struct v4l2_pix_format *pix = &f->fmt.pix;

	fmt = get_format(f);
	if (!fmt) {
		dev_err(&dev->pdev->dev, "Fourcc format (0x%08x) invalid.\n",
			f->fmt.pix.pixelformat);
		return -EINVAL;
	}

	v4l2_fill_mbus_format(&mbus_fmt, pix, fmt->code);
	v4l2_subdev_call(dev->subdev, video, try_mbus_fmt, &mbus_fmt);
	v4l2_fill_pix_format(pix, &mbus_fmt);
	pix->bytesperline = (pix->width * fmt->depth) >> 3;
	pix->sizeimage = pix->height * pix->bytesperline;

	if ((pix->sizeimage % dev->dma_req_len) != 0)
		return -EINVAL;

	switch (mbus_fmt.field) {
	case V4L2_FIELD_ANY:
		pix->field = V4L2_FIELD_NONE;
		break;
	case V4L2_FIELD_NONE:
		break;
	default:
		dev_err(&dev->pdev->dev, "Field type %d unsupported.\n", mbus_fmt.field);
		return -EINVAL;
	}

	return 0;
}

static int vidioc_s_fmt_vid_cap(struct file *file, void *priv,
					struct v4l2_format *f)
{
	struct cssp_cam_dev *dev = video_drvdata(file);
	struct vb2_queue *q = &dev->vb_vidq;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_mbus_framefmt mbus_fmt;
	int i = 0, rem;
	u32 bytesperline, height;

	int ret = vidioc_try_fmt_vid_cap(file, priv, f);
	if (ret < 0)
		return ret;

	if (vb2_is_streaming(q)) {
		dev_err(&dev->pdev->dev, "%s device busy\n", __func__);
		return -EBUSY;
	}

	dev->fmt = get_format(f);
	dev->width = f->fmt.pix.width;
	dev->height = f->fmt.pix.height;
	dev->field = f->fmt.pix.field;
	dev->colorspace = f->fmt.pix.colorspace;
	dev->bytesperline = f->fmt.pix.bytesperline;
	dev->sizeimage = f->fmt.pix.sizeimage;

	/* Set the sensor into the new format */
	v4l2_fill_mbus_format(&mbus_fmt, pix, dev->fmt->code);
	v4l2_subdev_call(dev->subdev, video, s_mbus_fmt, &mbus_fmt);

	/* Calculate DMA transfer parameters based on DMA request length */
	bytesperline = dev->bytesperline;
	do {
		rem = bytesperline % dev->dma_req_len;
		if (rem != 0) {
			bytesperline <<= 1;
			i++;
		}
	} while (rem != 0);
	height = dev->height >> i;

	/* Set the EDMA for the new resolution */
	dev->dma_tr_params.a_b_cnt = ACNT(dev->dma_req_len) | BCNT(bytesperline / dev->dma_req_len);
	dev->dma_tr_params.link_bcntrld = BCNTRLD(bytesperline / dev->dma_req_len) | LINK(0xffff);
	dev->dma_tr_params.ccnt = CCNT(height);

	return 0;
}

static int vidioc_reqbufs(struct file *file, void *priv,
			  struct v4l2_requestbuffers *p)
{
	struct cssp_cam_dev *dev = video_drvdata(file);
	return vb2_reqbufs(&dev->vb_vidq, p);
}

static int vidioc_querybuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	struct cssp_cam_dev *dev = video_drvdata(file);
	return vb2_querybuf(&dev->vb_vidq, p);
}

static int vidioc_qbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	struct cssp_cam_dev *dev = video_drvdata(file);
	return vb2_qbuf(&dev->vb_vidq, p);
}

static int vidioc_dqbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	struct cssp_cam_dev *dev = video_drvdata(file);
	return vb2_dqbuf(&dev->vb_vidq, p, file->f_flags & O_NONBLOCK);
}

static int vidioc_streamon(struct file *file, void *priv, enum v4l2_buf_type i)
{
	struct cssp_cam_dev *dev = video_drvdata(file);
	return vb2_streamon(&dev->vb_vidq, i);
}

static int vidioc_streamoff(struct file *file, void *priv, enum v4l2_buf_type i)
{
	struct cssp_cam_dev *dev = video_drvdata(file);
	return vb2_streamoff(&dev->vb_vidq, i);
}

static int vidioc_log_status(struct file *file, void *priv)
{
	struct cssp_cam_dev *dev = video_drvdata(file);

	v4l2_ctrl_handler_log_status(&dev->ctrl_handler, dev->v4l2_dev.name);
	return 0;
}

static int vidioc_enum_input(struct file *file, void *priv,
				struct v4l2_input *inp)
{
	if (inp->index > 0)
		return -EINVAL;

	inp->type = V4L2_INPUT_TYPE_CAMERA;
	sprintf(inp->name, "Camera %u", inp->index);

	return 0;
}

static int vidioc_g_input(struct file *file, void *priv, unsigned int *i)
{
	*i = 0;

	return 0;
}

static int vidioc_s_input(struct file *file, void *priv, unsigned int i)
{
	if (i > 0)
		return -EINVAL;

	return 0;
}

static int vidioc_subscribe_event(struct v4l2_fh *fh,
				const struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_CTRL:
		return v4l2_event_subscribe(fh, sub, 0, NULL);
	default:
		return -EINVAL;
	}
}

static const struct v4l2_ioctl_ops cssp_cam_ioctl_ops = {
	.vidioc_querycap		= vidioc_querycap,
	.vidioc_enum_fmt_vid_cap	= vidioc_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap		= vidioc_g_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap		= vidioc_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap		= vidioc_s_fmt_vid_cap,
	.vidioc_reqbufs			= vidioc_reqbufs,
	.vidioc_querybuf		= vidioc_querybuf,
	.vidioc_qbuf			= vidioc_qbuf,
	.vidioc_dqbuf			= vidioc_dqbuf,
	.vidioc_enum_input		= vidioc_enum_input,
	.vidioc_g_input			= vidioc_g_input,
	.vidioc_s_input			= vidioc_s_input,
	.vidioc_streamon		= vidioc_streamon,
	.vidioc_streamoff		= vidioc_streamoff,
	.vidioc_log_status		= vidioc_log_status,
	.vidioc_subscribe_event		= vidioc_subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,
};


/* ------------------------------------------------------------------
	File operations
   ------------------------------------------------------------------*/

static unsigned int video_poll(struct file *file, struct poll_table_struct *wait)
{
	struct cssp_cam_dev *dev = video_drvdata(file);
	struct v4l2_fh *fh = file->private_data;
	struct vb2_queue *q = &dev->vb_vidq;
	unsigned int res;

	dev_dbg(&dev->pdev->dev, "%s\n", __func__);

	res = vb2_poll(q, file, wait);
	if (v4l2_event_pending(fh))
		res |= POLLPRI;
	else
		poll_wait(file, &fh->wait, wait);
	return res;
}

static int video_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct cssp_cam_dev *dev = video_drvdata(file);
	int ret;

	dev_dbg(&dev->pdev->dev, "mmap called, vma=0x%08lx\n",
			(unsigned long)vma);

	ret = vb2_mmap(&dev->vb_vidq, vma);
	dev_dbg(&dev->pdev->dev, "vma start=0x%08lx, size=%ld, ret=%d\n",
		(unsigned long)vma->vm_start,
		(unsigned long)vma->vm_end - (unsigned long)vma->vm_start,
		ret);
	return ret;
}

static ssize_t video_read(struct file *file, char __user *buf,
		size_t size, loff_t *offset)
{
	struct cssp_cam_dev *cam_dev = video_drvdata(file);

	dev_dbg(&cam_dev->pdev->dev, "%s\n", __func__);

	return vb2_read(&cam_dev->vb_vidq, buf, size, offset,
			file->f_flags & O_NONBLOCK);
}

static int video_close(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct cssp_cam_dev *cam_dev = video_drvdata(file);

	dev_dbg(&cam_dev->pdev->dev, "close called (dev=%s), file %p\n",
		video_device_node_name(vdev), file);

	if (v4l2_fh_is_singular_file(file))
		vb2_queue_release(&cam_dev->vb_vidq);
	return v4l2_fh_release(file);
}

static const struct v4l2_file_operations cssp_cam_fops = {
	.owner		= THIS_MODULE,
	.open		= v4l2_fh_open,
	.release	= video_close,
	.read		= video_read,
	.poll		= video_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= video_mmap,
};


/* ------------------------------------------------------------------
	Driver initialization
   ------------------------------------------------------------------*/

static struct video_device cssp_cam_template = {
	.name		= "cssp_camera",
	.fops		= &cssp_cam_fops,
	.ioctl_ops	= &cssp_cam_ioctl_ops,
	.minor		= -1,
	.release	= video_device_release,
};

static int video_probe(struct cssp_cam_dev *cam_dev)
{
	struct video_device *vfd;
	struct v4l2_ctrl_handler *hdl;
	struct vb2_queue *q;
	int ret = 0;

	snprintf(cam_dev->v4l2_dev.name, sizeof(cam_dev->v4l2_dev.name),
			"%s-%03d", "cssp_camera", 0);
	ret = v4l2_device_register(NULL, &cam_dev->v4l2_dev);
	if (ret)
		goto free_dev;

	cam_dev->fmt = &formats[0];
	cam_dev->width = VGA_WIDTH;
	cam_dev->height = VGA_HEIGHT;
	cam_dev->sizeimage = VGA_WIDTH * VGA_HEIGHT * BYTES_PER_PIXEL;
	hdl = &cam_dev->ctrl_handler;
	v4l2_ctrl_handler_init(hdl, 0);

	if (hdl->error) {
		ret = hdl->error;
		goto unreg_dev;
	}
	cam_dev->v4l2_dev.ctrl_handler = hdl;

	/* initialize locks */
	spin_lock_init(&cam_dev->slock);

	/* initialize queue */
	q = &cam_dev->vb_vidq;
	memset(q, 0, sizeof(cam_dev->vb_vidq));
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_READ;
	q->drv_priv = cam_dev;
	q->buf_struct_size = sizeof(struct cssp_cam_buffer);
	q->ops = &cssp_cam_video_qops;
	q->mem_ops = &vb2_dma_contig_memops;

	ret = vb2_queue_init(q);
	if (ret != 0) {
		goto unreg_dev;
	}

	mutex_init(&cam_dev->mutex);

	/* init video dma queues */
	INIT_LIST_HEAD(&cam_dev->vidq.active);

	ret = -ENOMEM;
	vfd = video_device_alloc();
	if (!vfd)
		goto unreg_dev;

	*vfd = cssp_cam_template;
	vfd->v4l2_dev = &cam_dev->v4l2_dev;
	set_bit(V4L2_FL_USE_FH_PRIO, &vfd->flags);

	/*
	 * Provide a mutex to v4l2 core. It will be used to protect
	 * all fops and v4l2 ioctls.
	 */
	vfd->lock = &cam_dev->mutex;

	ret = video_register_device(vfd, VFL_TYPE_GRABBER, video_nr);
	if (ret < 0)
		goto rel_vdev;

	video_set_drvdata(vfd, cam_dev);

	if (video_nr != -1)
		video_nr++;

	cam_dev->vdev = vfd;
	v4l2_info(&cam_dev->v4l2_dev, "V4L2 device registered as %s\n",
	video_device_node_name(vfd));

	return 0;

rel_vdev:
	video_device_release(vfd);
unreg_dev:
	v4l2_ctrl_handler_free(hdl);
	v4l2_device_unregister(&cam_dev->v4l2_dev);
free_dev:
	return ret;
}

static int video_remove(struct cssp_cam_dev *cam_dev)
{
	if (cam_dev->dma_cont_ctx != NULL)
		vb2_dma_contig_cleanup_ctx(cam_dev->dma_cont_ctx);

	v4l2_info(&cam_dev->v4l2_dev, "unregistering %s\n",
			video_device_node_name(cam_dev->vdev));
	video_unregister_device(cam_dev->vdev);
	v4l2_device_unregister(&cam_dev->v4l2_dev);
	v4l2_ctrl_handler_free(&cam_dev->ctrl_handler);

	return 0;
}

#ifdef CONFIG_OF

static const struct of_device_id cssp_camera_of_match[] = {
	{ .compatible = "cssp-camera", },
	{ },
};
MODULE_DEVICE_TABLE(of, cssp_camera_of_match);

struct cssp_cam_platform_data *
of_get_cssp_platform_data(struct platform_device *pdev)
{
	struct cssp_cam_platform_data *pdata;
	struct cssp_cam_platform_data_storage *pstore;
	struct device *dev = &pdev->dev;
	struct device_node *np, *nps, *npc, *npa;
	struct i2c_adapter *adap;
	struct of_phandle_args args;
	u32 val, valarr[9];
	int ret, found, gpio_orientation_pin;

	np = dev->of_node;
	nps = NULL;
	npc = NULL;
	npa = NULL;
	adap = NULL;
	memset(&args, 0, sizeof(args));
	pstore = NULL;
	gpio_orientation_pin = -1;

	if (np == NULL) {
		dev_err(dev, "No OF device node\n");
		goto err_fail;
	}
	pstore = devm_kzalloc(dev, sizeof(*pstore), GFP_KERNEL);
	if (pstore == NULL) {
		dev_err(dev, "Failed to allocate platform data storage\n");
		goto err_fail;
	}

	/* link the structures together */
	pdata = &pstore->pdata;
	pdata->cam_i2c_board_info = &pstore->i2c_camera;
	pstore->i2c_camera.platform_data = &pstore->camera_link;
	pstore->camera_link.priv = &pstore->mt9t111_cam_info;

	ret = of_property_read_string(np, "cssp,camera-clk-name",
			&pdata->cam_clk_name);
	if (ret != 0) {
		dev_err(dev, "No cssp,camera-clk-name property\n");
		goto err_fail;
	}

	ret = of_property_read_u32(np, "cssp,camera-clk-rate",
			&pdata->cam_clk_rate);
	if (ret != 0) {
		dev_err(dev, "no cssp,camera-clk-rate property\n");
		goto err_fail;
	}

	/* we don't use the dma accessors, but we use the format */
	ret = of_parse_phandle_with_args(np, "dmas", "#dma-cells", 0, &args);
	if (ret != 0 || args.args_count < 1) {
		dev_err(dev, "No valid dmas property\n");
		goto err_fail;
	}
	pdata->dma_ch = args.args[0];
	/* release ref */
	of_node_put(args.np);
	args.np = NULL;

	/* can possibly fail */
	pdata->gpio_reset_pin = of_get_named_gpio(np,
			"reset-gpio", 0);

	gpio_orientation_pin = of_get_named_gpio(np,
			"orientation-gpio", 0);

	/* get sensor node */
	nps = of_get_child_by_name(np, "cssp,sensor");
	if (nps == NULL) {
		dev_err(dev, "Failed to get sensor node\n");
		goto err_fail;
	}

	/* find the i2c adapter number */
	npa = of_parse_phandle(nps, "i2c-adapter", 0);
	if (np == NULL) {
		dev_err(dev, "Failed to get i2c-adapter property");
		goto err_fail;
	}
	adap = of_find_i2c_adapter_by_node(npa);
	if (adap == NULL) {
		dev_err(dev, "Failed to find i2c-adapter");
		goto err_fail;
	}
	pstore->camera_link.i2c_adapter_id = adap->nr;

	/* release i2c adapter device ref */
	put_device(&adap->dev);
	adap = NULL;

	/* release i2c adapter device node */
	of_node_put(npa);
	npa = NULL;

	/* now find the sensor node */
	for_each_available_child_of_node(nps, npc) {
		/* we only support a single sensor for now */
		if (of_device_is_compatible(npc, "aptina,mt9t112")) {
			strncpy(pstore->i2c_camera.type, "mt9t112",
					sizeof(pstore->i2c_camera.type));
			found = 1;
			break;
		}
	}

	if (!found) {
		dev_err(dev, "Failed to find sensor node");
		goto err_fail;
	}

	if (of_property_read_u32(npc, "reg", &val) != 0) {
		dev_err(dev, "Could not get sensor reg property\n");
		goto err_fail;
	}
	pstore->i2c_camera.addr = val;

	if (of_property_read_u32_array(npc, "pll-divider", valarr, ARRAY_SIZE(valarr)) != 0) {
		dev_err(dev, "Could not get pll-divider property\n");
		goto err_fail;
	}
	pstore->mt9t111_cam_info.divider.m = valarr[0];
	pstore->mt9t111_cam_info.divider.n = valarr[1];
	pstore->mt9t111_cam_info.divider.p1 = valarr[2];
	pstore->mt9t111_cam_info.divider.p2 = valarr[3];
	pstore->mt9t111_cam_info.divider.p3 = valarr[4];
	pstore->mt9t111_cam_info.divider.p4 = valarr[5];
	pstore->mt9t111_cam_info.divider.p5 = valarr[6];
	pstore->mt9t111_cam_info.divider.p6 = valarr[7];
	pstore->mt9t111_cam_info.divider.p7 = valarr[8];

	if (of_property_read_u32(npc, "flags", &val) != 0) {
		dev_err(dev, "Could not get sensor flags property\n");
		goto err_fail;
	}

	/* read orientation gpio and set/clr MT9T112_FLAG_VFLIP */
	if (gpio_is_valid(gpio_orientation_pin)) {
		ret = gpio_request(gpio_orientation_pin, "camera orientation");
		if (ret != 0) {
			dev_err(dev, "Could not gpio_request orientation\n");
			goto err_fail;
		}
		ret = gpio_direction_input(gpio_orientation_pin);
		if (ret != 0) {
			dev_err(dev, "Could not set orientation to input\n");
			gpio_free(gpio_orientation_pin);
			goto err_fail;
		}
		ret = gpio_get_value(gpio_orientation_pin);
		if (ret < 0) {
			dev_err(dev, "Could not get orientation value\n");
			gpio_free(gpio_orientation_pin);
			goto err_fail;
		}
		gpio_free(gpio_orientation_pin);

		/* set orientation flag */

		/*
		 * But the driver in mainline doesn't support flip
		 * Commented out for now...
		 *
		 *	if (ret)
		 *		val |= MT9T112_FLAG_VFLIP;
		 *	else
		 *		val &= ~MT9T112_FLAG_VFLIP
		 */
	}
	pstore->mt9t111_cam_info.flags = val;

	/* release refs */

	of_node_put(npc);
	npc = NULL;
	of_node_put(nps);
	nps = NULL;

	return pdata;

err_fail:
	/* NULL is handled as a NOP */
	of_node_put(nps);
	of_node_put(npc);
	of_node_put(npa);

	/* release adapter */
	if (adap != NULL)
		put_device(&adap->dev);

	/* free memory (even if it will be automatically freed it's good practice) */
	if (pstore != NULL)
		devm_kfree(dev, pstore);

	return NULL;
}

#else
struct cssp_cam_platform_data *of_get_cssp_platform_data(struct platform_device *pdev)
{
	return NULL;
}
#endif

static int cssp_cam_probe(struct platform_device *pdev)
{
	struct cssp_cam_dev *cam_dev;
	struct cssp_cam_platform_data *pdata;
	struct pinctrl *pinctrl;
	int ret = 0, use_of_pdata = 0;

	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(pinctrl))
		dev_warn(&pdev->dev,
			"pins are not configured from the driver\n");

	pdata = pdev->dev.platform_data;

	/* if not found, try DT */
	if (pdata == NULL) {
		pdata = of_get_cssp_platform_data(pdev);
		use_of_pdata = pdata != NULL;
	}

	if (pdata == NULL) {
		dev_err(&pdev->dev, "missing platform data\n");
		return -ENODEV;
	}
	pdev->dev.platform_data = pdata;

	if (pdata->cam_i2c_board_info == NULL) {
		dev_err(&pdev->dev, "missing camera i2c board info\n");
		return -ENODEV;
	}

	cam_dev = kzalloc(sizeof(*cam_dev), GFP_KERNEL);
	if (!cam_dev)
		return -ENOMEM;

	/* keep the pointer of the of pdata storage */
	if (use_of_pdata)
		cam_dev->pstore = to_cssp_platform_data_storage(pdata);

	cam_dev->pdev = pdev;
	platform_set_drvdata(pdev, cam_dev);

	cam_dev->camera_board_info = pdata->cam_i2c_board_info;

	cam_dev->camera_clk = clk_get(&pdev->dev, pdata->cam_clk_name);
	if (IS_ERR(cam_dev->camera_clk)) {
		ret = PTR_ERR(cam_dev->camera_clk);
		dev_err(&pdev->dev, "cannot clk_get %s\n", pdata->cam_clk_name);
		goto fail0;
	}

	/* 32MHz */
	ret = clk_set_rate(cam_dev->camera_clk, pdata->cam_clk_rate);
	if (ret != 0) {
		dev_err(&pdev->dev, "failed to set clk rate\n");
		goto fail1;
	}

	if (clk_get_rate(cam_dev->camera_clk) != pdata->cam_clk_rate) {
		dev_err(&pdev->dev, "No accurate clock found\n");
		goto fail1;
	}

	clk_prepare(cam_dev->camera_clk);

	ret = configure_cssp(cam_dev);
	if (ret) {
		dev_err(&pdev->dev, "configure_cssp failed\n");
		goto fail1;
	}

	ret = configure_edma(cam_dev);
	if (ret) {
		dev_err(&pdev->dev, "configure_dma failed\n");
		goto fail2;
	}

	cam_dev->mode = FMT_2X8_EN | PCLK_POL | HS_EN;	// falling edge
	if (cam_dev->rev > 3)
		cam_dev->mode |= LDR_EN;

	ret = configure_camera_sensor(cam_dev);
	if (ret) {
		dev_err(&pdev->dev, "camera sensor configuration failed\n");
		goto fail3;
	}

	cam_dev->dma_cont_ctx = vb2_dma_contig_init_ctx(&pdev->dev);
	if (IS_ERR(cam_dev->dma_cont_ctx)) {
		ret = PTR_ERR(cam_dev->dma_cont_ctx);
		goto fail3;
	}

	ret = video_probe(cam_dev);
	if (ret)
		goto fail4;

	dev_err(&pdev->dev, "Loaded OK.\n");

	return 0;

fail4:
	vb2_dma_contig_cleanup_ctx(cam_dev->dma_cont_ctx);

fail3:
	edma_free_channel(cam_dev->dma_ch);
fail2:
	if (gpio_is_valid(cam_dev->reset_pin))
		gpio_free(cam_dev->reset_pin);

	iounmap(cam_dev->reg_base_virt);
	release_mem_region(cam_dev->reg_base_phys, cam_dev->reg_size);

fail1:
	clk_put(cam_dev->camera_clk);

fail0:
	kfree(cam_dev);

	return ret;
}

static int cssp_cam_remove(struct platform_device *pdev)
{
	struct cssp_cam_dev *cam = platform_get_drvdata(pdev);

	iounmap(cam->reg_base_virt);

	release_mem_region(cam->reg_base_phys, cam->reg_size);

	if (gpio_is_valid(cam->reset_pin))
		gpio_free(cam->reset_pin);

	if (cam->dma_ch)
		edma_free_channel(cam->dma_ch);

	video_remove(cam);

	clk_put(cam->camera_clk);

	kfree(cam);

	dev_info(&pdev->dev, "removed\n");

	return 0;
}


static struct platform_driver cssp_cam_driver = {
	.probe		= cssp_cam_probe,
	.remove		= cssp_cam_remove,
	.driver		= {
		.name	= "cssp-camera",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(cssp_camera_of_match),
	},
};

module_platform_driver(cssp_cam_driver);


/*
 * Macros sets license, author and description
 */
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Dan Aizenstros, Damian Eppel, Przemek Szewczyk");
MODULE_DESCRIPTION("QuickLogic Camera Interface driver");
