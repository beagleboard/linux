/*
 * drivers/media/video/omap/camera_core.c
 *
 * Copyright (C) 2004 Texas Instruments, Inc.
 *
 * Video-for-Linux (Version 2) camera capture driver for
 * the OMAP H2 and H3 camera controller.
 *
 * Adapted from omap24xx driver written by Andy Lowe (source@mvista.com)
 * Copyright (C) 2003-2004 MontaVista Software, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * History:
 *   27/03/05   Vladimir Barinov - Added support for power management
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/version.h>
#include <linux/dma-mapping.h>
#include <linux/fb.h>

#include <media/v4l2-common.h>

#include <asm/io.h>

#include "camera_hw_if.h"
#include "camera_core.h"

#define OMAP1CAM_VERSION KERNEL_VERSION(0, 0, 0)

static struct camera_device *camera_dev;
static void camera_core_sgdma_process(struct camera_device *cam);

/* Module parameters */
static int video_nr = -1;	/* Video device minor (-1 ==> auto assign) */

/* Maximum amount of memory to use for capture buffers.
 * Default is 4800KB, enough to double-buffer SXGA.
 */
static int capture_mem = 1280 * 960 * 2 * 2;

/* Size of video overlay framebuffer. This determines the maximum image size
 * that can be previewed. Default is 600KB, enough for sxga.
 */
static int overlay_mem = 640 * 480 * 2;

/* Enable the external sensor interface. Try to negotiate interface
 * parameters with the sensor and start using the new ones. The calls
 * to sensor_if_enable and sensor_if_disable do not need to be balanced.
 */
static int camera_sensor_if_enable(struct camera_device *cam)
{
	int rval;
	struct v4l2_ifparm p;

	rval = vidioc_int_g_ifparm(cam->sdev, &p);
	if (rval) {
		dev_err(cam->dev, "vidioc_int_g_ifparm failed with %d\n", rval);
		return rval;
	}

	cam->if_type = p.if_type;

	switch (p.if_type) {
	case V4L2_IF_TYPE_BT656:
		cam->if_u.bt656.xclk =
		    cam->cam_hardware->set_xclk(cam->if_u.bt656.xclk,
						cam->hardware_data);
		break;
	default:
		/* FIXME: how about other interfaces? */
		dev_err(cam->dev, "interface type %d not supported\n",
			p.if_type);
		return -EINVAL;
	}

	return 0;
}

static void camera_sensor_if_disable(const struct camera_device *cam)
{
	switch (cam->if_type) {
	case V4L2_IF_TYPE_BT656:
		break;
	}
}

/* Initialize the sensor hardware. */
static int camera_sensor_init(struct camera_device *cam)
{
	int err = 0;
	struct v4l2_int_device *sdev = cam->sdev;

	/* Enable the xclk output.  The sensor may (and does, in the case of
	 * the OV9640) require an xclk input in order for its initialization
	 * routine to work.
	 */

	/* Choose an arbitrary xclk frequency */
	cam->if_u.bt656.xclk = 21000000;

	cam->if_u.bt656.xclk = cam->cam_hardware->set_xclk(cam->if_u.bt656.xclk,
							   cam->hardware_data);

	err = camera_sensor_if_enable(cam);
	if (err) {
		dev_err(cam->dev, "sensor interface could not be enabled at "
			"initialization, %d\n", err);
		cam->sdev = NULL;
		goto out;
	}

	/* Power up sensor during sensor initialization */
	vidioc_int_s_power(sdev, 1);

	err = vidioc_int_dev_init(sdev);
	if (err) {
		dev_err(cam->dev, "cannot initialize sensor, error %d\n", err);
		/* Sensor initialization failed --- it's nonexistent to us! */
		cam->sdev = NULL;
		goto out;
	}

	dev_info(cam->dev, "sensor is %s\n", sdev->name);

out:
	camera_sensor_if_disable(cam);

	vidioc_int_s_power(sdev, 0);

	return err;
}

static void camera_sensor_exit(struct camera_device *cam)
{
	if (cam->sdev)
		vidioc_int_dev_exit(cam->sdev);
}

/* DMA completion routine for the scatter-gather DMA fragments.
 * This function is called when a scatter DMA fragment is completed
 */
static void camera_core_callback_sgdma(void *arg1, void *arg2)
{
	struct camera_device *cam = (struct camera_device *)arg1;
	int sgslot = (int)arg2;

	struct sgdma_state *sgdma;

	spin_lock(&cam->sg_lock);
	sgdma = cam->sgdma + sgslot;
	if (!sgdma->queued_sglist) {
		spin_unlock(&cam->sg_lock);
		dev_err(cam->dev, "SGDMA completed when none queued\n");
		return;
	}
	if (!--sgdma->queued_sglist) {
		/* Queue for this sglist is empty so check whether transfer
		 * of the frame has been completed
		 */
		if (sgdma->next_sglist == sgdma->sglen) {
			dma_callback_t callback = sgdma->callback;
			void *arg = sgdma->arg;
			/* All done with this sglist */
			cam->free_sgdma++;
			if (callback) {
				spin_unlock(&cam->sg_lock);
				(*callback) (cam, arg);
				camera_core_sgdma_process(cam);
				return;
			}
		}
	}
	spin_unlock(&cam->sg_lock);
	camera_core_sgdma_process(cam);

	return;
}

static void camera_core_sgdma_init(struct camera_device *cam)
{
	int sg;

	/* Initialize the underlying camera DMA */
	cam->cam_hardware->init_dma(cam->hardware_data);
	spin_lock_init(&cam->sg_lock);

	cam->free_sgdma = NUM_SG_DMA;
	cam->next_sgdma = 0;
	for (sg = 0; sg < NUM_SG_DMA; sg++) {
		cam->sgdma[sg].sglen = 0;
		cam->sgdma[sg].next_sglist = 0;
		cam->sgdma[sg].queued_sglist = 0;
		cam->sgdma[sg].csr = 0;
		cam->sgdma[sg].callback = NULL;
		cam->sgdma[sg].arg = NULL;
	}
}

/* Process the scatter-gather DMA queue by starting queued transfers
 * This function is called to program the DMA to start the transfer of an image.
 */
static void camera_core_sgdma_process(struct camera_device *cam)
{
	unsigned long irqflags;
	int queued_sgdma, sgslot;
	struct sgdma_state *sgdma;
	const struct scatterlist *sglist;

	spin_lock_irqsave(&cam->sg_lock, irqflags);
	if (1 == cam->in_use) {
		spin_unlock_irqrestore(&cam->sg_lock, irqflags);
		return;
	}
	cam->in_use = 1;
	spin_unlock_irqrestore(&cam->sg_lock, irqflags);

	queued_sgdma = NUM_SG_DMA - cam->free_sgdma;
	sgslot = (cam->next_sgdma + cam->free_sgdma) % (NUM_SG_DMA);
	while (queued_sgdma > 0) {
		sgdma = cam->sgdma + sgslot;
		while (sgdma->next_sglist < sgdma->sglen) {
			sglist = sgdma->sglist + sgdma->next_sglist;
			if (cam->cam_hardware->
			    start_dma(sgdma, camera_core_callback_sgdma,
				      (void *)cam, (void *)sgslot,
				      cam->hardware_data)) {
				/* DMA start failed */
				cam->in_use = 0;
				return;
			} else {
				/* DMA start successful */
				sgdma->next_sglist++;
				sgdma->queued_sglist++;
			}
		}
		queued_sgdma--;
		sgslot = (sgslot + 1) % (NUM_SG_DMA);
	}

	cam->in_use = 0;
}

/* Queue a scatter-gather DMA transfer from the camera to memory.
 * Returns zero if the transfer was successfully queued, or
 * non-zero if all of the scatter-gather slots are already in use.
 */
static int camera_core_sgdma_queue(struct camera_device *cam,
				   const struct scatterlist *sglist,
				   int sglen, dma_callback_t callback,
				   void *arg)
{
	unsigned long irqflags;
	struct sgdma_state *sgdma;

	if ((sglen < 0) || ((sglen > 0) & !sglist))
		return -EINVAL;

	spin_lock_irqsave(&cam->sg_lock, irqflags);

	if (!cam->free_sgdma) {
		spin_unlock_irqrestore(&cam->sg_lock, irqflags);
		return -EBUSY;
	}

	sgdma = cam->sgdma + cam->next_sgdma;

	sgdma->sglist = sglist;
	sgdma->sglen = sglen;
	sgdma->next_sglist = 0;
	sgdma->queued_sglist = 0;
	sgdma->csr = 0;
	sgdma->callback = callback;
	sgdma->arg = arg;

	cam->next_sgdma = (cam->next_sgdma + 1) % (NUM_SG_DMA);
	cam->free_sgdma--;

	spin_unlock_irqrestore(&cam->sg_lock, irqflags);

	camera_core_sgdma_process(cam);

	return 0;
}

/* -------------------overlay routines ------------------------------
 * Callback routine for overlay DMA completion. We just start another DMA
 * transfer unless overlay has been turned off
 */
static void camera_core_overlay_callback(void *arg1, void *arg)
{
	struct camera_device *cam = (struct camera_device *)arg1;
	int err;
	unsigned long irqflags;
	int i, j;
	int count, index;
	unsigned char *fb_buf =
	    phys_to_virt((unsigned long)camera_dev->fbuf.base);

	spin_lock_irqsave(&cam->overlay_lock, irqflags);

	if (!cam->previewing || cam->overlay_cnt == 0) {
		spin_unlock_irqrestore(&cam->overlay_lock, irqflags);
		return;
	}

	--cam->overlay_cnt;
	sg_dma_address(&cam->overlay_sglist) = cam->overlay_base_phys;
	sg_dma_len(&cam->overlay_sglist) = cam->pix.sizeimage;

	count = 0;
	j = ((cam->pix.width - 1) * cam->fbuf.fmt.bytesperline);
	for (i = 0; i < cam->pix.sizeimage; i += cam->pix.bytesperline) {
		for (index = 0; index < cam->pix.bytesperline; index++) {
			fb_buf[j] = *(((unsigned char *)cam->overlay_base) +
				      i + index);
			index++;
			fb_buf[j + 1] =
			    *(((unsigned char *)cam->overlay_base) + i + index);
			j = j - cam->fbuf.fmt.bytesperline;
		}
		count += 2;
		j = ((cam->pix.width - 1) * cam->fbuf.fmt.bytesperline) + count;
	}

	while (cam->overlay_cnt < 2) {
		err = camera_core_sgdma_queue(cam, &cam->overlay_sglist, 1,
					      camera_core_overlay_callback,
					      NULL);
		if (err)
			break;
		++cam->overlay_cnt;
	}

	spin_unlock_irqrestore(&cam->overlay_lock, irqflags);

}

static void camera_core_start_overlay(struct camera_device *cam)
{
	int err;
	unsigned long irqflags;

	if (!cam->previewing)
		return;

	spin_lock_irqsave(&cam->overlay_lock, irqflags);

	sg_dma_address(&cam->overlay_sglist) = cam->overlay_base_phys;
	sg_dma_len(&cam->overlay_sglist) = cam->pix.sizeimage;
	while (cam->overlay_cnt < 2) {
		err = camera_core_sgdma_queue(cam, &cam->overlay_sglist, 1,
					      camera_core_overlay_callback,
					      NULL);
		if (err)
			break;
		++cam->overlay_cnt;
	}

	spin_unlock_irqrestore(&cam->overlay_lock, irqflags);
}

/* ------------------ videobuf_queue_ops --------------------------------
 * This routine is called from interrupt context when a scatter-gather DMA
 * transfer of a videobuf_buffer completes.
 */
static void camera_core_vbq_complete(void *arg1, void *arg)
{
	struct camera_device *cam = (struct camera_device *)arg1;
	struct videobuf_buffer *vb = (struct videobuf_buffer *)arg;

	spin_lock(&cam->vbq_lock);

	do_gettimeofday(&vb->ts);
	vb->field_count = cam->field_count;
	cam->field_count += 2;
	vb->state = STATE_DONE;

	wake_up(&vb->done);

	spin_unlock(&cam->vbq_lock);
}

static void camera_core_vbq_release(struct videobuf_queue *q,
				    struct videobuf_buffer *vb)
{
	struct videobuf_dmabuf *dma = videobuf_to_dma(vb);
	videobuf_waiton(vb, 0, 0);
	videobuf_dma_unmap(q, dma);
	videobuf_dma_free(dma);

	vb->state = STATE_NEEDS_INIT;
}

/* Limit the number of available kernel image capture buffers based on the
 * number requested, the currently selected image size, and the maximum
 * amount of memory permitted for kernel capture buffers.
 */
static int camera_core_vbq_setup(struct videobuf_queue *q, unsigned int *cnt,
				 unsigned int *size)
{
	struct camera_fh *fh = q->priv_data;
	struct camera_device *cam = fh->cam;

	if (*cnt <= 0)
		*cnt = VIDEO_MAX_FRAME;	/* Supply a default number of buffers */

	if (*cnt > VIDEO_MAX_FRAME)
		*cnt = VIDEO_MAX_FRAME;

	spin_lock(&cam->img_lock);
	*size = cam->pix.sizeimage;
	spin_unlock(&cam->img_lock);

	while (*size * *cnt > capture_mem)
		(*cnt)--;

	return 0;
}

static int camera_core_vbq_prepare(struct videobuf_queue *q,
				   struct videobuf_buffer *vb,
				   enum v4l2_field field)
{
	struct camera_fh *fh = q->priv_data;
	struct camera_device *cam = fh->cam;
	int err = 0;

	spin_lock(&cam->img_lock);
	if (cam->pix.sizeimage > vb->bsize) {
		spin_unlock(&cam->img_lock);
		return -EINVAL;
	}
	vb->size = cam->pix.sizeimage;
	vb->width = cam->pix.width;
	vb->height = cam->pix.height;
	vb->field = field;
	spin_unlock(&cam->img_lock);

	if (vb->state == STATE_NEEDS_INIT)
		err = videobuf_iolock(q, vb, NULL);

	if (!err)
		vb->state = STATE_PREPARED;
	else
		camera_core_vbq_release(q, vb);

	return err;
}

static void camera_core_vbq_queue(struct videobuf_queue *q,
				  struct videobuf_buffer *vb)
{
	struct videobuf_dmabuf *dma = videobuf_to_dma(vb);
	struct camera_fh *fh = q->priv_data;
	struct camera_device *cam = fh->cam;
	enum videobuf_state state = vb->state;
	int err;

	vb->state = STATE_QUEUED;
	err = camera_core_sgdma_queue(cam, dma->sglist, dma->sglen,
				      camera_core_vbq_complete, vb);
	if (err) {
		/* Oops.  We're not supposed to get any errors here.  The only
		 * way we could get an error is if we ran out of scatter-gather
		 * DMA slots, but we are supposed to have at least as many
		 * scatter-gather DMA slots as video buffers so that can't
		 * happen.
		 */
		dev_dbg(cam->dev, "Failed to queue a video buffer for SGDMA\n");
		vb->state = state;
	}
}

/* IOCTL interface. */
static int vidioc_querycap(struct file *file, void *fh,
			   struct v4l2_capability *cap)
{
	struct camera_fh *ofh = fh;
	struct camera_device *cam = ofh->cam;

	strlcpy(cap->driver, CAM_NAME, sizeof(cap->driver));
	strlcpy(cap->card, cam->vfd->name, sizeof(cap->card));
	cap->version = OMAP1CAM_VERSION;
	cap->capabilities =
	    V4L2_CAP_VIDEO_CAPTURE |
	    V4L2_CAP_VIDEO_OVERLAY | V4L2_CAP_READWRITE | V4L2_CAP_STREAMING;

	return 0;
}

static int vidioc_enum_fmt_cap(struct file *file, void *fh,
			       struct v4l2_fmtdesc *f)
{
	struct camera_fh *ofh = fh;
	struct camera_device *cam = ofh->cam;

	return vidioc_int_enum_fmt_cap(cam->sdev, f);
}

static int vidioc_g_fmt_cap(struct file *file, void *fh, struct v4l2_format *f)
{
	struct camera_fh *ofh = fh;
	struct camera_device *cam = ofh->cam;

	/* Get the current format */
	memset(&f->fmt.pix, 0, sizeof(f->fmt.pix));
	f->fmt.pix = cam->pix;

	return 0;
}

static int vidioc_s_fmt_cap(struct file *file, void *fh, struct v4l2_format *f)
{
	struct camera_fh *ofh = fh;
	struct camera_device *cam = ofh->cam;
	int rval = 0;

	vidioc_int_try_fmt_cap(cam->sdev, f);

	cam->pix = f->fmt.pix;

	rval = vidioc_int_s_fmt_cap(cam->sdev, f);
	camera_sensor_if_enable(cam);

	return rval;
}

static int vidioc_try_fmt_cap(struct file *file, void *fh,
			      struct v4l2_format *f)
{
	struct camera_fh *ofh = fh;
	struct camera_device *cam = ofh->cam;

	return vidioc_int_try_fmt_cap(cam->sdev, f);
}

static int vidioc_reqbufs(struct file *file, void *fh,
			  struct v4l2_requestbuffers *b)
{
	struct camera_fh *ofh = fh;

	return videobuf_reqbufs(&ofh->vbq, b);
}

static int vidioc_querybuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct camera_fh *ofh = fh;

	return videobuf_querybuf(&ofh->vbq, b);
}

static int vidioc_qbuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct camera_fh *ofh = fh;

	return videobuf_qbuf(&ofh->vbq, b);
}

static int vidioc_dqbuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct camera_fh *ofh = fh;

	return videobuf_dqbuf(&ofh->vbq, b, file->f_flags & O_NONBLOCK);
}

static int vidioc_streamon(struct file *file, void *fh, enum v4l2_buf_type i)
{
	struct camera_fh *ofh = fh;
	struct camera_device *cam = ofh->cam;

	spin_lock(&cam->img_lock);

	if (cam->streaming || cam->reading) {
		spin_unlock(&cam->img_lock);
		return -EBUSY;
	} else
		cam->streaming = ofh;
	/* FIXME: start camera interface */

	spin_unlock(&cam->img_lock);

	return videobuf_streamon(&ofh->vbq);
}

static int vidioc_streamoff(struct file *file, void *fh, enum v4l2_buf_type i)
{
	struct camera_fh *ofh = fh;
	struct camera_device *cam = ofh->cam;
	int err;

	err = videobuf_streamoff(&ofh->vbq);
	if (err < 0)
		return err;

	spin_lock(&cam->img_lock);
	if (cam->streaming == ofh)
		cam->streaming = NULL;
	/* FIXME: stop camera interface */

	spin_unlock(&cam->img_lock);
	return 0;
}

static int vidioc_enum_input(struct file *file, void *fh,
			     struct v4l2_input *inp)
{
	if (inp->index > 0)
		return -EINVAL;

	strlcpy(inp->name, "camera", sizeof(inp->name));
	inp->type = V4L2_INPUT_TYPE_CAMERA;

	return 0;
}

static int vidioc_g_input(struct file *file, void *fh, unsigned int *i)
{
	*i = 0;

	return 0;
}

static int vidioc_s_input(struct file *file, void *fh, unsigned int i)
{
	if (i > 0)
		return -EINVAL;

	return 0;
}

static int vidioc_queryctrl(struct file *file, void *fh,
			    struct v4l2_queryctrl *a)
{
	struct camera_fh *ofh = fh;
	struct camera_device *cam = ofh->cam;

	return vidioc_int_queryctrl(cam->sdev, a);
}

static int vidioc_g_ctrl(struct file *file, void *fh, struct v4l2_control *a)
{
	struct camera_fh *ofh = fh;
	struct camera_device *cam = ofh->cam;

	return vidioc_int_g_ctrl(cam->sdev, a);
}

static int vidioc_s_ctrl(struct file *file, void *fh, struct v4l2_control *a)
{
	struct camera_fh *ofh = fh;
	struct camera_device *cam = ofh->cam;

	return vidioc_int_s_ctrl(cam->sdev, a);
}

static int vidioc_g_fbuf(struct file *file, void *fh,
			 struct v4l2_framebuffer *a)
{
	struct camera_fh *ofh = fh;
	struct camera_device *cam = ofh->cam;

	spin_lock(&cam->img_lock);
	*a = cam->fbuf;
	spin_unlock(&cam->img_lock);

	return 0;
}

static int vidioc_s_fbuf(struct file *file, void *fh,
			 struct v4l2_framebuffer *a)
{
	struct camera_fh *ofh = fh;
	struct camera_device *cam = ofh->cam;

	spin_lock(&cam->img_lock);
	if (cam->previewing) {
		spin_unlock(&cam->img_lock);
		return -EBUSY;
	}
	cam->fbuf.base = a->base;
	cam->fbuf.fmt = a->fmt;

	spin_unlock(&cam->img_lock);
	return 0;
}

static int vidioc_overlay(struct file *file, void *fh, unsigned int i)
{
	struct camera_fh *ofh = fh;
	struct camera_device *cam = ofh->cam;
	int enable = i;

	/* Check whether the capture format and
	 * the display format matches
	 * return failure if they are different
	 */
	if (cam->pix.pixelformat != cam->fbuf.fmt.pixelformat)
		return -EINVAL;

	/* If the camera image size is greater
	 * than LCD size return failure
	 */
	if ((cam->pix.width > cam->fbuf.fmt.height) ||
	    (cam->pix.height > cam->fbuf.fmt.width))
		return -EINVAL;

	if (!cam->previewing && enable) {
		cam->previewing = fh;
		cam->overlay_cnt = 0;
		camera_core_start_overlay(cam);
	} else if (!enable)
		cam->previewing = NULL;

	return 0;
}

/* File operations */
static unsigned int camera_core_poll(struct file *file,
				     struct poll_table_struct *wait)
{
	return -EINVAL;
}

/* Callback routine for read DMA completion. We just start another DMA
 * transfer unless overlay has been turned off
 */
static void camera_core_capture_callback(void *arg1, void *arg)
{
	struct camera_device *cam = (struct camera_device *)arg1;
	int err;
	unsigned long irqflags;
	static int done = 0;

	spin_lock_irqsave(&cam->capture_lock, irqflags);
	if (!cam->reading) {
		done = 0;
		cam->capture_started = 0;
		spin_unlock_irqrestore(&cam->capture_lock, irqflags);
		return;
	}

	if (done < 14) {
		++done;
		sg_dma_address(&cam->capture_sglist) = cam->capture_base_phys;
		sg_dma_len(&cam->capture_sglist) = cam->pix.sizeimage;
		err = camera_core_sgdma_queue(cam, &cam->capture_sglist, 1,
					      camera_core_capture_callback,
					      NULL);
	} else {
		cam->capture_completed = 1;
		if (cam->reading) {
			/* Wake up any process which are waiting for the
			 * DMA to complete
			 */
			wake_up_interruptible(&camera_dev->new_video_frame);
			sg_dma_address(&cam->capture_sglist) =
			    cam->capture_base_phys;
			sg_dma_len(&cam->capture_sglist) = cam->pix.sizeimage;
			err =
			   camera_core_sgdma_queue(cam, &cam->capture_sglist,
						   1,
						   camera_core_capture_callback,
						   NULL);
		}
	}

	spin_unlock_irqrestore(&cam->capture_lock, irqflags);
}

static ssize_t camera_core_read(struct file *file, char *data, size_t count,
				loff_t *ppos)
{
	struct camera_fh *fh = file->private_data;
	struct camera_device *cam = fh->cam;
	int err;
	unsigned long irqflags;
	long timeout;
#if 0				/* Use video_buf to do capture */
	int i;
	for (i = 0; i < 14; i++)
		videobuf_read_one(file, &fh->vbq, data, count, ppos);
	i = videobuf_read_one(file, &fh->vbq, data, count, ppos);
	return i;
#endif

	if (!cam->capture_base) {
		cam->capture_base = (unsigned long)
		    dma_alloc_coherent(NULL,
				       cam->pix.sizeimage,
				       (dma_addr_t *) &
				       cam->capture_base_phys,
				       GFP_KERNEL | GFP_DMA);
	}
	if (!cam->capture_base) {
		dev_err(cam->dev, "cannot allocate capture buffer\n");
		return 0;
	}

	spin_lock_irqsave(&cam->capture_lock, irqflags);
	cam->reading = fh;
	cam->capture_started = 1;
	sg_dma_address(&cam->capture_sglist) = cam->capture_base_phys;
	sg_dma_len(&cam->capture_sglist) = cam->pix.sizeimage;
	spin_unlock_irqrestore(&cam->capture_lock, irqflags);

	err = camera_core_sgdma_queue(cam, &cam->capture_sglist, 1,
				      camera_core_capture_callback, NULL);

	/* Wait till DMA is completed */
	timeout = HZ * 10;
	cam->capture_completed = 0;
	while (cam->capture_completed == 0) {
		timeout = interruptible_sleep_on_timeout
		    (&cam->new_video_frame, timeout);
		if (timeout == 0) {
			dev_err(cam->dev, "timeout waiting video frame\n");
			return -EIO;	/* Time out */
		}
	}
	/* Copy the data to the user buffer */
	err = copy_to_user(data, (void *)cam->capture_base, cam->pix.sizeimage);
	return (cam->pix.sizeimage - err);

}

static int camera_core_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct camera_fh *fh = file->private_data;

	return videobuf_mmap_mapper(&fh->vbq, vma);
}

static int camera_core_release(struct inode *inode, struct file *file)
{
	struct camera_fh *fh = file->private_data;
	struct camera_device *cam = fh->cam;

	file->private_data = NULL;
	kfree(fh);

	spin_lock(&cam->img_lock);

	if (cam->previewing == fh)
		cam->previewing = NULL;
	if (cam->streaming == fh)
		cam->streaming = NULL;
	if (cam->reading == fh)
		cam->reading = NULL;

	spin_unlock(&cam->img_lock);

	camera_dev->cam_hardware->finish_dma(cam->hardware_data);

	if (cam->capture_base) {
		dma_free_coherent(NULL, cam->pix.sizeimage,
				  (void *)cam->capture_base,
				  cam->capture_base_phys);
		cam->capture_base = 0;
		cam->capture_base_phys = 0;
	}
	if (fh->vbq.read_buf) {
		camera_core_vbq_release(&fh->vbq, fh->vbq.read_buf);
		kfree(fh->vbq.read_buf);
	}

	module_put(cam->sdev->module);

	cam->cam_hardware->close(cam->hardware_data);
	cam->active = 0;
	return 0;
}

static int camera_core_open(struct inode *inode, struct file *file)
{
	int minor = iminor(inode);
	struct camera_device *cam = camera_dev;
	struct camera_fh *fh;
	struct v4l2_format format;
	int rval;

	if (!cam || !cam->vfd || (cam->vfd->minor != minor))
		return -ENODEV;

	/* Allocate per-filehandle data */
	fh = kmalloc(sizeof(*fh), GFP_KERNEL);
	if (NULL == fh)
		return -ENOMEM;
	file->private_data = fh;
	fh->cam = cam;
	fh->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	spin_lock(&cam->img_lock);
	if (cam->active == 1) {
		dev_err(cam->dev, "Camera device Active\n");
		spin_unlock(&cam->img_lock);
		rval = -EPERM;
		goto err;
	}
	cam->active = 1;

	if (cam->sdev == NULL || !try_module_get(cam->sdev->module)) {
		spin_unlock(&cam->img_lock);
		rval = -ENODEV;
		goto err;
	}

	vidioc_int_g_fmt_cap(cam->sdev, &format);
	spin_unlock(&cam->img_lock);

	videobuf_queue_sg_init(&fh->vbq, &cam->vbq_ops, NULL, &cam->vbq_lock,
			    fh->type, V4L2_FIELD_NONE,
			    sizeof(struct videobuf_buffer), fh);

	cam->capture_completed = 0;
	cam->capture_started = 0;

	if (cam->cam_hardware->open(cam->hardware_data)) {
		dev_err(cam->dev, "Camera IF configuration failed\n");
		cam->active = 0;
		rval = -ENODEV;
		goto err;
	}
	rval = vidioc_s_fmt_cap(file, fh, &format);
	if (rval) {
		dev_err(cam->dev, "Camera sensor configuration failed (%d)\n",
			rval);
		cam->cam_hardware->close(cam->hardware_data);
		cam->active = 0;
		rval = -ENODEV;
		goto err;
	}

	return 0;

err:
	module_put(cam->sdev->module);
	kfree(fh);
	return rval;
}

#ifdef CONFIG_PM
static int camera_core_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct camera_device *cam = platform_get_drvdata(pdev);

	spin_lock(&cam->img_lock);
	if (cam->active)
		cam->cam_hardware->close(cam->hardware_data);

	vidioc_int_s_power(cam->sdev, 0);
	spin_unlock(&cam->img_lock);

	return 0;
}

static int camera_core_resume(struct platform_device *pdev)
{
	struct camera_device *cam = platform_get_drvdata(pdev);

	spin_lock(&cam->img_lock);
	vidioc_int_s_power(cam->sdev, 1);
	if (cam->active) {
		struct v4l2_format format;

		cam->capture_completed = 1;
		cam->cam_hardware->open(cam->hardware_data);

		vidioc_int_g_fmt_cap(cam->sdev, &format);
		vidioc_int_s_fmt_cap(cam->sdev, &format);
		camera_sensor_if_enable(cam);

		camera_core_sgdma_process(cam);
	}
	spin_unlock(&cam->img_lock);

	return 0;
}
#endif /* CONFIG_PM */

static struct file_operations camera_core_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.read		= camera_core_read,
	.poll		= camera_core_poll,
	.ioctl		= video_ioctl2,
	.mmap		= camera_core_mmap,
	.open		= camera_core_open,
	.release	= camera_core_release,
};
static ssize_t camera_streaming_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct camera_device *cam = dev_get_drvdata(dev);

	return sprintf(buf, "%s\n", cam->streaming ? "active" : "inactive");
}

static DEVICE_ATTR(streaming, S_IRUGO, camera_streaming_show, NULL);

static void camera_device_unregister(struct v4l2_int_device *s)
{
	struct camera_device *cam = s->u.slave->master->priv;

	camera_sensor_exit(cam);
}

static int camera_device_register(struct v4l2_int_device *s)
{
	struct camera_device *cam = s->u.slave->master->priv;
	struct video_device *vfd;
	int rval;

	/* We already have a slave. */
	if (cam->sdev)
		return -EBUSY;

	cam->sdev = s;

	if (device_create_file(cam->dev, &dev_attr_streaming) != 0) {
		dev_err(cam->dev, "could not register sysfs entry\n");
		rval = -EBUSY;
		goto err;
	}

	/* Initialize the video_device struct */
	vfd = cam->vfd = video_device_alloc();
	if (!vfd) {
		dev_err(cam->dev, " could not allocate video device struct\n");
		rval = -ENOMEM;
		goto err;
	}

	vfd->release = video_device_release;

	strlcpy(vfd->name, CAM_NAME, sizeof(vfd->name));
	vfd->type = VID_TYPE_CAPTURE | VID_TYPE_OVERLAY | VID_TYPE_CHROMAKEY;

	/* Need to register for a VID_HARDWARE_* ID in videodev.h */
	vfd->fops = &camera_core_fops;
	video_set_drvdata(vfd, cam);
	vfd->minor = -1;

	vfd->vidioc_querycap = vidioc_querycap;
	vfd->vidioc_enum_fmt_cap = vidioc_enum_fmt_cap;
	vfd->vidioc_g_fmt_cap = vidioc_g_fmt_cap;
	vfd->vidioc_s_fmt_cap = vidioc_s_fmt_cap;
	vfd->vidioc_try_fmt_cap = vidioc_try_fmt_cap;
	vfd->vidioc_reqbufs = vidioc_reqbufs;
	vfd->vidioc_querybuf = vidioc_querybuf;
	vfd->vidioc_qbuf = vidioc_qbuf;
	vfd->vidioc_dqbuf = vidioc_dqbuf;
	vfd->vidioc_streamon = vidioc_streamon;
	vfd->vidioc_streamoff = vidioc_streamoff;
	vfd->vidioc_enum_input = vidioc_enum_input;
	vfd->vidioc_g_input = vidioc_g_input;
	vfd->vidioc_s_input = vidioc_s_input;
	vfd->vidioc_queryctrl = vidioc_queryctrl;
	vfd->vidioc_g_ctrl = vidioc_g_ctrl;
	vfd->vidioc_s_ctrl = vidioc_s_ctrl;
	vfd->vidioc_g_fbuf = vidioc_g_fbuf;
	vfd->vidioc_s_fbuf = vidioc_s_fbuf;
	vfd->vidioc_overlay = vidioc_overlay;

	dev_info(cam->dev, "%s interface with %s sensor\n",
		 cam->cam_hardware->name, cam->sdev->name);

	if (video_register_device(vfd, VFL_TYPE_GRABBER, video_nr) < 0) {
		dev_err(cam->dev,
			"could not register Video for Linux device\n");
		rval = -ENODEV;
		goto err;
	}

	rval = camera_sensor_init(cam);
	if (rval)
		goto err;

	/* Disable the Camera after detection */
	cam->cam_hardware->disable(cam->hardware_data);

	return 0;

err:
	camera_device_unregister(s);

	return rval;
}

static struct v4l2_int_master camera_master = {
	.attach = camera_device_register,
	.detach = camera_device_unregister,
};

static struct v4l2_int_device camera = {
	.module	= THIS_MODULE,
	.name	= CAM_NAME,
	.type	= v4l2_int_type_master,
	.u	= {
		.master = &camera_master
	},
};

static int __init camera_core_probe(struct platform_device *pdev)
{
	struct camera_device *cam;
	int status = 0;

	cam = kzalloc(sizeof(struct camera_device), GFP_KERNEL);
	if (!cam) {
		dev_err(&pdev->dev, "could not allocate memory\n");
		status = -ENOMEM;
		goto err;
	}

	platform_set_drvdata(pdev, cam);

	cam->dev = &pdev->dev;

	/* Initialize the camera interface */
	cam->cam_hardware = &camera_hardware_if;
	cam->hardware_data = cam->cam_hardware->init();
	if (!cam->hardware_data) {
		dev_err(cam->dev, "cannot initialize interface hardware\n");
		status = -ENODEV;
		goto err;
	}

	/* Save the pointer to camera device in a global variable */
	camera_dev = cam;

	/* Initialize the videobuf queue ops */
	cam->vbq_ops.buf_setup = camera_core_vbq_setup;
	cam->vbq_ops.buf_prepare = camera_core_vbq_prepare;
	cam->vbq_ops.buf_queue = camera_core_vbq_queue;
	cam->vbq_ops.buf_release = camera_core_vbq_release;

	/* Initialize the overlay interface */
	cam->overlay_size = overlay_mem;
	if (cam->overlay_size > 0) {
		cam->overlay_base = (unsigned long)
		    dma_alloc_coherent(NULL,
				       cam->overlay_size,
				       (dma_addr_t *) &
				       cam->overlay_base_phys,
				       GFP_KERNEL | GFP_DMA);
		if (!cam->overlay_base) {
			dev_err(cam->dev,
				"cannot allocate overlay framebuffer\n");
			status = -ENOMEM;
			goto err;
		}
	}
	memset((void *)cam->overlay_base, 0, cam->overlay_size);
	spin_lock_init(&cam->overlay_lock);
	spin_lock_init(&cam->capture_lock);

	/* Initialize the spinlock used to serialize access to the image
	 * parameters
	 */
	spin_lock_init(&cam->img_lock);

	/* Initialize the wait queue */
	init_waitqueue_head(&cam->new_video_frame);

	/* Initialize the DMA structures */
	camera_core_sgdma_init(cam);

	platform_set_drvdata(pdev, cam);

	camera.priv = cam;

	if (v4l2_int_device_register(&camera))
		goto err;

	return 0;

err:
	vidioc_int_dev_exit(cam->sdev);
	cam->overlay_base = 0;
	return status;
}

static int camera_core_remove(struct platform_device *pdev)
{
	struct camera_device *cam = platform_get_drvdata(pdev);
	struct video_device *vfd;

	vfd = cam->vfd;
	if (vfd) {
		if (vfd->minor == -1) {
			/* The device never got registered, so release the
			 * video_device struct directly
			 */
			video_device_release(vfd);
		} else {
			/* The unregister function will release the
			 * video_device struct as well as unregistering it.
			 */
			video_unregister_device(vfd);
		}
		cam->vfd = NULL;
	}
	if (cam->overlay_base) {
		dma_free_coherent(NULL, cam->overlay_size,
				  (void *)cam->overlay_base,
				  cam->overlay_base_phys);
		cam->overlay_base = 0;
	}
	cam->overlay_base_phys = 0;

	vidioc_int_dev_exit(cam->sdev);
	cam->cam_hardware->cleanup(cam->hardware_data);
	kfree(cam);
	camera_dev = NULL;

	return 0;
}

static struct platform_driver camera_core_driver = {
	.driver	= {
		.name	= CAM_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= camera_core_probe,
	.remove		= camera_core_remove,
#ifdef CONFIG_PM
	.suspend	= camera_core_suspend,
	.resume		= camera_core_resume,
#endif
};

/* FIXME register omap16xx or omap24xx camera device in arch/arm/...
 * system initialization code, with its resources and mux setup, NOT here.
 * Then MODULE_ALIAS(CAM_NAME) so it hotplugs and coldplugs; this
 * "legacy" driver style is trouble.
 */
static struct platform_device *cam;

static void __exit camera_core_cleanup(void)
{
	platform_driver_unregister(&camera_core_driver);
	platform_device_unregister(cam);
}

static char banner[] __initdata = KERN_INFO "OMAP Camera driver initializing\n";

static int __init camera_core_init(void)
{

	printk(banner);
	platform_driver_register(&camera_core_driver);

	cam = platform_device_register_simple(CAM_NAME, -1, NULL, 0);

	return 0;
}

MODULE_AUTHOR("Texas Instruments.");
MODULE_DESCRIPTION("OMAP Video for Linux camera driver");
MODULE_LICENSE("GPL");

module_param(video_nr, int, 0);
MODULE_PARM_DESC(video_nr,
		 "Minor number for video device (-1 ==> auto assign)");
module_param(capture_mem, int, 0);
MODULE_PARM_DESC(capture_mem,
		 "Maximum amount of memory for capture buffers "
		 "(default 4800KB)");

module_init(camera_core_init);
module_exit(camera_core_cleanup);
