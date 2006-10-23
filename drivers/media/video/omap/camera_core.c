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
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/ctype.h>
#include <linux/pagemap.h>
#include <linux/mm.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/videodev.h>
#include <linux/pci.h>
#include <linux/version.h>
#include <asm/semaphore.h>
#include <asm/processor.h>
#include <linux/dma-mapping.h>
#include <linux/fb.h>

#include <asm/io.h>
#include <asm/byteorder.h>
#include <asm/irq.h>

#include "sensor_if.h"
#include "camera_hw_if.h"
#include "camera_core.h"
 
struct camera_device *camera_dev;
extern struct omap_camera_sensor camera_sensor_if;
extern struct camera_hardware camera_hardware_if;
 
static void camera_core_sgdma_process(struct camera_device *cam);

/* module parameters */
static int video_nr = -1;	/* video device minor (-1 ==> auto assign) */

/* Maximum amount of memory to use for capture buffers.
 * Default is 4800KB, enough to double-buffer SXGA.
 */
static int capture_mem = 1280*960*2*2;

/*Size of video overlay framebuffer. This determines the maximum image size
 *that can be previewed. Default is 600KB, enough for sxga.
 */
static int overlay_mem = 640*480*2;

 
/* DMA completion routine for the scatter-gather DMA fragments. */
/* This function is called when a scatter DMA fragment is completed */
static void
camera_core_callback_sgdma(void *arg1, void *arg2)
{
	struct camera_device *cam = (struct camera_device *)arg1;
	int sgslot = (int)arg2;

	struct sgdma_state *sgdma;

	spin_lock(&cam->sg_lock);
	sgdma = cam->sgdma + sgslot;
	if (!sgdma->queued_sglist)
	{
		spin_unlock(&cam->sg_lock);
		printk(KERN_ERR CAM_NAME ": SGDMA completed when none queued\n");
		return;
	}
	if (!--sgdma->queued_sglist) {
		/* queue for this sglist is empty so check whether transfer
		** of the frame has been completed */
		if (sgdma->next_sglist == sgdma->sglen) {
			dma_callback_t callback = sgdma->callback;
			void *arg = sgdma->arg;
			/* all done with this sglist */
			cam->free_sgdma++;
			if (callback) {
				spin_unlock(&cam->sg_lock);
				(*callback)(cam, arg);
				camera_core_sgdma_process(cam);
				return;
			}
		}
	}
	spin_unlock(&cam->sg_lock);
	camera_core_sgdma_process(cam);

	return;
}

static void
camera_core_sgdma_init(struct camera_device *cam)
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

/*
 * Process the scatter-gather DMA queue by starting queued transfers
 * This function is called to program the dma to start the transfer of an image.
 */
static void
camera_core_sgdma_process(struct camera_device *cam)
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
			if (cam->cam_hardware->start_dma(sgdma, camera_core_callback_sgdma,
				(void *)cam, (void *)sgslot, cam->hardware_data)) {
					/* dma start failed */
					cam->in_use = 0;
					return;
			}
			else {
				/* dma start successful */
				sgdma->next_sglist ++;
				sgdma->queued_sglist ++;
			}
		}
		queued_sgdma-- ;
		sgslot = (sgslot + 1) % (NUM_SG_DMA);
 	}

	cam->in_use = 0;
}

/* Queue a scatter-gather DMA transfer from the camera to memory.
 * Returns zero if the transfer was successfully queued, or
 * non-zero if all of the scatter-gather slots are already in use.
 */
static int
camera_core_sgdma_queue(struct camera_device *cam,
        const struct scatterlist *sglist, int sglen, dma_callback_t callback,
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


/* -------------------overlay routines ------------------------------*/
/* callback routine for overlay DMA completion. We just start another DMA
 * transfer unless overlay has been turned off
 */

static void
camera_core_overlay_callback(void *arg1, void *arg)
{
	struct camera_device *cam = (struct camera_device *)arg1;
	int err;
	unsigned long irqflags;
	int i, j;
	int count, index;
	unsigned char *fb_buf = phys_to_virt((unsigned long)camera_dev->fbuf.base);

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
	for (i = 0 ; i < cam->pix.sizeimage; i += cam->pix.bytesperline) {
		for (index = 0; index < cam->pix.bytesperline; index++) {
			fb_buf[j] = *(((unsigned char *) cam->overlay_base) +
								 i + index);
			index++;
			fb_buf[j + 1] = *(((unsigned char *) cam->overlay_base) + i + index);
			j = j - cam->fbuf.fmt.bytesperline;
		}
		count += 2;
		j = ((cam->pix.width - 1) * cam->fbuf.fmt.bytesperline) + count;
	}

	while (cam->overlay_cnt < 2) {
		err = camera_core_sgdma_queue(cam, &cam->overlay_sglist, 1,
			camera_core_overlay_callback, NULL);
		if (err)
			break;
		++cam->overlay_cnt;
	}

	spin_unlock_irqrestore(&cam->overlay_lock, irqflags);

}

 
static void
camera_core_start_overlay(struct camera_device *cam)
{
	int err;
	unsigned long irqflags;

	if (!cam->previewing) 
		return;

	spin_lock_irqsave(&cam->overlay_lock, irqflags);

	sg_dma_address(&cam->overlay_sglist) = cam->overlay_base_phys;
	sg_dma_len(&cam->overlay_sglist)= cam->pix.sizeimage;
	while (cam->overlay_cnt < 2) {
		err = camera_core_sgdma_queue(cam, &cam->overlay_sglist, 1,
				camera_core_overlay_callback, NULL);
		if (err)
			break;
		++cam->overlay_cnt;
	}

	spin_unlock_irqrestore(&cam->overlay_lock, irqflags);
}

/* ------------------ videobuf_queue_ops ---------------------------------------- */

/* This routine is called from interrupt context when a scatter-gather DMA
 * transfer of a videobuf_buffer completes.
 */
static void
camera_core_vbq_complete(void *arg1, void *arg)
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

static void
camera_core_vbq_release(struct videobuf_queue *q, struct videobuf_buffer *vb)
{
	videobuf_waiton(vb, 0, 0);
	videobuf_dma_unmap(q, &vb->dma);
	videobuf_dma_free(&vb->dma);

	vb->state = STATE_NEEDS_INIT;
}

/* Limit the number of available kernel image capture buffers based on the
 * number requested, the currently selected image size, and the maximum
 * amount of memory permitted for kernel capture buffers.
 */
static int
camera_core_vbq_setup(struct videobuf_queue *q, unsigned int *cnt, unsigned int *size)
{
	struct camera_device *cam = q->priv_data;

	if (*cnt <= 0)
		*cnt = VIDEO_MAX_FRAME; /* supply a default number of buffers */

	if (*cnt > VIDEO_MAX_FRAME)
		*cnt = VIDEO_MAX_FRAME;

	spin_lock(&cam->img_lock);
	*size = cam->pix.sizeimage;
	spin_unlock(&cam->img_lock);

	while (*size * *cnt > capture_mem)
		(*cnt)--;

	return 0;
}

static int
camera_core_vbq_prepare(struct videobuf_queue *q, struct videobuf_buffer *vb,
        enum v4l2_field field)
{
	struct camera_device *cam = q->priv_data;
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
		camera_core_vbq_release (q, vb);

	return err;
}

static void
camera_core_vbq_queue(struct videobuf_queue *q, struct videobuf_buffer *vb)
{
	struct camera_device *cam = q->priv_data;
	enum videobuf_state state = vb->state;
	int err;

	vb->state = STATE_QUEUED;
	err = camera_core_sgdma_queue(cam, vb->dma.sglist, vb->dma.sglen,
                camera_core_vbq_complete, vb);
	if (err) {
		/* Oops.  We're not supposed to get any errors here.  The only
		* way we could get an error is if we ran out of scatter-gather
		* DMA slots, but we are supposed to have at least as many
		* scatter-gather DMA slots as video buffers so that can't
		* happen.
		*/
		printk(KERN_DEBUG CAM_NAME
			": Failed to queue a video buffer for SGDMA\n");
		vb->state = state;
	}
}

/* ------------------ videobuf_queue_ops ---------------------------------------- */

static int
camera_core_do_ioctl(struct inode *inode, struct file *file, unsigned int cmd, 
		     void *arg)
{
	struct camera_fh *fh  = file->private_data;
	struct camera_device *cam = fh->cam;
	int err;

	switch (cmd) {
		case VIDIOC_ENUMINPUT:
		{
			/* default handler assumes 1 video input (the camera) */
			struct v4l2_input *input = (struct v4l2_input *)arg;
			int index = input->index;

			memset(input, 0, sizeof(*input));
			input->index = index;

			if (index > 0)
				return -EINVAL;

			strlcpy(input->name, "camera", sizeof(input->name));
			input->type = V4L2_INPUT_TYPE_CAMERA;

			return 0;
		}

		case VIDIOC_G_INPUT:
		{
			unsigned int *input = arg;
			*input = 0;

			return 0;
		}

		case VIDIOC_S_INPUT:
		{
			unsigned int *input = arg;

			if (*input > 0)
				return -EINVAL;

			return 0;
		}

		case VIDIOC_ENUM_FMT:
		{
			struct v4l2_fmtdesc *fmt = arg;
			return cam->cam_sensor->enum_pixformat(fmt, cam->sensor_data);
		}	

		case VIDIOC_TRY_FMT:
		{
			struct v4l2_format *fmt = arg;
			return cam->cam_sensor->try_format(&fmt->fmt.pix, cam->sensor_data);

		}

		case VIDIOC_G_FMT:
		{
			struct v4l2_format *fmt = arg;

			/* get the current format */
			memset(&fmt->fmt.pix, 0, sizeof (fmt->fmt.pix));
			fmt->fmt.pix = cam->pix;
			
			return 0;
		}

		case VIDIOC_S_FMT:
		{
			struct v4l2_format *fmt = arg;
			unsigned int temp_sizeimage = 0;

			temp_sizeimage = cam->pix.sizeimage;
			cam->cam_sensor->try_format(&fmt->fmt.pix, cam->sensor_data);
			cam->pix = fmt->fmt.pix;

 			cam->xclk = cam->cam_sensor->calc_xclk(&cam->pix,
 				&cam->nominal_timeperframe, cam->sensor_data);
 			cam->cparm.timeperframe = cam->nominal_timeperframe;
			cam->xclk = cam->cam_hardware->set_xclk(cam->xclk, cam->hardware_data);
			return cam->cam_sensor->configure(&cam->pix, cam->xclk, 
						&cam->cparm.timeperframe, cam->sensor_data);
		}

		case VIDIOC_QUERYCTRL:
		{
			struct v4l2_queryctrl *qc = arg;
			return cam->cam_sensor->query_control(qc, cam->sensor_data);
		}

		case VIDIOC_G_CTRL:
		{
			struct v4l2_control *vc = arg;
			return cam->cam_sensor->get_control(vc, cam->sensor_data);
		}

		case VIDIOC_S_CTRL:
		{
			struct v4l2_control *vc = arg;
			return cam->cam_sensor->set_control(vc, cam->sensor_data);
		}
		
		case VIDIOC_QUERYCAP:
		{
			struct v4l2_capability *cap = 
				(struct v4l2_capability *) arg;

			memset(cap, 0, sizeof(*cap));
			strlcpy(cap->driver, CAM_NAME, sizeof(cap->driver));
			strlcpy(cap->card, cam->vfd->name, sizeof(cap->card));
			cap->bus_info[0] = '\0';
			cap->version = KERNEL_VERSION(0, 0, 0);
			cap->capabilities =
				V4L2_CAP_VIDEO_CAPTURE |
				V4L2_CAP_VIDEO_OVERLAY |
				V4L2_CAP_READWRITE | 
				V4L2_CAP_STREAMING;
			return 0;
		}

		case VIDIOC_G_FBUF: /* Get the frame buffer parameters */
		{
			struct v4l2_framebuffer *fbuf =
				(struct v4l2_framebuffer *) arg;

			spin_lock(&cam->img_lock);
			*fbuf = cam->fbuf;
			spin_unlock(&cam->img_lock);
			return 0;
		}

		case VIDIOC_S_FBUF: /* set the frame buffer parameters */
		{
			struct v4l2_framebuffer *fbuf =
				(struct v4l2_framebuffer *) arg;

			spin_lock(&cam->img_lock);
			if (cam->previewing) {
				spin_unlock(&cam->img_lock);
				return -EBUSY;
			}
			cam->fbuf.base = fbuf->base;
			cam->fbuf.fmt = fbuf->fmt;	
			
			spin_unlock(&cam->img_lock);
			return 0;
		}

		case VIDIOC_OVERLAY:
		{
			int enable = *((int *) arg);

			/* 
			 * check whether the capture format and 
			 ** the display format matches 
			 * return failure if they are different
			 */
			if (cam->pix.pixelformat != cam->fbuf.fmt.pixelformat)
			{
				return -EINVAL;
			}

			/* If the camera image size is greater 
			** than LCD size return failure */
			if ((cam->pix.width > cam->fbuf.fmt.height) || 
				(cam->pix.height > cam->fbuf.fmt.width))
			{
				return -EINVAL;
			}
			
			if (!cam->previewing && enable)
			{
				cam->previewing = fh;
				cam->overlay_cnt = 0;
				camera_core_start_overlay(cam);
			}
			else if (!enable)
			{
				cam->previewing = NULL;
			}
	
			return 0;
		}

		case VIDIOC_REQBUFS:
			return videobuf_reqbufs(&fh->vbq, arg);

		case VIDIOC_QUERYBUF:
			return videobuf_querybuf(&fh->vbq, arg);

		case VIDIOC_QBUF:
			return videobuf_qbuf(&fh->vbq, arg);

		case VIDIOC_DQBUF:
			return videobuf_dqbuf(&fh->vbq, arg,
	   file->f_flags & O_NONBLOCK);

		case VIDIOC_STREAMON:
		{
			spin_lock(&cam->img_lock);

			if (cam->streaming || cam->reading) {
				spin_unlock(&cam->img_lock);
				return -EBUSY;
			}
			else {
				cam->streaming = fh;
				/* FIXME: start camera interface */
			}

			spin_unlock(&cam->img_lock);

			return videobuf_streamon(&fh->vbq);
		}
		case VIDIOC_STREAMOFF:
		{
			err = videobuf_streamoff(&fh->vbq);
			if (err < 0)
				return err;

			spin_lock(&cam->img_lock);
			if (cam->streaming == fh) {
				cam->streaming = NULL;
				/* FIXME: stop camera interface */
			}
			spin_unlock(&cam->img_lock);
			return 0;
		}
		case VIDIOC_ENUMSTD:
		case VIDIOC_G_STD:
		case VIDIOC_S_STD:
		case VIDIOC_QUERYSTD:
		{
			/* Digital cameras don't have an analog video standard, 
			 * so we don't need to implement these ioctls.
			 */
			 return -EINVAL;
		}
		case VIDIOC_G_AUDIO:
		case VIDIOC_S_AUDIO:
		case VIDIOC_G_AUDOUT:
		case VIDIOC_S_AUDOUT:
		{
			/* we don't have any audio inputs or outputs */
			return -EINVAL;
		}

		case VIDIOC_G_JPEGCOMP:
		case VIDIOC_S_JPEGCOMP:
		{
			/* JPEG compression is not supported */
			return -EINVAL;
		}

		case VIDIOC_G_TUNER:
		case VIDIOC_S_TUNER:
		case VIDIOC_G_MODULATOR:
		case VIDIOC_S_MODULATOR:
		case VIDIOC_G_FREQUENCY:
		case VIDIOC_S_FREQUENCY:
		{
			/* we don't have a tuner or modulator */
			return -EINVAL;
		}

		case VIDIOC_ENUMOUTPUT:
		case VIDIOC_G_OUTPUT:
		case VIDIOC_S_OUTPUT:
		{
			/* we don't have any video outputs */
			return -EINVAL;
		}

		default:
		{
			/* unrecognized ioctl */
			return -ENOIOCTLCMD;
		}
	}
	return 0;
}

/*
 *  file operations
 */

static unsigned
int camera_core_poll(struct file *file, struct poll_table_struct *wait)
{
	return -EINVAL;
}

/* ------------------------------------------------------------ */
/* callback routine for read DMA completion. We just start another DMA
 * transfer unless overlay has been turned off
 */
static void
camera_core_capture_callback(void *arg1, void *arg)
{
	struct camera_device *cam = (struct camera_device *)arg1;
	int err;
	unsigned long irqflags;
	static int done = 0;

	spin_lock_irqsave(&cam->capture_lock, irqflags);
	if (!cam->reading)
	{
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
			camera_core_capture_callback, NULL);		
	} else {
		cam->capture_completed = 1;
		if (cam->reading)
		{
			/* Wake up any process which are waiting for the 
			** DMA to complete */
			wake_up_interruptible(&camera_dev->new_video_frame);
			sg_dma_address(&cam->capture_sglist) = cam->capture_base_phys;
			sg_dma_len(&cam->capture_sglist) = cam->pix.sizeimage;
			err = camera_core_sgdma_queue(cam, &cam->capture_sglist, 1,
				camera_core_capture_callback, NULL);
		}
	}

	spin_unlock_irqrestore(&cam->capture_lock, irqflags);
}

 
static ssize_t
camera_core_read(struct file *file, char *data, size_t count, loff_t *ppos)
{
	struct camera_fh *fh = file->private_data;
	struct camera_device *cam = fh->cam;
	int err;
	unsigned long irqflags;
	long timeout;
#if 0	/* use video_buf to do capture */
	int i;
	for (i = 0; i < 14; i++)
		videobuf_read_one(file, &fh->vbq, data, count, ppos);
	i = videobuf_read_one(file, &fh->vbq, data, count, ppos);
	return i;
#endif
	
	if (!cam->capture_base) {
		cam->capture_base = (unsigned long)dma_alloc_coherent(NULL,
				cam->pix.sizeimage,
				(dma_addr_t *) &cam->capture_base_phys,
				GFP_KERNEL | GFP_DMA);
	}
	if (!cam->capture_base) {
		printk(KERN_ERR CAM_NAME
			": cannot allocate capture buffer\n");
		return 0;
	}

	spin_lock_irqsave(&cam->capture_lock, irqflags);
	cam->reading = fh;
	cam->capture_started = 1;
	sg_dma_address(&cam->capture_sglist) = cam->capture_base_phys;
	sg_dma_len(&cam->capture_sglist)= cam->pix.sizeimage;
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
			printk(KERN_ERR CAM_NAME ": timeout waiting video frame\n");	
			return -EIO; /* time out */
		}
	}
	/* copy the data to the user buffer */
	err = copy_to_user(data, (void *)cam->capture_base, cam->pix.sizeimage);
	return (cam->pix.sizeimage - err);
	
}

static int
camera_core_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct camera_fh *fh = file->private_data;

	return videobuf_mmap_mapper(&fh->vbq, vma);
}

static int
camera_core_ioctl(struct inode *inode, struct file *file, unsigned int cmd, 
		  unsigned long arg)
{

	return video_usercopy(inode, file, cmd, arg, camera_core_do_ioctl);
}

static int
camera_core_release(struct inode *inode, struct file *file)
{
	struct camera_fh *fh = file->private_data;
	struct camera_device *cam = fh->cam;
	
	file->private_data = NULL;
	kfree(fh);

	spin_lock(&cam->img_lock);
	if (cam->previewing == fh) {
		cam->previewing = NULL;
	}
	if (cam->streaming == fh) {
		cam->streaming = NULL;
	}
	if (cam->reading == fh) {
		cam->reading = NULL;
	}
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

	cam->cam_hardware->close(cam->hardware_data);
	cam->active = 0;
	return 0;
}

static int
camera_core_open(struct inode *inode, struct file *file)
{
	int minor = iminor(inode);
	struct camera_device *cam = camera_dev;
	struct camera_fh *fh;

	if (!cam || !cam->vfd || (cam->vfd->minor != minor))
		return -ENODEV;

	/* allocate per-filehandle data */
	fh = kmalloc(sizeof(*fh), GFP_KERNEL);
	if (NULL == fh)
		return -ENOMEM;
	file->private_data = fh;
	fh->cam = cam;
	fh->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	spin_lock(&cam->img_lock);
	if (cam->active == 1) {
		printk (KERN_ERR CAM_NAME ": Camera device Active\n");
		spin_unlock(&cam->img_lock);
		return -EPERM;
	}
	cam->active = 1;
	spin_unlock(&cam->img_lock);

	videobuf_queue_init(&fh->vbq, &cam->vbq_ops, NULL, &cam->vbq_lock,
		fh->type, V4L2_FIELD_NONE, sizeof(struct videobuf_buffer), fh);

	cam->capture_completed = 0;
	cam->capture_started = 0;

	if (cam->cam_hardware->open(cam->hardware_data))
	{
		printk (KERN_ERR CAM_NAME ": Camera IF configuration failed\n");
		cam->active = 0;
		return -ENODEV;
	}
	
	cam->xclk = cam->cam_hardware->set_xclk(cam->xclk, cam->hardware_data);
	/* program the sensor for the capture format and rate */
	if (cam->cam_sensor->configure(&cam->pix, cam->xclk, 
				&cam->cparm.timeperframe, cam->sensor_data))
	{
		printk (KERN_ERR CAM_NAME ": Camera sensor configuration failed\n");
		cam->cam_hardware->close(cam->hardware_data);
		cam->active = 0;
		return -ENODEV;
	}

	return 0;
}

#ifdef CONFIG_PM
static int camera_core_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct camera_device *cam = platform_get_drvdata(pdev);
	int ret = 0;

	spin_lock(&cam->img_lock);
	if (cam->active) {
		cam->cam_hardware->close(cam->hardware_data);
	}
	cam->cam_sensor->power_off(cam->sensor_data);
	spin_unlock(&cam->img_lock);

	return ret;
}

static int camera_core_resume(struct platform_device *pdev)
{
	struct camera_device *cam = platform_get_drvdata(pdev);
	int ret = 0;

	spin_lock(&cam->img_lock);
	cam->cam_sensor->power_on(cam->sensor_data);
	if (cam->active) {
		cam->capture_completed = 1;
		cam->cam_hardware->open(cam->hardware_data);
		cam->cam_hardware->set_xclk(cam->xclk, cam->hardware_data);

		cam->cam_sensor->configure(&cam->pix, cam->xclk,
					   &cam->cparm.timeperframe,
					   cam->sensor_data);
		camera_core_sgdma_process(cam);
	}
	spin_unlock(&cam->img_lock);
	
	return ret;
}
#endif	/* CONFIG_PM */

static struct file_operations camera_core_fops = 
{
	.owner			= THIS_MODULE,
	.llseek			= no_llseek,
	.read			= camera_core_read,
	.poll			= camera_core_poll,
	.ioctl			= camera_core_ioctl,
	.mmap			= camera_core_mmap,
	.open			= camera_core_open,
	.release		= camera_core_release,
};

static int __init camera_core_probe(struct platform_device *pdev)
{
	struct camera_device *cam;
	struct video_device *vfd;
	int	status;

	cam = kzalloc(sizeof(struct camera_device), GFP_KERNEL);
	if (!cam) {
		printk(KERN_ERR CAM_NAME ": could not allocate memory\n");
		status = -ENOMEM;
		goto err0;
	}

	/* Save the pointer to camera device in a global variable */
	camera_dev = cam;
	
	/* initialize the video_device struct */
	vfd = cam->vfd = video_device_alloc();
	if (!vfd) {
		printk(KERN_ERR CAM_NAME 
			": could not allocate video device struct\n");
		status = -ENOMEM;
		goto err1;
	}
	
 	vfd->release = video_device_release;

 	strlcpy(vfd->name, CAM_NAME, sizeof(vfd->name));
 	vfd->type = VID_TYPE_CAPTURE | VID_TYPE_OVERLAY | VID_TYPE_CHROMAKEY;
 	
 	/* need to register for a VID_HARDWARE_* ID in videodev.h */
 	vfd->hardware = 0;
 	vfd->fops = &camera_core_fops;
 	video_set_drvdata(vfd, cam);
 	vfd->minor = -1;

	/* initialize the videobuf queue ops */
	cam->vbq_ops.buf_setup = camera_core_vbq_setup;
	cam->vbq_ops.buf_prepare = camera_core_vbq_prepare;
	cam->vbq_ops.buf_queue = camera_core_vbq_queue;
	cam->vbq_ops.buf_release = camera_core_vbq_release;

	/* initilize the overlay interface */
	cam->overlay_size = overlay_mem;
	if (cam->overlay_size > 0)
	{
		cam->overlay_base = (unsigned long) dma_alloc_coherent(NULL,
					cam->overlay_size,
					(dma_addr_t *) &cam->overlay_base_phys,
					GFP_KERNEL | GFP_DMA);
		if (!cam->overlay_base) {
			printk(KERN_ERR CAM_NAME
				": cannot allocate overlay framebuffer\n");
			status = -ENOMEM;
			goto err2;
		}
	}
	memset((void*)cam->overlay_base, 0, cam->overlay_size);
	spin_lock_init(&cam->overlay_lock);
	spin_lock_init(&cam->capture_lock);

 	/*Initialise the pointer to the sensor interface and camera interface */
 	cam->cam_sensor = &camera_sensor_if;
 	cam->cam_hardware = &camera_hardware_if;

	/* initialize the camera interface */
	cam->hardware_data = cam->cam_hardware->init();
	if (!cam->hardware_data) {
		printk(KERN_ERR CAM_NAME ": cannot initialize interface hardware\n");
		status = -ENODEV;
		goto err3;
	}
 	 
	/* initialize the spinlock used to serialize access to the image 
	 * parameters
	 */
	spin_lock_init(&cam->img_lock);

	/* initialize the streaming capture parameters */
	cam->cparm.capability = V4L2_CAP_TIMEPERFRAME;
	cam->cparm.readbuffers = 1;

	/* Enable the xclk output.  The sensor may (and does, in the case of 
	 * the OV9640) require an xclk input in order for its initialization 
	 * routine to work.
	 */
	cam->xclk = 21000000;	/* choose an arbitrary xclk frequency */
	cam->xclk = cam->cam_hardware->set_xclk(cam->xclk, cam->hardware_data);

	/* initialize the sensor and define a default capture format cam->pix */
	cam->sensor_data = cam->cam_sensor->init(&cam->pix);
	if (!cam->sensor_data) {
		cam->cam_hardware->disable(cam->hardware_data);
		printk(KERN_ERR CAM_NAME ": cannot initialize sensor\n");
		status = -ENODEV;
		goto err4;
	}

	printk(KERN_INFO CAM_NAME ": %s interface with %s sensor\n",
		cam->cam_hardware->name, cam->cam_sensor->name);

	/* select an arbitrary default capture frame rate of 15fps */
	cam->nominal_timeperframe.numerator = 1;
	cam->nominal_timeperframe.denominator = 15;

	/* calculate xclk based on the default capture format and default 
	 * frame rate
	 */
	cam->xclk = cam->cam_sensor->calc_xclk(&cam->pix,
		&cam->nominal_timeperframe, cam->sensor_data);
 	cam->cparm.timeperframe = cam->nominal_timeperframe;

	/* initialise the wait queue */
	init_waitqueue_head(&cam->new_video_frame);

	/* Initialise the DMA structures */
	camera_core_sgdma_init(cam);

	/* Disable the Camera after detection */
	cam->cam_hardware->disable(cam->hardware_data);
	
	platform_set_drvdata(pdev, cam);
	
	if (video_register_device(vfd, VFL_TYPE_GRABBER, video_nr) < 0) {
		printk(KERN_ERR CAM_NAME 
			": could not register Video for Linux device\n");
		status = -ENODEV;
		goto err5;
	}

	printk(KERN_INFO CAM_NAME 
	       ": registered device video%d [v4l2]\n", vfd->minor);

	return 0;

 err5:
	cam->cam_sensor->cleanup(cam->sensor_data);
 err4:
	cam->cam_hardware->cleanup(cam->hardware_data);
 err3:
	dma_free_coherent(NULL, cam->overlay_size,
				(void *)cam->overlay_base, 
				cam->overlay_base_phys);
	cam->overlay_base = 0;
 err2:
	video_device_release(vfd);
 err1:
	kfree(cam);
	camera_dev = NULL;
 err0:
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
			** video_device struct directly
			*/
			video_device_release(vfd);
		} else {
			/* The unregister function will release the video_device
			** struct as well as unregistering it.
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

	cam->cam_sensor->cleanup(cam->sensor_data);
	cam->cam_hardware->cleanup(cam->hardware_data);
	kfree(cam);
	camera_dev = NULL;

	return 0;
}

static struct platform_driver camera_core_driver = {
	.driver = {
		.name		= CAM_NAME,
		.owner		= THIS_MODULE,
	},
	.probe			= camera_core_probe,
	.remove			= camera_core_remove,
#ifdef CONFIG_PM
	.suspend		= camera_core_suspend,
	.resume			= camera_core_resume,
#endif
};

static struct platform_device camera_core_device = {
	.name	= CAM_NAME,
	.dev	= {
			.release 	= NULL,
		  },
	.id	= 0,
};

void __exit
camera_core_cleanup(void)
{
	platform_driver_unregister(&camera_core_driver);
	platform_device_unregister(&camera_core_device);

	return;
}

static char banner[] __initdata = KERN_INFO "OMAP Camera driver initialzing\n";

int __init 
camera_core_init(void)
{

	printk(banner);
	platform_device_register(&camera_core_device);
	platform_driver_register(&camera_core_driver);

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
        "Maximum amount of memory for capture buffers (default 4800KB)");

module_init(camera_core_init);
module_exit(camera_core_cleanup);

