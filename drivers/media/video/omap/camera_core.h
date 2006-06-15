/*
 *  drivers/media/video/omap/camera_core.h
 *
 * Copyright (C) 2004 Texas Instruments, Inc. 
 * 
 * This package is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation. 
 * 
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR 
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED 
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE. 
 */

#ifndef CAMERA_CORE__H
#define CAMERA_CORE__H

struct camera_fh;

#include <media/video-buf.h>
#include <asm/scatterlist.h>

struct camera_device;
typedef void (*dma_callback_t)(void *arg1, void *arg2);

struct sgdma_state {
	const struct scatterlist *sglist;
	int sglen;              /* number of sglist entries */
	int next_sglist;        /* index of next sglist entry to process */
	int queued_sglist;      /* number of sglist entries queued for DMA */
	unsigned long csr;      /* DMA return code */
	dma_callback_t callback;
	void *arg;
};

/* NUM_SG_DMA is the number of scatter-gather DMA transfers that can be queued.
 */
#define NUM_SG_DMA VIDEO_MAX_FRAME+2
 
/* per-device data structure */
struct camera_device {
	struct device dev;
	struct video_device *vfd;
	
	spinlock_t overlay_lock;        /* spinlock for overlay DMA counter */
	int overlay_cnt;                /* count of queued overlay DMA xfers */
	struct scatterlist overlay_sglist;
	unsigned long overlay_base_phys;
	unsigned long overlay_base;
	unsigned long overlay_size;

	spinlock_t vbq_lock;            /* spinlock for videobuf queues */
	struct videobuf_queue_ops vbq_ops;      /* videobuf queue operations */
	unsigned long field_count;      /* field counter for videobuf_buffer */

	/* scatter-gather DMA management */
	spinlock_t sg_lock;
	int free_sgdma; /* number of free sg dma slots */
	int next_sgdma; /* index of next sg dma slot to use */
	struct sgdma_state sgdma[NUM_SG_DMA];
	char in_use;

	/* The img_lock is used to serialize access to the image parameters for 
	 * overlay and capture.  Need to use spin_lock_irq when writing to the 
	 * reading, streaming, and previewing parameters.  A regular spin_lock 
	 * will suffice for all other cases.
	 */
	spinlock_t img_lock;
 
 	/* We allow reading from at most one filehandle at a time.
 	 * non-NULL means reading is in progress.
 	 */
 	struct camera_fh *reading;
 	/* We allow streaming from at most one filehandle at a time.  
 	 * non-NULL means streaming is in progress.
 	 */
	struct camera_fh *streaming;
	/* We allow previewing from at most one filehandle at a time.  
	 * non-NULL means previewing is in progress.
	 */
	struct camera_fh *previewing;

	/* capture parameters (frame rate, number of buffers) */
	struct v4l2_captureparm cparm;

	/* This is the frame period actually requested by the user. */
	struct v4l2_fract nominal_timeperframe;
	
	/* frequency (in Hz) of camera interface xclk output */
	unsigned long xclk;

	/* Pointer to the sensor interface ops */
	struct omap_camera_sensor *cam_sensor;
	void *sensor_data;
	
	/* Pointer to the camera interface hardware ops */
	struct camera_hardware *cam_hardware;
	void *hardware_data;

	/* pix defines the size and pixel format of the image captured by the 
	 * sensor.  This also defines the size of the framebuffers.  The 
	 * same pool of framebuffers is used for video capture and video 
	 * overlay.  These parameters are set/queried by the 
	 * VIDIOC_S_FMT/VIDIOC_G_FMT ioctls with a CAPTURE buffer type.
	 */
	struct v4l2_pix_format pix;
	struct v4l2_pix_format pix2;

	/* crop defines the size and offset of the video overlay source window 
	 * within the framebuffer.  These parameters are set/queried by the 
	 * VIDIOC_S_CROP/VIDIOC_G_CROP ioctls with an OVERLAY buffer type.  
	 * The cropping rectangle allows a subset of the captured image to be 
	 * previewed.  It only affects the portion of the image previewed, not 
	 * captured; the entire camera image is always captured.
	 */
	struct v4l2_rect crop;

	/* win defines the size and offset of the video overlay target window 
	 * within the video display.  These parameters are set/queried by the 
	 * VIDIOC_S_FMT/VIDIOC_G_FMT ioctls with an OVERLAY buffer type.
	 */
	struct v4l2_window win;

	/* fbuf reflects the size of the video display.  It is queried with the 
	 * VIDIOC_G_FBUF ioctl.  The size of the video display cannot be 
	 * changed with the VIDIOC_S_FBUF ioctl.
	 */
	struct v4l2_framebuffer fbuf;

	/* end of generic stuff, the above should be common to all omaps */

	/* note, 2420 uses videobuf to do caprure, it is more memory efficient
	   we need 1710 and 2420 do capture in the same way */
	/* Variables to store the capture state */
	/* Wait till DMA is completed */
	wait_queue_head_t new_video_frame;
	char capture_completed;
	char capture_started;
 	spinlock_t capture_lock;
	struct scatterlist capture_sglist;
	unsigned long capture_base;
	unsigned long capture_base_phys;

	char active;
};

/* per-filehandle data structure */
struct camera_fh {
	struct camera_device *cam;
	enum v4l2_buf_type type;
	struct videobuf_queue vbq;
};

#define CAM_NAME "omap-camera"

#endif /* CAMERA_CORE__H */
