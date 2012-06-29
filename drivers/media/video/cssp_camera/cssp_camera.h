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

#ifndef CSSP_CAMERA_H
#define CSSP_CAMERA_H


static unsigned video_nr = -1;
module_param(video_nr, uint, 0644);
MODULE_PARM_DESC(video_nr, "videoX start number, -1 is autodetect");

static unsigned debug;
module_param(debug, uint, 0644);
MODULE_PARM_DESC(debug, "activates debug info");

static unsigned int vid_limit = 1;
module_param(vid_limit, uint, 0644);
MODULE_PARM_DESC(vid_limit, "capture memory limit in megabytes");

#define dprintk(dev, level, fmt, arg...) \
	v4l2_dbg(level, debug, &dev->v4l2_dev, fmt, ## arg)

#define VGA_WIDTH 640
#define VGA_HEIGHT 480

#define MAX_WIDTH 2048
#define MAX_HEIGHT 1536

#define VGA_RES (VGA_WIDTH * VGA_HEIGHT)
#define BYTES_PER_PIXEL 2
#define BYTES_PER_DMA_EVT 32

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
	int dma_ch;
	int gpio_reset_pin;
};


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

	/* Input Number */
	int				input;

	/* video capture */
	struct cssp_cam_fmt		*fmt;
	u32				width;
	u32				height;
	u32				bytesperline;
	u32				sizeimage;
	enum v4l2_colorspace		colorspace;
	struct vb2_queue		vb_vidq;
	enum v4l2_field			field;
	unsigned int			field_count;


	/* Camera Sensor */
	struct i2c_board_info		*camera_board_info;
	struct clk			*camera_clk;

	unsigned int			reg_base_virt;
	unsigned int			reg_base_phys;
	resource_size_t			reg_size;
	u16				mode;

	struct edmacc_param		dma_tr_params;
	int				dma_ch;
	u64				dma_mask;

	int				frame_cnt;

	int				reset_pin;
};


#endif /* CSSP_CAMERA_H */
