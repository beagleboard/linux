/*
 * linux/drivers/gpu/drm/omapdrm/omap_wb.h
 *
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Benoit Parrot, <bparrot@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __OMAP_WB_H__
#define __OMAP_WB_H__

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/hrtimer.h>
#include <drm/drm_fourcc.h>

#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>

#include "dss/omapdss.h"
#include "omap_drv.h"

#define WB_MODULE_NAME "omapwb"
#define WBM2M_MODULE_NAME "omapwb-m2m"
#define WBCAP_MODULE_NAME "omapwb-cap"

extern unsigned wbdebug;

#define log_dbg(dev, fmt, arg...)	\
		v4l2_dbg(1, wbdebug, &dev->v4l2_dev, "%s: " fmt, \
			 __func__, ## arg)
#define log_err(dev, fmt, arg...)	\
		v4l2_err(&dev->v4l2_dev, fmt, ## arg)
#define log_info(dev, fmt, arg...)	\
		v4l2_info(&dev->v4l2_dev, fmt, ## arg)

/* minimum and maximum frame sizes */
#define MIN_W		32
#define MIN_H		32
#define MAX_W		2048
#define MAX_H		2048

/* required alignments */
#define S_ALIGN		0	/* multiple of 1 */
#define H_ALIGN		0	/* multiple of 2 */

/* used as plane indices */
#define MAX_PLANES	2
#define LUMA_PLANE	0
#define CHROMA_PLANE	1

enum omap_wb_mode {
	OMAP_WB_NOT_CONFIGURED = 0,
	/* mem2mem from single ovl to wb */
	OMAP_WB_MEM2MEM_OVL = 1,
	/* mem2mem from N overlays via single mgr to wb */
	OMAP_WB_MEM2MEM_MGR = 2,
	/* capture from single mgr to wb */
	OMAP_WB_CAPTURE_MGR = 3
};

enum wb_state {
	WB_STATE_NONE = 0,
	WB_STATE_FIRST_FRAME,
	WB_STATE_CAPTURING,
	WB_STATE_STOPPING,
	WB_STATE_STOPPED,
};

/* driver info for each of the supported video formats */
struct wb_fmt {
	u32	fourcc;			/* standard format identifier */
	u8	coplanar;		/* set for unpacked Luma and Chroma */
	u8	depth[MAX_PLANES];	/* Bits per pixel per plane*/
};

extern struct wb_fmt wb_formats[];
extern unsigned int num_wb_formats;

/* Return a specific unsigned byte from an unsigned int */
#define GET_BYTE(c, b) ((c >> (b * 8)) & 0xff)

struct wb_buffer {
	struct vb2_v4l2_buffer	vb;
	struct list_head	list;
};

/*
 * per-queue, driver-specific private data.
 * MEM-2-MEM: Source: V4L2_BUF_TYPE_VIDEO_OUTPUT*
 *            Destination: V4L2_BUF_TYPE_VIDEO_CAPTURE*
 * CAPTURE:   Destination: V4L2_BUF_TYPE_VIDEO_CAPTURE* only
 */
struct wb_q_data {
	/* format info */
	struct v4l2_format	format;
	/* crop/compose rectangle */
	struct v4l2_rect	c_rect;
	/* format info */
	struct wb_fmt		*fmt;
};

enum {
	Q_DATA_SRC = 0,
	Q_DATA_DST = 1,
};

/* find our format description corresponding to the passed v4l2_format */
struct wb_fmt *find_format(struct v4l2_format *f);

struct wb_dev {
	struct v4l2_device	v4l2_dev;
	struct drm_device	*drm_dev;

	struct omap_drm_irq	wb_irq;
	atomic_t		irq_enabled;

	/* v4l2_ioctl mutex */
	struct mutex		lock;

	enum omap_wb_mode	mode;
	struct wbcap_dev	*cap;
	struct wbm2m_dev	*m2m;
};

/*
 * there is one wbcap_dev structure in the driver.
 */
struct wbcap_dev {
	struct v4l2_device	v4l2_dev;
	struct video_device	vdev;
	struct v4l2_fh		fh;
	struct wb_dev		*dev;
	struct v4l2_ctrl_handler hdl;

	/* dst queue data */
	struct wb_q_data	q_data[2];

	unsigned		input;

	struct vb2_queue	queue;
	struct vb2_alloc_ctx	*alloc_ctx;

	spinlock_t		qlock;
	struct list_head	buf_list;

	/* Current  v4l2_buffer */
	struct wb_buffer	*cur_frm;
	/* Next v4l2_buffer */
	struct wb_buffer	*next_frm;

	unsigned		field;
	unsigned		sequence;

	bool			stopping;
	wait_queue_head_t	event;

	enum wb_state state;

	/* timer used to wait for wb go bit to be cleared */
	struct hrtimer		wbgo_timer;
};

/*
 * there is one wbm2m_dev structure in the driver.
 */
struct wbm2m_dev {
	struct v4l2_device	v4l2_dev;
	struct video_device	vfd;
	struct v4l2_m2m_dev	*m2m_dev;
	struct wb_dev		*dev;
	struct drm_plane	*plane;

	/* v4l2 buffers lock */
	spinlock_t		lock;

	struct vb2_alloc_ctx	*alloc_ctx;
};

/*
 * There is one wbm2m_ctx structure for each m2m context.
 */
struct wbm2m_ctx {
	struct v4l2_fh		fh;
	struct wbm2m_dev	*dev;
	struct v4l2_ctrl_handler hdl;

	/* current frame seq */
	unsigned int		sequence;
	/* abort after next irq */
	unsigned int		aborting;

	/* src & dst queue data */
	struct wb_q_data	q_data[2];
};

static inline struct wb_buffer *to_wb_buffer(struct vb2_buffer *vb2)
{
	return container_of(vb2, struct wb_buffer, vb.vb2_buf);
}

static inline dma_addr_t vb2_dma_addr_plus_data_offset(struct vb2_buffer *vb,
						       unsigned int plane_no)
{
	return vb2_dma_contig_plane_dma_addr(vb, plane_no) +
		vb->planes[plane_no].data_offset;
}

enum omap_color_mode fourcc_to_dss(u32 fourcc);

void wbm2m_irq(struct wbm2m_dev *dev, uint32_t irqstatus);
int wbm2m_init(struct wb_dev *dev);
void wbm2m_cleanup(struct wb_dev *dev);

void wbcap_irq(struct wbcap_dev *dev, u32 irqstatus);
int wbcap_init(struct wb_dev *dev);
void wbcap_cleanup(struct wb_dev *dev);

#endif /* __OMAP_WB_H__ */
