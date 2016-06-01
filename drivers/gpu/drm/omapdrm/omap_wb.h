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

#define WBM2M_MODULE_NAME "omapwb-m2m"

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

/*
 * per-queue, driver-specific private data.
 * there is one source queue and one destination queue for each m2m context.
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

	/* v4l2_ioctl mutex */
	struct mutex		lock;

	struct wbm2m_dev	*m2m;
};

/*
 * there is one wbm2m_dev structure in the driver
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
	/* bufs done in this batch */
	unsigned int		bufs_completed;

	/* src & dst queue data */
	struct wb_q_data	q_data[2];
	struct vb2_v4l2_buffer	*src_vb;
	struct vb2_v4l2_buffer	*dst_vb;
};

static inline dma_addr_t vb2_dma_addr_plus_data_offset(struct vb2_buffer *vb,
						       unsigned int plane_no)
{
	return vb2_dma_contig_plane_dma_addr(vb, plane_no) +
		vb->planes[plane_no].data_offset;
}

enum omap_color_mode fourcc_to_dss(u32 fourcc);

#endif /* __OMAP_WB_H__ */
