/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2019 Texas Instruments Incorporated -  http://www.ti.com/
 * Author: Benoit Parrot <bparrot@ti.com>
 */

#ifndef __TIDSS_WB_H__
#define __TIDSS_WB_H__

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/hrtimer.h>
#include <drm/drmP.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_plane_helper.h>

#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>

#include "tidss_irq.h"
#include "tidss_dispc.h"
//#include "tidss_dispc7.h"
#include "tidss_drv.h"
#include "tidss_plane.h"

#define WB_MODULE_NAME "tidsswb"
#define WBM2M_MODULE_NAME "tidss-m2m"

extern unsigned int tidss_wbdebug;

#define log_dbg(dev, fmt, arg...)	\
		v4l2_dbg(1, tidss_wbdebug, &dev->v4l2_dev, "%s: " fmt, \
			 __func__, ## arg)
#define log_err(dev, fmt, arg...)	\
		v4l2_err(&dev->v4l2_dev, fmt, ## arg)
#define log_info(dev, fmt, arg...)	\
		v4l2_info(&dev->v4l2_dev, fmt, ## arg)

/* minimum and maximum frame sizes */
#define MIN_W		2
#define MIN_H		1
#define MAX_W		4096
#define MAX_H		4096

/* required alignments */
#define S_ALIGN		0	/* multiple of 1 */
#define H_ALIGN		0	/* multiple of 2 */

/* used as plane indices */
#define MAX_PLANES	2
#define LUMA_PLANE	0
#define CHROMA_PLANE	1

enum tidss_wb_mode {
	TIDSS_WB_NOT_CONFIGURED = 0,
	/* mem2mem from single ovl to wb */
	TIDSS_WB_MEM2MEM_OVL = 1,
	/* mem2mem from N overlays via single mgr to wb */
	TIDSS_WB_MEM2MEM_MGR = 2,
	/* capture from single mgr to wb */
	TIDSS_WB_CAPTURE_MGR = 3
};

enum tidss_wb_state {
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

extern const struct wb_fmt tidss_wb_formats[];
extern const unsigned int tidss_num_wb_formats;

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
	const struct wb_fmt	*fmt;
};

enum {
	Q_DATA_SRC = 0,
	Q_DATA_DST = 1,
};

/* find our format description corresponding to the passed v4l2_format */
const struct wb_fmt *tidss_wb_find_format(struct v4l2_format *f);
const struct wb_fmt *__tidss_wb_find_format(u32 fourcc);

struct wb_dev {
	struct v4l2_device	v4l2_dev;
	struct drm_device	*drm_dev;

	atomic_t		irq_enabled;

	/* v4l2_ioctl mutex */
	struct mutex		lock;

	enum tidss_wb_mode	mode;
	struct wbm2m_dev	*m2m;
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

	/* src & dst state data */
	struct drm_plane_state s_state;
	struct drm_framebuffer s_fb;
	struct drm_gem_cma_object s_cma_gem_obj[2];

	struct drm_plane_state d_state;
	struct drm_framebuffer d_fb;
	struct drm_gem_cma_object d_cma_gem_obj[2];
};

static inline struct wb_buffer *to_wb_buffer(struct vb2_buffer *vb2)
{
	return container_of(vb2, struct wb_buffer, vb.vb2_buf);
}

int tidss_wb_fourcc_v4l2_to_drm(u32 fourcc);

void tidss_wbm2m_irq(struct wbm2m_dev *dev, u64 irqstatus);
int tidss_wbm2m_init(struct wb_dev *dev);
void tidss_wbm2m_cleanup(struct wb_dev *dev);

#endif /* __TIDSS_WB_H__ */
