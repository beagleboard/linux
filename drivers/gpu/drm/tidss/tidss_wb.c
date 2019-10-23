// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Texas Instruments Incorporated -  http://www.ti.com/
 * Author: Benoit Parrot <bparrot@ti.com>
 */

#include <linux/module.h>
#include <linux/init.h>

#include "tidss_wb.h"

unsigned int tidss_wbdebug;
module_param(tidss_wbdebug, uint, 0644);
MODULE_PARM_DESC(wbdebug, "activates debug info");

const struct wb_fmt tidss_wb_formats[] = {
	{
		.fourcc		= V4L2_PIX_FMT_NV12,
		.coplanar	= 0,
		.depth		= {8, 4},
	},
	{
		.fourcc		= V4L2_PIX_FMT_NV12M,
		.coplanar	= 1,
		.depth		= {8, 4},
	},
	{
		.fourcc		= V4L2_PIX_FMT_YUYV,
		.coplanar	= 0,
		.depth		= {16, 0},
	},
	{
		.fourcc		= V4L2_PIX_FMT_UYVY,
		.coplanar	= 0,
		.depth		= {16, 0},
	},
	{
		/* "XR24", DRM_FORMAT_XRGB8888 */
		.fourcc		= V4L2_PIX_FMT_XBGR32,
		.coplanar	= 0,
		.depth		= {32, 0},
	},
};

const unsigned int tidss_num_wb_formats = ARRAY_SIZE(tidss_wb_formats);

/* find our format description corresponding to the passed v4l2_format */
const struct wb_fmt *__tidss_wb_find_format(u32 fourcc)
{
	const struct wb_fmt *fmt;
	unsigned int k;

	for (k = 0; k < tidss_num_wb_formats; k++) {
		fmt = &tidss_wb_formats[k];
		if (fmt->fourcc == fourcc)
			return fmt;
	}

	return NULL;
}

const struct wb_fmt *tidss_wb_find_format(struct v4l2_format *f)
{
	return __tidss_wb_find_format(f->fmt.pix_mp.pixelformat);
}

int tidss_wb_fourcc_v4l2_to_drm(u32 fourcc)
{
	switch (fourcc) {
	case V4L2_PIX_FMT_NV12:
	case V4L2_PIX_FMT_NV12M:
		return DRM_FORMAT_NV12;
	case V4L2_PIX_FMT_YUYV:
		return DRM_FORMAT_YUYV;
	case V4L2_PIX_FMT_UYVY:
		return DRM_FORMAT_UYVY;
	case V4L2_PIX_FMT_XBGR32:
		return DRM_FORMAT_XRGB8888;
	default:
		WARN(1, "WB: unsupported fourcc\n");
		return 0;
	}
}

void tidss_wb_irq(struct wb_dev *wdev, u64 irqstatus)
{
	const u64 mask = DSS_IRQ_DEVICE_WB_MASK;

	if (!wdev)
		return;

	irqstatus &= mask;
	if (!irqstatus)
		return;

	if (!atomic_read(&wdev->irq_enabled))
		return;

	switch (wdev->mode) {
	case TIDSS_WB_NOT_CONFIGURED:
		break;
	case TIDSS_WB_MEM2MEM_OVL:
		tidss_wbm2m_irq(wdev->m2m, irqstatus);
		break;
	case TIDSS_WB_MEM2MEM_MGR:
		/* To be added */
		break;
	case TIDSS_WB_CAPTURE_MGR:
		/* To be added */
		break;
	default:
		WARN_ONCE(1, "WB: unknown WB mode: 0x%x\n", wdev->mode);
		break;
	}
}

/*
 * The initial setup of this device instance. Note that the initial state of
 * the driver should be complete. So the initial format, standard, timings
 * and video input should all be initialized to some reasonable value.
 */
int tidss_wb_init(struct drm_device *drmdev)
{
	struct tidss_device *tidss = drmdev->dev_private;
	struct wb_dev *wdev;
	int ret = 0;

	/* Allocate a new instance */
	wdev = devm_kzalloc(drmdev->dev, sizeof(*wdev), GFP_KERNEL);
	if (!wdev)
		return -ENOMEM;

	wdev->drm_dev = drmdev;

	/* set pseudo v4l2 device name so we can use v4l2_printk */
	strlcpy(wdev->v4l2_dev.name, WB_MODULE_NAME,
		sizeof(wdev->v4l2_dev.name));

	tidss->wdev = wdev;

	mutex_init(&wdev->lock);

	atomic_set(&wdev->irq_enabled, 0);

	wdev->mode = TIDSS_WB_NOT_CONFIGURED;

	ret = tidss_wbm2m_init(wdev);
	if (ret) {
		log_err(wdev, "Failed to initialize wb m2m\n");
		return ret;
	}

	log_dbg(wdev, "WB loaded\n");
	return 0;
}

void tidss_wb_cleanup(struct drm_device *drmdev)
{
	struct tidss_device *tidss = drmdev->dev_private;
	struct wb_dev *wdev = tidss->wdev;

	log_dbg(wdev, "Cleanup WB\n");

	tidss_wbm2m_cleanup(wdev);
}

