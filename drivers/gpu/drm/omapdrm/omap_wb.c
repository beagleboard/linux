// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2016-2018 Texas Instruments Incorporated -  http://www.ti.com/
 * Author: Benoit Parrot <bparrot@ti.com>
 */

#include <linux/module.h>
#include <linux/init.h>

#include "omap_wb.h"

unsigned int wbdebug;
module_param(wbdebug, uint, 0644);
MODULE_PARM_DESC(wbdebug, "activates debug info");

struct wb_fmt wb_formats[] = {
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

unsigned int num_wb_formats = ARRAY_SIZE(wb_formats);

/* find our format description corresponding to the passed v4l2_format */
struct wb_fmt *find_format(struct v4l2_format *f)
{
	struct wb_fmt *fmt;
	unsigned int k;

	for (k = 0; k < num_wb_formats; k++) {
		fmt = &wb_formats[k];
		if (fmt->fourcc == f->fmt.pix_mp.pixelformat)
			return fmt;
	}

	return NULL;
}

int omap_wb_fourcc_v4l2_to_drm(u32 fourcc)
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

void omap_wb_irq(void *priv, u32 irqstatus)
{
	struct wb_dev *dev = (struct wb_dev *)priv;
	const u32 mask = OMAP_WB_IRQ_MASK |
			 DISPC_IRQ_VSYNC |
			 DISPC_IRQ_VSYNC2 |
			 DISPC_IRQ_VSYNC3 |
			 DISPC_IRQ_EVSYNC_EVEN |
			 DISPC_IRQ_EVSYNC_ODD;

	if (!dev)
		return;

	irqstatus &= mask;
	if (!irqstatus)
		return;

	if (!atomic_read(&dev->irq_enabled))
		return;

	switch (dev->mode) {
	case OMAP_WB_NOT_CONFIGURED:
		break;
	case OMAP_WB_MEM2MEM_OVL:
		wbm2m_irq(dev->m2m, irqstatus);
		break;
	case OMAP_WB_MEM2MEM_MGR:
		/* To be added */
		break;
	case OMAP_WB_CAPTURE_MGR:
		wbcap_irq(dev->cap, irqstatus);
		break;
	default:
		WARN_ONCE(1, "WB: unknown WB mode: 0x%x\n", dev->mode);
		break;
	}
}

/*
 * The initial setup of this device instance. Note that the initial state of
 * the driver should be complete. So the initial format, standard, timings
 * and video input should all be initialized to some reasonable value.
 */
int omap_wb_init(struct drm_device *drmdev)
{
	struct omap_drm_private *priv = drmdev->dev_private;
	struct wb_dev *dev;
	int ret = 0;

	/* Allocate a new instance */
	dev = devm_kzalloc(drmdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->drm_dev = drmdev;

	/* set pseudo v4l2 device name so we can use v4l2_printk */
	strlcpy(dev->v4l2_dev.name, WB_MODULE_NAME,
		sizeof(dev->v4l2_dev.name));

	priv->wb_private = dev;

	mutex_init(&dev->lock);

	atomic_set(&dev->irq_enabled, 0);

	dev->mode = OMAP_WB_NOT_CONFIGURED;

	ret = wbcap_init(dev);
	if (ret) {
		log_err(dev, "Failed to initialize wb capture\n");
		goto error;
	}

	ret = wbm2m_init(dev);
	if (ret) {
		log_err(dev, "Failed to initialize wb m2m\n");
		goto free_cap;
	}

	log_dbg(dev, "WB loaded\n");
	return 0;

free_cap:
	wbcap_cleanup(dev);
error:
	return ret;
}

void omap_wb_cleanup(struct drm_device *drmdev)
{
	struct omap_drm_private *priv = drmdev->dev_private;
	struct wb_dev *dev = priv->wb_private;

	log_dbg(dev, "Cleanup WB\n");

	wbcap_cleanup(dev);
	wbm2m_cleanup(dev);
}

