/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
 */

#ifndef __TIDSS_DRV_H__
#define __TIDSS_DRV_H__

#include <linux/spinlock.h>
#include <linux/rpmsg-remotedev/rpmsg-remotedev.h>

struct tidss_device {
	struct device *dev;		/* Underlying DSS device */
	struct drm_device *ddev;	/* DRM device for DSS */

	struct rpmsg_remotedev *rdev;
	struct rpmsg_remotedev_display_resinfo rres;

	struct drm_fbdev_cma *fbdev;

	struct dispc_device *dispc;
	const struct tidss_dispc_ops *dispc_ops;

	const struct tidss_features *features;

	unsigned int num_crtcs;
	unsigned int num_v_crtcs;
	struct drm_crtc *crtcs[8];
	struct drm_crtc *v_crtcs[8];

	unsigned int num_planes;
	struct drm_plane *planes[8];

	spinlock_t wait_lock;	/* protects the irq masks */
	u64 irq_mask;		/* enabled irqs in addition to wait_list */
	u64 irq_uf_mask;	/* underflow irq bits for all planes */

	struct drm_atomic_state *saved_state;
};

struct tidss_features {
	int (*dispc_init)(struct tidss_device *tidss);
};

#endif
