/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2018 Texas Instruments Incorporated - https://www.ti.com/
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
 */

#ifndef __TIDSS_DRV_H__
#define __TIDSS_DRV_H__

#include <linux/spinlock.h>

#define TIDSS_MAX_VPS 4
#define TIDSS_MAX_PLANES 4

/*
 * This is not dependent on the number of VPs.
 * For example, some SoCs have 2 VPs but 3 outputs coming out.
 */
#define TIDSS_MAX_OUTPUTS 4

/* For DSSes with 2 OLDI TXes */
#define TIDSS_MAX_BRIDGES_PER_PIPE	2

#define TIDSS_AM625_IDLE_OLDI_CLOCK 25000000

typedef u32 dispc_irq_t;

struct tidss_device {
	struct drm_device ddev;		/* DRM device for DSS */
	struct device *dev;		/* Underlying DSS device */

	const struct dispc_features *feat;
	struct dispc_device *dispc;

	unsigned int num_crtcs;
	struct drm_crtc *crtcs[TIDSS_MAX_VPS];

	unsigned int num_planes;
	struct drm_plane *planes[TIDSS_MAX_PLANES];

	unsigned int irq;

	spinlock_t wait_lock;	/* protects the irq masks */
	dispc_irq_t irq_mask;	/* enabled irqs in addition to wait_list */

	int num_domains; /* Handle attached PM domains */
	struct device **pd_dev;
	struct device_link **pd_link;

	u32 boot_enabled_vp_mask;
	bool simplefb_enabled;

	bool shared_mode; /* DSS resources shared between remote core and Linux */

	/* 1: VP owned by Linux 0: VP is owned by remote and shared with Linux */
	u32 shared_mode_owned_vps[TIDSS_MAX_VPS];
	bool shared_mode_own_oldi; /* Linux needs to configure OLDI in shared mode */
};

#define to_tidss(__dev) container_of(__dev, struct tidss_device, ddev)

int tidss_runtime_get(struct tidss_device *tidss);
void tidss_runtime_put(struct tidss_device *tidss);

#endif
