/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
 */

#ifndef __TIDSS_CRTC_H__
#define __TIDSS_CRTC_H__

#include <linux/wait.h>
#include <linux/completion.h>
#include <drm/drm_crtc.h>

#include "tidss_dispc.h"

#define to_tidss_crtc(c) container_of((c), struct tidss_crtc, crtc)

struct tidss_crtc {
	struct drm_crtc crtc;

	u32 hw_videoport;

	struct drm_pending_vblank_event *event;

	/* has crtc_atomic_enable been called? */
	bool enabled;

	struct completion framedone_completion;
};

#define to_tidss_crtc_state(x) container_of(x, struct tidss_crtc_state, base)

struct tidss_crtc_state {
	/* Must be first. */
	struct drm_crtc_state base;

	u32 bus_format;
	u32 bus_flags;

	uint64_t trans_key_mode;
	uint64_t trans_key;
	uint64_t background_color;
};

struct tidss_crtc *tidss_crtc_create(struct tidss_device *tidss, u32 hw_videoport,
				     struct drm_plane *primary, struct device_node *epnode);


void tidss_crtc_vblank_irq(struct drm_crtc *crtc);
void tidss_crtc_framedone_irq(struct drm_crtc *crtc);
void tidss_crtc_error_irq(struct drm_crtc *crtc, u64 irqstatus);

#endif
