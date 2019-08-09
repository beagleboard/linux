/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
 */

#ifndef __TIDSS_PLANE_H__
#define __TIDSS_PLANE_H__

#include "tidss_dispc.h"

#define to_tidss_plane(p) container_of((p), struct tidss_plane, plane)

struct tidss_plane {
	struct drm_plane plane;

	u32 hw_plane_id;

	bool reserved_wb;

};

struct tidss_plane *tidss_plane_create(struct tidss_device *tidss,
				       u32 hw_plane_id,	u32 plane_type,
				       u32 crtc_mask, const u32 *formats,
				       u32 num_formats);

dma_addr_t dispc7_plane_state_paddr(const struct drm_plane_state *state);
dma_addr_t dispc7_plane_state_p_uv_addr(const struct drm_plane_state *state);
struct drm_plane *tidss_plane_reserve_wb(struct drm_device *dev);
void tidss_plane_release_wb(struct drm_plane *plane);

#endif
