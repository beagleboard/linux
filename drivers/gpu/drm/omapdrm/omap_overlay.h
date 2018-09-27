/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2018 Texas Instruments Incorporated -  http://www.ti.com/
 * Author: Benoit Parrot, <bparrot@ti.com>
 */

#ifndef __OMAPDRM_OVERLAY_H__
#define __OMAPDRM_OVERLAY_H__

#include <linux/types.h>

enum drm_plane_type;

struct drm_device;
struct drm_mode_object;
struct drm_plane;
struct omap_drm_private;

/* Used to associate a HW overlay/plane to a plane */
struct omap_hw_overlay {
	int idx;

	const char *name;
	enum omap_plane_id overlay_id;

	enum omap_overlay_caps caps;
	/*
	 * The CRTC(s) this overlay is currently allowed on.
	 * When the overlay is unused and was not assigned to any crtc then
	 * this will be the equal to the plane possible_crtcs otherwise it
	 * will be the current crtc this overlay is displayed on.
	 * When clearing the overlay to plane assignemnt while going through
	 * an atomic_check sequence we need to remember which crtc the overlay
	 * was on as we do not want to create flicker. We want to be able to
	 * reassign the overlay to the same crtc it was previously on.
	 */
	u32 possible_crtcs;
};

/*
 * Global private object state for tracking resources that are shared across
 * multiple kms objects (planes/crtcs/etc).
 */
#define to_omap_global_state(x) container_of(x, struct omap_global_state, base)
struct omap_global_state {
	struct drm_private_state base;

	struct drm_atomic_state *state;

	/* global atomic state of assignment between pipes and planes */
	struct drm_plane *hwoverlay_to_plane[8];
};

static inline const char *overlay2name(enum omap_plane_id id)
{
	static const char * const name[] = {
		[OMAP_DSS_GFX] = "gfx",
		[OMAP_DSS_VIDEO1] = "vid1",
		[OMAP_DSS_VIDEO2] = "vid2",
		[OMAP_DSS_VIDEO3] = "vid3",
	};
	return name[id];
}

int omap_hwoverlays_init(struct drm_device *dev);
void omap_hwoverlays_destroy(struct drm_device *dev);
int omap_global_obj_init(struct omap_drm_private *priv);
void omap_global_obj_fini(struct omap_drm_private *priv);
struct omap_global_state *__must_check
omap_get_global_state(struct drm_atomic_state *s);
int omap_overlay_assign(struct drm_atomic_state *s, struct drm_plane *plane,
			u32 caps, u32 fourcc, u32 crtc_mask,
			struct omap_hw_overlay **overlay,
			struct omap_hw_overlay **r_overlay);
void omap_overlay_release(struct drm_atomic_state *s,
			  struct drm_plane *plane,
			  struct omap_hw_overlay *overlay);
void omap_overlay_disable(struct drm_atomic_state *s,
			  struct drm_plane *plane,
			  struct omap_hw_overlay *overlay);
int omap_overlay_assign_wb(struct omap_drm_private *priv,
			   struct drm_plane *plane, u32 crtc_mask,
			   struct omap_hw_overlay **overlay);
void omap_overlay_release_wb(struct omap_drm_private *priv,
			     struct drm_plane *plane,
			     struct omap_hw_overlay *overlay);

#endif /* __OMAPDRM_OVERLAY_H__ */
