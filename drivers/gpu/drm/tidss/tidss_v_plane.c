// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Subhajit Paul <subhajit_paul@ti.com>
 */

/*
 * Virtual Planes: reuse drivers/gpu/drm/tidss/tidss_plane.c
 */
#include <drm/drmP.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include "tidss_v_display.h"

#define V_PLANE_MAX_FORMATS (32)

struct v_plane_state {
	struct drm_plane_state state;
	bool need_update;
};

struct v_plane {
	struct drm_plane base;
	struct tidss_device *tidss;
	unsigned int remote_id;
	int possible_crtcs;
	int id;

	int nformats;
	u32 formats[V_PLANE_MAX_FORMATS];

	bool can_scale;
	bool can_mod_win;
	int fx;
	int fy;
	int fw;
	int fh;
};

#define to_v_plane(x) container_of(x, struct v_plane, base)
#define to_v_plane_state(x) container_of(x, struct v_plane_state, state)

unsigned int v_plane_get_remote_id(struct drm_plane *plane)
{
	struct v_plane *v_plane = to_v_plane(plane);

	return v_plane->remote_id;
}

bool v_plane_update_needed(struct drm_plane *plane)
{
	struct v_plane_state *v_state = to_v_plane_state(plane->state);

	return v_state->need_update;
}

static bool v_plane_state_changed(struct drm_plane_state *old, struct drm_plane_state *new)
{
	if (old->crtc != new->crtc)
		return true;

	if (old->fb != new->fb)
		return true;

	if (old->crtc_x != new->crtc_x)
		return true;

	if (old->crtc_y != new->crtc_y)
		return true;

	if (old->crtc_w != new->crtc_w)
		return true;

	if (old->crtc_h != new->crtc_h)
		return true;

	if (old->src_x != new->src_x)
		return true;

	if (old->src_y != new->src_y)
		return true;

	if (old->src_w != new->src_w)
		return true;

	if (old->src_h != new->src_h)
		return true;

	return false;
}

static int v_plane_atomic_check(struct drm_plane *plane,
				    struct drm_plane_state *state)
{
	struct v_plane *v_plane = to_v_plane(plane);
	struct drm_crtc_state *crtc_state;
	int ret;

	dev_dbg(plane->dev->dev, "%s\n", __func__);

	if (!state->crtc) {
		/*
		 * The visible field is not reset by the DRM core but only
		 * updated by drm_plane_helper_check_state(), set it manually.
		 */
		state->visible = false;
		return 0;
	}

	crtc_state = drm_atomic_get_crtc_state(state->state, state->crtc);
	if (IS_ERR(crtc_state))
		return PTR_ERR(crtc_state);

	ret = drm_atomic_helper_check_plane_state(state, crtc_state,
						  0,
						  INT_MAX,
						  true, true);
	if (ret < 0)
		return ret;

	if (v_plane->can_scale == false && (
				((state->src_w >> 16) != state->crtc_w) ||
				((state->src_h >> 16) != state->crtc_h)))
		return -EINVAL;

	if (v_plane->can_mod_win == false && (
				state->crtc_x != v_plane->fx ||
				state->crtc_y != v_plane->fy ||
				state->crtc_w != v_plane->fw ||
				state->crtc_h != v_plane->fh))
		return -EINVAL;

	return 0;
}

static void v_plane_atomic_update(struct drm_plane *plane,
				      struct drm_plane_state *old_state)
{
	struct drm_plane_state *plane_state = plane->state;
	struct v_plane_state *v_state = to_v_plane_state(plane_state);

	dev_dbg(plane->dev->dev, "%s\n", __func__);

	v_state->need_update = v_plane_state_changed(old_state, plane_state);
}

static void v_plane_atomic_disable(struct drm_plane *plane, struct drm_plane_state *old_state)
{
	struct drm_plane_state *plane_state = plane->state;
	struct v_plane_state *v_state = to_v_plane_state(plane_state);

	dev_dbg(plane->dev->dev, "%s\n", __func__);

	v_state->need_update = v_plane_state_changed(old_state, plane_state);
}

static const struct drm_plane_helper_funcs v_plane_helper_funcs = {
	.atomic_check = v_plane_atomic_check,
	.atomic_update = v_plane_atomic_update,
	.atomic_disable = v_plane_atomic_disable,
};

static void v_plane_destroy(struct drm_plane *plane)
{
	struct v_plane *v_plane = to_v_plane(plane);

	dev_dbg(plane->dev->dev, "%s\n", __func__);

	drm_plane_cleanup(plane);
	kfree(v_plane);
}

static struct drm_plane_state *
v_plane_atomic_duplicate_state(struct drm_plane *plane)
{
	struct v_plane_state *v_state;
	struct drm_plane_state *state;

	if (WARN_ON(!plane->state))
		return NULL;

	v_state = kmalloc(sizeof(*v_state), GFP_KERNEL);
	if (WARN_ON(!v_state))
		return NULL;

	state = &v_state->state;
	__drm_atomic_helper_plane_duplicate_state(plane, &v_state->state);

	v_state->need_update = false;

	return state;
}

static void v_plane_atomic_destroy_state(struct drm_plane *plane,
					   struct drm_plane_state *state)
{
	struct v_plane_state *v_state = to_v_plane_state(state);

	if (WARN_ON(!state))
		return;

	__drm_atomic_helper_plane_destroy_state(state);

	kfree(v_state);
}

static const struct drm_plane_funcs v_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.reset = drm_atomic_helper_plane_reset,
	.destroy = v_plane_destroy,
	.atomic_duplicate_state = v_plane_atomic_duplicate_state,
	.atomic_destroy_state = v_plane_atomic_destroy_state,
};

struct drm_plane *v_plane_init(struct tidss_device *tidss,
		int id, int crtc_bitmask,
		enum drm_plane_type type,
		struct rpmsg_remotedev_display_disp *vp, int index)
{
	struct drm_device *dev = tidss->ddev;
	struct drm_plane *plane;
	struct v_plane *v_plane;
	int ret;
	int fmt;
	struct rpmsg_remotedev_display_pipe *vid = &vp->pipes[index];

	if (vid->num_formats > V_PLANE_MAX_FORMATS)
		return NULL;

	v_plane = kzalloc(sizeof(*v_plane), GFP_KERNEL);
	if (!v_plane)
		return NULL;

	v_plane->tidss = tidss;
	v_plane->id = id;
	v_plane->remote_id = vid->pipe_id;
	v_plane->can_scale = vid->can_scale;
	v_plane->can_mod_win = vid->can_mod_win;
	if (!v_plane->can_mod_win) {
		v_plane->fx = vid->fixed_win_x;
		v_plane->fy = vid->fixed_win_y;
		v_plane->fw = vid->fixed_win_w;
		v_plane->fh = vid->fixed_win_h;
	}
	v_plane->possible_crtcs = crtc_bitmask;
	v_plane->nformats = vid->num_formats;

	for (fmt = 0; fmt < vid->num_formats; fmt++)
		v_plane->formats[fmt] = vid->formats[fmt];

	plane = &v_plane->base;

	ret = drm_universal_plane_init(dev, plane, crtc_bitmask,
				       &v_plane_funcs, v_plane->formats,
				       v_plane->nformats, NULL, type, "plane-0x%x", id);
	if (ret < 0)
		goto error;

	drm_plane_helper_add(plane, &v_plane_helper_funcs);

	drm_plane_create_zpos_immutable_property(plane, vid->initial_zorder);

	return plane;

error:
	kfree(v_plane);
	return NULL;
}

void v_plane_fini(struct tidss_device *tidss, struct drm_plane *plane)
{
	v_plane_destroy(plane);
}
