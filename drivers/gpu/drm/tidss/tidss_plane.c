// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
 */

#include <drm/drmP.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_plane_helper.h>

#include "tidss_crtc.h"
#include "tidss_drv.h"
#include "tidss_plane.h"

static void tidss_plane_info_init(struct drm_plane_state *state,
				  struct tidss_plane_info *info)
{
	memset(info, 0, sizeof(*info));

	info->fourcc		= state->fb->format->format;
	info->pos_x		= state->crtc_x;
	info->pos_y		= state->crtc_y;
	info->out_width		= state->crtc_w;
	info->out_height	= state->crtc_h;
	info->width		= state->src_w >> 16;
	info->height		= state->src_h >> 16;
	info->global_alpha	= state->alpha >> 8;
	info->zorder		= state->normalized_zpos;
	info->color_encoding	= state->color_encoding;
	info->color_range	= state->color_range;
}

static int tidss_plane_atomic_check(struct drm_plane *plane,
				    struct drm_plane_state *state)
{
	struct drm_device *ddev = plane->dev;
	struct tidss_device *tidss = ddev->dev_private;
	struct drm_crtc_state *crtc_state;
	struct drm_rect clip;
	struct tidss_plane *tplane = to_tidss_plane(plane);
	struct tidss_plane_info info;
	const struct drm_format_info *finfo;
	u32 hw_videoport;
	int ret;

	dev_dbg(ddev->dev, "%s\n", __func__);

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

	/*
	 * The HW is only able to start drawing at subpixel boundary
	 * (the two first checks bellow). At the end of a row the HW
	 * can only jump integer number of subpixels forward to
	 * beginning of next row. So we can only show picture with
	 * integer subpixel width (the third check). However, after
	 * reaching the end of the drawn picture the drawing starts
	 * again at the absolute memory address where top left corner
	 * position of the drawn picture is (so there is no need to
	 * check for odd height).
	 */

	finfo = drm_format_info(state->fb->format->format);

	if ((state->src_x >> 16) % finfo->hsub != 0) {
		dev_dbg(ddev->dev,
			"%s: x-position %u not divisible subpixel size %u\n",
			__func__, (state->src_x >> 16), finfo->hsub);
		return -EINVAL;
	}

	if ((state->src_y >> 16) % finfo->vsub != 0) {
		dev_dbg(ddev->dev,
			"%s: y-position %u not divisible subpixel size %u\n",
			__func__, (state->src_y >> 16), finfo->vsub);
		return -EINVAL;
	}

	if ((state->src_w >> 16) % finfo->hsub != 0) {
		dev_dbg(ddev->dev,
			"%s: src width %u not divisible by subpixel size %u\n",
			 __func__, (state->src_w >> 16), finfo->hsub);
		return -EINVAL;
	}

	clip.x1 = 0;
	clip.y1 = 0;
	clip.x2 = crtc_state->adjusted_mode.hdisplay;
	clip.y2 = crtc_state->adjusted_mode.vdisplay;
	ret = drm_plane_helper_check_state(state, &clip,
					   0, INT_MAX,
					   true, true);
	if (ret < 0)
		return ret;

	if (!state->visible)
		return 0;

	hw_videoport = to_tidss_crtc(state->crtc)->hw_videoport;

	tidss_plane_info_init(state, &info);

	return tidss->dispc_ops->plane_check(tidss->dispc,
					     tplane->hw_plane_id,
					     &info, hw_videoport);
}

static void tidss_plane_atomic_update(struct drm_plane *plane,
				      struct drm_plane_state *old_state)
{
	struct drm_device *ddev = plane->dev;
	struct tidss_device *tidss = ddev->dev_private;
	struct tidss_plane *tplane = to_tidss_plane(plane);
	struct tidss_plane_info info;
	int ret;
	u32 hw_videoport;
	struct drm_plane_state *state = plane->state;
	struct drm_framebuffer *fb = state->fb;
	uint32_t x, y;
	struct drm_gem_cma_object *gem;

	dev_dbg(ddev->dev, "%s\n", __func__);

	if (!state->visible) {
		tidss->dispc_ops->plane_enable(tidss->dispc, tplane->hw_plane_id,
					       false);
		return;
	}

	hw_videoport = to_tidss_crtc(state->crtc)->hw_videoport;

	tidss_plane_info_init(state, &info);

	x = state->src_x >> 16;
	y = state->src_y >> 16;

	gem = drm_fb_cma_get_gem_obj(fb, 0);

	info.paddr = gem->paddr + fb->offsets[0] + x * fb->format->cpp[0] +
		     y * fb->pitches[0];

	info.fb_width  = fb->pitches[0] / fb->format->cpp[0];
	info.cpp = fb->format->cpp[0];

	if (fb->format->num_planes == 2) {
		gem = drm_fb_cma_get_gem_obj(fb, 1);

		info.p_uv_addr = gem->paddr +
				 fb->offsets[0] +
				 (x * fb->format->cpp[0] / fb->format->hsub) +
				 (y * fb->pitches[0] / fb->format->vsub);

		info.fb_width_uv = fb->pitches[1] / fb->format->cpp[1];
		info.cpp_uv = fb->format->cpp[1];
	}

	ret = tidss->dispc_ops->plane_setup(tidss->dispc, tplane->hw_plane_id,
					    &info, hw_videoport);

	if (ret) {
		dev_err(plane->dev->dev, "Failed to setup plane %d\n",
			tplane->hw_plane_id);
		tidss->dispc_ops->plane_enable(tidss->dispc, tplane->hw_plane_id,
					       false);
		return;
	}

	tidss->dispc_ops->plane_enable(tidss->dispc, tplane->hw_plane_id, true);
}

static void tidss_plane_atomic_disable(struct drm_plane *plane,
				       struct drm_plane_state *old_state)
{
	struct drm_device *ddev = plane->dev;
	struct tidss_device *tidss = ddev->dev_private;
	struct tidss_plane *tplane = to_tidss_plane(plane);

	dev_dbg(ddev->dev, "%s\n", __func__);

	tidss->dispc_ops->plane_enable(tidss->dispc, tplane->hw_plane_id, false);
}

static const struct drm_plane_helper_funcs tidss_plane_helper_funcs = {
	.atomic_check = tidss_plane_atomic_check,
	.atomic_update = tidss_plane_atomic_update,
	.atomic_disable = tidss_plane_atomic_disable,
};

static const struct drm_plane_funcs tidss_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.reset = drm_atomic_helper_plane_reset,
	.destroy = drm_plane_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
};

struct tidss_plane *tidss_plane_create(struct tidss_device *tidss,
				       u32 hw_plane_id,	u32 plane_type,
				       u32 crtc_mask, const u32 *formats,
				       u32 num_formats)
{
	enum drm_plane_type type;
	uint32_t possible_crtcs;
	uint num_planes = tidss->dispc_ops->get_num_planes(tidss->dispc);
	int ret;
	struct tidss_plane *tplane;
	const struct tidss_plane_feat *pfeat;

	pfeat = tidss->dispc_ops->plane_feat(tidss->dispc, hw_plane_id);

	tplane = devm_kzalloc(tidss->dev, sizeof(*tplane), GFP_KERNEL);
	if (!tplane)
		return ERR_PTR(-ENOMEM);

	tplane->hw_plane_id = hw_plane_id;

	possible_crtcs = crtc_mask;
	type = plane_type;

	ret = drm_universal_plane_init(tidss->ddev, &tplane->plane,
				       possible_crtcs,
				       &tidss_plane_funcs,
				       formats, num_formats,
				       NULL, type, NULL);
	if (ret < 0)
		return ERR_PTR(ret);

	drm_plane_helper_add(&tplane->plane, &tidss_plane_helper_funcs);
	if (num_planes > 1)
		drm_plane_create_zpos_property(&tplane->plane, hw_plane_id, 0,
					       num_planes - 1);

	ret = drm_plane_create_color_properties(&tplane->plane,
						pfeat->color.encodings,
						pfeat->color.ranges,
						pfeat->color.default_encoding,
						pfeat->color.default_range);
	if (ret)
		return ERR_PTR(ret);

	if (pfeat->blend.global_alpha) {
		ret = drm_plane_create_alpha_property(&tplane->plane);
		if (ret)
			return ERR_PTR(ret);
	}

	return tplane;
}
