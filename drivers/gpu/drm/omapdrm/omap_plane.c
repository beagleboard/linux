/*
 * drivers/gpu/drm/omapdrm/omap_plane.c
 *
 * Copyright (C) 2011 Texas Instruments
 * Author: Rob Clark <rob.clark@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_plane_helper.h>

#include "omap_dmm_tiler.h"
#include "omap_drv.h"

/*
 * plane funcs
 */

#define to_omap_plane(x) container_of(x, struct omap_plane, base)

struct omap_plane {
	struct drm_plane base;
	enum omap_plane_id id;
	const char *name;

	bool reserved_wb;
};

struct omap_plane_state {
	struct drm_plane_state base;

	unsigned int zorder;
	unsigned int global_alpha;
	unsigned int pre_mult_alpha;
};

static inline struct omap_plane_state *
to_omap_plane_state(struct drm_plane_state *state)
{
	return container_of(state, struct omap_plane_state, base);
}

static int omap_plane_prepare_fb(struct drm_plane *plane,
				 struct drm_plane_state *new_state)
{
	if (!new_state->fb)
		return 0;

	return omap_framebuffer_pin(new_state->fb);
}

static void omap_plane_cleanup_fb(struct drm_plane *plane,
				  struct drm_plane_state *old_state)
{
	if (old_state->fb)
		omap_framebuffer_unpin(old_state->fb);
}

static void omap_plane_atomic_update(struct drm_plane *plane,
				     struct drm_plane_state *old_state)
{
	struct omap_drm_private *priv = plane->dev->dev_private;
	struct omap_plane *omap_plane = to_omap_plane(plane);
	struct drm_plane_state *state = plane->state;
	struct omap_plane_state *omap_state = to_omap_plane_state(state);
	struct omap_overlay_info info;
	int ret;

	DBG("%s, crtc=%p fb=%p", omap_plane->name, state->crtc, state->fb);

	if (!state->crtc->state->enable) {
		priv->dispc_ops->ovl_enable(omap_plane->id, false);
		return;
	}

	memset(&info, 0, sizeof(info));
	info.rotation_type = OMAP_DSS_ROT_NONE;
	info.rotation = DRM_ROTATE_0;
	info.zorder = omap_state->zorder;
	info.global_alpha = omap_state->global_alpha;
	info.pre_mult_alpha = omap_state->pre_mult_alpha;
	info.color_encoding = state->color_encoding;
	info.color_range = state->color_range;

	/* update scanout: */
	omap_framebuffer_update_scanout(state->fb, state, &info);

	DBG("%dx%d -> %dx%d (%d)", info.width, info.height,
			info.out_width, info.out_height,
			info.screen_width);
	DBG("%d,%d %pad %pad", info.pos_x, info.pos_y,
			&info.paddr, &info.p_uv_addr);

	/* and finally, update omapdss: */
	ret = priv->dispc_ops->ovl_setup(omap_plane->id, &info,
			      omap_crtc_timings(state->crtc), false,
			      omap_crtc_channel(state->crtc));
	if (ret) {
		dev_err(plane->dev->dev, "Failed to setup plane %s\n",
			omap_plane->name);
		priv->dispc_ops->ovl_enable(omap_plane->id, false);
		return;
	}

	priv->dispc_ops->ovl_enable(omap_plane->id, true);
}

static void omap_plane_atomic_disable(struct drm_plane *plane,
				      struct drm_plane_state *old_state)
{
	struct omap_drm_private *priv = plane->dev->dev_private;
	struct omap_plane_state *omap_state = to_omap_plane_state(plane->state);
	struct omap_plane *omap_plane = to_omap_plane(plane);

	plane->state->rotation = DRM_ROTATE_0;
	omap_state->zorder = plane->type == DRM_PLANE_TYPE_PRIMARY
			   ? 0 : omap_plane->id;

	priv->dispc_ops->ovl_enable(omap_plane->id, false);
}

static int omap_plane_atomic_check(struct drm_plane *plane,
				   struct drm_plane_state *state)
{
	struct drm_crtc_state *crtc_state;
	struct omap_plane *omap_plane = to_omap_plane(plane);

	if (omap_plane->reserved_wb)
		return -EBUSY;

	if (!state->fb)
		return 0;

	/* crtc should only be NULL when disabling (i.e., !state->fb) */
	if (WARN_ON(!state->crtc))
		return 0;

	crtc_state = drm_atomic_get_existing_crtc_state(state->state, state->crtc);
	/* we should have a crtc state if the plane is attached to a crtc */
	if (WARN_ON(!crtc_state))
		return 0;

	if (!crtc_state->enable)
		return 0;

	if (state->src_w == 0 || state->src_h == 0)
		return -EINVAL;

	if (state->crtc_w == 0 || state->crtc_h == 0)
		return -EINVAL;

	if (state->crtc_x < 0 || state->crtc_y < 0)
		return -EINVAL;

	if (state->crtc_x + state->crtc_w > crtc_state->adjusted_mode.hdisplay)
		return -EINVAL;

	if (state->crtc_y + state->crtc_h > crtc_state->adjusted_mode.vdisplay)
		return -EINVAL;

	if (state->rotation != DRM_ROTATE_0 &&
	    !omap_framebuffer_supports_rotation(state->fb))
		return -EINVAL;

	return 0;
}

static const struct drm_plane_helper_funcs omap_plane_helper_funcs = {
	.prepare_fb = omap_plane_prepare_fb,
	.cleanup_fb = omap_plane_cleanup_fb,
	.atomic_check = omap_plane_atomic_check,
	.atomic_update = omap_plane_atomic_update,
	.atomic_disable = omap_plane_atomic_disable,
};

static void omap_plane_destroy(struct drm_plane *plane)
{
	struct omap_plane *omap_plane = to_omap_plane(plane);

	DBG("%s", omap_plane->name);

	drm_plane_cleanup(plane);

	kfree(omap_plane);
}

/* helper to install properties which are common to planes and crtcs */
void omap_plane_install_properties(struct drm_plane *plane,
		struct drm_mode_object *obj)
{
	struct drm_device *dev = plane->dev;
	struct omap_drm_private *priv = dev->dev_private;

	if (priv->has_dmm) {
		if (!plane->rotation_property)
			drm_plane_create_rotation_property(plane,
							   DRM_ROTATE_0,
							   DRM_ROTATE_0 | DRM_ROTATE_90 |
							   DRM_ROTATE_180 | DRM_ROTATE_270 |
							   DRM_REFLECT_X | DRM_REFLECT_Y);

		/* Attach the rotation property also to the crtc object */
		if (plane->rotation_property && obj != &plane->base)
			drm_object_attach_property(obj, plane->rotation_property,
						   DRM_ROTATE_0);
	}

	drm_object_attach_property(obj, priv->zorder_prop, 0);
}

static struct drm_plane_state *
omap_plane_atomic_duplicate_state(struct drm_plane *plane)
{
	struct omap_plane_state *state;
	struct omap_plane_state *copy;

	if (WARN_ON(!plane->state))
		return NULL;

	state = to_omap_plane_state(plane->state);
	copy = kmemdup(state, sizeof(*state), GFP_KERNEL);
	if (copy == NULL)
		return NULL;

	__drm_atomic_helper_plane_duplicate_state(plane, &copy->base);

	return &copy->base;
}

static void omap_plane_atomic_destroy_state(struct drm_plane *plane,
					    struct drm_plane_state *state)
{
	__drm_atomic_helper_plane_destroy_state(state);
	kfree(to_omap_plane_state(state));
}

static void omap_plane_reset(struct drm_plane *plane)
{
	struct omap_plane *omap_plane = to_omap_plane(plane);
	struct omap_plane_state *omap_state;

	if (plane->state) {
		omap_plane_atomic_destroy_state(plane, plane->state);
		plane->state = NULL;
	}

	omap_state = kzalloc(sizeof(*omap_state), GFP_KERNEL);
	if (omap_state == NULL)
		return;

	/*
	 * Set defaults depending on whether we are a primary or overlay
	 * plane.
	 */
	omap_state->zorder = plane->type == DRM_PLANE_TYPE_PRIMARY
			   ? 0 : omap_plane->id;
	omap_state->base.rotation = DRM_ROTATE_0;
	omap_state->global_alpha = 0xff;
	omap_state->pre_mult_alpha = 0;
	omap_state->base.color_encoding = DRM_COLOR_YCBCR_BT601;
	omap_state->base.color_range = DRM_COLOR_YCBCR_FULL_RANGE;

	plane->state = &omap_state->base;
	plane->state->plane = plane;
}

static int omap_plane_atomic_set_property(struct drm_plane *plane,
					  struct drm_plane_state *state,
					  struct drm_property *property,
					  uint64_t val)
{
	struct omap_drm_private *priv = plane->dev->dev_private;
	struct omap_plane_state *omap_state = to_omap_plane_state(state);

	if (property == priv->zorder_prop)
		omap_state->zorder = val;
	else if (property == priv->global_alpha_prop)
		omap_state->global_alpha = val;
	else if (property == priv->pre_mult_alpha_prop)
		omap_state->pre_mult_alpha = val;
	else
		return -EINVAL;

	return 0;
}

static int omap_plane_atomic_get_property(struct drm_plane *plane,
					  const struct drm_plane_state *state,
					  struct drm_property *property,
					  uint64_t *val)
{
	struct omap_drm_private *priv = plane->dev->dev_private;
	const struct omap_plane_state *omap_state =
		container_of(state, const struct omap_plane_state, base);

	if (property == priv->zorder_prop)
		*val = omap_state->zorder;
	else if (property == priv->global_alpha_prop)
		*val = omap_state->global_alpha;
	else if (property == priv->pre_mult_alpha_prop)
		*val = omap_state->pre_mult_alpha;
	else
		return -EINVAL;

	return 0;
}

static const struct drm_plane_funcs omap_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.reset = omap_plane_reset,
	.destroy = omap_plane_destroy,
	.set_property = drm_atomic_helper_plane_set_property,
	.atomic_duplicate_state = omap_plane_atomic_duplicate_state,
	.atomic_destroy_state = omap_plane_atomic_destroy_state,
	.atomic_set_property = omap_plane_atomic_set_property,
	.atomic_get_property = omap_plane_atomic_get_property,
};

static bool omap_plane_supports_yuv(struct drm_plane *plane)
{
	struct omap_drm_private *priv = plane->dev->dev_private;
	struct omap_plane *omap_plane = to_omap_plane(plane);
	const u32 *formats =
		priv->dispc_ops->ovl_get_color_modes(omap_plane->id);
	int i;

	for (i = 0; formats[i]; i++)
		if (formats[i] == DRM_FORMAT_YUYV ||
		    formats[i] == DRM_FORMAT_UYVY ||
		    formats[i] == DRM_FORMAT_NV12)
			return true;

	return false;
}

static const char *plane_id_to_name[] = {
	[OMAP_DSS_GFX] = "gfx",
	[OMAP_DSS_VIDEO1] = "vid1",
	[OMAP_DSS_VIDEO2] = "vid2",
	[OMAP_DSS_VIDEO3] = "vid3",
};

static const enum omap_plane_id plane_idx_to_id[] = {
	OMAP_DSS_GFX,
	OMAP_DSS_VIDEO1,
	OMAP_DSS_VIDEO2,
	OMAP_DSS_VIDEO3,
};

/* initialize plane */
struct drm_plane *omap_plane_init(struct drm_device *dev,
		int idx, enum drm_plane_type type,
		u32 possible_crtcs)
{
	struct omap_drm_private *priv = dev->dev_private;
	struct drm_plane *plane;
	struct omap_plane *omap_plane;
	enum omap_plane_id id;
	int ret;
	u32 nformats;
	const u32 *formats;

	if (WARN_ON(idx >= ARRAY_SIZE(plane_idx_to_id)))
		return ERR_PTR(-EINVAL);

	id = plane_idx_to_id[idx];

	DBG("%s: type=%d", plane_id_to_name[id], type);

	omap_plane = kzalloc(sizeof(*omap_plane), GFP_KERNEL);
	if (!omap_plane)
		return ERR_PTR(-ENOMEM);

	formats = priv->dispc_ops->ovl_get_color_modes(id);
	for (nformats = 0; formats[nformats]; ++nformats)
		;
	omap_plane->id = id;
	omap_plane->name = plane_id_to_name[id];

	plane = &omap_plane->base;

	ret = drm_universal_plane_init(dev, plane, possible_crtcs,
				       &omap_plane_funcs, formats,
				       nformats, type, NULL);
	if (ret < 0)
		goto error;

	drm_plane_helper_add(plane, &omap_plane_helper_funcs);

	omap_plane_install_properties(plane, &plane->base);

	drm_object_attach_property(&plane->base, priv->global_alpha_prop, 0);
	drm_object_attach_property(&plane->base, priv->pre_mult_alpha_prop, 0);

	if (omap_plane_supports_yuv(plane))
		drm_plane_create_color_properties(plane,
					BIT(DRM_COLOR_YCBCR_BT601) |
					BIT(DRM_COLOR_YCBCR_BT709),
					BIT(DRM_COLOR_YCBCR_FULL_RANGE) |
					BIT(DRM_COLOR_YCBCR_LIMITED_RANGE),
					DRM_COLOR_YCBCR_BT601,
					DRM_COLOR_YCBCR_FULL_RANGE);

	return plane;

error:
	dev_err(dev->dev, "%s(): could not create plane: %s\n",
		__func__, plane_id_to_name[id]);

	kfree(omap_plane);
	return NULL;
}

int omap_plane_id(struct drm_plane *plane)
{
	struct omap_plane *omap_plane = to_omap_plane(plane);

	return omap_plane->id;
}

struct drm_plane *omap_plane_reserve_wb(struct drm_device *dev)
{
	struct omap_drm_private *priv = dev->dev_private;
	int i;

	/*
	 * Look from the last plane to the first to lessen chances of the
	 * display side trying to use the same plane as writeback.
	 */
	for (i = priv->num_planes - 1; i >= 0; --i) {
		struct drm_plane *plane = priv->planes[i];
		struct omap_plane *omap_plane = to_omap_plane(plane);

		if (plane->crtc || plane->fb)
			continue;

		if (omap_plane->reserved_wb)
			continue;

		omap_plane->reserved_wb = true;

		return plane;
	}

	return NULL;
}

void omap_plane_release_wb(struct drm_plane *plane)
{
	struct omap_plane *omap_plane = to_omap_plane(plane);

	WARN_ON(!omap_plane->reserved_wb);

	omap_plane->reserved_wb = false;
}
