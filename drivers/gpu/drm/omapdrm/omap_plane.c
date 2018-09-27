/*
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
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

#define to_omap_plane_state(x) container_of(x, struct omap_plane_state, base)

struct omap_plane_state {
	/* Must be first */
	struct drm_plane_state base;

	unsigned int global_alpha;
	unsigned int pre_mult_alpha;

	struct omap_hw_overlay *overlay;
	struct omap_hw_overlay *r_overlay;  /* right overlay */
};

#define to_omap_plane(x) container_of(x, struct omap_plane, base)

struct omap_plane {
	struct drm_plane base;
	enum omap_plane_id default_id;
	const char *name;

	bool reserved_wb;
	/*
	 * WB has not notion of atomic state we need to keep
	 * a reference to the allocated overlay here.
	 */
	struct omap_hw_overlay *reserved_wb_overlay;
};

static const char *plane_id_to_name[];

bool is_omap_plane_dual_overlay(struct drm_plane_state *state)
{
	struct omap_plane_state *omap_state = to_omap_plane_state(state);

	return !!omap_state->r_overlay;
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

static bool plane_enabled(struct drm_plane_state *state)
{
	return state->visible;
}

static void omap_plane_atomic_update(struct drm_plane *plane,
				     struct drm_plane_state *old_state)
{
	struct omap_drm_private *priv = plane->dev->dev_private;
	struct omap_plane *omap_plane = to_omap_plane(plane);
	struct drm_plane_state *state = plane->state;
	struct omap_plane_state *new_omap_state;
	struct omap_plane_state *old_omap_state;
	struct omap_overlay_info info, r_info;
	enum omap_plane_id ovl_id, r_ovl_id;
	int ret;
	bool dual_plane;

	new_omap_state = to_omap_plane_state(state);
	old_omap_state = to_omap_plane_state(old_state);

	dual_plane = !!new_omap_state->r_overlay;

	/* Cleanup previously held overlay if needed */
	omap_overlay_disable(old_state->state, plane, old_omap_state->overlay);
	omap_overlay_disable(old_state->state, plane,
			     old_omap_state->r_overlay);

	if (!new_omap_state->overlay) {
		DBG("[PLANE:%d:%s] overlay_id: ??? (null)",
		    plane->base.id, plane->name);
		return;
	}

	ovl_id = new_omap_state->overlay->overlay_id;
	DBG("[PLANE:%d:%s] overlay_id: %d", plane->base.id, plane->name,
	    ovl_id);
	DBG("%s, crtc=%p fb=%p", omap_plane->name, state->crtc, state->fb);

	memset(&info, 0, sizeof(info));
	info.rotation_type = OMAP_DSS_ROT_NONE;
	info.rotation = DRM_MODE_ROTATE_0;
	info.global_alpha = new_omap_state->global_alpha;
	info.pre_mult_alpha = new_omap_state->pre_mult_alpha;
	info.zorder = state->normalized_zpos;
	info.color_encoding = state->color_encoding;
	info.color_range = state->color_range;

	r_info = info;

	/* update scanout: */
	omap_framebuffer_update_scanout(state->fb, state, &info,
					dual_plane ? &r_info : NULL);

	DBG("%s: %dx%d -> %dx%d (%d)",
	    overlay2name(ovl_id), info.width, info.height,
	    info.out_width, info.out_height, info.screen_width);
	DBG("%d,%d %pad %pad", info.pos_x, info.pos_y,
	    &info.paddr, &info.p_uv_addr);

	if (dual_plane) {
		r_ovl_id = new_omap_state->r_overlay->overlay_id;
		/*
		 * If the current plane uses 2 hw planes the very next
		 * zorder is used by the r_overlay so we just use the
		 * main overlay zorder + 1
		 */
		r_info.zorder = info.zorder + 1;

		DBG("%s: %dx%d -> %dx%d (%d)",
		    overlay2name(r_ovl_id), r_info.width, r_info.height,
		    r_info.out_width, r_info.out_height, r_info.screen_width);
		DBG("%d,%d %pad %pad", r_info.pos_x, r_info.pos_y,
		    &r_info.paddr, &r_info.p_uv_addr);
	}

	/* and finally, update omapdss: */
	ret = priv->dispc_ops->ovl_setup(ovl_id, &info,
			      omap_crtc_timings(state->crtc), false,
			      omap_crtc_channel(state->crtc));
	if (ret) {
		dev_err(plane->dev->dev, "Failed to setup plane1 %s\n",
			omap_plane->name);
		priv->dispc_ops->ovl_enable(ovl_id, false);
		return;
	}

	priv->dispc_ops->ovl_enable(ovl_id, true);

	if (dual_plane) {
		ret = priv->dispc_ops->ovl_setup(r_ovl_id, &r_info,
				      omap_crtc_timings(state->crtc), false,
				      omap_crtc_channel(state->crtc));
		if (ret) {
			dev_err(plane->dev->dev, "Failed to setup plane2 %s\n",
				omap_plane->name);
			priv->dispc_ops->ovl_enable(r_ovl_id, false);
			priv->dispc_ops->ovl_enable(ovl_id, false);
			return;
		}

		priv->dispc_ops->ovl_enable(r_ovl_id, true);
	}
}

static void omap_plane_atomic_disable(struct drm_plane *plane,
				      struct drm_plane_state *old_state)
{
	struct drm_plane_state *state = plane->state;
	struct omap_plane_state *new_omap_state;
	struct omap_plane_state *old_omap_state;
	bool dual_plane;

	new_omap_state = to_omap_plane_state(state);
	old_omap_state = to_omap_plane_state(old_state);

	dual_plane = !!old_omap_state->r_overlay;

	DBG("%s: old plane state %p  new plane state %p", plane->name,
	    old_state, plane->state);

	DBG("%s: check old (overlay %p r_overlay %p)", plane->name,
	    old_omap_state->overlay, old_omap_state->r_overlay);
	DBG("%s: check new (overlay %p r_overlay %p)", plane->name,
	    new_omap_state->overlay, new_omap_state->r_overlay);

	if (!old_omap_state->overlay)
		return;

	plane->state->rotation = DRM_MODE_ROTATE_0;
	plane->state->zpos = plane->type == DRM_PLANE_TYPE_PRIMARY
			   ? 0 : old_omap_state->overlay->overlay_id;

	omap_overlay_disable(old_state->state, plane, old_omap_state->overlay);
	new_omap_state->overlay = NULL;
	if (dual_plane) {
		omap_overlay_disable(old_state->state, plane,
				     old_omap_state->r_overlay);
		new_omap_state->r_overlay = NULL;
	}
}

#define FRAC_16_16(mult, div)    (((mult) << 16) / (div))
static int omap_plane_atomic_check(struct drm_plane *plane,
				   struct drm_plane_state *state)
{
	struct omap_drm_private *priv = plane->dev->dev_private;
	struct drm_crtc *crtc;
	struct drm_crtc_state *crtc_state;
	struct omap_plane *omap_plane = to_omap_plane(plane);
	u16 width, height;
	u32 width_fp, height_fp;
	struct drm_plane_state *old_state = plane->state;
	struct omap_plane_state *omap_state = to_omap_plane_state(state);
	struct omap_global_state *omap_overlay_global_state;
	u32 crtc_mask;
	u32 fourcc;
	u32 caps = 0;
	struct drm_rect clip = {};
	bool new_hw_overlay = false;
	bool new_r_hw_overlay = false;
	bool is_fourcc_yuv = false;
	int min_scale, max_scale;
	int ret;

	if (omap_plane->reserved_wb)
		return -EBUSY;

	omap_overlay_global_state = omap_get_global_state(state->state);
	if (IS_ERR(omap_overlay_global_state))
		return PTR_ERR(omap_overlay_global_state);
	DBG("%s: omap_overlay_global_state: %p", plane->name,
	    omap_overlay_global_state);

	priv->dispc_ops->ovl_get_max_size(&width, &height);
	width_fp = width << 16;
	height_fp = height << 16;

	crtc = state->crtc ? state->crtc : plane->state->crtc;
	if (!crtc)
		return 0;

	crtc_state = drm_atomic_get_existing_crtc_state(state->state, crtc);
	/* we should have a crtc state if the plane is attached to a crtc */
	if (WARN_ON(!crtc_state))
		return 0;

	/* Make sure src/destination dimensions are within bounds. */
	if (state->src_h > height_fp || state->crtc_h > height)
		return -EINVAL;

	if (state->fb)
		is_fourcc_yuv = (state->fb->format->hsub == 2);

	if (state->src_w > width_fp || state->crtc_w > width) {
		/*
		 * We cannot have dual plane/overlay and trans_key_mode
		 * enabled concurrently, hence rejecting this configuration
		 */
		if (omap_crtc_atomic_get_trans_key_mode(crtc, crtc_state))
			return -EINVAL;

		if (is_fourcc_yuv &&
		    (((state->src_w >> 16) / 2 & 1) ||
		     state->crtc_w / 2 & 1)) {
			/*
			 * When calculating the split overlay width
			 * and it yield an odd value we will need to adjust
			 * the indivual width +/- 1. So make sure it fits
			 */
			if (state->src_w <= ((2 * width - 1) << 16) &&
			    state->crtc_w <= (2 * width - 1))
				new_r_hw_overlay = true;
			else
				return -EINVAL;
		} else {
			if (state->src_w <= (2 * width_fp) &&
			    state->crtc_w <= (2 * width))
				new_r_hw_overlay = true;
			else
				return -EINVAL;
		}
	}

	clip.x2 = crtc_state->adjusted_mode.hdisplay;
	clip.y2 = crtc_state->adjusted_mode.vdisplay;
	min_scale = FRAC_16_16(1, 4);
	max_scale = FRAC_16_16(8, 1);

	ret = drm_plane_helper_check_state(state, &clip,
					   min_scale, max_scale,
					   true, true);
	if (ret)
		return ret;

	if (state->rotation != DRM_MODE_ROTATE_0 &&
	    !omap_framebuffer_supports_rotation(state->fb))
		return -EINVAL;

	DBG("%s: check (%d -> %d)", plane->name,
	    plane_enabled(old_state), plane_enabled(state));

	if (plane_enabled(state)) {
		if ((state->src_w >> 16) != state->crtc_w ||
		    (state->src_h >> 16) != state->crtc_h)
			caps |= OMAP_DSS_OVL_CAP_SCALE;

		fourcc = state->fb->format->format;
		crtc_mask = drm_crtc_mask(state->crtc);

		/*
		 * (re)allocate hw overlay if we don't have one or
		 * there is a caps mismatch
		 */
		if (!omap_state->overlay ||
		    (caps & ~omap_state->overlay->caps)) {
			new_hw_overlay = true;
		} else {
			/* check if allowed on crtc */
			if (!(omap_state->overlay->possible_crtcs & crtc_mask))
				new_hw_overlay = true;

			/* check supported format */
			if (!priv->dispc_ops->ovl_color_mode_supported(
						omap_state->overlay->overlay_id,
						fourcc))
				new_hw_overlay = true;
		}
		/*
		 * check if we need two overlays and only have 1 or
		 * if we had 2 overlays but will only need 1
		 */
		if ((new_r_hw_overlay && !omap_state->r_overlay) ||
		    (!new_r_hw_overlay && omap_state->r_overlay))
			new_hw_overlay = true;

		if (new_hw_overlay) {
			struct omap_hw_overlay *old_ovl =
						omap_state->overlay;
			struct omap_hw_overlay *old_r_ovl =
						omap_state->r_overlay;
			struct omap_hw_overlay *new_ovl = NULL;
			struct omap_hw_overlay *new_r_ovl = NULL;

			omap_overlay_release(state->state, plane, old_ovl);
			omap_overlay_release(state->state, plane, old_r_ovl);

			ret = omap_overlay_assign(state->state, plane, caps,
						  fourcc, crtc_mask, &new_ovl,
						  new_r_hw_overlay ?
						  &new_r_ovl : NULL);
			if (ret) {
				DBG("%s: failed to assign hw_overlay(s)!",
				    plane->name);
				omap_state->overlay = NULL;
				omap_state->r_overlay = NULL;
				return ret;
			}

			omap_state->overlay = new_ovl;
			if (new_r_hw_overlay)
				omap_state->r_overlay = new_r_ovl;
			else
				omap_state->r_overlay = NULL;
		}
	} else {
		omap_overlay_release(state->state, plane, omap_state->overlay);
		omap_overlay_release(state->state, plane,
				     omap_state->r_overlay);
		omap_state->overlay = NULL;
		omap_state->r_overlay = NULL;
	}

	if (omap_state->overlay)
		DBG("plane: %s overlay_id: %d", plane->name,
		    omap_state->overlay->overlay_id);
	if (omap_state->r_overlay)
		DBG("plane: %s r_overlay_id: %d", plane->name,
		    omap_state->r_overlay->overlay_id);

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
							   DRM_MODE_ROTATE_0,
							   DRM_MODE_ROTATE_0 | DRM_MODE_ROTATE_90 |
							   DRM_MODE_ROTATE_180 | DRM_MODE_ROTATE_270 |
							   DRM_MODE_REFLECT_X | DRM_MODE_REFLECT_Y);

		/* Attach the rotation property also to the crtc object */
		if (plane->rotation_property && obj != &plane->base)
			drm_object_attach_property(obj, plane->rotation_property,
						   DRM_MODE_ROTATE_0);
	}

	drm_object_attach_property(obj, priv->zorder_prop, 0);
}

static void omap_plane_reset(struct drm_plane *plane)
{
	struct omap_plane *omap_plane = to_omap_plane(plane);
	struct omap_plane_state *omap_state;

	if (plane->state)
		__drm_atomic_helper_plane_destroy_state(plane->state);

	kfree(plane->state);
	plane->state = kzalloc(sizeof(struct omap_plane_state), GFP_KERNEL);
	if (!plane->state)
		return;

	omap_state = to_omap_plane_state(plane->state);

	plane->state->plane = plane;
	plane->state->rotation = DRM_MODE_ROTATE_0;

	/*
	 * Set the zpos default depending on whether we are a primary or overlay
	 * plane.
	 */
	plane->state->zpos = plane->type == DRM_PLANE_TYPE_PRIMARY
			   ? 0 : omap_plane->default_id;

	omap_state->global_alpha = 0xff;
	omap_state->pre_mult_alpha = 0;
	omap_state->base.color_encoding = DRM_COLOR_YCBCR_BT601;
	omap_state->base.color_range = DRM_COLOR_YCBCR_FULL_RANGE;
}

static struct drm_plane_state *
omap_plane_duplicate_state(struct drm_plane *plane)
{
	struct omap_plane_state *state, *current_state;

	if (WARN_ON(!plane->state))
		return NULL;

	current_state = to_omap_plane_state(plane->state);

	state = kmalloc(sizeof(*state), GFP_KERNEL);
	if (!state)
		return NULL;

	__drm_atomic_helper_plane_duplicate_state(plane, &state->base);

	state->global_alpha = current_state->global_alpha;
	state->pre_mult_alpha = current_state->pre_mult_alpha;
	state->overlay = current_state->overlay;
	state->r_overlay = current_state->r_overlay;

	return &state->base;
}

static void omap_plane_atomic_print_state(struct drm_printer *p,
					  const struct drm_plane_state *state)
{
	struct omap_plane_state *omap_state = to_omap_plane_state(state);

	drm_printf(p, "\toverlay=%p\n", omap_state->overlay);
	if (omap_state->overlay) {
		drm_printf(p, "\t\tidx=%d\n", omap_state->overlay->idx);
		drm_printf(p, "\t\toverlay_id=%d\n",
			   omap_state->overlay->overlay_id);
		drm_printf(p, "\t\tcaps=0x%x\n", omap_state->overlay->caps);
		drm_printf(p, "\t\tpossible_crtcs=0x%x\n",
			   omap_state->overlay->possible_crtcs);
	}

	drm_printf(p, "\tr_overlay=%p\n", omap_state->r_overlay);
	if (omap_state->r_overlay) {
		drm_printf(p, "\t\tidx=%d\n", omap_state->r_overlay->idx);
		drm_printf(p, "\t\toverlay_id=%d\n",
			   omap_state->r_overlay->overlay_id);
		drm_printf(p, "\t\tcaps=0x%x\n", omap_state->r_overlay->caps);
		drm_printf(p, "\t\tpossible_crtcs=0x%x\n",
			   omap_state->r_overlay->possible_crtcs);
	}
}

static int omap_plane_atomic_set_property(struct drm_plane *plane,
					  struct drm_plane_state *state,
					  struct drm_property *property,
					  uint64_t val)
{
	struct omap_drm_private *priv = plane->dev->dev_private;
	struct omap_plane_state *omap_state = to_omap_plane_state(state);

	if (property == priv->zorder_prop)
		state->zpos = val;
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
	const struct omap_plane_state *omap_state = to_omap_plane_state(state);

	if (property == priv->zorder_prop)
		*val = state->zpos;
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
	.atomic_duplicate_state = omap_plane_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_plane_destroy_state,
	.atomic_set_property = omap_plane_atomic_set_property,
	.atomic_get_property = omap_plane_atomic_get_property,
	.atomic_print_state = omap_plane_atomic_print_state,
};

static bool omap_plane_supports_yuv(struct drm_plane *plane)
{
	struct omap_drm_private *priv = plane->dev->dev_private;
	struct omap_plane *omap_plane = to_omap_plane(plane);
	const u32 *formats =
		priv->dispc_ops->ovl_get_color_modes(omap_plane->default_id);
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

struct drm_plane *omap_plane_init(struct drm_device *dev,
		int idx, enum drm_plane_type type,
		u32 possible_crtcs)
{
	struct omap_drm_private *priv = dev->dev_private;
	unsigned int num_planes = priv->dispc_ops->get_num_ovls();
	struct drm_plane *plane;
	struct omap_plane *omap_plane;
	int ret;
	u32 nformats;
	const u32 *formats;

	if (WARN_ON(idx >= num_planes))
		return ERR_PTR(-EINVAL);

	omap_plane = kzalloc(sizeof(*omap_plane), GFP_KERNEL);
	if (!omap_plane)
		return ERR_PTR(-ENOMEM);

	omap_plane->default_id = idx;
	omap_plane->name = plane_id_to_name[idx];

	DBG("%s: type=%d", omap_plane->name, type);
	DBG("	omap_plane->default_id: %d", omap_plane->default_id);
	DBG("	crtc_mask: 0x%04x", possible_crtcs);

	formats = priv->dispc_ops->ovl_get_color_modes(omap_plane->default_id);
	for (nformats = 0; formats[nformats]; ++nformats)
		;

	plane = &omap_plane->base;

	ret = drm_universal_plane_init(dev, plane, possible_crtcs,
				       &omap_plane_funcs, formats,
				       nformats, NULL, type, NULL);
	if (ret < 0)
		goto error;

	drm_plane_helper_add(plane, &omap_plane_helper_funcs);

	omap_plane_install_properties(plane, &plane->base);
	drm_plane_create_zpos_property(plane, 0, 0, num_planes - 1);

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
		__func__, omap_plane->name);

	kfree(omap_plane);
	return NULL;
}

enum omap_plane_id omap_plane_id_wb(struct drm_plane *plane)
{
	struct omap_plane *omap_plane = to_omap_plane(plane);

	return omap_plane->reserved_wb_overlay->overlay_id;
}

struct drm_plane *omap_plane_reserve_wb(struct drm_device *dev)
{
	struct omap_drm_private *priv = dev->dev_private;
	int i, ret;

	/*
	 * Look from the last plane to the first to lessen chances of the
	 * display side trying to use the same plane as writeback.
	 */
	for (i = priv->num_planes - 1; i >= 0; --i) {
		struct drm_plane *plane = priv->planes[i];
		struct omap_plane *omap_plane = to_omap_plane(plane);
		struct omap_hw_overlay *new_ovl = NULL;
		u32 crtc_mask = (1 << priv->num_crtcs) - 1;

		if (plane->crtc || plane->fb)
			continue;

		if (omap_plane->reserved_wb)
			continue;

		ret = omap_overlay_assign_wb(priv, plane, crtc_mask, &new_ovl);
		if (ret) {
			DBG("%s: failed to assign hw_overlay for wb!",
			    plane->name);
			return NULL;
		}

		omap_plane->reserved_wb = true;
		omap_plane->reserved_wb_overlay = new_ovl;

		return plane;
	}

	return NULL;
}

void omap_plane_release_wb(struct drm_plane *plane)
{
	struct omap_drm_private *priv = plane->dev->dev_private;
	struct omap_plane *omap_plane;

	/*
	 * This is also called on module unload at which point plane might
	 * not be set. In that case just return as there is nothing to do.
	 */
	if (!plane)
		return;

	omap_plane = to_omap_plane(plane);

	omap_overlay_release_wb(priv, plane, omap_plane->reserved_wb_overlay);
	omap_plane->reserved_wb = false;
	omap_plane->reserved_wb_overlay = NULL;
}
