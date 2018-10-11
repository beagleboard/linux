// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Rob Clark <rob.clark@linaro.org>
 */

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_fourcc.h>

#include "omap_dmm_tiler.h"
#include "omap_drv.h"

/*
 * plane funcs
 */

#define to_omap_plane_state(x) container_of(x, struct omap_plane_state, base)

struct omap_plane_state {
	/* Must be first. */
	struct drm_plane_state base;

	struct omap_hw_overlay *overlay;
	struct omap_hw_overlay *r_overlay;  /* right overlay */
};

#define to_omap_plane(x) container_of(x, struct omap_plane, base)

struct omap_plane {
	struct drm_plane base;
	enum omap_plane_id id;
	const char *name;
};

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
	bool dual_ovl;

	new_omap_state = to_omap_plane_state(state);
	old_omap_state = to_omap_plane_state(old_state);

	dual_ovl = is_omap_plane_dual_overlay(state);

	/* Cleanup previously held overlay if needed */
	omap_overlay_disable(old_state->state, plane, old_omap_state->overlay);
	omap_overlay_disable(old_state->state, plane,
			     old_omap_state->r_overlay);

	if (!new_omap_state->overlay) {
		DBG("[PLANE:%d:%s] overlay_id: ??? (%p)", plane->base.id, plane->name,
		    new_omap_state->overlay);
		return;
	}

	ovl_id = new_omap_state->overlay->overlay_id;
	DBG("[PLANE:%d:%s] overlay_id: %d", plane->base.id, plane->name,
	    ovl_id);
	DBG("%s, crtc=%p fb=%p", omap_plane->name, state->crtc, state->fb);

	memset(&info, 0, sizeof(info));
	info.rotation_type = OMAP_DSS_ROT_NONE;
	info.rotation = DRM_MODE_ROTATE_0;
	info.global_alpha = state->alpha >> 8;
	info.zorder = state->normalized_zpos;
	if (state->pixel_blend_mode == DRM_MODE_BLEND_PREMULTI)
		info.pre_mult_alpha = 1;
	else
		info.pre_mult_alpha = 0;
	info.color_encoding = state->color_encoding;
	info.color_range = state->color_range;

	r_info = info;

	/* update scanout: */
	omap_framebuffer_update_scanout(state->fb, state, &info,
					dual_ovl ? &r_info : NULL);

	DBG("%s: %dx%d -> %dx%d (%d)",
	    new_omap_state->overlay->name, info.width, info.height,
	    info.out_width, info.out_height, info.screen_width);
	DBG("%d,%d %pad %pad", info.pos_x, info.pos_y,
	    &info.paddr, &info.p_uv_addr);

	if (dual_ovl) {
		r_ovl_id = new_omap_state->r_overlay->overlay_id;
		/*
		 * If the current plane uses 2 hw planes the very next
		 * zorder is used by the r_overlay so we just use the
		 * main overlay zorder + 1
		 */
		r_info.zorder = info.zorder + 1;

		DBG("%s: %dx%d -> %dx%d (%d)",
		    new_omap_state->r_overlay->name,
		    r_info.width, r_info.height,
		    r_info.out_width, r_info.out_height, r_info.screen_width);
		DBG("%d,%d %pad %pad", r_info.pos_x, r_info.pos_y,
		    &r_info.paddr, &r_info.p_uv_addr);
	}

	/* and finally, update omapdss: */
	ret = priv->dispc_ops->ovl_setup(priv->dispc, ovl_id, &info,
			      omap_crtc_timings(state->crtc), false,
			      omap_crtc_channel(state->crtc));
	if (ret) {
		dev_err(plane->dev->dev, "Failed to setup plane1 %s\n",
			omap_plane->name);
		priv->dispc_ops->ovl_enable(priv->dispc, ovl_id, false);
		return;
	}

	priv->dispc_ops->ovl_enable(priv->dispc, ovl_id, true);

	if (dual_ovl) {
		ret = priv->dispc_ops->ovl_setup(priv->dispc, r_ovl_id, &r_info,
				      omap_crtc_timings(state->crtc), false,
				      omap_crtc_channel(state->crtc));
		if (ret) {
			dev_err(plane->dev->dev, "Failed to setup plane2 %s\n",
				omap_plane->name);
			priv->dispc_ops->ovl_enable(priv->dispc, r_ovl_id, false);
			priv->dispc_ops->ovl_enable(priv->dispc, ovl_id, false);
			return;
		}

		priv->dispc_ops->ovl_enable(priv->dispc, r_ovl_id, true);
	}
}

static void omap_plane_atomic_disable(struct drm_plane *plane,
				      struct drm_plane_state *old_state)
{
	struct drm_plane_state *state = plane->state;
	struct omap_plane_state *new_omap_state;
	struct omap_plane_state *old_omap_state;

	new_omap_state = to_omap_plane_state(state);
	old_omap_state = to_omap_plane_state(old_state);

	if (!old_omap_state->overlay)
		return;

	plane->state->rotation = DRM_MODE_ROTATE_0;
	plane->state->zpos = plane->type == DRM_PLANE_TYPE_PRIMARY
			   ? 0 : old_omap_state->overlay->overlay_id;

	omap_overlay_disable(old_state->state, plane, old_omap_state->overlay);
	new_omap_state->overlay = NULL;
	if (is_omap_plane_dual_overlay(old_state)) {
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
	u16 width, height;
	u32 width_fp, height_fp;
	struct drm_plane_state *old_state = plane->state;
	struct omap_plane_state *omap_state = to_omap_plane_state(state);
	struct omap_global_state *omap_overlay_global_state;
	u32 crtc_mask;
	u32 fourcc;
	u32 caps = 0;
	bool new_hw_overlay = false;
	bool new_r_hw_overlay = false;
	bool is_fourcc_yuv = false;
	int min_scale, max_scale;
	int ret;

	omap_overlay_global_state = omap_get_global_state(state->state);
	if (IS_ERR(omap_overlay_global_state))
		return PTR_ERR(omap_overlay_global_state);
	DBG("%s: omap_overlay_global_state: %p", plane->name,
	    omap_overlay_global_state);

	priv->dispc_ops->ovl_get_max_size(priv->dispc, &width, &height);
	width_fp = width << 16;
	height_fp = height << 16;

	crtc = state->crtc ? state->crtc : plane->state->crtc;
	if (!crtc)
		return 0;

	crtc_state = drm_atomic_get_existing_crtc_state(state->state, crtc);
	/* we should have a crtc state if the plane is attached to a crtc */
	if (WARN_ON(!crtc_state))
		return 0;

	/* Make sure dimensions are within bounds. */
	if (state->src_h > height_fp || state->crtc_h > height)
		return -EINVAL;

	if (state->fb)
		is_fourcc_yuv = state->fb->format->is_yuv;

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

	/*
	 * Note: these are just sanity checks to filter out totally bad scaling
	 * factors. The real limits must be calculated case by case, and
	 * unfortunately we currently do those checks only at the commit
	 * phase in dispc.
	 */
	min_scale = FRAC_16_16(1, 8);
	max_scale = FRAC_16_16(8, 1);

	ret = drm_atomic_helper_check_plane_state(state, crtc_state,
						  min_scale, max_scale,
						  true, true);
	if (ret)
		return ret;

	DBG("%s: check (%d -> %d)", plane->name,
	    old_state->visible, state->visible);

	if (state->visible) {
		if (state->rotation != DRM_MODE_ROTATE_0 &&
		    !omap_framebuffer_supports_rotation(state->fb))
			return -EINVAL;

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
			if (!priv->dispc_ops->ovl_color_mode_supported(priv->dispc,
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
		drm_atomic_helper_plane_destroy_state(plane, plane->state);

	omap_state = kzalloc(sizeof(*omap_state), GFP_KERNEL);
	if (!omap_state)
		return;

	__drm_atomic_helper_plane_reset(plane, &omap_state->base);

	/*
	 * Set the zpos default depending on whether we are a primary or overlay
	 * plane.
	 */
	plane->state->zpos = plane->type == DRM_PLANE_TYPE_PRIMARY
			   ? 0 : omap_plane->id;
	plane->state->color_encoding = DRM_COLOR_YCBCR_BT601;
	plane->state->color_range = DRM_COLOR_YCBCR_FULL_RANGE;
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
	if (!copy)
		return NULL;

	__drm_atomic_helper_plane_duplicate_state(plane, &copy->base);

	return &copy->base;
}

static void omap_plane_atomic_print_state(struct drm_printer *p,
					  const struct drm_plane_state *state)
{
	struct omap_plane_state *omap_state = to_omap_plane_state(state);

	drm_printf(p, "\toverlay=%s\n", omap_state->overlay ?
					omap_state->overlay->name : "(null)");
	if (omap_state->overlay) {
		drm_printf(p, "\t\tidx=%d\n", omap_state->overlay->idx);
		drm_printf(p, "\t\toverlay_id=%d\n",
			   omap_state->overlay->overlay_id);
		drm_printf(p, "\t\tcaps=0x%x\n", omap_state->overlay->caps);
		drm_printf(p, "\t\tpossible_crtcs=0x%x\n",
			   omap_state->overlay->possible_crtcs);
	}

	drm_printf(p, "\tr_overlay=%s\n", omap_state->r_overlay ?
					  omap_state->r_overlay->name :
					  "(null)");
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
					  u64 val)
{
	struct omap_drm_private *priv = plane->dev->dev_private;

	if (property == priv->zorder_prop)
		state->zpos = val;
	else
		return -EINVAL;

	return 0;
}

static int omap_plane_atomic_get_property(struct drm_plane *plane,
					  const struct drm_plane_state *state,
					  struct drm_property *property,
					  u64 *val)
{
	struct omap_drm_private *priv = plane->dev->dev_private;

	if (property == priv->zorder_prop)
		*val = state->zpos;
	else
		return -EINVAL;

	return 0;
}

static const struct drm_plane_funcs omap_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.reset = omap_plane_reset,
	.destroy = omap_plane_destroy,
	.atomic_duplicate_state = omap_plane_atomic_duplicate_state,
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
		priv->dispc_ops->ovl_get_color_modes(priv->dispc, omap_plane->id);
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

/* initialize plane */
struct drm_plane *omap_plane_init(struct drm_device *dev,
		int idx, enum drm_plane_type type,
		u32 possible_crtcs)
{
	struct omap_drm_private *priv = dev->dev_private;
	unsigned int num_planes = priv->dispc_ops->get_num_ovls(priv->dispc);
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

	omap_plane->id = idx;
	omap_plane->name = plane_id_to_name[idx];

	DBG("%s: type=%d", omap_plane->name, type);
	DBG("	omap_plane->id: %d", omap_plane->id);
	DBG("	crtc_mask: 0x%04x", possible_crtcs);

	formats = priv->dispc_ops->ovl_get_color_modes(priv->dispc,
						       omap_plane->id);
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
	drm_plane_create_alpha_property(plane);
	drm_plane_create_blend_mode_property(plane, BIT(DRM_MODE_BLEND_PREMULTI) |
					     BIT(DRM_MODE_BLEND_COVERAGE));

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
