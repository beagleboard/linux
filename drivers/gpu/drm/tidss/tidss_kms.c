// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
 */

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>

#include "tidss_crtc.h"
#include "tidss_drv.h"
#include "tidss_encoder.h"
#include "tidss_kms.h"
#include "tidss_plane.h"
#include "tidss_v_display.h"

static bool tidss_is_v_crtc(struct drm_crtc *crtc)
{
	int i;
	struct drm_device *dev = crtc->dev;
	struct tidss_device *tidss = dev->dev_private;

	for (i = 0; i < tidss->num_v_crtcs; i++)
		if (crtc == tidss->v_crtcs[i])
			return true;
	return false;
}

static void tidss_atomic_commit_tail(struct drm_atomic_state *old_state)
{
	struct drm_device *ddev = old_state->dev;
	struct tidss_device *tidss = ddev->dev_private;
	struct drm_crtc *crtc;
	struct drm_crtc_state *old_crtc_state, *new_crtc_state;
	int i;

	dev_dbg(ddev->dev, "%s\n", __func__);

	for_each_oldnew_crtc_in_state(old_state, crtc, old_crtc_state, new_crtc_state, i)
		if (!tidss_is_v_crtc(crtc))
			tidss->dispc_ops->runtime_get(tidss->dispc);

	drm_atomic_helper_commit_modeset_disables(ddev, old_state);
	drm_atomic_helper_commit_planes(ddev, old_state, 0);
	drm_atomic_helper_commit_modeset_enables(ddev, old_state);

	drm_atomic_helper_commit_hw_done(old_state);
	drm_atomic_helper_wait_for_flip_done(ddev, old_state);

	drm_atomic_helper_cleanup_planes(ddev, old_state);

	for_each_oldnew_crtc_in_state(old_state, crtc, old_crtc_state, new_crtc_state, i)
		if (!tidss_is_v_crtc(crtc))
			tidss->dispc_ops->runtime_put(tidss->dispc);
}

static const struct drm_mode_config_helper_funcs mode_config_helper_funcs = {
	.atomic_commit_tail = tidss_atomic_commit_tail,
};

static const struct drm_mode_config_funcs mode_config_funcs = {
	.fb_create = drm_gem_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static struct drm_crtc *tidss_v_modeset_init_v_crtc(struct tidss_device *tidss, struct rpmsg_remotedev_display_disp *vp)
{
	struct drm_device *dev = tidss->ddev;
	struct drm_encoder *encoder;
	struct drm_connector *connector;
	struct drm_plane *planes[8];
	struct drm_crtc *crtc;
	int last_crtc = tidss->num_crtcs + tidss->num_v_crtcs;
	int num_planes = 0;
	int p;

	encoder = v_encoder_init(tidss, last_crtc, 1 << last_crtc, vp);
	if (!encoder) {
		dev_err(dev->dev, "could not create encoder: %u\n", last_crtc);
		return NULL;
	}

	connector = v_connector_init(tidss, last_crtc, encoder, vp);
	if (!connector) {
		dev_err(dev->dev, "could not create connector: %u\n", last_crtc);
		goto connector_fail;
	}

	for (p = 0; p < vp->num_pipes; p++) {
		int plane_id = p << 8 | last_crtc;

		planes[p] = v_plane_init(tidss, plane_id, (1 << last_crtc),
				p == 0 ? DRM_PLANE_TYPE_PRIMARY : DRM_PLANE_TYPE_OVERLAY,
				vp, p);
		if (!planes[p]) {
			dev_err(dev->dev, "could not create plane: %u\n", plane_id);
			goto plane_fail;
		}
		num_planes++;
	}

	crtc = v_crtc_init(tidss, last_crtc, planes[0], vp);
	if (!crtc) {
		dev_err(dev->dev, "could not create crtc: %d\n", last_crtc);
		goto plane_fail;
	}

	tidss->num_v_crtcs++;

	return crtc;

plane_fail:
	for (p = 0; p < num_planes; p++)
		v_plane_fini(tidss, planes[p]);
	v_connector_fini(tidss, connector);
connector_fail:
	v_encoder_fini(tidss, encoder);
	return NULL;
}

static const struct rpmsg_remotedev_display_cb tidss_rdev_cb  = {
	.commit_done = v_crtc_commit_done,
	.buffer_done = v_crtc_buffer_done,
};

int tidss_modeset_init(struct tidss_device *tidss)
{
	struct drm_device *ddev = tidss->ddev;
	unsigned int i;
	int ret;

	dev_dbg(tidss->dev, "%s\n", __func__);

	drm_mode_config_init(ddev);

	ddev->mode_config.min_width = 8;
	ddev->mode_config.min_height = 8;
	ddev->mode_config.max_width = 8096;
	ddev->mode_config.max_height = 8096;
	ddev->mode_config.normalize_zpos = true;
	ddev->mode_config.funcs = &mode_config_funcs;
	ddev->mode_config.helper_private = &mode_config_helper_funcs;

	ret = tidss->dispc_ops->modeset_init(tidss->dispc);
	if (ret)
		goto err_mode_config_cleanup;

	if (tidss->rdev) {
		tidss->rdev->device.display.ops->get_res_info(tidss->rdev, &tidss->rres);

		if (tidss->rres.num_disps)
			tidss->rdev->device.display.cb_ops = &tidss_rdev_cb;

		for (i = 0; i < tidss->rres.num_disps; i++) {
			tidss->v_crtcs[i] = tidss_v_modeset_init_v_crtc(tidss, &tidss->rres.disps[i]);
			if (!tidss->v_crtcs[i]) {
				ret = -ENOMEM;
				goto err_mode_config_cleanup;
			}
		}
	}

	ret = drm_vblank_init(ddev, tidss->num_crtcs + tidss->num_v_crtcs);
	if (ret)
		goto err_mode_config_cleanup;

	/* Start with vertical blanking interrupt reporting disabled. */
	for (i = 0; i < tidss->num_crtcs; ++i)
		drm_crtc_vblank_reset(tidss->crtcs[i]);

	for (i = 0; i < tidss->num_v_crtcs; ++i)
		drm_crtc_vblank_reset(tidss->v_crtcs[i]);

	drm_mode_config_reset(ddev);

	dev_dbg(tidss->dev, "%s done\n", __func__);

	return 0;

err_mode_config_cleanup:
	drm_mode_config_cleanup(ddev);
	return ret;
}

void tidss_modeset_cleanup(struct tidss_device *tidss)
{
	struct drm_device *ddev = tidss->ddev;

	drm_mode_config_cleanup(ddev);
}
