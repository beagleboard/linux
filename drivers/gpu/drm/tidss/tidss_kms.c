// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
 */

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>

#include "tidss_crtc.h"
#include "tidss_drv.h"
#include "tidss_encoder.h"
#include "tidss_kms.h"
#include "tidss_plane.h"

static void tidss_atomic_commit_tail(struct drm_atomic_state *old_state)
{
	struct drm_device *ddev = old_state->dev;
	struct tidss_device *tidss = ddev->dev_private;

	dev_dbg(ddev->dev, "%s\n", __func__);

	tidss->dispc_ops->runtime_get(tidss->dispc);

	drm_atomic_helper_commit_modeset_disables(ddev, old_state);
	drm_atomic_helper_commit_planes(ddev, old_state, 0);
	drm_atomic_helper_commit_modeset_enables(ddev, old_state);

	drm_atomic_helper_commit_hw_done(old_state);
	drm_atomic_helper_wait_for_flip_done(ddev, old_state);

	drm_atomic_helper_cleanup_planes(ddev, old_state);

	tidss->dispc_ops->runtime_put(tidss->dispc);
}

static const struct drm_mode_config_helper_funcs mode_config_helper_funcs = {
	.atomic_commit_tail = tidss_atomic_commit_tail,
};

static void tidss_output_poll_changed(struct drm_device *dev)
{
	struct tidss_device *tidss = dev->dev_private;

	drm_fbdev_cma_hotplug_event(tidss->fbdev);
}

static const struct drm_mode_config_funcs mode_config_funcs = {
	.fb_create = drm_gem_fb_create,
	.output_poll_changed = tidss_output_poll_changed,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

int tidss_modeset_init(struct tidss_device *tidss)
{
	struct drm_device *ddev = tidss->ddev;
	int ret;
	int i;

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
		return ret;

	ret = drm_vblank_init(ddev, tidss->num_crtcs);
	if (ret)
		return ret;

	/* Start with vertical blanking interrupt reporting disabled. */
	for (i = 0; i < tidss->num_crtcs; ++i)
		drm_crtc_vblank_reset(tidss->crtcs[i]);

	drm_mode_config_reset(ddev);

	dev_dbg(tidss->dev, "%s done\n", __func__);

	return 0;
}
