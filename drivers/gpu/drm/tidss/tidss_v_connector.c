// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Subhajit Paul <subhajit_paul@ti.com>
 */

/*
 * Virtual connectors : reuse connector_init logic from
 * drivers/gpu/drm/bridge/panel.c
 */

#include <drm/drmP.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include "tidss_v_display.h"

struct v_connector {
	struct drm_connector base;
	struct tidss_device *tidss;
	int id;
	struct drm_encoder *encoder;
	struct drm_display_mode mode;
};

#define to_v_connector(x) container_of(x, struct v_connector, base)

static int v_connector_get_modes(struct drm_connector *connector)
{
	struct v_connector *v_connector = to_v_connector(connector);
	struct drm_device *dev = connector->dev;
	struct drm_display_mode *mode;
	const struct drm_display_mode *m = &v_connector->mode;

	dev_dbg(connector->dev->dev, "%s\n", __func__);

	mode = drm_mode_duplicate(dev, m);
	if (!mode) {
		dev_err(dev->dev, "failed to add mode %ux%u@%u\n",
				m->hdisplay, m->vdisplay, m->vrefresh);
		return 0;
	}

	mode->type |= (DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED);

	drm_mode_set_name(mode);

	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_connector_helper_funcs v_connector_helper_funcs = {
	.get_modes = v_connector_get_modes,
};

static void v_connector_destroy(struct drm_connector *connector)
{
	struct v_connector *v_connector = to_v_connector(connector);

	dev_dbg(connector->dev->dev, "%s\n", __func__);

	drm_connector_cleanup(connector);
	kfree(v_connector);
}

static const struct drm_connector_funcs v_connector_funcs = {
	.reset = drm_atomic_helper_connector_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = v_connector_destroy,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

struct drm_connector *v_connector_init(struct tidss_device *tidss, int id,
		struct drm_encoder *encoder, struct rpmsg_remotedev_display_disp *vp)
{
	struct drm_device *dev = tidss->ddev;
	struct drm_connector *connector = NULL;
	struct v_connector *v_connector;
	unsigned int hfp, hbp, hsw, vfp, vbp, vsw;

	v_connector = kzalloc(sizeof(struct v_connector), GFP_KERNEL);
	if (!v_connector)
		return NULL;

	hfp = vp->width / 100;
	if (!hfp)
		hfp = 1;

	hbp = hfp / 2;
	if (!hbp)
		hbp = 1;

	hsw = hbp / 2;
	if (!hsw)
		hsw = 1;

	vfp = vp->height / 100;
	if (!vfp)
		vfp = 1;

	vbp = vfp / 2;
	if (!vbp)
		vbp = 1;

	vsw = vbp / 2;
	if (!vsw)
		vsw = 1;

	v_connector->tidss = tidss;

	v_connector->mode.hdisplay = vp->width;
	v_connector->mode.hsync_start = v_connector->mode.hdisplay + hfp;
	v_connector->mode.hsync_end = v_connector->mode.hsync_start + hsw;
	v_connector->mode.htotal = v_connector->mode.hsync_end + hbp;

	v_connector->mode.vdisplay = vp->height;
	v_connector->mode.vsync_start = v_connector->mode.vdisplay + vfp;
	v_connector->mode.vsync_end = v_connector->mode.vsync_start + vsw;
	v_connector->mode.vtotal = v_connector->mode.vsync_end + vbp;

	v_connector->mode.vrefresh = vp->refresh;

	v_connector->mode.clock = (v_connector->mode.vtotal * v_connector->mode.htotal * v_connector->mode.vrefresh) / 1000;

	v_connector->encoder = encoder;
	v_connector->id = id;

	connector = &v_connector->base;

	drm_connector_init(dev, connector, &v_connector_funcs,
				DRM_MODE_CONNECTOR_VIRTUAL);
	drm_connector_helper_add(connector, &v_connector_helper_funcs);

	drm_connector_attach_encoder(connector, encoder);

	return connector;
}

void v_connector_fini(struct tidss_device *tidss, struct drm_connector *connector)
{
	v_connector_destroy(connector);
}
