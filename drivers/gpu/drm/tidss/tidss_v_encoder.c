// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Subhajit Paul <subhajit_paul@ti.com>
 */

/*
 * Virtual encoders : reuse drivers/gpu/drm/tidss/tidss_encoder.c
 */

#include <drm/drmP.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include "tidss_v_display.h"

struct v_encoder {
	struct drm_encoder base;
	struct tidss_device *tidss;
	int id;
	int possible_crtcs;
};

#define to_v_encoder(x) container_of(x, struct v_encoder, base)

static void v_encoder_disable(struct drm_encoder *encoder)
{
	dev_dbg(encoder->dev->dev, "%s\n", __func__);
}

static void v_encoder_enable(struct drm_encoder *encoder)
{
	dev_dbg(encoder->dev->dev, "%s\n", __func__);
}

static int v_encoder_atomic_check(struct drm_encoder *encoder,
				      struct drm_crtc_state *crtc_state,
				      struct drm_connector_state *conn_state)
{
	dev_dbg(encoder->dev->dev, "%s\n", __func__);

	return 0;
}

static void v_encoder_atomic_mode_set(struct drm_encoder *encoder,
				   struct drm_crtc_state *crtc_state,
				   struct drm_connector_state *conn_state)
{
	dev_dbg(encoder->dev->dev, "%s\n", __func__);
}

static const struct drm_encoder_helper_funcs v_encoder_helper_funcs = {
	.atomic_mode_set = v_encoder_atomic_mode_set,
	.disable = v_encoder_disable,
	.enable = v_encoder_enable,
	.atomic_check = v_encoder_atomic_check,
};

static void v_encoder_destroy(struct drm_encoder *encoder)
{
	struct v_encoder *v_encoder = to_v_encoder(encoder);

	dev_dbg(encoder->dev->dev, "%s\n", __func__);

	drm_encoder_cleanup(encoder);
	kfree(v_encoder);
}

static const struct drm_encoder_funcs v_encoder_funcs = {
	.destroy = v_encoder_destroy,
};

struct drm_encoder *v_encoder_init(struct tidss_device *tidss, int id, int crtc_mask,
		struct rpmsg_remotedev_display_disp *vp)
{
	struct drm_device *dev = tidss->ddev;
	struct drm_encoder *encoder = NULL;
	struct v_encoder *v_encoder;

	v_encoder = kzalloc(sizeof(*v_encoder), GFP_KERNEL);
	if (!v_encoder)
		return NULL;

	v_encoder->tidss = tidss;
	v_encoder->id = id;
	v_encoder->possible_crtcs = crtc_mask;

	encoder = &v_encoder->base;
	encoder->possible_crtcs = crtc_mask;

	drm_encoder_init(dev, encoder, &v_encoder_funcs,
			 DRM_MODE_ENCODER_VIRTUAL, "encoder-0x%x", id);
	drm_encoder_helper_add(encoder, &v_encoder_helper_funcs);

	return encoder;
}

void v_encoder_fini(struct tidss_device *tidss, struct drm_encoder *encoder)
{
	v_encoder_destroy(encoder);
}
