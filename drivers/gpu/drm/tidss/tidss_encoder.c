// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
 */

#include <linux/export.h>

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_panel.h>
#include <drm/drm_of.h>

#include "tidss_drv.h"
#include "tidss_encoder.h"
#include "tidss_crtc.h"

/* -----------------------------------------------------------------------------
 * Encoder
 */

static void tidss_encoder_disable(struct drm_encoder *encoder)
{
	struct drm_device *ddev = encoder->dev;

	dev_dbg(ddev->dev, "%s\n", __func__);

}

static void tidss_encoder_enable(struct drm_encoder *encoder)
{
	struct drm_device *ddev = encoder->dev;

	dev_dbg(ddev->dev, "%s\n", __func__);

}

static int tidss_encoder_atomic_check(struct drm_encoder *encoder,
				      struct drm_crtc_state *crtc_state,
				      struct drm_connector_state *conn_state)
{
	struct drm_device *ddev = encoder->dev;
	struct tidss_crtc_state *tcrtc_state = to_tidss_crtc_state(crtc_state);
	struct drm_display_info *di = &conn_state->connector->display_info;

	dev_dbg(ddev->dev, "%s\n", __func__);

	// XXX any cleaner way to set bus format and flags?
	tcrtc_state->bus_format = di->bus_formats[0];
	tcrtc_state->bus_flags = di->bus_flags;

	return 0;
}

static void tidss_encoder_mode_set(struct drm_encoder *encoder,
				   struct drm_crtc_state *crtc_state,
				   struct drm_connector_state *conn_state)
{
	struct drm_device *ddev = encoder->dev;

	dev_dbg(ddev->dev, "%s\n", __func__);

}

static const struct drm_encoder_helper_funcs encoder_helper_funcs = {
	.atomic_mode_set = tidss_encoder_mode_set,
	.disable = tidss_encoder_disable,
	.enable = tidss_encoder_enable,
	.atomic_check = tidss_encoder_atomic_check,
};

static const struct drm_encoder_funcs encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

struct tidss_encoder *tidss_encoder_create(struct tidss_device *tidss,
					   u32 encoder_type, u32 possible_crtcs)
{
	struct tidss_encoder *tenc;
	struct drm_encoder *enc;
	int ret;

	tenc = devm_kzalloc(tidss->dev, sizeof(*tenc), GFP_KERNEL);
	if (!tenc)
		return ERR_PTR(-ENOMEM);

	enc = &tenc->encoder;
	enc->possible_crtcs = possible_crtcs;

	ret = drm_encoder_init(tidss->ddev, enc, &encoder_funcs,
			       encoder_type, NULL);
	if (ret < 0)
		return ERR_PTR(ret);

	drm_encoder_helper_add(enc, &encoder_helper_funcs);

	dev_dbg(tidss->dev, "Encoder create done\n");

	return tenc;
}
