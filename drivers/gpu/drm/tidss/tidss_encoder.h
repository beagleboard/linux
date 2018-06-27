/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
 */

#ifndef __TIDSS_ENCODER_H__
#define __TIDSS_ENCODER_H__

#include <drm/drm_encoder.h>

struct tidss_device;
struct tidss_crtc;

struct tidss_encoder {
	struct drm_encoder encoder;
};

struct tidss_encoder *tidss_encoder_create(struct tidss_device *tidss,
					   u32 encoder_type, u32 possible_crtcs);

#endif
