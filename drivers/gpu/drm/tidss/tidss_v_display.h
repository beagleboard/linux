/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Subhajit Paul <subhajit_paul@ti.com>
 */

#ifndef __TIDSS_V_DISPLAY_H__
#define __TIDSS_V_DISPLAY_H__

#include "tidss_drv.h"

struct drm_encoder *v_encoder_init(struct tidss_device *tidss, int id,
		int crtc_mask, struct rpmsg_remotedev_display_disp *vp);
struct drm_connector *v_connector_init(struct tidss_device *tidss, int id,
		struct drm_encoder *encoder, struct rpmsg_remotedev_display_disp *vp);
struct drm_plane *v_plane_init(struct tidss_device *tidss, int id,
		int crtc_bitmask, enum drm_plane_type type, struct rpmsg_remotedev_display_disp *vp, int index);
struct drm_crtc *v_crtc_init(struct tidss_device *tidss, int id,
		struct drm_plane *plane, struct rpmsg_remotedev_display_disp *vp);

void v_crtc_fini(struct tidss_device *tidss, struct drm_crtc *crtc);
void v_plane_fini(struct tidss_device *tidss, struct drm_plane *plane);
void v_connector_fini(struct tidss_device *tidss, struct drm_connector *connector);
void v_encoder_fini(struct tidss_device *tidss, struct drm_encoder *encoder);

unsigned int v_plane_get_remote_id(struct drm_plane *plane);
bool v_plane_update_needed(struct drm_plane *plane);

void v_crtc_commit_done(struct rpmsg_remotedev_display_commit *commit, void *data);
void v_crtc_buffer_done(struct rpmsg_remotedev_display_buffer *buffer, void *data);
#endif
