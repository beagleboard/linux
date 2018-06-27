/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
 */

#ifndef __TIDSS_IRQ_H__
#define __TIDSS_IRQ_H__

#include <linux/types.h>

struct drm_crtc;
struct drm_device;

void tidss_irq_enable_vblank(struct drm_crtc *crtc);
void tidss_irq_disable_vblank(struct drm_crtc *crtc);

void tidss_irq_preinstall(struct drm_device *ddev);
int tidss_irq_postinstall(struct drm_device *ddev);
void tidss_irq_uninstall(struct drm_device *ddev);
irqreturn_t tidss_irq_handler(int irq, void *arg);

void tidss_irq_resume(struct drm_device *ddev);

#endif
