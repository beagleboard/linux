// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
 */

#include <drm/drmP.h>

#include "tidss_irq.h"
#include "tidss_drv.h"
#include "tidss_dispc.h"
#include "tidss_crtc.h"
#include "tidss_plane.h"

/* call with wait_lock and dispc runtime held */
static void tidss_irq_update(struct drm_device *ddev)
{
	struct tidss_device *tidss = ddev->dev_private;

	assert_spin_locked(&tidss->wait_lock);

	tidss->dispc_ops->write_irqenable(tidss->dispc, tidss->irq_mask);
}

void tidss_irq_enable_vblank(struct drm_crtc *crtc)
{
	struct drm_device *ddev = crtc->dev;
	struct tidss_device *tidss = ddev->dev_private;
	struct tidss_crtc *tcrtc = to_tidss_crtc(crtc);
	u32 hw_videoport = tcrtc->hw_videoport;
	unsigned long flags;

	spin_lock_irqsave(&tidss->wait_lock, flags);
	tidss->irq_mask |= DSS_IRQ_VP_VSYNC_EVEN(hw_videoport) |
			   DSS_IRQ_VP_VSYNC_ODD(hw_videoport);
	tidss_irq_update(ddev);
	spin_unlock_irqrestore(&tidss->wait_lock, flags);
}

void tidss_irq_disable_vblank(struct drm_crtc *crtc)
{
	struct drm_device *ddev = crtc->dev;
	struct tidss_device *tidss = ddev->dev_private;
	struct tidss_crtc *tcrtc = to_tidss_crtc(crtc);
	u32 hw_videoport = tcrtc->hw_videoport;
	unsigned long flags;

	spin_lock_irqsave(&tidss->wait_lock, flags);
	tidss->irq_mask &= ~(DSS_IRQ_VP_VSYNC_EVEN(hw_videoport) |
			     DSS_IRQ_VP_VSYNC_ODD(hw_videoport));
	tidss_irq_update(ddev);
	spin_unlock_irqrestore(&tidss->wait_lock, flags);
}

static void tidss_irq_fifo_underflow(struct tidss_device *tidss,
				     u64 irqstatus)
{
	static DEFINE_RATELIMIT_STATE(_rs, DEFAULT_RATELIMIT_INTERVAL,
				      DEFAULT_RATELIMIT_BURST);
	unsigned int i;
	u64 masked;

	spin_lock(&tidss->wait_lock);
	masked = irqstatus & tidss->irq_uf_mask & tidss->irq_mask;
	spin_unlock(&tidss->wait_lock);

	if (!masked)
		return;

	if (!__ratelimit(&_rs))
		return;

	DRM_ERROR("FIFO underflow on ");

	for (i = 0; i < DSS_MAX_PLANES; ++i) {
		if (masked & DSS_IRQ_PLANE_FIFO_UNDERFLOW(i))
			pr_cont("%u:%s ", i,
				tidss->dispc_ops->plane_name(tidss->dispc, i));
	}

	pr_cont("(%016llx)\n", irqstatus);
}

static void tidss_irq_ocp_error_handler(struct drm_device *ddev,
					u64 irqstatus)
{
	if (irqstatus & DSS_IRQ_DEVICE_OCP_ERR)
		dev_err_ratelimited(ddev->dev, "OCP error\n");
}

irqreturn_t tidss_irq_handler(int irq, void *arg)
{
	struct drm_device *ddev = (struct drm_device *) arg;
	struct tidss_device *tidss = ddev->dev_private;
	unsigned int id;
	u64 irqstatus;

	if (WARN_ON(!ddev->irq_enabled))
		return IRQ_NONE;

	irqstatus = tidss->dispc_ops->read_and_clear_irqstatus(tidss->dispc);

	for (id = 0; id < tidss->num_crtcs; id++) {
		struct drm_crtc *crtc = tidss->crtcs[id];
		struct tidss_crtc *tcrtc = to_tidss_crtc(crtc);
		u32 hw_videoport = tcrtc->hw_videoport;

		if (irqstatus & (DSS_IRQ_VP_VSYNC_EVEN(hw_videoport) |
				 DSS_IRQ_VP_VSYNC_ODD(hw_videoport)))
			tidss_crtc_vblank_irq(crtc);

		if (irqstatus & (DSS_IRQ_VP_FRAME_DONE(hw_videoport)))
			tidss_crtc_framedone_irq(crtc);

		if (irqstatus & DSS_IRQ_VP_SYNC_LOST(hw_videoport))
			tidss_crtc_error_irq(crtc, irqstatus);
	}

	tidss_irq_ocp_error_handler(ddev, irqstatus);
	tidss_irq_fifo_underflow(tidss, irqstatus);

	return IRQ_HANDLED;
}

void tidss_irq_preinstall(struct drm_device *ddev)
{
	struct tidss_device *tidss = ddev->dev_private;

	spin_lock_init(&tidss->wait_lock);

	tidss->dispc_ops->runtime_get(tidss->dispc);

	tidss->dispc_ops->write_irqenable(tidss->dispc, 0);
	tidss->dispc_ops->read_and_clear_irqstatus(tidss->dispc);

	tidss->dispc_ops->runtime_put(tidss->dispc);
}

int tidss_irq_postinstall(struct drm_device *ddev)
{
	struct tidss_device *tidss = ddev->dev_private;
	unsigned int i;
	unsigned long flags;

	tidss->dispc_ops->runtime_get(tidss->dispc);

	spin_lock_irqsave(&tidss->wait_lock, flags);

	tidss->irq_mask = DSS_IRQ_DEVICE_OCP_ERR;

	tidss->irq_uf_mask = 0;
	for (i = 0; i < tidss->num_planes; ++i) {
		struct tidss_plane *tplane = to_tidss_plane(tidss->planes[i]);

		tidss->irq_uf_mask |= DSS_IRQ_PLANE_FIFO_UNDERFLOW(tplane->hw_plane_id);
	}
	tidss->irq_mask |= tidss->irq_uf_mask;

	for (i = 0; i < tidss->num_crtcs; ++i) {
		struct tidss_crtc *tcrtc = to_tidss_crtc(tidss->crtcs[i]);

		tidss->irq_mask |= DSS_IRQ_VP_SYNC_LOST(tcrtc->hw_videoport);

		tidss->irq_mask |= DSS_IRQ_VP_FRAME_DONE(tcrtc->hw_videoport);
	}

	tidss_irq_update(ddev);

	spin_unlock_irqrestore(&tidss->wait_lock, flags);

	tidss->dispc_ops->runtime_put(tidss->dispc);

	return 0;
}

void tidss_irq_uninstall(struct drm_device *ddev)
{
	struct tidss_device *tidss = ddev->dev_private;

	tidss->dispc_ops->runtime_get(tidss->dispc);
	tidss->dispc_ops->write_irqenable(tidss->dispc, 0);
	tidss->dispc_ops->runtime_put(tidss->dispc);
}

void tidss_irq_resume(struct drm_device *ddev)
{
	struct tidss_device *tidss = ddev->dev_private;
	unsigned long flags;

	spin_lock_irqsave(&tidss->wait_lock, flags);
	tidss_irq_update(ddev);
	spin_unlock_irqrestore(&tidss->wait_lock, flags);
}
