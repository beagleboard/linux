// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Subhajit Paul <subhajit_paul@ti.com>
 */

/*
 * Virtual crtcs: reuse drivers/gpu/drm/tidss/tidss_crtc.c
 */

#include <drm/drmP.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_plane.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include "tidss_v_display.h"

static DEFINE_MUTEX(out_buffers_lock);
static LIST_HEAD(out_buffers_list);

struct v_crtc {
	struct drm_crtc base;
	struct tidss_device *tidss;
	int id;
	unsigned int remote_id;

	bool disable_in_progress;
	bool enabled;
	struct drm_pending_vblank_event *event;

	struct mutex vsync_lock;

	struct completion framedone_completion;
	struct work_struct fake_vsync;

};

struct v_crtc_buffer_ref {
	struct drm_framebuffer *fb;
	atomic_t refcount;
	struct list_head node;
};

#define to_v_crtc(x) container_of(x, struct v_crtc, base)

static void v_crtc_finish_page_flip(struct drm_crtc *crtc)
{
	struct v_crtc *v_crtc = to_v_crtc(crtc);
	struct drm_pending_vblank_event *event;
	unsigned long flags;

	spin_lock_irqsave(&crtc->dev->event_lock, flags);

	event = v_crtc->event;
	v_crtc->event = NULL;

	if (!event) {
		spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
		return;
	}

	drm_crtc_send_vblank_event(crtc, event);

	spin_unlock_irqrestore(&crtc->dev->event_lock, flags);

	drm_crtc_vblank_put(crtc);
}

static void v_crtc_vblank_irq(struct drm_crtc *crtc)
{
	struct v_crtc *v_crtc = to_v_crtc(crtc);

	drm_crtc_handle_vblank(crtc);

	v_crtc_finish_page_flip(crtc);

	mutex_lock(&v_crtc->vsync_lock);
	if (v_crtc->disable_in_progress) {
		v_crtc->disable_in_progress = false;
		complete(&v_crtc->framedone_completion);
	}
	mutex_unlock(&v_crtc->vsync_lock);

}

void v_crtc_commit_done(struct rpmsg_remotedev_display_commit *commit, void *data)
{
	struct drm_crtc *crtc = commit->priv;

	v_crtc_vblank_irq(crtc);

	kfree(commit);
}

void v_crtc_buffer_done(struct rpmsg_remotedev_display_buffer *buffer, void *data)
{
	struct v_crtc_buffer_ref *ref = buffer->priv;

	mutex_lock(&out_buffers_lock);
	if (atomic_dec_and_test(&ref->refcount)) {
		list_del(&ref->node);
		kfree(ref);
		// TODO release the fence here
	}
	mutex_unlock(&out_buffers_lock);

	kfree(buffer);
}

static void fake_vsync_fn(struct work_struct *work)
{
	struct v_crtc *v_crtc = container_of(work, struct v_crtc, fake_vsync);
	struct drm_crtc *crtc = &v_crtc->base;

	v_crtc_vblank_irq(crtc);
}


static u32 v_crtc_update_plane_mask(struct drm_crtc *crtc, struct drm_crtc_state *old, struct drm_crtc_state *new)
{
	struct drm_plane *plane;
	u32 update_plane_mask = 0;
	u32 cont_u_planes = 0;
	u32 new_planes = ~old->plane_mask & new->plane_mask;
	u32 old_planes = ~new->plane_mask & old->plane_mask;
	u32 cont_planes = old->plane_mask & new->plane_mask;

	drm_for_each_plane_mask(plane, crtc->dev, cont_planes)
		if (v_plane_update_needed(plane))
			cont_u_planes |= (1 << drm_plane_index(plane));


	update_plane_mask = new_planes | old_planes | cont_u_planes;

	return update_plane_mask;
}

static struct rpmsg_remotedev_display_buffer *v_crtc_buffer_alloc_for_req(struct drm_crtc *crtc, struct drm_framebuffer *fb,
		struct drm_plane_state *pstate)
{
	struct v_crtc *v_crtc = to_v_crtc(crtc);
	struct rpmsg_remotedev_display_buffer *buffer;
	struct v_crtc_buffer_ref *ref, *buffer_ref = NULL;
	int cnt;

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer)
		return NULL;


	mutex_lock(&out_buffers_lock);
	list_for_each_entry(ref, &out_buffers_list, node) {
		if (ref->fb == fb) {
			atomic_inc(&ref->refcount);
			buffer_ref = ref;
			break;
		}
	}
	if (buffer_ref)
		goto found;

	buffer_ref = kzalloc(sizeof(*buffer_ref), GFP_KERNEL);
	if (!buffer_ref) {
		mutex_unlock(&out_buffers_lock);
		return NULL;
	}
	buffer_ref->fb = fb;
	atomic_set(&buffer_ref->refcount, 1);
	// TODO acquire fence here
	list_add(&buffer_ref->node, &out_buffers_list);

found:
	mutex_unlock(&out_buffers_lock);

	buffer->width = pstate->src_w >> 16;
	buffer->height = pstate->src_h >> 16;
	buffer->format = fb->format->format;
	buffer->num_planes = min_t(u32, fb->format->num_planes, RPMSG_REMOTEDEV_DISPLAY_MAX_PLANES);
	WARN_ON(buffer->num_planes != fb->format->num_planes);

	for (cnt = 0; cnt < buffer->num_planes; cnt++) {
		struct drm_gem_cma_object *gem;

		gem = drm_fb_cma_get_gem_obj(fb, cnt);
		buffer->planes[cnt] = gem->paddr + fb->offsets[cnt] +
			(pstate->src_x >> 16) * fb->format->cpp[cnt] +
			(pstate->src_y >> 16) * fb->pitches[cnt];
		buffer->pitches[cnt] = fb->pitches[cnt];
	}

	buffer->rdev = v_crtc->tidss->rdev;
	buffer->priv = buffer_ref;

	return buffer;
}

static bool v_crtc_plane_state_to_vid_update_info(struct drm_crtc *crtc, struct drm_plane *plane,
		struct rpmsg_remotedev_display_pipe_update *vid)
{
	struct drm_plane_state *pstate;

	if (WARN_ON(!plane || !plane->state))
		return false;

	pstate = plane->state;

	vid->pipe_id = v_plane_get_remote_id(plane);

	if (!pstate->fb) {
		vid->enabled = false;
		dev_dbg(plane->dev->dev, "%s: disabling plane 0x%x\n", __func__, v_plane_get_remote_id(plane));
	} else {
		struct drm_framebuffer *fb = pstate->fb;
		struct rpmsg_remotedev_display_buffer *buffer;

		buffer = v_crtc_buffer_alloc_for_req(crtc, fb, pstate);
		if (!buffer) {
			dev_err(plane->dev->dev, "%s: could not allocate buffer\n", __func__);
			return false;
		}

		if (WARN_ON(!fb->format))
			return false;

		vid->enabled = true;
		vid->dst_w = pstate->crtc_w;
		vid->dst_h = pstate->crtc_h;
		vid->dst_x = pstate->crtc_x;
		vid->dst_y = pstate->crtc_y;

		vid->buffer = buffer;
		dev_dbg(plane->dev->dev, "%s: updating plane 0x%x\n", __func__, v_plane_get_remote_id(plane));

	}

	return true;
}


static void v_crtc_flush_to_remote(struct drm_crtc *crtc, u32 planes, char *stage)
{
	struct v_crtc *v_crtc = to_v_crtc(crtc);
	struct drm_plane *plane;
	int i = 0;
	struct rpmsg_remotedev_display_commit *commit;

	if (!planes)
		return;

	commit = kzalloc(sizeof(*commit), GFP_KERNEL);
	if (!commit)
		return;

	dev_dbg(crtc->dev->dev, "%s: flushing out vp in %s = 0x%x\n", __func__, stage, v_crtc->remote_id);
	drm_for_each_plane_mask(plane, crtc->dev, planes) {
		bool ret;

		if (WARN_ON(i >= RPMSG_REMOTEDEV_DISPLAY_MAX_PIPES))
			continue;
		ret = v_crtc_plane_state_to_vid_update_info(crtc, plane, &commit->pipes[i]);
		if (!ret) {
			dev_err(crtc->dev->dev, "%s: error creating vid commit req [%d]\n", __func__, i);
			return;
		}
		i++;
	}

	commit->disp_id = v_crtc->remote_id;
	commit->num_pipe_updates = i;
	commit->priv = crtc;
	commit->rdev = v_crtc->tidss->rdev;

	v_crtc->tidss->rdev->device.display.ops->commit(v_crtc->tidss->rdev, commit);
}

static int v_crtc_atomic_check(struct drm_crtc *crtc, struct drm_crtc_state *state)
{
	dev_dbg(crtc->dev->dev, "%s\n", __func__);

	return 0;
}

static void v_crtc_atomic_begin(struct drm_crtc *crtc, struct drm_crtc_state *old_crtc_state)
{
	dev_dbg(crtc->dev->dev, "%s\n", __func__);
}

static enum drm_mode_status v_crtc_mode_valid(struct drm_crtc *crtc,
					   const struct drm_display_mode *mode)
{
	dev_dbg(crtc->dev->dev, "%s\n", __func__);

	return MODE_OK;
}

static void v_crtc_atomic_enable(struct drm_crtc *crtc,
				     struct drm_crtc_state *old_state)
{
	struct v_crtc *v_crtc = to_v_crtc(crtc);
	u32 upd_planes;

	dev_dbg(crtc->dev->dev, "%s\n", __func__);

	WARN_ON(!crtc->state->event);

	/* Turn vertical blanking interrupt reporting on. */
	drm_crtc_vblank_on(crtc);
	WARN_ON(drm_crtc_vblank_get(crtc) != 0);
	v_crtc->enabled = true;

	spin_lock_irq(&crtc->dev->event_lock);
	if (crtc->state->event) {
		v_crtc->event = crtc->state->event;
		crtc->state->event = NULL;
	}
	spin_unlock_irq(&crtc->dev->event_lock);

	upd_planes = v_crtc_update_plane_mask(crtc, old_state, crtc->state);
	if (upd_planes)
		v_crtc_flush_to_remote(crtc, upd_planes, "enable");
	else
		schedule_work(&v_crtc->fake_vsync);
}

static void v_crtc_atomic_disable(struct drm_crtc *crtc,
				      struct drm_crtc_state *old_state)
{
	struct v_crtc *v_crtc = to_v_crtc(crtc);
	u32 upd_planes;

	dev_dbg(crtc->dev->dev, "%s\n", __func__);

	reinit_completion(&v_crtc->framedone_completion);

	WARN_ON(!crtc->state->event);

	WARN_ON(drm_crtc_vblank_get(crtc) != 0);

	mutex_lock(&v_crtc->vsync_lock);
	v_crtc->disable_in_progress = true;
	mutex_unlock(&v_crtc->vsync_lock);

	spin_lock_irq(&crtc->dev->event_lock);
	if (crtc->state->event) {
		v_crtc->event = crtc->state->event;
		crtc->state->event = NULL;
	}
	spin_unlock_irq(&crtc->dev->event_lock);

	upd_planes = v_crtc_update_plane_mask(crtc, old_state, crtc->state);
	if (upd_planes)
		v_crtc_flush_to_remote(crtc, upd_planes, "disable");
	else
		schedule_work(&v_crtc->fake_vsync);

	if (!wait_for_completion_timeout(&v_crtc->framedone_completion,
					 msecs_to_jiffies(500)))
		dev_err(crtc->dev->dev, "Timeout waiting for disable complete on v_crtc %d",
			v_crtc->remote_id);

	v_crtc->enabled = false;
	drm_crtc_vblank_off(crtc);
}


static void v_crtc_atomic_flush(struct drm_crtc *crtc,
				    struct drm_crtc_state *old_crtc_state)
{
	struct v_crtc *v_crtc = to_v_crtc(crtc);
	u32 upd_planes;

	dev_dbg(crtc->dev->dev, "%s\n", __func__);

	/* Only flush the CRTC if it is currently enabled. */
	if (!v_crtc->enabled)
		return;

	/* TODO : check if the old frame is still there */

	// I think we always need the event to signal flip done
	WARN_ON(!crtc->state->event);

	WARN_ON(drm_crtc_vblank_get(crtc) != 0);

	spin_lock_irq(&crtc->dev->event_lock);

	if (crtc->state->event) {
		v_crtc->event = crtc->state->event;
		crtc->state->event = NULL;
	}

	spin_unlock_irq(&crtc->dev->event_lock);

	upd_planes = v_crtc_update_plane_mask(crtc, old_crtc_state, crtc->state);
	if (upd_planes)
		v_crtc_flush_to_remote(crtc, upd_planes, "flush");
	else
		schedule_work(&v_crtc->fake_vsync);
}


static const struct drm_crtc_helper_funcs v_crtc_helper_funcs = {
	.atomic_check = v_crtc_atomic_check,
	.atomic_begin = v_crtc_atomic_begin,
	.atomic_flush = v_crtc_atomic_flush,
	.atomic_enable = v_crtc_atomic_enable,
	.atomic_disable = v_crtc_atomic_disable,

	.mode_valid = v_crtc_mode_valid,
};

static void v_crtc_destroy(struct drm_crtc *crtc)
{
	struct v_crtc *v_crtc = to_v_crtc(crtc);

	dev_dbg(crtc->dev->dev, "%s\n", __func__);

	drm_crtc_cleanup(crtc);
	kfree(v_crtc);
}

static int v_crtc_enable_vblank(struct drm_crtc *crtc)
{
	dev_dbg(crtc->dev->dev, "%s\n", __func__);

	return 0;
}

static void v_crtc_disable_vblank(struct drm_crtc *crtc)
{
	dev_dbg(crtc->dev->dev, "%s\n", __func__);
}

static const struct drm_crtc_funcs v_crtc_funcs = {
	.reset = drm_atomic_helper_crtc_reset,
	.destroy = v_crtc_destroy,
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
	.enable_vblank = v_crtc_enable_vblank,
	.disable_vblank = v_crtc_disable_vblank,
};

struct drm_crtc *v_crtc_init(struct tidss_device *tidss, int id,
		struct drm_plane *plane, struct rpmsg_remotedev_display_disp *vp)
{
	struct drm_device *dev = tidss->ddev;
	struct drm_crtc *crtc = NULL;
	struct v_crtc *v_crtc;
	int ret;

	v_crtc = kzalloc(sizeof(*v_crtc), GFP_KERNEL);
	if (!v_crtc)
		return NULL;

	v_crtc->tidss = tidss;
	v_crtc->id = id;
	v_crtc->remote_id = vp->disp_id;
	init_completion(&v_crtc->framedone_completion);
	INIT_WORK(&v_crtc->fake_vsync, fake_vsync_fn);

	mutex_init(&v_crtc->vsync_lock);

	crtc = &v_crtc->base;

	ret = drm_crtc_init_with_planes(dev, crtc, plane, NULL,
					&v_crtc_funcs, "crtc-0x%x", id);
	if (ret < 0)
		goto err;

	drm_crtc_helper_add(crtc, &v_crtc_helper_funcs);

	return crtc;

err:
	kfree(v_crtc);
	return NULL;
}

void v_crtc_fini(struct tidss_device *tidss, struct drm_crtc *crtc)
{
	v_crtc_destroy(crtc);
}

