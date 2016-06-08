/*
 * drivers/gpu/drm/omapdrm/omap_crtc.c
 *
 * Copyright (C) 2011 Texas Instruments
 * Author: Rob Clark <rob@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_mode.h>
#include <drm/drm_plane_helper.h>

#include "omap_drv.h"

#define to_omap_crtc(x) container_of(x, struct omap_crtc, base)

struct omap_crtc {
	struct drm_crtc base;

	const char *name;
	enum omap_channel channel;

	struct omap_video_timings timings;

	struct omap_drm_irq vblank_irq;
	struct omap_drm_irq error_irq;

	bool ignore_digit_sync_lost;

	bool pending;
	wait_queue_head_t pending_wait;
};

struct omap_crtc_state {
	struct drm_crtc_state base;

	unsigned int trans_key_mode;
	unsigned int trans_key;
	unsigned int default_color;
	bool partial_alpha_enabled;
};

static inline struct omap_crtc_state *
to_omap_crtc_state(struct drm_crtc_state *state)
{
	return container_of(state, struct omap_crtc_state, base);
}

static void omap_crtc_atomic_destroy_state(struct drm_crtc *crtc,
					   struct drm_crtc_state *state);

/* -----------------------------------------------------------------------------
 * Helper Functions
 */

uint32_t pipe2vbl(struct drm_crtc *crtc)
{
	struct omap_crtc *omap_crtc = to_omap_crtc(crtc);
	struct omap_drm_private *priv = crtc->dev->dev_private;

	return priv->dispc_ops->mgr_get_vsync_irq(omap_crtc->channel);
}

struct omap_video_timings *omap_crtc_timings(struct drm_crtc *crtc)
{
	struct omap_crtc *omap_crtc = to_omap_crtc(crtc);
	return &omap_crtc->timings;
}

enum omap_channel omap_crtc_channel(struct drm_crtc *crtc)
{
	struct omap_crtc *omap_crtc = to_omap_crtc(crtc);
	return omap_crtc->channel;
}

int omap_crtc_wait_pending(struct drm_crtc *crtc)
{
	struct omap_crtc *omap_crtc = to_omap_crtc(crtc);

	/*
	 * Timeout is set to a "sufficiently" high value, which should cover
	 * a single frame refresh even on slower displays.
	 */
	return wait_event_timeout(omap_crtc->pending_wait,
				  !omap_crtc->pending,
				  msecs_to_jiffies(250));
}

/* -----------------------------------------------------------------------------
 * DSS Manager Functions
 */

/*
 * Manager-ops, callbacks from output when they need to configure
 * the upstream part of the video pipe.
 *
 * Most of these we can ignore until we add support for command-mode
 * panels.. for video-mode the crtc-helpers already do an adequate
 * job of sequencing the setup of the video pipe in the proper order
 */

/* ovl-mgr-id -> crtc */
static struct omap_crtc *omap_crtcs[8];
static struct omap_dss_device *omap_crtc_output[8];

/* we can probably ignore these until we support command-mode panels: */
static int omap_crtc_dss_connect(enum omap_channel channel,
		struct omap_dss_device *dst)
{
	const struct dispc_ops *dispc_ops = dispc_get_ops();

	if (omap_crtc_output[channel])
		return -EINVAL;

	if ((dispc_ops->mgr_get_supported_outputs(channel) & dst->id) == 0)
		return -EINVAL;

	omap_crtc_output[channel] = dst;
	dst->dispc_channel_connected = true;

	return 0;
}

static void omap_crtc_dss_disconnect(enum omap_channel channel,
		struct omap_dss_device *dst)
{
	omap_crtc_output[channel] = NULL;
	dst->dispc_channel_connected = false;
}

static void omap_crtc_dss_start_update(enum omap_channel channel)
{
}

/* Called only from the encoder enable/disable and suspend/resume handlers. */
static void omap_crtc_set_enabled(struct drm_crtc *crtc, bool enable)
{
	struct drm_device *dev = crtc->dev;
	struct omap_drm_private *priv = dev->dev_private;
	struct omap_crtc *omap_crtc = to_omap_crtc(crtc);
	enum omap_channel channel = omap_crtc->channel;
	struct omap_irq_wait *wait;
	u32 framedone_irq, vsync_irq;
	int ret;

	if (omap_crtc_output[channel]->output_type == OMAP_DISPLAY_TYPE_HDMI) {
		priv->dispc_ops->mgr_enable(channel, enable);
		return;
	}

	if (priv->dispc_ops->mgr_is_enabled(channel) == enable)
		return;

	if (omap_crtc->channel == OMAP_DSS_CHANNEL_DIGIT) {
		/*
		 * Digit output produces some sync lost interrupts during the
		 * first frame when enabling, so we need to ignore those.
		 */
		omap_crtc->ignore_digit_sync_lost = true;
	}

	framedone_irq = priv->dispc_ops->mgr_get_framedone_irq(channel);
	vsync_irq = priv->dispc_ops->mgr_get_vsync_irq(channel);

	if (enable) {
		wait = omap_irq_wait_init(dev, vsync_irq, 1);
	} else {
		/*
		 * When we disable the digit output, we need to wait for
		 * FRAMEDONE to know that DISPC has finished with the output.
		 *
		 * OMAP2/3 does not have FRAMEDONE irq for digit output, and in
		 * that case we need to use vsync interrupt, and wait for both
		 * even and odd frames.
		 */

		if (framedone_irq)
			wait = omap_irq_wait_init(dev, framedone_irq, 1);
		else
			wait = omap_irq_wait_init(dev, vsync_irq, 2);
	}

	priv->dispc_ops->mgr_enable(channel, enable);

	ret = omap_irq_wait(dev, wait, msecs_to_jiffies(100));
	if (ret) {
		dev_err(dev->dev, "%s: timeout waiting for %s\n",
				omap_crtc->name, enable ? "enable" : "disable");
	}

	if (omap_crtc->channel == OMAP_DSS_CHANNEL_DIGIT) {
		omap_crtc->ignore_digit_sync_lost = false;
		/* make sure the irq handler sees the value above */
		mb();
	}
}


static int omap_crtc_dss_enable(enum omap_channel channel)
{
	struct omap_crtc *omap_crtc = omap_crtcs[channel];
	struct omap_drm_private *priv = omap_crtc->base.dev->dev_private;

	priv->dispc_ops->mgr_set_timings(omap_crtc->channel,
			&omap_crtc->timings);
	omap_crtc_set_enabled(&omap_crtc->base, true);

	return 0;
}

static void omap_crtc_dss_disable(enum omap_channel channel)
{
	struct omap_crtc *omap_crtc = omap_crtcs[channel];

	omap_crtc_set_enabled(&omap_crtc->base, false);
}

static void omap_crtc_dss_set_timings(enum omap_channel channel,
		const struct omap_video_timings *timings)
{
	struct omap_crtc *omap_crtc = omap_crtcs[channel];
	DBG("%s", omap_crtc->name);
	omap_crtc->timings = *timings;
}

static void omap_crtc_dss_set_lcd_config(enum omap_channel channel,
		const struct dss_lcd_mgr_config *config)
{
	struct omap_crtc *omap_crtc = omap_crtcs[channel];
	struct omap_drm_private *priv = omap_crtc->base.dev->dev_private;

	DBG("%s", omap_crtc->name);
	priv->dispc_ops->mgr_set_lcd_config(omap_crtc->channel, config);
}

static int omap_crtc_dss_register_framedone(
		enum omap_channel channel,
		void (*handler)(void *), void *data)
{
	return 0;
}

static void omap_crtc_dss_unregister_framedone(
		enum omap_channel channel,
		void (*handler)(void *), void *data)
{
}

static const struct dss_mgr_ops mgr_ops = {
	.connect = omap_crtc_dss_connect,
	.disconnect = omap_crtc_dss_disconnect,
	.start_update = omap_crtc_dss_start_update,
	.enable = omap_crtc_dss_enable,
	.disable = omap_crtc_dss_disable,
	.set_timings = omap_crtc_dss_set_timings,
	.set_lcd_config = omap_crtc_dss_set_lcd_config,
	.register_framedone_handler = omap_crtc_dss_register_framedone,
	.unregister_framedone_handler = omap_crtc_dss_unregister_framedone,
};

/* -----------------------------------------------------------------------------
 * Setup, Flush and Page Flip
 */

static void omap_crtc_complete_page_flip(struct drm_crtc *crtc)
{
	struct drm_pending_vblank_event *event;
	struct drm_device *dev = crtc->dev;
	unsigned long flags;

	event = crtc->state->event;

	if (!event)
		return;

	spin_lock_irqsave(&dev->event_lock, flags);

	list_del(&event->base.link);

	/*
	 * Queue the event for delivery if it's still linked to a file
	 * handle, otherwise just destroy it.
	 */
	if (event->base.file_priv)
		drm_crtc_send_vblank_event(crtc, event);
	else
		event->base.destroy(&event->base);

	spin_unlock_irqrestore(&dev->event_lock, flags);
}

static void omap_crtc_error_irq(struct omap_drm_irq *irq, uint32_t irqstatus)
{
	struct omap_crtc *omap_crtc =
			container_of(irq, struct omap_crtc, error_irq);

	if (omap_crtc->ignore_digit_sync_lost) {
		irqstatus &= ~DISPC_IRQ_SYNC_LOST_DIGIT;
		if (!irqstatus)
			return;
	}

	DRM_ERROR_RATELIMITED("%s: errors: %08x\n", omap_crtc->name, irqstatus);
}

static void omap_crtc_vblank_irq(struct omap_drm_irq *irq, uint32_t irqstatus)
{
	struct omap_crtc *omap_crtc =
			container_of(irq, struct omap_crtc, vblank_irq);
	struct drm_device *dev = omap_crtc->base.dev;
	struct omap_drm_private *priv = dev->dev_private;

	if (priv->dispc_ops->mgr_go_busy(omap_crtc->channel))
		return;

	DBG("%s: apply done", omap_crtc->name);

	__omap_irq_unregister(dev, &omap_crtc->vblank_irq);

	rmb();
	WARN_ON(!omap_crtc->pending);
	omap_crtc->pending = false;
	wmb();

	/* wake up userspace */
	omap_crtc_complete_page_flip(&omap_crtc->base);

	/* wake up omap_atomic_complete */
	wake_up(&omap_crtc->pending_wait);
}

static void omap_crtc_write_crtc_properties(struct drm_crtc *crtc)
{
	struct omap_drm_private *priv = crtc->dev->dev_private;
	struct omap_crtc *omap_crtc = to_omap_crtc(crtc);
	struct omap_overlay_manager_info info;
	const struct omap_crtc_state *omap_state =
		to_omap_crtc_state(crtc->state);

	memset(&info, 0, sizeof(info));
	info.default_color = omap_state->default_color;

	info.trans_key = omap_state->trans_key;

	switch (omap_state->trans_key_mode) {
	case 0:
	default:
		info.trans_enabled = false;
		break;
	case 1:
		info.trans_enabled = true;
		info.trans_key_type = OMAP_DSS_COLOR_KEY_GFX_DST;
		break;
	case 2:
		info.trans_enabled = true;
		info.trans_key_type = OMAP_DSS_COLOR_KEY_VID_SRC;
		break;
	}

	info.partial_alpha_enabled = omap_state->partial_alpha_enabled;

	priv->dispc_ops->mgr_setup(omap_crtc->channel, &info);
}

/* -----------------------------------------------------------------------------
 * CRTC Functions
 */

static void omap_crtc_reset(struct drm_crtc *crtc)
{
	struct omap_crtc_state *omap_state;

	if (crtc->state) {
		omap_crtc_atomic_destroy_state(crtc, crtc->state);
		crtc->state = NULL;
	}

	omap_state = kzalloc(sizeof(*omap_state), GFP_KERNEL);
	if (omap_state == NULL)
		return;

	omap_state->trans_key_mode = 0;
	omap_state->trans_key = 0;
	omap_state->default_color = 0;
	omap_state->partial_alpha_enabled = false;

	crtc->state = &omap_state->base;
	crtc->state->crtc = crtc;
}

static void omap_crtc_destroy(struct drm_crtc *crtc)
{
	struct omap_crtc *omap_crtc = to_omap_crtc(crtc);

	DBG("%s", omap_crtc->name);

	WARN_ON(omap_crtc->vblank_irq.registered);
	omap_irq_unregister(crtc->dev, &omap_crtc->error_irq);

	drm_crtc_cleanup(crtc);

	kfree(omap_crtc);
}

static bool omap_crtc_mode_fixup(struct drm_crtc *crtc,
		const struct drm_display_mode *mode,
		struct drm_display_mode *adjusted_mode)
{
	return true;
}

static void omap_crtc_enable(struct drm_crtc *crtc)
{
	struct omap_crtc *omap_crtc = to_omap_crtc(crtc);

	DBG("%s", omap_crtc->name);

	rmb();
	WARN_ON(omap_crtc->pending);
	omap_crtc->pending = true;
	wmb();

	omap_irq_register(crtc->dev, &omap_crtc->vblank_irq);

	drm_crtc_vblank_on(crtc);
}

static void omap_crtc_disable(struct drm_crtc *crtc)
{
	struct omap_crtc *omap_crtc = to_omap_crtc(crtc);

	DBG("%s", omap_crtc->name);

	drm_crtc_vblank_off(crtc);
}

static void omap_crtc_mode_set_nofb(struct drm_crtc *crtc)
{
	struct omap_crtc *omap_crtc = to_omap_crtc(crtc);
	struct drm_display_mode *mode = &crtc->state->adjusted_mode;

	DBG("%s: set mode: %d:\"%s\" %d %d %d %d %d %d %d %d %d %d 0x%x 0x%x",
	    omap_crtc->name, mode->base.id, mode->name,
	    mode->vrefresh, mode->clock,
	    mode->hdisplay, mode->hsync_start, mode->hsync_end, mode->htotal,
	    mode->vdisplay, mode->vsync_start, mode->vsync_end, mode->vtotal,
	    mode->type, mode->flags);

	copy_timings_drm_to_omap(&omap_crtc->timings, mode);
}

static int omap_crtc_atomic_check(struct drm_crtc *crtc,
				struct drm_crtc_state *state)
{
	if (state->color_mgmt_changed && state->gamma_lut) {
		uint length = state->gamma_lut->length /
			sizeof(struct drm_color_lut);

		if (length < 2)
			return -EINVAL;
	}

	return 0;
}

static void omap_crtc_atomic_begin(struct drm_crtc *crtc,
                                  struct drm_crtc_state *old_crtc_state)
{
}

static void omap_crtc_atomic_flush(struct drm_crtc *crtc,
                                  struct drm_crtc_state *old_crtc_state)
{
	struct omap_drm_private *priv = crtc->dev->dev_private;
	struct omap_crtc *omap_crtc = to_omap_crtc(crtc);

	WARN_ON(omap_crtc->vblank_irq.registered);

	omap_crtc_write_crtc_properties(crtc);

	if (crtc->state->color_mgmt_changed) {
		struct drm_color_lut *lut = NULL;
		uint length = 0;

		if (crtc->state->gamma_lut) {
			lut = (struct drm_color_lut *)
				crtc->state->gamma_lut->data;
			length = crtc->state->gamma_lut->length /
				sizeof(*lut);
		}
		priv->dispc_ops->mgr_set_gamma(omap_crtc->channel, lut, length);
	}

	if (priv->dispc_ops->mgr_is_enabled(omap_crtc->channel)) {

		DBG("%s: GO", omap_crtc->name);

		rmb();
		WARN_ON(omap_crtc->pending);
		omap_crtc->pending = true;
		wmb();

		priv->dispc_ops->mgr_go(omap_crtc->channel);
		omap_irq_register(crtc->dev, &omap_crtc->vblank_irq);
	}
}

static struct drm_crtc_state *
omap_crtc_atomic_duplicate_state(struct drm_crtc *crtc)
{
	struct omap_crtc_state *state;
	struct omap_crtc_state *copy;

	if (WARN_ON(!crtc->state))
		return NULL;

	state = to_omap_crtc_state(crtc->state);
	copy = kmemdup(state, sizeof(*state), GFP_KERNEL);
	if (copy == NULL)
		return NULL;

	__drm_atomic_helper_crtc_duplicate_state(crtc, &copy->base);

	return &copy->base;
}

static void omap_crtc_atomic_destroy_state(struct drm_crtc *crtc,
					   struct drm_crtc_state *state)
{
	__drm_atomic_helper_crtc_destroy_state(crtc, state);
	kfree(to_omap_crtc_state(state));
}

static bool omap_crtc_is_plane_prop(struct drm_device *dev,
	struct drm_property *property)
{
	struct omap_drm_private *priv = dev->dev_private;

	return property == priv->zorder_prop ||
		property == dev->mode_config.rotation_property;
}

static int omap_crtc_atomic_set_property(struct drm_crtc *crtc,
					 struct drm_crtc_state *state,
					 struct drm_property *property,
					 uint64_t val)
{
	struct drm_device *dev = crtc->dev;
	struct omap_drm_private *priv = dev->dev_private;
	struct omap_crtc_state *omap_state = to_omap_crtc_state(state);

	if (omap_crtc_is_plane_prop(dev, property)) {
		struct drm_plane_state *plane_state;
		struct drm_plane *plane = crtc->primary;

		/*
		 * Delegate property set to the primary plane. Get the plane
		 * state and set the property directly.
		 */

		plane_state = drm_atomic_get_plane_state(state->state, plane);
		if (IS_ERR(plane_state))
			return PTR_ERR(plane_state);

		return drm_atomic_plane_set_property(plane, plane_state,
				property, val);
	}

	if (property == priv->trans_key_mode_prop)
		omap_state->trans_key_mode = val;
	else if (property == priv->trans_key_prop)
		omap_state->trans_key = val;
	else if (property == priv->background_color_prop)
		omap_state->default_color = val;
	else if (property == priv->alpha_blender_prop)
		omap_state->partial_alpha_enabled = !!val;
	else
		return -EINVAL;

	return 0;
}

static int omap_crtc_atomic_get_property(struct drm_crtc *crtc,
					 const struct drm_crtc_state *state,
					 struct drm_property *property,
					 uint64_t *val)
{
	struct drm_device *dev = crtc->dev;
	struct omap_drm_private *priv = dev->dev_private;
	const struct omap_crtc_state *omap_state =
		container_of(state, const struct omap_crtc_state, base);

	if (omap_crtc_is_plane_prop(dev, property)) {
		/*
		 * Delegate property get to the primary plane. The
		 * drm_atomic_plane_get_property() function isn't exported, but
		 * can be called through drm_object_property_get_value() as that
		 * will call drm_atomic_get_property() for atomic drivers.
		 */
		return drm_object_property_get_value(&crtc->primary->base,
				property, val);
	}

	if (property == priv->trans_key_mode_prop)
		*val = omap_state->trans_key_mode;
	else if (property == priv->trans_key_prop)
		*val = omap_state->trans_key;
	else if (property == priv->background_color_prop)
		*val = omap_state->default_color;
	else if (property == priv->alpha_blender_prop)
		*val = omap_state->partial_alpha_enabled;
	else
		return -EINVAL;

	return 0;
}

static const struct drm_crtc_funcs omap_crtc_funcs = {
	.reset = omap_crtc_reset,
	.set_config = drm_atomic_helper_set_config,
	.destroy = omap_crtc_destroy,
	.page_flip = drm_atomic_helper_page_flip,
	.gamma_set = drm_atomic_helper_legacy_gamma_set,
	.set_property = drm_atomic_helper_crtc_set_property,
	.atomic_duplicate_state = omap_crtc_atomic_duplicate_state,
	.atomic_destroy_state = omap_crtc_atomic_destroy_state,
	.atomic_set_property = omap_crtc_atomic_set_property,
	.atomic_get_property = omap_crtc_atomic_get_property,
};

static const struct drm_crtc_helper_funcs omap_crtc_helper_funcs = {
	.mode_fixup = omap_crtc_mode_fixup,
	.mode_set_nofb = omap_crtc_mode_set_nofb,
	.disable = omap_crtc_disable,
	.enable = omap_crtc_enable,
	.atomic_check = omap_crtc_atomic_check,
	.atomic_begin = omap_crtc_atomic_begin,
	.atomic_flush = omap_crtc_atomic_flush,
};

/* -----------------------------------------------------------------------------
 * Init and Cleanup
 */

static const char *channel_names[] = {
	[OMAP_DSS_CHANNEL_LCD] = "lcd",
	[OMAP_DSS_CHANNEL_DIGIT] = "tv",
	[OMAP_DSS_CHANNEL_LCD2] = "lcd2",
	[OMAP_DSS_CHANNEL_LCD3] = "lcd3",
};

void omap_crtc_pre_init(void)
{
	dss_install_mgr_ops(&mgr_ops);
}

void omap_crtc_pre_uninit(void)
{
	dss_uninstall_mgr_ops();
}

static void omap_crtc_install_properties(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct drm_mode_object *obj = &crtc->base;
	struct omap_drm_private *priv = dev->dev_private;

	drm_object_attach_property(obj, priv->trans_key_mode_prop, 0);
	drm_object_attach_property(obj, priv->trans_key_prop, 0);
	drm_object_attach_property(obj, priv->background_color_prop, 0);
	drm_object_attach_property(obj, priv->alpha_blender_prop, 0);
}

/* initialize crtc */
struct drm_crtc *omap_crtc_init(struct drm_device *dev,
		struct drm_plane *plane, enum omap_channel channel, int id)
{
	struct omap_drm_private *priv = dev->dev_private;
	struct drm_crtc *crtc = NULL;
	struct omap_crtc *omap_crtc;
	int ret;

	DBG("%s", channel_names[channel]);

	omap_crtc = kzalloc(sizeof(*omap_crtc), GFP_KERNEL);
	if (!omap_crtc)
		return NULL;

	crtc = &omap_crtc->base;

	init_waitqueue_head(&omap_crtc->pending_wait);

	omap_crtc->channel = channel;
	omap_crtc->name = channel_names[channel];

	omap_crtc->vblank_irq.irqmask =
		priv->dispc_ops->mgr_get_vsync_irq(channel);
	omap_crtc->vblank_irq.irq = omap_crtc_vblank_irq;

	omap_crtc->error_irq.irqmask =
			priv->dispc_ops->mgr_get_sync_lost_irq(channel);
	omap_crtc->error_irq.irq = omap_crtc_error_irq;
	omap_irq_register(dev, &omap_crtc->error_irq);

	ret = drm_crtc_init_with_planes(dev, crtc, plane, NULL,
					&omap_crtc_funcs);
	if (ret < 0) {
		kfree(omap_crtc);
		return NULL;
	}

	drm_crtc_helper_add(crtc, &omap_crtc_helper_funcs);

	omap_crtc_install_properties(crtc);

	/* The dispc API adapts to what ever size, but the HW supports
	 * 256 element gamma table for LCDs and 1024 element table for
	 * OMAP_DSS_CHANNEL_DIGIT. X server assumes 256 element gamma
	 * tables so lets use that. Size of HW gamma table can be
	 * extracted with dispc_mgr_gamma_size(). If it returns 0
	 * gamma table is not supprted.
	 */
	if (priv->dispc_ops->mgr_gamma_size(channel)) {
		uint gamma_lut_size = 256;

		drm_crtc_enable_color_mgmt(crtc, 0, false, gamma_lut_size);
		drm_mode_crtc_set_gamma_size(crtc, gamma_lut_size);
	}

	omap_plane_install_properties(crtc->primary, &crtc->base);

	omap_crtcs[channel] = omap_crtc;

	return crtc;
}
