/*
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
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

#include <linux/of.h>
#include <linux/sort.h>
#include <linux/sys_soc.h>

#include "omap_dmm_tiler.h"
#include "omap_drv.h"

#define DRIVER_NAME		MODULE_NAME
#define DRIVER_DESC		"OMAP DRM"
#define DRIVER_DATE		"20110917"
#define DRIVER_MAJOR		1
#define DRIVER_MINOR		0
#define DRIVER_PATCHLEVEL	0

#define MAX_NR_DISPLAYS		8
static int display_order[MAX_NR_DISPLAYS];
static int display_order_nelm;
module_param_array_named(displays, display_order, int, &display_order_nelm,
			 0444);
MODULE_PARM_DESC(displays,
		 "ID array to specify the order of the active displays");

/*
 * mode config funcs
 */

/* Notes about mapping DSS and DRM entities:
 *    CRTC:        overlay
 *    encoder:     manager.. with some extension to allow one primary CRTC
 *                 and zero or more video CRTC's to be mapped to one encoder?
 *    connector:   dssdev.. manager can be attached/detached from different
 *                 devices
 */

static void omap_fb_output_poll_changed(struct drm_device *dev)
{
	struct omap_drm_private *priv = dev->dev_private;
	DBG("dev=%p", dev);
	if (priv->fbdev)
		drm_fb_helper_hotplug_event(priv->fbdev);
}

static void omap_atomic_wait_for_completion(struct drm_device *dev,
					    struct drm_atomic_state *old_state)
{
	struct drm_crtc_state *new_crtc_state;
	struct drm_crtc *crtc;
	unsigned int i;
	int ret;

	for_each_new_crtc_in_state(old_state, crtc, new_crtc_state, i) {
		if (!new_crtc_state->active)
			continue;

		ret = omap_crtc_wait_pending(crtc);

		if (!ret)
			dev_warn(dev->dev,
				 "atomic complete timeout (pipe %u)!\n", i);
	}
}

static void omap_atomic_commit_tail(struct drm_atomic_state *old_state)
{
	struct drm_device *dev = old_state->dev;
	struct omap_drm_private *priv = dev->dev_private;

	priv->dispc_ops->runtime_get();

	/* Apply the atomic update. */
	drm_atomic_helper_commit_modeset_disables(dev, old_state);

	if (priv->omaprev != 0x3430) {
		/* With the current dss dispc implementation we have to enable
		 * the new modeset before we can commit planes. The dispc ovl
		 * configuration relies on the video mode configuration been
		 * written into the HW when the ovl configuration is
		 * calculated.
		 *
		 * This approach is not ideal because after a mode change the
		 * plane update is executed only after the first vblank
		 * interrupt. The dispc implementation should be fixed so that
		 * it is able use uncommitted drm state information.
		 */
		drm_atomic_helper_commit_modeset_enables(dev, old_state);
		omap_atomic_wait_for_completion(dev, old_state);

		drm_atomic_helper_commit_planes(dev, old_state, 0);

		drm_atomic_helper_commit_hw_done(old_state);
	} else {
		/*
		 * OMAP3 DSS seems to have issues with the work-around above,
		 * resulting in endless sync losts if a crtc is enabled without
		 * a plane. For now, skip the WA for OMAP3.
		 */
		drm_atomic_helper_commit_planes(dev, old_state, 0);

		drm_atomic_helper_commit_modeset_enables(dev, old_state);

		drm_atomic_helper_commit_hw_done(old_state);
	}

	/*
	 * Wait for completion of the page flips to ensure that old buffers
	 * can't be touched by the hardware anymore before cleaning up planes.
	 */
	omap_atomic_wait_for_completion(dev, old_state);

	drm_atomic_helper_cleanup_planes(dev, old_state);

	priv->dispc_ops->runtime_put();
}

static int drm_atomic_state_normalized_zpos_cmp(const void *a, const void *b)
{
	const struct drm_plane_state *sa = *(struct drm_plane_state **)a;
	const struct drm_plane_state *sb = *(struct drm_plane_state **)b;

	if (sa->normalized_zpos != sb->normalized_zpos)
		return sa->normalized_zpos - sb->normalized_zpos;
	else
		return sa->plane->base.id - sb->plane->base.id;
}

static int omap_atomic_update_normalize_zpos(struct drm_device *dev,
					     struct drm_atomic_state *state)
{
	struct drm_crtc *crtc;
	struct drm_crtc_state *old_state, *new_state;
	struct drm_plane *plane;
	int c, i, n, inc;
	int total_planes = dev->mode_config.num_total_plane;
	struct drm_plane_state **states;
	int ret = 0;

	states = kmalloc_array(total_planes, sizeof(*states), GFP_KERNEL);
	if (!states)
		return -ENOMEM;

	for_each_oldnew_crtc_in_state(state, crtc, old_state, new_state, c) {
		if (old_state->plane_mask == new_state->plane_mask &&
		    !new_state->zpos_changed)
			continue;

		if (omap_crtc_atomic_get_trans_key_mode(crtc, new_state))
			continue;

		/* Reset plane increment and index value for every crtc */
		n = 0;

		/*
		 * Normalization process might create new states for planes
		 * which normalized_zpos has to be recalculated.
		 */
		drm_for_each_plane_mask(plane, dev, new_state->plane_mask) {
			struct drm_plane_state *plane_state =
				drm_atomic_get_plane_state(new_state->state,
							   plane);
			if (IS_ERR(plane_state)) {
				ret = PTR_ERR(plane_state);
				goto done;
			}
			states[n++] = plane_state;
		}

		sort(states, n, sizeof(*states),
		     drm_atomic_state_normalized_zpos_cmp, NULL);

		for (i = 0, inc = 0; i < n; i++) {
			plane = states[i]->plane;

			states[i]->normalized_zpos = i + inc;
			DRM_DEBUG_ATOMIC("[PLANE:%d:%s] updated normalized zpos value %d\n",
					 plane->base.id, plane->name,
					 states[i]->normalized_zpos);

			if (is_omap_plane_dual_overlay(states[i]))
				inc++;
		}
		new_state->zpos_changed = true;
	}

done:
	kfree(states);
	return ret;
}

static int omap_atomic_check(struct drm_device *dev,
			     struct drm_atomic_state *state)
{
	int ret;

	ret = drm_atomic_helper_check(dev, state);
	if (ret)
		return ret;

	if (dev->mode_config.normalize_zpos) {
		ret = omap_atomic_update_normalize_zpos(dev, state);
		if (ret)
			return ret;
	}

	return 0;
}

static const struct drm_mode_config_helper_funcs omap_mode_config_helper_funcs = {
	.atomic_commit_tail = omap_atomic_commit_tail,
};

static const struct drm_mode_config_funcs omap_mode_config_funcs = {
	.fb_create = omap_framebuffer_create,
	.output_poll_changed = omap_fb_output_poll_changed,
	.atomic_check = omap_atomic_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static int get_connector_type(struct omap_dss_device *dssdev)
{
	switch (dssdev->type) {
	case OMAP_DISPLAY_TYPE_HDMI:
		return DRM_MODE_CONNECTOR_HDMIA;
	case OMAP_DISPLAY_TYPE_DVI:
		return DRM_MODE_CONNECTOR_DVID;
	case OMAP_DISPLAY_TYPE_DSI:
		return DRM_MODE_CONNECTOR_DSI;
	case OMAP_DISPLAY_TYPE_DPI:
	case OMAP_DISPLAY_TYPE_DBI:
		return DRM_MODE_CONNECTOR_DPI;
	case OMAP_DISPLAY_TYPE_VENC:
		/* TODO: This could also be composite */
		return DRM_MODE_CONNECTOR_SVIDEO;
	case OMAP_DISPLAY_TYPE_SDI:
		return DRM_MODE_CONNECTOR_LVDS;
	default:
		return DRM_MODE_CONNECTOR_Unknown;
	}
}

static void omap_disconnect_dssdevs(struct drm_device *ddev)
{
	struct omap_drm_private *priv = ddev->dev_private;
	unsigned int i;

	for (i = 0; i < priv->num_dssdevs; i++) {
		struct omap_dss_device *dssdev = priv->dssdevs[i];

		dssdev->driver->disconnect(dssdev);
		priv->dssdevs[i] = NULL;
		omap_dss_put_device(dssdev);
	}

	priv->num_dssdevs = 0;
}

static int omap_compare_dssdevs(const void *a, const void *b)
{
	const struct omap_dss_device *dssdev1 = *(struct omap_dss_device **)a;
	const struct omap_dss_device *dssdev2 = *(struct omap_dss_device **)b;

	if (dssdev1->alias_id > dssdev2->alias_id)
		return 1;
	else if (dssdev1->alias_id < dssdev2->alias_id)
		return -1;
	return 0;
}

static void omap_collect_dssdevs(struct drm_device *ddev)
{
	struct omap_drm_private *priv = ddev->dev_private;
	struct omap_dss_device *dssdevs[ARRAY_SIZE(priv->dssdevs)];
	struct omap_dss_device *dssdev = NULL;
	int num_dssdevs = 0;
	unsigned long dssdev_mask = 0;
	int i;

	/* No displays should be enabled */
	if (display_order_nelm == 1 && display_order[0] < 0)
		return;

	for_each_dss_dev(dssdev) {
		omap_dss_get_device(dssdev);
		set_bit(num_dssdevs, &dssdev_mask);
		dssdevs[num_dssdevs++] = dssdev;
		if (num_dssdevs == ARRAY_SIZE(dssdevs)) {
			/* To balance the 'for_each_dss_dev' loop */
			omap_dss_put_device(dssdev);
			break;
		}
	}

	/* Sort the list by DT aliases */
	sort(dssdevs, num_dssdevs, sizeof(dssdevs[0]), omap_compare_dssdevs,
	     NULL);

	/* Do ordering based on the display_order parameter array */
	for (i = 0; i < display_order_nelm; i++) {
		int old_index = display_order[i];

		if ((old_index >= 0 && old_index < num_dssdevs) &&
		    (dssdev_mask & BIT(old_index))) {
			priv->dssdevs[priv->num_dssdevs++] = dssdevs[old_index];
			clear_bit(old_index, &dssdev_mask);
		} else {
			dev_err(ddev->dev,
				"Ignoring invalid displays module parameter\n");
			priv->num_dssdevs = 0;
			break;
		}
	}

	/* if the target list is empty, copy the collected dssdevs, if any */
	if (priv->num_dssdevs == 0) {
		for (i = 0; i < num_dssdevs; i++)
			priv->dssdevs[i] = dssdevs[i];

		priv->num_dssdevs = num_dssdevs;
	} else {
		u32 idx;

		/* check if we have dssdev which is not carried over */
		for_each_set_bit(idx, &dssdev_mask, ARRAY_SIZE(dssdevs))
			omap_dss_put_device(dssdevs[idx]);
	}
}

static int omap_connect_dssdevs(struct drm_device *ddev)
{
	struct omap_drm_private *priv = ddev->dev_private;
	u32 working = 0;
	int r, i, j;

	if (!omapdss_stack_is_ready())
		return -EPROBE_DEFER;

	omap_collect_dssdevs(ddev);

	for (i = 0; i < priv->num_dssdevs; i++) {
		struct omap_dss_device *dssdev = priv->dssdevs[i];

		r = dssdev->driver->connect(dssdev);
		if (r == -EPROBE_DEFER)
			goto cleanup;
		else if (r)
			dev_warn(dssdev->dev, "could not connect display: %s\n",
				 dssdev->name);
		else
			working |= BIT(i);
	}

	/* Remove the dssdevs if their connect failed */
	j = 0;
	for (i = 0; i < priv->num_dssdevs; i++) {
		if (working & BIT(i)) {
			if (j != i)
				priv->dssdevs[j] = priv->dssdevs[i];
			j++;
		} else {
			omap_dss_put_device(priv->dssdevs[i]);
		}
	}

	priv->num_dssdevs = j;

	return 0;

cleanup:
	/*
	 * if we are deferring probe, we disconnect the devices we previously
	 * connected
	 */
	omap_disconnect_dssdevs(ddev);

	return r;
}

static int omap_modeset_init_properties(struct drm_device *dev)
{
	struct omap_drm_private *priv = dev->dev_private;
	unsigned int num_planes = priv->dispc_ops->get_num_ovls();

	static const struct drm_prop_enum_list trans_key_mode_list[] = {
		{ 0, "disable"},
		{ 1, "gfx-dst"},
		{ 2, "vid-src"},
	};

	priv->zorder_prop = drm_property_create_range(dev, 0, "zorder", 0,
						      num_planes - 1);
	if (!priv->zorder_prop)
		return -ENOMEM;

	priv->global_alpha_prop = drm_property_create_range(dev, 0,
		"global_alpha", 0, 255);
	if (!priv->global_alpha_prop)
		return -ENOMEM;

	priv->pre_mult_alpha_prop = drm_property_create_bool(dev, 0,
		"pre_mult_alpha");
	if (!priv->pre_mult_alpha_prop)
		return -ENOMEM;

	/* crtc properties */

	priv->background_color_prop = drm_property_create_range(dev, 0,
		"background", 0, 0xffffff);
	if (!priv->background_color_prop)
		return -ENOMEM;

	priv->trans_key_mode_prop = drm_property_create_enum(dev, 0,
		"trans-key-mode",
		trans_key_mode_list, ARRAY_SIZE(trans_key_mode_list));
	if (!priv->trans_key_mode_prop)
		return -ENOMEM;

	priv->trans_key_prop = drm_property_create_range(dev, 0, "trans-key",
		0, 0xffffff);
	if (!priv->trans_key_prop)
		return -ENOMEM;

	priv->alpha_blender_prop = drm_property_create_bool(dev, 0,
		"alpha_blender");
	if (!priv->alpha_blender_prop)
		return -ENOMEM;

	return 0;
}

static int omap_modeset_init(struct drm_device *dev)
{
	struct omap_drm_private *priv = dev->dev_private;
	struct omap_dss_device *dssdev = NULL;
	int num_ovls = priv->dispc_ops->get_num_ovls();
	int num_mgrs = priv->dispc_ops->get_num_mgrs();
	int num_crtcs, crtc_idx, plane_idx;
	int ret, i;
	u32 plane_crtc_mask;

	drm_mode_config_init(dev);

	ret = omap_modeset_init_properties(dev);
	if (ret < 0)
		return ret;

	/*
	 * This function creates exactly one connector, encoder, crtc,
	 * and primary plane per each connected dss-device. Each
	 * connector->encoder->crtc chain is expected to be separate
	 * and each crtc is connect to a single dss-channel. If the
	 * configuration does not match the expectations or exceeds
	 * the available resources, the configuration is rejected.
	 */
	num_crtcs = priv->num_dssdevs;
	if (num_crtcs > num_mgrs || num_crtcs > num_ovls ||
	    num_crtcs > ARRAY_SIZE(priv->crtcs) ||
	    num_crtcs > ARRAY_SIZE(priv->planes) ||
	    num_crtcs > ARRAY_SIZE(priv->encoders) ||
	    num_crtcs > ARRAY_SIZE(priv->connectors)) {
		dev_err(dev->dev, "%s(): Too many connected displays\n",
			__func__);
		return -EINVAL;
	}

	/* All planes can be put to any CRTC */
	plane_crtc_mask = (1 << num_crtcs) - 1;

	dssdev = NULL;

	crtc_idx = 0;
	plane_idx = 0;
	for (i = 0; i < priv->num_dssdevs; i++) {
		struct omap_dss_device *dssdev = priv->dssdevs[i];
		struct drm_connector *connector;
		struct drm_encoder *encoder;
		struct drm_plane *plane;
		struct drm_crtc *crtc;

		encoder = omap_encoder_init(dev, dssdev);
		if (!encoder)
			return -ENOMEM;

		connector = omap_connector_init(dev,
				get_connector_type(dssdev), dssdev, encoder);
		if (!connector)
			return -ENOMEM;

		plane = omap_plane_init(dev, plane_idx, DRM_PLANE_TYPE_PRIMARY,
					plane_crtc_mask);
		if (IS_ERR(plane))
			return PTR_ERR(plane);

		crtc = omap_crtc_init(dev, plane, dssdev);
		if (IS_ERR(crtc))
			return PTR_ERR(crtc);

		drm_mode_connector_attach_encoder(connector, encoder);
		encoder->possible_crtcs = (1 << crtc_idx);

		priv->crtcs[priv->num_crtcs++] = crtc;
		priv->planes[priv->num_planes++] = plane;
		priv->encoders[priv->num_encoders++] = encoder;
		priv->connectors[priv->num_connectors++] = connector;

		plane_idx++;
		crtc_idx++;
	}

	/*
	 * Create normal planes for the remaining overlays:
	 */
	for (; plane_idx < num_ovls; plane_idx++) {
		struct drm_plane *plane;

		if (WARN_ON(priv->num_planes >= ARRAY_SIZE(priv->planes)))
			return -EINVAL;

		plane = omap_plane_init(dev, plane_idx, DRM_PLANE_TYPE_OVERLAY,
			plane_crtc_mask);
		if (IS_ERR(plane))
			return PTR_ERR(plane);

		priv->planes[priv->num_planes++] = plane;
	}

	DBG("registered %d planes, %d crtcs, %d encoders and %d connectors\n",
		priv->num_planes, priv->num_crtcs, priv->num_encoders,
		priv->num_connectors);

	dev->mode_config.min_width = 8;
	dev->mode_config.min_height = 2;

	/*
	 * Note: these values are used for multiple independent things:
	 * connector mode filtering, buffer sizes, crtc sizes...
	 * Use big enough values here to cover all use cases, and do more
	 * specific checking in the respective code paths.
	 */
	dev->mode_config.max_width = 8192;
	dev->mode_config.max_height = 8192;

	/* We want the zpos to be normalized */
	dev->mode_config.normalize_zpos = true;

	dev->mode_config.funcs = &omap_mode_config_funcs;
	dev->mode_config.helper_private = &omap_mode_config_helper_funcs;

	drm_mode_config_reset(dev);

	omap_drm_irq_install(dev);

	return 0;
}

/*
 * Enable the HPD in external components if supported
 */
static void omap_modeset_enable_external_hpd(struct drm_device *ddev)
{
	struct omap_drm_private *priv = ddev->dev_private;
	int i;

	for (i = 0; i < priv->num_dssdevs; i++) {
		struct omap_dss_device *dssdev = priv->dssdevs[i];

		if (dssdev->driver->enable_hpd)
			dssdev->driver->enable_hpd(dssdev);
	}
}

/*
 * Disable the HPD in external components if supported
 */
static void omap_modeset_disable_external_hpd(struct drm_device *ddev)
{
	struct omap_drm_private *priv = ddev->dev_private;
	int i;

	for (i = 0; i < priv->num_dssdevs; i++) {
		struct omap_dss_device *dssdev = priv->dssdevs[i];

		if (dssdev->driver->disable_hpd)
			dssdev->driver->disable_hpd(dssdev);
	}
}

/*
 * drm ioctl funcs
 */


static int ioctl_get_param(struct drm_device *dev, void *data,
		struct drm_file *file_priv)
{
	struct omap_drm_private *priv = dev->dev_private;
	struct drm_omap_param *args = data;

	DBG("%p: param=%llu", dev, args->param);

	switch (args->param) {
	case OMAP_PARAM_CHIPSET_ID:
		args->value = priv->omaprev;
		break;
	default:
		DBG("unknown parameter %lld", args->param);
		return -EINVAL;
	}

	return 0;
}

static int ioctl_set_param(struct drm_device *dev, void *data,
		struct drm_file *file_priv)
{
	struct drm_omap_param *args = data;

	switch (args->param) {
	default:
		DBG("unknown parameter %lld", args->param);
		return -EINVAL;
	}

	return 0;
}

#define OMAP_BO_USER_MASK	0x00ffffff	/* flags settable by userspace */

static int ioctl_gem_new(struct drm_device *dev, void *data,
		struct drm_file *file_priv)
{
	struct drm_omap_gem_new *args = data;
	u32 flags = args->flags & OMAP_BO_USER_MASK;

	VERB("%p:%p: size=0x%08x, flags=%08x", dev, file_priv,
	     args->size.bytes, flags);

	return omap_gem_new_handle(dev, file_priv, args->size, flags,
				   &args->handle);
}

static int ioctl_gem_info(struct drm_device *dev, void *data,
		struct drm_file *file_priv)
{
	struct drm_omap_gem_info *args = data;
	struct drm_gem_object *obj;
	int ret = 0;

	VERB("%p:%p: handle=%d", dev, file_priv, args->handle);

	obj = drm_gem_object_lookup(file_priv, args->handle);
	if (!obj)
		return -ENOENT;

	args->size = omap_gem_mmap_size(obj);
	args->offset = omap_gem_mmap_offset(obj);

	drm_gem_object_unreference_unlocked(obj);

	return ret;
}

static const struct drm_ioctl_desc ioctls[DRM_COMMAND_END - DRM_COMMAND_BASE] = {
	DRM_IOCTL_DEF_DRV(OMAP_GET_PARAM, ioctl_get_param,
			  DRM_AUTH | DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(OMAP_SET_PARAM, ioctl_set_param,
			  DRM_AUTH | DRM_MASTER | DRM_ROOT_ONLY),
	DRM_IOCTL_DEF_DRV(OMAP_GEM_NEW, ioctl_gem_new,
			  DRM_AUTH | DRM_RENDER_ALLOW),
	/* Deprecated, to be removed. */
	DRM_IOCTL_DEF_DRV(OMAP_GEM_CPU_PREP, drm_noop,
			  DRM_AUTH | DRM_RENDER_ALLOW),
	/* Deprecated, to be removed. */
	DRM_IOCTL_DEF_DRV(OMAP_GEM_CPU_FINI, drm_noop,
			  DRM_AUTH | DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(OMAP_GEM_INFO, ioctl_gem_info,
			  DRM_AUTH | DRM_RENDER_ALLOW),
};

/*
 * drm driver funcs
 */

static int dev_open(struct drm_device *dev, struct drm_file *file)
{
	file->driver_priv = NULL;

	DBG("open: dev=%p, file=%p", dev, file);

	return 0;
}

/**
 * lastclose - clean up after all DRM clients have exited
 * @dev: DRM device
 *
 * Take care of cleaning up after all DRM clients have exited.  In the
 * mode setting case, we want to restore the kernel's initial mode (just
 * in case the last client left us in a bad state).
 */
static void dev_lastclose(struct drm_device *dev)
{
	struct omap_drm_private *priv = dev->dev_private;
	int ret;

	DBG("lastclose: dev=%p", dev);

	if (priv->fbdev) {
		ret = drm_fb_helper_restore_fbdev_mode_unlocked(priv->fbdev);
		if (ret)
			DBG("failed to restore crtc mode");
	}
}

static const struct vm_operations_struct omap_gem_vm_ops = {
	.fault = omap_gem_fault,
	.open = drm_gem_vm_open,
	.close = drm_gem_vm_close,
};

static const struct file_operations omapdriver_fops = {
	.owner = THIS_MODULE,
	.open = drm_open,
	.unlocked_ioctl = drm_ioctl,
	.compat_ioctl = drm_compat_ioctl,
	.release = drm_release,
	.mmap = omap_gem_mmap,
	.poll = drm_poll,
	.read = drm_read,
	.llseek = noop_llseek,
};

static struct drm_driver omap_drm_driver = {
	.driver_features = DRIVER_MODESET | DRIVER_GEM  | DRIVER_PRIME |
		DRIVER_ATOMIC | DRIVER_RENDER,
	.open = dev_open,
	.lastclose = dev_lastclose,
#ifdef CONFIG_DEBUG_FS
	.debugfs_init = omap_debugfs_init,
#endif
	.prime_handle_to_fd = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle = drm_gem_prime_fd_to_handle,
	.gem_prime_export = omap_gem_prime_export,
	.gem_prime_import = omap_gem_prime_import,
	.gem_free_object = omap_gem_free_object,
	.gem_vm_ops = &omap_gem_vm_ops,
	.dumb_create = omap_gem_dumb_create,
	.dumb_map_offset = omap_gem_dumb_map_offset,
	.ioctls = ioctls,
	.num_ioctls = DRM_OMAP_NUM_IOCTLS,
	.fops = &omapdriver_fops,
	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	.date = DRIVER_DATE,
	.major = DRIVER_MAJOR,
	.minor = DRIVER_MINOR,
	.patchlevel = DRIVER_PATCHLEVEL,
};

static const struct soc_device_attribute omapdrm_soc_devices[] = {
	{ .family = "OMAP3", .data = (void *)0x3430 },
	{ .family = "OMAP4", .data = (void *)0x4430 },
	{ .family = "OMAP5", .data = (void *)0x5430 },
	{ .family = "DRA7",  .data = (void *)0x0752 },
	{ /* sentinel */ }
};

static int pdev_probe(struct platform_device *pdev)
{
	const struct soc_device_attribute *soc;
	struct omap_drm_private *priv;
	struct drm_device *ddev;
	unsigned int i;
	int ret;

	DBG("%s", pdev->name);

	if (omapdss_is_initialized() == false)
		return -EPROBE_DEFER;

	ret = dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&pdev->dev, "Failed to set the DMA mask\n");
		return ret;
	}

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	/* Allocate and initialize the DRM device. */
	ddev = drm_dev_alloc(&omap_drm_driver, &pdev->dev);
	if (IS_ERR(ddev))
		return PTR_ERR(ddev);

	ddev->dev_private = priv;
	platform_set_drvdata(pdev, ddev);

	omap_crtc_pre_init();

	ret = omap_connect_dssdevs(ddev);
	if (ret)
		goto err_crtc_uninit;

	/* Initialize the driver private structure. */
	priv->dispc_ops = dispc_get_ops();

	soc = soc_device_match(omapdrm_soc_devices);
	priv->omaprev = soc ? (unsigned int)soc->data : 0;
	priv->wq = alloc_ordered_workqueue("omapdrm", 0);

	spin_lock_init(&priv->list_lock);
	INIT_LIST_HEAD(&priv->obj_list);

	/* Get memory bandwidth limits */
	if (priv->dispc_ops->get_memory_bandwidth_limit)
		priv->max_bandwidth =
				priv->dispc_ops->get_memory_bandwidth_limit();

	omap_gem_init(ddev);

	ret = omap_global_obj_init(priv);
	if (ret)
		goto err_gem_deinit;

	ret = omap_hwoverlays_init(ddev);
	if (ret)
		goto err_free_priv_obj;

	ret = omap_modeset_init(ddev);
	if (ret) {
		dev_err(&pdev->dev, "omap_modeset_init failed: ret=%d\n", ret);
		goto err_free_overlays;
	}

	/* Initialize vblank handling, start with all CRTCs disabled. */
	ret = drm_vblank_init(ddev, priv->num_crtcs);
	if (ret) {
		dev_err(&pdev->dev, "could not init vblank\n");
		goto err_cleanup_modeset;
	}

	for (i = 0; i < priv->num_crtcs; i++)
		drm_crtc_vblank_off(priv->crtcs[i]);

	priv->fbdev = omap_fbdev_init(ddev);

	drm_kms_helper_poll_init(ddev);
	omap_modeset_enable_external_hpd(ddev);

	if (priv->dispc_ops->has_writeback()) {
		ret = omap_wb_init(ddev);
		if (ret)
			dev_warn(&pdev->dev, "failed to initialize writeback\n");
		else
			priv->wb_initialized = true;
	}

	/*
	 * Register the DRM device with the core and the connectors with
	 * sysfs.
	 */
	ret = drm_dev_register(ddev, 0);
	if (ret)
		goto err_cleanup_helpers;

	return 0;

err_cleanup_helpers:
	if (priv->wb_initialized)
		omap_wb_cleanup(ddev);

	omap_modeset_disable_external_hpd(ddev);

	drm_kms_helper_poll_fini(ddev);
	if (priv->fbdev)
		omap_fbdev_free(ddev);
err_cleanup_modeset:
	drm_mode_config_cleanup(ddev);
	omap_drm_irq_uninstall(ddev);
err_free_overlays:
	omap_hwoverlays_destroy(ddev);
err_free_priv_obj:
	omap_global_obj_fini(priv);
err_gem_deinit:
	omap_gem_deinit(ddev);
	destroy_workqueue(priv->wq);
	omap_disconnect_dssdevs(ddev);
err_crtc_uninit:
	omap_crtc_pre_uninit();
	drm_dev_unref(ddev);
	return ret;
}

static int pdev_remove(struct platform_device *pdev)
{
	struct drm_device *ddev = platform_get_drvdata(pdev);
	struct omap_drm_private *priv = ddev->dev_private;

	DBG("");

	drm_dev_unregister(ddev);

	if (priv->wb_initialized)
		omap_wb_cleanup(ddev);

	omap_modeset_disable_external_hpd(ddev);
	drm_kms_helper_poll_fini(ddev);

	if (priv->fbdev)
		omap_fbdev_free(ddev);

	drm_atomic_helper_shutdown(ddev);

	drm_mode_config_cleanup(ddev);

	omap_drm_irq_uninstall(ddev);
	omap_gem_deinit(ddev);

	omap_hwoverlays_destroy(ddev);
	omap_global_obj_fini(priv);

	destroy_workqueue(priv->wq);

	omap_disconnect_dssdevs(ddev);
	omap_crtc_pre_uninit();

	drm_dev_unref(ddev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int omap_drm_suspend_all_displays(struct drm_device *ddev)
{
	struct omap_drm_private *priv = ddev->dev_private;
	int i;

	for (i = 0; i < priv->num_dssdevs; i++) {
		struct omap_dss_device *dssdev = priv->dssdevs[i];

		if (!dssdev->driver)
			continue;

		if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE) {
			dssdev->driver->disable(dssdev);
			dssdev->activate_after_resume = true;
		} else {
			dssdev->activate_after_resume = false;
		}
	}

	return 0;
}

static int omap_drm_resume_all_displays(struct drm_device *ddev)
{
	struct omap_drm_private *priv = ddev->dev_private;
	int i;

	for (i = 0; i < priv->num_dssdevs; i++) {
		struct omap_dss_device *dssdev = priv->dssdevs[i];

		if (!dssdev->driver)
			continue;

		if (dssdev->activate_after_resume) {
			dssdev->driver->enable(dssdev);
			dssdev->activate_after_resume = false;
		}
	}

	return 0;
}

static int omap_drm_suspend(struct device *dev)
{
	struct drm_device *drm_dev = dev_get_drvdata(dev);

	drm_kms_helper_poll_disable(drm_dev);

	drm_modeset_lock_all(drm_dev);
	omap_drm_suspend_all_displays(drm_dev);
	drm_modeset_unlock_all(drm_dev);

	return 0;
}

static int omap_drm_resume(struct device *dev)
{
	struct drm_device *drm_dev = dev_get_drvdata(dev);

	drm_modeset_lock_all(drm_dev);
	omap_drm_resume_all_displays(drm_dev);
	drm_modeset_unlock_all(drm_dev);

	drm_kms_helper_poll_enable(drm_dev);

	return omap_gem_resume(drm_dev);
}
#endif

static SIMPLE_DEV_PM_OPS(omapdrm_pm_ops, omap_drm_suspend, omap_drm_resume);

static struct platform_driver pdev = {
	.driver = {
		.name = "omapdrm",
		.pm = &omapdrm_pm_ops,
	},
	.probe = pdev_probe,
	.remove = pdev_remove,
};

static struct platform_driver * const drivers[] = {
	&omap_dmm_driver,
	&pdev,
};

static int __init omap_drm_init(void)
{
	DBG("init");

	return platform_register_drivers(drivers, ARRAY_SIZE(drivers));
}

static void __exit omap_drm_fini(void)
{
	DBG("fini");

	platform_unregister_drivers(drivers, ARRAY_SIZE(drivers));
}

/* need late_initcall() so we load after dss_driver's are loaded */
late_initcall(omap_drm_init);
module_exit(omap_drm_fini);

MODULE_AUTHOR("Rob Clark <rob@ti.com>");
MODULE_DESCRIPTION("OMAP DRM Display Driver");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_LICENSE("GPL v2");
