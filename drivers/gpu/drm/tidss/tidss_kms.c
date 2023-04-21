// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 Texas Instruments Incorporated - https://www.ti.com/
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
 */

#include <linux/dma-fence.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_vblank.h>
#include <linux/of.h>

#include "tidss_crtc.h"
#include "tidss_dispc.h"
#include "tidss_drv.h"
#include "tidss_encoder.h"
#include "tidss_kms.h"
#include "tidss_plane.h"

static void tidss_atomic_commit_tail(struct drm_atomic_state *old_state)
{
	struct drm_device *ddev = old_state->dev;
	struct tidss_device *tidss = to_tidss(ddev);
	bool fence_cookie = dma_fence_begin_signalling();

	dev_dbg(ddev->dev, "%s\n", __func__);

	tidss_runtime_get(tidss);

	drm_atomic_helper_commit_modeset_disables(ddev, old_state);
	drm_atomic_helper_commit_planes(ddev, old_state, 0);
	drm_atomic_helper_commit_modeset_enables(ddev, old_state);

	drm_atomic_helper_commit_hw_done(old_state);
	dma_fence_end_signalling(fence_cookie);
	drm_atomic_helper_wait_for_flip_done(ddev, old_state);

	drm_atomic_helper_cleanup_planes(ddev, old_state);

	tidss_runtime_put(tidss);
}

static const struct drm_mode_config_helper_funcs mode_config_helper_funcs = {
	.atomic_commit_tail = tidss_atomic_commit_tail,
};

static int tidss_atomic_check(struct drm_device *ddev,
			      struct drm_atomic_state *state)
{
	struct drm_plane_state *opstate;
	struct drm_plane_state *npstate;
	struct drm_plane *plane;
	struct drm_crtc_state *cstate;
	struct drm_crtc *crtc;
	int ret, i;

	ret = drm_atomic_helper_check(ddev, state);
	if (ret)
		return ret;

	/*
	 * Add all active planes on a CRTC to the atomic state, if
	 * x/y/z position or activity of any plane on that CRTC
	 * changes. This is needed for updating the plane positions in
	 * tidss_crtc_position_planes() which is called from
	 * crtc_atomic_enable() and crtc_atomic_flush(). We have an
	 * extra flag to mark x,y-position changes and together
	 * with zpos_changed the condition recognizes all the above
	 * cases.
	 */
	for_each_oldnew_plane_in_state(state, plane, opstate, npstate, i) {
		if (!npstate->crtc || !npstate->visible)
			continue;

		if (!opstate->crtc || opstate->crtc_x != npstate->crtc_x ||
		    opstate->crtc_y != npstate->crtc_y) {
			cstate = drm_atomic_get_crtc_state(state,
							   npstate->crtc);
			if (IS_ERR(cstate))
				return PTR_ERR(cstate);
			to_tidss_crtc_state(cstate)->plane_pos_changed = true;
		}
	}

	for_each_new_crtc_in_state(state, crtc, cstate, i) {
		if (to_tidss_crtc_state(cstate)->plane_pos_changed ||
		    cstate->zpos_changed) {
			ret = drm_atomic_add_affected_planes(state, crtc);
			if (ret)
				return ret;
		}
	}

	return 0;
}

static const struct drm_mode_config_funcs mode_config_funcs = {
	.fb_create = drm_gem_fb_create,
	.atomic_check = tidss_atomic_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static enum dispc_oldi_modes tidss_parse_oldi_properties(struct tidss_device *tidss)
{
	int pixel_order;
	enum dispc_oldi_modes oldi_mode;
	struct device_node *oldi0_port, *oldi1_port;

	/*
	 * For dual-link / clone mode connections, the OLDI ports are expected
	 * at port reg = 0 and 2, while for single-link cases the OLDI port is
	 * expected only at port reg = 0.
	 */
	const u32 portnum_oldi0 = 0, portnum_oldi1 = 2;

	oldi0_port = of_graph_get_port_by_id(tidss->dev->of_node, portnum_oldi0);
	oldi1_port = of_graph_get_port_by_id(tidss->dev->of_node, portnum_oldi1);

	if (!(oldi0_port || oldi1_port)) {
		/* Keep OLDI TXes OFF if neither OLDI port is present. */
		oldi_mode = OLDI_MODE_OFF;
	} else if (oldi0_port && !oldi1_port) {
		/*
		 * OLDI0 port found, but not OLDI1 port. Setting single
		 * link output mode.
		 */
		oldi_mode = OLDI_MODE_SINGLE_LINK;
	} else if (!oldi0_port && oldi1_port) {
		/*
		 * The 2nd OLDI TX cannot be operated alone. This use case is
		 * not supported in the HW. Since the pins for OLDIs 0 and 1 are
		 * separate, one could theoretically set a clone mode over OLDIs
		 * 0 and 1 and just simply not use the OLDI 0. This is a hacky
		 * way to enable only OLDI TX 1 and hence is not officially
		 * supported.
		 */
		dev_err(tidss->dev,
			"%s: Single Mode over OLDI 1 is not supported in HW.\n",
			__func__);
		oldi_mode = OLDI_MODE_UNSUPPORTED;
	} else {
		/*
		 * OLDI Ports found for both the OLDI TXes. The DSS is to be
		 * configured in either Dual Link or Clone Mode.
		 */
		pixel_order = drm_of_lvds_get_dual_link_pixel_order(oldi0_port,
								    oldi1_port);
		switch (pixel_order) {
		case -EINVAL:
			/*
			 * The dual link properties were not found in at least
			 * one of the sink nodes. Since 2 OLDI ports are present
			 * in the DT, it can be safely assumed that the required
			 * configuration is Clone Mode.
			 */
			oldi_mode = OLDI_MODE_CLONE_SINGLE_LINK;
			break;

		case DRM_LVDS_DUAL_LINK_EVEN_ODD_PIXELS:
			/*
			 * Note that the OLDI TX 0 transmits the odd set of
			 * pixels while the OLDI TX 1 transmits the even set.
			 * This is a fixed configuration in the HW and an cannot
			 * be change via SW.
			 */
			dev_err(tidss->dev,
				"%s: EVEN-ODD Dual-Link Mode is not supported in HW.\n",
				__func__);
			oldi_mode = OLDI_MODE_UNSUPPORTED;
			break;

		case DRM_LVDS_DUAL_LINK_ODD_EVEN_PIXELS:
			oldi_mode = OLDI_MODE_DUAL_LINK;
			break;

		default:
			dev_err(tidss->dev, "%s: Unrecognized OLDI mode.\n",
				__func__);
			oldi_mode = OLDI_MODE_UNSUPPORTED;
			break;
		}
	}

	of_node_put(oldi0_port);
	of_node_put(oldi1_port);

	return oldi_mode;
}

static int tidss_dispc_modeset_init(struct tidss_device *tidss)
{
	struct device *dev = tidss->dev;
	unsigned int fourccs_len;
	const u32 *fourccs = dispc_plane_formats(tidss->dispc, &fourccs_len);
	unsigned int i, j;

	struct pipe {
		u32 vp_idx;
		struct drm_bridge *bridge[TIDSS_MAX_BRIDGES_PER_PIPE];
		u32 enc_type;
		u32 num_bridges;
	};

	const struct dispc_features *feat = tidss->feat;
	u32 num_outputs = feat->num_outputs;
	u32 max_planes = feat->num_planes;

	struct pipe pipes[TIDSS_MAX_VPS] = {0};

	u32 num_pipes = 0;
	u32 crtc_mask;
	enum dispc_oldi_modes oldi_mode = OLDI_MODE_UNAVAILABLE;
	u32 num_oldi = 0;
	u32 num_encoders = 0;
	u32 oldi_pipe_index = 0;

	if (feat->has_oldi) {
		oldi_mode = tidss_parse_oldi_properties(tidss);

		if ((oldi_mode == OLDI_MODE_DUAL_LINK ||
		     oldi_mode == OLDI_MODE_CLONE_SINGLE_LINK) &&
		    feat->subrev == DISPC_AM65X) {
			dev_err(dev,
				"%s: am65x-dss does not support this OLDI mode.\n",
				__func__);
			oldi_mode = OLDI_MODE_UNSUPPORTED;
		}
	}

	if (oldi_mode == OLDI_MODE_UNSUPPORTED)
		return -EINVAL;

	dispc_set_oldi_mode(tidss->dispc, oldi_mode);

	/* first find all the connected panels & bridges */

	for (i = 0; i < num_outputs; i++) {
		struct drm_panel *panel;
		struct drm_bridge *bridge;
		u32 enc_type = DRM_MODE_ENCODER_NONE;
		int ret;

		ret = drm_of_find_panel_or_bridge(dev->of_node, i, 0,
						  &panel, &bridge);
		if (ret == -ENODEV) {
			dev_dbg(dev, "no panel/bridge for port %d\n", i);
			continue;
		} else if (ret) {
			dev_dbg(dev, "port %d probe returned %d\n", i, ret);
			return ret;
		}

		if (panel) {
			u32 conn_type;

			dev_dbg(dev, "Setting up panel for port %d\n", i);

			switch (feat->output_type[i]) {
			case DISPC_OUTPUT_OLDI:
				enc_type = DRM_MODE_ENCODER_LVDS;
				conn_type = DRM_MODE_CONNECTOR_LVDS;
				break;
			case DISPC_OUTPUT_DPI:
				enc_type = DRM_MODE_ENCODER_DPI;
				conn_type = DRM_MODE_CONNECTOR_DPI;
				break;
			default:
				WARN_ON(1);
				return -EINVAL;
			}

			if (panel->connector_type != conn_type) {
				dev_err(dev,
					"%s: Panel %s has incompatible connector type for vp%d (%d != %d)\n",
					 __func__, dev_name(panel->dev), i,
					 panel->connector_type, conn_type);
				return -EINVAL;
			}

			bridge = devm_drm_panel_bridge_add(dev, panel);
			if (IS_ERR(bridge)) {
				dev_err(dev,
					"failed to set up panel bridge for port %d\n",
					i);
				return PTR_ERR(bridge);
			}
		}

		if (feat->output_type[i] == DISPC_OUTPUT_OLDI) {
			switch (oldi_mode) {
			case OLDI_MODE_OFF:
				dev_dbg(dev, "OLDI disconnected on port %d\n", i);
				continue;

			case OLDI_MODE_DUAL_LINK:
				/*
				 * The 2nd OLDI port of a dual-link sink does
				 * not require a separate bridge entity.
				 */
				if (num_oldi > 0) {
					drm_panel_bridge_remove(bridge);
					continue;
				}

				fallthrough;

			case OLDI_MODE_CLONE_SINGLE_LINK:
			case OLDI_MODE_SINGLE_LINK:
				/*
				 * Setting up pipe parameters when 1st OLDI
				 * port is detected.
				 */
				if (num_oldi == 0) {
					pipes[num_pipes].vp_idx = feat->output_source_vp[i];
					pipes[num_pipes].enc_type = enc_type;

					/*
					 * Saving the pipe index in case its
					 * required for 2nd OLDI Port.
					 */
					oldi_pipe_index = num_pipes;

					/*
					 * Incrememnt num_pipe when 1st oldi
					 * port is discovered. For the 2nd OLDI
					 * port, num_pipe need not be
					 * incremented because the 2nd
					 * Encoder-to-Bridge connection will
					 * still be the part of the first OLDI
					 * Port pipe.
					 */
					num_pipes++;
				}

				/*
				 * Bridge is required to be added only if the
				 * detected port is the first OLDI port (of any
				 * mode) or a subsequent port in Clone Mode.
				 */
				pipes[oldi_pipe_index].bridge[num_oldi] = bridge;
				pipes[oldi_pipe_index].num_bridges++;
				num_oldi++;
				break;

			case OLDI_MODE_UNAVAILABLE:
			default:
				dev_dbg(dev, "OLDI unavailable on this device.\n");
				break;
			}
		} else {
			pipes[num_pipes].vp_idx = feat->output_source_vp[i];
			pipes[num_pipes].bridge[0] = bridge;
			pipes[num_pipes].num_bridges++;
			pipes[num_pipes].enc_type = enc_type;
			num_pipes++;
		}
	}

	/* all planes can be on any crtc */
	crtc_mask = (1 << num_pipes) - 1;

	/* then create a plane, a crtc and an encoder for each panel/bridge */

	for (i = 0; i < num_pipes; ++i) {
		struct tidss_plane *tplane;
		struct tidss_crtc *tcrtc;
		struct drm_encoder *enc;
		u32 possible_clones = 0;
		u32 hw_plane_id = feat->vid_order[tidss->num_planes];
		int ret;

		tplane = tidss_plane_create(tidss, hw_plane_id,
					    DRM_PLANE_TYPE_PRIMARY, crtc_mask,
					    fourccs, fourccs_len);
		if (IS_ERR(tplane)) {
			dev_err(tidss->dev, "plane create failed\n");
			return PTR_ERR(tplane);
		}

		tidss->planes[tidss->num_planes++] = &tplane->plane;

		tcrtc = tidss_crtc_create(tidss, pipes[i].vp_idx,
					  &tplane->plane);
		if (IS_ERR(tcrtc)) {
			dev_err(tidss->dev, "crtc create failed\n");
			return PTR_ERR(tcrtc);
		}

		tidss->crtcs[tidss->num_crtcs++] = &tcrtc->crtc;

		possible_clones = (((1 << pipes[i].num_bridges) - 1)
				   << num_encoders);

		for (j = 0; j < pipes[i].num_bridges; j++) {
			enc = tidss_encoder_create(tidss, pipes[i].enc_type,
						   1 << tcrtc->crtc.index,
						   possible_clones);
			if (IS_ERR(enc)) {
				dev_err(tidss->dev, "encoder create failed\n");
				return PTR_ERR(enc);
			}

			ret = drm_bridge_attach(enc, pipes[i].bridge[j], NULL, 0);
			if (ret)
				return ret;
		}
		num_encoders += pipes[i].num_bridges;
	}

	/* create overlay planes of the leftover planes */

	while (tidss->num_planes < max_planes) {
		struct tidss_plane *tplane;
		u32 hw_plane_id = feat->vid_order[tidss->num_planes];

		tplane = tidss_plane_create(tidss, hw_plane_id,
					    DRM_PLANE_TYPE_OVERLAY, crtc_mask,
					    fourccs, fourccs_len);

		if (IS_ERR(tplane)) {
			dev_err(tidss->dev, "plane create failed\n");
			return PTR_ERR(tplane);
		}

		tidss->planes[tidss->num_planes++] = &tplane->plane;
	}

	return 0;
}

int tidss_modeset_init(struct tidss_device *tidss)
{
	struct drm_device *ddev = &tidss->ddev;
	int ret;

	dev_dbg(tidss->dev, "%s\n", __func__);

	ret = drmm_mode_config_init(ddev);
	if (ret)
		return ret;

	ddev->mode_config.min_width = 8;
	ddev->mode_config.min_height = 8;
	ddev->mode_config.max_width = 8096;
	ddev->mode_config.max_height = 8096;
	ddev->mode_config.normalize_zpos = true;
	ddev->mode_config.funcs = &mode_config_funcs;
	ddev->mode_config.helper_private = &mode_config_helper_funcs;

	ret = tidss_dispc_modeset_init(tidss);
	if (ret)
		return ret;

	ret = drm_vblank_init(ddev, tidss->num_crtcs);
	if (ret)
		return ret;

	drm_mode_config_reset(ddev);

	dev_dbg(tidss->dev, "%s done\n", __func__);

	return 0;
}
