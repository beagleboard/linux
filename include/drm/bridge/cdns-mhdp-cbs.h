/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Cadence MHDP DP bridge callbacks.
 *
 * Copyright: 2018 Cadence Design Systems, Inc.
 *
 * Author: Piotr Sroka <piotrs@cadence.com>
 */

#ifndef CDNS_MHDP_CBS_H
#define CDNS_MHDP_CBS_H

#include <drm/drm_bridge.h>

struct cdns_mhdp_mst_cbs_funcs {
	struct drm_encoder *(*create_mst_encoder)(void *priv_data,
						  struct drm_bridge *bridge);
	void (*destroy_mst_encoder)(void *priv_data, struct drm_bridge *bridge);
};

struct cdns_mhdp_mst_cbs {
	struct cdns_mhdp_mst_cbs_funcs funcs;
	void *priv_data;
};

int mhdp_bridge_attach_mst_cbs(struct drm_bridge *bridge,
			       struct cdns_mhdp_mst_cbs *cbs);

#endif
