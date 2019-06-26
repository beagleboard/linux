/*
 * Copyright (C) 2016 Chris Zhong <zyw@rock-chips.com>
 * Copyright (C) 2016 ROCKCHIP, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _CDN_DP_CORE_H
#define _CDN_DP_CORE_H

#include <drm/bridge/cdns-mhdp-common.h>
#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_dp_helper.h>
#include <drm/drm_panel.h>
#include "rockchip_drm_drv.h"

#define MAX_PHY		2

struct cdn_firmware_header {
	u32 size_bytes; /* size of the entire header+image(s) in bytes */
	u32 header_size; /* size of just the header in bytes */
	u32 iram_size; /* size of iram */
	u32 dram_size; /* size of dram */
};

struct cdn_dp_port {
	struct cdn_dp_device *dp;
	struct notifier_block event_nb;
	struct extcon_dev *extcon;
	struct phy *phy;
	u8 lanes;
	bool phy_enabled;
	u8 id;
};

struct cdn_dp_device {
	struct cdns_mhdp_device mhdp;
	struct drm_device *drm_dev;
	struct drm_encoder encoder;
	struct work_struct event_work;
	struct edid *edid;

	struct mutex lock;
	bool connected;
	bool active;
	bool suspended;

	const struct firmware *fw;	/* cdn dp firmware */
	bool fw_loaded;

	struct regmap *grf;
	struct clk *core_clk;
	struct clk *pclk;
	struct clk *grf_clk;
	struct reset_control *dptx_rst;
	struct reset_control *apb_rst;
	struct reset_control *core_rst;
	struct cdn_dp_port *port[MAX_PHY];
	u8 ports;
	u8 lanes;
	int active_port;

	u8 dpcd[DP_RECEIVER_CAP_SIZE];
	bool sink_has_audio;
};
#endif  /* _CDN_DP_CORE_H */
