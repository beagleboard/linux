// SPDX-License-Identifier: GPL-2.0
/*
 * Cadence MHDP DP MST bridge driver.
 *
 * Copyright: 2018 Cadence Design Systems, Inc.
 *
 * Author: Piotr Sroka <piotrs@cadence.com>
 */
#include <drm/drm_atomic_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fixed.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <linux/iopoll.h>

#include <drm/bridge/cdns-mhdp-common.h>
#include "cdns-mhdp.h"


static void cdns_mhdp_mst_stream_enable(struct cdns_mhdp_bridge *mhdp_bridge,
					const bool enable)
{
	u32 reg;
	struct cdns_mhdp_device *mhdp = mhdp_bridge->mhdp;
	const u8 stream_id = mhdp_bridge->stream_id;

	cdns_mhdp_reg_read(mhdp, CDNS_DP_MST_STREAM_CONFIG(stream_id), &reg);

	if (enable) {
		reg |= CDNS_DP_MST_STRM_CFG_STREAM_EN;
		reg &= ~CDNS_DP_MST_STRM_CFG_NO_VIDEO;
	} else {
		reg &= ~CDNS_DP_MST_STRM_CFG_STREAM_EN;
		reg |= CDNS_DP_MST_STRM_CFG_NO_VIDEO;
	}

	cdns_mhdp_reg_write(mhdp, CDNS_DP_MST_STREAM_CONFIG(stream_id), reg);
}

static inline s64 calc_fixed_avg_slots(const u32 pbn, const u32 pbn_div)
{
	s64 fixed_pbn, fixed_pbn_div, fixed_targ_avg_slots;

	fixed_pbn = drm_int2fixp(pbn);
	fixed_pbn_div = drm_int2fixp(pbn_div);
	fixed_targ_avg_slots = drm_fixp_div(fixed_pbn, fixed_pbn_div);

	return fixed_targ_avg_slots;
}

static void cdns_mhdp_set_rate_governing(struct cdns_mhdp_bridge *bridge,
					 const bool enable)
{
	struct cdns_mhdp_device *mhdp = bridge->mhdp;
	const u8 stream_id = bridge->stream_id;
	s64 fixed_targ_avg_slots, fixed_y;
	u32 x, y;

	if (!enable) {
		cdns_mhdp_reg_write(mhdp, CDNS_DP_RATE_GOVERNING(stream_id), 0);
		return;
	}

	fixed_targ_avg_slots = calc_fixed_avg_slots(bridge->pbn,
						    mhdp->mst_mgr.pbn_div);
	x = bridge->pbn / mhdp->mst_mgr.pbn_div;

	fixed_y = fixed_targ_avg_slots - drm_int2fixp(x);
	fixed_y *= 16;

	y = drm_fixp2int_ceil(fixed_y);

	cdns_mhdp_reg_write(mhdp, CDNS_DP_RATE_GOVERNING(stream_id),
			    CDNS_DP_RG_TARG_AV_SLOTS_Y(y) |
			    CDNS_DP_RG_TARG_AV_SLOTS_X(x) |
			    CDNS_DP_RG_ENABLE);
}


static struct drm_dp_payload *
cdns_mhdp_get_payload(struct cdns_mhdp_bridge *bridge)
{
	const int vcpi = bridge->connector->port->vcpi.vcpi;
	struct cdns_mhdp_device *mhdp = bridge->mhdp;
	int i;

	for (i = 0; i < mhdp->mst_mgr.max_payloads; i++) {
		struct drm_dp_payload *payload = &mhdp->mst_mgr.payloads[i];

		if (payload->vcpi == vcpi)
			return payload;
	}

	return NULL;
}


static int
cdns_mhdp_set_act_enable(struct cdns_mhdp_device *mhdp)
{
	u32 reg;
	int ret;

	cdns_mhdp_reg_read(mhdp, CDNS_DP_MTPH_CONTROL, &reg);
	cdns_mhdp_reg_write(mhdp, CDNS_DP_MTPH_CONTROL,
			    reg | CDNS_DP_MTPH_ACT_EN);

	ret = readl_poll_timeout(mhdp->regs + CDNS_DP_MTPH_STATUS, reg,
				 ((reg & CDNS_DP_MTPH_ACT_STATUS) == 0), 0,
				 30);
	if (ret) {
		dev_err(mhdp->dev,
			"ACT sequence cannot complete in 30us\n");
		return -EIO;
	}

	return drm_dp_check_act_status(&mhdp->mst_mgr);
}


static int
cdns_mhdp_apply_slot_allocation(struct cdns_mhdp_bridge *mhdp_bridge)
{
	struct drm_dp_payload *payload;
	struct cdns_mhdp_device *mhdp = mhdp_bridge->mhdp;
	u8 stream_id = mhdp_bridge->stream_id;

	payload = cdns_mhdp_get_payload(mhdp_bridge);

	if (!payload) {
		DRM_ERROR("payload is not found\n");
		return -EIO;
	}

	cdns_mhdp_reg_write(mhdp, CDNS_DP_MST_SLOT_ALLOCATE(stream_id),
			    CDNS_DP_S_ALLOC_START_SLOT(payload->start_slot) |
			    CDNS_DP_S_ALLOC_END_SLOT(payload->start_slot +
						     payload->num_slots - 1));

	return 0;
}

static void
cdns_mhdp_update_slot_allocation(struct cdns_mhdp_bridge *mhdp_bridge)
{
	struct drm_device *dev = mhdp_bridge->base.dev;
	struct drm_connector *connector;

	list_for_each_entry(connector, &dev->mode_config.connector_list, head) {
		struct cdns_mhdp_connector *mhdp_connector;

		if (!connector->encoder)
			continue;

		mhdp_connector = to_mhdp_connector(connector);
		if (!mhdp_connector->is_mst_connector)
			continue;

		if (mhdp_connector->bridge->stream_id != -1)
			cdns_mhdp_apply_slot_allocation(mhdp_connector->bridge);
	}
}

static enum drm_connector_status
cdns_dp_mst_detect(struct drm_connector *connector, bool force)
{
	enum drm_connector_status stat;
	struct cdns_mhdp_connector *mhdp_connector;
	struct cdns_mhdp_device *mhdp;

	mhdp_connector = to_mhdp_connector(connector);
	mhdp =  mhdp_connector->bridge->mhdp;

	stat = drm_dp_mst_detect_port(connector, &mhdp->mst_mgr,
				      mhdp_connector->port);
	return stat;
}

static void cdns_dp_mst_connector_destroy(struct drm_connector *connector)
{
	struct cdns_mhdp_connector *mhdp_connector;
	struct cdns_mhdp_bridge *mhdp_bridge;

	mhdp_connector = to_mhdp_connector(connector);
	mhdp_bridge = mhdp_connector->bridge;

	drm_connector_cleanup(&mhdp_connector->base);
	drm_bridge_remove(&mhdp_bridge->base);
	kfree(mhdp_connector);
	kfree(mhdp_bridge);
}

static const struct drm_connector_funcs cdns_mhdp_mst_connector_funcs = {
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
	.reset = drm_atomic_helper_connector_reset,
	.dpms = drm_helper_connector_dpms,
	.detect = cdns_dp_mst_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = cdns_dp_mst_connector_destroy,
};

static int cdns_mhdp_mst_get_ddc_modes(struct drm_connector *connector)
{
	struct cdns_mhdp_connector *mhdp_connector;
	struct cdns_mhdp_device *mhdp;
	struct edid *edid;
	int num_modes = 0;

	mhdp_connector = to_mhdp_connector(connector);
	mhdp =  mhdp_connector->bridge->mhdp;

	edid = drm_dp_mst_get_edid(connector, &mhdp->mst_mgr,
				   mhdp_connector->port);
	if (edid) {
		DRM_DEBUG_KMS("edid retrieved %p\n", edid);
		drm_connector_update_edid_property(connector, edid);
		num_modes = drm_add_edid_modes(connector, edid);
		kfree(edid);
	} else
		drm_connector_update_edid_property(connector, NULL);

	return num_modes;
}

static int cdns_mhdp_mst_get_modes(struct drm_connector *connector)
{
	return cdns_mhdp_mst_get_ddc_modes(connector);
}

static struct
drm_encoder *cdns_mhdp_mst_best_encoder(struct drm_connector *connector)
{
	struct cdns_mhdp_connector *mhdp_connector;

	mhdp_connector = to_mhdp_connector(connector);

	return mhdp_connector->bridge->base.encoder;
}

static const struct drm_connector_helper_funcs cdns_mhdp_mst_conn_helper_fun = {
	.get_modes = cdns_mhdp_mst_get_modes,
	.best_encoder = cdns_mhdp_mst_best_encoder,
};

void cdns_mhdp_mst_enable(struct drm_bridge *bridge)
{
	struct cdns_mhdp_bridge *mhdp_bridge = to_mhdp_bridge(bridge);
	struct cdns_mhdp_device *mhdp = mhdp_bridge->mhdp;
	struct drm_display_info *disp_info;
	struct drm_display_mode *mode;
	struct cdns_mhdp_connector *mhdp_connector;
	u32 bpp;
	enum pixel_format pxlfmt;
	int ret, slots, stream_id;

	disp_info = &mhdp_bridge->connector->base.display_info;

	pxlfmt = cdns_mhdp_get_pxlfmt(disp_info->color_formats);
	bpp = cdns_mhdp_get_bpp(disp_info->bpc, pxlfmt);

	mhdp_connector = mhdp_bridge->connector;
	if (mhdp_bridge->stream_id > -1) {
		DRM_ERROR("stream id is attached before bridge is enabled\n");
		return;
	}

	stream_id = bridge->encoder->crtc->index;

	mode = &bridge->encoder->crtc->state->adjusted_mode;
	mhdp_bridge->pbn = drm_dp_calc_pbn_mode(mode->clock, bpp);

	slots = drm_dp_find_vcpi_slots(&mhdp->mst_mgr, mhdp_bridge->pbn);
	ret = drm_dp_mst_allocate_vcpi(&mhdp->mst_mgr,
				       mhdp_connector->port,
				       mhdp_bridge->pbn, slots);
	if (ret == false) {
		DRM_ERROR("failed to allocate vcpi\n");
		return;
	}
	ret = drm_dp_update_payload_part1(&mhdp->mst_mgr);
	if (ret < 0)
		DRM_ERROR("failed update_payload_part1\n");

	mhdp_bridge->stream_id = stream_id;
	mhdp_bridge->is_active = true;

	cdns_mhdp_mst_stream_enable(mhdp_bridge, true);
	cdns_mhdp_configure_video(bridge);

	ret = cdns_mhdp_apply_slot_allocation(mhdp_bridge);
	if (ret < 0) {
		cdns_mhdp_mst_stream_enable(mhdp_bridge, false);
		mhdp_bridge->stream_id = -1;
		mhdp_bridge->is_active = false;
		return;
	}

	ret = cdns_mhdp_set_act_enable(mhdp);
	if (ret)
		DRM_ERROR("failed ACT sequence\n");

	cdns_mhdp_set_rate_governing(mhdp_bridge, true);

	drm_dp_update_payload_part2(&mhdp->mst_mgr);
}

void cdns_mhdp_mst_disable(struct drm_bridge *bridge)
{
	struct cdns_mhdp_bridge *mhdp_bridge = to_mhdp_bridge(bridge);
	struct cdns_mhdp_device *mhdp = mhdp_bridge->mhdp;
	struct cdns_mhdp_connector *connector = mhdp_bridge->connector;

	drm_dp_mst_reset_vcpi_slots(&mhdp->mst_mgr, connector->port);
	drm_dp_update_payload_part1(&mhdp->mst_mgr);

	cdns_mhdp_update_slot_allocation(mhdp_bridge);

	drm_dp_check_act_status(&mhdp->mst_mgr);

	drm_dp_update_payload_part2(&mhdp->mst_mgr);

	drm_dp_mst_deallocate_vcpi(&mhdp->mst_mgr, connector->port);

	cdns_mhdp_set_rate_governing(mhdp_bridge, false);
	cdns_mhdp_mst_stream_enable(mhdp_bridge, false);
	mhdp_bridge->stream_id = -1;
	mhdp_bridge->is_active = false;
}

static const struct drm_bridge_funcs cdns_mhdp_mst_bridge_funcs = {
	.enable = cdns_mhdp_enable,
	.disable = cdns_mhdp_mst_disable,
};


static struct cdns_mhdp_bridge*
cdns_mhpd_create_fake_mst_bridge(struct cdns_mhdp_device *mhdp,
				struct cdns_mhdp_connector *mhdp_connector)
{
	struct cdns_mhdp_bridge *mhdp_bridge;
	struct drm_encoder *encoder = NULL;
	struct cdns_mhdp_mst_cbs *cbs = &mhdp->cbs;

	mhdp_bridge = kzalloc(sizeof(*mhdp_bridge), GFP_KERNEL);
	if (!mhdp_bridge)
		return NULL;

	mhdp_bridge->mhdp = mhdp;
	mhdp_bridge->stream_id = -1;
	mhdp_bridge->connector = mhdp_connector;
	mhdp_bridge->is_active = false;

	mhdp_bridge->base.funcs = &cdns_mhdp_mst_bridge_funcs;

	drm_bridge_add(&mhdp_bridge->base);

	if (cbs->funcs.create_mst_encoder)
		encoder = cbs->funcs.create_mst_encoder(cbs->priv_data,
							&mhdp_bridge->base);
	if (encoder) {
		int ret;
		/* use the same drm device as is in the first encoder */
		encoder->dev = mhdp->bridge.base.encoder->dev;
		encoder->possible_crtcs &= ((1 << CDNS_MHDP_MAX_STREAMS) - 1);
		ret = drm_bridge_attach(encoder, &mhdp_bridge->base, NULL);
		if (ret) {
			dev_err(mhdp->dev, "bridge attaching error %d\n", ret);
			return NULL;
		}

		ret = drm_connector_attach_encoder(&mhdp_connector->base,
							encoder);
		if (ret) {
			dev_err(mhdp->dev, "failed to attach connector to encoder\n");
			return NULL;
		}
	}

	return mhdp_bridge;
}

static struct drm_connector *
cdns_mhdp_mst_cbs_add_connector(struct drm_dp_mst_topology_mgr *mgr,
				struct drm_dp_mst_port *port,
				const char *pathprop)
{
	struct cdns_mhdp_device *mhdp = mgr_to_mhdp(mgr);
	struct drm_device *dev = mhdp->bridge.base.dev;
	struct cdns_mhdp_connector *mhdp_connector;
	struct drm_connector *connector;
	struct drm_connector_state  *conn_state;
	int ret;

	mhdp_connector = kzalloc(sizeof(struct cdns_mhdp_connector),
				 GFP_KERNEL);
	if (!mhdp_connector)
		return NULL;

	mhdp_connector->is_mst_connector = true;
	connector = &mhdp_connector->base;
	mhdp_connector->port = port;
	DRM_DEBUG_KMS("\n");

	conn_state = kzalloc(sizeof(*conn_state), GFP_KERNEL);
	if (!conn_state)
		return NULL;

	__drm_atomic_helper_connector_reset(connector,
					    conn_state);

	drm_connector_init(dev, connector, &cdns_mhdp_mst_connector_funcs,
			   DRM_MODE_CONNECTOR_DisplayPort);
	drm_connector_helper_add(connector, &cdns_mhdp_mst_conn_helper_fun);
	mhdp_connector->bridge =
		cdns_mhpd_create_fake_mst_bridge(mhdp, mhdp_connector);

	drm_object_attach_property(&connector->base,
				   dev->mode_config.path_property, 0);
	drm_object_attach_property(&connector->base,
				   dev->mode_config.tile_property, 0);
	ret = drm_connector_set_path_property(connector, pathprop);

	if (ret)
		DRM_ERROR("set path propertty failed\n");

	return connector;
}

static void
cdns_mhdp_mst_cbs_destroy_connector(struct drm_dp_mst_topology_mgr *mgr,
				    struct drm_connector *connector)
{
	struct cdns_mhdp_connector *mhdp_connector;
	struct cdns_mhdp_device *mhdp;
	struct cdns_mhdp_bridge *mhdp_bridge;

	mhdp_connector = to_mhdp_connector(connector);
	mhdp_bridge = mhdp_connector->bridge;
	mhdp = mhdp_bridge->mhdp;

	drm_connector_unregister(&mhdp_connector->base);

	if (mhdp->cbs.funcs.create_mst_encoder)
		mhdp->cbs.funcs.destroy_mst_encoder(mhdp->cbs.priv_data,
						    &mhdp_bridge->base);
	drm_connector_put(&mhdp_connector->base);
}

static void
cdns_mhdp_mst_cbs_register_connector(struct drm_connector *connector)
{
	int ret;

	ret = drm_connector_register(connector);
	if (ret)
		DRM_ERROR("Register connector failed\n");

}

static const struct drm_dp_mst_topology_cbs mst_cbs = {
	.add_connector = cdns_mhdp_mst_cbs_add_connector,
	.register_connector = cdns_mhdp_mst_cbs_register_connector,
	.destroy_connector = cdns_mhdp_mst_cbs_destroy_connector,
};

static void cdns_mhdp_set_mst_enable(struct cdns_mhdp_device *mhdp, bool enable)
{
	u32 reg_val;

	cdns_mhdp_reg_read(mhdp, CDNS_DP_FRAMER_GLOBAL_CONFIG, &reg_val);

	if (enable)
		reg_val |= CDNS_DP_MST_EN;
	else
		reg_val &= ~CDNS_DP_MST_EN;

	cdns_mhdp_reg_write(mhdp, CDNS_DP_FRAMER_GLOBAL_CONFIG, reg_val);
}

bool cdns_mhdp_mst_probe(struct cdns_mhdp_device *mhdp)
{
	u8 mstm_cap;
	u8 dpcd_cap[DP_RECEIVER_CAP_SIZE];

	bool is_mst;

	if (!mhdp->can_mst)
		return false;

	drm_dp_dpcd_read(&mhdp->aux, DP_DPCD_REV, dpcd_cap,
			 DP_RECEIVER_CAP_SIZE);

	if (dpcd_cap[DP_DPCD_REV] < 0x12)
		return false;

	if (drm_dp_dpcd_readb(&mhdp->aux, DP_MSTM_CAP, &mstm_cap) != 1)
		return false;

	if (mstm_cap & DP_MST_CAP) {
		DRM_DEBUG_KMS("Sink is MST capable\n");
		is_mst = true;
	} else {
		DRM_DEBUG_KMS("Sink is not MST capable\n");
		is_mst = false;
	}

	if (is_mst != mhdp->is_mst) {
		mhdp->is_mst = is_mst;
		cdns_mhdp_set_mst_enable(mhdp, mhdp->is_mst);

		drm_dp_mst_topology_mgr_set_mst(&mhdp->mst_mgr,
						mhdp->is_mst);
	}

	return mhdp->is_mst;
}

int cdns_mhdp_mst_init(struct cdns_mhdp_device *mhdp)
{
	struct cdns_mhdp_bridge *bridge = &mhdp->bridge;
	struct drm_device *dev = bridge->base.dev;
	struct cdns_mhdp_connector *connector = bridge->connector;
	int ret;

	mhdp->mst_mgr.cbs = &mst_cbs;
	ret = drm_dp_mst_topology_mgr_init(&mhdp->mst_mgr, dev,
					   &mhdp->aux, 16,
					   CDNS_MHDP_MAX_STREAMS,
					   connector->base.base.id);
	mhdp->can_mst = ret ? false : true;
	mhdp->is_mst = false;
	bridge->stream_id = -1;

	return ret;
}

void cdns_mhdp_mst_deinit(struct cdns_mhdp_device *mhdp)
{
	if (mhdp->is_mst) {
		mhdp->is_mst = false;
		drm_dp_mst_topology_mgr_set_mst(&mhdp->mst_mgr,
						mhdp->is_mst);
	}

	if (mhdp->can_mst)
		drm_dp_mst_topology_mgr_destroy(&mhdp->mst_mgr);
}
