// SPDX-License-Identifier: GPL-2.0
/*
 * Cadence MHDP DP bridge driver.
 *
 * Copyright: 2018 Cadence Design Systems, Inc.
 *
 * Author: Quentin Schulz <quentin.schulz@free-electrons.com>
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/phy/phy.h>
#include <linux/phy/phy-dp.h>

#include <drm/bridge/cdns-mhdp-common.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_dp_helper.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_print.h>
#include <drm/drm_crtc_helper.h>

#include <sound/hdmi-codec.h>
#include <linux/irq.h>
#include <linux/of_irq.h>

#include "cdns-mhdp.h"
#include "cdns-mhdp-j721e.h"

#define FW_NAME					"cadence/mhdp8546.bin"
#define CDNS_MHDP_IMEM				0x10000

#define CDNS_DP_TRAINING_PATTERN_4		0x7

#define CDNS_KEEP_ALIVE_TIMEOUT			2000

static const struct of_device_id mhdp_ids[] = {
	{ .compatible = "cdns,mhdp8546", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mhdp_ids);

#define CDNS_LANE_1				BIT(0)
#define CDNS_LANE_2				BIT(1)
#define CDNS_LANE_4				BIT(2)
#define CDNS_SSC				BIT(3)
#define CDNS_SCRAMBLER				BIT(4)

#define CDNS_VOLT_SWING(x)			((x) & GENMASK(1, 0))
#define CDNS_FORCE_VOLT_SWING			BIT(2)

#define CDNS_PRE_EMPHASIS(x)			((x) & GENMASK(1, 0))
#define CDNS_FORCE_PRE_EMPHASIS			BIT(2)

#define CDNS_SUPPORT_TPS(x)			BIT((x) - 1)

#define CDNS_FAST_LINK_TRAINING			BIT(0)

#define CDNS_LANE_MAPPING_TYPE_C_LANE_0(x)	((x) & GENMASK(1, 0))
#define CDNS_LANE_MAPPING_TYPE_C_LANE_1(x)	((x) & GENMASK(3, 2))
#define CDNS_LANE_MAPPING_TYPE_C_LANE_2(x)	((x) & GENMASK(5, 4))
#define CDNS_LANE_MAPPING_TYPE_C_LANE_3(x)	((x) & GENMASK(7, 6))
#define CDNS_LANE_MAPPING_NORMAL		0xe4
#define CDNS_LANE_MAPPING_FLIPPED		0x1b

#define CDNS_DP_MAX_NUM_LANES			4
#define CDNS_DP_TEST_VSC_SDP			(1 << 6) /* 1.3+ */
#define CDNS_DP_TEST_COLOR_FORMAT_RAW_Y_ONLY	(1 << 7)

static inline struct cdns_mhdp_device *connector_to_mhdp(
	struct drm_connector *conn)
{
	struct cdns_mhdp_connector *mhdp_connector = to_mhdp_connector(conn);

	return mhdp_connector->bridge->mhdp;
}

static inline struct cdns_mhdp_device *bridge_to_mhdp(
	struct drm_bridge *bridge)
{
	struct cdns_mhdp_bridge *mhdp_bridge = to_mhdp_bridge(bridge);

	return mhdp_bridge->mhdp;
}

static unsigned int max_link_rate(struct cdns_mhdp_host host,
				  struct cdns_mhdp_sink sink)
{
	return min(host.link_rate, sink.link_rate);
}

static u8 eq_training_pattern_supported(struct cdns_mhdp_host host,
					struct cdns_mhdp_sink sink)
{
	return fls(host.pattern_supp & sink.pattern_supp);
}

static irqreturn_t mhdp_irq_handler(int irq, void *data)
{
	struct cdns_mhdp_device *mhdp = (struct cdns_mhdp_device *)data;
	u32 mbox_stat, apb_stat, sw_ev0, sw_ev1, sw_ev2, sw_ev3;

	apb_stat = readl(mhdp->regs + CDNS_APB_INT_STATUS);
	mbox_stat = readl(mhdp->regs + CDNS_MB_INT_STATUS);
	sw_ev0 = readl(mhdp->regs + CDNS_SW_EVENT0);
	sw_ev1 = readl(mhdp->regs + CDNS_SW_EVENT1);
	sw_ev2 = readl(mhdp->regs + CDNS_SW_EVENT2);
	sw_ev3 = readl(mhdp->regs + CDNS_SW_EVENT3);

	//dev_dbg(mhdp->dev, "MHDP IRQ apb %x, mbox %x, sw_ev %x/%x/%x/%x\n", apb_stat, mbox_stat, sw_ev0, sw_ev1, sw_ev2, sw_ev3);

	if (sw_ev0 & CDNS_DPTX_HPD)
		drm_kms_helper_hotplug_event(mhdp->bridge.base.dev);

	return IRQ_HANDLED;
}

static ssize_t mhdp_transfer(struct drm_dp_aux *aux,
			     struct drm_dp_aux_msg *msg)
{
	struct cdns_mhdp_device *mhdp = dev_get_drvdata(aux->dev);
	int ret;

	if (msg->request != DP_AUX_NATIVE_WRITE &&
	    msg->request != DP_AUX_NATIVE_READ)
		return -ENOTSUPP;

	if (msg->request == DP_AUX_NATIVE_WRITE) {
		const u8 *buf = msg->buffer;
		int i;

		for (i = 0; i < msg->size; ++i) {
			ret = cdns_mhdp_dpcd_write(mhdp,
						   msg->address + i, buf[i]);
			if (!ret)
				continue;

			DRM_DEV_ERROR(mhdp->dev, "Failed to write DPCD\n");

			return ret;
		}
	} else {
		ret = cdns_mhdp_dpcd_read(mhdp, msg->address,
					  msg->buffer, msg->size);
		if (ret) {
			DRM_DEV_ERROR(mhdp->dev, "Failed to read DPCD\n");

			return ret;
		}
	}

	return msg->size;
}

static int cdns_mhdp_get_modes(struct drm_connector *connector)
{
	struct cdns_mhdp_device *mhdp = connector_to_mhdp(connector);
	struct edid *edid;
	int num_modes;

	edid = drm_do_get_edid(connector, cdns_mhdp_get_edid_block, mhdp);
	if (!edid) {
		DRM_DEV_ERROR(mhdp->dev, "Failed to read EDID\n");

		return 0;
	}

	drm_connector_update_edid_property(connector, edid);
	num_modes = drm_add_edid_modes(connector, edid);
	kfree(edid);

	/*
	 * HACK: Warn about unsupported display formats until we deal
	 *       with them correctly.
	 */
	if (!(connector->display_info.color_formats &
	      mhdp->display_fmt.color_format))
		dev_warn(mhdp->dev,
			 "%s: No supported color_format found (0x%08x)\n",
			__func__, connector->display_info.color_formats);

	if (connector->display_info.bpc < mhdp->display_fmt.bpc)
		dev_warn(mhdp->dev, "%s: Display bpc only %d < %d\n",
			 __func__, connector->display_info.bpc,
			 mhdp->display_fmt.bpc);

	return num_modes;
}

static const struct drm_connector_helper_funcs cdns_mhdp_conn_helper_funcs = {
	.get_modes = cdns_mhdp_get_modes,
};

static enum drm_connector_status cdns_mhdp_detect(struct drm_connector *conn,
						  bool force)
{
	struct cdns_mhdp_device *mhdp = connector_to_mhdp(conn);
	int ret;

	ret = cdns_mhdp_get_hpd_status(mhdp);
	if (ret > 0) {
		mhdp->plugged = true;
		return connector_status_connected;
	}
	if (ret < 0)
		dev_err(mhdp->dev, "Failed to obtain HPD state\n");

	mhdp->plugged = false;

	return connector_status_disconnected;
}

static const struct drm_connector_funcs cdns_mhdp_conn_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
	.reset = drm_atomic_helper_connector_reset,
	.destroy = drm_connector_cleanup,
	.detect = cdns_mhdp_detect,
};

static int cdns_mhdp_attach(struct drm_bridge *bridge)
{
	struct cdns_mhdp_device *mhdp = bridge_to_mhdp(bridge);
	u32 bus_format = MEDIA_BUS_FMT_RGB121212_1X36;
	struct drm_connector *conn = &mhdp->connector.base;
	int ret;

	if (&mhdp->bridge.base != bridge)
		return -ENODEV;

	conn->polled = DRM_CONNECTOR_POLL_HPD;

	ret = drm_connector_init(bridge->dev, conn, &cdns_mhdp_conn_funcs,
				 DRM_MODE_CONNECTOR_DisplayPort);
	if (ret) {
		dev_err(mhdp->dev, "failed to init connector\n");
		return ret;
	}

	drm_connector_helper_add(conn, &cdns_mhdp_conn_helper_funcs);

	ret = drm_display_info_set_bus_formats(&conn->display_info,
					       &bus_format, 1);
	if (ret)
		return ret;

	conn->display_info.bus_flags = DRM_BUS_FLAG_DE_HIGH;
	/*
	 * HACK: DP is internal to J7 SoC and we need to use DRIVE_POSEDGE
	 * in the display controller. This is achieved for the time being
	 * by defining SAMPLE_NEGEDGE here.
	 */
	conn->display_info.bus_flags |= DRM_BUS_FLAG_PIXDATA_SAMPLE_NEGEDGE |
		DRM_BUS_FLAG_SYNC_SAMPLE_NEGEDGE;

	ret = drm_connector_attach_encoder(conn, bridge->encoder);
	if (ret) {
		dev_err(mhdp->dev, "failed to attach connector to encoder\n");
		return ret;
	}

	/* enable interrupts */
	//writel(~CDNS_APB_INT_MASK_SW_EVENT_INT, mhdp->regs + CDNS_APB_INT_MASK);
	writel(0, mhdp->regs + CDNS_APB_INT_MASK);
	writel(0, mhdp->regs + CDNS_MB_INT_MASK);

	return 0;
}

static void mhdp_link_training_init(struct cdns_mhdp_device *mhdp)
{
	u32 reg32;
	u8 i;
	union phy_configure_opts phy_cfg;

	drm_dp_dpcd_writeb(&mhdp->aux, DP_TRAINING_PATTERN_SET,
			   DP_TRAINING_PATTERN_DISABLE);

	/* Reset PHY configuration */
	reg32 = CDNS_PHY_COMMON_CONFIG | CDNS_PHY_TRAINING_TYPE(1);
	if (!(mhdp->host.lanes_cnt & CDNS_SCRAMBLER))
		reg32 |= CDNS_PHY_SCRAMBLER_BYPASS;

	cdns_mhdp_reg_write(mhdp, CDNS_DPTX_PHY_CONFIG, reg32);

	cdns_mhdp_reg_write(mhdp, CDNS_DP_ENHNCD,
			    mhdp->sink.enhanced & mhdp->host.enhanced);

	cdns_mhdp_reg_write(mhdp, CDNS_DP_LANE_EN,
			    CDNS_DP_LANE_EN_LANES(mhdp->link.num_lanes));

	drm_dp_link_configure(&mhdp->aux, &mhdp->link);
	phy_cfg.dp.link_rate = (mhdp->link.rate / 100);
	phy_cfg.dp.lanes = (mhdp->link.num_lanes);
	for (i = 0; i < 4; i++) {
		phy_cfg.dp.voltage[i] = 0;
		phy_cfg.dp.pre[i] = 0;
	}
	phy_cfg.dp.ssc = false;
	phy_cfg.dp.set_lanes = true;
	phy_cfg.dp.set_rate = true;
	phy_cfg.dp.set_voltages = true;
	phy_configure(mhdp->phy,  &phy_cfg);

	cdns_mhdp_reg_write(mhdp, CDNS_DPTX_PHY_CONFIG,
			    CDNS_PHY_COMMON_CONFIG |
			    CDNS_PHY_TRAINING_EN |
			    CDNS_PHY_TRAINING_TYPE(1) |
			    CDNS_PHY_SCRAMBLER_BYPASS);

	drm_dp_dpcd_writeb(&mhdp->aux, DP_TRAINING_PATTERN_SET,
			   DP_TRAINING_PATTERN_1 | DP_LINK_SCRAMBLING_DISABLE);
}

static void mhdp_get_adjust_train(struct cdns_mhdp_device *mhdp,
				  u8 link_status[DP_LINK_STATUS_SIZE],
				  u8 lanes_data[CDNS_DP_MAX_NUM_LANES],
				  union phy_configure_opts *phy_cfg)
{
	unsigned int i;
	u8 adjust, max_pre_emphasis, max_volt_swing;
	u8 set_volt, set_pre;

	max_pre_emphasis = CDNS_PRE_EMPHASIS(mhdp->host.pre_emphasis)
			   << DP_TRAIN_PRE_EMPHASIS_SHIFT;
	max_volt_swing = CDNS_VOLT_SWING(mhdp->host.volt_swing);

	for (i = 0; i < mhdp->link.num_lanes; i++) {
		/* Check if Voltage swing and pre-emphasis are within limits */
		adjust = drm_dp_get_adjust_request_voltage(link_status, i);
		set_volt = min_t(u8, adjust, max_volt_swing);

		adjust = drm_dp_get_adjust_request_pre_emphasis(link_status, i);
		set_pre = min_t(u8, adjust, max_pre_emphasis) >> DP_TRAIN_PRE_EMPHASIS_SHIFT;

		/* Voltage swing level and pre-emphasis level combination is not allowed:
		 * leaving pre-emphasis as-is, and adjusting voltage swing.
		 */
		if (set_volt + set_pre > 3)
			set_volt = 3 - set_pre;

		phy_cfg->dp.voltage[i] = set_volt;
		lanes_data[i] = set_volt;

		if (set_volt == max_volt_swing)
			lanes_data[i] |= DP_TRAIN_MAX_SWING_REACHED;

		phy_cfg->dp.pre[i] = set_pre;
		lanes_data[i] |= (set_pre << DP_TRAIN_PRE_EMPHASIS_SHIFT);

		if (set_pre == (max_pre_emphasis >> DP_TRAIN_PRE_EMPHASIS_SHIFT))
			lanes_data[i] |= DP_TRAIN_MAX_PRE_EMPHASIS_REACHED;
	}
}

static void mhdp_set_adjust_request_voltage(
	u8 link_status[DP_LINK_STATUS_SIZE], int lane, u8 volt)
{
	int i = DP_ADJUST_REQUEST_LANE0_1 + (lane >> 1);
	int s = ((lane & 1) ?
		 DP_ADJUST_VOLTAGE_SWING_LANE1_SHIFT :
		 DP_ADJUST_VOLTAGE_SWING_LANE0_SHIFT);
	int idx = i - DP_LANE0_1_STATUS;

	link_status[idx] &= ~(DP_ADJUST_VOLTAGE_SWING_LANE0_MASK << s);
	link_status[idx] |= volt << s;
}

static void mhdp_set_adjust_request_pre_emphasis(
	u8 link_status[DP_LINK_STATUS_SIZE], int lane, u8 pre_emphasis)
{
	int i = DP_ADJUST_REQUEST_LANE0_1 + (lane >> 1);
	int s = ((lane & 1) ?
		 DP_ADJUST_PRE_EMPHASIS_LANE1_SHIFT :
		 DP_ADJUST_PRE_EMPHASIS_LANE0_SHIFT);
	int idx = i - DP_LANE0_1_STATUS;

	link_status[idx] &= ~(DP_ADJUST_PRE_EMPHASIS_LANE0_MASK << s);
	link_status[idx] |= pre_emphasis << s;
}

static void mhdp_adjust_requested_eq(struct cdns_mhdp_device *mhdp,
				     u8 link_status[DP_LINK_STATUS_SIZE])
{
	unsigned int i;
	u8 volt, pre, max_volt = CDNS_VOLT_SWING(mhdp->host.volt_swing),
		      max_pre = CDNS_PRE_EMPHASIS(mhdp->host.pre_emphasis);

	for (i = 0; i < mhdp->link.num_lanes; i++) {
		volt = drm_dp_get_adjust_request_voltage(link_status, i);
		pre = drm_dp_get_adjust_request_pre_emphasis(link_status, i);
		if (volt + pre > 3)
			mhdp_set_adjust_request_voltage(link_status, i,
							3 - pre);
		if (mhdp->host.volt_swing & CDNS_FORCE_VOLT_SWING)
			mhdp_set_adjust_request_voltage(link_status, i,
							max_volt);
		if (mhdp->host.pre_emphasis & CDNS_FORCE_PRE_EMPHASIS)
			mhdp_set_adjust_request_pre_emphasis(link_status, i,
							     max_pre);
	}
}

static bool mhdp_link_training_channel_eq(struct cdns_mhdp_device *mhdp,
					  u8 eq_tps,
					  unsigned int training_interval)
{
	u8 lanes_data[CDNS_DP_MAX_NUM_LANES], fail_counter_short = 0;
	u8 dpcd[DP_LINK_STATUS_SIZE];
	u32 reg32;
	union phy_configure_opts phy_cfg;

	dev_dbg(mhdp->dev, "Link training - Starting EQ phase\n");

	/* Enable link training TPS[eq_tps] in PHY */
	reg32 = CDNS_PHY_COMMON_CONFIG | CDNS_PHY_TRAINING_EN |
		CDNS_PHY_TRAINING_TYPE(eq_tps);
	if (eq_tps != 4)
		reg32 |= CDNS_PHY_SCRAMBLER_BYPASS;
	cdns_mhdp_reg_write(mhdp, CDNS_DPTX_PHY_CONFIG, reg32);

	drm_dp_dpcd_writeb(&mhdp->aux, DP_TRAINING_PATTERN_SET,
			   (eq_tps != 4) ? eq_tps | DP_LINK_SCRAMBLING_DISABLE :
			   CDNS_DP_TRAINING_PATTERN_4);

	drm_dp_dpcd_read_link_status(&mhdp->aux, dpcd);

	do {
		mhdp_get_adjust_train(mhdp, dpcd, lanes_data, &phy_cfg);
		phy_cfg.dp.lanes = (mhdp->link.num_lanes);
		phy_cfg.dp.ssc = false;
		phy_cfg.dp.set_lanes = false;
		phy_cfg.dp.set_rate = false;
		phy_cfg.dp.set_voltages = true;
		phy_configure(mhdp->phy,  &phy_cfg);

		cdns_mhdp_adjust_lt(mhdp, mhdp->link.num_lanes,
				    training_interval, lanes_data, dpcd);

		if (!drm_dp_clock_recovery_ok(dpcd, mhdp->link.num_lanes))
			goto err;

		if (drm_dp_channel_eq_ok(dpcd, mhdp->link.num_lanes)) {
			dev_dbg(mhdp->dev,
				"Link training: EQ phase succeeded\n");
			return true;
		}

		fail_counter_short++;

		mhdp_adjust_requested_eq(mhdp, dpcd);
	} while (fail_counter_short < 5);

err:
	dev_dbg(mhdp->dev,
		"Link training - EQ phase failed for %d lanes and %d rate\n",
		mhdp->link.num_lanes, mhdp->link.rate);

	return false;
}

static void mhdp_adjust_requested_cr(struct cdns_mhdp_device *mhdp,
				     u8 link_status[DP_LINK_STATUS_SIZE],
				     u8 *req_volt, u8 *req_pre)
{
	const u32 max_volt = CDNS_VOLT_SWING(mhdp->host.volt_swing),
		  max_pre = CDNS_PRE_EMPHASIS(mhdp->host.pre_emphasis);
	unsigned int i;

	for (i = 0; i < mhdp->link.num_lanes; i++) {
		unsigned int val;

		val = mhdp->host.volt_swing & CDNS_FORCE_VOLT_SWING ?
		      max_volt : req_volt[i];
		mhdp_set_adjust_request_voltage(link_status, i, val);

		val = mhdp->host.pre_emphasis & CDNS_FORCE_PRE_EMPHASIS ?
		      max_pre : req_pre[i];
		mhdp_set_adjust_request_pre_emphasis(link_status, i, val);
	}
}

static void mhdp_validate_cr(struct cdns_mhdp_device *mhdp, bool *cr_done,
			     bool *same_before_adjust, bool *max_swing_reached,
			     u8 before_cr[DP_LINK_STATUS_SIZE],
			     u8 after_cr[DP_LINK_STATUS_SIZE], u8 *req_volt,
			     u8 *req_pre)
{
	const u8 max_volt = CDNS_VOLT_SWING(mhdp->host.volt_swing),
		 max_pre = CDNS_PRE_EMPHASIS(mhdp->host.pre_emphasis);
	bool same_pre, same_volt;
	unsigned int i;

	*same_before_adjust = false;
	*max_swing_reached = false;
	*cr_done = drm_dp_clock_recovery_ok(after_cr, mhdp->link.num_lanes);

	for (i = 0; i < mhdp->link.num_lanes; i++) {
		u8 tmp;

		tmp = drm_dp_get_adjust_request_voltage(after_cr, i);
		req_volt[i] = min_t(u8, tmp, max_volt);

		tmp = drm_dp_get_adjust_request_pre_emphasis(after_cr, i) >>
		      DP_TRAIN_PRE_EMPHASIS_SHIFT;
		req_pre[i] = min_t(u8, tmp, max_pre);

		same_pre = (before_cr[i] & DP_TRAIN_PRE_EMPHASIS_MASK) ==
			   req_pre[i] << DP_TRAIN_PRE_EMPHASIS_SHIFT;
		same_volt = (before_cr[i] & DP_TRAIN_VOLTAGE_SWING_MASK) ==
			    req_volt[i];
		if (same_pre && same_volt)
			*same_before_adjust = true;

		/* 3.1.5.2 in DP Standard v1.4. Table 3-1 */
		if (!*cr_done && req_volt[i] + req_pre[i] >= 3) {
			*max_swing_reached = true;
			return;
		}
	}
}

static bool mhdp_link_training_clock_recovery(struct cdns_mhdp_device *mhdp)
{
	u8 lanes_data[CDNS_DP_MAX_NUM_LANES],
	fail_counter_short = 0, fail_counter_cr_long = 0;
	u8 dpcd[DP_LINK_STATUS_SIZE];
	bool cr_done;
	union phy_configure_opts phy_cfg;

	dev_dbg(mhdp->dev, "Link training starting CR phase\n");

	mhdp_link_training_init(mhdp);

	drm_dp_dpcd_read_link_status(&mhdp->aux, dpcd);

	do {
		u8 requested_adjust_volt_swing[CDNS_DP_MAX_NUM_LANES] = {},
									requested_adjust_pre_emphasis[CDNS_DP_MAX_NUM_LANES] = {};
		bool same_before_adjust, max_swing_reached;

		mhdp_get_adjust_train(mhdp, dpcd, lanes_data, &phy_cfg);
		phy_cfg.dp.lanes = (mhdp->link.num_lanes);
		phy_cfg.dp.ssc = false;
		phy_cfg.dp.set_lanes = false;
		phy_cfg.dp.set_rate = false;
		phy_cfg.dp.set_voltages = true;
		phy_configure(mhdp->phy,  &phy_cfg);

		cdns_mhdp_adjust_lt(mhdp, mhdp->link.num_lanes, 100,
				    lanes_data, dpcd);

		mhdp_validate_cr(mhdp, &cr_done, &same_before_adjust,
				 &max_swing_reached, lanes_data, dpcd,
				 requested_adjust_volt_swing,
				 requested_adjust_pre_emphasis);

		if (max_swing_reached) {
			dev_err(mhdp->dev, "CR: max swing reached\n");
			goto err;
		}

		if (cr_done) {
			dev_dbg(mhdp->dev,
				"Link training: CR phase succeeded\n");
			return true;
		}

		/* Not all CR_DONE bits set */
		fail_counter_cr_long++;

		if (same_before_adjust) {
			fail_counter_short++;
			continue;
		}

		fail_counter_short = 0;
		/*
		 * Voltage swing/pre-emphasis adjust requested
		 * during CR phase
		 */
		mhdp_adjust_requested_cr(mhdp, dpcd,
					 requested_adjust_volt_swing,
					 requested_adjust_pre_emphasis);
	} while (fail_counter_short < 5 && fail_counter_cr_long < 10);

err:
	dev_dbg(mhdp->dev,
		"Link training: CR phase failed for %d lanes and %d rate\n",
		mhdp->link.num_lanes, mhdp->link.rate);

	return false;
}

static void lower_link_rate(struct drm_dp_link *link)
{
	switch (drm_dp_link_rate_to_bw_code(link->rate)) {
	case DP_LINK_BW_2_7:
		link->rate = drm_dp_bw_code_to_link_rate(DP_LINK_BW_1_62);
		break;
	case DP_LINK_BW_5_4:
		link->rate = drm_dp_bw_code_to_link_rate(DP_LINK_BW_2_7);
		break;
	case DP_LINK_BW_8_1:
		link->rate = drm_dp_bw_code_to_link_rate(DP_LINK_BW_5_4);
		break;
	}
}

static int mhdp_link_training(struct cdns_mhdp_device *mhdp,
			      unsigned int video_mode,
			      unsigned int training_interval)
{
	u32 reg32;
	union phy_configure_opts phy_cfg;
	const u8 eq_tps = eq_training_pattern_supported(mhdp->host, mhdp->sink);

	while (1) {
		if (!mhdp_link_training_clock_recovery(mhdp)) {
			if (drm_dp_link_rate_to_bw_code(mhdp->link.rate) !=
			    DP_LINK_BW_1_62) {
				dev_dbg(mhdp->dev,
					"Reducing link rate during CR phase\n");
				lower_link_rate(&mhdp->link);
				drm_dp_link_configure(&mhdp->aux, &mhdp->link);
				phy_cfg.dp.link_rate = (mhdp->link.rate / 100);
				phy_cfg.dp.lanes = (mhdp->link.num_lanes);
				phy_cfg.dp.ssc = false;
				phy_cfg.dp.set_lanes = false;
				phy_cfg.dp.set_rate = true;
				phy_cfg.dp.set_voltages = false;
				phy_configure(mhdp->phy,  &phy_cfg);

				continue;
			} else if (mhdp->link.num_lanes > 1) {
				dev_dbg(mhdp->dev,
					"Reducing lanes number during CR phase\n");
				mhdp->link.num_lanes >>= 1;
				mhdp->link.rate = max_link_rate(mhdp->host,
								mhdp->sink);
				drm_dp_link_configure(&mhdp->aux, &mhdp->link);
				phy_cfg.dp.link_rate = (mhdp->link.rate / 100);
				phy_cfg.dp.lanes = (mhdp->link.num_lanes);
				phy_cfg.dp.ssc = false;
				phy_cfg.dp.set_lanes = true;
				phy_cfg.dp.set_rate = false;
				phy_cfg.dp.set_voltages = false;
				phy_configure(mhdp->phy,  &phy_cfg);

				continue;
			}

			dev_dbg(mhdp->dev,
				"Link training failed during CR phase\n");
			goto err;
		}

		if (mhdp_link_training_channel_eq(mhdp, eq_tps,
						  training_interval))
			break;

		if (mhdp->link.num_lanes > 1) {
			dev_dbg(mhdp->dev,
				"Reducing lanes number during EQ phase\n");
			mhdp->link.num_lanes >>= 1;
			drm_dp_link_configure(&mhdp->aux, &mhdp->link);
			phy_cfg.dp.link_rate = (mhdp->link.rate / 100);
			phy_cfg.dp.lanes = (mhdp->link.num_lanes);
			phy_cfg.dp.ssc = false;
			phy_cfg.dp.set_lanes = true;
			phy_cfg.dp.set_rate = false;
			phy_cfg.dp.set_voltages = false;
			phy_configure(mhdp->phy,  &phy_cfg);

			continue;
		} else if (drm_dp_link_rate_to_bw_code(mhdp->link.rate) !=
			   DP_LINK_BW_1_62) {
			dev_dbg(mhdp->dev,
				"Reducing link rate during EQ phase\n");
			lower_link_rate(&mhdp->link);
			drm_dp_link_configure(&mhdp->aux, &mhdp->link);
			phy_cfg.dp.link_rate = (mhdp->link.rate / 100);
			phy_cfg.dp.lanes = (mhdp->link.num_lanes);
			phy_cfg.dp.ssc = false;
			phy_cfg.dp.set_lanes = false;
			phy_cfg.dp.set_rate = true;
			phy_cfg.dp.set_voltages = false;
			phy_configure(mhdp->phy,  &phy_cfg);

			continue;
		}

		dev_dbg(mhdp->dev, "Link training failed during EQ phase\n");
		goto err;
	}

	dev_dbg(mhdp->dev, "Link training successful\n");

	drm_dp_dpcd_writeb(&mhdp->aux, DP_TRAINING_PATTERN_SET,
			   (mhdp->host.lanes_cnt & CDNS_SCRAMBLER) ? 0 :
			   DP_LINK_SCRAMBLING_DISABLE);

	/* SW reset DPTX framer */
	cdns_mhdp_reg_write(mhdp, CDNS_DP_SW_RESET, 1);
	cdns_mhdp_reg_write(mhdp, CDNS_DP_SW_RESET, 0);

	cdns_mhdp_reg_write(mhdp, CDNS_DP_FRAMER_GLOBAL_CONFIG,
			    CDNS_DP_NUM_LANES(mhdp->link.num_lanes) |
			    CDNS_DP_DISABLE_PHY_RST |
			    CDNS_DP_WR_FAILING_EDGE_VSYNC |
			    (!video_mode ? CDNS_DP_NO_VIDEO_MODE : 0));

	/* Reset PHY config */
	reg32 = CDNS_PHY_COMMON_CONFIG | CDNS_PHY_TRAINING_TYPE(1);
	if (!(mhdp->host.lanes_cnt & CDNS_SCRAMBLER))
		reg32 |= CDNS_PHY_SCRAMBLER_BYPASS;
	cdns_mhdp_reg_write(mhdp, CDNS_DPTX_PHY_CONFIG, reg32);

	return 0;
err:
	/* Reset PHY config */
	reg32 = CDNS_PHY_COMMON_CONFIG | CDNS_PHY_TRAINING_TYPE(1);
	if (!(mhdp->host.lanes_cnt & CDNS_SCRAMBLER))
		reg32 |= CDNS_PHY_SCRAMBLER_BYPASS;
	cdns_mhdp_reg_write(mhdp, CDNS_DPTX_PHY_CONFIG, reg32);

	drm_dp_dpcd_writeb(&mhdp->aux, DP_TRAINING_PATTERN_SET,
			   DP_TRAINING_PATTERN_DISABLE);

	return -EIO;
}

static void cdns_mhdp_disable(struct drm_bridge *bridge)
{
	struct cdns_mhdp_device *mhdp = bridge_to_mhdp(bridge);

	dev_dbg(mhdp->dev, "bridge disable\n");

	cdns_mhdp_set_video_status(mhdp, 0);

	mhdp->link_up = false;

	if (mhdp->plugged)
		drm_dp_link_power_down(&mhdp->aux, &mhdp->link);

	cdns_mhdp_j721e_disable(mhdp);
}

static u32 get_training_interval_us(struct cdns_mhdp_device *mhdp,
				    u32 interval)
{
	if (interval == 0)
		return 400;
	if (interval < 5)
		return 4000 << (interval - 1);
	dev_err(mhdp->dev,
		"wrong training interval returned by DPCD: %d\n", interval);
	return 0;
}

static int cdns_mhdp_link_up(struct cdns_mhdp_device *mhdp)
{
	u32 resp, dp_framer_global_config, video_mode;
	u8 reg0[DP_RECEIVER_CAP_SIZE], amp[2];

	/*
	 * Upon power-on reset/device disconnection: [2:0] bits should be 0b001
	 * and [7:5] bits 0b000.
	 */
	drm_dp_dpcd_writeb(&mhdp->aux, DP_SET_POWER, 1);

	drm_dp_link_probe(&mhdp->aux, &mhdp->link);

	dev_dbg(mhdp->dev, "Set sink device power state via DPCD\n");
	drm_dp_link_power_up(&mhdp->aux, &mhdp->link);
	/* FIXME (CDNS): do we have to wait for 100ms before going on? */
	mdelay(100);

	mhdp->sink.link_rate = mhdp->link.rate;
	mhdp->sink.lanes_cnt = mhdp->link.num_lanes;
	mhdp->sink.enhanced = !!(mhdp->link.capabilities &
				 DP_LINK_CAP_ENHANCED_FRAMING);

	drm_dp_dpcd_read(&mhdp->aux, DP_DPCD_REV, reg0, DP_RECEIVER_CAP_SIZE);

	mhdp->sink.pattern_supp = CDNS_SUPPORT_TPS(1) | CDNS_SUPPORT_TPS(2);
	if (drm_dp_tps3_supported(reg0))
		mhdp->sink.pattern_supp |= CDNS_SUPPORT_TPS(3);
	if (drm_dp_tps4_supported(reg0))
		mhdp->sink.pattern_supp |= CDNS_SUPPORT_TPS(4);

	mhdp->sink.fast_link = !!(reg0[DP_MAX_DOWNSPREAD] &
				  DP_NO_AUX_HANDSHAKE_LINK_TRAINING);

	mhdp->link.rate = max_link_rate(mhdp->host, mhdp->sink);
	mhdp->link.num_lanes = min_t(u8, mhdp->sink.lanes_cnt,
				     mhdp->host.lanes_cnt & GENMASK(2, 0));
	cdns_mhdp_reg_read(mhdp, CDNS_DP_FRAMER_GLOBAL_CONFIG, &resp);

	dp_framer_global_config = be32_to_cpu(resp);

	video_mode = !(dp_framer_global_config & CDNS_DP_NO_VIDEO_MODE);

	if (dp_framer_global_config & CDNS_DP_FRAMER_EN)
		cdns_mhdp_reg_write(mhdp, CDNS_DP_FRAMER_GLOBAL_CONFIG,
				    dp_framer_global_config &
				    ~CDNS_DP_FRAMER_EN);

	/* Spread AMP if required, enable 8b/10b coding */
	amp[0] = (mhdp->host.lanes_cnt & CDNS_SSC) ? DP_SPREAD_AMP_0_5 : 0;
	amp[1] = DP_SET_ANSI_8B10B;
	drm_dp_dpcd_write(&mhdp->aux, DP_DOWNSPREAD_CTRL, amp, 2);

	if (mhdp->host.fast_link & mhdp->sink.fast_link) {
		/* FIXME: implement fastlink */
		dev_dbg(mhdp->dev, "fastlink\n");
	} else {
		const u32 interval = reg0[DP_TRAINING_AUX_RD_INTERVAL] &
				     DP_TRAINING_AUX_RD_MASK;
		const u32 interval_us = get_training_interval_us(mhdp,
								 interval);
		if (!interval_us ||
		    mhdp_link_training(mhdp, video_mode, interval_us)) {
			dev_err(mhdp->dev, "Link training failed. Exiting.\n");
			return -EIO;
		}
	}

	mhdp->link_up = true;

	return 0;
}

u32 cdns_mhdp_get_bpp(struct cdns_mhdp_display_fmt *fmt)
{
	u32 bpp;

	if (fmt->y_only)
		return fmt->bpc;

	switch (fmt->color_format) {
	case DRM_COLOR_FORMAT_RGB444:
	case DRM_COLOR_FORMAT_YCRCB444:
		bpp = fmt->bpc * 3;
		break;
	case DRM_COLOR_FORMAT_YCRCB422:
		bpp = fmt->bpc * 2;
		break;
	case DRM_COLOR_FORMAT_YCRCB420:
		bpp = fmt->bpc * 3 / 2;
		break;
	default:
		bpp = fmt->bpc * 3;
		WARN_ON(1);
	}
	return bpp;
}

static int cdns_mhdp_sst_enable(struct drm_bridge *bridge)
{
	struct cdns_mhdp_bridge *mhdp_bridge = to_mhdp_bridge(bridge);
	struct cdns_mhdp_device *mhdp = mhdp_bridge->mhdp;
	u32 rate, vs, vs_f, required_bandwidth, available_bandwidth;
	u32 tu_size = 30, line_thresh1, line_thresh2, line_thresh = 0;
	struct drm_display_mode *mode;
	int pxlclock;
	u32 bpp, bpc, pxlfmt;

	pxlfmt = mhdp->display_fmt.color_format;
	bpc = mhdp->display_fmt.bpc;

	mode = &bridge->encoder->crtc->state->mode;
	pxlclock = mode->crtc_clock;

	mhdp_bridge->stream_id = 0;

	rate = mhdp->link.rate / 1000;

	bpp = cdns_mhdp_get_bpp(&mhdp->display_fmt);

	/* find optimal tu_size */
	required_bandwidth = pxlclock * bpp / 8;
	available_bandwidth = mhdp->link.num_lanes * rate;
	do {
		tu_size += 2;

		vs_f = tu_size * required_bandwidth / available_bandwidth;
		vs = vs_f / 1000;
		vs_f = vs_f % 1000;
		/*
		 * FIXME (CDNS): downspreading?
		 * It's unused is what I've been told.
		 */
	} while ((vs == 1 || ((vs_f > 850 || vs_f < 100) && vs_f != 0) ||
		  tu_size - vs < 2) && tu_size < 64);

	if (vs > 64)
		return -EINVAL;

	cdns_mhdp_reg_write(mhdp, CDNS_DP_FRAMER_TU,
			    CDNS_DP_FRAMER_TU_VS(vs) |
			    CDNS_DP_FRAMER_TU_SIZE(tu_size) |
			    CDNS_DP_FRAMER_TU_CNT_RST_EN);

	line_thresh1 = ((vs + 1) << 5) * 8 / bpp;
	line_thresh2 = (pxlclock << 5) / 1000 / rate * (vs + 1) - (1 << 5);
	line_thresh = line_thresh1 - line_thresh2 / mhdp->link.num_lanes;
	line_thresh = (line_thresh >> 5) + 2;
	cdns_mhdp_reg_write(mhdp, CDNS_DP_LINE_THRESH(0),
			    line_thresh & GENMASK(5, 0));

	cdns_mhdp_reg_write(mhdp, CDNS_DP_STREAM_CONFIG_2(0),
			    CDNS_DP_SC2_TU_VS_DIFF((tu_size - vs > 3) ?
						   0 : tu_size - vs));

	cdns_mhdp_configure_video(bridge);

	cdns_mhdp_set_video_status(mhdp, 1);

	return 0;
}

void cdns_mhdp_configure_video(struct drm_bridge *bridge)
{
	struct cdns_mhdp_bridge *mhdp_bridge = to_mhdp_bridge(bridge);
	struct cdns_mhdp_device *mhdp = mhdp_bridge->mhdp;
	unsigned int dp_framer_sp = 0, msa_horizontal_1,
			   msa_vertical_1, bnd_hsync2vsync, hsync2vsync_pol_ctrl,
			   misc0 = 0, misc1 = 0, pxl_repr,
			   front_porch, back_porch, msa_h0, msa_v0, hsync, vsync,
			   dp_vertical_1;
	struct drm_display_mode *mode;
	u32 bpp, bpc, pxlfmt;
	u32 tmp;
	u8 stream_id = mhdp_bridge->stream_id;

	mode = &bridge->encoder->crtc->state->mode;

	pxlfmt = mhdp->display_fmt.color_format;
	bpc = mhdp->display_fmt.bpc;

	/* if YCBCR supported and stream not SD, use ITU709 */
	/* FIXME: handle ITU version with YCBCR420 when supported */
	if ((pxlfmt == DRM_COLOR_FORMAT_YCRCB444 ||
	     pxlfmt == DRM_COLOR_FORMAT_YCRCB422) && mode->crtc_vdisplay >= 720)
		misc0 = DP_YCBCR_COEFFICIENTS_ITU709;

	bpp = cdns_mhdp_get_bpp(&mhdp->display_fmt);

	switch (pxlfmt) {
	case DRM_COLOR_FORMAT_RGB444:
		pxl_repr = CDNS_DP_FRAMER_RGB << CDNS_DP_FRAMER_PXL_FORMAT;
		misc0 |= DP_COLOR_FORMAT_RGB;
		break;
	case DRM_COLOR_FORMAT_YCRCB444:
		pxl_repr = CDNS_DP_FRAMER_YCBCR444 << CDNS_DP_FRAMER_PXL_FORMAT;
		misc0 |= DP_COLOR_FORMAT_YCbCr444 | DP_TEST_DYNAMIC_RANGE_CEA;
		break;
	case DRM_COLOR_FORMAT_YCRCB422:
		pxl_repr = CDNS_DP_FRAMER_YCBCR422 << CDNS_DP_FRAMER_PXL_FORMAT;
		misc0 |= DP_COLOR_FORMAT_YCbCr422 | DP_TEST_DYNAMIC_RANGE_CEA;
		break;
	case DRM_COLOR_FORMAT_YCRCB420:
		pxl_repr = CDNS_DP_FRAMER_YCBCR420 << CDNS_DP_FRAMER_PXL_FORMAT;
		break;
	default:
		pxl_repr = CDNS_DP_FRAMER_Y_ONLY << CDNS_DP_FRAMER_PXL_FORMAT;
	}

	switch (bpc) {
	case 6:
		misc0 |= DP_TEST_BIT_DEPTH_6;
		pxl_repr |= CDNS_DP_FRAMER_6_BPC;
		break;
	case 8:
		misc0 |= DP_TEST_BIT_DEPTH_8;
		pxl_repr |= CDNS_DP_FRAMER_8_BPC;
		break;
	case 10:
		misc0 |= DP_TEST_BIT_DEPTH_10;
		pxl_repr |= CDNS_DP_FRAMER_10_BPC;
		break;
	case 12:
		misc0 |= DP_TEST_BIT_DEPTH_12;
		pxl_repr |= CDNS_DP_FRAMER_12_BPC;
		break;
	case 16:
		misc0 |= DP_TEST_BIT_DEPTH_16;
		pxl_repr |= CDNS_DP_FRAMER_16_BPC;
		break;
	}

	bnd_hsync2vsync = CDNS_IP_BYPASS_V_INTERFACE;
	if (mode->flags & DRM_MODE_FLAG_INTERLACE)
		bnd_hsync2vsync |= CDNS_IP_DET_INTERLACE_FORMAT;

	cdns_mhdp_reg_write(mhdp, CDNS_BND_HSYNC2VSYNC(stream_id),
			    bnd_hsync2vsync);

	if (mode->flags & DRM_MODE_FLAG_INTERLACE &&
	    mode->flags & DRM_MODE_FLAG_PHSYNC)
		hsync2vsync_pol_ctrl = CDNS_H2V_HSYNC_POL_ACTIVE_LOW |
				       CDNS_H2V_VSYNC_POL_ACTIVE_LOW;
	else
		hsync2vsync_pol_ctrl = 0;

	cdns_mhdp_reg_write(mhdp, CDNS_HSYNC2VSYNC_POL_CTRL(stream_id),
			    hsync2vsync_pol_ctrl);

	cdns_mhdp_reg_write(mhdp, CDNS_DP_FRAMER_PXL_REPR(stream_id), pxl_repr);

	if (mode->flags & DRM_MODE_FLAG_INTERLACE)
		dp_framer_sp |= CDNS_DP_FRAMER_INTERLACE;
	if (mode->flags & DRM_MODE_FLAG_NHSYNC)
		dp_framer_sp |= CDNS_DP_FRAMER_HSYNC_POL_LOW;
	if (mode->flags & DRM_MODE_FLAG_NVSYNC)
		dp_framer_sp |= CDNS_DP_FRAMER_VSYNC_POL_LOW;
	cdns_mhdp_reg_write(mhdp, CDNS_DP_FRAMER_SP(stream_id), dp_framer_sp);

	front_porch = mode->crtc_hsync_start - mode->crtc_hdisplay;
	back_porch = mode->crtc_htotal - mode->crtc_hsync_end;
	cdns_mhdp_reg_write(mhdp, CDNS_DP_FRONT_BACK_PORCH(stream_id),
			    CDNS_DP_FRONT_PORCH(front_porch) |
			    CDNS_DP_BACK_PORCH(back_porch));

	cdns_mhdp_reg_write(mhdp, CDNS_DP_BYTE_COUNT(stream_id),
			    mode->crtc_hdisplay * bpp / 8);

	msa_h0 = mode->crtc_htotal - mode->crtc_hsync_start;
	cdns_mhdp_reg_write(mhdp, CDNS_DP_MSA_HORIZONTAL_0(stream_id),
			    CDNS_DP_MSAH0_H_TOTAL(mode->crtc_htotal) |
			    CDNS_DP_MSAH0_HSYNC_START(msa_h0));

	hsync = mode->crtc_hsync_end - mode->crtc_hsync_start;
	msa_horizontal_1 = CDNS_DP_MSAH1_HSYNC_WIDTH(hsync) |
			   CDNS_DP_MSAH1_HDISP_WIDTH(mode->crtc_hdisplay);
	if (mode->flags & DRM_MODE_FLAG_NHSYNC)
		msa_horizontal_1 |= CDNS_DP_MSAH1_HSYNC_POL_LOW;
	cdns_mhdp_reg_write(mhdp, CDNS_DP_MSA_HORIZONTAL_1(stream_id),
			    msa_horizontal_1);

	msa_v0 = mode->crtc_vtotal - mode->crtc_vsync_start;
	cdns_mhdp_reg_write(mhdp, CDNS_DP_MSA_VERTICAL_0(stream_id),
			    CDNS_DP_MSAV0_V_TOTAL(mode->crtc_vtotal) |
			    CDNS_DP_MSAV0_VSYNC_START(msa_v0));

	vsync = mode->crtc_vsync_end - mode->crtc_vsync_start;
	msa_vertical_1 = CDNS_DP_MSAV1_VSYNC_WIDTH(vsync) |
			 CDNS_DP_MSAV1_VDISP_WIDTH(mode->crtc_vdisplay);
	if (mode->flags & DRM_MODE_FLAG_NVSYNC)
		msa_vertical_1 |= CDNS_DP_MSAV1_VSYNC_POL_LOW;
	cdns_mhdp_reg_write(mhdp, CDNS_DP_MSA_VERTICAL_1(stream_id),
			    msa_vertical_1);

	if ((mode->flags & DRM_MODE_FLAG_INTERLACE) &&
	    mode->crtc_vtotal % 2 == 0)
		misc1 = DP_TEST_INTERLACED;
	if (mhdp->display_fmt.y_only)
		misc1 |= CDNS_DP_TEST_COLOR_FORMAT_RAW_Y_ONLY;
	/* FIXME: use VSC SDP for Y420 */
	/* FIXME: (CDNS) no code for Y420 in bare metal test */
	if (pxlfmt == DRM_COLOR_FORMAT_YCRCB420)
		misc1 = CDNS_DP_TEST_VSC_SDP;

	cdns_mhdp_reg_write(mhdp, CDNS_DP_MSA_MISC(stream_id),
			    misc0 | (misc1 << 8));

	cdns_mhdp_reg_write(mhdp, CDNS_DP_HORIZONTAL(stream_id),
			    CDNS_DP_H_HSYNC_WIDTH(hsync) |
			    CDNS_DP_H_H_TOTAL(mode->crtc_hdisplay));

	cdns_mhdp_reg_write(mhdp, CDNS_DP_VERTICAL_0(stream_id),
			    CDNS_DP_V0_VHEIGHT(mode->crtc_vdisplay) |
			    CDNS_DP_V0_VSTART(msa_v0));

	dp_vertical_1 = CDNS_DP_V1_VTOTAL(mode->crtc_vtotal);
	if ((mode->flags & DRM_MODE_FLAG_INTERLACE) &&
	    mode->crtc_vtotal % 2 == 0)
		dp_vertical_1 |= CDNS_DP_V1_VTOTAL_EVEN;

	cdns_mhdp_reg_write(mhdp, CDNS_DP_VERTICAL_1(stream_id), dp_vertical_1);

	cdns_mhdp_reg_write_bit(mhdp, CDNS_DP_VB_ID(stream_id), 2, 1,
				(mode->flags & DRM_MODE_FLAG_INTERLACE) ?
				CDNS_DP_VB_ID_INTERLACED : 0);


	cdns_mhdp_reg_read(mhdp, CDNS_DP_FRAMER_GLOBAL_CONFIG, &tmp);
	tmp |= CDNS_DP_FRAMER_EN;
	cdns_mhdp_reg_write(mhdp, CDNS_DP_FRAMER_GLOBAL_CONFIG, tmp);
}

void cdns_mhdp_enable(struct drm_bridge *bridge)
{
	struct cdns_mhdp_bridge *mhdp_bridge = to_mhdp_bridge(bridge);
	struct cdns_mhdp_device *mhdp = mhdp_bridge->mhdp;

	dev_dbg(mhdp->dev, "bridge enable\n");

	cdns_mhdp_j721e_enable(mhdp);

	if (!mhdp->link_up)
		cdns_mhdp_link_up(mhdp);

	cdns_mhdp_sst_enable(bridge);
}

static void cdns_mhdp_detach(struct drm_bridge *bridge)
{
	struct cdns_mhdp_device *mhdp = bridge_to_mhdp(bridge);

	writel(~0, mhdp->regs + CDNS_APB_INT_MASK);
	writel(~0, mhdp->regs + CDNS_MB_INT_MASK);
}

static bool cdns_mhdp_mode_fixup(struct drm_bridge *bridge,
				 const struct drm_display_mode *mode,
				 struct drm_display_mode *adj)
{
	/* Fixup sync polarities, both hsync and vsync are active high */
	adj->flags = mode->flags;
	adj->flags |= (DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC);
	adj->flags &= ~(DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC);

	return true;
}

static const struct drm_bridge_funcs cdns_mhdp_bridge_funcs = {
	.enable = cdns_mhdp_enable,
	.disable = cdns_mhdp_disable,
	.attach = cdns_mhdp_attach,
	.detach = cdns_mhdp_detach,
	.mode_fixup = cdns_mhdp_mode_fixup,
};

static int load_firmware(struct cdns_mhdp_device *mhdp, const char *name,
			 unsigned int addr)
{
	const struct firmware *fw;
	int ret;

	ret = request_firmware(&fw, name, mhdp->dev);
	if (ret) {
		dev_err(mhdp->dev, "failed to load firmware (%s), ret: %d\n",
			name, ret);
		return ret;
	}

	memcpy_toio(mhdp->regs + addr, fw->data, fw->size);

	release_firmware(fw);

	return 0;
}

static int cdns_mhdp_audio_hw_params(struct device *dev, void *data,
				     struct hdmi_codec_daifmt *daifmt,
				     struct hdmi_codec_params *params)
{
	struct cdns_mhdp_device *mhdp = dev_get_drvdata(dev);
	struct audio_info audio = {
		.sample_width = params->sample_width,
		.sample_rate = params->sample_rate,
		.channels = params->channels,
	};
	int ret;

	if (daifmt->fmt != HDMI_I2S) {
		DRM_DEV_ERROR(dev, "Invalid format %d\n", daifmt->fmt);
		return -EINVAL;
	}

	audio.format = AFMT_I2S;

	ret = cdns_mhdp_audio_config(mhdp, &audio);
	if (!ret)
		mhdp->audio_info = audio;

	return 0;
}

static void cdns_mhdp_audio_shutdown(struct device *dev, void *data)
{
	struct cdns_mhdp_device *mhdp = dev_get_drvdata(dev);
	int ret;

	ret = cdns_mhdp_audio_stop(mhdp, &mhdp->audio_info);
	if (!ret)
		mhdp->audio_info.format = AFMT_UNUSED;
}

static int cdns_mhdp_audio_digital_mute(struct device *dev, void *data,
					bool enable)
{
	struct cdns_mhdp_device *mhdp = dev_get_drvdata(dev);

	return cdns_mhdp_audio_mute(mhdp, enable);
}

static int cdns_mhdp_audio_get_eld(struct device *dev, void *data,
				   u8 *buf, size_t len)
{
	struct cdns_mhdp_device *mhdp = dev_get_drvdata(dev);

	memcpy(buf, mhdp->connector.base.eld,
	       min(sizeof(mhdp->connector.base.eld), len));

	return 0;
}

static const struct hdmi_codec_ops audio_codec_ops = {
	.hw_params = cdns_mhdp_audio_hw_params,
	.audio_shutdown = cdns_mhdp_audio_shutdown,
	.digital_mute = cdns_mhdp_audio_digital_mute,
	.get_eld = cdns_mhdp_audio_get_eld,
};

static int mhdp_probe(struct platform_device *pdev)
{
	struct resource *regs;
	struct cdns_mhdp_device *mhdp;
	struct clk *clk;
	int ret;
	unsigned int reg;
	unsigned long rate;
	u32 resp;
	int irq;
	u32 lanes_prop;

	struct hdmi_codec_pdata codec_data = {
		.i2s = 1,
		.max_i2s_channels = 8,
		.ops = &audio_codec_ops,
	};

	mhdp = devm_kzalloc(&pdev->dev, sizeof(struct cdns_mhdp_device),
			    GFP_KERNEL);
	if (!mhdp)
		return -ENOMEM;

	clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "couldn't get clk: %ld\n", PTR_ERR(clk));
		return PTR_ERR(clk);
	}

	mhdp->clk = clk;
	mhdp->dev = &pdev->dev;
	dev_set_drvdata(&pdev->dev, mhdp);

	drm_dp_aux_init(&mhdp->aux);
	mhdp->aux.dev = &pdev->dev;
	mhdp->aux.transfer = mhdp_transfer;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mhdp->regs = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(mhdp->regs))
		return PTR_ERR(mhdp->regs);

	mhdp->phy = devm_phy_get(&pdev->dev, "dpphy");
	if (IS_ERR(mhdp->phy)) {
		dev_err(&pdev->dev, "no PHY configured\n");
		return PTR_ERR(mhdp->phy);
	}

	platform_set_drvdata(pdev, mhdp);

	clk_prepare_enable(clk);

	pm_runtime_enable(&pdev->dev);
	ret = pm_runtime_get_sync(&pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "pm_runtime_get_sync failed\n");
		return ret;
	}

	ret = cdns_mhdp_j721e_init(mhdp);
	if (ret != 0) {
		dev_err(&pdev->dev, "J721E Wrapper initialization failed: %d\n",
			ret);
		return ret;
	}

	/* Release uCPU reset and stall it. */
	writel(CDNS_CPU_STALL, mhdp->regs + CDNS_APB_CTRL);

	ret = load_firmware(mhdp, FW_NAME, CDNS_MHDP_IMEM);
	if (ret)
		return ret;

	rate = clk_get_rate(clk);
	writel(rate % 1000000, mhdp->regs + CDNS_SW_CLK_L);
	writel(rate / 1000000, mhdp->regs + CDNS_SW_CLK_H);

	dev_dbg(&pdev->dev, "func clk rate %lu Hz\n", rate);

	/* Leave debug mode, release stall */
	writel(0, mhdp->regs + CDNS_APB_CTRL);

	writel(~0, mhdp->regs + CDNS_MB_INT_MASK);
	writel(~0, mhdp->regs + CDNS_APB_INT_MASK);

	irq = platform_get_irq(pdev, 0);
	ret = devm_request_threaded_irq(mhdp->dev, irq, NULL, mhdp_irq_handler,
					IRQF_ONESHOT, "mhdp8546", mhdp);
	if (ret) {
		dev_err(&pdev->dev,
			"cannot install IRQ %d\n", irq);
		return -EIO;
	}

	/*
	 * Wait for the KEEP_ALIVE "message" on the first 8 bits.
	 * Updated each sched "tick" (~2ms)
	 */
	ret = readl_poll_timeout(mhdp->regs + CDNS_KEEP_ALIVE, reg,
				 reg & CDNS_KEEP_ALIVE_MASK, 500,
				 CDNS_KEEP_ALIVE_TIMEOUT);
	if (ret) {
		dev_err(&pdev->dev,
			"device didn't give any life sign: reg %d\n", reg);
		return -EIO;
	}

	/* Read source capabilities, based on PHY's device tree properties. */
	ret = device_property_read_u32(&(mhdp->phy->dev), "num_lanes",
				       &(lanes_prop));
	if (ret)
		mhdp->host.lanes_cnt = CDNS_LANE_4 | CDNS_SCRAMBLER;
	else
		mhdp->host.lanes_cnt = lanes_prop | CDNS_SCRAMBLER;

	ret = device_property_read_u32(&(mhdp->phy->dev), "max_bit_rate",
				       &(mhdp->host.link_rate));
	if (ret)
		mhdp->host.link_rate = drm_dp_bw_code_to_link_rate(DP_LINK_BW_8_1);
	else
		/* PHY uses Mb/s, DRM uses tens of kb/s. */
		mhdp->host.link_rate *= 100;

	mhdp->host.volt_swing = CDNS_VOLT_SWING(3);
	mhdp->host.pre_emphasis = CDNS_PRE_EMPHASIS(3);
	mhdp->host.pattern_supp = CDNS_SUPPORT_TPS(1) |
				  CDNS_SUPPORT_TPS(2) | CDNS_SUPPORT_TPS(3) |
				  CDNS_SUPPORT_TPS(4);
	mhdp->host.fast_link = 0;
	mhdp->host.lane_mapping = CDNS_LANE_MAPPING_NORMAL;
	mhdp->host.enhanced = true;

	/* The only currently supported format */
	mhdp->display_fmt.y_only = false;
	mhdp->display_fmt.color_format = DRM_COLOR_FORMAT_RGB444;
	mhdp->display_fmt.bpc = 8;

	mhdp->bridge.base.of_node = pdev->dev.of_node;
	mhdp->bridge.base.funcs = &cdns_mhdp_bridge_funcs;

	/* Init events to 0 as it's not cleared by FW at boot but on read */
	readl(mhdp->regs + CDNS_SW_EVENT0);
	readl(mhdp->regs + CDNS_SW_EVENT1);
	readl(mhdp->regs + CDNS_SW_EVENT2);
	readl(mhdp->regs + CDNS_SW_EVENT3);

	/* Activate uCPU */
	ret = cdns_mhdp_set_firmware_active(mhdp, true);
	if (ret) {
		dev_err(mhdp->dev, "Failed to activate DP\n");
		return ret;
	}

	mhdp->audio_pdev = platform_device_register_data(
				   mhdp->dev, HDMI_CODEC_DRV_NAME, PLATFORM_DEVID_AUTO,
				   &codec_data, sizeof(codec_data));

	ret = phy_init(mhdp->phy);
	if (ret) {
		dev_err(mhdp->dev, "Failed to initialize PHY: %d\n", ret);
		return ret;
	}

	/* Enable VIF clock for stream 0 */
	cdns_mhdp_reg_read(mhdp, CDNS_DPTX_CAR, &resp);
	cdns_mhdp_reg_write(mhdp, CDNS_DPTX_CAR,
			    resp | CDNS_VIF_CLK_EN | CDNS_VIF_CLK_RSTN);

	mhdp->bridge.connector = &mhdp->connector;
	mhdp->connector.bridge = &mhdp->bridge;
	mhdp->bridge.mhdp = mhdp;
	mhdp->bridge.is_active = false;

	drm_bridge_add(&mhdp->bridge.base);

	return 0;
}

MODULE_FIRMWARE(FW_NAME);

static int mhdp_remove(struct platform_device *pdev)
{
	struct cdns_mhdp_device *mhdp = dev_get_drvdata(&pdev->dev);
	int ret;

	platform_device_unregister(mhdp->audio_pdev);

	drm_bridge_remove(&mhdp->bridge.base);

	ret = cdns_mhdp_set_firmware_active(mhdp, false);
	if (ret) {
		dev_err(mhdp->dev, "Failed to de-activate DP\n");
		return ret;
	}

	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	clk_disable_unprepare(mhdp->clk);

	/* FIXME: check for missing functions */

	return 0;
}

static struct platform_driver mhdp_driver = {
	.driver	= {
		.name		= "cdns-mhdp",
		.of_match_table	= of_match_ptr(mhdp_ids),
	},
	.probe	= mhdp_probe,
	.remove	= mhdp_remove,
};
module_platform_driver(mhdp_driver);

MODULE_AUTHOR("Quentin Schulz <quentin.schulz@free-electrons.com>");
MODULE_AUTHOR("Przemyslaw Gaj <pgaj@cadence.com>");
MODULE_AUTHOR("Damian Kos <dkos@cadence.com>");
MODULE_AUTHOR("Piotr Sroka <piotrs@cadence.com>");
MODULE_DESCRIPTION("Cadence MHDP DP bridge driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:cdns-mhdp");
