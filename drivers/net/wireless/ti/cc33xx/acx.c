// SPDX-License-Identifier: GPL-2.0-only
/*
 * This file is part of wl1271
 *
 * Copyright (C) 2008-2009 Nokia Corporation
 *
 * Contact: Luciano Coelho <luciano.coelho@nokia.com>
 */

#include "acx.h"

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>

#include "wlcore.h"
#include "tx.h"
#include "debug.h"
#include "wl12xx_80211.h"


int wl18xx_acx_dynamic_fw_traces(struct wl1271 *wl)
{
	struct acx_dynamic_fw_traces_cfg *acx;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx dynamic fw traces config %d",
		     wl->dynamic_fw_traces);

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->dynamic_fw_traces = cpu_to_le32(wl->dynamic_fw_traces);

	ret = cc33xx_cmd_configure(wl, ACX_DYNAMIC_TRACES_CFG,
				   acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("acx config dynamic fw traces failed: %d", ret);
		goto out;
	}
out:
	kfree(acx);
	return ret;
}

int wl18xx_acx_clear_statistics(struct wl1271 *wl)
{
	struct wl18xx_acx_clear_statistics *acx;
	int ret = 0;

	cc33xx_debug(DEBUG_ACX, "acx clear statistics");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	ret = cc33xx_cmd_configure(wl, ACX_CLEAR_STATISTICS, acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("failed to clear firmware statistics: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}


int cc33xx_acx_wake_up_conditions(struct wl1271 *wl, struct wl12xx_vif *wlvif,
				  u8 wake_up_event, u8 listen_interval)
{
	struct acx_wake_up_condition *wake_up;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx wake up conditions (wake_up_event %d listen_interval %d)",
		     wake_up_event, listen_interval);

	wake_up = kzalloc(sizeof(*wake_up), GFP_KERNEL);
	if (!wake_up) {
		ret = -ENOMEM;
		goto out;
	}

	wake_up->role_id = wlvif->role_id;
	wake_up->wake_up_event = wake_up_event;
	wake_up->listen_interval = listen_interval;

	ret = cc33xx_cmd_configure(wl, ACX_WAKE_UP_CONDITIONS,
				   wake_up, sizeof(*wake_up));
	if (ret < 0) {
		cc33xx_warning("could not set wake up conditions: %d", ret);
		goto out;
	}

out:
	kfree(wake_up);
	return ret;
}

int cc33xx_acx_sleep_auth(struct wl1271 *wl, u8 sleep_auth)
{
	struct acx_sleep_auth *auth;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx sleep auth %d", sleep_auth);

	auth = kzalloc(sizeof(*auth), GFP_KERNEL);
	if (!auth) {
		ret = -ENOMEM;
		goto out;
	}

	auth->sleep_auth = sleep_auth;

	ret = cc33xx_cmd_configure(wl, ACX_SLEEP_AUTH, auth, sizeof(*auth));
	if (ret < 0) {
		cc33xx_error("could not configure sleep_auth to %d: %d",
			     sleep_auth, ret);
		goto out;
	}

	wl->sleep_auth = sleep_auth;
out:
	kfree(auth);
	return ret;
}

int cc33xx_ble_enable(struct wl1271 *wl, u8 ble_enable)
{
	struct debug_header *buf;
	int ret;

	cc33xx_debug(DEBUG_ACX, "ble enable");

	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf) {
		ret = -ENOMEM;
		goto out;
	}

	//wl1271_cmd_debug

	ret = wl1271_cmd_debug(wl,BLE_ENABLE ,buf ,sizeof(*buf));
	if (ret < 0) {
		cc33xx_error("could not enable ble");
		goto out;
	}

	wl->ble_enable = 1;
out:
	kfree(buf);
	return ret;
}



int cc33xx_acx_tx_power(struct wl1271 *wl, struct wl12xx_vif *wlvif,
			int power)
{
	struct acx_current_tx_power *acx;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx dot11_cur_tx_pwr %d", power);

	if (power < 0 || power > 25)
		return -EINVAL;

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->role_id = wlvif->role_id;
	acx->current_tx_power = power * 10;

	ret = cc33xx_cmd_configure(wl, DOT11_CUR_TX_PWR, acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("configure of tx power failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int cc33xx_acx_feature_cfg(struct wl1271 *wl, struct wl12xx_vif *wlvif)
{
	struct acx_feature_config *feature;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx feature cfg");

	feature = kzalloc(sizeof(*feature), GFP_KERNEL);
	if (!feature) {
		ret = -ENOMEM;
		goto out;
	}

	/* DF_ENCRYPTION_DISABLE and DF_SNIFF_MODE_ENABLE are disabled */
	feature->role_id = wlvif->role_id;
	feature->data_flow_options = 0;
	feature->options = 0;

	ret = cc33xx_cmd_configure(wl, ACX_FEATURE_CFG,
				   feature, sizeof(*feature));
	if (ret < 0) {
		cc33xx_error("Couldn't set HW encryption");
		goto out;
	}

out:
	kfree(feature);
	return ret;
}

int wl1271_acx_mem_map(struct wl1271 *wl, struct acx_header *mem_map,
		       size_t len)
{
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx mem map");

	ret = wl1271_cmd_interrogate(wl, MEM_MAP_INTR, mem_map,
				     sizeof(struct acx_header), len);
	if (ret < 0)
		return ret;

	return 0;
}

int wl1271_acx_rx_msdu_life_time(struct wl1271 *wl)
{
	struct acx_rx_msdu_lifetime *acx;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx rx msdu life time");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->lifetime = cpu_to_le32(wl->conf.rx.rx_msdu_life_time);
	ret = cc33xx_cmd_configure(wl, DOT11_RX_MSDU_LIFE_TIME,
				   acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("failed to set rx msdu life time: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int wl1271_acx_slot(struct wl1271 *wl, struct wl12xx_vif *wlvif,
		    enum acx_slot_type slot_time)
{
	struct acx_slot *slot;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx slot");

	slot = kzalloc(sizeof(*slot), GFP_KERNEL);
	if (!slot) {
		ret = -ENOMEM;
		goto out;
	}

	slot->role_id = wlvif->role_id;
	slot->slot_time = slot_time;
	ret = cc33xx_cmd_configure(wl, SLOT_CFG, slot, sizeof(*slot));

	if (ret < 0) {
		cc33xx_warning("failed to set slot time: %d", ret);
		goto out;
	}

out:
	kfree(slot);
	return ret;
}

int cc33xx_acx_group_address_tbl(struct wl1271 *wl, struct wl12xx_vif *wlvif,
				 bool enable, void *mc_list, u32 mc_list_len)
{
	cc33xx_debug(DEBUG_ACX, "acx group address tbl not supported yet");
	return 0;
}

int wl1271_acx_service_period_timeout(struct wl1271 *wl,
				      struct wl12xx_vif *wlvif)
{
	struct acx_rx_timeout *rx_timeout;
	int ret;

	rx_timeout = kzalloc(sizeof(*rx_timeout), GFP_KERNEL);
	if (!rx_timeout) {
		ret = -ENOMEM;
		goto out;
	}

	cc33xx_debug(DEBUG_ACX, "acx service period timeout");

	rx_timeout->role_id = wlvif->role_id;
	rx_timeout->ps_poll_timeout = cpu_to_le16(wl->conf.rx.ps_poll_timeout);
	rx_timeout->upsd_timeout = cpu_to_le16(wl->conf.rx.upsd_timeout);

	ret = cc33xx_cmd_configure(wl, ACX_SERVICE_PERIOD_TIMEOUT,
				   rx_timeout, sizeof(*rx_timeout));
	if (ret < 0) {
		cc33xx_warning("failed to set service period timeout: %d",
			       ret);
		goto out;
	}

out:
	kfree(rx_timeout);
	return ret;
}

int cc33xx_acx_rts_threshold(struct wl1271 *wl, struct wl12xx_vif *wlvif,
			     u32 rts_threshold)
{
	struct acx_rts_threshold *rts;
	int ret;

	/*
	 * If the RTS threshold is not configured or out of range, use the
	 * default value.
	 */
	if (rts_threshold > IEEE80211_MAX_RTS_THRESHOLD)
		rts_threshold = wl->conf.rx.rts_threshold;

	cc33xx_debug(DEBUG_ACX, "acx rts threshold: %d", rts_threshold);

	rts = kzalloc(sizeof(*rts), GFP_KERNEL);
	if (!rts) {
		ret = -ENOMEM;
		goto out;
	}

	rts->role_id = wlvif->role_id;
	rts->threshold = cpu_to_le16((u16)rts_threshold);

	ret = cc33xx_cmd_configure(wl, DOT11_RTS_THRESHOLD, rts, sizeof(*rts));
	if (ret < 0) {
		cc33xx_warning("failed to set rts threshold: %d", ret);
		goto out;
	}

out:
	kfree(rts);
	return ret;
}

int wl1271_acx_dco_itrim_params(struct wl1271 *wl)
{
	struct acx_dco_itrim_params *dco;
	struct conf_itrim_settings *c = &wl->conf.itrim;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx dco itrim parameters");

	dco = kzalloc(sizeof(*dco), GFP_KERNEL);
	if (!dco) {
		ret = -ENOMEM;
		goto out;
	}

	dco->enable = c->enable;
	dco->timeout = cpu_to_le32(c->timeout);

	ret = cc33xx_cmd_configure(wl, ACX_SET_DCO_ITRIM_PARAMS,
				   dco, sizeof(*dco));
	if (ret < 0) {
		cc33xx_warning("failed to set dco itrim parameters: %d", ret);
		goto out;
	}

out:
	kfree(dco);
	return ret;
}

int cc33xx_acx_beacon_filter_opt(struct wl1271 *wl, struct wl12xx_vif *wlvif,
				 bool enable_filter)
{
	struct acx_beacon_filter_option *beacon_filter = NULL;
	int ret = 0;

	cc33xx_debug(DEBUG_ACX, "acx beacon filter opt enable=%d",
		     enable_filter);

	if (enable_filter &&
	    wl->conf.conn.bcn_filt_mode == CONF_BCN_FILT_MODE_DISABLED)
		goto out;

	beacon_filter = kzalloc(sizeof(*beacon_filter), GFP_KERNEL);
	if (!beacon_filter) {
		ret = -ENOMEM;
		goto out;
	}

	beacon_filter->role_id = wlvif->role_id;
	beacon_filter->enable = enable_filter;

	/*
	 * When set to zero, and the filter is enabled, beacons
	 * without the unicast TIM bit set are dropped.
	 */
	beacon_filter->max_num_beacons = 0;

	ret = cc33xx_cmd_configure(wl, ACX_BEACON_FILTER_OPT,
				   beacon_filter, sizeof(*beacon_filter));
	if (ret < 0) {
		cc33xx_warning("failed to set beacon filter opt: %d", ret);
		goto out;
	}

out:
	kfree(beacon_filter);
	return ret;
}

int wl1271_acx_beacon_filter_table(struct wl1271 *wl,
				   struct wl12xx_vif *wlvif)
{
	struct acx_beacon_filter_ie_table *ie_table;
	int i, idx = 0;
	int ret;
	bool vendor_spec = false;

	cc33xx_debug(DEBUG_ACX, "acx beacon filter table");

	ie_table = kzalloc(sizeof(*ie_table), GFP_KERNEL);
	if (!ie_table) {
		ret = -ENOMEM;
		goto out;
	}

	/* configure default beacon pass-through rules */
	ie_table->role_id = wlvif->role_id;
	ie_table->num_ie = 0;
	for (i = 0; i < wl->conf.conn.bcn_filt_ie_count; i++) {
		struct conf_bcn_filt_rule *r = &(wl->conf.conn.bcn_filt_ie[i]);
		ie_table->table[idx++] = r->ie;
		ie_table->table[idx++] = r->rule;

		if (r->ie == WLAN_EID_VENDOR_SPECIFIC) {
			/* only one vendor specific ie allowed */
			if (vendor_spec)
				continue;

			/* for vendor specific rules configure the
			   additional fields */
			memcpy(&(ie_table->table[idx]), r->oui,
			       CONF_BCN_IE_OUI_LEN);
			idx += CONF_BCN_IE_OUI_LEN;
			ie_table->table[idx++] = r->type;
			memcpy(&(ie_table->table[idx]), r->version,
			       CONF_BCN_IE_VER_LEN);
			idx += CONF_BCN_IE_VER_LEN;
			vendor_spec = true;
		}

		ie_table->num_ie++;
	}

	ret = cc33xx_cmd_configure(wl, ACX_BEACON_FILTER_TABLE,
				   ie_table, sizeof(*ie_table));
	if (ret < 0) {
		cc33xx_warning("failed to set beacon filter table: %d", ret);
		goto out;
	}

out:
	kfree(ie_table);
	return ret;
}

#define ACX_CONN_MONIT_DISABLE_VALUE  0xffffffff

int wl1271_acx_conn_monit_params(struct wl1271 *wl, struct wl12xx_vif *wlvif,
				 bool enable)
{
	struct acx_conn_monit_params *acx;
	u32 threshold = ACX_CONN_MONIT_DISABLE_VALUE;
	u32 timeout = ACX_CONN_MONIT_DISABLE_VALUE;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx connection monitor parameters: %s",
		     enable ? "enabled" : "disabled");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	if (enable) {
		threshold = wl->conf.conn.synch_fail_thold;
		timeout = wl->conf.conn.bss_lose_timeout;
	}

	acx->role_id = wlvif->role_id;
	acx->synch_fail_thold = cpu_to_le32(threshold);
	acx->bss_lose_timeout = cpu_to_le32(timeout);

	ret = cc33xx_cmd_configure(wl, ACX_CONN_MONIT_PARAMS,
				   acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("failed to set connection monitor "
			       "parameters: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}


int wl1271_acx_sg_enable(struct wl1271 *wl, bool enable)
{
	struct acx_bt_wlan_coex *pta;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx sg enable");

	pta = kzalloc(sizeof(*pta), GFP_KERNEL);
	if (!pta) {
		ret = -ENOMEM;
		goto out;
	}

	if (enable)
		pta->enable = wl->conf.sg.state;
	else
		pta->enable = CONF_SG_DISABLE;

	ret = cc33xx_cmd_configure(wl, ACX_SG_ENABLE, pta, sizeof(*pta));
	if (ret < 0) {
		cc33xx_warning("failed to set softgemini enable: %d", ret);
		goto out;
	}

out:
	kfree(pta);
	return ret;
}

int wl12xx_acx_sg_cfg(struct wl1271 *wl)
{
	struct acx_bt_wlan_coex_param *param;
	struct conf_sg_settings *c = &wl->conf.sg;
	int i, ret;

	cc33xx_debug(DEBUG_ACX, "acx sg cfg");

	param = kzalloc(sizeof(*param), GFP_KERNEL);
	if (!param) {
		ret = -ENOMEM;
		goto out;
	}

	/* BT-WLAN coext parameters */
	for (i = 0; i < WLCORE_CONF_SG_PARAMS_MAX; i++)
		param->params[i] = cpu_to_le32(c->params[i]);
	param->param_idx = WLCORE_CONF_SG_PARAMS_ALL;

	ret = cc33xx_cmd_configure(wl, ACX_SG_CFG, param, sizeof(*param));
	if (ret < 0) {
		cc33xx_warning("failed to set sg config: %d", ret);
		goto out;
	}

out:
	kfree(param);
	return ret;
}

int wl1271_acx_cca_threshold(struct wl1271 *wl)
{
	struct acx_energy_detection *detection;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx cca threshold");

	detection = kzalloc(sizeof(*detection), GFP_KERNEL);
	if (!detection) {
		ret = -ENOMEM;
		goto out;
	}

	detection->rx_cca_threshold = cpu_to_le16(wl->conf.rx.rx_cca_threshold);
	detection->tx_energy_detection = wl->conf.tx.tx_energy_detection;

	ret = cc33xx_cmd_configure(wl, ACX_CCA_THRESHOLD,
				   detection, sizeof(*detection));
	if (ret < 0)
		cc33xx_warning("failed to set cca threshold: %d", ret);

out:
	kfree(detection);
	return ret;
}

int wl1271_acx_bcn_dtim_options(struct wl1271 *wl, struct wl12xx_vif *wlvif)
{
	struct acx_beacon_broadcast *bb;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx bcn dtim options");

	bb = kzalloc(sizeof(*bb), GFP_KERNEL);
	if (!bb) {
		ret = -ENOMEM;
		goto out;
	}

	bb->role_id = wlvif->role_id;
	bb->beacon_rx_timeout = cpu_to_le16(wl->conf.conn.beacon_rx_timeout);
	bb->broadcast_timeout = cpu_to_le16(wl->conf.conn.broadcast_timeout);
	bb->rx_broadcast_in_ps = wl->conf.conn.rx_broadcast_in_ps;
	bb->ps_poll_threshold = wl->conf.conn.ps_poll_threshold;

	ret = cc33xx_cmd_configure(wl, ACX_BCN_DTIM_OPTIONS, bb, sizeof(*bb));
	if (ret < 0) {
		cc33xx_warning("failed to set rx config: %d", ret);
		goto out;
	}

out:
	kfree(bb);
	return ret;
}
 
int cc33xx_assoc_info_cfg(struct wl1271 *wl, struct wl12xx_vif *wlvif, struct ieee80211_sta *sta,u16 aid)
{
    struct assoc_info_cfg *cfg;
    int ret;

    cc33xx_debug(DEBUG_ACX, "acx aid");

    cfg = kzalloc(sizeof(*cfg), GFP_KERNEL);
    if (!cfg) {
        ret = -ENOMEM;
        goto out;
    }

    cfg->role_id = wlvif->role_id;
    cfg->aid = cpu_to_le16(aid);
    cfg->wmm_enabled = wlvif->wmm_enabled;

    cfg->nontransmitted = wlvif->nontransmitted;
    cfg->bssid_index = wlvif->bssid_index;
    cfg->bssid_indicator = wlvif->bssid_indicator;
	cfg->ht_supported = sta->ht_cap.ht_supported;
	cfg->vht_supported = sta->vht_cap.vht_supported;
	cfg->has_he = sta->he_cap.has_he;
    memcpy(cfg->transmitter_bssid,
		wlvif->transmitter_bssid, 
		ETH_ALEN);
    ret = cc33xx_cmd_configure(wl, ASSOC_INFO_CFG, cfg, sizeof(*cfg));
    if (ret < 0) {
        cc33xx_warning("failed to set aid: %d", ret);
        goto out;
    }

out:
    kfree(cfg);
    return ret;
}

int wl1271_acx_aid(struct wl1271 *wl, struct wl12xx_vif *wlvif, u16 aid)
{
	struct acx_aid *acx_aid;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx aid");

	acx_aid = kzalloc(sizeof(*acx_aid), GFP_KERNEL);
	if (!acx_aid) {
		ret = -ENOMEM;
		goto out;
	}

	acx_aid->role_id = wlvif->role_id;
	acx_aid->aid = cpu_to_le16(aid);

	ret = cc33xx_cmd_configure(wl, ACX_AID, acx_aid, sizeof(*acx_aid));
	if (ret < 0) {
		cc33xx_warning("failed to set aid: %d", ret);
		goto out;
	}

out:
	kfree(acx_aid);
	return ret;
}

int wl1271_acx_event_mbox_mask(struct wl1271 *wl, u32 event_mask)
{
	struct acx_event_mask *mask;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx event mbox mask");

	mask = kzalloc(sizeof(*mask), GFP_KERNEL);
	if (!mask) {
		ret = -ENOMEM;
		goto out;
	}

	/* high event mask is unused */
	mask->high_event_mask = cpu_to_le32(0xffffffff);
	mask->event_mask = cpu_to_le32(event_mask);

	ret = cc33xx_cmd_configure(wl, ACX_EVENT_MBOX_MASK,
				   mask, sizeof(*mask));
	if (ret < 0) {
		cc33xx_warning("failed to set acx_event_mbox_mask: %d", ret);
		goto out;
	}

out:
	kfree(mask);
	return ret;
}

int wl1271_acx_set_preamble(struct wl1271 *wl, struct wl12xx_vif *wlvif,
			    enum acx_preamble_type preamble)
{
	struct acx_preamble *acx;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx_set_preamble");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->role_id = wlvif->role_id;
	acx->preamble = preamble;

	ret = cc33xx_cmd_configure(wl, PREAMBLE_TYPE_CFG, acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("Setting of preamble failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int wl1271_acx_cts_protect(struct wl1271 *wl, struct wl12xx_vif *wlvif,
			   enum acx_ctsprotect_type ctsprotect)
{
	struct acx_ctsprotect *acx;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx_set_ctsprotect");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->role_id = wlvif->role_id;
	acx->ctsprotect = ctsprotect;

	ret = cc33xx_cmd_configure(wl, CTS_PROTECTION_CFG, acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("Setting of ctsprotect failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int wl1271_acx_statistics(struct wl1271 *wl, void *stats)
{
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx statistics");

	ret = wl1271_cmd_interrogate(wl, ACX_STATISTICS, stats,
				     sizeof(struct acx_header),
				     wl->stats.fw_stats_len);
	if (ret < 0) {
		cc33xx_warning("acx statistics failed: %d", ret);
		return -ENOMEM;
	}

	return 0;
}

int cc33xx_update_ap_rates(struct wl1271 *wl,u8 role_id,u32 basic_rates_set,u32 supported_rates){

	struct ap_rates_class_cfg *cfg;
	int ret;
	cc33xx_debug(DEBUG_AP, "Attempting to Update Basic Rates and Supported Rates");

	cfg = kzalloc(sizeof(*cfg), GFP_KERNEL);

    if (!cfg) {
        ret = -ENOMEM;
        goto out;
    }

	cfg->basic_rates_set = cpu_to_le32(basic_rates_set);
	cfg->supported_rates = cpu_to_le32(supported_rates);
	cfg->role_id = role_id;
	ret = cc33xx_cmd_configure(wl, AP_RATES_CFG, cfg, sizeof(*cfg));
	if (ret < 0) {
		cc33xx_warning("Updating AP Rates  failed: %d", ret);
		goto out;
	}

out:
    kfree(cfg);
    return ret;
}

int cc33xx_tx_param_cfg(struct wl1271 *wl, struct wl12xx_vif *wlvif,
              u8 ac, u8 cw_min, u16 cw_max, u8 aifsn, u16 txop, bool acm,
              u8 ps_schem, u8 is_mu_edca, u8 mu_edca_aifs, u8 mu_edca_ecw_min_max,
              u8 mu_edca_timer)
{
    struct tx_param_cfg *cfg;
    int ret = 0;

    cc33xx_debug(DEBUG_ACX, "tx param cfg %d cw_ming %d cw_max %d "
             "aifs %d txop %d", ac, cw_min, cw_max, aifsn, txop);

    cc33xx_debug(DEBUG_ACX, "tx param cfg ps_schem %d is_mu_edca %d mu_edca_aifs %d "
                 "mu_edca_ecw_min_max %d mu_edca_timer %d",
                 ps_schem, is_mu_edca, mu_edca_aifs, mu_edca_ecw_min_max, mu_edca_timer);

    cfg = kzalloc(sizeof(*cfg), GFP_KERNEL);

    if (!cfg) {
        ret = -ENOMEM;
        goto out;
    }

    cfg->role_id = wlvif->role_id;
    cfg->ac = ac;
    cfg->cw_min = cw_min;
    cfg->cw_max = cpu_to_le16(cw_max);
    cfg->aifsn = aifsn;
    cfg->tx_op_limit = cpu_to_le16(txop);
    cfg->acm = cpu_to_le16(acm);

    cfg->ps_scheme = ps_schem;
    cfg->is_mu_edca = is_mu_edca;
    cfg->mu_edca_aifs = mu_edca_aifs;
    cfg->mu_edca_ecw_min_max = mu_edca_ecw_min_max;
    cfg->mu_edca_timer = mu_edca_timer;

    ret = cc33xx_cmd_configure(wl, TX_PARAMS_CFG, cfg, sizeof(*cfg));
    if (ret < 0) {
        cc33xx_warning("tx param cfg failed: %d", ret);
        goto out;
    }

out:
    kfree(cfg);
    return ret;
}

int wl1271_acx_ac_cfg(struct wl1271 *wl, struct wl12xx_vif *wlvif,
		      u8 ac, u8 cw_min, u16 cw_max, u8 aifsn, u16 txop)
{
	struct acx_ac_cfg *acx;
	int ret = 0;

	cc33xx_debug(DEBUG_ACX, "acx ac cfg %d cw_ming %d cw_max %d "
		     "aifs %d txop %d", ac, cw_min, cw_max, aifsn, txop);

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);

	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->role_id = wlvif->role_id;
	acx->ac = ac;
	acx->cw_min = cw_min;
	acx->cw_max = cpu_to_le16(cw_max);
	acx->aifsn = aifsn;
	acx->tx_op_limit = cpu_to_le16(txop);

	ret = cc33xx_cmd_configure(wl, ACX_AC_CFG, acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("acx ac cfg failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int wl1271_acx_tid_cfg(struct wl1271 *wl, struct wl12xx_vif *wlvif,
		       u8 queue_id, u8 channel_type,
		       u8 tsid, u8 ps_scheme, u8 ack_policy,
		       u32 apsd_conf0, u32 apsd_conf1)
{
	struct acx_tid_config *acx;
	int ret = 0;

	cc33xx_debug(DEBUG_ACX, "acx tid config");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);

	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->role_id = wlvif->role_id;
	acx->queue_id = queue_id;
	acx->channel_type = channel_type;
	acx->tsid = tsid;
	acx->ps_scheme = ps_scheme;
	acx->ack_policy = ack_policy;
	acx->apsd_conf[0] = cpu_to_le32(apsd_conf0);
	acx->apsd_conf[1] = cpu_to_le32(apsd_conf1);

	ret = cc33xx_cmd_configure(wl, ACX_TID_CFG, acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("Setting of tid config failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int cc33xx_acx_frag_threshold(struct wl1271 *wl, u32 frag_threshold)
{
	struct acx_frag_threshold *acx;
	int ret = 0;

	/*
	 * If the fragmentation is not configured or out of range, use the
	 * default value.
	 */
	if (frag_threshold > IEEE80211_MAX_FRAG_THRESHOLD)
		frag_threshold = wl->conf.tx.frag_threshold;

	cc33xx_debug(DEBUG_ACX, "acx frag threshold: %d", frag_threshold);

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);

	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->frag_threshold = cpu_to_le16((u16)frag_threshold);
	ret = cc33xx_cmd_configure(wl, ACX_FRAG_CFG, acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("Setting of frag threshold failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int wl1271_acx_tx_config_options(struct wl1271 *wl)
{
	struct acx_tx_config_options *acx;
	int ret = 0;

	cc33xx_debug(DEBUG_ACX, "acx tx config options");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);

	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->tx_compl_timeout = cpu_to_le16(wl->conf.tx.tx_compl_timeout);
	acx->tx_compl_threshold = cpu_to_le16(wl->conf.tx.tx_compl_threshold);
	ret = cc33xx_cmd_configure(wl, ACX_TX_CONFIG_OPT, acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("Setting of tx options failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int wl12xx_acx_mem_cfg(struct wl1271 *wl)
{
	struct wl12xx_acx_config_memory *mem_conf;
	struct conf_memory_settings *mem;
	int ret;

	cc33xx_debug(DEBUG_ACX, "wl1271 mem cfg");

	mem_conf = kzalloc(sizeof(*mem_conf), GFP_KERNEL);
	if (!mem_conf) {
		ret = -ENOMEM;
		goto out;
	}

	mem = &wl->conf.mem;

	/* memory config */
	mem_conf->num_stations = mem->num_stations;
	mem_conf->rx_mem_block_num = mem->rx_block_num;
	mem_conf->tx_min_mem_block_num = mem->tx_min_block_num;
	mem_conf->num_ssid_profiles = mem->ssid_profiles;
	mem_conf->total_tx_descriptors = cpu_to_le32(wl->num_tx_desc);
	mem_conf->dyn_mem_enable = mem->dynamic_memory;
	mem_conf->tx_free_req = mem->min_req_tx_blocks;
	mem_conf->rx_free_req = mem->min_req_rx_blocks;
	mem_conf->tx_min = mem->tx_min;
	mem_conf->fwlog_blocks = wl->conf.fwlog.mem_blocks;

	ret = cc33xx_cmd_configure(wl, ACX_MEM_CFG, mem_conf,
				   sizeof(*mem_conf));
	if (ret < 0) {
		cc33xx_warning("wl1271 mem config failed: %d", ret);
		goto out;
	}

out:
	kfree(mem_conf);
	return ret;
}

int wl1271_acx_init_mem_config(struct wl1271 *wl)
{
	int ret;

	wl->target_mem_map = kzalloc(sizeof(struct wl1271_acx_mem_map),
				     GFP_KERNEL);
	if (!wl->target_mem_map) {
		cc33xx_error("couldn't allocate target memory map");
		return -ENOMEM;
	}

	/* we now ask for the firmware built memory map */
	ret = wl1271_acx_mem_map(wl, (void *)wl->target_mem_map,
				 sizeof(struct wl1271_acx_mem_map));
	if (ret < 0) {
		cc33xx_error("couldn't retrieve firmware memory map");
		kfree(wl->target_mem_map);
		wl->target_mem_map = NULL;
		return ret;
	}

	/* initialize TX block book keeping */
	wl->tx_blocks_available =
		le32_to_cpu(wl->target_mem_map->num_tx_mem_blocks);
	cc33xx_debug(DEBUG_TX, "available tx blocks: %d",
		     wl->tx_blocks_available);

	cc33xx_debug(DEBUG_TX, "available tx descriptor: %d available rx blocks %d",
		     wl->target_mem_map->num_tx_descriptor,
		     wl->target_mem_map->num_rx_mem_blocks);

	return 0;
}

int wl1271_acx_init_rx_interrupt(struct wl1271 *wl)
{
	struct wl1271_acx_rx_config_opt *rx_conf;
	int ret;

	cc33xx_debug(DEBUG_ACX, "wl1271 rx interrupt config");

	rx_conf = kzalloc(sizeof(*rx_conf), GFP_KERNEL);
	if (!rx_conf) {
		ret = -ENOMEM;
		goto out;
	}

	rx_conf->threshold = cpu_to_le16(wl->conf.rx.irq_pkt_threshold);
	rx_conf->timeout = cpu_to_le16(wl->conf.rx.irq_timeout);
	rx_conf->mblk_threshold = cpu_to_le16(wl->conf.rx.irq_blk_threshold);
	rx_conf->queue_type = wl->conf.rx.queue_type;

	ret = cc33xx_cmd_configure(wl, ACX_RX_CONFIG_OPT, rx_conf,
				   sizeof(*rx_conf));
	if (ret < 0) {
		cc33xx_warning("wl1271 rx config opt failed: %d", ret);
		goto out;
	}

out:
	kfree(rx_conf);
	return ret;
}

int wl1271_acx_bet_enable(struct wl1271 *wl, struct wl12xx_vif *wlvif,
			  bool enable)
{
	struct wl1271_acx_bet_enable *acx = NULL;
	int ret = 0;

	cc33xx_debug(DEBUG_ACX, "acx bet enable");

	if (enable && wl->conf.conn.bet_enable == CONF_BET_MODE_DISABLE)
		goto out;

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->role_id = wlvif->role_id;
	acx->enable = enable ? CONF_BET_MODE_ENABLE : CONF_BET_MODE_DISABLE;
	acx->max_consecutive = wl->conf.conn.bet_max_consecutive;

	ret = cc33xx_cmd_configure(wl, ACX_BET_ENABLE, acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("acx bet enable failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int wl1271_acx_arp_ip_filter(struct wl1271 *wl, struct wl12xx_vif *wlvif,
			     u8 enable, __be32 address)
{
	struct wl1271_acx_arp_filter *acx;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx arp ip filter, enable: %d", enable);

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->role_id = wlvif->role_id;
	acx->version = ACX_IPV4_VERSION;
	acx->enable = enable;

	if (enable)
		memcpy(acx->address, &address, ACX_IPV4_ADDR_SIZE);

	ret = cc33xx_cmd_configure(wl, ACX_ARP_IP_FILTER,
				   acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("failed to set arp ip filter: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int wl1271_acx_pm_config(struct wl1271 *wl)
{
	struct wl1271_acx_pm_config *acx = NULL;
	struct  conf_pm_config_settings *c = &wl->conf.pm_config;
	int ret = 0;

	cc33xx_debug(DEBUG_ACX, "acx pm config");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->host_clk_settling_time = cpu_to_le32(c->host_clk_settling_time);
	acx->host_fast_wakeup_support = c->host_fast_wakeup_support;

	ret = cc33xx_cmd_configure(wl, ACX_PM_CONFIG, acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("acx pm config failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int wl1271_acx_rssi_snr_trigger(struct wl1271 *wl, struct wl12xx_vif *wlvif,
				bool enable, s16 thold, u8 hyst)
{
	struct wl1271_acx_rssi_snr_trigger *acx = NULL;
	int ret = 0;

	cc33xx_debug(DEBUG_ACX, "acx rssi snr trigger");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	wlvif->last_rssi_event = -1;

	acx->role_id = wlvif->role_id;
	acx->pacing = cpu_to_le16(wl->conf.roam_trigger.trigger_pacing);
	acx->metric = WL1271_ACX_TRIG_METRIC_RSSI_BEACON;
	acx->type = WL1271_ACX_TRIG_TYPE_EDGE;
	if (enable)
		acx->enable = WL1271_ACX_TRIG_ENABLE;
	else
		acx->enable = WL1271_ACX_TRIG_DISABLE;

	acx->index = WL1271_ACX_TRIG_IDX_RSSI;
	acx->dir = WL1271_ACX_TRIG_DIR_BIDIR;
	acx->threshold = cpu_to_le16(thold);
	acx->hysteresis = hyst;

	ret = cc33xx_cmd_configure(wl, ACX_RSSI_SNR_TRIGGER, acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("acx rssi snr trigger setting failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int wl1271_acx_rssi_snr_avg_weights(struct wl1271 *wl,
				    struct wl12xx_vif *wlvif)
{
	struct wl1271_acx_rssi_snr_avg_weights *acx = NULL;
	struct conf_roam_trigger_settings *c = &wl->conf.roam_trigger;
	int ret = 0;

	cc33xx_debug(DEBUG_ACX, "acx rssi snr avg weights");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->role_id = wlvif->role_id;
	acx->rssi_beacon = c->avg_weight_rssi_beacon;
	acx->rssi_data = c->avg_weight_rssi_data;
	acx->snr_beacon = c->avg_weight_snr_beacon;
	acx->snr_data = c->avg_weight_snr_data;

	ret = cc33xx_cmd_configure(wl, ACX_RSSI_SNR_WEIGHTS, acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("acx rssi snr trigger weights failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int cc33xx_acx_set_ht_capabilities(struct wl1271 *wl,
				    struct ieee80211_sta_ht_cap *ht_cap,
				    bool allow_ht_operation, u8 hlid)
{
	struct wl1271_acx_ht_capabilities *acx;
	int ret = 0;
	u32 ht_capabilites = 0;

	cc33xx_debug(DEBUG_ACX, "acx ht capabilities setting "
		     "sta supp: %d sta cap: %d", ht_cap->ht_supported,
		     ht_cap->cap);

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	if (allow_ht_operation && ht_cap->ht_supported) {
		/* no need to translate capabilities - use the spec values */
		ht_capabilites = ht_cap->cap;

		/*
		 * this bit is not employed by the spec but only by FW to
		 * indicate peer HT support
		 */
		ht_capabilites |= WL12XX_HT_CAP_HT_OPERATION;

		/* get data from A-MPDU parameters field */
		acx->ampdu_max_length = ht_cap->ampdu_factor;
		acx->ampdu_min_spacing = ht_cap->ampdu_density;
	}

	acx->hlid = hlid;
	acx->ht_capabilites = cpu_to_le32(ht_capabilites);

	ret = cc33xx_cmd_configure(wl, ACX_PEER_HT_CAP, acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("acx ht capabilities setting failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}


int wl1271_acx_set_ht_information(struct wl1271 *wl,
				   struct wl12xx_vif *wlvif,
				   u16 ht_operation_mode,
				   u32 he_oper_params, u16 he_oper_nss_set)
{
	struct wl1271_acx_ht_information *acx;
	int ret = 0;

	cc33xx_debug(DEBUG_ACX, "acx ht information setting");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->role_id = wlvif->role_id;
	acx->ht_protection =
		(u8)(ht_operation_mode & IEEE80211_HT_OP_MODE_PROTECTION);
	acx->rifs_mode = 0;
	acx->gf_protection =
		!!(ht_operation_mode & IEEE80211_HT_OP_MODE_NON_GF_STA_PRSNT);

	acx->dual_cts_protection = 0;
	
    	cc33xx_debug(DEBUG_ACX, "HE operation: 0x%xm mcs: 0x%x",he_oper_params, he_oper_nss_set);

	acx->he_operation = cpu_to_le32(he_oper_params);
	acx->bss_basic_mcs_set = cpu_to_le16(he_oper_nss_set);
	acx->qos_info_more_data_ack_bit = 0; // TODO
	ret = cc33xx_cmd_configure(wl, BSS_OPERATION_CFG, acx, sizeof(*acx));

	if (ret < 0) {
		cc33xx_warning("acx ht information setting failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

/* Configure BA session initiator/receiver parameters setting in the FW. */
int wl12xx_acx_set_ba_initiator_policy(struct wl1271 *wl,
				       struct wl12xx_vif *wlvif)
{
	struct wl1271_acx_ba_initiator_policy *acx;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx ba initiator policy");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	/* set for the current role */
	acx->role_id = wlvif->role_id;
	acx->tid_bitmap = wl->conf.ht.tx_ba_tid_bitmap;
	acx->win_size = wl->conf.ht.tx_ba_win_size;
	acx->inactivity_timeout = wl->conf.ht.inactivity_timeout;

	ret = cc33xx_cmd_configure(wl,
				   ACX_BA_SESSION_INIT_POLICY,
				   acx,
				   sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("acx ba initiator policy failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

/* setup BA session receiver setting in the FW. */
int wl12xx_acx_set_ba_receiver_session(struct wl1271 *wl, u8 tid_index,
				       u16 ssn, bool enable, u8 peer_hlid,
				       u8 win_size)
{
	struct wl1271_acx_ba_receiver_setup *acx;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx ba receiver session setting");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->hlid = peer_hlid;
	acx->tid = tid_index;
	acx->enable = enable;
	acx->win_size =	win_size;
	acx->ssn = ssn;

	ret = wlcore_cmd_configure_failsafe(wl, BA_SESSION_RX_SETUP_CFG, acx,
					    sizeof(*acx),
					    BIT(CMD_STATUS_NO_RX_BA_SESSION));
	if (ret < 0) {
		cc33xx_warning("acx ba receiver session failed: %d", ret);
		goto out;
	}

	/* sometimes we can't start the session */
	if (ret == CMD_STATUS_NO_RX_BA_SESSION) {
		cc33xx_warning("no fw rx ba on tid %d", tid_index);
		ret = -EBUSY;
		goto out;
	}

	ret = 0;
out:
	kfree(acx);
	return ret;
}


int wl12xx_acx_static_calibration_configure(struct wl1271 *wl,
                        u8 file_version,
						u8 payload_struct_version,
					    u8 *data_ptr,
					    bool valid_data)
{
	struct wl1271_acx_static_calibration_cfg *acx;
	size_t acx_size;
	struct calibration_header *tmp_header;
	struct calibration_header_fw*  tmp_fw_header;
	int ret;

	tmp_header = (struct calibration_header *)data_ptr;
	tmp_fw_header  = &(tmp_header->cal_header_fw);

	cc33xx_debug(DEBUG_ACX, "acx static calibration configuration");

	if (valid_data)
		acx_size = ALIGN(sizeof(*acx) + tmp_fw_header->length, 4);
	else
		acx_size = sizeof(*acx);

	acx = kzalloc(acx_size, GFP_KERNEL);
	if (!acx) {
		cc33xx_warning("acx static calibration configuration "
				"process failed due to memory allocation "
				"failure");
		return -ENOMEM;
	}

	// let FW know if the configuration data is valid
	acx->valid_data = valid_data;
    acx->file_version = file_version;
	acx->payload_struct_version = payload_struct_version;
	
	if (!valid_data)
	{
		cc33xx_debug(DEBUG_ACX, "sending non valid data info");
		goto out;
	}

	// copy header & payload
	memcpy(&(acx->calibration_header), tmp_fw_header,
		sizeof(acx->calibration_header) + tmp_fw_header->length);

	cc33xx_debug(DEBUG_ACX, "acx calibration payload length: %d",
			acx->calibration_header.length);

out:
	ret = cc33xx_cmd_configure(wl,
				   STATIC_CALIBRATION_CFG,
				   acx,
				   acx_size);

	if (ret < 0)
		cc33xx_warning("acx command sending failed: %d", ret);
	
	kfree(acx);
	return ret;
}

int wl12xx_acx_tsf_info(struct wl1271 *wl, struct wl12xx_vif *wlvif,
			u64 *mactime)
{
	struct wl12xx_acx_fw_tsf_information *tsf_info;
	int ret;

	tsf_info = kzalloc(sizeof(*tsf_info), GFP_KERNEL);
	if (!tsf_info) {
		ret = -ENOMEM;
		goto out;
	}

	tsf_info->role_id = wlvif->role_id;

	ret = wl1271_cmd_interrogate(wl, ACX_TSF_INFO, tsf_info,
				sizeof(struct acx_header), sizeof(*tsf_info));
	if (ret < 0) {
		cc33xx_warning("acx tsf info interrogate failed");
		goto out;
	}

	*mactime = le32_to_cpu(tsf_info->current_tsf_low) |
		((u64) le32_to_cpu(tsf_info->current_tsf_high) << 32);

out:
	kfree(tsf_info);
	return ret;
}

int cc33xx_acx_ps_rx_streaming(struct wl1271 *wl, struct wl12xx_vif *wlvif,
			       bool enable)
{
	struct cc33xx_acx_ps_rx_streaming *rx_streaming;
	u32 conf_queues, enable_queues;
	int i, ret = 0;

	cc33xx_debug(DEBUG_ACX, "acx ps rx streaming");

	rx_streaming = kzalloc(sizeof(*rx_streaming), GFP_KERNEL);
	if (!rx_streaming) {
		ret = -ENOMEM;
		goto out;
	}

	conf_queues = wl->conf.rx_streaming.queues;
	if (enable)
		enable_queues = conf_queues;
	else
		enable_queues = 0;

	for (i = 0; i < 8; i++) {
		/*
		 * Skip non-changed queues, to avoid redundant acxs.
		 * this check assumes conf.rx_streaming.queues can't
		 * be changed while rx_streaming is enabled.
		 */
		if (!(conf_queues & BIT(i)))
			continue;

		rx_streaming->role_id = wlvif->role_id;
		rx_streaming->tid = i;
		rx_streaming->enable = enable_queues & BIT(i);
		rx_streaming->period = wl->conf.rx_streaming.interval;
		rx_streaming->timeout = wl->conf.rx_streaming.interval;

		ret = cc33xx_cmd_configure(wl, ACX_PS_RX_STREAMING,
					   rx_streaming,
					   sizeof(*rx_streaming));
		if (ret < 0) {
			cc33xx_warning("acx ps rx streaming failed: %d", ret);
			goto out;
		}
	}
out:
	kfree(rx_streaming);
	return ret;
}

int wl1271_acx_ap_max_tx_retry(struct wl1271 *wl, struct wl12xx_vif *wlvif)
{
	struct wl1271_acx_ap_max_tx_retry *acx = NULL;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx ap max tx retry");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx)
		return -ENOMEM;

	acx->role_id = wlvif->role_id;
	acx->max_tx_retry = cpu_to_le16(wl->conf.tx.max_tx_retries);

	ret = cc33xx_cmd_configure(wl, ACX_MAX_TX_FAILURE, acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("acx ap max tx retry failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int cc33xx_acx_config_ps(struct wl1271 *wl, struct wl12xx_vif *wlvif)
{
	struct wl1271_acx_config_ps *config_ps;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx config ps");

	config_ps = kzalloc(sizeof(*config_ps), GFP_KERNEL);
	if (!config_ps) {
		ret = -ENOMEM;
		goto out;
	}

	config_ps->exit_retries = wl->conf.conn.psm_exit_retries;
	config_ps->enter_retries = wl->conf.conn.psm_entry_retries;
	config_ps->null_data_rate = cpu_to_le32(wlvif->basic_rate);

	ret = cc33xx_cmd_configure(wl, ACX_CONFIG_PS, config_ps,
				   sizeof(*config_ps));

	if (ret < 0) {
		cc33xx_warning("acx config ps failed: %d", ret);
		goto out;
	}

out:
	kfree(config_ps);
	return ret;
}

int wl1271_acx_set_inconnection_sta(struct wl1271 *wl,
				    struct wl12xx_vif *wlvif, u8 *addr)
{
	struct wl1271_acx_inconnection_sta *acx = NULL;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx set inconnaction sta %pM", addr);

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx)
		return -ENOMEM;

	memcpy(acx->addr, addr, ETH_ALEN);
	acx->role_id = wlvif->role_id;

	ret = cc33xx_cmd_configure(wl, ACX_UPDATE_INCONNECTION_STA_LIST,
				   acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("acx set inconnaction sta failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int wl12xx_acx_set_rate_mgmt_params(struct wl1271 *wl)
{
	struct wl12xx_acx_set_rate_mgmt_params *acx = NULL;
	struct conf_rate_policy_settings *conf = &wl->conf.rate;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx set rate mgmt params");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx)
		return -ENOMEM;

	acx->index = ACX_RATE_MGMT_ALL_PARAMS;
	acx->rate_retry_score = cpu_to_le16(conf->rate_retry_score);
	acx->per_add = cpu_to_le16(conf->per_add);
	acx->per_th1 = cpu_to_le16(conf->per_th1);
	acx->per_th2 = cpu_to_le16(conf->per_th2);
	acx->max_per = cpu_to_le16(conf->max_per);
	acx->inverse_curiosity_factor = conf->inverse_curiosity_factor;
	acx->tx_fail_low_th = conf->tx_fail_low_th;
	acx->tx_fail_high_th = conf->tx_fail_high_th;
	acx->per_alpha_shift = conf->per_alpha_shift;
	acx->per_add_shift = conf->per_add_shift;
	acx->per_beta1_shift = conf->per_beta1_shift;
	acx->per_beta2_shift = conf->per_beta2_shift;
	acx->rate_check_up = conf->rate_check_up;
	acx->rate_check_down = conf->rate_check_down;
	memcpy(acx->rate_retry_policy, conf->rate_retry_policy,
	       sizeof(acx->rate_retry_policy));

	ret = cc33xx_cmd_configure(wl, ACX_SET_RATE_MGMT_PARAMS,
				   acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("acx set rate mgmt params failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int wl12xx_acx_config_hangover(struct wl1271 *wl)
{
	struct wl12xx_acx_config_hangover *acx;
	struct conf_hangover_settings *conf = &wl->conf.hangover;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx config hangover");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->recover_time = cpu_to_le32(conf->recover_time);
	acx->hangover_period = conf->hangover_period;
	acx->dynamic_mode = conf->dynamic_mode;
	acx->early_termination_mode = conf->early_termination_mode;
	acx->max_period = conf->max_period;
	acx->min_period = conf->min_period;
	acx->increase_delta = conf->increase_delta;
	acx->decrease_delta = conf->decrease_delta;
	acx->quiet_time = conf->quiet_time;
	acx->increase_time = conf->increase_time;
	acx->window_size = conf->window_size;

	ret = cc33xx_cmd_configure(wl, ACX_CONFIG_HANGOVER, acx,
				   sizeof(*acx));

	if (ret < 0) {
		cc33xx_warning("acx config hangover failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;

}

int wlcore_acx_average_rssi(struct wl1271 *wl, struct wl12xx_vif *wlvif,
			    s8 *avg_rssi)
{
	struct acx_roaming_stats *acx;
	int ret = 0;

	cc33xx_debug(DEBUG_ACX, "acx roaming statistics");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->role_id = wlvif->role_id;
	ret = wl1271_cmd_interrogate(wl, ACX_ROAMING_STATISTICS_TBL,
				     acx, sizeof(*acx), sizeof(*acx));
	if (ret	< 0) {
		cc33xx_warning("acx roaming statistics failed: %d", ret);
		ret = -ENOMEM;
		goto out;
	}

	// RazB: test for now ++DEBUG++
	*avg_rssi = -60;//acx->rssi_beacon;
out:
	kfree(acx);
	return ret;
}

#ifdef CONFIG_PM
/* Set the global behaviour of RX filters - On/Off + default action */
int cc33xx_acx_default_rx_filter_enable(struct wl1271 *wl, bool enable,
					enum rx_filter_action action)
{
	struct acx_default_rx_filter *acx;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx default rx filter en: %d act: %d",
		     enable, action);

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx)
		return -ENOMEM;

	acx->enable = enable;
	acx->default_action = action;

	ret = cc33xx_cmd_configure(wl, ACX_ENABLE_RX_DATA_FILTER, acx,
				   sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("acx default rx filter enable failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

/* Configure or disable a specific RX filter pattern */
int cc33xx_acx_set_rx_filter(struct wl1271 *wl, u8 index, bool enable,
			     struct cc33xx_rx_filter *filter)
{
	struct acx_rx_filter_cfg *acx;
	int fields_size = 0;
	int acx_size;
	int ret;

	WARN_ON(enable && !filter);
	WARN_ON(index >= CC33XX_MAX_RX_FILTERS);

	cc33xx_debug(DEBUG_ACX,
		     "acx set rx filter idx: %d enable: %d filter: %p",
		     index, enable, filter);

	if (enable) {
		fields_size = cc33xx_rx_filter_get_fields_size(filter);

		cc33xx_debug(DEBUG_ACX, "act: %d num_fields: %d field_size: %d",
		      filter->action, filter->num_fields, fields_size);
	}

	acx_size = ALIGN(sizeof(*acx) + fields_size, 4);
	acx = kzalloc(acx_size, GFP_KERNEL);

	if (!acx)
		return -ENOMEM;

	acx->enable = enable;
	acx->index = index;

	if (enable) {
		acx->num_fields = filter->num_fields;
		acx->action = filter->action;
		wl1271_rx_filter_flatten_fields(filter, acx->fields);
	}

	wl1271_dump(DEBUG_ACX, "RX_FILTER: ", acx, acx_size);

	ret = cc33xx_cmd_configure(wl, ACX_SET_RX_DATA_FILTER, acx, acx_size);
	if (ret < 0) {
		cc33xx_warning("setting rx filter failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}
#endif /* CONFIG_PM */



int cc33xx_acx_host_if_cfg_bitmap(struct wl1271 *wl, u32 host_cfg_bitmap,
				  u32 sdio_blk_size, u32 extra_mem_blks,
				  u32 len_field_size)
{
	struct cc33xx_acx_host_config_bitmap *bitmap_conf;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx cfg bitmap %d blk %d spare %d field %d",
		     host_cfg_bitmap, sdio_blk_size, extra_mem_blks,
		     len_field_size);

	bitmap_conf = kzalloc(sizeof(*bitmap_conf), GFP_KERNEL);
	if (!bitmap_conf) {
		ret = -ENOMEM;
		goto out;
	}

	bitmap_conf->host_cfg_bitmap = cpu_to_le32(host_cfg_bitmap);
	bitmap_conf->host_sdio_block_size = cpu_to_le32(sdio_blk_size);
	bitmap_conf->extra_mem_blocks = cpu_to_le32(extra_mem_blks);
	bitmap_conf->length_field_size = cpu_to_le32(len_field_size);

	ret = cc33xx_cmd_configure(wl, ACX_HOST_IF_CFG_BITMAP,
				   bitmap_conf, sizeof(*bitmap_conf));
	if (ret < 0) {
		cc33xx_warning("wl1271 bitmap config opt failed: %d", ret);
		goto out;
	}

out:
	kfree(bitmap_conf);

	return ret;
}

int cc33xx_acx_set_checksum_state(struct wl1271 *wl)
{
	struct wl18xx_acx_checksum_state *acx;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx checksum state");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->checksum_state = CHECKSUM_OFFLOAD_ENABLED;

	ret = cc33xx_cmd_configure(wl, ACX_CSUM_CONFIG, acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("failed to set Tx checksum state: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}


int cc33xx_acx_peer_ht_operation_mode(struct wl1271 *wl, u8 hlid, bool wide)
{
	struct wlcore_peer_ht_operation_mode *acx;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx peer ht operation mode hlid %d bw %d",
		     hlid, wide);

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->hlid = hlid;
	acx->bandwidth = wide ? WLCORE_BANDWIDTH_40MHZ : WLCORE_BANDWIDTH_20MHZ;

	ret = cc33xx_cmd_configure(wl, ACX_PEER_HT_OPERATION_MODE_CFG, acx,
				   sizeof(*acx));

	if (ret < 0) {
		cc33xx_warning("acx peer ht operation mode failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;

}

/*
 * this command is basically the same as wl1271_acx_ht_capabilities,
 * with the addition of supported rates. they should be unified in
 * the next fw api change
 */
int cc33xx_acx_set_peer_cap(struct wl1271 *wl,
			    struct ieee80211_sta_ht_cap *ht_cap,
			    struct ieee80211_sta_he_cap *he_cap,
			    struct wl12xx_vif *wlvif,
			    bool allow_ht_operation,
			    u32 rate_set, u8 hlid)
{
	struct wlcore_acx_peer_cap *acx;
	int ret = 0;
	u32 ht_capabilites = 0;

	cc33xx_debug(DEBUG_ACX,
		     "acx set cap ht_supp: %d ht_cap: %d rates: 0x%x",
		     ht_cap->ht_supported, ht_cap->cap, rate_set);

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	if (allow_ht_operation && ht_cap->ht_supported) {
		/* no need to translate capabilities - use the spec values */
		ht_capabilites = ht_cap->cap;

		/*
		 * this bit is not employed by the spec but only by FW to
		 * indicate peer HT support
		 */
		ht_capabilites |= WL12XX_HT_CAP_HT_OPERATION;

		/* get data from A-MPDU parameters field */
		acx->ampdu_max_length = ht_cap->ampdu_factor;
		acx->ampdu_min_spacing = ht_cap->ampdu_density;
	}

	acx->ht_capabilites = cpu_to_le32(ht_capabilites);
	acx->supported_rates = cpu_to_le32(rate_set);

	acx->role_id = wlvif->role_id;
	acx->has_he = he_cap->has_he;
	memcpy(acx->mac_cap_info, he_cap->he_cap_elem.mac_cap_info, 6);
	acx->nominal_packet_padding = (he_cap->he_cap_elem.phy_cap_info[8] & NOMINAL_PACKET_PADDING);
    ret = cc33xx_cmd_configure(wl, PEER_CAP_CFG, acx, sizeof(*acx));

	if (ret < 0) {
		cc33xx_warning("acx ht capabilities setting failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

/*
 * When the host is suspended, we don't want to get any fast-link/PSM
 * notifications
 */
int cc33xx_acx_interrupt_notify_config(struct wl1271 *wl,
				       bool action)
{
	struct wl18xx_acx_interrupt_notify *acx;
	int ret = 0;

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->enable = action;
	ret = cc33xx_cmd_configure(wl, ACX_INTERRUPT_NOTIFY, acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("acx interrupt notify setting failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

/*
 * When the host is suspended, we can configure the FW to disable RX BA
 * notifications.
 */
int cc33xx_acx_rx_ba_filter(struct wl1271 *wl, bool action)
{
	struct wl18xx_acx_rx_ba_filter *acx;
	int ret = 0;

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->enable = (u32)action;
	ret = cc33xx_cmd_configure(wl, ACX_RX_BA_FILTER, acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("acx rx ba activity filter setting failed: %d",
			       ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int wl18xx_acx_ap_sleep(struct wl1271 *wl)
{
	struct acx_ap_sleep_cfg *acx;
	struct conf_ap_sleep_settings *conf = &wl->conf.ap_sleep;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx config ap sleep");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->idle_duty_cycle = conf->idle_duty_cycle;
	acx->connected_duty_cycle = conf->connected_duty_cycle;
	acx->max_stations_thresh = conf->max_stations_thresh;
	acx->idle_conn_thresh = conf->idle_conn_thresh;

	ret = cc33xx_cmd_configure(wl, ACX_AP_SLEEP_CFG, acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("acx config ap-sleep failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}



int wl18xx_acx_time_sync_cfg(struct wl1271 *wl)
{
	struct acx_time_sync_cfg *acx;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx time sync cfg: mode %d, addr: %pM",
		     wl->conf.sg.params[CC33XX_CONF_SG_TIME_SYNC],
		     wl->zone_master_mac_addr);

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->sync_mode = wl->conf.sg.params[CC33XX_CONF_SG_TIME_SYNC];
	memcpy(acx->zone_mac_addr, wl->zone_master_mac_addr, ETH_ALEN);

	ret = cc33xx_cmd_configure(wl, ACX_TIME_SYNC_CFG,
				   acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("acx time sync cfg failed: %d", ret);
		goto out;
	}
out:
	kfree(acx);
	return ret;
}
