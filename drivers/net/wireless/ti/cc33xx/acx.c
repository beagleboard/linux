// SPDX-License-Identifier: GPL-2.0-only
/*
 * This file is part of cc33xx
 *
 * Copyright (C) 2008-2009 Nokia Corporation
 *
 * Contact: Luciano Coelho <luciano.coelho@nokia.com>
 */

#include "acx.h"


int cc33xx_acx_dynamic_fw_traces(struct cc33xx *wl)
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

int cc33xx_acx_clear_statistics(struct cc33xx *wl)
{
	struct acx_header *acx;
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

int cc33xx_acx_wake_up_conditions(struct cc33xx *wl, struct cc33xx_vif *wlvif,
				  u8 wake_up_event, u8 listen_interval)
{
	struct acx_wake_up_condition *wake_up;
	int ret;

	cc33xx_debug(DEBUG_ACX,
		     "acx wake up conditions (wake_up_event %d listen_interval %d)",
		     wake_up_event, listen_interval);

	wake_up = kzalloc(sizeof(*wake_up), GFP_KERNEL);
	if (!wake_up) {
		ret = -ENOMEM;
		goto out;
	}

	wake_up->wake_up_event = wake_up_event;
	wake_up->listen_interval = listen_interval;

	ret = cc33xx_cmd_configure(wl, WAKE_UP_CONDITIONS_CFG,
				   wake_up, sizeof(*wake_up));
	if (ret < 0) {
		cc33xx_warning("could not set wake up conditions: %d", ret);
		goto out;
	}

out:
	kfree(wake_up);
	return ret;
}

int cc33xx_acx_sleep_auth(struct cc33xx *wl, u8 sleep_auth)
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

int cc33xx_ble_enable(struct cc33xx *wl, u8 ble_enable)
{
	struct debug_header *buf;
	int ret;

	cc33xx_debug(DEBUG_ACX, "ble enable");

	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf) {
		ret = -ENOMEM;
		goto out;
	}

	ret = cc33xx_cmd_debug(wl, BLE_ENABLE, buf, sizeof(*buf));
	if (ret < 0) {
		cc33xx_error("could not enable ble");
		goto out;
	}

	wl->ble_enable = 1;
out:
	kfree(buf);
	return ret;
}

int cc33xx_acx_tx_power(struct cc33xx *wl, struct cc33xx_vif *wlvif,
			int power)
{
	struct acx_tx_power_cfg *acx;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx TX_POWER_CFG %d", power);

	if (power < CC33XX_MIN_TXPWR) {
		cc33xx_warning("Configured Tx power %d dBm."
			       " Increasing to minimum %d dBm",
			       power, CC33XX_MIN_TXPWR);
		power = CC33XX_MIN_TXPWR;
	} else if (power > CC33XX_MAX_TXPWR) {
		cc33xx_warning("Configured Tx power %d dBm is bigger than upper"
			       " limit: %d dBm. Attenuating to max limit",
			       power, CC33XX_MAX_TXPWR);
		power = CC33XX_MAX_TXPWR;
	}

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->role_id = wlvif->role_id;
	acx->tx_power = power;

	ret = cc33xx_cmd_configure(wl, TX_POWER_CFG, acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("Configure of tx power failed: %d", ret);
		goto out;
	}

	wlvif->power_level = power;

out:
	kfree(acx);
	return ret;
}

int cc33xx_acx_feature_cfg(struct cc33xx *wl, struct cc33xx_vif *wlvif)
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

static int cc33xx_acx_mem_map(struct cc33xx *wl,
			      struct acx_header *mem_map, size_t len)
{
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx mem map");

	ret = cc33xx_cmd_interrogate(wl, MEM_MAP_INTR, mem_map,
				     sizeof(struct acx_header), len);
	if (ret < 0)
		return ret;

	return 0;
}

static int cc33xx_acx_get_fw_versions(struct cc33xx *wl,
				      struct cc33xx_acx_fw_versions *get_fw_versions,
				      size_t len)
{
	int ret;
	cc33xx_debug(DEBUG_ACX, "acx get FW versions");

	ret = cc33xx_cmd_interrogate(wl, GET_FW_VERSIONS_INTR, get_fw_versions,
				     sizeof(struct cc33xx_acx_fw_versions), len);
	if (ret < 0)
		return ret;
	return 0;
}

int cc33xx_acx_slot(struct cc33xx *wl, struct cc33xx_vif *wlvif,
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

int cc33xx_acx_group_address_tbl(struct cc33xx *wl, struct cc33xx_vif *wlvif,
				 bool enable, void *mc_list, u32 mc_list_len)
{
	struct acx_dot11_grp_addr_tbl *acx;
	int ret;

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	cc33xx_debug(DEBUG_ACX, "acx group address tbl");

	acx->enabled = enable;
	acx->num_groups = mc_list_len;
	memcpy(acx->mac_table, mc_list, mc_list_len * ETH_ALEN);

	ret = cc33xx_cmd_configure(wl, DOT11_GROUP_ADDRESS_TBL,
				   acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("failed to set group addr table: %d", ret);
		goto out;
	}
out:
	kfree(acx);
	return ret;
}

int cc33xx_acx_service_period_timeout(struct cc33xx *wl,
				      struct cc33xx_vif *wlvif)
{
	struct acx_rx_timeout *rx_timeout;
	struct conf_rx_settings *rx_settings = &wl->conf.host_conf.rx;
	int ret;

	rx_timeout = kzalloc(sizeof(*rx_timeout), GFP_KERNEL);
	if (!rx_timeout) {
		ret = -ENOMEM;
		goto out;
	}

	cc33xx_debug(DEBUG_ACX, "acx service period timeout");

	rx_timeout->role_id = wlvif->role_id;
	rx_timeout->ps_poll_timeout = cpu_to_le16(rx_settings->ps_poll_timeout);
	rx_timeout->upsd_timeout = cpu_to_le16(rx_settings->upsd_timeout);

	ret = cc33xx_cmd_configure(wl, ACX_SERVICE_PERIOD_TIMEOUT,
				   rx_timeout, sizeof(*rx_timeout));
	if (ret < 0) {
		cc33xx_warning("failed to set service period timeout: %d", ret);
		goto out;
	}

out:
	kfree(rx_timeout);
	return ret;
}

int cc33xx_acx_rts_threshold(struct cc33xx *wl, struct cc33xx_vif *wlvif,
			     u32 rts_threshold)
{
	struct acx_rts_threshold *rts;
	int ret;

	/*
	 * If the RTS threshold is not configured or out of range, use the
	 * default value.
	 */
	if (rts_threshold > IEEE80211_MAX_RTS_THRESHOLD)
		rts_threshold = wl->conf.host_conf.rx.rts_threshold;

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

int cc33xx_acx_beacon_filter_opt(struct cc33xx *wl, struct cc33xx_vif *wlvif,
				 bool enable_filter)
{
	struct acx_beacon_filter_option *beacon_filter = NULL;
	int ret = 0;

	cc33xx_debug(DEBUG_ACX, "acx beacon filter opt enable=%d",
		     enable_filter);

	if (enable_filter &&
	    wl->conf.host_conf.conn.bcn_filt_mode == CONF_BCN_FILT_MODE_DISABLED)
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

	ret = cc33xx_cmd_configure(wl, BEACON_FILTER_OPT,
				   beacon_filter, sizeof(*beacon_filter));
	if (ret < 0) {
		cc33xx_warning("failed to set beacon filter opt: %d", ret);
		goto out;
	}

out:
	kfree(beacon_filter);
	return ret;
}

int cc33xx_acx_beacon_filter_table(struct cc33xx *wl, struct cc33xx_vif *wlvif)
{
	struct acx_beacon_filter_ie_table *ie_table;
	struct conf_bcn_filt_rule bcn_filt_ie[32];
	struct conf_bcn_filt_rule* p_bcn_filt_ie;
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
	p_bcn_filt_ie =&(wl->conf.host_conf.conn.bcn_filt_ie0);
	memcpy(bcn_filt_ie,p_bcn_filt_ie,32* sizeof(struct conf_bcn_filt_rule));
	for (i = 0; i < wl->conf.host_conf.conn.bcn_filt_ie_count; i++) {
		struct conf_bcn_filt_rule *r = &bcn_filt_ie[i];
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

	ret = cc33xx_cmd_configure(wl, BEACON_FILTER_TABLE,
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

int cc33xx_acx_conn_monit_params(struct cc33xx *wl, struct cc33xx_vif *wlvif,
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
		threshold = wl->conf.host_conf.conn.synch_fail_thold;
		timeout = wl->conf.host_conf.conn.bss_lose_timeout;
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

int cc33xx_acx_bcn_dtim_options(struct cc33xx *wl, struct cc33xx_vif *wlvif)
{
	struct acx_beacon_broadcast *bb;
	struct conf_conn_settings *conn_settings = &wl->conf.host_conf.conn;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx bcn dtim options");

	bb = kzalloc(sizeof(*bb), GFP_KERNEL);
	if (!bb) {
		ret = -ENOMEM;
		goto out;
	}

	bb->role_id = wlvif->role_id;
	bb->beacon_rx_timeout = cpu_to_le16(conn_settings->beacon_rx_timeout);
	bb->broadcast_timeout = cpu_to_le16(conn_settings->broadcast_timeout);
	bb->rx_broadcast_in_ps = conn_settings->rx_broadcast_in_ps;
	bb->ps_poll_threshold = conn_settings->ps_poll_threshold;

	ret = cc33xx_cmd_configure(wl, ACX_BCN_DTIM_OPTIONS, bb, sizeof(*bb));
	if (ret < 0) {
		cc33xx_warning("failed to set rx config: %d", ret);
		goto out;
	}

out:
	kfree(bb);
	return ret;
}

int cc33xx_assoc_info_cfg(struct cc33xx *wl, struct cc33xx_vif *wlvif,
			  struct ieee80211_sta *sta, u16 aid)
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
	cfg->ht_supported = sta->deflink.ht_cap.ht_supported;
	cfg->vht_supported = sta->deflink.vht_cap.vht_supported;
	cfg->has_he = sta->deflink.he_cap.has_he;
	memcpy(cfg->transmitter_bssid, wlvif->transmitter_bssid, ETH_ALEN);
	ret = cc33xx_cmd_configure(wl, ASSOC_INFO_CFG, cfg, sizeof(*cfg));
	if (ret < 0) {
		cc33xx_warning("failed to set aid: %d", ret);
		goto out;
	}

out:
	kfree(cfg);
	return ret;
}

int cc33xx_acx_event_mbox_mask(struct cc33xx *wl, u32 event_mask)
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

int cc33xx_acx_set_preamble(struct cc33xx *wl, struct cc33xx_vif *wlvif,
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

int cc33xx_acx_cts_protect(struct cc33xx *wl, struct cc33xx_vif *wlvif,
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

int cc33xx_acx_statistics(struct cc33xx *wl, void *stats)
{
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx statistics");

	ret = cc33xx_cmd_interrogate(wl, ACX_STATISTICS, stats,
				     sizeof(struct acx_header),
				     sizeof(struct cc33xx_acx_statistics_t));
	if (ret < 0) {
		cc33xx_warning("acx statistics failed: %d", ret);
		return -ENOMEM;
	}

	return 0;
}

int cc33xx_update_ap_rates(struct cc33xx *wl, u8 role_id,
			   u32 basic_rates_set, u32 supported_rates)
{
	struct ap_rates_class_cfg *cfg;
	int ret;
	cc33xx_debug(DEBUG_AP,
		     "Attempting to Update Basic Rates and Supported Rates");

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

int cc33xx_tx_param_cfg(struct cc33xx *wl, struct cc33xx_vif *wlvif, u8 ac,
			u8 cw_min, u16 cw_max, u8 aifsn, u16 txop, bool acm,
			u8 ps_scheme, u8 is_mu_edca, u8 mu_edca_aifs,
			u8 mu_edca_ecw_min_max, u8 mu_edca_timer)
{
	struct tx_param_cfg *cfg;
	int ret = 0;

	cc33xx_debug(DEBUG_ACX,
		     "tx param cfg %d cw_ming %d cw_max %d aifs %d txop %d",
		     ac, cw_min, cw_max, aifsn, txop);

	cc33xx_debug(DEBUG_ACX, "tx param cfg ps_scheme %d is_mu_edca %d "
		     "mu_edca_aifs %d mu_edca_ecw_min_max %d mu_edca_timer %d",
		     ps_scheme, is_mu_edca, mu_edca_aifs, mu_edca_ecw_min_max,
		     mu_edca_timer);

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
	cfg->ps_scheme = ps_scheme;
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

int cc33xx_acx_frag_threshold(struct cc33xx *wl, u32 frag_threshold)
{
	struct acx_frag_threshold *acx;
	int ret = 0;

	/*
	 * If the fragmentation is not configured or out of range, use the
	 * default value.
	 */
	if (frag_threshold > IEEE80211_MAX_FRAG_THRESHOLD)
		frag_threshold = wl->conf.host_conf.tx.frag_threshold;

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

int cc33xx_acx_mem_cfg(struct cc33xx *wl)
{
	struct cc33xx_acx_config_memory *mem_conf;
	struct conf_memory_settings *mem;
	int ret;

	cc33xx_debug(DEBUG_ACX, "cc33xx mem cfg");

	mem_conf = kzalloc(sizeof(*mem_conf), GFP_KERNEL);
	if (!mem_conf) {
		ret = -ENOMEM;
		goto out;
	}

	mem = &wl->conf.host_conf.mem;

	/* memory config */
	mem_conf->num_stations = mem->num_stations;
	mem_conf->rx_mem_block_num = mem->rx_block_num;
	mem_conf->tx_min_mem_block_num = mem->tx_min_block_num;
	mem_conf->num_ssid_profiles = mem->ssid_profiles;
	mem_conf->total_tx_descriptors = cpu_to_le32(CC33XX_NUM_TX_DESCRIPTORS);
	mem_conf->dyn_mem_enable = mem->dynamic_memory;
	mem_conf->tx_free_req = mem->min_req_tx_blocks;
	mem_conf->rx_free_req = mem->min_req_rx_blocks;
	mem_conf->tx_min = mem->tx_min;
	mem_conf->fwlog_blocks = wl->conf.host_conf.fwlog.mem_blocks;

	ret = cc33xx_cmd_configure(wl, ACX_MEM_CFG, mem_conf,
				   sizeof(*mem_conf));
	if (ret < 0) {
		cc33xx_warning("cc33xx mem config failed: %d", ret);
		goto out;
	}

out:
	kfree(mem_conf);
	return ret;
}

int cc33xx_acx_init_mem_config(struct cc33xx *wl)
{
	int ret;

	wl->target_mem_map = kzalloc(sizeof(struct cc33xx_acx_mem_map_t),
				     GFP_KERNEL);
	if (!wl->target_mem_map) {
		cc33xx_error("couldn't allocate target memory map");
		return -ENOMEM;
	}

	/* we now ask for the firmware built memory map */
	ret = cc33xx_acx_mem_map(wl, (void *)wl->target_mem_map,
				 sizeof(struct cc33xx_acx_mem_map_t));
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

	cc33xx_debug(DEBUG_TX,
		     "available tx descriptor: %d available rx blocks %d",
		     wl->target_mem_map->num_tx_descriptor,
		     wl->target_mem_map->num_rx_mem_blocks);

	return 0;
}

int cc33xx_acx_init_get_fw_versions(struct cc33xx *wl)
{
	int ret;

	wl->all_versions.fw_ver = kzalloc(sizeof(struct cc33xx_acx_fw_versions),
					GFP_KERNEL);
	if (!wl->all_versions.fw_ver) {
		cc33xx_error("couldn't allocate cc33xx_acx_fw_versions");
		return -ENOMEM;
	}

	ret = cc33xx_acx_get_fw_versions(wl, (void *)wl->all_versions.fw_ver,
				sizeof(struct cc33xx_acx_fw_versions));
	if (ret < 0) {
		cc33xx_error("couldn't retrieve firmware versions");
		kfree(wl->all_versions.fw_ver);
		wl->all_versions.fw_ver = NULL;
		return ret;
	}

	return 0;
}

int cc33xx_acx_init_rx_interrupt(struct cc33xx *wl)
{
	struct cc33xx_acx_rx_config_opt *rx_conf;
	struct conf_rx_settings *rx_settings = &wl->conf.host_conf.rx;
	int ret;

	cc33xx_debug(DEBUG_ACX, "cc33xx rx interrupt config");

	rx_conf = kzalloc(sizeof(*rx_conf), GFP_KERNEL);
	if (!rx_conf) {
		ret = -ENOMEM;
		goto out;
	}

	rx_conf->threshold = cpu_to_le16(rx_settings->irq_pkt_threshold);
	rx_conf->timeout = cpu_to_le16(rx_settings->irq_timeout);
	rx_conf->mblk_threshold = cpu_to_le16(rx_settings->irq_blk_threshold);
	rx_conf->queue_type = rx_settings->queue_type;

	ret = cc33xx_cmd_configure(wl, ACX_RX_CONFIG_OPT, rx_conf,
				   sizeof(*rx_conf));
	if (ret < 0) {
		cc33xx_warning("cc33xx rx config opt failed: %d", ret);
		goto out;
	}

out:
	kfree(rx_conf);
	return ret;
}

int cc33xx_acx_bet_enable(struct cc33xx *wl,
			  struct cc33xx_vif *wlvif, bool enable)
{
	struct cc33xx_acx_bet_enable *acx = NULL;
	int ret = 0;

	cc33xx_debug(DEBUG_ACX, "acx bet enable");

	if (enable && wl->conf.host_conf.conn.bet_enable == CONF_BET_MODE_DISABLE)
		goto out;

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->role_id = wlvif->role_id;
	acx->enable = enable ? CONF_BET_MODE_ENABLE : CONF_BET_MODE_DISABLE;
	acx->max_consecutive = wl->conf.host_conf.conn.bet_max_consecutive;

	ret = cc33xx_cmd_configure(wl, ACX_BET_ENABLE, acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("acx bet enable failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int cc33xx_acx_arp_ip_filter(struct cc33xx *wl, struct cc33xx_vif *wlvif,
			     u8 enable, __be32 address)
{
	struct cc33xx_acx_arp_filter *acx;
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

	ret = cc33xx_cmd_configure(wl, ACX_ARP_IP_FILTER, acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("failed to set arp ip filter: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int cc33xx_acx_rssi_snr_trigger(struct cc33xx *wl, struct cc33xx_vif *wlvif,
				bool enable, s16 thold, u8 hyst)
{
	struct cc33xx_acx_rssi_snr_trigger *acx = NULL;
	int ret = 0;

	cc33xx_debug(DEBUG_ACX, "acx rssi snr trigger");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	wlvif->last_rssi_event = -1;

	acx->role_id = wlvif->role_id;
	acx->pacing = cpu_to_le16(wl->conf.host_conf.roam_trigger.trigger_pacing);
	acx->metric = CC33XX_ACX_TRIG_METRIC_RSSI_BEACON;
	acx->type = CC33XX_ACX_TRIG_TYPE_EDGE;
	if (enable)
		acx->enable = CC33XX_ACX_TRIG_ENABLE;
	else
		acx->enable = CC33XX_ACX_TRIG_DISABLE;

	acx->index = CC33XX_ACX_TRIG_IDX_RSSI;
	acx->dir = CC33XX_ACX_TRIG_DIR_BIDIR;
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

int cc33xx_acx_rssi_snr_avg_weights(struct cc33xx *wl, struct cc33xx_vif *wlvif)
{
	struct cc33xx_acx_rssi_snr_avg_weights_t *acx = NULL;
	struct conf_roam_trigger_settings *c = &wl->conf.host_conf.roam_trigger;
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

int cc33xx_acx_set_ht_capabilities(struct cc33xx *wl,
				    struct ieee80211_sta_ht_cap *ht_cap,
				    bool allow_ht_operation, u8 hlid)
{
	struct cc33xx_acx_ht_capabilities *acx;
	int ret = 0;
	u32 ht_capabilites = 0;

	cc33xx_debug(DEBUG_ACX,
		     "acx ht capabilities setting sta supp: %d sta cap: %d",
		     ht_cap->ht_supported, ht_cap->cap);

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
		ht_capabilites |= CC33XX_HT_CAP_HT_OPERATION;

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


int cc33xx_acx_set_ht_information(struct cc33xx *wl, struct cc33xx_vif *wlvif,
				  u16 ht_operation_mode, u32 he_oper_params,
				  u16 he_oper_nss_set)
{
	struct cc33xx_acx_ht_information *acx;
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

	cc33xx_debug(DEBUG_ACX, "HE operation: 0x%xm mcs: 0x%x",
		     he_oper_params, he_oper_nss_set);

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
int cc33xx_acx_set_ba_initiator_policy(struct cc33xx *wl,
				       struct cc33xx_vif *wlvif)
{
	struct cc33xx_acx_ba_initiator_policy *acx;
	struct conf_ht_setting *ht_settings = &wl->conf.host_conf.ht;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx ba initiator policy");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	/* set for the current role */
	acx->role_id = wlvif->role_id;
	acx->tid_bitmap = ht_settings->tx_ba_tid_bitmap;
	acx->win_size = ht_settings->tx_ba_win_size;
	acx->inactivity_timeout = cpu_to_le16(ht_settings->inactivity_timeout);

	ret = cc33xx_cmd_configure(wl, ACX_BA_SESSION_INIT_POLICY,
				   acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("acx ba initiator policy failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

/* setup BA session receiver setting in the FW. */
int cc33xx_acx_set_ba_receiver_session(struct cc33xx *wl, u8 tid_index, u16 ssn,
				       bool enable, u8 peer_hlid, u8 win_size)
{
	struct cc33xx_acx_ba_receiver_setup *acx;
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
	acx->ssn = cpu_to_le16(ssn);

	ret = wlcore_cmd_configure_failsafe(wl, BA_SESSION_RX_SETUP_CFG,
					    acx, sizeof(*acx),
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


int cc33xx_acx_static_calibration_configure(struct cc33xx *wl,
					    struct calibration_file_header *file_header,
					    u8 *calibration_entry_ptr,
					    bool valid_data)
{
	struct cc33xx_acx_static_calibration_cfg *acx;
	size_t size;
	struct calibration_header *calibration_header;
	struct calibration_header_fw *fw_calibration_header;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx static calibration configuration");

	if (valid_data) {
		calibration_header = (struct calibration_header *)calibration_entry_ptr;

		/* Extract the part of header that goes to FW */
		fw_calibration_header = &(calibration_header->cal_header_fw);

		size = ALIGN(sizeof(struct cc33xx_acx_static_calibration_cfg)
				 + fw_calibration_header->length, 4);
	} else {
		size = sizeof(struct cc33xx_acx_static_calibration_cfg);
	}

	acx = kzalloc(size, GFP_KERNEL);
	if (!acx) {
		cc33xx_warning("acx static calibration configuration "
				"process failed due to memory allocation "
				"failure");
		return -ENOMEM;
	}
	acx->valid_data = valid_data;

	if (!valid_data) {
		cc33xx_debug(DEBUG_ACX, "Sending empty calibration data to FW");
		goto out;
	}

	acx->file_version = file_header->file_version;
	acx->payload_struct_version = file_header->payload_struct_version;

	/* copy header & payload */
	memcpy(&(acx->calibration_header), fw_calibration_header,
	       sizeof(acx->calibration_header) + fw_calibration_header->length);

	cc33xx_debug(DEBUG_ACX, "acx calibration payload length: %d",
			acx->calibration_header.length);

out:
	ret = cc33xx_cmd_configure(wl, STATIC_CALIBRATION_CFG, acx, size);

	if (ret < 0)
		cc33xx_warning("acx command sending failed: %d", ret);

	kfree(acx);
	return ret;
}

int cc33xx_acx_tsf_info(struct cc33xx *wl, struct cc33xx_vif *wlvif,
			u64 *mactime)
{
	struct cc33xx_acx_fw_tsf_information *tsf_info;
	int ret;

	tsf_info = kzalloc(sizeof(*tsf_info), GFP_KERNEL);
	if (!tsf_info) {
		ret = -ENOMEM;
		goto out;
	}

	tsf_info->role_id = wlvif->role_id;

	ret = cc33xx_cmd_interrogate(wl, ACX_TSF_INFO, tsf_info,
				     sizeof(struct acx_header),
				     sizeof(*tsf_info));
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

int cc33xx_acx_ps_rx_streaming(struct cc33xx *wl,
			       struct cc33xx_vif *wlvif, bool enable)
{
	struct cc33xx_acx_ps_rx_streaming *rx_streaming;
	struct cc33xx_host_conf *host_conf = &wl->conf.host_conf;
	u32 conf_queues, enable_queues;
	int i, ret = 0;

	cc33xx_debug(DEBUG_ACX, "acx ps rx streaming");

	rx_streaming = kzalloc(sizeof(*rx_streaming), GFP_KERNEL);
	if (!rx_streaming) {
		ret = -ENOMEM;
		goto out;
	}

	conf_queues = host_conf->rx_streaming.queues;
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
		rx_streaming->period = host_conf->rx_streaming.interval;
		rx_streaming->timeout = host_conf->rx_streaming.interval;

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

int cc33xx_acx_ap_max_tx_retry(struct cc33xx *wl, struct cc33xx_vif *wlvif)
{
	struct cc33xx_acx_ap_max_tx_retry *acx = NULL;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx ap max tx retry");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx)
		return -ENOMEM;

	acx->role_id = wlvif->role_id;
	acx->max_tx_retry = cpu_to_le16(wl->conf.host_conf.tx.max_tx_retries);

	ret = cc33xx_cmd_configure(wl, ACX_MAX_TX_FAILURE, acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("acx ap max tx retry failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int cc33xx_acx_config_ps(struct cc33xx *wl, struct cc33xx_vif *wlvif)
{
	struct cc33xx_acx_config_ps *config_ps;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx config ps");

	config_ps = kzalloc(sizeof(*config_ps), GFP_KERNEL);
	if (!config_ps) {
		ret = -ENOMEM;
		goto out;
	}

	config_ps->exit_retries = wl->conf.host_conf.conn.psm_exit_retries;
	config_ps->enter_retries = wl->conf.host_conf.conn.psm_entry_retries;
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

int cc33xx_acx_set_inconnection_sta(struct cc33xx *wl,
				    struct cc33xx_vif *wlvif, u8 *addr)
{
	struct cc33xx_acx_inconnection_sta *acx = NULL;
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

int wlcore_acx_average_rssi(struct cc33xx *wl,
			    struct cc33xx_vif *wlvif, s8 *avg_rssi)
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

	ret = cc33xx_cmd_interrogate(wl, RSSI_INTR,
				     acx, sizeof(*acx), sizeof(*acx));
	if (ret	< 0) {
		cc33xx_warning("acx roaming statistics failed: %d", ret);
		ret = -ENOMEM;
		goto out;
	}

	*avg_rssi = acx->rssi_beacon;

out:
	kfree(acx);
	return ret;
}

int wlcore_acx_get_tx_rate(struct cc33xx *wl, struct cc33xx_vif *wlvif,
			    struct station_info *sinfo)
{
	struct acx_preamble_and_tx_rate *acx;
	int ret;
	cc33xx_debug(DEBUG_ACX, "acx get tx rate");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->role_id = wlvif->role_id;

	ret = cc33xx_cmd_interrogate(wl, GET_PREAMBLE_AND_TX_RATE_INTR,
				     acx, sizeof(*acx), sizeof(*acx));
	if (ret	< 0) {
		cc33xx_warning("acx get preamble and tx rate failed: %d", ret);
		ret = -ENOMEM;
		goto out;
	}

	//flags handler:
    sinfo->txrate.flags = 0;
	if(acx->preamble == CONF_PREAMBLE_TYPE_AC_VHT)
		sinfo->txrate.flags = RATE_INFO_FLAGS_VHT_MCS;
	else if((acx->preamble >= CONF_PREAMBLE_TYPE_AX_SU) && (acx->preamble <= CONF_PREAMBLE_TYPE_AX_TB_NDP_FB))
		sinfo->txrate.flags = RATE_INFO_FLAGS_HE_MCS;
    else if((acx->preamble == CONF_PREAMBLE_TYPE_N_MIXED_MODE) || (acx->preamble == CONF_PREAMBLE_TYPE_GREENFIELD)){
        sinfo->txrate.flags = RATE_INFO_FLAGS_MCS;
    }

	//mcs & legacy handler:
	if (acx->tx_rate >= CONF_HW_RATE_INDEX_MCS0)
		sinfo->txrate.mcs = acx->tx_rate - CONF_HW_RATE_INDEX_MCS0;
	else
		sinfo->txrate.legacy = cc33xx_idx_to_rate_100Kbps[acx->tx_rate -1];

    sinfo->txrate.nss = 1;
    sinfo->txrate.bw = RATE_INFO_BW_20;
    sinfo->txrate.he_gi = NL80211_RATE_INFO_HE_GI_3_2;
    sinfo->txrate.he_dcm = 0;
	sinfo->txrate.he_ru_alloc = 0;
    sinfo->txrate.n_bonded_ch = 0;
	sinfo->filled |= BIT_ULL(NL80211_STA_INFO_TX_BITRATE);
out:
	kfree(acx);
	return ret;

}

#ifdef CONFIG_PM
/* Set the global behaviour of RX filters - On/Off + default action */
int cc33xx_acx_default_rx_filter_enable(struct cc33xx *wl, bool enable,
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
	acx->special_packet_bitmask = 0;

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
int cc33xx_acx_set_rx_filter(struct cc33xx *wl, u8 index, bool enable,
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
		cc33xx_rx_filter_flatten_fields(filter, acx->fields);
	}

	cc33xx_dump(DEBUG_ACX, "RX_FILTER: ", acx, acx_size);

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

int cc33xx_acx_host_if_cfg_bitmap(struct cc33xx *wl, u32 host_cfg_bitmap,
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
		cc33xx_warning("cc33xx bitmap config opt failed: %d", ret);
		goto out;
	}

out:
	kfree(bitmap_conf);

	return ret;
}

int cc33xx_acx_peer_ht_operation_mode(struct cc33xx *wl, u8 hlid, bool wide)
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
 * this command is basically the same as cc33xx_acx_ht_capabilities,
 * with the addition of supported rates. they should be unified in
 * the next fw api change
 */
int cc33xx_acx_set_peer_cap(struct cc33xx *wl,
			    struct ieee80211_sta_ht_cap *ht_cap,
			    struct ieee80211_sta_he_cap *he_cap,
			    struct cc33xx_vif *wlvif, bool allow_ht_operation,
			    u32 rate_set, u8 hlid)
{
	struct wlcore_acx_peer_cap *acx;
	int ret = 0;
	u32 ht_capabilites = 0;
	u8 *cap_info = NULL;
	u8 dcm_max_const_rx_mask = IEEE80211_HE_PHY_CAP3_DCM_MAX_CONST_RX_MASK;
	u8 partial_bw_ext_range = IEEE80211_HE_PHY_CAP6_PARTIAL_BW_EXT_RANGE;

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
		ht_capabilites |= CC33XX_HT_CAP_HT_OPERATION;

		/* get data from A-MPDU parameters field */
		acx->ampdu_max_length = ht_cap->ampdu_factor;
		acx->ampdu_min_spacing = ht_cap->ampdu_density;
	}

	acx->ht_capabilites = cpu_to_le32(ht_capabilites);
	acx->supported_rates = cpu_to_le32(rate_set);

	acx->role_id = wlvif->role_id;
	acx->has_he = he_cap->has_he;
	memcpy(acx->mac_cap_info, he_cap->he_cap_elem.mac_cap_info, 6);
	cap_info = he_cap->he_cap_elem.phy_cap_info;
	acx->nominal_packet_padding = (cap_info[8] & NOMINAL_PACKET_PADDING);
	/* Max DCM constelation for RX - bits [4:3] in PHY capabilities byte 3 */
	acx->dcm_max_constelation = (cap_info[3] & dcm_max_const_rx_mask) >> 3;
	acx->er_upper_supported = ((cap_info[6] & partial_bw_ext_range) != 0);
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
int cc33xx_acx_interrupt_notify_config(struct cc33xx *wl, bool action)
{
	struct cc33xx_acx_interrupt_notify *acx;
	int ret = 0;

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->enable = cpu_to_le32(action);
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
int cc33xx_acx_rx_ba_filter(struct cc33xx *wl, bool action)
{
	struct cc33xx_acx_rx_ba_filter *acx;
	int ret = 0;

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->enable = cpu_to_le32(action);
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

int cc33xx_acx_ap_sleep(struct cc33xx *wl)
{
	struct acx_ap_sleep_cfg *acx;
	struct conf_ap_sleep_settings *conf = &wl->conf.host_conf.ap_sleep;
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

int cc33xx_acx_twt_setup(struct cc33xx *wl, u32 min_wake_duration_usec,
		       u32 min_wake_interval_mantissa, u32 min_wake_interval_exponent,
			   u32 max_wake_interval_mantissa, u32 max_wake_interval_exponent,
			   u8 valid_params)
{
	struct acx_twt_setup *acx;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx config twt setup. valid_params: %d, "
				"min_wake_duration_usec: %d, min_wake_interval_mantissa: %d, "
				"min_wake_interval_exponent: %d, max_wake_interval_mantissa: %d, "
				"max_wake_interval_exponent: %d",
				valid_params, min_wake_duration_usec,
				min_wake_interval_mantissa, min_wake_interval_exponent,
				max_wake_interval_mantissa, max_wake_interval_exponent);

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->min_wake_duration_usec = cpu_to_le32(min_wake_duration_usec);
	acx->min_wake_interval_mantissa = cpu_to_le32(min_wake_interval_mantissa);
	acx->min_wake_interval_exponent = cpu_to_le32(min_wake_interval_exponent);
	acx->max_wake_interval_mantissa = cpu_to_le32(max_wake_interval_mantissa);
	acx->max_wake_interval_exponent = cpu_to_le32(max_wake_interval_exponent);
	acx->valid_params = valid_params;

	ret = cc33xx_cmd_configure(wl, TWT_SETUP, acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("acx config twt setup failed: %d", ret);
		goto out;
	}

	wl->min_wake_duration_usec = min_wake_duration_usec;
	wl->min_wake_interval_mantissa = min_wake_interval_mantissa;
	wl->min_wake_interval_exponent = min_wake_interval_exponent;
	wl->max_wake_interval_mantissa = max_wake_interval_mantissa;
	wl->max_wake_interval_exponent = max_wake_interval_exponent;
out:
	kfree(acx);
	return ret;
}

int cc33xx_acx_twt_terminate(struct cc33xx *wl)
{
	struct acx_twt_terminate *acx;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx config twt terminate");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	ret = cc33xx_cmd_configure(wl, TWT_TERMINATE, acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("acx config twt terminate failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int cc33xx_acx_twt_suspend(struct cc33xx *wl)
{
	struct acx_twt_terminate *acx;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx config twt suspend");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	ret = cc33xx_cmd_configure(wl, TWT_SUSPEND, acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("acx config twt suspend failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int cc33xx_acx_twt_resume(struct cc33xx *wl)
{
	struct acx_twt_terminate *acx;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx config twt resume");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	ret = cc33xx_cmd_configure(wl, TWT_RESUME, acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("acx config twt resume failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}






















//Bashar
int cc33xx_acx_set_antenna_select(struct cc33xx *wl, u8 selection)
{
	struct acx_antenna_select *acx;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx setting antenna to %d", selection);

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->selection = selection;

	ret = cc33xx_cmd_configure(wl, SET_ANTENNA_SELECT_CFG,
				   acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("acx setting antenna failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int cc33xx_acx_set_tsf(struct cc33xx *wl, u64 tsf_val)
{
	struct debug_set_tsf *set_tsf_cmd;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx set tsf. new tsf value: %llx", tsf_val);

	set_tsf_cmd = kzalloc(sizeof(*set_tsf_cmd), GFP_KERNEL);
	if (!set_tsf_cmd) {
		ret = -ENOMEM;
		goto out;
	}

	set_tsf_cmd->tsf_val = cpu_to_le64(tsf_val);

	ret = cc33xx_cmd_debug(wl, SET_TSF, set_tsf_cmd, sizeof(*set_tsf_cmd));
	if (ret < 0) {
		cc33xx_error("acx set tsf failed: %d", ret);
		goto out;
	}

out:
	kfree(set_tsf_cmd);
	return ret;
}

int cc33xx_acx_trigger_fw_assert(struct cc33xx *wl)
{
	struct debug_header *buf;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx trigger firmware assert");

	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf) {
		ret = -ENOMEM;
		goto out;
	}

	ret = cc33xx_cmd_debug(wl, TRIGGER_FW_ASSERT, buf, sizeof(*buf));
	if (ret < 0) {
		cc33xx_error("failed to trigger firmware assert");
		goto out;
	}

out:
	kfree(buf);
	return ret;
}

int cc33xx_acx_burst_mode_cfg(struct cc33xx *wl, u8 burst_disable)
{
	struct debug_burst_mode_cfg *burst_mode_cfg;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx burst mode cfg. burst_disable = %d",
		     burst_disable);

	burst_mode_cfg = kzalloc(sizeof(*burst_mode_cfg), GFP_KERNEL);
	if (!burst_mode_cfg) {
		ret = -ENOMEM;
		goto out;
	}

	burst_mode_cfg->burst_disable = burst_disable;

	ret = cc33xx_cmd_debug(wl, BURST_MODE_CFG, burst_mode_cfg,
			       sizeof(*burst_mode_cfg));
	if (ret < 0) {
		cc33xx_warning("acx burst mode cfg failed: %d", ret);
		goto out;
	}

out:
	kfree(burst_mode_cfg);
	return ret;
}

int cc33xx_acx_antenna_diversity_enable(struct cc33xx *wl, u8 diversity_enable)
{
	struct acx_antenna_diversity_enable *antenna_diversity_enable;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx antenna diversity enable. diversity_enable = %d", diversity_enable);

	antenna_diversity_enable = kzalloc(sizeof(*antenna_diversity_enable), GFP_KERNEL);
	if (!antenna_diversity_enable) {
		ret = -ENOMEM;
		goto out;
	}

	antenna_diversity_enable->diversity_enable = diversity_enable;

	ret = cc33xx_cmd_configure(wl, ANT_DIV_ENABLE, antenna_diversity_enable,
							sizeof(*antenna_diversity_enable));
	if (ret < 0) {
		cc33xx_warning("acx antenna diversity enable failed: %d", ret);
		goto out;
	}

out:
	kfree(antenna_diversity_enable);
	return ret;
}

int cc33xx_acx_antenna_diversity_set_rssi_threshold(struct cc33xx *wl, s8 rssi_threshold)
{
	struct acx_antenna_diversity_rssi_threshold *set_rssi_threshold_cmd;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx antenna diversity set rssi threshold to %d",
					rssi_threshold);

	set_rssi_threshold_cmd = kzalloc(sizeof(*set_rssi_threshold_cmd), GFP_KERNEL);
	if (!set_rssi_threshold_cmd) {
		ret = -ENOMEM;
		goto out;
	}

	set_rssi_threshold_cmd->rssi_threshold = rssi_threshold;

	ret = cc33xx_cmd_configure(wl, ANT_DIV_SET_RSSI_THRESHOLD, set_rssi_threshold_cmd,
					sizeof(*set_rssi_threshold_cmd));
	if (ret < 0) {
		cc33xx_warning("acx antenna diversity set rssi threshold failed: %d", ret);
		goto out;
	}

out:
	kfree(set_rssi_threshold_cmd);
	return ret;
}

int cc33xx_acx_antenna_diversity_select_default_antenna(struct cc33xx *wl, u8 default_antenna)
{
	struct acx_antenna_diversity_select_default_antenna *select_default_antenna_cmd;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx antenna diversity select default antenna. default_antenna = %d", default_antenna);

	select_default_antenna_cmd = kzalloc(sizeof(*select_default_antenna_cmd), GFP_KERNEL);
	if (!select_default_antenna_cmd) {
		ret = -ENOMEM;
		goto out;
	}

	select_default_antenna_cmd->default_antenna = default_antenna;

	ret = cc33xx_cmd_configure(wl, ANT_DIV_SELECT_DEFAULT_ANTENNA, select_default_antenna_cmd, sizeof(*select_default_antenna_cmd));
	if (ret < 0) {
		cc33xx_warning("acx antenna diversity select default antenna failed: %d", ret);
		goto out;
	}

out:
	kfree(select_default_antenna_cmd);
	return ret;
}
