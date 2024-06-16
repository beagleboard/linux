// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022-2024 Texas Instruments Incorporated - https://www.ti.com/
 */

#include "acx.h"

int cc33xx_acx_clear_statistics(struct cc33xx *cc)
{
	struct acx_header *acx;
	int ret = 0;

	cc33xx_debug(DEBUG_ACX, "acx clear statistics");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	ret = cc33xx_cmd_configure(cc, ACX_CLEAR_STATISTICS, acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("failed to clear firmware statistics: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int cc33xx_acx_wake_up_conditions(struct cc33xx *cc, struct cc33xx_vif *wlvif,
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

	ret = cc33xx_cmd_configure(cc, WAKE_UP_CONDITIONS_CFG,
				   wake_up, sizeof(*wake_up));
	if (ret < 0) {
		cc33xx_warning("could not set wake up conditions: %d", ret);
		goto out;
	}

out:
	kfree(wake_up);
	return ret;
}

int cc33xx_acx_sleep_auth(struct cc33xx *cc, u8 sleep_auth)
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

	ret = cc33xx_cmd_configure(cc, ACX_SLEEP_AUTH, auth, sizeof(*auth));
	if (ret < 0) {
		cc33xx_error("could not configure sleep_auth to %d: %d",
			     sleep_auth, ret);
		goto out;
	}

	cc->sleep_auth = sleep_auth;
out:
	kfree(auth);
	return ret;
}

int cc33xx_ble_enable(struct cc33xx *cc, u8 ble_enable)
{
	struct debug_header *buf;
	int ret;

	cc33xx_debug(DEBUG_ACX, "ble enable");

	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf) {
		ret = -ENOMEM;
		goto out;
	}

	ret = cc33xx_cmd_debug(cc, BLE_ENABLE, buf, sizeof(*buf));
	if (ret < 0) {
		cc33xx_error("could not enable ble");
		goto out;
	}

	cc->ble_enable = 1;
out:
	kfree(buf);
	return ret;
}

int cc33xx_acx_tx_power(struct cc33xx *cc, struct cc33xx_vif *wlvif,
			int power)
{
	struct acx_tx_power_cfg *acx;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx TX_POWER_CFG %d", power);

	if (power < CC33XX_MIN_TXPWR) {
		cc33xx_warning("Configured Tx power %d dBm. Increasing to minimum %d dBm",
			       power, CC33XX_MIN_TXPWR);
		power = CC33XX_MIN_TXPWR;
	} else if (power > CC33XX_MAX_TXPWR) {
		cc33xx_warning("Configured Tx power %d dBm is bigger than upper limit: %d dBm. Attenuating to max limit",
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

	ret = cc33xx_cmd_configure(cc, TX_POWER_CFG, acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("Configure of tx power failed: %d", ret);
		goto out;
	}

	wlvif->power_level = power;

out:
	kfree(acx);
	return ret;
}

static int cc33xx_acx_mem_map(struct cc33xx *cc,
			      struct acx_header *memeroy_map, size_t len)
{
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx mem map");

	ret = cc33xx_cmd_interrogate(cc, MEM_MAP_INTR, memeroy_map,
				     sizeof(struct acx_header), len);
	if (ret < 0)
		return ret;

	return 0;
}

static int cc33xx_acx_get_fw_versions(struct cc33xx *cc,
				      struct cc33xx_acx_fw_versions *get_fw_versions,
				      size_t len)
{
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx get FW versions");

	ret = cc33xx_cmd_interrogate(cc, GET_FW_VERSIONS_INTR, get_fw_versions,
				     sizeof(struct cc33xx_acx_fw_versions), len);
	if (ret < 0)
		return ret;
	return 0;
}

int cc33xx_acx_slot(struct cc33xx *cc, struct cc33xx_vif *wlvif,
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
	ret = cc33xx_cmd_configure(cc, SLOT_CFG, slot, sizeof(*slot));

	if (ret < 0) {
		cc33xx_warning("failed to set slot time: %d", ret);
		goto out;
	}

out:
	kfree(slot);
	return ret;
}

int cc33xx_acx_group_address_tbl(struct cc33xx *cc, bool enable, void *mc_list, u32 mc_list_len)
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

	ret = cc33xx_cmd_configure(cc, DOT11_GROUP_ADDRESS_TBL,
				   acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("failed to set group addr table: %d", ret);
		goto out;
	}
out:
	kfree(acx);
	return ret;
}

int cc33xx_acx_beacon_filter_opt(struct cc33xx *cc, struct cc33xx_vif *wlvif,
				 bool enable_filter)
{
	struct acx_beacon_filter_option *beacon_filter = NULL;
	int ret = 0;

	cc33xx_debug(DEBUG_ACX, "acx beacon filter opt enable=%d",
		     enable_filter);

	if (enable_filter &&
	    cc->conf.host_conf.conn.bcn_filt_mode == CONF_BCN_FILT_MODE_DISABLED)
		goto out;

	beacon_filter = kzalloc(sizeof(*beacon_filter), GFP_KERNEL);
	if (!beacon_filter) {
		ret = -ENOMEM;
		goto out;
	}

	beacon_filter->role_id = wlvif->role_id;
	beacon_filter->enable = enable_filter;

	/* When set to zero, and the filter is enabled, beacons
	 * without the unicast TIM bit set are dropped.
	 */
	beacon_filter->max_num_beacons = 0;

	ret = cc33xx_cmd_configure(cc, BEACON_FILTER_OPT,
				   beacon_filter, sizeof(*beacon_filter));
	if (ret < 0) {
		cc33xx_warning("failed to set beacon filter opt: %d", ret);
		goto out;
	}

out:
	kfree(beacon_filter);
	return ret;
}

int cc33xx_acx_beacon_filter_table(struct cc33xx *cc, struct cc33xx_vif *wlvif)
{
	struct acx_beacon_filter_ie_table *ie_table;
	struct conf_bcn_filt_rule bcn_filt_ie[32];
	struct conf_bcn_filt_rule *p_bcn_filt_ie;
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
	p_bcn_filt_ie =  &cc->conf.host_conf.conn.bcn_filt_ie0;
	memcpy(bcn_filt_ie, p_bcn_filt_ie, 32 * sizeof(struct conf_bcn_filt_rule));
	for (i = 0; i < cc->conf.host_conf.conn.bcn_filt_ie_count; i++) {
		struct conf_bcn_filt_rule *r = &bcn_filt_ie[i];

		ie_table->table[idx++] = r->ie;
		ie_table->table[idx++] = r->rule;

		if (r->ie == WLAN_EID_VENDOR_SPECIFIC) {
			/* only one vendor specific ie allowed */
			if (vendor_spec)
				continue;

			/* for vendor specific rules configure the
			 * additional fields
			 */
			memcpy(&ie_table->table[idx], r->oui,
			       CONF_BCN_IE_OUI_LEN);
			idx += CONF_BCN_IE_OUI_LEN;
			ie_table->table[idx++] = r->type;
			memcpy(&ie_table->table[idx], r->version,
			       CONF_BCN_IE_VER_LEN);
			idx += CONF_BCN_IE_VER_LEN;
			vendor_spec = true;
		}

		ie_table->num_ie++;
	}

	ret = cc33xx_cmd_configure(cc, BEACON_FILTER_TABLE,
				   ie_table, sizeof(*ie_table));
	if (ret < 0) {
		cc33xx_warning("failed to set beacon filter table: %d", ret);
		goto out;
	}

out:
	kfree(ie_table);
	return ret;
}

int cc33xx_assoc_info_cfg(struct cc33xx *cc, struct cc33xx_vif *wlvif,
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
	ret = cc33xx_cmd_configure(cc, ASSOC_INFO_CFG, cfg, sizeof(*cfg));
	if (ret < 0) {
		cc33xx_warning("failed to set aid: %d", ret);
		goto out;
	}

out:
	kfree(cfg);
	return ret;
}

int cc33xx_acx_set_preamble(struct cc33xx *cc, struct cc33xx_vif *wlvif,
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

	ret = cc33xx_cmd_configure(cc, PREAMBLE_TYPE_CFG, acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("Setting of preamble failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int cc33xx_acx_cts_protect(struct cc33xx *cc, struct cc33xx_vif *wlvif,
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

	ret = cc33xx_cmd_configure(cc, CTS_PROTECTION_CFG, acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("Setting of ctsprotect failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int cc33xx_acx_statistics(struct cc33xx *cc, void *stats)
{
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx statistics");

	ret = cc33xx_cmd_interrogate(cc, ACX_STATISTICS, stats,
				     sizeof(struct acx_header),
				     sizeof(struct cc33xx_acx_statistics));
	if (ret < 0) {
		cc33xx_warning("acx statistics failed: %d", ret);
		return -ENOMEM;
	}

	return 0;
}

int cc33xx_update_ap_rates(struct cc33xx *cc, u8 role_id,
			   u32 basic_rates_set, u32 supported_rates)
{
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
	ret = cc33xx_cmd_configure(cc, AP_RATES_CFG, cfg, sizeof(*cfg));
	if (ret < 0) {
		cc33xx_warning("Updating AP Rates  failed: %d", ret);
		goto out;
	}

out:
	kfree(cfg);
	return ret;
}

int cc33xx_tx_param_cfg(struct cc33xx *cc, struct cc33xx_vif *wlvif, u8 ac,
			u8 cw_min, u16 cw_max, u8 aifsn, u16 txop, bool acm,
			u8 ps_scheme, u8 is_mu_edca, u8 mu_edca_aifs,
			u8 mu_edca_ecw_min_max, u8 mu_edca_timer)
{
	struct tx_param_cfg *cfg;
	int ret = 0;

	cc33xx_debug(DEBUG_ACX,
		     "tx param cfg %d cw_ming %d cw_max %d aifs %d txop %d",
		     ac, cw_min, cw_max, aifsn, txop);

	cc33xx_debug(DEBUG_ACX, "tx param cfg ps_scheme %d is_mu_edca %d mu_edca_aifs %d mu_edca_ecw_min_max %d mu_edca_timer %d",
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

	ret = cc33xx_cmd_configure(cc, TX_PARAMS_CFG, cfg, sizeof(*cfg));
	if (ret < 0) {
		cc33xx_warning("tx param cfg failed: %d", ret);
		goto out;
	}

out:
	kfree(cfg);
	return ret;
}

int cc33xx_acx_init_mem_config(struct cc33xx *cc)
{
	int ret;

	cc->target_mem_map = kzalloc(sizeof(*cc->target_mem_map),
				     GFP_KERNEL);
	if (!cc->target_mem_map) {
		cc33xx_error("couldn't allocate target memory map");
		return -ENOMEM;
	}

	/* we now ask for the firmware built memory map */
	ret = cc33xx_acx_mem_map(cc, (void *)cc->target_mem_map,
				 sizeof(struct cc33xx_acx_mem_map));
	if (ret < 0) {
		cc33xx_error("couldn't retrieve firmware memory map");
		kfree(cc->target_mem_map);
		cc->target_mem_map = NULL;
		return ret;
	}

	/* initialize TX block book keeping */
	cc->tx_blocks_available =
		le32_to_cpu(cc->target_mem_map->num_tx_mem_blocks);
	cc33xx_debug(DEBUG_TX, "available tx blocks: %d",
		     cc->tx_blocks_available);

	cc33xx_debug(DEBUG_TX,
		     "available tx descriptor: %d available rx blocks %d",
		     cc->target_mem_map->num_tx_descriptor,
		     cc->target_mem_map->num_rx_mem_blocks);

	return 0;
}

int cc33xx_acx_init_get_fw_versions(struct cc33xx *cc)
{
	int ret;

	cc->fw_ver = kzalloc(sizeof(*cc->fw_ver),
			     GFP_KERNEL);
	if (!cc->fw_ver) {
		cc33xx_error("couldn't allocate cc33xx_acx_fw_versions");
		return -ENOMEM;
	}

	ret = cc33xx_acx_get_fw_versions(cc, (void *)cc->fw_ver,
					 sizeof(struct cc33xx_acx_fw_versions));
	if (ret < 0) {
		cc33xx_error("couldn't retrieve firmware versions");
		kfree(cc->fw_ver);
		cc->fw_ver = NULL;
		return ret;
	}

	return 0;
}

int cc33xx_acx_set_ht_information(struct cc33xx *cc, struct cc33xx_vif *wlvif,
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
	acx->qos_info_more_data_ack_bit = 0;
	ret = cc33xx_cmd_configure(cc, BSS_OPERATION_CFG, acx, sizeof(*acx));

	if (ret < 0) {
		cc33xx_warning("acx ht information setting failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

/* setup BA session receiver setting in the FW. */
int cc33xx_acx_set_ba_receiver_session(struct cc33xx *cc, u8 tid_index, u16 ssn,
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

	ret = cc33xx_cmd_configure_failsafe(cc, BA_SESSION_RX_SETUP_CFG,
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

int cc33xx_acx_tsf_info(struct cc33xx *cc,
			struct cc33xx_vif *wlvif, u64 *mactime)
{
	struct cc33xx_acx_fw_tsf_information *tsf_info;
	int ret = 0;

	tsf_info = kzalloc(sizeof(*tsf_info), GFP_KERNEL);
	if (!tsf_info) {
		ret = -ENOMEM;
		goto out;
	}

	tsf_info->role_id = wlvif->role_id;

	*mactime = le32_to_cpu(tsf_info->current_tsf_low) |
		((u64)le32_to_cpu(tsf_info->current_tsf_high) << 32);

out:
	kfree(tsf_info);
	return ret;
}

int cc33xx_acx_config_ps(struct cc33xx *cc, struct cc33xx_vif *wlvif)
{
	struct cc33xx_acx_config_ps *config_ps;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx config ps");

	config_ps = kzalloc(sizeof(*config_ps), GFP_KERNEL);
	if (!config_ps) {
		ret = -ENOMEM;
		goto out;
	}

	config_ps->exit_retries = cc->conf.host_conf.conn.psm_exit_retries;
	config_ps->enter_retries = cc->conf.host_conf.conn.psm_entry_retries;
	config_ps->null_data_rate = cpu_to_le32(wlvif->basic_rate);

	ret = cc33xx_cmd_configure(cc, ACX_CONFIG_PS, config_ps,
				   sizeof(*config_ps));

	if (ret < 0) {
		cc33xx_warning("acx config ps failed: %d", ret);
		goto out;
	}

out:
	kfree(config_ps);
	return ret;
}

int cc33xx_acx_average_rssi(struct cc33xx *cc,
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

	ret = cc33xx_cmd_interrogate(cc, RSSI_INTR,
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

static const u16 cc33xx_idx_to_rate_100kbps[] = {
	10, 20, 55, 110, 60, 90, 120, 180, 240, 360, 480, 540
};

int cc33xx_acx_get_tx_rate(struct cc33xx *cc, struct cc33xx_vif *wlvif,
			   struct station_info *sinfo)
{
	struct acx_preamble_and_tx_rate *acx;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx set tx rate");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->role_id = wlvif->role_id;

	ret = cc33xx_cmd_interrogate(cc, GET_PREAMBLE_AND_TX_RATE_INTR,
				     acx, sizeof(*acx), sizeof(*acx));
	if (ret	< 0) {
		cc33xx_warning("acx get preamble and tx rate failed: %d", ret);
		ret = -ENOMEM;
		goto out;
	}

	sinfo->txrate.flags = 0;
	if (acx->preamble == CONF_PREAMBLE_TYPE_AC_VHT)
		sinfo->txrate.flags = RATE_INFO_FLAGS_VHT_MCS;
	else if ((acx->preamble >= CONF_PREAMBLE_TYPE_AX_SU) &&
		 (acx->preamble <= CONF_PREAMBLE_TYPE_AX_TB_NDP_FB))
		sinfo->txrate.flags = RATE_INFO_FLAGS_HE_MCS;
	else if ((acx->preamble == CONF_PREAMBLE_TYPE_N_MIXED_MODE) ||
		 (acx->preamble == CONF_PREAMBLE_TYPE_GREENFIELD))
		sinfo->txrate.flags = RATE_INFO_FLAGS_MCS;

	if (acx->tx_rate >= CONF_HW_RATE_INDEX_MCS0)
		sinfo->txrate.mcs = acx->tx_rate - CONF_HW_RATE_INDEX_MCS0;
	else
		sinfo->txrate.legacy = cc33xx_idx_to_rate_100kbps[acx->tx_rate - 1];

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
int cc33xx_acx_default_rx_filter_enable(struct cc33xx *cc, bool enable,
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

	ret = cc33xx_cmd_configure(cc, ACX_ENABLE_RX_DATA_FILTER, acx,
				   sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("acx default rx filter enable failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

static int cc33xx_rx_filter_get_fields_size(struct cc33xx_rx_filter *filter)
{
	int i, fields_size = 0;

	for (i = 0; i < filter->num_fields; i++) {
		fields_size += filter->fields[i].len - sizeof(u8 *)
					+ sizeof(struct cc33xx_rx_filter_field);
	}

	return fields_size;
}

static void cc33xx_rx_filter_flatten_fields(struct cc33xx_rx_filter *filter,
					    u8 *buf)
{
	int i;
	struct cc33xx_rx_filter_field *field;

	for (i = 0; i < filter->num_fields; i++) {
		field = (struct cc33xx_rx_filter_field *)buf;

		field->offset = filter->fields[i].offset;
		field->flags = filter->fields[i].flags;
		field->len = filter->fields[i].len;

		memcpy(&field->pattern, filter->fields[i].pattern, field->len);
		buf += sizeof(struct cc33xx_rx_filter_field) - sizeof(u8 *);
		buf += field->len;
	}
}

/* Configure or disable a specific RX filter pattern */
int cc33xx_acx_set_rx_filter(struct cc33xx *cc, u8 index, bool enable,
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

	ret = cc33xx_cmd_configure(cc, ACX_SET_RX_DATA_FILTER, acx, acx_size);
	if (ret < 0) {
		cc33xx_warning("setting rx filter failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}
#endif /* CONFIG_PM */

/* this command is basically the same as cc33xx_acx_ht_capabilities,
 * with the addition of supported rates. they should be unified in
 * the next fw api change
 */
int cc33xx_acx_set_peer_cap(struct cc33xx *cc,
			    struct ieee80211_sta_ht_cap *ht_cap,
			    struct ieee80211_sta_he_cap *he_cap,
			    struct cc33xx_vif *wlvif, bool allow_ht_operation,
			    u32 rate_set, u8 hlid)
{
	struct cc33xx_acx_peer_cap *acx;
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

		/* this bit is not employed by the spec but only by FW to
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
	ret = cc33xx_cmd_configure(cc, PEER_CAP_CFG, acx, sizeof(*acx));

	if (ret < 0) {
		cc33xx_warning("acx ht capabilities setting failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int cc33xx_acx_trigger_fw_assert(struct cc33xx *cc)
{
	struct debug_header *buf;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx trigger firmware assert");

	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf) {
		ret = -ENOMEM;
		goto out;
	}

	ret = cc33xx_cmd_debug(cc, TRIGGER_FW_ASSERT, buf, sizeof(*buf));
	if (ret < 0) {
		cc33xx_error("failed to trigger firmware assert");
		goto out;
	}

out:
	kfree(buf);
	return ret;
}
