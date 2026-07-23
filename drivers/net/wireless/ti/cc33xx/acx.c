// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022-2024 Texas Instruments Incorporated - https://www.ti.com/
 */

#include "acx.h"

int cc33xx_acx_wake_up_conditions(struct cc33xx *cc, struct cc33xx_vif *wlvif,
				  u8 wake_up_event, u8 listen_interval)
{
	struct acx_wake_up_condition *wake_up;
	int ret;

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
	struct conf_bcn_filt_rule *r = &cc->conf.host_conf.conn.bcn_filt_ie0;
	struct conf_bcn_filt_rule *itr_end;
	int idx = 0;
	int ret;
	bool vendor_spec = false;

	if (WARN_ON(cc->conf.host_conf.conn.bcn_filt_ie_count > CONF_MAX_BCN_FILT_IE_COUNT))
		return -EINVAL;

	itr_end = r + cc->conf.host_conf.conn.bcn_filt_ie_count;

	ie_table = kzalloc(sizeof(*ie_table), GFP_KERNEL);
	if (!ie_table) {
		ret = -ENOMEM;
		goto out;
	}

	/* configure default beacon pass-through rules */
	ie_table->role_id = wlvif->role_id;
	ie_table->num_ie = 0;

	while (r < itr_end) {
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
		r++;
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

int cc33xx_update_ap_rates(struct cc33xx *cc, u8 role_id,
			   u32 basic_rates_set, u32 supported_rates)
{
	struct ap_rates_class_cfg *cfg;
	int ret;

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

int cc33xx_acx_get_slow_clock_type(struct cc33xx *cc)
{
	struct acx_slow_clk_type *acx;
	int ret = 0;

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx)
		ret = -ENOMEM;

	ret = cc33xx_cmd_interrogate(cc, GET_SLOW_CLK_SOURCE, acx,
			sizeof(struct acx_header), sizeof(struct acx_slow_clk_type));
	cc->is_ext_slw_clk = acx->is_ext_slw_clk;

	kfree(acx);
	return ret;
}

int cc33xx_acx_set_ht_information(struct cc33xx *cc, struct cc33xx_vif *wlvif,
				  u16 ht_operation_mode, u32 he_oper_params,
				  u16 he_oper_nss_set)
{
	struct cc33xx_acx_ht_information *acx;
	int ret = 0;

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

int cc33xx_acx_average_rssi(struct cc33xx *cc,
			    struct cc33xx_vif *wlvif, s8 *avg_rssi)
{
	struct acx_roaming_stats *acx;
	int ret = 0;

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
/* Configure ARP IP - send IP to add, or 0.0.0.0 to remove */
int cc33xx_acx_arp_ip_config(struct cc33xx *cc, u8 role_id, __be32 ip_addr)
{
	struct acx_arp_ip_cfg *acx;
	int ret;


	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->role_id = role_id;

	if (ip_addr) {
		memcpy(acx->address, &ip_addr, ACX_IPV4_ADDR_LENGTH);
		cc33xx_debug(DEBUG_ACX,
			     "acx arp ip filter: SETTING role=%d, IP=%pI4", role_id, &ip_addr);
	} else {
		memset(acx->address, 0, ACX_IPV4_ADDR_LENGTH);
		cc33xx_debug(DEBUG_ACX,
			     "acx arp ip filter: CLEARING role=%d, IP=0.0.0.0", role_id);
	}

	ret = cc33xx_cmd_configure(cc, ACX_ARP_IP_CFG, acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("acx arp ip filter failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

/* Enable/disable ARP offload feature */
int cc33xx_acx_arp_offload(struct cc33xx *cc, bool enable)
{
	struct acx_arp_offload *acx;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx arp offload enable: %d", enable);

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx)
		return -ENOMEM;

	acx->enable = enable;

	ret = cc33xx_cmd_configure(cc, ACX_ARP_OFFLOAD_ENABLE, acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("acx arp offload failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

/* Set the global behaviour of RX filters - On/Off + default action */
int cc33xx_acx_default_rx_filter_enable(struct cc33xx *cc, bool enable,
					enum rx_filter_action action)
{
	struct acx_default_rx_filter *acx;
	int ret;

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
	u8 data_len;

	for (i = 0; i < filter->num_fields; i++) {
		data_len = (filter->fields[i].flags & CC33XX_RX_FILTER_FLAG_MASKED) ?
		           filter->fields[i].len * 2 : filter->fields[i].len;
		fields_size += RX_FILTER_FIELD_OVERHEAD + data_len;
	}

	return fields_size;
}

static void cc33xx_rx_filter_flatten_fields(struct cc33xx_rx_filter *filter,
					    u8 *buf)
{
	int i;
	struct cc33xx_rx_filter_field *field;
	u8 data_len;

	for (i = 0; i < filter->num_fields; i++) {
		field = (struct cc33xx_rx_filter_field *)buf;

		field->offset = filter->fields[i].offset;
		field->flags = filter->fields[i].flags;
		field->len = filter->fields[i].len;

		data_len = (filter->fields[i].flags & CC33XX_RX_FILTER_FLAG_MASKED) ?
		           filter->fields[i].len * 2 : filter->fields[i].len;
		buf += sizeof(struct cc33xx_rx_filter_field) - sizeof(u8 *);
		buf += data_len;
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

	if (enable)
		fields_size = cc33xx_rx_filter_get_fields_size(filter);

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
				struct ieee80211_sta_vht_cap *vht_cap,
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
	acx->vht_capabilities = cpu_to_le32(vht_cap->cap);
	acx->vht_supported = vht_cap->vht_supported;
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

int cc33xx_acx_set_antenna_select(struct cc33xx *cc, u8 selection)
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

	ret = cc33xx_cmd_configure(cc, SET_ANTENNA_SELECT_CFG,
				   acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("acx setting antenna failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int cc33xx_acx_set_regdoamin_and_tx_control_params(struct cc33xx *cc, struct acx_phy_regdomain_tx_control_params *params)
{
	struct acx_phy_regdomain_tx_control_params *acx;
	int ret;

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->bitmask = params->bitmask;
	acx->country_code = params->country_code;
	acx->reg_domain   = params->reg_domain;

	memcpy(acx->ble_ch_lim_1M, params->ble_ch_lim_1M, sizeof(u8) * BLE_LIM_CHANNELS_COUNT);
	memcpy(acx->ble_ch_lim_2M, params->ble_ch_lim_2M, sizeof(u8) * BLE_LIM_CHANNELS_COUNT);

	memcpy(acx->per_channel_power_limit, params->per_channel_power_limit, sizeof(u8) * REG_RULES_COUNT);


	ret = cc33xx_cmd_configure(cc, PHY_REGDOMAIN_TX_POWER_PARAMS,
				   acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("acx regdoamin_and_tx_control_params failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int cc33xx_acx_set_tsf(struct cc33xx *cc, u64 tsf_val)
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

	ret = cc33xx_cmd_debug(cc, SET_TSF, set_tsf_cmd, sizeof(*set_tsf_cmd));
	if (ret < 0) {
		cc33xx_error("acx set tsf failed: %d", ret);
		goto out;
	}

out:
	kfree(set_tsf_cmd);
	return ret;
}

int cc33xx_acx_trigger_fw_assert(struct cc33xx *cc)
{
	struct debug_header *buf;
	int ret;

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

int cc33xx_acx_burst_mode_cfg(struct cc33xx *cc, u8 burst_disable)
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

	ret = cc33xx_cmd_debug(cc, BURST_MODE_CFG, burst_mode_cfg,
			       sizeof(*burst_mode_cfg));
	if (ret < 0) {
		cc33xx_warning("acx burst mode cfg failed: %d", ret);
		goto out;
	}

out:
	kfree(burst_mode_cfg);
	return ret;
}

int cc33xx_acx_get_antenna_diversity_status(struct cc33xx *cc)
{
	struct acx_diversity_status *get_diversity_status_cmd;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx get antenna diversity status");

	get_diversity_status_cmd = kzalloc(sizeof(*get_diversity_status_cmd), GFP_KERNEL);
	if (!get_diversity_status_cmd) {
		ret = -ENOMEM;
		goto out;
	}

	ret = cc33xx_cmd_interrogate(cc, GET_ANT_DIV_STATUS, get_diversity_status_cmd,
				sizeof(struct acx_header), sizeof(*get_diversity_status_cmd));
	if (ret < 0) {
		cc33xx_warning("acx get antenna diversity status failed: %d", ret);
		goto out;
	}

	ret = get_diversity_status_cmd->enable;

out:
	kfree(get_diversity_status_cmd);
	return ret;
}

int cc33xx_acx_set_antenna_diversity_status(struct cc33xx *cc, u8 enable)
{
	struct acx_diversity_status *set_diversity_status_cmd;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx set antenna diversity status. enable = %d", enable);

	set_diversity_status_cmd = kzalloc(sizeof(*set_diversity_status_cmd), GFP_KERNEL);
	if (!set_diversity_status_cmd) {
		ret = -ENOMEM;
		goto out;
	}

	set_diversity_status_cmd->enable = enable;

	ret = cc33xx_cmd_configure(cc, ANT_DIV_ENABLE, set_diversity_status_cmd,
							sizeof(*set_diversity_status_cmd));
	if (ret < 0) {
		cc33xx_warning("acx set antenna diversity status failed: %d", ret);
		goto out;
	}

out:
	kfree(set_diversity_status_cmd);
	return ret;
}

int cc33xx_acx_antenna_diversity_get_rssi_threshold(struct cc33xx *cc, s8 *threshold)
{
	struct acx_diversity_rssi_threshold *get_rssi_threshold_cmd;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx antenna diversity get rssi threshold");

	get_rssi_threshold_cmd = kzalloc(sizeof(*get_rssi_threshold_cmd), GFP_KERNEL);
	if (!get_rssi_threshold_cmd) {
		ret = -ENOMEM;
		goto out;
	}

	ret = cc33xx_cmd_interrogate(cc, GET_ANT_DIV_RSSI_THRESHOLD, get_rssi_threshold_cmd,
				sizeof(struct acx_header), sizeof(*get_rssi_threshold_cmd));
	if (ret < 0) {
		cc33xx_warning("acx antenna diversity get rssi threshold failed: %d", ret);
		goto out;
	}

	*threshold = get_rssi_threshold_cmd->rssi_threshold;
	ret = 0;

out:
	kfree(get_rssi_threshold_cmd);
	return ret;
}

int cc33xx_acx_antenna_diversity_set_rssi_threshold(struct cc33xx *cc, s8 rssi_threshold)
{
	struct acx_diversity_rssi_threshold *set_rssi_threshold_cmd;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx antenna diversity set rssi threshold to %d",
					rssi_threshold);

	set_rssi_threshold_cmd = kzalloc(sizeof(*set_rssi_threshold_cmd), GFP_KERNEL);
	if (!set_rssi_threshold_cmd) {
		ret = -ENOMEM;
		goto out;
	}

	set_rssi_threshold_cmd->rssi_threshold = rssi_threshold;

	ret = cc33xx_cmd_configure(cc, ANT_DIV_SET_RSSI_THRESHOLD, set_rssi_threshold_cmd,
					sizeof(*set_rssi_threshold_cmd));
	if (ret < 0) {
		cc33xx_warning("acx antenna diversity set rssi threshold failed: %d", ret);
		goto out;
	}

out:
	kfree(set_rssi_threshold_cmd);
	return ret;
}

int cc33xx_acx_antenna_diversity_get_default_antenna(struct cc33xx *cc)
{
	struct acx_diversity_default_antenna *get_default_antenna_cmd;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx antenna diversity get default antenna");

	get_default_antenna_cmd = kzalloc(sizeof(*get_default_antenna_cmd), GFP_KERNEL);
	if (!get_default_antenna_cmd) {
		ret = -ENOMEM;
		goto out;
	}

	ret = cc33xx_cmd_interrogate(cc, GET_ANT_DIV_DEFAULT_ANTENNA, get_default_antenna_cmd,
				sizeof(struct acx_header), sizeof(*get_default_antenna_cmd));
	if (ret < 0) {
		cc33xx_warning("acx antenna diversity get default antenna failed: %d", ret);
		goto out;
	}

	ret = get_default_antenna_cmd->default_antenna;

out:
	kfree(get_default_antenna_cmd);
	return ret;
}

int cc33xx_acx_antenna_diversity_select_default_antenna(struct cc33xx *cc, u8 default_antenna)
{
	struct acx_diversity_default_antenna *select_default_antenna_cmd;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx antenna diversity select default antenna. default_antenna = %d", default_antenna);

	select_default_antenna_cmd = kzalloc(sizeof(*select_default_antenna_cmd), GFP_KERNEL);
	if (!select_default_antenna_cmd) {
		ret = -ENOMEM;
		goto out;
	}

	select_default_antenna_cmd->default_antenna = default_antenna;

	ret = cc33xx_cmd_configure(cc, ANT_DIV_SELECT_DEFAULT_ANTENNA, select_default_antenna_cmd, sizeof(*select_default_antenna_cmd));
	if (ret < 0) {
		cc33xx_warning("acx antenna diversity select default antenna failed: %d", ret);
		goto out;
	}

out:
	kfree(select_default_antenna_cmd);
	return ret;
}

int cc33xx_acx_cqm_rssi_config(struct cc33xx *cc, struct cc33xx_vif *wlvif, bool enable, s8 threshold, u8 hysteresis)
{
	struct acx_cqm_rssi_config *cqm_params;
	int ret = 0;
	int role_id = wlvif->role_id;

	cc33xx_debug(DEBUG_CMD, "CQM config: role=%u, enable=%d, threshold=%d dBm, hysteresis=%d",
		     role_id, enable, threshold, hysteresis);

	if (wlvif && enable) {
		wlvif->last_rssi_event = -1;
		cc33xx_info("CQM: Reset last_rssi_event for role %u", role_id);
	}

	cqm_params = kzalloc(sizeof(*cqm_params), GFP_KERNEL);
	if (!cqm_params) {
		cc33xx_error("CQM: Failed to allocate command buffer");
		return -ENOMEM;
	}

	cqm_params->role_id = role_id;
	cqm_params->enable = enable;
	cqm_params->threshold_dbm = threshold;
	cqm_params->hysteresis_db = hysteresis;

	ret = cc33xx_cmd_configure(cc, CQM_RSSI_CONFIG, cqm_params, sizeof(*cqm_params));
	if (ret < 0) {
		cc33xx_error("CQM: Failed to send configure command: %d", ret);
	}

	kfree(cqm_params);
	return ret;
}

int cc33xx_acx_statistics(struct cc33xx *cc, void *stats)
{
	int ret;

	ret = cc33xx_cmd_interrogate(cc, GET_STATISTICS, stats,
				     sizeof(struct acx_header),
				     sizeof(struct cc33xx_acx_statistics));
	if (ret < 0) {
		cc33xx_warning("acx statistics failed: %d", ret);
		return -ENOMEM;
	}

	return 0;
}

int cc33xx_acx_clear_statistics(struct cc33xx *cc)
{
	struct acx_header *acx;
	int ret;

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx)
		return -ENOMEM;

	ret = cc33xx_cmd_configure(cc, RESET_STATS,
				   acx, sizeof(*acx));
	if (ret < 0)
		cc33xx_warning("clear stats failed: %d", ret);

	kfree(acx);
	return ret;
}

int cc33xx_acx_twt_setup(struct cc33xx *cc, u32 min_wake_duration_usec,
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

	ret = cc33xx_cmd_configure(cc, TWT_SETUP, acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("acx config twt setup failed: %d", ret);
		goto out;
	}

	cc->min_wake_duration_usec = min_wake_duration_usec;
	cc->min_wake_interval_mantissa = min_wake_interval_mantissa;
	cc->min_wake_interval_exponent = min_wake_interval_exponent;
	cc->max_wake_interval_mantissa = max_wake_interval_mantissa;
	cc->max_wake_interval_exponent = max_wake_interval_exponent;
out:
	kfree(acx);
	return ret;
}

int cc33xx_acx_twt_terminate(struct cc33xx *cc)
{
	struct acx_twt_terminate *acx;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx config twt terminate");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	ret = cc33xx_cmd_configure(cc, TWT_TERMINATE, acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("acx config twt terminate failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int cc33xx_acx_twt_suspend(struct cc33xx *cc)
{
	struct acx_twt_terminate *acx;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx config twt suspend");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	ret = cc33xx_cmd_configure(cc, TWT_SUSPEND, acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("acx config twt suspend failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int cc33xx_acx_twt_resume(struct cc33xx *cc)
{
	struct acx_twt_terminate *acx;
	int ret;

	cc33xx_debug(DEBUG_ACX, "acx config twt resume");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	ret = cc33xx_cmd_configure(cc, TWT_RESUME, acx, sizeof(*acx));
	if (ret < 0) {
		cc33xx_warning("acx config twt resume failed: %d", ret);
		goto out;
	}

out:

	kfree(acx);
	return ret;
}
