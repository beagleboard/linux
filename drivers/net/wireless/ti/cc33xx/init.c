// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022-2024 Texas Instruments Incorporated - https://www.ti.com/
 */

#include <linux/firmware.h>
#include "acx.h"
#include "cmd.h"
#include "conf.h"
#include "event.h"
#include "tx.h"
#include "init.h"

static int cc33xx_init_phy_vif_config(struct cc33xx *cc,
				      struct cc33xx_vif *wlvif)
{
	int ret;

	ret = cc33xx_acx_slot(cc, wlvif, DEFAULT_SLOT_TIME);
	if (ret < 0)
		return ret;

	return 0;
}

static int cc33xx_init_sta_beacon_filter(struct cc33xx *cc,
					 struct cc33xx_vif *wlvif)
{
	int ret;

	ret = cc33xx_acx_beacon_filter_table(cc, wlvif);
	if (ret < 0)
		return ret;

	/* disable beacon filtering until we get the first beacon */
	ret = cc33xx_acx_beacon_filter_opt(cc, wlvif, false);
	if (ret < 0)
		return ret;

	return 0;
}

static int cc33xx_ap_init_templates(struct cc33xx *cc,
				    struct ieee80211_vif *vif)
{
	struct cc33xx_vif *wlvif = cc33xx_vif_to_data(vif);
	int ret;

	/* when operating as AP we want to receive external beacons for
	 * configuring ERP protection.
	 */
	ret = cc33xx_acx_beacon_filter_opt(cc, wlvif, false);
	if (ret < 0)
		return ret;

	return 0;
}

static void cc33xx_set_ba_policies(struct cc33xx *cc, struct cc33xx_vif *wlvif)
{
	/* Reset the BA RX indicators */
	wlvif->ba_allowed = true;
	cc->ba_rx_session_count = 0;

	/* BA is supported in STA/AP modes */
	wlvif->ba_support = (wlvif->bss_type != BSS_TYPE_AP_BSS &&
				wlvif->bss_type != BSS_TYPE_STA_BSS);
}

/* vif-specifc initialization */
static int cc33xx_init_sta_role(struct cc33xx *cc, struct cc33xx_vif *wlvif)
{
	int ret = cc33xx_acx_group_address_tbl(cc, true, NULL, 0);

	if (ret < 0)
		return ret;

	/* Beacon filtering */
	ret = cc33xx_init_sta_beacon_filter(cc, wlvif);
	if (ret < 0)
		return ret;

	return 0;
}

/* vif-specific initialization */
static int cc33xx_init_ap_role(struct cc33xx *cc, struct cc33xx_vif *wlvif)
{
	int ret;

	/* initialize Tx power */
	ret = cc33xx_acx_tx_power(cc, wlvif, wlvif->power_level);
	if (ret < 0)
		return ret;

	if (cc->radar_debug_mode)
		cc33xx_cmd_generic_cfg(cc, wlvif,
				       CC33XX_CFG_FEATURE_RADAR_DEBUG,
				       cc->radar_debug_mode, 0);

	return 0;
}

int cc33xx_init_vif_specific(struct cc33xx *cc, struct ieee80211_vif *vif)
{
	struct cc33xx_vif *wlvif = cc33xx_vif_to_data(vif);
	struct conf_tx_ac_category *conf_ac;
	struct conf_tx_ac_category ac_conf[4];
	struct conf_tx_tid tid_conf[8];
	struct conf_tx_settings *tx_settings = &cc->conf.host_conf.tx;
	struct conf_tx_ac_category *p_wl_host_ac_conf = &tx_settings->ac_conf0;
	struct conf_tx_tid *p_wl_host_tid_conf = &tx_settings->tid_conf0;
	bool is_ap = (wlvif->bss_type == BSS_TYPE_AP_BSS);
	u8 ps_scheme = cc->conf.mac.ps_scheme;
	int ret, i;

	/* consider all existing roles before configuring psm. */

	if (cc->ap_count == 0 && is_ap) { /* first AP */
		ret = cc33xx_acx_sleep_auth(cc, CC33XX_PSM_ELP);
		if (ret < 0)
			return ret;

		/* unmask ap events */
		cc->event_mask |= cc->ap_event_mask;

	/* first STA, no APs */
	} else if (cc->sta_count == 0 && cc->ap_count == 0 && !is_ap) {
		u8 sta_auth = cc->conf.host_conf.conn.sta_sleep_auth;
		/* Configure for power according to debugfs */
		if (sta_auth != CC33XX_PSM_ILLEGAL)
			ret = cc33xx_acx_sleep_auth(cc, sta_auth);
		/* Configure for ELP power saving */
		else
			ret = cc33xx_acx_sleep_auth(cc, CC33XX_PSM_ELP);

		if (ret < 0)
			return ret;
	}

	/* Mode specific init */
	if (is_ap) {
		ret = cc33xx_init_ap_role(cc, wlvif);
		if (ret < 0)
			return ret;
	} else {
		ret = cc33xx_init_sta_role(cc, wlvif);
		if (ret < 0)
			return ret;
	}

	cc33xx_init_phy_vif_config(cc, wlvif);

	/* Default TID/AC configuration */
	WARN_ON(tx_settings->tid_conf_count != tx_settings->ac_conf_count);
	memcpy(ac_conf, p_wl_host_ac_conf, 4 * sizeof(struct conf_tx_ac_category));
	memcpy(tid_conf, p_wl_host_tid_conf, 8 * sizeof(struct conf_tx_tid));

	for (i = 0; i < tx_settings->tid_conf_count; i++) {
		conf_ac =  &ac_conf[i];

		/* If no ps poll is used, send legacy ps scheme in cmd */
		if (ps_scheme == PS_SCHEME_NOPSPOLL)
			ps_scheme = PS_SCHEME_LEGACY;

		ret = cc33xx_tx_param_cfg(cc, wlvif, conf_ac->ac,
					  conf_ac->cw_min, conf_ac->cw_max,
					  conf_ac->aifsn, conf_ac->tx_op_limit,
					  false, ps_scheme, conf_ac->is_mu_edca,
					  conf_ac->mu_edca_aifs,
					  conf_ac->mu_edca_ecw_min_max,
					  conf_ac->mu_edca_timer);

		if (ret < 0)
			return ret;
	}

	/* Mode specific init - post mem init */
	if (is_ap)
		ret = cc33xx_ap_init_templates(cc, vif);

	if (ret < 0)
		return ret;

	/* Configure initiator BA sessions policies */
	cc33xx_set_ba_policies(cc, wlvif);

	return 0;
}

int cc33xx_hw_init(struct cc33xx *cc)
{
	cc33xx_acx_init_mem_config(cc);

	cc33xx_debug(DEBUG_TX, "available tx blocks: %d", 16);
	cc->last_fw_rls_idx = 0;
	cc->partial_rx.status = CURR_RX_START;
	return 0;
}

int cc33xx_download_ini_params_and_wait(struct cc33xx *cc)
{
	struct cc33xx_cmd_ini_params_download *cmd;
	size_t command_size = ALIGN((sizeof(*cmd) + sizeof(cc->conf)), 4);
	int ret;

	cc33xx_set_max_buffer_size(cc, INI_MAX_BUFFER_SIZE);

	cc33xx_debug(DEBUG_ACX,
		     "Downloading INI configurations to FW, payload Length: %zu",
		     sizeof(cc->conf));

	cmd = kzalloc(command_size, GFP_KERNEL);
	if (!cmd) {
		cc33xx_set_max_buffer_size(cc, CMD_MAX_BUFFER_SIZE);
		return -ENOMEM;
	}

	cmd->length = cpu_to_le32(sizeof(cc->conf));

	/* copy INI file params payload */
	memcpy((cmd->payload), &cc->conf, sizeof(cc->conf));

	ret = cc33xx_cmd_send(cc, CMD_DOWNLOAD_INI_PARAMS,
			      cmd, command_size, 0);
	if (ret < 0) {
		cc33xx_warning("download INI params to FW command sending failed: %d",
			       ret);
	} else {
		cc33xx_debug(DEBUG_BOOT, "INI Params downloaded successfully");
	}

	cc33xx_set_max_buffer_size(cc, CMD_MAX_BUFFER_SIZE);
	kfree(cmd);
	return ret;
}
