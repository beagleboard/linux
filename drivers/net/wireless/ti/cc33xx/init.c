// SPDX-License-Identifier: GPL-2.0-only
/*
 * This file is part of wl1271
 *
 * Copyright (C) 2009 Nokia Corporation
 *
 * Contact: Luciano Coelho <luciano.coelho@nokia.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/firmware.h>

#include "debug.h"
#include "init.h"
#include "wl12xx_80211.h"
#include "acx.h"
#include "cmd.h"
#include "tx.h"
#include "io.h"

struct calibration_file_header {
	u8 	file_version;
	u8 	payload_struct_version;
	u16 	entries_count;
};

int wl1271_init_templates_config(struct wl1271 *wl)
{
	int ret;
	size_t max_size;

	/* send empty templates for fw memory reservation */
	ret = wl1271_cmd_template_set(wl, CC33XX_INVALID_ROLE_ID,
				      wl->scan_templ_id_2_4, NULL,
				      WL1271_CMD_TEMPL_MAX_SIZE,
				      0, WL1271_RATE_AUTOMATIC);
	if (ret < 0)
		return ret;

	ret = wl1271_cmd_template_set(wl, CC33XX_INVALID_ROLE_ID,
				      wl->scan_templ_id_5,
				      NULL, WL1271_CMD_TEMPL_MAX_SIZE, 0,
				      WL1271_RATE_AUTOMATIC);
	if (ret < 0)
		return ret;

	if (wl->quirks & WLCORE_QUIRK_DUAL_PROBE_TMPL) {
		ret = wl1271_cmd_template_set(wl, CC33XX_INVALID_ROLE_ID,
					      wl->sched_scan_templ_id_2_4,
					      NULL,
					      WL1271_CMD_TEMPL_MAX_SIZE,
					      0, WL1271_RATE_AUTOMATIC);
		if (ret < 0)
			return ret;

		ret = wl1271_cmd_template_set(wl, CC33XX_INVALID_ROLE_ID,
					      wl->sched_scan_templ_id_5,
					      NULL,
					      WL1271_CMD_TEMPL_MAX_SIZE,
					      0, WL1271_RATE_AUTOMATIC);
		if (ret < 0)
			return ret;
	}

	ret = wl1271_cmd_template_set(wl, CC33XX_INVALID_ROLE_ID,
				      CMD_TEMPL_NULL_DATA, NULL,
				      sizeof(struct wl12xx_null_data_template),
				      0, WL1271_RATE_AUTOMATIC);
	if (ret < 0)
		return ret;

	ret = wl1271_cmd_template_set(wl, CC33XX_INVALID_ROLE_ID,
				      CMD_TEMPL_PS_POLL, NULL,
				      sizeof(struct wl12xx_ps_poll_template),
				      0, WL1271_RATE_AUTOMATIC);
	if (ret < 0)
		return ret;

	ret = wl1271_cmd_template_set(wl, CC33XX_INVALID_ROLE_ID,
				      CMD_TEMPL_QOS_NULL_DATA, NULL,
				      sizeof
				      (struct ieee80211_qos_hdr),
				      0, WL1271_RATE_AUTOMATIC);
	if (ret < 0)
		return ret;

	ret = wl1271_cmd_template_set(wl, CC33XX_INVALID_ROLE_ID,
				      CMD_TEMPL_PROBE_RESPONSE, NULL,
				      WL1271_CMD_TEMPL_DFLT_SIZE,
				      0, WL1271_RATE_AUTOMATIC);
	if (ret < 0)
		return ret;

	ret = wl1271_cmd_template_set(wl, CC33XX_INVALID_ROLE_ID,
				      CMD_TEMPL_BEACON, NULL,
				      WL1271_CMD_TEMPL_DFLT_SIZE,
				      0, WL1271_RATE_AUTOMATIC);
	if (ret < 0)
		return ret;

	max_size = sizeof(struct wl12xx_arp_rsp_template) +
		   WL1271_EXTRA_SPACE_MAX;
	ret = wl1271_cmd_template_set(wl, CC33XX_INVALID_ROLE_ID,
				      CMD_TEMPL_ARP_RSP, NULL,
				      max_size,
				      0, WL1271_RATE_AUTOMATIC);
	if (ret < 0)
		return ret;

	/*
	 * Put very large empty placeholders for all templates. These
	 * reserve memory for later.
	 */
	ret = wl1271_cmd_template_set(wl, CC33XX_INVALID_ROLE_ID,
				      CMD_TEMPL_AP_PROBE_RESPONSE, NULL,
				      WL1271_CMD_TEMPL_MAX_SIZE,
				      0, WL1271_RATE_AUTOMATIC);
	if (ret < 0)
		return ret;

	ret = wl1271_cmd_template_set(wl, CC33XX_INVALID_ROLE_ID,
				      CMD_TEMPL_AP_BEACON, NULL,
				      WL1271_CMD_TEMPL_MAX_SIZE,
				      0, WL1271_RATE_AUTOMATIC);
	if (ret < 0)
		return ret;

	ret = wl1271_cmd_template_set(wl, CC33XX_INVALID_ROLE_ID,
				      CMD_TEMPL_DEAUTH_AP, NULL,
				      sizeof
				      (struct wl12xx_disconn_template),
				      0, WL1271_RATE_AUTOMATIC);
	if (ret < 0)
		return ret;

	return 0;
}

static int wl1271_ap_init_deauth_template(struct wl1271 *wl,
					  struct wl12xx_vif *wlvif)
{
	struct wl12xx_disconn_template *tmpl;
	int ret;
	u32 rate;

	tmpl = kzalloc(sizeof(*tmpl), GFP_KERNEL);
	if (!tmpl) {
		ret = -ENOMEM;
		goto out;
	}

	tmpl->header.frame_ctl = cpu_to_le16(IEEE80211_FTYPE_MGMT |
					     IEEE80211_STYPE_DEAUTH);

	rate = cc33xx_tx_min_rate_get(wl, wlvif->basic_rate_set);
	ret = wl1271_cmd_template_set(wl, wlvif->role_id,
				      CMD_TEMPL_DEAUTH_AP,
				      tmpl, sizeof(*tmpl), 0, rate);

out:
	kfree(tmpl);
	return ret;
}

static int wl1271_ap_init_null_template(struct wl1271 *wl,
					struct ieee80211_vif *vif)
{
	struct wl12xx_vif *wlvif = cc33xx_vif_to_data(vif);
	struct ieee80211_hdr_3addr *nullfunc;
	int ret;
	u32 rate;

	nullfunc = kzalloc(sizeof(*nullfunc), GFP_KERNEL);
	if (!nullfunc) {
		ret = -ENOMEM;
		goto out;
	}

	nullfunc->frame_control = cpu_to_le16(IEEE80211_FTYPE_DATA |
					      IEEE80211_STYPE_NULLFUNC |
					      IEEE80211_FCTL_FROMDS);

	/* nullfunc->addr1 is filled by FW */

	memcpy(nullfunc->addr2, vif->addr, ETH_ALEN);
	memcpy(nullfunc->addr3, vif->addr, ETH_ALEN);

	rate = cc33xx_tx_min_rate_get(wl, wlvif->basic_rate_set);
	ret = wl1271_cmd_template_set(wl, wlvif->role_id,
				      CMD_TEMPL_NULL_DATA, nullfunc,
				      sizeof(*nullfunc), 0, rate);

out:
	kfree(nullfunc);
	return ret;
}

static int wl1271_ap_init_qos_null_template(struct wl1271 *wl,
					    struct ieee80211_vif *vif)
{
	struct wl12xx_vif *wlvif = cc33xx_vif_to_data(vif);
	struct ieee80211_qos_hdr *qosnull;
	int ret;
	u32 rate;

	qosnull = kzalloc(sizeof(*qosnull), GFP_KERNEL);
	if (!qosnull) {
		ret = -ENOMEM;
		goto out;
	}

	qosnull->frame_control = cpu_to_le16(IEEE80211_FTYPE_DATA |
					     IEEE80211_STYPE_QOS_NULLFUNC |
					     IEEE80211_FCTL_FROMDS);

	/* qosnull->addr1 is filled by FW */

	memcpy(qosnull->addr2, vif->addr, ETH_ALEN);
	memcpy(qosnull->addr3, vif->addr, ETH_ALEN);

	rate = cc33xx_tx_min_rate_get(wl, wlvif->basic_rate_set);
	ret = wl1271_cmd_template_set(wl, wlvif->role_id,
				      CMD_TEMPL_QOS_NULL_DATA, qosnull,
				      sizeof(*qosnull), 0, rate);

out:
	kfree(qosnull);
	return ret;
}

static int cc33xx_init_phy_vif_config(struct wl1271 *wl,
					    struct wl12xx_vif *wlvif)
{
	int ret;

	ret = wl1271_acx_slot(wl, wlvif, DEFAULT_SLOT_TIME);
	if (ret < 0)
		return ret;

	ret = wl1271_acx_service_period_timeout(wl, wlvif);
	if (ret < 0)
		return ret;

	ret = cc33xx_acx_rts_threshold(wl, wlvif, wl->hw->wiphy->rts_threshold);
	if (ret < 0)
		return ret;

	return 0;
}

static int wl1271_init_sta_beacon_filter(struct wl1271 *wl,
					 struct wl12xx_vif *wlvif)
{
	int ret;

	ret = wl1271_acx_beacon_filter_table(wl, wlvif);
	if (ret < 0)
		return ret;

	/* disable beacon filtering until we get the first beacon */
	ret = cc33xx_acx_beacon_filter_opt(wl, wlvif, false);
	if (ret < 0)
		return ret;

	return 0;
}

int wl1271_init_pta(struct wl1271 *wl)
{
	int ret;

	ret = wl12xx_acx_sg_cfg(wl);
	if (ret < 0)
		return ret;

	ret = wl1271_acx_sg_enable(wl, wl->sg_enabled);
	if (ret < 0)
		return ret;

	return 0;
}

int wl1271_init_energy_detection(struct wl1271 *wl)
{
	int ret;

	ret = wl1271_acx_cca_threshold(wl);
	if (ret < 0)
		return ret;

	return 0;
}

static int wl1271_init_beacon_broadcast(struct wl1271 *wl,
					struct wl12xx_vif *wlvif)
{
	int ret;

	ret = wl1271_acx_bcn_dtim_options(wl, wlvif);
	if (ret < 0)
		return ret;

	return 0;
}

/* generic sta initialization (non vif-specific) */
int cc33xx_sta_hw_init(struct wl1271 *wl, struct wl12xx_vif *wlvif)
{
	int ret;

	/* PS config */
	ret = cc33xx_acx_config_ps(wl, wlvif);
	if (ret < 0)
		return ret;

	return 0;
}

/* generic ap initialization (non vif-specific) */
static int cc33xx_ap_hw_init(struct wl1271 *wl)
{
	int ret;
	/* configure AP sleep, if enabled */
	ret = wl18xx_acx_ap_sleep(wl);
	if (ret < 0)
		return ret;

	return 0;
}

int wl1271_ap_init_templates(struct wl1271 *wl, struct ieee80211_vif *vif)
{
	struct wl12xx_vif *wlvif = cc33xx_vif_to_data(vif);
	int ret;

	ret = wl1271_ap_init_deauth_template(wl, wlvif);
	if (ret < 0)
		return ret;

	ret = wl1271_ap_init_null_template(wl, vif);
	if (ret < 0)
		return ret;

	ret = wl1271_ap_init_qos_null_template(wl, vif);
	if (ret < 0)
		return ret;

	/*
	 * when operating as AP we want to receive external beacons for
	 * configuring ERP protection.
	 */
	ret = cc33xx_acx_beacon_filter_opt(wl, wlvif, false);
	if (ret < 0)
		return ret;

	return 0;
}

static int cc33xx_ap_hw_init_post_mem(struct wl1271 *wl,
				      struct ieee80211_vif *vif)
{
	return wl1271_ap_init_templates(wl, vif);
}



static int cc33xx_set_ba_policies(struct wl1271 *wl, struct wl12xx_vif *wlvif)
{
	/* Reset the BA RX indicators */
	wlvif->ba_allowed = true;
	wl->ba_rx_session_count = 0;

	/* BA is supported in STA/AP modes */
	if (wlvif->bss_type != BSS_TYPE_AP_BSS &&
	    wlvif->bss_type != BSS_TYPE_STA_BSS) {
		wlvif->ba_support = false;
		return 0;
	}

	wlvif->ba_support = true;

	/* 802.11n initiator BA session setting */
	return wl12xx_acx_set_ba_initiator_policy(wl, wlvif);
}

/* Applies when MAC addr is other than 0x0 */
static bool find_calibration_data(u8 *id, u8 **data_ptr, u8 *stop_address)
{
	struct calibration_header *calibration_header;
	struct calibration_header_fw *calibration_header_fw;
	int compare_result = 0;
	bool mac_match = false;

	do {
		// cast to a struct for convenient fields reading
		calibration_header = (struct calibration_header *)(*data_ptr);
		calibration_header_fw = &(calibration_header->cal_header_fw);

		if (calibration_header->static_pattern != 0x7F7F) {
		cc33xx_debug(DEBUG_BOOT, "problem with sync pattern, read: %d, data ptr %x",
					calibration_header->static_pattern, *data_ptr);
		break;
		}

		compare_result = memcmp(calibration_header_fw->chip_id, id,
					ETH_ALEN);
		if (0 == compare_result) {
			mac_match = true;
			*data_ptr = (u8 *)calibration_header;
			break;
		}

		// advance ptr by specified payload length to next entry in file
		*data_ptr = (u8 *)((u32)calibration_header
				+ sizeof(*calibration_header)
				+ calibration_header_fw->length);

	} while ((u32 )*data_ptr < (u32 )stop_address);

	return mac_match;
}

static int get_device_calibration_data(u8 *chip_id, u8 **file_ptr,
				       size_t total_data_length)
{
	bool mac_match = false;
	u8 *stop_address;
	u8 broadcast_mac_address[ETH_ALEN] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
	struct calibration_file_header *file_header;
	
	file_header = (struct calibration_file_header *) *file_ptr;

	file_header++;
	*file_ptr = (u8 *)file_header; // pass through 4 bytes of header

	stop_address = (u8 *)((u32)(*file_ptr) + total_data_length);


	mac_match = find_calibration_data(chip_id,
						file_ptr,
						stop_address);
	if(false == mac_match)
	{
		cc33xx_warning("No calibration found for this mac address, "
			"looking for default calibration (labeled mac FF:FF:FF:FF:FF:FF)");
		*file_ptr = (u8 *)file_header;
		mac_match = find_calibration_data(broadcast_mac_address,
						file_ptr,
						stop_address);
	}
	else
	{
		cc33xx_debug(DEBUG_BOOT, "calibration MAC address match");
	}

	if (false == mac_match) {
		cc33xx_debug(DEBUG_BOOT, "no data available for static " 
					 "calibration");
		return -ENXIO;
	} 

	return 0;
}

static int download_static_calibration_data(struct wl1271 *wl)
{
	int ret;
	const struct firmware *fw = NULL;
	const char *calibration_file_name = "ti-connectivity"
					    "/static_calibration.bin";
	u8 *mac_address = (u8 *)wl->efuse_mac_address;
	bool valid_calibration_data = false;
	bool file_loaded;
	u8 *file_ptr = NULL;
	struct calibration_file_header *file_header;

	cc33xx_debug(DEBUG_BOOT,"efuse mac address: "
				"%02x:%02x:%02x:%02x:%02x:%02x",
				mac_address[5], mac_address[4], mac_address[3],
				mac_address[2], mac_address[1], mac_address[0]);

	ret = request_firmware(&fw, calibration_file_name, wl->dev);
	if (ret < 0) {
		cc33xx_warning("could not get firmware %s: %d",
				calibration_file_name, ret);
		valid_calibration_data = false;
		file_loaded = false;
		goto out;
	} else {
		file_loaded = true;
	}

	file_ptr = (u8 *)fw->data;

    file_header = (struct calibration_file_header *)file_ptr;
	cc33xx_debug(DEBUG_BOOT, "parsing static calibration file version: %d,"
			 	 "payload struct ver: %d, entries count: %d file size: %d",
			 	 file_header->file_version, 
				 file_header->payload_struct_version,
			 	 file_header->entries_count,
				 fw->size);
    
	ret = get_device_calibration_data(mac_address,
					  &file_ptr,
					  fw->size);

	if (ret < 0) {
		cc33xx_warning("can't find device's static calibration data " 
				"in calibration file and cannot find default calibration");
		valid_calibration_data = false;
	} else {
		valid_calibration_data = true;
	}

out:
	ret = wl12xx_acx_static_calibration_configure(wl,
							  file_header->file_version,
							  file_header->payload_struct_version, 	
						      file_ptr,
						      valid_calibration_data);
	if (file_loaded)
		release_firmware(fw);

	return ret;
}

/* vif-specifc initialization */
static int cc33xx_init_sta_role(struct wl1271 *wl, struct wl12xx_vif *wlvif)
{
	int ret;
	
	ret = cc33xx_acx_group_address_tbl(wl, wlvif, true, NULL, 0);
	if (ret < 0)
		return ret;

	/* Initialize connection monitoring thresholds */
	ret = wl1271_acx_conn_monit_params(wl, wlvif, false);
	if (ret < 0)
		return ret;

	/* Beacon filtering */
	ret = wl1271_init_sta_beacon_filter(wl, wlvif);
	if (ret < 0)
		return ret;

	/* Beacons and broadcast settings */
	ret = wl1271_init_beacon_broadcast(wl, wlvif);
	if (ret < 0)
		return ret;

	/* Configure rssi/snr averaging weights */
	ret = wl1271_acx_rssi_snr_avg_weights(wl, wlvif);
	if (ret < 0)
		return ret;

	return 0;
}


/* vif-specific initialization */
static int cc33xx_init_ap_role(struct wl1271 *wl, struct wl12xx_vif *wlvif)
{
	int ret;

	ret = wl1271_acx_ap_max_tx_retry(wl, wlvif);
	if (ret < 0)
		return ret;

	/* initialize Tx power */
	ret = cc33xx_acx_tx_power(wl, wlvif, wlvif->power_level);
	if (ret < 0)
		return ret;

	if (wl->radar_debug_mode)
		wlcore_cmd_generic_cfg(wl, wlvif,
				       WLCORE_CFG_FEATURE_RADAR_DEBUG,
				       wl->radar_debug_mode, 0);

	return 0;
}

int cc33xx_init_vif_specific(struct wl1271 *wl, struct ieee80211_vif *vif)
{
	struct wl12xx_vif *wlvif = cc33xx_vif_to_data(vif);
	struct conf_tx_ac_category *conf_ac;
	struct conf_tx_tid *conf_tid;
	bool is_ap = (wlvif->bss_type == BSS_TYPE_AP_BSS);
	int ret, i;
	
	/* consider all existing roles before configuring psm. */

	if (wl->ap_count == 0 && is_ap) { /* first AP */
		ret = cc33xx_acx_sleep_auth(wl, CC33XX_PSM_ELP);
		if (ret < 0)
			return ret;

		/* unmask ap events */
		wl->event_mask |= wl->ap_event_mask;
		ret = cc33xx_event_unmask(wl);
		if (ret < 0)
			return ret;
	/* first STA, no APs */
	} else if (wl->sta_count == 0 && wl->ap_count == 0 && !is_ap) {
		u8 sta_auth = wl->conf.conn.sta_sleep_auth;
		/* Configure for power according to debugfs */
		if (sta_auth != CC33XX_PSM_ILLEGAL)
			ret = cc33xx_acx_sleep_auth(wl, sta_auth);
		/* Configure for ELP power saving */
		else
			ret = cc33xx_acx_sleep_auth(wl, CC33XX_PSM_ELP);

		if (ret < 0)
			return ret;
	}

	/* Get static calibration data and send it to FW*/
	ret = download_static_calibration_data(wl);
	if (ret < 0)
		return ret;

	/* Mode specific init */
	if (is_ap) {
		ret = cc33xx_ap_hw_init(wl);
		if (ret < 0)
			return ret;

		ret = cc33xx_init_ap_role(wl, wlvif);
		if (ret < 0)
			return ret;
	} else {
		ret = cc33xx_sta_hw_init(wl, wlvif);
		if (ret < 0)
			return ret;

		ret = cc33xx_init_sta_role(wl, wlvif);
		if (ret < 0)
			return ret;
	}

	cc33xx_init_phy_vif_config(wl, wlvif);

	/* Default TID/AC configuration */
	BUG_ON(wl->conf.tx.tid_conf_count != wl->conf.tx.ac_conf_count);
	for (i = 0; i < wl->conf.tx.tid_conf_count; i++) {
		conf_ac = &wl->conf.tx.ac_conf[i];
		conf_tid = &wl->conf.tx.tid_conf[i];

//TODO: RazB - need to configure MUEDCA
        ret = cc33xx_tx_param_cfg(wl, wlvif, conf_ac->ac,
                    conf_ac->cw_min, conf_ac->cw_max,
                    conf_ac->aifsn, conf_ac->tx_op_limit, false,
                    conf_tid->ps_scheme, conf_ac->is_mu_edca,
		    conf_ac->mu_edca_aifs, conf_ac->mu_edca_ecw_min_max,
		    conf_ac->mu_edca_timer);
	
        if (ret < 0)
                return ret;
	}

	/* Configure HW encryption */
	ret = cc33xx_acx_feature_cfg(wl, wlvif);
	if (ret < 0)
		return ret;

	/* Mode specific init - post mem init */
	if (is_ap)
		ret = cc33xx_ap_hw_init_post_mem(wl, vif);
	else

	if (ret < 0)
		return ret;

	/* Configure initiator BA sessions policies */
	ret = cc33xx_set_ba_policies(wl, wlvif);
	if (ret < 0)
		return ret;

	return 0;
}

int wl1271_hw_init(struct wl1271 *wl)
{
	int ret;
	ret = wl1271_acx_init_mem_config(wl);
	
	cc33xx_debug(DEBUG_OSPREY, "Skipping wl1271_hw_init");	
	cc33xx_debug(DEBUG_TX, "available tx blocks: %d", 16);
	wl->last_fw_rls_idx = 0;
	wl->partial_rx.status = CURR_RX_START;
	return 0;
}
