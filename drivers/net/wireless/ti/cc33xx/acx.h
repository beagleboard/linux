/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (C) 2022-2024 Texas Instruments Incorporated - https://www.ti.com/
 */

#ifndef __ACX_H__
#define __ACX_H__

#include "cmd.h"
#include "debug.h"

enum {
	/* Regular PS: simple sending of packets */
	PS_SCHEME_LEGACY         = 0,
	/* UPSD: sending a packet triggers a UPSD downstream*/
	PS_SCHEME_UPSD_TRIGGER   = 1,
	/* Mixed mode is partially supported: we are not going to sleep, and
	 * triggers (on APSD AC's) are not sent when service period ends with
	 * more_data = 1.
	 */
	PS_SCHEME_MIXED_MODE     = 2,
	/* Legacy PSPOLL: a PSPOLL packet will be sent before every data packet
	 * transmission in this queue.
	 */
	PS_SCHEME_LEGACY_PSPOLL  = 3,
	/* Scheduled APSD mode. */
	PS_SCHEME_SAPSD          = 4,
	/* No PSPOLL: move to active after first packet. no need to sent pspoll */
	PS_SCHEME_NOPSPOLL       = 5,

	MAX_PS_SCHEME = PS_SCHEME_NOPSPOLL
};

/* Target's information element */
struct acx_header {
	struct cc33xx_cmd_header cmd;

	/* acx (or information element) header */
	__le16 id;

	/* payload length (not including headers */
	__le16 len;
} __packed;

struct debug_header {
	struct cc33xx_cmd_header cmd;

	/* debug (or information element) header */
	__le16 id;

	/* payload length (not including headers */
	__le16 len;
} __packed;

enum cc33xx_role {
	CC33XX_ROLE_STA = 0,
	CC33XX_ROLE_IBSS,
	CC33XX_ROLE_AP,
	CC33XX_ROLE_DEVICE,
	CC33XX_ROLE_P2P_CL,
	CC33XX_ROLE_P2P_GO,
	CC33XX_ROLE_MESH_POINT,

	ROLE_TRANSCEIVER     = 16,

	CC33XX_INVALID_ROLE_TYPE = 0xff
};

enum cc33xx_psm_mode {
	/* Active mode */
	CC33XX_PSM_CAM = 0,

	/* Power save mode */
	CC33XX_PSM_PS = 1,

	/* Extreme low power */
	CC33XX_PSM_ELP = 2,

	CC33XX_PSM_MAX = CC33XX_PSM_ELP,

	/* illegal out of band value of PSM mode */
	CC33XX_PSM_ILLEGAL = 0xff
};

struct acx_sleep_auth {
	struct acx_header header;

	/* The sleep level authorization of the device. */
	/* 0 - Always active*/
	/* 1 - Power down mode: light / fast sleep*/
	/* 2 - ELP mode: Deep / Max sleep*/
	u8  sleep_auth;
	u8  padding[3];
} __packed;

enum acx_slot_type {
	SLOT_TIME_LONG = 0,
	SLOT_TIME_SHORT = 1,
	DEFAULT_SLOT_TIME = SLOT_TIME_SHORT,
	MAX_SLOT_TIMES = 0xFF
};

struct acx_slot {
	struct acx_header header;

	u8 role_id;
	u8 slot_time;
	u8 reserved[2];
} __packed;

#define ACX_MC_ADDRESS_GROUP_MAX	(20)
#define ADDRESS_GROUP_MAX_LEN		(ETH_ALEN * ACX_MC_ADDRESS_GROUP_MAX)

struct acx_dot11_grp_addr_tbl {
	struct acx_header header;

	u8 enabled;
	u8 num_groups;
	u8 pad[2];
	u8 mac_table[ADDRESS_GROUP_MAX_LEN];
} __packed;

struct acx_beacon_filter_option {
	struct acx_header header;

	u8 role_id;
	u8 enable;
	/* The number of beacons without the unicast TIM
	 * bit set that the firmware buffers before
	 * signaling the host about ready frames.
	 * When set to 0 and the filter is enabled, beacons
	 * without the unicast TIM bit set are dropped.
	 */
	u8 max_num_beacons;
	u8 pad;
} __packed;

/* ACXBeaconFilterEntry (not 221)
 * Byte Offset     Size (Bytes)    Definition
 * ===========     ============    ==========
 * 0               1               IE identifier
 * 1               1               Treatment bit mask
 *
 * ACXBeaconFilterEntry (221)
 * Byte Offset     Size (Bytes)    Definition
 * ===========     ============    ==========
 * 0               1               IE identifier
 * 1               1               Treatment bit mask
 * 2               3               OUI
 * 5               1               Type
 * 6               2               Version
 *
 *
 * Treatment bit mask - The information element handling:
 * bit 0 - The information element is compared and transferred
 * in case of change.
 * bit 1 - The information element is transferred to the host
 * with each appearance or disappearance.
 * Note that both bits can be set at the same time.
 */

enum {
	BEACON_FILTER_TABLE_MAX_IE_NUM				= 32,
	BEACON_FILTER_TABLE_MAX_VENDOR_SPECIFIC_IE_NUM		= 6,
	BEACON_FILTER_TABLE_IE_ENTRY_SIZE			= 2,
	BEACON_FILTER_TABLE_EXTRA_VENDOR_SPECIFIC_IE_SIZE	= 6
};

#define BEACON_FILTER_TABLE_MAX_SIZE					\
		((BEACON_FILTER_TABLE_MAX_IE_NUM *			\
		BEACON_FILTER_TABLE_IE_ENTRY_SIZE) +			\
		(BEACON_FILTER_TABLE_MAX_VENDOR_SPECIFIC_IE_NUM *	\
		BEACON_FILTER_TABLE_EXTRA_VENDOR_SPECIFIC_IE_SIZE))

struct acx_beacon_filter_ie_table {
	struct acx_header header;

	u8 role_id;
	u8 num_ie;
	u8 pad[2];
	u8 table[BEACON_FILTER_TABLE_MAX_SIZE];
} __packed;

struct acx_energy_detection {
	struct acx_header header;

	/* The RX Clear Channel Assessment threshold in the PHY */
	__le16 rx_cca_threshold;
	u8 tx_energy_detection;
	u8 pad;
} __packed;

struct acx_event_mask {
	struct acx_header header;

	__le32 event_mask;
	__le32 high_event_mask; /* Unused */
} __packed;

struct acx_tx_power_cfg {
	struct acx_header header;

	u8 role_id;
	s8 tx_power;
	u8 padding[2];
} __packed;

struct acx_wake_up_condition {
	struct acx_header header;

	u8 wake_up_event;
	u8 listen_interval;
	u8 padding[2];
} __packed;

struct assoc_info_cfg {
	struct acx_header header;

	u8 role_id;
	__le16 aid;
	u8 wmm_enabled;
	u8 nontransmitted;
	u8 bssid_index;
	u8 bssid_indicator;
	u8 transmitter_bssid[ETH_ALEN];
	u8 ht_supported;
	u8 vht_supported;
	u8 has_he;
} __packed;

enum acx_preamble_type {
	ACX_PREAMBLE_LONG = 0,
	ACX_PREAMBLE_SHORT = 1
};

struct acx_preamble {
	struct acx_header header;

	/* When set, the WiLink transmits the frames with a short preamble and
	 * when cleared, the WiLink transmits the frames with a long preamble.
	 */
	u8 role_id;
	u8 preamble;
	u8 padding[2];
} __packed;

enum acx_ctsprotect_type {
	CTSPROTECT_DISABLE = 0,
	CTSPROTECT_ENABLE = 1
};

struct acx_ctsprotect {
	struct acx_header header;
	u8 role_id;
	u8 ctsprotect;
	u8 padding[2];
} __packed;

struct ap_rates_class_cfg {
	struct acx_header header;
	u8 role_id;
	__le32 basic_rates_set;
	__le32 supported_rates;
	u8 padding[3];
} __packed;

struct tx_param_cfg {
	struct acx_header header;

	u8 role_id;
	u8 ac;
	u8 aifsn;
	u8 cw_min;

	__le16 cw_max;
	__le16 tx_op_limit;

	__le16 acm;

	u8 ps_scheme;

	u8 is_mu_edca;
	u8 mu_edca_aifs;
	u8 mu_edca_ecw_min_max;
	u8 mu_edca_timer;

	u8 reserved;
} __packed;

struct cc33xx_acx_config_memory {
	struct acx_header header;

	u8 rx_mem_block_num;
	u8 tx_min_mem_block_num;
	u8 num_stations;
	u8 num_ssid_profiles;
	__le32 total_tx_descriptors;
	u8 dyn_mem_enable;
	u8 tx_free_req;
	u8 rx_free_req;
	u8 tx_min;
	u8 fwlog_blocks;
	u8 padding[3];
} __packed;

struct cc33xx_acx_mem_map {
	struct acx_header header;

	/* Number of blocks FW allocated for TX packets */
	__le32 num_tx_mem_blocks;

	/* Number of blocks FW allocated for RX packets */
	__le32 num_rx_mem_blocks;

	/* Number of TX descriptor that allocated. */
	__le32 num_tx_descriptor;

	__le32 tx_result;

} __packed;

struct cc33xx_acx_fw_versions {
	struct acx_header header;

	__le16 major_version;
	__le16 minor_version;
	__le16 api_version;
	__le16 build_version;

	u8 phy_version[8];
	u8 container_type;
	u8 padding[3];
} __packed;

/* special capability bit (not employed by the 802.11n spec) */
#define CC33XX_HT_CAP_HT_OPERATION BIT(16)

/* ACX_HT_BSS_OPERATION
 * Configure HT capabilities - AP rules for behavior in the BSS.
 */
struct cc33xx_acx_ht_information {
	struct acx_header header;

	u8 role_id;

	/* Values: 0 - RIFS not allowed, 1 - RIFS allowed */
	u8 rifs_mode;

	/* Values: 0 - 3 like in spec */
	u8 ht_protection;

	/* Values: 0 - GF protection not required, 1 - GF protection required */
	u8 gf_protection;

	/* Values: 0 - Dual CTS protection not required,
	 *         1 - Dual CTS Protection required
	 * Note: When this value is set to 1 FW will protect all TXOP with RTS
	 * frame and will not use CTS-to-self regardless of the value of the
	 * ACX_CTS_PROTECTION information element
	 */
	u8 dual_cts_protection;

	__le32 he_operation;

	__le16 bss_basic_mcs_set;
	u8 qos_info_more_data_ack_bit;

} __packed;

struct cc33xx_acx_ba_receiver_setup {
	struct acx_header header;

	/* Specifies link id, range 0-31 */
	u8 hlid;

	u8 tid;

	u8 enable;

	/* Windows size in number of packets */
	u8 win_size;

	/* BA session starting sequence number.  RANGE 0-FFF */
	__le16 ssn;

	u8 padding[2];
} __packed;

struct cc33xx_acx_fw_tsf_information {
	struct acx_header header;

	u8 role_id;
	u8 padding1[3];
	__le32 current_tsf_high;
	__le32 current_tsf_low;
	__le32 last_bttt_high;
	__le32 last_tbtt_low;
	u8 last_dtim_count;
	u8 padding2[3];
} __packed;

#define ACX_RATE_MGMT_ALL_PARAMS 0xff
#define ACX_IPV4_ADDR_LENGTH 4

struct acx_default_rx_filter {
	struct acx_header header;
	u8 enable;

	/* action of type FILTER_XXX */
	u8 default_action;

	/* special packet bitmask - packet that use for trigger the host */
	u8 special_packet_bitmask;

	u8 padding;
} __packed;

struct acx_arp_ip_cfg {
	struct acx_header header;
	u8 role_id;
	u8 address[ACX_IPV4_ADDR_LENGTH];
	u8 padding[3];
} __packed;

struct acx_arp_offload {
	struct acx_header header;
	u8 enable;
	u8 padding[3];
} __packed;
struct acx_rx_filter_cfg {
	struct acx_header header;

	u8 enable;

	/* 0 - WL1271_MAX_RX_FILTERS-1 */
	u8 index;

	u8 action;

	u8 num_fields;
	u8 fields[];
} __packed;

struct acx_roaming_stats {
	struct acx_header header;

	u8	role_id;
	u8	pad[3];
	__le32	missed_beacons;
	u8	snr_data;
	u8	snr_bacon;
	s8	rssi_data;
	s8	rssi_beacon;
} __packed;

enum cfg {
	CTS_PROTECTION_CFG			= 0,
	TX_PARAMS_CFG				= 1,
	ASSOC_INFO_CFG				= 2,
	PEER_CAP_CFG				= 3,
	BSS_OPERATION_CFG			= 4,
	SLOT_CFG				= 5,
	PREAMBLE_TYPE_CFG			= 6,
	DOT11_GROUP_ADDRESS_TBL			= 7,
	BA_SESSION_RX_SETUP_CFG			= 8,
	ACX_SLEEP_AUTH				= 9,
	RESERVED				= 10,
	AP_RATES_CFG				= 11,
	WAKE_UP_CONDITIONS_CFG			= 12,
	SET_ANTENNA_SELECT_CFG			= 13,
	TX_POWER_CFG				= 14,
	VENDOR_IE_CFG				= 15,
	START_COEX_STATISTICS_CFG		= 16,
	BEACON_FILTER_OPT			= 17,
	BEACON_FILTER_TABLE			= 18,
	ACX_ENABLE_RX_DATA_FILTER		= 19,
	ACX_SET_RX_DATA_FILTER			= 20,
	ACX_GET_DATA_FILTER_STATISTICS		= 21,
	TWT_SETUP				= 22,
	TWT_TERMINATE				= 23,
	TWT_SUSPEND				= 24,
	TWT_RESUME				= 25,
	ANT_DIV_ENABLE				= 26,
	ANT_DIV_SET_RSSI_THRESHOLD		= 27,
	ANT_DIV_SELECT_DEFAULT_ANTENNA		= 28,
	RESET_DECRYPT_PACKETS_COUNT	= 29,
	ENABLE_CHANNEL_UTILIZATION_NEXT_SCAN = 30,
	SET_SEED_CFG			= 31,
	RESET_STATS			= 32,
	PHY_REGDOMAIN_TX_POWER_PARAMS = 33,
	CQM_RSSI_CONFIG			= 34,
	ACX_ARP_IP_CFG			= 35,
	ACX_ARP_OFFLOAD_ENABLE		= 36,

	LAST_CFG_VALUE,
	MAX_DOT11_CFG = LAST_CFG_VALUE,

	MAX_CFG = 0xFFFF	/*force enumeration to 16bits*/
};

enum cmd_debug {
	UPLINK_MULTI_USER_CFG,
	UPLINK_MULTI_USER_DATA_CFG,
	OPERATION_MODE_CTRL_CFG,
	UPLINK_POWER_HEADER_CFG,
	MCS_FIXED_RATE_CFG,
	GI_LTF_CFG,
	TRANSMIT_OMI_CFG,
	TB_ONLY_CFG,
	BA_SESSION_CFG,
	FORCE_PS_CFG,
	RATE_OVERRRIDE_CFG,
	BLS_CFG,
	BLE_ENABLE,
	SET_TSF,
	RTS_TH_CFG,
	LINK_ADAPT_CFG,
	CALIB_BITMAP_CFG,
	PWR_PARTIAL_MODES_CFG,
	TRIGGER_FW_ASSERT,
	BURST_MODE_CFG,
	THERMAL_PROTECTION_CFG,
	OVERRIDE_TEMPERATURE,

	LAST_DEBUG_VALUE,

	MAX_DEBUG = 0xFFFF /*force enumeration to 16bits*/

};

enum interrogate_opt {
	MEM_MAP_INTR = 0,
	GET_FW_VERSIONS_INTR = 1,
	RSSI_INTR = 2,
	GET_ANTENNA_SELECT_INTR = 3,
	GET_PREAMBLE_AND_TX_RATE_INTR = 4,
	GET_MAC_ADDRESS = 5,
	READ_COEX_STATISTICS = 6,
	GET_ANT_DIV_STATUS = 7,
	GET_ANT_DIV_RSSI_THRESHOLD = 8,
	GET_ANT_DIV_DEFAULT_ANTENNA = 9,
	GET_LATEST_CHANNEL_UTILIZATION_SURVEY = 10,
	GET_DECRYPT_PACKETS_COUNTS = 11,
	GET_ROLE_CHANNEL_NUMBER = 12,
	GET_STATISTICS = 13,
	GET_SP_VERSIONS_INTR = 14,
	GET_LINK_INACTIVITY = 15,
	// RESERVED = 16,
	GET_SLOW_CLK_SOURCE = 17,
	LAST_IE_VALUE,
	MAX_DOT11_IE = LAST_IE_VALUE,

	MAX_IE = 0xFFFF /*force enumeration to 16bits*/
};

/* ACX_PEER_CAP
 * this struct is very similar to cc33xx_acx_ht_capabilities, with the
 * addition of supported rates
 */
#define NOMINAL_PACKET_PADDING (0xC0)
struct cc33xx_acx_peer_cap {
	struct acx_header header;

	u8 role_id;

	u8 pading_0[3];

	/* rates supported by the remote peer */
	__le32 supported_rates;

	/* bitmask of capability bits supported by the peer */
	__le32 ht_capabilites;
	/* This the maximum A-MPDU length supported by the AP. The FW may not
	 * exceed this length when sending A-MPDUs
	 */
	u8 ampdu_max_length;

	/* This is the minimal spacing required when sending A-MPDUs to the AP*/
	u8 ampdu_min_spacing;

	u8 padding_1[2];

	/* bitmask of capability bits supported by the peer */
	__le32 	vht_capabilities;

	/* VHT peer support */
	bool vht_supported;

	u8 padding_2[3];

	/* HE capabilities */
	u8 mac_cap_info[8];

	/* Nominal packet padding value, used for determining the packet extension duration */
	u8 nominal_packet_padding;

	/* HE peer support */
	bool has_he;

	u8 dcm_max_constelation;

	u8 er_upper_supported;
} __packed;

struct acx_antenna_select {
	struct acx_header header;

	u8 selection;
	u8 padding[3];
} __packed;

#define BLE_LIM_CHANNELS_COUNT (40)
#define REG_RULES_COUNT		   (520)

struct acx_phy_regdomain_tx_control_params {
	struct acx_header header;

	u8  bitmask;
	u32 country_code;
	u8  reg_domain;
	u8  ble_ch_lim_1M[BLE_LIM_CHANNELS_COUNT];
	u8  ble_ch_lim_2M[BLE_LIM_CHANNELS_COUNT];
	u8  per_channel_power_limit[REG_RULES_COUNT];
	u8 	padding[2];
} __packed;

struct debug_set_tsf {
	struct debug_header header;

	__le64 tsf_val;
} __packed;

struct debug_burst_mode_cfg {
	struct debug_header header;

	u8 burst_disable;
	u8 padding[3];
} __packed;
struct acx_twt_setup {
	struct acx_header header;
	__le32 min_wake_duration_usec;
	__le32 min_wake_interval_mantissa;
	__le32 min_wake_interval_exponent;
	__le32 max_wake_interval_mantissa;
	__le32 max_wake_interval_exponent;
	u8 valid_params;
	u8 padding[3];
} __packed;

#define MIN_WAKE_DURATION_VALID				BIT(0)
#define MIN_WAKE_INTERVAL_MANTISSA_VALID	BIT(1)
#define MIN_WAKE_INTERVAL_EXPONENT_VALID	BIT(2)
#define MAX_WAKE_INTERVAL_MANTISSA_VALID	BIT(3)
#define MAX_WAKE_INTERVAL_EXPONENT_VALID	BIT(4)

struct acx_twt_terminate {
	struct acx_header header;
} __packed;

struct acx_preamble_and_tx_rate {
	struct acx_header header;
	u16 tx_rate;
	u8 preamble;
	u8 role_id;
} __packed;

static const u16 cc33xx_idx_to_rate_100Kbps[] = {
	10, 20, 55, 110, 60, 90, 120, 180, 240, 360, 480, 540
};

struct cc33xx_coex_statistics {
	__le32 wifi_request_assertion_log;
	__le32 wifi_request_de_assertion_log;
	__le32 wifi_grant_assertion_log;
	__le32 wifi_grant_deassertion_log;
	__le32 wifi_prio_reject_log;
	__le32 wifi_grant_during_dual_ant_assertion_log;
	__le32 wifi_grant_during_dual_ant_deassertion_log;
	__le32 ble_request_assertion_log;
	__le32 ble_request_deassertion_log;
	__le32 ble_grant_assertion_log;
	__le32 ble_grant_deassertion_log;
	__le32 ble_tx_high_prio_reject_log;
	__le32 ble_tx_low_prio_reject_log;
	__le32 ble_rx_high_prio_reject_log;
	__le32 ble_rx_low_prio_reject_log;
	__le32 soc_request_assertion_log;
	__le32 soc_request_deassertion_log;
	__le32 soc_grant_assertion_log;
	__le32 soc_grant_deassertion_log;
	__le32 soc_high_prio_reject_log;
	__le32 soc_low_prio_reject_log;
} __packed;

struct cc33xx_coex_stat_and_entities {
	__u8 coex_entities_bitmap;
	__u8 padding[3];
	struct cc33xx_coex_statistics coex_statistics;
} __packed;


struct cc33xx_acx_coex_statistics {
	struct acx_header header;

	struct cc33xx_coex_stat_and_entities coex_stat;
} __packed;

struct cc33xx_acx_coex_statistics_cfg {
	struct acx_header header;

	__u8 coex_statictics;
	__u8 padding[3];
} __packed;

struct acx_diversity_status {
	struct acx_header header;

    u8 enable;
	u8 padding[3];
} __packed;

struct acx_slow_clk_type {
	struct acx_header header;

	u8 is_ext_slw_clk;
	u8 padding[3];
} __packed;

struct acx_diversity_rssi_threshold {
	struct acx_header header;

    s8 rssi_threshold;
	u8 padding[3];
} __packed;

struct acx_diversity_default_antenna {
	struct acx_header header;

    u8 default_antenna;
	u8 padding[3];
} __packed;

struct acx_cqm_rssi_config {
	struct acx_header header;

	u8 role_id;
	u8 enable;
	s8 threshold_dbm;
	u8 hysteresis_db;
} __packed;

struct reset_decrypt_count_cfg {
	u8 role_id;
	u8 padding[3];
} __packed;

struct decrypt_packets_status_count {
	__le32 roleid;

	struct {
		__le32 decrypted_ok_unicast_count;
		__le32 decrypted_error_unicast_count;
		__le32 decrypted_ok_multicast_count;
		__le32 decrypted_error_multicast_count;
		__le32 decrypted_ok_broadcast_count;
		__le32 decrypted_error_broadcast_count;
	} counters;
} __packed;

struct channel_utilization_survey_results {
	u8 channel_number;
	u8 channel_load;
	s8 noise_floor_avg;
	u8 bss_count;
} __packed;

struct role_current_channel_number {
	u8 role_type;
	u8 role_id;
	__le16 channel_num;
} __packed;

struct cc33xx_acx_power_stats {
	__le32 sleep_time_count;
	__le32 sleep_time_avg;
	__le32 sleep_cycle_avg;
	__le32 sleep_percent;
} __packed;

struct cc33xx_acx_statistics {
	struct acx_header header;

	struct cc33xx_acx_power_stats power;
} __packed;

int cc33xx_acx_wake_up_conditions(struct cc33xx *cc, struct cc33xx_vif *wlvif,
				  u8 wake_up_event, u8 listen_interval);
int cc33xx_acx_sleep_auth(struct cc33xx *cc, u8 sleep_auth);
int cc33xx_ble_enable(struct cc33xx *cc, u8 ble_enable);
int cc33xx_acx_tx_power(struct cc33xx *cc, struct cc33xx_vif *wlvif, int power);
int cc33xx_acx_slot(struct cc33xx *cc, struct cc33xx_vif *wlvif,
		    enum acx_slot_type slot_time);
int cc33xx_acx_group_address_tbl(struct cc33xx *cc, bool enable, void *mc_list, u32 mc_list_len);
int cc33xx_acx_beacon_filter_opt(struct cc33xx *cc, struct cc33xx_vif *wlvif,
				 bool enable_filter);
int cc33xx_acx_beacon_filter_table(struct cc33xx *cc, struct cc33xx_vif *wlvif);
int cc33xx_assoc_info_cfg(struct cc33xx *cc, struct cc33xx_vif *wlvif,
			  struct ieee80211_sta *sta, u16 aid);
int cc33xx_acx_set_preamble(struct cc33xx *cc, struct cc33xx_vif *wlvif,
			    enum acx_preamble_type preamble);
int cc33xx_acx_cts_protect(struct cc33xx *cc, struct cc33xx_vif *wlvif,
			   enum acx_ctsprotect_type ctsprotect);
int cc33xx_tx_param_cfg(struct cc33xx *cc, struct cc33xx_vif *wlvif, u8 ac,
			u8 cw_min, u16 cw_max, u8 aifsn, u16 txop, bool acm,
			u8 ps_scheme, u8 is_mu_edca, u8 mu_edca_aifs,
			u8 mu_edca_ecw_min_max, u8 mu_edca_timer);
int cc33xx_update_ap_rates(struct cc33xx *cc, u8 role_id,
			   u32 basic_rates_set, u32 supported_rates);
int cc33xx_acx_init_mem_config(struct cc33xx *cc);
int cc33xx_acx_init_get_fw_versions(struct cc33xx *cc);
int cc33xx_acx_set_ht_information(struct cc33xx *cc, struct cc33xx_vif *wlvif,
				  u16 ht_operation_mode, u32 he_oper_params,
				  u16 he_oper_nss_set);
int cc33xx_acx_set_ba_receiver_session(struct cc33xx *cc, u8 tid_index, u16 ssn,
				       bool enable, u8 peer_hlid, u8 win_size);
int cc33xx_acx_get_tx_rate(struct cc33xx *cc, struct cc33xx_vif *wlvif,
			   struct station_info *sinfo);
int cc33xx_acx_average_rssi(struct cc33xx *cc,
			    struct cc33xx_vif *wlvif, s8 *avg_rssi);
int cc33xx_acx_arp_ip_config(struct cc33xx *cc, u8 role_id, __be32 ip_addr);
int cc33xx_acx_arp_offload(struct cc33xx *cc, bool enable);
int cc33xx_acx_default_rx_filter_enable(struct cc33xx *cc, bool enable,
					enum rx_filter_action action);
int cc33xx_acx_set_rx_filter(struct cc33xx *cc, u8 index, bool enable,
			     struct cc33xx_rx_filter *filter);
int cc33xx_acx_set_peer_cap(struct cc33xx *cc,
			    struct ieee80211_sta_ht_cap *ht_cap,
			    struct ieee80211_sta_vht_cap *vht_cap,
			    struct ieee80211_sta_he_cap *he_cap,
			    struct cc33xx_vif *wlvif, bool allow_ht_operation,
			    u32 rate_set, u8 hlid);
int cc33xx_acx_set_antenna_select(struct cc33xx *cc, u8 selection);
int cc33xx_acx_set_tsf(struct cc33xx *cc, u64 tsf_val);
int cc33xx_acx_trigger_fw_assert(struct cc33xx *cc);
int cc33xx_acx_burst_mode_cfg(struct cc33xx *cc, u8 burst_disable);
int cc33xx_acx_get_antenna_diversity_status(struct cc33xx *cc);
int cc33xx_acx_set_antenna_diversity_status(struct cc33xx *cc, u8 enable);
int cc33xx_acx_get_slow_clock_type(struct cc33xx *cc);
int cc33xx_acx_antenna_diversity_get_rssi_threshold(struct cc33xx *cc, s8 *threshold);
int cc33xx_acx_antenna_diversity_set_rssi_threshold(struct cc33xx *cc, s8 rssi_threshold);
int cc33xx_acx_antenna_diversity_get_default_antenna(struct cc33xx *cc);
int cc33xx_acx_antenna_diversity_select_default_antenna(struct cc33xx *cc, u8 default_antenna);
int cc33xx_acx_cqm_rssi_config(struct cc33xx *cc, struct cc33xx_vif *wlvif,
			       bool enable, s8 threshold, u8 hysteresis);
int cc33xx_acx_twt_setup(struct cc33xx *wl,
			 u32 min_wake_duration_usec,
			 u32 min_wake_interval_mantissa,
			 u32 min_wake_interval_exponent,
			 u32 max_wake_interval_mantissa,
			 u32 max_wake_interval_exponent,
			 u8 valid_params);
int cc33xx_acx_twt_terminate(struct cc33xx *wl);
int cc33xx_acx_twt_resume(struct cc33xx *wl);
int cc33xx_acx_twt_suspend(struct cc33xx *wl);
int cc33xx_acx_set_regdoamin_and_tx_control_params(struct cc33xx *cc, struct acx_phy_regdomain_tx_control_params *params);
int cc33xx_acx_statistics(struct cc33xx *cc, void *stats);
int cc33xx_acx_clear_statistics(struct cc33xx *cc);

#endif /* __CC33XX_ACX_H__ */
