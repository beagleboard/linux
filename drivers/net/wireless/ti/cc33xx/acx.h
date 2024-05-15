/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * This file is part of cc33xx
 *
 * Copyright (C) 1998-2009 Texas Instruments. All rights reserved.
 * Copyright (C) 2008-2010 Nokia Corporation
 *
 * Contact: Luciano Coelho <luciano.coelho@nokia.com>
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
	   triggers (on APSD AC's) are not sent when service period ends with
	   more_data = 1. */
	PS_SCHEME_MIXED_MODE     = 2,
	/* Legacy PSPOLL: a PSPOLL packet will be sent before every data packet
	   transmission in this queue.*/
	PS_SCHEME_LEGACY_PSPOLL  = 3,
	/* Scheduled APSD mode. */
	PS_SCHEME_SAPSD          = 4,
	/* No PSPOLL: move to active after first packet. no need to sent
	   pspoll */
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

struct acx_rx_timeout {
	struct acx_header header;

	u8 role_id;
	u8 reserved;
	__le16 ps_poll_timeout;
	__le16 upsd_timeout;
	u8 padding[2];
} __packed;

struct acx_rts_threshold {
	struct acx_header header;

	u8 role_id;
	u8 reserved;
	__le16 threshold;
} __packed;

struct acx_beacon_filter_option {
	struct acx_header header;

	u8 role_id;
	u8 enable;
	/*
	 * The number of beacons without the unicast TIM
	 * bit set that the firmware buffers before
	 * signaling the host about ready frames.
	 * When set to 0 and the filter is enabled, beacons
	 * without the unicast TIM bit set are dropped.
	 */
	u8 max_num_beacons;
	u8 pad[1];
} __packed;

/*
 * ACXBeaconFilterEntry (not 221)
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

struct acx_conn_monit_params {
       struct acx_header header;

	   u8 role_id;
	   u8 padding[3];
       __le32 synch_fail_thold; /* number of beacons missed */
       __le32 bss_lose_timeout; /* number of TU's from synch fail */
} __packed;

struct acx_energy_detection {
	struct acx_header header;

	/* The RX Clear Channel Assessment threshold in the PHY */
	__le16 rx_cca_threshold;
	u8 tx_energy_detection;
	u8 pad;
} __packed;

struct acx_beacon_broadcast {
	struct acx_header header;

	u8 role_id;
	/* Enables receiving of broadcast packets in PS mode */
	u8 rx_broadcast_in_ps;

	__le16 beacon_rx_timeout;
	__le16 broadcast_timeout;

	/* Consecutive PS Poll failures before updating the host */
	u8 ps_poll_threshold;
	u8 pad[1];
} __packed;

struct acx_event_mask {
	struct acx_header header;

	__le32 event_mask;
	__le32 high_event_mask; /* Unused */
} __packed;

struct acx_feature_config {
	struct acx_header header;

	u8 role_id;
	u8 padding[3];
	__le32 options;
	__le32 data_flow_options;
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

struct assoc_info_cfg
{
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
}__packed;

enum acx_preamble_type {
	ACX_PREAMBLE_LONG = 0,
	ACX_PREAMBLE_SHORT = 1
};

struct acx_preamble {
	struct acx_header header;

	/*
	 * When set, the WiLink transmits the frames with a short preamble and
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
}__packed;

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

    u8 reserved[1];

} __packed;

struct acx_frag_threshold {
	struct acx_header header;
	__le16 frag_threshold;
	u8 padding[2];
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

struct cc33xx_acx_mem_map_t {
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

	u8 phy_version[6];
	u8 padding[2];
} __packed;

struct cc33xx_acx_rx_config_opt {
	struct acx_header header;

	__le16 mblk_threshold;
	__le16 threshold;
	__le16 timeout;
	u8 queue_type;
	u8 reserved;
} __packed;

struct cc33xx_acx_bet_enable {
	struct acx_header header;

	u8 role_id;
	u8 enable;
	u8 max_consecutive;
	u8 padding[1];
} __packed;

#define ACX_IPV4_VERSION 4
#define ACX_IPV4_ADDR_SIZE 4

/* bitmap of enabled arp_filter features */
#define ACX_ARP_FILTER_ARP_FILTERING	BIT(0)
#define ACX_ARP_FILTER_AUTO_ARP		BIT(1)

struct cc33xx_acx_arp_filter {
	struct acx_header header;
	u8 role_id;
	u8 version;         /* ACX_IPV4_VERSION, ACX_IPV6_VERSION */
	u8 enable;          /* bitmap of enabled ARP filtering features */
	u8 padding[1];
	u8 address[16];     /* The configured device IP address - all ARP
			       requests directed to this IP address will pass
			       through. For IPv4, the first four bytes are
			       used. */
} __packed;

/* TODO: maybe this needs to be moved somewhere else? */
enum {
	HOST_IF_CFG_RX_FIFO_ENABLE	= BIT(0),
	HOST_IF_CFG_TX_EXTRA_BLKS_SWAP	= BIT(1),
	HOST_IF_CFG_TX_PAD_TO_SDIO_BLK	= BIT(3),
	HOST_IF_CFG_RX_PAD_TO_SDIO_BLK	= BIT(4),
	HOST_IF_CFG_ADD_RX_ALIGNMENT	= BIT(6)
};

enum {
	CC33XX_ACX_TRIG_TYPE_LEVEL = 0,
	CC33XX_ACX_TRIG_TYPE_EDGE,
};

enum {
	CC33XX_ACX_TRIG_DIR_LOW = 0,
	CC33XX_ACX_TRIG_DIR_HIGH,
	CC33XX_ACX_TRIG_DIR_BIDIR,
};

enum {
	CC33XX_ACX_TRIG_ENABLE = 1,
	CC33XX_ACX_TRIG_DISABLE,
};

enum {
	CC33XX_ACX_TRIG_METRIC_RSSI_BEACON = 0,
	CC33XX_ACX_TRIG_METRIC_RSSI_DATA,
	CC33XX_ACX_TRIG_METRIC_SNR_BEACON,
	CC33XX_ACX_TRIG_METRIC_SNR_DATA,
};

enum {
	CC33XX_ACX_TRIG_IDX_RSSI = 0,
	CC33XX_ACX_TRIG_COUNT = 8,
};

struct cc33xx_acx_rssi_snr_trigger {
	struct acx_header header;

	u8 role_id;
	u8 metric;
	u8 type;
	u8 dir;
	__le16 threshold;
	__le16 pacing; /* 0 - 60000 ms */
	u8 hysteresis;
	u8 index;
	u8 enable;
	u8 padding[1];
};

struct cc33xx_acx_rssi_snr_avg_weights_t {
	struct acx_header header;

	u8 role_id;
	u8 padding[3];
	u8 rssi_beacon;
	u8 rssi_data;
	u8 snr_beacon;
	u8 snr_data;
};

/* special capability bit (not employed by the 802.11n spec) */
#define CC33XX_HT_CAP_HT_OPERATION BIT(16)

/*
 * ACX_PEER_HT_CAP
 * Configure HT capabilities - declare the capabilities of the peer
 * we are connected to.
 */
struct cc33xx_acx_ht_capabilities {
	struct acx_header header;

	/* bitmask of capability bits supported by the peer */
	__le32 ht_capabilites;

	/* Indicates to which link these capabilities apply. */
	u8 hlid;

	/*
	 * This the maximum A-MPDU length supported by the AP. The FW may not
	 * exceed this length when sending A-MPDUs
	 */
	u8 ampdu_max_length;

	/* This is the minimal spacing required when sending A-MPDUs to the AP*/
	u8 ampdu_min_spacing;

	u8 padding;
} __packed;

/*
 * ACX_HT_BSS_OPERATION
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

	/*
	 * Values: 0 - Dual CTS protection not required,
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

struct cc33xx_acx_ba_initiator_policy {
	struct acx_header header;

	/* Specifies role Id, Range 0-7, 0xFF means ANY role. */
	u8 role_id;

	/*
	 * Per TID setting for allowing TX BA. Set a bit to 1 to allow
	 * TX BA sessions for the corresponding TID.
	 */
	u8 tid_bitmap;

	/* Windows size in number of packets */
	u8 win_size;

	u8 padding1[1];

	/* As initiator inactivity timeout in time units(TU) of 1024us */
	__le16 inactivity_timeout;

	u8 padding[2];
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

struct calibration_header_fw {

	/* chip id to assign each chip with its appropriate data */
	u8 chip_id[ETH_ALEN];
	/* payload can be of different length, chosen by user*/
	__le16 length;

} __packed;

struct calibration_header {
	/* preamble static pattern */
	__le16 static_pattern;

	struct calibration_header_fw cal_header_fw;

} __packed;

struct calibration_file_header {
	u8	file_version;
	u8	payload_struct_version;
	__le16 entries_count;
} __packed;

struct cc33xx_acx_static_calibration_cfg {

	struct acx_header header;

	bool valid_data;
	u8 file_version;
	u8 payload_struct_version;
	u8 padding;
	struct calibration_header_fw calibration_header;
	u8  payload[0];

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

struct cc33xx_acx_ps_rx_streaming {
	struct acx_header header;

	u8 role_id;
	u8 tid;
	u8 enable;

	/* interval between triggers (10-100 msec) */
	u8 period;

	/* timeout before first trigger (0-200 msec) */
	u8 timeout;
	u8 padding[3];
} __packed;

struct cc33xx_acx_ap_max_tx_retry {
	struct acx_header header;

	u8 role_id;
	u8 padding_1;

	/*
	 * the number of frames transmission failures before
	 * issuing the aging event.
	 */
	__le16 max_tx_retry;
} __packed;

struct cc33xx_acx_config_ps {
	struct acx_header header;

	u8 exit_retries;
	u8 enter_retries;
	u8 padding[2];
	__le32 null_data_rate;
} __packed;

struct cc33xx_acx_inconnection_sta {
	struct acx_header header;

	u8 addr[ETH_ALEN];
	u8 role_id;
	u8 padding;
} __packed;

#define ACX_RATE_MGMT_ALL_PARAMS 0xff

struct acx_default_rx_filter {
	struct acx_header header;
	u8 enable;

	/* action of type FILTER_XXX */
	u8 default_action;

	/* special packet bitmask - packet that use for trigger the host */
	u8 special_packet_bitmask;

	u8 padding;
} __packed;

struct acx_rx_filter_cfg {
	struct acx_header header;

	u8 enable;

	/* 0 - WL1271_MAX_RX_FILTERS-1 */
	u8 index;

	u8 action;

	u8 num_fields;
	u8 fields[0];
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

typedef enum {
	CTS_PROTECTION_CFG				= 0,
	TX_PARAMS_CFG					= 1,
	ASSOC_INFO_CFG					= 2,
	PEER_CAP_CFG					= 3,
	BSS_OPERATION_CFG				= 4,
	SLOT_CFG						= 5,
	PREAMBLE_TYPE_CFG				= 6,
	DOT11_GROUP_ADDRESS_TBL			= 7,
	BA_SESSION_RX_SETUP_CFG			= 8,
	ACX_SLEEP_AUTH					= 9,
	STATIC_CALIBRATION_CFG			= 10,
	AP_RATES_CFG					= 11,
	WAKE_UP_CONDITIONS_CFG			= 12,
	SET_ANTENNA_SELECT_CFG			= 13,
	TX_POWER_CFG					= 14,
	VENDOR_IE_CFG					= 15,
	START_COEX_STATISTICS_CFG		= 16,
	BEACON_FILTER_OPT				= 17,
	BEACON_FILTER_TABLE				= 18,
	ACX_ENABLE_RX_DATA_FILTER		= 19,
	ACX_SET_RX_DATA_FILTER			= 20,
	ACX_GET_DATA_FILTER_STATISTICS	= 21,
    TWT_SETUP                	    = 22,
    TWT_TERMINATE            	    = 23,
    TWT_SUSPEND           	   	    = 24,
    TWT_RESUME              	    = 25,
	ANT_DIV_ENABLE 					= 26,
	ANT_DIV_SET_RSSI_THRESHOLD 		= 27,
	ANT_DIV_SELECT_DEFAULT_ANTENNA 	= 28,

	LAST_CFG_VALUE			,
	MAX_DOT11_CFG = LAST_CFG_VALUE	,

	MAX_CFG = 0xFFFF	/*force enumeration to 16bits*/
} Cfg_e;

typedef enum {
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

	LAST_DEBUG_VALUE,

	MAX_DEBUG = 0xFFFF /*force enumeration to 16bits*/

} cmdDebug_e;

typedef enum {
	MEM_MAP_INTR = 0,
	GET_FW_VERSIONS_INTR = 1,
	RSSI_INTR = 2,
	GET_ANTENNA_SELECT_INTR = 3,
	GET_PREAMBLE_AND_TX_RATE_INTR = 4,
	GET_MAC_ADDRESS = 5,
	READ_COEX_STATISTICS = 6,
	LAST_IE_VALUE,
	MAX_DOT11_IE = LAST_IE_VALUE,

	MAX_IE = 0xFFFF /*force enumeration to 16bits*/
} Interrogate_e;

//TODO: RazB - Need to remove this enum when we finish over all the commands
enum {
	ACX_MEM_CFG = LAST_CFG_VALUE	 ,
	ACX_SLOT                         ,
	ACX_AC_CFG                       ,
	ACX_MEM_MAP                      ,
	ACX_AID                          ,
	ACX_MEDIUM_USAGE                 ,
	ACX_STATISTICS                   ,
	ACX_PWR_CONSUMPTION_STATISTICS   ,
	ACX_TID_CFG                      ,
	ACX_PS_RX_STREAMING              ,
	ACX_BEACON_FILTER_OPT            ,
	ACX_NOISE_HIST                   ,
	ACX_HDK_VERSION                  ,
	ACX_PD_THRESHOLD                 ,
	ACX_TX_CONFIG_OPT                ,
	ACX_CCA_THRESHOLD                ,
	ACX_EVENT_MBOX_MASK              ,
	ACX_CONN_MONIT_PARAMS            ,
	ACX_DISABLE_BROADCASTS           ,
	ACX_BCN_DTIM_OPTIONS             ,
	ACX_SG_ENABLE                    ,
	ACX_SG_CFG                       ,
	ACX_FM_COEX_CFG                  ,
	ACX_BEACON_FILTER_TABLE          ,
	ACX_ARP_IP_FILTER                ,
	ACX_ROAMING_STATISTICS_TBL       ,
	ACX_RATE_POLICY                  ,
	ACX_CTS_PROTECTION               ,
	ACX_PREAMBLE_TYPE                ,
	ACX_ERROR_CNT                    ,
	ACX_IBSS_FILTER                  ,
	ACX_SERVICE_PERIOD_TIMEOUT       ,
	ACX_TSF_INFO                     ,
	ACX_CONFIG_PS_WMM                ,
	ACX_RX_CONFIG_OPT                ,
	ACX_FRAG_CFG                     ,
	ACX_BET_ENABLE                   ,
	ACX_RSSI_SNR_TRIGGER             ,
	ACX_RSSI_SNR_WEIGHTS             ,
	ACX_KEEP_ALIVE_MODE              ,
	ACX_BA_SESSION_INIT_POLICY       ,
	ACX_BA_SESSION_RX_SETUP          ,
	ACX_PEER_HT_CAP                  ,
	ACX_HT_BSS_OPERATION             ,
	ACX_COEX_ACTIVITY                ,
	ACX_BURST_MODE                   ,
	ACX_SET_RATE_MGMT_PARAMS         ,
	ACX_GET_RATE_MGMT_PARAMS         ,
	ACX_SET_RATE_ADAPT_PARAMS        ,
	ACX_SET_DCO_ITRIM_PARAMS         ,
	ACX_GEN_FW_CMD                   ,
	ACX_HOST_IF_CFG_BITMAP           ,
	ACX_MAX_TX_FAILURE               ,
	ACX_UPDATE_INCONNECTION_STA_LIST ,
	DOT11_RX_MSDU_LIFE_TIME          ,
	DOT11_CUR_TX_PWR                 ,
	DOT11_RTS_THRESHOLD              ,
	ACX_PM_CONFIG                    ,
	ACX_CONFIG_PS                    ,
	ACX_CONFIG_HANGOVER              ,
	ACX_FEATURE_CFG                  ,
	ACX_PROTECTION_CFG               ,
};

enum {
	ACX_NS_IPV6_FILTER		 = 0x0050,
	ACX_PEER_HT_OPERATION_MODE_CFG	 = 0x0051,
	ACX_CSUM_CONFIG			 = 0x0052,
	ACX_SIM_CONFIG			 = 0x0053,
	ACX_CLEAR_STATISTICS		 = 0x0054,
	ACX_AUTO_RX_STREAMING		 = 0x0055,
	ACX_PEER_CAP			 = 0x0056,
	ACX_INTERRUPT_NOTIFY		 = 0x0057,
	ACX_RX_BA_FILTER		 = 0x0058,
	ACX_AP_SLEEP_CFG                 = 0x0059,
	ACX_DYNAMIC_TRACES_CFG		 = 0x005A,
	ACX_TIME_SYNC_CFG		 = 0x005B,
};

/*
 * ACX_DYNAMIC_TRACES_CFG
 * configure the FW dynamic traces
 */
struct acx_dynamic_fw_traces_cfg {
	struct acx_header header;
	__le32 dynamic_fw_traces;
} __packed;

/* numbers of bits the length field takes (add 1 for the actual number) */
#define CC33XX_HOST_IF_LEN_SIZE_FIELD 15

struct cc33xx_acx_host_config_bitmap {
	struct acx_header header;

	__le32 host_cfg_bitmap;

	__le32 host_sdio_block_size;

	/* extra mem blocks per frame in TX. */
	__le32 extra_mem_blocks;

	/*
	 * number of bits of the length field in the first TX word
	 * (up to 15 - for using the entire 16 bits).
	 */
	__le32 length_field_size;

} __packed;

struct cc33xx_acx_error_stats {
	__le32 error_frame_non_ctrl;
	__le32 error_frame_ctrl;
	__le32 error_frame_during_protection;
	__le32 null_frame_tx_start;
	__le32 null_frame_cts_start;
	__le32 bar_retry;
	__le32 num_frame_cts_nul_flid;
	__le32 tx_abort_failure;
	__le32 tx_resume_failure;
	__le32 rx_cmplt_db_overflow_cnt;
	__le32 elp_while_rx_exch;
	__le32 elp_while_tx_exch;
	__le32 elp_while_tx;
	__le32 elp_while_nvic_pending;
	__le32 rx_excessive_frame_len;
	__le32 burst_mismatch;
	__le32 tbc_exch_mismatch;
} __packed;

#define NUM_OF_RATES_INDEXES 30
struct cc33xx_acx_tx_stats {
	__le32 tx_prepared_descs;
	__le32 tx_cmplt;
	__le32 tx_template_prepared;
	__le32 tx_data_prepared;
	__le32 tx_template_programmed;
	__le32 tx_data_programmed;
	__le32 tx_burst_programmed;
	__le32 tx_starts;
	__le32 tx_stop;
	__le32 tx_start_templates;
	__le32 tx_start_int_templates;
	__le32 tx_start_fw_gen;
	__le32 tx_start_data;
	__le32 tx_start_null_frame;
	__le32 tx_exch;
	__le32 tx_retry_template;
	__le32 tx_retry_data;
	__le32 tx_retry_per_rate[NUM_OF_RATES_INDEXES];
	__le32 tx_exch_pending;
	__le32 tx_exch_expiry;
	__le32 tx_done_template;
	__le32 tx_done_data;
	__le32 tx_done_int_template;
	__le32 tx_cfe1;
	__le32 tx_cfe2;
	__le32 frag_called;
	__le32 frag_mpdu_alloc_failed;
	__le32 frag_init_called;
	__le32 frag_in_process_called;
	__le32 frag_tkip_called;
	__le32 frag_key_not_found;
	__le32 frag_need_fragmentation;
	__le32 frag_bad_mblk_num;
	__le32 frag_failed;
	__le32 frag_cache_hit;
	__le32 frag_cache_miss;
} __packed;

struct cc33xx_acx_rx_stats {
	__le32 rx_beacon_early_term;
	__le32 rx_out_of_mpdu_nodes;
	__le32 rx_hdr_overflow;
	__le32 rx_dropped_frame;
	__le32 rx_done_stage;
	__le32 rx_done;
	__le32 rx_defrag;
	__le32 rx_defrag_end;
	__le32 rx_cmplt;
	__le32 rx_pre_complt;
	__le32 rx_cmplt_task;
	__le32 rx_phy_hdr;
	__le32 rx_timeout;
	__le32 rx_rts_timeout;
	__le32 rx_timeout_wa;
	__le32 defrag_called;
	__le32 defrag_init_called;
	__le32 defrag_in_process_called;
	__le32 defrag_tkip_called;
	__le32 defrag_need_defrag;
	__le32 defrag_decrypt_failed;
	__le32 decrypt_key_not_found;
	__le32 defrag_need_decrypt;
	__le32 rx_tkip_replays;
	__le32 rx_xfr;
} __packed;

struct cc33xx_acx_isr_stats {
	__le32 irqs;
} __packed;

#define PWR_STAT_MAX_CONT_MISSED_BCNS_SPREAD 10

struct cc33xx_acx_pwr_stats {
	__le32 missing_bcns_cnt;
	__le32 rcvd_bcns_cnt;
	__le32 connection_out_of_sync;
	__le32 cont_miss_bcns_spread[PWR_STAT_MAX_CONT_MISSED_BCNS_SPREAD];
	__le32 rcvd_awake_bcns_cnt;
	__le32 sleep_time_count;
	__le32 sleep_time_avg;
	__le32 sleep_cycle_avg;
	__le32 sleep_percent;
	__le32 ap_sleep_active_conf;
	__le32 ap_sleep_user_conf;
	__le32 ap_sleep_counter;
} __packed;

struct cc33xx_acx_rx_filter_stats {
	__le32 beacon_filter;
	__le32 arp_filter;
	__le32 mc_filter;
	__le32 dup_filter;
	__le32 data_filter;
	__le32 ibss_filter;
	__le32 protection_filter;
	__le32 accum_arp_pend_requests;
	__le32 max_arp_queue_dep;
} __packed;

struct cc33xx_acx_rx_rate_stats {
	__le32 rx_frames_per_rates[50];
} __packed;

#define AGGR_STATS_TX_AGG	16
#define AGGR_STATS_RX_SIZE_LEN	16

struct cc33xx_acx_aggr_stats {
	__le32 tx_agg_rate[AGGR_STATS_TX_AGG];
	__le32 tx_agg_len[AGGR_STATS_TX_AGG];
	__le32 rx_size[AGGR_STATS_RX_SIZE_LEN];
} __packed;

#define PIPE_STATS_HW_FIFO	11

struct cc33xx_acx_pipeline_stats {
	__le32 hs_tx_stat_fifo_int;
	__le32 hs_rx_stat_fifo_int;
	__le32 enc_tx_stat_fifo_int;
	__le32 enc_rx_stat_fifo_int;
	__le32 rx_complete_stat_fifo_int;
	__le32 pre_proc_swi;
	__le32 post_proc_swi;
	__le32 sec_frag_swi;
	__le32 pre_to_defrag_swi;
	__le32 defrag_to_rx_xfer_swi;
	__le32 dec_packet_in;
	__le32 dec_packet_in_fifo_full;
	__le32 dec_packet_out;
	__le16 pipeline_fifo_full[PIPE_STATS_HW_FIFO];
	__le16 padding;
} __packed;

#define DIVERSITY_STATS_NUM_OF_ANT	2

struct cc33xx_acx_diversity_stats {
	__le32 num_of_packets_per_ant[DIVERSITY_STATS_NUM_OF_ANT];
	__le32 total_num_of_toggles;
} __packed;

struct cc33xx_acx_thermal_stats {
	__le16 irq_thr_low;
	__le16 irq_thr_high;
	__le16 tx_stop;
	__le16 tx_resume;
	__le16 false_irq;
	__le16 adc_source_unexpected;
} __packed;

#define CC33XX_NUM_OF_CALIBRATIONS_ERRORS 18
struct cc33xx_acx_calib_failure_stats {
	__le16 fail_count[CC33XX_NUM_OF_CALIBRATIONS_ERRORS];
	__le32 calib_count;
} __packed;

struct cc33xx_roaming_stats {
	s32 rssi_level;
} __packed;

struct cc33xx_dfs_stats {
	__le32 num_of_radar_detections;
} __packed;

struct cc33xx_acx_statistics_t {
	struct acx_header header;

	struct cc33xx_acx_error_stats		error;
	struct cc33xx_acx_tx_stats		tx;
	struct cc33xx_acx_rx_stats		rx;
	struct cc33xx_acx_isr_stats		isr;
	struct cc33xx_acx_pwr_stats		pwr;
	struct cc33xx_acx_rx_filter_stats	rx_filter;
	struct cc33xx_acx_rx_rate_stats		rx_rate;
	struct cc33xx_acx_aggr_stats		aggr_size;
	struct cc33xx_acx_pipeline_stats	pipeline;
	struct cc33xx_acx_diversity_stats	diversity;
	struct cc33xx_acx_thermal_stats		thermal;
	struct cc33xx_acx_calib_failure_stats	calib;
	struct cc33xx_roaming_stats		roaming;
	struct cc33xx_dfs_stats			dfs;
} __packed;

enum wlcore_bandwidth {
	WLCORE_BANDWIDTH_20MHZ,
	WLCORE_BANDWIDTH_40MHZ,
};

struct wlcore_peer_ht_operation_mode {
	struct acx_header header;

	u8 hlid;
	u8 bandwidth; /* enum wlcore_bandwidth */
	u8 padding[2];
};

/*
 * ACX_PEER_CAP
 * this struct is very similar to cc33xx_acx_ht_capabilities, with the
 * addition of supported rates
 */
#define NOMINAL_PACKET_PADDING (0xC0)
struct wlcore_acx_peer_cap {
	struct acx_header header;

	u8 role_id;

	/* rates supported by the remote peer */
	__le32 supported_rates;

	/* bitmask of capability bits supported by the peer */
	__le32 ht_capabilites;
	/*
	* This the maximum A-MPDU length supported by the AP. The FW may not
	* exceed this length when sending A-MPDUs
	*/
	u8 ampdu_max_length;

	/* This is the minimal spacing required when sending A-MPDUs to the AP*/
	u8 ampdu_min_spacing;

	/* HE capabilities */
	u8 mac_cap_info[8];

	/* Nominal packet padding value, used for determining the packet extension duration */
	u8 nominal_packet_padding;

	/* HE peer support */
	bool has_he;

	u8 dcm_max_constelation;

	u8 er_upper_supported;

	u8 paddin[1];
} __packed;

/*
 * ACX_INTERRUPT_NOTIFY
 * enable/disable fast-link/PSM notification from FW
 */
struct cc33xx_acx_interrupt_notify {
	struct acx_header header;
	__le32 enable;
};

/*
 * ACX_RX_BA_FILTER
 * enable/disable RX BA filtering in FW
 */
struct cc33xx_acx_rx_ba_filter {
	struct acx_header header;
	__le32 enable;
};

struct acx_ap_sleep_cfg {
	struct acx_header header;
	/* Duty Cycle (20-80% of staying Awake) for IDLE AP
	 * (0: disable)
	 */
	u8 idle_duty_cycle;
	/* Duty Cycle (20-80% of staying Awake) for Connected AP
	 * (0: disable)
	 */
	u8 connected_duty_cycle;
	/* Maximum stations that are allowed to be connected to AP
	 *  (255: no limit)
	 */
	u8 max_stations_thresh;
	/* Timeout till enabling the Sleep Mechanism after data stops
	 * [unit: 100 msec]
	 */
	u8 idle_conn_thresh;
} __packed;

struct acx_antenna_select {
	struct acx_header header;

	u8 selection;
	u8 padding[3];
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
	u8 padding [3];
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

static const u16 cc33xx_idx_to_rate_100Kbps[] =
{
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

struct acx_antenna_diversity_enable {
	struct acx_header header;

    u8 diversity_enable;
	u8 padding[3];
} __packed;

struct acx_antenna_diversity_rssi_threshold {
	struct acx_header header;

    s8 rssi_threshold;
	u8 padding[3];
} __packed;

struct acx_antenna_diversity_select_default_antenna {
	struct acx_header header;

    u8 default_antenna;
	u8 padding[3];
} __packed;


int cc33xx_acx_wake_up_conditions(struct cc33xx *wl,
				  struct cc33xx_vif *wlvif,
				  u8 wake_up_event, u8 listen_interval);
int cc33xx_acx_sleep_auth(struct cc33xx *wl, u8 sleep_auth);
int cc33xx_ble_enable(struct cc33xx *wl, u8 ble_enable);
int cc33xx_acx_tx_power(struct cc33xx *wl, struct cc33xx_vif *wlvif, int power);
int cc33xx_acx_feature_cfg(struct cc33xx *wl, struct cc33xx_vif *wlvif);
int cc33xx_acx_slot(struct cc33xx *wl, struct cc33xx_vif *wlvif,
		    enum acx_slot_type slot_time);
int cc33xx_acx_group_address_tbl(struct cc33xx *wl, struct cc33xx_vif *wlvif,
				 bool enable, void *mc_list, u32 mc_list_len);
int cc33xx_acx_service_period_timeout(struct cc33xx *wl,
				      struct cc33xx_vif *wlvif);
int cc33xx_acx_rts_threshold(struct cc33xx *wl, struct cc33xx_vif *wlvif,
			     u32 rts_threshold);
int cc33xx_acx_beacon_filter_opt(struct cc33xx *wl, struct cc33xx_vif *wlvif,
				 bool enable_filter);
int cc33xx_acx_beacon_filter_table(struct cc33xx *wl, struct cc33xx_vif *wlvif);
int cc33xx_acx_conn_monit_params(struct cc33xx *wl, struct cc33xx_vif *wlvif,
				 bool enable);
int cc33xx_acx_bcn_dtim_options(struct cc33xx *wl, struct cc33xx_vif *wlvif);
int cc33xx_assoc_info_cfg(struct cc33xx *wl, struct cc33xx_vif *wlvif,
			  struct ieee80211_sta *sta,u16 aid);
int cc33xx_acx_event_mbox_mask(struct cc33xx *wl, u32 event_mask);
int cc33xx_acx_set_preamble(struct cc33xx *wl, struct cc33xx_vif *wlvif,
			    enum acx_preamble_type preamble);
int cc33xx_acx_cts_protect(struct cc33xx *wl, struct cc33xx_vif *wlvif,
			   enum acx_ctsprotect_type ctsprotect);
int cc33xx_acx_statistics(struct cc33xx *wl, void *stats);
int cc33xx_tx_param_cfg(struct cc33xx *wl, struct cc33xx_vif *wlvif, u8 ac,
			u8 cw_min, u16 cw_max, u8 aifsn, u16 txop, bool acm,
			u8 ps_scheme, u8 is_mu_edca, u8 mu_edca_aifs,
			u8 mu_edca_ecw_min_max, u8 mu_edca_timer);
int cc33xx_update_ap_rates(struct cc33xx *wl, u8 role_id, u32 basic_rates_set,
			   u32 supported_rates);
int cc33xx_acx_frag_threshold(struct cc33xx *wl, u32 frag_threshold);
int cc33xx_acx_mem_cfg(struct cc33xx *wl);
int cc33xx_acx_init_mem_config(struct cc33xx *wl);
int cc33xx_acx_init_get_fw_versions(struct cc33xx *wl);
int cc33xx_acx_init_rx_interrupt(struct cc33xx *wl);
int cc33xx_acx_bet_enable(struct cc33xx *wl,
			  struct cc33xx_vif *wlvif, bool enable);
int cc33xx_acx_arp_ip_filter(struct cc33xx *wl, struct cc33xx_vif *wlvif,
			     u8 enable, __be32 address);
int cc33xx_acx_rssi_snr_trigger(struct cc33xx *wl, struct cc33xx_vif *wlvif,
				bool enable, s16 thold, u8 hyst);
int cc33xx_acx_rssi_snr_avg_weights(struct cc33xx *wl,
				    struct cc33xx_vif *wlvif);
int cc33xx_acx_set_ht_capabilities(struct cc33xx *wl,
				   struct ieee80211_sta_ht_cap *ht_cap,
				   bool allow_ht_operation, u8 hlid);
int cc33xx_acx_set_ht_information(struct cc33xx *wl, struct cc33xx_vif *wlvif,
				  u16 ht_operation_mode, u32 he_oper_params,
				  u16 he_oper_nss_set);
int cc33xx_acx_set_ba_initiator_policy(struct cc33xx *wl,
				       struct cc33xx_vif *wlvif);
int cc33xx_acx_set_ba_receiver_session(struct cc33xx *wl, u8 tid_index, u16 ssn,
				       bool enable, u8 peer_hlid, u8 win_size);
int cc33xx_acx_static_calibration_configure(struct cc33xx *wl,
					    struct calibration_file_header *file_header,
					    u8 *calibration_entry_ptr,
					    bool valid_data);
int cc33xx_acx_tsf_info(struct cc33xx *wl,
			struct cc33xx_vif *wlvif, u64 *mactime);
int cc33xx_acx_ps_rx_streaming(struct cc33xx *wl,
			       struct cc33xx_vif *wlvif, bool enable);
int cc33xx_acx_ap_max_tx_retry(struct cc33xx *wl, struct cc33xx_vif *wlvif);
int cc33xx_acx_config_ps(struct cc33xx *wl, struct cc33xx_vif *wlvif);
int cc33xx_acx_set_inconnection_sta(struct cc33xx *wl,
				    struct cc33xx_vif *wlvif, u8 *addr);
int wlcore_acx_get_tx_rate(struct cc33xx *wl, struct cc33xx_vif *wlvif,
			   struct station_info *sinfo);
int wlcore_acx_average_rssi(struct cc33xx *wl,
			    struct cc33xx_vif *wlvif, s8 *avg_rssi);
int cc33xx_acx_default_rx_filter_enable(struct cc33xx *wl, bool enable,
					enum rx_filter_action action);
int cc33xx_acx_set_rx_filter(struct cc33xx *wl, u8 index, bool enable,
			     struct cc33xx_rx_filter *filter);
int cc33xx_acx_dynamic_fw_traces(struct cc33xx *wl);
int cc33xx_acx_clear_statistics(struct cc33xx *wl);
int cc33xx_acx_host_if_cfg_bitmap(struct cc33xx *wl, u32 host_cfg_bitmap,
				  u32 sdio_blk_size, u32 extra_mem_blks,
				  u32 len_field_size);
int cc33xx_acx_peer_ht_operation_mode(struct cc33xx *wl, u8 hlid, bool wide);
int cc33xx_acx_set_peer_cap(struct cc33xx *wl,
			    struct ieee80211_sta_ht_cap *ht_cap,
			    struct ieee80211_sta_he_cap *he_cap,
			    struct cc33xx_vif *wlvif, bool allow_ht_operation,
			    u32 rate_set, u8 hlid);
int cc33xx_acx_interrupt_notify_config(struct cc33xx *wl, bool action);
int cc33xx_acx_rx_ba_filter(struct cc33xx *wl, bool action);
int cc33xx_acx_ap_sleep(struct cc33xx *wl);
int cc33xx_acx_set_antenna_select(struct cc33xx *wl, u8 selection);
int cc33xx_acx_set_tsf(struct cc33xx *wl, u64 tsf_val);
int cc33xx_acx_trigger_fw_assert(struct cc33xx *wl);
int cc33xx_acx_burst_mode_cfg(struct cc33xx *wl, u8 burst_disable);
int cc33xx_acx_antenna_diversity_enable(struct cc33xx *wl, u8 diversity_enable);
int cc33xx_acx_antenna_diversity_set_rssi_threshold(struct cc33xx *wl, s8 rssi_threshold);
int cc33xx_acx_antenna_diversity_select_default_antenna(struct cc33xx *wl, u8 default_antenna);

int cc33xx_acx_twt_setup(struct cc33xx *wl, u32 min_wake_duration_usec,
               u32 min_wake_interval_mantissa, u32 min_wake_interval_exponent, u32 max_wake_interval_mantissa, u32 max_wake_interval_exponent, u8 valid_params);
int cc33xx_acx_twt_terminate(struct cc33xx *wl);
int cc33xx_acx_twt_resume(struct cc33xx *wl);
int cc33xx_acx_twt_suspend(struct cc33xx *wl);


#endif /* __CC33XX_ACX_H__ */
