/*
 * Linux cfg80211 Vendor Extension Code
 *
 * Copyright (C) 2020, Broadcom.
 *
 *      Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 *
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 *
 *
 * <<Broadcom-WL-IPTag/Dual:>>
 */

/*
 * New vendor interface additon to nl80211/cfg80211 to allow vendors
 * to implement proprietary features over the cfg80211 stack.
 */

#ifndef _wl_cfgvendor_h_
#define _wl_cfgvendor_h_

#define OUI_BRCM    0x001018
#define OUI_GOOGLE  0x001A11
#define BRCM_VENDOR_SUBCMD_PRIV_STR	1
#define ATTRIBUTE_U32_LEN                  (NLA_HDRLEN  + 4)
#define VENDOR_ID_OVERHEAD                 ATTRIBUTE_U32_LEN
#define VENDOR_SUBCMD_OVERHEAD             ATTRIBUTE_U32_LEN
#define VENDOR_DATA_OVERHEAD               (NLA_HDRLEN)
#define ETHERTYPE_IP            0x0800          /* IP */
#define ETHERTYPE_IPV6          0x86dd          /* IP protocol version 6 */

enum brcm_vendor_attr {
	BRCM_ATTR_DRIVER_CMD		= 0,
	BRCM_ATTR_DRIVER_KEY_PMK	= 1,
	BRCM_ATTR_DRIVER_FEATURE_FLAGS	= 2,
	BRCM_ATTR_DRIVER_RAND_MAC	= 3,
	BRCM_ATTR_SAE_PWE		= 4,
	BRCM_ATTR_DRIVER_MAX		= 5
};

enum brcm_wlan_vendor_features {
	BRCM_WLAN_VENDOR_FEATURE_KEY_MGMT_OFFLOAD	= 0,
	BRCM_WLAN_VENDOR_FEATURES_MAX			= 1
};

typedef enum wifi_error {
	WIFI_SUCCESS = 0,
	WIFI_ERROR_NONE = 0,
	WIFI_ERROR_UNKNOWN = -1,
	WIFI_ERROR_UNINITIALIZED = -2,
	WIFI_ERROR_NOT_SUPPORTED = -3,
	WIFI_ERROR_NOT_AVAILABLE = -4,
	WIFI_ERROR_INVALID_ARGS = -5,
	WIFI_ERROR_INVALID_REQUEST_ID = -6,
	WIFI_ERROR_TIMED_OUT = -7,
	WIFI_ERROR_TOO_MANY_REQUESTS = -8,
	WIFI_ERROR_OUT_OF_MEMORY = -9,
	WIFI_ERROR_BUSY = -10
} wifi_error_t;

#define SCAN_RESULTS_COMPLETE_FLAG_LEN       ATTRIBUTE_U32_LEN
#define SCAN_INDEX_HDR_LEN                   (NLA_HDRLEN)
#define SCAN_ID_HDR_LEN                      ATTRIBUTE_U32_LEN
#define SCAN_FLAGS_HDR_LEN                   ATTRIBUTE_U32_LEN
#define GSCAN_NUM_RESULTS_HDR_LEN            ATTRIBUTE_U32_LEN
#define GSCAN_CH_BUCKET_MASK_HDR_LEN         ATTRIBUTE_U32_LEN
#define GSCAN_RESULTS_HDR_LEN                (NLA_HDRLEN)
#define GSCAN_BATCH_RESULT_HDR_LEN  (SCAN_INDEX_HDR_LEN + SCAN_ID_HDR_LEN + \
									SCAN_FLAGS_HDR_LEN + \
							        GSCAN_NUM_RESULTS_HDR_LEN + \
								GSCAN_CH_BUCKET_MASK_HDR_LEN + \
									GSCAN_RESULTS_HDR_LEN)

#define VENDOR_REPLY_OVERHEAD       (VENDOR_ID_OVERHEAD + \
									VENDOR_SUBCMD_OVERHEAD + \
									VENDOR_DATA_OVERHEAD)

#define GSCAN_ATTR_SET1				10
#define GSCAN_ATTR_SET2				20
#define GSCAN_ATTR_SET3				30
#define GSCAN_ATTR_SET4				40
#define GSCAN_ATTR_SET5				50
#define GSCAN_ATTR_SET6				60
#define GSCAN_ATTR_SET7				70
#define GSCAN_ATTR_SET8				80
#define GSCAN_ATTR_SET9				90
#define GSCAN_ATTR_SET10			100
#define GSCAN_ATTR_SET11			110
#define GSCAN_ATTR_SET12			120
#define GSCAN_ATTR_SET13			130
#define GSCAN_ATTR_SET14			140

#define NAN_SVC_INFO_LEN			255
#define NAN_SID_ENABLE_FLAG_INVALID	0xff
#define NAN_SID_BEACON_COUNT_INVALID	0xff
#define WL_NAN_DW_INTERVAL 512

#define CFG80211_VENDOR_CMD_REPLY_SKB_SZ	100
#define CFG80211_VENDOR_EVT_SKB_SZ			2048

#define SUPP_SAE_PWE_LOOP	0x00
#define SUPP_SAE_PWE_H2E	0x01
#define SUPP_SAE_PWE_TRANS	0x02

typedef enum {
	/* don't use 0 as a valid subcommand */
	VENDOR_NL80211_SUBCMD_UNSPECIFIED,

	/* define all vendor startup commands between 0x0 and 0x0FFF */
	VENDOR_NL80211_SUBCMD_RANGE_START = 0x0001,
	VENDOR_NL80211_SUBCMD_RANGE_END   = 0x0FFF,

	/* define all GScan related commands between 0x1000 and 0x10FF */
	ANDROID_NL80211_SUBCMD_GSCAN_RANGE_START = 0x1000,
	ANDROID_NL80211_SUBCMD_GSCAN_RANGE_END   = 0x10FF,

	/* define all RTT related commands between 0x1100 and 0x11FF */
	ANDROID_NL80211_SUBCMD_RTT_RANGE_START = 0x1100,
	ANDROID_NL80211_SUBCMD_RTT_RANGE_END   = 0x11FF,

	ANDROID_NL80211_SUBCMD_LSTATS_RANGE_START = 0x1200,
	ANDROID_NL80211_SUBCMD_LSTATS_RANGE_END   = 0x12FF,

	ANDROID_NL80211_SUBCMD_TDLS_RANGE_START = 0x1300,
	ANDROID_NL80211_SUBCMD_TDLS_RANGE_END	= 0x13FF,

	ANDROID_NL80211_SUBCMD_DEBUG_RANGE_START = 0x1400,
	ANDROID_NL80211_SUBCMD_DEBUG_RANGE_END	= 0x14FF,

	/* define all NearbyDiscovery related commands between 0x1500 and 0x15FF */
	ANDROID_NL80211_SUBCMD_NBD_RANGE_START = 0x1500,
	ANDROID_NL80211_SUBCMD_NBD_RANGE_END   = 0x15FF,

	/* define all wifi calling related commands between 0x1600 and 0x16FF */
	ANDROID_NL80211_SUBCMD_WIFI_OFFLOAD_RANGE_START = 0x1600,
	ANDROID_NL80211_SUBCMD_WIFI_OFFLOAD_RANGE_END	= 0x16FF,

	/* define all NAN related commands between 0x1700 and 0x17FF */
	ANDROID_NL80211_SUBCMD_NAN_RANGE_START = 0x1700,
	ANDROID_NL80211_SUBCMD_NAN_RANGE_END   = 0x17FF,

	/* define all packet filter related commands between 0x1800 and 0x18FF */
	ANDROID_NL80211_SUBCMD_PKT_FILTER_RANGE_START = 0x1800,
	ANDROID_NL80211_SUBCMD_PKT_FILTER_RANGE_END   = 0x18FF,

	/* define all tx power related commands between 0x1900 and 0x19FF */
	ANDROID_NL80211_SUBCMD_TX_POWER_RANGE_START =	0x1900,
	ANDROID_NL80211_SUBCMD_TX_POWER_RANGE_END =	0x19FF,

	/* define all TWT related commands between 0x2140 and 0x214F */
	ANDROID_NL80211_SUBCMD_TWT_START	=	0x2140,
	ANDROID_NL80211_SUBCMD_TWT_END		=	0x214F,

	/* This is reserved for future usage */

} ANDROID_VENDOR_SUB_COMMAND;

enum andr_vendor_subcmd {
	GSCAN_SUBCMD_GET_CAPABILITIES = ANDROID_NL80211_SUBCMD_GSCAN_RANGE_START,
	GSCAN_SUBCMD_SET_CONFIG,
	GSCAN_SUBCMD_SET_SCAN_CONFIG,
	GSCAN_SUBCMD_ENABLE_GSCAN,
	GSCAN_SUBCMD_GET_SCAN_RESULTS,
	GSCAN_SUBCMD_SCAN_RESULTS,
	GSCAN_SUBCMD_SET_HOTLIST,
	GSCAN_SUBCMD_SET_SIGNIFICANT_CHANGE_CONFIG,
	GSCAN_SUBCMD_ENABLE_FULL_SCAN_RESULTS,
	GSCAN_SUBCMD_GET_CHANNEL_LIST,
	/* ANDR_WIFI_XXX although not related to gscan are defined here */
	ANDR_WIFI_SUBCMD_GET_FEATURE_SET,
	ANDR_WIFI_SUBCMD_GET_FEATURE_SET_MATRIX,
	ANDR_WIFI_RANDOM_MAC_OUI,
	ANDR_WIFI_NODFS_CHANNELS,
	ANDR_WIFI_SET_COUNTRY,
	GSCAN_SUBCMD_SET_EPNO_SSID,
	WIFI_SUBCMD_SET_SSID_WHITELIST,
	WIFI_SUBCMD_SET_LAZY_ROAM_PARAMS,
	WIFI_SUBCMD_ENABLE_LAZY_ROAM,
	WIFI_SUBCMD_SET_BSSID_PREF,
	WIFI_SUBCMD_SET_BSSID_BLACKLIST,
	GSCAN_SUBCMD_ANQPO_CONFIG,
	WIFI_SUBCMD_SET_RSSI_MONITOR,
	WIFI_SUBCMD_CONFIG_ND_OFFLOAD,
	WIFI_SUBCMD_CONFIG_TCPACK_SUP,
	WIFI_SUBCMD_FW_ROAM_POLICY,
	WIFI_SUBCMD_ROAM_CAPABILITY,
	WIFI_SUBCMD_SET_LATENCY_MODE,
	RTT_SUBCMD_SET_CONFIG = ANDROID_NL80211_SUBCMD_RTT_RANGE_START,
	RTT_SUBCMD_CANCEL_CONFIG,
	RTT_SUBCMD_GETCAPABILITY,
	RTT_SUBCMD_GETAVAILCHANNEL,
	RTT_SUBCMD_SET_RESPONDER,
	RTT_SUBCMD_CANCEL_RESPONDER,
	LSTATS_SUBCMD_GET_INFO = ANDROID_NL80211_SUBCMD_LSTATS_RANGE_START,

	DEBUG_START_LOGGING = ANDROID_NL80211_SUBCMD_DEBUG_RANGE_START,
	DEBUG_TRIGGER_MEM_DUMP,
	DEBUG_GET_MEM_DUMP,
	DEBUG_GET_VER,
	DEBUG_GET_RING_STATUS,
	DEBUG_GET_RING_DATA,
	DEBUG_GET_FEATURE,
	DEBUG_RESET_LOGGING,

	DEBUG_TRIGGER_DRIVER_MEM_DUMP,
	DEBUG_GET_DRIVER_MEM_DUMP,
	DEBUG_START_PKT_FATE_MONITORING,
	DEBUG_GET_TX_PKT_FATES,
	DEBUG_GET_RX_PKT_FATES,
	DEBUG_GET_WAKE_REASON_STATS,
	DEBUG_GET_FILE_DUMP_BUF,
	DEBUG_FILE_DUMP_DONE_IND,
	DEBUG_SET_HAL_START,
	DEBUG_SET_HAL_STOP,
	DEBUG_SET_HAL_PID,

	WIFI_OFFLOAD_SUBCMD_START_MKEEP_ALIVE = ANDROID_NL80211_SUBCMD_WIFI_OFFLOAD_RANGE_START,
	WIFI_OFFLOAD_SUBCMD_STOP_MKEEP_ALIVE,

	NAN_WIFI_SUBCMD_ENABLE = ANDROID_NL80211_SUBCMD_NAN_RANGE_START,	 /* 0x1700 */
	NAN_WIFI_SUBCMD_DISABLE,						 /* 0x1701 */
	NAN_WIFI_SUBCMD_REQUEST_PUBLISH,					 /* 0x1702 */
	NAN_WIFI_SUBCMD_REQUEST_SUBSCRIBE,					 /* 0x1703 */
	NAN_WIFI_SUBCMD_CANCEL_PUBLISH,						 /* 0x1704 */
	NAN_WIFI_SUBCMD_CANCEL_SUBSCRIBE,					 /* 0x1705 */
	NAN_WIFI_SUBCMD_TRANSMIT,						 /* 0x1706 */
	NAN_WIFI_SUBCMD_CONFIG,							 /* 0x1707 */
	NAN_WIFI_SUBCMD_TCA,							 /* 0x1708 */
	NAN_WIFI_SUBCMD_STATS,							 /* 0x1709 */
	NAN_WIFI_SUBCMD_GET_CAPABILITIES,					 /* 0x170A */
	NAN_WIFI_SUBCMD_DATA_PATH_IFACE_CREATE,					 /* 0x170B */
	NAN_WIFI_SUBCMD_DATA_PATH_IFACE_DELETE,					 /* 0x170C */
	NAN_WIFI_SUBCMD_DATA_PATH_REQUEST,					 /* 0x170D */
	NAN_WIFI_SUBCMD_DATA_PATH_RESPONSE,					 /* 0x170E */
	NAN_WIFI_SUBCMD_DATA_PATH_END,						 /* 0x170F */
	NAN_WIFI_SUBCMD_DATA_PATH_SEC_INFO,					 /* 0x1710 */
	NAN_WIFI_SUBCMD_VERSION_INFO,						 /* 0x1711 */
	NAN_WIFI_SUBCMD_ENABLE_MERGE,						 /* 0x1712 */
	APF_SUBCMD_GET_CAPABILITIES = ANDROID_NL80211_SUBCMD_PKT_FILTER_RANGE_START,
	APF_SUBCMD_SET_FILTER,
	WIFI_SUBCMD_TX_POWER_SCENARIO = ANDROID_NL80211_SUBCMD_TX_POWER_RANGE_START,

	ANDR_TWT_SUBCMD_GET_CAP = ANDROID_NL80211_SUBCMD_TWT_START,
	ANDR_TWT_SUBCMD_SETUP,
	ANDR_TWT_SUBCMD_TEARDOWN,
	ANDR_TWT_SUBCMD_INFO_FRAME,
	ANDR_TWT_SUBCMD_GET_STATS,
	ANDR_TWT_SUBCMD_CLR_STATS,
	/* Add more sub commands here */
	VENDOR_SUBCMD_MAX
};

enum gscan_attributes {
    GSCAN_ATTRIBUTE_NUM_BUCKETS = GSCAN_ATTR_SET1,
    GSCAN_ATTRIBUTE_BASE_PERIOD,
    GSCAN_ATTRIBUTE_BUCKETS_BAND,
    GSCAN_ATTRIBUTE_BUCKET_ID,
    GSCAN_ATTRIBUTE_BUCKET_PERIOD,
    GSCAN_ATTRIBUTE_BUCKET_NUM_CHANNELS,
    GSCAN_ATTRIBUTE_BUCKET_CHANNELS,
    GSCAN_ATTRIBUTE_NUM_AP_PER_SCAN,
    GSCAN_ATTRIBUTE_REPORT_THRESHOLD,
    GSCAN_ATTRIBUTE_NUM_SCANS_TO_CACHE,
    GSCAN_ATTRIBUTE_BAND = GSCAN_ATTRIBUTE_BUCKETS_BAND,

    GSCAN_ATTRIBUTE_ENABLE_FEATURE = GSCAN_ATTR_SET2,
    GSCAN_ATTRIBUTE_SCAN_RESULTS_COMPLETE,
    GSCAN_ATTRIBUTE_FLUSH_FEATURE,
    GSCAN_ATTRIBUTE_ENABLE_FULL_SCAN_RESULTS,
    GSCAN_ATTRIBUTE_REPORT_EVENTS,
    /* remaining reserved for additional attributes */
    GSCAN_ATTRIBUTE_NUM_OF_RESULTS = GSCAN_ATTR_SET3,
    GSCAN_ATTRIBUTE_FLUSH_RESULTS,
    GSCAN_ATTRIBUTE_SCAN_RESULTS,                       /* flat array of wifi_scan_result */
    GSCAN_ATTRIBUTE_SCAN_ID,                            /* indicates scan number */
    GSCAN_ATTRIBUTE_SCAN_FLAGS,                         /* indicates if scan was aborted */
    GSCAN_ATTRIBUTE_AP_FLAGS,                           /* flags on significant change event */
    GSCAN_ATTRIBUTE_NUM_CHANNELS,
    GSCAN_ATTRIBUTE_CHANNEL_LIST,
    GSCAN_ATTRIBUTE_CH_BUCKET_BITMASK,

	/* remaining reserved for additional attributes */

    GSCAN_ATTRIBUTE_SSID = GSCAN_ATTR_SET4,
    GSCAN_ATTRIBUTE_BSSID,
    GSCAN_ATTRIBUTE_CHANNEL,
    GSCAN_ATTRIBUTE_RSSI,
    GSCAN_ATTRIBUTE_TIMESTAMP,
    GSCAN_ATTRIBUTE_RTT,
    GSCAN_ATTRIBUTE_RTTSD,

    /* remaining reserved for additional attributes */

    GSCAN_ATTRIBUTE_HOTLIST_BSSIDS = GSCAN_ATTR_SET5,
    GSCAN_ATTRIBUTE_RSSI_LOW,
    GSCAN_ATTRIBUTE_RSSI_HIGH,
    GSCAN_ATTRIBUTE_HOSTLIST_BSSID_ELEM,
    GSCAN_ATTRIBUTE_HOTLIST_FLUSH,
    GSCAN_ATTRIBUTE_HOTLIST_BSSID_COUNT,

    /* remaining reserved for additional attributes */
    GSCAN_ATTRIBUTE_RSSI_SAMPLE_SIZE = GSCAN_ATTR_SET6,
    GSCAN_ATTRIBUTE_LOST_AP_SAMPLE_SIZE,
    GSCAN_ATTRIBUTE_MIN_BREACHING,
    GSCAN_ATTRIBUTE_SIGNIFICANT_CHANGE_BSSIDS,
    GSCAN_ATTRIBUTE_SIGNIFICANT_CHANGE_FLUSH,

    /* EPNO */
    GSCAN_ATTRIBUTE_EPNO_SSID_LIST = GSCAN_ATTR_SET7,
    GSCAN_ATTRIBUTE_EPNO_SSID,
    GSCAN_ATTRIBUTE_EPNO_SSID_LEN,
    GSCAN_ATTRIBUTE_EPNO_RSSI,
    GSCAN_ATTRIBUTE_EPNO_FLAGS,
    GSCAN_ATTRIBUTE_EPNO_AUTH,
    GSCAN_ATTRIBUTE_EPNO_SSID_NUM,
    GSCAN_ATTRIBUTE_EPNO_FLUSH,

    /* Roam SSID Whitelist and BSSID pref */
    GSCAN_ATTRIBUTE_WHITELIST_SSID = GSCAN_ATTR_SET8,
    GSCAN_ATTRIBUTE_NUM_WL_SSID,
    GSCAN_ATTRIBUTE_WL_SSID_LEN,
    GSCAN_ATTRIBUTE_WL_SSID_FLUSH,
    GSCAN_ATTRIBUTE_WHITELIST_SSID_ELEM,
    GSCAN_ATTRIBUTE_NUM_BSSID,
    GSCAN_ATTRIBUTE_BSSID_PREF_LIST,
    GSCAN_ATTRIBUTE_BSSID_PREF_FLUSH,
    GSCAN_ATTRIBUTE_BSSID_PREF,
    GSCAN_ATTRIBUTE_RSSI_MODIFIER,

    /* Roam cfg */
    GSCAN_ATTRIBUTE_A_BAND_BOOST_THRESHOLD = GSCAN_ATTR_SET9,
    GSCAN_ATTRIBUTE_A_BAND_PENALTY_THRESHOLD,
    GSCAN_ATTRIBUTE_A_BAND_BOOST_FACTOR,
    GSCAN_ATTRIBUTE_A_BAND_PENALTY_FACTOR,
    GSCAN_ATTRIBUTE_A_BAND_MAX_BOOST,
    GSCAN_ATTRIBUTE_LAZY_ROAM_HYSTERESIS,
    GSCAN_ATTRIBUTE_ALERT_ROAM_RSSI_TRIGGER,
    GSCAN_ATTRIBUTE_LAZY_ROAM_ENABLE,

    /* BSSID blacklist */
    GSCAN_ATTRIBUTE_BSSID_BLACKLIST_FLUSH = GSCAN_ATTR_SET10,
    GSCAN_ATTRIBUTE_BLACKLIST_BSSID,

    GSCAN_ATTRIBUTE_ANQPO_HS_LIST = GSCAN_ATTR_SET11,
    GSCAN_ATTRIBUTE_ANQPO_HS_LIST_SIZE,
    GSCAN_ATTRIBUTE_ANQPO_HS_NETWORK_ID,
    GSCAN_ATTRIBUTE_ANQPO_HS_NAI_REALM,
    GSCAN_ATTRIBUTE_ANQPO_HS_ROAM_CONSORTIUM_ID,
    GSCAN_ATTRIBUTE_ANQPO_HS_PLMN,

    /* Adaptive scan attributes */
    GSCAN_ATTRIBUTE_BUCKET_STEP_COUNT = GSCAN_ATTR_SET12,
    GSCAN_ATTRIBUTE_BUCKET_MAX_PERIOD,

    /* ePNO cfg */
    GSCAN_ATTRIBUTE_EPNO_5G_RSSI_THR = GSCAN_ATTR_SET13,
    GSCAN_ATTRIBUTE_EPNO_2G_RSSI_THR,
    GSCAN_ATTRIBUTE_EPNO_INIT_SCORE_MAX,
    GSCAN_ATTRIBUTE_EPNO_CUR_CONN_BONUS,
    GSCAN_ATTRIBUTE_EPNO_SAME_NETWORK_BONUS,
    GSCAN_ATTRIBUTE_EPNO_SECURE_BONUS,
    GSCAN_ATTRIBUTE_EPNO_5G_BONUS,

    /* Android O Roaming features */
    GSCAN_ATTRIBUTE_ROAM_STATE_SET = GSCAN_ATTR_SET14,

    GSCAN_ATTRIBUTE_MAX
};

enum gscan_bucket_attributes {
	GSCAN_ATTRIBUTE_CH_BUCKET_1,
	GSCAN_ATTRIBUTE_CH_BUCKET_2,
	GSCAN_ATTRIBUTE_CH_BUCKET_3,
	GSCAN_ATTRIBUTE_CH_BUCKET_4,
	GSCAN_ATTRIBUTE_CH_BUCKET_5,
	GSCAN_ATTRIBUTE_CH_BUCKET_6,
	GSCAN_ATTRIBUTE_CH_BUCKET_7
};

enum gscan_ch_attributes {
	GSCAN_ATTRIBUTE_CH_ID_1,
	GSCAN_ATTRIBUTE_CH_ID_2,
	GSCAN_ATTRIBUTE_CH_ID_3,
	GSCAN_ATTRIBUTE_CH_ID_4,
	GSCAN_ATTRIBUTE_CH_ID_5,
	GSCAN_ATTRIBUTE_CH_ID_6,
	GSCAN_ATTRIBUTE_CH_ID_7
};

enum rtt_attributes {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)) || (ANDROID_VERSION >= 12)
	RTT_ATTRIBUTE_INVALID,
#endif
	RTT_ATTRIBUTE_TARGET_CNT,
	RTT_ATTRIBUTE_TARGET_INFO,
	RTT_ATTRIBUTE_TARGET_MAC,
	RTT_ATTRIBUTE_TARGET_TYPE,
	RTT_ATTRIBUTE_TARGET_PEER,
	RTT_ATTRIBUTE_TARGET_CHAN,
	RTT_ATTRIBUTE_TARGET_PERIOD,
	RTT_ATTRIBUTE_TARGET_NUM_BURST,
	RTT_ATTRIBUTE_TARGET_NUM_FTM_BURST,
	RTT_ATTRIBUTE_TARGET_NUM_RETRY_FTM,
	RTT_ATTRIBUTE_TARGET_NUM_RETRY_FTMR,
	RTT_ATTRIBUTE_TARGET_LCI,
	RTT_ATTRIBUTE_TARGET_LCR,
	RTT_ATTRIBUTE_TARGET_BURST_DURATION,
	RTT_ATTRIBUTE_TARGET_PREAMBLE,
	RTT_ATTRIBUTE_TARGET_BW,
	RTT_ATTRIBUTE_RESULTS_COMPLETE,
	RTT_ATTRIBUTE_RESULTS_PER_TARGET,
	RTT_ATTRIBUTE_RESULT_CNT,
	RTT_ATTRIBUTE_RESULT,
	RTT_ATTRIBUTE_RESULT_DETAIL,
	/* Add any new RTT_ATTRIBUTE prior to RTT_ATTRIBUTE_MAX */
	RTT_ATTRIBUTE_MAX
};

enum wifi_rssi_monitor_attr {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)) || (ANDROID_VERSION >= 12)
	RSSI_MONITOR_ATTRIBUTE_INVALID,
#endif
	RSSI_MONITOR_ATTRIBUTE_MAX_RSSI,
	RSSI_MONITOR_ATTRIBUTE_MIN_RSSI,
	RSSI_MONITOR_ATTRIBUTE_START,
	RSSI_MONITOR_ATTRIBUTE_MAX
};

enum wifi_sae_key_attr {
	BRCM_SAE_KEY_ATTR_PEER_MAC,
	BRCM_SAE_KEY_ATTR_PMK,
	BRCM_SAE_KEY_ATTR_PMKID
};

enum debug_attributes {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)) || (ANDROID_VERSION >= 12)
	DEBUG_ATTRIBUTE_INVALID,
#endif
	DEBUG_ATTRIBUTE_GET_DRIVER,
	DEBUG_ATTRIBUTE_GET_FW,
	DEBUG_ATTRIBUTE_RING_ID,
	DEBUG_ATTRIBUTE_RING_NAME,
	DEBUG_ATTRIBUTE_RING_FLAGS,
	DEBUG_ATTRIBUTE_LOG_LEVEL,
	DEBUG_ATTRIBUTE_LOG_TIME_INTVAL,
	DEBUG_ATTRIBUTE_LOG_MIN_DATA_SIZE,
	DEBUG_ATTRIBUTE_FW_DUMP_LEN,
	DEBUG_ATTRIBUTE_FW_DUMP_DATA,
	DEBUG_ATTRIBUTE_FW_ERR_CODE,
	DEBUG_ATTRIBUTE_RING_DATA,
	DEBUG_ATTRIBUTE_RING_STATUS,
	DEBUG_ATTRIBUTE_RING_NUM,
	DEBUG_ATTRIBUTE_DRIVER_DUMP_LEN,
	DEBUG_ATTRIBUTE_DRIVER_DUMP_DATA,
	DEBUG_ATTRIBUTE_PKT_FATE_NUM,
	DEBUG_ATTRIBUTE_PKT_FATE_DATA,
	DEBUG_ATTRIBUTE_HANG_REASON,
	/* Please add new attributes from here to sync up old HAL */
	DEBUG_ATTRIBUTE_MAX
};

typedef enum {
	DUMP_LEN_ATTR_INVALID = 0,
	DUMP_LEN_ATTR_MEMDUMP = 1,
	DUMP_LEN_ATTR_SSSR_C0_D11_BEFORE = 2,
	DUMP_LEN_ATTR_SSSR_C0_D11_AFTER = 3,
	DUMP_LEN_ATTR_SSSR_C1_D11_BEFORE = 4,
	DUMP_LEN_ATTR_SSSR_C1_D11_AFTER = 5,
	DUMP_LEN_ATTR_SSSR_C2_D11_BEFORE = 6,
	DUMP_LEN_ATTR_SSSR_C2_D11_AFTER = 7,
	DUMP_LEN_ATTR_SSSR_DIG_BEFORE = 8,
	DUMP_LEN_ATTR_SSSR_DIG_AFTER = 9,
	DUMP_LEN_ATTR_TIMESTAMP = 10,
	DUMP_LEN_ATTR_GENERAL_LOG = 11,
	DUMP_LEN_ATTR_ECNTRS = 12,
	DUMP_LEN_ATTR_SPECIAL_LOG = 13,
	DUMP_LEN_ATTR_DHD_DUMP = 14,
	DUMP_LEN_ATTR_EXT_TRAP = 15,
	DUMP_LEN_ATTR_HEALTH_CHK = 16,
	DUMP_LEN_ATTR_PRESERVE_LOG = 17,
	DUMP_LEN_ATTR_COOKIE = 18,
	DUMP_LEN_ATTR_FLOWRING_DUMP = 19,
	DUMP_LEN_ATTR_PKTLOG = 20,
	DUMP_LEN_ATTR_PKTLOG_DEBUG = 21,
	DUMP_FILENAME_ATTR_DEBUG_DUMP = 22,
	DUMP_FILENAME_ATTR_MEM_DUMP = 23,
	DUMP_FILENAME_ATTR_SSSR_CORE_0_BEFORE_DUMP = 24,
	DUMP_FILENAME_ATTR_SSSR_CORE_0_AFTER_DUMP = 25,
	DUMP_FILENAME_ATTR_SSSR_CORE_1_BEFORE_DUMP = 26,
	DUMP_FILENAME_ATTR_SSSR_CORE_1_AFTER_DUMP = 27,
	DUMP_FILENAME_ATTR_SSSR_CORE_2_BEFORE_DUMP = 28,
	DUMP_FILENAME_ATTR_SSSR_CORE_2_AFTER_DUMP = 29,
	DUMP_FILENAME_ATTR_SSSR_DIG_BEFORE_DUMP = 30,
	DUMP_FILENAME_ATTR_SSSR_DIG_AFTER_DUMP = 31,
	DUMP_FILENAME_ATTR_PKTLOG_DUMP = 32,
	DUMP_FILENAME_ATTR_PKTLOG_DEBUG_DUMP = 33,
	DUMP_LEN_ATTR_STATUS_LOG = 34,
	DUMP_LEN_ATTR_AXI_ERROR = 35,
	DUMP_FILENAME_ATTR_AXI_ERROR_DUMP = 36,
	DUMP_LEN_ATTR_RTT_LOG = 37
	/* Please add new attributes from here to sync up old HAL */
} EWP_DUMP_EVENT_ATTRIBUTE;

/* Attributes associated with DEBUG_GET_DUMP_BUF */
typedef enum {
	DUMP_BUF_ATTR_INVALID = 0,
	DUMP_BUF_ATTR_MEMDUMP = 1,
	DUMP_BUF_ATTR_SSSR_C0_D11_BEFORE = 2,
	DUMP_BUF_ATTR_SSSR_C0_D11_AFTER = 3,
	DUMP_BUF_ATTR_SSSR_C1_D11_BEFORE = 4,
	DUMP_BUF_ATTR_SSSR_C1_D11_AFTER = 5,
	DUMP_BUF_ATTR_SSSR_C2_D11_BEFORE = 6,
	DUMP_BUF_ATTR_SSSR_C2_D11_AFTER = 7,
	DUMP_BUF_ATTR_SSSR_DIG_BEFORE = 8,
	DUMP_BUF_ATTR_SSSR_DIG_AFTER = 9,
	DUMP_BUF_ATTR_TIMESTAMP = 10,
	DUMP_BUF_ATTR_GENERAL_LOG = 11,
	DUMP_BUF_ATTR_ECNTRS = 12,
	DUMP_BUF_ATTR_SPECIAL_LOG = 13,
	DUMP_BUF_ATTR_DHD_DUMP = 14,
	DUMP_BUF_ATTR_EXT_TRAP = 15,
	DUMP_BUF_ATTR_HEALTH_CHK = 16,
	DUMP_BUF_ATTR_PRESERVE_LOG = 17,
	DUMP_BUF_ATTR_COOKIE = 18,
	DUMP_BUF_ATTR_FLOWRING_DUMP = 19,
	DUMP_BUF_ATTR_PKTLOG = 20,
	DUMP_BUF_ATTR_PKTLOG_DEBUG = 21,
	DUMP_BUF_ATTR_STATUS_LOG = 22,
	DUMP_BUF_ATTR_AXI_ERROR = 23,
	DUMP_BUF_ATTR_RTT_LOG = 24,
	DUMP_BUF_ATTR_SDTC_ETB_DUMP = 25,
	DUMP_BUF_ATTR_PKTID_MAP_LOG = 26,
	DUMP_BUF_ATTR_PKTID_UNMAP_LOG = 27,
	/* Please add new attributes from here to sync up old HAL */
	DUMP_BUF_ATTR_MAX
} EWP_DUMP_CMD_ATTRIBUTE;

enum mkeep_alive_attributes {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)) || (ANDROID_VERSION >= 12)
	MKEEP_ALIVE_ATTRIBUTE_INVALID,
#endif
	MKEEP_ALIVE_ATTRIBUTE_ID,
	MKEEP_ALIVE_ATTRIBUTE_IP_PKT,
	MKEEP_ALIVE_ATTRIBUTE_IP_PKT_LEN,
	MKEEP_ALIVE_ATTRIBUTE_SRC_MAC_ADDR,
	MKEEP_ALIVE_ATTRIBUTE_DST_MAC_ADDR,
	MKEEP_ALIVE_ATTRIBUTE_PERIOD_MSEC,
	MKEEP_ALIVE_ATTRIBUTE_ETHER_TYPE,
	MKEEP_ALIVE_ATTRIBUTE_MAX
};

typedef enum wl_vendor_event {
	BRCM_VENDOR_EVENT_UNSPEC		= 0,
	BRCM_VENDOR_EVENT_PRIV_STR		= 1,
	GOOGLE_GSCAN_SIGNIFICANT_EVENT		= 2,
	GOOGLE_GSCAN_GEOFENCE_FOUND_EVENT	= 3,
	GOOGLE_GSCAN_BATCH_SCAN_EVENT		= 4,
	GOOGLE_SCAN_FULL_RESULTS_EVENT		= 5,
	GOOGLE_RTT_COMPLETE_EVENT		= 6,
	GOOGLE_SCAN_COMPLETE_EVENT		= 7,
	GOOGLE_GSCAN_GEOFENCE_LOST_EVENT	= 8,
	GOOGLE_SCAN_EPNO_EVENT			= 9,
	GOOGLE_DEBUG_RING_EVENT			= 10,
	GOOGLE_FW_DUMP_EVENT			= 11,
	GOOGLE_PNO_HOTSPOT_FOUND_EVENT		= 12,
	GOOGLE_RSSI_MONITOR_EVENT		= 13,
	GOOGLE_MKEEP_ALIVE_EVENT		= 14,

	/*
	 * BRCM specific events should be placed after
	 * the Generic events so that enums don't mismatch
	 * between the DHD and HAL
	 */
	GOOGLE_NAN_EVENT_ENABLED		= 15,
	GOOGLE_NAN_EVENT_DISABLED		= 16,
	GOOGLE_NAN_EVENT_SUBSCRIBE_MATCH	= 17,
	GOOGLE_NAN_EVENT_REPLIED		= 18,
	GOOGLE_NAN_EVENT_PUBLISH_TERMINATED	= 19,
	GOOGLE_NAN_EVENT_SUBSCRIBE_TERMINATED	= 20,
	GOOGLE_NAN_EVENT_DE_EVENT		= 21,
	GOOGLE_NAN_EVENT_FOLLOWUP		= 22,
	GOOGLE_NAN_EVENT_TRANSMIT_FOLLOWUP_IND	= 23,
	GOOGLE_NAN_EVENT_DATA_REQUEST		= 24,
	GOOGLE_NAN_EVENT_DATA_CONFIRMATION	= 25,
	GOOGLE_NAN_EVENT_DATA_END		= 26,
	GOOGLE_NAN_EVENT_BEACON			= 27,
	GOOGLE_NAN_EVENT_SDF			= 28,
	GOOGLE_NAN_EVENT_TCA			= 29,
	GOOGLE_NAN_EVENT_SUBSCRIBE_UNMATCH	= 30,
	GOOGLE_NAN_EVENT_UNKNOWN		= 31,
	GOOGLE_ROAM_EVENT_START			= 32,
	BRCM_VENDOR_EVENT_HANGED                = 33,
	BRCM_VENDOR_EVENT_SAE_KEY               = 34,
	BRCM_VENDOR_EVENT_BEACON_RECV           = 35,
	BRCM_VENDOR_EVENT_PORT_AUTHORIZED       = 36,
	GOOGLE_FILE_DUMP_EVENT			= 37,
	BRCM_VENDOR_EVENT_CU			= 38,
	BRCM_VENDOR_EVENT_WIPS			= 39,
	NAN_ASYNC_RESPONSE_DISABLED		= 40,
	BRCM_VENDOR_EVENT_RCC_INFO		= 41,
	BRCM_VENDOR_EVENT_TWT			= 43,
	WL_VENDOR_EVENT_LAST
} wl_vendor_event_t;

enum andr_wifi_attr {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)) || (ANDROID_VERSION >= 12)
	ANDR_WIFI_ATTRIBUTE_INVALID,
#endif
	ANDR_WIFI_ATTRIBUTE_NUM_FEATURE_SET,
	ANDR_WIFI_ATTRIBUTE_FEATURE_SET,
	ANDR_WIFI_ATTRIBUTE_RANDOM_MAC_OUI,
	ANDR_WIFI_ATTRIBUTE_NODFS_SET,
	ANDR_WIFI_ATTRIBUTE_COUNTRY,
	ANDR_WIFI_ATTRIBUTE_ND_OFFLOAD_VALUE,
	ANDR_WIFI_ATTRIBUTE_TCPACK_SUP_VALUE,
	ANDR_WIFI_ATTRIBUTE_LATENCY_MODE,
	ANDR_WIFI_ATTRIBUTE_RANDOM_MAC,
	ANDR_WIFI_ATTRIBUTE_TX_POWER_SCENARIO,
	ANDR_WIFI_ATTRIBUTE_THERMAL_MITIGATION,
	ANDR_WIFI_ATTRIBUTE_THERMAL_COMPLETION_WINDOW,
	ANDR_WIFI_ATTRIBUTE_VOIP_MODE,
	ANDR_WIFI_ATTRIBUTE_DTIM_MULTIPLIER,
	/* Any new ANDR_WIFI attribute add prior to the ANDR_WIFI_ATTRIBUTE_MAX */
	ANDR_WIFI_ATTRIBUTE_MAX
};
enum apf_attributes {
	APF_ATTRIBUTE_VERSION,
	APF_ATTRIBUTE_MAX_LEN,
	APF_ATTRIBUTE_PROGRAM,
	APF_ATTRIBUTE_PROGRAM_LEN,
	APF_ATTRIBUTE_MAX
};

typedef enum wl_vendor_gscan_attribute {
	ATTR_START_GSCAN,
	ATTR_STOP_GSCAN,
	ATTR_SET_SCAN_BATCH_CFG_ID, /* set batch scan params */
	ATTR_SET_SCAN_GEOFENCE_CFG_ID, /* set list of bssids to track */
	ATTR_SET_SCAN_SIGNIFICANT_CFG_ID, /* set list of bssids, rssi threshold etc.. */
	ATTR_SET_SCAN_CFG_ID, /* set common scan config params here */
	ATTR_GET_GSCAN_CAPABILITIES_ID,
    /* Add more sub commands here */
	ATTR_GSCAN_MAX
} wl_vendor_gscan_attribute_t;

typedef enum gscan_batch_attribute {
	ATTR_GSCAN_BATCH_BESTN,
	ATTR_GSCAN_BATCH_MSCAN,
	ATTR_GSCAN_BATCH_BUFFER_THRESHOLD
} gscan_batch_attribute_t;

typedef enum gscan_geofence_attribute {
	ATTR_GSCAN_NUM_HOTLIST_BSSID,
	ATTR_GSCAN_HOTLIST_BSSID
} gscan_geofence_attribute_t;

typedef enum gscan_complete_event {
	WIFI_SCAN_COMPLETE,
	WIFI_SCAN_THRESHOLD_NUM_SCANS,
	WIFI_SCAN_BUFFER_THR_BREACHED
} gscan_complete_event_t;

#ifdef DHD_WAKE_STATUS
enum wake_stat_attributes {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0)) || (ANDROID_VERSION >= 12)
	WAKE_STAT_ATTRIBUTE_INVALID,
#endif
	WAKE_STAT_ATTRIBUTE_TOTAL_CMD_EVENT,
	WAKE_STAT_ATTRIBUTE_CMD_EVENT_WAKE,
	WAKE_STAT_ATTRIBUTE_CMD_EVENT_COUNT,
	WAKE_STAT_ATTRIBUTE_CMD_EVENT_COUNT_USED,
	WAKE_STAT_ATTRIBUTE_TOTAL_DRIVER_FW,
	WAKE_STAT_ATTRIBUTE_DRIVER_FW_WAKE,
	WAKE_STAT_ATTRIBUTE_DRIVER_FW_COUNT,
	WAKE_STAT_ATTRIBUTE_DRIVER_FW_COUNT_USED,
	WAKE_STAT_ATTRIBUTE_TOTAL_RX_DATA_WAKE,
	WAKE_STAT_ATTRIBUTE_RX_UNICAST_COUNT,
	WAKE_STAT_ATTRIBUTE_RX_MULTICAST_COUNT,
	WAKE_STAT_ATTRIBUTE_RX_BROADCAST_COUNT,
	WAKE_STAT_ATTRIBUTE_RX_ICMP_PKT,
	WAKE_STAT_ATTRIBUTE_RX_ICMP6_PKT,
	WAKE_STAT_ATTRIBUTE_RX_ICMP6_RA,
	WAKE_STAT_ATTRIBUTE_RX_ICMP6_NA,
	WAKE_STAT_ATTRIBUTE_RX_ICMP6_NS,
	WAKE_STAT_ATTRIBUTE_IPV4_RX_MULTICAST_ADD_CNT,
	WAKE_STAT_ATTRIBUTE_IPV6_RX_MULTICAST_ADD_CNT,
	WAKE_STAT_ATTRIBUTE_OTHER_RX_MULTICAST_ADD_CNT,
	WAKE_STAT_ATTRIBUTE_RX_MULTICAST_PKT_INFO,
	/* Please add new attributes from here to sync up old HAL */
	WAKE_STAT_ATTRIBUTE_MAX
};

typedef struct rx_data_cnt_details_t {
	int rx_unicast_cnt;		/* Total rx unicast packet which woke up host */
	int rx_multicast_cnt;   /* Total rx multicast packet which woke up host */
	int rx_broadcast_cnt;   /* Total rx broadcast packet which woke up host */
} RX_DATA_WAKE_CNT_DETAILS;

typedef struct rx_wake_pkt_type_classification_t {
	int icmp_pkt;   /* wake icmp packet count */
	int icmp6_pkt;  /* wake icmp6 packet count */
	int icmp6_ra;   /* wake icmp6 RA packet count */
	int icmp6_na;   /* wake icmp6 NA packet count */
	int icmp6_ns;   /* wake icmp6 NS packet count */
} RX_WAKE_PKT_TYPE_CLASSFICATION;

typedef struct rx_multicast_cnt_t {
	int ipv4_rx_multicast_addr_cnt; /* Rx wake packet was ipv4 multicast */
	int ipv6_rx_multicast_addr_cnt; /* Rx wake packet was ipv6 multicast */
	int other_rx_multicast_addr_cnt; /* Rx wake packet was non-ipv4 and non-ipv6 */
} RX_MULTICAST_WAKE_DATA_CNT;

typedef struct wlan_driver_wake_reason_cnt_t {
	int total_cmd_event_wake;    /* Total count of cmd event wakes */
	int *cmd_event_wake_cnt;     /* Individual wake count array, each index a reason */
	int cmd_event_wake_cnt_sz;   /* Max number of cmd event wake reasons */
	int cmd_event_wake_cnt_used; /* Number of cmd event wake reasons specific to the driver */
	int total_driver_fw_local_wake;    /* Total count of drive/fw wakes, for local reasons */
	int *driver_fw_local_wake_cnt;     /* Individual wake count array, each index a reason */
	int driver_fw_local_wake_cnt_sz;   /* Max number of local driver/fw wake reasons */
	/* Number of local driver/fw wake reasons specific to the driver */
	int driver_fw_local_wake_cnt_used;
	int total_rx_data_wake;     /* total data rx packets, that woke up host */
	RX_DATA_WAKE_CNT_DETAILS rx_wake_details;
	RX_WAKE_PKT_TYPE_CLASSFICATION rx_wake_pkt_classification_info;
	RX_MULTICAST_WAKE_DATA_CNT rx_multicast_wake_pkt_info;
} WLAN_DRIVER_WAKE_REASON_CNT;
#endif /* DHD_WAKE_STATUS */

#define BRCM_VENDOR_WIPS_EVENT_BUF_LEN	128
typedef enum wl_vendor_wips_attr_type {
	WIPS_ATTR_DEAUTH_CNT = 1,
	WIPS_ATTR_DEAUTH_BSSID,
	WIPS_ATTR_CURRENT_RSSI,
	WIPS_ATTR_DEAUTH_RSSI
} wl_vendor_wips_attr_type_t;

#define BRCM_VENDOR_GET_RCC_EVENT_BUF_LEN	\
	sizeof(uint32) + DOT11_MAX_SSID_LEN +	\
	sizeof(int32) + (sizeof(uint16) * MAX_ROAM_CHANNEL)
typedef enum wl_vendor_get_rcc_attr_type {
	RCC_ATTRIBUTE_SSID = 1,
	RCC_ATTRIBUTE_SSID_LEN,
	RCC_ATTRIBUTE_NUM_CHANNELS,
	RCC_ATTRIBUTE_CHANNEL_LIST
} wl_vendor_get_rcc_attr_type_t;

/* Chipset roaming capabilities */
typedef struct wifi_roaming_capabilities {
	u32 max_blacklist_size;
	u32 max_whitelist_size;
} wifi_roaming_capabilities_t;

typedef enum {
	SET_HAL_START_ATTRIBUTE_DEINIT = 0x0001,
	SET_HAL_START_ATTRIBUTE_PRE_INIT = 0x0002,
	SET_HAL_START_ATTRIBUTE_EVENT_SOCK_PID = 0x0003,
	/* Add any new HAL_START attribute prior to SET_HAL_START_ATTRIBUTE_MAX */
	SET_HAL_START_ATTRIBUTE_MAX
} SET_HAL_START_ATTRIBUTE;

#ifdef WL_TWT
typedef enum {
	WIFI_TWT_EVENT_SETUP	= 1,
	WIFI_TWT_EVENT_TEARDOWN	= 2,
	WIFI_TWT_EVENT_INFO_FRM	= 3,
	WIFI_TWT_EVENT_NOTIFY	= 4
} wifi_twt_sub_event;

typedef enum {
	WIFI_TWT_ATTR_NONE		= 0,
	WIFI_TWT_ATTR_SUB_EVENT		= 1,
	WIFI_TWT_ATTR_REASON_CODE	= 2,
	WIFI_TWT_ATTR_STATUS		= 3,
	WIFI_TWT_ATTR_SETUP_CMD		= 4,
	WIFI_TWT_ATTR_FLOW_FLAGS	= 5,
	WIFI_TWT_ATTR_FLOW_ID		= 6,
	WIFI_TWT_ATTR_CHANNEL		= 7,
	WIFI_TWT_ATTR_NEGOTIATION_TYPE	= 8,
	WIFI_TWT_ATTR_WAKETIME_H	= 9,
	WIFI_TWT_ATTR_WAKETIME_L	= 10,
	WIFI_TWT_ATTR_WAKE_DURATION	= 11,
	WIFI_TWT_ATTR_WAKE_INTERVAL	= 12,
	WIFI_TWT_ATTR_BID		= 13,
	WIFI_TWT_ATTR_ALLTWT		= 14,
	WIFI_TWT_ATTR_NEXT_TWT_H	= 15,
	WIFI_TWT_ATTR_NEXT_TWT_L	= 16,
	WIFI_TWT_ATTR_CONFIG_ID		= 17,
	WIFI_TWT_ATTR_NOTIFICATION	= 18,
	WIFI_TWT_ATTR_FLOW_TYPE		= 19,
	WIFI_TWT_ATTR_TRIGGER_TYPE	= 20,

	WIFI_TWT_ATTR_MAX
} wifi_twt_attribute;
#endif /* WL_TWT */

#ifdef WL_TWT_HAL_IF
#define BRCM_TWT_HAL_VENDOR_EVENT_BUF_LEN   500

typedef enum {
	ANDR_TWT_ATTR_NONE		= 0,
	ANDR_TWT_ATTR_CONFIG_ID		= 1,
	ANDR_TWT_ATTR_NEGOTIATION_TYPE	= 2,
	ANDR_TWT_ATTR_TRIGGER_TYPE	= 3,
	ANDR_TWT_ATTR_WAKE_DURATION	= 4,
	ANDR_TWT_ATTR_WAKE_INTERVAL	= 5,
	ANDR_TWT_ATTR_WAKE_INTERVAL_MIN	= 6,
	ANDR_TWT_ATTR_WAKE_INTERVAL_MAX	= 7,
	ANDR_TWT_ATTR_WAKE_DURATION_MIN	= 8,
	ANDR_TWT_ATTR_WAKE_DURATION_MAX	= 9,
	ANDR_TWT_ATTR_AVG_PKT_SIZE	= 10,
	ANDR_TWT_ATTR_AVG_PKT_NUM	= 11,
	ANDR_TWT_ATTR_WAKETIME_OFFSET	= 12,
	ANDR_TWT_ATTR_ALL_TWT		= 13,
	ANDR_TWT_ATTR_RESUME_TIME	= 14,
	ANDR_TWT_ATTR_AVG_EOSP_DUR	= 15,
	ANDR_TWT_ATTR_EOSP_CNT		= 16,
	ANDR_TWT_ATTR_NUM_SP		= 17,
	ANDR_TWT_ATTR_DEVICE_CAP	= 18,
	ANDR_TWT_ATTR_PEER_CAP		= 19,
	ANDR_TWT_ATTR_STATUS		= 20,
	ANDR_TWT_ATTR_REASON_CODE	= 21,
	ANDR_TWT_ATTR_TWT_RESUMED	= 22,
	ANDR_TWT_ATTR_TWT_NOTIFICATION	= 23,
	ANDR_TWT_ATTR_SUB_EVENT		= 24,
	ANDR_TWT_ATTR_NUM_PEER_STATS	= 25,
	ANDR_TWT_ATTR_AVG_PKT_NUM_TX	= 26,
	ANDR_TWT_ATTR_AVG_PKT_SIZE_TX	= 27,
	ANDR_TWT_ATTR_AVG_PKT_NUM_RX	= 28,
	ANDR_TWT_ATTR_AVG_PKT_SIZE_RX	= 29,
	ANDR_TWT_ATTR_MAX
} andr_twt_attribute;

typedef enum {
	ANDR_TWT_EVENT_SETUP	= 1,
	ANDR_TWT_EVENT_TEARDOWN	= 2,
	ANDR_TWT_EVENT_INFO_FRM	= 3,
	ANDR_TWT_EVENT_NOTIFY	= 4
} andr_twt_sub_event;
#endif /* WL_TWT_HAL_IF */

/* Capture the BRCM_VENDOR_SUBCMD_PRIV_STRINGS* here */
#define BRCM_VENDOR_SCMD_CAPA	"cap"
#define MEMDUMP_PATH_LEN	128

#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 13, 0)) || defined(WL_VENDOR_EXT_SUPPORT)
extern int wl_cfgvendor_attach(struct wiphy *wiphy, dhd_pub_t *dhd);
extern int wl_cfgvendor_detach(struct wiphy *wiphy);
extern int wl_cfgvendor_send_async_event(struct wiphy *wiphy,
                  struct net_device *dev, int event_id, const void  *data, int len);
extern int wl_cfgvendor_send_hotlist_event(struct wiphy *wiphy,
                struct net_device *dev, void  *data, int len, wl_vendor_event_t event);
#else
static INLINE int wl_cfgvendor_attach(struct wiphy *wiphy,
		dhd_pub_t *dhd) { UNUSED_PARAMETER(wiphy); UNUSED_PARAMETER(dhd); return 0; }
static INLINE int wl_cfgvendor_detach(struct wiphy *wiphy) { UNUSED_PARAMETER(wiphy); return 0; }
static INLINE int wl_cfgvendor_send_async_event(struct wiphy *wiphy,
                  struct net_device *dev, int event_id, const void  *data, int len)
{ return 0; }
static INLINE int wl_cfgvendor_send_hotlist_event(struct wiphy *wiphy,
	struct net_device *dev, void  *data, int len, wl_vendor_event_t event)
{ return 0; }
#endif /*  (LINUX_VERSION_CODE > KERNEL_VERSION(3, 13, 0)) || defined(WL_VENDOR_EXT_SUPPORT) */

#if defined(WL_SUPP_EVENT) && \
	((LINUX_VERSION_CODE > KERNEL_VERSION(3, 13, 0)) || defined(WL_VENDOR_EXT_SUPPORT))
extern int wl_cfgvendor_send_supp_eventstring(const char *func, const char *fmt, ...);
int wl_cfgvendor_notify_supp_event_str(const char *evt_name, const char *fmt, ...);
#define SUPP_LOG_LEN 256
#define PRINT_SUPP_LOG(fmt, ...) \
	 wl_cfgvendor_send_supp_eventstring(__func__, fmt, ##__VA_ARGS__);
#define SUPP_LOG(args)  PRINT_SUPP_LOG  args;
#define SUPP_EVT_LOG(evt_name, fmt, ...) \
    wl_cfgvendor_notify_supp_event_str(evt_name, fmt, ##__VA_ARGS__);
#define SUPP_EVENT(args) SUPP_EVT_LOG args
#else
#define SUPP_LOG(x)
#define SUPP_EVENT(x)
#endif /* WL_SUPP_EVENT && (kernel > (3, 13, 0)) || WL_VENDOR_EXT_SUPPORT */

#ifdef CONFIG_COMPAT
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 6, 0))
#define COMPAT_STRUCT_IFACE(normal_structure, value)	\
	compat_ ## normal_structure compat_ ## iface;	\
	int compat_task_state = in_compat_syscall();			\
	normal_structure value;
#else
#define COMPAT_STRUCT_IFACE(normal_structure, value)	\
	compat_ ## normal_structure compat_ ## iface;	\
	int compat_task_state = is_compat_task();			\
	normal_structure value;
#endif

#define COMPAT_BZERO_IFACE(normal_structure, value)	\
	do { \
		if (compat_task_state) {	\
			bzero(&compat_ ## value, sizeof(compat_ ## normal_structure));	\
		} else { \
			bzero(&value, sizeof(normal_structure));	\
		} \
	} while (0)

#define COMPAT_ASSIGN_VALUE(normal_structure, member, value)	\
	do { \
		if (compat_task_state) {	\
			compat_ ## normal_structure.member = value; \
		} else { \
			normal_structure.member = value; \
		} \
	} while (0)

#define COMPAT_MEMCOPY_IFACE(output, total_len, normal_structure, value, wifi_rate_stat)	\
	do { \
		if (compat_task_state) {	\
			memcpy(output, &compat_ ## value, sizeof(compat_ ## normal_structure));	\
			output += (sizeof(compat_ ## value) - sizeof(wifi_rate_stat));	\
			total_len += sizeof(compat_ ## normal_structure);	\
		} else { \
			memcpy(output, &value, sizeof(normal_structure));	\
			output += (sizeof(value) - sizeof(wifi_rate_stat));	\
			total_len += sizeof(normal_structure);	\
		} \
	} while (0)
#else
#define COMPAT_STRUCT_IFACE(normal_structure, value)	normal_structure value;
#define COMPAT_BZERO_IFACE(normal_structure, value)	bzero(&value, sizeof(normal_structure));
#define COMPAT_ASSIGN_VALUE(normal_structure, member, value)	normal_structure.member = value;
#define COMPAT_MEMCOPY_IFACE(output, total_len, normal_structure, value, rate_stat)	\
	do { \
		memcpy(output, &value, sizeof(normal_structure));	\
		output += (sizeof(value) - sizeof(wifi_rate_stat));	\
		total_len += sizeof(normal_structure);	\
	} while (0)
#endif /* CONFIG_COMPAT */

#if (defined(CONFIG_ARCH_MSM) && defined(SUPPORT_WDEV_CFG80211_VENDOR_EVENT_ALLOC)) || \
		LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0)
#define CFG80211_VENDOR_EVENT_ALLOC(wiphy, wdev, len, type, kflags) \
	cfg80211_vendor_event_alloc(wiphy, wdev, len, type, kflags);
#else
#define CFG80211_VENDOR_EVENT_ALLOC(wiphy, wdev, len, type, kflags) \
	cfg80211_vendor_event_alloc(wiphy, len, type, kflags);
#endif /* (defined(CONFIG_ARCH_MSM) && defined(SUPPORT_WDEV_CFG80211_VENDOR_EVENT_ALLOC)) || */
	/* LINUX_VERSION_CODE >= KERNEL_VERSION(4, 1, 0) */
int wl_cfgvendor_nan_send_async_disable_resp(struct wireless_dev *wdev);

#ifdef WL_CFGVENDOR_SEND_HANG_EVENT
void wl_cfgvendor_send_hang_event(struct net_device *dev, u16 reason,
	char *string, int hang_info_cnt);
void wl_cfgvendor_simple_hang_event(struct net_device *dev, u16 reason);
void wl_copy_hang_info_if_falure(struct net_device *dev, u16 reason, s32 ret);
#endif /* WL_CFGVENDOR_SEND_HANG_EVENT */
#ifdef DHD_PKT_LOGGING
int wl_cfgvendor_dbg_send_pktlog_dbg_file_dump_evt(struct net_device *ndev);
#endif /* DHD_PKT_LOGGING */
int wl_cfgvendor_connect_params_handler(struct wiphy *wiphy, struct wireless_dev *wdev,
	const void  *data, int len);
int wl_cfgvendor_start_ap_params_handler(struct wiphy *wiphy, struct wireless_dev *wdev,
	const void  *data, int len);
#endif /* _wl_cfgvendor_h_ */
