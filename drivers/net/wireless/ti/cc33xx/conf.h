/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (C) 2022-2024 Texas Instruments Incorporated - https://www.ti.com/
 */

#ifndef __CONF_H__
#define __CONF_H__

struct cc33xx_conf_header {
	u32 magic;
	u16 fw_major_version;
	u16 fw_minor_version;
	u16 fw_api_version;
	u16 fw_build_version;
	u32 checksum;
} __packed;

#define CC33XX_CONF_MAGIC				0x10e100ca

#define CC33XX_CONF_FW_MAJOR_VERSION 	0x0001
#define CC33XX_CONF_FW_MINOR_VERSION 	0x0007
#define CC33XX_CONF_FW_API_VERSION   	0x0000
#define CC33XX_CONF_FW_BUILD_VERSION 	0x013C

#define CC33XX_CONF_MASK				0x0000ffff
#define CC33X_CONF_SIZE					(sizeof(struct cc33xx_conf_file))

#define CONF_HW_RXTX_RATE_UNSUPPORTED 0xff

enum {
	CONF_HW_BIT_RATE_1MBPS   = BIT(1),
	CONF_HW_BIT_RATE_2MBPS   = BIT(2),
	CONF_HW_BIT_RATE_5_5MBPS = BIT(3),
	CONF_HW_BIT_RATE_11MBPS  = BIT(4),
	CONF_HW_BIT_RATE_6MBPS   = BIT(5),
	CONF_HW_BIT_RATE_9MBPS   = BIT(6),
	CONF_HW_BIT_RATE_12MBPS  = BIT(7),
	CONF_HW_BIT_RATE_18MBPS  = BIT(8),
	CONF_HW_BIT_RATE_24MBPS  = BIT(9),
	CONF_HW_BIT_RATE_36MBPS  = BIT(10),
	CONF_HW_BIT_RATE_48MBPS  = BIT(11),
	CONF_HW_BIT_RATE_54MBPS  = BIT(12),
	CONF_HW_BIT_RATE_MCS_0   = BIT(13),
	CONF_HW_BIT_RATE_MCS_1   = BIT(14),
	CONF_HW_BIT_RATE_MCS_2   = BIT(15),
	CONF_HW_BIT_RATE_MCS_3   = BIT(16),
	CONF_HW_BIT_RATE_MCS_4   = BIT(17),
	CONF_HW_BIT_RATE_MCS_5   = BIT(18),
	CONF_HW_BIT_RATE_MCS_6   = BIT(19),
	CONF_HW_BIT_RATE_MCS_7   = BIT(20)
};

enum {
	CONF_HW_RATE_INDEX_1MBPS      = 1,
	CONF_HW_RATE_INDEX_2MBPS      = 2,
	CONF_HW_RATE_INDEX_5_5MBPS    = 3,
	CONF_HW_RATE_INDEX_11MBPS     = 4,
	CONF_HW_RATE_INDEX_6MBPS      = 5,
	CONF_HW_RATE_INDEX_9MBPS      = 6,
	CONF_HW_RATE_INDEX_12MBPS     = 7,
	CONF_HW_RATE_INDEX_18MBPS     = 8,
	CONF_HW_RATE_INDEX_24MBPS     = 9,
	CONF_HW_RATE_INDEX_36MBPS     = 10,
	CONF_HW_RATE_INDEX_48MBPS     = 11,
	CONF_HW_RATE_INDEX_54MBPS     = 12,
	CONF_HW_RATE_INDEX_MCS0       = 13,
	CONF_HW_RATE_INDEX_MCS1       = 14,
	CONF_HW_RATE_INDEX_MCS2       = 15,
	CONF_HW_RATE_INDEX_MCS3       = 16,
	CONF_HW_RATE_INDEX_MCS4       = 17,
	CONF_HW_RATE_INDEX_MCS5       = 18,
	CONF_HW_RATE_INDEX_MCS6       = 19,
	CONF_HW_RATE_INDEX_MCS7       = 20,

	CONF_HW_RATE_INDEX_MAX        = CONF_HW_RATE_INDEX_MCS7,
};

enum {
	CONF_PREAMBLE_TYPE_SHORT	= 0,
	CONF_PREAMBLE_TYPE_LONG		= 1,
	CONF_PREAMBLE_TYPE_OFDM		= 2,
	CONF_PREAMBLE_TYPE_N_MIXED_MODE	= 3,
	CONF_PREAMBLE_TYPE_GREENFIELD	= 4,
	CONF_PREAMBLE_TYPE_AX_SU	= 5,
	CONF_PREAMBLE_TYPE_AX_MU	= 6,
	CONF_PREAMBLE_TYPE_AX_SU_ER	= 7,
	CONF_PREAMBLE_TYPE_AX_TB	= 8,
	CONF_PREAMBLE_TYPE_AX_TB_NDP_FB	= 9,
	CONF_PREAMBLE_TYPE_AC_VHT	= 10,
	CONF_PREAMBLE_TYPE_BE_EHT_MU	= 13,
	CONF_PREAMBLE_TYPE_BE_EHT_TB	= 14,
	CONF_PREAMBLE_TYPE_INVALID	= 0xFF
};

enum {
	CONF_PS_SCHEME_LEGACY = 0,
	CONF_PS_SCHEME_UPSD_TRIGGER = 1,
	CONF_PS_SCHEME_LEGACY_PSPOLL = 2,
	CONF_PS_SCHEME_SAPSD = 3,
};

enum {
	CONF_TX_AC_BE = 0,
	CONF_TX_AC_BK = 1,
	CONF_TX_AC_VI = 2,
	CONF_TX_AC_VO = 3,
	CONF_TX_AC_CTS2SELF = 4,
	CONF_TX_AC_ANY_TID = 0xff
};

enum {
	CONF_BCN_FILT_MODE_DISABLED = 0,
	CONF_BCN_FILT_MODE_ENABLED = 1
};

struct cc33xx_clk_cfg {
	u32 n;
	u32 m;
	u32 p;
	u32 q;
	u8 swallow;
};

#define CONF_TX_MAX_AC_COUNT 4

struct conf_tx_ac_category {
	/* The AC class identifier.
	 *
	 * Range: enum conf_tx_ac
	 */
	u8 ac;

	/* The contention window minimum size (in slots) for the access
	 * class.
	 *
	 * Range: uint8_t
	 */
	u8 cw_min;

	/* The contention window maximum size (in slots) for the access
	 * class.
	 *
	 * Range: uint8_t
	 */
	u16 cw_max;

	/* The AIF value (in slots) for the access class.
	 *
	 * Range: uint8_t
	 */
	u8 aifsn;

	/* The TX Op Limit (in microseconds) for the access class.
	 *
	 * Range: uint16_t
	 */
	u16 tx_op_limit;

	/* Is the MU EDCA configured
	 *
	 * Range: uint8_t
	 */
	u8 is_mu_edca;

	/*  The AIFSN value for the corresponding access class
	 *
	 * Range: uint8_t
	 */
	u8 mu_edca_aifs;

	/* The ECWmin and ECWmax value is indicating contention window maximum
	 * size (in slots) for the access
	 *
	 * Range: uint8_t
	 */
	u8 mu_edca_ecw_min_max;

	/* The MU EDCA timer (in microseconds) obtaining an EDCA TXOP
	 * for STA using MU EDCA parameters
	 *
	 * Range: uint8_t
	 */
	u8 mu_edca_timer;
} __packed;

struct conf_tx_tid {
	u8 channel_type;
	u8 ps_scheme;
} __packed;

#define CONF_TX_RATE_MASK_BASIC     (CONF_HW_BIT_RATE_1MBPS | CONF_HW_BIT_RATE_2MBPS)

#define CONF_TX_ENABLED_RATES       (CONF_HW_BIT_RATE_1MBPS |    \
	CONF_HW_BIT_RATE_2MBPS | CONF_HW_BIT_RATE_5_5MBPS |      \
	CONF_HW_BIT_RATE_6MBPS | CONF_HW_BIT_RATE_9MBPS |        \
	CONF_HW_BIT_RATE_11MBPS | CONF_HW_BIT_RATE_12MBPS |      \
	CONF_HW_BIT_RATE_18MBPS | CONF_HW_BIT_RATE_24MBPS |      \
	CONF_HW_BIT_RATE_36MBPS | CONF_HW_BIT_RATE_48MBPS |      \
	CONF_HW_BIT_RATE_54MBPS)

#define CONF_TX_MCS_RATES (CONF_HW_BIT_RATE_MCS_0 |              \
	CONF_HW_BIT_RATE_MCS_1 | CONF_HW_BIT_RATE_MCS_2 |        \
	CONF_HW_BIT_RATE_MCS_3 | CONF_HW_BIT_RATE_MCS_4 |        \
	CONF_HW_BIT_RATE_MCS_5 | CONF_HW_BIT_RATE_MCS_6 |        \
	CONF_HW_BIT_RATE_MCS_7)

#define CONF_TX_IBSS_DEFAULT_RATES  (CONF_HW_BIT_RATE_1MBPS |       \
		CONF_HW_BIT_RATE_2MBPS | CONF_HW_BIT_RATE_5_5MBPS | \
		CONF_HW_BIT_RATE_11MBPS | CONF_TX_OFDM_RATES)

#define CONF_TX_CCK_RATES  (CONF_HW_BIT_RATE_1MBPS |		\
	CONF_HW_BIT_RATE_2MBPS | CONF_HW_BIT_RATE_5_5MBPS |	\
	CONF_HW_BIT_RATE_11MBPS)

#define CONF_TX_OFDM_RATES (CONF_HW_BIT_RATE_6MBPS |             \
	CONF_HW_BIT_RATE_12MBPS | CONF_HW_BIT_RATE_24MBPS |      \
	CONF_HW_BIT_RATE_36MBPS | CONF_HW_BIT_RATE_48MBPS |      \
	CONF_HW_BIT_RATE_54MBPS)

#define CONF_BCN_IE_OUI_LEN 3
#define CONF_BCN_IE_VER_LEN 2

struct conf_tx_settings {
	/* Configuration for access categories for TX rate control.
	 */
	u8 ac_conf_count;
	/*struct conf_tx_ac_category ac_conf[CONF_TX_MAX_AC_COUNT];*/
	struct conf_tx_ac_category ac_conf0;
	struct conf_tx_ac_category ac_conf1;
	struct conf_tx_ac_category ac_conf2;
	struct conf_tx_ac_category ac_conf3;

	/* AP-mode - allow this number of TX retries to a station before an
	 * event is triggered from FW.
	 * In AP-mode the hlids of unreachable stations are given in the
	 * "sta_tx_retry_exceeded" member in the event mailbox.
	 */
	u8 max_tx_retries;

	/* Configuration for TID parameters.
	 */
	u8 tid_conf_count;
	/* struct conf_tx_tid tid_conf[]; */
	struct conf_tx_tid tid_conf0;
	struct conf_tx_tid tid_conf1;
	struct conf_tx_tid tid_conf2;
	struct conf_tx_tid tid_conf3;
	struct conf_tx_tid tid_conf4;
	struct conf_tx_tid tid_conf5;
	struct conf_tx_tid tid_conf6;
	struct conf_tx_tid tid_conf7;

	/* Max time in msec the FW may delay frame TX-Complete interrupt.
	 *
	 * Range: uint16_t
	 */
	u16 tx_compl_timeout;

	/* The rate used for control messages and scanning on the 2.4GHz band
	 *
	 * Range: CONF_HW_BIT_RATE_* bit mask
	 */
	u32 basic_rate;

	/* The rate used for control messages and scanning on the 5GHz band
	 *
	 * Range: CONF_HW_BIT_RATE_* bit mask
	 */
	u32 basic_rate_5;

	/* Time in ms for Tx watchdog timer to expire */
	u32 tx_watchdog_timeout;
} __packed;

#define CONF_MAX_BCN_FILT_IE_COUNT 32

struct conf_bcn_filt_rule {
	/* IE number to which to associate a rule.
	 *
	 * Range: uint8_t
	 */
	u8 ie;

	/* Rule to associate with the specific ie.
	 *
	 * Range: CONF_BCN_RULE_PASS_ON_*
	 */
	u8 rule;

	/* OUI for the vendor specifie IE (221)
	 */
	u8 oui[3];

	/* Type for the vendor specifie IE (221)
	 */
	u8 type;

	/* Version for the vendor specifie IE (221)
	 */
	u8 version[2];
} __packed;

struct conf_conn_settings {
	/* Enable or disable the beacon filtering.
	 *
	 * Range: CONF_BCN_FILT_MODE_*
	 */
	u8 bcn_filt_mode;

	/* Configure Beacon filter pass-through rules.
	 */
	u8 bcn_filt_ie_count;
	/*struct conf_bcn_filt_rule bcn_filt_ie[CONF_MAX_BCN_FILT_IE_COUNT];*/
	/* struct conf_bcn_filt_rule bcn_filt_ie[32]; */
	struct conf_bcn_filt_rule bcn_filt_ie0;
	struct conf_bcn_filt_rule bcn_filt_ie1;
	struct conf_bcn_filt_rule bcn_filt_ie2;
	struct conf_bcn_filt_rule bcn_filt_ie3;
	struct conf_bcn_filt_rule bcn_filt_ie4;
	struct conf_bcn_filt_rule bcn_filt_ie5;
	struct conf_bcn_filt_rule bcn_filt_ie6;
	struct conf_bcn_filt_rule bcn_filt_ie7;
	struct conf_bcn_filt_rule bcn_filt_ie8;
	struct conf_bcn_filt_rule bcn_filt_ie9;
	struct conf_bcn_filt_rule bcn_filt_ie10;
	struct conf_bcn_filt_rule bcn_filt_ie11;
	struct conf_bcn_filt_rule bcn_filt_ie12;
	struct conf_bcn_filt_rule bcn_filt_ie13;
	struct conf_bcn_filt_rule bcn_filt_ie14;
	struct conf_bcn_filt_rule bcn_filt_ie15;
	struct conf_bcn_filt_rule bcn_filt_ie16;
	struct conf_bcn_filt_rule bcn_filt_ie17;
	struct conf_bcn_filt_rule bcn_filt_ie18;
	struct conf_bcn_filt_rule bcn_filt_ie19;
	struct conf_bcn_filt_rule bcn_filt_ie20;
	struct conf_bcn_filt_rule bcn_filt_ie21;
	struct conf_bcn_filt_rule bcn_filt_ie22;
	struct conf_bcn_filt_rule bcn_filt_ie23;
	struct conf_bcn_filt_rule bcn_filt_ie24;
	struct conf_bcn_filt_rule bcn_filt_ie25;
	struct conf_bcn_filt_rule bcn_filt_ie26;
	struct conf_bcn_filt_rule bcn_filt_ie27;
	struct conf_bcn_filt_rule bcn_filt_ie28;
	struct conf_bcn_filt_rule bcn_filt_ie29;
	struct conf_bcn_filt_rule bcn_filt_ie30;
	struct conf_bcn_filt_rule bcn_filt_ie31;

	/* The number of consecutive beacons to lose, before the firmware
	 * becomes out of synch.
	 *
	 * Range: uint32_t
	 */
	u32 synch_fail_thold;

	/* After out-of-synch, the number of TU's to wait without a further
	 * received beacon (or probe response) before issuing the BSS_EVENT_LOSE
	 * event.
	 *
	 * Range: uint32_t
	 */
	u32 bss_lose_timeout;

	/* Specifies the dynamic PS timeout in ms that will be used
	 * by the FW when in AUTO_PS mode
	 */
	u16 dynamic_ps_timeout;

	/* Maximum listen interval supported by the driver in units of beacons.
	 *
	 * Range: uint16_t
	 */
	u8 max_listen_interval;

	/* Default sleep authorization for a new STA interface. This determines
	 * whether we can go to ELP.
	 */
	u8 sta_sleep_auth;

	/* Default RX BA Activity filter configuration
	 */
	u8 suspend_rx_ba_activity;
} __packed;

struct conf_scan_settings {
	/* The minimum time to wait on each channel for active scans
	 * This value will be used whenever there's a connected interface.
	 *
	 * Range: uint32_t tu/1000
	 */
	u32 min_dwell_time_active;

	/* The maximum time to wait on each channel for active scans
	 * This value will be currently used whenever there's a
	 * connected interface. It shouldn't exceed 30000 (~30ms) to avoid
	 * possible interference of voip traffic going on while scanning.
	 *
	 * Range: uint32_t tu/1000
	 */
	u32 max_dwell_time_active;

	/* The minimum time to wait on each channel for active scans
	 * when it's possible to have longer scan dwell times.
	 * Currently this is used whenever we're idle on all interfaces.
	 * Longer dwell times improve detection of networks within a
	 * single scan.
	 *
	 * Range: uint32_t tu/1000
	 */
	u32 min_dwell_time_active_long;

	/* The maximum time to wait on each channel for active scans
	 * when it's possible to have longer scan dwell times.
	 * See min_dwell_time_active_long
	 *
	 * Range: uint32_t tu/1000
	 */
	u32 max_dwell_time_active_long;

	/* time to wait on the channel for passive scans (in TU/1000) */
	u32 dwell_time_passive;

	/* time to wait on the channel for DFS scans (in TU/1000) */
	u32 dwell_time_dfs;

	/* Number of probe requests to transmit on each active scan channel
	 *
	 * Range: uint8_t
	 */
	u16 num_probe_reqs;

	/* Scan trigger (split scan) timeout. The FW will split the scan
	 * operation into slices of the given time and allow the FW to schedule
	 * other tasks in between.
	 *
	 * Range: uint32_t Microsecs
	 */
	u32 split_scan_timeout;
} __packed;

struct conf_sched_scan_settings {
	/* The base time to wait on the channel for active scans (in TU/1000).
	 * The minimum dwell time is calculated according to this:
	 * min_dwell_time = base + num_of_probes_to_be_sent * delta_per_probe
	 * The maximum dwell time is calculated according to this:
	 * max_dwell_time = min_dwell_time + max_dwell_time_delta
	 */
	u32 base_dwell_time;

	/* The delta between the min dwell time and max dwell time for
	 * active scans (in TU/1000s). The max dwell time is used by the FW once
	 * traffic is detected on the channel.
	 */
	u32 max_dwell_time_delta;

	/* Delta added to min dwell time per each probe in 2.4 GHz (TU/1000) */
	u32 dwell_time_delta_per_probe;

	/* Delta added to min dwell time per each probe in 5 GHz (TU/1000) */
	u32 dwell_time_delta_per_probe_5;

	/* time to wait on the channel for passive scans (in TU/1000) */
	u32 dwell_time_passive;

	/* time to wait on the channel for DFS scans (in TU/1000) */
	u32 dwell_time_dfs;

	/* number of probe requests to send on each channel in active scans */
	u8 num_probe_reqs;

	/* RSSI threshold to be used for filtering */
	s8 rssi_threshold;

	/* SNR threshold to be used for filtering */
	s8 snr_threshold;

	/* number of short intervals scheduled scan cycles before
	 * switching to long intervals
	 */
	u8 num_short_intervals;

	/* interval between each long scheduled scan cycle (in ms) */
	u16 long_interval;
} __packed;

struct conf_ht_setting {
	u8 rx_ba_win_size;

	/* DEFAULT / WIDE / SISO20 */
	u8 mode;
} __packed;

struct conf_fwlog {
	/* Continuous or on-demand */
	u8 mode;

	/* Number of memory blocks dedicated for the FW logger
	 *
	 * Range: 2-16, or 0 to disable the FW logger
	 */
	u8 mem_blocks;

	/* Minimum log level threshold */
	u8 severity;

	/* Include/exclude timestamps from the log messages */
	u8 timestamp;

	/* See enum cc33xx_fwlogger_output */
	u8 output;

	/* Regulates the frequency of log messages */
	u8 threshold;
} __packed;

enum cc33xx_ht_mode {
	/* Default - use MIMO, fallback to SISO20 */
	HT_MODE_DEFAULT = 0,

	/* Wide - use SISO40 */
	HT_MODE_WIDE = 1,

	/* Use SISO20 */
	HT_MODE_SISO20 = 2,
};

struct coex_wifi_group_priorities {
	/*
	 * WLAN activity groups priorities
	 *
	 * Range: 0 - 15
	 */
	u8 coex_enabled;
	u8 wlan_group_core_active_priority;
	u8 wlan_group_traffic_priority;
	u8 wlan_group_scan_priority;
	u8 wlan_group_twt_traffic_priority;
	u8 wlan_group_management_sequence_priority;
	u8 wlan_group_scan_high_priority;
	u8 wlan_group_broadcast_multicast_priority;
	u8 wlan_group_beacon_priority;
	u8 wlan_group_management_sequence_urgent_priority;
	u8 wlan_group_beacon_urgent_priority;
	u8 wlan_group_phy_command_priority;
} __packed;

struct coex_ble_group_priorities {
	/*
	 * BLE command groups priorities
	 *
	 * Range: 0 - 15
	 */
	u8 ble_group_test;
	u8 ble_group_setup;
	u8 ble_group_connected;
	u8 ble_group_observer;
	u8 ble_group_broadcaster;
	u8 ble_group_initiator;
	u8 ble_group_urgent_priority;
} __packed;

struct coex_external_soc_priorities {
	/*
	 * External SoC low and high priorities
	 *
	 * Range: 0 - 15
	 */
	u8 low_priority;
	u8 high_priority;
} __packed;

struct conf_coex_configuration {
	/*
	 * Work without Coex HW
	 *
	 * Range: 1 - YES, 0 - NO
	 */
	u8 Disable_coex;

	/*
	 * Tie breaker configuration
	 *
	 * Range: 0 - 3 for each tie breaker.
	 *
	 * 0 is lowest and 3 is the highest.
	 */
	u8 tie_breaker_ble;
	u8 tie_breaker_wifi;
	u8 tie_breaker_ext_soc;

	/*
	 * Coex BLE configuration
	 */
	u8 ble_enabled;
	u8 ble_grant_polarity;
	u8 ble_pta_signalling_mode;
	u8 ble_tx_bypass_val;
	u8 ble_rx_bypass_val;

	/*
	 * Coex WiFi configuration
	 */
	u8 wifi_enabled;
	u8 wifi_grant_polarity;
	u8 wifi_alt_band_input_bypass;
	u8 wifi_rx_only_input_bypass;
	u8 wifi_alt_band_input_bypass_val;
	u8 wifi_rx_only_input_bypass_val;

	/*
	 * External SoC entity enable
	 *
	 * 0 - NO
	 * 1 - YES
	 */
	u8 is_Ext_soc_enable;
	/*
	 * External SoC PTA signalling mode
	 *
	 * 00 - Reserved
	 * 01 - 1-wire
	 * 02 - 2-wires
	 * 11 - 3-wires
	 */
	u8 ext_soc_pta_signalling_mode;
	/*
	 * External SoC request polarity
	 *
	 * 0 - Active Low (Default)
	 *
	 * 1 - Active High
	 */
	u8 ext_soc_request_polarity;
	/*
	 * External SoC priority polarity
	 *
	 * 0 - Active Low (Default)
	 *
	 * 1 - Active High
	 */
	u8 ext_soc_priority_polarity;
	/*
	 * External SoC grant polarity
	 *
	 * 0 - Active Low
	 *
	 * 1 - Active High (Default)
	 */
	u8 ext_soc_grant_polarity;
	/*
	 * External SoC grant renew bypass
	 *
	 * 0 - SOC needs to renew grant.
	 *
	 * 1 - Bypass SOC grant renew
	 */
	u8 ext_soc_grant_renew_bypass;
	/*
	 * External SoC request signal detection mechanism
	 *
	 * 00 - Rise edge detection (L -> H)
	 * 01 - Fall edge detection (H -> L)
	 * 10 - Level detection (active high)
	 * 11 - Level detection (active low)
	 */
	u8 ext_soc_request_signal_detection;

	/*
	 * External SoC GPIO pins
	 */
	u8 ext_soc_grant_pin;
	u8 ext_soc_request_pin;
	u8 ext_soc_priority_pin;

	/*
	 * Coex grant delays
	 *
	 * Range: 0 - 200 usec
	 */
	u8 wifi_to_ble_grant_delay;
	u8 wifi_to_ext_soc_grant_delay;
	u8 ble_to_wifi_grant_delay;
	u8 ble_to_ext_soc_grant_delay;
	u8 ext_soc_to_wifi_grant_delay;
	u8 ext_soc_to_ble_grant_delay;

	/*
	 * Coex grant times
	 */
	u16 wifi_min_grant_time;
	u16 ble_min_grant_time;
	u16 ble_max_grant_time;
	u16 ext_soc_min_grant_time;
	u16 ext_soc_max_grant_time;
	/*
	 * BLE T2 time
	 *
	 * Range: 0 - 50 us
	 */
	u8 ble_t2_time;
	/*
	 * External SoC T2 time
	 *
	 * Range: 0 - 50 us
	 */
	u8 ext_soc_t2_time;

	struct coex_wifi_group_priorities wifi_group_priorities;
	struct coex_ble_group_priorities ble_group_priorities;
	struct coex_external_soc_priorities external_soc_priorities;

} __attribute__((__packed__));

struct conf_iomux_configuration {
	/* For any iomux pull value:
	 * 1: Pull up
	 * 2: Pull down
	 * 3: Pull disable
	 * ff: Default value set by HW
	 * ANY other value is invalid
	 */
	u8 slow_clock_in_pull_val;
	u8 sdio_clk_pull_val;
	u8 sdio_cmd_pull_val;
	u8 sdio_d0_pull_val;
	u8 sdio_d1_pull_val;
	u8 sdio_d2_pull_val;
	u8 sdio_d3_pull_val;
	u8 host_irq_wl_pull_val;
	u8 uart1_tx_pull_val;
	u8 uart1_rx_pull_val;
	u8 uart1_cts_pull_val;
	u8 uart1_rts_pull_val;
	u8 coex_priority_pull_val;
	u8 coex_req_pull_val;
	u8 coex_grant_pull_val;
	u8 host_irq_ble_pull_val;
	u8 fast_clk_req_pull_val;
	u8 ant_sel_pull_val;
} __packed;

struct conf_ant_diversity {
	/* First beacons after antenna switch.
	 * In this window we asses our satisfaction from the new antenna.
	 */
	u8 fast_switching_window;

	/* Deltas above this threshold between the curiosity score and
	 * the average RSSI will lead to antenna switch.
	 */
	u8 rssi_delta_for_switching;

	/* Used in the first beacons after antenna switch:
	 * Deltas above this threshold between the average RSSI and
	 * the curiosity score will make us switch back the antennas.
	 */
	u8 rssi_delta_for_fast_switching;

	/* Curiosity punishment in beacon timeout after an antenna switch.
	 */
	u8 curiosity_punish;

	/* Curiosity raise in beacon timeout not after an antenna switch.
	 */
	u8 curiosity_raise;

	/* Used for the average RSSI punishment in beacon timeout
	 * not after antenna switch.
	 */
	u8 consecutive_missed_beacons_threshold;

	/* Used in the curiosity metric.
	 */
	u8 compensation_log;

	/* Used in the average RSSI metric.
	 */
	u8 log_alpha;

	/* Curiosity initialization score.
	 */
	s8 initial_curiosity;

	/* MR configuration: should the AP follow the STA antenna or use the default antenna.
	 */
	u8 ap_follows_sta;

	/* MR configuration: should the BLE follow the STA antenna or use the default antenna.
	 */
	u8 ble_follows_sta;

	/* The antenna to use when the diversity mechanism is not in charge.
	 */
	u8 default_antenna;

	u8 rssi_low_limit;

	u8 wifi_grant_override;
	u8 wifi_grant_override_val;
	u8 ext_soc_grant_override;
	u8 ext_soc_grant_override_val;
	u8 ble_grant_override;
	u8 ble_grant_override_val;

	u8 grant_override;

	u8 external_mask;

	u8 antenna_control_override;

	u8 sw_enable_register;

	/*
	 * SW index
	 * Bit 0 - SW index value
	 * Bit 1 - SW index override
	 */
	u8 sw_index;

	/*
	 * GPIO antenna index (0-5)
	 */
	u8 gpio_antenna_index_0;
	u8 gpio_antenna_index_1;
	u8 gpio_antenna_index_2;
	u8 gpio_antenna_index_3;
	u8 gpio_antenna_index_4;
	u8 gpio_antenna_index_5;

	/*
	 * Antenna selection bits
	 */
	u8 ant_sel_0;
	u8 ant_sel_1;
	u8 ant_sel_2;
	u8 ant_sel_3;
} __packed;

struct conf_thermal_thresholds {
	s16 high_threshold_24G;
	s16 low_threshold_24G;
	s16 high_threshold_5G;
	s16 low_threshold_5G;
	s16 high_sample_threshold;
	u8 enable_value;
} __attribute__((__packed__));

struct conf_limit105c_params {
	u8 enable_value;
	u8 maxMCS;
	u8 maxOFDM;
} __attribute__((__packed__));

struct conf_gpadc_params {
	u8  overrideEn;
	u8  disable_measuring;
	u8 pmcioSlop;
	u16 pmcioIntercept;
	u8 rfcioSlop;
	u16 rfcioIntercept;
	u32 measurePeriodUsec;
	u8 dcSlopExponent;
} __attribute__((__packed__));

struct cc33xx_core_conf {
	u8 enable_5ghz;
	u8 enable_ble;
	u8 enable_at_test_debug;
	u8 disable_beamforming_fftp;
	u32 ble_uart_baudrate;
	u8 enable_flow_ctrl;
	s8 ble_default_tx_power;
	u8 listen_interval;
	u8 wake_up_event;
	u8 suspend_listen_interval;
	u8 suspend_wake_up_event;
	u8 per_channel_power_limit[520];
	u32 internal_slowclk_wakeup_earlier;
	u32 internal_slowclk_open_window_longer;
	u32 external_slowclk_wakeup_earlier;
	u32 external_slowclk_open_window_longer;
	u32 slowclk_sampled_cycles;
	struct conf_coex_configuration coex_configuration;
	/* Prevent HW recovery. FW will remain stuck. */
	u8 no_recovery;
	u8 disable_logger;
	u8 mixed_mode_support;
	u8 sram_ldo_voltage_trimming;
	u32 xtal_settling_time_usec;
	u8 max_rx_ampdu_len;
	u32 country_code;
	struct conf_ant_diversity ant_diversity;
	struct conf_iomux_configuration iomux_configuration;
	struct conf_thermal_thresholds thermal_thresholds;
	struct conf_limit105c_params limit105c_params;
	struct conf_gpadc_params gpadc_params;
} __packed;

struct cc33xx_mac_conf {
	u8 ps_mode;
	u8 ps_scheme;
	u8 he_enable;
	u8 vht_enable;
	u8 ap_max_num_stations;
	u8 fw_defrag;
	u16 rx_memblks_override;
	u8 rts_mode;
} __packed;

struct cc33xx_phy_conf {
	u8 insertion_loss_2_4ghz[2];
	u8 insertion_loss_5ghz[2];
	u8 reserved_0[2];
	u8 ant_gain_2_4ghz[2];
	u8 ant_gain_5ghz[2];
	u8 reserved_1[2];
	u8 ble_ch_lim_1m[40];
	u8 ble_ch_lim_2m[40];
	u8 one_time_calibration_only;
	u8 is_diplexer_present;
	u8 num_of_antennas;
	u8 reg_domain;
	u16 calib_period;
	u8 tx_psat_compensation_2_4GHz;
	u8 tx_psat_compensation_5GHz;
	u8 psat_reserved;
	u8 Is85cDevice;
	u16 product_type;
	u32 gpio_data[4];
	u16 xtalCorrAbove95C;
	u8 reserved[6];
} __packed;

struct cc33xx_host_conf {
	struct conf_tx_settings tx;
	struct conf_conn_settings conn;
	struct conf_scan_settings scan;
	struct conf_sched_scan_settings sched_scan;
	struct conf_ht_setting ht;
	struct conf_fwlog fwlog;

} __packed;

struct cc33xx_crc_conf {
	u32 userChecksum;
} __packed;

struct cc33xx_conf_file {
	struct cc33xx_conf_header header;
	struct cc33xx_phy_conf phy;
	struct cc33xx_mac_conf mac;
	struct cc33xx_core_conf core;
	struct cc33xx_host_conf host_conf;
	struct cc33xx_crc_conf crc_conf;
} __packed;

#endif
