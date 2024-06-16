/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (C) 2022-2024 Texas Instruments Incorporated - https://www.ti.com/
 */

#ifndef __CONF_H__
#define __CONF_H__

struct cc33xx_conf_header {
	__le32 magic;
	__le32 version;
	__le32 checksum;
} __packed;

#define CC33XX_CONF_MAGIC	0x10e100ca
#define CC33XX_CONF_VERSION	0x01070069
#define CC33XX_CONF_MASK	0x0000ffff
#define CC33X_CONF_SIZE	(sizeof(struct cc33xx_conf_file))

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

#define CONF_HW_RXTX_RATE_UNSUPPORTED 0xff

enum conf_rx_queue_type {
	CONF_RX_QUEUE_TYPE_LOW_PRIORITY,  /* All except the high priority */
	CONF_RX_QUEUE_TYPE_HIGH_PRIORITY, /* Management and voice packets */
};

struct cc33xx_clk_cfg {
	u32 n;
	u32 m;
	u32 p;
	u32 q;
	u8 swallow;
};

struct conf_rx_settings {
	/* The maximum amount of time, in TU, before the
	 * firmware discards the MSDU.
	 *
	 * Range: 0 - 0xFFFFFFFF
	 */
	u32 rx_msdu_life_time;

	/* Packet detection threshold in the PHY.
	 *
	 * FIXME: details unknown.
	 */
	u32 packet_detection_threshold;

	/* The longest time the STA will wait to receive traffic from the AP
	 * after a PS-poll has been transmitted.
	 *
	 * Range: 0 - 200000
	 */
	u16 ps_poll_timeout;
	/* The longest time the STA will wait to receive traffic from the AP
	 * after a frame has been sent from an UPSD enabled queue.
	 *
	 * Range: 0 - 200000
	 */
	u16 upsd_timeout;

	/* The number of octets in an MPDU, below which an RTS/CTS
	 * handshake is not performed.
	 *
	 * Range: 0 - 4096
	 */
	u16 rts_threshold;

	/* The RX Clear Channel Assessment threshold in the PHY
	 * (the energy threshold).
	 *
	 * Range: ENABLE_ENERGY_D  == 0x140A
	 *        DISABLE_ENERGY_D == 0xFFEF
	 */
	u16 rx_cca_threshold;

	/* Occupied Rx mem-blocks number which requires interrupting the host
	 * (0 = no buffering, 0xffff = disabled).
	 *
	 * Range: uint16_t
	 */
	u16 irq_blk_threshold;

	/* Rx packets number which requires interrupting the host
	 * (0 = no buffering).
	 *
	 * Range: uint16_t
	 */
	u16 irq_pkt_threshold;

	/* Max time in msec the FW may delay RX-Complete interrupt.
	 *
	 * Range: 1 - 100
	 */
	u16 irq_timeout;

	/* The RX queue type.
	 *
	 * Range: RX_QUEUE_TYPE_RX_LOW_PRIORITY, RX_QUEUE_TYPE_RX_HIGH_PRIORITY,
	 */
	u8 queue_type;
} __packed;

#define CONF_TX_MAX_RATE_CLASSES       10

#define CONF_TX_RATE_MASK_UNSPECIFIED  0
#define CONF_TX_RATE_MASK_BASIC        (CONF_HW_BIT_RATE_1MBPS | \
					CONF_HW_BIT_RATE_2MBPS)
#define CONF_TX_RATE_RETRY_LIMIT       10

/* basic rates for p2p operations (probe req/resp, etc.) */
#define CONF_TX_RATE_MASK_BASIC_P2P    CONF_HW_BIT_RATE_6MBPS

/* Rates supported for data packets when operating as STA/AP. Note the absence
 * of the 22Mbps rate. There is a FW limitation on 12 rates so we must drop
 * one. The rate dropped is not mandatory under any operating mode.
 */
#define CONF_TX_ENABLED_RATES       (CONF_HW_BIT_RATE_1MBPS |    \
	CONF_HW_BIT_RATE_2MBPS | CONF_HW_BIT_RATE_5_5MBPS |      \
	CONF_HW_BIT_RATE_6MBPS | CONF_HW_BIT_RATE_9MBPS |        \
	CONF_HW_BIT_RATE_11MBPS | CONF_HW_BIT_RATE_12MBPS |      \
	CONF_HW_BIT_RATE_18MBPS | CONF_HW_BIT_RATE_24MBPS |      \
	CONF_HW_BIT_RATE_36MBPS | CONF_HW_BIT_RATE_48MBPS |      \
	CONF_HW_BIT_RATE_54MBPS)

#define CONF_TX_CCK_RATES  (CONF_HW_BIT_RATE_1MBPS |		\
	CONF_HW_BIT_RATE_2MBPS | CONF_HW_BIT_RATE_5_5MBPS |	\
	CONF_HW_BIT_RATE_11MBPS)

#define CONF_TX_OFDM_RATES (CONF_HW_BIT_RATE_6MBPS |             \
	CONF_HW_BIT_RATE_12MBPS | CONF_HW_BIT_RATE_24MBPS |      \
	CONF_HW_BIT_RATE_36MBPS | CONF_HW_BIT_RATE_48MBPS |      \
	CONF_HW_BIT_RATE_54MBPS)

#define CONF_TX_MCS_RATES (CONF_HW_BIT_RATE_MCS_0 |              \
	CONF_HW_BIT_RATE_MCS_1 | CONF_HW_BIT_RATE_MCS_2 |        \
	CONF_HW_BIT_RATE_MCS_3 | CONF_HW_BIT_RATE_MCS_4 |        \
	CONF_HW_BIT_RATE_MCS_5 | CONF_HW_BIT_RATE_MCS_6 |        \
	CONF_HW_BIT_RATE_MCS_7)

/* Default rates for management traffic when operating in AP mode. This
 * should be configured according to the basic rate set of the AP
 */
#define CONF_TX_AP_DEFAULT_MGMT_RATES  (CONF_HW_BIT_RATE_1MBPS | \
	CONF_HW_BIT_RATE_2MBPS | CONF_HW_BIT_RATE_5_5MBPS)

/* default rates for working as IBSS (11b and OFDM) */
#define CONF_TX_IBSS_DEFAULT_RATES  (CONF_HW_BIT_RATE_1MBPS |       \
		CONF_HW_BIT_RATE_2MBPS | CONF_HW_BIT_RATE_5_5MBPS | \
		CONF_HW_BIT_RATE_11MBPS | CONF_TX_OFDM_RATES)

struct conf_tx_rate_class {
	/* The rates enabled for this rate class.
	 *
	 * Range: CONF_HW_BIT_RATE_* bit mask
	 */
	u32 enabled_rates;

	/* The dot11 short retry limit used for TX retries.
	 *
	 * Range: uint8_t
	 */
	u8 short_retry_limit;

	/* The dot11 long retry limit used for TX retries.
	 *
	 * Range: uint8_t
	 */
	u8 long_retry_limit;

	/* Flags controlling the attributes of TX transmission.
	 *
	 * Range: bit 0: Truncate - when set, FW attempts to send a frame stop
	 *               when the total valid per-rate attempts have
	 *               been exhausted; otherwise transmissions
	 *               will continue at the lowest available rate
	 *               until the appropriate one of the
	 *               short_retry_limit, long_retry_limit,
	 *               dot11_max_transmit_msdu_life_time, or
	 *               max_tx_life_time, is exhausted.
	 *            1: Preamble Override - indicates if the preamble type
	 *               should be used in TX.
	 *            2: Preamble Type - the type of the preamble to be used by
	 *               the policy (0 - long preamble, 1 - short preamble.
	 */
	u8 aflags;
} __packed;

#define CONF_TX_MAX_AC_COUNT 4

/* Slot number setting to start transmission at PIFS interval */
#define CONF_TX_AIFS_PIFS 1
/* Slot number setting to start transmission at DIFS interval normal
 * DCF access
 */
#define CONF_TX_AIFS_DIFS 2

enum conf_tx_ac {
	CONF_TX_AC_BE = 0,         /* best effort / legacy */
	CONF_TX_AC_BK = 1,         /* background */
	CONF_TX_AC_VI = 2,         /* video */
	CONF_TX_AC_VO = 3,         /* voice */
	CONF_TX_AC_CTS2SELF = 4,   /* fictitious AC, follows AC_VO */
	CONF_TX_AC_ANY_TID = 0xff
};

struct conf_sig_weights {
	/* RSSI from beacons average weight.
	 *
	 * Range: uint8_t
	 */
	u8 rssi_bcn_avg_weight;

	/* RSSI from data average weight.
	 *
	 * Range: uint8_t
	 */
	u8 rssi_pkt_avg_weight;

	/* SNR from beacons average weight.
	 *
	 * Range: uint8_t
	 */
	u8 snr_bcn_avg_weight;

	/* SNR from data average weight.
	 *
	 * Range: uint8_t
	 */
	u8 snr_pkt_avg_weight;
} __packed;

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

	/*  The AIFSN value for the corresonding access class
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

#define CONF_TX_MAX_TID_COUNT 8

/* Allow TX BA on all TIDs but 6,7. These are currently reserved in the FW */
#define CONF_TX_BA_ENABLED_TID_BITMAP 0x3F

enum {
	CONF_CHANNEL_TYPE_DCF = 0,   /* DC/LEGACY*/
	CONF_CHANNEL_TYPE_EDCF = 1,  /* EDCA*/
	CONF_CHANNEL_TYPE_HCCA = 2,  /* HCCA*/
};

enum {
	CONF_PS_SCHEME_LEGACY = 0,
	CONF_PS_SCHEME_UPSD_TRIGGER = 1,
	CONF_PS_SCHEME_LEGACY_PSPOLL = 2,
	CONF_PS_SCHEME_SAPSD = 3,
};

enum {
	CONF_ACK_POLICY_LEGACY = 0,
	CONF_ACK_POLICY_NO_ACK = 1,
	CONF_ACK_POLICY_BLOCK = 2,
};

struct conf_tx_tid {
	u8 queue_id;
	u8 channel_type;
	u8 tsid;
	u8 ps_scheme;
	u8 ack_policy;
	u32 apsd_conf[2];
} __packed;

struct conf_tx_settings {
	/* The TX ED value for TELEC Enable/Disable.
	 *
	 * Range: 0, 1
	 */
	u8 tx_energy_detection;

	/* Configuration for rate classes for TX (currently only one
	 * rate class supported). Used in non-AP mode.
	 */
	struct conf_tx_rate_class sta_rc_conf;

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

	/* AP-mode - after this number of seconds a connected station is
	 * considered inactive.
	 */
	u16 ap_aging_period;

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

	/* The TX fragmentation threshold.
	 *
	 * Range: uint16_t
	 */
	u16 frag_threshold;

	/* Max time in msec the FW may delay frame TX-Complete interrupt.
	 *
	 * Range: uint16_t
	 */
	u16 tx_compl_timeout;

	/* Completed TX packet count which requires to issue the TX-Complete
	 * interrupt.
	 *
	 * Range: uint16_t
	 */
	u16 tx_compl_threshold;

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

	/* TX retry limits for templates
	 */
	u8 tmpl_short_retry_limit;
	u8 tmpl_long_retry_limit;

	/* Time in ms for Tx watchdog timer to expire */
	u32 tx_watchdog_timeout;

	/* when a slow link has this much packets pending, it becomes a low
	 * priority link, scheduling-wise
	 */
	u8 slow_link_thold;

	/* when a fast link has this much packets pending, it becomes a low
	 * priority link, scheduling-wise
	 */
	u8 fast_link_thold;
} __packed;

enum {
	CONF_WAKE_UP_EVENT_BEACON    = 0x00, /* Wake on every Beacon */
	CONF_WAKE_UP_EVENT_DTIM      = 0x01, /* Wake on every DTIM */
	CONF_WAKE_UP_EVENT_N_DTIM    = 0x02, /* Wake every Nth DTIM */
	CONF_WAKE_UP_EVENT_LIMIT     = CONF_WAKE_UP_EVENT_N_DTIM,
	/* Not supported: */
	CONF_WAKE_UP_EVENT_N_BEACONS = 0x03, /* Wake every Nth beacon */
	CONF_WAKE_UP_EVENT_BITS_MASK = 0x0F
};

#define CONF_MAX_BCN_FILT_IE_COUNT 32

#define CONF_BCN_RULE_PASS_ON_CHANGE         BIT(0)
#define CONF_BCN_RULE_PASS_ON_APPEARANCE     BIT(1)

#define CONF_BCN_IE_OUI_LEN    3
#define CONF_BCN_IE_VER_LEN    2

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

enum conf_bcn_filt_mode {
	CONF_BCN_FILT_MODE_DISABLED = 0,
	CONF_BCN_FILT_MODE_ENABLED = 1
};

enum conf_bet_mode {
	CONF_BET_MODE_DISABLE = 0,
	CONF_BET_MODE_ENABLE = 1,
};

struct conf_conn_settings {
	/* Enable or disable the beacon filtering.
	 *
	 * Range: CONF_BCN_FILT_MODE_*
	 */
	u8 bcn_filt_mode;

	/* Configure Beacon filter pass-thru rules.
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

	/* Beacon receive timeout.
	 *
	 * Range: uint32_t
	 */
	u32 beacon_rx_timeout;

	/* Broadcast receive timeout.
	 *
	 * Range: uint32_t
	 */
	u32 broadcast_timeout;

	/* Enable/disable reception of broadcast packets in power save mode
	 *
	 * Range: 1 - enable, 0 - disable
	 */
	u8 rx_broadcast_in_ps;

	/* Consecutive PS Poll failures before sending event to driver
	 *
	 * Range: uint8_t
	 */
	u8 ps_poll_threshold;

	/* Configuration of signal average weights.
	 */
	struct conf_sig_weights sig_weights;

	/* Specifies if beacon early termination procedure is enabled or
	 * disabled.
	 *
	 * Range: CONF_BET_MODE_*
	 */
	u8 bet_enable;

	/* Specifies the maximum number of consecutive beacons that may be
	 * early terminated. After this number is reached at least one full
	 * beacon must be correctly received in FW before beacon ET
	 * resumes.
	 *
	 * Range 0 - 255
	 */
	u8 bet_max_consecutive;

	/* Specifies the maximum number of times to try PSM entry if it fails
	 * (if sending the appropriate null-func message fails.)
	 *
	 * Range 0 - 255
	 */
	u8 psm_entry_retries;

	/* Specifies the maximum number of times to try PSM exit if it fails
	 * (if sending the appropriate null-func message fails.)
	 *
	 * Range 0 - 255
	 */
	u8 psm_exit_retries;

	/* Specifies the maximum number of times to try transmit the PSM entry
	 * null-func frame for each PSM entry attempt
	 *
	 * Range 0 - 255
	 */
	u8 psm_entry_nullfunc_retries;

	/* Specifies the dynamic PS timeout in ms that will be used
	 * by the FW when in AUTO_PS mode
	 */
	u16 dynamic_ps_timeout;

	/* Specifies whether dynamic PS should be disabled and PSM forced.
	 * This is required for certain WiFi certification tests.
	 */
	u8 forced_ps;

	/* Specifies the interval of the connection keep-alive null-func
	 * frame in ms.
	 *
	 * Range: 1000 - 3600000
	 */
	u32 keep_alive_interval;

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

struct conf_itrim_settings {
	/* enable dco itrim */
	u8 enable;

	/* moderation timeout in microsecs from the last TX */
	u32 timeout;
} __packed;

enum conf_fast_wakeup {
	CONF_FAST_WAKEUP_ENABLE,
	CONF_FAST_WAKEUP_DISABLE,
};

struct conf_pm_config_settings {
	/* Host clock settling time
	 *
	 * Range: 0 - 30000 us
	 */
	u32 host_clk_settling_time;

	/* Host fast wakeup support
	 *
	 * Range: enum conf_fast_wakeup
	 */
	u8 host_fast_wakeup_support;
} __packed;

struct conf_roam_trigger_settings {
	/* The minimum interval between two trigger events.
	 *
	 * Range: 0 - 60000 ms
	 */
	u16 trigger_pacing;

	/* The weight for rssi/beacon average calculation
	 *
	 * Range: 0 - 255
	 */
	u8 avg_weight_rssi_beacon;

	/* The weight for rssi/data frame average calculation
	 *
	 * Range: 0 - 255
	 */
	u8 avg_weight_rssi_data;

	/* The weight for snr/beacon average calculation
	 *
	 * Range: 0 - 255
	 */
	u8 avg_weight_snr_beacon;

	/* The weight for snr/data frame average calculation
	 *
	 * Range: 0 - 255
	 */
	u8 avg_weight_snr_data;
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
	u8 tx_ba_win_size;
	u16 inactivity_timeout;

	/* bitmap of enabled TIDs for TX BA sessions */
	u8 tx_ba_tid_bitmap;

	/* DEFAULT / WIDE / SISO20 */
	u8 mode;
} __packed;

struct conf_memory_settings {
	/* Number of stations supported in IBSS mode */
	u8 num_stations;

	/* Number of ssid profiles used in IBSS mode */
	u8 ssid_profiles;

	/* Number of memory buffers allocated to rx pool */
	u8 rx_block_num;

	/* Minimum number of blocks allocated to tx pool */
	u8 tx_min_block_num;

	/* Disable/Enable dynamic memory */
	u8 dynamic_memory;

	/* Minimum required free tx memory blocks in order to assure optimum
	 * performance
	 *
	 * Range: 0-120
	 */
	u8 min_req_tx_blocks;

	/* Minimum required free rx memory blocks in order to assure optimum
	 * performance
	 *
	 * Range: 0-120
	 */
	u8 min_req_rx_blocks;

	/* Minimum number of mem blocks (free+used) guaranteed for TX
	 *
	 * Range: 0-120
	 */
	u8 tx_min;
} __packed;

struct conf_rx_streaming_settings {
	/* RX Streaming duration (in msec) from last tx/rx
	 *
	 * Range: uint32_t
	 */
	u32 duration;

	/* Bitmap of tids to be polled during RX streaming.
	 * (Note: it doesn't look like it really matters)
	 *
	 * Range: 0x1-0xff
	 */
	u8 queues;

	/* RX Streaming interval.
	 * (Note:this value is also used as the rx streaming timeout)
	 * Range: 0 (disabled), 10 - 100
	 */
	u8 interval;

	/* enable rx streaming also when there is no coex activity
	 */
	u8 always;
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

#define ACX_RATE_MGMT_NUM_OF_RATES 13
struct conf_rate_policy_settings {
	u16 rate_retry_score;
	u16 per_add;
	u16 per_th1;
	u16 per_th2;
	u16 max_per;
	u8 inverse_curiosity_factor;
	u8 tx_fail_low_th;
	u8 tx_fail_high_th;
	u8 per_alpha_shift;
	u8 per_add_shift;
	u8 per_beta1_shift;
	u8 per_beta2_shift;
	u8 rate_check_up;
	u8 rate_check_down;
	u8 rate_retry_policy[13];
} __packed;

struct conf_hangover_settings {
	u32 recover_time;
	u8 hangover_period;
	u8 dynamic_mode;
	u8 early_termination_mode;
	u8 max_period;
	u8 min_period;
	u8 increase_delta;
	u8 decrease_delta;
	u8 quiet_time;
	u8 increase_time;
	u8 window_size;
} __packed;

enum {
	CLOCK_CONFIG_16_2_M	= 1,
	CLOCK_CONFIG_16_368_M,
	CLOCK_CONFIG_16_8_M,
	CLOCK_CONFIG_19_2_M,
	CLOCK_CONFIG_26_M,
	CLOCK_CONFIG_32_736_M,
	CLOCK_CONFIG_33_6_M,
	CLOCK_CONFIG_38_468_M,
	CLOCK_CONFIG_52_M,

	NUM_CLOCK_CONFIGS,
};

enum cc33xx_ht_mode {
	/* Default - use MIMO, fallback to SISO20 */
	HT_MODE_DEFAULT = 0,

	/* Wide - use SISO40 */
	HT_MODE_WIDE = 1,

	/* Use SISO20 */
	HT_MODE_SISO20 = 2,
};

struct conf_ap_sleep_settings {
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

#define CHANNELS_COUNT 39 /* 14 2.4GHz channels, 25 5GHz channels*/
#define PER_CHANNEL_REG_RULE_BYTES 13
#define REG_RULES_COUNT (CHANNELS_COUNT * PER_CHANNEL_REG_RULE_BYTES) /* 507 */

/* TX Power limitation for a channel, used for reg domain */
struct conf_channel_power_limit {
	u32 reg_lim_0;
	u32 reg_lim_1;
	u32 reg_lim_2;
	u8  reg_lim_3;
} __packed;

struct conf_coex_configuration {
	/* Work without Coex HW
	 *
	 * Range: 1 - YES, 0 - NO
	 */
	u8 disable_coex;
	/* Yes/No Choose if External SoC entity is connected
	 *
	 * Range: 1 - YES, 0 - NO
	 */
	u8 is_ext_soc_enable;
	/* External SoC grant polarity
	 *
	 * 0 - Active Low
	 *
	 * 1 - Active High (Default)
	 */
	u8 ext_soc_grant_polarity;
	/* External SoC priority polarity
	 *
	 * 0 - Active Low (Default)
	 *
	 * 1 - Active High
	 */
	u8 ext_soc_priority_polarity;
	/* External SoC request polarity
	 *
	 * 0 - Active Low (Default)
	 *
	 * 1 - Active High
	 */
	u8 ext_soc_request_polarity;
	u16 ext_soc_min_grant_time;
	u16 ext_soc_max_grant_time;
	/* Range: 0 - 20 us
	 */
	u8 ext_soc_t2_time;

	u8 ext_soc_to_wifi_grant_delay;
	u8 ext_soc_to_ble_grant_delay;
} __packed;

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
} __packed;

struct cc33xx_core_conf {
	u8 enable_5ghz;
	u8 enable_ble;
	u8 enable_at_test_debug;
	u8 disable_beamforming_fftp;
	u32 ble_uart_baudrate;
	u8 enable_flow_ctrl;
	u8 listen_interval;
	u8 wake_up_event;
	u8 suspend_listen_interval;
	u8 suspend_wake_up_event;
	u8 per_channel_power_limit[507];
	u32 internal_slowclk_wakeup_earlier;
	u32 internal_slowclk_open_window_longer;
	u32 external_slowclk_wakeup_earlier;
	u32 external_slowclk_open_window_longer;
	struct conf_coex_configuration coex_configuration;
	/* Prevent HW recovery. FW will remain stuck. */
	u8 no_recovery;
	u8 disable_logger;
	u8 mixed_mode_support;
	u8 sram_ldo_voltage_trimming;
	u32 xtal_settling_time_usec;
	struct conf_ant_diversity ant_diversity;
	struct conf_iomux_configuration iomux_configuration;
} __packed;

struct cc33xx_mac_conf {
	u8 ps_scheme;
	u8 he_enable;
	u8 ap_max_num_stations;
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
} __packed;

struct cc33xx_host_conf {
	struct conf_rx_settings rx;
	struct conf_tx_settings tx;
	struct conf_conn_settings conn;
	struct conf_itrim_settings itrim;
	struct conf_pm_config_settings pm_config;
	struct conf_roam_trigger_settings roam_trigger;
	struct conf_scan_settings scan;
	struct conf_sched_scan_settings sched_scan;
	struct conf_ht_setting ht;
	struct conf_memory_settings mem;
	struct conf_rx_streaming_settings rx_streaming;
	struct conf_fwlog fwlog;
	struct conf_rate_policy_settings rate;
	struct conf_hangover_settings hangover;
	struct conf_ap_sleep_settings ap_sleep;

} __packed;

struct cc33xx_conf_file {
	struct cc33xx_conf_header header;
	struct cc33xx_phy_conf phy;
	struct cc33xx_mac_conf mac;
	struct cc33xx_core_conf core;
	struct cc33xx_host_conf host_conf;
} __packed;

#endif
