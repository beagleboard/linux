/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * This file is part of wl1271
 *
 * Copyright (C) 1998-2009 Texas Instruments. All rights reserved.
 * Copyright (C) 2008-2010 Nokia Corporation
 *
 * Contact: Luciano Coelho <luciano.coelho@nokia.com>
 */

#ifndef __ACX_H__
#define __ACX_H__


#include "wlcore.h"
#include "cmd.h"

/*************************************************************************

    Host Interrupt Register (WiLink -> Host)

**************************************************************************/
/* HW Initiated interrupt Watchdog timer expiration */
#define WL1271_ACX_INTR_WATCHDOG           BIT(0)
/* Init sequence is done (masked interrupt, detection through polling only ) */
#define WL1271_ACX_INTR_INIT_COMPLETE      BIT(1)
/* Event was entered to Event MBOX #A*/
#define WL1271_ACX_INTR_EVENT_A            BIT(2)
/* Event was entered to Event MBOX #B*/
#define WL1271_ACX_INTR_EVENT_B            BIT(3)
/* Command processing completion*/
#define WL1271_ACX_INTR_CMD_COMPLETE       BIT(4)
/* Signaling the host on HW wakeup */
#define WL1271_ACX_INTR_HW_AVAILABLE       BIT(5)
/* The MISC bit is used for aggregation of RX, TxComplete and TX rate update */
#define WL1271_ACX_INTR_DATA               BIT(6)
/* Trace message on MBOX #A */
#define WL1271_ACX_INTR_TRACE_A            BIT(7)
/* Trace message on MBOX #B */
#define WL1271_ACX_INTR_TRACE_B            BIT(8)
/* SW FW Initiated interrupt Watchdog timer expiration */
#define WL1271_ACX_SW_INTR_WATCHDOG        BIT(9)

#define WL1271_ACX_INTR_ALL             0xFFFFFFFF

/* all possible interrupts - only appropriate ones will be masked in */
#define WLCORE_ALL_INTR_MASK		(WL1271_ACX_INTR_WATCHDOG     | \
					WL1271_ACX_INTR_EVENT_A       | \
					WL1271_ACX_INTR_EVENT_B       | \
					WL1271_ACX_INTR_HW_AVAILABLE  | \
					WL1271_ACX_INTR_DATA          | \
					WL1271_ACX_SW_INTR_WATCHDOG)

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

struct acx_error_counter {
	struct acx_header header;

	/* The number of PLCP errors since the last time this */
	/* information element was interrogated. This field is */
	/* automatically cleared when it is interrogated.*/
	__le32 PLCP_error;

	/* The number of FCS errors since the last time this */
	/* information element was interrogated. This field is */
	/* automatically cleared when it is interrogated.*/
	__le32 FCS_error;

	/* The number of MPDUs without PLCP header errors received*/
	/* since the last time this information element was interrogated. */
	/* This field is automatically cleared when it is interrogated.*/
	__le32 valid_frame;

	/* the number of missed sequence numbers in the squentially */
	/* values of frames seq numbers */
	__le32 seq_num_miss;
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

enum {
	HOSTIF_PCI_MASTER_HOST_INDIRECT,
	HOSTIF_PCI_MASTER_HOST_DIRECT,
	HOSTIF_SLAVE,
	HOSTIF_PKT_RING,
	HOSTIF_DONTCARE = 0xFF
};

#define DEFAULT_UCAST_PRIORITY          0
#define DEFAULT_RX_Q_PRIORITY           0
#define DEFAULT_RXQ_PRIORITY            0 /* low 0 .. 15 high  */
#define DEFAULT_RXQ_TYPE                0x07    /* All frames, Data/Ctrl/Mgmt */
#define TRACE_BUFFER_MAX_SIZE           256

#define  DP_RX_PACKET_RING_CHUNK_SIZE 1600
#define  DP_TX_PACKET_RING_CHUNK_SIZE 1600
#define  DP_RX_PACKET_RING_CHUNK_NUM 2
#define  DP_TX_PACKET_RING_CHUNK_NUM 2
#define  DP_TX_COMPLETE_TIME_OUT 20

#define TX_MSDU_LIFETIME_MIN       0
#define TX_MSDU_LIFETIME_MAX       3000
#define TX_MSDU_LIFETIME_DEF       512
#define RX_MSDU_LIFETIME_MIN       0
#define RX_MSDU_LIFETIME_MAX       0xFFFFFFFF
#define RX_MSDU_LIFETIME_DEF       512000

struct acx_rx_msdu_lifetime {
	struct acx_header header;

	/*
	 * The maximum amount of time, in TU, before the
	 * firmware discards the MSDU.
	 */
	__le32 lifetime;
} __packed;

enum acx_slot_type {
	SLOT_TIME_LONG = 0,
	SLOT_TIME_SHORT = 1,
	DEFAULT_SLOT_TIME = SLOT_TIME_SHORT,
	MAX_SLOT_TIMES = 0xFF
};

#define STATION_WONE_INDEX 0

struct acx_slot {
	struct acx_header header;

	u8 role_id;
	u8 slot_time;
	u8 reserved[2];
} __packed;


#define ACX_MC_ADDRESS_GROUP_MAX	(8)
#define ADDRESS_GROUP_MAX_LEN	        (ETH_ALEN * ACX_MC_ADDRESS_GROUP_MAX)

struct acx_dot11_grp_addr_tbl {
	struct acx_header header;

	u8 role_id;
	u8 enabled;
	u8 num_groups;
	u8 pad[1];
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
#define	BEACON_FILTER_TABLE_MAX_IE_NUM		       (32)
#define BEACON_FILTER_TABLE_MAX_VENDOR_SPECIFIC_IE_NUM (6)
#define BEACON_FILTER_TABLE_IE_ENTRY_SIZE	       (2)
#define BEACON_FILTER_TABLE_EXTRA_VENDOR_SPECIFIC_IE_SIZE (6)
#define BEACON_FILTER_TABLE_MAX_SIZE ((BEACON_FILTER_TABLE_MAX_IE_NUM * \
			    BEACON_FILTER_TABLE_IE_ENTRY_SIZE) + \
			   (BEACON_FILTER_TABLE_MAX_VENDOR_SPECIFIC_IE_NUM * \
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

struct acx_bt_wlan_coex {
	struct acx_header header;

	u8 enable;
	u8 pad[3];
} __packed;

struct acx_bt_wlan_coex_param {
	struct acx_header header;

	__le32 params[WLCORE_CONF_SG_PARAMS_MAX];
	u8 param_idx;
	u8 padding[3];
} __packed;

struct acx_dco_itrim_params {
	struct acx_header header;

	u8 enable;
	u8 padding[3];
	__le32 timeout;
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

#define SCAN_PASSIVE		BIT(0)
#define SCAN_5GHZ_BAND		BIT(1)
#define SCAN_TRIGGERED		BIT(2)
#define SCAN_PRIORITY_HIGH	BIT(3)

/* When set, disable HW encryption */
#define DF_ENCRYPTION_DISABLE      0x01
#define DF_SNIFF_MODE_ENABLE       0x80

struct acx_feature_config {
	struct acx_header header;

	u8 role_id;
	u8 padding[3];
	__le32 options;
	__le32 data_flow_options;
} __packed;

struct acx_current_tx_power {
	struct acx_header header;

	u8  role_id;
	u8  current_tx_power;
	u8  padding[2];
} __packed;

struct acx_wake_up_condition {
	struct acx_header header;

	u8 role_id;
	u8 wake_up_event; /* Only one bit can be set */
	u8 listen_interval;
	u8 pad[1];
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
	//u8 padding[3]; incase total size of the struct variables isnt a multiple of 32 bit then add padding
}__packed;


struct acx_aid {
	struct acx_header header;

	/*
	 * To be set when associated with an AP.
	 *
	 */
	u8 role_id;
	u8 reserved;
	__le16 aid;
} __packed;

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

struct acx_rate_class {
	__le32 enabled_rates;
	u8 short_retry_limit;
	u8 long_retry_limit;
	u8 aflags;
	u8 reserved;
};

struct ap_rates_class_cfg {

	struct acx_header header;
	u8 role_id;
	__le32 basic_rates_set;
	__le32 supported_rates;
    u8 padding[3];
}__packed;

struct acx_rate_policy {
	struct acx_header header;

	__le32 rate_policy_idx;
	struct acx_rate_class rate_policy;
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

    u8 reserved[1];

} __packed;

struct acx_ac_cfg {
	struct acx_header header;
	u8 role_id;
	u8 ac;
	u8 aifsn;
	u8 cw_min;
	__le16 cw_max;
	__le16 tx_op_limit;
	u8 ps_scheme;
} __packed;

struct acx_tid_config {
	struct acx_header header;
	u8 role_id;
	u8 queue_id;
	u8 channel_type;
	u8 tsid;
	u8 ps_scheme;
	u8 ack_policy;
	u8 padding[2];
	__le32 apsd_conf[2];
} __packed;

struct acx_frag_threshold {
	struct acx_header header;
	__le16 frag_threshold;
	u8 padding[2];
} __packed;

struct acx_tx_config_options {
	struct acx_header header;
	__le16 tx_compl_timeout;     /* msec */
	__le16 tx_compl_threshold;   /* number of packets */
} __packed;

struct wl12xx_acx_config_memory {
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

struct wl1271_acx_mem_map {
	struct acx_header header;

	/* Number of blocks FW allocated for TX packets */
	__le32 num_tx_mem_blocks;

	/* Number of blocks FW allocated for RX packets */
	__le32 num_rx_mem_blocks;

	/* Number of TX descriptor that allocated. */
	__le32 num_tx_descriptor;

	__le32 tx_result;

} __packed;

struct wl1271_acx_rx_config_opt {
	struct acx_header header;

	__le16 mblk_threshold;
	__le16 threshold;
	__le16 timeout;
	u8 queue_type;
	u8 reserved;
} __packed;


struct wl1271_acx_bet_enable {
	struct acx_header header;

	u8 role_id;
	u8 enable;
	u8 max_consecutive;
	u8 padding[1];
} __packed;

#define ACX_IPV4_VERSION 4
#define ACX_IPV6_VERSION 6
#define ACX_IPV4_ADDR_SIZE 4

/* bitmap of enabled arp_filter features */
#define ACX_ARP_FILTER_ARP_FILTERING	BIT(0)
#define ACX_ARP_FILTER_AUTO_ARP		BIT(1)

struct wl1271_acx_arp_filter {
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

struct wl1271_acx_pm_config {
	struct acx_header header;

	__le32 host_clk_settling_time;
	u8 host_fast_wakeup_support;
	u8 padding[3];
} __packed;

/* TODO: maybe this needs to be moved somewhere else? */
#define HOST_IF_CFG_RX_FIFO_ENABLE     BIT(0)
#define HOST_IF_CFG_TX_EXTRA_BLKS_SWAP BIT(1)
#define HOST_IF_CFG_TX_PAD_TO_SDIO_BLK BIT(3)
#define HOST_IF_CFG_RX_PAD_TO_SDIO_BLK BIT(4)
#define HOST_IF_CFG_ADD_RX_ALIGNMENT   BIT(6)

enum {
	WL1271_ACX_TRIG_TYPE_LEVEL = 0,
	WL1271_ACX_TRIG_TYPE_EDGE,
};

enum {
	WL1271_ACX_TRIG_DIR_LOW = 0,
	WL1271_ACX_TRIG_DIR_HIGH,
	WL1271_ACX_TRIG_DIR_BIDIR,
};

enum {
	WL1271_ACX_TRIG_ENABLE = 1,
	WL1271_ACX_TRIG_DISABLE,
};

enum {
	WL1271_ACX_TRIG_METRIC_RSSI_BEACON = 0,
	WL1271_ACX_TRIG_METRIC_RSSI_DATA,
	WL1271_ACX_TRIG_METRIC_SNR_BEACON,
	WL1271_ACX_TRIG_METRIC_SNR_DATA,
};

enum {
	WL1271_ACX_TRIG_IDX_RSSI = 0,
	WL1271_ACX_TRIG_COUNT = 8,
};

struct wl1271_acx_rssi_snr_trigger {
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

struct wl1271_acx_rssi_snr_avg_weights {
	struct acx_header header;

	u8 role_id;
	u8 padding[3];
	u8 rssi_beacon;
	u8 rssi_data;
	u8 snr_beacon;
	u8 snr_data;
};


/* special capability bit (not employed by the 802.11n spec) */
#define WL12XX_HT_CAP_HT_OPERATION BIT(16)

/*
 * ACX_PEER_HT_CAP
 * Configure HT capabilities - declare the capabilities of the peer
 * we are connected to.
 */
struct wl1271_acx_ht_capabilities {
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
struct wl1271_acx_ht_information {
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

	u32 he_operation;
	//u8 bss_color_info;

	u16 bss_basic_mcs_set;
	u8 qos_info_more_data_ack_bit;

} __packed;

struct wl1271_acx_ba_initiator_policy {
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
	u16 inactivity_timeout;

	u8 padding[2];
} __packed;

struct wl1271_acx_ba_receiver_setup {
	struct acx_header header;

	/* Specifies link id, range 0-31 */
	u8 hlid;

	u8 tid;

	u8 enable;

	/* Windows size in number of packets */
	u8 win_size;

	/* BA session starting sequence number.  RANGE 0-FFF */
	u16 ssn;

	u8 padding[2];
} __packed;

struct calibration_header_fw {

	/* chip id to assign each chip with its appropriate data */
	u8 chip_id[ETH_ALEN];
	/* payload can be of different length, chosen by user*/
	u16 length;

} __packed;


struct calibration_header {
	/* preamble static pattern */
	u16 static_pattern;

	struct calibration_header_fw cal_header_fw;

} __packed;







struct wl1271_acx_static_calibration_cfg {

	struct acx_header header;
	
	bool valid_data;
	u8 file_version;
	u8 payload_struct_version;
	u8 padding;
	struct calibration_header_fw calibration_header;
	u8  payload[0];
	
} __packed;

struct wl12xx_acx_fw_tsf_information {
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

struct wl1271_acx_ap_max_tx_retry {
	struct acx_header header;

	u8 role_id;
	u8 padding_1;

	/*
	 * the number of frames transmission failures before
	 * issuing the aging event.
	 */
	__le16 max_tx_retry;
} __packed;

struct wl1271_acx_config_ps {
	struct acx_header header;

	u8 exit_retries;
	u8 enter_retries;
	u8 padding[2];
	__le32 null_data_rate;
} __packed;

struct wl1271_acx_inconnection_sta {
	struct acx_header header;

	u8 addr[ETH_ALEN];
	u8 role_id;
	u8 padding;
} __packed;

#define ACX_RATE_MGMT_ALL_PARAMS 0xff
struct wl12xx_acx_set_rate_mgmt_params {
	struct acx_header header;

	u8 index; /* 0xff to configure all params */
	u8 padding1;
	__le16 rate_retry_score;
	__le16 per_add;
	__le16 per_th1;
	__le16 per_th2;
	__le16 max_per;
	u8 inverse_curiosity_factor;
	u8 tx_fail_low_th;
	u8 tx_fail_high_th;
	u8 per_alpha_shift;
	u8 per_add_shift;
	u8 per_beta1_shift;
	u8 per_beta2_shift;
	u8 rate_check_up;
	u8 rate_check_down;
	u8 rate_retry_policy[ACX_RATE_MGMT_NUM_OF_RATES];
	u8 padding2[2];
} __packed;

struct wl12xx_acx_config_hangover {
	struct acx_header header;

	__le32 recover_time;
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
	u8 padding[2];
} __packed;


struct acx_default_rx_filter {
	struct acx_header header;
	u8 enable;

	/* action of type FILTER_XXX */
	u8 default_action;

	u8 pad[2];
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
	u32	missed_beacons;
	u8	snr_data;
	u8	snr_bacon;
	s8	rssi_data;
	s8	rssi_beacon;
} __packed;

typedef enum
{
    CTS_PROTECTION_CFG           = 0,
    TX_PARAMS_CFG                = 1,
    ASSOC_INFO_CFG               = 2,
    PEER_CAP_CFG                 = 3,
    BSS_OPERATION_CFG            = 4,
    SLOT_CFG                     = 5,
    PREAMBLE_TYPE_CFG            = 6,
    DOT11_GROUP_ADDRESS_TBL      = 7, 
    BA_SESSION_RX_SETUP_CFG      = 8,
    ACX_SLEEP_AUTH               = 9,
    STATIC_CALIBRATION_CFG       = 10,
	AP_RATES_CFG                 = 11,

    LAST_CFG_VALUE                  ,
    MAX_DOT11_CFG = LAST_CFG_VALUE   ,

    MAX_CFG = 0xFFFF   /*force enumeration to 16bits*/
} Cfg_e;

typedef enum
{
	UPLINK_MULTI_USER_CFG       ,
    UPLINK_MULTI_USER_DATA_CFG  ,
    OPERATION_MODE_CTRL_CFG     ,
    UPLINK_POWER_HEADER_CFG     ,
    MCS_FIXED_RATE_CFG          ,
    GI_LTF_CFG                  ,
    TRANSMIT_OMI_CFG            ,
    TB_ONLY_CFG                 ,
    BA_SESSION_CFG              ,
	FORCE_PS_CFG                ,
	RATE_OVERRRIDE_CFG          ,
	BLS_CFG                     ,
	BLE_ENABLE                  , 

	LAST_DEBUG_VALUE            ,

    MAX_DEBUG = 0xFFFF   			/*force enumeration to 16bits*/

}cmdDebug_e;

typedef enum
{
    BEACON_RSSI_INTR                ,

    LAST_DEBUG_READ_VALUE           ,

    MAX_DEBUG_READ = 0xFFFF   /*force enumeration to 16bits*/
} cmdDebugRead_e;

typedef enum
{
    MEM_MAP_INTR                 = 0,

    LAST_IE_VALUE                  ,
    MAX_DOT11_IE = LAST_IE_VALUE   ,

    MAX_IE = 0xFFFF   /*force enumeration to 16bits*/
} Interrogate_e;

//TODO: RazB - Need to remove this enum when we finish over all the commands
enum {

	ACX_WAKE_UP_CONDITIONS           = LAST_CFG_VALUE,
	ACX_MEM_CFG                      ,
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
	ACX_ENABLE_RX_DATA_FILTER        ,
	ACX_SET_RX_DATA_FILTER           ,
	ACX_GET_DATA_FILTER_STATISTICS   ,
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

struct wl18xx_acx_clear_statistics {
	struct acx_header header;
};



/* numbers of bits the length field takes (add 1 for the actual number) */
#define WL18XX_HOST_IF_LEN_SIZE_FIELD 15

#define WL18XX_ACX_EVENTS_VECTOR	(WL1271_ACX_INTR_WATCHDOG	| \
					 WL1271_ACX_INTR_INIT_COMPLETE	| \
					 WL1271_ACX_INTR_EVENT_A	| \
					 WL1271_ACX_INTR_EVENT_B	| \
					 WL1271_ACX_INTR_CMD_COMPLETE	| \
					 WL1271_ACX_INTR_HW_AVAILABLE	| \
					 WL1271_ACX_INTR_DATA		| \
					 WL1271_ACX_SW_INTR_WATCHDOG)

#define WL18XX_INTR_MASK		(WL1271_ACX_INTR_WATCHDOG	| \
					 WL1271_ACX_INTR_EVENT_A	| \
					 WL1271_ACX_INTR_EVENT_B	| \
					 WL1271_ACX_INTR_HW_AVAILABLE	| \
					 WL1271_ACX_INTR_DATA		| \
					 WL1271_ACX_SW_INTR_WATCHDOG)

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

enum {
	CHECKSUM_OFFLOAD_DISABLED = 0,
	CHECKSUM_OFFLOAD_ENABLED  = 1,
	CHECKSUM_OFFLOAD_FAKE_RX  = 2,
	CHECKSUM_OFFLOAD_INVALID  = 0xFF
};

struct wl18xx_acx_checksum_state {
	struct acx_header header;

	 /* enum acx_checksum_state */
	u8 checksum_state;
	u8 pad[3];
} __packed;


struct wl18xx_acx_error_stats {
	u32 error_frame_non_ctrl;
	u32 error_frame_ctrl;
	u32 error_frame_during_protection;
	u32 null_frame_tx_start;
	u32 null_frame_cts_start;
	u32 bar_retry;
	u32 num_frame_cts_nul_flid;
	u32 tx_abort_failure;
	u32 tx_resume_failure;
	u32 rx_cmplt_db_overflow_cnt;
	u32 elp_while_rx_exch;
	u32 elp_while_tx_exch;
	u32 elp_while_tx;
	u32 elp_while_nvic_pending;
	u32 rx_excessive_frame_len;
	u32 burst_mismatch;
	u32 tbc_exch_mismatch;
} __packed;

#define NUM_OF_RATES_INDEXES 30
struct wl18xx_acx_tx_stats {
	u32 tx_prepared_descs;
	u32 tx_cmplt;
	u32 tx_template_prepared;
	u32 tx_data_prepared;
	u32 tx_template_programmed;
	u32 tx_data_programmed;
	u32 tx_burst_programmed;
	u32 tx_starts;
	u32 tx_stop;
	u32 tx_start_templates;
	u32 tx_start_int_templates;
	u32 tx_start_fw_gen;
	u32 tx_start_data;
	u32 tx_start_null_frame;
	u32 tx_exch;
	u32 tx_retry_template;
	u32 tx_retry_data;
	u32 tx_retry_per_rate[NUM_OF_RATES_INDEXES];
	u32 tx_exch_pending;
	u32 tx_exch_expiry;
	u32 tx_done_template;
	u32 tx_done_data;
	u32 tx_done_int_template;
	u32 tx_cfe1;
	u32 tx_cfe2;
	u32 frag_called;
	u32 frag_mpdu_alloc_failed;
	u32 frag_init_called;
	u32 frag_in_process_called;
	u32 frag_tkip_called;
	u32 frag_key_not_found;
	u32 frag_need_fragmentation;
	u32 frag_bad_mblk_num;
	u32 frag_failed;
	u32 frag_cache_hit;
	u32 frag_cache_miss;
} __packed;

struct wl18xx_acx_rx_stats {
	u32 rx_beacon_early_term;
	u32 rx_out_of_mpdu_nodes;
	u32 rx_hdr_overflow;
	u32 rx_dropped_frame;
	u32 rx_done_stage;
	u32 rx_done;
	u32 rx_defrag;
	u32 rx_defrag_end;
	u32 rx_cmplt;
	u32 rx_pre_complt;
	u32 rx_cmplt_task;
	u32 rx_phy_hdr;
	u32 rx_timeout;
	u32 rx_rts_timeout;
	u32 rx_timeout_wa;
	u32 defrag_called;
	u32 defrag_init_called;
	u32 defrag_in_process_called;
	u32 defrag_tkip_called;
	u32 defrag_need_defrag;
	u32 defrag_decrypt_failed;
	u32 decrypt_key_not_found;
	u32 defrag_need_decrypt;
	u32 rx_tkip_replays;
	u32 rx_xfr;
} __packed;

struct wl18xx_acx_isr_stats {
	u32 irqs;
} __packed;

#define PWR_STAT_MAX_CONT_MISSED_BCNS_SPREAD 10

struct wl18xx_acx_pwr_stats {
	u32 missing_bcns_cnt;
	u32 rcvd_bcns_cnt;
	u32 connection_out_of_sync;
	u32 cont_miss_bcns_spread[PWR_STAT_MAX_CONT_MISSED_BCNS_SPREAD];
	u32 rcvd_awake_bcns_cnt;
	u32 sleep_time_count;
	u32 sleep_time_avg;
	u32 sleep_cycle_avg;
	u32 sleep_percent;
	u32 ap_sleep_active_conf;
	u32 ap_sleep_user_conf;
	u32 ap_sleep_counter;
} __packed;

struct wl18xx_acx_rx_filter_stats {
	u32 beacon_filter;
	u32 arp_filter;
	u32 mc_filter;
	u32 dup_filter;
	u32 data_filter;
	u32 ibss_filter;
	u32 protection_filter;
	u32 accum_arp_pend_requests;
	u32 max_arp_queue_dep;
} __packed;

struct wl18xx_acx_rx_rate_stats {
	u32 rx_frames_per_rates[50];
} __packed;

#define AGGR_STATS_TX_AGG	16
#define AGGR_STATS_RX_SIZE_LEN	16

struct wl18xx_acx_aggr_stats {
	u32 tx_agg_rate[AGGR_STATS_TX_AGG];
	u32 tx_agg_len[AGGR_STATS_TX_AGG];
	u32 rx_size[AGGR_STATS_RX_SIZE_LEN];
} __packed;

#define PIPE_STATS_HW_FIFO	11

struct wl18xx_acx_pipeline_stats {
	u32 hs_tx_stat_fifo_int;
	u32 hs_rx_stat_fifo_int;
	u32 enc_tx_stat_fifo_int;
	u32 enc_rx_stat_fifo_int;
	u32 rx_complete_stat_fifo_int;
	u32 pre_proc_swi;
	u32 post_proc_swi;
	u32 sec_frag_swi;
	u32 pre_to_defrag_swi;
	u32 defrag_to_rx_xfer_swi;
	u32 dec_packet_in;
	u32 dec_packet_in_fifo_full;
	u32 dec_packet_out;
	u16 pipeline_fifo_full[PIPE_STATS_HW_FIFO];
	u16 padding;
} __packed;

#define DIVERSITY_STATS_NUM_OF_ANT	2

struct wl18xx_acx_diversity_stats {
	u32 num_of_packets_per_ant[DIVERSITY_STATS_NUM_OF_ANT];
	u32 total_num_of_toggles;
} __packed;

struct wl18xx_acx_thermal_stats {
	u16 irq_thr_low;
	u16 irq_thr_high;
	u16 tx_stop;
	u16 tx_resume;
	u16 false_irq;
	u16 adc_source_unexpected;
} __packed;

#define WL18XX_NUM_OF_CALIBRATIONS_ERRORS 18
struct wl18xx_acx_calib_failure_stats {
	u16 fail_count[WL18XX_NUM_OF_CALIBRATIONS_ERRORS];
	u32 calib_count;
} __packed;

struct wl18xx_roaming_stats {
	s32 rssi_level;
} __packed;

struct wl18xx_dfs_stats {
	u32 num_of_radar_detections;
} __packed;

struct wl18xx_acx_statistics {
	struct acx_header header;

	struct wl18xx_acx_error_stats		error;
	struct wl18xx_acx_tx_stats		tx;
	struct wl18xx_acx_rx_stats		rx;
	struct wl18xx_acx_isr_stats		isr;
	struct wl18xx_acx_pwr_stats		pwr;
	struct wl18xx_acx_rx_filter_stats	rx_filter;
	struct wl18xx_acx_rx_rate_stats		rx_rate;
	struct wl18xx_acx_aggr_stats		aggr_size;
	struct wl18xx_acx_pipeline_stats	pipeline;
	struct wl18xx_acx_diversity_stats	diversity;
	struct wl18xx_acx_thermal_stats		thermal;
	struct wl18xx_acx_calib_failure_stats	calib;
	struct wl18xx_roaming_stats		roaming;
	struct wl18xx_dfs_stats			dfs;
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
 * this struct is very similar to wl1271_acx_ht_capabilities, with the
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

    u8 padding[3];
} __packed;

/*
 * ACX_INTERRUPT_NOTIFY
 * enable/disable fast-link/PSM notification from FW
 */
struct wl18xx_acx_interrupt_notify {
	struct acx_header header;
	u32 enable;
};

/*
 * ACX_RX_BA_FILTER
 * enable/disable RX BA filtering in FW
 */
struct wl18xx_acx_rx_ba_filter {
	struct acx_header header;
	u32 enable;
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

/*
 * ACX_TIME_SYNC_CFG
 * configure the time sync parameters
 */
struct acx_time_sync_cfg {
	struct acx_header header;
	u8 sync_mode;
	u8 zone_mac_addr[ETH_ALEN];
	u8 padding[1];
} __packed;


int cc33xx_acx_wake_up_conditions(struct wl1271 *wl,
				  struct wl12xx_vif *wlvif,
				  u8 wake_up_event, u8 listen_interval);
int cc33xx_acx_sleep_auth(struct wl1271 *wl, u8 sleep_auth);
int cc33xx_ble_enable(struct wl1271 *wl, u8 ble_enable);
int cc33xx_acx_tx_power(struct wl1271 *wl, struct wl12xx_vif *wlvif,
			int power);
int cc33xx_acx_feature_cfg(struct wl1271 *wl, struct wl12xx_vif *wlvif);
int wl1271_acx_mem_map(struct wl1271 *wl,
		       struct acx_header *mem_map, size_t len);
int wl1271_acx_rx_msdu_life_time(struct wl1271 *wl);
int wl1271_acx_slot(struct wl1271 *wl, struct wl12xx_vif *wlvif,
		    enum acx_slot_type slot_time);
int cc33xx_acx_group_address_tbl(struct wl1271 *wl, struct wl12xx_vif *wlvif,
				 bool enable, void *mc_list, u32 mc_list_len);
int wl1271_acx_service_period_timeout(struct wl1271 *wl,
				      struct wl12xx_vif *wlvif);
int cc33xx_acx_rts_threshold(struct wl1271 *wl, struct wl12xx_vif *wlvif,
			     u32 rts_threshold);
int wl1271_acx_dco_itrim_params(struct wl1271 *wl);
int cc33xx_acx_beacon_filter_opt(struct wl1271 *wl, struct wl12xx_vif *wlvif,
				 bool enable_filter);
int wl1271_acx_beacon_filter_table(struct wl1271 *wl,
				   struct wl12xx_vif *wlvif);
int wl1271_acx_conn_monit_params(struct wl1271 *wl, struct wl12xx_vif *wlvif,
				 bool enable);
int wl1271_acx_sg_enable(struct wl1271 *wl, bool enable);
int wl12xx_acx_sg_cfg(struct wl1271 *wl);
int wl1271_acx_cca_threshold(struct wl1271 *wl);
int wl1271_acx_bcn_dtim_options(struct wl1271 *wl, struct wl12xx_vif *wlvif);
int cc33xx_assoc_info_cfg(struct wl1271 *wl, struct wl12xx_vif *wlvif, struct ieee80211_sta *sta,u16 aid);
int wl1271_acx_aid(struct wl1271 *wl, struct wl12xx_vif *wlvif, u16 aid);
int wl1271_acx_event_mbox_mask(struct wl1271 *wl, u32 event_mask);
int wl1271_acx_set_preamble(struct wl1271 *wl, struct wl12xx_vif *wlvif,
			    enum acx_preamble_type preamble);
int wl1271_acx_cts_protect(struct wl1271 *wl, struct wl12xx_vif *wlvif,
			   enum acx_ctsprotect_type ctsprotect);
int wl1271_acx_statistics(struct wl1271 *wl, void *stats);
int cc33xx_tx_param_cfg(struct wl1271 *wl, struct wl12xx_vif *wlvif,
              u8 ac, u8 cw_min, u16 cw_max, u8 aifsn, u16 txop, bool acm,
              u8 ps_schem, u8 is_mu_edca, u8 mu_edca_aifs, u8 mu_edca_ecw_min_max,
              u8 mu_edca_timer);
int cc33xx_update_ap_rates(struct wl1271 *wl,u8 role_id,u32 basic_rates_set,u32 supported_rates);
int wl1271_acx_ac_cfg(struct wl1271 *wl, struct wl12xx_vif *wlvif,
		      u8 ac, u8 cw_min, u16 cw_max, u8 aifsn, u16 txop);
int wl1271_acx_tid_cfg(struct wl1271 *wl, struct wl12xx_vif *wlvif,
		       u8 queue_id, u8 channel_type,
		       u8 tsid, u8 ps_scheme, u8 ack_policy,
		       u32 apsd_conf0, u32 apsd_conf1);
int cc33xx_acx_frag_threshold(struct wl1271 *wl, u32 frag_threshold);
int wl1271_acx_tx_config_options(struct wl1271 *wl);
int wl12xx_acx_mem_cfg(struct wl1271 *wl);
int wl1271_acx_init_mem_config(struct wl1271 *wl);
int wl1271_acx_init_rx_interrupt(struct wl1271 *wl);
int wl1271_acx_smart_reflex(struct wl1271 *wl);
int wl1271_acx_bet_enable(struct wl1271 *wl, struct wl12xx_vif *wlvif,
			  bool enable);
int wl1271_acx_arp_ip_filter(struct wl1271 *wl, struct wl12xx_vif *wlvif,
			     u8 enable, __be32 address);
int wl1271_acx_pm_config(struct wl1271 *wl);
int wl1271_acx_rssi_snr_trigger(struct wl1271 *wl, struct wl12xx_vif *wlvif,
				bool enable, s16 thold, u8 hyst);
int wl1271_acx_rssi_snr_avg_weights(struct wl1271 *wl,
				    struct wl12xx_vif *wlvif);
int cc33xx_acx_set_ht_capabilities(struct wl1271 *wl,
				    struct ieee80211_sta_ht_cap *ht_cap,
				    bool allow_ht_operation, u8 hlid);
int wl1271_acx_set_ht_information(struct wl1271 *wl,
				   struct wl12xx_vif *wlvif,
				   u16 ht_operation_mode,
				   u32 he_oper_params, u16 he_oper_nss_set);
int wl12xx_acx_set_ba_initiator_policy(struct wl1271 *wl,
				       struct wl12xx_vif *wlvif);
int wl12xx_acx_set_ba_receiver_session(struct wl1271 *wl, u8 tid_index,
				       u16 ssn, bool enable, u8 peer_hlid,
				       u8 win_size);
int wl12xx_acx_static_calibration_configure(struct wl1271 *wl,
                        u8 file_version,
						u8 payload_struct_version,
					    u8 *data_ptr,
					    bool valid_data);						
int wl12xx_acx_tsf_info(struct wl1271 *wl, struct wl12xx_vif *wlvif,
			u64 *mactime);
int cc33xx_acx_ps_rx_streaming(struct wl1271 *wl, struct wl12xx_vif *wlvif,
			       bool enable);
int wl1271_acx_ap_max_tx_retry(struct wl1271 *wl, struct wl12xx_vif *wlvif);
int cc33xx_acx_config_ps(struct wl1271 *wl, struct wl12xx_vif *wlvif);
int wl1271_acx_set_inconnection_sta(struct wl1271 *wl,
				    struct wl12xx_vif *wlvif, u8 *addr);
int wl12xx_acx_set_rate_mgmt_params(struct wl1271 *wl);
int wl12xx_acx_config_hangover(struct wl1271 *wl);
int wlcore_acx_average_rssi(struct wl1271 *wl, struct wl12xx_vif *wlvif,
			    s8 *avg_rssi);

int cc33xx_acx_default_rx_filter_enable(struct wl1271 *wl, bool enable,
					enum rx_filter_action action);
int cc33xx_acx_set_rx_filter(struct wl1271 *wl, u8 index, bool enable,
			     struct cc33xx_rx_filter *filter);

int wl18xx_acx_dynamic_fw_traces(struct wl1271 *wl);
int wl18xx_acx_clear_statistics(struct wl1271 *wl);

int cc33xx_acx_host_if_cfg_bitmap(struct wl1271 *wl, u32 host_cfg_bitmap,
				  u32 sdio_blk_size, u32 extra_mem_blks,
				  u32 len_field_size);
int cc33xx_acx_set_checksum_state(struct wl1271 *wl);
int cc33xx_acx_peer_ht_operation_mode(struct wl1271 *wl, u8 hlid, bool wide);
int cc33xx_acx_set_peer_cap(struct wl1271 *wl,
			    struct ieee80211_sta_ht_cap *ht_cap,
			    struct ieee80211_sta_he_cap *he_cap,
			    struct wl12xx_vif *wlvif,
			    bool allow_ht_operation,
			    u32 rate_set, u8 hlid);
int cc33xx_acx_interrupt_notify_config(struct wl1271 *wl, bool action);
int cc33xx_acx_rx_ba_filter(struct wl1271 *wl, bool action);
int wl18xx_acx_ap_sleep(struct wl1271 *wl);
int wl18xx_acx_time_sync_cfg(struct wl1271 *wl);
u32 wl18xx_sta_get_ap_rate_mask(struct wl1271 *wl, struct wl12xx_vif *wlvif);


#endif /* __WL1271_ACX_H__ */
