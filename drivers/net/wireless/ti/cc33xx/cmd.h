/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * This file is part of wl1271
 *
 * Copyright (C) 1998-2009 Texas Instruments. All rights reserved.
 * Copyright (C) 2009 Nokia Corporation
 *
 * Contact: Luciano Coelho <luciano.coelho@nokia.com>
 */

#ifndef __CMD_H__
#define __CMD_H__

#include "wlcore.h"

struct acx_header;


int cc33xx_cmd_send(struct wl1271 *wl, u16 id, void *buf, size_t len,
		    size_t res_len);
int cc33xx_cmd_role_enable(struct wl1271 *wl, u8 *addr, u8 role_type,
			   u8 *role_id);
int cc33xx_cmd_role_disable(struct wl1271 *wl, u8 *role_id);
int wl12xx_cmd_role_start_sta(struct wl1271 *wl, struct wl12xx_vif *wlvif);
int wl12xx_cmd_role_stop_sta(struct wl1271 *wl, struct wl12xx_vif *wlvif);
int wl12xx_cmd_role_start_ap(struct wl1271 *wl, struct wl12xx_vif *wlvif);
int wl12xx_cmd_role_stop_ap(struct wl1271 *wl, struct wl12xx_vif *wlvif);
int wl12xx_cmd_role_start_ibss(struct wl1271 *wl, struct wl12xx_vif *wlvif);
int cc33xx_start_dev(struct wl1271 *wl, struct wl12xx_vif *wlvif,
		     enum nl80211_band band, int channel);
int wl12xx_cmd_role_start_dev(struct wl1271 *wl,struct wl12xx_vif *wlvif,
		     enum nl80211_band band, int channel);
int cc33xx_stop_dev(struct wl1271 *wl, struct wl12xx_vif *wlvif);
int wl1271_cmd_test(struct wl1271 *wl, void *buf, size_t buf_len, u8 answer);
int wl1271_cmd_interrogate(struct wl1271 *wl, u16 id, void *buf,
			   size_t cmd_len, size_t res_len);
int wl1271_cmd_debug_inter(struct wl1271 *wl, u16 id, void *buf,
						size_t cmd_len, size_t res_len);
int cc33xx_cmd_configure(struct wl1271 *wl, u16 id, void *buf, size_t len);
int wl1271_cmd_debug(struct wl1271 *wl, u16 id, void *buf, size_t len);
int wlcore_cmd_configure_failsafe(struct wl1271 *wl, u16 id, void *buf,
				  size_t len, unsigned long valid_rets);
int wl1271_cmd_data_path(struct wl1271 *wl, bool enable);
int wl1271_cmd_ps_mode(struct wl1271 *wl, struct wl12xx_vif *wlvif,
		       u8 ps_mode, u16 auto_ps_timeout);
int wl1271_cmd_read_memory(struct wl1271 *wl, u32 addr, void *answer,
			   size_t len);
int wl1271_cmd_template_set(struct wl1271 *wl, u8 role_id,
			    u16 template_id, void *buf, size_t buf_len,
			    int index, u32 rates);
int wl12xx_cmd_build_null_data(struct wl1271 *wl, struct wl12xx_vif *wlvif);
int wl1271_cmd_build_ps_poll(struct wl1271 *wl, struct wl12xx_vif *wlvif,
			     u16 aid);
int wl12xx_cmd_build_probe_req(struct wl1271 *wl, struct wl12xx_vif *wlvif,
			       u8 role_id, u8 band,
			       const u8 *ssid, size_t ssid_len,
			       const u8 *ie, size_t ie_len, const u8 *common_ie,
			       size_t common_ie_len, bool sched_scan);
struct sk_buff *wl1271_cmd_build_ap_probe_req(struct wl1271 *wl,
					      struct wl12xx_vif *wlvif,
					      struct sk_buff *skb);
int wl1271_cmd_build_arp_rsp(struct wl1271 *wl, struct wl12xx_vif *wlvif);
int wl1271_build_qos_null_data(struct wl1271 *wl, struct ieee80211_vif *vif);
int cc33xx_cmd_set_default_wep_key(struct wl1271 *wl, u8 id, u8 hlid);
int wl1271_cmd_set_sta_key(struct wl1271 *wl, struct wl12xx_vif *wlvif,
			   u16 action, u8 id, u8 key_type,
			   u8 key_size, const u8 *key, const u8 *addr,
			   u32 tx_seq_32, u16 tx_seq_16);
int wl1271_cmd_set_ap_key(struct wl1271 *wl, struct wl12xx_vif *wlvif,
			  u16 action, u8 id, u8 key_type,
			  u8 key_size, const u8 *key, u8 hlid, u32 tx_seq_32,
			  u16 tx_seq_16);
int cc33xx_cmd_set_peer_state(struct wl1271 *wl, struct wl12xx_vif *wlvif,
			      u8 hlid);
int cc33xx_roc(struct wl1271 *wl, struct wl12xx_vif *wlvif, u8 role_id,
	       enum nl80211_band band, u8 channel);
int wl12xx_croc(struct wl1271 *wl, u8 role_id);

int wl12xx_cmd_add_peer(struct wl1271 *wl, struct wl12xx_vif *wlvif,
			struct ieee80211_sta *sta, u8 *hlid, u8 is_connected);

int wl12xx_cmd_remove_peer(struct wl1271 *wl, struct wl12xx_vif *wlvif,
			   u8 hlid);
void wlcore_set_pending_regdomain_ch(struct wl1271 *wl, u16 channel,
				     enum nl80211_band band);
int wlcore_cmd_regdomain_config_locked(struct wl1271 *wl);
int wlcore_cmd_generic_cfg(struct wl1271 *wl, struct wl12xx_vif *wlvif,
			   u8 feature, u8 enable, u8 value);
int wl12xx_cmd_config_fwlog(struct wl1271 *wl);
int wl12xx_cmd_start_fwlog(struct wl1271 *wl);
int cc33xx_cmd_stop_fwlog(struct wl1271 *wl);
int wl12xx_cmd_channel_switch(struct wl1271 *wl,
			      struct wl12xx_vif *wlvif,
			      struct ieee80211_channel_switch *ch_switch);
int wl12xx_cmd_stop_channel_switch(struct wl1271 *wl,
				   struct wl12xx_vif *wlvif);

int cc33xx_set_link(struct wl1271 *wl, struct wl12xx_vif *wlvif, u8 link);
void cc33xx_clear_link(struct wl1271 *wl, struct wl12xx_vif *wlvif, u8 *hlid);

u8 wlcore_get_native_channel_type(u8 nl_channel_type);
int cc33xx_cmd_role_start_transceiver(struct wl1271 *wl, u8 role_id);
int cc33xx_cmd_role_stop_transceiver(struct wl1271 *wl);
int cc33xx_cmd_plt_enable(struct wl1271 *wl, u8 role_id);
int cc33xx_cmd_plt_disable(struct wl1271 *wl);

int cmd_channel_switch(struct wl1271 *wl, struct wl12xx_vif *wlvif,
			      struct ieee80211_channel_switch *ch_switch);
int cmd_dfs_master_restart(struct wl1271 *wl, struct wl12xx_vif *wlvif);
int cmd_set_cac(struct wl1271 *wl, struct wl12xx_vif *wlvif, bool start);

int cmd_smart_config_start(struct wl1271 *wl, u32 group_bitmap);
int cmd_smart_config_stop(struct wl1271 *wl);
int cmd_smart_config_set_group_key(struct wl1271 *wl, u16 group_id, u8 key_len, u8 *key);
int cmd_get_device_info(struct wl1271 *wl, u8 *info_buffer, size_t buffer_len);
int cmd_download_container_chunk(struct wl1271 *wl, u8 *chunk, size_t chunk_len, bool is_last_chunk);

enum cc33xx_cmd
{
    CMD_EMPTY,
    CMD_SET_KEYS                        = 1,
    CMD_SET_LINK_CONNECTION_STATE       = 2,

    CMD_CHANNEL_SWITCH                  = 3,
    CMD_STOP_CHANNEL_SWICTH             = 4,

    CMD_REMAIN_ON_CHANNEL               = 5,
    CMD_CANCEL_REMAIN_ON_CHANNEL        = 6,

    CMD_START_DHCP_MGMT_SEQ             = 7,
    CMD_STOP_DHCP_MGMT_SEQ              = 8,

    CMD_START_SECURITY_MGMT_SEQ         = 9,
    CMD_STOP_SECURITY_MGMT_SEQ          = 10,

    CMD_START_ARP_MGMT_SEQ              = 11,
    CMD_STOP_ARP_MGMT_SEQ               = 12,

    CMD_START_DNS_MGMT_SEQ              = 13,
    CMD_STOP_DNS_MGMT_SEQ               = 14,

    /* Access point commands */
    CMD_ADD_PEER                        = 15,
    CMD_REMOVE_PEER                     = 16,

    /* Role API */
    CMD_ROLE_ENABLE                     = 17,
    CMD_ROLE_DISABLE                    = 18,
    CMD_ROLE_START                      = 19,
    CMD_ROLE_STOP                       = 20,

    CMD_AP_SET_BEACON_INFO              = 21,              /* Set AP beacon template */

    // Managed sequence of sending deauth / disassoc frame
    CMD_SEND_DEAUTH_DISASSOC            = 22,

    CMD_SCHED_STATE_EVENT               = 23,
    CMD_SCAN                            = 24,
    CMD_STOP_SCAN                       = 25,
    CMD_SET_PROBE_IE                    = 26,


    CMD_CONFIGURE                       = 27,
    CMD_INTERROGATE                     = 28,

	CMD_DEBUG                           = 29,
	CMD_DEBUG_READ						= 30,

	CMD_TEST_MODE                       = 31,
	CMD_PLT_ENABLE                      = 32,
	CMD_PLT_DISABLE                     = 33,
	CMD_CONNECTION_SCAN_SSID_CFG        = 34,
	CMD_BM_READ_DEVICE_INFO             = 35,
	CMD_CONTAINER_DOWNLOAD              = 36,
	CMD_LAST_COMMAND,                   // must be the last

    MAX_COMMAND_ID_OSPREY = 0x7FFF,
};

//TODO: RazB - Need to remove this enum when we finish over all the commands
enum wl1271_commands {

	CMD_ENABLE_RX	= CMD_LAST_COMMAND,
	CMD_ENABLE_TX	,
	CMD_DISABLE_RX	,
	CMD_DISABLE_TX	,

	CMD_READ_MEMORY ,
	CMD_WRITE_MEMORY,
	CMD_SET_TEMPLATE,
	CMD_TEST		,
	CMD_NOISE_HIST	,
	CMD_QUIET_ELEMENT_SET_STATE ,
	CMD_SET_BCN_MODE ,

	CMD_MEASUREMENT	,
	CMD_STOP_MEASUREMENT,
	CMD_SET_PS_MODE		,

	CMD_AP_DISCOVERY	,
	CMD_STOP_AP_DISCOVERY	,
	CMD_HEALTH_CHECK	,
	//CMD_DEBUG		,
	CMD_TRIGGER_SCAN_TO	,
	CMD_CONNECTION_SCAN_CFG	, 
	CMD_START_PERIODIC_SCAN	,
	CMD_STOP_PERIODIC_SCAN	,

	CMD_CONFIG_FWLOGGER		,
	CMD_START_FWLOGGER		,
	CMD_STOP_FWLOGGER		,

	/* DFS */
	CMD_START_RADAR_DETECTION	,
	CMD_STOP_RADAR_DETECTION	,

	/* WIFI Direct */
	CMD_WFD_START_DISCOVERY	,
	CMD_WFD_STOP_DISCOVERY	,
	CMD_WFD_ATTRIBUTE_CONFIG,
	CMD_GENERIC_CFG			,
	CMD_NOP				,

	/* start of 18xx specific commands */
	CMD_DFS_CHANNEL_CONFIG		,
	CMD_SMART_CONFIG_START		,
	CMD_SMART_CONFIG_STOP		,
	CMD_SMART_CONFIG_SET_GROUP_KEY	,

	CMD_CAC_START			,
	CMD_CAC_STOP			,
	CMD_DFS_MASTER_RESTART	,
	CMD_DFS_RADAR_DETECTION_DEBUG	,

	MAX_COMMAND_ID = 0xFFFF,
};

#define MAX_CMD_PARAMS 572

enum cmd_templ {
	CMD_TEMPL_NULL_DATA = 0,
	CMD_TEMPL_BEACON,
	CMD_TEMPL_CFG_PROBE_REQ_2_4,
	CMD_TEMPL_CFG_PROBE_REQ_5,
	CMD_TEMPL_PROBE_RESPONSE,
	CMD_TEMPL_QOS_NULL_DATA,
	CMD_TEMPL_PS_POLL,
	CMD_TEMPL_DISCONNECT,
	CMD_TEMPL_APP_PROBE_REQ_2_4_LEGACY,
	CMD_TEMPL_APP_PROBE_REQ_5_LEGACY,
	CMD_TEMPL_BAR,           /* for firmware internal use only */
	CMD_TEMPL_CTS,           /*
				  * For CTS-to-self (FastCTS) mechanism
				  * for BT/WLAN coexistence (SoftGemini). */
	CMD_TEMPL_AP_BEACON,
	CMD_TEMPL_AP_PROBE_RESPONSE,
	CMD_TEMPL_ARP_RSP,
	CMD_TEMPL_DEAUTH_AP,
	CMD_TEMPL_TEMPORARY,
	CMD_TEMPL_LINK_MEASUREMENT_REPORT,
	CMD_TEMPL_PROBE_REQ_2_4_PERIODIC,
	CMD_TEMPL_PROBE_REQ_5_PERIODIC,

	CMD_TEMPL_MAX = 0xff
};

/* unit ms */
#define WL1271_COMMAND_TIMEOUT     2000
#define WL1271_CMD_TEMPL_DFLT_SIZE 252
#define WL1271_CMD_TEMPL_MAX_SIZE  512
#define WL1271_EVENT_TIMEOUT       5000

struct cc33xx_cmd_header {

	struct NAB_header NAB_header;
	__le16 id;
	__le16 status;

	/* payload */
	u8 data[0];
} __packed;

#define WL1271_CMD_MAX_PARAMS 572

struct wl1271_command {
	struct cc33xx_cmd_header header;
	u8  parameters[WL1271_CMD_MAX_PARAMS];
} __packed;

enum {
	CMD_MAILBOX_IDLE		=  0,
	CMD_STATUS_SUCCESS		=  1,
	CMD_STATUS_UNKNOWN_CMD		=  2,
	CMD_STATUS_UNKNOWN_IE		=  3,
	CMD_STATUS_REJECT_MEAS_SG_ACTIVE	= 11,
	CMD_STATUS_RX_BUSY		= 13,
	CMD_STATUS_INVALID_PARAM		= 14,
	CMD_STATUS_TEMPLATE_TOO_LARGE		= 15,
	CMD_STATUS_OUT_OF_MEMORY		= 16,
	CMD_STATUS_STA_TABLE_FULL		= 17,
	CMD_STATUS_RADIO_ERROR		= 18,
	CMD_STATUS_WRONG_NESTING		= 19,
	CMD_STATUS_TIMEOUT		= 21, /* Driver internal use.*/
	CMD_STATUS_FW_RESET		= 22, /* Driver internal use.*/
	CMD_STATUS_TEMPLATE_OOM		= 23,
	CMD_STATUS_NO_RX_BA_SESSION	= 24,

	MAX_COMMAND_STATUS
};

#define CMDMBOX_HEADER_LEN 4
#define CMDMBOX_INFO_ELEM_HEADER_LEN 4

enum {
	BSS_TYPE_IBSS = 0,
	BSS_TYPE_STA_BSS = 2,
	BSS_TYPE_AP_BSS = 3,
	MAX_BSS_TYPE = 0xFF
};

#define WL1271_JOIN_CMD_CTRL_TX_FLUSH     0x80 /* Firmware flushes all Tx */
#define WL1271_JOIN_CMD_TX_SESSION_OFFSET 1
#define WL1271_JOIN_CMD_BSS_TYPE_5GHZ 0x10

struct cc33xx_cmd_role_enable {
	struct cc33xx_cmd_header header;

	u8 role_type;
	u8 mac_address[ETH_ALEN];
	u8 reserved;
} __packed;



struct command_complete_header {
	__le16 id;
	__le16 status;

	/* payload */
	u8 data[0];
} __packed;



struct cc33xx_cmd_complete_role_enable {
    struct command_complete_header header;
    u8 role_id;
    u8 padding[3];
} __packed;


struct cc33xx_cmd_role_disable {
	struct cc33xx_cmd_header header;

	u8 role_id;
	u8 padding[3];
} __packed;

enum wlcore_band {
	WLCORE_BAND_2_4GHZ		= 0,
	WLCORE_BAND_5GHZ		= 1,
	WLCORE_BAND_JAPAN_4_9_GHZ	= 2,
	WLCORE_BAND_DEFAULT		= WLCORE_BAND_2_4GHZ,
	WLCORE_BAND_INVALID		= 0x7E,
	WLCORE_BAND_MAX_RADIO		= 0x7F,
};

enum wlcore_channel_type {
	WLCORE_CHAN_NO_HT,
	WLCORE_CHAN_HT20,
	WLCORE_CHAN_HT40MINUS,
	WLCORE_CHAN_HT40PLUS
};

struct cc33xx_cmd_role_start {
	struct cc33xx_cmd_header header;
	u8 role_id;
	u8 role_type;
	u8 band;
	u8 channel;

	/* enum wlcore_channel_type */
	u8 channel_type;

	union {
		struct {
			u8 padding_1[54];
		} __packed device;
		/* sta & p2p_cli use the same struct */
		struct {
			u8 bssid[ETH_ALEN];

			__le32 remote_rates; /* remote supported rates */

			/*
			 * The target uses this field to determine the rate at
			 * which to transmit control frame responses (such as
			 * ACK or CTS frames).
			 */
			__le32 basic_rate_set;
			__le32 local_rates; /* local supported rates */

			u8 ssid_type;
			u8 ssid_len;
			u8 ssid[IEEE80211_MAX_SSID_LEN];

			__le16 beacon_interval; /* in TBTTs */
		} __packed sta;
		struct {
			u8 bssid[ETH_ALEN];
			u8 hlid; /* data hlid */
			u8 dtim_interval;
			__le32 remote_rates; /* remote supported rates */

			__le32 basic_rate_set;
			__le32 local_rates; /* local supported rates */

			u8 ssid_type;
			u8 ssid_len;
			u8 ssid[IEEE80211_MAX_SSID_LEN];

			__le16 beacon_interval; /* in TBTTs */

			u8 padding_1[2];
		} __packed ibss;
		/* ap & p2p_go use the same struct */
		struct {
			__le16 beacon_interval; /* in TBTTs */

			__le32 basic_rate_set;
			__le32 local_rates; /* local supported rates */

			u8 dtim_interval;
			/*
			 * ap supports wmm (note that there is additional
			 * per-sta wmm configuration)
			 */
			u8 wmm;
			u8 padding_1[42];
		} __packed ap;
	};
	u8 padding;
} __packed;

struct cc33xx_cmd_complete_role_start {
    struct command_complete_header header;
    union {
        struct {
            u8 hlid;
            u8 session;
			u8 padding[2];
        } __packed sta;
        struct {
            /* The host link id for the AP's global queue */
            u8 global_hlid;
            /* The host link id for the AP's broadcast queue */
            u8 broadcast_hlid;
            u8 bcast_session_id;
            u8 global_session_id;
        } __packed ap;
    };
} __packed;
struct cc33xx_cmd_role_stop {
	struct cc33xx_cmd_header header;

	u8 role_id;
	u8 padding[3];

} __packed;

struct cmd_enabledisable_path {
	struct cc33xx_cmd_header header;

	u8 channel;
	u8 padding[3];
} __packed;

#define WL1271_RATE_AUTOMATIC  0

struct wl1271_cmd_template_set {
	struct cc33xx_cmd_header header;

	u8 role_id;
	u8 template_type;
	__le16 len;
	u8 index;  /* relevant only for KLV_TEMPLATE type */
	u8 padding[3];

	__le32 enabled_rates;
	u8 short_retry_limit;
	u8 long_retry_limit;
	u8 aflags;
	u8 reserved;

	u8 template_data[WL1271_CMD_TEMPL_MAX_SIZE];
} __packed;

#define TIM_ELE_ID    5
#define PARTIAL_VBM_MAX    251

struct wl1271_tim {
	u8 identity;
	u8 length;
	u8 dtim_count;
	u8 dtim_period;
	u8 bitmap_ctrl;
	u8 pvb_field[PARTIAL_VBM_MAX]; /* Partial Virtual Bitmap */
} __packed;

enum wl1271_cmd_ps_mode {
	STATION_AUTO_PS_MODE,   /* Dynamic Power Save */
	STATION_ACTIVE_MODE,
	STATION_POWER_SAVE_MODE
};

struct wl1271_cmd_ps_params {
	struct cc33xx_cmd_header header;

	u8 role_id;
	u8 ps_mode; /* STATION_* */
	u16 auto_ps_timeout;
} __packed;

/* HW encryption keys */
#define NUM_ACCESS_CATEGORIES_COPY 4

enum wl1271_cmd_key_action {
	KEY_ADD_OR_REPLACE = 1,
	KEY_REMOVE         = 2,
	KEY_SET_ID         = 3,
	MAX_KEY_ACTION     = 0xffff,
};

enum wl1271_cmd_lid_key_type {
	UNICAST_LID_TYPE     = 0,
	BROADCAST_LID_TYPE   = 1,
	WEP_DEFAULT_LID_TYPE = 2
};

enum wl1271_cmd_key_type {
	KEY_NONE = 0,
	KEY_WEP  = 1,
	KEY_TKIP = 2,
	KEY_AES  = 3,
	KEY_GEM  = 4,
	KEY_IGTK = 5,
};

struct wl1271_cmd_set_keys {
	struct cc33xx_cmd_header header;

	/*
	 * Indicates whether the HLID is a unicast key set
	 * or broadcast key set. A special value 0xFF is
	 * used to indicate that the HLID is on WEP-default
	 * (multi-hlids). of type wl1271_cmd_lid_key_type.
	 */
	u8 hlid;

	/*
	 * In WEP-default network (hlid == 0xFF) used to
	 * indicate which network STA/IBSS/AP role should be
	 * changed
	 */
	u8 lid_key_type;

	/*
	 * Key ID - For TKIP and AES key types, this field
	 * indicates the value that should be inserted into
	 * the KeyID field of frames transmitted using this
	 * key entry. For broadcast keys the index use as a
	 * marker for TX/RX key.
	 * For WEP default network (HLID=0xFF), this field
	 * indicates the ID of the key to add or remove.
	 */
	u8 key_id;
	u8 reserved_1;

	/* key_action_e */
	__le16 key_action;

	/* key size in bytes */
	u8 key_size;

	/* key_type_e */
	u8 key_type;

	/* This field holds the security key data to add to the STA table */
	u8 key[MAX_KEY_SIZE];
	__le16 ac_seq_num16[NUM_ACCESS_CATEGORIES_COPY];
	__le32 ac_seq_num32[NUM_ACCESS_CATEGORIES_COPY];
} __packed;

struct wl1271_cmd_test_header {
	u8 id;
	u8 padding[3];
} __packed;

enum wl1271_channel_tune_bands {
	WL1271_CHANNEL_TUNE_BAND_2_4,
	WL1271_CHANNEL_TUNE_BAND_5,
	WL1271_CHANNEL_TUNE_BAND_4_9
};

#define WL1271_PD_REFERENCE_POINT_BAND_B_G  0

/*
 * There are three types of disconnections:
 *
 * DISCONNECT_IMMEDIATE: the fw doesn't send any frames
 * DISCONNECT_DEAUTH:    the fw generates a DEAUTH request with the reason
 *                       we have passed
 * DISCONNECT_DISASSOC:  the fw generates a DESASSOC request with the reason
 *                       we have passed
 */
enum wl1271_disconnect_type {
	DISCONNECT_IMMEDIATE,
	DISCONNECT_DEAUTH,
	DISCONNECT_DISASSOC
};

#define WL1271_CMD_STA_STATE_CONNECTED  1

struct wl12xx_cmd_set_peer_state {
	struct cc33xx_cmd_header header;

	u8 hlid;
	u8 state;
	u8 padding[2];
} __packed;

struct cc33xx_cmd_roc {
	struct cc33xx_cmd_header header;

	u8 role_id;
	u8 channel;
	u8 band;
	u8 padding;
};

struct wl12xx_cmd_croc {
	struct cc33xx_cmd_header header;

	u8 role_id;
	u8 padding[3];
};

enum wl12xx_ssid_type {
	WL12XX_SSID_TYPE_PUBLIC = 0,
	WL12XX_SSID_TYPE_HIDDEN = 1,
	WL12XX_SSID_TYPE_ANY = 2,
};

enum wl1271_psd_type {
	WL1271_PSD_LEGACY = 0,
	WL1271_PSD_UPSD_TRIGGER = 1,
	WL1271_PSD_LEGACY_PSPOLL = 2,
	WL1271_PSD_SAPSD = 3
};
#define MAX_SIZE_BEACON_TEMP    (450)
struct wl12xx_cmd_set_beacon_info
{
	struct cc33xx_cmd_header header;
	
    u8    	role_id;
    u16     beacon_len;
    u8   	beacon[MAX_SIZE_BEACON_TEMP];
	u8		padding[3];
} __packed;

struct wl12xx_cmd_add_peer {
	struct cc33xx_cmd_header header;

	u8 is_connected;
	u8 role_id;
	u8 role_type;
	u8 link_type;
	u8 addr[ETH_ALEN];
	u8 aid;
	u8 psd_type[NUM_ACCESS_CATEGORIES_COPY];
	__le32 supported_rates;
	u8 bss_index;
	u8 sp_len;
	u8 wmm;
    u32 ht_capabilities;
    u8  ampdu_params;

    /* HE peer support */
    bool has_he;
	bool mfp;
	u8 padding[3];
} __packed;

struct cc33xx_cmd_complete_add_peer {
	struct command_complete_header header;
	u8 hlid;
	u8 session_id;
} __packed;

struct wl12xx_cmd_remove_peer {
	struct cc33xx_cmd_header header;
	u8 hlid;
	u8 role_id;
	u8 padding[2];
	
} __packed;

/*
 * Continuous mode - packets are transferred to the host periodically
 * via the data path.
 * On demand - Log messages are stored in a cyclic buffer in the
 * firmware, and only transferred to the host when explicitly requested
 */
enum wl12xx_fwlogger_log_mode {
	CC33XX_FWLOG_CONTINUOUS,
};

/* Include/exclude timestamps from the log messages */
enum cc33xx_fwlogger_timestamp {
	CC33XX_FWLOG_TIMESTAMP_DISABLED,
	CC33XX_FWLOG_TIMESTAMP_ENABLED
};

/*
 * Logs can be routed to the debug pinouts (where available), to the host bus
 * (SDIO/SPI), or dropped
 */
enum cc33xx_fwlogger_output {
	CC33XX_FWLOG_OUTPUT_NONE,
	CC33XX_FWLOG_OUTPUT_DBG_PINS,
	CC33XX_FWLOG_OUTPUT_HOST,
};

struct wl12xx_cmd_regdomain_dfs_config {
	struct cc33xx_cmd_header header;

	__le32 ch_bit_map1;
	__le32 ch_bit_map2;
	u8 dfs_region;
	u8 padding[3];
} __packed;

enum wlcore_generic_cfg_feature {
	WLCORE_CFG_FEATURE_RADAR_DEBUG = 2,
};

struct wlcore_cmd_generic_cfg {
	struct cc33xx_cmd_header header;

	u8 role_id;
	u8 feature;
	u8 enable;
	u8 value;
} __packed;

struct wl12xx_cmd_config_fwlog {
	struct cc33xx_cmd_header header;

	/* See enum wl12xx_fwlogger_log_mode */
	u8 logger_mode;

	/* Minimum log level threshold */
	u8 log_severity;

	/* Include/exclude timestamps from the log messages */
	u8 timestamp;

	/* See enum wl1271_fwlogger_output */
	u8 output;

	/* Regulates the frequency of log messages */
	u8 threshold;

	u8 padding[3];
} __packed;

struct wl12xx_cmd_start_fwlog {
	struct cc33xx_cmd_header header;
} __packed;

struct wl12xx_cmd_stop_fwlog {
	struct cc33xx_cmd_header header;
} __packed;

struct wl12xx_cmd_stop_channel_switch {
	struct cc33xx_cmd_header header;

	u8 role_id;
	u8 padding[3];
} __packed;

/* Used to check radio status after calibration */
#define MAX_TLV_LENGTH		500
#define TEST_CMD_P2G_CAL	2	/* TX BiP */

struct wl1271_cmd_cal_p2g {
	struct cc33xx_cmd_header header;

	struct wl1271_cmd_test_header test;

	__le32 ver;
	__le16 len;
	u8 buf[MAX_TLV_LENGTH];
	u8 type;
	u8 padding;

	__le16 radio_status;

	u8 sub_band_mask;
	u8 padding2;
} __packed;

struct cmd_channel_switch {
	struct cc33xx_cmd_header header;

	u8 role_id;

	/* The new serving channel */
	u8 channel;
	/* Relative time of the serving channel switch in TBTT units */
	u8 switch_time;
	/* Stop the role TX, should expect it after radar detection */
	u8 stop_tx;

	__le32 local_supported_rates;

	u8 channel_type;
	u8 band;

	u8 padding[2];
} __packed;

struct cmd_smart_config_start {
	struct cc33xx_cmd_header header;

	__le32 group_id_bitmask;
} __packed;

struct cmd_smart_config_set_group_key {
	struct cc33xx_cmd_header header;

	__le32 group_id;

	u8 key[16];
} __packed;

struct cmd_dfs_master_restart {
	struct cc33xx_cmd_header header;

	u8 role_id;
	u8 padding[3];
} __packed;

/* cac_start and cac_stop share the same params */
struct cmd_cac_start {
	struct cc33xx_cmd_header header;

	u8 role_id;
	u8 channel;
	u8 band;
	u8 bandwidth;
} __packed;


/* PLT structs */


struct cc33xx_cmd_PLT_enable
{
	struct cc33xx_cmd_header header;
	u32 dummy;
};

struct cc33xx_cmd_PLT_disable
{
	struct cc33xx_cmd_header header;
	u32 dummy;
};

struct cc33xx_cmd_container_download {
	struct cc33xx_cmd_header header;
	u32 length;
	u8 payload[0];
} __packed;

struct cc33xx_cmd_get_device_info {
	struct cc33xx_cmd_header header;
	u8 device_info[700];
} __packed;

#endif /* __WL1271_CMD_H__ */
