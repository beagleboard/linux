/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * This file is part of cc33xx
 *
 * Copyright (C) 1998-2009 Texas Instruments. All rights reserved.
 * Copyright (C) 2008-2009 Nokia Corporation
 *
 * Contact: Luciano Coelho <luciano.coelho@nokia.com>
 */

#ifndef __WLCORE_I_H__
#define __WLCORE_I_H__

#include <linux/mutex.h>
#include <linux/completion.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/bitops.h>
#include <net/mac80211.h>

#include "conf.h"
#include "ini.h"

struct cc33xx_family_data {
	const char *name;
	const char *nvs_name;	/* nvs file */
	const char *cfg_name;	/* cfg file */
};


#define CC33XX_TX_SECURITY_LO16(s) ((u16)((s) & 0xffff))
#define CC33XX_TX_SECURITY_HI32(s) ((u32)(((s) >> 16) & 0xffffffff))
#define CC33XX_TX_SQN_POST_RECOVERY_PADDING 0xff
/* Use smaller padding for GEM, as some  APs have issues when it's too big */
#define CC33XX_TX_SQN_POST_RECOVERY_PADDING_GEM 0x20


#define CC33XX_CIPHER_SUITE_GEM 0x00147201

#define CC33XX_BUSY_WORD_LEN (sizeof(u32))

#define CC33XX_ELP_HW_STATE_ASLEEP 0
#define CC33XX_ELP_HW_STATE_IRQ    1

#define CC33XX_DEFAULT_BEACON_INT  100
#define CC33XX_DEFAULT_DTIM_PERIOD 1

#define CC33XX_MAX_ROLES           4
#define CC33XX_INVALID_ROLE_ID     0xff
#define CC33XX_INVALID_LINK_ID     0xff

#define CC33XX_MAX_LINKS 21


/* the driver supports the 2.4Ghz and 5Ghz bands */
#define WLCORE_NUM_BANDS           2

#define CC33XX_MAX_RATE_POLICIES 16

/* Defined by FW as 0. Will not be freed or allocated. */
#define CC33XX_SYSTEM_HLID         0

/*
 * When in AP-mode, we allow (at least) this number of packets
 * to be transmitted to FW for a STA in PS-mode. Only when packets are
 * present in the FW buffers it will wake the sleeping STA. We want to put
 * enough packets for the driver to transmit all of its buffered data before
 * the STA goes to sleep again. But we don't want to take too much memory
 * as it might hurt the throughput of active STAs.
 */
#define CC33XX_PS_STA_MAX_PACKETS  2

#define CC33XX_AP_BSS_INDEX        0

enum wlcore_state {
	WLCORE_STATE_OFF,
	WLCORE_STATE_RESTARTING,
	WLCORE_STATE_ON,
};

struct cc33xx;

enum {
	FW_VER_CHIP,
	FW_VER_IF_TYPE,
	FW_VER_MAJOR,
	FW_VER_SUBTYPE,
	FW_VER_MINOR,

	NUM_FW_VER
};

#define NUM_TX_QUEUES              4

#define CC33XX_MAX_CHANNELS 64
struct cc33xx_scan {
	struct cfg80211_scan_request *req;
	unsigned long scanned_ch[BITS_TO_LONGS(CC33XX_MAX_CHANNELS)];
	bool failed;
	u8 state;
	u8 ssid[IEEE80211_MAX_SSID_LEN+1];
	size_t ssid_len;
};

struct cc33xx_if_operations {
	void (*interface_claim)(struct device *child);
	void (*interface_release)(struct device *child);
	int __must_check (*read)(struct device *child, int addr, void *buf,
				 size_t len, bool fixed);
	int __must_check (*write)(struct device *child, int addr, void *buf,
				  size_t len, bool fixed);
	void (*reset)(struct device *child);
	void (*init)(struct device *child);
	int (*power)(struct device *child, bool enable);
	void (*set_block_size) (struct device *child, unsigned int blksz);
	size_t (*get_max_transaction_len) (struct device *child);
	void (*set_irq_handler) (struct device *child, void* irq_handler);
	void (*enable_irq) (struct device *child);
	void (*disable_irq) (struct device *child);
};

struct wlcore_platdev_data {
	struct cc33xx_if_operations *if_ops;
	const struct cc33xx_family_data *family;
	void (*irq_handler)(struct platform_device *pdev);
	int  gpio_irq_num;

	bool ref_clock_xtal;	/* specify whether the clock is XTAL or not */
	u32 ref_clock_freq;	/* in Hertz */
	u32 tcxo_clock_freq;	/* in Hertz, tcxo is always XTAL */
	bool pwr_in_suspend;
};

#define MAX_NUM_KEYS 14
#define MAX_KEY_SIZE 32

struct cc33xx_ap_key {
	u8 id;
	u8 key_type;
	u8 key_size;
	u8 key[MAX_KEY_SIZE];
	u8 hlid;
	u32 tx_seq_32;
	u16 tx_seq_16;
};

enum cc33xx_flags {
	CC33XX_FLAG_GPIO_POWER,
	CC33XX_FLAG_TX_QUEUE_STOPPED,
	CC33XX_FLAG_TX_PENDING,
	CC33XX_FLAG_IN_ELP,
	CC33XX_FLAG_IRQ_RUNNING,
	CC33XX_FLAG_FW_TX_BUSY,
	CC33XX_FLAG_DUMMY_PACKET_PENDING,
	CC33XX_FLAG_SUSPENDED,
	CC33XX_FLAG_PENDING_WORK,
	CC33XX_FLAG_SOFT_GEMINI,
	CC33XX_FLAG_DRIVER_REMOVED,
	CC33XX_FLAG_RECOVERY_IN_PROGRESS,
	CC33XX_FLAG_VIF_CHANGE_IN_PROGRESS,
	CC33XX_FLAG_IO_FAILED,
	CC33XX_FLAG_REINIT_TX_WDOG,
};

enum cc33xx_vif_flags {
	WLVIF_FLAG_INITIALIZED,
	WLVIF_FLAG_STA_ASSOCIATED,
	WLVIF_FLAG_STA_AUTHORIZED,
	WLVIF_FLAG_IBSS_JOINED,
	WLVIF_FLAG_AP_STARTED,
	WLVIF_FLAG_IN_PS,
	WLVIF_FLAG_STA_STATE_SENT,
	WLVIF_FLAG_RX_STREAMING_STARTED,
	WLVIF_FLAG_PSPOLL_FAILURE,
	WLVIF_FLAG_CS_PROGRESS,
	WLVIF_FLAG_AP_PROBE_RESP_SET,
	WLVIF_FLAG_IN_USE,
	WLVIF_FLAG_ACTIVE,
	WLVIF_FLAG_BEACON_DISABLED,
};

struct cc33xx_vif;

struct cc33xx_link {
	/* AP-mode - TX queue per AC in link */
	struct sk_buff_head tx_queue[NUM_TX_QUEUES];

	/* accounting for allocated / freed packets in FW */
	u8 allocated_pkts;
	u8 prev_freed_pkts;

	u8 addr[ETH_ALEN];

	/* bitmap of TIDs where RX BA sessions are active for this link */
	u8 ba_bitmap;

	/* the last fw rate index we used for this link */
	u8 fw_rate_idx;

	/* the last fw rate [Mbps] we used for this link */
	u8 fw_rate_mbps;

	/* The wlvif this link belongs to. Might be null for global links */
	struct cc33xx_vif *wlvif;

	/*
	 * total freed FW packets on the link - used for tracking the
	 * AES/TKIP PN across recoveries. Re-initialized each time
	 * from the cc33xx_station structure.
	 */
	u64 total_freed_pkts;
};

#define CC33XX_MAX_RX_FILTERS 5
#define CC33XX_RX_FILTER_MAX_FIELDS 8

#define CC33XX_RX_FILTER_ETH_HEADER_SIZE 14
#define CC33XX_RX_FILTER_MAX_FIELDS_SIZE 95
#define RX_FILTER_FIELD_OVERHEAD				\
	(sizeof(struct cc33xx_rx_filter_field) - sizeof(u8 *))
#define CC33XX_RX_FILTER_MAX_PATTERN_SIZE			\
	(CC33XX_RX_FILTER_MAX_FIELDS_SIZE - RX_FILTER_FIELD_OVERHEAD)

#define CC33XX_RX_FILTER_FLAG_MASK                BIT(0)
#define CC33XX_RX_FILTER_FLAG_IP_HEADER           0
#define CC33XX_RX_FILTER_FLAG_ETHERNET_HEADER     BIT(1)

enum rx_filter_action {
	FILTER_DROP = 0,
	FILTER_SIGNAL = 1,
	FILTER_FW_HANDLE = 2
};

enum plt_mode {
	PLT_OFF = 0,
	PLT_ON = 1,
	PLT_FEM_DETECT = 2,
	PLT_CHIP_AWAKE = 3
};

struct cc33xx_rx_filter_field {
	__le16 offset;
	u8 len;
	u8 flags;
	u8 *pattern;
} __packed;

struct cc33xx_rx_filter {
	u8 action;
	int num_fields;
	struct cc33xx_rx_filter_field fields[CC33XX_RX_FILTER_MAX_FIELDS];
};

struct cc33xx_station {
	u8 hlid;
	bool in_connection;

	/*
	 * total freed FW packets on the link to the STA - used for tracking the
	 * AES/TKIP PN across recoveries. Re-initialized each time from the
	 * cc33xx_station structure.
	 * Used in both AP and STA mode.
	 */
	u64 total_freed_pkts;
};

struct cc33xx_vif {
	struct cc33xx *wl;
	struct list_head list;
	unsigned long flags;
	u8 bss_type;
	u8 p2p; /* we are using p2p role */
	u8 role_id;

	/* sta/ibss specific */
	u8 dev_role_id;
	u8 dev_hlid;

	union {
		struct {
			u8 hlid;

			u8 basic_rate_idx;
			u8 ap_rate_idx;
			u8 p2p_rate_idx;

			bool qos;
			/* channel type we started the STA role with */
			enum nl80211_channel_type role_chan_type;
		} sta;
		struct {
			u8 global_hlid;
			u8 bcast_hlid;

			/* HLIDs bitmap of associated stations */
			unsigned long sta_hlid_map[BITS_TO_LONGS(
							CC33XX_MAX_LINKS)];

			/* recoreded keys - set here before AP startup */
			struct cc33xx_ap_key *recorded_keys[MAX_NUM_KEYS];

			u8 mgmt_rate_idx;
			u8 bcast_rate_idx;
			u8 ucast_rate_idx[CONF_TX_MAX_AC_COUNT];
		} ap;
	};

	/* the hlid of the last transmitted skb */
	int last_tx_hlid;

	/* counters of packets per AC, across all links in the vif */
	int tx_queue_count[NUM_TX_QUEUES];

	unsigned long links_map[BITS_TO_LONGS(CC33XX_MAX_LINKS)];

	u8 ssid[IEEE80211_MAX_SSID_LEN + 1];
	u8 ssid_len;

	/* The current band */
	enum nl80211_band band;
	int channel;
	enum nl80211_channel_type channel_type;

	u32 bitrate_masks[WLCORE_NUM_BANDS];
	u32 basic_rate_set;

	/*
	 * currently configured rate set:
	 *	bits  0-15 - 802.11abg rates
	 *	bits 16-23 - 802.11n   MCS index mask
	 * support only 1 stream, thus only 8 bits for the MCS rates (0-7).
	 */
	u32 basic_rate;
	u32 rate_set;

	/* probe-req template for the current AP */
	struct sk_buff *probereq;

	/* Beaconing interval (needed for ad-hoc) */
	u32 beacon_int;

	/* Default key (for WEP) */
	u32 default_key;

	/* Our association ID */
	u16 aid;

	/* retry counter for PSM entries */
	u8 psm_entry_retry;

	/* in dBm */
	int power_level;

	int rssi_thold;
	int last_rssi_event;

	/* save the current encryption type for auto-arp config */
	u8 encryption_type;
	__be32 ip_addr;

	/* RX BA constraint value */
	bool ba_support;
	bool ba_allowed;

	bool wmm_enabled;

	bool radar_enabled;

	/* Rx Streaming */
	struct work_struct rx_streaming_enable_work;
	struct work_struct rx_streaming_disable_work;
	struct timer_list rx_streaming_timer;

	struct delayed_work channel_switch_work;
	struct delayed_work connection_loss_work;

	/* number of in connection stations */
	int inconn_count;

	/*
	 * This vif's queues are mapped to mac80211 HW queues as:
	 * VO - hw_queue_base
	 * VI - hw_queue_base + 1
	 * BE - hw_queue_base + 2
	 * BK - hw_queue_base + 3
	 */
	int hw_queue_base;

	/* do we have a pending auth reply? (and ROC) */
	bool ap_pending_auth_reply;

	/* time when we sent the pending auth reply */
	unsigned long pending_auth_reply_time;

	/* work for canceling ROC after pending auth reply */
	struct delayed_work pending_auth_complete_work;

	struct delayed_work roc_timeout_work;

	/* update rate conrol */
	enum ieee80211_sta_rx_bandwidth rc_update_bw;
	struct ieee80211_sta_ht_cap rc_ht_cap;
	struct work_struct rc_update_work;

	/*
	 * total freed FW packets on the link.
	 * For STA this holds the PN of the link to the AP.
	 * For AP this holds the PN of the broadcast link.
	 */
	u64 total_freed_pkts;

	/* for MBSSID: this BSS is a nontransmitted BSS profile */ // Katya
	/* Relevant for STA role */
	/* Consider to add under .sta*/
	bool nontransmitted;

	/* for MBSSID: update transmitter BSSID */
	u8 transmitter_bssid[ETH_ALEN];

	/* for MBSSID: BSSID index */
	u8 bssid_index;

	/* for MBSSID: BSSID indicator */
	u8 bssid_indicator;

	/* for STA: if connection established and has HE support*/
	u8 sta_has_he;

	/*
	 * This struct must be last!
	 * data that has to be saved acrossed reconfigs (e.g. recovery)
	 * should be declared in this struct.
	 */
	struct {
		u8 persistent[0];
	};
};


static inline struct cc33xx_vif *cc33xx_vif_to_data(struct ieee80211_vif *vif)
{
	WARN_ON(!vif);
	return (struct cc33xx_vif *)vif->drv_priv;
}

static inline
struct ieee80211_vif *cc33xx_wlvif_to_vif(struct cc33xx_vif *wlvif)
{
	return container_of((void *)wlvif, struct ieee80211_vif, drv_priv);
}

static inline bool wlcore_is_p2p_mgmt(struct cc33xx_vif *wlvif)
{
	return cc33xx_wlvif_to_vif(wlvif)->type == NL80211_IFTYPE_P2P_DEVICE;
}

#define cc33xx_for_each_wlvif(wl, wlvif) \
		list_for_each_entry(wlvif, &wl->wlvif_list, list)

#define cc33xx_for_each_wlvif_continue(wl, wlvif) \
		list_for_each_entry_continue(wlvif, &wl->wlvif_list, list)

#define cc33xx_for_each_wlvif_bss_type(wl, wlvif, _bss_type)	\
		cc33xx_for_each_wlvif(wl, wlvif)		\
			if (wlvif->bss_type == _bss_type)

#define cc33xx_for_each_wlvif_sta(wl, wlvif)	\
		cc33xx_for_each_wlvif_bss_type(wl, wlvif, BSS_TYPE_STA_BSS)

#define cc33xx_for_each_wlvif_ap(wl, wlvif)	\
		cc33xx_for_each_wlvif_bss_type(wl, wlvif, BSS_TYPE_AP_BSS)

int cc33xx_plt_start(struct cc33xx *wl, const enum plt_mode plt_mode);
int cc33xx_plt_stop(struct cc33xx *wl);
int cc33xx_recalc_rx_streaming(struct cc33xx *wl, struct cc33xx_vif *wlvif);
void cc33xx_queue_recovery_work(struct cc33xx *wl);
size_t cc33xx_copy_fwlog(struct cc33xx *wl, u8 *memblock, size_t maxlen);
int cc33xx_rx_filter_alloc_field(struct cc33xx_rx_filter *filter,
				 u16 offset, u8 flags,
				 const u8 *pattern, u8 len);
void cc33xx_rx_filter_free(struct cc33xx_rx_filter *filter);
struct cc33xx_rx_filter *cc33xx_rx_filter_alloc(void);
int cc33xx_rx_filter_get_fields_size(struct cc33xx_rx_filter *filter);
void cc33xx_rx_filter_flatten_fields(struct cc33xx_rx_filter *filter,
				     u8 *buf);
void cc33xx_flush_deferred_work(struct cc33xx *wl);

#define JOIN_TIMEOUT 5000 /* 5000 milliseconds to join */

#define SESSION_COUNTER_MAX 6 /* maximum value for the session counter */
#define SESSION_COUNTER_INVALID 7 /* used with dummy_packet */

#define CC33XX_MAX_TXPWR 21 /* maximum power limit is 21dBm */
#define CC33XX_MIN_TXPWR -10 /* minmum power limit is -10dBm */

#define CC33XX_TX_QUEUE_LOW_WATERMARK  32
#define CC33XX_TX_QUEUE_HIGH_WATERMARK 256

#define CC33XX_RX_QUEUE_MAX_LEN 256

/* cc33xx needs a 200ms sleep after power on, and a 20ms sleep before power
   on in case is has been shut down shortly before */
#define CC33XX_PRE_POWER_ON_SLEEP 20 /* in milliseconds */
#define CC33XX_POWER_ON_SLEEP 200 /* in milliseconds */

/* Macros to handle cc33xx.sta_rate_set */
#define HW_BG_RATES_MASK	0xffff
#define HW_HT_RATES_OFFSET	16
#define HW_MIMO_RATES_OFFSET	24

#endif /* __WLCORE_I_H__ */
