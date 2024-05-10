/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * This file is part of cc33xx
 *
 * Copyright (C) 1998-2009 Texas Instruments. All rights reserved.
 * Copyright (C) 2009 Nokia Corporation
 *
 * Contact: Luciano Coelho <luciano.coelho@nokia.com>
 */

#ifndef __TX_H__
#define __TX_H__





#define CC33XX_TX_HW_BLOCK_SPARE        1
/* for special cases - namely, TKIP and GEM */
#define CC33XX_TX_HW_EXTRA_BLOCK_SPARE  2
#define CC33XX_TX_HW_BLOCK_SIZE         256

#define CC33XX_TX_STATUS_DESC_ID_MASK    0x7F
#define CC33XX_TX_STATUS_STAT_BIT_IDX    7

/* Indicates this TX HW frame is not padded to SDIO block size */
#define CC33XX_TX_CTRL_NOT_PADDED	BIT(7)

/*
 * The FW uses a special bit to indicate a wide channel should be used in
 * the rate policy.
 */
#define CONF_TX_RATE_USE_WIDE_CHAN BIT(31)

#define TX_HW_MGMT_PKT_LIFETIME_TU       2000
#define TX_HW_AP_MODE_PKT_LIFETIME_TU    8000

#define TX_HW_ATTR_SAVE_RETRIES          BIT(0)
#define TX_HW_ATTR_HEADER_PAD            BIT(1)
#define TX_HW_ATTR_SESSION_COUNTER       (BIT(2) | BIT(3) | BIT(4))
#define TX_HW_ATTR_RATE_POLICY           (BIT(5) | BIT(6) | BIT(7) | \
					  BIT(8) | BIT(9))
#define TX_HW_ATTR_LAST_WORD_PAD         (BIT(10) | BIT(11))
#define TX_HW_ATTR_TX_CMPLT_REQ          BIT(12)
#define TX_HW_ATTR_TX_DUMMY_REQ          BIT(13)
#define TX_HW_ATTR_HOST_ENCRYPT          BIT(14)
#define TX_HW_ATTR_EAPOL_FRAME           BIT(15)

#define TX_HW_ATTR_OFST_SAVE_RETRIES     0
#define TX_HW_ATTR_OFST_HEADER_PAD       1
#define TX_HW_ATTR_OFST_SESSION_COUNTER  2
#define TX_HW_ATTR_OFST_RATE_POLICY      5
#define TX_HW_ATTR_OFST_LAST_WORD_PAD    10
#define TX_HW_ATTR_OFST_TX_CMPLT_REQ     12

#define TX_HW_RESULT_QUEUE_LEN           16
#define TX_HW_RESULT_QUEUE_LEN_MASK      0xf

#define CC33XX_TX_ALIGN_TO 4
#define CC33XX_EXTRA_SPACE_TKIP 4
#define CC33XX_EXTRA_SPACE_AES  8
#define CC33XX_EXTRA_SPACE_MAX  8

#define CC33XX_TX_EXTRA_HEADROOM (sizeof(struct cc33xx_tx_hw_descr) + IEEE80211_HT_CTL_LEN)

/* Used for management frames and dummy packets */
#define CC33XX_TID_MGMT 7

/* stop a ROC for pending authentication reply after this time (ms) */
#define WLCORE_PEND_AUTH_ROC_TIMEOUT     1000
#define CC33xx_PEND_ROC_COMPLETE_TIMEOUT 2000 

struct cc33xx_tx_mem {
	/*
	 * Total number of memory blocks allocated by the host for
	 * this packet.
	 */
	u8 total_mem_blocks;

	/*
	 * control bits
	 */
	u8 ctrl;
} __packed;

/*
 * On cc33xx based devices, when TX packets are aggregated, each packet
 * size must be aligned to the SDIO block size. The maximum block size
 * is bounded by the type of the padded bytes field that is sent to the
 * FW. Currently the type is u8, so the maximum block size is 256 bytes.
 */
// For CC33xx O3
#define CC33XX_BUS_BLOCK_SIZE 128

struct cc33xx_tx_hw_descr {
	/* Length of packet in words, including descriptor+header+data */
	__le16 length;
	
	struct cc33xx_tx_mem cc33xx_mem;
	// michal temp removal - need four bytes for ax header

	/* Packet identifier used also in the Tx-Result. */
	u8 id;
	/* The packet TID value (as User-Priority) */
	u8 tid;
	/* host link ID (HLID) */
	u8 hlid;
	u8  ac;
	/*
	* Max delay in TUs until transmission. The last device time the
	* packet can be transmitted is: start_time + (1024 * life_time)
	*/
	__le16 life_time;
	/* Bitwise fields - see TX_ATTR... definitions above. */
	__le16 tx_attr;
} __packed;

enum cc33xx_tx_hw_res_status {
	TX_SUCCESS          = 0,
	TX_HW_ERROR         = 1,
	TX_DISABLED         = 2,
	TX_RETRY_EXCEEDED   = 3,
	TX_TIMEOUT          = 4,
	TX_KEY_NOT_FOUND    = 5,
	TX_PEER_NOT_FOUND   = 6,
	TX_SESSION_MISMATCH = 7,
	TX_LINK_NOT_VALID   = 8,
};

struct cc33xx_tx_hw_res_descr {
	/* Packet Identifier - same value used in the Tx descriptor.*/
	u8 id;
	/* The status of the transmission, indicating success or one of
	   several possible reasons for failure. */
	u8 status;
	/* Total air access duration including all retrys and overheads.*/
	__le16 medium_usage;
	/* The time passed from host xfer to Tx-complete.*/
	__le32 fw_handling_time;
	/* Total media delay
	   (from 1st EDCA AIFS counter until TX Complete). */
	__le32 medium_delay;
	/* LS-byte of last TKIP seq-num (saved per AC for recovery). */
	u8 tx_security_sequence_number_lsb;
	/* Retry count - number of transmissions without successful ACK.*/
	u8 ack_failures;
	/* The rate that succeeded getting ACK
	   (Valid only if status=SUCCESS). */
	u8 rate_class_index;
	/* for 4-byte alignment. */
	u8 spare;
} __packed;



struct cc33xx_tx_hw_res_if {
	__le32 tx_result_fw_counter;
	__le32 tx_result_host_counter;
	struct cc33xx_tx_hw_res_descr tx_results_queue[TX_HW_RESULT_QUEUE_LEN];
} __packed;

enum wlcore_queue_stop_reason {
	WLCORE_QUEUE_STOP_REASON_WATERMARK,
	WLCORE_QUEUE_STOP_REASON_FW_RESTART,
	WLCORE_QUEUE_STOP_REASON_FLUSH,
	WLCORE_QUEUE_STOP_REASON_SPARE_BLK, /* 18xx specific */
};

static inline int cc33xx_tx_get_queue(int queue)
{
	switch (queue) {
	case 0:
		return CONF_TX_AC_VO;
	case 1:
		return CONF_TX_AC_VI;
	case 2:
		return CONF_TX_AC_BE;
	case 3:
		return CONF_TX_AC_BK;
	default:
		return CONF_TX_AC_BE;
	}
}

static inline
int wlcore_tx_get_mac80211_queue(struct cc33xx_vif *wlvif, int queue)
{
	int mac_queue = wlvif->hw_queue_base;

	switch (queue) {
	case CONF_TX_AC_VO:
		return mac_queue + 0;
	case CONF_TX_AC_VI:
		return mac_queue + 1;
	case CONF_TX_AC_BE:
		return mac_queue + 2;
	case CONF_TX_AC_BK:
		return mac_queue + 3;
	default:
		return mac_queue + 2;
	}
}

static inline int cc33xx_tx_total_queue_count(struct cc33xx *wl)
{
	int i, count = 0;

	for (i = 0; i < NUM_TX_QUEUES; i++)
		count += wl->tx_queue_count[i];

	return count;
}


void cc33xx_tx_immediate_complete(struct cc33xx *wl);
void cc33xx_tx_work(struct work_struct *work);
int wlcore_tx_work_locked(struct cc33xx *wl);
void cc33xx_tx_reset_wlvif(struct cc33xx *wl, struct cc33xx_vif *wlvif);
void cc33xx_tx_reset(struct cc33xx *wl);
void cc33xx_tx_flush(struct cc33xx *wl);
u8 wlcore_rate_to_idx(struct cc33xx *wl, u8 rate, enum nl80211_band band);
u32 cc33xx_tx_enabled_rates_get(struct cc33xx *wl, u32 rate_set,
				enum nl80211_band rate_band);
u32 cc33xx_tx_min_rate_get(struct cc33xx *wl, u32 rate_set);
u8 cc33xx_tx_get_hlid(struct cc33xx *wl, struct cc33xx_vif *wlvif,
		      struct sk_buff *skb, struct ieee80211_sta *sta);
void cc33xx_tx_reset_link_queues(struct cc33xx *wl, u8 hlid);
void cc33xx_handle_tx_low_watermark(struct cc33xx *wl);
bool cc33xx_is_dummy_packet(struct cc33xx *wl, struct sk_buff *skb);
void cc33xx_rearm_rx_streaming(struct cc33xx *wl, unsigned long *active_hlids);
unsigned int wlcore_calc_packet_alignment(struct cc33xx *wl,
					  unsigned int packet_length);
void cc33xx_free_tx_id(struct cc33xx *wl, int id);
void wlcore_stop_queue_locked(struct cc33xx *wl, struct cc33xx_vif *wlvif,
			      u8 queue, enum wlcore_queue_stop_reason reason);
void wlcore_stop_queue(struct cc33xx *wl, struct cc33xx_vif *wlvif, u8 queue,
		       enum wlcore_queue_stop_reason reason);
void wlcore_wake_queue(struct cc33xx *wl, struct cc33xx_vif *wlvif, u8 queue,
		       enum wlcore_queue_stop_reason reason);
void wlcore_stop_queues(struct cc33xx *wl,
			enum wlcore_queue_stop_reason reason);
void wlcore_wake_queues(struct cc33xx *wl,
			enum wlcore_queue_stop_reason reason);
bool wlcore_is_queue_stopped_by_reason(struct cc33xx *wl,
				       struct cc33xx_vif *wlvif, u8 queue,
				       enum wlcore_queue_stop_reason reason);
bool
wlcore_is_queue_stopped_by_reason_locked(struct cc33xx *wl,
					 struct cc33xx_vif *wlvif,
					 u8 queue,
					 enum wlcore_queue_stop_reason reason);
bool wlcore_is_queue_stopped_locked(struct cc33xx *wl, struct cc33xx_vif *wlvif,
				    u8 queue);

/* from main.c */
void cc33xx_free_sta(struct cc33xx *wl, struct cc33xx_vif *wlvif, u8 hlid);
void cc33xx_rearm_tx_watchdog_locked(struct cc33xx *wl);

#endif
