/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * This file is part of cc33xx
 *
 * Copyright (C) 1998-2009 Texas Instruments. All rights reserved.
 * Copyright (C) 2008-2009 Nokia Corporation
 *
 * Contact: Luciano Coelho <luciano.coelho@nokia.com>
 */

#ifndef __EVENT_H__
#define __EVENT_H__

/*
 * Mbox events
 *
 * The event mechanism is based on a pair of event buffers (buffers A and
 * B) at fixed locations in the target's memory. The host processes one
 * buffer while the other buffer continues to collect events. If the host
 * is not processing events, an interrupt is issued to signal that a buffer
 * is ready. Once the host is done with processing events from one buffer,
 * it signals the target (with an ACK interrupt) that the event buffer is
 * free.
 */

enum {
	RSSI_SNR_TRIGGER_0_EVENT_ID              = BIT(0),
	RSSI_SNR_TRIGGER_1_EVENT_ID              = BIT(1),
	RSSI_SNR_TRIGGER_2_EVENT_ID              = BIT(2),
	RSSI_SNR_TRIGGER_3_EVENT_ID              = BIT(3),
	RSSI_SNR_TRIGGER_4_EVENT_ID              = BIT(4),
	RSSI_SNR_TRIGGER_5_EVENT_ID              = BIT(5),
	RSSI_SNR_TRIGGER_6_EVENT_ID              = BIT(6),
	RSSI_SNR_TRIGGER_7_EVENT_ID              = BIT(7),

	EVENT_MBOX_ALL_EVENT_ID			 = 0x7fffffff,
};

enum {
	SCAN_COMPLETE_EVENT_ID                   = BIT(8),
	RADAR_DETECTED_EVENT_ID                  = BIT(9),
	CHANNEL_SWITCH_COMPLETE_EVENT_ID         = BIT(10),
	BSS_LOSS_EVENT_ID                        = BIT(11),
	MAX_TX_FAILURE_EVENT_ID                  = BIT(12),
	DUMMY_PACKET_EVENT_ID                    = BIT(13),
	INACTIVE_STA_EVENT_ID                    = BIT(14),
	PEER_REMOVE_COMPLETE_EVENT_ID            = BIT(15),
	PERIODIC_SCAN_COMPLETE_EVENT_ID          = BIT(16),
	BA_SESSION_RX_CONSTRAINT_EVENT_ID        = BIT(17),
	REMAIN_ON_CHANNEL_COMPLETE_EVENT_ID      = BIT(18),
	DFS_CHANNELS_CONFIG_COMPLETE_EVENT       = BIT(19),
	PERIODIC_SCAN_REPORT_EVENT_ID            = BIT(20),
	RX_BA_WIN_SIZE_CHANGE_EVENT_ID           = BIT(21),
	SMART_CONFIG_SYNC_EVENT_ID               = BIT(22),
	SMART_CONFIG_DECODE_EVENT_ID             = BIT(23),
	TIME_SYNC_EVENT_ID                       = BIT(24),
	FW_LOGGER_INDICATION			= BIT(25),
};

/* events the driver might want to wait for */
enum wlcore_wait_event {
	WLCORE_EVENT_ROLE_STOP_COMPLETE,
	WLCORE_EVENT_PEER_REMOVE_COMPLETE,
	WLCORE_EVENT_DFS_CONFIG_COMPLETE
};

enum {
	EVENT_ENTER_POWER_SAVE_FAIL = 0,
	EVENT_ENTER_POWER_SAVE_SUCCESS,
};


#define NUM_OF_RSSI_SNR_TRIGGERS 8

struct fw_logger_information {
	__le32 max_buff_size;
	__le32 actual_buff_size;
	__le32 num_trace_drop;
	__le32 buff_read_ptr;
	__le32 buff_write_ptr;
} __packed;

struct cc33xx;

int cc33xx_event_unmask(struct cc33xx *wl);

void wlcore_event_soft_gemini_sense(struct cc33xx *wl, u8 enable);
void wlcore_event_sched_scan_completed(struct cc33xx *wl,
				       u8 status);
void wlcore_event_ba_rx_constraint(struct cc33xx *wl,
				   unsigned long roles_bitmap,
				   unsigned long allowed_bitmap);
void wlcore_event_channel_switch(struct cc33xx *wl,
				 unsigned long roles_bitmap,
				 bool success);
void wlcore_event_beacon_loss(struct cc33xx *wl, unsigned long roles_bitmap);
void wlcore_event_dummy_packet(struct cc33xx *wl);
void wlcore_event_max_tx_failure(struct cc33xx *wl, unsigned long sta_bitmap);
void wlcore_event_inactive_sta(struct cc33xx *wl, unsigned long sta_bitmap);
void wlcore_event_roc_complete(struct cc33xx *wl);
void wlcore_event_rssi_trigger(struct cc33xx *wl, s8 *metric_arr);
int  wlcore_event_fw_logger(struct cc33xx *wl);

int cc33xx_wait_for_event(struct cc33xx *wl, enum wlcore_wait_event event,
			  bool *timeout);

void deffer_event(struct cc33xx *wl, const void *event_payload, size_t event_length);
void process_deferred_events(struct cc33xx *wl);
void flush_deferred_event_list(struct cc33xx *wl);
#endif
