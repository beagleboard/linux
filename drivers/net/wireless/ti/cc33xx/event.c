// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022-2024 Texas Instruments Incorporated - https://www.ti.com/
 */

#include "acx.h"
#include "event.h"
#include "ps.h"
#include "io.h"
#include "scan.h"

#define CC33XX_WAIT_EVENT_FAST_POLL_COUNT 20

struct cc33xx_event_mailbox {
	__le32 events_vector;

	u8 number_of_scan_results;
	u8 number_of_sched_scan_results;

	__le16 channel_switch_role_id_bitmap;

	s8 rssi_snr_trigger_metric[NUM_OF_RSSI_SNR_TRIGGERS];

	/* bitmap of removed links */
	__le32 hlid_removed_bitmap;

	/* rx ba constraint */
	__le16 rx_ba_role_id_bitmap; /* 0xfff means any role. */
	__le16 rx_ba_allowed_bitmap;

	/* bitmap of roc completed (by role id) */
	__le16 roc_completed_bitmap;

	/* bitmap of stations (by role id) with bss loss */
	__le16 bss_loss_bitmap;

	/* bitmap of stations (by HLID) which exceeded max tx retries */
	__le16 tx_retry_exceeded_bitmap;

	/* time sync high msb*/
	__le16 time_sync_tsf_high_msb;

	/* bitmap of inactive stations (by HLID) */
	__le16 inactive_sta_bitmap;

	/* time sync high lsb*/
	__le16 time_sync_tsf_high_lsb;

	/* rx BA win size indicated by RX_BA_WIN_SIZE_CHANGE_EVENT_ID */
	u8 rx_ba_role_id;
	u8 rx_ba_link_id;
	u8 rx_ba_win_size;
	u8 padding;

	/* smart config */
	u8 sc_ssid_len;
	u8 sc_pwd_len;
	u8 sc_token_len;
	u8 padding1;
	u8 sc_ssid[32];
	u8 sc_pwd[64];
	u8 sc_token[32];

	/* smart config sync channel */
	u8 sc_sync_channel;
	u8 sc_sync_band;

	/* time sync low msb*/
	__le16 time_sync_tsf_low_msb;

	/* radar detect */
	u8 radar_channel;
	u8 radar_type;

	/* time sync low lsb*/
	__le16 time_sync_tsf_low_lsb;

	u8 ble_event[260];

} __packed;

struct event_node {
	struct llist_node node;
	struct cc33xx_event_mailbox event_data;
};

void deffer_event(struct cc33xx *cc,
		  const void *event_payload, size_t event_length)
{
	struct event_node *event_node;
	bool ret;

	if (WARN_ON(event_length != sizeof(event_node->event_data)))
		return;

	event_node = kzalloc(sizeof(*event_node), GFP_KERNEL);
	if (WARN_ON(!event_node))
		return;

	memcpy(&event_node->event_data,
	       event_payload, sizeof(event_node->event_data));

	llist_add(&event_node->node, &cc->event_list);
	ret = queue_work(cc->freezable_wq, &cc->irq_deferred_work);

	cc33xx_debug(DEBUG_IRQ, "Queued deferred work (%d)", ret);
}

static inline struct llist_node *get_event_list(struct cc33xx *cc)
{
	struct llist_node *node;

	node = llist_del_all(&cc->event_list);
	if (!node)
		return NULL;

	return llist_reverse_order(node);
}

void flush_deferred_event_list(struct cc33xx *cc)
{
	struct event_node *event_node, *tmp;
	struct llist_node *event_list;

	event_list = get_event_list(cc);
	llist_for_each_entry_safe(event_node, tmp, event_list, node) {
		kfree(event_node);
	}
}

static int wait_for_event_or_timeout(struct cc33xx *cc, u32 mask, bool *timeout)
{
	u32 event;
	unsigned long timeout_time;
	u16 poll_count = 0;
	int ret = 0;
	struct event_node *event_node, *tmp;
	struct llist_node *event_list;
	u32 vector;

	*timeout = false;

	timeout_time = jiffies + msecs_to_jiffies(CC33XX_EVENT_TIMEOUT);

	do {
		if (time_after(jiffies, timeout_time)) {
			cc33xx_debug(DEBUG_CMD, "timeout waiting for event %d",
				     (int)mask);
			*timeout = true;
			goto out;
		}

		poll_count++;
		if (poll_count < CC33XX_WAIT_EVENT_FAST_POLL_COUNT)
			usleep_range(50, 51);
		else
			usleep_range(1000, 5000);

		vector = 0;
		event_list = get_event_list(cc);
		llist_for_each_entry_safe(event_node, tmp, event_list, node) {
			vector |= le32_to_cpu(event_node->event_data.events_vector);
		}

		event  = vector & mask;
	} while (!event);

out:

	return ret;
}

int cc33xx_wait_for_event(struct cc33xx *cc, enum cc33xx_wait_event event,
			  bool *timeout)
{
	u32 local_event;

	switch (event) {
	case CC33XX_EVENT_PEER_REMOVE_COMPLETE:
		local_event = PEER_REMOVE_COMPLETE_EVENT_ID;
		break;

	case CC33XX_EVENT_DFS_CONFIG_COMPLETE:
		local_event = DFS_CHANNELS_CONFIG_COMPLETE_EVENT;
		break;

	default:
		/* event not implemented */
		return 0;
	}
	return wait_for_event_or_timeout(cc, local_event, timeout);
}

static void cc33xx_event_sched_scan_completed(struct cc33xx *cc, u8 status)
{
	cc33xx_debug(DEBUG_EVENT,
		     "PERIODIC_SCAN_COMPLETE_EVENT (status 0x%0x)", status);

	if (cc->mac80211_scan_stopped) {
		cc->mac80211_scan_stopped = false;
	} else {
		if (cc->sched_vif) {
			ieee80211_sched_scan_stopped(cc->hw);
			cc->sched_vif = NULL;
		}
	}
}

static void cc33xx_event_channel_switch(struct cc33xx *cc,
					unsigned long roles_bitmap,
				 bool success)
{
	struct cc33xx_vif *wlvif;
	struct ieee80211_vif *vif;

	cc33xx_debug(DEBUG_EVENT, "%s: roles=0x%lx success=%d",
		     __func__, roles_bitmap, success);

	cc33xx_for_each_wlvif(cc, wlvif) {
		if (wlvif->role_id == CC33XX_INVALID_ROLE_ID ||
		    !test_bit(wlvif->role_id, &roles_bitmap))
			continue;

		if (!test_and_clear_bit(WLVIF_FLAG_CS_PROGRESS,
					&wlvif->flags))
			continue;

		vif = cc33xx_wlvif_to_vif(wlvif);

		if (wlvif->bss_type == BSS_TYPE_STA_BSS) {
			ieee80211_chswitch_done(vif, success, 0);
			cancel_delayed_work(&wlvif->channel_switch_work);
		} else {
			set_bit(WLVIF_FLAG_BEACON_DISABLED, &wlvif->flags);
			ieee80211_csa_finish(vif, 0);
		}
	}
}

static void cc33xx_disconnect_sta(struct cc33xx *cc, unsigned long sta_bitmap)
{
	u32 num_packets = cc->conf.host_conf.tx.max_tx_retries;
	struct cc33xx_vif *wlvif;
	struct ieee80211_vif *vif;
	struct ieee80211_sta *sta;
	const u8 *addr;
	int h;

	for_each_set_bit(h, &sta_bitmap, CC33XX_MAX_LINKS) {
		bool found = false;
		/* find the ap vif connected to this sta */
		cc33xx_for_each_wlvif_ap(cc, wlvif) {
			if (!test_bit(h, wlvif->ap.sta_hlid_map))
				continue;
			found = true;
			break;
		}
		if (!found)
			continue;

		vif = cc33xx_wlvif_to_vif(wlvif);
		addr = cc->links[h].addr;

		rcu_read_lock();
		sta = ieee80211_find_sta(vif, addr);
		if (sta) {
			cc33xx_debug(DEBUG_EVENT, "remove sta %d", h);
			ieee80211_report_low_ack(sta, num_packets);
		}
		rcu_read_unlock();
	}
}

static void cc33xx_event_max_tx_failure(struct cc33xx *cc,
					unsigned long sta_bitmap)
{
	cc33xx_disconnect_sta(cc, sta_bitmap);
}

static void cc33xx_event_roc_complete(struct cc33xx *cc)
{
	if (cc->roc_vif)
		ieee80211_ready_on_channel(cc->hw);
}

static void cc33xx_event_beacon_loss(struct cc33xx *cc,
				     unsigned long roles_bitmap)
{
	/* We are HW_MONITOR device. On beacon loss - queue
	 * connection loss work. Cancel it on REGAINED event.
	 */
	struct cc33xx_vif *wlvif;
	struct ieee80211_vif *vif;
	int delay = cc->conf.host_conf.conn.synch_fail_thold;

	delay *= cc->conf.host_conf.conn.bss_lose_timeout;

	cc33xx_info("Beacon loss detected. roles:0x%lx", roles_bitmap);

	cc33xx_for_each_wlvif_sta(cc, wlvif) {
		if (wlvif->role_id == CC33XX_INVALID_ROLE_ID ||
		    !test_bit(wlvif->role_id, &roles_bitmap))
			continue;

		vif = cc33xx_wlvif_to_vif(wlvif);

		/* don't attempt roaming in case of p2p */
		if (wlvif->p2p) {
			ieee80211_connection_loss(vif);
			continue;
		}

		/* if the work is already queued, it should take place.
		 * We don't want to delay the connection loss
		 * indication any more.
		 */
		ieee80211_queue_delayed_work(cc->hw,
					     &wlvif->connection_loss_work,
					     msecs_to_jiffies(delay));

		ieee80211_cqm_beacon_loss_notify(vif, GFP_KERNEL);
	}
}

void process_deferred_events(struct cc33xx *cc)
{
	struct event_node *event_node, *tmp;
	struct llist_node *event_list;
	u32 vector;

	event_list = get_event_list(cc);

	llist_for_each_entry_safe(event_node, tmp, event_list, node) {
		struct cc33xx_event_mailbox *event_data;

		event_data = &event_node->event_data;

		vector = le32_to_cpu(event_node->event_data.events_vector);
		cc33xx_debug(DEBUG_EVENT, "MBOX vector: 0x%x", vector);

		if (vector & SCAN_COMPLETE_EVENT_ID) {
			cc33xx_debug(DEBUG_EVENT, "scan results: %d",
				     event_node->event_data.number_of_scan_results);

			if (cc->scan_wlvif)
				cc33xx_scan_completed(cc, cc->scan_wlvif);
		}

		if (vector & PERIODIC_SCAN_COMPLETE_EVENT_ID)
			cc33xx_event_sched_scan_completed(cc, 1);

		if (vector & BSS_LOSS_EVENT_ID) {
			u16 bss_loss_bitmap = le16_to_cpu(event_data->bss_loss_bitmap);

			cc33xx_event_beacon_loss(cc, bss_loss_bitmap);
		}

		if (vector & MAX_TX_FAILURE_EVENT_ID) {
			u16 tx_retry_exceeded_bitmap =
				le16_to_cpu(event_data->tx_retry_exceeded_bitmap);

			cc33xx_event_max_tx_failure(cc, tx_retry_exceeded_bitmap);
		}

		if (vector & PERIODIC_SCAN_REPORT_EVENT_ID) {
			cc33xx_debug(DEBUG_EVENT,
				     "PERIODIC_SCAN_REPORT (results %d)",
				     event_data->number_of_sched_scan_results);

			cc33xx_scan_sched_scan_results(cc);
		}

		if (vector & CHANNEL_SWITCH_COMPLETE_EVENT_ID) {
			u16 channel_switch_role_id_bitmap =
				le16_to_cpu(event_data->channel_switch_role_id_bitmap);

			cc33xx_event_channel_switch(cc, channel_switch_role_id_bitmap, true);
		}

		if (vector & REMAIN_ON_CHANNEL_COMPLETE_EVENT_ID)
			cc33xx_event_roc_complete(cc);

		kfree(event_node);
	}
}
