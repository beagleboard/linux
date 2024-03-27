// SPDX-License-Identifier: GPL-2.0-only
/*
 * This file is part of cc33xx
 *
 * Copyright (C) 2008-2009 Nokia Corporation
 *
 * Contact: Luciano Coelho <luciano.coelho@nokia.com>
 */

#include "wlcore.h"
#include "debug.h"
#include "io.h"
#include "event.h"
#include "ps.h"
#include "scan.h"
#include "cc33xx_80211.h"

#define CC33XX_LOGGER_SDIO_BUFF_MAX	(0x1020)
#define CC33XX_DATA_RAM_BASE_ADDRESS	(0x20000000)
#define CC33XX_LOGGER_SDIO_BUFF_ADDR	(0x40159c)
#define CC33XX_LOGGER_BUFF_OFFSET	(sizeof(struct fw_logger_information))
#define CC33XX_LOGGER_READ_POINT_OFFSET	(12)


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

} __packed;

struct event_node{
	struct llist_node node;
	struct cc33xx_event_mailbox event_data;
};

void deffer_event(struct cc33xx *wl,
			const void *event_payload, size_t event_length)
{
	struct event_node* event_node;
	bool ret;

	if (WARN_ON(event_length != sizeof event_node->event_data))
		return;

	event_node = kzalloc(sizeof *event_node, GFP_KERNEL);
	if (WARN_ON(!event_node))
		return;

	memcpy(&event_node->event_data,
		event_payload, sizeof (event_node->event_data));

	llist_add(&event_node->node, &wl->event_list);
	ret = queue_work(wl->freezable_wq, &wl->irq_deferred_work);

	cc33xx_debug(DEBUG_IRQ, "Queued deferred work (%d)", ret);
}

inline static struct llist_node* get_event_list(struct cc33xx *wl)
{
	struct llist_node* node;

	node = llist_del_all(&wl->event_list);
	if (!node)
		return NULL;

	return llist_reverse_order(node);
}

void process_deferred_events(struct cc33xx *wl)
{
	struct event_node *event_node, *tmp;
	struct llist_node *event_list;
	u32 vector;

	event_list = get_event_list(wl);

	llist_for_each_entry_safe(event_node, tmp, event_list, node){

		struct cc33xx_event_mailbox *event_data;

		event_data = &event_node->event_data;

		print_hex_dump(KERN_DEBUG, "Deferred event dump:",
			DUMP_PREFIX_OFFSET, 4, 4,
			event_data, 64/*sizeof event_node->event_data*/,
			false);

		vector = le32_to_cpu(event_node->event_data.events_vector);
		cc33xx_debug(DEBUG_EVENT, "MBOX vector: 0x%x", vector);

		if (vector & SCAN_COMPLETE_EVENT_ID) {
			cc33xx_debug(DEBUG_EVENT, "scan results: %d",
				event_node->event_data.number_of_scan_results);

			if (wl->scan_wlvif)
				cc33xx_scan_completed(wl, wl->scan_wlvif);
		}

		if (vector & PERIODIC_SCAN_COMPLETE_EVENT_ID)
		{
			wlcore_event_sched_scan_completed(wl, 1);
		}

		if (vector & BSS_LOSS_EVENT_ID)
			wlcore_event_beacon_loss(wl,
				le16_to_cpu(event_data->bss_loss_bitmap));

		if (vector & MAX_TX_FAILURE_EVENT_ID)
			wlcore_event_max_tx_failure(wl,
				le16_to_cpu(event_data->tx_retry_exceeded_bitmap));

		if (vector & PERIODIC_SCAN_REPORT_EVENT_ID) {
			cc33xx_debug(DEBUG_EVENT,
				"PERIODIC_SCAN_REPORT_EVENT (results %d)",
				event_data->number_of_sched_scan_results);

			wlcore_scan_sched_scan_results(wl);
		}

		if (vector & REMAIN_ON_CHANNEL_COMPLETE_EVENT_ID)
			wlcore_event_roc_complete(wl);
		kfree(event_node);
	}

}

void flush_deferred_event_list(struct cc33xx *wl)
{
	struct event_node *event_node, *tmp;
	struct llist_node *event_list;

	event_list = get_event_list(wl);
	llist_for_each_entry_safe(event_node, tmp, event_list, node){
		cc33xx_debug(DEBUG_IRQ, "Freeing event");
		kfree(event_node);
	}
}
#define CC33XX_WAIT_EVENT_FAST_POLL_COUNT 20
int wait_for_event_or_timeout(struct cc33xx *wl, u32 mask, bool *timeout)
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
		event_list = get_event_list(wl);
		llist_for_each_entry_safe(event_node, tmp, event_list, node){
			vector |= le32_to_cpu(event_node->event_data.events_vector);
		}

		event  = vector & mask;
	} while (!event);

out:

	return ret;
}



int cc33xx_wait_for_event(struct cc33xx *wl, enum wlcore_wait_event event,
			  bool *timeout)
{
	u32 local_event;

	switch (event) {
	case WLCORE_EVENT_PEER_REMOVE_COMPLETE:
		local_event = PEER_REMOVE_COMPLETE_EVENT_ID;
		break;

	case WLCORE_EVENT_DFS_CONFIG_COMPLETE:
		local_event = DFS_CHANNELS_CONFIG_COMPLETE_EVENT;
		break;

	default:
		/* event not implemented */
		return 0;
	}
	return wait_for_event_or_timeout(wl, local_event, timeout);
}


int wlcore_event_fw_logger(struct cc33xx *wl)
{
	int ret;
	struct fw_logger_information fw_log;
	u8  *buffer;
	u32 internal_fw_addrbase = CC33XX_DATA_RAM_BASE_ADDRESS;
	u32 addr = CC33XX_LOGGER_SDIO_BUFF_ADDR;
	u32 end_buff_addr = CC33XX_LOGGER_SDIO_BUFF_ADDR +
				CC33XX_LOGGER_BUFF_OFFSET;
	u32 available_len;
	u32 actual_len;
	u32 clear_addr;
	size_t len;
	u32 start_loc;

	buffer = kzalloc(CC33XX_LOGGER_SDIO_BUFF_MAX, GFP_KERNEL);
	if (!buffer) {
		cc33xx_error("Fail to allocate fw logger memory");
		fw_log.actual_buff_size = cpu_to_le32(0);
		goto out;
	}

	ret = wlcore_read(wl, addr, buffer, CC33XX_LOGGER_SDIO_BUFF_MAX,
			  false);
	if (ret < 0) {
		cc33xx_error("Fail to read logger buffer, error_id = %d",
			     ret);
		fw_log.actual_buff_size = cpu_to_le32(0);
		goto free_out;
	}

	memcpy(&fw_log, buffer, sizeof(fw_log));

	if (le32_to_cpu(fw_log.actual_buff_size) == 0)
		goto free_out;

	actual_len = le32_to_cpu(fw_log.actual_buff_size);
	start_loc = (le32_to_cpu(fw_log.buff_read_ptr) -
			internal_fw_addrbase) - addr;
	end_buff_addr += le32_to_cpu(fw_log.max_buff_size);
	available_len = end_buff_addr -
			(le32_to_cpu(fw_log.buff_read_ptr) -
				 internal_fw_addrbase);
	actual_len = min(actual_len, available_len);
	len = actual_len;

	cc33xx_copy_fwlog(wl, &buffer[start_loc], len);
	clear_addr = addr + start_loc + le32_to_cpu(fw_log.actual_buff_size) +
			internal_fw_addrbase;

	len = le32_to_cpu(fw_log.actual_buff_size) - len;
	if (len) {
		cc33xx_copy_fwlog(wl,
				  &buffer[CC33XX_LOGGER_BUFF_OFFSET],
				  len);
		clear_addr = addr + CC33XX_LOGGER_BUFF_OFFSET + len +
				internal_fw_addrbase;
	}

	/* double check that clear address and write pointer are the same */
	if (clear_addr != le32_to_cpu(fw_log.buff_write_ptr)) {
		cc33xx_error("Calculate of clear addr Clear = %x, write = %x",
			     clear_addr, le32_to_cpu(fw_log.buff_write_ptr));
	}

	/* indicate FW about Clear buffer */
	//ret = wlcore_write32(wl, addr + CC33XX_LOGGER_READ_POINT_OFFSET,
	//			     fw_log.buff_write_ptr);
free_out:
	kfree(buffer);
out:
	return le32_to_cpu(fw_log.actual_buff_size);
}

void wlcore_event_rssi_trigger(struct cc33xx *wl, s8 *metric_arr)
{
	struct cc33xx_vif *wlvif;
	struct ieee80211_vif *vif;
	enum nl80211_cqm_rssi_threshold_event event;
	s8 metric = metric_arr[0];

	cc33xx_debug(DEBUG_EVENT, "RSSI trigger metric: %d", metric);

	/* TODO: check actual multi-role support */
	cc33xx_for_each_wlvif_sta(wl, wlvif) {
		if (metric <= wlvif->rssi_thold)
			event = NL80211_CQM_RSSI_THRESHOLD_EVENT_LOW;
		else
			event = NL80211_CQM_RSSI_THRESHOLD_EVENT_HIGH;

		vif = cc33xx_wlvif_to_vif(wlvif);
		if (event != wlvif->last_rssi_event)
			ieee80211_cqm_rssi_notify(vif, event, metric,
						  GFP_KERNEL);
		wlvif->last_rssi_event = event;
	}
}

static void cc33xx_stop_ba_event(struct cc33xx *wl, struct cc33xx_vif *wlvif)
{
	struct ieee80211_vif *vif = cc33xx_wlvif_to_vif(wlvif);

	if (wlvif->bss_type != BSS_TYPE_AP_BSS) {
		u8 hlid = wlvif->sta.hlid;
		if (!wl->links[hlid].ba_bitmap)
			return;
		ieee80211_stop_rx_ba_session(vif, wl->links[hlid].ba_bitmap,
					     vif->bss_conf.bssid);
	} else {
		u8 hlid;
		struct cc33xx_link *lnk;
		for_each_set_bit(hlid, wlvif->ap.sta_hlid_map,
				 wl->num_links) {
			lnk = &wl->links[hlid];
			if (!lnk->ba_bitmap)
				continue;

			ieee80211_stop_rx_ba_session(vif,
						     lnk->ba_bitmap,
						     lnk->addr);
		}
	}
}

void wlcore_event_soft_gemini_sense(struct cc33xx *wl, u8 enable)
{
	struct cc33xx_vif *wlvif;

	if (enable) {
		set_bit(CC33XX_FLAG_SOFT_GEMINI, &wl->flags);
	} else {
		clear_bit(CC33XX_FLAG_SOFT_GEMINI, &wl->flags);
		cc33xx_for_each_wlvif_sta(wl, wlvif) {
			cc33xx_recalc_rx_streaming(wl, wlvif);
		}
	}
}

void wlcore_event_sched_scan_completed(struct cc33xx *wl,
				       u8 status)
{
	cc33xx_debug(DEBUG_EVENT, "PERIODIC_SCAN_COMPLETE_EVENT (status 0x%0x)",
		     status);

	if (wl->mac80211_scan_stopped){
		wl->mac80211_scan_stopped = false;
	}
	else{
		if (wl->sched_vif) {
			ieee80211_sched_scan_stopped(wl->hw);
			wl->sched_vif = NULL;
		}
	}

}

void wlcore_event_ba_rx_constraint(struct cc33xx *wl,
				   unsigned long roles_bitmap,
				   unsigned long allowed_bitmap)
{
	struct cc33xx_vif *wlvif;

	cc33xx_debug(DEBUG_EVENT, "%s: roles=0x%lx allowed=0x%lx",
		     __func__, roles_bitmap, allowed_bitmap);

	cc33xx_for_each_wlvif(wl, wlvif) {
		if (wlvif->role_id == CC33XX_INVALID_ROLE_ID ||
		    !test_bit(wlvif->role_id , &roles_bitmap))
			continue;

		wlvif->ba_allowed = !!test_bit(wlvif->role_id,
					       &allowed_bitmap);
		if (!wlvif->ba_allowed)
			cc33xx_stop_ba_event(wl, wlvif);
	}
}

void wlcore_event_channel_switch(struct cc33xx *wl,
				 unsigned long roles_bitmap,
				 bool success)
{
	struct cc33xx_vif *wlvif;
	struct ieee80211_vif *vif;

	cc33xx_debug(DEBUG_EVENT, "%s: roles=0x%lx success=%d",
		     __func__, roles_bitmap, success);

	cc33xx_for_each_wlvif(wl, wlvif) {
		if (wlvif->role_id == CC33XX_INVALID_ROLE_ID ||
		    !test_bit(wlvif->role_id , &roles_bitmap))
			continue;

		if (!test_and_clear_bit(WLVIF_FLAG_CS_PROGRESS,
					&wlvif->flags))
			continue;

		vif = cc33xx_wlvif_to_vif(wlvif);

		if (wlvif->bss_type == BSS_TYPE_STA_BSS) {
			ieee80211_chswitch_done(vif, success);
			cancel_delayed_work(&wlvif->channel_switch_work);
		} else {
			set_bit(WLVIF_FLAG_BEACON_DISABLED, &wlvif->flags);
			ieee80211_csa_finish(vif);
		}
	}
}

void wlcore_event_dummy_packet(struct cc33xx *wl)
{
	if (wl->plt) {
		cc33xx_info("Got DUMMY_PACKET event in PLT mode.  FW bug, ignoring.");
		return;
	}

	cc33xx_debug(DEBUG_EVENT, "DUMMY_PACKET_ID_EVENT_ID");
	cc33xx_tx_dummy_packet(wl);
}

static void wlcore_disconnect_sta(struct cc33xx *wl, unsigned long sta_bitmap)
{
	u32 num_packets = wl->conf.host_conf.tx.max_tx_retries;
	struct cc33xx_vif *wlvif;
	struct ieee80211_vif *vif;
	struct ieee80211_sta *sta;
	const u8 *addr;
	int h;

	for_each_set_bit(h, &sta_bitmap, wl->num_links) {
		bool found = false;
		/* find the ap vif connected to this sta */
		cc33xx_for_each_wlvif_ap(wl, wlvif) {
			if (!test_bit(h, wlvif->ap.sta_hlid_map))
				continue;
			found = true;
			break;
		}
		if (!found)
			continue;

		vif = cc33xx_wlvif_to_vif(wlvif);
		addr = wl->links[h].addr;

		rcu_read_lock();
		sta = ieee80211_find_sta(vif, addr);
		if (sta) {
			cc33xx_debug(DEBUG_EVENT, "remove sta %d", h);
			ieee80211_report_low_ack(sta, num_packets);
		}
		rcu_read_unlock();
	}
}

void wlcore_event_max_tx_failure(struct cc33xx *wl, unsigned long sta_bitmap)
{
	cc33xx_debug(DEBUG_EVENT, "MAX_TX_FAILURE_EVENT_ID");
	wlcore_disconnect_sta(wl, sta_bitmap);
}

void wlcore_event_inactive_sta(struct cc33xx *wl, unsigned long sta_bitmap)
{
	cc33xx_debug(DEBUG_EVENT, "INACTIVE_STA_EVENT_ID");
	wlcore_disconnect_sta(wl, sta_bitmap);
}

void wlcore_event_roc_complete(struct cc33xx *wl)
{
	cc33xx_debug(DEBUG_EVENT, "REMAIN_ON_CHANNEL_COMPLETE_EVENT_ID");
	if (wl->roc_vif)
		ieee80211_ready_on_channel(wl->hw);
}

void wlcore_event_beacon_loss(struct cc33xx *wl, unsigned long roles_bitmap)
{
	/*
	 * We are HW_MONITOR device. On beacon loss - queue
	 * connection loss work. Cancel it on REGAINED event.
	 */
	struct cc33xx_vif *wlvif;
	struct ieee80211_vif *vif;
	int delay = wl->conf.host_conf.conn.synch_fail_thold *
				wl->conf.host_conf.conn.bss_lose_timeout;

	cc33xx_info("Beacon loss detected. roles:0x%lx", roles_bitmap);

	cc33xx_for_each_wlvif_sta(wl, wlvif) {
		if (wlvif->role_id == CC33XX_INVALID_ROLE_ID ||
		    !test_bit(wlvif->role_id , &roles_bitmap))
			continue;

		vif = cc33xx_wlvif_to_vif(wlvif);

		/* don't attempt roaming in case of p2p */
		if (wlvif->p2p) {
			ieee80211_connection_loss(vif);
			continue;
		}

		/*
		 * if the work is already queued, it should take place.
		 * We don't want to delay the connection loss
		 * indication any more.
		 */
		ieee80211_queue_delayed_work(wl->hw,
					     &wlvif->connection_loss_work,
					     msecs_to_jiffies(delay));

		ieee80211_cqm_beacon_loss_notify(vif, GFP_KERNEL);
	}
}

int cc33xx_event_unmask(struct cc33xx *wl)
{
	int ret;

	cc33xx_debug(DEBUG_EVENT, "unmasking event_mask 0x%x", wl->event_mask);
	ret = cc33xx_acx_event_mbox_mask(wl, ~(wl->event_mask));
	if (ret < 0)
		return ret;

	return 0;
}
