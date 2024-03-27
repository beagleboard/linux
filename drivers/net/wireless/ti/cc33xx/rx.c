// SPDX-License-Identifier: GPL-2.0-only
/*
 * This file is part of cc33xx
 *
 * Copyright (C) 2009 Nokia Corporation
 *
 * Contact: Luciano Coelho <luciano.coelho@nokia.com>
 */

#include <linux/gfp.h>
#include <linux/sched.h>

#include "wlcore.h"
#include "debug.h"
#include "acx.h"
#include "rx.h"
#include "tx.h"
#include "io.h"





/* Construct the rx status structure for upper layers */
static void cc33xx_rx_status(struct cc33xx *wl,
			     struct cc33xx_rx_descriptor *desc,
			     struct ieee80211_rx_status *status,
			     u8 beacon, u8 probe_rsp)
{
	memset(status, 0, sizeof(struct ieee80211_rx_status));

	if ((desc->flags & CC33XX_RX_DESC_BAND_MASK) == CC33XX_RX_DESC_BAND_BG)
		status->band = NL80211_BAND_2GHZ;
	else if ((desc->flags & CC33XX_RX_DESC_BAND_MASK) == CC33XX_RX_DESC_BAND_J)
		status->band = NL80211_BAND_2GHZ;
	else if ((desc->flags & CC33XX_RX_DESC_BAND_MASK) == CC33XX_RX_DESC_BAND_A)
		status->band = NL80211_BAND_5GHZ;
	else
		status->band = NL80211_BAND_5GHZ; /* todo -Should be 6GHZ when added */


	status->rate_idx = wlcore_rate_to_idx(wl, desc->rate, status->band);


	if (desc->frame_format == CC33xx_VHT)
		status->encoding = RX_ENC_VHT;
	else if ((desc->frame_format == CC33xx_HT_MF) ||
		(desc->frame_format == CC33xx_HT_GF))
		status->encoding = RX_ENC_HT;
	else if ((desc->frame_format == CC33xx_B_SHORT) ||
		(desc->frame_format == CC33xx_B_LONG) ||
		(desc->frame_format == CC33xx_LEGACY_OFDM))
		status->encoding = RX_ENC_LEGACY;
	else
		status->encoding = RX_ENC_HE;

	/*
	* Read the signal level and antenna diversity indication.
	* The msb in the signal level is always set as it is a
	* negative number.
	* The antenna indication is the msb of the rssi.
	*/
	status->signal = ((desc->rssi & RSSI_LEVEL_BITMASK) | BIT(7));
	status->antenna = ((desc->rssi & ANT_DIVERSITY_BITMASK) >> 7);

	/*
	 * FIXME: In wl1251, the SNR should be divided by two.  In cc33xx we
	 * need to divide by two for now, but TI has been discussing about
	 * changing it.  This needs to be rechecked.
	 */
	 /*  is snr available? used?*/
	/*wl->noise = desc->rssi - (desc->snr >> 1);*/

	status->freq = ieee80211_channel_to_frequency(desc->channel,
						      status->band);

	if (desc->flags & CC33XX_RX_DESC_ENCRYPT_MASK) {
		u8 desc_err_code = desc->status & CC33XX_RX_DESC_STATUS_MASK;

		/* Frame is sent to driver with the IV (for PN replay check)
		 * but without the MIC */
		status->flag |=  RX_FLAG_MMIC_STRIPPED |
				 RX_FLAG_DECRYPTED |
				 RX_FLAG_MIC_STRIPPED;


		if (unlikely(desc_err_code & CC33XX_RX_DESC_MIC_FAIL)) {
			status->flag |= RX_FLAG_MMIC_ERROR;
			cc33xx_warning("Michael MIC error. Desc: 0x%x",
				       desc_err_code);
		}
	}

	if (beacon || probe_rsp)
		status->boottime_ns = ktime_get_boottime_ns();
	if (beacon)
		wlcore_set_pending_regdomain_ch(wl, (u16)desc->channel,
						status->band);
	status->nss = 1;
}


/* Copy part\ all of the descriptor. Allocate skb, or drop corrupted packet */
int wlcore_rx_getPacketDescriptor(struct cc33xx *wl, u8 *raw_buffer_ptr,
				  u16 *raw_buffer_len)
{
	u16 missing_desc_bytes;
	u16 available_desc_bytes;
	u16 pkt_data_len;
	struct sk_buff *skb;
	u16 prev_buffer_len = *raw_buffer_len;

	missing_desc_bytes = sizeof(struct cc33xx_rx_descriptor) -
				wl->partial_rx.handled_bytes;
	available_desc_bytes = min(*raw_buffer_len, missing_desc_bytes);
	memcpy(((u8 *)(&wl->partial_rx.desc))+wl->partial_rx.handled_bytes,
		raw_buffer_ptr,available_desc_bytes);

	/* If descriptor was not completed */
	if (available_desc_bytes != missing_desc_bytes) {
		wl->partial_rx.handled_bytes += *raw_buffer_len;
		wl->partial_rx.status = CURR_RX_DESC;
		*raw_buffer_len = 0;
		goto out;
	} else {
		wl->partial_rx.handled_bytes += available_desc_bytes;
		*raw_buffer_len -= available_desc_bytes;
	}

	/* Descriptor was fully copied */
	pkt_data_len = wl->partial_rx.original_bytes -
			sizeof(struct cc33xx_rx_descriptor);


	if (unlikely(wl->partial_rx.desc.status & CC33XX_RX_DESC_DECRYPT_FAIL)) {
		cc33xx_warning("corrupted packet in RX: status: 0x%x len: %d",
			wl->partial_rx.desc.status & CC33XX_RX_DESC_STATUS_MASK,
			pkt_data_len);

		/* If frame can be fully dropped */
		if (pkt_data_len <= *raw_buffer_len) {
			*raw_buffer_len -=  pkt_data_len;
			wl->partial_rx.status = CURR_RX_START;
		}
		else {
			wl->partial_rx.handled_bytes += *raw_buffer_len;
			wl->partial_rx.status = CURR_RX_DROP;
			*raw_buffer_len = 0;
		}
		goto out;
	}


	skb = __dev_alloc_skb(pkt_data_len , GFP_KERNEL);
	if (!skb) {
		cc33xx_error("Couldn't allocate RX frame");
		/* If frame can be fully dropped */
		if (pkt_data_len <= *raw_buffer_len) {
			*raw_buffer_len -=  pkt_data_len;
			wl->partial_rx.status = CURR_RX_START;
		} else {
		/* Dropped partial frame */
			wl->partial_rx.handled_bytes += *raw_buffer_len;
			wl->partial_rx.status = CURR_RX_DROP;
			*raw_buffer_len = 0;
		}
		goto out;
	}

	wl->partial_rx.skb = skb;
	wl->partial_rx.status = CURR_RX_DATA;

	out:
	/* Function return the amount of consumed bytes */
	return (prev_buffer_len - *raw_buffer_len);
}

/* Copy part or all of the packet's data. push skb to queue if possible */
int wlcore_rx_getPacketData(struct cc33xx *wl, u8 *raw_buffer_ptr,
			    u16 *raw_buffer_len)
{
	u16 missing_data_bytes;
	u16 available_data_bytes;
	u32 defer_count;
	enum wl_rx_buf_align rx_align;
	u16 extra_bytes;
	struct ieee80211_hdr *hdr;
	u8 beacon = 0;
	u8 is_probe_resp = 0;
	u8 is_data = 0;
	u16 seq_num;
	u16 prev_buffer_len = *raw_buffer_len;

	cc33xx_debug(DEBUG_RX, "current rx data: original bytes: %d, "
		"handled bytes %d, desc pad len %d, missing_data_bytes %d",
		wl->partial_rx.original_bytes, wl->partial_rx.handled_bytes,
		wl->partial_rx.desc.pad_len,missing_data_bytes);

	missing_data_bytes = wl->partial_rx.original_bytes -
				wl->partial_rx.handled_bytes;
	available_data_bytes = min(missing_data_bytes,*raw_buffer_len);

	skb_put_data(wl->partial_rx.skb, raw_buffer_ptr, available_data_bytes);

	/* Check if we didn't manage to copy the entire packet - got out,
	* continue next time */
	if (available_data_bytes != missing_data_bytes) {
	wl->partial_rx.handled_bytes += *raw_buffer_len;
	wl->partial_rx.status = CURR_RX_DATA;
	*raw_buffer_len = 0;
	goto out;
	}
	else {
	*raw_buffer_len -=  available_data_bytes;
	}

	/* Data fully copied */

	rx_align = wl->partial_rx.desc.header_alignment;
	if (rx_align == WLCORE_RX_BUF_PADDED)
		skb_pull(wl->partial_rx.skb, RX_BUF_ALIGN);


	extra_bytes = wl->partial_rx.desc.pad_len;
	if (extra_bytes != 0)
		skb_trim(wl->partial_rx.skb, wl->partial_rx.skb->len - extra_bytes);

	hdr = (struct ieee80211_hdr *)wl->partial_rx.skb->data;

	if (ieee80211_is_beacon(hdr->frame_control))
		beacon = 1;
	if (ieee80211_is_data_present(hdr->frame_control))
		is_data = 1;
	if (ieee80211_is_probe_resp(hdr->frame_control))
		is_probe_resp = 1;


	cc33xx_rx_status(wl, &wl->partial_rx.desc,
		     IEEE80211_SKB_RXCB(wl->partial_rx.skb),
		     beacon, is_probe_resp);


	seq_num = (le16_to_cpu(hdr->seq_ctrl) & IEEE80211_SCTL_SEQ) >> 4;
	cc33xx_debug(DEBUG_RX, "rx skb 0x%p: %d B %s seq %d link id %d",
			wl->partial_rx.skb,
			wl->partial_rx.skb->len - wl->partial_rx.desc.pad_len,
			beacon ? "beacon" : "", seq_num, wl->partial_rx.desc.hlid);

	cc33xx_debug(DEBUG_RX, "rx frame. frame type 0x%x, frame length 0x%x, "
			"frame address 0x%lx",hdr->frame_control,
			wl->partial_rx.skb->len,
			(unsigned long)wl->partial_rx.skb->data);

	/* Adding frame to queue */
	skb_queue_tail(&wl->deferred_rx_queue, wl->partial_rx.skb);
	wl->rx_counter++;
	wl->partial_rx.status = CURR_RX_START;

	/* Make sure the deferred queues don't get too long */
	defer_count = skb_queue_len(&wl->deferred_tx_queue) +
			skb_queue_len(&wl->deferred_rx_queue);
	if (defer_count >= CC33XX_RX_QUEUE_MAX_LEN)
		cc33xx_flush_deferred_work(wl);
	else
		queue_work(wl->freezable_netstack_wq, &wl->netstack_work);

out:
	return (prev_buffer_len - *raw_buffer_len);
}

int wlcore_rx_dropPacketData(struct cc33xx *wl, u8 *raw_buffer_ptr,
			     u16 *raw_buffer_len)
{

	u16 prev_buffer_len = *raw_buffer_len;

	/* Can we drop the entire frame ? */
	if (*raw_buffer_len >=
		(wl->partial_rx.original_bytes - wl->partial_rx.handled_bytes)){
		*raw_buffer_len -= wl->partial_rx.original_bytes -
				wl->partial_rx.handled_bytes;
		wl->partial_rx.handled_bytes = 0;
		wl->partial_rx.status = CURR_RX_START;
	} else {
		wl->partial_rx.handled_bytes += *raw_buffer_len;
		*raw_buffer_len = 0;
	}

	return (prev_buffer_len - *raw_buffer_len);
}

/* Handle single packet from the RX buffer. We don't have to be aligned to
 * packet boundary (buffer may start \ end in the middle of packet) */
static void cc33xx_rx_handle_packet(struct cc33xx *wl, u8 *raw_buffer_ptr,
				    u16 *raw_buffer_len)
{
	struct cc33xx_rx_descriptor *desc;
	u16 consumedBytes;


	if (CURR_RX_START == wl->partial_rx.status) {
		BUG_ON(*raw_buffer_len < 2);
		desc = (struct cc33xx_rx_descriptor *)raw_buffer_ptr;
		wl->partial_rx.original_bytes = desc->length;
		wl->partial_rx.handled_bytes = 0;
		wl->partial_rx.status = CURR_RX_DESC;

		cc33xx_debug(DEBUG_RX,
			"rx frame. desc length 0x%x, alignment 0x%x, padding 0x%x",
			desc->length, desc->header_alignment, desc->pad_len);
	}


	/* start \ continue copy descriptor */
	if (CURR_RX_DESC == wl->partial_rx.status) {
		consumedBytes = wlcore_rx_getPacketDescriptor(wl, raw_buffer_ptr,
							raw_buffer_len);
		raw_buffer_ptr += consumedBytes;
	}

	/* Check if we are in the middle of dropped packet */
	if (unlikely(CURR_RX_DROP == wl->partial_rx.status)){
		consumedBytes = wlcore_rx_dropPacketData(wl, raw_buffer_ptr,
							raw_buffer_len);
		raw_buffer_ptr += consumedBytes;
	}

	/* start \ continue copy descriptor */
	if (CURR_RX_DATA == wl->partial_rx.status) {
		consumedBytes = wlcore_rx_getPacketData(wl, raw_buffer_ptr,
							raw_buffer_len);
		raw_buffer_ptr += consumedBytes;
	}
}


/*
 * It is assumed that SDIO buffer was read prior to this function (data buffer
 * is read along with the status). The RX function gets pointer to the RX data
 * and its length. This buffer may contain unknown number of packets, separated
 * by hif descriptor and 0-3 bytes padding if required.
 * The last packet may be truncated in the middle, and should be saved for next
 * iteration.
 */
int wlcore_rx(struct cc33xx *wl, u8 *rx_buf_ptr, u16 rx_buf_len)
{
	u16 local_rx_buffer_len = rx_buf_len;
	u16 pkt_offset = 0;
	u16 consumed_bytes;
	u16 prev_rx_buf_len;


	/* Split data into separate packets */
	while (local_rx_buffer_len > 0) {
		cc33xx_debug(DEBUG_RX,"start loop. buffer length %d" ,
			local_rx_buffer_len);


		/*
		* the handle data call can only fail in memory-outage
		* conditions, in that case the received frame will just
		* be dropped.
		*/
		prev_rx_buf_len = local_rx_buffer_len;
		cc33xx_rx_handle_packet (wl,
					rx_buf_ptr + pkt_offset,
					&local_rx_buffer_len);
		consumed_bytes = prev_rx_buf_len - local_rx_buffer_len;

		pkt_offset +=  consumed_bytes;

		cc33xx_debug(DEBUG_RX,"end rx loop. buffer length %d, "
			"packet counter %d, current packet status %d" ,
			local_rx_buffer_len, wl->rx_counter, wl->partial_rx.status);
	}

	return(0);
}

#ifdef CONFIG_PM
int cc33xx_rx_filter_enable(struct cc33xx *wl,
		int index, bool enable,
		struct cc33xx_rx_filter *filter)
{
	int ret;

	if (!!test_bit(index, wl->rx_filter_enabled) == enable) {
		cc33xx_warning("Request to enable an already "
			"enabled rx filter %d", index);
		return 0;
	}

	ret = cc33xx_acx_set_rx_filter(wl, index, enable, filter);

	if (ret) {
		cc33xx_error("Failed to %s rx data filter %d (err=%d)",
			enable ? "enable" : "disable", index, ret);
		return ret;
	}

	if (enable)
		__set_bit(index, wl->rx_filter_enabled);
	else
		__clear_bit(index, wl->rx_filter_enabled);

	return 0;
}

int cc33xx_rx_filter_clear_all(struct cc33xx *wl)
{
	int i, ret = 0;

	for (i = 0; i < CC33XX_MAX_RX_FILTERS; i++) {
		if (!test_bit(i, wl->rx_filter_enabled))
		continue;
		ret = cc33xx_rx_filter_enable(wl, i, 0, NULL);
		if (ret)
			goto out;
	}

out:
	return ret;
}
#endif /* CONFIG_PM */
