/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * This file is part of cc33xx
 *
 * Copyright (C) 1998-2009 Texas Instruments. All rights reserved.
 * Copyright (C) 2008-2009 Nokia Corporation
 *
 * Contact: Luciano Coelho <luciano.coelho@nokia.com>
 */

#ifndef __RX_H__
#define __RX_H__


/*
 * RX Descriptor flags:
 *
 * Bits 0-1 - band
 * Bit  2   - STBC
 * Bit  3   - A-MPDU
 * Bit  4   - HT
 * Bits 5-7 - encryption
 */
#define CC33XX_RX_DESC_BAND_MASK    0x03
#define CC33XX_RX_DESC_ENCRYPT_MASK 0xE0

#define CC33XX_RX_DESC_BAND_BG      0x00
#define CC33XX_RX_DESC_BAND_J       0x01
#define CC33XX_RX_DESC_BAND_A       0x02


/*
 * RX Descriptor status
 *
 * Bits 0-2 - error code
 * Bits 3-5 - process_id tag (AP mode FW)
 * Bits 6-7 - reserved
 */
enum {
	CC33XX_RX_DESC_SUCCESS		= 0x00,
	CC33XX_RX_DESC_DECRYPT_FAIL	= 0x01,
	CC33XX_RX_DESC_MIC_FAIL		= 0x02,
	CC33XX_RX_DESC_STATUS_MASK	= 0x07
};

/*
 * Account for the padding inserted by the FW in case of RX_ALIGNMENT
 * or for fixing alignment in case the packet wasn't aligned.
 */
#define RX_BUF_ALIGN                 2

/* Describes the alignment state of a Rx buffer */
enum wl_rx_buf_align {
	WLCORE_RX_BUF_ALIGNED,
	WLCORE_RX_BUF_UNALIGNED,
	WLCORE_RX_BUF_PADDED,
};

enum wl_rx_curr_status
{
	CURR_RX_START,
	CURR_RX_DROP,
	CURR_RX_DESC,
	CURR_RX_DATA
};

struct cc33xx_rx_descriptor {
	__le16 length;
	u8  header_alignment;
	u8  status;
	__le32 timestamp;

	u8  flags;
	u8  rate;
	u8  channel;
	s8  rssi;
	u8  snr;

	u8  hlid;
	u8  pad_len;
	u8  frame_format;
} __packed;

struct partial_rx_frame{
	struct sk_buff *skb;
	struct cc33xx_rx_descriptor desc;
	u16 handled_bytes;
	u16 original_bytes; /* including descriptor */
	enum wl_rx_curr_status status;
};


int wlcore_rx(struct cc33xx *wl, u8 *rx_buf_ptr, u16 rx_buf_len);
int cc33xx_rx_filter_enable(struct cc33xx *wl, int index, bool enable,
			    struct cc33xx_rx_filter *filter);
int cc33xx_rx_filter_clear_all(struct cc33xx *wl);


#endif /* __RX_H__ */
