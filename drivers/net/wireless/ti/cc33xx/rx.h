/* SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (C) 2022-2024 Texas Instruments Incorporated - https://www.ti.com/
 */

#ifndef __RX_H__
#define __RX_H__

struct cc33xx;
struct cc33xx_rx_filter;

/* RX Descriptor flags:
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

/* RX Descriptor status
 *
 * Bits 0-2 - error code
 * Bits 3-5 - process_id tag (AP mode FW)
 * Bits 6   - new key
 * Bits 7   - reserved
 */
enum {
	CC33XX_RX_DESC_SUCCESS		= 0x00,
	CC33XX_RX_DESC_DECRYPT_FAIL	= 0x01,
	CC33XX_RX_DESC_MIC_FAIL		= 0x02,
	CC33XX_RX_DESC_STATUS_MASK	= 0x07,
	CC33XX_RX_DESC_NEW_KEY		= 0x40
};

/* Account for the padding inserted by the FW in case of RX_ALIGNMENT
 * or for fixing alignment in case the packet wasn't aligned.
 */
#define RX_BUF_ALIGN                 2

/* Describes the alignment state of a Rx buffer */
enum cc33xx_rx_buf_align {
	CC33XX_RX_BUF_ALIGNED,
	CC33XX_RX_BUF_UNALIGNED,
	CC33XX_RX_BUF_PADDED,
};

enum cc33xx_rx_curr_status {
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

struct partial_rx_frame {
	struct sk_buff *skb;
	struct cc33xx_rx_descriptor desc;
	u16 handled_bytes;
	u16 original_bytes; /* including descriptor */
	enum cc33xx_rx_curr_status status;
};

int cc33xx_rx(struct cc33xx *cc, u8 *rx_buf_ptr, u16 rx_buf_len);
int cc33xx_rx_filter_enable(struct cc33xx *cc, int index, bool enable,
			    struct cc33xx_rx_filter *filter);
int cc33xx_rx_filter_clear_all(struct cc33xx *cc);
void cc33xx_flush_deferred_work(struct cc33xx *cc);
void cc33xx_free_wowlan_patterns_memory(struct cc33xx *cc);
int cc33xx_rx_filter_alloc_field(struct cc33xx_rx_filter *filter, u16 offset,
				 u8 flags, const u8 *pattern, u8 len);
struct cc33xx_rx_filter *cc33xx_rx_filter_alloc(void);
int cc33xx_clear_wowlan_search_patterns(struct cc33xx *cc);
int cc33xx_set_wowlan_search_mode(struct cc33xx *cc, bool enable);
void cc33xx_rx_filter_free(struct cc33xx_rx_filter *filter);
int cc33xx_get_wowlan_search_filters(struct cc33xx *cc, struct cc33xx_rx_filter ***filters_out);
int cc33xx_parse_wowlan_hex_string(const char *input, u8 *pattern, u8 *mask,
				   size_t max_len, bool *has_mask);
int cc33xx_parse_wowlan_search_pattern(char *input,
					u8 *header_bytes, int *header_len,
					u8 *header_mask, bool *header_has_mask,
					u8 *payload_bytes, int *payload_len,
					u8 *payload_mask, bool *payload_has_mask,
					u16 *payload_offsett, bool *search_mode);
int cc33xx_is_wowlan_search_enabled(struct cc33xx *cc);
int cc33xx_add_wowlan_search_pattern(struct cc33xx *cc, char *input);

#endif /* __RX_H__ */
