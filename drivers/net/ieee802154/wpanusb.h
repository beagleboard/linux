/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Definitions shared between kernel and WPANUSB firmware
 *
 * Copyright (C) 2018 Intel Corp.
 *
 * Written by Andrei Emeltchenko <andrei.emeltchenko@intel.com>
 */

#define WPANUSB_VENDOR_ID	0x2fe3
#define WPANUSB_PRODUCT_ID	0x0101

#define BEAGLECONNECT_VENDOR_ID  0x2047
#define BEAGLECONNECT_PRODUCT_ID 0x0aa5

enum wpanusb_requests {
	RESET,
	TX,
	XMIT_ASYNC,
	ED,
	SET_CHANNEL,
	START,
	STOP,
	SET_SHORT_ADDR,
	SET_PAN_ID,
	SET_IEEE_ADDR,
	SET_TXPOWER,
	SET_CCA_MODE,
	SET_CCA_ED_LEVEL,
	SET_CSMA_PARAMS,
	SET_LBT,
	SET_FRAME_RETRIES,
	SET_PROMISCUOUS_MODE,
	GET_EXTENDED_ADDR,
	GET_SUPPORTED_CHANNELS,
};

struct set_channel {
	__u8 page;
	__u8 channel;
} __packed;

struct set_short_addr {
	__le16 short_addr;
} __packed;

struct set_pan_id {
	__le16 pan_id;
} __packed;

struct set_ieee_addr {
	__le64 ieee_addr;
} __packed;
