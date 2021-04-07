/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Subhajit Paul <subhajit_paul@ti.com>
 */

#ifndef __RPMSG_KDRV_TRANSPORT_DEMODEV_H__
#define __RPMSG_KDRV_TRANSPORT_DEMODEV_H__

#include "rpmsg-kdrv-transport-common.h"

enum rpmsg_kdrv_display_message_type {
	RPMSG_KDRV_TP_DEMODEV_PING_REQUEST,
	RPMSG_KDRV_TP_DEMODEV_PING_RESPONSE,
	RPMSG_KDRV_TP_DEMODEV_S2C_MESSAGE,
	RPMSG_KDRV_TP_DEMODEV_C2S_MESSAGE,
	RPMSG_KDRV_TP_DEMODEV_MAX,
};

/*
 * Maximum length of demo device data
 */
#define RPMSG_KDRV_TP_DEMODEV_DEVICE_DATA_LEN	(32)

/*
 * Maximum length of demo device message data
 */
#define RPMSG_KDRV_TP_DEMODEV_MESSAGE_DATA_LEN	(128)

/*
 * per-device data for demo device
 */
struct rpmsg_kdrv_demodev_device_data {
	/* Does the device send all vsyncs? */
	u8 charString[RPMSG_KDRV_TP_DEMODEV_DEVICE_DATA_LEN];
} __packed;

/*
 * message header for demo device
 */
struct rpmsg_kdrv_demodev_message_header {
	/* enum: rpmsg_kdrv_demodev_message_type */
	u8 message_type;
} __packed;

/* demo device ping request - always client to server */
struct rpmsg_kdrv_demodev_ping_request {
	/* message header */
	struct rpmsg_kdrv_demodev_message_header header;
	/* ping data */
	u8 data[RPMSG_KDRV_TP_DEMODEV_MESSAGE_DATA_LEN];
} __packed;

/* demo device ping response - always server to client */
struct rpmsg_kdrv_demodev_ping_response {
	/* message header */
	struct rpmsg_kdrv_demodev_message_header header;
	/* ping data */
	u8 data[RPMSG_KDRV_TP_DEMODEV_MESSAGE_DATA_LEN];
} __packed;

/* demo device server to client one-way message */
struct rpmsg_kdrv_demodev_s2c_message {
	/* message header */
	struct rpmsg_kdrv_demodev_message_header header;
	/* message data */
	u8 data[RPMSG_KDRV_TP_DEMODEV_MESSAGE_DATA_LEN];
} __packed;

/* demo device client to server one-way message */
struct rpmsg_kdrv_demodev_c2s_message {
	/* message header */
	struct rpmsg_kdrv_demodev_message_header header;
	/* message data */
	u8 data[RPMSG_KDRV_TP_DEMODEV_MESSAGE_DATA_LEN];
} __packed;

#endif
