/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Subhajit Paul <subhajit_paul@ti.com>
 */

#ifndef __RPMSG_KDRV_TRANSPORT_H__
#define __RPMSG_KDRV_TRANSPORT_H__

#include "rpmsg-kdrv-transport-common.h"

enum rpmsg_kdrv_init_message_type {
	RPMSG_KDRV_TP_INIT_DEV_INFO_REQUEST,
	RPMSG_KDRV_TP_INIT_DEV_INFO_RESPONSE,
	RPMSG_KDRV_TP_INIT_CONNECT_MESSAGE,
	RPMSG_KDRV_TP_INIT_DISCONNECT_MESSAGE,
	RPMSG_KDRV_TP_INIT_MAX,
};

/*
 * message header for init device
 */
struct rpmsg_kdrv_init_message_header {
	/* enum: rpmsg_kdrv_init_message_type */
	u8 message_type;
} __packed;

/*
 * init device request to provide list of devices
 */
struct rpmsg_kdrv_init_dev_info_request {
	/* message header */
	struct rpmsg_kdrv_init_message_header header;
} __packed;

struct rpmsg_kdrv_init_device_info {
	/* device id */
	u8 device_id;
	/* device type (display, capture etc) */
	u8 device_type;
	/* name of device */
	u8 device_name[RPMSG_KDRV_TP_DEVICE_NAME_LEN];
	/* device specific info length */
	u16 device_data_len;
	/* per device-type info offset */
	u16 device_data_offset;
} __packed;

/*
 * init device response with list of devices
 */
struct rpmsg_kdrv_init_dev_info_response {
	/* message header */
	struct rpmsg_kdrv_init_message_header header;
	/*number of exported devices */
	u8 num_devices;
	/* list of exported devices */
	struct rpmsg_kdrv_init_device_info devices[RPMSG_KDRV_TP_MAX_DEVICES];
	/* device specific data */
	u8 device_data[0];
} __packed;

/*
 * init device per-device connect message
 */
struct rpmsg_kdrv_init_connect_message {
	/* message header */
	struct rpmsg_kdrv_init_message_header header;
	/* device ID to connect */
	u8 device_id;
} __packed;

/*
 * init device per-device disconnect message
 */
struct rpmsg_kdrv_init_disconnect_message {
	/* message header */
	struct rpmsg_kdrv_init_message_header header;
	/* device ID to disconnect */
	u8 device_id;
} __packed;

#endif
