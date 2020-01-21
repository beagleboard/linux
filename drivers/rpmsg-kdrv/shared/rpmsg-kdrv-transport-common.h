/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Subhajit Paul <subhajit_paul@ti.com>
 */

#ifndef __RPMSG_KDRV_TRANSPORT_COMMON_H__
#define __RPMSG_KDRV_TRANSPORT_COMMON_H__

/*
 * Device types supported by RPMSG-KDRV framework
 * Currently supported device types: display
 * Planned future support for capture and i2c devices
 */
#define RPMSG_KDRV_TP_DEVICE_TYPE_INIT		(0x0)
#define RPMSG_KDRV_TP_DEVICE_TYPE_DISPLAY	(0x1)
#define RPMSG_KDRV_TP_DEVICE_TYPE_DEMO		(0x2)
#define RPMSG_KDRV_TP_DEVICE_TYPE_ETHSWITCH	(0x3)
/* More device types here*/
#define RPMSG_KDRV_TP_DEVICE_TYPE_MAX		(0x4)

/*
 * Maximum number of proxy devices per remotecore
 */
#define RPMSG_KDRV_TP_MAX_DEVICES		(2)

/*
 * Maximum length of proxy device name
 */
#define RPMSG_KDRV_TP_DEVICE_NAME_LEN		(32)

/*
 * Statically assigned device ID for init device
 * Remote device framework dynamically assigns device
 * IDs for other devices. All dynamically assigned IDs
 * are greater than RPMSG_KDRV_TP_DEVICE_ID_INIT
 */
#define RPMSG_KDRV_TP_DEVICE_ID_INIT		(0)

/*
 * Packet IDs are assigned dynamically (for REQUEST packets)
 * starting from RPMSG_KDRV_TP_PACKET_ID_FIRST
 * For MESSAGE packets, framework can use RPMSG_KDRV_TP_PACKET_ID_NONE
 */
#define RPMSG_KDRV_TP_PACKET_ID_NONE		(0x10)
#define RPMSG_KDRV_TP_PACKET_ID_FIRST		(RPMSG_KDRV_TP_PACKET_ID_NONE + 1)

enum rpmsg_kdrv_packet_source {
	RPMSG_KDRV_TP_PACKET_SOURCE_SERVER,
	RPMSG_KDRV_TP_PACKET_SOURCE_CLIENT,
	RPMSG_KDRV_TP_PACKET_SOURCE_MAX,
};

enum rpmsg_kdrv_packet_type {
	RPMSG_KDRV_TP_PACKET_TYPE_REQUEST,
	RPMSG_KDRV_TP_PACKET_TYPE_RESPONSE,
	RPMSG_KDRV_TP_PACKET_TYPE_MESSAGE,
	RPMSG_KDRV_TP_PACKET_TYPE_MAX,
};

/*RPMSG_KDRV message :
 * => device_header
 * => message_header : defined by each device type
 * => request / response / message payload
 */
struct rpmsg_kdrv_device_header {
	/* ID of device sending the packet */
	u8 device_id;
	/* enum: rpmsg_kdrv_packet_type */
	u8 packet_type;
	/* enum: rpmsg_kdrv_packet_source */
	u8 packet_source;
	/* dynamically assigned packet ID for response matching */
	u32 packet_id;
	/* size of packet */
	u32 packet_size;
} __packed;

#endif
