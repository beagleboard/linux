/***
 *
 *  include/rtcfg.h
 *
 *  Real-Time Configuration Distribution Protocol
 *
 *  Copyright (C) 2004, 2005 Jan Kiszka <jan.kiszka@web.de>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef __RTCFG_H_
#define __RTCFG_H_

#include <rtnet_chrdev.h>

#define ERTCFG_START 0x0F00
#define ESTAGE1SIZE ERTCFG_START

#define FLAG_STAGE_2_DATA 0x0001
#define FLAG_READY 0x0002
#define FLAG_ASSIGN_ADDR_BY_MAC 0x0100

#define RTCFG_ADDR_MAC 0x00
#define RTCFG_ADDR_IP 0x01
#define RTCFG_ADDR_MASK 0xFF

typedef enum {
	RTCFG_CMD_SERVER,
	RTCFG_CMD_ADD,
	RTCFG_CMD_DEL,
	RTCFG_CMD_WAIT,
	RTCFG_CMD_CLIENT,
	RTCFG_CMD_ANNOUNCE,
	RTCFG_CMD_READY,
	RTCFG_CMD_DETACH,

	/* internal usage only */
	RTCFG_TIMER,
	RTCFG_FRM_STAGE_1_CFG,
	RTCFG_FRM_ANNOUNCE_NEW,
	RTCFG_FRM_ANNOUNCE_REPLY,
	RTCFG_FRM_STAGE_2_CFG,
	RTCFG_FRM_STAGE_2_CFG_FRAG,
	RTCFG_FRM_ACK_CFG,
	RTCFG_FRM_READY,
	RTCFG_FRM_HEARTBEAT,
	RTCFG_FRM_DEAD_STATION
} RTCFG_EVENT;

struct rtskb;
struct rtcfg_station;
struct rtcfg_connection;
struct rtcfg_file;

struct rtcfg_cmd {
	struct rtnet_ioctl_head head;

	union {
		struct {
			__u32 period;
			__u32 burstrate;
			__u32 heartbeat;
			__u32 threshold;
			__u32 flags;
		} server;

		struct {
			__u32 addr_type;
			__u32 ip_addr;
			__u8 mac_addr[DEV_ADDR_LEN];
			__u32 timeout;
			__u16 stage1_size;
			__u16 __padding;
			void *stage1_data;
			const char *stage2_filename;

			/* internal usage only */
			struct rtcfg_connection *conn_buf;
			struct rtcfg_file *stage2_file;
		} add;

		struct {
			__u32 addr_type;
			__u32 ip_addr;
			__u8 mac_addr[DEV_ADDR_LEN];

			/* internal usage only */
			struct rtcfg_connection *conn_buf;
			struct rtcfg_file *stage2_file;
		} del;

		struct {
			__u32 timeout;
		} wait;

		struct {
			__u32 timeout;
			__u32 max_stations;
			__u64 buffer_size;
			void *buffer;

			/* internal usage only */
			struct rtcfg_station *station_buf;
			struct rtskb *rtskb;
		} client;

		struct {
			__u32 timeout;
			__u32 flags;
			__u32 burstrate;
			__u32 __padding;
			__u64 buffer_size;
			void *buffer;

			/* internal usage only */
			struct rtskb *rtskb;
		} announce;

		struct {
			__u32 timeout;
		} ready;

		struct {
			/* internal usage only */
			struct rtcfg_connection *conn_buf;
			struct rtcfg_file *stage2_file;
			struct rtcfg_station *station_addr_list;
			struct rtskb *stage2_chain;
		} detach;

		__u64 __padding[16];
	} args;

	/* internal usage only */
	union {
		struct {
			int ifindex;
			RTCFG_EVENT event_id;
		} data;

		__u64 __padding[2];
	} internal;
};

#define RTCFG_IOC_SERVER                                                       \
	_IOW(RTNET_IOC_TYPE_RTCFG, RTCFG_CMD_SERVER, struct rtcfg_cmd)
#define RTCFG_IOC_ADD                                                          \
	_IOW(RTNET_IOC_TYPE_RTCFG, RTCFG_CMD_ADD, struct rtcfg_cmd)
#define RTCFG_IOC_DEL                                                          \
	_IOW(RTNET_IOC_TYPE_RTCFG, RTCFG_CMD_DEL, struct rtcfg_cmd)
#define RTCFG_IOC_WAIT                                                         \
	_IOW(RTNET_IOC_TYPE_RTCFG, RTCFG_CMD_WAIT, struct rtcfg_cmd)
#define RTCFG_IOC_CLIENT                                                       \
	_IOW(RTNET_IOC_TYPE_RTCFG, RTCFG_CMD_CLIENT, struct rtcfg_cmd)
#define RTCFG_IOC_ANNOUNCE                                                     \
	_IOW(RTNET_IOC_TYPE_RTCFG, RTCFG_CMD_ANNOUNCE, struct rtcfg_cmd)
#define RTCFG_IOC_READY                                                        \
	_IOW(RTNET_IOC_TYPE_RTCFG, RTCFG_CMD_READY, struct rtcfg_cmd)
#define RTCFG_IOC_DETACH                                                       \
	_IOW(RTNET_IOC_TYPE_RTCFG, RTCFG_CMD_DETACH, struct rtcfg_cmd)

#endif /* __RTCFG_H_ */
