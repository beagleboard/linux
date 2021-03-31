/***
 *
 *	include/rtcfg/rtcfg_conn_event.h
 *
 *	Real-Time Configuration Distribution Protocol
 *
 *	Copyright (C) 2003-2005 Jan Kiszka <jan.kiszka@web.de>
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; either version 2 of the License, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program; if not, write to the Free Software
 *	Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef __RTCFG_CONN_EVENT_H_
#define __RTCFG_CONN_EVENT_H_

#include <linux/netdevice.h>

#include <rtcfg_chrdev.h>
#include <rtcfg/rtcfg_file.h>
#include <rtnet_internal.h>

typedef enum {
	RTCFG_CONN_SEARCHING,
	RTCFG_CONN_STAGE_1,
	RTCFG_CONN_STAGE_2,
	RTCFG_CONN_READY,
	RTCFG_CONN_DEAD
} RTCFG_CONN_STATE;

struct rtcfg_connection {
	struct list_head entry;
	int ifindex;
	RTCFG_CONN_STATE state;
	u8 mac_addr[MAX_ADDR_LEN];
	unsigned int addr_type;
	union {
#if IS_ENABLED(CONFIG_XENO_DRIVERS_NET_RTIPV4)
		u32 ip_addr;
#endif
	} addr;
	void *stage1_data;
	size_t stage1_size;
	struct rtcfg_file *stage2_file;
	u32 cfg_offs;
	unsigned int flags;
	unsigned int burstrate;
	nanosecs_abs_t last_frame;
	u64 cfg_timeout;
#ifdef CONFIG_XENO_OPT_VFILE
	struct xnvfile_regular proc_entry;
#endif
};

int rtcfg_do_conn_event(struct rtcfg_connection *conn, RTCFG_EVENT event_id,
			void *event_data);

#endif /* __RTCFG_CONN_EVENT_H_ */
