/***
 *
 *  include/rtcfg/rtcfg_frame.h
 *
 *  Real-Time Configuration Distribution Protocol
 *
 *  Copyright (C) 2003 Jan Kiszka <jan.kiszka@web.de>
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

#ifndef __RTCFG_FRAME_H_
#define __RTCFG_FRAME_H_

#include <linux/init.h>
#include <linux/if_packet.h>
#include <asm/byteorder.h>

#include <rtcfg/rtcfg_event.h>


#define ETH_RTCFG                   0x9022

#define RTCFG_SKB_PRIO \
    RTSKB_PRIO_VALUE(QUEUE_MIN_PRIO-1, RTSKB_DEF_NRT_CHANNEL)

#define RTCFG_ID_STAGE_1_CFG        0
#define RTCFG_ID_ANNOUNCE_NEW       1
#define RTCFG_ID_ANNOUNCE_REPLY     2
#define RTCFG_ID_STAGE_2_CFG        3
#define RTCFG_ID_STAGE_2_CFG_FRAG   4
#define RTCFG_ID_ACK_CFG            5
#define RTCFG_ID_READY              6
#define RTCFG_ID_HEARTBEAT          7
#define RTCFG_ID_DEAD_STATION       8

#define RTCFG_ADDRSIZE_MAC          0
#define RTCFG_ADDRSIZE_IP           4
#define RTCFG_MAX_ADDRSIZE          RTCFG_ADDRSIZE_IP

#define RTCFG_FLAG_STAGE_2_DATA 0
#define RTCFG_FLAG_READY        1

#define _RTCFG_FLAG_STAGE_2_DATA (1 << RTCFG_FLAG_STAGE_2_DATA)
#define _RTCFG_FLAG_READY        (1 << RTCFG_FLAG_READY)

struct rtcfg_frm_head {
#if defined(__LITTLE_ENDIAN_BITFIELD)
    u8 id:5;
    u8 version:3;
#elif defined(__BIG_ENDIAN_BITFIELD)
    u8 version:3;
    u8 id:5;
#else
    #error unsupported byte order
#endif
} __attribute__((packed));

struct rtcfg_frm_stage_1_cfg {
    struct rtcfg_frm_head head;
    u8                    addr_type;
    u8                    client_addr[0];
    u8                    server_addr[0];
    u8                    burstrate;
    u16                   cfg_len;
    u8                    cfg_data[0];
} __attribute__((packed));

struct rtcfg_frm_announce {
    struct rtcfg_frm_head head;
    u8                    addr_type;
    u8                    addr[0];
    u8                    flags;
    u8                    burstrate;
} __attribute__((packed));

struct rtcfg_frm_stage_2_cfg {
    struct rtcfg_frm_head head;
    u8                    flags;
    u32                   stations;
    u16                   heartbeat_period;
    u32                   cfg_len;
    u8                    cfg_data[0];
} __attribute__((packed));

struct rtcfg_frm_stage_2_cfg_frag {
    struct rtcfg_frm_head head;
    u32                   frag_offs;
    u8                    cfg_data[0];
} __attribute__((packed));

struct rtcfg_frm_ack_cfg {
    struct rtcfg_frm_head head;
    u32                   ack_len;
} __attribute__((packed));

struct rtcfg_frm_simple {
    struct rtcfg_frm_head head;
} __attribute__((packed));

struct rtcfg_frm_dead_station {
    struct rtcfg_frm_head head;
    u8                    addr_type;
    u8                    logical_addr[0];
    u8                    physical_addr[32];
} __attribute__((packed));


int rtcfg_send_stage_1(struct rtcfg_connection *conn);
int rtcfg_send_stage_2(struct rtcfg_connection *conn, int send_data);
int rtcfg_send_stage_2_frag(struct rtcfg_connection *conn);
int rtcfg_send_announce_new(int ifindex);
int rtcfg_send_announce_reply(int ifindex, u8 *dest_mac_addr);
int rtcfg_send_ack(int ifindex);
int rtcfg_send_dead_station(struct rtcfg_connection *conn);

int rtcfg_send_simple_frame(int ifindex, int frame_id, u8 *dest_addr);

#define rtcfg_send_ready(ifindex)                                   \
    rtcfg_send_simple_frame(ifindex, RTCFG_ID_READY, NULL)
#define rtcfg_send_heartbeat(ifindex)                               \
    rtcfg_send_simple_frame(ifindex, RTCFG_ID_HEARTBEAT,            \
                            device[ifindex].spec.clt.srv_mac_addr)

int __init rtcfg_init_frames(void);
void rtcfg_cleanup_frames(void);

#endif /* __RTCFG_FRAME_H_ */
