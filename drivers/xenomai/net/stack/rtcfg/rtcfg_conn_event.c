/***
 *
 *  rtcfg/rtcfg_conn_event.c
 *
 *  Real-Time Configuration Distribution Protocol
 *
 *  Copyright (C) 2003-2005 Jan Kiszka <jan.kiszka@web.de>
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

#include <linux/kernel.h>

#include <ipv4/route.h>
#include <rtcfg/rtcfg.h>
#include <rtcfg/rtcfg_conn_event.h>
#include <rtcfg/rtcfg_event.h>
#include <rtcfg/rtcfg_frame.h>


/****************************** states ***************************************/
static int rtcfg_conn_state_searching(
    struct rtcfg_connection *conn, RTCFG_EVENT event_id, void* event_data);
static int rtcfg_conn_state_stage_1(
    struct rtcfg_connection *conn, RTCFG_EVENT event_id, void* event_data);
static int rtcfg_conn_state_stage_2(
    struct rtcfg_connection *conn, RTCFG_EVENT event_id, void* event_data);
static int rtcfg_conn_state_ready(
    struct rtcfg_connection *conn, RTCFG_EVENT event_id, void* event_data);
static int rtcfg_conn_state_dead(
    struct rtcfg_connection *conn, RTCFG_EVENT event_id, void* event_data);


#ifdef CONFIG_XENO_DRIVERS_NET_RTCFG_DEBUG
const char *rtcfg_conn_state[] = {
    "RTCFG_CONN_SEARCHING",
    "RTCFG_CONN_STAGE_1",
    "RTCFG_CONN_STAGE_2",
    "RTCFG_CONN_READY",
    "RTCFG_CONN_DEAD"
};
#endif /* CONFIG_XENO_DRIVERS_NET_RTCFG_DEBUG */


static void rtcfg_conn_recv_announce_new(struct rtcfg_connection *conn,
                                         struct rtskb *rtskb);
static void rtcfg_conn_check_cfg_timeout(struct rtcfg_connection *conn);
static void rtcfg_conn_check_heartbeat(struct rtcfg_connection *conn);



static int (*state[])(struct rtcfg_connection *conn, RTCFG_EVENT event_id,
                      void* event_data) =
{
    rtcfg_conn_state_searching,
    rtcfg_conn_state_stage_1,
    rtcfg_conn_state_stage_2,
    rtcfg_conn_state_ready,
    rtcfg_conn_state_dead
};



int rtcfg_do_conn_event(struct rtcfg_connection *conn, RTCFG_EVENT event_id,
                        void* event_data)
{
    int conn_state = conn->state;


    RTCFG_DEBUG(3, "RTcfg: %s() conn=%p, event=%s, state=%s\n", __FUNCTION__,
                conn, rtcfg_event[event_id], rtcfg_conn_state[conn_state]);

    return (*state[conn_state])(conn, event_id, event_data);
}



static void rtcfg_next_conn_state(struct rtcfg_connection *conn,
                                  RTCFG_CONN_STATE state)
{
    RTCFG_DEBUG(4, "RTcfg: next connection state=%s \n",
                rtcfg_conn_state[state]);

    conn->state = state;
}



static int rtcfg_conn_state_searching(struct rtcfg_connection *conn,
                                      RTCFG_EVENT event_id, void* event_data)
{
    struct rtcfg_device *rtcfg_dev = &device[conn->ifindex];
    struct rtskb        *rtskb = (struct rtskb *)event_data;


    switch (event_id) {
        case RTCFG_FRM_ANNOUNCE_NEW:
            rtcfg_conn_recv_announce_new(conn, rtskb);
            break;

        case RTCFG_FRM_ANNOUNCE_REPLY:
            conn->last_frame = rtskb->time_stamp;

            rtcfg_next_conn_state(conn, RTCFG_CONN_READY);

            rtcfg_dev->stations_found++;
            rtcfg_dev->stations_ready++;
            rtcfg_dev->spec.srv.clients_configured++;
            if (rtcfg_dev->spec.srv.clients_configured ==
                rtcfg_dev->other_stations)
                rtcfg_complete_cmd(conn->ifindex, RTCFG_CMD_WAIT, 0);

            break;

        default:
            RTCFG_DEBUG(1, "RTcfg: unknown event %s for conn %p in %s()\n",
                        rtcfg_event[event_id], conn, __FUNCTION__);
            return -EINVAL;
    }
    return 0;
}



static int rtcfg_conn_state_stage_1(struct rtcfg_connection *conn,
                                    RTCFG_EVENT event_id, void* event_data)
{
    struct rtskb             *rtskb     = (struct rtskb *)event_data;
    struct rtcfg_device      *rtcfg_dev = &device[conn->ifindex];
    struct rtcfg_frm_ack_cfg *ack_cfg;
    int                      packets;


    switch (event_id) {
        case RTCFG_FRM_ACK_CFG:
            conn->last_frame = rtskb->time_stamp;

            ack_cfg = (struct rtcfg_frm_ack_cfg *)rtskb->data;
            conn->cfg_offs = ntohl(ack_cfg->ack_len);

            if ((conn->flags & _RTCFG_FLAG_STAGE_2_DATA) != 0) {
                if (conn->cfg_offs >= conn->stage2_file->size) {
                    rtcfg_dev->spec.srv.clients_configured++;
                    if (rtcfg_dev->spec.srv.clients_configured ==
                        rtcfg_dev->other_stations)
                        rtcfg_complete_cmd(conn->ifindex, RTCFG_CMD_WAIT, 0);
                    rtcfg_next_conn_state(conn,
                        ((conn->flags & _RTCFG_FLAG_READY) != 0) ?
                        RTCFG_CONN_READY : RTCFG_CONN_STAGE_2);
                } else {
                    packets = conn->burstrate;
                    while ((conn->cfg_offs < conn->stage2_file->size) &&
                        (packets > 0)) {
                        rtcfg_send_stage_2_frag(conn);
                        packets--;
                    }
                }
            } else {
                rtcfg_dev->spec.srv.clients_configured++;
                if (rtcfg_dev->spec.srv.clients_configured ==
                    rtcfg_dev->other_stations)
                    rtcfg_complete_cmd(conn->ifindex, RTCFG_CMD_WAIT, 0);
                rtcfg_next_conn_state(conn,
                    ((conn->flags & _RTCFG_FLAG_READY) != 0) ?
                    RTCFG_CONN_READY : RTCFG_CONN_STAGE_2);
            }

            break;

        case RTCFG_TIMER:
            rtcfg_conn_check_cfg_timeout(conn);
            break;

        default:
            RTCFG_DEBUG(1, "RTcfg: unknown event %s for conn %p in %s()\n",
                        rtcfg_event[event_id], conn, __FUNCTION__);
            return -EINVAL;
    }
    return 0;
}



static int rtcfg_conn_state_stage_2(struct rtcfg_connection *conn,
                                    RTCFG_EVENT event_id, void* event_data)
{
    struct rtskb        *rtskb = (struct rtskb *)event_data;
    struct rtcfg_device *rtcfg_dev = &device[conn->ifindex];


    switch (event_id) {
        case RTCFG_FRM_READY:
            conn->last_frame = rtskb->time_stamp;

            rtcfg_next_conn_state(conn, RTCFG_CONN_READY);

            conn->flags |= _RTCFG_FLAG_READY;
            rtcfg_dev->stations_ready++;

            if (rtcfg_dev->stations_ready == rtcfg_dev->other_stations)
                rtcfg_complete_cmd(conn->ifindex, RTCFG_CMD_READY, 0);

            break;

        case RTCFG_TIMER:
            rtcfg_conn_check_cfg_timeout(conn);
            break;

        default:
            RTCFG_DEBUG(1, "RTcfg: unknown event %s for conn %p in %s()\n",
                        rtcfg_event[event_id], conn, __FUNCTION__);
            return -EINVAL;
    }
    return 0;
}



static int rtcfg_conn_state_ready(struct rtcfg_connection *conn,
                                  RTCFG_EVENT event_id, void* event_data)
{
    struct rtskb *rtskb = (struct rtskb *)event_data;


    switch (event_id) {
        case RTCFG_TIMER:
            rtcfg_conn_check_heartbeat(conn);
            break;

        case RTCFG_FRM_HEARTBEAT:
            conn->last_frame = rtskb->time_stamp;
            break;

        default:
            RTCFG_DEBUG(1, "RTcfg: unknown event %s for conn %p in %s()\n",
                        rtcfg_event[event_id], conn, __FUNCTION__);
            return -EINVAL;
    }
    return 0;
}



static int rtcfg_conn_state_dead(struct rtcfg_connection *conn,
                                 RTCFG_EVENT event_id, void* event_data)
{
    switch (event_id) {
        case RTCFG_FRM_ANNOUNCE_NEW:
            rtcfg_conn_recv_announce_new(conn, (struct rtskb *)event_data);
            break;

        case RTCFG_FRM_ANNOUNCE_REPLY:
            /* Spec to-do: signal station that it is assumed to be dead
               (=> reboot command?) */

        default:
            RTCFG_DEBUG(1, "RTcfg: unknown event %s for conn %p in %s()\n",
                        rtcfg_event[event_id], conn, __FUNCTION__);
            return -EINVAL;
    }
    return 0;
}



static void rtcfg_conn_recv_announce_new(struct rtcfg_connection *conn,
                                         struct rtskb *rtskb)
{
    struct rtcfg_device       *rtcfg_dev = &device[conn->ifindex];
    struct rtcfg_frm_announce *announce_new;
    int                       packets;


    conn->last_frame = rtskb->time_stamp;

    announce_new = (struct rtcfg_frm_announce *)rtskb->data;

    conn->flags = announce_new->flags;
    if (announce_new->burstrate < conn->burstrate)
        conn->burstrate = announce_new->burstrate;

    rtcfg_next_conn_state(conn, RTCFG_CONN_STAGE_1);

    rtcfg_dev->stations_found++;
    if ((conn->flags & _RTCFG_FLAG_READY) != 0)
        rtcfg_dev->stations_ready++;

    if (((conn->flags & _RTCFG_FLAG_STAGE_2_DATA) != 0) &&
        (conn->stage2_file != NULL)) {
        packets = conn->burstrate - 1;

        rtcfg_send_stage_2(conn, 1);

        while ((conn->cfg_offs < conn->stage2_file->size) &&
            (packets > 0)) {
            rtcfg_send_stage_2_frag(conn);
            packets--;
        }
    } else {
        rtcfg_send_stage_2(conn, 0);
        conn->flags &= ~_RTCFG_FLAG_STAGE_2_DATA;
    }
}



static void rtcfg_conn_check_cfg_timeout(struct rtcfg_connection *conn)
{
    struct rtcfg_device *rtcfg_dev;


    if (!conn->cfg_timeout)
        return;

    if (rtdm_clock_read() >= conn->last_frame + conn->cfg_timeout) {
        rtcfg_dev = &device[conn->ifindex];

        rtcfg_dev->stations_found--;
        if (conn->state == RTCFG_CONN_STAGE_2)
            rtcfg_dev->spec.srv.clients_configured--;

        rtcfg_next_conn_state(conn, RTCFG_CONN_SEARCHING);
        conn->cfg_offs = 0;
        conn->flags    = 0;

#if IS_ENABLED(CONFIG_XENO_DRIVERS_NET_RTIPV4)
        if (conn->addr_type == RTCFG_ADDR_IP) {
            struct rtnet_device *rtdev;

            /* MAC address yet unknown -> use broadcast address */
            rtdev = rtdev_get_by_index(conn->ifindex);
            if (rtdev == NULL)
                return;
            memcpy(conn->mac_addr, rtdev->broadcast, MAX_ADDR_LEN);
            rtdev_dereference(rtdev);
        }
#endif /* CONFIG_XENO_DRIVERS_NET_RTIPV4 */
    }
}



static void rtcfg_conn_check_heartbeat(struct rtcfg_connection *conn)
{
    u64                 timeout;
    struct rtcfg_device *rtcfg_dev;


    timeout = device[conn->ifindex].spec.srv.heartbeat_timeout;
    if (!timeout)
        return;

    if (rtdm_clock_read() >= conn->last_frame + timeout) {
        rtcfg_dev = &device[conn->ifindex];

        rtcfg_dev->stations_found--;
        rtcfg_dev->stations_ready--;
        rtcfg_dev->spec.srv.clients_configured--;

        rtcfg_send_dead_station(conn);

        rtcfg_next_conn_state(conn, RTCFG_CONN_DEAD);
        conn->cfg_offs = 0;
        conn->flags    = 0;

#if IS_ENABLED(CONFIG_XENO_DRIVERS_NET_RTIPV4)
        if ((conn->addr_type & RTCFG_ADDR_MASK) == RTCFG_ADDR_IP) {
            struct rtnet_device *rtdev = rtdev_get_by_index(conn->ifindex);

            rt_ip_route_del_host(conn->addr.ip_addr, rtdev);

            if (rtdev == NULL)
                return;

            if (!(conn->addr_type & FLAG_ASSIGN_ADDR_BY_MAC))
                /* MAC address yet unknown -> use broadcast address */
                memcpy(conn->mac_addr, rtdev->broadcast, MAX_ADDR_LEN);

            rtdev_dereference(rtdev);
        }
#endif /* CONFIG_XENO_DRIVERS_NET_RTIPV4 */
    }
}
