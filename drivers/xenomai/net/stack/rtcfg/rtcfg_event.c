/***
 *
 *  rtcfg/rtcfg_event.c
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
#include <linux/list.h>
#include <linux/vmalloc.h>

#include <rtdev.h>
#include <ipv4/route.h>
#include <rtcfg/rtcfg.h>
#include <rtcfg/rtcfg_client_event.h>
#include <rtcfg/rtcfg_conn_event.h>
#include <rtcfg/rtcfg_file.h>
#include <rtcfg/rtcfg_frame.h>
#include <rtcfg/rtcfg_timer.h>

/*** Common and Server States ***/
static int rtcfg_main_state_off(int ifindex, RTCFG_EVENT event_id,
				void *event_data);
static int rtcfg_main_state_server_running(int ifindex, RTCFG_EVENT event_id,
					   void *event_data);

#ifdef CONFIG_XENO_DRIVERS_NET_RTCFG_DEBUG
const char *rtcfg_event[] = { "RTCFG_CMD_SERVER",
			      "RTCFG_CMD_ADD",
			      "RTCFG_CMD_DEL",
			      "RTCFG_CMD_WAIT",
			      "RTCFG_CMD_CLIENT",
			      "RTCFG_CMD_ANNOUNCE",
			      "RTCFG_CMD_READY",
			      "RTCFG_CMD_DETACH",
			      "RTCFG_TIMER",
			      "RTCFG_FRM_STAGE_1_CFG",
			      "RTCFG_FRM_ANNOUNCE_NEW",
			      "RTCFG_FRM_ANNOUNCE_REPLY",
			      "RTCFG_FRM_STAGE_2_CFG",
			      "RTCFG_FRM_STAGE_2_CFG_FRAG",
			      "RTCFG_FRM_ACK_CFG",
			      "RTCFG_FRM_READY",
			      "RTCFG_FRM_HEARTBEAT",
			      "RTCFG_FRM_DEAD_STATION" };

const char *rtcfg_main_state[] = { "RTCFG_MAIN_OFF",
				   "RTCFG_MAIN_SERVER_RUNNING",
				   "RTCFG_MAIN_CLIENT_0",
				   "RTCFG_MAIN_CLIENT_1",
				   "RTCFG_MAIN_CLIENT_ANNOUNCED",
				   "RTCFG_MAIN_CLIENT_ALL_KNOWN",
				   "RTCFG_MAIN_CLIENT_ALL_FRAMES",
				   "RTCFG_MAIN_CLIENT_2",
				   "RTCFG_MAIN_CLIENT_READY" };

int rtcfg_debug = RTCFG_DEFAULT_DEBUG_LEVEL;
#endif /* CONFIG_XENO_DRIVERS_NET_RTCFG_DEBUG */

struct rtcfg_device device[MAX_RT_DEVICES];

static int (*state[])(int ifindex, RTCFG_EVENT event_id,
		      void *event_data) = { rtcfg_main_state_off,
					    rtcfg_main_state_server_running,
					    rtcfg_main_state_client_0,
					    rtcfg_main_state_client_1,
					    rtcfg_main_state_client_announced,
					    rtcfg_main_state_client_all_known,
					    rtcfg_main_state_client_all_frames,
					    rtcfg_main_state_client_2,
					    rtcfg_main_state_client_ready };

static int rtcfg_server_add(struct rtcfg_cmd *cmd_event);
static int rtcfg_server_del(struct rtcfg_cmd *cmd_event);
static int rtcfg_server_detach(int ifindex, struct rtcfg_cmd *cmd_event);
static int rtcfg_server_recv_announce(int ifindex, RTCFG_EVENT event_id,
				      struct rtskb *rtskb);
static int rtcfg_server_recv_ack(int ifindex, struct rtskb *rtskb);
static int rtcfg_server_recv_simple_frame(int ifindex, RTCFG_EVENT event_id,
					  struct rtskb *rtskb);

int rtcfg_do_main_event(int ifindex, RTCFG_EVENT event_id, void *event_data)
{
	int main_state;

	rtdm_mutex_lock(&device[ifindex].dev_mutex);

	main_state = device[ifindex].state;

	RTCFG_DEBUG(3, "RTcfg: %s() rtdev=%d, event=%s, state=%s\n",
		    __FUNCTION__, ifindex, rtcfg_event[event_id],
		    rtcfg_main_state[main_state]);

	return (*state[main_state])(ifindex, event_id, event_data);
}

void rtcfg_next_main_state(int ifindex, RTCFG_MAIN_STATE state)
{
	RTCFG_DEBUG(4, "RTcfg: next main state=%s \n", rtcfg_main_state[state]);

	device[ifindex].state = state;
}

static int rtcfg_main_state_off(int ifindex, RTCFG_EVENT event_id,
				void *event_data)
{
	struct rtcfg_device *rtcfg_dev = &device[ifindex];
	struct rt_proc_call *call = (struct rt_proc_call *)event_data;
	struct rtcfg_cmd *cmd_event;
	int ret;

	cmd_event = rtpc_get_priv(call, struct rtcfg_cmd);
	switch (event_id) {
	case RTCFG_CMD_SERVER:
		INIT_LIST_HEAD(&rtcfg_dev->spec.srv.conn_list);

		ret = rtdm_timer_init(&rtcfg_dev->timer, rtcfg_timer,
				      "rtcfg-timer");
		if (ret == 0) {
			ret = rtdm_timer_start(
				&rtcfg_dev->timer, XN_INFINITE,
				(nanosecs_rel_t)cmd_event->args.server.period *
					1000000,
				RTDM_TIMERMODE_RELATIVE);
			if (ret < 0)
				rtdm_timer_destroy(&rtcfg_dev->timer);
		}
		if (ret < 0) {
			rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);
			return ret;
		}

		if (cmd_event->args.server.flags & _RTCFG_FLAG_READY)
			set_bit(RTCFG_FLAG_READY, &rtcfg_dev->flags);
		set_bit(FLAG_TIMER_STARTED, &rtcfg_dev->flags);

		rtcfg_dev->burstrate = cmd_event->args.server.burstrate;

		rtcfg_dev->spec.srv.heartbeat =
			cmd_event->args.server.heartbeat;

		rtcfg_dev->spec.srv.heartbeat_timeout =
			((u64)cmd_event->args.server.heartbeat) * 1000000 *
			cmd_event->args.server.threshold;

		rtcfg_next_main_state(ifindex, RTCFG_MAIN_SERVER_RUNNING);

		rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);

		break;

	case RTCFG_CMD_CLIENT:
		rtcfg_dev->spec.clt.station_addr_list =
			cmd_event->args.client.station_buf;
		cmd_event->args.client.station_buf = NULL;

		rtcfg_dev->spec.clt.max_stations =
			cmd_event->args.client.max_stations;
		rtcfg_dev->other_stations = -1;

		rtcfg_queue_blocking_call(ifindex, call);

		rtcfg_next_main_state(ifindex, RTCFG_MAIN_CLIENT_0);

		rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);

		return -CALL_PENDING;

	default:
		rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);

		RTCFG_DEBUG(1, "RTcfg: unknown event %s for rtdev %d in %s()\n",
			    rtcfg_event[event_id], ifindex, __FUNCTION__);
		return -EINVAL;
	}
	return 0;
}

/*** Server States ***/

static int rtcfg_main_state_server_running(int ifindex, RTCFG_EVENT event_id,
					   void *event_data)
{
	struct rt_proc_call *call;
	struct rtcfg_cmd *cmd_event;
	struct rtcfg_device *rtcfg_dev;
	struct rtskb *rtskb;

	switch (event_id) {
	case RTCFG_CMD_ADD:
		call = (struct rt_proc_call *)event_data;
		cmd_event = rtpc_get_priv(call, struct rtcfg_cmd);

		return rtcfg_server_add(cmd_event);

	case RTCFG_CMD_DEL:
		call = (struct rt_proc_call *)event_data;
		cmd_event = rtpc_get_priv(call, struct rtcfg_cmd);

		return rtcfg_server_del(cmd_event);

	case RTCFG_CMD_WAIT:
		call = (struct rt_proc_call *)event_data;

		rtcfg_dev = &device[ifindex];

		if (rtcfg_dev->spec.srv.clients_configured ==
		    rtcfg_dev->other_stations)
			rtpc_complete_call(call, 0);
		else
			rtcfg_queue_blocking_call(ifindex, call);

		rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);

		return -CALL_PENDING;

	case RTCFG_CMD_READY:
		call = (struct rt_proc_call *)event_data;

		rtcfg_dev = &device[ifindex];

		if (rtcfg_dev->stations_ready == rtcfg_dev->other_stations)
			rtpc_complete_call(call, 0);
		else
			rtcfg_queue_blocking_call(ifindex, call);

		if (!test_and_set_bit(RTCFG_FLAG_READY, &rtcfg_dev->flags))
			rtcfg_send_ready(ifindex);

		rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);

		return -CALL_PENDING;

	case RTCFG_CMD_DETACH:
		call = (struct rt_proc_call *)event_data;
		cmd_event = rtpc_get_priv(call, struct rtcfg_cmd);

		return rtcfg_server_detach(ifindex, cmd_event);

	case RTCFG_FRM_ANNOUNCE_NEW:
	case RTCFG_FRM_ANNOUNCE_REPLY:
		rtskb = (struct rtskb *)event_data;
		return rtcfg_server_recv_announce(ifindex, event_id, rtskb);

	case RTCFG_FRM_ACK_CFG:
		rtskb = (struct rtskb *)event_data;
		return rtcfg_server_recv_ack(ifindex, rtskb);

	case RTCFG_FRM_READY:
	case RTCFG_FRM_HEARTBEAT:
		rtskb = (struct rtskb *)event_data;
		return rtcfg_server_recv_simple_frame(ifindex, event_id, rtskb);

	default:
		rtdm_mutex_unlock(&device[ifindex].dev_mutex);

		RTCFG_DEBUG(1, "RTcfg: unknown event %s for rtdev %d in %s()\n",
			    rtcfg_event[event_id], ifindex, __FUNCTION__);
		return -EINVAL;
	}
	return 0;
}

/*** Server Command Event Handlers ***/

static int rtcfg_server_add(struct rtcfg_cmd *cmd_event)
{
	struct rtcfg_device *rtcfg_dev;
	struct rtcfg_connection *conn;
	struct rtcfg_connection *new_conn;
	struct list_head *entry;
	unsigned int addr_type;

	rtcfg_dev = &device[cmd_event->internal.data.ifindex];
	addr_type = cmd_event->args.add.addr_type & RTCFG_ADDR_MASK;

	new_conn = cmd_event->args.add.conn_buf;
	memset(new_conn, 0, sizeof(struct rtcfg_connection));

	new_conn->ifindex = cmd_event->internal.data.ifindex;
	new_conn->state = RTCFG_CONN_SEARCHING;
	new_conn->addr_type = cmd_event->args.add.addr_type;
#if IS_ENABLED(CONFIG_XENO_DRIVERS_NET_RTIPV4)
	new_conn->addr.ip_addr = cmd_event->args.add.ip_addr;
#endif
	new_conn->stage1_data = cmd_event->args.add.stage1_data;
	new_conn->stage1_size = cmd_event->args.add.stage1_size;
	new_conn->burstrate = rtcfg_dev->burstrate;
	new_conn->cfg_timeout = ((u64)cmd_event->args.add.timeout) * 1000000;

	if (cmd_event->args.add.addr_type == RTCFG_ADDR_IP) {
#if IS_ENABLED(CONFIG_XENO_DRIVERS_NET_RTIPV4)
		struct rtnet_device *rtdev;

		/* MAC address yet unknown -> use broadcast address */
		rtdev = rtdev_get_by_index(cmd_event->internal.data.ifindex);
		if (rtdev == NULL) {
			rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);
			return -ENODEV;
		}
		memcpy(new_conn->mac_addr, rtdev->broadcast, MAX_ADDR_LEN);
		rtdev_dereference(rtdev);
#else /* !CONFIG_XENO_DRIVERS_NET_RTIPV4 */
		return -EPROTONOSUPPORT;
#endif /* CONFIG_XENO_DRIVERS_NET_RTIPV4 */
	} else
		memcpy(new_conn->mac_addr, cmd_event->args.add.mac_addr,
		       MAX_ADDR_LEN);

	/* get stage 2 file */
	if (cmd_event->args.add.stage2_file != NULL) {
		if (cmd_event->args.add.stage2_file->buffer != NULL) {
			new_conn->stage2_file = cmd_event->args.add.stage2_file;
			rtcfg_add_file(new_conn->stage2_file);

			cmd_event->args.add.stage2_file = NULL;
		} else {
			new_conn->stage2_file = rtcfg_get_file(
				cmd_event->args.add.stage2_file->name);
			if (new_conn->stage2_file == NULL) {
				rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);
				return 1;
			}
		}
	}

	list_for_each (entry, &rtcfg_dev->spec.srv.conn_list) {
		conn = list_entry(entry, struct rtcfg_connection, entry);

		if (
#if IS_ENABLED(CONFIG_XENO_DRIVERS_NET_RTIPV4)
			((addr_type == RTCFG_ADDR_IP) &&
			 (conn->addr.ip_addr == cmd_event->args.add.ip_addr)) ||
#endif /* CONFIG_XENO_DRIVERS_NET_RTIPV4 */
			((addr_type == RTCFG_ADDR_MAC) &&
			 (memcmp(conn->mac_addr, new_conn->mac_addr,
				 MAX_ADDR_LEN) == 0))) {
			rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);

			if ((new_conn->stage2_file) &&
			    (rtcfg_release_file(new_conn->stage2_file) == 0)) {
				/* Note: This assignment cannot overwrite a valid file pointer.
		 * Effectively, it will only be executed when
		 * new_conn->stage2_file is the pointer originally passed by
		 * rtcfg_ioctl. But checking this assumptions does not cause
		 * any harm :o)
		 */
				RTNET_ASSERT(cmd_event->args.add.stage2_file ==
						     NULL,
					     ;);

				cmd_event->args.add.stage2_file =
					new_conn->stage2_file;
			}

			return -EEXIST;
		}
	}

	list_add_tail(&new_conn->entry, &rtcfg_dev->spec.srv.conn_list);
	rtcfg_dev->other_stations++;

	rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);

	cmd_event->args.add.conn_buf = NULL;
	cmd_event->args.add.stage1_data = NULL;

	return 0;
}

static int rtcfg_server_del(struct rtcfg_cmd *cmd_event)
{
	struct rtcfg_connection *conn;
	struct list_head *entry;
	unsigned int addr_type;
	struct rtcfg_device *rtcfg_dev;

	rtcfg_dev = &device[cmd_event->internal.data.ifindex];
	addr_type = cmd_event->args.add.addr_type & RTCFG_ADDR_MASK;

	list_for_each (entry, &rtcfg_dev->spec.srv.conn_list) {
		conn = list_entry(entry, struct rtcfg_connection, entry);

		if ((addr_type == conn->addr_type) &&
		    (
#if IS_ENABLED(CONFIG_XENO_DRIVERS_NET_RTIPV4)
			    ((addr_type == RTCFG_ADDR_IP) &&
			     (conn->addr.ip_addr ==
			      cmd_event->args.add.ip_addr)) ||
#endif /* CONFIG_XENO_DRIVERS_NET_RTIPV4 */
			    ((addr_type == RTCFG_ADDR_MAC) &&
			     (memcmp(conn->mac_addr,
				     cmd_event->args.add.mac_addr,
				     MAX_ADDR_LEN) == 0)))) {
			list_del(&conn->entry);
			rtcfg_dev->other_stations--;

			if (conn->state > RTCFG_CONN_SEARCHING) {
				rtcfg_dev->stations_found--;
				if (conn->state >= RTCFG_CONN_STAGE_2)
					rtcfg_dev->spec.srv.clients_configured--;
				if (conn->flags & _RTCFG_FLAG_READY)
					rtcfg_dev->stations_ready--;
			}

			if ((conn->stage2_file) &&
			    (rtcfg_release_file(conn->stage2_file) == 0))
				cmd_event->args.del.stage2_file =
					conn->stage2_file;

			rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);

			cmd_event->args.del.conn_buf = conn;

			return 0;
		}
	}

	rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);

	return -ENOENT;
}

static int rtcfg_server_detach(int ifindex, struct rtcfg_cmd *cmd_event)
{
	struct rtcfg_connection *conn;
	struct rtcfg_device *rtcfg_dev = &device[ifindex];

	if (!list_empty(&rtcfg_dev->spec.srv.conn_list)) {
		conn = list_entry(rtcfg_dev->spec.srv.conn_list.next,
				  struct rtcfg_connection, entry);

		list_del(&conn->entry);
		rtcfg_dev->other_stations--;

		if (conn->state > RTCFG_CONN_SEARCHING) {
			rtcfg_dev->stations_found--;
			if (conn->state >= RTCFG_CONN_STAGE_2)
				rtcfg_dev->spec.srv.clients_configured--;
			if (conn->flags & _RTCFG_FLAG_READY)
				rtcfg_dev->stations_ready--;
		}

		if ((conn->stage2_file) &&
		    (rtcfg_release_file(conn->stage2_file) == 0))
			cmd_event->args.detach.stage2_file = conn->stage2_file;

		rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);

		cmd_event->args.detach.conn_buf = conn;

		return -EAGAIN;
	}

	if (test_and_clear_bit(FLAG_TIMER_STARTED, &rtcfg_dev->flags))
		rtdm_timer_destroy(&rtcfg_dev->timer);
	rtcfg_reset_device(ifindex);

	rtcfg_next_main_state(ifindex, RTCFG_MAIN_OFF);

	rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);

	return 0;
}

/*** Server Frame Event Handlers ***/

static int rtcfg_server_recv_announce(int ifindex, RTCFG_EVENT event_id,
				      struct rtskb *rtskb)
{
	struct rtcfg_device *rtcfg_dev = &device[ifindex];
	struct list_head *entry;
	struct rtcfg_frm_announce *announce;
	struct rtcfg_connection *conn;

	if (rtskb->len < sizeof(struct rtcfg_frm_announce)) {
		rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);
		RTCFG_DEBUG(1, "RTcfg: received invalid announce frame\n");
		return -EINVAL;
	}

	announce = (struct rtcfg_frm_announce *)rtskb->data;

	list_for_each (entry, &rtcfg_dev->spec.srv.conn_list) {
		conn = list_entry(entry, struct rtcfg_connection, entry);

		switch (announce->addr_type) {
#if IS_ENABLED(CONFIG_XENO_DRIVERS_NET_RTIPV4)
			u32 announce_addr;
		case RTCFG_ADDR_IP:
			memcpy(&announce_addr, announce->addr, 4);

			if (((conn->addr_type & RTCFG_ADDR_MASK) ==
			     RTCFG_ADDR_IP) &&
			    (announce_addr == conn->addr.ip_addr)) {
				/* save MAC address - Ethernet-specific! */
				memcpy(conn->mac_addr,
				       rtskb->mac.ethernet->h_source, ETH_ALEN);

				/* update routing table */
				rt_ip_route_add_host(conn->addr.ip_addr,
						     conn->mac_addr,
						     rtskb->rtdev);

				/* remove IP address */
				__rtskb_pull(rtskb, RTCFG_ADDRSIZE_IP);

				rtcfg_do_conn_event(conn, event_id, rtskb);

				goto out;
			}
			break;
#endif /* CONFIG_XENO_DRIVERS_NET_RTIPV4 */

		case RTCFG_ADDR_MAC:
			/* Ethernet-specific! */
			if (memcmp(conn->mac_addr,
				   rtskb->mac.ethernet->h_source,
				   ETH_ALEN) == 0) {
				rtcfg_do_conn_event(conn, event_id, rtskb);

				goto out;
			}
			break;
		}
	}

out:
	rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);

	kfree_rtskb(rtskb);
	return 0;
}

static int rtcfg_server_recv_ack(int ifindex, struct rtskb *rtskb)
{
	struct rtcfg_device *rtcfg_dev = &device[ifindex];
	struct list_head *entry;
	struct rtcfg_connection *conn;

	if (rtskb->len < sizeof(struct rtcfg_frm_ack_cfg)) {
		rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);
		RTCFG_DEBUG(1, "RTcfg: received invalid ack_cfg frame\n");
		return -EINVAL;
	}

	list_for_each (entry, &rtcfg_dev->spec.srv.conn_list) {
		conn = list_entry(entry, struct rtcfg_connection, entry);

		/* find the corresponding connection - Ethernet-specific! */
		if (memcmp(conn->mac_addr, rtskb->mac.ethernet->h_source,
			   ETH_ALEN) != 0)
			continue;

		rtcfg_do_conn_event(conn, RTCFG_FRM_ACK_CFG, rtskb);

		break;
	}

	rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);

	kfree_rtskb(rtskb);
	return 0;
}

static int rtcfg_server_recv_simple_frame(int ifindex, RTCFG_EVENT event_id,
					  struct rtskb *rtskb)
{
	struct rtcfg_device *rtcfg_dev = &device[ifindex];
	struct list_head *entry;
	struct rtcfg_connection *conn;

	list_for_each (entry, &rtcfg_dev->spec.srv.conn_list) {
		conn = list_entry(entry, struct rtcfg_connection, entry);

		/* find the corresponding connection - Ethernet-specific! */
		if (memcmp(conn->mac_addr, rtskb->mac.ethernet->h_source,
			   ETH_ALEN) != 0)
			continue;

		rtcfg_do_conn_event(conn, event_id, rtskb);

		break;
	}

	rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);

	kfree_rtskb(rtskb);
	return 0;
}

/*** Utility Functions ***/

void rtcfg_queue_blocking_call(int ifindex, struct rt_proc_call *call)
{
	rtdm_lockctx_t context;
	struct rtcfg_device *rtcfg_dev = &device[ifindex];

	rtdm_lock_get_irqsave(&rtcfg_dev->event_calls_lock, context);
	list_add_tail(&call->list_entry, &rtcfg_dev->event_calls);
	rtdm_lock_put_irqrestore(&rtcfg_dev->event_calls_lock, context);
}

struct rt_proc_call *rtcfg_dequeue_blocking_call(int ifindex)
{
	rtdm_lockctx_t context;
	struct rt_proc_call *call;
	struct rtcfg_device *rtcfg_dev = &device[ifindex];

	rtdm_lock_get_irqsave(&rtcfg_dev->event_calls_lock, context);
	if (!list_empty(&rtcfg_dev->event_calls)) {
		call = (struct rt_proc_call *)rtcfg_dev->event_calls.next;
		list_del(&call->list_entry);
	} else
		call = NULL;
	rtdm_lock_put_irqrestore(&rtcfg_dev->event_calls_lock, context);

	return call;
}

void rtcfg_complete_cmd(int ifindex, RTCFG_EVENT event_id, int result)
{
	struct rt_proc_call *call;
	struct rtcfg_cmd *cmd_event;

	while (1) {
		call = rtcfg_dequeue_blocking_call(ifindex);
		if (call == NULL)
			break;

		cmd_event = rtpc_get_priv(call, struct rtcfg_cmd);

		rtpc_complete_call(call, (cmd_event->internal.data.event_id ==
					  event_id) ?
						 result :
						 -EINVAL);
	}
}

void rtcfg_reset_device(int ifindex)
{
	struct rtcfg_device *rtcfg_dev = &device[ifindex];

	rtcfg_dev->other_stations = 0;
	rtcfg_dev->stations_found = 0;
	rtcfg_dev->stations_ready = 0;
	rtcfg_dev->flags = 0;
	rtcfg_dev->burstrate = 0;

	memset(&rtcfg_dev->spec, 0, sizeof(rtcfg_dev->spec));
	INIT_LIST_HEAD(&rtcfg_dev->spec.srv.conn_list);
}

void rtcfg_init_state_machines(void)
{
	int i;
	struct rtcfg_device *rtcfg_dev;

	memset(device, 0, sizeof(device));

	for (i = 0; i < MAX_RT_DEVICES; i++) {
		rtcfg_dev = &device[i];
		rtcfg_dev->state = RTCFG_MAIN_OFF;

		rtdm_mutex_init(&rtcfg_dev->dev_mutex);

		INIT_LIST_HEAD(&rtcfg_dev->event_calls);
		rtdm_lock_init(&rtcfg_dev->event_calls_lock);
	}
}

void rtcfg_cleanup_state_machines(void)
{
	int i;
	struct rtcfg_device *rtcfg_dev;
	struct rtcfg_connection *conn;
	struct list_head *entry;
	struct list_head *tmp;
	struct rt_proc_call *call;

	for (i = 0; i < MAX_RT_DEVICES; i++) {
		rtcfg_dev = &device[i];

		if (test_and_clear_bit(FLAG_TIMER_STARTED, &rtcfg_dev->flags))
			rtdm_timer_destroy(&rtcfg_dev->timer);

		/*
	 * No need to synchronize with rtcfg_timer here: the task running
	 * rtcfg_timer is already dead.
	 */

		rtdm_mutex_destroy(&rtcfg_dev->dev_mutex);

		if (rtcfg_dev->state == RTCFG_MAIN_SERVER_RUNNING) {
			list_for_each_safe (entry, tmp,
					    &rtcfg_dev->spec.srv.conn_list) {
				conn = list_entry(
					entry, struct rtcfg_connection, entry);

				if (conn->stage1_data != NULL)
					kfree(conn->stage1_data);

				if ((conn->stage2_file != NULL) &&
				    (rtcfg_release_file(conn->stage2_file) ==
				     0)) {
					vfree(conn->stage2_file->buffer);
					kfree(conn->stage2_file);
				}

				kfree(entry);
			}
		} else if (rtcfg_dev->state != RTCFG_MAIN_OFF) {
			if (rtcfg_dev->spec.clt.station_addr_list != NULL)
				kfree(rtcfg_dev->spec.clt.station_addr_list);

			if (rtcfg_dev->spec.clt.stage2_chain != NULL)
				kfree_rtskb(rtcfg_dev->spec.clt.stage2_chain);
		}

		while (1) {
			call = rtcfg_dequeue_blocking_call(i);
			if (call == NULL)
				break;

			rtpc_complete_call_nrt(call, -ENODEV);
		}
	}
}
