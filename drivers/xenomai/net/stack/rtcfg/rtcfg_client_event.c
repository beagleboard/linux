/***
 *
 *  rtcfg/rtcfg_client_event.c
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

#include <ipv4/route.h>
#include <rtcfg/rtcfg.h>
#include <rtcfg/rtcfg_event.h>
#include <rtcfg/rtcfg_frame.h>
#include <rtcfg/rtcfg_timer.h>

static int rtcfg_client_get_frag(int ifindex, struct rt_proc_call *call);
static void rtcfg_client_detach(int ifindex, struct rt_proc_call *call);
static void rtcfg_client_recv_stage_1(int ifindex, struct rtskb *rtskb);
static int rtcfg_client_recv_announce(int ifindex, struct rtskb *rtskb);
static void rtcfg_client_recv_stage_2_cfg(int ifindex, struct rtskb *rtskb);
static void rtcfg_client_recv_stage_2_frag(int ifindex, struct rtskb *rtskb);
static int rtcfg_client_recv_ready(int ifindex, struct rtskb *rtskb);
static void rtcfg_client_recv_dead_station(int ifindex, struct rtskb *rtskb);
static void rtcfg_client_update_server(int ifindex, struct rtskb *rtskb);

/*** Client States ***/

int rtcfg_main_state_client_0(int ifindex, RTCFG_EVENT event_id,
			      void *event_data)
{
	struct rtskb *rtskb = (struct rtskb *)event_data;
	struct rt_proc_call *call = (struct rt_proc_call *)event_data;

	switch (event_id) {
	case RTCFG_CMD_DETACH:
		rtcfg_client_detach(ifindex, call);
		break;

	case RTCFG_FRM_STAGE_1_CFG:
		rtcfg_client_recv_stage_1(ifindex, rtskb);
		break;

	case RTCFG_FRM_ANNOUNCE_NEW:
		if (rtcfg_client_recv_announce(ifindex, rtskb) == 0)
			rtdm_mutex_unlock(&device[ifindex].dev_mutex);
		kfree_rtskb(rtskb);
		break;

	case RTCFG_FRM_READY:
		if (rtcfg_client_recv_ready(ifindex, rtskb) == 0)
			rtdm_mutex_unlock(&device[ifindex].dev_mutex);
		break;

	default:
		rtdm_mutex_unlock(&device[ifindex].dev_mutex);
		RTCFG_DEBUG(1, "RTcfg: unknown event %s for rtdev %d in %s()\n",
			    rtcfg_event[event_id], ifindex, __FUNCTION__);
		return -EINVAL;
	}
	return 0;
}

int rtcfg_main_state_client_1(int ifindex, RTCFG_EVENT event_id,
			      void *event_data)
{
	struct rtcfg_device *rtcfg_dev = &device[ifindex];
	struct rtskb *rtskb = (struct rtskb *)event_data;
	struct rt_proc_call *call = (struct rt_proc_call *)event_data;
	struct rtcfg_cmd *cmd_event;
	int ret;

	switch (event_id) {
	case RTCFG_CMD_CLIENT:
		/* second trial (buffer was probably too small) */
		rtcfg_queue_blocking_call(ifindex,
					  (struct rt_proc_call *)event_data);

		rtcfg_next_main_state(ifindex, RTCFG_MAIN_CLIENT_0);

		rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);

		return -CALL_PENDING;

	case RTCFG_CMD_ANNOUNCE:
		cmd_event = rtpc_get_priv(call, struct rtcfg_cmd);

		if (cmd_event->args.announce.burstrate == 0) {
			rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);
			return -EINVAL;
		}

		rtcfg_queue_blocking_call(ifindex,
					  (struct rt_proc_call *)event_data);

		if (cmd_event->args.announce.flags & _RTCFG_FLAG_STAGE_2_DATA)
			set_bit(RTCFG_FLAG_STAGE_2_DATA, &rtcfg_dev->flags);
		if (cmd_event->args.announce.flags & _RTCFG_FLAG_READY)
			set_bit(RTCFG_FLAG_READY, &rtcfg_dev->flags);
		if (cmd_event->args.announce.burstrate < rtcfg_dev->burstrate)
			rtcfg_dev->burstrate =
				cmd_event->args.announce.burstrate;

		rtcfg_next_main_state(ifindex, RTCFG_MAIN_CLIENT_ANNOUNCED);

		ret = rtcfg_send_announce_new(ifindex);
		if (ret < 0) {
			rtcfg_dequeue_blocking_call(ifindex);
			rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);
			return ret;
		}

		rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);

		return -CALL_PENDING;

	case RTCFG_CMD_DETACH:
		rtcfg_client_detach(ifindex, call);
		break;

	case RTCFG_FRM_ANNOUNCE_NEW:
		if (rtcfg_client_recv_announce(ifindex, rtskb) == 0) {
			rtcfg_send_announce_reply(
				ifindex, rtskb->mac.ethernet->h_source);
			rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);
		}

		kfree_rtskb(rtskb);
		break;

	case RTCFG_FRM_ANNOUNCE_REPLY:
		if (rtcfg_client_recv_announce(ifindex, rtskb) == 0)
			rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);

		kfree_rtskb(rtskb);
		break;

	case RTCFG_FRM_READY:
		if (rtcfg_client_recv_ready(ifindex, rtskb) == 0)
			rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);
		break;

	case RTCFG_FRM_STAGE_1_CFG:
		/* ignore */
		rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);
		kfree_rtskb(rtskb);
		break;

	default:
		rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);
		RTCFG_DEBUG(1, "RTcfg: unknown event %s for rtdev %d in %s()\n",
			    rtcfg_event[event_id], ifindex, __FUNCTION__);
		return -EINVAL;
	}
	return 0;
}

int rtcfg_main_state_client_announced(int ifindex, RTCFG_EVENT event_id,
				      void *event_data)
{
	struct rtskb *rtskb = (struct rtskb *)event_data;
	struct rt_proc_call *call = (struct rt_proc_call *)event_data;
	struct rtcfg_device *rtcfg_dev;

	switch (event_id) {
	case RTCFG_CMD_ANNOUNCE:
		return rtcfg_client_get_frag(ifindex, call);

	case RTCFG_CMD_DETACH:
		rtcfg_client_detach(ifindex, call);
		break;

	case RTCFG_FRM_STAGE_2_CFG:
		rtcfg_client_recv_stage_2_cfg(ifindex, rtskb);
		break;

	case RTCFG_FRM_STAGE_2_CFG_FRAG:
		rtcfg_client_recv_stage_2_frag(ifindex, rtskb);
		break;

	case RTCFG_FRM_ANNOUNCE_NEW:
		if (rtcfg_client_recv_announce(ifindex, rtskb) == 0) {
			rtcfg_send_announce_reply(
				ifindex, rtskb->mac.ethernet->h_source);

			rtcfg_dev = &device[ifindex];
			if (rtcfg_dev->stations_found ==
			    rtcfg_dev->other_stations)
				rtcfg_next_main_state(
					ifindex, RTCFG_MAIN_CLIENT_ALL_KNOWN);

			rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);
		}
		kfree_rtskb(rtskb);
		break;

	case RTCFG_FRM_ANNOUNCE_REPLY:
		if (rtcfg_client_recv_announce(ifindex, rtskb) == 0) {
			rtcfg_dev = &device[ifindex];
			if (rtcfg_dev->stations_found ==
			    rtcfg_dev->other_stations)
				rtcfg_next_main_state(
					ifindex, RTCFG_MAIN_CLIENT_ALL_KNOWN);

			rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);
		}
		kfree_rtskb(rtskb);
		break;

	case RTCFG_FRM_READY:
		if (rtcfg_client_recv_ready(ifindex, rtskb) == 0)
			rtdm_mutex_unlock(&device[ifindex].dev_mutex);
		break;

	case RTCFG_FRM_STAGE_1_CFG:
		/* ignore */
		rtdm_mutex_unlock(&device[ifindex].dev_mutex);
		kfree_rtskb(rtskb);
		break;

	default:
		rtdm_mutex_unlock(&device[ifindex].dev_mutex);
		RTCFG_DEBUG(1, "RTcfg: unknown event %s for rtdev %d in %s()\n",
			    rtcfg_event[event_id], ifindex, __FUNCTION__);
		return -EINVAL;
	}

	return 0;
}

int rtcfg_main_state_client_all_known(int ifindex, RTCFG_EVENT event_id,
				      void *event_data)
{
	struct rtskb *rtskb = (struct rtskb *)event_data;
	struct rt_proc_call *call = (struct rt_proc_call *)event_data;

	switch (event_id) {
	case RTCFG_CMD_ANNOUNCE:
		return rtcfg_client_get_frag(ifindex, call);

	case RTCFG_CMD_DETACH:
		rtcfg_client_detach(ifindex, call);
		break;

	case RTCFG_FRM_STAGE_2_CFG_FRAG:
		rtcfg_client_recv_stage_2_frag(ifindex, rtskb);
		break;

	case RTCFG_FRM_READY:
		if (rtcfg_client_recv_ready(ifindex, rtskb) == 0)
			rtdm_mutex_unlock(&device[ifindex].dev_mutex);
		break;

	case RTCFG_FRM_ANNOUNCE_NEW:
		if (rtcfg_client_recv_announce(ifindex, rtskb) == 0) {
			rtcfg_send_announce_reply(
				ifindex, rtskb->mac.ethernet->h_source);
			rtdm_mutex_unlock(&device[ifindex].dev_mutex);
		}
		kfree_rtskb(rtskb);
		break;

	case RTCFG_FRM_DEAD_STATION:
		rtcfg_client_recv_dead_station(ifindex, rtskb);
		break;

	case RTCFG_FRM_STAGE_1_CFG:
		/* ignore */
		rtdm_mutex_unlock(&device[ifindex].dev_mutex);
		kfree_rtskb(rtskb);
		break;

	default:
		rtdm_mutex_unlock(&device[ifindex].dev_mutex);
		RTCFG_DEBUG(1, "RTcfg: unknown event %s for rtdev %d in %s()\n",
			    rtcfg_event[event_id], ifindex, __FUNCTION__);
		return -EINVAL;
	}
	return 0;
}

int rtcfg_main_state_client_all_frames(int ifindex, RTCFG_EVENT event_id,
				       void *event_data)
{
	struct rtskb *rtskb = (struct rtskb *)event_data;
	struct rt_proc_call *call = (struct rt_proc_call *)event_data;
	struct rtcfg_device *rtcfg_dev;

	switch (event_id) {
	case RTCFG_CMD_DETACH:
		rtcfg_client_detach(ifindex, call);
		break;

	case RTCFG_FRM_ANNOUNCE_NEW:
		if (rtcfg_client_recv_announce(ifindex, rtskb) == 0) {
			rtcfg_send_announce_reply(
				ifindex, rtskb->mac.ethernet->h_source);

			rtcfg_dev = &device[ifindex];
			if (rtcfg_dev->stations_found ==
			    rtcfg_dev->other_stations) {
				rtcfg_complete_cmd(ifindex, RTCFG_CMD_ANNOUNCE,
						   0);

				rtcfg_next_main_state(
					ifindex,
					test_bit(RTCFG_FLAG_READY,
						 &rtcfg_dev->flags) ?
						RTCFG_MAIN_CLIENT_READY :
						RTCFG_MAIN_CLIENT_2);
			}

			rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);
		}
		kfree_rtskb(rtskb);
		break;

	case RTCFG_FRM_ANNOUNCE_REPLY:
		if (rtcfg_client_recv_announce(ifindex, rtskb) == 0) {
			rtcfg_dev = &device[ifindex];
			if (rtcfg_dev->stations_found ==
			    rtcfg_dev->other_stations) {
				rtcfg_complete_cmd(ifindex, RTCFG_CMD_ANNOUNCE,
						   0);

				rtcfg_next_main_state(
					ifindex,
					test_bit(RTCFG_FLAG_READY,
						 &rtcfg_dev->flags) ?
						RTCFG_MAIN_CLIENT_READY :
						RTCFG_MAIN_CLIENT_2);
			}

			rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);
		}
		kfree_rtskb(rtskb);
		break;

	case RTCFG_FRM_READY:
		if (rtcfg_client_recv_ready(ifindex, rtskb) == 0)
			rtdm_mutex_unlock(&device[ifindex].dev_mutex);
		break;

	case RTCFG_FRM_DEAD_STATION:
		rtcfg_client_recv_dead_station(ifindex, rtskb);
		break;

	case RTCFG_FRM_STAGE_1_CFG:
		/* ignore */
		rtdm_mutex_unlock(&device[ifindex].dev_mutex);
		kfree_rtskb(rtskb);
		break;

	default:
		rtdm_mutex_unlock(&device[ifindex].dev_mutex);
		RTCFG_DEBUG(1, "RTcfg: unknown event %s for rtdev %d in %s()\n",
			    rtcfg_event[event_id], ifindex, __FUNCTION__);
		return -EINVAL;
	}
	return 0;
}

int rtcfg_main_state_client_2(int ifindex, RTCFG_EVENT event_id,
			      void *event_data)
{
	struct rtskb *rtskb = (struct rtskb *)event_data;
	struct rt_proc_call *call = (struct rt_proc_call *)event_data;
	struct rtcfg_device *rtcfg_dev;

	switch (event_id) {
	case RTCFG_CMD_READY:
		rtcfg_dev = &device[ifindex];

		if (rtcfg_dev->stations_ready == rtcfg_dev->other_stations)
			rtpc_complete_call(call, 0);
		else
			rtcfg_queue_blocking_call(ifindex, call);

		rtcfg_next_main_state(ifindex, RTCFG_MAIN_CLIENT_READY);

		if (!test_and_set_bit(RTCFG_FLAG_READY, &rtcfg_dev->flags))
			rtcfg_send_ready(ifindex);

		rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);

		return -CALL_PENDING;

	case RTCFG_CMD_DETACH:
		rtcfg_client_detach(ifindex, call);
		break;

	case RTCFG_FRM_READY:
		if (rtcfg_client_recv_ready(ifindex, rtskb) == 0)
			rtdm_mutex_unlock(&device[ifindex].dev_mutex);
		break;

	case RTCFG_FRM_ANNOUNCE_NEW:
		if (rtcfg_client_recv_announce(ifindex, rtskb) == 0) {
			rtcfg_send_announce_reply(
				ifindex, rtskb->mac.ethernet->h_source);
			rtdm_mutex_unlock(&device[ifindex].dev_mutex);
		}
		kfree_rtskb(rtskb);
		break;

	case RTCFG_FRM_DEAD_STATION:
		rtcfg_client_recv_dead_station(ifindex, rtskb);
		break;

	case RTCFG_FRM_STAGE_1_CFG:
		/* ignore */
		rtdm_mutex_unlock(&device[ifindex].dev_mutex);
		kfree_rtskb(rtskb);
		break;

	default:
		rtdm_mutex_unlock(&device[ifindex].dev_mutex);
		RTCFG_DEBUG(1, "RTcfg: unknown event %s for rtdev %d in %s()\n",
			    rtcfg_event[event_id], ifindex, __FUNCTION__);
		return -EINVAL;
	}
	return 0;
}

int rtcfg_main_state_client_ready(int ifindex, RTCFG_EVENT event_id,
				  void *event_data)
{
	struct rtskb *rtskb = (struct rtskb *)event_data;
	struct rt_proc_call *call = (struct rt_proc_call *)event_data;
	struct rtcfg_device *rtcfg_dev;

	switch (event_id) {
	case RTCFG_CMD_DETACH:
		rtcfg_client_detach(ifindex, call);
		break;

	case RTCFG_FRM_READY:
		if (rtcfg_client_recv_ready(ifindex, rtskb) == 0) {
			rtcfg_dev = &device[ifindex];
			if (rtcfg_dev->stations_ready ==
			    rtcfg_dev->other_stations)
				rtcfg_complete_cmd(ifindex, RTCFG_CMD_READY, 0);

			rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);
		}
		break;

	case RTCFG_FRM_ANNOUNCE_NEW:
		if (rtcfg_client_recv_announce(ifindex, rtskb) == 0) {
			rtcfg_send_announce_reply(
				ifindex, rtskb->mac.ethernet->h_source);
			rtdm_mutex_unlock(&device[ifindex].dev_mutex);
		}
		kfree_rtskb(rtskb);
		break;

	case RTCFG_FRM_DEAD_STATION:
		rtcfg_client_recv_dead_station(ifindex, rtskb);
		break;

	case RTCFG_FRM_STAGE_1_CFG:
		rtcfg_client_update_server(ifindex, rtskb);
		break;

	default:
		rtdm_mutex_unlock(&device[ifindex].dev_mutex);
		RTCFG_DEBUG(1, "RTcfg: unknown event %s for rtdev %d in %s()\n",
			    rtcfg_event[event_id], ifindex, __FUNCTION__);
		return -EINVAL;
	}
	return 0;
}

/*** Client Command Event Handlers ***/

static int rtcfg_client_get_frag(int ifindex, struct rt_proc_call *call)
{
	struct rtcfg_device *rtcfg_dev = &device[ifindex];

	if (test_bit(RTCFG_FLAG_STAGE_2_DATA, &rtcfg_dev->flags) == 0) {
		rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);
		return -EINVAL;
	}

	rtcfg_send_ack(ifindex);

	if (rtcfg_dev->spec.clt.cfg_offs >= rtcfg_dev->spec.clt.cfg_len) {
		if (rtcfg_dev->stations_found == rtcfg_dev->other_stations) {
			rtpc_complete_call(call, 0);

			rtcfg_next_main_state(ifindex,
					      test_bit(RTCFG_FLAG_READY,
						       &rtcfg_dev->flags) ?
						      RTCFG_MAIN_CLIENT_READY :
						      RTCFG_MAIN_CLIENT_2);
		} else {
			rtcfg_next_main_state(ifindex,
					      RTCFG_MAIN_CLIENT_ALL_FRAMES);
			rtcfg_queue_blocking_call(ifindex, call);
		}
	} else
		rtcfg_queue_blocking_call(ifindex, call);

	rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);

	return -CALL_PENDING;
}

/* releases rtcfg_dev->dev_mutex on return */
static void rtcfg_client_detach(int ifindex, struct rt_proc_call *call)
{
	struct rtcfg_device *rtcfg_dev = &device[ifindex];
	struct rtcfg_cmd *cmd_event;

	cmd_event = rtpc_get_priv(call, struct rtcfg_cmd);

	cmd_event->args.detach.station_addr_list =
		rtcfg_dev->spec.clt.station_addr_list;
	cmd_event->args.detach.stage2_chain = rtcfg_dev->spec.clt.stage2_chain;

	while (1) {
		call = rtcfg_dequeue_blocking_call(ifindex);
		if (call == NULL)
			break;

		rtpc_complete_call(call, -ENODEV);
	}

	if (test_and_clear_bit(FLAG_TIMER_STARTED, &rtcfg_dev->flags))
		rtdm_timer_destroy(&rtcfg_dev->timer);
	rtcfg_reset_device(ifindex);

	rtcfg_next_main_state(cmd_event->internal.data.ifindex, RTCFG_MAIN_OFF);

	rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);
}

/*** Client Frame Event Handlers ***/

static void rtcfg_client_recv_stage_1(int ifindex, struct rtskb *rtskb)
{
	struct rtcfg_frm_stage_1_cfg *stage_1_cfg;
	struct rt_proc_call *call;
	struct rtcfg_cmd *cmd_event;
	struct rtcfg_device *rtcfg_dev = &device[ifindex];
	u8 addr_type;
	int ret;

	if (rtskb->len < sizeof(struct rtcfg_frm_stage_1_cfg)) {
		rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);
		RTCFG_DEBUG(1, "RTcfg: received invalid stage_1_cfg frame\n");
		kfree_rtskb(rtskb);
		return;
	}

	stage_1_cfg = (struct rtcfg_frm_stage_1_cfg *)rtskb->data;
	__rtskb_pull(rtskb, sizeof(struct rtcfg_frm_stage_1_cfg));

	addr_type = stage_1_cfg->addr_type;

	switch (stage_1_cfg->addr_type) {
#if IS_ENABLED(CONFIG_XENO_DRIVERS_NET_RTIPV4)
	case RTCFG_ADDR_IP: {
		struct rtnet_device *rtdev, *tmp;
		u32 daddr, saddr, mask, bcast;

		if (rtskb->len < sizeof(struct rtcfg_frm_stage_1_cfg) +
					 2 * RTCFG_ADDRSIZE_IP) {
			rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);
			RTCFG_DEBUG(1, "RTcfg: received invalid stage_1_cfg "
				       "frame\n");
			kfree_rtskb(rtskb);
			return;
		}

		rtdev = rtskb->rtdev;

		memcpy(&daddr, stage_1_cfg->client_addr, 4);
		stage_1_cfg =
			(struct rtcfg_frm_stage_1_cfg *)(((u8 *)stage_1_cfg) +
							 RTCFG_ADDRSIZE_IP);

		memcpy(&saddr, stage_1_cfg->server_addr, 4);
		stage_1_cfg =
			(struct rtcfg_frm_stage_1_cfg *)(((u8 *)stage_1_cfg) +
							 RTCFG_ADDRSIZE_IP);

		__rtskb_pull(rtskb, 2 * RTCFG_ADDRSIZE_IP);

		/* Broadcast: IP is used to address client */
		if (rtskb->pkt_type == PACKET_BROADCAST) {
			/* directed to us? */
			if (daddr != rtdev->local_ip) {
				rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);
				kfree_rtskb(rtskb);
				return;
			}

			/* Unicast: IP address is assigned by the server */
		} else {
			/* default netmask */
			if (ntohl(daddr) <= 0x7FFFFFFF) /* 127.255.255.255  */
				mask = 0x000000FF; /* 255.0.0.0        */
			else if (ntohl(daddr) <=
				 0xBFFFFFFF) /* 191.255.255.255  */
				mask = 0x0000FFFF; /* 255.255.0.0      */
			else
				mask = 0x00FFFFFF; /* 255.255.255.0    */
			bcast = daddr | (~mask);

			rt_ip_route_del_all(rtdev); /* cleanup routing table */

			rtdev->local_ip = daddr;
			rtdev->broadcast_ip = bcast;

			if ((tmp = rtdev_get_loopback()) != NULL) {
				rt_ip_route_add_host(daddr, tmp->dev_addr, tmp);
				rtdev_dereference(tmp);
			}

			if (rtdev->flags & IFF_BROADCAST)
				rt_ip_route_add_host(bcast, rtdev->broadcast,
						     rtdev);
		}

		/* update routing table */
		rt_ip_route_add_host(saddr, rtskb->mac.ethernet->h_source,
				     rtdev);

		rtcfg_dev->spec.clt.srv_addr.ip_addr = saddr;
		break;
	}
#endif /* CONFIG_XENO_DRIVERS_NET_RTIPV4 */

	case RTCFG_ADDR_MAC:
		/* nothing to do */
		break;

	default:
		rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);
		RTCFG_DEBUG(1, "RTcfg: unknown addr_type %d in %s()\n",
			    stage_1_cfg->addr_type, __FUNCTION__);
		kfree_rtskb(rtskb);
		return;
	}

	rtcfg_dev->spec.clt.addr_type = addr_type;

	/* Ethernet-specific */
	memcpy(rtcfg_dev->spec.clt.srv_mac_addr, rtskb->mac.ethernet->h_source,
	       ETH_ALEN);

	rtcfg_dev->burstrate = stage_1_cfg->burstrate;

	rtcfg_next_main_state(ifindex, RTCFG_MAIN_CLIENT_1);

	rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);

	while (1) {
		call = rtcfg_dequeue_blocking_call(ifindex);
		if (call == NULL)
			break;

		cmd_event = rtpc_get_priv(call, struct rtcfg_cmd);

		if (cmd_event->internal.data.event_id == RTCFG_CMD_CLIENT) {
			ret = 0;

			/* note: only the first pending call gets data */
			if ((rtskb != NULL) &&
			    (cmd_event->args.client.buffer_size > 0)) {
				ret = ntohs(stage_1_cfg->cfg_len);

				cmd_event->args.client.rtskb = rtskb;
				rtskb = NULL;
			}
		} else
			ret = -EINVAL;

		rtpc_complete_call(call, ret);
	}

	if (rtskb)
		kfree_rtskb(rtskb);
}

static int rtcfg_add_to_station_list(struct rtcfg_device *rtcfg_dev,
				     u8 *mac_addr, u8 flags)
{
	if (rtcfg_dev->stations_found == rtcfg_dev->spec.clt.max_stations) {
		RTCFG_DEBUG(
			1, "RTcfg: insufficient memory for storing new station "
			   "address\n");
		return -ENOMEM;
	}

	/* Ethernet-specific! */
	memcpy(&rtcfg_dev->spec.clt.station_addr_list[rtcfg_dev->stations_found]
			.mac_addr,
	       mac_addr, ETH_ALEN);

	rtcfg_dev->spec.clt.station_addr_list[rtcfg_dev->stations_found].flags =
		flags;

	rtcfg_dev->stations_found++;
	if ((flags & _RTCFG_FLAG_READY) != 0)
		rtcfg_dev->stations_ready++;

	return 0;
}

/* Notes:
 *  o rtcfg_client_recv_announce does not release the passed rtskb.
 *  o On success, rtcfg_client_recv_announce returns without releasing the
 *    device lock.
 */
static int rtcfg_client_recv_announce(int ifindex, struct rtskb *rtskb)
{
	struct rtcfg_frm_announce *announce_frm;
	struct rtcfg_device *rtcfg_dev = &device[ifindex];
	u32 i;
	u32 announce_frm_addr;
	int result;

	announce_frm = (struct rtcfg_frm_announce *)rtskb->data;

	if (rtskb->len < sizeof(struct rtcfg_frm_announce)) {
		rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);
		RTCFG_DEBUG(1,
			    "RTcfg: received invalid announce frame (id: %d)\n",
			    announce_frm->head.id);
		return -EINVAL;
	}

	switch (announce_frm->addr_type) {
#if IS_ENABLED(CONFIG_XENO_DRIVERS_NET_RTIPV4)
	case RTCFG_ADDR_IP:
		if (rtskb->len <
		    sizeof(struct rtcfg_frm_announce) + RTCFG_ADDRSIZE_IP) {
			rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);
			RTCFG_DEBUG(1,
				    "RTcfg: received invalid announce frame "
				    "(id: %d)\n",
				    announce_frm->head.id);
			return -EINVAL;
		}

		memcpy(&announce_frm_addr, announce_frm->addr, 4);

		/* update routing table */
		rt_ip_route_add_host(announce_frm_addr,
				     rtskb->mac.ethernet->h_source,
				     rtskb->rtdev);

		announce_frm =
			(struct rtcfg_frm_announce *)(((u8 *)announce_frm) +
						      RTCFG_ADDRSIZE_IP);

		break;
#endif /* CONFIG_XENO_DRIVERS_NET_RTIPV4 */

	case RTCFG_ADDR_MAC:
		/* nothing to do */
		break;

	default:
		rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);
		RTCFG_DEBUG(1, "RTcfg: unknown addr_type %d in %s()\n",
			    announce_frm->addr_type, __FUNCTION__);
		return -EINVAL;
	}

	for (i = 0; i < rtcfg_dev->stations_found; i++)
		/* Ethernet-specific! */
		if (memcmp(rtcfg_dev->spec.clt.station_addr_list[i].mac_addr,
			   rtskb->mac.ethernet->h_source, ETH_ALEN) == 0)
			return 0;

	result = rtcfg_add_to_station_list(
		rtcfg_dev, rtskb->mac.ethernet->h_source, announce_frm->flags);
	if (result < 0)
		rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);

	return result;
}

static void rtcfg_client_queue_frag(int ifindex, struct rtskb *rtskb,
				    size_t data_len)
{
	struct rtcfg_device *rtcfg_dev = &device[ifindex];
	struct rt_proc_call *call;
	struct rtcfg_cmd *cmd_event;
	int result;

	rtskb_trim(rtskb, data_len);

	if (rtcfg_dev->spec.clt.stage2_chain == NULL)
		rtcfg_dev->spec.clt.stage2_chain = rtskb;
	else {
		rtcfg_dev->spec.clt.stage2_chain->chain_end->next = rtskb;
		rtcfg_dev->spec.clt.stage2_chain->chain_end = rtskb;
	}

	rtcfg_dev->spec.clt.cfg_offs += data_len;
	rtcfg_dev->spec.clt.chain_len += data_len;

	if ((rtcfg_dev->spec.clt.cfg_offs >= rtcfg_dev->spec.clt.cfg_len) ||
	    (++rtcfg_dev->spec.clt.packet_counter == rtcfg_dev->burstrate)) {
		while (1) {
			call = rtcfg_dequeue_blocking_call(ifindex);
			if (call == NULL)
				break;

			cmd_event = rtpc_get_priv(call, struct rtcfg_cmd);

			result = 0;

			/* note: only the first pending call gets data */
			if (rtcfg_dev->spec.clt.stage2_chain != NULL) {
				result = rtcfg_dev->spec.clt.chain_len;
				cmd_event->args.announce.rtskb =
					rtcfg_dev->spec.clt.stage2_chain;
				rtcfg_dev->spec.clt.stage2_chain = NULL;
			}

			rtpc_complete_call(call,
					   (cmd_event->internal.data.event_id ==
					    RTCFG_CMD_ANNOUNCE) ?
						   result :
						   -EINVAL);
		}

		rtcfg_dev->spec.clt.packet_counter = 0;
		rtcfg_dev->spec.clt.chain_len = 0;
	}
}

static void rtcfg_client_recv_stage_2_cfg(int ifindex, struct rtskb *rtskb)
{
	struct rtcfg_frm_stage_2_cfg *stage_2_cfg;
	struct rtcfg_device *rtcfg_dev = &device[ifindex];
	size_t data_len;
	int ret;

	if (rtskb->len < sizeof(struct rtcfg_frm_stage_2_cfg)) {
		rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);
		RTCFG_DEBUG(1, "RTcfg: received invalid stage_2_cfg frame\n");
		kfree_rtskb(rtskb);
		return;
	}

	stage_2_cfg = (struct rtcfg_frm_stage_2_cfg *)rtskb->data;
	__rtskb_pull(rtskb, sizeof(struct rtcfg_frm_stage_2_cfg));

	if (stage_2_cfg->heartbeat_period) {
		ret = rtdm_timer_init(&rtcfg_dev->timer, rtcfg_timer,
				      "rtcfg-timer");
		if (ret == 0) {
			ret = rtdm_timer_start(
				&rtcfg_dev->timer, XN_INFINITE,
				(nanosecs_rel_t)ntohs(
					stage_2_cfg->heartbeat_period) *
					1000000,
				RTDM_TIMERMODE_RELATIVE);
			if (ret < 0)
				rtdm_timer_destroy(&rtcfg_dev->timer);
		}

		if (ret < 0)
			/*ERRMSG*/ rtdm_printk(
				"RTcfg: unable to create timer task\n");
		else
			set_bit(FLAG_TIMER_STARTED, &rtcfg_dev->flags);
	}

	/* add server to station list */
	if (rtcfg_add_to_station_list(rtcfg_dev, rtskb->mac.ethernet->h_source,
				      stage_2_cfg->flags) < 0) {
		rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);
		RTCFG_DEBUG(1, "RTcfg: unable to process stage_2_cfg frage\n");
		kfree_rtskb(rtskb);
		return;
	}

	rtcfg_dev->other_stations = ntohl(stage_2_cfg->stations);
	rtcfg_dev->spec.clt.cfg_len = ntohl(stage_2_cfg->cfg_len);
	data_len = MIN(rtcfg_dev->spec.clt.cfg_len, rtskb->len);

	if (test_bit(RTCFG_FLAG_STAGE_2_DATA, &rtcfg_dev->flags) &&
	    (data_len > 0)) {
		rtcfg_client_queue_frag(ifindex, rtskb, data_len);
		rtskb = NULL;

		if (rtcfg_dev->stations_found == rtcfg_dev->other_stations)
			rtcfg_next_main_state(ifindex,
					      RTCFG_MAIN_CLIENT_ALL_KNOWN);
	} else {
		if (rtcfg_dev->stations_found == rtcfg_dev->other_stations) {
			rtcfg_complete_cmd(ifindex, RTCFG_CMD_ANNOUNCE, 0);

			rtcfg_next_main_state(ifindex,
					      test_bit(RTCFG_FLAG_READY,
						       &rtcfg_dev->flags) ?
						      RTCFG_MAIN_CLIENT_READY :
						      RTCFG_MAIN_CLIENT_2);
		} else
			rtcfg_next_main_state(ifindex,
					      RTCFG_MAIN_CLIENT_ALL_FRAMES);

		rtcfg_send_ack(ifindex);
	}

	rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);

	if (rtskb != NULL)
		kfree_rtskb(rtskb);
}

static void rtcfg_client_recv_stage_2_frag(int ifindex, struct rtskb *rtskb)
{
	struct rtcfg_frm_stage_2_cfg_frag *stage_2_frag;
	struct rtcfg_device *rtcfg_dev = &device[ifindex];
	size_t data_len;

	if (rtskb->len < sizeof(struct rtcfg_frm_stage_2_cfg_frag)) {
		rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);
		RTCFG_DEBUG(1,
			    "RTcfg: received invalid stage_2_cfg_frag frame\n");
		kfree_rtskb(rtskb);
		return;
	}

	stage_2_frag = (struct rtcfg_frm_stage_2_cfg_frag *)rtskb->data;
	__rtskb_pull(rtskb, sizeof(struct rtcfg_frm_stage_2_cfg_frag));

	data_len =
		MIN(rtcfg_dev->spec.clt.cfg_len - rtcfg_dev->spec.clt.cfg_offs,
		    rtskb->len);

	if (test_bit(RTCFG_FLAG_STAGE_2_DATA, &rtcfg_dev->flags) == 0) {
		RTCFG_DEBUG(1, "RTcfg: unexpected stage 2 fragment, we did not "
			       "request any data!\n");

	} else if (rtcfg_dev->spec.clt.cfg_offs !=
		   ntohl(stage_2_frag->frag_offs)) {
		RTCFG_DEBUG(1,
			    "RTcfg: unexpected stage 2 fragment (expected: %d, "
			    "received: %d)\n",
			    rtcfg_dev->spec.clt.cfg_offs,
			    ntohl(stage_2_frag->frag_offs));

		rtcfg_send_ack(ifindex);
		rtcfg_dev->spec.clt.packet_counter = 0;
	} else {
		rtcfg_client_queue_frag(ifindex, rtskb, data_len);
		rtskb = NULL;
	}

	rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);

	if (rtskb != NULL)
		kfree_rtskb(rtskb);
}

/* Notes:
 *  o On success, rtcfg_client_recv_ready returns without releasing the
 *    device lock.
 */
static int rtcfg_client_recv_ready(int ifindex, struct rtskb *rtskb)
{
	struct rtcfg_device *rtcfg_dev = &device[ifindex];
	u32 i;

	if (rtskb->len < sizeof(struct rtcfg_frm_simple)) {
		rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);
		RTCFG_DEBUG(1, "RTcfg: received invalid ready frame\n");
		kfree_rtskb(rtskb);
		return -EINVAL;
	}

	for (i = 0; i < rtcfg_dev->stations_found; i++)
		/* Ethernet-specific! */
		if (memcmp(rtcfg_dev->spec.clt.station_addr_list[i].mac_addr,
			   rtskb->mac.ethernet->h_source, ETH_ALEN) == 0) {
			if ((rtcfg_dev->spec.clt.station_addr_list[i].flags &
			     _RTCFG_FLAG_READY) == 0) {
				rtcfg_dev->spec.clt.station_addr_list[i].flags |=
					_RTCFG_FLAG_READY;
				rtcfg_dev->stations_ready++;
			}
			break;
		}

	kfree_rtskb(rtskb);
	return 0;
}

static void rtcfg_client_recv_dead_station(int ifindex, struct rtskb *rtskb)
{
	struct rtcfg_frm_dead_station *dead_station_frm;
	struct rtcfg_device *rtcfg_dev = &device[ifindex];
	u32 i;

	dead_station_frm = (struct rtcfg_frm_dead_station *)rtskb->data;

	if (rtskb->len < sizeof(struct rtcfg_frm_dead_station)) {
		rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);
		RTCFG_DEBUG(1, "RTcfg: received invalid dead station frame\n");
		kfree_rtskb(rtskb);
		return;
	}

	switch (dead_station_frm->addr_type) {
#if IS_ENABLED(CONFIG_XENO_DRIVERS_NET_RTIPV4)
	case RTCFG_ADDR_IP: {
		u32 ip;

		if (rtskb->len <
		    sizeof(struct rtcfg_frm_dead_station) + RTCFG_ADDRSIZE_IP) {
			rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);
			RTCFG_DEBUG(
				1,
				"RTcfg: received invalid dead station frame\n");
			kfree_rtskb(rtskb);
			return;
		}

		memcpy(&ip, dead_station_frm->logical_addr, 4);

		/* only delete remote IPs from routing table */
		if (rtskb->rtdev->local_ip != ip)
			rt_ip_route_del_host(ip, rtskb->rtdev);

		dead_station_frm = (struct rtcfg_frm_dead_station
					    *)(((u8 *)dead_station_frm) +
					       RTCFG_ADDRSIZE_IP);

		break;
	}
#endif /* CONFIG_XENO_DRIVERS_NET_RTIPV4 */

	case RTCFG_ADDR_MAC:
		/* nothing to do */
		break;

	default:
		rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);
		RTCFG_DEBUG(1, "RTcfg: unknown addr_type %d in %s()\n",
			    dead_station_frm->addr_type, __FUNCTION__);
		kfree_rtskb(rtskb);
		return;
	}

	for (i = 0; i < rtcfg_dev->stations_found; i++)
		/* Ethernet-specific! */
		if (memcmp(rtcfg_dev->spec.clt.station_addr_list[i].mac_addr,
			   dead_station_frm->physical_addr, ETH_ALEN) == 0) {
			if ((rtcfg_dev->spec.clt.station_addr_list[i].flags &
			     _RTCFG_FLAG_READY) != 0)
				rtcfg_dev->stations_ready--;

			rtcfg_dev->stations_found--;
			memmove(&rtcfg_dev->spec.clt.station_addr_list[i],
				&rtcfg_dev->spec.clt.station_addr_list[i + 1],
				sizeof(struct rtcfg_station) *
					(rtcfg_dev->stations_found - i));

			if (rtcfg_dev->state == RTCFG_MAIN_CLIENT_ALL_KNOWN)
				rtcfg_next_main_state(
					ifindex, RTCFG_MAIN_CLIENT_ANNOUNCED);
			break;
		}

	rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);

	kfree_rtskb(rtskb);
}

static void rtcfg_client_update_server(int ifindex, struct rtskb *rtskb)
{
	struct rtcfg_frm_stage_1_cfg *stage_1_cfg;
	struct rtcfg_device *rtcfg_dev = &device[ifindex];

	if (rtskb->len < sizeof(struct rtcfg_frm_stage_1_cfg)) {
		rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);
		RTCFG_DEBUG(1, "RTcfg: received invalid stage_1_cfg frame\n");
		kfree_rtskb(rtskb);
		return;
	}

	stage_1_cfg = (struct rtcfg_frm_stage_1_cfg *)rtskb->data;
	__rtskb_pull(rtskb, sizeof(struct rtcfg_frm_stage_1_cfg));

	switch (stage_1_cfg->addr_type) {
#if IS_ENABLED(CONFIG_XENO_DRIVERS_NET_RTIPV4)
	case RTCFG_ADDR_IP: {
		struct rtnet_device *rtdev;
		u32 daddr, saddr;

		if (rtskb->len < sizeof(struct rtcfg_frm_stage_1_cfg) +
					 2 * RTCFG_ADDRSIZE_IP) {
			rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);
			RTCFG_DEBUG(1, "RTcfg: received invalid stage_1_cfg "
				       "frame\n");
			kfree_rtskb(rtskb);
			break;
		}

		rtdev = rtskb->rtdev;

		memcpy(&daddr, stage_1_cfg->client_addr, 4);
		stage_1_cfg =
			(struct rtcfg_frm_stage_1_cfg *)(((u8 *)stage_1_cfg) +
							 RTCFG_ADDRSIZE_IP);

		memcpy(&saddr, stage_1_cfg->server_addr, 4);
		stage_1_cfg =
			(struct rtcfg_frm_stage_1_cfg *)(((u8 *)stage_1_cfg) +
							 RTCFG_ADDRSIZE_IP);

		__rtskb_pull(rtskb, 2 * RTCFG_ADDRSIZE_IP);

		/* directed to us? */
		if ((rtskb->pkt_type == PACKET_BROADCAST) &&
		    (daddr != rtdev->local_ip)) {
			rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);
			kfree_rtskb(rtskb);
			return;
		}

		/* update routing table */
		rt_ip_route_add_host(saddr, rtskb->mac.ethernet->h_source,
				     rtdev);

		rtcfg_dev->spec.clt.srv_addr.ip_addr = saddr;
		break;
	}
#endif /* CONFIG_XENO_DRIVERS_NET_RTIPV4 */

	case RTCFG_ADDR_MAC:
		/* nothing to do */
		break;

	default:
		rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);
		RTCFG_DEBUG(1, "RTcfg: unknown addr_type %d in %s()\n",
			    stage_1_cfg->addr_type, __FUNCTION__);
		kfree_rtskb(rtskb);
		return;
	}

	/* Ethernet-specific */
	memcpy(rtcfg_dev->spec.clt.srv_mac_addr, rtskb->mac.ethernet->h_source,
	       ETH_ALEN);

	rtcfg_send_announce_reply(ifindex, rtskb->mac.ethernet->h_source);

	rtdm_mutex_unlock(&rtcfg_dev->dev_mutex);

	kfree_rtskb(rtskb);
}
