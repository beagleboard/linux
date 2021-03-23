/***
 *
 *  include/ipv4.h
 *
 *  Real-Time IP/UDP/ICMP stack
 *
 *  Copyright (C) 2004 Jan Kiszka <jan.kiszka@web.de>
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

#ifndef __IPV4_H_
#define __RTCFG_H_

#include <rtnet_chrdev.h>

struct ipv4_cmd {
	struct rtnet_ioctl_head head;

	union {
		/*** rtroute ***/
		struct {
			__u32 ip_addr;
		} solicit;

		struct {
			__u8 dev_addr[DEV_ADDR_LEN];
			__u32 ip_addr;
		} gethost;

		struct {
			__u8 dev_addr[DEV_ADDR_LEN];
			__u32 ip_addr;
		} addhost;

		struct {
			__u32 ip_addr;
		} delhost;

		struct {
			__u32 net_addr;
			__u32 net_mask;
			__u32 gw_addr;
		} addnet;

		struct {
			__u32 net_addr;
			__u32 net_mask;
		} delnet;

		/*** rtping ***/
		struct {
			__u32 ip_addr;
			__u16 id;
			__u16 sequence;
			__u32 msg_size;
			__u32 timeout;
			__s64 rtt;
		} ping;

		__u64 __padding[8];
	} args;
};

#define IOC_RT_HOST_ROUTE_ADD _IOW(RTNET_IOC_TYPE_IPV4, 0, struct ipv4_cmd)
#define IOC_RT_HOST_ROUTE_SOLICIT _IOW(RTNET_IOC_TYPE_IPV4, 1, struct ipv4_cmd)
#define IOC_RT_HOST_ROUTE_DELETE                                               \
	_IOW(RTNET_IOC_TYPE_IPV4, 2 | RTNET_IOC_NODEV_PARAM, struct ipv4_cmd)
#define IOC_RT_NET_ROUTE_ADD                                                   \
	_IOW(RTNET_IOC_TYPE_IPV4, 3 | RTNET_IOC_NODEV_PARAM, struct ipv4_cmd)
#define IOC_RT_NET_ROUTE_DELETE                                                \
	_IOW(RTNET_IOC_TYPE_IPV4, 4 | RTNET_IOC_NODEV_PARAM, struct ipv4_cmd)
#define IOC_RT_PING                                                            \
	_IOWR(RTNET_IOC_TYPE_IPV4, 5 | RTNET_IOC_NODEV_PARAM, struct ipv4_cmd)
#define IOC_RT_HOST_ROUTE_DELETE_DEV                                           \
	_IOW(RTNET_IOC_TYPE_IPV4, 6, struct ipv4_cmd)
#define IOC_RT_HOST_ROUTE_GET                                                  \
	_IOWR(RTNET_IOC_TYPE_IPV4, 7 | RTNET_IOC_NODEV_PARAM, struct ipv4_cmd)
#define IOC_RT_HOST_ROUTE_GET_DEV _IOWR(RTNET_IOC_TYPE_IPV4, 8, struct ipv4_cmd)

#endif /* __IPV4_H_ */
