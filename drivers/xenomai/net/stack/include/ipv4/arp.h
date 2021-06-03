/***
 *
 *  include/ipv4/arp.h - Adress Resolution Protocol for RTnet
 *
 *  RTnet - real-time networking subsystem
 *  Copyright (C) 1999,2000 Zentropic Computing, LLC
 *                2002 Ulrich Marx <marx@kammer.uni-hannover.de>
 *                2004 Jan Kiszka <jan.kiszka@web.de>
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

#ifndef __RTNET_ARP_H_
#define __RTNET_ARP_H_

#include <linux/if_arp.h>
#include <linux/init.h>
#include <linux/types.h>

#include <ipv4/route.h>

#define RT_ARP_SKB_PRIO                                                        \
	RTSKB_PRIO_VALUE(QUEUE_MIN_PRIO - 1, RTSKB_DEF_NRT_CHANNEL)

void rt_arp_send(int type, int ptype, u32 dest_ip, struct rtnet_device *rtdev,
		 u32 src_ip, unsigned char *dest_hw, unsigned char *src_hw,
		 unsigned char *target_hw);

static inline void rt_arp_solicit(struct rtnet_device *rtdev, u32 target)
{
	rt_arp_send(ARPOP_REQUEST, ETH_P_ARP, target, rtdev, rtdev->local_ip,
		    NULL, NULL, NULL);
}

void __init rt_arp_init(void);
void rt_arp_release(void);

#endif /* __RTNET_ARP_H_ */
