/***
 *
 *  ipv4/icmp.h
 *
 *  RTnet - real-time networking subsystem
 *  Copyright (C) 1999, 2000 Zentropic Computing, LLC
 *                2002       Ulrich Marx <marx@kammer.uni-hannover.de>
 *                2004, 2005 Jan Kiszka <jan.kiszka@web.de>
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

#ifndef __RTNET_ICMP_H_
#define __RTNET_ICMP_H_

#include <linux/init.h>

#include <rtskb.h>
#include <rtnet_rtpc.h>
#include <ipv4/protocol.h>

#define RT_ICMP_PRIO RTSKB_PRIO_VALUE(QUEUE_MIN_PRIO - 1, RTSKB_DEF_NRT_CHANNEL)

#define ICMP_REPLY_POOL_SIZE 8

void rt_icmp_queue_echo_request(struct rt_proc_call *call);
void rt_icmp_dequeue_echo_request(struct rt_proc_call *call);
void rt_icmp_cleanup_echo_requests(void);
int rt_icmp_send_echo(u32 daddr, u16 id, u16 sequence, size_t msg_size);

#ifdef CONFIG_XENO_DRIVERS_NET_RTIPV4_ICMP
void __init rt_icmp_init(void);
void rt_icmp_release(void);
#else /* !CONFIG_XENO_DRIVERS_NET_RTIPV4_ICMP */
#define rt_icmp_init()                                                         \
	do {                                                                   \
	} while (0)
#define rt_icmp_release()                                                      \
	do {                                                                   \
	} while (0)
#endif /* CONFIG_XENO_DRIVERS_NET_RTIPV4_ICMP */

#endif /* __RTNET_ICMP_H_ */
