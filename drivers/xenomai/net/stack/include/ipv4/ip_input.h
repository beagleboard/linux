/* ipv4/ip_input.h
 *
 * RTnet - real-time networking subsystem
 * Copyright (C) 1999,2000 Zentropic Computing, LLC
 *               2002 Ulrich Marx <marx@kammer.uni-hannover.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#ifndef __RTNET_IP_INPUT_H_
#define __RTNET_IP_INPUT_H_

#include <rtskb.h>
#include <stack_mgr.h>


extern int rt_ip_rcv(struct rtskb *skb, struct rtpacket_type *pt);

#if IS_ENABLED(CONFIG_XENO_DRIVERS_NET_ADDON_PROXY)
typedef void (*rt_ip_fallback_handler_t)(struct rtskb *skb);

/*
 * This hook can be used to register a fallback handler for incoming
 * IP packets. Typically this is done to move over to the standard Linux
 * IP protocol (e.g. for handling TCP).
 * Manipulating the fallback handler is expected to happen only when the
 * RTnetinterfaces are shut down (avoiding race conditions).
 *
 * Note that merging RT and non-RT traffic this way most likely breaks hard
 * real-time constraints!
 */
extern rt_ip_fallback_handler_t rt_ip_fallback_handler;
#endif


#endif  /* __RTNET_IP_INPUT_H_ */
