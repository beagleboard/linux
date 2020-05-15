/* ipv4/ip_fragment.h
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
#ifndef __RTNET_IP_FRAGMENT_H_
#define __RTNET_IP_FRAGMENT_H_

#include <linux/init.h>

#include <rtskb.h>
#include <ipv4/protocol.h>


extern struct rtskb *rt_ip_defrag(struct rtskb *skb,
                                  struct rtinet_protocol *ipprot);

extern void rt_ip_frag_invalidate_socket(struct rtsocket *sock);

extern int __init rt_ip_fragment_init(void);
extern void rt_ip_fragment_cleanup(void);


#endif  /* __RTNET_IP_FRAGMENT_H_ */
