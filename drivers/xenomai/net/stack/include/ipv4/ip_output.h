/***
 *
 *  include/ipv4/ip_output.h - prepare outgoing IP packets
 *
 *  RTnet - real-time networking subsystem
 *  Copyright (C) 1999,2000 Zentropic Computing, LLC
 *                2002 Ulrich Marx <marx@kammer.uni-hannover.de>
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

#ifndef __RTNET_IP_OUTPUT_H_
#define __RTNET_IP_OUTPUT_H_

#include <linux/init.h>

#include <rtdev.h>
#include <ipv4/route.h>

extern int rt_ip_build_xmit(struct rtsocket *sk,
			    int getfrag(const void *, unsigned char *,
					unsigned int, unsigned int),
			    const void *frag, unsigned length,
			    struct dest_route *rt, int flags);

extern void __init rt_ip_init(void);
extern void rt_ip_release(void);

#endif /* __RTNET_IP_OUTPUT_H_ */
