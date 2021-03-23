/*
 *  RTnet - real-time networking subsystem
 *  Copyright (C) 2005-2011 Jan Kiszka <jan.kiszka@web.de>
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
 */

#ifndef _COBALT_RTDM_NET_H
#define _COBALT_RTDM_NET_H

#include <rtdm/rtdm.h>
#include <rtdm/uapi/net.h>
#include <rtdm/driver.h>

struct rtnet_callback {
    void    (*func)(struct rtdm_fd *, void *);
    void    *arg;
};

#define RTNET_RTIOC_CALLBACK    _IOW(RTIOC_TYPE_NETWORK, 0x12, \
				     struct rtnet_callback)

/* utility functions */

/* provided by rt_ipv4 */
unsigned long rt_inet_aton(const char *ip);

/* provided by rt_packet */
int rt_eth_aton(unsigned char *addr_buf, const char *mac);

#define RTNET_RTDM_VER 914

#endif  /* _COBALT_RTDM_NET_H */
