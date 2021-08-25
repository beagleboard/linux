/***
 *
 *  include/ipv4/tcp.h
 *
 *  Copyright (C) 2009 Vladimir Zapolskiy <vladimir.zapolskiy@siemens.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License, version 2, as
 *  published by the Free Software Foundation.
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

#ifndef __RTNET_TCP_H_
#define __RTNET_TCP_H_

#include <rtskb.h>
#include <ipv4/protocol.h>

/* Maximum number of active tcp sockets, must be power of 2 */
#define RT_TCP_SOCKETS 32

/*Maximum number of active tcp connections, must be power of 2 */
#define RT_TCP_CONNECTIONS 64

/* Maximum size of TCP input window */
#define RT_TCP_WINDOW 4096

/* Maximum number of retransmissions of invalid segments */
#define RT_TCP_RETRANSMIT 3

/* Number of milliseconds to wait for ACK */
#define RT_TCP_WAIT_TIME 10

/* Priority of RST|ACK replies (error condition => non-RT prio) */
#define RT_TCP_RST_PRIO                                                        \
	RTSKB_PRIO_VALUE(QUEUE_MIN_PRIO - 1, RTSKB_DEF_NRT_CHANNEL)

/* rtskb pool for sending socket-less RST|ACK */
#define RT_TCP_RST_POOL_SIZE 8

#endif /* __RTNET_TCP_H_ */
