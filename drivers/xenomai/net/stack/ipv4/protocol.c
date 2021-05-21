/***
 *
 *  ipv4/protocol.c
 *
 *  rtnet - real-time networking subsystem
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

#include <linux/socket.h>
#include <linux/in.h>

#include <rtnet_socket.h>
#include <ipv4/protocol.h>


struct rtinet_protocol *rt_inet_protocols[MAX_RT_INET_PROTOCOLS];

/***
 * rt_inet_add_protocol
 */
void rt_inet_add_protocol(struct rtinet_protocol *prot)
{
    unsigned char hash = rt_inet_hashkey(prot->protocol);


    if ( rt_inet_protocols[hash]==NULL )
	rt_inet_protocols[hash] = prot;
}
EXPORT_SYMBOL_GPL(rt_inet_add_protocol);


/***
 * rt_inet_del_protocol
 */
void rt_inet_del_protocol(struct rtinet_protocol *prot)
{
    unsigned char hash = rt_inet_hashkey(prot->protocol);


    if ( prot==rt_inet_protocols[hash] )
	rt_inet_protocols[hash] = NULL;
}
EXPORT_SYMBOL_GPL(rt_inet_del_protocol);



/***
 * rt_inet_socket - initialize an Internet socket
 * @sock: socket structure
 * @protocol: protocol id
 */
int rt_inet_socket(struct rtdm_fd *fd, int protocol)
{
    struct rtinet_protocol  *prot;


    if (protocol == 0)
	switch (rtdm_fd_to_context(fd)->device->driver->socket_type) {
	case SOCK_DGRAM:
	    protocol = IPPROTO_UDP;
	    break;
	case SOCK_STREAM:
	    protocol = IPPROTO_TCP;
	    break;
	}

    prot = rt_inet_protocols[rt_inet_hashkey(protocol)];

    /* create the socket (call the socket creator) */
    if ((prot != NULL) && (prot->protocol == protocol))
	return prot->init_socket(fd);
    else {
	rtdm_printk("RTnet: protocol with id %d not found\n", protocol);

	return -ENOPROTOOPT;
    }
}
EXPORT_SYMBOL_GPL(rt_inet_socket);
