/***
 *
 *  ipv4/ip_sock.c
 *
 *  Copyright (C) 2003       Hans-Peter Bock <hpbock@avaapgh.de>
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

#include <linux/errno.h>
#include <linux/socket.h>
#include <linux/in.h>

#include <rtnet_socket.h>


int rt_ip_setsockopt(struct rtsocket *s, int level, int optname,
		     const void *optval, socklen_t optlen)
{
    int err = 0;


    if (level != SOL_IP)
	return -ENOPROTOOPT;

    if (optlen < sizeof(unsigned int))
	return -EINVAL;

    switch (optname) {
	case IP_TOS:
	    s->prot.inet.tos = *(unsigned int *)optval;
	    break;

	default:
	    err = -ENOPROTOOPT;
	    break;
    }

    return err;
}



int rt_ip_getsockopt(struct rtsocket *s, int level, int optname,
		     void *optval, socklen_t *optlen)
{
    int err = 0;


    if (*optlen < sizeof(unsigned int))
	return -EINVAL;

    switch (optname) {
	case IP_TOS:
	    *(unsigned int *)optval = s->prot.inet.tos;
	    *optlen = sizeof(unsigned int);
	    break;

	default:
	    err = -ENOPROTOOPT;
	    break;
    }

    return err;
}



int rt_ip_getsockname(struct rtsocket *s, struct sockaddr *addr,
		      socklen_t *addrlen)
{
    struct sockaddr_in *usin = (struct sockaddr_in *)addr;


    if (*addrlen < sizeof(struct sockaddr_in))
	return -EINVAL;

    usin->sin_family      = AF_INET;
    usin->sin_addr.s_addr = s->prot.inet.saddr;
    usin->sin_port        = s->prot.inet.sport;

    memset(usin->sin_zero, 0, sizeof(usin->sin_zero));

    *addrlen = sizeof(struct sockaddr_in);

    return 0;
}



int rt_ip_getpeername(struct rtsocket *s, struct sockaddr *addr,
		      socklen_t *addrlen)
{
    struct sockaddr_in *usin = (struct sockaddr_in *)addr;


    if (*addrlen < sizeof(struct sockaddr_in))
	return -EINVAL;

    usin->sin_family      = AF_INET;
    usin->sin_addr.s_addr = s->prot.inet.daddr;
    usin->sin_port        = s->prot.inet.dport;

    memset(usin->sin_zero, 0, sizeof(usin->sin_zero));

    *addrlen = sizeof(struct sockaddr_in);

    return 0;
}



int rt_ip_ioctl(struct rtdm_fd *fd, int request, void *arg)
{
    struct rtsocket *sock = rtdm_fd_to_private(fd);
    struct _rtdm_getsockaddr_args   *getaddr = arg;
    struct _rtdm_getsockopt_args    *getopt  = arg;
    struct _rtdm_setsockopt_args    *setopt  = arg;


    switch (request) {
	case _RTIOC_SETSOCKOPT:
	    return rt_ip_setsockopt(sock, setopt->level, setopt->optname,
				    setopt->optval, setopt->optlen);

	case _RTIOC_GETSOCKOPT:
	    return rt_ip_getsockopt(sock, getopt->level, getopt->optname,
				    getopt->optval, getopt->optlen);

	case _RTIOC_GETSOCKNAME:
	    return rt_ip_getsockname(sock, getaddr->addr, getaddr->addrlen);

	case _RTIOC_GETPEERNAME:
	    return rt_ip_getpeername(sock, getaddr->addr, getaddr->addrlen);

	default:
	    return rt_socket_if_ioctl(fd, request, arg);
    }
}
EXPORT_SYMBOL_GPL(rt_ip_ioctl);
