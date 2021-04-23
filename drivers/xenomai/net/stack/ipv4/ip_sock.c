/***
 *
 *  ipv4/ip_sock.c
 *
 *  Copyright (C) 2003       Hans-Peter Bock <hpbock@avaapgh.de>
 *                2004, 2005 Jan Kiszka <jan.kiszka@web.de>
 *                2019       Sebastian Smolorz <sebastian.smolorz@gmx.de>
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

int rt_ip_setsockopt(struct rtdm_fd *fd, struct rtsocket *s, int level,
		     int optname, const void __user *optval, socklen_t optlen)
{
	int err = 0;
	unsigned int _tos, *tos;

	if (level != SOL_IP)
		return -ENOPROTOOPT;

	if (optlen < sizeof(unsigned int))
		return -EINVAL;

	switch (optname) {
	case IP_TOS:
		tos = rtnet_get_arg(fd, &_tos, optval, sizeof(_tos));
		if (IS_ERR(tos))
			return PTR_ERR(tos);
		else
			s->prot.inet.tos = *tos;
		break;

	default:
		err = -ENOPROTOOPT;
		break;
	}

	return err;
}

int rt_ip_getsockopt(struct rtdm_fd *fd, struct rtsocket *s, int level,
		     int optname, void __user *optval, socklen_t __user *optlen)
{
	int err = 0;
	unsigned int tos;
	socklen_t _len, *len;

	len = rtnet_get_arg(fd, &_len, optlen, sizeof(_len));
	if (IS_ERR(len))
		return PTR_ERR(len);

	if (*len < sizeof(unsigned int))
		return -EINVAL;

	switch (optname) {
	case IP_TOS:
		tos = s->prot.inet.tos;
		err = rtnet_put_arg(fd, optval, &tos, sizeof(tos));
		if (!err) {
			*len = sizeof(unsigned int);
			err = rtnet_put_arg(fd, optlen, len, sizeof(socklen_t));
		}
		break;

	default:
		err = -ENOPROTOOPT;
		break;
	}

	return err;
}

int rt_ip_getsockname(struct rtdm_fd *fd, struct rtsocket *s,
		      struct sockaddr __user *addr, socklen_t __user *addrlen)
{
	struct sockaddr_in _sin;
	socklen_t *len, _len;
	int ret;

	len = rtnet_get_arg(fd, &_len, addrlen, sizeof(_len));
	if (IS_ERR(len))
		return PTR_ERR(len);

	if (*len < sizeof(struct sockaddr_in))
		return -EINVAL;

	_sin.sin_family = AF_INET;
	_sin.sin_addr.s_addr = s->prot.inet.saddr;
	_sin.sin_port = s->prot.inet.sport;
	memset(&_sin.sin_zero, 0, sizeof(_sin.sin_zero));
	ret = rtnet_put_arg(fd, addr, &_sin, sizeof(_sin));
	if (ret)
		return ret;

	*len = sizeof(struct sockaddr_in);
	ret = rtnet_put_arg(fd, addrlen, len, sizeof(socklen_t));

	return ret;
}

int rt_ip_getpeername(struct rtdm_fd *fd, struct rtsocket *s,
		      struct sockaddr __user *addr, socklen_t __user *addrlen)
{
	struct sockaddr_in _sin;
	socklen_t *len, _len;
	int ret;

	len = rtnet_get_arg(fd, &_len, addrlen, sizeof(_len));
	if (IS_ERR(len))
		return PTR_ERR(len);

	if (*len < sizeof(struct sockaddr_in))
		return -EINVAL;

	_sin.sin_family = AF_INET;
	_sin.sin_addr.s_addr = s->prot.inet.daddr;
	_sin.sin_port = s->prot.inet.dport;
	memset(&_sin.sin_zero, 0, sizeof(_sin.sin_zero));
	ret = rtnet_put_arg(fd, addr, &_sin, sizeof(_sin));
	if (ret)
		return ret;

	*len = sizeof(struct sockaddr_in);
	ret = rtnet_put_arg(fd, addrlen, len, sizeof(socklen_t));

	return ret;
}

int rt_ip_ioctl(struct rtdm_fd *fd, int request, void __user *arg)
{
	struct rtsocket *sock = rtdm_fd_to_private(fd);
	struct _rtdm_getsockaddr_args _getaddr, *getaddr;
	struct _rtdm_getsockopt_args _getopt, *getopt;
	struct _rtdm_setsockopt_args _setopt, *setopt;

	switch (request) {
	case _RTIOC_SETSOCKOPT:
		setopt = rtnet_get_arg(fd, &_setopt, arg, sizeof(_setopt));
		if (IS_ERR(setopt))
			return PTR_ERR(setopt);

		return rt_ip_setsockopt(fd, sock, setopt->level,
					setopt->optname, setopt->optval,
					setopt->optlen);

	case _RTIOC_GETSOCKOPT:
		getopt = rtnet_get_arg(fd, &_getopt, arg, sizeof(_getopt));
		if (IS_ERR(getopt))
			return PTR_ERR(getopt);

		return rt_ip_getsockopt(fd, sock, getopt->level,
					getopt->optname, getopt->optval,
					getopt->optlen);

	case _RTIOC_GETSOCKNAME:
		getaddr = rtnet_get_arg(fd, &_getaddr, arg, sizeof(_getaddr));
		if (IS_ERR(getaddr))
			return PTR_ERR(getaddr);

		return rt_ip_getsockname(fd, sock, getaddr->addr,
					 getaddr->addrlen);

	case _RTIOC_GETPEERNAME:
		getaddr = rtnet_get_arg(fd, &_getaddr, arg, sizeof(_getaddr));
		if (IS_ERR(getaddr))
			return PTR_ERR(getaddr);

		return rt_ip_getpeername(fd, sock, getaddr->addr,
					 getaddr->addrlen);

	default:
		return rt_socket_if_ioctl(fd, request, arg);
	}
}
EXPORT_SYMBOL_GPL(rt_ip_ioctl);
