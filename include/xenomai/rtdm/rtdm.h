/*
 * Copyright (C) 2005, 2006 Jan Kiszka <jan.kiszka@web.de>
 * Copyright (C) 2005 Joerg Langenberg <joerg.langenberg@gmx.net>
 *
 * Xenomai is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Xenomai is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Xenomai; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#ifndef _COBALT_RTDM_RTDM_H
#define _COBALT_RTDM_RTDM_H

#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/ioctl.h>
#include <linux/sched.h>
#include <linux/socket.h>
#include <cobalt/kernel/ppd.h>
#include <rtdm/fd.h>

typedef __u32 socklen_t;

#include <rtdm/uapi/rtdm.h>

int __rtdm_dev_open(const char *path, int oflag);

int __rtdm_dev_socket(int protocol_family,
		      int socket_type, int protocol);

static inline int rtdm_open(const char *path, int oflag, ...)
{
	return __rtdm_dev_open(path, oflag);
}

static inline int rtdm_socket(int protocol_family,
			      int socket_type, int protocol)
{
	return __rtdm_dev_socket(protocol_family, socket_type, protocol);
}

static inline int rtdm_close(int fd)
{
	return rtdm_fd_close(fd, RTDM_FD_MAGIC);
}

#define rtdm_fcntl(__fd, __cmd, __args...)	\
	rtdm_fd_fcntl(__fd, __cmd, ##__args)

#define rtdm_ioctl(__fd, __request, __args...)	\
	rtdm_fd_ioctl(__fd, __request, ##__args)

static inline ssize_t rtdm_read(int fd, void *buf, size_t count)
{
	return rtdm_fd_read(fd, buf, count);
}

static inline ssize_t rtdm_write(int fd, const void *buf, size_t count)
{
	return rtdm_fd_write(fd, buf, count);
}

static inline ssize_t rtdm_recvmsg(int s, struct user_msghdr *msg, int flags)
{
	return rtdm_fd_recvmsg(s, msg, flags);
}

static inline ssize_t rtdm_sendmsg(int s, const struct user_msghdr *msg, int flags)
{
	return rtdm_fd_sendmsg(s, msg, flags);
}

static inline
ssize_t rtdm_recvfrom(int s, void *buf, size_t len, int flags,
		      struct sockaddr *from,
		      socklen_t *fromlen)
{
	struct user_msghdr msg;
	struct iovec iov;
	ssize_t ret;

	iov.iov_base = buf;
	iov.iov_len = len;
	msg.msg_name = from;
	msg.msg_namelen = from ? *fromlen : 0;
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;
	msg.msg_control = NULL;
	msg.msg_controllen = 0;

	ret = rtdm_recvmsg(s, &msg, flags);
	if (ret < 0)
		return ret;

	if (from)
		*fromlen = msg.msg_namelen;

	return ret;
}

static inline ssize_t rtdm_recv(int s, void *buf, size_t len, int flags)
{
	return rtdm_recvfrom(s, buf, len, flags, NULL, NULL);
}

static inline ssize_t rtdm_sendto(int s, const void *buf, size_t len,
				  int flags, const struct sockaddr *to,
				  socklen_t tolen)
{
	struct user_msghdr msg;
	struct iovec iov;

	iov.iov_base = (void *)buf;
	iov.iov_len = len;
	msg.msg_name = (struct sockaddr *)to;
	msg.msg_namelen = tolen;
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;
	msg.msg_control = NULL;
	msg.msg_controllen = 0;

	return rtdm_sendmsg(s, &msg, flags);
}

static inline ssize_t rtdm_send(int s, const void *buf, size_t len, int flags)
{
	return rtdm_sendto(s, buf, len, flags, NULL, 0);
}

static inline int rtdm_getsockopt(int s, int level, int optname,
				  void *optval, socklen_t *optlen)
{
	struct _rtdm_getsockopt_args args = {
		level, optname, optval, optlen
	};

	return rtdm_ioctl(s, _RTIOC_GETSOCKOPT, &args);
}

static inline int rtdm_setsockopt(int s, int level, int optname,
				  const void *optval, socklen_t optlen)
{
	struct _rtdm_setsockopt_args args = {
		level, optname, (void *)optval, optlen
	};

	return rtdm_ioctl(s, _RTIOC_SETSOCKOPT, &args);
}

static inline int rtdm_bind(int s, const struct sockaddr *my_addr,
			    socklen_t addrlen)
{
	struct _rtdm_setsockaddr_args args = {
		my_addr, addrlen
	};

	return rtdm_ioctl(s, _RTIOC_BIND, &args);
}

static inline int rtdm_connect(int s, const struct sockaddr *serv_addr,
			       socklen_t addrlen)
{
	struct _rtdm_setsockaddr_args args = {
		serv_addr, addrlen
	};

	return rtdm_ioctl(s, _RTIOC_CONNECT, &args);
}

static inline int rtdm_listen(int s, int backlog)
{
	return rtdm_ioctl(s, _RTIOC_LISTEN, backlog);
}

static inline int rtdm_accept(int s, struct sockaddr *addr,
			      socklen_t *addrlen)
{
	struct _rtdm_getsockaddr_args args = {
		addr, addrlen
	};

	return rtdm_ioctl(s, _RTIOC_ACCEPT, &args);
}

static inline int rtdm_getsockname(int s, struct sockaddr *name,
				   socklen_t *namelen)
{
	struct _rtdm_getsockaddr_args args = {
		name, namelen
	};

	return rtdm_ioctl(s, _RTIOC_GETSOCKNAME, &args);
}

static inline int rtdm_getpeername(int s, struct sockaddr *name,
				   socklen_t *namelen)
{
	struct _rtdm_getsockaddr_args args = {
		name, namelen
	};

	return rtdm_ioctl(s, _RTIOC_GETPEERNAME, &args);
}

static inline int rtdm_shutdown(int s, int how)
{
	return rtdm_ioctl(s, _RTIOC_SHUTDOWN, how);
}

#endif /* _COBALT_RTDM_RTDM_H */
