/**
 * This file is part of the Xenomai project.
 *
 * @note Copyright (C) 2009 Philippe Gerum <rpm@xenomai.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */
#ifndef _RTIPC_INTERNAL_H
#define _RTIPC_INTERNAL_H

#include <linux/uio.h>
#include <cobalt/kernel/registry.h>
#include <cobalt/kernel/clock.h>
#include <cobalt/kernel/select.h>
#include <rtdm/rtdm.h>
#include <rtdm/compat.h>
#include <rtdm/driver.h>

#define RTIPC_IOV_FASTMAX  16

struct rtipc_protocol;

struct rtipc_private {
	struct rtipc_protocol *proto;
	DECLARE_XNSELECT(send_block);
	DECLARE_XNSELECT(recv_block);
	void *state;
};

struct rtipc_protocol {
	const char *proto_name;
	int proto_statesz;
	int (*proto_init)(void);
	void (*proto_exit)(void);
	struct {
		int (*socket)(struct rtdm_fd *fd);
		void (*close)(struct rtdm_fd *fd);
		ssize_t (*recvmsg)(struct rtdm_fd *fd,
				   struct user_msghdr *msg, int flags);
		ssize_t (*sendmsg)(struct rtdm_fd *fd,
				   const struct user_msghdr *msg, int flags);
		ssize_t (*read)(struct rtdm_fd *fd,
				void *buf, size_t len);
		ssize_t (*write)(struct rtdm_fd *fd,
				 const void *buf, size_t len);
		int (*ioctl)(struct rtdm_fd *fd,
			     unsigned int request, void *arg);
		unsigned int (*pollstate)(struct rtdm_fd *fd);
	} proto_ops;
};

static inline void *rtipc_fd_to_state(struct rtdm_fd *fd)
{
	struct rtipc_private *p = rtdm_fd_to_private(fd);
	return p->state;
}

static inline nanosecs_rel_t rtipc_timeval_to_ns(const struct timeval *tv)
{
	nanosecs_rel_t ns = tv->tv_usec * 1000;

	if (tv->tv_sec)
		ns += (nanosecs_rel_t)tv->tv_sec * 1000000000UL;

	return ns;
}

static inline void rtipc_ns_to_timeval(struct timeval *tv, nanosecs_rel_t ns)
{
	unsigned long nsecs;

	tv->tv_sec = xnclock_divrem_billion(ns, &nsecs);
	tv->tv_usec = nsecs / 1000;
}

int rtipc_get_iovec(struct rtdm_fd *fd, struct iovec **iov,
		    const struct user_msghdr *msg,
		    struct iovec *iov_fast);

int rtipc_put_iovec(struct rtdm_fd *fd, struct iovec *iov,
		    const struct user_msghdr *msg,
		    struct iovec *iov_fast);

static inline
void rtipc_drop_iovec(struct iovec *iov, struct iovec *iov_fast)
{
	if (iov != iov_fast)
		xnfree(iov);
}

int rtipc_get_sockaddr(struct rtdm_fd *fd,
		       struct sockaddr_ipc **saddrp,
		       const void *arg);

int rtipc_put_sockaddr(struct rtdm_fd *fd, void *arg,
		       const struct sockaddr_ipc *saddr);

int rtipc_get_sockoptout(struct rtdm_fd *fd,
			 struct _rtdm_getsockopt_args *sopt,
			 const void *arg);

int rtipc_put_sockoptout(struct rtdm_fd *fd, void *arg,
			 const struct _rtdm_getsockopt_args *sopt);

int rtipc_get_sockoptin(struct rtdm_fd *fd,
			struct _rtdm_setsockopt_args *sopt,
			const void *arg);

int rtipc_get_timeval(struct rtdm_fd *fd, struct timeval *tv,
		      const void *arg, size_t arglen);

int rtipc_put_timeval(struct rtdm_fd *fd, void *arg,
		      const struct timeval *tv, size_t arglen);

int rtipc_get_length(struct rtdm_fd *fd, size_t *lenp,
		     const void *arg, size_t arglen);

int rtipc_get_arg(struct rtdm_fd *fd, void *dst, const void *src,
		  size_t len);

int rtipc_put_arg(struct rtdm_fd *fd, void *dst, const void *src,
		  size_t len);

ssize_t rtipc_get_iov_flatlen(struct iovec *iov, int iovlen);

extern struct rtipc_protocol xddp_proto_driver;

extern struct rtipc_protocol iddp_proto_driver;

extern struct rtipc_protocol bufp_proto_driver;

extern struct xnptree rtipc_ptree;

#define rtipc_wait_context		xnthread_wait_context
#define rtipc_prepare_wait		xnthread_prepare_wait
#define rtipc_get_wait_context		xnthread_get_wait_context
#define rtipc_peek_wait_head(obj)	xnsynch_peek_pendq(&(obj)->synch_base)

#define COMPAT_CASE(__op)	case __op __COMPAT_CASE(__op  ## _COMPAT)

#endif /* !_RTIPC_INTERNAL_H */
