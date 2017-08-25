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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <rtdm/ipc.h>
#include <rtdm/compat.h>
#include "internal.h"

MODULE_DESCRIPTION("Real-time IPC interface");
MODULE_AUTHOR("Philippe Gerum <rpm@xenomai.org>");
MODULE_LICENSE("GPL");

static struct rtipc_protocol *protocols[IPCPROTO_MAX] = {
#ifdef CONFIG_XENO_DRIVERS_RTIPC_XDDP
	[IPCPROTO_XDDP - 1] = &xddp_proto_driver,
#endif
#ifdef CONFIG_XENO_DRIVERS_RTIPC_IDDP
	[IPCPROTO_IDDP - 1] = &iddp_proto_driver,
#endif
#ifdef CONFIG_XENO_DRIVERS_RTIPC_BUFP
	[IPCPROTO_BUFP - 1] = &bufp_proto_driver,
#endif
};

DEFINE_XNPTREE(rtipc_ptree, "rtipc");

int rtipc_get_arg(struct rtdm_fd *fd, void *dst, const void *src, size_t len)
{
	if (!rtdm_fd_is_user(fd)) {
		memcpy(dst, src, len);
		return 0;
	}

	return rtdm_copy_from_user(fd, dst, src, len);
}

int rtipc_put_arg(struct rtdm_fd *fd, void *dst, const void *src, size_t len)
{
	if (!rtdm_fd_is_user(fd)) {
		memcpy(dst, src, len);
		return 0;
	}

	return rtdm_copy_to_user(fd, dst, src, len);
}

int rtipc_get_iovec(struct rtdm_fd *fd, struct iovec **iovp,
		    const struct user_msghdr *msg,
		    struct iovec *iov_fast)
{
	size_t len = sizeof(struct iovec) * msg->msg_iovlen;
	struct iovec *iov = iov_fast;

	/*
	 * If the I/O vector doesn't fit in the fast memory, allocate
	 * a chunk from the system heap which is large enough to hold
	 * it.
	 */
	if (msg->msg_iovlen > RTIPC_IOV_FASTMAX) {
		iov = xnmalloc(len);
		if (iov == NULL)
			return -ENOMEM;
	}

	*iovp = iov;

	if (!rtdm_fd_is_user(fd)) {
		memcpy(iov, msg->msg_iov, len);
		return 0;
	}

#ifdef CONFIG_XENO_ARCH_SYS3264
	if (rtdm_fd_is_compat(fd))
		return sys32_get_iovec(iov,
			       (struct compat_iovec __user *)msg->msg_iov,
			       msg->msg_iovlen);
#endif

	return rtdm_copy_from_user(fd, iov, msg->msg_iov, len);
}

int rtipc_put_iovec(struct rtdm_fd *fd, struct iovec *iov,
		    const struct user_msghdr *msg,
		    struct iovec *iov_fast)
{
	size_t len = sizeof(iov[0]) * msg->msg_iovlen;
	int ret;

	if (!rtdm_fd_is_user(fd)) {
		memcpy(msg->msg_iov, iov, len);
		ret = 0;
	} else
#ifdef CONFIG_XENO_ARCH_SYS3264
		if (rtdm_fd_is_compat(fd))
			ret = sys32_put_iovec((struct compat_iovec __user *)msg->msg_iov,
					      iov, msg->msg_iovlen);
		else
#endif
			ret = rtdm_copy_to_user(fd, msg->msg_iov, iov, len);

	if (iov != iov_fast)
		xnfree(iov);

	return ret;
}

int rtipc_get_sockaddr(struct rtdm_fd *fd, struct sockaddr_ipc **saddrp,
		       const void *arg)
{
	const struct _rtdm_setsockaddr_args *p;
	struct _rtdm_setsockaddr_args sreq;
	int ret;

	if (!rtdm_fd_is_user(fd)) {
		p = arg;
		if (p->addrlen > 0) {
			if (p->addrlen != sizeof(**saddrp))
				return -EINVAL;
			memcpy(*saddrp, p->addr, sizeof(**saddrp));
		} else {
			if (p->addr)
				return -EINVAL;
			*saddrp = NULL;
		}
		return 0;
	}

#ifdef CONFIG_XENO_ARCH_SYS3264
	if (rtdm_fd_is_compat(fd)) {
		struct compat_rtdm_setsockaddr_args csreq;
		ret = rtdm_safe_copy_from_user(fd, &csreq, arg, sizeof(csreq));
		if (ret)
			return ret;
		if (csreq.addrlen > 0) {
			if (csreq.addrlen != sizeof(**saddrp))
				return -EINVAL;
			return rtdm_safe_copy_from_user(fd, *saddrp,
							compat_ptr(csreq.addr),
							sizeof(**saddrp));
		}
		if (csreq.addr)
			return -EINVAL;

		*saddrp = NULL;

		return 0;
	}
#endif

	ret = rtdm_safe_copy_from_user(fd, &sreq, arg, sizeof(sreq));
	if (ret)
		return ret;
	if (sreq.addrlen > 0) {
		if (sreq.addrlen != sizeof(**saddrp))
			return -EINVAL;
		return rtdm_safe_copy_from_user(fd, *saddrp,
						sreq.addr, sizeof(**saddrp));
	}
	if (sreq.addr)
		return -EINVAL;

	*saddrp = NULL;

	return 0;
}

int rtipc_put_sockaddr(struct rtdm_fd *fd, void *arg,
		       const struct sockaddr_ipc *saddr)
{
	const struct _rtdm_getsockaddr_args *p;
	struct _rtdm_getsockaddr_args sreq;
	socklen_t len;
	int ret;

	if (!rtdm_fd_is_user(fd)) {
		p = arg;
		if (*p->addrlen < sizeof(*saddr))
			return -EINVAL;
		memcpy(p->addr, saddr, sizeof(*saddr));
		*p->addrlen = sizeof(*saddr);
		return 0;
	}

#ifdef CONFIG_XENO_ARCH_SYS3264
	if (rtdm_fd_is_compat(fd)) {
		struct compat_rtdm_getsockaddr_args csreq;
		ret = rtdm_safe_copy_from_user(fd, &csreq, arg, sizeof(csreq));
		if (ret)
			return ret;

		ret = rtdm_safe_copy_from_user(fd, &len,
					       compat_ptr(csreq.addrlen),
					       sizeof(len));
		if (ret)
			return ret;

		if (len < sizeof(*saddr))
			return -EINVAL;

		ret = rtdm_safe_copy_to_user(fd, compat_ptr(csreq.addr),
					     saddr, sizeof(*saddr));
		if (ret)
			return ret;

		len = sizeof(*saddr);
		return rtdm_safe_copy_to_user(fd, compat_ptr(csreq.addrlen),
					      &len, sizeof(len));
	}
#endif

	sreq.addr = NULL;
	sreq.addrlen = NULL;
	ret = rtdm_safe_copy_from_user(fd, &sreq, arg, sizeof(sreq));
	if (ret)
		return ret;

	ret = rtdm_safe_copy_from_user(fd, &len, sreq.addrlen, sizeof(len));
	if (ret)
		return ret;

	if (len < sizeof(*saddr))
		return -EINVAL;

	ret = rtdm_safe_copy_to_user(fd, sreq.addr, saddr, sizeof(*saddr));
	if (ret)
		return ret;

	len = sizeof(*saddr);

	return rtdm_safe_copy_to_user(fd, sreq.addrlen, &len, sizeof(len));
}

int rtipc_get_sockoptout(struct rtdm_fd *fd, struct _rtdm_getsockopt_args *sopt,
			 const void *arg)
{
	if (!rtdm_fd_is_user(fd)) {
		*sopt = *(struct _rtdm_getsockopt_args *)arg;
		return 0;
	}

#ifdef CONFIG_XENO_ARCH_SYS3264
	if (rtdm_fd_is_compat(fd)) {
		struct compat_rtdm_getsockopt_args csopt;
		int ret;
		ret = rtdm_safe_copy_from_user(fd, &csopt, arg, sizeof(csopt));
		if (ret)
			return ret;
		sopt->level = csopt.level;
		sopt->optname = csopt.optname;
		sopt->optval = compat_ptr(csopt.optval);
		sopt->optlen = compat_ptr(csopt.optlen);
		return 0;
	}
#endif

	return rtdm_safe_copy_from_user(fd, sopt, arg, sizeof(*sopt));
}

int rtipc_put_sockoptout(struct rtdm_fd *fd, void *arg,
			 const struct _rtdm_getsockopt_args *sopt)
{
	if (!rtdm_fd_is_user(fd)) {
		*(struct _rtdm_getsockopt_args *)arg = *sopt;
		return 0;
	}

#ifdef CONFIG_XENO_ARCH_SYS3264
	if (rtdm_fd_is_compat(fd)) {
		struct compat_rtdm_getsockopt_args csopt;
		int ret;
		csopt.level = sopt->level;
		csopt.optname = sopt->optname;
		csopt.optval = ptr_to_compat(sopt->optval);
		csopt.optlen = ptr_to_compat(sopt->optlen);
		ret = rtdm_safe_copy_to_user(fd, arg, &csopt, sizeof(csopt));
		if (ret)
			return ret;
		return 0;
	}
#endif

	return rtdm_safe_copy_to_user(fd, arg, sopt, sizeof(*sopt));
}

int rtipc_get_sockoptin(struct rtdm_fd *fd, struct _rtdm_setsockopt_args *sopt,
			const void *arg)
{
	if (!rtdm_fd_is_user(fd)) {
		*sopt = *(struct _rtdm_setsockopt_args *)arg;
		return 0;
	}

#ifdef CONFIG_XENO_ARCH_SYS3264
	if (rtdm_fd_is_compat(fd)) {
		struct compat_rtdm_setsockopt_args csopt;
		int ret;
		ret = rtdm_safe_copy_from_user(fd, &csopt, arg, sizeof(csopt));
		if (ret)
			return ret;
		sopt->level = csopt.level;
		sopt->optname = csopt.optname;
		sopt->optval = compat_ptr(csopt.optval);
		sopt->optlen = csopt.optlen;
		return 0;
	}
#endif

	return rtdm_safe_copy_from_user(fd, sopt, arg, sizeof(*sopt));
}

int rtipc_get_timeval(struct rtdm_fd *fd, struct timeval *tv,
		      const void *arg, size_t arglen)
{
#ifdef CONFIG_XENO_ARCH_SYS3264
	if (rtdm_fd_is_compat(fd)) {
		if (arglen != sizeof(struct compat_timeval))
			return -EINVAL;
		return sys32_get_timeval(tv, arg);
	}
#endif

	if (arglen != sizeof(*tv))
		return -EINVAL;

	if (!rtdm_fd_is_user(fd)) {
		*tv = *(struct timeval *)arg;
		return 0;
	}

	return rtdm_safe_copy_from_user(fd, tv, arg, sizeof(*tv));
}

int rtipc_put_timeval(struct rtdm_fd *fd, void *arg,
		      const struct timeval *tv, size_t arglen)
{
#ifdef CONFIG_XENO_ARCH_SYS3264
	if (rtdm_fd_is_compat(fd)) {
		if (arglen != sizeof(struct compat_timeval))
			return -EINVAL;
		return sys32_put_timeval(arg, tv);
	}
#endif

	if (arglen != sizeof(*tv))
		return -EINVAL;

	if (!rtdm_fd_is_user(fd)) {
		*(struct timeval *)arg = *tv;
		return 0;
	}

	return rtdm_safe_copy_to_user(fd, arg, tv, sizeof(*tv));
}

int rtipc_get_length(struct rtdm_fd *fd, size_t *lenp,
		     const void *arg, size_t arglen)
{
#ifdef CONFIG_XENO_ARCH_SYS3264
	if (rtdm_fd_is_compat(fd)) {
		const compat_size_t *csz;
		if (arglen != sizeof(*csz))
			return -EINVAL;
		csz = arg;
		return csz == NULL ||
			!access_rok(csz, sizeof(*csz)) ||
			__xn_get_user(*lenp, csz) ? -EFAULT : 0;
	}
#endif

	if (arglen != sizeof(size_t))
		return -EINVAL;

	if (!rtdm_fd_is_user(fd)) {
		*lenp = *(size_t *)arg;
		return 0;
	}

	return rtdm_safe_copy_from_user(fd, lenp, arg, sizeof(*lenp));
}

ssize_t rtipc_get_iov_flatlen(struct iovec *iov, int iovlen)
{
	ssize_t len;
	int nvec;

	/* Return the flattened vector length. */
	for (len = 0, nvec = 0; nvec < iovlen; nvec++) {
		ssize_t l = iov[nvec].iov_len;
		if (l < 0 || len + l < len) /* SuS wants this. */
			return -EINVAL;
		len += l;
	}

	return len;
}

static int rtipc_socket(struct rtdm_fd *fd, int protocol)
{
	struct rtipc_protocol *proto;
	struct rtipc_private *priv;
	int ret;

	if (protocol < 0 || protocol >= IPCPROTO_MAX)
		return -EPROTONOSUPPORT;

	if (protocol == IPCPROTO_IPC)
		/* Default protocol is IDDP */
		protocol = IPCPROTO_IDDP;

	proto = protocols[protocol - 1];
	if (proto == NULL)	/* Not compiled in? */
		return -ENOPROTOOPT;

	priv = rtdm_fd_to_private(fd);
	priv->proto = proto;
	priv->state = kmalloc(proto->proto_statesz, GFP_KERNEL);
	if (priv->state == NULL)
		return -ENOMEM;

	xnselect_init(&priv->send_block);
	xnselect_init(&priv->recv_block);

	ret = proto->proto_ops.socket(fd);
	if (ret)
		kfree(priv->state);

	return ret;
}

static void rtipc_close(struct rtdm_fd *fd)
{
	struct rtipc_private *priv = rtdm_fd_to_private(fd);
	/*
	 * CAUTION: priv->state shall be released by the
	 * proto_ops.close() handler when appropriate (which may be
	 * done asynchronously later, see XDDP).
	 */
	priv->proto->proto_ops.close(fd);
	xnselect_destroy(&priv->recv_block);
	xnselect_destroy(&priv->send_block);
}

static ssize_t rtipc_recvmsg(struct rtdm_fd *fd,
			     struct user_msghdr *msg, int flags)
{
	struct rtipc_private *priv = rtdm_fd_to_private(fd);
	return priv->proto->proto_ops.recvmsg(fd, msg, flags);
}

static ssize_t rtipc_sendmsg(struct rtdm_fd *fd,
			     const struct user_msghdr *msg, int flags)
{
	struct rtipc_private *priv = rtdm_fd_to_private(fd);
	return priv->proto->proto_ops.sendmsg(fd, msg, flags);
}

static ssize_t rtipc_read(struct rtdm_fd *fd,
			  void *buf, size_t len)
{
	struct rtipc_private *priv = rtdm_fd_to_private(fd);
	return priv->proto->proto_ops.read(fd, buf, len);
}

static ssize_t rtipc_write(struct rtdm_fd *fd,
			   const void *buf, size_t len)
{
	struct rtipc_private *priv = rtdm_fd_to_private(fd);
	return priv->proto->proto_ops.write(fd, buf, len);
}

static int rtipc_ioctl(struct rtdm_fd *fd,
		       unsigned int request, void *arg)
{
	struct rtipc_private *priv = rtdm_fd_to_private(fd);
	return priv->proto->proto_ops.ioctl(fd, request, arg);
}

static int rtipc_select(struct rtdm_fd *fd, struct xnselector *selector,
			unsigned int type, unsigned int index)
{
	struct rtipc_private *priv = rtdm_fd_to_private(fd);
	struct xnselect_binding *binding;
	unsigned int pollstate, mask;
	struct xnselect *block;
	spl_t s;
	int ret;
	
	if (type != XNSELECT_READ && type != XNSELECT_WRITE)
		return -EINVAL;

	binding = xnmalloc(sizeof(*binding));
	if (binding == NULL)
		return -ENOMEM;

	cobalt_atomic_enter(s);

	pollstate = priv->proto->proto_ops.pollstate(fd);

	if (type == XNSELECT_READ) {
		mask = pollstate & POLLIN;
		block = &priv->recv_block;
	} else {
		mask = pollstate & POLLOUT;
		block = &priv->send_block;
	}

	ret = xnselect_bind(block, binding, selector, type, index, mask);

	cobalt_atomic_leave(s);

	if (ret)
		xnfree(binding);

	return ret;
}

static struct rtdm_driver rtipc_driver = {
	.profile_info		=	RTDM_PROFILE_INFO(rtipc,
							  RTDM_CLASS_RTIPC,
							  RTDM_SUBCLASS_GENERIC,
							  1),
	.device_flags		=	RTDM_PROTOCOL_DEVICE,
	.device_count		=	1,
	.context_size		=	sizeof(struct rtipc_private),
	.protocol_family	=	PF_RTIPC,
	.socket_type		=	SOCK_DGRAM,
	.ops = {
		.socket		=	rtipc_socket,
		.close		=	rtipc_close,
		.recvmsg_rt	=	rtipc_recvmsg,
		.recvmsg_nrt	=	NULL,
		.sendmsg_rt	=	rtipc_sendmsg,
		.sendmsg_nrt	=	NULL,
		.ioctl_rt	=	rtipc_ioctl,
		.ioctl_nrt	=	rtipc_ioctl,
		.read_rt	=	rtipc_read,
		.read_nrt	=	NULL,
		.write_rt	=	rtipc_write,
		.write_nrt	=	NULL,
		.select		=	rtipc_select,
	},
};

static struct rtdm_device device = {
	.driver = &rtipc_driver,
	.label = "rtipc",
};

int __init __rtipc_init(void)
{
	int ret, n;

	if (!realtime_core_enabled())
		return 0;

	for (n = 0; n < IPCPROTO_MAX; n++) {
		if (protocols[n] && protocols[n]->proto_init) {
			ret = protocols[n]->proto_init();
			if (ret)
				return ret;
		}
	}

	return rtdm_dev_register(&device);
}

void __exit __rtipc_exit(void)
{
	int n;

	if (!realtime_core_enabled())
		return;

	rtdm_dev_unregister(&device);

	for (n = 0; n < IPCPROTO_MAX; n++) {
		if (protocols[n] && protocols[n]->proto_exit)
			protocols[n]->proto_exit();
	}
}

module_init(__rtipc_init);
module_exit(__rtipc_exit);
