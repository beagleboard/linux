/***
 *
 *  packet/af_packet.c
 *
 *  RTnet - real-time networking subsystem
 *  Copyright (C) 2003-2006 Jan Kiszka <jan.kiszka@web.de>
 *  Copyright (C) 2006 Jorge Almeida <j-almeida@criticalsoftware.com>
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

#include <linux/module.h>
#include <linux/sched.h>
#include <linux/err.h>

#include <rtnet_iovec.h>
#include <rtnet_socket.h>
#include <stack_mgr.h>

MODULE_LICENSE("GPL");

/***
 *  rt_packet_rcv
 */
static int rt_packet_rcv(struct rtskb *skb, struct rtpacket_type *pt)
{
	struct rtsocket *sock =
		container_of(pt, struct rtsocket, prot.packet.packet_type);
	int ifindex = sock->prot.packet.ifindex;
	void (*callback_func)(struct rtdm_fd *, void *);
	void *callback_arg;
	rtdm_lockctx_t context;

	if (unlikely((ifindex != 0) && (ifindex != skb->rtdev->ifindex)))
		return -EUNATCH;

#ifdef CONFIG_XENO_DRIVERS_NET_ETH_P_ALL
	if (pt->type == htons(ETH_P_ALL)) {
		struct rtskb *clone_skb = rtskb_clone(skb, &sock->skb_pool);
		if (clone_skb == NULL)
			goto out;
		skb = clone_skb;
	} else
#endif /* CONFIG_XENO_DRIVERS_NET_ETH_P_ALL */
		if (unlikely(rtskb_acquire(skb, &sock->skb_pool) < 0)) {
		kfree_rtskb(skb);
		goto out;
	}

	rtskb_queue_tail(&sock->incoming, skb);
	rtdm_sem_up(&sock->pending_sem);

	rtdm_lock_get_irqsave(&sock->param_lock, context);
	callback_func = sock->callback_func;
	callback_arg = sock->callback_arg;
	rtdm_lock_put_irqrestore(&sock->param_lock, context);

	if (callback_func)
		callback_func(rt_socket_fd(sock), callback_arg);

out:
	return 0;
}

static bool rt_packet_trylock(struct rtpacket_type *pt)
{
	struct rtsocket *sock =
		container_of(pt, struct rtsocket, prot.packet.packet_type);
	struct rtdm_fd *fd = rtdm_private_to_fd(sock);

	if (rtdm_fd_lock(fd) < 0)
		return false;

	return true;
}

static void rt_packet_unlock(struct rtpacket_type *pt)
{
	struct rtsocket *sock =
		container_of(pt, struct rtsocket, prot.packet.packet_type);
	struct rtdm_fd *fd = rtdm_private_to_fd(sock);

	rtdm_fd_unlock(fd);
}

/***
 *  rt_packet_bind
 */
static int rt_packet_bind(struct rtdm_fd *fd, struct rtsocket *sock,
			  const struct sockaddr *addr, socklen_t addrlen)
{
	struct sockaddr_ll _sll, *sll;
	struct rtpacket_type *pt = &sock->prot.packet.packet_type;
	int new_type;
	int ret;
	rtdm_lockctx_t context;

	if (addrlen < sizeof(struct sockaddr_ll))
		return -EINVAL;

	sll = rtnet_get_arg(fd, &_sll, addr, sizeof(_sll));
	if (IS_ERR(sll))
		return PTR_ERR(sll);

	if (sll->sll_family != AF_PACKET)
		return -EINVAL;

	new_type =
		(sll->sll_protocol != 0) ? sll->sll_protocol : sock->protocol;

	rtdm_lock_get_irqsave(&sock->param_lock, context);

	/* release existing binding */
	if (pt->type != 0)
		rtdev_remove_pack(pt);

	pt->type = new_type;
	sock->prot.packet.ifindex = sll->sll_ifindex;

	/* if protocol is non-zero, register the packet type */
	if (new_type != 0) {
		pt->handler = rt_packet_rcv;
		pt->err_handler = NULL;
		pt->trylock = rt_packet_trylock;
		pt->unlock = rt_packet_unlock;

		ret = rtdev_add_pack(pt);
	} else
		ret = 0;

	rtdm_lock_put_irqrestore(&sock->param_lock, context);

	return ret;
}

/***
 *  rt_packet_getsockname
 */
static int rt_packet_getsockname(struct rtdm_fd *fd, struct rtsocket *sock,
				 struct sockaddr *addr, socklen_t *addrlen)
{
	struct sockaddr_ll _sll, *sll;
	struct rtnet_device *rtdev;
	rtdm_lockctx_t context;
	socklen_t _namelen, *namelen;
	int ret;

	namelen = rtnet_get_arg(fd, &_namelen, addrlen, sizeof(_namelen));
	if (IS_ERR(namelen))
		return PTR_ERR(namelen);

	if (*namelen < sizeof(struct sockaddr_ll))
		return -EINVAL;

	sll = rtnet_get_arg(fd, &_sll, addr, sizeof(_sll));
	if (IS_ERR(sll))
		return PTR_ERR(sll);

	rtdm_lock_get_irqsave(&sock->param_lock, context);

	sll->sll_family = AF_PACKET;
	sll->sll_ifindex = sock->prot.packet.ifindex;
	sll->sll_protocol = sock->protocol;

	rtdm_lock_put_irqrestore(&sock->param_lock, context);

	rtdev = rtdev_get_by_index(sll->sll_ifindex);
	if (rtdev != NULL) {
		sll->sll_hatype = rtdev->type;
		sll->sll_halen = rtdev->addr_len;
		memcpy(sll->sll_addr, rtdev->dev_addr, rtdev->addr_len);
		rtdev_dereference(rtdev);
	} else {
		sll->sll_hatype = 0;
		sll->sll_halen = 0;
	}

	*namelen = sizeof(struct sockaddr_ll);

	ret = rtnet_put_arg(fd, addr, sll, sizeof(*sll));
	if (ret)
		return ret;

	return rtnet_put_arg(fd, addrlen, namelen, sizeof(*namelen));
}

/***
 * rt_packet_socket - initialize a packet socket
 */
static int rt_packet_socket(struct rtdm_fd *fd, int protocol)
{
	struct rtsocket *sock = rtdm_fd_to_private(fd);
	int ret;

	if ((ret = rt_socket_init(fd, protocol)) != 0)
		return ret;

	sock->prot.packet.packet_type.type = protocol;
	sock->prot.packet.ifindex = 0;
	sock->prot.packet.packet_type.trylock = rt_packet_trylock;
	sock->prot.packet.packet_type.unlock = rt_packet_unlock;

	/* if protocol is non-zero, register the packet type */
	if (protocol != 0) {
		sock->prot.packet.packet_type.handler = rt_packet_rcv;
		sock->prot.packet.packet_type.err_handler = NULL;

		if ((ret = rtdev_add_pack(&sock->prot.packet.packet_type)) <
		    0) {
			rt_socket_cleanup(fd);
			return ret;
		}
	}

	return 0;
}

/***
 *  rt_packet_close
 */
static void rt_packet_close(struct rtdm_fd *fd)
{
	struct rtsocket *sock = rtdm_fd_to_private(fd);
	struct rtpacket_type *pt = &sock->prot.packet.packet_type;
	struct rtskb *del;
	rtdm_lockctx_t context;

	rtdm_lock_get_irqsave(&sock->param_lock, context);

	if (pt->type != 0) {
		rtdev_remove_pack(pt);
		pt->type = 0;
	}

	rtdm_lock_put_irqrestore(&sock->param_lock, context);

	/* free packets in incoming queue */
	while ((del = rtskb_dequeue(&sock->incoming)) != NULL) {
		kfree_rtskb(del);
	}

	rt_socket_cleanup(fd);
}

/***
 *  rt_packet_ioctl
 */
static int rt_packet_ioctl(struct rtdm_fd *fd, unsigned int request,
			   void __user *arg)
{
	struct rtsocket *sock = rtdm_fd_to_private(fd);
	const struct _rtdm_setsockaddr_args *setaddr;
	struct _rtdm_setsockaddr_args _setaddr;
	const struct _rtdm_getsockaddr_args *getaddr;
	struct _rtdm_getsockaddr_args _getaddr;

	/* fast path for common socket IOCTLs */
	if (_IOC_TYPE(request) == RTIOC_TYPE_NETWORK)
		return rt_socket_common_ioctl(fd, request, arg);

	switch (request) {
	case _RTIOC_BIND:
		setaddr = rtnet_get_arg(fd, &_setaddr, arg, sizeof(_setaddr));
		if (IS_ERR(setaddr))
			return PTR_ERR(setaddr);
		return rt_packet_bind(fd, sock, setaddr->addr,
				      setaddr->addrlen);

	case _RTIOC_GETSOCKNAME:
		getaddr = rtnet_get_arg(fd, &_getaddr, arg, sizeof(_getaddr));
		if (IS_ERR(getaddr))
			return PTR_ERR(getaddr);
		return rt_packet_getsockname(fd, sock, getaddr->addr,
					     getaddr->addrlen);

	default:
		return rt_socket_if_ioctl(fd, request, arg);
	}
}

/***
 *  rt_packet_recvmsg
 */
static ssize_t rt_packet_recvmsg(struct rtdm_fd *fd, struct user_msghdr *msg,
				 int msg_flags)
{
	struct rtsocket *sock = rtdm_fd_to_private(fd);
	ssize_t len;
	size_t copy_len;
	struct rtskb *rtskb;
	struct sockaddr_ll sll;
	int ret, flags;
	nanosecs_rel_t timeout = sock->timeout;
	socklen_t namelen;
	struct iovec iov_fast[RTDM_IOV_FASTMAX], *iov;

	if (msg->msg_iovlen < 0)
		return -EINVAL;

	if (msg->msg_iovlen == 0)
		return 0;

	ret = rtdm_get_iovec(fd, &iov, msg, iov_fast);
	if (ret)
		return ret;

	/* non-blocking receive? */
	if (msg_flags & MSG_DONTWAIT)
		timeout = -1;

	ret = rtdm_sem_timeddown(&sock->pending_sem, timeout, NULL);
	if (unlikely(ret < 0))
		switch (ret) {
		default:
			ret = -EBADF; /* socket has been closed */
			/* fallthrough */
		case -EWOULDBLOCK:
		case -ETIMEDOUT:
		case -EINTR:
			rtdm_drop_iovec(iov, iov_fast);
			return ret;
		}

	rtskb = rtskb_dequeue_chain(&sock->incoming);
	RTNET_ASSERT(rtskb != NULL, return -EFAULT;);

	/* copy the address if required. */
	if (msg->msg_name) {
		struct rtnet_device *rtdev = rtskb->rtdev;
		memset(&sll, 0, sizeof(sll));
		sll.sll_family = AF_PACKET;
		sll.sll_hatype = rtdev->type;
		sll.sll_protocol = rtskb->protocol;
		sll.sll_pkttype = rtskb->pkt_type;
		sll.sll_ifindex = rtdev->ifindex;

		/* Ethernet specific - we rather need some parse handler here */
		memcpy(sll.sll_addr, rtskb->mac.ethernet->h_source, ETH_ALEN);
		sll.sll_halen = ETH_ALEN;
		ret = rtnet_put_arg(fd, msg->msg_name, &sll, sizeof(sll));
		if (ret)
			goto fail;

		namelen = sizeof(sll);
		ret = rtnet_put_arg(fd, &msg->msg_namelen, &namelen,
				    sizeof(namelen));
		if (ret)
			goto fail;
	}

	/* Include the header in raw delivery */
	if (rtdm_fd_to_context(fd)->device->driver->socket_type != SOCK_DGRAM)
		rtskb_push(rtskb, rtskb->data - rtskb->mac.raw);

	/* The data must not be longer than the available buffer size */
	copy_len = rtskb->len;
	len = rtdm_get_iov_flatlen(iov, msg->msg_iovlen);
	if (len < 0) {
		copy_len = len;
		goto out;
	}

	if (copy_len > len) {
		copy_len = len;
		flags = msg->msg_flags | MSG_TRUNC;
		ret = rtnet_put_arg(fd, &msg->msg_flags, &flags,
				    sizeof(flags));
		if (ret)
			goto fail;
	}

	copy_len = rtnet_write_to_iov(fd, iov, msg->msg_iovlen, rtskb->data,
				      copy_len);
out:
	if ((msg_flags & MSG_PEEK) == 0) {
		kfree_rtskb(rtskb);
	} else {
		rtskb_queue_head(&sock->incoming, rtskb);
		rtdm_sem_up(&sock->pending_sem);
	}

	rtdm_drop_iovec(iov, iov_fast);

	return copy_len;
fail:
	copy_len = ret;
	goto out;
}

/***
 *  rt_packet_sendmsg
 */
static ssize_t rt_packet_sendmsg(struct rtdm_fd *fd,
				 const struct user_msghdr *msg, int msg_flags)
{
	struct rtsocket *sock = rtdm_fd_to_private(fd);
	size_t len;
	struct sockaddr_ll _sll, *sll;
	struct rtnet_device *rtdev;
	struct rtskb *rtskb;
	unsigned short proto;
	unsigned char *addr;
	int ifindex;
	ssize_t ret;
	struct iovec iov_fast[RTDM_IOV_FASTMAX], *iov;

	if (msg_flags & MSG_OOB) /* Mirror BSD error message compatibility */
		return -EOPNOTSUPP;
	if (msg_flags & ~MSG_DONTWAIT)
		return -EINVAL;

	if (msg->msg_iovlen < 0)
		return -EINVAL;

	if (msg->msg_iovlen == 0)
		return 0;

	ret = rtdm_get_iovec(fd, &iov, msg, iov_fast);
	if (ret)
		return ret;

	if (msg->msg_name == NULL) {
		/* Note: We do not care about races with rt_packet_bind here -
	   the user has to do so. */
		ifindex = sock->prot.packet.ifindex;
		proto = sock->prot.packet.packet_type.type;
		addr = NULL;
		sll = NULL;
	} else {
		sll = rtnet_get_arg(fd, &_sll, msg->msg_name, sizeof(_sll));
		if (IS_ERR(sll)) {
			ret = PTR_ERR(sll);
			goto abort;
		}

		if ((msg->msg_namelen < sizeof(struct sockaddr_ll)) ||
		    (msg->msg_namelen <
		     (sll->sll_halen +
		      offsetof(struct sockaddr_ll, sll_addr))) ||
		    ((sll->sll_family != AF_PACKET) &&
		     (sll->sll_family != AF_UNSPEC))) {
			ret = -EINVAL;
			goto abort;
		}

		ifindex = sll->sll_ifindex;
		proto = sll->sll_protocol;
		addr = sll->sll_addr;
	}

	if ((rtdev = rtdev_get_by_index(ifindex)) == NULL) {
		ret = -ENODEV;
		goto abort;
	}

	len = rtdm_get_iov_flatlen(iov, msg->msg_iovlen);
	rtskb = alloc_rtskb(rtdev->hard_header_len + len, &sock->skb_pool);
	if (rtskb == NULL) {
		ret = -ENOBUFS;
		goto out;
	}

	/* If an RTmac discipline is active, this becomes a pure sanity check to
       avoid writing beyond rtskb boundaries. The hard check is then performed
       upon rtdev_xmit() by the discipline's xmit handler. */
	if (len >
	    rtdev->mtu +
		    ((rtdm_fd_to_context(fd)->device->driver->socket_type ==
		      SOCK_RAW) ?
			     rtdev->hard_header_len :
			     0)) {
		ret = -EMSGSIZE;
		goto err;
	}

	if ((sll != NULL) && (sll->sll_halen != rtdev->addr_len)) {
		ret = -EINVAL;
		goto err;
	}

	rtskb_reserve(rtskb, rtdev->hard_header_len);

	rtskb->rtdev = rtdev;
	rtskb->priority = sock->priority;

	if (rtdev->hard_header) {
		int hdr_len;

		ret = -EINVAL;
		hdr_len = rtdev->hard_header(rtskb, rtdev, ntohs(proto), addr,
					     NULL, len);
		if (rtdm_fd_to_context(fd)->device->driver->socket_type !=
		    SOCK_DGRAM) {
			rtskb->tail = rtskb->data;
			rtskb->len = 0;
		} else if (hdr_len < 0)
			goto err;
	}

	ret = rtnet_read_from_iov(fd, iov, msg->msg_iovlen,
				  rtskb_put(rtskb, len), len);

	if ((rtdev->flags & IFF_UP) != 0) {
		if ((ret = rtdev_xmit(rtskb)) == 0)
			ret = len;
	} else {
		ret = -ENETDOWN;
		goto err;
	}

out:
	rtdev_dereference(rtdev);
abort:
	rtdm_drop_iovec(iov, iov_fast);

	return ret;
err:
	kfree_rtskb(rtskb);
	goto out;
}

static struct rtdm_driver packet_proto_drv = {
    .profile_info =     RTDM_PROFILE_INFO(packet,
					RTDM_CLASS_NETWORK,
					RTDM_SUBCLASS_RTNET,
					RTNET_RTDM_VER),
    .device_flags =     RTDM_PROTOCOL_DEVICE,
    .device_count =     1,
    .context_size =     sizeof(struct rtsocket),

    .protocol_family =  PF_PACKET,
    .socket_type =      SOCK_DGRAM,


    .ops = {
	.socket =       rt_packet_socket,
	.close =        rt_packet_close,
	.ioctl_rt =     rt_packet_ioctl,
	.ioctl_nrt =    rt_packet_ioctl,
	.recvmsg_rt =   rt_packet_recvmsg,
	.sendmsg_rt =   rt_packet_sendmsg,
	.select =       rt_socket_select_bind,
    },
};

static struct rtdm_device packet_proto_dev = {
	.driver = &packet_proto_drv,
	.label = "packet",
};

static struct rtdm_driver raw_packet_proto_drv = {
    .profile_info =     RTDM_PROFILE_INFO(raw_packet,
					RTDM_CLASS_NETWORK,
					RTDM_SUBCLASS_RTNET,
					RTNET_RTDM_VER),
    .device_flags =     RTDM_PROTOCOL_DEVICE,
    .device_count =     1,
    .context_size =     sizeof(struct rtsocket),

    .protocol_family =  PF_PACKET,
    .socket_type =      SOCK_RAW,

    .ops = {
	.socket =       rt_packet_socket,
	.close =        rt_packet_close,
	.ioctl_rt =     rt_packet_ioctl,
	.ioctl_nrt =    rt_packet_ioctl,
	.recvmsg_rt =   rt_packet_recvmsg,
	.sendmsg_rt =   rt_packet_sendmsg,
	.select =       rt_socket_select_bind,
    },
};

static struct rtdm_device raw_packet_proto_dev = {
	.driver = &raw_packet_proto_drv,
	.label = "raw_packet",
};

static int __init rt_packet_proto_init(void)
{
	int err;

	err = rtdm_dev_register(&packet_proto_dev);
	if (err)
		return err;

	err = rtdm_dev_register(&raw_packet_proto_dev);
	if (err)
		rtdm_dev_unregister(&packet_proto_dev);

	return err;
}

static void rt_packet_proto_release(void)
{
	rtdm_dev_unregister(&packet_proto_dev);
	rtdm_dev_unregister(&raw_packet_proto_dev);
}

module_init(rt_packet_proto_init);
module_exit(rt_packet_proto_release);

/**********************************************************
 * Utilities                                              *
 **********************************************************/

static int hex2int(unsigned char hex_char)
{
	if ((hex_char >= '0') && (hex_char <= '9'))
		return hex_char - '0';
	else if ((hex_char >= 'a') && (hex_char <= 'f'))
		return hex_char - 'a' + 10;
	else if ((hex_char >= 'A') && (hex_char <= 'F'))
		return hex_char - 'A' + 10;
	else
		return -EINVAL;
}

int rt_eth_aton(unsigned char *addr_buf, const char *mac)
{
	int i = 0;
	int nibble;

	while (1) {
		if (*mac == 0)
			return -EINVAL;

		if ((nibble = hex2int(*mac++)) < 0)
			return nibble;
		*addr_buf = nibble << 4;

		if (*mac == 0)
			return -EINVAL;

		if ((nibble = hex2int(*mac++)) < 0)
			return nibble;
		*addr_buf++ |= nibble;

		if (++i == 6)
			break;

		if ((*mac == 0) || (*mac++ != ':'))
			return -EINVAL;
	}
	return 0;
}

EXPORT_SYMBOL_GPL(rt_eth_aton);
