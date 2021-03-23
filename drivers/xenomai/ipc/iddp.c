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
#include <linux/list.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <cobalt/kernel/heap.h>
#include <cobalt/kernel/bufd.h>
#include <cobalt/kernel/map.h>
#include <rtdm/ipc.h>
#include "internal.h"

#define IDDP_SOCKET_MAGIC 0xa37a37a8

struct iddp_message {
	struct list_head next;
	int from;
	size_t rdoff;
	size_t len;
	char data[];
};

struct iddp_socket {
	int magic;
	struct sockaddr_ipc name;
	struct sockaddr_ipc peer;
	struct xnheap *bufpool;
	struct xnheap privpool;
	rtdm_waitqueue_t *poolwaitq;
	rtdm_waitqueue_t privwaitq;
	size_t poolsz;
	rtdm_sem_t insem;
	struct list_head inq;
	u_long status;
	xnhandle_t handle;
	char label[XNOBJECT_NAME_LEN];
	nanosecs_rel_t rx_timeout;
	nanosecs_rel_t tx_timeout;
	unsigned long stalls;	/* Buffer stall counter. */
	struct rtipc_private *priv;
};

static struct sockaddr_ipc nullsa = {
	.sipc_family = AF_RTIPC,
	.sipc_port = -1
};

static struct xnmap *portmap;

static rtdm_waitqueue_t poolwaitq;

#define _IDDP_BINDING   0
#define _IDDP_BOUND     1
#define _IDDP_CONNECTED 2

#ifdef CONFIG_XENO_OPT_VFILE

static char *__iddp_link_target(void *obj)
{
	struct iddp_socket *sk = obj;

	return kasformat("%d", sk->name.sipc_port);
}

extern struct xnptree rtipc_ptree;

static struct xnpnode_link __iddp_pnode = {
	.node = {
		.dirname = "iddp",
		.root = &rtipc_ptree,
		.ops = &xnregistry_vlink_ops,
	},
	.target = __iddp_link_target,
};

#else /* !CONFIG_XENO_OPT_VFILE */

static struct xnpnode_link __iddp_pnode = {
	.node = {
		.dirname = "iddp",
	},
};

#endif /* !CONFIG_XENO_OPT_VFILE */

static inline void __iddp_init_mbuf(struct iddp_message *mbuf, size_t len)
{
	mbuf->rdoff = 0;
	mbuf->len = len;
	INIT_LIST_HEAD(&mbuf->next);
}

static struct iddp_message *
__iddp_alloc_mbuf(struct iddp_socket *sk, size_t len,
		  nanosecs_rel_t timeout, int flags, int *pret)
{
	struct iddp_message *mbuf = NULL;
	rtdm_toseq_t timeout_seq;
	rtdm_lockctx_t s;
	int ret = 0;

	rtdm_toseq_init(&timeout_seq, timeout);

	for (;;) {
		mbuf = xnheap_alloc(sk->bufpool, len + sizeof(*mbuf));
		if (mbuf) {
			__iddp_init_mbuf(mbuf, len);
			break;
		}
		if (flags & MSG_DONTWAIT) {
			ret = -EAGAIN;
			break;
		}
		/*
		 * No luck, no buffer free. Wait for a buffer to be
		 * released and retry. Admittedly, we might create a
		 * thundering herd effect if many waiters put a lot of
		 * memory pressure on the pool, but in this case, the
		 * pool size should be adjusted.
		 */
		rtdm_waitqueue_lock(sk->poolwaitq, s);
		++sk->stalls;
		ret = rtdm_timedwait_locked(sk->poolwaitq, timeout, &timeout_seq);
		rtdm_waitqueue_unlock(sk->poolwaitq, s);
		if (unlikely(ret == -EIDRM))
			ret = -ECONNRESET;
		if (ret)
			break;
	}

	*pret = ret;

	return mbuf;
}

static void __iddp_free_mbuf(struct iddp_socket *sk,
			     struct iddp_message *mbuf)
{
	xnheap_free(sk->bufpool, mbuf);
	rtdm_waitqueue_broadcast(sk->poolwaitq);
}

static int iddp_socket(struct rtdm_fd *fd)
{
	struct rtipc_private *priv = rtdm_fd_to_private(fd);
	struct iddp_socket *sk = priv->state;

	sk->magic = IDDP_SOCKET_MAGIC;
	sk->name = nullsa;	/* Unbound */
	sk->peer = nullsa;
	sk->bufpool = &cobalt_heap;
	sk->poolwaitq = &poolwaitq;
	sk->poolsz = 0;
	sk->status = 0;
	sk->handle = 0;
	sk->rx_timeout = RTDM_TIMEOUT_INFINITE;
	sk->tx_timeout = RTDM_TIMEOUT_INFINITE;
	sk->stalls = 0;
	*sk->label = 0;
	INIT_LIST_HEAD(&sk->inq);
	rtdm_sem_init(&sk->insem, 0);
	rtdm_waitqueue_init(&sk->privwaitq);
	sk->priv = priv;

	return 0;
}

static void iddp_close(struct rtdm_fd *fd)
{
	struct rtipc_private *priv = rtdm_fd_to_private(fd);
	struct iddp_socket *sk = priv->state;
	struct iddp_message *mbuf;
	rtdm_lockctx_t s;
	void *poolmem;
	u32 poolsz;

	rtdm_sem_destroy(&sk->insem);
	rtdm_waitqueue_destroy(&sk->privwaitq);

	if (test_bit(_IDDP_BOUND, &sk->status)) {
		if (sk->handle)
			xnregistry_remove(sk->handle);
		if (sk->name.sipc_port > -1) {
			cobalt_atomic_enter(s);
			xnmap_remove(portmap, sk->name.sipc_port);
			cobalt_atomic_leave(s);
		}
		if (sk->bufpool != &cobalt_heap) {
			poolmem = xnheap_get_membase(&sk->privpool);
			poolsz = xnheap_get_size(&sk->privpool);
			xnheap_destroy(&sk->privpool);
			xnheap_vfree(poolmem);
			return;
		}
	}

	/* Send unread datagrams back to the system heap. */
	while (!list_empty(&sk->inq)) {
		mbuf = list_entry(sk->inq.next, struct iddp_message, next);
		list_del(&mbuf->next);
		xnheap_free(&cobalt_heap, mbuf);
	}

	kfree(sk);

	return;
}

static ssize_t __iddp_recvmsg(struct rtdm_fd *fd,
			      struct iovec *iov, int iovlen, int flags,
			      struct sockaddr_ipc *saddr)
{
	struct rtipc_private *priv = rtdm_fd_to_private(fd);
	struct iddp_socket *sk = priv->state;
	ssize_t maxlen, len, wrlen, vlen;
	rtdm_toseq_t timeout_seq, *toseq;
	int nvec, rdoff, ret, dofree;
	struct iddp_message *mbuf;
	nanosecs_rel_t timeout;
	struct xnbufd bufd;
	rtdm_lockctx_t s;

	if (!test_bit(_IDDP_BOUND, &sk->status))
		return -EAGAIN;

	maxlen = rtdm_get_iov_flatlen(iov, iovlen);
	if (maxlen == 0)
		return 0;

	if (flags & MSG_DONTWAIT) {
		timeout = RTDM_TIMEOUT_NONE;
		toseq = NULL;
	} else {
		timeout = sk->rx_timeout;
		toseq = &timeout_seq;
	}

	/* We want to pick one buffer from the queue. */
	
	for (;;) {
		ret = rtdm_sem_timeddown(&sk->insem, timeout, toseq);
		if (unlikely(ret)) {
			if (ret == -EIDRM)
				return -ECONNRESET;
			return ret;
		}
		/* We may have spurious wakeups. */
		cobalt_atomic_enter(s);
		if (!list_empty(&sk->inq))
			break;
		cobalt_atomic_leave(s);
	}

	/* Pull heading message from input queue. */
	mbuf = list_entry(sk->inq.next, struct iddp_message, next);
	rdoff = mbuf->rdoff;
	len = mbuf->len - rdoff;
	if (saddr) {
		saddr->sipc_family = AF_RTIPC;
		saddr->sipc_port = mbuf->from;
	}
	if (maxlen >= len) {
		list_del(&mbuf->next);
		dofree = 1;
		if (list_empty(&sk->inq)) /* -> non-readable */
			xnselect_signal(&priv->recv_block, 0);

	} else {
		/* Buffer is only partially read: repost. */
		mbuf->rdoff += maxlen;
		len = maxlen;
		dofree = 0;
	}

	if (!dofree)
		rtdm_sem_up(&sk->insem);

	cobalt_atomic_leave(s);

	/* Now, write "len" bytes from mbuf->data to the vector cells */
	for (nvec = 0, wrlen = len; nvec < iovlen && wrlen > 0; nvec++) {
		if (iov[nvec].iov_len == 0)
			continue;
		vlen = wrlen >= iov[nvec].iov_len ? iov[nvec].iov_len : wrlen;
		if (rtdm_fd_is_user(fd)) {
			xnbufd_map_uread(&bufd, iov[nvec].iov_base, vlen);
			ret = xnbufd_copy_from_kmem(&bufd, mbuf->data + rdoff, vlen);
			xnbufd_unmap_uread(&bufd);
		} else {
			xnbufd_map_kread(&bufd, iov[nvec].iov_base, vlen);
			ret = xnbufd_copy_from_kmem(&bufd, mbuf->data + rdoff, vlen);
			xnbufd_unmap_kread(&bufd);
		}
		if (ret < 0)
			break;
		iov[nvec].iov_base += vlen;
		iov[nvec].iov_len -= vlen;
		wrlen -= vlen;
		rdoff += vlen;
	}

	if (dofree)
		__iddp_free_mbuf(sk, mbuf);

	return ret ?: len;
}

static ssize_t iddp_recvmsg(struct rtdm_fd *fd,
			    struct user_msghdr *msg, int flags)
{
	struct iovec iov_fast[RTDM_IOV_FASTMAX], *iov;
	struct sockaddr_ipc saddr;
	ssize_t ret;

	if (flags & ~MSG_DONTWAIT)
		return -EINVAL;

	if (msg->msg_name) {
		if (msg->msg_namelen < sizeof(struct sockaddr_ipc))
			return -EINVAL;
	} else if (msg->msg_namelen != 0)
		return -EINVAL;

	if (msg->msg_iovlen >= UIO_MAXIOV)
		return -EINVAL;

	/* Copy I/O vector in */
	ret = rtdm_get_iovec(fd, &iov, msg, iov_fast);
	if (ret)
		return ret;

	ret = __iddp_recvmsg(fd, iov, msg->msg_iovlen, flags, &saddr);
	if (ret <= 0) {
		rtdm_drop_iovec(iov, iov_fast);
		return ret;
	}

	/* Copy the updated I/O vector back */
	if (rtdm_put_iovec(fd, iov, msg, iov_fast))
		return -EFAULT;

	/* Copy the source address if required. */
	if (msg->msg_name) {
		if (rtipc_put_arg(fd, msg->msg_name, &saddr, sizeof(saddr)))
			return -EFAULT;
		msg->msg_namelen = sizeof(struct sockaddr_ipc);
	}

	return ret;
}

static ssize_t iddp_read(struct rtdm_fd *fd, void *buf, size_t len)
{
	struct iovec iov = { .iov_base = buf, .iov_len = len };

	return __iddp_recvmsg(fd, &iov, 1, 0, NULL);
}

static ssize_t __iddp_sendmsg(struct rtdm_fd *fd,
			      struct iovec *iov, int iovlen, int flags,
			      const struct sockaddr_ipc *daddr)
{
	struct rtipc_private *priv = rtdm_fd_to_private(fd);
	struct iddp_socket *sk = priv->state, *rsk;
	struct iddp_message *mbuf;
	ssize_t len, rdlen, vlen;
	int nvec, wroff, ret;
	struct rtdm_fd *rfd;
	struct xnbufd bufd;
	rtdm_lockctx_t s;

	len = rtdm_get_iov_flatlen(iov, iovlen);
	if (len == 0)
		return 0;

	cobalt_atomic_enter(s);
	rfd = xnmap_fetch_nocheck(portmap, daddr->sipc_port);
	if (rfd && rtdm_fd_lock(rfd) < 0)
		rfd = NULL;
	cobalt_atomic_leave(s);
	if (rfd == NULL)
		return -ECONNRESET;

	rsk = rtipc_fd_to_state(rfd);
	if (!test_bit(_IDDP_BOUND, &rsk->status)) {
		rtdm_fd_unlock(rfd);
		return -ECONNREFUSED;
	}

	mbuf = __iddp_alloc_mbuf(rsk, len, sk->tx_timeout, flags, &ret);
	if (unlikely(ret)) {
		rtdm_fd_unlock(rfd);
		return ret;
	}

	/* Now, move "len" bytes to mbuf->data from the vector cells */
	for (nvec = 0, rdlen = len, wroff = 0;
	     nvec < iovlen && rdlen > 0; nvec++) {
		if (iov[nvec].iov_len == 0)
			continue;
		vlen = rdlen >= iov[nvec].iov_len ? iov[nvec].iov_len : rdlen;
		if (rtdm_fd_is_user(fd)) {
			xnbufd_map_uread(&bufd, iov[nvec].iov_base, vlen);
			ret = xnbufd_copy_to_kmem(mbuf->data + wroff, &bufd, vlen);
			xnbufd_unmap_uread(&bufd);
		} else {
			xnbufd_map_kread(&bufd, iov[nvec].iov_base, vlen);
			ret = xnbufd_copy_to_kmem(mbuf->data + wroff, &bufd, vlen);
			xnbufd_unmap_kread(&bufd);
		}
		if (ret < 0)
			goto fail;
		iov[nvec].iov_base += vlen;
		iov[nvec].iov_len -= vlen;
		rdlen -= vlen;
		wroff += vlen;
	}

	cobalt_atomic_enter(s);

	/*
	 * CAUTION: we must remain atomic from the moment we signal
	 * POLLIN, until sem_up has happened.
	 */
	if (list_empty(&rsk->inq)) /* -> readable */
		xnselect_signal(&rsk->priv->recv_block, POLLIN);

	mbuf->from = sk->name.sipc_port;

	if (flags & MSG_OOB)
		list_add(&mbuf->next, &rsk->inq);
	else
		list_add_tail(&mbuf->next, &rsk->inq);

	rtdm_sem_up(&rsk->insem); /* Will resched. */

	cobalt_atomic_leave(s);

	rtdm_fd_unlock(rfd);

	return len;

fail:
	__iddp_free_mbuf(rsk, mbuf);

	rtdm_fd_unlock(rfd);

	return ret;
}

static ssize_t iddp_sendmsg(struct rtdm_fd *fd,
			    const struct user_msghdr *msg, int flags)
{
	struct rtipc_private *priv = rtdm_fd_to_private(fd);
	struct iovec iov_fast[RTDM_IOV_FASTMAX], *iov;
	struct iddp_socket *sk = priv->state;
	struct sockaddr_ipc daddr;
	ssize_t ret;

	if (flags & ~(MSG_OOB | MSG_DONTWAIT))
		return -EINVAL;

	if (msg->msg_name) {
		if (msg->msg_namelen != sizeof(struct sockaddr_ipc))
			return -EINVAL;

		/* Fetch the destination address to send to. */
		if (rtipc_get_arg(fd, &daddr, msg->msg_name, sizeof(daddr)))
			return -EFAULT;

		if (daddr.sipc_port < 0 ||
		    daddr.sipc_port >= CONFIG_XENO_OPT_IDDP_NRPORT)
			return -EINVAL;
	} else {
		if (msg->msg_namelen != 0)
			return -EINVAL;
		daddr = sk->peer;
		if (daddr.sipc_port < 0)
			return -EDESTADDRREQ;
	}

	if (msg->msg_iovlen >= UIO_MAXIOV)
		return -EINVAL;

	/* Copy I/O vector in */
	ret = rtdm_get_iovec(fd, &iov, msg, iov_fast);
	if (ret)
		return ret;

	ret = __iddp_sendmsg(fd, iov, msg->msg_iovlen, flags, &daddr);
	if (ret <= 0) {
		rtdm_drop_iovec(iov, iov_fast);
		return ret;
	}

	/* Copy updated I/O vector back */
	return rtdm_put_iovec(fd, iov, msg, iov_fast) ?: ret;
}

static ssize_t iddp_write(struct rtdm_fd *fd,
			  const void *buf, size_t len)
{
	struct rtipc_private *priv = rtdm_fd_to_private(fd);
	struct iovec iov = { .iov_base = (void *)buf, .iov_len = len };
	struct iddp_socket *sk = priv->state;

	if (sk->peer.sipc_port < 0)
		return -EDESTADDRREQ;

	return __iddp_sendmsg(fd, &iov, 1, 0, &sk->peer);
}

static int __iddp_bind_socket(struct rtdm_fd *fd,
			      struct sockaddr_ipc *sa)
{
	struct rtipc_private *priv = rtdm_fd_to_private(fd);
	struct iddp_socket *sk = priv->state;
	int ret = 0, port;
	rtdm_lockctx_t s;
	void *poolmem;
	size_t poolsz;

	if (sa->sipc_family != AF_RTIPC)
		return -EINVAL;

	if (sa->sipc_port < -1 ||
	    sa->sipc_port >= CONFIG_XENO_OPT_IDDP_NRPORT)
		return -EINVAL;

	cobalt_atomic_enter(s);
	if (test_bit(_IDDP_BOUND, &sk->status) ||
	    __test_and_set_bit(_IDDP_BINDING, &sk->status))
		ret = -EADDRINUSE;
	cobalt_atomic_leave(s);
	if (ret)
		return ret;

	/* Will auto-select a free port number if unspec (-1). */
	port = sa->sipc_port;
	cobalt_atomic_enter(s);
	port = xnmap_enter(portmap, port, fd);
	cobalt_atomic_leave(s);
	if (port < 0)
		return port == -EEXIST ? -EADDRINUSE : -ENOMEM;

	sa->sipc_port = port;

	/*
	 * Allocate a local buffer pool if we were told to do so via
	 * setsockopt() before we got there.
	 */
	poolsz = sk->poolsz;
	if (poolsz > 0) {
		poolsz = PAGE_ALIGN(poolsz);
		poolmem = xnheap_vmalloc(poolsz);
		if (poolmem == NULL) {
			ret = -ENOMEM;
			goto fail;
		}

		ret = xnheap_init(&sk->privpool, poolmem, poolsz);
		if (ret) {
			xnheap_vfree(poolmem);
			goto fail;
		}
		xnheap_set_name(&sk->privpool, "iddp-pool@%d", port);
		sk->poolwaitq = &sk->privwaitq;
		sk->bufpool = &sk->privpool;
	}

	sk->name = *sa;
	/* Set default destination if unset at binding time. */
	if (sk->peer.sipc_port < 0)
		sk->peer = *sa;

	if (*sk->label) {
		ret = xnregistry_enter(sk->label, sk,
				       &sk->handle, &__iddp_pnode.node);
		if (ret) {
			if (poolsz > 0) {
				xnheap_destroy(&sk->privpool);
				xnheap_vfree(poolmem);
			}
			goto fail;
		}
	}

	cobalt_atomic_enter(s);
	__clear_bit(_IDDP_BINDING, &sk->status);
	__set_bit(_IDDP_BOUND, &sk->status);
	if (xnselect_signal(&priv->send_block, POLLOUT))
		xnsched_run();
	cobalt_atomic_leave(s);

	return 0;
fail:
	xnmap_remove(portmap, port);
	clear_bit(_IDDP_BINDING, &sk->status);

	return ret;
}

static int __iddp_connect_socket(struct iddp_socket *sk,
				 struct sockaddr_ipc *sa)
{
	struct sockaddr_ipc _sa;
	struct iddp_socket *rsk;
	int ret, resched = 0;
	rtdm_lockctx_t s;
	xnhandle_t h;

	if (sa == NULL) {
		_sa = nullsa;
		sa = &_sa;
		goto set_assoc;
	}

	if (sa->sipc_family != AF_RTIPC)
		return -EINVAL;

	if (sa->sipc_port < -1 ||
	    sa->sipc_port >= CONFIG_XENO_OPT_IDDP_NRPORT)
		return -EINVAL;
	/*
	 * - If a valid sipc_port is passed in the [0..NRPORT-1] range,
	 * it is used verbatim and the connection succeeds
	 * immediately, regardless of whether the destination is
	 * bound at the time of the call.
	 *
	 * - If sipc_port is -1 and a label was set via IDDP_LABEL,
	 * connect() blocks for the requested amount of time (see
	 * SO_RCVTIMEO) until a socket is bound to the same label.
	 *
	 * - If sipc_port is -1 and no label is given, the default
	 * destination address is cleared, meaning that any subsequent
	 * write() to the socket will return -EDESTADDRREQ, until a
	 * valid destination address is set via connect() or bind().
	 *
	 * - In all other cases, -EINVAL is returned.
	 */
	if (sa->sipc_port < 0 && *sk->label) {
		ret = xnregistry_bind(sk->label,
				      sk->rx_timeout, XN_RELATIVE, &h);
		if (ret)
			return ret;

		cobalt_atomic_enter(s);
		rsk = xnregistry_lookup(h, NULL);
		if (rsk == NULL || rsk->magic != IDDP_SOCKET_MAGIC)
			ret = -EINVAL;
		else {
			/* Fetch labeled port number. */
			sa->sipc_port = rsk->name.sipc_port;
			resched = xnselect_signal(&sk->priv->send_block, POLLOUT);
		}
		cobalt_atomic_leave(s);
		if (ret)
			return ret;
	} else if (sa->sipc_port < 0)
		sa = &nullsa;
set_assoc:
	cobalt_atomic_enter(s);
	if (!test_bit(_IDDP_BOUND, &sk->status))
		/* Set default name. */
		sk->name = *sa;
	/* Set default destination. */
	sk->peer = *sa;
	if (sa->sipc_port < 0)
		__clear_bit(_IDDP_CONNECTED, &sk->status);
	else
		__set_bit(_IDDP_CONNECTED, &sk->status);
	if (resched)
		xnsched_run();
	cobalt_atomic_leave(s);

	return 0;
}

static int __iddp_setsockopt(struct iddp_socket *sk,
			     struct rtdm_fd *fd,
			     void *arg)
{
	struct _rtdm_setsockopt_args sopt;
	struct rtipc_port_label plabel;
	struct timeval tv;
	rtdm_lockctx_t s;
	size_t len;
	int ret;

	ret = rtipc_get_sockoptin(fd, &sopt, arg);
	if (ret)
		return ret;

	if (sopt.level == SOL_SOCKET) {
		switch (sopt.optname) {

		case SO_RCVTIMEO_OLD:
			ret = rtipc_get_timeval(fd, &tv, sopt.optval, sopt.optlen);
			if (ret)
				return ret;
			sk->rx_timeout = rtipc_timeval_to_ns(&tv);
			break;

		case SO_SNDTIMEO_OLD:
			ret = rtipc_get_timeval(fd, &tv, sopt.optval, sopt.optlen);
			if (ret)
				return ret;
			sk->tx_timeout = rtipc_timeval_to_ns(&tv);
			break;

		default:
			ret = -EINVAL;
		}

		return ret;
	}

	if (sopt.level != SOL_IDDP)
		return -ENOPROTOOPT;

	switch (sopt.optname) {

	case IDDP_POOLSZ:
		ret = rtipc_get_length(fd, &len, sopt.optval, sopt.optlen);
		if (ret)
			return ret;
		if (len == 0)
			return -EINVAL;
		cobalt_atomic_enter(s);
		/*
		 * We may not do this more than once, and we have to
		 * do this before the first binding.
		 */
		if (test_bit(_IDDP_BOUND, &sk->status) ||
		    test_bit(_IDDP_BINDING, &sk->status))
			ret = -EALREADY;
		else
			sk->poolsz = len;
		cobalt_atomic_leave(s);
		break;

	case IDDP_LABEL:
		if (sopt.optlen < sizeof(plabel))
			return -EINVAL;
		if (rtipc_get_arg(fd, &plabel, sopt.optval, sizeof(plabel)))
			return -EFAULT;
		cobalt_atomic_enter(s);
		/*
		 * We may attach a label to a client socket which was
		 * previously bound in IDDP.
		 */
		if (test_bit(_IDDP_BINDING, &sk->status))
			ret = -EALREADY;
		else {
			strcpy(sk->label, plabel.label);
			sk->label[XNOBJECT_NAME_LEN-1] = 0;
		}
		cobalt_atomic_leave(s);
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

static int __iddp_getsockopt(struct iddp_socket *sk,
			     struct rtdm_fd *fd,
			     void *arg)
{
	struct _rtdm_getsockopt_args sopt;
	struct rtipc_port_label plabel;
	struct timeval tv;
	rtdm_lockctx_t s;
	socklen_t len;
	int ret;

	ret = rtipc_get_sockoptout(fd, &sopt, arg);
	if (ret)
		return ret;

	ret = rtipc_get_arg(fd, &len, sopt.optlen, sizeof(len));
	if (ret)
		return ret;

	if (sopt.level == SOL_SOCKET) {
		switch (sopt.optname) {

		case SO_RCVTIMEO_OLD:
			rtipc_ns_to_timeval(&tv, sk->rx_timeout);
			ret = rtipc_put_timeval(fd, sopt.optval, &tv, len);
			if (ret)
				return ret;
			break;

		case SO_SNDTIMEO_OLD:
			rtipc_ns_to_timeval(&tv, sk->tx_timeout);
			ret = rtipc_put_timeval(fd, sopt.optval, &tv, len);
			if (ret)
				return ret;
			break;

		default:
			ret = -EINVAL;
		}

		return ret;
	}

	if (sopt.level != SOL_IDDP)
		return -ENOPROTOOPT;

	switch (sopt.optname) {

	case IDDP_LABEL:
		if (len < sizeof(plabel))
			return -EINVAL;
		cobalt_atomic_enter(s);
		strcpy(plabel.label, sk->label);
		cobalt_atomic_leave(s);
		if (rtipc_put_arg(fd, sopt.optval, &plabel, sizeof(plabel)))
			return -EFAULT;
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

static int __iddp_ioctl(struct rtdm_fd *fd,
			unsigned int request, void *arg)
{
	struct rtipc_private *priv = rtdm_fd_to_private(fd);
	struct sockaddr_ipc saddr, *saddrp = &saddr;
	struct iddp_socket *sk = priv->state;
	int ret = 0;

	switch (request) {

	COMPAT_CASE(_RTIOC_CONNECT):
		ret = rtipc_get_sockaddr(fd, &saddrp, arg);
		if (ret)
		  return ret;
		ret = __iddp_connect_socket(sk, saddrp);
		break;

	COMPAT_CASE(_RTIOC_BIND):
		ret = rtipc_get_sockaddr(fd, &saddrp, arg);
		if (ret)
			return ret;
		if (saddrp == NULL)
			return -EFAULT;
		ret = __iddp_bind_socket(fd, saddrp);
		break;

	COMPAT_CASE(_RTIOC_GETSOCKNAME):
		ret = rtipc_put_sockaddr(fd, arg, &sk->name);
		break;

	COMPAT_CASE(_RTIOC_GETPEERNAME):
		ret = rtipc_put_sockaddr(fd, arg, &sk->peer);
		break;

	COMPAT_CASE(_RTIOC_SETSOCKOPT):
		ret = __iddp_setsockopt(sk, fd, arg);
		break;

	COMPAT_CASE(_RTIOC_GETSOCKOPT):
		ret = __iddp_getsockopt(sk, fd, arg);
		break;

	case _RTIOC_LISTEN:
	COMPAT_CASE(_RTIOC_ACCEPT):
		ret = -EOPNOTSUPP;
		break;

	case _RTIOC_SHUTDOWN:
		ret = -ENOTCONN;
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

static int iddp_ioctl(struct rtdm_fd *fd,
		      unsigned int request, void *arg)
{
	int ret;

	switch (request) {
	COMPAT_CASE(_RTIOC_BIND):
		if (rtdm_in_rt_context())
			return -ENOSYS;	/* Try downgrading to NRT */
		/* fallthrough */
	default:
		ret = __iddp_ioctl(fd, request, arg);
	}

	return ret;
}

static int iddp_init(void)
{
	portmap = xnmap_create(CONFIG_XENO_OPT_IDDP_NRPORT, 0, 0);
	if (portmap == NULL)
		return -ENOMEM;

	rtdm_waitqueue_init(&poolwaitq);

	return 0;
}

static void iddp_exit(void)
{
	rtdm_waitqueue_destroy(&poolwaitq);
	xnmap_delete(portmap);
}

static unsigned int iddp_pollstate(struct rtdm_fd *fd) /* atomic */
{
	struct rtipc_private *priv = rtdm_fd_to_private(fd);
	struct iddp_socket *sk = priv->state;
	unsigned int mask = 0;
	struct rtdm_fd *rfd;

	if (test_bit(_IDDP_BOUND, &sk->status) && !list_empty(&sk->inq))
		mask |= POLLIN;

	/*
	 * If the socket is connected, POLLOUT means that the peer
	 * exists. Otherwise POLLOUT is always set, assuming the
	 * client is likely to use explicit addressing in send
	 * operations.
	 *
	 * If the peer exists, we still can't really know whether
	 * writing to the socket would block as it depends on the
	 * message size and other highly dynamic factors, so pretend
	 * it would not.
	 */
	if (test_bit(_IDDP_CONNECTED, &sk->status)) {
		rfd = xnmap_fetch_nocheck(portmap, sk->peer.sipc_port);
		if (rfd)
			mask |= POLLOUT;
	} else
		mask |= POLLOUT;

	return mask;
}

struct rtipc_protocol iddp_proto_driver = {
	.proto_name = "iddp",
	.proto_statesz = sizeof(struct iddp_socket),
	.proto_init = iddp_init,
	.proto_exit = iddp_exit,
	.proto_ops = {
		.socket = iddp_socket,
		.close = iddp_close,
		.recvmsg = iddp_recvmsg,
		.sendmsg = iddp_sendmsg,
		.read = iddp_read,
		.write = iddp_write,
		.ioctl = iddp_ioctl,
		.pollstate = iddp_pollstate,
	}
};
