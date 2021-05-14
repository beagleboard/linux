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
#include <cobalt/kernel/map.h>
#include <cobalt/kernel/bufd.h>
#include <rtdm/ipc.h>
#include "internal.h"

#define BUFP_SOCKET_MAGIC 0xa61a61a6

struct bufp_socket {
	int magic;
	struct sockaddr_ipc name;
	struct sockaddr_ipc peer;

	void *bufmem;
	size_t bufsz;
	u_long status;
	xnhandle_t handle;
	char label[XNOBJECT_NAME_LEN];

	off_t rdoff;
	off_t rdrsvd;
	int rdsem;
	off_t wroff;
	off_t wrrsvd;
	int wrsem;
	size_t fillsz;
	rtdm_event_t i_event;
	rtdm_event_t o_event;

	nanosecs_rel_t rx_timeout;
	nanosecs_rel_t tx_timeout;

	struct rtipc_private *priv;
};

struct bufp_wait_context {
	struct rtipc_wait_context wc;
	size_t len;
	struct bufp_socket *sk;
};

static struct sockaddr_ipc nullsa = {
	.sipc_family = AF_RTIPC,
	.sipc_port = -1
};

static struct xnmap *portmap;

#define _BUFP_BINDING   0
#define _BUFP_BOUND     1
#define _BUFP_CONNECTED 2

#ifdef CONFIG_XENO_OPT_VFILE

static char *__bufp_link_target(void *obj)
{
	struct bufp_socket *sk = obj;

	return kasformat("%d", sk->name.sipc_port);
}

extern struct xnptree rtipc_ptree;

static struct xnpnode_link __bufp_pnode = {
	.node = {
		.dirname = "bufp",
		.root = &rtipc_ptree,
		.ops = &xnregistry_vlink_ops,
	},
	.target = __bufp_link_target,
};

#else /* !CONFIG_XENO_OPT_VFILE */

static struct xnpnode_link __bufp_pnode = {
	.node = {
		.dirname = "bufp",
	},
};

#endif /* !CONFIG_XENO_OPT_VFILE */

static int bufp_socket(struct rtdm_fd *fd)
{
	struct rtipc_private *priv = rtdm_fd_to_private(fd);
	struct bufp_socket *sk = priv->state;

	sk->magic = BUFP_SOCKET_MAGIC;
	sk->name = nullsa;	/* Unbound */
	sk->peer = nullsa;
	sk->bufmem = NULL;
	sk->bufsz = 0;
	sk->rdoff = 0;
	sk->wroff = 0;
	sk->fillsz = 0;
	sk->rdrsvd = 0;
	sk->wrrsvd = 0;
	sk->status = 0;
	sk->handle = 0;
	sk->rx_timeout = RTDM_TIMEOUT_INFINITE;
	sk->tx_timeout = RTDM_TIMEOUT_INFINITE;
	*sk->label = 0;
	rtdm_event_init(&sk->i_event, 0);
	rtdm_event_init(&sk->o_event, 0);
	sk->priv = priv;

	return 0;
}

static void bufp_close(struct rtdm_fd *fd)
{
	struct rtipc_private *priv = rtdm_fd_to_private(fd);
	struct bufp_socket *sk = priv->state;
	rtdm_lockctx_t s;

	rtdm_event_destroy(&sk->i_event);
	rtdm_event_destroy(&sk->o_event);

	if (test_bit(_BUFP_BOUND, &sk->status)) {
		if (sk->name.sipc_port > -1) {
			cobalt_atomic_enter(s);
			xnmap_remove(portmap, sk->name.sipc_port);
			cobalt_atomic_leave(s);
		}

		if (sk->handle)
			xnregistry_remove(sk->handle);

		if (sk->bufmem)
			xnheap_vfree(sk->bufmem);
	}

	kfree(sk);
}

static ssize_t __bufp_readbuf(struct bufp_socket *sk,
			      struct xnbufd *bufd,
			      int flags)
{
	struct bufp_wait_context wait, *bufwc;
	struct rtipc_wait_context *wc;
	struct xnthread *waiter;
	size_t rbytes, n, avail;
	ssize_t len, ret, xret;
	rtdm_toseq_t toseq;
	rtdm_lockctx_t s;
	off_t rdoff;
	int resched;

	len = bufd->b_len;

	rtdm_toseq_init(&toseq, sk->rx_timeout);

	cobalt_atomic_enter(s);
redo:
	for (;;) {
		/*
		 * We should be able to read a complete message of the
		 * requested length, or block.
		 */
		avail = sk->fillsz - sk->rdrsvd;
		if (avail < len)
			goto wait;

		/* Reserve a read slot into the circular buffer. */
		rdoff = sk->rdoff;
		sk->rdoff = (rdoff + len) % sk->bufsz;
		sk->rdrsvd += len;
		sk->rdsem++;
		rbytes = ret = len;

		do {
			if (rdoff + rbytes > sk->bufsz)
				n = sk->bufsz - rdoff;
			else
				n = rbytes;
			/*
			 * Drop the lock before copying data to
			 * user. The read slot is consumed in any
			 * case: the non-copied portion of the message
			 * is lost on bad write.
			 */
			cobalt_atomic_leave(s);
			xret = xnbufd_copy_from_kmem(bufd, sk->bufmem + rdoff, n);
			cobalt_atomic_enter(s);
			if (xret < 0) {
				ret = -EFAULT;
				break;
			}

			rbytes -= n;
			rdoff = (rdoff + n) % sk->bufsz;
		} while (rbytes > 0);

		if (--sk->rdsem > 0)
			goto out;

		resched = 0;
		if (sk->fillsz == sk->bufsz) /* -> becomes writable */
			resched |= xnselect_signal(&sk->priv->send_block, POLLOUT);

		sk->fillsz -= sk->rdrsvd;
		sk->rdrsvd = 0;

		if (sk->fillsz == 0) /* -> becomes non-readable */
			resched |= xnselect_signal(&sk->priv->recv_block, 0);

		/*
		 * Wake up all threads pending on the output wait
		 * queue, if we freed enough room for the leading one
		 * to post its message.
		 */
		waiter = rtipc_peek_wait_head(&sk->o_event);
		if (waiter == NULL)
			goto out;

		wc = rtipc_get_wait_context(waiter);
		XENO_BUG_ON(COBALT, wc == NULL);
		bufwc = container_of(wc, struct bufp_wait_context, wc);
		if (bufwc->len + sk->fillsz <= sk->bufsz)
			/* This call rescheds internally. */
			rtdm_event_pulse(&sk->o_event);
		else if (resched)
			xnsched_run();
		/*
		 * We cannot fail anymore once some data has been
		 * copied via the buffer descriptor, so no need to
		 * check for any reason to invalidate the latter.
		 */
		goto out;

	wait:
		if (flags & MSG_DONTWAIT) {
			ret = -EWOULDBLOCK;
			break;
		}

		/*
		 * Check whether writers are already waiting for
		 * sending data, while we are about to wait for
		 * receiving some. In such a case, we have a
		 * pathological use of the buffer. We must allow for a
		 * short read to prevent a deadlock.
		 */
		if (sk->fillsz > 0 && rtipc_peek_wait_head(&sk->o_event)) {
			len = sk->fillsz;
			goto redo;
		}

		wait.len = len;
		wait.sk = sk;
		rtipc_prepare_wait(&wait.wc);
		/*
		 * Keep the nucleus lock across the wait call, so that
		 * we don't miss a pulse.
		 */
		ret = rtdm_event_timedwait(&sk->i_event,
					   sk->rx_timeout, &toseq);
		if (unlikely(ret))
			break;
	}
out:
	cobalt_atomic_leave(s);

	return ret;
}

static ssize_t __bufp_recvmsg(struct rtdm_fd *fd,
			      struct iovec *iov, int iovlen, int flags,
			      struct sockaddr_ipc *saddr)
{
	struct rtipc_private *priv = rtdm_fd_to_private(fd);
	struct bufp_socket *sk = priv->state;
	ssize_t len, wrlen, vlen, ret;
	struct xnbufd bufd;
	int nvec;

	if (!test_bit(_BUFP_BOUND, &sk->status))
		return -EAGAIN;

	len = rtdm_get_iov_flatlen(iov, iovlen);
	if (len == 0)
		return 0;
	/*
	 * We may only return complete messages to readers, so there
	 * is no point in waiting for messages which are larger than
	 * what the buffer can hold.
	 */
	if (len > sk->bufsz)
		return -EINVAL;

	/*
	 * Write "len" bytes from the buffer to the vector cells. Each
	 * cell is handled as a separate message.
	 */
	for (nvec = 0, wrlen = len; nvec < iovlen && wrlen > 0; nvec++) {
		if (iov[nvec].iov_len == 0)
			continue;
		vlen = wrlen >= iov[nvec].iov_len ? iov[nvec].iov_len : wrlen;
		if (rtdm_fd_is_user(fd)) {
			xnbufd_map_uread(&bufd, iov[nvec].iov_base, vlen);
			ret = __bufp_readbuf(sk, &bufd, flags);
			xnbufd_unmap_uread(&bufd);
		} else {
			xnbufd_map_kread(&bufd, iov[nvec].iov_base, vlen);
			ret = __bufp_readbuf(sk, &bufd, flags);
			xnbufd_unmap_kread(&bufd);
		}
		if (ret < 0)
			return ret;
		iov[nvec].iov_base += vlen;
		iov[nvec].iov_len -= vlen;
		wrlen -= vlen;
		if (ret < vlen)
			/* Short reads may happen in rare cases. */
			break;
	}

	/*
	 * There is no way to determine who the sender was since we
	 * process data in byte-oriented mode, so we just copy our own
	 * sockaddr to send back a valid address.
	 */
	if (saddr)
		*saddr = sk->name;

	return len - wrlen;
}

static ssize_t bufp_recvmsg(struct rtdm_fd *fd,
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

	ret = __bufp_recvmsg(fd, iov, msg->msg_iovlen, flags, &saddr);
	if (ret <= 0) {
		rtdm_drop_iovec(iov, iov_fast);
		return ret;
	}

	/* Copy the updated I/O vector back */
	if (rtdm_put_iovec(fd, iov, msg, iov_fast))
		return -EFAULT;

	/* Copy the source address if required. */
	if (msg->msg_name) {
		if (rtipc_put_arg(fd, msg->msg_name,
				  &saddr, sizeof(saddr)))
			return -EFAULT;
		msg->msg_namelen = sizeof(struct sockaddr_ipc);
	}

	return ret;
}

static ssize_t bufp_read(struct rtdm_fd *fd, void *buf, size_t len)
{
	struct iovec iov = { .iov_base = buf, .iov_len = len };

	return __bufp_recvmsg(fd, &iov, 1, 0, NULL);
}

static ssize_t __bufp_writebuf(struct bufp_socket *rsk,
			       struct bufp_socket *sk,
			       struct xnbufd *bufd,
			       int flags)
{
	struct bufp_wait_context wait, *bufwc;
	struct rtipc_wait_context *wc;
	struct xnthread *waiter;
	size_t wbytes, n, avail;
	ssize_t len, ret, xret;
	rtdm_toseq_t toseq;
	rtdm_lockctx_t s;
	off_t wroff;
	int resched;

	len = bufd->b_len;

	rtdm_toseq_init(&toseq, sk->tx_timeout);

	cobalt_atomic_enter(s);

	for (;;) {
		/*
		 * No short or scattered writes: we should write the
		 * entire message atomically or block.
		 */
		avail = rsk->fillsz + rsk->wrrsvd;
		if (avail + len > rsk->bufsz)
			goto wait;

		/* Reserve a write slot into the circular buffer. */
		wroff = rsk->wroff;
		rsk->wroff = (wroff + len) % rsk->bufsz;
		rsk->wrrsvd += len;
		rsk->wrsem++;
		wbytes = ret = len;

		do {
			if (wroff + wbytes > rsk->bufsz)
				n = rsk->bufsz - wroff;
			else
				n = wbytes;
			/*
			 * We have to drop the lock while reading in
			 * data, but we can't rollback on bad read
			 * from user because some other thread might
			 * have populated the memory ahead of our
			 * write slot already: bluntly clear the
			 * unavailable bytes on copy error.
			 */
			cobalt_atomic_leave(s);
			xret = xnbufd_copy_to_kmem(rsk->bufmem + wroff, bufd, n);
			cobalt_atomic_enter(s);
			if (xret < 0) {
				memset(rsk->bufmem + wroff, 0, n);
				ret = -EFAULT;
				break;
			}

			wbytes -= n;
			wroff = (wroff + n) % rsk->bufsz;
		} while (wbytes > 0);

		if (--rsk->wrsem > 0)
			goto out;

		resched = 0;
		if (rsk->fillsz == 0) /* -> becomes readable */
			resched |= xnselect_signal(&rsk->priv->recv_block, POLLIN);

		rsk->fillsz += rsk->wrrsvd;
		rsk->wrrsvd = 0;

		if (rsk->fillsz == rsk->bufsz) /* becomes non-writable */
			resched |= xnselect_signal(&rsk->priv->send_block, 0);
		/*
		 * Wake up all threads pending on the input wait
		 * queue, if we accumulated enough data to feed the
		 * leading one.
		 */
		waiter = rtipc_peek_wait_head(&rsk->i_event);
		if (waiter == NULL)
			goto out;

		wc = rtipc_get_wait_context(waiter);
		XENO_BUG_ON(COBALT, wc == NULL);
		bufwc = container_of(wc, struct bufp_wait_context, wc);
		if (bufwc->len <= rsk->fillsz)
			rtdm_event_pulse(&rsk->i_event);
		else if (resched)
			xnsched_run();
		/*
		 * We cannot fail anymore once some data has been
		 * copied via the buffer descriptor, so no need to
		 * check for any reason to invalidate the latter.
		 */
		goto out;
	wait:
		if (flags & MSG_DONTWAIT) {
			ret = -EWOULDBLOCK;
			break;
		}

		wait.len = len;
		wait.sk = rsk;
		rtipc_prepare_wait(&wait.wc);
		/*
		 * Keep the nucleus lock across the wait call, so that
		 * we don't miss a pulse.
		 */
		ret = rtdm_event_timedwait(&rsk->o_event,
					   sk->tx_timeout, &toseq);
		if (unlikely(ret))
			break;
	}
out:
	cobalt_atomic_leave(s);

	return ret;
}

static ssize_t __bufp_sendmsg(struct rtdm_fd *fd,
			      struct iovec *iov, int iovlen, int flags,
			      const struct sockaddr_ipc *daddr)
{
	struct rtipc_private *priv = rtdm_fd_to_private(fd);
	struct bufp_socket *sk = priv->state, *rsk;
	ssize_t len, rdlen, vlen, ret = 0;
	struct rtdm_fd *rfd;
	struct xnbufd bufd;
	rtdm_lockctx_t s;
	int nvec;

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
	if (!test_bit(_BUFP_BOUND, &rsk->status)) {
		rtdm_fd_unlock(rfd);
		return -ECONNREFUSED;
	}

	/*
	 * We may only send complete messages, so there is no point in
	 * accepting messages which are larger than what the buffer
	 * can hold.
	 */
	if (len > rsk->bufsz) {
		ret = -EINVAL;
		goto fail;
	}

	/*
	 * Read "len" bytes to the buffer from the vector cells. Each
	 * cell is handled as a separate message.
	 */
	for (nvec = 0, rdlen = len; nvec < iovlen && rdlen > 0; nvec++) {
		if (iov[nvec].iov_len == 0)
			continue;
		vlen = rdlen >= iov[nvec].iov_len ? iov[nvec].iov_len : rdlen;
		if (rtdm_fd_is_user(fd)) {
			xnbufd_map_uread(&bufd, iov[nvec].iov_base, vlen);
			ret = __bufp_writebuf(rsk, sk, &bufd, flags);
			xnbufd_unmap_uread(&bufd);
		} else {
			xnbufd_map_kread(&bufd, iov[nvec].iov_base, vlen);
			ret = __bufp_writebuf(rsk, sk, &bufd, flags);
			xnbufd_unmap_kread(&bufd);
		}
		if (ret < 0)
			goto fail;
		iov[nvec].iov_base += vlen;
		iov[nvec].iov_len -= vlen;
		rdlen -= vlen;
	}

	rtdm_fd_unlock(rfd);

	return len - rdlen;
fail:
	rtdm_fd_unlock(rfd);

	return ret;
}

static ssize_t bufp_sendmsg(struct rtdm_fd *fd,
			    const struct user_msghdr *msg, int flags)
{
	struct rtipc_private *priv = rtdm_fd_to_private(fd);
	struct iovec iov_fast[RTDM_IOV_FASTMAX], *iov;
	struct bufp_socket *sk = priv->state;
	struct sockaddr_ipc daddr;
	ssize_t ret;

	if (flags & ~MSG_DONTWAIT)
		return -EINVAL;

	if (msg->msg_name) {
		if (msg->msg_namelen != sizeof(struct sockaddr_ipc))
			return -EINVAL;

		/* Fetch the destination address to send to. */
		if (rtipc_get_arg(fd, &daddr, msg->msg_name, sizeof(daddr)))
			return -EFAULT;

		if (daddr.sipc_port < 0 ||
		    daddr.sipc_port >= CONFIG_XENO_OPT_BUFP_NRPORT)
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

	ret = __bufp_sendmsg(fd, iov, msg->msg_iovlen, flags, &daddr);
	if (ret <= 0) {
		rtdm_drop_iovec(iov, iov_fast);
		return ret;
	}

	/* Copy updated I/O vector back */
	return rtdm_put_iovec(fd, iov, msg, iov_fast) ?: ret;
}

static ssize_t bufp_write(struct rtdm_fd *fd,
			  const void *buf, size_t len)
{
	struct rtipc_private *priv = rtdm_fd_to_private(fd);
	struct iovec iov = { .iov_base = (void *)buf, .iov_len = len };
	struct bufp_socket *sk = priv->state;

	if (sk->peer.sipc_port < 0)
		return -EDESTADDRREQ;

	return __bufp_sendmsg(fd, &iov, 1, 0, &sk->peer);
}

static int __bufp_bind_socket(struct rtipc_private *priv,
			      struct sockaddr_ipc *sa)
{
	struct bufp_socket *sk = priv->state;
	int ret = 0, port;
	struct rtdm_fd *fd;
	rtdm_lockctx_t s;

	if (sa->sipc_family != AF_RTIPC)
		return -EINVAL;

	if (sa->sipc_port < -1 ||
	    sa->sipc_port >= CONFIG_XENO_OPT_BUFP_NRPORT)
		return -EINVAL;

	cobalt_atomic_enter(s);
	if (test_bit(_BUFP_BOUND, &sk->status) ||
	    __test_and_set_bit(_BUFP_BINDING, &sk->status))
		ret = -EADDRINUSE;
	cobalt_atomic_leave(s);
	
	if (ret)
		return ret;

	/* Will auto-select a free port number if unspec (-1). */
	port = sa->sipc_port;
	fd = rtdm_private_to_fd(priv);
	cobalt_atomic_enter(s);
	port = xnmap_enter(portmap, port, fd);
	cobalt_atomic_leave(s);
	if (port < 0)
		return port == -EEXIST ? -EADDRINUSE : -ENOMEM;

	sa->sipc_port = port;

	/*
	 * The caller must have told us how much memory is needed for
	 * buffer space via setsockopt(), before we got there.
	 */
	if (sk->bufsz == 0)
		return -ENOBUFS;

	sk->bufmem = xnheap_vmalloc(sk->bufsz);
	if (sk->bufmem == NULL) {
		ret = -ENOMEM;
		goto fail;
	}

	sk->name = *sa;
	/* Set default destination if unset at binding time. */
	if (sk->peer.sipc_port < 0)
		sk->peer = *sa;

	if (*sk->label) {
		ret = xnregistry_enter(sk->label, sk,
				       &sk->handle, &__bufp_pnode.node);
		if (ret) {
			xnheap_vfree(sk->bufmem);
			goto fail;
		}
	}

	cobalt_atomic_enter(s);
	__clear_bit(_BUFP_BINDING, &sk->status);
	__set_bit(_BUFP_BOUND, &sk->status);
	if (xnselect_signal(&priv->send_block, POLLOUT))
		xnsched_run();
	cobalt_atomic_leave(s);

	return 0;
fail:
	xnmap_remove(portmap, port);
	clear_bit(_BUFP_BINDING, &sk->status);

	return ret;
}

static int __bufp_connect_socket(struct bufp_socket *sk,
				 struct sockaddr_ipc *sa)
{
	struct sockaddr_ipc _sa;
	struct bufp_socket *rsk;
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
	    sa->sipc_port >= CONFIG_XENO_OPT_BUFP_NRPORT)
		return -EINVAL;
	/*
	 * - If a valid sipc_port is passed in the [0..NRPORT-1] range,
	 * it is used verbatim and the connection succeeds
	 * immediately, regardless of whether the destination is
	 * bound at the time of the call.
	 *
	 * - If sipc_port is -1 and a label was set via BUFP_LABEL,
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
		if (rsk == NULL || rsk->magic != BUFP_SOCKET_MAGIC)
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
	if (!test_bit(_BUFP_BOUND, &sk->status))
		/* Set default name. */
		sk->name = *sa;
	/* Set default destination. */
	sk->peer = *sa;
	if (sa->sipc_port < 0)
		__clear_bit(_BUFP_CONNECTED, &sk->status);
	else
		__set_bit(_BUFP_CONNECTED, &sk->status);
	if (resched)
		xnsched_run();
	cobalt_atomic_leave(s);

	return 0;
}

static int __bufp_setsockopt(struct bufp_socket *sk,
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

	if (sopt.level != SOL_BUFP)
		return -ENOPROTOOPT;

	switch (sopt.optname) {

	case BUFP_BUFSZ:
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
		if (test_bit(_BUFP_BOUND, &sk->status) ||
		    test_bit(_BUFP_BINDING, &sk->status))
			ret = -EALREADY;
		else
			sk->bufsz = len;
		cobalt_atomic_leave(s);
		break;

	case BUFP_LABEL:
		if (sopt.optlen < sizeof(plabel))
			return -EINVAL;
		if (rtipc_get_arg(fd, &plabel, sopt.optval, sizeof(plabel)))
			return -EFAULT;
		cobalt_atomic_enter(s);
		/*
		 * We may attach a label to a client socket which was
		 * previously bound in BUFP.
		 */
		if (test_bit(_BUFP_BINDING, &sk->status))
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

static int __bufp_getsockopt(struct bufp_socket *sk,
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

	if (rtipc_get_arg(fd, &len, sopt.optlen, sizeof(len)))
		return -EFAULT;

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

	if (sopt.level != SOL_BUFP)
		return -ENOPROTOOPT;

	switch (sopt.optname) {

	case BUFP_LABEL:
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

static int __bufp_ioctl(struct rtdm_fd *fd,
			unsigned int request, void *arg)
{
	struct rtipc_private *priv = rtdm_fd_to_private(fd);
	struct sockaddr_ipc saddr, *saddrp = &saddr;
	struct bufp_socket *sk = priv->state;
	int ret = 0;

	switch (request) {

	COMPAT_CASE(_RTIOC_CONNECT):
		ret = rtipc_get_sockaddr(fd, &saddrp, arg);
		if (ret)
		  return ret;
		ret = __bufp_connect_socket(sk, saddrp);
		break;

	COMPAT_CASE(_RTIOC_BIND):
		ret = rtipc_get_sockaddr(fd, &saddrp, arg);
		if (ret)
			return ret;
		if (saddrp == NULL)
			return -EFAULT;
		ret = __bufp_bind_socket(priv, saddrp);
		break;

	COMPAT_CASE(_RTIOC_GETSOCKNAME):
		ret = rtipc_put_sockaddr(fd, arg, &sk->name);
		break;

	COMPAT_CASE(_RTIOC_GETPEERNAME):
		ret = rtipc_put_sockaddr(fd, arg, &sk->peer);
		break;

	COMPAT_CASE(_RTIOC_SETSOCKOPT):
		ret = __bufp_setsockopt(sk, fd, arg);
		break;

	COMPAT_CASE(_RTIOC_GETSOCKOPT):
		ret = __bufp_getsockopt(sk, fd, arg);
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

static int bufp_ioctl(struct rtdm_fd *fd,
		      unsigned int request, void *arg)
{
	int ret;

	switch (request) {
	COMPAT_CASE(_RTIOC_BIND):
		if (rtdm_in_rt_context())
			return -ENOSYS;	/* Try downgrading to NRT */
		/* fallthrough */
	default:
		ret = __bufp_ioctl(fd, request, arg);
	}

	return ret;
}

static unsigned int bufp_pollstate(struct rtdm_fd *fd) /* atomic */
{
	struct rtipc_private *priv = rtdm_fd_to_private(fd);
	struct bufp_socket *sk = priv->state, *rsk;
	unsigned int mask = 0;
	struct rtdm_fd *rfd;

	if (test_bit(_BUFP_BOUND, &sk->status) && sk->fillsz > 0)
		mask |= POLLIN;

	/*
	 * If the socket is connected, POLLOUT means that the peer
	 * exists, is bound and can receive data. Otherwise POLLOUT is
	 * always set, assuming the client is likely to use explicit
	 * addressing in send operations.
	 */
	if (test_bit(_BUFP_CONNECTED, &sk->status)) {
		rfd = xnmap_fetch_nocheck(portmap, sk->peer.sipc_port);
		if (rfd) {
			rsk = rtipc_fd_to_state(rfd);
			if (rsk->fillsz < rsk->bufsz)
				mask |= POLLOUT;
		}
	} else
		mask |= POLLOUT;

	return mask;
}

static int bufp_init(void)
{
	portmap = xnmap_create(CONFIG_XENO_OPT_BUFP_NRPORT, 0, 0);
	if (portmap == NULL)
		return -ENOMEM;

	return 0;
}

static void bufp_exit(void)
{
	xnmap_delete(portmap);
}

struct rtipc_protocol bufp_proto_driver = {
	.proto_name = "bufp",
	.proto_statesz = sizeof(struct bufp_socket),
	.proto_init = bufp_init,
	.proto_exit = bufp_exit,
	.proto_ops = {
		.socket = bufp_socket,
		.close = bufp_close,
		.recvmsg = bufp_recvmsg,
		.sendmsg = bufp_sendmsg,
		.read = bufp_read,
		.write = bufp_write,
		.ioctl = bufp_ioctl,
		.pollstate = bufp_pollstate,
	}
};
