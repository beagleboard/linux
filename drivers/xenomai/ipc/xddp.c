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
#include <linux/string.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <cobalt/kernel/heap.h>
#include <cobalt/kernel/bufd.h>
#include <cobalt/kernel/pipe.h>
#include <rtdm/ipc.h>
#include "internal.h"

#define XDDP_SOCKET_MAGIC 0xa21a21a2

struct xddp_message {
	struct xnpipe_mh mh;
	char data[];
};

struct xddp_socket {
	int magic;
	struct sockaddr_ipc name;
	struct sockaddr_ipc peer;

	int minor;
	size_t poolsz;
	xnhandle_t handle;
	char label[XNOBJECT_NAME_LEN];
	struct rtdm_fd *fd;			/* i.e. RTDM socket fd */

	struct xddp_message *buffer;
	int buffer_port;
	struct xnheap *bufpool;
	struct xnheap privpool;
	size_t fillsz;
	size_t curbufsz;	/* Current streaming buffer size */
	u_long status;
	rtdm_lock_t lock;

	nanosecs_rel_t timeout;	/* connect()/recvmsg() timeout */
	size_t reqbufsz;	/* Requested streaming buffer size */

	int (*monitor)(struct rtdm_fd *fd, int event, long arg);
	struct rtipc_private *priv;
};

static struct sockaddr_ipc nullsa = {
	.sipc_family = AF_RTIPC,
	.sipc_port = -1
};

static struct rtdm_fd *portmap[CONFIG_XENO_OPT_PIPE_NRDEV]; /* indexes RTDM fildes */

#define _XDDP_SYNCWAIT  0
#define _XDDP_ATOMIC    1
#define _XDDP_BINDING   2
#define _XDDP_BOUND     3
#define _XDDP_CONNECTED 4

#ifdef CONFIG_XENO_OPT_VFILE

static char *__xddp_link_target(void *obj)
{
	struct xddp_socket *sk = obj;

	return kasformat("/dev/rtp%d", sk->minor);
}

extern struct xnptree rtipc_ptree;

static struct xnpnode_link __xddp_pnode = {
	.node = {
		.dirname = "xddp",
		.root = &rtipc_ptree,
		.ops = &xnregistry_vlink_ops,
	},
	.target = __xddp_link_target,
};

#else /* !CONFIG_XENO_OPT_VFILE */

static struct xnpnode_link __xddp_pnode = {
	.node = {
		.dirname = "xddp",
	},
};

#endif /* !CONFIG_XENO_OPT_VFILE */

static void *__xddp_alloc_handler(size_t size, void *skarg) /* nklock free */
{
	struct xddp_socket *sk = skarg;
	void *buf;

	/* Try to allocate memory for the incoming message. */
	buf = xnheap_alloc(sk->bufpool, size);
	if (unlikely(buf == NULL)) {
		if (sk->monitor)
			sk->monitor(sk->fd, XDDP_EVTNOBUF, size);
		if (size > xnheap_get_size(sk->bufpool))
			buf = (void *)-1; /* Will never succeed. */
	}

	return buf;
}

static int __xddp_resize_streambuf(struct xddp_socket *sk) /* sk->lock held */
{
	if (sk->buffer)
		xnheap_free(sk->bufpool, sk->buffer);

	if (sk->reqbufsz == 0) {
		sk->buffer = NULL;
		sk->curbufsz = 0;
		return 0;
	}

	sk->buffer = xnheap_alloc(sk->bufpool, sk->reqbufsz);
	if (sk->buffer == NULL) {
		sk->curbufsz = 0;
		return -ENOMEM;
	}

	sk->curbufsz = sk->reqbufsz;

	return 0;
}

static void __xddp_free_handler(void *buf, void *skarg) /* nklock free */
{
	struct xddp_socket *sk = skarg;
	rtdm_lockctx_t s;

	if (buf != sk->buffer) {
		xnheap_free(sk->bufpool, buf);
		return;
	}

	/* Reset the streaming buffer. */

	rtdm_lock_get_irqsave(&sk->lock, s);

	sk->fillsz = 0;
	sk->buffer_port = -1;
	__clear_bit(_XDDP_SYNCWAIT, &sk->status);
	__clear_bit(_XDDP_ATOMIC, &sk->status);

	/*
	 * If a XDDP_BUFSZ request is pending, resize the streaming
	 * buffer on-the-fly.
	 */
	if (unlikely(sk->curbufsz != sk->reqbufsz))
		__xddp_resize_streambuf(sk);

	rtdm_lock_put_irqrestore(&sk->lock, s);
}

static void __xddp_output_handler(struct xnpipe_mh *mh, void *skarg) /* nklock held */
{
	struct xddp_socket *sk = skarg;

	if (sk->monitor)
		sk->monitor(sk->fd, XDDP_EVTOUT, xnpipe_m_size(mh));
}

static int __xddp_input_handler(struct xnpipe_mh *mh, int retval, void *skarg) /* nklock held */
{
	struct xddp_socket *sk = skarg;

	if (sk->monitor) {
		if (retval == 0)
			/* Callee may alter the return value passed to userland. */
			retval = sk->monitor(sk->fd, XDDP_EVTIN, xnpipe_m_size(mh));
		else if (retval == -EPIPE && mh == NULL)
			sk->monitor(sk->fd, XDDP_EVTDOWN, 0);
	}

	if (retval == 0 &&
	    (__xnpipe_pollstate(sk->minor) & POLLIN) != 0 &&
	    xnselect_signal(&sk->priv->recv_block, POLLIN))
		xnsched_run();

	return retval;
}

static void __xddp_release_handler(void *skarg) /* nklock free */
{
	struct xddp_socket *sk = skarg;
	void *poolmem;
	u32 poolsz;

	if (sk->bufpool == &sk->privpool) {
		poolmem = xnheap_get_membase(&sk->privpool);
		poolsz = xnheap_get_size(&sk->privpool);
		xnheap_destroy(&sk->privpool);
		xnheap_vfree(poolmem);
	} else if (sk->buffer)
		xnfree(sk->buffer);

	kfree(sk);
}

static int xddp_socket(struct rtdm_fd *fd)
{
	struct rtipc_private *priv = rtdm_fd_to_private(fd);
	struct xddp_socket *sk = priv->state;

	sk->magic = XDDP_SOCKET_MAGIC;
	sk->name = nullsa;	/* Unbound */
	sk->peer = nullsa;
	sk->minor = -1;
	sk->handle = 0;
	*sk->label = 0;
	sk->poolsz = 0;
	sk->buffer = NULL;
	sk->buffer_port = -1;
	sk->bufpool = NULL;
	sk->fillsz = 0;
	sk->status = 0;
	sk->timeout = RTDM_TIMEOUT_INFINITE;
	sk->curbufsz = 0;
	sk->reqbufsz = 0;
	sk->monitor = NULL;
	rtdm_lock_init(&sk->lock);
	sk->priv = priv;

	return 0;
}

static void xddp_close(struct rtdm_fd *fd)
{
	struct rtipc_private *priv = rtdm_fd_to_private(fd);
	struct xddp_socket *sk = priv->state;
	rtdm_lockctx_t s;

	sk->monitor = NULL;

	if (!test_bit(_XDDP_BOUND, &sk->status))
		return;

	cobalt_atomic_enter(s);
	portmap[sk->name.sipc_port] = NULL;
	cobalt_atomic_leave(s);

	if (sk->handle)
		xnregistry_remove(sk->handle);

	xnpipe_disconnect(sk->minor);
}

static ssize_t __xddp_recvmsg(struct rtdm_fd *fd,
			      struct iovec *iov, int iovlen, int flags,
			      struct sockaddr_ipc *saddr)
{
	struct rtipc_private *priv = rtdm_fd_to_private(fd);
	struct xddp_message *mbuf = NULL; /* Fake GCC */
	struct xddp_socket *sk = priv->state;
	ssize_t maxlen, len, wrlen, vlen;
	nanosecs_rel_t timeout;
	struct xnpipe_mh *mh;
	int nvec, rdoff, ret;
	struct xnbufd bufd;
	spl_t s;

	if (!test_bit(_XDDP_BOUND, &sk->status))
		return -EAGAIN;

	maxlen = rtipc_get_iov_flatlen(iov, iovlen);
	if (maxlen == 0)
		return 0;

	timeout = (flags & MSG_DONTWAIT) ? RTDM_TIMEOUT_NONE : sk->timeout;
	/* Pull heading message from the input queue. */
	len = xnpipe_recv(sk->minor, &mh, timeout);
	if (len < 0)
		return len == -EIDRM ? 0 : len;
	if (len > maxlen) {
		ret = -ENOBUFS;
		goto out;
	}

	mbuf = container_of(mh, struct xddp_message, mh);

	if (saddr)
		*saddr = sk->name;

	/* Write "len" bytes from mbuf->data to the vector cells */
	for (ret = 0, nvec = 0, rdoff = 0, wrlen = len;
	     nvec < iovlen && wrlen > 0; nvec++) {
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
			goto out;
		iov[nvec].iov_base += vlen;
		iov[nvec].iov_len -= vlen;
		wrlen -= vlen;
		rdoff += vlen;
	}
out:
	xnheap_free(sk->bufpool, mbuf);
	cobalt_atomic_enter(s);
	if ((__xnpipe_pollstate(sk->minor) & POLLIN) == 0 &&
	    xnselect_signal(&priv->recv_block, 0))
		xnsched_run();
	cobalt_atomic_leave(s);

	return ret ?: len;
}

static ssize_t xddp_recvmsg(struct rtdm_fd *fd,
			    struct user_msghdr *msg, int flags)
{
	struct iovec iov[RTIPC_IOV_MAX];
	struct sockaddr_ipc saddr;
	ssize_t ret;

	if (flags & ~MSG_DONTWAIT)
		return -EINVAL;

	if (msg->msg_name) {
		if (msg->msg_namelen < sizeof(struct sockaddr_ipc))
			return -EINVAL;
	} else if (msg->msg_namelen != 0)
		return -EINVAL;

	if (msg->msg_iovlen >= RTIPC_IOV_MAX)
		return -EINVAL;

	/* Copy I/O vector in */
	ret = rtipc_get_iovec(fd, iov, msg);
	if (ret)
		return ret;

	ret = __xddp_recvmsg(fd, iov, msg->msg_iovlen, flags, &saddr);
	if (ret <= 0)
		return ret;

	/* Copy the updated I/O vector back */
	if (rtipc_put_iovec(fd, iov, msg))
		return -EFAULT;

	/* Copy the source address if required. */
	if (msg->msg_name) {
		if (rtipc_put_arg(fd, msg->msg_name, &saddr, sizeof(saddr)))
			return -EFAULT;
		msg->msg_namelen = sizeof(struct sockaddr_ipc);
	}

	return ret;
}

static ssize_t xddp_read(struct rtdm_fd *fd, void *buf, size_t len)
{
	struct iovec iov = { .iov_base = buf, .iov_len = len };

	return __xddp_recvmsg(fd, &iov, 1, 0, NULL);
}

static ssize_t __xddp_stream(struct xddp_socket *sk,
			     int from, struct xnbufd *bufd)
{
	struct xddp_message *mbuf;
	size_t fillptr, rembytes;
	rtdm_lockctx_t s;
	ssize_t outbytes;
	int ret;

	/*
	 * xnpipe_msend() and xnpipe_mfixup() routines will only grab
	 * the nklock directly or indirectly, so holding our socket
	 * lock across those calls is fine.
	 */
	rtdm_lock_get_irqsave(&sk->lock, s);

	/*
	 * There are two cases in which we must remove the cork
	 * unconditionally and send the incoming data as a standalone
	 * datagram: the destination port does not support streaming,
	 * or its streaming buffer is already filled with data issued
	 * from another port.
	 */
	if (sk->curbufsz == 0 ||
	    (sk->buffer_port >= 0 && sk->buffer_port != from)) {
		/* This will end up into a standalone datagram. */
		outbytes = 0;
		goto out;
	}

	mbuf = sk->buffer;
	rembytes = sk->curbufsz - sizeof(*mbuf) - sk->fillsz;
	outbytes = bufd->b_len > rembytes ? rembytes : bufd->b_len;
	if (likely(outbytes > 0)) {
	repeat:
		/* Mark the beginning of a should-be-atomic section. */
		__set_bit(_XDDP_ATOMIC, &sk->status);
		fillptr = sk->fillsz;
		sk->fillsz += outbytes;

		rtdm_lock_put_irqrestore(&sk->lock, s);
		ret = xnbufd_copy_to_kmem(mbuf->data + fillptr,
					  bufd, outbytes);
		rtdm_lock_get_irqsave(&sk->lock, s);

		if (ret < 0) {
			outbytes = ret;
			__clear_bit(_XDDP_ATOMIC, &sk->status);
			goto out;
		}

		/* We haven't been atomic, let's try again. */
		if (!__test_and_clear_bit(_XDDP_ATOMIC, &sk->status))
			goto repeat;

		if (__test_and_set_bit(_XDDP_SYNCWAIT, &sk->status))
			outbytes = xnpipe_mfixup(sk->minor,
						 &mbuf->mh, outbytes);
		else {
			sk->buffer_port = from;
			outbytes = xnpipe_send(sk->minor, &mbuf->mh,
					       outbytes + sizeof(*mbuf),
					       XNPIPE_NORMAL);
			if (outbytes > 0)
				outbytes -= sizeof(*mbuf);
		}
	}

out:
	rtdm_lock_put_irqrestore(&sk->lock, s);

	return outbytes;
}

static ssize_t __xddp_sendmsg(struct rtdm_fd *fd,
			      struct iovec *iov, int iovlen, int flags,
			      const struct sockaddr_ipc *daddr)
{
	struct rtipc_private *priv = rtdm_fd_to_private(fd);
	ssize_t len, rdlen, wrlen, vlen, ret, sublen;
	struct xddp_socket *sk = priv->state;
	struct xddp_message *mbuf;
	struct xddp_socket *rsk;
	struct rtdm_fd *rfd;
	int nvec, to, from;
	struct xnbufd bufd;
	rtdm_lockctx_t s;

	len = rtipc_get_iov_flatlen(iov, iovlen);
	if (len == 0)
		return 0;

	from = sk->name.sipc_port;
	to = daddr->sipc_port;

	cobalt_atomic_enter(s);
	rfd = portmap[to];
	if (rfd && rtdm_fd_lock(rfd) < 0)
		rfd = NULL;
	cobalt_atomic_leave(s);

	if (rfd == NULL)
		return -ECONNRESET;

	rsk = rtipc_fd_to_state(rfd);
	if (!test_bit(_XDDP_BOUND, &rsk->status)) {
		rtdm_fd_unlock(rfd);
		return -ECONNREFUSED;
	}

	sublen = len;
	nvec = 0;

	/*
	 * If active, the streaming buffer is already pending on the
	 * output queue, so we basically have nothing to do during a
	 * MSG_MORE -> MSG_NONE transition. Therefore, we only have to
	 * take care of filling that buffer when MSG_MORE is
	 * given. Yummie.
	 */
	if (flags & MSG_MORE) {
		for (rdlen = sublen, wrlen = 0;
		     nvec < iovlen && rdlen > 0; nvec++) {
			if (iov[nvec].iov_len == 0)
				continue;
			vlen = rdlen >= iov[nvec].iov_len ? iov[nvec].iov_len : rdlen;
			if (rtdm_fd_is_user(fd)) {
				xnbufd_map_uread(&bufd, iov[nvec].iov_base, vlen);
				ret = __xddp_stream(rsk, from, &bufd);
				xnbufd_unmap_uread(&bufd);
			} else {
				xnbufd_map_kread(&bufd, iov[nvec].iov_base, vlen);
				ret = __xddp_stream(rsk, from, &bufd);
				xnbufd_unmap_kread(&bufd);
			}
			if (ret < 0)
				goto fail_unlock;
			wrlen += ret;
			rdlen -= ret;
			iov[nvec].iov_base += ret;
			iov[nvec].iov_len -= ret;
			/*
			 * In case of a short write to the streaming
			 * buffer, send the unsent part as a
			 * standalone datagram.
			 */
			if (ret < vlen) {
				sublen = rdlen;
				goto nostream;
			}
		}
		len = wrlen;
		goto done;
	}

nostream:
	mbuf = xnheap_alloc(rsk->bufpool, sublen + sizeof(*mbuf));
	if (unlikely(mbuf == NULL)) {
		ret = -ENOMEM;
		goto fail_unlock;
	}

	/*
	 * Move "sublen" bytes to mbuf->data from the vector cells
	 */
	for (rdlen = sublen, wrlen = 0; nvec < iovlen && rdlen > 0; nvec++) {
		if (iov[nvec].iov_len == 0)
			continue;
		vlen = rdlen >= iov[nvec].iov_len ? iov[nvec].iov_len : rdlen;
		if (rtdm_fd_is_user(fd)) {
			xnbufd_map_uread(&bufd, iov[nvec].iov_base, vlen);
			ret = xnbufd_copy_to_kmem(mbuf->data + wrlen, &bufd, vlen);
			xnbufd_unmap_uread(&bufd);
		} else {
			xnbufd_map_kread(&bufd, iov[nvec].iov_base, vlen);
			ret = xnbufd_copy_to_kmem(mbuf->data + wrlen, &bufd, vlen);
			xnbufd_unmap_kread(&bufd);
		}
		if (ret < 0)
			goto fail_freebuf;
		iov[nvec].iov_base += vlen;
		iov[nvec].iov_len -= vlen;
		rdlen -= vlen;
		wrlen += vlen;
	}

	ret = xnpipe_send(rsk->minor, &mbuf->mh,
			  sublen + sizeof(*mbuf),
			  (flags & MSG_OOB) ?
			  XNPIPE_URGENT : XNPIPE_NORMAL);

	if (unlikely(ret < 0)) {
	fail_freebuf:
		xnheap_free(rsk->bufpool, mbuf);
	fail_unlock:
		rtdm_fd_unlock(rfd);
		return ret;
	}
done:
	rtdm_fd_unlock(rfd);

	return len;
}

static ssize_t xddp_sendmsg(struct rtdm_fd *fd,
			    const struct user_msghdr *msg, int flags)
{
	struct rtipc_private *priv = rtdm_fd_to_private(fd);
	struct xddp_socket *sk = priv->state;
	struct iovec iov[RTIPC_IOV_MAX];
	struct sockaddr_ipc daddr;
	ssize_t ret;

	/*
	 * We accept MSG_DONTWAIT, but do not care about it, since
	 * writing to the real-time endpoint of a message pipe must be
	 * a non-blocking operation.
	 */
	if (flags & ~(MSG_MORE | MSG_OOB | MSG_DONTWAIT))
		return -EINVAL;

	/*
	 * MSG_MORE and MSG_OOB are mutually exclusive in our
	 * implementation.
	 */
	if ((flags & (MSG_MORE | MSG_OOB)) == (MSG_MORE | MSG_OOB))
		return -EINVAL;

	if (msg->msg_name) {
		if (msg->msg_namelen != sizeof(struct sockaddr_ipc))
			return -EINVAL;

		/* Fetch the destination address to send to. */
		if (rtipc_get_arg(fd, &daddr, msg->msg_name, sizeof(daddr)))
			return -EFAULT;

		if (daddr.sipc_port < 0 ||
		    daddr.sipc_port >= CONFIG_XENO_OPT_PIPE_NRDEV)
			return -EINVAL;
	} else {
		if (msg->msg_namelen != 0)
			return -EINVAL;
		daddr = sk->peer;
		if (daddr.sipc_port < 0)
			return -EDESTADDRREQ;
	}

	if (msg->msg_iovlen >= RTIPC_IOV_MAX)
		return -EINVAL;

	/* Copy I/O vector in */
	ret = rtipc_get_iovec(fd, iov, msg);
	if (ret)
		return ret;

	ret = __xddp_sendmsg(fd, iov, msg->msg_iovlen, flags, &daddr);
	if (ret <= 0)
		return ret;

	/* Copy updated I/O vector back */
	return rtipc_put_iovec(fd, iov, msg) ?: ret;
}

static ssize_t xddp_write(struct rtdm_fd *fd,
			  const void *buf, size_t len)
{
	struct rtipc_private *priv = rtdm_fd_to_private(fd);
	struct iovec iov = { .iov_base = (void *)buf, .iov_len = len };
	struct xddp_socket *sk = priv->state;

	if (sk->peer.sipc_port < 0)
		return -EDESTADDRREQ;

	return __xddp_sendmsg(fd, &iov, 1, 0, &sk->peer);
}

static int __xddp_bind_socket(struct rtipc_private *priv,
			      struct sockaddr_ipc *sa)
{
	struct xddp_socket *sk = priv->state;
	struct xnpipe_operations ops;
	rtdm_lockctx_t s;
	size_t poolsz;
	void *poolmem;
	int ret = 0;

	if (sa->sipc_family != AF_RTIPC)
		return -EINVAL;

	/* Allow special port -1 for auto-selection. */
	if (sa->sipc_port < -1 ||
	    sa->sipc_port >= CONFIG_XENO_OPT_PIPE_NRDEV)
		return -EINVAL;

	cobalt_atomic_enter(s);
	if (test_bit(_XDDP_BOUND, &sk->status) ||
	    __test_and_set_bit(_XDDP_BINDING, &sk->status))
		ret = -EADDRINUSE;
	cobalt_atomic_leave(s);
	if (ret)
		return ret;

	poolsz = sk->poolsz;
	if (poolsz > 0) {
		poolsz = xnheap_rounded_size(poolsz);
		poolsz += xnheap_rounded_size(sk->reqbufsz);
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

		sk->bufpool = &sk->privpool;
	} else
		sk->bufpool = &cobalt_heap;

	if (sk->reqbufsz > 0) {
		sk->buffer = xnheap_alloc(sk->bufpool, sk->reqbufsz);
		if (sk->buffer == NULL) {
			ret = -ENOMEM;
			goto fail_freeheap;
		}
		sk->curbufsz = sk->reqbufsz;
	}

	sk->fd = rtdm_private_to_fd(priv);

	ops.output = &__xddp_output_handler;
	ops.input = &__xddp_input_handler;
	ops.alloc_ibuf = &__xddp_alloc_handler;
	ops.free_ibuf = &__xddp_free_handler;
	ops.free_obuf = &__xddp_free_handler;
	ops.release = &__xddp_release_handler;

	ret = xnpipe_connect(sa->sipc_port, &ops, sk);
	if (ret < 0) {
		if (ret == -EBUSY)
			ret = -EADDRINUSE;
	fail_freeheap:
		if (poolsz > 0) {
			xnheap_destroy(&sk->privpool);
			xnheap_vfree(poolmem);
		}
	fail:
		clear_bit(_XDDP_BINDING, &sk->status);
		return ret;
	}

	sk->minor = ret;
	sa->sipc_port = ret;
	sk->name = *sa;
	/* Set default destination if unset at binding time. */
	if (sk->peer.sipc_port < 0)
		sk->peer = *sa;

	if (poolsz > 0)
		xnheap_set_name(sk->bufpool, "xddp-pool@%d", sa->sipc_port);

	if (*sk->label) {
		ret = xnregistry_enter(sk->label, sk, &sk->handle,
				       &__xddp_pnode.node);
		if (ret) {
			/* The release handler will cleanup the pool for us. */
			xnpipe_disconnect(sk->minor);
			return ret;
		}
	}

	cobalt_atomic_enter(s);
	portmap[sk->minor] = rtdm_private_to_fd(priv);
	__clear_bit(_XDDP_BINDING, &sk->status);
	__set_bit(_XDDP_BOUND, &sk->status);
	if (xnselect_signal(&priv->send_block, POLLOUT))
		xnsched_run();
	cobalt_atomic_leave(s);

	return 0;
}

static int __xddp_connect_socket(struct xddp_socket *sk,
				 struct sockaddr_ipc *sa)
{
	struct sockaddr_ipc _sa;
	struct xddp_socket *rsk;
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
	    sa->sipc_port >= CONFIG_XENO_OPT_PIPE_NRDEV)
		return -EINVAL;
	/*
	 * - If a valid sipc_port is passed in the [0..NRDEV-1] range,
	 * it is used verbatim and the connection succeeds
	 * immediately, regardless of whether the destination is
	 * bound at the time of the call.
	 *
	 * - If sipc_port is -1 and a label was set via XDDP_LABEL,
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
				      sk->timeout, XN_RELATIVE, &h);
		if (ret)
			return ret;

		cobalt_atomic_enter(s);
		rsk = xnregistry_lookup(h, NULL);
		if (rsk == NULL || rsk->magic != XDDP_SOCKET_MAGIC)
			ret = -EINVAL;
		else {
			/* Fetch labeled port number. */
			sa->sipc_port = rsk->minor;
			resched = xnselect_signal(&sk->priv->send_block, POLLOUT);
		}
		cobalt_atomic_leave(s);
		if (ret)
			return ret;
	} else if (sa->sipc_port < 0)
		sa = &nullsa;
set_assoc:
	cobalt_atomic_enter(s);
	if (!test_bit(_XDDP_BOUND, &sk->status))
		/* Set default name. */
		sk->name = *sa;
	/* Set default destination. */
	sk->peer = *sa;
	if (sa->sipc_port < 0)
		__clear_bit(_XDDP_CONNECTED, &sk->status);
	else
		__set_bit(_XDDP_CONNECTED, &sk->status);
	if (resched)
		xnsched_run();
	cobalt_atomic_leave(s);

	return 0;
}

static int __xddp_setsockopt(struct xddp_socket *sk,
			     struct rtdm_fd *fd,
			     void *arg)
{
	int (*monitor)(struct rtdm_fd *fd, int event, long arg);
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

		case SO_RCVTIMEO:
			ret = rtipc_get_timeval(fd, &tv, sopt.optval, sopt.optlen);
			if (ret)
				return ret;
			sk->timeout = rtipc_timeval_to_ns(&tv);
			break;

		default:
			ret = -EINVAL;
		}

		return ret;
	}

	if (sopt.level != SOL_XDDP)
		return -ENOPROTOOPT;

	switch (sopt.optname) {

	case XDDP_BUFSZ:
		ret = rtipc_get_length(fd, &len, sopt.optval, sopt.optlen);
		if (ret)
			return ret;
		if (len > 0) {
			len += sizeof(struct xddp_message);
			if (sk->bufpool &&
			    len > xnheap_get_size(sk->bufpool)) {
				return -EINVAL;
			}
		}
		rtdm_lock_get_irqsave(&sk->lock, s);
		sk->reqbufsz = len;
		if (len != sk->curbufsz &&
		    !test_bit(_XDDP_SYNCWAIT, &sk->status) &&
		    test_bit(_XDDP_BOUND, &sk->status))
			ret = __xddp_resize_streambuf(sk);
		rtdm_lock_put_irqrestore(&sk->lock, s);
		break;

	case XDDP_POOLSZ:
		ret = rtipc_get_length(fd, &len, sopt.optval, sopt.optlen);
		if (ret)
			return ret;
		if (len == 0)
			return -EINVAL;
		cobalt_atomic_enter(s);
		if (test_bit(_XDDP_BOUND, &sk->status) ||
		    test_bit(_XDDP_BINDING, &sk->status))
			ret = -EALREADY;
		else
			sk->poolsz = len;
		cobalt_atomic_leave(s);
		break;

	case XDDP_MONITOR:
		/* Monitoring is available from kernel-space only. */
		if (rtdm_fd_is_user(fd))
			return -EPERM;
		if (sopt.optlen != sizeof(monitor))
			return -EINVAL;
		if (rtipc_get_arg(NULL, &monitor, sopt.optval, sizeof(monitor)))
			return -EFAULT;
		sk->monitor = monitor;
		break;

	case XDDP_LABEL:
		if (sopt.optlen < sizeof(plabel))
			return -EINVAL;
		if (rtipc_get_arg(fd, &plabel, sopt.optval, sizeof(plabel)))
			return -EFAULT;
		cobalt_atomic_enter(s);
		if (test_bit(_XDDP_BOUND, &sk->status) ||
		    test_bit(_XDDP_BINDING, &sk->status))
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

static int __xddp_getsockopt(struct xddp_socket *sk,
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

		case SO_RCVTIMEO:
			rtipc_ns_to_timeval(&tv, sk->timeout);
			ret = rtipc_put_timeval(fd, sopt.optval, &tv, len);
			if (ret)
				return ret;
			break;

		default:
			ret = -EINVAL;
		}

		return ret;
	}

	if (sopt.level != SOL_XDDP)
		return -ENOPROTOOPT;

	switch (sopt.optname) {

	case XDDP_LABEL:
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

static int __xddp_ioctl(struct rtdm_fd *fd,
			unsigned int request, void *arg)
{
	struct rtipc_private *priv = rtdm_fd_to_private(fd);
	struct sockaddr_ipc saddr, *saddrp = &saddr;
	struct xddp_socket *sk = priv->state;
	int ret = 0;

	switch (request) {

	COMPAT_CASE(_RTIOC_CONNECT):
		ret = rtipc_get_sockaddr(fd, &saddrp, arg);
		if (ret == 0)
			ret = __xddp_connect_socket(sk, saddrp);
		break;

	COMPAT_CASE(_RTIOC_BIND):
		ret = rtipc_get_sockaddr(fd, &saddrp, arg);
		if (ret)
			return ret;
		if (saddrp == NULL)
			return -EFAULT;
		ret = __xddp_bind_socket(priv, saddrp);
		break;

	COMPAT_CASE(_RTIOC_GETSOCKNAME):
		ret = rtipc_put_sockaddr(fd, arg, &sk->name);
		break;

	COMPAT_CASE(_RTIOC_GETPEERNAME):
		ret = rtipc_put_sockaddr(fd, arg, &sk->peer);
		break;

	COMPAT_CASE(_RTIOC_SETSOCKOPT):
		ret = __xddp_setsockopt(sk, fd, arg);
		break;

	COMPAT_CASE(_RTIOC_GETSOCKOPT):
		ret = __xddp_getsockopt(sk, fd, arg);
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

static int xddp_ioctl(struct rtdm_fd *fd,
		      unsigned int request, void *arg)
{
	int ret;

	switch (request) {
	COMPAT_CASE(_RTIOC_BIND):
		if (rtdm_in_rt_context())
			return -ENOSYS;	/* Try downgrading to NRT */
	default:
		ret = __xddp_ioctl(fd, request, arg);
	}

	return ret;
}

static unsigned int xddp_pollstate(struct rtdm_fd *fd) /* atomic */
{
	struct rtipc_private *priv = rtdm_fd_to_private(fd);
	struct xddp_socket *sk = priv->state, *rsk;
	unsigned int mask = 0, pollstate;
	struct rtdm_fd *rfd;

	pollstate = __xnpipe_pollstate(sk->minor);
	if (test_bit(_XDDP_BOUND, &sk->status))
		mask |= (pollstate & POLLIN);

	/*
	 * If the socket is connected, POLLOUT means that the peer
	 * exists, is bound and can receive data. Otherwise POLLOUT is
	 * always set, assuming the client is likely to use explicit
	 * addressing in send operations.
	 */
	if (test_bit(_XDDP_CONNECTED, &sk->status)) {
		rfd = portmap[sk->peer.sipc_port];
		if (rfd) {
			rsk = rtipc_fd_to_state(rfd);
			mask |= (pollstate & POLLOUT);
		}
	} else
		mask |= POLLOUT;

	return mask;
}

struct rtipc_protocol xddp_proto_driver = {
	.proto_name = "xddp",
	.proto_statesz = sizeof(struct xddp_socket),
	.proto_ops = {
		.socket = xddp_socket,
		.close = xddp_close,
		.recvmsg = xddp_recvmsg,
		.sendmsg = xddp_sendmsg,
		.read = xddp_read,
		.write = xddp_write,
		.ioctl = xddp_ioctl,
		.pollstate = xddp_pollstate,
	}
};
