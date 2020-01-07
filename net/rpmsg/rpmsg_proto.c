// SPDX-License-Identifier: GPL-2.0-only
/* AF_RPMSG: Remote processor messaging sockets
 *
 * Copyright (C) 2011-2020 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Ohad Ben-Cohen <ohad@wizery.com>
 * Robert Tivy <rtivy@ti.com>
 * G Anthony <a0783926@ti.com>
 * Suman Anna <s-anna@ti.com>
 */

#define pr_fmt(fmt)    "%s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/skbuff.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/rpmsg.h>
#include <linux/radix-tree.h>
#include <linux/remoteproc.h>
#include <net/sock.h>
#include <uapi/linux/rpmsg_socket.h>

#define RPMSG_CB(skb)	(*(struct sockaddr_rpmsg *)&((skb)->cb))

/* Maximum buffer size supported by virtio rpmsg transport.
 * Must match value as in drivers/rpmsg/virtio_rpmsg_bus.c
 */
#define RPMSG_BUF_SIZE               (512)

struct rpmsg_socket {
	struct sock sk;
	struct rpmsg_device *rpdev;
	struct rpmsg_endpoint *endpt;
	int rproc_id;
};

/* Connection and socket states */
enum {
	RPMSG_CONNECTED = 1,
	RPMSG_OPEN,
	RPMSG_LISTENING,
	RPMSG_CLOSED,
};

/* A single-level radix-tree-based scheme is used to maintain the rpmsg
 * channels we're exposing to userland. The radix tree maps a rproc index
 * id to its published rpmsg-proto channel. Only a single rpmsg device is
 * supported at the moment from each remote processor. This can be easily
 * scaled to multiple devices using unique destination addresses but this
 *_will_ require additional semantic changes on bind() and connect().
 */
static RADIX_TREE(rpmsg_channels, GFP_KERNEL);

/* Synchronization of access to the tree is achieved using a mutex,
 * because we're using non-atomic radix tree allocations.
 */
static DEFINE_MUTEX(rpmsg_channels_lock);

static int rpmsg_sock_cb(struct rpmsg_device *rpdev, void *data, int len,
			 void *priv, u32 src);

static struct proto rpmsg_proto = {
	.name		= "RPMSG",
	.owner		= THIS_MODULE,
	.obj_size	= sizeof(struct rpmsg_socket),
};

/* Retrieve the rproc instance so that it can be used for retrieving
 * the processor id associated with the rpmsg channel.
 */
static inline struct rproc *rpdev_to_rproc(struct rpmsg_device *rpdev)
{
	return rproc_get_by_child(&rpdev->dev);
}

/* Retrieve the rproc id. The rproc id _relies_ on aliases being defined
 * in the DT blob for each of the remoteproc devices, and is essentially
 * the alias id. These are assumed to match to be fixed for a particular
 * SoC, and this provides a means to have a fixed interface to identify
 * a remote processor.
 */
static int rpmsg_sock_get_proc_id(struct rpmsg_device *rpdev)
{
	struct rproc *rproc = rpdev_to_rproc(rpdev);
	int id;

	if (!rproc) {
		WARN_ON(1);
		return -EINVAL;
	}

	id = rproc_get_id(rproc);
	WARN_ON(id < 0);

	return id;
}

static int rpmsg_sock_connect(struct socket *sock, struct sockaddr *addr,
			      int alen, int flags)
{
	struct sock *sk = sock->sk;
	struct rpmsg_socket *rpsk;
	struct sockaddr_rpmsg *sa;
	int err = 0;
	struct rpmsg_device *rpdev;

	if (sk->sk_state != RPMSG_OPEN)
		return -EBADFD;

	if (sk->sk_type != SOCK_SEQPACKET)
		return -EINVAL;

	if (!addr || addr->sa_family != AF_RPMSG)
		return -EINVAL;

	if (alen < sizeof(*sa))
		return -EINVAL;

	sa = (struct sockaddr_rpmsg *)addr;

	lock_sock(sk);

	rpsk = container_of(sk, struct rpmsg_socket, sk);

	mutex_lock(&rpmsg_channels_lock);

	/* find the set of channels exposed by this remote processor */
	rpdev = radix_tree_lookup(&rpmsg_channels, sa->vproc_id);
	if (!rpdev) {
		err = -EINVAL;
		goto out;
	}

	rpsk->rproc_id = sa->vproc_id;
	rpsk->rpdev = rpdev;

	/* XXX take care of disconnection state too */
	sk->sk_state = RPMSG_CONNECTED;

out:
	mutex_unlock(&rpmsg_channels_lock);
	release_sock(sk);
	return err;
}

static int rpmsg_sock_sendmsg(struct socket *sock, struct msghdr *msg,
			      size_t len)
{
	struct sock *sk = sock->sk;
	struct rpmsg_socket *rpsk;
	ssize_t max_payload;
	char payload[RPMSG_BUF_SIZE];/* todo: sane payload length methodology */
	int err;

	/* XXX check for sock_error as well ? */
	/* XXX handle noblock ? */
	if (msg->msg_flags & MSG_OOB)
		return -EOPNOTSUPP;

	/* no payload ? */
	if (!msg->msg_iter.iov->iov_base)
		return -EINVAL;

	lock_sock(sk);

	/* we don't support loopback at this point */
	if (sk->sk_state != RPMSG_CONNECTED) {
		err = -ENOTCONN;
		goto out;
	}

	rpsk = container_of(sk, struct rpmsg_socket, sk);

	max_payload = rpmsg_get_mtu(rpsk->rpdev->ept);
	if (max_payload < 0) {
		err = -EINVAL;
		goto out;
	}

	/* make sure the length is valid for copying into kernel buffer */
	if (len > max_payload) {
		err = -EMSGSIZE;
		goto out;
	}

	/* XXX for now, ignore the peer address. later use it
	 * with rpmsg_sendto, but only if user is root
	 */
	err = memcpy_from_msg(payload, msg, len);
	if (err)
		goto out;

	err = rpmsg_send(rpsk->rpdev->ept, payload, len);
	if (err)
		pr_err("rpmsg_send failed: %d\n", err);
	else
		err = len;

out:
	release_sock(sk);
	return err;
}

static int rpmsg_sock_recvmsg(struct socket *sock, struct msghdr *msg,
			      size_t len, int flags)
{
	struct sock *sk = sock->sk;
	struct sockaddr_rpmsg *sa;
	struct sk_buff *skb;
	int noblock = flags & MSG_DONTWAIT;
	int ret;

	if (flags & MSG_OOB) {
		pr_err("MSG_OOB: %d\n", EOPNOTSUPP);
		return -EOPNOTSUPP;
	}

	msg->msg_namelen = 0;

	skb = skb_recv_datagram(sk, flags, noblock, &ret);
	if (!skb) {
		/* check for shutdown ? */
		pr_err("skb_recv_datagram: %d\n", ret);
		return ret;
	}

	if (msg->msg_name) {
		msg->msg_namelen = sizeof(*sa);
		sa = (struct sockaddr_rpmsg *)msg->msg_name;
		sa->vproc_id = RPMSG_CB(skb).vproc_id;
		sa->addr = RPMSG_CB(skb).addr;
		sa->family = AF_RPMSG;
	}

	if (len > skb->len) {
		len = skb->len;
	} else if (len < skb->len) {
		pr_warn("user buffer is too small\n");
		/* XXX truncate or error ? */
		msg->msg_flags |= MSG_TRUNC;
	}

	ret = skb_copy_datagram_msg(skb, 0, msg, len);
	if (ret) {
		pr_err("error copying skb data: %d\n", ret);
		goto out_free;
	}

	ret = len;

out_free:
	skb_free_datagram(sk, skb);
	return ret;
}

static __poll_t rpmsg_sock_poll(struct file *file, struct socket *sock,
				poll_table *wait)
{
	struct sock *sk = sock->sk;
	__poll_t mask = 0;

	poll_wait(file, sk_sleep(sk), wait);

	/* exceptional events? */
	if (sk->sk_err || !skb_queue_empty(&sk->sk_error_queue))
		mask |= EPOLLERR;
	if (sk->sk_shutdown & RCV_SHUTDOWN)
		mask |= EPOLLRDHUP;
	if (sk->sk_shutdown == SHUTDOWN_MASK)
		mask |= EPOLLHUP;

	/* readable? */
	if (!skb_queue_empty(&sk->sk_receive_queue) ||
	    (sk->sk_shutdown & RCV_SHUTDOWN))
		mask |= EPOLLIN | EPOLLRDNORM;

	if (sk->sk_state == RPMSG_CLOSED)
		mask |= EPOLLHUP;

	/* XXX is writable ?
	 * this depends on the destination processor.
	 * if loopback: we're writable unless no memory
	 * if to remote: we need enabled rpmsg buffer or user supplied bufs
	 * for now, let's always be writable.
	 */
	mask |= EPOLLOUT | EPOLLWRNORM | EPOLLWRBAND;

	return mask;
}

/* return bound socket address information, either local or remote */
static int rpmsg_sock_getname(struct socket *sock, struct sockaddr *addr,
			      int peer)
{
	struct sock *sk = sock->sk;
	struct rpmsg_socket *rpsk;
	struct rpmsg_device *rpdev;
	struct sockaddr_rpmsg *sa;
	int ret;

	rpsk = container_of(sk, struct rpmsg_socket, sk);

	lock_sock(sk);
	rpdev = rpsk->rpdev;
	if (!rpdev) {
		ret = peer ? -ENOTCONN : -EINVAL;
		goto out;
	}

	addr->sa_family = AF_RPMSG;
	sa = (struct sockaddr_rpmsg *)addr;
	ret = sizeof(*sa);

	if (peer) {
		sa->vproc_id = rpsk->rproc_id;
		sa->addr = rpdev->dst;
	} else {
		sa->vproc_id = RPMSG_LOCALHOST;
		sa->addr = rpsk->endpt ? rpsk->endpt->addr : rpsk->rpdev->src;
	}

out:
	release_sock(sk);
	return ret;
}

static int rpmsg_sock_release(struct socket *sock)
{
	struct sock *sk = sock->sk;
	struct rpmsg_socket *rpsk = container_of(sk, struct rpmsg_socket, sk);

	if (!sk)
		return 0;

	/* function can be called with NULL endpoints, so it is effective for
	 * Rx sockets and a no-op for Tx sockets
	 */
	rpmsg_destroy_ept(rpsk->endpt);

	sock_put(sock->sk);

	return 0;
}

/* Notes:
 * - calling connect after bind isn't currently supported (is it even needed?).
 * - userspace arguments to bind aren't intuitive: one needs to provide
 *   the vproc id of the remote processor that the channel needs to be shared
 *   with, and the -local- source address the channel is to be bound with
 */
static int
rpmsg_sock_bind(struct socket *sock, struct sockaddr *uaddr, int addr_len)
{
	struct sock *sk = sock->sk;
	struct rpmsg_socket *rpsk = container_of(sk, struct rpmsg_socket, sk);
	struct rpmsg_device *rpdev;
	struct rpmsg_endpoint *endpt;
	struct rpmsg_channel_info chinfo = {};
	struct sockaddr_rpmsg *sa = (struct sockaddr_rpmsg *)uaddr;

	if (sock->state == SS_CONNECTED)
		return -EINVAL;

	if (addr_len != sizeof(*sa))
		return -EINVAL;

	if (sa->family != AF_RPMSG)
		return -EINVAL;

	if (rpsk->endpt)
		return -EBUSY;

	if (sk->sk_state != RPMSG_OPEN)
		return -EINVAL;

	rpdev = radix_tree_lookup(&rpmsg_channels, sa->vproc_id);
	if (!rpdev)
		return -EINVAL;

	/* bind this socket with a receiving endpoint */
	chinfo.src = sa->addr;
	chinfo.dst = RPMSG_ADDR_ANY;
	endpt = rpmsg_create_ept(rpdev, rpmsg_sock_cb, sk, chinfo);
	if (!endpt)
		return -EINVAL;

	rpsk->rpdev = rpdev;
	rpsk->endpt = endpt;
	rpsk->rproc_id = sa->vproc_id;

	sk->sk_state = RPMSG_LISTENING;

	return 0;
}

static const struct proto_ops rpmsg_sock_ops = {
	.family		= PF_RPMSG,
	.owner		= THIS_MODULE,

	.release	= rpmsg_sock_release,
	.connect	= rpmsg_sock_connect,
	.getname	= rpmsg_sock_getname,
	.sendmsg	= rpmsg_sock_sendmsg,
	.recvmsg	= rpmsg_sock_recvmsg,
	.poll		= rpmsg_sock_poll,
	.bind		= rpmsg_sock_bind,

	.listen		= sock_no_listen,
	.accept		= sock_no_accept,
	.ioctl		= sock_no_ioctl,
	.mmap		= sock_no_mmap,
	.socketpair	= sock_no_socketpair,
	.shutdown	= sock_no_shutdown,
	.setsockopt	= sock_no_setsockopt,
	.getsockopt	= sock_no_getsockopt
};

static void rpmsg_sock_destruct(struct sock *sk)
{
}

static int rpmsg_sock_create(struct net *net, struct socket *sock, int proto,
			     int kern)
{
	struct sock *sk;
	struct rpmsg_socket *rpsk;

	if (sock->type != SOCK_SEQPACKET)
		return -ESOCKTNOSUPPORT;
	if (proto != 0)
		return -EPROTONOSUPPORT;

	sk = sk_alloc(net, PF_RPMSG, GFP_KERNEL, &rpmsg_proto, kern);
	if (!sk)
		return -ENOMEM;

	sock->state = SS_UNCONNECTED;
	sock->ops = &rpmsg_sock_ops;
	sock_init_data(sock, sk);

	sk->sk_destruct = rpmsg_sock_destruct;
	sk->sk_protocol = proto;

	sk->sk_state = RPMSG_OPEN;

	rpsk = container_of(sk, struct rpmsg_socket, sk);
	/* use RPMSG_LOCALHOST to serve as an invalid value */
	rpsk->rproc_id = RPMSG_LOCALHOST;

	return 0;
}

static const struct net_proto_family rpmsg_proto_family = {
	.family = PF_RPMSG,
	.create	= rpmsg_sock_create,
	.owner = THIS_MODULE,
};

static int __rpmsg_sock_cb(struct device *dev, int from_vproc_id, void *data,
			   int len, struct sock *sk, u32 src)
{
	struct rpmsg_socket *rpsk = container_of(sk, struct rpmsg_socket, sk);
	struct sk_buff *skb;
	int ret;

#if defined(CONFIG_DYNAMIC_DEBUG)
	dynamic_hex_dump("rpmsg_proto Rx: ", DUMP_PREFIX_NONE, 16, 1, data,
			 len, true);
#endif

	lock_sock(sk);

	switch (sk->sk_state) {
	case RPMSG_CONNECTED:
		if (rpsk->rpdev->dst != src)
			dev_warn(dev, "unexpected source address: %d\n", src);
		break;
	case RPMSG_LISTENING:
		/* When an inbound message is received while we're listening,
		 * we implicitly become connected
		 */
		sk->sk_state = RPMSG_CONNECTED;
		rpsk->rpdev->dst = src;
		break;
	default:
		dev_warn(dev, "unexpected inbound message (from %d)\n", src);
		break;
	}

	skb = sock_alloc_send_skb(sk, len, 1, &ret);
	if (!skb) {
		dev_err(dev, "sock_alloc_send_skb failed: %d\n", ret);
		ret = -ENOMEM;
		goto out;
	}

	RPMSG_CB(skb).vproc_id = from_vproc_id;
	RPMSG_CB(skb).addr = src;
	RPMSG_CB(skb).family = AF_RPMSG;

	memcpy(skb_put(skb, len), data, len);

	ret = sock_queue_rcv_skb(sk, skb);
	if (ret) {
		dev_err(dev, "sock_queue_rcv_skb failed: %d\n", ret);
		kfree_skb(skb);
	}

out:
	release_sock(sk);
	return ret;
}

static int rpmsg_sock_cb(struct rpmsg_device *rpdev, void *data, int len,
			 void *priv, u32 src)
{
	int id = rpmsg_sock_get_proc_id(rpdev);

	return __rpmsg_sock_cb(&rpdev->dev, id, data, len, priv, src);
}

static int rpmsg_proto_cb(struct rpmsg_device *rpdev, void *data, int len,
			  void *priv, u32 src)
{
	dev_err(&rpdev->dev, "rpmsg_proto device not designed to receive any messages\n");
	return 0;
}

static int rpmsg_proto_probe(struct rpmsg_device *rpdev)
{
	struct device *dev = &rpdev->dev;
	int ret, dst = rpdev->dst, id;
	struct rpmsg_device *vrp_dev;

	if (WARN_ON(dst == RPMSG_ADDR_ANY))
		return -EINVAL;

	id = rpmsg_sock_get_proc_id(rpdev);
	if (id < 0)
		return -EINVAL;

	mutex_lock(&rpmsg_channels_lock);

	/* are we exposing a rpmsg proto device for this remote processor yet?
	 * If not, associate id/device for later lookup in rpmsg_sock_bind().
	 * Multiple devices per remote processor are not supported.
	 */
	vrp_dev = radix_tree_lookup(&rpmsg_channels, id);
	if (!vrp_dev) {
		ret = radix_tree_insert(&rpmsg_channels, id, rpdev);
		if (ret) {
			dev_err(dev, "radix_tree_insert failed: %d\n", ret);
			goto out;
		}
	} else {
		ret = -ENODEV;
		dev_err(dev, "multiple rpmsg-proto devices from the same rproc is not supported.\n");
		goto out;
	}

out:
	mutex_unlock(&rpmsg_channels_lock);

	return ret;
}

static void rpmsg_proto_remove(struct rpmsg_device *rpdev)
{
	struct device *dev = &rpdev->dev;
	int id, dst = rpdev->dst;
	struct rpmsg_device *vrp_dev;

	if (dst == RPMSG_ADDR_ANY)
		return;

	id = rpmsg_sock_get_proc_id(rpdev);

	mutex_lock(&rpmsg_channels_lock);

	vrp_dev = radix_tree_lookup(&rpmsg_channels, id);
	if (!vrp_dev) {
		dev_err(dev, "can't find rpmsg device for rproc %d\n", id);
		goto out;
	}
	if (vrp_dev != rpdev)
		dev_err(dev, "can't match the stored rpdev for rproc %d\n", id);

	if (!radix_tree_delete(&rpmsg_channels, id))
		dev_err(dev, "failed to delete rpdev for rproc %d\n", id);

out:
	mutex_unlock(&rpmsg_channels_lock);
}

static struct rpmsg_device_id rpmsg_proto_id_table[] = {
	{ .name	= "rpmsg-proto" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, rpmsg_proto_id_table);

static struct rpmsg_driver rpmsg_proto_driver = {
	.drv.name	= KBUILD_MODNAME,
	.id_table	= rpmsg_proto_id_table,
	.probe		= rpmsg_proto_probe,
	.callback	= rpmsg_proto_cb,
	.remove		= rpmsg_proto_remove,
};

static int __init rpmsg_proto_init(void)
{
	int ret;

	ret = proto_register(&rpmsg_proto, 0);
	if (ret) {
		pr_err("proto_register failed: %d\n", ret);
		return ret;
	}

	ret = sock_register(&rpmsg_proto_family);
	if (ret) {
		pr_err("sock_register failed: %d\n", ret);
		goto proto_unreg;
	}

	ret = register_rpmsg_driver(&rpmsg_proto_driver);
	if (ret) {
		pr_err("register_rpmsg_driver failed: %d\n", ret);
		goto sock_unreg;
	}

	return 0;

sock_unreg:
	sock_unregister(PF_RPMSG);
proto_unreg:
	proto_unregister(&rpmsg_proto);
	return ret;
}

static void __exit rpmsg_proto_exit(void)
{
	unregister_rpmsg_driver(&rpmsg_proto_driver);
	sock_unregister(PF_RPMSG);
	proto_unregister(&rpmsg_proto);
}

module_init(rpmsg_proto_init);
module_exit(rpmsg_proto_exit);

MODULE_DESCRIPTION("Remote processor messaging protocol");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("rpmsg:rpmsg-proto");
MODULE_ALIAS_NETPROTO(AF_RPMSG);
