/*
 * AF_RPMSG: Remote processor messaging sockets
 *
 * Copyright (C) 2011-2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Ohad Ben-Cohen <ohad@wizery.com>
 * Robert Tivy <rtivy@ti.com>
 * G Anthony <a0783926@ti.com>
 * Suman Anna <s-anna@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
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

/* Used to distinguish between bound and connected socket channels in the
 * radix tree index space.
 * Must match value as in drivers/rpmsg/virtio_rpmsg_bus.c:
 */
#define RPMSG_RESERVED_ADDRESSES     (1024)

/* Maximum buffer size supported by virtio rpmsg transport.
 * Must match value as in drivers/rpmsg/virtio_rpmsg_bus.c
 */
#define RPMSG_BUF_SIZE               (512)

struct rpmsg_socket {
	struct sock sk;
	struct rpmsg_channel *rpdev;
	int rproc_id;
	struct list_head elem;
	bool unregister_rpdev;
};

/* Connection and socket states */
enum {
	RPMSG_CONNECTED = 1,
	RPMSG_OPEN,
	RPMSG_LISTENING,
	RPMSG_CLOSED,
	RPMSG_ERROR,
};

/* A two-level radix-tree-based scheme is used to maintain the rpmsg channels
 * we're exposing to userland. The first radix tree maps vproc index id
 * to its channels, and the second radix tree associates each channel
 * with its destination addresses (so sockaddr_rpmsg lookups are quick).
 *
 * Currently only channels with a valid dst address are supported (aka 'client'
 * channels as opposed to 'server' channels which usually only have a valid
 * src address).
 */
static RADIX_TREE(rpmsg_channels, GFP_KERNEL);

/* Synchronization of access to the tree is achieved using a mutex,
 * because we're using non-atomic radix tree allocations.
 */
static DEFINE_MUTEX(rpmsg_channels_lock);

/* A radix tree is used to retrieve the virtproc_info structure
 * from the associated system-wide unique processor id.
 */
static RADIX_TREE(rpmsg_vprocs, GFP_KERNEL);
static DEFINE_MUTEX(rpmsg_vprocs_lock);

static struct proto rpmsg_proto = {
	.name		= "RPMSG",
	.owner		= THIS_MODULE,
	.obj_size = sizeof(struct rpmsg_socket),
};

/* Retrieve the rproc instance so that it can be used for retrieving
 * the processor id associated with the rpmsg channel.
 */
static struct rproc *rpdev_to_rproc(struct rpmsg_channel *rpdev)
{
	struct virtio_device *vdev;

	vdev = rpmsg_get_virtio_dev(rpdev);
	if (!vdev)
		return NULL;

	return rproc_vdev_to_rproc_safe(vdev);
}

/* Retrieve the rproc id. The rproc id _relies_ on aliases being defined
 * in the DT blob for each of the remoteproc devices, and is essentially
 * the alias id. These are assumed to match to be fixed for a particular
 * SoC, and this provides a means to have a fixed interface to identify
 * a remote processor.
 */
static int rpmsg_sock_get_proc_id(struct rpmsg_channel *rpdev)
{
	struct rproc *rproc = rpdev_to_rproc(rpdev);
	int id;

	if (!rproc) {
		WARN_ON(1);
		return -EINVAL;
	}

	id = rproc_get_alias_id(rproc);
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
	struct radix_tree_root *vrp_channels;
	struct rpmsg_channel *rpdev;

	if (sk->sk_state != RPMSG_OPEN)
		return -EBADFD;

	if (sk->sk_type != SOCK_SEQPACKET)
		return -EINVAL;

	if (!addr || addr->sa_family != AF_RPMSG)
		return -EINVAL;

	if (alen < sizeof(*sa))
		return -EINVAL;

	sa = (struct sockaddr_rpmsg *)addr;

	mutex_lock(&rpmsg_channels_lock);

	lock_sock(sk);

	rpsk = container_of(sk, struct rpmsg_socket, sk);

	/* find the set of channels exposed by this remote processor */
	vrp_channels = radix_tree_lookup(&rpmsg_channels, sa->vproc_id);
	if (!vrp_channels) {
		err = -EINVAL;
		goto out;
	}

	/* find the specific channel we need to connect with, by dst addr:  */
	rpdev = radix_tree_lookup(vrp_channels, sa->addr);
	if (!rpdev) {
		err = -EINVAL;
		goto out;
	}

	rpsk->rproc_id = sa->vproc_id;
	rpsk->rpdev = rpdev;

	/* bind this socket with its rpmsg endpoint */
	list_add_tail(&rpsk->elem, rpdev->ept->priv);

	/* XXX take care of disconnection state too */
	sk->sk_state = RPMSG_CONNECTED;

out:
	release_sock(sk);
	mutex_unlock(&rpmsg_channels_lock);
	return err;
}

static int rpmsg_sock_sendmsg(struct socket *sock, struct msghdr *msg,
			      size_t len)
{
	struct sock *sk = sock->sk;
	struct rpmsg_socket *rpsk;
	char payload[RPMSG_BUF_SIZE];/* todo: sane payload length methodology */
	int err;

	/* XXX check for sock_error as well ? */
	/* XXX handle noblock ? */
	if (msg->msg_flags & MSG_OOB)
		return -EOPNOTSUPP;

	/* no payload ? */
	if (!msg->msg_iter.iov->iov_base)
		return -EINVAL;

	/* make sure the length is valid for copying into kernel buffer */
	if (len > RPMSG_BUF_SIZE - sizeof(struct rpmsg_hdr))
		return -EMSGSIZE;

	lock_sock(sk);

	/* we don't support Tx on errored-out sockets */
	if (sk->sk_state == RPMSG_ERROR) {
		release_sock(sk);
		return -ESHUTDOWN;
	}

	/* we don't support loopback at this point */
	if (sk->sk_state != RPMSG_CONNECTED) {
		release_sock(sk);
		return -ENOTCONN;
	}

	rpsk = container_of(sk, struct rpmsg_socket, sk);

	/* XXX for now, ignore the peer address. later use it
	 * with rpmsg_sendto, but only if user is root
	 */
	err = memcpy_from_msg(payload, msg, len);
	if (err)
		goto out;

	err = rpmsg_send(rpsk->rpdev, payload, len);
	if (err)
		pr_err("rpmsg_send failed: %d\n", err);

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

	/* return failure on errored-out Rx sockets */
	lock_sock(sk);
	if (sk->sk_state == RPMSG_ERROR) {
		release_sock(sk);
		return -ENOLINK;
	}
	release_sock(sk);

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

static unsigned int rpmsg_sock_poll(struct file *file, struct socket *sock,
				    poll_table *wait)
{
	struct sock *sk = sock->sk;
	unsigned int mask = 0;

	poll_wait(file, sk_sleep(sk), wait);

	/* exceptional events? */
	if (sk->sk_err || !skb_queue_empty(&sk->sk_error_queue))
		mask |= POLLERR;
	if (sk->sk_state == RPMSG_ERROR)
		mask |= POLLERR;
	if (sk->sk_shutdown & RCV_SHUTDOWN)
		mask |= POLLRDHUP;
	if (sk->sk_shutdown == SHUTDOWN_MASK)
		mask |= POLLHUP;

	/* readable? */
	if (!skb_queue_empty(&sk->sk_receive_queue) ||
	    (sk->sk_shutdown & RCV_SHUTDOWN))
		mask |= POLLIN | POLLRDNORM;

	if (sk->sk_state == RPMSG_CLOSED)
		mask |= POLLHUP;

	/* XXX is writable ?
	 * this depends on the destination processor.
	 * if loopback: we're writable unless no memory
	 * if to remote: we need enabled rpmsg buffer or user supplied bufs
	 * for now, let's always be writable.
	 */
	mask |= POLLOUT | POLLWRNORM | POLLWRBAND;

	return mask;
}

/* return bound socket address information, either local or remote
 * note: len is just an output parameter, doesn't carry any input value
 */
static int rpmsg_sock_getname(struct socket *sock, struct sockaddr *addr,
			      int *len, int peer)
{
	struct sock *sk = sock->sk;
	struct rpmsg_socket *rpsk;
	struct rpmsg_channel *rpdev;
	struct sockaddr_rpmsg *sa;

	rpsk = container_of(sk, struct rpmsg_socket, sk);
	rpdev = rpsk->rpdev;

	if (!rpdev)
		return -ENOTCONN;

	addr->sa_family = AF_RPMSG;

	sa = (struct sockaddr_rpmsg *)addr;

	*len = sizeof(*sa);

	if (peer) {
		sa->vproc_id = rpsk->rproc_id;
		sa->addr = rpdev->dst;
	} else {
		sa->vproc_id = RPMSG_LOCALHOST;
		sa->addr = rpsk->rpdev->src;
	}

	return 0;
}

static int rpmsg_sock_release(struct socket *sock)
{
	struct sock *sk = sock->sk;
	struct rpmsg_socket *rpsk = container_of(sk, struct rpmsg_socket, sk);
	struct virtproc_info *vrp = NULL;
	int ret;

	if (!sk)
		return 0;

	mutex_lock(&rpmsg_channels_lock);
	if (rpsk->unregister_rpdev) { /* Rx (bound) sockets */
		/* The bound socket's rpmsg device will be removed by rpmsg bus
		 * core during recovery, but only after the published rpmsg
		 * channel is removed (device registration order). The check for
		 * valid vrp will ensure that rpmsg_destroy_channel will not be
		 * called if the release from userspace occurs first. However,
		 * the socket can be released much later than the recreated vrp
		 * as well, so an additional check for a sane socket state is
		 * also needed.
		 */
		vrp = radix_tree_lookup(&rpmsg_vprocs, rpsk->rproc_id);
		if (vrp && sk->sk_state != RPMSG_ERROR) {
			rpsk->rpdev->ept->priv = NULL;
			mutex_unlock(&rpmsg_channels_lock);
			ret = rpmsg_destroy_channel(rpsk->rpdev);
			if (ret) {
				pr_err("rpmsg_destroy_channel failed for sk %p\n",
				       sk);
			}
			goto release;
		}
	} else { /* Tx (connected) sockets */
		if (sk->sk_state != RPMSG_ERROR)
			list_del(&rpsk->elem);
	}
	mutex_unlock(&rpmsg_channels_lock);

release:
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
	struct rpmsg_channel *rpdev;
	struct sockaddr_rpmsg *sa = (struct sockaddr_rpmsg *)uaddr;
	struct virtproc_info *vrp;

	if (sock->state == SS_CONNECTED)
		return -EINVAL;

	if (addr_len != sizeof(*sa))
		return -EINVAL;

	if (sa->family != AF_RPMSG)
		return -EINVAL;

	/* do not allow fixed addresses above the dynamically allocated range */
	if (sa->addr >= RPMSG_RESERVED_ADDRESSES)
		return -EINVAL;

	if (rpsk->rpdev)
		return -EBUSY;

	if (sk->sk_state != RPMSG_OPEN)
		return -EINVAL;

	vrp = radix_tree_lookup(&rpmsg_vprocs, sa->vproc_id);
	if (!vrp)
		return -EINVAL;

	rpdev = rpmsg_create_channel(vrp, "rpmsg-proto", "", sa->addr,
				     RPMSG_ADDR_ANY);
	if (!rpdev)
		return -EINVAL;

	if (!rpdev->ept) {
		rpmsg_destroy_channel(rpdev);
		return -EINVAL;
	}

	rpsk->rpdev = rpdev;
	rpsk->unregister_rpdev = true;
	rpsk->rproc_id = sa->vproc_id;

	/* bind this socket with its rpmsg endpoint */
	rpdev->ept->priv = sk;

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
	INIT_LIST_HEAD(&rpsk->elem);
	/* use RPMSG_LOCALHOST to serve as an invalid value */
	rpsk->rproc_id = RPMSG_LOCALHOST;

	return 0;
}

static const struct net_proto_family rpmsg_proto_family = {
	.family = PF_RPMSG,
	.create	= rpmsg_sock_create,
	.owner = THIS_MODULE,
};

static void __rpmsg_proto_cb(struct device *dev, int from_vproc_id, void *data,
			     int len, struct sock *sk, u32 src)
{
	struct rpmsg_socket *rpsk = container_of(sk, struct rpmsg_socket, sk);
	struct sk_buff *skb;
	int ret;

#if defined(CONFIG_DYNAMIC_DEBUG)
	dynamic_hex_dump("rpmsg_proto Rx: ", DUMP_PREFIX_NONE, 16, 1, data,
			 len, true);
#endif

	if (!sk) {
		dev_warn(dev, "callback received for deleted socket (from %d)\n",
			 src);
		return;
	}

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
}

static void rpmsg_proto_cb(struct rpmsg_channel *rpdev, void *data, int len,
			   void *priv, u32 src)
{
	int id = rpmsg_sock_get_proc_id(rpdev);

	/* published rpmsg channels from remote side reuse their end-point's
	 * private field for storing the list of connected sockets, so cannot
	 * process messages.
	 */
	if (rpdev->src >= RPMSG_RESERVED_ADDRESSES) {
		dev_err(&rpdev->dev, "rpmsg_proto device not designed to receive any messages\n");
		return;
	}

	__rpmsg_proto_cb(&rpdev->dev, id, data, len, priv, src);
}

/* every channel we're probed with is exposed to userland via the Socket API */
static int rpmsg_proto_probe(struct rpmsg_channel *rpdev)
{
	struct device *dev = &rpdev->dev;
	int ret, dst = rpdev->dst, id;
	struct radix_tree_root *vrp_channels;
	struct virtproc_info *vrp;
	struct list_head *sock_list = NULL;

	id = rpmsg_sock_get_proc_id(rpdev);

	vrp = radix_tree_lookup(&rpmsg_vprocs, id);
	if (vrp && vrp != rpdev->vrp)
		dev_err(dev, "id %d already associated to different vrp\n",
			id);

	if (dst == RPMSG_ADDR_ANY) {
		/* do not announce bound sockets to remote processor */
		rpdev->announce = false;
		return 0;
	}

	/* associate id/vrp for later lookup in rpmsg_sock_bind() */
	if (!vrp) {
		mutex_lock(&rpmsg_vprocs_lock);
		ret = radix_tree_insert(&rpmsg_vprocs, (unsigned long)id,
					rpdev->vrp);
		mutex_unlock(&rpmsg_vprocs_lock);
		if (ret) {
			dev_err(dev, "radix_tree_insert(%d) failed: %d\n",
				id, ret);
			return ret;
		}
	}

	mutex_lock(&rpmsg_channels_lock);

	/* are we exposing channels for this remote processor yet ? */
	vrp_channels = radix_tree_lookup(&rpmsg_channels, id);
	/* not yet ? let's prepare the 2nd radix tree level then */
	if (!vrp_channels) {
		vrp_channels = kzalloc(sizeof(*vrp_channels), GFP_KERNEL);
		INIT_RADIX_TREE(vrp_channels, GFP_KERNEL);
		/* now let's associate the new channel with its vrp */
		ret = radix_tree_insert(&rpmsg_channels, id, vrp_channels);
		if (ret) {
			dev_err(dev, "radix_tree_insert failed: %d\n", ret);
			kfree(vrp_channels);
			goto out;
		}
	} else {
		ret = -ENODEV;
		dev_err(dev, "multiple rpmsg-proto devices from the same rproc is not supported.\n");
		goto out;
	}

	WARN_ON(!!rpdev->ept->priv);
	sock_list = kzalloc(sizeof(*sock_list), GFP_KERNEL);
	if (!sock_list) {
		dev_err(dev, "failed to allocate list_head\n");
		ret = -ENOMEM;
		goto out;
	}
	INIT_LIST_HEAD(sock_list);

	/* let's associate the new channel with its dst */
	ret = radix_tree_insert(vrp_channels, dst, rpdev);
	if (ret) {
		dev_err(dev, "failed to add rpmsg addr %d: %d\n", dst, ret);
		kfree(sock_list);
		goto out;
	}
	rpdev->ept->priv = sock_list;

out:
	mutex_unlock(&rpmsg_channels_lock);

	return ret;
}

static void rpmsg_proto_remove(struct rpmsg_channel *rpdev)
{
	struct device *dev = &rpdev->dev;
	int id, dst = rpdev->dst, src = rpdev->src;
	struct radix_tree_root *vrp_channels;
	struct list_head *sk_list;
	struct rpmsg_socket *rpsk, *tmp;
	struct sock *sk;

	id = rpmsg_sock_get_proc_id(rpdev);

	mutex_lock(&rpmsg_channels_lock);

	/* Only remove non-reserved channels from the radix trees, as only these
	 * were "probed" (published from remote processor and added to radix
	 * trees).  Note: bind is not causing a "true" probe, and bound
	 * sockets have src addresses < RPMSG_RESERVED_ADDRESSES.
	 */
	if (src >= RPMSG_RESERVED_ADDRESSES) {
		vrp_channels = radix_tree_lookup(&rpmsg_channels, id);
		if (!vrp_channels) {
			dev_err(dev, "can't find channels for this vrp: %d\n",
				id);
			goto out;
		}

		mutex_lock(&rpmsg_vprocs_lock);
		if (!radix_tree_delete(&rpmsg_vprocs, id))
			dev_err(dev, "failed to delete id %d\n", id);
		mutex_unlock(&rpmsg_vprocs_lock);

		/* mark all connected sockets invalid and remove them
		 * from the rpdev's list.
		 */
		sk_list = rpdev->ept->priv;
		list_for_each_entry_safe(rpsk, tmp, sk_list, elem) {
			rpsk->sk.sk_state = RPMSG_ERROR;
			list_del(&rpsk->elem);
		}
		kfree(sk_list);
		rpdev->ept->priv = NULL;

		if (!radix_tree_delete(vrp_channels, dst))
			dev_err(dev, "failed to delete rpmsg %d\n", dst);

		if (!radix_tree_delete(&rpmsg_channels, id))
			dev_err(dev, "failed to delete vrp_channels for id %d\n",
				id);
		kfree(vrp_channels);
	} else {
		/* mark the associated bound socket as invalid if it has not
		 * already been deleted by rpmsg_sock_release().
		 */
		sk = rpdev->ept->priv;
		if (sk) {
			lock_sock(sk);
			sk->sk_state = RPMSG_ERROR;
			rpsk = container_of(sk, struct rpmsg_socket, sk);
			rpsk->rpdev = NULL;
			sk->sk_error_report(sk);
			release_sock(sk);
		}
	}

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
