// SPDX-License-Identifier: GPL-2.0-only
/*
 * Remote Processor Procedure Call Driver
 *
 * Copyright (C) 2012-2020 Texas Instruments Incorporated - http://www.ti.com/
 *	Erik Rainey <erik.rainey@ti.com>
 *	Suman Anna <s-anna@ti.com>
 */

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/idr.h>
#include <linux/poll.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/fdtable.h>
#include <linux/remoteproc.h>
#include <linux/rpmsg.h>
#include <linux/rpmsg_rpc.h>
#include <linux/sched/signal.h>

#include "rpmsg_rpc_internal.h"

#define RPPC_MAX_DEVICES	(8)
#define RPPC_MAX_REG_FDS	(10)

#define RPPC_SIG_NUM_PARAM(sig) ((sig).num_param - 1)

/* TODO: remove these fields */
#define RPPC_JOBID_DISCRETE	(0)
#define RPPC_POOLID_DEFAULT	(0x8000)

static struct class *rppc_class;
static dev_t rppc_dev;

/* store all remote rpc connection services (usually one per remoteproc) */
static DEFINE_IDR(rppc_devices);
static DEFINE_MUTEX(rppc_devices_lock);

/*
 * Retrieve the rproc instance so that it can be used for performing
 * address translations
 */
static inline struct rproc *rpdev_to_rproc(struct rpmsg_device *rpdev)
{
	return rproc_get_by_child(&rpdev->dev);
}

/*
 * A wrapper function to translate local physical addresses to the remote core
 * device addresses (virtual addresses that a code on remote processor can use
 * directly.
 *
 * XXX: Fix this to return negative values on errors to follow normal kernel
 *      conventions, and since 0 can also be a valid remote processor address
 *
 * Returns a remote processor device address on success, 0 otherwise
 */
dev_addr_t rppc_local_to_remote_da(struct rppc_instance *rpc, phys_addr_t pa)
{
	int ret;
	struct rproc *rproc;
	u64 da = 0;
	dev_addr_t rda;
	struct device *dev = rpc->dev;

	if (mutex_lock_interruptible(&rpc->rppcdev->lock))
		return 0;

	rproc = rpdev_to_rproc(rpc->rppcdev->rpdev);
	if (!rproc) {
		dev_err(dev, "error getting rproc for rpdev 0x%x\n",
			(u32)rpc->rppcdev->rpdev);
	} else {
		ret = rproc_pa_to_da(rproc, pa, &da);
		if (ret) {
			dev_err(dev, "error from rproc_pa_to_da, rproc = %p, pa = %pa ret = %d\n",
				rproc, &pa, ret);
		}
	}
	rda = (dev_addr_t)da;

	mutex_unlock(&rpc->rppcdev->lock);

	return rda;
}

static void rppc_print_msg(struct rppc_instance *rpc, char *prefix,
			   char buffer[512])
{
	struct rppc_msg_header *hdr = (struct rppc_msg_header *)buffer;
	struct rppc_instance_handle *hdl = NULL;
	struct rppc_query_function *info = NULL;
	struct rppc_packet *packet = NULL;
	struct rppc_param_data *param = NULL;
	struct device *dev = rpc->dev;
	u32 i = 0, paramsz = sizeof(*param);

	dev_dbg(dev, "%s HDR: msg_type = %d msg_len = %d\n",
		prefix, hdr->msg_type, hdr->msg_len);

	switch (hdr->msg_type) {
	case RPPC_MSGTYPE_CREATE_RESP:
	case RPPC_MSGTYPE_DELETE_RESP:
		hdl = RPPC_PAYLOAD(buffer, rppc_instance_handle);
		dev_dbg(dev, "%s endpoint = %d status = %d\n",
			prefix, hdl->endpoint_address, hdl->status);
		break;
	case RPPC_MSGTYPE_FUNCTION_INFO:
		info = RPPC_PAYLOAD(buffer, rppc_query_function);
		dev_dbg(dev, "%s (info not yet implemented)\n", prefix);
		break;
	case RPPC_MSGTYPE_FUNCTION_CALL:
		packet = RPPC_PAYLOAD(buffer, rppc_packet);
		dev_dbg(dev, "%s PACKET: desc = %04x msg_id = %04x flags = %08x func = 0x%08x result = %d size = %u\n",
			prefix, packet->desc, packet->msg_id,
			packet->flags, packet->fxn_id,
			packet->result, packet->data_size);
		param = (struct rppc_param_data *)packet->data;
		for (i = 0; i < (packet->data_size / paramsz); i++) {
			dev_dbg(dev, "%s param[%u] size = %zu data = %zu (0x%08x)",
				prefix, i, param[i].size, param[i].data,
				param[i].data);
		}
		break;
	default:
		break;
	}
}

/* free any outstanding function calls */
static void rppc_delete_fxns(struct rppc_instance *rpc)
{
	struct rppc_function_list *pos, *n;

	if (!list_empty(&rpc->fxn_list)) {
		mutex_lock(&rpc->lock);
		list_for_each_entry_safe(pos, n, &rpc->fxn_list, list) {
			list_del(&pos->list);
			kfree(pos->function);
			kfree(pos);
		}
		mutex_unlock(&rpc->lock);
	}
}

static
struct rppc_function *rppc_find_fxn(struct rppc_instance *rpc, u16 msg_id)
{
	struct rppc_function *function = NULL;
	struct rppc_function_list *pos, *n;
	struct device *dev = rpc->dev;

	mutex_lock(&rpc->lock);
	list_for_each_entry_safe(pos, n, &rpc->fxn_list, list) {
		dev_dbg(dev, "looking for msg %u, found msg %u\n",
			msg_id, pos->msg_id);
		if (pos->msg_id == msg_id) {
			function = pos->function;
			list_del(&pos->list);
			kfree(pos);
			break;
		}
	}
	mutex_unlock(&rpc->lock);

	return function;
}

static int rppc_add_fxn(struct rppc_instance *rpc,
			struct rppc_function *function, u16 msg_id)
{
	struct rppc_function_list *fxn = NULL;
	struct device *dev = rpc->dev;

	fxn = kzalloc(sizeof(*fxn), GFP_KERNEL);
	if (!fxn)
		return -ENOMEM;

	fxn->function = function;
	fxn->msg_id = msg_id;
	mutex_lock(&rpc->lock);
	list_add(&fxn->list, &rpc->fxn_list);
	mutex_unlock(&rpc->lock);
	dev_dbg(dev, "added msg id %u to list", msg_id);

	return 0;
}

static
void rppc_handle_create_resp(struct rppc_instance *rpc, char *data, int len)
{
	struct device *dev = rpc->dev;
	struct rppc_msg_header *hdr = (struct rppc_msg_header *)data;
	struct rppc_instance_handle *hdl;
	u32 exp_len = sizeof(*hdl) + sizeof(*hdr);

	if (len != exp_len) {
		dev_err(dev, "invalid response message length %d (expected %d bytes)",
			len, exp_len);
		rpc->state = RPPC_STATE_STALE;
		return;
	}

	hdl = RPPC_PAYLOAD(data, rppc_instance_handle);

	mutex_lock(&rpc->lock);
	if (rpc->state != RPPC_STATE_STALE && hdl->status == 0) {
		rpc->dst = hdl->endpoint_address;
		rpc->state = RPPC_STATE_CONNECTED;
	} else {
		rpc->state = RPPC_STATE_STALE;
	}
	rpc->in_transition = 0;
	dev_dbg(dev, "creation response: status %d addr 0x%x\n",
		hdl->status, hdl->endpoint_address);

	complete(&rpc->reply_arrived);
	mutex_unlock(&rpc->lock);
}

static
void rppc_handle_delete_resp(struct rppc_instance *rpc, char *data, int len)
{
	struct device *dev = rpc->dev;
	struct rppc_msg_header *hdr = (struct rppc_msg_header *)data;
	struct rppc_instance_handle *hdl;
	u32 exp_len = sizeof(*hdl) + sizeof(*hdr);

	if (len != exp_len) {
		dev_err(dev, "invalid response message length %d (expected %d bytes)",
			len, exp_len);
		rpc->state = RPPC_STATE_STALE;
		return;
	}
	if (hdr->msg_len != sizeof(*hdl)) {
		dev_err(dev, "disconnect message was incorrect size!\n");
		rpc->state = RPPC_STATE_STALE;
		return;
	}

	hdl = RPPC_PAYLOAD(data, rppc_instance_handle);
	dev_dbg(dev, "deletion response: status %d addr 0x%x\n",
		hdl->status, hdl->endpoint_address);
	mutex_lock(&rpc->lock);
	rpc->dst = 0;
	rpc->state = RPPC_STATE_DISCONNECTED;
	rpc->in_transition = 0;
	complete(&rpc->reply_arrived);
	mutex_unlock(&rpc->lock);
}

/*
 * store the received message and wake up any blocking processes,
 * waiting for new data. The allocated buffer would be freed after
 * the user-space reads the packet.
 */
static void rppc_handle_fxn_resp(struct rppc_instance *rpc, char *data, int len)
{
	struct rppc_msg_header *hdr = (struct rppc_msg_header *)data;
	struct sk_buff *skb;
	char *skbdata;

	/* TODO: need to check the response length? */
	skb = alloc_skb(hdr->msg_len, GFP_KERNEL);
	if (!skb)
		return;
	skbdata = skb_put(skb, hdr->msg_len);
	memcpy(skbdata, hdr->msg_data, hdr->msg_len);

	mutex_lock(&rpc->lock);
	skb_queue_tail(&rpc->queue, skb);
	mutex_unlock(&rpc->lock);

	wake_up_interruptible(&rpc->readq);
}

/*
 * callback function for processing the different responses
 * from the remote processor on a particular rpmsg channel
 * instance.
 */
static int rppc_cb(struct rpmsg_device *rpdev,
		   void *data, int len, void *priv, u32 src)
{
	struct rppc_msg_header *hdr = data;
	struct rppc_instance *rpc = priv;
	struct device *dev = rpc->dev;
	char *buf = (char *)data;

	dev_dbg(dev, "<== incoming msg src %d len %d msg_type %d msg_len %d\n",
		src, len, hdr->msg_type, hdr->msg_len);
	rppc_print_msg(rpc, "RX:", buf);

	if (len <= sizeof(*hdr)) {
		dev_err(dev, "message truncated\n");
		rpc->state = RPPC_STATE_STALE;
		return -EINVAL;
	}

	switch (hdr->msg_type) {
	case RPPC_MSGTYPE_CREATE_RESP:
		rppc_handle_create_resp(rpc, data, len);
		break;
	case RPPC_MSGTYPE_DELETE_RESP:
		rppc_handle_delete_resp(rpc, data, len);
		break;
	case RPPC_MSGTYPE_FUNCTION_CALL:
	case RPPC_MSGTYPE_FUNCTION_RET:
		rppc_handle_fxn_resp(rpc, data, len);
		break;
	default:
		dev_warn(dev, "unexpected msg type: %d\n", hdr->msg_type);
		break;
	}

	return 0;
}

/*
 * send a connection request to the remote rpc connection service. Use
 * the new local address created during .open for this instance as the
 * source address to complete the connection.
 */
static int rppc_connect(struct rppc_instance *rpc,
			struct rppc_create_instance *connect)
{
	int ret = 0;
	u32 len = 0;
	char kbuf[512];
	struct rppc_device *rppcdev = rpc->rppcdev;
	struct rppc_msg_header *hdr = (struct rppc_msg_header *)&kbuf[0];

	if (rpc->state == RPPC_STATE_CONNECTED) {
		dev_dbg(rpc->dev, "endpoint already connected\n");
		return -EISCONN;
	}

	hdr->msg_type = RPPC_MSGTYPE_CREATE_REQ;
	hdr->msg_len = sizeof(*connect);
	memcpy(hdr->msg_data, connect, hdr->msg_len);
	len = sizeof(struct rppc_msg_header) + hdr->msg_len;

	init_completion(&rpc->reply_arrived);
	rpc->in_transition = 1;
	ret = rpmsg_send_offchannel(rppcdev->rpdev->ept, rpc->ept->addr,
				    rppcdev->rpdev->dst, (char *)kbuf, len);
	if (ret > 0) {
		dev_err(rpc->dev, "rpmsg_send failed: %d\n", ret);
		return ret;
	}

	ret = wait_for_completion_interruptible_timeout(&rpc->reply_arrived,
							msecs_to_jiffies(5000));
	if (rpc->state == RPPC_STATE_CONNECTED)
		return 0;

	if (rpc->state == RPPC_STATE_STALE)
		return -ENXIO;

	if (ret > 0) {
		dev_err(rpc->dev, "premature wakeup: %d\n", ret);
		return -EIO;
	}

	return -ETIMEDOUT;
}

static void rppc_disconnect(struct rppc_instance *rpc)
{
	int ret;
	size_t len;
	char kbuf[512];
	struct rppc_device *rppcdev = rpc->rppcdev;
	struct rppc_msg_header *hdr = (struct rppc_msg_header *)&kbuf[0];
	struct rppc_instance_handle *handle =
				RPPC_PAYLOAD(kbuf, rppc_instance_handle);

	if (rpc->state != RPPC_STATE_CONNECTED)
		return;

	hdr->msg_type = RPPC_MSGTYPE_DELETE_REQ;
	hdr->msg_len = sizeof(u32);
	handle->endpoint_address = rpc->dst;
	handle->status = 0;
	len = sizeof(struct rppc_msg_header) + hdr->msg_len;

	dev_dbg(rpc->dev, "disconnecting from RPC service at %d\n",
		rpc->dst);
	ret = rpmsg_send_offchannel(rppcdev->rpdev->ept, rpc->ept->addr,
				    rppcdev->rpdev->dst, kbuf, len);
	if (ret)
		dev_err(rpc->dev, "rpmsg_send failed: %d\n", ret);

	/*
	 * TODO: should we wait for a message to come back?
	 * For now, no.
	 */
	wait_for_completion_interruptible(&rpc->reply_arrived);
}

static int rppc_register_buffers(struct rppc_instance *rpc,
				 unsigned long arg)
{
	struct rppc_buf_fds data;
	int *fds = NULL;
	struct rppc_dma_buf **bufs = NULL;
	struct rppc_dma_buf *tmp;
	int i = 0, ret = 0;

	if (copy_from_user(&data, (char __user *)arg, sizeof(data)))
		return -EFAULT;

	/* impose a maximum number of buffers for now */
	if (data.num > RPPC_MAX_REG_FDS)
		return -EINVAL;

	fds = kcalloc(data.num, sizeof(*fds), GFP_KERNEL);
	if (!fds)
		return -ENOMEM;

	if (copy_from_user(fds, (char __user *)data.fds,
			   sizeof(*fds) * data.num)) {
		ret = -EFAULT;
		goto free_fds;
	}

	for (i = 0; i < data.num; i++) {
		rcu_read_lock();
		if (!fcheck(fds[i])) {
			rcu_read_unlock();
			ret = -EBADF;
			goto free_fds;
		}
		rcu_read_unlock();

		tmp = rppc_find_dmabuf(rpc, fds[i]);
		if (!IS_ERR_OR_NULL(tmp)) {
			ret = -EEXIST;
			goto free_fds;
		}
	}

	bufs = kcalloc(data.num, sizeof(*bufs), GFP_KERNEL);
	if (!bufs) {
		ret = -ENOMEM;
		goto free_fds;
	}

	for (i = 0; i < data.num; i++) {
		bufs[i] = rppc_alloc_dmabuf(rpc, fds[i], false);
		if (IS_ERR(bufs[i])) {
			ret = PTR_ERR(bufs[i]);
			break;
		}
	}
	if (i == data.num)
		goto free_bufs;

	for (i -= 1; i >= 0; i--)
		rppc_free_dmabuf(bufs[i]->id, bufs[i], rpc);

free_bufs:
	kfree(bufs);
free_fds:
	kfree(fds);
	return ret;
}

static int rppc_unregister_buffers(struct rppc_instance *rpc,
				   unsigned long arg)
{
	struct rppc_buf_fds data;
	int *fds = NULL;
	struct rppc_dma_buf **bufs = NULL;
	int i = 0, ret = 0;

	if (copy_from_user(&data, (char __user *)arg, sizeof(data)))
		return -EFAULT;

	/* impose a maximum number of buffers for now */
	if (data.num > RPPC_MAX_REG_FDS)
		return -EINVAL;

	fds = kcalloc(data.num, sizeof(*fds), GFP_KERNEL);
	if (!fds)
		return -ENOMEM;

	if (copy_from_user(fds, (char __user *)data.fds,
			   sizeof(*fds) * data.num)) {
		ret = -EFAULT;
		goto free_fds;
	}

	bufs = kcalloc(data.num, sizeof(*bufs), GFP_KERNEL);
	if (!bufs) {
		ret = -ENOMEM;
		goto free_fds;
	}

	for (i = 0; i < data.num; i++) {
		rcu_read_lock();
		if (!fcheck(fds[i])) {
			rcu_read_unlock();
			ret = -EBADF;
			goto free_bufs;
		}
		rcu_read_unlock();

		bufs[i] = rppc_find_dmabuf(rpc, fds[i]);
		if (IS_ERR_OR_NULL(bufs[i])) {
			ret = -EEXIST;
			goto free_bufs;
		}
	}

	for (i = 0; i < data.num; i++)
		rppc_free_dmabuf(bufs[i]->id, bufs[i], rpc);

free_bufs:
	kfree(bufs);
free_fds:
	kfree(fds);
	return ret;
}

/*
 * create a new rpc instance that a user-space client can use to invoke
 * remote functions. A new local address would be created and tied with
 * this instance for uniquely identifying the messages communicated by
 * this instance with the remote side.
 *
 * The function is blocking if there is no underlying connection manager
 * channel, unless the device is opened with non-blocking flags specifically.
 */
static int rppc_open(struct inode *inode, struct file *filp)
{
	struct rppc_device *rppcdev;
	struct rppc_instance *rpc;
	struct rpmsg_channel_info chinfo = {};

	rppcdev = container_of(inode->i_cdev, struct rppc_device, cdev);

	if (!rppcdev->rpdev)
		if ((filp->f_flags & O_NONBLOCK) ||
		    wait_for_completion_interruptible(&rppcdev->comp))
			return -EBUSY;

	rpc = kzalloc(sizeof(*rpc), GFP_KERNEL);
	if (!rpc)
		return -ENOMEM;

	mutex_init(&rpc->lock);
	skb_queue_head_init(&rpc->queue);
	init_waitqueue_head(&rpc->readq);
	INIT_LIST_HEAD(&rpc->fxn_list);
	idr_init(&rpc->dma_idr);
	rpc->in_transition = 0;
	rpc->msg_id = 0;
	rpc->state = RPPC_STATE_DISCONNECTED;
	rpc->rppcdev = rppcdev;

	rpc->dev = get_device(rppcdev->dev);
	chinfo.src = RPMSG_ADDR_ANY;
	chinfo.dst = RPMSG_ADDR_ANY;
	rpc->ept = rpmsg_create_ept(rppcdev->rpdev, rppc_cb, rpc, chinfo);
	if (!rpc->ept) {
		dev_err(rpc->dev, "create ept failed\n");
		put_device(rpc->dev);
		kfree(rpc);
		return -ENOMEM;
	}
	filp->private_data = rpc;

	mutex_lock(&rppcdev->lock);
	list_add(&rpc->list, &rppcdev->instances);
	mutex_unlock(&rppcdev->lock);

	dev_dbg(rpc->dev, "local addr assigned: 0x%x\n", rpc->ept->addr);

	return 0;
}

/*
 * release and free all the resources associated with a particular rpc
 * instance. This includes the data structures maintaining the current
 * outstanding function invocations, and all the buffers registered for
 * use with this instance. Send a disconnect message and cleanup the
 * local end-point only if the instance is in a normal state, with the
 * remote connection manager functional.
 */
static int rppc_release(struct inode *inode, struct file *filp)
{
	struct rppc_instance *rpc = filp->private_data;
	struct rppc_device *rppcdev = rpc->rppcdev;

	dev_dbg(rpc->dev, "releasing Instance %p, in state %d\n", rpc,
		rpc->state);

	if (rpc->state != RPPC_STATE_STALE) {
		if (rpc->ept) {
			rppc_disconnect(rpc);
			rpmsg_destroy_ept(rpc->ept);
			rpc->ept = NULL;
		}
	}

	rppc_delete_fxns(rpc);

	mutex_lock(&rpc->lock);
	idr_for_each(&rpc->dma_idr, rppc_free_dmabuf, rpc);
	idr_destroy(&rpc->dma_idr);
	mutex_unlock(&rpc->lock);

	mutex_lock(&rppcdev->lock);
	list_del(&rpc->list);
	mutex_unlock(&rppcdev->lock);

	dev_dbg(rpc->dev, "instance %p has been deleted!\n", rpc);
	if (list_empty(&rppcdev->instances))
		dev_dbg(rpc->dev, "all instances have been removed!\n");

	put_device(rpc->dev);
	kfree(rpc);
	return 0;
}

static long rppc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct rppc_instance *rpc = filp->private_data;
	struct rppc_create_instance connect;
	int ret = 0;

	dev_dbg(rpc->dev, "%s: cmd %d, arg 0x%lx\n", __func__, cmd, arg);

	if (_IOC_TYPE(cmd) != RPPC_IOC_MAGIC)
		return -ENOTTY;

	if (_IOC_NR(cmd) > RPPC_IOC_MAXNR)
		return -ENOTTY;

	switch (cmd) {
	case RPPC_IOC_CREATE:
		ret = copy_from_user(&connect, (char __user *)arg,
				     sizeof(connect));
		if (ret) {
			dev_err(rpc->dev, "%s: %d: copy_from_user fail: %d\n",
				__func__, _IOC_NR(cmd), ret);
			ret = -EFAULT;
		} else {
			connect.name[sizeof(connect.name) - 1] = '\0';
			ret = rppc_connect(rpc, &connect);
		}
		break;
	case RPPC_IOC_BUFREGISTER:
		ret = rppc_register_buffers(rpc, arg);
		break;
	case RPPC_IOC_BUFUNREGISTER:
		ret = rppc_unregister_buffers(rpc, arg);
		break;
	default:
		dev_err(rpc->dev, "unhandled ioctl cmd: %d\n", cmd);
		break;
	}

	return ret;
}

static ssize_t rppc_read(struct file *filp, char __user *buf, size_t len,
			 loff_t *offp)
{
	struct rppc_instance *rpc = filp->private_data;
	struct rppc_packet *packet = NULL;
	struct rppc_param_data *parameters = NULL;
	struct rppc_function *function = NULL;
	struct rppc_function_return returned;
	struct sk_buff *skb = NULL;
	int ret = 0;
	int use = sizeof(returned);
	DEFINE_WAIT(wait);

	if (mutex_lock_interruptible(&rpc->lock))
		return -ERESTARTSYS;

	/* instance is invalid */
	if (rpc->state == RPPC_STATE_STALE) {
		mutex_unlock(&rpc->lock);
		return -ENXIO;
	}

	/* not yet connected to the remote side */
	if (rpc->state == RPPC_STATE_DISCONNECTED) {
		mutex_unlock(&rpc->lock);
		return -ENOTCONN;
	}

	if (len > use) {
		mutex_unlock(&rpc->lock);
		return -EOVERFLOW;
	}
	if (len < use) {
		mutex_unlock(&rpc->lock);
		return -EINVAL;
	}

	/* TODO: Use the much simpler wait_event_interruptible API */
	while (skb_queue_empty(&rpc->queue)) {
		mutex_unlock(&rpc->lock);
		/* non-blocking requested ? return now */
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;

		prepare_to_wait_exclusive(&rpc->readq, &wait,
					  TASK_INTERRUPTIBLE);
		if (skb_queue_empty(&rpc->queue) &&
		    rpc->state != RPPC_STATE_STALE)
			schedule();
		finish_wait(&rpc->readq, &wait);
		if (signal_pending(current))
			return -ERESTARTSYS;

		ret = mutex_lock_interruptible(&rpc->lock);
		if (ret < 0)
			return -ERESTARTSYS;

		if (rpc->state == RPPC_STATE_STALE) {
			mutex_unlock(&rpc->lock);
			return -ENXIO;
		}

		/* make sure state is sane while we waited */
		if (rpc->state != RPPC_STATE_CONNECTED) {
			mutex_unlock(&rpc->lock);
			ret = -EIO;
			goto out;
		}
	}

	skb = skb_dequeue(&rpc->queue);
	if (WARN_ON(!skb)) {
		mutex_unlock(&rpc->lock);
		ret = -EIO;
		goto out;
	}

	mutex_unlock(&rpc->lock);

	packet = (struct rppc_packet *)skb->data;
	parameters = (struct rppc_param_data *)packet->data;

	/*
	 * pull the function memory from the list and untranslate
	 * the remote device address pointers in the packet back
	 * to MPU pointers.
	 */
	function = rppc_find_fxn(rpc, packet->msg_id);
	if (function && function->num_translations > 0) {
		ret = rppc_xlate_buffers(rpc, function, RPPC_RPA_TO_UVA);
		if (ret < 0) {
			dev_err(rpc->dev, "failed to translate back pointers from remote core!\n");
			goto failure;
		}
	}
	returned.fxn_id = RPPC_FXN_MASK(packet->fxn_id);
	returned.status = packet->result;

	if (copy_to_user(buf, &returned, use)) {
		dev_err(rpc->dev, "%s: copy_to_user fail\n", __func__);
		ret = -EFAULT;
	} else {
		ret = use;
	}

failure:
	kfree(function);
	kfree_skb(skb);
out:
	return ret;
}

static ssize_t rppc_write(struct file *filp, const char __user *ubuf,
			  size_t len, loff_t *offp)
{
	struct rppc_instance *rpc = filp->private_data;
	struct rppc_device *rppcdev = rpc->rppcdev;
	struct device *dev = rpc->dev;
	struct rppc_msg_header *hdr = NULL;
	struct rppc_function *function = NULL;
	struct rppc_packet *packet = NULL;
	struct rppc_param_data *parameters = NULL;
	char kbuf[512];
	int use = 0, ret = 0, param = 0;
	u32 sig_idx = 0;
	u32 sig_prm = 0;
	static u32 rppc_atomic_size[RPPC_PARAM_ATOMIC_MAX] = {
		0, /* RPPC_PARAM_VOID */
		1, /* RPPC_PARAM_S08 */
		1, /* RPPC_PARAM_U08 */
		2, /* RPPC_PARAM_S16 */
		2, /* RPPC_PARAM_U16 */
		4, /* RPPC_PARAM_S32 */
		4, /* RPPC_PARAM_U32 */
		8, /* RPPC_PARAM_S64 */
		8  /* RPPC_PARAM_U64 */
	};

	if (len < sizeof(*function)) {
		ret = -ENOTSUPP;
		goto failure;
	}

	if (len > (sizeof(*function) + RPPC_MAX_TRANSLATIONS *
				sizeof(struct rppc_param_translation))) {
		ret = -ENOTSUPP;
		goto failure;
	}

	if (rpc->state != RPPC_STATE_CONNECTED) {
		ret = -ENOTCONN;
		goto failure;
	}

	function = kzalloc(len, GFP_KERNEL);
	if (!function) {
		ret = -ENOMEM;
		goto failure;
	}

	if (copy_from_user(function, ubuf, len)) {
		ret = -EMSGSIZE;
		goto failure;
	}

	if (function->fxn_id >= rppcdev->num_funcs - 1) {
		ret = -EINVAL;
		goto failure;
	}

	/* increment the message id and wrap if needed */
	rpc->msg_id = (rpc->msg_id + 1) & 0xFFFF;

	memset(kbuf, 0, sizeof(kbuf));
	sig_idx = function->fxn_id + 1;
	hdr = (struct rppc_msg_header *)kbuf;
	hdr->msg_type = RPPC_MSGTYPE_FUNCTION_CALL;
	hdr->msg_len = sizeof(*packet);
	packet = RPPC_PAYLOAD(kbuf, rppc_packet);
	packet->desc = RPPC_DESC_EXEC_SYNC;
	packet->msg_id = rpc->msg_id;
	packet->flags = (RPPC_JOBID_DISCRETE << 16) | RPPC_POOLID_DEFAULT;
	packet->fxn_id = RPPC_SET_FXN_IDX(function->fxn_id);
	packet->result = 0;
	packet->data_size = sizeof(*parameters) * function->num_params;

	/* check the signatures against what were published */
	if (RPPC_SIG_NUM_PARAM(rppcdev->signatures[sig_idx]) !=
		function->num_params) {
		dev_err(dev, "number of parameters mismatch! params = %u expected = %u\n",
			function->num_params,
			RPPC_SIG_NUM_PARAM(rppcdev->signatures[sig_idx]));
		ret = -EINVAL;
		goto failure;
	}

	/*
	 * compute the parameter pointer changes last since this will cause the
	 * cache operations
	 */
	parameters = (struct rppc_param_data *)packet->data;
	for (param = 0; param < function->num_params; param++) {
		u32 param_type;

		sig_prm = param + 1;
		param_type = rppcdev->signatures[sig_idx].params[sig_prm].type;
		/*
		 * check to make sure the parameter description matches the
		 * signature published from the other side.
		 */
		if (function->params[param].type == RPPC_PARAM_TYPE_PTR &&
		    !RPPC_IS_PTR(param_type)) {
			dev_err(dev, "parameter %u Pointer Type Mismatch sig type:%x func %u\n",
				param, param_type, sig_idx);
			ret = -EINVAL;
			goto failure;
		} else if (param > 0 && function->params[param].type ==
			RPPC_PARAM_TYPE_ATOMIC) {
			if (!RPPC_IS_ATOMIC(param_type)) {
				dev_err(dev, "parameter Atomic Type Mismatch\n");
				ret = -EINVAL;
				goto failure;
			} else {
				if (rppc_atomic_size[param_type] !=
					function->params[param].size) {
					dev_err(dev, "size mismatch! u:%u sig:%u\n",
						function->params[param].size,
						rppc_atomic_size[param_type]);
					ret = -EINVAL;
					goto failure;
				}
			}
		}

		parameters[param].size = function->params[param].size;

		/* check the type and lookup if it's a pointer */
		if (function->params[param].type == RPPC_PARAM_TYPE_PTR) {
			/*
			 * internally the buffer translations takes care of the
			 * offsets.
			 */
			int fd = function->params[param].fd;

			parameters[param].data = (size_t)rppc_buffer_lookup(rpc,
				(virt_addr_t)function->params[param].data,
				(virt_addr_t)function->params[param].base, fd);
		} else if (function->params[param].type ==
			   RPPC_PARAM_TYPE_ATOMIC) {
			parameters[param].data = function->params[param].data;
		} else {
			ret = -ENOTSUPP;
			goto failure;
		}
	}

	/* compute the size of the rpmsg packet */
	use = sizeof(*hdr) + hdr->msg_len + packet->data_size;

	/* failed to provide the translation data */
	if (function->num_translations > 0 &&
	    len < (sizeof(*function) + (function->num_translations *
				sizeof(struct rppc_param_translation)))) {
		ret = -EINVAL;
		goto failure;
	}

	/*
	 * if there are pointers to translate for the user, do so now.
	 * alter our copy of function and the user's parameters so that
	 * the proper pointers can be sent to remote cores
	 */
	if (function->num_translations > 0) {
		ret = rppc_xlate_buffers(rpc, function, RPPC_UVA_TO_RPA);
		if (ret < 0) {
			dev_err(dev, "failed to translate all pointers for remote core!\n");
			goto failure;
		}
	}

	ret = rppc_add_fxn(rpc, function, rpc->msg_id);
	if (ret < 0) {
		rppc_xlate_buffers(rpc, function, RPPC_RPA_TO_UVA);
		goto failure;
	}

	rppc_print_msg(rpc, "TX:", kbuf);

	ret = rpmsg_send_offchannel(rppcdev->rpdev->ept, rpc->ept->addr,
				    rpc->dst, kbuf, use);
	if (ret) {
		dev_err(dev, "rpmsg_send failed: %d\n", ret);
		rppc_find_fxn(rpc, rpc->msg_id);
		rppc_xlate_buffers(rpc, function, RPPC_RPA_TO_UVA);
		goto failure;
	}
	dev_dbg(dev, "==> sent msg to remote endpoint %u\n", rpc->dst);

failure:
	if (ret >= 0)
		ret = len;
	else
		kfree(function);

	return ret;
}

static __poll_t rppc_poll(struct file *filp, struct poll_table_struct *wait)
{
	struct rppc_instance *rpc = filp->private_data;
	__poll_t mask = 0;

	poll_wait(filp, &rpc->readq, wait);
	if (rpc->state == RPPC_STATE_STALE) {
		mask = EPOLLERR;
		goto out;
	}

	/* if the queue is not empty set the poll bit correctly */
	if (!skb_queue_empty(&rpc->queue))
		mask |= (EPOLLIN | EPOLLRDNORM);

	/* TODO: writes are deemed to be successful always, fix this later */
	if (true)
		mask |= EPOLLOUT | EPOLLWRNORM;

out:
	return mask;
}

static const struct file_operations rppc_fops = {
	.owner = THIS_MODULE,
	.open = rppc_open,
	.release = rppc_release,
	.unlocked_ioctl = rppc_ioctl,
	.read = rppc_read,
	.write = rppc_write,
	.poll = rppc_poll,
};

/*
 * send a function query message, the sysfs entry will be created
 * during the processing of the response message
 */
static int rppc_query_function(struct rpmsg_device *rpdev)
{
	int ret = 0;
	u32 len = 0;
	char kbuf[512];
	struct rppc_device *rppcdev = dev_get_drvdata(&rpdev->dev);
	struct rppc_msg_header *hdr = (struct rppc_msg_header *)&kbuf[0];
	struct rppc_query_function *fxn_info =
				(struct rppc_query_function *)hdr->msg_data;

	if (rppcdev->cur_func >= rppcdev->num_funcs)
		return -EINVAL;

	hdr->msg_type = RPPC_MSGTYPE_FUNCTION_QUERY;
	hdr->msg_len = sizeof(*fxn_info);
	len = sizeof(*hdr) + hdr->msg_len;
	fxn_info->info_type = RPPC_INFOTYPE_FUNC_SIGNATURE;
	fxn_info->fxn_id = rppcdev->cur_func++;

	dev_dbg(&rpdev->dev, "sending function query type %u for function %u\n",
		fxn_info->info_type, fxn_info->fxn_id);
	ret = rpmsg_send(rpdev->ept, (char *)kbuf, len);
	if (ret) {
		dev_err(&rpdev->dev, "rpmsg_send failed: %d\n", ret);
		return ret;
	}

	return 0;
}

static void
rppc_handle_devinfo_resp(struct rpmsg_device *rpdev, char *data, int len)
{
	struct rppc_device *rppcdev = dev_get_drvdata(&rpdev->dev);
	struct rppc_device_info *info;
	u32 exp_len = sizeof(*info) + sizeof(struct rppc_msg_header);

	if (len != exp_len) {
		dev_err(&rpdev->dev, "invalid message length %d (expected %d bytes)",
			len, exp_len);
		return;
	}

	info = RPPC_PAYLOAD(data, rppc_device_info);
	if (info->num_funcs > RPPC_MAX_NUM_FUNCS) {
		rppcdev->num_funcs = 0;
		dev_err(&rpdev->dev, "number of functions (%d) exceeds the limit supported(%d)\n",
			info->num_funcs, RPPC_MAX_NUM_FUNCS);
		return;
	}

	rppcdev->num_funcs = info->num_funcs;
	rppcdev->signatures = kcalloc(rppcdev->num_funcs,
				      sizeof(struct rppc_func_signature),
				      GFP_KERNEL);
	if (!rppcdev->signatures)
		return;

	dev_info(&rpdev->dev, "published functions = %u\n", info->num_funcs);

	/* send the function query for first function */
	if (rppc_query_function(rpdev) == -EINVAL)
		dev_err(&rpdev->dev, "failed to get a reasonable number of functions!\n");
}

static void
rppc_handle_fxninfo_resp(struct rpmsg_device *rpdev, char *data, int len)
{
	struct rppc_device *rppcdev = dev_get_drvdata(&rpdev->dev);
	struct rppc_query_function *fxn_info;
	struct rppc_func_signature *signature;
	u32 exp_len = sizeof(*fxn_info) + sizeof(struct rppc_msg_header);
	int i;

	if (len != exp_len) {
		dev_err(&rpdev->dev, "invalid message length %d (expected %d bytes)",
			len, exp_len);
		return;
	}

	fxn_info = RPPC_PAYLOAD(data, rppc_query_function);
	dev_dbg(&rpdev->dev, "response for function query of type %u\n",
		fxn_info->info_type);

	switch (fxn_info->info_type) {
	case RPPC_INFOTYPE_FUNC_SIGNATURE:
		if (fxn_info->fxn_id >= rppcdev->num_funcs) {
			dev_err(&rpdev->dev, "function(%d) is out of range!\n",
				fxn_info->fxn_id);
			break;
		}

		memcpy(&rppcdev->signatures[fxn_info->fxn_id],
		       &fxn_info->info.signature, sizeof(*signature));

		/* TODO: delete these debug prints later */
		dev_dbg(&rpdev->dev, "received info for func(%d); name = %s #params = %u\n",
			fxn_info->fxn_id, fxn_info->info.signature.name,
			fxn_info->info.signature.num_param);
		signature = &rppcdev->signatures[fxn_info->fxn_id];
		for (i = 0; i < signature->num_param; i++) {
			dev_dbg(&rpdev->dev, "param[%u] type = %x dir = %u\n",
				i, signature->params[i].type,
				signature->params[i].direction);
		}

		/* query again until we've hit our limit */
		if (rppc_query_function(rpdev) == -EINVAL) {
			dev_dbg(&rpdev->dev, "reached end of function list!\n");
			rppc_create_sysfs(rppcdev);
		}
		break;
	default:
		dev_err(&rpdev->dev, "unrecognized fxn query response %u\n",
			fxn_info->info_type);
		break;
	}
}

static int rppc_driver_cb(struct rpmsg_device *rpdev, void *data, int len,
			  void *priv, u32 src)
{
	struct rppc_msg_header *hdr = data;
	char *buf = (char *)data;

	dev_dbg(&rpdev->dev, "<== incoming drv msg src %d len %d msg_type %d msg_len %d\n",
		src, len, hdr->msg_type, hdr->msg_len);

	if (len <= sizeof(*hdr)) {
		dev_err(&rpdev->dev, "message truncated\n");
		return -EINVAL;
	}

	switch (hdr->msg_type) {
	case RPPC_MSGTYPE_DEVINFO_RESP:
		rppc_handle_devinfo_resp(rpdev, buf, len);
		break;
	case RPPC_MSGTYPE_FUNCTION_INFO:
		rppc_handle_fxninfo_resp(rpdev, buf, len);
		break;
	default:
		dev_err(&rpdev->dev, "unrecognized message type %u\n",
			hdr->msg_type);
		break;
	}

	return 0;
}

static int find_rpccdev_by_name(int id, void *p, void *data)
{
	struct rppc_device *rppcdev = p;

	return strcmp(rppcdev->desc, data) ? 0 : (int)p;
}

/*
 * send a device info query message, the device will be created
 * during the processing of the response message
 */
static int rppc_device_create(struct rpmsg_device *rpdev)
{
	int ret;
	u32 len;
	char kbuf[512];
	struct rppc_msg_header *hdr = (struct rppc_msg_header *)&kbuf[0];

	hdr->msg_type = RPPC_MSGTYPE_DEVINFO_REQ;
	hdr->msg_len = 0;
	len = sizeof(*hdr);
	ret = rpmsg_send(rpdev->ept, (char *)kbuf, len);
	if (ret) {
		dev_err(&rpdev->dev, "rpmsg_send failed: %d\n", ret);
		return ret;
	}

	return 0;
}

static int rppc_probe(struct rpmsg_device *rpdev)
{
	int ret, minor;
	int major = MAJOR(rppc_dev);
	struct rppc_device *rppcdev = NULL;
	dev_t dev;
	char namedesc[RPMSG_NAME_SIZE];

	dev_info(&rpdev->dev, "probing service %s with src %u dst %u\n",
		 rpdev->desc, rpdev->src, rpdev->dst);

	mutex_lock(&rppc_devices_lock);
	snprintf(namedesc, sizeof(namedesc), "%s", rpdev->desc);
	rppcdev = (struct rppc_device *)idr_for_each(&rppc_devices,
						find_rpccdev_by_name, namedesc);
	if (rppcdev) {
		rppcdev->rpdev = rpdev;
		dev_set_drvdata(&rpdev->dev, rppcdev);
		goto serv_up;
	}

	rppcdev = kzalloc(sizeof(*rppcdev), GFP_KERNEL);
	if (!rppcdev) {
		ret = -ENOMEM;
		goto exit;
	}

	minor = idr_alloc(&rppc_devices, rppcdev, 0, 0, GFP_KERNEL);
	if (minor < 0) {
		ret = minor;
		dev_err(&rpdev->dev, "failed to get a minor number: %d\n", ret);
		goto free_rppcdev;
	}

	INIT_LIST_HEAD(&rppcdev->instances);
	mutex_init(&rppcdev->lock);
	init_completion(&rppcdev->comp);

	rppcdev->minor = minor;
	rppcdev->rpdev = rpdev;
	strncpy(rppcdev->desc, namedesc, RPMSG_NAME_SIZE);
	dev_set_drvdata(&rpdev->dev, rppcdev);

	cdev_init(&rppcdev->cdev, &rppc_fops);
	rppcdev->cdev.owner = THIS_MODULE;
	dev = MKDEV(major, minor);
	ret = cdev_add(&rppcdev->cdev, dev, 1);
	if (ret) {
		dev_err(&rpdev->dev, "cdev_add failed: %d\n", ret);
		goto free_id;
	}

serv_up:
	rppcdev->dev = device_create(rppc_class, &rpdev->dev,
				     MKDEV(major, rppcdev->minor), NULL,
				     namedesc);
	if (IS_ERR(rppcdev->dev)) {
		ret = PTR_ERR(rppcdev->dev);

		dev_err(&rpdev->dev, "device_create failed: %d\n", ret);
		goto free_cdev;
	}
	dev_set_drvdata(rppcdev->dev, rppcdev);

	ret = rppc_device_create(rpdev);
	if (ret) {
		dev_err(&rpdev->dev, "failed to query channel info: %d\n", ret);
		dev = MKDEV(MAJOR(rppc_dev), rppcdev->minor);
		goto free_dev;
	}

	complete_all(&rppcdev->comp);

	dev_dbg(&rpdev->dev, "new RPPC connection srv channel: %u -> %u!\n",
		rpdev->src, rpdev->dst);

	mutex_unlock(&rppc_devices_lock);
	return 0;

free_dev:
	device_destroy(rppc_class, dev);
free_cdev:
	cdev_del(&rppcdev->cdev);
free_id:
	idr_remove(&rppc_devices, rppcdev->minor);
free_rppcdev:
	kfree(rppcdev);
exit:
	mutex_unlock(&rppc_devices_lock);
	return ret;
}

static void rppc_remove(struct rpmsg_device *rpdev)
{
	struct rppc_device *rppcdev = dev_get_drvdata(&rpdev->dev);
	struct rppc_instance *rpc = NULL;
	int major = MAJOR(rppc_dev);

	dev_dbg(&rpdev->dev, "removing rpmsg-rpc device %u.%u\n",
		major, rppcdev->minor);

	mutex_lock(&rppc_devices_lock);

	rppc_remove_sysfs(rppcdev);
	rppcdev->cur_func = 0;
	kfree(rppcdev->signatures);

	/* if there are no instances in the list, just teardown */
	if (list_empty(&rppcdev->instances)) {
		dev_dbg(&rpdev->dev, "no instances, removing device!\n");
		device_destroy(rppc_class, MKDEV(major, rppcdev->minor));
		cdev_del(&rppcdev->cdev);
		idr_remove(&rppc_devices, rppcdev->minor);
		kfree(rppcdev);
		mutex_unlock(&rppc_devices_lock);
		return;
	}

	/*
	 * if there are rpc instances that means that this is a recovery
	 * operation. Don't clean the rppcdev, and retain it for reuse.
	 * mark each instance as invalid, and complete any on-going transactions
	 */
	init_completion(&rppcdev->comp);
	mutex_lock(&rppcdev->lock);
	list_for_each_entry(rpc, &rppcdev->instances, list) {
		dev_dbg(&rpdev->dev, "instance %p in state %d\n",
			rpc, rpc->state);
		if (rpc->state == RPPC_STATE_CONNECTED && rpc->in_transition)
			complete_all(&rpc->reply_arrived);
		rpc->state = RPPC_STATE_STALE;
		wake_up_interruptible(&rpc->readq);
	}
	device_destroy(rppc_class, MKDEV(major, rppcdev->minor));
	rppcdev->dev = NULL;
	rppcdev->rpdev = NULL;
	mutex_unlock(&rppcdev->lock);
	mutex_unlock(&rppc_devices_lock);
	dev_dbg(&rpdev->dev, "removed rpmsg rpmsg-rpc service %s\n",
		rpdev->desc);
}

static struct rpmsg_device_id rppc_id_table[] = {
	{.name = "rpmsg-rpc"},
	{},
};

static struct rpmsg_driver rppc_driver = {
	.drv.name = KBUILD_MODNAME,
	.id_table = rppc_id_table,
	.probe = rppc_probe,
	.remove = rppc_remove,
	.callback = rppc_driver_cb,
};

static int __init rppc_init(void)
{
	int ret;

	ret = alloc_chrdev_region(&rppc_dev, 0, RPPC_MAX_DEVICES,
				  KBUILD_MODNAME);
	if (ret) {
		pr_err("alloc_chrdev_region failed: %d\n", ret);
		goto out;
	}

	rppc_class = class_create(THIS_MODULE, KBUILD_MODNAME);
	if (IS_ERR(rppc_class)) {
		ret = PTR_ERR(rppc_class);
		pr_err("class_create failed: %d\n", ret);
		goto unreg_region;
	}

	ret = register_rpmsg_driver(&rppc_driver);
	if (ret) {
		pr_err("register_rpmsg_driver failed: %d\n", ret);
		goto destroy_class;
	}
	return 0;

destroy_class:
	class_destroy(rppc_class);
unreg_region:
	unregister_chrdev_region(rppc_dev, RPPC_MAX_DEVICES);
out:
	return ret;
}

static void __exit rppc_exit(void)
{
	unregister_rpmsg_driver(&rppc_driver);
	class_destroy(rppc_class);
	unregister_chrdev_region(rppc_dev, RPPC_MAX_DEVICES);
}

module_init(rppc_init);
module_exit(rppc_exit);
MODULE_DEVICE_TABLE(rpmsg, rppc_id_table);

MODULE_AUTHOR("Suman Anna <s-anna@ti.com>");
MODULE_AUTHOR("Erik Rainey <erik.rainey@ti.com>");
MODULE_DESCRIPTION("Remote Processor Procedure Call Driver");
MODULE_ALIAS("rpmsg:rpmsg-rpc");
MODULE_LICENSE("GPL v2");
