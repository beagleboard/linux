/*
 * Texas Instruments System Control Interface Protocol Driver
 *
 * Copyright (C) 2015-2016 Texas Instruments Incorporated - http://www.ti.com/
 *	Nishanth Menon
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/bitmap.h>
#include <linux/debugfs.h>
#include <linux/export.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/soc/ti/ti-msgmgr.h>
#include <linux/soc/ti/ti_sci_protocol.h>
#include <linux/reboot.h>

#include "ti_sci.h"

/* List of all TI SCI devices active in system */
static LIST_HEAD(ti_sci_list);
/* Protection for the entire list */
static DEFINE_MUTEX(ti_sci_list_mutex);

/**
 * struct ti_sci_xfer - Structure representing a message flow
 * @tx_message:	Transmit message
 * @rx_len:	Receive message length
 * @xfer_buf:	Preallocated buffer to store receive message
 *		Since we work with request-ACK protocol, we can
 *		reuse the same buffer for the rx path as we
 *		use for the tx path.
 * @done:	completion event
 */
struct ti_sci_xfer {
	struct ti_msgmgr_message tx_message;
	u8 rx_len;
	u8 *xfer_buf;
	struct completion done;
};

/**
 * struct ti_sci_xfers_info - Structure to manage transfer information
 * @sem_xfer_count:	Counting Semaphore for managing max simultaneous
 *			Messages.
 * @xfer_block:		Preallocated Message array
 * @xfer_alloc_table:	Bitmap table for allocated messages.
 *			Index of this bitmap table is also used for message
 *			sequence identifier.
 * @xfer_lock:		Protection for message allocation
 */
struct ti_sci_xfers_info {
	struct semaphore sem_xfer_count;
	struct ti_sci_xfer *xfer_block;
	unsigned long *xfer_alloc_table;
	/* protect transfer allocation */
	spinlock_t xfer_lock;
};

/**
 * struct ti_sci_desc - Description of SoC integration
 * @default_host_id:	Host identifier representing the compute entity
 * @max_rx_timeout_ms:	Timeout for communication with SoC (in Milliseconds)
 * @max_msgs: Maximum number of messages that can be pending
 *		  simultaneously in the system
 * @max_msg_size: Maximum size of data per message that can be handled.
 */
struct ti_sci_desc {
	u8 default_host_id;
	int max_rx_timeout_ms;
	int max_msgs;
	int max_msg_size;
};

/**
 * struct ti_sci_info - Structure representing a TI SCI instance
 * @dev:	Device pointer
 * @desc:	SoC description for this instance
 * @nb:	Reboot Notifier block
 * @d:		Debugfs file entry
 * @debug_region: Memory region where the debug message are available
 * @debug_region_size: Debug region size
 * @debug_buffer: Buffer allocated to copy debug messages.
 * @handle:	Instance of TI SCI handle to send to clients.
 * @cl:		Mailbox Client
 * @chan_tx:	Transmit mailbox channel
 * @chan_rx:	Receive mailbox channel
 * @minfo:	Message info
 * @node:	list head
 * @host_id:	Host ID
 * @users:	Number of users of this instance
 */
struct ti_sci_info {
	struct device *dev;
	struct notifier_block nb;
	const struct ti_sci_desc *desc;
	struct dentry *d;
	void __iomem *debug_region;
	char *debug_buffer;
	size_t debug_region_size;
	struct ti_sci_handle handle;
	struct mbox_client cl;
	struct mbox_chan *chan_tx;
	struct mbox_chan *chan_rx;
	struct ti_sci_xfers_info minfo;
	struct list_head node;
	u8 host_id;
	/* protected by ti_sci_list_mutex */
	int users;

};

#define cl_to_ti_sci_info(c)	container_of(c, struct ti_sci_info, cl)
#define handle_to_ti_sci_info(h) container_of(h, struct ti_sci_info, handle)
#define reboot_to_ti_sci_info(n) container_of(n, struct ti_sci_info, nb)

#ifdef CONFIG_DEBUG_FS

/**
 * ti_sci_debug_show() - Helper to dump the debug log
 * @s:	sequence file pointer
 * @unused:	unused.
 *
 * Return: 0
 */
static int ti_sci_debug_show(struct seq_file *s, void *unused)
{
	struct ti_sci_info *info = s->private;

	memcpy_fromio(info->debug_buffer, info->debug_region,
		      info->debug_region_size);
	/*
	 * We don't trust firmware to leave NULL terminated last byte (hence
	 * we have allocated 1 extra 0 byte). Since we cannot guarantee any
	 * specific data format for debug messages, We just present the data
	 * in the buffer as is - we expect the messages to be self explanatory.
	 */
	seq_puts(s, info->debug_buffer);
	return 0;
}

/**
 * ti_sci_debug_open() - debug file open
 * @inode:	inode pointer
 * @file:	file pointer
 *
 * Return: result of single_open
 */
static int ti_sci_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, ti_sci_debug_show, inode->i_private);
}

/* log file operations */
static const struct file_operations ti_sci_debug_fops = {
	.open = ti_sci_debug_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/**
 * ti_sci_debugfs_create() - Create log debug file
 * @pdev:	platform device pointer
 * @info:	Pointer to SCI entity information
 *
 * Return: 0 if all went fine, else corresponding error.
 */
static int ti_sci_debugfs_create(struct platform_device *pdev,
				 struct ti_sci_info *info)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	char debug_name[50] = "ti_sci_debug@";

	/* Debug region is optional */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					   "debug_messages");
	info->debug_region = devm_ioremap_resource(dev, res);
	if (IS_ERR(info->debug_region))
		return 0;
	info->debug_region_size = resource_size(res);

	info->debug_buffer = devm_kcalloc(dev, info->debug_region_size + 1,
					  sizeof(char), GFP_KERNEL);
	if (!info->debug_buffer)
		return -ENOMEM;
	/* Setup NULL termination */
	info->debug_buffer[info->debug_region_size] = 0;

	info->d = debugfs_create_file(strncat(debug_name, dev_name(dev),
					      sizeof(debug_name) -
					      sizeof("ti_sci_debug@")),
				      0444, NULL, info, &ti_sci_debug_fops);
	if (IS_ERR(info->d))
		return PTR_ERR(info->d);

	dev_dbg(dev, "Debug region => %p, size = %zu bytes, resource: %pr\n",
		info->debug_region, info->debug_region_size, res);
	return 0;
}

/**
 * ti_sci_debugfs_destroy() - clean up log debug file
 * @pdev:	platform device pointer
 * @info:	Pointer to SCI entity information
 */
static void ti_sci_debugfs_destroy(struct platform_device *pdev,
				   struct ti_sci_info *info)
{
	if (IS_ERR(info->debug_region))
		return;

	debugfs_remove(info->d);
}
#else /* CONFIG_DEBUG_FS */
static inline int ti_sci_debugfs_create(struct platform_device *dev,
					struct ti_sci_info *info)
{
	return 0;
}

static inline void ti_sci_debugfs_destroy(struct platform_device *dev,
					  struct ti_sci_info *info)
{
}
#endif /* CONFIG_DEBUG_FS */

/**
 * ti_sci_dump_header_dbg() - Helper to dump a message header.
 * @dev:	Device pointer corresponding to the SCI entity
 * @hdr:	pointer to header.
 */
static inline void ti_sci_dump_header_dbg(struct device *dev,
					  struct ti_sci_msg_hdr *hdr)
{
	dev_dbg(dev, "MSGHDR:type=0x%04x host=0x%02x seq=0x%02x flags=0x%08x\n",
		hdr->type, hdr->host, hdr->seq, hdr->flags);
}

/**
 * ti_sci_rx_callback() - mailbox client callback for receive messages
 * @cl:	client pointer
 * @m:	mailbox message
 *
 * Processes one received message to appropriate transfer information and
 * signals completion of the transfer.
 *
 * NOTE: This function will be invoked in IRQ context, hence should be
 * as optimal as possible.
 */
static void ti_sci_rx_callback(struct mbox_client *cl, void *m)
{
	struct ti_sci_info *info = cl_to_ti_sci_info(cl);
	struct device *dev = info->dev;
	struct ti_sci_xfers_info *minfo = &info->minfo;
	struct ti_msgmgr_message *mbox_msg = m;
	struct ti_sci_msg_hdr *hdr = (struct ti_sci_msg_hdr *)mbox_msg->buf;
	struct ti_sci_xfer *xfer;
	u8 xfer_id;

	xfer_id = hdr->seq;

	/*
	 * Are we even expecting this?
	 * NOTE: barriers were implicit in locks used for modifying the bitmap
	 */
	if (!test_bit(xfer_id, minfo->xfer_alloc_table)) {
		dev_err(dev, "Message for %d is not expected!\n", xfer_id);
		return;
	}

	xfer = &minfo->xfer_block[xfer_id];

	/* Is the message of valid length? */
	if (mbox_msg->len > info->desc->max_msg_size) {
		dev_err(dev, "Unable to handle %zu xfer(max %d)\n",
			mbox_msg->len, info->desc->max_msg_size);
		ti_sci_dump_header_dbg(dev, hdr);
		return;
	}
	if (mbox_msg->len < xfer->rx_len) {
		dev_err(dev, "Recv xfer %zu < expected %d length\n",
			mbox_msg->len, xfer->rx_len);
		ti_sci_dump_header_dbg(dev, hdr);
		return;
	}

	ti_sci_dump_header_dbg(dev, hdr);
	/* Take a copy to the rx buffer.. */
	memcpy(xfer->xfer_buf, mbox_msg->buf, xfer->rx_len);
	complete(&xfer->done);
}

/**
 * ti_sci_get_one_xfer() - Allocate one message
 * @info:	Pointer to SCI entity information
 * @msg_type:	Message type
 * @msg_flags:	Flag to set for the message
 * @tx_message_size: transmit message size
 * @rx_message_size: receive message size
 *
 * Helper function which is used by various command functions that are
 * exposed to clients of this driver for allocating a message traffic event.
 *
 * This function can sleep depending on pending requests already in the system
 * for the SCI entity. Further, this also holds a spinlock to maintain integrity
 * of internal data structures.
 *
 * Return: 0 if all went fine, else corresponding error.
 */
static struct ti_sci_xfer *ti_sci_get_one_xfer(struct ti_sci_info *info,
					       u16 msg_type, u32 msg_flags,
					       size_t tx_message_size,
					       size_t rx_message_size)
{
	struct ti_sci_xfers_info *minfo = &info->minfo;
	struct ti_sci_xfer *xfer;
	struct ti_sci_msg_hdr *hdr;
	unsigned long flags;
	unsigned long bit_pos;
	u8 xfer_id;
	int ret;
	int timeout;

	/* Ensure we have sane transfer sizes */
	if (rx_message_size > info->desc->max_msg_size ||
	    tx_message_size > info->desc->max_msg_size ||
	    rx_message_size < sizeof(*hdr) || tx_message_size < sizeof(*hdr))
		return ERR_PTR(-ERANGE);

	/*
	 * Ensure we have only controlled number of pending messages.
	 * Ideally, we might just have to wait a single message, be
	 * conservative and wait 5 times that..
	 */
	timeout = msecs_to_jiffies(info->desc->max_rx_timeout_ms) * 5;
	ret = down_timeout(&minfo->sem_xfer_count, timeout);
	if (ret < 0)
		return ERR_PTR(ret);

	/* Keep the locked section as small as possible */
	spin_lock_irqsave(&minfo->xfer_lock, flags);
	bit_pos = find_first_zero_bit(minfo->xfer_alloc_table,
				      info->desc->max_msgs);
	set_bit(bit_pos, minfo->xfer_alloc_table);
	spin_unlock_irqrestore(&minfo->xfer_lock, flags);

	/*
	 * We already ensured in probe that we can have max messages that can
	 * fit in  hdr.seq - NOTE: this improves access latencies
	 * to predictable O(1) access, BUT, it opens us to risk if
	 * remote misbehaves with corrupted message sequence responses.
	 * If that happens, we are going to be messed up anyways..
	 */
	xfer_id = (u8)bit_pos;

	xfer = &minfo->xfer_block[xfer_id];

	hdr = (struct ti_sci_msg_hdr *)xfer->tx_message.buf;
	xfer->tx_message.len = tx_message_size;
	xfer->rx_len = (u8)rx_message_size;

	reinit_completion(&xfer->done);

	hdr->seq = xfer_id;
	hdr->type = msg_type;
	hdr->host = info->host_id;
	hdr->flags = msg_flags;

	return xfer;
}

/**
 * ti_sci_put_one_xfer() - Release a message
 * @minfo:	transfer info pointer
 * @xfer:	message that was reserved by ti_sci_get_one_xfer
 *
 * This holds a spinlock to maintain integrity of internal data structures.
 */
static void ti_sci_put_one_xfer(struct ti_sci_xfers_info *minfo,
				struct ti_sci_xfer *xfer)
{
	unsigned long flags;
	struct ti_sci_msg_hdr *hdr;
	u8 xfer_id;

	hdr = (struct ti_sci_msg_hdr *)xfer->tx_message.buf;
	xfer_id = hdr->seq;

	/*
	 * Keep the locked section as small as possible
	 * NOTE: we might escape with smp_mb and no lock here..
	 * but just be conservative and symmetric.
	 */
	spin_lock_irqsave(&minfo->xfer_lock, flags);
	clear_bit(xfer_id, minfo->xfer_alloc_table);
	spin_unlock_irqrestore(&minfo->xfer_lock, flags);

	/* Increment the count for the next user to get through */
	up(&minfo->sem_xfer_count);
}

/**
 * ti_sci_do_xfer() - Do one transfer
 * @info:	Pointer to SCI entity information
 * @xfer:	Transfer to initiate and wait for response
 *
 * Return: -ETIMEDOUT in case of no response, if transmit error,
 *	   return corresponding error, else if all goes well,
 *	   return 0.
 */
static inline int ti_sci_do_xfer(struct ti_sci_info *info,
				 struct ti_sci_xfer *xfer)
{
	int ret;
	int timeout;
	struct device *dev = info->dev;

	ret = mbox_send_message(info->chan_tx, &xfer->tx_message);
	if (ret < 0)
		return ret;

	ret = 0;

	/* And we wait for the response. */
	timeout = msecs_to_jiffies(info->desc->max_rx_timeout_ms);
	if (!wait_for_completion_timeout(&xfer->done, timeout)) {
		dev_err(dev, "Mbox timedout in resp(caller: %pF)\n",
			(void *)_RET_IP_);
		ret = -ETIMEDOUT;
	}
	/*
	 * NOTE: we might prefer not to need the mailbox ticker to manage the
	 * transfer queueing since the protocol layer queues things by itself.
	 * Unfortunately, we have to kick the mailbox framework after we have
	 * received our message.
	 */
	mbox_client_txdone(info->chan_tx, ret);

	return ret;
}

/**
 * ti_sci_cmd_get_revision() - command to get the revision of the SCI entity
 * @info:	Pointer to SCI entity information
 *
 * Updates the SCI information in the internal data structure.
 *
 * Return: 0 if all went fine, else return appropriate error.
 */
static int ti_sci_cmd_get_revision(struct ti_sci_info *info)
{
	struct device *dev = info->dev;
	struct ti_sci_handle *handle = &info->handle;
	struct ti_sci_version_info *ver = &handle->version;
	struct ti_sci_msg_resp_version *rev_info;
	struct ti_sci_xfer *xfer;
	int ret;

	/* No need to setup flags since it is expected to respond */
	xfer = ti_sci_get_one_xfer(info, TI_SCI_MSG_VERSION,
				   0x0, sizeof(struct ti_sci_msg_hdr),
				   sizeof(*rev_info));
	if (IS_ERR(xfer)) {
		ret = PTR_ERR(xfer);
		dev_err(dev, "Message alloc failed(%d)\n", ret);
		return ret;
	}

	rev_info = (struct ti_sci_msg_resp_version *)xfer->xfer_buf;

	ret = ti_sci_do_xfer(info, xfer);
	if (ret) {
		dev_err(dev, "Mbox send fail %d\n", ret);
		goto fail;
	}

	ver->abi_major = rev_info->abi_major;
	ver->abi_minor = rev_info->abi_minor;
	ver->firmware_revision = rev_info->firmware_revision;
	strncpy(ver->firmware_description, rev_info->firmware_description,
		sizeof(ver->firmware_description));

fail:
	ti_sci_put_one_xfer(&info->minfo, xfer);
	return ret;
}

/**
 * ti_sci_is_response_ack() - Generic ACK/NACK message checkup
 * @r:	pointer to response buffer
 *
 * Return: true if the response was an ACK, else returns false.
 */
static inline bool ti_sci_is_response_ack(void *r)
{
	struct ti_sci_msg_hdr *hdr = r;

	return hdr->flags & TI_SCI_FLAG_RESP_GENERIC_ACK ? true : false;
}

/**
 * ti_sci_set_device_state() - Set device state helper
 * @handle:	pointer to TI SCI handle
 * @id:		Device identifier
 * @flags:	flags to setup for the device
 * @state:	State to move the device to
 *
 * Return: 0 if all went well, else returns appropriate error value.
 */
static int ti_sci_set_device_state(const struct ti_sci_handle *handle,
				   u32 id, u32 flags, u8 state)
{
	struct ti_sci_info *info;
	struct ti_sci_msg_req_set_device_state *req;
	struct ti_sci_msg_hdr *resp;
	struct ti_sci_xfer *xfer;
	struct device *dev;
	int ret = 0;

	if (IS_ERR(handle))
		return PTR_ERR(handle);
	if (!handle)
		return -EINVAL;

	info = handle_to_ti_sci_info(handle);
	dev = info->dev;

	xfer = ti_sci_get_one_xfer(info, TI_SCI_MSG_SET_DEVICE_STATE,
				   flags | TI_SCI_FLAG_REQ_ACK_ON_PROCESSED,
				   sizeof(*req), sizeof(*resp));
	if (IS_ERR(xfer)) {
		ret = PTR_ERR(xfer);
		dev_err(dev, "Message alloc failed(%d)\n", ret);
		return ret;
	}
	req = (struct ti_sci_msg_req_set_device_state *)xfer->xfer_buf;
	req->id = id;
	req->state = state;

	ret = ti_sci_do_xfer(info, xfer);
	if (ret) {
		dev_err(dev, "Mbox send fail %d\n", ret);
		goto fail;
	}

	resp = (struct ti_sci_msg_hdr *)xfer->xfer_buf;

	ret = ti_sci_is_response_ack(resp) ? 0 : -ENODEV;

fail:
	ti_sci_put_one_xfer(&info->minfo, xfer);

	return ret;
}

/**
 * ti_sci_get_device_state() - Get device state helper
 * @handle:	Handle to the device
 * @id:		Device Identifier
 * @clcnt:	Pointer to Context Loss Count
 * @resets:	pointer to resets
 * @p_state:	pointer to p_state
 * @c_state:	pointer to c_state
 *
 * Return: 0 if all went fine, else return appropriate error.
 */
static int ti_sci_get_device_state(const struct ti_sci_handle *handle,
				   u32 id,  u32 *clcnt,  u32 *resets,
				    u8 *p_state,  u8 *c_state)
{
	struct ti_sci_info *info;
	struct ti_sci_msg_req_get_device_state *req;
	struct ti_sci_msg_resp_get_device_state *resp;
	struct ti_sci_xfer *xfer;
	struct device *dev;
	int ret = 0;

	if (IS_ERR(handle))
		return PTR_ERR(handle);
	if (!handle)
		return -EINVAL;

	if (!clcnt && !resets && !p_state && !c_state)
		return -EINVAL;

	info = handle_to_ti_sci_info(handle);
	dev = info->dev;

	/* Response is expected, so need of any flags */
	xfer = ti_sci_get_one_xfer(info, TI_SCI_MSG_GET_DEVICE_STATE,
				   0, sizeof(*req), sizeof(*resp));
	if (IS_ERR(xfer)) {
		ret = PTR_ERR(xfer);
		dev_err(dev, "Message alloc failed(%d)\n", ret);
		return ret;
	}
	req = (struct ti_sci_msg_req_get_device_state *)xfer->xfer_buf;
	req->id = id;

	ret = ti_sci_do_xfer(info, xfer);
	if (ret) {
		dev_err(dev, "Mbox send fail %d\n", ret);
		goto fail;
	}

	resp = (struct ti_sci_msg_resp_get_device_state *)xfer->xfer_buf;
	if (!ti_sci_is_response_ack(resp)) {
		ret = -ENODEV;
		goto fail;
	}

	if (clcnt)
		*clcnt = resp->context_loss_count;
	if (resets)
		*resets = resp->resets;
	if (p_state)
		*p_state = resp->programmed_state;
	if (c_state)
		*c_state = resp->current_state;
fail:
	ti_sci_put_one_xfer(&info->minfo, xfer);

	return ret;
}

/**
 * ti_sci_cmd_get_device() - command to request for device managed by TISCI
 * @handle:	Pointer to TISCI handle as retrieved by *ti_sci_get_handle
 * @id:		Device Identifier
 *
 * Request for the device - NOTE: the client MUST maintain integrity of
 * usage count by balancing get_device with put_device. No refcounting is
 * managed by driver for that purpose.
 *
 * NOTE: The request is for exclusive access for the processor.
 *
 * Return: 0 if all went fine, else return appropriate error.
 */
static int ti_sci_cmd_get_device(const struct ti_sci_handle *handle, u32 id)
{
	return ti_sci_set_device_state(handle, id,
				       MSG_FLAG_DEVICE_EXCLUSIVE,
				       MSG_DEVICE_SW_STATE_ON);
}

/**
 * ti_sci_cmd_idle_device() - Command to idle a device managed by TISCI
 * @handle:	Pointer to TISCI handle as retrieved by *ti_sci_get_handle
 * @id:		Device Identifier
 *
 * Request for the device - NOTE: the client MUST maintain integrity of
 * usage count by balancing get_device with put_device. No refcounting is
 * managed by driver for that purpose.
 *
 * Return: 0 if all went fine, else return appropriate error.
 */
static int ti_sci_cmd_idle_device(const struct ti_sci_handle *handle, u32 id)
{
	return ti_sci_set_device_state(handle, id,
				       MSG_FLAG_DEVICE_EXCLUSIVE,
				       MSG_DEVICE_SW_STATE_RETENTION);
}

/**
 * ti_sci_cmd_put_device() - command to release a device managed by TISCI
 * @handle:	Pointer to TISCI handle as retrieved by *ti_sci_get_handle
 * @id:		Device Identifier
 *
 * Request for the device - NOTE: the client MUST maintain integrity of
 * usage count by balancing get_device with put_device. No refcounting is
 * managed by driver for that purpose.
 *
 * Return: 0 if all went fine, else return appropriate error.
 */
static int ti_sci_cmd_put_device(const struct ti_sci_handle *handle, u32 id)
{
	return ti_sci_set_device_state(handle, id,
				       0, MSG_DEVICE_SW_STATE_AUTO_OFF);
}

/**
 * ti_sci_cmd_dev_is_valid() - Is the device valid
 * @handle:	Pointer to TISCI handle as retrieved by *ti_sci_get_handle
 * @id:		Device Identifier
 *
 * Return: 0 if all went fine and the device ID is valid, else return
 * appropriate error.
 */
static int ti_sci_cmd_dev_is_valid(const struct ti_sci_handle *handle, u32 id)
{
	u8 unused;

	/* check the device state which will also tell us if the ID is valid */
	return ti_sci_get_device_state(handle, id, NULL, NULL, NULL, &unused);
}

/**
 * ti_sci_cmd_dev_get_clcnt() - Get context loss counter
 * @handle:	Pointer to TISCI handle
 * @id:		Device Identifier
 * @count:	Pointer to Context Loss counter to populate
 *
 * Return: 0 if all went fine, else return appropriate error.
 */
static int ti_sci_cmd_dev_get_clcnt(const struct ti_sci_handle *handle, u32 id,
				    u32 *count)
{
	return ti_sci_get_device_state(handle, id, count, NULL, NULL, NULL);
}

/**
 * ti_sci_cmd_dev_is_idle() - Check if the device is requested to be idle
 * @handle:	Pointer to TISCI handle
 * @id:		Device Identifier
 * @r_state:	true if requested to be idle
 *
 * Return: 0 if all went fine, else return appropriate error.
 */
static int ti_sci_cmd_dev_is_idle(const struct ti_sci_handle *handle, u32 id,
				  bool *r_state)
{
	int ret;
	u8 state;

	if (!r_state)
		return -EINVAL;

	ret = ti_sci_get_device_state(handle, id, NULL, NULL, &state, NULL);
	if (ret)
		return ret;

	*r_state = (state == MSG_DEVICE_SW_STATE_RETENTION);

	return 0;
}

/**
 * ti_sci_cmd_dev_is_stop() - Check if the device is requested to be stopped
 * @handle:	Pointer to TISCI handle
 * @id:		Device Identifier
 * @r_state:	true if requested to be stopped
 * @curr_state:	true if currently stopped.
 *
 * Return: 0 if all went fine, else return appropriate error.
 */
static int ti_sci_cmd_dev_is_stop(const struct ti_sci_handle *handle, u32 id,
				  bool *r_state,  bool *curr_state)
{
	int ret;
	u8 p_state, c_state;

	if (!r_state && !curr_state)
		return -EINVAL;

	ret =
	    ti_sci_get_device_state(handle, id, NULL, NULL, &p_state, &c_state);
	if (ret)
		return ret;

	if (r_state)
		*r_state = (p_state == MSG_DEVICE_SW_STATE_AUTO_OFF);
	if (curr_state)
		*curr_state = (c_state == MSG_DEVICE_HW_STATE_OFF);

	return 0;
}

/**
 * ti_sci_cmd_dev_is_on() - Check if the device is requested to be ON
 * @handle:	Pointer to TISCI handle
 * @id:		Device Identifier
 * @r_state:	true if requested to be ON
 * @curr_state:	true if currently ON and active
 *
 * Return: 0 if all went fine, else return appropriate error.
 */
static int ti_sci_cmd_dev_is_on(const struct ti_sci_handle *handle, u32 id,
				bool *r_state,  bool *curr_state)
{
	int ret;
	u8 p_state, c_state;

	if (!r_state && !curr_state)
		return -EINVAL;

	ret =
	    ti_sci_get_device_state(handle, id, NULL, NULL, &p_state, &c_state);
	if (ret)
		return ret;

	if (r_state)
		*r_state = (p_state == MSG_DEVICE_SW_STATE_ON);
	if (curr_state)
		*curr_state = (c_state == MSG_DEVICE_HW_STATE_ON);

	return 0;
}

/**
 * ti_sci_cmd_dev_is_trans() - Check if the device is currently transitioning
 * @handle:	Pointer to TISCI handle
 * @id:		Device Identifier
 * @curr_state:	true if currently transitioning.
 *
 * Return: 0 if all went fine, else return appropriate error.
 */
static int ti_sci_cmd_dev_is_trans(const struct ti_sci_handle *handle, u32 id,
				   bool *curr_state)
{
	int ret;
	u8 state;

	if (!curr_state)
		return -EINVAL;

	ret = ti_sci_get_device_state(handle, id, NULL, NULL, NULL, &state);
	if (ret)
		return ret;

	*curr_state = (state == MSG_DEVICE_HW_STATE_TRANS);

	return 0;
}

/**
 * ti_sci_cmd_set_device_resets() - command to set resets for device managed
 *				    by TISCI
 * @handle:	Pointer to TISCI handle as retrieved by *ti_sci_get_handle
 * @id:		Device Identifier
 * @reset_state: Device specific reset bit field
 *
 * Return: 0 if all went fine, else return appropriate error.
 */
static int ti_sci_cmd_set_device_resets(const struct ti_sci_handle *handle,
					u32 id, u32 reset_state)
{
	struct ti_sci_info *info;
	struct ti_sci_msg_req_set_device_resets *req;
	struct ti_sci_msg_hdr *resp;
	struct ti_sci_xfer *xfer;
	struct device *dev;
	int ret = 0;

	if (IS_ERR(handle))
		return PTR_ERR(handle);
	if (!handle)
		return -EINVAL;

	info = handle_to_ti_sci_info(handle);
	dev = info->dev;

	xfer = ti_sci_get_one_xfer(info, TI_SCI_MSG_SET_DEVICE_RESETS,
				   TI_SCI_FLAG_REQ_ACK_ON_PROCESSED,
				   sizeof(*req), sizeof(*resp));
	if (IS_ERR(xfer)) {
		ret = PTR_ERR(xfer);
		dev_err(dev, "Message alloc failed(%d)\n", ret);
		return ret;
	}
	req = (struct ti_sci_msg_req_set_device_resets *)xfer->xfer_buf;
	req->id = id;
	req->resets = reset_state;

	ret = ti_sci_do_xfer(info, xfer);
	if (ret) {
		dev_err(dev, "Mbox send fail %d\n", ret);
		goto fail;
	}

	resp = (struct ti_sci_msg_hdr *)xfer->xfer_buf;

	ret = ti_sci_is_response_ack(resp) ? 0 : -ENODEV;

fail:
	ti_sci_put_one_xfer(&info->minfo, xfer);

	return ret;
}

/**
 * ti_sci_cmd_get_device_resets() - Get reset state for device managed
 *				    by TISCI
 * @handle:		Pointer to TISCI handle
 * @id:			Device Identifier
 * @reset_state:	Pointer to reset state to populate
 *
 * Return: 0 if all went fine, else return appropriate error.
 */
static int ti_sci_cmd_get_device_resets(const struct ti_sci_handle *handle,
					u32 id, u32 *reset_state)
{
	return ti_sci_get_device_state(handle, id, NULL, reset_state, NULL,
				       NULL);
}

/**
 * ti_sci_set_clock_state() - Set clock state helper
 * @handle:	pointer to TI SCI handle
 * @dev_id:	Device identifier this request is for
 * @clk_id:	Clock identifier for the device for this request.
 *		Each device has it's own set of clock inputs. This indexes
 *		which clock input to modify.
 * @flags:	Header flags as needed
 * @state:	State to request for the clock.
 *
 * Return: 0 if all went well, else returns appropriate error value.
 */
static int ti_sci_set_clock_state(const struct ti_sci_handle *handle,
				  u32 dev_id, u8 clk_id,
				  u32 flags, u8 state)
{
	struct ti_sci_info *info;
	struct ti_sci_msg_req_set_clock_state *req;
	struct ti_sci_msg_hdr *resp;
	struct ti_sci_xfer *xfer;
	struct device *dev;
	int ret = 0;

	if (IS_ERR(handle))
		return PTR_ERR(handle);
	if (!handle)
		return -EINVAL;

	info = handle_to_ti_sci_info(handle);
	dev = info->dev;

	xfer = ti_sci_get_one_xfer(info, TI_SCI_MSG_SET_CLOCK_STATE,
				   flags | TI_SCI_FLAG_REQ_ACK_ON_PROCESSED,
				   sizeof(*req), sizeof(*resp));
	if (IS_ERR(xfer)) {
		ret = PTR_ERR(xfer);
		dev_err(dev, "Message alloc failed(%d)\n", ret);
		return ret;
	}
	req = (struct ti_sci_msg_req_set_clock_state *)xfer->xfer_buf;
	req->dev_id = dev_id;
	req->clk_id = clk_id;
	req->request_state = state;

	ret = ti_sci_do_xfer(info, xfer);
	if (ret) {
		dev_err(dev, "Mbox send fail %d\n", ret);
		goto fail;
	}

	resp = (struct ti_sci_msg_hdr *)xfer->xfer_buf;

	ret = ti_sci_is_response_ack(resp) ? 0 : -ENODEV;

fail:
	ti_sci_put_one_xfer(&info->minfo, xfer);

	return ret;
}

/**
 * ti_sci_cmd_get_clock_state() - Get clock state helper
 * @handle:	pointer to TI SCI handle
 * @dev_id:	Device identifier this request is for
 * @clk_id:	Clock identifier for the device for this request.
 *		Each device has it's own set of clock inputs. This indexes
 *		which clock input to modify.
 * @programmed_state:	State requested for clock to move to
 * @current_state:	State that the clock is currently in
 *
 * Return: 0 if all went well, else returns appropriate error value.
 */
static int ti_sci_cmd_get_clock_state(const struct ti_sci_handle *handle,
				      u32 dev_id, u8 clk_id,
				      u8 *programmed_state, u8 *current_state)
{
	struct ti_sci_info *info;
	struct ti_sci_msg_req_get_clock_state *req;
	struct ti_sci_msg_resp_get_clock_state *resp;
	struct ti_sci_xfer *xfer;
	struct device *dev;
	int ret = 0;

	if (IS_ERR(handle))
		return PTR_ERR(handle);
	if (!handle)
		return -EINVAL;

	if (!programmed_state && !current_state)
		return -EINVAL;

	info = handle_to_ti_sci_info(handle);
	dev = info->dev;

	xfer = ti_sci_get_one_xfer(info, TI_SCI_MSG_GET_CLOCK_STATE,
				   TI_SCI_FLAG_REQ_ACK_ON_PROCESSED,
				   sizeof(*req), sizeof(*resp));
	if (IS_ERR(xfer)) {
		ret = PTR_ERR(xfer);
		dev_err(dev, "Message alloc failed(%d)\n", ret);
		return ret;
	}
	req = (struct ti_sci_msg_req_get_clock_state *)xfer->xfer_buf;
	req->dev_id = dev_id;
	req->clk_id = clk_id;

	ret = ti_sci_do_xfer(info, xfer);
	if (ret) {
		dev_err(dev, "Mbox send fail %d\n", ret);
		goto fail;
	}

	resp = (struct ti_sci_msg_resp_get_clock_state *)xfer->xfer_buf;

	if (!ti_sci_is_response_ack(resp)) {
		ret = -ENODEV;
		goto fail;
	}

	if (programmed_state)
		*programmed_state = resp->programmed_state;
	if (current_state)
		*current_state = resp->current_state;

fail:
	ti_sci_put_one_xfer(&info->minfo, xfer);

	return ret;
}

/**
 * ti_sci_cmd_get_clock() - Get control of a clock from TI SCI
 * @handle:	pointer to TI SCI handle
 * @dev_id:	Device identifier this request is for
 * @clk_id:	Clock identifier for the device for this request.
 *		Each device has it's own set of clock inputs. This indexes
 *		which clock input to modify.
 * @needs_ssc: 'true' if Spread Spectrum clock is desired, else 'false'
 * @can_change_freq: 'true' if frequency change is desired, else 'false'
 * @enable_input_term: 'true' if input termination is desired, else 'false'
 *
 * Return: 0 if all went well, else returns appropriate error value.
 */
static int ti_sci_cmd_get_clock(const struct ti_sci_handle *handle, u32 dev_id,
				u8 clk_id, bool needs_ssc, bool can_change_freq,
				bool enable_input_term)
{
	u32 flags = 0;

	flags |= needs_ssc ? MSG_FLAG_CLOCK_ALLOW_SSC : 0;
	flags |= can_change_freq ? MSG_FLAG_CLOCK_ALLOW_FREQ_CHANGE : 0;
	flags |= enable_input_term ? MSG_FLAG_CLOCK_INPUT_TERM : 0;

	return ti_sci_set_clock_state(handle, dev_id, clk_id, flags,
				      MSG_CLOCK_SW_STATE_REQ);
}

/**
 * ti_sci_cmd_idle_clock() - Idle a clock which is in our control
 * @handle:	pointer to TI SCI handle
 * @dev_id:	Device identifier this request is for
 * @clk_id:	Clock identifier for the device for this request.
 *		Each device has it's own set of clock inputs. This indexes
 *		which clock input to modify.
 *
 * NOTE: This clock must have been requested by get_clock previously.
 *
 * Return: 0 if all went well, else returns appropriate error value.
 */
static int ti_sci_cmd_idle_clock(const struct ti_sci_handle *handle,
				 u32 dev_id, u8 clk_id)
{
	return ti_sci_set_clock_state(handle, dev_id, clk_id, 0,
				      MSG_CLOCK_SW_STATE_UNREQ);
}

/**
 * ti_sci_cmd_put_clock() - Release a clock from our control back to TISCI
 * @handle:	pointer to TI SCI handle
 * @dev_id:	Device identifier this request is for
 * @clk_id:	Clock identifier for the device for this request.
 *		Each device has it's own set of clock inputs. This indexes
 *		which clock input to modify.
 *
 * NOTE: This clock must have been requested by get_clock previously.
 *
 * Return: 0 if all went well, else returns appropriate error value.
 */
static int ti_sci_cmd_put_clock(const struct ti_sci_handle *handle,
				u32 dev_id, u8 clk_id)
{
	return ti_sci_set_clock_state(handle, dev_id, clk_id, 0,
				      MSG_CLOCK_SW_STATE_AUTO);
}

/**
 * ti_sci_cmd_clk_is_auto() - Is the clock being auto managed
 * @handle:	pointer to TI SCI handle
 * @dev_id:	Device identifier this request is for
 * @clk_id:	Clock identifier for the device for this request.
 *		Each device has it's own set of clock inputs. This indexes
 *		which clock input to modify.
 * @req_state: state indicating if the clock is auto managed
 *
 * Return: 0 if all went well, else returns appropriate error value.
 */
static int ti_sci_cmd_clk_is_auto(const struct ti_sci_handle *handle,
				  u32 dev_id, u8 clk_id, bool *req_state)
{
	u8 state = 0;
	int ret;

	if (!req_state)
		return -EINVAL;

	ret = ti_sci_cmd_get_clock_state(handle, dev_id, clk_id, &state, NULL);
	if (ret)
		return ret;

	*req_state = (state == MSG_CLOCK_SW_STATE_AUTO);
	return 0;
}

/**
 * ti_sci_cmd_clk_is_on() - Is the clock ON
 * @handle:	pointer to TI SCI handle
 * @dev_id:	Device identifier this request is for
 * @clk_id:	Clock identifier for the device for this request.
 *		Each device has it's own set of clock inputs. This indexes
 *		which clock input to modify.
 * @req_state: state indicating if the clock is managed by us and enabled
 * @curr_state: state indicating if the clock is ready for operation
 *
 * Return: 0 if all went well, else returns appropriate error value.
 */
static int ti_sci_cmd_clk_is_on(const struct ti_sci_handle *handle, u32 dev_id,
				u8 clk_id, bool *req_state, bool *curr_state)
{
	u8 c_state = 0, r_state = 0;
	int ret;

	if (!req_state && !curr_state)
		return -EINVAL;

	ret = ti_sci_cmd_get_clock_state(handle, dev_id, clk_id,
					 &r_state, &c_state);
	if (ret)
		return ret;

	if (req_state)
		*req_state = (r_state == MSG_CLOCK_SW_STATE_REQ);
	if (curr_state)
		*curr_state = (c_state == MSG_CLOCK_HW_STATE_READY);
	return 0;
}

/**
 * ti_sci_cmd_clk_is_off() - Is the clock OFF
 * @handle:	pointer to TI SCI handle
 * @dev_id:	Device identifier this request is for
 * @clk_id:	Clock identifier for the device for this request.
 *		Each device has it's own set of clock inputs. This indexes
 *		which clock input to modify.
 * @req_state: state indicating if the clock is managed by us and disabled
 * @curr_state: state indicating if the clock is NOT ready for operation
 *
 * Return: 0 if all went well, else returns appropriate error value.
 */
static int ti_sci_cmd_clk_is_off(const struct ti_sci_handle *handle, u32 dev_id,
				 u8 clk_id, bool *req_state, bool *curr_state)
{
	u8 c_state = 0, r_state = 0;
	int ret;

	if (!req_state && !curr_state)
		return -EINVAL;

	ret = ti_sci_cmd_get_clock_state(handle, dev_id, clk_id,
					 &r_state, &c_state);
	if (ret)
		return ret;

	if (req_state)
		*req_state = (r_state == MSG_CLOCK_SW_STATE_UNREQ);
	if (curr_state)
		*curr_state = (c_state == MSG_CLOCK_HW_STATE_NOT_READY);
	return 0;
}

/**
 * ti_sci_cmd_clk_set_parent() - Set the clock source of a specific device clock
 * @handle:	pointer to TI SCI handle
 * @dev_id:	Device identifier this request is for
 * @clk_id:	Clock identifier for the device for this request.
 *		Each device has it's own set of clock inputs. This indexes
 *		which clock input to modify.
 * @parent_id:	Parent clock identifier to set
 *
 * Return: 0 if all went well, else returns appropriate error value.
 */
static int ti_sci_cmd_clk_set_parent(const struct ti_sci_handle *handle,
				     u32 dev_id, u8 clk_id, u8 parent_id)
{
	struct ti_sci_info *info;
	struct ti_sci_msg_req_set_clock_parent *req;
	struct ti_sci_msg_hdr *resp;
	struct ti_sci_xfer *xfer;
	struct device *dev;
	int ret = 0;

	if (IS_ERR(handle))
		return PTR_ERR(handle);
	if (!handle)
		return -EINVAL;

	info = handle_to_ti_sci_info(handle);
	dev = info->dev;

	xfer = ti_sci_get_one_xfer(info, TI_SCI_MSG_SET_CLOCK_PARENT,
				   TI_SCI_FLAG_REQ_ACK_ON_PROCESSED,
				   sizeof(*req), sizeof(*resp));
	if (IS_ERR(xfer)) {
		ret = PTR_ERR(xfer);
		dev_err(dev, "Message alloc failed(%d)\n", ret);
		return ret;
	}
	req = (struct ti_sci_msg_req_set_clock_parent *)xfer->xfer_buf;
	req->dev_id = dev_id;
	req->clk_id = clk_id;
	req->parent_id = parent_id;

	ret = ti_sci_do_xfer(info, xfer);
	if (ret) {
		dev_err(dev, "Mbox send fail %d\n", ret);
		goto fail;
	}

	resp = (struct ti_sci_msg_hdr *)xfer->xfer_buf;

	ret = ti_sci_is_response_ack(resp) ? 0 : -ENODEV;

fail:
	ti_sci_put_one_xfer(&info->minfo, xfer);

	return ret;
}

/**
 * ti_sci_cmd_clk_get_parent() - Get current parent clock source
 * @handle:	pointer to TI SCI handle
 * @dev_id:	Device identifier this request is for
 * @clk_id:	Clock identifier for the device for this request.
 *		Each device has it's own set of clock inputs. This indexes
 *		which clock input to modify.
 * @parent_id:	Current clock parent
 *
 * Return: 0 if all went well, else returns appropriate error value.
 */
static int ti_sci_cmd_clk_get_parent(const struct ti_sci_handle *handle,
				     u32 dev_id, u8 clk_id, u8 *parent_id)
{
	struct ti_sci_info *info;
	struct ti_sci_msg_req_get_clock_parent *req;
	struct ti_sci_msg_resp_get_clock_parent *resp;
	struct ti_sci_xfer *xfer;
	struct device *dev;
	int ret = 0;

	if (IS_ERR(handle))
		return PTR_ERR(handle);
	if (!handle || !parent_id)
		return -EINVAL;

	info = handle_to_ti_sci_info(handle);
	dev = info->dev;

	xfer = ti_sci_get_one_xfer(info, TI_SCI_MSG_GET_CLOCK_PARENT,
				   TI_SCI_FLAG_REQ_ACK_ON_PROCESSED,
				   sizeof(*req), sizeof(*resp));
	if (IS_ERR(xfer)) {
		ret = PTR_ERR(xfer);
		dev_err(dev, "Message alloc failed(%d)\n", ret);
		return ret;
	}
	req = (struct ti_sci_msg_req_get_clock_parent *)xfer->xfer_buf;
	req->dev_id = dev_id;
	req->clk_id = clk_id;

	ret = ti_sci_do_xfer(info, xfer);
	if (ret) {
		dev_err(dev, "Mbox send fail %d\n", ret);
		goto fail;
	}

	resp = (struct ti_sci_msg_resp_get_clock_parent *)xfer->xfer_buf;

	if (!ti_sci_is_response_ack(resp))
		ret = -ENODEV;
	else
		*parent_id = resp->parent_id;

fail:
	ti_sci_put_one_xfer(&info->minfo, xfer);

	return ret;
}

/**
 * ti_sci_cmd_clk_get_num_parents() - Get num parents of the current clk source
 * @handle:	pointer to TI SCI handle
 * @dev_id:	Device identifier this request is for
 * @clk_id:	Clock identifier for the device for this request.
 *		Each device has it's own set of clock inputs. This indexes
 *		which clock input to modify.
 * @num_parents: Returns he number of parents to the current clock.
 *
 * Return: 0 if all went well, else returns appropriate error value.
 */
static int ti_sci_cmd_clk_get_num_parents(const struct ti_sci_handle *handle,
					  u32 dev_id, u8 clk_id,
					  u8 *num_parents)
{
	struct ti_sci_info *info;
	struct ti_sci_msg_req_get_clock_num_parents *req;
	struct ti_sci_msg_resp_get_clock_num_parents *resp;
	struct ti_sci_xfer *xfer;
	struct device *dev;
	int ret = 0;

	if (IS_ERR(handle))
		return PTR_ERR(handle);
	if (!handle || !num_parents)
		return -EINVAL;

	info = handle_to_ti_sci_info(handle);
	dev = info->dev;

	xfer = ti_sci_get_one_xfer(info, TI_SCI_MSG_GET_NUM_CLOCK_PARENTS,
				   TI_SCI_FLAG_REQ_ACK_ON_PROCESSED,
				   sizeof(*req), sizeof(*resp));
	if (IS_ERR(xfer)) {
		ret = PTR_ERR(xfer);
		dev_err(dev, "Message alloc failed(%d)\n", ret);
		return ret;
	}
	req = (struct ti_sci_msg_req_get_clock_num_parents *)xfer->xfer_buf;
	req->dev_id = dev_id;
	req->clk_id = clk_id;

	ret = ti_sci_do_xfer(info, xfer);
	if (ret) {
		dev_err(dev, "Mbox send fail %d\n", ret);
		goto fail;
	}

	resp = (struct ti_sci_msg_resp_get_clock_num_parents *)xfer->xfer_buf;

	if (!ti_sci_is_response_ack(resp))
		ret = -ENODEV;
	else
		*num_parents = resp->num_parents;

fail:
	ti_sci_put_one_xfer(&info->minfo, xfer);

	return ret;
}

/**
 * ti_sci_cmd_clk_get_match_freq() - Find a good match for frequency
 * @handle:	pointer to TI SCI handle
 * @dev_id:	Device identifier this request is for
 * @clk_id:	Clock identifier for the device for this request.
 *		Each device has it's own set of clock inputs. This indexes
 *		which clock input to modify.
 * @min_freq:	The minimum allowable frequency in Hz. This is the minimum
 *		allowable programmed frequency and does not account for clock
 *		tolerances and jitter.
 * @target_freq: The target clock frequency in Hz. A frequency will be
 *		processed as close to this target frequency as possible.
 * @max_freq:	The maximum allowable frequency in Hz. This is the maximum
 *		allowable programmed frequency and does not account for clock
 *		tolerances and jitter.
 * @match_freq:	Frequency match in Hz response.
 *
 * Return: 0 if all went well, else returns appropriate error value.
 */
static int ti_sci_cmd_clk_get_match_freq(const struct ti_sci_handle *handle,
					 u32 dev_id, u8 clk_id, u64 min_freq,
					 u64 target_freq, u64 max_freq,
					 u64 *match_freq)
{
	struct ti_sci_info *info;
	struct ti_sci_msg_req_query_clock_freq *req;
	struct ti_sci_msg_resp_query_clock_freq *resp;
	struct ti_sci_xfer *xfer;
	struct device *dev;
	int ret = 0;

	if (IS_ERR(handle))
		return PTR_ERR(handle);
	if (!handle || !match_freq)
		return -EINVAL;

	info = handle_to_ti_sci_info(handle);
	dev = info->dev;

	xfer = ti_sci_get_one_xfer(info, TI_SCI_MSG_QUERY_CLOCK_FREQ,
				   TI_SCI_FLAG_REQ_ACK_ON_PROCESSED,
				   sizeof(*req), sizeof(*resp));
	if (IS_ERR(xfer)) {
		ret = PTR_ERR(xfer);
		dev_err(dev, "Message alloc failed(%d)\n", ret);
		return ret;
	}
	req = (struct ti_sci_msg_req_query_clock_freq *)xfer->xfer_buf;
	req->dev_id = dev_id;
	req->clk_id = clk_id;
	req->min_freq_hz = min_freq;
	req->target_freq_hz = target_freq;
	req->max_freq_hz = max_freq;

	ret = ti_sci_do_xfer(info, xfer);
	if (ret) {
		dev_err(dev, "Mbox send fail %d\n", ret);
		goto fail;
	}

	resp = (struct ti_sci_msg_resp_query_clock_freq *)xfer->xfer_buf;

	if (!ti_sci_is_response_ack(resp))
		ret = -ENODEV;
	else
		*match_freq = resp->freq_hz;

fail:
	ti_sci_put_one_xfer(&info->minfo, xfer);

	return ret;
}

/**
 * ti_sci_cmd_clk_set_freq() - Set a frequency for clock
 * @handle:	pointer to TI SCI handle
 * @dev_id:	Device identifier this request is for
 * @clk_id:	Clock identifier for the device for this request.
 *		Each device has it's own set of clock inputs. This indexes
 *		which clock input to modify.
 * @min_freq:	The minimum allowable frequency in Hz. This is the minimum
 *		allowable programmed frequency and does not account for clock
 *		tolerances and jitter.
 * @target_freq: The target clock frequency in Hz. A frequency will be
 *		processed as close to this target frequency as possible.
 * @max_freq:	The maximum allowable frequency in Hz. This is the maximum
 *		allowable programmed frequency and does not account for clock
 *		tolerances and jitter.
 *
 * Return: 0 if all went well, else returns appropriate error value.
 */
static int ti_sci_cmd_clk_set_freq(const struct ti_sci_handle *handle,
				   u32 dev_id, u8 clk_id, u64 min_freq,
				   u64 target_freq, u64 max_freq)
{
	struct ti_sci_info *info;
	struct ti_sci_msg_req_set_clock_freq *req;
	struct ti_sci_msg_hdr *resp;
	struct ti_sci_xfer *xfer;
	struct device *dev;
	int ret = 0;

	if (IS_ERR(handle))
		return PTR_ERR(handle);
	if (!handle)
		return -EINVAL;

	info = handle_to_ti_sci_info(handle);
	dev = info->dev;

	xfer = ti_sci_get_one_xfer(info, TI_SCI_MSG_SET_CLOCK_FREQ,
				   TI_SCI_FLAG_REQ_ACK_ON_PROCESSED,
				   sizeof(*req), sizeof(*resp));
	if (IS_ERR(xfer)) {
		ret = PTR_ERR(xfer);
		dev_err(dev, "Message alloc failed(%d)\n", ret);
		return ret;
	}
	req = (struct ti_sci_msg_req_set_clock_freq *)xfer->xfer_buf;
	req->dev_id = dev_id;
	req->clk_id = clk_id;
	req->min_freq_hz = min_freq;
	req->target_freq_hz = target_freq;
	req->max_freq_hz = max_freq;

	ret = ti_sci_do_xfer(info, xfer);
	if (ret) {
		dev_err(dev, "Mbox send fail %d\n", ret);
		goto fail;
	}

	resp = (struct ti_sci_msg_hdr *)xfer->xfer_buf;

	ret = ti_sci_is_response_ack(resp) ? 0 : -ENODEV;

fail:
	ti_sci_put_one_xfer(&info->minfo, xfer);

	return ret;
}

/**
 * ti_sci_cmd_clk_get_freq() - Get current frequency
 * @handle:	pointer to TI SCI handle
 * @dev_id:	Device identifier this request is for
 * @clk_id:	Clock identifier for the device for this request.
 *		Each device has it's own set of clock inputs. This indexes
 *		which clock input to modify.
 * @freq:	Currently frequency in Hz
 *
 * Return: 0 if all went well, else returns appropriate error value.
 */
static int ti_sci_cmd_clk_get_freq(const struct ti_sci_handle *handle,
				   u32 dev_id, u8 clk_id, u64 *freq)
{
	struct ti_sci_info *info;
	struct ti_sci_msg_req_get_clock_freq *req;
	struct ti_sci_msg_resp_get_clock_freq *resp;
	struct ti_sci_xfer *xfer;
	struct device *dev;
	int ret = 0;

	if (IS_ERR(handle))
		return PTR_ERR(handle);
	if (!handle || !freq)
		return -EINVAL;

	info = handle_to_ti_sci_info(handle);
	dev = info->dev;

	xfer = ti_sci_get_one_xfer(info, TI_SCI_MSG_GET_CLOCK_FREQ,
				   TI_SCI_FLAG_REQ_ACK_ON_PROCESSED,
				   sizeof(*req), sizeof(*resp));
	if (IS_ERR(xfer)) {
		ret = PTR_ERR(xfer);
		dev_err(dev, "Message alloc failed(%d)\n", ret);
		return ret;
	}
	req = (struct ti_sci_msg_req_get_clock_freq *)xfer->xfer_buf;
	req->dev_id = dev_id;
	req->clk_id = clk_id;

	ret = ti_sci_do_xfer(info, xfer);
	if (ret) {
		dev_err(dev, "Mbox send fail %d\n", ret);
		goto fail;
	}

	resp = (struct ti_sci_msg_resp_get_clock_freq *)xfer->xfer_buf;

	if (!ti_sci_is_response_ack(resp))
		ret = -ENODEV;
	else
		*freq = resp->freq_hz;

fail:
	ti_sci_put_one_xfer(&info->minfo, xfer);

	return ret;
}

static int ti_sci_cmd_core_reboot(const struct ti_sci_handle *handle)
{
	struct ti_sci_info *info;
	struct ti_sci_msg_req_reboot *req;
	struct ti_sci_msg_hdr *resp;
	struct ti_sci_xfer *xfer;
	struct device *dev;
	int ret = 0;

	if (IS_ERR(handle))
		return PTR_ERR(handle);
	if (!handle)
		return -EINVAL;

	info = handle_to_ti_sci_info(handle);
	dev = info->dev;

	xfer = ti_sci_get_one_xfer(info, TI_SCI_MSG_SYS_RESET,
				   TI_SCI_FLAG_REQ_ACK_ON_PROCESSED,
				   sizeof(*req), sizeof(*resp));
	if (IS_ERR(xfer)) {
		ret = PTR_ERR(xfer);
		dev_err(dev, "Message alloc failed(%d)\n", ret);
		return ret;
	}
	req = (struct ti_sci_msg_req_reboot *)xfer->xfer_buf;

	ret = ti_sci_do_xfer(info, xfer);
	if (ret) {
		dev_err(dev, "Mbox send fail %d\n", ret);
		goto fail;
	}

	resp = (struct ti_sci_msg_hdr *)xfer->xfer_buf;

	if (!ti_sci_is_response_ack(resp))
		ret = -ENODEV;
	else
		ret = 0;

fail:
	ti_sci_put_one_xfer(&info->minfo, xfer);

	return ret;
}

/**
 * ti_sci_cmd_set_irq() - Command to configure the route between the dev and
 *			  the host and get the host irq in response.
 * @handle:	Pointer to TISCI handle.
 * @s_host:	Host ID of the IRQ destination. A negative value
 *		can be passed if destination host id is same as the ti-sci
 *		interface host id.
 * @dev:	IRQ source peripheral ID.
 * @irq:	IRQ source offset within the peripheral.
 * @share:	If enabled, the resultant IRQ will be shared with other events.
 * @poll:	If non-zero, the configured IRQ will not result in a
 *		physical interrupt.
 * @group:	The requested IRQ will be added to this virtual interrupt group.
 *		In order to create a new virtual interrupt group this should be
 *		-1 along with @share field enabled.
 * @host_irq:	Input IRQ number to host interrupt controller.
 *
 * Return: 0 if all went fine, else return appropriate error.
 */
static int ti_sci_cmd_set_irq(const struct ti_sci_handle *handle, u8 s_host,
			      u16 dev_id, u16 irq, u8 share, u8 poll,
			      u32 *group, u32 *glb_evt, u32 *ia_id, u32 *vint,
			      u16 *host_irq, u8 *vint_status_bit)
{
	struct ti_sci_msg_resp_set_irq *resp;
	struct ti_sci_msg_req_set_irq *req;
	struct ti_sci_xfer *xfer;
	struct ti_sci_info *info;
	struct device *dev;
	int ret = 0;

	if (IS_ERR(handle))
		return PTR_ERR(handle);
	if (!handle)
		return -EINVAL;

	info = handle_to_ti_sci_info(handle);
	dev = info->dev;

	xfer = ti_sci_get_one_xfer(info, TI_SCI_MSG_SET_IRQ,
				   TI_SCI_FLAG_REQ_ACK_ON_PROCESSED,
				   sizeof(*req), sizeof(*resp));
	if (IS_ERR(xfer)) {
		ret = PTR_ERR(xfer);
		dev_err(dev, "Message alloc failed(%d)\n", ret);
		return ret;
	}
	req = (struct ti_sci_msg_req_set_irq *)xfer->xfer_buf;
	req->secondary_host = s_host;
	req->dev_id = dev_id;
	req->irq = irq;
	req->share = share;
	req->pool = poll;
	req->group = *group;

	ret = ti_sci_do_xfer(info, xfer);
	if (ret) {
		dev_err(dev, "Mbox send fail %d\n", ret);
		goto fail;
	}

	resp = (struct ti_sci_msg_resp_set_irq *)xfer->xfer_buf;

	if (!ti_sci_is_response_ack(resp)) {
		ret = -ENODEV;
	} else {
		*glb_evt = resp->global_event;
		*ia_id = resp->ia_id;
		*vint = resp->vint;
		*group = resp->group;
		*host_irq = resp->host_irq;
		*vint_status_bit = resp->vint_status_bit;
	};

fail:
	ti_sci_put_one_xfer(&info->minfo, xfer);

	return ret;
}

/**
 * ti_sci_cmd_free_irq() - Command to free host's IRQ from dev.
 * @handle:	Pointer to TISCI handle.
 * @s_host:	Host ID of the interrupt route destination. A negative value
 *		can be passed if destination host id is same as the ti-sci
 *		interface host id.
 * @dev_id:	Interrupt source peripheral ID.
 * @irq:	IRQ source within the peripheral.
 * @group:	virtual interrupt group ID. -1 can be passed if not applicable.
 *
 * Return: 0 if all went fine, else return appropriate error.
 */
static int ti_sci_cmd_free_irq(const struct ti_sci_handle *handle, u8 s_host,
			       u16 dev_id, u16 irq, u32 glb_evt, u32 group)
{
	struct ti_sci_msg_req_free_irq *req;
	struct ti_sci_msg_hdr *resp;
	struct ti_sci_xfer *xfer;
	struct ti_sci_info *info;
	struct device *dev;
	int ret = 0;

	if (IS_ERR(handle))
		return PTR_ERR(handle);
	if (!handle)
		return -EINVAL;

	info = handle_to_ti_sci_info(handle);
	dev = info->dev;

	xfer = ti_sci_get_one_xfer(info, TI_SCI_MSG_FREE_IRQ,
				   TI_SCI_FLAG_REQ_ACK_ON_PROCESSED,
				   sizeof(*req), sizeof(*resp));
	if (IS_ERR(xfer)) {
		ret = PTR_ERR(xfer);
		dev_err(dev, "Message alloc failed(%d)\n", ret);
		return ret;
	}
	req = (struct ti_sci_msg_req_free_irq *)xfer->xfer_buf;
	req->secondary_host = s_host;
	req->dev_id = dev_id;
	req->irq = irq;
	req->global_event = glb_evt;
	req->group = group;

	ret = ti_sci_do_xfer(info, xfer);
	if (ret) {
		dev_err(dev, "Mbox send fail %d\n", ret);
		goto fail;
	}

	resp = (struct ti_sci_msg_hdr *)xfer->xfer_buf;

	ret = ti_sci_is_response_ack(resp) ? 0 : -ENODEV;

fail:
	ti_sci_put_one_xfer(&info->minfo, xfer);

	return ret;
}

/**
 * ti_sci_cmd_ring_allocate() - Allocate RA ring
 * @handle:	pointer to TI SCI handle
 * @secondary_host: Specifies a host ID for which the TISCI header host ID
 *	is proxying the request for.
 * @nav_id: Device ID of Navigator Subsystem from which the ring is allocated
 * @index: Ring index.
 * @addr_lo: The ring base address lo 32 bits
 * @addr_hi: The ring base address hi 32 bits
 * @count: Number of ring elements.
 * @mode: The mode of the ring
 * @size: The ring element size.
 * @order_id: Specifies the ring's bus order ID.
 * @share: Specifies whether the allocated and configured ring is unshared,
 *	limited shared, or open shared.
 * @type: Specifies the ring type to be allocated for dynamic ring requests
 *
 * Return: 0 if all went well, else returns appropriate error value.
 *
 * See @ti_sci_msg_ring_allocate for more info.
 */
static int ti_sci_cmd_ring_allocate(
			      const struct ti_sci_handle *handle,
			      u8 s_host, u32 nav_id, u32 *index,
			      u32 addr_lo, u32 addr_hi,
			      u32 count, u8 mode, u8 size,
			      u8 order_id,
			      u8 share,
			      u16 type)
{
	struct ti_sci_msg_ring_allocate_resp *resp;
	struct ti_sci_msg_ring_allocate *req;
	struct ti_sci_xfer *xfer;
	struct ti_sci_info *info;
	struct device *dev;
	int ret = 0;

	if (IS_ERR(handle))
		return PTR_ERR(handle);
	if (!handle || !index)
		return -EINVAL;

	info = handle_to_ti_sci_info(handle);
	dev = info->dev;

	xfer = ti_sci_get_one_xfer(info, TI_SCI_MSG_RM_RING_ALLOCATE,
				   TI_SCI_FLAG_REQ_ACK_ON_PROCESSED,
				   sizeof(*req), sizeof(*resp));
	if (IS_ERR(xfer)) {
		ret = PTR_ERR(xfer);
		dev_err(dev, "RM_RA:Message alloc failed(%d)\n", ret);
		return ret;
	}
	req = (struct ti_sci_msg_ring_allocate *)xfer->xfer_buf;
	req->secondary_host = s_host;
	req->nav_id = nav_id;
	req->index = *index;
	req->addr_lo = addr_lo;
	req->addr_hi = addr_hi;
	req->count = count;
	req->mode = mode;
	req->size = size;
	req->order_id = order_id;
	req->share = share;
	req->type = type;

	ret = ti_sci_do_xfer(info, xfer);
	if (ret) {
		dev_err(dev, "RM_RA:Mbox send fail %d\n", ret);
		goto fail;
	}

	resp = (struct ti_sci_msg_ring_allocate_resp *)xfer->xfer_buf;

	if (!ti_sci_is_response_ack(resp))
		ret = -ENODEV;
	else
		*index = resp->index;

fail:
	ti_sci_put_one_xfer(&info->minfo, xfer);
	dev_dbg(dev, "RM_RA:allocate ring %u ret:%d\n", *index, ret);

	return ret;
}

/**
 * ti_sci_cmd_ring_free() - Free RA ring
 * @handle:	pointer to TI SCI handle
 * @secondary_host: Specifies a host ID for which the TISCI header host ID
 *	is proxying the request for.
 * @nav_id: Device ID of Navigator Subsystem from which the ring is allocated
 * @index: Ring index.
 *
 * Return: 0 if all went well, else returns appropriate error value.
 *
 * See @ti_sci_msg_ring_free for more info.
 */
static int ti_sci_cmd_ring_free(const struct ti_sci_handle *handle,
				u8 s_host, u32 nav_id, u32 index)
{
	struct ti_sci_msg_ring_free *req;
	struct ti_sci_msg_hdr *resp;
	struct ti_sci_xfer *xfer;
	struct ti_sci_info *info;
	struct device *dev;
	int ret = 0;

	if (IS_ERR(handle))
		return PTR_ERR(handle);
	if (!handle)
		return -EINVAL;

	info = handle_to_ti_sci_info(handle);
	dev = info->dev;

	xfer = ti_sci_get_one_xfer(info, TI_SCI_MSG_RM_RING_FREE,
				   TI_SCI_FLAG_REQ_ACK_ON_PROCESSED,
				   sizeof(*req), sizeof(*resp));
	if (IS_ERR(xfer)) {
		ret = PTR_ERR(xfer);
		dev_err(dev, "RM_RA:Message alloc failed(%d)\n", ret);
		return ret;
	}
	req = (struct ti_sci_msg_ring_free *)xfer->xfer_buf;
	req->secondary_host = s_host;
	req->nav_id = nav_id;
	req->index = index;

	ret = ti_sci_do_xfer(info, xfer);
	if (ret) {
		dev_err(dev, "RM_RA:Mbox send fail %d\n", ret);
		goto fail;
	}

	resp = (struct ti_sci_msg_hdr *)xfer->xfer_buf;

	ret = ti_sci_is_response_ack(resp) ? 0 : -ENODEV;

fail:
	ti_sci_put_one_xfer(&info->minfo, xfer);
	dev_dbg(dev, "RM_RA:free ring %u ret:%d\n", index, ret);

	return ret;
}

/**
 * ti_sci_cmd_ring_reset() - Reset RA ring
 * @handle:	pointer to TI SCI handle
 * @nav_id: Device ID of Navigator Subsystem from which the ring is allocated
 * @index: Ring index.
 *
 * Return: 0 if all went well, else returns appropriate error value.
 *
 * See @ti_sci_msg_ring_reset for more info.
 */
static int ti_sci_cmd_ring_reset(const struct ti_sci_handle *handle,
				 u32 nav_id, u32 index)
{
	struct ti_sci_msg_ring_reset *req;
	struct ti_sci_msg_hdr *resp;
	struct ti_sci_xfer *xfer;
	struct ti_sci_info *info;
	struct device *dev;
	int ret = 0;

	if (IS_ERR(handle))
		return PTR_ERR(handle);
	if (!handle)
		return -EINVAL;

	info = handle_to_ti_sci_info(handle);
	dev = info->dev;

	xfer = ti_sci_get_one_xfer(info, TI_SCI_MSG_RM_RING_RESET,
				   TI_SCI_FLAG_REQ_ACK_ON_PROCESSED,
				   sizeof(*req), sizeof(*resp));
	if (IS_ERR(xfer)) {
		ret = PTR_ERR(xfer);
		dev_err(dev, "RM_RA:Message alloc failed(%d)\n", ret);
		return ret;
	}
	req = (struct ti_sci_msg_ring_reset *)xfer->xfer_buf;
	req->nav_id = nav_id;
	req->index = index;

	ret = ti_sci_do_xfer(info, xfer);
	if (ret) {
		dev_err(dev, "RM_RA:Mbox send fail %d\n", ret);
		goto fail;
	}

	resp = (struct ti_sci_msg_hdr *)xfer->xfer_buf;

	ret = ti_sci_is_response_ack(resp) ? 0 : -ENODEV;

fail:
	ti_sci_put_one_xfer(&info->minfo, xfer);
	dev_dbg(dev, "RM_RA:reset ring %u ret:%d\n", index, ret);

	return ret;
}

/**
 * ti_sci_cmd_ring_reconfig() - Reconfigure qmode RA ring
 * @handle:	pointer to TI SCI handle
 * @nav_id: Device ID of Navigator Subsystem from which the ring is allocated
 * @index: Ring index.
 * @mode: The mode of the ring
 *
 * Return: 0 if all went well, else returns appropriate error value.
 *
 * See @ti_sci_msg_ring_reconfig for more info.
 */
static int ti_sci_cmd_ring_reconfig(
			      const struct ti_sci_handle *handle,
			      u32 nav_id, u32 index, u8 *mode,
			      u32 *addr_lo,
			      u32 *addr_hi,
			      u32 *count,
			      u8 *size,
			      u8 *order_id)
{
	struct ti_sci_msg_ring_reconfig_resp *resp;
	struct ti_sci_msg_ring_reconfig *req;
	struct ti_sci_xfer *xfer;
	struct ti_sci_info *info;
	struct device *dev;
	int ret = 0;

	if (IS_ERR(handle))
		return PTR_ERR(handle);
	if (!handle || !index)
		return -EINVAL;

	info = handle_to_ti_sci_info(handle);
	dev = info->dev;

	xfer = ti_sci_get_one_xfer(info, TI_SCI_MSG_RM_RING_RECONFIG,
				   TI_SCI_FLAG_REQ_ACK_ON_PROCESSED,
				   sizeof(*req), sizeof(*resp));
	if (IS_ERR(xfer)) {
		ret = PTR_ERR(xfer);
		dev_err(dev, "RM_RA:Message reconfig failed(%d)\n", ret);
		return ret;
	}
	req = (struct ti_sci_msg_ring_reconfig *)xfer->xfer_buf;
	req->nav_id = nav_id;
	req->index = index;
	req->addr_lo = addr_lo ? *addr_lo : TI_SCI_RM_NULL_U32;
	req->addr_hi = addr_hi ? *addr_hi : TI_SCI_RM_NULL_U32;
	req->count = count ? *count : TI_SCI_RM_NULL_U32;
	req->mode = mode ? *mode : TI_SCI_RM_NULL_U8;
	req->size = size ? *size : TI_SCI_RM_NULL_U8;
	req->order_id = order_id ? *order_id : TI_SCI_RM_NULL_U8;

	ret = ti_sci_do_xfer(info, xfer);
	if (ret) {
		dev_err(dev, "RM_RA:Mbox send fail %d\n", ret);
		goto fail;
	}

	resp = (struct ti_sci_msg_ring_reconfig_resp *)xfer->xfer_buf;

	if (!ti_sci_is_response_ack(resp)) {
		ret = -ENODEV;
	} else {
		if (mode)
			*mode = resp->old_mode;
		if (addr_lo)
			*addr_lo = resp->old_addr_lo;
		if (addr_hi)
			*addr_hi = resp->old_addr_hi;
		if (count)
			*count = resp->old_count;
		if (size)
			*size = resp->old_size;
		if (order_id)
			*order_id = resp->old_order_id;
	};

fail:
	ti_sci_put_one_xfer(&info->minfo, xfer);
	dev_dbg(dev, "RM_RA:reconfig ring %u ret:%d\n", index, ret);

	return ret;
}

static int ti_sci_cmd_rm_psil_pair(const struct ti_sci_handle *handle,
				   u32 nav_id, u32 src_thread, u32 dst_thread)
{
	struct ti_sci_msg_hdr *resp;
	struct ti_sci_msg_psil_pair *req;
	struct ti_sci_xfer *xfer;
	struct ti_sci_info *info;
	struct device *dev;
	int ret = 0;

	if (IS_ERR(handle))
		return PTR_ERR(handle);
	if (!handle)
		return -EINVAL;

	info = handle_to_ti_sci_info(handle);
	dev = info->dev;

	xfer = ti_sci_get_one_xfer(info, TI_SCI_MSG_RM_PSIL_PAIR,
				   TI_SCI_FLAG_REQ_ACK_ON_PROCESSED,
				   sizeof(*req), sizeof(*resp));
	if (IS_ERR(xfer)) {
		ret = PTR_ERR(xfer);
		dev_err(dev, "RM_PSIL:Message reconfig failed(%d)\n", ret);
		return ret;
	}
	req = (struct ti_sci_msg_psil_pair *)xfer->xfer_buf;
	req->nav_id = nav_id;
	req->src_thread = src_thread;
	req->dst_thread = dst_thread;

	ret = ti_sci_do_xfer(info, xfer);
	if (ret) {
		dev_err(dev, "RM_PSIL:Mbox send fail %d\n", ret);
		goto fail;
	}

	resp = (struct ti_sci_msg_hdr *)xfer->xfer_buf;
	ret = ti_sci_is_response_ack(resp) ? 0 : -ENODEV;

fail:
	ti_sci_put_one_xfer(&info->minfo, xfer);

	return ret;
}

static int ti_sci_cmd_rm_psil_unpair(const struct ti_sci_handle *handle,
				     u32 nav_id, u32 src_thread, u32 dst_thread)
{
	struct ti_sci_msg_hdr *resp;
	struct ti_sci_msg_psil_unpair *req;
	struct ti_sci_xfer *xfer;
	struct ti_sci_info *info;
	struct device *dev;
	int ret = 0;

	if (IS_ERR(handle))
		return PTR_ERR(handle);
	if (!handle)
		return -EINVAL;

	info = handle_to_ti_sci_info(handle);
	dev = info->dev;

	xfer = ti_sci_get_one_xfer(info, TI_SCI_MSG_RM_PSIL_UNPAIR,
				   TI_SCI_FLAG_REQ_ACK_ON_PROCESSED,
				   sizeof(*req), sizeof(*resp));
	if (IS_ERR(xfer)) {
		ret = PTR_ERR(xfer);
		dev_err(dev, "RM_PSIL:Message reconfig failed(%d)\n", ret);
		return ret;
	}
	req = (struct ti_sci_msg_psil_unpair *)xfer->xfer_buf;
	req->nav_id = nav_id;
	req->src_thread = src_thread;
	req->dst_thread = dst_thread;

	ret = ti_sci_do_xfer(info, xfer);
	if (ret) {
		dev_err(dev, "RM_PSIL:Mbox send fail %d\n", ret);
		goto fail;
	}

	resp = (struct ti_sci_msg_hdr *)xfer->xfer_buf;
	ret = ti_sci_is_response_ack(resp) ? 0 : -ENODEV;

fail:
	ti_sci_put_one_xfer(&info->minfo, xfer);

	return ret;
}

static int ti_sci_cmd_rm_udmap_tx_ch_alloc(
			const struct ti_sci_handle *handle,
			const struct ti_sci_rm_udmap_tx_ch_alloc *params,
			u32 *index)
{
	struct ti_sci_msg_udmap_tx_ch_alloc_resp *resp;
	struct ti_sci_msg_udmap_tx_ch_alloc *req;
	struct ti_sci_xfer *xfer;
	struct ti_sci_info *info;
	struct device *dev;
	int ret = 0;

	if (IS_ERR(handle))
		return PTR_ERR(handle);
	if (!handle)
		return -EINVAL;

	info = handle_to_ti_sci_info(handle);
	dev = info->dev;

	xfer = ti_sci_get_one_xfer(info, TI_SCI_MSG_RM_UDMAP_TX_ALLOC,
				   TI_SCI_FLAG_REQ_ACK_ON_PROCESSED,
				   sizeof(*req), sizeof(*resp));
	if (IS_ERR(xfer)) {
		ret = PTR_ERR(xfer);
		dev_err(dev, "Message alloc failed(%d)\n", ret);
		return ret;
	}
	req = (struct ti_sci_msg_udmap_tx_ch_alloc *)xfer->xfer_buf;
	req->secondary_host = params->secondary_host;
	req->nav_id = params->nav_id;
	req->index = params->index;
	req->tx_pause_on_err = params->tx_pause_on_err;
	req->tx_filt_einfo = params->tx_filt_einfo;
	req->tx_filt_pswords = params->tx_filt_pswords;
	req->tx_atype = params->tx_atype;
	req->tx_chan_type = params->tx_chan_type;
	req->tx_supr_tdpkt = params->tx_supr_tdpkt;
	req->tx_fetch_size = params->tx_fetch_size;
	req->tx_credit_count = params->tx_credit_count;
	req->txcq_qnum = params->txcq_qnum;
	req->tx_priority = params->tx_priority;
	req->tx_qos = params->tx_qos;
	req->tx_orderid = params->tx_orderid;
	req->fdepth = params->fdepth;
	req->tx_sched_priority = params->tx_sched_priority;
	req->share = params->share;
	req->type = params->type;

	ret = ti_sci_do_xfer(info, xfer);
	if (ret) {
		dev_err(dev, "Mbox send fail %d\n", ret);
		goto fail;
	}

	resp = (struct ti_sci_msg_udmap_tx_ch_alloc_resp *)xfer->xfer_buf;

	if (!ti_sci_is_response_ack(resp))
		ret = -EINVAL;
	else
		*index = resp->index;

fail:
	ti_sci_put_one_xfer(&info->minfo, xfer);

	return ret;
}

static int ti_sci_cmd_rm_udmap_tx_ch_free(const struct ti_sci_handle *handle,
					  u8 secondary_host, u32 nav_id,
					  u32 index)
{
	struct ti_sci_msg_hdr *resp;
	struct ti_sci_msg_udmap_tx_ch_free *req;
	struct ti_sci_xfer *xfer;
	struct ti_sci_info *info;
	struct device *dev;
	int ret = 0;

	if (IS_ERR(handle))
		return PTR_ERR(handle);
	if (!handle)
		return -EINVAL;

	info = handle_to_ti_sci_info(handle);
	dev = info->dev;

	xfer = ti_sci_get_one_xfer(info, TI_SCI_MSG_RM_UDMAP_TX_FREE,
				   TI_SCI_FLAG_REQ_ACK_ON_PROCESSED,
				   sizeof(*req), sizeof(*resp));
	if (IS_ERR(xfer)) {
		ret = PTR_ERR(xfer);
		dev_err(dev, "Message alloc failed(%d)\n", ret);
		return ret;
	}
	req = (struct ti_sci_msg_udmap_tx_ch_free *)xfer->xfer_buf;
	req->secondary_host = secondary_host;
	req->nav_id = nav_id;
	req->index = index;

	ret = ti_sci_do_xfer(info, xfer);
	if (ret) {
		dev_err(dev, "Mbox send fail %d\n", ret);
		goto fail;
	}

	resp = (struct ti_sci_msg_hdr *)xfer->xfer_buf;
	ret = ti_sci_is_response_ack(resp) ? 0 : -EINVAL;

fail:
	ti_sci_put_one_xfer(&info->minfo, xfer);

	return ret;
}

static int ti_sci_cmd_rm_udmap_rx_ch_alloc(
			const struct ti_sci_handle *handle,
			const struct ti_sci_rm_udmap_rx_ch_alloc *params,
			u32 *index, u32 *def_flow_index,
			u32 *rng_flow_start_index, u32 *rng_flow_cnt)
{
	struct ti_sci_msg_udmap_rx_ch_alloc_resp *resp;
	struct ti_sci_msg_udmap_rx_ch_alloc *req;
	struct ti_sci_xfer *xfer;
	struct ti_sci_info *info;
	struct device *dev;
	int ret = 0;

	if (IS_ERR(handle))
		return PTR_ERR(handle);
	if (!handle)
		return -EINVAL;

	info = handle_to_ti_sci_info(handle);
	dev = info->dev;

	xfer = ti_sci_get_one_xfer(info, TI_SCI_MSG_RM_UDMAP_RX_ALLOC,
				   TI_SCI_FLAG_REQ_ACK_ON_PROCESSED,
				   sizeof(*req), sizeof(*resp));
	if (IS_ERR(xfer)) {
		ret = PTR_ERR(xfer);
		dev_err(dev, "Message alloc failed(%d)\n", ret);
		return ret;
	}
	req = (struct ti_sci_msg_udmap_rx_ch_alloc *)xfer->xfer_buf;
	req->secondary_host = params->secondary_host;
	req->nav_id = params->nav_id;
	req->index = params->index;
	req->rx_fetch_size = params->rx_fetch_size;
	req->rxcq_qnum = params->rxcq_qnum;
	req->rx_priority = params->rx_priority;
	req->rx_qos = params->rx_qos;
	req->rx_orderid = params->rx_orderid;
	req->rx_sched_priority = params->rx_sched_priority;
	req->flowid_start = params->flowid_start;
	req->flowid_cnt = params->flowid_cnt;
	req->rx_pause_on_err = params->rx_pause_on_err;
	req->rx_atype = params->rx_atype;
	req->rx_chan_type = params->rx_chan_type;
	req->rx_ignore_short = params->rx_ignore_short;
	req->rx_ignore_long = params->rx_ignore_long;
	req->share = params->share;
	req->type = params->type;

	ret = ti_sci_do_xfer(info, xfer);
	if (ret) {
		dev_err(dev, "Mbox send fail %d\n", ret);
		goto fail;
	}

	resp = (struct ti_sci_msg_udmap_rx_ch_alloc_resp *)xfer->xfer_buf;

	if (!ti_sci_is_response_ack(resp)) {
		ret = -EINVAL;
	} else {
		*index = resp->index;
		*def_flow_index = resp->def_flow_index;
		*rng_flow_start_index = resp->rng_flow_start_index;
		*rng_flow_cnt = resp->rng_flow_cnt;
	}

fail:
	ti_sci_put_one_xfer(&info->minfo, xfer);

	return ret;
}

static int ti_sci_cmd_rm_udmap_rx_ch_free(const struct ti_sci_handle *handle,
					  u8 secondary_host, u32 nav_id,
					  u32 index)
{
	struct ti_sci_msg_hdr *resp;
	struct ti_sci_msg_udmap_rx_ch_free *req;
	struct ti_sci_xfer *xfer;
	struct ti_sci_info *info;
	struct device *dev;
	int ret = 0;

	if (IS_ERR(handle))
		return PTR_ERR(handle);
	if (!handle)
		return -EINVAL;

	info = handle_to_ti_sci_info(handle);
	dev = info->dev;

	xfer = ti_sci_get_one_xfer(info, TI_SCI_MSG_RM_UDMAP_RX_FREE,
				   TI_SCI_FLAG_REQ_ACK_ON_PROCESSED,
				   sizeof(*req), sizeof(*resp));
	if (IS_ERR(xfer)) {
		ret = PTR_ERR(xfer);
		dev_err(dev, "Message alloc failed(%d)\n", ret);
		return ret;
	}
	req = (struct ti_sci_msg_udmap_rx_ch_free *)xfer->xfer_buf;
	req->secondary_host = secondary_host;
	req->nav_id = nav_id;
	req->index = index;

	ret = ti_sci_do_xfer(info, xfer);
	if (ret) {
		dev_err(dev, "Mbox send fail %d\n", ret);
		goto fail;
	}

	resp = (struct ti_sci_msg_hdr *)xfer->xfer_buf;
	ret = ti_sci_is_response_ack(resp) ? 0 : -EINVAL;

fail:
	ti_sci_put_one_xfer(&info->minfo, xfer);

	return ret;
}

static int ti_sci_cmd_rm_udmap_rx_flow_cfg(
			const struct ti_sci_handle *handle,
			const struct ti_sci_rm_udmap_rx_flow_cfg *params)
{
	struct ti_sci_msg_hdr *resp;
	struct ti_sci_msg_udmap_rx_flow_cfg *req;
	struct ti_sci_xfer *xfer;
	struct ti_sci_info *info;
	struct device *dev;
	int ret = 0;

	if (IS_ERR(handle))
		return PTR_ERR(handle);
	if (!handle)
		return -EINVAL;

	info = handle_to_ti_sci_info(handle);
	dev = info->dev;

	xfer = ti_sci_get_one_xfer(info, TI_SCI_MSG_RM_UDMAP_FLOW_CFG,
				   TI_SCI_FLAG_REQ_ACK_ON_PROCESSED,
				   sizeof(*req), sizeof(*resp));
	if (IS_ERR(xfer)) {
		ret = PTR_ERR(xfer);
		dev_err(dev, "Message alloc failed(%d)\n", ret);
		return ret;
	}
	req = (struct ti_sci_msg_udmap_rx_flow_cfg *)xfer->xfer_buf;
	req->nav_id = params->nav_id;
	req->flow_index = params->flow_index;
	req->rx_ch_index = params->rx_ch_index;
	req->rx_einfo_present = params->rx_einfo_present;
	req->rx_psinfo_present = params->rx_psinfo_present;
	req->rx_error_handling = params->rx_error_handling;
	req->rx_desc_type = params->rx_desc_type;
	req->rx_sop_offset = params->rx_sop_offset;
	req->rx_dest_qnum = params->rx_dest_qnum;
	req->rx_ps_location = params->rx_ps_location;
	req->rx_src_tag_hi = params->rx_src_tag_hi;
	req->rx_src_tag_lo = params->rx_src_tag_lo;
	req->rx_dest_tag_hi = params->rx_dest_tag_hi;
	req->rx_dest_tag_lo = params->rx_dest_tag_lo;
	req->rx_src_tag_hi_sel = params->rx_src_tag_hi_sel;
	req->rx_src_tag_lo_sel = params->rx_src_tag_lo_sel;
	req->rx_dest_tag_hi_sel = params->rx_dest_tag_hi_sel;
	req->rx_dest_tag_lo_sel = params->rx_dest_tag_lo_sel;
	req->rx_size_thresh_en = params->rx_size_thresh_en;
	req->rx_fdq0_sz0_qnum = params->rx_fdq0_sz0_qnum;
	req->rx_fdq1_qnum = params->rx_fdq1_qnum;
	req->rx_fdq2_qnum = params->rx_fdq2_qnum;
	req->rx_fdq3_qnum = params->rx_fdq3_qnum;

	ret = ti_sci_do_xfer(info, xfer);
	if (ret) {
		dev_err(dev, "Mbox send fail %d\n", ret);
		goto fail;
	}

	resp = (struct ti_sci_msg_hdr *)xfer->xfer_buf;
	ret = ti_sci_is_response_ack(resp) ? 0 : -EINVAL;

fail:
	ti_sci_put_one_xfer(&info->minfo, xfer);

	return ret;
}

static int ti_sci_cmd_rm_udmap_rx_flow_opt_cfg(
			const struct ti_sci_handle *handle,
			const struct ti_sci_rm_udmap_rx_flow_opt_cfg *params)
{
	struct ti_sci_msg_hdr *resp;
	struct rm_ti_sci_msg_udmap_rx_flow_opt_cfg *req;
	struct ti_sci_xfer *xfer;
	struct ti_sci_info *info;
	struct device *dev;
	int ret = 0;

	if (IS_ERR(handle))
		return PTR_ERR(handle);
	if (!handle)
		return -EINVAL;

	info = handle_to_ti_sci_info(handle);
	dev = info->dev;

	xfer = ti_sci_get_one_xfer(info, TI_SCI_MSG_RM_UDMAP_OPT_FLOW_CFG,
				   TI_SCI_FLAG_REQ_ACK_ON_PROCESSED,
				   sizeof(*req), sizeof(*resp));
	if (IS_ERR(xfer)) {
		ret = PTR_ERR(xfer);
		dev_err(dev, "Message alloc failed(%d)\n", ret);
		return ret;
	}
	req = (struct rm_ti_sci_msg_udmap_rx_flow_opt_cfg *)xfer->xfer_buf;
	req->nav_id = params->nav_id;
	req->flow_index = params->flow_index;
	req->rx_ch_index = params->rx_ch_index;
	req->rx_size_thresh0 = params->rx_size_thresh0;
	req->rx_size_thresh1 = params->rx_size_thresh1;
	req->rx_size_thresh2 = params->rx_size_thresh2;
	req->rx_fdq0_sz1_qnum = params->rx_fdq0_sz1_qnum;
	req->rx_fdq0_sz2_qnum = params->rx_fdq0_sz2_qnum;
	req->rx_fdq0_sz3_qnum = params->rx_fdq0_sz3_qnum;

	ret = ti_sci_do_xfer(info, xfer);
	if (ret) {
		dev_err(dev, "Mbox send fail %d\n", ret);
		goto fail;
	}

	resp = (struct ti_sci_msg_hdr *)xfer->xfer_buf;
	ret = ti_sci_is_response_ack(resp) ? 0 : -EINVAL;

fail:
	ti_sci_put_one_xfer(&info->minfo, xfer);

	return ret;
}

/*
 * ti_sci_setup_ops() - Setup the operations structures
 * @info:	pointer to TISCI pointer
 */
static void ti_sci_setup_ops(struct ti_sci_info *info)
{
	struct ti_sci_ops *ops = &info->handle.ops;
	struct ti_sci_core_ops *core_ops = &ops->core_ops;
	struct ti_sci_dev_ops *dops = &ops->dev_ops;
	struct ti_sci_clk_ops *cops = &ops->clk_ops;
	struct ti_sci_irq_ops *iops = &ops->irq_ops;
	struct ti_sci_rm_ringacc_ops *rops = &ops->rm_ring_ops;
	struct ti_sci_rm_psil_ops *psilops = &ops->rm_psil_ops;
	struct ti_sci_rm_udmap_ops *udmap_ops = &ops->rm_udmap_ops;

	core_ops->reboot_device = ti_sci_cmd_core_reboot;

	dops->get_device = ti_sci_cmd_get_device;
	dops->idle_device = ti_sci_cmd_idle_device;
	dops->put_device = ti_sci_cmd_put_device;

	dops->is_valid = ti_sci_cmd_dev_is_valid;
	dops->get_context_loss_count = ti_sci_cmd_dev_get_clcnt;
	dops->is_idle = ti_sci_cmd_dev_is_idle;
	dops->is_stop = ti_sci_cmd_dev_is_stop;
	dops->is_on = ti_sci_cmd_dev_is_on;
	dops->is_transitioning = ti_sci_cmd_dev_is_trans;
	dops->set_device_resets = ti_sci_cmd_set_device_resets;
	dops->get_device_resets = ti_sci_cmd_get_device_resets;

	cops->get_clock = ti_sci_cmd_get_clock;
	cops->idle_clock = ti_sci_cmd_idle_clock;
	cops->put_clock = ti_sci_cmd_put_clock;
	cops->is_auto = ti_sci_cmd_clk_is_auto;
	cops->is_on = ti_sci_cmd_clk_is_on;
	cops->is_off = ti_sci_cmd_clk_is_off;

	cops->set_parent = ti_sci_cmd_clk_set_parent;
	cops->get_parent = ti_sci_cmd_clk_get_parent;
	cops->get_num_parents = ti_sci_cmd_clk_get_num_parents;

	cops->get_best_match_freq = ti_sci_cmd_clk_get_match_freq;
	cops->set_freq = ti_sci_cmd_clk_set_freq;
	cops->get_freq = ti_sci_cmd_clk_get_freq;

	iops->set_irq = ti_sci_cmd_set_irq;
	iops->free_irq = ti_sci_cmd_free_irq;

	rops->allocate = ti_sci_cmd_ring_allocate;
	rops->free = ti_sci_cmd_ring_free;
	rops->reset = ti_sci_cmd_ring_reset;
	rops->reconfig = ti_sci_cmd_ring_reconfig;

	psilops->pair = ti_sci_cmd_rm_psil_pair;
	psilops->unpair = ti_sci_cmd_rm_psil_unpair;

	udmap_ops->tx_ch_alloc = ti_sci_cmd_rm_udmap_tx_ch_alloc;
	udmap_ops->tx_ch_free = ti_sci_cmd_rm_udmap_tx_ch_free;
	udmap_ops->rx_ch_alloc = ti_sci_cmd_rm_udmap_rx_ch_alloc;
	udmap_ops->rx_ch_free = ti_sci_cmd_rm_udmap_rx_ch_free;
	udmap_ops->rx_flow_cfg = ti_sci_cmd_rm_udmap_rx_flow_cfg;
	udmap_ops->rx_flow_opt_cfg = ti_sci_cmd_rm_udmap_rx_flow_opt_cfg;
}

/**
 * ti_sci_get_handle() - Get the TI SCI handle for a device
 * @dev:	Pointer to device for which we want SCI handle
 *
 * NOTE: The function does not track individual clients of the framework
 * and is expected to be maintained by caller of TI SCI protocol library.
 * ti_sci_put_handle must be balanced with successful ti_sci_get_handle
 * Return: pointer to handle if successful, else:
 * -EPROBE_DEFER if the instance is not ready
 * -ENODEV if the required node handler is missing
 * -EINVAL if invalid conditions are encountered.
 */
const struct ti_sci_handle *ti_sci_get_handle(struct device *dev)
{
	struct device_node *ti_sci_np;
	struct list_head *p;
	struct ti_sci_handle *handle = NULL;
	struct ti_sci_info *info;

	if (!dev) {
		pr_err("I need a device pointer\n");
		return ERR_PTR(-EINVAL);
	}
	ti_sci_np = of_get_parent(dev->of_node);
	if (!ti_sci_np) {
		dev_err(dev, "No OF information\n");
		return ERR_PTR(-EINVAL);
	}

	mutex_lock(&ti_sci_list_mutex);
	list_for_each(p, &ti_sci_list) {
		info = list_entry(p, struct ti_sci_info, node);
		if (ti_sci_np == info->dev->of_node) {
			handle = &info->handle;
			info->users++;
			break;
		}
	}
	mutex_unlock(&ti_sci_list_mutex);
	of_node_put(ti_sci_np);

	if (!handle)
		return ERR_PTR(-EPROBE_DEFER);

	return handle;
}
EXPORT_SYMBOL_GPL(ti_sci_get_handle);

/**
 * ti_sci_get_by_phandle() - Get the TI SCI handle using DT phandle
 * @np: device node
 * @propname: property name containing phandle on TISCI node
 *
 * NOTE: The function does not track individual clients of the framework
 * and is expected to be maintained by caller of TI SCI protocol library.
 * ti_sci_put_handle must be balanced with successful ti_sci_get_handle
 * Return: pointer to handle if successful, else:
 * -EPROBE_DEFER if the instance is not ready
 * -ENODEV if the required node handler is missing
 * -EINVAL if invalid conditions are encountered.
 */
const struct ti_sci_handle *ti_sci_get_by_phandle(struct device_node *np,
						  const char *property)
{
	struct device_node *ti_sci_np;
	struct list_head *p;
	struct ti_sci_handle *handle = NULL;
	struct ti_sci_info *info;

	if (!np) {
		pr_err("I need a device pointer\n");
		return ERR_PTR(-EINVAL);
	}

	ti_sci_np = of_parse_phandle(np, property, 0);
	if (!ti_sci_np)
		return ERR_PTR(-ENODEV);

	mutex_lock(&ti_sci_list_mutex);
	list_for_each(p, &ti_sci_list) {
		info = list_entry(p, struct ti_sci_info, node);
		if (ti_sci_np == info->dev->of_node) {
			handle = &info->handle;
			info->users++;
			break;
		}
	}
	mutex_unlock(&ti_sci_list_mutex);
	of_node_put(ti_sci_np);

	if (!handle)
		return ERR_PTR(-EPROBE_DEFER);

	return handle;
}
EXPORT_SYMBOL_GPL(ti_sci_get_by_phandle);

/**
 * ti_sci_put_handle() - Release the handle acquired by ti_sci_get_handle
 * @handle:	Handle acquired by ti_sci_get_handle
 *
 * NOTE: The function does not track individual clients of the framework
 * and is expected to be maintained by caller of TI SCI protocol library.
 * ti_sci_put_handle must be balanced with successful ti_sci_get_handle
 *
 * Return: 0 is successfully released
 * if an error pointer was passed, it returns the error value back,
 * if null was passed, it returns -EINVAL;
 */
int ti_sci_put_handle(const struct ti_sci_handle *handle)
{
	struct ti_sci_info *info;

	if (IS_ERR(handle))
		return PTR_ERR(handle);
	if (!handle)
		return -EINVAL;

	info = handle_to_ti_sci_info(handle);
	mutex_lock(&ti_sci_list_mutex);
	if (!WARN_ON(!info->users))
		info->users--;
	mutex_unlock(&ti_sci_list_mutex);

	return 0;
}
EXPORT_SYMBOL_GPL(ti_sci_put_handle);

static void devm_ti_sci_release(struct device *dev, void *res)
{
	const struct ti_sci_handle **ptr = res;
	const struct ti_sci_handle *handle = *ptr;
	int ret;

	ret = ti_sci_put_handle(handle);
	if (ret)
		dev_err(dev, "failed to put handle %d\n", ret);
}

/**
 * devm_ti_sci_get_handle() - Managed get handle
 * @dev:	device for which we want SCI handle for.
 *
 * NOTE: This releases the handle once the device resources are
 * no longer needed. MUST NOT BE released with ti_sci_put_handle.
 * The function does not track individual clients of the framework
 * and is expected to be maintained by caller of TI SCI protocol library.
 *
 * Return: 0 if all went fine, else corresponding error.
 */
const struct ti_sci_handle *devm_ti_sci_get_handle(struct device *dev)
{
	const struct ti_sci_handle **ptr;
	const struct ti_sci_handle *handle;

	ptr = devres_alloc(devm_ti_sci_release, sizeof(*ptr), GFP_KERNEL);
	if (!ptr)
		return ERR_PTR(-ENOMEM);
	handle = ti_sci_get_handle(dev);

	if (!IS_ERR(handle)) {
		*ptr = handle;
		devres_add(dev, ptr);
	} else {
		devres_free(ptr);
	}

	return handle;
}
EXPORT_SYMBOL_GPL(devm_ti_sci_get_handle);

static int tisci_reboot_handler(struct notifier_block *nb, unsigned long mode,
				void *cmd)
{
	struct ti_sci_info *info = reboot_to_ti_sci_info(nb);
	const struct ti_sci_handle *handle = &info->handle;

	ti_sci_cmd_core_reboot(handle);

	/* call fail OR pass, we should not be here in the first place */
	return NOTIFY_BAD;
}

/* Description for K2G */
static const struct ti_sci_desc ti_sci_pmmc_k2g_desc = {
	.default_host_id = 2,
	/* Conservative duration */
	.max_rx_timeout_ms = 1000,
	/* Limited by MBOX_TX_QUEUE_LEN. K2G can handle upto 128 messages! */
	.max_msgs = 20,
	.max_msg_size = 64,
};

static const struct of_device_id ti_sci_of_match[] = {
	{.compatible = "ti,k2g-sci", .data = &ti_sci_pmmc_k2g_desc},
	{ /* Sentinel */ },
};
MODULE_DEVICE_TABLE(of, ti_sci_of_match);

static int ti_sci_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_device_id *of_id;
	const struct ti_sci_desc *desc;
	struct ti_sci_xfer *xfer;
	struct ti_sci_info *info = NULL;
	struct ti_sci_xfers_info *minfo;
	struct mbox_client *cl;
	int ret = -EINVAL;
	int i;
	int reboot = 0;
	u32 h_id;

	of_id = of_match_device(ti_sci_of_match, dev);
	if (!of_id) {
		dev_err(dev, "OF data missing\n");
		return -EINVAL;
	}
	desc = of_id->data;

	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->dev = dev;
	info->desc = desc;
	ret = of_property_read_u32(dev->of_node, "ti,host-id", &h_id);
	/* if the property is not present in DT, use a default from desc */
	if (ret < 0) {
		info->host_id = info->desc->default_host_id;
	} else {
		if (!h_id) {
			dev_warn(dev, "Host ID 0 is reserved for firmware\n");
			info->host_id = info->desc->default_host_id;
		} else {
			info->host_id = h_id;
		}
	}

	reboot = of_property_read_bool(dev->of_node,
				       "ti,system-reboot-controller");
	INIT_LIST_HEAD(&info->node);
	minfo = &info->minfo;

	/*
	 * Pre-allocate messages
	 * NEVER allocate more than what we can indicate in hdr.seq
	 * if we have data description bug, force a fix..
	 */
	if (WARN_ON(desc->max_msgs >=
		    1 << 8 * sizeof(((struct ti_sci_msg_hdr *)0)->seq)))
		return -EINVAL;

	minfo->xfer_block = devm_kcalloc(dev,
					 desc->max_msgs,
					 sizeof(*minfo->xfer_block),
					 GFP_KERNEL);
	if (!minfo->xfer_block)
		return -ENOMEM;

	minfo->xfer_alloc_table = devm_kzalloc(dev,
					       BITS_TO_LONGS(desc->max_msgs)
					       * sizeof(unsigned long),
					       GFP_KERNEL);
	if (!minfo->xfer_alloc_table)
		return -ENOMEM;
	bitmap_zero(minfo->xfer_alloc_table, desc->max_msgs);

	/* Pre-initialize the buffer pointer to pre-allocated buffers */
	for (i = 0, xfer = minfo->xfer_block; i < desc->max_msgs; i++, xfer++) {
		xfer->xfer_buf = devm_kcalloc(dev, 1, desc->max_msg_size,
					      GFP_KERNEL);
		if (!xfer->xfer_buf)
			return -ENOMEM;

		xfer->tx_message.buf = xfer->xfer_buf;
		init_completion(&xfer->done);
	}

	ret = ti_sci_debugfs_create(pdev, info);
	if (ret)
		dev_warn(dev, "Failed to create debug file\n");

	platform_set_drvdata(pdev, info);

	cl = &info->cl;
	cl->dev = dev;
	cl->tx_block = false;
	cl->rx_callback = ti_sci_rx_callback;
	cl->knows_txdone = true;

	spin_lock_init(&minfo->xfer_lock);
	sema_init(&minfo->sem_xfer_count, desc->max_msgs);

	info->chan_rx = mbox_request_channel_byname(cl, "rx");
	if (IS_ERR(info->chan_rx)) {
		ret = PTR_ERR(info->chan_rx);
		goto out;
	}

	info->chan_tx = mbox_request_channel_byname(cl, "tx");
	if (IS_ERR(info->chan_tx)) {
		ret = PTR_ERR(info->chan_tx);
		goto out;
	}
	ret = ti_sci_cmd_get_revision(info);
	if (ret) {
		dev_err(dev, "Unable to communicate with TISCI(%d)\n", ret);
		goto out;
	}

	ti_sci_setup_ops(info);

	if (reboot) {
		info->nb.notifier_call = tisci_reboot_handler;
		info->nb.priority = 128;

		ret = register_restart_handler(&info->nb);
		if (ret) {
			dev_err(dev, "reboot registration fail(%d)\n", ret);
			return ret;
		}
	}

	dev_info(dev, "ABI: %d.%d (firmware rev 0x%04x '%s')\n",
		 info->handle.version.abi_major, info->handle.version.abi_minor,
		 info->handle.version.firmware_revision,
		 info->handle.version.firmware_description);

	mutex_lock(&ti_sci_list_mutex);
	list_add_tail(&info->node, &ti_sci_list);
	mutex_unlock(&ti_sci_list_mutex);

	return of_platform_populate(dev->of_node, NULL, NULL, dev);
out:
	if (!IS_ERR(info->chan_tx))
		mbox_free_channel(info->chan_tx);
	if (!IS_ERR(info->chan_rx))
		mbox_free_channel(info->chan_rx);
	debugfs_remove(info->d);
	return ret;
}

static int ti_sci_remove(struct platform_device *pdev)
{
	struct ti_sci_info *info;
	struct device *dev = &pdev->dev;
	int ret = 0;

	of_platform_depopulate(dev);

	info = platform_get_drvdata(pdev);

	if (info->nb.notifier_call)
		unregister_restart_handler(&info->nb);

	mutex_lock(&ti_sci_list_mutex);
	if (info->users)
		ret = -EBUSY;
	else
		list_del(&info->node);
	mutex_unlock(&ti_sci_list_mutex);

	if (!ret) {
		ti_sci_debugfs_destroy(pdev, info);

		/* Safe to free channels since no more users */
		mbox_free_channel(info->chan_tx);
		mbox_free_channel(info->chan_rx);
	}

	return ret;
}

static struct platform_driver ti_sci_driver = {
	.probe = ti_sci_probe,
	.remove = ti_sci_remove,
	.driver = {
		   .name = "ti-sci",
		   .of_match_table = of_match_ptr(ti_sci_of_match),
	},
};
module_platform_driver(ti_sci_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("TI System Control Interface(SCI) driver");
MODULE_AUTHOR("Nishanth Menon");
MODULE_ALIAS("platform:ti-sci");
