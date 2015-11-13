/*
 * Texas Instruments' Message Manager Driver
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
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

#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/ti-msgmgr.h>

#define Q_DATA_OFFSET(proxy, queue, reg)	\
		     ((0x10000 * (proxy)) + (0x80 * (queue)) + ((reg) * 4))
#define Q_STATE_OFFSET(queue)			((queue) * 0x4)
#define Q_STATE_ENTRY_COUNT_MASK		(0xFFF000)

/**
 * struct ti_msgmgr_desc - Description of message manager integration
 * @queue_count:	Number of Queues
 * @max_message_size:	Message size in bytes
 * @max_messages:	Number of messages
 * @q_slices:		Number of queue engines
 * @q_proxies:		Number of queue proxies per page
 * @data_first_reg:	First data register for proxy data region
 * @data_last_reg:	Last data register for proxy data region
 * @tx_polled:		Do I need to use polled mechanism for tx
 * @tx_poll_timeout_ms: Timeout in ms if polled
 *
 * This structure is used in of match data to describe how integration
 * for a specific compatible SoC is done.
 */
struct ti_msgmgr_desc {
	u8 queue_count;
	u8 max_message_size;
	u8 max_messages;
	u8 q_slices;
	u8 q_proxies;
	u8 data_first_reg;
	u8 data_last_reg;
	bool tx_polled;
	int tx_poll_timeout_ms;
};

/**
 * struct ti_queue_inst - Description of a queue instance
 * @name:	Queue Name
 * @queue_id:	Queue Identifier as mapped on SoC
 * @proxy_id:	Proxy Identifier as mapped on SoC
 * @irq:	IRQ for Rx Queue
 * @is_tx:	'true' if transmit queue, else, 'false'
 * @queue_buff_start: First register of Data Buffer
 * @queue_buff_end: Last (or confirmation) register of Data buffer
 * @queue_state: Queue status register
 * @chan:	Mailbox channel
 * @rx_buff:	Receive buffer pointer allocated at probe, max_message_size
 */
struct ti_queue_inst {
	const char *name;
	u32 queue_id;
	u32 proxy_id;
	int irq;
	bool is_tx;
	void __iomem *queue_buff_start;
	void __iomem *queue_buff_end;
	void __iomem *queue_state;
	struct mbox_chan *chan;
	u32 *rx_buff;
};

/**
 * struct ti_msgmgr_inst - Description of a Message Manager Instance
 * @dev:	device pointer corresponding to the Message Manager instance
 * @desc:	Description of the SoC integration
 * @queue_proxy_region:	Queue proxy region where queue buffers are located
 * @queue_state_debug_region:	Queue status register regions
 * @num_valid_queues:	Number of valid queues defined for the processor
 *		Note: other queues are probably reserved for other processors
 *		in the SoC.
 * @qinsts:	Array of valid Queue Instances for the Processor
 * @mbox:	Mailbox Controller
 * @chans:	Array for channels corresponding to the Queue Instances.
 */
struct ti_msgmgr_inst {
	struct device *dev;
	const struct ti_msgmgr_desc *desc;
	void __iomem *queue_proxy_region;
	void __iomem *queue_state_debug_region;
	u8 num_valid_queues;
	struct ti_queue_inst *qinsts;
	struct mbox_controller mbox;
	struct mbox_chan *chans;
};

/**
 * ti_msgmgr_queue_get_num_messages() - Get the number of pending messages
 * @qinst:	Queue instance for which we check the number of pending messages
 *
 * Return: number of messages pending in the queue (0 == no pending messages)
 */
static inline int ti_msgmgr_queue_get_num_messages(struct ti_queue_inst *qinst)
{
	u32 val;

	/*
	 * We cannot use relaxed operation here - update may happen
	 * real-time.
	 */
	val = readl(qinst->queue_state) & Q_STATE_ENTRY_COUNT_MASK;
	val >>= __ffs(Q_STATE_ENTRY_COUNT_MASK);

	return val;
}

/**
 * ti_msgmgr_queue_rx_interrupt() - Interrupt handler for receive Queue
 * @irq:	Interrupt number
 * @p:		Channel Pointer
 *
 * Return: -EINVAL if there is no instance
 * IRQ_NONE if the interrupt is not ours.
 * IRQ_HANDLED if the rx interrupt was successfully handled.
 */
static irqreturn_t ti_msgmgr_queue_rx_interrupt(int irq, void *p)
{
	struct mbox_chan *chan = p;
	struct device *dev = chan->mbox->dev;
	struct ti_msgmgr_inst *inst = dev_get_drvdata(dev);
	struct ti_queue_inst *qinst = chan->con_priv;
	const struct ti_msgmgr_desc *desc;
	int msg_count, num_words;
	struct ti_msgmgr_message message;
	void __iomem *data_reg;
	u32 *word_data;

	if (WARN_ON(!inst)) {
		dev_err(dev, "no platform drv data??\n");
		return -EINVAL;
	}

	/* Do I have an invalid interrupt source? */
	if (qinst->is_tx) {
		dev_err(dev, "Cannot handle rx interrupt on tx channel %s\n",
			qinst->name);
		return IRQ_NONE;
	}

	/* Do I actually have messages to read? */
	msg_count = ti_msgmgr_queue_get_num_messages(qinst);
	if (!msg_count) {
		dev_dbg(dev, "Spurious event - 0 pending data!\n");
		return IRQ_NONE;
	}

	/*
	 * I have no idea about the protocol being used to communicate with the
	 * remote producer - 0 is valid data, so I wont make a judgement of how
	 * many bytes I should be reading. Let the client figure this out.. I
	 * just read the full message and pass it on..
	 */
	desc = inst->desc;
	message.len = desc->max_message_size;
	message.buf = (u8 *)qinst->rx_buff;

	/*
	 * NOTE about register access involved here:
	 * the hardware block is implemented with 32bit access operations and no
	 * support for data splitting.  We don't want the hardware to misbehave
	 * with sub 32bit access - For example: if the last register read is
	 * split into byte wise access, it can result in the queue getting
	 * stuck or indeterminate behavior. Hence, we do not use memcpy_fromio
	 * here.
	 * Also note that the final register read automatically marks the
	 * queue message as read.
	 */
	for (data_reg = qinst->queue_buff_start, word_data = qinst->rx_buff,
	     num_words = (desc->max_message_size / sizeof(u32));
	     num_words; num_words--, data_reg += sizeof(u32), word_data++)
		*word_data = readl(data_reg);

	/*
	 * Last register read automatically clears the IRQ if only 1 message
	 * is pending - so send the data up the stack..
	 * NOTE: Client is expected to be as optimal as possible, since
	 * we invoke the handler in IRQ context.
	 */
	mbox_chan_received_data(chan, (void *)&message);

	return IRQ_HANDLED;
}

/**
 * ti_msgmgr_queue_peek_data() - Peek to see if there are any rx messages.
 * @chan:	Channel Pointer
 *
 * Return: 'true' if there is pending rx data, 'false' if there is none.
 */
static bool ti_msgmgr_queue_peek_data(struct mbox_chan *chan)
{
	struct ti_queue_inst *qinst = chan->con_priv;
	int msg_count;

	if (qinst->is_tx)
		return false;

	msg_count = ti_msgmgr_queue_get_num_messages(qinst);

	return msg_count ? true : false;
}

/**
 * ti_msgmgr_last_tx_done() - See if all the tx messages are sent
 * @chan:	Channel pointer
 *
 * Return: 'true' is no pending tx data, 'false' if there are any.
 */
static bool ti_msgmgr_last_tx_done(struct mbox_chan *chan)
{
	struct ti_queue_inst *qinst = chan->con_priv;
	int msg_count;

	if (!qinst->is_tx)
		return false;

	msg_count = ti_msgmgr_queue_get_num_messages(qinst);

	/* if we have any messages pending.. */
	return msg_count ? false : true;
}

/**
 * ti_msgmgr_send_data() - Send data
 * @chan:	Channel Pointer
 * @data:	ti_msgmgr_message * Message Pointer
 *
 * Return: 0 if all goes good, else appropriate error messages.
 */
static int ti_msgmgr_send_data(struct mbox_chan *chan, void *data)
{
	struct device *dev = chan->mbox->dev;
	struct ti_msgmgr_inst *inst = dev_get_drvdata(dev);
	const struct ti_msgmgr_desc *desc;
	struct ti_queue_inst *qinst = chan->con_priv;
	int msg_count, num_words, trail_bytes;
	struct ti_msgmgr_message *message = data;
	void __iomem *data_reg;
	u32 *word_data;

	if (WARN_ON(!inst)) {
		dev_err(dev, "no platform drv data??\n");
		return -EINVAL;
	}
	desc = inst->desc;

	if (desc->max_message_size < message->len) {
		dev_err(dev, "Queue %s message length %d > max %d\n",
			qinst->name, message->len, desc->max_message_size);
		return -EINVAL;
	}

	/* Are we able to send this or not? */
	msg_count = ti_msgmgr_queue_get_num_messages(qinst);
	if (msg_count >= desc->max_messages) {
		dev_warn(dev, "Queue %s is full (%d messages)\n", qinst->name,
			 msg_count);
		return -EBUSY;
	}

	/*
	 * NOTE about register access involved here:
	 * the hardware block is implemented with 32bit access operations and no
	 * support for data splitting.  We don't want the hardware to misbehave
	 * with sub 32bit access - For example: if the last register write is
	 * split into byte wise access, it can result in the queue getting
	 * stuck or dummy messages being transmitted or indeterminate behavior.
	 * Hence, we do not use memcpy_toio here.
	 */
	for (data_reg = qinst->queue_buff_start,
	     num_words = message->len / sizeof(u32),
	     word_data = (u32 *)message->buf;
	     num_words; num_words--, data_reg += sizeof(u32), word_data++)
		writel(*word_data, data_reg);

	trail_bytes = message->len % sizeof(u32);
	if (trail_bytes) {
		u32 data_trail = *word_data;

		/* Ensure all unused data is 0 */
		data_trail &= 0xFFFFFFFF >> (8 * (sizeof(u32) - trail_bytes));
		writel(data_trail, data_reg);
		data_reg++;
	}
	/*
	 * 'data_reg' indicates next register to write. If we did not already
	 * write on tx complete reg(last reg), we must do so for transmit
	 */
	if (data_reg <= qinst->queue_buff_end)
		writel(0, qinst->queue_buff_end);

	return 0;
}

/**
 * ti_msgmgr_queue_startup() - Startup queue
 * @chan:	Channel pointer
 *
 * Return: 0 if all goes good, else return corresponding error message
 */
static int ti_msgmgr_queue_startup(struct mbox_chan *chan)
{
	struct ti_queue_inst *qinst = chan->con_priv;
	struct device *dev = chan->mbox->dev;
	int ret;

	if (!qinst->is_tx) {
		/*
		 * With the expectation that the IRQ might be shared in
		 * future
		 */
		ret = request_irq(qinst->irq, ti_msgmgr_queue_rx_interrupt,
				  IRQF_SHARED, qinst->name, chan);
		if (ret) {
			dev_err(dev, "Unable to get IRQ %d on %s(res=%d)\n",
				qinst->irq, qinst->name, ret);
			return ret;
		}
	}

	return 0;
}

/**
 * ti_msgmgr_queue_shutdown() - Shutdown the queue
 * @chan:	Channel pointer
 */
static void ti_msgmgr_queue_shutdown(struct mbox_chan *chan)
{
	struct ti_queue_inst *qinst = chan->con_priv;

	if (!qinst->is_tx)
		free_irq(qinst->irq, chan);
}

/**
 * ti_msgmgr_of_xlate() - Translation of phandle to queue
 * @mbox:	Mailbox controller
 * @p:		phandle pointer
 *
 * Return: Mailbox channel corresponding to the queue, else return error
 * pointer.
 */
static struct mbox_chan *ti_msgmgr_of_xlate(struct mbox_controller *mbox,
					    const struct of_phandle_args *p)
{
	struct ti_msgmgr_inst *inst;
	struct device_node *np;
	phandle phandle = p->args[0];
	struct ti_queue_inst *qinst;
	int i;
	struct mbox_chan *chan = ERR_PTR(-ENOENT);

	inst = container_of(mbox, struct ti_msgmgr_inst, mbox);
	if (WARN_ON(!inst))
		return ERR_PTR(-EINVAL);

	np = of_find_node_by_phandle(phandle);
	if (!np) {
		dev_err(inst->dev, "Unable to find phandle %p\n", p);
		return ERR_PTR(-ENODEV);
	}

	for (qinst = inst->qinsts, i = 0; i < inst->num_valid_queues;
	     i++, qinst++) {
		/* not using strncmp: What could be a valid length here?? */
		if (!strcmp(qinst->name, np->name)) {
			chan = qinst->chan;
			break;
		}
	}
	of_node_put(np);

	return chan;
}

/* Queue operations */
static const struct mbox_chan_ops ti_msgmgr_chan_ops = {
	.startup = ti_msgmgr_queue_startup,
	.shutdown = ti_msgmgr_queue_shutdown,
	.peek_data = ti_msgmgr_queue_peek_data,
	.last_tx_done = ti_msgmgr_last_tx_done,
	.send_data = ti_msgmgr_send_data,
};

/* Keystone K2G SoC integration details */
static const struct ti_msgmgr_desc k2g_desc = {
	.queue_count = 64,
	.max_message_size = 64,
	.max_messages = 128,
	.q_slices = 1,
	.q_proxies = 1,
	.data_first_reg = 16,
	.data_last_reg = 31,
	.tx_polled = false,
};

static const struct of_device_id ti_msgmgr_of_match[] = {
	{.compatible = "ti,k2g-message-manager", .data = &k2g_desc},
	{.compatible = "ti,message-manager", .data = &k2g_desc},
	{ /* Sentinel */ }
};
MODULE_DEVICE_TABLE(of, ti_msgmgr_of_match);

static int ti_msgmgr_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_device_id *of_id;
	struct device_node *np, *child;
	struct resource *res;
	const struct ti_msgmgr_desc *desc;
	struct ti_msgmgr_inst *inst;
	struct ti_queue_inst *qinst;
	struct mbox_controller *mbox;
	struct mbox_chan *chans;
	int queue_count;
	int i;
	int ret = -EINVAL;

	if (!dev->of_node) {
		dev_err(dev, "no OF information\n");
		return -EINVAL;
	}
	np = dev->of_node;

	of_id = of_match_device(ti_msgmgr_of_match, dev);
	if (!of_id) {
		dev_err(dev, "OF data missing\n");
		return -EINVAL;
	}
	desc = of_id->data;

	inst = devm_kzalloc(dev, sizeof(*inst), GFP_KERNEL);
	if (!inst)
		return -ENOMEM;

	inst->dev = dev;
	inst->desc = desc;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					   "queue_proxy_region");
	inst->queue_proxy_region = devm_ioremap_resource(dev, res);
	if (IS_ERR(inst->queue_proxy_region))
		return PTR_ERR(inst->queue_proxy_region);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					   "queue_state_debug_region");
	inst->queue_state_debug_region = devm_ioremap_resource(dev, res);
	if (IS_ERR(inst->queue_state_debug_region))
		return PTR_ERR(inst->queue_state_debug_region);

	dev_dbg(dev, "proxy region=%p, queue_state=%p\n",
		inst->queue_proxy_region, inst->queue_state_debug_region);

	queue_count = of_get_available_child_count(np);
	if (!queue_count) {
		dev_err(dev, "No child nodes representing queues\n");
		return -EINVAL;
	}
	if (queue_count > desc->queue_count) {
		dev_err(dev, "Number of queues %d > max %d\n",
			queue_count, desc->queue_count);
		return -ERANGE;
	}
	inst->num_valid_queues = queue_count;

	qinst = devm_kzalloc(dev, sizeof(*qinst) * queue_count, GFP_KERNEL);
	if (!qinst)
		return -ENOMEM;
	inst->qinsts = qinst;

	chans = devm_kzalloc(dev, sizeof(*chans) * queue_count, GFP_KERNEL);
	if (!chans)
		return -ENOMEM;
	inst->chans = chans;

	for (i = 0, child = NULL; i < queue_count; i++, qinst++, chans++) {
		child = of_get_next_available_child(np, child);
		ret = of_property_read_u32(child, "ti,queue-id",
					   &qinst->queue_id);
		if (ret) {
			dev_err(dev, "Child node %s[idx=%d] has no queue-id\n",
				child->name, i);
			return -EINVAL;
		}
		if (qinst->queue_id > desc->queue_count) {
			dev_err(dev, "Child node %s[idx=%d] queuid %d > %d\n",
				child->name, i, qinst->queue_id,
				desc->queue_count);
			return -ERANGE;
		}

		ret = of_property_read_u32(child, "ti,proxy-id",
					   &qinst->proxy_id);
		if (ret) {
			dev_err(dev, "Child node %s[idx=%d] has no proxy-id\n",
				child->name, i);
			return -EINVAL;
		}

		qinst->irq = of_irq_get_byname(child, "rx");
		/* TX queues don't have IRQ numbers */
		if (qinst->irq < 0) {
			qinst->irq = -1;
			qinst->is_tx = true;
		}

		qinst->queue_buff_start = inst->queue_proxy_region +
		    Q_DATA_OFFSET(qinst->proxy_id, qinst->queue_id,
				  desc->data_first_reg);
		qinst->queue_buff_end = inst->queue_proxy_region +
		    Q_DATA_OFFSET(qinst->proxy_id, qinst->queue_id,
				  desc->data_last_reg);
		qinst->queue_state = inst->queue_state_debug_region +
		    Q_STATE_OFFSET(qinst->queue_id);
		qinst->name = child->name;
		qinst->chan = chans;

		/* Allocate usage buffer for rx */
		if (!qinst->is_tx) {
			qinst->rx_buff = devm_kzalloc(dev,
						      desc->max_message_size,
						      GFP_KERNEL);
			if (!qinst->rx_buff)
				return -ENOMEM;
		}

		chans->con_priv = qinst;

		dev_dbg(dev, "[%d] qidx=%d pidx=%d irq=%d q_s=%p q_e = %p\n",
			i, qinst->queue_id, qinst->proxy_id, qinst->irq,
			qinst->queue_buff_start, qinst->queue_buff_end);
	}

	mbox = &inst->mbox;
	mbox->dev = dev;
	mbox->ops = &ti_msgmgr_chan_ops;
	mbox->chans = inst->chans;
	mbox->num_chans = inst->num_valid_queues;
	mbox->txdone_irq = false;
	mbox->txdone_poll = desc->tx_polled;
	if (desc->tx_polled)
		mbox->txpoll_period = desc->tx_poll_timeout_ms;
	mbox->of_xlate = ti_msgmgr_of_xlate;

	platform_set_drvdata(pdev, inst);
	ret = mbox_controller_register(mbox);
	if (ret)
		dev_err(dev, "Failed to register mbox_controller(%d)\n", ret);

	return ret;
}

static int ti_msgmgr_remove(struct platform_device *pdev)
{
	struct ti_msgmgr_inst *inst;

	inst = platform_get_drvdata(pdev);
	mbox_controller_unregister(&inst->mbox);

	return 0;
}

static struct platform_driver ti_msgmgr_driver = {
	.probe = ti_msgmgr_probe,
	.remove = ti_msgmgr_remove,
	.driver = {
		   .name = "ti-msgmgr",
		   .of_match_table = of_match_ptr(ti_msgmgr_of_match),
	},
};
module_platform_driver(ti_msgmgr_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("TI message manager driver");
MODULE_AUTHOR("Nishanth Menon");
MODULE_ALIAS("platform:ti-msgmgr");
