// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 Alibaba Group Holding Limited.
 */

#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#define MBOX_MAX_MSG_LEN	28
#define WJ_MBOX_SEND_MAX_MESSAGE_LENGTH 28
#define HEXDUMP_BYTES_PER_LINE	28
#define HEXDUMP_LINE_LEN	((HEXDUMP_BYTES_PER_LINE * 4) + 2)
#define HEXDUMP_MAX_LEN		(HEXDUMP_LINE_LEN *		\
				(MBOX_MAX_MSG_LEN / HEXDUMP_BYTES_PER_LINE))

static struct dentry *root_debugfs_dir;

struct mbox_client_light_device {
	struct device		*dev;
	void __iomem		*tx_mmio;
	void __iomem		*rx_mmio;
	struct mbox_chan	*tx_channel;
	struct mbox_chan	*rx_channel;
	char			*rx_buffer;
	char	*message;
	spinlock_t		lock;
};

static ssize_t mbox_client_light_message_write(struct file *filp,
					      const char __user *userbuf,
					      size_t count, loff_t *ppos)
{
	struct mbox_client_light_device *tdev = filp->private_data;
	void *data;
	int ret;

	if (!tdev->tx_channel) {
		dev_err(tdev->dev, "Channel cannot do Tx\n");
		return -EINVAL;
	}

	if (count > WJ_MBOX_SEND_MAX_MESSAGE_LENGTH)
		count = WJ_MBOX_SEND_MAX_MESSAGE_LENGTH;

	tdev->message = kzalloc(MBOX_MAX_MSG_LEN, GFP_KERNEL);
	if (!tdev->message)
		return -ENOMEM;

	ret = copy_from_user(tdev->message, userbuf, count);
	if (ret) {
		ret = -EFAULT;
		goto out;
	}

	data = tdev->message;
	print_hex_dump(KERN_INFO, __func__, DUMP_PREFIX_NONE, 16, 1, tdev->message, MBOX_MAX_MSG_LEN, true);

	ret = mbox_send_message(tdev->tx_channel, data);
	if (ret < 0)
		dev_err(tdev->dev, "Failed to send message via mailbox\n");

out:
	kfree(tdev->message);
	return ret < 0 ? ret : count;
}

static ssize_t mbox_client_light_message_read(struct file *filp,
					     char __user *userbuf,
					     size_t count, loff_t *ppos)
{
	struct mbox_client_light_device *tdev = filp->private_data;
	unsigned long flags;

	print_hex_dump(KERN_INFO, __func__, DUMP_PREFIX_NONE, 16, 1, tdev->rx_buffer, MBOX_MAX_MSG_LEN, true);
	spin_lock_irqsave(&tdev->lock, flags);
	memset(tdev->rx_buffer, 0, MBOX_MAX_MSG_LEN);
	spin_unlock_irqrestore(&tdev->lock, flags);

	return MBOX_MAX_MSG_LEN;
}

static const struct file_operations mbox_client_light_message_ops = {
	.write	= mbox_client_light_message_write,
	.read	= mbox_client_light_message_read,
	.open	= simple_open,
	.llseek	= generic_file_llseek,
};

static int index_names = 0;
static bool debugfs_dir_created = false;
static const char* file_names[] = {"mbox-client0", "mbox-client1"};

static int mbox_client_light_add_debugfs(struct platform_device *pdev,
					struct mbox_client_light_device *tdev)
{
	if (!debugfs_initialized())
		return 0;

	if (index_names > 2) {
		dev_err(&pdev->dev, "Max device index is 2\n");
		return 0;
	}

	if (!debugfs_dir_created) {
		root_debugfs_dir = debugfs_create_dir("mailbox",NULL);
		if (!root_debugfs_dir) {
			dev_err(&pdev->dev,
				"Failed to create mailbox debugfs\n");
			return -EINVAL;
		}
		debugfs_dir_created = true;
	}

	debugfs_create_file(file_names[index_names], 0600, root_debugfs_dir,
			    tdev, &mbox_client_light_message_ops);

	index_names++;
	return 0;
}

static void mbox_client_light_receive_message(struct mbox_client *client,
					     void *message)
{
	struct mbox_client_light_device *tdev = dev_get_drvdata(client->dev);
	char *data = message;

	spin_lock(&tdev->lock);
	memcpy(tdev->rx_buffer, data, MBOX_MAX_MSG_LEN);
	spin_unlock(&tdev->lock);
	print_hex_dump(KERN_INFO, __func__, DUMP_PREFIX_NONE, 16, 1, tdev->rx_buffer, MBOX_MAX_MSG_LEN, true);
}

static struct mbox_chan *
mbox_client_light_request_channel(struct platform_device *pdev,
				 const char *name)
{
	struct mbox_client *client;
	struct mbox_chan *channel;

	client = devm_kzalloc(&pdev->dev, sizeof(*client), GFP_KERNEL);
	if (!client)
		return ERR_PTR(-ENOMEM);

	client->dev		= &pdev->dev;
	client->tx_block	= true;
	client->knows_txdone	= false;
	client->tx_tout		= 500;
	client->rx_callback	= mbox_client_light_receive_message;

	channel = mbox_request_channel_byname(client, name);
	if (IS_ERR(channel)) {
		devm_kfree(&pdev->dev, client);
		dev_warn(&pdev->dev, "Failed to request %s channel\n", name);
		return NULL;
	}

	return channel;
}

static int mbox_client_light_probe(struct platform_device *pdev)
{
	struct mbox_client_light_device *tdev;
	int ret;
	static int chan_idx = 0;

	tdev = devm_kzalloc(&pdev->dev, sizeof(*tdev), GFP_KERNEL);
	if (!tdev)
		return -ENOMEM;

	if (!chan_idx)
		tdev->tx_channel = mbox_client_light_request_channel(pdev, "902");
	else
		tdev->tx_channel = mbox_client_light_request_channel(pdev, "906");
	if (!tdev->tx_channel) {
		dev_err(&pdev->dev, "Request channel failed\n");
		return -EPROBE_DEFER;
	}
	chan_idx++;
	/* In fact, rx_channel is same with tx_channel in C-SKY's mailbox */
	tdev->rx_channel = tdev->tx_channel;

	tdev->dev = &pdev->dev;
	platform_set_drvdata(pdev, tdev);

	spin_lock_init(&tdev->lock);

	tdev->rx_buffer = devm_kzalloc(&pdev->dev,
					MBOX_MAX_MSG_LEN, GFP_KERNEL);
	if (!tdev->rx_buffer)
		return -ENOMEM;

	ret = mbox_client_light_add_debugfs(pdev, tdev);
	if (ret)
		return ret;

	dev_info(&pdev->dev, "Successfully registered\n");

	return 0;
}

static int mbox_client_light_remove(struct platform_device *pdev)
{
	struct mbox_client_light_device *tdev = platform_get_drvdata(pdev);

	debugfs_remove_recursive(root_debugfs_dir);

	if (tdev->tx_channel)
		mbox_free_channel(tdev->tx_channel);

	if (tdev->rx_channel && tdev->rx_channel != tdev->tx_channel)
		mbox_free_channel(tdev->rx_channel);

	return 0;
}

static const struct of_device_id mbox_client_light_match[] = {
	{ .compatible = "thead,light-mbox-client" },
	{},
};

static struct platform_driver mbox_client_light_driver = {
	.driver = {
		.name = "thead,light-mbox-client",
		.of_match_table = mbox_client_light_match,
	},
	.probe  = mbox_client_light_probe,
	.remove = mbox_client_light_remove,
};
module_platform_driver(mbox_client_light_driver);

MODULE_AUTHOR("Alibaba Group Holding Limited");
MODULE_DESCRIPTION("Thead Light mailbox IPC client driver");
MODULE_LICENSE("GPL v2");
