// SPDX-License-Identifier: GPL-2.0+

#include <linux/debugfs.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/virtio_mailbox.h>

static bool tx_block = true;
module_param(tx_block, bool, 0660);

static bool debug = false;
module_param(debug, bool, 0660);

static char *chan_name = "vchan0";
module_param(chan_name, charp, 0);

static int MBOX_MAX_MSG_LEN = VIRTIO_MAILBOX_MSG_SIZE_DEFAULT;
module_param(MBOX_MAX_MSG_LEN, int, 0660);

static struct dentry *root_debugfs_dir;

struct mbox_client_device {
	struct device		*dev;
	void __iomem		*tx_mmio;
	void __iomem		*rx_mmio;
	struct mbox_chan	*channel;
	char			*rx_buffer;
	char			*message;
	spinlock_t		lock;
};

static ssize_t mbox_client_message_write(struct file *filp,
					      const char __user *userbuf,
					      size_t count, loff_t *ppos)
{
	struct mbox_client_device *tdev = filp->private_data;
	void *data;
	int ret;

	if (!tdev->channel) {
		dev_err(tdev->dev, "Channel cannot do Tx\n");
		return -EINVAL;
	}

	if (count > MBOX_MAX_MSG_LEN)
		count = MBOX_MAX_MSG_LEN;

	tdev->message = kzalloc(MBOX_MAX_MSG_LEN, GFP_KERNEL);
	if (!tdev->message)
		return -ENOMEM;

	ret = copy_from_user(tdev->message, userbuf, count);
	if (ret) {
		ret = -EFAULT;
		goto out;
	}

	data = tdev->message;

	if (debug)
		print_hex_dump(KERN_INFO, "Xmit: ", DUMP_PREFIX_NONE, 16, 1,
				tdev->message, MBOX_MAX_MSG_LEN, true);

	ret = mbox_send_message(tdev->channel, data);
	if (ret < 0)
		dev_err(tdev->dev, "Failed to send message via mailbox(%d)\n", ret);

out:
	kfree(tdev->message);
	return ret < 0 ? ret : count;
}

static ssize_t mbox_client_message_read(struct file *filp,
					     char __user *userbuf,
					     size_t count, loff_t *ppos)
{
	struct mbox_client_device *tdev = filp->private_data;
	unsigned long flags;

	if (debug)
		print_hex_dump(KERN_INFO, "Read: ", DUMP_PREFIX_NONE, 16, 1,
				tdev->rx_buffer, MBOX_MAX_MSG_LEN, true);

	spin_lock_irqsave(&tdev->lock, flags);
	memset(tdev->rx_buffer, 0, MBOX_MAX_MSG_LEN);
	spin_unlock_irqrestore(&tdev->lock, flags);

	return MBOX_MAX_MSG_LEN;
}

static const struct file_operations mbox_client_message_ops = {
	.write	= mbox_client_message_write,
	.read	= mbox_client_message_read,
	.open	= simple_open,
	.llseek	= generic_file_llseek,
};

static int index_names = 0;
static bool debugfs_dir_created = false;
static const char* file_names[] = {"mbox-client0", "mbox-client1"};

static int mbox_client_add_debugfs(struct platform_device *pdev,
					struct mbox_client_device *tdev)
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
			    tdev, &mbox_client_message_ops);

	index_names++;
	return 0;
}

static void mbox_client_receive_message(struct mbox_client *client,
					     void *message)
{
	struct mbox_client_device *tdev = dev_get_drvdata(client->dev);
	char *data = message;

	spin_lock(&tdev->lock);
	memcpy(tdev->rx_buffer, data, MBOX_MAX_MSG_LEN);
	spin_unlock(&tdev->lock);

	if (debug)
		print_hex_dump(KERN_INFO, "Recv: ", DUMP_PREFIX_NONE, 16, 1,
				tdev->rx_buffer, MBOX_MAX_MSG_LEN, true);
}

static void mbox_client_tx_done(struct mbox_client *cl, void *msg, int ret)
{
	//printk("tx_done\n");
}

static struct mbox_chan * mbox_client_request_channel(struct platform_device *pdev,
				 const char *name)
{
	struct mbox_client *client;
	struct mbox_chan *channel;

	client = devm_kzalloc(&pdev->dev, sizeof(*client), GFP_KERNEL);
	if (!client)
		return ERR_PTR(-ENOMEM);

	client->dev		= &pdev->dev;
	client->tx_block	= tx_block;
	client->knows_txdone	= false;
	client->tx_tout		= 1000;
	client->tx_done		= mbox_client_tx_done;
	client->rx_callback	= mbox_client_receive_message;

	channel = mbox_request_channel_byname(client, name);
	if (IS_ERR(channel)) {
		devm_kfree(&pdev->dev, client);
		dev_warn(&pdev->dev, "Failed to request %s channel\n", name);
		return NULL;
	}

	return channel;
}

static int mbox_client_probe(struct platform_device *pdev)
{
	struct mbox_client_device *tdev;
	int ret;

	tdev = devm_kzalloc(&pdev->dev, sizeof(*tdev), GFP_KERNEL);
	if (!tdev)
		return -ENOMEM;

	tdev->channel = mbox_client_request_channel(pdev, chan_name);
	if (!tdev->channel) {
		dev_err(&pdev->dev, "Request channel failed\n");
		return -EPROBE_DEFER;
	}

	tdev->dev = &pdev->dev;
	platform_set_drvdata(pdev, tdev);

	spin_lock_init(&tdev->lock);

	tdev->rx_buffer = devm_kzalloc(&pdev->dev,
					MBOX_MAX_MSG_LEN, GFP_KERNEL);
	if (!tdev->rx_buffer)
		return -ENOMEM;

	ret = mbox_client_add_debugfs(pdev, tdev);
	if (ret)
		return ret;

	dev_info(&pdev->dev, "Successfully registered\n");

	return 0;
}

static int mbox_client_remove(struct platform_device *pdev)
{
	struct mbox_client_device *tdev = platform_get_drvdata(pdev);

	debugfs_remove_recursive(root_debugfs_dir);

	if (tdev->channel)
		mbox_free_channel(tdev->channel);

	return 0;
}

static const struct of_device_id mbox_client_match[] = {
	{ .compatible = "thead,vmbox-client" },
	{},
};

static struct platform_driver mbox_client_driver = {
	.driver = {
		.name = "vmbox-client",
		.of_match_table = mbox_client_match,
	},
	.probe  = mbox_client_probe,
	.remove = mbox_client_remove,
};
module_platform_driver(mbox_client_driver);

MODULE_DESCRIPTION("Virtio Mailbox Client test driver");
MODULE_AUTHOR("Xianting Tian <xianting.tian@linux.alibaba.com>");
MODULE_LICENSE("GPL");
