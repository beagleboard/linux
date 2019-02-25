// SPDX-License-Identifier: GPL-2.0
/*
 * PRU Remote Processor Messaging Driver
 *
 * Copyright (C) 2015-2019 Texas Instruments Incorporated - http://www.ti.com/
 *	Jason Reeder <jreeder@ti.com>
 *	Suman Anna <s-anna@ti.com>
 */

#include <linux/kernel.h>
#include <linux/rpmsg.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/kfifo.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/rpmsg/virtio_rpmsg.h>

#define PRU_MAX_DEVICES				(16)
/* Matches the definition in virtio_rpmsg_bus.c */
#define RPMSG_BUF_SIZE				(512)
#define MAX_FIFO_MSG				(32)
#define FIFO_MSG_SIZE				RPMSG_BUF_SIZE

/**
 * struct rpmsg_pru_dev - Structure that contains the per-device data
 * @rpdev: rpmsg channel device that is associated with this rpmsg_pru device
 * @dev: device
 * @cdev: character device
 * @locked: boolean used to determine whether or not the device file is in use
 * @devt: dev_t structure for the rpmsg_pru device
 * @msg_fifo: kernel fifo used to buffer the messages between userspace and PRU
 * @msg_len: array storing the lengths of each message in the kernel fifo
 * @msg_idx_rd: kernel fifo read index
 * @msg_idx_wr: kernel fifo write index
 * @wait_list: wait queue used to implement the poll operation of the character
 *             device
 *
 * Each rpmsg_pru device provides an interface, using an rpmsg channel (rpdev),
 * between a user space character device (cdev) and a PRU core. A kernel fifo
 * (msg_fifo) is used to buffer the messages in the kernel that are
 * being passed between the character device and the PRU.
 */
struct rpmsg_pru_dev {
	struct rpmsg_device *rpdev;
	struct device *dev;
	struct cdev cdev;
	bool locked;
	dev_t devt;
	struct kfifo msg_fifo;
	u32 msg_len[MAX_FIFO_MSG];
	int msg_idx_rd;
	int msg_idx_wr;
	wait_queue_head_t wait_list;
};

static struct class *rpmsg_pru_class;
static dev_t rpmsg_pru_devt;
static DEFINE_MUTEX(rpmsg_pru_lock);
static DEFINE_IDR(rpmsg_pru_minors);

static int rpmsg_pru_open(struct inode *inode, struct file *filp)
{
	struct rpmsg_pru_dev *prudev;
	int ret = -EACCES;

	prudev = container_of(inode->i_cdev, struct rpmsg_pru_dev, cdev);

	mutex_lock(&rpmsg_pru_lock);
	if (!prudev->locked) {
		prudev->locked = true;
		filp->private_data = prudev;
		ret = 0;
	}
	mutex_unlock(&rpmsg_pru_lock);

	if (ret)
		dev_err(prudev->dev, "Device already open\n");

	return ret;
}

static int rpmsg_pru_release(struct inode *inode, struct file *filp)
{
	struct rpmsg_pru_dev *prudev;

	prudev = container_of(inode->i_cdev, struct rpmsg_pru_dev, cdev);
	mutex_lock(&rpmsg_pru_lock);
	prudev->locked = false;
	mutex_unlock(&rpmsg_pru_lock);
	return 0;
}

static ssize_t rpmsg_pru_read(struct file *filp, char __user *buf,
			      size_t count, loff_t *f_pos)
{
	int ret;
	u32 length;
	struct rpmsg_pru_dev *prudev;

	prudev = filp->private_data;

	if (kfifo_is_empty(&prudev->msg_fifo) &&
	    (filp->f_flags & O_NONBLOCK))
		return -EAGAIN;

	ret = wait_event_interruptible(prudev->wait_list,
				       !kfifo_is_empty(&prudev->msg_fifo));
	if (ret)
		return -EINTR;

	ret = kfifo_to_user(&prudev->msg_fifo, buf,
			    prudev->msg_len[prudev->msg_idx_rd], &length);
	prudev->msg_idx_rd = (prudev->msg_idx_rd + 1) % MAX_FIFO_MSG;

	return ret ? ret : length;
}

static ssize_t rpmsg_pru_write(struct file *filp, const char __user *buf,
			       size_t count, loff_t *f_pos)
{
	int ret;
	struct rpmsg_pru_dev *prudev;
	static char rpmsg_pru_buf[RPMSG_BUF_SIZE];

	prudev = filp->private_data;

	if (count > RPMSG_BUF_SIZE - sizeof(struct rpmsg_hdr)) {
		dev_err(prudev->dev, "Data too large for RPMsg Buffer\n");
		return -EINVAL;
	}

	if (copy_from_user(rpmsg_pru_buf, buf, count)) {
		dev_err(prudev->dev, "Error copying buffer from user space");
		return -EFAULT;
	}

	ret = rpmsg_send(prudev->rpdev->ept, (void *)rpmsg_pru_buf, count);
	if (ret)
		dev_err(prudev->dev, "rpmsg_send failed: %d\n", ret);

	return ret ? ret : count;
}

static unsigned int rpmsg_pru_poll(struct file *filp,
				   struct poll_table_struct *wait)
{
	int mask;
	struct rpmsg_pru_dev *prudev;

	prudev = filp->private_data;

	poll_wait(filp, &prudev->wait_list, wait);

	mask = POLLOUT | POLLWRNORM;

	if (!kfifo_is_empty(&prudev->msg_fifo))
		mask |= POLLIN | POLLRDNORM;

	return mask;
}

static const struct file_operations rpmsg_pru_fops = {
	.owner = THIS_MODULE,
	.open = rpmsg_pru_open,
	.release = rpmsg_pru_release,
	.read = rpmsg_pru_read,
	.write = rpmsg_pru_write,
	.poll = rpmsg_pru_poll,
};

static int rpmsg_pru_cb(struct rpmsg_device *rpdev, void *data, int len,
			void *priv, u32 src)
{
	u32 length;
	struct rpmsg_pru_dev *prudev;

	prudev = dev_get_drvdata(&rpdev->dev);

	if (kfifo_avail(&prudev->msg_fifo) < len) {
		dev_err(&rpdev->dev, "Not enough space on the FIFO\n");
		return -ENOSPC;
	}

	if ((prudev->msg_idx_wr + 1) % MAX_FIFO_MSG ==
		prudev->msg_idx_rd) {
		dev_err(&rpdev->dev, "Message length table is full\n");
		return -ENOSPC;
	}

	length = kfifo_in(&prudev->msg_fifo, data, len);
	prudev->msg_len[prudev->msg_idx_wr] = length;
	prudev->msg_idx_wr = (prudev->msg_idx_wr + 1) % MAX_FIFO_MSG;

	wake_up_interruptible(&prudev->wait_list);

	return 0;
}

static int rpmsg_pru_probe(struct rpmsg_device *rpdev)
{
	int ret;
	struct rpmsg_pru_dev *prudev;
	int minor_got;

	prudev = devm_kzalloc(&rpdev->dev, sizeof(*prudev), GFP_KERNEL);
	if (!prudev)
		return -ENOMEM;

	mutex_lock(&rpmsg_pru_lock);
	minor_got = idr_alloc(&rpmsg_pru_minors, prudev, 0, PRU_MAX_DEVICES,
			      GFP_KERNEL);
	mutex_unlock(&rpmsg_pru_lock);
	if (minor_got < 0) {
		ret = minor_got;
		dev_err(&rpdev->dev, "Failed to get a minor number for the rpmsg_pru device: %d\n",
			ret);
		goto fail_alloc_minor;
	}

	prudev->devt = MKDEV(MAJOR(rpmsg_pru_devt), minor_got);

	cdev_init(&prudev->cdev, &rpmsg_pru_fops);
	prudev->cdev.owner = THIS_MODULE;
	ret = cdev_add(&prudev->cdev, prudev->devt, 1);
	if (ret) {
		dev_err(&rpdev->dev, "Unable to add cdev for the rpmsg_pru device\n");
		goto fail_add_cdev;
	}

	prudev->dev = device_create(rpmsg_pru_class, &rpdev->dev, prudev->devt,
				    NULL, "rpmsg_pru%d", rpdev->dst);
	if (IS_ERR(prudev->dev)) {
		dev_err(&rpdev->dev, "Unable to create the rpmsg_pru device\n");
		ret = PTR_ERR(prudev->dev);
		goto fail_create_device;
	}

	prudev->rpdev = rpdev;

	ret = kfifo_alloc(&prudev->msg_fifo, MAX_FIFO_MSG * FIFO_MSG_SIZE,
			  GFP_KERNEL);
	if (ret) {
		dev_err(&rpdev->dev, "Unable to allocate fifo for the rpmsg_pru device\n");
		goto fail_alloc_fifo;
	}

	init_waitqueue_head(&prudev->wait_list);

	dev_set_drvdata(&rpdev->dev, prudev);

	dev_info(&rpdev->dev, "new rpmsg_pru device: /dev/rpmsg_pru%d",
		 rpdev->dst);

	return 0;

fail_alloc_fifo:
	device_destroy(rpmsg_pru_class, prudev->devt);
fail_create_device:
	cdev_del(&prudev->cdev);
fail_add_cdev:
	mutex_lock(&rpmsg_pru_lock);
	idr_remove(&rpmsg_pru_minors, minor_got);
	mutex_unlock(&rpmsg_pru_lock);
fail_alloc_minor:
	return ret;
}

static void rpmsg_pru_remove(struct rpmsg_device *rpdev)
{
	struct rpmsg_pru_dev *prudev;

	prudev = dev_get_drvdata(&rpdev->dev);

	kfifo_free(&prudev->msg_fifo);
	device_destroy(rpmsg_pru_class, prudev->devt);
	cdev_del(&prudev->cdev);
	mutex_lock(&rpmsg_pru_lock);
	idr_remove(&rpmsg_pru_minors, MINOR(prudev->devt));
	mutex_unlock(&rpmsg_pru_lock);
}

/* .name matches on RPMsg Channels and causes a probe */
static const struct rpmsg_device_id rpmsg_driver_pru_id_table[] = {
	{ .name	= "rpmsg-pru" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, rpmsg_driver_pru_id_table);

static struct rpmsg_driver rpmsg_pru_driver = {
	.drv.name	= KBUILD_MODNAME,
	.id_table	= rpmsg_driver_pru_id_table,
	.probe		= rpmsg_pru_probe,
	.callback	= rpmsg_pru_cb,
	.remove		= rpmsg_pru_remove,
};

static int __init rpmsg_pru_init(void)
{
	int ret;

	rpmsg_pru_class = class_create(THIS_MODULE, "rpmsg_pru");
	if (IS_ERR(rpmsg_pru_class)) {
		pr_err("Unable to create class\n");
		ret = PTR_ERR(rpmsg_pru_class);
		goto fail_create_class;
	}

	ret = alloc_chrdev_region(&rpmsg_pru_devt, 0, PRU_MAX_DEVICES,
				  "rpmsg_pru");
	if (ret) {
		pr_err("Unable to allocate chrdev region\n");
		goto fail_alloc_region;
	}

	ret = register_rpmsg_driver(&rpmsg_pru_driver);
	if (ret) {
		pr_err("Unable to register rpmsg driver");
		goto fail_register_rpmsg_driver;
	}

	return 0;

fail_register_rpmsg_driver:
	unregister_chrdev_region(rpmsg_pru_devt, PRU_MAX_DEVICES);
fail_alloc_region:
	class_destroy(rpmsg_pru_class);
fail_create_class:
	return ret;
}

static void __exit rpmsg_pru_exit(void)
{
	unregister_rpmsg_driver(&rpmsg_pru_driver);
	idr_destroy(&rpmsg_pru_minors);
	mutex_destroy(&rpmsg_pru_lock);
	class_destroy(rpmsg_pru_class);
	unregister_chrdev_region(rpmsg_pru_devt, PRU_MAX_DEVICES);
}

module_init(rpmsg_pru_init);
module_exit(rpmsg_pru_exit);

MODULE_AUTHOR("Jason Reeder <jreeder@ti.com>");
MODULE_DESCRIPTION("PRU Remote Processor Messaging Driver");
MODULE_LICENSE("GPL v2");
