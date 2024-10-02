// SPDX-License-Identifier: GPL-2.0-only
/*
 * Character device interface driver for Remoteproc framework.
 *
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 */

#include <linux/cdev.h>
#include <linux/compat.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/remoteproc.h>
#include <linux/uaccess.h>
#include <uapi/linux/remoteproc_cdev.h>

#include "remoteproc_internal.h"

#define NUM_RPROC_DEVICES	64
static dev_t rproc_major;

struct rproc_cdev {
	struct rproc *rproc;
	bool cdev_put_on_release;
	/* Protects the attachments lists */
	struct mutex mutex;
	struct list_head attachments;
};

struct rproc_cdev_attach {
	struct dma_buf *dmabuf;
	struct list_head node;
};

static ssize_t rproc_cdev_write(struct file *filp, const char __user *buf, size_t len, loff_t *pos)
{
	struct rproc_cdev *rproc_cdev = filp->private_data;
	struct rproc *rproc = rproc_cdev->rproc;
	int ret = 0;
	char cmd[10];

	if (!len || len > sizeof(cmd))
		return -EINVAL;

	ret = copy_from_user(cmd, buf, len);
	if (ret)
		return -EFAULT;

	if (!strncmp(cmd, "start", len)) {
		ret = rproc_boot(rproc);
	} else if (!strncmp(cmd, "stop", len)) {
		ret = rproc_shutdown(rproc);
	} else if (!strncmp(cmd, "detach", len)) {
		ret = rproc_detach(rproc);
	} else {
		dev_err(&rproc->dev, "Unrecognized option\n");
		ret = -EINVAL;
	}

	return ret ? ret : len;
}

static long rproc_device_ioctl(struct file *filp, unsigned int ioctl, unsigned long arg)
{
	struct rproc_cdev *rproc_cdev = filp->private_data;
	struct rproc *rproc = rproc_cdev->rproc;
	void __user *argp = (void __user *)arg;
	s32 param;

	switch (ioctl) {
	case RPROC_SET_SHUTDOWN_ON_RELEASE:
		if (copy_from_user(&param, argp, sizeof(s32)))
			return -EFAULT;

		rproc_cdev->cdev_put_on_release = !!param;
		break;
	case RPROC_GET_SHUTDOWN_ON_RELEASE:
		param = (s32)rproc_cdev->cdev_put_on_release;
		if (copy_to_user(argp, &param, sizeof(s32)))
			return -EFAULT;

		break;
	case RPROC_IOC_DMA_BUF_ATTACH:
		{
			struct rproc_dma_buf_attach_data data;
			struct rproc_cdev_attach *attach;
			struct dma_buf *dmabuf;
			dma_addr_t da;
			int ret;

			if (copy_from_user(&data, (void __user *)arg, _IOC_SIZE(ioctl)))
				return -EFAULT;

			dmabuf = dma_buf_get(data.fd);
			if (IS_ERR(dmabuf))
				return PTR_ERR(dmabuf);

			ret = rproc_attach_dmabuf(rproc, dmabuf);
			if (ret) {
				dma_buf_put(dmabuf);
				return ret;
			}

			ret = rproc_dmabuf_get_da(rproc, dmabuf, &da);
			if (ret) {
				rproc_detach_dmabuf(rproc, dmabuf);
				dma_buf_put(dmabuf);
				return ret;
			}
			data.da = da;

			/* Save for later removal */
			attach = kzalloc(sizeof(*attach), GFP_KERNEL);
			if (!attach) {
				rproc_detach_dmabuf(rproc, dmabuf);
				dma_buf_put(dmabuf);
				return -ENOMEM;
			}
			attach->dmabuf = dmabuf;
			list_add_tail(&attach->node, &rproc_cdev->attachments);

			if (copy_to_user((void __user *)arg, &data, _IOC_SIZE(ioctl)))
				return -EFAULT;
		}
		break;
	default:
		dev_err(&rproc->dev, "Unsupported ioctl\n");
		return -EINVAL;
	}

	return 0;
}

static int rproc_cdev_open(struct inode *inode, struct file *file)
{
	struct rproc *rproc = container_of(inode->i_cdev, struct rproc, cdev);
	struct rproc_cdev *rproc_cdev;

	rproc_cdev = kzalloc(sizeof(*rproc_cdev), GFP_KERNEL);
	if (!rproc_cdev)
		return -ENOMEM;

	rproc_cdev->rproc = rproc;
	mutex_init(&rproc_cdev->mutex);
	INIT_LIST_HEAD(&rproc_cdev->attachments);

	file->private_data = rproc_cdev;

	return 0;
}

static int rproc_cdev_release(struct inode *inode, struct file *filp)
{
	struct rproc_cdev *rproc_cdev = filp->private_data;
	struct rproc *rproc = rproc_cdev->rproc;
	int ret = 0;

	if (rproc_cdev->cdev_put_on_release) {
		if (rproc->state == RPROC_RUNNING)
			rproc_shutdown(rproc);
		else if (rproc->state == RPROC_ATTACHED)
			ret = rproc_detach(rproc);
	}

	/* Release all buffers attached with this file */
	struct rproc_cdev_attach *attach, *atmp;
	list_for_each_entry_safe(attach, atmp, &rproc_cdev->attachments, node) {
		rproc_detach_dmabuf(rproc, attach->dmabuf);
		dma_buf_put(attach->dmabuf);
		kfree(attach);
	}
	mutex_destroy(&rproc_cdev->mutex);
	kfree(rproc_cdev);

	return ret;
}

static const struct file_operations rproc_fops = {
	.write = rproc_cdev_write,
	.unlocked_ioctl = rproc_device_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
	.open = rproc_cdev_open,
	.release = rproc_cdev_release,
};

int rproc_char_device_add(struct rproc *rproc)
{
	int ret;

	cdev_init(&rproc->cdev, &rproc_fops);
	rproc->cdev.owner = THIS_MODULE;

	rproc->dev.devt = MKDEV(MAJOR(rproc_major), rproc->index);
	cdev_set_parent(&rproc->cdev, &rproc->dev.kobj);
	ret = cdev_add(&rproc->cdev, rproc->dev.devt, 1);
	if (ret < 0)
		dev_err(&rproc->dev, "Failed to add char dev for %s\n", rproc->name);

	return ret;
}

void rproc_char_device_remove(struct rproc *rproc)
{
	cdev_del(&rproc->cdev);
}

void __init rproc_init_cdev(void)
{
	int ret;

	ret = alloc_chrdev_region(&rproc_major, 0, NUM_RPROC_DEVICES, "remoteproc");
	if (ret < 0)
		pr_err("Failed to alloc rproc_cdev region, err %d\n", ret);
}
