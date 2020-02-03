// SPDX-License-Identifier: GPL-2.0
/*
 * DMA-BUF contiguous buffer physical address user-space exporter
 *
 * Copyright (C) 2018-2019 Texas Instruments Incorporated - http://www.ti.com/
 *	Andrew F. Davis <afd@ti.com>
 */

#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include <uapi/linux/dma_buf_phys.h>

#define DEVICE_NAME "dma-buf-phys"

struct dma_buf_phys_priv {
	struct miscdevice miscdev;
};

struct dma_buf_phys_file {
	struct device *dev;
	struct dma_buf *dma_buf;
	struct dma_buf_attachment *attachment;
	struct sg_table *sgt;
};

static int dma_buf_phys_open(struct inode *inode, struct file *file)
{
	struct miscdevice *miscdev = file->private_data;
	struct device *dev = miscdev->this_device;
	struct dma_buf_phys_file *priv;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->dev = dev;
	file->private_data = (void *)priv;

	return 0;
}

static int dma_buf_phys_release(struct inode *inode, struct file *file)
{
	struct dma_buf_phys_file *priv = file->private_data;

	if (priv->attachment && priv->sgt)
		dma_buf_unmap_attachment(priv->attachment, priv->sgt, DMA_BIDIRECTIONAL);
	if (priv->dma_buf && priv->attachment)
		dma_buf_detach(priv->dma_buf, priv->attachment);
	if (priv->dma_buf)
		dma_buf_put(priv->dma_buf);

	kfree(priv);

	return 0;
}

static int dma_buf_phys_convert(struct dma_buf_phys_file *priv, int fd, u64 *phys)
{
	struct device *dev = priv->dev;
	struct dma_buf *dma_buf;
	struct dma_buf_attachment *attachment;
	struct sg_table *sgt;
	dma_addr_t dma_addr;
	int ret;

	dma_buf = dma_buf_get(fd);
	if (IS_ERR(dma_buf))
		return PTR_ERR(dma_buf);

	/* Attach as the parent device as it will have the correct DMA ops set */
	attachment = dma_buf_attach(dma_buf, dev->parent);
	if (IS_ERR(attachment)) {
		ret = PTR_ERR(attachment);
		goto fail_put;
	}

	sgt = dma_buf_map_attachment(attachment, DMA_BIDIRECTIONAL);
	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);
		goto fail_detach;
	}

	/* Without PAT only physically contiguous buffers can be supported */
	if (sgt->orig_nents != 1) {
		dev_err(dev, "DMA-BUF not contiguous\n");
		ret = -EINVAL;
		goto fail_unmap;
	}

	dma_addr = sg_dma_address(sgt->sgl);

	*phys = dma_addr;

	priv->dma_buf = dma_buf;
	priv->attachment = attachment;
	priv->sgt = sgt;

	return 0;

fail_unmap:
	dma_buf_unmap_attachment(attachment, sgt, DMA_BIDIRECTIONAL);
fail_detach:
	dma_buf_detach(dma_buf, attachment);
fail_put:
	dma_buf_put(dma_buf);

	return ret;
}

static long dma_buf_phys_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct dma_buf_phys_file *priv = file->private_data;

	switch (cmd) {
	case DMA_BUF_PHYS_IOC_CONVERT:
	{
		struct dma_buf_phys_data data;
		int ret;

		/*
		 * TODO: this should likely be properly serialized, but I
		 * see no reason this file would ever need to be shared.
		 */
		/* one attachment per file */
		if (priv->dma_buf)
			return -EFAULT;

		if (copy_from_user(&data, (void __user *)arg, _IOC_SIZE(cmd)))
			return -EFAULT;

		ret = dma_buf_phys_convert(priv, data.fd, &data.phys);
		if (ret)
			return ret;

		if (copy_to_user((void __user *)arg, &data, _IOC_SIZE(cmd)))
			return -EFAULT;

		break;
	}
	default:
		return -ENOTTY;
	}

	return 0;
}

static const struct file_operations dma_buf_phys_fops = {
	.owner = THIS_MODULE,
	.open = dma_buf_phys_open,
	.release = dma_buf_phys_release,
	.unlocked_ioctl = dma_buf_phys_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= dma_buf_phys_ioctl,
#endif
};

static int dma_buf_phys_probe(struct platform_device *pdev)
{
	struct dma_buf_phys_priv *priv;
	struct device *dev = &pdev->dev;
	int err;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	dev_set_drvdata(dev, priv);

	priv->miscdev.minor = MISC_DYNAMIC_MINOR;
	priv->miscdev.name = devm_kasprintf(dev, GFP_KERNEL, "%s", DEVICE_NAME);
	priv->miscdev.fops = &dma_buf_phys_fops;
	priv->miscdev.parent = dev;
	err = misc_register(&priv->miscdev);
	if (err) {
		dev_err(dev, "unable to register DMA-BUF to Phys misc device\n");
		return err;
	}

	return 0;
}

static int dma_buf_phys_remove(struct platform_device *pdev)
{
	struct dma_buf_phys_priv *priv = dev_get_drvdata(&pdev->dev);

	misc_deregister(&priv->miscdev);

	return 0;
}

static const struct of_device_id dma_buf_phys_of_match[] = {
	{ .compatible = "ti,dma_buf_phys", },
	{},
};
MODULE_DEVICE_TABLE(of, dma_buf_phys_of_match);

static struct platform_driver dma_buf_phys_driver = {
	.probe = dma_buf_phys_probe,
	.remove = dma_buf_phys_remove,
	.driver = {
		.name = "dma_buf_phys",
		.of_match_table = dma_buf_phys_of_match,
	}
};

static int __init dma_buf_phys_init(void)
{
	struct platform_device *pdev;
	int ret;

	ret = platform_driver_register(&dma_buf_phys_driver);
	if (ret)
		return ret;

	pdev = platform_device_register_simple("dma_buf_phys", -1, NULL, 0);

	return PTR_ERR_OR_ZERO(pdev);
}
device_initcall(dma_buf_phys_init);

MODULE_AUTHOR("Andrew F. Davis <afd@ti.com>");
MODULE_DESCRIPTION("DMA-BUF contiguous buffer physical address user-space exporter");
MODULE_LICENSE("GPL v2");
