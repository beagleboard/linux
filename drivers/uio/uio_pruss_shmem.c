/*
 * Programmable Real-Time Unit Sub System (PRUSS) UIO driver (uio_pruss)
 *
 * This driver exports PRUSS host event out interrupts and PRUSS, L3 RAM,
 * and DDR RAM to user space for applications interacting with PRUSS firmware
 *
 * Copyright (C) 2010-11 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/device.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/uio_driver.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/genalloc.h>
#include <linux/of_address.h>
#include <linux/of_device.h>

#define DRV_NAME "pruss_uio_shmem"
#define DRV_VERSION "1.0"

struct uio_pruss_shmem_dev {
	struct uio_info *info;
	void __iomem *prussio_vaddr;
};

static void pruss_shmem_cleanup(struct device *dev, struct uio_pruss_shmem_dev *gdev)
{
	struct uio_info *p = gdev->info;

	uio_unregister_device(p);
	iounmap(gdev->prussio_vaddr);
	kfree(gdev->info);
	kfree(gdev);
}

static int pruss_shmem_probe(struct platform_device *pdev)
{
	struct uio_pruss_shmem_dev *gdev;
	struct resource *regs_prussio;
	struct resource res;
	struct device *dev = &pdev->dev;
	int ret, len;
	struct uio_info *p;

	dev_info(dev, "Allocating gdev\n");
	gdev = kzalloc(sizeof(struct uio_pruss_shmem_dev), GFP_KERNEL);
	if (!gdev)
		return -ENOMEM;

	dev_info(dev, "Allocating info\n");
	gdev->info = kzalloc(sizeof(struct uio_info), GFP_KERNEL);
	if (!gdev->info) {
		ret = -ENOMEM;
		goto err_free_gdev;
	}

	dev_info(dev, "Requesting resource\n");
	if (dev->of_node) {
		ret = of_address_to_resource(dev->of_node, 0, &res);
		if (IS_ERR_VALUE(ret)) {
			dev_err(dev, "Failed to parse DT reg\n");
			goto err_free_info;
		}
		regs_prussio = &res;
	} else {
		regs_prussio = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!regs_prussio) {
			dev_err(dev, "No PRUSS I/O resource specified\n");
			goto err_free_info;
		}
	}
	if (!regs_prussio) {
		dev_err(dev, "No PRUSS I/O resource specified\n");
		ret = -EIO;
		goto err_free_info;
	}
	if (!regs_prussio->start) {
		dev_err(dev, "Invalid memory resource\n");
		ret = -EIO;
		goto err_free_info;
	}
	dev_info(dev, "Mapping resource\n");
	len = resource_size(regs_prussio);
	gdev->prussio_vaddr = ioremap(regs_prussio->start, len);
	if (!gdev->prussio_vaddr) {
		dev_err(dev, "Can't remap PRUSS I/O  address range\n");
		ret = -ENOMEM;
		goto err_free_info;
	}

	p = gdev->info;
	p->mem[0].addr = regs_prussio->start;
	p->mem[0].size = resource_size(regs_prussio);
	p->mem[0].memtype = UIO_MEM_PHYS;

	p->name = "pruss_shmem";
	p->version = DRV_VERSION;

	dev_info(dev, "Registering with uio driver\n");
	ret = uio_register_device(dev, p);
	if (ret < 0) {
		goto err_free_register;
	}

	dev_info(dev, "Saving platform data\n");
	platform_set_drvdata(pdev, gdev);
	return 0;

err_free_register:
	uio_unregister_device(p);
	iounmap(gdev->prussio_vaddr);
err_free_info:
	kfree(gdev->info);
err_free_gdev:
	kfree(gdev);

	return ret;
}

static int pruss_shmem_remove(struct platform_device *dev)
{
	struct uio_pruss_shmem_dev *gdev = platform_get_drvdata(dev);

	pruss_shmem_cleanup(&dev->dev, gdev);
	return 0;
}

static const struct of_device_id pruss_shmem_dt_ids[] = {
	{ .compatible = "ti,pruss-shmem", .data = NULL, },
	{},
};
MODULE_DEVICE_TABLE(of, pruss_shmem_dt_ids);

static struct platform_driver pruss_shmem_driver = {
	.probe = pruss_shmem_probe,
	.remove = pruss_shmem_remove,
	.driver = {
		   .name = DRV_NAME,
		   .of_match_table = pruss_shmem_dt_ids,
		   },
};

module_platform_driver(pruss_shmem_driver);

MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRV_VERSION);
MODULE_AUTHOR("Amit Chatterjee <amit.chatterjee@ti.com>");
MODULE_AUTHOR("Pratheesh Gangadhar <pratheesh@ti.com>");
MODULE_AUTHOR("Jason Kridner <jdk@ti.com>");
