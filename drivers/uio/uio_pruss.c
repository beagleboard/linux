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
#include <linux/platform_data/uio_pruss.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/of.h>
#ifdef CONFIG_ARCH_DAVINCI_DA850
#include <linux/genalloc.h>
#else
#include <linux/pm_runtime.h>
#endif

#define DRV_NAME "pruss_uio"
#define DRV_VERSION "1.0"

#ifdef CONFIG_ARCH_DAVINCI_DA850
static int sram_pool_sz = SZ_16K;
module_param(sram_pool_sz, int, 0);
MODULE_PARM_DESC(sram_pool_sz, "sram pool size to allocate");
#endif


static int extram_pool_sz = SZ_256K;
module_param(extram_pool_sz, int, 0);
MODULE_PARM_DESC(extram_pool_sz, "external ram pool size to allocate");

/*
 * Host event IRQ numbers from PRUSS - PRUSS can generate up to 8 interrupt
 * events to AINTC of ARM host processor - which can be used for IPC b/w PRUSS
 * firmware and user space application, async notification from PRU firmware
 * to user space application
 * 3	PRU_EVTOUT0
 * 4	PRU_EVTOUT1
 * 5	PRU_EVTOUT2
 * 6	PRU_EVTOUT3
 * 7	PRU_EVTOUT4
 * 8	PRU_EVTOUT5
 * 9	PRU_EVTOUT6
 * 10	PRU_EVTOUT7
*/
#define MAX_PRUSS_EVT	8

#define PINTC_HIDISR	0x0038
#define PINTC_HIPIR	0x0900
#define HIPIR_NOPEND	0x80000000
#define PINTC_HIER	0x1500

struct uio_pruss_dev {
	struct uio_info info[MAX_PRUSS_EVT];
	dma_addr_t ddr_paddr;
	void __iomem *prussio_vaddr;
	void *ddr_vaddr;
	unsigned int pintc_base;
#ifdef CONFIG_ARCH_DAVINCI_DA850
	struct clk *pruss_clk;
	struct gen_pool *sram_pool;
	dma_addr_t sram_paddr;
	unsigned long sram_vaddr;
#endif
};

static irqreturn_t pruss_handler(int irq, struct uio_info *info)
{
	struct uio_pruss_dev *gdev = info->priv;
	int intr_bit = 2 + (info - gdev->info);
	int val, intr_mask = (1 << intr_bit);
	void __iomem *base = gdev->prussio_vaddr + gdev->pintc_base;
	void __iomem *intren_reg = base + PINTC_HIER;
	void __iomem *intrdis_reg = base + PINTC_HIDISR;
	void __iomem *intrstat_reg = base + PINTC_HIPIR + (intr_bit << 2);

	val = ioread32(intren_reg);
	/* Is interrupt enabled and active ? */
	if (!(val & intr_mask) && (ioread32(intrstat_reg) & HIPIR_NOPEND))
		return IRQ_NONE;
	/* Disable interrupt */
	iowrite32(intr_bit, intrdis_reg);
	return IRQ_HANDLED;
}

static void pruss_cleanup(struct device *dev, struct uio_pruss_dev *gdev)
{
	int i;

	for (i = 0; i < MAX_PRUSS_EVT; i++) {
		uio_unregister_device(&gdev->info[i]);
		kfree(gdev->info[i].name);
	}
	iounmap(gdev->prussio_vaddr);
	if (gdev->ddr_vaddr) {
		dma_free_coherent(dev, extram_pool_sz, gdev->ddr_vaddr,
			gdev->ddr_paddr);
	}
#ifdef CONFIG_ARCH_DAVINCI_DA850
	if (gdev->sram_vaddr)
		gen_pool_free(gdev->sram_pool,
			      gdev->sram_vaddr,
			      sram_pool_sz);
	clk_disable(gdev->pruss_clk);
	clk_put(gdev->pruss_clk);
#else
	pm_runtime_put(dev);
	pm_runtime_disable(dev);
#endif
	kfree(gdev);
}

static int pruss_probe(struct platform_device *pdev)
{
	struct uio_pruss_dev *gdev;
	struct resource *regs_prussio;
	struct device *dev = &pdev->dev;
	int ret, i, len;
	struct uio_pruss_pdata *pdata = dev_get_platdata(dev);

	gdev = kzalloc(sizeof(struct uio_pruss_dev), GFP_KERNEL);
	if (!gdev)
		return -ENOMEM;

#ifdef CONFIG_ARCH_DAVINCI_DA850
	/* Power on PRU in case its not done as part of boot-loader */
	gdev->pruss_clk = clk_get(dev, "pruss");
	if (IS_ERR(gdev->pruss_clk)) {
		dev_err(dev, "Failed to get clock\n");
		ret = PTR_ERR(gdev->pruss_clk);
		goto err_free_gdev;
	}

	ret = clk_enable(gdev->pruss_clk);
	if (ret) {
		dev_err(dev, "Failed to enable clock\n");
		goto err_clk_put;
	}
#else
	pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		dev_err(dev, "pm_runtime_get_sync() failed\n");
		pm_runtime_disable(dev);
		goto err_free_gdev;
	}
#endif

	regs_prussio = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs_prussio) {
		dev_err(dev, "No PRUSS I/O resource specified\n");
		ret = -EIO;
		goto err_clk_disable;
	}

	if (!regs_prussio->start) {
		dev_err(dev, "Invalid memory resource\n");
		ret = -EIO;
		goto err_clk_disable;
	}

	if (dev->of_node) {
		ret = of_property_read_u32(dev->of_node,
					   "ti,pintc-offset",
					   &gdev->pintc_base);
		if (ret < 0) {
			dev_err(dev, "Can't parse ti,pintc-offset property\n");
			goto err_clk_disable;
		}
	} else
		gdev->pintc_base = pdata->pintc_base;

#ifdef CONFIG_ARCH_DAVINCI_DA850
	if (pdata->sram_pool) {
		gdev->sram_pool = pdata->sram_pool;
		gdev->sram_vaddr =
			(unsigned long)gen_pool_dma_alloc(gdev->sram_pool,
					sram_pool_sz, &gdev->sram_paddr);
		if (!gdev->sram_vaddr) {
			dev_err(dev, "Could not allocate SRAM pool\n");
			ret = -ENOMEM;
			goto err_clk_disable;
		}
	}
#endif

	gdev->ddr_vaddr = dma_alloc_coherent(dev, extram_pool_sz,
				&gdev->ddr_paddr, GFP_KERNEL | GFP_DMA);
	if (!gdev->ddr_vaddr) {
		dev_err(dev, "Could not allocate external memory\n");
		ret = -ENOMEM;
		goto err_free_sram;
	}

	len = resource_size(regs_prussio);
	gdev->prussio_vaddr = ioremap(regs_prussio->start, len);
	if (!gdev->prussio_vaddr) {
		dev_err(dev, "Can't remap PRUSS I/O address range\n");
		ret = -ENOMEM;
		goto err_free_ddr_vaddr;
	}

	for (i = 0; i < MAX_PRUSS_EVT; i++) {
		struct uio_info *p = &gdev->info[i];

		p->mem[0].name = "pruss";
		p->mem[0].addr = regs_prussio->start;
		p->mem[0].size = resource_size(regs_prussio);
#ifdef CONFIG_ARCH_K3
		p->mem[0].memtype = UIO_MEM_IOVA;
#else
		p->mem[0].memtype = UIO_MEM_PHYS;
#endif

		/* note: some userspace code uses hardcoded mem indices... */
#ifdef CONFIG_ARCH_DAVINCI_DA850
		p->mem[1].name = "sram";
		p->mem[1].addr = gdev->sram_paddr;
		p->mem[1].size = sram_pool_sz;
		p->mem[1].memtype = UIO_MEM_PHYS;

		p->mem[2].name = "ddr";
		p->mem[2].addr = gdev->ddr_paddr;
		p->mem[2].size = extram_pool_sz;
		p->mem[2].memtype = UIO_MEM_PHYS;
#else
		p->mem[1].name = "ddr";
		p->mem[1].addr = gdev->ddr_paddr;
		p->mem[1].size = extram_pool_sz;
#ifdef CONFIG_ARCH_K3
		p->mem[1].memtype = UIO_MEM_IOVA;
#else
		p->mem[1].memtype = UIO_MEM_PHYS;
#endif
		p->mem[1].memtype = UIO_MEM_PHYS;
#endif

		ret = platform_get_irq(pdev, i);
		if (ret < 0) {
			dev_err(dev, "Failed to obtain irq %d (%d)\n", i, ret);
			goto err_unloop;
		}

		p->name = kasprintf(GFP_KERNEL, "pruss_evt%d", i);
		p->version = DRV_VERSION;

		/* Register PRUSS IRQ lines */
		p->irq = ret;
		p->handler = pruss_handler;
		p->priv = gdev;

		ret = uio_register_device(dev, p);
		if (ret < 0) {
			kfree(p->name);
			goto err_unloop;
		}
	}

	platform_set_drvdata(pdev, gdev);
	return 0;

err_unloop:
	while( --i >= 0 ) {
		uio_unregister_device(&gdev->info[i]);
		kfree(gdev->info[i].name);
	}
	iounmap(gdev->prussio_vaddr);
err_free_ddr_vaddr:
	dma_free_coherent(dev, extram_pool_sz, gdev->ddr_vaddr,
			  gdev->ddr_paddr);
err_free_sram:
#ifdef CONFIG_ARCH_DAVINCI_DA850
	if (pdata->sram_pool)
		gen_pool_free(gdev->sram_pool, gdev->sram_vaddr, sram_pool_sz);
err_clk_disable:
	clk_disable(gdev->pruss_clk);
err_clk_put:
	clk_put(gdev->pruss_clk);
#else
err_clk_disable:
	pm_runtime_put(dev);
	pm_runtime_disable(dev);
#endif
err_free_gdev:
	kfree(gdev);

	return ret;
}

static int pruss_remove(struct platform_device *dev)
{
	struct uio_pruss_dev *gdev = platform_get_drvdata(dev);

	pruss_cleanup(&dev->dev, gdev);
	return 0;
}

static const struct of_device_id pruss_dt_ids[] = {
	{ .compatible = "ti,pruss-v1" },
	{ .compatible = "ti,pruss-v2" },
	{},
};
MODULE_DEVICE_TABLE(of, pruss_dt_ids);

static struct platform_driver pruss_driver = {
	.probe = pruss_probe,
	.remove = pruss_remove,
	.driver = {
		   .name = DRV_NAME,
		   .of_match_table = pruss_dt_ids,
		   },
};

module_platform_driver(pruss_driver);

MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRV_VERSION);
MODULE_AUTHOR("Amit Chatterjee <amit.chatterjee@ti.com>");
MODULE_AUTHOR("Pratheesh Gangadhar <pratheesh@ti.com>");
