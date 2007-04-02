/*
 *  linux/drivers/mtd/onenand/omap2.c
 *
 *  OneNAND driver for OMAP2
 *
 *  Copyright (C) 2005-2006 Nokia Corporation
 *
 *  Author: Jarkko Lavinen <jarkko.lavinen@nokia.com> and Juha Yrjola
 *  IRQ and DMA support written by Timo Teras
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; see the file COPYING. If not, write to the Free Software
 * Foundation, 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/onenand.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include <asm/io.h>
#include <asm/mach/flash.h>
#include <asm/arch/gpmc.h>
#include <asm/arch/onenand.h>
#include <asm/arch/gpio.h>
#include <asm/arch/gpmc.h>
#include <asm/arch/pm.h>

#include <linux/dma-mapping.h>
#include <asm/dma-mapping.h>
#include <asm/arch/dma.h>

#include <asm/arch/board.h>

#define ONENAND_IO_SIZE		SZ_128K
#define ONENAND_BUFRAM_SIZE	(1024 * 5)

struct omap2_onenand {
	struct platform_device *pdev;
	int gpmc_cs;
	unsigned long phys_base;
	int gpio_irq;
	struct mtd_info mtd;
	struct mtd_partition *parts;
	struct onenand_chip onenand;
	struct completion irq_done;
	struct completion dma_done;
	int dma_channel;
};

static unsigned short omap2_onenand_readw(void __iomem *addr)
{
	return readw(addr);
}

static void omap2_onenand_writew(unsigned short value, void __iomem *addr)
{
	writew(value, addr);
}

static void omap2_onenand_dma_cb(int lch, u16 ch_status, void *data)
{
	struct omap2_onenand *info = data;

	complete(&info->dma_done);
}

static irqreturn_t omap2_onenand_interrupt(int irq, void *dev_id)
{
	struct omap2_onenand *info = dev_id;

	complete(&info->irq_done);

	return IRQ_HANDLED;
}

static int omap2_onenand_wait(struct mtd_info *mtd, int state)
{
	struct omap2_onenand *info = container_of(mtd, struct omap2_onenand, mtd);
	unsigned int interrupt = 0;
	unsigned int ctrl;
	unsigned long timeout;
	u32 syscfg;

	if (state == FL_RESETING) {
		udelay(1);
		return 0;
	}

	if (state != FL_READING) {
		int result;
		/* Turn interrupts on */
		syscfg = omap2_onenand_readw(info->onenand.base + ONENAND_REG_SYS_CFG1);
		syscfg |= ONENAND_SYS_CFG1_IOBE;
		omap2_onenand_writew(syscfg, info->onenand.base + ONENAND_REG_SYS_CFG1);

		INIT_COMPLETION(info->irq_done);
		result = omap_get_gpio_datain(info->gpio_irq);
		if (result == -1) {
			ctrl = omap2_onenand_readw(info->onenand.base + ONENAND_REG_CTRL_STATUS);
			printk(KERN_ERR "onenand_wait: gpio error, state = %d, ctrl = 0x%04x\n", state, ctrl);
			return -EIO;
		}
		if (result == 0) {
			int retry_cnt = 0;
retry:
			result = wait_for_completion_timeout(&info->irq_done,
						    msecs_to_jiffies(20));
			if (result == 0) {
				/* Timeout after 20ms */
				ctrl = omap2_onenand_readw(info->onenand.base + ONENAND_REG_CTRL_STATUS);
				if (ctrl & ONENAND_CTRL_ONGO) {
					/* The operation seems to be still going - so give it some more time */
					retry_cnt += 1;
					if (retry_cnt < 3)
						goto retry;
					interrupt = omap2_onenand_readw(info->onenand.base + ONENAND_REG_INTERRUPT);
					printk(KERN_ERR "onenand_wait: timeout state=%d ctrl=0x%04x intr=0x%04x\n", state, ctrl, interrupt);
					return -EIO;
				}
				interrupt = omap2_onenand_readw(info->onenand.base + ONENAND_REG_INTERRUPT);
				if ((interrupt & ONENAND_INT_MASTER) == 0)
					printk(KERN_WARNING "onenand_wait: timeout state=%d ctrl=0x%04x intr=0x%04x\n", state, ctrl, interrupt);
			}
		}
	} else {
		/* Turn interrupts off */
		syscfg = omap2_onenand_readw(info->onenand.base + ONENAND_REG_SYS_CFG1);
		syscfg &= ~ONENAND_SYS_CFG1_IOBE;
		omap2_onenand_writew(syscfg, info->onenand.base + ONENAND_REG_SYS_CFG1);

		timeout = jiffies + msecs_to_jiffies(20);
		while (time_before(jiffies, timeout)) {
			if (omap2_onenand_readw(info->onenand.base + ONENAND_REG_INTERRUPT) &
			    ONENAND_INT_MASTER)
				break;
		}
	}

	/* To get correct interrupt status in timeout case */
	interrupt = omap2_onenand_readw(info->onenand.base + ONENAND_REG_INTERRUPT);
	ctrl = omap2_onenand_readw(info->onenand.base + ONENAND_REG_CTRL_STATUS);

	if (ctrl & ONENAND_CTRL_ERROR) {
		printk(KERN_ERR "onenand_wait: controller error = 0x%04x\n", ctrl);
		if (ctrl & ONENAND_CTRL_LOCK)
			printk(KERN_ERR "onenand_erase: Device is write protected!!!\n");
		return ctrl;
	}

	if (ctrl & 0xFE9F)
		printk(KERN_WARNING "onenand_wait: unexpected controller status = 0x%04x  state = %d  interrupt = 0x%04x\n", ctrl, state, interrupt);

	if (interrupt & ONENAND_INT_READ) {
		int ecc = omap2_onenand_readw(info->onenand.base + ONENAND_REG_ECC_STATUS);
		if (ecc) {
			printk(KERN_ERR "onenand_wait: ECC error = 0x%04x\n", ecc);
			if (ecc & ONENAND_ECC_2BIT_ALL) {
				mtd->ecc_stats.failed++;
				return ecc;
			} else if (ecc & ONENAND_ECC_1BIT_ALL)
				mtd->ecc_stats.corrected++;
		}
	} else if (state == FL_READING) {
		printk(KERN_ERR "onenand_wait: read timeout! ctrl=0x%04x intr=0x%04x\n", ctrl, interrupt);
		return -EIO;
	}

	return 0;
}

static inline int omap2_onenand_bufferram_offset(struct mtd_info *mtd, int area)
{
	struct onenand_chip *this = mtd->priv;

	if (ONENAND_CURRENT_BUFFERRAM(this)) {
		if (area == ONENAND_DATARAM)
			return mtd->writesize;
		if (area == ONENAND_SPARERAM)
			return mtd->oobsize;
	}

	return 0;
}

static int omap2_onenand_read_bufferram(struct mtd_info *mtd, int area,
					unsigned char *buffer, int offset,
					size_t count)
{
	struct omap2_onenand *info = container_of(mtd, struct omap2_onenand, mtd);
	struct onenand_chip *this = mtd->priv;
	dma_addr_t dma_src, dma_dst;
	int bram_offset;

	bram_offset = omap2_onenand_bufferram_offset(mtd, area) + area + offset;
	if (1 || (info->dma_channel < 0) || ((void *) buffer >= (void *) high_memory) ||
	    (bram_offset & 3) || (((unsigned int) buffer) & 3) ||
	    (count < 1024) || (count & 3)) {
		memcpy(buffer, (void *)(this->base + bram_offset), count);
		return 0;
	}

	dma_src = info->phys_base + bram_offset;
	dma_dst = dma_map_single(&info->pdev->dev, buffer, count, DMA_FROM_DEVICE);
	if (dma_mapping_error(dma_dst)) {
		dev_err(&info->pdev->dev,
			"Couldn't DMA map a %d byte buffer\n",
			count);
		return -1;
	}

	omap_set_dma_transfer_params(info->dma_channel, OMAP_DMA_DATA_TYPE_S32,
				     count / 4, 1, 0, 0, 0);
	omap_set_dma_src_params(info->dma_channel, 0, OMAP_DMA_AMODE_POST_INC,
				dma_src, 0, 0);
	omap_set_dma_dest_params(info->dma_channel, 0, OMAP_DMA_AMODE_POST_INC,
				 dma_dst, 0, 0);

	INIT_COMPLETION(info->dma_done);
	omap2_block_sleep();
	omap_start_dma(info->dma_channel);
	wait_for_completion(&info->dma_done);
	omap2_allow_sleep();

	dma_unmap_single(&info->pdev->dev, dma_dst, count, DMA_FROM_DEVICE);

	return 0;
}

static int omap2_onenand_write_bufferram(struct mtd_info *mtd, int area,
					 const unsigned char *buffer, int offset,
					 size_t count)
{
	struct omap2_onenand *info = container_of(mtd, struct omap2_onenand, mtd);
	struct onenand_chip *this = mtd->priv;
	dma_addr_t dma_src, dma_dst;
	int bram_offset;

	bram_offset = omap2_onenand_bufferram_offset(mtd, area) + area + offset;
	if (1 || (info->dma_channel < 0) || ((void *) buffer >= (void *) high_memory) ||
	    (bram_offset & 3) || (((unsigned int) buffer) & 3) ||
	    (count < 1024) || (count & 3)) {
		memcpy((void *)(this->base + bram_offset), buffer, count);
		return 0;
	}

	dma_src = dma_map_single(&info->pdev->dev, (void *) buffer, count,
				 DMA_TO_DEVICE);
	dma_dst = info->phys_base + bram_offset;
	if (dma_mapping_error(dma_dst)) {
		dev_err(&info->pdev->dev,
			"Couldn't DMA map a %d byte buffer\n",
			count);
		return -1;
	}

	omap_set_dma_transfer_params(info->dma_channel, OMAP_DMA_DATA_TYPE_S16,
				     count / 2, 1, 0, 0, 0);
	omap_set_dma_src_params(info->dma_channel, 0, OMAP_DMA_AMODE_POST_INC,
				dma_src, 0, 0);
	omap_set_dma_dest_params(info->dma_channel, 0, OMAP_DMA_AMODE_POST_INC,
				 dma_dst, 0, 0);

	INIT_COMPLETION(info->dma_done);
	omap_start_dma(info->dma_channel);
	wait_for_completion(&info->dma_done);

	dma_unmap_single(&info->pdev->dev, dma_dst, count, DMA_TO_DEVICE);

	return 0;
}

static void __devexit omap2_onenand_shutdown(struct platform_device *pdev)
{
	struct omap2_onenand *info = dev_get_drvdata(&pdev->dev);

	/* With certain content in the buffer RAM, the OMAP boot ROM code
	 * can recognize the flash chip incorrectly. Zero it out before
	 * soft reset.
	 */
	memset(info->onenand.base, 0, ONENAND_BUFRAM_SIZE);
}

static int __devinit omap2_onenand_probe(struct platform_device *pdev)
{
	struct omap_onenand_platform_data *pdata;
	struct omap2_onenand *info;
	int r;

	pdata = pdev->dev.platform_data;
	if (pdata == NULL) {
		dev_err(&pdev->dev, "platform data missing\n");
		return -ENODEV;
	}

	info = kzalloc(sizeof(struct omap2_onenand), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	init_completion(&info->irq_done);
	init_completion(&info->dma_done);
	info->gpmc_cs = pdata->cs;
	info->gpio_irq = pdata->gpio_irq;

	r = gpmc_cs_request(info->gpmc_cs, ONENAND_IO_SIZE, &info->phys_base);
	if (r < 0) {
		dev_err(&pdev->dev, "Cannot request GPMC CS\n");
		goto err_kfree;
	}

	if (request_mem_region(info->phys_base, ONENAND_IO_SIZE,
			       pdev->dev.driver->name) == NULL) {
		dev_err(&pdev->dev, "Cannot reserve memory region at 0x%08lx, size: 0x%x\n",
			info->phys_base, ONENAND_IO_SIZE);
		r = -EBUSY;
		goto err_free_cs;
	}
	info->onenand.base = ioremap(info->phys_base, ONENAND_IO_SIZE);
	if (info->onenand.base == NULL) {
		r = -ENOMEM;
		goto err_release_mem_region;
	}

	if (pdata->onenand_setup != NULL) {
		r = pdata->onenand_setup(info->onenand.base);
		if (r < 0) {
			dev_err(&pdev->dev, "Onenand platform setup failed: %d\n", r);
			goto err_iounmap;
		}
        }

	if ((r = omap_request_gpio(info->gpio_irq)) < 0) {
		dev_err(&pdev->dev,  "Failed to request GPIO%d for OneNAND\n",
		        info->gpio_irq);
		goto err_iounmap;
	}
	omap_set_gpio_direction(info->gpio_irq, 1);

	if ((r = request_irq(OMAP_GPIO_IRQ(info->gpio_irq),
			     omap2_onenand_interrupt, SA_TRIGGER_RISING,
			     pdev->dev.driver->name, info)) < 0)
		goto err_release_gpio;

	r = omap_request_dma(0, pdev->dev.driver->name,
			     omap2_onenand_dma_cb, (void *) info,
			     &info->dma_channel);
	if (r == 0) {
		omap_set_dma_write_mode(info->dma_channel, OMAP_DMA_WRITE_NON_POSTED);
		omap_set_dma_src_data_pack(info->dma_channel, 1);
		omap_set_dma_src_burst_mode(info->dma_channel, OMAP_DMA_DATA_BURST_8);
		omap_set_dma_dest_data_pack(info->dma_channel, 1);
		omap_set_dma_dest_burst_mode(info->dma_channel, OMAP_DMA_DATA_BURST_8);
	} else {
		dev_info(&pdev->dev,
			 "failed to allocate DMA for OneNAND, using PIO instead\n");
		info->dma_channel = -1;
	}

	dev_info(&pdev->dev, "initializing on CS%d, phys base 0x%08lx, virtual base %p\n",
		 info->gpmc_cs, info->phys_base, info->onenand.base);

	info->pdev = pdev;
	info->mtd.name = pdev->dev.bus_id;
	info->mtd.priv = &info->onenand;
	info->mtd.owner = THIS_MODULE;
	info->onenand.wait = omap2_onenand_wait;
	info->onenand.read_bufferram = omap2_onenand_read_bufferram;
	info->onenand.write_bufferram = omap2_onenand_write_bufferram;

	if ((r = onenand_scan(&info->mtd, 1)) < 0)
		goto err_release_dma;

#ifdef CONFIG_MTD_PARTITIONS
	if (pdata->parts != NULL)
		r = add_mtd_partitions(&info->mtd, pdata->parts, pdata->nr_parts);
	else
#endif
		r = add_mtd_device(&info->mtd);
	if (r < 0)
		goto err_release_onenand;

	platform_set_drvdata(pdev, info);

	return 0;

err_release_onenand:
	onenand_release(&info->mtd);
err_release_dma:
	if (info->dma_channel != -1)
		omap_free_dma(info->dma_channel);
	free_irq(OMAP_GPIO_IRQ(info->gpio_irq), info);
err_release_gpio:
	omap_free_gpio(info->gpio_irq);
err_iounmap:
	iounmap(info->onenand.base);
err_release_mem_region:
	release_mem_region(info->phys_base, ONENAND_IO_SIZE);
err_free_cs:
	gpmc_cs_free(info->gpmc_cs);
err_kfree:
	kfree(info);

	return r;
}

static int __devexit omap2_onenand_remove(struct platform_device *pdev)
{
	struct omap2_onenand *info = dev_get_drvdata(&pdev->dev);

	BUG_ON(info == NULL);

#ifdef CONFIG_MTD_PARTITIONS
	if (info->parts)
		del_mtd_partitions(&info->mtd);
	else
		del_mtd_device(&info->mtd);
#else
	del_mtd_device(&info->mtd);
#endif

	onenand_release(&info->mtd);
	if (info->dma_channel != -1)
		omap_free_dma(info->dma_channel);
	omap2_onenand_shutdown(pdev);
	platform_set_drvdata(pdev, NULL);
	free_irq(OMAP_GPIO_IRQ(info->gpio_irq), info);
	omap_free_gpio(info->gpio_irq);
	iounmap(info->onenand.base);
	release_mem_region(info->phys_base, ONENAND_IO_SIZE);
	kfree(info);

	return 0;
}

static struct platform_driver omap2_onenand_driver = {
	.probe		= omap2_onenand_probe,
	.remove		= omap2_onenand_remove,
	.shutdown	= omap2_onenand_shutdown,
	.driver		= {
		.name	= "omap2-onenand",
		.owner  = THIS_MODULE,
	},
};

MODULE_ALIAS(DRIVER_NAME);

static int __init omap2_onenand_init(void)
{
	printk(KERN_INFO "OMAP2 OneNAND driver initializing\n");
	return platform_driver_register(&omap2_onenand_driver);
}

static void __exit omap2_onenand_exit(void)
{
	platform_driver_unregister(&omap2_onenand_driver);
}

module_init(omap2_onenand_init);
module_exit(omap2_onenand_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jarkko Lavinen <jarkko.lavinen@nokia.com>");
MODULE_DESCRIPTION("Glue layer for OneNAND flash on OMAP2");
