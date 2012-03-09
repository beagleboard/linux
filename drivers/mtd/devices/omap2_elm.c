/*
 * OMAP2 Error Location Module
 *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
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

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/io.h>
#include <linux/pm_runtime.h>

#include <plat/elm.h>

#define ELM_SYSCONFIG			0x010
#define ELM_SYSSTATUS			0x014
#define ELM_IRQSTATUS			0x018
#define ELM_IRQENABLE			0x01c
#define ELM_LOCATION_CONFIG		0x020
#define ELM_PAGE_CTRL			0x080
#define ELM_SYNDROME_FRAGMENT_0		0x400
#define ELM_SYNDROME_FRAGMENT_1		0x404
#define ELM_SYNDROME_FRAGMENT_2		0x408
#define ELM_SYNDROME_FRAGMENT_3		0x40c
#define ELM_SYNDROME_FRAGMENT_4		0x410
#define ELM_SYNDROME_FRAGMENT_5		0x414
#define ELM_SYNDROME_FRAGMENT_6		0x418
#define ELM_LOCATION_STATUS		0x800
#define ELM_ERROR_LOCATION_0		0x880
#define ELM_ERROR_LOCATION_1		0x884
#define ELM_ERROR_LOCATION_2		0x888
#define ELM_ERROR_LOCATION_3		0x88c
#define ELM_ERROR_LOCATION_4		0x890
#define ELM_ERROR_LOCATION_5		0x894
#define ELM_ERROR_LOCATION_6		0x898
#define ELM_ERROR_LOCATION_7		0x89c
#define ELM_ERROR_LOCATION_8		0x8a0
#define ELM_ERROR_LOCATION_9		0x8a4
#define ELM_ERROR_LOCATION_10		0x8a8
#define ELM_ERROR_LOCATION_11		0x8ac
#define ELM_ERROR_LOCATION_12		0x8b0
#define ELM_ERROR_LOCATION_13		0x8b4
#define ELM_ERROR_LOCATION_14		0x8b8
#define ELM_ERROR_LOCATION_15		0x8bc

/* ELM System Configuration Register */
#define ELM_SYSCONFIG_SOFTRESET		BIT(1)
#define ELM_SYSCONFIG_SIDLE_MASK	(3 << 3)
#define ELM_SYSCONFIG_SMART_IDLE	(2 << 3)

/* ELM System Status Register */
#define ELM_SYSSTATUS_RESETDONE		BIT(0)

/* ELM Interrupt Status Register */
#define INTR_STATUS_PAGE_VALID		BIT(8)
#define INTR_STATUS_LOC_VALID_7		BIT(7)
#define INTR_STATUS_LOC_VALID_6		BIT(6)
#define INTR_STATUS_LOC_VALID_5		BIT(5)
#define INTR_STATUS_LOC_VALID_4		BIT(4)
#define INTR_STATUS_LOC_VALID_3		BIT(3)
#define INTR_STATUS_LOC_VALID_2		BIT(2)
#define INTR_STATUS_LOC_VALID_1		BIT(1)
#define INTR_STATUS_LOC_VALID_0		BIT(0)

/* ELM Interrupt Enable Register */
#define INTR_EN_PAGE_MASK		BIT(8)
#define INTR_EN_LOCATION_MASK_7		BIT(7)
#define INTR_EN_LOCATION_MASK_6		BIT(6)
#define INTR_EN_LOCATION_MASK_5		BIT(5)
#define INTR_EN_LOCATION_MASK_4		BIT(4)
#define INTR_EN_LOCATION_MASK_3		BIT(3)
#define INTR_EN_LOCATION_MASK_2		BIT(2)
#define INTR_EN_LOCATION_MASK_1		BIT(1)
#define INTR_EN_LOCATION_MASK_0		BIT(0)

/* ELM Location Configuration Register */
#define ECC_SIZE_MASK			(0x7ff << 16)
#define ECC_BCH_LEVEL_MASK		(0x3 << 0)
#define ECC_BCH4_LEVEL			(0x0 << 0)
#define ECC_BCH8_LEVEL			(0x1 << 0)
#define ECC_BCH16_LEVEL			(0x2 << 0)

/* ELM Page Definition Register */
#define PAGE_MODE_SECTOR_7		BIT(7)
#define PAGE_MODE_SECTOR_6		BIT(6)
#define PAGE_MODE_SECTOR_5		BIT(5)
#define PAGE_MODE_SECTOR_4		BIT(4)
#define PAGE_MODE_SECTOR_3		BIT(3)
#define PAGE_MODE_SECTOR_2		BIT(2)
#define PAGE_MODE_SECTOR_1		BIT(1)
#define PAGE_MODE_SECTOR_0		BIT(0)

/* ELM syndrome */
#define ELM_SYNDROME_VALID		BIT(16)

/* ELM_LOCATION_STATUS Register */
#define ECC_CORRECTABLE_MASK		BIT(8)
#define ECC_NB_ERRORS_MASK		(0x1f << 0)

/*  ELM_ERROR_LOCATION_0-15 Registers */
#define ECC_ERROR_LOCATION_MASK		(0x1fff << 0)

#define OMAP_ECC_SIZE			(0x7ff)

#define DRIVER_NAME	"omap2_elm"

static void  __iomem *elm_base;
static struct completion elm_completion;
static struct mtd_info *mtd;
static int bch_scheme;

static void elm_write_reg(int idx, u32 val)
{
	writel(val, elm_base + idx);
}

static u32 elm_read_reg(int idx)
{
	return readl(elm_base + idx);
}

/**
 * omap_elm_config - Configure ELM for BCH ECC scheme
 * @bch_type:	type of BCH ECC scheme
 */
void omap_elm_config(int bch_type)
{
	u32 reg_val;
	u32 buffer_size = OMAP_ECC_SIZE;

	reg_val = (bch_type & ECC_BCH_LEVEL_MASK) | (buffer_size << 16);
	elm_write_reg(ELM_LOCATION_CONFIG, reg_val);

	/* clearing interrupts */
	reg_val = elm_read_reg(ELM_IRQSTATUS);
	elm_write_reg(ELM_IRQSTATUS, reg_val & INTR_STATUS_LOC_VALID_0);
	elm_write_reg(ELM_IRQSTATUS, INTR_STATUS_LOC_VALID_0);

	/* enable in interrupt mode */
	reg_val = elm_read_reg(ELM_IRQENABLE);
	reg_val |= INTR_EN_LOCATION_MASK_0;
	elm_write_reg(ELM_IRQENABLE, reg_val);

	/* config in Continuous mode */
	reg_val = elm_read_reg(ELM_PAGE_CTRL);
	reg_val &= ~PAGE_MODE_SECTOR_0;
	elm_write_reg(ELM_PAGE_CTRL, reg_val);
}

/**
 * omap_configure_elm - Configure ELM for BCH ECC scheme
 * @mtd_info:	mtd info structure
 * @bch_type:	type of BCH ECC scheme
 *
 * Configures the ELM module to support BCH error correction
 */
void omap_configure_elm(struct mtd_info *mtd_info, int bch_type)
{
	omap_elm_config(bch_type);
	mtd = mtd_info;
	bch_scheme = bch_type;
}
EXPORT_SYMBOL(omap_configure_elm);

/**
 * omap_elm_load_syndrome - Load ELM syndrome reg
 * @bch_type:	type of BCH ECC scheme
 * @syndrome:	Syndrome polynomial
 *
 * Load the syndrome polynomial to syndrome registers
 */
void omap_elm_load_syndrome(int bch_type, char *syndrome)
{
	int reg_val;
	int i;

	for (i = 0; i < 4; i++) {
		reg_val = syndrome[0] | syndrome[1] << 8 |
			syndrome[2] << 16 | syndrome[3] << 24;
		elm_write_reg(ELM_SYNDROME_FRAGMENT_0 + i * 4, reg_val);
		syndrome += 4;
	}
}

/**
 * omap_elm_start_processing - Start calculting error location
 */
void omap_elm_start_processing(void)
{
	u32 reg_val;

	reg_val = elm_read_reg(ELM_SYNDROME_FRAGMENT_6);
	reg_val |= ELM_SYNDROME_VALID;
	elm_write_reg(ELM_SYNDROME_FRAGMENT_6, reg_val);
}

void rotate_ecc_bytes(u8 *src, u8 *dst)
{
	int i;

	for (i = 0; i < BCH8_ECC_OOB_BYTES; i++)
		dst[BCH8_ECC_OOB_BYTES - 1 - i] = src[i];
}

/**
 * omap_elm_decode_bch_error - Locate error pos
 * @bch_type:	Type of BCH ECC scheme
 * @ecc_calc:	Calculated ECC bytes from GPMC
 * @err_loc:	Error location bytes
 */
int omap_elm_decode_bch_error(int bch_type, char *ecc_calc,
		unsigned int *err_loc)
{
	u8 ecc_data[BCH_MAX_ECC_BYTES_PER_SECTOR] = {0};
	u32 reg_val;
	int i, err_no;

	rotate_ecc_bytes(ecc_calc, ecc_data);
	omap_elm_load_syndrome(bch_type, ecc_data);
	omap_elm_start_processing();
	wait_for_completion(&elm_completion);
	reg_val = elm_read_reg(ELM_LOCATION_STATUS);

	if (reg_val & ECC_CORRECTABLE_MASK) {
		err_no = reg_val & ECC_NB_ERRORS_MASK;

		for (i = 0; i < err_no; i++) {
			reg_val = elm_read_reg(ELM_ERROR_LOCATION_0 + i * 4);
			err_loc[i] = reg_val;
		}

		return err_no;
	}

	return -EINVAL;
}
EXPORT_SYMBOL(omap_elm_decode_bch_error);

static irqreturn_t omap_elm_isr(int this_irq, void *dev_id)
{
	u32 reg_val;

	reg_val = elm_read_reg(ELM_IRQSTATUS);

	if (reg_val & INTR_STATUS_LOC_VALID_0) {
		elm_write_reg(ELM_IRQSTATUS, reg_val & INTR_STATUS_LOC_VALID_0);
		complete(&elm_completion);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static int omap_elm_probe(struct platform_device *pdev)
{
	int  ret_status = 0;
	struct resource *res = NULL, *irq = NULL;

	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);

	if (irq == NULL)
		return -EINVAL;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (res == NULL)
		return -EINVAL;

	if (!request_mem_region(res->start, resource_size(res),
				dev_name(&pdev->dev)))
		return -EBUSY;

	elm_base = ioremap(res->start, resource_size(res));

	if (!elm_base) {
		dev_dbg(&pdev->dev, "can't ioremap\n");
		ret_status = -ENOMEM;
		goto err_remap;
	}

	pm_runtime_enable(&pdev->dev);
	if (pm_runtime_get_sync(&pdev->dev)) {
		ret_status = -EINVAL;
		dev_dbg(&pdev->dev, "can't enable clock\n");
		goto err_clk;
	}

	ret_status = request_irq(irq->start, omap_elm_isr, 0, pdev->name,
			&pdev->dev);

	if (ret_status) {
		pr_err("failure requesting irq %i\n", irq->start);
		goto err_irq;
	}

	init_completion(&elm_completion);
	return ret_status;

err_irq:
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
err_clk:
	iounmap(elm_base);
err_remap:
	release_mem_region(res->start, resource_size(res));
	return ret_status;
}

static int omap_elm_remove(struct platform_device *pdev)
{
	struct resource *res = NULL;

	iounmap(elm_base);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));
	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	free_irq(res->start, &pdev->dev);
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	return 0;
}


#ifdef CONFIG_PM
static int omap_elm_suspend(struct platform_device *pdev, pm_message_t state)
{
	if (mtd && mtd->suspend)
		mtd->suspend(mtd);
	pm_runtime_put_sync(&pdev->dev);
	return 0;
}

static int omap_elm_resume(struct platform_device *pdev)
{
	pm_runtime_get_sync(&pdev->dev);
	/* Restore ELM context by configuring */
	omap_elm_config(bch_scheme);
	return 0;
}
#endif

static struct platform_driver omap_elm_driver = {
	.probe		= omap_elm_probe,
	.remove		= omap_elm_remove,
#ifdef CONFIG_PM
	.suspend	= omap_elm_suspend,
	.resume		= omap_elm_resume,
#endif
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init omap_elm_init(void)
{

	return platform_driver_register(&omap_elm_driver);
}

static void __exit omap_elm_exit(void)
{
	platform_driver_unregister(&omap_elm_driver);
}

module_init(omap_elm_init);
module_exit(omap_elm_exit);

MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_LICENSE("GPL");
