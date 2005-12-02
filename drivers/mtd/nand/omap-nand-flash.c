/*
 * drivers/mtd/nand/omap-nand-flash.c
 *
 * Copyright (c) 2004 Texas Instruments, Jian Zhang <jzhang@ti.com>
 * Copyright (c) 2004 David Brownell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#include <asm/io.h>
#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/flash.h>
#include <asm/arch/tc.h>

#include <asm/io.h>
#include <asm/arch/hardware.h>

#define	DRIVER_NAME	"omapnand"

#ifdef CONFIG_MTD_PARTITIONS
static const char *part_probes[] = { "cmdlinepart", NULL };
#endif

struct omap_nand_info {
	struct nand_platform_data *pdata;
	struct mtd_partition	*parts;
	struct mtd_info		mtd;
	struct nand_chip	nand;
};

/*
 *	hardware specific access to control-lines
 *	NOTE:  boards may use different bits for these!!
 */
#define	MASK_CLE	0x02
#define	MASK_ALE	0x04
static void omap_nand_hwcontrol(struct mtd_info *mtd, int cmd)
{
	struct nand_chip *this = mtd->priv;
	unsigned long IO_ADDR_W = (unsigned long) this->IO_ADDR_W;

	switch (cmd) {
		case NAND_CTL_SETCLE: IO_ADDR_W |= MASK_CLE; break;
		case NAND_CTL_CLRCLE: IO_ADDR_W &= ~MASK_CLE; break;
		case NAND_CTL_SETALE: IO_ADDR_W |= MASK_ALE; break;
		case NAND_CTL_CLRALE: IO_ADDR_W &= ~MASK_ALE; break;
	}
	this->IO_ADDR_W = (void __iomem *) IO_ADDR_W;
}

static int omap_nand_dev_ready(struct mtd_info *mtd)
{
	struct omap_nand_info *info = container_of(mtd, struct omap_nand_info, mtd);

	return info->pdata->dev_ready(info->pdata);
}

static int __devinit omap_nand_probe(struct platform_device *pdev)
{
	struct omap_nand_info		*info;
	struct nand_platform_data	*pdata = pdev->dev.platform_data;
	struct resource			*res = pdev->resource;
	unsigned long			size = res->end - res->start + 1;
	int				err;

	info = kmalloc(sizeof(struct omap_nand_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	memset(info, 0, sizeof(struct omap_nand_info));

	if (!request_mem_region(res->start, size, pdev->dev.driver->name)) {
		err = -EBUSY;
		goto out_free_info;
	}

	info->nand.IO_ADDR_R = ioremap(res->start, size);
	if (!info->nand.IO_ADDR_R) {
		err = -ENOMEM;
		goto out_release_mem_region;
	}
	info->nand.IO_ADDR_W = info->nand.IO_ADDR_R;
	info->nand.hwcontrol = omap_nand_hwcontrol;
	info->nand.eccmode = NAND_ECC_SOFT;
	info->nand.options = pdata->options;
	if (pdata->dev_ready)
		info->nand.dev_ready = omap_nand_dev_ready;
	else
		info->nand.chip_delay = 20;

	info->mtd.name = pdev->dev.bus_id;
	info->mtd.priv = &info->nand;

	info->pdata = pdata;

	/* DIP switches on H2 and some other boards change between 8 and 16 bit
	 * bus widths for flash.  Try the other width if the first try fails.
	 */
	if (nand_scan(&info->mtd, 1)) {
		info->nand.options ^= NAND_BUSWIDTH_16;
		if (nand_scan(&info->mtd, 1)) {
			err = -ENXIO;
			goto out_iounmap;
		}
	}
	info->mtd.owner = THIS_MODULE;

#ifdef CONFIG_MTD_PARTITIONS
	err = parse_mtd_partitions(&info->mtd, part_probes, &info->parts, 0);
	if (err > 0)
		add_mtd_partitions(&info->mtd, info->parts, err);
	else if (err < 0 && pdata->parts)
		add_mtd_partitions(&info->mtd, pdata->parts, pdata->nr_parts);
	else
#endif
		add_mtd_device(&info->mtd);

	platform_set_drvdata(pdev, info);

	return 0;

out_iounmap:
	iounmap(info->nand.IO_ADDR_R);
out_release_mem_region:
	release_mem_region(res->start, size);
out_free_info:
	kfree(info);

	return err;
}

static int omap_nand_remove(struct platform_device *pdev)
{
	struct omap_nand_info *info = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	/* Release NAND device, its internal structures and partitions */
	nand_release(&info->mtd);
	iounmap(info->nand.IO_ADDR_R);
	kfree(info);
	return 0;
}

static struct platform_driver omap_nand_driver = {
	.probe		= omap_nand_probe,
	.remove		= omap_nand_remove,
	.driver		= {
		.name	= DRIVER_NAME,
	},
};
MODULE_ALIAS(DRIVER_NAME);

static int __init omap_nand_init(void)
{
	return platform_driver_register(&omap_nand_driver);
}

static void __exit omap_nand_exit(void)
{
	platform_driver_unregister(&omap_nand_driver);
}

module_init(omap_nand_init);
module_exit(omap_nand_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jian Zhang <jzhang@ti.com> (and others)");
MODULE_DESCRIPTION("Glue layer for NAND flash on TI OMAP boards");

