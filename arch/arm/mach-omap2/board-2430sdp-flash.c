/*
 * linux/arch/arm/mach-omap2/board-2430sdp-flash.c
 *
 * Copyright (C) 2007 MontaVista Software, Inc. <source@mvista.com>
 * Author: Kevin Hilman
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <asm/mach/flash.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/onenand_regs.h>

#include <asm/io.h>
#include <mach/onenand.h>
#include <mach/board.h>
#include <mach/gpmc.h>
#include <mach/nand.h>

#define ONENAND_MAP 0x20000000
#define GPMC_OFF_CONFIG1_0 0x60

enum fstype {
	NAND = 0,
	NOR,
	ONENAND,
	UNKNOWN = -1
};

static enum fstype flash_type = NAND;

static struct mtd_partition nand_partitions[] = {
	{
		.name		= "X-Loader",
		.offset		= 0,
		.size		= 4*(64*2048),  /* 0-3 blks reserved.
						   Mandated by ROM code */
		.mask_flags	= MTD_WRITEABLE	/* force read-only */
	},
	{
		.name		= "U-Boot",
		.offset		= MTDPART_OFS_APPEND,
		.size		=  4*(64*2048),
		.mask_flags	= MTD_WRITEABLE	/* force read-only */
	},
	{
		.name		= "U-Boot Environment",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 2*(64*2048),
	},
	{
		.name		= "Kernel",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 32*(64*2048),		/* 4*1M */
	},
	{
		.name		= "File System",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
	},
};
static struct omap_nand_platform_data sdp_nand_data = {
	.parts		= nand_partitions,
	.nr_parts	= ARRAY_SIZE(nand_partitions),
	.dma_channel	= -1,	/* disable DMA in OMAP OneNAND driver */
};

static struct platform_device sdp_nand_device = {
	.name		= "omap2-nand",
	.id		= -1,
	.dev = {
		.platform_data = &sdp_nand_data,
	},
};

static struct mtd_partition onenand_partitions[] = {
	{
		.name		= "(OneNAND)X-Loader",
		.offset		= 0,
		.size		= 4*(64*2048),  /* 0-3 blks reserved.
						   Mandated by ROM code */
		.mask_flags	= MTD_WRITEABLE	/* force read-only */
	},
	{
		.name		= "(OneNAND)U-Boot",
		.offset		= MTDPART_OFS_APPEND,
		.size		=  2*(64*2048),
		.mask_flags	= MTD_WRITEABLE	/* force read-only */
	},
	{
		.name		= "(OneNAND)U-Boot Environment",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 1*(64*2048),
	},
	{
		.name		= "(OneNAND)Kernel",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 4*(64*2048),
	},
	{
		.name		= "(OneNAND)File System",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
	},
};

static struct omap_onenand_platform_data sdp_onenand_data = {
	.parts		= onenand_partitions,
	.nr_parts	= ARRAY_SIZE(onenand_partitions),
	.dma_channel	= -1,	/* disable DMA in OMAP OneNAND driver */
};

static struct platform_device sdp_onenand_device = {
	.name		= "omap2-onenand",
	.id		= -1,
	.dev = {
		.platform_data = &sdp_onenand_data,
	},
};

void __init sdp2430_flash_init(void)
{
	void __iomem *gpmc_base_add, *gpmc_cs_base_add;
	unsigned char cs = 0;

	gpmc_base_add = (__force void __iomem *)OMAP243X_GPMC_VIRT;
	while (cs < GPMC_CS_NUM) {
		int ret = 0;

		/* Each GPMC set for a single CS is at offset 0x30 */
		gpmc_cs_base_add = (gpmc_base_add + GPMC_OFF_CONFIG1_0 +
				    (cs*0x30));

		/* xloader/Uboot would have programmed the NAND/oneNAND
		 * base address for us This is a ugly hack. The proper
		 * way of doing this is to pass the setup of u-boot up
		 * to kernel using kernel params - something on the
		 * lines of machineID. Check if Nand/oneNAND is
		 * configured */
		ret = __raw_readl(gpmc_cs_base_add + GPMC_CS_CONFIG1);
		if ((ret & 0xC00) == (0x800)) {
			/* Found it!! */
			printk(KERN_INFO "NAND: Found NAND on CS %d \n", cs);
			flash_type = NAND;
			break;
		}
		ret = __raw_readl(gpmc_cs_base_add + GPMC_CS_CONFIG7);
		if ((ret & 0x3F) == (ONENAND_MAP >> 24)) {
			/* Found it!! */
			flash_type = ONENAND;
			break;
		}
		cs++;
	}
	if (cs >= GPMC_CS_NUM) {
		printk(KERN_INFO "MTD: Unable to find MTD configuration in "
				 "GPMC   - not registering.\n");
		return;
	}

	if (flash_type == NAND) {
		sdp_nand_data.cs	       = cs;
		sdp_nand_data.gpmc_cs_baseaddr = gpmc_cs_base_add;
		sdp_nand_data.gpmc_baseaddr    = gpmc_base_add;

		if (platform_device_register(&sdp_nand_device) < 0) {
			printk(KERN_ERR "Unable to register NAND device\n");
			return;
		}
	}

	if (flash_type == ONENAND) {
		sdp_onenand_data.cs = cs;

		if (platform_device_register(&sdp_onenand_device) < 0) {
			printk(KERN_ERR "Unable to register OneNAND device\n");
			return;
		}
	}
}
