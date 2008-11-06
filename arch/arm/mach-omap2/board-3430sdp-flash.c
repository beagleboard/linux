/*
 * linux/arch/arm/mach-omap2/board-3430sdp-flash.c
 *
 * Copyright (c) 2007 Texas Instruments
 *
 * Modified from mach-omap2/board-2430sdp-flash.c
 * Author: Rohit Choraria <rohitkc@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/onenand_regs.h>
#include <linux/types.h>
#include <linux/io.h>

#include <asm/mach/flash.h>
#include <mach/onenand.h>
#include <mach/board.h>
#include <mach/gpmc.h>
#include <mach/nand.h>

#define NAND_BLOCK_SIZE		SZ_128K

static struct mtd_partition sdp_nor_partitions[] = {
	/* bootloader (U-Boot, etc) in first sector */
	{
		.name		= "Bootloader-NOR",
		.offset		= 0,
		.size		= SZ_256K,
		.mask_flags	= MTD_WRITEABLE, /* force read-only */
	},
	/* bootloader params in the next sector */
	{
		.name		= "Params-NOR",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_256K,
		.mask_flags	= 0,
	},
	/* kernel */
	{
		.name		= "Kernel-NOR",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_2M,
		.mask_flags	= 0
	},
	/* file system */
	{
		.name		= "Filesystem-NOR",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
		.mask_flags	= 0
	}
};

static struct flash_platform_data sdp_nor_data = {
	.map_name	= "cfi_probe",
	.width		= 2,
	.parts		= sdp_nor_partitions,
	.nr_parts	= ARRAY_SIZE(sdp_nor_partitions),
};

static struct resource sdp_nor_resource = {
	.start		= 0,
	.end		= 0,
	.flags		= IORESOURCE_MEM,
};

static struct platform_device sdp_nor_device = {
	.name		= "omapflash",
	.id		= 0,
	.dev		= {
			.platform_data = &sdp_nor_data,
	},
	.num_resources	= 1,
	.resource	= &sdp_nor_resource,
};

static int sdp_onenand_setup(void __iomem *, int freq);

static struct mtd_partition sdp_onenand_partitions[] = {
	{
		.name		= "X-Loader-OneNAND",
		.offset		= 0,
		.size		= 4 * (64 * 2048),
		.mask_flags	= MTD_WRITEABLE	 /* force read-only */
	},
	{
		.name		= "U-Boot-OneNAND",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 2 * (64 * 2048),
		.mask_flags	= MTD_WRITEABLE	 /* force read-only */
	},
	{
		.name		= "U-Boot Environment-OneNAND",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 1 * (64 * 2048),
	},
	{
		.name		= "Kernel-OneNAND",
		.offset		= MTDPART_OFS_APPEND,
		.size		= 16 * (64 * 2048),
	},
	{
		.name		= "File System-OneNAND",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
	},
};

static struct omap_onenand_platform_data sdp_onenand_data = {
	.parts		= sdp_onenand_partitions,
	.nr_parts	= ARRAY_SIZE(sdp_onenand_partitions),
	.onenand_setup	= sdp_onenand_setup,
	.dma_channel	= -1,	/* disable DMA in OMAP OneNAND driver */
};

static struct platform_device sdp_onenand_device = {
	.name		= "omap2-onenand",
	.id		= -1,
	.dev = {
		.platform_data = &sdp_onenand_data,
	},
};

/*
 * sdp_onenand_setup - The function configures the onenand flash.
 * @onenand_base: Onenand base address
 *
 * @return int:	Currently always returning zero.
 */
static int sdp_onenand_setup(void __iomem *onenand_base, int freq)
{
	/* Onenand setup does nothing at present */
	return 0;
}

static struct mtd_partition sdp_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name           = "X-Loader-NAND",
		.offset         = 0,
		.size           = 4 * NAND_BLOCK_SIZE,
		.mask_flags     = MTD_WRITEABLE,        /* force read-only */
	},
	{
		.name           = "U-Boot-NAND",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x80000 */
		.size           = 4 * NAND_BLOCK_SIZE,
		.mask_flags     = MTD_WRITEABLE,        /* force read-only */
	},
	{
		.name           = "Boot Env-NAND",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x100000 */
		.size           = 2 * NAND_BLOCK_SIZE,
	},
	{
		.name           = "Kernel-NAND",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x140000 */
		.size           = 32 * NAND_BLOCK_SIZE,
	},
	{
		.name           = "File System - NAND",
		.size           = MTDPART_SIZ_FULL,
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x540000 */
	},
};

static struct omap_nand_platform_data sdp_nand_data = {
	.parts          = sdp_nand_partitions,
	.nr_parts       = ARRAY_SIZE(sdp_nand_partitions),
	.nand_setup     = NULL,
	.dma_channel    = -1,           /* disable DMA in OMAP NAND driver */
	.dev_ready      = NULL,
};

static struct resource sdp_nand_resource = {
	.flags          = IORESOURCE_MEM,
};

static struct platform_device sdp_nand_device = {
	.name           = "omap2-nand",
	.id             = 0,
	.dev            = {
	.platform_data  = &sdp_nand_data,
	},
	.num_resources  = 1,
	.resource       = &sdp_nand_resource,
};


/**
 * sdp3430_flash_init - Identify devices connected to GPMC and register.
 *
 * @return - void.
 */
void __init sdp3430_flash_init(void)
{
	u8		cs = 0;
	u8              nandcs = GPMC_CS_NUM + 1;
	u8		onenandcs = GPMC_CS_NUM + 1;
	unsigned long	gpmc_base_add;

	gpmc_base_add   = OMAP34XX_GPMC_VIRT;

	/* Configure start address and size of NOR device */
	if (omap_rev() > OMAP3430_REV_ES1_0) {
		sdp_nor_resource.start	= FLASH_BASE_SDPV2;
		sdp_nor_resource.end	= FLASH_BASE_SDPV2
						+ FLASH_SIZE_SDPV2 - 1;
	} else {
		sdp_nor_resource.start	= FLASH_BASE_SDPV1;
		sdp_nor_resource.end	= FLASH_BASE_SDPV1
						+ FLASH_SIZE_SDPV1 - 1;
	}

	if (platform_device_register(&sdp_nor_device) < 0)
		printk(KERN_ERR "Unable to register NOR device\n");

	while (cs < GPMC_CS_NUM) {
		u32 ret = 0;
		ret = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG1);

		/*
		* xloader/Uboot would have programmed the NAND/oneNAND
		* base address for us This is a ugly hack. The proper
		* way of doing this is to pass the setup of u-boot up
		* to kernel using kernel params - something on the
		* lines of machineID. Check if oneNAND is configured
		*/
		if ((ret & 0xC00) == 0x800) {
			/* Found it!! */
			if (nandcs > GPMC_CS_NUM)
				nandcs = cs;
		} else {
			ret = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG7);
			if ((ret & 0x3F) == (ONENAND_MAP >> 24))
			onenandcs = cs;
		}
		cs++;
	}
	if ((nandcs > GPMC_CS_NUM) && (onenandcs > GPMC_CS_NUM)) {
		printk(KERN_INFO "NAND/OneNAND: Unable to find configuration "
				" in GPMC\n ");
		return;
	}

	if (nandcs < GPMC_CS_NUM) {
		sdp_nand_data.cs        = nandcs;
		sdp_nand_data.gpmc_cs_baseaddr   = (void *)(gpmc_base_add +
					GPMC_CS0_BASE + nandcs*GPMC_CS_SIZE);
		sdp_nand_data.gpmc_baseaddr     = (void *) (gpmc_base_add);

		if (platform_device_register(&sdp_nand_device) < 0)
			printk(KERN_ERR "Unable to register NAND device\n");
	}

	if (onenandcs < GPMC_CS_NUM) {
		sdp_onenand_data.cs = onenandcs;
		if (platform_device_register(&sdp_onenand_device) < 0)
			printk(KERN_ERR "Unable to register OneNAND device\n");
	}
}
