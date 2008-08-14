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
/**
 * sdp3430_flash_init - Identify devices connected to GPMC and register.
 *
 * @return - void.
 */
void __init sdp3430_flash_init(void)
{
	u8		cs = 0;
	u8		onenandcs = GPMC_CS_NUM + 1;

	/* Configure start address and size of NOR device */
	if (is_sil_rev_greater_than(OMAP3430_REV_ES1_0)) {
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
		ret = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG7);

		/*
		* xloader/Uboot would have programmed the oneNAND
		* base address for us This is a ugly hack. The proper
		* way of doing this is to pass the setup of u-boot up
		* to kernel using kernel params - something on the
		* lines of machineID. Check if oneNAND is configured
		*/
		if ((ret & 0x3F) == (ONENAND_MAP >> 24))
			onenandcs = cs;
		cs++;
	}
	if (onenandcs > GPMC_CS_NUM) {
		printk(KERN_INFO "OneNAND: Unable to find configuration "
				" in GPMC\n ");
		return;
	}

	if (onenandcs < GPMC_CS_NUM) {
		sdp_onenand_data.cs = onenandcs;
		if (platform_device_register(&sdp_onenand_device) < 0)
			printk(KERN_ERR "Unable to register OneNAND device\n");
	}
}
