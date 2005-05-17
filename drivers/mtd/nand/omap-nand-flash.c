/*
 *  drivers/mtd/nand/omap-nand-flash.c
 *
 *  Copyright (c) 2004 Texas Instruments
 *  Jian Zhang <jzhang@ti.com>
 *  Copyright (c) 2004 David Brownell
 *
 *  Derived from drivers/mtd/autcpu12.c
 *
 *  Copyright (c) 2002 Thomas Gleixner <tgxl@linutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Overview:
 *   This is a device driver for the NAND flash device found on the
 *   TI H3/H2  boards. It supports 16-bit 32MiB Samsung k9f5616 chip.
 *
 */

#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/gpio.h>
#include <asm/arch/mux.h>
#include <asm/arch/tc.h>
#include <asm/sizes.h>
#include <asm/mach-types.h>

#define H3_NAND_RB_GPIO_PIN		10
#define H2_NAND_RB_GPIO_PIN		62
#define NETSTAR_NAND_RB_GPIO_PIN	 1
/*
 * MTD structure for H3 board
 */
static struct mtd_info *omap_nand_mtd = NULL;

static void __iomem *omap_nand_flash_base;

/*
 * Define partitions for flash devices
 */

#ifdef CONFIG_MTD_PARTITIONS
static struct mtd_partition static_partition[] = {
	{ .name = "Booting Image",
	  .offset =	0,
	  .size = 64 * 1024,
	  .mask_flags =	MTD_WRITEABLE  /* force read-only */
 	},
	{ .name = "U-Boot",
	  .offset =	MTDPART_OFS_APPEND,
	  .size = 256 * 1024,
          .mask_flags = MTD_WRITEABLE  /* force read-only */
 	},
	{ .name = "U-Boot Environment",
	  .offset =	MTDPART_OFS_APPEND,
	  .size = 192 * 1024
	},
	{ .name = "Kernel",
	  .offset =	MTDPART_OFS_APPEND,
	  .size = 2 * SZ_1M
	},
	{ .name = "File System",
	  .size = MTDPART_SIZ_FULL,
	  .offset =	MTDPART_OFS_APPEND,
	},
};

const char *part_probes[] = { "cmdlinepart", NULL,  };

#endif

/* H2/H3 maps two address LSBs to CLE and ALE; MSBs make CS_2B */
#define	MASK_CLE	0x02
#define	MASK_ALE	0x04


/* 
 *	hardware specific access to control-lines
*/
static void omap_nand_hwcontrol(struct mtd_info *mtd, int cmd)
{
	struct nand_chip *this = mtd->priv;
	u32 IO_ADDR_W = (u32) this->IO_ADDR_W;

	IO_ADDR_W &= ~(MASK_ALE|MASK_CLE);
	switch(cmd){
		case NAND_CTL_SETCLE: IO_ADDR_W |= MASK_CLE; break;
		case NAND_CTL_SETALE: IO_ADDR_W |= MASK_ALE; break;
	}
	this->IO_ADDR_W = (void __iomem *) IO_ADDR_W;
}

/*
 *	chip busy R/B detection
 */
static int omap_nand_ready(struct mtd_info *mtd)
{
	if (machine_is_omap_h3())
		return omap_get_gpio_datain(H3_NAND_RB_GPIO_PIN);
	if (machine_is_omap_h2())
		return omap_get_gpio_datain(H2_NAND_RB_GPIO_PIN);
	if (machine_is_netstar())
		return omap_get_gpio_datain(NETSTAR_NAND_RB_GPIO_PIN);
	return 0;
}

/* Scan to find existance of the device at omap_nand_flash_base.
   This also allocates oob and data internal buffers */
static int __init probe_nand_chip(void)
{
        struct nand_chip *this;

        this = (struct nand_chip *) (&omap_nand_mtd[1]);
       
	/* Initialize structures */
        memset((char *) this, 0, sizeof(struct nand_chip));
        
	this->IO_ADDR_R = omap_nand_flash_base;
        this->IO_ADDR_W = omap_nand_flash_base;
        this->options = NAND_SAMSUNG_LP_OPTIONS;
        this->hwcontrol = omap_nand_hwcontrol;
        this->eccmode = NAND_ECC_SOFT;

        /* try 16-bit chip first */
	this->options |= NAND_BUSWIDTH_16;
        if (nand_scan (omap_nand_mtd, 1)) {
		if (machine_is_omap_h3()) 
			return -ENXIO;

		/* then try 8-bit chip for H2 */
        	memset((char *) this, 0, sizeof(struct nand_chip));
        	this->IO_ADDR_R = omap_nand_flash_base;
        	this->IO_ADDR_W = omap_nand_flash_base;
		this->options = NAND_SAMSUNG_LP_OPTIONS;
		this->hwcontrol = omap_nand_hwcontrol;
        	this->eccmode = NAND_ECC_SOFT;
                if (nand_scan (omap_nand_mtd, 1)) {
                        return -ENXIO;
                }
        }

	return 0;
}

static char nand1_name [] = "nand";

/*
 * Main initialization routine
 */
int __init omap_nand_init (void)
{
	struct nand_chip *this;
	struct mtd_partition *dynamic_partition = 0;
	int err = 0;
	int nandboot = 0;

	if (!(machine_is_omap_h2() || machine_is_omap_h3() || machine_is_netstar()))
		return -ENODEV;

	/* Allocate memory for MTD device structure and private data */
	omap_nand_mtd = kmalloc (sizeof(struct mtd_info) + sizeof (struct nand_chip),
				GFP_KERNEL);
	if (!omap_nand_mtd) {
		printk (KERN_WARNING "Unable to allocate NAND MTD device structure.\n");
		err = -ENOMEM;
		goto out;
	}

	/* Get pointer to private data */
	this = (struct nand_chip *) (&omap_nand_mtd[1]);

	/* Initialize structures */
	memset((char *) omap_nand_mtd, 0, sizeof(struct mtd_info) + sizeof(struct nand_chip));

	/* Link the private data with the MTD structure */
	omap_nand_mtd->priv = this;

	if (machine_is_omap_h2()) {
		/* FIXME on H2, R/B needs M7_1610_GPIO62 ... */
		this->chip_delay = 15;
		omap_cfg_reg(L3_1610_FLASH_CS2B_OE);
		omap_cfg_reg(M8_1610_FLASH_CS2B_WE);
	} else if (machine_is_omap_h3()) {
		if (omap_request_gpio(H3_NAND_RB_GPIO_PIN) != 0) {
			printk(KERN_ERR "NAND: Unable to get GPIO pin for R/B, use delay\n");
        		/* 15 us command delay time */
        		this->chip_delay = 15;
		} else {
			/* GPIO10 for input. it is in GPIO1 module */
			omap_set_gpio_direction(H3_NAND_RB_GPIO_PIN, 1);
		
			/* GPIO10 Func_MUX_CTRL reg bit 29:27, Configure V2 to mode1 as GPIO */
			/* GPIO10 pullup/down register, Enable pullup on GPIO10 */
			omap_cfg_reg(V2_1710_GPIO10);

			this->dev_ready = omap_nand_ready;
		}
	} else if (machine_is_netstar()) {
		if (omap_request_gpio(NETSTAR_NAND_RB_GPIO_PIN) != 0) {
			printk(KERN_ERR "NAND: Unable to get GPIO pin for R/B, use delay\n");
			/* 15 us command delay time */
			this->chip_delay = 15;
		} else {
			omap_set_gpio_direction(NETSTAR_NAND_RB_GPIO_PIN, 1);
			this->dev_ready = omap_nand_ready;
		}
	}

        /* try the first address */
	omap_nand_flash_base = ioremap(OMAP_NAND_FLASH_START1, SZ_4K);
	omap_nand_mtd->name = nand1_name;
	if (probe_nand_chip()){
		nandboot = 1;
		/* try the second address */
		iounmap(omap_nand_flash_base);
		omap_nand_flash_base = ioremap(OMAP_NAND_FLASH_START2, SZ_4K);
		if (probe_nand_chip()){
			iounmap(omap_nand_flash_base);
                        err = -ENXIO;
                        goto out_mtd;
		}
	}

	/* Register the partitions */
	switch(omap_nand_mtd->size) {
	case SZ_128M:
		if (!(machine_is_netstar()))
			goto out_unsupported;
		/* fall through */
	case SZ_32M:
#ifdef CONFIG_MTD_PARTITIONS
		err = parse_mtd_partitions(omap_nand_mtd, part_probes,
					&dynamic_partition, 0);
		if (err > 0)
			err = add_mtd_partitions(omap_nand_mtd,
					dynamic_partition, err);
		else if (nandboot)
			err = add_mtd_partitions(omap_nand_mtd,
					static_partition,
					ARRAY_SIZE(static_partition));
		else
#endif
			err = add_mtd_device(omap_nand_mtd);
		if (err)
			goto out_buf;
		break;
out_unsupported:
	default:
		printk(KERN_WARNING "Unsupported NAND device\n");
		err = -ENXIO;
		goto out_buf;
	}

	goto out;

out_buf:
	nand_release (omap_nand_mtd);
	if (this->dev_ready) {
		if (machine_is_omap_h2())
			omap_free_gpio(H2_NAND_RB_GPIO_PIN);
		else if (machine_is_omap_h3())
	 		omap_free_gpio(H3_NAND_RB_GPIO_PIN);
		else if (machine_is_netstar())
			omap_free_gpio(NETSTAR_NAND_RB_GPIO_PIN);
	}
	iounmap(omap_nand_flash_base);

out_mtd:
	kfree (omap_nand_mtd);
out:
	return err;
}

module_init(omap_nand_init);

/*
 * Clean up routine
 */
static void __exit omap_nand_cleanup (void)
{
        struct nand_chip *this = omap_nand_mtd->priv;
	if (this->dev_ready) {
		if (machine_is_omap_h2())
			omap_free_gpio(H2_NAND_RB_GPIO_PIN);
		else if (machine_is_omap_h3())
	 		omap_free_gpio(H3_NAND_RB_GPIO_PIN);
		else if (machine_is_netstar())
			omap_free_gpio(NETSTAR_NAND_RB_GPIO_PIN);
	}

	/* nand_release frees MTD partitions, MTD structure
	   and nand internal buffers*/
	nand_release (omap_nand_mtd);
	kfree (omap_nand_mtd);
 
	iounmap(omap_nand_flash_base);
}

module_exit(omap_nand_cleanup);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jian Zhang <jzhang@ti.com>");
MODULE_DESCRIPTION("Glue layer for NAND flash on H2/H3 boards");
