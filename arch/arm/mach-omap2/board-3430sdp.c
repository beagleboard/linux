/*
 * linux/arch/arm/mach-omap2/board-3430sdp.c
 *
 * Copyright (C) 2007 Texas Instruments
 *
 * Modified from mach-omap2/board-generic.c
 *
 * Initial code: Syed Mohammed Khasim
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/err.h>
#include <linux/clk.h>

#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>

#include <asm/arch/gpio.h>
#include <asm/arch/mux.h>
#include <asm/arch/board.h>
#include <asm/arch/common.h>
#include <asm/arch/dma.h>
#include <asm/arch/gpmc.h>

#include <asm/io.h>
#include <asm/delay.h>

#define	SDP3430_FLASH_CS	0
#define	SDP3430_SMC91X_CS	3

static struct mtd_partition sdp3430_partitions[] = {
	/* bootloader (U-Boot, etc) in first sector */
	{
		  .name		= "bootloader",
		  .offset		= 0,
		  .size		= SZ_256K,
		  .mask_flags	= MTD_WRITEABLE, /* force read-only */
	},
	/* bootloader params in the next sector */
	{
		  .name		= "params",
		  .offset		= MTDPART_OFS_APPEND,
		  .size		= SZ_128K,
		  .mask_flags	= 0,
	},
	/* kernel */
	{
		  .name		= "kernel",
		  .offset		= MTDPART_OFS_APPEND,
		  .size		= SZ_2M,
		  .mask_flags	= 0
	},
	/* file system */
	{
		  .name		= "filesystem",
		  .offset		= MTDPART_OFS_APPEND,
		  .size		= MTDPART_SIZ_FULL,
		  .mask_flags	= 0
	}
};

static struct flash_platform_data sdp3430_flash_data = {
	.map_name	= "cfi_probe",
	.width		= 2,
	.parts		= sdp3430_partitions,
	.nr_parts	= ARRAY_SIZE(sdp3430_partitions),
};

static struct resource sdp3430_flash_resource = {
	.start		= FLASH_BASE,
	.end		= FLASH_BASE + SZ_64M - 1,
	.flags		= IORESOURCE_MEM,
};

static struct platform_device sdp3430_flash_device = {
	.name		= "omapflash",
	.id		= 0,
	.dev		= {
		.platform_data	= &sdp3430_flash_data,
	},
	.num_resources	= 1,
	.resource	= &sdp3430_flash_resource,
};

static struct resource sdp3430_smc91x_resources[] = {
	[0] = {
		.start	= OMAP34XX_ETHR_START,
		.end	= OMAP34XX_ETHR_START + SZ_4K,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= OMAP_GPIO_IRQ(OMAP34XX_ETHR_GPIO_IRQ),
		.end	= 0,
		.flags		= IORESOURCE_IRQ,
	},
};

static struct platform_device sdp3430_smc91x_device = {
	.name		= "smc91x",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(sdp3430_smc91x_resources),
	.resource	= sdp3430_smc91x_resources,
};

static struct platform_device *sdp3430_devices[] __initdata = {
	&sdp3430_smc91x_device,
	&sdp3430_flash_device,
};

static inline void __init sdp3430_init_smc91x(void)
{
	int eth_cs;
	unsigned long cs_mem_base;
	unsigned int rate;
	struct clk *l3ck;

	eth_cs	= SDP3430_SMC91X_CS;

	l3ck = clk_get(NULL, "core_l3_ck");
	if (IS_ERR(l3ck))
		rate = 100000000;
	else
		rate = clk_get_rate(l3ck);

	if (gpmc_cs_request(eth_cs, SZ_16M, &cs_mem_base) < 0) {
		printk(KERN_ERR "Failed to request GPMC mem for smc91x\n");
		return;
	}

	sdp3430_smc91x_resources[0].start = cs_mem_base + 0x0;
	sdp3430_smc91x_resources[0].end   = cs_mem_base + 0xf;
	udelay(100);

	if (omap_request_gpio(OMAP34XX_ETHR_GPIO_IRQ) < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for smc91x IRQ\n",
			OMAP34XX_ETHR_GPIO_IRQ);
		return;
	}
	omap_set_gpio_direction(OMAP34XX_ETHR_GPIO_IRQ, 1);
}

static void __init omap_3430sdp_init_irq(void)
{
	omap2_init_common_hw();
	omap_init_irq();
	omap_gpio_init();
	sdp3430_init_smc91x();
}

static struct omap_uart_config sdp3430_uart_config __initdata = {
	.enabled_uarts	= ((1 << 0) | (1 << 1) | (1 << 2)),
};

static struct omap_board_config_kernel sdp3430_config[] = {
	{ OMAP_TAG_UART,	&sdp3430_uart_config },
};

static void __init omap_3430sdp_init(void)
{
	platform_add_devices(sdp3430_devices, ARRAY_SIZE(sdp3430_devices));
	omap_board_config = sdp3430_config;
	omap_board_config_size = ARRAY_SIZE(sdp3430_config);
	omap_serial_init();
}

static void __init omap_3430sdp_map_io(void)
{
	omap2_map_common_io();
}

MACHINE_START(OMAP_SDP3430, "OMAP3430 sdp3430 board")
	/* Maintainer: Syed Khasim - Texas Instruments Inc */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap_3430sdp_map_io,
	.init_irq	= omap_3430sdp_init_irq,
	.init_machine	= omap_3430sdp_init,
	.timer		= &omap_timer,
MACHINE_END
