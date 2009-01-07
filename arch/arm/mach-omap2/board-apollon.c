/*
 * linux/arch/arm/mach-omap2/board-apollon.c
 *
 * Copyright (C) 2005-2008 Samsung Electronics
 * Author: Kyungmin Park <kyungmin.park@samsung.com>
 *
 * Modified from mach-omap2/board-h4.c
 *
 * Code for apollon OMAP2 board. Should work on many OMAP2 systems where
 * the bootloader passes the board-specific data to the kernel.
 * Do not put any board specific code to this file; create a new machine
 * type if you need custom low-level initializations.
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
#include <linux/mtd/onenand.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/spi/spi.h>
#include <linux/spi/tsc210x.h>

#include <asm/io.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/flash.h>

#include <mach/gpio.h>
#include <mach/led.h>
#include <mach/mux.h>
#include <mach/usb.h>
#include <mach/board.h>
#include <mach/common.h>
#include <mach/gpmc.h>

/* LED & Switch macros */
#define LED0_GPIO13		13
#define LED1_GPIO14		14
#define LED2_GPIO15		15
#define LED3_GPIO92		92
#define LED4_GPIO93		93

#define APOLLON_FLASH_CS	0
#define APOLLON_ETH_CS		1
#define APOLLON_NOR_CS		3

#define APOLLON_ONENAND_CS2_ADDRESS	(0x00000e40 | (0x10000000 >> 24))
#define APOLLON_EXT_CS3_ADDRESS		(0x00000c40 | (0x18000000 >> 24))

static struct mtd_partition apollon_partitions[] = {
	{
		.name		= "X-Loader + U-Boot",
		.offset		= 0,
		.size		= SZ_128K,
		.mask_flags	= MTD_WRITEABLE,
	},
	{
		.name		= "params",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_128K,
	},
	{
		.name		= "kernel",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_2M,
	},
	{
		.name		= "rootfs",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_16M,
	},
	{
		.name		= "filesystem00",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_32M,
	},
	{
		.name		= "filesystem01",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
	},
};

static struct flash_platform_data apollon_flash_data = {
	.parts		= apollon_partitions,
	.nr_parts	= ARRAY_SIZE(apollon_partitions),
};

static struct resource apollon_flash_resource[] = {
	[0] = {
		.flags		= IORESOURCE_MEM,
	},
};

static struct platform_device apollon_onenand_device = {
	.name		= "onenand",
	.id		= -1,
	.dev		= {
		.platform_data	= &apollon_flash_data,
	},
	.num_resources	= ARRAY_SIZE(apollon_flash_resource),
	.resource	= apollon_flash_resource,
};

static struct mtd_partition apollon_nor_partitions[] = {
	{
		.name		= "U-Boot",
		.offset		= 0,
		.size		= SZ_128K,
		.mask_flags	= MTD_WRITEABLE,
	},
	{
		.name		= "params",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_128K,
	},
	{
		.name		= "kernel",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_2M,
	},
	{
		.name		= "rootfs",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_4M - SZ_256K,
	},
	{
		.name		= "application",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_8M + SZ_2M,
	},
	{
		.name		= "reserved",
		.offset		= MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
	},
};

static struct flash_platform_data apollon_nor_data = {
	.map_name		= "cfi_probe",
	.width		= 2,
	.parts		= apollon_nor_partitions,
	.nr_parts	= ARRAY_SIZE(apollon_nor_partitions),
};

static struct resource apollon_nor_resource[] = {
	[0] = {
		.flags	= IORESOURCE_MEM,
	}
};

static struct platform_device apollon_nor_device = {
	.name		= "omapflash",
	.id		= -1,
	.dev		= {
		.platform_data	= &apollon_nor_data,
	},
	.num_resources	= ARRAY_SIZE(apollon_nor_resource),
	.resource	= apollon_nor_resource,
};

static void __init apollon_flash_init(void)
{
	unsigned long base;

	if (gpmc_cs_request(APOLLON_FLASH_CS, SZ_128K, &base) < 0) {
		printk(KERN_ERR "Cannot request OneNAND GPMC CS\n");
		return;
	}
	apollon_flash_resource[0].start = base;
	apollon_flash_resource[0].end   = base + SZ_128K - 1;

	if (gpmc_cs_request(APOLLON_NOR_CS, SZ_32M, &base) < 0) {
		printk(KERN_ERR "Cannot request NOR GPMC CS\n");
		return;
	}
	apollon_nor_resource[0].start = base;
	apollon_nor_resource[0].end   = base + SZ_32M - 1;
}

static struct resource apollon_smc91x_resources[] = {
	[0] = {
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start	= gpio_to_irq(APOLLON_ETHR_GPIO_IRQ),
		.end	= gpio_to_irq(APOLLON_ETHR_GPIO_IRQ),
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	},
};

static struct platform_device apollon_smc91x_device = {
	.name		= "smc91x",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(apollon_smc91x_resources),
	.resource	= apollon_smc91x_resources,
};

static struct platform_device apollon_lcd_device = {
	.name		= "apollon_lcd",
	.id		= -1,
};

static struct gpio_led apollon_led_config[] = {
	{
		.name			= "d2",
		.gpio			= LED0_GPIO13,
		.default_trigger	= "heartbeat",
	},
	{
		.name			= "d3",
		.gpio			= LED1_GPIO14,
	},
	{
		.name			= "d4",
		.gpio			= LED2_GPIO15,
	},
	{
		.name			= "d5",
		.gpio			= LED3_GPIO92,
	},
	{
		.name			= "d6",
		.gpio			= LED4_GPIO93,
	},
};

static struct gpio_led_platform_data apollon_led_data = {
	.num_leds	= ARRAY_SIZE(apollon_led_config),
	.leds		= apollon_led_config,
};

static struct platform_device apollon_led_device = {
	.name		= "leds-gpio",
	.id		= -1,
	.dev		= {
		.platform_data	= &apollon_led_data,
	},
};

static struct platform_device *apollon_devices[] __initdata = {
	&apollon_onenand_device,
	&apollon_nor_device,
	&apollon_smc91x_device,
	&apollon_lcd_device,
	&apollon_led_device,
};

static inline void __init apollon_init_smc91x(void)
{
	unsigned long base;
	unsigned int rate;
	struct clk *gpmc_fck;
	int eth_cs;

	gpmc_fck = clk_get(NULL, "gpmc_fck");	/* Always on ENABLE_ON_INIT */
	if (IS_ERR(gpmc_fck)) {
		WARN_ON(1);
		return;
	}

	clk_enable(gpmc_fck);
	rate = clk_get_rate(gpmc_fck);

	eth_cs = APOLLON_ETH_CS;

	/* Make sure CS1 timings are correct */
	gpmc_cs_write_reg(eth_cs, GPMC_CS_CONFIG1, 0x00011200);

	if (rate >= 160000000) {
		gpmc_cs_write_reg(eth_cs, GPMC_CS_CONFIG2, 0x001f1f01);
		gpmc_cs_write_reg(eth_cs, GPMC_CS_CONFIG3, 0x00080803);
		gpmc_cs_write_reg(eth_cs, GPMC_CS_CONFIG4, 0x1c0b1c0a);
		gpmc_cs_write_reg(eth_cs, GPMC_CS_CONFIG5, 0x041f1F1F);
		gpmc_cs_write_reg(eth_cs, GPMC_CS_CONFIG6, 0x000004C4);
	} else if (rate >= 130000000) {
		gpmc_cs_write_reg(eth_cs, GPMC_CS_CONFIG2, 0x001f1f00);
		gpmc_cs_write_reg(eth_cs, GPMC_CS_CONFIG3, 0x00080802);
		gpmc_cs_write_reg(eth_cs, GPMC_CS_CONFIG4, 0x1C091C09);
		gpmc_cs_write_reg(eth_cs, GPMC_CS_CONFIG5, 0x041f1F1F);
		gpmc_cs_write_reg(eth_cs, GPMC_CS_CONFIG6, 0x000004C4);
	} else {/* rate = 100000000 */
		gpmc_cs_write_reg(eth_cs, GPMC_CS_CONFIG2, 0x001f1f00);
		gpmc_cs_write_reg(eth_cs, GPMC_CS_CONFIG3, 0x00080802);
		gpmc_cs_write_reg(eth_cs, GPMC_CS_CONFIG4, 0x1C091C09);
		gpmc_cs_write_reg(eth_cs, GPMC_CS_CONFIG5, 0x031A1F1F);
		gpmc_cs_write_reg(eth_cs, GPMC_CS_CONFIG6, 0x000003C2);
	}

	if (gpmc_cs_request(eth_cs, SZ_16M, &base) < 0) {
		printk(KERN_ERR "Failed to request GPMC CS for smc91x\n");
		goto out;
	}
	apollon_smc91x_resources[0].start = base + 0x300;
	apollon_smc91x_resources[0].end   = base + 0x30f;
	udelay(100);

	omap_cfg_reg(W4__24XX_GPIO74);
	if (gpio_request(APOLLON_ETHR_GPIO_IRQ, "SMC91x irq") < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for smc91x IRQ\n",
			APOLLON_ETHR_GPIO_IRQ);
		gpmc_cs_free(eth_cs);
		goto out;
	}
	gpio_direction_input(APOLLON_ETHR_GPIO_IRQ);

out:
	clk_disable(gpmc_fck);
	clk_put(gpmc_fck);
}

static void __init omap_apollon_init_irq(void)
{
	omap2_init_common_hw(NULL);
	omap_init_irq();
	omap_gpio_init();
	apollon_init_smc91x();
}

static struct tsc210x_config tsc_platform_data = {
	.use_internal		= 1,
	.monitor		= TSC_TEMP,
	.mclk			= "sys_clkout",
};

static struct spi_board_info apollon_spi_board_info[] = {
	[0] = {
		.modalias	= "tsc2101",
		.irq		= OMAP_GPIO_IRQ(85),
		.bus_num	= 1,
		.chip_select	= 0,
		.mode		= SPI_MODE_1,
		.max_speed_hz	= 6000000,
		.platform_data	= &tsc_platform_data,
	},
};

static struct omap_uart_config apollon_uart_config __initdata = {
	.enabled_uarts = (1 << 0) | (0 << 1) | (0 << 2),
};

static struct omap_usb_config apollon_usb_config __initdata = {
	.register_dev	= 1,
	.hmc_mode	= 0x14,	/* 0:dev 1:host1 2:disable */

	.pins[0]	= 6,
};

static struct omap_lcd_config apollon_lcd_config __initdata = {
	.ctrl_name	= "internal",
};

static struct omap_board_config_kernel apollon_config[] __initdata = {
	{ OMAP_TAG_UART,	&apollon_uart_config },
	{ OMAP_TAG_USB,		&apollon_usb_config },
	{ OMAP_TAG_LCD,		&apollon_lcd_config },
};

static void __init apollon_led_init(void)
{
	/* LED0 - AA10 */
	omap_cfg_reg(AA10_242X_GPIO13);
	/* LED1  - AA6 */
	omap_cfg_reg(AA6_242X_GPIO14);
	/* LED2  - AA4 */
	omap_cfg_reg(AA4_242X_GPIO15);

	if (apollon_plus()) {
		/* LED3 - M15 */
		omap_cfg_reg(M15_24XX_GPIO92);
		/* LED4 - P20 */
		omap_cfg_reg(P20_24XX_GPIO93);
	} else
		apollon_led_data.num_leds = 3;
}

static void __init apollon_usb_init(void)
{
	/* USB device */
	/* DEVICE_SUSPEND */
	omap_cfg_reg(P21_242X_GPIO12);
	gpio_request(12, "USB suspend");
	gpio_direction_output(12, 0);
}

static void __init apollon_tsc_init(void)
{
	/* TSC2101 */
	omap_cfg_reg(N15_24XX_GPIO85);
	gpio_request(85, "tsc2101 irq");
	gpio_direction_input(85);

	omap_cfg_reg(W14_24XX_SYS_CLKOUT);	/* mclk */
}

static void __init apollon_cs_init(void)
{
	unsigned long base;
	unsigned int rate;
	struct clk *l3ck;
	u32 value;
	int cs, sync = 0;

	l3ck = clk_get(NULL, "core_l3_ck");
	if (IS_ERR(l3ck))
		rate = 100000000;
	else
		rate = clk_get_rate(l3ck);

	/* CS2: OneNAND */
	cs = 2;
	value = gpmc_cs_read_reg(0, GPMC_CS_CONFIG1);
	gpmc_cs_write_reg(cs, GPMC_CS_CONFIG1, value);
	value = gpmc_cs_read_reg(0, GPMC_CS_CONFIG2);
	gpmc_cs_write_reg(cs, GPMC_CS_CONFIG2, value);
	value = gpmc_cs_read_reg(0, GPMC_CS_CONFIG3);
	gpmc_cs_write_reg(cs, GPMC_CS_CONFIG3, value);
	value = gpmc_cs_read_reg(0, GPMC_CS_CONFIG4);
	gpmc_cs_write_reg(cs, GPMC_CS_CONFIG4, value);
	value = gpmc_cs_read_reg(0, GPMC_CS_CONFIG5);
	gpmc_cs_write_reg(cs, GPMC_CS_CONFIG5, value);
	value = gpmc_cs_read_reg(0, GPMC_CS_CONFIG6);
	gpmc_cs_write_reg(cs, GPMC_CS_CONFIG6, value);

	gpmc_cs_write_reg(cs, GPMC_CS_CONFIG7, APOLLON_ONENAND_CS2_ADDRESS);

	/* CS3: External NOR */
	cs = APOLLON_NOR_CS;
	if (rate >= 160000000) {
		sync = 1;
		gpmc_cs_write_reg(cs, GPMC_CS_CONFIG1, 0xe5011211);
		gpmc_cs_write_reg(cs, GPMC_CS_CONFIG2, 0x00090b01);
		gpmc_cs_write_reg(cs, GPMC_CS_CONFIG3, 0x00020201);
		gpmc_cs_write_reg(cs, GPMC_CS_CONFIG4, 0x09030b03);
		gpmc_cs_write_reg(cs, GPMC_CS_CONFIG5, 0x010a0a0c);
		gpmc_cs_write_reg(cs, GPMC_CS_CONFIG6, 0x00000000);
	} else if (rate >= 130000000) {
		/* Not yet know ... Use the async values */
		gpmc_cs_write_reg(cs, GPMC_CS_CONFIG1, 0x00021201);
		gpmc_cs_write_reg(cs, GPMC_CS_CONFIG2, 0x00121601);
		gpmc_cs_write_reg(cs, GPMC_CS_CONFIG3, 0x00040401);
		gpmc_cs_write_reg(cs, GPMC_CS_CONFIG4, 0x12061605);
		gpmc_cs_write_reg(cs, GPMC_CS_CONFIG5, 0x01151317);
		gpmc_cs_write_reg(cs, GPMC_CS_CONFIG6, 0x00000000);
	} else {/* rate = 100000000 */
		sync = 1;
		gpmc_cs_write_reg(cs, GPMC_CS_CONFIG1, 0xe1001202);
		gpmc_cs_write_reg(cs, GPMC_CS_CONFIG2, 0x00151501);
		gpmc_cs_write_reg(cs, GPMC_CS_CONFIG3, 0x00050501);
		gpmc_cs_write_reg(cs, GPMC_CS_CONFIG4, 0x0e070e07);
		gpmc_cs_write_reg(cs, GPMC_CS_CONFIG5, 0x01131F1F);
		gpmc_cs_write_reg(cs, GPMC_CS_CONFIG6, 0x00000000);
	}

	gpmc_cs_write_reg(cs, GPMC_CS_CONFIG7, APOLLON_EXT_CS3_ADDRESS);

	if (gpmc_cs_request(cs, SZ_32M, &base) < 0) {
		printk(KERN_ERR "Failed to request GPMC CS for external\n");
		return;
	}

	/* Synchronous mode */
	if (sync) {
		void __iomem *addr = ioremap(base, SZ_32M);
		writew(0xaa, addr + 0xaaa);
		writew(0x55, addr + 0x554);
		writew(0xc0, addr + 0x24aaa);
		iounmap(addr);
	}

	gpmc_cs_free(cs);
}

static void __init omap_apollon_init(void)
{
	apollon_cs_init();
	apollon_led_init();
	apollon_flash_init();
	apollon_usb_init();
	apollon_tsc_init();

	/*
 	 * Make sure the serial ports are muxed on at this point.
	 * You have to mux them off in device drivers later on
	 * if not needed.
	 */
	platform_add_devices(apollon_devices, ARRAY_SIZE(apollon_devices));
	omap_board_config = apollon_config;
	omap_board_config_size = ARRAY_SIZE(apollon_config);
	omap_serial_init();
	omap_register_i2c_bus(1, 100, NULL, 0);
	omap_register_i2c_bus(2, 100, NULL, 0);

	spi_register_board_info(apollon_spi_board_info,
				ARRAY_SIZE(apollon_spi_board_info));

	apollon_mmc_init();
}

static void __init omap_apollon_map_io(void)
{
	omap2_set_globals_242x();
	omap2_map_common_io();
}

MACHINE_START(OMAP_APOLLON, "OMAP24xx Apollon")
	/* Maintainer: Kyungmin Park <kyungmin.park@samsung.com> */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap_apollon_map_io,
	.init_irq	= omap_apollon_init_irq,
	.init_machine	= omap_apollon_init,
	.timer		= &omap_timer,
MACHINE_END
