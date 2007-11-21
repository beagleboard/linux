/*
 * linux/arch/arm/mach-omap2/board-2430osk.c
 *
 * Copyright (C) 2007 Mistral Software Pvt Ltd
 *
 * Modified from mach-omap2/board-generic.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>

#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <asm/arch/gpio.h>
#include <asm/arch/board.h>
#include <asm/arch/common.h>

#include <asm/io.h>

static void __init omap_2430osk_init_irq(void)
{
	omap2_init_common_hw();
	omap_init_irq();
	omap_gpio_init();
}

static struct omap_uart_config osk2430_uart_config __initdata = {
	.enabled_uarts	= ((1 << 0) | (1 << 1) | (1 << 2)),
};

static struct omap_board_config_kernel osk2430_config[] __initdata = {

	{OMAP_TAG_UART, &osk2430_uart_config},
};

#if	defined(CONFIG_I2C_OMAP) || defined(CONFIG_I2C_OMAP_MODULE)

#define OMAP2_I2C_BASE1		0x48070000
#define OMAP2_I2C_BASE2		0x48072000
#define OMAP2_I2C_INT1		56
#define OMAP2_I2C_INT2		57

static u32 omap2_i2c1_clkrate	= 400;
static u32 omap2_i2c2_clkrate	= 2600;

static struct resource i2c_resources1[] = {
	{
		.start	= OMAP2_I2C_BASE1,
		.end	= OMAP2_I2C_BASE1 + 0x3f,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= OMAP2_I2C_INT1,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource i2c_resources2[] = {
	{
		.start	= OMAP2_I2C_BASE2,
		.end	= OMAP2_I2C_BASE2 + 0x3f,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= OMAP2_I2C_INT2,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device omap_i2c_device1 = {
	.name		= "i2c_omap",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(i2c_resources1),
	.resource	= i2c_resources1,
	.dev		= {
		.platform_data	= &omap2_i2c1_clkrate,
	},
};

static struct platform_device omap_i2c_device2 = {
	.name		= "i2c_omap",
	.id		= 2,
	.num_resources	= ARRAY_SIZE(i2c_resources2),
	.resource	= i2c_resources2,
	.dev		= {
		.platform_data	= &omap2_i2c2_clkrate,
	},
};

static void omap_init_i2c(void)
{
	(void) platform_device_register(&omap_i2c_device2);
	(void) platform_device_register(&omap_i2c_device1);
}

#else

static void omap_init_i2c(void) {}

#endif

static int __init omap2430_i2c_init(void)
{
	omap_init_i2c();
	return 0;
}

static void __init omap_2430osk_init(void)
{
	omap_board_config = osk2430_config;
	omap_board_config_size = ARRAY_SIZE(osk2430_config);
	omap_serial_init();
}

static void __init omap_2430osk_map_io(void)
{
	omap2_set_globals_243x();
	omap2_map_common_io();
}

arch_initcall(omap2430_i2c_init);

MACHINE_START(OMAP_2430OSK, "OMAP2430 2430OSK board")
	/* Maintainer: Syed Khasim */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap_2430osk_map_io,
	.init_irq	= omap_2430osk_init_irq,
	.init_machine	= omap_2430osk_init,
	.timer		= &omap_timer,
MACHINE_END
