/*
 * linux/arch/arm/mach-omap1/board-nokia770.c
 *
 * Modified from board-generic.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>

#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <asm/arch/gpio.h>
#include <asm/arch/mux.h>
#include <asm/arch/usb.h>
#include <asm/arch/board.h>
#include <asm/arch/common.h>

static void __init omap_nokia770_init_irq(void)
{
	omap1_init_common_hw();
	omap_init_irq();
}

static struct spi_board_info nokia770_spi_board_info[] __initdata = {
	[0] = {
		.modalias       = "lcd_lph8923",
		.bus_num        = 2,
		.chip_select    = 3,
		.max_speed_hz   = 12000000,
	},
};

static struct platform_device *nokia770_devices[] __initdata = {
};

/* assume no Mini-AB port */

static struct omap_usb_config nokia770_usb_config __initdata = {
	.otg		= 1,
	.register_host	= 1,
	.register_dev	= 1,
	.hmc_mode	= 16,
	.pins[0]	= 6,
};

static struct omap_mmc_config nokia770_mmc_config __initdata = {
	.mmc[0] = {
		.enabled	= 0,
		.wire4		= 0,
		.wp_pin		= -1,
		.power_pin	= -1,
		.switch_pin	= -1,
	},
	.mmc[1] = {
		.enabled	= 0,
		.wire4		= 0,
		.wp_pin		= -1,
		.power_pin	= -1,
		.switch_pin	= -1,
	},
};

static struct omap_board_config_kernel nokia770_config[] = {
	{ OMAP_TAG_USB,		NULL },
	{ OMAP_TAG_MMC,		&nokia770_mmc_config },
};

static void __init omap_nokia770_init(void)
{
	nokia770_config[0].data = &nokia770_usb_config;

	platform_add_devices(nokia770_devices, ARRAY_SIZE(nokia770_devices));
	spi_register_board_info(nokia770_spi_board_info,
				ARRAY_SIZE(nokia770_spi_board_info));
	omap_board_config = nokia770_config;
	omap_board_config_size = ARRAY_SIZE(nokia770_config);
	omap_serial_init();
}

static void __init omap_nokia770_map_io(void)
{
	omap1_map_common_io();
}

MACHINE_START(NOKIA770, "Nokia 770")
	.phys_io	= 0xfff00000,
	.io_pg_offst	= ((0xfef00000) >> 18) & 0xfffc,
	.boot_params	= 0x10000100,
	.map_io		= omap_nokia770_map_io,
	.init_irq	= omap_nokia770_init_irq,
	.init_machine	= omap_nokia770_init,
	.timer		= &omap_timer,
MACHINE_END
