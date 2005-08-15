/*
 * linux/arch/arm/mach-omap/omap2/board-generic.c
 *
 * Copyright (C) 2005 Nokia Corporation
 * Author: Paul Mundt <paul.mundt@nokia.com>
 *
 * Modified from mach-omap/omap1/board-generic.c
 *
 * Code for generic OMAP2 board. Should work on many OMAP2 systems where
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
#include <linux/device.h>

#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <asm/arch/gpio.h>
#include <asm/arch/mux.h>
#include <asm/arch/usb.h>
#include <asm/arch/board.h>
#include <asm/arch/common.h>

static int __initdata generic_serial_ports[OMAP_MAX_NR_PORTS] = {1, 1, 1};

static void __init omap_generic_init_irq(void)
{
	omap_init_irq();
}

static struct omap_board_config_kernel generic_config[] = {
};

static void __init omap_generic_init(void)
{
	const struct omap_uart_config *uart_conf;

	/*
	 * Make sure the serial ports are muxed on at this point.
	 * You have to mux them off in device drivers later on
	 * if not needed.
	 */

	uart_conf = omap_get_config(OMAP_TAG_UART, struct omap_uart_config);
	if (uart_conf != NULL) {
		unsigned int enabled_ports, i;

		enabled_ports = uart_conf->enabled_uarts;
		for (i = 0; i < 3; i++) {
			if (!(enabled_ports & (1 << i)))
				generic_serial_ports[i] = 0;
		}
	}

	omap_board_config = generic_config;
	omap_board_config_size = ARRAY_SIZE(generic_config);
	omap_serial_init(generic_serial_ports);
}

static void __init omap_generic_map_io(void)
{
	omap_map_common_io();
}

MACHINE_START(OMAP_GENERIC, "Generic OMAP24xx")
	/* Maintainer: Paul Mundt <paul.mundt@nokia.com> */
	.phys_ram	= 0x80000000,
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap_generic_map_io,
	.init_irq	= omap_generic_init_irq,
	.init_machine	= omap_generic_init,
	.timer		= &omap_timer,
MACHINE_END
