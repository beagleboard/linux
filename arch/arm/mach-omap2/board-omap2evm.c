/*
 * linux/arch/arm/mach-omap2/board-omap2evm.c
 *
 * Copyright (C) 2008 Mistral Solutions Pvt Ltd
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
#include <linux/io.h>

#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <asm/arch/gpio.h>
#include <asm/arch/board.h>
#include <asm/arch/common.h>
#include <asm/arch/hsmmc.h>

static struct resource omap2evm_smc911x_resources[] = {
	[0] =   {
		.start  = OMAP2EVM_ETHR_START,
		.end    = (OMAP2EVM_ETHR_START + OMAP2EVM_ETHR_SIZE - 1),
		.flags  = IORESOURCE_MEM,
	},
	[1] =   {
		.start  = OMAP_GPIO_IRQ(OMAP2EVM_ETHR_GPIO_IRQ),
		.end    = OMAP_GPIO_IRQ(OMAP2EVM_ETHR_GPIO_IRQ),
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device omap2evm_smc911x_device = {
	.name       = "smc911x",
	.id     = -1,
	.num_resources  = ARRAY_SIZE(omap2evm_smc911x_resources),
	.resource   = &omap2evm_smc911x_resources [0],
};

static inline void __init omap2evm_init_smc911x(void)
{
	int gpio = OMAP2EVM_ETHR_GPIO_IRQ;
	int ret;

	ret = gpio_request(gpio, "smc911x IRQ");
	if (ret < 0) {
		printk(KERN_ERR "Failed to request GPIO %d for smc911x IRQ\n",
				gpio);
		return;
	}
	gpio_direction_input(gpio);

}

static struct platform_device omap2_evm_lcd_device = {
	.name		= "omap2evm_lcd",
	.id		= -1,
};

static struct omap_lcd_config omap2_evm_lcd_config __initdata = {
	.ctrl_name	= "internal",
};

static void __init omap2_evm_init_irq(void)
{
	omap2_init_common_hw(NULL);
	omap_init_irq();
	omap_gpio_init();
	omap2evm_init_smc911x();
}

static struct omap_uart_config omap2_evm_uart_config __initdata = {
	.enabled_uarts	= ((1 << 0) | (1 << 1) | (1 << 2)),
};

static struct omap_mmc_config omap2_evm_mmc_config __initdata = {
	.mmc [0] = {
		.enabled        = 1,
		.wire4          = 1,
	},
};

static struct omap_board_config_kernel omap2_evm_config[] __initdata = {
	{ OMAP_TAG_UART,	&omap2_evm_uart_config },
	{ OMAP_TAG_LCD,		&omap2_evm_lcd_config },
	{ OMAP_TAG_MMC,		&omap2_evm_mmc_config },
};

static int __init omap2_evm_i2c_init(void)
{
	/*
	 * Registering bus 2 first to avoid twl4030 misbehaving as OMAP2EVM
	 * has twl4030 on bus 2
	 */
	omap_register_i2c_bus(2, 2600, NULL, 0);
	omap_register_i2c_bus(1, 400, NULL, 0);
	return 0;
}

static struct platform_device *omap2_evm_devices[] __initdata = {
	&omap2_evm_lcd_device,
	&omap2evm_smc911x_device,
};

static void __init omap2_evm_init(void)
{
	platform_add_devices(omap2_evm_devices, ARRAY_SIZE(omap2_evm_devices));
	omap_board_config = omap2_evm_config;
	omap_board_config_size = ARRAY_SIZE(omap2_evm_config);
	omap_serial_init();
	hsmmc_init();
}

static void __init omap2_evm_map_io(void)
{
	omap2_set_globals_243x();
	omap2_map_common_io();
}

arch_initcall(omap2_evm_i2c_init);

MACHINE_START(OMAP2EVM, "OMAP2EVM Board")
	/* Maintainer:  Arun KS <arunks@mistralsolutions.com> */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap2_evm_map_io,
	.init_irq	= omap2_evm_init_irq,
	.init_machine	= omap2_evm_init,
	.timer		= &omap_timer,
MACHINE_END
