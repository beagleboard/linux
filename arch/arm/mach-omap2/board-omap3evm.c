/*
 * linux/arch/arm/mach-omap2/board-omap3evm.c
 *
 * Copyright (C) 2008 Texas Instruments
 *
 * Modified from mach-omap2/board-3430sdp.c
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
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/input.h>
#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>

#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <asm/arch/gpio.h>
#include <asm/arch/keypad.h>
#include <asm/arch/board.h>
#include <asm/arch/hsmmc.h>
#include <asm/arch/usb-musb.h>
#include <asm/arch/usb-ehci.h>
#include <asm/arch/common.h>
#include <asm/arch/mcspi.h>

static struct resource omap3evm_smc911x_resources[] = {
	[0] =	{
		.start  = OMAP3EVM_ETHR_START,
		.end    = (OMAP3EVM_ETHR_START + OMAP3EVM_ETHR_SIZE - 1),
		.flags  = IORESOURCE_MEM,
	},
	[1] =	{
		.start  = OMAP_GPIO_IRQ(OMAP3EVM_ETHR_GPIO_IRQ),
		.end    = OMAP_GPIO_IRQ(OMAP3EVM_ETHR_GPIO_IRQ),
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device omap3evm_smc911x_device = {
	.name		= "smc911x",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(omap3evm_smc911x_resources),
	.resource	= &omap3evm_smc911x_resources [0],
};

static inline void __init omap3evm_init_smc911x(void)
{
	int eth_cs;
	struct clk *l3ck;
	unsigned int rate;

	eth_cs = OMAP3EVM_SMC911X_CS;

	l3ck = clk_get(NULL, "l3_ck");
	if (IS_ERR(l3ck))
		rate = 100000000;
	else
		rate = clk_get_rate(l3ck);

	if (omap_request_gpio(OMAP3EVM_ETHR_GPIO_IRQ) < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for smc911x IRQ\n",
			OMAP3EVM_ETHR_GPIO_IRQ);
		return;
	}

	omap_set_gpio_direction(OMAP3EVM_ETHR_GPIO_IRQ, 1);
}

static struct omap_uart_config omap3_evm_uart_config __initdata = {
	.enabled_uarts	= ((1 << 0) | (1 << 1) | (1 << 2)),
};

static int __init omap3_evm_i2c_init(void)
{
	omap_register_i2c_bus(1, 2600, NULL, 0);
	omap_register_i2c_bus(2, 400, NULL, 0);
	omap_register_i2c_bus(3, 400, NULL, 0);
	return 0;
}

static struct omap_mmc_config omap3_evm_mmc_config __initdata = {
	.mmc [0] = {
		.enabled        = 1,
		.wire4          = 1,
	},
};

static struct platform_device omap3_evm_lcd_device = {
	.name		= "omap3evm_lcd",
	.id		= -1,
};

static struct omap_lcd_config omap3_evm_lcd_config __initdata = {
	.ctrl_name	= "internal",
};

static struct platform_device omap3_evm_twl4030rtc_device = {
	.name		= "twl4030_rtc",
	.id		= -1,
};

static void ads7846_dev_init(void)
{
	if (omap_request_gpio(OMAP3_EVM_TS_GPIO) < 0)
		printk(KERN_ERR "can't get ads7846 pen down GPIO\n");

	omap_set_gpio_direction(OMAP3_EVM_TS_GPIO, 1);

	omap_set_gpio_debounce(OMAP3_EVM_TS_GPIO, 1);
	omap_set_gpio_debounce_time(OMAP3_EVM_TS_GPIO, 0xa);
}

static int ads7846_get_pendown_state(void)
{
	return !omap_get_gpio_datain(OMAP3_EVM_TS_GPIO);
}

struct ads7846_platform_data ads7846_config = {
	.get_pendown_state	= ads7846_get_pendown_state,
	.keep_vref_on		= 1,
};

static struct omap2_mcspi_device_config ads7846_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,  /* 0: slave, 1: master */
};

struct spi_board_info omap3evm_spi_board_info[] = {
	[0] = {
		.modalias		= "ads7846",
		.bus_num		= 1,
		.chip_select		= 0,
		.max_speed_hz		= 1500000,
		.controller_data	= &ads7846_mcspi_config,
		.irq			= OMAP_GPIO_IRQ(OMAP3_EVM_TS_GPIO),
		.platform_data		= &ads7846_config,
	},
};

static int omap3evm_keymap[] = {
	KEY(0, 0, KEY_LEFT),
	KEY(0, 1, KEY_RIGHT),
	KEY(0, 2, KEY_A),
	KEY(0, 3, KEY_B),
	KEY(1, 0, KEY_DOWN),
	KEY(1, 1, KEY_UP),
	KEY(1, 2, KEY_E),
	KEY(1, 3, KEY_F),
	KEY(2, 0, KEY_ENTER),
	KEY(2, 1, KEY_I),
	KEY(2, 2, KEY_J),
	KEY(2, 3, KEY_K),
	KEY(3, 0, KEY_M),
	KEY(3, 1, KEY_N),
	KEY(3, 2, KEY_O),
	KEY(3, 3, KEY_P)
};

static struct omap_kp_platform_data omap3evm_kp_data = {
	.rows		= 4,
	.cols		= 4,
	.keymap 	= omap3evm_keymap,
	.keymapsize	= ARRAY_SIZE(omap3evm_keymap),
	.rep		= 1,
};

static struct platform_device omap3evm_kp_device = {
	.name		= "omap_twl4030keypad",
	.id		= -1,
	.dev		= {
				.platform_data = &omap3evm_kp_data,
			},
};

static void __init omap3_evm_init_irq(void)
{
	omap2_init_common_hw();
	omap_init_irq();
	omap_gpio_init();
	omap3evm_init_smc911x();
}

static struct omap_board_config_kernel omap3_evm_config[] __initdata = {
	{ OMAP_TAG_UART,	&omap3_evm_uart_config },
	{ OMAP_TAG_MMC,		&omap3_evm_mmc_config },
	{ OMAP_TAG_LCD,		&omap3_evm_lcd_config },
};

static struct platform_device *omap3_evm_devices[] __initdata = {
	&omap3_evm_lcd_device,
	&omap3evm_kp_device,
#ifdef CONFIG_RTC_DRV_TWL4030
	&omap3_evm_twl4030rtc_device,
#endif
	&omap3evm_smc911x_device,
};

static void __init omap3_evm_init(void)
{
	platform_add_devices(omap3_evm_devices, ARRAY_SIZE(omap3_evm_devices));
	omap_board_config = omap3_evm_config;
	omap_board_config_size = ARRAY_SIZE(omap3_evm_config);

	spi_register_board_info(omap3evm_spi_board_info,
				ARRAY_SIZE(omap3evm_spi_board_info));

	omap_serial_init();
	hsmmc_init();
	usb_musb_init();
	usb_ehci_init();
	omap3evm_flash_init();
	ads7846_dev_init();
}

arch_initcall(omap3_evm_i2c_init);

static void __init omap3_evm_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

MACHINE_START(OMAP3EVM, "OMAP3 EVM")
	/* Maintainer: Syed Mohammed Khasim - Texas Instruments */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap3_evm_map_io,
	.init_irq	= omap3_evm_init_irq,
	.init_machine	= omap3_evm_init,
	.timer		= &omap_timer,
MACHINE_END
