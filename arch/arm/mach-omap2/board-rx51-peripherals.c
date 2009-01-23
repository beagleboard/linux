/*
 * linux/arch/arm/mach-omap2/board-rx51-flash.c
 *
 * Copyright (C) 2008 Nokia
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/spi/spi.h>
#include <linux/spi/tsc2005.h>
#include <linux/i2c.h>
#include <linux/i2c/twl4030.h>
#include <linux/clk.h>
#include <linux/delay.h>

#include <mach/mcspi.h>
#include <mach/gpio.h>
#include <mach/mux.h>
#include <mach/board.h>
#include <mach/common.h>
#include <mach/keypad.h>
#include <mach/dma.h>
#include <mach/gpmc.h>

#define RX51_DEBUG_BASE			0x08000000  /* debug board */
#define RX51_ETHR_START			RX51_DEBUG_BASE
#define RX51_ETHR_GPIO_IRQ		54

#define RX51_TSC2005_RESET_GPIO		104
#define RX51_TSC2005_IRQ_GPIO		100

#define	RX51_SMC91X_CS			1

static struct resource rx51_smc91x_resources[] = {
	[0] = {
		.start	= RX51_ETHR_START,
		.end	= RX51_ETHR_START + SZ_4K,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= OMAP_GPIO_IRQ(RX51_ETHR_GPIO_IRQ),
		.end	= 0,
		.flags		= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	},
};

static struct platform_device rx51_smc91x_device = {
	.name		= "smc91x",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(rx51_smc91x_resources),
	.resource	= rx51_smc91x_resources,
};

static struct tsc2005_platform_data tsc2005_config = {
	.reset_gpio 		= RX51_TSC2005_RESET_GPIO, /* not used */

	.ts_x_plate_ohm		= 280,
	.ts_hw_avg		= 0,
	.ts_touch_pressure	= 1500,
	.ts_stab_time		= 1000,
	.ts_pressure_max	= 2048,
	.ts_pressure_fudge	= 2,
	.ts_x_max		= 4096,
	.ts_x_fudge		= 4,
	.ts_y_max		= 4096,
	.ts_y_fudge		= 7,
};

static struct omap2_mcspi_device_config tsc2005_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel = 1,
};


static struct spi_board_info rx51_peripherals_spi_board_info[] = {
	[0] = {
		.modalias		= "tsc2005",
		.bus_num		= 1,
		.chip_select		= 0,
		.irq	 		= OMAP_GPIO_IRQ(RX51_TSC2005_IRQ_GPIO),
		.max_speed_hz   	= 6000000,
		.controller_data	= &tsc2005_mcspi_config,
		.platform_data		= &tsc2005_config,
	},
};

static int rx51_keymap[] = {
	KEY(0, 0, KEY_Q),
	KEY(0, 1, KEY_W),
	KEY(0, 2, KEY_E),
	KEY(0, 3, KEY_R),
	KEY(0, 4, KEY_T),
	KEY(0, 5, KEY_Y),
	KEY(0, 6, KEY_U),
	KEY(0, 7, KEY_I),
	KEY(1, 0, KEY_O),
	KEY(1, 1, KEY_D),
	KEY(1, 2, KEY_DOT ),
	KEY(1, 3, KEY_V),
	KEY(1, 4, KEY_DOWN ),
	KEY(2, 0, KEY_P),
	KEY(2, 1, KEY_F),
	KEY(2, 2, KEY_UP ),
	KEY(2, 3, KEY_B),
	KEY(2, 4, KEY_RIGHT ),
	KEY(3, 0, KEY_COMMA ),
	KEY(3, 1, KEY_G),
	KEY(3, 2, KEY_ENTER ),
	KEY(3, 3, KEY_N),
	KEY(4, 0, KEY_BACKSPACE ),
	KEY(4, 1, KEY_H),
	KEY(4, 3, KEY_M),
	KEY(4, 4, KEY_LEFTCTRL),
	KEY(5, 1, KEY_J),
	KEY(5, 2, KEY_Z),
	KEY(5, 3, KEY_SPACE),
	KEY(5, 4, KEY_LEFTSHIFT),
	KEY(6, 0, KEY_A),
	KEY(6, 1, KEY_K),
	KEY(6, 2, KEY_X),
	KEY(6, 3, KEY_SPACE),
	KEY(6, 4, KEY_FN ),
	KEY(7, 0, KEY_S),
	KEY(7, 1, KEY_L),
	KEY(7, 2, KEY_C),
	KEY(7, 3, KEY_LEFT),
	KEY(0xff, 0, KEY_F6),
	KEY(0xff, 1, KEY_F7),
	KEY(0xff, 2, KEY_F8),
	KEY(0xff, 4, KEY_F9),
	KEY(0xff, 5, KEY_F10),
};

static struct twl4030_keypad_data rx51_kp_data = {
	.rows		= 8,
	.cols		= 8,
	.keymap 	= rx51_keymap,
	.keymapsize 	= ARRAY_SIZE(rx51_keymap),
	.rep		= 1,
};

static struct platform_device *rx51_peripherals_devices[] = {
	&rx51_smc91x_device,
};

static void __init rx51_init_smc91x(void)
{
	int eth_cs;
	unsigned long cs_mem_base;
	unsigned int rate;
	struct clk *l3ck;

	eth_cs	= RX51_SMC91X_CS;

	l3ck = clk_get(NULL, "core_l3_ck");
	if (IS_ERR(l3ck))
		rate = 100000000;
	else
		rate = clk_get_rate(l3ck);

	if (gpmc_cs_request(eth_cs, SZ_16M, &cs_mem_base) < 0) {
		printk(KERN_ERR "Failed to request GPMC mem for smc91x\n");
		return;
	}

	rx51_smc91x_resources[0].start = cs_mem_base + 0x0;
	rx51_smc91x_resources[0].end   = cs_mem_base + 0xf;
	udelay(100);

	if (gpio_request(RX51_ETHR_GPIO_IRQ, "SMC91X irq") < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for smc91x IRQ\n",
			RX51_ETHR_GPIO_IRQ);
		return;
	}
	gpio_direction_input(RX51_ETHR_GPIO_IRQ);
}

static void __init rx51_init_tsc2005(void)
{
	int r;

	r = gpio_request(RX51_TSC2005_IRQ_GPIO, "tsc2005 DAV IRQ");
	if (r >= 0) {
		gpio_direction_input(RX51_TSC2005_IRQ_GPIO);
	} else {
		printk(KERN_ERR "unable to get DAV GPIO");
	}
}

static struct twl4030_usb_data rx51_usb_data = {
	.usb_mode		= T2_USB_MODE_ULPI,
};

static struct twl4030_madc_platform_data rx51_madc_data = {
	.irq_line		= 1,
};

static struct twl4030_gpio_platform_data rx51_gpio_data = {
	.gpio_base		= OMAP_MAX_GPIO_LINES,
	.irq_base		= TWL4030_GPIO_IRQ_BASE,
	.irq_end		= TWL4030_GPIO_IRQ_END,
};

static struct twl4030_platform_data rx51_twldata = {
	.irq_base		= TWL4030_IRQ_BASE,
	.irq_end		= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.gpio			= &rx51_gpio_data,
	.keypad			= &rx51_kp_data,
	.madc			= &rx51_madc_data,
	.usb			= &rx51_usb_data,
};

static struct i2c_board_info __initdata rx51_peripherals_i2c_board_info_1[] = {
	{
		I2C_BOARD_INFO("twl4030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &rx51_twldata,
	},
};

static int __init rx51_i2c_init(void)
{
	omap_register_i2c_bus(1, 2600, rx51_peripherals_i2c_board_info_1,
			ARRAY_SIZE(rx51_peripherals_i2c_board_info_1));
	omap_register_i2c_bus(2, 100, NULL, 0);
	omap_register_i2c_bus(3, 400, NULL, 0);
	return 0;
}


void __init rx51_peripherals_init(void)
{
	platform_add_devices(rx51_peripherals_devices,
				ARRAY_SIZE(rx51_peripherals_devices));
	spi_register_board_info(rx51_peripherals_spi_board_info,
				ARRAY_SIZE(rx51_peripherals_spi_board_info));
	rx51_i2c_init();
	rx51_init_smc91x();
	rx51_init_tsc2005();
}

