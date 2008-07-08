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
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/i2c/twl4030.h>
#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>

#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <asm/arch/mcspi.h>
#include <asm/arch/gpio.h>
#include <asm/arch/mux.h>
#include <asm/arch/board.h>
#include <asm/arch/usb-musb.h>
#include <asm/arch/usb-ehci.h>
#include <asm/arch/hsmmc.h>
#include <asm/arch/common.h>
#include <asm/arch/keypad.h>
#include <asm/arch/dma.h>
#include <asm/arch/gpmc.h>

#include <asm/io.h>
#include <asm/delay.h>

#define	SDP3430_SMC91X_CS	3

#define ENABLE_VAUX3_DEDICATED	0x03
#define ENABLE_VAUX3_DEV_GRP	0x20


static struct resource sdp3430_smc91x_resources[] = {
	[0] = {
		.start	= OMAP34XX_ETHR_START,
		.end	= OMAP34XX_ETHR_START + SZ_4K,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= 0,
		.end	= 0,
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_LOWLEVEL,
	},
};

static struct platform_device sdp3430_smc91x_device = {
	.name		= "smc91x",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(sdp3430_smc91x_resources),
	.resource	= sdp3430_smc91x_resources,
};

static int sdp3430_keymap[] = {
	KEY(0, 0, KEY_LEFT),
	KEY(0, 1, KEY_RIGHT),
	KEY(0, 2, KEY_A),
	KEY(0, 3, KEY_B),
	KEY(0, 4, KEY_C),
	KEY(1, 0, KEY_DOWN),
	KEY(1, 1, KEY_UP),
	KEY(1, 2, KEY_E),
	KEY(1, 3, KEY_F),
	KEY(1, 4, KEY_G),
	KEY(2, 0, KEY_ENTER),
	KEY(2, 1, KEY_I),
	KEY(2, 2, KEY_J),
	KEY(2, 3, KEY_K),
	KEY(2, 4, KEY_3),
	KEY(3, 0, KEY_M),
	KEY(3, 1, KEY_N),
	KEY(3, 2, KEY_O),
	KEY(3, 3, KEY_P),
	KEY(3, 4, KEY_Q),
	KEY(4, 0, KEY_R),
	KEY(4, 1, KEY_4),
	KEY(4, 2, KEY_T),
	KEY(4, 3, KEY_U),
	KEY(4, 4, KEY_D),
	KEY(5, 0, KEY_V),
	KEY(5, 1, KEY_W),
	KEY(5, 2, KEY_L),
	KEY(5, 3, KEY_S),
	KEY(5, 4, KEY_H),
	0
};

static struct omap_kp_platform_data sdp3430_kp_data = {
	.rows		= 5,
	.cols		= 6,
	.keymap 	= sdp3430_keymap,
	.keymapsize 	= ARRAY_SIZE(sdp3430_keymap),
	.rep		= 1,
};

static struct platform_device sdp3430_kp_device = {
	.name		= "omap_twl4030keypad",
	.id		= -1,
	.dev		= {
		.platform_data	= &sdp3430_kp_data,
	},
};

static int ts_gpio;

/**
 * @brief ads7846_dev_init : Requests & sets GPIO line for pen-irq
 *
 * @return - void. If request gpio fails then Flag KERN_ERR.
 */
static void ads7846_dev_init(void)
{
	if (omap_request_gpio(ts_gpio) < 0) {
		printk(KERN_ERR "can't get ads746 pen down GPIO\n");
		return;
	}

	omap_set_gpio_direction(ts_gpio, 1);

	omap_set_gpio_debounce(ts_gpio, 1);
	omap_set_gpio_debounce_time(ts_gpio, 0xa);
}

static int ads7846_get_pendown_state(void)
{
	return !omap_get_gpio_datain(ts_gpio);
}

/*
 * This enable(1)/disable(0) the voltage for TS: uses twl4030 calls
 */
static int ads7846_vaux_control(int vaux_cntrl)
{
	int ret = 0;

#ifdef CONFIG_TWL4030_CORE
	/* check for return value of ldo_use: if success it returns 0 */
	if (vaux_cntrl == VAUX_ENABLE) {
		if (ret != twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VAUX3_DEDICATED, TWL4030_VAUX3_DEDICATED))
			return -EIO;
		if (ret != twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VAUX3_DEV_GRP, TWL4030_VAUX3_DEV_GRP))
			return -EIO;
	} else if (vaux_cntrl == VAUX_DISABLE) {
		if (ret != twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			0x00, TWL4030_VAUX3_DEDICATED))
			return -EIO;
		if (ret != twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			0x00, TWL4030_VAUX3_DEV_GRP))
			return -EIO;
	}
#else
	ret = -EIO;
#endif
	return ret;
}

static struct ads7846_platform_data tsc2046_config __initdata = {
	.get_pendown_state	= ads7846_get_pendown_state,
	.keep_vref_on		= 1,
	.vaux_control		= ads7846_vaux_control,
};


static struct omap2_mcspi_device_config tsc2046_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,  /* 0: slave, 1: master */
};

static struct spi_board_info sdp3430_spi_board_info[] __initdata = {
	[0] = {
		/*
		 * TSC2046 operates at a max freqency of 2MHz, so
		 * operate slightly below at 1.5MHz
		 */
		.modalias		= "ads7846",
		.bus_num		= 1,
		.chip_select		= 0,
		.max_speed_hz		= 1500000,
		.controller_data	= &tsc2046_mcspi_config,
		.irq			= 0,
		.platform_data		= &tsc2046_config,
	},
};

static struct platform_device sdp3430_lcd_device = {
	.name		= "sdp2430_lcd",
	.id		= -1,
};

static struct platform_device *sdp3430_devices[] __initdata = {
	&sdp3430_smc91x_device,
	&sdp3430_kp_device,
	&sdp3430_lcd_device,
};

static inline void __init sdp3430_init_smc91x(void)
{
	int eth_cs;
	unsigned long cs_mem_base;
	int eth_gpio = 0;

	eth_cs = SDP3430_SMC91X_CS;

	if (gpmc_cs_request(eth_cs, SZ_16M, &cs_mem_base) < 0) {
		printk(KERN_ERR "Failed to request GPMC mem for smc91x\n");
		return;
	}

	sdp3430_smc91x_resources[0].start = cs_mem_base + 0x0;
	sdp3430_smc91x_resources[0].end   = cs_mem_base + 0xf;
	udelay(100);

	if (is_sil_rev_greater_than(OMAP3430_REV_ES1_0))
		eth_gpio = OMAP34XX_ETHR_GPIO_IRQ_SDPV2;
	else
		eth_gpio = OMAP34XX_ETHR_GPIO_IRQ_SDPV1;

	sdp3430_smc91x_resources[1].start = OMAP_GPIO_IRQ(eth_gpio);

	if (omap_request_gpio(eth_gpio) < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for smc91x IRQ\n",
			eth_gpio);
		return;
	}
	omap_set_gpio_direction(eth_gpio, 1);
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

static struct omap_lcd_config sdp3430_lcd_config __initdata = {
	.ctrl_name	= "internal",
};

static struct omap_mmc_config sdp3430_mmc_config __initdata = {
	.mmc [0] = {
		.enabled	= 1,
		.wire4		= 1,
	},
};

static struct omap_board_config_kernel sdp3430_config[] __initdata = {
	{ OMAP_TAG_UART,	&sdp3430_uart_config },
	{OMAP_TAG_LCD,		&sdp3430_lcd_config},
	{OMAP_TAG_MMC,		&sdp3430_mmc_config },
};

static int __init omap3430_i2c_init(void)
{
	omap_register_i2c_bus(1, 2600, NULL, 0);
	omap_register_i2c_bus(2, 400, NULL, 0);
	omap_register_i2c_bus(3, 400, NULL, 0);
	return 0;
}

extern void __init sdp3430_flash_init(void);

static void __init omap_3430sdp_init(void)
{
	platform_add_devices(sdp3430_devices, ARRAY_SIZE(sdp3430_devices));
	omap_board_config = sdp3430_config;
	omap_board_config_size = ARRAY_SIZE(sdp3430_config);
	if (is_sil_rev_greater_than(OMAP3430_REV_ES1_0))
		ts_gpio = OMAP34XX_TS_GPIO_IRQ_SDPV2;
	else
		ts_gpio = OMAP34XX_TS_GPIO_IRQ_SDPV1;
	sdp3430_spi_board_info[0].irq = OMAP_GPIO_IRQ(ts_gpio);
	spi_register_board_info(sdp3430_spi_board_info,
				ARRAY_SIZE(sdp3430_spi_board_info));
	ads7846_dev_init();
	sdp3430_flash_init();
	omap_serial_init();
	usb_musb_init();
	usb_ehci_init();
	hsmmc_init();
}

static void __init omap_3430sdp_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}
arch_initcall(omap3430_i2c_init);

MACHINE_START(OMAP_3430SDP, "OMAP3430 3430SDP board")
	/* Maintainer: Syed Khasim - Texas Instruments Inc */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap_3430sdp_map_io,
	.init_irq	= omap_3430sdp_init_irq,
	.init_machine	= omap_3430sdp_init,
	.timer		= &omap_timer,
MACHINE_END
