/*
 * linux/arch/arm/mach-omap2/board-2430sdp.c
 *
 * Copyright (C) 2006 Texas Instruments
 *
 * Modified from mach-omap2/board-generic.c
 *
 * Initial Code : Based on a patch from Komal Shah and Richard Woodruff
 * Updated the Code for 2430 SDP : Syed Mohammed Khasim
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
#include <linux/input.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/spi/spi.h>
#include <linux/spi/tsc2046.h>

#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>

#include <asm/arch/gpio.h>
#include <asm/arch/mux.h>
#include <asm/arch/board.h>
#include <asm/arch/common.h>
#include <asm/arch/keypad.h>
#include <asm/arch/gpmc.h>
#include <asm/arch/mcspi.h>
#include "prcm-regs.h"

#include <asm/io.h>

#define	SDP2430_FLASH_CS	0
#define	SDP2430_SMC91X_CS	5

/* TSC2046 (touchscreen) */
#define TS_GPIO                 24

static struct mtd_partition sdp2430_partitions[] = {
	/* bootloader (U-Boot, etc) in first sector */
	{
		.name		= "bootloader",
		.offset		= 0,
		.size		= SZ_256K,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
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

static struct flash_platform_data sdp2430_flash_data = {
	.map_name	= "cfi_probe",
	.width		= 2,
	.parts		= sdp2430_partitions,
	.nr_parts	= ARRAY_SIZE(sdp2430_partitions),
};

static struct resource sdp2430_flash_resource = {
	.start		= SDP2430_CS0_BASE,
	.end		= SDP2430_CS0_BASE + SZ_64M - 1,
	.flags		= IORESOURCE_MEM,
};

static struct platform_device sdp2430_flash_device = {
	.name		= "omapflash",
	.id		= 0,
	.dev = {
		.platform_data	= &sdp2430_flash_data,
	},
	.num_resources	= 1,
	.resource	= &sdp2430_flash_resource,
};

static struct resource sdp2430_smc91x_resources[] = {
	[0] = {
		.start	= SDP2430_CS0_BASE,
		.end	= SDP2430_CS0_BASE + SZ_64M - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= OMAP_GPIO_IRQ(OMAP24XX_ETHR_GPIO_IRQ),
		.end	= OMAP_GPIO_IRQ(OMAP24XX_ETHR_GPIO_IRQ),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device sdp2430_smc91x_device = {
	.name		= "smc91x",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(sdp2430_smc91x_resources),
	.resource	= sdp2430_smc91x_resources,
};

/*
 * Key mapping for 2430 SDP board
 */

static int sdp2430_keymap[] = {
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

static struct omap_kp_platform_data sdp2430_kp_data = {
	.rows		= 5,
	.cols		= 6,
	.keymap 	= sdp2430_keymap,
	.keymapsize 	= ARRAY_SIZE(sdp2430_keymap),
	.rep		= 1,
};

static struct platform_device sdp2430_kp_device = {
	.name		= "omap_twl4030keypad",
	.id		= -1,
	.dev		= {
		.platform_data	= &sdp2430_kp_data,
	},
};

static struct platform_device *sdp2430_devices[] __initdata = {
	&sdp2430_smc91x_device,
	&sdp2430_flash_device,
	&sdp2430_kp_device,
};

static struct tsc2046_platform_data tsc2046_config = {
	.dav_gpio       = TS_GPIO,
	.gpio_debounce  = 0xa,
};

static struct omap2_mcspi_device_config tsc2046_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel = 0,  /* 0: slave, 1: master */
};

static struct spi_board_info sdp2430_spi_board_info[] __initdata = {
	[0] = {
		/*
		 * TSC2046 operates at a max freqency of 2MHz, so
		 * operate slightly below at 1.5MHz
		 */
		.modalias	 = "tsc2046",
		.bus_num	 = 1,
		.chip_select	 = 0,
		.max_speed_hz    = 1500000,
		.controller_data = &tsc2046_mcspi_config,
		.platform_data   = &tsc2046_config,
	},
};

static inline void __init sdp2430_init_smc91x(void)
{
	int eth_cs;
	unsigned long cs_mem_base;
	unsigned int rate;
	struct clk *l3ck;

	eth_cs = SDP2430_SMC91X_CS;

	l3ck = clk_get(NULL, "core_l3_ck");
	if (IS_ERR(l3ck))
		rate = 100000000;
	else
		rate = clk_get_rate(l3ck);

	/* Make sure CS1 timings are correct, for 2430 always muxed */
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
	} else { /* rate = 100000000 */
		gpmc_cs_write_reg(eth_cs, GPMC_CS_CONFIG2, 0x001f1f00);
		gpmc_cs_write_reg(eth_cs, GPMC_CS_CONFIG3, 0x00080802);
		gpmc_cs_write_reg(eth_cs, GPMC_CS_CONFIG4, 0x1C091C09);
		gpmc_cs_write_reg(eth_cs, GPMC_CS_CONFIG5, 0x031A1F1F);
		gpmc_cs_write_reg(eth_cs, GPMC_CS_CONFIG6, 0x000003C2);
	}

	if (gpmc_cs_request(eth_cs, SZ_16M, &cs_mem_base) < 0) {
		printk(KERN_ERR "Failed to request GPMC mem for smc91x\n");
		return;
	}

	sdp2430_smc91x_resources[0].start = cs_mem_base + 0x300;
	sdp2430_smc91x_resources[0].end = cs_mem_base + 0x30f;
	udelay(100);

	if (omap_request_gpio(OMAP24XX_ETHR_GPIO_IRQ) < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for smc91x IRQ\n",
			OMAP24XX_ETHR_GPIO_IRQ);
		gpmc_cs_free(eth_cs);
		return;
	}
	omap_set_gpio_direction(OMAP24XX_ETHR_GPIO_IRQ, 1);

}

static void __init omap_2430sdp_init_irq(void)
{
	omap2_init_common_hw();
	omap_init_irq();
	omap_gpio_init();
	sdp2430_init_smc91x();
}

static struct omap_uart_config sdp2430_uart_config __initdata = {
	.enabled_uarts = ((1 << 0) | (1 << 1) | (1 << 2)),
};

static struct omap_board_config_kernel sdp2430_config[] __initdata = {
	{OMAP_TAG_UART, &sdp2430_uart_config},
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

extern void __init sdp2430_flash_init(void);
extern void __init sdp2430_usb_init(void);

static void __init omap_2430sdp_init(void)
{
	platform_add_devices(sdp2430_devices, ARRAY_SIZE(sdp2430_devices));
	omap_board_config = sdp2430_config;
	omap_board_config_size = ARRAY_SIZE(sdp2430_config);
	omap_serial_init();

	sdp2430_flash_init();
	sdp2430_usb_init();

	spi_register_board_info(sdp2430_spi_board_info,
				ARRAY_SIZE(sdp2430_spi_board_info));
}

static void __init omap_2430sdp_map_io(void)
{
	omap2_map_common_io();
}

arch_initcall(omap2430_i2c_init);

MACHINE_START(OMAP_2430SDP, "OMAP2430 sdp2430 board")
	/* Maintainer: Syed Khasim - Texas Instruments Inc */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap_2430sdp_map_io,
	.init_irq	= omap_2430sdp_init_irq,
	.init_machine	= omap_2430sdp_init,
	.timer		= &omap_timer,
MACHINE_END
