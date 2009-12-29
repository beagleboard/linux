/*
 * linux/arch/arm/mach-omap2/board-n8x0.c
 *
 * Copyright (C) 2005-2009 Nokia Corporation
 * Author: Juha Yrjola <juha.yrjola@nokia.com>
 *
 * Modified from mach-omap2/board-generic.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/stddef.h>
#include <linux/platform_device.h>
#include <linux/omapfb.h>
#include <linux/spi/spi.h>
#include <linux/usb/musb.h>

#include <asm/mach/arch.h>
#include <asm/mach-types.h>

#include <plat/board.h>
#include <plat/common.h>
#include <mach/irqs.h>
#include <plat/mcspi.h>
#include <plat/onenand.h>
#include <plat/serial.h>
#include <plat/cbus.h>
#include <plat/lcd_mipid.h>
#include <plat/blizzard.h>

#include <../drivers/cbus/tahvo.h>

static struct omap2_mcspi_device_config mipid_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,
};

/* REVISIT: Why both this and the omap_lcd_config below? */
static struct mipid_platform_data n8x0_mipid_platform_data = {
	.nreset_gpio	= 30,
	.data_lines	= 24,
};

static struct omap2_mcspi_device_config p54spi_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel = 1,
};

static struct spi_board_info n800_spi_board_info[] __initdata = {
	{
		.modalias	= "lcd_mipid",
		.bus_num	= 1,
		.chip_select	= 1,
		.max_speed_hz	= 4000000,
		.controller_data= &mipid_mcspi_config,
		.platform_data	= &n8x0_mipid_platform_data,
	},
	{
		.modalias	= "p54spi",
		.bus_num	= 2,
		.chip_select	= 0,
		.max_speed_hz   = 48000000,
		.controller_data = &p54spi_mcspi_config,
	},
};

#if defined(CONFIG_FB_OMAP) || defined(CONFIG_FB_OMAP_MODULE)

#define N8X0_BLIZZARD_POWERDOWN_GPIO	15

/* REVISIT: Get rid of the these *_config entries, use platform_data */
/* Seems to be only used for ctrl_name */
static struct omap_lcd_config n8x0_lcd_config __initdata = {
	//.nreset_gpio	= 30,		// REVISIT: Needed?
	//.data_lines	= 24,		// REVISIT: Needed?
	.ctrl_name	= "blizzard",
	//.panel_name	= "ls041y3",	// REVISIT: Needed?
};

static struct omap_fbmem_config n8x0_fbmem0_config __initdata = {
	.size = 752 * 1024,
};

static struct omap_fbmem_config n8x0_fbmem1_config __initdata = {
	.size = 752 * 1024,
};

static struct omap_fbmem_config n8x0_fbmem2_config __initdata = {
	.size = 752 * 1024,
};

static void mipid_shutdown(struct mipid_platform_data *pdata)
{
	if (pdata->nreset_gpio != -1) {
		pr_info("shutdown LCD\n");
		gpio_set_value(pdata->nreset_gpio, 0);
		msleep(120);
	}
}

static void __init mipid_dev_init(void)
{
	n8x0_mipid_platform_data.shutdown = mipid_shutdown;
}

static struct {
	struct clk *sys_ck;
} blizzard;

static int blizzard_get_clocks(void)
{
	blizzard.sys_ck = clk_get(0, "osc_ck");
	if (IS_ERR(blizzard.sys_ck)) {
		printk(KERN_ERR "can't get Blizzard clock\n");
		return PTR_ERR(blizzard.sys_ck);
	}
	return 0;
}

static unsigned long blizzard_get_clock_rate(struct device *dev)
{
	return clk_get_rate(blizzard.sys_ck);
}

static void blizzard_enable_clocks(int enable)
{
	if (enable)
		clk_enable(blizzard.sys_ck);
	else
		clk_disable(blizzard.sys_ck);
}

static void blizzard_power_up(struct device *dev)
{
	/* Vcore to 1.475V */
	tahvo_set_clear_reg_bits(0x07, 0, 0xf);
	msleep(10);

	blizzard_enable_clocks(1);
	gpio_set_value(N8X0_BLIZZARD_POWERDOWN_GPIO, 1);
}

static void blizzard_power_down(struct device *dev)
{
	gpio_set_value(N8X0_BLIZZARD_POWERDOWN_GPIO, 0);
	blizzard_enable_clocks(0);

	/* Vcore to 1.005V */
	tahvo_set_clear_reg_bits(0x07, 0xf, 0);
}

static struct blizzard_platform_data n8x0_blizzard_data = {
	.power_up	= blizzard_power_up,
	.power_down	= blizzard_power_down,
	.get_clock_rate	= blizzard_get_clock_rate,
	.te_connected	= 1,
};

static void __init blizzard_dev_init(void)
{
	int r;

	r = gpio_request(N8X0_BLIZZARD_POWERDOWN_GPIO, "Blizzard pd");
	if (r < 0)
		return;
	gpio_direction_output(N8X0_BLIZZARD_POWERDOWN_GPIO, 1);

	blizzard_get_clocks();
	omapfb_set_ctrl_platform_data(&n8x0_blizzard_data);
}

static struct omap_board_config_kernel n8x0_config[] __initdata = {
	{ OMAP_TAG_LCD,				&n8x0_lcd_config },
	{ OMAP_TAG_FBMEM,			&n8x0_fbmem0_config },
	{ OMAP_TAG_FBMEM,			&n8x0_fbmem1_config },
	{ OMAP_TAG_FBMEM,			&n8x0_fbmem2_config },
};

#else

static inline void mipid_dev_init()
{
}
static inline void blizzard_dev_init()
{
}

#endif

#if defined(CONFIG_MTD_ONENAND_OMAP2) || \
	defined(CONFIG_MTD_ONENAND_OMAP2_MODULE)

static struct mtd_partition onenand_partitions[] = {
	{
		.name           = "bootloader",
		.offset         = 0,
		.size           = 0x20000,
		.mask_flags     = MTD_WRITEABLE,	/* Force read-only */
	},
	{
		.name           = "config",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 0x60000,
	},
	{
		.name           = "kernel",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 0x200000,
	},
	{
		.name           = "initfs",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 0x400000,
	},
	{
		.name           = "rootfs",
		.offset         = MTDPART_OFS_APPEND,
		.size           = MTDPART_SIZ_FULL,
	},
};

static struct cbus_host_platform_data n8x0_cbus_data = {
	.clk_gpio	= 66,
	.dat_gpio	= 65,
	.sel_gpio	= 64,
};

static struct platform_device n8x0_cbus_device = {
	.name		= "cbus",
	.id		= -1,
	.dev		= {
		.platform_data = &n8x0_cbus_data,
	},
};

static struct omap_onenand_platform_data board_onenand_data = {
	.cs		= 0,
	.gpio_irq	= 26,
	.parts		= onenand_partitions,
	.nr_parts	= ARRAY_SIZE(onenand_partitions),
	.flags		= ONENAND_SYNC_READ,
};

static void __init n8x0_onenand_init(void)
{
	gpmc_onenand_init(&board_onenand_data);
}

#else

static void __init n8x0_onenand_init(void) {}

#endif

static void __init n8x0_map_io(void)
{

#if defined(CONFIG_FB_OMAP) || defined(CONFIG_FB_OMAP_MODULE)
	/* REVISIT: Get rid of these */
	omap_board_config = n8x0_config;
	omap_board_config_size = ARRAY_SIZE(n8x0_config);
#endif

	omap2_set_globals_242x();
	omap2_map_common_io();
}

static void __init n8x0_init_irq(void)
{
	omap2_init_common_hw(NULL, NULL);
	omap_init_irq();
	omap_gpio_init();
}

#ifdef CONFIG_MACH_NOKIA_N8X0_USB
extern void n8x0_usb_init(void);
#else
static inline void n8x0_usb_init (void) {}
#endif

static void __init n8x0_init_machine(void)
{
	n8x0_usb_init();

	platform_device_register(&n8x0_cbus_device);

	/* FIXME: add n810 spi devices */
	spi_register_board_info(n800_spi_board_info,
				ARRAY_SIZE(n800_spi_board_info));

	omap_serial_init();
	mipid_dev_init();
	blizzard_dev_init();
	n8x0_onenand_init();
}

MACHINE_START(NOKIA_N800, "Nokia N800")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= n8x0_map_io,
	.init_irq	= n8x0_init_irq,
	.init_machine	= n8x0_init_machine,
	.timer		= &omap_timer,
MACHINE_END

MACHINE_START(NOKIA_N810, "Nokia N810")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= n8x0_map_io,
	.init_irq	= n8x0_init_irq,
	.init_machine	= n8x0_init_machine,
	.timer		= &omap_timer,
MACHINE_END

MACHINE_START(NOKIA_N810_WIMAX, "Nokia N810 WiMAX")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= n8x0_map_io,
	.init_irq	= n8x0_init_irq,
	.init_machine	= n8x0_init_machine,
	.timer		= &omap_timer,
MACHINE_END
