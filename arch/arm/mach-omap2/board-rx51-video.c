/*
 * linux/arch/arm/mach-omap2/board-rx51-video.c
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
#include <linux/delay.h>
#include <linux/spi/spi.h>

#include <mach/lcd_mipid.h>
#include <mach/mcspi.h>
#include <mach/gpio.h>
#include <mach/board.h>


static struct omap2_mcspi_device_config mipid_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,
};

static struct platform_device rx51_lcd_device = {
	.name		= "lcd_mipid",
	.id		= -1,
};

static void mipid_shutdown(struct mipid_platform_data *pdata)
{
	if (pdata->nreset_gpio != -1) {
		pr_info("shutdown LCD\n");
		gpio_direction_output(pdata->nreset_gpio, 0);
		msleep(120);
	}
}

static struct mipid_platform_data rx51_mipid_platform_data = {
	.shutdown = mipid_shutdown,
};

static void __init mipid_dev_init(void)
{
	const struct omap_lcd_config *conf;

	conf = omap_get_config(OMAP_TAG_LCD, struct omap_lcd_config);
	if (conf != NULL) {
		rx51_mipid_platform_data.nreset_gpio = conf->nreset_gpio;
		rx51_mipid_platform_data.data_lines = conf->data_lines;
	}
}

static struct spi_board_info rx51_video_spi_board_info[] = {
	[0] = {
		.modalias		= "lcd_mipid",
		.bus_num		= 1,
		.chip_select		= 2,
		.max_speed_hz		= 6000000,
		.controller_data	= &mipid_mcspi_config,
		.platform_data		= &rx51_mipid_platform_data,
	},
};

static struct platform_device *rx51_video_devices[] = {
	&rx51_lcd_device,
};

void __init rx51_video_init(void)
{
	platform_add_devices(rx51_video_devices, ARRAY_SIZE(rx51_video_devices));
	spi_register_board_info(rx51_video_spi_board_info,
			ARRAY_SIZE(rx51_video_spi_board_info));
	mipid_dev_init();
}

