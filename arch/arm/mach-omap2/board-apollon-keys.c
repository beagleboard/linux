/*
 * linux/arch/arm/mach-omap2/board-apollon-keys.c
 *
 * Copyright (C) 2007 Samsung Electronics
 * Author: Kyungmin Park <kyungmin.park@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>

#include <mach/gpio.h>
#include <mach/mux.h>

#define SW_ENTER_GPIO16		16
#define SW_UP_GPIO17		17
#define SW_DOWN_GPIO58		58
#define SW_LEFT_GPIO95		95
#define SW_RIGHT_GPIO96		96
#define SW_ESC_GPIO97		97

static struct gpio_keys_button apollon_gpio_keys_buttons[] = {
	[0] = {
		.code		= KEY_ENTER,
		.gpio		= SW_ENTER_GPIO16,
		.desc		= "enter sw",
	},
	[1] = {
		.code		= KEY_UP,
		.gpio		= SW_UP_GPIO17,
		.desc		= "up sw",
	},
	[2] = {
		.code		= KEY_DOWN,
		.gpio		= SW_DOWN_GPIO58,
		.desc		= "down sw",
	},
	[3] = {
		.code		= KEY_LEFT,
		.gpio		= SW_LEFT_GPIO95,
		.desc		= "left sw",
	},
	[4] = {
		.code		= KEY_RIGHT,
		.gpio		= SW_RIGHT_GPIO96,
		.desc		= "right sw",
	},
	[5] = {
		.code		= KEY_ESC,
		.gpio		= SW_ESC_GPIO97,
		.desc		= "esc sw",
	},
};

static struct gpio_keys_platform_data apollon_gpio_keys = {
	.buttons		= apollon_gpio_keys_buttons,
	.nbuttons		= ARRAY_SIZE(apollon_gpio_keys_buttons),
};

static struct platform_device apollon_gpio_keys_device = {
	.name			= "gpio-keys",
	.id			= -1,
	.dev			= {
		.platform_data	= &apollon_gpio_keys,
	},
};

static void __init apollon_sw_init(void)
{
	/* Enter SW - Y11 */
	omap_cfg_reg(Y11_242X_GPIO16);
	/* Up SW - AA12 */
	omap_cfg_reg(AA12_242X_GPIO17);
	/* Down SW - AA8 */
	omap_cfg_reg(AA8_242X_GPIO58);

	if (apollon_plus()) {
		/* Left SW - P18 */
		omap_cfg_reg(P18_24XX_GPIO95);
		/* Right SW - M18 */
		omap_cfg_reg(M18_24XX_GPIO96);
		/* Esc SW - L14 */
		omap_cfg_reg(L14_24XX_GPIO97);
	} else
		apollon_gpio_keys.nbuttons = 3;
}

static int __init omap_apollon_keys_init(void)
{
	apollon_sw_init();

	return platform_device_register(&apollon_gpio_keys_device);
}

arch_initcall(omap_apollon_keys_init);
