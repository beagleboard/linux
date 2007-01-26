/*
 * linux/arch/arm/mach-omap2/board-n800-usb.c
 *
 * Copyright (C) 2006 Nokia Corporation
 * Author: Juha Yrjola
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/usb/musb.h>
#include <asm/arch/gpmc.h>
#include <asm/arch/gpio.h>

#define TUSB_ASYNC_CS		1
#define TUSB_SYNC_CS		4
#define GPIO_TUSB_INT		58
#define GPIO_TUSB_ENABLE	0

static int tusb_set_power(int state);

#if	defined(CONFIG_USB_MUSB_OTG)
#	define BOARD_MODE	MUSB_OTG
#elif	defined(CONFIG_USB_MUSB_PERIPHERAL)
#	define BOARD_MODE	MUSB_PERIPHERAL
#else	/* defined(CONFIG_USB_MUSB_HOST) */
#	define BOARD_MODE	MUSB_HOST
#endif

static struct musb_hdrc_platform_data tusb_data = {
	.mode		= BOARD_MODE,
	.multipoint	= 1,
	.set_power	= tusb_set_power,
	.min_power	= 25,	/* x2 = 50 mA drawn from VBUS as peripheral */
};

/*
 * Enable or disable power to TUSB6010. When enabling, turn on 3.3 V and
 * 1.5 V voltage regulators of PM companion chip. Companion chip will then
 * provide then PGOOD signal to TUSB6010 which will release it from reset.
 */
static int tusb_set_power(int state)
{
	int i, retval = 0;

	if (state) {
		omap_set_gpio_dataout(GPIO_TUSB_ENABLE, 1);
		msleep(1);

		/* Wait until TUSB6010 pulls INT pin down */
		i = 100;
		while (i && omap_get_gpio_datain(GPIO_TUSB_INT)) {
			msleep(1);
			i--;
		}

		if (!i) {
			printk(KERN_ERR "tusb: powerup failed\n");
			retval = -ENODEV;
		}
	} else {
		omap_set_gpio_dataout(GPIO_TUSB_ENABLE, 0);
		msleep(10);
	}

	return retval;
}

void __init n800_usb_init(void)
{
	int ret = 0;
	static char	announce[] __initdata = KERN_INFO "TUSB 6010\n";

	/* PM companion chip power control pin */
	ret = omap_request_gpio(GPIO_TUSB_ENABLE);
	if (ret != 0) {
		printk(KERN_ERR "Could not get TUSB power GPIO%i\n",
		       GPIO_TUSB_ENABLE);
		return;
	}
	omap_set_gpio_direction(GPIO_TUSB_ENABLE, 0);

	tusb_set_power(0);

	ret = tusb6010_setup_interface(&tusb_data, TUSB6010_REFCLK_19, 2,
					TUSB_ASYNC_CS, TUSB_SYNC_CS,
					GPIO_TUSB_INT, 0x3f);
	if (ret != 0)
		goto err;

	printk(announce);

	return;

err:
	omap_free_gpio(GPIO_TUSB_ENABLE);
}
