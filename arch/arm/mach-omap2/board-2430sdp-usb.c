/*
 * linux/arch/arm/mach-omap2/board-2430sdp-usb.c
 *
 * Copyright (C) 2007 MontaVista Software, Inc. <source@mvista.com>
 * Author: Kevin Hilman
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

#include <asm/arch/hardware.h>
#include <asm/arch/usb.h>

static struct resource musb_resources[] = {
	[0] = {
		.start	= OMAP243X_HS_BASE,
		.end	= OMAP243X_HS_BASE + SZ_8K,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {	/* general IRQ */
		.start	= INT_243X_HS_USB_MC,
		.flags	= IORESOURCE_IRQ,
	},
	[2] = {	/* DMA IRQ */
		.start	= INT_243X_HS_USB_DMA,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct musb_hdrc_platform_data musb_plat = {
#ifdef CONFIG_USB_MUSB_OTG
	.mode		= MUSB_OTG,
#elif CONFIG_USB_MUSB_HDRC_HCD
	.mode		= MUSB_HOST,
#elif CONFIG_USB_GADGET_MUSB_HDRC
	.mode		= MUSB_PERIPHERAL,
#endif
	.multipoint	= 1,
};

static u64 musb_dmamask = ~(u32)0;

static struct platform_device musb_device = {
	.name		= "musb_hdrc",
	.id		= 0,
	.dev = {
		.dma_mask		= &musb_dmamask,
		.coherent_dma_mask	= 0xffffffff,
		.platform_data		= &musb_plat,
	},
	.num_resources	= ARRAY_SIZE(musb_resources),
	.resource	= musb_resources,
};

void __init sdp2430_usb_init(void)
{
	if (platform_device_register(&musb_device) < 0) {
		printk(KERN_ERR "Unable to register HS-USB (MUSB) device\n");
		return;
	}
}

