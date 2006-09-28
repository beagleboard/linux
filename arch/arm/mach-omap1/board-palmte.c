/*
 * linux/arch/arm/mach-omap1/board-palmte.c
 *
 * Modified from board-generic.c
 *
 * Support for the Palm Tungsten E PDA.
 *
 * Original version : Laurent Gonzalez
 *
 * Maintainters : http://palmtelinux.sf.net
 *                palmtelinux-developpers@lists.sf.net
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>

#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>

#include <asm/arch/gpio.h>
#include <asm/arch/mux.h>
#include <asm/arch/usb.h>
#include <asm/arch/tc.h>
#include <asm/arch/dma.h>
#include <asm/arch/board.h>
#include <asm/arch/irda.h>
#include <asm/arch/keypad.h>
#include <asm/arch/common.h>

static void __init omap_palmte_init_irq(void)
{
	omap1_init_common_hw();
	omap_init_irq();
	omap_gpio_init();
}

static int palmte_keymap[] = {
	KEY(0, 0, KEY_F1),
	KEY(0, 1, KEY_F2),
	KEY(0, 2, KEY_F3),
	KEY(0, 3, KEY_F4),
	KEY(0, 4, KEY_POWER),
	KEY(1, 0, KEY_LEFT),
	KEY(1, 1, KEY_DOWN),
	KEY(1, 2, KEY_UP),
	KEY(1, 3, KEY_RIGHT),
	KEY(1, 4, KEY_CENTER),
	0,
};

static struct omap_kp_platform_data palmte_kp_data = {
	.rows	= 8,
	.cols	= 8,
	.keymap = palmte_keymap,
	.rep	= 1,
};

static struct resource palmte_kp_resources[] = {
	[0]	= {
		.start	= INT_KEYBOARD,
		.end	= INT_KEYBOARD,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device palmte_kp_device = {
	.name		= "omap-keypad",
	.id		= -1,
	.dev		= {
		.platform_data	= &palmte_kp_data,
	},
	.num_resources	= ARRAY_SIZE(palmte_kp_resources),
	.resource	= palmte_kp_resources,
};

static struct mtd_partition palmte_rom_partitions[] = {
	/* PalmOS "Small ROM", contains the bootloader and the debugger */
	{
		.name		= "smallrom",
		.offset		= 0,
		.size		= 0xa000,
		.mask_flags	= MTD_WRITEABLE,
	},
	/* PalmOS "Big ROM", a filesystem with all the OS code and data */
	{
		.name		= "bigrom",
		.offset		= SZ_128K,
		/*
		 * 0x5f0000 bytes big in the multi-language ("EFIGS") version,
		 * 0x7b0000 bytes in the English-only ("enUS") version.
		 */
		.size		= 0x7b0000,
		.mask_flags	= MTD_WRITEABLE,
	},
};

static struct flash_platform_data palmte_rom_data = {
	.map_name	= "map_rom",
	.width		= 2,
	.parts		= palmte_rom_partitions,
	.nr_parts	= ARRAY_SIZE(palmte_rom_partitions),
};

static struct resource palmte_rom_resource = {
	.start		= OMAP_CS0_PHYS,
	.end		= OMAP_CS0_PHYS + SZ_8M - 1,
	.flags		= IORESOURCE_MEM,
};

static struct platform_device palmte_rom_device = {
	.name		= "omapflash",
	.id		= -1,
	.dev		= {
		.platform_data	= &palmte_rom_data,
	},
	.num_resources	= 1,
	.resource	= &palmte_rom_resource,
};

static struct platform_device palmte_lcd_device = {
	.name		= "lcd_palmte",
	.id		= -1,
};

static struct omap_backlight_config palmte_backlight_config = {
	.default_intensity	= 0xa0,
};

static struct platform_device palmte_backlight_device = {
	.name		= "omap-bl",
	.id		= -1,
	.dev		= {
		.platform_data	= &palmte_backlight_config,
	},
};

static struct omap_irda_config palmte_irda_config = {
	.transceiver_cap	= IR_SIRMODE,
	.rx_channel		= OMAP_DMA_UART3_RX,
	.tx_channel		= OMAP_DMA_UART3_TX,
	.dest_start		= UART3_THR,
	.src_start		= UART3_RHR,
	.tx_trigger		= 0,
	.rx_trigger		= 0,
};

static struct resource palmte_irda_resources[] = {
	[0]	= {
		.start	= INT_UART3,
		.end	= INT_UART3,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device palmte_irda_device = {
	.name		= "omapirda",
	.id		= -1,
	.dev		= {
		.platform_data	= &palmte_irda_config,
	},
	.num_resources	= ARRAY_SIZE(palmte_irda_resources),
	.resource	= palmte_irda_resources,
};

static struct platform_device *devices[] __initdata = {
	&palmte_rom_device,
	&palmte_kp_device,
	&palmte_lcd_device,
	&palmte_backlight_device,
	&palmte_irda_device,
};

static struct omap_usb_config palmte_usb_config __initdata = {
	.register_dev	= 1,	/* Mini-B only receptacle */
	.hmc_mode	= 0,
	.pins[0]	= 2,
};

static struct omap_mmc_config palmte_mmc_config __initdata = {
	.mmc[0]		= {
		.enabled 	= 1,
		.wp_pin		= PALMTE_MMC_WP_GPIO,
		.power_pin	= PALMTE_MMC_POWER_GPIO,
		.switch_pin	= PALMTE_MMC_SWITCH_GPIO,
	},
};

static struct omap_lcd_config palmte_lcd_config __initdata = {
	.ctrl_name	= "internal",
};

static struct omap_uart_config palmte_uart_config __initdata = {
	.enabled_uarts = (1 << 0) | (1 << 1) | (0 << 2),
};

static struct omap_board_config_kernel palmte_config[] = {
	{ OMAP_TAG_USB,		&palmte_usb_config },
	{ OMAP_TAG_MMC,		&palmte_mmc_config },
	{ OMAP_TAG_LCD,		&palmte_lcd_config },
	{ OMAP_TAG_UART,	&palmte_uart_config },
};

/* Periodically check for changes on important input pins */
struct timer_list palmte_pin_timer;
int prev_power, prev_headphones;

static void palmte_pin_handler(unsigned long data) {
	int power, headphones;

	power = !omap_get_gpio_datain(PALMTE_DC_GPIO);
	headphones = omap_get_gpio_datain(PALMTE_HEADPHONES_GPIO);

	if (power && !prev_power)
		printk(KERN_INFO "PM: cable connected\n");
	else if (!power && prev_power)
		printk(KERN_INFO "PM: cable disconnected\n");

	if (headphones && !prev_headphones) {
		/* Headphones connected, disable speaker */
		omap_set_gpio_dataout(PALMTE_SPEAKER_GPIO, 0);
		printk(KERN_INFO "PM: speaker off\n");
	} else if (!headphones && prev_headphones) {
		/* Headphones unplugged, re-enable speaker */
		omap_set_gpio_dataout(PALMTE_SPEAKER_GPIO, 1);
		printk(KERN_INFO "PM: speaker on\n");
	}

	prev_power = power;
	prev_headphones = headphones;
	mod_timer(&palmte_pin_timer, jiffies + msecs_to_jiffies(500));
}

static void __init palmte_gpio_setup(void)
{
	/* Set TSC2102 PINTDAV pin as input */
	if (omap_request_gpio(PALMTE_PINTDAV_GPIO)) {
		printk(KERN_ERR "Could not reserve PINTDAV GPIO!\n");
		return;
	}
	omap_set_gpio_direction(PALMTE_PINTDAV_GPIO, 1);

	/* Monitor cable-connected signals */
	if (omap_request_gpio(PALMTE_DC_GPIO) ||
			omap_request_gpio(PALMTE_USB_OR_DC_GPIO) ||
			omap_request_gpio(PALMTE_USBDETECT_GPIO)) {
		printk(KERN_ERR "Could not reserve cable signal GPIO!\n");
		return;
	}
	omap_set_gpio_direction(PALMTE_DC_GPIO, 1);
	omap_set_gpio_direction(PALMTE_USB_OR_DC_GPIO, 1);
	omap_set_gpio_direction(PALMTE_USBDETECT_GPIO, 1);

	/* Set speaker-enable pin as output */
	if (omap_request_gpio(PALMTE_SPEAKER_GPIO)) {
		printk(KERN_ERR "Could not reserve speaker GPIO!\n");
		return;
	}
	omap_set_gpio_direction(PALMTE_SPEAKER_GPIO, 0);

	/* Monitor the headphones-connected signal */
	if (omap_request_gpio(PALMTE_HEADPHONES_GPIO)) {
		printk(KERN_ERR "Could not reserve headphones signal GPIO!\n");
		return;
	}
	omap_set_gpio_direction(PALMTE_HEADPHONES_GPIO, 1);

	prev_power = omap_get_gpio_datain(PALMTE_DC_GPIO);
	prev_headphones = !omap_get_gpio_datain(PALMTE_HEADPHONES_GPIO);
	setup_timer(&palmte_pin_timer, palmte_pin_handler, 0);
	palmte_pin_handler(0);
}

static void __init omap_palmte_init(void)
{
	omap_board_config = palmte_config;
	omap_board_config_size = ARRAY_SIZE(palmte_config);

	platform_add_devices(devices, ARRAY_SIZE(devices));

	omap_serial_init();
	palmte_gpio_setup();
}

static void __init omap_palmte_map_io(void)
{
	omap1_map_common_io();
}

MACHINE_START(OMAP_PALMTE, "OMAP310 based Palm Tungsten E")
	.phys_io	= 0xfff00000,
	.io_pg_offst	= ((0xfef00000) >> 18) & 0xfffc,
	.boot_params	= 0x10000100,
	.map_io		= omap_palmte_map_io,
	.init_irq	= omap_palmte_init_irq,
	.init_machine	= omap_palmte_init,
	.timer		= &omap_timer,
MACHINE_END
