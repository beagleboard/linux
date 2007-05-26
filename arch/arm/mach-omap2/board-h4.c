/*
 * linux/arch/arm/mach-omap2/board-h4.c
 *
 * Copyright (C) 2005 Nokia Corporation
 * Author: Paul Mundt <paul.mundt@nokia.com>
 *
 * Modified from mach-omap/omap1/board-generic.c
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
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/i2c.h>

#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>

#include <asm/arch/gpio.h>
#include <asm/arch/gpioexpander.h>
#include <asm/arch/mux.h>
#include <asm/arch/usb.h>
#include <asm/arch/irda.h>
#include <asm/arch/board.h>
#include <asm/arch/common.h>
#include <asm/arch/keypad.h>
#include <asm/arch/menelaus.h>
#include <asm/arch/dma.h>
#include <asm/arch/gpmc.h>

#include <asm/io.h>

#define H4_FLASH_CS	0
#define H4_SMC91X_CS	1

static unsigned int row_gpios[6] = { 88, 89, 124, 11, 6, 96 };
static unsigned int col_gpios[7] = { 90, 91, 100, 36, 12, 97, 98 };

static int h4_keymap[] = {
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
	KEY(4, 4, KEY_ENTER),
	KEY(5, 0, KEY_V),
	KEY(5, 1, KEY_W),
	KEY(5, 2, KEY_L),
	KEY(5, 3, KEY_S),
	KEY(5, 4, KEY_ENTER),
	0
};

static struct mtd_partition h4_partitions[] = {
	/* bootloader (U-Boot, etc) in first sector */
	{
	      .name		= "bootloader",
	      .offset		= 0,
	      .size		= SZ_128K,
	      .mask_flags	= MTD_WRITEABLE, /* force read-only */
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

static struct flash_platform_data h4_flash_data = {
	.map_name	= "cfi_probe",
	.width		= 2,
	.parts		= h4_partitions,
	.nr_parts	= ARRAY_SIZE(h4_partitions),
};

static struct resource h4_flash_resource = {
	.flags		= IORESOURCE_MEM,
};

static struct platform_device h4_flash_device = {
	.name		= "omapflash",
	.id		= 0,
	.dev		= {
		.platform_data	= &h4_flash_data,
	},
	.num_resources	= 1,
	.resource	= &h4_flash_resource,
};

/* Select between the IrDA and aGPS module
 */
#if defined(CONFIG_OMAP_IR) || defined(CONFIG_OMAP_IR_MODULE)
static int h4_select_irda(struct device *dev, int state)
{
	unsigned char expa;
	int err = 0;

	if ((err = read_gpio_expa(&expa, 0x21))) {
		printk(KERN_ERR "Error reading from I/O expander\n");
		return err;
	}

	/* 'P6' enable/disable IRDA_TX and IRDA_RX */
	if (state & IR_SEL) {	/* IrDa */
		if ((err = write_gpio_expa(expa | 0x01, 0x21))) {
			printk(KERN_ERR "Error writing to I/O expander\n");
			return err;
		}
	} else {
		if ((err = write_gpio_expa(expa & ~0x01, 0x21))) {
			printk(KERN_ERR "Error writing to I/O expander\n");
			return err;
		}
	}
	return err;
}

static void set_trans_mode(struct work_struct *work)
{
	struct omap_irda_config *irda_config =
		container_of(work, struct omap_irda_config, gpio_expa.work);
	int mode = irda_config->mode;
	unsigned char expa;
	int err = 0;

	if ((err = read_gpio_expa(&expa, 0x20)) != 0) {
		printk(KERN_ERR "Error reading from I/O expander\n");
	}

	expa &= ~0x01;

	if (!(mode & IR_SIRMODE)) { /* MIR/FIR */
		expa |= 0x01;
	}

	if ((err = write_gpio_expa(expa, 0x20)) != 0) {
		printk(KERN_ERR "Error writing to I/O expander\n");
	}
}

static int h4_transceiver_mode(struct device *dev, int mode)
{
	struct omap_irda_config *irda_config = dev->platform_data;

	irda_config->mode = mode;
	cancel_delayed_work(&irda_config->gpio_expa);
	PREPARE_DELAYED_WORK(&irda_config->gpio_expa, set_trans_mode);
	schedule_delayed_work(&irda_config->gpio_expa, 0);

	return 0;
}
#else
static int h4_select_irda(struct device *dev, int state) { return 0; }
static int h4_transceiver_mode(struct device *dev, int mode) { return 0; }
#endif

static struct omap_irda_config h4_irda_data = {
	.transceiver_cap	= IR_SIRMODE | IR_MIRMODE | IR_FIRMODE,
	.transceiver_mode	= h4_transceiver_mode,
	.select_irda	 	= h4_select_irda,
	.rx_channel		= OMAP24XX_DMA_UART3_RX,
	.tx_channel		= OMAP24XX_DMA_UART3_TX,
	.dest_start		= OMAP_UART3_BASE,
	.src_start		= OMAP_UART3_BASE,
	.tx_trigger		= OMAP24XX_DMA_UART3_TX,
	.rx_trigger		= OMAP24XX_DMA_UART3_RX,
};

static struct resource h4_irda_resources[] = {
	[0] = {
		.start	= INT_24XX_UART3_IRQ,
		.end	= INT_24XX_UART3_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device h4_irda_device = {
	.name		= "omapirda",
	.id		= -1,
	.dev		= {
		.platform_data	= &h4_irda_data,
	},
	.num_resources	= 1,
	.resource	= h4_irda_resources,
};

static struct omap_kp_platform_data h4_kp_data = {
	.rows		= 6,
	.cols		= 7,
	.keymap 	= h4_keymap,
	.keymapsize 	= ARRAY_SIZE(h4_keymap),
	.rep		= 1,
	.row_gpios 	= row_gpios,
	.col_gpios 	= col_gpios,
};

static struct platform_device h4_kp_device = {
	.name		= "omap-keypad",
	.id		= -1,
	.dev		= {
		.platform_data = &h4_kp_data,
	},
};

static struct platform_device h4_lcd_device = {
	.name		= "lcd_h4",
	.id		= -1,
};

static struct platform_device *h4_devices[] __initdata = {
	&h4_flash_device,
	&h4_irda_device,
	&h4_kp_device,
	&h4_lcd_device,
};

/* 2420 Sysboot setup (2430 is different) */
static u32 get_sysboot_value(void)
{
	return (omap_readl(OMAP24XX_CONTROL_STATUS) & 0xFFF);
}

/* FIXME: This function should be moved to some other file, gpmc.c? */

/* H4-2420's always used muxed mode, H4-2422's always use non-muxed
 *
 * Note: OMAP-GIT doesn't correctly do is_cpu_omap2422 and is_cpu_omap2423
 *  correctly.  The macro needs to look at production_id not just hawkeye.
 */
static u32 is_gpmc_muxed(void)
{
	u32 mux;
	mux = get_sysboot_value();
	if ((mux & 0xF) == 0xd)
		return 1;	/* NAND config (could be either) */
	if (mux & 0x2)		/* if mux'ed */
		return 1;
	else
		return 0;
}

static inline void __init h4_init_debug(void)
{
	int eth_cs;
	unsigned long cs_mem_base;
	unsigned int muxed, rate;
	struct clk *l3ck;

	eth_cs	= H4_SMC91X_CS;

	l3ck = clk_get(NULL, "core_l3_ck");
	if (IS_ERR(l3ck))
		rate = 100000000;
	else
		rate = clk_get_rate(l3ck);

	if (is_gpmc_muxed())
		muxed = 0x200;
	else
		muxed = 0;

	/* Make sure CS1 timings are correct */
	gpmc_cs_write_reg(eth_cs, GPMC_CS_CONFIG1,
			  0x00011000 | muxed);

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
	} else {/* rate = 100000000 */
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

	udelay(100);

	omap_cfg_reg(M15_24XX_GPIO92);
	if (debug_card_init(cs_mem_base, OMAP24XX_ETHR_GPIO_IRQ) < 0)
		gpmc_cs_free(eth_cs);
}

static void __init h4_init_flash(void)
{
	unsigned long base;

	if (gpmc_cs_request(H4_FLASH_CS, SZ_64M, &base) < 0) {
		printk("Can't request GPMC CS for flash\n");
		return;
	}
	h4_flash_resource.start	= base;
	h4_flash_resource.end	= base + SZ_64M - 1;
}

static void __init omap_h4_init_irq(void)
{
	omap2_init_common_hw();
	omap_init_irq();
	omap_gpio_init();
	h4_init_flash();
}

static struct omap_uart_config h4_uart_config __initdata = {
#ifdef	CONFIG_MACH_OMAP2_H4_USB1
	.enabled_uarts = ((1 << 0) | (1 << 1)),
#else
	.enabled_uarts = ((1 << 0) | (1 << 1) | (1 << 2)),
#endif
};

static struct omap_mmc_config h4_mmc_config __initdata = {
	.mmc [0] = {
		.enabled	= 1,
		.wire4		= 1,
		.wp_pin		= -1,
		.power_pin	= -1,
		.switch_pin	= -1,
	},
};

static struct omap_lcd_config h4_lcd_config __initdata = {
	.ctrl_name	= "internal",
};

static struct omap_usb_config h4_usb_config __initdata = {
#ifdef	CONFIG_MACH_OMAP2_H4_USB1
	/* NOTE:  usb1 could also be used with 3 wire signaling */
	.pins[1]	= 4,
#endif

#ifdef	CONFIG_MACH_OMAP_H4_OTG
	/* S1.10 ON -- USB OTG port
	 * usb0 switched to Mini-AB port and isp1301 transceiver;
	 * S2.POS3 = OFF, S2.POS4 = ON ... to allow battery charging
	 */
	.otg		= 1,
	.pins[0]	= 4,
#ifdef	CONFIG_USB_GADGET_OMAP
	/* use OTG cable, or standard A-to-MiniB */
	.hmc_mode	= 0x14,	/* 0:dev/otg 1:host 2:disable */
#elif	defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)
	/* use OTG cable, or NONSTANDARD (B-to-MiniB) */
	.hmc_mode	= 0x11,	/* 0:host 1:host 2:disable */
#endif	/* XX */

#else
	/* S1.10 OFF -- usb "download port"
	 * usb0 switched to Mini-B port and isp1105 transceiver;
	 * S2.POS3 = ON, S2.POS4 = OFF ... to enable battery charging
	 */
	.register_dev	= 1,
	.pins[0]	= 3,
//	.hmc_mode	= 0x14,	/* 0:dev 1:host 2:disable */
	.hmc_mode	= 0x00,	/* 0:dev|otg 1:disable 2:disable */
#endif
};

static struct omap_board_config_kernel h4_config[] __initdata = {
	{ OMAP_TAG_UART,	&h4_uart_config },
	{ OMAP_TAG_MMC,		&h4_mmc_config },
	{ OMAP_TAG_LCD,		&h4_lcd_config },
	{ OMAP_TAG_USB,		&h4_usb_config },
};

#ifdef	CONFIG_MACH_OMAP_H4_TUSB

#include <linux/usb/musb.h>

static struct musb_hdrc_platform_data tusb_data = {
	.mode		= MUSB_OTG,
	.min_power	= 25,	/* x2 = 50 mA drawn from VBUS as peripheral */

	/* 1.8V supplied by Menelaus, other voltages supplied by VBAT;
	 * so no switching.
	 */
};

static void __init tusb_evm_setup(void)
{
	static char	announce[] __initdata =
				KERN_INFO "TUSB 6010 EVM\n";
	int		irq;
	unsigned	dmachan = 0;

	/* There are at least 32 different combinations of boards that
	 * are loosely called "H4", with a 2420 ... different OMAP chip
	 * revisions (with pin mux changes for DMAREQ, GPMC errata, etc),
	 * modifications of the CPU board, mainboard, EVM, TUSB etc.
	 * Plus omap2422, omap2423, etc.
	 *
	 * So you might need to tweak this setup to make the TUSB EVM
	 * behave on your particular setup ...
	 */

	/* Already set up:  GPMC AD[0..15], CLK, nOE, nWE, nADV_ALE */
	omap_cfg_reg(E2_GPMC_NCS2);
	omap_cfg_reg(L2_GPMC_NCS7);
	omap_cfg_reg(M1_GPMC_WAIT2);

	switch ((system_rev >> 8) & 0x0f) {
	case 0:		/* ES 1.0 */
	case 1:		/* ES 2.0 */
		/* Assume early board revision without optional ES2.0
		 * rework to swap J15 & AA10 so DMAREQ0 works
		 */
		omap_cfg_reg(AA10_242X_GPIO13);
		irq = 13;
		// omap_cfg_reg(J15_24XX_DMAREQ0);
		break;
	default:
		/* Later Menelaus boards can support all 6 DMA request
		 * lines, at the price of boot flash A23-A26.
		 */
		omap_cfg_reg(J15_24XX_GPIO99);
		irq = 99;
		dmachan = (1 << 1) | (1 << 0);
#if !(defined(CONFIG_MTD_OMAP_NOR) || defined(CONFIG_MTD_OMAP_NOR_MODULE))
		dmachan |= (1 << 5) | (1 << 4) (1 << 3) | (1 << 2);
#endif
		break;
	}

	if (tusb6010_setup_interface(&tusb_data,
			TUSB6010_REFCLK_24, /* waitpin */ 2,
			/* async cs */ 2, /* sync cs */ 7,
			irq, dmachan) == 0)
		printk(announce);
}

#endif

static struct i2c_board_info __initdata h4_i2c_board_info[] = {
#ifdef CONFIG_MENELAUS
	{
		I2C_BOARD_INFO("menelaus", 0x72),
		.irq = INT_24XX_SYS_NIRQ,
	},
#endif
};

static void __init omap_h4_init(void)
{
	/*
	 * Make sure the serial ports are muxed on at this point.
	 * You have to mux them off in device drivers later on
	 * if not needed.
	 */
#if defined(CONFIG_OMAP_IR) || defined(CONFIG_OMAP_IR_MODULE)
	omap_cfg_reg(K15_24XX_UART3_TX);
	omap_cfg_reg(K14_24XX_UART3_RX);
#endif

#if defined(CONFIG_KEYBOARD_OMAP) || defined(CONFIG_KEYBOARD_OMAP_MODULE)
	if (omap_has_menelaus()) {
		row_gpios[5] = 0;
		col_gpios[2] = 15;
		col_gpios[6] = 18;
	}
#endif

#ifdef	CONFIG_MACH_OMAP2_H4_USB1
	/* S3.3 controls whether these pins are for UART2 or USB1 */
	omap_cfg_reg(N14_24XX_USB1_SE0);
	omap_cfg_reg(P15_24XX_USB1_DAT);
	omap_cfg_reg(W20_24XX_USB1_TXEN);
	omap_cfg_reg(V19_24XX_USB1_RCV);
#endif

	i2c_register_board_info(1, h4_i2c_board_info,
			ARRAY_SIZE(h4_i2c_board_info));

	platform_add_devices(h4_devices, ARRAY_SIZE(h4_devices));
	omap_board_config = h4_config;
	omap_board_config_size = ARRAY_SIZE(h4_config);
	omap_serial_init();

	/* smc91x, debug leds, ps/2, extra uarts */
	h4_init_debug();

#ifdef	CONFIG_MACH_OMAP_H4_TUSB
	tusb_evm_setup();
#endif

}

static void __init omap_h4_map_io(void)
{
	omap2_map_common_io();
}

MACHINE_START(OMAP_H4, "OMAP2420 H4 board")
	/* Maintainer: Paul Mundt <paul.mundt@nokia.com> */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap_h4_map_io,
	.init_irq	= omap_h4_init_irq,
	.init_machine	= omap_h4_init,
	.timer		= &omap_timer,
MACHINE_END
