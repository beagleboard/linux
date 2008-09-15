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
#include <linux/i2c/menelaus.h>
#include <linux/spi/spi.h>
#include <linux/spi/tsc210x.h>

#include <media/v4l2-int-device.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>

#include <mach/control.h>
#include <mach/gpio.h>
#include <mach/gpioexpander.h>
#include <mach/mux.h>
#include <mach/usb.h>
#include <mach/irda.h>
#include <mach/board.h>
#include <mach/common.h>
#include <mach/keypad.h>
#include <mach/dma.h>
#include <mach/gpmc.h>

#include <asm/io.h>

#include <../drivers/media/video/ov9640.h>

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
	return (omap_ctrl_readl(OMAP24XX_CONTROL_STATUS) &
		(OMAP2_SYSBOOT_5_MASK | OMAP2_SYSBOOT_4_MASK |
		 OMAP2_SYSBOOT_3_MASK | OMAP2_SYSBOOT_2_MASK |
		 OMAP2_SYSBOOT_1_MASK | OMAP2_SYSBOOT_0_MASK));
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
	struct clk *gpmc_fck;

	eth_cs	= H4_SMC91X_CS;

	gpmc_fck = clk_get(NULL, "gpmc_fck");	/* Always on ENABLE_ON_INIT */
	if (IS_ERR(gpmc_fck)) {
		WARN_ON(1);
		return;
	}

	clk_enable(gpmc_fck);
	rate = clk_get_rate(gpmc_fck);
	clk_disable(gpmc_fck);
	clk_put(gpmc_fck);

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
		goto out;
	}

	udelay(100);

	omap_cfg_reg(M15_24XX_GPIO92);
	if (debug_card_init(cs_mem_base, OMAP24XX_ETHR_GPIO_IRQ) < 0)
		gpmc_cs_free(eth_cs);

out:
	clk_disable(gpmc_fck);
	clk_put(gpmc_fck);
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
	omap2_init_common_hw(NULL);
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
/*	.hmc_mode	= 0x14,*/	/* 0:dev 1:host 2:disable */
	.hmc_mode	= 0x00,		/* 0:dev|otg 1:disable 2:disable */
#endif
};

/* ----------------------------------------------------------------------- */

static struct tsc210x_config tsc_platform_data = {
	.use_internal		= 1,
	.monitor		= TSC_VBAT | TSC_TEMP,
	/* REVISIT temp calibration data -- board-specific; from EEPROM? */
	.mclk			= "sys_clkout",
};

static struct spi_board_info h4_spi_board_info[] __initdata = {
	{
		.modalias	= "tsc2101",
		.bus_num	= 1,
		.chip_select	= 0,
		.mode		= SPI_MODE_1,
		.irq		= OMAP_GPIO_IRQ(93),
		.max_speed_hz	= 16000000,
		.platform_data	= &tsc_platform_data,
	},

	/* nCS1 -- to lcd board, but unused
	 * nCS2 -- to WLAN/miniPCI
	 */
};

/* ----------------------------------------------------------------------- */

static struct omap_board_config_kernel h4_config[] __initdata = {
	{ OMAP_TAG_UART,	&h4_uart_config },
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
		/* omap_cfg_reg(J15_24XX_DMAREQ0); */
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

#if defined(CONFIG_VIDEO_OV9640) || defined(CONFIG_VIDEO_OV9640_MODULE)
/*
 * Common OV9640 register initialization for all image sizes, pixel formats,
 * and frame rates
 */
const static struct ov9640_reg ov9640_common[] = {
	{ 0x12, 0x80 }, { 0x11, 0x80 }, { 0x13, 0x8F },	/* COM7, CLKRC, COM8 */
	{ 0x01, 0x80 }, { 0x02, 0x80 }, { 0x04, 0x00 },	/* BLUE, RED, COM1 */
	{ 0x0E, 0x81 }, { 0x0F, 0x4F }, { 0x14, 0x4A },	/* COM5, COM6, COM9 */
	{ 0x16, 0x02 }, { 0x1B, 0x01 }, { 0x24, 0x70 },	/* ?, PSHFT, AEW */
	{ 0x25, 0x68 }, { 0x26, 0xD3 }, { 0x27, 0x90 },	/* AEB, VPT, BBIAS */
	{ 0x2A, 0x00 }, { 0x2B, 0x00 }, { 0x32, 0x24 },	/* EXHCH, EXHCL, HREF */
	{ 0x33, 0x02 }, { 0x37, 0x02 }, { 0x38, 0x13 },	/* CHLF, ADC, ACOM */
	{ 0x39, 0xF0 }, { 0x3A, 0x00 }, { 0x3B, 0x01 },	/* OFON, TSLB, COM11 */
	{ 0x3D, 0x90 }, { 0x3E, 0x02 }, { 0x3F, 0xF2 },	/* COM13, COM14, EDGE */
	{ 0x41, 0x02 }, { 0x42, 0xC8 },		/* COM16, COM17 */
	{ 0x43, 0xF0 }, { 0x44, 0x10 }, { 0x45, 0x6C },	/* ?, ?, ? */
	{ 0x46, 0x6C }, { 0x47, 0x44 }, { 0x48, 0x44 },	/* ?, ?, ? */
	{ 0x49, 0x03 }, { 0x59, 0x49 }, { 0x5A, 0x94 },	/* ?, ?, ? */
	{ 0x5B, 0x46 }, { 0x5C, 0x84 }, { 0x5D, 0x5C },	/* ?, ?, ? */
	{ 0x5E, 0x08 }, { 0x5F, 0x00 }, { 0x60, 0x14 },	/* ?, ?, ? */
	{ 0x61, 0xCE },					/* ? */
	{ 0x62, 0x70 }, { 0x63, 0x00 }, { 0x64, 0x04 },	/* LCC1, LCC2, LCC3 */
	{ 0x65, 0x00 }, { 0x66, 0x00 },			/* LCC4, LCC5 */
	{ 0x69, 0x00 }, { 0x6A, 0x3E }, { 0x6B, 0x3F },	/* HV, MBD, DBLV */
	{ 0x6C, 0x40 }, { 0x6D, 0x30 }, { 0x6E, 0x4B },	/* GSP1, GSP2, GSP3 */
	{ 0x6F, 0x60 }, { 0x70, 0x70 }, { 0x71, 0x70 },	/* GSP4, GSP5, GSP6 */
	{ 0x72, 0x70 }, { 0x73, 0x70 }, { 0x74, 0x60 },	/* GSP7, GSP8, GSP9 */
	{ 0x75, 0x60 }, { 0x76, 0x50 }, { 0x77, 0x48 },	/* GSP10,GSP11,GSP12 */
	{ 0x78, 0x3A }, { 0x79, 0x2E }, { 0x7A, 0x28 },	/* GSP13,GSP14,GSP15 */
	{ 0x7B, 0x22 }, { 0x7C, 0x04 }, { 0x7D, 0x07 },	/* GSP16,GST1, GST2 */
	{ 0x7E, 0x10 }, { 0x7F, 0x28 }, { 0x80, 0x36 },	/* GST3, GST4, GST5 */
	{ 0x81, 0x44 }, { 0x82, 0x52 }, { 0x83, 0x60 },	/* GST6, GST7, GST8 */
	{ 0x84, 0x6C }, { 0x85, 0x78 }, { 0x86, 0x8C },	/* GST9, GST10,GST11 */
	{ 0x87, 0x9E }, { 0x88, 0xBB }, { 0x89, 0xD2 },	/* GST12,GST13,GST14 */
	{ 0x8A, 0xE6 }, { 0x13, 0x8F }, { 0x00, 0x7F },	/* GST15, COM8 */
	{ OV9640_REG_TERM, OV9640_VAL_TERM }
};

static int ov9640_sensor_power_set(int power)
{
	unsigned char expa;
	int err;

	/* read current state of GPIO EXPA outputs */
	if ((err = read_gpio_expa(&expa, 0x20))) {
		printk(KERN_ERR "Error reading GPIO EXPA 0x20\n");
		return err;
	}

	expa = power ? expa | 0x80 : expa & ~0x08;

	/* Set GPIO EXPA P3 (CAMERA_MODULE_EN) to power-up sensor */
	if ((err = write_gpio_expa(expa, 0x20))) {
		printk(KERN_ERR "Error writing to GPIO EXPA 0x20\n");
		return err;
	}

	if (power) {
		/* read current state of GPIO EXPA outputs */
		if ((err = read_gpio_expa(&expa, 0x22))) {
			printk(KERN_ERR "Error reading GPIO EXPA\n");
			return err;
		}
		/* Clear GPIO EXPA P7 (CAM_RST) */
		if ((err = write_gpio_expa(expa & ~0x80, 0x22))) {
			printk(KERN_ERR "Error writing to GPIO EXPA\n");
			return err;
		}
	}

	return err;
}

static struct v4l2_ifparm ifparm = {
	.if_type = V4L2_IF_TYPE_BT656,
	.u = {
		.bt656 = {
			 .frame_start_on_rising_vs = 1,
			 .nobt_vs_inv = 1,
			 .mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT,
			 .clock_min = OV9640_XCLK_MIN,
			 .clock_max = OV9640_XCLK_MAX,
		 },
	},
};

static int ov9640_ifparm(struct v4l2_ifparm *p)
{
	*p = ifparm;

	return 0;
}

static struct ov9640_platform_data h4_ov9640_platform_data = {
	.power_set	= ov9640_sensor_power_set,
	.default_regs	= ov9640_common,
	.ifparm		= ov9640_ifparm,
};
#endif

static struct i2c_board_info __initdata h4_i2c_board_info[] = {
	{
		I2C_BOARD_INFO("rv5c387a", 0x32),
		/* no IRQ wired to OMAP; nINTB goes to AGPS */
	},
	{
		I2C_BOARD_INFO("menelaus", 0x72),
		.irq = INT_24XX_SYS_NIRQ,
	},
	{
		I2C_BOARD_INFO("isp1301_omap", 0x2d),
		.irq		= OMAP_GPIO_IRQ(125),
	},
#if defined(CONFIG_VIDEO_OV9640) || defined(CONFIG_VIDEO_OV9640_MODULE)
	{
		I2C_BOARD_INFO("ov9640", 0x30),
		.platform_data = &h4_ov9640_platform_data,
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

	/* Menelaus interrupt */
	omap_cfg_reg(W19_24XX_SYS_NIRQ);

	i2c_register_board_info(1, h4_i2c_board_info,
			ARRAY_SIZE(h4_i2c_board_info));

	platform_add_devices(h4_devices, ARRAY_SIZE(h4_devices));
	omap_board_config = h4_config;
	omap_board_config_size = ARRAY_SIZE(h4_config);
	omap_serial_init();
	h4_mmc_init();
	omap_register_i2c_bus(1, 100, h4_i2c_board_info,
			      ARRAY_SIZE(h4_i2c_board_info));

	/* smc91x, debug leds, ps/2, extra uarts */
	h4_init_debug();

#ifdef	CONFIG_MACH_OMAP_H4_TUSB
	tusb_evm_setup();
#endif

	/* defaults seem ok for:
	 * omap_cfg_reg(U18_24XX_SPI1_SCK);
	 * omap_cfg_reg(V20_24XX_SPI1_MOSI);
	 * omap_cfg_reg(T18_24XX_SPI1_MISO);
	 * omap_cfg_reg(U19_24XX_SPI1_NCS0);
	 */

	/* TSC2101 */
	omap_cfg_reg(P20_24XX_GPIO93);
	gpio_request(93, "tsc_irq");
	gpio_direction_input(93);

	omap_cfg_reg(W14_24XX_SYS_CLKOUT);	/* mclk */
	/* defaults seem ok for:
	 * omap_cfg_reg(Y15_EAC_AC_SCLK);	// bclk
	 * omap_cfg_reg(R14_EAC_AC_FS);
	 * omap_cfg_reg(V15_EAC_AC_DOUT);
	 * omap_cfg_reg(W15_EAC_AC_DIN);
	 */

	spi_register_board_info(h4_spi_board_info,
				ARRAY_SIZE(h4_spi_board_info));
}

static void __init omap_h4_map_io(void)
{
	omap2_set_globals_242x();
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
