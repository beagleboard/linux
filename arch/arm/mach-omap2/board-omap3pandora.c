/*
 * board-omap3pandora.c (Pandora Handheld Console)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <linux/i2c/twl4030.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/flash.h>
#include <asm/mach/map.h>

#include <mach/board.h>
#include <mach/common.h>
#include <mach/gpio.h>
#include <mach/gpmc.h>
#include <mach/hardware.h>
#include <mach/mmc.h>
#include <mach/nand.h>
#include <mach/usb-ehci.h>
#include <mach/usb-musb.h>

#include "sdram-micron-mt46h32m32lf-6.h"

#define NAND_BLOCK_SIZE SZ_128K
#define GPMC_CS0_BASE  0x60
#define GPMC_CS_SIZE   0x30

static struct mtd_partition omap3pandora_nand_partitions[] = {
	{
		.name           = "xloader",
		.offset         = 0,			/* Offset = 0x00000 */
		.size           = 4 * NAND_BLOCK_SIZE,
		.mask_flags     = MTD_WRITEABLE
	}, {
		.name           = "uboot",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x80000 */
		.size           = 14 * NAND_BLOCK_SIZE,
	}, {
		.name           = "uboot environment",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x240000 */
		.size           = 2 * NAND_BLOCK_SIZE,
	}, {
		.name           = "linux",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x280000 */
		.size           = 32 * NAND_BLOCK_SIZE,
	}, {
		.name           = "rootfs",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x680000 */
		.size           = MTDPART_SIZ_FULL,
	},
};

static struct omap_nand_platform_data omap3pandora_nand_data = {
	.parts		= omap3pandora_nand_partitions,
	.nr_parts	= ARRAY_SIZE(omap3pandora_nand_partitions),
	.dma_channel	= -1,	/* disable DMA in OMAP NAND driver */
};

static struct resource omap3pandora_nand_resource[] = {
	{
		.flags		= IORESOURCE_MEM,
	},
};

static struct platform_device omap3pandora_nand_device = {
	.name		= "omap2-nand",
	.id		= -1,
	.dev		= {
		.platform_data	= &omap3pandora_nand_data,
	},
	.num_resources	= ARRAY_SIZE(omap3pandora_nand_resource),
	.resource	= omap3pandora_nand_resource,
};

static void __init omap3pandora_flash_init(void)
{
	u8 cs = 0;
	u8 nandcs = GPMC_CS_NUM + 1;

	u32 gpmc_base_add = OMAP34XX_GPMC_VIRT;

	/* find out the chip-select on which NAND exists */
	while (cs < GPMC_CS_NUM) {
		u32 ret = 0;
		ret = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG1);

		if ((ret & 0xC00) == 0x800) {
			printk(KERN_INFO "Found NAND on CS%d\n", cs);
			if (nandcs > GPMC_CS_NUM)
				nandcs = cs;
		}
		cs++;
	}

	if (nandcs > GPMC_CS_NUM) {
		printk(KERN_INFO "NAND: Unable to find configuration "
				 "in GPMC\n ");
		return;
	}

	if (nandcs < GPMC_CS_NUM) {
		omap3pandora_nand_data.cs = nandcs;
		omap3pandora_nand_data.gpmc_cs_baseaddr = (void *)
			(gpmc_base_add + GPMC_CS0_BASE + nandcs * GPMC_CS_SIZE);
		omap3pandora_nand_data.gpmc_baseaddr = (void *) (gpmc_base_add);

		printk(KERN_INFO "Registering NAND on CS%d\n", nandcs);
		if (platform_device_register(&omap3pandora_nand_device) < 0)
			printk(KERN_ERR "Unable to register NAND device\n");
	}
}

static struct omap_uart_config omap3pandora_uart_config __initdata = {
	.enabled_uarts	= (1 << 2), /* UART3 */
};

static struct twl4030_gpio_platform_data omap3pandora_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
};

static struct twl4030_usb_data omap3pandora_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

static struct twl4030_platform_data omap3pandora_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,
	.gpio		= &omap3pandora_gpio_data,
	.usb		= &omap3pandora_usb_data,
};

static struct i2c_board_info __initdata omap3pandora_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl4030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &omap3pandora_twldata,
	},
};

static int __init omap3pandora_i2c_init(void)
{
	omap_register_i2c_bus(1, 2600, omap3pandora_i2c_boardinfo,
			ARRAY_SIZE(omap3pandora_i2c_boardinfo));
	/* i2c2 pins are not connected */
	omap_register_i2c_bus(3, 400, NULL, 0);
	return 0;
}

static void __init omap3pandora_init_irq(void)
{
	omap2_init_common_hw(mt46h32m32lf6_sdrc_params);
	omap_init_irq();
	omap_gpio_init();
}

static struct platform_device omap3pandora_lcd_device = {
	.name		= "pandora_lcd",
	.id		= -1,
};

static struct omap_lcd_config omap3pandora_lcd_config __initdata = {
	.ctrl_name	= "internal",
};

static struct omap_board_config_kernel omap3pandora_config[] __initdata = {
	{ OMAP_TAG_UART,	&omap3pandora_uart_config },
	{ OMAP_TAG_LCD,		&omap3pandora_lcd_config },
};

static struct platform_device *omap3pandora_devices[] __initdata = {
	&omap3pandora_lcd_device,
};

static struct twl4030_hsmmc_info mmc[] __initdata = {
	{
		.mmc		= 1,
		.wires		= 4,
		.gpio_cd	= -EINVAL,
	},
	{}	/* Terminator */
};

static void __init omap3pandora_init(void)
{
	omap3pandora_i2c_init();
	platform_add_devices(omap3pandora_devices, ARRAY_SIZE(omap3pandora_devices));
	omap_board_config = omap3pandora_config;
	omap_board_config_size = ARRAY_SIZE(omap3pandora_config);
	omap_serial_init();
	hsmmc_init(mmc);
	usb_musb_init();
	usb_ehci_init();
	omap3pandora_flash_init();
}

static void __init omap3pandora_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

MACHINE_START(OMAP3_PANDORA, "Pandora Handheld Console")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap3pandora_map_io,
	.init_irq	= omap3pandora_init_irq,
	.init_machine	= omap3pandora_init,
	.timer		= &omap_timer,
MACHINE_END
