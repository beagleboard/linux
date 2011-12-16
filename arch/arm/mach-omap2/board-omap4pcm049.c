/*
 * Board support file for Phytec phyCORE-OMAP4 Board.
 *
 * Copyright (C) 2011 Phytec Messtechnik GmbH
 *
 * Author: Jan Weitzel <armlinux@phytec.de>
 *
 * Based on mach-omap2/board-omap4panda.c
 *
 * Author: David Anders <x0132446@ti.com>
 *
 * Author: Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/usb/otg.h>
#include <linux/i2c/twl.h>
#include <linux/i2c/at24.h>
#include <linux/mfd/stmpe.h>
#include <linux/leds-pca9532.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/smsc911x.h>

#include <mach/hardware.h>
#include <asm/hardware/gic.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <video/omapdss.h>

#include <plat/board.h>
#include <plat/usb.h>
#include <plat/gpmc.h>
#include <plat/gpmc-smsc911x.h>
#include <plat/mmc.h>
#include <video/omap-panel-generic-dpi.h>

#include "common.h"
#include "hsmmc.h"
#include "control.h"
#include "mux.h"
#include "common-board-devices.h"

#define OMAP4_PCM049_ETH_GPIO_IRQ	121
#define OMAP4_PCM049_ETH_CS		5
#define OMAP4_PCM049_STMPE811_GPIO_IRQ	117
#define OMAP4_PCM049_LCD_ENABLE		118

static struct gpio_led gpio_leds[] = {
	{
		.name			= "modul:red:status1",
		.default_trigger	= "heartbeat",
		.gpio			= 152,
	},
	{
		.name			= "modul:green:status2",
		.default_trigger	= "mmc0",
		.gpio			= 153,
	},
};

static struct gpio_led_platform_data gpio_led_info = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_led_info,
	},
};

static const struct usbhs_omap_board_data usbhs_bdata __initconst = {
	.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[1] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,
	.phy_reset  = false,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};

static void __init omap4_ehci_init(void)
{
	struct clk *phy_ref_clk;
	/* FREF_CLK3 provides the 19.2 MHz reference clock to the PHY */
	phy_ref_clk = clk_get(NULL, "auxclk3_ck");
	if (IS_ERR(phy_ref_clk)) {
		pr_err("Cannot request auxclk3\n");
		return;
	}
	clk_set_rate(phy_ref_clk, 19200000);
	clk_enable(phy_ref_clk);

	usbhs_init(&usbhs_bdata);
	return;
}

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_UTMI,
	.mode			= MUSB_OTG,
	.power			= 100,
};

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= -EINVAL,
	}, {
		.mmc		= 5,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_wp	= -EINVAL,
		.gpio_cd	= 30,	/* wk30 */
		.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34, /* 3V3 */
	}, {}	/* Terminator */
};

#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
static struct omap_smsc911x_platform_data __initdata board_smsc911x_data = {
	.cs             = OMAP4_PCM049_ETH_CS,
	.gpio_irq       = OMAP4_PCM049_ETH_GPIO_IRQ,
	.gpio_reset     = -EINVAL,
	.flags          = SMSC911X_USE_16BIT,
};

static inline void __init pcm049_init_smsc911x(void)
{
	omap_mux_init_gpio(OMAP4_PCM049_ETH_GPIO_IRQ, OMAP_PIN_INPUT);
	gpmc_smsc911x_init(&board_smsc911x_data);
}
#else
static inline void __init pcm049_init_smsc911x(void) { return; }
#endif

static int omap4_twl6030_hsmmc_late_init(struct device *dev)
{
	int ret = 0;
	struct platform_device *pdev = container_of(dev,
				struct platform_device, dev);
	struct omap_mmc_platform_data *pdata = dev->platform_data;

	/* Setting MMC1 Card detect Irq */
	if (pdev->id == 0) {
		ret = twl6030_mmc_card_detect_config();
		if (ret)
			dev_err(dev, "%s: Error card detect config(%d)\n",
				__func__, ret);
		pdata->slots[0].card_detect_irq = TWL6030_IRQ_BASE +
						MMCDETECT_INTR_OFFSET;
		pdata->slots[0].card_detect = twl6030_mmc_card_detect;
	}
	return ret;
}

static __init void omap4_twl6030_hsmmc_set_late_init(struct device *dev)
{
	struct omap_mmc_platform_data *pdata;

	/* dev can be null if CONFIG_MMC_OMAP_HS is not set */
	if (!dev) {
		pr_err("Failed omap4_twl6030_hsmmc_set_late_init\n");
		return;
	}
	pdata = dev->platform_data;

	pdata->init =	omap4_twl6030_hsmmc_late_init;
}

static int __init omap4_twl6030_hsmmc_init(struct omap2_hsmmc_info *controllers)
{
	struct omap2_hsmmc_info *c;

	omap2_hsmmc_init(controllers);
	for (c = controllers; c->mmc; c++)
		omap4_twl6030_hsmmc_set_late_init(c->dev);

	return 0;
}

/* Fixed regulator for max1027 */
static struct regulator_consumer_supply pcm049_vcc_3v3_consumer_supply[] = {
	REGULATOR_SUPPLY("vcc", "4-0064"),
};

struct regulator_init_data pcm049_vcc_3v3_initdata = {
	.consumer_supplies = pcm049_vcc_3v3_consumer_supply,
	.num_consumer_supplies = ARRAY_SIZE(pcm049_vcc_3v3_consumer_supply),
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
};

static struct fixed_voltage_config pcm049_vcc_3v3_config = {
	.supply_name		= "pcm049_vcc_3v3",
	.microvolts		= 3300000,
	.gpio			= -EINVAL,
	.enabled_at_boot	= 1,
	.init_data		= &pcm049_vcc_3v3_initdata,
};

static struct platform_device pcm049_vcc_3v3_device = {
	.name	= "reg-fixed-voltage",
	.id	= 0,
	.dev	= {
		.platform_data = &pcm049_vcc_3v3_config,
	},
};

static struct at24_platform_data board_eeprom = {
	.byte_len = 4096,
	.page_size = 32,
	.flags = AT24_FLAG_ADDR16,
};

static struct stmpe_gpio_platform_data pba_gpio_stm_data = {
	.gpio_base = -1,
	.norequest_mask = STMPE_GPIO_NOREQ_811_TOUCH,
};

static struct stmpe_ts_platform_data pba_ts_stm_pdata = {
	.sample_time = 4,
	.mod_12b = 1,
	.ref_sel = 0,
	.adc_freq = 1,
	.ave_ctrl = 3,
	.touch_det_delay = 3,
	.settling = 3,
	.fraction_z = 7,
	.i_drive = 0,
};

static struct stmpe_platform_data pba_stm_pdata = {
	.blocks = STMPE_BLOCK_GPIO | STMPE_BLOCK_TOUCHSCREEN,
	.irq_base = TWL6030_IRQ_END,
	.irq_trigger = IRQF_TRIGGER_RISING,
	.irq_invert_polarity = true,
	.gpio = &pba_gpio_stm_data,
	.ts = &pba_ts_stm_pdata,
};

static struct pca9532_platform_data pba_pca9532 = {
	.leds = {
		{
			.name = "board:red:free_use1",
			.state = PCA9532_OFF,
			.type = PCA9532_TYPE_LED,
		}, {
			.name = "board:yellow:free_use2",
			.state = PCA9532_OFF,
			.type = PCA9532_TYPE_LED,
		}, {
			.name = "board:yellow:free_use3",
			.state = PCA9532_OFF,
			.type = PCA9532_TYPE_LED,
		}, {
			.name = "board:green:free_use4",
			.state = PCA9532_OFF,
			.type = PCA9532_TYPE_LED,
		},
	},
	.psc = { 1, 1 },
	.pwm = { 1, 1 },
};

static struct i2c_board_info __initdata pcm049_i2c_1_boardinfo[] = {
	{
		I2C_BOARD_INFO("at24", 0x57), /* E0=1, E1=1, E2=1 */
		.platform_data = &board_eeprom,
	},
};

static struct i2c_board_info __initdata pcm049_i2c_3_boardinfo[] = {
};

static struct i2c_board_info __initdata pcm049_i2c_4_boardinfo[] = {
	{
		I2C_BOARD_INFO("stmpe811", 0x41),	/* Touch controller */
		.irq = OMAP_GPIO_IRQ(OMAP4_PCM049_STMPE811_GPIO_IRQ),
		.platform_data = &pba_stm_pdata,
	}, {
		I2C_BOARD_INFO("max1037", 0x64),	/* A/D converter */
	}, {
		I2C_BOARD_INFO("pca9533", 0x62),	/* Leds pca9533 */
		.platform_data = &pba_pca9532,
	}
};

static struct twl4030_platform_data pcm049_twldata;

static int __init pcm049_i2c_init(void)
{
	/* I2C1 */
	omap4_pmic_get_config(&pcm049_twldata, TWL_COMMON_PDATA_USB,
			TWL_COMMON_REGULATOR_VDAC |
			TWL_COMMON_REGULATOR_VAUX2 |
			TWL_COMMON_REGULATOR_VAUX3 |
			TWL_COMMON_REGULATOR_VMMC |
			TWL_COMMON_REGULATOR_VPP |
			TWL_COMMON_REGULATOR_VANA |
			TWL_COMMON_REGULATOR_VCXIO |
			TWL_COMMON_REGULATOR_VUSB |
			TWL_COMMON_REGULATOR_CLK32KG);
	omap4_pmic_init("twl6030", &pcm049_twldata);
	i2c_register_board_info(1, pcm049_i2c_1_boardinfo,
				ARRAY_SIZE(pcm049_i2c_1_boardinfo));

	/* I2C3 */
	omap_register_i2c_bus(3, 400, pcm049_i2c_3_boardinfo,
				ARRAY_SIZE(pcm049_i2c_3_boardinfo));

	/* I2C4 */
	if (omap_mux_init_gpio(OMAP4_PCM049_STMPE811_GPIO_IRQ, OMAP_PIN_INPUT))
		printk(KERN_ERR "Failed to mux GPIO%d for STMPE811 IRQ\n",
			OMAP4_PCM049_STMPE811_GPIO_IRQ);
	else if (gpio_request(OMAP4_PCM049_STMPE811_GPIO_IRQ, "STMPE811 irq"))
		printk(KERN_ERR "Failed to request GPIO%d for STMPE811 IRQ\n",
			OMAP4_PCM049_STMPE811_GPIO_IRQ);
	else
		gpio_direction_input(OMAP4_PCM049_STMPE811_GPIO_IRQ);

	omap_register_i2c_bus(4, 400, pcm049_i2c_4_boardinfo,
				ARRAY_SIZE(pcm049_i2c_4_boardinfo));
	return 0;
}

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	OMAP4_MUX(SDMMC5_DAT0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT2, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP4_MUX(SDMMC5_DAT3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),

	/* dispc2_data23 */
	OMAP4_MUX(USBB2_ULPITLL_STP, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data22 */
	OMAP4_MUX(USBB2_ULPITLL_DIR, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data21 */
	OMAP4_MUX(USBB2_ULPITLL_NXT, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data20 */
	OMAP4_MUX(USBB2_ULPITLL_DAT0, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data19 */
	OMAP4_MUX(USBB2_ULPITLL_DAT1, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data18 */
	OMAP4_MUX(USBB2_ULPITLL_DAT2, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data15 */
	OMAP4_MUX(USBB2_ULPITLL_DAT3, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data14 */
	OMAP4_MUX(USBB2_ULPITLL_DAT4, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data13 */
	OMAP4_MUX(USBB2_ULPITLL_DAT5, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data12 */
	OMAP4_MUX(USBB2_ULPITLL_DAT6, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data11 */
	OMAP4_MUX(USBB2_ULPITLL_DAT7, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data10 */
	OMAP4_MUX(DPM_EMU3, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data9 */
	OMAP4_MUX(DPM_EMU4, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data16 */
	OMAP4_MUX(DPM_EMU5, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data17 */
	OMAP4_MUX(DPM_EMU6, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_hsync */
	OMAP4_MUX(DPM_EMU7, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_pclk */
	OMAP4_MUX(DPM_EMU8, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_vsync */
	OMAP4_MUX(DPM_EMU9, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_de */
	OMAP4_MUX(DPM_EMU10, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data8 */
	OMAP4_MUX(DPM_EMU11, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data7 */
	OMAP4_MUX(DPM_EMU12, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data6 */
	OMAP4_MUX(DPM_EMU13, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data5 */
	OMAP4_MUX(DPM_EMU14, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data4 */
	OMAP4_MUX(DPM_EMU15, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data3 */
	OMAP4_MUX(DPM_EMU16, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data2 */
	OMAP4_MUX(DPM_EMU17, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data1 */
	OMAP4_MUX(DPM_EMU18, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),
	/* dispc2_data0 */
	OMAP4_MUX(DPM_EMU19, OMAP_PIN_OUTPUT | OMAP_MUX_MODE5),

	{ .reg_offset = OMAP_MUX_TERMINATOR },
};

static struct omap_device_pad serial2_pads[] __initdata = {
	OMAP_MUX_STATIC("uart2_cts.uart2_cts",
			 OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0),
	OMAP_MUX_STATIC("uart2_rts.uart2_rts",
			 OMAP_PIN_OUTPUT | OMAP_MUX_MODE0),
	OMAP_MUX_STATIC("uart2_rx.uart2_rx",
			 OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0),
	OMAP_MUX_STATIC("uart2_tx.uart2_tx",
			 OMAP_PIN_OUTPUT | OMAP_MUX_MODE0),
};

static struct omap_device_pad serial3_pads[] __initdata = {
	OMAP_MUX_STATIC("uart3_cts_rctx.uart3_cts_rctx",
			 OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0),
	OMAP_MUX_STATIC("uart3_rts_sd.uart3_rts_sd",
			 OMAP_PIN_OUTPUT | OMAP_MUX_MODE0),
	OMAP_MUX_STATIC("uart3_rx_irrx.uart3_rx_irrx",
			 OMAP_PIN_INPUT | OMAP_MUX_MODE0),
	OMAP_MUX_STATIC("uart3_tx_irtx.uart3_tx_irtx",
			 OMAP_PIN_OUTPUT | OMAP_MUX_MODE0),
};

static struct omap_board_data serial2_data __initdata = {
	.id             = 1,
	.pads           = serial2_pads,
	.pads_cnt       = ARRAY_SIZE(serial2_pads),
};

static struct omap_board_data serial3_data __initdata = {
	.id             = 2,
	.pads           = serial3_pads,
	.pads_cnt       = ARRAY_SIZE(serial3_pads),
};

static inline void board_serial_init(void)
{
	omap_serial_init_port(&serial2_data, NULL);
	omap_serial_init_port(&serial3_data, NULL);
}
#else
#define board_mux	NULL

static inline void board_serial_init(void)
{
	omap_serial_init();
}
#endif

/* Display */
static int pcm049_panel_enable_lcd(struct omap_dss_device *dssdev)
{
	return gpio_direction_output(OMAP4_PCM049_LCD_ENABLE, 1);
}

static void pcm049_panel_disable_lcd(struct omap_dss_device *dssdev)
{
	gpio_direction_output(OMAP4_PCM049_LCD_ENABLE, 0);
	return;
}

/* Using generic display panel */
static struct panel_generic_dpi_data omap4_dpi_panel = {
	.name			= "pd050vl1",
	.platform_enable	= pcm049_panel_enable_lcd,
	.platform_disable	= pcm049_panel_disable_lcd,
};

struct omap_dss_device pcm049_dpi_device = {
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.name			= "dpi",
	.driver_name		= "generic_dpi_panel",
	.data			= &omap4_dpi_panel,
	.phy.dpi.data_lines	= 24,
	.channel		= OMAP_DSS_CHANNEL_LCD2,
};

static void pcm049_dvi_mux_init(void)
{
	/* PAD0_HDMI_HPD_PAD1_HDMI_CEC */
	omap_mux_init_signal("hdmi_hpd",
			OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("hdmi_cec",
			OMAP_PIN_INPUT_PULLUP);
	/* PAD0_HDMI_DDC_SCL_PAD1_HDMI_DDC_SDA */
	omap_mux_init_signal("hdmi_ddc_scl",
			OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("hdmi_ddc_sda",
			OMAP_PIN_INPUT_PULLUP);
}

static struct omap_dss_device  pcm049_dvi_device = {
	.name = "dvi",
	.driver_name = "hdmi_panel",
	.type = OMAP_DISPLAY_TYPE_HDMI,
	.clocks	= {
		.dispc	= {
			.dispc_fclk_src	= OMAP_DSS_CLK_SRC_FCK,
		},
		.hdmi	= {
			.regn	= 15,
			.regm2	= 1,
		},
	},
	.channel = OMAP_DSS_CHANNEL_DIGIT,
};

static struct omap_dss_device *pcm049_dss_devices[] = {
	&pcm049_dpi_device,
	&pcm049_dvi_device,
};

static struct omap_dss_board_info pcm049_dss_data = {
	.num_devices	= ARRAY_SIZE(pcm049_dss_devices),
	.devices	= pcm049_dss_devices,
	.default_device = &pcm049_dpi_device,
};

void pcm049_display_init(void)
{
	omap_mux_init_gpio(OMAP4_PCM049_LCD_ENABLE, OMAP_PIN_OUTPUT);

	if ((gpio_request(OMAP4_PCM049_LCD_ENABLE, "DISP_ENA") == 0) &&
		(gpio_direction_output(OMAP4_PCM049_LCD_ENABLE, 1) == 0)) {
		gpio_export(OMAP4_PCM049_LCD_ENABLE, 0);
		gpio_set_value(OMAP4_PCM049_LCD_ENABLE, 0);
	} else
		printk(KERN_ERR "could not obtain gpio for DISP_ENA");
	pcm049_dvi_mux_init();
	omap_display_init(&pcm049_dss_data);
}

static struct platform_device *pcm049_devices[] __initdata = {
	&pcm049_vcc_3v3_device,
	&leds_gpio,
};

#define	TWL_PHOENIX_DEV_ON	0x25

static void pcm049_power_off(void)
{
	printk(KERN_INFO "Goodbye phyCORE OMAP4!\n");
	twl_i2c_write_u8(TWL6030_MODULE_ID0, 0x7, TWL_PHOENIX_DEV_ON);
}


static void __init pcm049_init(void)
{
	pm_power_off = pcm049_power_off;
	omap4_mux_init(board_mux, NULL, OMAP_PACKAGE_CBS);
	pcm049_init_smsc911x();
	pcm049_i2c_init();
	platform_add_devices(pcm049_devices, ARRAY_SIZE(pcm049_devices));
	board_serial_init();
	omap_sdrc_init(NULL, NULL);
	omap4_twl6030_hsmmc_init(mmc);
	omap4_ehci_init();
	usb_musb_init(&musb_board_data);
	pcm049_display_init();
}

static void __init pcm049_map_io(void)
{
	omap2_set_globals_443x();
	omap44xx_map_common_io();
}

MACHINE_START(PCM049, "phyCORE OMAP4")
	/* Maintainer: Jan Weitzel - Phytec Messtechnik GmbH */
	.atag_offset	= 0x100,
	.reserve	= omap_reserve,
	.map_io		= pcm049_map_io,
	.init_early	= omap4430_init_early,
	.init_irq	= gic_init_irq,
	.handle_irq	= gic_handle_irq,
	.init_machine	= pcm049_init,
	.timer		= &omap4_timer,
MACHINE_END
