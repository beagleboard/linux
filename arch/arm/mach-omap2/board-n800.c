/*
 * linux/arch/arm/mach-omap2/board-n800.c
 *
 * Copyright (C) 2005-2007 Nokia Corporation
 * Author: Juha Yrjola <juha.yrjola@nokia.com>
 *
 * Modified from mach-omap2/board-generic.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <mach/gpio.h>
#include <mach/usb.h>
#include <mach/board.h>
#include <mach/common.h>
#include <mach/mcspi.h>
#include <mach/lcd_mipid.h>
#include <mach/clock.h>
#include <mach/menelaus.h>
#include <mach/omapfb.h>
#include <mach/blizzard.h>
#include <mach/onenand.h>
#include <mach/board-nokia.h>

#include <../drivers/cbus/tahvo.h>
#include <../drivers/media/video/tcm825x.h>

/* FIXME: These should be done in platform_data */
#define OMAP_TAG_TEA5761		0x4f10
#define OMAP_TAG_TMP105			0x4f11

struct omap_tea5761_config {
	u16 enable_gpio;
};

struct omap_tmp105_config {
	u16 tmp105_irq_pin;
	int (* set_power)(int enable);
};


#define N800_BLIZZARD_POWERDOWN_GPIO	15
#define N800_STI_GPIO			62
#define N800_KEYB_IRQ_GPIO		109
#define N800_DAV_IRQ_GPIO		103
#define N800_TSC2301_RESET_GPIO		118

void __init nokia_n800_init_irq(void)
{
	omap2_init_common_hw(NULL);
	omap_init_irq();
	omap_gpio_init();

#ifdef CONFIG_OMAP_STI
	if (gpio_request(N800_STI_GPIO, "STI") < 0) {
		printk(KERN_ERR "Failed to request GPIO %d for STI\n",
		       N800_STI_GPIO);
		return;
	}

	gpio_direction_output(N800_STI_GPIO, 0);
#endif
}

#if defined(CONFIG_MENELAUS) && defined(CONFIG_SENSORS_TMP105)

static int n800_tmp105_set_power(int enable)
{
	return menelaus_set_vaux(enable ? 2800 : 0);
}

#else

#define n800_tmp105_set_power NULL

#endif

static struct omap_uart_config n800_uart_config __initdata = {
	.enabled_uarts = (1 << 0) | (1 << 2),
};

#include "../../../drivers/cbus/retu.h"

static struct omap_fbmem_config n800_fbmem0_config __initdata = {
	.size = 752 * 1024,
};

static struct omap_fbmem_config n800_fbmem1_config __initdata = {
	.size = 752 * 1024,
};

static struct omap_fbmem_config n800_fbmem2_config __initdata = {
	.size = 752 * 1024,
};

static struct omap_tmp105_config n800_tmp105_config __initdata = {
	.tmp105_irq_pin = 125,
	.set_power = n800_tmp105_set_power,
};

static void mipid_shutdown(struct mipid_platform_data *pdata)
{
	if (pdata->nreset_gpio != -1) {
		pr_info("shutdown LCD\n");
		gpio_set_value(pdata->nreset_gpio, 0);
		msleep(120);
	}
}

static struct mipid_platform_data n800_mipid_platform_data = {
	.shutdown = mipid_shutdown,
};

static void __init mipid_dev_init(void)
{
	const struct omap_lcd_config *conf;

	conf = omap_get_config(OMAP_TAG_LCD, struct omap_lcd_config);
	if (conf != NULL) {
		n800_mipid_platform_data.nreset_gpio = conf->nreset_gpio;
		n800_mipid_platform_data.data_lines = conf->data_lines;
	}
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
	gpio_set_value(N800_BLIZZARD_POWERDOWN_GPIO, 1);
}

static void blizzard_power_down(struct device *dev)
{
	gpio_set_value(N800_BLIZZARD_POWERDOWN_GPIO, 0);
	blizzard_enable_clocks(0);

	/* Vcore to 1.005V */
	tahvo_set_clear_reg_bits(0x07, 0xf, 0);
}

static struct blizzard_platform_data n800_blizzard_data = {
	.power_up	= blizzard_power_up,
	.power_down	= blizzard_power_down,
	.get_clock_rate	= blizzard_get_clock_rate,
	.te_connected	= 1,
};

static void __init blizzard_dev_init(void)
{
	int r;

	r = gpio_request(N800_BLIZZARD_POWERDOWN_GPIO, "Blizzard pd");
	if (r < 0)
		return;
	gpio_direction_output(N800_BLIZZARD_POWERDOWN_GPIO, 1);

	blizzard_get_clocks();
	omapfb_set_ctrl_platform_data(&n800_blizzard_data);
}

static struct omap_board_config_kernel n800_config[] __initdata = {
	{ OMAP_TAG_UART,	                &n800_uart_config },
	{ OMAP_TAG_FBMEM,			&n800_fbmem0_config },
	{ OMAP_TAG_FBMEM,			&n800_fbmem1_config },
	{ OMAP_TAG_FBMEM,			&n800_fbmem2_config },
	{ OMAP_TAG_TMP105,			&n800_tmp105_config },
};

static int __init tea5761_dev_init(void)
{
	const struct omap_tea5761_config *info;
	int enable_gpio = 0;

	info = omap_get_config(OMAP_TAG_TEA5761, struct omap_tea5761_config);
	if (info)
		enable_gpio = info->enable_gpio;

	if (enable_gpio) {
		pr_debug("Enabling tea5761 at GPIO %d\n",
			 enable_gpio);

		if (gpio_request(enable_gpio, "TEA5761 enable") < 0) {
			printk(KERN_ERR "Can't request GPIO %d\n",
			       enable_gpio);
			return -ENODEV;
		}

		gpio_direction_output(enable_gpio, 0);
		udelay(50);
		gpio_set_value(enable_gpio, 1);
	}

	return 0;
}

static struct omap2_mcspi_device_config tsc2301_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel = 1,
};

static struct omap2_mcspi_device_config mipid_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,
};

static struct omap2_mcspi_device_config cx3110x_mcspi_config = {
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
		.platform_data	= &n800_mipid_platform_data,
	}, {
		.modalias	= "cx3110x",
		.bus_num	= 2,
		.chip_select	= 0,
		.max_speed_hz   = 48000000,
		.controller_data= &cx3110x_mcspi_config,
	},
	{
		.modalias	= "tsc2301",
		.bus_num	= 1,
		.chip_select	= 0,
		.max_speed_hz   = 6000000,
		.controller_data= &tsc2301_mcspi_config,
	},
};

static struct spi_board_info n810_spi_board_info[] __initdata = {
	{
		.modalias	 = "lcd_mipid",
		.bus_num	 = 1,
		.chip_select	 = 1,
		.max_speed_hz	 = 4000000,
		.controller_data = &mipid_mcspi_config,
		.platform_data	 = &n800_mipid_platform_data,
	},
	{
		.modalias	 = "cx3110x",
		.bus_num	 = 2,
		.chip_select	 = 0,
		.max_speed_hz    = 48000000,
		.controller_data = &cx3110x_mcspi_config,
	},
	{
		.modalias	 = "tsc2005",
		.bus_num	 = 1,
		.chip_select	 = 0,
		.max_speed_hz    = 6000000,
	},
};

#if defined(CONFIG_CBUS_RETU) && defined(CONFIG_LEDS_OMAP_PWM)

void retu_keypad_led_set_power(struct omap_pwm_led_platform_data *self,
			       int on_off)
{
	if (on_off) {
		retu_write_reg(RETU_REG_CTRL_SET, 1 << 6);
		msleep(2);
		retu_write_reg(RETU_REG_CTRL_SET, 1 << 3);
	} else {
		retu_write_reg(RETU_REG_CTRL_CLR, (1 << 6) | (1 << 3));
	}
}

static struct omap_pwm_led_platform_data n800_keypad_led_data = {
	.name = "keypad",
	.intensity_timer = 10,
	.blink_timer = 9,
	.set_power = retu_keypad_led_set_power,
};

static struct platform_device n800_keypad_led_device = {
	.name		= "omap_pwm_led",
	.id		= -1,
	.dev		= {
		.platform_data = &n800_keypad_led_data,
	},
};
#endif

#if defined(CONFIG_TOUCHSCREEN_TSC2301)
static void __init n800_ts_set_config(void)
{
	const struct omap_lcd_config *conf;

	conf = omap_get_config(OMAP_TAG_LCD, struct omap_lcd_config);
	if (conf != NULL) {
		if (strcmp(conf->panel_name, "lph8923") == 0) {
			tsc2301_config.ts_x_plate_ohm	= 180;
			tsc2301_config.ts_hw_avg	= 8;
			tsc2301_config.ts_max_pressure	= 2048;
			tsc2301_config.ts_touch_pressure = 400;
			tsc2301_config.ts_stab_time	= 100;
			tsc2301_config.ts_pressure_fudge = 2;
			tsc2301_config.ts_x_max		= 4096;
			tsc2301_config.ts_x_fudge	= 4;
			tsc2301_config.ts_y_max		= 4096;
			tsc2301_config.ts_y_fudge	= 7;
		} else if (strcmp(conf->panel_name, "ls041y3") == 0) {
			tsc2301_config.ts_x_plate_ohm	= 280;
			tsc2301_config.ts_hw_avg	= 8;
			tsc2301_config.ts_touch_pressure = 400;
			tsc2301_config.ts_max_pressure	= 2048;
			tsc2301_config.ts_stab_time	= 1000;
			tsc2301_config.ts_pressure_fudge = 2;
			tsc2301_config.ts_x_max		= 4096;
			tsc2301_config.ts_x_fudge	= 4;
			tsc2301_config.ts_y_max		= 4096;
			tsc2301_config.ts_y_fudge	= 7;
		} else {
			printk(KERN_ERR "Unknown panel type, set default "
			       "touchscreen configuration\n");
			tsc2301_config.ts_x_plate_ohm	= 200;
			tsc2301_config.ts_stab_time	= 100;
		}
	}
}
#else
static inline void n800_ts_set_config(void)
{
}
#endif

#if defined(CONFIG_CBUS_RETU_HEADSET)
static struct platform_device retu_headset_device = {
	.name	= "retu-headset",
	.id	= -1,
};
#endif

static struct platform_device *n800_devices[] __initdata = {
#if defined(CONFIG_CBUS_RETU) && defined(CONFIG_LEDS_OMAP_PWM)
	&n800_keypad_led_device,
#endif
#if defined(CONFIG_CBUS_RETU_HEADSET)
	&retu_headset_device,
#endif
};

#ifdef CONFIG_MENELAUS
static int n800_auto_sleep_regulators(void)
{
	u32 val;
	int ret;

	val = EN_VPLL_SLEEP | EN_VMMC_SLEEP    \
		| EN_VAUX_SLEEP | EN_VIO_SLEEP \
		| EN_VMEM_SLEEP | EN_DC3_SLEEP \
		| EN_VC_SLEEP | EN_DC2_SLEEP;

	ret = menelaus_set_regulator_sleep(1, val);
	if (ret < 0) {
		printk(KERN_ERR "Could not set regulators to sleep on "
			"menelaus: %u\n", ret);
		return ret;
	}
	return 0;
}

static int n800_auto_voltage_scale(void)
{
	int ret;

	ret = menelaus_set_vcore_hw(1400, 1050);
	if (ret < 0) {
		printk(KERN_ERR "Could not set VCORE voltage on "
			"menelaus: %u\n", ret);
		return ret;
	}
	return 0;
}

static int n800_menelaus_init(struct device *dev)
{
	int ret;

	ret = n800_auto_voltage_scale();
	if (ret < 0)
		return ret;
	ret = n800_auto_sleep_regulators();
	if (ret < 0)
		return ret;
	return 0;
}

static struct menelaus_platform_data n800_menelaus_platform_data = {
	.late_init = n800_menelaus_init,
};
#endif

static struct i2c_board_info __initdata n800_i2c_board_info_1[] = {
	{
		I2C_BOARD_INFO("menelaus", 0x72),
		.irq = INT_24XX_SYS_NIRQ,
		.platform_data = &n800_menelaus_platform_data,
	},
};

extern struct tcm825x_platform_data n800_tcm825x_platform_data;

static struct i2c_board_info __initdata_or_module n8x0_i2c_board_info_2[] = {
	{
		I2C_BOARD_INFO(TCM825X_NAME, TCM825X_I2C_ADDR),
#if defined (CONFIG_VIDEO_TCM825X) || defined (CONFIG_VIDEO_TCM825X_MODULE)
		.platform_data = &n800_tcm825x_platform_data,
#endif
	},
};


static struct i2c_board_info __initdata_or_module n800_i2c_board_info_2[] = {
	{
		I2C_BOARD_INFO("tea5761", 0x10),
	},
};

static struct i2c_board_info __initdata_or_module n810_i2c_board_info_2[] = {
	{
		I2C_BOARD_INFO("lm8323", 0x45),
		.irq		= OMAP_GPIO_IRQ(109),
	},
	{
		I2C_BOARD_INFO("tsl2563", 0x29),
	},
	{
		I2C_BOARD_INFO("lp5521", 0x32),
	},
};

#if defined(CONFIG_MTD_ONENAND_OMAP2) || defined(CONFIG_MTD_ONENAND_OMAP2_MODULE)

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

static struct omap_onenand_platform_data board_onenand_data = {
	.cs		= 0,
	.gpio_irq	= 26,
	.parts		= onenand_partitions,
	.nr_parts	= ARRAY_SIZE(onenand_partitions),
	.flags		= ONENAND_SYNC_READ,
};

static void __init board_onenand_init(void)
{
	gpmc_onenand_init(&board_onenand_data);
}

#endif

void __init nokia_n800_common_init(void)
{
	platform_add_devices(n800_devices, ARRAY_SIZE(n800_devices));

	n800_mmc_init();
	n800_dsp_init();
	n800_usb_init();
	n800_cam_init();
	if (machine_is_nokia_n800())
		spi_register_board_info(n800_spi_board_info,
				ARRAY_SIZE(n800_spi_board_info));
	if (machine_is_nokia_n810()) {
		spi_register_board_info(n810_spi_board_info,
				ARRAY_SIZE(n810_spi_board_info));
	}
	omap_serial_init();
	omap_register_i2c_bus(1, 400, n800_i2c_board_info_1,
			      ARRAY_SIZE(n800_i2c_board_info_1));
	omap_register_i2c_bus(2, 400, n8x0_i2c_board_info_2,
			      ARRAY_SIZE(n8x0_i2c_board_info_2));
	if (machine_is_nokia_n800())
		i2c_register_board_info(2, n800_i2c_board_info_2,
			ARRAY_SIZE(n800_i2c_board_info_2));
	if (machine_is_nokia_n810())
		i2c_register_board_info(2, n810_i2c_board_info_2,
			ARRAY_SIZE(n810_i2c_board_info_2));
		
	mipid_dev_init();
	blizzard_dev_init();
	board_onenand_init();
}

static void __init nokia_n800_init(void)
{
	nokia_n800_common_init();

	n800_ts_set_config();
	tea5761_dev_init();
	board_onenand_init();
}

void __init nokia_n800_map_io(void)
{
	omap_board_config = n800_config;
	omap_board_config_size = ARRAY_SIZE(n800_config);

	omap2_set_globals_242x();
	omap2_map_common_io();
}

MACHINE_START(NOKIA_N800, "Nokia N800")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= nokia_n800_map_io,
	.init_irq	= nokia_n800_init_irq,
	.init_machine	= nokia_n800_init,
	.timer		= &omap_timer,
MACHINE_END
