/*
 * linux/arch/arm/mach-omap2/board-n800.c
 *
 * Copyright (C) 2005 Nokia Corporation
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
#include <linux/spi/tsc2301.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/arch/gpio.h>
#include <asm/arch/usb.h>
#include <asm/arch/board.h>
#include <asm/arch/common.h>
#include <asm/arch/mcspi.h>
#include <asm/arch/menelaus.h>
#include <asm/arch/lcd_mipid.h>
#include <asm/arch/clock.h>
#include <asm/arch/gpio-switch.h>
#include <asm/arch/omapfb.h>
#include <asm/arch/blizzard.h>

#include <../drivers/cbus/tahvo.h>

#define N800_BLIZZARD_POWERDOWN_GPIO 15
#define N800_STI_GPIO		62
#define N800_CAM_SENSOR_RESET_GPIO	53
#define N800_KEYB_IRQ_GPIO		109

static void __init nokia_n800_init_irq(void)
{
	omap2_init_common_hw();
	omap_init_irq();
	omap_gpio_init();

#ifdef CONFIG_OMAP_STI
	if (omap_request_gpio(N800_STI_GPIO) < 0) {
		printk(KERN_ERR "Failed to request GPIO %d for STI\n",
		       N800_STI_GPIO);
		return;
	}

	omap_set_gpio_direction(N800_STI_GPIO, 0);
	omap_set_gpio_dataout(N800_STI_GPIO, 0);
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
		omap_set_gpio_dataout(pdata->nreset_gpio, 0);
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
	omap_set_gpio_dataout(N800_BLIZZARD_POWERDOWN_GPIO, 1);
}

static void blizzard_power_down(struct device *dev)
{
	omap_set_gpio_dataout(N800_BLIZZARD_POWERDOWN_GPIO, 0);
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

	r = omap_request_gpio(N800_BLIZZARD_POWERDOWN_GPIO);
	if (r < 0)
		return;
	omap_set_gpio_direction(N800_BLIZZARD_POWERDOWN_GPIO, 0);
	omap_set_gpio_dataout(N800_BLIZZARD_POWERDOWN_GPIO, 1);

	blizzard_get_clocks();
	omapfb_set_ctrl_platform_data(&n800_blizzard_data);
}

#if defined(CONFIG_CBUS_RETU) && defined(CONFIG_VIDEO_CAMERA_SENSOR_TCM825X) && \
	defined(CONFIG_MENELAUS)
#define SUPPORT_SENSOR
#endif

#ifdef SUPPORT_SENSOR

static int sensor_okay;

/*
 * VSIM1	--> CAM_IOVDD	--> IOVDD (1.8 V)
 */
static int tcm825x_sensor_power_on(void *data)
{
	int ret;

	if (!sensor_okay)
		return -ENODEV;

	/* Set VMEM to 1.5V and VIO to 2.5V */
	ret = menelaus_set_vmem(1500);
	if (ret < 0) {
		/* Try once more, it seems the sensor power up causes
		 * some problems on the I2C bus. */
		ret = menelaus_set_vmem(1500);
		if (ret < 0)
			return ret;
	}
	msleep(1);

	ret = menelaus_set_vio(2500);
	if (ret < 0)
		return ret;

	/* Set VSim1 on */
	retu_write_reg(RETU_REG_CTRL_SET, 0x0080);
	msleep(100);

	omap_set_gpio_dataout(N800_CAM_SENSOR_RESET_GPIO, 1);
	msleep(1);

	return 0;
}

static int tcm825x_sensor_power_off(void * data)
{
	int ret;

	omap_set_gpio_dataout(N800_CAM_SENSOR_RESET_GPIO, 0);
	msleep(1);

	/* Set VSim1 off */
	retu_write_reg(RETU_REG_CTRL_CLR, 0x0080);
	msleep(1);

	/* Set VIO_MODE to off */
	ret = menelaus_set_vio(0);
	if (ret < 0)
		return ret;
	msleep(1);

	/* Set VMEM_MODE to off */
	ret = menelaus_set_vmem(0);
	if (ret < 0)
		return ret;
	msleep(1);

	return 0;
}

static struct omap_camera_sensor_config n800_sensor_config = {
	.power_on   = tcm825x_sensor_power_on,
	.power_off  = tcm825x_sensor_power_off,
};

static void __init n800_cam_init(void)
{
	int r;

	r = omap_request_gpio(N800_CAM_SENSOR_RESET_GPIO);
	if (r < 0)
		return;

	omap_set_gpio_dataout(N800_CAM_SENSOR_RESET_GPIO, 0);
	omap_set_gpio_direction(N800_CAM_SENSOR_RESET_GPIO, 0);

	sensor_okay = 1;
}

#else

static inline void n800_cam_init(void) {}

#endif

static struct omap_board_config_kernel n800_config[] __initdata = {
	{ OMAP_TAG_UART,	                &n800_uart_config },
#ifdef SUPPORT_SENSOR
	{ OMAP_TAG_CAMERA_SENSOR,		&n800_sensor_config },
#endif
	{ OMAP_TAG_FBMEM,			&n800_fbmem0_config },
	{ OMAP_TAG_FBMEM,			&n800_fbmem1_config },
	{ OMAP_TAG_FBMEM,			&n800_fbmem2_config },
	{ OMAP_TAG_TMP105,			&n800_tmp105_config },
};


static int n800_get_keyb_irq_state(struct device *dev)
{
	return !omap_get_gpio_datain(N800_KEYB_IRQ_GPIO);
}

static struct tsc2301_platform_data tsc2301_config = {
	.reset_gpio	= 118,
	.dav_gpio	= 103,
	.pen_int_gpio	= 106,
	.keymap = {
		-1,		/* Event for bit 0 */
		KEY_UP,		/* Event for bit 1 (up) */
		KEY_F5,		/* Event for bit 2 (home) */
		-1,		/* Event for bit 3 */
		KEY_LEFT,	/* Event for bit 4 (left) */
		KEY_ENTER,	/* Event for bit 5 (enter) */
		KEY_RIGHT,	/* Event for bit 6 (right) */
		-1,		/* Event for bit 7 */
		KEY_ESC,	/* Event for bit 8 (cycle) */
		KEY_DOWN,	/* Event for bit 9 (down) */
		KEY_F4,		/* Event for bit 10 (menu) */
		-1,		/* Event for bit 11 */
		KEY_F8,		/* Event for bit 12 (Zoom-) */
		KEY_F6,		/* Event for bit 13 (FS) */
		KEY_F7,		/* Event for bit 14 (Zoom+) */
		-1,		/* Event for bit 15 */
	},
	.kp_rep 	= 0,
	.get_keyb_irq_state = n800_get_keyb_irq_state,
};

static void tsc2301_dev_init(void)
{
	int gpio = N800_KEYB_IRQ_GPIO;

	if (omap_request_gpio(gpio) < 0) {
		printk(KERN_ERR "can't get KBIRQ GPIO\n");
		return;
	}
	omap_set_gpio_direction(gpio, 1);
	tsc2301_config.keyb_int = OMAP_GPIO_IRQ(gpio);
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
	[0] = {
		.modalias	= "lcd_mipid",
		.bus_num	= 1,
		.chip_select	= 1,
		.max_speed_hz	= 4000000,
		.controller_data= &mipid_mcspi_config,
		.platform_data	= &n800_mipid_platform_data,
	}, [1] = {
		.modalias	= "cx3110x",
		.bus_num	= 2,
		.chip_select	= 0,
		.max_speed_hz   = 48000000,
		.controller_data= &cx3110x_mcspi_config,
	}, [2] = {
		.modalias	= "tsc2301",
		.bus_num	= 1,
		.chip_select	= 0,
		.max_speed_hz   = 6000000,
		.controller_data= &tsc2301_mcspi_config,
		.platform_data  = &tsc2301_config,
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
			tsc2301_config.ts_hw_avg	= 4;
			tsc2301_config.ts_ignore_last	= 1;
			tsc2301_config.ts_max_pressure	= 255;
			tsc2301_config.ts_stab_time	= 100;
		} else if (strcmp(conf->panel_name, "ls041y3") == 0) {
			tsc2301_config.ts_x_plate_ohm	= 280;
			tsc2301_config.ts_hw_avg	= 16;
			tsc2301_config.ts_touch_pressure= 215;
			tsc2301_config.ts_max_pressure	= 255;
			tsc2301_config.ts_ignore_last	= 1;
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

static struct omap_gpio_switch n800_gpio_switches[] __initdata = {
	{
		.name			= "bat_cover",
		.gpio			= -1,
		.debounce_rising	= 100,
		.debounce_falling	= 0,
		.notify			= n800_mmc_slot1_cover_handler,
		.notify_data		= NULL,
	}, {
		.name			= "headphone",
		.gpio			= -1,
		.debounce_rising	= 200,
		.debounce_falling	= 200,
	}, {
		.name			= "cam_act",
		.gpio			= -1,
		.debounce_rising	= 200,
		.debounce_falling	= 200,
	}, {
		.name			= "cam_turn",
		.gpio			= -1,
		.debounce_rising	= 100,
		.debounce_falling	= 100,
	},
};

static struct platform_device *n800_devices[] __initdata = {
#if defined(CONFIG_CBUS_RETU) && defined(CONFIG_LEDS_OMAP_PWM)
	&n800_keypad_led_device,
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

static struct i2c_board_info __initdata n800_i2c_board_info[] = {
#ifdef CONFIG_MENELAUS
	{
		I2C_BOARD_INFO("menelaus", 0x72),
		.irq = INT_24XX_SYS_NIRQ,
		.platform_data = &n800_menelaus_platform_data,
	},
#endif
};

static void __init nokia_n800_init(void)
{
	platform_add_devices(n800_devices, ARRAY_SIZE(n800_devices));

	i2c_register_board_info(1, n800_i2c_board_info,
			ARRAY_SIZE(n800_i2c_board_info));

	n800_flash_init();
	n800_mmc_init();
	n800_bt_init();
	n800_audio_init(&tsc2301_config);
	n800_dsp_init();
	n800_usb_init();
	n800_cam_init();
	n800_ts_set_config();
	spi_register_board_info(n800_spi_board_info,
				ARRAY_SIZE(n800_spi_board_info));
	omap_serial_init();
	mipid_dev_init();
	blizzard_dev_init();
	tsc2301_dev_init();
	omap_register_gpio_switches(n800_gpio_switches,
				    ARRAY_SIZE(n800_gpio_switches));
}

static void __init nokia_n800_map_io(void)
{
	omap_board_config = n800_config;
	omap_board_config_size = ARRAY_SIZE(n800_config);

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
