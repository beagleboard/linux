// SPDX-License-Identifier: GPL-2.0-only
/*
 * mipi_dsi.h - MIPI dsi module
 *
 * Copyright (c) 2020 Seeed Studio
 *
 * I2C slave address: 0x45
 */

#ifndef __MIPI_DSI_H__
#define __MIPI_DSI_H__


#include <linux/interrupt.h>
#include <linux/bitops.h>
#include <linux/slab.h>
#include <linux/string.h>

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/pm.h>

#include <drm/drm_crtc.h>
#include <drm/drm_device.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_modes.h>

#include <video/mipi_display.h>

#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>


#ifdef I2C_DSI_DBG
#define DBG_FUNC(format, x...)		printk(KERN_INFO "[DSI]%s:" format"\n", __func__, ##x)
#define DBG_PRINT(format, x...)		printk(KERN_INFO "[DSI]" format"\n", ##x)
#else
#define DBG_FUNC(format, x...)
#define DBG_PRINT(format, x...)
#endif

#define DSI_DRIVER_NAME		        "i2c_mipi_dsi"

/* i2c: commands */
enum REG_ADDR {
	REG_ID = 0x80,
	REG_PORTA,	/* BIT(2) for horizontal flip, BIT(3) for vertical flip */
	REG_PORTB,  // --
	REG_PORTC,
	REG_PORTD,
	REG_POWERON,// --
	REG_PWM,    // --
	REG_DDRA,
	REG_DDRB,
	REG_DDRC,
	REG_DDRD,
	REG_TEST,
	REG_WR_ADDRL,
	REG_WR_ADDRH,
	REG_READH,
	REG_READL,
	REG_WRITEH,
	REG_WRITEL,
	REG_ID2,

	REG_LCD_RST,
	REG_TP_RST,
	REG_TP_STATUS,
	REG_TP_POINT,
	REG_TP_VERSION,
	REG_ADC1,
	REG_ADC2,
	REG_MCU_AUTO_RESET,

	REG_MAX
};

#define DSI_DCS_WRITE(dsi, seq...)		\
	{									\
		int ret = 0;					\
		const u8 d[] = { seq };			\
		ret = mipi_dsi_dcs_write_buffer(dsi, d, ARRAY_SIZE(d));	\
		if (ret < 0)		\
			return ret;		\
	}

struct panel_data {
	void (*set_dsi)(struct mipi_dsi_device *dsi);
	const struct drm_panel_funcs *funcs;
};

struct i2c_mipi_dsi {
	struct i2c_client *i2c;
	struct mutex mutex;

	// panel
	struct drm_panel panel;
	struct panel_data *panel_data;

	// dsi
	struct mipi_dsi_device *dsi;

	// tp
	struct input_dev *input;
	struct touchscreen_properties prop;
	uint32_t tp_point_rotate;

	// backlight
	int brightness;

	// mcu auto reset enable when the tp driver is not working
	uint32_t mcu_auto_reset;
};
#define panel_to_md(_p)	container_of(_p, struct i2c_mipi_dsi, panel)

static int i2c_md_read(struct i2c_mipi_dsi *md, u8 reg, u8 *buf, int len);

#endif /*End of header guard macro */
