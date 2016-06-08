/*
 * TLC59108 DPI Panel Driver
 *
 * Copyright (C) 2013 Texas Instruments
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/of_device.h>

#include <video/of_display_timing.h>

#include "../dss/omapdss.h"

#define TLC_NAME		"tlc59108"
#define TLC_I2C_ADDR		0x40

#define TLC59108_MODE1		0x00
#define TLC59108_PWM2		0x04
#define TLC59108_LEDOUT0	0x0c
#define TLC59108_LEDOUT1	0x0d

struct panel_drv_data {
	struct omap_dss_device dssdev;
	struct omap_dss_device *in;

	struct omap_video_timings videomode;

	struct gpio_desc *enable_gpio;
	struct regmap *regmap;

	const struct tlc_board_data *board_data;
};

struct tlc_board_data {
	struct omap_video_timings timings;
	const unsigned int *init_seq;
	unsigned init_seq_len;
};

static const unsigned int tlc_10_inch_init_seq[] = {
	/* Init the TLC chip */
	TLC59108_MODE1, 0x01,
	/* LDR0: ON, LDR1: OFF, LDR2: PWM, LDR3: OFF */
	TLC59108_LEDOUT0, 0x21,
	/* Set LED2 PWM to full */
	TLC59108_PWM2, 0xff,
	/* LDR4: OFF, LDR5: OFF, LDR6: OFF, LDR7: ON */
	TLC59108_LEDOUT1, 0x40,
};

static const struct tlc_board_data tlc_10_inch_data = {
	.timings = {
		.x_res          = 1280,
		.y_res          = 800,

		.pixelclock     = 69300404,

		.hfp            = 48,
		.hsw            = 32,
		.hbp            = 44,

		.vfp            = 4,
		.vsw            = 7,
		.vbp            = 12,

		.vsync_level    = OMAPDSS_SIG_ACTIVE_LOW,
		.hsync_level    = OMAPDSS_SIG_ACTIVE_LOW,
		.data_pclk_edge = OMAPDSS_DRIVE_SIG_RISING_EDGE,
		.de_level       = OMAPDSS_SIG_ACTIVE_HIGH,
		.sync_pclk_edge = OMAPDSS_DRIVE_SIG_RISING_EDGE,
	},
	.init_seq = tlc_10_inch_init_seq,
	.init_seq_len = ARRAY_SIZE(tlc_10_inch_init_seq),
};

static int tlc_init(struct panel_drv_data *ddata)
{
	struct regmap *map = ddata->regmap;
	unsigned i, len;
	const unsigned int *seq;

	len = ddata->board_data->init_seq_len;
	seq = ddata->board_data->init_seq;

	for (i = 0; i < len; i += 2)
		regmap_write(map, seq[i], seq[i + 1]);

	return 0;
}

static int tlc_uninit(struct panel_drv_data *ddata)
{
	struct regmap *map = ddata->regmap;

	/* clear TLC chip regs */
	regmap_write(map, TLC59108_PWM2, 0x0);
	regmap_write(map, TLC59108_LEDOUT0, 0x0);
	regmap_write(map, TLC59108_LEDOUT1, 0x0);

	regmap_write(map, TLC59108_MODE1, 0x0);

	return 0;
}

#define to_panel_data(p) container_of(p, struct panel_drv_data, dssdev)

static int panel_dpi_connect(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	int r;

	if (omapdss_device_is_connected(dssdev))
		return 0;

	r = in->ops.dpi->connect(in, dssdev);
	if (r)
		return r;

	return 0;
}

static void panel_dpi_disconnect(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	if (!omapdss_device_is_connected(dssdev))
		return;

	in->ops.dpi->disconnect(in, dssdev);
}

static int panel_dpi_enable(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;
	int r;

	if (!omapdss_device_is_connected(dssdev))
		return -ENODEV;

	if (omapdss_device_is_enabled(dssdev))
		return 0;

	in->ops.dpi->set_timings(in, &ddata->videomode);

	r = in->ops.dpi->enable(in);
	if (r)
		return r;

	tlc_init(ddata);

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	return 0;
}

static void panel_dpi_disable(struct omap_dss_device *dssdev)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	if (!omapdss_device_is_enabled(dssdev))
		return;

	tlc_uninit(ddata);

	in->ops.dpi->disable(in);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static void panel_dpi_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	ddata->videomode = *timings;
	dssdev->panel.timings = *timings;

	in->ops.dpi->set_timings(in, timings);
}

static void panel_dpi_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);

	*timings = ddata->videomode;
}

static int panel_dpi_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	struct panel_drv_data *ddata = to_panel_data(dssdev);
	struct omap_dss_device *in = ddata->in;

	return in->ops.dpi->check_timings(in, timings);
}

static struct omap_dss_driver panel_dpi_ops = {
	.connect	= panel_dpi_connect,
	.disconnect	= panel_dpi_disconnect,

	.enable		= panel_dpi_enable,
	.disable	= panel_dpi_disable,

	.set_timings	= panel_dpi_set_timings,
	.get_timings	= panel_dpi_get_timings,
	.check_timings	= panel_dpi_check_timings,

	.get_resolution	= omapdss_default_get_resolution,
};

static const struct of_device_id tlc59108_of_match[] = {
	{
		.compatible = "omapdss,ti,tlc59108-lp101",
		.data = &tlc_10_inch_data,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, tlc59108_of_match);

static int tlc_probe_of(struct device *dev)
{
	struct panel_drv_data *ddata = dev_get_drvdata(dev);
	struct device_node *np = dev->of_node;
	const struct of_device_id *of_dev_id;
	struct gpio_desc *gpio;

	gpio = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(gpio))
		return PTR_ERR(gpio);
	ddata->enable_gpio = gpio;

	ddata->in = omapdss_of_find_source_for_first_ep(np);
	if (IS_ERR(ddata->in)) {
		dev_err(dev, "failed to find video source\n");
		return PTR_ERR(ddata->in);
	}

	of_dev_id = of_match_device(tlc59108_of_match, dev);
	if (!of_dev_id) {
		dev_err(dev, "Unable to match device\n");
		return -ENODEV;
	}

	ddata->board_data = of_dev_id->data;
	ddata->videomode = ddata->board_data->timings;

	return 0;
}

struct regmap_config tlc59108_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int tlc59108_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int r;
	struct regmap *regmap;
	struct panel_drv_data *ddata;
	struct device *dev = &client->dev;
	struct omap_dss_device *dssdev;
	unsigned int val;

	ddata = devm_kzalloc(dev, sizeof(*ddata), GFP_KERNEL);
	if (ddata == NULL)
		return -ENOMEM;

	dev_set_drvdata(dev, ddata);

	r = tlc_probe_of(dev);
	if (r)
		return r;

	regmap = devm_regmap_init_i2c(client, &tlc59108_regmap_config);
	if (IS_ERR(regmap)) {
		r = PTR_ERR(regmap);
		dev_err(dev, "Failed to init regmap: %d\n", r);
		goto err_gpio;
	}

	ddata->regmap = regmap;

	usleep_range(10000, 15000);

	/* Try to read a TLC register to verify if i2c works */
	r = regmap_read(ddata->regmap, TLC59108_MODE1, &val);
	if (r < 0) {
		dev_err(dev, "Failed to set MODE1: %d\n", r);
		goto err_read;
	}

	dssdev = &ddata->dssdev;
	dssdev->dev = dev;
	dssdev->driver = &panel_dpi_ops;
	dssdev->type = OMAP_DISPLAY_TYPE_DPI;
	dssdev->owner = THIS_MODULE;
	dssdev->panel.timings = ddata->videomode;

	r = omapdss_register_display(dssdev);
	if (r) {
		dev_err(dev, "Failed to register panel\n");
		goto err_reg;
	}

	dev_info(dev, "Successfully initialized %s\n", TLC_NAME);

	return 0;
err_reg:
err_read:
err_gpio:
	omap_dss_put_device(ddata->in);
	return r;
}

static int tlc59108_i2c_remove(struct i2c_client *client)
{
	struct panel_drv_data *ddata = dev_get_drvdata(&client->dev);
	struct omap_dss_device *dssdev = &ddata->dssdev;
	struct omap_dss_device *in = ddata->in;

	if (ddata->enable_gpio)
		gpiod_set_value_cansleep(ddata->enable_gpio, 0);

	omapdss_unregister_display(dssdev);

	panel_dpi_disable(dssdev);
	panel_dpi_disconnect(dssdev);

	omap_dss_put_device(in);

	return 0;
}

static const struct i2c_device_id tlc59108_id[] = {
	{ TLC_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tlc59108_id);

static struct i2c_driver tlc59108_i2c_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= TLC_NAME,
		.of_match_table = tlc59108_of_match,
	},
	.id_table	= tlc59108_id,
	.probe		= tlc59108_i2c_probe,
	.remove		= tlc59108_i2c_remove,
};

module_i2c_driver(tlc59108_i2c_driver);

MODULE_AUTHOR("Archit Taneja  <archit@ti.com>");
MODULE_DESCRIPTION("TLC-59108 DPI Panel Driver");
MODULE_LICENSE("GPL");
