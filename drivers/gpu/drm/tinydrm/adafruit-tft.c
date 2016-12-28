#define DEBUG

/*
 * DRM driver for Adafruit MIPI compatible SPI TFT displays
 *
 * Copyright 2016 Noralf Trønnes
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <drm/tinydrm/hx8340.h>
#include <drm/tinydrm/mipi-dbi.h>
#include <drm/tinydrm/st7735r.h>
#include <drm/tinydrm/tinydrm.h>
#include <drm/tinydrm/tinydrm-helpers.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <video/mipi_display.h>

struct adafruit_tft_display {
	const struct drm_display_mode mode;
	const struct drm_simple_display_pipe_funcs funcs;
	bool write_only;
	bool dc;
};

enum adafruit_tft_display_ids {
	ADAFRUIT_797,
	ADAFRUIT_358,
};

/*
 * 2.2" Color TFT LCD display - HX8340BN, 9-bit mode (#797)
 * Product: http://www.adafruit.com/products/797
 * Schematics: https://github.com/adafruit/Adafruit-2.2-SPI-TFT
 *
 * It's hard to tell if it should be possible to read from the controller.
 * The datasheet says that SDI is used for input and output, but
 * the schematics indicate that MISO is connected.
 * One user reports a failed attempt to read from it. So err on the safe
 * side and mark it is as write-only in case it tries to drive MOSI.
 *
 * Init sequence taken from the BTL221722-276L datasheet
 */
static void adafruit_tft_797_enable(struct drm_simple_display_pipe *pipe,
				    struct drm_crtc_state *crtc_state)
{
	struct tinydrm_device *tdev = pipe_to_tinydrm(pipe);
	struct mipi_dbi *mipi = mipi_dbi_from_tinydrm(tdev);
	struct device *dev = tdev->drm.dev;
	struct regmap *reg = mipi->reg;
	u8 addr_mode;
	int ret;

	dev_dbg(dev, "%s\n", __func__);

	mutex_lock(&tdev->dev_lock);

	if (tdev->prepared)
		goto out_unlock;

	if (mipi->regulator) {
		ret = regulator_enable(mipi->regulator);
		if (ret) {
			dev_err(dev, "Failed to enable regulator %d\n", ret);
			goto out_unlock;
		}
	}

	mipi_dbi_hw_reset(mipi);
	ret = mipi_dbi_write(reg, HX8340_SETEXTCMD, 0xFF, 0x83, 0x40);
	if (ret) {
		dev_err(dev, "Error writing command %d\n", ret);
		goto out_unlock;
	}

	mipi_dbi_write(reg, MIPI_DCS_EXIT_SLEEP_MODE);
	msleep(150);

	/* Undocumented register */
	mipi_dbi_write(reg, 0xCA, 0x70, 0x00, 0xD9);

	mipi_dbi_write(reg, HX8340_SETOSC, 0x01, 0x11);

	/* Driving ability Setting */
	mipi_dbi_write(reg, 0xC9,
		       0x90, 0x49, 0x10, 0x28, 0x28, 0x10, 0x00, 0x06);
	msleep(20);

	/* Gamma 2.2 Setting */
	mipi_dbi_write(reg, HX8340_SETGAMMAP,
		       0x60, 0x71, 0x01, 0x0E, 0x05, 0x02, 0x09, 0x31, 0x0A);
	mipi_dbi_write(reg, HX8340_SETGAMMAN,
		       0x67, 0x30, 0x61, 0x17, 0x48, 0x07, 0x05, 0x33);
	msleep(10);

	mipi_dbi_write(reg, HX8340_SETPWCTR5, 0x35, 0x20, 0x45);
	mipi_dbi_write(reg, HX8340_SETPWCTR4, 0x33, 0x25, 0x4C);
	msleep(10);

	mipi_dbi_write(reg, MIPI_DCS_SET_PIXEL_FORMAT, 0x05);

	switch (mipi->rotation) {
	default:
		addr_mode = 0;
		break;
	case 90:
		addr_mode = HX8340_MADCTL_MV | HX8340_MADCTL_MY;
		break;
	case 180:
		addr_mode = HX8340_MADCTL_MY;
		break;
	case 270:
		addr_mode = HX8340_MADCTL_MX | HX8340_MADCTL_MV;
		break;
	}
	addr_mode |= HX8340_MADCTL_BGR;
	mipi_dbi_write(reg, MIPI_DCS_SET_ADDRESS_MODE, addr_mode);

	mipi_dbi_write(reg, MIPI_DCS_SET_DISPLAY_ON);
	msleep(50);

	tdev->prepared = true;
	if (pipe->plane.state->fb)
		schedule_work(&tdev->dirty_work);

out_unlock:
	mutex_unlock(&tdev->dev_lock);
}

/*
 * 1.8" Color TFT LCD display - ST7735R (#358)
 * Product: https://www.adafruit.com/products/358
 *
 * Init sequence taken from the Adafruit-ST7735-Library (Black Tab)
 */
static void adafruit_tft_358_enable(struct drm_simple_display_pipe *pipe,
				    struct drm_crtc_state *crtc_state)
{
	struct tinydrm_device *tdev = pipe_to_tinydrm(pipe);
	struct mipi_dbi *mipi = mipi_dbi_from_tinydrm(tdev);
	struct device *dev = tdev->drm.dev;
	struct regmap *reg = mipi->reg;
	u8 addr_mode;
	int ret;

	dev_dbg(dev, "%s\n", __func__);

	mutex_lock(&tdev->dev_lock);

	if (tdev->prepared)
		goto out_unlock;

	if (mipi->regulator) {
		ret = regulator_enable(mipi->regulator);
		if (ret) {
			dev_err(dev, "Failed to enable regulator %d\n", ret);
			goto out_unlock;
		}
	}

	mipi_dbi_hw_reset(mipi);
	ret = mipi_dbi_write(reg, MIPI_DCS_SOFT_RESET);
	if (ret) {
		dev_err(dev, "Error writing command %d\n", ret);
		goto out_unlock;
	}

	msleep(150);

	mipi_dbi_write(reg, MIPI_DCS_EXIT_SLEEP_MODE);
	msleep(500);

	mipi_dbi_write(reg, ST7735R_FRMCTR1, 0x01, 0x2C, 0x2D);
	mipi_dbi_write(reg, ST7735R_FRMCTR2, 0x01, 0x2C, 0x2D);
	mipi_dbi_write(reg, ST7735R_FRMCTR3,
		       0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D);
	mipi_dbi_write(reg, ST7735R_INVCTR, 0x07);

	mipi_dbi_write(reg, ST7735R_PWCTR1, 0xA2, 0x02, 0x84);
	mipi_dbi_write(reg, ST7735R_PWCTR2, 0xC5);
	mipi_dbi_write(reg, ST7735R_PWCTR3, 0x0A, 0x00);
	mipi_dbi_write(reg, ST7735R_PWCTR4, 0x8A, 0x2A);
	mipi_dbi_write(reg, ST7735R_PWCTR5, 0x8A, 0xEE);

	mipi_dbi_write(reg, ST7735R_VMCTR1, 0x0E);
	mipi_dbi_write(reg, MIPI_DCS_EXIT_INVERT_MODE);

	switch (mipi->rotation) {
	default:
		addr_mode = ST7735R_MADCTL_MX | ST7735R_MADCTL_MY;
		break;
	case 90:
		addr_mode = ST7735R_MADCTL_MX | ST7735R_MADCTL_MV;
		break;
	case 180:
		addr_mode = 0;
		break;
	case 270:
		addr_mode = ST7735R_MADCTL_MY | ST7735R_MADCTL_MV;
		break;
	}
	mipi_dbi_write(reg, MIPI_DCS_SET_ADDRESS_MODE, addr_mode);

	mipi_dbi_write(reg, MIPI_DCS_SET_PIXEL_FORMAT, 0x05);

	mipi_dbi_write(reg, ST7735R_GAMCTRP1,
		       0x02, 0x1c, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2d,
		       0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10);
	mipi_dbi_write(reg, ST7735R_GAMCTRN1,
		       0x03, 0x1d, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D,
		       0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10);

	mipi_dbi_write(reg, MIPI_DCS_ENTER_NORMAL_MODE);
	msleep(20);

	mipi_dbi_write(reg, MIPI_DCS_SET_DISPLAY_ON);
	msleep(100);

	tdev->prepared = true;
	if (pipe->plane.state->fb)
		schedule_work(&tdev->dirty_work);

out_unlock:
	mutex_unlock(&tdev->dev_lock);
}

static const struct adafruit_tft_display adafruit_tft_displays[] = {
	[ADAFRUIT_797] = {
		.mode = {
			TINYDRM_MODE(176, 220, 34, 43),
		},
		.funcs = {
			.enable = adafruit_tft_797_enable,
			.disable = mipi_dbi_pipe_disable,
			.update = tinydrm_display_pipe_update,
		},
		.write_only = true,
	},
	[ADAFRUIT_358] = {
		.mode = {
			TINYDRM_MODE(128, 160, 28, 35),
		},
		.funcs = {
			.enable = adafruit_tft_358_enable,
			.disable = mipi_dbi_pipe_disable,
			.update = tinydrm_display_pipe_update,
		},
		.dc = true,
		.write_only = true,
	},
};

static const struct of_device_id adafruit_tft_of_match[] = {
	{ .compatible = "adafruit,tft797",  .data = (void *)ADAFRUIT_797 },
	{ .compatible = "adafruit,tft358",  .data = (void *)ADAFRUIT_358 },
	{},
};
MODULE_DEVICE_TABLE(of, adafruit_tft_of_match);

static const struct spi_device_id adafruit_tft_id[] = {
	{ "tft797",  ADAFRUIT_797 },
	{ "tft358",  ADAFRUIT_358 },
	{ },
};
MODULE_DEVICE_TABLE(spi, adafruit_tft_id);

static struct drm_driver adafruit_tft_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_PRIME |
				  DRIVER_ATOMIC,
	TINYDRM_GEM_DRIVER_OPS,
	.lastclose		= tinydrm_lastclose,
	.debugfs_init		= mipi_dbi_debugfs_init,
	.debugfs_cleanup	= mipi_dbi_debugfs_cleanup,
	.name			= "adafruit-tft",
	.desc			= "Adafruit TFT",
	.date			= "20160317",
	.major			= 1,
	.minor			= 0,
};

static int adafruit_tft_probe(struct spi_device *spi)
{
	const struct adafruit_tft_display *display;
	const struct of_device_id *of_id;
	struct device *dev = &spi->dev;
	struct tinydrm_device *tdev;
	struct gpio_desc *dc = NULL;
	struct mipi_dbi *mipi;
	u32 rotation = 0;
	int id, ret;

	of_id = of_match_device(adafruit_tft_of_match, dev);
	if (of_id) {
		id = (int)of_id->data;
	} else {
		const struct spi_device_id *spi_id = spi_get_device_id(spi);

		if (!spi_id)
			return -EINVAL;

		id = spi_id->driver_data;
	}

	if (!dev->coherent_dma_mask) {
		ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(32));
		if (ret)
			dev_warn(dev, "Failed to set dma mask %d\n", ret);
	}

	display = &adafruit_tft_displays[id];

	mipi = devm_kzalloc(dev, sizeof(*mipi), GFP_KERNEL);
	if (!mipi)
		return -ENOMEM;

	mipi->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(mipi->reset)) {
		dev_err(dev, "Failed to get gpio 'reset'\n");
		return PTR_ERR(mipi->reset);
	}

	if (display->dc) {
		dc = devm_gpiod_get_optional(dev, "dc", GPIOD_OUT_LOW);
		if (IS_ERR(dc)) {
			dev_err(dev, "Failed to get gpio 'dc'\n");
			return PTR_ERR(dc);
		}
	}

	mipi->regulator = devm_regulator_get_optional(dev, "power");
	if (IS_ERR(mipi->regulator)) {
		ret = PTR_ERR(mipi->regulator);
		if (ret != -ENODEV)
			return ret;

		mipi->regulator = NULL;
	}

	mipi->backlight = tinydrm_of_find_backlight(dev);
	if (IS_ERR(mipi->backlight))
		return PTR_ERR(mipi->backlight);

	device_property_read_u32(dev, "rotation", &rotation);

	mipi->reg = mipi_dbi_spi_init(spi, dc, display->write_only);
	if (IS_ERR(mipi->reg))
		return PTR_ERR(mipi->reg);

	ret = mipi_dbi_init(dev, mipi, &display->funcs, &adafruit_tft_driver,
			    &display->mode, rotation);
	if (ret)
		return ret;

	tdev = &mipi->tinydrm;

	ret = devm_tinydrm_register(tdev);
	if (ret)
		return ret;

	spi_set_drvdata(spi, tdev);

	DRM_DEBUG_DRIVER("Initialized %s:%s @%uMHz on minor %d\n",
			 tdev->drm.driver->name, dev_name(dev),
			 spi->max_speed_hz / 1000000,
			 tdev->drm.primary->index);

	return 0;
}

static struct spi_driver adafruit_tft_spi_driver = {
	.driver = {
		.name = "adafruit-tft",
		.owner = THIS_MODULE,
		.of_match_table = adafruit_tft_of_match,
		.pm = &tinydrm_simple_pm_ops,
	},
	.id_table = adafruit_tft_id,
	.probe = adafruit_tft_probe,
	.shutdown = tinydrm_spi_shutdown,
};
module_spi_driver(adafruit_tft_spi_driver);

MODULE_DESCRIPTION("Adafruit MIPI compatible SPI displays");
MODULE_AUTHOR("Noralf Trønnes");
MODULE_LICENSE("GPL");
