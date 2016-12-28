#define DEBUG

/*
 * DRM driver for Multi-Inno MI0283QT panels
 *
 * Copyright 2016 Noralf Trønnes
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

/* TODO: DT binding

of: Add vendor prefix for Multi-Inno
Multi-Inno Technology Co.,Ltd is a Hong Kong based company offering
LCD, LCD module products and complete panel solutions.

Documentation/devicetree/bindings/vendor-prefixes.txt
-------------------------------------------------------------------------------
multi-inno	Multi-Inno Technology Co.,Ltd
-------------------------------------------------------------------------------

dt-bindings: Add Multi-Inno MI0283QT binding
Add device-tree binding documentation for the MI0283QT display panel.

Documentation/devicetree/bindings/display/multi-inno,mi0283qt.txt
-------------------------------------------------------------------------------
Multi-Inno MI0283QT display panel

Required properties:
- compatible:	"multi-inno,mi0283qt".

The node for this driver must be a child node of a SPI controller, hence
all mandatory properties described in ../spi/spi-bus.txt must be specified.

Optional properties:
- dc-gpios:	D/C pin. The presence/absence of this GPIO determines
		the panel interface mode (IM[3:0] pins):
		- present: IM=x110 4-wire 8-bit data serial interface
		- absent:  IM=x101 3-wire 9-bit data serial interface
- reset-gpios:	Reset pin
- power-supply:	A regulator node for the supply voltage.
- backlight:	phandle of the backlight device attached to the panel
- rotation:	panel rotation in degrees counter clockwise (0,90,180,270)
- write-only:	LCD controller is write only. This depends on the interface
		mode, SPI master driver and wiring:
		- IM=11xx and MISO not connected
		- IM=01xx and SPI master driver doesn't support spi-3wire (SDA)

Example:
	mi0283qt@0{
		compatible = "multi-inno,mi0283qt";
		reg = <0>;
		spi-max-frequency = <32000000>;
		rotation = <90>;
		dc-gpios = <&gpio 25 0>;
		backlight = <&backlight>;
	};
-------------------------------------------------------------------------------
*/

#include <drm/tinydrm/ili9341.h>
#include <drm/tinydrm/mipi-dbi.h>
#include <drm/tinydrm/tinydrm.h>
#include <drm/tinydrm/tinydrm-helpers.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <video/mipi_display.h>

static void mi0283qt_enable(struct drm_simple_display_pipe *pipe,
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

	/* Avoid flicker by skipping setup if the bootloader has done it */
	if (mipi_dbi_display_is_on(reg)) {
		tdev->prepared = true;
		goto out_unlock;
	}

	mipi_dbi_hw_reset(mipi);
	ret = mipi_dbi_write(reg, MIPI_DCS_SOFT_RESET);
	if (ret) {
		dev_err(dev, "Error writing command %d\n", ret);
		goto out_unlock;
	}

	msleep(20);

	mipi_dbi_write(reg, MIPI_DCS_SET_DISPLAY_OFF);

	mipi_dbi_write(reg, ILI9341_PWCTRLB, 0x00, 0x83, 0x30);
	mipi_dbi_write(reg, ILI9341_PWRSEQ, 0x64, 0x03, 0x12, 0x81);
	mipi_dbi_write(reg, ILI9341_DTCTRLA, 0x85, 0x01, 0x79);
	mipi_dbi_write(reg, ILI9341_PWCTRLA, 0x39, 0x2c, 0x00, 0x34, 0x02);
	mipi_dbi_write(reg, ILI9341_PUMPCTRL, 0x20);
	mipi_dbi_write(reg, ILI9341_DTCTRLB, 0x00, 0x00);

	/* Power Control */
	mipi_dbi_write(reg, ILI9341_PWCTRL1, 0x26);
	mipi_dbi_write(reg, ILI9341_PWCTRL2, 0x11);
	/* VCOM */
	mipi_dbi_write(reg, ILI9341_VMCTRL1, 0x35, 0x3e);
	mipi_dbi_write(reg, ILI9341_VMCTRL2, 0xbe);

	/* Memory Access Control */
	mipi_dbi_write(reg, MIPI_DCS_SET_PIXEL_FORMAT, 0x55);

	switch (mipi->rotation) {
	default:
		addr_mode = ILI9341_MADCTL_MV | ILI9341_MADCTL_MY |
			    ILI9341_MADCTL_MX;
		break;
	case 90:
		addr_mode = ILI9341_MADCTL_MY;
		break;
	case 180:
		addr_mode = ILI9341_MADCTL_MV;
		break;
	case 270:
		addr_mode = ILI9341_MADCTL_MX;
		break;
	}
	addr_mode |= ILI9341_MADCTL_BGR;
	mipi_dbi_write(reg, MIPI_DCS_SET_ADDRESS_MODE, addr_mode);

	/* Frame Rate */
	mipi_dbi_write(reg, ILI9341_FRMCTR1, 0x00, 0x1b);

	/* Gamma */
	mipi_dbi_write(reg, ILI9341_EN3GAM, 0x08);
	mipi_dbi_write(reg, MIPI_DCS_SET_GAMMA_CURVE, 0x01);
	mipi_dbi_write(reg, ILI9341_PGAMCTRL,
		       0x1f, 0x1a, 0x18, 0x0a, 0x0f, 0x06, 0x45, 0x87,
		       0x32, 0x0a, 0x07, 0x02, 0x07, 0x05, 0x00);
	mipi_dbi_write(reg, ILI9341_NGAMCTRL,
		       0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3a, 0x78,
		       0x4d, 0x05, 0x18, 0x0d, 0x38, 0x3a, 0x1f);

	/* DDRAM */
	mipi_dbi_write(reg, ILI9341_ETMOD, 0x07);

	/* Display */
	mipi_dbi_write(reg, ILI9341_DISCTRL, 0x0a, 0x82, 0x27, 0x00);
	mipi_dbi_write(reg, MIPI_DCS_EXIT_SLEEP_MODE);
	msleep(100);

	mipi_dbi_write(reg, MIPI_DCS_SET_DISPLAY_ON);
	msleep(100);

	tdev->prepared = true;
	if (pipe->plane.state->fb)
		schedule_work(&tdev->dirty_work);

out_unlock:
	mutex_unlock(&tdev->dev_lock);
}

static const struct drm_simple_display_pipe_funcs mi0283qt_pipe_funcs = {
	.enable = mi0283qt_enable,
	.disable = mipi_dbi_pipe_disable,
	.update = tinydrm_display_pipe_update,
};

static const struct drm_display_mode mi0283qt_mode = {
	TINYDRM_MODE(320, 240, 58, 43),
};

static const struct of_device_id mi0283qt_of_match[] = {
	{ .compatible = "multi-inno,mi0283qt" },
	{},
};
MODULE_DEVICE_TABLE(of, mi0283qt_of_match);

static const struct spi_device_id mi0283qt_id[] = {
	{ "mi0283qt", 0 },
	{ },
};
MODULE_DEVICE_TABLE(spi, mi0283qt_id);

static struct drm_driver mi0283qt_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_PRIME |
				  DRIVER_ATOMIC,
	TINYDRM_GEM_DRIVER_OPS,
	.lastclose		= tinydrm_lastclose,
	.debugfs_init		= mipi_dbi_debugfs_init,
	.debugfs_cleanup	= mipi_dbi_debugfs_cleanup,
	.name			= "mi0283qt",
	.desc			= "Multi-Inno MI0283QT",
	.date			= "20160614",
	.major			= 1,
	.minor			= 0,
};

static int mi0283qt_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct tinydrm_device *tdev;
	struct mipi_dbi *mipi;
	struct gpio_desc *dc;
	u32 rotation = 0;
	bool writeonly;
	int ret;

	if (!dev->coherent_dma_mask) {
		ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(32));
		if (ret)
			dev_warn(dev, "Failed to set dma mask %d\n", ret);
	}

	mipi = devm_kzalloc(dev, sizeof(*mipi), GFP_KERNEL);
	if (!mipi)
		return -ENOMEM;

	mipi->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(mipi->reset)) {
		dev_err(dev, "Failed to get gpio 'reset'\n");
		return PTR_ERR(mipi->reset);
	}

	dc = devm_gpiod_get_optional(dev, "dc", GPIOD_OUT_LOW);
	if (IS_ERR(dc)) {
		dev_err(dev, "Failed to get gpio 'dc'\n");
		return PTR_ERR(dc);
	}

	mipi->regulator = devm_regulator_get_optional(dev, "power");
	if (IS_ERR(mipi->regulator)) {
		ret = PTR_ERR(mipi->regulator);
		if (ret != -ENODEV)
			return ret;

		mipi->regulator = NULL;
	}

	mipi->enable_delay_ms = 50;
	mipi->backlight = tinydrm_of_find_backlight(dev);
	if (IS_ERR(mipi->backlight))
		return PTR_ERR(mipi->backlight);

	writeonly = device_property_read_bool(dev, "write-only");
	device_property_read_u32(dev, "rotation", &rotation);

	mipi->reg = mipi_dbi_spi_init(spi, dc, writeonly);
	if (IS_ERR(mipi->reg))
		return PTR_ERR(mipi->reg);

	ret = mipi_dbi_init(dev, mipi, &mi0283qt_pipe_funcs, &mi0283qt_driver,
			    &mi0283qt_mode, rotation);
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

static struct spi_driver mi0283qt_spi_driver = {
	.driver = {
		.name = "mi0283qt",
		.owner = THIS_MODULE,
		.of_match_table = mi0283qt_of_match,
		.pm = &tinydrm_simple_pm_ops,
	},
	.id_table = mi0283qt_id,
	.probe = mi0283qt_probe,
	.shutdown = tinydrm_spi_shutdown,
};
module_spi_driver(mi0283qt_spi_driver);

MODULE_DESCRIPTION("Multi-Inno MI0283QT DRM driver");
MODULE_AUTHOR("Noralf Trønnes");
MODULE_LICENSE("GPL");
