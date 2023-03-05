// SPDX-License-Identifier: GPL-2.0
/*
 * Diiver for panels based on Himax HX8394 controller
 * Copyright (c) 2023, Alibaba-inc Co., Ltd
 *
 */

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_device.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#include <video/mipi_display.h>

struct hx8394_panel_cmd {
	char cmdlen;
	char cmddata[0x40];
};

struct hx8394_panel_desc {
	const struct drm_display_mode *display_mode;

	unsigned long mode_flags;
	enum mipi_dsi_pixel_format format;
	unsigned int lanes;
	const struct hx8394_panel_cmd *on_cmds;
	unsigned int on_cmds_num;
};

struct panel_info {
	struct drm_panel base;
	struct mipi_dsi_device *link;
	const struct hx8394_panel_desc *desc;

	struct gpio_desc	*reset;
	struct regulator	*hsvcc;
	struct regulator	*vspn3v3;

	bool prepared;
	bool enabled;
};

static inline struct panel_info *to_panel_info(struct drm_panel *panel)
{
	return container_of(panel, struct panel_info, base);
}

static int hx8394_send_mipi_cmds(struct drm_panel *panel, const struct hx8394_panel_cmd *cmds)
{
	struct panel_info *pinfo = to_panel_info(panel);
	unsigned int i = 0;
	int err;

	for (i = 0; i < pinfo->desc->on_cmds_num; i++) {
		err = mipi_dsi_dcs_write_buffer(pinfo->link, &(cmds[i].cmddata[0]), cmds[i].cmdlen);
		if (err < 0)
			return err;
	}

	return 0;
}

static int hx8394_panel_disable(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);
	int err;

	if (!pinfo->enabled)
		return 0;

	err = mipi_dsi_dcs_set_display_off(pinfo->link);
	if (err < 0) {
		dev_err(panel->dev, "failed to set display off: %d\n", err);
		return err;
	}

	pinfo->enabled = false;

	return 0;
}

static int hx8394_panel_unprepare(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);
	int err;

	if (!pinfo->prepared)
		return 0;

	err = mipi_dsi_dcs_set_display_off(pinfo->link);
	if (err < 0)
		dev_err(panel->dev, "failed to set display off: %d\n", err);

	err = mipi_dsi_dcs_enter_sleep_mode(pinfo->link);
	if (err < 0)
		dev_err(panel->dev, "failed to enter sleep mode: %d\n", err);

	/* sleep_mode_delay: 1ms - 2ms */
	usleep_range(1000, 2000);

	gpiod_set_value(pinfo->reset, 1);
	regulator_disable(pinfo->hsvcc);
	regulator_disable(pinfo->vspn3v3);

	pinfo->prepared = false;

	return 0;
}

static int hx8394_panel_prepare(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);
	int ret;

	if (pinfo->prepared)
		return 0;
	gpiod_set_value(pinfo->reset, 1);

	/* Power the panel */
	ret = regulator_enable(pinfo->hsvcc);
	if (ret) {
		dev_err(pinfo->base.dev, "Failed to enable hsvcc supply: %d\n", ret);
		return ret;
	}

	usleep_range(1000, 2000);
	ret = regulator_enable(pinfo->vspn3v3);
	if (ret) {
		dev_err(pinfo->base.dev, "Failed to enable vspn3v3 supply: %d\n", ret);
		goto fail;
	}
	usleep_range(5000, 6000);

	gpiod_set_value(pinfo->reset, 0);
	msleep(180);

	pinfo->prepared = true;

	return 0;

fail:
	gpiod_set_value(pinfo->reset, 1);
	regulator_disable(pinfo->hsvcc);
	return ret;
}

static int hx8394_read_id(struct mipi_dsi_device *dsi, u8 *id1)
{
	int ret;

	ret = mipi_dsi_dcs_read(dsi, 0xDA, id1, 1);
	if (ret < 0) {
		dev_err(&dsi->dev, "could not read ID1\n");
		return ret;
	}
	dev_info(&dsi->dev, "ID1 : 0x%02x\n", *id1);

	return 0;
}

static int hx8394_panel_enable(struct drm_panel *panel)
{
	struct panel_info *pinfo = to_panel_info(panel);
	int ret;
	u8 id1;

	if (pinfo->enabled)
		return 0;

	ret = hx8394_read_id(pinfo->link, &id1);
	if (ret < 0)
		dev_info(panel->dev, "No LCD connected,pls check your hardware! ret:%d\n", ret);

	/* send init code */
	ret = hx8394_send_mipi_cmds(panel, pinfo->desc->on_cmds);
	if (ret < 0) {
		dev_err(panel->dev, "failed to send DCS Init Code: %d\n", ret);
		return ret;
	}

	ret = mipi_dsi_dcs_exit_sleep_mode(pinfo->link);
	if (ret < 0) {
		dev_err(panel->dev, "failed to exit sleep mode: %d\n", ret);
		return ret;
	}

	msleep(120);

	ret = mipi_dsi_dcs_set_display_on(pinfo->link);
	if (ret < 0) {
		dev_err(panel->dev, "failed to set display on: %d\n", ret);
		return ret;
	}

	pinfo->enabled = true;

	return 0;
}

static int hx8394_panel_get_modes(struct drm_panel *panel,
			       struct drm_connector *connector)
{
	struct panel_info *pinfo = to_panel_info(panel);
	const struct drm_display_mode *m = pinfo->desc->display_mode;
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, m);
	if (!mode) {
		dev_err(pinfo->base.dev, "failed to add mode %ux%u@%u\n",
			m->hdisplay, m->vdisplay, drm_mode_vrefresh(m));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);

	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;

	return 1;
}

static const struct drm_panel_funcs panel_funcs = {
	.disable = hx8394_panel_disable,
	.unprepare = hx8394_panel_unprepare,
	.prepare = hx8394_panel_prepare,
	.enable = hx8394_panel_enable,
	.get_modes = hx8394_panel_get_modes,
};

static const struct drm_display_mode hx8394_default_mode = {
	.clock		= 76000,
	.hdisplay	= 720,
	.hsync_start	= 720 + 45,
	.hsync_end	= 720 + 45 + 8,
	.htotal		= 720 + 45 + 8 + 45,

	.vdisplay	= 1280,
	.vsync_start	= 1280 + 16,
	.vsync_end	= 1280 + 16 + 8,
	.vtotal		= 1280 + 16 + 8 + 16,

	.width_mm	= 62,
	.height_mm	= 110,
	.flags          = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC,
};

static const struct hx8394_panel_cmd hx8394_on_cmds[] = {
	{ .cmdlen = 4,	.cmddata = {0xB9, 0xFF, 0x83, 0x94} },
	{ .cmdlen = 11,	.cmddata = {0xB1, 0x48, 0x0A, 0x6A, 0x09, 0x33, 0x54,
				0x71, 0x71, 0x2E, 0x45} },
	{ .cmdlen = 7,	.cmddata = {0xBA, 0x63, 0x03, 0x68, 0x6B, 0xB2, 0xC0} },
	{ .cmdlen = 7,	.cmddata = {0xB2, 0x00, 0x80, 0x64, 0x0C, 0x06, 0x2F} },
	{ .cmdlen = 22, .cmddata = {0xB4, 0x1C, 0x78, 0x1C, 0x78, 0x1C, 0x78, 0x01,
				0x0C, 0x86, 0x75, 0x00, 0x3F, 0x1C, 0x78, 0x1C,
				0x78, 0x1C, 0x78, 0x01, 0x0C, 0x86} },
	{ .cmdlen = 34, .cmddata = {0xD3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08,
				0x08, 0x32, 0x10, 0x05, 0x00, 0x05, 0x32, 0x13,
				0xC1, 0x00, 0x01, 0x32, 0x10, 0x08, 0x00, 0x00,
				0x37, 0x03, 0x07, 0x07, 0x37, 0x05, 0x05, 0x37,
				0x0C, 0x40} },
	{ .cmdlen = 45, .cmddata = {0xD5, 0x18, 0x18, 0x18, 0x18, 0x22, 0x23, 0x20,
				0x21, 0x04, 0x05, 0x06, 0x07, 0x00, 0x01, 0x02,
				0x03, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
				0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
				0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
				0x18, 0x19, 0x19, 0x19, 0x19} },
	{ .cmdlen = 45, .cmddata = {0xD6, 0x18, 0x18, 0x19, 0x19, 0x21, 0x20, 0x23,
				0x22, 0x03, 0x02, 0x01, 0x00, 0x07, 0x06, 0x05,
				0x04, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
				0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
				0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
				0x18, 0x19, 0x19, 0x18, 0x18} },
	{ .cmdlen = 59, .cmddata = {0xE0, 0x07, 0x08, 0x09, 0x0D, 0x10, 0x14, 0x16,
				0x13, 0x24, 0x36, 0x48, 0x4A, 0x58, 0x6F, 0x76,
				0x80, 0x97, 0xA5, 0xA8, 0xB5, 0xC6, 0x62, 0x63,
				0x68, 0x6F, 0x72, 0x78, 0x7F, 0x7F, 0x00, 0x02,
				0x08, 0x0D, 0x0C, 0x0E, 0x0F, 0x10, 0x24, 0x36,
				0x48, 0x4A, 0x58, 0x6F, 0x78, 0x82, 0x99, 0xA4,
				0xA0, 0xB1, 0xC0, 0x5E, 0x5E, 0x64, 0x6B, 0x6C,
				0x73, 0x7F, 0x7F} },
	{ .cmdlen = 2, .cmddata = {0xCC, 0x03} },
	{ .cmdlen = 3, .cmddata = {0xC0, 0x1F, 0x73} },
	{ .cmdlen = 3, .cmddata = {0xB6, 0x90, 0x90} },
	{ .cmdlen = 2, .cmddata = {0xD4, 0x02} },
	{ .cmdlen = 2, .cmddata = {0xBD, 0x01} },
	{ .cmdlen = 2, .cmddata = {0xB1, 0x00} },
	{ .cmdlen = 2, .cmddata = {0xBD, 0x00} },
	{ .cmdlen = 8, .cmddata = {0xBF, 0x40, 0x81, 0x50, 0x00, 0x1A, 0xFC, 0x01} },

	{ .cmdlen = 2, .cmddata = {0x36, 0x02} },
};

static const struct hx8394_panel_desc hx8394_desc = {
	.display_mode = &hx8394_default_mode,
	.mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_VIDEO_BURST,
	.format = MIPI_DSI_FMT_RGB888,
	.lanes = 4,
	.on_cmds = hx8394_on_cmds,
	.on_cmds_num = ARRAY_SIZE(hx8394_on_cmds),
};

static const struct of_device_id panel_of_match[] = {
	{
		.compatible = "himax,hx8394",
		.data = &hx8394_desc,
	},
	{
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, panel_of_match);

static int hx8394_panel_add(struct panel_info *pinfo)
{
	struct device *dev = &pinfo->link->dev;
	int ret;

	pinfo->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(pinfo->reset))
		return dev_err_probe(dev, PTR_ERR(pinfo->reset),
				"Couldn't get our reset GPIO\n");

	pinfo->hsvcc =  devm_regulator_get(dev, "hsvcc");
	if (IS_ERR(pinfo->hsvcc))
		return dev_err_probe(dev, PTR_ERR(pinfo->hsvcc),
				"Failed to request hsvcc regulator\n");

	pinfo->vspn3v3 =  devm_regulator_get(dev, "vspn3v3");
	if (IS_ERR(pinfo->vspn3v3))
		return dev_err_probe(dev, PTR_ERR(pinfo->vspn3v3),
				"Failed to request vspn3v3 regulator\n");

	drm_panel_init(&pinfo->base, dev, &panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);

	ret = drm_panel_of_backlight(&pinfo->base);
	if (ret)
		return ret;

	drm_panel_add(&pinfo->base);

	return 0;
}

static int hx8394_panel_probe(struct mipi_dsi_device *dsi)
{
	struct panel_info *pinfo;
	const struct hx8394_panel_desc *desc;
	int err;

	pinfo = devm_kzalloc(&dsi->dev, sizeof(*pinfo), GFP_KERNEL);
	if (!pinfo)
		return -ENOMEM;

	desc = of_device_get_match_data(&dsi->dev);
	dsi->mode_flags = desc->mode_flags;
	dsi->format = desc->format;
	dsi->lanes = desc->lanes;
	pinfo->desc = desc;

	pinfo->link = dsi;
	mipi_dsi_set_drvdata(dsi, pinfo);

	err = hx8394_panel_add(pinfo);
	if (err < 0)
		return err;

	err = mipi_dsi_attach(dsi);
	if (err < 0)
		drm_panel_remove(&pinfo->base);

	return err;
}

static int hx8394_panel_remove(struct mipi_dsi_device *dsi)
{
	struct panel_info *pinfo = mipi_dsi_get_drvdata(dsi);
	int err;

	err = hx8394_panel_disable(&pinfo->base);
	if (err < 0)
		dev_err(&dsi->dev, "failed to disable panel: %d\n", err);

	err = hx8394_panel_unprepare(&pinfo->base);
	if (err < 0)
		dev_err(&dsi->dev, "failed to unprepare panel: %d\n", err);

	err = mipi_dsi_detach(dsi);
	if (err < 0)
		dev_err(&dsi->dev, "failed to detach from DSI host: %d\n", err);

	drm_panel_remove(&pinfo->base);

	return 0;
}

static void hx8394_panel_shutdown(struct mipi_dsi_device *dsi)
{
	struct panel_info *pinfo = mipi_dsi_get_drvdata(dsi);

	hx8394_panel_disable(&pinfo->base);
	hx8394_panel_unprepare(&pinfo->base);
}

static struct mipi_dsi_driver panel_driver = {
	.driver = {
		.name = "panel-himax8394",
		.of_match_table = panel_of_match,
	},
	.probe = hx8394_panel_probe,
	.remove = hx8394_panel_remove,
	.shutdown = hx8394_panel_shutdown,
};
module_mipi_dsi_driver(panel_driver);

MODULE_DESCRIPTION("Himax8394 driver");
MODULE_LICENSE("GPL v2");
