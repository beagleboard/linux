/*
 * Copyright (C) 2012 Texas Instruments
 * Author: Rob Clark <robdclark@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/consumer.h>
#include <linux/backlight.h>
#include <video/display_timing.h>
#include <video/of_display_timing.h>
#include <video/videomode.h>
#include <linux/of_gpio.h>

#include "tilcdc_drv.h"

struct panel_module {
	struct tilcdc_module base;
	struct tilcdc_panel_info *info;
	struct display_timings *timings;
	struct backlight_device *backlight;
	int gpio;
	struct pinctrl *pinctrl;
	char *selected_state_name;
};
#define to_panel_module(x) container_of(x, struct panel_module, base)


/*
 * Encoder:
 */

struct panel_encoder {
	struct drm_encoder base;
	struct panel_module *mod;
};
#define to_panel_encoder(x) container_of(x, struct panel_encoder, base)


static void panel_encoder_destroy(struct drm_encoder *encoder)
{
	struct panel_encoder *panel_encoder = to_panel_encoder(encoder);
	drm_encoder_cleanup(encoder);
	kfree(panel_encoder);
}

static void panel_encoder_dpms(struct drm_encoder *encoder, int mode)
{
	struct panel_encoder *panel_encoder = to_panel_encoder(encoder);
	struct backlight_device *backlight = panel_encoder->mod->backlight;

	if (!backlight)
		return;

	backlight->props.power = mode == DRM_MODE_DPMS_ON
				     ? FB_BLANK_UNBLANK : FB_BLANK_POWERDOWN;
	backlight_update_status(backlight);
}

static bool panel_encoder_mode_fixup(struct drm_encoder *encoder,
		const struct drm_display_mode *mode,
		struct drm_display_mode *adjusted_mode)
{
	/* nothing needed */
	return true;
}

static void panel_encoder_prepare(struct drm_encoder *encoder)
{
	struct panel_encoder *panel_encoder = to_panel_encoder(encoder);
	panel_encoder_dpms(encoder, DRM_MODE_DPMS_OFF);
	tilcdc_crtc_set_panel_info(encoder->crtc, panel_encoder->mod->info);
}

static void panel_encoder_commit(struct drm_encoder *encoder)
{
	panel_encoder_dpms(encoder, DRM_MODE_DPMS_ON);
}

static void panel_encoder_mode_set(struct drm_encoder *encoder,
		struct drm_display_mode *mode,
		struct drm_display_mode *adjusted_mode)
{
	/* nothing needed */
}

static const struct drm_encoder_funcs panel_encoder_funcs = {
		.destroy        = panel_encoder_destroy,
};

static const struct drm_encoder_helper_funcs panel_encoder_helper_funcs = {
		.dpms           = panel_encoder_dpms,
		.mode_fixup     = panel_encoder_mode_fixup,
		.prepare        = panel_encoder_prepare,
		.commit         = panel_encoder_commit,
		.mode_set       = panel_encoder_mode_set,
};

static struct drm_encoder *panel_encoder_create(struct drm_device *dev,
		struct panel_module *mod)
{
	struct panel_encoder *panel_encoder;
	struct drm_encoder *encoder;
	int ret;

	panel_encoder = kzalloc(sizeof(*panel_encoder), GFP_KERNEL);
	if (!panel_encoder) {
		dev_err(dev->dev, "allocation failed\n");
		return NULL;
	}

	panel_encoder->mod = mod;

	encoder = &panel_encoder->base;
	encoder->possible_crtcs = 1;

	ret = drm_encoder_init(dev, encoder, &panel_encoder_funcs,
			DRM_MODE_ENCODER_LVDS);
	if (ret < 0)
		goto fail;

	drm_encoder_helper_add(encoder, &panel_encoder_helper_funcs);

	return encoder;

fail:
	panel_encoder_destroy(encoder);
	return NULL;
}

/*
 * Connector:
 */

struct panel_connector {
	struct drm_connector base;

	struct drm_encoder *encoder;  /* our connected encoder */
	struct panel_module *mod;
};
#define to_panel_connector(x) container_of(x, struct panel_connector, base)


static void panel_connector_destroy(struct drm_connector *connector)
{
	struct panel_connector *panel_connector = to_panel_connector(connector);
	drm_connector_cleanup(connector);
	kfree(panel_connector);
}

static enum drm_connector_status panel_connector_detect(
		struct drm_connector *connector,
		bool force)
{
	return connector_status_connected;
}

static int panel_connector_get_modes(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;
	struct panel_connector *panel_connector = to_panel_connector(connector);
	struct display_timings *timings = panel_connector->mod->timings;
	int i;

	for (i = 0; i < timings->num_timings; i++) {
		struct drm_display_mode *mode = drm_mode_create(dev);
		struct videomode vm;

		if (videomode_from_timing(timings, &vm, i))
			break;

		drm_display_mode_from_videomode(&vm, mode);

		mode->type = DRM_MODE_TYPE_DRIVER;

		if (timings->native_mode == i)
			mode->type |= DRM_MODE_TYPE_PREFERRED;

		drm_mode_set_name(mode);
		drm_mode_probed_add(connector, mode);
	}

	return i;
}

static int panel_connector_mode_valid(struct drm_connector *connector,
		  struct drm_display_mode *mode)
{
	struct tilcdc_drm_private *priv = connector->dev->dev_private;
	/* our only constraints are what the crtc can generate: */
	return tilcdc_crtc_mode_valid(priv->crtc, mode, 0);
}

static struct drm_encoder *panel_connector_best_encoder(
		struct drm_connector *connector)
{
	struct panel_connector *panel_connector = to_panel_connector(connector);
	return panel_connector->encoder;
}

static const struct drm_connector_funcs panel_connector_funcs = {
	.destroy            = panel_connector_destroy,
	.dpms               = drm_helper_connector_dpms,
	.detect             = panel_connector_detect,
	.fill_modes         = drm_helper_probe_single_connector_modes,
};

static const struct drm_connector_helper_funcs panel_connector_helper_funcs = {
	.get_modes          = panel_connector_get_modes,
	.mode_valid         = panel_connector_mode_valid,
	.best_encoder       = panel_connector_best_encoder,
};

static struct drm_connector *panel_connector_create(struct drm_device *dev,
		struct panel_module *mod, struct drm_encoder *encoder)
{
	struct panel_connector *panel_connector;
	struct drm_connector *connector;
	int ret;

	panel_connector = kzalloc(sizeof(*panel_connector), GFP_KERNEL);
	if (!panel_connector) {
		dev_err(dev->dev, "allocation failed\n");
		return NULL;
	}

	panel_connector->encoder = encoder;
	panel_connector->mod = mod;

	connector = &panel_connector->base;

	drm_connector_init(dev, connector, &panel_connector_funcs,
			DRM_MODE_CONNECTOR_LVDS);
	drm_connector_helper_add(connector, &panel_connector_helper_funcs);

	connector->interlace_allowed = 0;
	connector->doublescan_allowed = 0;

	ret = drm_mode_connector_attach_encoder(connector, encoder);
	if (ret)
		goto fail;

	drm_sysfs_connector_add(connector);

	return connector;

fail:
	panel_connector_destroy(connector);
	return NULL;
}

/*
 * Module:
 */

static int panel_modeset_init(struct tilcdc_module *mod, struct drm_device *dev)
{
	struct panel_module *panel_mod = to_panel_module(mod);
	struct tilcdc_drm_private *priv = dev->dev_private;
	struct drm_encoder *encoder;
	struct drm_connector *connector;

	encoder = panel_encoder_create(dev, panel_mod);
	if (!encoder)
		return -ENOMEM;

	connector = panel_connector_create(dev, panel_mod, encoder);
	if (!connector)
		return -ENOMEM;

	priv->encoders[priv->num_encoders++] = encoder;
	priv->connectors[priv->num_connectors++] = connector;

	return 0;
}

static void panel_destroy(struct tilcdc_module *mod)
{
	struct panel_module *panel_mod = to_panel_module(mod);

	if (panel_mod->timings) {
		display_timings_release(panel_mod->timings);
		kfree(panel_mod->timings);
	}

	tilcdc_module_cleanup(mod);
	kfree(panel_mod->info);
	kfree(panel_mod);
}

static const struct tilcdc_module_ops panel_module_ops = {
		.modeset_init = panel_modeset_init,
		.destroy = panel_destroy,
};

/*
 * Device:
 */

/* maybe move this somewhere common if it is needed by other outputs? */
static struct tilcdc_panel_info * of_get_panel_info(struct device_node *np)
{
	struct device_node *info_np;
	struct tilcdc_panel_info *info;
	int ret = 0;

	if (!np) {
		pr_err("%s: no devicenode given\n", __func__);
		return NULL;
	}

	info_np = of_get_child_by_name(np, "panel-info");
	if (!info_np) {
		pr_err("%s: could not find panel-info node\n", __func__);
		return NULL;
	}

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		pr_err("%s: allocation failed\n", __func__);
		return NULL;
	}

	ret |= of_property_read_u32(info_np, "ac-bias", &info->ac_bias);
	ret |= of_property_read_u32(info_np, "ac-bias-intrpt", &info->ac_bias_intrpt);
	ret |= of_property_read_u32(info_np, "dma-burst-sz", &info->dma_burst_sz);
	ret |= of_property_read_u32(info_np, "bpp", &info->bpp);
	ret |= of_property_read_u32(info_np, "fdd", &info->fdd);
	ret |= of_property_read_u32(info_np, "sync-edge", &info->sync_edge);
	ret |= of_property_read_u32(info_np, "sync-ctrl", &info->sync_ctrl);
	ret |= of_property_read_u32(info_np, "raster-order", &info->raster_order);
	ret |= of_property_read_u32(info_np, "fifo-th", &info->fifo_th);

	/* optional: */
	info->tft_alt_mode      = of_property_read_bool(info_np, "tft-alt-mode");
	info->invert_pxl_clk    = of_property_read_bool(info_np, "invert-pxl-clk");

	if (ret) {
		pr_err("%s: error reading panel-info properties\n", __func__);
		kfree(info);
		return NULL;
	}

	return info;
}

static struct of_device_id panel_of_match[];

static ssize_t pinmux_show_state(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct panel_module *panel_mod = platform_get_drvdata(pdev);
	const char *name;

	name = panel_mod->selected_state_name;
	if (name == NULL || strlen(name) == 0)
		name = "none";
	return sprintf(buf, "%s\n", name);
}

static ssize_t pinmux_store_state(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct panel_module *panel_mod = platform_get_drvdata(pdev);
	struct pinctrl_state *state;
	char *state_name;
	char *s;
	int err;

	/* duplicate (as a null terminated string) */
	state_name = kmalloc(count + 1, GFP_KERNEL);
	if (state_name == NULL)
		return -ENOMEM;
	memcpy(state_name, buf, count);
	state_name[count] = '\0';

	/* and chop off newline */
	s = strchr(state_name, '\n');
	if (s != NULL)
		*s = '\0';

	/* try to select default state at first (if it exists) */
	state = pinctrl_lookup_state(panel_mod->pinctrl, state_name);
	if (!IS_ERR(state)) {
		err = pinctrl_select_state(panel_mod->pinctrl, state);
		if (err != 0)
			dev_err(dev, "Failed to select state %s\n",
					state_name);
	} else {
		dev_err(dev, "Failed to find state %s\n", state_name);
		err = PTR_RET(state);
	}

	if (err == 0) {
		kfree(panel_mod->selected_state_name);
		panel_mod->selected_state_name = state_name;
	}

	return err ? err : count;
}

static DEVICE_ATTR(pinmux_state, S_IWUSR | S_IRUGO,
		   pinmux_show_state, pinmux_store_state);

static struct attribute *pinmux_attributes[] = {
	&dev_attr_pinmux_state.attr,
	NULL
};

static const struct attribute_group pinmux_attr_group = {
	.attrs = pinmux_attributes,
};

static int panel_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = pdev->dev.of_node;
	struct panel_module *panel_mod;
	struct tilcdc_module *mod;
	struct pinctrl_state *state;
	enum of_gpio_flags ofgpioflags;
	unsigned long gpioflags;
	char *state_name;
	int ret = -EINVAL;

	/* bail out early if no DT data: */
	if (!node) {
		dev_err(&pdev->dev, "device-tree data is missing\n");
		return -ENXIO;
	}

	panel_mod = kzalloc(sizeof(*panel_mod), GFP_KERNEL);
	if (!panel_mod)
		return -ENOMEM;

	platform_set_drvdata(pdev, panel_mod);

	mod = &panel_mod->base;

	tilcdc_module_init(mod, "panel", &panel_module_ops);

	state_name = kmalloc(strlen(PINCTRL_STATE_DEFAULT) + 1,
			GFP_KERNEL);
	if (state_name == NULL) {
		dev_err(dev, "Failed to allocate state name\n");
		ret = -ENOMEM;
		goto fail;
	}
	panel_mod->selected_state_name = state_name;
	strcpy(panel_mod->selected_state_name, PINCTRL_STATE_DEFAULT);

	panel_mod->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(panel_mod->pinctrl)) {
		dev_err(dev, "Failed to get pinctrl\n");
		ret = PTR_RET(panel_mod->pinctrl);
		goto fail;
	}

	/* try to select default state at first (if it exists) */
	state = pinctrl_lookup_state(panel_mod->pinctrl,
			panel_mod->selected_state_name);
	if (!IS_ERR(state)) {
		ret = pinctrl_select_state(panel_mod->pinctrl, state);
		if (ret != 0) {
			dev_err(dev, "Failed to select default state\n");
			goto fail;
		}
	} else {
		panel_mod->selected_state_name = '\0';
	}

	/* Register sysfs hooks */
	ret = sysfs_create_group(&dev->kobj, &pinmux_attr_group);
	if (ret) {
		dev_err(dev, "Failed to create sysfs group\n");
		goto fail;
	}

	panel_mod->timings = of_get_display_timings(node);
	if (!panel_mod->timings) {
		dev_err(&pdev->dev, "could not get panel timings\n");
		goto fail;
	}

	panel_mod->info = of_get_panel_info(node);
	if (!panel_mod->info) {
		dev_err(&pdev->dev, "could not get panel info\n");
		goto fail;
	}

	panel_mod->backlight = of_find_backlight_by_node(node);
	if (panel_mod->backlight)
		dev_info(&pdev->dev, "found backlight\n");

	panel_mod->gpio = of_get_named_gpio_flags(pdev->dev.of_node, "ti,power-gpio",
                       0, &ofgpioflags);
	if (IS_ERR_VALUE(panel_mod->gpio)) {
		dev_warn(&pdev->dev, "panel: No power control GPIO\n");
	} else {
		gpioflags = GPIOF_DIR_OUT;
		if (ofgpioflags & OF_GPIO_ACTIVE_LOW) {
			gpioflags |= GPIOF_INIT_LOW;
			dev_info(&pdev->dev, "Power GPIO active low, initial state set to low\n");
		} else {
			gpioflags |= GPIOF_INIT_HIGH;
			dev_info(&pdev->dev, "Power GPIO active high, initial state set to high\n");
		}
		ret = devm_gpio_request_one(&pdev->dev, panel_mod->gpio,
				gpioflags, "panel:PDN");
		if (ret != 0) {
			dev_err(&pdev->dev, "Failed to request power gpio\n");
			goto fail;
		}
	}

	return 0;

fail:
	panel_destroy(mod);
	return ret;
}

static int panel_remove(struct platform_device *pdev)
{
	return 0;
}

static struct of_device_id panel_of_match[] = {
		{ .compatible = "tilcdc,panel", },
		{ },
};
MODULE_DEVICE_TABLE(of, panel_of_match);

struct platform_driver panel_driver = {
	.probe = panel_probe,
	.remove = panel_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "panel",
		.of_match_table = panel_of_match,
	},
};

int __init tilcdc_panel_init(void)
{
	return platform_driver_register(&panel_driver);
}

void __exit tilcdc_panel_fini(void)
{
	platform_driver_unregister(&panel_driver);
}
