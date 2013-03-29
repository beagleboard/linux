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

#include <linux/i2c.h>
#include <linux/of_i2c.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/consumer.h>

#include "tilcdc_drv.h"

struct tfp410_module {
	struct tilcdc_module base;
	struct i2c_adapter *i2c;
	enum of_gpio_flags ofgpioflags;
	int gpio;
	struct pinctrl *pinctrl;
	char *selected_state_name;
};
#define to_tfp410_module(x) container_of(x, struct tfp410_module, base)


static const struct tilcdc_panel_info dvi_info = {
		.ac_bias                = 255,
		.ac_bias_intrpt         = 0,
		.dma_burst_sz           = 16,
		.bpp                    = 16,
		.fdd                    = 0x80,
		.tft_alt_mode           = 0,
		.sync_edge              = 0,
		.sync_ctrl              = 1,
		.raster_order           = 0,
};

/*
 * Encoder:
 */

struct tfp410_encoder {
	struct drm_encoder base;
	struct tfp410_module *mod;
	int dpms;
};
#define to_tfp410_encoder(x) container_of(x, struct tfp410_encoder, base)


static void tfp410_encoder_destroy(struct drm_encoder *encoder)
{
	struct tfp410_encoder *tfp410_encoder = to_tfp410_encoder(encoder);
	drm_encoder_cleanup(encoder);
	kfree(tfp410_encoder);
}

static void tfp410_encoder_dpms(struct drm_encoder *encoder, int mode)
{
	struct tfp410_encoder *tfp410_encoder = to_tfp410_encoder(encoder);
	int state;

	if (tfp410_encoder->dpms == mode)
		return;

	if (IS_ERR_VALUE(tfp410_encoder->mod->gpio))
		return;

	state = mode == DRM_MODE_DPMS_ON;
	if (tfp410_encoder->mod->ofgpioflags & OF_GPIO_ACTIVE_LOW)
		state = !state;

	pr_debug("%s: dpms %d, gpio-state %d\n", __func__,
			mode == DRM_MODE_DPMS_ON, state);

	gpio_direction_output(tfp410_encoder->mod->gpio, state);

	tfp410_encoder->dpms = mode;
}

static bool tfp410_encoder_mode_fixup(struct drm_encoder *encoder,
		const struct drm_display_mode *mode,
		struct drm_display_mode *adjusted_mode)
{
	/* nothing needed */
	return true;
}

static void tfp410_encoder_prepare(struct drm_encoder *encoder)
{
	tfp410_encoder_dpms(encoder, DRM_MODE_DPMS_OFF);
	tilcdc_crtc_set_panel_info(encoder->crtc, &dvi_info);
}

static void tfp410_encoder_commit(struct drm_encoder *encoder)
{
	tfp410_encoder_dpms(encoder, DRM_MODE_DPMS_ON);
}

static void tfp410_encoder_mode_set(struct drm_encoder *encoder,
		struct drm_display_mode *mode,
		struct drm_display_mode *adjusted_mode)
{
	/* nothing needed */
}

static const struct drm_encoder_funcs tfp410_encoder_funcs = {
		.destroy        = tfp410_encoder_destroy,
};

static const struct drm_encoder_helper_funcs tfp410_encoder_helper_funcs = {
		.dpms           = tfp410_encoder_dpms,
		.mode_fixup     = tfp410_encoder_mode_fixup,
		.prepare        = tfp410_encoder_prepare,
		.commit         = tfp410_encoder_commit,
		.mode_set       = tfp410_encoder_mode_set,
};

static struct drm_encoder *tfp410_encoder_create(struct drm_device *dev,
		struct tfp410_module *mod)
{
	struct tfp410_encoder *tfp410_encoder;
	struct drm_encoder *encoder;
	int ret;

	tfp410_encoder = kzalloc(sizeof(*tfp410_encoder), GFP_KERNEL);
	if (!tfp410_encoder) {
		dev_err(dev->dev, "allocation failed\n");
		return NULL;
	}

	tfp410_encoder->dpms = DRM_MODE_DPMS_OFF;
	tfp410_encoder->mod = mod;

	encoder = &tfp410_encoder->base;
	encoder->possible_crtcs = 1;

	ret = drm_encoder_init(dev, encoder, &tfp410_encoder_funcs,
			DRM_MODE_ENCODER_TMDS);
	if (ret < 0)
		goto fail;

	drm_encoder_helper_add(encoder, &tfp410_encoder_helper_funcs);

	return encoder;

fail:
	tfp410_encoder_destroy(encoder);
	return NULL;
}

/*
 * Connector:
 */

struct tfp410_connector {
	struct drm_connector base;

	struct drm_encoder *encoder;  /* our connected encoder */
	struct tfp410_module *mod;
};
#define to_tfp410_connector(x) container_of(x, struct tfp410_connector, base)


static void tfp410_connector_destroy(struct drm_connector *connector)
{
	struct tfp410_connector *tfp410_connector = to_tfp410_connector(connector);
	drm_connector_cleanup(connector);
	kfree(tfp410_connector);
}

static enum drm_connector_status tfp410_connector_detect(
		struct drm_connector *connector,
		bool force)
{
	struct tfp410_connector *tfp410_connector = to_tfp410_connector(connector);

	if (drm_probe_ddc(tfp410_connector->mod->i2c))
		return connector_status_connected;

	return connector_status_unknown;
}

static int tfp410_connector_get_modes(struct drm_connector *connector)
{
	struct tfp410_connector *tfp410_connector = to_tfp410_connector(connector);
	struct edid *edid;
	int ret = 0;

	edid = drm_get_edid(connector, tfp410_connector->mod->i2c);

	drm_mode_connector_update_edid_property(connector, edid);

	if (edid) {
		ret = drm_add_edid_modes(connector, edid);
		kfree(edid);
	}

	return ret;
}

static int tfp410_connector_mode_valid(struct drm_connector *connector,
		  struct drm_display_mode *mode)
{
	struct tilcdc_drm_private *priv = connector->dev->dev_private;
	/* our only constraints are what the crtc can generate: */
	return tilcdc_crtc_mode_valid(priv->crtc, mode,
			priv->allow_non_rblank ? 0 : 1);
}

static struct drm_encoder *tfp410_connector_best_encoder(
		struct drm_connector *connector)
{
	struct tfp410_connector *tfp410_connector = to_tfp410_connector(connector);
	return tfp410_connector->encoder;
}

static const struct drm_connector_funcs tfp410_connector_funcs = {
	.destroy            = tfp410_connector_destroy,
	.dpms               = drm_helper_connector_dpms,
	.detect             = tfp410_connector_detect,
	.fill_modes         = drm_helper_probe_single_connector_modes,
};

static const struct drm_connector_helper_funcs tfp410_connector_helper_funcs = {
	.get_modes          = tfp410_connector_get_modes,
	.mode_valid         = tfp410_connector_mode_valid,
	.best_encoder       = tfp410_connector_best_encoder,
};

static struct drm_connector *tfp410_connector_create(struct drm_device *dev,
		struct tfp410_module *mod, struct drm_encoder *encoder)
{
	struct tfp410_connector *tfp410_connector;
	struct drm_connector *connector;
	int ret;

	tfp410_connector = kzalloc(sizeof(*tfp410_connector), GFP_KERNEL);
	if (!tfp410_connector) {
		dev_err(dev->dev, "allocation failed\n");
		return NULL;
	}

	tfp410_connector->encoder = encoder;
	tfp410_connector->mod = mod;

	connector = &tfp410_connector->base;

	drm_connector_init(dev, connector, &tfp410_connector_funcs,
			DRM_MODE_CONNECTOR_DVID);
	drm_connector_helper_add(connector, &tfp410_connector_helper_funcs);

	connector->polled = DRM_CONNECTOR_POLL_CONNECT |
			DRM_CONNECTOR_POLL_DISCONNECT;

	connector->interlace_allowed = 0;
	connector->doublescan_allowed = 0;

	ret = drm_mode_connector_attach_encoder(connector, encoder);
	if (ret)
		goto fail;

	drm_sysfs_connector_add(connector);

	return connector;

fail:
	tfp410_connector_destroy(connector);
	return NULL;
}

/*
 * Module:
 */

static int tfp410_modeset_init(struct tilcdc_module *mod, struct drm_device *dev)
{
	struct tfp410_module *tfp410_mod = to_tfp410_module(mod);
	struct tilcdc_drm_private *priv = dev->dev_private;
	struct drm_encoder *encoder;
	struct drm_connector *connector;

	encoder = tfp410_encoder_create(dev, tfp410_mod);
	if (!encoder)
		return -ENOMEM;

	connector = tfp410_connector_create(dev, tfp410_mod, encoder);
	if (!connector)
		return -ENOMEM;

	priv->encoders[priv->num_encoders++] = encoder;
	priv->connectors[priv->num_connectors++] = connector;

	return 0;
}

static void tfp410_destroy(struct tilcdc_module *mod)
{
	struct tfp410_module *tfp410_mod = to_tfp410_module(mod);

	if (tfp410_mod->i2c)
		i2c_put_adapter(tfp410_mod->i2c);

	if (!IS_ERR_VALUE(tfp410_mod->gpio))
		gpio_free(tfp410_mod->gpio);

	tilcdc_module_cleanup(mod);
	kfree(tfp410_mod);
}

static const struct tilcdc_module_ops tfp410_module_ops = {
		.modeset_init = tfp410_modeset_init,
		.destroy = tfp410_destroy,
};

/*
 * Device:
 */

static struct of_device_id tfp410_of_match[];

static ssize_t pinmux_show_state(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tfp410_module *tfp410_mod = platform_get_drvdata(pdev);
	const char *name;

	name = tfp410_mod->selected_state_name;
	if (name == NULL || strlen(name) == 0)
		name = "none";
	return sprintf(buf, "%s\n", name);
}

static ssize_t pinmux_store_state(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct tfp410_module *tfp410_mod = platform_get_drvdata(pdev);
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
	state = pinctrl_lookup_state(tfp410_mod->pinctrl, state_name);
	if (!IS_ERR(state)) {
		err = pinctrl_select_state(tfp410_mod->pinctrl, state);
		if (err != 0)
			dev_err(dev, "Failed to select state %s\n",
					state_name);
	} else {
		dev_err(dev, "Failed to find state %s\n", state_name);
		err = PTR_RET(state);
	}

	if (err == 0) {
		kfree(tfp410_mod->selected_state_name);
		tfp410_mod->selected_state_name = state_name;
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

static int tfp410_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = pdev->dev.of_node;
	struct device_node *i2c_node;
	struct tfp410_module *tfp410_mod;
	struct tilcdc_module *mod;
	struct pinctrl_state *state;
	uint32_t i2c_phandle;
	unsigned long gpioflags;
	char *state_name;
	int ret = -EINVAL;

	/* bail out early if no DT data: */
	if (!node) {
		dev_err(&pdev->dev, "device-tree data is missing\n");
		return -ENXIO;
	}

	tfp410_mod = kzalloc(sizeof(*tfp410_mod), GFP_KERNEL);
	if (!tfp410_mod)
		return -ENOMEM;

	platform_set_drvdata(pdev, tfp410_mod);

	mod = &tfp410_mod->base;

	tilcdc_module_init(mod, "tfp410", &tfp410_module_ops);

	state_name = kmalloc(strlen(PINCTRL_STATE_DEFAULT) + 1,
			GFP_KERNEL);
	if (state_name == NULL) {
		dev_err(dev, "Failed to allocate state name\n");
		ret = -ENOMEM;
		goto fail;
	}
	tfp410_mod->selected_state_name = state_name;
	strcpy(tfp410_mod->selected_state_name, PINCTRL_STATE_DEFAULT);

	tfp410_mod->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(tfp410_mod->pinctrl)) {
		dev_err(dev, "Failed to get pinctrl\n");
		ret = PTR_RET(tfp410_mod->pinctrl);
		goto fail;
	}

	/* try to select default state at first (if it exists) */
	state = pinctrl_lookup_state(tfp410_mod->pinctrl,
			tfp410_mod->selected_state_name);
	if (!IS_ERR(state)) {
		ret = pinctrl_select_state(tfp410_mod->pinctrl, state);
		if (ret != 0) {
			dev_err(dev, "Failed to select default state\n");
			goto fail;
		}
	} else {
		tfp410_mod->selected_state_name = '\0';
	}

	/* Register sysfs hooks */
	ret = sysfs_create_group(&dev->kobj, &pinmux_attr_group);
	if (ret) {
		dev_err(dev, "Failed to create sysfs group\n");
		goto fail;
	}


	if (of_property_read_u32(node, "i2c", &i2c_phandle)) {
		dev_err(&pdev->dev, "could not get i2c bus phandle\n");
		goto fail;
	}

	i2c_node = of_find_node_by_phandle(i2c_phandle);
	if (!i2c_node) {
		dev_err(&pdev->dev, "could not get i2c bus node\n");
		goto fail;
	}

	tfp410_mod->i2c = of_find_i2c_adapter_by_node(i2c_node);
	if (!tfp410_mod->i2c) {
		dev_err(&pdev->dev, "could not get i2c\n");
		goto fail;
	}

	of_node_put(i2c_node);

	tfp410_mod->gpio = of_get_named_gpio_flags(pdev->dev.of_node, "ti,power-gpio",
                       0, &tfp410_mod->ofgpioflags);
	if (IS_ERR_VALUE(tfp410_mod->gpio)) {
		dev_warn(&pdev->dev, "tftp410: No power control GPIO\n");
	} else {
		gpioflags = GPIOF_DIR_OUT;
		if (tfp410_mod->ofgpioflags & OF_GPIO_ACTIVE_LOW) {
			gpioflags |= GPIOF_INIT_LOW;
			dev_info(&pdev->dev, "Power GPIO active low, initial state set to low\n");
		} else {
			gpioflags |= GPIOF_INIT_HIGH;
			dev_info(&pdev->dev, "Power GPIO active high, initial state set to high\n");
		}
		ret = devm_gpio_request_one(&pdev->dev, tfp410_mod->gpio,
				gpioflags, "tfp410:PDN");
		if (ret != 0) {
			dev_err(&pdev->dev, "Failed to request power gpio\n");
			goto fail;
		}
	}

	return 0;

fail:
	tfp410_destroy(mod);
	return ret;
}

static int tfp410_remove(struct platform_device *pdev)
{
	return 0;
}

static struct of_device_id tfp410_of_match[] = {
		{ .compatible = "tilcdc,tfp410", },
		{ },
};
MODULE_DEVICE_TABLE(of, tfp410_of_match);

struct platform_driver tfp410_driver = {
	.probe = tfp410_probe,
	.remove = tfp410_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "tfp410",
		.of_match_table = tfp410_of_match,
	},
};

int __init tilcdc_tfp410_init(void)
{
	return platform_driver_register(&tfp410_driver);
}

void __exit tilcdc_tfp410_fini(void)
{
	platform_driver_unregister(&tfp410_driver);
}
