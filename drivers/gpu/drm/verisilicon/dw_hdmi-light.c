// SPDX-License-Identifier: GPL-2.0

#include <linux/component.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_encoder.h>
#include <drm/drm_of.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/bridge/dw_hdmi.h>

#include "dw_hdmi_tx_phy_gen2.h"

struct light_hdmi {
	struct device *dev;
	struct drm_encoder encoder;
	struct dw_hdmi *dw_hdmi;
	/* TODO: may not be used */
	struct regmap *sysreg;
};

static void dw_hdmi_light_encoder_enable(struct drm_encoder *encoder)
{
}

static int dw_hdmi_light_encoder_atomic_check(struct drm_encoder *encoder,
					      struct drm_crtc_state *crtc_state,
					      struct drm_connector_state *conn_state)
{
	return 0;
}

static struct dw_hdmi_plat_data light_hdmi_drv_data = {
	.mode_valid	= dw_hdmi_tx_phy_gen2_mode_valid,
	.configure_phy	= dw_hdmi_tx_phy_gen2_configure,
};

static const struct of_device_id dw_hdmi_light_dt_ids[] = {
	{
		.compatible = "thead,light-hdmi-tx",
		.data = &light_hdmi_drv_data,
	},
	{
		/* sentinel */
	},
};

static const struct drm_encoder_helper_funcs dw_hdmi_light_encoder_helper_funcs = {
	.enable		= dw_hdmi_light_encoder_enable,
	.atomic_check	= dw_hdmi_light_encoder_atomic_check,
};

static int dw_hdmi_light_bind(struct device *dev, struct device *master,
			      void *data)
{
	int ret = 0;
	struct platform_device *pdev = to_platform_device(dev);
	struct device_node *np = dev->of_node;
	const struct of_device_id *match;
	const struct dw_hdmi_plat_data *plat_data;
	struct light_hdmi *hdmi;
	struct drm_device *drm = data;
	struct drm_encoder *encoder;

	if (!np)
		return -ENODEV;
	hdmi = dev_get_drvdata(dev);

	match = of_match_node(dw_hdmi_light_dt_ids, np);
	if (unlikely(!match))
		return -ENODEV;

	plat_data = match->data;
	hdmi->dev = &pdev->dev;
	encoder = &hdmi->encoder;

	encoder->possible_crtcs = drm_of_find_possible_crtcs(drm, np);
	if (!encoder->possible_crtcs)
		return -EPROBE_DEFER;

	drm_encoder_helper_add(encoder, &dw_hdmi_light_encoder_helper_funcs);
	drm_simple_encoder_init(drm, encoder, DRM_MODE_ENCODER_TMDS);

	hdmi->dw_hdmi = dw_hdmi_bind(pdev, encoder, plat_data);
	if (IS_ERR(hdmi->dw_hdmi)) {
		ret = PTR_ERR(hdmi->dw_hdmi);
		drm_encoder_cleanup(encoder);
	}

	pm_runtime_enable(dev);

	return ret;
}

static void dw_hdmi_light_unbind(struct device *dev, struct device *master,
				 void *data)
{
	struct light_hdmi *hdmi = dev_get_drvdata(dev);

	pm_runtime_disable(dev);

	dw_hdmi_unbind(hdmi->dw_hdmi);
}

static const struct component_ops dw_hdmi_light_ops = {
	.bind	= dw_hdmi_light_bind,
	.unbind = dw_hdmi_light_unbind,
};

static int dw_hdmi_light_probe(struct platform_device *pdev)
{
	struct light_hdmi *hdmi;

	hdmi = devm_kzalloc(&pdev->dev, sizeof(*hdmi), GFP_KERNEL);
	if (!hdmi)
		return -ENOMEM;

	platform_set_drvdata(pdev, hdmi);

	return component_add(&pdev->dev, &dw_hdmi_light_ops);
}

static int dw_hdmi_light_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &dw_hdmi_light_ops);

	return 0;
}

#ifdef CONFIG_PM
static int hdmi_runtime_suspend(struct device *dev)
{
	struct light_hdmi *hdmi = dev_get_drvdata(dev);

	return dw_hdmi_runtime_suspend(hdmi->dw_hdmi);
}

static int hdmi_runtime_resume(struct device *dev)
{
	struct light_hdmi *hdmi = dev_get_drvdata(dev);

	return dw_hdmi_runtime_resume(hdmi->dw_hdmi);
}
#endif

static const struct dev_pm_ops dw_hdmi_pm_ops = {
    SET_LATE_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				 pm_runtime_force_resume)
    SET_RUNTIME_PM_OPS(hdmi_runtime_suspend, hdmi_runtime_resume, NULL)
};

struct platform_driver dw_hdmi_light_platform_driver = {
	.probe	= dw_hdmi_light_probe,
	.remove	= dw_hdmi_light_remove,
	.driver	= {
		.name = "dwhdmi-light",
		.of_match_table = dw_hdmi_light_dt_ids,
		.pm = &dw_hdmi_pm_ops,
	},
};

MODULE_AUTHOR("You Xiao <youxiao.fc@linux.alibaba.com>");
MODULE_DESCRIPTION("Light Platforms Specific DW-HDMI Driver Extention");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:dwhdmi-light");
