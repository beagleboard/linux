// SPDX-License-Identifier: GPL-2.0
/**
 * dwc3-am65.c - glue layer for TI's AM654 SoC's USB controller
 *
 * Copyright (c) 2018 Texas Instruments Incorporated - http://www.ti.com
 *
 */

#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>

#define AM65_USBCTRL_OTG_PWRDN		BIT(8)
#define AM65_USBCTRL_VBUS_DET_EN		BIT(5)
#define AM65_USBCTRL_VBUSVALID_DET_EN	BIT(4)

struct dwc3_am65 {
	struct device *dev;
	struct regmap *ctrl_mmr;
	u32 usbctrl_reg;
};

static int dwc3_am65_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int ret;
	struct dwc3_am65 *am65;

	am65 = devm_kzalloc(dev, sizeof(*am65), GFP_KERNEL);
	if (!am65)
		return -ENOMEM;

	am65->dev = dev;

	am65->ctrl_mmr = syscon_regmap_lookup_by_phandle(np, "syscon-usbctrl");
	if (IS_ERR(am65->ctrl_mmr)) {
		am65->ctrl_mmr = NULL;
	} else {
		if (of_property_read_u32_index(np, "syscon-usbctrl", 1,
					       &am65->usbctrl_reg)) {
			dev_err(dev, "couldn't get usbctrl reg. offset\n");
			return -EINVAL;
		}
	}

	platform_set_drvdata(pdev, am65);

	if (am65->ctrl_mmr) {
		/* disable OTG_PWRDN and turn on VBUS detection */
		regmap_update_bits(am65->ctrl_mmr, am65->usbctrl_reg,
				   AM65_USBCTRL_OTG_PWRDN | AM65_USBCTRL_VBUS_DET_EN | AM65_USBCTRL_VBUS_DET_EN,
				   AM65_USBCTRL_VBUS_DET_EN | AM65_USBCTRL_VBUS_DET_EN);
	}

	pm_runtime_enable(dev);
	pm_runtime_get_sync(dev);

	ret = of_platform_populate(np, NULL, NULL, dev);
	if (ret) {
		pm_runtime_put_sync(dev);
		pm_runtime_disable(dev);
		return ret;
	}

	return 0;
}

static int dwc3_am65_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dwc3_am65 *am65 = platform_get_drvdata(pdev);

	of_platform_depopulate(dev);

	if (am65->ctrl_mmr) {
		/* enable OTG_PWRDN and turn off VBUS detection */
		regmap_update_bits(am65->ctrl_mmr, am65->usbctrl_reg,
				   AM65_USBCTRL_OTG_PWRDN | AM65_USBCTRL_VBUS_DET_EN | AM65_USBCTRL_VBUS_DET_EN,
				   AM65_USBCTRL_OTG_PWRDN);
	}

	pm_runtime_put_sync(dev);
	pm_runtime_disable(dev);

	return 0;
}

static const struct of_device_id of_dwc3_am65_match[] = {
	{ .compatible = "ti,am654-dwc3" },
	{ /* Sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_dwc3_am65_match);

static struct platform_driver dwc3_am65_driver = {
	.probe		= dwc3_am65_probe,
	.remove		= dwc3_am65_remove,
	.driver		= {
		.name	= "dwc3-am65",
		.of_match_table = of_dwc3_am65_match,
	},
};

module_platform_driver(dwc3_am65_driver);
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("DesignWare USB3 TI AM65 Glue Layer");
MODULE_AUTHOR("Roger Quadros <rogerq@ti.com>");
