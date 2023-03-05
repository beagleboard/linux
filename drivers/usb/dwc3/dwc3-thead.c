// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2018, The Linux Foundation. All rights reserved.
 *
 * Inspired by dwc3-of-simple.c
 */

#include <linux/acpi.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/of_clk.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/extcon.h>
#include <linux/of_platform.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/platform_device.h>
#include <linux/phy/phy.h>
#include <linux/usb/of.h>

#include "core.h"

/* USB3_DRD registers */
#define USB_CLK_GATE_STS		0x0
#define USB_LOGIC_ANALYZER_TRACE_STS0	0x4
#define USB_LOGIC_ANALYZER_TRACE_STS1	0x8
#define USB_GPIO			0xc
#define USB_DEBUG_STS0			0x10
#define USB_DEBUG_STS1			0x14
#define USB_DEBUG_STS2			0x18
#define USBCTL_CLK_CTRL0		0x1c
#define USBPHY_CLK_CTRL1		0x20
#define USBPHY_TEST_CTRL0		0x24
#define USBPHY_TEST_CTRL1		0x28
#define USBPHY_TEST_CTRL2		0x2c
#define USBPHY_TEST_CTRL3		0x30
#define USB_SSP_EN			0x34
#define USB_HADDR_SEL			0x38
#define USB_SYS			0x3c
#define USB_HOST_STATUS		0x40
#define USB_HOST_CTRL			0x44
#define USBPHY_HOST_CTRL		0x48
#define USBPHY_HOST_STATUS		0x4c
#define USB_TEST_REG0			0x50
#define USB_TEST_REG1			0x54
#define USB_TEST_REG2			0x58
#define USB_TEST_REG3			0x5c

/* Bit fields */
/* USB_SYS */
#define TEST_POWERDOWN_SSP	BIT(2)
#define TEST_POWERDOWN_HSP	BIT(1)
#define COMMONONN		BIT(0)

/* USB_SSP_EN */
#define REF_SSP_EN		BIT(0)

/* USBPHY_HOST_CTRL */
#define HOST_U2_PORT_DISABLE	BIT(6)
#define HOST_U3_PORT_DISABLE	BIT(5)

/* MISC_SYSREG registers */
#define USB3_DRD_SWRST			0x14

/* Bit fields */
/* USB3_DRD_SWRST */
#define USB3_DRD_VCCRST		BIT(2)
#define USB3_DRD_PHYRST		BIT(1)
#define USB3_DRD_PRST		BIT(0)
#define USB3_DRD_MASK		GENMASK(2, 0)

struct dwc3_thead {
	struct device		*dev;
	struct platform_device	*dwc3;

	struct regmap		*usb0_apb;
	struct regmap		*misc_sysreg;

	struct extcon_dev	*edev;
	struct extcon_dev	*host_edev;
	struct notifier_block	vbus_nb;
	struct notifier_block	host_nb;

	struct gpio_desc	*hubswitch;
	struct regulator	*hub1v2;
	struct regulator	*hub5v;
	struct regulator	*vbus;

	enum usb_dr_mode	mode;
	bool			is_suspended;
	bool			pm_suspended;
};

static int dwc3_thead_vbus_notifier(struct notifier_block *nb,
				   unsigned long event, void *ptr)
{
	struct dwc3_thead *thead = container_of(nb, struct dwc3_thead, vbus_nb);

	/* enable vbus override for device mode */
	thead->mode = event ? USB_DR_MODE_PERIPHERAL : USB_DR_MODE_HOST;

	return NOTIFY_DONE;
}

static int dwc3_thead_host_notifier(struct notifier_block *nb,
				   unsigned long event, void *ptr)
{
	struct dwc3_thead *thead = container_of(nb, struct dwc3_thead, host_nb);

	/* disable vbus override in host mode */
	thead->mode = event ? USB_DR_MODE_HOST : USB_DR_MODE_PERIPHERAL;

	return NOTIFY_DONE;
}

static int dwc3_thead_register_extcon(struct dwc3_thead *thead)
{
	struct device		*dev = thead->dev;
	struct extcon_dev	*host_edev;
	int			ret;

	if (!of_property_read_bool(dev->of_node, "extcon"))
		return 0;

	thead->edev = extcon_get_edev_by_phandle(dev, 0);
	if (IS_ERR(thead->edev))
		return PTR_ERR(thead->edev);

	thead->vbus_nb.notifier_call = dwc3_thead_vbus_notifier;

	thead->host_edev = extcon_get_edev_by_phandle(dev, 1);
	if (IS_ERR(thead->host_edev))
		thead->host_edev = NULL;

	ret = devm_extcon_register_notifier(dev, thead->edev, EXTCON_USB,
					    &thead->vbus_nb);
	if (ret < 0) {
		dev_err(dev, "VBUS notifier register failed\n");
		return ret;
	}

	if (thead->host_edev)
		host_edev = thead->host_edev;
	else
		host_edev = thead->edev;

	thead->host_nb.notifier_call = dwc3_thead_host_notifier;
	ret = devm_extcon_register_notifier(dev, host_edev, EXTCON_USB_HOST,
					    &thead->host_nb);
	if (ret < 0) {
		dev_err(dev, "Host notifier register failed\n");
		return ret;
	}

	/* Update initial VBUS override based on extcon state */
	if (extcon_get_state(thead->edev, EXTCON_USB) ||
	    !extcon_get_state(host_edev, EXTCON_USB_HOST))
		dwc3_thead_vbus_notifier(&thead->vbus_nb, true, thead->edev);
	else
		dwc3_thead_vbus_notifier(&thead->vbus_nb, false, thead->edev);

	return 0;
}

static int dwc3_thead_suspend(struct dwc3_thead *thead)
{
	return 0;
}

static int dwc3_thead_resume(struct dwc3_thead *thead)
{
	return 0;
}

static int dwc3_thead_of_register_core(struct platform_device *pdev)
{
	struct dwc3_thead	*thead = platform_get_drvdata(pdev);
	struct device_node	*np = pdev->dev.of_node, *dwc3_np;
	struct device		*dev = &pdev->dev;
	int			ret;

	dwc3_np = of_get_child_by_name(np, "dwc3");
	if (!dwc3_np) {
		dev_err(dev, "failed to find dwc3 core child\n");
		return -ENODEV;
	}

	ret = of_platform_populate(np, NULL, NULL, dev);
	if (ret) {
		dev_err(dev, "failed to register dwc3 core - %d\n", ret);
		goto node_put;
	}

	thead->dwc3 = of_find_device_by_node(dwc3_np);
	if (!thead->dwc3) {
		ret = -ENODEV;
		dev_err(dev, "failed to get dwc3 platform device\n");
	}

node_put:
	of_node_put(dwc3_np);

	return ret;
}

static void dwc3_thead_savepower_u3phy(struct dwc3_thead *thead)
{
	struct device *dev = thead->dev;
	int reg;

	/* config usb top within USB ctrl & PHY reset */
	regmap_update_bits(thead->misc_sysreg, USB3_DRD_SWRST,
				USB3_DRD_MASK, USB3_DRD_PRST);

	/*
	 * dwc reg also need to be configed to save power
	 * 1 set USB_SYS[COMMONONN] while GCTL[SOFITPSYNC]=1
	 *   notice GCTL[SOFITPSYNC] should be mutex with GFLADJ[LPM_SEL].
	 * 2 enable GUSB3PIPECLT[SUSPENDEN]
	 */
	regmap_update_bits(thead->usb0_apb, USB_SYS,
				COMMONONN, COMMONONN);
	regmap_update_bits(thead->usb0_apb, USB_SSP_EN,
				REF_SSP_EN, REF_SSP_EN);
	regmap_write(thead->usb0_apb, USB_HOST_CTRL, 0x1101);

	regmap_update_bits(thead->misc_sysreg, USB3_DRD_SWRST,
				USB3_DRD_MASK, USB3_DRD_MASK);

	regmap_read(thead->usb0_apb, USB_SYS, &reg);
	dev_dbg(dev, "usb_sys:0x%x\n", reg);

	regmap_read(thead->usb0_apb, USB_HOST_CTRL, &reg);
	dev_dbg(dev, "host_ctrl:0x%x\n", reg);

	regmap_read(thead->misc_sysreg, USB3_DRD_SWRST, &reg);
	dev_dbg(dev, "drd_swrst:0x%x\n", reg);
}

static int dwc3_thead_probe(struct platform_device *pdev)
{
	struct device		*dev = &pdev->dev;
	struct device_node	*np = dev->of_node;
	struct dwc3_thead	*thead;
	int			ret;

	thead = devm_kzalloc(&pdev->dev, sizeof(*thead), GFP_KERNEL);
	if (!thead)
		return -ENOMEM;

	platform_set_drvdata(pdev, thead);
	thead->dev = &pdev->dev;

	thead->hubswitch = devm_gpiod_get(dev, "hubswitch", GPIOD_OUT_HIGH);
	if (IS_ERR(thead->hubswitch))
		dev_dbg(dev, "no need to get hubswitch GPIO\n");

	thead->vbus = devm_regulator_get(dev, "vbus");
	if (IS_ERR(thead->vbus))
		dev_dbg(dev, "no need to get vbus\n");
	else
		regulator_enable(thead->vbus);

	thead->hub1v2 = devm_regulator_get(dev, "hub1v2");
	if (IS_ERR(thead->hub1v2))
		dev_dbg(dev, "no need to set hub1v2\n");
	else
		regulator_enable(thead->hub1v2);

	thead->hub5v = devm_regulator_get(dev, "hub5v");
	if (IS_ERR(thead->hub5v))
		dev_dbg(dev, "no need to set hub5v\n");
	else
		regulator_enable(thead->hub5v);

	thead->misc_sysreg = syscon_regmap_lookup_by_phandle(np, "usb3-misc-regmap");
	if (IS_ERR(thead->misc_sysreg))
		return PTR_ERR(thead->misc_sysreg);

	thead->usb0_apb = syscon_regmap_lookup_by_phandle(np, "usb3-drd-regmap");
	if (IS_ERR(thead->usb0_apb))
		return PTR_ERR(thead->usb0_apb);

	dwc3_thead_savepower_u3phy(thead);

	ret = dwc3_thead_of_register_core(pdev);
	if (ret) {
		dev_err(dev, "failed to register DWC3 Core, err=%d\n", ret);
		goto depopulate;
	}

	/* fixme: need to verify because hw can't support detect event */
	ret = dwc3_thead_register_extcon(thead);
	if (ret)
		goto err;


	device_init_wakeup(&pdev->dev, 1);
	thead->is_suspended = false;
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_forbid(dev);
	dev_info(dev, "light dwc3 probe ok!\n");

	return 0;

depopulate:
	of_platform_depopulate(&pdev->dev);
err:
	return ret;
}

static int dwc3_thead_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	pm_runtime_allow(dev);
	pm_runtime_disable(dev);

	return 0;
}

static int __maybe_unused dwc3_thead_pm_suspend(struct device *dev)
{
	struct dwc3_thead *thead = dev_get_drvdata(dev);
	int ret = 0;

	ret = dwc3_thead_suspend(thead);
	if (!ret)
		thead->pm_suspended = true;

	return ret;
}

static int __maybe_unused dwc3_thead_pm_resume(struct device *dev)
{
	struct dwc3_thead *thead = dev_get_drvdata(dev);
	int ret;

	ret = dwc3_thead_resume(thead);
	if (!ret)
		thead->pm_suspended = false;

	return ret;
}

static int __maybe_unused dwc3_thead_runtime_suspend(struct device *dev)
{
	struct dwc3_thead *thead = dev_get_drvdata(dev);

	return dwc3_thead_suspend(thead);
}

static int __maybe_unused dwc3_thead_runtime_resume(struct device *dev)
{
	struct dwc3_thead *thead = dev_get_drvdata(dev);

	return dwc3_thead_resume(thead);
}

static const struct dev_pm_ops dwc3_thead_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(dwc3_thead_pm_suspend, dwc3_thead_pm_resume)
	SET_RUNTIME_PM_OPS(dwc3_thead_runtime_suspend, dwc3_thead_runtime_resume,
			   NULL)
};

static const struct of_device_id dwc3_thead_of_match[] = {
	{ .compatible = "thead,dwc3" },
	{ },
};
MODULE_DEVICE_TABLE(of, dwc3_thead_of_match);

static struct platform_driver dwc3_thead_driver = {
	.probe		= dwc3_thead_probe,
	.remove		= dwc3_thead_remove,
	.driver		= {
		.name	= "dwc3-thead",
	/*
	 * fixme: need to verify because hw can't support plug event
	 */
//		.pm	= &dwc3_thead_dev_pm_ops,
		.of_match_table	= dwc3_thead_of_match,
	},
};

module_platform_driver(dwc3_thead_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("DesignWare DWC3 Thead Glue Driver");
