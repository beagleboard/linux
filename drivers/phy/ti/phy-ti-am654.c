// SPDX-License-Identifier: GPL-2.0
/**
 * PCIe SERDES driver for AM654x SoC
 *
 * Copyright (C) 2018 Texas Instruments
 * Author: Kishon Vijay Abraham I <kishon@ti.com>
 */

#include <dt-bindings/phy/phy.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mux/consumer.h>
#include <linux/of_device.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#define CMU_R07C		0x7c
#define CMU_MASTER_CDN_O	BIT(24)

#define COMLANE_R138		0xb38
#define CONFIG_VERSION_REG_MASK	GENMASK(23, 16)
#define CONFIG_VERSION_REG_SHIFT 16
#define VERSION			0x70

#define COMLANE_R190		0xb90
#define L1_MASTER_CDN_O		BIT(9)

#define COMLANE_R194		0xb94
#define CMU_OK_I_0		BIT(19)

#define SERDES_CTRL		0x1fd0
#define POR_EN			BIT(29)

#define WIZ_LANEXCTL_STS	0x1fe0
#define TX0_ENABLE_OVL		BIT(31)
#define TX0_ENABLE_MASK		GENMASK(30, 29)
#define TX0_ENABLE_SHIFT	29
#define TX0_DISABLE_STATE	0x0
#define TX0_SLEEP_STATE		0x1
#define TX0_SNOOZE_STATE	0x2
#define TX0_ENABLE_STATE	0x3
#define RX0_ENABLE_OVL		BIT(15)
#define RX0_ENABLE_MASK		GENMASK(14, 13)
#define RX0_ENABLE_SHIFT	13
#define RX0_DISABLE_STATE	0x0
#define RX0_SLEEP_STATE		0x1
#define RX0_SNOOZE_STATE	0x2
#define RX0_ENABLE_STATE	0x3

#define WIZ_PLL_CTRL		0x1ff4
#define PLL_ENABLE_OVL		BIT(31)
#define PLL_ENABLE_MASK		GENMASK(30, 29)
#define PLL_ENABLE_SHIFT	29
#define PLL_DISABLE_STATE	0x0
#define PLL_SLEEP_STATE		0x1
#define PLL_SNOOZE_STATE	0x2
#define PLL_ENABLE_STATE	0x3
#define PLL_OK			BIT(28)

#define PLL_LOCK_TIME		100000	/* in microseconds */
#define SLEEP_TIME		100	/* in microseconds */

#define LANE_USB3		0x0
#define LANE_PCIE0_LANE0	0x1

#define LANE_PCIE1_LANE0	0x0
#define LANE_PCIE0_LANE1	0x1

static struct regmap_config serdes_am654_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.fast_io = true,
};

struct serdes_am654 {
	struct regmap		*regmap;
	struct device		*dev;
	struct mux_control	*control;
	bool			busy;
	u32			type;
};

static int serdes_am654_enable_pll(struct serdes_am654 *phy)
{
	u32 mask = PLL_ENABLE_OVL | PLL_ENABLE_MASK;
	u32 val = PLL_ENABLE_OVL | (PLL_ENABLE_STATE << PLL_ENABLE_SHIFT);

	regmap_update_bits(phy->regmap, WIZ_PLL_CTRL, mask, val);

	return regmap_read_poll_timeout(phy->regmap, WIZ_PLL_CTRL, val,
					val & PLL_OK, 1000, PLL_LOCK_TIME);
}

static void serdes_am654_disable_pll(struct serdes_am654 *phy)
{
	u32 mask = PLL_ENABLE_OVL | PLL_ENABLE_MASK;

	regmap_update_bits(phy->regmap, WIZ_PLL_CTRL, mask, 0);
}

static int serdes_am654_enable_txrx(struct serdes_am654 *phy)
{
	u32 mask;
	u32 val;

	/* Enable TX */
	mask = TX0_ENABLE_OVL | TX0_ENABLE_MASK;
	val = TX0_ENABLE_OVL | (TX0_ENABLE_STATE << TX0_ENABLE_SHIFT);
	regmap_update_bits(phy->regmap, WIZ_LANEXCTL_STS, mask, val);

	/* Enable RX */
	mask = RX0_ENABLE_OVL | RX0_ENABLE_MASK;
	val = RX0_ENABLE_OVL | (RX0_ENABLE_STATE << RX0_ENABLE_SHIFT);
	regmap_update_bits(phy->regmap, WIZ_LANEXCTL_STS, mask, val);

	return 0;
}

static int serdes_am654_disable_txrx(struct serdes_am654 *phy)
{
	u32 mask;

	/* Disable TX */
	mask = TX0_ENABLE_OVL | TX0_ENABLE_MASK;
	regmap_update_bits(phy->regmap, WIZ_LANEXCTL_STS, mask, 0);

	/* Disable RX */
	mask = RX0_ENABLE_OVL | RX0_ENABLE_MASK;
	regmap_update_bits(phy->regmap, WIZ_LANEXCTL_STS, mask, 0);

	return 0;
}

static int serdes_am654_power_on(struct phy *x)
{
	struct serdes_am654 *phy = phy_get_drvdata(x);
	struct device *dev = phy->dev;
	int ret;
	u32 val;

	ret = serdes_am654_enable_pll(phy);
	if (ret) {
		dev_err(dev, "Failed to enable PLL\n");
		return ret;
	}

	ret = serdes_am654_enable_txrx(phy);
	if (ret) {
		dev_err(dev, "Failed to enable TX RX\n");
		return ret;
	}

	return regmap_read_poll_timeout(phy->regmap, COMLANE_R194, val,
					val & CMU_OK_I_0, SLEEP_TIME,
					PLL_LOCK_TIME);
}

static int serdes_am654_power_off(struct phy *x)
{
	struct serdes_am654 *phy = phy_get_drvdata(x);

	serdes_am654_disable_txrx(phy);
	serdes_am654_disable_pll(phy);

	return 0;
}

static int serdes_am654_init(struct phy *x)
{
	struct serdes_am654 *phy = phy_get_drvdata(x);
	u32 mask;
	u32 val;

	mask = CONFIG_VERSION_REG_MASK;
	val = VERSION << CONFIG_VERSION_REG_SHIFT;
	regmap_update_bits(phy->regmap, COMLANE_R138, mask, val);

	val = CMU_MASTER_CDN_O;
	regmap_update_bits(phy->regmap, CMU_R07C, val, val);

	val = L1_MASTER_CDN_O;
	regmap_update_bits(phy->regmap, COMLANE_R190, val, val);

	return 0;
}

static int serdes_am654_reset(struct phy *x)
{
	struct serdes_am654 *phy = phy_get_drvdata(x);
	u32 val;

	val = POR_EN;
	regmap_update_bits(phy->regmap, SERDES_CTRL, val, val);
	mdelay(1);
	regmap_update_bits(phy->regmap, SERDES_CTRL, val, 0);

	return 0;
}

struct phy *serdes_am654_xlate(struct device *dev, struct of_phandle_args
				 *args)
{
	struct serdes_am654 *am654_phy;
	struct phy *phy;
	int ret;

	phy = of_phy_simple_xlate(dev, args);
	if (IS_ERR(phy))
		return phy;

	am654_phy = phy_get_drvdata(phy);
	if (am654_phy->type != args->args[0] && am654_phy->busy)
		return ERR_PTR(-EBUSY);

	ret = mux_control_select(am654_phy->control, args->args[1]);
	if (ret) {
		dev_err(dev, "Failed to select SERDES Lane Function\n");
		return ERR_PTR(ret);
	}

	am654_phy->busy = true;
	am654_phy->type = args->args[0];

	return phy;
}

static const struct phy_ops ops = {
	.reset		= serdes_am654_reset,
	.init		= serdes_am654_init,
	.power_on	= serdes_am654_power_on,
	.power_off	= serdes_am654_power_off,
	.owner		= THIS_MODULE,
};

static const struct of_device_id serdes_am654_id_table[] = {
	{
		.compatible = "ti,phy-am654-serdes",
	},
	{}
};
MODULE_DEVICE_TABLE(of, serdes_am654_id_table);

static int serdes_am654_probe(struct platform_device *pdev)
{
	struct phy_provider *phy_provider;
	struct device *dev = &pdev->dev;
	struct serdes_am654 *am654_phy;
	struct mux_control *control;
	struct regmap *regmap;
	struct resource *res;
	void __iomem *base;
	struct phy *phy;

	am654_phy = devm_kzalloc(dev, sizeof(*am654_phy), GFP_KERNEL);
	if (!am654_phy)
		return -ENOMEM;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "serdes");
	base = devm_ioremap_resource(dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	regmap = devm_regmap_init_mmio(dev, base, &serdes_am654_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(dev, "Failed to initialize regmap\n");
		return PTR_ERR(regmap);
	}

	control = devm_mux_control_get(dev, NULL);
	if (IS_ERR(control))
		return PTR_ERR(control);

	am654_phy->dev = dev;
	am654_phy->regmap = regmap;
	am654_phy->control = control;

	pm_runtime_enable(dev);

	phy = devm_phy_create(dev, NULL, &ops);
	if (IS_ERR(phy))
		return PTR_ERR(phy);

	phy_set_drvdata(phy, am654_phy);
	phy_provider = devm_of_phy_provider_register(dev, serdes_am654_xlate);

	return PTR_ERR_OR_ZERO(phy_provider);
}

static int serdes_am654_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);

	return 0;
}

static struct platform_driver serdes_am654_driver = {
	.probe		= serdes_am654_probe,
	.remove		= serdes_am654_remove,
	.driver		= {
		.name	= "phy-am654",
		.of_match_table = serdes_am654_id_table,
	},
};
module_platform_driver(serdes_am654_driver);

MODULE_ALIAS("platform:phy-am654");
MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_DESCRIPTION("TI AM654x SERDES driver");
MODULE_LICENSE("GPL v2");
