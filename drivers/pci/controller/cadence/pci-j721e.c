// SPDX-License-Identifier: GPL-2.0
/*
 * pci-j721e - PCIe controller driver for TI's J721E SoCs
 *
 * Copyright (C) 2020-2023 Texas Instruments Incorporated - http://www.ti.com
 * Author: Kishon Vijay Abraham I <kishon@ti.com>
 */

#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regmap.h>

#include "../../pci.h"
#include "pcie-cadence.h"
#include "pci-j721e.h"

static irqreturn_t j721e_pcie_link_irq_handler(int irq, void *priv)
{
	struct j721e_pcie *pcie = priv;
	struct device *dev = pcie->cdns_pcie->dev;
	u32 reg;

	reg = j721e_pcie_intd_readl(pcie, STATUS_REG_SYS_2);
	if (!(reg & pcie->linkdown_irq_regfield))
		return IRQ_NONE;

	dev_err(dev, "LINK DOWN!\n");

	j721e_pcie_intd_writel(pcie, STATUS_CLR_REG_SYS_2, pcie->linkdown_irq_regfield);
	return IRQ_HANDLED;
}

static void j721e_pcie_config_link_irq(struct j721e_pcie *pcie)
{
	u32 reg;

	reg = j721e_pcie_intd_readl(pcie, ENABLE_REG_SYS_2);
	reg |= pcie->linkdown_irq_regfield;
	j721e_pcie_intd_writel(pcie, ENABLE_REG_SYS_2, reg);
}

void j721e_pcie_remove_link_irq(struct j721e_pcie *pcie)
{
	u32 reg;

	reg = j721e_pcie_intd_readl(pcie, CLEAR_REG_SYS_2);
	reg |= pcie->linkdown_irq_regfield;
	j721e_pcie_intd_writel(pcie, CLEAR_REG_SYS_2, reg);
}
EXPORT_SYMBOL_GPL(j721e_pcie_remove_link_irq);

static int j721e_pcie_start_link(struct cdns_pcie *cdns_pcie)
{
	struct j721e_pcie *pcie = dev_get_drvdata(cdns_pcie->dev);
	u32 reg;

	reg = j721e_pcie_user_readl(pcie, J721E_PCIE_USER_CMD_STATUS);
	reg |= LINK_TRAINING_ENABLE;
	j721e_pcie_user_writel(pcie, J721E_PCIE_USER_CMD_STATUS, reg);

	return 0;
}

static void j721e_pcie_stop_link(struct cdns_pcie *cdns_pcie)
{
	struct j721e_pcie *pcie = dev_get_drvdata(cdns_pcie->dev);
	u32 reg;

	reg = j721e_pcie_user_readl(pcie, J721E_PCIE_USER_CMD_STATUS);
	reg &= ~LINK_TRAINING_ENABLE;
	j721e_pcie_user_writel(pcie, J721E_PCIE_USER_CMD_STATUS, reg);
}

static bool j721e_pcie_link_up(struct cdns_pcie *cdns_pcie)
{
	struct j721e_pcie *pcie = dev_get_drvdata(cdns_pcie->dev);
	u32 reg;

	reg = j721e_pcie_user_readl(pcie, J721E_PCIE_USER_LINKSTATUS);
	reg &= LINK_STATUS;
	if (reg == LINK_UP_DL_COMPLETED)
		return true;

	return false;
}

const struct cdns_pcie_ops j721e_pcie_ops = {
	.start_link = j721e_pcie_start_link,
	.stop_link = j721e_pcie_stop_link,
	.link_up = j721e_pcie_link_up,
};
EXPORT_SYMBOL_GPL(j721e_pcie_ops);

static int j721e_pcie_set_mode(struct j721e_pcie *pcie, struct regmap *syscon,
				unsigned int offset)
{
	struct device *dev = pcie->cdns_pcie->dev;
	u32 mask = J721E_MODE_RC;
	u32 mode = pcie->mode;
	u32 val = 0;
	int ret = 0;

	if (mode == PCI_MODE_RC)
		val = J721E_MODE_RC;

	ret = regmap_update_bits(syscon, offset, mask, val);
	if (ret)
		dev_err(dev, "failed to set pcie mode\n");

	return ret;
}

static int j721e_pcie_set_link_speed(struct j721e_pcie *pcie,
			     struct regmap *syscon, unsigned int offset)
{
	struct device *dev = pcie->cdns_pcie->dev;
	struct device_node *np = dev->of_node;
	int link_speed;
	u32 val = 0;
	int ret;

	link_speed = of_pci_get_max_link_speed(np);
	if (link_speed < 2)
		link_speed = 2;

	val = link_speed - 1;
	ret = regmap_update_bits(syscon, offset, GENERATION_SEL_MASK, val);
	if (ret)
		dev_err(dev, "failed to set link speed\n");

	return ret;
}

static int j721e_pcie_set_lane_count(struct j721e_pcie *pcie,
				     struct regmap *syscon, unsigned int offset)
{
	struct device *dev = pcie->cdns_pcie->dev;
	u32 lanes = pcie->num_lanes;
	u32 mask = BIT(8);
	u32 val = 0;
	int ret;

	if (pcie->max_lanes == 4)
		mask = GENMASK(9, 8);

	val = LANE_COUNT(lanes - 1);
	ret = regmap_update_bits(syscon, offset, mask, val);
	if (ret)
		dev_err(dev, "failed to set link count\n");

	return ret;
}

static int j721e_pcie_ctrl_init(struct j721e_pcie *pcie)
{
	struct device *dev = pcie->cdns_pcie->dev;
	struct device_node *node = dev->of_node;
	struct of_phandle_args args;
	unsigned int offset = 0;
	struct regmap *syscon;
	int ret;

	syscon = syscon_regmap_lookup_by_phandle(node, "ti,syscon-pcie-ctrl");
	if (IS_ERR(syscon)) {
		dev_err(dev, "Unable to get ti,syscon-pcie-ctrl regmap\n");
		return PTR_ERR(syscon);
	}

	/* Do not error out to maintain old DT compatibility */
	ret = of_parse_phandle_with_fixed_args(node, "ti,syscon-pcie-ctrl", 1,
						0, &args);
	if (!ret)
		offset = args.args[0];

	ret = j721e_pcie_set_mode(pcie, syscon, offset);
	if (ret < 0) {
		dev_err(dev, "Failed to set pci mode\n");
		return ret;
	}

	ret = j721e_pcie_set_link_speed(pcie, syscon, offset);
	if (ret < 0) {
		dev_err(dev, "Failed to set link speed\n");
		return ret;
	}

	ret = j721e_pcie_set_lane_count(pcie, syscon, offset);
	if (ret < 0) {
		dev_err(dev, "Failed to set num-lanes\n");
		return ret;
	}

	return 0;
}

static int j721e_pcie_setup_link_interrupts(struct j721e_pcie *pcie)
{
	struct device *dev = pcie->cdns_pcie->dev;
	struct platform_device *pdev = to_platform_device(dev);
	int ret;

	ret = platform_get_irq_byname(pdev, "link_state");
	if (ret < 0)
		return ret;
	ret = devm_request_irq(dev, ret, j721e_pcie_link_irq_handler, 0,
				"j721e-pcie-link-down-irq", pcie);
	if (ret < 0) {
		dev_err(dev, "failed to request link state IRQ %d\n", ret);
		return ret;
	}

	j721e_pcie_config_link_irq(pcie);
	return 0;
}

int j721e_pcie_common_init(struct j721e_pcie *pcie)
{
	struct device *dev = pcie->cdns_pcie->dev;
	struct platform_device *pdev = to_platform_device(dev);
	struct device_node *node = dev->of_node;
	const struct j721e_pcie_data *data;
	void __iomem *base;
	u32 num_lanes;
	int ret;

	data = of_device_get_match_data(dev);
	if (!data)
		return -EINVAL;

	base = devm_platform_ioremap_resource_byname(pdev, "intd_cfg");
	if (IS_ERR(base))
		return PTR_ERR(base);
	pcie->intd_cfg_base = base;

	base = devm_platform_ioremap_resource_byname(pdev, "user_cfg");
	if (IS_ERR(base))
		return PTR_ERR(base);
	pcie->user_cfg_base = base;

	ret = of_property_read_u32(node, "num-lanes", &num_lanes);
	if (ret || num_lanes > data->max_lanes) {
		dev_warn(dev, "num-lanes property not provided or invalid, setting num-lanes to 1\n");
		num_lanes = 1;
	}

	pcie->num_lanes = num_lanes;
	pcie->max_lanes = data->max_lanes;

	if (dma_set_mask_and_coherent(dev, DMA_BIT_MASK(48)))
		return -EINVAL;

	dev_set_drvdata(dev, pcie);

	pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		dev_err(dev, "pm_runtime_get_sync failed\n");
		goto err_get_sync;
	}

	ret = j721e_pcie_ctrl_init(pcie);
	if (ret < 0) {
		dev_err(dev, "pm_runtime_get_sync failed\n");
		goto err_get_sync;
	}

	ret = j721e_pcie_setup_link_interrupts(pcie);
	if (ret < 0) {
		dev_err(dev, "interrupt setup failed\n");
		goto err_get_sync;
	}

	return 0;

err_get_sync:
	pm_runtime_put(dev);
	pm_runtime_disable(dev);

	return ret;

}
EXPORT_SYMBOL_GPL(j721e_pcie_common_init);

void j721e_disable_common_init(struct device *dev)
{
	pm_runtime_put(dev);
	pm_runtime_disable(dev);
}
EXPORT_SYMBOL_GPL(j721e_disable_common_init);

MODULE_LICENSE("GPL v2");
