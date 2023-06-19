// SPDX-License-Identifier: GPL-2.0
/*
 * pci-j721e-ep - PCIe end-point controller driver for TI's J721E SoCs
 *
 * Copyright (C) 2023 Texas Instruments Incorporated - http://www.ti.com
 * Author: Kishon Vijay Abraham I <kishon@ti.com>
 */

#include <linux/clk.h>
#include <linux/of_device.h>

#include "pcie-cadence.h"
#include "pci-j721e.h"

static const struct j721e_pcie_data j721e_pcie_ep_data = {
	.mode = PCI_MODE_EP,
	.linkdown_irq_regfield = LINK_DOWN,
};

static const struct j721e_pcie_data j7200_pcie_ep_data = {
	.mode = PCI_MODE_EP,
	.quirk_detect_quiet_flag = true,
	.quirk_disable_flr = true,
};

static const struct j721e_pcie_data am64_pcie_ep_data = {
	.mode = PCI_MODE_EP,
	.linkdown_irq_regfield = J7200_LINK_DOWN,
};

static const struct j721e_pcie_data j784s4_pcie_ep_data = {
	.mode = PCI_MODE_EP,
	.linkdown_irq_regfield = LINK_DOWN,
	.max_lanes = 4,
};

static const struct of_device_id of_j721e_pcie_ep_match[] = {
	{
		.compatible = "ti,j721e-pcie-ep",
		.data = &j721e_pcie_ep_data,
	},
	{
		.compatible = "ti,j7200-pcie-ep",
		.data = &j7200_pcie_ep_data,
	},
	{
		.compatible = "ti,am64-pcie-ep",
		.data = &am64_pcie_ep_data,
	},
	{
		.compatible = "ti,j784s4-pcie-ep",
		.data = &j784s4_pcie_ep_data,
	},
	{},
};
MODULE_DEVICE_TABLE(of, of_j721e_pcie_ep_match);

static int j721e_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct j721e_pcie_data *data;
	struct cdns_pcie *cdns_pcie;
	struct j721e_pcie *pcie;
	struct cdns_pcie_ep *ep = NULL;
	int ret;

	data = of_device_get_match_data(dev);
	if (!data)
		return -EINVAL;

	pcie = devm_kzalloc(dev, sizeof(*pcie), GFP_KERNEL);
	if (!pcie)
		return -ENOMEM;

	ep = devm_kzalloc(dev, sizeof(*ep), GFP_KERNEL);
	if (!ep)
		return -ENOMEM;

	ep->quirk_detect_quiet_flag = data->quirk_detect_quiet_flag;
	ep->quirk_disable_flr = data->quirk_disable_flr;

	cdns_pcie = &ep->pcie;
	cdns_pcie->dev = dev;
	cdns_pcie->ops = &j721e_pcie_ops;
	pcie->cdns_pcie = cdns_pcie;

	pcie->mode = PCI_MODE_EP;
	pcie->linkdown_irq_regfield = data->linkdown_irq_regfield;

	ret = j721e_pcie_common_init(pcie);
	if (ret)
		return ret;

	ret = cdns_pcie_init_phy(dev, cdns_pcie);
	if (ret) {
		dev_err(dev, "Failed to init phy\n");
		goto err_get_sync;
	}

	ret = cdns_pcie_ep_setup(ep);
	if (ret < 0)
		goto err_pcie_setup;


	return 0;

err_pcie_setup:
	cdns_pcie_disable_phy(cdns_pcie);

err_get_sync:
	j721e_disable_common_init(dev);

	return ret;
}

static int j721e_pcie_remove(struct platform_device *pdev)
{
	struct j721e_pcie *pcie = platform_get_drvdata(pdev);
	struct cdns_pcie *cdns_pcie = pcie->cdns_pcie;
	struct device *dev = &pdev->dev;

	j721e_pcie_remove_link_irq(pcie);
	cdns_pcie_stop_link(cdns_pcie);
	cdns_pcie_deinit_phy(cdns_pcie);
	j721e_disable_common_init(dev);

	return 0;
}

static struct platform_driver j721e_pcie_ep_driver = {
	.probe  = j721e_pcie_probe,
	.remove = j721e_pcie_remove,
	.driver = {
		.name	= "j721e-pcie-ep",
		.of_match_table = of_j721e_pcie_ep_match,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(j721e_pcie_ep_driver);
MODULE_AUTHOR("Kishon Vijay Abraham I <kishon@ti.com>");
MODULE_LICENSE("GPL v2");
