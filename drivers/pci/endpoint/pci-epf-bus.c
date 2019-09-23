// SPDX-License-Identifier: GPL-2.0
/**
 * PCI Endpoint *Function* Bus Driver
 *
 * Copyright (C) 2019 Texas Instruments
 * Author: Kishon Vijay Abraham I <kishon@ti.com>
 */

#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pci-epf.h>
#include <linux/platform_device.h>

static int pci_epf_bus_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = of_node_get(dev->of_node);
	struct device_node *child;
	struct pci_epf *epf;

	for_each_child_of_node(node, child) {
		epf = devm_pci_epf_of_create(dev, child);
		if (IS_ERR(epf)) {
			dev_err(dev, "Failed to create PCI EPF device %s\n",
				node->full_name);
			of_node_put(child);
			break;
		}
	}
	of_node_put(node);

	return 0;
}

static const struct of_device_id pci_epf_bus_id_table[] = {
	{ .compatible = "pci-epf-bus" },
	{}
};
MODULE_DEVICE_TABLE(of, pci_epf_bus_id_table);

static struct platform_driver pci_epf_bus_driver = {
	.probe		= pci_epf_bus_probe,
	.driver		= {
		.name	= "pci-epf-bus",
		.of_match_table = of_match_ptr(pci_epf_bus_id_table),
	},
};

module_platform_driver(pci_epf_bus_driver);

MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_DESCRIPTION("PCI EPF Bus Driver");
MODULE_LICENSE("GPL v2");
