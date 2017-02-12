/*
 * PRU-ICSS platform driver for various TI SoCs
 *
 * Copyright (C) 2014-2017 Texas Instruments Incorporated - http://www.ti.com/
 *	Suman Anna <s-anna@ti.com>
 *	Andrew F. Davis <afd@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/of_device.h>

#include "pruss.h"

/**
 * struct pruss_private_data - PRUSS driver private data
 * @aux_data: auxiliary data used for creating the child nodes
 * @has_no_sharedram: flag to indicate the absence of PRUSS Shared Data RAM
 */
struct pruss_private_data {
	struct of_dev_auxdata *aux_data;
	bool has_no_sharedram;
};

/**
 * struct pruss_match_private_data - private data to handle multiple instances
 * @device_name: device name of the PRUSS instance
 * @priv_data: PRUSS driver private data for this PRUSS instance
 */
struct pruss_match_private_data {
	const char *device_name;
	struct pruss_private_data *priv_data;
};

static const struct of_device_id pruss_of_match[];

static const
struct pruss_private_data *pruss_get_private_data(struct platform_device *pdev)
{
	const struct pruss_match_private_data *data;
	const struct of_device_id *match;

	match = of_match_device(pruss_of_match, &pdev->dev);
	if (!match)
		return ERR_PTR(-ENODEV);

	data = match->data;
	for (; data && data->device_name; data++) {
		if (!strcmp(dev_name(&pdev->dev), data->device_name))
			return data->priv_data;
	}

	return NULL;
}

static int pruss_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct pruss *pruss;
	struct resource *res;
	int ret, i;
	const struct pruss_private_data *data;
	const char *mem_names[PRUSS_MEM_MAX] = { "dram0", "dram1", "shrdram2",
						 "cfg", "iep", "mii_rt" };

	if (!node) {
		dev_err(dev, "Non-DT platform device not supported\n");
		return -ENODEV;
	}

	data = pruss_get_private_data(pdev);
	if (IS_ERR_OR_NULL(data)) {
		dev_err(dev, "missing private data\n");
		return -ENODEV;
	}

	ret = dma_set_coherent_mask(dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(dev, "dma_set_coherent_mask: %d\n", ret);
		return ret;
	}

	pruss = devm_kzalloc(dev, sizeof(*pruss), GFP_KERNEL);
	if (!pruss)
		return -ENOMEM;

	pruss->dev = dev;

	for (i = 0; i < ARRAY_SIZE(mem_names); i++) {
		if (data->has_no_sharedram && !strcmp(mem_names[i], "shrdram2"))
			continue;
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   mem_names[i]);
		pruss->mem_regions[i].va = devm_ioremap_resource(dev, res);
		if (IS_ERR(pruss->mem_regions[i].va)) {
			dev_err(dev, "failed to parse and map memory resource %d %s\n",
				i, mem_names[i]);
			return PTR_ERR(pruss->mem_regions[i].va);
		}
		pruss->mem_regions[i].pa = res->start;
		pruss->mem_regions[i].size = resource_size(res);

		dev_dbg(dev, "memory %8s: pa %pa size 0x%x va %p\n",
			mem_names[i], &pruss->mem_regions[i].pa,
			pruss->mem_regions[i].size, pruss->mem_regions[i].va);
	}

	platform_set_drvdata(pdev, pruss);

	dev_info(&pdev->dev, "creating PRU cores and other child platform devices\n");
	ret = of_platform_populate(node, NULL, data->aux_data, &pdev->dev);
	if (ret)
		dev_err(dev, "of_platform_populate failed\n");

	return ret;
}

static int pruss_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	dev_info(dev, "remove PRU cores and other child platform devices\n");
	of_platform_depopulate(dev);

	return 0;
}

/*
 * auxdata lookup table for giving specific device names to PRU platform
 * devices. The device names are used in the driver to find private data
 * specific to a PRU-core such as an id and a firmware name etc, especially
 * needed when there are multiple PRUSS instances present on a SoC.
 * XXX: The auxdata in general is not a recommended usage, and this should
 *      eventually be eliminated. The current usage allows us to define the
 *      PRU device names with an identifier like xxxxxxxx.pru0 agnostic of
 *      name defined in device tree.
 */
static struct of_dev_auxdata am335x_pruss_rproc_auxdata_lookup[] = {
	OF_DEV_AUXDATA("ti,am3356-pru", 0x4a334000, "4a334000.pru0", NULL),
	OF_DEV_AUXDATA("ti,am3356-pru", 0x4a338000, "4a338000.pru1", NULL),
	{ /* sentinel */ },
};

static struct of_dev_auxdata am437x_pruss1_rproc_auxdata_lookup[] = {
	OF_DEV_AUXDATA("ti,am4376-pru", 0x54434000, "54434000.pru0", NULL),
	OF_DEV_AUXDATA("ti,am4376-pru", 0x54438000, "54438000.pru1", NULL),
	{ /* sentinel */ },
};

static struct of_dev_auxdata am437x_pruss0_rproc_auxdata_lookup[] = {
	OF_DEV_AUXDATA("ti,am4376-pru", 0x54474000, "54474000.pru0", NULL),
	OF_DEV_AUXDATA("ti,am4376-pru", 0x54478000, "54478000.pru1", NULL),
	{ /* sentinel */ },
};

static struct of_dev_auxdata am57xx_pruss1_rproc_auxdata_lookup[] = {
	OF_DEV_AUXDATA("ti,am5728-pru", 0x4b234000, "4b234000.pru0", NULL),
	OF_DEV_AUXDATA("ti,am5728-pru", 0x4b238000, "4b238000.pru1", NULL),
	{ /* sentinel */ },
};

static struct of_dev_auxdata am57xx_pruss2_rproc_auxdata_lookup[] = {
	OF_DEV_AUXDATA("ti,am5728-pru", 0x4b2b4000, "4b2b4000.pru0", NULL),
	OF_DEV_AUXDATA("ti,am5728-pru", 0x4b2b8000, "4b2b8000.pru1", NULL),
	{ /* sentinel */ },
};

static struct of_dev_auxdata k2g_pruss0_rproc_auxdata_lookup[] = {
	OF_DEV_AUXDATA("ti,k2g-pru", 0x20ab4000, "20ab4000.pru0", NULL),
	OF_DEV_AUXDATA("ti,k2g-pru", 0x20ab8000, "20ab8000.pru1", NULL),
	{ /* sentinel */ },
};

static struct of_dev_auxdata k2g_pruss1_rproc_auxdata_lookup[] = {
	OF_DEV_AUXDATA("ti,k2g-pru", 0x20af4000, "20af4000.pru0", NULL),
	OF_DEV_AUXDATA("ti,k2g-pru", 0x20af8000, "20af8000.pru1", NULL),
	{ /* sentinel */ },
};

/* instance-specific driver private data */
static struct pruss_private_data am335x_pruss_priv_data = {
	.aux_data = am335x_pruss_rproc_auxdata_lookup,
};

static struct pruss_private_data am437x_pruss1_priv_data = {
	.aux_data = am437x_pruss1_rproc_auxdata_lookup,
};

static struct pruss_private_data am437x_pruss0_priv_data = {
	.aux_data = am437x_pruss0_rproc_auxdata_lookup,
	.has_no_sharedram = true,
};

static struct pruss_private_data am57xx_pruss1_priv_data = {
	.aux_data = am57xx_pruss1_rproc_auxdata_lookup,
};

static struct pruss_private_data am57xx_pruss2_priv_data = {
	.aux_data = am57xx_pruss2_rproc_auxdata_lookup,
};

static struct pruss_private_data k2g_pruss0_priv_data = {
	.aux_data = k2g_pruss0_rproc_auxdata_lookup,
};

static struct pruss_private_data k2g_pruss1_priv_data = {
	.aux_data = k2g_pruss1_rproc_auxdata_lookup,
};

static struct pruss_match_private_data am335x_match_data[] = {
	{
		.device_name	= "4a300000.pruss",
		.priv_data	= &am335x_pruss_priv_data,
	},
	{
		/* sentinel */
	},
};

static struct pruss_match_private_data am437x_match_data[] = {
	{
		.device_name	= "54400000.pruss",
		.priv_data	= &am437x_pruss1_priv_data,
	},
	{
		.device_name	= "54440000.pruss",
		.priv_data	= &am437x_pruss0_priv_data,
	},
	{
		/* sentinel */
	},
};

static struct pruss_match_private_data am57xx_match_data[] = {
	{
		.device_name	= "4b200000.pruss",
		.priv_data	= &am57xx_pruss1_priv_data,
	},
	{
		.device_name	= "4b280000.pruss",
		.priv_data	= &am57xx_pruss2_priv_data,
	},
	{
		/* sentinel */
	},
};

static struct pruss_match_private_data k2g_match_data[] = {
	{
		.device_name	= "20a80000.pruss",
		.priv_data	= &k2g_pruss0_priv_data,
	},
	{
		.device_name	= "20ac0000.pruss",
		.priv_data	= &k2g_pruss1_priv_data,
	},
	{
		/* sentinel */
	},
};

static const struct of_device_id pruss_of_match[] = {
	{ .compatible = "ti,am3356-pruss", .data = &am335x_match_data, },
	{ .compatible = "ti,am4376-pruss", .data = &am437x_match_data, },
	{ .compatible = "ti,am5728-pruss", .data = &am57xx_match_data, },
	{ .compatible = "ti,k2g-pruss", .data = &k2g_match_data, },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, pruss_of_match);

static struct platform_driver pruss_driver = {
	.driver = {
		.name = "ti-pruss",
		.of_match_table = pruss_of_match,
	},
	.probe  = pruss_probe,
	.remove = pruss_remove,
};
module_platform_driver(pruss_driver);

MODULE_AUTHOR("Suman Anna <s-anna@ti.com>");
MODULE_DESCRIPTION("PRU-ICSS Subsystem Driver");
MODULE_LICENSE("GPL v2");
