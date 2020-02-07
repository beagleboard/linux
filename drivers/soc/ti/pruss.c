// SPDX-License-Identifier: GPL-2.0-only
/*
 * PRU-ICSS platform driver for various TI SoCs
 *
 * Copyright (C) 2014-2020 Texas Instruments Incorporated - http://www.ti.com/
 *	Suman Anna <s-anna@ti.com>
 *	Andrew F. Davis <afd@ti.com>
 */

#include <linux/clk-provider.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/pruss_driver.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#define PRUSS_CFG_IEPCLK	0x30
#define ICSSG_CFG_CORE_SYNC	0x3c

/**
 * struct pruss_private_data - PRUSS driver private data
 * @has_no_sharedram: flag to indicate the absence of PRUSS Shared Data RAM
 */
struct pruss_private_data {
	bool has_no_sharedram;
};

/**
 * struct pruss_match_private_data - private data to handle multiple instances
 * @device_name: device name of the PRUSS instance
 * @priv_data: PRUSS driver private data for this PRUSS instance
 */
struct pruss_match_private_data {
	const char *device_name;
	const struct pruss_private_data *priv_data;
};

static void pruss_of_free_clk_provider(void *data)
{
	struct device_node *clk_mux_np = data;

	of_clk_del_provider(clk_mux_np);
	of_node_put(clk_mux_np);
}

static int pruss_clk_mux_setup(struct pruss *pruss, struct clk *clk_mux,
			       char *mux_name, unsigned int reg_offset)
{
	unsigned int num_parents;
	const char **parent_names;
	void __iomem *reg;
	int ret;
	char *clk_mux_name = NULL;
	struct device_node *clk_mux_np;

	clk_mux_np = of_get_child_by_name(pruss->dev->of_node, mux_name);
	if (!clk_mux_np)
		return -EINVAL;

	num_parents = of_clk_get_parent_count(clk_mux_np);
	if (num_parents < 1) {
		dev_err(pruss->dev, "mux-clock %pOF must have parents\n",
			clk_mux_np);
		return -EINVAL;
	}

	parent_names = devm_kcalloc(pruss->dev, sizeof(char *), num_parents,
				    GFP_KERNEL);
	if (!parent_names)
		return -ENOMEM;

	of_clk_parent_fill(clk_mux_np, parent_names, num_parents);

	clk_mux_name = devm_kasprintf(pruss->dev, GFP_KERNEL, "%s.%pOFn",
				      dev_name(pruss->dev), clk_mux_np);
	if (!clk_mux_name)
		return -ENOMEM;

	reg = pruss->cfg_base + reg_offset;
	/*
	 * WARN: dev must be NULL to avoid recursive incrementing
	 * of module refcnt
	 */
	clk_mux = clk_register_mux(NULL, clk_mux_name, parent_names,
				   num_parents, 0, reg, 0, 1, 0, NULL);
	if (IS_ERR(clk_mux))
		return PTR_ERR(clk_mux);

	ret = devm_add_action_or_reset(pruss->dev,
				       (void(*)(void *))clk_unregister_mux,
				       clk_mux);
	if (ret) {
		dev_err(pruss->dev, "failed to add clkmux reset action %d",
			ret);
		return ret;
	}

	ret = of_clk_add_provider(clk_mux_np, of_clk_src_simple_get, clk_mux);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(pruss->dev, pruss_of_free_clk_provider,
				       clk_mux_np);
	if (ret)
		dev_err(pruss->dev, "failed to add clkmux reset action %d",
			ret);

	return ret;
}

static const
struct pruss_private_data *pruss_get_private_data(struct platform_device *pdev)
{
	const struct pruss_match_private_data *data;

	if (!of_device_is_compatible(pdev->dev.of_node, "ti,am4376-pruss"))
		return NULL;

	data = of_device_get_match_data(&pdev->dev);
	for (; data && data->device_name; data++) {
		if (!strcmp(dev_name(&pdev->dev), data->device_name))
			return data->priv_data;
	}

	return ERR_PTR(-ENODEV);
}

static const struct regmap_config syscon_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
};

static int pruss_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct device_node *np;
	struct pruss *pruss;
	struct resource res;
	int ret, i, index;
	const struct pruss_private_data *data;
	const char *mem_names[PRUSS_MEM_MAX] = { "dram0", "dram1", "shrdram2" };
	struct regmap_config syscon_config = syscon_regmap_config;

	if (!node) {
		dev_err(dev, "Non-DT platform device not supported\n");
		return -ENODEV;
	}

	data = pruss_get_private_data(pdev);
	if (IS_ERR(data)) {
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

	np = of_get_child_by_name(node, "memories");
	if (!np) {
		dev_err(dev, "%pOF is missing memories node\n", np);
		return -ENODEV;
	}

	for (i = 0; i < ARRAY_SIZE(mem_names); i++) {
		if (data && data->has_no_sharedram &&
		    !strcmp(mem_names[i], "shrdram2"))
			continue;

		index = of_property_match_string(np, "reg-names", mem_names[i]);
		if (index < 0) {
			of_node_put(np);
			return index;
		}

		if (of_address_to_resource(np, index, &res)) {
			of_node_put(np);
			return -EINVAL;
		}

		pruss->mem_regions[i].va = devm_ioremap(dev, res.start,
							resource_size(&res));
		if (!pruss->mem_regions[i].va) {
			dev_err(dev, "failed to parse and map memory resource %d %s\n",
				i, mem_names[i]);
			of_node_put(np);
			return -ENOMEM;
		}
		pruss->mem_regions[i].pa = res.start;
		pruss->mem_regions[i].size = resource_size(&res);

		dev_dbg(dev, "memory %8s: pa %pa size 0x%zx va %pK\n",
			mem_names[i], &pruss->mem_regions[i].pa,
			pruss->mem_regions[i].size, pruss->mem_regions[i].va);
	}
	of_node_put(np);

	platform_set_drvdata(pdev, pruss);

	pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		dev_err(dev, "couldn't enable module\n");
		pm_runtime_put_noidle(dev);
		goto rpm_disable;
	}

	np = of_get_child_by_name(node, "cfg");
	if (!np) {
		dev_err(dev, "%pOF is missing cfg node\n", np);
		ret = -ENODEV;
		goto rpm_put;
	}

	if (of_address_to_resource(np, 0, &res)) {
		ret = -ENOMEM;
		of_node_put(np);
		goto rpm_put;
	}
	of_node_put(np);

	pruss->cfg_base = devm_ioremap(dev, res.start, resource_size(&res));
	if (!pruss->cfg_base) {
		ret = -ENOMEM;
		goto rpm_put;
	}

	if (!of_device_is_compatible(pdev->dev.of_node, "ti,am654-icssg") &&
	    !of_device_is_compatible(pdev->dev.of_node, "ti,j721e-icssg"))
		goto skip_mux;

	ret = pruss_clk_mux_setup(pruss, pruss->core_clk_mux, "coreclk-mux",
				  ICSSG_CFG_CORE_SYNC);
	if (ret) {
		dev_err(dev, "failed to setup coreclk-mux\n");
		goto rpm_put;
	}

	ret = pruss_clk_mux_setup(pruss, pruss->core_clk_mux, "iepclk-mux",
				  PRUSS_CFG_IEPCLK);
	if (ret) {
		dev_err(dev, "failed to setup iepclk-mux\n");
		goto rpm_put;
	}

skip_mux:
	syscon_config.name = kasprintf(GFP_KERNEL, "%pOFn@%llx", np,
				       (u64)res.start);
	syscon_config.max_register = resource_size(&res) - 4;

	pruss->cfg = regmap_init_mmio(NULL, pruss->cfg_base, &syscon_config);
	kfree(syscon_config.name);
	if (IS_ERR(pruss->cfg)) {
		dev_err(dev, "regmap_init_mmio failed for cfg, ret = %ld\n",
			PTR_ERR(pruss->cfg));
		ret = PTR_ERR(pruss->cfg);
		goto rpm_put;
	}

	dev_dbg(&pdev->dev, "creating PRU cores and other child platform devices\n");
	ret = of_platform_populate(node, NULL, NULL, &pdev->dev);
	if (ret) {
		dev_err(dev, "of_platform_populate failed\n");
		goto reg_exit;
	}

	return 0;

reg_exit:
	regmap_exit(pruss->cfg);
rpm_put:
	pm_runtime_put_sync(dev);
rpm_disable:
	pm_runtime_disable(dev);
	return ret;
}

static int pruss_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pruss *pruss = platform_get_drvdata(pdev);

	regmap_exit(pruss->cfg);

	dev_dbg(dev, "remove PRU cores and other child platform devices\n");
	of_platform_depopulate(dev);

	pm_runtime_put_sync(dev);
	pm_runtime_disable(dev);

	return 0;
}

/* instance-specific driver private data */
static const struct pruss_private_data am437x_pruss1_priv_data = {
	.has_no_sharedram = false,
};

static const struct pruss_private_data am437x_pruss0_priv_data = {
	.has_no_sharedram = true,
};

static const struct pruss_match_private_data am437x_match_data[] = {
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

static const struct of_device_id pruss_of_match[] = {
	{ .compatible = "ti,am3356-pruss", .data = NULL, },
	{ .compatible = "ti,am4376-pruss", .data = &am437x_match_data, },
	{ .compatible = "ti,am5728-pruss", .data = NULL, },
	{ .compatible = "ti,k2g-pruss", .data = NULL, },
	{ .compatible = "ti,am654-icssg", .data = NULL, },
	{ .compatible = "ti,j721e-icssg", .data = NULL, },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, pruss_of_match);

static struct platform_driver pruss_driver = {
	.driver = {
		.name = "pruss",
		.of_match_table = pruss_of_match,
	},
	.probe  = pruss_probe,
	.remove = pruss_remove,
};
module_platform_driver(pruss_driver);

MODULE_AUTHOR("Suman Anna <s-anna@ti.com>");
MODULE_DESCRIPTION("PRU-ICSS Subsystem Driver");
MODULE_LICENSE("GPL v2");
