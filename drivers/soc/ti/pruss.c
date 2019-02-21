// SPDX-License-Identifier: GPL-2.0
/*
 * PRU-ICSS platform driver for various TI SoCs
 *
 * Copyright (C) 2014-2019 Texas Instruments Incorporated - http://www.ti.com/
 *	Suman Anna <s-anna@ti.com>
 *	Andrew F. Davis <afd@ti.com>
 *	Tero Kristo <t-kristo@ti.com>
 */

#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/pruss_driver.h>
#include <linux/regmap.h>
#include <linux/remoteproc.h>

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

/**
 * pruss_get() - get the pruss for a given PRU remoteproc
 * @rproc: remoteproc handle of a PRU instance
 *
 * Finds the parent pruss device for a PRU given the @rproc handle of the
 * PRU remote processor. This function increments the pruss device's refcount,
 * so always use pruss_put() to decrement it back once pruss isn't needed
 * anymore.
 *
 * Returns the pruss handle on success, and an ERR_PTR on failure using one
 * of the following error values
 *    -EINVAL if invalid parameter
 *    -ENODEV if PRU device or PRUSS device is not found
 */
struct pruss *pruss_get(struct rproc *rproc)
{
	struct pruss *pruss;
	struct device *dev;
	struct platform_device *ppdev;

	if (IS_ERR_OR_NULL(rproc))
		return ERR_PTR(-EINVAL);

	dev = &rproc->dev;
	if (!dev->parent)
		return ERR_PTR(-ENODEV);

	/* rudimentary check to make sure rproc handle is for a PRU or RTU */
	if (!strstr(dev_name(dev->parent), "pru") &&
	    !strstr(dev_name(dev->parent), "rtu"))
		return ERR_PTR(-ENODEV);

	ppdev = to_platform_device(dev->parent->parent);
	pruss = platform_get_drvdata(ppdev);
	if (!pruss)
		return ERR_PTR(-ENODEV);

	get_device(pruss->dev);

	return pruss;
}
EXPORT_SYMBOL_GPL(pruss_get);

/**
 * pruss_put() - decrement pruss device's usecount
 * @pruss: pruss handle
 *
 * Complimentary function for pruss_get(). Needs to be called
 * after the PRUSS is used, and only if the pruss_get() succeeds.
 */
void pruss_put(struct pruss *pruss)
{
	if (IS_ERR_OR_NULL(pruss))
		return;

	put_device(pruss->dev);
}
EXPORT_SYMBOL_GPL(pruss_put);

/**
 * pruss_request_mem_region() - request a memory resource
 * @pruss: the pruss instance
 * @mem_id: the memory resource id
 * @region: pointer to memory region structure to be filled in
 *
 * This function allows a client driver to request a memory resource,
 * and if successful, will let the client driver own the particular
 * memory region until released using the pruss_release_mem_region()
 * API.
 *
 * Returns the memory region if requested resource is available, an
 * error otherwise
 */
int pruss_request_mem_region(struct pruss *pruss, enum pruss_mem mem_id,
			     struct pruss_mem_region *region)
{
	if (!pruss || !region)
		return -EINVAL;

	if (mem_id >= PRUSS_MEM_MAX)
		return -EINVAL;

	mutex_lock(&pruss->lock);

	if (pruss->mem_in_use[mem_id]) {
		mutex_unlock(&pruss->lock);
		return -EBUSY;
	}

	*region = pruss->mem_regions[mem_id];
	pruss->mem_in_use[mem_id] = region;

	mutex_unlock(&pruss->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(pruss_request_mem_region);

/**
 * pruss_release_mem_region() - release a memory resource
 * @pruss: the pruss instance
 * @region: the memory region to release
 *
 * This function is the complimentary function to
 * pruss_request_mem_region(), and allows the client drivers to
 * release back a memory resource.
 *
 * Returns 0 on success, an error code otherwise
 */
int pruss_release_mem_region(struct pruss *pruss,
			     struct pruss_mem_region *region)
{
	int id;

	if (!pruss || !region)
		return -EINVAL;

	mutex_lock(&pruss->lock);

	/* find out the memory region being released */
	for (id = 0; id < PRUSS_MEM_MAX; id++) {
		if (pruss->mem_in_use[id] == region)
			break;
	}

	if (id == PRUSS_MEM_MAX) {
		mutex_unlock(&pruss->lock);
		return -EINVAL;
	}

	pruss->mem_in_use[id] = NULL;

	mutex_unlock(&pruss->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(pruss_release_mem_region);

/**
 * pruss_regmap_read() - read a PRUSS syscon sub-module register
 * @pruss: the pruss instance handle
 * @mod: the pruss syscon sub-module identifier
 * @reg: register offset within the sub-module
 * @val: pointer to return the value in
 *
 * Reads a given register within one of the PRUSS sub-modules represented
 * by a syscon and returns it through the passed-in @val pointer
 *
 * Returns 0 on success, or an error code otherwise
 */
int pruss_regmap_read(struct pruss *pruss, enum pruss_syscon mod,
		      unsigned int reg, unsigned int *val)
{
	struct regmap *map;

	if (IS_ERR_OR_NULL(pruss) || mod < PRUSS_SYSCON_CFG ||
	    mod >= PRUSS_SYSCON_MAX)
		return -EINVAL;

	map = (mod == PRUSS_SYSCON_CFG) ? pruss->cfg :
	       ((mod == PRUSS_SYSCON_IEP ? pruss->iep : pruss->mii_rt));

	return regmap_read(map, reg, val);
}
EXPORT_SYMBOL_GPL(pruss_regmap_read);

/**
 * pruss_regmap_update() - configure a PRUSS syscon sub-module register
 * @pruss: the pruss instance handle
 * @mod: the pruss syscon sub-module identifier
 * @reg: register offset within the sub-module
 * @mask: bit mask to use for programming the @val
 * @val: value to write
 *
 * Programs a given register within one of the PRUSS sub-modules represented
 * by a syscon
 *
 * Returns 0 on success, or an error code otherwise
 */
int pruss_regmap_update(struct pruss *pruss, enum pruss_syscon mod,
			unsigned int reg, unsigned int mask, unsigned int val)
{
	struct regmap *map;

	if (IS_ERR_OR_NULL(pruss) || mod < PRUSS_SYSCON_CFG ||
	    mod >= PRUSS_SYSCON_MAX)
		return -EINVAL;

	map = (mod == PRUSS_SYSCON_CFG) ? pruss->cfg :
	       ((mod == PRUSS_SYSCON_IEP ? pruss->iep : pruss->mii_rt));

	return regmap_update_bits(map, reg, mask, val);
}
EXPORT_SYMBOL_GPL(pruss_regmap_update);

/* Custom configuration of couple of PRUSS clocks only on AM65x SoCs */
static int pruss_configure_clocks(struct platform_device *pdev,
				  struct pruss *pruss)
{
	int ret;

	if (!of_device_is_compatible(pdev->dev.of_node, "ti,am654-icssg"))
		return 0;

	ret = pruss_regmap_update(pruss, PRUSS_SYSCON_CFG, ICSSG_CFG_CORE_SYNC,
				  ICSSG_CORE_VBUSP_SYNC_EN,
				  ICSSG_CORE_VBUSP_SYNC_EN);
	if (ret)
		return ret;

	ret = pruss_regmap_update(pruss, PRUSS_SYSCON_CFG, PRUSS_CFG_IEPCLK,
				  PRUSS_IEPCLK_IEP_OCP_CLK_EN,
				  PRUSS_IEPCLK_IEP_OCP_CLK_EN);
	if (ret)
		return ret;

	return 0;
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
	mutex_init(&pruss->lock);

	np = of_get_child_by_name(node, "cfg");
	if (!np) {
		dev_err(dev, "%pOF is missing cfg node\n", np);
		return -ENODEV;
	}

	pruss->cfg = syscon_node_to_regmap(np);
	of_node_put(np);
	if (IS_ERR(pruss->cfg))
		return -ENODEV;

	np = of_get_child_by_name(node, "iep");
	if (!np) {
		dev_err(dev, "%pOF is missing iep node\n", np);
		return -ENODEV;
	}

	pruss->iep = syscon_node_to_regmap(np);
	of_node_put(np);
	if (IS_ERR(pruss->iep))
		return -ENODEV;

	np = of_get_child_by_name(node, "mii-rt");
	if (!np) {
		dev_err(dev, "%pOF is missing mii-rt node\n", np);
		return -ENODEV;
	}

	pruss->mii_rt = syscon_node_to_regmap(np);
	of_node_put(np);
	if (IS_ERR(pruss->mii_rt))
		return -ENODEV;

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

	ret = pruss_configure_clocks(pdev, pruss);
	if (ret) {
		dev_err(dev, "clock frequency config failed, ret = %d\n", ret);
		return ret;
	}

	dev_dbg(&pdev->dev, "creating PRU cores and other child platform devices\n");
	ret = of_platform_populate(node, NULL, NULL, &pdev->dev);
	if (ret)
		dev_err(dev, "of_platform_populate failed\n");

	return ret;
}

static int pruss_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	dev_dbg(dev, "remove PRU cores and other child platform devices\n");
	of_platform_depopulate(dev);

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
