/*
 * PRU-ICSS platform driver for various TI SoCs
 *
 * Copyright (C) 2014-2016 Texas Instruments Incorporated - http://www.ti.com/
 *	Suman Anna <s-anna@ti.com>
 *	Andrew F. Davis <afd@ti.com>
 *	Roger Quadros <rogerq@ti.com>
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

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/remoteproc.h>
#include <linux/virtio.h>
#include <linux/pruss.h>

#include <linux/platform_data/remoteproc-pruss.h>

#include "remoteproc_internal.h"
#include "pruss.h"

/**
 * struct pruss_private_data - PRUSS driver private data
 * @aux_data: auxiliary data used for creating the child nodes
 * @has_reset: flag to indicate the presence of global module reset
 * @has_no_syscfg: flag to indicate the absence of PRUSS_SYSCFG functionality
 * @has_no_sharedram: flag to indicate the absence of PRUSS Shared Data RAM
 * @uses_wrapper: flag to indicate the dependence on PRUSS syscfg wrapper module
 */
struct pruss_private_data {
	struct of_dev_auxdata *aux_data;
	bool has_reset;
	bool has_no_syscfg;
	bool has_no_sharedram;
	bool uses_wrapper;
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

static DEFINE_MUTEX(pruss_list_mutex);
static LIST_HEAD(pruss_list);

/**
 * pruss_get() - get the pruss for the given device
 * @dev: device interested in the pruss
 *
 * Finds the pruss device referenced by the "pruss" property in the
 * requesting (client) device's device node.
 *
 * This function increments the pruss device's refcount, so always
 * use pruss_put() to decrement it back once pruss isn't needed anymore.
 *
 * Returns the pruss handle on success, and NULL on failure.
 */
struct pruss *pruss_get(struct device *dev)
{
	struct pruss *pruss = NULL, *p;
	struct device_node *np;

	if (!dev)
		return NULL;

	np = of_parse_phandle(dev->of_node, "pruss", 0);
	if (!np)
		return NULL;

	mutex_lock(&pruss_list_mutex);
	list_for_each_entry(p, &pruss_list, node) {
		if (p->dev->of_node == np) {
			pruss = p;
			get_device(pruss->dev);
			break;
		}
	}

	mutex_unlock(&pruss_list_mutex);
	of_node_put(np);

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
	if (!pruss)
		return;

	put_device(pruss->dev);
}
EXPORT_SYMBOL_GPL(pruss_put);

/*
 * get the rproc phandle corresponding to a pru_id.
 * Caller must call rproc_put() when done with rproc.
 */
static struct rproc *__pruss_rproc_get(struct pruss *pruss,
				       enum pruss_pru_id pru_id)
{
	struct device_node *rproc_np;
	struct platform_device *pdev;
	struct rproc *rproc;

	/* get rproc corresponding to pru_id */
	switch (pru_id) {
	case PRUSS_PRU0:
		rproc_np = of_get_child_by_name(pruss->dev->of_node, "pru0");
		break;
	case PRUSS_PRU1:
		rproc_np = of_get_child_by_name(pruss->dev->of_node, "pru1");
		break;
	default:
		return ERR_PTR(-EINVAL);
	}

	if (!rproc_np)
		return ERR_PTR(-ENODEV);

	pdev = of_find_device_by_node(rproc_np);
	of_node_put(rproc_np);

	if (!pdev)
		/* probably PRU not yet probed */
		return ERR_PTR(-EPROBE_DEFER);

	rproc = platform_get_drvdata(pdev);
	if (!rproc)
		return ERR_PTR(-EPROBE_DEFER);

	get_device(&rproc->dev);

	return rproc;
}

/**
 * pruss_rproc_get() - Get the rproc instance corresponding to pru_id
 * @pruss: the pruss instance
 * @pru_id: the PRU id of which we need the rproc instance
 *
 * Allows only one user to own the rproc resource at a time.
 * Caller must call pruss_put_rproc() when done with using the rproc.
 *
 * Returns rproc handle on success. ERR_PTR on failure e.g.
 * -EBUSY if PRU is already reserved by someone else
 * -ENODEV if not yet available.
 * -EINVAL if invalid parameters.
 */
struct rproc *pruss_rproc_get(struct pruss *pruss,
			      enum pruss_pru_id pru_id)
{
	struct rproc *rproc;
	int ret;

	if (!pruss)
		return ERR_PTR(-EINVAL);

	rproc = __pruss_rproc_get(pruss, pru_id);
	if (IS_ERR(rproc))
		return rproc;

	mutex_lock(&pruss->lock);

	if (pruss->pru_in_use[pru_id]) {
		ret = -EBUSY;
		goto unlock;
	}

	pruss->pru_in_use[pru_id] = rproc;

	mutex_unlock(&pruss->lock);

	return rproc;

unlock:
	mutex_unlock(&pruss->lock);
	rproc_put(rproc);

	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(pruss_rproc_get);

/* find out PRU ID from the rproc instance */
static enum pruss_pru_id pruss_rproc_to_pru_id(struct pruss *pruss,
					       struct rproc *rproc)
{
	enum pruss_pru_id pru_id;

	for (pru_id = PRUSS_PRU0; pru_id < PRUSS_NUM_PRUS; pru_id++)
		if (pruss->pru_in_use[pru_id] == rproc)
			return pru_id;

	return -1;
}

/**
 * pruss_rproc_put() - release the PRU rproc resource
 * @pruss: the pruss instance
 * @rproc: the rproc resource to release
 *
 * Releases the rproc resource and makes it available to other
 * users.
 */
void pruss_rproc_put(struct pruss *pruss, struct rproc *rproc)
{
	int pru_id;

	if (!pruss || !rproc)
		return;

	mutex_lock(&pruss->lock);

	pru_id = pruss_rproc_to_pru_id(pruss, rproc);
	if (pru_id < 0) {
		mutex_unlock(&pruss->lock);
		return;
	}

	if (!pruss->pru_in_use[pru_id]) {
		mutex_unlock(&pruss->lock);
		return;
	}
	pruss->pru_in_use[pru_id] = NULL;

	mutex_unlock(&pruss->lock);

	rproc_put(rproc);
}
EXPORT_SYMBOL(pruss_rproc_put);

/**
 * pruss_request_mem_region() - request a memory resource
 * @pruss: the pruss instance
 * @mem_id: the memory resource id
 * @region: pointer to memory region structure to be filled in
 *
 * This function allows a client driver to requests a memory resource,
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

static inline u32 pruss_read_reg(struct pruss *pruss, enum pruss_mem region,
				 unsigned int reg)
{
	return readl_relaxed(pruss->mem_regions[region].va + reg);
}

static inline void pruss_write_reg(struct pruss *pruss, enum pruss_mem region,
				   unsigned int reg, u32 val)
{
	writel_relaxed(val, pruss->mem_regions[region].va + reg);
}

static inline void pruss_set_reg(struct pruss *pruss, enum pruss_mem region,
				 unsigned int reg, u32 mask, u32 set)
{
	u32 val;

	val = pruss_read_reg(pruss, region, reg);
	val &= ~mask;
	val |= (set & mask);
	pruss_write_reg(pruss, region, reg, val);
}

/* firmware must be idle when calling this function */
static void pruss_disable_module(struct pruss *pruss)
{
	if (pruss->data->has_no_syscfg)
		goto put_sync;

	/* configure Smart Standby */
	pruss_set_reg(pruss, PRUSS_MEM_CFG, PRUSS_CFG_SYSCFG,
		      PRUSS_SYSCFG_STANDBY_MODE_MASK,
		      PRUSS_SYSCFG_STANDBY_MODE_SMART);

	/* initiate MStandby */
	if (!pruss->data->uses_wrapper) {
		pruss_set_reg(pruss, PRUSS_MEM_CFG, PRUSS_CFG_SYSCFG,
			      PRUSS_SYSCFG_STANDBY_INIT,
			      PRUSS_SYSCFG_STANDBY_INIT);
	}

put_sync:
	/* tell PRCM to initiate IDLE request */
	pm_runtime_put_sync(pruss->dev);
}

/*
 * This function programs the PRUSS_SYSCFG.STANDBY_INIT bit to achieve dual
 * functionalities - one is to deassert the MStandby signal to the device
 * PRCM, and the other is to enable OCP master ports to allow accesses
 * outside of the PRU-ICSS. The function has to wait for the PRCM to
 * acknowledge through the monitoring of the PRUSS_SYSCFG.SUB_MWAIT bit.
 */
static int pruss_enable_ocp_master_ports(struct pruss *pruss)
{
	int i;
	bool ready;

	/* SYSCFG programming STANDBY_INIT programming handled by wrapper */
	if (pruss->data->uses_wrapper)
		return 0;

	pruss_set_reg(pruss, PRUSS_MEM_CFG, PRUSS_CFG_SYSCFG,
		      PRUSS_SYSCFG_STANDBY_INIT, 0);

	/* wait till we are ready for transactions - delay is arbitrary */
	for (i = 0; i < 10; i++) {
		ready = !(pruss_read_reg(pruss, PRUSS_MEM_CFG, PRUSS_CFG_SYSCFG)
			  & PRUSS_SYSCFG_SUB_MWAIT_READY);
		if (ready)
			break;
		udelay(5);
	}

	if (!ready) {
		dev_err(pruss->dev, "timeout waiting for SUB_MWAIT_READY\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int pruss_enable_module(struct pruss *pruss)
{
	int ret;

	/* tell PRCM to de-assert IDLE request */
	ret = pm_runtime_get_sync(pruss->dev);
	if (ret < 0) {
		pm_runtime_put_noidle(pruss->dev);
		return ret;
	}

	if (pruss->data->has_no_syscfg)
		return ret;

	/* configure for smart idle & smart standby */
	pruss_set_reg(pruss, PRUSS_MEM_CFG, PRUSS_CFG_SYSCFG,
		      PRUSS_SYSCFG_IDLE_MODE_MASK,
		      PRUSS_SYSCFG_IDLE_MODE_SMART);
	pruss_set_reg(pruss, PRUSS_MEM_CFG, PRUSS_CFG_SYSCFG,
		      PRUSS_SYSCFG_STANDBY_MODE_MASK,
		      PRUSS_SYSCFG_STANDBY_MODE_SMART);

	/* enable OCP master ports/disable MStandby */
	ret = pruss_enable_ocp_master_ports(pruss);
	if (ret)
		pruss_disable_module(pruss);

	return ret;
}

/**
 * pruss_cfg_gpimode() - set the GPI mode of the PRU
 * @pruss: the pruss instance handle
 * @rproc: the rproc instance handle of the PRU
 * @mode: GPI mode to set
 *
 * Sets the GPI mode for a given PRU by programming the
 * corresponding PRUSS_CFG_GPCFGx register
 *
 * Returns 0 on success, or an error code otherwise
 */
int pruss_cfg_gpimode(struct pruss *pruss, struct rproc *rproc,
		      enum pruss_gpi_mode mode)
{
	u32 reg;
	int pru_id;

	pru_id = pruss_rproc_to_pru_id(pruss, rproc);
	if (pru_id < 0 || pru_id >= PRUSS_NUM_PRUS) {
		dev_err(pruss->dev, "%s: PRU id not found, %d\n",
			__func__, pru_id);
		return -EINVAL;
	}

	reg = PRUSS_CFG_GPCFG0 + (0x4 * pru_id);

	mutex_lock(&pruss->cfg_lock);
	pruss_set_reg(pruss, PRUSS_MEM_CFG, reg,
		      PRUSS_GPCFG_PRU_GPI_MODE_MASK,
		      mode << PRUSS_GPCFG_PRU_GPI_MODE_SHIFT);
	mutex_unlock(&pruss->cfg_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(pruss_cfg_gpimode);

/**
 * pruss_cfg_set_gpmux() - set the GP_MUX mode of the PRU
 * @pruss: the pruss instance handle
 * @pru_id: the PRU id for which the GP_MUX mode is to be set
 * @mux_sel: GP mux sel value to set
 *
 * Sets the GP MUX mode for a given PRU by programming the
 * corresponding PRUSS_CFG_GPCFGx register. This API is currently
 * limited to be invoked only by the pru_rproc driver
 *
 * Returns 0 on success, or an error code otherwise
 */
int pruss_cfg_set_gpmux(struct pruss *pruss, enum pruss_pru_id pru_id,
			enum pruss_gp_mux_sel mux_sel)
{
	u32 reg, val, mask;

	if (pru_id < 0 || pru_id >= PRUSS_NUM_PRUS) {
		dev_err(pruss->dev, "%s: invalid PRU id, %d\n",
			__func__, pru_id);
		return -EINVAL;
	}

	reg = PRUSS_CFG_GPCFG0 + (0x4 * pru_id);
	val = mux_sel << PRUSS_GPCFG_PRU_MUX_SEL_SHIFT;
	mask = PRUSS_GPCFG_PRU_MUX_SEL_MASK;

	pruss_set_reg(pruss, PRUSS_MEM_CFG, reg, mask, val);

	return 0;
}
EXPORT_SYMBOL_GPL(pruss_cfg_set_gpmux);

/**
 * pruss_cfg_miirt_enable() - Enable/disable MII RT Events
 * @pruss: the pruss instance
 * @enable: enable/disable
 *
 * Enable/disable the MII RT Events for the PRUSS.
 */
void pruss_cfg_miirt_enable(struct pruss *pruss, bool enable)
{
	u32 set = enable ? PRUSS_MII_RT_EVENT_EN : 0;

	mutex_lock(&pruss->cfg_lock);
	pruss_set_reg(pruss, PRUSS_MEM_CFG, PRUSS_CFG_MII_RT,
		      PRUSS_MII_RT_EVENT_EN, set);
	mutex_unlock(&pruss->cfg_lock);
}
EXPORT_SYMBOL_GPL(pruss_cfg_miirt_enable);

/**
 * pruss_cfg_xfr_enable() - Enable/disable XIN XOUT shift functionality
 * @pruss: the pruss instance
 * @enable: enable/disable
 */
void pruss_cfg_xfr_enable(struct pruss *pruss, bool enable)
{
	u32 set = enable ? PRUSS_SPP_XFER_SHIFT_EN : 0;

	mutex_lock(&pruss->cfg_lock);
	pruss_set_reg(pruss, PRUSS_MEM_CFG, PRUSS_CFG_SPP,
		      PRUSS_SPP_XFER_SHIFT_EN, set);
	mutex_unlock(&pruss->cfg_lock);
}
EXPORT_SYMBOL_GPL(pruss_cfg_xfr_enable);

#ifdef CONFIG_PM_SLEEP
static int pruss_suspend(struct device *dev)
{
	struct pruss *pruss = dev_get_drvdata(dev);
	u32 syscfg_val;

	if (pruss->data->has_no_syscfg)
		return 0;

	syscfg_val = pruss_read_reg(pruss, PRUSS_MEM_CFG, PRUSS_CFG_SYSCFG);
	pruss->in_standby = syscfg_val & PRUSS_SYSCFG_STANDBY_INIT;

	/* initiate MStandby, undo the MStandby config in probe */
	if (!pruss->in_standby && !pruss->data->uses_wrapper) {
		pruss_set_reg(pruss, PRUSS_MEM_CFG, PRUSS_CFG_SYSCFG,
			      PRUSS_SYSCFG_STANDBY_INIT,
			      PRUSS_SYSCFG_STANDBY_INIT);
	}

	return 0;
}

static int pruss_resume(struct device *dev)
{
	struct pruss *pruss = dev_get_drvdata(dev);
	int ret = 0;

	/* re-enable OCP master ports/disable MStandby */
	if (!pruss->data->has_no_syscfg && !pruss->in_standby) {
		ret = pruss_enable_ocp_master_ports(pruss);
		if (ret)
			dev_err(dev, "%s failed\n", __func__);
	}

	return ret;
}
#endif /* CONFIG_PM_SLEEP */

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
	struct pruss_platform_data *pdata = dev_get_platdata(dev);
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

	if (data->has_reset && (!pdata || !pdata->deassert_reset ||
				!pdata->assert_reset || !pdata->reset_name)) {
		dev_err(dev, "platform data (reset configuration information) missing\n");
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
	pruss->data = data;
	mutex_init(&pruss->lock);
	mutex_init(&pruss->cfg_lock);

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

	if (data->has_reset) {
		ret = pdata->deassert_reset(pdev, pdata->reset_name);
		if (ret) {
			dev_err(dev, "deassert_reset failed: %d\n", ret);
			goto err_fail;
		}
	}

	pm_runtime_enable(dev);
	ret = pruss_enable_module(pruss);
	if (ret < 0) {
		dev_err(dev, "couldn't enable pruss\n");
		goto err_rpm_fail;
	}

	platform_set_drvdata(pdev, pruss);

	mutex_lock(&pruss_list_mutex);
	list_add_tail(&pruss->node, &pruss_list);
	mutex_unlock(&pruss_list_mutex);

	dev_info(&pdev->dev, "creating PRU cores and other child platform devices\n");
	ret = of_platform_populate(node, NULL, data->aux_data, &pdev->dev);
	if (ret) {
		dev_err(dev, "of_platform_populate failed\n");
		goto err_of_fail;
	}

	return 0;

err_of_fail:
	mutex_lock(&pruss_list_mutex);
	list_del(&pruss->node);
	mutex_unlock(&pruss_list_mutex);

	pruss_disable_module(pruss);
err_rpm_fail:
	pm_runtime_disable(dev);
	if (data->has_reset)
		pdata->assert_reset(pdev, pdata->reset_name);
err_fail:
	return ret;
}

static int pruss_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pruss_platform_data *pdata = dev_get_platdata(dev);
	struct pruss *pruss = platform_get_drvdata(pdev);

	dev_info(dev, "remove PRU cores and other child platform devices\n");
	of_platform_depopulate(dev);

	mutex_lock(&pruss_list_mutex);
	list_del(&pruss->node);
	mutex_unlock(&pruss_list_mutex);

	pruss_disable_module(pruss);
	pm_runtime_disable(dev);
	if (pruss->data->has_reset)
		pdata->assert_reset(pdev, pdata->reset_name);

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
	OF_DEV_AUXDATA("ti,am3352-pru", 0x4a334000, "4a334000.pru0", NULL),
	OF_DEV_AUXDATA("ti,am3352-pru", 0x4a338000, "4a338000.pru1", NULL),
	{ /* sentinel */ },
};

static struct of_dev_auxdata am437x_pruss1_rproc_auxdata_lookup[] = {
	OF_DEV_AUXDATA("ti,am4372-pru", 0x54434000, "54434000.pru0", NULL),
	OF_DEV_AUXDATA("ti,am4372-pru", 0x54438000, "54438000.pru1", NULL),
	{ /* sentinel */ },
};

static struct of_dev_auxdata am437x_pruss0_rproc_auxdata_lookup[] = {
	OF_DEV_AUXDATA("ti,am4372-pru", 0x54474000, "54474000.pru0", NULL),
	OF_DEV_AUXDATA("ti,am4372-pru", 0x54478000, "54478000.pru1", NULL),
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
	.has_reset = true,
};

static struct pruss_private_data am437x_pruss1_priv_data = {
	.aux_data = am437x_pruss1_rproc_auxdata_lookup,
	.has_reset = false, /* handled by parent wrapper */
	.uses_wrapper = true,
};

static struct pruss_private_data am437x_pruss0_priv_data = {
	.aux_data = am437x_pruss0_rproc_auxdata_lookup,
	.has_reset = false, /* handled by parent wrapper */
	.has_no_syscfg = true,
	.has_no_sharedram = true,
	.uses_wrapper = true,
};

static struct pruss_private_data am57xx_pruss1_priv_data = {
	.aux_data = am57xx_pruss1_rproc_auxdata_lookup,
};

static struct pruss_private_data am57xx_pruss2_priv_data = {
	.aux_data = am57xx_pruss2_rproc_auxdata_lookup,
};

static struct pruss_private_data k2g_pruss0_priv_data = {
	.aux_data = k2g_pruss0_rproc_auxdata_lookup,
	.has_no_syscfg = true,
};

static struct pruss_private_data k2g_pruss1_priv_data = {
	.aux_data = k2g_pruss1_rproc_auxdata_lookup,
	.has_no_syscfg = true,
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
	{ .compatible = "ti,am3352-pruss", .data = &am335x_match_data, },
	{ .compatible = "ti,am4372-pruss", .data = &am437x_match_data, },
	{ .compatible = "ti,am5728-pruss", .data = &am57xx_match_data, },
	{ .compatible = "ti,k2g-pruss", .data = &k2g_match_data, },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, pruss_of_match);

static SIMPLE_DEV_PM_OPS(pruss_pm_ops, pruss_suspend, pruss_resume);

static struct platform_driver pruss_driver = {
	.driver = {
		.name = "ti-pruss",
		.pm = &pruss_pm_ops,
		.of_match_table = pruss_of_match,
	},
	.probe  = pruss_probe,
	.remove = pruss_remove,
};
module_platform_driver(pruss_driver);

MODULE_AUTHOR("Suman Anna <s-anna@ti.com>");
MODULE_DESCRIPTION("PRU-ICSS Subsystem Driver");
MODULE_LICENSE("GPL v2");
