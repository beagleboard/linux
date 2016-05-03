/*
 * Copyright (C) 2016 Texas Instruments Ltd
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/dma-mapping.h>
#include <linux/platform_data/omap_drm.h>

#include "omapdss.h"
#include "dss6.h"

#define DSS_REVISION	0x00
#define DSS_SYSCONFIG	0x10
#define DSS_SYSSTATUS	0x14
#define DSS_RFBI_CTRL	0x18
#define DSS_DPI_CTRL	0x1c
#define DSS_DEBUG_CFG	0x40

#define REG_GET(data, idx, start, end) \
	FLD_GET(dss6_read(data, idx), start, end)

#define REG_FLD_MOD(data, idx, val, start, end) \
	dss6_write(data, idx, FLD_MOD(dss6_read(data, idx), val, start, end))

struct dss_features {
	int num_ports;
};

static const struct dss_features k2g_dss_feats = {
	.num_ports = 1,
};

static const struct of_device_id dss6_of_match[];

struct dss_data {
	struct platform_device *pdev;
	void __iomem *base;

	struct clk *fclk;

	const struct dss_features *feat;
};

static struct dss_data *dss_get_data(struct device *dev)
{
	return dev_get_drvdata(dev);
}

/* omapdrm device */

/*
 * HACK. For OMAP, we create the omapdrm device in platform code. That will
 * be removed when omapdss and omapdrm are merged. To avoid creating such
 * platform code for K2G, we create omapdrm device after omapdss's probe
 * has succeeded.
 */

static struct omap_drm_platform_data platform_data;

static void omapdrm_release(struct device *dev)
{
}

static struct platform_device omap_drm_device = {
	.dev = {
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.platform_data = &platform_data,
		.release = omapdrm_release,
	},
	.name = "omapdrm",
	.id = 0,
};

static int initialize_omapdrm_device(void)
{
	platform_data.omaprev = 0;
	return platform_device_register(&omap_drm_device);
}

static void uninitialize_omapdrm_device(void)
{
	platform_device_unregister(&omap_drm_device);
}

/* omapdrm device end */

static inline void dss6_write(struct dss_data *dss_data, u16 reg, u32 val)
{
	iowrite32(val, dss_data->base + reg);
}

static inline u32 dss6_read(struct dss_data *dss_data, u16 reg)
{
	return ioread32(dss_data->base + reg);
}

void dss6_ungate_dpi_clk(struct device *dev, int port)
{
	struct dss_data *dss_data = dss_get_data(dev);

	REG_FLD_MOD(dss_data, DSS_DPI_CTRL, 1, 1, 0);
}

void dss6_gate_dpi_clk(struct device *dev, int port)
{
	struct dss_data *dss_data = dss_get_data(dev);

	REG_FLD_MOD(dss_data, DSS_DPI_CTRL, 0, 1, 0);
}

static int dss6_init_features(struct platform_device *pdev)
{
	struct dss_data *dss_data = dss_get_data(&pdev->dev);
	const struct of_device_id *match;

	match = of_match_node(dss6_of_match, pdev->dev.of_node);
	if (!match) {
		dev_err(&pdev->dev, "Unsupported DSS version\n");
		return -ENODEV;
	}

	dss_data->feat = match->data;

	return 0;
}

static int dss6_init_ports(struct platform_device *pdev)
{
	struct dss_data *dss_data = dss_get_data(&pdev->dev);
	struct device_node *parent = pdev->dev.of_node;
	struct device_node *port;

	port = omapdss_of_get_next_port(parent, NULL);
	if (!port)
		return 0;

	if (dss_data->feat->num_ports == 0)
		return 0;

	do {
		int r;
		u32 reg;

		r = of_property_read_u32(port, "reg", &reg);
		if (r)
			reg = 0;

		if (reg >= dss_data->feat->num_ports)
			continue;

		dpi6_init_port(pdev, port);
	} while ((port = omapdss_of_get_next_port(parent, port)) != NULL);

	return 0;
}

static void dss6_uninit_ports(struct platform_device *pdev)
{
	struct dss_data *dss_data = dss_get_data(&pdev->dev);
	struct device_node *parent = pdev->dev.of_node;
	struct device_node *port;

	port = omapdss_of_get_next_port(parent, NULL);
	if (!port)
		return;

	if (dss_data->feat->num_ports == 0)
		return;

	do {
		u32 reg;
		int r;

		r = of_property_read_u32(port, "reg", &reg);
		if (r)
			reg = 0;

		if (reg >= dss_data->feat->num_ports)
			continue;

		dpi6_uninit_port(port);
	} while ((port = omapdss_of_get_next_port(parent, port)) != NULL);
}

static int dss6_bind(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct dss_data *dss_data;
	struct resource *dss_mem;
	int r;

	dss_data = devm_kzalloc(dev, sizeof(*dss_data), GFP_KERNEL);
	if (!dss_data)
		return -ENOMEM;

	dev_set_drvdata(dev, dss_data);
	dss_data->pdev = pdev;

	r = dss6_init_features(dss_data->pdev);
	if (r)
		return r;

	dss_mem = platform_get_resource(dss_data->pdev, IORESOURCE_MEM, 0);
	if (!dss_mem) {
		dev_err(dev, "Failed to get IORESOURCE_MEM\n");
		return -EINVAL;
	}

	dss_data->base = devm_ioremap(&pdev->dev, dss_mem->start,
				      resource_size(dss_mem));
	if (!dss_data->base) {
		dev_err(dev, "Failed to ioremap\n");
		return -ENOMEM;
	}

	r = dss6_init_ports(pdev);
	if (r)
		goto err_init_ports;

	dss_data->fclk = devm_clk_get(dev, "fck");
	if (IS_ERR(dss_data->fclk)) {
		dev_err(dev, "Failed to get fclk\n");
		return PTR_ERR(dss_data->fclk);
	}

	dev_dbg(&pdev->dev, "DSS fclk %lu Hz\n", clk_get_rate(dss_data->fclk));

	pm_runtime_enable(&pdev->dev);

	r = component_bind_all(&pdev->dev, NULL);
	if (r)
		goto err_component;

	omapdss_gather_components(dev);
	omapdss_set_is_initialized(true);

	return 0;

err_component:
	pm_runtime_disable(&pdev->dev);
err_init_ports:
	return r;
}

static void dss6_unbind(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);

	omapdss_set_is_initialized(false);

	component_unbind_all(&pdev->dev, NULL);

	dss6_uninit_ports(pdev);

	pm_runtime_disable(&pdev->dev);
}

static const struct component_master_ops dss6_component_ops = {
	.bind = dss6_bind,
	.unbind = dss6_unbind,
};

static int dss6_component_compare(struct device *dev, void *data)
{
	struct device *child = data;
	return dev == child;
}

static int dss6_add_child_component(struct device *dev, void *data)
{
	struct component_match **match = data;

	component_match_add(dev->parent, match, dss6_component_compare, dev);

	return 0;
}

static int dss6_probe(struct platform_device *pdev)
{
	struct component_match *match = NULL;
	int r;

	/* add all the child devices as components */
	device_for_each_child(&pdev->dev, &match, dss6_add_child_component);

	if (!match) {
		dev_err(&pdev->dev, "No submodules found\n");
		return -EINVAL;
	}

	r = component_master_add_with_match(&pdev->dev, &dss6_component_ops, match);
	if (r)
		return r;

	r = initialize_omapdrm_device();
	if (r) {
		component_master_del(&pdev->dev, &dss6_component_ops);
		return r;
	}

	return 0;
}

static int dss6_remove(struct platform_device *pdev)
{
	uninitialize_omapdrm_device();

	component_master_del(&pdev->dev, &dss6_component_ops);
	return 0;
}

static int dss6_runtime_suspend(struct device *dev)
{
	struct dss_data *dss_data = dss_get_data(dev);

	clk_disable_unprepare(dss_data->fclk);

	return 0;
}

static int dss6_runtime_resume(struct device *dev)
{
	struct dss_data *dss_data = dss_get_data(dev);


	clk_prepare_enable(dss_data->fclk);

	if (REG_GET(dss_data, DSS_SYSSTATUS, 0, 0) == 0)
		dev_warn(dev, "DSS FUNC RESET not done!\n");

	return 0;
}

static const struct dev_pm_ops dss6_pm_ops = {
	.runtime_suspend = dss6_runtime_suspend,
	.runtime_resume = dss6_runtime_resume,
};

static const struct of_device_id dss6_of_match[] = {
	{ .compatible = "ti,k2g-dss", .data = &k2g_dss_feats, },
	{},
};

MODULE_DEVICE_TABLE(of, dss6_of_match);

static struct platform_driver dss6_driver = {
	.probe		= dss6_probe,
	.remove		= dss6_remove,
	.driver         = {
		.name   = "omap_dss6",
		.pm	= &dss6_pm_ops,
		.of_match_table = dss6_of_match,
		.suppress_bind_attrs = true,
	},
};

static int __init omap_dss_init(void)
{
	int r;

	r = platform_driver_register(&dss6_driver);
	if (r)
		return r;

	r = dispc6_init_platform_driver();
	if (r) {
		platform_driver_unregister(&dss6_driver);
		return r;
	}

	return 0;
}

static void __exit omap_dss_exit(void)
{
	dispc6_uninit_platform_driver();
	platform_driver_unregister(&dss6_driver);
}

module_init(omap_dss_init);
module_exit(omap_dss_exit);

MODULE_AUTHOR("Tomi Valkeinen <tomi.valkeinen@ti.com>");
MODULE_DESCRIPTION("OMAP6 Display Subsystem");
MODULE_LICENSE("GPL v2");
