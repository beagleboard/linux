// SPDX-License-Identifier: GPL-2.0
/*
 * PRU-ICSS SoC bus driver for various TI SoCs
 *
 * Copyright (C) 2016-2019 Texas Instruments Incorporated - http://www.ti.com/
 *	Suman Anna <s-anna@ti.com>
 *	Keerthy <j-keerthy@ti.com>
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include <linux/platform_data/ti-pruss.h>

/**
 * struct pruss_soc_bus - PRUSS SoC bus structure
 * @syscfg: kernel mapped address for SYSCFG register
 * @has_reset: cached variable for storing global module reset flag
 */
struct pruss_soc_bus {
	void __iomem *syscfg;
	bool has_reset;
};

/**
 * struct pruss_soc_bus_match_data - PRUSS SoC bus driver match data
 * @has_reset: flag to indicate the presence of global module reset
 */
struct pruss_soc_bus_match_data {
	bool has_reset;
};

static int pruss_soc_bus_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct pruss_platform_data *pdata = dev_get_platdata(dev);
	struct pruss_soc_bus *psoc_bus;
	const struct pruss_soc_bus_match_data *data;
	int ret;

	psoc_bus = devm_kzalloc(dev, sizeof(*psoc_bus), GFP_KERNEL);
	if (!psoc_bus)
		return -ENOMEM;

	psoc_bus->syscfg = of_iomap(node, 0);
	if (!psoc_bus->syscfg)
		return -ENOMEM;

	data = of_device_get_match_data(dev);
	if (!data) {
		dev_err(dev, "missing match data\n");
		return -ENODEV;
	}

	if (data->has_reset && (!pdata || !pdata->deassert_reset ||
				!pdata->assert_reset || !pdata->reset_name)) {
		dev_err(dev, "platform data (reset configuration information) missing\n");
		return -ENODEV;
	}
	psoc_bus->has_reset = data->has_reset;
	platform_set_drvdata(pdev, psoc_bus);

	if (psoc_bus->has_reset) {
		ret = pdata->deassert_reset(pdev, pdata->reset_name);
		if (ret) {
			dev_err(dev, "deassert_reset failed: %d\n", ret);
			goto fail_reset;
		}
	}

	pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		pm_runtime_put_noidle(dev);
		goto fail_clock;
	}

	ret = of_platform_populate(node, NULL, NULL, dev);
	if (ret)
		goto fail_of;

	return 0;

fail_of:
	pm_runtime_put_sync(dev);
fail_clock:
	pm_runtime_disable(dev);
	if (psoc_bus->has_reset)
		pdata->assert_reset(pdev, pdata->reset_name);
fail_reset:
	iounmap(psoc_bus->syscfg);
	return ret;
}

static int pruss_soc_bus_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pruss_platform_data *pdata = dev_get_platdata(dev);
	struct pruss_soc_bus *psoc_bus = platform_get_drvdata(pdev);

	of_platform_depopulate(dev);

	pm_runtime_put_sync(dev);
	pm_runtime_disable(dev);

	if (psoc_bus->has_reset)
		pdata->assert_reset(pdev, pdata->reset_name);
	iounmap(psoc_bus->syscfg);

	return 0;
}

/* instance-specific driver private data */
static const struct pruss_soc_bus_match_data am335x_data = {
	.has_reset = true,
};

static const struct pruss_soc_bus_match_data am437x_data = {
	.has_reset = true,
};

static const struct pruss_soc_bus_match_data am57xx_data = {
	.has_reset = false,
};

static const struct of_device_id pruss_soc_bus_of_match[] = {
	{ .compatible = "ti,am3356-pruss-soc-bus", .data = &am335x_data, },
	{ .compatible = "ti,am4376-pruss-soc-bus", .data = &am437x_data, },
	{ .compatible = "ti,am5728-pruss-soc-bus", .data = &am57xx_data, },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, pruss_soc_bus_of_match);

static struct platform_driver pruss_soc_bus_driver = {
	.driver	= {
		.name = "pruss-soc-bus",
		.of_match_table = pruss_soc_bus_of_match,
	},
	.probe	= pruss_soc_bus_probe,
	.remove	= pruss_soc_bus_remove,
};
module_platform_driver(pruss_soc_bus_driver);

MODULE_AUTHOR("Suman Anna <s-anna@ti.com>");
MODULE_AUTHOR("Keerthy <j-keerthy@ti.com>");
MODULE_DESCRIPTION("PRU-ICSS SoC Bus Driver for TI SoCs");
MODULE_LICENSE("GPL v2");
