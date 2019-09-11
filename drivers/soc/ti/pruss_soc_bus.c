// SPDX-License-Identifier: GPL-2.0
/*
 * PRU-ICSS SoC bus driver for various TI SoCs
 *
 * Copyright (C) 2016-2019 Texas Instruments Incorporated - http://www.ti.com/
 *	Suman Anna <s-anna@ti.com>
 *	Keerthy <j-keerthy@ti.com>
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include <linux/platform_data/ti-pruss.h>

#define SYSCFG_STANDBY_INIT	BIT(4)
#define SYSCFG_SUB_MWAIT_READY	BIT(5)

#define SYSCFG_STANDBY_MODE_FORCE	(0 << 2)
#define SYSCFG_STANDBY_MODE_NO		(1 << 2)
#define SYSCFG_STANDBY_MODE_SMART	(2 << 2)
#define SYSCFG_STANDBY_MODE_MASK	(3 << 2)

#define SYSCFG_IDLE_MODE_FORCE		0
#define SYSCFG_IDLE_MODE_NO		1
#define SYSCFG_IDLE_MODE_SMART		2
#define SYSCFG_IDLE_MODE_MASK		3

/**
 * struct pruss_soc_bus - PRUSS SoC bus structure
 * @syscfg: kernel mapped address for SYSCFG register
 * @in_standby: flag for storing standby status
 * @has_reset: cached variable for storing global module reset flag
 * @skip_syscfg: flag to indicate if PRCM master standby/slave idle is needed
 */
struct pruss_soc_bus {
	void __iomem *syscfg;
	bool in_standby;
	bool has_reset;
	bool skip_syscfg;
};

/**
 * struct pruss_soc_bus_match_data - PRUSS SoC bus driver match data
 * @has_reset: flag to indicate the presence of global module reset
 * @uses_prcm: flag to indicate the usage of PRCM master standby/slave idle
 *	       protocol
 */
struct pruss_soc_bus_match_data {
	bool has_reset;
	bool uses_prcm;
};

static inline void pruss_soc_bus_rmw(void __iomem *reg, u32 mask, u32 set)
{
	u32 val;

	val = readl_relaxed(reg);
	val &= ~mask;
	val |= (set & mask);
	writel_relaxed(val, reg);
}

/*
 * This function programs the PRUSS_SYSCFG.STANDBY_INIT bit to achieve dual
 * functionalities - one is to deassert the MStandby signal to the device
 * PRCM, and the other is to enable OCP master ports to allow accesses
 * outside of the PRU-ICSS. The function has to wait for the PRCM to
 * acknowledge through the monitoring of the PRUSS_SYSCFG.SUB_MWAIT bit.
 */
static int pruss_soc_bus_enable_ocp_master_ports(struct device *dev)
{
	struct pruss_soc_bus *psoc_bus = dev_get_drvdata(dev);
	u32 syscfg_val, i;
	bool ready = false;

	pruss_soc_bus_rmw(psoc_bus->syscfg, SYSCFG_STANDBY_INIT, 0);

	/* wait till we are ready for transactions - delay is arbitrary */
	for (i = 0; i < 10; i++) {
		syscfg_val = readl_relaxed(psoc_bus->syscfg);
		ready = !(syscfg_val & SYSCFG_SUB_MWAIT_READY);
		if (ready)
			break;
		udelay(5);
	}

	if (!ready) {
		dev_err(dev, "timeout waiting for SUB_MWAIT_READY\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int __maybe_unused pruss_soc_bus_suspend(struct device *dev)
{
	struct pruss_soc_bus *psoc_bus = dev_get_drvdata(dev);
	u32 syscfg_val;

	if (psoc_bus->skip_syscfg)
		return 0;

	syscfg_val = readl_relaxed(psoc_bus->syscfg);
	psoc_bus->in_standby = syscfg_val & SYSCFG_STANDBY_INIT;

	/* initiate MStandby, undo the MStandby config in probe */
	if (!psoc_bus->in_standby) {
		pruss_soc_bus_rmw(psoc_bus->syscfg, SYSCFG_STANDBY_INIT,
				  SYSCFG_STANDBY_INIT);
	}

	return 0;
}

static int __maybe_unused pruss_soc_bus_resume(struct device *dev)
{
	struct pruss_soc_bus *psoc_bus = dev_get_drvdata(dev);
	int ret = 0;

	/* re-enable OCP master ports/disable MStandby */
	if (!psoc_bus->skip_syscfg && !psoc_bus->in_standby) {
		ret = pruss_soc_bus_enable_ocp_master_ports(dev);
		if (ret)
			dev_err(dev, "%s failed\n", __func__);
	}

	return ret;
}

/* firmware must be idle when calling this function */
static void pruss_disable_module(struct device *dev)
{
	struct pruss_soc_bus *psoc_bus = dev_get_drvdata(dev);

	if (psoc_bus->skip_syscfg)
		goto put_sync;

	/* configure Smart Standby */
	pruss_soc_bus_rmw(psoc_bus->syscfg, SYSCFG_STANDBY_MODE_MASK,
			  SYSCFG_STANDBY_MODE_SMART);

	/* initiate MStandby */
	pruss_soc_bus_rmw(psoc_bus->syscfg, SYSCFG_STANDBY_INIT,
			  SYSCFG_STANDBY_INIT);

put_sync:
	/* initiate IDLE request, disable clocks */
	pm_runtime_put_sync(dev);
}

static int pruss_enable_module(struct device *dev)
{
	struct pruss_soc_bus *psoc_bus = dev_get_drvdata(dev);
	int ret;

	/* enable clocks, de-assert IDLE request */
	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		pm_runtime_put_noidle(dev);
		return ret;
	}

	if (psoc_bus->skip_syscfg)
		return ret;

	/* configure for Smart Idle & Smart Standby */
	pruss_soc_bus_rmw(psoc_bus->syscfg, SYSCFG_IDLE_MODE_MASK,
			  SYSCFG_IDLE_MODE_SMART);
	pruss_soc_bus_rmw(psoc_bus->syscfg, SYSCFG_STANDBY_MODE_MASK,
			  SYSCFG_STANDBY_MODE_SMART);

	/* enable OCP master ports/disable MStandby */
	ret = pruss_soc_bus_enable_ocp_master_ports(dev);
	if (ret)
		pruss_disable_module(dev);

	return ret;
}

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
	psoc_bus->skip_syscfg = !data->uses_prcm;
	platform_set_drvdata(pdev, psoc_bus);

	if (psoc_bus->has_reset) {
		ret = pdata->deassert_reset(pdev, pdata->reset_name);
		if (ret) {
			dev_err(dev, "deassert_reset failed: %d\n", ret);
			goto fail_reset;
		}
	}

	pm_runtime_enable(dev);
	ret = pruss_enable_module(dev);
	if (ret < 0) {
		dev_err(dev, "couldn't enable module\n");
		goto fail_module;
	}

	ret = of_platform_populate(node, NULL, NULL, dev);
	if (ret)
		goto fail_of;

	return 0;

fail_of:
	pruss_disable_module(dev);
fail_module:
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

	pruss_disable_module(dev);
	pm_runtime_disable(dev);

	if (psoc_bus->has_reset)
		pdata->assert_reset(pdev, pdata->reset_name);
	iounmap(psoc_bus->syscfg);

	return 0;
}

/* instance-specific driver private data */
static const struct pruss_soc_bus_match_data am335x_data = {
	.has_reset = true,
	.uses_prcm = true,
};

static const struct pruss_soc_bus_match_data am437x_data = {
	.has_reset = true,
	.uses_prcm = true,
};

static const struct pruss_soc_bus_match_data am57xx_data = {
	.has_reset = false,
	.uses_prcm = true,
};

static const struct pruss_soc_bus_match_data k2g_data = {
	.has_reset = false,
	.uses_prcm = false,
};

static const struct of_device_id pruss_soc_bus_of_match[] = {
	{ .compatible = "ti,am3356-pruss-soc-bus", .data = &am335x_data, },
	{ .compatible = "ti,am4376-pruss-soc-bus", .data = &am437x_data, },
	{ .compatible = "ti,am5728-pruss-soc-bus", .data = &am57xx_data, },
	{ .compatible = "ti,k2g-pruss-soc-bus", .data = &k2g_data, },
	{ .compatible = "ti,am654-icssg-soc-bus", .data = &k2g_data, },
	{ .compatible = "ti,j721e-icssg-soc-bus", .data = &k2g_data, },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, pruss_soc_bus_of_match);

static SIMPLE_DEV_PM_OPS(pruss_soc_bus_pm_ops,
			 pruss_soc_bus_suspend, pruss_soc_bus_resume);

static struct platform_driver pruss_soc_bus_driver = {
	.driver	= {
		.name = "pruss-soc-bus",
		.pm = &pruss_soc_bus_pm_ops,
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
