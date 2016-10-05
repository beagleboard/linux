/*
 * PRU-ICSS SYSCFG IP wrapper driver for TI AM437x SoCs
 *
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 *	Suman Anna <s-anna@ti.com>
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

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include <linux/platform_data/remoteproc-pruss.h>

#define SYSCFG_OFFSET		0x4
#define SYSCFG_STANDBY_INIT	BIT(4)
#define SYSCFG_SUB_MWAIT_READY	BIT(5)

/**
 * struct am437x_pruss_wrapper - PRUSS wrapper structure
 * @base: kernel mapped address for wrapper register base
 * @in_standby: flag for storing standby status
 */
struct am437x_pruss_wrapper {
	void __iomem *base;
	bool in_standby;
};

static inline void pruss_wrapper_rmw(void __iomem *base, unsigned int offset,
				     u32 mask, u32 set)
{
	u32 val;

	val = readl_relaxed(base + offset);
	val &= ~mask;
	val |= (set & mask);
	writel_relaxed(val, base + offset);
}

/*
 * This function programs the PRUSS_SYSCFG.STANDBY_INIT bit to achieve dual
 * functionalities - one is to deassert the MStandby signal to the device
 * PRCM, and the other is to enable OCP master ports to allow accesses
 * outside of the PRU-ICSS. The function has to wait for the PRCM to
 * acknowledge through the monitoring of the PRUSS_SYSCFG.SUB_MWAIT bit.
 */
static int pruss_wrapper_enable_ocp_master_ports(struct device *dev)
{
	struct am437x_pruss_wrapper *wrapper = dev_get_drvdata(dev);
	u32 syscfg_val, i;
	bool ready = false;

	pruss_wrapper_rmw(wrapper->base, SYSCFG_OFFSET, SYSCFG_STANDBY_INIT, 0);

	/* wait till we are ready for transactions - delay is arbitrary */
	for (i = 0; i < 10; i++) {
		syscfg_val = readl_relaxed(wrapper->base + SYSCFG_OFFSET);
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

#ifdef CONFIG_PM_SLEEP
static int am437x_pruss_wrapper_suspend(struct device *dev)
{
	struct am437x_pruss_wrapper *wrapper = dev_get_drvdata(dev);
	u32 syscfg_val;

	syscfg_val = readl_relaxed(wrapper->base + SYSCFG_OFFSET);
	wrapper->in_standby = syscfg_val & SYSCFG_STANDBY_INIT;

	/* initiate MStandby, undo the MStandby config in probe */
	if (!wrapper->in_standby) {
		pruss_wrapper_rmw(wrapper->base, SYSCFG_OFFSET,
				  SYSCFG_STANDBY_INIT, SYSCFG_STANDBY_INIT);
	}

	return 0;
}

static int am437x_pruss_wrapper_resume(struct device *dev)
{
	struct am437x_pruss_wrapper *wrapper = dev_get_drvdata(dev);
	int ret = 0;

	/* re-enable OCP master ports/disable MStandby */
	if (!wrapper->in_standby) {
		ret = pruss_wrapper_enable_ocp_master_ports(dev);
		if (ret)
			dev_err(dev, "%s failed\n", __func__);
	}

	return ret;
}
#endif /* CONFIG_PM_SLEEP */

static int am437x_pruss_wrapper_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct pruss_platform_data *pdata = dev_get_platdata(dev);
	struct am437x_pruss_wrapper *wrapper;
	int ret;

	if (!pdata || !pdata->deassert_reset || !pdata->assert_reset ||
	    !pdata->reset_name) {
		dev_err(dev, "platform data (reset configuration information) missing\n");
		return -ENODEV;
	}

	wrapper = devm_kzalloc(dev, sizeof(*wrapper), GFP_KERNEL);
	if (!wrapper)
		return -ENOMEM;

	wrapper->base = of_iomap(node, 0);
	if (!wrapper->base)
		return -ENOMEM;

	platform_set_drvdata(pdev, wrapper);

	ret = pdata->deassert_reset(pdev, pdata->reset_name);
	if (ret) {
		dev_err(dev, "deassert_reset failed: %d\n", ret);
		goto fail_reset;
	}

	pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		pm_runtime_put_noidle(dev);
		goto fail_clock;
	}

	ret = pruss_wrapper_enable_ocp_master_ports(dev);
	if (ret)
		goto fail_ocp;

	ret = of_platform_populate(node, NULL, NULL, dev);
	if (ret)
		goto fail_ocp;

	return 0;

fail_ocp:
	pm_runtime_put_sync(dev);
fail_clock:
	pm_runtime_disable(dev);
	pdata->assert_reset(pdev, pdata->reset_name);
fail_reset:
	iounmap(wrapper->base);
	return ret;
}

static int am437x_pruss_wrapper_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct pruss_platform_data *pdata = dev_get_platdata(dev);
	struct am437x_pruss_wrapper *wrapper = platform_get_drvdata(pdev);

	of_platform_depopulate(dev);

	pruss_wrapper_rmw(wrapper->base, SYSCFG_OFFSET,
			  SYSCFG_STANDBY_INIT, SYSCFG_STANDBY_INIT);
	pm_runtime_put_sync(dev);
	pm_runtime_disable(dev);
	pdata->assert_reset(pdev, pdata->reset_name);
	iounmap(wrapper->base);

	return 0;
}

static SIMPLE_DEV_PM_OPS(am437x_pruss_wrapper_pm_ops,
			 am437x_pruss_wrapper_suspend,
			 am437x_pruss_wrapper_resume);

static const struct of_device_id am437x_pruss_wrapper_of_match[] = {
	{ .compatible = "ti,am4372-pruss-wrapper" },
	{  },
};
MODULE_DEVICE_TABLE(of, am437x_pruss_wrapper_of_match);

static struct platform_driver am437x_pruss_wrapper_driver = {
	.driver	= {
		.name = "am437x-pruss-wrapper",
		.pm = &am437x_pruss_wrapper_pm_ops,
		.of_match_table = am437x_pruss_wrapper_of_match,
	},
	.probe	= am437x_pruss_wrapper_probe,
	.remove	= am437x_pruss_wrapper_remove,
};
module_platform_driver(am437x_pruss_wrapper_driver);

MODULE_AUTHOR("Suman Anna <s-anna@ti.com>");
MODULE_DESCRIPTION("PRU-ICSS SYSCFG IP Wrapper Driver for AM437x SoCs");
MODULE_LICENSE("GPL v2");
