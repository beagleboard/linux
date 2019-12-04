// SPDX-License-Identifier: GPL-2.0
/*
 * Watchdog driver for the K3 RTI module
 *
 * (c) Copyright 2019 Texas Instruments Inc.
 * All rights reserved.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mod_devicetable.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/watchdog.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/pm_runtime.h>

#define MODULE_NAME "rti-wdt"
#define DEFAULT_HEARTBEAT 60
#define MAX_HEARTBEAT     1000

/* Timer register set definition */
#define RTIDWDCTRL	0x90
#define RTIDWDPRLD	0x94
#define RTIWDSTATUS	0x98
#define RTIWDKEY	0x9c
#define RTIDWDCNTR	0xa0
#define RTIWWDRXCTRL	0xa4
#define RTIWWDSIZECTRL	0xa8

#define RTIWWDRX_NMI	0xa

#define RTIWWDSIZE_50P	0x50

#define WDENABLE_KEY	0xa98559da

#define WDKEY_SEQ0		0xe51a
#define WDKEY_SEQ1		0xa35c

#define WDT_PRELOAD_SHIFT	13

#define WDT_PRELOAD_MAX		0xfff

#define DWDST			BIT(1)

static int heartbeat;

/*
 * struct to hold data for each WDT device
 * @base - base io address of WD device
 * @clk - source clock of WDT
 * @wdd - hold watchdog device as is in WDT core
 */
struct rti_wdt_device {
	void __iomem		*base;
	struct clk		*clk;
	struct watchdog_device	wdd;
};

static int rti_wdt_start(struct watchdog_device *wdd)
{
	u32 timer_margin;
	unsigned long freq;
	struct rti_wdt_device *wdt = watchdog_get_drvdata(wdd);

	freq = clk_get_rate(wdt->clk);

	/* set timeout period */
	timer_margin = (u64)wdd->timeout * freq;
	timer_margin >>= WDT_PRELOAD_SHIFT;
	if (timer_margin > WDT_PRELOAD_MAX)
		timer_margin = WDT_PRELOAD_MAX;
	writel_relaxed(timer_margin, wdt->base + RTIDWDPRLD);

	/* Set min heartbeat to 1.1x window size */
	wdd->min_hw_heartbeat_ms = 11 * wdd->timeout * 1000 / 20;

	/* Generate NMI when wdt expires */
	writel_relaxed(RTIWWDRX_NMI, wdt->base + RTIWWDRXCTRL);

	/* Window size 50% */
	writel_relaxed(RTIWWDSIZE_50P, wdt->base + RTIWWDSIZECTRL);

	readl_relaxed(wdt->base + RTIWWDSIZECTRL);

	/* enable watchdog */
	writel_relaxed(WDENABLE_KEY, wdt->base + RTIDWDCTRL);
	return 0;
}

static int rti_wdt_ping(struct watchdog_device *wdd)
{
	struct rti_wdt_device *wdt = watchdog_get_drvdata(wdd);

	/* put watchdog in service state */
	writel_relaxed(WDKEY_SEQ0, wdt->base + RTIWDKEY);
	/* put watchdog in active state */
	writel_relaxed(WDKEY_SEQ1, wdt->base + RTIWDKEY);

	if (readl_relaxed(wdt->base + RTIWDSTATUS))
		WARN_ON_ONCE(1);

	return 0;
}

static unsigned int rti_wdt_get_timeleft(struct watchdog_device *wdd)
{
	u64 timer_counter;
	unsigned long freq;
	u32 val;
	struct rti_wdt_device *wdt = watchdog_get_drvdata(wdd);

	/* if timeout has occurred then return 0 */
	val = readl_relaxed(wdt->base + RTIWDSTATUS);
	if (val & DWDST)
		return 0;

	freq = clk_get_rate(wdt->clk);
	if (!freq)
		return 0;

	timer_counter = readl_relaxed(wdt->base + RTIDWDCNTR);

	do_div(timer_counter, freq);

	return timer_counter;
}

static const struct watchdog_info rti_wdt_info = {
	.options = WDIOF_KEEPALIVEPING,
	.identity = "K3 RTI Watchdog",
};

static const struct watchdog_ops rti_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= rti_wdt_start,
	.ping		= rti_wdt_ping,
	.get_timeleft	= rti_wdt_get_timeleft,
};

static int rti_wdt_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev = &pdev->dev;
	struct resource *wdt_mem;
	struct watchdog_device *wdd;
	struct rti_wdt_device *wdt;

	wdt = devm_kzalloc(dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	wdt->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(wdt->clk)) {
		if (PTR_ERR(wdt->clk) != -EPROBE_DEFER)
			dev_err(dev, "failed to get clock\n");
		return PTR_ERR(wdt->clk);
	}

	pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "runtime pm failed\n");
		return ret;
	}

	platform_set_drvdata(pdev, wdt);

	wdd = &wdt->wdd;
	wdd->info = &rti_wdt_info;
	wdd->ops = &rti_wdt_ops;
	wdd->min_timeout = 1;
	/* Set min heartbeat to 1.1x window size */
	wdd->min_hw_heartbeat_ms = 11 * DEFAULT_HEARTBEAT * 1000 / 20;
	wdd->max_hw_heartbeat_ms = MAX_HEARTBEAT * 1000;
	wdd->timeout = DEFAULT_HEARTBEAT;
	wdd->parent = dev;

	set_bit(WDOG_RESET_KEEPALIVE, &wdd->status);

	watchdog_init_timeout(wdd, heartbeat, dev);

	dev_info(dev, "heartbeat %d sec\n", wdd->timeout);

	watchdog_set_drvdata(wdd, wdt);
	watchdog_set_nowayout(wdd, 1);
	watchdog_set_restart_priority(wdd, 128);

	wdt_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	wdt->base = devm_ioremap_resource(dev, wdt_mem);
	if (IS_ERR(wdt->base)) {
		ret = PTR_ERR(wdt->base);
		goto err_iomap;
	}

	ret = watchdog_register_device(wdd);
	if (ret) {
		dev_err(dev, "cannot register watchdog device\n");
		goto err_iomap;
	}

	return 0;

err_iomap:
	pm_runtime_put_sync(&pdev->dev);

	return ret;
}

static int rti_wdt_remove(struct platform_device *pdev)
{
	struct rti_wdt_device *wdt = platform_get_drvdata(pdev);

	watchdog_unregister_device(&wdt->wdd);
	pm_runtime_put(&pdev->dev);

	return 0;
}

static const struct of_device_id rti_wdt_of_match[] = {
	{ .compatible = "ti,rti-wdt", },
	{},
};
MODULE_DEVICE_TABLE(of, rti_wdt_of_match);

static struct platform_driver rti_wdt_driver = {
	.driver = {
		.name = "rti-wdt",
		.of_match_table = rti_wdt_of_match,
	},
	.probe = rti_wdt_probe,
	.remove = rti_wdt_remove,
};

module_platform_driver(rti_wdt_driver);

MODULE_AUTHOR("Tero Kristo <t-kristo@ti.com>");
MODULE_DESCRIPTION("K3 RTI Watchdog Driver");

module_param(heartbeat, int, 0);
MODULE_PARM_DESC(heartbeat,
		 "Watchdog heartbeat period in seconds from 1 to "
		 __MODULE_STRING(MAX_HEARTBEAT) ", default "
		 __MODULE_STRING(DEFAULT_HEARTBEAT));

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rti-wdt");
