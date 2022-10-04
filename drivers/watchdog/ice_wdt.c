// SPDX-License-Identifier: GPL-2.0-or-later

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/watchdog.h>

#define DRV_NAME		"ice-wdt"

#define WDT_LOCK		0x0
#define WDT_OT_LOAD_H		0x4
#define WDT_OT_LOAD_L		0x8
#define WDT_RMOD		0xC
#define WDT_EN			0x10
#define WDT_KNOCK		0x14
#define WDT_EOI 		0x18
#define WDT_CUR_VALUE_H 	0x1C
#define WDT_CUR_VALUE_L 	0x20
#define WDT_INT_STATE		0x24

#define REGS_WRITE_UNLOCK 	0x5ada7200
#define REGS_WRITE_LOCK 	0x5ada7201

#define TOUCH_WATCHDOG		0x55aadd22

#define ICE_WDT_TIMEOUT 	120

#define DEFAULT_TIMEOUT 	0xcccccccc

static unsigned int timeout;

struct ice_wdt_dev {
	struct watchdog_device wdt_dev;
	void __iomem *wdt_base;
};

static const struct watchdog_info ice_wdt_info = {
	.identity       = DRV_NAME,
	.options        = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
};

static void ice_wdt_change_timeout(struct watchdog_device *wdt_dev,
		unsigned int timeout)
{
	struct ice_wdt_dev *ice_wdt = watchdog_get_drvdata(wdt_dev);
	u32 timetick;

	timetick = (DEFAULT_TIMEOUT/120) * timeout;

	writel(REGS_WRITE_UNLOCK, ice_wdt->wdt_base + WDT_LOCK);
	writel(timetick, ice_wdt->wdt_base + WDT_OT_LOAD_L);
	writel(REGS_WRITE_LOCK, ice_wdt->wdt_base + WDT_LOCK);
}


static int ice_wdt_restart(struct watchdog_device *wdt_dev,
			     unsigned long action, void *data)
{
	struct ice_wdt_dev *ice_wdt = watchdog_get_drvdata(wdt_dev);

	writel(REGS_WRITE_UNLOCK, ice_wdt->wdt_base + WDT_LOCK);
	writel(0, ice_wdt->wdt_base + WDT_EN);
	writel(0, ice_wdt->wdt_base + WDT_RMOD);
	writel(0, ice_wdt->wdt_base + WDT_OT_LOAD_H);
	writel(0xff, ice_wdt->wdt_base + WDT_OT_LOAD_L);
	writel(1, ice_wdt->wdt_base + WDT_EN);
	writel(REGS_WRITE_LOCK, ice_wdt->wdt_base + WDT_LOCK);

	while (1);

	return 0;
}

static int ice_wdt_ping(struct watchdog_device *wdt_dev)
{
	struct ice_wdt_dev *ice_wdt = watchdog_get_drvdata(wdt_dev);

	writel(REGS_WRITE_UNLOCK, ice_wdt->wdt_base + WDT_LOCK);
	writel(TOUCH_WATCHDOG, ice_wdt->wdt_base + WDT_KNOCK);
	writel(REGS_WRITE_LOCK, ice_wdt->wdt_base + WDT_LOCK);

	return 0;
}

static int ice_wdt_set_timeout(struct watchdog_device *wdt_dev,
				 unsigned int timeout)
{
	wdt_dev->timeout = timeout;

	ice_wdt_change_timeout(wdt_dev, timeout);
	ice_wdt_ping(wdt_dev);

	return 0;
}

static int ice_wdt_stop(struct watchdog_device *wdt_dev)
{
	struct ice_wdt_dev *ice_wdt = watchdog_get_drvdata(wdt_dev);

	writel(REGS_WRITE_UNLOCK, ice_wdt->wdt_base + WDT_LOCK);
	writel(0, ice_wdt->wdt_base + WDT_EN);
	writel(REGS_WRITE_LOCK, ice_wdt->wdt_base + WDT_LOCK);

	return 0;
}

static int ice_wdt_start(struct watchdog_device *wdt_dev)
{
	struct ice_wdt_dev *ice_wdt = watchdog_get_drvdata(wdt_dev);

	writel(REGS_WRITE_UNLOCK, ice_wdt->wdt_base + WDT_LOCK);
	writel(1, ice_wdt->wdt_base + WDT_RMOD);
	writel(1, ice_wdt->wdt_base + WDT_EN);
	writel(REGS_WRITE_LOCK, ice_wdt->wdt_base + WDT_LOCK);

	return 0;
}

static const struct watchdog_ops ice_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= ice_wdt_start,
	.stop		= ice_wdt_stop,
	.ping		= ice_wdt_ping,
	.set_timeout	= ice_wdt_set_timeout,
	.restart        = ice_wdt_restart,
};

static const struct of_device_id ice_wdt_dt_ids[] = {
	{ .compatible = "ice,ice-wdt",},
	{ }
};

static int ice_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ice_wdt_dev *ice_wdt;
	int err;

	ice_wdt = devm_kzalloc(dev, sizeof(*ice_wdt), GFP_KERNEL);
	if (!ice_wdt) {
		return -ENOMEM;
	}

	ice_wdt->wdt_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ice_wdt->wdt_base)) {
		return PTR_ERR(ice_wdt->wdt_base);
	}

	ice_wdt->wdt_dev.parent = dev;
	ice_wdt->wdt_dev.info = &ice_wdt_info;
	ice_wdt->wdt_dev.ops = &ice_wdt_ops;
	ice_wdt->wdt_dev.timeout = ICE_WDT_TIMEOUT;
	ice_wdt->wdt_dev.min_timeout = 2;

	watchdog_set_drvdata(&ice_wdt->wdt_dev, ice_wdt);

	watchdog_init_timeout(&ice_wdt->wdt_dev, timeout, dev);
	watchdog_set_restart_priority(&ice_wdt->wdt_dev, 128);

	ice_wdt_stop(&ice_wdt->wdt_dev);

	err = devm_watchdog_register_device(dev, &ice_wdt->wdt_dev);
	if (err)
		return err;

	dev_info(dev, "Watchdog enabled (timeout=%d sec)\n", ice_wdt->wdt_dev.timeout);

	return 0;
}

static struct platform_driver ice_wdt_driver = {
	.probe		= ice_wdt_probe,
	.driver		= {
		.name		= DRV_NAME,
		.of_match_table	= ice_wdt_dt_ids,
	},
};

module_platform_driver(ice_wdt_driver);

module_param(timeout, uint, 0);
MODULE_PARM_DESC(timeout, "Watchdog heartbeat in seconds");

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Ice Watchdog Timer Driver");
