// SPDX-License-Identifier: GPL-2.0-only
/*
 * rtc-tps6594x.c -- TPS6594x Real Time Clock driver.
 *
 * Copyright (C) 2022 Texas Instruments Incorporated - https://www.ti.com
 *
 * TODO: alarm support
 */

#include <linux/bcd.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mfd/tps6594x.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/rtc.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

struct tps6594x_rtc {
	struct rtc_device	*rtc;
	struct device		*dev;
};

#define TPS6594X_NUM_TIME_REGS	(TPS6594X_RTC_YEARS - TPS6594X_RTC_SECONDS + 1)

static int tps6594x_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	unsigned char rtc_data[TPS6594X_NUM_TIME_REGS];
	struct tps6594x *tps6594x = dev_get_drvdata(dev->parent);
	int ret;

	/* Reset TPS6594X_RTC_CTRL_REG_GET_TIME bit to zero, required for latch */
	ret = regmap_update_bits(tps6594x->regmap, TPS6594X_RTC_CTRL_1,
		TPS6594X_RTC_CTRL_REG_GET_TIME, 0);
	if (ret < 0) {
		dev_err(dev, "RTC CTRL reg update failed, err: %d\n", ret);
		return ret;
	}

	/* Copy RTC counting registers to static registers or latches */
	ret = regmap_update_bits(tps6594x->regmap, TPS6594X_RTC_CTRL_1,
		TPS6594X_RTC_CTRL_REG_GET_TIME, TPS6594X_RTC_CTRL_REG_GET_TIME);
	if (ret < 0) {
		dev_err(dev, "RTC CTRL reg update failed, err: %d\n", ret);
		return ret;
	}

	ret = regmap_bulk_read(tps6594x->regmap, TPS6594X_RTC_SECONDS,
			rtc_data, TPS6594X_NUM_TIME_REGS);
	if (ret < 0) {
		dev_err(dev, "RTC_SECONDS reg read failed, err = %d\n", ret);
		return ret;
	}

	tm->tm_sec = bcd2bin(rtc_data[0]);
	tm->tm_min = bcd2bin(rtc_data[1]);
	tm->tm_hour = bcd2bin(rtc_data[2]);
	tm->tm_mday = bcd2bin(rtc_data[3]);
	tm->tm_mon = bcd2bin(rtc_data[4]) - 1;
	tm->tm_year = bcd2bin(rtc_data[5]) + 100;

	return ret;
}

static int tps6594x_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	unsigned char rtc_data[TPS6594X_NUM_TIME_REGS];
	struct tps6594x *tps6594x = dev_get_drvdata(dev->parent);
	int ret, retries = 5;
	unsigned int val;

	rtc_data[0] = bin2bcd(tm->tm_sec);
	rtc_data[1] = bin2bcd(tm->tm_min);
	rtc_data[2] = bin2bcd(tm->tm_hour);
	rtc_data[3] = bin2bcd(tm->tm_mday);
	rtc_data[4] = bin2bcd(tm->tm_mon + 1);
	rtc_data[5] = bin2bcd(tm->tm_year - 100);

	/* Stop RTC while updating the RTC time registers */
	ret = regmap_update_bits(tps6594x->regmap, TPS6594X_RTC_CTRL_1,
				 TPS6594X_RTC_CTRL_REG_STOP_RTC, 0);
	if (ret < 0) {
		dev_err(dev, "RTC stop failed, err = %d\n", ret);
		return ret;
	}

	/* Waiting till RTC isn't running */
	do {
		ret = regmap_read(tps6594x->regmap, TPS6594X_RTC_STATUS, &val);
		if (ret < 0) {
			dev_err(dev, "RTC_STATUS reg read failed, err = %d\n", ret);
			return ret;
		}
		msleep(20);
	} while (--retries && (val & TPS6594X_RTC_STATUS_RUN));

	if (!retries) {
		dev_err(dev, "RTC_STATUS is still RUNNING\n");
		return -ETIMEDOUT;
	}

	ret = regmap_bulk_write(tps6594x->regmap, TPS6594X_RTC_SECONDS,
		rtc_data, TPS6594X_NUM_TIME_REGS);
	if (ret < 0) {
		dev_err(dev, "RTC_SECONDS reg write failed, err = %d\n", ret);
		return ret;
	}

	/* Start back RTC */
	ret = regmap_update_bits(tps6594x->regmap, TPS6594X_RTC_CTRL_1,
				 TPS6594X_RTC_CTRL_REG_STOP_RTC,
				 TPS6594X_RTC_CTRL_REG_STOP_RTC);
	if (ret < 0)
		dev_err(dev, "RTC start failed, err = %d\n", ret);

	return ret;
}

static const struct rtc_class_ops tps6594x_rtc_ops = {
	.read_time	= tps6594x_rtc_read_time,
	.set_time	= tps6594x_rtc_set_time,
};

static int tps6594x_rtc_probe(struct platform_device *pdev)
{
	struct tps6594x *tps6594x = dev_get_drvdata(pdev->dev.parent);
	struct tps6594x_rtc *tps6594x_rtc = NULL;
	int ret;

	tps6594x_rtc = devm_kzalloc(&pdev->dev, sizeof(struct tps6594x_rtc), GFP_KERNEL);
	if (!tps6594x_rtc)
		return -ENOMEM;

	tps6594x_rtc->dev = &pdev->dev;
	platform_set_drvdata(pdev, tps6594x_rtc);

	/* Start RTC */
	ret = regmap_update_bits(tps6594x->regmap, TPS6594X_RTC_CTRL_1,
				 TPS6594X_RTC_CTRL_REG_STOP_RTC,
				 TPS6594X_RTC_CTRL_REG_STOP_RTC);
	if (ret < 0) {
		dev_err(&pdev->dev, "RTC_CTRL write failed, err = %d\n", ret);
		return ret;
	}

	tps6594x_rtc->rtc = devm_rtc_device_register(&pdev->dev, pdev->name,
				&tps6594x_rtc_ops, THIS_MODULE);
	if (IS_ERR(tps6594x_rtc->rtc)) {
		ret = PTR_ERR(tps6594x_rtc->rtc);
		dev_err(&pdev->dev, "RTC register failed, err = %d\n", ret);
		return ret;
	}

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id of_tps6594x_rtc_match[] = {
	{ .compatible = "ti,tps6594x-rtc", },
	{},
};
MODULE_DEVICE_TABLE(of, of_tps6594x_rtc_match);
#endif

static struct platform_driver tps6594x_rtc_driver = {
	.probe		= tps6594x_rtc_probe,
	.driver		= {
		.name	= "tps6594x-rtc",
		.of_match_table = of_match_ptr(of_tps6594x_rtc_match),
	},
};

module_platform_driver(tps6594x_rtc_driver);

MODULE_ALIAS("platform:tps6594x_rtc");
MODULE_DESCRIPTION("TI TPS6594x series RTC driver");
MODULE_AUTHOR("Keerthy J <j-keerthy@ti.com>");
MODULE_LICENSE("GPL");
