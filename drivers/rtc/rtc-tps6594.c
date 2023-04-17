// SPDX-License-Identifier: GPL-2.0
/*
 * RTC driver for tps6594 PMIC
 *
 * Copyright (C) 2022 BayLibre Incorporated - https://www.baylibre.com/
 */

#include <linux/bcd.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/rtc.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/property.h>

#include <linux/mfd/tps6594.h>

#define TPS6594_GET_TIME_ON TPS6594_BIT_GET_TIME
#define TPS6594_GET_TIME_OFF 0
#define TPS6594_IT_ALARM_ON TPS6594_BIT_IT_ALARM
#define TPS6594_IT_ALARM_OFF 0
#define TPS6594_AUTO_COMP_ON TPS6594_BIT_IT_ALARM

/* Total number of RTC registers needed to set time*/
#define NUM_TIME_REGS (TPS6594_REG_RTC_WEEKS - TPS6594_REG_RTC_SECONDS + 1)

/* Total number of RTC alarm register */
#define NUM_TIME_ALARM_REGS (NUM_TIME_REGS - 1)

/* Total number of RTC registers needed to set compensation registers */
#define NUM_COMP_REGS (TPS6594_REG_RTC_COMP_MSB - TPS6594_REG_RTC_COMP_LSB + 1)

/*
 * Min and max values supported with 'offset' interface (swapped sign)
 * After conversion, the values does not exceed the range [-32767, 33767] which COMP_REG must
 * conform to
 */
#define MIN_OFFSET (-277774)
#define MAX_OFFSET (277774)

/* Number of ticks per hour */
#define TICKS_PER_HOUR (32768 * 3600)

/* Multiplier for ppb conversions */
#define PPB_MULT (1000000000LL)

static int tps6594_rtc_alarm_irq_enable(struct device *dev,
					unsigned int enabled)
{
	struct tps6594 *tps = dev_get_drvdata(dev->parent);
	u8 val = 0;
	int ret;

	val = enabled ? TPS6594_IT_ALARM_ON : TPS6594_IT_ALARM_OFF;

	ret = regmap_update_bits(tps->regmap, TPS6594_REG_RTC_INTERRUPTS,
				 TPS6594_BIT_IT_ALARM, val);

	return ret;
}

/* Pulse GET_TIME field of RTC_CTRL_1 to store a timestamp in shadow registers */
static int tps6594_rtc_shadow_timestamp(struct device *dev, struct tps6594 *tps)
{
	int ret;

	/*
	 * Set GET_TIME to 0. This way, next time we set GET_TIME to 1 we are sure to store an
	 * up-to-date timestamp
	 */
	ret = regmap_clear_bits(tps->regmap, TPS6594_REG_RTC_CTRL_1,
				TPS6594_BIT_GET_TIME);
	if (ret < 0)
		return ret;

	/*
	 * Copy content of RTC registers to shadow registers or latches to read a coherent
	 * timestamp
	 */
	return regmap_set_bits(tps->regmap, TPS6594_REG_RTC_CTRL_1,
			       TPS6594_BIT_GET_TIME);
}

/*
 * Gets current tps6594 RTC time and date parameters.
 *
 * The RTC's time/alarm representation is not what gmtime(3) requires
 * Linux to use:
 *
 *  - Months are 1..12 vs Linux 0-11
 *  - Years are 0..99 vs Linux 1900..N (we assume 21st century)
 */
static int tps6594_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	unsigned char rtc_data[NUM_TIME_REGS];
	struct tps6594 *tps = dev_get_drvdata(dev->parent);
	int ret;

	/* Check if RTC is running. */
	ret = regmap_test_bits(tps->regmap, TPS6594_REG_RTC_STATUS,
			       TPS6594_BIT_RUN);
	if (ret < 0)
		return ret;
	if (ret == 0)
		return -EINVAL;

	ret = tps6594_rtc_shadow_timestamp(dev, tps);
	if (ret < 0)
		return ret;

	/* Read shadowed RTC registers */
	ret = regmap_bulk_read(tps->regmap, TPS6594_REG_RTC_SECONDS, rtc_data,
			       NUM_TIME_REGS);
	if (ret < 0)
		return ret;

	tm->tm_sec = bcd2bin(rtc_data[0]);
	tm->tm_min = bcd2bin(rtc_data[1]);
	tm->tm_hour = bcd2bin(rtc_data[2]);
	tm->tm_mday = bcd2bin(rtc_data[3]);
	tm->tm_mon = bcd2bin(rtc_data[4]) - 1;
	tm->tm_year = bcd2bin(rtc_data[5]) + 100;
	tm->tm_wday = bcd2bin(rtc_data[6]);

	return ret;
}

/*
 * Sets current tps6594 RTC time and date parameters.
 *
 * The RTC's time/alarm representation is not what gmtime(3) requires
 * Linux to use:
 *
 *  - Months are 1..12 vs Linux 0-11
 *  - Years are 0..99 vs Linux 1900..N (we assume 21st century)
 */
static int tps6594_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	unsigned char rtc_data[NUM_TIME_REGS];
	struct tps6594 *tps = dev_get_drvdata(dev->parent);
	int ret;

	rtc_data[0] = bin2bcd(tm->tm_sec);
	rtc_data[1] = bin2bcd(tm->tm_min);
	rtc_data[2] = bin2bcd(tm->tm_hour);
	rtc_data[3] = bin2bcd(tm->tm_mday);
	rtc_data[4] = bin2bcd(tm->tm_mon + 1);
	rtc_data[5] = bin2bcd(tm->tm_year - 100);
	rtc_data[6] = bin2bcd(tm->tm_wday);

	/* Stop RTC while updating the RTC time registers */
	ret = regmap_clear_bits(tps->regmap, TPS6594_REG_RTC_CTRL_1,
				TPS6594_BIT_STOP_RTC);
	if (ret < 0)
		return ret;

	/* Update all the time registers in one shot */
	ret = regmap_bulk_write(tps->regmap, TPS6594_REG_RTC_SECONDS, rtc_data,
				NUM_TIME_REGS);
	if (ret < 0)
		return ret;

	/* Start back RTC */
	return regmap_set_bits(tps->regmap, TPS6594_REG_RTC_CTRL_1,
			       TPS6594_BIT_STOP_RTC);
}

/*
 * Gets current tps6594 RTC alarm time.
 */
static int tps6594_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	unsigned char alarm_data[NUM_TIME_ALARM_REGS];
	u32 int_val;
	struct tps6594 *tps = dev_get_drvdata(dev->parent);
	int ret;

	ret = regmap_bulk_read(tps->regmap, TPS6594_REG_ALARM_SECONDS,
			       alarm_data, NUM_TIME_ALARM_REGS);
	if (ret < 0)
		return ret;

	alm->time.tm_sec = bcd2bin(alarm_data[0]);
	alm->time.tm_min = bcd2bin(alarm_data[1]);
	alm->time.tm_hour = bcd2bin(alarm_data[2]);
	alm->time.tm_mday = bcd2bin(alarm_data[3]);
	alm->time.tm_mon = bcd2bin(alarm_data[4]) - 1;
	alm->time.tm_year = bcd2bin(alarm_data[5]) + 100;

	ret = regmap_read(tps->regmap, TPS6594_REG_RTC_INTERRUPTS, &int_val);
	if (ret < 0)
		return ret;

	alm->enabled = int_val & TPS6594_BIT_IT_ALARM ? 1 : 0;

	return ret;
}

static int tps6594_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	unsigned char alarm_data[NUM_TIME_ALARM_REGS];
	struct tps6594 *tps = dev_get_drvdata(dev->parent);
	int ret;

	/* Disable alarm irq before changing the alarm timestamp */
	ret = tps6594_rtc_alarm_irq_enable(dev, 0);
	if (ret)
		return ret;

	alarm_data[0] = bin2bcd(alm->time.tm_sec);
	alarm_data[1] = bin2bcd(alm->time.tm_min);
	alarm_data[2] = bin2bcd(alm->time.tm_hour);
	alarm_data[3] = bin2bcd(alm->time.tm_mday);
	alarm_data[4] = bin2bcd(alm->time.tm_mon + 1);
	alarm_data[5] = bin2bcd(alm->time.tm_year - 100);

	/* Update all the alarm registers in one shot */
	ret = regmap_bulk_write(tps->regmap, TPS6594_REG_ALARM_SECONDS,
				alarm_data, NUM_TIME_ALARM_REGS);
	if (ret < 0)
		return ret;

	if (alm->enabled)
		ret = tps6594_rtc_alarm_irq_enable(dev, 1);

	return ret;
}

static int tps6594_rtc_set_calibration(struct device *dev, int calibration)
{
	unsigned char comp_data[NUM_COMP_REGS];
	struct tps6594 *tps = dev_get_drvdata(dev->parent);
	s16 value;
	int ret;

	/*
	 * TPS6594 uses two's complement 16 bit value for compensation for RTC
	 * crystal inaccuracies. One time every hour when seconds counter
	 * increments from 0 to 1 compensation value will be added to internal
	 * RTC counter value.
	 *
	 *
	 * Valid range for compensation value: [-32767 .. 32767]
	 */
	if (calibration < -32767 || calibration > 32767) {
		dev_err(dev, "RTC calibration value out of range: %d\n",
			calibration);
		return -EINVAL;
	}

	value = (s16)calibration;

	comp_data[0] = (u16)value & 0xFF;
	comp_data[1] = ((u16)value >> 8) & 0xFF;

	/* Update all the compensation registers in one shot */
	ret = regmap_bulk_write(tps->regmap, TPS6594_REG_RTC_COMP_LSB,
				comp_data, NUM_COMP_REGS);
	if (ret < 0)
		return ret;

	/* Enable automatic compensation */
	return regmap_set_bits(tps->regmap, TPS6594_REG_RTC_CTRL_1,
			       TPS6594_BIT_AUTO_COMP);
}

static int tps6594_rtc_get_calibration(struct device *dev, int *calibration)
{
	unsigned char comp_data[NUM_COMP_REGS];
	struct tps6594 *tps = dev_get_drvdata(dev->parent);
	unsigned int ctrl;
	u16 value;
	int ret;

	ret = regmap_read(tps->regmap, TPS6594_REG_RTC_CTRL_1, &ctrl);
	if (ret < 0)
		return ret;

	/* If automatic compensation is not enabled report back zero */
	if (!(ctrl & TPS6594_BIT_AUTO_COMP)) {
		*calibration = 0;
		return 0;
	}

	ret = regmap_bulk_read(tps->regmap, TPS6594_REG_RTC_COMP_LSB, comp_data,
			       NUM_COMP_REGS);
	if (ret < 0)
		return ret;

	value = (u16)comp_data[0] | ((u16)comp_data[1] << 8);

	*calibration = (s16)value;

	return ret;
}

static int tps6594_rtc_read_offset(struct device *dev, long *offset)
{
	int calibration;
	s64 tmp;
	int ret;

	ret = tps6594_rtc_get_calibration(dev, &calibration);
	if (ret < 0)
		return ret;

	/* Convert from RTC calibration register format to ppb format */
	tmp = calibration * (s64)PPB_MULT;
	if (tmp < 0)
		tmp -= TICKS_PER_HOUR / 2LL;
	else
		tmp += TICKS_PER_HOUR / 2LL;
	tmp = div_s64(tmp, TICKS_PER_HOUR);

	/*
	 * Offset value operates in negative way, so swap sign.
	 * See 8.3.10.5, (32768 - COMP_REG)
	 */
	*offset = (long)-tmp;

	return ret;
}

static int tps6594_rtc_set_offset(struct device *dev, long offset)
{
	int calibration;
	s64 tmp;

	/* Make sure offset value is within supported range */
	if (offset < MIN_OFFSET || offset > MAX_OFFSET)
		return -ERANGE;

	/* Convert from ppb format to RTC calibration register format */
	tmp = offset * (s64)TICKS_PER_HOUR;
	if (tmp < 0)
		tmp -= PPB_MULT / 2LL;
	else
		tmp += PPB_MULT / 2LL;
	tmp = div_s64(tmp, PPB_MULT);

	/* Offset value operates in negative way, so swap sign */
	calibration = (int)-tmp;

	return tps6594_rtc_set_calibration(dev, calibration);
}

static irqreturn_t tps6594_rtc_interrupt(int irq, void *rtc)
{
	struct device *dev = rtc;
	unsigned long events = 0;
	struct tps6594 *tps = dev_get_drvdata(dev->parent);
	struct rtc_device *rtc_dev = dev_get_drvdata(dev);
	int ret;
	u32 rtc_reg;

	ret = regmap_read(tps->regmap, TPS6594_REG_RTC_STATUS, &rtc_reg);
	if (ret)
		return IRQ_NONE;

	if (rtc_reg & TPS6594_BIT_ALARM)
		events = RTC_IRQF | RTC_AF;

	/* Notify RTC core on event */
	rtc_update_irq(rtc_dev, 1, events);

	return IRQ_HANDLED;
}

static const struct rtc_class_ops tps6594_rtc_ops = {
	.read_time = tps6594_rtc_read_time,
	.set_time = tps6594_rtc_set_time,
	.read_alarm = tps6594_rtc_read_alarm,
	.set_alarm = tps6594_rtc_set_alarm,
	.alarm_irq_enable = tps6594_rtc_alarm_irq_enable,
	.read_offset = tps6594_rtc_read_offset,
	.set_offset = tps6594_rtc_set_offset,
};

static int tps6594_rtc_probe(struct platform_device *pdev)
{
	struct tps6594 *tps;
	struct rtc_device *rtc;
	int irq;
	int ret;

	tps = dev_get_drvdata(pdev->dev.parent);

	rtc = devm_kzalloc(&pdev->dev, sizeof(*rtc), GFP_KERNEL);
	if (!rtc)
		return -ENOMEM;

	rtc = devm_rtc_allocate_device(&pdev->dev);
	if (IS_ERR(rtc))
		return PTR_ERR(rtc);

	/* Enable crystal oscillator */
	ret = regmap_set_bits(tps->regmap, TPS6594_REG_RTC_CTRL_2,
			      TPS6594_BIT_XTAL_EN);
	if (ret < 0)
		return ret;

	ret = regmap_test_bits(tps->regmap, TPS6594_REG_RTC_STATUS,
			       TPS6594_BIT_RUN);
	if (ret < 0)
		return ret;
	/* RTC not running */
	if (ret == 0) {
		/* Start rtc */
		ret = regmap_set_bits(tps->regmap, TPS6594_REG_RTC_CTRL_1,
				      TPS6594_BIT_STOP_RTC);
		if (ret < 0)
			return ret;

		mdelay(100);

		/*
		 * RTC should be running now. Check if this is the case.
		 * If not it might be a missing oscillator.
		 */
		ret = regmap_test_bits(tps->regmap, TPS6594_REG_RTC_STATUS,
				       TPS6594_BIT_RUN);
		if (ret < 0)
			return ret;
		if (ret == 0)
			return -ENODEV;

		/* Stop RTC until first call to `tps6594_rtc_set_time */
		ret = regmap_clear_bits(tps->regmap, TPS6594_REG_RTC_CTRL_1,
					TPS6594_BIT_STOP_RTC);
		if (ret < 0)
			return ret;
	}

	platform_set_drvdata(pdev, rtc);

	irq = platform_get_irq_byname(pdev, TPS6594_IRQ_NAME_ALARM);
	if (irq < 0) {
		dev_err(&pdev->dev, "Failed to get irq\n");
		return irq;
	}

	ret = devm_request_threaded_irq(&pdev->dev, irq, NULL,
					tps6594_rtc_interrupt, IRQF_ONESHOT,
					TPS6594_IRQ_NAME_ALARM, &pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to request_threaded_irq\n");
		return ret;
	}

	rtc->ops = &tps6594_rtc_ops;
	rtc->range_min = RTC_TIMESTAMP_BEGIN_2000;
	rtc->range_max = RTC_TIMESTAMP_END_2099;

	return devm_rtc_register_device(rtc);
}

static struct platform_driver tps6594_rtc_driver = {
	.probe		= tps6594_rtc_probe,
	.driver		= {
		.name	= "tps6594-rtc",
	},
};

module_platform_driver(tps6594_rtc_driver);

MODULE_ALIAS("platform:tps6594-rtc");
MODULE_AUTHOR("Esteban Blanc <eblanc@baylibre.com>");
MODULE_DESCRIPTION("TPS6594 RTC driver");
MODULE_LICENSE("GPL");
