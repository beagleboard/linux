/*
 * rtc-twl4030.c -- TWL4030 Real Time Clock interface
 *
 * Copyright (C) 2007 MontaVista Software, Inc
 * Author: Alexandre Rusev <source@mvista.com>
 *
 * Based on original TI driver twl4030-rtc.c
 *   Copyright (C) 2006 Texas Instruments, Inc.
 *
 * Based on rtc-omap.c
 *   Copyright (C) 2003 MontaVista Software, Inc.
 *   Author: George G. Davis <gdavis@mvista.com> or <source@mvista.com>
 *   Copyright (C) 2006 David Brownell
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/i2c/twl4030.h>
#include <linux/i2c/twl4030-rtc.h>
#include <linux/io.h>
#include <linux/irq.h>

#include <asm/mach/time.h>
#include <asm/system.h>
#include <mach/hardware.h>

#define ALL_TIME_REGS		6

/*
 * Supports 1 byte read from TWL4030 RTC register.
 */
static int twl4030_rtc_read_u8(u8 *data, u8 reg)
{
	int ret;

	ret = twl4030_i2c_read_u8(TWL4030_MODULE_RTC, data, reg);
	if (ret < 0)
		pr_err("twl4030_rtc: Could not read TWL4030"
		       "register %X - error %d\n", reg, ret);
	return ret;
}

/*
 * Supports 1 byte write to TWL4030 RTC registers.
 */
static int twl4030_rtc_write_u8(u8 data, u8 reg)
{
	int ret;

	ret = twl4030_i2c_write_u8(TWL4030_MODULE_RTC, data, reg);
	if (ret < 0)
		pr_err("twl4030_rtc: Could not write TWL4030"
		       "register %X - error %d\n", reg, ret);
	return ret;
}

/*
 * Cache the value for timer/alarm interrupts register; this is
 * only changed by callers holding rtc ops lock (or resume).
 */
static unsigned char rtc_irq_bits;

/*
 * Enable timer and/or alarm interrupts.
 */
static int set_rtc_irq_bit(unsigned char bit)
{
	unsigned char val;
	int ret;

	val = rtc_irq_bits | bit;
	ret = twl4030_rtc_write_u8(val, REG_RTC_INTERRUPTS_REG);
	if (ret == 0)
		rtc_irq_bits = val;

	return ret;
}

/*
 * Disable timer and/or alarm interrupts.
 */
static int mask_rtc_irq_bit(unsigned char bit)
{
	unsigned char val;
	int ret;

	val = rtc_irq_bits & ~bit;
	ret = twl4030_rtc_write_u8(val, REG_RTC_INTERRUPTS_REG);
	if (ret == 0)
		rtc_irq_bits = val;

	return ret;
}

static inline int twl4030_rtc_alarm_irq_set_state(int enabled)
{
	int ret;

	if (enabled)
		ret = set_rtc_irq_bit(BIT_RTC_INTERRUPTS_REG_IT_ALARM_M);
	else
		ret = mask_rtc_irq_bit(BIT_RTC_INTERRUPTS_REG_IT_ALARM_M);

	return ret;
}

static inline int twl4030_rtc_irq_set_state(int enabled)
{
	int ret;

	if (enabled)
		ret = set_rtc_irq_bit(BIT_RTC_INTERRUPTS_REG_IT_TIMER_M);
	else
		ret = mask_rtc_irq_bit(BIT_RTC_INTERRUPTS_REG_IT_TIMER_M);

	return ret;
}

/*
 * Gets current TWL4030 RTC time and date parameters.
 *
 * The RTC's time/alarm representation is not what gmtime(3) requires
 * Linux to use:
 *
 *  - Months are 1..12 vs Linux 0-11
 *  - Years are 0..99 vs Linux 1900..N (we assume 21st century)
 */
static int twl4030_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	unsigned char rtc_data[ALL_TIME_REGS + 1];
	int ret;
	u8 save_control;

	ret = twl4030_rtc_read_u8(&save_control, REG_RTC_CTRL_REG);
	if (ret < 0)
		return ret;

	save_control |= BIT_RTC_CTRL_REG_GET_TIME_M;

	ret = twl4030_rtc_write_u8(save_control, REG_RTC_CTRL_REG);
	if (ret < 0)
		return ret;

	ret = twl4030_i2c_read(TWL4030_MODULE_RTC, rtc_data,
			       REG_SECONDS_REG, ALL_TIME_REGS);

	if (ret < 0) {
		dev_err(dev, "rtc_read_time error %d\n", ret);
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

static int twl4030_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	unsigned char save_control;
	unsigned char rtc_data[ALL_TIME_REGS + 1];
	int ret;

	rtc_data[1] = bin2bcd(tm->tm_sec);
	rtc_data[2] = bin2bcd(tm->tm_min);
	rtc_data[3] = bin2bcd(tm->tm_hour);
	rtc_data[4] = bin2bcd(tm->tm_mday);
	rtc_data[5] = bin2bcd(tm->tm_mon + 1);
	rtc_data[6] = bin2bcd(tm->tm_year - 100);

	/* Stop RTC while updating the TC registers */
	ret = twl4030_rtc_read_u8(&save_control, REG_RTC_CTRL_REG);
	if (ret < 0)
		goto out;

	save_control &= ~BIT_RTC_CTRL_REG_STOP_RTC_M;
	twl4030_rtc_write_u8(save_control, REG_RTC_CTRL_REG);
	if (ret < 0)
		goto out;

	/* update all the time registers in one shot */
	ret = twl4030_i2c_write(TWL4030_MODULE_RTC, rtc_data,
			REG_SECONDS_REG, ALL_TIME_REGS);
	if (ret < 0) {
		dev_err(dev, "rtc_set_time error %d\n", ret);
		goto out;
	}

	/* Start back RTC */
	save_control |= BIT_RTC_CTRL_REG_STOP_RTC_M;
	ret = twl4030_rtc_write_u8(save_control, REG_RTC_CTRL_REG);

out:
	return ret;
}

/*
 * Gets current TWL4030 RTC alarm time.
 */
static int twl4030_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	unsigned char rtc_data[ALL_TIME_REGS + 1];
	int ret;

	ret = twl4030_i2c_read(TWL4030_MODULE_RTC, rtc_data,
			       REG_ALARM_SECONDS_REG, ALL_TIME_REGS);
	if (ret < 0) {
		dev_err(dev, "rtc_read_alarm error %d\n", ret);
		return ret;
	}

	/* some of these fields may be wildcard/"match all" */
	alm->time.tm_sec = bcd2bin(rtc_data[0]);
	alm->time.tm_min = bcd2bin(rtc_data[1]);
	alm->time.tm_hour = bcd2bin(rtc_data[2]);
	alm->time.tm_mday = bcd2bin(rtc_data[3]);
	alm->time.tm_mon = bcd2bin(rtc_data[4]) - 1;
	alm->time.tm_year = bcd2bin(rtc_data[5]) + 100;

	/* report cached alarm enable state */
	if (rtc_irq_bits & BIT_RTC_INTERRUPTS_REG_IT_ALARM_M)
		alm->enabled = 1;

	return ret;
}

static int twl4030_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	unsigned char alarm_data[ALL_TIME_REGS + 1];
	int ret;

	ret = twl4030_rtc_alarm_irq_set_state(0);
	if (ret)
		goto out;

	alarm_data[1] = bin2bcd(alm->time.tm_sec);
	alarm_data[2] = bin2bcd(alm->time.tm_min);
	alarm_data[3] = bin2bcd(alm->time.tm_hour);
	alarm_data[4] = bin2bcd(alm->time.tm_mday);
	alarm_data[5] = bin2bcd(alm->time.tm_mon + 1);
	alarm_data[6] = bin2bcd(alm->time.tm_year - 100);

	/* update all the alarm registers in one shot */
	ret = twl4030_i2c_write(TWL4030_MODULE_RTC, alarm_data,
			REG_ALARM_SECONDS_REG, ALL_TIME_REGS);
	if (ret) {
		dev_err(dev, "rtc_set_alarm error %d\n", ret);
		goto out;
	}

	if (alm->enabled)
		ret = twl4030_rtc_alarm_irq_set_state(1);
out:
	return ret;
}

#ifdef	CONFIG_RTC_INTF_DEV

static int twl4030_rtc_ioctl(struct device *dev, unsigned int cmd,
			     unsigned long arg)
{
	switch (cmd) {
	case RTC_AIE_OFF:
		return twl4030_rtc_alarm_irq_set_state(0);
	case RTC_AIE_ON:
		return twl4030_rtc_alarm_irq_set_state(1);
	case RTC_UIE_OFF:
		return twl4030_rtc_irq_set_state(0);
	case RTC_UIE_ON:
		return twl4030_rtc_irq_set_state(1);

	default:
		return -ENOIOCTLCMD;
	}
}

#else
#define	omap_rtc_ioctl	NULL
#endif

static irqreturn_t twl4030_rtc_interrupt(int irq, void *rtc)
{
	unsigned long events = 0;
	int ret = IRQ_NONE;
	int res;
	u8 rd_reg;

	res = twl4030_rtc_read_u8(&rd_reg, REG_RTC_STATUS_REG);
	if (res)
		goto out;
	/*
	 * Figure out source of interrupt: ALARM or TIMER in RTC_STATUS_REG.
	 * only one (ALARM or RTC) interrupt source may be enabled
	 * at time, we also could check our results
	 * by reading RTS_INTERRUPTS_REGISTER[IT_TIMER,IT_ALARM]
	 */
	if (rd_reg & BIT_RTC_STATUS_REG_ALARM_M)
		events |= RTC_IRQF | RTC_AF;
	else
		events |= RTC_IRQF | RTC_UF;

	res = twl4030_rtc_write_u8(rd_reg | BIT_RTC_STATUS_REG_ALARM_M,
				   REG_RTC_STATUS_REG);
	if (res)
		goto out;

	/* Clear on Read enabled. RTC_IT bit of REG_PWR_ISR1 needs
	 * 2 reads to clear the interrupt. One read is done in
	 * do_twl4030_pwrirq(). Doing the second read, to clear
	 * the bit.
	 */
	res = twl4030_i2c_read_u8(TWL4030_MODULE_INT, &rd_reg, REG_PWR_ISR1);
	if (res)
		goto out;

	/* Notify RTC core on event */
	rtc_update_irq(rtc, 1, events);

	ret = IRQ_HANDLED;
out:
	return ret;
}

static struct rtc_class_ops twl4030_rtc_ops = {
	.ioctl		= twl4030_rtc_ioctl,
	.read_time	= twl4030_rtc_read_time,
	.set_time	= twl4030_rtc_set_time,
	.read_alarm	= twl4030_rtc_read_alarm,
	.set_alarm	= twl4030_rtc_set_alarm,
};

static int __devinit twl4030_rtc_probe(struct platform_device *pdev)
{
	struct twl4030rtc_platform_data *pdata = pdev->dev.platform_data;
	struct rtc_device *rtc;
	int ret = 0;
	u8 rd_reg;

	if (pdata != NULL && pdata->init != NULL) {
		ret = pdata->init();
		if (ret < 0)
			goto out;
	}

	rtc = rtc_device_register(pdev->name,
				  &pdev->dev, &twl4030_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc)) {
		ret = -EINVAL;
		dev_err(&pdev->dev, "can't register RTC device, err %ld\n",
			PTR_ERR(rtc));
		goto out0;

	}

	platform_set_drvdata(pdev, rtc);

	ret = twl4030_rtc_read_u8(&rd_reg, REG_RTC_STATUS_REG);

	if (ret < 0)
		goto out1;

	if (rd_reg & BIT_RTC_STATUS_REG_POWER_UP_M)
		dev_warn(&pdev->dev, "Power up reset detected.\n");

	if (rd_reg & BIT_RTC_STATUS_REG_ALARM_M)
		dev_warn(&pdev->dev, "Pending Alarm interrupt detected.\n");

	/* Clear RTC Power up reset and pending alarm interrupts */
	ret = twl4030_rtc_write_u8(rd_reg, REG_RTC_STATUS_REG);
	if (ret < 0)
		goto out1;

	ret = request_irq(TWL4030_PWRIRQ_RTC, twl4030_rtc_interrupt,
				0, rtc->dev.bus_id, rtc);
	if (ret < 0) {
		dev_err(&pdev->dev, "IRQ is not free.\n");
		goto out1;
	}

	/* Check RTC module status, Enable if it is off */
	ret = twl4030_rtc_read_u8(&rd_reg, REG_RTC_CTRL_REG);
	if (ret < 0)
		goto out2;

	if (!(rd_reg & BIT_RTC_CTRL_REG_STOP_RTC_M)) {
		dev_info(&pdev->dev, "Enabling TWL4030-RTC.\n");
		rd_reg = BIT_RTC_CTRL_REG_STOP_RTC_M;
		ret = twl4030_rtc_write_u8(rd_reg, REG_RTC_CTRL_REG);
		if (ret < 0)
			goto out2;
	}

	ret = twl4030_i2c_read_u8(TWL4030_MODULE_INT, &rd_reg, REG_PWR_IMR1);
	if (ret < 0)
		goto out2;

	rd_reg &= PWR_RTC_IT_UNMASK;
	/* MASK PWR - we will need this */
	ret = twl4030_i2c_write_u8(TWL4030_MODULE_INT, rd_reg, REG_PWR_IMR1);
	if (ret < 0)
		goto out2;

	ret = twl4030_i2c_read_u8(TWL4030_MODULE_INT, &rd_reg, REG_PWR_EDR1);
	if (ret < 0)
		goto out2;

	/* Rising edge detection enabled, needed for RTC alarm */
	rd_reg |= 0x80;
	ret = twl4030_i2c_write_u8(TWL4030_MODULE_INT, rd_reg, REG_PWR_EDR1);
	if (ret < 0)
		goto out2;

	/* init cached IRQ enable bits */
	ret = twl4030_rtc_read_u8(&rtc_irq_bits, REG_RTC_INTERRUPTS_REG);
	if (ret < 0)
		goto out2;

	return ret;


out2:
	free_irq(TWL4030_MODIRQ_PWR, rtc);
out1:
	rtc_device_unregister(rtc);
out0:
	if (pdata != NULL && pdata->exit != NULL)
		pdata->exit();
out:
	return ret;
}

/*
 * Disable all TWL4030 RTC module interrupts.
 * Sets status flag to free.
 */
static int __devexit twl4030_rtc_remove(struct platform_device *pdev)
{
	/* leave rtc running, but disable irqs */
	struct twl4030rtc_platform_data *pdata = pdev->dev.platform_data;
	struct rtc_device *rtc = platform_get_drvdata(pdev);

	mask_rtc_irq_bit(BIT_RTC_INTERRUPTS_REG_IT_ALARM_M);
	mask_rtc_irq_bit(BIT_RTC_INTERRUPTS_REG_IT_TIMER_M);

	free_irq(TWL4030_MODIRQ_PWR, rtc);

	if (pdata != NULL && pdata->exit != NULL)
		pdata->exit();

	rtc_device_unregister(rtc);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static void twl4030_rtc_shutdown(struct platform_device *pdev)
{
	mask_rtc_irq_bit(BIT_RTC_INTERRUPTS_REG_IT_TIMER_M |
			 BIT_RTC_INTERRUPTS_REG_IT_ALARM_M);
}

#ifdef CONFIG_PM

static unsigned char irqstat;

static int twl4030_rtc_suspend(struct platform_device *pdev, pm_message_t state)
{
	irqstat = rtc_irq_bits;

	/* REVISIT alarm may need to wake us from sleep */
	mask_rtc_irq_bit(BIT_RTC_INTERRUPTS_REG_IT_TIMER_M |
			 BIT_RTC_INTERRUPTS_REG_IT_ALARM_M);
	return 0;
}

static int twl4030_rtc_resume(struct platform_device *pdev)
{
	set_rtc_irq_bit(irqstat);
	return 0;
}

#else
#define twl4030_rtc_suspend NULL
#define twl4030_rtc_resume  NULL
#endif

MODULE_ALIAS("platform:twl4030_rtc");

static struct platform_driver twl4030rtc_driver = {
	.probe		= twl4030_rtc_probe,
	.remove		= __devexit_p(twl4030_rtc_remove),
	.shutdown	= twl4030_rtc_shutdown,
	.suspend	= twl4030_rtc_suspend,
	.resume		= twl4030_rtc_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "twl4030_rtc",
	},
};

static int __init twl4030_rtc_init(void)
{
	return platform_driver_register(&twl4030rtc_driver);
}
module_init(twl4030_rtc_init);

static void __exit twl4030_rtc_exit(void)
{
	platform_driver_unregister(&twl4030rtc_driver);
}
module_exit(twl4030_rtc_exit);

MODULE_AUTHOR("Texas Instruments, MontaVista Software");
MODULE_LICENSE("GPL");
