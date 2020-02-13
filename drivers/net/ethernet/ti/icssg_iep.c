// SPDX-License-Identifier: GPL-2.0
/* Texas Instruments ICSSG Industrial Ethernet Peripheral (IEP) Driver
 *
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com
 *
 */

#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/timekeeping.h>

#include "icssg_iep.h"

#define IEP_MAX_DEF_INC		0xf
#define IEP_MAX_COMPEN_INC		0xfff
#define IEP_MAX_COMPEN_COUNT	0xffffff

#define IEP_GLOBAL_CFG_REG	0x0
#define IEP_GLOBAL_STATUS_REG	0x4
#define IEP_COMPEN_REG		0x8
#define IEP_SLOW_COMPEN_REG	0xc
#define IEP_COUNT_LOW_REG	0x10
#define IEP_COUNT_HIGH_REG	0x14
#define IEP_CAP_CFG_REG		0x18
#define IEP_CAP_STATUS_REG	0x1c

#define IEP_GLOBAL_CFG_CNT_ENABLE	BIT(0)
#define IEP_GLOBAL_CFG_DEFAULT_INC_MASK		GENMASK(7, 4)
#define IEP_GLOBAL_CFG_DEFAULT_INC_SHIFT	4
#define IEP_GLOBAL_CFG_COMPEN_INC_MASK		GENMASK(19, 8)
#define IEP_GLOBAL_CFG_COMPEN_INC_SHIFT		8

static void iep_settime(struct icssg_iep *iep, u64 tstamp)
{
	u32 val;

	val = upper_32_bits(tstamp);
	regmap_write(iep->map, IEP_COUNT_HIGH_REG, val);
	val = lower_32_bits(tstamp);
	regmap_write(iep->map, IEP_COUNT_LOW_REG, val);
}

static u64 iep_gettime(struct icssg_iep *iep)
{
	u64 val;
	u32 tmp;

	regmap_read(iep->map, IEP_COUNT_LOW_REG, &tmp);
	val = tmp;
	regmap_read(iep->map, IEP_COUNT_HIGH_REG, &tmp);
	val |= (u64)tmp << 32;

	return val;
}

static void iep_enable(struct icssg_iep *iep)
{
	regmap_update_bits(iep->map, IEP_GLOBAL_CFG_REG,
			   IEP_GLOBAL_CFG_CNT_ENABLE,
			   IEP_GLOBAL_CFG_CNT_ENABLE);
}

static void iep_disable(struct icssg_iep *iep)
{
	regmap_update_bits(iep->map, IEP_GLOBAL_CFG_REG,
			   IEP_GLOBAL_CFG_CNT_ENABLE,
			   0);
}

static void iep_set_default_inc(struct icssg_iep *iep, u8 def_inc)
{
	regmap_update_bits(iep->map, IEP_GLOBAL_CFG_REG,
			   IEP_GLOBAL_CFG_DEFAULT_INC_MASK,
			   def_inc << IEP_GLOBAL_CFG_DEFAULT_INC_SHIFT);
}

static void iep_set_compensation_inc(struct icssg_iep *iep, u16 compen_inc)
{
	struct device *dev = regmap_get_device(iep->map);

	if (compen_inc > IEP_MAX_COMPEN_INC) {
		dev_err(dev, "%s: too high compensation inc %d\n",
			__func__, compen_inc);
		compen_inc = IEP_MAX_COMPEN_INC;
	}

	regmap_update_bits(iep->map, IEP_GLOBAL_CFG_REG,
			   IEP_GLOBAL_CFG_COMPEN_INC_MASK,
			   compen_inc << IEP_GLOBAL_CFG_COMPEN_INC_SHIFT);
}

static void iep_set_compensation_count(struct icssg_iep *iep, u32 compen_count)
{
	struct device *dev = regmap_get_device(iep->map);

	if (compen_count > IEP_MAX_COMPEN_COUNT) {
		dev_err(dev, "%s: too high compensation count %d\n",
			__func__, compen_count);
		compen_count = IEP_MAX_COMPEN_COUNT;
	}

	regmap_write(iep->map, IEP_COMPEN_REG, compen_count);
}

static void iep_set_slow_compensation_count(struct icssg_iep *iep,
					    u32 compen_count)
{
	regmap_write(iep->map, IEP_SLOW_COMPEN_REG, compen_count);
}

/* PTP PHC operations */
static int iep_ptp_adjfreq(struct ptp_clock_info *ptp, s32 ppb)
{
	struct icssg_iep *iep = container_of(ptp, struct icssg_iep, ptp_info);
	u32 cyc_count;
	u16 cmp_inc;

	mutex_lock(&iep->ptp_clk_mutex);

	/* ppb is amount of frequency we want to adjust in 1GHz (billion)
	 * e.g. 100ppb means we need to speed up clock by 100Hz
	 * i.e. at end of 1 second (1 billion ns) clock time, we should be
	 * counting 100 more ns.
	 * We use IEP slow compensation to achieve continuous freq. adjustment.
	 * There are 2 parts. Cycle time and adjustment per cycle.
	 * Simplest case would be 1 sec Cycle time. Then adjustment
	 * pre cycle would be (def_inc + ppb) value.
	 * Cycle time will have to be chosen based on how worse the ppb is.
	 * e.g. smaller the ppb, cycle time has to be large.
	 * The minimum adjustment we can do is +-1ns per cycle so let's
	 * reduce the cycle time to get 1ns per cycle adjustment.
	 *	1ppb = 1sec cycle time & 1ns adjust
	 *	1000ppb = 1/1000 cycle time & 1ns adjust per cycle
	 */

	iep->slow_cmp_inc = 1;	/* 1ns adjust per cycle */
	if (ppb < 0) {
		iep->slow_cmp_inc = -iep->slow_cmp_inc;
		ppb = -ppb;
	}

	cyc_count = NSEC_PER_SEC;		/* 1s cycle time @1GHz */
	cyc_count /= ppb;		/* cycle time per ppb */

	/* slow_cmp_count is decremented every clock cycle, e.g. @250MHz */
	cyc_count /= iep->clk_tick_time;
	iep->slow_cmp_count = cyc_count;

	/* iep->clk_tick_time is def_inc */
	cmp_inc = iep->clk_tick_time + iep->slow_cmp_inc;
	iep_set_compensation_inc(iep, cmp_inc);
	iep_set_slow_compensation_count(iep, iep->slow_cmp_count);
	iep->slow_cmp_active = 1;

	mutex_unlock(&iep->ptp_clk_mutex);

	return 0;
}

static int iep_ptp_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
	struct icssg_iep *iep = container_of(ptp, struct icssg_iep, ptp_info);
	s64 ns;

	mutex_lock(&iep->ptp_clk_mutex);
	ns = iep_gettime(iep);
	ns += delta;
	iep_settime(iep, ns);
	mutex_unlock(&iep->ptp_clk_mutex);

	return 0;
}

static int iep_ptp_gettime(struct ptp_clock_info *ptp,
			   struct timespec64 *ts)
{
	struct icssg_iep *iep = container_of(ptp, struct icssg_iep, ptp_info);
	u64 ns;

	mutex_lock(&iep->ptp_clk_mutex);
	ns = iep_gettime(iep);
	*ts = ns_to_timespec64(ns);
	mutex_unlock(&iep->ptp_clk_mutex);

	return 0;
}

static int iep_ptp_settime(struct ptp_clock_info *ptp,
			   const struct timespec64 *ts)
{
	struct icssg_iep *iep = container_of(ptp, struct icssg_iep, ptp_info);
	u64 ns;

	mutex_lock(&iep->ptp_clk_mutex);
	ns = timespec64_to_ns(ts);
	iep_settime(iep, ns);
	mutex_unlock(&iep->ptp_clk_mutex);

	return 0;
}

static int iep_ptp_enable(struct ptp_clock_info *ptp,
			  struct ptp_clock_request *rq, int on)
{
	return -EOPNOTSUPP;
}

static struct ptp_clock_info iep_ptp_info = {
	.owner		= THIS_MODULE,
	.name		= "ICSS IEP timer",
	.max_adj	= 10000000,
	.adjfreq	= iep_ptp_adjfreq,
	.adjtime	= iep_ptp_adjtime,
	.gettime64	= iep_ptp_gettime,
	.settime64	= iep_ptp_settime,
	.enable		= iep_ptp_enable,
};

int icssg_iep_init(struct icssg_iep *iep, struct device *parent_dev,
		   struct regmap *iep_map, u32 refclk_freq)
{
	int ret;
	u32 def_inc;
	u64 tstamp;

	iep->map = iep_map;
	iep->refclk_freq = refclk_freq;
	mutex_init(&iep->ptp_clk_mutex);

	def_inc = NSEC_PER_SEC / iep->refclk_freq;	/* ns per clock tick */
	if (def_inc > IEP_MAX_DEF_INC)
		/* iep_core_clk too slow to be supported */
		return -EINVAL;

	iep_set_default_inc(iep, def_inc);
	iep_set_compensation_inc(iep, def_inc);
	iep_set_compensation_count(iep, 0);
	iep_set_slow_compensation_count(iep, 0);
	iep_enable(iep);

	tstamp = ktime_to_ns(ktime_get_real());
	iep_settime(iep, tstamp);

	iep->clk_tick_time = def_inc;
	iep->ptp_info = iep_ptp_info;
	iep->ptp_clock = ptp_clock_register(&iep->ptp_info, parent_dev);
	if (IS_ERR(iep->ptp_clock)) {
		ret = PTR_ERR(iep->ptp_clock);
		iep->ptp_clock = NULL;
		dev_err(parent_dev, "Failed to register ptp clk %d\n", ret);
		goto err_disable;
	}
	iep->ptp_index = ptp_clock_index(iep->ptp_clock);

	return 0;

err_disable:
	iep_disable(iep);

	return ret;
}

int icssg_iep_exit(struct icssg_iep *iep)
{
	ptp_clock_unregister(iep->ptp_clock);
	iep_disable(iep);

	return 0;
}
