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

#include "icss_iep.h"

#define IEP_MAX_DEF_INC		0xf
#define IEP_MAX_COMPEN_INC		0xfff
#define IEP_MAX_COMPEN_COUNT	0xffffff

#define IEP_CMP_REG0(n)		(0x78 + (n) * 8)
#define IEP_CMP_REG1(n)		(0x7c + (n) * 8)

#define IEP_GLOBAL_CFG_CNT_ENABLE	BIT(0)
#define IEP_GLOBAL_CFG_DEFAULT_INC_MASK		GENMASK(7, 4)
#define IEP_GLOBAL_CFG_DEFAULT_INC_SHIFT	4
#define IEP_GLOBAL_CFG_COMPEN_INC_MASK		GENMASK(19, 8)
#define IEP_GLOBAL_CFG_COMPEN_INC_SHIFT		8

#define IEP_GLOBAL_STATUS_CNT_OVF	BIT(0)

#define IEP_CMP_CFG_SHADOW_EN		BIT(17)
#define IEP_CMP_CFG_CMP0_RST_CNT_EN	BIT(0)
#define IEP_CMP_CFG_CMP_EN(cmp)		(1 << ((cmp) + 1))

#define IEP_CMP_STATUS(cmp)		(1 << (cmp))

#define IEP_MIN_CMP	0
#define IEP_MAX_CMP	15

static void icss_iep_settime(struct icss_iep *iep, u64 ns)
{
	u32 val;

	if (iep->ops && iep->ops->settime) {
		iep->ops->settime(iep, ns);
		return;
	}

	val = upper_32_bits(ns);
	regmap_write(iep->map, IEP_COUNT_HIGH_REG, val);
	val = lower_32_bits(ns);
	regmap_write(iep->map, IEP_COUNT_LOW_REG, val);
}

static u64 icss_iep_gettime(struct icss_iep *iep)
{
	u64 val;
	u32 tmp;

	if (iep->ops && iep->ops->gettime)
		return iep->ops->gettime(iep);

	regmap_read(iep->map, IEP_COUNT_LOW_REG, &tmp);
	val = tmp;
	regmap_read(iep->map, IEP_COUNT_HIGH_REG, &tmp);
	val |= (u64)tmp << 32;

	return val;
}

static void icss_iep_enable(struct icss_iep *iep)
{
	regmap_update_bits(iep->map, IEP_GLOBAL_CFG_REG,
			   IEP_GLOBAL_CFG_CNT_ENABLE,
			   IEP_GLOBAL_CFG_CNT_ENABLE);
}

static void icss_iep_disable(struct icss_iep *iep)
{
	regmap_update_bits(iep->map, IEP_GLOBAL_CFG_REG,
			   IEP_GLOBAL_CFG_CNT_ENABLE,
			   0);
}

static void icss_iep_enable_shadow_mode(struct icss_iep *iep, u32 cycle_time_ns)
{
	u32 cycle_time;
	int cmp;

	/* FIXME: check why we need to decrement by def_inc */
	cycle_time = cycle_time_ns - iep->def_inc;

	icss_iep_disable(iep);

	/* disable shadow mode */
	regmap_update_bits(iep->map, IEP_CMP_CFG_REG,
			   IEP_CMP_CFG_SHADOW_EN, 0);

	/* enable shadow mode */
	regmap_update_bits(iep->map, IEP_CMP_CFG_REG,
			   IEP_CMP_CFG_SHADOW_EN, IEP_CMP_CFG_SHADOW_EN);

	/* clear counters */
	regmap_write(iep->map, IEP_COUNT_HIGH_REG, 0);
	regmap_write(iep->map, IEP_COUNT_LOW_REG, 0);

	/* clear overflow status */
	regmap_update_bits(iep->map, IEP_GLOBAL_STATUS_REG,
			   IEP_GLOBAL_STATUS_CNT_OVF,
			   IEP_GLOBAL_STATUS_CNT_OVF);

	/* clear compare status */
	for (cmp = IEP_MIN_CMP; cmp < IEP_MAX_CMP; cmp++) {
		regmap_update_bits(iep->map, IEP_CMP_STATUS_REG,
				   IEP_CMP_STATUS(cmp), IEP_CMP_STATUS(cmp));
	}

	/* enable reset counter on CMP0 event */
	regmap_update_bits(iep->map, IEP_CMP_CFG_REG,
			   IEP_CMP_CFG_CMP0_RST_CNT_EN,
			   IEP_CMP_CFG_CMP0_RST_CNT_EN);
	/* enable compare */
	regmap_update_bits(iep->map, IEP_CMP_CFG_REG,
			   IEP_CMP_CFG_CMP_EN(0),
			   IEP_CMP_CFG_CMP_EN(0));

	/* set CMP0 value to cycle time */
	regmap_write(iep->map, IEP_CMP_REG0(0), cycle_time);
	regmap_write(iep->map, IEP_CMP_REG1(0), cycle_time);

	icss_iep_enable(iep);
}

static void icss_iep_set_default_inc(struct icss_iep *iep, u8 def_inc)
{
	regmap_update_bits(iep->map, IEP_GLOBAL_CFG_REG,
			   IEP_GLOBAL_CFG_DEFAULT_INC_MASK,
			   def_inc << IEP_GLOBAL_CFG_DEFAULT_INC_SHIFT);
}

static void icss_iep_set_compensation_inc(struct icss_iep *iep, u16 compen_inc)
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

static void icss_iep_set_compensation_count(struct icss_iep *iep,
					    u32 compen_count)
{
	struct device *dev = regmap_get_device(iep->map);

	if (compen_count > IEP_MAX_COMPEN_COUNT) {
		dev_err(dev, "%s: too high compensation count %d\n",
			__func__, compen_count);
		compen_count = IEP_MAX_COMPEN_COUNT;
	}

	regmap_write(iep->map, IEP_COMPEN_REG, compen_count);
}

static void icss_iep_set_slow_compensation_count(struct icss_iep *iep,
						 u32 compen_count)
{
	regmap_write(iep->map, IEP_SLOW_COMPEN_REG, compen_count);
}

/* PTP PHC operations */
static int icss_iep_ptp_adjfreq(struct ptp_clock_info *ptp, s32 ppb)
{
	struct icss_iep *iep = container_of(ptp, struct icss_iep, ptp_info);
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

	if (iep->cycle_time_ns)
		iep->slow_cmp_inc = iep->clk_tick_time;	/* 4ns adj per cycle */
	else
		iep->slow_cmp_inc = 1;	/* 1ns adjust per cycle */

	if (ppb < 0) {
		iep->slow_cmp_inc = -iep->slow_cmp_inc;
		ppb = -ppb;
	}

	cyc_count = NSEC_PER_SEC;		/* 1s cycle time @1GHz */
	cyc_count /= ppb;		/* cycle time per ppb */

	/* slow_cmp_count is decremented every clock cycle, e.g. @250MHz */
	if (!iep->cycle_time_ns)
		cyc_count /= iep->clk_tick_time;
	iep->slow_cmp_count = cyc_count;

	/* iep->clk_tick_time is def_inc */
	cmp_inc = iep->clk_tick_time + iep->slow_cmp_inc;
	icss_iep_set_compensation_inc(iep, cmp_inc);
	icss_iep_set_slow_compensation_count(iep, iep->slow_cmp_count);
	iep->slow_cmp_active = 1;

	mutex_unlock(&iep->ptp_clk_mutex);

	return 0;
}

static int icss_iep_ptp_adjtime(struct ptp_clock_info *ptp, s64 delta)
{
	struct icss_iep *iep = container_of(ptp, struct icss_iep, ptp_info);
	s64 ns;

	mutex_lock(&iep->ptp_clk_mutex);
	if (iep->ops && iep->ops->adjtime) {
		iep->ops->adjtime(iep, delta);
	} else {
		ns = icss_iep_gettime(iep);
		ns += delta;
		icss_iep_settime(iep, ns);
	}
	mutex_unlock(&iep->ptp_clk_mutex);

	return 0;
}

static int icss_iep_ptp_gettime(struct ptp_clock_info *ptp,
				struct timespec64 *ts)
{
	struct icss_iep *iep = container_of(ptp, struct icss_iep, ptp_info);
	u64 ns;

	mutex_lock(&iep->ptp_clk_mutex);
	ns = icss_iep_gettime(iep);
	*ts = ns_to_timespec64(ns);
	mutex_unlock(&iep->ptp_clk_mutex);

	return 0;
}

static int icss_iep_ptp_settime(struct ptp_clock_info *ptp,
				const struct timespec64 *ts)
{
	struct icss_iep *iep = container_of(ptp, struct icss_iep, ptp_info);
	u64 ns;

	mutex_lock(&iep->ptp_clk_mutex);
	ns = timespec64_to_ns(ts);
	icss_iep_settime(iep, ns);
	mutex_unlock(&iep->ptp_clk_mutex);

	return 0;
}

static int icss_iep_ptp_enable(struct ptp_clock_info *ptp,
			       struct ptp_clock_request *rq, int on)
{
	return -EOPNOTSUPP;
}

static struct ptp_clock_info icss_iep_ptp_info = {
	.owner		= THIS_MODULE,
	.name		= "ICSS IEP timer",
	.max_adj	= 10000000,
	.adjfreq	= icss_iep_ptp_adjfreq,
	.adjtime	= icss_iep_ptp_adjtime,
	.gettime64	= icss_iep_ptp_gettime,
	.settime64	= icss_iep_ptp_settime,
	.enable		= icss_iep_ptp_enable,
};

int icss_iep_init(struct icss_iep *iep, struct device *parent_dev,
		  struct regmap *iep_map, u32 refclk_freq,
		  u32 cycle_time_ns)
{
	int ret;
	u32 def_inc;

	iep->map = iep_map;
	iep->refclk_freq = refclk_freq;
	mutex_init(&iep->ptp_clk_mutex);

	def_inc = NSEC_PER_SEC / iep->refclk_freq;	/* ns per clock tick */
	if (def_inc > IEP_MAX_DEF_INC)
		/* iep_core_clk too slow to be supported */
		return -EINVAL;

	iep->def_inc = def_inc;
	icss_iep_set_default_inc(iep, def_inc);
	icss_iep_set_compensation_inc(iep, def_inc);
	icss_iep_set_compensation_count(iep, 0);
	icss_iep_set_slow_compensation_count(iep, 0);
	if (cycle_time_ns)
		icss_iep_enable_shadow_mode(iep, cycle_time_ns);
	else
		icss_iep_enable(iep);

	iep->cycle_time_ns = cycle_time_ns;
	regmap_write(iep->map, IEP_COUNT_HIGH_REG, 0);
	regmap_write(iep->map, IEP_COUNT_LOW_REG, 0);

	iep->clk_tick_time = def_inc;
	iep->ptp_info = icss_iep_ptp_info;

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
	icss_iep_disable(iep);

	return ret;
}

int icss_iep_exit(struct icss_iep *iep)
{
	if (iep->ptp_clock)
		ptp_clock_unregister(iep->ptp_clock);
	icss_iep_disable(iep);

	return 0;
}
