/* SPDX-License-Identifier: GPL-2.0 */
/* Texas Instruments ICSSG Industrial Ethernet Peripheral (IEP) Driver
 *
 * Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/
 *
 */

#ifndef __NET_TI_ICSS_IEP_H
#define __NET_TI_ICSS_IEP_H

#include <linux/mutex.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/regmap.h>

#define IEP_GLOBAL_CFG_REG	0x0
#define IEP_GLOBAL_STATUS_REG	0x4
#define IEP_COMPEN_REG		0x8
#define IEP_SLOW_COMPEN_REG	0xc
#define IEP_COUNT_LOW_REG	0x10
#define IEP_COUNT_HIGH_REG	0x14
#define IEP_CAP_CFG_REG		0x18
#define IEP_CAP_STATUS_REG	0x1c
#define IEP_CMP_CFG_REG		0x70
#define IEP_CMP_STATUS_REG	0x74

struct icss_iep;

/* Firmware specific clock operations */
struct icss_iep_clockops {
	void (*settime)(struct icss_iep *iep, u64 ns);
	void (*adjtime)(struct icss_iep *iep, s64 delta);
	u64 (*gettime)(struct icss_iep *iep);
};

struct icss_iep {
	struct regmap *map;
	u32 refclk_freq;
	int clk_tick_time;	/* one refclk tick time in ns */
	struct ptp_clock_info ptp_info;
	struct ptp_clock *ptp_clock;
	int ptp_index;
	struct mutex ptp_clk_mutex;	/* PHC access serializer */
	unsigned int slow_cmp_active;
	u32 def_inc;
	s16 slow_cmp_inc;
	u32 slow_cmp_count;
	const struct icss_iep_clockops *ops;
	u32 cycle_time_ns;
};

int icss_iep_init(struct icss_iep *iep, struct device *parent_dev,
		  struct regmap *iep_map, u32 refclk_freq,
		  u32 cycle_time_ns);
int icss_iep_exit(struct icss_iep *iep);

#endif /* __NET_TI_ICSS_IEP_H */
