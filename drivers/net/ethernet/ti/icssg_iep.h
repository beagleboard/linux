/* SPDX-License-Identifier: GPL-2.0 */
/* Texas Instruments ICSSG Industrial Ethernet Peripheral (IEP) Driver
 *
 * Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/
 *
 */

#ifndef __NET_TI_ICSSG_IEP_H
#define __NET_TI_ICSSG_IEP_H

#include <linux/mutex.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/regmap.h>

struct icssg_iep {
	struct regmap *map;
	u32 refclk_freq;
	int clk_tick_time;	/* one refclk tick time in ns */
	struct ptp_clock_info ptp_info;
	struct ptp_clock *ptp_clock;
	int ptp_index;
	struct mutex ptp_clk_mutex;	/* PHC access serializer */
	unsigned int slow_cmp_active;
	s16 slow_cmp_inc;
	u32 slow_cmp_count;
};

int icssg_iep_init(struct icssg_iep *iep, struct device *parent_dev,
		   struct regmap *iep_map, u32 refclk_freq);
int icssg_iep_exit(struct icssg_iep *iep);

#endif /* __NET_TI_ICSSG_IEP_H */
