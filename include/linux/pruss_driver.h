/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * PRU-ICSS sub-system specific definitions
 *
 * Copyright (C) 2014-2021 Texas Instruments Incorporated - https://www.ti.com/
 *	Suman Anna <s-anna@ti.com>
 */

#ifndef _PRUSS_DRIVER_H_
#define _PRUSS_DRIVER_H_

#include <linux/mutex.h>
#include <linux/pruss.h>
#include <linux/types.h>

/**
 * struct pruss - PRUSS parent structure
 * @dev: pruss device pointer
 * @cfg_base: base iomap for CFG region
 * @cfg_regmap: regmap for config region
 * @mem_regions: data for each of the PRUSS memory regions
 * @mem_in_use: to indicate if memory resource is in use
 * @lock: mutex to serialize access to resources
 * @core_clk_mux: clk handle for PRUSS CORE_CLK_MUX
 * @iep_clk_mux: clk handle for PRUSS IEP_CLK_MUX
 */
struct pruss {
	struct device *dev;
	void __iomem *cfg_base;
	struct regmap *cfg_regmap;
	struct pruss_mem_region mem_regions[PRUSS_MEM_MAX];
	struct pruss_mem_region *mem_in_use[PRUSS_MEM_MAX];
	struct mutex lock; /* PRU resource lock */
	struct clk *core_clk_mux;
	struct clk *iep_clk_mux;
};

#endif	/* _PRUSS_DRIVER_H_ */
