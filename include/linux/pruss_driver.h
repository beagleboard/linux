/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * PRU-ICSS sub-system specific definitions
 *
 * Copyright (C) 2014-2020 Texas Instruments Incorporated - http://www.ti.com/
 *	Suman Anna <s-anna@ti.com>
 */

#ifndef _PRUSS_DRIVER_H_
#define _PRUSS_DRIVER_H_

#include <linux/types.h>
#include <linux/pruss.h>
#include <linux/mutex.h>

/**
 * struct pruss - PRUSS parent structure
 * @dev: pruss device pointer
 * @cfg_base: base iomap for CFG region
 * @cfg: regmap for config region
 * @mem_regions: data for each of the PRUSS memory regions
 * @mem_in_use: to indicate if memory resource is in use
 * @lock: mutex to serialize access to resources
 * @core_clk_mux: clk handle for PRUSS CORE_CLK_MUX
 & @iep_clk_mux: clk handle for PRUSS IEP_CLK_MUX
 * @has_ocp_syscfg: flag to indicate if PRCM master standby/slave idle is
 *		    supported
 */
struct pruss {
	struct device *dev;
	void __iomem *cfg_base;
	struct regmap *cfg;
	struct pruss_mem_region mem_regions[PRUSS_MEM_MAX];
	struct pruss_mem_region *mem_in_use[PRUSS_MEM_MAX];
	struct mutex lock; /* PRU resource lock */
	struct clk *core_clk_mux;
	struct clk *iep_clk_mux;
	unsigned int has_ocp_syscfg : 1;
};

/**
 * pruss_cfg_get_gpmux() - get the current GPMUX value for a PRU device
 * @pruss: pruss instance
 * @id: PRU identifier (0-1)
 * @mux: pointer to store the current mux value into
 */
static inline int pruss_cfg_get_gpmux(struct pruss *pruss,
				      enum pruss_pru_id id, u8 *mux)
{
	int ret = 0;
	u32 val;

	ret = pruss_cfg_read(pruss, PRUSS_CFG_GPCFG(id), &val);
	if (!ret)
		*mux = (u8)((val & PRUSS_GPCFG_PRU_MUX_SEL_MASK) >>
			    PRUSS_GPCFG_PRU_MUX_SEL_SHIFT);
	return ret;
}

/**
 * pruss_cfg_set_gpmux() - set the GPMUX value for a PRU device
 * @pruss: pruss instance
 * @pru_id: PRU identifier (0-1)
 * @mux: new mux value for PRU
 */
static inline int pruss_cfg_set_gpmux(struct pruss *pruss,
				      enum pruss_pru_id id, u8 mux)
{
	if (mux >= PRUSS_GP_MUX_SEL_MAX)
		return -EINVAL;

	return pruss_cfg_update(pruss, PRUSS_CFG_GPCFG(id),
				PRUSS_GPCFG_PRU_MUX_SEL_MASK,
				(u32)mux << PRUSS_GPCFG_PRU_MUX_SEL_SHIFT);
}

#endif	/* _PRUSS_DRIVER_H_ */
