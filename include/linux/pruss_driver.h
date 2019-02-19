/* SPDX-License-Identifier: GPL-2.0 */
/*
 * PRU-ICSS sub-system specific definitions
 *
 * Copyright (C) 2014-2019 Texas Instruments Incorporated - http://www.ti.com/
 *	Suman Anna <s-anna@ti.com>
 */

#ifndef _PRUSS_DRIVER_H_
#define _PRUSS_DRIVER_H_

#include <linux/pruss.h>

/* maximum number of system events */
#define MAX_PRU_SYS_EVENTS	64

/* maximum number of interrupt channels */
#define MAX_PRU_CHANNELS	10

/* minimum starting host interrupt number for MPU */
#define MIN_PRU_HOST_INT	2

/* maximum number of host interrupts */
#define MAX_PRU_HOST_INT	10

/**
 * struct pruss_intc_config - INTC configuration info
 * @sysev_to_ch: system events to channel mapping information
 * @ch_to_host: interrupt channel to host interrupt information
 */
struct pruss_intc_config {
	s8 sysev_to_ch[MAX_PRU_SYS_EVENTS];
	s8 ch_to_host[MAX_PRU_CHANNELS];
};

/**
 * struct pruss - PRUSS parent structure
 * @dev: pruss device pointer
 * @cfg: regmap for config region
 * @iep: regmap for IEP sub-module
 * @mii_rt: regmap for MII_RT sub-module
 * @mem_regions: data for each of the PRUSS memory regions
 * @mem_in_use: to indicate if memory resource is in use
 * @lock: mutex to serialize access to resources
 */
struct pruss {
	struct device *dev;
	struct regmap *cfg;
	struct regmap *iep;
	struct regmap *mii_rt;
	struct pruss_mem_region mem_regions[PRUSS_MEM_MAX];
	struct pruss_mem_region *mem_in_use[PRUSS_MEM_MAX];
	struct mutex lock; /* PRU resource lock */
};

int pruss_intc_configure(struct pruss *pruss,
			 struct pruss_intc_config *intc_config);
int pruss_intc_unconfigure(struct pruss *pruss,
			   struct pruss_intc_config *intc_config);

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

	ret = pruss_regmap_read(pruss, PRUSS_SYSCON_CFG,
				PRUSS_CFG_GPCFG(id), &val);
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

	return pruss_regmap_update(pruss, PRUSS_SYSCON_CFG, PRUSS_CFG_GPCFG(id),
				   PRUSS_GPCFG_PRU_MUX_SEL_MASK,
				  (u32)mux << PRUSS_GPCFG_PRU_MUX_SEL_SHIFT);
}

#endif	/* _PRUSS_DRIVER_H_ */
