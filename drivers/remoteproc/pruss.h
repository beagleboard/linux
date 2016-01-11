/* SPDX-License-Identifier: GPL-2.0 */
/*
 * PRU-ICSS sub-system specific definitions
 *
 * Copyright (C) 2014-2018 Texas Instruments Incorporated - http://www.ti.com/
 *	Suman Anna <s-anna@ti.com>
 */

#ifndef _PRUSS_H_
#define _PRUSS_H_

#include <linux/pruss.h>

/* maximum number of system events */
#define MAX_PRU_SYS_EVENTS	64

/* maximum number of interrupt channels */
#define MAX_PRU_CHANNELS	10

/* minimum starting host interrupt number for MPU */
#define MIN_PRU_HOST_INT	2

/* maximum number of host interrupts */
#define MAX_PRU_HOST_INT	10

/* PRU_ICSS_INTC registers */
#define PRU_INTC_REVID		0x0000
#define PRU_INTC_CR		0x0004
#define PRU_INTC_GER		0x0010
#define PRU_INTC_GNLR		0x001C
#define PRU_INTC_SISR		0x0020
#define PRU_INTC_SICR		0x0024
#define PRU_INTC_EISR		0x0028
#define PRU_INTC_EICR		0x002C
#define PRU_INTC_HIEISR		0x0034
#define PRU_INTC_HIDISR		0x0038
#define PRU_INTC_GPIR		0x0080
#define PRU_INTC_SRSR0		0x0200
#define PRU_INTC_SRSR1		0x0204
#define PRU_INTC_SECR0		0x0280
#define PRU_INTC_SECR1		0x0284
#define PRU_INTC_ESR0		0x0300
#define PRU_INTC_ESR1		0x0304
#define PRU_INTC_ECR0		0x0380
#define PRU_INTC_ECR1		0x0384
#define PRU_INTC_CMR(x)		(0x0400 + (x) * 4)
#define PRU_INTC_HMR(x)		(0x0800 + (x) * 4)
#define PRU_INTC_HIPIR(x)	(0x0900 + (x) * 4)
#define PRU_INTC_SIPR0		0x0D00
#define PRU_INTC_SIPR1		0x0D04
#define PRU_INTC_SITR0		0x0D80
#define PRU_INTC_SITR1		0x0D84
#define PRU_INTC_HINLR(x)	(0x1100 + (x) * 4)
#define PRU_INTC_HIER		0x1500

/* HIPIR register bit-fields */
#define INTC_HIPIR_NONE_HINT	0x80000000

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
 * @host_mask: indicate which HOST IRQs are enabled
 * @lock: mutex to serialize access to resources
 */
struct pruss {
	struct device *dev;
	struct regmap *cfg;
	struct regmap *iep;
	struct regmap *mii_rt;
	struct pruss_mem_region mem_regions[PRUSS_MEM_MAX];
	struct pruss_mem_region *mem_in_use[PRUSS_MEM_MAX];
	u32 host_mask;
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

#endif	/* _PRUSS_H_ */
