/*
 * PRU-ICSS sub-system specific definitions
 *
 * Copyright (C) 2014-2016 Texas Instruments Incorporated - http://www.ti.com/
 *	Suman Anna <s-anna@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _PRUSS_H_
#define _PRUSS_H_

/* number of PRUs within a PRUSS */
#ifndef PRUSS_NUM_PRUS
#define PRUSS_NUM_PRUS		2
#endif

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

/*
 * PRU_ICSS_CFG registers
 * SYSCFG, ISRP, ISP, IESP, IECP, SCRP applicable on AMxxxx devices only
 */
#define PRUSS_CFG_REVID		0x00
#define PRUSS_CFG_SYSCFG	0x04
#define PRUSS_CFG_GPCFG0	0x08
#define PRUSS_CFG_GPCFG1	0x0C
#define PRUSS_CFG_CGR		0x10
#define PRUSS_CFG_ISRP		0x14
#define PRUSS_CFG_ISP		0x18
#define PRUSS_CFG_IESP		0x1C
#define PRUSS_CFG_IECP		0x20
#define PRUSS_CFG_SCRP		0x24
#define PRUSS_CFG_PMAO		0x28
#define PRUSS_CFG_MII_RT	0x2C
#define PRUSS_CFG_IEPCLK	0x30
#define PRUSS_CFG_SPP		0x34
#define PRUSS_CFG_PIN_MX	0x40

/* PRUSS_SYSCFG register bits */
#define PRUSS_SYSCFG_SUB_MWAIT_READY	BIT(5)
#define PRUSS_SYSCFG_STANDBY_INIT	BIT(4)

#define PRUSS_SYSCFG_STANDBY_MODE_FORCE	(0 << 2)
#define PRUSS_SYSCFG_STANDBY_MODE_NO	(1 << 2)
#define PRUSS_SYSCFG_STANDBY_MODE_SMART	(2 << 2)
#define PRUSS_SYSCFG_STANDBY_MODE_MASK	(3 << 2)

#define PRUSS_SYSCFG_IDLE_MODE_FORCE	0
#define PRUSS_SYSCFG_IDLE_MODE_NO	1
#define PRUSS_SYSCFG_IDLE_MODE_SMART	2
#define PRUSS_SYSCFG_IDLE_MODE_MASK	3

/* PRUSS_GPCFG register bits */
#define PRUSS_GPCFG_PRU_GPO_SH_SEL		BIT(25)

#define PRUSS_GPCFG_PRU_DIV1_SHIFT		20
#define PRUSS_GPCFG_PRU_DIV1_MASK		GENMASK(24, 20)

#define PRUSS_GPCFG_PRU_DIV0_SHIFT		15
#define PRUSS_GPCFG_PRU_DIV0_MASK		GENMASK(15, 19)

#define PRUSS_GPCFG_PRU_GPO_MODE		BIT(14)
#define PRUSS_GPCFG_PRU_GPO_MODE_DIRECT		0
#define PRUSS_GPCFG_PRU_GPO_MODE_SERIAL		BIT(14)

#define PRUSS_GPCFG_PRU_GPI_SB			BIT(13)

#define PRUSS_GPCFG_PRU_GPI_DIV1_SHIFT		8
#define PRUSS_GPCFG_PRU_GPI_DIV1_MASK		GENMASK(12, 8)

#define PRUSS_GPCFG_PRU_GPI_DIV0_SHIFT		3
#define PRUSS_GPCFG_PRU_GPI_DIV0_MASK		GENMASK(7, 3)

#define PRUSS_GPCFG_PRU_GPI_CLK_MODE_POSITIVE	0
#define PRUSS_GPCFG_PRU_GPI_CLK_MODE_NEGATIVE	BIT(2)
#define PRUSS_GPCFG_PRU_GPI_CLK_MODE		BIT(2)

#define PRUSS_GPCFG_PRU_GPI_MODE_MASK		GENMASK(1, 0)
#define PRUSS_GPCFG_PRU_GPI_MODE_SHIFT		0

#define PRUSS_GPCFG_PRU_MUX_SEL_SHIFT		26
#define PRUSS_GPCFG_PRU_MUX_SEL_MASK		GENMASK(29, 26)

/* PRUSS_MII_RT register bits */
#define PRUSS_MII_RT_EVENT_EN			BIT(0)

/* PRUSS_SPP register bits */
#define PRUSS_SPP_XFER_SHIFT_EN			BIT(1)
#define PRUSS_SPP_PRU1_PAD_HP_EN		BIT(0)

/**
 * enum pruss_gp_mux_sel - PRUSS GPI/O Mux modes for the
 * PRUSS_GPCFG0/1 registers
 */
enum pruss_gp_mux_sel {
	PRUSS_GP_MUX_SEL_GP = 0,
	PRUSS_GP_MUX_SEL_ENDAT,
	PRUSS_GP_MUX_SEL_RESERVED,
	PRUSS_GP_MUX_SEL_SD,
	PRUSS_GP_MUX_SEL_MII2,
	PRUSS_GP_MUX_MAX,
};

/**
 * struct pruss_intc_config - INTC configuration info
 * @sysev_to_ch: system events to channel mapping information
 * @ch_to_host: interrupt channel to host interrupt information
 */
struct pruss_intc_config {
	int sysev_to_ch[MAX_PRU_SYS_EVENTS];
	int ch_to_host[MAX_PRU_CHANNELS];
};

struct pru_rproc;

/**
 * struct pruss - PRUSS parent structure
 * @node: list node of this object
 * @dev: pruss device pointer
 * @mem_regions: data for each of the PRUSS memory regions
 * @mem_in_use: to indicate if memory resource is in use
 * @data: pointer to store PRUSS instance private data
 * @host_mask: indicate which HOST IRQs are enabled
 * @pru_running: flag to indicate if PRU is running
 * @pru_in_use: flag to indicate if PRU is used
 * @lock: mutex to serialize access to resources
 * @cfg_lock: mutex to serialize access to CFG
 * @in_standby: flag for storing standby status
 */
struct pruss {
	struct list_head node;
	struct device *dev;
	struct pruss_mem_region mem_regions[PRUSS_MEM_MAX];
	struct pruss_mem_region *mem_in_use[PRUSS_MEM_MAX];
	const struct pruss_private_data *data;
	u32 host_mask;
	bool pru_running[PRUSS_NUM_PRUS];
	struct rproc *pru_in_use[PRUSS_NUM_PRUS];
	struct mutex lock; /* PRU resource lock */
	struct mutex cfg_lock; /* PRUSS CFG register access lock */
	bool in_standby;
};

int pruss_intc_configure(struct pruss *pruss,
			 struct pruss_intc_config *intc_config);
int pruss_intc_unconfigure(struct pruss *pruss,
			   struct pruss_intc_config *intc_config);
int pruss_cfg_set_gpmux(struct pruss *pruss, enum pruss_pru_id pru_id,
			enum pruss_gp_mux_sel mux_sel);

#endif	/* _PRUSS_H_ */
