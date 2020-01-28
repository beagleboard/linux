/* SPDX-License-Identifier: GPL-2.0-only */
/**
 * PRU-ICSS Subsystem user interfaces
 *
 * Copyright (C) 2015-2020 Texas Instruments Incorporated - http://www.ti.com
 *	Suman Anna <s-anna@ti.com>
 *	Tero Kristo <t-kristo@ti.com>
 */

#ifndef __LINUX_PRUSS_H
#define __LINUX_PRUSS_H

#include <linux/bits.h>
#include <linux/err.h>
#include <linux/types.h>

/*
 * PRU_ICSS_CFG registers
 * SYSCFG, ISRP, ISP, IESP, IECP, SCRP applicable on AMxxxx devices only
 */
#define PRUSS_CFG_REVID		0x00
#define PRUSS_CFG_SYSCFG	0x04
#define PRUSS_CFG_GPCFG(x)	(0x08 + (x) * 4)
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
 *
 * NOTE: The below defines are the most common values, but there
 * are some exceptions like on 66AK2G, where the RESERVED and MII2
 * values are interchanged. Also, this bit-field does not exist on
 * AM335x SoCs
 */
enum pruss_gp_mux_sel {
	PRUSS_GP_MUX_SEL_GP = 0,
	PRUSS_GP_MUX_SEL_ENDAT,
	PRUSS_GP_MUX_SEL_RESERVED,
	PRUSS_GP_MUX_SEL_SD,
	PRUSS_GP_MUX_SEL_MII2,
	PRUSS_GP_MUX_SEL_MAX,
};

/**
 * enum pruss_gpi_mode - PRUSS GPI configuration modes, used
 *			 to program the PRUSS_GPCFG0/1 registers
 */
enum pruss_gpi_mode {
	PRUSS_GPI_MODE_DIRECT = 0,
	PRUSS_GPI_MODE_PARALLEL,
	PRUSS_GPI_MODE_28BIT_SHIFT,
	PRUSS_GPI_MODE_MII,
};

/**
 * enum pruss_pru_id - PRU core identifiers
 */
enum pruss_pru_id {
	PRUSS_PRU0 = 0,
	PRUSS_PRU1,
	PRUSS_NUM_PRUS,
};

/**
 * enum pru_ctable_idx - Configurable Constant table index identifiers
 */
enum pru_ctable_idx {
	PRU_C24 = 0,
	PRU_C25,
	PRU_C26,
	PRU_C27,
	PRU_C28,
	PRU_C29,
	PRU_C30,
	PRU_C31,
};

/**
 * enum pruss_mem - PRUSS memory range identifiers
 */
enum pruss_mem {
	PRUSS_MEM_DRAM0 = 0,
	PRUSS_MEM_DRAM1,
	PRUSS_MEM_SHRD_RAM2,
	PRUSS_MEM_MAX,
};

/**
 * struct pruss_mem_region - PRUSS memory region structure
 * @va: kernel virtual address of the PRUSS memory region
 * @pa: physical (bus) address of the PRUSS memory region
 * @size: size of the PRUSS memory region
 */
struct pruss_mem_region {
	void __iomem *va;
	phys_addr_t pa;
	size_t size;
};

struct device_node;
struct rproc;
struct pruss;

#if IS_ENABLED(CONFIG_TI_PRUSS)

struct pruss *pruss_get(struct rproc *rproc);
void pruss_put(struct pruss *pruss);
int pruss_request_mem_region(struct pruss *pruss, enum pruss_mem mem_id,
			     struct pruss_mem_region *region);
int pruss_release_mem_region(struct pruss *pruss,
			     struct pruss_mem_region *region);
int pruss_cfg_read(struct pruss *pruss, unsigned int reg, unsigned int *val);
int pruss_cfg_update(struct pruss *pruss, unsigned int reg,
		     unsigned int mask, unsigned int val);

#else

static inline struct pruss *pruss_get(struct rproc *rproc)
{
	return ERR_PTR(-ENOTSUPP);
}

static inline void pruss_put(struct pruss *pruss) { }

static inline int pruss_request_mem_region(struct pruss *pruss,
					   enum pruss_mem mem_id,
					   struct pruss_mem_region *region)
{
	return -ENOTSUPP;
}

static inline int pruss_release_mem_region(struct pruss *pruss,
					   struct pruss_mem_region *region)
{
	return -ENOTSUPP;
}

static inline int pruss_cfg_read(struct pruss *pruss, unsigned int reg,
				 unsigned int *val)
{
	return -ENOTSUPP;
}

static inline int pruss_cfg_update(struct pruss *pruss, unsigned int reg,
				   unsigned int mask, unsigned int val)
{
	return -ENOTSUPP;
}

#endif /* CONFIG_TI_PRUSS */

#if IS_ENABLED(CONFIG_PRU_REMOTEPROC)

struct rproc *pru_rproc_get(struct device_node *node, int index);
void pru_rproc_put(struct rproc *rproc);
enum pruss_pru_id pru_rproc_get_id(struct rproc *rproc);
int pru_rproc_set_ctable(struct rproc *rproc, enum pru_ctable_idx c, u32 addr);

#else

static inline struct rproc *pru_rproc_get(struct device_node *node, int index)
{
	return ERR_PTR(-ENOTSUPP);
}

static inline void pru_rproc_put(struct rproc *rproc) { }

static inline enum pruss_pru_id pru_rproc_get_id(struct rproc *rproc)
{
	return -ENOTSUPP;
}

static inline int pru_rproc_set_ctable(struct rproc *rproc,
				       enum pru_ctable_idx c, u32 addr)
{
	return -ENOTSUPP;
}

#endif /* CONFIG_PRU_REMOTEPROC */

#endif /* __LINUX_PRUSS_H */
