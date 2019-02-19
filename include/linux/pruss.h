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

#include <linux/err.h>
#include <linux/types.h>

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

struct device_node;
struct rproc;
struct pruss;

#if IS_ENABLED(CONFIG_TI_PRUSS)

struct pruss *pruss_get(struct rproc *rproc);
void pruss_put(struct pruss *pruss);

#else

static inline struct pruss *pruss_get(struct rproc *rproc)
{
	return ERR_PTR(-ENOTSUPP);
}

static inline void pruss_put(struct pruss *pruss) { }

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
