/* SPDX-License-Identifier: GPL-2.0-only */
/**
 * PRU-ICSS Subsystem user interfaces
 *
 * Copyright (C) 2015-2021 Texas Instruments Incorporated - https://www.ti.com
 *	Suman Anna <s-anna@ti.com>
 *	Tero Kristo <t-kristo@ti.com>
 */

#ifndef __LINUX_PRUSS_H
#define __LINUX_PRUSS_H

#include <linux/device.h>
#include <linux/err.h>
#include <linux/types.h>

#define PRU_RPROC_DRVNAME "pru-rproc"

/*
 * enum pruss_pru_id - PRU core identifiers
 */
enum pruss_pru_id {
	PRUSS_PRU0 = 0,
	PRUSS_PRU1,
	PRUSS_NUM_PRUS,
};

/*
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
	return ERR_PTR(-EOPNOTSUPP);
}

static inline void pruss_put(struct pruss *pruss) { }

#endif /* CONFIG_TI_PRUSS */

#if IS_ENABLED(CONFIG_PRU_REMOTEPROC)

struct rproc *pru_rproc_get(struct device_node *np, int index,
			    enum pruss_pru_id *pru_id);
void pru_rproc_put(struct rproc *rproc);
int pru_rproc_set_ctable(struct rproc *rproc, enum pru_ctable_idx c, u32 addr);

#else

static inline struct rproc *
pru_rproc_get(struct device_node *np, int index, enum pruss_pru_id *pru_id)
{
	return ERR_PTR(-EOPNOTSUPP);
}

static inline void pru_rproc_put(struct rproc *rproc) { }

static inline int pru_rproc_set_ctable(struct rproc *rproc,
				       enum pru_ctable_idx c, u32 addr)
{
	return -EOPNOTSUPP;
}

#endif /* CONFIG_PRU_REMOTEPROC */

static inline bool is_pru_rproc(struct device *dev)
{
	const char *drv_name = dev_driver_string(dev);

	if (strncmp(drv_name, PRU_RPROC_DRVNAME, sizeof(PRU_RPROC_DRVNAME)))
		return false;

	return true;
}

#endif /* __LINUX_PRUSS_H */
