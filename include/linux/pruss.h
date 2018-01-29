/* SPDX-License-Identifier: GPL-2.0 */
/**
 * PRU-ICSS Subsystem user interfaces
 *
 * Copyright (C) 2015-2018 Texas Instruments Incorporated - http://www.ti.com
 *	Suman Anna <s-anna@ti.com>
 */

#ifndef __LINUX_PRUSS_H
#define __LINUX_PRUSS_H

#include <linux/remoteproc.h>

/**
 * enum pruss_pru_id - PRU core identifiers
 */
enum pruss_pru_id {
	PRUSS_PRU0 = 0,
	PRUSS_PRU1,
	PRUSS_NUM_PRUS,
};

#if IS_ENABLED(CONFIG_PRUSS_REMOTEPROC)

int pruss_intc_trigger(unsigned int irq);

struct rproc *pru_rproc_get(struct device_node *node, int index);
void pru_rproc_put(struct rproc *rproc);
enum pruss_pru_id pru_rproc_get_id(struct rproc *rproc);

#else

static inline int pruss_intc_trigger(unsigned int irq)
{
	return -ENOTSUPP;
}

static inline struct rproc *pru_rproc_get(struct device_node *node, int index)
{
	return ERR_PTR(-ENOTSUPP);
}

static inline void pru_rproc_put(struct rproc *rproc) { }

static inline enum pruss_pru_id pru_rproc_get_id(struct rproc *rproc)
{
	return -ENOTSUPP;
}

#endif /* CONFIG_PRUSS_REMOTEPROC */

#endif /* __LINUX_PRUSS_H */
