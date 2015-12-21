/**
 * PRU-ICSS Subsystem user interfaces
 *
 * Copyright (C) 2015-2016 Texas Instruments Incorporated - http://www.ti.com
 *	Roger Quadros <rogerq@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __LINUX_PRUSS_H
#define __LINUX_PRUSS_H

/**
 * enum pruss_pru_id - PRU core identifiers
 */
enum pruss_pru_id {
	PRUSS_PRU0 = 0,
	PRUSS_PRU1,
	PRUSS_NUM_PRUS,
};

struct pruss;

#if IS_ENABLED(CONFIG_PRUSS_REMOTEPROC)

struct pruss *pruss_get(struct device *dev);
void pruss_put(struct pruss *pruss);
struct rproc *pruss_rproc_get(struct pruss *pruss,
			      enum pruss_pru_id pru_id);
void pruss_rproc_put(struct pruss *pruss, struct rproc *rproc);

#else

static inline struct pruss *pruss_get(struct device *dev)
{
	return ERR_PTR(-ENOTSUPP);
}

static inline void pruss_put(struct pruss *pruss) { }

static inline struct rproc *pruss_rproc_get(struct pruss *pruss,
					    enum pruss_pru_id pru_id)
{
	return ERR_PTR(-ENOTSUPP);
}

static inline void pruss_rproc_put(struct pruss *pruss, struct rproc *rproc) { }

#endif /* CONFIG_PRUSS_REMOTEPROC */

#endif /* __LINUX_PRUSS_H */
