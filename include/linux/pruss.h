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

/**
 * enum pruss_mem - PRUSS memory range identifiers
 */
enum pruss_mem {
	PRUSS_MEM_DRAM0 = 0,
	PRUSS_MEM_DRAM1,
	PRUSS_MEM_SHRD_RAM2,
	PRUSS_MEM_INTC,
	PRUSS_MEM_CFG,
	PRUSS_MEM_IEP,
	PRUSS_MEM_MII_RT,
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

struct pruss;

#if IS_ENABLED(CONFIG_PRUSS_REMOTEPROC)

struct pruss *pruss_get(struct device *dev);
void pruss_put(struct pruss *pruss);
struct rproc *pruss_rproc_get(struct pruss *pruss,
			      enum pruss_pru_id pru_id);
void pruss_rproc_put(struct pruss *pruss, struct rproc *rproc);
int pruss_request_mem_region(struct pruss *pruss, enum pruss_mem mem_id,
			     struct pruss_mem_region *region);
int pruss_release_mem_region(struct pruss *pruss,
			     struct pruss_mem_region *region);

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

static inline int pruss_request_mem_region(struct pruss *pruss,
					   enum pruss_mem mem_id,
					   struct pruss_mem_region *region)
{
	return ERR_PTR(-ENOTSUPP);
}

static inline int pruss_release_mem_region(struct pruss *pruss,
					   struct pruss_mem_region *region)
{
	return ERR_PTR(-ENOTSUPP);
}

#endif /* CONFIG_PRUSS_REMOTEPROC */

#endif /* __LINUX_PRUSS_H */
