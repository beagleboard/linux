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

/**
 * enum pruss_pru_id - PRUSS GPI configuration modes, used
 *		       to program the PRUSS_GPCFG0/1 registers
 */
enum pruss_gpi_mode {
	PRUSS_GPI_MODE_DIRECT = 0,
	PRUSS_GPI_MODE_PARALLEL,
	PRUSS_GPI_MODE_28BIT_SHIFT,
	PRUSS_GPI_MODE_MII,
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
int pruss_cfg_gpimode(struct pruss *pruss, struct rproc *rproc,
		      enum pruss_gpi_mode mode);
void pruss_cfg_miirt_enable(struct pruss *pruss, bool enable);
void pruss_cfg_xfr_enable(struct pruss *pruss, bool enable);
int pru_rproc_set_ctable(struct rproc *rproc, enum pru_ctable_idx c, u32 addr);
int pruss_intc_trigger(unsigned int irq);

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
	return -ENOTSUPP;
}

static inline int pruss_release_mem_region(struct pruss *pruss,
					   struct pruss_mem_region *region)
{
	return -ENOTSUPP;
}

static inline int pruss_cfg_gpimode(struct pruss *pruss, struct rproc *rproc,
				    enum pruss_gpi_mode mode)
{
	return -ENOTSUPP;
}

static inline void pruss_cfg_miirt_enable(struct pruss *pruss, bool enable) { }

static inline void pruss_cfg_xfr_enable(struct pruss *pruss, bool enable) { }

static inline int pru_rproc_set_ctable(struct rproc *rproc,
				       enum pru_ctable_idx c, u32 addr)
{
	return -ENOTSUPP;
}

static inline int pruss_intc_trigger(unsigned int irq)
{
	return -ENOTSUPP;
}

#endif /* CONFIG_PRUSS_REMOTEPROC */

#endif /* __LINUX_PRUSS_H */
