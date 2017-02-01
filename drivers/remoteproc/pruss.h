/*
 * PRU-ICSS sub-system specific definitions
 *
 * Copyright (C) 2014-2017 Texas Instruments Incorporated - http://www.ti.com/
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
 * struct pruss - PRUSS parent structure
 * @dev: pruss device pointer
 * @mem_regions: data for each of the PRUSS memory regions
 */
struct pruss {
	struct device *dev;
	struct pruss_mem_region mem_regions[PRUSS_MEM_MAX];
};

#endif	/* _PRUSS_H_ */
