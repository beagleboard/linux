/*
 *  Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com
 *  Author: Sricharan R <r.sricharan@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

/*
 * struct dma_xbar_device: dma crossbar device description
 * @safe_map: safe default value to initialize the crossbar
 * @dma_max: maximum number of dma request lines available at dma controller
 * @dma_map: array of dma request lines to crossbar number mapping
 * @dma_xbar_bas: dma crossbar base address
 * @reg_offs: offsets for each dma request number
 * @write: function to write in to the dma crossbar
 * @xbar_ops: pointer to xbar_ops for map and unmap.
 */
struct dma_xbar_device {
	uint safe_map;
	uint dma_max;
	uint *dma_map;
	void __iomem *dma_xbar_base;
	int *reg_offs;
	void (*write)(int, int, struct dma_xbar_device *);
	const struct xbar_ops *ops;
};

struct xbar_ops {
	uint32_t (*map)(uint32_t, struct dma_xbar_device *);
	void (*unmap)(uint32_t,  struct dma_xbar_device *);
};
