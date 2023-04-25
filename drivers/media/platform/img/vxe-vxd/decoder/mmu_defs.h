/* SPDX-License-Identifier: GPL-2.0 */
/*
 * V-DEC MMU Definitions
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Amit Makani <amit.makani@ti.com>
 */

#ifndef _VXD_MMU_DEF_H_
#define _VXD_MMU_DEF_H_

/*
 * This type defines MMU variant.
 */
enum mmu_etype {
	MMU_TYPE_NONE        = 0,
	MMU_TYPE_32BIT,
	MMU_TYPE_36BIT,
	MMU_TYPE_40BIT,
	MMU_TYPE_FORCE32BITS = 0x7FFFFFFFU
};

/**
 * enum mmu_eheap_id - This type defines the MMU heaps.
 * @MMU_HEAP_IMAGE_BUFFERS_UNTILED: Heap for untiled video buffers
 * @MMU_HEAP_BITSTREAM_BUFFERS : Heap for bitstream buffers
 * @MMU_HEAP_STREAM_BUFFERS : Heap for Stream buffers
 * @MMU_HEAP_MAX : Number of heaps
 * @MMU_HEAP_FORCE32BITS: MMU_HEAP_FORCE32BITS
 */
enum mmu_eheap_id {
	MMU_HEAP_IMAGE_BUFFERS_UNTILED = 0x00,
	MMU_HEAP_BITSTREAM_BUFFERS,
	MMU_HEAP_STREAM_BUFFERS,
	MMU_HEAP_MAX,
	MMU_HEAP_FORCE32BITS           = 0x7FFFFFFFU
};

#endif /* _VXD_MMU_DEFS_H_ */
