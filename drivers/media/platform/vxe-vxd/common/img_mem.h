/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Typedefs for memory pool and attributes
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Amit Makani <amit.makani@ti.com>
 *
 * Re-written for upstreamimg
 *	Sidraya Jayagond <sidraya.bj@pathpartnertech.com>
 */
#ifndef __IMG_MEM__
#define __IMG_MEM__

/*
 * This type defines the memory attributes.
 * @0x00000001: Memory to be allocated as cached
 * @0x00000002: Memory to be allocated as uncached
 * @0x00000004: Memory to be allocated as write-combined
 *	(or equivalent buffered/burst writes mechanism)
 * @0x00001000: Memory can be read only by the core
 * @0x00002000: Memory can be written only by the core
 * @0x00010000: Memory should be readable by the cpu
 * @0x00020000: Memory should be writable by the cpu
 */
enum sys_emem_attrib {
	SYS_MEMATTRIB_CACHED             = 0x00000001,
	SYS_MEMATTRIB_UNCACHED           = 0x00000002,
	SYS_MEMATTRIB_WRITECOMBINE       = 0x00000004,
	SYS_MEMATTRIB_SECURE             = 0x00000010,
	SYS_MEMATTRIB_INPUT              = 0x00000100,
	SYS_MEMATTRIB_OUTPUT             = 0x00000200,
	SYS_MEMATTRIB_INTERNAL           = 0x00000400,
	SYS_MEMATTRIB_CORE_READ_ONLY     = 0x00001000,
	SYS_MEMATTRIB_CORE_WRITE_ONLY    = 0x00002000,
	SYS_MEMATTRIB_CPU_READ           = 0x00010000,
	SYS_MEMATTRIB_CPU_WRITE          = 0x00020000,
	SYS_MEMATTRIB_FORCE32BITS	 = 0x7FFFFFFFU
};

#endif /* __IMG_MEM__ */
