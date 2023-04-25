/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Address allocation management API.
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Lakshmi Sankar <lakshmisankar-t@ti.com>
 *
 * Re-written for upstream
 *	Sidraya Jayagond <sidraya.bj@pathpartnertech.com>
 */
#ifndef __ADDR_ALLOC_H__
#define __ADDR_ALLOC_H__

#include <linux/types.h>
#include "ra.h"

/* Defines whether sequential or random allocation is used */
enum {
	SEQUENTIAL_ALLOCATION,
	RANDOM_ALLOCATION,
	RANDOM_FORCE32BITS = 0x7FFFFFFFU
};

/**
 * struct addr_region - Memory region structure
 *@name: A pointer to a sring containing the name of the region.
 *		NULL for the default memory region.
 *@base_addr: The base address of the memory region.
 *@size: The size of the memory region.
 *@guard_band: The size of any guard band to be used.
 *		Guard bands can be useful in separating block allocations
 *		and allows the caller to detect erroneous accesses
 *		into these areas.
 *@nxt_region:Used internally by the ADDR API.A pointer used to point
 *		to the next memory region.
 *@arena: Used internally by the ADDR API. A to a structure used to
 *		maintain and perform address allocation.
 *
 * This structure contains information about the memory region.
 */
struct addr_region {
	unsigned char *name;
	unsigned long long base_addr;
	unsigned long long size;
	unsigned int guard_band;
	struct addr_region *nxt_region;
	void *arena;
};

/*
 * This structure contains the context for allocation.
 *@regions: Pointer the first region in the list.
 *@default_region: Pointer the default region.
 *@no_regions: Number of regions currently available (including default)
 */
struct addr_context {
	struct addr_region *regions;
	struct addr_region *default_region;
	unsigned int no_regions;
};

/*
 * @Function	ADDR_Initialise
 * @Description
 * This function is used to initialise the address alocation sub-system.
 * NOTE: This function may be called multiple times. The initialisation only
 * happens the first time it is called.
 * @Return	IMG_SUCCESS or an error code.
 */
int addr_initialise(void);

/*
 * @Function	addr_deinitialise
 * @Description
 * This function is used to de-initialise the address alocation sub-system.
 * @Return	IMG_SUCCESS or an error code.
 */
int addr_deinitialise(void);

/*
 * @Function	addr_define_mem_region
 * @Description
 * This function is used define a memory region.
 * NOTE: The region structure MUST be defined in static memory as this
 * is retained and used by the ADDR sub-system.
 * NOTE: Invalid parameters are trapped by asserts.
 * @Input	region: A pointer to a region structure.
 * @Return	IMG_RESULT  : IMG_SUCCESS or an error code.
 */
int addr_define_mem_region(struct addr_region * const region);

/*
 * @Function	addr_malloc
 * @Description
 * This function is used allocate space within a memory region.
 * NOTE: Allocation failures or invalid parameters are trapped by asserts.
 * @Input	name: Is a pointer the name of the memory region.
 *		NULL can be used to allocate space from the
 *		default memory region.
 * @Input	size: The size (in bytes) of the allocation.
 * @Output	base_adr : The address of the allocated space.
 * @Return	IMG_SUCCESS or an error code.
 */
int addr_malloc(const unsigned char *const name,
		unsigned long long size,
		unsigned long long *const base_adr);

/*
 * @Function	addr_free
 * @Description
 * This function is used free a previously allocate space within
 * a memory region.
 * NOTE: Invalid parameters are trapped by asserts.
 * @Input	name: Is a pointer to the name of the memory region.
 *		NULL is used to free space from the default memory region.
 *@Input	addr: The address allocated.
 *@Return	IMG_SUCCESS or an error code.
 */
int addr_free(const unsigned char * const name, unsigned long long addr);

/*
 * @Function	addr_cx_initialise
 * @Description
 * This function is used to initialise the address allocation sub-system with
 * an external context structure.
 * NOTE: This function should be call only once for the context.
 * @Input	context : Pointer to context structure.
 * @Return	IMG_SUCCESS or an error code.
 */
int addr_cx_initialise(struct addr_context * const context);

/*
 * @Function	addr_cx_deinitialise
 * @Description
 * This function is used to de-initialise the address allocation
 * sub-system with an external context structure.
 * @Input	context : Pointer to context structure.
 * @Return	IMG_SUCCESS or an error code.
 */
int addr_cx_deinitialise(struct addr_context * const context);

/*
 * @Function	addr_cx_define_mem_region
 * @Description
 * This function is used define a memory region with an external
 * context structure.
 * NOTE: The region structure MUST be defined in static memory as this
 * is retained and used by the ADDR sub-system.
 * NOTE: Invalid parameters are trapped by asserts.
 * @Input	context : Pointer to context structure.
 * @Input	region : A pointer to a region structure.
 * @Return	IMG_SUCCESS or an error code.
 */
int addr_cx_define_mem_region(struct addr_context *const context,
			      struct addr_region *const region);

/*
 * @Function	addr_cx_malloc
 * @Description
 * This function is used allocate space within a memory region with
 * an external context structure.
 * NOTE: Allocation failures or invalid parameters are trapped by asserts.
 * @Input	context : Pointer to context structure.
 * @Input	name : Is a pointer the name of the memory region.
 *		NULL can be used to allocate space from the
 *		default memory region.
 * @Input	size : The size (in bytes) of the allocation.
 * @Output	base_adr : The address of the allocated space.
 * @Return	IMG_SUCCESS or an error code.
 */
int addr_cx_malloc(struct addr_context * const context,
		   const unsigned char *const name,
		   unsigned long long size,
		   unsigned long long *const base_adr);

/*
 * @Function	addr_cx_malloc_res
 * @Description
 * This function is used allocate space within a memory region with
 * an external context structure.
 * NOTE: Allocation failures are returned in IMG_RESULT, however invalid
 * parameters are trapped by asserts.
 * @Input	context : Pointer to context structure.
 * @Input	name : Is a pointer the name of the memory region.
 *		NULL can be used to allocate space from the
 *		default memory region.
 * @Input	size : The size (in bytes) of the allocation.
 * @Input	base_adr : Pointer to the address of the allocated space.
 * @Return	IMG_SUCCESS or an error code.
 */
int addr_cx_malloc_res(struct addr_context *const context,
		       const unsigned char *const name,
		       unsigned long long size,
		       unsigned long long * const base_adr);

/*
 * @Function	addr_cx_malloc1_res
 * @Description
 * This function is used allocate space within a memory region with
 * an external context structure.
 * NOTE: Allocation failures are returned in IMG_RESULT, however invalid
 * parameters are trapped by asserts.
 * @Input	context : Pointer to context structure.
 * @Input	name : Is a pointer the name of the memory region.
 *		NULL can be used to allocate space from the
 *		default memory region.
 * @Input	size	: The size (in bytes) of the allocation.
 * @Input	alignment : The required byte alignment (1, 2, 4, 8, 16 etc).
 * @Input	base_adr : Pointer to the address of the allocated space.
 * @Return	IMG_SUCCESS or an error code.
 */
int addr_cx_malloc_align_res(struct addr_context *const context,
			     const unsigned char *const name,
			     unsigned long long size,
			     unsigned long long alignment,
			     unsigned long long *const base_adr);

/*
 * @Function	addr_cx_free
 * @Description
 * This function is used free a previously allocate space within a memory region
 * with an external context structure.
 * NOTE: Invalid parameters are trapped by asserts.
 * @Input	context : Pointer to context structure.
 * @Input	name : Is a pointer the name of the memory region.
 *		NULL is used to free space from the
 *		default memory region.
 * @Input	addr : The address allocated.
 * @Return	IMG_SUCCESS or an error code.
 */
int addr_cx_free(struct addr_context *const context,
		 const unsigned char *const name,
		 unsigned long long addr);

#endif /* __ADDR_ALLOC_H__	*/
