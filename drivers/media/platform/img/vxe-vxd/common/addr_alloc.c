// SPDX-License-Identifier: GPL-2.0
/*
 * Address allocation APIs - used to manage address allocation
 * with a number of predefined regions.
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

#include <linux/slab.h>
#include <linux/printk.h>
#include <linux/mutex.h>
#include <linux/dma-mapping.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>

#include "addr_alloc.h"
#include "hash.h"
#include "img_errors.h"

/* Global context. */
static struct addr_context global_ctx = {0};
/* Sub-system initialized. */
static int global_initialized;
/* Count of contexts. */
static unsigned int num_ctx;
/* Global mutex */
static struct mutex *global_lock;

/**
 * addr_initialise - addr_initialise
 */

int addr_initialise(void)
{
	unsigned int result = IMG_ERROR_ALREADY_INITIALISED;

	/* If we are not initialized */
	if (!global_initialized)
		result = addr_cx_initialise(&global_ctx);
	return result;
}

int addr_cx_initialise(struct addr_context * const context)
{
	unsigned int result = IMG_ERROR_FATAL;

	if (!context)
		return IMG_ERROR_INVALID_PARAMETERS;

	if (!global_initialized) {
		/* Initialise context */
		memset(context, 0x00, sizeof(struct addr_context));

		/* If no mutex associated with this resource */
		if (!global_lock) {
			/* Create one */

			global_lock = kzalloc(sizeof(*global_lock), GFP_KERNEL);
			if (!global_lock)
				return -ENOMEM;

			mutex_init(global_lock);
		}

		mutex_lock_nested(global_lock, SUBCLASS_ADDR_ALLOC);

		/* Initialise the hash functions. */
		result = vid_hash_initialise();
		if (result != IMG_SUCCESS) {
			mutex_unlock(global_lock);
			return IMG_ERROR_UNEXPECTED_STATE;
		}

		/* Initialise the arena functions */
		result = vid_ra_initialise();
		if (result != IMG_SUCCESS) {
			mutex_unlock(global_lock);
			result = vid_hash_finalise();
			return IMG_ERROR_UNEXPECTED_STATE;
		}

		/* We are now initialized */
		global_initialized = TRUE;
		result = IMG_SUCCESS;
	} else {
		mutex_lock_nested(global_lock, SUBCLASS_ADDR_ALLOC);
	}

	num_ctx++;
	mutex_unlock(global_lock);

	return result;
}

int addr_deinitialise(void)
{
	return addr_cx_deinitialise(&global_ctx);
}

int addr_cx_deinitialise(struct addr_context * const context)
{
	struct addr_region *tmp_region = NULL;
	unsigned int result = IMG_ERROR_FATAL;

	if (!context)
		return IMG_ERROR_INVALID_PARAMETERS;

	if (global_initialized) {
		mutex_lock_nested(global_lock, SUBCLASS_ADDR_ALLOC);

		tmp_region = context->regions;

		/* Delete all arena structure */
		if (context->default_region)
			result = vid_ra_delete(context->default_region->arena);

		while (tmp_region) {
			result = vid_ra_delete(tmp_region->arena);
			tmp_region = tmp_region->nxt_region;
		}

		if (num_ctx != 0)
			num_ctx--;

		result = IMG_SUCCESS;
		if (num_ctx == 0) {
			/* Free off resources */
			result = vid_hash_finalise();
			result = vid_ra_deinit();
			global_initialized = FALSE;

			mutex_unlock(global_lock);
			mutex_destroy(global_lock);
			kfree(global_lock);
			global_lock = NULL;
		} else {
			mutex_unlock(global_lock);
		}
	}

	return result;
}

int addr_define_mem_region(struct addr_region * const region)
{
	return addr_cx_define_mem_region(&global_ctx, region);
}

int addr_cx_define_mem_region(struct addr_context * const context,
			      struct addr_region * const region)
{
	struct addr_region *tmp_region = NULL;
	unsigned int result = IMG_SUCCESS;

	if (!context || !region)
		return IMG_ERROR_INVALID_PARAMETERS;

	mutex_lock_nested(global_lock, SUBCLASS_ADDR_ALLOC);

	tmp_region = context->regions;

	/* Ensure the link to the next is NULL */
	region->nxt_region = NULL;

	/* If this is the default memory region */
	if (!region->name) {
		/* Should not previously have been defined */
		if (context->default_region) {
			mutex_unlock(global_lock);
			return IMG_ERROR_UNEXPECTED_STATE;
		}

		context->default_region = region;
		context->no_regions++;

		/*
		 * Create an arena for memory allocation
		 * name of resource arena for debug
		 * start of resource
		 * size of resource
		 * allocation quantum
		 * import allocator
		 * import deallocator
		 * import handle
		 */
		result = vid_ra_create("memory",
				       region->base_addr,
				       region->size,
				       1,
				       NULL,
				       NULL,
				       NULL,
				       &region->arena);

		if (result != IMG_SUCCESS) {
			mutex_unlock(global_lock);
			return IMG_ERROR_UNEXPECTED_STATE;
		}
	} else {
		/*
		 * Run down the list of existing named regions
		 * to check if there is a region with this name
		 */
		while (tmp_region &&
		       (strcmp(region->name, tmp_region->name) != 0) &&
		       tmp_region->nxt_region) {
			tmp_region = tmp_region->nxt_region;
		}

		/* If we have items in the list */
		if (tmp_region) {
			/*
			 * Check we didn't stop because the name
			 * clashes with one already defined.
			 */

			if (strcmp(region->name, tmp_region->name) == 0 ||
			    tmp_region->nxt_region) {
				mutex_unlock(global_lock);
				return IMG_ERROR_UNEXPECTED_STATE;
			}

			/* Add to end of list */
			tmp_region->nxt_region = region;
		} else {
			/* Add to head of list */
			context->regions = region;
		}

		context->no_regions++;

		/*
		 * Create an arena for memory allocation
		 * name of resource arena for debug
		 * start of resource
		 * size of resource
		 * allocation quantum
		 * import allocator
		 * import deallocator
		 * import handle
		 */
		result = vid_ra_create(region->name,
				       region->base_addr,
				       region->size,
				       1,
				       NULL,
				       NULL,
				       NULL,
				       &region->arena);

		if (result != IMG_SUCCESS) {
			mutex_unlock(global_lock);
			return IMG_ERROR_UNEXPECTED_STATE;
		}
	}

	mutex_unlock(global_lock);

	/* Check the arean was created OK */
	if (!region->arena)
		return IMG_ERROR_UNEXPECTED_STATE;

	return result;
}

int addr_malloc(const unsigned char * const name,
		unsigned long long size,
		unsigned long long * const base_adr)
{
	return addr_cx_malloc(&global_ctx, name, size, base_adr);
}

int addr_cx_malloc(struct addr_context * const context,
		   const unsigned char * const name,
		   unsigned long long size,
		   unsigned long long * const base_adr)
{
	unsigned int result = IMG_ERROR_FATAL;
	struct addr_region *tmp_region = NULL;

	if (!context || !base_adr || !name)
		return IMG_ERROR_INVALID_PARAMETERS;

	*(base_adr) = (unsigned long long)-1LL;

	mutex_lock_nested(global_lock, SUBCLASS_ADDR_ALLOC);

	tmp_region = context->regions;

	/*
	 * Run down the list of existing named
	 * regions to locate this
	 */
	while (tmp_region && (strcmp(name, tmp_region->name) != 0) && (tmp_region->nxt_region))
		tmp_region = tmp_region->nxt_region;

	/* If there was no match. */
	if (!tmp_region || (strcmp(name, tmp_region->name) != 0)) {
		/* Use the default */
		if (!context->default_region) {
			mutex_unlock(global_lock);
			return IMG_ERROR_UNEXPECTED_STATE;
		}

		tmp_region = context->default_region;
	}

	if (!tmp_region) {
		mutex_unlock(global_lock);
		return IMG_ERROR_UNEXPECTED_STATE;
	}

	/* Allocate size + guard band */
	result = vid_ra_alloc(tmp_region->arena,
			      size + tmp_region->guard_band,
			      NULL,
			      NULL,
			      SEQUENTIAL_ALLOCATION,
			      1,
			      base_adr);
	if (result != IMG_SUCCESS) {
		mutex_unlock(global_lock);
		return IMG_ERROR_OUT_OF_MEMORY;
	}

	mutex_unlock(global_lock);

	return result;
}

int addr_cx_malloc_res(struct addr_context * const context,
		       const unsigned char * const name,
		       unsigned long long size,
		       unsigned long long * const base_adr)
{
	unsigned int result = IMG_ERROR_FATAL;
	struct addr_region *tmp_region = NULL;

	if (!context || !base_adr || !name)
		return IMG_ERROR_INVALID_PARAMETERS;

	mutex_lock_nested(global_lock, SUBCLASS_ADDR_ALLOC);

	tmp_region = context->regions;
	/* If the allocation is for the default region */
	/*
	 * Run down the list of existing named
	 * regions to locate this
	 */
	while (tmp_region && (strcmp(name, tmp_region->name) != 0) && (tmp_region->nxt_region))
		tmp_region = tmp_region->nxt_region;

	/* If there was no match. */
	if (!tmp_region || (strcmp(name, tmp_region->name) != 0)) {
		/* Use the default */
		if (!context->default_region) {
			mutex_unlock(global_lock);
			return IMG_ERROR_UNEXPECTED_STATE;
		}
		tmp_region = context->default_region;
	}
	if (!tmp_region) {
		mutex_unlock(global_lock);
		return IMG_ERROR_UNEXPECTED_STATE;
	}
	/* Allocate size + guard band */
	result = vid_ra_alloc(tmp_region->arena, size + tmp_region->guard_band,
			      NULL, NULL, SEQUENTIAL_ALLOCATION, 1, base_adr);
	if (result != IMG_SUCCESS) {
		mutex_unlock(global_lock);
		return IMG_ERROR_OUT_OF_MEMORY;
	}
	mutex_unlock(global_lock);

	return result;
}

int addr_cx_malloc_align_res(struct addr_context * const context,
			     const unsigned char * const name,
			     unsigned long long size,
			     unsigned long long alignment,
			     unsigned long long * const base_adr)
{
	unsigned int result;
	struct addr_region *tmp_region = NULL;

	if (!context || !base_adr || !name)
		return IMG_ERROR_INVALID_PARAMETERS;

	mutex_lock_nested(global_lock, SUBCLASS_ADDR_ALLOC);

	tmp_region = context->regions;

	/*
	 * Run down the list of existing named
	 * regions to locate this
	 */
	while (tmp_region &&
	       (strcmp(name, tmp_region->name) != 0) &&
	       (tmp_region->nxt_region)) {
		tmp_region = tmp_region->nxt_region;
	}
	/* If there was no match. */
	if (!tmp_region ||
	    (strcmp(name, tmp_region->name) != 0)) {
		/* Use the default */
		if (!context->default_region) {
			mutex_unlock(global_lock);
			return IMG_ERROR_UNEXPECTED_STATE;
		}

		tmp_region = context->default_region;
	}

	if (!tmp_region) {
		mutex_unlock(global_lock);
		return IMG_ERROR_UNEXPECTED_STATE;
	}
	/* Allocate size + guard band */
	result = vid_ra_alloc(tmp_region->arena,
			      size + tmp_region->guard_band,
			      NULL,
			      NULL,
			      SEQUENTIAL_ALLOCATION,
			      alignment,
			      base_adr);
	if (result != IMG_SUCCESS) {
		mutex_unlock(global_lock);
		return IMG_ERROR_OUT_OF_MEMORY;
	}

	mutex_unlock(global_lock);

	return result;
}

int addr_free(const unsigned char * const name, unsigned long long addr)
{
	return addr_cx_free(&global_ctx, name, addr);
}

int addr_cx_free(struct addr_context * const context,
		 const unsigned char * const name,
		 unsigned long long addr)
{
	struct addr_region *tmp_region;
	unsigned int result;

	if (!context)
		return IMG_ERROR_INVALID_PARAMETERS;

	tmp_region = context->regions;

	mutex_lock_nested(global_lock, SUBCLASS_ADDR_ALLOC);

	/* If the allocation is for the default region */
	if (!name) {
		if (!context->default_region) {
			result = IMG_ERROR_INVALID_PARAMETERS;
			goto error;
		}
		tmp_region = context->default_region;
	} else {
		/*
		 * Run down the list of existing named
		 * regions to locate this
		 */
		while (tmp_region &&
		       (strcmp(name, tmp_region->name) != 0) &&
		       tmp_region->nxt_region) {
			tmp_region = tmp_region->nxt_region;
		}

		/* If there was no match */
		if (!tmp_region || (strcmp(name, tmp_region->name) != 0)) {
			/* Use the default */
			if (!context->default_region) {
				result = IMG_ERROR_INVALID_PARAMETERS;
				goto error;
			}
			tmp_region = context->default_region;
		}
	}

	/* Free the address */
	result = vid_ra_free(tmp_region->arena, addr);

error:
	mutex_unlock(global_lock);
	return result;
}
