/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Resource pool manager API.
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
#ifndef __POOLAPI_H__
#define __POOLAPI_H__

#include "img_errors.h"
#include "lst.h"

/*
 * This is the prototype for "free" callback functions.  This function
 * is called when resources are returned to the pools list of free resources.
 * NOTE: The "freed" resource is then allocated and passed to the callback
 * function.
 */
typedef void (*pfrecalbkpntr)(unsigned int ui32resid, void *resparam);

/*
 * This is the prototype for "destructor" callback functions.  This function
 * is called when a resource registered with the resource pool manager is to
 * be destroyed.
 */
typedef void (*pdestcallbkptr)(void *resparam, void *cb_handle);

/*
 * pool_init - This function is used to initializes the resource pool manager component
 * and should be called at start-up.
 */
int pool_init(void);

/*
 * This function is used to deinitialises the resource pool manager component
 * and would normally be called at shutdown.
 */
void pool_deinit(void);

/*
 * This function is used to create a resource pool into which resources can be
 * placed.
 */
int pool_api_create(void **poolhndle);

/*
 * This function is used to destroy a resource pool.
 * NOTE: Destroying a resource pool destroys all of the resources within the
 * pool by calling the associated destructor function #POOL_pfnDestructor
 * defined when the resource what registered using POOL_ResRegister().
 *
 * NOTE: All of the pools resources must be in the pools free list - the
 * allocated list must be empty.
 */
int pool_destroy(void *poolhndle);

/*
 * This function is used to set or remove a free callback function on a pool.
 * The free callback function gets call for any resources already in the
 * pools free list or for any resources that subsequently get freed.
 * NOTE: The resource passed to the callback function has been allocated before
 * the callback is made.
 */
int pool_setfreecalbck(void *poolhndle, pfrecalbkpntr pfnfree);

/*
 * This function is used to register a resource within a resource pool.  The
 * resource is added to the pools allocated or free list based on the value
 * of bAlloc.
 */
int pool_resreg(void *poolhndle, pdestcallbkptr fndestructor,
		void *resparam, unsigned int resparamsize,
		int balloc, unsigned int *residptr,
		void **poolreshndle, void *cb_handle);

/*
 * This function is used to destroy a resource.
 */
int pool_resdestroy(void *poolreshndle, int bforce);

/*
 * This function is used to get/allocate a resource from a pool.  This moves
 * the resource from the free to allocated list.
 */
int pool_resalloc(void *poolhndle, void *poolreshndle);

/*
 * This function is used to free a resource and return it to the pools lists of
 * free resources.
 * NOTE: The resources is only moved to the free list when all references to
 * the resource have been freed.
 */
int pool_resfree(void *poolreshndle);

/*
 * This function is used to clone a resource - this creates an additional
 * reference to the resource.
 * NOTE: The resources is only moved to the free list when all references to
 * the resource have been freed.
 * NOTE: If this function is used to clone the resource's pvParam data then
 * the clone of the data is freed when the clone of the resource is freed.
 * The resource destructor is NOT used for this - simply an IMG_FREE.
 */
int pool_resclone(void *poolreshndle, void **clonereshndle, void **resparam);

#endif /* __POOLAPI_H__ */
