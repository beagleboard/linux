// SPDX-License-Identifier: GPL-2.0
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

#include <linux/slab.h>
#include <linux/printk.h>
#include <linux/mutex.h>
#include <linux/dma-mapping.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>
#include <linux/types.h>

#include "idgen_api.h"
#include "lst.h"
#include "pool_api.h"
#include "img_errors.h"

/*
 * list can be modified by different instances. So please,
 * make sure to acquire mutex lock before initializing the list.
 */
static struct mutex *shared_res_mutex_handle;

/*
 * Max resource ID's.
 */
#define POOL_IDGEN_MAX_ID	   (0xFFFFFFFF)
/*
 * Size of blocks used for ID's.
 */
#define POOL_IDGEN_BLOCK_SIZE  (50)

/*
 * Indicates if the pool API has been indialized or not.
 * zero if not done. 1 if done.
 */
static int poolinitdone;

/* list of resource pool */
static struct lst_t poollist = {0};

/**
 * struct poollist - Structure contains resource list information.
 * @link: to be able to part of single linked list
 * @pool_mutex: lock
 * @freereslst: list of free resource structure
 * @actvreslst: list of active resource structure
 * @pfnfree: pool free callback function
 * @idgenhandle: ID generator context handl
 */
struct poollist {
	void **link;
	struct mutex *pool_mutex; /* Mutex lock */
	struct lst_t freereslst;
	struct lst_t actvreslst;
	pfrecalbkpntr pfnfree;
	void *idgenhandle;
};

/*
 * This structure contains pool resource.
 */
struct poolres {
	void **link; /* to be able to part of single linked list */
	/* Resource id */
	unsigned int resid;
	/* Pointer to destructor function */
	pdestcallbkptr desfunc;
	/* resource param */
	void *resparam;
	/* size of resource param in bytes */
	unsigned int resparmsize;
	/* pointer to resource pool list */
	struct poollist *respoollst;
	/* 1 if this is a clone of the original resource */
	int isclone;
	/* pointer to original resource */
	struct poolres *origres;
	/* list of cloned resource structures. Only used on the original */
	struct lst_t clonereslst;
	/* reference count. Only used on the original resource */
	unsigned int refcnt;
	void *cb_handle;
};

/*
 * This function initializes the list if not done earlier.
 */
int pool_init(void)
{
	/* Check if list already initialized */
	if (!poolinitdone) {
		/*
		 * list can be modified by different instances. So please,
		 * make sure to acquire mutex lock before initializing the list.
		 */

		shared_res_mutex_handle = kzalloc(sizeof(*shared_res_mutex_handle), GFP_KERNEL);
		if (!shared_res_mutex_handle)
			return -ENOMEM;

		mutex_init(shared_res_mutex_handle);

		/* initialize the list of pools */
		lst_init(&poollist);
		/* Get initialized flag to true */
		poolinitdone = 1;
	}

	return 0;
}

/*
 * This function de-initializes the list.
 */
void pool_deinit(void)
{
	struct poollist *respoollist;

	/* Check if list initialized */
	if (poolinitdone) {
		/* destroy any active pools */
		respoollist = (struct poollist *)lst_first(&poollist);
		while (respoollist) {
			pool_destroy(respoollist);
			respoollist = (struct poollist *)lst_first(&poollist);
		}

		/* Destroy mutex */
		mutex_destroy(shared_res_mutex_handle);
		kfree(shared_res_mutex_handle);
		shared_res_mutex_handle = NULL;

		/* set initialized flag to 0 */
		poolinitdone = 0;
	}
}

/*
 * This function creates pool.
 */
int pool_api_create(void **poolhndle)
{
	struct poollist *respoollist;
	unsigned int result = 0;

	/* Allocate a pool structure */
	respoollist = kzalloc(sizeof(*respoollist), GFP_KERNEL);
	if (!respoollist)
		return IMG_ERROR_OUT_OF_MEMORY;

	/* Initialize the pool info */
	lst_init(&respoollist->freereslst);
	lst_init(&respoollist->actvreslst);

	/* Create mutex */
	respoollist->pool_mutex = kzalloc(sizeof(*respoollist->pool_mutex), GFP_KERNEL);
	if (!respoollist->pool_mutex) {
		result = ENOMEM;
		goto error_create_context;
	}
	mutex_init(respoollist->pool_mutex);

	/* Create context for the Id generator */
	result = idgen_createcontext(POOL_IDGEN_MAX_ID,
				     POOL_IDGEN_BLOCK_SIZE, 0,
				     &respoollist->idgenhandle);
	if (result != IMG_SUCCESS)
		goto error_create_context;

	/* Disable interrupts */
	mutex_lock_nested(shared_res_mutex_handle, SUBCLASS_POOL_RES);

	/* Add to list of pools */
	lst_add(&poollist, respoollist);

	/* Enable interrupts */
	mutex_unlock(shared_res_mutex_handle);

	/* Return handle to pool */
	*poolhndle = respoollist;

	return IMG_SUCCESS;

	/* Error handling. */
error_create_context:
	kfree(respoollist);

	return result;
}

/*
 * This function destroys the pool.
 */
int pool_destroy(void *poolhndle)
{
	struct poollist *respoollist = poolhndle;
	struct poolres *respool;
	struct poolres *clonerespool;
	unsigned int result = 0;

	if (!poolinitdone || !respoollist) {
		result = IMG_ERROR_INVALID_PARAMETERS;
		goto	error_nolock;
	}

	/* Lock the pool */
	mutex_lock_nested(respoollist->pool_mutex, SUBCLASS_POOL);

	/* Disable interrupts */
	/*
	 * We need to check if we really need to check disable,
	 * interrupts because before deleting we need to make sure the
	 * pool lst is not being used other process. As of now getting ipl
	 * global mutex
	 */
	mutex_lock_nested(shared_res_mutex_handle, SUBCLASS_POOL_RES);

	/* Remove the pool from the active list */
	lst_remove(&poollist, respoollist);

	/* Enable interrupts */
	mutex_unlock(shared_res_mutex_handle);

	/* Destroy any resources in the free list */
	respool = (struct poolres *)lst_removehead(&respoollist->freereslst);
	while (respool) {
		respool->desfunc(respool->resparam, respool->cb_handle);
		kfree(respool);
		respool = (struct poolres *)
				lst_removehead(&respoollist->freereslst);
	}

	/* Destroy any resources in the active list */
	respool = (struct poolres *)lst_removehead(&respoollist->actvreslst);
	while (respool) {
		clonerespool = (struct poolres *)
				lst_removehead(&respool->clonereslst);
		while (clonerespool) {
			/*
			 * If we created a copy of the resources pvParam
			 * then free it.
			 * kfree(NULL) is safe and this check is probably not
			 * required
			 */
			kfree(clonerespool->resparam);

			kfree(clonerespool);
			clonerespool = (struct poolres *)
					lst_removehead(&respool->clonereslst);
		}

		/* Call the resource destructor */
		respool->desfunc(respool->resparam, respool->cb_handle);
		kfree(respool);
		respool = (struct poolres *)
				lst_removehead(&respoollist->actvreslst);
	}
	/* Destroy the context for the Id generator */
	if (respoollist->idgenhandle)
		result = idgen_destroycontext(respoollist->idgenhandle);

	/* Unlock the pool */
	mutex_unlock(respoollist->pool_mutex);

	/* Destroy mutex */
	mutex_destroy(respoollist->pool_mutex);
	kfree(respoollist->pool_mutex);
	respoollist->pool_mutex = NULL;

	/* Free the pool structure */
	kfree(respoollist);

	return IMG_SUCCESS;

error_nolock:
	return result;
}

int pool_setfreecalbck(void *poolhndle, pfrecalbkpntr pfnfree)
{
	struct poollist *respoollist = poolhndle;
	struct poolres *respool;
	unsigned int result = 0;

	if (!poolinitdone || !respoollist) {
		result = IMG_ERROR_INVALID_PARAMETERS;
		goto error_nolock;
	}

	/* Lock the pool */
	mutex_lock_nested(respoollist->pool_mutex, SUBCLASS_POOL);

	respoollist->pfnfree = pfnfree;

	/* If free callback set */
	if (respoollist->pfnfree) {
		/* Move resources from free to active list */
		respool = (struct poolres *)
				lst_removehead(&respoollist->freereslst);
		while (respool) {
			/* Add to active list */
			lst_add(&respoollist->actvreslst, respool);
			respool->refcnt++;

			/* Unlock the pool */
			mutex_unlock(respoollist->pool_mutex);

			/* Call the free callback */
			respoollist->pfnfree(respool->resid, respool->resparam);

			/* Lock the pool */
			mutex_lock_nested(respoollist->pool_mutex, SUBCLASS_POOL);

			/* Get next free resource */
			respool = (struct poolres *)
				lst_removehead(&respoollist->freereslst);
		}
	}

	/* Unlock the pool */
	mutex_unlock(respoollist->pool_mutex);

	/* Return IMG_SUCCESS */
	return IMG_SUCCESS;

error_nolock:
	return result;
}

int pool_resreg(void *poolhndle, pdestcallbkptr fndestructor,
		void *resparam, unsigned int resparamsize,
		int balloc, unsigned int *residptr,
		void **poolreshndle, void *cb_handle)
{
	struct poollist *respoollist = poolhndle;
	struct poolres *respool;
	unsigned int result = 0;

	if (!poolinitdone || !respoollist) {
		result = IMG_ERROR_INVALID_PARAMETERS;
		goto	error_nolock;
	}

	/* Allocate a resource structure */
	respool = kzalloc(sizeof(*respool), GFP_KERNEL);
	if (!respool)
		return IMG_ERROR_OUT_OF_MEMORY;

	/* Setup the resource */
	respool->desfunc  = fndestructor;
	respool->cb_handle = cb_handle;
	respool->resparam = resparam;
	respool->resparmsize = resparamsize;
	respool->respoollst = respoollist;
	lst_init(&respool->clonereslst);

	/* Lock the pool */
	mutex_lock_nested(respoollist->pool_mutex, SUBCLASS_POOL);

	/* Set resource id */
	result = idgen_allocid(respoollist->idgenhandle,
			       (void *)respool, &respool->resid);
	if (result != IMG_SUCCESS) {
		kfree(respool);
		/* Unlock the pool */
		mutex_unlock(respoollist->pool_mutex);
		return result;
	}

	/* If allocated or free callback not set */
	if (balloc || respoollist->pfnfree) {
		/* Add to active list */
		lst_add(&respoollist->actvreslst, respool);
		respool->refcnt++;
	} else {
		/* Add to free list */
		lst_add(&respoollist->freereslst, respool);
	}

	/* Return the resource id */
	if (residptr)
		*residptr = respool->resid;

	/* Return the handle to the resource */
	if (poolreshndle)
		*poolreshndle = respool;

	/* Unlock the pool */
	mutex_unlock(respoollist->pool_mutex);

	/* If free callback set */
	if (respoollist->pfnfree) {
		/* Call the free callback */
		respoollist->pfnfree(respool->resid, respool->resparam);
	}

	/* Return IMG_SUCCESS */
	return IMG_SUCCESS;

error_nolock:
	return result;
}

int pool_resdestroy(void *poolreshndle, int bforce)
{
	struct poolres *respool = poolreshndle;
	struct poollist *respoollist;
	struct poolres *origrespool;
	unsigned int result = 0;

	if (!poolinitdone || !respool) {
		result = IMG_ERROR_INVALID_PARAMETERS;
		goto error_nolock;
	}

	respoollist = respool->respoollst;

	/* If this is a clone */
	if (respool->isclone) {
		/* Get access to the original */
		origrespool = respool->origres;
		if (!origrespool) {
			result = IMG_ERROR_UNEXPECTED_STATE;
			goto error_nolock;
		}

		if (origrespool->isclone) {
			result = IMG_ERROR_UNEXPECTED_STATE;
			goto error_nolock;
		}

		/* Remove from the clone list */
		lst_remove(&origrespool->clonereslst, respool);

		/* Free resource id */
		result = idgen_freeid(respoollist->idgenhandle,
				      respool->resid);
		if (result != IMG_SUCCESS)
			return result;

		/*
		 * If we created a copy of the resources pvParam then free it
		 * kfree(NULL) is safe and this check is probably not required.
		 */
		kfree(respool->resparam);

		/* Free the clone resource structure */
		kfree(respool);

		/* Set resource to be "freed" to the original */
		respool = origrespool;
	}

	/* If there are still outstanding references */
	if (!bforce && respool->refcnt != 0) {
		/*
		 * We may need to mark the resource and destroy it when
		 * there are no outstanding references
		 */
		return IMG_SUCCESS;
	}

	/* Has the resource outstanding references */
	if (respool->refcnt != 0) {
		/* Remove the resource from the active list */
		lst_remove(&respoollist->actvreslst, respool);
	} else {
		/* Remove the resource from the free list */
		lst_remove(&respoollist->freereslst, respool);
	}

	/* Free resource id */
	result = idgen_freeid(respoollist->idgenhandle,
			      respool->resid);
	if (result != IMG_SUCCESS)
		return result;

	/* Call the resource destructor */
	respool->desfunc(respool->resparam, respool->cb_handle);
	kfree(respool);

	return IMG_SUCCESS;

error_nolock:
	return result;
}

int pool_resalloc(void *poolhndle, void *poolreshndle)
{
	struct poollist *respoollist = poolhndle;
	struct poolres *respool = poolreshndle;
	unsigned int result = 0;

	if (!poolinitdone || !respoollist || !poolreshndle) {
		result = IMG_ERROR_INVALID_PARAMETERS;
		goto error_nolock;
	}

	/* Lock the pool */
	mutex_lock_nested(respoollist->pool_mutex, SUBCLASS_POOL);

	/* Remove resource from free list */
	lst_remove(&respoollist->freereslst, respool);

	/* Add to active list */
	lst_add(&respoollist->actvreslst, respool);
	respool->refcnt++;

	/* Unlock the pool */
	mutex_unlock(respoollist->pool_mutex);

	/* Return IMG_SUCCESS */
	return IMG_SUCCESS;

error_nolock:
	return result;
}

int pool_resfree(void *poolreshndle)
{
	struct poolres *respool = poolreshndle;
	struct poollist *respoollist;
	struct poolres *origrespool;
	unsigned int result = 0;

	if (!poolinitdone || !respool) {
		result = IMG_ERROR_INVALID_PARAMETERS;
		goto error_nolock;
	}

	respoollist = respool->respoollst;

	/* Lock the pool */
	mutex_lock_nested(respoollist->pool_mutex, SUBCLASS_POOL);

	/* If this is a clone */
	if (respool->isclone) {
		/* Get access to the original */
		origrespool = respool->origres;
		if (!origrespool) {
			mutex_unlock(respoollist->pool_mutex);
			return IMG_ERROR_INVALID_PARAMETERS;
		}

		/* Remove from the clone list */
		lst_remove(&origrespool->clonereslst, respool);

		/* Free resource id */
		result = idgen_freeid(respoollist->idgenhandle,
				      respool->resid);
		if (result != IMG_SUCCESS) {
			/* Unlock the pool */
			mutex_unlock(respoollist->pool_mutex);
			return result;
		}

		/*
		 * If we created a copy of the resources pvParam then free it
		 * kfree(NULL) is safe and this check is probably not required.
		 */
		kfree(respool->resparam);

		/* Free the clone resource structure */
		kfree(respool);

		/* Set resource to be "freed" to the original */
		respool = origrespool;
	}

	/* Update the reference count */
	respool->refcnt--;

	/* If there are still outstanding references */
	if (respool->refcnt != 0) {
		/* Unlock the pool */
		mutex_unlock(respoollist->pool_mutex);
		/* Return IMG_SUCCESS */
		return IMG_SUCCESS;
	}

	/* Remove the resource from the active list */
	lst_remove(&respoollist->actvreslst, respool);

	/* If free callback set */
	if (respoollist->pfnfree) {
		/* Add to active list */
		lst_add(&respoollist->actvreslst, respool);
		respool->refcnt++;
	} else {
		/* Add to free list */
		lst_add(&respoollist->freereslst, respool);
	}

	/* Unlock the pool */
	mutex_unlock(respoollist->pool_mutex);

	/* If free callback set */
	if (respoollist->pfnfree) {
		/* Call the free callback */
		respoollist->pfnfree(respool->resid, respool->resparam);
	}

	/* Return IMG_SUCCESS */
	return IMG_SUCCESS;

error_nolock:
	return result;
}

int pool_resclone(void *poolreshndle, void **clonereshndle, void **resparam)
{
	struct poolres *respool = poolreshndle;
	struct poollist *respoollist;
	struct poolres *origrespool = respool;
	struct poolres *clonerespool;
	unsigned int result = 0;

	if (!poolinitdone || !respool) {
		result = IMG_ERROR_INVALID_PARAMETERS;
		goto error_nolock;
	}

	/* Allocate a resource structure */
	clonerespool = kzalloc(sizeof(*clonerespool), GFP_KERNEL);
	if (!clonerespool)
		return IMG_ERROR_OUT_OF_MEMORY;

	respoollist = respool->respoollst;
	if (!respoollist)
		return IMG_ERROR_FATAL;

	/* Lock the pool */
	mutex_lock_nested(respoollist->pool_mutex, SUBCLASS_POOL);

	/* Set resource id */
	result = idgen_allocid(respoollist->idgenhandle,
			       (void *)clonerespool, &clonerespool->resid);
	if (result != IMG_SUCCESS)
		goto error_alloc_id;

	/* If this is a clone, set the original */
	if (respool->isclone)
		origrespool = respool->origres;

	/* Setup the cloned resource */
	clonerespool->isclone = 1;
	clonerespool->respoollst = respoollist;
	clonerespool->origres = origrespool;

	/* Add to clone list */
	lst_add(&origrespool->clonereslst, clonerespool);
	origrespool->refcnt++;

	/* If ppvParam is not IMG_NULL */
	if (resparam) {
		/* If the size of the original vParam is 0 */
		if (origrespool->resparmsize == 0) {
			*resparam = NULL;
		} else {
			/* Allocate memory for a copy of the original vParam */
			/*
			 * kmemdup allocates memory of length
			 * origrespool->resparmsize and to resparam and copy
			 * origrespool->resparam to resparam of the allocated
			 * length
			 */
			*resparam = kmemdup(origrespool->resparam,
					    origrespool->resparmsize,
					    GFP_KERNEL);
			if (!(*resparam)) {
				result = IMG_ERROR_OUT_OF_MEMORY;
				goto error_copy_param;
			}
		}
	}

	/* Unlock the pool */
	mutex_unlock(respoollist->pool_mutex);

	/* Return the cloned resource */
	*clonereshndle = clonerespool;

	/* Return IMG_SUCCESS */
	return IMG_SUCCESS;

	/* Error handling. */
error_copy_param:
	lst_remove(&origrespool->clonereslst, clonerespool);
	origrespool->refcnt--;
error_alloc_id:
	kfree(clonerespool);

	/* Unlock the pool */
	mutex_unlock(respoollist->pool_mutex);

error_nolock:
	return result;
}
