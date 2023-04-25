// SPDX-License-Identifier: GPL-2.0
/*
 * This component is used to track decoder resources,
 * and share them across other components.
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
#include <linux/types.h>
#include <linux/dma-mapping.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>

#include "dq.h"
#include "idgen_api.h"
#include "rman_api.h"
#include "img_errors.h"

/*
 * The following macros are used to build/decompose the composite resource Id
 * made up from the bucket index + 1 and the allocated resource Id.
 */
#define RMAN_CRESID_BUCKET_INDEX_BITS	(8)
#define RMAN_CRESID_RES_ID_BITS		(32 - RMAN_CRESID_BUCKET_INDEX_BITS)
#define RMAN_CRESID_MAX_RES_ID		((1 << RMAN_CRESID_RES_ID_BITS) - 1)
#define RMAN_CRESID_RES_ID_MASK		(RMAN_CRESID_MAX_RES_ID)
#define RMAN_CRESID_BUCKET_SHIFT	(RMAN_CRESID_RES_ID_BITS)
#define RMAN_CRESID_MAX_BUCKET_INDEX	\
			((1 << RMAN_CRESID_BUCKET_INDEX_BITS) - 1)

#define RMAN_MAX_ID		4096
#define RMAN_ID_BLOCKSIZE	256

/* global state variable */
static unsigned char inited;
static struct rman_bucket *bucket_array[RMAN_CRESID_MAX_BUCKET_INDEX] = {0};
static struct rman_bucket *global_res_bucket;
static struct rman_bucket *shared_res_bucket;
static struct mutex *shared_res_mutex_handle;
static struct mutex *global_mutex;

/*
 * This structure contains the bucket information.
 */
struct rman_bucket {
	void		**link; /* to be part of single linked list */
	struct dq_linkage_t	res_list;
	unsigned int		bucket_idx;
	void		*id_gen;
	unsigned int	res_cnt;
};

/*
 * This structure contains the resource details for a resource registered with
 * the resource manager.
 */
struct rman_res {
	struct dq_linkage_t	link; /* to be part of double linked list */
	struct rman_bucket	*bucket;
	unsigned int		type_id;
	rman_fn_free		fn_free;
	void			*param;
	unsigned int		res_id;
	struct mutex		*mutex_handle; /*resource mutex */
	unsigned char		*res_name;
	struct rman_res		*shared_res;
	unsigned int		ref_cnt;
};

/*
 * initialization
 */
int rman_initialise(void)
{
	unsigned int ret;

	if (!inited) {
		shared_res_mutex_handle = kzalloc(sizeof(*shared_res_mutex_handle), GFP_KERNEL);
		if (!shared_res_mutex_handle)
			return IMG_ERROR_OUT_OF_MEMORY;

		mutex_init(shared_res_mutex_handle);

		/* Set initialised flag */
		inited = TRUE;

		/* Create the global resource bucket */
		ret = rman_create_bucket((void **)&global_res_bucket);
		IMG_DBG_ASSERT(ret == IMG_SUCCESS);
		if (ret != IMG_SUCCESS)
			return ret;

		/* Create the shared resource bucket */
		ret = rman_create_bucket((void **)&shared_res_bucket);
		IMG_DBG_ASSERT(ret == IMG_SUCCESS);
		if (ret != IMG_SUCCESS)
			return ret;

		global_mutex = kzalloc(sizeof(*global_mutex), GFP_KERNEL);
		if (!global_mutex)
			return IMG_ERROR_OUT_OF_MEMORY;

		mutex_init(global_mutex);
	}
	return IMG_SUCCESS;
}

/*
 * deinitialization
 */
void rman_deinitialise(void)
{
	unsigned int	i;

	if (inited) {
		/* Destroy the golbal resource bucket */
		rman_destroy_bucket(global_res_bucket);

		/* Destroy the shared resource bucket */
		rman_destroy_bucket(shared_res_bucket);

		/* Make sure we destroy the mutex after destroying the bucket */
		mutex_destroy(global_mutex);
		kfree(global_mutex);
		global_mutex = NULL;

		/* Destroy mutex */
		mutex_destroy(shared_res_mutex_handle);
		kfree(shared_res_mutex_handle);
		shared_res_mutex_handle = NULL;

		/* Check all buckets destroyed */
		for (i = 0; i < RMAN_CRESID_MAX_BUCKET_INDEX; i++)
			IMG_DBG_ASSERT(!bucket_array[i]);

		/* Reset initialised flag */
		inited = FALSE;
	}
}

int rman_create_bucket(void **res_bucket_handle)
{
	struct rman_bucket	*bucket;
	unsigned int			i;
	int			ret;

	IMG_DBG_ASSERT(inited);

	/* Allocate a bucket structure */
	bucket = kzalloc(sizeof(*bucket), GFP_KERNEL);
	IMG_DBG_ASSERT(bucket);
	if (!bucket)
		return IMG_ERROR_OUT_OF_MEMORY;

	/* Initialise the resource list */
	dq_init(&bucket->res_list);

	/* Then start allocating resource ids at the first */
	ret = idgen_createcontext(RMAN_MAX_ID, RMAN_ID_BLOCKSIZE, FALSE,
				  &bucket->id_gen);
	if (ret != IMG_SUCCESS) {
		kfree(bucket);
		IMG_DBG_ASSERT("failed to create IDGEN context" == NULL);
		return ret;
	}

	/* Locate free bucket index within the table */
	mutex_lock_nested(shared_res_mutex_handle, SUBCLASS_RMAN);
	for (i = 0; i < RMAN_CRESID_MAX_BUCKET_INDEX; i++) {
		if (!bucket_array[i])
			break;
	}
	if (i >= RMAN_CRESID_MAX_BUCKET_INDEX) {
		mutex_unlock(shared_res_mutex_handle);
		idgen_destroycontext(bucket->id_gen);
		kfree(bucket);
		IMG_DBG_ASSERT("No free buckets left" == NULL);
		return IMG_ERROR_GENERIC_FAILURE;
	}

	/* Allocate bucket index */
	bucket->bucket_idx = i;
	bucket_array[i] = bucket;

	mutex_unlock(shared_res_mutex_handle);

	/* Return the bucket handle */
	*res_bucket_handle = bucket;

	return IMG_SUCCESS;
}

void rman_destroy_bucket(void *res_bucket_handle)
{
	struct rman_bucket *bucket = (struct rman_bucket *)res_bucket_handle;

	IMG_DBG_ASSERT(inited);

	IMG_DBG_ASSERT(bucket);
	if (!bucket)
		return;

	IMG_DBG_ASSERT(bucket->bucket_idx < RMAN_CRESID_MAX_BUCKET_INDEX);
	IMG_DBG_ASSERT(bucket_array[bucket->bucket_idx]);

	/* Free all resources from the bucket */
	rman_free_resources(res_bucket_handle, RMAN_TYPE_P1);
	rman_free_resources(res_bucket_handle, RMAN_TYPE_P2);
	rman_free_resources(res_bucket_handle, RMAN_TYPE_P3);
	rman_free_resources(res_bucket_handle, RMAN_ALL_TYPES);

	/* free sticky resources last: other resources are dependent on them */
	rman_free_resources(res_bucket_handle, RMAN_STICKY);
	/* Use proper locking around global buckets.  */
	mutex_lock_nested(shared_res_mutex_handle, SUBCLASS_RMAN);

	/* Free from array of bucket pointers */
	bucket_array[bucket->bucket_idx] = NULL;

	mutex_unlock(shared_res_mutex_handle);

	/* Free the bucket itself */
	idgen_destroycontext(bucket->id_gen);
	kfree(bucket);
}

void *rman_get_global_bucket(void)
{
	IMG_DBG_ASSERT(inited);
	IMG_DBG_ASSERT(global_res_bucket);

	/* Return the handle of the global resource bucket */
	return global_res_bucket;
}

int rman_register_resource(void *res_bucket_handle, unsigned int type_id,
			   rman_fn_free fnfree, void *param,
			   void **res_handle, unsigned int *res_id)
{
	struct rman_bucket *bucket = (struct rman_bucket *)res_bucket_handle;
	struct rman_res		*res;
	int			 ret;

	IMG_DBG_ASSERT(inited);
	IMG_DBG_ASSERT(type_id != RMAN_ALL_TYPES);

	IMG_DBG_ASSERT(res_bucket_handle);
	if (!res_bucket_handle)
		return IMG_ERROR_GENERIC_FAILURE;

	/* Allocate a resource structure */
	res = kzalloc(sizeof(*res), GFP_KERNEL);
	IMG_DBG_ASSERT(res);
	if (!res)
		return IMG_ERROR_OUT_OF_MEMORY;

	/* Fill in the resource structure */
	res->bucket	= bucket;
	res->type_id	= type_id;
	res->fn_free	= fnfree;
	res->param	= param;

	/* Allocate resource Id */
	mutex_lock_nested(global_mutex, SUBCLASS_RMAN);
	ret = idgen_allocid(bucket->id_gen, res, &res->res_id);
	mutex_unlock(global_mutex);
	if (ret != IMG_SUCCESS) {
		IMG_DBG_ASSERT("failed to allocate RMAN id" == NULL);
		return ret;
	}
	IMG_DBG_ASSERT(res->res_id <= RMAN_CRESID_MAX_RES_ID);

	/* add this resource to the bucket */
	mutex_lock_nested(shared_res_mutex_handle, SUBCLASS_RMAN);
	dq_addtail(&bucket->res_list, res);

	/* Update count of resources */
	bucket->res_cnt++;
	mutex_unlock(shared_res_mutex_handle);

	/* If resource handle required */
	if (res_handle)
		*res_handle = res;

	/* If resource id required */
	if (res_id)
		*res_id = rman_get_resource_id(res);

	return IMG_SUCCESS;
}

unsigned int rman_get_resource_id(void *res_handle)
{
	struct rman_res *res = res_handle;
	unsigned int		ext_res_id;

	IMG_DBG_ASSERT(res_handle);
	if (!res_handle)
		return 0;

	IMG_DBG_ASSERT(res->res_id <= RMAN_CRESID_MAX_RES_ID);
	IMG_DBG_ASSERT(res->bucket->bucket_idx < RMAN_CRESID_MAX_BUCKET_INDEX);
	if (res->bucket->bucket_idx >= RMAN_CRESID_MAX_BUCKET_INDEX)
		return 0;

	ext_res_id = (((res->bucket->bucket_idx + 1) <<
			 RMAN_CRESID_BUCKET_SHIFT) | res->res_id);

	return ext_res_id;
}

static void *rman_getresource_int(void *res_bucket_handle, unsigned int res_id,
				  unsigned int type_id, void **res_handle)
{
	struct rman_bucket *bucket = (struct rman_bucket *)res_bucket_handle;
	struct rman_res	*res;
	int		ret;

	IMG_DBG_ASSERT(res_id <= RMAN_CRESID_MAX_RES_ID);

	/* Loop over the resources in this bucket till we find the required id */
	mutex_lock_nested(global_mutex, SUBCLASS_RMAN);
	ret = idgen_gethandle(bucket->id_gen, res_id, (void **)&res);
	mutex_unlock(global_mutex);
	if (ret != IMG_SUCCESS) {
		IMG_DBG_ASSERT("failed to get RMAN resource" == NULL);
		return NULL;
	}

	/* If the resource handle is required */
	if (res_handle)
		*res_handle = res; /* Return it */

	/* If the resource was not found */
	IMG_DBG_ASSERT(res);
	IMG_DBG_ASSERT((void *)res != &bucket->res_list);
	if (!res || ((void *)res == &bucket->res_list))
		return NULL;

	/* Cross check the type */
	IMG_DBG_ASSERT(type_id == res->type_id);

	/* Return the resource. */
	return res->param;
}

int rman_get_resource(unsigned int res_id, unsigned int type_id, void **param,
		      void **res_handle)
{
	unsigned int	bucket_idx = (res_id >> RMAN_CRESID_BUCKET_SHIFT) - 1;
	unsigned int	int_res_id = (res_id & RMAN_CRESID_RES_ID_MASK);
	void	*local_param;

	IMG_DBG_ASSERT(bucket_idx < RMAN_CRESID_MAX_BUCKET_INDEX);
	if (bucket_idx >= RMAN_CRESID_MAX_BUCKET_INDEX)
		return IMG_ERROR_INVALID_ID; /* Happens when bucket_idx == 0 */

	IMG_DBG_ASSERT(bucket_array[bucket_idx]);
	if (!bucket_array[bucket_idx])
		return IMG_ERROR_INVALID_ID;

	local_param = rman_getresource_int(bucket_array[bucket_idx],
					   int_res_id, type_id,
					   res_handle);

	/* If we didn't find the resource */
	if (!local_param)
		return IMG_ERROR_INVALID_ID;

	/* Return the resource */
	if (param)
		*param = local_param;

	return IMG_SUCCESS;
}

int rman_get_named_resource(unsigned char *res_name, rman_fn_alloc fn_alloc,
			    void *alloc_info, void *res_bucket_handle,
			    unsigned int type_id, rman_fn_free fn_free,
			    void **param, void **res_handle, unsigned int *res_id)
{
	struct rman_bucket *bucket = res_bucket_handle;
	struct rman_res	*res;
	unsigned int		ret;
	void	*local_param;
	unsigned char	found = FALSE;

	IMG_DBG_ASSERT(inited);

	IMG_DBG_ASSERT(res_bucket_handle);
	if (!res_bucket_handle)
		return IMG_ERROR_GENERIC_FAILURE;

	/* Lock the shared resources */
	mutex_lock_nested(shared_res_mutex_handle, SUBCLASS_RMAN);
	res = (struct rman_res *)dq_first(&bucket->res_list);
	while (res && ((void *)res != &bucket->res_list)) {
		/* If resource already in the shared list */
		if (res->res_name && (strcmp(res_name,
					     res->res_name) == 0)) {
			IMG_DBG_ASSERT(res->fn_free == fn_free);
			found = TRUE;
			break;
		}

		/* Move to next resource */
		res = (struct rman_res *)dq_next(res);
	}
	mutex_unlock(shared_res_mutex_handle);

	/* If the named resource was not found */
	if (!found) {
		/* Allocate the resource */
		ret = fn_alloc(alloc_info, &local_param);
		IMG_DBG_ASSERT(ret == IMG_SUCCESS);
		if (ret != IMG_SUCCESS)
			return ret;

		/* Register the named resource */
		ret = rman_register_resource(res_bucket_handle, type_id,
					     fn_free, local_param,
					     (void **)&res, NULL);
		IMG_DBG_ASSERT(ret == IMG_SUCCESS);
		if (ret != IMG_SUCCESS)
			return ret;

		mutex_lock_nested(shared_res_mutex_handle, SUBCLASS_RMAN);
		res->res_name = res_name;
		mutex_unlock(shared_res_mutex_handle);
	}

	/* Return the pvParam value */
	*param = res->param;

	/* If resource handle required */
	if (res_handle)
		*res_handle = res;

	/* If resource id required */
	if (res_id)
		*res_id = rman_get_resource_id(res);

	/* Exit */
	return IMG_SUCCESS;
}

static void rman_free_resource_int(struct rman_res *res)
{
	struct rman_bucket *bucket = res->bucket;

	/* Remove the resource from the active list */
	mutex_lock_nested(shared_res_mutex_handle, SUBCLASS_RMAN);

	/* Remove from list */
	dq_remove(res);

	/* Update count of resources */
	bucket->res_cnt--;

	mutex_unlock(shared_res_mutex_handle);

	/* If mutex associated with the resource */
	if (res->mutex_handle) {
		/* Destroy mutex */
		mutex_destroy(res->mutex_handle);
		kfree(res->mutex_handle);
		res->mutex_handle = NULL;
	}

	/* If this resource is not already shared */
	if (res->shared_res) {
		/* Lock the shared resources */
		mutex_lock_nested(shared_res_mutex_handle, SUBCLASS_RMAN);

		/* Update the reference count */
		IMG_DBG_ASSERT(res->shared_res->ref_cnt != 0);
		res->shared_res->ref_cnt--;

		/* If this is the last free for the shared resource */
		if (res->shared_res->ref_cnt == 0)
			/* Free the shared resource */
			rman_free_resource_int(res->shared_res);

		/* UnLock the shared resources */
		mutex_unlock(shared_res_mutex_handle);
	} else {
		/* If there is a free callback function. */
		if (res->fn_free)
			/* Call resource free callback */
			res->fn_free(res->param);
	}

	/* If the resource has a name then free it */
	kfree(res->res_name);

	/* Free the resource ID. */
	mutex_lock_nested(global_mutex, SUBCLASS_RMAN);
	idgen_freeid(bucket->id_gen, res->res_id);
	mutex_unlock(global_mutex);

	/* Free a resource structure */
	kfree(res);
}

void rman_free_resource(void *res_handle)
{
	struct rman_res *res;

	IMG_DBG_ASSERT(inited);

	IMG_DBG_ASSERT(res_handle);
	if (!res_handle)
		return;

	/* Get access to the resource structure */
	res = (struct rman_res *)res_handle;

	/* Free resource */
	rman_free_resource_int(res);
}

void rman_lock_resource(void *res_handle)
{
	struct rman_res	*res;

	IMG_DBG_ASSERT(inited);

	IMG_DBG_ASSERT(res_handle);
	if (!res_handle)
		return;

	/* Get access to the resource structure */
	res = (struct rman_res *)res_handle;

	/* If this is a shared resource */
	if (res->shared_res)
		/* We need to lock/unlock the underlying shared resource */
		res = res->shared_res;

	/* If no mutex associated with this resource */
	if (!res->mutex_handle) {
		/* Create one */

		res->mutex_handle = kzalloc(sizeof(*res->mutex_handle), GFP_KERNEL);
		if (!res->mutex_handle)
			return;

		mutex_init(res->mutex_handle);
	}

	/* lock it */
	mutex_lock(res->mutex_handle);
}

void rman_unlock_resource(void *res_handle)
{
	struct rman_res *res;

	IMG_DBG_ASSERT(inited);

	IMG_DBG_ASSERT(res_handle);
	if (!res_handle)
		return;

	/* Get access to the resource structure */
	res = (struct rman_res *)res_handle;

	/* If this is a shared resource */
	if (res->shared_res)
		/* We need to lock/unlock the underlying shared resource */
		res = res->shared_res;

	IMG_DBG_ASSERT(res->mutex_handle);

	/* Unlock mutex */
	mutex_unlock(res->mutex_handle);
}

void rman_free_resources(void *res_bucket_handle, unsigned int type_id)
{
	struct rman_bucket *bucket = (struct rman_bucket *)res_bucket_handle;
	struct rman_res		*res;

	IMG_DBG_ASSERT(inited);

	IMG_DBG_ASSERT(res_bucket_handle);
	if (!res_bucket_handle)
		return;

	/* Scan the active list looking for the resources to be freed */
	mutex_lock_nested(shared_res_mutex_handle, SUBCLASS_RMAN);
	res = (struct rman_res *)dq_first(&bucket->res_list);
	while ((res) && ((void *)res != &bucket->res_list)) {
		/* If this is resource is to be removed */
		if ((type_id == RMAN_ALL_TYPES  &&
		     res->type_id != RMAN_STICKY) ||
		     res->type_id == type_id) {
			/* Yes, remove it, Free current resource */
			mutex_unlock(shared_res_mutex_handle);
			rman_free_resource_int(res);
			mutex_lock_nested(shared_res_mutex_handle, SUBCLASS_RMAN);

			/* Restart from the beginning of the list */
			res = (struct rman_res *)dq_first(&bucket->res_list);
		} else {
			/* Move to next resource */
			res = (struct rman_res *)lst_next(res);
		}
	}
	mutex_unlock(shared_res_mutex_handle);
}
