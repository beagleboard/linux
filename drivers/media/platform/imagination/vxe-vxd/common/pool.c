// SPDX-License-Identifier: GPL-2.0
/*
 * Object Pool Memory Allocator
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

#include <linux/dma-mapping.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>

#include "img_errors.h"
#include "pool.h"

#define BUFF_MAX_SIZE 4096
#define BUFF_MAX_GROW 32

/* 64 bits */
#define ALIGN_SIZE (sizeof(long long) - 1)

struct pool {
	unsigned char *name;
	unsigned int size;
	unsigned int grow;
	struct buffer *buffers;
	struct object *objects;
};

struct buffer {
	struct buffer *next;
};

struct object {
	struct object *next;
};

static inline unsigned char *strdup_cust(const unsigned char *str)
{
	unsigned char *r = kmalloc(strlen(str) + 1, GFP_KERNEL);

	if (r)
		strcpy(r, str);
	return r;
}

/*
 * pool_create - Create an sObject pool
 * @name: Name of sObject pool for diagnostic purposes
 * @obj_size: size of each sObject in the pool in bytes
 * @pool_hdnl: Will contain NULL or sObject pool handle
 *
 * This function Create an sObject pool
 */

int pool_create(const unsigned char * const name,
		unsigned int obj_size,
		struct pool ** const pool_hdnl)
{
	struct pool *local_pool = NULL;
	unsigned int result = IMG_ERROR_FATAL;

	if (!name || !pool_hdnl) {
		result = IMG_ERROR_INVALID_PARAMETERS;
		return result;
	}

	local_pool = kmalloc((sizeof(*local_pool)), GFP_KERNEL);
	if (!local_pool) {
		result = IMG_ERROR_INVALID_PARAMETERS;
		return result;
	}

	local_pool->name = strdup_cust((unsigned char *)name);
	local_pool->size = obj_size;
	local_pool->buffers = NULL;
	local_pool->objects = NULL;
	local_pool->grow =
		(BUFF_MAX_SIZE - sizeof(struct buffer)) /
		(obj_size + ALIGN_SIZE);

	if (local_pool->grow == 0)
		local_pool->grow = 1;
	else if (local_pool->grow > BUFF_MAX_GROW)
		local_pool->grow = BUFF_MAX_GROW;

	*pool_hdnl = local_pool;
	result = IMG_SUCCESS;

	return result;
}

/*
 * @Function	pool_delete
 * @Description
 * Delete an sObject pool. All psObjects allocated from the pool must
 * be free'd with pool_free() before deleting the sObject pool.
 * @Input	pool : Object Pool pointer
 * @Return IMG_SUCCESS or an error code.
 */
int pool_delete(struct pool * const pool_arg)
{
	struct buffer *local_buf = NULL;
	unsigned int result = IMG_ERROR_FATAL;

	if (!pool_arg) {
		result = IMG_ERROR_INVALID_PARAMETERS;
		return result;
	}

	local_buf = pool_arg->buffers;
	while (local_buf) {
		local_buf = local_buf->next;
		kfree(pool_arg->buffers);
		pool_arg->buffers = local_buf;
	}

	kfree(pool_arg->name);
	pool_arg->name = NULL;

	kfree(pool_arg);
	result = IMG_SUCCESS;

	return result;
}

/*
 * @Function	pool_alloc
 * @Description
 * Allocate an sObject from an sObject pool.
 * @Input	pool_arg : Object Pool
 * @Output	obj_hndl : Pointer containing the handle to the
 * object created or IMG_NULL
 * @Return    IMG_SUCCESS or an error code.
 */
int pool_alloc(struct pool * const pool_arg,
	       void ** const obj_hndl)
{
	struct object *local_obj1 = NULL;
	struct buffer *local_buf = NULL;
	unsigned int idx = 0;
	unsigned int sz = 0;
	unsigned int result = IMG_ERROR_FATAL;

	if (!pool_arg || !obj_hndl) {
		result = IMG_ERROR_INVALID_PARAMETERS;
		return result;
	}

	if (!pool_arg->objects) {
		sz = (pool_arg->size + ALIGN_SIZE);
		sz *= (pool_arg->grow + sizeof(struct buffer));
		local_buf = kmalloc(sz, GFP_KERNEL);
		if (!local_buf) {
			result = IMG_ERROR_MALLOC_FAILED;
			return result;
		}

		local_buf->next = pool_arg->buffers;
		pool_arg->buffers = local_buf;

		for (idx = 0; idx < pool_arg->grow; idx++) {
			struct object *local_obj2;
			unsigned char *temp_ptr = NULL;

			local_obj2 = (struct object *)(((unsigned char *)(local_buf + 1))
				+ (idx * (pool_arg->size + ALIGN_SIZE)));

			temp_ptr = (unsigned char *)local_obj2;
			if ((unsigned long)temp_ptr & ALIGN_SIZE) {
				temp_ptr += ((ALIGN_SIZE + 1)
					- ((unsigned long)temp_ptr & ALIGN_SIZE));
				local_obj2 = (struct object *)temp_ptr;
			}

			local_obj2->next = pool_arg->objects;
			pool_arg->objects = local_obj2;
		}
	}

	if (!pool_arg->objects) {
		result = IMG_ERROR_UNEXPECTED_STATE;
		return result;
	}

	local_obj1 = pool_arg->objects;
	pool_arg->objects = local_obj1->next;

	*obj_hndl = (void *)(local_obj1);
	result = IMG_SUCCESS;

	return result;
}

/*
 * @Function	pool_free
 * @Description
 * Free an sObject previously allocated from an sObject pool.
 * @Input	pool_arg : Object Pool pointer.
 * @Output	h_object : Handle to the object to be freed.
 * @Return	IMG_SUCCESS or an error code.
 */
int pool_free(struct pool * const pool_arg,
	      void * const obj_hndl)
{
	struct object *object = NULL;
	unsigned int result = IMG_ERROR_FATAL;

	if (!pool_arg || !obj_hndl) {
		result = IMG_ERROR_INVALID_PARAMETERS;
		return result;
	}

	object = (struct object *)obj_hndl;
	object->next = pool_arg->objects;
	pool_arg->objects = object;

	result = IMG_SUCCESS;

	return result;
}
