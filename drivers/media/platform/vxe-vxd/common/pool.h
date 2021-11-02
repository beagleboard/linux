/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Object Pool Memory Allocator header
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
#ifndef _pool_h_
#define _pool_h_

#include <linux/types.h>

struct pool;

/**
 * pool_create - Create an sObject pool
 * @name: Name of sObject pool for diagnostic purposes
 * @obj_size: size of each sObject in the pool in bytes
 * @pool: Will contain NULL or sObject pool handle
 *
 * Return	IMG_SUCCESS or an error code.
 */
int pool_create(const unsigned char * const name,
		unsigned int obj_size,
		struct pool ** const pool);

/*
 * @Function	pool_delete
 * @Description
 * Delete an sObject pool. All psObjects allocated from the pool must
 * be free'd with pool_free() before deleting the sObject pool.
 * @Input	pool : Object Pool pointer
 * @Return IMG_SUCCESS or an error code.
 */
int pool_delete(struct pool * const pool);

/*
 * @Function	pool_alloc
 * @Description
 * Allocate an Object from an Object pool.
 * @Input	pool : Object Pool
 * @Output	obj_hdnl : Pointer containing the handle to the
 * object created or IMG_NULL
 * @Return    IMG_SUCCESS or an error code.
 */
int pool_alloc(struct pool * const pool,
	       void ** const obj_hdnl);

/*
 * @Function	pool_free
 * @Description
 * Free an sObject previously allocated from an sObject pool.
 * @Input	pool : Object Pool pointer.
 * @Output	obj_hdnl : Handle to the object to be freed.
 * @Return	IMG_SUCCESS or an error code.
 */
int pool_free(struct pool * const pool,
	      void * const obj_hdnl);

#endif /* _pool_h_ */
