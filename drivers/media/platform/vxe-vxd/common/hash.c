// SPDX-License-Identifier: GPL-2.0
/*
 * Self scaling hash tables.
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

#include "hash.h"
#include "img_errors.h"
#include "pool.h"

/* pool of struct hash objects */
static struct pool *global_hashpool;

/* pool of struct bucket objects */
static struct pool *global_bucketpool;

static int global_initialized;

/* Each entry in a hash table is placed into a bucket */
struct bucket {
	struct bucket *next;
	unsigned long long key;
	unsigned long long value;
};

struct hash {
	struct bucket **table;
	unsigned int size;
	unsigned int count;
	unsigned int minimum_size;
};

/**
 * hash_func - Hash function intended for hashing addresses.
 * @vale : The key to hash.
 * @size : The size of the hash table
 */
static unsigned int hash_func(unsigned long long vale,
			      unsigned int size)
{
	unsigned int hash = (unsigned int)(vale);

	hash += (hash << 12);
	hash ^= (hash >> 22);
	hash += (hash << 4);
	hash ^= (hash >> 9);
	hash += (hash << 10);
	hash ^= (hash >> 2);
	hash += (hash << 7);
	hash ^= (hash >> 12);
	hash &= (size - 1);
	return hash;
}

/*
 * @Function	hash_chain_insert
 * @Description
 * Hash function intended for hashing addresses.
 * @Input	bucket : The bucket
 * @Input	table : The hash table
 * @Input	size : The size of the hash table
 * @Return	IMG_SUCCESS or an error code.
 */
static int hash_chain_insert(struct bucket *bucket,
			     struct bucket **table,
			     unsigned int size)
{
	unsigned int idx;
	unsigned int result = IMG_ERROR_FATAL;

	if (!bucket || !table || !size) {
		result = IMG_ERROR_INVALID_PARAMETERS;
		return result;
	}

	idx = hash_func(bucket->key, size);

	if (idx < size) {
		result = IMG_SUCCESS;
		bucket->next = table[idx];
		table[idx] = bucket;
	}

	return result;
}

/*
 * @Function	hash_rehash
 * @Description
 * Iterate over every entry in an old hash table and rehash into the new table.
 * @Input	old_table : The old hash table
 * @Input	old_size : The size of the old hash table
 * @Input	new_table : The new hash table
 * @Input	new_sz : The size of the new hash table
 * @Return	IMG_SUCCESS or an error code.
 */
static int hash_rehash(struct bucket **old_table,
		       unsigned int old_size,
		       struct bucket **new_table,
		       unsigned int new_sz)
{
	unsigned int idx;
	unsigned int result = IMG_ERROR_FATAL;

	if (!old_table || !new_table) {
		result = IMG_ERROR_INVALID_PARAMETERS;
		return result;
	}

	for (idx = 0; idx < old_size; idx++) {
		struct bucket *bucket;
		struct bucket *nex_bucket;

		bucket = old_table[idx];
		while (bucket) {
			nex_bucket = bucket->next;
			result = hash_chain_insert(bucket, new_table, new_sz);
			if (result != IMG_SUCCESS) {
				result = IMG_ERROR_UNEXPECTED_STATE;
				return result;
			}
			bucket = nex_bucket;
		}
	}
	result = IMG_SUCCESS;

	return result;
}

/*
 * @Function	hash_resize
 * @Description
 * Attempt to resize a hash table, failure to allocate a new larger hash table
 * is not considered a hard failure. We simply continue and allow the table to
 * fill up, the effect is to allow hash chains to become longer.
 * @Input	hash_arg : Pointer to the hash table
 * @Input	new_sz : The size of the new hash table
 * @Return	IMG_SUCCESS or an error code.
 */
static int hash_resize(struct hash *hash_arg,
		       unsigned int new_sz)
{
	unsigned int malloc_sz = 0;
	unsigned int result = IMG_ERROR_FATAL;
	unsigned int idx;

	if (!hash_arg) {
		result = IMG_ERROR_INVALID_PARAMETERS;
		return result;
	}

	if (new_sz != hash_arg->size) {
		struct bucket **new_bkt_table;

		malloc_sz = (sizeof(struct bucket *) * new_sz);
		new_bkt_table = kmalloc(malloc_sz, GFP_KERNEL);

		if (!new_bkt_table) {
			result = IMG_ERROR_MALLOC_FAILED;
			return result;
		}

		for (idx = 0; idx < new_sz; idx++)
			new_bkt_table[idx] = NULL;

		result = hash_rehash(hash_arg->table,
				     hash_arg->size,
				     new_bkt_table,
				     new_sz);

		if (result != IMG_SUCCESS) {
			kfree(new_bkt_table);
			new_bkt_table = NULL;
			result = IMG_ERROR_UNEXPECTED_STATE;
			return result;
		}

		kfree(hash_arg->table);
		hash_arg->table = new_bkt_table;
		hash_arg->size = new_sz;
	}
	result = IMG_SUCCESS;

	return result;
}

static unsigned int private_max(unsigned int a, unsigned int b)
{
	unsigned int ret = (a > b) ? a : b;
	return ret;
}

/*
 * @Function	vid_hash_initialise
 * @Description
 * To initialise the hash module.
 * @Input	None
 * @Return	IMG_SUCCESS or an error code.
 */
int vid_hash_initialise(void)
{
	unsigned int result = IMG_ERROR_ALREADY_COMPLETE;

	if (!global_initialized) {
		if (global_hashpool || global_bucketpool) {
			result = IMG_ERROR_UNEXPECTED_STATE;
			return result;
		}

		result = pool_create("img-hash",
				     sizeof(struct hash),
				     &global_hashpool);

		if (result != IMG_SUCCESS) {
			result = IMG_ERROR_UNEXPECTED_STATE;
			return result;
		}

		result = pool_create("img-sBucket",
				     sizeof(struct bucket),
				     &global_bucketpool);
		if (result != IMG_SUCCESS) {
			if (global_bucketpool) {
				result = pool_delete(global_bucketpool);
				global_bucketpool = NULL;
			}
			result = IMG_ERROR_UNEXPECTED_STATE;
			return result;
		}
		global_initialized = true;
		result = IMG_SUCCESS;
	}
	return result;
}

/*
 * @Function	vid_hash_finalise
 * @Description
 * To finalise the hash module. All allocated hash tables should
 * be deleted before calling this function.
 * @Input	None
 * @Return	IMG_SUCCESS or an error code.
 */
int vid_hash_finalise(void)
{
	unsigned int result = IMG_ERROR_FATAL;

	if (global_initialized) {
		if (global_hashpool) {
			result = pool_delete(global_hashpool);
			if (result != IMG_SUCCESS)
				return result;

			global_hashpool = NULL;
		}

		if (global_bucketpool) {
			result = pool_delete(global_bucketpool);
			if (result != IMG_SUCCESS)
				return result;

			global_bucketpool = NULL;
		}
		global_initialized = false;
		result = IMG_SUCCESS;
	}

	return result;
}

/*
 * @Function	vid_hash_create
 * @Description
 * Create a self scaling hash table.
 * @Input	initial_size : Initial and minimum size of the hash table.
 * @Output	hash_arg : Will countin the hash table handle or NULL.
 * @Return	IMG_SUCCESS or an error code.
 */
int vid_hash_create(unsigned int initial_size,
		    struct hash ** const hash_arg)
{
	unsigned int idx;
	unsigned int tbl_sz = 0;
	unsigned int result = IMG_ERROR_FATAL;
	struct hash *local_hash = NULL;

	if (!hash_arg) {
		result = IMG_ERROR_INVALID_PARAMETERS;
		return result;
	}

	if (global_initialized) {
		pool_alloc(global_hashpool, ((void **)&local_hash));
		if (!local_hash) {
			result = IMG_ERROR_UNEXPECTED_STATE;
			*hash_arg = NULL;
			return result;
		}

		local_hash->count = 0;
		local_hash->size = initial_size;
		local_hash->minimum_size = initial_size;

		tbl_sz = (sizeof(struct bucket *) * local_hash->size);
		local_hash->table = kmalloc(tbl_sz, GFP_KERNEL);
		if (!local_hash->table) {
			result = pool_free(global_hashpool, local_hash);
			if (result != IMG_SUCCESS)
				result = IMG_ERROR_UNEXPECTED_STATE;
			result |= IMG_ERROR_MALLOC_FAILED;
			*hash_arg = NULL;
			return result;
		}

		for (idx = 0; idx < local_hash->size; idx++)
			local_hash->table[idx] = NULL;

		*hash_arg = local_hash;
		result = IMG_SUCCESS;
	}
	return result;
}

/*
 * @Function	vid_hash_delete
 * @Description
 * To delete a hash table, all entries in the table should be
 * removed before calling this function.
 * @Input	hash_arg : Hash table pointer
 * @Return	IMG_SUCCESS or an error code.
 */
int vid_hash_delete(struct hash * const hash_arg)
{
	unsigned int result = IMG_ERROR_FATAL;

	if (!hash_arg) {
		result = IMG_ERROR_INVALID_PARAMETERS;
		return result;
	}

	if (global_initialized) {
		if (hash_arg->count != 0) {
			result = IMG_ERROR_UNEXPECTED_STATE;
			return result;
		}

		kfree(hash_arg->table);
		hash_arg->table = NULL;

		result = pool_free(global_hashpool, hash_arg);
		if (result != IMG_SUCCESS) {
			result = IMG_ERROR_UNEXPECTED_STATE;
			return result;
		}
	}
	return result;
}

/*
 * @Function	vid_hash_insert
 * @Description
 * To insert a key value pair into a hash table.
 * @Input	hash_arg : Hash table pointer
 * @Input	key : Key value
 * @Input	value : The value associated with the key.
 * @Return	IMG_SUCCESS or an error code.
 */
int vid_hash_insert(struct hash * const hash_arg,
		    unsigned long long key,
		    unsigned long long value)
{
	struct bucket *ps_bucket = NULL;
	unsigned int result = IMG_ERROR_FATAL;

	if (!hash_arg) {
		result = IMG_ERROR_INVALID_PARAMETERS;
		return result;
	}

	if (global_initialized) {
		result = pool_alloc(global_bucketpool, ((void **)&ps_bucket));
		if (result != IMG_SUCCESS || !ps_bucket) {
			result = IMG_ERROR_UNEXPECTED_STATE;
			return result;
		}
		ps_bucket->next = NULL;
		ps_bucket->key = key;
		ps_bucket->value = value;

		result = hash_chain_insert(ps_bucket,
					   hash_arg->table,
					   hash_arg->size);

		if (result != IMG_SUCCESS) {
			pool_free(global_bucketpool, ((void **)&ps_bucket));
			result = IMG_ERROR_UNEXPECTED_STATE;
			return result;
		}

		hash_arg->count++;

		/* check if we need to think about re-balancing */
		if ((hash_arg->count << 1) > hash_arg->size) {
			result = hash_resize(hash_arg, (hash_arg->size << 1));
			if (result != IMG_SUCCESS) {
				result = IMG_ERROR_UNEXPECTED_STATE;
				return result;
			}
		}
		result = IMG_SUCCESS;
	}
	return result;
}

/*
 * @Function	vid_hash_remove
 * @Description
 * To remove a key value pair from a hash table
 * @Input	hash_arg : Hash table pointer
 * @Input	key : Key value
 * @Input	ret_result : 0 if the key is missing or the value
 *		associated with the key.
 * @Return	IMG_SUCCESS or an error code.
 */
int vid_hash_remove(struct hash * const hash_arg,
		    unsigned long long key,
		    unsigned long * const ret_result)
{
	unsigned int idx;
	unsigned int tmp1 = 0;
	unsigned int tmp2 = 0;
	unsigned int result = IMG_ERROR_FATAL;
	struct bucket **bucket = NULL;

	if (!hash_arg) {
		result = IMG_ERROR_INVALID_PARAMETERS;
		return result;
	}

	idx = hash_func(key, hash_arg->size);

	for (bucket = &hash_arg->table[idx]; (*bucket) != NULL;
		bucket = &((*bucket)->next)) {
		if ((*bucket)->key == key) {
			struct bucket *ps_bucket = (*bucket);

			unsigned long long value = ps_bucket->value;

			*bucket = ps_bucket->next;
			result = pool_free(global_bucketpool, ps_bucket);

			hash_arg->count--;

			/* check if we need to think about re-balencing */
			if (hash_arg->size > (hash_arg->count << 2) &&
			    hash_arg->size > hash_arg->minimum_size) {
				tmp1 = (hash_arg->size >> 1);
				tmp2 = hash_arg->minimum_size;
				result = hash_resize(hash_arg,
						     private_max(tmp1, tmp2));
			}
			*ret_result = value;
			result = IMG_SUCCESS;
			break;
		}
	}
	return result;
}
