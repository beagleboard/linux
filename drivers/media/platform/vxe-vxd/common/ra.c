// SPDX-License-Identifier: GPL-2.0
/*
 * Implements generic resource allocation.
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
#include "ra.h"

static unsigned char	global_init;

/* pool of struct arena's */
static struct pool *global_pool_arena;

/* pool of struct boundary tag */
static struct pool *global_pool_bt;

/**
 * ra_request_alloc_fail - ra_request_alloc_fail
 * @import_hdnl : Callback handle.
 * @requested_size : Requested allocation size.
 * @ref : Pointer to user reference data.
 * @alloc_flags : Allocation flags.
 * @actual_size : Pointer to contain the actual allocated size.
 * @base_addr : Allocation base(always 0,it is failing).
 *
 * Default callback allocator used if no callback is specified, always fails
 * to allocate further resources to the arena.
 */
static int ra_request_alloc_fail(void *import_hdnl,
				 unsigned long long requested_size,
				 unsigned long long *actual_size,
				 void **ref,
				 unsigned int alloc_flags,
				 unsigned long long *base_addr)
{
	if (base_addr)
		*base_addr = 0;

	return IMG_SUCCESS;
}

/*
 * @Function	ra_log2
 * @Description
 * Calculates the Log2(n) with n being a 64-bit value.
 *
 * @Input	value : Input value.
 * @Output	None
 * @Return	result : Log2(ui64Value).
 */

static unsigned int ra_log2(unsigned long long value)
{
	int res = 0;

	value >>= 1;
	while (value > 0) {
		value >>= 1;
		res++;
	}
	return res;
}

/*
 * @Function	ra_segment_list_insert_after
 * @Description	Insert a boundary tag into an arena segment list after a
 *		specified boundary tag.
 * @Input	arena_arg : Pointer to the input arena.
 * @Input	bt_here_arg : The boundary tag before which psBTToInsert
 *		will be added .
 * @Input	bt_to_insert_arg : The boundary tag to insert.
 * @Output	None
 * @Return	None
 */
static void ra_segment_list_insert_after(struct arena *arena_arg,
					 struct btag *bt_here_arg,
					 struct btag *bt_to_insert_arg)
{
	bt_to_insert_arg->nxt_seg = bt_here_arg->nxt_seg;
	bt_to_insert_arg->prv_seg = bt_here_arg;

	if (!bt_here_arg->nxt_seg)
		arena_arg->tail_seg = bt_to_insert_arg;
	else
		bt_here_arg->nxt_seg->prv_seg = bt_to_insert_arg;

	bt_here_arg->nxt_seg = bt_to_insert_arg;
}

/*
 * @Function	ra_segment_list_insert
 * @Description
 * Insert a boundary tag into an arena segment list at the appropriate point.
 * @Input	arena_arg : Pointer to the input arena.
 * @Input	bt_to_insert_arg : The boundary tag to insert.
 * @Output	None
 * @Return	None
 */
static void ra_segment_list_insert(struct arena *arena_arg,
				   struct btag *bt_to_insert_arg)
{
	/* insert into the segment chain */
	if (!arena_arg->head_seg) {
		arena_arg->head_seg = bt_to_insert_arg;
		arena_arg->tail_seg = bt_to_insert_arg;
		bt_to_insert_arg->nxt_seg = NULL;
		bt_to_insert_arg->prv_seg = NULL;
	} else {
		struct btag *bt_scan = arena_arg->head_seg;

		while (bt_scan->nxt_seg &&
		       bt_to_insert_arg->base >=
		       bt_scan->nxt_seg->base) {
			bt_scan = bt_scan->nxt_seg;
		}
		ra_segment_list_insert_after(arena_arg,
					     bt_scan,
					     bt_to_insert_arg);
	}
}

/*
 * @Function	ra_SegmentListRemove
 * @Description
 * Insert a boundary tag into an arena segment list at the appropriate point.
 * @Input	arena_arg : Pointer to the input arena.
 * @Input	bt_to_remove_arg : The boundary tag to insert.
 * @Output	None
 * @Return	None
 */
static void ra_segment_list_remove(struct arena *arena_arg,
				   struct btag *bt_to_remove_arg)
{
	if (!bt_to_remove_arg->prv_seg)
		arena_arg->head_seg = bt_to_remove_arg->nxt_seg;
	else
		bt_to_remove_arg->prv_seg->nxt_seg = bt_to_remove_arg->nxt_seg;

	if (!bt_to_remove_arg->nxt_seg)
		arena_arg->tail_seg = bt_to_remove_arg->prv_seg;
	else
		bt_to_remove_arg->nxt_seg->prv_seg = bt_to_remove_arg->prv_seg;
}

/*
 * @Function	ra_segment_split
 * @Description
 * Split a segment into two, maintain the arena segment list.
 * The boundary tag should not be in the free table. Neither the original or
 * the new psBTNeighbour bounary tag will be in the free table.
 * @Input	arena_arg : Pointer to the input arena.
 * @Input	bt_to_split_arg : The boundary tag to split.
 *		The required segment size of boundary tag after the split.
 * @Output	None
 * @Return	btag *: New boundary tag.
 */
static struct btag *ra_segment_split(struct arena *arena_arg,
				     struct btag *bt_to_split_arg,
				     unsigned long long size)
{
	struct btag *local_bt_neighbour = NULL;
	int res = IMG_ERROR_FATAL;

	res = pool_alloc(global_pool_bt, ((void **)&local_bt_neighbour));
	if (res != IMG_SUCCESS)
		return NULL;

	local_bt_neighbour->prv_seg = bt_to_split_arg;
	local_bt_neighbour->nxt_seg = bt_to_split_arg->nxt_seg;
	local_bt_neighbour->bt_type = RA_BOUNDARY_TAG_TYPE_FREE;
	local_bt_neighbour->size = (bt_to_split_arg->size - size);
	local_bt_neighbour->base = (bt_to_split_arg->base + size);
	local_bt_neighbour->nxt_free = NULL;
	local_bt_neighbour->prv_free = NULL;
	local_bt_neighbour->ref = bt_to_split_arg->ref;

	if (!bt_to_split_arg->nxt_seg)
		arena_arg->tail_seg = local_bt_neighbour;
	else
		bt_to_split_arg->nxt_seg->prv_seg = local_bt_neighbour;

	bt_to_split_arg->nxt_seg = local_bt_neighbour;
	bt_to_split_arg->size = size;

	return local_bt_neighbour;
}

/*
 * @Function	ra_free_list_insert
 * @Description
 * Insert a boundary tag into an arena free table.
 * @Input	arena_arg : Pointer to the input arena.
 * @Input	bt_arg : The boundary tag to insert into an arena
 *		free table.
 * @Output	None
 * @Return	None
 */
static void ra_free_list_insert(struct arena *arena_arg,
				struct btag *bt_arg)
{
	unsigned int index = ra_log2(bt_arg->size);

	bt_arg->bt_type = RA_BOUNDARY_TAG_TYPE_FREE;
	if (index < FREE_TABLE_LIMIT)
		bt_arg->nxt_free = arena_arg->head_free[index];
	else
		bt_arg->nxt_free = NULL;

	bt_arg->prv_free = NULL;

	if (index < FREE_TABLE_LIMIT) {
		if (arena_arg->head_free[index])
			arena_arg->head_free[index]->prv_free = bt_arg;
	}

	if (index < FREE_TABLE_LIMIT)
		arena_arg->head_free[index] = bt_arg;
}

/*
 * @Function	ra_free_list_remove
 * @Description
 * Remove a boundary tag from an arena free table.
 * @Input	arena_arg : Pointer to the input arena.
 * @Input	bt_arg    : The boundary tag to remove from
 *		an arena free table.
 * @Output	None
 * @Return	None
 */
static void ra_free_list_remove(struct arena *arena_arg,
				struct btag *bt_arg)
{
	unsigned int index = ra_log2(bt_arg->size);

	if (bt_arg->nxt_free)
		bt_arg->nxt_free->prv_free = bt_arg->prv_free;

	if (!bt_arg->prv_free && index < FREE_TABLE_LIMIT)
		arena_arg->head_free[index] = bt_arg->nxt_free;
	else if (bt_arg->prv_free)
		bt_arg->prv_free->nxt_free = bt_arg->nxt_free;
}

/*
 * @Function	ra_build_span_marker
 * @Description
 * Construct a span marker boundary tag.
 * @Input	base : The base of the boundary tag.
 * @Output	None
 * @Return    btag * : New span marker boundary tag
 */
static struct btag *ra_build_span_marker(unsigned long long base)
{
	struct btag *local_bt = NULL;
	int res = IMG_ERROR_FATAL;

	res = pool_alloc(global_pool_bt, ((void **)&local_bt));
	if (res != IMG_SUCCESS)
		return NULL;

	local_bt->bt_type = RA_BOUNDARY_TAG_TYPE_SPAN;
	local_bt->base = base;
	local_bt->size = 0;
	local_bt->nxt_seg = NULL;
	local_bt->prv_seg = NULL;
	local_bt->nxt_free = NULL;
	local_bt->prv_free = NULL;
	local_bt->ref = NULL;

	return local_bt;
}

/*
 * @Function	ra_build_bt
 * @Description
 * Construct a boundary tag for a free segment.
 * @Input	    ui64Base    : The base of the resource segment.
 * @Input	    ui64Size    : The extent of the resource segment.
 * @Output	None
 * @Return    btag *       : New boundary tag
 */
static struct btag *ra_build_bt(unsigned long long base, unsigned long long size)
{
	struct btag *local_bt = NULL;
	int res = IMG_ERROR_FATAL;

	res = pool_alloc(global_pool_bt, ((void **)&local_bt));

	if (res != IMG_SUCCESS)
		return local_bt;

	local_bt->bt_type = RA_BOUNDARY_TAG_TYPE_FREE;
	local_bt->base = base;
	local_bt->size = size;
	local_bt->nxt_seg = NULL;
	local_bt->prv_seg = NULL;
	local_bt->nxt_free = NULL;
	local_bt->prv_free = NULL;
	local_bt->ref = NULL;

	return local_bt;
}

/*
 * @Function	ra_insert_resource
 * @Description
 * Add a free resource segment to an arena.
 * @Input	base : The base of the resource segment.
 * @Input	size : The size of the resource segment.
 * @Output	None
 * @Return	IMG_SUCCESS or an error code.
 */
static int ra_insert_resource(struct arena *arena_arg,
			      unsigned long long base,
			      unsigned long long size)
{
	struct btag *local_bt = NULL;

	local_bt = ra_build_bt(base, size);
	if (!local_bt)
		return IMG_ERROR_UNEXPECTED_STATE;

	ra_segment_list_insert(arena_arg, local_bt);
	ra_free_list_insert(arena_arg, local_bt);
	arena_arg->max_idx = ra_log2(size);
	if (1ULL << arena_arg->max_idx < size)
		arena_arg->max_idx++;

	return IMG_SUCCESS;
}

/*
 * @Function	ra_insert_resource_span
 * @Description
 * Add a free resource span to an arena, complete with span markers.
 * @Input	arena_arg : Pointer to the input arena.
 * @Input	base : The base of the resource segment.
 * @Input	size : The size of the resource segment.
 * @Output	None
 * @Return	btag * : The boundary tag representing
 *		the free resource segment.
 */
static struct btag *ra_insert_resource_span(struct arena *arena_arg,
					    unsigned long long base,
					    unsigned long long size)
{
	struct btag *local_bt = NULL;
	struct btag *local_bt_span_start = NULL;
	struct btag *local_bt_span_end = NULL;

	local_bt_span_start = ra_build_span_marker(base);
	if (!local_bt_span_start)
		return NULL;

	local_bt_span_end = ra_build_span_marker(base + size);
	if (!local_bt_span_end) {
		pool_free(global_pool_bt, local_bt_span_start);
		return NULL;
	}

	local_bt = ra_build_bt(base, size);
	if (!local_bt) {
		pool_free(global_pool_bt, local_bt_span_end);
		pool_free(global_pool_bt, local_bt_span_start);
		return NULL;
	}

	ra_segment_list_insert(arena_arg, local_bt_span_start);
	ra_segment_list_insert_after(arena_arg,
				     local_bt_span_start,
				     local_bt);
	ra_free_list_insert(arena_arg, local_bt);
	ra_segment_list_insert_after(arena_arg,
				     local_bt,
				     local_bt_span_end);

	return local_bt;
}

/*
 * @Function	ra_free_bt
 * @Description
 * Free a boundary tag taking care of the segment list and the
 *		boundary tag free table.
 * @Input	arena_arg : Pointer to the input arena.
 * @Input	bt_arg : The boundary tag to free.
 * @Output	None
 * @Return	None
 */
static void ra_free_bt(struct arena *arena_arg,
		       struct btag *bt_arg)
{
	struct btag *bt_neibr;

	/* try and coalesce with left bt_neibr */
	bt_neibr = bt_arg->prv_seg;
	if (bt_neibr &&
	    bt_neibr->bt_type == RA_BOUNDARY_TAG_TYPE_FREE &&
	    bt_neibr->base + bt_neibr->size == bt_arg->base) {
		ra_free_list_remove(arena_arg, bt_neibr);
		ra_segment_list_remove(arena_arg, bt_neibr);
		bt_arg->base = bt_neibr->base;
		bt_arg->size += bt_neibr->size;
		pool_free(global_pool_bt, bt_neibr);
	}

	/* try to coalesce with right psBTNeighbour */
	bt_neibr = bt_arg->nxt_seg;
	if (bt_neibr &&
	    bt_neibr->bt_type == RA_BOUNDARY_TAG_TYPE_FREE &&
	    bt_arg->base + bt_arg->size == bt_neibr->base) {
		ra_free_list_remove(arena_arg, bt_neibr);
		ra_segment_list_remove(arena_arg, bt_neibr);
		bt_arg->size += bt_neibr->size;
		pool_free(global_pool_bt, bt_neibr);
	}

	if (bt_arg->nxt_seg &&
	    bt_arg->nxt_seg->bt_type == RA_BOUNDARY_TAG_TYPE_SPAN &&
	    bt_arg->prv_seg && bt_arg->prv_seg->bt_type ==
	    RA_BOUNDARY_TAG_TYPE_SPAN) {
		struct btag *ps_bt_nxt = bt_arg->nxt_seg;
		struct btag *ps_bt_prev = bt_arg->prv_seg;

		ra_segment_list_remove(arena_arg, ps_bt_nxt);
		ra_segment_list_remove(arena_arg, ps_bt_prev);
		ra_segment_list_remove(arena_arg, bt_arg);
		arena_arg->import_free_fxn(arena_arg->import_hdnl,
					   bt_arg->base,
					   bt_arg->ref);
		pool_free(global_pool_bt, ps_bt_nxt);
		pool_free(global_pool_bt, ps_bt_prev);
		pool_free(global_pool_bt, bt_arg);
	} else {
		ra_free_list_insert(arena_arg, bt_arg);
	}
}

static int ra_check_btag(struct arena *arena_arg,
			 unsigned long long size_arg,
			 void **ref,
			 struct btag *bt_arg,
			 unsigned long long align_arg,
			 unsigned long long *base_arg,
			 unsigned int align_log2)
{
	unsigned long long local_align_base;
	int res = IMG_ERROR_FATAL;

	while (bt_arg) {
		if (align_arg > 1ULL)
			local_align_base = ((bt_arg->base + align_arg - 1)
				>> align_log2) << align_log2;
		else
			local_align_base = bt_arg->base;

		if ((bt_arg->base + bt_arg->size) >=
		     (local_align_base + size_arg)) {
			ra_free_list_remove(arena_arg, bt_arg);

			/*
			 * with align_arg we might need to discard the front of
			 * this segment
			 */
			if (local_align_base > bt_arg->base) {
				struct btag *btneighbor;

				btneighbor = ra_segment_split(arena_arg,
							      bt_arg,
							      (local_align_base -
							       bt_arg->base));
				/*
				 * Partition the buffer, create a new boundary
				 * tag
				 */
				if (!btneighbor)
					return IMG_ERROR_UNEXPECTED_STATE;

				ra_free_list_insert(arena_arg, bt_arg);
				bt_arg = btneighbor;
			}

			/*
			 * The segment might be too big, if so, discard the back
			 * of the segment
			 */
			if (bt_arg->size > size_arg) {
				struct btag *btneighbor;

				btneighbor = ra_segment_split(arena_arg,
							      bt_arg,
							      size_arg);
				/*
				 * Partition the buffer, create a new boundary
				 * tag
				 */
				if (!btneighbor)
					return IMG_ERROR_UNEXPECTED_STATE;

				ra_free_list_insert(arena_arg, btneighbor);
			}

			bt_arg->bt_type = RA_BOUNDARY_TAG_TYPE_LIVE;

			res = vid_hash_insert(arena_arg->hash_tbl,
					      bt_arg->base,
					      (unsigned long)bt_arg);
			if (res != IMG_SUCCESS) {
				ra_free_bt(arena_arg, bt_arg);
				*base_arg = 0;
				return IMG_ERROR_UNEXPECTED_STATE;
			}

			if (ref)
				*ref = bt_arg->ref;

			*base_arg = bt_arg->base;
			return IMG_SUCCESS;
		}
		bt_arg = bt_arg->nxt_free;
	}

	return res;
}

/*
 * @Function	ra_attempt_alloc_aligned
 * @Description	Attempt to allocate from an arena
 * @Input	arena_arg: Pointer to the input arena
 * @Input	size_arg: The requested allocation size
 * @Input	ref: The user references associated with the allocated
 *		segment
 * @Input	align_arg: Required alignment
 * @Output	base_arg: Allocated resource size
 * @Return	IMG_SUCCESS or an error code
 */
static int ra_attempt_alloc_aligned(struct arena *arena_arg,
				    unsigned long long size_arg,
				    void **ref,
				    unsigned long long align_arg,
				    unsigned long long *base_arg)
{
	unsigned int index;
	unsigned int align_log2;
	int res = IMG_ERROR_FATAL;

	if (!arena_arg || !base_arg)
		return IMG_ERROR_INVALID_PARAMETERS;

	/*
	 * Take the log of the alignment to get number of bits to shift
	 * left/right for multiply/divide. Assumption made here is that
	 * alignment has to be a power of 2 value. Aserting otherwise.
	 */
	align_log2 = ra_log2(align_arg);

	/*
	 * Search for a near fit free boundary tag, start looking at the
	 * log2 free table for our required size and work on up the table.
	 */
	index = ra_log2(size_arg);

	/*
	 * If the Size required is exactly 2**n then use the n bucket, because
	 * we know that every free block in that bucket is larger than 2**n,
	 * otherwise start at then next bucket up.
	 */
	if (size_arg > (1ull << index))
		index++;

	while ((index < FREE_TABLE_LIMIT) && !arena_arg->head_free[index])
		index++;

	if (index >= FREE_TABLE_LIMIT) {
		pr_err("requested allocation size doesn't fit in the arena. Increase MMU HEAP Size\n");
		return IMG_ERROR_OUT_OF_MEMORY;
	}

	while (index < FREE_TABLE_LIMIT) {
		if (arena_arg->head_free[index]) {
			/* we have a cached free boundary tag */
			struct btag *local_bt =
				arena_arg->head_free[index];

			res = ra_check_btag(arena_arg,
					    size_arg,
					    ref,
					    local_bt,
					    align_arg,
					    base_arg,
					    align_log2);
			if (res != IMG_SUCCESS)
				return res;
		}
		index++;
	}

	return IMG_SUCCESS;
}

/*
 * @Function	vid_ra_init
 * @Description	Initializes the RA module. Must be called before any other
 *		ra API function
 * @Return	IMG_SUCCESS or an error code
 *
 */
int vid_ra_initialise(void)
{
	int res = IMG_ERROR_FATAL;

	if (!global_init) {
		res = pool_create("img-arena",
				  sizeof(struct arena),
				  &global_pool_arena);
		if (res != IMG_SUCCESS)
			return IMG_ERROR_UNEXPECTED_STATE;

		res = pool_create("img-bt",
				  sizeof(struct btag),
				  &global_pool_bt);
		if (res != IMG_SUCCESS) {
			res = pool_delete(global_pool_arena);
			global_pool_arena = NULL;
			return IMG_ERROR_UNEXPECTED_STATE;
		}
		global_init = 1;
		res = IMG_SUCCESS;
	}

	return res;
}

/*
 * @Function	vid_ra_deinit
 * @Description	Deinitializes the RA module
 * @Return	IMG_SUCCESS or an error code
 *
 */
int vid_ra_deinit(void)
{
	int res = IMG_ERROR_FATAL;

	if (global_init) {
		if (global_pool_arena) {
			res = pool_delete(global_pool_arena);
			global_pool_arena = NULL;
		}
		if (global_pool_bt) {
			res = pool_delete(global_pool_bt);
			global_pool_bt = NULL;
		}
		global_init = 0;
		res = IMG_SUCCESS;
	}
	return res;
}

/*
 * @Function	vid_ra_create
 * @Description	Used to create a resource arena.
 * @Input	name: The name of the arena for diagnostic purposes
 * @Input	base_arg: The base of an initial resource span or 0
 * @Input	size_arg: The size of an initial resource span or 0
 * @Input	quantum: The arena allocation quantum
 * @Input	(*import_alloc_fxn): A resource allocation callback or NULL
 * @Input	(*import_free_fxn): A resource de-allocation callback or NULL
 * @Input	import_hdnl: Handle passed to alloc and free or NULL
 * @Output	arena_hndl: The handle for the arene being created, or NULL
 * @Return	IMG_SUCCESS or an error code
 *
 */
int vid_ra_create(const unsigned char * const name,
		  unsigned long long base_arg,
		  unsigned long long size_arg,
		  unsigned long	quantum,
		  int (*import_alloc_fxn)(void * const import_hdnl,
					  unsigned long long req_sz,
					  unsigned long long * const actl_sz,
					  void ** const ref,
					  unsigned int alloc_flags,
					  unsigned long long * const base_arg),
		  int (*import_free_fxn)(void * const import_hdnl,
					 unsigned long long import_base,
					 void * const import_ref),
					 void *import_hdnl,
					 void **arena_hndl)
{
	struct arena *local_arena = NULL;
	unsigned int idx = 0;
	int res = IMG_ERROR_FATAL;

	if (!arena_hndl)
		return IMG_ERROR_INVALID_PARAMETERS;

	*(arena_hndl) = NULL;

	if (global_init) {
		res = pool_alloc(global_pool_arena, ((void **)&local_arena));
		if (!local_arena || res != IMG_SUCCESS)
			return IMG_ERROR_UNEXPECTED_STATE;

		local_arena->name = NULL;
		if (name)
			local_arena->name = kstrdup((const signed char *)name,
						    GFP_KERNEL);
		if (import_alloc_fxn)
			local_arena->import_alloc_fxn = import_alloc_fxn;
		else
			local_arena->import_alloc_fxn = ra_request_alloc_fail;

		local_arena->import_free_fxn = import_free_fxn;
		local_arena->import_hdnl = import_hdnl;

		for (idx = 0; idx < FREE_TABLE_LIMIT; idx++)
			local_arena->head_free[idx] = NULL;

		local_arena->head_seg = NULL;
		local_arena->tail_seg = NULL;
		local_arena->quantum = quantum;

		res = vid_hash_create(MINIMUM_HASH_SIZE,
				      &local_arena->hash_tbl);

		if (!local_arena->hash_tbl) {
			vid_hash_delete(local_arena->hash_tbl);
			kfree(local_arena->name);
			local_arena->name = NULL;
			return IMG_ERROR_UNEXPECTED_STATE;
		}

		//if (size_arg > (unsigned long long)0) {
		if (size_arg > 0ULL) {
			size_arg = (size_arg + quantum - 1) / quantum * quantum;

			res = ra_insert_resource(local_arena,
						 base_arg,
						 size_arg);
			if (res != IMG_SUCCESS) {
				vid_hash_delete(local_arena->hash_tbl);
				pool_free(global_pool_arena, local_arena);
				kfree(local_arena->name);
				local_arena->name = NULL;
				return IMG_ERROR_UNEXPECTED_STATE;
			}
		}
		*(arena_hndl) = local_arena;
		res = IMG_SUCCESS;
	}

	return res;
}

/*
 * @Function	vid_ra_delete
 * @Description	Used to delete a resource arena. All resources allocated from
 *		the arena must be freed before deleting the arena
 * @Input	arena_hndl: The handle to the arena to delete
 * @Return	IMG_SUCCESS or an error code
 *
 */
int vid_ra_delete(void * const arena_hndl)
{
	int res = IMG_ERROR_FATAL;
	struct arena *local_arena = NULL;
	unsigned int idx;

	if (!arena_hndl)
		return IMG_ERROR_INVALID_PARAMETERS;

	if (global_init) {
		local_arena = (struct arena *)arena_hndl;
		kfree(local_arena->name);
		local_arena->name = NULL;
		for (idx = 0; idx < FREE_TABLE_LIMIT; idx++)
			local_arena->head_free[idx] = NULL;

		while (local_arena->head_seg) {
			struct btag *local_bt = local_arena->head_seg;

			ra_segment_list_remove(local_arena, local_bt);
		}
		res = vid_hash_delete(local_arena->hash_tbl);
		if (res != IMG_SUCCESS)
			return IMG_ERROR_UNEXPECTED_STATE;

		res = pool_free(global_pool_arena, local_arena);
		if (res != IMG_SUCCESS)
			return IMG_ERROR_UNEXPECTED_STATE;
	}

	return res;
}

/*
 * @Function	vid_ra_add
 * @Description	Used to add a resource span to an arena. The span must not
 *		overlap with any span previously added to the arena
 * @Input	base_arg: The base_arg of the span
 * @Input	size_arg: The size of the span
 * @Return	IMG_SUCCESS or an error code
 *
 */
int vid_ra_add(void * const arena_hndl, unsigned long long base_arg, unsigned long long size_arg)
{
	int res = IMG_ERROR_FATAL;
	struct arena *local_arena = NULL;

	if (!arena_hndl)
		return IMG_ERROR_INVALID_PARAMETERS;

	if (global_init) {
		local_arena = (struct arena *)arena_hndl;
		size_arg = (size_arg + local_arena->quantum - 1) /
			local_arena->quantum * local_arena->quantum;

		res = ra_insert_resource(local_arena, base_arg, size_arg);
		if (res != IMG_SUCCESS)
			return IMG_ERROR_INVALID_PARAMETERS;
	}

	return res;
}

/*
 * @Function	vid_ra_alloc
 * @Description	Used to allocate resource from an arena
 * @Input	arena_hndl: The handle to the arena to create the resource
 * @Input	request_size: The requested size of resource segment
 * @Input	actl_size: The actualSize of resource segment
 * @Input	ref: The user reference associated with allocated resource
 *		span
 * @Input	alloc_flags: AllocationFlags influencing allocation policy
 * @Input	align_arg: The alignment constraint required for the allocated
 *		segment
 * @Output	base_args: The base of the allocated resource
 * @Return	IMG_SUCCESS or an error code
 *
 */
int vid_ra_alloc(void * const arena_hndl,
		 unsigned long long request_size,
		 unsigned long long * const actl_sz,
		 void ** const ref,
		 unsigned int alloc_flags,
		 unsigned long long alignarg,
		 unsigned long long * const basearg)
{
	int res = IMG_ERROR_FATAL;
	struct arena *arn_ctx = NULL;
	unsigned long long loc_size = request_size;

	if (!arena_hndl)
		return IMG_ERROR_INVALID_PARAMETERS;

	if (global_init) {
		arn_ctx = (struct arena *)arena_hndl;
		loc_size = ((loc_size + arn_ctx->quantum - 1) /
				arn_ctx->quantum) * arn_ctx->quantum;

		if (actl_sz)
			*actl_sz = loc_size;

		/*
		 * If allocation failed then we might have an import source
		 * which can provide more resource, else we will have to fail
		 * the allocation to the caller.
		 */
		if (alloc_flags == RA_SEQUENTIAL_ALLOCATION)
			res = ra_attempt_alloc_aligned(arn_ctx,
						       loc_size,
						       ref,
						       alignarg,
						       basearg);

		if (res != IMG_SUCCESS) {
			void *import_ref = NULL;
			unsigned long long import_base = 0ULL;
			unsigned long long locimprt_reqsz = loc_size;
			unsigned long long locimprt_actsz = 0ULL;

			res = arn_ctx->import_alloc_fxn(arn_ctx->import_hdnl,
					locimprt_reqsz,
					&locimprt_actsz,
					&import_ref,
					alloc_flags,
					&import_base);

			if (res == IMG_SUCCESS) {
				struct btag *local_bt =
					ra_insert_resource_span(arn_ctx,
								import_base,
								locimprt_actsz);

				/*
				 * Successfully import more resource, create a
				 * span to represent it and retry the allocation
				 * attempt
				 */
				if (!local_bt) {
					/*
					 * Insufficient resources to insert the
					 * newly acquired span, so free it back
					 */
					arn_ctx->import_free_fxn(arn_ctx->import_hdnl,
							import_base,
							import_ref);
					return IMG_ERROR_UNEXPECTED_STATE;
				}
				local_bt->ref = import_ref;
				if (alloc_flags == RA_SEQUENTIAL_ALLOCATION) {
					res = ra_attempt_alloc_aligned(arn_ctx,
								       loc_size,
								       ref,
								       alignarg,
								       basearg);
				}
			}
		}
	}

	return res;
}

/*
 * @Function	vid_ra_free
 * @Description	Used to free a resource segment
 * @Input	arena_hndl: The arena the segment was originally allocated from
 * @Input	base_arg: The base of the span
 * @Return	IMG_SUCCESS or an error code
 *
 */
int vid_ra_free(void * const arena_hndl, unsigned long long base_arg)
{
	int res = IMG_ERROR_FATAL;
	struct arena *local_arena = NULL;
	struct btag *local_bt = NULL;
	unsigned long	uip_res;

	if (!arena_hndl)
		return IMG_ERROR_INVALID_PARAMETERS;

	if (global_init) {
		local_arena = (struct arena *)arena_hndl;

		res = vid_hash_remove(local_arena->hash_tbl,
				      base_arg,
				      &uip_res);
		if (res != IMG_SUCCESS)
			return res;
		local_bt = (struct btag *)uip_res;

		ra_free_bt(local_arena, local_bt);
	}

	return res;
}
