/* SPDX-License-Identifier: GPL-2.0 */
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
 *      Sidraya Jayagond <sidraya.bj@pathpartnertech.com>
 */
#ifndef _RA_H_
#define _RA_H_

#define MINIMUM_HASH_SIZE  (64)
#define FREE_TABLE_LIMIT   (64)

/* Defines whether sequential or random allocation is used */
enum {
	RA_SEQUENTIAL_ALLOCATION = 0,
	RA_RANDOM_ALLOCATION,
	RA_FORCE32BITS = 0x7FFFFFFFU
};

/* Defines boundary tag type */
enum eboundary_tag_type {
	RA_BOUNDARY_TAG_TYPE_SPAN = 0,
	RA_BOUNDARY_TAG_TYPE_FREE,
	RA_BOUNDARY_TAG_TYPE_LIVE,
	RA_BOUNDARY_TAG_TYPE_MAX,
	RA_BOUNDARY_FORCE32BITS = 0x7FFFFFFFU
};

/*
 * @Description
 * Boundary tags, used to describe a resource segment
 *
 * @enum0: span markers
 * @enum1: free resource segment
 * @enum2: allocated resource segment
 * @enum3: max
 * @base,size: The base resource of this segment and extent of this segment
 * @nxt_seg, prv_seg: doubly linked ordered list of all segments
 *			within the arena
 * @nxt_free, prv_free: doubly linked un-ordered list of free segments
 * @reference : a user reference associated with this span, user
 *		references are currently only provided in
 *		the callback mechanism
 */
struct btag {
	unsigned int bt_type;
	unsigned long long base;
	unsigned long long size;
	struct btag *nxt_seg;
	struct btag *prv_seg;
	struct btag *nxt_free;
	struct btag *prv_free;
	void *ref;
};

/*
 * @Description
 * resource allocation arena
 *
 * @name: arena for diagnostics output
 * @quantum: allocations within this arena are quantum sized
 * @max_idx: index of the last position in the psBTHeadFree table,
 *		with available free space
 * @import_alloc_fxn: import interface, if provided
 * @import_free_fxn: import interface, if provided
 * @import_hdnl: import interface, if provided
 * @head_free: head of list of free boundary tags for indexed by Log2
 *		of the boundary tag size. Power-of-two table of free lists
 * @head_seg, tail_seg : resource ordered segment list
 * @ps_hash : segment address to boundary tag hash table
 */
struct arena {
	unsigned char *name;
	unsigned long quantum;
	unsigned int max_idx;
	int (*import_alloc_fxn)(void *import_hdnl,
				unsigned long long requested_size,
				unsigned long long *actual_size,
				void **ref,
				unsigned int alloc_flags,
				unsigned long long *base_addr);
	int (*import_free_fxn)(void *import_hdnl,
			       unsigned long long base,
			       void *ref);
	void *import_hdnl;
	struct btag *head_free[FREE_TABLE_LIMIT];
	struct btag *head_seg;
	struct btag *tail_seg;
	struct hash *hash_tbl;
};

/*
 * @Function	vid_ra_init
 * @Description	Initializes the RA module. Must be called before any other
 *		ra API function
 * @Return	IMG_SUCCESS or an error code
 *
 */
int vid_ra_initialise(void);

/*
 * @Function	vid_ra_deinit
 * @Description	Deinitializes the RA module
 * @Return	IMG_SUCCESS or an error code
 *
 */
int vid_ra_deinit(void);

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
		  unsigned long quantum,
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
		  void **arena_hndl);

/*
 * @Function	vid_ra_delete
 * @Description	Used to delete a resource arena. All resources allocated from
 *		the arena must be freed before deleting the arena
 * @Input	arena_hndl: The handle to the arena to delete
 * @Return	IMG_SUCCESS or an error code
 *
 */
int vid_ra_delete(void * const arena_hndl);

/*
 * @Function	vid_ra_add
 * @Description	Used to add a resource span to an arena. The span must not
 *		overlap with any span previously added to the arena
 * @Input	base_arg: The base_arg of the span
 * @Input	size_arg: The size of the span
 * @Return	IMG_SUCCESS or an error code
 *
 */
int vid_ra_add(void * const arena_hndl, unsigned long long base_arg, unsigned long long size_arg);

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
		 unsigned long long align_arg,
		 unsigned long long * const base_arg);

/*
 * @Function	vid_ra_free
 * @Description	Used to free a resource segment
 * @Input	arena_hndl: The arena the segment was originally allocated from
 * @Input	base_arg: The base of the span
 * @Return	IMG_SUCCESS or an error code
 *
 */
int vid_ra_free(void * const arena_hndl, unsigned long long base_arg);

#endif
