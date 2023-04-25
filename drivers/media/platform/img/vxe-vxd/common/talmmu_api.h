/* SPDX-License-Identifier: GPL-2.0 */
/*
 * TAL MMU Extensions.
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
#include "addr_alloc.h"
#include "ra.h"
#include "lst.h"

#ifndef __TALMMU_API_H__
#define __TALMMU_API_H__

#define	TALMMU_MAX_DEVICE_HEAPS	(32)
#define	TALMMU_MAX_TEMPLATES	(32)

/* MMU type */
enum talmmu_mmu_type {
	/* 4kb pages and 32-bit address range */
	TALMMU_MMUTYPE_4K_PAGES_32BIT_ADDR = 0x1,
	/* variable size pages and 32-bit address */
	TALMMU_MMUTYPE_VAR_PAGES_32BIT_ADDR,
	/* 4kb pages and 36-bit address range */
	TALMMU_MMUTYPE_4K_PAGES_36BIT_ADDR,
	/* 4kb pages and 40-bit address range */
	TALMMU_MMUTYPE_4K_PAGES_40BIT_ADDR,
	/* variable size pages and 40-bit address range */
	TALMMU_MMUTYPE_VP_40BIT,
	TALMMU_MMUTYPE_FORCE32BITS = 0x7FFFFFFFU
};

/* Device flags */
enum talmmu_dev_flags {
	TALMMU_DEVFLAGS_NONE = 0x0,
	TALMMU_DEVFLAGS_FORCE32BITS = 0x7FFFFFFFU
};

/* Heap type */
enum talmmu_heap_type {
	TALMMU_HEAP_SHARED_EXPORTED,
	TALMMU_HEAP_PERCONTEXT,
	TALMMU_HEAP_FORCE32BITS = 0x7FFFFFFFU
};

/* Heap flags */
enum talmmu_eheapflags {
	TALMMU_HEAPFLAGS_NONE = 0x0,
	TALMMU_HEAPFLAGS_SET_CACHE_CONSISTENCY = 0x00000001,
	TALMMU_HEAPFLAGS_128BYTE_INTERLEAVE = 0x00000002,
	TALMMU_HEAPFLAGS_256BYTE_INTERLEAVE = 0x00000004,
	TALMMU_HEAPFLAGS_FORCE32BITS = 0x7FFFFFFFU
};

/* Contains the device memory information */
struct talmmu_devmem_info {
	/* device id */
	unsigned int device_id;
	/* mmu type */
	enum talmmu_mmu_type mmu_type;
	/* Device flags - bit flags that can be combined */
	enum talmmu_dev_flags dev_flags;
	/* Name of the memory space for page directory allocations */
	unsigned char *pagedir_memspace_name;
	/* Name of the memory space for page table allocations */
	unsigned char *pagetable_memspace_name;
	/* Page size in bytes */
	unsigned int page_size;
	/* PTD alignment, must be multiple of Page size */
	unsigned int ptd_alignment;
};

struct talmmu_heap_info {
	/* heap id */
	unsigned int heap_id;
	/* heap type */
	enum talmmu_heap_type heap_type;
	/* heap flags - bit flags that can be combined */
	enum talmmu_eheapflags heap_flags;
	/* Name of the memory space for memory allocations */
	unsigned char *memspace_name;
	/* Base device virtual address */
	unsigned int basedev_virtaddr;
	/* size in bytes */
	unsigned int size;
};

/* Device memory template information */
struct talmmu_dm_tmpl {
	/* list */
	struct lst_t list;
	/* Copy of device memory info structure */
	struct talmmu_devmem_info devmem_info;
	/* Memory space ID for PTD allocations */
	void *ptd_memspace_hndl;
	/* Memory space ID for Page Table allocations */
	void *ptentry_memspace_hndl;
	/* number of heaps */
	unsigned int num_heaps;
	/* Array of heap pointers */
	struct talmmu_devmem_heap *devmem_heap[TALMMU_MAX_DEVICE_HEAPS];
	/* Number of active contexts */
	unsigned int num_ctxs;
	/* List of device memory context created from this template */
	struct lst_t devmem_ctx_list;
	/* Number of bits to shift right to obtain page number */
	unsigned int page_num_shift;
	/* Mask to extract byte-within-page */
	unsigned int byte_in_pagemask;
	/* Heap alignment */
	unsigned int heap_alignment;
	/* Page table entries/page */
	unsigned int pagetable_entries_perpage;
	/* Number of bits to shift right to obtain page table number */
	unsigned int pagetable_num_shift;
	/* Mask to extract index-within-page-table */
	unsigned int index_in_pagetable_mask;
	/* Number of bits to shift right to obtain page dir number */
	unsigned int pagedir_num_shift;
};

/* Device memory heap information */
struct talmmu_devmem_heap {
	/* list item */
	struct lst_t list;
	/* Copy of the heap info structure */
	struct talmmu_heap_info heap_info;
	/* Pointer to the device memory template */
	struct talmmu_dm_tmpl *devmem_template;
	/* true if device virtual address offset allocated externally by user */
	unsigned int ext_dev_virtaddr;
	/* list of memory allocations */
	struct lst_t memory_list;
	/* Memory space ID for memory allocations */
	void *memspace_hndl;
	/* Address context structure */
	struct addr_context ctx;
	/* Regions structure */
	struct addr_region regions;
	/* size of heap guard band */
	unsigned int guardband;
};

struct talmmu_devmem_ctx {
	/* list item */
	struct lst_t list;
	/* Pointer to device template */
	struct talmmu_dm_tmpl *devmem_template;
	/* No. of heaps */
	unsigned int num_heaps;
	/* Array of heap pointers */
	struct talmmu_devmem_heap *devmem_heap[TALMMU_MAX_DEVICE_HEAPS];
	/* The MMU context id */
	unsigned int mmu_ctx_id;
	/* Pointer to the memory that represents Page directory */
	unsigned int *pagedir;
};

struct talmmu_memory {
	/* list item */
	struct lst_t list;
	/* Heap from which memory was allocated */
	struct talmmu_devmem_heap *devmem_heap;
	/* Context through which memory was allocated */
	struct talmmu_devmem_ctx *devmem_ctx;
	/* size */
	unsigned int size;
	/* alignment */
	unsigned int alignment;
	/* device virtual offset of allocation */
	unsigned int dev_virtoffset;
	/* true if device virtual address offset allocated externally by user */
	unsigned int ext_dev_virtaddr;
};

/* This type defines the event types for the TALMMU callbacks */
enum talmmu_event {
	/* Function to flush the cache. */
	TALMMU_EVENT_FLUSH_CACHE,
	/*! Function to write the page directory address to the device */
	TALMMU_EVENT_WRITE_PAGE_DIRECTORY_REF,
	/* Placeholder*/
	TALMMU_NO_OF_EVENTS
};

enum talmmu_heap_option_id {
	/* Add guard band to all mallocs */
	TALMMU_HEAP_OPT_ADD_GUARD_BAND,
	TALMMU_HEAP_OPT_SET_MEM_ATTRIB,
	TALMMU_HEAP_OPT_SET_MEM_POOL,

	/* Placeholder */
	TALMMU_NO_OF_OPTIONS,
	TALMMU_NO_OF_FORCE32BITS = 0x7FFFFFFFU
};

struct talmmu_guardband_options {
	unsigned int guardband;
};

union talmmu_heap_options {
	/* Guardband parameters */
	struct talmmu_guardband_options guardband_opt;
};

int talmmu_init(void);
int talmmu_deinit(void);
int talmmu_devmem_template_create(struct talmmu_devmem_info *devmem_info,
				  void **devmem_template_hndl);
int talmmu_devmem_heap_add(void *devmem_tmplt_hndl,
			   struct talmmu_heap_info *heap_info_arg);
int talmmu_devmem_template_destroy(void *devmem_tmplt_hndl);
int talmmu_devmem_ctx_create(void *devmem_tmplt_hndl,
			     unsigned int mmu_ctx_id,
			     void **devmem_ctx_hndl);
int talmmu_devmem_ctx_destroy(void *devmem_ctx_hndl);
int talmmu_get_heap_handle(unsigned int hid,
			   void *devmem_ctx_hndl,
			   void **devmem_heap_hndl);
/**
 * talmmu_devmem_heap_empty - talmmu_devmem_heap_empty
 * @devmem_heap_hndl: device memory heap handle
 *
 * This function is used for emptying the device memory heap list
 */

int talmmu_devmem_heap_empty(void *devmem_heap_hndl);
void talmmu_devmem_heap_options(void *devmem_heap_hndl,
				enum talmmu_heap_option_id heap_opt_id,
				union talmmu_heap_options heap_options);
int talmmu_devmem_addr_alloc(void *devmem_ctx_hndl,
			     void *devmem_heap_hndl,
			     unsigned int size,
			     unsigned int align,
			     void **mem_hndl);
int talmmu_devmem_addr_free(void *mem_hndl);
int talmmu_get_dev_virt_addr(void *mem_hndl,
			     unsigned int *dev_virt);

#endif /* __TALMMU_API_H__ */
