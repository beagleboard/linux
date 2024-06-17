/* SPDX-License-Identifier: GPL-2.0 */
/*
 * IMG DEC MMU Library
 *
 * Copyright (c) Imagination Technologies Ltd.
 * Copyright (c) 2021 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Authors:
 *	Angela Stegmaier <angelabaker@ti.com>
 *
 * Re-written for upstreamimg
 *	Sidraya Jayagond <sidraya.bj@pathpartnertech.com>
 */

#ifndef IMG_DEC_MMU_MMU_H
#define IMG_DEC_MMU_MMU_H

#include <linux/types.h>

#ifndef MMU_PHYS_SIZE
/* @brief MMU physical address size in bits */
#define MMU_PHYS_SIZE 40
#endif

#ifndef MMU_VIRT_SIZE
/* @brief MMU virtual address size in bits */
#define MMU_VIRT_SIZE 32
#endif

#ifndef MMU_PAGE_SIZE
/* @brief Page size in bytes */
#define MMU_PAGE_SIZE 4096u
#define MMU_PAGE_SHIFT 12
#define MMU_DIR_SHIFT 22
#endif

#if MMU_VIRT_SIZE == 32
/* @brief max number of pagetable that can be stored in the directory entry */
#define MMU_N_TABLE (MMU_PAGE_SIZE / 4u)
/* @brief max number of page mapping in the pagetable */
#define MMU_N_PAGE (MMU_PAGE_SIZE / 4u)
#endif

/* @brief Memory flag used to mark a page mapping as invalid */
#define MMU_FLAG_VALID 0x1
#define MMU_FLAG_INVALID 0x0

/*
 * This type defines MMU variant.
 */
enum mmu_etype {
	MMU_TYPE_NONE = 0,
	MMU_TYPE_32BIT,
	MMU_TYPE_36BIT,
	MMU_TYPE_40BIT,
	MMU_TYPE_FORCE32BITS = 0x7FFFFFFFU
};

/* @brief Page offset mask in virtual address - bottom bits */
static const unsigned long VIRT_PAGE_OFF_MASK = ((1 << MMU_PAGE_SHIFT) - 1);
/* @brief Page table index mask in virtual address - middle bits */
static const unsigned long VIRT_PAGE_TBL_MASK =
	(((1 << MMU_DIR_SHIFT) - 1) & ~(((1 << MMU_PAGE_SHIFT) - 1)));
/* @brief Directory index mask in virtual address - high bits */
static const unsigned long VIRT_DIR_IDX_MASK = (~((1 << MMU_DIR_SHIFT) - 1));

/*
 * struct mmu_heap_alloc - information about a virtual mem heap allocation
 * @virt_addr: pointer to start of the allocation
 * @alloc_size: size in bytes
 */
struct mmu_heap_alloc {
	unsigned long	virt_addr;
	unsigned long	alloc_size;
};

/*
 * struct mmu_page_cfg - mmu_page configuration
 * @phys_addr: physical address - unsigned long long is used to support extended physical
 *	       address on 32bit system
 * @cpu_virt_addr: CPU virtual address pointer
 */
struct mmu_page_cfg {
	unsigned long long	phys_addr;
	unsigned long	cpu_virt_addr;
};

/*
 * typedef mmu_pfn_page_alloc - page table allocation function
 *
 * Pointer to a function implemented by the used allocator to create 1
 * page table (used for the MMU mapping - directory page and mapping page)
 *
 * Return:
 * * A populated mmu_page_cfg structure with the result of the page alloc.
 * * NULL if the allocation failed.
 */
typedef struct mmu_page_cfg *(*mmu_pfn_page_alloc) (void *);

/*
 * typedef mmu_pfn_page_free
 * @arg1: pointer to the mmu_page_cfg that is allocated using mmu_pfn_page_alloc
 *
 * Pointer to a function to free the allocated page table used for MMU mapping.
 *
 * @return void
 */
typedef void (*mmu_pfn_page_free) (struct mmu_page_cfg *arg1);

/*
 * typedef mmu_pfn_page_update
 * @arg1: pointer to the mmu_page_cfg that is allocated using mmu_pfn_page_alloc
 *
 * Pointer to a function to update Device memory on non Unified Memory
 *
 * @return void
 */
typedef void (*mmu_pfn_page_update) (struct mmu_page_cfg *arg1);

/*
 * typedef mmu_pfn_page_write
 * @mmu_page: mmu_page mmu page configuration to be written
 * @offset: offset in entries (32b word)
 * @pa_to_write: pa_to_write physical address to write
 * @flags: flags bottom part of the entry used as flags for the MMU (including
 *	   valid flag)
 *
 * Pointer to a function to write to a device address
 *
 * @return void
 */
typedef void (*mmu_pfn_page_write) (struct mmu_page_cfg *mmu_page,
				    unsigned int offset,
				    unsigned long long pa_to_write, unsigned int flags);

/*
 * struct mmu_info
 * @pfn_page_alloc: function pointer for allocating a physical page used in
 *		    MMU mapping
 * @alloc_ctx: allocation context handler
 * @pfn_page_free: function pointer for freeing a physical page used in
 *		   MMU mapping
 * @pfn_page_write: function pointer to write a physical address onto a page.
 *		    If NULL, then internal function is used. Internal function
 *		    assumes that MMU_PHYS_SIZE is the MMU size.
 * @pfn_page_update: function pointer to update a physical page on device if
 *		     non UMA.
 */
struct mmu_info {
	mmu_pfn_page_alloc	pfn_page_alloc;
	void			*alloc_ctx;
	mmu_pfn_page_free	pfn_page_free;
	mmu_pfn_page_write	pfn_page_write;
	mmu_pfn_page_update	pfn_page_update;
};

/*
 * mmu_get_page_size() - Access the compilation specified page size of the
 *			 MMU (in Bytes)
 */
static inline unsigned long mmu_get_page_size(void)
{
	return MMU_PAGE_SIZE;
}

struct mmu_directory *mmu_create_directory(const struct mmu_info *mmu_info_ops);
int mmu_destroy_directory(struct mmu_directory *mmu_dir);

struct mmu_page_cfg *mmu_directory_get_page(struct mmu_directory *mmu_dir);

struct mmu_map *mmu_directory_map_sg(struct mmu_directory *mmu_dir,
				     void *phys_page_sg,
				     const struct mmu_heap_alloc *dev_va,
				     unsigned int map_flag);
int mmu_directory_unmap(struct mmu_map *map);

unsigned int mmu_directory_get_pagetable_entry(struct mmu_directory *mmu_dir,
					       unsigned long dev_virt_addr);

#endif /* IMG_DEC_MMU_MMU_H */
