// SPDX-License-Identifier: GPL-2.0
/*
 * Implementation of Gasket page table support.
 *
 * Copyright (C) 2018 Google, Inc.
 */

/*
 * Implementation of Gasket page table support.
 *
 * This file assumes 4kB pages throughout; can be factored out when necessary.
 *
 * There is a configurable number of page table entries, as well as a
 * configurable bit index for the extended address flag. Both of these are
 * specified in gasket_page_table_init through the page_table_config parameter.
 *
 * The following example assumes:
 *   page_table_config->total_entries = 8192
 *   page_table_config->extended_bit = 63
 *
 * Address format:
 * Simple addresses - those whose containing pages are directly placed in the
 * device's address translation registers - are laid out as:
 * [ 63 - 25: 0 | 24 - 12: page index | 11 - 0: page offset ]
 * page index:  The index of the containing page in the device's address
 *              translation registers.
 * page offset: The index of the address into the containing page.
 *
 * Extended address - those whose containing pages are contained in a second-
 * level page table whose address is present in the device's address translation
 * registers - are laid out as:
 * [ 63: flag | 62 - 34: 0 | 33 - 21: dev/level 0 index |
 *   20 - 12: host/level 1 index | 11 - 0: page offset ]
 * flag:        Marker indicating that this is an extended address. Always 1.
 * dev index:   The index of the first-level page in the device's extended
 *              address translation registers.
 * host index:  The index of the containing page in the [host-resident] second-
 *              level page table.
 * page offset: The index of the address into the containing [second-level]
 *              page.
 */
#include "gasket_page_table.h"

#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/file.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pagemap.h>
#include <linux/version.h>
#include <linux/vmalloc.h>

#if __has_include(<linux/dma-buf.h>)
MODULE_IMPORT_NS(DMA_BUF);
#endif

#include "gasket_constants.h"
#include "gasket_core.h"

/* Constants & utility macros */
/* The number of pages that can be mapped into each second-level page table. */
#define GASKET_PAGES_PER_SUBTABLE 512

/* The starting position of the page index in a simple virtual address. */
#define GASKET_SIMPLE_PAGE_SHIFT 12

/* Flag indicating that a [device] slot is valid for use. */
#define GASKET_VALID_SLOT_FLAG 1

/*
 * The starting position of the level 0 page index (i.e., the entry in the
 * device's extended address registers) in an extended address.
 * Also can be thought of as (log2(PAGE_SIZE) + log2(PAGES_PER_SUBTABLE)),
 * or (12 + 9).
 */
#define GASKET_EXTENDED_LVL0_SHIFT 21

/*
 * Number of first level pages that Gasket chips support. Equivalent to
 * log2(NUM_LVL0_PAGE_TABLES)
 *
 * At a maximum, allowing for a 34 bits address space (or 16GB)
 *   = GASKET_EXTENDED_LVL0_WIDTH + (log2(PAGE_SIZE) + log2(PAGES_PER_SUBTABLE)
 * or, = 13 + 9 + 12
 */
#define GASKET_EXTENDED_LVL0_WIDTH 13

/*
 * The starting position of the level 1 page index (i.e., the entry in the
 * host second-level/sub- table) in an extended address.
 */
#define GASKET_EXTENDED_LVL1_SHIFT 12

/*
 * Utilities for accessing flags bitfields.
 */
#define MASK(field)            (((1u << field##_WIDTH) - 1) << field##_SHIFT)
#define GET(field, flags)      (((flags) & MASK(field)) >> field##_SHIFT)
#define SET(field, flags, val) (((flags) & ~MASK(field)) | ((val) << field##_SHIFT))

#define FLAGS_STATUS_SHIFT 0
#define FLAGS_STATUS_WIDTH 1

#define FLAGS_DMA_DIRECTION_SHIFT 1
#define FLAGS_DMA_DIRECTION_WIDTH 2

/* Type declarations */
/* Valid states for a struct gasket_page_table_entry. */
enum pte_status {
	PTE_FREE,
	PTE_INUSE,
};

/*
 * Mapping metadata for a single page.
 *
 * In this file, host-side page table entries are referred to as that (or PTEs).
 * Where device vs. host entries are differentiated, device-side or -visible
 * entries are called "slots". A slot may be either an entry in the device's
 * address translation table registers or an entry in a second-level page
 * table ("subtable").
 *
 * The full data in this structure is visible on the host [of course]. Only
 * the address contained in dma_addr is communicated to the device; that points
 * to the actual page mapped and described by this structure.
 */
struct gasket_page_table_entry {
	/*
	 * Internal structure matches gasket_page_table_ioctl_flags.flags.
	 * NOTE: All fields should have a default value of 0. This ensures that
	 * the kernel will be backwards compatible with old drivers.
	 */
	u32 flags;

	/*
	 * Index for alignment into host vaddrs.
	 * When a user specifies a host address for a mapping, that address may
	 * not be page-aligned. Offset is the index into the containing page of
	 * the host address (i.e., host_vaddr & (PAGE_SIZE - 1)).
	 * This is necessary for translating between user-specified addresses
	 * and page-aligned addresses.
	 */
	int offset;

	/* Address of the page in DMA space. */
	dma_addr_t dma_addr;

	/* Linux page descriptor for the page described by this structure. */
	struct page *page;

	/*
	 * If this is an extended and first-level entry, sublevel points
	 * to the second-level entries underneath this entry.
	 */
	struct gasket_page_table_entry *sublevel;
};

/*
 * Maintains virtual to physical address mapping for a coherent page that is
 * allocated by this module for a given device.
 * Note that coherent pages mappings virt mapping cannot be tracked by the
 * Linux kernel, and coherent pages don't have a struct page associated,
 * hence Linux kernel cannot perform a get_user_page_xx() on a phys address
 * that was allocated coherent.
 * This structure trivially implements this mechanism.
 */
struct gasket_coherent_page_entry {
	/* Phys address, dma'able by the owner device */
	dma_addr_t paddr;

	/* Kernel virtual address */
	u64 user_virt;

	/* User virtual address that was mapped by the mmap kernel subsystem */
	dma_addr_t kernel_virt;

	/*
	 * Whether this page has been mapped into a user land process virtual
	 * space
	 */
	u32 in_use;
};

/* Storage for dmabuf mapping information. */
struct gasket_dmabuf_mapping {
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *attachment;
	struct sg_table *sgt;
	enum dma_data_direction direction;
	struct list_head list;
};

/*
 * [Host-side] page table descriptor.
 *
 * This structure tracks the metadata necessary to manage both simple and
 * extended page tables.
 */
struct gasket_page_table {
	/* The config used to create this page table. */
	struct gasket_page_table_config config;

	/* The number of simple (single-level) entries in the page table. */
	uint num_simple_entries;

	/* The number of extended (two-level) entries in the page table. */
	uint num_extended_entries;

	/* Array of [host-side] page table entries. */
	struct gasket_page_table_entry *entries;

	/* Number of actively mapped kernel pages in this table. */
	uint num_active_pages;

	/* Device register: base of/first slot in the page table. */
	u64 __iomem *base_slot;

	/* Device register: holds the offset indicating the start of the
	 * extended address region of the device's address translation table.
	 */
	u64 __iomem *extended_offset_reg;

	/* Device structure for the underlying device. Only used for logging. */
	struct device *device;

	/* PCI system descriptor for the underlying device. */
	struct pci_dev *pci_dev;

	/* Location of the extended address bit for this Gasket device. */
	u64 extended_flag;

	/* Mutex to protect page table internals. */
	struct mutex mutex;

	/* Number of coherent pages accessible thru by this page table */
	int num_coherent_pages;

	/*
	 * List of coherent memory (physical) allocated for a device.
	 *
	 * This structure also remembers the user virtual mapping, this is
	 * hacky, but we need to do this because the kernel doesn't keep track
	 * of the user coherent pages (pfn pages), and virt to coherent page
	 * mapping.
	 * TODO: use find_vma() APIs to convert host address to vm_area, to
	 * dma_addr_t instead of storing user virtu address in
	 * gasket_coherent_page_entry
	 *
	 * Note that the user virtual mapping is created by the driver, in
	 * gasket_mmap function, so user_virt belongs in the driver anyhow.
	 */
	struct gasket_coherent_page_entry *coherent_pages;

	/* List of dmabufs currently attached and mapped. */
	struct list_head dmabufs;
};

/* See gasket_page_table.h for description. */
int gasket_page_table_init(struct gasket_page_table **ppg_tbl,
			   const struct gasket_bar_data *bar_data,
			   const struct gasket_page_table_config *page_table_config,
			   struct device *device, struct pci_dev *pci_dev)
{
	ulong bytes;
	struct gasket_page_table *pg_tbl;
	ulong total_entries = page_table_config->total_entries;

	/*
	 * TODO: Verify config->total_entries against value read from the
	 * hardware register that contains the page table size.
	 */
	if (total_entries == ULONG_MAX) {
		dev_dbg(device, "Error reading page table size. "
			"Initializing page table with size 0\n");
		total_entries = 0;
	}

	dev_dbg(device,
		"Attempting to initialize page table of size 0x%lx\n",
		total_entries);

	dev_dbg(device,
		"Table has base reg 0x%x, extended offset reg 0x%x\n",
		page_table_config->base_reg,
		page_table_config->extended_reg);

	*ppg_tbl = kzalloc(sizeof(**ppg_tbl), GFP_KERNEL);
	if (!*ppg_tbl) {
		dev_dbg(device, "No memory for page table\n");
		return -ENOMEM;
	}

	pg_tbl = *ppg_tbl;
	bytes = total_entries * sizeof(struct gasket_page_table_entry);
	if (bytes != 0) {
		pg_tbl->entries = vzalloc(bytes);
		if (!pg_tbl->entries) {
			dev_dbg(device,
				"No memory for address translation metadata\n");
			kfree(pg_tbl);
			*ppg_tbl = NULL;
			return -ENOMEM;
		}
	}

	mutex_init(&pg_tbl->mutex);
	memcpy(&pg_tbl->config, page_table_config, sizeof(*page_table_config));
	if (pg_tbl->config.mode == GASKET_PAGE_TABLE_MODE_NORMAL ||
	    pg_tbl->config.mode == GASKET_PAGE_TABLE_MODE_SIMPLE) {
		pg_tbl->num_simple_entries = total_entries;
		pg_tbl->num_extended_entries = 0;
		pg_tbl->extended_flag = 1ull << page_table_config->extended_bit;
	} else {
		pg_tbl->num_simple_entries = 0;
		pg_tbl->num_extended_entries = total_entries;
		pg_tbl->extended_flag = 0;
	}
	pg_tbl->num_active_pages = 0;
	pg_tbl->base_slot =
		(u64 __iomem *)&bar_data->virt_base[page_table_config->base_reg];
	pg_tbl->extended_offset_reg =
		(u64 __iomem *)&bar_data->virt_base[page_table_config->extended_reg];
	pg_tbl->device = get_device(device);
	pg_tbl->pci_dev = pci_dev;
	INIT_LIST_HEAD(&pg_tbl->dmabufs);

	dev_dbg(device, "Page table initialized successfully\n");

	return 0;
}

/*
 * Check if a range of PTEs is free.
 * The page table mutex must be held by the caller.
 */
static bool gasket_is_pte_range_free(struct gasket_page_table_entry *ptes,
				     uint num_entries)
{
	int i;

	for (i = 0; i < num_entries; i++) {
		if (GET(FLAGS_STATUS, ptes[i].flags) != PTE_FREE)
			return false;
	}

	return true;
}

/*
 * Free a second level page [sub]table.
 * The page table mutex must be held before this call.
 */
static void gasket_free_extended_subtable(struct gasket_page_table *pg_tbl,
					  struct gasket_page_table_entry *pte,
					  u64 __iomem *slot)
{
	/* Release the page table from the driver */
	pte->flags = SET(FLAGS_STATUS, pte->flags, PTE_FREE);

	/* Release the page table from the device */
	writeq(0, slot);

	if (pte->dma_addr)
		dma_unmap_page(pg_tbl->device, pte->dma_addr, PAGE_SIZE,
			       DMA_TO_DEVICE);

	vfree(pte->sublevel);

	if (pte->page)
		free_page((ulong)page_address(pte->page));

	memset(pte, 0, sizeof(struct gasket_page_table_entry));
}

/*
 * Actually perform collection.
 * The page table mutex must be held by the caller.
 */
static void
gasket_page_table_garbage_collect_nolock(struct gasket_page_table *pg_tbl)
{
	struct gasket_page_table_entry *pte;
	u64 __iomem *slot;

	/* XXX FIX ME XXX -- more efficient to keep a usage count */
	/* rather than scanning the second level page tables */

	for (pte = pg_tbl->entries + pg_tbl->num_simple_entries,
	     slot = pg_tbl->base_slot + pg_tbl->num_simple_entries;
	     pte < pg_tbl->entries + pg_tbl->config.total_entries;
	     pte++, slot++) {
		if (GET(FLAGS_STATUS, pte->flags) == PTE_INUSE) {
			if (gasket_is_pte_range_free(pte->sublevel,
						     GASKET_PAGES_PER_SUBTABLE))
				gasket_free_extended_subtable(pg_tbl, pte,
							      slot);
		}
	}
}

/* See gasket_page_table.h for description. */
void gasket_page_table_garbage_collect(struct gasket_page_table *pg_tbl)
{
	mutex_lock(&pg_tbl->mutex);
	gasket_page_table_garbage_collect_nolock(pg_tbl);
	mutex_unlock(&pg_tbl->mutex);
}

/* See gasket_page_table.h for description. */
void gasket_page_table_cleanup(struct gasket_page_table *pg_tbl)
{
	/* Deallocate free second-level tables. */
	gasket_page_table_garbage_collect(pg_tbl);

	/* TODO: Check that all PTEs have been freed? */

	vfree(pg_tbl->entries);
	pg_tbl->entries = NULL;

	put_device(pg_tbl->device);
	kfree(pg_tbl);
}

/* See gasket_page_table.h for description. */
int gasket_page_table_partition(struct gasket_page_table *pg_tbl,
				uint num_simple_entries)
{
	int i, start;

	mutex_lock(&pg_tbl->mutex);
	if (num_simple_entries > pg_tbl->config.total_entries) {
		mutex_unlock(&pg_tbl->mutex);
		return -EINVAL;
	}

	gasket_page_table_garbage_collect_nolock(pg_tbl);

	start = min(pg_tbl->num_simple_entries, num_simple_entries);

	for (i = start; i < pg_tbl->config.total_entries; i++) {
		if (GET(FLAGS_STATUS, pg_tbl->entries[i].flags) != PTE_FREE) {
			dev_err(pg_tbl->device, "entry %d is not free\n", i);
			mutex_unlock(&pg_tbl->mutex);
			return -EBUSY;
		}
	}

	pg_tbl->num_simple_entries = num_simple_entries;
	pg_tbl->num_extended_entries =
		pg_tbl->config.total_entries - num_simple_entries;
	writeq(num_simple_entries, pg_tbl->extended_offset_reg);

	mutex_unlock(&pg_tbl->mutex);
	return 0;
}
EXPORT_SYMBOL(gasket_page_table_partition);

/*
 * Return whether a host buffer was mapped as coherent memory.
 *
 * A Gasket page_table currently support one contiguous dma range, mapped to one
 * contiguous virtual memory range. Check if the host_addr is within that range.
 */
static int is_coherent(struct gasket_page_table *pg_tbl, ulong host_addr)
{
	u64 min, max;

	/* whether the host address is within user virt range */
	if (!pg_tbl->coherent_pages)
		return 0;

	min = (u64)pg_tbl->coherent_pages[0].user_virt;
	max = min + PAGE_SIZE * pg_tbl->num_coherent_pages;

	return min <= host_addr && host_addr < max;
}

/* Safely return a page to the OS. */
static bool gasket_release_page(struct page *page)
{
	if (!page)
		return false;

	if (!PageReserved(page))
		SetPageDirty(page);
	put_page(page);

	return true;
}

/*
 * Get and map last level page table buffers.
 *
 * slots is the location(s) to write device-mapped page address. If this is a
 * simple mapping, these will be address translation registers. If this is
 * an extended mapping, these will be within a second-level page table
 * allocated by the host and so must have their __iomem attribute casted away.
 */
static int gasket_perform_mapping(struct gasket_page_table *pg_tbl,
				  struct gasket_page_table_entry *ptes,
				  u64 __iomem *slots,
				  struct sg_page_iter *sg_iter,
				  ulong host_addr,
				  uint num_pages, u32 flags,
				  int is_simple_mapping)
{
	int ret;
	ulong offset;
	struct page *page;
	dma_addr_t dma_addr;
	ulong page_addr;
	int i;
	enum dma_data_direction direction;

	/* Must have a virtual host address or a sg iterator, but not both. */
	if (!((uintptr_t)host_addr ^ (uintptr_t)sg_iter)) {
		dev_err(pg_tbl->device, "need sg_iter or host_addr\n");
		return -EINVAL;
	}

	direction = GET(FLAGS_DMA_DIRECTION, flags);
	if (direction == DMA_NONE) {
		dev_err(pg_tbl->device, "invalid DMA direction flags=0x%lx\n",
			(unsigned long)flags);
		return -EINVAL;
	}

	for (i = 0; i < num_pages; i++) {
		page_addr = host_addr + i * PAGE_SIZE;
		offset = page_addr & (PAGE_SIZE - 1);
		dev_dbg(pg_tbl->device, "%s i %d\n", __func__, i);
		if (sg_iter) {
			if (!__sg_page_iter_next(sg_iter))
				return -EINVAL;

			/* Page already mapped for DMA. */
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 1, 0)
			ptes[i].dma_addr = sg_page_iter_dma_address(sg_iter);
#else
			ptes[i].dma_addr = sg_page_iter_dma_address(
				container_of(sg_iter, struct sg_dma_page_iter, base));
#endif
			ptes[i].page = NULL;
			offset = 0;
		} else if (is_coherent(pg_tbl, host_addr)) {
			u64 off =
				(u64)host_addr -
				(u64)pg_tbl->coherent_pages[0].user_virt;
			ptes[i].page = NULL;
			ptes[i].offset = offset;
			ptes[i].dma_addr = pg_tbl->coherent_pages[0].paddr +
					   off + i * PAGE_SIZE;
		} else {
			ret = get_user_pages_fast(page_addr - offset, 1,
						  direction != DMA_TO_DEVICE,
						  &page);

			if (ret <= 0) {
				dev_err(pg_tbl->device,
					"get user pages failed for addr=0x%lx, "
					"offset=0x%lx [ret=%d]\n",
					page_addr, offset, ret);
				return ret ? ret : -ENOMEM;
			}
			++pg_tbl->num_active_pages;

			ptes[i].page = page;
			ptes[i].offset = offset;

			/* Map the page into DMA space. */
			ptes[i].dma_addr = dma_map_page(pg_tbl->device, page, 0, PAGE_SIZE,
							GET(FLAGS_DMA_DIRECTION, flags));
			dev_dbg(pg_tbl->device,
				"%s i %d pte %p pfn %p -> mapped %llx\n",
				__func__, i, &ptes[i],
				(void *)page_to_pfn(page),
				(unsigned long long)ptes[i].dma_addr);

			if (dma_mapping_error(pg_tbl->device,
					      ptes[i].dma_addr)) {
				dev_dbg(pg_tbl->device,
					"%s i %d -> fail to map page %llx "
					"[pfn %p phys %p]\n",
					__func__, i,
					(unsigned long long)ptes[i].dma_addr,
					(void *)page_to_pfn(page),
					(void *)page_to_phys(page));

				/* clean up */
				if (gasket_release_page(ptes[i].page))
					--pg_tbl->num_active_pages;

				memset(&ptes[i], 0, sizeof(struct gasket_page_table_entry));
				return -EINVAL;
			}
		}

		/* Make the DMA-space address available to the device. */
		dma_addr = (ptes[i].dma_addr + offset) | GASKET_VALID_SLOT_FLAG;

		if (is_simple_mapping)
			writeq(dma_addr, &slots[i]);
		else
			((u64 __force *)slots)[i] = dma_addr;

		/* Set PTE flags equal to flags param with STATUS=PTE_INUSE. */
		ptes[i].flags = SET(FLAGS_STATUS, flags, PTE_INUSE);
	}
	return 0;
}

/*
 * Return the index of the page for the address in the simple table.
 * Does not perform validity checking.
 */
static int gasket_simple_page_idx(struct gasket_page_table *pg_tbl,
				  u64 dev_addr)
{
	return (dev_addr >> GASKET_SIMPLE_PAGE_SHIFT) &
		(pg_tbl->config.total_entries - 1);
}

/*
 * Return the level 0 page index for the given address.
 * Does not perform validity checking.
 */
static ulong gasket_extended_lvl0_page_idx(struct gasket_page_table *pg_tbl,
					   u64 dev_addr)
{
	return (dev_addr >> GASKET_EXTENDED_LVL0_SHIFT) &
	       (pg_tbl->config.total_entries - 1);
}

/*
 * Return the level 1 page index for the given address.
 * Does not perform validity checking.
 */
static ulong gasket_extended_lvl1_page_idx(struct gasket_page_table *pg_tbl,
					   u64 dev_addr)
{
	return (dev_addr >> GASKET_EXTENDED_LVL1_SHIFT) &
	       (GASKET_PAGES_PER_SUBTABLE - 1);
}

/*
 * Allocate page table entries in a simple table.
 * The page table mutex must be held by the caller.
 */
static int gasket_alloc_simple_entries(struct gasket_page_table *pg_tbl,
				       u64 dev_addr, uint num_pages)
{
	if (!gasket_is_pte_range_free(pg_tbl->entries +
				      gasket_simple_page_idx(pg_tbl, dev_addr),
				      num_pages))
		return -EBUSY;

	return 0;
}

/*
 * Unmap and release mapped pages.
 * The page table mutex must be held by the caller.
 */
static void gasket_perform_unmapping(struct gasket_page_table *pg_tbl,
				     struct gasket_page_table_entry *ptes,
				     u64 __iomem *slots, uint num_pages,
				     int is_simple_mapping)
{
	int i;
	/*
	 * For each page table entry and corresponding entry in the device's
	 * address translation table:
	 */
	for (i = 0; i < num_pages; i++) {
		/* release the address from the device, */
		if (is_simple_mapping)
			writeq(0, &slots[i]);
		else
			((u64 __force *)slots)[i] = 0;

		/* release the address from the driver, */
		if (GET(FLAGS_STATUS, ptes[i].flags) == PTE_INUSE) {
			if (ptes[i].page && ptes[i].dma_addr) {
				dma_unmap_page(pg_tbl->device, ptes[i].dma_addr, PAGE_SIZE,
					       GET(FLAGS_DMA_DIRECTION, ptes[i].flags));
			}
			if (gasket_release_page(ptes[i].page))
				--pg_tbl->num_active_pages;
		}

		/* and clear the PTE. */
		memset(&ptes[i], 0, sizeof(struct gasket_page_table_entry));
	}
}

/*
 * Unmap and release pages mapped to simple addresses.
 * The page table mutex must be held by the caller.
 */
static void gasket_unmap_simple_pages(struct gasket_page_table *pg_tbl,
				      u64 dev_addr, uint num_pages)
{
	uint slot = gasket_simple_page_idx(pg_tbl, dev_addr);

	gasket_perform_unmapping(pg_tbl, pg_tbl->entries + slot,
				 pg_tbl->base_slot + slot, num_pages, 1);
}

/*
 * Unmap and release buffers to extended addresses.
 * The page table mutex must be held by the caller.
 */
static void gasket_unmap_extended_pages(struct gasket_page_table *pg_tbl,
					u64 dev_addr, uint num_pages)
{
	uint slot_idx, remain, len;
	struct gasket_page_table_entry *pte;
	u64 __iomem *slot_base;

	remain = num_pages;
	slot_idx = gasket_extended_lvl1_page_idx(pg_tbl, dev_addr);
	pte = pg_tbl->entries + pg_tbl->num_simple_entries +
	      gasket_extended_lvl0_page_idx(pg_tbl, dev_addr);

	while (remain > 0) {
		/* TODO: Add check to ensure pte remains valid? */
		len = min(remain, GASKET_PAGES_PER_SUBTABLE - slot_idx);

		if (GET(FLAGS_STATUS, pte->flags) == PTE_INUSE) {
			slot_base = (u64 __iomem *)(page_address(pte->page) +
						    pte->offset);
			gasket_perform_unmapping(pg_tbl,
						 pte->sublevel + slot_idx,
						 slot_base + slot_idx, len, 0);
			/*
			 * Extended page tables are in DRAM so they need to be
			 * synced each time they are updated.
			 */
			dma_sync_single_for_device(pg_tbl->device,
						   pte->dma_addr + slot_idx * sizeof(u64),
						   len * sizeof(u64), DMA_TO_DEVICE);
		}

		remain -= len;
		slot_idx = 0;
		pte++;
	}
}

/* Evaluates to nonzero if the specified virtual address is simple. */
static inline bool gasket_addr_is_simple(struct gasket_page_table *pg_tbl,
					 u64 addr)
{
	return !((addr) & (pg_tbl)->extended_flag);
}

/*
 * Convert (simple, page, offset) into a device address.
 * Examples:
 * Simple page 0, offset 32:
 *  Input (1, 0, 32), Output 0x20
 * Simple page 1000, offset 511:
 *  Input (1, 1000, 511), Output 0x3E81FF
 * Extended page 0, offset 32:
 *  Input (0, 0, 32), Output 0x8000000020
 * Extended page 1000, offset 511:
 *  Input (0, 1000, 511), Output 0x8003E81FF
 */
static u64 gasket_components_to_dev_address(struct gasket_page_table *pg_tbl,
					      int is_simple, uint page_index,
					      uint offset)
{
	u64 dev_addr = (page_index << GASKET_SIMPLE_PAGE_SHIFT) | offset;

	return is_simple ? dev_addr : (pg_tbl->extended_flag | dev_addr);
}

/*
 * Validity checking for simple addresses.
 *
 * Verify that address translation commutes (from address to/from page + offset)
 * and that the requested page range starts and ends within the set of
 * currently-partitioned simple pages.
 */
static bool gasket_is_simple_dev_addr_bad(struct gasket_page_table *pg_tbl,
					  u64 dev_addr, uint num_pages)
{
	ulong page_offset = dev_addr & (PAGE_SIZE - 1);
	ulong page_index =
		(dev_addr / PAGE_SIZE) & (pg_tbl->config.total_entries - 1);

	if (gasket_components_to_dev_address(pg_tbl, 1, page_index,
					     page_offset) != dev_addr) {
		dev_err(pg_tbl->device, "address is invalid, 0x%llX\n",
			dev_addr);
		return true;
	}

	if (page_index >= pg_tbl->num_simple_entries) {
		dev_err(pg_tbl->device,
			"starting slot at %lu is too large, max is < %u\n",
			page_index, pg_tbl->num_simple_entries);
		return true;
	}

	if (page_index + num_pages > pg_tbl->num_simple_entries) {
		dev_err(pg_tbl->device,
			"ending slot at %lu is too large, max is <= %u\n",
			page_index + num_pages, pg_tbl->num_simple_entries);
		return true;
	}

	return false;
}

/*
 * Validity checking for extended addresses.
 *
 * Verify that address translation commutes (from address to/from page +
 * offset) and that the requested page range starts and ends within the set of
 * currently-partitioned extended pages.
 */
static bool gasket_is_extended_dev_addr_bad(struct gasket_page_table *pg_tbl,
					    u64 dev_addr, uint num_pages)
{
	/* Starting byte index of dev_addr into the first mapped page */
	ulong page_offset = dev_addr & (PAGE_SIZE - 1);
	ulong page_global_idx, page_lvl0_idx;
	ulong num_lvl0_pages;
	u64 addr;

	/* check if the device address is out of bound */
	addr = dev_addr & ~((pg_tbl)->extended_flag);
	if (addr >> (GASKET_EXTENDED_LVL0_WIDTH + GASKET_EXTENDED_LVL0_SHIFT)) {
		dev_err(pg_tbl->device,
			"device address out of bounds: 0x%llx\n", dev_addr);
		return true;
	}

	/* Find the starting sub-page index in the space of all sub-pages. */
	page_global_idx = (dev_addr / PAGE_SIZE) &
		(pg_tbl->config.total_entries * GASKET_PAGES_PER_SUBTABLE - 1);

	/* Find the starting level 0 index. */
	page_lvl0_idx = gasket_extended_lvl0_page_idx(pg_tbl, dev_addr);

	/* Get the count of affected level 0 pages. */
	num_lvl0_pages = (num_pages + GASKET_PAGES_PER_SUBTABLE - 1) /
		GASKET_PAGES_PER_SUBTABLE;

	if (gasket_components_to_dev_address(pg_tbl, 0, page_global_idx,
					     page_offset) != dev_addr) {
		dev_err(pg_tbl->device, "address is invalid: 0x%llx\n",
			dev_addr);
		return true;
	}

	if (page_lvl0_idx >= pg_tbl->num_extended_entries) {
		dev_err(pg_tbl->device,
			"starting level 0 slot at %lu is too large, max is < "
			"%u\n", page_lvl0_idx, pg_tbl->num_extended_entries);
		return true;
	}

	if (page_lvl0_idx + num_lvl0_pages > pg_tbl->num_extended_entries) {
		dev_err(pg_tbl->device,
			"ending level 0 slot at %lu is too large, max is <= %u\n",
			page_lvl0_idx + num_lvl0_pages,
			pg_tbl->num_extended_entries);
		return true;
	}

	return false;
}

/*
 * Non-locking entry to unmapping routines.
 * The page table mutex must be held by the caller.
 */
static void gasket_page_table_unmap_nolock(struct gasket_page_table *pg_tbl,
					   u64 dev_addr, uint num_pages)
{
	if (!num_pages)
		return;

	if (gasket_addr_is_simple(pg_tbl, dev_addr))
		gasket_unmap_simple_pages(pg_tbl, dev_addr, num_pages);
	else
		gasket_unmap_extended_pages(pg_tbl, dev_addr, num_pages);
}

/*
 * Allocate and map pages to simple addresses.
 * If there is an error, no pages are mapped.
 */
static int gasket_map_simple_pages(struct gasket_page_table *pg_tbl,
				   struct sg_page_iter *sg_iter,
				   ulong host_addr, u64 dev_addr,
				   uint num_pages, u32 flags)
{
	int ret;
	uint slot_idx = gasket_simple_page_idx(pg_tbl, dev_addr);

	ret = gasket_alloc_simple_entries(pg_tbl, dev_addr, num_pages);
	if (ret) {
		dev_err(pg_tbl->device,
			"page table slots %u (@ 0x%llx) to %u are not available\n",
			slot_idx, (long long unsigned int)dev_addr,
			slot_idx + num_pages - 1);
		return ret;
	}

	ret = gasket_perform_mapping(pg_tbl, pg_tbl->entries + slot_idx,
				     pg_tbl->base_slot + slot_idx, sg_iter,
				     host_addr, num_pages, flags, 1);

	if (ret) {
		gasket_page_table_unmap_nolock(pg_tbl, dev_addr, num_pages);
		dev_err(pg_tbl->device, "gasket_perform_mapping %d\n", ret);
	}
	return ret;
}

/*
 * Allocate a second level page table.
 * The page table mutex must be held by the caller.
 */
static int gasket_alloc_extended_subtable(struct gasket_page_table *pg_tbl,
					  struct gasket_page_table_entry *pte,
					  u64 __iomem *slot)
{
	ulong page_addr, subtable_bytes;
	dma_addr_t dma_addr;

	/* XXX FIX ME XXX this is inefficient for non-4K page sizes */

	/* GFP_DMA flag must be passed to architectures for which
	 * part of the memory range is not considered DMA'able.
	 * This seems to be the case for Juno board with 4.5.0 Linaro kernel
	 */
	page_addr = get_zeroed_page(GFP_KERNEL | GFP_DMA);
	if (!page_addr)
		return -ENOMEM;
	pte->page = virt_to_page((void *)page_addr);
	pte->offset = 0;

	subtable_bytes = sizeof(struct gasket_page_table_entry) *
		GASKET_PAGES_PER_SUBTABLE;
	pte->sublevel = vzalloc(subtable_bytes);
	if (!pte->sublevel) {
		free_page(page_addr);
		memset(pte, 0, sizeof(struct gasket_page_table_entry));
		return -ENOMEM;
	}

	/* Map the page into DMA space. */
	pte->dma_addr = dma_map_page(pg_tbl->device, pte->page, 0, PAGE_SIZE,
				     DMA_TO_DEVICE);
	if (dma_mapping_error(pg_tbl->device, pte->dma_addr)) {
		dev_dbg(pg_tbl->device,
			"%s -> fail to map page %llx "
			"[pfn %p phys %p]\n",
			__func__,
			(unsigned long long)pte->dma_addr,
			(void *)page_to_pfn(pte->page),
			(void *)page_to_phys(pte->page));

		/* clean up */
		free_page(page_addr);
		vfree(pte->sublevel);
		memset(pte, 0, sizeof(struct gasket_page_table_entry));

		return -ENOMEM;
	}

	/* make the addresses available to the device */
	dma_addr = (pte->dma_addr + pte->offset) | GASKET_VALID_SLOT_FLAG;
	writeq(dma_addr, slot);

	pte->flags = SET(FLAGS_STATUS, pte->flags, PTE_INUSE);

	return 0;
}

/*
 * Allocate slots in an extended page table.  Check to see if a range of page
 * table slots are available. If necessary, memory is allocated for second level
 * page tables.
 *
 * Note that memory for second level page tables is allocated as needed, but
 * that memory is only freed on the final close	of the device file, when the
 * page tables are repartitioned, or the the device is removed.  If there is an
 * error or if the full range of slots is not available, any memory
 * allocated for second level page tables remains allocated until final close,
 * repartition, or device removal.
 *
 * The page table mutex must be held by the caller.
 */
static int gasket_alloc_extended_entries(struct gasket_page_table *pg_tbl,
					 u64 dev_addr, uint num_entries)
{
	int ret = 0;
	uint remain, subtable_slot_idx, len;
	struct gasket_page_table_entry *pte;
	u64 __iomem *slot;

	remain = num_entries;
	subtable_slot_idx = gasket_extended_lvl1_page_idx(pg_tbl, dev_addr);
	pte = pg_tbl->entries + pg_tbl->num_simple_entries +
	      gasket_extended_lvl0_page_idx(pg_tbl, dev_addr);
	slot = pg_tbl->base_slot + pg_tbl->num_simple_entries +
	       gasket_extended_lvl0_page_idx(pg_tbl, dev_addr);

	while (remain > 0) {
		len = min(remain,
			  GASKET_PAGES_PER_SUBTABLE - subtable_slot_idx);

		if (GET(FLAGS_STATUS, pte->flags) == PTE_FREE) {
			ret = gasket_alloc_extended_subtable(pg_tbl, pte, slot);
			if (ret) {
				dev_err(pg_tbl->device,
					"no memory for extended addr subtable\n");
				return ret;
			}
		} else {
			if (!gasket_is_pte_range_free(pte->sublevel +
						      subtable_slot_idx, len))
				return -EBUSY;
		}

		remain -= len;
		subtable_slot_idx = 0;
		pte++;
		slot++;
	}

	return 0;
}

/*
 * gasket_map_extended_pages - Get and map buffers to extended addresses.
 * If there is an error, no pages are mapped.
 */
static int gasket_map_extended_pages(struct gasket_page_table *pg_tbl,
				     struct sg_page_iter *sg_iter,
				     ulong host_addr, u64 dev_addr,
				     uint num_pages, u32 flags)
{
	int ret;
	u64 dev_addr_end;
	uint slot_idx, remain, len;
	struct gasket_page_table_entry *pte;
	u64 __iomem *slot_base;

	ret = gasket_alloc_extended_entries(pg_tbl, dev_addr, num_pages);
	if (ret) {
		dev_addr_end = dev_addr + (num_pages / PAGE_SIZE) - 1;
		dev_err(pg_tbl->device,
			"page table slots (%lu,%lu) (@ 0x%llx) to (%lu,%lu) "
			"are not available\n",
			gasket_extended_lvl0_page_idx(pg_tbl, dev_addr),
			gasket_extended_lvl1_page_idx(pg_tbl, dev_addr),
			(long long unsigned int)dev_addr,
			gasket_extended_lvl0_page_idx(pg_tbl, dev_addr_end),
			gasket_extended_lvl1_page_idx(pg_tbl, dev_addr_end));
		return ret;
	}

	remain = num_pages;
	slot_idx = gasket_extended_lvl1_page_idx(pg_tbl, dev_addr);
	pte = pg_tbl->entries + pg_tbl->num_simple_entries +
	      gasket_extended_lvl0_page_idx(pg_tbl, dev_addr);

	while (remain > 0) {
		len = min(remain, GASKET_PAGES_PER_SUBTABLE - slot_idx);

		slot_base =
			(u64 __iomem *)(page_address(pte->page) + pte->offset);
		ret = gasket_perform_mapping(pg_tbl, pte->sublevel + slot_idx,
					     slot_base + slot_idx, sg_iter,
					     host_addr, len, flags, 0);
		if (ret) {
			gasket_page_table_unmap_nolock(pg_tbl, dev_addr,
						       num_pages);
			return ret;
		}

		/*
		 * Extended page tables are in DRAM so they need to be synced
		 * each time they are updated.
		 */
		dma_sync_single_for_device(pg_tbl->device,
					   pte->dma_addr + slot_idx * sizeof(u64),
					   len * sizeof(u64), DMA_TO_DEVICE);

		remain -= len;
		slot_idx = 0;
		pte++;
		if (host_addr)
			host_addr += len * PAGE_SIZE;
	}

	return 0;
}

/*
 * See gasket_page_table.h for general description.
 *
 * gasket_page_table_map calls either gasket_map_simple_pages() or
 * gasket_map_extended_pages() to actually perform the mapping.
 *
 * The page table mutex is held for the entire operation.
 */
int gasket_page_table_map(struct gasket_page_table *pg_tbl, ulong host_addr,
			  u64 dev_addr, uint num_pages, u32 flags)
{
	int ret;

	if (!num_pages)
		return 0;

	mutex_lock(&pg_tbl->mutex);

	if (gasket_addr_is_simple(pg_tbl, dev_addr)) {
		ret = gasket_map_simple_pages(pg_tbl, NULL, host_addr, dev_addr,
					      num_pages, flags);
	} else {
		ret = gasket_map_extended_pages(pg_tbl, NULL, host_addr, dev_addr,
						num_pages, flags);
	}

	mutex_unlock(&pg_tbl->mutex);

	dev_dbg(pg_tbl->device,
		"%s done: ha %llx daddr %llx num %d, flags %x ret %d\n",
		__func__, (unsigned long long)host_addr,
		(unsigned long long)dev_addr, num_pages, flags, ret);
	return ret;
}
EXPORT_SYMBOL(gasket_page_table_map);

/*
 * See gasket_page_table.h for general description.
 *
 * gasket_page_table_unmap takes the page table lock and calls either
 * gasket_unmap_simple_pages() or gasket_unmap_extended_pages() to
 * actually unmap the pages from device space.
 *
 * The page table mutex is held for the entire operation.
 */
void gasket_page_table_unmap(struct gasket_page_table *pg_tbl, u64 dev_addr,
			     uint num_pages)
{
	if (!num_pages)
		return;

	mutex_lock(&pg_tbl->mutex);
	gasket_page_table_unmap_nolock(pg_tbl, dev_addr, num_pages);
	mutex_unlock(&pg_tbl->mutex);
}
EXPORT_SYMBOL(gasket_page_table_unmap);

int gasket_page_table_map_dmabuf(struct gasket_page_table *pg_tbl, int fd,
				 u64 dev_addr, uint num_pages, u32 flags)
{
	int ret, locked = 0;
	struct dma_buf *dmabuf = NULL;
	struct dma_buf_attachment *attachment = NULL;
	struct sg_table *sgt = NULL;
	struct sg_page_iter sg_iter;
	struct gasket_dmabuf_mapping *mapping = NULL;
	enum dma_data_direction direction = GET(FLAGS_DMA_DIRECTION, flags);

	if (direction == DMA_NONE) {
		dev_err(pg_tbl->device,
			"invalid DMA direction flags=0x%x\n", flags);
		return -EINVAL;
	}

	if (!num_pages)
		return 0;

	dmabuf = dma_buf_get(fd);
	if (IS_ERR(dmabuf))
		return PTR_ERR(dmabuf);

	if (PAGE_ALIGN(dmabuf->size) / PAGE_SIZE < num_pages)
		return -EINVAL;

	mapping = kzalloc(sizeof(*mapping), GFP_KERNEL);
	if (!mapping) {
		ret = -ENOMEM;
		goto out;
	}

	attachment = dma_buf_attach(dmabuf, pg_tbl->device);
	if (IS_ERR(attachment)) {
		ret = PTR_ERR(attachment);
		goto out;
	}

	sgt = dma_buf_map_attachment(attachment, direction);
	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);
		goto out;
	}

	mutex_lock(&pg_tbl->mutex);
	locked = 1;

	__sg_page_iter_start(&sg_iter, sgt->sgl, sgt->nents, 0);
	if (gasket_addr_is_simple(pg_tbl, dev_addr)) {
		ret = gasket_map_simple_pages(pg_tbl, &sg_iter, 0, dev_addr,
					      num_pages, flags);
	} else {
		ret = gasket_map_extended_pages(pg_tbl, &sg_iter, 0, dev_addr,
						num_pages, flags);
	}

	if (!ret) {
		INIT_LIST_HEAD(&mapping->list);
		get_dma_buf(dmabuf);
		mapping->dmabuf = dmabuf;
		mapping->attachment = attachment;
		mapping->sgt = sgt;
		mapping->direction = direction;
		list_add(&mapping->list, &pg_tbl->dmabufs);
		sgt = NULL;
		attachment = NULL;
		mapping = NULL;
	}

out:
	if (locked)
		mutex_unlock(&pg_tbl->mutex);

	if (!IS_ERR_OR_NULL(sgt))
		dma_buf_unmap_attachment(attachment, sgt, direction);

	if (!IS_ERR_OR_NULL(attachment))
		dma_buf_detach(dmabuf, attachment);

	kfree(mapping);
	dma_buf_put(dmabuf);

	return ret;
}
EXPORT_SYMBOL(gasket_page_table_map_dmabuf);

/* Detach dmabuf from our device if attached, NULL to detach all. */
static void gasket_page_table_detach_dmabuf_nolock(struct gasket_page_table *pg_tbl,
						   struct dma_buf *dmabuf)
{
	struct gasket_dmabuf_mapping *mapping, *tmp;

	list_for_each_entry_safe(mapping, tmp, &pg_tbl->dmabufs, list) {
		if (!dmabuf || mapping->dmabuf == dmabuf) {
			dma_buf_unmap_attachment(mapping->attachment,
						 mapping->sgt,
						 mapping->direction);
			dma_buf_detach(mapping->dmabuf, mapping->attachment);
			dma_buf_put(mapping->dmabuf);
			list_del(&mapping->list);
			kfree(mapping);
		}
	}
}

int gasket_page_table_unmap_dmabuf(struct gasket_page_table *pg_tbl, int fd,
				   u64 dev_addr, uint num_pages)
{
	struct dma_buf *dmabuf;

	dmabuf = dma_buf_get(fd);
	if (IS_ERR(dmabuf))
		return PTR_ERR(dmabuf);

	if (PAGE_ALIGN(dmabuf->size) / PAGE_SIZE < num_pages) {
		dma_buf_put(dmabuf);
		return -EINVAL;
	}

	mutex_lock(&pg_tbl->mutex);

	gasket_page_table_unmap_nolock(pg_tbl, dev_addr, num_pages);
	gasket_page_table_detach_dmabuf_nolock(pg_tbl, dmabuf);

	mutex_unlock(&pg_tbl->mutex);

	dma_buf_put(dmabuf);

	return 0;
}
EXPORT_SYMBOL(gasket_page_table_unmap_dmabuf);

static void gasket_page_table_unmap_all_nolock(struct gasket_page_table *pg_tbl)
{
	gasket_page_table_detach_dmabuf_nolock(pg_tbl, NULL);

	gasket_unmap_simple_pages(pg_tbl,
				  gasket_components_to_dev_address(pg_tbl, 1, 0,
								   0),
				  pg_tbl->num_simple_entries);
	gasket_unmap_extended_pages(pg_tbl,
				    gasket_components_to_dev_address(pg_tbl, 0,
								     0, 0),
				    pg_tbl->num_extended_entries *
				    GASKET_PAGES_PER_SUBTABLE);
}

/* See gasket_page_table.h for description. */
void gasket_page_table_unmap_all(struct gasket_page_table *pg_tbl)
{
	mutex_lock(&pg_tbl->mutex);
	gasket_page_table_unmap_all_nolock(pg_tbl);
	mutex_unlock(&pg_tbl->mutex);
}
EXPORT_SYMBOL(gasket_page_table_unmap_all);

/* See gasket_page_table.h for description. */
void gasket_page_table_reset(struct gasket_page_table *pg_tbl)
{
	mutex_lock(&pg_tbl->mutex);
	gasket_page_table_unmap_all_nolock(pg_tbl);
	writeq(pg_tbl->config.total_entries, pg_tbl->extended_offset_reg);
	mutex_unlock(&pg_tbl->mutex);
}

/* See gasket_page_table.h for description. */
int gasket_page_table_lookup_page(
	struct gasket_page_table *pg_tbl, u64 dev_addr, struct page **ppage,
	ulong *poffset)
{
	uint page_num;
	struct gasket_page_table_entry *pte;

	mutex_lock(&pg_tbl->mutex);
	if (gasket_addr_is_simple(pg_tbl, dev_addr)) {
		page_num = gasket_simple_page_idx(pg_tbl, dev_addr);
		if (page_num >= pg_tbl->num_simple_entries)
			goto fail;

		pte = pg_tbl->entries + page_num;
		if (GET(FLAGS_STATUS, pte->flags) != PTE_INUSE)
			goto fail;
	} else {
		/* Find the level 0 entry, */
		page_num = gasket_extended_lvl0_page_idx(pg_tbl, dev_addr);
		if (page_num >= pg_tbl->num_extended_entries)
			goto fail;

		pte = pg_tbl->entries + pg_tbl->num_simple_entries + page_num;
		if (GET(FLAGS_STATUS, pte->flags) != PTE_INUSE)
			goto fail;

		/* and its contained level 1 entry. */
		page_num = gasket_extended_lvl1_page_idx(pg_tbl, dev_addr);
		pte = pte->sublevel + page_num;
		if (GET(FLAGS_STATUS, pte->flags) != PTE_INUSE)
			goto fail;
	}

	*ppage = pte->page;
	*poffset = pte->offset;
	mutex_unlock(&pg_tbl->mutex);
	return 0;

fail:
	*ppage = NULL;
	*poffset = 0;
	mutex_unlock(&pg_tbl->mutex);
	return -EINVAL;
}

/* See gasket_page_table.h for description. */
bool gasket_page_table_are_addrs_bad(
	struct gasket_page_table *pg_tbl, ulong host_addr, u64 dev_addr,
	ulong bytes)
{
	if (host_addr & (PAGE_SIZE - 1)) {
		dev_err(pg_tbl->device,
			"host mapping address 0x%lx must be page aligned\n",
			host_addr);
		return true;
	}

	return gasket_page_table_is_dev_addr_bad(pg_tbl, dev_addr, bytes);
}
EXPORT_SYMBOL(gasket_page_table_are_addrs_bad);

/* See gasket_page_table.h for description. */
bool gasket_page_table_is_dev_addr_bad(
	struct gasket_page_table *pg_tbl, u64 dev_addr, ulong bytes)
{
	uint num_pages = bytes / PAGE_SIZE;

	if (bytes & (PAGE_SIZE - 1)) {
		dev_err(pg_tbl->device,
			"mapping size 0x%lX must be page aligned\n", bytes);
		return true;
	}

	if (num_pages == 0) {
		dev_err(pg_tbl->device,
			"requested mapping is less than one page: %lu / %lu\n",
			bytes, PAGE_SIZE);
		return true;
	}

	if (gasket_addr_is_simple(pg_tbl, dev_addr))
		return gasket_is_simple_dev_addr_bad(pg_tbl, dev_addr,
						     num_pages);
	return gasket_is_extended_dev_addr_bad(pg_tbl, dev_addr, num_pages);
}
EXPORT_SYMBOL(gasket_page_table_is_dev_addr_bad);

/* See gasket_page_table.h for description. */
uint gasket_page_table_max_size(struct gasket_page_table *page_table)
{
	if (!page_table)
		return 0;
	return page_table->config.total_entries;
}
EXPORT_SYMBOL(gasket_page_table_max_size);

/* See gasket_page_table.h for description. */
uint gasket_page_table_num_entries(struct gasket_page_table *pg_tbl)
{
	if (!pg_tbl)
		return 0;
	return pg_tbl->num_simple_entries + pg_tbl->num_extended_entries;
}
EXPORT_SYMBOL(gasket_page_table_num_entries);

/* See gasket_page_table.h for description. */
uint gasket_page_table_num_simple_entries(struct gasket_page_table *pg_tbl)
{
	if (!pg_tbl)
		return 0;
	return pg_tbl->num_simple_entries;
}
EXPORT_SYMBOL(gasket_page_table_num_simple_entries);

/* See gasket_page_table.h for description. */
uint gasket_page_table_num_active_pages(struct gasket_page_table *pg_tbl)
{
	if (!pg_tbl)
		return 0;
	return pg_tbl->num_active_pages;
}
EXPORT_SYMBOL(gasket_page_table_num_active_pages);

/* See gasket_page_table.h */
int gasket_page_table_system_status(struct gasket_page_table *page_table)
{
	if (!page_table)
		return GASKET_STATUS_LAMED;

	if (gasket_page_table_num_entries(page_table) == 0) {
		dev_dbg(page_table->device, "Page table size is 0\n");
		return GASKET_STATUS_LAMED;
	}

	return GASKET_STATUS_ALIVE;
}

/* Record the host_addr to coherent dma memory mapping. */
int gasket_set_user_virt(
	struct gasket_dev *gasket_dev, u64 size, dma_addr_t dma_address,
	ulong vma)
{
	int j;
	struct gasket_page_table *pg_tbl;

	unsigned int num_pages = size / PAGE_SIZE;

	/*
	 * TODO: for future chipset, better handling of the case where multiple
	 * page tables are supported on a given device
	 */
	pg_tbl = gasket_dev->page_table[0];
	if (!pg_tbl) {
		dev_dbg(gasket_dev->dev, "%s: invalid page table index\n",
			__func__);
		return 0;
	}
	for (j = 0; j < num_pages; j++) {
		pg_tbl->coherent_pages[j].user_virt =
			(u64)vma + j * PAGE_SIZE;
	}
	return 0;
}

/* Allocate a block of coherent memory. */
int gasket_alloc_coherent_memory(struct gasket_dev *gasket_dev, u64 size,
				 dma_addr_t *dma_address, u64 index)
{
	dma_addr_t handle;
	void *mem;
	int j;
	unsigned int num_pages = (size + PAGE_SIZE - 1) / PAGE_SIZE;
	const struct gasket_driver_desc *driver_desc =
		gasket_get_driver_desc(gasket_dev);

	if (!gasket_dev->page_table[index])
		return -EFAULT;

	if (num_pages == 0)
		return -EINVAL;

	mem = dma_alloc_coherent(gasket_get_device(gasket_dev),
				 num_pages * PAGE_SIZE, &handle, GFP_KERNEL);
	if (!mem)
		goto nomem;

	gasket_dev->page_table[index]->num_coherent_pages = num_pages;

	/* allocate the physical memory block */
	gasket_dev->page_table[index]->coherent_pages =
		kcalloc(num_pages, sizeof(struct gasket_coherent_page_entry),
			GFP_KERNEL);
	if (!gasket_dev->page_table[index]->coherent_pages)
		goto nomem;

	gasket_dev->coherent_buffer.length_bytes =
		PAGE_SIZE * (num_pages);
	gasket_dev->coherent_buffer.phys_base = handle;
	gasket_dev->coherent_buffer.virt_base = mem;

	*dma_address = driver_desc->coherent_buffer_description.base;
	for (j = 0; j < num_pages; j++) {
		gasket_dev->page_table[index]->coherent_pages[j].paddr =
			handle + j * PAGE_SIZE;
		gasket_dev->page_table[index]->coherent_pages[j].kernel_virt =
			(ulong)mem + j * PAGE_SIZE;
	}

	return 0;

nomem:
	if (mem) {
		dma_free_coherent(gasket_get_device(gasket_dev),
				  num_pages * PAGE_SIZE, mem, handle);
		gasket_dev->coherent_buffer.length_bytes = 0;
		gasket_dev->coherent_buffer.virt_base = NULL;
		gasket_dev->coherent_buffer.phys_base = 0;
	}

	kfree(gasket_dev->page_table[index]->coherent_pages);
	gasket_dev->page_table[index]->coherent_pages = NULL;
	gasket_dev->page_table[index]->num_coherent_pages = 0;
	return -ENOMEM;
}

/* Free a block of coherent memory. */
int gasket_free_coherent_memory(struct gasket_dev *gasket_dev, u64 size,
				dma_addr_t dma_address, u64 index)
{
	const struct gasket_driver_desc *driver_desc;

	if (!gasket_dev->page_table[index])
		return -EFAULT;

	driver_desc = gasket_get_driver_desc(gasket_dev);

	if (driver_desc->coherent_buffer_description.base != dma_address)
		return -EADDRNOTAVAIL;

	gasket_free_coherent_memory_all(gasket_dev, index);

	return 0;
}

/* Release all coherent memory. */
void gasket_free_coherent_memory_all(
	struct gasket_dev *gasket_dev, u64 index)
{
	if (!gasket_dev->page_table[index])
		return;

	if (gasket_dev->coherent_buffer.length_bytes) {
		dma_free_coherent(gasket_get_device(gasket_dev),
				  gasket_dev->coherent_buffer.length_bytes,
				  gasket_dev->coherent_buffer.virt_base,
				  gasket_dev->coherent_buffer.phys_base);
		gasket_dev->coherent_buffer.length_bytes = 0;
		gasket_dev->coherent_buffer.virt_base = NULL;
		gasket_dev->coherent_buffer.phys_base = 0;
	}

	kfree(gasket_dev->page_table[index]->coherent_pages);
	gasket_dev->page_table[index]->coherent_pages = NULL;
	gasket_dev->page_table[index]->num_coherent_pages = 0;
}
