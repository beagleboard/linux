/*
 * Copyright (C) 2001,2002,2003 Philippe Gerum <rpm@xenomai.org>.
 *
 * Xenomai is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version.
 *
 * Xenomai is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Xenomai; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 * 02111-1307, USA.
 */
#ifndef _COBALT_KERNEL_HEAP_H
#define _COBALT_KERNEL_HEAP_H

#include <linux/string.h>
#include <cobalt/kernel/lock.h>
#include <cobalt/kernel/list.h>
#include <cobalt/uapi/kernel/types.h>
#include <cobalt/uapi/kernel/heap.h>

/**
 * @addtogroup cobalt_core_heap
 * @{
 *
 * @par Implementation constraints
 *
 * - Minimum page size is 2 ** XNHEAP_MINLOG2 (must be large enough to
 * hold a pointer).
 *
 * - Maximum page size is 2 ** XNHEAP_MAXLOG2.
 *
 * - Requested block size is rounded up to XNHEAP_MINLOG2.
 *
 * - Requested block size larger than 2 times the XNHEAP_PAGESZ is
 * rounded up to the next page boundary and obtained from the free
 * page list. So we need a bucket for each power of two between
 * XNHEAP_MINLOG2 and XNHEAP_MAXLOG2 inclusive, plus one to honor
 * requests ranging from the maximum page size to twice this size.
 */
#define XNHEAP_PAGESZ	  PAGE_SIZE
#define XNHEAP_MINLOG2    3
#define XNHEAP_MAXLOG2    22	/* Holds pagemap.bcount blocks */
#define XNHEAP_MINALLOCSZ (1 << XNHEAP_MINLOG2)
#define XNHEAP_MINALIGNSZ (1 << 4) /* i.e. 16 bytes */
#define XNHEAP_NBUCKETS   (XNHEAP_MAXLOG2 - XNHEAP_MINLOG2 + 2)
#define XNHEAP_MAXHEAPSZ  (1 << 31) /* i.e. 2Gb */

#define XNHEAP_PFREE   0
#define XNHEAP_PCONT   1
#define XNHEAP_PLIST   2

struct xnpagemap {
	/** PFREE, PCONT, PLIST or log2 */
	u32 type : 8;
	/** Number of active blocks */
	u32 bcount : 24;
};

struct xnheap {
	/** SMP lock */
	DECLARE_XNLOCK(lock);
	/** Base address of the page array */
	caddr_t membase;
	/** Memory limit of page array */
	caddr_t memlim;
	/** Number of pages in the freelist */
	int npages;
	/** Head of the free page list */
	caddr_t freelist;
	/** Address of the page map */
	struct xnpagemap *pagemap;
	/** Link to heapq */
	struct list_head next;
	/** log2 bucket list */
	struct xnbucket {
		caddr_t freelist;
		int fcount;
	} buckets[XNHEAP_NBUCKETS];
	char name[XNOBJECT_NAME_LEN];
	/** Size of storage area */
	u32 size;
	/** Used/busy storage size */
	u32 used;
};

extern struct xnheap cobalt_heap;

#define xnmalloc(size)     xnheap_alloc(&cobalt_heap, size)
#define xnfree(ptr)        xnheap_free(&cobalt_heap, ptr)

static inline u32 xnheap_get_size(const struct xnheap *heap)
{
	return heap->size;
}

static inline u32 xnheap_get_free(const struct xnheap *heap)
{
	return heap->size - heap->used;
}

static inline void *xnheap_get_membase(const struct xnheap *heap)
{
	return heap->membase;
}

static inline u32 xnheap_rounded_size(u32 size)
{
	if (size < 2 * XNHEAP_PAGESZ)
		return 2 * XNHEAP_PAGESZ;

	return ALIGN(size, XNHEAP_PAGESZ);
}

/* Private interface. */

#ifdef CONFIG_XENO_OPT_VFILE
void xnheap_init_proc(void);
void xnheap_cleanup_proc(void);
#else /* !CONFIG_XENO_OPT_VFILE */
static inline void xnheap_init_proc(void) { }
static inline void xnheap_cleanup_proc(void) { }
#endif /* !CONFIG_XENO_OPT_VFILE */

/* Public interface. */

void *xnheap_vmalloc(size_t size);

void xnheap_vfree(void *p);

int xnheap_init(struct xnheap *heap, void *membase, u32 size);

void xnheap_set_name(struct xnheap *heap,
		     const char *name, ...);

void xnheap_destroy(struct xnheap *heap);

void *xnheap_alloc(struct xnheap *heap, u32 size);

void xnheap_free(struct xnheap *heap, void *block);

int xnheap_check_block(struct xnheap *heap, void *block);

static inline char *xnstrdup(const char *s)
{
	char *p;

	p = xnmalloc(strlen(s) + 1);
	if (p == NULL)
		return NULL;

	return strcpy(p, s);
}

/** @} */

#endif /* !_COBALT_KERNEL_HEAP_H */
