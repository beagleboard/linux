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
#include <linux/rbtree.h>
#include <cobalt/kernel/lock.h>
#include <cobalt/kernel/list.h>
#include <cobalt/uapi/kernel/types.h>
#include <cobalt/uapi/kernel/heap.h>

/**
 * @addtogroup cobalt_core_heap
 * @{
 */

#define XNHEAP_PAGE_SHIFT	9 /* 2^9 => 512 bytes */
#define XNHEAP_PAGE_SIZE	(1UL << XNHEAP_PAGE_SHIFT)
#define XNHEAP_PAGE_MASK	(~(XNHEAP_PAGE_SIZE - 1))
#define XNHEAP_MIN_LOG2		4 /* 16 bytes */
/*
 * Use bucketed memory for sizes between 2^XNHEAP_MIN_LOG2 and
 * 2^(XNHEAP_PAGE_SHIFT-1).
 */
#define XNHEAP_MAX_BUCKETS	(XNHEAP_PAGE_SHIFT - XNHEAP_MIN_LOG2)
#define XNHEAP_MIN_ALIGN	(1U << XNHEAP_MIN_LOG2)
/* Maximum size of a heap (4Gb - PAGE_SIZE). */
#define XNHEAP_MAX_HEAPSZ	(4294967295U - PAGE_SIZE + 1)
/* Bits we need for encoding a page # */
#define XNHEAP_PGENT_BITS      (32 - XNHEAP_PAGE_SHIFT)
/* Each page is represented by a page map entry. */
#define XNHEAP_PGMAP_BYTES	sizeof(struct xnheap_pgentry)

struct xnheap_pgentry {
	/* Linkage in bucket list. */
	unsigned int prev : XNHEAP_PGENT_BITS;
	unsigned int next : XNHEAP_PGENT_BITS;
	/*  page_list or log2. */
	unsigned int type : 6;
	/*
	 * We hold either a spatial map of busy blocks within the page
	 * for bucketed memory (up to 32 blocks per page), or the
	 * overall size of the multi-page block if entry.type ==
	 * page_list.
	 */
	union {
		u32 map;
		u32 bsize;
	};
};

/*
 * A range descriptor is stored at the beginning of the first page of
 * a range of free pages. xnheap_range.size is nrpages *
 * XNHEAP_PAGE_SIZE. Ranges are indexed by address and size in
 * rbtrees.
 */
struct xnheap_range {
	struct rb_node addr_node;
	struct rb_node size_node;
	size_t size;
};

struct xnheap {
	void *membase;
	struct rb_root addr_tree;
	struct rb_root size_tree;
	struct xnheap_pgentry *pagemap;
	size_t usable_size;
	size_t used_size;
	u32 buckets[XNHEAP_MAX_BUCKETS];
	char name[XNOBJECT_NAME_LEN];
	DECLARE_XNLOCK(lock);
	struct list_head next;
};

extern struct xnheap cobalt_heap;

#define xnmalloc(size)     xnheap_alloc(&cobalt_heap, size)
#define xnfree(ptr)        xnheap_free(&cobalt_heap, ptr)

static inline void *xnheap_get_membase(const struct xnheap *heap)
{
	return heap->membase;
}

static inline
size_t xnheap_get_size(const struct xnheap *heap)
{
	return heap->usable_size;
}

static inline
size_t xnheap_get_used(const struct xnheap *heap)
{
	return heap->used_size;
}

static inline
size_t xnheap_get_free(const struct xnheap *heap)
{
	return heap->usable_size - heap->used_size;
}

int xnheap_init(struct xnheap *heap,
		void *membase, size_t size);

void xnheap_destroy(struct xnheap *heap);

void *xnheap_alloc(struct xnheap *heap, size_t size);

void xnheap_free(struct xnheap *heap, void *block);

ssize_t xnheap_check_block(struct xnheap *heap, void *block);

void xnheap_set_name(struct xnheap *heap,
		     const char *name, ...);

void *xnheap_vmalloc(size_t size);

void xnheap_vfree(void *p);

static inline void *xnheap_zalloc(struct xnheap *heap, size_t size)
{
	void *p;

	p = xnheap_alloc(heap, size);
	if (p)
		memset(p, 0, size);

	return p;
}

static inline char *xnstrdup(const char *s)
{
	char *p;

	p = xnmalloc(strlen(s) + 1);
	if (p == NULL)
		return NULL;

	return strcpy(p, s);
}

#ifdef CONFIG_XENO_OPT_VFILE
void xnheap_init_proc(void);
void xnheap_cleanup_proc(void);
#else /* !CONFIG_XENO_OPT_VFILE */
static inline void xnheap_init_proc(void) { }
static inline void xnheap_cleanup_proc(void) { }
#endif /* !CONFIG_XENO_OPT_VFILE */

/** @} */

#endif /* !_COBALT_KERNEL_HEAP_H */
