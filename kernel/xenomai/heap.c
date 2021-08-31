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
#include <stdarg.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/log2.h>
#include <linux/bitops.h>
#include <linux/mm.h>
#include <asm/pgtable.h>
#include <cobalt/kernel/assert.h>
#include <cobalt/kernel/heap.h>
#include <cobalt/kernel/vfile.h>

/**
 * @ingroup cobalt_core
 * @defgroup cobalt_core_heap Dynamic memory allocation services
 *
 * This code implements a variant of the allocator described in
 * "Design of a General Purpose Memory Allocator for the 4.3BSD Unix
 * Kernel" by Marshall K. McKusick and Michael J. Karels (USENIX
 * 1988), see http://docs.FreeBSD.org/44doc/papers/kernmalloc.pdf.
 * The free page list is maintained in rbtrees for fast lookups of
 * multi-page memory ranges, and pages holding bucketed memory have a
 * fast allocation bitmap to manage their blocks internally.
 *@{
 */
struct xnheap cobalt_heap;		/* System heap */
EXPORT_SYMBOL_GPL(cobalt_heap);

static LIST_HEAD(heapq);	/* Heap list for v-file dump */

static int nrheaps;

#ifdef CONFIG_XENO_OPT_VFILE

static struct xnvfile_rev_tag vfile_tag;

static struct xnvfile_snapshot_ops vfile_ops;

struct vfile_priv {
	struct xnheap *curr;
};

struct vfile_data {
	size_t all_mem;
	size_t free_mem;
	char name[XNOBJECT_NAME_LEN];
};

static struct xnvfile_snapshot vfile = {
	.privsz = sizeof(struct vfile_priv),
	.datasz = sizeof(struct vfile_data),
	.tag = &vfile_tag,
	.ops = &vfile_ops,
};

static int vfile_rewind(struct xnvfile_snapshot_iterator *it)
{
	struct vfile_priv *priv = xnvfile_iterator_priv(it);

	if (list_empty(&heapq)) {
		priv->curr = NULL;
		return 0;
	}

	priv->curr = list_first_entry(&heapq, struct xnheap, next);

	return nrheaps;
}

static int vfile_next(struct xnvfile_snapshot_iterator *it, void *data)
{
	struct vfile_priv *priv = xnvfile_iterator_priv(it);
	struct vfile_data *p = data;
	struct xnheap *heap;

	if (priv->curr == NULL)
		return 0;	/* We are done. */

	heap = priv->curr;
	if (list_is_last(&heap->next, &heapq))
		priv->curr = NULL;
	else
		priv->curr = list_entry(heap->next.next,
					struct xnheap, next);

	p->all_mem = xnheap_get_size(heap);
	p->free_mem = xnheap_get_free(heap);
	knamecpy(p->name, heap->name);

	return 1;
}

static int vfile_show(struct xnvfile_snapshot_iterator *it, void *data)
{
	struct vfile_data *p = data;

	if (p == NULL)
		xnvfile_printf(it, "%9s %9s  %s\n",
			       "TOTAL", "FREE", "NAME");
	else
		xnvfile_printf(it, "%9zu %9zu  %s\n",
			       p->all_mem,
			       p->free_mem,
			       p->name);
	return 0;
}

static struct xnvfile_snapshot_ops vfile_ops = {
	.rewind = vfile_rewind,
	.next = vfile_next,
	.show = vfile_show,
};

void xnheap_init_proc(void)
{
	xnvfile_init_snapshot("heap", &vfile, &cobalt_vfroot);
}

void xnheap_cleanup_proc(void)
{
	xnvfile_destroy_snapshot(&vfile);
}

#endif /* CONFIG_XENO_OPT_VFILE */

enum xnheap_pgtype {
	page_free =0,
	page_cont =1,
	page_list =2
};

static inline u32 __always_inline
gen_block_mask(int log2size)
{
	return -1U >> (32 - (XNHEAP_PAGE_SIZE >> log2size));
}

static inline  __always_inline
int addr_to_pagenr(struct xnheap *heap, void *p)
{
	return ((void *)p - heap->membase) >> XNHEAP_PAGE_SHIFT;
}

static inline  __always_inline
void *pagenr_to_addr(struct xnheap *heap, int pg)
{
	return heap->membase + (pg << XNHEAP_PAGE_SHIFT);
}

#ifdef CONFIG_XENO_OPT_DEBUG_MEMORY
/*
 * Setting page_cont/page_free in the page map is only required for
 * enabling full checking of the block address in free requests, which
 * may be extremely time-consuming when deallocating huge blocks
 * spanning thousands of pages. We only do such marking when running
 * in memory debug mode.
 */
static inline bool
page_is_valid(struct xnheap *heap, int pg)
{
	switch (heap->pagemap[pg].type) {
	case page_free:
	case page_cont:
		return false;
	case page_list:
	default:
		return true;
	}
}

static void mark_pages(struct xnheap *heap,
		       int pg, int nrpages,
		       enum xnheap_pgtype type)
{
	while (nrpages-- > 0)
		heap->pagemap[pg].type = type;
}

#else

static inline bool
page_is_valid(struct xnheap *heap, int pg)
{
	return true;
}

static void mark_pages(struct xnheap *heap,
		       int pg, int nrpages,
		       enum xnheap_pgtype type)
{ }

#endif

static struct xnheap_range *
search_size_ge(struct rb_root *t, size_t size)
{
	struct rb_node *rb, *deepest = NULL;
	struct xnheap_range *r;
	
	/*
	 * We first try to find an exact match. If that fails, we walk
	 * the tree in logical order by increasing size value from the
	 * deepest node traversed until we find the first successor to
	 * that node, or nothing beyond it, whichever comes first.
	 */
	rb = t->rb_node;
	while (rb) {
		deepest = rb;
		r = rb_entry(rb, struct xnheap_range, size_node);
		if (size < r->size) {
			rb = rb->rb_left;
			continue;
		}
		if (size > r->size) {
			rb = rb->rb_right;
			continue;
		}
		return r;
	}

	rb = deepest;
	while (rb) {
		r = rb_entry(rb, struct xnheap_range, size_node);
		if (size <= r->size)
			return r;
		rb = rb_next(rb);
	}

	return NULL;
}

static struct xnheap_range *
search_left_mergeable(struct xnheap *heap, struct xnheap_range *r)
{
  	struct rb_node *node = heap->addr_tree.rb_node;
	struct xnheap_range *p;

  	while (node) {
		p = rb_entry(node, struct xnheap_range, addr_node);
		if ((void *)p + p->size == (void *)r)
			return p;
		if (&r->addr_node < node)
  			node = node->rb_left;
		else
  			node = node->rb_right;
	}

	return NULL;
}

static struct xnheap_range *
search_right_mergeable(struct xnheap *heap, struct xnheap_range *r)
{
  	struct rb_node *node = heap->addr_tree.rb_node;
	struct xnheap_range *p;

  	while (node) {
		p = rb_entry(node, struct xnheap_range, addr_node);
		if ((void *)r + r->size == (void *)p)
			return p;
		if (&r->addr_node < node)
  			node = node->rb_left;
		else
  			node = node->rb_right;
	}

	return NULL;
}

static void insert_range_bysize(struct xnheap *heap, struct xnheap_range *r)
{
  	struct rb_node **new = &heap->size_tree.rb_node, *parent = NULL;
	struct xnheap_range *p;

  	while (*new) {
  		p = container_of(*new, struct xnheap_range, size_node);
		parent = *new;
  		if (r->size <= p->size)
  			new = &((*new)->rb_left);
  		else
  			new = &((*new)->rb_right);
  	}

  	rb_link_node(&r->size_node, parent, new);
  	rb_insert_color(&r->size_node, &heap->size_tree);
}

static void insert_range_byaddr(struct xnheap *heap, struct xnheap_range *r)
{
  	struct rb_node **new = &heap->addr_tree.rb_node, *parent = NULL;
	struct xnheap_range *p;

  	while (*new) {
  		p = container_of(*new, struct xnheap_range, addr_node);
		parent = *new;
  		if (r < p)
  			new = &((*new)->rb_left);
  		else
  			new = &((*new)->rb_right);
  	}

  	rb_link_node(&r->addr_node, parent, new);
  	rb_insert_color(&r->addr_node, &heap->addr_tree);
}

static int reserve_page_range(struct xnheap *heap, size_t size)
{
	struct xnheap_range *new, *splitr;

	/* Find a suitable range of pages covering 'size'. */
	new = search_size_ge(&heap->size_tree, size);
	if (new == NULL)
		return -1;

	rb_erase(&new->size_node, &heap->size_tree);
	if (new->size == size) {
		rb_erase(&new->addr_node, &heap->addr_tree);
		return addr_to_pagenr(heap, new);
	}

	/*
	 * The free range fetched is larger than what we need: split
	 * it in two, the upper part is returned to the caller, the
	 * lower part is sent back to the free list, which makes
	 * reindexing by address pointless.
	 */
	splitr = new;
	splitr->size -= size;
	new = (struct xnheap_range *)((void *)new + splitr->size);
	insert_range_bysize(heap, splitr);

	return addr_to_pagenr(heap, new);
}

static void release_page_range(struct xnheap *heap,
			       void *page, size_t size)
{
	struct xnheap_range *freed = page, *left, *right;
	bool addr_linked = false;

	freed->size = size;

	left = search_left_mergeable(heap, freed);
	if (left) {
		rb_erase(&left->size_node, &heap->size_tree);
		left->size += freed->size;
		freed = left;
		addr_linked = true;
	}

	right = search_right_mergeable(heap, freed);
	if (right) {
		rb_erase(&right->size_node, &heap->size_tree);
		freed->size += right->size;
		if (addr_linked)
			rb_erase(&right->addr_node, &heap->addr_tree);
		else
			rb_replace_node(&right->addr_node, &freed->addr_node,
					&heap->addr_tree);
	} else if (!addr_linked)
		insert_range_byaddr(heap, freed);

	insert_range_bysize(heap, freed);
	mark_pages(heap, addr_to_pagenr(heap, page),
		   size >> XNHEAP_PAGE_SHIFT, page_free);
}

static void add_page_front(struct xnheap *heap,
			   int pg, int log2size)
{
	struct xnheap_pgentry *new, *head, *next;
	int ilog;

	/* Insert page at front of the per-bucket page list. */
	
	ilog = log2size - XNHEAP_MIN_LOG2;
	new = &heap->pagemap[pg];
	if (heap->buckets[ilog] == -1U) {
		heap->buckets[ilog] = pg;
		new->prev = new->next = pg;
	} else {
		head = &heap->pagemap[heap->buckets[ilog]];
		new->prev = heap->buckets[ilog];
		new->next = head->next;
		next = &heap->pagemap[new->next];
		next->prev = pg;
		head->next = pg;
		heap->buckets[ilog] = pg;
	}
}

static void remove_page(struct xnheap *heap,
			int pg, int log2size)
{
	struct xnheap_pgentry *old, *prev, *next;
	int ilog = log2size - XNHEAP_MIN_LOG2;

	/* Remove page from the per-bucket page list. */

	old = &heap->pagemap[pg];
	if (pg == old->next)
		heap->buckets[ilog] = -1U;
	else {
		if (pg == heap->buckets[ilog])
			heap->buckets[ilog] = old->next;
		prev = &heap->pagemap[old->prev];
		prev->next = old->next;
		next = &heap->pagemap[old->next];
		next->prev = old->prev;
	}
}

static void move_page_front(struct xnheap *heap,
			    int pg, int log2size)
{
	int ilog = log2size - XNHEAP_MIN_LOG2;

	/* Move page at front of the per-bucket page list. */
	
	if (heap->buckets[ilog] == pg)
		return;	 /* Already at front, no move. */
		
	remove_page(heap, pg, log2size);
	add_page_front(heap, pg, log2size);
}

static void move_page_back(struct xnheap *heap,
			   int pg, int log2size)
{
	struct xnheap_pgentry *old, *last, *head, *next;
	int ilog;

	/* Move page at end of the per-bucket page list. */
	
	old = &heap->pagemap[pg];
	if (pg == old->next) /* Singleton, no move. */
		return;
		
	remove_page(heap, pg, log2size);

	ilog = log2size - XNHEAP_MIN_LOG2;
	head = &heap->pagemap[heap->buckets[ilog]];
	last = &heap->pagemap[head->prev];
	old->prev = head->prev;
	old->next = last->next;
	next = &heap->pagemap[old->next];
	next->prev = pg;
	last->next = pg;
}

static void *add_free_range(struct xnheap *heap,
			    size_t bsize, int log2size)
{
	int pg;

	pg = reserve_page_range(heap, ALIGN(bsize, XNHEAP_PAGE_SIZE));
	if (pg < 0)
		return NULL;
	
	/*
	 * Update the page entry.  If @log2size is non-zero
	 * (i.e. bsize < XNHEAP_PAGE_SIZE), bsize is (1 << log2Size)
	 * between 2^XNHEAP_MIN_LOG2 and 2^(XNHEAP_PAGE_SHIFT - 1).
	 * Save the log2 power into entry.type, then update the
	 * per-page allocation bitmap to reserve the first block.
	 *
	 * Otherwise, we have a larger block which may span multiple
	 * pages: set entry.type to page_list, indicating the start of
	 * the page range, and entry.bsize to the overall block size.
	 */
	if (log2size) {
		heap->pagemap[pg].type = log2size;
		/*
		 * Mark the first object slot (#0) as busy, along with
		 * the leftmost bits we won't use for this log2 size.
		 */
		heap->pagemap[pg].map = ~gen_block_mask(log2size) | 1;
		/*
		 * Insert the new page at front of the per-bucket page
		 * list, enforcing the assumption that pages with free
		 * space live close to the head of this list.
		 */
		add_page_front(heap, pg, log2size);
	} else {
		heap->pagemap[pg].type = page_list;
		heap->pagemap[pg].bsize = (u32)bsize;
		mark_pages(heap, pg + 1,
			   (bsize >> XNHEAP_PAGE_SHIFT) - 1, page_cont);
	}

	heap->used_size += bsize;

	return pagenr_to_addr(heap, pg);
}

/**
 * @fn void *xnheap_alloc(struct xnheap *heap, size_t size)
 * @brief Allocate a memory block from a memory heap.
 *
 * Allocates a contiguous region of memory from an active memory heap.
 * Such allocation is guaranteed to be time-bounded.
 *
 * @param heap The descriptor address of the heap to get memory from.
 *
 * @param size The size in bytes of the requested block.
 *
 * @return The address of the allocated region upon success, or NULL
 * if no memory is available from the specified heap.
 *
 * @coretags{unrestricted}
 */
void *xnheap_alloc(struct xnheap *heap, size_t size)
{
	int log2size, ilog, pg, b = -1;
	size_t bsize;
	void *block;
	spl_t s;

	if (size == 0)
		return NULL;

	if (size < XNHEAP_MIN_ALIGN) {
		bsize = size = XNHEAP_MIN_ALIGN;
		log2size = XNHEAP_MIN_LOG2;
	} else {
		log2size = ilog2(size);
		if (log2size < XNHEAP_PAGE_SHIFT) {
			if (size & (size - 1))
				log2size++;
			bsize = 1 << log2size;
		} else
			bsize = ALIGN(size, XNHEAP_PAGE_SIZE);
	}
	
	/*
	 * Allocate entire pages directly from the pool whenever the
	 * block is larger or equal to XNHEAP_PAGE_SIZE.  Otherwise,
	 * use bucketed memory.
	 *
	 * NOTE: Fully busy pages from bucketed memory are moved back
	 * at the end of the per-bucket page list, so that we may
	 * always assume that either the heading page has some room
	 * available, or no room is available from any page linked to
	 * this list, in which case we should immediately add a fresh
	 * page.
	 */
	xnlock_get_irqsave(&heap->lock, s);

	if (bsize >= XNHEAP_PAGE_SIZE)
		/* Add a range of contiguous free pages. */
		block = add_free_range(heap, bsize, 0);
	else {
		ilog = log2size - XNHEAP_MIN_LOG2;
		XENO_WARN_ON(MEMORY, ilog < 0 || ilog >= XNHEAP_MAX_BUCKETS);
		pg = heap->buckets[ilog];
		/*
		 * Find a block in the heading page if any. If there
		 * is none, there won't be any down the list: add a
		 * new page right away.
		 */
		if (pg < 0 || heap->pagemap[pg].map == -1U)
			block = add_free_range(heap, bsize, log2size);
		else {
			b = ffs(~heap->pagemap[pg].map) - 1;
			/*
			 * Got one block from the heading per-bucket
			 * page, tag it as busy in the per-page
			 * allocation map.
			 */
			heap->pagemap[pg].map |= (1U << b);
			heap->used_size += bsize;
			block = heap->membase +
				(pg << XNHEAP_PAGE_SHIFT) +
				(b << log2size);
			if (heap->pagemap[pg].map == -1U)
				move_page_back(heap, pg, log2size);
		}
	}

	xnlock_put_irqrestore(&heap->lock, s);

	return block;
}
EXPORT_SYMBOL_GPL(xnheap_alloc);

/**
 * @fn void xnheap_free(struct xnheap *heap, void *block)
 * @brief Release a block to a memory heap.
 *
 * Releases a memory block to a heap.
 *
 * @param heap The heap descriptor.
 *
 * @param block The block to be returned to the heap.
 *
 * @coretags{unrestricted}
 */
void xnheap_free(struct xnheap *heap, void *block)
{
	unsigned long pgoff, boff;
	int log2size, pg, n;
	size_t bsize;
	u32 oldmap;
	spl_t s;

	xnlock_get_irqsave(&heap->lock, s);

	/* Compute the heading page number in the page map. */
	pgoff = block - heap->membase;
	pg = pgoff >> XNHEAP_PAGE_SHIFT;

	if (!page_is_valid(heap, pg))
		goto bad;
	
	switch (heap->pagemap[pg].type) {
	case page_list:
		bsize = heap->pagemap[pg].bsize;
		XENO_WARN_ON(MEMORY, (bsize & (XNHEAP_PAGE_SIZE - 1)) != 0);
		release_page_range(heap, pagenr_to_addr(heap, pg), bsize);
		break;

	default:
		log2size = heap->pagemap[pg].type;
		bsize = (1 << log2size);
		XENO_WARN_ON(MEMORY, bsize >= XNHEAP_PAGE_SIZE);
		boff = pgoff & ~XNHEAP_PAGE_MASK;
		if ((boff & (bsize - 1)) != 0) /* Not at block start? */
			goto bad;

		n = boff >> log2size; /* Block position in page. */
		oldmap = heap->pagemap[pg].map;
		heap->pagemap[pg].map &= ~(1U << n);

		/*
		 * If the page the block was sitting on is fully idle,
		 * return it to the pool. Otherwise, check whether
		 * that page is transitioning from fully busy to
		 * partially busy state, in which case it should move
		 * toward the front of the per-bucket page list.
		 */
		if (heap->pagemap[pg].map == ~gen_block_mask(log2size)) {
			remove_page(heap, pg, log2size);
			release_page_range(heap, pagenr_to_addr(heap, pg),
					   XNHEAP_PAGE_SIZE);
		} else if (oldmap == -1U)
			move_page_front(heap, pg, log2size);
	}

	heap->used_size -= bsize;

	xnlock_put_irqrestore(&heap->lock, s);

	return;
bad:
	xnlock_put_irqrestore(&heap->lock, s);

	XENO_WARN(MEMORY, 1, "invalid block %p in heap %s",
		  block, heap->name);
}
EXPORT_SYMBOL_GPL(xnheap_free);

ssize_t xnheap_check_block(struct xnheap *heap, void *block)
{
	unsigned long pg, pgoff, boff;
	ssize_t ret = -EINVAL;
	size_t bsize;
	spl_t s;

	xnlock_get_irqsave(&heap->lock, s);

	/* Calculate the page number from the block address. */
	pgoff = block - heap->membase;
	pg = pgoff >> XNHEAP_PAGE_SHIFT;
	if (page_is_valid(heap, pg)) {
		if (heap->pagemap[pg].type == page_list)
			bsize = heap->pagemap[pg].bsize;
		else {
			bsize = (1 << heap->pagemap[pg].type);
			boff = pgoff & ~XNHEAP_PAGE_MASK;
			if ((boff & (bsize - 1)) != 0) /* Not at block start? */
				goto out;
		}
		ret = (ssize_t)bsize;
	}
out:
	xnlock_put_irqrestore(&heap->lock, s);

	return ret;
}
EXPORT_SYMBOL_GPL(xnheap_check_block);

/**
 * @fn xnheap_init(struct xnheap *heap, void *membase, u32 size)
 * @brief Initialize a memory heap.
 *
 * Initializes a memory heap suitable for time-bounded allocation
 * requests of dynamic memory.
 *
 * @param heap The address of a heap descriptor to initialize.
 *
 * @param membase The address of the storage area.
 *
 * @param size The size in bytes of the storage area.  @a size must be
 * a multiple of XNHEAP_PAGE_SIZE and smaller than (4Gb - PAGE_SIZE)
 * in the current implementation.
 *
 * @return 0 is returned upon success, or:
 *
 * - -EINVAL is returned if @a size is either greater than
 *   XNHEAP_MAX_HEAPSZ, or not aligned on PAGE_SIZE.
 *
 * - -ENOMEM is returned upon failure of allocating the meta-data area
 * used internally to maintain the heap.
 *
 * @coretags{secondary-only}
 */
int xnheap_init(struct xnheap *heap, void *membase, size_t size)
{
	int n, nrpages;
	spl_t s;

	secondary_mode_only();

 	if (size > XNHEAP_MAX_HEAPSZ || !PAGE_ALIGNED(size))
		return -EINVAL;

	/* Reset bucket page lists, all empty. */
	for (n = 0; n < XNHEAP_MAX_BUCKETS; n++)
		heap->buckets[n] = -1U;

	xnlock_init(&heap->lock);

	nrpages = size >> XNHEAP_PAGE_SHIFT;
	heap->pagemap = vzalloc(sizeof(struct xnheap_pgentry) * nrpages);
	if (heap->pagemap == NULL)
		return -ENOMEM;

	heap->membase = membase;
	heap->usable_size = size;
	heap->used_size = 0;
		      
	/*
	 * The free page pool is maintained as a set of ranges of
	 * contiguous pages indexed by address and size in rbtrees.
	 * Initially, we have a single range in those trees covering
	 * the whole memory we have been given for the heap. Over
	 * time, that range will be split then possibly re-merged back
	 * as allocations and deallocations take place.
	 */
	heap->size_tree = RB_ROOT;
	heap->addr_tree = RB_ROOT;
	release_page_range(heap, membase, size);

	/* Default name, override with xnheap_set_name() */
	ksformat(heap->name, sizeof(heap->name), "(%p)", heap);

	xnlock_get_irqsave(&nklock, s);
	list_add_tail(&heap->next, &heapq);
	nrheaps++;
	xnvfile_touch_tag(&vfile_tag);
	xnlock_put_irqrestore(&nklock, s);

	return 0;
}
EXPORT_SYMBOL_GPL(xnheap_init);

/**
 * @fn void xnheap_destroy(struct xnheap *heap)
 * @brief Destroys a memory heap.
 *
 * Destroys a memory heap.
 *
 * @param heap The heap descriptor.
 *
 * @coretags{secondary-only}
 */
void xnheap_destroy(struct xnheap *heap)
{
	spl_t s;

	secondary_mode_only();

	xnlock_get_irqsave(&nklock, s);
	list_del(&heap->next);
	nrheaps--;
	xnvfile_touch_tag(&vfile_tag);
	xnlock_put_irqrestore(&nklock, s);
	vfree(heap->pagemap);
}
EXPORT_SYMBOL_GPL(xnheap_destroy);

/**
 * @fn xnheap_set_name(struct xnheap *heap,const char *name,...)
 * @brief Set the heap's name string.
 *
 * Set the heap name that will be used in statistic outputs.
 *
 * @param heap The address of a heap descriptor.
 *
 * @param name Name displayed in statistic outputs. This parameter can
 * be a printk()-like format argument list.
 *
 * @coretags{task-unrestricted}
 */
void xnheap_set_name(struct xnheap *heap, const char *name, ...)
{
	va_list args;

	va_start(args, name);
	kvsformat(heap->name, sizeof(heap->name), name, args);
	va_end(args);
}
EXPORT_SYMBOL_GPL(xnheap_set_name);

void *xnheap_vmalloc(size_t size)
{
	/*
	 * We want memory used in real-time context to be pulled from
	 * ZONE_NORMAL, however we don't need it to be physically
	 * contiguous.
	 *
	 * 32bit systems which would need HIGHMEM for running a Cobalt
	 * configuration would also be required to support PTE
	 * pinning, which not all architectures provide.  Moreover,
	 * pinning PTEs eagerly for a potentially (very) large amount
	 * of memory may quickly degrade performance.
	 *
	 * If using a different kernel/user memory split cannot be the
	 * answer for those configs, it's likely that basing such
	 * software on a 32bit system had to be wrong in the first
	 * place anyway.
	 */
	return __vmalloc(size, GFP_KERNEL, PAGE_KERNEL);
}
EXPORT_SYMBOL_GPL(xnheap_vmalloc);

void xnheap_vfree(void *p)
{
	vfree(p);
}
EXPORT_SYMBOL_GPL(xnheap_vfree);

/** @} */
