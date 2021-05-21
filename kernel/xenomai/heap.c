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
#include <linux/kconfig.h>
#include <asm/pgtable.h>
#include <cobalt/kernel/assert.h>
#include <cobalt/kernel/heap.h>
#include <cobalt/kernel/vfile.h>

/**
 * @ingroup cobalt_core
 * @defgroup cobalt_core_heap Dynamic memory allocation services
 *
 * The implementation of the memory allocator follows the algorithm
 * described in a USENIX 1988 paper called "Design of a General
 * Purpose Memory Allocator for the 4.3BSD Unix Kernel" by Marshall
 * K. McKusick and Michael J. Karels. You can find it at various
 * locations on the net, including
 * http://docs.FreeBSD.org/44doc/papers/kernmalloc.pdf.
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

static void init_freelist(struct xnheap *heap)
{
	caddr_t freepage;
	int n, lastpgnum;

	heap->used = 0;
	memset(heap->buckets, 0, sizeof(heap->buckets));
	lastpgnum = heap->npages - 1;

	/* Mark each page as free in the page map. */
	for (n = 0, freepage = heap->membase;
	     n < lastpgnum; n++, freepage += XNHEAP_PAGESZ) {
		*((caddr_t *)freepage) = freepage + XNHEAP_PAGESZ;
		heap->pagemap[n].type = XNHEAP_PFREE;
		heap->pagemap[n].bcount = 0;
	}

	*((caddr_t *) freepage) = NULL;
	heap->pagemap[lastpgnum].type = XNHEAP_PFREE;
	heap->pagemap[lastpgnum].bcount = 0;
	heap->memlim = freepage + XNHEAP_PAGESZ;

	/* The first page starts the free list. */
	heap->freelist = heap->membase;
}

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
 * @param size The size in bytes of the storage area.  @a size
 * must be a multiple of PAGE_SIZE and smaller than 2 Gb in the
 * current implementation.
 *
 * @return 0 is returned upon success, or:
 *
 * - -EINVAL is returned if @a size is either:
 *
 *   - not aligned on PAGE_SIZE
 *   - smaller than 2 * PAGE_SIZE
 *   - greater than 2 Gb (XNHEAP_MAXHEAPSZ)
 *
 * - -ENOMEM is returned upon failure of allocating the meta-data area
 * used internally to maintain the heap.
 *
 * @coretags{secondary-only}
 */
int xnheap_init(struct xnheap *heap, void *membase, u32 size)
{
	spl_t s;

	secondary_mode_only();

	/*
	 * HEAPSIZE must be aligned on XNHEAP_PAGESZ.
	 * HEAPSIZE must be lower than XNHEAP_MAXHEAPSZ.
	 */
 	if (size > XNHEAP_MAXHEAPSZ ||
	    !IS_ALIGNED(size, XNHEAP_PAGESZ))
		return -EINVAL;

	/*
	 * We need to reserve 4 bytes in a page map for each page
	 * which is addressable into the storage area.  pmapsize =
	 * (size / XNHEAP_PAGESZ) * sizeof(struct xnpagemap).
	 */
	heap->size = size;
	heap->membase = membase;
	heap->npages = size / XNHEAP_PAGESZ;
	/*
	 * The heap must contain at least two addressable pages to
	 * cope with allocation sizes between XNHEAP_PAGESZ and 2 *
	 * XNHEAP_PAGESZ.
	 */
	if (heap->npages < 2)
		return -EINVAL;

	heap->pagemap = kmalloc(sizeof(struct xnpagemap) * heap->npages,
				GFP_KERNEL);
	if (heap->pagemap == NULL)
		return -ENOMEM;

	xnlock_init(&heap->lock);
	init_freelist(heap);

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
	kfree(heap->pagemap);
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

/*
 * get_free_range() -- Obtain a range of contiguous free pages to
 * fulfill an allocation of 2 ** log2size.  The caller must have
 * acquired the heap lock.
 */

static caddr_t get_free_range(struct xnheap *heap, u32 bsize, int log2size)
{
	caddr_t block, eblock, freepage, lastpage, headpage, freehead = NULL;
	u32 pagenum, pagecont, freecont;

	freepage = heap->freelist;
	while (freepage) {
		headpage = freepage;
		freecont = 0;
		/*
		 * Search for a range of contiguous pages in the free
		 * page list. The range must be 'bsize' long.
		 */
		do {
			lastpage = freepage;
			freepage = *((caddr_t *) freepage);
			freecont += XNHEAP_PAGESZ;
		}
		while (freepage == lastpage + XNHEAP_PAGESZ &&
		       freecont < bsize);

		if (freecont >= bsize) {
			/*
			 * Ok, got it. Just update the free page list,
			 * then proceed to the next step.
			 */
			if (headpage == heap->freelist)
				heap->freelist = *((caddr_t *)lastpage);
			else
				*((caddr_t *)freehead) = *((caddr_t *)lastpage);

			goto splitpage;
		}
		freehead = lastpage;
	}

	return NULL;

splitpage:

	/*
	 * At this point, headpage is valid and points to the first
	 * page of a range of contiguous free pages larger or equal
	 * than 'bsize'.
	 */
	if (bsize < XNHEAP_PAGESZ) {
		/*
		 * If the allocation size is smaller than the page
		 * size, split the page in smaller blocks of this
		 * size, building a free list of free blocks.
		 */
		for (block = headpage, eblock =
			     headpage + XNHEAP_PAGESZ - bsize; block < eblock;
		     block += bsize)
			*((caddr_t *)block) = block + bsize;

		*((caddr_t *)eblock) = NULL;
	} else
		*((caddr_t *)headpage) = NULL;

	pagenum = (headpage - heap->membase) / XNHEAP_PAGESZ;

	/*
	 * Update the page map.  If log2size is non-zero (i.e. bsize
	 * <= 2 * pagesize), store it in the first page's slot to
	 * record the exact block size (which is a power of
	 * two). Otherwise, store the special marker XNHEAP_PLIST,
	 * indicating the start of a block whose size is a multiple of
	 * the standard page size, but not necessarily a power of two.
	 * In any case, the following pages slots are marked as
	 * 'continued' (PCONT).
	 */
	heap->pagemap[pagenum].type = log2size ? : XNHEAP_PLIST;
	heap->pagemap[pagenum].bcount = 1;

	for (pagecont = bsize / XNHEAP_PAGESZ; pagecont > 1; pagecont--) {
		heap->pagemap[pagenum + pagecont - 1].type = XNHEAP_PCONT;
		heap->pagemap[pagenum + pagecont - 1].bcount = 0;
	}

	return headpage;
}

/**
 * @fn void *xnheap_alloc(struct xnheap *heap, u32 size)
 * @brief Allocate a memory block from a memory heap.
 *
 * Allocates a contiguous region of memory from an active memory heap.
 * Such allocation is guaranteed to be time-bounded.
 *
 * @param heap The descriptor address of the heap to get memory from.
 *
 * @param size The size in bytes of the requested block. Sizes lower
 * or equal to the page size are rounded either to the minimum
 * allocation size if lower than this value, or to the minimum
 * alignment size if greater or equal to this value. In the current
 * implementation, with MINALLOC = 8 and MINALIGN = 16, a 7 bytes
 * request will be rounded to 8 bytes, and a 17 bytes request will be
 * rounded to 32.
 *
 * @return The address of the allocated region upon success, or NULL
 * if no memory is available from the specified heap.
 *
 * @coretags{unrestricted}
 */
void *xnheap_alloc(struct xnheap *heap, u32 size)
{
	u32 pagenum, bsize;
	int log2size, ilog;
	caddr_t block;
	spl_t s;

	if (size == 0)
		return NULL;

	/*
	 * Sizes lower or equal to the page size are rounded either to
	 * the minimum allocation size if lower than this value, or to
	 * the minimum alignment size if greater or equal to this
	 * value.
	 */
	if (size > XNHEAP_PAGESZ)
		size = ALIGN(size, XNHEAP_PAGESZ);
	else if (size <= XNHEAP_MINALIGNSZ)
		size = ALIGN(size, XNHEAP_MINALLOCSZ);
	else
		size = ALIGN(size, XNHEAP_MINALIGNSZ);

	/*
	 * It is more space efficient to directly allocate pages from
	 * the free page list whenever the requested size is greater
	 * than 2 times the page size. Otherwise, use the bucketed
	 * memory blocks.
	 */
	if (likely(size <= XNHEAP_PAGESZ * 2)) {
		/*
		 * Find the first power of two greater or equal to the
		 * rounded size.
		 */
		bsize = size < XNHEAP_MINALLOCSZ ? XNHEAP_MINALLOCSZ : size;
		log2size = order_base_2(bsize);
		bsize = 1 << log2size;
		ilog = log2size - XNHEAP_MINLOG2;
		xnlock_get_irqsave(&heap->lock, s);
		block = heap->buckets[ilog].freelist;
		if (block == NULL) {
			block = get_free_range(heap, bsize, log2size);
			if (block == NULL)
				goto out;
			if (bsize <= XNHEAP_PAGESZ)
				heap->buckets[ilog].fcount += (XNHEAP_PAGESZ >> log2size) - 1;
		} else {
			if (bsize <= XNHEAP_PAGESZ)
				--heap->buckets[ilog].fcount;
			XENO_BUG_ON(COBALT, (caddr_t)block < heap->membase ||
				    (caddr_t)block >= heap->memlim);
			pagenum = ((caddr_t)block - heap->membase) / XNHEAP_PAGESZ;
			++heap->pagemap[pagenum].bcount;
		}
		heap->buckets[ilog].freelist = *((caddr_t *)block);
		heap->used += bsize;
	} else {
		if (size > heap->size)
			return NULL;

		xnlock_get_irqsave(&heap->lock, s);

		/* Directly request a free page range. */
		block = get_free_range(heap, size, 0);
		if (block)
			heap->used += size;
	}
out:
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
	caddr_t freepage, lastpage, nextpage, tailpage, freeptr, *tailptr;
	int log2size, npages, nblocks, xpage, ilog;
	u32 pagenum, pagecont, boffset, bsize;
	spl_t s;

	xnlock_get_irqsave(&heap->lock, s);

	if ((caddr_t)block < heap->membase || (caddr_t)block >= heap->memlim)
		goto bad_block;

	/* Compute the heading page number in the page map. */
	pagenum = ((caddr_t)block - heap->membase) / XNHEAP_PAGESZ;
	boffset = ((caddr_t)block - (heap->membase + pagenum * XNHEAP_PAGESZ));

	switch (heap->pagemap[pagenum].type) {
	case XNHEAP_PFREE:	/* Unallocated page? */
	case XNHEAP_PCONT:	/* Not a range heading page? */
	bad_block:
		xnlock_put_irqrestore(&heap->lock, s);
		XENO_BUG(COBALT);
		return;

	case XNHEAP_PLIST:
		npages = 1;
		while (npages < heap->npages &&
		       heap->pagemap[pagenum + npages].type == XNHEAP_PCONT)
			npages++;

		bsize = npages * XNHEAP_PAGESZ;

	free_page_list:
		/* Link all freed pages in a single sub-list. */
		for (freepage = (caddr_t) block,
			     tailpage = (caddr_t) block + bsize - XNHEAP_PAGESZ;
		     freepage < tailpage; freepage += XNHEAP_PAGESZ)
			*((caddr_t *) freepage) = freepage + XNHEAP_PAGESZ;

	free_pages:
		/* Mark the released pages as free. */
		for (pagecont = 0; pagecont < npages; pagecont++)
			heap->pagemap[pagenum + pagecont].type = XNHEAP_PFREE;

		/*
		 * Return the sub-list to the free page list, keeping
		 * an increasing address order to favor coalescence.
		 */
		for (nextpage = heap->freelist, lastpage = NULL;
		     nextpage != NULL && nextpage < (caddr_t) block;
		     lastpage = nextpage, nextpage = *((caddr_t *)nextpage))
			;	/* Loop */

		*((caddr_t *)tailpage) = nextpage;

		if (lastpage)
			*((caddr_t *)lastpage) = (caddr_t)block;
		else
			heap->freelist = (caddr_t)block;
		break;

	default:
		log2size = heap->pagemap[pagenum].type;
		bsize = (1 << log2size);
		if ((boffset & (bsize - 1)) != 0) /* Not a block start? */
			goto bad_block;

		/*
		 * Return the page to the free list if we've just
		 * freed its last busy block. Pages from multi-page
		 * blocks are always pushed to the free list (bcount
		 * value for the heading page is always 1).
		 */

		ilog = log2size - XNHEAP_MINLOG2;
		if (likely(--heap->pagemap[pagenum].bcount > 0)) {
			/* Return the block to the bucketed memory space. */
			*((caddr_t *)block) = heap->buckets[ilog].freelist;
			heap->buckets[ilog].freelist = block;
			++heap->buckets[ilog].fcount;
			break;
		}

		/*
		 * In the simplest case, we only have a single block
		 * to deal with, which spans multiple pages. We just
		 * need to release it as a list of pages, without
		 * caring about the consistency of the bucket.
		 */
		npages = bsize / XNHEAP_PAGESZ;
		if (unlikely(npages > 1))
			goto free_page_list;

		freepage = heap->membase + pagenum * XNHEAP_PAGESZ;
		block = freepage;
		tailpage = freepage;
		nextpage = freepage + XNHEAP_PAGESZ;
		nblocks = XNHEAP_PAGESZ >> log2size;
		heap->buckets[ilog].fcount -= (nblocks - 1);
		XENO_BUG_ON(COBALT, heap->buckets[ilog].fcount < 0);

		/*
		 * Still easy case: all free blocks are laid on a
		 * single page we are now releasing. Just clear the
		 * bucket and bail out.
		 */
		if (likely(heap->buckets[ilog].fcount == 0)) {
			heap->buckets[ilog].freelist = NULL;
			goto free_pages;
		}

		/*
		 * Worst case: multiple pages are traversed by the
		 * bucket list. Scan the list to remove all blocks
		 * belonging to the freed page. We are done whenever
		 * all possible blocks from the freed page have been
		 * traversed, or we hit the end of list, whichever
		 * comes first.
		 */
		for (tailptr = &heap->buckets[ilog].freelist, freeptr = *tailptr, xpage = 1;
		     freeptr != NULL && nblocks > 0; freeptr = *((caddr_t *) freeptr)) {
			if (unlikely(freeptr < freepage || freeptr >= nextpage)) {
				if (unlikely(xpage)) {
					*tailptr = freeptr;
					xpage = 0;
				}
				tailptr = (caddr_t *)freeptr;
			} else {
				--nblocks;
				xpage = 1;
			}
		}
		*tailptr = freeptr;
		goto free_pages;
	}

	heap->used -= bsize;

	xnlock_put_irqrestore(&heap->lock, s);
}
EXPORT_SYMBOL_GPL(xnheap_free);

int xnheap_check_block(struct xnheap *heap, void *block)
{
	int ptype, ret = -EINVAL;
	u32 pagenum, boffset;
	spl_t s;

	xnlock_get_irqsave(&heap->lock, s);

	if ((caddr_t)block < heap->membase || (caddr_t)block >= heap->memlim)
		goto out;

	/* Compute the heading page number in the page map. */
	pagenum = ((caddr_t)block - heap->membase) / XNHEAP_PAGESZ;
	boffset = ((caddr_t)block - (heap->membase + pagenum * XNHEAP_PAGESZ));
	ptype = heap->pagemap[pagenum].type;

	/* Raise error if page unallocated or not heading a range. */
	if (ptype != XNHEAP_PFREE && ptype != XNHEAP_PCONT)
		ret = 0;
out:
	xnlock_put_irqrestore(&heap->lock, s);

	return ret;
}
EXPORT_SYMBOL_GPL(xnheap_check_block);

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
