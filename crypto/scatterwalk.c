// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Cryptographic API.
 *
 * Cipher operations.
 *
 * Copyright (c) 2002 James Morris <jmorris@intercode.com.au>
 *               2002 Adam J. Richter <adam@yggdrasil.com>
 *               2004 Jean-Luc Cooke <jlcooke@certainkey.com>
 */

#include <crypto/scatterwalk.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/scatterlist.h>

static inline void memcpy_dir(void *buf, void *sgdata, size_t nbytes, int out)
{
	void *src = out ? buf : sgdata;
	void *dst = out ? sgdata : buf;

	memcpy(dst, src, nbytes);
}

void scatterwalk_copychunks(void *buf, struct scatter_walk *walk,
			    size_t nbytes, int out)
{
	for (;;) {
		unsigned int len_this_page = scatterwalk_pagelen(walk);
		u8 *vaddr;

		if (len_this_page > nbytes)
			len_this_page = nbytes;

		if (out != 2) {
			vaddr = scatterwalk_map(walk);
			memcpy_dir(buf, vaddr, len_this_page, out);
			scatterwalk_unmap(vaddr);
		}

		scatterwalk_advance(walk, len_this_page);

		if (nbytes == len_this_page)
			break;

		buf += len_this_page;
		nbytes -= len_this_page;

		scatterwalk_pagedone(walk, out & 1, 1);
	}
}
EXPORT_SYMBOL_GPL(scatterwalk_copychunks);

void scatterwalk_map_and_copy(void *buf, struct scatterlist *sg,
			      unsigned int start, unsigned int nbytes, int out)
{
	struct scatter_walk walk;
	struct scatterlist tmp[2];

	if (!nbytes)
		return;

	sg = scatterwalk_ffwd(tmp, sg, start);

	scatterwalk_start(&walk, sg);
	scatterwalk_copychunks(buf, &walk, nbytes, out);
	scatterwalk_done(&walk, out, 0);
}
EXPORT_SYMBOL_GPL(scatterwalk_map_and_copy);

/**
 * memcpy_sglist() - Copy data from one scatterlist to another
 * @dst: The destination scatterlist.  Can be NULL if @nbytes == 0.
 * @src: The source scatterlist.  Can be NULL if @nbytes == 0.
 * @nbytes: Number of bytes to copy
 *
 * The scatterlists can describe exactly the same memory, in which case this
 * function is a no-op.  No other overlaps are supported.
 *
 * Context: Any context
 */
void memcpy_sglist(struct scatterlist *dst, struct scatterlist *src,
		   unsigned int nbytes)
{
	unsigned int src_offset, dst_offset;

	if (unlikely(nbytes == 0)) /* in case src and/or dst is NULL */
		return;

	src_offset = src->offset;
	dst_offset = dst->offset;
	for (;;) {
		/* Compute the length to copy this step. */
		unsigned int len = min3(src->offset + src->length - src_offset,
					dst->offset + dst->length - dst_offset,
					nbytes);
		struct page *src_page = sg_page(src);
		struct page *dst_page = sg_page(dst);
		const void *src_virt;
		void *dst_virt;

		if (IS_ENABLED(CONFIG_HIGHMEM)) {
			/* HIGHMEM: we may have to actually map the pages. */
			const unsigned int src_oip = offset_in_page(src_offset);
			const unsigned int dst_oip = offset_in_page(dst_offset);
			const unsigned int limit = PAGE_SIZE;

			/* Further limit len to not cross a page boundary. */
			len = min3(len, limit - src_oip, limit - dst_oip);

			/* Compute the source and destination pages. */
			src_page += src_offset / PAGE_SIZE;
			dst_page += dst_offset / PAGE_SIZE;

			if (src_page != dst_page) {
				/* Copy between different pages. */
				memcpy_page(dst_page, dst_oip,
					    src_page, src_oip, len);
				flush_dcache_page(dst_page);
			} else if (src_oip != dst_oip) {
				/* Copy between different parts of same page. */
				dst_virt = kmap_local_page(dst_page);
				memcpy(dst_virt + dst_oip, dst_virt + src_oip,
				       len);
				kunmap_local(dst_virt);
				flush_dcache_page(dst_page);
			} /* Else, it's the same memory.  No action needed. */
		} else {
			/*
			 * !HIGHMEM: no mapping needed.  Just work in the linear
			 * buffer of each sg entry.  Note that we can cross page
			 * boundaries, as they are not significant in this case.
			 */
			src_virt = page_address(src_page) + src_offset;
			dst_virt = page_address(dst_page) + dst_offset;
			if (src_virt != dst_virt) {
				memcpy(dst_virt, src_virt, len);
				if (ARCH_IMPLEMENTS_FLUSH_DCACHE_PAGE)
					__scatterwalk_flush_dcache_pages(
						dst_page, dst_offset, len);
			} /* Else, it's the same memory.  No action needed. */
		}
		nbytes -= len;
		if (nbytes == 0) /* No more to copy? */
			break;

		/*
		 * There's more to copy.  Advance the offsets by the length
		 * copied this step, and advance the sg entries as needed.
		 */
		src_offset += len;
		if (src_offset >= src->offset + src->length) {
			src = sg_next(src);
			src_offset = src->offset;
		}
		dst_offset += len;
		if (dst_offset >= dst->offset + dst->length) {
			dst = sg_next(dst);
			dst_offset = dst->offset;
		}
	}
}
EXPORT_SYMBOL_GPL(memcpy_sglist);

struct scatterlist *scatterwalk_ffwd(struct scatterlist dst[2],
				     struct scatterlist *src,
				     unsigned int len)
{
	for (;;) {
		if (!len)
			return src;

		if (src->length > len)
			break;

		len -= src->length;
		src = sg_next(src);
	}

	sg_init_table(dst, 2);
	sg_set_page(dst, sg_page(src), src->length - len, src->offset + len);
	scatterwalk_crypto_chain(dst, sg_next(src), 2);

	return dst;
}
EXPORT_SYMBOL_GPL(scatterwalk_ffwd);
