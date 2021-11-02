// SPDX-License-Identifier: GPL-2.0
/*
 * IMG DEC Memory Manager for unified memory
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

#include <linux/dma-mapping.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>

#include "img_mem_man.h"

void img_mmu_get_pages(void **page_args, void *sgt_args)
{
	struct page **pages = (struct page **)page_args;
	struct sg_table *sgt = sgt_args;
	struct scatterlist *sgl = sgt->sgl;
	int i;

	i = 0;
	while (sgl) {
		pages[i++] = sg_page(sgl);
		sgl = sg_next(sgl);
	}
}

unsigned int img_mmu_get_orig_nents(void *sgt_args)
{
	struct sg_table *sgt = sgt_args;

	return sgt->orig_nents;
}

void img_mmu_set_sgt_nents(void *sgt_args, int ret)
{
	struct sg_table *sgt = sgt_args;

	sgt->nents = ret;
}

void img_mmu_set_sg_table(void **sg_table_args, void *buffer)
{
	struct sg_table **sg_table = (struct sg_table **)sg_table_args;

	*sg_table = buffer;
}

unsigned int img_mmu_get_sgl_length(void *sgl_args)
{
	struct scatterlist *sgl = (struct scatterlist *)sgl_args;

	return sgl->length;
}

void *img_mmu_get_sgl(void *sgt_args)
{
	struct sg_table *sgt = sgt_args;

	return sgt->sgl;
}

static int unified_alloc(void *device, struct heap *heap,
			 unsigned long size, enum mem_attr attr,
			 struct buffer *buffer)
{
	struct sg_table *sgt;
	void *sgl;
	int pages;
	int ret;

	dev_dbg(device, "%s:%d buffer %d (0x%p)\n", __func__, __LINE__,
		buffer->id, buffer);

	sgt = kmalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt)
		return -ENOMEM;

	pages = (size + PAGE_SIZE - 1) / PAGE_SIZE;

	ret = sg_alloc_table(sgt, pages, GFP_KERNEL);
	if (ret)
		goto sg_alloc_table_failed;

	sgl = img_mmu_get_sgl(sgt);
	while (sgl) {
		void *page;
		unsigned long long dma_addr;

		page = alloc_page(heap->options.unified.gfp_type);
		if (!page) {
			dev_err(device, "%s alloc_page failed!\n", __func__);
			ret = -ENOMEM;
			goto alloc_page_failed;
		}

		/*
		 * dma_map_page() is probably going to fail if alloc flags are
		 * GFP_HIGHMEM, since it is not mapped to CPU. Hopefully, this
		 * will never happen because memory of this sort cannot be used
		 * for DMA anyway. To check if this is the case, build with
		 * debug, set trace_physical_pages=1 and check if page_address
		 * printed above is NULL
		 */
		dma_addr = dma_map_page(device, page, 0, PAGE_SIZE, DMA_BIDIRECTIONAL);
		if (dma_mapping_error(device, dma_addr)) {
			__free_page(page);
			dev_err(device, "%s dma_map_page failed!\n", __func__);
			ret = -EIO;
			goto alloc_page_failed;
		}
		dma_unmap_page(device, dma_addr, PAGE_SIZE, DMA_BIDIRECTIONAL);

		sg_set_page(sgl, page, PAGE_SIZE, 0);

		sgl = sg_next(sgl);
	}

	buffer->priv = sgt;
	return 0;

alloc_page_failed:
	sgl = img_mmu_get_sgl(sgt);
	while (sgl) {
		void *page = sg_page(sgl);

		if (page)
			__free_page(page);

		sgl = sg_next(sgl);
	}
	sg_free_table(sgt);
sg_alloc_table_failed:
	kfree(sgt);
	return ret;
}

static void unified_free(struct heap *heap, struct buffer *buffer)
{
	void *dev = buffer->device;
	void *sgt = buffer->priv;
	void *sgl;

	dev_dbg(dev, "%s:%d buffer %d (0x%p)\n", __func__, __LINE__,
		buffer->id, buffer);

	if (buffer->kptr) {
		dev_dbg(dev, "%s vunmap 0x%p\n", __func__, buffer->kptr);
		dma_unmap_sg(dev, img_mmu_get_sgl(sgt), img_mmu_get_orig_nents(sgt),
			     DMA_FROM_DEVICE);
		vunmap(buffer->kptr);
	}

	sgl = img_mmu_get_sgl(sgt);
	while (sgl) {
		__free_page(sg_page(sgl));
		sgl = sg_next(sgl);
	}
	sg_free_table(sgt);
	kfree(sgt);
}

static int unified_map_km(struct heap *heap, struct buffer *buffer)
{
	void *dev = buffer->device;
	void *sgt = buffer->priv;
	void *sgl = img_mmu_get_sgl(sgt);
	unsigned int num_pages = sg_nents(sgl);
	unsigned int orig_nents = img_mmu_get_orig_nents(sgt);
	void **pages;
	int ret;
	pgprot_t prot;

	dev_dbg(dev, "%s:%d buffer %d (0x%p)\n", __func__, __LINE__, buffer->id, buffer);

	if (buffer->kptr) {
		dev_warn(dev, "%s called for already mapped buffer %d\n", __func__, buffer->id);
		return 0;
	}

	pages = kmalloc_array(num_pages, sizeof(void *), GFP_KERNEL);
	if (!pages)
		return -ENOMEM;

	img_mmu_get_pages(pages, sgt);

	prot = PAGE_KERNEL;
	prot = pgprot_writecombine(prot);
	buffer->kptr = vmap((struct page **)pages, num_pages, VM_MAP, prot);
	kfree(pages);
	if (!buffer->kptr) {
		dev_err(dev, "%s vmap failed!\n", __func__);
		return -EFAULT;
	}

	ret = dma_map_sg(dev, sgl, orig_nents, DMA_FROM_DEVICE);

	if (ret <= 0) {
		dev_err(dev, "%s dma_map_sg failed!\n", __func__);
		vunmap(buffer->kptr);
		return -EFAULT;
	}
	dev_dbg(dev, "%s:%d buffer %d orig_nents %d nents %d\n", __func__,
		__LINE__, buffer->id, orig_nents, ret);

	img_mmu_set_sgt_nents(sgt, ret);

	dev_dbg(dev, "%s:%d buffer %d vmap to 0x%p\n", __func__, __LINE__,
		buffer->id, buffer->kptr);

	return 0;
}

static int unified_get_sg_table(struct heap *heap, struct buffer *buffer, void **sg_table)
{
	img_mmu_set_sg_table(sg_table, buffer->priv);
	return 0;
}

static void unified_sync_cpu_to_dev(struct heap *heap, struct buffer *buffer)
{
	void *dev = buffer->device;
	void *sgt = buffer->priv;

	if (!buffer->kptr)
		return;

	dev_dbg(dev, "%s:%d buffer %d (0x%p)\n", __func__, __LINE__, buffer->id, buffer);

	dma_sync_sg_for_device(dev, img_mmu_get_sgl(sgt), img_mmu_get_orig_nents(sgt),
			       DMA_TO_DEVICE);
}

static void unified_sync_dev_to_cpu(struct heap *heap, struct buffer *buffer)
{
	void *dev = buffer->device;
	void *sgt = buffer->priv;

	if (!buffer->kptr)
		return;

	dev_dbg(dev, "%s:%d buffer %d (0x%p)\n", __func__, __LINE__,
		buffer->id, buffer);

	dma_sync_sg_for_cpu(dev, img_mmu_get_sgl(sgt), img_mmu_get_orig_nents(sgt),
			    DMA_FROM_DEVICE);
}

static void unified_heap_destroy(struct heap *heap)
{
}

static struct heap_ops unified_heap_ops = {
	.alloc = unified_alloc,
	.free = unified_free,
	.map_km = unified_map_km,
	.get_sg_table = unified_get_sg_table,
	.sync_cpu_to_dev = unified_sync_cpu_to_dev,
	.sync_dev_to_cpu = unified_sync_dev_to_cpu,
	.destroy = unified_heap_destroy,
};

int img_mem_unified_init(const struct heap_config *heap_cfg,
			 struct heap *heap)
{
	heap->ops = &unified_heap_ops;
	return 0;
}
