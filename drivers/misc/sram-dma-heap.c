// SPDX-License-Identifier: GPL-2.0
/*
 * SRAM DMA-Heap userspace exporter
 *
 * Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/
 *	Andrew F. Davis <afd@ti.com>
 */

#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/genalloc.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>

#include "sram.h"

struct sram_dma_heap {
	struct dma_heap *heap;
	struct gen_pool *pool;
};

struct sram_dma_heap_buffer {
	struct gen_pool *pool;
	struct list_head attachments;
	struct mutex attachments_lock;
	unsigned long len;
	void *vaddr;
	phys_addr_t paddr;
};

struct dma_heap_attachment {
	struct device *dev;
	struct sg_table *table;
	struct list_head list;
};

static int dma_heap_attach(struct dma_buf *dmabuf,
			   struct dma_buf_attachment *attachment)
{
	struct sram_dma_heap_buffer *buffer = dmabuf->priv;
	struct dma_heap_attachment *a;
	struct sg_table *table;

	a = kzalloc(sizeof(*a), GFP_KERNEL);
	if (!a)
		return -ENOMEM;

	table = kmalloc(sizeof(*table), GFP_KERNEL);
	if (!table) {
		kfree(a);
		return -ENOMEM;
	}
	if (sg_alloc_table(table, 1, GFP_KERNEL)) {
		kfree(table);
		kfree(a);
		return -ENOMEM;
	}
	sg_set_page(table->sgl, pfn_to_page(PFN_DOWN(buffer->paddr)), buffer->len, 0);

	a->table = table;
	a->dev = attachment->dev;
	INIT_LIST_HEAD(&a->list);

	attachment->priv = a;

	mutex_lock(&buffer->attachments_lock);
	list_add(&a->list, &buffer->attachments);
	mutex_unlock(&buffer->attachments_lock);

	return 0;
}

static void dma_heap_detatch(struct dma_buf *dmabuf,
			     struct dma_buf_attachment *attachment)
{
	struct sram_dma_heap_buffer *buffer = dmabuf->priv;
	struct dma_heap_attachment *a = attachment->priv;

	mutex_lock(&buffer->attachments_lock);
	list_del(&a->list);
	mutex_unlock(&buffer->attachments_lock);

	sg_free_table(a->table);
	kfree(a->table);
	kfree(a);
}

static struct sg_table *dma_heap_map_dma_buf(struct dma_buf_attachment *attachment,
					     enum dma_data_direction direction)
{
	struct dma_heap_attachment *a = attachment->priv;
	struct sg_table *table = a->table;

	if (!dma_map_sg_attrs(attachment->dev, table->sgl, table->nents,
			      direction, DMA_ATTR_SKIP_CPU_SYNC))
		return ERR_PTR(-ENOMEM);

	return table;
}

static void dma_heap_unmap_dma_buf(struct dma_buf_attachment *attachment,
				   struct sg_table *table,
				   enum dma_data_direction direction)
{
	dma_unmap_sg_attrs(attachment->dev, table->sgl, table->nents,
			   direction, DMA_ATTR_SKIP_CPU_SYNC);
}

static void dma_heap_dma_buf_release(struct dma_buf *dmabuf)
{
	struct sram_dma_heap_buffer *buffer = dmabuf->priv;

	gen_pool_free(buffer->pool, (unsigned long)buffer->vaddr, buffer->len);
	kfree(buffer);
}

static int dma_heap_mmap(struct dma_buf *dmabuf, struct vm_area_struct *vma)
{
	struct sram_dma_heap_buffer *buffer = dmabuf->priv;
	int ret;

	/* SRAM mappings are not cached */
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	ret = vm_iomap_memory(vma, buffer->paddr, buffer->len);
	if (ret)
		pr_err("Could not map buffer to userspace\n");

	return ret;
}

static void *dma_heap_vmap(struct dma_buf *dmabuf)
{
	struct sram_dma_heap_buffer *buffer = dmabuf->priv;

	return buffer->vaddr;
}

static const struct dma_buf_ops sram_dma_heap_buf_ops = {
	.attach = dma_heap_attach,
	.detach = dma_heap_detatch,
	.map_dma_buf = dma_heap_map_dma_buf,
	.unmap_dma_buf = dma_heap_unmap_dma_buf,
	.release = dma_heap_dma_buf_release,
	.mmap = dma_heap_mmap,
	.vmap = dma_heap_vmap,
};

static int sram_dma_heap_allocate(struct dma_heap *heap,
				  unsigned long len,
				  unsigned long fd_flags,
				  unsigned long heap_flags)
{
	struct sram_dma_heap *sram_dma_heap = dma_heap_get_drvdata(heap);
	struct sram_dma_heap_buffer *buffer;

	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
	struct dma_buf *dmabuf;
	int ret;

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;
	buffer->pool = sram_dma_heap->pool;
	INIT_LIST_HEAD(&buffer->attachments);
	mutex_init(&buffer->attachments_lock);
	buffer->len = len;

	buffer->vaddr = (void *)gen_pool_alloc(buffer->pool, buffer->len);
	if (!buffer->vaddr) {
		ret = -ENOMEM;
		goto free_buffer;
	}

	buffer->paddr = gen_pool_virt_to_phys(buffer->pool, (unsigned long)buffer->vaddr);
	if (buffer->paddr == -1) {
		ret = -ENOMEM;
		goto free_pool;
	}

	/* create the dmabuf */
	exp_info.ops = &sram_dma_heap_buf_ops;
	exp_info.size = buffer->len;
	exp_info.flags = fd_flags;
	exp_info.priv = buffer;
	dmabuf = dma_buf_export(&exp_info);
	if (IS_ERR(dmabuf)) {
		ret = PTR_ERR(dmabuf);
		goto free_pool;
	}

	ret = dma_buf_fd(dmabuf, fd_flags);
	if (ret < 0) {
		dma_buf_put(dmabuf);
		/* just return, as put will call release and that will free */
		return ret;
	}

	return ret;

free_pool:
	gen_pool_free(buffer->pool, (unsigned long)buffer->vaddr, buffer->len);
free_buffer:
	kfree(buffer);

	return ret;
}

static struct dma_heap_ops sram_dma_heap_ops = {
	.allocate = sram_dma_heap_allocate,
};

int sram_dma_heap_export(struct sram_dev *sram,
			 struct sram_reserve *block,
			 phys_addr_t start,
			 struct sram_partition *part)
{
	struct sram_dma_heap *sram_dma_heap;
	struct dma_heap_export_info exp_info;

	dev_info(sram->dev, "Exporting SRAM pool '%s'\n", block->label);

	sram_dma_heap = kzalloc(sizeof(*sram_dma_heap), GFP_KERNEL);
	if (!sram_dma_heap)
		return -ENOMEM;
	sram_dma_heap->pool = part->pool;

	exp_info.name = block->label;
	exp_info.ops = &sram_dma_heap_ops;
	exp_info.priv = sram_dma_heap;
	sram_dma_heap->heap = dma_heap_add(&exp_info);
	if (IS_ERR(sram_dma_heap->heap)) {
		int ret = PTR_ERR(sram_dma_heap->heap);
		kfree(sram_dma_heap);
		return ret;
	}

	return 0;
}
