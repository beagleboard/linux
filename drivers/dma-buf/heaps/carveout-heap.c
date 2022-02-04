// SPDX-License-Identifier: GPL-2.0
/*
 * Carveout DMA-Heap userspace exporter
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

struct carveout_dma_heap {
	struct dma_heap *heap;
	struct gen_pool *pool;
	bool cached;
};

struct carveout_dma_heap_buffer {
	struct gen_pool *pool;
	struct list_head attachments;
	struct mutex attachments_lock;
	struct mutex vmap_lock;
	int vmap_cnt;
	unsigned long len;
	void *vaddr;
	phys_addr_t paddr;
	bool cached;
};

struct dma_heap_attachment {
	struct device *dev;
	struct sg_table *table;
	struct list_head list;
};

static int dma_heap_attach(struct dma_buf *dmabuf,
			   struct dma_buf_attachment *attachment)
{
	struct carveout_dma_heap_buffer *buffer = dmabuf->priv;
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
	struct carveout_dma_heap_buffer *buffer = dmabuf->priv;
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
	struct carveout_dma_heap_buffer *buffer = dmabuf->priv;

	if (buffer->vmap_cnt > 0) {
		WARN(1, "%s: buffer still mapped in the kernel\n", __func__);
		memunmap(buffer->vaddr);
	}

	gen_pool_free(buffer->pool, buffer->paddr, buffer->len);
	kfree(buffer);
}

static int dma_heap_mmap(struct dma_buf *dmabuf, struct vm_area_struct *vma)
{
	struct carveout_dma_heap_buffer *buffer = dmabuf->priv;
	int ret;

	if (!buffer->cached)
		vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	ret = vm_iomap_memory(vma, buffer->paddr, buffer->len);
	if (ret)
		pr_err("Could not map buffer to userspace\n");

	return ret;
}

static void *dma_heap_vmap(struct dma_buf *dmabuf)
{
	struct carveout_dma_heap_buffer *buffer = dmabuf->priv;
	void *vaddr;

	mutex_lock(&buffer->vmap_lock);

	if (buffer->vmap_cnt) {
		buffer->vmap_cnt++;
		vaddr = buffer->vaddr;
		goto exit;
	}
	if (buffer->cached)
		vaddr = memremap(buffer->paddr, buffer->len, MEMREMAP_WB);
	else
		vaddr = memremap(buffer->paddr, buffer->len, MEMREMAP_WC);
	if (!vaddr) {
		pr_err("Could not memremap buffer\n");
		goto exit;
	}
	if (IS_ERR(vaddr))
		goto exit;
	buffer->vaddr = vaddr;
	buffer->vmap_cnt++;

exit:
	mutex_unlock(&buffer->vmap_lock);
	return vaddr;
}

static void dma_heap_vunmap(struct dma_buf *dmabuf, void *vaddr)
{
	struct carveout_dma_heap_buffer *buffer = dmabuf->priv;

	mutex_lock(&buffer->vmap_lock);
	if (!--buffer->vmap_cnt) {
		memunmap(buffer->vaddr);
		buffer->vaddr = NULL;
	}
	mutex_unlock(&buffer->vmap_lock);
}

const struct dma_buf_ops carveout_dma_heap_buf_ops = {
	.attach = dma_heap_attach,
	.detach = dma_heap_detatch,
	.map_dma_buf = dma_heap_map_dma_buf,
	.unmap_dma_buf = dma_heap_unmap_dma_buf,
	.release = dma_heap_dma_buf_release,
	.mmap = dma_heap_mmap,
	.vmap = dma_heap_vmap,
	.vunmap = dma_heap_vunmap,
};

static int carveout_dma_heap_allocate(struct dma_heap *heap,
				      unsigned long len,
				      unsigned long fd_flags,
				      unsigned long heap_flags)
{
	struct carveout_dma_heap *carveout_dma_heap = dma_heap_get_drvdata(heap);
	struct carveout_dma_heap_buffer *buffer;

	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
	struct dma_buf *dmabuf;
	int ret;

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;
	buffer->pool = carveout_dma_heap->pool;
	buffer->cached = carveout_dma_heap->cached;
	INIT_LIST_HEAD(&buffer->attachments);
	mutex_init(&buffer->attachments_lock);
	mutex_init(&buffer->vmap_lock);
	buffer->len = len;

	buffer->paddr = gen_pool_alloc(buffer->pool, buffer->len);
	if (!buffer->paddr) {
		ret = -ENOMEM;
		goto free_buffer;
	}

	/* create the dmabuf */
	exp_info.ops = &carveout_dma_heap_buf_ops;
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
	gen_pool_free(buffer->pool, buffer->paddr, buffer->len);
free_buffer:
	kfree(buffer);

	return ret;
}

static struct dma_heap_ops carveout_dma_heap_ops = {
	.allocate = carveout_dma_heap_allocate,
};

int carveout_dma_heap_export(phys_addr_t base, size_t size, const char *name, bool cached)
{
	struct carveout_dma_heap *carveout_dma_heap;
	struct dma_heap_export_info exp_info;
	int ret;

	carveout_dma_heap = kzalloc(sizeof(*carveout_dma_heap), GFP_KERNEL);
	if (!carveout_dma_heap)
		return -ENOMEM;

	carveout_dma_heap->pool = gen_pool_create(PAGE_SHIFT, NUMA_NO_NODE);
	if (IS_ERR(carveout_dma_heap->pool)) {
		pr_err("Carveout Heap: Could not create memory pool\n");
		ret = PTR_ERR(carveout_dma_heap->pool);
		goto free_carveout_dma_heap;
	}
	ret = gen_pool_add(carveout_dma_heap->pool, base, size, NUMA_NO_NODE);
	if (ret) {
		pr_err("Carveout Heap: Could not add memory to pool\n");
		goto free_pool;
	}

	carveout_dma_heap->cached = cached;

	exp_info.name = name;
	exp_info.ops = &carveout_dma_heap_ops;
	exp_info.priv = carveout_dma_heap;
	carveout_dma_heap->heap = dma_heap_add(&exp_info);
	if (IS_ERR(carveout_dma_heap->heap)) {
		pr_err("Carveout Heap: Could not add DMA-Heap\n");
		ret = PTR_ERR(carveout_dma_heap->heap);
		goto free_pool;
	}

	pr_info("Carveout Heap: Exported %zu MiB at %pa\n", size / SZ_1M, &base);

	return 0;

free_pool:
	gen_pool_destroy(carveout_dma_heap->pool);
free_carveout_dma_heap:
	kfree(carveout_dma_heap);
	return ret;
}

#ifdef CONFIG_OF_RESERVED_MEM
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_reserved_mem.h>

#define MAX_HEAP_AREAS 7
struct reserved_mem heap_areas[MAX_HEAP_AREAS];
size_t heap_area_count;

static int __init carveout_dma_heap_init_areas(void)
{
	int i;

	for (i = 0; i < heap_area_count; i++) {
		struct reserved_mem *rmem = &heap_areas[i];
		bool cached = !of_get_flat_dt_prop(rmem->fdt_node, "no-map", NULL);
		int ret = carveout_dma_heap_export(rmem->base, rmem->size, rmem->name, cached);
		if (ret) {
			pr_err("Carveout Heap: could not export as DMA-Heap\n");
			return ret;
		}
	}

	return 0;
}
fs_initcall(carveout_dma_heap_init_areas);

static int __init rmem_dma_heap_carveout_setup(struct reserved_mem *rmem)
{
	phys_addr_t align = PAGE_SIZE;
	phys_addr_t mask = align - 1;

	if ((rmem->base & mask) || (rmem->size & mask)) {
		pr_err("Carveout Heap: incorrect alignment of region\n");
		return -EINVAL;
	}

	/* Sanity check */
	if (heap_area_count == ARRAY_SIZE(heap_areas)) {
		pr_err("Not enough slots for DMA-Heap reserved regions!\n");
		return -ENOSPC;
	}

	/*
	 * Each reserved area must be initialized later, when more kernel
	 * subsystems (like slab allocator) are available.
	 */
	heap_areas[heap_area_count] = *rmem;
	heap_area_count++;

	return 0;
}
RESERVEDMEM_OF_DECLARE(dma_heap_carveout, "dma-heap-carveout", rmem_dma_heap_carveout_setup);

#endif
