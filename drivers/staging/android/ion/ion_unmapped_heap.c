/*
 * drivers/staging/android/ion/ion_unmapped_heap.c
 *
 * Copyright (C) 2016-2017 Linaro, Inc.
 * Copyright (C) Allwinner 2014
 * Author: <sunny@allwinnertech.com> for Allwinner.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/*
 * ION heap type for handling physical memory heap not mapped
 * in the linux-based OS.
 *
 * "unmapped heap" buffers are default not mapped but buffer owner
 * can explicitly request mapping for some specific purpose.
 *
 * Based on Allwinner work (allocation thru gen_pool) and
 * HiSilicon work (create ION heaps from DT nodes,
 * Author: Chen Feng <puck.chen@hisilicon.com>).
 */

#include <linux/err.h>
#include <linux/genalloc.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>

#include "ion.h"

/*
 * TODO: non-contigous unammped heaps:
 * - add a flag to specify contiguity constraint?
 * - define antoher heap type that allocate to the smae pool(s)?
 */

struct ion_unmapped_heap {
	struct ion_heap heap;
	struct gen_pool *pool;
	phys_addr_t base;
	size_t size;
};

struct unmapped_buffer_priv {
	phys_addr_t base;
};

static phys_addr_t get_buffer_base(struct unmapped_buffer_priv *priv)
{
	return priv->base;
}

static struct device *heap2dev(struct ion_heap *heap)
{
	return heap->dev->dev.this_device;
}

static phys_addr_t ion_unmapped_allocate(struct ion_heap *heap,
					   unsigned long size,
					   phys_addr_t *addr)
{
	struct ion_unmapped_heap *umh =
		container_of(heap, struct ion_unmapped_heap, heap);
	unsigned long offset = gen_pool_alloc(umh->pool, size);

	if (!offset) {
		dev_err(heap2dev(heap),
			"%s(%d) err: alloc 0x%08x bytes failed\n",
			__func__, __LINE__, (u32)size);
		return false;
	}

	*addr = offset;
	return true;
}

static void ion_unmapped_free(struct ion_heap *heap, phys_addr_t addr,
			    unsigned long size)
{
	struct ion_unmapped_heap *umh =
		container_of(heap, struct ion_unmapped_heap, heap);

	gen_pool_free(umh->pool, addr, size);
}

static struct sg_table *ion_unmapped_heap_map_dma(struct ion_heap *heap,
						struct ion_buffer *buffer)
{
	struct sg_table *table;
	int ret;

	table = kzalloc(sizeof(struct sg_table), GFP_KERNEL);
	if (!table)
		return ERR_PTR(-ENOMEM);
	ret = sg_alloc_table(table, 1, GFP_KERNEL);
	if (ret) {
		kfree(table);
		return ERR_PTR(ret);
	}
	sg_set_page(table->sgl,
		    phys_to_page(get_buffer_base(buffer->priv_virt)),
		    buffer->size, 0);

	return table;
}

void ion_unmapped_heap_unmap_dma(struct ion_heap *heap,
				struct ion_buffer *buffer)
{
	sg_free_table(buffer->sg_table);
	kfree(buffer->sg_table);
}

static int ion_unmapped_heap_allocate(struct ion_heap *heap,
				    struct ion_buffer *buffer,
				    unsigned long size,
				    unsigned long flags)
{
	struct unmapped_buffer_priv *priv;
	phys_addr_t base;
	int rc = -EINVAL;

	if (!ion_unmapped_allocate(heap, size, &base))
		return -ENOMEM;

	priv = devm_kzalloc(heap2dev(heap), sizeof(*priv), GFP_KERNEL);
	if (IS_ERR_OR_NULL(priv)) {
		rc = -ENOMEM;
		goto err;
	}

	priv->base = base;
	buffer->size = roundup(size, PAGE_SIZE);
	buffer->priv_virt = priv;

	buffer->sg_table = ion_unmapped_heap_map_dma(heap, buffer);
	if (!buffer->sg_table) {
		rc = -ENOMEM;
		goto err;
	}
	sg_dma_address(buffer->sg_table->sgl) = priv->base;
	sg_dma_len(buffer->sg_table->sgl) = size;
	return 0;
err:
	ion_unmapped_free(heap, base, size);
	devm_kfree(heap2dev(heap), priv);
	buffer->priv_virt = NULL;
	return rc;
}

static void ion_unmapped_heap_free(struct ion_buffer *buffer)
{
	struct ion_heap *heap = buffer->heap;

	ion_unmapped_heap_unmap_dma(heap, buffer);
	ion_unmapped_free(heap, get_buffer_base(buffer->priv_virt),
			 buffer->size);
	devm_kfree(heap2dev(heap), buffer->priv_virt);
	buffer->priv_virt = NULL;
}

static int ion_unmapped_heap_map_user(struct ion_heap *heap,
				    struct ion_buffer *buffer,
				    struct vm_area_struct *vma)
{
	phys_addr_t pa = get_buffer_base(buffer->priv_virt);

	/*
	 * when user call ION_IOC_ALLOC not with ION_FLAG_CACHED, ion_mmap will
	 * change vma->vm_page_prot to pgprot_writecombine itself, so we do not
	 * need change to pgprot_writecombine here manually.
	 */
	return remap_pfn_range(vma, vma->vm_start,
				__phys_to_pfn(pa) + vma->vm_pgoff,
				vma->vm_end - vma->vm_start,
				vma->vm_page_prot);
}

static struct ion_heap_ops unmapped_heap_ops = {
	.allocate = ion_unmapped_heap_allocate,
	.free = ion_unmapped_heap_free,
	.map_user = ion_unmapped_heap_map_user,
	.map_kernel = ion_heap_map_kernel,
	.unmap_kernel = ion_heap_unmap_kernel,
};

struct ion_heap *ion_unmapped_heap_create(phys_addr_t base, size_t size)
{
	struct ion_unmapped_heap *umh;

	umh = kzalloc(sizeof(struct ion_unmapped_heap), GFP_KERNEL);
	if (!umh)
		return ERR_PTR(-ENOMEM);

	umh->pool = gen_pool_create(PAGE_SHIFT, -1);
	if (!umh->pool) {
		kfree(umh);
		return ERR_PTR(-ENOMEM);
	}
	umh->base = base;
	umh->size = size;

	gen_pool_add(umh->pool, umh->base, size, -1);
	umh->heap.ops = &unmapped_heap_ops;
	umh->heap.type = ION_HEAP_TYPE_UNMAPPED;

	return &umh->heap;
}
EXPORT_SYMBOL(ion_unmapped_heap_create);

void ion_unmapped_heap_destroy(struct ion_heap *heap)
{
	struct ion_unmapped_heap *umh =
	     container_of(heap, struct  ion_unmapped_heap, heap);

	gen_pool_destroy(umh->pool);
	kfree(umh);
	umh = NULL;
}
EXPORT_SYMBOL(ion_unmapped_heap_destroy);

#if defined(CONFIG_ION_DUMMY_UNMAPPED_HEAP) && CONFIG_ION_DUMMY_UNMAPPED_SIZE
#define DUMMY_UNAMMPED_HEAP_NAME	"unmapped_contiguous"

static int ion_add_dummy_unmapped_heaps(void)
{
        struct ion_heap *heap;
	const char name[] = DUMMY_UNAMMPED_HEAP_NAME;

	heap = ion_unmapped_heap_create(CONFIG_ION_DUMMY_UNMAPPED_BASE,
					CONFIG_ION_DUMMY_UNMAPPED_SIZE);
	if (IS_ERR(heap))
		return PTR_ERR(heap);

	heap->name = kzalloc(sizeof(name), GFP_KERNEL);
	if (IS_ERR(heap->name)) {
		kfree(heap);
		return PTR_ERR(heap->name);
	}
	memcpy((char *)heap->name, name, sizeof(name));

	ion_device_add_heap(heap);
        return 0;
}
device_initcall(ion_add_dummy_unmapped_heaps);
#endif
