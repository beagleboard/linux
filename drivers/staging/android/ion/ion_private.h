/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ION Memory Allocator - Internal header
 *
 * Copyright (C) 2019 Google, Inc.
 */

#ifndef _ION_PRIVATE_H
#define _ION_PRIVATE_H

#include <linux/dcache.h>
#include <linux/dma-buf.h>
#include <linux/ion.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/plist.h>
#include <linux/rbtree.h>
#include <linux/rwsem.h>
#include <linux/types.h>

/**
 * struct ion_device - the metadata of the ion device node
 * @dev:		the actual misc device
 * @lock:		rwsem protecting the tree of heaps, heap_bitmap and
 *			clients
 * @heap_ids:		bitmap of register heap ids
 */
struct ion_device {
	struct miscdevice dev;
	struct rw_semaphore lock;
	DECLARE_BITMAP(heap_ids, ION_NUM_MAX_HEAPS);
	struct plist_head heaps;
	struct dentry *debug_root;
	int heap_cnt;
};

/* ion_buffer manipulators */
extern struct ion_buffer *ion_buffer_alloc(struct ion_device *dev, size_t len,
					   unsigned int heap_id_mask,
					   unsigned int flags);
extern void ion_buffer_release(struct ion_buffer *buffer);
extern int ion_buffer_destroy(struct ion_device *dev,
			      struct ion_buffer *buffer);
extern void *ion_buffer_kmap_get(struct ion_buffer *buffer);
extern void ion_buffer_kmap_put(struct ion_buffer *buffer);

/* ion dmabuf allocator */
extern struct dma_buf *ion_dmabuf_alloc(struct ion_device *dev, size_t len,
					unsigned int heap_id_mask,
					unsigned int flags);
extern int ion_free(struct ion_buffer *buffer);

/* ion heap helpers */
extern int ion_heap_cleanup(struct ion_heap *heap);

u64 ion_get_total_heap_bytes(void);

#endif /* _ION_PRIVATE_H */
