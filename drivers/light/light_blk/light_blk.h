/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _LIGHT_BLK_H
#define _LIGHT_BLK_H

#include <linux/virtio_types.h>
#include <linux/virtio_blk.h>

static const unsigned int offsets[] = {
	offsetof(struct virtio_blk_config, capacity),
	offsetof(struct virtio_blk_config, size_max),
	offsetof(struct virtio_blk_config, seg_max),
	offsetof(struct virtio_blk_config, geometry),
	offsetof(struct virtio_blk_config, blk_size),
	offsetof(struct virtio_blk_config, physical_block_exp),
	offsetof(struct virtio_blk_config, alignment_offset),
	offsetof(struct virtio_blk_config, min_io_size),
	offsetof(struct virtio_blk_config, opt_io_size),
	offsetof(struct virtio_blk_config, wce),
	offsetof(struct virtio_blk_config, unused),
	offsetof(struct virtio_blk_config, num_queues),
	offsetof(struct virtio_blk_config, max_discard_sectors),
	offsetof(struct virtio_blk_config, max_discard_seg),
	offsetof(struct virtio_blk_config, discard_sector_alignment),
	offsetof(struct virtio_blk_config, max_write_zeroes_sectors),
	offsetof(struct virtio_blk_config, max_write_zeroes_seg),
	offsetof(struct virtio_blk_config, write_zeroes_may_unmap),
};

enum offsets_index {
	CAPACITY,
	MAX_SEG_SIZE,
	SEG_MAX,
	GEO,
	BLK_SIZE,
	EXP,
	ALIGN_OFF,
	MIN_IO_SIZE,
	OPT_IO_SIZE,
	WCE,
	UNNSED,
	NUM_QUEUES,
	MAX_DISCARD_SECS,
	MAX_DISCARD_SEG,
	DISCARD_SECTOR_ALIGN,
	MAX_WRITE_ZEROS_SECS,
	MAX_WRITE_ZEROS_SEG,
	WRITE_ZEROS_MAY_UMAP,
};

#endif
