/* SPDX-License-Identifier: (GPL-2.0 WITH Linux-syscall-note) OR MIT */
#ifndef _UAPI_LINUX_VIRTIO_VDMABUF_H
#define _UAPI_LINUX_VIRTIO_VDMABUF_H

/*
 * user can use this ioctl to alloc dmabuf, and get its
 * fd, buf_id
 */
#define VIRTIO_VDMABUF_IOCTL_ALLOC_FD \
_IOC(_IOC_NONE, 'G', 2, sizeof(struct virtio_vdmabuf_alloc))

/*
 * user can use this ioctl to get dmabuf fd via buf_id,
 * which is passed to us from remote peer
 */
#define VIRTIO_VDMABUF_IOCTL_IMPORT_FD \
_IOC(_IOC_NONE, 'G', 3, sizeof(struct virtio_vdmabuf_import))

/* dmabuf flag */
#define VIRTIO_VDAMBUF_NONCACHED	0x1

/*
 * Max single alloc size is (768 * PAGE_SIZE) only for
 * VIRTIO_VDMABUF_TYPE_SYSTEM heap
 * eg, it is 3M when PAGE_SIZE is 4K
 */
#define VIRTIO_VDAMBUF_MAX_ALLOC_SIZE	(768 * PAGE_SIZE)

/*
 * size          : user fill dmabuf size only when use heap VIRTIO_VDMABUF_TYPE_SYSTEM or
 *                 VIRTIO_VDMABUF_HEAP_TYPE_SYSTEM_CONTIG or VIRTIO_VDMABUF_HEAP_USER
 * align         : user fill dmabuf align, kernel use PAGE_SIZE as the align by default
 * uaddr         : user fill user buf addr only when use VIRTIO_VDMABUF_HEAP_USER heap
 * fd            : kernel fill dmabuf fd
 * buf_id        : kernel fill id of dmabuf
 * heap_type     : user fill heap type
 * carveout_type : user fill carveout type when heap type is VIRTIO_VDMABUF_HEAP_TYPE_CARVEOUT
 * flags         : user fill buf flags, eg, noncache(VIRTIO_VDAMBUF_NONCACHED),
 *                 kernel use cache mode by default
 */
struct virtio_vdmabuf_alloc {
	size_t size;
	int align;
	void *uaddr;
	int fd;
	unsigned int buf_id;
	int heap_type;
	int carveout_type;
	int flags;
};

/*
 * buf_id : user fill the dmabuf id
 * fd     : kernel fill the dmabuf fd
 * size   : kernel fill the dmabuf size
 */
struct virtio_vdmabuf_import {
	unsigned int buf_id;
	int fd;
	size_t size;
};

/* virtio-vdmabuf supported heap type */
enum virtio_vdmabuf_heap_type {
	VIRTIO_VDMABUF_HEAP_TYPE_USER,
	VIRTIO_VDMABUF_HEAP_TYPE_SYSTEM,
	VIRTIO_VDMABUF_HEAP_TYPE_SYSTEM_CONTIG,
	VIRTIO_VDMABUF_HEAP_TYPE_CARVEOUT, /* reserved mem */
	VIRTIO_VDMABUF_HEAP_NUM_HEAPS,
};

/* carveout type */
enum virtio_vdmabuf_carveout_id {
	VIRTIO_VDMABUF_CARVEOUT_TYPE_VI,
	VIRTIO_VDMABUF_CARVEOUT_TYPE_VO,
	VIRTIO_VDMABUF_CARVEOUT_TYPE_ENC,
	VIRTIO_VDMABUF_CARVEOUT_TYPE_DEC,
	VIRTIO_VDMABUF_CARVEOUT_TYPE_GPU,
	VIRTIO_VDMABUF_CARVEOUT_TYPE_DPU,
	VIRTIO_VDMABUF_CARVEOUTS_NUM,
};

#endif
