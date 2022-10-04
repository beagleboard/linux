/* SPDX-License-Identifier: (MIT OR GPL-2.0) */
#ifndef _LINUX_VIRTIO_VDMABUF_H
#define _LINUX_VIRTIO_VDMABUF_H

#include <uapi/linux/virtio_vdmabuf.h>
#include <linux/hashtable.h>
#include "../../drivers/vhost/vhost.h"

#define UINT32_MAX			((u32)~0U)
#define VIRTIO_VDMABUF_MAX_ID		UINT32_MAX
#define VIRTIO_VDMABUF_HASH_BITS	7
#define VIRTIO_VDMABUF_CARVEOUT_NAME_LEN	16
#define VIRTIO_VDMABUF_MAX_OPS		6
#define VIRTIO_VDMABUF_MAX_BP_NUM	(VIRTIO_VDAMBUF_MAX_ALLOC_SIZE / PAGE_SIZE)
#define VIRTIO_VDMABUF_CARVEOUT_NAMES	{{"vi"},{"vo"},{"enc"},{"dec"},{"gpu"},{"dpu"}}

struct buf_pair {
	struct page *page;
	phys_addr_t addr;
	unsigned int size;
};

struct virtio_vdmabuf_buf {
	int vmid;
	size_t size;
	unsigned int buf_id;

	struct mutex lock;

	/* list of devices attached to this buffer */
	struct list_head attachments;

	struct dma_buf *dma_buf;

	/* validity of the buffer */
	bool valid;

	/* set if the buffer is imported from remote */
	bool imported;

	/* size of private */
	size_t sz_priv;
	/* private data associated with the exported buffer */
	void *priv;

	void *host;

	struct hlist_node node;

	/* real buf info */
	int heap_type;
	int carveout_type;
	int flags;
	int bp_num;
	struct buf_pair bp[];
};

struct virtio_vdmabuf_attachment {
	struct device *dev;
	struct sg_table *sgt;
	struct list_head list;
};

struct carveout_buf {
	phys_addr_t addr;
	unsigned int size;
	bool ready;
};

struct virtio_vdmabuf_event {
	struct list_head list;

	/*
	 * 0: vmid
	 * 1: buf_id
	 * 2: heap_type
	 * 3: carveout_type
	 * 4: flags
	 * 5: bp num
	 */
	unsigned int op[VIRTIO_VDMABUF_MAX_OPS];
	struct buf_pair bp[];
};

struct virtio_vdmabuf_event_queue {
	wait_queue_head_t e_wait;
	struct list_head e_list;

	spinlock_t e_lock;
	struct mutex e_readlock;
};

/* driver information */
struct virtio_vdmabuf_info {
	struct device *dev;

	struct list_head head_vdmabuf_list;
	struct list_head kvm_instances;

	spinlock_t local_lock;
	DECLARE_HASHTABLE(buf_list_local, VIRTIO_VDMABUF_HASH_BITS);
	spinlock_t import_lock;
	DECLARE_HASHTABLE(buf_list_import, VIRTIO_VDMABUF_HASH_BITS);

	void *priv;
	struct mutex g_mutex;
	bool host_ready;
};

struct vhost_vdmabuf;
struct virtio_vdmabuf;

/*
 * IOCTL definitions
 */
typedef int (*vhost_vdmabuf_ioctl_t)(struct vhost_vdmabuf *vdmabuf,
				      void *data);
typedef int (*virtio_vdmabuf_ioctl_t)(struct virtio_vdmabuf *vdmabuf,
				      void *data);

struct vhost_vdmabuf_ioctl_desc {
	unsigned int cmd;
	int flags;
	vhost_vdmabuf_ioctl_t func;
	const char *name;
};

struct virtio_vdmabuf_ioctl_desc {
	unsigned int cmd;
	int flags;
	virtio_vdmabuf_ioctl_t func;
	const char *name;
};

#define VIRTIO_VDMABUF_IOCTL_DEF(ioctl, _func, _flags)	\
	[_IOC_NR(ioctl)] = {			\
			.cmd = ioctl,		\
			.func = _func,		\
			.flags = _flags,	\
			.name = #ioctl		\
}

/* msg structures */
struct virtio_vdmabuf_msg {
	struct list_head list;
	unsigned int cmd;

	/*
	 * 0: vmid
	 * 1: buf_id
	 * 2: heap_type
	 * 3: carveout_type
	 * 4: flags
	 * 5: bp num
	 */
	unsigned int op[VIRTIO_VDMABUF_MAX_OPS];
	struct buf_pair bp[];
};

enum {
	VDMABUF_VQ_RECV = 0,
	VDMABUF_VQ_SEND = 1,
	VDMABUF_VQ_MAX  = 2,
};

enum virtio_vdmabuf_cmd {
	VIRTIO_VDMABUF_CMD_VMID_REQ,
	VIRTIO_VDMABUF_CMD_VMID_REPLY,
	VIRTIO_VDMABUF_CMD_IMPORT_REQ,
	VIRTIO_VDMABUF_CMD_IMPORT_REPLY,
	VIRTIO_VDMABUF_CMD_REL_NOTIFY,
};

struct virtio_vdmabuf {
        /* virtio device structure */
        struct virtio_device *vdev;

        /* virtual queue array */
        struct virtqueue *vqs[VDMABUF_VQ_MAX];

        /* ID of guest OS */
        unsigned int vmid;

        /* spin lock that needs to be acquired before accessing
         * virtual queue
         */
        spinlock_t vq_lock;
        struct mutex recv_lock;
        struct mutex send_lock;

        spinlock_t msg_list_lock;
        struct list_head msg_list;

        /* workqueue */
        struct workqueue_struct *wq;
        struct work_struct recv_work;
        struct work_struct send_work;
        struct work_struct send_msg_work;

        struct virtio_vdmabuf_event_queue *eq_import;
};

struct vhost_vdmabuf {
	struct vhost_dev dev;
	struct vhost_virtqueue vqs[VDMABUF_VQ_MAX];
	struct vhost_work send_work;
	struct virtio_vdmabuf_event_queue *eq_import;
	unsigned int vmid;

	struct list_head msg_list;
	struct list_head list;

	spinlock_t local_lock;
	DECLARE_HASHTABLE(buf_list_local, VIRTIO_VDMABUF_HASH_BITS);
	spinlock_t import_lock;
	DECLARE_HASHTABLE(buf_list_import, VIRTIO_VDMABUF_HASH_BITS);
};

static inline int vdmabuf_msg_size(int bp_num)
{
	return sizeof(struct virtio_vdmabuf_msg) +
			sizeof(struct buf_pair) * bp_num;
}

/* guest api */
static inline int
virtio_vdmabuf_add_buf(struct virtio_vdmabuf_info *info,
		       struct virtio_vdmabuf_buf *new,
		       bool local)
{
	if (local)
		hash_add(info->buf_list_local, &new->node, new->buf_id);
	else
		hash_add(info->buf_list_import, &new->node, new->buf_id);

	return 0;
}

static inline struct virtio_vdmabuf_buf
*virtio_vdmabuf_find_buf(struct virtio_vdmabuf_info *info,
			 unsigned int buf_id, bool local)
{
	struct virtio_vdmabuf_buf *found;

	if (local) {
		hash_for_each_possible(info->buf_list_local, found, node, buf_id)
			if (found->buf_id == buf_id)
				return found;
	} else {
		hash_for_each_possible(info->buf_list_import, found, node, buf_id)
			if (found->buf_id == buf_id)
				return found;
	}

	return NULL;
}

static inline int
virtio_vdmabuf_del_buf(struct virtio_vdmabuf_info *info,
		       unsigned int buf_id, bool local)
{
	struct virtio_vdmabuf_buf *found;

	found = virtio_vdmabuf_find_buf(info, buf_id, local);
	if (!found)
		return -ENOENT;

	hash_del(&found->node);

	return 0;
}

/* host api */
static inline int
vhost_vdmabuf_add_buf(struct vhost_vdmabuf *vdmabuf,
		      struct virtio_vdmabuf_buf *new,
		      bool local)
{
	if (local)
		hash_add(vdmabuf->buf_list_local, &new->node, new->buf_id);
	else
		hash_add(vdmabuf->buf_list_import, &new->node, new->buf_id);

	return 0;
}

static inline struct virtio_vdmabuf_buf
*vhost_vdmabuf_find_buf(struct vhost_vdmabuf *vdmabuf,
			unsigned int buf_id, bool local)
{
	struct virtio_vdmabuf_buf *found;

	if (local) {
		hash_for_each_possible(vdmabuf->buf_list_local, found, node, buf_id)
			if (found->buf_id == buf_id)
				return found;
	} else {
		hash_for_each_possible(vdmabuf->buf_list_import, found, node, buf_id)
			if (found->buf_id == buf_id)
				return found;
	}

	return NULL;
}

static inline int
vhost_vdmabuf_del_buf(struct vhost_vdmabuf *vdmabuf,
		      unsigned int buf_id, bool local)
{
	struct virtio_vdmabuf_buf *found;

	found = vhost_vdmabuf_find_buf(vdmabuf, buf_id, local);
	if (!found)
		return -ENOENT;

	hash_del(&found->node);

	return 0;
}

#endif
