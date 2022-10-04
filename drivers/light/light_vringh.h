/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef _LIGHT_VRINGH_H
#define _LIGHT_VRINGH_H

#include <linux/vringh.h>
#include <linux/virtio_config.h>
#include <linux/virtio.h>
#include <linux/netdevice.h>
#include "../vhost/vhost.h"

#define CNT_MASK		0x1F

#define LIGHT_MAX_VRINGS	32

struct light_vring {
	struct vring vr;
	void *va;
	int len;
};

struct light_vringh {
	struct light_vring vring;
	struct vringh vrh;
	struct vringh_kiov riov;
	struct vringh_kiov wiov;
	u16 head;
	struct mutex vr_mutex;
	struct light_vdev *vdev;
};

struct light_vdev {
	int virtio_id;
	struct device dev;
	struct device *device;
	wait_queue_head_t waitq;
	unsigned long out_bytes;
	unsigned long in_bytes;
	unsigned long out_bytes_dma;
	unsigned long in_bytes_dma;
	struct mutex mutex;
	spinlock_t lock;
	int wakeup;
	volatile unsigned char __iomem *frontend_intr_reg;
	volatile unsigned char __iomem *backend_intr_reg;
	u32 enable_intr;
	u32 clear_intr;
	int irq;
#ifdef CONFIG_LIGHT_NET
	struct napi_struct napi;
#endif
	struct work_struct work;
	struct dma_chan *dma_ch[2];
	struct list_head list;
	struct task_struct *task;
#ifdef CONFIG_LIGHT_BLK
	struct vhost_virtqueue vqs[1];
	struct vhost_dev vhost_dev;
	struct block_device *this_bdev;
#endif
	void *priv;
	int vq_num;
	int index;
	void (*kick)(void *);
	void (*notify)(struct light_vdev *);
	struct light_vringh vvr[LIGHT_MAX_VRINGS];
};

#endif
