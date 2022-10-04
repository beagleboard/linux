// SPDX-License-Identifier: (MIT OR GPL-2.0)
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/random.h>
#include <linux/poll.h>
#include <linux/spinlock.h>
#include <linux/dma-buf.h>
#include <linux/virtio.h>
#include <linux/virtio_ids.h>
#include <linux/virtio_config.h>
#include <linux/virtio_vdmabuf.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include <linux/of_fdt.h>
#include <linux/of.h>

/* one global drv object */
static struct virtio_vdmabuf_info *drv_info = NULL;

/*
 * carveout_buf config demo in dts,
 *
 * vdmabuf_reserved_memory {
 * 	reg = <0x0 0x82000000 0x0 0x4000
 *	       0x0 0x82004000 0x0 0x4000
 *	       0x0 0x82008000 0x0 0x4000>;
 *	reg-names = "vi", "vo", "enc";
 * };
 */
static struct carveout_buf carveout_bufs[VIRTIO_VDMABUF_CARVEOUTS_NUM] = { 0 };

static char carveout_names[VIRTIO_VDMABUF_CARVEOUTS_NUM][VIRTIO_VDMABUF_CARVEOUT_NAME_LEN] =
			VIRTIO_VDMABUF_CARVEOUT_NAMES;

static unsigned int get_buf_id(void)
{
	static int buf_id = 0;

	buf_id = buf_id < VIRTIO_VDMABUF_MAX_ID ? buf_id + 1 : 0;
	return buf_id;
}

static int carveout_buf_setup(void)
{
	struct device_node *node;
	struct resource res;
	int i, index;
	int ret;

	node = of_find_node_by_name(NULL, "vdmabuf_reserved_memory");
	if (!node) {
		ret = -EINVAL;
		dev_err(drv_info->dev,
			"failed to find vdmabuf_reserved_memory node\n");
	}

	for (i = 0; i <= VIRTIO_VDMABUF_CARVEOUTS_NUM; i++) {
		index = of_property_match_string(node, "reg-names",
						 carveout_names[i]);
		if (index < 0)
			goto exit;

		if (of_address_to_resource(node, index, &res))
			goto exit;

		carveout_bufs[i].addr = res.start;
		carveout_bufs[i].size = resource_size(&res);
		carveout_bufs[i].ready = true;
	}

exit:
	of_node_put(node);
	return index;
}

static struct sg_table *get_sg_table(struct virtio_vdmabuf_buf *exp_buf)
{
	int heap_type = exp_buf->heap_type;
	struct carveout_buf *carveout_sg;
	struct sg_table *sgt;
	struct scatterlist *sgl;
	int i, ret;


	switch (heap_type) {
	case VIRTIO_VDMABUF_HEAP_TYPE_USER:
	case VIRTIO_VDMABUF_HEAP_TYPE_SYSTEM:
	case VIRTIO_VDMABUF_HEAP_TYPE_SYSTEM_CONTIG:
		/* SYSTEM, SYSTEM_CONFIG has the same logic */
		sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
		if (!sgt)
			return ERR_PTR(-ENOMEM);

		ret = sg_alloc_table(sgt, exp_buf->bp_num, GFP_KERNEL);
		if (ret) {
			kfree(sgt);
			return ERR_PTR(-ENOMEM);
		}

		sgl = sgt->sgl;

		for (i = 0; i < exp_buf->bp_num; i++) {
			sg_set_page(sgl, exp_buf->bp[i].page,
				    exp_buf->bp[i].size, 0);
			sgl = sg_next(sgl);
		}

		break;

	case VIRTIO_VDMABUF_HEAP_TYPE_CARVEOUT:
		carveout_sg = kzalloc(sizeof(struct carveout_buf), GFP_KERNEL);
		if (!carveout_sg)
			return ERR_PTR(-ENOMEM);

		carveout_sg->addr = exp_buf->bp[0].addr;
		carveout_sg->size = exp_buf->bp[0].size;

		sgt = (struct sg_table *)carveout_sg;

		break;

	default:
		return NULL;
	}

	return sgt;
}

static void put_sg_table(struct virtio_vdmabuf_buf *buf,
			 struct sg_table *sgt)
{
	int heap_type = buf->heap_type;

	switch (heap_type) {
	case VIRTIO_VDMABUF_HEAP_TYPE_USER:
	case VIRTIO_VDMABUF_HEAP_TYPE_SYSTEM:
	case VIRTIO_VDMABUF_HEAP_TYPE_SYSTEM_CONTIG:
		sg_free_table(sgt);
		kfree(sgt);
		break;

	case VIRTIO_VDMABUF_HEAP_TYPE_CARVEOUT:
		kfree(sgt);
		break;

	default:
		break;
	}
}

static int sg_table_map(struct device *dev, struct virtio_vdmabuf_buf *exp_buf,
			struct sg_table *sgt, enum dma_data_direction dir)
{
	int heap_type = exp_buf->heap_type;
	struct carveout_buf *carveout_sg;
	dma_addr_t dma_handle;

	switch (heap_type) {
	case VIRTIO_VDMABUF_HEAP_TYPE_USER:
	case VIRTIO_VDMABUF_HEAP_TYPE_SYSTEM:
	case VIRTIO_VDMABUF_HEAP_TYPE_SYSTEM_CONTIG:
		/* SYSTEM, SYSTEM_CONFIG has the same logic */
		if (dma_map_sgtable(dev, sgt, dir, 0)) {
			dev_err(dev, "[%s:%d] error\n",
				__func__, __LINE__);
			sg_free_table(sgt);
			kfree(sgt);
			return -EINVAL;
		}

		break;

	case VIRTIO_VDMABUF_HEAP_TYPE_CARVEOUT:
		carveout_sg = (struct carveout_buf *)sgt;
		dma_handle = dma_map_single(dev, (void *)carveout_sg->addr,
					    carveout_sg->size, dir);
		if (dma_mapping_error(dev, dma_handle)) {
			dev_err(dev, "[%s:%d] error\n",
				__func__, __LINE__);
			kfree(carveout_sg);
			return -EINVAL;
		}

		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int sg_table_unmap(struct device *dev, struct virtio_vdmabuf_buf *exp_buf,
			  struct sg_table *sgt, enum dma_data_direction dir)
{
	int heap_type = exp_buf->heap_type;

	switch (heap_type) {
	case VIRTIO_VDMABUF_HEAP_TYPE_USER:
	case VIRTIO_VDMABUF_HEAP_TYPE_SYSTEM:
	case VIRTIO_VDMABUF_HEAP_TYPE_SYSTEM_CONTIG:
		dma_unmap_sgtable(dev, sgt, dir, 0);
		break;

	case VIRTIO_VDMABUF_HEAP_TYPE_CARVEOUT:
		dma_unmap_single(dev, exp_buf->bp[0].addr,
				 exp_buf->bp[0].size, dir);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static struct sg_table *virtio_vdmabuf_dmabuf_map(struct dma_buf_attachment *attachment,
						  enum dma_data_direction dir)
{
	struct virtio_vdmabuf_buf *exp_buf = attachment->dmabuf->priv;
	struct virtio_vdmabuf_attachment *a = attachment->priv;
	struct sg_table *sgt = a->sgt;
	int ret;

	ret = sg_table_map(a->dev, exp_buf, sgt, dir);
	if (ret)
		return ERR_PTR(ret);

	return sgt;
}

static void virtio_vdmabuf_dmabuf_unmap(struct dma_buf_attachment *attachment,
					struct sg_table *sgt,
					enum dma_data_direction dir)
{
	struct virtio_vdmabuf_buf *exp_buf = attachment->dmabuf->priv;

	sg_table_unmap(attachment->dev, exp_buf, sgt, dir);
}

static int virtio_vdmabuf_dmabuf_mmap(struct dma_buf *dmabuf,
				      struct vm_area_struct *vma)
{
	struct virtio_vdmabuf_buf *exp_buf = dmabuf->priv;
	unsigned long start, pfn;
	int size;
	int i, ret = 0;

	if (!exp_buf)
		return -EINVAL;

	if (vma->vm_end - vma->vm_start > exp_buf->size) {
		dev_warn(drv_info->dev,
			 "vm_end[%lu] - vm_start[%lu] [%lu] > mem size[%ld]\n",
			 vma->vm_end, vma->vm_start,
			 vma->vm_end - vma->vm_start,
			 exp_buf->size);
		return -EINVAL;
	}

	if (exp_buf->flags & VIRTIO_VDAMBUF_NONCACHED)
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	start = vma->vm_start;

	switch (exp_buf->heap_type) {
	case VIRTIO_VDMABUF_HEAP_TYPE_USER:
	case VIRTIO_VDMABUF_HEAP_TYPE_SYSTEM:
	case VIRTIO_VDMABUF_HEAP_TYPE_SYSTEM_CONTIG:
		/* SYSTEM, SYSTEM_CONFIG has the same logic */
		for (i = 0; i < exp_buf->bp_num; i++) {
			pfn = page_to_pfn(exp_buf->bp[i].page);
			size = exp_buf->bp[i].size;
			ret = remap_pfn_range(vma, start, pfn, size,
					      vma->vm_page_prot);
			if (ret)
				return ret;

			start += size;
		}

		break;

	case VIRTIO_VDMABUF_HEAP_TYPE_CARVEOUT:
		ret = vm_iomap_memory(vma, exp_buf->bp[i].addr,
				      exp_buf->bp[i].size);
		break;

	default:
		break;
	}

	return ret;
}

static void *virtio_vdmabuf_dmabuf_vmap(struct dma_buf *dmabuf)
{
	struct virtio_vdmabuf_buf *exp_buf = dmabuf->priv;
	int heap_type = exp_buf->heap_type;
	struct page **pages;
	unsigned long pfn;
	void *addr;
	int i, nr_pages;

	switch (heap_type) {
	case VIRTIO_VDMABUF_HEAP_TYPE_USER:
	case VIRTIO_VDMABUF_HEAP_TYPE_SYSTEM:
		nr_pages = exp_buf->bp_num;
		pages = kzalloc(nr_pages * sizeof(struct page *),
				GFP_KERNEL);
		if (!pages)
			return ERR_PTR(-ENOMEM);

		for (i = 0; i < exp_buf->bp_num; i++)
			pages[i] = exp_buf->bp[i].page;

		addr = vm_map_ram(pages, exp_buf->bp_num, 0); /* or vmap */
		kfree(pages);
		return addr;

	case VIRTIO_VDMABUF_HEAP_TYPE_SYSTEM_CONTIG:
		nr_pages = exp_buf->size / PAGE_SIZE;
		pages = kzalloc(nr_pages * sizeof(struct page *),
				GFP_KERNEL);
		if (!pages)
			return ERR_PTR(-ENOMEM);

		 /* convert the head page of config memory to pfn */
		pfn = page_to_pfn(exp_buf->bp[0].page);
		for (i = 0; i < nr_pages; i++)
			pages[i] = pfn_to_page(pfn + i);

		addr = vm_map_ram(pages, exp_buf->bp_num, 0); /* or vmap */
		kfree(pages);
		return addr;

	case VIRTIO_VDMABUF_HEAP_TYPE_CARVEOUT:
		return ioremap(exp_buf->bp[0].addr,
			       exp_buf->bp[0].size);

	default:
		return ERR_PTR(-EINVAL);
	}

	return NULL;
}

static void virtio_vdmabuf_dmabuf_vunmap(struct dma_buf *dmabuf, void *vaddr)
{
	struct virtio_vdmabuf_buf *exp_buf = dmabuf->priv;
	int heap_type = exp_buf->heap_type;

	switch (heap_type) {
	case VIRTIO_VDMABUF_HEAP_TYPE_USER:
	case VIRTIO_VDMABUF_HEAP_TYPE_SYSTEM:
	case VIRTIO_VDMABUF_HEAP_TYPE_SYSTEM_CONTIG:
		vm_unmap_ram(vaddr, exp_buf->bp_num);
		break;

	case VIRTIO_VDMABUF_HEAP_TYPE_CARVEOUT:
		iounmap(vaddr);
		break;

	default:
		break;
	}
}

static int send_msg_to_host(enum virtio_vdmabuf_cmd cmd, void *data,
			    int bp_num)
{
	struct virtio_vdmabuf *vdmabuf = drv_info->priv;
	struct virtio_vdmabuf_buf *exp_buf;
	struct virtio_vdmabuf_msg *msg;
	unsigned int buf_id;
	unsigned long irqflags;
	int ret;

	if (bp_num > VIRTIO_VDMABUF_MAX_BP_NUM) {
		dev_err(drv_info->dev, "[%s:%d] max bp num reached %d\n",
			__func__, __LINE__, bp_num);
		return -EINVAL;
	}

	msg = kzalloc(struct_size(msg, bp, bp_num), GFP_KERNEL);
	if (!msg)
		return -ENOMEM;
	msg->op[5] = bp_num;

	switch (cmd) {
	case VIRTIO_VDMABUF_CMD_VMID_REQ:
		/*
		 * set vmid to a default value, it will be changed
		 * after we received reply
		 */
		vdmabuf->vmid = 0;

		break;

	/* Guest played importer role */

	case VIRTIO_VDMABUF_CMD_REL_NOTIFY:
		buf_id = *(unsigned int *)data;

		spin_lock_irqsave(&drv_info->import_lock, irqflags);
		/* remove dmabuf with buf_id from local hash table */
		ret = virtio_vdmabuf_del_buf(drv_info, buf_id, false);
		spin_unlock_irqrestore(&drv_info->import_lock, irqflags);
		if (ret)
			return ret;

		msg->op[0] = vdmabuf->vmid;
		msg->op[1] = buf_id;

		break;

	case VIRTIO_VDMABUF_CMD_IMPORT_REQ:
		buf_id = *(unsigned int *)data;
		msg->op[0] = vdmabuf->vmid;
		msg->op[1] = buf_id;

		break;

	/* Guest played exporter role */
	case VIRTIO_VDMABUF_CMD_IMPORT_REPLY:
		exp_buf = (struct virtio_vdmabuf_buf *)data;

		msg->op[0] = exp_buf->vmid;
		msg->op[1] = exp_buf->buf_id;
		msg->op[2] = exp_buf->heap_type;
		msg->op[3] = exp_buf->carveout_type;
		msg->op[4] = exp_buf->flags;
		msg->op[5] = exp_buf->bp_num;
		memcpy(msg->bp, exp_buf->bp, sizeof(exp_buf->bp[0]) * exp_buf->bp_num);

		break;

	default:
		/* no command found */
		kfree(msg);
		return -EINVAL;
	}

	msg->cmd = cmd;

	spin_lock_irqsave(&vdmabuf->msg_list_lock, irqflags);
	list_add_tail(&msg->list, &vdmabuf->msg_list);
	spin_unlock_irqrestore(&vdmabuf->msg_list_lock, irqflags);

	queue_work(vdmabuf->wq, &vdmabuf->send_msg_work);

	return 0;
}

#if 0
static void virtio_vdmabuf_clear_buf(struct virtio_vdmabuf_buf *exp_buf)
{
	dma_buf_unmap_attachment(exp_buf->attach, exp_buf->sgt,
				 DMA_BIDIRECTIONAL);

	if (exp_buf->dma_buf) {
		dma_buf_detach(exp_buf->dma_buf, exp_buf->attach);
		/* close connection to dma-buf completely */
		dma_buf_put(exp_buf->dma_buf);
		exp_buf->dma_buf = NULL;
	}
}
#endif

static int virtio_vdmabuf_remove_buf(struct virtio_vdmabuf_info *drv_info,
				     struct virtio_vdmabuf_buf *exp_buf,
				     bool local)
{
	unsigned int buf_id = exp_buf->buf_id;
	unsigned long irqflags;
	spinlock_t *lock;
	int ret;

	//virtio_vdmabuf_clear_buf(exp_buf);

	if (local)
		lock = &drv_info->local_lock;
	else
		lock = &drv_info->import_lock;

	spin_lock_irqsave(lock, irqflags);
	ret = virtio_vdmabuf_del_buf(drv_info, buf_id, local);
	spin_unlock_irqrestore(lock, irqflags);
	if (ret)
		return ret;

	kfree(exp_buf);
	return 0;
}

/* parse msg from host */
static int parse_msg_from_host(struct virtio_vdmabuf *vdmabuf,
			       struct virtio_vdmabuf_msg *msg)
{
	struct virtio_vdmabuf_event *event;
	struct virtio_vdmabuf_buf *exp_buf;
	unsigned long irqflags;
	unsigned int buf_id;
	unsigned vmid;
	int i, bp_num;

	dev_dbg(drv_info->dev, "received msg cmd %d\n", msg->cmd);

	switch (msg->cmd) {
	/* Host's rely for guest's VMID request by open() */
	case VIRTIO_VDMABUF_CMD_VMID_REPLY:
		vdmabuf->vmid = msg->op[0];
		if (!vdmabuf->vmid)
			dev_err(drv_info->dev, "vmid should be not 0\n");

		dev_dbg(drv_info->dev, "vmid %d\n", vdmabuf->vmid);

		wake_up_interruptible(&vdmabuf->eq_import->e_wait);

		break;

	/* 1, Guest played dmabuf exporter role */

	/* Guest received import request from host */
	case VIRTIO_VDMABUF_CMD_IMPORT_REQ:
		vmid = msg->op[0];
		if (vdmabuf->vmid != vmid) {
			dev_err(drv_info->dev, "vmid does not match %d %d\n",
				vdmabuf->vmid, vmid);

			return -EINVAL;
		}

		buf_id = msg->op[1];

		spin_lock_irqsave(&drv_info->local_lock, irqflags);
		exp_buf = virtio_vdmabuf_find_buf(drv_info, buf_id, true);
		if (!exp_buf) {
			spin_unlock_irqrestore(&drv_info->local_lock, irqflags);
			dev_err(drv_info->dev, "no exp_buf found for buf id %d\n",
				buf_id);

			return -ENOENT;
		}
		spin_unlock_irqrestore(&drv_info->local_lock, irqflags);

		send_msg_to_host(VIRTIO_VDMABUF_CMD_IMPORT_REPLY, exp_buf,
				 exp_buf->bp_num);

		/*
		 * Only increment the reference count on the dmabuf ,Then this
		 * dmabuf won't be released at this side until we received
		 * REL_NOTIFY command to decrease the reference count.
		 */
		get_dma_buf(exp_buf->dma_buf);

		break;

	/* Guest received dmabuf released request from host */
	case VIRTIO_VDMABUF_CMD_REL_NOTIFY:
		vmid = msg->op[0];
		if (vdmabuf->vmid != vmid) {
			dev_err(drv_info->dev, "[%s:%d] %d %d\n",
				__func__, __LINE__, vdmabuf->vmid, vmid);
			return -EINVAL;
		}

		buf_id = msg->op[1];

		spin_lock_irqsave(&drv_info->local_lock, irqflags);
		exp_buf = virtio_vdmabuf_find_buf(drv_info, buf_id, true);
		if (!exp_buf) {
			spin_unlock_irqrestore(&drv_info->local_lock, irqflags);
			dev_err(drv_info->dev, "can't find buffer\n");
			return -ENOENT;
		}

		dma_buf_put(exp_buf->dma_buf);
		spin_unlock_irqrestore(&drv_info->local_lock, irqflags);

		break;

	/* 2, Guest played dmabuf importer role */

	/* Guest received host's ACK for its import request
	 * by ioctl(..,VIRTIO_VDMABUF_IOCTL_GET_FD,..)
	 */
	case VIRTIO_VDMABUF_CMD_IMPORT_REPLY:
		vmid = msg->op[0];
		bp_num = msg->op[5];

		if (vdmabuf->vmid != vmid) {
			dev_err(drv_info->dev, "[%s:%d] %d %d\n",
				__func__, __LINE__, vdmabuf->vmid, vmid);
			return -EINVAL;
		}

		if (bp_num > VIRTIO_VDMABUF_MAX_BP_NUM) {
			dev_err(drv_info->dev, "[%s:%d] max bp num reached %d\n",
				__func__, __LINE__, bp_num);
			return -EINVAL;
		}

		event = kzalloc(struct_size(event, bp, bp_num), GFP_KERNEL);
		if (!event)
			return -ENOMEM;

		memcpy(event->op, msg->op, sizeof(event->op));
		for (i = 0; i < bp_num; i++) {
			/*
			 * no need to copy page info, as the page info from
			 * host is invalid at guest side
			 */
			event->bp[i].addr = msg->bp[i].addr;
			event->bp[i].size = msg->bp[i].size;
		}

		spin_lock_irqsave(&vdmabuf->eq_import->e_lock, irqflags);
		list_add_tail(&event->list, &vdmabuf->eq_import->e_list);
		wake_up_interruptible(&vdmabuf->eq_import->e_wait);
		spin_unlock_irqrestore(&vdmabuf->eq_import->e_lock, irqflags);

		break;

	default:
		dev_err(drv_info->dev, "invalid cmd\n");
		return -EINVAL;
	}

	return 0;
}

static int virtio_vdmabuf_fill_recv_msg(struct virtio_vdmabuf *vdmabuf,
					struct virtio_vdmabuf_msg *msg)
{
	struct virtqueue *vq = vdmabuf->vqs[VDMABUF_VQ_RECV];
	struct scatterlist sg;

	if (!msg)
		return -EINVAL;

	sg_init_one(&sg, msg, sizeof(struct virtio_vdmabuf_msg));
	return virtqueue_add_inbuf(vq, &sg, 1, msg, GFP_ATOMIC);
}

static int virtio_vdambuf_fill_queue(struct virtqueue *vq)
{
	struct virtqueue *vq_ = vq;
	struct virtio_vdmabuf_msg *msg;
	struct scatterlist sg;
	int added = 0;
	int ret, size;

	do {
		msg = kzalloc(struct_size(msg, bp, VIRTIO_VDMABUF_MAX_BP_NUM),
			      GFP_KERNEL);
		if (!msg)
			break;

		size = sizeof(struct virtio_vdmabuf_msg) +
			sizeof(struct buf_pair) * VIRTIO_VDMABUF_MAX_BP_NUM;
		sg_init_one(&sg, msg, size);
		ret = virtqueue_add_inbuf(vq_, &sg, 1, msg, GFP_KERNEL);
		if (ret) {
			kfree(msg);
			break;
		}

		added++;
	} while(vq->num_free);

	dev_info(drv_info->dev, "filled %d msg buffers to vq\n", added);
	return added;
}

static void virtio_vdmabuf_recv_work(struct work_struct *work)
{
	struct virtio_vdmabuf *vdmabuf =
		container_of(work, struct virtio_vdmabuf, recv_work);
	struct virtqueue *vq = vdmabuf->vqs[VDMABUF_VQ_RECV];
	struct virtio_vdmabuf_msg *msg;
	int sz, ret;

	mutex_lock(&vdmabuf->recv_lock);

	do {
		virtqueue_disable_cb(vq);
		for (;;) {
			msg = virtqueue_get_buf(vq, &sz);
			if (!msg)
				break;

			/* valid size */
			if (sz == vdmabuf_msg_size(VIRTIO_VDMABUF_MAX_BP_NUM)) {
				ret = parse_msg_from_host(vdmabuf, msg);
				if (ret)
					dev_err(drv_info->dev, "msg parse error %d\n",
						ret);

				ret = virtio_vdmabuf_fill_recv_msg(vdmabuf, msg);
				if (ret < 0) {
					dev_warn(drv_info->dev,
						 "failed to fill recv msg to vq\n");
					kfree(msg);
				}
			} else
				dev_err(drv_info->dev,
					"received malformed message\n");
		}
	} while (!virtqueue_enable_cb(vq));

	mutex_unlock(&vdmabuf->recv_lock);
}

static void virtio_vdmabuf_send_msg_work(struct work_struct *work)
{
	struct virtio_vdmabuf *vdmabuf =
		container_of(work, struct virtio_vdmabuf, send_msg_work);
	struct virtqueue *vq = vdmabuf->vqs[VDMABUF_VQ_SEND];
	struct scatterlist sg;
	struct virtio_vdmabuf_msg *msg;
	unsigned long irqflags;
	bool added = false;
	int ret, size;

	mutex_lock(&vdmabuf->send_lock);

	for (;;) {
		spin_lock_irqsave(&vdmabuf->msg_list_lock, irqflags);
		if (list_empty(&vdmabuf->msg_list)) {
			spin_unlock_irqrestore(&vdmabuf->msg_list_lock, irqflags);
			break;
		}

		msg = list_first_entry(&vdmabuf->msg_list,
				       struct virtio_vdmabuf_msg, list);
		if (!msg) {
			dev_warn(drv_info->dev, "msg is null\n");
			spin_unlock_irqrestore(&vdmabuf->msg_list_lock, irqflags);
			continue;
		}

		list_del_init(&msg->list);
		spin_unlock_irqrestore(&vdmabuf->msg_list_lock, irqflags);

		size = sizeof(struct virtio_vdmabuf_msg) +
				sizeof(struct buf_pair) * msg->op[5];

		dev_dbg(drv_info->dev, "send msg cmd %d, size %d\n", msg->cmd, size);

		sg_init_one(&sg, msg, size);
		ret = virtqueue_add_outbuf(vq, &sg, 1, msg, GFP_KERNEL);
		if (ret < 0) {
			dev_err(drv_info->dev, "failed to add msg to vq\n");
			break;
		}

		added = true;
	}

	if (added)
		virtqueue_kick(vq);

	mutex_unlock(&vdmabuf->send_lock);
}

static void virtio_vdmabuf_send_work(struct work_struct *work)
{
	struct virtio_vdmabuf *vdmabuf =
		container_of(work, struct virtio_vdmabuf, send_work);
	struct virtqueue *vq = vdmabuf->vqs[VDMABUF_VQ_SEND];
	struct virtio_vdmabuf_msg *msg;
	unsigned int sz;
	bool added = false;

	mutex_lock(&vdmabuf->send_lock);

	do {
		virtqueue_disable_cb(vq);

		for (;;) {
			msg = virtqueue_get_buf(vq, &sz);
			if (!msg)
				break;

			kfree(msg);
			added = true;
		}
	} while (!virtqueue_enable_cb(vq));

	mutex_unlock(&vdmabuf->send_lock);

	/* use this chance to send msg to host if we have */
	if (added)
		queue_work(vdmabuf->wq, &vdmabuf->send_msg_work);
}

static void virtio_vdmabuf_recv_cb(struct virtqueue *vq)
{
	struct virtio_vdmabuf *vdmabuf = vq->vdev->priv;

	if (!vdmabuf)
		return;

	queue_work(vdmabuf->wq, &vdmabuf->recv_work);
}

static void virtio_vdmabuf_send_cb(struct virtqueue *vq)
{
	struct virtio_vdmabuf *vdmabuf = vq->vdev->priv;

	if (!vdmabuf)
		return;

	queue_work(vdmabuf->wq, &vdmabuf->send_work);
}

static void virtio_vdmabuf_empty_queue(struct virtqueue *vq)
{
	void *buf;
	int sz;

	while (1) {
		buf = virtqueue_get_buf(vq, &sz);
		if (buf == NULL)
			break;
		kfree(buf);
	}
}

static int virtio_vdmabuf_remove_all_bufs(struct virtio_vdmabuf *vdmabuf)
{
	struct virtio_vdmabuf_buf *found;
	struct hlist_node *tmp;
	struct virtqueue *vq;
	int bkt;
	int ret;

	hash_for_each_safe(drv_info->buf_list_local, bkt,
			   tmp, found, node) {
		ret = virtio_vdmabuf_remove_buf(drv_info,
						found, true);
		if (ret)
			return ret;
	}

	hash_for_each_safe(drv_info->buf_list_import, bkt,
			   tmp, found, node) {
		ret = virtio_vdmabuf_remove_buf(drv_info,
						found, false);
		if (ret)
			return ret;
	}

	if (drv_info->host_ready) {
		vq = vdmabuf->vqs[VDMABUF_VQ_RECV];
		virtio_vdmabuf_empty_queue(vq);
	}

	return 0;
}

static void virtio_vdmabuf_release_priv(struct virtio_vdmabuf_buf *exp_buf)
{
	int i;

	switch (exp_buf->heap_type) {
	case VIRTIO_VDMABUF_HEAP_TYPE_USER:
	case VIRTIO_VDMABUF_HEAP_TYPE_SYSTEM:
		for (i = 0; i < exp_buf->bp_num; i++)
			put_page(exp_buf->bp[i].page);

		break;

	case VIRTIO_VDMABUF_HEAP_TYPE_SYSTEM_CONTIG:
		__free_pages(exp_buf->bp[0].page,
			     get_order(exp_buf->bp[0].size));
		break;

	case VIRTIO_VDMABUF_HEAP_TYPE_CARVEOUT:
		/* no need to release */
		break;

	default:
		break;
	}
}

static void virtio_vdmabuf_dmabuf_release(struct dma_buf *dmabuf)
{
	struct virtio_vdmabuf_buf *exp_buf = dmabuf->priv;
	unsigned long irqflags;
	int buf_id;
	int ret;

	if (!exp_buf)
		return;

	exp_buf->valid = false;
	exp_buf->dma_buf = NULL;
	buf_id = exp_buf->buf_id;

	if (exp_buf->imported) {
		ret = send_msg_to_host(VIRTIO_VDMABUF_CMD_REL_NOTIFY,
				       &buf_id, 0);
		if (ret < 0)
			dev_err(drv_info->dev,
				"failed(%d) to send dmabuf(%d) release cmd\n",
				ret, buf_id);
	} else {
		spin_lock_irqsave(&drv_info->local_lock, irqflags);
		ret = virtio_vdmabuf_del_buf(drv_info, buf_id, true);
		spin_unlock_irqrestore(&drv_info->local_lock, irqflags);
		if (ret)
			dev_err(drv_info->dev,
				"failed(%d) to del dmabuf(%d) from local list\n",
				ret, buf_id);

		virtio_vdmabuf_release_priv(exp_buf);
	}

	kfree(exp_buf);
}

static int virtio_vdmabuf_dmabuf_begin_cpu_access(struct dma_buf *dmabuf,
						  enum dma_data_direction dir)
{
	struct virtio_vdmabuf_buf *exp_buf = dmabuf->priv;
	struct virtio_vdmabuf_attachment *a;
	int heap_type = exp_buf->heap_type;
	struct carveout_buf *c;

	mutex_lock(&exp_buf->lock);
	switch (heap_type) {
	case VIRTIO_VDMABUF_HEAP_TYPE_USER:
	case VIRTIO_VDMABUF_HEAP_TYPE_SYSTEM:
	case VIRTIO_VDMABUF_HEAP_TYPE_SYSTEM_CONTIG:
		list_for_each_entry(a, &exp_buf->attachments, list)
			dma_sync_sgtable_for_cpu(a->dev, a->sgt, dir);
		break;

	case VIRTIO_VDMABUF_HEAP_TYPE_CARVEOUT:
		list_for_each_entry(a, &exp_buf->attachments, list) {
			c = (struct carveout_buf *)a->sgt;
			dma_sync_single_for_cpu(a->dev, c->addr, c->size,
						dir);
		}
		break;

	default:
		break;
	}
	mutex_unlock(&exp_buf->lock);

	return 0;
}

static int virtio_vdmabuf_dmabuf_end_cpu_access(struct dma_buf *dmabuf,
						enum dma_data_direction dir)
{
	struct virtio_vdmabuf_buf *exp_buf = dmabuf->priv;
	struct virtio_vdmabuf_attachment *a;
	int heap_type = exp_buf->heap_type;
	struct carveout_buf *c;

	mutex_lock(&exp_buf->lock);
	switch (heap_type) {
	case VIRTIO_VDMABUF_HEAP_TYPE_USER:
	case VIRTIO_VDMABUF_HEAP_TYPE_SYSTEM:
	case VIRTIO_VDMABUF_HEAP_TYPE_SYSTEM_CONTIG:
		list_for_each_entry(a, &exp_buf->attachments, list)
			dma_sync_sgtable_for_device(a->dev, a->sgt, dir);
		break;

	case VIRTIO_VDMABUF_HEAP_TYPE_CARVEOUT:
		list_for_each_entry(a, &exp_buf->attachments, list) {
			c = (struct carveout_buf *)a->sgt;
			dma_sync_single_for_device(a->dev, c->addr, c->size,
						   dir);
		}

		break;

	default:
		break;
	}
	mutex_unlock(&exp_buf->lock);

	return 0;
}

static int virtio_vdmabuf_dmabuf_attach(struct dma_buf *dmabuf,
					struct dma_buf_attachment *attachment)
{
	struct virtio_vdmabuf_buf *exp_buf = dmabuf->priv;
	struct virtio_vdmabuf_attachment *a;
	struct sg_table *sgt;

	a = kzalloc(sizeof(*a), GFP_KERNEL);
	if (!a)
		return -ENOMEM;

	sgt = get_sg_table(exp_buf);
	if (IS_ERR(sgt)) {
		kfree(a);
		return -ENOMEM;
	}

	a->sgt = sgt;
	a->dev = attachment->dev;
	INIT_LIST_HEAD(&a->list);

	attachment->priv = a;

	mutex_lock(&exp_buf->lock);
	list_add(&a->list, &exp_buf->attachments);
	mutex_unlock(&exp_buf->lock);

	return 0;
}

static void virtio_vdmabuf_dmabuf_detach(struct dma_buf *dmabuf,
					 struct dma_buf_attachment *attachment)
{
	struct virtio_vdmabuf_attachment *a = attachment->priv;
	struct virtio_vdmabuf_buf *exp_buf = dmabuf->priv;

	mutex_lock(&exp_buf->lock);
	list_del(&a->list);
	mutex_unlock(&exp_buf->lock);
	put_sg_table(exp_buf, a->sgt);

	kfree(a);
}

static const struct dma_buf_ops virtio_vdmabuf_dmabuf_ops =  {
	.attach = virtio_vdmabuf_dmabuf_attach,
	.detach = virtio_vdmabuf_dmabuf_detach,
	.map_dma_buf = virtio_vdmabuf_dmabuf_map,
	.unmap_dma_buf = virtio_vdmabuf_dmabuf_unmap,
	.release = virtio_vdmabuf_dmabuf_release,
	.mmap = virtio_vdmabuf_dmabuf_mmap,
	.vmap = virtio_vdmabuf_dmabuf_vmap,
	.vunmap = virtio_vdmabuf_dmabuf_vunmap,
	.begin_cpu_access = virtio_vdmabuf_dmabuf_begin_cpu_access,
	.end_cpu_access = virtio_vdmabuf_dmabuf_end_cpu_access,
};

static int virtio_vdmabuf_create_mirror_dmabuf(struct virtio_vdmabuf_import *attr,
					       struct virtio_vdmabuf_event *event)
{
	struct virtio_vdmabuf *vdmabuf = drv_info->priv;
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
	struct virtio_vdmabuf_buf *exp_buf;
	struct dma_buf *dmabuf;
	unsigned long irqflags;
	int heap_type;
	int carveout_type;
	unsigned int buf_id;
	phys_addr_t addr;
	int bp_num;
	int ret = -ENOMEM;
	int i, size;

	buf_id = event->op[1];
	if (attr->buf_id != buf_id)
		return -EINVAL;

	heap_type = event->op[2];
	bp_num = event->op[5];

	if (bp_num <= 0 || bp_num > VIRTIO_VDMABUF_MAX_BP_NUM)
		return -EINVAL;

	exp_buf = kzalloc(struct_size(exp_buf, bp, bp_num), GFP_KERNEL);
	if (!exp_buf)
		goto err_exp;

	exp_buf->bp_num = bp_num;

	mutex_init(&exp_buf->lock);

	switch (heap_type) {
	case VIRTIO_VDMABUF_HEAP_TYPE_SYSTEM:
		for (i = 0; i < bp_num; i++) {
			/* actually guest can't access the page from host in light */
			exp_buf->bp[i].page = phys_to_page(event->bp[i].addr);
			exp_buf->bp[i].addr = event->bp[i].addr;
			exp_buf->bp[i].size = event->bp[i].size;
			exp_buf->size += event->bp[i].size;
		}

		break;

	case VIRTIO_VDMABUF_HEAP_TYPE_SYSTEM_CONTIG:
		if (exp_buf->bp_num != 1) {
			dev_err(drv_info->dev, "[%s:%d] error %d\n",
				__func__, __LINE__, exp_buf->bp_num);
			exp_buf->bp_num = 1;
		}

		addr = event->bp[0].addr;
		exp_buf->bp[0].page = phys_to_page(addr);
		exp_buf->bp[0].size = event->bp[0].size;

		exp_buf->size = exp_buf->bp[0].size;

		break;

	case VIRTIO_VDMABUF_HEAP_TYPE_USER:
		return -EINVAL;

		break;

	case VIRTIO_VDMABUF_HEAP_TYPE_CARVEOUT:
		if (exp_buf->bp_num != 1) {
			dev_err(drv_info->dev, "[%s:%d] error %d\n",
				__func__, __LINE__, exp_buf->bp_num);
			exp_buf->bp_num = 1;
		}

		carveout_type = exp_buf->carveout_type;
		addr = event->bp[0].addr;
		size = event->bp[0].size;
		if (addr != carveout_bufs[carveout_type].addr &&
		    size != carveout_bufs[carveout_type].size) {
			dev_err(drv_info->dev, "[%s:%d] error\n",
				__func__, __LINE__);
			return -EINVAL;
		}

		exp_buf->bp[0].addr = addr;
		exp_buf->bp[0].size = size;
		exp_buf->size = size;

		break;

	default:
		return -EINVAL;
	}

	exp_info.ops = &virtio_vdmabuf_dmabuf_ops;
	exp_info.size = exp_buf->size;
	exp_info.flags = O_RDWR;
	exp_info.priv = exp_buf;

	/* export real dambuf */
	dmabuf = dma_buf_export(&exp_info);
	if (IS_ERR_OR_NULL(dmabuf))
		goto err_exp;

	ret = dma_buf_fd(dmabuf, 0);
	if (ret < 0) {
		dma_buf_put(dmabuf);
		goto err_exp;
	}
	attr->fd = ret;
	attr->size = exp_buf->size;

	INIT_LIST_HEAD(&exp_buf->attachments);
	exp_buf->vmid = vdmabuf->vmid;
	exp_buf->dma_buf = dmabuf;
	exp_buf->buf_id = attr->buf_id;
	exp_buf->heap_type = event->op[2];
	exp_buf->carveout_type = event->op[3];
	exp_buf->flags = event->op[4];

	/*
	 * it indicates this is a mirrored dmabuf,
	 * we need send release cmd to host when
	 * this dambuf is released.
	 */
	exp_buf->imported = true;
	exp_buf->valid = true;

	spin_lock_irqsave(&drv_info->import_lock, irqflags);
	virtio_vdmabuf_add_buf(drv_info, exp_buf, false);
	spin_unlock_irqrestore(&drv_info->import_lock, irqflags);

	return 0;

err_exp:
	kfree(exp_buf);
	return ret;
}

static int virtio_vdmabuf_create_dmabuf(struct virtio_vdmabuf *vdmabuf,
				        struct virtio_vdmabuf_alloc *attr)
{
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
	int carveout_type = attr->carveout_type;
	int heap_type = attr->heap_type;
	size_t size = attr->size;
	struct virtio_vdmabuf_buf *exp_buf;
	struct dma_buf *dmabuf;
	unsigned long irqflags;
	struct page *page = NULL;
	int ret, i = 0, npages, bp_num;

	/* For carveout, buf size is fixed, user don't need specify it */
	if (attr->heap_type != VIRTIO_VDMABUF_HEAP_TYPE_CARVEOUT) {
		npages = bp_num = DIV_ROUND_UP(size, PAGE_SIZE);
		if (npages <= 0)
			return -EINVAL;
	}

	if (attr->heap_type == VIRTIO_VDMABUF_HEAP_TYPE_SYSTEM_CONTIG ||
	    attr->heap_type == VIRTIO_VDMABUF_HEAP_TYPE_CARVEOUT)
		bp_num = 1;

	exp_buf = kzalloc(struct_size(exp_buf, bp, bp_num), GFP_KERNEL);
	if (!exp_buf) {
		ret = -ENOMEM;
		goto err_exp;
	}

	mutex_init(&exp_buf->lock);

	exp_info.ops = &virtio_vdmabuf_dmabuf_ops;
	exp_info.flags = O_RDWR;
	exp_info.priv = exp_buf;

	switch (heap_type) {
	case VIRTIO_VDMABUF_HEAP_TYPE_USER:
		return -EINVAL; /* Not support currently */

	case VIRTIO_VDMABUF_HEAP_TYPE_SYSTEM:
		exp_buf->size = exp_info.size = npages * PAGE_SIZE;
		exp_buf->bp_num = npages;

		for (i = 0; i < npages; i++) {
			page = alloc_page(GFP_KERNEL);
			if (!page) {
				ret = -ENOMEM;
				goto err_alloc;
			}

			exp_buf->bp[i].page = page;
			exp_buf->bp[i].addr = page_to_phys(page);
			exp_buf->bp[i].size = PAGE_SIZE;
		}

		break;

	case VIRTIO_VDMABUF_HEAP_TYPE_SYSTEM_CONTIG:
		exp_buf->size = exp_info.size = npages * PAGE_SIZE;
		/* only need 1 bp to record Compound Page */
		exp_buf->bp_num = 1;

		page = alloc_pages(GFP_KERNEL, get_order(exp_buf->size));
		if (!page) {
			ret = -ENOMEM;
			goto err_exp;
		}
		exp_buf->bp[0].page = page;
		exp_buf->bp[0].addr = page_to_phys(page);
		exp_buf->bp[0].size = exp_buf->size;

		break;

	case VIRTIO_VDMABUF_HEAP_TYPE_CARVEOUT:
		if (carveout_type >= VIRTIO_VDMABUF_CARVEOUTS_NUM ||
		    !carveout_bufs[carveout_type].ready)
			return -EINVAL;

		exp_buf->bp_num = 1;

		exp_buf->bp[0].addr = carveout_bufs[carveout_type].addr;
		if (size <= 0 || size > carveout_bufs[carveout_type].size)
			size = carveout_bufs[carveout_type].size;
		exp_buf->bp[0].size = size;

		exp_buf->size = exp_info.size = size;
		attr->size = exp_buf->size;

		break;

	default:
		/* no command found */
		ret = -EINVAL;
		goto err_exp;
	}

	/* export real dambuf */
	dmabuf = dma_buf_export(&exp_info);
	if (IS_ERR_OR_NULL(dmabuf))
		goto err_alloc;

	ret = dma_buf_fd(dmabuf, 0);
	if (ret < 0) {
		dma_buf_put(dmabuf);
		goto err_alloc;
	}
	attr->fd = ret;

	INIT_LIST_HEAD(&exp_buf->attachments);
	exp_buf->vmid = vdmabuf->vmid;
	exp_buf->heap_type = heap_type;
	exp_buf->carveout_type = carveout_type;
	exp_buf->flags = attr->flags;
	exp_buf->dma_buf = dmabuf;
	exp_buf->buf_id = get_buf_id();
	exp_buf->valid = true;
	exp_buf->imported = false;

	attr->buf_id = exp_buf->buf_id;

	spin_lock_irqsave(&drv_info->local_lock, irqflags);
	virtio_vdmabuf_add_buf(drv_info, exp_buf, true);
	spin_unlock_irqrestore(&drv_info->local_lock, irqflags);

	return 0;

err_alloc:
	if (heap_type == VIRTIO_VDMABUF_HEAP_TYPE_SYSTEM_CONTIG)
		__free_pages(page, get_order(exp_buf->size));

	while(i--)
		put_page(exp_buf->bp[i].page);
err_exp:
	kfree(exp_buf);
	return ret;
}

static int virtio_vdmabuf_open(struct inode *inode, struct file *filp)
{
	struct virtio_vdmabuf *vdmabuf;
	int ret;

	if (!drv_info) {
		dev_err(drv_info->dev, "virtio vdmabuf driver is not ready\n");
		return -EINVAL;
	}
	vdmabuf = drv_info->priv;

	mutex_lock(&drv_info->g_mutex);
	if (drv_info->host_ready) {
		ret = send_msg_to_host(VIRTIO_VDMABUF_CMD_VMID_REQ, NULL, 0);
		if (ret < 0) {
			dev_err(drv_info->dev, "fail to send vmid req\n");
			return ret;
		}

		/* host's reply of vmid req will wakeup us */
		if (wait_event_interruptible(vdmabuf->eq_import->e_wait,
					     vdmabuf->vmid != 0)) {
			dev_err(drv_info->dev, "Uh, this err is not expected\n");
			return -ERESTARTSYS;
		}
	}
	mutex_unlock(&drv_info->g_mutex);

	filp->private_data = vdmabuf;

	return 0;
}

static int import_ioctl(struct virtio_vdmabuf *vdmabuf, void *data)
{
	struct virtio_vdmabuf_import *attr = data;
	struct virtio_vdmabuf_buf *imp_buf;
	struct virtio_vdmabuf_event *event;
	unsigned long irqflags;
	int ret = 0;

	if (vdmabuf->vmid <= 0)
		return -EINVAL;

	spin_lock_irqsave(&drv_info->import_lock, irqflags);
	imp_buf = virtio_vdmabuf_find_buf(drv_info, attr->buf_id, false);
	if (imp_buf && imp_buf->valid) {
		ret = dma_buf_fd(imp_buf->dma_buf, 0);
		if (ret < 0) {
			dma_buf_put(imp_buf->dma_buf);
			spin_unlock_irqrestore(&drv_info->import_lock, irqflags);
			return ret;
		}

		attr->fd = ret;
		attr->size = imp_buf->size;
		spin_unlock_irqrestore(&drv_info->import_lock, irqflags);

		return 0;
	}
	spin_unlock_irqrestore(&drv_info->import_lock, irqflags);

	/*
	 * We can't find the dambuf with buf_id in local hash list,
	 * need send import request to host to get it.
	 */

	mutex_lock(&drv_info->g_mutex);

	send_msg_to_host(VIRTIO_VDMABUF_CMD_IMPORT_REQ, &attr->buf_id, 0);

	/* host's ack of import request will wakeup us */
	if (wait_event_interruptible(vdmabuf->eq_import->e_wait,
				     !list_empty(&vdmabuf->eq_import->e_list))) {
		mutex_unlock(&drv_info->g_mutex);
		dev_err(drv_info->dev, "This err is not expected\n");
		return -ERESTARTSYS;
	}

	spin_lock_irqsave(&vdmabuf->eq_import->e_lock, irqflags);
	event = list_first_entry(&vdmabuf->eq_import->e_list,
				 struct virtio_vdmabuf_event, list);
	/* safely del the event from list and free it */
	list_del(&event->list);
	spin_unlock_irqrestore(&vdmabuf->eq_import->e_lock, irqflags);

	/* create local mirror dmabuf */
	ret = virtio_vdmabuf_create_mirror_dmabuf(attr, event);
	if (ret)
		dev_err(drv_info->dev, "create mirror dmabuf failed %d\n",
			ret);

	kfree(event);

	mutex_unlock(&drv_info->g_mutex);

	return ret;
}

static int alloc_ioctl(struct virtio_vdmabuf *vdmabuf, void *data)
{
	struct virtio_vdmabuf_alloc *attr = data;
	int ret;

	mutex_lock(&drv_info->g_mutex);
	ret = virtio_vdmabuf_create_dmabuf(vdmabuf, attr);
	mutex_unlock(&drv_info->g_mutex);

	return ret;
}

static const struct virtio_vdmabuf_ioctl_desc virtio_vdmabuf_ioctls[] = {
	VIRTIO_VDMABUF_IOCTL_DEF(VIRTIO_VDMABUF_IOCTL_ALLOC_FD, alloc_ioctl, 0),
	VIRTIO_VDMABUF_IOCTL_DEF(VIRTIO_VDMABUF_IOCTL_IMPORT_FD, import_ioctl, 0),
};

static long virtio_vdmabuf_ioctl(struct file *filp, unsigned int cmd,
				 unsigned long param)
{
	struct virtio_vdmabuf *vdmabuf = filp->private_data;
	const struct virtio_vdmabuf_ioctl_desc *ioctl = NULL;
	unsigned int nr = _IOC_NR(cmd);
	int ret;
	virtio_vdmabuf_ioctl_t func;
	char *kdata;

	if (nr >= ARRAY_SIZE(virtio_vdmabuf_ioctls)) {
		dev_err(drv_info->dev, "invalid ioctl\n");
		return -EINVAL;
	}

	ioctl = &virtio_vdmabuf_ioctls[nr];

	func = ioctl->func;

	if (unlikely(!func)) {
		dev_err(drv_info->dev, "no function\n");
		return -EINVAL;
	}

	kdata = kvmalloc(_IOC_SIZE(cmd), GFP_KERNEL);
	if (!kdata)
		return -ENOMEM;

	if (copy_from_user(kdata, (void __user *)param,
			   _IOC_SIZE(cmd)) != 0) {
		dev_err(drv_info->dev,
			"failed to copy from user arguments\n");
		ret = -EFAULT;
		goto ioctl_error;
	}

	ret = func(vdmabuf, kdata);

	if (copy_to_user((void __user *)param, kdata,
			 _IOC_SIZE(cmd)) != 0) {
		dev_err(drv_info->dev,
			"failed to copy to user arguments\n");
		ret = -EFAULT;
		goto ioctl_error;
	}

ioctl_error:
	kvfree(kdata);
	return ret;
}

static unsigned int virtio_vdmabuf_event_poll(struct file *filp,
					      struct poll_table_struct *wait)
{
	return 0;
}

static ssize_t virtio_vdmabuf_event_read(struct file *filp, char __user *buf,
					 size_t cnt, loff_t *ofst)
{
	return 0;
}

static int virtio_vdmabuf_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static const struct file_operations virtio_vdmabuf_fops = {
	.owner = THIS_MODULE,
	.open = virtio_vdmabuf_open,
	.release = virtio_vdmabuf_release,
	.read = virtio_vdmabuf_event_read,
	.poll = virtio_vdmabuf_event_poll,
	.unlocked_ioctl = virtio_vdmabuf_ioctl,
};

static struct miscdevice virtio_vdmabuf_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "virtio-vdmabuf",
	.fops = &virtio_vdmabuf_fops,
};

static int virtio_vdmabuf_probe(struct virtio_device *vdev)
{
	vq_callback_t *cbs[] = {
		virtio_vdmabuf_recv_cb,
		virtio_vdmabuf_send_cb,
	};
	static const char *const names[] = {
		"recv",
		"send",
	};
	struct virtio_vdmabuf *vdmabuf;
	struct virtqueue *vq;
	int ret = 0;

	if (!drv_info)
		return -EINVAL;

	vdmabuf = drv_info->priv;

	if (!vdmabuf)
		return -EINVAL;

	vdmabuf->vdev = vdev;
	vdev->priv = vdmabuf;

	/* initialize spinlock for synchronizing virtqueue accesses */
	spin_lock_init(&vdmabuf->vq_lock);

	/* initialize spinlock for synchronizing msg_list accesses */
	spin_lock_init(&vdmabuf->msg_list_lock);

	ret = virtio_find_vqs(vdmabuf->vdev, VDMABUF_VQ_MAX, vdmabuf->vqs,
			      cbs, names, NULL);
	if (ret) {
		dev_err(drv_info->dev, "Cannot find any vqs\n");
		return ret;
	}

	vq = vdmabuf->vqs[VDMABUF_VQ_RECV];
	ret = virtio_vdambuf_fill_queue(vq);
	if (!ret)
		goto vqs_del;

	INIT_LIST_HEAD(&vdmabuf->msg_list);
	INIT_WORK(&vdmabuf->recv_work, virtio_vdmabuf_recv_work);
	INIT_WORK(&vdmabuf->send_work, virtio_vdmabuf_send_work);
	INIT_WORK(&vdmabuf->send_msg_work, virtio_vdmabuf_send_msg_work);

	mutex_lock(&drv_info->g_mutex);
	drv_info->host_ready = true;
	mutex_unlock(&drv_info->g_mutex);

	dev_info(drv_info->dev, "virtio_vdmabuf: init successfully\n");

	return 0;

vqs_del:
	vdev->config->del_vqs(vdev);
	return ret;
}

static void virtio_vdmabuf_remove(struct virtio_device *vdev)
{
	struct virtio_vdmabuf *vdmabuf;

	if (!drv_info)
		return;

	mutex_lock(&drv_info->g_mutex);
	drv_info->host_ready = false;
	mutex_unlock(&drv_info->g_mutex);

	vdmabuf = drv_info->priv;
	flush_work(&vdmabuf->recv_work);
	flush_work(&vdmabuf->send_work);
	flush_work(&vdmabuf->send_msg_work);

	vdev->config->reset(vdev);
	vdev->config->del_vqs(vdev);
}

static struct virtio_device_id id_table[] = {
	{ VIRTIO_ID_VDMABUF, VIRTIO_DEV_ANY_ID },
	{ 0 },
};

static struct virtio_driver virtio_vdmabuf_vdev_drv = {
	.driver.name =  KBUILD_MODNAME,
	.driver.owner = THIS_MODULE,
	.id_table =     id_table,
	.probe =        virtio_vdmabuf_probe,
	.remove =       virtio_vdmabuf_remove,
};

static int __init virtio_vdmabuf_init(void)
{
	struct virtio_vdmabuf *vdmabuf;
	int ret = 0;

	drv_info = kvcalloc(1, sizeof(*drv_info), GFP_KERNEL);
	if (!drv_info)
		return -ENOMEM;

	vdmabuf = kvcalloc(1, sizeof(*vdmabuf), GFP_KERNEL);
	if (!vdmabuf) {
		ret =  -ENOMEM;
		goto free_2;
	}

	vdmabuf->eq_import = kvcalloc(1, sizeof(*vdmabuf->eq_import),
				      GFP_KERNEL);
	if (!vdmabuf->eq_import)
		goto free_1;

	drv_info->priv = (void *)vdmabuf;
	drv_info->host_ready = false;

	ret = carveout_buf_setup();
	if (ret < 0)
		pr_warn("virtio-vdmabuf: carveout bufs setup failed %d\n",
			 ret);

	mutex_init(&drv_info->g_mutex);

	ret = misc_register(&virtio_vdmabuf_miscdev);
	if (ret) {
		dev_err(drv_info->dev,
			"virtio-vdmabuf misc driver can't be registered\n");
		goto free_1;
	}
	dma_coerce_mask_and_coherent(virtio_vdmabuf_miscdev.this_device,
				     DMA_BIT_MASK(64));
	drv_info->dev = virtio_vdmabuf_miscdev.this_device;


	spin_lock_init(&vdmabuf->eq_import->e_lock);
	INIT_LIST_HEAD(&vdmabuf->eq_import->e_list);
	init_waitqueue_head(&vdmabuf->eq_import->e_wait);

	spin_lock_init(&drv_info->local_lock);
	hash_init(drv_info->buf_list_local);
	spin_lock_init(&drv_info->import_lock);
	hash_init(drv_info->buf_list_import);

	vdmabuf->wq = create_workqueue("virtio_vdmabuf_wq");

	ret = register_virtio_driver(&virtio_vdmabuf_vdev_drv);
	if (ret) {
		dev_err(drv_info->dev,
			"vdmabuf driver can't be registered\n");
		goto misc_dereg;
	}

	return 0;

misc_dereg:
	misc_deregister(&virtio_vdmabuf_miscdev);
	kvfree(vdmabuf->eq_import);
free_1:
	kvfree(vdmabuf);
free_2:
	kvfree(drv_info);
	return ret;
}

static void __exit virtio_vdmabuf_deinit(void)
{
	struct virtio_vdmabuf *vdmabuf = drv_info->priv;
	struct virtio_vdmabuf_event *event, *event_tmp;
	unsigned long irqflags;

	misc_deregister(&virtio_vdmabuf_miscdev);
	unregister_virtio_driver(&virtio_vdmabuf_vdev_drv);

	if (vdmabuf->wq)
		destroy_workqueue(vdmabuf->wq);

	spin_lock_irqsave(&vdmabuf->eq_import->e_lock, irqflags);

	list_for_each_entry_safe(event, event_tmp,
				 &vdmabuf->eq_import->e_list,
				 list) {
		list_del(&event->list);
		kfree(event);
	}

	spin_unlock_irqrestore(&vdmabuf->eq_import->e_lock, irqflags);

	/* freeing all exported buffers */
	virtio_vdmabuf_remove_all_bufs(vdmabuf);

	kvfree(vdmabuf->eq_import);
	kvfree(vdmabuf);
	kvfree(drv_info);
}

module_init(virtio_vdmabuf_init);
module_exit(virtio_vdmabuf_deinit);

MODULE_DESCRIPTION("Virtio Vdmabuf frontend driver");
MODULE_AUTHOR("Xianting Tian <xianting.tian@linux.alibaba.com>");
MODULE_LICENSE("GPL and additional rights");
