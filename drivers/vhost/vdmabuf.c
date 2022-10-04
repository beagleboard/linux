// SPDX-License-Identifier: (MIT OR GPL-2.0)
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/miscdevice.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/hashtable.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/dma-buf.h>
#include <linux/vhost.h>
#include <linux/vfio.h>
#include <linux/virtio_vdmabuf.h>
#include <linux/of_address.h>
#include <linux/of_fdt.h>
#include <linux/of.h>

enum {
	VHOST_VDMABUF_FEATURES = 1ULL << VIRTIO_F_VERSION_1,
};

static struct virtio_vdmabuf_info *drv_info;
static unsigned int vhost_vmid; /* current only support one guest */

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

static inline void vhost_vdmabuf_add(struct vhost_vdmabuf *new)
{
	list_add_tail(&new->list, &drv_info->head_vdmabuf_list);
}

static inline struct vhost_vdmabuf *vhost_vdmabuf_find(unsigned int vmid)
{
	struct vhost_vdmabuf *found;

	list_for_each_entry(found, &drv_info->head_vdmabuf_list, list)
		if (found->vmid == vmid)
			return found;

	return NULL;
}

static inline bool vhost_vdmabuf_del(struct vhost_vdmabuf *vdmabuf)
{
	struct vhost_vdmabuf *iter, *temp;

	list_for_each_entry_safe(iter, temp,
				 &drv_info->head_vdmabuf_list,
				 list)
		if (iter == vdmabuf) {
			list_del(&iter->list);
			return true;
		}

	return false;
}

static inline void vhost_vdmabuf_del_all(void)
{
	struct vhost_vdmabuf *iter, *temp;

	list_for_each_entry_safe(iter, temp,
				 &drv_info->head_vdmabuf_list,
				 list) {
		list_del(&iter->list);
		kfree(iter);
	}
}

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

static struct sg_table *vhost_vdmabuf_dmabuf_map(struct dma_buf_attachment *attachment,
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

static void vhost_vdmabuf_dmabuf_unmap(struct dma_buf_attachment *attachment,
				       struct sg_table *sgt,
				       enum dma_data_direction dir)
{
	struct virtio_vdmabuf_buf *exp_buf = attachment->dmabuf->priv;

	sg_table_unmap(attachment->dev, exp_buf, sgt, dir);
}

static int vhost_vdmabuf_dmabuf_mmap(struct dma_buf *dmabuf,
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

static void *vhost_vdmabuf_dmabuf_vmap(struct dma_buf *dmabuf)
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

static void vhost_vdmabuf_dmabuf_vunmap(struct dma_buf *dmabuf, void *vaddr)
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

static void vhost_vdmabuf_release_priv(struct virtio_vdmabuf_buf *exp_buf)
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

static int send_msg_to_guest(struct vhost_vdmabuf *vdmabuf,
			     enum virtio_vdmabuf_cmd cmd, void *data,
			     int bp_num);

static void vhost_vdmabuf_dmabuf_release(struct dma_buf *dma_buf)
{
	struct virtio_vdmabuf_buf *exp_buf = dma_buf->priv;
	struct vhost_vdmabuf *vdmabuf;
	unsigned long irqflags;
	unsigned int buf_id;
	int ret;

	if (!exp_buf)
		return;

	exp_buf->dma_buf = NULL;
	exp_buf->valid = false;
	buf_id = exp_buf->buf_id;
	vdmabuf = exp_buf->host;

	if (exp_buf->imported) {
		ret = send_msg_to_guest(vdmabuf, VIRTIO_VDMABUF_CMD_REL_NOTIFY,
					&buf_id, 0);
		if (ret < 0)
			dev_err(drv_info->dev,
				"failed(%d) to send dmabuf(%d) release cmd\n",
				ret, buf_id);
	} else {
		spin_lock_irqsave(&vdmabuf->local_lock, irqflags);
		ret = vhost_vdmabuf_del_buf(vdmabuf, buf_id, true);
		spin_unlock_irqrestore(&vdmabuf->local_lock, irqflags);
		if (ret)
			dev_err(drv_info->dev,
				"failed(%d) to del dmabuf(%d) from local list\n",
				ret, buf_id);

		vhost_vdmabuf_release_priv(exp_buf);
	}

	kfree(exp_buf);
}

static int vhost_vdmabuf_dmabuf_begin_cpu_access(struct dma_buf *dmabuf,
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

static int vhost_vdmabuf_dmabuf_end_cpu_access(struct dma_buf *dmabuf,
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

static int vhost_vdmabuf_dmabuf_attach(struct dma_buf *dmabuf,
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

static void vhost_vdmabuf_dmabuf_detach(struct dma_buf *dmabuf,
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

static const struct dma_buf_ops vhost_vdmabuf_dmabuf_ops = {
	.attach = vhost_vdmabuf_dmabuf_attach,
	.detach = vhost_vdmabuf_dmabuf_detach,
	.map_dma_buf = vhost_vdmabuf_dmabuf_map,
	.unmap_dma_buf = vhost_vdmabuf_dmabuf_unmap,
	.release = vhost_vdmabuf_dmabuf_release,
	.mmap = vhost_vdmabuf_dmabuf_mmap,
	.vmap = vhost_vdmabuf_dmabuf_vmap,
	.vunmap = vhost_vdmabuf_dmabuf_vunmap,
	.begin_cpu_access = vhost_vdmabuf_dmabuf_begin_cpu_access,
	.end_cpu_access = vhost_vdmabuf_dmabuf_end_cpu_access,
};

static int send_msg_to_guest(struct vhost_vdmabuf *vdmabuf,
		             enum virtio_vdmabuf_cmd cmd, void *data,
			     int bp_num)
{
	struct virtio_vdmabuf_msg *msg;
	struct virtio_vdmabuf_buf *exp_buf;
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
	case VIRTIO_VDMABUF_CMD_VMID_REPLY:
		msg->op[0] = vdmabuf->vmid;
		break;

	/* Host played dmabuf importer role */

	case VIRTIO_VDMABUF_CMD_IMPORT_REQ:
		buf_id = *(unsigned int *)data;
		msg->op[0] = vdmabuf->vmid;
		msg->op[1] = buf_id;

		break;

	case VIRTIO_VDMABUF_CMD_REL_NOTIFY:
		buf_id = *(unsigned int *)data;

		spin_lock_irqsave(&vdmabuf->import_lock, irqflags);
		ret = vhost_vdmabuf_del_buf(vdmabuf, buf_id, false);
		spin_unlock_irqrestore(&vdmabuf->import_lock, irqflags);
		if (ret)
			return ret;

		msg->op[0] = vdmabuf->vmid;
		msg->op[1] = buf_id;

		break;

	/* Host played dmabuf exporter role */

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

	list_add_tail(&msg->list, &vdmabuf->msg_list);
	vhost_work_queue(&vdmabuf->dev, &vdmabuf->send_work);

	return 0;
}

static void send_to_recvq(struct vhost_vdmabuf *vdmabuf,
			  struct vhost_virtqueue *vq)
{
	struct virtio_vdmabuf_msg *msg;
	int head, in, out, in_size;
	bool added = false;
	int ret, size;

	mutex_lock(&vq->mutex);

	if (!vhost_vq_get_backend(vq))
		goto out;

	vhost_disable_notify(&vdmabuf->dev, vq);

	for (;;) {
		if (list_empty(&vdmabuf->msg_list))
			break;

		head = vhost_get_vq_desc(vq, vq->iov, ARRAY_SIZE(vq->iov),
					 &out, &in, NULL, NULL);

		if (head < 0 || head == vq->num)
			break;

		in_size = iov_length(&vq->iov[out], in);

		if (in_size != vdmabuf_msg_size(VIRTIO_VDMABUF_MAX_BP_NUM)) {
			dev_err(drv_info->dev, "tx msg with wrong size %d\n",
				in_size);
			break;
		}

		msg = list_first_entry(&vdmabuf->msg_list,
				       struct virtio_vdmabuf_msg, list);

		dev_dbg(drv_info->dev, "send msg cmd %d\n", msg->cmd);

		list_del_init(&msg->list);

		size = vdmabuf_msg_size(msg->op[5]);
		ret = __copy_to_user(vq->iov[out].iov_base, msg, size);
		if (ret) {
			dev_err(drv_info->dev,
				"fail to copy tx msg\n");
			break;
		}

		vhost_add_used(vq, head, in_size);
		added = true;

		kfree(msg);
	}

	vhost_enable_notify(&vdmabuf->dev, vq);
	if (added)
		vhost_signal(&vdmabuf->dev, vq);
out:
	mutex_unlock(&vq->mutex);
}

static void vhost_send_msg_work(struct vhost_work *work)
{
	struct vhost_vdmabuf *vdmabuf = container_of(work,
						     struct vhost_vdmabuf,
						     send_work);
	struct vhost_virtqueue *vq = &vdmabuf->vqs[VDMABUF_VQ_RECV];

	send_to_recvq(vdmabuf, vq);
}

/* parse incoming message from a guest */
static int parse_msg_from_guest(struct vhost_vdmabuf *vdmabuf,
		     struct virtio_vdmabuf_msg *msg)
{
	struct virtio_vdmabuf_event *event;
	struct virtio_vdmabuf_buf *exp_buf;
	unsigned long irqflags;
	unsigned int buf_id;
	unsigned vmid;
	int bp_num;
	int i, ret = 0;

	dev_dbg(drv_info->dev, "received msg cmd %d\n", msg->cmd);

	switch (msg->cmd) {
	case VIRTIO_VDMABUF_CMD_VMID_REQ:
		send_msg_to_guest(vdmabuf, VIRTIO_VDMABUF_CMD_VMID_REPLY, NULL, 0);
		break;

	/* Host played dmabuf exporter role */

	case VIRTIO_VDMABUF_CMD_IMPORT_REQ:
		vmid = msg->op[0];
		if (vdmabuf->vmid != vmid) {
			dev_err(drv_info->dev, "vmid does not match %d %d\n",
				vdmabuf->vmid, vmid);
			return -EINVAL;
		}

		buf_id = msg->op[1];

		spin_lock_irqsave(&vdmabuf->local_lock, irqflags);
		exp_buf = vhost_vdmabuf_find_buf(vdmabuf, buf_id, true);
		if (!exp_buf) {
			spin_unlock_irqrestore(&vdmabuf->local_lock, irqflags);
			dev_err(drv_info->dev, "no exp_buf found for buf id %d\n",
				buf_id);

			return -ENOENT;
		}
		spin_unlock_irqrestore(&vdmabuf->local_lock, irqflags);

		send_msg_to_guest(vdmabuf, VIRTIO_VDMABUF_CMD_IMPORT_REPLY,
				  exp_buf, exp_buf->bp_num);

		get_dma_buf(exp_buf->dma_buf);

		break;

	case VIRTIO_VDMABUF_CMD_REL_NOTIFY:
		vmid = msg->op[0];
		if (vdmabuf->vmid != vmid)
			return -EINVAL;

		buf_id = msg->op[1];

		spin_lock_irqsave(&vdmabuf->local_lock, irqflags);
		exp_buf = vhost_vdmabuf_find_buf(vdmabuf, buf_id, true);
		if (!exp_buf) {
			spin_unlock_irqrestore(&vdmabuf->local_lock, irqflags);
			dev_err(drv_info->dev, "can't find buffer\n");
			return -ENOENT;
		}

		dma_buf_put(exp_buf->dma_buf);
		spin_unlock_irqrestore(&vdmabuf->local_lock, irqflags);

		break;

	/* Host played dmabuf importer role */

	case VIRTIO_VDMABUF_CMD_IMPORT_REPLY:
		vmid = msg->op[0];
		buf_id = msg->op[1];
		bp_num = msg->op[5];

		if (vdmabuf->vmid != vmid) {
			dev_err(drv_info->dev, "vmid do not match %d %d\n",
				vdmabuf->vmid, vmid);
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
			 * guest is invalid at host side
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
		ret = -EINVAL;
		break;
	}

	return ret;
}

static void vhost_vdmabuf_handle_send_kick(struct vhost_work *work)
{
	struct vhost_virtqueue *vq = container_of(work,
						  struct vhost_virtqueue,
						  poll.work);
	struct vhost_vdmabuf *vdmabuf = container_of(vq->dev,
						     struct vhost_vdmabuf,
						     dev);
	struct virtio_vdmabuf_msg *msg;
	int head, in, out, in_size;
	bool added = false;
	int ret;

	mutex_lock(&vq->mutex);

	if (!vhost_vq_get_backend(vq))
		goto out;

	vhost_disable_notify(&vdmabuf->dev, vq);

	msg = kzalloc(struct_size(msg, bp, VIRTIO_VDMABUF_MAX_BP_NUM),
				  GFP_KERNEL);
	if (!msg) {
		dev_err(drv_info->dev, "kzalloc failed\n");
		return;
	}

	/* Make sure we will process all pending requests */
	for (;;) {
		head = vhost_get_vq_desc(vq, vq->iov, ARRAY_SIZE(vq->iov),
					 &out, &in, NULL, NULL);

		if (head < 0 || head == vq->num)
			break;

		in_size = iov_length(&vq->iov[in], out);
		/*
		 * As the size of the msg from guest doesn't have a certain
		 * value, it is hard to do the check, so remove it.
		if (in_size != vdmabuf_msg_size(VIRTIO_VDMABUF_MAX_BP_NUM)) {
			dev_err(drv_info->dev, "rx msg with wrong size %d\n",
				in_size);
			break;
		}*/

		if (__copy_from_user(msg, vq->iov[in].iov_base, in_size)) {
			dev_err(drv_info->dev,
				"err: can't get the msg from vq\n");
			break;
		}

		ret = parse_msg_from_guest(vdmabuf, msg);
		if (ret) {
			dev_err(drv_info->dev, "msg parse error %d", ret);

			break;
		}

		vhost_add_used(vq, head, in_size);
		added = true;
	}

	kfree(msg);

	vhost_enable_notify(&vdmabuf->dev, vq);
	if (added)
		vhost_signal(&vdmabuf->dev, vq);
out:
	mutex_unlock(&vq->mutex);
}

static void vhost_vdmabuf_handle_recv_kick(struct vhost_work *work)
{
	struct vhost_virtqueue *vq = container_of(work,
						  struct vhost_virtqueue,
						  poll.work);
	struct vhost_vdmabuf *vdmabuf = container_of(vq->dev,
						     struct vhost_vdmabuf,
						     dev);

	send_to_recvq(vdmabuf, vq);
}

static int vhost_vdmabuf_open(struct inode *inode, struct file *filp)
{
	struct vhost_vdmabuf *vdmabuf;
	struct vhost_virtqueue **vqs;
	int ret = 0;

	if (!drv_info) {
		dev_err(drv_info->dev,
			"vhost-vdmabuf: can't open misc device\n");
		return -EINVAL;
	}

	/* each vdmabuf on behave of one guest */
	vdmabuf = kzalloc(sizeof(*vdmabuf), GFP_KERNEL |
			   __GFP_RETRY_MAYFAIL);
	if (!vdmabuf)
		return -ENOMEM;

	vqs = kmalloc_array(ARRAY_SIZE(vdmabuf->vqs), sizeof(*vqs),
			    GFP_KERNEL);
	if (!vqs) {
		kfree(vdmabuf);
		return -ENOMEM;
	}

	vdmabuf->eq_import = kcalloc(1, sizeof(*(vdmabuf->eq_import)), GFP_KERNEL);
	if (!vdmabuf->eq_import) {
		kfree(vdmabuf);
		kfree(vqs);
		return -ENOMEM;
	}

	vqs[VDMABUF_VQ_SEND] = &vdmabuf->vqs[VDMABUF_VQ_SEND];
	vqs[VDMABUF_VQ_RECV] = &vdmabuf->vqs[VDMABUF_VQ_RECV];
	vdmabuf->vqs[VDMABUF_VQ_SEND].handle_kick = vhost_vdmabuf_handle_send_kick;
	vdmabuf->vqs[VDMABUF_VQ_RECV].handle_kick = vhost_vdmabuf_handle_recv_kick;

	vhost_dev_init(&vdmabuf->dev, vqs, ARRAY_SIZE(vdmabuf->vqs),
		       UIO_MAXIOV, 0, 0, true, NULL);

	INIT_LIST_HEAD(&vdmabuf->msg_list);
	vhost_work_init(&vdmabuf->send_work, vhost_send_msg_work);
	vdmabuf->vmid = task_pid_nr(current);

	/* init guest's local and import list */
	spin_lock_init(&vdmabuf->local_lock);
	hash_init(vdmabuf->buf_list_local);
	spin_lock_init(&vdmabuf->import_lock);
	hash_init(vdmabuf->buf_list_import);

	/* add vdmabuf to list as we may have multiple guests */
	vhost_vdmabuf_add(vdmabuf);

	vhost_vmid = vdmabuf->vmid;

	mutex_init(&vdmabuf->eq_import->e_readlock);
	spin_lock_init(&vdmabuf->eq_import->e_lock);

	/* Initialize event queue */
	INIT_LIST_HEAD(&vdmabuf->eq_import->e_list);
	init_waitqueue_head(&vdmabuf->eq_import->e_wait);

	filp->private_data = vdmabuf;

	return ret;
}

static void vhost_vdmabuf_flush(struct vhost_vdmabuf *vdmabuf)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(vdmabuf->vqs); i++)
		if (vdmabuf->vqs[i].handle_kick)
			vhost_poll_flush(&vdmabuf->vqs[i].poll);

	vhost_work_flush(&vdmabuf->dev, &vdmabuf->send_work);
}

static int vhost_vdmabuf_release(struct inode *inode, struct file *filp)
{
	struct vhost_vdmabuf *vdmabuf = filp->private_data;
	struct virtio_vdmabuf_event *event, *event_tmp;

	if (!vhost_vdmabuf_del(vdmabuf))
		return -EINVAL;

	mutex_lock(&drv_info->g_mutex);

	list_for_each_entry_safe(event, event_tmp,
				 &vdmabuf->eq_import->e_list,
				 list) {
		list_del(&event->list);
		kfree(event);
	}

	vhost_vdmabuf_flush(vdmabuf);
	vhost_dev_cleanup(&vdmabuf->dev);

	kfree(vdmabuf->eq_import);
	kfree(vdmabuf->dev.vqs);
	kfree(vdmabuf);

	filp->private_data = NULL;
	mutex_unlock(&drv_info->g_mutex);

	return 0;
}

static unsigned int vhost_vdmabuf_event_poll(struct file *filp,
					     struct poll_table_struct *wait)
{
	return 0;
}

static ssize_t vhost_vdmabuf_event_read(struct file *filp, char __user *buf,
					size_t cnt, loff_t *ofst)
{
	return 0;
}

static int vhost_vdmabuf_start(struct vhost_vdmabuf *vdmabuf)
{
        struct vhost_virtqueue *vq;
        int i, ret;

	mutex_lock(&vdmabuf->dev.mutex);

        ret = vhost_dev_check_owner(&vdmabuf->dev);
        if (ret)
                goto err;

	for (i = 0; i < ARRAY_SIZE(vdmabuf->vqs); i++) {
		vq = &vdmabuf->vqs[i];

		mutex_lock(&vq->mutex);

		if (!vhost_vq_access_ok(vq)) {
			ret = -EFAULT;
			goto err_vq;
		}

		if (!vhost_vq_get_backend(vq)) {
			vhost_vq_set_backend(vq, vdmabuf);
			ret = vhost_vq_init_access(vq);
			if (ret)
				goto err_vq;
		}

		mutex_unlock(&vq->mutex);
	}

	mutex_unlock(&vdmabuf->dev.mutex);
        return 0;

err_vq:
        vhost_vq_set_backend(vq, NULL);
        mutex_unlock(&vq->mutex);

	for (i = 0; i < ARRAY_SIZE(vdmabuf->vqs); i++) {
		vq = &vdmabuf->vqs[i];

		mutex_lock(&vq->mutex);
		vhost_vq_set_backend(vq, NULL);
		mutex_unlock(&vq->mutex);
	}

err:
	mutex_unlock(&vdmabuf->dev.mutex);
        return ret;
}

static int vhost_vdmabuf_stop(struct vhost_vdmabuf *vdmabuf)
{
        struct vhost_virtqueue *vq;
        int i, ret;

	mutex_lock(&vdmabuf->dev.mutex);

        ret = vhost_dev_check_owner(&vdmabuf->dev);
        if (ret)
                goto err;

	for (i = 0; i < ARRAY_SIZE(vdmabuf->vqs); i++) {
		vq = &vdmabuf->vqs[i];

		mutex_lock(&vq->mutex);
		vhost_vq_set_backend(vq, NULL);
		mutex_unlock(&vq->mutex);
	}

err:
	mutex_unlock(&vdmabuf->dev.mutex);
        return ret;
}

static int vhost_vdmabuf_set_features(struct vhost_vdmabuf *vdmabuf,
				      u64 features)
{
	struct vhost_virtqueue *vq;
	int i;

	if (features & ~VHOST_VDMABUF_FEATURES)
		return -EOPNOTSUPP;

	mutex_lock(&vdmabuf->dev.mutex);
	if ((features & (1 << VHOST_F_LOG_ALL)) &&
	    !vhost_log_access_ok(&vdmabuf->dev)) {
		mutex_unlock(&vdmabuf->dev.mutex);
		return -EFAULT;
	}

	for (i = 0; i < ARRAY_SIZE(vdmabuf->vqs); i++) {
		vq = &vdmabuf->vqs[i];
		mutex_lock(&vq->mutex);
		vq->acked_features = features;
		mutex_unlock(&vq->mutex);
	}

	mutex_unlock(&vdmabuf->dev.mutex);
	return 0;
}

/* wrapper ioctl for vhost interface control */
static int vhost_core_ioctl(struct file *filp, unsigned int cmd,
			    unsigned long param)
{
	struct vhost_vdmabuf *vdmabuf = filp->private_data;
	void __user *argp = (void __user *)param;
	u64 features;
	int ret, start;

	switch (cmd) {
	case VHOST_GET_FEATURES:
		features = VHOST_VDMABUF_FEATURES;
		if (copy_to_user(argp, &features, sizeof(features)))
			return -EFAULT;
		return 0;
	case VHOST_SET_FEATURES:
		if (copy_from_user(&features, argp, sizeof(features)))
			return -EFAULT;
		return vhost_vdmabuf_set_features(vdmabuf, features);
	case VHOST_VDMABUF_SET_RUNNING:
		if (copy_from_user(&start, argp, sizeof(start)))
                        return -EFAULT;

		if (start)
			return vhost_vdmabuf_start(vdmabuf);
                else
                        return vhost_vdmabuf_stop(vdmabuf);
	default:
		mutex_lock(&vdmabuf->dev.mutex);
		ret = vhost_dev_ioctl(&vdmabuf->dev, cmd, argp);
		if (ret == -ENOIOCTLCMD) {
			ret = vhost_vring_ioctl(&vdmabuf->dev, cmd, argp);
		} else {
			vhost_vdmabuf_flush(vdmabuf);
		}
		mutex_unlock(&vdmabuf->dev.mutex);
	}

	return ret;
}

static int vhost_vdmabuf_create_dmabuf(struct vhost_vdmabuf *vdmabuf,
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
	int i = 0, ret, npages, bp_num;

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

	exp_info.ops = &vhost_vdmabuf_dmabuf_ops;
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
	exp_buf->host = vdmabuf;
	exp_buf->valid = true;
	exp_buf->imported = false;

	attr->buf_id = exp_buf->buf_id;

	spin_lock_irqsave(&vdmabuf->local_lock, irqflags);
	vhost_vdmabuf_add_buf(vdmabuf, exp_buf, true);
	spin_unlock_irqrestore(&vdmabuf->local_lock, irqflags);

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

static int vhost_vdmabuf_create_mirror_dmabuf(struct vhost_vdmabuf *vdmabuf,
					      struct virtio_vdmabuf_import *attr,
					      struct virtio_vdmabuf_event *event)
{
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
	if (attr->buf_id != buf_id) {
		dev_err(drv_info->dev, "buf id does not match %d %d\n",
			attr->buf_id, buf_id);
		return -EINVAL;
	}

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
		for (i = 0; i < exp_buf->bp_num; i++) {
			/* as can access guest's page, we can directly convert */
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

	exp_info.ops = &vhost_vdmabuf_dmabuf_ops;
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
	exp_buf->host = vdmabuf;

	/*
	 * it indicates this is a mirrored dmabuf,
	 * we need send release cmd to host when
	 * this dambuf is released.
	 */
	exp_buf->imported = true;
	exp_buf->valid = true;

	spin_lock_irqsave(&vdmabuf->import_lock, irqflags);
	vhost_vdmabuf_add_buf(vdmabuf, exp_buf, false);
	spin_unlock_irqrestore(&vdmabuf->import_lock, irqflags);

	return 0;

err_exp:
	kfree(exp_buf);
	return ret;
}

static int alloc_ioctl(struct vhost_vdmabuf *vdmabuf, void *data)
{
	struct virtio_vdmabuf_alloc *attr = data;
	int ret;

	mutex_lock(&vdmabuf->dev.mutex);
	ret = vhost_vdmabuf_create_dmabuf(vdmabuf, attr);
	mutex_unlock(&vdmabuf->dev.mutex);

	return ret;
}

static int import_ioctl(struct vhost_vdmabuf *vdmabuf, void *data)
{
	struct virtio_vdmabuf_import *attr = data;
	struct virtio_vdmabuf_buf *imp_buf;
	struct virtio_vdmabuf_event *event;
	unsigned long irqflags;
	int ret = 0;

	spin_lock_irqsave(&vdmabuf->import_lock, irqflags);
	imp_buf = vhost_vdmabuf_find_buf(vdmabuf, attr->buf_id, false);
	if (imp_buf && imp_buf->valid) {
		ret = dma_buf_fd(imp_buf->dma_buf, 0);
		if (ret < 0) {
			dma_buf_put(imp_buf->dma_buf);
			spin_unlock_irqrestore(&vdmabuf->import_lock, irqflags);
			return ret;
		}

		attr->fd = ret;
		attr->size = imp_buf->size;
		spin_unlock_irqrestore(&vdmabuf->import_lock, irqflags);

		return 0;
	}
	spin_unlock_irqrestore(&vdmabuf->import_lock, irqflags);

	/*
	 * We can't find the dambuf with buf_id in local hash list,
	 * need send import request to host to get it.
	 */

	mutex_lock(&vdmabuf->dev.mutex);

	send_msg_to_guest(vdmabuf, VIRTIO_VDMABUF_CMD_IMPORT_REQ, &attr->buf_id, 0);

	/* host's ack of import request will wakeup us */
	if (wait_event_interruptible(vdmabuf->eq_import->e_wait,
				     !list_empty(&vdmabuf->eq_import->e_list))) {
		mutex_unlock(&vdmabuf->dev.mutex);
		dev_err(drv_info->dev, "OMG, this err is not expected\n");
		return -ERESTARTSYS;
	}

	spin_lock_irqsave(&vdmabuf->eq_import->e_lock, irqflags);
	event = list_first_entry(&vdmabuf->eq_import->e_list,
				 struct virtio_vdmabuf_event, list);
	/* safely del the event from list and free it */
	list_del(&event->list);
	spin_unlock_irqrestore(&vdmabuf->eq_import->e_lock, irqflags);

	/* create local mirror dmabuf */
	ret = vhost_vdmabuf_create_mirror_dmabuf(vdmabuf, attr, event);
	if (ret)
		dev_err(drv_info->dev, "create mirror dmabuf failed %d\n",
			ret);

	kfree(event);

	mutex_unlock(&vdmabuf->dev.mutex);
	return ret;
}

static const struct vhost_vdmabuf_ioctl_desc vhost_vdmabuf_ioctls[] = {
	VIRTIO_VDMABUF_IOCTL_DEF(VIRTIO_VDMABUF_IOCTL_ALLOC_FD, alloc_ioctl, 0),
	VIRTIO_VDMABUF_IOCTL_DEF(VIRTIO_VDMABUF_IOCTL_IMPORT_FD, import_ioctl, 0),
};

static long vhost_vdmabuf_ioctl(struct file *filp, unsigned int cmd,
				unsigned long param)
{
	int ret = -EINVAL;

	/* check if cmd is vhost's */
	if (_IOC_TYPE(cmd) == VHOST_VIRTIO)
		ret = vhost_core_ioctl(filp, cmd, param);

	return ret;
}

static const struct file_operations vhost_vdmabuf_fops = {
	.owner = THIS_MODULE,
	.open = vhost_vdmabuf_open,
	.release = vhost_vdmabuf_release,
	.read = vhost_vdmabuf_event_read,
	.poll = vhost_vdmabuf_event_poll,
	.unlocked_ioctl = vhost_vdmabuf_ioctl,
};

/*
 * vhost-vdmabuf dev is used by hypervisor, eg kvmtool,
 * to setup this vhost backend.
 */
static struct miscdevice vhost_vdmabuf_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "vhost-vdmabuf",
	.fops = &vhost_vdmabuf_fops,
};

static int vhost_vdmabuf_user_open(struct inode *inode, struct file *filp)
{
	struct vhost_vdmabuf *vdmabuf;

	vdmabuf = vhost_vdmabuf_find(vhost_vmid);
	if (!vdmabuf) {
		dev_warn(drv_info->dev,
			 "vhost-vdmabuf: no vdmabuf with vmid %d found\n",
			 vhost_vmid);
		return -EINVAL;
	}

	filp->private_data = vdmabuf;

	return 0;
}

static int vhost_vdmabuf_user_release(struct inode *inode, struct file *filp)
{
	mutex_lock(&drv_info->g_mutex);
	filp->private_data = NULL;
	mutex_unlock(&drv_info->g_mutex);

	return 0;
}

static long vhost_vdmabuf_user_ioctl(struct file *filp, unsigned int cmd,
				     unsigned long param)
{
	struct vhost_vdmabuf *vdmabuf = filp->private_data;
	const struct vhost_vdmabuf_ioctl_desc *ioctl;
	vhost_vdmabuf_ioctl_t func;
	unsigned int nr;
	int ret;
	char *kdata;

	nr = _IOC_NR(cmd);

	if (nr >= ARRAY_SIZE(vhost_vdmabuf_ioctls)) {
		dev_err(drv_info->dev, "invalid ioctl\n");
		return -EINVAL;
	}

	ioctl = &vhost_vdmabuf_ioctls[nr];

	func = ioctl->func;

	if (unlikely(!func)) {
		dev_err(drv_info->dev, "no function\n");
		return -EINVAL;
	}

	kdata = kmalloc(_IOC_SIZE(cmd), GFP_KERNEL);
	if (!kdata)
		return -ENOMEM;

	if (copy_from_user(kdata, (void __user *)param,
			   _IOC_SIZE(cmd)) != 0) {
		dev_err(drv_info->dev,
			"failed to copy args from userspace\n");
		ret = -EFAULT;
		goto ioctl_error;
	}

	ret = func(vdmabuf, kdata);

	if (copy_to_user((void __user *)param, kdata,
			 _IOC_SIZE(cmd)) != 0) {
		dev_err(drv_info->dev,
			"failed to copy args back to userspace\n");
		ret = -EFAULT;
		goto ioctl_error;
	}

ioctl_error:
	kfree(kdata);
	return ret;
}

static const struct file_operations vhost_vdmabuf_user_fops = {
	.owner = THIS_MODULE,
	.open = vhost_vdmabuf_user_open,
	.release = vhost_vdmabuf_user_release,
	.unlocked_ioctl = vhost_vdmabuf_user_ioctl,
};

/*
 * vhost-user-vdmabuf dev is used by dmabuf user to alloc or
 * get dambuf via buf id.
 */
static struct miscdevice vhost_vdmabuf_user_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "vhost-user-vdmabuf",
	.fops = &vhost_vdmabuf_user_fops,
};

static int __init vhost_vdmabuf_init(void)
{
	int ret = 0;

	/* register vhost setup dev */
	ret = misc_register(&vhost_vdmabuf_miscdev);
	if (ret) {
		dev_err(drv_info->dev,
			 "vhost-vdmabuf: driver can't be registered\n");
		return ret;
	}

	dma_coerce_mask_and_coherent(vhost_vdmabuf_miscdev.this_device,
				     DMA_BIT_MASK(64));

	drv_info = kcalloc(1, sizeof(*drv_info), GFP_KERNEL);
	if (!drv_info) {
		misc_deregister(&vhost_vdmabuf_miscdev);
		return -ENOMEM;
	}

	drv_info->dev = vhost_vdmabuf_miscdev.this_device;

	/* register dmabuf alloc & get dev for user */
	ret = misc_register(&vhost_vdmabuf_user_miscdev);
	if (ret) {
		misc_deregister(&vhost_vdmabuf_miscdev);
		dev_err(drv_info->dev,
			"vhost-vdmabuf: driver can't be registered\n");
		return ret;
	}

	ret = carveout_buf_setup();
	if (ret < 0)
		dev_warn(drv_info->dev,
			 "vhost-vdmabuf: carveout bufs setup failed %d\n",
			 ret);

	mutex_init(&drv_info->g_mutex);

	INIT_LIST_HEAD(&drv_info->head_vdmabuf_list);

	dev_info(drv_info->dev, "vhost-vdmabuf: init successfully\n");

	return 0;
}

static void __exit vhost_vdmabuf_deinit(void)
{
	misc_deregister(&vhost_vdmabuf_miscdev);
	misc_deregister(&vhost_vdmabuf_user_miscdev);
	vhost_vdmabuf_del_all();

	kfree(drv_info);
	drv_info = NULL;
}

module_init(vhost_vdmabuf_init);
module_exit(vhost_vdmabuf_deinit);

MODULE_DESCRIPTION("Vhost Vdmabuf Driver");
MODULE_AUTHOR("Xianting Tian <xianting.tian@linux.alibaba.com>");
MODULE_LICENSE("GPL and additional rights");
