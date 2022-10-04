// SPDX-License-Identifier: GPL-2.0-only
#include <linux/virtio_blk.h>
#include <linux/blkdev.h>
#include <linux/bio.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include "../light_vringh.h"
#include "light_blk.h"

enum {
	LIGHT_VBLK_VQ_REQ = 0,
	LIGHT_VBLK_VQ_MAX = 1,
};

static DEFINE_MUTEX(vdev_list_lock);
static LIST_HEAD(vdev_list);

static DEFINE_SPINLOCK(vblk_io_complete_lock);

struct vblk_bio_info {
	struct light_vdev *vdev;
	struct kvec kiov;
	u16 avail_head;
	u8 vring_idx;
};

extern void kvm_arch_notify_guest(void);

static void light_vblk_notify(struct light_vdev *vdev)
{
	kvm_arch_notify_guest();
}

static int light_vblk_map_vring(struct light_vdev *vdev,
				unsigned int num,
				struct vhost_vring_addr *a)
{
	int ret = -EINVAL;
	int vr_size;
	phys_addr_t phys_addr;
	struct light_vringh *vvr;
	struct light_vring *vr;
	u32 vq_index = a->index;
	u32 vq_size = num;
	u32 vq_align = PAGE_SIZE;
	u32 vq_pfn = a->desc_user_addr;

	if (num <= 0 || vq_index >= LIGHT_MAX_VRINGS)
		goto err;

	vvr = &vdev->vvr[vq_index];
	vr = &vdev->vvr[vq_index].vring;

	vr_size = PAGE_ALIGN(round_up(vring_size(vq_size, vq_align), 4));

	/* init vring */
	phys_addr = (phys_addr_t)vq_pfn << PAGE_SHIFT;
	vr->va = (void *) phys_to_virt(phys_addr);
	pr_debug("Blk backend: vring(%d) virtual addr %lx, phys addr %llx\n",
		 vq_index, (unsigned long)vr->va, phys_addr);

	vr->len = vr_size;

	vring_init(&vr->vr, vq_size, vr->va, vq_align);
	ret = vringh_init_kern(&vvr->vrh,
			       0, /* don't need features  */
			       vq_size, false,
			       vr->vr.desc, vr->vr.avail,
			       vr->vr.used);
	if (ret) {
		pr_err("%s %d err %d\n", __func__, __LINE__, ret);
		goto err;
	}

	vringh_kiov_init(&vvr->riov, NULL, 0);
	vringh_kiov_init(&vvr->wiov, NULL, 0);
	vvr->vdev = vdev;

	vdev->vq_num++;

	pr_info("Blk backend: added vq %d with size %d\n",
			vq_index, vq_size);

	return 0;
err:
	return ret;
}

static void light_vblk_io_handle(struct light_vdev *, u32);

static blk_status_t light_vblk_check_hdr_format(struct light_vdev *vdev,
						int vr_idx)
{
	struct vring_desc *desc;
	struct light_vring *vring;
	struct vringh *vrh;
	u32 desc_idx, avail_idx;

	vring = &vdev->vvr[vr_idx].vring;
	vrh = &vdev->vvr[vr_idx].vrh;
	avail_idx = vrh->last_avail_idx & (vring->vr.num - 1);
	desc_idx = vring->vr.avail->ring[avail_idx];
	desc = &vring->vr.desc[desc_idx];

	if (desc->len != sizeof(struct virtio_blk_outhdr)) {
		pr_err("blk header error: size %d\n", desc->len);
		return BLK_STS_IOERR;
	}

	if (!(desc->flags & VRING_DESC_F_NEXT)) {
		pr_err("blk header error: next\n");
		return BLK_STS_IOERR;
	}

	if (desc->flags & VRING_DESC_F_WRITE) {
		pr_err("blk header error: write\n");
		return BLK_STS_IOERR;
	}

	return BLK_STS_OK;
}

static void get_device_id(struct kvec *kiov)
{
	unsigned long vaddr;
	char id_buf[VIRTIO_BLK_ID_BYTES] = "\x12\x34\x56\x78";

	vaddr = (unsigned long)phys_to_virt((unsigned long)kiov->iov_base);
	memcpy((void *)vaddr, (void *)id_buf, VIRTIO_BLK_ID_BYTES);
}

static blk_status_t light_vblk_issue_discard(struct light_vdev *vdev,
					     struct kvec *kiov)
{
	int ret;
	unsigned long vaddr;
	struct virtio_blk_discard_write_zeroes *range;

	/*
	 * As now we set max_discard_segments = 1 in frontend,
	 * so we only have one range.
	 */
	vaddr = (unsigned long)phys_to_virt((unsigned long)kiov->iov_base);
	range = (struct virtio_blk_discard_write_zeroes *)vaddr;

	ret = blkdev_issue_discard(vdev->this_bdev, range->sector,
				   range->num_sectors,
				   GFP_KERNEL, range->flags);
	if (ret == -EOPNOTSUPP)
		return BLK_STS_NOTSUPP;
	return ret != 0 ? BLK_STS_IOERR : BLK_STS_OK;
}

static blk_status_t light_vblk_issue_zeroout(struct light_vdev *vdev,
					     struct kvec *kiov)
{
	int ret;
	unsigned long vaddr;
	struct virtio_blk_discard_write_zeroes *range;

	/*
	 * As now we set max_discard_segments = 1 in frontend,
	 * so we only have one range.
	 */
	vaddr = (unsigned long)phys_to_virt((unsigned long)kiov->iov_base);
	range = (struct virtio_blk_discard_write_zeroes *)vaddr;

	ret = blkdev_issue_zeroout(vdev->this_bdev, range->sector,
				   range->num_sectors,
				   GFP_KERNEL, range->flags);
	if (ret == -EOPNOTSUPP)
		return BLK_STS_NOTSUPP;
	return ret != 0 ? BLK_STS_IOERR : BLK_STS_OK;
}

static inline void light_vblk_get_hdr(struct kvec *kiov,
				      struct virtio_blk_outhdr *hdr)
{
	unsigned long vaddr;
	size_t len;

	len = min(kiov->iov_len, sizeof(struct virtio_blk_outhdr));

	vaddr = (unsigned long)phys_to_virt((unsigned long)kiov->iov_base);
	memcpy((void *)hdr, (void *)vaddr, len);
}

static int light_vblk_set_backend(struct light_vdev *vdev,
				  unsigned int index, int fd)
{
	struct file *file;
	struct inode *inode;
	int ret;

	ret = vhost_dev_check_owner(&vdev->vhost_dev);
	if (ret)
		goto err;

	if (index >= LIGHT_VBLK_VQ_MAX) {
		ret = -ENOBUFS;
		goto err;
	}

	file = fget(fd);
	if (IS_ERR(file)) {
		ret = PTR_ERR(file);
		goto err;
	}

	inode = file->f_mapping->host;
	if (!S_ISBLK(inode->i_mode)) {
		ret = -EFAULT;
		goto err;
	}

	vdev->this_bdev = inode->i_bdev;

err:
	return ret;
}

static bool light_vblk_ring_avail(struct light_vdev *vdev, int vr_idx)
{
	struct light_vring *vring;
	struct vringh *vrh;

	vring = &vdev->vvr[vr_idx].vring;
	vrh = &vdev->vvr[vr_idx].vrh;

	/*
	 * vrh->last_avail_idx will be +1 after fetch a new avail idx
	 * in vringh_getdesc_kern().
	 */
	return vrh->last_avail_idx != vring->vr.avail->idx;
}

static void light_vblk_disable_notify(struct light_vdev *vdev, int vr_idx)
{
	struct light_vring *vring;

	vring = &vdev->vvr[vr_idx].vring;

	vring->vr.used->flags |= VRING_USED_F_NO_NOTIFY;
}

static void light_vblk_enable_notify(struct light_vdev *vdev, int vr_idx)
{
	struct light_vring *vring;

	vring = &vdev->vvr[vr_idx].vring;

	vring->vr.used->flags &= ~VRING_USED_F_NO_NOTIFY;
}

static void light_vblk_handle_guest_kick(struct vhost_work *work)
{
	struct vhost_virtqueue *vq;
	struct light_vdev *vdev;
	u32 qid;

	vq = container_of(work, struct vhost_virtqueue, poll.work);
	vdev = container_of(vq->dev, struct light_vdev, vhost_dev);

	for (qid = 0; qid < vdev->vq_num; qid++) {
		/*
		 * disable frontend notify us if we need handle
		 * io requests, then poll.
		 */
		if (light_vblk_ring_avail(vdev, qid))
			light_vblk_disable_notify(vdev, qid);

		while (light_vblk_ring_avail(vdev, qid))
			/*
			 * main vblk io handler, which contains 4
			 * parts:
			 * 1, check hdr format
			 * 2, get hdr
			 * 3, handle payload io
			 * 4, write io done status
			 */
			light_vblk_io_handle(vdev, qid);

		/* enable frontend notify us */
		light_vblk_enable_notify(vdev, qid);
	}
}

static void light_vblk_endio(struct bio *bio)
{
	size_t total_len;
	unsigned long vaddr;
	struct light_vringh *vvr;
	struct vringh *vrh;
	struct kvec *kiov;
	struct vblk_bio_info *bio_info;
	u16 avail_head;
	u32 vr_idx;
	struct light_vdev *vdev;

	bio_info = (struct vblk_bio_info *)bio->bi_private;
	kiov = &bio_info->kiov;
	avail_head = bio_info->avail_head;
	vr_idx = bio_info->vring_idx;
	vvr = &bio_info->vdev->vvr[vr_idx];
	vrh = &vvr->vrh;
	vdev = vvr->vdev;

	/*
	 * one single vblk io length contains 3 parts:
	 * hdr, payload, status
	 */
	total_len = sizeof(struct virtio_blk_outhdr) +
			   bio->bi_iter.bi_size +
			   1; /* 1 byte is for vblk io status */
	vaddr = (unsigned long)phys_to_virt((unsigned long)kiov->iov_base);
	*(blk_status_t *)vaddr = bio->bi_status;

	vringh_complete_kern(vrh, avail_head, total_len);
	if (vringh_need_notify_kern(vrh) > 0)
		vdev->notify(vdev);

	bio->bi_private = NULL;
	bio->bi_end_io = NULL;
	bio_put(bio);
	kfree(bio_info);
}

static void light_vblk_io_complete(struct light_vdev *vdev,
				   blk_status_t status,
				   struct kvec *kiov,
				   u16 avail_head, u32 vr_idx)
{
	struct light_vringh *vvr;
	struct vringh *vrh;
	unsigned long vaddr;
	size_t total_len;

	vvr = &vdev->vvr[vr_idx];
	vrh = &vvr->vrh;
	total_len = sizeof(struct virtio_blk_outhdr) +
		    VIRTIO_BLK_ID_BYTES + 1;
	vaddr = (unsigned long)phys_to_virt((unsigned long)kiov->iov_base);
	*(blk_status_t *)vaddr = status;

	spin_lock_bh(&vblk_io_complete_lock);
	vringh_complete_kern(vrh, avail_head, total_len);
	if (vringh_need_notify_kern(vrh) > 0)
		vdev->notify(vdev);
	spin_unlock_bh(&vblk_io_complete_lock);
}

static inline void light_vblk_prepare_bio(struct light_vdev *vdev,
					  struct bio *bio,
					  struct vringh_kiov *kiov,
					  struct virtio_blk_outhdr *hdr,
					  u32 iov_cnt)
{
	unsigned long addr;
	struct bio_vec *bv;
	unsigned int offset;
	struct kvec *iov;
	size_t len;
	int i;

	bio->bi_iter.bi_sector = hdr->sector;
	bio_set_dev(bio, vdev->this_bdev);
	bio->bi_vcnt = iov_cnt;
	bio->bi_iter.bi_size = 0;
	bio->bi_end_io = light_vblk_endio;
	bio->bi_status = BLK_STS_OK;
	bv = bio->bi_io_vec;
	for (i = 0; i < iov_cnt; i++) {
		iov = &kiov->iov[kiov->i];

		addr = (unsigned long)iov->iov_base;
		len = iov->iov_len;
		offset = offset_in_page(addr);

		bv[i].bv_page = phys_to_page(addr);
		bv[i].bv_len = len;
		bv[i].bv_offset = offset;

		bio->bi_iter.bi_size += len;

		kiov->i++;
	}
}

/*
 * Use the standard VRINGH infrastructure in the kernel to fetch new
 * descriptors, initiate the copies and update the used ring.
 */
static void light_vblk_io_handle(struct light_vdev *vdev, u32 vr_idx)
{
	struct light_vringh *vvr = &vdev->vvr[vr_idx];
	struct vringh_kiov *riov = &vvr->riov;
	struct vringh_kiov *wiov = &vvr->wiov;
	struct vringh *vrh = &vvr->vrh;
	struct vringh_kiov *kiov;
	u16 *head = &vvr->head;
	struct bio *bio;
	struct vblk_bio_info *bio_info;
	static struct virtio_blk_outhdr hdr;
	blk_status_t status = BLK_STS_OK;
	u32 iov_cnt;

	/*
	 * Add more comments below to make it easy to understand the logic.
	 * hdr format check must done before getting its riov and wiov,
	 * as in vringh_getdesc_kern() it will let vrh->last_avail_idx++
	 * after a new avail idx is got. But in this format check func,
	 * it must use vrh->last_avail_idx before +1. It will be the
	 * next avavil idx after +1, but what we need is the current
	 * avail idx.
	 */
	status = light_vblk_check_hdr_format(vdev, vr_idx);
	if (status != BLK_STS_OK)
		goto err;

	/* Fetch a new IOVEC if all previous elements have been processed */
	if (riov->i == riov->used && wiov->i == wiov->used) {
		if (vringh_getdesc_kern(vrh, riov, wiov, head, GFP_KERNEL) <= 0) {
			status = BLK_STS_IOERR;
			goto err;
		}
	}

	/* the first iov(riov->i = 0) in riov is for hdr */
	light_vblk_get_hdr(&riov->iov[riov->i++], &hdr);

	if (riov->i < riov->used) {
		kiov = riov;
		/* it is iov count of guest write io payload */
		iov_cnt = riov->used - 1; /* first iov(riov->i is 0) is hdr */
	} else {
		kiov = wiov;
		/* it is iov count of guest read io payload */
		iov_cnt = wiov->used - 1; /* last iov is for status iov */
	}

	bio_info = kmalloc(sizeof(struct vblk_bio_info), GFP_KERNEL | __GFP_ZERO);
	if (!bio_info) {
		status = BLK_STS_NOSPC;
		goto err;
	}

	/*
	 * bio_alloc() is guaranteed to return a bio when allowed
	 * to sleep and we request a valid number of vectors.
	 * see bio_alloc_bioset().
	 */
	bio = bio_alloc(GFP_KERNEL, iov_cnt);

	switch (hdr.type) {
	case VIRTIO_BLK_T_IN:
		bio->bi_opf = REQ_OP_READ;
		break;
	case VIRTIO_BLK_T_OUT:
		bio->bi_opf = REQ_OP_WRITE;
		break;
	case VIRTIO_BLK_T_DISCARD:
		status = light_vblk_issue_discard(vdev, &riov->iov[riov->i++]);
		goto free;
	case VIRTIO_BLK_T_FLUSH:
		bio->bi_opf = REQ_OP_FLUSH;
		break;
	case VIRTIO_BLK_T_WRITE_ZEROES:
		status = light_vblk_issue_zeroout(vdev, &riov->iov[riov->i++]);
		goto free;
	case VIRTIO_BLK_T_GET_ID:
		get_device_id(&wiov->iov[wiov->i++]);
		goto free;
	default:
		pr_warn("request type %d\n", hdr.type);
		status = BLK_STS_NOTSUPP;
		goto free;
	}

	light_vblk_prepare_bio(vdev, bio, kiov, &hdr, iov_cnt);

	/*
	 * wiov is the IN iov, which needs backend writes data to it,
	 * for one single vblk io, the last iov in wiov is for io status,
	 * backend needs write io complete status to this iov in end_io
	 * for frontend fetching.
	 */
	bio_info->kiov.iov_base = wiov->iov[wiov->i].iov_base;
	bio_info->vring_idx = vr_idx;
	bio_info->avail_head = *head;
	bio_info->vdev = vdev;
	bio->bi_private = (struct bio_info *)bio_info;

	vringh_kiov_cleanup(riov);
	vringh_kiov_cleanup(wiov);

	submit_bio(bio);

	return;

free:
	bio_put(bio);
	kfree(bio_info);
err:
	/*
	 * 'wiov->used - 1' is the status iov index,
	 * it is the last iov in wiov.
	 */
	light_vblk_io_complete(vdev, status, &wiov->iov[wiov->used - 1],
			       *head, vr_idx);
	vringh_kiov_cleanup(riov);
	vringh_kiov_cleanup(wiov);
}

static int light_vblk_open(struct inode *inode, struct file *f)
{
	struct vhost_virtqueue **vqs;
	struct light_vdev *vdev;

	/* Alloc light_vdev */
	vdev = kzalloc(sizeof(struct light_vdev), GFP_KERNEL);
	if (!vdev)
		return -ENOMEM;

        vqs = kmalloc_array(ARRAY_SIZE(vdev->vqs), sizeof(*vqs), GFP_KERNEL);
        if (!vqs) {
		kfree(vdev);
                return -ENOMEM;
        }

	mutex_init(&vdev->mutex);
	spin_lock_init(&vdev->lock);

	mutex_lock(&vdev_list_lock);
	list_add_tail(&vdev->list, &vdev_list);
	mutex_unlock(&vdev_list_lock);

	vqs[LIGHT_VBLK_VQ_REQ] = &vdev->vqs[LIGHT_VBLK_VQ_REQ];
	vdev->vqs[LIGHT_VBLK_VQ_REQ].handle_kick = light_vblk_handle_guest_kick;
	vhost_dev_init(&vdev->vhost_dev, vqs, LIGHT_VBLK_VQ_MAX, 0,
		       0, 0, true, NULL);

	vdev->notify = light_vblk_notify;

	f->private_data = vdev;

	return 0;
}

static int light_vblk_release(struct inode *inode, struct file *f)
{
	struct light_vdev *vdev = f->private_data;

	mutex_lock(&vdev_list_lock);
	list_del_init(&vdev->list);
	mutex_unlock(&vdev_list_lock);

	kfree(vdev->vhost_dev.vqs);
	kfree(vdev);

	return 0;
}

static long light_vblk_ioctl(struct file *f, unsigned int ioctl,
			    unsigned long arg)
{
	struct light_vdev *vdev = f->private_data;
	void __user *argp = (void __user *)arg;
	static struct vhost_vring_state s;
	struct vhost_vring_file b;
	struct vhost_vring_addr a;
	int r = -EFAULT;

	mutex_lock(&vdev->mutex);

        switch (ioctl) {
	case VHOST_BLK_SET_BACKEND:
		if (copy_from_user(&b, argp, sizeof b))
			goto out;
		r = light_vblk_set_backend(vdev, b.index, b.fd);
		break;
        case VHOST_SET_VRING_NUM:
		if (copy_from_user(&s, argp, sizeof s))
			goto out;
		r = 0;
		break;
	case VHOST_SET_VRING_ADDR:
		if (copy_from_user(&a, argp, sizeof a))
			goto out;

		r = light_vblk_map_vring(vdev, s.num, &a);
		break;
	default:
		r = vhost_dev_ioctl(&vdev->vhost_dev, ioctl, argp);
		if (r == -ENOIOCTLCMD)
			r = vhost_vring_ioctl(&vdev->vhost_dev, ioctl, argp);
		break;
	}

out:
	mutex_unlock(&vdev->mutex);

	return r;
}

static const struct file_operations light_vblk_fops = {
	.owner          = THIS_MODULE,
	.release        = light_vblk_release,
	.open           = light_vblk_open,
	.unlocked_ioctl = light_vblk_ioctl,
};

static struct miscdevice light_vblk_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "vhost-blk",
	.fops = &light_vblk_fops,
};

static int __init light_vblk_init(void)
{
	int rc;

	rc = misc_register(&light_vblk_misc);
	if (rc < 0)
		return rc;

	pr_debug("vhost-blk driver init successfully\n");

	return 0;
}

static void __exit light_vblk_exit(void)
{
	struct light_vdev *vdev;

	misc_deregister(&light_vblk_misc);

	mutex_lock(&vdev_list_lock);
	list_for_each_entry(vdev, &vdev_list, list) {
		list_del_init(&vdev->list);
		kfree(vdev);
	}
	mutex_unlock(&vdev_list_lock);
}

module_init(light_vblk_init);
module_exit(light_vblk_exit);

MODULE_DESCRIPTION("Vhost-blk driver");
MODULE_AUTHOR("Xianting Tian <xianting.tian@linux.alibaba.com>");
MODULE_LICENSE("GPL v2");
