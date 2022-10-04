// SPDX-License-Identifier: GPL-2.0-only

#include <linux/virtio_mailbox.h>
#include <linux/virtio_ids.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/mailbox_controller.h>
#include <linux/virtio_light.h>
#include "../light_vringh.h"

static struct light_vdev *vdev;
static volatile unsigned char __iomem *virtio_config;
static u32 chan_num;
static u32 msg_size;

#define DRV_NAME	"light-virtmbox"

struct virtio_mbox {
	struct mbox_controller controller;
	struct light_vdev *vdev;
	unsigned int chan_num;
	struct mbox_chan *chans;
	struct virtio_chan *vchans;
	struct light_vringh *vrh;
	bool ready;
};

struct virtio_chan {
	struct light_vringh *in_vrh;
	struct light_vringh *out_vrh;
	struct virtio_mbox *vmbox;
	struct mbox_chan *chan;
	bool in_use;
	u8 idx;
	spinlock_t lock;
};

static inline volatile unsigned char *offset(u32 index)
{
	return virtio_config + VIRTIO_LIGHT_CONFIG + offsets[index];
}

static void light_vmbox_notify(struct vringh *vrh)
{
#ifdef CONFIG_SOC_INT_SRC7
	writel(1, vdev->frontend_intr_reg);
#else
	/* Write the interrupt register to signal the other end */
	writel(1, vdev->frontend_intr_reg + 0x10);
#endif
}

static inline bool light_vmbox_vring_avail(struct light_vringh *lvrh)
{
	/*
	 * vrh.last_avail_idx will be +1 after fetch a new avail idx
	 * in vringh_getdesc_kern().
	 */
	return lvrh->vrh.last_avail_idx != lvrh->vring.vr.avail->idx;
}

static int light_vmbox_vring_process(struct light_vringh *lvrh, void *vmsg,
				     struct mbox_chan *chan, bool read)
{
	struct virtio_chan *vchan = chan->con_priv;
	u16 *avail_head = &lvrh->head;
	struct kvec *kiov;
	struct vringh_kiov *riov = &lvrh->riov;
	struct vringh_kiov *wiov = &lvrh->wiov;
	unsigned long vaddr;
	struct vringh *vrh = &lvrh->vrh;
	int ret = 0;

	/* normally, we can get avail buf in the first while loop */
	while (read && !light_vmbox_vring_avail(lvrh)) {
		cpu_relax();
		pr_debug("we are waiting for one avail buf in in_queue\n");
	}

	if (riov->i == riov->used && wiov->i == wiov->used) {
		ret = vringh_getdesc_kern(vrh, riov, wiov, avail_head, GFP_ATOMIC);
		/* Check if there are available descriptors */
		if (ret <= 0)
			return ret;
	}

	if (read)
		kiov = &wiov->iov[wiov->i];
	else
		kiov = &riov->iov[riov->i];

	vaddr = (unsigned long)phys_to_virt((unsigned long)kiov->iov_base);

	if (read) {
		memcpy((void *)vaddr, vmsg, msg_size);
		writel(1, offset(MSG_H) + vchan->idx * 4);
	} else {
		mbox_chan_received_data(chan, (void *)vaddr);
		writel(1, offset(ACK_H) + vchan->idx * 4);
	}

	vringh_complete_kern(vrh, *avail_head, msg_size);
	if (vringh_need_notify_kern(vrh) > 0)
		vringh_notify(vrh); /* send Ack to frontend */

	vringh_kiov_cleanup(riov);
	vringh_kiov_cleanup(wiov);

	return 0;
}

static void light_vmbox_msg_process(struct light_vdev *vdev)
{
	struct virtio_mbox *vmbox = vdev->priv;
	struct virtio_chan *vchans = vmbox->vchans;
	struct light_vringh *lvrh;
	unsigned long flags;
	u32 msg;
	int i;

	for (i = 0; i < vmbox->chan_num; i++) {
		spin_lock_irqsave(&vchans[i].lock, flags);

		msg = readl(offset(MSG_G) + vchans[i].idx * 4);
		if (vchans[i].in_use && msg) {
			lvrh = vchans[i].out_vrh;

			/*
			 * don't worry, it will be fast.
			 * normally, we only has one avail desc to handle
			 */
			while (light_vmbox_vring_avail(lvrh))
				light_vmbox_vring_process(lvrh, NULL,
						vchans[i].chan, false);

			writel(0, offset(MSG_G) + vchans[i].idx * 4);
		}

		spin_unlock_irqrestore(&vchans[i].lock, flags);
	}
}

static void light_vmbox_ack_process(struct light_vdev *vdev)
{
	struct virtio_mbox *vmbox = vdev->priv;
	struct virtio_chan *vchans = vmbox->vchans;
	unsigned long flags;
	u32 ack;
	int i;

	for (i = 0; i < vmbox->chan_num; i++) {
		spin_lock_irqsave(&vchans[i].lock, flags);

		ack = readl(offset(ACK_G) + vchans[i].idx * 4);
		if (vchans[i].in_use && ack) {
			mbox_chan_txdone(vchans[i].chan, 0);
			writel(0, offset(ACK_G) + vchans[i].idx * 4);
		}

		spin_unlock_irqrestore(&vchans[i].lock, flags);
	}
}

static irqreturn_t light_vmbox_interrupt(int irq, void *opaque)
{
	struct light_vdev *virt_dev;

	virt_dev = (struct light_vdev *)opaque;
	if (virt_dev->virtio_id != VIRTIO_ID_MAILBOX)
		return IRQ_NONE;

#ifdef CONFIG_SOC_INT_SRC7
	writel(0, vdev->backend_intr_reg);
#else
	/* clear interrupt*/
	writel(vdev->clear_intr, vdev->backend_intr_reg + 0x4);
#endif

	/* handle Ack */
	light_vmbox_ack_process(virt_dev);

	/* handle Msg */
	light_vmbox_msg_process(virt_dev);

	return IRQ_HANDLED;
}

static int light_vmbox_vring_map(struct light_vdev *vdev)
{
	int i, ret, vr_size;
	phys_addr_t phys_addr;
	struct virtio_mbox *vmbox = vdev->priv;
	u32 vq_num;
	u32 entry_num;
	u32 align;
	u32 count = 0;

	/* Wait frontend driver initialization ready, then we can go */
	while (!(readl(virtio_config + VIRTIO_LIGHT_STATUS) & VIRTIO_CONFIG_S_DRIVER_OK)) {
		msleep(200);
		if (count++ == 50) {
			count = 0;
			pr_debug("Vmbox backend: waiting frontend driver ready!!!\n");
		}
	}
	/* We already know frontend is ready, now reset status */
	writel(0, virtio_config + VIRTIO_LIGHT_STATUS);

	entry_num = readl(virtio_config + VIRTIO_LIGHT_QUEUE_SIZE);
	if (entry_num == 0)
		return -EINVAL;

	align = readl(virtio_config + VIRTIO_LIGHT_QUEUE_ALIGN);
	if (align < 0)
		return -EINVAL;

	vq_num = readl(virtio_config + VIRTIO_LIGHT_QUEUE_NUM);
	if (vq_num == 0 || vq_num != 2 * chan_num) {
		pr_info("Vmbox backend: vq num not match(%d, %d)\n",
				vq_num, 2 * chan_num);
		return ret;
	}

	for (i = 0; i < vq_num; i++) {
		struct light_vringh *vvr = &vdev->vvr[i];
		struct light_vring *vr = &vdev->vvr[i].vring;

		vr_size = PAGE_ALIGN(round_up(vring_size(entry_num, align), 4));

		/* init vring */
		phys_addr = (phys_addr_t)readl(virtio_config +
					       VIRTIO_LIGHT_QUEUE_PFN + i * 4) << PAGE_SHIFT;
		vr->va = (void *) phys_to_virt(phys_addr);
		pr_debug("Vmbox backend: vring(%d) virtual addr %lx, phys addr %llx\n",
				i, (unsigned long)vr->va, phys_addr);

		vr->len = vr_size;

		vring_init(&vr->vr, entry_num, vr->va, align);
		ret = vringh_init_kern(&vvr->vrh,
				       0, /* don't need features  */
				       entry_num, false, vr->vr.desc, vr->vr.avail,
				       vr->vr.used);
		if (ret) {
			pr_err("%s %d err %d\n", __func__, __LINE__, ret);
			goto err;
		}

		vringh_kiov_init(&vvr->riov, NULL, 0);
		vringh_kiov_init(&vvr->wiov, NULL, 0);
		vvr->vdev = vdev;
		vvr->vrh.notify = light_vmbox_notify;
	}

	if (request_irq(vdev->irq, light_vmbox_interrupt, IRQF_SHARED,
			"virtio_mbox", vdev) < 0) {
		pr_err("Vmbox backend: register IRQ %d failed\n", vdev->irq);
		goto err;
	}

	vmbox->ready = true;

	pr_info("Vmbox backend added device(%d) with %d vqs %d entries.\n",
			vdev->virtio_id, vq_num, entry_num);
	return 0;
err:
	return ret;
}

static struct mbox_chan *light_vmbox_xlate(struct mbox_controller *mbox,
				const struct of_phandle_args *sp)
{
	struct mbox_chan *chan;
	unsigned int ch = sp->args[0];
	struct virtio_chan *vchan;
	unsigned long flags;

	if (ch >= mbox->num_chans) {
		dev_err(mbox->dev, "Invalid channel idx %d\n", ch);
		return ERR_PTR(-EINVAL);
	}

	chan = &mbox->chans[ch];
	vchan = chan->con_priv;

	spin_lock_irqsave(&vchan->lock, flags);
	if (vchan->in_use) {
		dev_err(mbox->dev, "Channel idx %d is in use\n", ch);
		spin_unlock_irqrestore(&vchan->lock, flags);
		return ERR_PTR(-EINVAL);
	}
	spin_unlock_irqrestore(&vchan->lock, flags);

	return chan;
}

static int light_vmbox_send_data(struct mbox_chan *chan, void *vmsg)
{
	int ret;

	struct virtio_chan *vchan = chan->con_priv;

	if (!vchan->vmbox->ready)
		return 0;

	ret = light_vmbox_vring_process(vchan->in_vrh, vmsg, chan, true);
	if (ret)
		dev_err(chan->mbox->dev, "message send error %d\n", ret);

	return 0;
}

static int light_vmbox_startup(struct mbox_chan *chan)
{
	struct virtio_chan *vchan = chan->con_priv;
	unsigned long flags;

	spin_lock_irqsave(&vchan->lock, flags);
	vchan->in_use = true;
	spin_unlock_irqrestore(&vchan->lock, flags);

	return 0;
}

static void light_vmbox_shutdown(struct mbox_chan *chan)
{
	struct virtio_chan *vchan = chan->con_priv;
	unsigned long flags;

	spin_lock_irqsave(&vchan->lock, flags);
	vchan->in_use = false;
	spin_unlock_irqrestore(&vchan->lock, flags);
}

static const struct mbox_chan_ops light_vmbox_ops = {
	.send_data	= light_vmbox_send_data,
	.startup	= light_vmbox_startup,
	.shutdown	= light_vmbox_shutdown,
};

static int light_vmbox_setup(struct light_vdev* vdev)
{
	struct virtio_mbox *vmbox;
	struct virtio_chan *vchans;
	int ret = -ENOMEM;
	int i;

	vmbox = devm_kzalloc(vdev->device, sizeof(*vmbox), GFP_KERNEL);
	if (!vmbox)
		return ret;
	vdev->priv = vmbox;

	/*
	 * vmbox->ready is set to true after guest ready,
	 * then user can use vmbox normally.
	 */
	vmbox->ready = false;

	vmbox->vdev = vdev;
	vmbox->chan_num = chan_num;
	vmbox->chans = devm_kcalloc(vmbox->vdev->device, vmbox->chan_num,
				   sizeof(*vmbox->chans), GFP_KERNEL);
	if (!vmbox->chans)
		goto free_vmbox;

	vchans = devm_kcalloc(vmbox->vdev->device, vmbox->chan_num,
			     sizeof(*vchans), GFP_KERNEL);
	if (!vchans)
		goto free_chans;
	vmbox->vchans = vchans;

	vmbox->controller.dev = vmbox->vdev->device;
	vmbox->controller.num_chans = vmbox->chan_num;
	vmbox->controller.chans = &vmbox->chans[0];
	vmbox->controller.ops = &light_vmbox_ops;
	vmbox->controller.of_xlate = &light_vmbox_xlate;
	vmbox->controller.txdone_irq = true; /* see comments of mbox_chan_txdone() */

	/* map chan to in,out vrh pair */
	for (i = 0; i < vmbox->chan_num; i++) {
		vchans[i].vmbox = vmbox;
		vchans[i].chan = &vmbox->chans[i];
		vchans[i].idx = i;
		vchans[i].in_vrh = &vdev->vvr[2 * i];
		vchans[i].out_vrh = &vdev->vvr[2 * i + 1];
		spin_lock_init(&(vchans[i].lock));

		/* The standard mailbox channel mapped to vq */
		vmbox->chans[i].con_priv = &vchans[i];
	}

	ret = devm_mbox_controller_register(vmbox->vdev->device,
					    &vmbox->controller);
	if (ret) {
		dev_err(vmbox->vdev->device, "Could not register mailbox controller\n");
		goto free_vchans;
	}

	dev_set_drvdata(vmbox->vdev->device, vmbox);

	dev_info(vmbox->vdev->device, "registered %d channels\n", vmbox->chan_num);

	return 0;

free_vchans:
	kfree(vchans);
free_chans:
	kfree(vmbox->chans);
free_vmbox:
	kfree(vmbox);

	return ret;
}

static void vmbox_work_handler(struct work_struct *work)
{
	if (light_vmbox_setup(vdev)) {
		pr_err("Mbox backend: vmbox setup failed!!!\n");
		return;
	}

	if (light_vmbox_vring_map(vdev)) {
		pr_err("Mbox backend: vring map failed!!!\n");
		return;
	}
}

static int light_mbox_probe(struct platform_device *pdev)
{
	struct device *dev =  &pdev->dev;
	struct device_node *node = dev->of_node;
	u64 features;
	int i, irq, rc = 0;
	char magic[4] = {'v', 'i', 'r', 't'};

	/* Alloc light_vdev */
	vdev = devm_kzalloc(dev, sizeof(struct light_vdev), GFP_KERNEL);
	if (!vdev)
		return -ENOMEM;
	vdev->device = dev;

	INIT_WORK(&vdev->work, vmbox_work_handler);

	mutex_init(&vdev->mutex);

	virtio_config = ioremap(0x1fe000, VIRTIO_LIGHT_CONFIG_LEN);

	/* Reset status, frontend will fill it */
	writel(0, virtio_config + VIRTIO_LIGHT_STATUS);

	features |= 1ULL << VIRTIO_F_VERSION_1;
	features |= 1ULL << VIRTIO_MAILBOX_F_CHAN_NUM;
	features |= 1ULL << VIRTIO_MAILBOX_F_MSG_SIZE;
	writel(features, virtio_config + VIRTIO_LIGHT_DEVICE_FEATURES_LOW);

	writel(*(u32 *)((void *)magic), virtio_config +
			VIRTIO_LIGHT_MAGIC_VALUE);

	writel(1, virtio_config + VIRTIO_LIGHT_VERSION);

	vdev->virtio_id = VIRTIO_ID_MAILBOX;
	writel(vdev->virtio_id, virtio_config + VIRTIO_LIGHT_DEVICE_ID);

	writel(64, virtio_config + VIRTIO_LIGHT_QUEUE_SIZE_MAX);

	for (i = 0; i < chan_num; i++) {
		writel(0, offset(ACK_H) + i * 4);
		writel(0, offset(MSG_H) + i * 4);
	}

	/* Get vmbox chan_numm & chan_size */
	rc = of_property_read_u32(node, "chan_num", &chan_num);
	if (rc < 0) {
		chan_num = VIRTIO_MAILBOX_CHAN_NUM_DEFAULT;
		pr_err("Vmbox backend: use default chan_num: %d\n", chan_num);
	}
	writel(chan_num, offset(CHAN_NUM));

	rc = of_property_read_u32(node, "msg_size", &msg_size);
	if (rc < 0) {
		msg_size = VIRTIO_MAILBOX_MSG_SIZE_DEFAULT;
		pr_err("Vmbox backend: use default msg_size: %d\n", msg_size);
	}
	writel(msg_size, offset(MSG_SIZE));

#ifdef CONFIG_SOC_INT_SRC7
	vdev->frontend_intr_reg = ioremap(0xFFEF018094, 4);
	vdev->backend_intr_reg = ioremap(0xFFFF019094, 4);
#else
	/* MPW use mailbox as interrupt */
	vdev->frontend_intr_reg = ioremap(0xffefc50000, 0x100);
	vdev->backend_intr_reg  = ioremap(0xffffc3b000, 0x100);
	vdev->enable_intr = 1;
	vdev->clear_intr = 1;
	writel(vdev->enable_intr, vdev->frontend_intr_reg + 0xc);
	writel(vdev->clear_intr, vdev->frontend_intr_reg + 0x4);
#endif

	/* Common IRQ request */
	irq = of_irq_get(node, 0);
	if (irq <= 0) {
		pr_err("Cannot get IRQ resource for blk backend\n");
		rc = -EINVAL;
		goto err;
	}
	vdev->irq = irq;

	schedule_work(&vdev->work);

	pr_info("light_mbox backend driver init successfully(%d %d)\n",
			chan_num, msg_size);

	return 0;

err:
	kfree(vdev);
	return rc;
}

static int light_mbox_remove(struct platform_device *pdev)
{
	cancel_work_sync(&vdev->work);
	free_irq(vdev->irq, vdev);
	kfree(vdev);

	return 0;
}

static const struct of_device_id light_of_table[] = {
	{ .compatible = "thead,virtmbox-backend" },
	{ }
};

MODULE_DEVICE_TABLE(of, light_of_table);

static struct platform_driver light_mbox_driver = {
	.probe  = light_mbox_probe,
	.remove = light_mbox_remove,
	.driver = {
		.name = DRV_NAME,
		.of_match_table = light_of_table,
	},
};
module_platform_driver(light_mbox_driver);

MODULE_DESCRIPTION("T-head virtio-mailbox backend driver");
MODULE_AUTHOR("Xianting Tian <xianting.tian@linux.alibaba.com>");
MODULE_LICENSE("GPL");
