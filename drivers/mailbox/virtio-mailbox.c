// SPDX-License-Identifier: GPL-2.0+

#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/virtio.h>
#include <linux/virtio_mailbox.h>
#include <linux/virtio_config.h>
#include <linux/virtio_light.h>
#include <linux/virtio_ids.h>

#define DRIVER_NAME	"virtio-mailbox"

static inline u32 virtio_cread32_nosleep(struct virtio_device *vdev,
				 unsigned int offset)
{
	__virtio32 ret;

	vdev->config->get(vdev, offset, &ret, sizeof(ret));
	return virtio32_to_cpu(vdev, ret);
}

static inline void virtio_cwrite32_nosleep(struct virtio_device *vdev,
				   unsigned int offset, u32 val)
{
	__virtio32 v;

	v = cpu_to_virtio32(vdev, val);
	vdev->config->set(vdev, offset, &v, sizeof(v));
}

struct virtio_mbox {
	struct mbox_controller controller;
	struct virtio_device *vdev;
	unsigned int chan_num;
	unsigned int msg_size;
	struct mbox_chan *chans;
	struct virtio_chan *vchans;
	struct virtqueue **in_vqs;
	struct virtqueue **out_vqs;
};

struct virtio_chan {
	struct virtqueue *in_vq;
	struct virtqueue *out_vq;
	struct virtio_mbox *vmbox;
	struct mbox_chan *chan;
	bool in_use;
	u8 idx;
	spinlock_t lock;
};

static int add_invq_buf(struct virtqueue *vq, void *buf, u32 msg_size)
{
	struct scatterlist sg[1];
	int ret;

	sg_init_one(sg, buf, msg_size);

	ret = virtqueue_add_inbuf(vq, sg, 1, buf, GFP_ATOMIC);

	if (!ret)
		ret = vq->num_free;
	return ret;
}

static int fill_invq(struct virtqueue *vq, spinlock_t *lock,
		     unsigned int msg_size)
{
	int nr_added_bufs;
	int ret;
	void *buf;

	nr_added_bufs = 0;
	do {
		buf = kmalloc(msg_size, GFP_KERNEL);
		if (!buf)
			return -ENOMEM;

		spin_lock_irq(lock);
		ret = add_invq_buf(vq, buf, msg_size);
		if (ret < 0) {
			spin_unlock_irq(lock);
			kfree(buf);
			return ret;
		}
		nr_added_bufs++;
		spin_unlock_irq(lock);
	} while (ret > 0);

	return nr_added_bufs;
}

/* callback for virtio inqueue */
static void virtio_mbox_invq_cb(struct virtqueue *vq)
{
	struct virtio_mbox *vmbox = vq->vdev->priv;
	struct mbox_chan *chan;
	struct virtio_chan *vchan;
	unsigned int len;
	u32 chan_idx, msg, val = 0;
	unsigned long flags;
	void *buf;

	/*
	 * As vq->index is in range {0,1,2,...vmbox->chan_num*2},
	 * vq->index in range {0,2,4,...} is for in_vqs;
	 * vq->index in range {1,3,5,...} is for out_vqs;
	 * one in and out vq pair is for one chan, eg,
	 * {0,1} for chan 0;
	 * {2,3} for chan 1;
	 * {4,5} for chan 2,etc
	 */
	chan_idx = vq->index/2;
	chan = &vmbox->chans[chan_idx];
	vchan = chan->con_priv;

	msg = virtio_cread32_nosleep(vmbox->vdev, offsets[MSG_H] + chan_idx * 4);

	spin_lock_irqsave(&vchan->lock, flags);
	if (!vchan->in_use || !msg) {
		spin_unlock_irqrestore(&vchan->lock, flags);
		return;
	}
	spin_unlock_irqrestore(&vchan->lock, flags);

	virtio_cwrite32_nosleep(vmbox->vdev, offsets[MSG_H] + chan_idx * 4, val);

	while (1) {
		buf = virtqueue_get_buf(vq, &len);
		if (buf == NULL)
			break;
		/*
		 * assume client copied vmsg to its private buf
		 * when this func return
		 */
		mbox_chan_received_data(chan, buf);
		add_invq_buf(vq, buf, vmbox->msg_size);

		/* mark Ack */
		val = 1;
		virtio_cwrite32_nosleep(vmbox->vdev, offsets[ACK_G] + chan_idx * 4, val);

		/* tell backend to handle Ack */
		virtqueue_kick(vq);
	}
}

/*
 * callback for virtio outqueue
 * we use this func to reclaim used buffer and handle ack
 */
static void virtio_mbox_outvq_cb(struct virtqueue *vq)
{
	struct virtio_mbox *vmbox = vq->vdev->priv;
	struct mbox_chan *chan;
	struct virtio_chan *vchan;
	unsigned int len;
	int chan_idx;
	void *buf;
	unsigned long flags;
	u32 ack, val = 0;

	/* see comments in virtio_mbox_invq_cb() */
	chan_idx = vq->index/2;
	chan = &vmbox->chans[chan_idx];
	vchan = chan->con_priv;

	ack = virtio_cread32_nosleep(vmbox->vdev, offsets[ACK_H] + chan_idx * 4);
	spin_lock_irqsave(&vchan->lock, flags);
	if (!vchan->in_use || !ack) {
		spin_unlock_irqrestore(&vchan->lock, flags);
		return;
	}
	spin_unlock_irqrestore(&vchan->lock, flags);
	virtio_cwrite32_nosleep(vmbox->vdev, offsets[ACK_H] + chan_idx * 4, val);

	/*
	 * don't warry, free used buf should be fast,
	 * normally we only have one buf to free.
	 */
	while (1) {
		buf = virtqueue_get_buf(vq, &len);
		if (buf == NULL)
			break;
		kfree(buf);
	}

	/*
	 * backend Ack
	 * when virtio_mbox_outvq_cb() is called, we think backend already
	 * received last Tx msg we send by virtio_mbox_send_data(), so we
	 * think it is Ack from backend, then we can mark last Tx done and
	 * send next msg.
	 */
	mbox_chan_txdone(chan, 0);
}

static int init_vqs(struct virtio_mbox *vmbox)
{
	vq_callback_t **callbacks;
	struct virtqueue **vqs;
	int ret = -ENOMEM;
	int i, j, total_vqs;
	const char **names;
	char q_name[2][32];

	total_vqs = vmbox->chan_num * 2;

	vqs = kcalloc(total_vqs, sizeof(struct virtqueue *), GFP_KERNEL);
	callbacks = kmalloc_array(total_vqs, sizeof(vq_callback_t *), GFP_KERNEL);
	names = kmalloc_array(total_vqs, sizeof(char *), GFP_KERNEL);
	vmbox->in_vqs = kmalloc_array(total_vqs, sizeof(struct virtqueue *), GFP_KERNEL);
	vmbox->out_vqs = kmalloc_array(total_vqs, sizeof(struct virtqueue *), GFP_KERNEL);
	if (!vqs || !callbacks || !names || !vmbox->in_vqs || !vmbox->out_vqs)
		goto free;

	j = 0;
	for (i = 0; i < vmbox->chan_num; i++) {
		callbacks[j] = virtio_mbox_invq_cb;
		callbacks[j + 1] = virtio_mbox_outvq_cb;
		sprintf(q_name[0], "mailbox_input.%d", i);
		sprintf(q_name[1], "mailbox_output.%d", i);
		names[j] = q_name[0];
		names[j + 1] = q_name[1];
		j += 2;
	}

	ret = virtio_find_vqs(vmbox->vdev, total_vqs, vqs, callbacks, names, NULL);
	if (ret)
		goto free;

	j = 0;
	for (i = 0; i < vmbox->chan_num; i++) {
		vmbox->in_vqs[i] = vqs[j];
		vmbox->out_vqs[i] = vqs[j + 1];
		j += 2;
	}

	kfree(names);
	kfree(callbacks);
	kfree(vqs);

	return 0;

free:
	kfree(vmbox->out_vqs);
	kfree(vmbox->in_vqs);
	kfree(names);
	kfree(callbacks);
	kfree(vqs);

	return ret;
}

static void virtio_mbox_del_vqs(struct virtio_mbox *vmbox)
{
	struct virtio_device *vdev = vmbox->vdev;

	vdev->config->del_vqs(vdev);
}

static void virtio_mbox_reclaim_used_bufs(struct virtio_mbox *vmbox)
{
	struct virtio_chan *vchans = vmbox->vchans;
	struct virtqueue *vq;
	unsigned long flags;
	unsigned int len;
	u32 chan_idx;
	void *buf;

	/*
	 * go through all vqs with the order by vq->index:
	 * 0, 1, 2, 3...
	 */
	virtio_device_for_each_vq(vmbox->vdev, vq) {
		chan_idx = vq->index/2;

		spin_lock_irqsave(&vchans[chan_idx].lock, flags);

		if (vchans[chan_idx].in_use) {
			if (chan_idx & 1)
				vchans[chan_idx].in_use = false;

			while (1) {
				buf = virtqueue_get_buf(vq, &len);
				if (buf == NULL)
					break;
				kfree(buf);
			}
		}

		spin_unlock_irqrestore(&vchans[chan_idx].lock, flags);
	}
}

static void remove_vq_common(struct virtio_mbox *vmbox)
{
	vmbox->vdev->config->reset(vmbox->vdev);

	virtio_mbox_reclaim_used_bufs(vmbox);

	virtio_mbox_del_vqs(vmbox);
}

static struct mbox_chan *virtio_mbox_xlate(struct mbox_controller *mbox,
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
		spin_unlock_irqrestore(&vchan->lock, flags);
		dev_err(mbox->dev, "Channel idx %d is in use\n", ch);
		return ERR_PTR(-EINVAL);
	}
	spin_unlock_irqrestore(&vchan->lock, flags);

	return chan;
}

static int virtio_mbox_send_data(struct mbox_chan *chan, void *vmsg)
{
	struct virtio_chan *vchan = chan->con_priv;
	struct virtio_mbox *vmbox = vchan->vmbox;
	struct scatterlist sg[1];
	struct virtqueue *out_vq;
	void *data;
	u32 val = 1;
	int err = 0;

	out_vq = vchan->out_vq;

	data = kmemdup(vmsg, vmbox->msg_size, GFP_ATOMIC);
	if (!data) {
		dev_err(chan->mbox->dev, "memory alloc fialed\n");
		goto exit;
	}
	sg_init_one(sg, data, vmbox->msg_size);
	err = virtqueue_add_outbuf(out_vq, sg, 1, data, GFP_ATOMIC);
	if (err) {
		kfree(data);
		dev_err(chan->mbox->dev, "add outbuf failed %d\n", err);
		goto exit;
	}

	virtio_cwrite32_nosleep(vmbox->vdev, offsets[MSG_G] + vchan->idx * 4, val);

	/* tell backend for new data */
	virtqueue_kick(out_vq);

exit:
	return 0;
}

static int virtio_mbox_startup(struct mbox_chan *chan)
{
	struct virtio_chan *vchan = chan->con_priv;
	unsigned long flags;

	spin_lock_irqsave(&vchan->lock, flags);
	vchan->in_use = true;
	spin_unlock_irqrestore(&vchan->lock, flags);

	return 0;
}

static void virtio_mbox_shutdown(struct mbox_chan *chan)
{
	struct virtio_chan *vchan = chan->con_priv;
	unsigned long flags;

	spin_lock_irqsave(&vchan->lock, flags);
	vchan->in_use = false;
	spin_unlock_irqrestore(&vchan->lock, flags);
}

static const struct mbox_chan_ops virtio_mbox_ops = {
	.send_data	= virtio_mbox_send_data,
	.startup	= virtio_mbox_startup,
	.shutdown	= virtio_mbox_shutdown,
};

/* setup vqs and vmbox */
static int virtio_mbox_setup(struct virtio_device *vdev)
{
	struct virtio_mbox *vmbox;
	struct virtio_chan *vchans;
	int ret = -ENOMEM;
	int i;

	vmbox = devm_kzalloc(&vdev->dev, sizeof(*vmbox), GFP_KERNEL);
	if (!vmbox)
		return ret;
	vdev->priv = vmbox;

	vmbox->vdev = vdev;

	ret = virtio_cread_feature(vdev, VIRTIO_MAILBOX_F_CHAN_NUM,
				   struct virtio_mailbox_config, chan_num,
				   &vmbox->chan_num);
	if (ret)
		vmbox->chan_num = VIRTIO_MAILBOX_CHAN_NUM_DEFAULT;

	ret = virtio_cread_feature(vdev, VIRTIO_MAILBOX_F_MSG_SIZE,
				   struct virtio_mailbox_config, msg_size,
				   &vmbox->msg_size);
	if (ret)
		vmbox->msg_size = VIRTIO_MAILBOX_MSG_SIZE_DEFAULT;


	vmbox->chans = devm_kcalloc(&vmbox->vdev->dev, vmbox->chan_num,
				    sizeof(*vmbox->chans), GFP_KERNEL);
	if (!vmbox->chans)
		goto free_vmbox;

	vchans = devm_kcalloc(&vmbox->vdev->dev, vmbox->chan_num,
			      sizeof(*vchans), GFP_KERNEL);
	if (!vchans)
		goto free_chans;
	vmbox->vchans = vchans;

	/*
	 * the actual dev is detected by virtio_light bus in
	 * virtio_light_probe():
	 * vm_dev->vdev.dev.parent = dev;
	 */
	vmbox->controller.dev = vdev->dev.parent;

	vmbox->controller.num_chans = vmbox->chan_num;
	vmbox->controller.chans = &vmbox->chans[0];
	vmbox->controller.ops = &virtio_mbox_ops;
	vmbox->controller.of_xlate = &virtio_mbox_xlate;
	vmbox->controller.txdone_irq = true; /* see comments of mbox_chan_txdone() */

	ret = init_vqs(vmbox);
	if (ret)
		goto free_vchans;

	/* map chan to in,out queue pair */
	for (i = 0; i < vmbox->chan_num; i++) {
		vchans[i].vmbox = vmbox;
		vchans[i].chan = &vmbox->chans[i];
		vchans[i].idx = i;
		vchans[i].in_vq = vmbox->in_vqs[i];
		vchans[i].out_vq = vmbox->out_vqs[i];
		spin_lock_init(&(vchans[i].lock));

		/* The standard mailbox channel mapped to vq */
		vmbox->chans[i].con_priv = &vchans[i];

		ret = fill_invq(vmbox->in_vqs[i], &vchans[i].lock, vmbox->msg_size);
		if (ret == -ENOMEM)
			goto free_vchans;
	}

	ret = devm_mbox_controller_register(&vmbox->vdev->dev, &vmbox->controller);
	if (ret) {
		dev_err(&vmbox->vdev->dev, "Could not register mailbox controller\n");
		goto free_vchans;
	}

	dev_set_drvdata(&vmbox->vdev->dev, vmbox);

	dev_info(&vmbox->vdev->dev, "registered %d channels\n", vmbox->chan_num);

	return 0;

free_vchans:
	kfree(vchans);
free_chans:
	kfree(vmbox->chans);
free_vmbox:
	kfree(vmbox);

	return ret;
}

static int virtio_mbox_probe(struct virtio_device *vdev)
{
	struct device *dev = &vdev->dev;
	struct virtio_mbox *vmbox;
	int i, ret = 0;
	u32 val = 0;

	ret = virtio_mbox_setup(vdev);
	if (ret) {
		dev_err(dev, "virtio_mbox setup failed\n");
		return ret;
	}

	vmbox = vdev->priv;
	for (i = 0; i < vmbox->chan_num; i++) {
		virtio_cwrite32(vdev, offsets[ACK_G] + i * 4, val);
		virtio_cwrite32(vdev, offsets[MSG_G] + i * 4, val);
	}

	virtio_device_ready(vdev);

	dev_err(dev, "frontend driver init successfully\n");

	return ret;
}

static void virtio_mbox_remove(struct virtio_device *vdev)
{
	struct virtio_mbox *vmbox = vdev->priv;

	kfree(vmbox->chans);
	kfree(vmbox->vchans);
	kfree(vmbox->out_vqs);
	kfree(vmbox->in_vqs);

	remove_vq_common(vmbox);

	kfree(vmbox);
}

static struct virtio_device_id id_table[] = {
	{ VIRTIO_ID_MAILBOX, VIRTIO_DEV_ANY_ID },
	{ 0 },
};

static unsigned int features[] = {
	VIRTIO_MAILBOX_F_CHAN_NUM, VIRTIO_MAILBOX_F_MSG_SIZE,
};

static struct virtio_driver virtio_mailbox_driver = {
	.feature_table = features,
	.feature_table_size = ARRAY_SIZE(features),
	.driver.name =	KBUILD_MODNAME,
	.driver.owner =	THIS_MODULE,
	.id_table =	id_table,
	.probe =	virtio_mbox_probe,
	.remove =	virtio_mbox_remove,
};

static __init int virtio_mailbox_driver_init(void)
{
	int ret;

        ret = register_virtio_driver(&virtio_mailbox_driver);

	return ret;
}
module_init(virtio_mailbox_driver_init);

static __exit void virtio_mailbox_driver_exit(void)
{
	unregister_virtio_driver(&virtio_mailbox_driver);
}
module_exit(virtio_mailbox_driver_exit);

MODULE_DEVICE_TABLE(virtio, id_table);
MODULE_AUTHOR("Xianting Tian <xianting.tian@linux.alibaba.com>");
MODULE_DESCRIPTION("Virtio mailbox driver");
MODULE_LICENSE("GPL");
