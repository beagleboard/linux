/*
 * OMAP mailbox driver
 *
 * Copyright (C) 2006 Nokia Corporation. All rights reserved.
 *
 * Contact: Toshihiro Kobayashi <toshihiro.kobayashi@nokia.com>
 *		Restructured by Hiroshi DOYU <Hiroshi.DOYU@nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/blkdev.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <asm/arch/mailbox.h>
#include "mailbox.h"

static struct omap_mbox *mboxes;
static DEFINE_RWLOCK(mboxes_lock);

/* Mailbox Sequence Bit function */
void omap_mbox_init_seq(struct omap_mbox *mbox)
{
	mbox_seq_init(mbox);
}
EXPORT_SYMBOL(omap_mbox_init_seq);

/*
 * message sender
 */
static int __mbox_msg_send(struct omap_mbox *mbox, mbox_msg_t msg, void *arg)
{
	int ret = 0, i = 1000;

	while (mbox_fifo_full(mbox)) {
		if (mbox->ops->type == OMAP_MBOX_TYPE2)
			return -1;
		if (--i == 0)
			return -1;
		udelay(1);
	}

	if (mbox->msg_sender_cb && arg) {
		ret = mbox->msg_sender_cb(arg);
		if (ret)
			goto out;
	}

	mbox_seq_toggle(mbox, &msg);
	mbox_fifo_write(mbox, msg);
 out:
	return ret;
}

int omap_mbox_msg_send(struct omap_mbox *mbox, mbox_msg_t msg, void* arg)
{
	struct request *rq;
	int ret = 0;

	rq = blk_get_request(mbox->txq, WRITE, GFP_ATOMIC);
	if (unlikely(!rq)) {
		ret = -ENOMEM;
		goto fail;
	}

	rq->data = (void *)msg;
	blk_insert_request(mbox->txq, rq, 0, arg);
 fail:
	return ret;
}
EXPORT_SYMBOL(omap_mbox_msg_send);

/*
 * Message receiver(workqueue)
 */
static void mbox_msg_receiver(struct work_struct *work)
{
	struct omap_mbox *mbox =
	    container_of(work, struct omap_mbox, msg_receive);
	struct request_queue *q = mbox->rxq;
	struct request *rq;
	mbox_msg_t msg;

	if (mbox->msg_receive_cb == NULL) {
		sysfs_notify(&mbox->dev.kobj, NULL, "mbox");
		return;
	}

	while (1) {
		rq = elv_next_request(q);
		if (!rq)
			break;

		msg = (mbox_msg_t)rq->data;

		blkdev_dequeue_request(rq);
		end_that_request_last(rq, 0);

		if (mbox->msg_receive_cb)
			mbox->msg_receive_cb(msg);
	}
}

/*
 * Mailbox interrupt handler
 */
static void mbox_txq_fn(request_queue_t *q)
{
	struct omap_mbox *mbox = q->queuedata;
	int ret;
	struct request *rq;

	while ((rq = elv_next_request(q)) != NULL) {
		ret = __mbox_msg_send(mbox, (mbox_msg_t)rq->data, rq->special);
		if (ret) {
			blk_stop_queue(q);
			enable_mbox_irq(mbox, IRQ_TX);
			return;
		}
		blkdev_dequeue_request(rq);
		end_that_request_last(rq, 0);
	}
}

static void mbox_rxq_fn(request_queue_t *q)
{
	struct request *rq;
	struct omap_mbox *mbox = q->queuedata;
	mbox_msg_t msg;

	while (!mbox_fifo_empty(mbox)) {
		rq = blk_get_request(q, WRITE, GFP_ATOMIC);
		if (unlikely(!rq))
			goto nomem;

		msg = mbox_fifo_read(mbox);
		rq->data = (void *)msg;

		if (unlikely(mbox_seq_test(mbox, msg))) {
			pr_info("mbox: Illegal seq bit!(%08x)\n", msg);
			if (mbox->err_notify)
				mbox->err_notify();
		}

		blk_insert_request(mbox->rxq, rq, 0, NULL);
		if (mbox->ops->type == OMAP_MBOX_TYPE1)
			break;
	}

	/* no more messages in the fifo. clear IRQ source. */
	ack_mbox_irq(mbox, IRQ_RX);
	enable_mbox_irq(mbox, IRQ_RX);
 nomem:
	queue_work(mbox->workq, &mbox->msg_receive);
}

static irqreturn_t mbox_interrupt(int irq, void *p)
{
	struct omap_mbox *mbox = (struct omap_mbox *)p;

	if (is_mbox_irq(mbox, IRQ_TX)) {
		disable_mbox_irq(mbox, IRQ_TX);
		ack_mbox_irq(mbox, IRQ_TX);
		blk_start_queue(mbox->txq);
	}

	if (is_mbox_irq(mbox, IRQ_RX)) {
		disable_mbox_irq(mbox, IRQ_RX);
		blk_start_queue(mbox->rxq);
	}

	return IRQ_HANDLED;
}

/*
 * sysfs files
 */
static ssize_t
omap_mbox_write(struct device *dev, struct device_attribute *attr,
		const char * buf, size_t count)
{
	int ret;
	mbox_msg_t *p = (mbox_msg_t *)buf;
	struct omap_mbox *mbox = dev_get_drvdata(dev);

	for (; count >= sizeof(mbox_msg_t); count -= sizeof(mbox_msg_t)) {
		ret = omap_mbox_msg_send(mbox, be32_to_cpu(*p), NULL);
		if (ret)
			return -EAGAIN;
		p++;
	}

	return  (size_t)((char *)p - buf);
}

static ssize_t
omap_mbox_read(struct device *dev, struct device_attribute *attr,
	       char * buf)
{
	struct request *rq;
	mbox_msg_t *p = (mbox_msg_t *)buf;
	struct omap_mbox *mbox = dev_get_drvdata(dev);
	struct request_queue *q = mbox->rxq;

	while ((rq = elv_next_request(q)) != NULL) {
		*p = (mbox_msg_t)rq->data;
		blkdev_dequeue_request(rq);
		end_that_request_last(rq, 0);

		if (unlikely(mbox_seq_test(mbox, *p))) {
			pr_info("mbox: Illegal seq bit!(%08x) ignored\n", *p);
			continue;
		}
		p++;
	}

	pr_debug("%02x %02x %02x %02x\n", buf[0], buf[1], buf[2], buf[3]);

	return (size_t)((char *)p - buf);
}

static DEVICE_ATTR(mbox, S_IRUGO | S_IWUSR, omap_mbox_read, omap_mbox_write);

static ssize_t mbox_show(struct class *class, char *buf)
{
	return sprintf(buf, "mbox");
}

static CLASS_ATTR(mbox, S_IRUGO, mbox_show, NULL);

static struct class omap_mbox_class = {
	.name = "omap_mbox",
};

static int omap_mbox_init(struct omap_mbox *mbox)
{
	int ret;
	request_queue_t *q;

	if (likely(mbox->ops->startup)) {
		ret = mbox->ops->startup(mbox);
		if (unlikely(ret))
			return ret;
	}

	mbox->dev.class = &omap_mbox_class;
	strlcpy(mbox->dev.bus_id, mbox->name, KOBJ_NAME_LEN);
	dev_set_drvdata(&mbox->dev, mbox);

	ret = device_register(&mbox->dev);
	if (unlikely(ret))
		goto fail_device_reg;

	ret = device_create_file(&mbox->dev, &dev_attr_mbox);
	if (unlikely(ret)) {
		printk(KERN_ERR
		       "device_create_file failed: %d\n", ret);
		goto fail_create_mbox;
	}

	spin_lock_init(&mbox->lock);
	INIT_WORK(&mbox->msg_receive, mbox_msg_receiver);

	ret = request_irq(mbox->irq, mbox_interrupt, IRQF_DISABLED,
			  mbox->name, mbox);
	if (unlikely(ret)) {
		printk(KERN_ERR
		       "failed to register mailbox interrupt:%d\n", ret);
		goto fail_request_irq;
	}
	enable_mbox_irq(mbox, IRQ_RX);

	q = blk_init_queue(mbox_txq_fn, &mbox->lock);
	if (!q)
		goto fail_blkinit_txq;
	mbox->txq = q;
	q->queuedata = mbox;

	q = blk_init_queue(mbox_rxq_fn, &mbox->lock);
	if (!q)
		goto fail_blkinit_rxq;
	mbox->rxq = q;
	q->queuedata = mbox;

	return 0;

 fail_blkinit_rxq:
	blk_cleanup_queue(mbox->txq);
 fail_blkinit_txq:
	free_irq(mbox->irq, mbox);
 fail_request_irq:
	device_remove_file(&mbox->dev, &dev_attr_mbox);
 fail_create_mbox:
	device_unregister(&mbox->dev);
 fail_device_reg:
	if (unlikely(mbox->ops->shutdown))
		mbox->ops->shutdown(mbox);

	return ret;
}

static void omap_mbox_fini(struct omap_mbox *mbox)
{
	blk_cleanup_queue(mbox->txq);
	blk_cleanup_queue(mbox->rxq);

	flush_workqueue(mbox->workq);
	destroy_workqueue(mbox->workq);

	free_irq(mbox->irq, mbox);
	device_remove_file(&mbox->dev, &dev_attr_mbox);
	class_unregister(&omap_mbox_class);

	if (unlikely(mbox->ops->shutdown))
		mbox->ops->shutdown(mbox);
}

static struct omap_mbox **find_mboxes(const char *name)
{
	struct omap_mbox **p;

	for (p = &mboxes; *p; p = &(*p)->next) {
		if (strcmp((*p)->name, name) == 0)
			break;
	}

	return p;
}

struct omap_mbox *omap_mbox_get(const char *name)
{
	struct omap_mbox *mbox;
	char queue_name[MBOX_NAME_LEN];
	int ret;

	read_lock(&mboxes_lock);
	mbox = *(find_mboxes(name));
	if (mbox == NULL) {
		read_unlock(&mboxes_lock);
		return ERR_PTR(-ENOENT);
	}

	strcpy(queue_name, "mailbox/");
	strcat(queue_name, mbox->name);
	mbox->workq = create_singlethread_workqueue(queue_name);

	read_unlock(&mboxes_lock);

	ret = omap_mbox_init(mbox);
	if (ret)
		return ERR_PTR(-ENODEV);

	return mbox;
}
EXPORT_SYMBOL(omap_mbox_get);

void omap_mbox_put(struct omap_mbox *mbox)
{
	omap_mbox_fini(mbox);
}
EXPORT_SYMBOL(omap_mbox_put);

int omap_mbox_register(struct omap_mbox *mbox)
{
	int ret = 0;
	struct omap_mbox **tmp;

	if (!mbox)
		return -EINVAL;
	if (mbox->next)
		return -EBUSY;

	write_lock(&mboxes_lock);
	tmp = find_mboxes(mbox->name);
	if (*tmp)
		ret = -EBUSY;
	else
		*tmp = mbox;
	write_unlock(&mboxes_lock);

	return ret;
}
EXPORT_SYMBOL(omap_mbox_register);

int omap_mbox_unregister(struct omap_mbox *mbox)
{
	struct omap_mbox **tmp;

	write_lock(&mboxes_lock);
	tmp = &mboxes;
	while (*tmp) {
		if (mbox == *tmp) {
			*tmp = mbox->next;
			mbox->next = NULL;
			write_unlock(&mboxes_lock);
			return 0;
		}
		tmp = &(*tmp)->next;
	}
	write_unlock(&mboxes_lock);

	return -EINVAL;
}
EXPORT_SYMBOL(omap_mbox_unregister);

static int __init omap_mbox_class_init(void)
{
	int ret = class_register(&omap_mbox_class);
	if (!ret)
		ret = class_create_file(&omap_mbox_class, &class_attr_mbox);

	return ret;
}

static void __exit omap_mbox_class_exit(void)
{
	class_remove_file(&omap_mbox_class, &class_attr_mbox);
	class_unregister(&omap_mbox_class);
}

subsys_initcall(omap_mbox_class_init);
module_exit(omap_mbox_class_exit);

MODULE_LICENSE("GPL");
