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
#include <linux/err.h>
#include <asm/io.h>
#include <asm/delay.h>
#include <asm/arch/mailbox.h>
#include "mailbox.h"

static struct omap_mbox *mboxes;
static DEFINE_RWLOCK(mboxes_lock);

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

	read_lock(&mboxes_lock);
	mbox = *(find_mboxes(name));
	read_unlock(&mboxes_lock);

	return mbox;
}
EXPORT_SYMBOL(omap_mbox_get);

/* Mailbox Sequence Bit function */
void omap_mbox_init_seq(struct omap_mbox *mbox)
{
	mbox_seq_init(mbox);
}
EXPORT_SYMBOL(omap_mbox_init_seq);

/*
 * message sender
 */
int omap_mbox_msg_send(struct omap_mbox *mbox, mbox_msg_t msg, void* arg)
{
	int ret;
	int i = 1000;
	static DEFINE_MUTEX(msg_send_lock);

	while (mbox_fifo_full(mbox)) {
		if (mbox->ops->type == OMAP_MBOX_TYPE2) {
			enable_mbox_irq(mbox, IRQ_TX);
			wait_event_interruptible(mbox->tx_waitq,
						 !mbox_fifo_full(mbox));
		} else
			udelay(1);

		if (--i == 0)
			return -1;
	}


	mutex_lock(&msg_send_lock);

	if (mbox->msg_sender_cb && arg) {
		ret = mbox->msg_sender_cb(arg);
		if (ret)
			goto out;
	}

	mbox_seq_toggle(mbox, &msg);
	mbox_fifo_write(mbox, msg);
 out:
	mutex_unlock(&msg_send_lock);

	return 0;
}
EXPORT_SYMBOL(omap_mbox_msg_send);

/*
 * Message receiver(workqueue)
 */
static void mbox_msg_receiver(void *p)
{
	struct omap_mbox *mbox = (struct omap_mbox *)p;
	struct omap_mbq *mbq = mbox->mbq;
	mbox_msg_t msg;

	while (!mbq_empty(mbq)) {
		msg = mbq_get(mbq);
		enable_mbox_irq(mbox, IRQ_RX);

		if (unlikely(mbox_seq_test(mbox, msg))) {
			printk(KERN_ERR
			       "mbox: illegal seq bit! ignoring this command. "
			       "(%08x)\n", msg);
			continue;
		}

		if (likely(mbox->msg_receive_cb))
			mbox->msg_receive_cb(msg);
	}
}

/*
 * Mailbox interrupt handler
 */
static irqreturn_t mbox_interrupt(int irq, void *p, struct pt_regs *regs)
{
	mbox_msg_t msg;
	struct omap_mbox *mbox = (struct omap_mbox *)p;

	if (is_mbox_irq(mbox, IRQ_TX)) {
		disable_mbox_irq(mbox, IRQ_TX);
		/*
		 * NOTE: this doesn't seeem to work as explained in the manual.
		 * IRQSTATUS:NOTFULL can't be cleared even we write 1 to that bit.
		 * It is always set when it's not full, regardless of IRQENABLE setting.
		 */
		ack_mbox_irq(mbox, IRQ_TX);
		wake_up_interruptible_all(&mbox->tx_waitq);
	}

	if (!is_mbox_irq(mbox, IRQ_RX))
		return IRQ_HANDLED;

	while (!mbox_fifo_empty(mbox)) {
		msg = mbox_fifo_read(mbox);
		if (mbq_add(mbox->mbq, msg)) {	/* mbq full */
			disable_mbox_irq(mbox, IRQ_RX);
			goto flush_queue;
		}
		if (mbox->ops->type == OMAP_MBOX_TYPE1)
			break;
	}

	/* no more messages in the fifo. clear IRQ source. */
	ack_mbox_irq(mbox, IRQ_RX);
 flush_queue:
	schedule_work(&mbox->msg_receive);

	return IRQ_HANDLED;
}

/*
 * sysfs files
 */
static ssize_t mbox_attr_write(struct class_device *dev, const char *buf,
			      size_t count)
{
	int ret;
	mbox_msg_t msg;
	struct omap_mbox *mbox = class_get_devdata(dev);

	msg = (mbox_msg_t) simple_strtoul(buf, NULL, 16);

	ret = omap_mbox_msg_send(mbox, msg, NULL);
	if (ret)
		return -1;

	return count;
}

static ssize_t mbox_attr_read(struct class_device *dev, char *buf)
{
	struct omap_mbox *mbox = class_get_devdata(dev);

	return sprintf(buf, mbox->name);
}

static CLASS_DEVICE_ATTR(mbox, S_IALLUGO, mbox_attr_read, mbox_attr_write);

static ssize_t mbox_show(struct class *class, char *buf)
{
	return sprintf(buf, "mbox");
}

static CLASS_ATTR(mbox, S_IRUGO, mbox_show, NULL);

static struct class omap_mbox_class = {
	.name = "mbox",
};

static int omap_mbox_init(struct omap_mbox *mbox)
{
	int ret;

	if (likely(mbox->ops->startup)) {
		ret = mbox->ops->startup(mbox);
		if (unlikely(ret))
			return ret;
	}

	mbox->class_dev.class = &omap_mbox_class;
	strlcpy(mbox->class_dev.class_id, mbox->name, KOBJ_NAME_LEN);
	class_set_devdata(&mbox->class_dev, mbox);

	ret = class_device_register(&mbox->class_dev);
	if (unlikely(ret))
		return ret;

	class_device_create_file(&mbox->class_dev, &class_device_attr_mbox);

	ret = request_irq(mbox->irq, mbox_interrupt, SA_INTERRUPT,
			  mbox->name, mbox);
	if (unlikely(ret)) {
		printk(KERN_ERR
		       "failed to register mailbox interrupt:%d\n", ret);
		goto fail1;
	}
	enable_mbox_irq(mbox, IRQ_RX);

	spin_lock_init(&mbox->lock);
	INIT_WORK(&mbox->msg_receive, mbox_msg_receiver, mbox);
	init_waitqueue_head(&mbox->tx_waitq);

	ret = mbq_init(&mbox->mbq);
	if (unlikely(ret))
		goto fail2;

	return 0;
 fail2:
	free_irq(mbox->irq, mbox);
	class_remove_file(&omap_mbox_class, &class_attr_mbox);
	class_unregister(&omap_mbox_class);
 fail1:
	if (unlikely(mbox->ops->shutdown))
		mbox->ops->shutdown(mbox);

	return ret;
}

static void omap_mbox_shutdown(struct omap_mbox *mbox)
{
	free_irq(mbox->irq, mbox);
	class_remove_file(&omap_mbox_class, &class_attr_mbox);
	class_unregister(&omap_mbox_class);

	if (unlikely(mbox->ops->shutdown))
		mbox->ops->shutdown(mbox);
}

int omap_mbox_register(struct omap_mbox *mbox)
{
	int ret = 0;
	struct omap_mbox **tmp;

	if (!mbox)
		return -EINVAL;
	if (mbox->next)
		return -EBUSY;

	ret = omap_mbox_init(mbox);
	if (ret)
		return ret;

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

			omap_mbox_shutdown(mbox);

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
