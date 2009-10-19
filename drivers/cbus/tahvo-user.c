/**
 * drivers/cbus/tahvo-user.c
 *
 * Tahvo user space interface functions
 *
 * Copyright (C) 2004, 2005 Nokia Corporation
 *
 * Written by Mikko Ylinen <mikko.k.ylinen@nokia.com>
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License. See the file "COPYING" in the main directory of this
 * archive for more details.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>

#include <asm/uaccess.h>

#include "tahvo.h"

#include "user_retu_tahvo.h"

/* Maximum size of IRQ node buffer/pool */
#define TAHVO_MAX_IRQ_BUF_LEN	16

#define PFX			"tahvo-user: "

/* Bitmap for marking the interrupt sources as having the handlers */
static u32 tahvo_irq_bits;

/* For allowing only one user process to subscribe to the tahvo interrupts */
static struct file *tahvo_irq_subscr = NULL;

/* For poll and IRQ passing */
struct tahvo_irq {
	u32 id;
	struct list_head node;
};

static spinlock_t tahvo_irqs_lock;
static struct tahvo_irq *tahvo_irq_block;
static LIST_HEAD(tahvo_irqs);
static LIST_HEAD(tahvo_irqs_reserve);

/* Wait queue - used when user wants to read the device */
DECLARE_WAIT_QUEUE_HEAD(tahvo_user_waitqueue);

/* Semaphore to protect irq subscription sequence */
static struct mutex tahvo_mutex;

/* This array specifies TAHVO register types (read/write/toggle) */
static const u8 tahvo_access_bits[] = {
	1,
	4,
	1,
	3,
	3,
	3,
	3,
	3,
	3,
	3,
	3,
	3,
	3,
	1
};

/*
 * The handler for all TAHVO interrupts.
 *
 * arg is the interrupt source in TAHVO.
 */
static void tahvo_user_irq_handler(unsigned long arg)
{
	struct tahvo_irq *irq;

	/* user has to re-enable the interrupt once ready
	 * for receiving them again */
	tahvo_disable_irq(arg);
	tahvo_ack_irq(arg);

	spin_lock(&tahvo_irqs_lock);
	if (list_empty(&tahvo_irqs_reserve)) {
		spin_unlock(&tahvo_irqs_lock);
		return;
	}
	irq = list_entry((&tahvo_irqs_reserve)->next, struct tahvo_irq, node);
	irq->id = arg;
	list_move_tail(&irq->node, &tahvo_irqs);
	spin_unlock(&tahvo_irqs_lock);

	/* wake up waiting thread */
	wake_up(&tahvo_user_waitqueue);
}

/*
 * This routine sets up the interrupt handler and marks an interrupt source
 * in TAHVO as a candidate for signal delivery to the user process.
 */
static int tahvo_user_subscribe_to_irq(int id, struct file *filp)
{
	int ret;

	mutex_lock(&tahvo_mutex);
	if ((tahvo_irq_subscr != NULL) && (tahvo_irq_subscr != filp)) {
		mutex_unlock(&tahvo_mutex);
		return -EBUSY;
	}
	/* Store the file pointer of the first user process registering IRQs */
	tahvo_irq_subscr = filp;
	mutex_unlock(&tahvo_mutex);

	if (tahvo_irq_bits & (1 << id))
		return 0;

	ret = tahvo_request_irq(id, tahvo_user_irq_handler, id, "");
	if (ret < 0)
		return ret;

	/* Mark that this interrupt has a handler */
	tahvo_irq_bits |= 1 << id;

	return 0;
}

/*
 * Unregister all TAHVO interrupt handlers
 */
static void tahvo_unreg_irq_handlers(void)
{
	int id;

	if (!tahvo_irq_bits)
		return;

	for (id = 0; id < MAX_TAHVO_IRQ_HANDLERS; id++)
		if (tahvo_irq_bits & (1 << id))
			tahvo_free_irq(id);

	tahvo_irq_bits = 0;
}

/*
 * Write to TAHVO register.
 * Returns 0 upon success, a negative error value otherwise.
 */
static int tahvo_user_write_with_mask(u32 field, u16 value)
{
	u32 mask;
	u32 reg;
	u_short tmp;
	unsigned long flags;

	mask = MASK(field);
	reg = REG(field);

	/* Detect bad mask and reg */
	if (mask == 0 || reg > TAHVO_REG_MAX ||
	    tahvo_access_bits[reg] == READ_ONLY) {
		printk(KERN_ERR PFX "invalid arguments (reg=%#x, mask=%#x)\n",
		       reg, mask);
		return -EINVAL;
	}

	/* Justify value according to mask */
	while (!(mask & 1)) {
		value = value << 1;
		mask = mask >> 1;
	}

	spin_lock_irqsave(&tahvo_lock, flags);
	if (tahvo_access_bits[reg] == TOGGLE) {
		/* No need to detect previous content of register */
		tmp = 0;
	} else {
		/* Read current value of register */
		tmp = tahvo_read_reg(reg);
	}
	/* Generate a new value */
	tmp = (tmp & ~MASK(field)) | (value & MASK(field));
	/* Write data to TAHVO */
	tahvo_write_reg(reg, tmp);
	spin_unlock_irqrestore(&tahvo_lock, flags);

	return 0;
}

/*
 * Read TAHVO register.
 */
static u32 tahvo_user_read_with_mask(u32 field)
{
	u_short value;
	u32 mask, reg;

	mask = MASK(field);
	reg = REG(field);

	/* Detect bad mask and reg */
	if (mask == 0 || reg > TAHVO_REG_MAX) {
		printk(KERN_ERR PFX "invalid arguments (reg=%#x, mask=%#x)\n",
		       reg, mask);
		return -EINVAL;
	}

	/* Read the register */
	value = tahvo_read_reg(reg) & mask;

	/* Right justify value */
	while (!(mask & 1)) {
		value = value >> 1;
		mask = mask >> 1;
	}

	return value;
}

/*
 * Close device
 */
static int tahvo_close(struct inode *inode, struct file *filp)
{
	/* Unregister all interrupts that have been registered */
	if (tahvo_irq_subscr == filp) {
		tahvo_unreg_irq_handlers();
		tahvo_irq_subscr = NULL;
	}

	return 0;
}

/*
 * Device control (ioctl)
 */
static int tahvo_ioctl(struct inode *inode, struct file *filp,
		       unsigned int cmd, unsigned long arg)
{
	struct retu_tahvo_write_parms par;
	int ret;

	switch (cmd) {
	case URT_IOCT_IRQ_SUBSCR:
		return tahvo_user_subscribe_to_irq(arg, filp);
	case TAHVO_IOCH_READ:
		return tahvo_user_read_with_mask(arg);
	case TAHVO_IOCX_WRITE:
		ret = copy_from_user(&par, (void __user *) arg, sizeof(par));
		if (ret)
			printk(KERN_ERR "copy_from_user failed: %d\n", ret);
		par.result = tahvo_user_write_with_mask(par.field, par.value);
		ret = copy_to_user((void __user *) arg, &par, sizeof(par));
		if (ret)
			printk(KERN_ERR "copy_to_user failed: %d\n", ret);
		break;
	default:
		return -ENOIOCTLCMD;
	}
	return 0;
}

/*
 * Read from device
 */
static ssize_t tahvo_read(struct file *filp, char *buf, size_t count,
			  loff_t * offp)
{
	struct tahvo_irq *irq;

	u32 nr, i;

	/* read not permitted if neither filp nor anyone has registered IRQs */
	if (tahvo_irq_subscr != filp)
		return -EPERM;

	if ((count < sizeof(u32)) || ((count % sizeof(u32)) != 0))
		return -EINVAL;

	nr = count / sizeof(u32);

	for (i = 0; i < nr; i++) {
		unsigned long flags;
		u32 irq_id;
		int ret;

		ret = wait_event_interruptible(tahvo_user_waitqueue,
					       !list_empty(&tahvo_irqs));
		if (ret < 0)
			return ret;

		spin_lock_irqsave(&tahvo_irqs_lock, flags);
		irq = list_entry((&tahvo_irqs)->next, struct tahvo_irq, node);
		irq_id = irq->id;
		list_move(&irq->node, &tahvo_irqs_reserve);
		spin_unlock_irqrestore(&tahvo_irqs_lock, flags);

		ret = copy_to_user(buf + i * sizeof(irq_id), &irq_id,
                                  sizeof(irq_id));
		if (ret)
			printk(KERN_ERR "copy_to_user failed: %d\n", ret);
	}

	return count;
}

/*
 * Poll method
 */
static unsigned tahvo_poll(struct file *filp, struct poll_table_struct *pt)
{
	if (!list_empty(&tahvo_irqs))
		return POLLIN;

	poll_wait(filp, &tahvo_user_waitqueue, pt);

	if (!list_empty(&tahvo_irqs))
		return POLLIN;
	else
		return 0;
}

static struct file_operations tahvo_user_fileops = {
	.owner = THIS_MODULE,
	.ioctl = tahvo_ioctl,
	.read = tahvo_read,
	.release = tahvo_close,
	.poll = tahvo_poll
};

static struct miscdevice tahvo_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "tahvo",
	.fops = &tahvo_user_fileops
};

/*
 * Initialization
 *
 * @return 0 if successful, error value otherwise.
 */
int tahvo_user_init(void)
{
	struct tahvo_irq *irq;
	int res, i;

	irq = kmalloc(sizeof(*irq) * TAHVO_MAX_IRQ_BUF_LEN, GFP_KERNEL);
	if (irq == NULL) {
		printk(KERN_ERR PFX "kmalloc failed\n");
		return -ENOMEM;
	}
	memset(irq, 0, sizeof(*irq) * TAHVO_MAX_IRQ_BUF_LEN);
	for (i = 0; i < TAHVO_MAX_IRQ_BUF_LEN; i++)
		list_add(&irq[i].node, &tahvo_irqs_reserve);

	tahvo_irq_block = irq;

	spin_lock_init(&tahvo_irqs_lock);
	mutex_init(&tahvo_mutex);

	/* Request a misc device */
	res = misc_register(&tahvo_device);
	if (res < 0) {
		printk(KERN_ERR PFX "unable to register misc device for %s\n",
		       tahvo_device.name);
		kfree(irq);
		return res;
	}

	return 0;
}

/*
 * Cleanup.
 */
void tahvo_user_cleanup(void)
{
	/* Unregister our misc device */
	misc_deregister(&tahvo_device);
	/* Unregister and disable all TAHVO interrupts */
	tahvo_unreg_irq_handlers();
	kfree(tahvo_irq_block);
}

MODULE_DESCRIPTION("Tahvo ASIC user space functions");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mikko Ylinen");
