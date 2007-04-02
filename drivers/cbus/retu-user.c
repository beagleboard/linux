/**
 * drivers/cbus/retu-user.c
 *
 * Retu user space interface functions
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

#include "retu.h"

#include "user_retu_tahvo.h"

/* Maximum size of IRQ node buffer/pool */
#define RETU_MAX_IRQ_BUF_LEN	16

#define PFX			"retu-user: "

/* Bitmap for marking the interrupt sources as having the handlers */
static u32 retu_irq_bits;

/* For allowing only one user process to subscribe to the retu interrupts */
static struct file *retu_irq_subscr = NULL;

/* For poll and IRQ passing */
struct retu_irq {
	u32 id;
	struct list_head node;
};

static spinlock_t retu_irqs_lock;
static struct retu_irq *retu_irq_block;
static LIST_HEAD(retu_irqs);
static LIST_HEAD(retu_irqs_reserve);

/* Wait queue - used when user wants to read the device */
DECLARE_WAIT_QUEUE_HEAD(retu_user_waitqueue);

/* Semaphore to protect irq subscription sequence */
static struct mutex retu_mutex;

/* This array specifies RETU register types (read/write/toggle) */
static const u8 retu_access_bits[] = {
	1,
	4,
	3,
	3,
	1,
	3,
	3,
	0,
	3,
	3,
	3,
	3,
	3,
	3,
	3,
	4,
	4,
	3,
	0,
	0,
	0,
	0,
	1,
	3,
	3,
	3,
	3,
	3,
	3,
	3,
	3,
	3
};

/*
 * The handler for all RETU interrupts.
 *
 * arg is the interrupt source in RETU.
 */
static void retu_user_irq_handler(unsigned long arg)
{
	struct retu_irq *irq;

	retu_ack_irq(arg);

	spin_lock(&retu_irqs_lock);
	if (list_empty(&retu_irqs_reserve)) {
		spin_unlock(&retu_irqs_lock);
		return;
	}
	irq = list_entry((&retu_irqs_reserve)->next, struct retu_irq, node);
	irq->id = arg;
	list_move_tail(&irq->node, &retu_irqs);
	spin_unlock(&retu_irqs_lock);

	/* wake up waiting thread */
	wake_up(&retu_user_waitqueue);
}

/*
 * This routine sets up the interrupt handler and marks an interrupt source
 * in RETU as a candidate for signal delivery to the user process.
 */
static int retu_user_subscribe_to_irq(int id, struct file *filp)
{
	int ret;

	mutex_lock(&retu_mutex);
	if ((retu_irq_subscr != NULL) && (retu_irq_subscr != filp)) {
		mutex_unlock(&retu_mutex);
		return -EBUSY;
	}
	/* Store the file pointer of the first user process registering IRQs */
	retu_irq_subscr = filp;
	mutex_unlock(&retu_mutex);

	if (retu_irq_bits & (1 << id))
		return 0;

	ret = retu_request_irq(id, retu_user_irq_handler, id, "");
	if (ret < 0)
		return ret;

	/* Mark that this interrupt has a handler */
	retu_irq_bits |= 1 << id;

	return 0;
}

/*
 * Unregisters all RETU interrupt handlers.
 */
static void retu_unreg_irq_handlers(void)
{
	int id;

	if (!retu_irq_bits)
		return;

	for (id = 0; id < MAX_RETU_IRQ_HANDLERS; id++)
		if (retu_irq_bits & (1 << id))
			retu_free_irq(id);

	retu_irq_bits = 0;
}

/*
 * Write to RETU register.
 * Returns 0 upon success, a negative error value otherwise.
 */
static int retu_user_write_with_mask(u32 field, u16 value)
{
	u32 mask;
	u32 reg;
	u_short tmp;
	unsigned long flags;

	mask = MASK(field);
	reg = REG(field);

	/* Detect bad mask and reg */
	if (mask == 0 || reg > RETU_REG_MAX ||
	    retu_access_bits[reg] == READ_ONLY) {
		printk(KERN_ERR PFX "invalid arguments (reg=%#x, mask=%#x)\n",
		       reg, mask);
		return -EINVAL;
	}

	/* Justify value according to mask */
	while (!(mask & 1)) {
		value = value << 1;
		mask = mask >> 1;
	}

	spin_lock_irqsave(&retu_lock, flags);
	if (retu_access_bits[reg] == TOGGLE) {
		/* No need to detect previous content of register */
		tmp = 0;
	} else {
		/* Read current value of register */
		tmp = retu_read_reg(reg);
	}

	/* Generate new value */
	tmp = (tmp & ~MASK(field)) | (value & MASK(field));
	/* Write data to RETU */
	retu_write_reg(reg, tmp);
	spin_unlock_irqrestore(&retu_lock, flags);

	return 0;
}

/*
 * Read RETU register.
 */
static u32 retu_user_read_with_mask(u32 field)
{
	u_short value;
	u32 mask, reg;

	mask = MASK(field);
	reg = REG(field);

	/* Detect bad mask and reg */
	if (mask == 0 || reg > RETU_REG_MAX) {
		printk(KERN_ERR PFX "invalid arguments (reg=%#x, mask=%#x)\n",
		       reg, mask);
		return -EINVAL;
	}

	/* Read the register */
	value = retu_read_reg(reg) & mask;

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
static int retu_close(struct inode *inode, struct file *filp)
{
	/* Unregister all interrupts that have been registered */
	if (retu_irq_subscr == filp) {
		retu_unreg_irq_handlers();
		retu_irq_subscr = NULL;
	}

	return 0;
}

/*
 * Device control (ioctl)
 */
static int retu_ioctl(struct inode *inode, struct file *filp,
		      unsigned int cmd, unsigned long arg)
{
	struct retu_tahvo_write_parms par;
	int ret;

	switch (cmd) {
	case URT_IOCT_IRQ_SUBSCR:
		return retu_user_subscribe_to_irq(arg, filp);
	case RETU_IOCH_READ:
		return retu_user_read_with_mask(arg);
	case RETU_IOCX_WRITE:
		ret = copy_from_user(&par, (void __user *) arg, sizeof(par));
		if (ret)
			printk(KERN_ERR "copy_from_user failed: %d\n", ret);
		par.result = retu_user_write_with_mask(par.field, par.value);
		ret = copy_to_user((void __user *) arg, &par, sizeof(par));
		if (ret)
			printk(KERN_ERR "copy_to_user failed: %d\n", ret);
		break;
	case RETU_IOCH_ADC_READ:
		return retu_read_adc(arg);
	default:
		return -ENOIOCTLCMD;
	}
	return 0;
}

/*
 * Read from device
 */
static ssize_t retu_read(struct file *filp, char *buf, size_t count,
			 loff_t * offp)
{
	struct retu_irq *irq;

	u32 nr, i;

	/* read not permitted if neither filp nor anyone has registered IRQs */
	if (retu_irq_subscr != filp)
		return -EPERM;

	if ((count < sizeof(u32)) || ((count % sizeof(u32)) != 0))
		return -EINVAL;

	nr = count / sizeof(u32);

	for (i = 0; i < nr; i++) {
		unsigned long flags;
		u32 irq_id;
		int ret;

		ret = wait_event_interruptible(retu_user_waitqueue,
					       !list_empty(&retu_irqs));
		if (ret < 0)
			return ret;

		spin_lock_irqsave(&retu_irqs_lock, flags);
		irq = list_entry((&retu_irqs)->next, struct retu_irq, node);
		irq_id = irq->id;
		list_move(&irq->node, &retu_irqs_reserve);
		spin_unlock_irqrestore(&retu_irqs_lock, flags);

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
static unsigned retu_poll(struct file *filp, struct poll_table_struct *pt)
{
	if (!list_empty(&retu_irqs))
		return POLLIN;

	poll_wait(filp, &retu_user_waitqueue, pt);

	if (!list_empty(&retu_irqs))
		return POLLIN;
	else
		return 0;
}

static struct file_operations retu_user_fileops = {
	.owner = THIS_MODULE,
	.ioctl = retu_ioctl,
	.read = retu_read,
	.release = retu_close,
	.poll = retu_poll
};

static struct miscdevice retu_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "retu",
	.fops = &retu_user_fileops
};

/*
 * Initialization
 *
 * @return 0 if successful, error value otherwise.
 */
int retu_user_init(void)
{
	struct retu_irq *irq;
	int res, i;

	irq = kmalloc(sizeof(*irq) * RETU_MAX_IRQ_BUF_LEN, GFP_KERNEL);
	if (irq == NULL) {
		printk(KERN_ERR PFX "kmalloc failed\n");
		return -ENOMEM;
	}
	memset(irq, 0, sizeof(*irq) * RETU_MAX_IRQ_BUF_LEN);
	for (i = 0; i < RETU_MAX_IRQ_BUF_LEN; i++)
		list_add(&irq[i].node, &retu_irqs_reserve);

	retu_irq_block = irq;

	spin_lock_init(&retu_irqs_lock);
	mutex_init(&retu_mutex);

	/* Request a misc device */
	res = misc_register(&retu_device);
	if (res < 0) {
		printk(KERN_ERR PFX "unable to register misc device for %s\n",
		       retu_device.name);
		kfree(irq);
		return res;
	}

	return 0;
}

/*
 * Cleanup.
 */
void retu_user_cleanup(void)
{
	/* Unregister our misc device */
	misc_deregister(&retu_device);
	/* Unregister and disable all RETU interrupts used by this module */
	retu_unreg_irq_handlers();
	kfree(retu_irq_block);
}

MODULE_DESCRIPTION("Retu ASIC user space functions");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mikko Ylinen");
