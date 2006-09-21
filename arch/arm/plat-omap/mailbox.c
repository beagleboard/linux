/*
 * OMAP mailbox driver
 *
 * Copyright (C) 2006 Nokia Corporation. All rights reserved.
 *
 * Contact: Toshihiro Kobayashi <toshihiro.kobayashi@nokia.com>
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
#include <linux/err.h>
#ifdef CONFIG_ARCH_OMAP2
#include <linux/clk.h>
#endif
#include <asm/io.h>
#ifdef CONFIG_ARCH_OMAP1
#include <asm/delay.h>
#endif
#include <asm/arch/mailbox.h>
#include "mailbox_hw.h"

#if defined(CONFIG_ARCH_OMAP1)

#define read_mbx(m, msgp) \
		do { \
			*(msgp) = omap_readw((m)->data_r); \
			*(msgp) |= ((mbx_msg_t)omap_readw((m)->cmd_r)) << 16; \
		} while (0)
#define write_mbx(m, msg) \
		do { \
			omap_writew((msg) & 0xffff, (m)->data_w); \
			omap_writew((msg) >> 16, (m)->cmd_w); \
		} while (0)
#define enable_newmsg_irq(m)	enable_irq((m)->irq)
#define disable_newmsg_irq(m)	disable_irq((m)->irq)
#define mbx_is_notfull(m)	(omap_readw((m)->flag_w) == 0)

#elif defined(CONFIG_ARCH_OMAP2)

#define omap_bit_setl(b,r) \
		do { omap_writel(omap_readl(r) | (b), (r)); } while(0)
#define omap_bit_clrl(b,r) \
		do { omap_writel(omap_readl(r) & ~(b), (r)); } while(0)

#define read_mbx(m, msgp) \
		do { *(msgp) = omap_readl((m)->message_r); } while (0)
#define write_mbx(m, msg)	omap_writel(msg, (m)->message_w)
#define enable_newmsg_irq(m)	omap_bit_setl((m)->newmsg_bit, (m)->irqenable)
#define disable_newmsg_irq(m)	omap_bit_clrl((m)->newmsg_bit, (m)->irqenable)
#define enable_notfull_irq(m)	omap_bit_setl((m)->notfull_bit, (m)->irqenable)
#define disable_notfull_irq(m)	omap_bit_clrl((m)->notfull_bit, (m)->irqenable)
#define clear_newmsg_irq(m)	omap_writel((m)->newmsg_bit, (m)->irqstatus)
#define clear_notfull_irq(m)	omap_writel((m)->notfull_bit, (m)->irqstatus)
#define notfull_irq_enabled(m)	(omap_readl((m)->irqenable) & (m)->notfull_bit)
#define has_newmsg_irq(m)	(omap_readl((m)->irqstatus) & (m)->newmsg_bit)
#define has_notfull_irq(m)	(omap_readl((m)->irqstatus) & (m)->notfull_bit)
#define mbx_nomsg(m)		(omap_readl((m)->msgstatus_r) == 0)
#define mbx_is_notfull(m)	(omap_readl((m)->fifostatus_w) == 0)

#endif /* CONFIG_ARCH_OMAP2 */

static void do_mbx(void *p);

#define MBQ_DEPTH	16
struct mbq {
	mbx_msg_t msg[MBQ_DEPTH];
	int rp, wp, full;
};

#define mbq_inc(p)	do { if (++(p) == MBQ_DEPTH) (p) = 0; } while(0)

#if defined(CONFIG_ARCH_OMAP1)
#  define MBX_USE_SEQ_BIT	/* XXX */
#elif defined(CONFIG_ARCH_OMAP2)
#  undef MBX_USE_SEQ_BIT
#endif

struct mbx {
	char *name;
	unsigned int irq;
	char irq_devid_newmsg;
#ifdef CONFIG_ARCH_OMAP2
	char irq_devid_notfull;
#endif
#ifdef MBX_USE_SEQ_BIT
	mbx_msg_t seq_snd;
	mbx_msg_t seq_rcv;
		/* seq_rcv should be initialized with any value other than
		 * 0 and 1 << 31, to allow either value for the first
		 * message.  */
#endif
	mbx_receiver_t *receiver_map[MBX_CMD_MAX];
	struct work_struct work;
	struct mbq mbq;
#ifdef CONFIG_ARCH_OMAP2
	wait_queue_head_t full_wait_q;
#endif

#if defined(CONFIG_ARCH_OMAP1)
	void *cmd_w;
	void *data_w;
	void *flag_w;
	void *cmd_r;
	void *data_r;
#elif defined(CONFIG_ARCH_OMAP2)
	void *irqenable;
	void *irqstatus;
	void *message_w;
	void *message_r;
	void *fifostatus_w;
	void *msgstatus_r;
	u32 notfull_bit;
	u32 newmsg_bit;
#endif
};

#if defined(CONFIG_ARCH_OMAP1)

#if defined(CONFIG_ARCH_OMAP15XX)
#define INT_DSP_MAILBOX1	INT_1510_DSP_MAILBOX1
#elif defined(CONFIG_ARCH_OMAP16XX)
#define INT_DSP_MAILBOX1	INT_1610_DSP_MAILBOX1
#endif

static struct mbx mbx_dsp = {
	.name = "DSP",
	.irq = INT_DSP_MAILBOX1,
	.work = __WORK_INITIALIZER(mbx_dsp.work, do_mbx, &mbx_dsp),

	.cmd_w  = (void *)MAILBOX_ARM2DSP1b,
	.data_w = (void *)MAILBOX_ARM2DSP1,
	.flag_w = (void *)MAILBOX_ARM2DSP1_Flag,
	.cmd_r  = (void *)MAILBOX_DSP2ARM1b,
	.data_r = (void *)MAILBOX_DSP2ARM1,
};

#elif defined(CONFIG_ARCH_OMAP2)

/*
 * MAILBOX 0: ARM -> DSP,
 * MAILBOX 1: ARM <- DSP.
 * MAILBOX 2: ARM -> IVA,
 * MAILBOX 3: ARM <- IVA.
 */
static struct mbx mbx_dsp = {
	.name = "DSP",
	.irq = INT_24XX_MAIL_U0_MPU,
	.work = __WORK_INITIALIZER(mbx_dsp.work, do_mbx, &mbx_dsp),
	.full_wait_q = __WAIT_QUEUE_HEAD_INITIALIZER(mbx_dsp.full_wait_q),

	.irqenable    = (void *)MAILBOX_IRQENABLE_0,
	.irqstatus    = (void *)MAILBOX_IRQSTATUS_0,
	.message_w    = (void *)MAILBOX_MESSAGE_0,
	.message_r    = (void *)MAILBOX_MESSAGE_1,
	.fifostatus_w = (void *)MAILBOX_FIFOSTATUS_0,
	.msgstatus_r  = (void *)MAILBOX_MSGSTATUS_1,
	.notfull_bit  = MAILBOX_IRQ_NOTFULL(0),
	.newmsg_bit   = MAILBOX_IRQ_NEWMSG(1),
};
static struct mbx mbx_iva = {
	.name = "IVA",
	.irq = INT_24XX_MAIL_U3_MPU,
	.work = __WORK_INITIALIZER(mbx_iva.work, do_mbx, &mbx_iva),
	.full_wait_q = __WAIT_QUEUE_HEAD_INITIALIZER(mbx_iva.full_wait_q),

	.irqenable    = (void *)MAILBOX_IRQENABLE_3,
	.irqstatus    = (void *)MAILBOX_IRQSTATUS_3,
	.message_w    = (void *)MAILBOX_MESSAGE_2,
	.message_r    = (void *)MAILBOX_MESSAGE_3,
	.fifostatus_w = (void *)MAILBOX_FIFOSTATUS_2,
	.msgstatus_r  = (void *)MAILBOX_MSGSTATUS_3,
	.notfull_bit  = MAILBOX_IRQ_NOTFULL(2),
	.newmsg_bit   = MAILBOX_IRQ_NEWMSG(3),
};

#endif /* CONFIG_ARCH_OMAP2 */

static struct mbx *mbxes[] = {
	&mbx_dsp,
#ifdef CONFIG_ARCH_OMAP2
	&mbx_iva,
#endif
};

struct mbx *mbx_get(const char *id)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mbxes); i++) {
		if (!strcmp(id, mbxes[i]->name))
			return mbxes[i];
	}

	return ERR_PTR(-ENOENT);
}

#if defined(CONFIG_ARCH_OMAP1)
static __inline__ int mbsync_irq_save(struct mbx *mbx, unsigned long *flags,
				      int try_cnt)
{
	int cnt;

	local_irq_save(*flags);
	if (mbx_is_notfull(mbx))
		return 0;
	/*
	 * mailbox is busy. wait for some usecs...
	 */
	local_irq_restore(*flags);
	for (cnt = 0; cnt < try_cnt; cnt++) {
		udelay(1);
		local_irq_save(*flags);
		if (mbx_is_notfull(mbx))	/* success! */
			return 0;
		local_irq_restore(*flags);
	}

	/* fail! */
	return -1;
}
#elif defined(CONFIG_ARCH_OMAP2)
static __inline__ int mbsync_irq_save(struct mbx *mbx, unsigned long *flags)
{
	long current_state;
	DECLARE_WAITQUEUE(wait, current);

	do {
		local_irq_save(*flags);
		if (mbx_is_notfull(mbx))
			return 0;

		/*
		 * mailbox is busy.
		 */
		local_irq_restore(*flags);
		enable_notfull_irq(mbx);

		/* wait until the FIFO becomes not-full */
		add_wait_queue(&mbx->full_wait_q, &wait);
		current_state = current->state;
		set_current_state(TASK_INTERRUPTIBLE);
		if (!mbx_is_notfull(mbx))	/* last check */
			schedule();
		set_current_state(current_state);
		remove_wait_queue(&mbx->full_wait_q, &wait);

		if (signal_pending(current))
			return -1;
	} while (1);
}
#endif

/*
 * message dispatcher API
 */
int mbx_send(struct mbx *mbx, mbx_msg_t msg)
{
	unsigned long flags;

#if defined(CONFIG_ARCH_OMAP1)
	/*
	 * DSP mailbox interrupt latency must be less than 1ms.
	 */
	if (mbsync_irq_save(mbx, &flags, 1000) < 0) {
		printk(KERN_ERR
		       "mailbox(%s) is busy. message 0x%08x is aborting.\n",
		       mbx->name, msg);
		return -1;
	}
#elif defined(CONFIG_ARCH_OMAP2)
	if (mbsync_irq_save(mbx, &flags) < 0)
		return -1;
#endif

#ifdef MBX_USE_SEQ_BIT
	/* add seq_snd to msg */
	msg = (msg & 0x7fffffff) | mbx->seq_snd;
	/* flip seq_snd */
	mbx->seq_snd ^= 1 << 31;
#endif

	write_mbx(mbx, msg);

	local_irq_restore(flags);
	return 0;
}

/*
 * register / unregister API
 */
int register_mbx_receiver(struct mbx *mbx, unsigned char cmd,
			  mbx_receiver_t *rcv)
{
	if (cmd >= MBX_CMD_MAX) {
		printk(KERN_ERR "register_mbx_receiver(): "
		       "bad cmd (0x%x)\n", cmd);
		return -EINVAL;
	}
	if (mbx->receiver_map[cmd] != NULL) {
		printk(KERN_ERR "register_mbx_receiver(): cmd 0x%x is "
		       "already reserved.\n", cmd);
		return -EINVAL;
	}

	mbx->receiver_map[cmd] = rcv;
	return 0;
}

int unregister_mbx_receiver(struct mbx *mbx, unsigned char cmd,
			    mbx_receiver_t *rcv)
{
	if (cmd >= MBX_CMD_MAX) {
		printk(KERN_ERR "unregister_mbx_receiver(): "
		       "bad cmd (0x%x)\n", cmd);
		return -EINVAL;
	}
	if (mbx->receiver_map[cmd] != rcv) {
		printk(KERN_ERR "unregister_mbx_receiver(): cmd 0x%x and "
		       "receiver function mismatch!\n", cmd);
		return -EINVAL;
	}

	mbx->receiver_map[cmd] = NULL;
	return 0;
}

/*
 * IRQ disable / enable API
 */
void disable_mbx_irq(struct mbx *mbx)
{
	disable_irq(mbx->irq);
}

void enable_mbx_irq(struct mbx *mbx)
{
	enable_irq(mbx->irq);
}

/*
 * init_seq API
 */
void mbx_init_seq(struct mbx *mbx)
{
#ifdef MBX_USE_SEQ_BIT
	/* backward compatibility */
	mbx->seq_snd = 0x80000000;

	/* any value other than 0 and 1 << 31 */
	mbx->seq_rcv = 0xffffffff;
#endif /* MBX_USE_SEQ_BIT */
}

/*
 * receiver workqueue
 */
static void do_mbx(void *p)
{
	int empty = 0;
	struct mbx *mbx = (struct mbx *)p;
	struct mbq *mbq = &mbx->mbq;
	mbx_receiver_t *receiver;
	mbx_msg_t msg;
#ifdef MBX_USE_SEQ_BIT
	mbx_msg_t seq;
#endif

	disable_newmsg_irq(mbx);
	if ((mbq->rp == mbq->wp) && !mbq->full)
		empty = 1;
	enable_newmsg_irq(mbx);

	while (!empty) {
		msg = mbq->msg[mbq->rp];
#ifdef MBX_USE_SEQ_BIT
		seq = msg & (1 << 31);

		if (seq == mbx->seq_rcv) {
			printk(KERN_ERR
			       "mbx: illegal seq bit! ignoring this command. "
			       "(%08x)\n", msg);
			goto inc;
		}
		mbx->seq_rcv = seq;
#endif

		/* call receiver function */
		if ((receiver = mbx->receiver_map[(msg >> 24) & 0x7f]) == NULL)
			printk(KERN_ERR
			       "mbx: unknown message (%08x) received from "
			       "%s.\n", msg, mbx->name);
		else
			receiver(msg);

#ifdef MBX_USE_SEQ_BIT
inc:
#endif
		disable_newmsg_irq(mbx);
		mbq_inc(mbq->rp);
		if (mbq->rp == mbq->wp)
			empty = 1;
		/* if mbq has been full, now we have a room. */
		if (mbq->full) {
			mbq->full = 0;
			enable_newmsg_irq(mbx);
		}
		enable_newmsg_irq(mbx);
	}
}

/*
 * interrupt handler
 */
static irqreturn_t mbx_int_newmsg(int irq, void *p, struct pt_regs *regs)
{
	struct mbx *mbx = container_of(p, struct mbx, irq_devid_newmsg);
	struct mbq *mbq = &mbx->mbq;
	mbx_msg_t *msg;

#ifdef CONFIG_ARCH_OMAP2
	/*
	 * mailbox IRQ can be muxed.
	 * if it is not a newmsg interrupt, do nothing.
	 */
	if (!has_newmsg_irq(mbx))
		return IRQ_NONE;
#endif

	do {
#ifdef CONFIG_ARCH_OMAP2
		if (mbx_nomsg(mbx)) {
			/* no more messages in the fifo. clear IRQ source. */
			clear_newmsg_irq(mbx);
			break;
		}
#endif

		msg = &mbq->msg[mbq->wp];
		read_mbx(mbx, msg);

		mbq_inc(mbq->wp);
		if (mbq->wp == mbq->rp) {	/* mbq is full */
			mbq->full = 1;
			disable_newmsg_irq(mbx);
			break;
		}
#if defined(CONFIG_ARCH_OMAP1)
	} while (0);	/* do it once */
#elif defined(CONFIG_ARCH_OMAP2)
	} while (1);
#endif

	schedule_work(&mbx->work);
	return IRQ_HANDLED;
}

#ifdef CONFIG_ARCH_OMAP2
static irqreturn_t mbx_int_notfull(int irq, void *p, struct pt_regs *regs)
{
	struct mbx *mbx = container_of(p, struct mbx, irq_devid_notfull);

	/*
	 * mailbox IRQ can be muxed.
	 * if it is not a notfull interrupt, we do nothing.
	 */
#if 0
	if (!has_notfull_irq(mbx))
#else
	if (!(has_notfull_irq(mbx) && notfull_irq_enabled(mbx)))
#endif
		return IRQ_NONE;

	disable_notfull_irq(mbx);

#if 0	/*
	 * note: this doesn't seeem to work as explained in the manual.
	 * IRQSTATUS:NOTFULL can't be cleared even we write 1 to that bit.
	 * It is always set when it's not full, regardless of IRQENABLE setting.
	 */
	clear_notfull_irq(mbx);
#endif

	wake_up_interruptible_all(&mbx->full_wait_q);
	return IRQ_HANDLED;
}
#endif /* CONFIG_ARCH_OMAP2 */

static int __init mbx_request_irq(struct mbx *mbx, const char *devname)
{
	int ret;

#ifdef CONFIG_ARCH_OMAP2
	enable_newmsg_irq(mbx);
#endif

	ret = request_irq(mbx->irq, mbx_int_newmsg, SA_INTERRUPT | SA_SHIRQ,
			  devname, &mbx->irq_devid_newmsg);
	if (ret) {
		printk(KERN_ERR
		       "failed to register DSP mailbox newmsg interrupt: "
		       "%d\n", ret);
		return ret;
	}

#ifdef CONFIG_ARCH_OMAP2
	ret = request_irq(mbx->irq, mbx_int_notfull, SA_INTERRUPT | SA_SHIRQ,
			  devname, &mbx->irq_devid_notfull);
	if (ret) {
		printk(KERN_ERR
		       "failed to register DSP mailbox notfull interrupt: "
		       "%d\n", ret);
		return ret;
	}
#endif

	return 0;
}

static int __init omap_mailbox_init(void)
{
	int ret;
#ifdef CONFIG_ARCH_OMAP2
	struct clk *mbx_ick_handle;
#endif

	printk(KERN_INFO "Initializing OMAP Mailboxes\n");
#ifdef CONFIG_ARCH_OMAP2
	/*
	 * FIXME: mbx_ick will never unsed
	 */
	mbx_ick_handle = clk_get(NULL, "mailboxes_ick");
	if (IS_ERR(mbx_ick_handle)) {
		printk("Could not get mailboxes_ick\n");
		return -ENODEV;
	} else
		clk_enable(mbx_ick_handle);
#endif

	if ((ret = mbx_request_irq(&mbx_dsp, "mbx_dsp")) != 0)
		return ret;
#ifdef CONFIG_ARCH_OMAP2
	if ((ret = mbx_request_irq(&mbx_iva, "mbx_iva")) != 0)
		return ret;
#endif

	return 0;
}

arch_initcall(omap_mailbox_init);

EXPORT_SYMBOL(mbx_get);
EXPORT_SYMBOL(mbx_send);
EXPORT_SYMBOL(register_mbx_receiver);
EXPORT_SYMBOL(unregister_mbx_receiver);
EXPORT_SYMBOL(disable_mbx_irq);
EXPORT_SYMBOL(enable_mbx_irq);
EXPORT_SYMBOL(mbx_init_seq);
