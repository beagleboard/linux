/*
 * This file is part of OMAP DSP driver (DSP Gateway version 3.3.1)
 *
 * Copyright (C) 2002-2006 Nokia Corporation. All rights reserved.
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

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include "dsp_mbcmd.h"
#include "dsp.h"

/*
 * value seen through read()
 */
#define DSP_ERR_WDT	0x00000001
#define DSP_ERR_MMU	0x00000002
static unsigned long errval;

static DECLARE_WAIT_QUEUE_HEAD(err_wait_q);
static int errcnt;
static u16 wdtval;	/* FIXME: read through ioctl */
static u32 mmu_fadr;	/* FIXME: read through ioctl */

/*
 * DSP error detection device file operations
 */
static ssize_t dsp_err_read(struct file *file, char __user *buf, size_t count,
			    loff_t *ppos)
{
	unsigned long flags;
	int status;

	if (count < 4)
		return 0;

	if (errcnt == 0) {
		long current_state;
		DECLARE_WAITQUEUE(wait, current);

		add_wait_queue(&err_wait_q, &wait);
		current_state = current->state;
		set_current_state(TASK_INTERRUPTIBLE);
		if (errcnt == 0)	/* last check */
			schedule();
		set_current_state(current_state);
		remove_wait_queue(&err_wait_q, &wait);
		if (signal_pending(current))
			return -EINTR;
	}

	local_irq_save(flags);
	status = copy_to_user(buf, &errval, 4);
	if (status) {
		local_irq_restore(flags);
		return -EFAULT;
	}
	errcnt = 0;
	local_irq_restore(flags);

	return 4;
}

static unsigned int dsp_err_poll(struct file *file, poll_table *wait)
{
	unsigned int mask = 0;

	poll_wait(file, &err_wait_q, wait);
	if (errcnt != 0)
		mask |= POLLIN | POLLRDNORM;

	return mask;
}

struct file_operations dsp_err_fops = {
	.owner = THIS_MODULE,
	.poll  = dsp_err_poll,
	.read  = dsp_err_read,
};

/*
 * set / clear functions
 */

/* DSP MMU */
static void dsp_err_mmu_set(unsigned long arg)
{
	disable_irq(omap_dsp->mmu_irq);
	mmu_fadr = (u32)arg;
}

static void dsp_err_mmu_clr(void)
{
	enable_irq(omap_dsp->mmu_irq);
}

/* WDT */
static void dsp_err_wdt_set(unsigned long arg)
{
	wdtval = (u16)arg;
}

/*
 * error code handler
 */
static struct {
	unsigned long val;
	void (*set)(unsigned long arg);
	void (*clr)(void);
} dsp_err_desc[ERRCODE_MAX] = {
	[ERRCODE_MMU] = { DSP_ERR_MMU, dsp_err_mmu_set, dsp_err_mmu_clr },
	[ERRCODE_WDT] = { DSP_ERR_WDT, dsp_err_wdt_set, NULL },
};

void dsp_err_set(enum errcode_e code, unsigned long arg)
{
	if (dsp_err_desc[code].set != NULL)
		dsp_err_desc[code].set(arg);

	errval |= dsp_err_desc[code].val;
	errcnt++;
	wake_up_interruptible(&err_wait_q);
}

void dsp_err_clear(enum errcode_e code)
{
	errval &= ~dsp_err_desc[code].val;

	if (dsp_err_desc[code].clr != NULL)
		dsp_err_desc[code].clr();
}

int dsp_err_isset(enum errcode_e code)
{
	return (errval & dsp_err_desc[code].val) ? 1 : 0;
}

/*
 * functions called from mailbox interrupt routine
 */
static void mbox_err_wdt(u16 data)
{
	dsp_err_set(DSP_ERR_WDT, (unsigned long)data);
}

#ifdef OLD_BINARY_SUPPORT
/* v3.3 obsolete */
void mbox_wdt(struct mbcmd *mb)
{
	mbox_err_wdt(mb->data);
}
#endif

extern void mbox_err_ipbfull(void);
extern void mbox_err_fatal(u8 tid);

void mbox_err(struct mbcmd *mb)
{
	u8 eid = mb->cmd_l;
	char *eidnm = subcmd_name(mb);
	u8 tid;

	if (eidnm) {
		printk(KERN_WARNING
		       "mbox: ERR from DSP (%s): 0x%04x\n", eidnm, mb->data);
	} else {
		printk(KERN_WARNING
		       "mbox: ERR from DSP (unknown EID=%02x): %04x\n",
		       eid, mb->data);
	}

	switch (eid) {
	case EID_IPBFULL:
		mbox_err_ipbfull();
		break;

	case EID_FATAL:
		tid = mb->data & 0x00ff;
		mbox_err_fatal(tid);
		break;

	case EID_WDT:
		mbox_err_wdt(mb->data);
		break;
	}
}

/*
 *
 */
void dsp_err_start(void)
{
	enum errcode_e i;

	for (i = 0; i < ERRCODE_MAX; i++) {
		if (dsp_err_isset(i))
			dsp_err_clear(i);
	}

	errcnt = 0;
}

void dsp_err_stop(void)
{
	wake_up_interruptible(&err_wait_q);
}
