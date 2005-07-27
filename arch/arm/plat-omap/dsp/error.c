/*
 * linux/arch/arm/mach-omap/dsp/error.c
 *
 * OMAP DSP error detection I/F device driver
 *
 * Copyright (C) 2002-2005 Nokia Corporation
 *
 * Written by Toshihiro Kobayashi <toshihiro.kobayashi@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * 2005/03/11:  DSP Gateway version 3.3
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/major.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/ioctls.h>
#include <asm/arch/dsp.h>
#include "dsp.h"

static DECLARE_WAIT_QUEUE_HEAD(err_wait_q);
static unsigned long errcode;
static int errcnt;
static unsigned short wdtval;	/* FIXME: read through ioctl */
static unsigned long mmu_fadr;	/* FIXME: read through ioctl */

/*
 * DSP error detection device file operations
 */
static ssize_t dsp_err_read(struct file *file, char *buf, size_t count,
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
	status = copy_to_user(buf, &errcode, 4);
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
 * DSP MMU
 */
void dsp_err_mmu_set(unsigned long adr)
{
	disable_irq(INT_DSP_MMU);
	errcode |= OMAP_DSP_ERRDT_MMU;
	errcnt++;
	mmu_fadr = adr;
	wake_up_interruptible(&err_wait_q);
}

void dsp_err_mmu_clear(void)
{
	errcode &= ~OMAP_DSP_ERRDT_MMU;
	enable_irq(INT_DSP_MMU);
}

int dsp_err_mmu_isset(void)
{
	return (errcode & OMAP_DSP_ERRDT_MMU) ? 1 : 0;
}

/*
 * WDT
 */
void dsp_err_wdt_clear(void)
{
	errcode &= ~OMAP_DSP_ERRDT_WDT;
}

int dsp_err_wdt_isset(void)
{
	return (errcode & OMAP_DSP_ERRDT_WDT) ? 1 : 0;
}

/*
 * functions called from mailbox1 interrupt routine
 */
static void mbx1_err_wdt(unsigned short data)
{
	errcode |= OMAP_DSP_ERRDT_WDT;
	errcnt++;
	wdtval = data;
	wake_up_interruptible(&err_wait_q);
}

#ifdef OLD_BINARY_SUPPORT
/* v3.3 obsolete */
void mbx1_wdt(struct mbcmd *mb)
{
	mbx1_err_wdt(mb->data);
}
#endif

extern void mbx1_err_ipbfull(void);
extern void mbx1_err_fatal(unsigned char tid);

void mbx1_err(struct mbcmd *mb)
{
	unsigned char eid = mb->cmd_l;
	char *eidnm = subcmd_name(mb);
	unsigned char tid;

	if (eidnm) {
		printk(KERN_WARNING
		       "mbx: ERR from DSP (%s): 0x%04x\n", eidnm, mb->data);
	} else {
		printk(KERN_WARNING
		       "mbx: ERR from DSP (unknown EID=%02x): %04x\n",
		       eid, mb->data);
	}

	switch (eid) {
	case OMAP_DSP_EID_IPBFULL:
		mbx1_err_ipbfull();
		break;

	case OMAP_DSP_EID_FATAL:
		tid = mb->data & 0x00ff;
		mbx1_err_fatal(tid);
		break;

	case OMAP_DSP_EID_WDT:
		mbx1_err_wdt(mb->data);
		break;
	}
}

/*
 *
 */
void dsp_err_start(void)
{
	errcnt = 0;
	if (dsp_err_wdt_isset())
		dsp_err_wdt_clear();
	if (dsp_err_mmu_isset())
		dsp_err_mmu_clear();
}

void dsp_err_stop(void)
{
	wake_up_interruptible(&err_wait_q);
}
