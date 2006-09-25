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

#include <linux/sched.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <asm/arch/mailbox.h>
#include "dsp_mbcmd.h"
#include "dsp.h"
#include "ipbuf.h"

static struct ipbuf_head *g_ipbuf;
struct ipbcfg ipbcfg;
struct ipbuf_sys *ipbuf_sys_da, *ipbuf_sys_ad;
static struct ipblink ipb_free = IPBLINK_INIT;
static int ipbuf_sys_hold_mem_active;

static ssize_t ipbuf_show(struct device *dev, struct device_attribute *attr,
			  char *buf);
static struct device_attribute dev_attr_ipbuf = __ATTR_RO(ipbuf);

void ipbuf_stop(void)
{
	int i;

	device_remove_file(omap_dsp->dev, &dev_attr_ipbuf);

	spin_lock(&ipb_free.lock);
	RESET_IPBLINK(&ipb_free);
	spin_unlock(&ipb_free.lock);

	ipbcfg.ln = 0;
	if (g_ipbuf) {
		kfree(g_ipbuf);
		g_ipbuf = NULL;
	}
	for (i = 0; i < ipbuf_sys_hold_mem_active; i++) {
		dsp_mem_disable((void *)daram_base);
	}
	ipbuf_sys_hold_mem_active = 0;
}

int ipbuf_config(u16 ln, u16 lsz, void *base)
{
	size_t lsz_byte = ((size_t)lsz) << 1;
	size_t size;
	int ret = 0;
	int i;

	/*
	 * global IPBUF
	 */
	if (((unsigned long)base) & 0x3) {
		printk(KERN_ERR
		       "omapdsp: global ipbuf address(0x%p) is not "
		       "32-bit aligned!\n", base);
		return -EINVAL;
	}
	size = lsz_byte * ln;
	if (dsp_address_validate(base, size, "global ipbuf") < 0)
		return -EINVAL;

	g_ipbuf = kmalloc(sizeof(struct ipbuf_head) * ln, GFP_KERNEL);
	if (g_ipbuf == NULL) {
		printk(KERN_ERR
		       "omapdsp: memory allocation for ipbuf failed.\n");
		return -ENOMEM;
	}
	for (i = 0; i < ln; i++) {
		void *top, *btm;

		top = base + (sizeof(struct ipbuf) + lsz_byte) * i;
		btm = base + (sizeof(struct ipbuf) + lsz_byte) * (i+1) - 1;
		g_ipbuf[i].p = (struct ipbuf *)top;
		g_ipbuf[i].bid = i;
		if (((unsigned long)top & 0xfffe0000) !=
		    ((unsigned long)btm & 0xfffe0000)) {
			/*
			 * an ipbuf line should not cross
			 * 64k-word boundary.
			 */
			printk(KERN_ERR
			       "omapdsp: ipbuf[%d] crosses 64k-word boundary!\n"
			       "  @0x%p, size=0x%08x\n", i, top, lsz_byte);
			ret = -EINVAL;
			goto free_out;
		}
	}
	ipbcfg.ln       = ln;
	ipbcfg.lsz      = lsz;
	ipbcfg.base     = base;
	ipbcfg.bsycnt   = ln;	/* DSP holds all ipbufs initially. */
	ipbcfg.cnt_full = 0;

	printk(KERN_INFO
	       "omapdsp: IPBUF configuration\n"
	       "           %d words * %d lines at 0x%p.\n",
	       ipbcfg.lsz, ipbcfg.ln, ipbcfg.base);

	device_create_file(omap_dsp->dev, &dev_attr_ipbuf);

	return ret;

free_out:
	kfree(g_ipbuf);
	g_ipbuf = NULL;
	return ret;
}

int ipbuf_sys_config(void *p, arm_dsp_dir_t dir)
{
	char *dir_str = (dir == DIR_D2A) ? "D2A" : "A2D";

	if (((unsigned long)p) & 0x3) {
		printk(KERN_ERR
		       "omapdsp: system ipbuf(%s) address(0x%p) is "
		       "not 32-bit aligned!\n", dir_str, p);
		return -1;
	}
	if (dsp_address_validate(p, sizeof(struct ipbuf_sys),
				 "system ipbuf(%s)", dir_str) < 0)
		return -1;
	if (dsp_mem_type(p, sizeof(struct ipbuf_sys)) != MEM_TYPE_EXTERN) {
		printk(KERN_WARNING
		       "omapdsp: system ipbuf(%s) is placed in"
		       " DSP internal memory.\n"
		       "         It will prevent DSP from idling.\n", dir_str);
		ipbuf_sys_hold_mem_active++;
		/*
		 * dsp_mem_enable() never fails because
		 * it has been already enabled in dspcfg process and
		 * this will just increment the usecount.
		 */
		dsp_mem_enable((void *)daram_base);
	}

	if (dir == DIR_D2A)
		ipbuf_sys_da = p;
	else
		ipbuf_sys_ad = p;

	return 0;
}

int ipbuf_p_validate(void *p, arm_dsp_dir_t dir)
{
	char *dir_str = (dir == DIR_D2A) ? "D2A" : "A2D";

	if (((unsigned long)p) & 0x3) {
		printk(KERN_ERR
		       "omapdsp: private ipbuf(%s) address(0x%p) is "
		       "not 32-bit aligned!\n", dir_str, p);
		return -1;
	}
	return dsp_address_validate(p, sizeof(struct ipbuf_p),
				    "private ipbuf(%s)", dir_str);
}

/*
 * Global IPBUF operations
 */
struct ipbuf_head *bid_to_ipbuf(u16 bid)
{
	return &g_ipbuf[bid];
}

struct ipbuf_head *get_free_ipbuf(u8 tid)
{
	struct ipbuf_head *ipb_h;

	if (dsp_mem_enable_ipbuf() < 0)
		return NULL;

	spin_lock(&ipb_free.lock);

	if (ipblink_empty(&ipb_free)) {
		/* FIXME: wait on queue when not available.  */
		ipb_h = NULL;
		goto out;
	}
	ipb_h = &g_ipbuf[ipb_free.top];
	ipb_h->p->la = tid;	/* lock */
	__ipblink_del_top(&ipb_free);
out:
	spin_unlock(&ipb_free.lock);
	dsp_mem_disable_ipbuf();

	return ipb_h;
}

void release_ipbuf(struct ipbuf_head *ipb_h)
{
	if (ipb_h->p->la == TID_FREE) {
		printk(KERN_WARNING
		       "omapdsp: attempt to release unlocked IPBUF[%d].\n",
		       ipb_h->bid);
		/*
		 * FIXME: re-calc bsycnt
		 */
		return;
	}
	ipb_h->p->la = TID_FREE;
	ipb_h->p->sa = TID_FREE;
	ipblink_add_tail(&ipb_free, ipb_h->bid);
}

static int try_yld(struct ipbuf_head *ipb_h)
{
	int status;

	ipb_h->p->sa = TID_ANON;
	status = mbcompose_send(BKYLD, 0, ipb_h->bid);
	if (status < 0) {
		/* DSP is busy and ARM keeps this line. */
		release_ipbuf(ipb_h);
		return status;
	}

	ipb_bsycnt_inc(&ipbcfg);
	return 0;
}

/*
 * balancing ipbuf lines with DSP
 */
static void do_balance_ipbuf(void)
{
	while (ipbcfg.bsycnt <= ipbcfg.ln / 4) {
		struct ipbuf_head *ipb_h;

		if ((ipb_h = get_free_ipbuf(TID_ANON)) == NULL)
			return;
		if (try_yld(ipb_h) < 0)
			return;
	}
}

static DECLARE_WORK(balance_ipbuf_work, (void (*)(void *))do_balance_ipbuf,
		    NULL);

void balance_ipbuf(void)
{
	schedule_work(&balance_ipbuf_work);
}

/* for process context */
void unuse_ipbuf(struct ipbuf_head *ipb_h)
{
	if (ipbcfg.bsycnt > ipbcfg.ln / 4) {
		/* we don't have enough IPBUF lines. let's keep it. */
		release_ipbuf(ipb_h);
	} else {
		/* we have enough IPBUF lines. let's return this line to DSP. */
		ipb_h->p->la = TID_ANON;
		try_yld(ipb_h);
		balance_ipbuf();
	}
}

/* for interrupt context */
void unuse_ipbuf_nowait(struct ipbuf_head *ipb_h)
{
	release_ipbuf(ipb_h);
	balance_ipbuf();
}

/*
 * functions called from mailbox interrupt routine
 */

void mbox_err_ipbfull(void)
{
	ipbcfg.cnt_full++;
}

/*
 * sysfs files
 */
static ssize_t ipbuf_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	int len = 0;
	u16 bid;

	for (bid = 0; bid < ipbcfg.ln; bid++) {
		struct ipbuf_head *ipb_h = &g_ipbuf[bid];
		u16 la = ipb_h->p->la;
		u16 ld = ipb_h->p->ld;
		u16 c  = ipb_h->p->c;

		if (len > PAGE_SIZE - 100) {
			len += sprintf(buf + len, "out of buffer.\n");
			goto finish;
		}

		len += sprintf(buf + len, "ipbuf[%d]: adr = 0x%p\n",
			       bid, ipb_h->p);
		if (la == TID_FREE) {
			len += sprintf(buf + len,
				       "  DSPtask[%d]->Linux "
				       "(already read and now free for Linux)\n",
				       ld);
		} else if (ld == TID_FREE) {
			len += sprintf(buf + len,
				       "  Linux->DSPtask[%d] "
				       "(already read and now free for DSP)\n",
				       la);
		} else if (ipbuf_is_held(ld, bid)) {
			len += sprintf(buf + len,
				       "  DSPtask[%d]->Linux "
				       "(waiting to be read)\n"
				       "  count = %d\n", ld, c);
		} else {
			len += sprintf(buf + len,
				       "  Linux->DSPtask[%d] "
				       "(waiting to be read)\n"
				       "  count = %d\n", la, c);
		}
	}

	len += sprintf(buf + len, "\nFree IPBUF link: ");
	spin_lock(&ipb_free.lock);
	ipblink_for_each(bid, &ipb_free) {
		len += sprintf(buf + len, "%d ", bid);
	}
	spin_unlock(&ipb_free.lock);
	len += sprintf(buf + len, "\n");
	len += sprintf(buf + len, "IPBFULL error count: %ld\n",
		       ipbcfg.cnt_full);

finish:
	return len;
}
