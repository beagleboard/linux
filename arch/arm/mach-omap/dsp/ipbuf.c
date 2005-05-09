/*
 * linux/arch/arm/mach-omap/dsp/ipbuf.c
 *
 * IPBUF handler
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
 * 2005/02/17:  DSP Gateway version 3.2
 */

#include <linux/init.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/device.h>
#include <asm/signal.h>
#include <asm/arch/dsp.h>
#include "dsp.h"
#include "ipbuf.h"

struct ipbuf **ipbuf;
struct ipbcfg ipbcfg;
struct ipbuf_sys *ipbuf_sys_da, *ipbuf_sys_ad;
static struct ipblink ipb_free = IPBLINK_INIT;

void ipbuf_stop(void)
{
	ipbcfg.ln = 0;
	if (ipbuf) {
		kfree(ipbuf);
		ipbuf = NULL;
	}
}

/*
 * ipbuf_config() is called by mailbox workqueue
 */
int ipbuf_config(unsigned short ln, unsigned short lsz, unsigned long adr)
{
	void *base;
	unsigned long lsz_byte = ((unsigned long)lsz) << 1;
	size_t size;
	int ret = 0;
	int i;

	spin_lock(&ipb_free.lock);
	INIT_IPBLINK(&ipb_free);
	spin_unlock(&ipb_free.lock);

	/*
	 * global IPBUF
	 */
	if (adr & 0x1) {
		printk(KERN_ERR
		       "mbx: global ipbuf address (0x%08lx) is odd number!\n",
		       adr);
		return -EINVAL;
	}
	size = lsz_byte * ln;
	if (adr + size > DSPSPACE_SIZE) {
		printk(KERN_ERR
		       "mbx: ipbuf address (0x%08lx) and size (0x%08x) is "
		       "illegal!\n", adr, size);
		return -EINVAL;
	}
	base = dspword_to_virt(adr);
	ipbuf = kmalloc(sizeof(void *) * ln, GFP_KERNEL);
	if (ipbuf == NULL) {
		printk(KERN_ERR "mbx: memory allocation for ipbuf failed.\n");
		return -ENOMEM;
	}
	for (i = 0; i < ln; i++) {
		void *top, *btm;

		top = base + (sizeof(struct ipbuf) + lsz_byte) * i;
		btm = base + (sizeof(struct ipbuf) + lsz_byte) * (i+1) - 1;
		ipbuf[i] = (struct ipbuf *)top;
		if (((unsigned long)top & 0xfffe0000) !=
		    ((unsigned long)btm & 0xfffe0000)) {
			/*
			 * an ipbuf line should not cross
			 * 64k-word boundary.
			 */
			printk(KERN_ERR
			       "omapdsp: ipbuf[%d] crosses 64k-word boundary!\n"
			       "  @0x%p, size=0x%08lx\n", i, top, lsz_byte);
			ret = -EINVAL;
		}
	}
	ipbcfg.ln       = ln;
	ipbcfg.lsz      = lsz;
	ipbcfg.adr      = adr;
	ipbcfg.bsycnt   = ln;	/* DSP holds all ipbufs initially. */
	ipbcfg.cnt_full = 0;

	printk(KERN_INFO
	       "omapdsp: IPBUF configuration\n"
	       "           %d words * %d lines at 0x%p.\n",
	       ipbcfg.lsz, ipbcfg.ln, dspword_to_virt(ipbcfg.adr));

	return ret;
}

/*
 * Global IPBUF operations
 */
unsigned short get_free_ipbuf(unsigned char tid)
{
	unsigned short bid;

	if (ipblink_empty(&ipb_free)) {
		/* FIXME: wait on queue when not available.  */
		return OMAP_DSP_BID_NULL;
	}

	/*
	 * FIXME: dsp_enable_dspmem!
	 */
	spin_lock(&ipb_free.lock);
	bid = ipb_free.top;
	ipbuf[bid]->la = tid;	/* lock */
	ipblink_del_top(&ipb_free, ipbuf);
	spin_unlock(&ipb_free.lock);

	return bid;
}

void release_ipbuf(unsigned short bid)
{
	if (ipbuf[bid]->la == OMAP_DSP_TID_FREE) {
		printk(KERN_WARNING
		       "omapdsp: attempt to release unlocked IPBUF[%d].\n",
		       bid);
		/*
		 * FIXME: re-calc bsycnt
		 */
		return;
	}
	ipbuf[bid]->la = OMAP_DSP_TID_FREE;
	ipbuf[bid]->sa = OMAP_DSP_TID_FREE;
	spin_lock(&ipb_free.lock);
	ipblink_add_tail(&ipb_free, bid, ipbuf);
	spin_unlock(&ipb_free.lock);
}

static int try_yld(unsigned short bid)
{
	struct mbcmd mb;
	int status;

	ipbuf[bid]->sa = OMAP_DSP_TID_ANON;
	mbcmd_set(mb, MBCMD(BKYLD), 0, bid);
	status = dsp_mbsend(&mb);
	if (status < 0) {
		/* DSP is busy and ARM keeps this line. */
		release_ipbuf(bid);
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
		unsigned short bid;

		bid = get_free_ipbuf(OMAP_DSP_TID_ANON);
		if (bid == OMAP_DSP_BID_NULL)
			return;
		if (try_yld(bid) < 0)
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
void unuse_ipbuf(unsigned short bid)
{
	if (ipbcfg.bsycnt > ipbcfg.ln / 4) {
		/* we don't have enough IPBUF lines. let's keep it. */
		release_ipbuf(bid);
	} else {
		/* we have enough IPBUF lines. let's return this line to DSP. */
		ipbuf[bid]->la = OMAP_DSP_TID_ANON;
		try_yld(bid);
		balance_ipbuf();
	}
}

/* for interrupt context */
void unuse_ipbuf_nowait(unsigned short bid)
{
	release_ipbuf(bid);
	balance_ipbuf();
}

/*
 * functions called from mailbox1 interrupt routine
 */

void mbx1_err_ipbfull(void)
{
	ipbcfg.cnt_full++;
}

/*
 * sysfs files
 */
static ssize_t ipbuf_show(struct device *dev, char *buf)
{
	int len = 0;
	unsigned short bid;

	for (bid = 0; bid < ipbcfg.ln; bid++) {
		unsigned short la = ipbuf[bid]->la;
		unsigned short ld = ipbuf[bid]->ld;
		unsigned short c  = ipbuf[bid]->c;

		if (len > PAGE_SIZE - 100) {
			len += sprintf(buf + len, "out of buffer.\n");
			goto finish;
		}

		len += sprintf(buf + len, "ipbuf[%d]: adr = 0x%p\n",
			       bid, ipbuf[bid]);
		if (la == OMAP_DSP_TID_FREE) {
			len += sprintf(buf + len,
				       "  DSPtask[%d]->Linux "
				       "(already read and now free for Linux)\n",
				       ld);
		} else if (ld == OMAP_DSP_TID_FREE) {
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
	ipblink_for_each(bid, &ipb_free, ipbuf) {
		len += sprintf(buf + len, "%d ", bid);
	}
	spin_unlock(&ipb_free.lock);
	len += sprintf(buf + len, "\n");
	len += sprintf(buf + len, "IPBFULL error count: %ld\n",
		       ipbcfg.cnt_full);

finish:
	return len;
}

struct device_attribute dev_attr_ipbuf = __ATTR_RO(ipbuf);
