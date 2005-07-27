/*
 * linux/arch/arm/mach-omap/dsp/ipbuf.h
 *
 * Header for IPBUF
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
 * 2005/05/17:  DSP Gateway version 3.3
 */

struct ipbuf {
	unsigned short c;	/* count */
	unsigned short next;	/* link */
	unsigned short la;	/* lock owner (ARM side) */
	unsigned short sa;	/* sync word (ARM->DSP) */
	unsigned short ld;	/* lock owner (DSP side) */
	unsigned short sd;	/* sync word (DSP->ARM) */
	unsigned char d[0];	/* data */
};

struct ipbuf_p {
	unsigned short c;	/* count */
	unsigned short s;	/* sync word */
	unsigned short al;	/* data address lower */
	unsigned short ah;	/* data address upper */
};

struct ipbuf_sys {
	unsigned short s;	/* sync word */
	unsigned short d[31];	/* data */
};

struct ipbcfg {
	unsigned short ln;
	unsigned short lsz;
	void *base;
	unsigned short bsycnt;
	unsigned long cnt_full;	/* count of IPBFULL error */
};

extern struct ipbuf **ipbuf;
extern struct ipbcfg ipbcfg;
extern struct ipbuf_sys *ipbuf_sys_da, *ipbuf_sys_ad;

#define ipb_bsycnt_inc(ipbcfg) \
	do { \
		disable_irq(INT_D2A_MB1); \
		(ipbcfg)->bsycnt++; \
		enable_irq(INT_D2A_MB1); \
	} while(0)

#define ipb_bsycnt_dec(ipbcfg) \
	do { \
		disable_irq(INT_D2A_MB1); \
		(ipbcfg)->bsycnt--; \
		enable_irq(INT_D2A_MB1); \
	} while(0)

#define dsp_mem_enable_ipbuf()	dsp_mem_enable(ipbcfg.base)
#define dsp_mem_disable_ipbuf()	dsp_mem_disable(ipbcfg.base)

struct ipblink {
	spinlock_t lock;
	unsigned short top;
	unsigned short tail;
};

#define IPBLINK_INIT { \
		.lock = SPIN_LOCK_UNLOCKED, \
		.top  = OMAP_DSP_BID_NULL, \
		.tail = OMAP_DSP_BID_NULL, \
	}

#define INIT_IPBLINK(link) \
	do { \
		spin_lock_init(&(link)->lock); \
		(link)->top  = OMAP_DSP_BID_NULL; \
		(link)->tail = OMAP_DSP_BID_NULL; \
	} while(0)

#define ipblink_empty(link)	((link)->top == OMAP_DSP_BID_NULL)

static __inline__ void ipblink_del_top(struct ipblink *link,
				       struct ipbuf **ipbuf)
{
	struct ipbuf *bufp = ipbuf[link->top];

	if ((link->top = bufp->next) == OMAP_DSP_BID_NULL)
		link->tail = OMAP_DSP_BID_NULL;
	else
		bufp->next = OMAP_DSP_BID_NULL;
}

static __inline__ void ipblink_add_tail(struct ipblink *link,
					unsigned short bid,
					struct ipbuf **ipbuf)
{
	if (ipblink_empty(link))
		link->top = bid;
	else
		ipbuf[link->tail]->next = bid;
	link->tail = bid;
}

static __inline__ void ipblink_add_pvt(struct ipblink *link)
{
	link->top  = OMAP_DSP_BID_PVT;
	link->tail = OMAP_DSP_BID_PVT;
}

static __inline__ void ipblink_del_pvt(struct ipblink *link)
{
	link->top  = OMAP_DSP_BID_NULL;
	link->tail = OMAP_DSP_BID_NULL;
}

#define ipblink_for_each(bid, link, ipbuf) \
	for (bid = (link)->top; bid != OMAP_DSP_BID_NULL; bid = ipbuf[bid]->next)
