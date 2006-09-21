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

struct ipbuf {
	u16 c;			/* count */
	u16 next;		/* link */
	u16 la;			/* lock owner (ARM side) */
	u16 sa;			/* sync word (ARM->DSP) */
	u16 ld;			/* lock owner (DSP side) */
	u16 sd;			/* sync word (DSP->ARM) */
	unsigned char d[0];	/* data */
};

struct ipbuf_p {
	u16 c;		/* count */
	u16 s;		/* sync word */
	u16 al;		/* data address lower */
	u16 ah;		/* data address upper */
};

#define IPBUF_SYS_DLEN	31

struct ipbuf_sys {
	u16 s;			/* sync word */
	u16 d[IPBUF_SYS_DLEN];	/* data */
};

struct ipbcfg {
	u16 ln;
	u16 lsz;
	void *base;
	u16 bsycnt;
	unsigned long cnt_full;	/* count of IPBFULL error */
};

struct ipbuf_head {
	u16 bid;
	struct ipbuf *p;
};

extern struct ipbcfg ipbcfg;
extern struct ipbuf_sys *ipbuf_sys_da, *ipbuf_sys_ad;

#define ipb_bsycnt_inc(ipbcfg)				\
	do {						\
		disable_irq(omap_dsp->mbox->irq);	\
		(ipbcfg)->bsycnt++;			\
		enable_irq(omap_dsp->mbox->irq);	\
	} while(0)

#define ipb_bsycnt_dec(ipbcfg)				\
	do {						\
		disable_irq(omap_dsp->mbox->irq);	\
		(ipbcfg)->bsycnt--;			\
		enable_irq(omap_dsp->mbox->irq);	\
	} while(0)

#define dsp_mem_enable_ipbuf()	dsp_mem_enable(ipbcfg.base)
#define dsp_mem_disable_ipbuf()	dsp_mem_disable(ipbcfg.base)

struct ipblink {
	spinlock_t lock;
	u16 top;
	u16 tail;
};

#define IPBLINK_INIT {				\
		.lock = SPIN_LOCK_UNLOCKED,	\
		.top  = BID_NULL,		\
		.tail = BID_NULL,		\
	}

#define INIT_IPBLINK(link)			\
	do {					\
		spin_lock_init(&(link)->lock);	\
		(link)->top  = BID_NULL;	\
		(link)->tail = BID_NULL;	\
	} while(0)

#define RESET_IPBLINK(link)			\
	do {					\
		(link)->top  = BID_NULL;	\
		(link)->tail = BID_NULL;	\
	} while(0)

#define ipblink_empty(link)	((link)->top == BID_NULL)

static __inline__ void __ipblink_del_top(struct ipblink *link)
{
	struct ipbuf_head *ipb_h = bid_to_ipbuf(link->top);

	if ((link->top = ipb_h->p->next) == BID_NULL)
		link->tail = BID_NULL;
	else
		ipb_h->p->next = BID_NULL;
}

static __inline__ void ipblink_del_top(struct ipblink *link)
{
	spin_lock(&link->lock);
	__ipblink_del_top(link);
	spin_unlock(&link->lock);
}

static __inline__ void __ipblink_add_tail(struct ipblink *link, u16 bid)
{
	if (ipblink_empty(link))
		link->top = bid;
	else
		bid_to_ipbuf(link->tail)->p->next = bid;
	link->tail = bid;
}

static __inline__ void ipblink_add_tail(struct ipblink *link, u16 bid)
{
	spin_lock(&link->lock);
	__ipblink_add_tail(link, bid);
	spin_unlock(&link->lock);
}

static __inline__ void __ipblink_flush(struct ipblink *link)
{
	u16 bid;

	while (!ipblink_empty(link)) {
		bid = link->top;
		__ipblink_del_top(link);
		unuse_ipbuf(bid_to_ipbuf(bid));
	}
}

static __inline__ void ipblink_flush(struct ipblink *link)
{
	spin_lock(&link->lock);
	__ipblink_flush(link);
	spin_unlock(&link->lock);
}

static __inline__ void __ipblink_add_pvt(struct ipblink *link)
{
	link->top  = BID_PVT;
	link->tail = BID_PVT;
}

static __inline__ void ipblink_add_pvt(struct ipblink *link)
{
	spin_lock(&link->lock);
	__ipblink_add_pvt(link);
	spin_unlock(&link->lock);
}

static __inline__ void __ipblink_del_pvt(struct ipblink *link)
{
	link->top  = BID_NULL;
	link->tail = BID_NULL;
}

static __inline__ void ipblink_del_pvt(struct ipblink *link)
{
	spin_lock(&link->lock);
	__ipblink_del_pvt(link);
	spin_unlock(&link->lock);
}

static __inline__ void __ipblink_flush_pvt(struct ipblink *link)
{
	if (!ipblink_empty(link))
		ipblink_del_pvt(link);
}

static __inline__ void ipblink_flush_pvt(struct ipblink *link)
{
	spin_lock(&link->lock);
	__ipblink_flush_pvt(link);
	spin_unlock(&link->lock);
}

#define ipblink_for_each(bid, link) \
	for (bid = (link)->top; bid != BID_NULL; bid = bid_to_ipbuf(bid)->p->next)
