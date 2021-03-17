/***
 *
 *  include/rtskb_fifo.h
 *
 *  RTnet - real-time networking subsystem
 *  Copyright (C) 2006 Jan Kiszka <jan.kiszka@web.de>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef __RTSKB_FIFO_H_
#define __RTSKB_FIFO_H_

#include <rtskb.h>

struct rtskb_fifo {
	unsigned long read_pos ____cacheline_aligned_in_smp;
	rtdm_lock_t read_lock;
	unsigned long size_mask;
	unsigned long write_pos ____cacheline_aligned_in_smp;
	rtdm_lock_t write_lock;
	struct rtskb *buffer[0];
};

#define DECLARE_RTSKB_FIFO(name_prefix, size)                                  \
	struct {                                                               \
		struct rtskb_fifo fifo;                                        \
		struct rtskb *__buffer[(size)];                                \
	} name_prefix

static inline int __rtskb_fifo_insert(struct rtskb_fifo *fifo,
				      struct rtskb *rtskb)
{
	unsigned long pos = fifo->write_pos;
	unsigned long new_pos = (pos + 1) & fifo->size_mask;

	if (unlikely(new_pos == fifo->read_pos))
		return -EAGAIN;

	fifo->buffer[pos] = rtskb;

	/* rtskb must have been written before write_pos update */
	smp_wmb();

	fifo->write_pos = new_pos;

	return 0;
}

static inline int rtskb_fifo_insert(struct rtskb_fifo *fifo,
				    struct rtskb *rtskb)
{
	rtdm_lockctx_t context;
	int result;

	rtdm_lock_get_irqsave(&fifo->write_lock, context);
	result = __rtskb_fifo_insert(fifo, rtskb);
	rtdm_lock_put_irqrestore(&fifo->write_lock, context);

	return result;
}

static inline int rtskb_fifo_insert_inirq(struct rtskb_fifo *fifo,
					  struct rtskb *rtskb)
{
	int result;

	rtdm_lock_get(&fifo->write_lock);
	result = __rtskb_fifo_insert(fifo, rtskb);
	rtdm_lock_put(&fifo->write_lock);

	return result;
}

static inline struct rtskb *__rtskb_fifo_remove(struct rtskb_fifo *fifo)
{
	unsigned long pos = fifo->read_pos;
	struct rtskb *result;

	/* check FIFO status first */
	if (unlikely(pos == fifo->write_pos))
		return NULL;

	/* at least one rtskb is enqueued, so get the next one */
	result = fifo->buffer[pos];

	/* result must have been read before read_pos update */
	smp_rmb();

	fifo->read_pos = (pos + 1) & fifo->size_mask;

	/* read_pos must have been written for a consitent fifo state on exit */
	smp_wmb();

	return result;
}

static inline struct rtskb *rtskb_fifo_remove(struct rtskb_fifo *fifo)
{
	rtdm_lockctx_t context;
	struct rtskb *result;

	rtdm_lock_get_irqsave(&fifo->read_lock, context);
	result = __rtskb_fifo_remove(fifo);
	rtdm_lock_put_irqrestore(&fifo->read_lock, context);

	return result;
}

static inline struct rtskb *rtskb_fifo_remove_inirq(struct rtskb_fifo *fifo)
{
	struct rtskb *result;

	rtdm_lock_get(&fifo->read_lock);
	result = __rtskb_fifo_remove(fifo);
	rtdm_lock_put(&fifo->read_lock);

	return result;
}

/* for now inlined... */
static inline void rtskb_fifo_init(struct rtskb_fifo *fifo, unsigned long size)
{
	fifo->read_pos = 0;
	fifo->write_pos = 0;
	fifo->size_mask = size - 1;
	rtdm_lock_init(&fifo->read_lock);
	rtdm_lock_init(&fifo->write_lock);
}

#endif /* __RTSKB_FIFO_H_ */
