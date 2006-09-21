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

struct fifo_struct {
	spinlock_t lock;
	char *buf;
	size_t sz;
	size_t cnt;
	unsigned int wp;
};

static inline int alloc_fifo(struct fifo_struct *fifo, size_t sz)
{
	if ((fifo->buf = kmalloc(sz, GFP_KERNEL)) == NULL) {
		fifo->sz = 0;
		return -ENOMEM;
	}
	fifo->sz = sz;
	fifo->cnt = 0;
	fifo->wp = 0;
	return 0;
}

static inline int init_fifo(struct fifo_struct *fifo, size_t sz)
{
	spin_lock_init(&fifo->lock);
	return alloc_fifo(fifo, sz);
}

static inline void free_fifo(struct fifo_struct *fifo)
{
	spin_lock(&fifo->lock);
	if (fifo->buf == NULL) {
		spin_unlock(&fifo->lock);
		return;
	}

	kfree(fifo->buf);
	fifo->buf = NULL;
	fifo->sz = 0;
	spin_unlock(&fifo->lock);
}

static inline void flush_fifo(struct fifo_struct *fifo)
{
	spin_lock(&fifo->lock);
	fifo->cnt = 0;
	fifo->wp = 0;
	spin_unlock(&fifo->lock);
}

#define fifo_empty(fifo)	((fifo)->cnt == 0)

static inline int realloc_fifo(struct fifo_struct *fifo, size_t sz)
{
	int ret = sz;

	spin_lock(&fifo->lock);
	if (!fifo_empty(fifo)) {
		ret = -EBUSY;
		goto out;
	}

	/* free */
	if (fifo->buf)
		kfree(fifo->buf);

	/* alloc */
	ret = alloc_fifo(fifo, sz);

out:
	spin_unlock(&fifo->lock);
	return ret;
}

static inline void write_word_to_fifo(struct fifo_struct *fifo, u16 word)
{
	spin_lock(&fifo->lock);
	*(u16 *)&fifo->buf[fifo->wp] = word;
	if ((fifo->wp += 2) == fifo->sz)
		fifo->wp = 0;
	if ((fifo->cnt += 2) > fifo->sz)
		fifo->cnt = fifo->sz;
	spin_unlock(&fifo->lock);
}

/*
 * (before)
 *
 * [*******----------*************]
 *         ^wp
 *  <---------------------------->  sz = 30
 *  <----->          <----------->  cnt = 20
 *
 * (read: count=16)
 *  <->              <----------->  count = 16
 *                   <----------->  cnt1 = 13
 *                   ^rp
 *
 * (after)
 * [---****-----------------------]
 *         ^wp
 */
static inline ssize_t copy_to_user_fm_fifo(char *dst, struct fifo_struct *fifo,
					   size_t count)
{
	int rp;
	ssize_t ret;

	/* fifo size can be zero */
	if (fifo->sz == 0)
		return 0;

	spin_lock(&fifo->lock);
	if (count > fifo->cnt)
		count = fifo->cnt;

	if ((rp = fifo->wp - fifo->cnt) >= 0) {
		/* valid area is straight */
		if (copy_to_user(dst, &fifo->buf[rp], count)) {
			ret = -EFAULT;
			goto out;
		}
	} else {
		int cnt1 = -rp;
		rp += fifo->sz;
		if (cnt1 >= count) {
			/* requested area is straight */
			if (copy_to_user(dst, &fifo->buf[rp], count)) {
				ret = -EFAULT;
				goto out;
			}
		} else {
			if (copy_to_user(dst, &fifo->buf[rp], cnt1)) {
				ret = -EFAULT;
				goto out;
			}
			if (copy_to_user(dst+cnt1, fifo->buf, count-cnt1)) {
				ret = -EFAULT;
				goto out;
			}
		}
	}
	fifo->cnt -= count;
	ret = count;

out:
	spin_unlock(&fifo->lock);
	return ret;
}
