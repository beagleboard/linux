/*
 * Copyright (C) 2001,2002,2003,2004 Philippe Gerum <rpm@xenomai.org>.
 * Copyright (C) 2005 Dmitry Adamushko <dmitry.adamushko@gmail.com>
 *
 * Xenomai is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation, Inc., 675 Mass Ave, Cambridge MA
 * 02139, USA; either version 2 of the License, or (at your option)
 * any later version.
 *
 * Xenomai is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 * 02111-1307, USA.
 */
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/fcntl.h>
#include <linux/poll.h>
#include <linux/termios.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <cobalt/kernel/sched.h>
#include <cobalt/kernel/heap.h>
#include <cobalt/kernel/pipe.h>
#include <cobalt/kernel/apc.h>

static int xnpipe_asyncsig = SIGIO;

struct xnpipe_state xnpipe_states[XNPIPE_NDEVS];
EXPORT_SYMBOL_GPL(xnpipe_states);

#define XNPIPE_BITMAP_SIZE	((XNPIPE_NDEVS + BITS_PER_LONG - 1) / BITS_PER_LONG)

static unsigned long xnpipe_bitmap[XNPIPE_BITMAP_SIZE];

static LIST_HEAD(xnpipe_sleepq);

static LIST_HEAD(xnpipe_asyncq);

int xnpipe_wakeup_apc;

static struct class *xnpipe_class;

/* Allocation of minor values */

static inline int xnpipe_minor_alloc(int minor)
{
	spl_t s;

	if ((minor < 0 && minor != XNPIPE_MINOR_AUTO) || minor >= XNPIPE_NDEVS)
		return -ENODEV;

	xnlock_get_irqsave(&nklock, s);

	if (minor == XNPIPE_MINOR_AUTO)
		minor = find_first_zero_bit(xnpipe_bitmap, XNPIPE_NDEVS);

	if (minor == XNPIPE_NDEVS ||
	    (xnpipe_bitmap[minor / BITS_PER_LONG] &
	     (1UL << (minor % BITS_PER_LONG))))
		minor = -EBUSY;
	else
		xnpipe_bitmap[minor / BITS_PER_LONG] |=
			(1UL << (minor % BITS_PER_LONG));

	xnlock_put_irqrestore(&nklock, s);

	return minor;
}

static inline void xnpipe_minor_free(int minor)
{
	xnpipe_bitmap[minor / BITS_PER_LONG] &=
		~(1UL << (minor % BITS_PER_LONG));
}

static inline void xnpipe_enqueue_wait(struct xnpipe_state *state, int mask)
{
	if (state->wcount != 0x7fffffff && state->wcount++ == 0)
		list_add_tail(&state->slink, &xnpipe_sleepq);

	state->status |= mask;
}

static inline void xnpipe_dequeue_wait(struct xnpipe_state *state, int mask)
{
	if (state->status & mask)
		if (--state->wcount == 0) {
			list_del(&state->slink);
			state->status &= ~mask;
		}
}

static inline void xnpipe_dequeue_all(struct xnpipe_state *state, int mask)
{
	if (state->status & mask) {
		if (state->wcount) {
			state->wcount = 0;
			list_del(&state->slink);
			state->status &= ~mask;
		}
	}
}

/* Must be entered with nklock held, interrupts off. */
#define xnpipe_wait(__state, __mask, __s, __cond)			\
({									\
	wait_queue_head_t *__waitq;					\
	DEFINE_WAIT(__wait);						\
	int __sigpending;						\
									\
	if ((__mask) & XNPIPE_USER_WREAD)				\
		__waitq = &(__state)->readq;				\
	else								\
		__waitq = &(__state)->syncq;				\
									\
	xnpipe_enqueue_wait(__state, __mask);				\
	xnlock_put_irqrestore(&nklock, __s);				\
									\
	for (;;) {							\
		__sigpending = signal_pending(current);			\
		if (__sigpending)					\
			break;						\
		prepare_to_wait_exclusive(__waitq, &__wait, TASK_INTERRUPTIBLE); \
		if (__cond)						\
			break;						\
		schedule();						\
	}								\
									\
	finish_wait(__waitq, &__wait);					\
									\
	/* Restore the interrupt state initially set by the caller. */	\
	xnlock_get_irqsave(&nklock, __s);				\
	xnpipe_dequeue_wait(__state, __mask);				\
									\
	__sigpending;							\
})

static void xnpipe_wakeup_proc(void *cookie)
{
	struct xnpipe_state *state;
	unsigned long rbits;
	spl_t s;

	xnlock_get_irqsave(&nklock, s);

	/*
	 * NOTE: sleepers might enter/leave the queue while we don't
	 * hold the nklock in these wakeup loops. So we iterate over
	 * each sleeper list until we find no more candidate for
	 * wakeup after an entire scan, redoing the scan from the list
	 * head otherwise.
	 */
	for (;;) {
		if (list_empty(&xnpipe_sleepq))
			goto check_async;

		state = list_first_entry(&xnpipe_sleepq, struct xnpipe_state, slink);

		for (;;) {
			rbits = state->status & XNPIPE_USER_ALL_READY;
			if (rbits)
				break;
			if (list_is_last(&state->slink, &xnpipe_sleepq))
				goto check_async;
			state = list_next_entry(state, slink);
		}

		state->status &= ~rbits;

		if ((rbits & XNPIPE_USER_WREAD_READY) != 0) {
			if (waitqueue_active(&state->readq)) {
				xnlock_put_irqrestore(&nklock, s);
				wake_up_interruptible(&state->readq);
				xnlock_get_irqsave(&nklock, s);
			}
		}
		if ((rbits & XNPIPE_USER_WSYNC_READY) != 0) {
			if (waitqueue_active(&state->syncq)) {
				xnlock_put_irqrestore(&nklock, s);
				wake_up_interruptible(&state->syncq);
				xnlock_get_irqsave(&nklock, s);
			}
		}
	}

check_async:
	/*
	 * Scan the async queue, sending the proper signal to
	 * subscribers.
	 */
	for (;;) {
		if (list_empty(&xnpipe_asyncq))
			goto out;

		state = list_first_entry(&xnpipe_asyncq, struct xnpipe_state, alink);

		for (;;) {
			if (state->status & XNPIPE_USER_SIGIO)
				break;
			if (list_is_last(&state->alink, &xnpipe_asyncq))
				goto out;
			state = list_next_entry(state, alink);
		}

		state->status &= ~XNPIPE_USER_SIGIO;
		xnlock_put_irqrestore(&nklock, s);
		kill_fasync(&state->asyncq, xnpipe_asyncsig, POLL_IN);
		xnlock_get_irqsave(&nklock, s);
	}
out:
	xnlock_put_irqrestore(&nklock, s);
}

static inline void xnpipe_schedule_request(void) /* hw IRQs off */
{
	__xnapc_schedule(xnpipe_wakeup_apc);
}

static inline ssize_t xnpipe_flush_bufq(void (*fn)(void *buf, void *xstate),
					struct list_head *q,
					void *xstate)
{
	struct xnpipe_mh *mh, *tmp;
	ssize_t n = 0;

	if (list_empty(q))
		return 0;

	/* Queue is private, no locking is required. */
	list_for_each_entry_safe(mh, tmp, q, link) {
		list_del(&mh->link);
		n += xnpipe_m_size(mh);
		fn(mh, xstate);
	}

	/* Return the overall count of bytes flushed. */
	return n;
}

/*
 * Move the specified queue contents to a private queue, then call the
 * flush handler to purge it. The latter runs without locking.
 * Returns the number of bytes flushed. Must be entered with nklock
 * held, interrupts off.
 */
#define xnpipe_flushq(__state, __q, __f, __s)				\
({									\
	LIST_HEAD(__privq);						\
	ssize_t __n;							\
									\
	list_splice_init(&(state)->__q, &__privq);			\
	(__state)->nr ## __q = 0;					\
	xnlock_put_irqrestore(&nklock, (__s));				\
	__n = xnpipe_flush_bufq((__state)->ops.__f, &__privq, (__state)->xstate);	\
	xnlock_get_irqsave(&nklock, (__s));				\
									\
	__n;								\
})

static void *xnpipe_default_alloc_ibuf(size_t size, void *xstate)
{
	void *buf;

	buf = xnmalloc(size);
	if (likely(buf != NULL))
		return buf;

	if (size > xnheap_get_size(&cobalt_heap))
		/* Request will never succeed. */
		return (struct xnpipe_mh *)-1;

	return NULL;
}

static void xnpipe_default_free_ibuf(void *buf, void *xstate)
{
	xnfree(buf);
}

static void xnpipe_default_release(void *xstate)
{
}

static inline int xnpipe_set_ops(struct xnpipe_state *state,
				 struct xnpipe_operations *ops)
{
	state->ops = *ops;

	if (ops->free_obuf == NULL)
		/*
		 * Caller must provide a way to free unread outgoing
		 * buffers.
		 */
		return -EINVAL;

	/* Set some default handlers for common usage. */
	if (ops->alloc_ibuf == NULL)
		state->ops.alloc_ibuf = xnpipe_default_alloc_ibuf;
	if (ops->free_ibuf == NULL)
		state->ops.free_ibuf = xnpipe_default_free_ibuf;
	if (ops->release == NULL)
		state->ops.release = xnpipe_default_release;

	return 0;
}

int xnpipe_connect(int minor, struct xnpipe_operations *ops, void *xstate)
{
	struct xnpipe_state *state;
	int need_sched = 0, ret;
	spl_t s;

	minor = xnpipe_minor_alloc(minor);
	if (minor < 0)
		return minor;

	state = &xnpipe_states[minor];

	xnlock_get_irqsave(&nklock, s);

	ret = xnpipe_set_ops(state, ops);
	if (ret) {
		xnlock_put_irqrestore(&nklock, s);
		return ret;
	}

	state->status |= XNPIPE_KERN_CONN;
	xnsynch_init(&state->synchbase, XNSYNCH_FIFO, NULL);
	state->xstate = xstate;
	state->ionrd = 0;

	if (state->status & XNPIPE_USER_CONN) {
		if (state->status & XNPIPE_USER_WREAD) {
			/*
			 * Wake up the regular Linux task waiting for
			 * the kernel side to connect (xnpipe_open).
			 */
			state->status |= XNPIPE_USER_WREAD_READY;
			need_sched = 1;
		}

		if (state->asyncq) {	/* Schedule asynch sig. */
			state->status |= XNPIPE_USER_SIGIO;
			need_sched = 1;
		}
	}

	if (need_sched)
		xnpipe_schedule_request();

	xnlock_put_irqrestore(&nklock, s);

	return minor;
}
EXPORT_SYMBOL_GPL(xnpipe_connect);

int xnpipe_disconnect(int minor)
{
	struct xnpipe_state *state;
	int need_sched = 0;
	spl_t s;

	if (minor < 0 || minor >= XNPIPE_NDEVS)
		return -ENODEV;

	state = &xnpipe_states[minor];

	xnlock_get_irqsave(&nklock, s);

	if ((state->status & XNPIPE_KERN_CONN) == 0) {
		xnlock_put_irqrestore(&nklock, s);
		return -EBADF;
	}

	state->status &= ~XNPIPE_KERN_CONN;

	state->ionrd -= xnpipe_flushq(state, outq, free_obuf, s);

	if ((state->status & XNPIPE_USER_CONN) == 0)
		goto cleanup;

	xnpipe_flushq(state, inq, free_ibuf, s);

	if (xnsynch_destroy(&state->synchbase) == XNSYNCH_RESCHED)
		xnsched_run();

	if (state->status & XNPIPE_USER_WREAD) {
		/*
		 * Wake up the regular Linux task waiting for some
		 * operation from the Xenomai side (read/write or
		 * poll).
		 */
		state->status |= XNPIPE_USER_WREAD_READY;
		need_sched = 1;
	}

	if (state->asyncq) {	/* Schedule asynch sig. */
		state->status |= XNPIPE_USER_SIGIO;
		need_sched = 1;
	}

cleanup:
	/*
	 * If xnpipe_release() has not fully run, enter lingering
	 * close. This will prevent the extra state from being wiped
	 * out until then.
	 */
	if (state->status & XNPIPE_USER_CONN)
		state->status |= XNPIPE_KERN_LCLOSE;
	else {
		xnlock_put_irqrestore(&nklock, s);
		state->ops.release(state->xstate);
		xnlock_get_irqsave(&nklock, s);
		xnpipe_minor_free(minor);
	}

	if (need_sched)
		xnpipe_schedule_request();

	xnlock_put_irqrestore(&nklock, s);

	return 0;
}
EXPORT_SYMBOL_GPL(xnpipe_disconnect);

ssize_t xnpipe_send(int minor, struct xnpipe_mh *mh, size_t size, int flags)
{
	struct xnpipe_state *state;
	int need_sched = 0;
	spl_t s;

	if (minor < 0 || minor >= XNPIPE_NDEVS)
		return -ENODEV;

	if (size <= sizeof(*mh))
		return -EINVAL;

	state = &xnpipe_states[minor];

	xnlock_get_irqsave(&nklock, s);

	if ((state->status & XNPIPE_KERN_CONN) == 0) {
		xnlock_put_irqrestore(&nklock, s);
		return -EBADF;
	}

	xnpipe_m_size(mh) = size - sizeof(*mh);
	xnpipe_m_rdoff(mh) = 0;
	state->ionrd += xnpipe_m_size(mh);

	if (flags & XNPIPE_URGENT)
		list_add(&mh->link, &state->outq);
	else
		list_add_tail(&mh->link, &state->outq);

	state->nroutq++;

	if ((state->status & XNPIPE_USER_CONN) == 0) {
		xnlock_put_irqrestore(&nklock, s);
		return (ssize_t) size;
	}

	if (state->status & XNPIPE_USER_WREAD) {
		/*
		 * Wake up the regular Linux task waiting for input
		 * from the Xenomai side.
		 */
		state->status |= XNPIPE_USER_WREAD_READY;
		need_sched = 1;
	}

	if (state->asyncq) {	/* Schedule asynch sig. */
		state->status |= XNPIPE_USER_SIGIO;
		need_sched = 1;
	}

	if (need_sched)
		xnpipe_schedule_request();

	xnlock_put_irqrestore(&nklock, s);

	return (ssize_t) size;
}
EXPORT_SYMBOL_GPL(xnpipe_send);

ssize_t xnpipe_mfixup(int minor, struct xnpipe_mh *mh, ssize_t size)
{
	struct xnpipe_state *state;
	spl_t s;

	if (minor < 0 || minor >= XNPIPE_NDEVS)
		return -ENODEV;

	if (size < 0)
		return -EINVAL;

	state = &xnpipe_states[minor];

	xnlock_get_irqsave(&nklock, s);

	if ((state->status & XNPIPE_KERN_CONN) == 0) {
		xnlock_put_irqrestore(&nklock, s);
		return -EBADF;
	}

	xnpipe_m_size(mh) += size;
	state->ionrd += size;

	xnlock_put_irqrestore(&nklock, s);

	return (ssize_t) size;
}
EXPORT_SYMBOL_GPL(xnpipe_mfixup);

ssize_t xnpipe_recv(int minor, struct xnpipe_mh **pmh, xnticks_t timeout)
{
	struct xnpipe_state *state;
	struct xnpipe_mh *mh;
	xntmode_t mode;
	ssize_t ret;
	int info;
	spl_t s;

	if (minor < 0 || minor >= XNPIPE_NDEVS)
		return -ENODEV;

	if (xnsched_interrupt_p())
		return -EPERM;

	state = &xnpipe_states[minor];

	xnlock_get_irqsave(&nklock, s);

	if ((state->status & XNPIPE_KERN_CONN) == 0) {
		ret = -EBADF;
		goto unlock_and_exit;
	}

	/*
	 * If we received a relative timespec, rescale it to an
	 * absolute time value based on the monotonic clock.
	 */
	mode = XN_RELATIVE;
	if (timeout != XN_NONBLOCK && timeout != XN_INFINITE) {
		mode = XN_ABSOLUTE;
		timeout += xnclock_read_monotonic(&nkclock);
	}

	for (;;) {
		if (!list_empty(&state->inq))
			break;

		if (timeout == XN_NONBLOCK) {
			ret = -EWOULDBLOCK;
			goto unlock_and_exit;
		}

		info = xnsynch_sleep_on(&state->synchbase, timeout, mode);
		if (info & XNTIMEO) {
			ret = -ETIMEDOUT;
			goto unlock_and_exit;
		}
		if (info & XNBREAK) {
			ret = -EINTR;
			goto unlock_and_exit;
		}
		if (info & XNRMID) {
			ret = -EIDRM;
			goto unlock_and_exit;
		}
	}

	mh = list_get_entry(&state->inq, struct xnpipe_mh, link);
	*pmh = mh;
	state->nrinq--;
	ret = (ssize_t)xnpipe_m_size(mh);

	if (state->status & XNPIPE_USER_WSYNC) {
		state->status |= XNPIPE_USER_WSYNC_READY;
		xnpipe_schedule_request();
	}

unlock_and_exit:

	xnlock_put_irqrestore(&nklock, s);

	return ret;
}
EXPORT_SYMBOL_GPL(xnpipe_recv);

int xnpipe_flush(int minor, int mode)
{
	struct xnpipe_state *state;
	int msgcount;
	spl_t s;

	if (minor < 0 || minor >= XNPIPE_NDEVS)
		return -ENODEV;

	state = &xnpipe_states[minor];

	xnlock_get_irqsave(&nklock, s);

	if ((state->status & XNPIPE_KERN_CONN) == 0) {
		xnlock_put_irqrestore(&nklock, s);
		return -EBADF;
	}

	msgcount = state->nroutq + state->nrinq;

	if (mode & XNPIPE_OFLUSH)
		state->ionrd -= xnpipe_flushq(state, outq, free_obuf, s);

	if (mode & XNPIPE_IFLUSH)
		xnpipe_flushq(state, inq, free_ibuf, s);

	if ((state->status & XNPIPE_USER_WSYNC) &&
	    msgcount > state->nroutq + state->nrinq) {
		state->status |= XNPIPE_USER_WSYNC_READY;
		xnpipe_schedule_request();
	}

	xnlock_put_irqrestore(&nklock, s);

	return 0;
}
EXPORT_SYMBOL_GPL(xnpipe_flush);

int xnpipe_pollstate(int minor, unsigned int *mask_r)
{
	struct xnpipe_state *state;
	int ret = 0;
	spl_t s;

	if (minor < 0 || minor >= XNPIPE_NDEVS)
		return -ENODEV;

	state = xnpipe_states + minor;

	xnlock_get_irqsave(&nklock, s);

	if (state->status & XNPIPE_KERN_CONN) {
		*mask_r = POLLOUT;
		if (!list_empty(&state->inq))
			*mask_r |= POLLIN;
	} else
		ret = -EIO;

	xnlock_put_irqrestore(&nklock, s);

	return ret;
}
EXPORT_SYMBOL_GPL(xnpipe_pollstate);

/* Must be entered with nklock held, interrupts off. */
#define xnpipe_cleanup_user_conn(__state, __s)				\
	do {								\
		xnpipe_flushq((__state), outq, free_obuf, (__s));	\
		xnpipe_flushq((__state), inq, free_ibuf, (__s));	\
		(__state)->status &= ~XNPIPE_USER_CONN;			\
		if ((__state)->status & XNPIPE_KERN_LCLOSE) {		\
			(__state)->status &= ~XNPIPE_KERN_LCLOSE;	\
			xnlock_put_irqrestore(&nklock, (__s));		\
			(__state)->ops.release((__state)->xstate);	\
			xnlock_get_irqsave(&nklock, (__s));		\
			xnpipe_minor_free(xnminor_from_state(__state));	\
		}							\
	} while(0)

/*
 * Open the pipe from user-space.
 */

static int xnpipe_open(struct inode *inode, struct file *file)
{
	int minor, err = 0, sigpending;
	struct xnpipe_state *state;
	spl_t s;

	minor = MINOR(inode->i_rdev);

	if (minor >= XNPIPE_NDEVS)
		return -ENXIO;	/* TssTss... stop playing with mknod() ;o) */

	state = &xnpipe_states[minor];

	xnlock_get_irqsave(&nklock, s);

	/* Enforce exclusive open for the message queues. */
	if (state->status & (XNPIPE_USER_CONN | XNPIPE_USER_LCONN)) {
		xnlock_put_irqrestore(&nklock, s);
		return -EBUSY;
	}

	state->status |= XNPIPE_USER_LCONN;

	xnlock_put_irqrestore(&nklock, s);

	file->private_data = state;
	init_waitqueue_head(&state->readq);
	init_waitqueue_head(&state->syncq);

	xnlock_get_irqsave(&nklock, s);

	state->status |= XNPIPE_USER_CONN;
	state->status &= ~XNPIPE_USER_LCONN;
	state->wcount = 0;

	state->status &=
		~(XNPIPE_USER_ALL_WAIT | XNPIPE_USER_ALL_READY |
		  XNPIPE_USER_SIGIO);

	if ((state->status & XNPIPE_KERN_CONN) == 0) {
		if (file->f_flags & O_NONBLOCK) {
			xnpipe_cleanup_user_conn(state, s);
			xnlock_put_irqrestore(&nklock, s);
			return -EWOULDBLOCK;
		}

		sigpending = xnpipe_wait(state, XNPIPE_USER_WREAD, s,
					 state->status & XNPIPE_KERN_CONN);
		if (sigpending) {
			xnpipe_cleanup_user_conn(state, s);
			xnlock_put_irqrestore(&nklock, s);
			return -ERESTARTSYS;
		}
	}

	if (err)
		xnpipe_cleanup_user_conn(state, s);

	xnlock_put_irqrestore(&nklock, s);

	return err;
}

static int xnpipe_release(struct inode *inode, struct file *file)
{
	struct xnpipe_state *state = file->private_data;
	spl_t s;

	xnlock_get_irqsave(&nklock, s);

	xnpipe_dequeue_all(state, XNPIPE_USER_WREAD);
	xnpipe_dequeue_all(state, XNPIPE_USER_WSYNC);

	if (state->status & XNPIPE_KERN_CONN) {
		/* Unblock waiters. */
		if (xnsynch_pended_p(&state->synchbase)) {
			xnsynch_flush(&state->synchbase, XNRMID);
			xnsched_run();
		}
	}

	if (state->ops.input)
		state->ops.input(NULL, -EPIPE, state->xstate);

	if (state->asyncq) {	/* Clear the async queue */
		list_del(&state->alink);
		state->status &= ~XNPIPE_USER_SIGIO;
		xnlock_put_irqrestore(&nklock, s);
		fasync_helper(-1, file, 0, &state->asyncq);
		xnlock_get_irqsave(&nklock, s);
	}

	xnpipe_cleanup_user_conn(state, s);
	/*
	 * The extra state may not be available from now on, if
	 * xnpipe_disconnect() entered lingering close before we got
	 * there; so calling xnpipe_cleanup_user_conn() should be the
	 * last thing we do.
	 */
	xnlock_put_irqrestore(&nklock, s);

	return 0;
}

static ssize_t xnpipe_read(struct file *file,
			   char *buf, size_t count, loff_t *ppos)
{
	struct xnpipe_state *state = file->private_data;
	int sigpending, err = 0;
	size_t nbytes, inbytes;
	struct xnpipe_mh *mh;
	ssize_t ret;
	spl_t s;

	if (!access_ok(VERIFY_WRITE, buf, count))
		return -EFAULT;

	xnlock_get_irqsave(&nklock, s);

	if ((state->status & XNPIPE_KERN_CONN) == 0) {
		xnlock_put_irqrestore(&nklock, s);
		return -EPIPE;
	}
	/*
	 * Queue probe and proc enqueuing must be seen atomically,
	 * including from the Xenomai side.
	 */
	if (list_empty(&state->outq)) {
		if (file->f_flags & O_NONBLOCK) {
			xnlock_put_irqrestore(&nklock, s);
			return -EWOULDBLOCK;
		}

		sigpending = xnpipe_wait(state, XNPIPE_USER_WREAD, s,
					 !list_empty(&state->outq));

		if (list_empty(&state->outq)) {
			xnlock_put_irqrestore(&nklock, s);
			return sigpending ? -ERESTARTSYS : 0;
		}
	}

	mh = list_get_entry(&state->outq, struct xnpipe_mh, link);
	state->nroutq--;

	/*
	 * We allow more data to be appended to the current message
	 * bucket while its contents is being copied to the user
	 * buffer, therefore, we need to loop until: 1) all the data
	 * has been copied, 2) we consumed the user buffer space
	 * entirely.
	 */

	inbytes = 0;

	for (;;) {
		nbytes = xnpipe_m_size(mh) - xnpipe_m_rdoff(mh);

		if (nbytes + inbytes > count)
			nbytes = count - inbytes;

		if (nbytes == 0)
			break;

		xnlock_put_irqrestore(&nklock, s);

		/* More data could be appended while doing this: */
		err = __copy_to_user(buf + inbytes,
				     xnpipe_m_data(mh) + xnpipe_m_rdoff(mh),
				     nbytes);

		xnlock_get_irqsave(&nklock, s);

		if (err) {
			err = -EFAULT;
			break;
		}

		inbytes += nbytes;
		xnpipe_m_rdoff(mh) += nbytes;
	}

	state->ionrd -= inbytes;
	ret = inbytes;

	if (xnpipe_m_size(mh) > xnpipe_m_rdoff(mh)) {
		list_add(&mh->link, &state->outq);
		state->nroutq++;
	} else {
		/*
		 * We always want to fire the output handler because
		 * whatever the error state is for userland (e.g
		 * -EFAULT), we did pull a message from our output
		 * queue.
		 */
		if (state->ops.output)
			state->ops.output(mh, state->xstate);
		xnlock_put_irqrestore(&nklock, s);
		state->ops.free_obuf(mh, state->xstate);
		xnlock_get_irqsave(&nklock, s);
		if (state->status & XNPIPE_USER_WSYNC) {
			state->status |= XNPIPE_USER_WSYNC_READY;
			xnpipe_schedule_request();
		}
	}

	xnlock_put_irqrestore(&nklock, s);

	return err ? : ret;
}

static ssize_t xnpipe_write(struct file *file,
			    const char *buf, size_t count, loff_t *ppos)
{
	struct xnpipe_state *state = file->private_data;
	struct xnpipe_mh *mh;
	int pollnum, ret;
	spl_t s;

	if (count == 0)
		return 0;

	if (!access_ok(VERIFY_READ, buf, count))
		return -EFAULT;

	xnlock_get_irqsave(&nklock, s);

retry:
	if ((state->status & XNPIPE_KERN_CONN) == 0) {
		xnlock_put_irqrestore(&nklock, s);
		return -EPIPE;
	}

	pollnum = state->nrinq + state->nroutq;
	xnlock_put_irqrestore(&nklock, s);

	mh = state->ops.alloc_ibuf(count + sizeof(*mh), state->xstate);
	if (mh == (struct xnpipe_mh *)-1)
		return -ENOMEM;

	if (mh == NULL) {
		if (file->f_flags & O_NONBLOCK)
			return -EWOULDBLOCK;

		xnlock_get_irqsave(&nklock, s);
		if (xnpipe_wait(state, XNPIPE_USER_WSYNC, s,
				pollnum > state->nrinq + state->nroutq)) {
			xnlock_put_irqrestore(&nklock, s);
			return -ERESTARTSYS;
		}
		goto retry;
	}

	xnpipe_m_size(mh) = count;
	xnpipe_m_rdoff(mh) = 0;

	if (copy_from_user(xnpipe_m_data(mh), buf, count)) {
		state->ops.free_ibuf(mh, state->xstate);
		return -EFAULT;
	}

	xnlock_get_irqsave(&nklock, s);

	list_add_tail(&mh->link, &state->inq);
	state->nrinq++;

	/* Wake up a Xenomai sleeper if any. */
	if (xnsynch_wakeup_one_sleeper(&state->synchbase))
		xnsched_run();

	if (state->ops.input) {
		ret = state->ops.input(mh, 0, state->xstate);
		if (ret)
			count = (size_t)ret;
	}

	if (file->f_flags & O_SYNC) {
		if (!list_empty(&state->inq)) {
			if (xnpipe_wait(state, XNPIPE_USER_WSYNC, s,
					list_empty(&state->inq)))
				count = -ERESTARTSYS;
		}
	}

	xnlock_put_irqrestore(&nklock, s);

	return (ssize_t)count;
}

static long xnpipe_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct xnpipe_state *state = file->private_data;
	int ret = 0;
	ssize_t n;
	spl_t s;

	switch (cmd) {
	case XNPIPEIOC_GET_NRDEV:

		if (put_user(XNPIPE_NDEVS, (int *)arg))
			return -EFAULT;

		break;

	case XNPIPEIOC_OFLUSH:

		xnlock_get_irqsave(&nklock, s);

		if ((state->status & XNPIPE_KERN_CONN) == 0) {
			xnlock_put_irqrestore(&nklock, s);
			return -EPIPE;
		}

		n = xnpipe_flushq(state, outq, free_obuf, s);
		state->ionrd -= n;
		goto kick_wsync;

	case XNPIPEIOC_IFLUSH:

		xnlock_get_irqsave(&nklock, s);

		if ((state->status & XNPIPE_KERN_CONN) == 0) {
			xnlock_put_irqrestore(&nklock, s);
			return -EPIPE;
		}

		n = xnpipe_flushq(state, inq, free_ibuf, s);

	kick_wsync:

		if (n > 0 && (state->status & XNPIPE_USER_WSYNC)) {
			state->status |= XNPIPE_USER_WSYNC_READY;
			xnpipe_schedule_request();
		}

		xnlock_put_irqrestore(&nklock, s);
		ret = n;
		break;

	case XNPIPEIOC_SETSIG:

		if (arg < 1 || arg >= _NSIG)
			return -EINVAL;

		xnpipe_asyncsig = arg;
		break;

	case FIONREAD:

		n = (state->status & XNPIPE_KERN_CONN) ? state->ionrd : 0;

		if (put_user(n, (int *)arg))
			return -EFAULT;

		break;

	case TCGETS:
		/* For isatty() probing. */
		return -ENOTTY;

	default:

		return -EINVAL;
	}

	return ret;
}

static int xnpipe_fasync(int fd, struct file *file, int on)
{
	struct xnpipe_state *state = file->private_data;
	int ret, queued;
	spl_t s;

	queued = (state->asyncq != NULL);
	ret = fasync_helper(fd, file, on, &state->asyncq);

	if (state->asyncq) {
		if (!queued) {
			xnlock_get_irqsave(&nklock, s);
			list_add_tail(&state->alink, &xnpipe_asyncq);
			xnlock_put_irqrestore(&nklock, s);
		}
	} else if (queued) {
		xnlock_get_irqsave(&nklock, s);
		list_del(&state->alink);
		xnlock_put_irqrestore(&nklock, s);
	}

	return ret;
}

static unsigned xnpipe_poll(struct file *file, poll_table *pt)
{
	struct xnpipe_state *state = file->private_data;
	unsigned r_mask = 0, w_mask = 0;
	spl_t s;

	poll_wait(file, &state->readq, pt);

	xnlock_get_irqsave(&nklock, s);

	if (state->status & XNPIPE_KERN_CONN)
		w_mask |= (POLLOUT | POLLWRNORM);
	else
		r_mask |= POLLHUP;

	if (!list_empty(&state->outq))
		r_mask |= (POLLIN | POLLRDNORM);
	else
		/*
		 * Procs which have issued a timed out poll req will
		 * remain linked to the sleepers queue, and will be
		 * silently unlinked the next time the Xenomai side
		 * kicks xnpipe_wakeup_proc().
		 */
		xnpipe_enqueue_wait(state, XNPIPE_USER_WREAD);

	xnlock_put_irqrestore(&nklock, s);

	return r_mask | w_mask;
}

static struct file_operations xnpipe_fops = {
	.read = xnpipe_read,
	.write = xnpipe_write,
	.poll = xnpipe_poll,
	.unlocked_ioctl = xnpipe_ioctl,
	.open = xnpipe_open,
	.release = xnpipe_release,
	.fasync = xnpipe_fasync
};

int xnpipe_mount(void)
{
	struct xnpipe_state *state;
	struct device *cldev;
	int i;

	for (state = &xnpipe_states[0];
	     state < &xnpipe_states[XNPIPE_NDEVS]; state++) {
		state->status = 0;
		state->asyncq = NULL;
		INIT_LIST_HEAD(&state->inq);
		state->nrinq = 0;
		INIT_LIST_HEAD(&state->outq);
		state->nroutq = 0;
	}

	xnpipe_class = class_create(THIS_MODULE, "rtpipe");
	if (IS_ERR(xnpipe_class)) {
		printk(XENO_ERR "error creating rtpipe class, err=%ld\n",
		       PTR_ERR(xnpipe_class));
		return -EBUSY;
	}

	for (i = 0; i < XNPIPE_NDEVS; i++) {
		cldev = device_create(xnpipe_class, NULL,
				      MKDEV(XNPIPE_DEV_MAJOR, i),
				      NULL, "rtp%d", i);
		if (IS_ERR(cldev)) {
			printk(XENO_ERR
			       "can't add device class, major=%d, minor=%d, err=%ld\n",
			       XNPIPE_DEV_MAJOR, i, PTR_ERR(cldev));
			class_destroy(xnpipe_class);
			return -EBUSY;
		}
	}

	if (register_chrdev(XNPIPE_DEV_MAJOR, "rtpipe", &xnpipe_fops)) {
		printk(XENO_ERR
		       "unable to reserve major #%d for message pipe support\n",
		       XNPIPE_DEV_MAJOR);
		return -EBUSY;
	}

	xnpipe_wakeup_apc =
	    xnapc_alloc("pipe_wakeup", &xnpipe_wakeup_proc, NULL);

	return 0;
}

void xnpipe_umount(void)
{
	int i;

	xnapc_free(xnpipe_wakeup_apc);
	unregister_chrdev(XNPIPE_DEV_MAJOR, "rtpipe");

	for (i = 0; i < XNPIPE_NDEVS; i++)
		device_destroy(xnpipe_class, MKDEV(XNPIPE_DEV_MAJOR, i));

	class_destroy(xnpipe_class);
}
