/*
 * Written by Gilles Chanteperdrix <gilles.chanteperdrix@xenomai.org>.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <stdarg.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <cobalt/kernel/select.h>
#include <rtdm/fd.h>
#include "internal.h"
#include "thread.h"
#include "signal.h"
#include "timer.h"
#include "mqueue.h"
#include "clock.h"
#include <trace/events/cobalt-posix.h>

#define COBALT_MSGMAX		65536
#define COBALT_MSGSIZEMAX	(16*1024*1024)
#define COBALT_MSGPRIOMAX	32768

struct cobalt_mq {
	unsigned magic;

	struct list_head link;

	struct xnsynch receivers;
	struct xnsynch senders;
	size_t memsize;
	char *mem;
	struct list_head queued;
	struct list_head avail;
	int nrqueued;

	/* mq_notify */
	struct siginfo si;
	mqd_t target_qd;
	struct cobalt_thread *target;

	struct mq_attr attr;

	unsigned refs;
	char name[COBALT_MAXNAME];
	xnhandle_t handle;

	DECLARE_XNSELECT(read_select);
	DECLARE_XNSELECT(write_select);
};

struct cobalt_mqd {
	struct cobalt_mq *mq;
	struct rtdm_fd fd;
};

struct cobalt_msg {
	struct list_head link;
	unsigned int prio;
	size_t len;
	char data[0];
};

struct cobalt_mqwait_context {
	struct xnthread_wait_context wc;
	struct cobalt_msg *msg;
};

static struct mq_attr default_attr = {
      .mq_maxmsg = 10,
      .mq_msgsize = 8192,
};

static LIST_HEAD(cobalt_mqq);

static inline struct cobalt_msg *mq_msg_alloc(struct cobalt_mq *mq)
{
	if (list_empty(&mq->avail))
		return NULL;

	return list_get_entry(&mq->avail, struct cobalt_msg, link);
}

static inline void mq_msg_free(struct cobalt_mq *mq, struct cobalt_msg * msg)
{
	list_add(&msg->link, &mq->avail); /* For earliest re-use of the block. */
}

static inline int mq_init(struct cobalt_mq *mq, const struct mq_attr *attr)
{
	unsigned i, msgsize, memsize;
	char *mem;

	if (attr == NULL)
		attr = &default_attr;
	else {
		if (attr->mq_maxmsg <= 0 || attr->mq_msgsize <= 0)
			return -EINVAL;
		if (attr->mq_maxmsg > COBALT_MSGMAX)
			return -EINVAL;
		if (attr->mq_msgsize > COBALT_MSGSIZEMAX)
			return -EINVAL;
	}

	msgsize = attr->mq_msgsize + sizeof(struct cobalt_msg);

	/* Align msgsize on natural boundary. */
	if ((msgsize % sizeof(unsigned long)))
		msgsize +=
		    sizeof(unsigned long) - (msgsize % sizeof(unsigned long));

	memsize = msgsize * attr->mq_maxmsg;
	memsize = PAGE_ALIGN(memsize);
	if (get_order(memsize) > MAX_ORDER)
		return -ENOSPC;

	mem = xnheap_vmalloc(memsize);
	if (mem == NULL)
		return -ENOSPC;

	mq->memsize = memsize;
	INIT_LIST_HEAD(&mq->queued);
	mq->nrqueued = 0;
	xnsynch_init(&mq->receivers, XNSYNCH_PRIO | XNSYNCH_NOPIP, NULL);
	xnsynch_init(&mq->senders, XNSYNCH_PRIO | XNSYNCH_NOPIP, NULL);
	mq->mem = mem;

	/* Fill the pool. */
	INIT_LIST_HEAD(&mq->avail);
	for (i = 0; i < attr->mq_maxmsg; i++) {
		struct cobalt_msg *msg = (struct cobalt_msg *) (mem + i * msgsize);
		mq_msg_free(mq, msg);
	}

	mq->attr = *attr;
	mq->target = NULL;
	xnselect_init(&mq->read_select);
	xnselect_init(&mq->write_select);
	mq->magic = COBALT_MQ_MAGIC;
	mq->refs = 2;

	return 0;
}

static inline void mq_destroy(struct cobalt_mq *mq)
{
	int resched;
	spl_t s;

	xnlock_get_irqsave(&nklock, s);
	resched = (xnsynch_destroy(&mq->receivers) == XNSYNCH_RESCHED);
	resched = (xnsynch_destroy(&mq->senders) == XNSYNCH_RESCHED) || resched;
	list_del(&mq->link);
	xnlock_put_irqrestore(&nklock, s);
	xnselect_destroy(&mq->read_select);
	xnselect_destroy(&mq->write_select);
	xnregistry_remove(mq->handle);
	xnheap_vfree(mq->mem);
	kfree(mq);

	if (resched)
		xnsched_run();
}

static int mq_unref_inner(struct cobalt_mq *mq, spl_t s)
{
	int destroy;

	destroy = --mq->refs == 0;
	xnlock_put_irqrestore(&nklock, s);

	if (destroy)
		mq_destroy(mq);

	return destroy;
}

static int mq_unref(struct cobalt_mq *mq)
{
	spl_t s;

	xnlock_get_irqsave(&nklock, s);
	return mq_unref_inner(mq, s);
}

static void mqd_close(struct rtdm_fd *fd)
{
	struct cobalt_mqd *mqd = container_of(fd, struct cobalt_mqd, fd);
	struct cobalt_mq *mq = mqd->mq;

	kfree(mqd);
	mq_unref(mq);
}

int
mqd_select(struct rtdm_fd *fd, struct xnselector *selector,
	   unsigned type, unsigned index)
{
	struct cobalt_mqd *mqd = container_of(fd, struct cobalt_mqd, fd);
	struct xnselect_binding *binding;
	struct cobalt_mq *mq;
	int err;
	spl_t s;

	if (type == XNSELECT_READ || type == XNSELECT_WRITE) {
		binding = xnmalloc(sizeof(*binding));
		if (!binding)
			return -ENOMEM;
	} else
		return -EBADF;

	xnlock_get_irqsave(&nklock, s);
	mq = mqd->mq;

	switch(type) {
	case XNSELECT_READ:
		err = -EBADF;
		if ((rtdm_fd_flags(fd) & COBALT_PERMS_MASK) == O_WRONLY)
			goto unlock_and_error;

		err = xnselect_bind(&mq->read_select, binding,
				selector, type, index,
				!list_empty(&mq->queued));
		if (err)
			goto unlock_and_error;
		break;

	case XNSELECT_WRITE:
		err = -EBADF;
		if ((rtdm_fd_flags(fd) & COBALT_PERMS_MASK) == O_RDONLY)
			goto unlock_and_error;

		err = xnselect_bind(&mq->write_select, binding,
				selector, type, index,
				!list_empty(&mq->avail));
		if (err)
			goto unlock_and_error;
		break;
	}
	xnlock_put_irqrestore(&nklock, s);
	return 0;

      unlock_and_error:
	xnlock_put_irqrestore(&nklock, s);
	xnfree(binding);
	return err;
}

static struct rtdm_fd_ops mqd_ops = {
	.close = mqd_close,
	.select = mqd_select,
};

static inline int mqd_create(struct cobalt_mq *mq, unsigned long flags, int ufd)
{
	struct cobalt_mqd *mqd;

	if (cobalt_ppd_get(0) == &cobalt_kernel_ppd)
		return -EPERM;

	mqd = kmalloc(sizeof(*mqd), GFP_KERNEL);
	if (mqd == NULL)
		return -ENOSPC;

	mqd->fd.oflags = flags;
	mqd->mq = mq;

	return rtdm_fd_enter(&mqd->fd, ufd, COBALT_MQD_MAGIC, &mqd_ops);
}

static int mq_open(int uqd, const char *name, int oflags,
		   int mode, struct mq_attr *attr)
{
	struct cobalt_mq *mq;
	xnhandle_t handle;
	spl_t s;
	int err;

	if (name[0] != '/' || name[1] == '\0')
		return -EINVAL;

  retry_bind:
	err = xnregistry_bind(&name[1], XN_NONBLOCK, XN_RELATIVE, &handle);
	switch (err) {
	case 0:
		/* Found */
		if ((oflags & (O_CREAT | O_EXCL)) == (O_CREAT | O_EXCL))
			return -EEXIST;

		xnlock_get_irqsave(&nklock, s);
		mq = xnregistry_lookup(handle, NULL);
		if (mq && mq->magic != COBALT_MQ_MAGIC) {
			xnlock_put_irqrestore(&nklock, s);
			return -EINVAL;
		}

		if (mq) {
			++mq->refs;
			xnlock_put_irqrestore(&nklock, s);
		} else {
			xnlock_put_irqrestore(&nklock, s);
			goto retry_bind;
		}

		err = mqd_create(mq, oflags & (O_NONBLOCK | COBALT_PERMS_MASK),
				uqd);
		if (err < 0) {
			mq_unref(mq);
			return err;
		}
		break;

	case -EWOULDBLOCK:
		/* Not found */
		if ((oflags & O_CREAT) == 0)
			return (mqd_t)-ENOENT;

		mq = kmalloc(sizeof(*mq), GFP_KERNEL);
		if (mq == NULL)
			return -ENOSPC;

		err = mq_init(mq, attr);
		if (err) {
			kfree(mq);
			return err;
		}

		snprintf(mq->name, sizeof(mq->name), "%s", &name[1]);

		err = mqd_create(mq, oflags & (O_NONBLOCK | COBALT_PERMS_MASK),
				uqd);
		if (err < 0) {
			mq_destroy(mq);
			return err;
		}

		xnlock_get_irqsave(&nklock, s);
		err = xnregistry_enter(mq->name, mq, &mq->handle, NULL);
		if (err < 0)
			--mq->refs;
		else
			list_add_tail(&mq->link, &cobalt_mqq);
		xnlock_put_irqrestore(&nklock, s);
		if (err < 0) {
			rtdm_fd_close(uqd, COBALT_MQD_MAGIC);
			if (err == -EEXIST)
				goto retry_bind;
			return err;
		}
		break;

	default:
		return err;
	}

	return 0;
}

static inline int mq_close(mqd_t fd)
{
	return rtdm_fd_close(fd, COBALT_MQD_MAGIC);
}

static inline int mq_unlink(const char *name)
{
	struct cobalt_mq *mq;
	xnhandle_t handle;
	spl_t s;
	int err;

	if (name[0] != '/' || name[1] == '\0')
		return -EINVAL;

	err = xnregistry_bind(&name[1], XN_NONBLOCK, XN_RELATIVE, &handle);
	if (err == -EWOULDBLOCK)
		return -ENOENT;
	if (err)
		return err;

	xnlock_get_irqsave(&nklock, s);
	mq = xnregistry_lookup(handle, NULL);
	if (!mq) {
		err = -ENOENT;
		goto err_unlock;
	}
	if (mq->magic != COBALT_MQ_MAGIC) {
		err = -EINVAL;
	  err_unlock:
		xnlock_put_irqrestore(&nklock, s);

		return err;
	}
	if (mq_unref_inner(mq, s) == 0)
		xnregistry_unlink(&name[1]);
	return 0;
}

static inline struct cobalt_msg *
mq_trysend(struct cobalt_mqd *mqd, size_t len)
{
	struct cobalt_msg *msg;
	struct cobalt_mq *mq;
	unsigned flags;

	mq = mqd->mq;
	flags = rtdm_fd_flags(&mqd->fd) & COBALT_PERMS_MASK;

	if (flags != O_WRONLY && flags != O_RDWR)
		return ERR_PTR(-EBADF);

	if (len > mq->attr.mq_msgsize)
		return ERR_PTR(-EMSGSIZE);

	msg = mq_msg_alloc(mq);
	if (msg == NULL)
		return ERR_PTR(-EAGAIN);

	if (list_empty(&mq->avail))
		xnselect_signal(&mq->write_select, 0);

	return msg;
}

static inline struct cobalt_msg *
mq_tryrcv(struct cobalt_mqd *mqd, size_t len)
{
	struct cobalt_msg *msg;
	unsigned int flags;
	struct cobalt_mq *mq;

	mq = mqd->mq;
	flags = rtdm_fd_flags(&mqd->fd) & COBALT_PERMS_MASK;

	if (flags != O_RDONLY && flags != O_RDWR)
		return ERR_PTR(-EBADF);

	if (len < mq->attr.mq_msgsize)
		return ERR_PTR(-EMSGSIZE);

	if (list_empty(&mq->queued))
		return ERR_PTR(-EAGAIN);

	msg = list_get_entry(&mq->queued, struct cobalt_msg, link);
	mq->nrqueued--;

	if (list_empty(&mq->queued))
		xnselect_signal(&mq->read_select, 0);

	return msg;
}

static struct cobalt_msg *
mq_timedsend_inner(struct cobalt_mqd *mqd,
		   size_t len, const void __user *u_ts,
		   int (*fetch_timeout)(struct timespec *ts,
					const void __user *u_ts))
{
	struct cobalt_mqwait_context mwc;
	struct cobalt_msg *msg;
	struct cobalt_mq *mq;
	struct timespec ts;
	xntmode_t tmode;
	xnticks_t to;
	spl_t s;
	int ret;

	to = XN_INFINITE;
	tmode = XN_RELATIVE;
redo:
	xnlock_get_irqsave(&nklock, s);
	msg = mq_trysend(mqd, len);
	if (msg != ERR_PTR(-EAGAIN))
		goto out;

	if (rtdm_fd_flags(&mqd->fd) & O_NONBLOCK)
		goto out;

	if (fetch_timeout) {
		xnlock_put_irqrestore(&nklock, s);
		ret = fetch_timeout(&ts, u_ts);
		if (ret)
			return ERR_PTR(ret);
		if ((unsigned long)ts.tv_nsec >= ONE_BILLION)
			return ERR_PTR(-EINVAL);
		to = ts2ns(&ts) + 1;
		tmode = XN_REALTIME;
		fetch_timeout = NULL;
		goto redo;
	}

	mq = mqd->mq;
	xnthread_prepare_wait(&mwc.wc);
	ret = xnsynch_sleep_on(&mq->senders, to, tmode);
	if (ret) {
		if (ret & XNBREAK)
			msg = ERR_PTR(-EINTR);
		else if (ret & XNTIMEO)
			msg = ERR_PTR(-ETIMEDOUT);
		else if (ret & XNRMID)
			msg = ERR_PTR(-EBADF);
	} else
		msg = mwc.msg;
out:
	xnlock_put_irqrestore(&nklock, s);

	return msg;
}

static void mq_release_msg(struct cobalt_mq *mq, struct cobalt_msg *msg)
{
	struct cobalt_mqwait_context *mwc;
	struct xnthread_wait_context *wc;
	struct xnthread *thread;

	/*
	 * Try passing the free message slot to a waiting sender, link
	 * it to the free queue otherwise.
	 */
	if (xnsynch_pended_p(&mq->senders)) {
		thread = xnsynch_wakeup_one_sleeper(&mq->senders);
		wc = xnthread_get_wait_context(thread);
		mwc = container_of(wc, struct cobalt_mqwait_context, wc);
		mwc->msg = msg;
		xnthread_complete_wait(wc);
	} else {
		mq_msg_free(mq, msg);
		if (list_is_singular(&mq->avail))
			xnselect_signal(&mq->write_select, 1);
	}
}

static int
mq_finish_send(struct cobalt_mqd *mqd, struct cobalt_msg *msg)
{
	struct cobalt_mqwait_context *mwc;
	struct xnthread_wait_context *wc;
	struct cobalt_sigpending *sigp;
	struct xnthread *thread;
	struct cobalt_mq *mq;
	spl_t s;

	mq = mqd->mq;

	xnlock_get_irqsave(&nklock, s);
	/* Can we do pipelined sending? */
	if (xnsynch_pended_p(&mq->receivers)) {
		thread = xnsynch_wakeup_one_sleeper(&mq->receivers);
		wc = xnthread_get_wait_context(thread);
		mwc = container_of(wc, struct cobalt_mqwait_context, wc);
		mwc->msg = msg;
		xnthread_complete_wait(wc);
	} else {
		/* Nope, have to go through the queue. */
		list_add_priff(msg, &mq->queued, prio, link);
		mq->nrqueued++;

		/*
		 * If first message and no pending reader, send a
		 * signal if notification was enabled via mq_notify().
		 */
		if (list_is_singular(&mq->queued)) {
			xnselect_signal(&mq->read_select, 1);
			if (mq->target) {
				sigp = cobalt_signal_alloc();
				if (sigp) {
					cobalt_copy_siginfo(SI_MESGQ, &sigp->si, &mq->si);
					if (cobalt_signal_send(mq->target, sigp, 0) <= 0)
						cobalt_signal_free(sigp);
				}
				mq->target = NULL;
			}
		}
	}
	xnsched_run();
	xnlock_put_irqrestore(&nklock, s);

	return 0;
}

static struct cobalt_msg *
mq_timedrcv_inner(struct cobalt_mqd *mqd,
		  size_t len,
		  const void __user *u_ts,
		  int (*fetch_timeout)(struct timespec *ts,
				       const void __user *u_ts))
{
	struct cobalt_mqwait_context mwc;
	struct cobalt_msg *msg;
	struct cobalt_mq *mq;
	struct timespec ts;
	xntmode_t tmode;
	xnticks_t to;
	spl_t s;
	int ret;

	to = XN_INFINITE;
	tmode = XN_RELATIVE;
redo:
	xnlock_get_irqsave(&nklock, s);
	msg = mq_tryrcv(mqd, len);
	if (msg != ERR_PTR(-EAGAIN))
		goto out;

	if (rtdm_fd_flags(&mqd->fd) & O_NONBLOCK)
		goto out;

	if (fetch_timeout) {
		xnlock_put_irqrestore(&nklock, s);
		ret = fetch_timeout(&ts, u_ts);
		if (ret)
			return ERR_PTR(ret);
		if (ts.tv_nsec >= ONE_BILLION)
			return ERR_PTR(-EINVAL);
		to = ts2ns(&ts) + 1;
		tmode = XN_REALTIME;
		fetch_timeout = NULL;
		goto redo;
	}

	mq = mqd->mq;
	xnthread_prepare_wait(&mwc.wc);
	ret = xnsynch_sleep_on(&mq->receivers, to, tmode);
	if (ret == 0)
		msg = mwc.msg;
	else if (ret & XNRMID)
		msg = ERR_PTR(-EBADF);
	else if (ret & XNTIMEO)
		msg = ERR_PTR(-ETIMEDOUT);
	else
		msg = ERR_PTR(-EINTR);
out:
	xnlock_put_irqrestore(&nklock, s);

	return msg;
}

static int
mq_finish_rcv(struct cobalt_mqd *mqd, struct cobalt_msg *msg)
{
	spl_t s;

	xnlock_get_irqsave(&nklock, s);
	mq_release_msg(mqd->mq, msg);
	xnsched_run();
	xnlock_put_irqrestore(&nklock, s);

	return 0;
}

static inline int mq_getattr(struct cobalt_mqd *mqd, struct mq_attr *attr)
{
	struct cobalt_mq *mq;
	spl_t s;

	mq = mqd->mq;
	*attr = mq->attr;
	xnlock_get_irqsave(&nklock, s);
	attr->mq_flags = rtdm_fd_flags(&mqd->fd);
	attr->mq_curmsgs = mq->nrqueued;
	xnlock_put_irqrestore(&nklock, s);

	return 0;
}

static inline int
mq_notify(struct cobalt_mqd *mqd, unsigned index, const struct sigevent *evp)
{
	struct cobalt_thread *thread = cobalt_current_thread();
	struct cobalt_mq *mq;
	int err;
	spl_t s;

	if (evp && ((evp->sigev_notify != SIGEV_SIGNAL &&
		     evp->sigev_notify != SIGEV_NONE) ||
		    (unsigned int)(evp->sigev_signo - 1) > SIGRTMAX - 1))
		return -EINVAL;

	if (xnsched_interrupt_p() || thread == NULL)
		return -EPERM;

	xnlock_get_irqsave(&nklock, s);
	mq = mqd->mq;
	if (mq->target && mq->target != thread) {
		err = -EBUSY;
		goto unlock_and_error;
	}

	if (evp == NULL || evp->sigev_notify == SIGEV_NONE)
		/* Here, mq->target == cobalt_current_thread() or NULL. */
		mq->target = NULL;
	else {
		mq->target = thread;
		mq->target_qd = index;
		mq->si.si_signo = evp->sigev_signo;
		mq->si.si_errno = 0;
		mq->si.si_code = SI_MESGQ;
		mq->si.si_value = evp->sigev_value;
		/*
		 * XXX: we differ from the regular kernel here, which
		 * passes the sender's pid/uid data into the
		 * receiver's namespaces. We pass the receiver's creds
		 * into the init namespace instead.
		 */
		mq->si.si_pid = task_pid_nr(current);
		mq->si.si_uid = get_current_uuid();
	}

	xnlock_put_irqrestore(&nklock, s);
	return 0;

      unlock_and_error:
	xnlock_put_irqrestore(&nklock, s);
	return err;
}

static inline struct cobalt_mqd *cobalt_mqd_get(mqd_t ufd)
{
	struct rtdm_fd *fd;

	fd = rtdm_fd_get(ufd, COBALT_MQD_MAGIC);
	if (IS_ERR(fd)) {
		int err = PTR_ERR(fd);
		if (err == -EBADF && cobalt_current_process() == NULL)
			err = -EPERM;
		return ERR_PTR(err);
	}

	return container_of(fd, struct cobalt_mqd, fd);
}

static inline void cobalt_mqd_put(struct cobalt_mqd *mqd)
{
	rtdm_fd_put(&mqd->fd);
}

int __cobalt_mq_notify(mqd_t fd, const struct sigevent *evp)
{
	struct cobalt_mqd *mqd;
	int ret;

	mqd = cobalt_mqd_get(fd);
	if (IS_ERR(mqd))
		ret = PTR_ERR(mqd);
	else {
		trace_cobalt_mq_notify(fd, evp);
		ret = mq_notify(mqd, fd, evp);
		cobalt_mqd_put(mqd);
	}

	return ret;
}

COBALT_SYSCALL(mq_notify, primary,
	       (mqd_t fd, const struct sigevent *__user evp))
{
	struct sigevent sev;

	if (evp && cobalt_copy_from_user(&sev, evp, sizeof(sev)))
		return -EFAULT;

	return __cobalt_mq_notify(fd, evp ? &sev : NULL);
}

int __cobalt_mq_open(const char __user *u_name, int oflags,
		     mode_t mode, struct mq_attr *attr)
{
	char name[COBALT_MAXNAME];
	unsigned int len;
	mqd_t uqd;
	int ret;

	len = cobalt_strncpy_from_user(name, u_name, sizeof(name));
	if (len < 0)
		return -EFAULT;

	if (len >= sizeof(name))
		return -ENAMETOOLONG;

	if (len == 0)
		return -EINVAL;

	trace_cobalt_mq_open(name, oflags, mode);

	uqd = __rtdm_anon_getfd("[cobalt-mq]", oflags);
	if (uqd < 0)
		return uqd;

	ret = mq_open(uqd, name, oflags, mode, attr);
	if (ret < 0) {
		__rtdm_anon_putfd(uqd);
		return ret;
	}

	return uqd;
}

COBALT_SYSCALL(mq_open, lostage,
	       (const char __user *u_name, int oflags,
		mode_t mode, struct mq_attr __user *u_attr))
{
	struct mq_attr _attr, *attr = &_attr;

	if ((oflags & O_CREAT) && u_attr) {
		if (cobalt_copy_from_user(&_attr, u_attr, sizeof(_attr)))
			return -EFAULT;
	} else
		attr = NULL;

	return __cobalt_mq_open(u_name, oflags, mode, attr);
}

COBALT_SYSCALL(mq_close, lostage, (mqd_t uqd))
{
	trace_cobalt_mq_close(uqd);

	return mq_close(uqd);
}

COBALT_SYSCALL(mq_unlink, lostage, (const char __user *u_name))
{
	char name[COBALT_MAXNAME];
	unsigned len;

	len = cobalt_strncpy_from_user(name, u_name, sizeof(name));
	if (len < 0)
		return -EFAULT;
	if (len >= sizeof(name))
		return -ENAMETOOLONG;

	trace_cobalt_mq_unlink(name);

	return mq_unlink(name);
}

int __cobalt_mq_getattr(mqd_t uqd, struct mq_attr *attr)
{
	struct cobalt_mqd *mqd;
	int ret;

	mqd = cobalt_mqd_get(uqd);
	if (IS_ERR(mqd))
		return PTR_ERR(mqd);

	ret = mq_getattr(mqd, attr);
	cobalt_mqd_put(mqd);
	if (ret)
		return ret;

	trace_cobalt_mq_getattr(uqd, attr);

	return 0;
}

COBALT_SYSCALL(mq_getattr, current,
	       (mqd_t uqd, struct mq_attr __user *u_attr))
{
	struct mq_attr attr;
	int ret;

	ret = __cobalt_mq_getattr(uqd, &attr);
	if (ret)
		return ret;

	return cobalt_copy_to_user(u_attr, &attr, sizeof(attr));
}

static inline int mq_fetch_timeout(struct timespec *ts,
				   const void __user *u_ts)
{
	return u_ts == NULL ? -EFAULT :
		cobalt_copy_from_user(ts, u_ts, sizeof(*ts));
}

int __cobalt_mq_timedsend(mqd_t uqd, const void __user *u_buf, size_t len,
			  unsigned int prio, const void __user *u_ts,
			  int (*fetch_timeout)(struct timespec *ts,
					       const void __user *u_ts))
{
	struct cobalt_msg *msg;
	struct cobalt_mqd *mqd;
	int ret;

	mqd = cobalt_mqd_get(uqd);
	if (IS_ERR(mqd))
		return PTR_ERR(mqd);

	if (prio >= COBALT_MSGPRIOMAX) {
		ret = -EINVAL;
		goto out;
	}

	if (len > 0 && !access_rok(u_buf, len)) {
		ret = -EFAULT;
		goto out;
	}

	trace_cobalt_mq_send(uqd, u_buf, len, prio);
	msg = mq_timedsend_inner(mqd, len, u_ts, fetch_timeout);
	if (IS_ERR(msg)) {
		ret = PTR_ERR(msg);
		goto out;
	}

	ret = cobalt_copy_from_user(msg->data, u_buf, len);
	if (ret) {
		mq_finish_rcv(mqd, msg);
		goto out;
	}
	msg->len = len;
	msg->prio = prio;
	ret = mq_finish_send(mqd, msg);
out:
	cobalt_mqd_put(mqd);

	return ret;
}

COBALT_SYSCALL(mq_timedsend, primary,
	       (mqd_t uqd, const void __user *u_buf, size_t len,
		unsigned int prio, const struct timespec __user *u_ts))
{
	return __cobalt_mq_timedsend(uqd, u_buf, len, prio,
				     u_ts, u_ts ? mq_fetch_timeout : NULL);
}

int __cobalt_mq_timedreceive(mqd_t uqd, void __user *u_buf,
			     ssize_t *lenp,
			     unsigned int __user *u_prio,
			     const void __user *u_ts,
			     int (*fetch_timeout)(struct timespec *ts,
						  const void __user *u_ts))
{
	struct cobalt_mqd *mqd;
	struct cobalt_msg *msg;
	unsigned int prio;
	int ret;

	mqd = cobalt_mqd_get(uqd);
	if (IS_ERR(mqd))
		return PTR_ERR(mqd);

	if (*lenp > 0 && !access_wok(u_buf, *lenp)) {
		ret = -EFAULT;
		goto fail;
	}

	msg = mq_timedrcv_inner(mqd, *lenp, u_ts, fetch_timeout);
	if (IS_ERR(msg)) {
		ret = PTR_ERR(msg);
		goto fail;
	}

	ret = cobalt_copy_to_user(u_buf, msg->data, msg->len);
	if (ret) {
		mq_finish_rcv(mqd, msg);
		goto fail;
	}

	*lenp = msg->len;
	prio = msg->prio;
	ret = mq_finish_rcv(mqd, msg);
	if (ret)
		goto fail;

	cobalt_mqd_put(mqd);

	if (u_prio && __xn_put_user(prio, u_prio))
		return -EFAULT;

	return 0;
fail:
	cobalt_mqd_put(mqd);

	return ret;
}

COBALT_SYSCALL(mq_timedreceive, primary,
	       (mqd_t uqd, void __user *u_buf,
		ssize_t __user *u_len,
		unsigned int __user *u_prio,
		const struct timespec __user *u_ts))
{
	ssize_t len;
	int ret;

	ret = cobalt_copy_from_user(&len, u_len, sizeof(len));
	if (ret)
		return ret;

	ret = __cobalt_mq_timedreceive(uqd, u_buf, &len, u_prio,
				       u_ts, u_ts ? mq_fetch_timeout : NULL);

	return ret ?: cobalt_copy_to_user(u_len, &len, sizeof(*u_len));
}
