/*
 * Copyright (C) 2005 Jan Kiszka <jan.kiszka@web.de>
 * Copyright (C) 2005 Joerg Langenberg <joerg.langenberg@gmx.net>
 * Copyright (C) 2013,2014 Gilles Chanteperdrix <gch@xenomai.org>.
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
#include <linux/list.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/poll.h>
#include <linux/kthread.h>
#include <linux/fdtable.h>
#include <cobalt/kernel/registry.h>
#include <cobalt/kernel/lock.h>
#include <cobalt/kernel/ppd.h>
#include <trace/events/cobalt-rtdm.h>
#include <rtdm/fd.h>
#include "internal.h"
#include "posix/process.h"
#include "posix/syscall.h"

#define RTDM_SETFL_MASK (O_NONBLOCK)

DEFINE_PRIVATE_XNLOCK(fdtree_lock);
static LIST_HEAD(rtdm_fd_cleanup_queue);
static struct semaphore rtdm_fd_cleanup_sem;

struct rtdm_fd_index {
	struct xnid id;
	struct rtdm_fd *fd;
};

static int enosys(void)
{
	return -ENOSYS;
}

static int enodev(void)
{
	return -ENODEV;
}

static void nop_close(struct rtdm_fd *fd)
{
}

static inline struct rtdm_fd_index *
fetch_fd_index(struct cobalt_ppd *p, int ufd)
{
	struct xnid *id = xnid_fetch(&p->fds, ufd);
	if (id == NULL)
		return NULL;

	return container_of(id, struct rtdm_fd_index, id);
}

static struct rtdm_fd *fetch_fd(struct cobalt_ppd *p, int ufd)
{
	struct rtdm_fd_index *idx = fetch_fd_index(p, ufd);
	if (idx == NULL)
		return NULL;

	return idx->fd;
}

#define assign_invalid_handler(__handler)				\
	do								\
		(__handler) = (typeof(__handler))enodev;		\
	while (0)

/* Calling this handler should beget ENODEV if not implemented. */
#define assign_invalid_default_handler(__handler)			\
	do								\
		if ((__handler) == NULL)				\
			(__handler) = (typeof(__handler))enodev;	\
	while (0)

#define __assign_default_handler(__handler, __placeholder)		\
	do								\
		if ((__handler) == NULL)				\
			(__handler) = (typeof(__handler))__placeholder;	\
	while (0)

/* Calling this handler should beget ENOSYS if not implemented. */
#define assign_default_handler(__handler)				\
	__assign_default_handler(__handler, enosys)

#define __rt(__handler)		__handler ## _rt
#define __nrt(__handler)	__handler ## _nrt

/*
 * Install a placeholder returning ENODEV if none of the dual handlers
 * are implemented, ENOSYS otherwise for NULL handlers to trigger the
 * adaptive switch.
 */
#define assign_default_dual_handlers(__handler)				\
	do								\
		if (__rt(__handler) || __nrt(__handler)) {		\
			assign_default_handler(__rt(__handler));	\
			assign_default_handler(__nrt(__handler));	\
		} else {						\
			assign_invalid_handler(__rt(__handler));	\
			assign_invalid_handler(__nrt(__handler));	\
		}							\
	while (0)

#ifdef CONFIG_XENO_ARCH_SYS3264

static inline void set_compat_bit(struct rtdm_fd *fd)
{
	struct pt_regs *regs;

	if (cobalt_ppd_get(0) == &cobalt_kernel_ppd)
		fd->compat = 0;
	else {
		regs = task_pt_regs(current);
		XENO_BUG_ON(COBALT, !__xn_syscall_p(regs));
		fd->compat = __COBALT_CALL_COMPAT(__xn_reg_sys(regs));
	}
}

#else	/* !CONFIG_XENO_ARCH_SYS3264 */

static inline void set_compat_bit(struct rtdm_fd *fd)
{
}

#endif	/* !CONFIG_XENO_ARCH_SYS3264 */

int rtdm_fd_enter(struct rtdm_fd *fd, int ufd, unsigned int magic,
		  struct rtdm_fd_ops *ops)
{
	struct rtdm_fd_index *idx;
	struct cobalt_ppd *ppd;
	spl_t s;
	int ret;

	secondary_mode_only();

	if (magic == 0)
		return -EINVAL;

	idx = kmalloc(sizeof(*idx), GFP_KERNEL);
	if (idx == NULL)
		return -ENOMEM;

	assign_default_dual_handlers(ops->ioctl);
	assign_default_dual_handlers(ops->read);
	assign_default_dual_handlers(ops->write);
	assign_default_dual_handlers(ops->recvmsg);
	assign_default_dual_handlers(ops->sendmsg);
	assign_invalid_default_handler(ops->select);
	assign_invalid_default_handler(ops->mmap);
	__assign_default_handler(ops->close, nop_close);

	ppd = cobalt_ppd_get(0);
	fd->magic = magic;
	fd->ops = ops;
	fd->owner = ppd;
	fd->refs = 1;
	set_compat_bit(fd);

	idx->fd = fd;

	xnlock_get_irqsave(&fdtree_lock, s);
	ret = xnid_enter(&ppd->fds, &idx->id, ufd);
	xnlock_put_irqrestore(&fdtree_lock, s);
	if (ret < 0) {
		kfree(idx);
		ret = -EBUSY;
	}

	return ret;
}

/**
 * @brief Retrieve and lock a RTDM file descriptor
 *
 * @param[in] ufd User-side file descriptor
 * @param[in] magic Magic word for lookup validation
 *
 * @return Pointer to the RTDM file descriptor matching @a ufd, or
 * ERR_PTR(-EBADF).
 *
 * @note The file descriptor returned must be later released by a call
 * to rtdm_fd_put().
 *
 * @coretags{unrestricted}
 */
struct rtdm_fd *rtdm_fd_get(int ufd, unsigned int magic)
{
	struct cobalt_ppd *p = cobalt_ppd_get(0);
	struct rtdm_fd *fd;
	spl_t s;

	xnlock_get_irqsave(&fdtree_lock, s);
	fd = fetch_fd(p, ufd);
	if (fd == NULL || (magic != 0 && fd->magic != magic)) {
		fd = ERR_PTR(-EBADF);
		goto out;
	}

	++fd->refs;
out:
	xnlock_put_irqrestore(&fdtree_lock, s);

	return fd;
}
EXPORT_SYMBOL_GPL(rtdm_fd_get);

struct lostage_trigger_close {
	struct ipipe_work_header work; /* Must be first */
};

static int fd_cleanup_thread(void *data)
{
	struct rtdm_fd *fd;
	int err;
	spl_t s;

	for (;;) {
		set_cpus_allowed_ptr(current, cpu_online_mask);

		do {
			err = down_killable(&rtdm_fd_cleanup_sem);
		} while (err && !kthread_should_stop());

		if (kthread_should_stop())
			break;

		xnlock_get_irqsave(&fdtree_lock, s);
		fd = list_first_entry(&rtdm_fd_cleanup_queue,
				struct rtdm_fd, cleanup);
		list_del(&fd->cleanup);
		xnlock_put_irqrestore(&fdtree_lock, s);

		fd->ops->close(fd);
	}

	return 0;
}

static void lostage_trigger_close(struct ipipe_work_header *work)
{
	up(&rtdm_fd_cleanup_sem);
}

static void __put_fd(struct rtdm_fd *fd, spl_t s)
{
	int destroy;

	destroy = --fd->refs == 0;
	xnlock_put_irqrestore(&fdtree_lock, s);

	if (!destroy)
		return;

	if (ipipe_root_p)
		fd->ops->close(fd);
	else {
		struct lostage_trigger_close closework = {
			.work = {
				.size = sizeof(closework),
				.handler = lostage_trigger_close,
			},
		};

		xnlock_get_irqsave(&fdtree_lock, s);
		list_add_tail(&fd->cleanup, &rtdm_fd_cleanup_queue);
		xnlock_put_irqrestore(&fdtree_lock, s);

		ipipe_post_work_root(&closework, work);
	}
}

/**
 * @brief Release a RTDM file descriptor obtained via rtdm_fd_get()
 *
 * @param[in] fd RTDM file descriptor to release
 *
 * @note Every call to rtdm_fd_get() must be matched by a call to
 * rtdm_fd_put().
 *
 * @coretags{unrestricted}
 */
void rtdm_fd_put(struct rtdm_fd *fd)
{
	spl_t s;

	xnlock_get_irqsave(&fdtree_lock, s);
	__put_fd(fd, s);
}
EXPORT_SYMBOL_GPL(rtdm_fd_put);

/**
 * @brief Hold a reference on a RTDM file descriptor
 *
 * @param[in] fd Target file descriptor
 *
 * @note rtdm_fd_lock() increments the reference counter of @a fd. You
 * only need to call this function in special scenarios, e.g. when
 * keeping additional references to the file descriptor that have
 * different lifetimes. Only use rtdm_fd_lock() on descriptors that
 * are currently locked via an earlier rtdm_fd_get()/rtdm_fd_lock() or
 * while running a device operation handler.
 *
 * @coretags{unrestricted}
 */
int rtdm_fd_lock(struct rtdm_fd *fd)
{
	spl_t s;

	xnlock_get_irqsave(&fdtree_lock, s);
	if (fd->refs == 0) {
		xnlock_put_irqrestore(&fdtree_lock, s);
		return -EIDRM;
	}
	++fd->refs;
	xnlock_put_irqrestore(&fdtree_lock, s);

	return 0;
}
EXPORT_SYMBOL_GPL(rtdm_fd_lock);

/**
 * @brief Drop a reference on a RTDM file descriptor
 *
 * @param[in] fd Target file descriptor
 *
 * @note Every call to rtdm_fd_lock() must be matched by a call to
 * rtdm_fd_unlock().
 *
 * @coretags{unrestricted}
 */
void rtdm_fd_unlock(struct rtdm_fd *fd)
{
	spl_t s;

	xnlock_get_irqsave(&fdtree_lock, s);
	/* Warn if fd was unreferenced. */
	XENO_WARN_ON(COBALT, fd->refs <= 0);
	__put_fd(fd, s);
}
EXPORT_SYMBOL_GPL(rtdm_fd_unlock);

int rtdm_fd_fcntl(int ufd, int cmd, ...)
{
	struct rtdm_fd *fd;
	va_list ap;
	int arg;
	int ret;

	fd = rtdm_fd_get(ufd, 0);
	if (IS_ERR(fd))
		return PTR_ERR(fd);

	va_start(ap, cmd);
	arg = va_arg(ap, int);
	va_end(ap);

	switch (cmd) {
	case F_GETFL:
		ret = fd->oflags;
		break;
	case F_SETFL:
		fd->oflags = (fd->oflags & ~RTDM_SETFL_MASK) |
			(arg & RTDM_SETFL_MASK);
		ret = 0;
		break;
	default:
		ret = -EINVAL;
	}

	rtdm_fd_put(fd);

	return ret;
}
EXPORT_SYMBOL_GPL(rtdm_fd_fcntl);

static struct rtdm_fd *get_fd_fixup_mode(int ufd)
{
	struct xnthread *thread;
	struct rtdm_fd *fd;
	
	fd = rtdm_fd_get(ufd, 0);
	if (IS_ERR(fd))
		return fd;

	/*
	 * Mode is selected according to the following convention:
	 *
	 * - Cobalt threads must try running the syscall from primary
	 * mode as a first attempt, regardless of their scheduling
	 * class. The driver handler may ask for demoting the caller
	 * to secondary mode by returning -ENOSYS.
	 *
	 * - Regular threads (i.e. not bound to Cobalt) may only run
	 * the syscall from secondary mode.
	 */
	thread = xnthread_current();
	if (unlikely(ipipe_root_p)) {
		if (thread == NULL ||
		    xnthread_test_localinfo(thread, XNDESCENT))
			return fd;
	} else if (likely(thread))
		return fd;

	/*
	 * We need to switch to the converse mode. Since all callers
	 * bear the "adaptive" tag, we just pass -ENOSYS back to the
	 * syscall dispatcher to get switched to the next mode.
	 */
	rtdm_fd_put(fd);

	return ERR_PTR(-ENOSYS);
}

int rtdm_fd_ioctl(int ufd, unsigned int request, ...)
{
	struct rtdm_fd *fd;
	void __user *arg;
	va_list args;
	int err, ret;

	fd = get_fd_fixup_mode(ufd);
	if (IS_ERR(fd)) {
		err = PTR_ERR(fd);
		goto out;
	}

	va_start(args, request);
	arg = va_arg(args, void __user *);
	va_end(args);

	set_compat_bit(fd);

	trace_cobalt_fd_ioctl(current, fd, ufd, request);

	if (ipipe_root_p)
		err = fd->ops->ioctl_nrt(fd, request, arg);
	else
		err = fd->ops->ioctl_rt(fd, request, arg);

	if (!XENO_ASSERT(COBALT, !spltest()))
		splnone();

	if (err < 0) {
		ret = __rtdm_dev_ioctl_core(fd, request, arg);
		if (ret != -ENOSYS)
			err = ret;
	}

	rtdm_fd_put(fd);
  out:
	if (err < 0)
		trace_cobalt_fd_ioctl_status(current, fd, ufd, err);

	return err;
}
EXPORT_SYMBOL_GPL(rtdm_fd_ioctl);

ssize_t
rtdm_fd_read(int ufd, void __user *buf, size_t size)
{
	struct rtdm_fd *fd;
	ssize_t ret;

	fd = get_fd_fixup_mode(ufd);
	if (IS_ERR(fd)) {
		ret = PTR_ERR(fd);
		goto out;
	}

	set_compat_bit(fd);

	trace_cobalt_fd_read(current, fd, ufd, size);

	if (ipipe_root_p)
		ret = fd->ops->read_nrt(fd, buf, size);
	else
		ret = fd->ops->read_rt(fd, buf, size);

	if (!XENO_ASSERT(COBALT, !spltest()))
		    splnone();

	rtdm_fd_put(fd);

  out:
	if (ret < 0)
		trace_cobalt_fd_read_status(current, fd, ufd, ret);

	return ret;
}
EXPORT_SYMBOL_GPL(rtdm_fd_read);

ssize_t rtdm_fd_write(int ufd, const void __user *buf, size_t size)
{
	struct rtdm_fd *fd;
	ssize_t ret;

	fd = get_fd_fixup_mode(ufd);
	if (IS_ERR(fd)) {
		ret = PTR_ERR(fd);
		goto out;
	}

	set_compat_bit(fd);

	trace_cobalt_fd_write(current, fd, ufd, size);

	if (ipipe_root_p)
		ret = fd->ops->write_nrt(fd, buf, size);
	else
		ret = fd->ops->write_rt(fd, buf, size);

	if (!XENO_ASSERT(COBALT, !spltest()))
		splnone();

	rtdm_fd_put(fd);

  out:
	if (ret < 0)
		trace_cobalt_fd_write_status(current, fd, ufd, ret);

	return ret;
}
EXPORT_SYMBOL_GPL(rtdm_fd_write);

ssize_t rtdm_fd_recvmsg(int ufd, struct user_msghdr *msg, int flags)
{
	struct rtdm_fd *fd;
	ssize_t ret;

	fd = get_fd_fixup_mode(ufd);
	if (IS_ERR(fd)) {
		ret = PTR_ERR(fd);
		goto out;
	}

	set_compat_bit(fd);

	trace_cobalt_fd_recvmsg(current, fd, ufd, flags);

	if (ipipe_root_p)
		ret = fd->ops->recvmsg_nrt(fd, msg, flags);
	else
		ret = fd->ops->recvmsg_rt(fd, msg, flags);

	if (!XENO_ASSERT(COBALT, !spltest()))
		splnone();

	rtdm_fd_put(fd);
out:
	if (ret < 0)
		trace_cobalt_fd_recvmsg_status(current, fd, ufd, ret);

	return ret;
}
EXPORT_SYMBOL_GPL(rtdm_fd_recvmsg);

ssize_t rtdm_fd_sendmsg(int ufd, const struct user_msghdr *msg, int flags)
{
	struct rtdm_fd *fd;
	ssize_t ret;

	fd = get_fd_fixup_mode(ufd);
	if (IS_ERR(fd)) {
		ret = PTR_ERR(fd);
		goto out;
	}

	set_compat_bit(fd);

	trace_cobalt_fd_sendmsg(current, fd, ufd, flags);

	if (ipipe_root_p)
		ret = fd->ops->sendmsg_nrt(fd, msg, flags);
	else
		ret = fd->ops->sendmsg_rt(fd, msg, flags);

	if (!XENO_ASSERT(COBALT, !spltest()))
		splnone();

	rtdm_fd_put(fd);
out:
	if (ret < 0)
		trace_cobalt_fd_sendmsg_status(current, fd, ufd, ret);

	return ret;
}
EXPORT_SYMBOL_GPL(rtdm_fd_sendmsg);

static void
__fd_close(struct cobalt_ppd *p, struct rtdm_fd_index *idx, spl_t s)
{
	xnid_remove(&p->fds, &idx->id);
	__put_fd(idx->fd, s);

	kfree(idx);
}

int rtdm_fd_close(int ufd, unsigned int magic)
{
	struct rtdm_fd_index *idx;
	struct cobalt_ppd *ppd;
	struct rtdm_fd *fd;
	spl_t s;

	secondary_mode_only();

	ppd = cobalt_ppd_get(0);

	xnlock_get_irqsave(&fdtree_lock, s);
	idx = fetch_fd_index(ppd, ufd);
	if (idx == NULL)
		goto ebadf;

	fd = idx->fd;
	if (magic != 0 && fd->magic != magic) {
ebadf:
		xnlock_put_irqrestore(&fdtree_lock, s);
		return -EBADF;
	}

	set_compat_bit(fd);

	trace_cobalt_fd_close(current, fd, ufd, fd->refs);

	/*
	 * In dual kernel mode, the linux-side fdtable and the RTDM
	 * ->close() handler are asynchronously managed, i.e.  the
	 * handler execution may be deferred after the regular file
	 * descriptor was removed from the fdtable if some refs on
	 * rtdm_fd are still pending.
	 */
	__fd_close(ppd, idx, s);
	__close_fd(current->files, ufd);

	return 0;
}
EXPORT_SYMBOL_GPL(rtdm_fd_close);

int rtdm_fd_mmap(int ufd, struct _rtdm_mmap_request *rma,
		 void **u_addrp)
{
	struct rtdm_fd *fd;
	int ret;

	secondary_mode_only();

	fd = rtdm_fd_get(ufd, 0);
	if (IS_ERR(fd)) {
		ret = PTR_ERR(fd);
		goto out;
	}

	set_compat_bit(fd);

	trace_cobalt_fd_mmap(current, fd, ufd, rma);

	if (rma->flags & (MAP_FIXED|MAP_ANONYMOUS)) {
		ret = -ENODEV;
		goto unlock;
	}

	ret = __rtdm_mmap_from_fdop(fd, rma->length, rma->offset,
				    rma->prot, rma->flags, u_addrp);
unlock:
	rtdm_fd_put(fd);
out:
	if (ret)
		trace_cobalt_fd_mmap_status(current, fd, ufd, ret);

	return ret;
}

int rtdm_fd_valid_p(int ufd)
{
	struct rtdm_fd *fd;
	spl_t s;

	xnlock_get_irqsave(&fdtree_lock, s);
	fd = fetch_fd(cobalt_ppd_get(0), ufd);
	xnlock_put_irqrestore(&fdtree_lock, s);

	return fd != NULL;
}

/**
 * @brief Bind a selector to specified event types of a given file descriptor
 * @internal
 *
 * This function is invoked by higher RTOS layers implementing select-like
 * services. It shall not be called directly by RTDM drivers.
 *
 * @param[in] ufd User-side file descriptor to bind to
 * @param[in,out] selector Selector object that shall be bound to the given
 * event
 * @param[in] type Event type the caller is interested in
 *
 * @return 0 on success, otherwise:
 *
 * - -EBADF is returned if the file descriptor @a ufd cannot be resolved.
 * - -EINVAL is returned if @a type is invalid.
 *
 * @coretags{task-unrestricted}
 */
int rtdm_fd_select(int ufd, struct xnselector *selector,
		   unsigned int type)
{
	struct rtdm_fd *fd;
	int ret;

	fd = rtdm_fd_get(ufd, 0);
	if (IS_ERR(fd))
		return PTR_ERR(fd);

	set_compat_bit(fd);

	ret = fd->ops->select(fd, selector, type, ufd);

	if (!XENO_ASSERT(COBALT, !spltest()))
		splnone();

	rtdm_fd_put(fd);

	return ret;
}

static void destroy_fd(void *cookie, struct xnid *id)
{
	struct cobalt_ppd *p = cookie;
	struct rtdm_fd_index *idx;
	spl_t s;

	idx = container_of(id, struct rtdm_fd_index, id);
	xnlock_get_irqsave(&fdtree_lock, s);
	__fd_close(p, idx, 0);
}

void rtdm_fd_cleanup(struct cobalt_ppd *p)
{
	/*
	 * This is called on behalf of a (userland) task exit handler,
	 * so we don't have to deal with the regular file descriptors,
	 * we only have to empty our own index.
	 */
	xntree_cleanup(&p->fds, p, destroy_fd);
}

void rtdm_fd_init(void)
{
	sema_init(&rtdm_fd_cleanup_sem, 0);
	kthread_run(fd_cleanup_thread, NULL, "rtdm_fd");
}

static inline void warn_user(struct file *file, const char *call)
{
	struct dentry *dentry = file->f_path.dentry;
	
	printk(XENO_WARNING
	       "%s[%d] called regular %s() on /dev/rtdm/%s\n",
	       current->comm, task_pid_nr(current), call + 5, dentry->d_name.name);
}

static ssize_t dumb_read(struct file *file, char  __user *buf,
			 size_t count, loff_t __user *ppos)
{
	warn_user(file, __func__);
	return -EINVAL;
}

static ssize_t dumb_write(struct file *file,  const char __user *buf,
			  size_t count, loff_t __user *ppos)
{
	warn_user(file, __func__);
	return -EINVAL;
}

static unsigned int dumb_poll(struct file *file, poll_table *pt)
{
	warn_user(file, __func__);
	return -EINVAL;
}

static long dumb_ioctl(struct file *file, unsigned int cmd,
		       unsigned long arg)
{
	warn_user(file, __func__);
	return -EINVAL;
}

const struct file_operations rtdm_dumb_fops = {
	.read		= dumb_read,
	.write		= dumb_write,
	.poll		= dumb_poll,
	.unlocked_ioctl	= dumb_ioctl,
};
