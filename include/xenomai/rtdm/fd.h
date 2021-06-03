/*
 * Copyright (C) 2005-2007 Jan Kiszka <jan.kiszka@web.de>
 * Copyright (C) 2005 Joerg Langenberg <joerg.langenberg@gmx.net>
 * Copyright (C) 2008,2013,2014 Gilles Chanteperdrix <gch@xenomai.org>.
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
#ifndef _COBALT_KERNEL_FD_H
#define _COBALT_KERNEL_FD_H

#include <linux/types.h>
#include <linux/socket.h>
#include <linux/file.h>
#include <cobalt/kernel/tree.h>
#include <asm-generic/xenomai/syscall.h>

struct vm_area_struct;
struct rtdm_fd;
struct _rtdm_mmap_request;
struct xnselector;
struct cobalt_ppd;
struct rtdm_device;

/**
 * @file
 * @anchor File operation handlers
 * @addtogroup rtdm_device_register
 * @{
 */

/**
 * Open handler for named devices
 *
 * @param[in] fd File descriptor associated with opened device instance
 * @param[in] oflags Open flags as passed by the user
 *
 * The file descriptor carries a device minor information which can be
 * retrieved by a call to rtdm_fd_minor(fd). The minor number can be
 * used for distinguishing devices managed by a driver.
 *
 * @return 0 on success. On failure, a negative error code is returned.
 *
 * @see @c open() in IEEE Std 1003.1,
 * http://www.opengroup.org/onlinepubs/009695399
 */
int rtdm_open_handler(struct rtdm_fd *fd, int oflags);

/**
 * Socket creation handler for protocol devices
 *
 * @param[in] fd File descriptor associated with opened device instance
 * @param[in] protocol Protocol number as passed by the user
 *
 * @return 0 on success. On failure, a negative error code is returned.
 *
 * @see @c socket() in IEEE Std 1003.1,
 * http://www.opengroup.org/onlinepubs/009695399
 */
int rtdm_socket_handler(struct rtdm_fd *fd, int protocol);

/**
 * Close handler
 *
 * @param[in] fd File descriptor associated with opened
 * device instance.
 *
 * @see @c close() in IEEE Std 1003.1,
 * http://www.opengroup.org/onlinepubs/009695399
 */
void rtdm_close_handler(struct rtdm_fd *fd);

/**
 * IOCTL handler
 *
 * @param[in] fd File descriptor
 * @param[in] request Request number as passed by the user
 * @param[in,out] arg Request argument as passed by the user
 *
 * @return A positive value or 0 on success. On failure return either
 * -ENOSYS, to request that the function be called again from the opposite
 * realtime/non-realtime context, or another negative error code.
 *
 * @see @c ioctl() in IEEE Std 1003.1,
 * http://www.opengroup.org/onlinepubs/009695399
 */
int rtdm_ioctl_handler(struct rtdm_fd *fd, unsigned int request, void __user *arg);

/**
 * Read handler
 *
 * @param[in] fd File descriptor
 * @param[out] buf Input buffer as passed by the user
 * @param[in] size Number of bytes the user requests to read
 *
 * @return On success, the number of bytes read. On failure return either
 * -ENOSYS, to request that this handler be called again from the opposite
 * realtime/non-realtime context, or another negative error code.
 *
 * @see @c read() in IEEE Std 1003.1,
 * http://www.opengroup.org/onlinepubs/009695399
 */
ssize_t rtdm_read_handler(struct rtdm_fd *fd, void __user *buf, size_t size);

/**
 * Write handler
 *
 * @param[in] fd File descriptor
 * @param[in] buf Output buffer as passed by the user
 * @param[in] size Number of bytes the user requests to write
 *
 * @return On success, the number of bytes written. On failure return
 * either -ENOSYS, to request that this handler be called again from the
 * opposite realtime/non-realtime context, or another negative error code.
 *
 * @see @c write() in IEEE Std 1003.1,
 * http://www.opengroup.org/onlinepubs/009695399
 */
ssize_t rtdm_write_handler(struct rtdm_fd *fd, const void __user *buf, size_t size);

/**
 * Receive message handler
 *
 * @param[in] fd File descriptor
 * @param[in,out] msg Message descriptor as passed by the user, automatically
 * mirrored to safe kernel memory in case of user mode call
 * @param[in] flags Message flags as passed by the user
 *
 * @return On success, the number of bytes received. On failure return
 * either -ENOSYS, to request that this handler be called again from the
 * opposite realtime/non-realtime context, or another negative error code.
 *
 * @see @c recvmsg() in IEEE Std 1003.1,
 * http://www.opengroup.org/onlinepubs/009695399
 */
ssize_t rtdm_recvmsg_handler(struct rtdm_fd *fd, struct user_msghdr *msg, int flags);

/**
 * Transmit message handler
 *
 * @param[in] fd File descriptor
 * @param[in] msg Message descriptor as passed by the user, automatically
 * mirrored to safe kernel memory in case of user mode call
 * @param[in] flags Message flags as passed by the user
 *
 * @return On success, the number of bytes transmitted. On failure return
 * either -ENOSYS, to request that this handler be called again from the
 * opposite realtime/non-realtime context, or another negative error code.
 *
 * @see @c sendmsg() in IEEE Std 1003.1,
 * http://www.opengroup.org/onlinepubs/009695399
 */
ssize_t rtdm_sendmsg_handler(struct rtdm_fd *fd, const struct user_msghdr *msg, int flags);

/**
 * Select handler
 *
 * @param[in] fd File descriptor
 * @param selector Pointer to the selector structure
 * @param type Type of events (@a XNSELECT_READ, @a XNSELECT_WRITE, or @a
 * XNSELECT_EXCEPT)
 * @param index Index of the file descriptor
 *
 * @return 0 on success. On failure, a negative error code is
 * returned.
 *
 * @see @c select() in POSIX.1-2001,
 * http://pubs.opengroup.org/onlinepubs/007908799/xsh/select.html
 */
int rtdm_select_handler(struct rtdm_fd *fd, struct xnselector *selector,
			unsigned int type, unsigned int index);

/**
 * Memory mapping handler
 *
 * @param[in] fd File descriptor
 * @param[in] vma Virtual memory area descriptor
 *
 * @return 0 on success. On failure, a negative error code is
 * returned.
 *
 * @see @c mmap() in POSIX.1-2001,
 * http://pubs.opengroup.org/onlinepubs/7908799/xsh/mmap.html
 *
 * @note The address hint passed to the mmap() request is deliberately
 * ignored by RTDM.
 */
int rtdm_mmap_handler(struct rtdm_fd *fd, struct vm_area_struct *vma);

/**
 * Allocate mapping region in address space
 *
 * When present, this optional handler should return the start address
 * of a free region in the process's address space, large enough to
 * cover the ongoing mmap() operation. If unspecified, the default
 * architecture-defined handler is invoked.
 *
 * Most drivers can omit this handler, except on MMU-less platforms
 * (see second note).
 *
 * @param[in] fd File descriptor
 * @param[in] len Length of the requested region
 * @param[in] pgoff Page frame number to map to (see second note).
 * @param[in] flags Requested mapping flags
 *
 * @return The start address of the mapping region on success. On
 * failure, a negative error code should be returned, with -ENOSYS
 * meaning that the driver does not want to provide such information,
 * in which case the ongoing mmap() operation will fail.
 *
 * @note The address hint passed to the mmap() request is deliberately
 * ignored by RTDM, and therefore not passed to this handler.
 *
 * @note On MMU-less platforms, this handler is required because RTDM
 * issues mapping requests over a shareable character device
 * internally. In such context, the RTDM core may pass a null @a pgoff
 * argument to the handler, for probing for the logical start address
 * of the memory region to map to. Otherwise, when @a pgoff is
 * non-zero, pgoff << PAGE_SHIFT is usually returned.
 */
unsigned long
rtdm_get_unmapped_area_handler(struct rtdm_fd *fd,
			       unsigned long len, unsigned long pgoff,
			       unsigned long flags);
/**
 * @anchor rtdm_fd_ops
 * @brief RTDM file operation descriptor.
 *
 * This structure describes the operations available with a RTDM
 * device, defining handlers for submitting I/O requests. Those
 * handlers are implemented by RTDM device drivers.
 */
struct rtdm_fd_ops {
	/** See rtdm_open_handler(). */
	int (*open)(struct rtdm_fd *fd, int oflags);
	/** See rtdm_socket_handler(). */
	int (*socket)(struct rtdm_fd *fd, int protocol);
	/** See rtdm_close_handler(). */
	void (*close)(struct rtdm_fd *fd);
	/** See rtdm_ioctl_handler(). */
	int (*ioctl_rt)(struct rtdm_fd *fd,
			unsigned int request, void __user *arg);
	/** See rtdm_ioctl_handler(). */
	int (*ioctl_nrt)(struct rtdm_fd *fd,
			 unsigned int request, void __user *arg);
	/** See rtdm_read_handler(). */
	ssize_t (*read_rt)(struct rtdm_fd *fd,
			   void __user *buf, size_t size);
	/** See rtdm_read_handler(). */
	ssize_t (*read_nrt)(struct rtdm_fd *fd,
			    void __user *buf, size_t size);
	/** See rtdm_write_handler(). */
	ssize_t (*write_rt)(struct rtdm_fd *fd,
			    const void __user *buf, size_t size);
	/** See rtdm_write_handler(). */
	ssize_t (*write_nrt)(struct rtdm_fd *fd,
			     const void __user *buf, size_t size);
	/** See rtdm_recvmsg_handler(). */
	ssize_t (*recvmsg_rt)(struct rtdm_fd *fd,
			      struct user_msghdr *msg, int flags);
	/** See rtdm_recvmsg_handler(). */
	ssize_t (*recvmsg_nrt)(struct rtdm_fd *fd,
			       struct user_msghdr *msg, int flags);
	/** See rtdm_sendmsg_handler(). */
	ssize_t (*sendmsg_rt)(struct rtdm_fd *fd,
			      const struct user_msghdr *msg, int flags);
	/** See rtdm_sendmsg_handler(). */
	ssize_t (*sendmsg_nrt)(struct rtdm_fd *fd,
			       const struct user_msghdr *msg, int flags);
	/** See rtdm_select_handler(). */
	int (*select)(struct rtdm_fd *fd,
		      struct xnselector *selector,
		      unsigned int type, unsigned int index);
	/** See rtdm_mmap_handler(). */
	int (*mmap)(struct rtdm_fd *fd,
		    struct vm_area_struct *vma);
	/** See rtdm_get_unmapped_area_handler(). */
	unsigned long (*get_unmapped_area)(struct rtdm_fd *fd,
					   unsigned long len,
					   unsigned long pgoff,
					   unsigned long flags);
};

/** @} File operation handlers */

struct rtdm_fd {
	unsigned int magic;
	struct rtdm_fd_ops *ops;
	struct cobalt_ppd *owner;
	unsigned int refs;
	int ufd;
	int minor;
	int oflags;
#ifdef CONFIG_XENO_ARCH_SYS3264
	int compat;
#endif
	bool stale;
	struct list_head cleanup;
	struct list_head next;	/* in dev->openfd_list */
};

#define RTDM_FD_MAGIC 0x52544446

#define RTDM_FD_COMPAT	__COBALT_COMPAT_BIT
#define RTDM_FD_COMPATX	__COBALT_COMPATX_BIT

int __rtdm_anon_getfd(const char *name, int flags);

void __rtdm_anon_putfd(int ufd);

static inline struct cobalt_ppd *rtdm_fd_owner(const struct rtdm_fd *fd)
{
	return fd->owner;
}

static inline int rtdm_fd_ufd(const struct rtdm_fd *fd)
{
	return fd->ufd;
}

static inline int rtdm_fd_minor(const struct rtdm_fd *fd)
{
	return fd->minor;
}

static inline int rtdm_fd_flags(const struct rtdm_fd *fd)
{
	return fd->oflags;
}

#ifdef CONFIG_XENO_ARCH_SYS3264
static inline int rtdm_fd_is_compat(const struct rtdm_fd *fd)
{
	return fd->compat;
}
#else
static inline int rtdm_fd_is_compat(const struct rtdm_fd *fd)
{
	return 0;
}
#endif

int rtdm_fd_enter(struct rtdm_fd *rtdm_fd, int ufd,
		  unsigned int magic, struct rtdm_fd_ops *ops);

int rtdm_fd_register(struct rtdm_fd *fd, int ufd);

struct rtdm_fd *rtdm_fd_get(int ufd, unsigned int magic);

int rtdm_fd_lock(struct rtdm_fd *fd);

void rtdm_fd_put(struct rtdm_fd *fd);

void rtdm_fd_unlock(struct rtdm_fd *fd);

int rtdm_fd_fcntl(int ufd, int cmd, ...);

int rtdm_fd_ioctl(int ufd, unsigned int request, ...);

ssize_t rtdm_fd_read(int ufd, void __user *buf, size_t size);

ssize_t rtdm_fd_write(int ufd, const void __user *buf, size_t size);

int rtdm_fd_close(int ufd, unsigned int magic);

ssize_t rtdm_fd_recvmsg(int ufd, struct user_msghdr *msg, int flags);

int __rtdm_fd_recvmmsg(int ufd, void __user *u_msgvec, unsigned int vlen,
		       unsigned int flags, void __user *u_timeout,
		       int (*get_mmsg)(struct mmsghdr *mmsg, void __user *u_mmsg),
		       int (*put_mmsg)(void __user **u_mmsg_p, const struct mmsghdr *mmsg),
		       int (*get_timespec)(struct timespec *ts, const void __user *u_ts));

ssize_t rtdm_fd_sendmsg(int ufd, const struct user_msghdr *msg,
			int flags);

int __rtdm_fd_sendmmsg(int ufd, void __user *u_msgvec, unsigned int vlen,
		       unsigned int flags,
		       int (*get_mmsg)(struct mmsghdr *mmsg, void __user *u_mmsg),
		       int (*put_mmsg)(void __user **u_mmsg_p, const struct mmsghdr *mmsg));

int rtdm_fd_mmap(int ufd, struct _rtdm_mmap_request *rma,
		 void **u_addrp);

int rtdm_fd_valid_p(int ufd);

int rtdm_fd_select(int ufd, struct xnselector *selector,
		   unsigned int type);

int rtdm_device_new_fd(struct rtdm_fd *fd, int ufd,
		struct rtdm_device *dev);

void rtdm_device_flush_fds(struct rtdm_device *dev);

void rtdm_fd_cleanup(struct cobalt_ppd *p);

void rtdm_fd_init(void);

#endif /* _COBALT_KERNEL_FD_H */
