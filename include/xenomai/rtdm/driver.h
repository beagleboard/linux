/**
 * @file
 * Real-Time Driver Model for Xenomai, driver API header
 *
 * Copyright (C) 2005-2007 Jan Kiszka <jan.kiszka@web.de>
 * Copyright (C) 2005 Joerg Langenberg <joerg.langenberg@gmx.net>
 * Copyright (C) 2008 Gilles Chanteperdrix <gilles.chanteperdrix@xenomai.org>
 * Copyright (C) 2014 Philippe Gerum <rpm@xenomai.org>
 *
 * Xenomai is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Xenomai is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Xenomai; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * @ingroup driverapi
 */
#ifndef _COBALT_RTDM_DRIVER_H
#define _COBALT_RTDM_DRIVER_H

#include <asm/atomic.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/wait.h>
#include <linux/notifier.h>
#include <xenomai/version.h>
#include <cobalt/kernel/heap.h>
#include <cobalt/kernel/sched.h>
#include <cobalt/kernel/intr.h>
#include <cobalt/kernel/synch.h>
#include <cobalt/kernel/select.h>
#include <cobalt/kernel/clock.h>
#include <cobalt/kernel/apc.h>
#include <cobalt/kernel/init.h>
#include <cobalt/kernel/ancillaries.h>
#include <cobalt/kernel/tree.h>
#include <rtdm/fd.h>
#include <rtdm/rtdm.h>

/* debug support */
#include <cobalt/kernel/assert.h>
#include <trace/events/cobalt-rtdm.h>
#ifdef CONFIG_PCI
#include <asm-generic/xenomai/pci_ids.h>
#endif /* CONFIG_PCI */
#include <asm/xenomai/syscall.h>

struct class;
typedef struct xnselector rtdm_selector_t;
enum rtdm_selecttype;

/*!
 * @addtogroup rtdm_device_register
 * @{
 */

/*!
 * @anchor dev_flags @name Device Flags
 * Static flags describing a RTDM device
 * @{
 */
/** If set, only a single instance of the device can be requested by an
 *  application. */
#define RTDM_EXCLUSIVE			0x0001

/**
 * Use fixed minor provided in the rtdm_device description for
 * registering. If this flag is absent, the RTDM core assigns minor
 * numbers to devices managed by a driver in order of registration.
 */
#define RTDM_FIXED_MINOR		0x0002

/** If set, the device is addressed via a clear-text name. */
#define RTDM_NAMED_DEVICE		0x0010

/** If set, the device is addressed via a combination of protocol ID and
 *  socket type. */
#define RTDM_PROTOCOL_DEVICE		0x0020

/** Mask selecting the device type. */
#define RTDM_DEVICE_TYPE_MASK		0x00F0

/** Flag indicating a secure variant of RTDM (not supported here) */
#define RTDM_SECURE_DEVICE		0x80000000
/** @} Device Flags */

/** Maximum number of named devices per driver. */
#define RTDM_MAX_MINOR	4096

/** @} rtdm_device_register */

/*!
 * @addtogroup rtdm_sync
 * @{
 */

/*!
 * @anchor RTDM_SELECTTYPE_xxx   @name RTDM_SELECTTYPE_xxx
 * Event types select can bind to
 * @{
 */
enum rtdm_selecttype {
	/** Select input data availability events */
	RTDM_SELECTTYPE_READ = XNSELECT_READ,

	/** Select ouput buffer availability events */
	RTDM_SELECTTYPE_WRITE = XNSELECT_WRITE,

	/** Select exceptional events */
	RTDM_SELECTTYPE_EXCEPT = XNSELECT_EXCEPT
};
/** @} RTDM_SELECTTYPE_xxx */

/** @} rtdm_sync */

/**
 * @brief Device context
 *
 * A device context structure is associated with every open device instance.
 * RTDM takes care of its creation and destruction and passes it to the
 * operation handlers when being invoked.
 *
 * Drivers can attach arbitrary data immediately after the official
 * structure.  The size of this data is provided via
 * rtdm_driver.context_size during device registration.
 */
struct rtdm_dev_context {
	struct rtdm_fd fd;

	/** Set of active device operation handlers */
	/** Reference to owning device */
	struct rtdm_device *device;

	/** Begin of driver defined context data structure */
	char dev_private[0];
};

static inline struct rtdm_dev_context *rtdm_fd_to_context(struct rtdm_fd *fd)
{
	return container_of(fd, struct rtdm_dev_context, fd);
}

/**
 * Locate the driver private area associated to a device context structure
 *
 * @param[in] fd File descriptor structure associated with opened
 * device instance
 *
 * @return The address of the private driver area associated to @a
 * file descriptor.
 */
static inline void *rtdm_fd_to_private(struct rtdm_fd *fd)
{
	return &rtdm_fd_to_context(fd)->dev_private[0];
}

/**
 * Locate a device file descriptor structure from its driver private area
 *
 * @param[in] dev_private Address of a private context area
 *
 * @return The address of the file descriptor structure defining @a
 * dev_private.
 */
static inline struct rtdm_fd *rtdm_private_to_fd(void *dev_private)
{
	struct rtdm_dev_context *ctx;
	ctx = container_of(dev_private, struct rtdm_dev_context, dev_private);
	return &ctx->fd;
}

/**
 * Tell whether the passed file descriptor belongs to an application.
 *
 * @param[in] fd File descriptor
 *
 * @return true if passed file descriptor belongs to an application,
 * false otherwise.
 */
static inline bool rtdm_fd_is_user(struct rtdm_fd *fd)
{
	return rtdm_fd_owner(fd) != &cobalt_kernel_ppd;
}

/**
 * Locate a device structure from a file descriptor.
 *
 * @param[in] fd File descriptor
 *
 * @return The address of the device structure to which this file
 * descriptor is attached.
 */
static inline struct rtdm_device *rtdm_fd_device(struct rtdm_fd *fd)
{
	return rtdm_fd_to_context(fd)->device;
}

/**
 * @brief RTDM profile information
 *
 * This descriptor details the profile information associated to a
 * RTDM class of device managed by a driver.
 *
 * @anchor rtdm_profile_info
 */
struct rtdm_profile_info {
	/** Device class name */
	const char *name;
	/** Device class ID, see @ref RTDM_CLASS_xxx */
	int class_id;
	/** Device sub-class, see RTDM_SUBCLASS_xxx definition in the
	    @ref rtdm_profiles "Device Profiles" */
	int subclass_id;
	/** Supported device profile version */
	int version;
	/** Reserved */
	unsigned int magic;
	struct module *owner;
	struct class *kdev_class;
};

struct rtdm_driver;

/**
 * @brief RTDM state management handlers
 */
struct rtdm_sm_ops {
	/** Handler called upon transition to COBALT_STATE_WARMUP */ 
	int (*start)(struct rtdm_driver *drv);
	/** Handler called upon transition to COBALT_STATE_TEARDOWN */ 
	int (*stop)(struct rtdm_driver *drv);
};

/**
 * @brief RTDM driver
 *
 * This descriptor describes a RTDM device driver. The structure holds
 * runtime data, therefore it must reside in writable memory.
 */
struct rtdm_driver {
	/**
	 * Class profile information. The RTDM_PROFILE_INFO() macro @b
	 * must be used for filling up this field.
	 * @anchor rtdm_driver_profile
	 */
	struct rtdm_profile_info profile_info;
	/**
	 * Device flags, see @ref dev_flags "Device Flags" for details
	 * @anchor rtdm_driver_flags
	 */
	int device_flags;
	/**
	 * Size of the private memory area the core should
	 * automatically allocate for each open file descriptor, which
	 * is usable for storing the context data associated to each
	 * connection. The allocated memory is zero-initialized. The
	 * start of this area can be retrieved by a call to
	 * rtdm_fd_to_private().
	 */
	size_t context_size;
	/** Protocol device identification: protocol family (PF_xxx) */
	int protocol_family;
	/** Protocol device identification: socket type (SOCK_xxx) */
	int socket_type;
	/** I/O operation handlers */
	struct rtdm_fd_ops ops;
	/** State management handlers */
	struct rtdm_sm_ops smops;
	/**
	 * Count of devices this driver manages. This value is used to
	 * allocate a chrdev region for named devices.
	 */
	int device_count;
	/** Base minor for named devices. */
	int base_minor;
	/** Reserved area */
	struct {
		union {
			struct {
				struct cdev cdev;
				int major;
			} named;
		};
		atomic_t refcount;
		struct notifier_block nb_statechange;
		DECLARE_BITMAP(minor_map, RTDM_MAX_MINOR);
	};
};

#define RTDM_CLASS_MAGIC	0x8284636c

/**
 * @brief Initializer for class profile information.
 *
 * This macro must be used to fill in the @ref rtdm_profile_info
 * "class profile information" field from a RTDM driver.
 *
 * @param __name Class name (unquoted).
 *
 * @param __id Class major identification number
 * (profile_version.class_id).
 *
 * @param __subid Class minor identification number
 * (profile_version.subclass_id).
 *
 * @param __version Profile version number.
 *
 * @note See @ref rtdm_profiles "Device Profiles".
 */
#define RTDM_PROFILE_INFO(__name, __id, __subid, __version)	\
{								\
	.name = ( # __name ),					\
	.class_id = (__id),					\
	.subclass_id = (__subid),				\
	.version = (__version),					\
	.magic = ~RTDM_CLASS_MAGIC,				\
	.owner = THIS_MODULE,					\
	.kdev_class = NULL,					\
}

int rtdm_drv_set_sysclass(struct rtdm_driver *drv, struct class *cls);

/**
 * @brief RTDM device
 *
 * This descriptor describes a RTDM device instance. The structure
 * holds runtime data, therefore it must reside in writable memory.
 */
struct rtdm_device {
	/** Device driver. */
	struct rtdm_driver *driver;
	/** Driver definable device data */
	void *device_data;
	/**
	 * Device label template for composing the device name. A
	 * limited printf-like format string is assumed, with a
	 * provision for replacing the first %d/%i placeholder found
	 * in the string by the device minor number.  It is up to the
	 * driver to actually mention this placeholder or not,
	 * depending on the naming convention for its devices.  For
	 * named devices, the corresponding device node will
	 * automatically appear in the /dev/rtdm hierachy with
	 * hotplug-enabled device filesystems (DEVTMPFS).
	 */
	const char *label;
	/**
	 * Minor number of the device. If RTDM_FIXED_MINOR is present
	 * in the driver flags, the value stored in this field is used
	 * verbatim by rtdm_dev_register(). Otherwise, the RTDM core
	 * automatically assigns minor numbers to all devices managed
	 * by the driver referred to by @a driver, in order of
	 * registration, storing the resulting values into this field.
	 *
	 * Device nodes created for named devices in the Linux /dev
	 * hierarchy are assigned this minor number.
	 *
	 * The minor number of the current device handling an I/O
	 * request can be retreived by a call to rtdm_fd_minor().
	 */
	int minor;
	/** Reserved area. */
	struct {
		unsigned int magic;
		char *name;
		union {
			struct {
				xnhandle_t handle;
			} named;
			struct {
				struct xnid id;
			} proto;
		};
		dev_t rdev;
		struct device *kdev;
		struct class *kdev_class;
		atomic_t refcount;
		struct rtdm_fd_ops ops;
		wait_queue_head_t putwq;
		struct list_head openfd_list;
	};
};

/* --- device registration --- */

int rtdm_dev_register(struct rtdm_device *device);

void rtdm_dev_unregister(struct rtdm_device *device);

#ifndef DOXYGEN_CPP /* Avoid static inline tags for RTDM in doxygen */

static inline struct device *rtdm_dev_to_kdev(struct rtdm_device *device)
{
	return device->kdev;
}

/* --- clock services --- */
static inline nanosecs_abs_t rtdm_clock_read(void)
{
	return xnclock_read_realtime(&nkclock);
}

static inline nanosecs_abs_t rtdm_clock_read_monotonic(void)
{
	return xnclock_read_monotonic(&nkclock);
}
#endif /* !DOXYGEN_CPP */

/* --- timeout sequences */

typedef nanosecs_abs_t rtdm_toseq_t;

void rtdm_toseq_init(rtdm_toseq_t *timeout_seq, nanosecs_rel_t timeout);

/*!
 * @addtogroup rtdm_sync
 * @{
 */

/*!
 * @defgroup rtdm_sync_biglock Big dual kernel lock
 * @{
 */

/**
 * @brief Enter atomic section (dual kernel only)
 *
 * This call opens a fully atomic section, serializing execution with
 * respect to all interrupt handlers (including for real-time IRQs)
 * and Xenomai threads running on all CPUs.
 *
 * @param __context name of local variable to store the context
 * in. This variable updated by the real-time core will hold the
 * information required to leave the atomic section properly.
 *
 * @note Atomic sections may be nested. The caller is allowed to sleep
 * on a blocking Xenomai service from primary mode within an atomic
 * section delimited by cobalt_atomic_enter/cobalt_atomic_leave calls.
 * On the contrary, sleeping on a regular Linux kernel service while
 * holding such lock is NOT valid.
 *
 * @note Since the strongest lock is acquired by this service, it can
 * be used to synchronize real-time and non-real-time contexts.
 *
 * @warning This service is not portable to the Mercury core, and
 * should be restricted to Cobalt-specific use cases, mainly for the
 * purpose of porting existing dual-kernel drivers which still depend
 * on the obsolete RTDM_EXECUTE_ATOMICALLY() construct.
 */
#define cobalt_atomic_enter(__context)				\
	do {							\
		xnlock_get_irqsave(&nklock, (__context));	\
		xnsched_lock();					\
	} while (0)

/**
 * @brief Leave atomic section (dual kernel only)
 *
 * This call closes an atomic section previously opened by a call to
 * cobalt_atomic_enter(), restoring the preemption and interrupt state
 * which prevailed prior to entering the exited section.
 *
 * @param __context name of local variable which stored the context.
 *
 * @warning This service is not portable to the Mercury core, and
 * should be restricted to Cobalt-specific use cases.
 */
#define cobalt_atomic_leave(__context)				\
	do {							\
		xnsched_unlock();				\
		xnlock_put_irqrestore(&nklock, (__context));	\
	} while (0)

/**
 * @brief Execute code block atomically (DEPRECATED)
 *
 * Generally, it is illegal to suspend the current task by calling
 * rtdm_task_sleep(), rtdm_event_wait(), etc. while holding a spinlock. In
 * contrast, this macro allows to combine several operations including
 * a potentially rescheduling call to an atomic code block with respect to
 * other RTDM_EXECUTE_ATOMICALLY() blocks. The macro is a light-weight
 * alternative for protecting code blocks via mutexes, and it can even be used
 * to synchronise real-time and non-real-time contexts.
 *
 * @param code_block Commands to be executed atomically
 *
 * @note It is not allowed to leave the code block explicitly by using
 * @c break, @c return, @c goto, etc. This would leave the global lock held
 * during the code block execution in an inconsistent state. Moreover, do not
 * embed complex operations into the code bock. Consider that they will be
 * executed under preemption lock with interrupts switched-off. Also note that
 * invocation of rescheduling calls may break the atomicity until the task
 * gains the CPU again.
 *
 * @coretags{unrestricted}
 *
 * @deprecated This construct will be phased out in Xenomai
 * 3.0. Please use rtdm_waitqueue services instead.
 *
 * @see cobalt_atomic_enter().
 */
#ifdef DOXYGEN_CPP /* Beautify doxygen output */
#define RTDM_EXECUTE_ATOMICALLY(code_block)	\
{						\
	<ENTER_ATOMIC_SECTION>			\
	code_block;				\
	<LEAVE_ATOMIC_SECTION>			\
}
#else /* This is how it really works */
static inline __attribute__((deprecated)) void
rtdm_execute_atomically(void) { }

#define RTDM_EXECUTE_ATOMICALLY(code_block)		\
{							\
	spl_t __rtdm_s;					\
							\
	rtdm_execute_atomically();			\
	xnlock_get_irqsave(&nklock, __rtdm_s);		\
	xnsched_lock();					\
	code_block;					\
	xnsched_unlock();				\
	xnlock_put_irqrestore(&nklock, __rtdm_s);	\
}
#endif

/** @} Big dual kernel lock */

/**
 * @defgroup rtdm_sync_spinlock Spinlock with preemption deactivation
 * @{
 */

/**
 * Static lock initialisation
 */
#define RTDM_LOCK_UNLOCKED(__name)	IPIPE_SPIN_LOCK_UNLOCKED

#define DEFINE_RTDM_LOCK(__name)		\
	rtdm_lock_t __name = RTDM_LOCK_UNLOCKED(__name)

/** Lock variable */
typedef ipipe_spinlock_t rtdm_lock_t;

/** Variable to save the context while holding a lock */
typedef unsigned long rtdm_lockctx_t;

/**
 * Dynamic lock initialisation
 *
 * @param lock Address of lock variable
 *
 * @coretags{task-unrestricted}
 */
static inline void rtdm_lock_init(rtdm_lock_t *lock)
{
	raw_spin_lock_init(lock);
}

/**
 * Acquire lock from non-preemptible contexts
 *
 * @param lock Address of lock variable
 *
 * @coretags{unrestricted}
 */
static inline void rtdm_lock_get(rtdm_lock_t *lock)
{
	XENO_BUG_ON(COBALT, !spltest());
	raw_spin_lock(lock);
	xnsched_lock();
}

/**
 * Release lock without preemption restoration
 *
 * @param lock Address of lock variable
 *
 * @coretags{unrestricted, might-switch}
 */
static inline void rtdm_lock_put(rtdm_lock_t *lock)
{
	raw_spin_unlock(lock);
	xnsched_unlock();
}

/**
 * Acquire lock and disable preemption, by stalling the head domain.
 *
 * @param __lock Address of lock variable
 * @param __context name of local variable to store the context in
 *
 * @coretags{unrestricted}
 */
#define rtdm_lock_get_irqsave(__lock, __context)	\
	((__context) = __rtdm_lock_get_irqsave(__lock))

static inline rtdm_lockctx_t __rtdm_lock_get_irqsave(rtdm_lock_t *lock)
{
	rtdm_lockctx_t context;

	context = ipipe_test_and_stall_head();
	raw_spin_lock(lock);
	xnsched_lock();

	return context;
}

/**
 * Release lock and restore preemption state
 *
 * @param lock Address of lock variable
 * @param context name of local variable which stored the context
 *
 * @coretags{unrestricted}
 */
static inline
void rtdm_lock_put_irqrestore(rtdm_lock_t *lock, rtdm_lockctx_t context)
{
	raw_spin_unlock(lock);
	xnsched_unlock();
	ipipe_restore_head(context);
}

/**
 * Disable preemption locally
 *
 * @param __context name of local variable to store the context in
 *
 * @coretags{unrestricted}
 */
#define rtdm_lock_irqsave(__context)	\
	splhigh(__context)

/**
 * Restore preemption state
 *
 * @param __context name of local variable which stored the context
 *
 * @coretags{unrestricted}
 */
#define rtdm_lock_irqrestore(__context)	\
	splexit(__context)

/** @} Spinlock with Preemption Deactivation */

#ifndef DOXYGEN_CPP

struct rtdm_waitqueue {
	struct xnsynch wait;
};
typedef struct rtdm_waitqueue rtdm_waitqueue_t;

#define RTDM_WAITQUEUE_INITIALIZER(__name) {		 \
	    .wait = XNSYNCH_WAITQUEUE_INITIALIZER((__name).wait), \
	}

#define DEFINE_RTDM_WAITQUEUE(__name)				\
	struct rtdm_waitqueue __name = RTDM_WAITQUEUE_INITIALIZER(__name)

#define DEFINE_RTDM_WAITQUEUE_ONSTACK(__name)	\
	DEFINE_RTDM_WAITQUEUE(__name)

static inline void rtdm_waitqueue_init(struct rtdm_waitqueue *wq)
{
	*wq = (struct rtdm_waitqueue)RTDM_WAITQUEUE_INITIALIZER(*wq);
}

static inline void rtdm_waitqueue_destroy(struct rtdm_waitqueue *wq)
{
	xnsynch_destroy(&wq->wait);
}

static inline int __rtdm_dowait(struct rtdm_waitqueue *wq,
				nanosecs_rel_t timeout, xntmode_t timeout_mode)
{
	int ret;
	
	ret = xnsynch_sleep_on(&wq->wait, timeout, timeout_mode);
	if (ret & XNBREAK)
		return -EINTR;
	if (ret & XNTIMEO)
		return -ETIMEDOUT;
	if (ret & XNRMID)
		return -EIDRM;
	return 0;
}

static inline int __rtdm_timedwait(struct rtdm_waitqueue *wq,
				   nanosecs_rel_t timeout, rtdm_toseq_t *toseq)
{
	if (toseq && timeout > 0)
		return __rtdm_dowait(wq, *toseq, XN_ABSOLUTE);

	return __rtdm_dowait(wq, timeout, XN_RELATIVE);
}

#define rtdm_timedwait_condition_locked(__wq, __cond, __timeout, __toseq) \
	({								\
		int __ret = 0;						\
		while (__ret == 0 && !(__cond))				\
			__ret = __rtdm_timedwait(__wq, __timeout, __toseq); \
		__ret;							\
	})

#define rtdm_wait_condition_locked(__wq, __cond)			\
	({								\
		int __ret = 0;						\
		while (__ret == 0 && !(__cond))				\
			__ret = __rtdm_dowait(__wq,			\
					      XN_INFINITE, XN_RELATIVE); \
		__ret;							\
	})

#define rtdm_timedwait_condition(__wq, __cond, __timeout, __toseq)	\
	({								\
		spl_t __s;						\
		int __ret;						\
		xnlock_get_irqsave(&nklock, __s);			\
		__ret = rtdm_timedwait_condition_locked(__wq, __cond,	\
					      __timeout, __toseq);	\
		xnlock_put_irqrestore(&nklock, __s);			\
		__ret;							\
	})

#define rtdm_timedwait(__wq, __timeout, __toseq)			\
	__rtdm_timedwait(__wq, __timeout, __toseq)

#define rtdm_timedwait_locked(__wq, __timeout, __toseq)			\
	rtdm_timedwait(__wq, __timeout, __toseq)

#define rtdm_wait_condition(__wq, __cond)				\
	({								\
		spl_t __s;						\
		int __ret;						\
		xnlock_get_irqsave(&nklock, __s);			\
		__ret = rtdm_wait_condition_locked(__wq, __cond);	\
		xnlock_put_irqrestore(&nklock, __s);			\
		__ret;							\
	})

#define rtdm_wait(__wq)							\
	__rtdm_dowait(__wq, XN_INFINITE, XN_RELATIVE)

#define rtdm_wait_locked(__wq)  rtdm_wait(__wq)

#define rtdm_waitqueue_lock(__wq, __context)  cobalt_atomic_enter(__context)

#define rtdm_waitqueue_unlock(__wq, __context)  cobalt_atomic_leave(__context)

#define rtdm_waitqueue_signal(__wq)					\
	({								\
		struct xnthread *__waiter;				\
		__waiter = xnsynch_wakeup_one_sleeper(&(__wq)->wait);	\
		xnsched_run();						\
		__waiter != NULL;					\
	})

#define __rtdm_waitqueue_flush(__wq, __reason)				\
	({								\
		int __ret;						\
		__ret = xnsynch_flush(&(__wq)->wait, __reason);		\
		xnsched_run();						\
		__ret == XNSYNCH_RESCHED;				\
	})

#define rtdm_waitqueue_broadcast(__wq)	\
	__rtdm_waitqueue_flush(__wq, 0)

#define rtdm_waitqueue_flush(__wq)	\
	__rtdm_waitqueue_flush(__wq, XNBREAK)

#define rtdm_waitqueue_wakeup(__wq, __waiter)				\
	do {								\
		xnsynch_wakeup_this_sleeper(&(__wq)->wait, __waiter);	\
		xnsched_run();						\
	} while (0)

#define rtdm_for_each_waiter(__pos, __wq)		\
	xnsynch_for_each_sleeper(__pos, &(__wq)->wait)

#define rtdm_for_each_waiter_safe(__pos, __tmp, __wq)	\
	xnsynch_for_each_sleeper_safe(__pos, __tmp, &(__wq)->wait)

#endif /* !DOXYGEN_CPP */

/** @} rtdm_sync */

/* --- Interrupt management services --- */
/*!
 * @addtogroup rtdm_irq
 * @{
 */

typedef struct xnintr rtdm_irq_t;

/*!
 * @anchor RTDM_IRQTYPE_xxx   @name RTDM_IRQTYPE_xxx
 * Interrupt registrations flags
 * @{
 */
/** Enable IRQ-sharing with other real-time drivers */
#define RTDM_IRQTYPE_SHARED		XN_IRQTYPE_SHARED
/** Mark IRQ as edge-triggered, relevant for correct handling of shared
 *  edge-triggered IRQs */
#define RTDM_IRQTYPE_EDGE		XN_IRQTYPE_EDGE
/** @} RTDM_IRQTYPE_xxx */

/**
 * Interrupt handler
 *
 * @param[in] irq_handle IRQ handle as returned by rtdm_irq_request()
 *
 * @return 0 or a combination of @ref RTDM_IRQ_xxx flags
 */
typedef int (*rtdm_irq_handler_t)(rtdm_irq_t *irq_handle);

/*!
 * @anchor RTDM_IRQ_xxx   @name RTDM_IRQ_xxx
 * Return flags of interrupt handlers
 * @{
 */
/** Unhandled interrupt */
#define RTDM_IRQ_NONE			XN_IRQ_NONE
/** Denote handled interrupt */
#define RTDM_IRQ_HANDLED		XN_IRQ_HANDLED
/** Request interrupt disabling on exit */
#define RTDM_IRQ_DISABLE		XN_IRQ_DISABLE
/** @} RTDM_IRQ_xxx */

/**
 * Retrieve IRQ handler argument
 *
 * @param irq_handle IRQ handle
 * @param type Type of the pointer to return
 *
 * @return The argument pointer registered on rtdm_irq_request() is returned,
 * type-casted to the specified @a type.
 *
 * @coretags{unrestricted}
 */
#define rtdm_irq_get_arg(irq_handle, type)	((type *)irq_handle->cookie)
/** @} rtdm_irq */

int rtdm_irq_request(rtdm_irq_t *irq_handle, unsigned int irq_no,
		     rtdm_irq_handler_t handler, unsigned long flags,
		     const char *device_name, void *arg);

#ifndef DOXYGEN_CPP /* Avoid static inline tags for RTDM in doxygen */
static inline int rtdm_irq_free(rtdm_irq_t *irq_handle)
{
	if (!XENO_ASSERT(COBALT, xnsched_root_p()))
		return -EPERM;
	xnintr_detach(irq_handle);
	return 0;
}

static inline int rtdm_irq_enable(rtdm_irq_t *irq_handle)
{
	xnintr_enable(irq_handle);
	return 0;
}

static inline int rtdm_irq_disable(rtdm_irq_t *irq_handle)
{
	xnintr_disable(irq_handle);
	return 0;
}
#endif /* !DOXYGEN_CPP */

/* --- non-real-time signalling services --- */

/*!
 * @addtogroup rtdm_nrtsignal
 * @{
 */

typedef struct rtdm_nrtsig rtdm_nrtsig_t;
/**
 * Non-real-time signal handler
 *
 * @param[in] nrt_sig Signal handle pointer as passed to rtdm_nrtsig_init()
 * @param[in] arg Argument as passed to rtdm_nrtsig_init()
 *
 * @note The signal handler will run in soft-IRQ context of the non-real-time
 * subsystem. Note the implications of this context, e.g. no invocation of
 * blocking operations.
 */
typedef void (*rtdm_nrtsig_handler_t)(rtdm_nrtsig_t *nrt_sig, void *arg);

struct rtdm_nrtsig {
	rtdm_nrtsig_handler_t handler;
	void *arg;
};

void rtdm_schedule_nrt_work(struct work_struct *lostage_work);
/** @} rtdm_nrtsignal */

#ifndef DOXYGEN_CPP /* Avoid static inline tags for RTDM in doxygen */
static inline void rtdm_nrtsig_init(rtdm_nrtsig_t *nrt_sig,
				rtdm_nrtsig_handler_t handler, void *arg)
{
	nrt_sig->handler = handler;
	nrt_sig->arg = arg;
}

static inline void rtdm_nrtsig_destroy(rtdm_nrtsig_t *nrt_sig)
{
	nrt_sig->handler = NULL;
	nrt_sig->arg = NULL;
}

void rtdm_nrtsig_pend(rtdm_nrtsig_t *nrt_sig);
#endif /* !DOXYGEN_CPP */

/* --- timer services --- */

/*!
 * @addtogroup rtdm_timer
 * @{
 */

typedef struct xntimer rtdm_timer_t;

/**
 * Timer handler
 *
 * @param[in] timer Timer handle as returned by rtdm_timer_init()
 */
typedef void (*rtdm_timer_handler_t)(rtdm_timer_t *timer);

/*!
 * @anchor RTDM_TIMERMODE_xxx   @name RTDM_TIMERMODE_xxx
 * Timer operation modes
 * @{
 */
enum rtdm_timer_mode {
	/** Monotonic timer with relative timeout */
	RTDM_TIMERMODE_RELATIVE = XN_RELATIVE,

	/** Monotonic timer with absolute timeout */
	RTDM_TIMERMODE_ABSOLUTE = XN_ABSOLUTE,

	/** Adjustable timer with absolute timeout */
	RTDM_TIMERMODE_REALTIME = XN_REALTIME
};
/** @} RTDM_TIMERMODE_xxx */

/** @} rtdm_timer */

int rtdm_timer_init(rtdm_timer_t *timer, rtdm_timer_handler_t handler,
		    const char *name);

void rtdm_timer_destroy(rtdm_timer_t *timer);

int rtdm_timer_start(rtdm_timer_t *timer, nanosecs_abs_t expiry,
		     nanosecs_rel_t interval, enum rtdm_timer_mode mode);

void rtdm_timer_stop(rtdm_timer_t *timer);

#ifndef DOXYGEN_CPP /* Avoid static inline tags for RTDM in doxygen */
static inline int rtdm_timer_start_in_handler(rtdm_timer_t *timer,
					      nanosecs_abs_t expiry,
					      nanosecs_rel_t interval,
					      enum rtdm_timer_mode mode)
{
	return xntimer_start(timer, expiry, interval, (xntmode_t)mode);
}

static inline void rtdm_timer_stop_in_handler(rtdm_timer_t *timer)
{
	xntimer_stop(timer);
}
#endif /* !DOXYGEN_CPP */

/* --- task services --- */
/*!
 * @addtogroup rtdm_task
 * @{
 */

typedef struct xnthread rtdm_task_t;

/**
 * Real-time task procedure
 *
 * @param[in,out] arg argument as passed to rtdm_task_init()
 */
typedef void (*rtdm_task_proc_t)(void *arg);

/**
 * @anchor rtdmtaskprio @name Task Priority Range
 * Maximum and minimum task priorities
 * @{ */
#define RTDM_TASK_LOWEST_PRIORITY	0
#define RTDM_TASK_HIGHEST_PRIORITY	99
/** @} Task Priority Range */

/**
 * @anchor rtdmchangetaskprio @name Task Priority Modification
 * Raise or lower task priorities by one level
 * @{ */
#define RTDM_TASK_RAISE_PRIORITY	(+1)
#define RTDM_TASK_LOWER_PRIORITY	(-1)
/** @} Task Priority Modification */

/** @} rtdm_task */

int rtdm_task_init(rtdm_task_t *task, const char *name,
		   rtdm_task_proc_t task_proc, void *arg,
		   int priority, nanosecs_rel_t period);
int __rtdm_task_sleep(xnticks_t timeout, xntmode_t mode);
void rtdm_task_busy_sleep(nanosecs_rel_t delay);

#ifndef DOXYGEN_CPP /* Avoid static inline tags for RTDM in doxygen */
static inline void rtdm_task_destroy(rtdm_task_t *task)
{
	xnthread_cancel(task);
	xnthread_join(task, true);
}

static inline int rtdm_task_should_stop(void)
{
	return xnthread_test_info(xnthread_current(), XNCANCELD);
}

void rtdm_task_join(rtdm_task_t *task);

static inline void __deprecated rtdm_task_join_nrt(rtdm_task_t *task,
						   unsigned int poll_delay)
{
	rtdm_task_join(task);
}

static inline void rtdm_task_set_priority(rtdm_task_t *task, int priority)
{
	union xnsched_policy_param param = { .rt = { .prio = priority } };
	spl_t s;

	splhigh(s);
	xnthread_set_schedparam(task, &xnsched_class_rt, &param);
	xnsched_run();
	splexit(s);
}

static inline int rtdm_task_set_period(rtdm_task_t *task,
				       nanosecs_abs_t start_date,
				       nanosecs_rel_t period)
{
	if (period < 0)
		period = 0;
	if (start_date == 0)
		start_date = XN_INFINITE;

	return xnthread_set_periodic(task, start_date, XN_ABSOLUTE, period);
}

static inline int rtdm_task_unblock(rtdm_task_t *task)
{
	spl_t s;
	int res;

	splhigh(s);
	res = xnthread_unblock(task);
	xnsched_run();
	splexit(s);

	return res;
}

static inline rtdm_task_t *rtdm_task_current(void)
{
	return xnthread_current();
}

static inline int rtdm_task_wait_period(unsigned long *overruns_r)
{
	if (!XENO_ASSERT(COBALT, !xnsched_unblockable_p()))
		return -EPERM;
	return xnthread_wait_period(overruns_r);
}

static inline int rtdm_task_sleep(nanosecs_rel_t delay)
{
	return __rtdm_task_sleep(delay, XN_RELATIVE);
}

static inline int
rtdm_task_sleep_abs(nanosecs_abs_t wakeup_date, enum rtdm_timer_mode mode)
{
	/* For the sake of a consistent API usage... */
	if (mode != RTDM_TIMERMODE_ABSOLUTE && mode != RTDM_TIMERMODE_REALTIME)
		return -EINVAL;
	return __rtdm_task_sleep(wakeup_date, (xntmode_t)mode);
}

/* rtdm_task_sleep_abs shall be used instead */
static inline int __deprecated rtdm_task_sleep_until(nanosecs_abs_t wakeup_time)
{
	return __rtdm_task_sleep(wakeup_time, XN_REALTIME);
}

#define rtdm_task_busy_wait(__condition, __spin_ns, __sleep_ns)			\
	({									\
		__label__ done;							\
		nanosecs_abs_t __end;						\
		int __ret = 0;							\
		for (;;) {							\
			__end = rtdm_clock_read_monotonic() + __spin_ns;	\
			for (;;) {						\
				if (__condition)				\
					goto done;				\
				if (rtdm_clock_read_monotonic() >= __end)	\
					break;					\
			}							\
			__ret = rtdm_task_sleep(__sleep_ns);			\
			if (__ret)						\
				break;						\
		}								\
	done:									\
		__ret;								\
	})

#define rtdm_wait_context	xnthread_wait_context

static inline
void rtdm_wait_complete(struct rtdm_wait_context *wc)
{
	xnthread_complete_wait(wc);
}

static inline
int rtdm_wait_is_completed(struct rtdm_wait_context *wc)
{
	return xnthread_wait_complete_p(wc);
}

static inline void rtdm_wait_prepare(struct rtdm_wait_context *wc)
{
	xnthread_prepare_wait(wc);
}

static inline
struct rtdm_wait_context *rtdm_wait_get_context(rtdm_task_t *task)
{
	return xnthread_get_wait_context(task);
}

#endif /* !DOXYGEN_CPP */

/* --- event services --- */

typedef struct rtdm_event {
	struct xnsynch synch_base;
	DECLARE_XNSELECT(select_block);
} rtdm_event_t;

#define RTDM_EVENT_PENDING		XNSYNCH_SPARE1

void rtdm_event_init(rtdm_event_t *event, unsigned long pending);
int rtdm_event_select(rtdm_event_t *event, rtdm_selector_t *selector,
		      enum rtdm_selecttype type, unsigned fd_index);
int rtdm_event_wait(rtdm_event_t *event);
int rtdm_event_timedwait(rtdm_event_t *event, nanosecs_rel_t timeout,
			 rtdm_toseq_t *timeout_seq);
void rtdm_event_signal(rtdm_event_t *event);

void rtdm_event_clear(rtdm_event_t *event);

void rtdm_event_pulse(rtdm_event_t *event);

void rtdm_event_destroy(rtdm_event_t *event);

/* --- semaphore services --- */

typedef struct rtdm_sem {
	unsigned long value;
	struct xnsynch synch_base;
	DECLARE_XNSELECT(select_block);
} rtdm_sem_t;

void rtdm_sem_init(rtdm_sem_t *sem, unsigned long value);
int rtdm_sem_select(rtdm_sem_t *sem, rtdm_selector_t *selector,
		    enum rtdm_selecttype type, unsigned fd_index);
int rtdm_sem_down(rtdm_sem_t *sem);
int rtdm_sem_timeddown(rtdm_sem_t *sem, nanosecs_rel_t timeout,
		       rtdm_toseq_t *timeout_seq);
void rtdm_sem_up(rtdm_sem_t *sem);

void rtdm_sem_destroy(rtdm_sem_t *sem);

/* --- mutex services --- */

typedef struct rtdm_mutex {
	struct xnsynch synch_base;
	atomic_t fastlock;
} rtdm_mutex_t;

void rtdm_mutex_init(rtdm_mutex_t *mutex);
int rtdm_mutex_lock(rtdm_mutex_t *mutex);
int rtdm_mutex_timedlock(rtdm_mutex_t *mutex, nanosecs_rel_t timeout,
			 rtdm_toseq_t *timeout_seq);
void rtdm_mutex_unlock(rtdm_mutex_t *mutex);
void rtdm_mutex_destroy(rtdm_mutex_t *mutex);

/* --- utility functions --- */

#define rtdm_printk(format, ...)	printk(format, ##__VA_ARGS__)

#define rtdm_printk_ratelimited(fmt, ...)  do {				\
	if (xnclock_ratelimit())					\
		printk(fmt, ##__VA_ARGS__);				\
} while (0)

#ifndef DOXYGEN_CPP /* Avoid static inline tags for RTDM in doxygen */
static inline void *rtdm_malloc(size_t size)
{
	return xnmalloc(size);
}

static inline void rtdm_free(void *ptr)
{
	xnfree(ptr);
}

int rtdm_mmap_to_user(struct rtdm_fd *fd,
		      void *src_addr, size_t len,
		      int prot, void **pptr,
		      struct vm_operations_struct *vm_ops,
		      void *vm_private_data);

int rtdm_iomap_to_user(struct rtdm_fd *fd,
		       phys_addr_t src_addr, size_t len,
		       int prot, void **pptr,
		       struct vm_operations_struct *vm_ops,
		       void *vm_private_data);

int rtdm_mmap_kmem(struct vm_area_struct *vma, void *va);

int rtdm_mmap_vmem(struct vm_area_struct *vma, void *va);

int rtdm_mmap_iomem(struct vm_area_struct *vma, phys_addr_t pa);

int rtdm_munmap(void *ptr, size_t len);

static inline int rtdm_read_user_ok(struct rtdm_fd *fd,
				    const void __user *ptr, size_t size)
{
	return access_rok(ptr, size);
}

static inline int rtdm_rw_user_ok(struct rtdm_fd *fd,
				  const void __user *ptr, size_t size)
{
	return access_wok(ptr, size);
}

static inline int rtdm_copy_from_user(struct rtdm_fd *fd,
				      void *dst, const void __user *src,
				      size_t size)
{
	return __xn_copy_from_user(dst, src, size) ? -EFAULT : 0;
}

static inline int rtdm_safe_copy_from_user(struct rtdm_fd *fd,
					   void *dst, const void __user *src,
					   size_t size)
{
	return cobalt_copy_from_user(dst, src, size);
}

static inline int rtdm_copy_to_user(struct rtdm_fd *fd,
				    void __user *dst, const void *src,
				    size_t size)
{
	return __xn_copy_to_user(dst, src, size) ? -EFAULT : 0;
}

static inline int rtdm_safe_copy_to_user(struct rtdm_fd *fd,
					 void __user *dst, const void *src,
					 size_t size)
{
	return cobalt_copy_to_user(dst, src, size);
}

static inline int rtdm_strncpy_from_user(struct rtdm_fd *fd,
					 char *dst,
					 const char __user *src, size_t count)
{
	return cobalt_strncpy_from_user(dst, src, count);
}

static inline bool rtdm_available(void)
{
	return realtime_core_enabled();
}

static inline int rtdm_rt_capable(struct rtdm_fd *fd)
{
	if (!XENO_ASSERT(COBALT, !xnsched_interrupt_p()))
		return 0;

	if (!rtdm_fd_is_user(fd))
		return !xnsched_root_p();

	return xnthread_current() != NULL;
}

static inline int rtdm_in_rt_context(void)
{
	return (ipipe_current_domain != ipipe_root_domain);
}

#define RTDM_IOV_FASTMAX  16

int rtdm_get_iovec(struct rtdm_fd *fd, struct iovec **iov,
		   const struct user_msghdr *msg,
		   struct iovec *iov_fast);

int rtdm_put_iovec(struct rtdm_fd *fd, struct iovec *iov,
		   const struct user_msghdr *msg,
		   struct iovec *iov_fast);

static inline
void rtdm_drop_iovec(struct iovec *iov, struct iovec *iov_fast)
{
	if (iov != iov_fast)
		xnfree(iov);
}

ssize_t rtdm_get_iov_flatlen(struct iovec *iov, int iovlen);

#endif /* !DOXYGEN_CPP */

#endif /* _COBALT_RTDM_DRIVER_H */
