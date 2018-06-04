/*
 * This file is part of the Xenomai project.
 *
 * Copyright (C) 2014 Philippe Gerum <rpm@xenomai.org>
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
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <rtdm/cobalt.h>
#include <rtdm/driver.h>
#include <rtdm/udd.h>

struct udd_context {
	u32 event_count;
};

static int udd_open(struct rtdm_fd *fd, int oflags)
{
	struct udd_context *context;
	struct udd_device *udd;
	int ret;

	udd = container_of(rtdm_fd_device(fd), struct udd_device, __reserved.device);
	if (udd->ops.open) {
		ret = udd->ops.open(fd, oflags);
		if (ret)
			return ret;
	}

	context = rtdm_fd_to_private(fd);
	context->event_count = 0;

	return 0;
}

static void udd_close(struct rtdm_fd *fd)
{
	struct udd_device *udd;

	udd = container_of(rtdm_fd_device(fd), struct udd_device, __reserved.device);
	if (udd->ops.close)
		udd->ops.close(fd);
}

static int udd_ioctl_rt(struct rtdm_fd *fd,
			unsigned int request, void __user *arg)
{
	struct udd_signotify signfy;
	struct udd_reserved *ur;
	struct udd_device *udd;
	rtdm_event_t done;
	int ret;

	udd = container_of(rtdm_fd_device(fd), struct udd_device, __reserved.device);
	if (udd->ops.ioctl) {
		ret = udd->ops.ioctl(fd, request, arg);
		if (ret != -ENOSYS)
			return ret;
	}

	ur = &udd->__reserved;

	switch (request) {
	case UDD_RTIOC_IRQSIG:
		ret = rtdm_safe_copy_from_user(fd, &signfy, arg, sizeof(signfy));
		if (ret)
			return ret;
		/* Early check, we'll redo at each signal issue. */
		if (signfy.pid <= 0)
			ur->signfy.pid = -1;
		else {
			if (signfy.sig < SIGRTMIN || signfy.sig > SIGRTMAX)
				return -EINVAL;
			if (cobalt_thread_find_local(signfy.pid) == NULL)
				return -EINVAL;
			ur->signfy = signfy;
		}
		break;
	case UDD_RTIOC_IRQEN:
	case UDD_RTIOC_IRQDIS:
		if (udd->irq == UDD_IRQ_NONE || udd->irq == UDD_IRQ_CUSTOM)
			return -EIO;
		rtdm_event_init(&done, 0);
		if (request == UDD_RTIOC_IRQEN)
			udd_enable_irq(udd, &done);
		else
			udd_disable_irq(udd, &done);
		ret = rtdm_event_wait(&done);
		if (ret != -EIDRM)
			rtdm_event_destroy(&done);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static ssize_t udd_read_rt(struct rtdm_fd *fd,
			   void __user *buf, size_t len)
{
	struct udd_context *context;
	struct udd_reserved *ur;
	struct udd_device *udd;
	ssize_t ret;
	u32 count;

	if (len != sizeof(count))
		return -EINVAL;

	udd = container_of(rtdm_fd_device(fd), struct udd_device, __reserved.device);
	if (udd->irq == UDD_IRQ_NONE)
		return -EIO;

	ur = &udd->__reserved;
	context = rtdm_fd_to_private(fd);

	for (;;) {
		if (atomic_read(&ur->event) != context->event_count)
			break;
		ret = rtdm_event_wait(&ur->pulse);
		if (ret)
			return ret;
	}

	count = atomic_read(&ur->event);
	context->event_count = count;
	ret = rtdm_copy_to_user(fd, buf, &count, sizeof(count));

	return ret ?: sizeof(count);
}

static ssize_t udd_write_rt(struct rtdm_fd *fd,
			    const void __user *buf, size_t len)
{
	int ret;
	u32 val;

	if (len != sizeof(val))
		return -EINVAL;

	ret = rtdm_safe_copy_from_user(fd, &val, buf, sizeof(val));
	if (ret)
		return ret;

	ret = udd_ioctl_rt(fd, val ? UDD_RTIOC_IRQEN : UDD_RTIOC_IRQDIS, NULL);

	return ret ?: len;
}

static int udd_select(struct rtdm_fd *fd, struct xnselector *selector,
		      unsigned int type, unsigned int index)
{
	struct udd_device *udd;

	udd = container_of(rtdm_fd_device(fd), struct udd_device, __reserved.device);
	if (udd->irq == UDD_IRQ_NONE)
		return -EIO;

	return rtdm_event_select(&udd->__reserved.pulse,
				 selector, type, index);
}

static int udd_irq_handler(rtdm_irq_t *irqh)
{
	struct udd_device *udd;
	int ret;

	udd = rtdm_irq_get_arg(irqh, struct udd_device);
	ret = udd->ops.interrupt(udd);
	if (ret == RTDM_IRQ_HANDLED)
		udd_notify_event(udd);

	return ret;
}

static int mapper_open(struct rtdm_fd *fd, int oflags)
{
	int minor = rtdm_fd_minor(fd);
	struct udd_device *udd;

	/*
	 * Check that we are opening a mapper instance pointing at a
	 * valid memory region. e.g. UDD creates the companion device
	 * "foo,mapper" on the fly when registering the main device
	 * "foo". Userland may then open("/dev/foo,mapper0", ...)
	 * followed by a call to mmap() for mapping the memory region
	 * #0 as declared in the mem_regions[] array of the main
	 * device.
	 *
	 * We support sparse region arrays, so the device minor shall
	 * match the mem_regions[] index exactly.
	 */
	if (minor < 0 || minor >= UDD_NR_MAPS)
		return -EIO;

	udd = udd_get_device(fd);
	if (udd->mem_regions[minor].type == UDD_MEM_NONE)
		return -EIO;

	return 0;
}

static void mapper_close(struct rtdm_fd *fd)
{
	/* nop */
}

static int mapper_mmap(struct rtdm_fd *fd, struct vm_area_struct *vma)
{
	struct udd_memregion *rn;
	struct udd_device *udd;
	size_t len;
	int ret;

	udd = udd_get_device(fd);
	if (udd->ops.mmap)
		/* Offload to client driver if handler is present. */
		return udd->ops.mmap(fd, vma);

	/* Otherwise DIY using the RTDM helpers. */

	len = vma->vm_end - vma->vm_start;
	rn = udd->mem_regions + rtdm_fd_minor(fd);
	if (rn->len < len)
		/* Can't map that much, bail out. */
		return -EINVAL;

	switch (rn->type) {
	case UDD_MEM_PHYS:
		ret = rtdm_mmap_iomem(vma, rn->addr);
		break;
	case UDD_MEM_LOGICAL:
		ret = rtdm_mmap_kmem(vma, (void *)rn->addr);
		break;
	case UDD_MEM_VIRTUAL:
		ret = rtdm_mmap_vmem(vma, (void *)rn->addr);
		break;
	default:
		ret = -EINVAL;	/* Paranoid, can't happen. */
	}

	return ret;
}

static inline int check_memregion(struct udd_device *udd,
				  struct udd_memregion *rn)
{
	if (rn->name == NULL)
		return -EINVAL;

	if (rn->addr == 0)
		return -EINVAL;

	if (rn->len == 0)
		return -EINVAL;

	return 0;
}

static inline int register_mapper(struct udd_device *udd)
{
	struct udd_reserved *ur = &udd->__reserved;
	struct rtdm_driver *drv = &ur->mapper_driver;
	struct udd_mapper *mapper;
	struct udd_memregion *rn;
	int n, ret;

	ur->mapper_name = kasformat("%s,mapper%%d", udd->device_name);
	if (ur->mapper_name == NULL)
		return -ENOMEM;

	drv->profile_info = (struct rtdm_profile_info)
		RTDM_PROFILE_INFO(mapper, RTDM_CLASS_MEMORY,
				  RTDM_SUBCLASS_GENERIC, 0);
	drv->device_flags = RTDM_NAMED_DEVICE|RTDM_FIXED_MINOR;
	drv->device_count = UDD_NR_MAPS;
	drv->base_minor = 0;
	drv->ops = (struct rtdm_fd_ops){
		.open		=	mapper_open,
		.close		=	mapper_close,
		.mmap		=	mapper_mmap,
	};

	for (n = 0, mapper = ur->mapdev; n < UDD_NR_MAPS; n++, mapper++) {
		rn = udd->mem_regions + n;
		if (rn->type == UDD_MEM_NONE)
			continue;
		mapper->dev.driver = drv;
		mapper->dev.label = ur->mapper_name;
		mapper->dev.minor = n;
		mapper->udd = udd;
		ret = rtdm_dev_register(&mapper->dev);
		if (ret)
			goto undo;
	}

	return 0;
undo:
	while (--n >= 0)
		rtdm_dev_unregister(&ur->mapdev[n].dev);

	return ret;
}

/**
 * @brief Register a UDD device
 *
 * This routine registers a mini-driver at the UDD core.
 *
 * @param udd @ref udd_device "UDD device descriptor" which should
 * describe the new device properties.
 *
 * @return Zero is returned upon success, otherwise a negative error
 * code is received, from the set of error codes defined by
 * rtdm_dev_register(). In addition, the following error codes can be
 * returned:
 *
 * - -EINVAL, some of the memory regions declared in the
 *   udd_device.mem_regions[] array have invalid properties, i.e. bad
 *   type, NULL name, zero length or address. Any undeclared region
 *   entry from the array must bear the UDD_MEM_NONE type.
 *
 * - -EINVAL, if udd_device.irq is different from UDD_IRQ_CUSTOM and
 * UDD_IRQ_NONE but invalid, causing rtdm_irq_request() to fail.
 *
 * - -EINVAL, if udd_device.device_flags contains invalid flags.
 *
 * - -ENXIO can be received if this service is called while the Cobalt
 * kernel is disabled.
 *
 * @coretags{secondary-only}
 */
int udd_register_device(struct udd_device *udd)
{
	struct rtdm_device *dev = &udd->__reserved.device;
	struct udd_reserved *ur = &udd->__reserved;
	struct rtdm_driver *drv = &ur->driver;
	struct udd_memregion *rn;
	int ret, n;

	if (!realtime_core_enabled())
		return -ENXIO;

	if (udd->device_flags & RTDM_PROTOCOL_DEVICE)
		return -EINVAL;

	if (udd->irq != UDD_IRQ_NONE && udd->irq != UDD_IRQ_CUSTOM &&
	    udd->ops.interrupt == NULL)
		return -EINVAL;

	for (n = 0, ur->nr_maps = 0; n < UDD_NR_MAPS; n++) {
		/* We allow sparse region arrays. */
		rn = udd->mem_regions + n;
		if (rn->type == UDD_MEM_NONE)
			continue;
		ret = check_memregion(udd, rn);
		if (ret)
			return ret;
		udd->__reserved.nr_maps++;
	}

	drv->profile_info = (struct rtdm_profile_info)
		RTDM_PROFILE_INFO(udd->device_name, RTDM_CLASS_UDD,
				  udd->device_subclass, 0);
	drv->device_flags = RTDM_NAMED_DEVICE|udd->device_flags;
	drv->device_count = 1;
	drv->context_size = sizeof(struct udd_context);
	drv->ops = (struct rtdm_fd_ops){
		.open = udd_open,
		.ioctl_rt = udd_ioctl_rt,
		.read_rt = udd_read_rt,
		.write_rt = udd_write_rt,
		.close = udd_close,
		.select = udd_select,
	};

	dev->driver = drv;
	dev->label = udd->device_name;

	ret = rtdm_dev_register(dev);
	if (ret)
		return ret;

	if (ur->nr_maps > 0) {
		ret = register_mapper(udd);
		if (ret)
			goto fail_mapper;
	} else
		ur->mapper_name = NULL;

	atomic_set(&ur->event, 0);
	rtdm_event_init(&ur->pulse, 0);
	ur->signfy.pid = -1;

	if (udd->irq != UDD_IRQ_NONE && udd->irq != UDD_IRQ_CUSTOM) {
		ret = rtdm_irq_request(&ur->irqh, udd->irq,
				       udd_irq_handler, 0,
				       dev->name, udd);
		if (ret)
			goto fail_irq_request;
	}

	return 0;

fail_irq_request:
	for (n = 0; n < UDD_NR_MAPS; n++) {
		rn = udd->mem_regions + n;
		if (rn->type != UDD_MEM_NONE)
			rtdm_dev_unregister(&ur->mapdev[n].dev);
	}
fail_mapper:
	rtdm_dev_unregister(dev);
	if (ur->mapper_name)
		kfree(ur->mapper_name);

	return ret;
}
EXPORT_SYMBOL_GPL(udd_register_device);

/**
 * @brief Unregister a UDD device
 *
 * This routine unregisters a mini-driver from the UDD core. This
 * routine waits until all connections to @a udd have been closed
 * prior to unregistering.
 *
 * @param udd UDD device descriptor
 *
 * @return Zero is returned upon success, otherwise -ENXIO is received
 * if this service is called while the Cobalt kernel is disabled.
 *
 * @coretags{secondary-only}
 */
int udd_unregister_device(struct udd_device *udd)
{
	struct udd_reserved *ur = &udd->__reserved;
	struct udd_memregion *rn;
	int n;

	if (!realtime_core_enabled())
		return -ENXIO;

	rtdm_event_destroy(&ur->pulse);

	if (udd->irq != UDD_IRQ_NONE && udd->irq != UDD_IRQ_CUSTOM)
		rtdm_irq_free(&ur->irqh);

	for (n = 0; n < UDD_NR_MAPS; n++) {
		rn = udd->mem_regions + n;
		if (rn->type != UDD_MEM_NONE)
			rtdm_dev_unregister(&ur->mapdev[n].dev);
	}

	if (ur->mapper_name)
		kfree(ur->mapper_name);

	rtdm_dev_unregister(&ur->device);

	return 0;
}
EXPORT_SYMBOL_GPL(udd_unregister_device);

/**
 * @brief Notify an IRQ event for an unmanaged interrupt
 *
 * When the UDD core shall hand over the interrupt management for a
 * device to the mini-driver (see UDD_IRQ_CUSTOM), the latter should
 * notify the UDD core when IRQ events are received by calling this
 * service.
 *
 * As a result, the UDD core wakes up any Cobalt thread waiting for
 * interrupts on the device via a read(2) or select(2) call.
 *
 * @param udd UDD device descriptor receiving the IRQ.
 *
 * @coretags{coreirq-only}
 *
 * @note In case the @ref udd_irq_handler "IRQ handler" from the
 * mini-driver requested the UDD core not to re-enable the interrupt
 * line, the application may later request the unmasking by issuing
 * the UDD_RTIOC_IRQEN ioctl(2) command. Writing a non-zero integer to
 * the device via the write(2) system call has the same effect.
 */
void udd_notify_event(struct udd_device *udd)
{
	struct udd_reserved *ur = &udd->__reserved;
	union sigval sival;

	atomic_inc(&ur->event);
	rtdm_event_signal(&ur->pulse);

	if (ur->signfy.pid > 0) {
		sival.sival_int = atomic_read(&ur->event);
		__cobalt_sigqueue(ur->signfy.pid, ur->signfy.sig, &sival);
	}
}
EXPORT_SYMBOL_GPL(udd_notify_event);

struct irqswitch_work {
	struct ipipe_work_header work; /* Must be first. */
	rtdm_irq_t *irqh;
	int enabled;
	rtdm_event_t *done;
};

static void lostage_irqswitch_line(struct ipipe_work_header *work)
{
	struct irqswitch_work *rq;

	/*
	 * This runs from secondary mode, we may flip the IRQ state
	 * now.
	 */
	rq = container_of(work, struct irqswitch_work, work);
	if (rq->enabled)
		rtdm_irq_enable(rq->irqh);
	else
		rtdm_irq_disable(rq->irqh);

	if (rq->done)
		rtdm_event_signal(rq->done);
}

static void switch_irq_line(rtdm_irq_t *irqh, int enable, rtdm_event_t *done)
{
	struct irqswitch_work switchwork = {
		.work = {
			.size = sizeof(switchwork),
			.handler = lostage_irqswitch_line,
		},
		.irqh = irqh,
		.enabled = enable,
		.done = done,
	};

	/*
	 * Not pretty, but we may not traverse the kernel code for
	 * enabling/disabling IRQ lines from primary mode. So we have
	 * to send a deferrable root request (i.e. low-level APC) to
	 * be callable from real-time context.
	 */
	ipipe_post_work_root(&switchwork, work);
}

/**
 * @brief Enable the device IRQ line
 *
 * This service issues a request to the regular kernel for enabling
 * the IRQ line registered by the driver. If the caller runs in
 * primary mode, the request is scheduled but deferred until the
 * current CPU leaves the real-time domain (see note). Otherwise, the
 * request is immediately handled.
 *
 * @param udd The UDD driver handling the IRQ to disable. If no IRQ
 * was registered by the driver at the UDD core, this routine has no
 * effect.
 *
 * @param done Optional event to signal upon completion. If non-NULL,
 * @a done will be posted by a call to rtdm_event_signal() after the
 * interrupt line is enabled.
 *
 * @coretags{unrestricted}
 *
 * @note The deferral is required as some interrupt management code
 * involved in enabling interrupt lines may not be safely executed
 * from primary mode. By passing a valid @a done object address, the
 * caller can wait for the request to complete, by sleeping on
 * rtdm_event_wait().
 */
void udd_enable_irq(struct udd_device *udd, rtdm_event_t *done)
{
	struct udd_reserved *ur = &udd->__reserved;

	if (udd->irq != UDD_IRQ_NONE && udd->irq != UDD_IRQ_CUSTOM)
		switch_irq_line(&ur->irqh, 1, done);
}
EXPORT_SYMBOL_GPL(udd_enable_irq);

/**
 * @brief Disable the device IRQ line
 *
 * This service issues a request to the regular kernel for disabling
 * the IRQ line registered by the driver. If the caller runs in
 * primary mode, the request is scheduled but deferred until the
 * current CPU leaves the real-time domain (see note). Otherwise, the
 * request is immediately handled.
 *
 * @param udd The UDD driver handling the IRQ to disable. If no IRQ
 * was registered by the driver at the UDD core, this routine has no
 * effect.
 *
 * @param done Optional event to signal upon completion. If non-NULL,
 * @a done will be posted by a call to rtdm_event_signal() after the
 * interrupt line is disabled.
 *
 * @coretags{unrestricted}
 *
 * @note The deferral is required as some interrupt management code
 * involved in disabling interrupt lines may not be safely executed
 * from primary mode. By passing a valid @a done object address, the
 * caller can wait for the request to complete, by sleeping on
 * rtdm_event_wait().
 */
void udd_disable_irq(struct udd_device *udd, rtdm_event_t *done)
{
	struct udd_reserved *ur = &udd->__reserved;

	if (udd->irq != UDD_IRQ_NONE && udd->irq != UDD_IRQ_CUSTOM)
		switch_irq_line(&ur->irqh, 0, done);
}
EXPORT_SYMBOL_GPL(udd_disable_irq);

/**
 * @brief RTDM file descriptor to target UDD device
 *
 * Retrieves the UDD device from a RTDM file descriptor.
 *
 * @param fd File descriptor received by an ancillary I/O handler
 * from a mini-driver based on the UDD core.
 *
 * @return A pointer to the UDD device to which @a fd refers to.
 *
 * @note This service is intended for use by mini-drivers based on the
 * UDD core exclusively. Passing file descriptors referring to other
 * RTDM devices will certainly lead to invalid results.
 *
 * @coretags{mode-unrestricted}
 */
struct udd_device *udd_get_device(struct rtdm_fd *fd)
{
	struct rtdm_device *dev = rtdm_fd_device(fd);

	if (dev->driver->profile_info.class_id == RTDM_CLASS_MEMORY)
		return container_of(dev, struct udd_mapper, dev)->udd;

	return container_of(dev, struct udd_device, __reserved.device);
}
EXPORT_SYMBOL_GPL(udd_get_device);

MODULE_LICENSE("GPL");
