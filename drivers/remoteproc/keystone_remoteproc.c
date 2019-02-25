// SPDX-License-Identifier: GPL-2.0
/*
 * TI Keystone DSP remoteproc driver
 *
 * Copyright (C) 2015-2019 Texas Instruments Incorporated - http://www.ti.com/
 */

#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/workqueue.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/remoteproc.h>
#include <linux/miscdevice.h>
#include <linux/uio_driver.h>
#include <linux/reset.h>

#include <uapi/linux/keystone_remoteproc.h>

#include "remoteproc_internal.h"

#define DRIVER_UIO_VERSION			"0.1"

#define KEYSTONE_RPROC_MAX_RSC_TABLE		SZ_1K
#define KEYSTONE_RPROC_LOCAL_ADDRESS_MASK	(SZ_16M - 1)

/*
 * XXX: evaluate if this param needs to be enhanced so that the switch between
 * userspace and remoteproc core loaders can be controlled per device.
 */
static bool use_rproc_core_loader;
module_param(use_rproc_core_loader, bool, 0444);

/**
 * struct keystone_rproc_mem - internal memory structure
 * @cpu_addr: MPU virtual address of the memory region
 * @bus_addr: Bus address used to access the memory region
 * @dev_addr: Device address of the memory region from DSP view
 * @size: Size of the memory region
 * @kobj: kobject for the sysfs directory file
 */
struct keystone_rproc_mem {
	void __iomem *cpu_addr;
	phys_addr_t bus_addr;
	u32 dev_addr;
	size_t size;
	struct kobject kobj;
};

/**
 * struct keystone_rproc - keystone remote processor driver structure
 * @dev: cached device pointer
 * @rproc: remoteproc device handle
 * @mem: internal memory regions data
 * @num_mems: number of internal memory regions
 * @dev_ctrl: device control regmap handle
 * @reset: reset control handle
 * @boot_offset: boot register offset in @dev_ctrl regmap
 * @irq_ring: irq entry for vring
 * @irq_fault: irq entry for exception
 * @kick_gpio: gpio used for virtio kicks
 * @workqueue: workqueue for processing virtio interrupts
 * @misc: misc device structure used to expose fops to user-space
 * @uio: uio device information
 * @mlock: lock to protect resources in fops
 * @lock: lock to protect shared resources within UIO interrupt handlers
 * @flags: flags to keep track of UIO interrupt occurrence
 * @rsc_table: resource table pointer copied from userspace
 * @rsc_table_size: size of resource table
 * @loaded_rsc_table: kernel pointer of loaded resource table
 * @boot_addr: remote processor boot address used with userspace loader
 * @open_count: fops open reference counter
 * @use_userspace_loader: flag to denote if driver is configured for userspace
 *			  loader
 */
struct keystone_rproc {
	struct device *dev;
	struct rproc *rproc;
	struct keystone_rproc_mem *mem;
	int num_mems;
	struct regmap *dev_ctrl;
	struct reset_control *reset;
	u32 boot_offset;
	int irq_ring;
	int irq_fault;
	int kick_gpio;
	struct work_struct workqueue;
	struct miscdevice misc;
	struct uio_info uio;
	struct mutex mlock; /* fops lock */
	spinlock_t lock; /* uio handler lock */
	unsigned long flags;
	struct resource_table *rsc_table;
	int rsc_table_size;
	void *loaded_rsc_table;
	u32 boot_addr;
	int open_count;
	unsigned int use_userspace_loader : 1;
};

struct mem_sysfs_entry {
	struct attribute attr;
	ssize_t (*show)(struct keystone_rproc_mem *mem, char *buf);
	ssize_t (*store)(struct keystone_rproc_mem *mem, const char *buf,
			 size_t len);
};

static ssize_t mem_addr_show(struct keystone_rproc_mem *mem, char *buf)
{
	return sprintf(buf, "%pa\n", &mem->bus_addr);
}

static ssize_t mem_size_show(struct keystone_rproc_mem *mem, char *buf)
{
	return sprintf(buf, "0x%016zx\n", mem->size);
}

static struct mem_sysfs_entry addr_attribute =
	__ATTR(addr, 0444, mem_addr_show, NULL);
static struct mem_sysfs_entry size_attribute =
	__ATTR(size, 0444, mem_size_show, NULL);

static struct attribute *attrs[] = {
	&addr_attribute.attr,
	&size_attribute.attr,
	NULL,	/* sentinel */
};

#define to_dsp_mem(m) container_of(m, struct keystone_rproc_mem, kobj)

static ssize_t mem_type_show(struct kobject *kobj, struct attribute *attr,
			     char *buf)
{
	struct keystone_rproc_mem *mem = to_dsp_mem(kobj);
	struct mem_sysfs_entry *entry;

	entry = container_of(attr, struct mem_sysfs_entry, attr);
	if (!entry->show)
		return -EIO;

	return entry->show(mem, buf);
}

static const struct sysfs_ops mem_sysfs_ops = {
	.show = mem_type_show,
};

static struct kobj_type mem_attr_type = {
	.sysfs_ops	= &mem_sysfs_ops,
	.default_attrs	= attrs,
};

static int keystone_rproc_mem_add_attrs(struct keystone_rproc *ksproc)
{
	int i, ret;
	struct keystone_rproc_mem *mem;
	struct kobject *kobj_parent = &ksproc->misc.this_device->kobj;

	for (i = 0; i < ksproc->num_mems; i++) {
		mem = &ksproc->mem[i];
		kobject_init(&mem->kobj, &mem_attr_type);
		ret = kobject_add(&mem->kobj, kobj_parent, "memory%d", i);
		if (ret)
			goto err_kobj;
		ret = kobject_uevent(&mem->kobj, KOBJ_ADD);
		if (ret)
			goto err_kobj;
	}

	return 0;

err_kobj:
	for (; i >= 0; i--) {
		mem = &ksproc->mem[i];
		kobject_put(&mem->kobj);
	}
	return ret;
}

static void keystone_rproc_mem_del_attrs(struct keystone_rproc *ksproc)
{
	int i;
	struct keystone_rproc_mem *mem;

	for (i = 0; i < ksproc->num_mems; i++) {
		mem = &ksproc->mem[i];
		kobject_put(&mem->kobj);
	}
}

static void *keystone_rproc_da_to_va(struct rproc *rproc, u64 da, int len,
				     u32 flags);

/* uio handler dealing with userspace controlled exception interrupt */
static irqreturn_t keystone_rproc_uio_handler(int irq, struct uio_info *uio)
{
	struct keystone_rproc *ksproc = uio->priv;

	spin_lock(&ksproc->lock);
	if (!__test_and_set_bit(0, &ksproc->flags))
		disable_irq_nosync(irq);
	spin_unlock(&ksproc->lock);

	return IRQ_HANDLED;
}

/* uio driver interrupt control dealing with exception interrupt */
static int keystone_rproc_uio_irqcontrol(struct uio_info *uio, s32 irq_on)
{
	struct keystone_rproc *ksproc = uio->priv;
	unsigned long flags;

	spin_lock_irqsave(&ksproc->lock, flags);
	if (irq_on) {
		if (__test_and_clear_bit(0, &ksproc->flags))
			enable_irq(uio->irq);
	} else {
		if (!__test_and_set_bit(0, &ksproc->flags))
			disable_irq(uio->irq);
	}
	spin_unlock_irqrestore(&ksproc->lock, flags);

	return 0;
}

/* Reset previously set rsc table variables */
static void keystone_rproc_reset_rsc_table(struct keystone_rproc *ksproc)
{
	kfree(ksproc->rsc_table);
	ksproc->rsc_table = NULL;
	ksproc->loaded_rsc_table = NULL;
	ksproc->rsc_table_size = 0;
}

/*
 * Create/delete the virtio devices in kernel once the user-space loading is
 * complete, configure the remoteproc states appropriately, and boot or reset
 * the remote processor. The resource table should have been published through
 * KEYSTONE_RPROC_IOC_SET_RSC_TABLE & KEYSTONE_RPROC_IOC_SET_LOADED_RSC_TABLE
 * ioctls before invoking this. The boot address is passed through the
 * KEYSTONE_RPROC_IOC_SET_STATE ioctl when setting the KEYSTONE_RPROC_RUNNING
 * state.
 *
 * NOTE:
 * The ioctls KEYSTONE_RPROC_IOC_DSP_RESET and KEYSTONE_RPROC_IOC_DSP_BOOT
 * are restricted to support the booting or resetting the DSP devices only
 * for firmware images without any resource table.
 */
static int keystone_rproc_set_state(struct keystone_rproc *ksproc,
				    void __user *argp)
{
	struct rproc *rproc = ksproc->rproc;
	struct keystone_rproc_set_state_params set_state_params;
	int ret = 0;

	if (copy_from_user(&set_state_params, argp, sizeof(set_state_params)))
		return -EFAULT;

	switch (set_state_params.state) {
	case KEYSTONE_RPROC_RUNNING:
		if (!ksproc->rsc_table || !ksproc->loaded_rsc_table)
			return -EINVAL;

		/*
		 * store boot address for .get_boot_addr() rproc fw ops
		 * XXX: validate the boot address so it is not set to a
		 * random address
		 */
		ksproc->boot_addr = set_state_params.boot_addr;

		/*
		 * invoke rproc_boot to trigger the boot, the resource table
		 * is parsed during the process and is agnostic of the presence
		 * or absence of virtio devices
		 */
		ret = rproc_boot(rproc);
		break;

	case KEYSTONE_RPROC_OFFLINE:
		if (rproc->state != RPROC_RUNNING)
			return -EINVAL;

		/* invoke rproc_shutdown to match rproc_boot */
		rproc_shutdown(rproc);

		mutex_lock(&ksproc->mlock);
		keystone_rproc_reset_rsc_table(ksproc);
		mutex_unlock(&ksproc->mlock);

		break;

	default:
		ret = -ENOTSUPP;
	}

	return ret;
}

/* Copy the resource table from userspace into kernel */
static int keystone_rproc_set_rsc_table(struct keystone_rproc *ksproc,
					void __user *data)
{
	unsigned long len = 0;
	void *rsc_table = NULL;

	if (!data)
		return -EFAULT;

	if (copy_from_user(&len, data, sizeof(len)))
		return -EFAULT;

	if (len >= KEYSTONE_RPROC_MAX_RSC_TABLE)
		return -EOVERFLOW;

	data += sizeof(len);

	rsc_table = kzalloc(len, GFP_KERNEL);
	if (!rsc_table)
		return -ENOMEM;

	if (copy_from_user(rsc_table, data, len))
		goto error_return;

	mutex_lock(&ksproc->mlock);

	kfree(ksproc->rsc_table);

	ksproc->rsc_table = rsc_table;
	ksproc->rsc_table_size = len;
	ksproc->loaded_rsc_table = NULL;

	mutex_unlock(&ksproc->mlock);

	return 0;

error_return:
	kfree(rsc_table);
	return -EFAULT;
}

/*
 * Store the equivalent kernel virtual address of the loaded resource table in
 * device memory. Userspace published the device address of the loaded resource
 * table.
 */
static int keystone_rproc_set_loaded_rsc_table(struct keystone_rproc *ksproc,
					       unsigned int dma_addr)
{
	struct rproc *rproc = ksproc->rproc;
	void *ptr;

	if (!ksproc->rsc_table_size || !ksproc->rsc_table)
		return -EINVAL;

	ptr = keystone_rproc_da_to_va(rproc, dma_addr, ksproc->rsc_table_size,
				      RPROC_FLAGS_NONE);
	if (!ptr)
		return -EINVAL;

	ksproc->loaded_rsc_table = ptr;

	return 0;
}

/* Put the DSP processor into reset */
static void keystone_rproc_dsp_reset(struct keystone_rproc *ksproc)
{
	reset_control_assert(ksproc->reset);
}

/* Configure the boot address and boot the DSP processor */
static int keystone_rproc_dsp_boot(struct keystone_rproc *ksproc,
				   uint32_t boot_addr)
{
	int ret;

	if (boot_addr & (SZ_1K - 1)) {
		dev_err(ksproc->dev, "invalid boot address 0x%x, must be aligned on a 1KB boundary\n",
			boot_addr);
		return -EINVAL;
	}

	ret = regmap_write(ksproc->dev_ctrl, ksproc->boot_offset, boot_addr);
	if (ret) {
		dev_err(ksproc->dev, "regmap_write of boot address failed, status = %d\n",
			ret);
		return ret;
	}

	reset_control_deassert(ksproc->reset);

	return 0;
}

static long
keystone_rproc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct miscdevice *misc = filp->private_data;
	struct keystone_rproc *ksproc =
		container_of(misc, struct keystone_rproc, misc);
	void __user *argp = (void __user *)arg;
	int ret = 0;

	dev_dbg(ksproc->dev, "%s: cmd 0x%.8x (%d), arg 0x%lx\n",
		__func__, cmd, _IOC_NR(cmd), arg);

	if (_IOC_TYPE(cmd) != KEYSTONE_RPROC_IOC_MAGIC)
		return -ENOTTY;

	if (_IOC_NR(cmd) >= KEYSTONE_RPROC_IOC_MAXNR)
		return -ENOTTY;

	switch (cmd) {
	case KEYSTONE_RPROC_IOC_SET_STATE:
		ret = keystone_rproc_set_state(ksproc, argp);
		break;

	case KEYSTONE_RPROC_IOC_SET_RSC_TABLE:
		ret = keystone_rproc_set_rsc_table(ksproc, argp);
		break;

	case KEYSTONE_RPROC_IOC_SET_LOADED_RSC_TABLE:
		ret = keystone_rproc_set_loaded_rsc_table(ksproc, arg);
		break;

	case KEYSTONE_RPROC_IOC_DSP_RESET:
		if (ksproc->rsc_table) {
			ret = -EINVAL;
			break;
		}

		keystone_rproc_dsp_reset(ksproc);
		break;

	case KEYSTONE_RPROC_IOC_DSP_BOOT:
		if (ksproc->rsc_table) {
			ret = -EINVAL;
			break;
		}

		ret = keystone_rproc_dsp_boot(ksproc, arg);
		break;

	default:
		ret = -ENOTTY;
		break;
	}

	if (ret) {
		dev_err(ksproc->dev, "error in ioctl call: cmd 0x%.8x (%d), ret %d\n",
			cmd, _IOC_NR(cmd), ret);
	}

	return ret;
}

/*
 * Map DSP memories into userspace for supporting Userspace loading.
 *
 * This is a custom mmap function following semantics based on the UIO
 * mmap implementation. The vm_pgoff passed in the vma structure is a
 * combination of the memory region index and the actual page offset in
 * that region. This checks if user request is in valid range before
 * providing mmap access.
 *
 * XXX: Evaluate this approach, as the internal memories can be mapped in
 * whole into userspace as they are not super-large, or switch to using
 * direct addresses to look more like a traditional implementation.
 */
static int keystone_rproc_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct miscdevice *misc = file->private_data;
	struct keystone_rproc *ksproc =
		container_of(misc, struct keystone_rproc, misc);
	size_t size = vma->vm_end - vma->vm_start;
	size_t req_offset;
	u32 idx;

	idx = vma->vm_pgoff & KEYSTONE_RPROC_UIO_MAP_INDEX_MASK;

	if (idx >= ksproc->num_mems) {
		dev_err(ksproc->dev, "invalid mmap region index %d\n", idx);
		return -EINVAL;
	}

	req_offset = (vma->vm_pgoff - idx) << PAGE_SHIFT;
	if (req_offset + size < req_offset) {
		dev_err(ksproc->dev, "invalid request - overflow, mmap offset = 0x%zx size 0x%zx region %d\n",
			req_offset, size, idx);
		return -EINVAL;
	}

	if ((req_offset + size) > ksproc->mem[idx].size) {
		dev_err(ksproc->dev, "invalid request - out of range, mmap offset 0x%zx size 0x%zx region %d\n",
			req_offset, size, idx);
		return -EINVAL;
	}

	vma->vm_page_prot =
		phys_mem_access_prot(file,
				     (ksproc->mem[idx].bus_addr >> PAGE_SHIFT) +
				     (vma->vm_pgoff - idx), size,
				     vma->vm_page_prot);

	if (remap_pfn_range(vma, vma->vm_start,
			    (ksproc->mem[idx].bus_addr >> PAGE_SHIFT) +
			    (vma->vm_pgoff - idx), size, vma->vm_page_prot))
		return -EAGAIN;

	return 0;
}

static int keystone_rproc_open(struct inode *inode, struct file *file)
{
	struct miscdevice *misc = file->private_data;
	struct keystone_rproc *ksproc =
		container_of(misc, struct keystone_rproc, misc);

	mutex_lock(&ksproc->mlock);
	ksproc->open_count++;
	mutex_unlock(&ksproc->mlock);

	return 0;
}

static int keystone_rproc_release(struct inode *inode, struct file *filp)
{
	struct miscdevice *misc = filp->private_data;
	struct keystone_rproc *ksproc =
		container_of(misc, struct keystone_rproc, misc);
	struct rproc *rproc = ksproc->rproc;

	mutex_lock(&ksproc->mlock);

	if ((WARN_ON(ksproc->open_count == 0)))
		goto end;

	if (--ksproc->open_count > 0)
		goto end;

	if (rproc->state != RPROC_OFFLINE) {
		rproc_shutdown(rproc);
		WARN_ON(rproc->state != RPROC_OFFLINE);
	}

	keystone_rproc_reset_rsc_table(ksproc);

end:
	mutex_unlock(&ksproc->mlock);
	return 0;
}

/*
 * File operations exposed through a miscdevice for supporting
 * the userspace loader/boot mechanism.
 */
static const struct file_operations keystone_rproc_fops = {
	.owner		= THIS_MODULE,
	.unlocked_ioctl	= keystone_rproc_ioctl,
	.mmap		= keystone_rproc_mmap,
	.open		= keystone_rproc_open,
	.release	= keystone_rproc_release,
};

/*
 * Used only with userspace loader/boot mechanism, the parsing of the firmware
 * is done in userspace, and a copy of the resource table is added for the
 * kernel-level access through an ioctl. Create the remoteproc cached table
 * using this resource table and configure the table pointer and table size
 * accordingly to allow the remoteproc core to process the resource table for
 * creating the vrings and traces.
 */
static int keystone_rproc_load_rsc_table(struct rproc *rproc,
					 const struct firmware *fw)
{
	struct keystone_rproc *ksproc = rproc->priv;

	rproc->cached_table = kmemdup(ksproc->rsc_table, ksproc->rsc_table_size,
				      GFP_KERNEL);
	if (!rproc->cached_table)
		return -ENOMEM;

	rproc->table_ptr = rproc->cached_table;
	rproc->table_sz = ksproc->rsc_table_size;

	return 0;
}

/*
 * Used only with userspace loader/boot mechanism, the device address of the
 * loaded resource table is published to the kernel-level through an ioctl
 * at which point the equivalent kernel virtual pointer is stored in a local
 * variable in the keystone_rproc device structure. Return this kernel pointer
 * to the remoteproc core for runtime publishing/modification of the resource
 * table entries.
 *
 * NOTE: Only loaded resource tables in the DSP internal memories is supported
 *       at present.
 */
static struct resource_table *
keystone_rproc_find_loaded_rsc_table(struct rproc *rproc,
				     const struct firmware *fw)
{
	struct keystone_rproc *ksproc = rproc->priv;

	return ksproc->loaded_rsc_table;
}

/*
 * Used only with userspace loader/boot mechanism, the boot address
 * is published to the kernel-level through an ioctl call and is
 * stored in a local variable in the keystone_rproc device structure.
 * Return this address to the remoteproc core through the .get_boot_addr()
 * remoteproc firmware ops
 */
static u32 keystone_rproc_get_boot_addr(struct rproc *rproc,
					const struct firmware *fw)
{
	struct keystone_rproc *ksproc = rproc->priv;

	return ksproc->boot_addr;
}

/*
 * Process the remoteproc exceptions
 *
 * The exception reporting on Keystone DSP remote processors is very simple
 * compared to the equivalent processors on the OMAP family, it is notified
 * through a software-designed specific interrupt source in the IPC interrupt
 * generation register.
 *
 * This function just invokes the rproc_report_crash to report the exception
 * to the remoteproc driver core, to trigger a recovery. This is the case
 * only when using in-kernel remoteproc core loader/boot mechanism, and is
 * handled through an UIO interrupt otherwise.
 */
static irqreturn_t keystone_rproc_exception_interrupt(int irq, void *dev_id)
{
	struct keystone_rproc *ksproc = dev_id;

	rproc_report_crash(ksproc->rproc, RPROC_FATAL_ERROR);

	return IRQ_HANDLED;
}

/*
 * Main virtqueue message workqueue function
 *
 * This function is executed upon scheduling of the keystone remoteproc
 * driver's workqueue. The workqueue is scheduled by the vring ISR handler.
 *
 * There is no payload message indicating the virtqueue index as is the
 * case with mailbox-based implementations on OMAP family. As such, this
 * handler processes both the Tx and Rx virtqueue indices on every invocation.
 * The rproc_vq_interrupt function can detect if there are new unprocessed
 * messages or not (returns IRQ_NONE vs IRQ_HANDLED), but there is no need
 * to check for these return values. The index 0 triggering will process all
 * pending Rx buffers, and the index 1 triggering will process all newly
 * available Tx buffers and will wakeup any potentially blocked senders.
 *
 * NOTE:
 * 1. A payload could be added by using some of the source bits in the
 *    IPC interrupt generation registers, but this would need additional
 *    changes to the overall IPC stack, and currently there are no benefits
 *    of adapting that approach.
 * 2. The current logic is based on an inherent design assumption of supporting
 *    only 2 vrings, but this can be changed if needed.
 */
static void handle_event(struct work_struct *work)
{
	struct keystone_rproc *ksproc =
		container_of(work, struct keystone_rproc, workqueue);

	rproc_vq_interrupt(ksproc->rproc, 0);
	rproc_vq_interrupt(ksproc->rproc, 1);
}

/*
 * Interrupt handler for processing vring kicks from remote processor
 */
static irqreturn_t keystone_rproc_vring_interrupt(int irq, void *dev_id)
{
	struct keystone_rproc *ksproc = dev_id;

	schedule_work(&ksproc->workqueue);

	return IRQ_HANDLED;
}

/*
 * Power up the DSP remote processor.
 *
 * This function will be invoked only after the firmware for this rproc
 * was loaded, parsed successfully, and all of its resource requirements
 * were met. The function skips releasing the processor from reset and
 * registering for the exception interrupt if using the userspace controlled
 * load/boot mechanism. The processor will be started through an ioctl when
 * controlled from userspace, but the virtio interrupt still is handled at
 * the kernel layer.
 */
static int keystone_rproc_start(struct rproc *rproc)
{
	struct keystone_rproc *ksproc = rproc->priv;
	int ret;

	INIT_WORK(&ksproc->workqueue, handle_event);

	ret = request_irq(ksproc->irq_ring, keystone_rproc_vring_interrupt, 0,
			  dev_name(ksproc->dev), ksproc);
	if (ret) {
		dev_err(ksproc->dev, "failed to enable vring interrupt, ret = %d\n",
			ret);
		goto out;
	}

	if (!ksproc->use_userspace_loader) {
		ret = request_irq(ksproc->irq_fault,
				  keystone_rproc_exception_interrupt, 0,
				  dev_name(ksproc->dev), ksproc);
		if (ret) {
			dev_err(ksproc->dev, "failed to enable exception interrupt, ret = %d\n",
				ret);
			goto free_vring_irq;
		}
	}

	ret = keystone_rproc_dsp_boot(ksproc, rproc->bootaddr);
	if (ret)
		goto free_exc_irq;

	return 0;

free_exc_irq:
	free_irq(ksproc->irq_fault, ksproc);
free_vring_irq:
	free_irq(ksproc->irq_ring, ksproc);
	flush_work(&ksproc->workqueue);
out:
	return ret;
}

/*
 * Stop the DSP remote processor.
 *
 * This function puts the DSP processor into reset, and finishes processing
 * of any pending messages. The reset procedure is completed only if using
 * kernel-mode remoteproc loading/booting mechanism, it is handled outside
 * if using userspace load/boot mechanism either through an ioctl, or when
 * the handle to the device is closed without triggering a reset.
 */
static int keystone_rproc_stop(struct rproc *rproc)
{
	struct keystone_rproc *ksproc = rproc->priv;

	if (!ksproc->use_userspace_loader) {
		keystone_rproc_dsp_reset(ksproc);
		free_irq(ksproc->irq_fault, ksproc);
	}

	free_irq(ksproc->irq_ring, ksproc);
	flush_work(&ksproc->workqueue);
	return 0;
}

/*
 * Kick the remote processor to notify about pending unprocessed messages.
 * The vqid usage is not used and is inconsequential, as the kick is performed
 * through a simulated GPIO (an bit in an IPC interrupt-triggering register),
 * the remote processor is expected to process both its Tx and Rx virtqueues.
 */
static void keystone_rproc_kick(struct rproc *rproc, int vqid)
{
	struct keystone_rproc *ksproc = rproc->priv;

	if (WARN_ON(ksproc->kick_gpio < 0))
		return;

	gpio_set_value(ksproc->kick_gpio, 1);
}

/*
 * Custom function to translate a DSP device address (internal RAMs only) to a
 * kernel virtual address.  The DSPs can access their RAMs at either an internal
 * address visible only from a DSP, or at the SoC-level bus address. Both these
 * addresses need to be looked through for translation. The translated addresses
 * can be used either by the remoteproc core for loading (when using kernel
 * remoteproc loader), or by any rpmsg bus drivers.
 */
static void *keystone_rproc_da_to_va(struct rproc *rproc, u64 da, int len,
				     u32 flags)
{
	struct keystone_rproc *ksproc = rproc->priv;
	void __iomem *va = NULL;
	phys_addr_t bus_addr;
	u32 dev_addr, offset;
	size_t size;
	int i;

	if (len <= 0)
		return NULL;

	for (i = 0; i < ksproc->num_mems; i++) {
		bus_addr = ksproc->mem[i].bus_addr;
		dev_addr = ksproc->mem[i].dev_addr;
		size = ksproc->mem[i].size;

		if (da < KEYSTONE_RPROC_LOCAL_ADDRESS_MASK) {
			/* handle DSP-view addresses */
			if ((da >= dev_addr) &&
			    ((da + len) <= (dev_addr + size))) {
				offset = da - dev_addr;
				va = ksproc->mem[i].cpu_addr + offset;
				break;
			}
		} else {
			/* handle SoC-view addresses */
			if ((da >= bus_addr) &&
			    (da + len) <= (bus_addr + size)) {
				offset = da - bus_addr;
				va = ksproc->mem[i].cpu_addr + offset;
				break;
			}
		}
	}

	return (__force void *)va;
}

static const struct rproc_ops keystone_rproc_ops = {
	.start		= keystone_rproc_start,
	.stop		= keystone_rproc_stop,
	.kick		= keystone_rproc_kick,
	.da_to_va	= keystone_rproc_da_to_va,
};

static int keystone_rproc_of_get_memories(struct platform_device *pdev,
					  struct keystone_rproc *ksproc)
{
	static const char * const mem_names[] = {"l2sram", "l1pram", "l1dram"};
	struct device *dev = &pdev->dev;
	struct resource *res;
	int num_mems = 0;
	int i;

	num_mems = ARRAY_SIZE(mem_names);
	ksproc->mem = devm_kcalloc(ksproc->dev, num_mems,
				   sizeof(*ksproc->mem), GFP_KERNEL);
	if (!ksproc->mem)
		return -ENOMEM;

	for (i = 0; i < num_mems; i++) {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   mem_names[i]);
		ksproc->mem[i].cpu_addr = devm_ioremap_resource(dev, res);
		if (IS_ERR(ksproc->mem[i].cpu_addr)) {
			dev_err(dev, "failed to parse and map %s memory\n",
				mem_names[i]);
			return PTR_ERR(ksproc->mem[i].cpu_addr);
		}
		ksproc->mem[i].bus_addr = res->start;
		ksproc->mem[i].dev_addr =
				res->start & KEYSTONE_RPROC_LOCAL_ADDRESS_MASK;
		ksproc->mem[i].size = resource_size(res);

		/* zero out memories to start in a pristine state */
		memset((__force void *)ksproc->mem[i].cpu_addr, 0,
		       ksproc->mem[i].size);
	}
	ksproc->num_mems = num_mems;

	return 0;
}

static int keystone_rproc_of_get_dev_syscon(struct platform_device *pdev,
					    struct keystone_rproc *ksproc)
{
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	int ret;

	if (!of_property_read_bool(np, "ti,syscon-dev")) {
		dev_err(dev, "ti,syscon-dev property is absent\n");
		return -EINVAL;
	}

	ksproc->dev_ctrl =
		syscon_regmap_lookup_by_phandle(np, "ti,syscon-dev");
	if (IS_ERR(ksproc->dev_ctrl)) {
		ret = PTR_ERR(ksproc->dev_ctrl);
		return ret;
	}

	if (of_property_read_u32_index(np, "ti,syscon-dev", 1,
				       &ksproc->boot_offset)) {
		dev_err(dev, "couldn't read the boot register offset\n");
		return -EINVAL;
	}

	return 0;
}

static int keystone_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct keystone_rproc *ksproc;
	struct miscdevice *misc;
	struct uio_info *uio;
	struct rproc *rproc;
	int dsp_id;
	char *uio_name = NULL;
	char *fw_name = NULL;
	char *template = "keystone-dsp%d-fw";
	int name_len = 0;
	int ret = 0;

	if (!np) {
		dev_err(dev, "only DT-based devices are supported\n");
		return -ENODEV;
	}

	dsp_id = of_alias_get_id(np, "rproc");
	if (dsp_id < 0) {
		dev_warn(dev, "device does not have an alias id\n");
		return dsp_id;
	}

	/* construct a name for uio devices - assuming a single digit alias */
	name_len = strlen("dsp%d");
	uio_name = devm_kzalloc(dev, name_len, GFP_KERNEL);
	if (!uio_name)
		return -ENOMEM;
	snprintf(uio_name, name_len, "dsp%d", dsp_id);

	/* construct a custom default fw name - subject to change in future */
	name_len = strlen(template); /* assuming a single digit alias */
	fw_name = devm_kzalloc(dev, name_len, GFP_KERNEL);
	if (!fw_name)
		return -ENOMEM;
	snprintf(fw_name, name_len, template, dsp_id);

	rproc = rproc_alloc(dev, dev_name(dev), &keystone_rproc_ops, fw_name,
			    sizeof(*ksproc));
	if (!rproc)
		return -ENOMEM;

	rproc->has_iommu = false;

	ksproc = rproc->priv;
	ksproc->rproc = rproc;
	ksproc->dev = dev;
	ksproc->use_userspace_loader = !use_rproc_core_loader;

	/*
	 * customize the remoteproc core config flags and ELF fw ops for
	 * userspace loader/boot mechanism
	 */
	if (ksproc->use_userspace_loader) {
		rproc->recovery_disabled = true;
		rproc->auto_boot = false;
		rproc->skip_firmware_request = 1;
		rproc->skip_load = 1;
		rproc->deny_sysfs_ops = true;

		rproc->ops->parse_fw = keystone_rproc_load_rsc_table;
		rproc->ops->find_loaded_rsc_table =
					keystone_rproc_find_loaded_rsc_table;
		rproc->ops->get_boot_addr = keystone_rproc_get_boot_addr;
		rproc->ops->sanity_check = NULL;
		rproc->ops->load = NULL;
	}

	mutex_init(&ksproc->mlock);
	spin_lock_init(&ksproc->lock);

	ret = keystone_rproc_of_get_dev_syscon(pdev, ksproc);
	if (ret)
		goto free_rproc;

	ksproc->reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(ksproc->reset)) {
		ret = PTR_ERR(ksproc->reset);
		goto free_rproc;
	}

	/* enable clock for accessing DSP internal memories */
	pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		dev_err(dev, "failed to enable clock, status = %d\n", ret);
		pm_runtime_put_noidle(dev);
		goto disable_rpm;
	}

	ret = keystone_rproc_of_get_memories(pdev, ksproc);
	if (ret)
		goto disable_clk;

	ksproc->irq_ring = platform_get_irq_byname(pdev, "vring");
	if (ksproc->irq_ring < 0) {
		ret = ksproc->irq_ring;
		dev_err(dev, "failed to get vring interrupt, status = %d\n",
			ret);
		goto disable_clk;
	}

	ksproc->irq_fault = platform_get_irq_byname(pdev, "exception");
	if (ksproc->irq_fault < 0) {
		ret = ksproc->irq_fault;
		dev_err(dev, "failed to get exception interrupt, status = %d\n",
			ret);
		goto disable_clk;
	}

	ksproc->kick_gpio = of_get_named_gpio_flags(np, "kick-gpios", 0, NULL);
	if (ksproc->kick_gpio < 0) {
		ret = ksproc->kick_gpio;
		dev_err(dev, "failed to get gpio for virtio kicks, status = %d\n",
			ret);
		goto disable_clk;
	}

	if (of_reserved_mem_device_init(dev))
		dev_warn(dev, "device does not have specific CMA pool\n");

	/* ensure the DSP is in reset before loading firmware */
	ret = reset_control_status(ksproc->reset);
	if (ret < 0) {
		dev_err(dev, "failed to get reset status, status = %d\n", ret);
		goto release_mem;
	} else if (ret == 0) {
		WARN(1, "device is not in reset\n");
		keystone_rproc_dsp_reset(ksproc);
	}

	ret = rproc_add(rproc);
	if (ret) {
		dev_err(dev, "failed to add register device with remoteproc core, status = %d\n",
			ret);
		goto release_mem;
	}

	platform_set_drvdata(pdev, ksproc);

	if (ksproc->use_userspace_loader) {
		uio = &ksproc->uio;
		uio->name = uio_name;
		uio->version = DRIVER_UIO_VERSION;
		uio->irq = ksproc->irq_fault;
		uio->priv = ksproc;
		uio->handler = keystone_rproc_uio_handler;
		uio->irqcontrol	= keystone_rproc_uio_irqcontrol;
		ret = uio_register_device(dev, uio);
		if (ret) {
			dev_err(dev, "failed to register uio device, status = %d\n",
				ret);
			goto del_rproc;
		}
		dev_dbg(dev, "registered uio device %s\n", uio->name);

		misc = &ksproc->misc;
		misc->minor = MISC_DYNAMIC_MINOR;
		misc->name = uio->name;
		misc->fops = &keystone_rproc_fops;
		misc->parent = dev;
		ret = misc_register(misc);
		if (ret) {
			dev_err(dev, "failed to register misc device, status = %d\n",
				ret);
			goto unregister_uio;
		}

		ret = keystone_rproc_mem_add_attrs(ksproc);
		if (ret) {
			dev_err(ksproc->dev, "error creating sysfs files (%d)\n",
				ret);
			goto unregister_misc;
		}

		dev_dbg(dev, "registered misc device %s\n", misc->name);
	}

	return 0;

unregister_misc:
	misc_deregister(misc);
unregister_uio:
	uio_unregister_device(uio);
del_rproc:
	rproc_del(rproc);
release_mem:
	of_reserved_mem_device_release(dev);
disable_clk:
	pm_runtime_put_sync(dev);
disable_rpm:
	pm_runtime_disable(dev);
free_rproc:
	rproc_free(rproc);
	return ret;
}

static int keystone_rproc_remove(struct platform_device *pdev)
{
	struct keystone_rproc *ksproc = platform_get_drvdata(pdev);

	if (ksproc->use_userspace_loader) {
		keystone_rproc_mem_del_attrs(ksproc);
		misc_deregister(&ksproc->misc);
		uio_unregister_device(&ksproc->uio);
	}
	rproc_del(ksproc->rproc);
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	rproc_free(ksproc->rproc);
	of_reserved_mem_device_release(&pdev->dev);

	return 0;
}

static const struct of_device_id keystone_rproc_of_match[] = {
	{ .compatible = "ti,k2hk-dsp", },
	{ .compatible = "ti,k2l-dsp", },
	{ .compatible = "ti,k2e-dsp", },
	{ .compatible = "ti,k2g-dsp", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, keystone_rproc_of_match);

static struct platform_driver keystone_rproc_driver = {
	.probe	= keystone_rproc_probe,
	.remove	= keystone_rproc_remove,
	.driver	= {
		.name = "keystone-rproc",
		.of_match_table = keystone_rproc_of_match,
	},
};

static int __init keystone_rproc_init(void)
{
	keystone_rproc_driver.driver.suppress_bind_attrs =
		!use_rproc_core_loader;

	return platform_driver_register(&keystone_rproc_driver);
}
module_init(keystone_rproc_init);

static void __exit keystone_rproc_exit(void)
{
	platform_driver_unregister(&keystone_rproc_driver);
}
module_exit(keystone_rproc_exit);

MODULE_AUTHOR("Suman Anna <s-anna@ti.com>");
MODULE_AUTHOR("Sam Nelson <sam.nelson@ti.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("TI Keystone DSP Remoteproc driver");
