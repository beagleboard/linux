// SPDX-License-Identifier: GPL-2.0-or-later

#define pr_fmt(fmt) "virtio-khv-mmio: " fmt

#include <linux/acpi.h>
#include <linux/dma-mapping.h>
#include <linux/highmem.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/virtio.h>
#include <linux/virtio_config.h>
#include <uapi/linux/virtio_mmio.h>
#include <linux/mmio_khv.h>
#include <linux/virtio_ring.h>
#include <linux/virtio_ids.h>

static volatile unsigned char __iomem *notify_guest_reg;
static volatile unsigned char __iomem *notify_host_reg;

/* The alignment to use between consumer and producer parts of vring.
 * Currently hardcoded to the page size. */
#define VIRTIO_MMIO_VRING_ALIGN		PAGE_SIZE



#define to_virtio_khv_mmio_device(_plat_dev) \
	container_of(_plat_dev, struct virtio_khv_mmio_device, vdev)

struct virtio_khv_mmio_device {
	struct virtio_device vdev;
	struct platform_device *pdev;

	u64 base;
	unsigned long version;

	/* a list of queues so we can dispatch IRQs */
	spinlock_t lock;
	struct list_head virtqueues;
};

struct virtio_khv_mmio_vq_info {
	/* the actual virtqueue */
	struct virtqueue *vq;

	/* the list node for the virtqueues list */
	struct list_head node;
};

/* Configuration interface */

static u64 vm_get_features(struct virtio_device *vdev)
{
	struct virtio_khv_mmio_device *vm_dev = to_virtio_khv_mmio_device(vdev);
	u64 features;

	writex(1, vm_dev->base + VIRTIO_MMIO_DEVICE_FEATURES_SEL, WIDTH_32);
	features = readx(vm_dev->base + VIRTIO_MMIO_DEVICE_FEATURES, WIDTH_32);
	features <<= 32;

	writex(0, vm_dev->base + VIRTIO_MMIO_DEVICE_FEATURES_SEL, WIDTH_32);
	features |= readx(vm_dev->base + VIRTIO_MMIO_DEVICE_FEATURES, WIDTH_32);

	return features;
}

static int vm_finalize_features(struct virtio_device *vdev)
{
	struct virtio_khv_mmio_device *vm_dev = to_virtio_khv_mmio_device(vdev);

	/* Give virtio_ring a chance to accept features. */
	vring_transport_features(vdev);

	/* Make sure there is are no mixed devices */
	if (vm_dev->version == 2 &&
			!__virtio_test_bit(vdev, VIRTIO_F_VERSION_1)) {
		dev_err(&vdev->dev, "New virtio-mmio devices (version 2) must provide VIRTIO_F_VERSION_1 feature!\n");
		return -EINVAL;
	}

	writex(1, vm_dev->base + VIRTIO_MMIO_DRIVER_FEATURES_SEL, WIDTH_32);
	writex((u32)(vdev->features >> 32),
			vm_dev->base + VIRTIO_MMIO_DRIVER_FEATURES, WIDTH_32);

	writex(0, vm_dev->base + VIRTIO_MMIO_DRIVER_FEATURES_SEL, WIDTH_32);
	writex((u32)vdev->features,
			vm_dev->base + VIRTIO_MMIO_DRIVER_FEATURES, WIDTH_32);

	return 0;
}

static void vm_get(struct virtio_device *vdev, unsigned offset,
		   void *buf, unsigned len)
{
	struct virtio_khv_mmio_device *vm_dev = to_virtio_khv_mmio_device(vdev);
	u64 base = vm_dev->base + VIRTIO_MMIO_CONFIG;
	u8 b;
	__le16 w;
	__le32 l;

	if (vm_dev->version == 1) {
		u8 *ptr = buf;
		int i;

		for (i = 0; i < len; i++)
			ptr[i] = readx(base + offset + i, WIDTH_8);
		return;
	}

	switch (len) {
	case 1:
		b = readx(base + offset, WIDTH_8);
		memcpy(buf, &b, sizeof b);
		break;
	case 2:
		w = cpu_to_le16(readx(base + offset, WIDTH_16));
		memcpy(buf, &w, sizeof w);
		break;
	case 4:
		l = cpu_to_le32(readx(base + offset, WIDTH_32));
		memcpy(buf, &l, sizeof l);
		break;
	case 8:
		l = cpu_to_le32(readx(base + offset, WIDTH_32));
		memcpy(buf, &l, sizeof l);
		l = cpu_to_le32(readx(base + offset + sizeof l, WIDTH_32));
		memcpy(buf + sizeof l, &l, sizeof l);
		break;
	default:
		BUG();
	}
}

static void vm_set(struct virtio_device *vdev, unsigned offset,
		   const void *buf, unsigned len)
{
	struct virtio_khv_mmio_device *vm_dev = to_virtio_khv_mmio_device(vdev);
	u64 base = vm_dev->base + VIRTIO_MMIO_CONFIG;
	u8 b;
	__le16 w;
	__le32 l;

	if (vm_dev->version == 1) {
		const u8 *ptr = buf;
		int i;

		for (i = 0; i < len; i++)
			writex(ptr[i], base + offset + i, WIDTH_8);

		return;
	}

	switch (len) {
	case 1:
		memcpy(&b, buf, sizeof b);
		writex(b, base + offset, WIDTH_8);
		break;
	case 2:
		memcpy(&w, buf, sizeof w);
		writex(le16_to_cpu(w), base + offset, WIDTH_16);
		break;
	case 4:
		memcpy(&l, buf, sizeof l);
		writex(le32_to_cpu(l), base + offset, WIDTH_32);
		break;
	case 8:
		memcpy(&l, buf, sizeof l);
		writex(le32_to_cpu(l), base + offset, WIDTH_32);
		memcpy(&l, buf + sizeof l, sizeof l);
		writex(le32_to_cpu(l), base + offset + sizeof l, WIDTH_32);
		break;
	default:
		BUG();
	}
}

static u32 vm_generation(struct virtio_device *vdev)
{
	struct virtio_khv_mmio_device *vm_dev = to_virtio_khv_mmio_device(vdev);

	if (vm_dev->version == 1)
		return 0;
	else
		return readx(vm_dev->base + VIRTIO_MMIO_CONFIG_GENERATION, WIDTH_32);
}

static u8 vm_get_status(struct virtio_device *vdev)
{
	struct virtio_khv_mmio_device *vm_dev = to_virtio_khv_mmio_device(vdev);

	return readx(vm_dev->base + VIRTIO_MMIO_STATUS, WIDTH_32) & 0xff;
}

static void vm_set_status(struct virtio_device *vdev, u8 status)
{
	struct virtio_khv_mmio_device *vm_dev = to_virtio_khv_mmio_device(vdev);

	/* We should never be setting status to 0. */
	BUG_ON(status == 0);

	writex(status, vm_dev->base + VIRTIO_MMIO_STATUS, WIDTH_32);
}

static void vm_reset(struct virtio_device *vdev)
{
	struct virtio_khv_mmio_device *vm_dev = to_virtio_khv_mmio_device(vdev);

	/* 0 status means a reset. */
	writex(0, vm_dev->base + VIRTIO_MMIO_STATUS, WIDTH_32);
}



/* Transport interface */

/* the notify function used when creating a virt queue */
static bool vm_notify(struct virtqueue *vq)
{
	struct virtio_khv_mmio_device *vm_dev = to_virtio_khv_mmio_device(vq->vdev);

	writex(vq->index, vm_dev->base + VIRTIO_MMIO_QUEUE_NOTIFY, WIDTH_32);
	return true;
}

/* Notify all virtqueues on an interrupt. */
static irqreturn_t vm_interrupt(int irq, void *opaque)
{
	struct virtio_khv_mmio_device *vm_dev = opaque;
	struct virtio_khv_mmio_vq_info *info;
	unsigned long flags;
	irqreturn_t ret = IRQ_NONE;

#ifdef CONFIG_SOC_INT_SRC7
	writel(0, notify_guest_reg);
#else
	/* clear interrupt */
	writel(1, notify_guest_reg + 0x4);
#endif

	spin_lock_irqsave(&vm_dev->lock, flags);
	list_for_each_entry(info, &vm_dev->virtqueues, node)
		ret |= vring_interrupt(irq, info->vq);
	spin_unlock_irqrestore(&vm_dev->lock, flags);

	return ret;
}

static void vm_del_vq(struct virtqueue *vq)
{
	struct virtio_khv_mmio_device *vm_dev = to_virtio_khv_mmio_device(vq->vdev);
	struct virtio_khv_mmio_vq_info *info = vq->priv;
	unsigned long flags;
	unsigned int index = vq->index;

	spin_lock_irqsave(&vm_dev->lock, flags);
	list_del(&info->node);
	spin_unlock_irqrestore(&vm_dev->lock, flags);

	/* Select and deactivate the queue */
	writex(index, vm_dev->base + VIRTIO_MMIO_QUEUE_SEL, WIDTH_32);
	if (vm_dev->version == 1) {
		writex(0, vm_dev->base + VIRTIO_MMIO_QUEUE_PFN, WIDTH_32);
	} else {
		writex(0, vm_dev->base + VIRTIO_MMIO_QUEUE_READY, WIDTH_32);
		WARN_ON(readx(vm_dev->base + VIRTIO_MMIO_QUEUE_READY, WIDTH_32));
	}

	vring_del_virtqueue(vq);

	kfree(info);
}

static void vm_del_vqs(struct virtio_device *vdev)
{
	struct virtio_khv_mmio_device *vm_dev = to_virtio_khv_mmio_device(vdev);
	struct virtqueue *vq, *n;

	list_for_each_entry_safe(vq, n, &vdev->vqs, list)
		vm_del_vq(vq);

	free_irq(platform_get_irq(vm_dev->pdev, 0), vm_dev);
}

static struct virtqueue *vm_setup_vq(struct virtio_device *vdev, unsigned index,
				  void (*callback)(struct virtqueue *vq),
				  const char *name, bool ctx)
{
	struct virtio_khv_mmio_device *vm_dev = to_virtio_khv_mmio_device(vdev);
	struct virtio_khv_mmio_vq_info *info;
	struct virtqueue *vq;
	unsigned long flags;
	unsigned int num;
	int err;

	if (!name)
		return NULL;

	/* Select the queue we're interested in */
	writex(index, vm_dev->base + VIRTIO_MMIO_QUEUE_SEL, WIDTH_32);

	/* Queue shouldn't already be set up. */
	if (readx(vm_dev->base + (vm_dev->version == 1 ?
			VIRTIO_MMIO_QUEUE_PFN : VIRTIO_MMIO_QUEUE_READY), WIDTH_32)) {
		err = -ENOENT;
		goto error_available;
	}

	/* Allocate and fill out our active queue description */
	info = kmalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		err = -ENOMEM;
		goto error_kmalloc;
	}

	num = readx(vm_dev->base + VIRTIO_MMIO_QUEUE_NUM_MAX, WIDTH_32);
	if (num == 0) {
		err = -ENOENT;
		goto error_new_virtqueue;
	}

	/* Create the vring */
	vq = vring_create_virtqueue(index, num, VIRTIO_MMIO_VRING_ALIGN, vdev,
				 true, true, ctx, vm_notify, callback, name);
	if (!vq) {
		err = -ENOMEM;
		goto error_new_virtqueue;
	}

	/* Activate the queue */
	writex(virtqueue_get_vring_size(vq), vm_dev->base + VIRTIO_MMIO_QUEUE_NUM, WIDTH_32);
	if (vm_dev->version == 1) {
		u64 q_pfn = virtqueue_get_desc_addr(vq) >> PAGE_SHIFT;

		/*
		 * virtio-mmio v1 uses a 32bit QUEUE PFN. If we have something
		 * that doesn't fit in 32bit, fail the setup rather than
		 * pretending to be successful.
		 */
		if (q_pfn >> 32) {
			dev_err(&vdev->dev,
				"platform bug: legacy virtio-mmio must not be used with RAM above 0x%llxGB\n",
				0x1ULL << (32 + PAGE_SHIFT - 30));
			err = -E2BIG;
			goto error_bad_pfn;
		}

		writex(PAGE_SIZE, vm_dev->base + VIRTIO_MMIO_QUEUE_ALIGN, WIDTH_32);
		writex(q_pfn, vm_dev->base + VIRTIO_MMIO_QUEUE_PFN, WIDTH_32);
	} else {
		u64 addr;

		addr = virtqueue_get_desc_addr(vq);
		writex((u32)addr, vm_dev->base + VIRTIO_MMIO_QUEUE_DESC_LOW, WIDTH_32);
		writex((u32)(addr >> 32),
				vm_dev->base + VIRTIO_MMIO_QUEUE_DESC_HIGH, WIDTH_32);

		addr = virtqueue_get_avail_addr(vq);
		writex((u32)addr, vm_dev->base + VIRTIO_MMIO_QUEUE_AVAIL_LOW, WIDTH_32);
		writex((u32)(addr >> 32),
				vm_dev->base + VIRTIO_MMIO_QUEUE_AVAIL_HIGH, WIDTH_32);

		addr = virtqueue_get_used_addr(vq);
		writex((u32)addr, vm_dev->base + VIRTIO_MMIO_QUEUE_USED_LOW, WIDTH_32);
		writex((u32)(addr >> 32),
				vm_dev->base + VIRTIO_MMIO_QUEUE_USED_HIGH, WIDTH_32);

		writex(1, vm_dev->base + VIRTIO_MMIO_QUEUE_READY, WIDTH_32);
	}

	vq->priv = info;
	info->vq = vq;

	spin_lock_irqsave(&vm_dev->lock, flags);
	list_add(&info->node, &vm_dev->virtqueues);
	spin_unlock_irqrestore(&vm_dev->lock, flags);

	return vq;

error_bad_pfn:
	vring_del_virtqueue(vq);
error_new_virtqueue:
	if (vm_dev->version == 1) {
		writex(0, vm_dev->base + VIRTIO_MMIO_QUEUE_PFN, WIDTH_32);
	} else {
		writex(0, vm_dev->base + VIRTIO_MMIO_QUEUE_READY, WIDTH_32);
		WARN_ON(readx(vm_dev->base + VIRTIO_MMIO_QUEUE_READY, WIDTH_32));
	}
	kfree(info);
error_kmalloc:
error_available:
	return ERR_PTR(err);
}

static int vm_find_vqs(struct virtio_device *vdev, unsigned nvqs,
		       struct virtqueue *vqs[],
		       vq_callback_t *callbacks[],
		       const char * const names[],
		       const bool *ctx,
		       struct irq_affinity *desc)
{
	struct virtio_khv_mmio_device *vm_dev = to_virtio_khv_mmio_device(vdev);
	int irq = platform_get_irq(vm_dev->pdev, 0);
	int i, err, queue_idx = 0;

	if (irq < 0)
		return irq;

	err = request_irq(irq, vm_interrupt, IRQF_SHARED,
			dev_name(&vdev->dev), vm_dev);
	if (err)
		return err;

	for (i = 0; i < nvqs; ++i) {
		if (!names[i]) {
			vqs[i] = NULL;
			continue;
		}

		vqs[i] = vm_setup_vq(vdev, queue_idx++, callbacks[i], names[i],
				     ctx ? ctx[i] : false);
		if (IS_ERR(vqs[i])) {
			vm_del_vqs(vdev);
			return PTR_ERR(vqs[i]);
		}
	}

	return 0;
}

static const char *vm_bus_name(struct virtio_device *vdev)
{
	struct virtio_khv_mmio_device *vm_dev = to_virtio_khv_mmio_device(vdev);

	return vm_dev->pdev->name;
}

static bool vm_get_shm_region(struct virtio_device *vdev,
			      struct virtio_shm_region *region, u8 id)
{
	struct virtio_khv_mmio_device *vm_dev = to_virtio_khv_mmio_device(vdev);
	u64 len, addr;

	/* Select the region we're interested in */
	writex(id, vm_dev->base + VIRTIO_MMIO_SHM_SEL, WIDTH_32);

	/* Read the region size */
	len = (u64) readx(vm_dev->base + VIRTIO_MMIO_SHM_LEN_LOW, WIDTH_32);
	len |= (u64) readx(vm_dev->base + VIRTIO_MMIO_SHM_LEN_HIGH, WIDTH_32) << 32;

	region->len = len;

	/* Check if region length is -1. If that's the case, the shared memory
	 * region does not exist and there is no need to proceed further.
	 */
	if (len == ~(u64)0)
		return false;

	/* Read the region base address */
	addr = (u64) readx(vm_dev->base + VIRTIO_MMIO_SHM_BASE_LOW, WIDTH_32);
	addr |= (u64) readx(vm_dev->base + VIRTIO_MMIO_SHM_BASE_HIGH, WIDTH_32) << 32;

	region->addr = addr;

	return true;
}

static const struct virtio_config_ops virtio_khv_mmio_config_ops = {
	.get		= vm_get,
	.set		= vm_set,
	.generation	= vm_generation,
	.get_status	= vm_get_status,
	.set_status	= vm_set_status,
	.reset		= vm_reset,
	.find_vqs	= vm_find_vqs,
	.del_vqs	= vm_del_vqs,
	.get_features	= vm_get_features,
	.finalize_features = vm_finalize_features,
	.bus_name	= vm_bus_name,
	.get_shm_region = vm_get_shm_region,
};


static void virtio_khv_mmio_release_dev(struct device *_d)
{
	struct virtio_device *vdev =
			container_of(_d, struct virtio_device, dev);
	struct virtio_khv_mmio_device *vm_dev =
			container_of(vdev, struct virtio_khv_mmio_device, vdev);
	struct platform_device *pdev = vm_dev->pdev;

	devm_kfree(&pdev->dev, vm_dev);
}

/* Platform device */


static int virtio_khv_mmio_probe(struct platform_device *pdev)
{
	struct virtio_khv_mmio_device *vm_dev;
	unsigned long magic;
	int rc;

	vm_dev = devm_kzalloc(&pdev->dev, sizeof(*vm_dev), GFP_KERNEL);
	if (!vm_dev)
		return -ENOMEM;

	vm_dev->vdev.dev.parent = &pdev->dev;
	vm_dev->vdev.dev.release = virtio_khv_mmio_release_dev;
	vm_dev->vdev.config = &virtio_khv_mmio_config_ops;
	vm_dev->pdev = pdev;
	INIT_LIST_HEAD(&vm_dev->virtqueues);
	spin_lock_init(&vm_dev->lock);

#ifdef CONFIG_SOC_INT_SRC7
	notify_guest_reg = ioremap(0xFFEF018094, 4);
	notify_host_reg = ioremap(0xFFFF019094, 4);
#else
	notify_guest_reg = ioremap(0xffefc50000, 0x100);
	notify_host_reg  = ioremap(0xffffc3b000, 0x100);
	writel(1, notify_host_reg + 0xc);
	writel(1, notify_host_reg + 0x4);
#endif

	rc = of_property_read_u64((&pdev->dev)->of_node, "reg", &vm_dev->base);
	if (rc < 0)
		return -EFAULT;

	/* Check magic value */
	magic = readx(vm_dev->base + VIRTIO_MMIO_MAGIC_VALUE, WIDTH_32);
	if (magic != ('v' | 'i' << 8 | 'r' << 16 | 't' << 24)) {
		dev_warn(&pdev->dev, "Wrong magic value 0x%08lx!\n", magic);
		return -ENODEV;
	}

	/* Check device version */
	vm_dev->version = readx(vm_dev->base + VIRTIO_MMIO_VERSION, WIDTH_32);
	if (vm_dev->version < 1 || vm_dev->version > 2) {
		dev_err(&pdev->dev, "Version %ld not supported!\n",
				vm_dev->version);
		return -ENXIO;
	}

	vm_dev->vdev.id.device = readx(vm_dev->base + VIRTIO_MMIO_DEVICE_ID, WIDTH_32);
	if (vm_dev->vdev.id.device == 0) {
		/*
		 * virtio-mmio device with an ID 0 is a (dummy) placeholder
		 * with no function. End probing now with no error reported.
		 */
		return -ENODEV;
	}
	vm_dev->vdev.id.vendor = readx(vm_dev->base + VIRTIO_MMIO_VENDOR_ID, WIDTH_32);

	if (vm_dev->version == 1) {
		writex(PAGE_SIZE, vm_dev->base + VIRTIO_MMIO_GUEST_PAGE_SIZE, WIDTH_32);

		rc = dma_set_mask(&pdev->dev, DMA_BIT_MASK(64));
		/*
		 * In the legacy case, ensure our coherently-allocated virtio
		 * ring will be at an address expressable as a 32-bit PFN.
		 */
		if (!rc)
			dma_set_coherent_mask(&pdev->dev,
					      DMA_BIT_MASK(32 + PAGE_SHIFT));
	} else {
		rc = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
	}
	if (rc)
		rc = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (rc)
		dev_warn(&pdev->dev, "Failed to enable 64-bit or 32-bit DMA.  Trying to continue, but this might not work.\n");

	platform_set_drvdata(pdev, vm_dev);

	rc = register_virtio_device(&vm_dev->vdev);
	if (rc)
		put_device(&vm_dev->vdev.dev);

	return rc;
}

static int virtio_khv_mmio_remove(struct platform_device *pdev)
{
	struct virtio_khv_mmio_device *vm_dev = platform_get_drvdata(pdev);
	unregister_virtio_device(&vm_dev->vdev);

	return 0;
}



/* Devices list parameter */

#if defined(CONFIG_VIRTIO_MMIO_CMDLINE_DEVICES)

static struct device vm_cmdline_parent = {
	.init_name = "virtio-mmio-cmdline",
};

static int vm_cmdline_parent_registered;
static int vm_cmdline_id;

static int vm_cmdline_set(const char *device,
		const struct kernel_param *kp)
{
	int err;
	struct resource resources[2] = {};
	char *str;
	long long int base, size;
	unsigned int irq;
	int processed, consumed = 0;
	struct platform_device *pdev;

	/* Consume "size" part of the command line parameter */
	size = memparse(device, &str);

	/* Get "@<base>:<irq>[:<id>]" chunks */
	processed = sscanf(str, "@%lli:%u%n:%d%n",
			&base, &irq, &consumed,
			&vm_cmdline_id, &consumed);

	/*
	 * sscanf() must process at least 2 chunks; also there
	 * must be no extra characters after the last chunk, so
	 * str[consumed] must be '\0'
	 */
	if (processed < 2 || str[consumed] || irq == 0)
		return -EINVAL;

	resources[0].flags = IORESOURCE_MEM;
	resources[0].start = base;
	resources[0].end = base + size - 1;

	resources[1].flags = IORESOURCE_IRQ;
	resources[1].start = resources[1].end = irq;

	if (!vm_cmdline_parent_registered) {
		err = device_register(&vm_cmdline_parent);
		if (err) {
			pr_err("Failed to register parent device!\n");
			return err;
		}
		vm_cmdline_parent_registered = 1;
	}

	pr_info("Registering device virtio-mmio.%d at 0x%llx-0x%llx, IRQ %d.\n",
		       vm_cmdline_id,
		       (unsigned long long)resources[0].start,
		       (unsigned long long)resources[0].end,
		       (int)resources[1].start);

	pdev = platform_device_register_resndata(&vm_cmdline_parent,
			"virtio-mmio", vm_cmdline_id++,
			resources, ARRAY_SIZE(resources), NULL, 0);

	return PTR_ERR_OR_ZERO(pdev);
}

static int vm_cmdline_get_device(struct device *dev, void *data)
{
	char *buffer = data;
	unsigned int len = strlen(buffer);
	struct platform_device *pdev = to_platform_device(dev);

	snprintf(buffer + len, PAGE_SIZE - len, "0x%llx@0x%llx:%llu:%d\n",
			pdev->resource[0].end - pdev->resource[0].start + 1ULL,
			(unsigned long long)pdev->resource[0].start,
			(unsigned long long)pdev->resource[1].start,
			pdev->id);
	return 0;
}

static int vm_cmdline_get(char *buffer, const struct kernel_param *kp)
{
	buffer[0] = '\0';
	device_for_each_child(&vm_cmdline_parent, buffer,
			vm_cmdline_get_device);
	return strlen(buffer) + 1;
}

static const struct kernel_param_ops vm_cmdline_param_ops = {
	.set = vm_cmdline_set,
	.get = vm_cmdline_get,
};

device_param_cb(device, &vm_cmdline_param_ops, NULL, S_IRUSR);

static int vm_unregister_cmdline_device(struct device *dev,
		void *data)
{
	platform_device_unregister(to_platform_device(dev));

	return 0;
}

static void vm_unregister_cmdline_devices(void)
{
	if (vm_cmdline_parent_registered) {
		device_for_each_child(&vm_cmdline_parent, NULL,
				vm_unregister_cmdline_device);
		device_unregister(&vm_cmdline_parent);
		vm_cmdline_parent_registered = 0;
	}
}

#else

static void vm_unregister_cmdline_devices(void)
{
}

#endif

/* Platform driver */

static const struct of_device_id virtio_khv_mmio_match[] = {
	{ .compatible = "virtio,khv-mmio", },
	{},
};
MODULE_DEVICE_TABLE(of, virtio_khv_mmio_match);

#ifdef CONFIG_ACPI
static const struct acpi_device_id virtio_khv_mmio_acpi_match[] = {
	{ "LNRO0005", },
	{ }
};
MODULE_DEVICE_TABLE(acpi, virtio_khv_mmio_acpi_match);
#endif

static struct platform_driver virtio_khv_mmio_driver = {
	.probe		= virtio_khv_mmio_probe,
	.remove		= virtio_khv_mmio_remove,
	.driver		= {
		.name	= "virtio-khv-mmio",
		.of_match_table	= virtio_khv_mmio_match,
		.acpi_match_table = ACPI_PTR(virtio_khv_mmio_acpi_match),
	},
};

static int __init virtio_khv_mmio_init(void)
{
	return platform_driver_register(&virtio_khv_mmio_driver);
}

static void __exit virtio_khv_mmio_exit(void)
{
	platform_driver_unregister(&virtio_khv_mmio_driver);
	vm_unregister_cmdline_devices();
}

module_init(virtio_khv_mmio_init);
module_exit(virtio_khv_mmio_exit);

MODULE_AUTHOR("Xianting Tian <xianting.tian@linux.alibaba.com>");
MODULE_DESCRIPTION("Platform bus driver for khv memory mapped virtio devices");
MODULE_LICENSE("GPL");
