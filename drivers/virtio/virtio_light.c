// SPDX-License-Identifier: GPL-2.0-or-later
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
#include <uapi/linux/virtio_light.h>
#include <linux/virtio_ring.h>
#include <linux/delay.h>
#include <linux/of_irq.h>

/* The alignment to use between consumer and producer parts of vring.
 *  * Currently hardcoded to the page size. */
#define VIRTIO_LIGHT_VRING_ALIGN		PAGE_SIZE

#define to_virtio_light_device(_plat_dev) \
	container_of(_plat_dev, struct virtio_light_device, vdev)

struct virtio_light_device {
	struct virtio_device vdev;
	struct platform_device *pdev;

	void __iomem *base;
	unsigned long version;

	/* a list of queues so we can dispatch IRQs */
	spinlock_t lock;
	struct list_head virtqueues;
	volatile unsigned char __iomem *backend_intr_reg;
	volatile unsigned char __iomem *frontend_intr_reg;
	u32 enable_intr;
	u32 clear_intr;
};

struct virtio_light_vq_info {
	/* the actual virtqueue */
	struct virtqueue *vq;

	/* the list node for the virtqueues list */
	struct list_head node;
};

static u8 vm_get_status(struct virtio_device *vdev)
{
	struct virtio_light_device *vm_dev = to_virtio_light_device(vdev);

	return readl(vm_dev->base + VIRTIO_LIGHT_STATUS) & 0xff;
}

static void vm_set_status(struct virtio_device *vdev, u8 status)
{
	struct virtio_light_device *vm_dev = to_virtio_light_device(vdev);

	/* We should never be setting status to 0. */
	BUG_ON(status == 0);

	writel(status, vm_dev->base + VIRTIO_LIGHT_STATUS);
}

static void vm_reset(struct virtio_device *vdev)
{
	struct virtio_light_device *vm_dev = to_virtio_light_device(vdev);

	/* 0 status means a reset. */
	writel(0, vm_dev->base + VIRTIO_LIGHT_STATUS);
}

/* the notify function */
static bool vm_notify(struct virtqueue *vq)
{
	struct virtio_light_device *vm_dev = to_virtio_light_device(vq->vdev);

#ifdef CONFIG_SOC_INT_SRC7
	writel(1, vm_dev->backend_intr_reg);
#else
	/* Write the interrupt register to signal the other end */
	writel(1, vm_dev->backend_intr_reg + 0x10);
#endif

	return true;
}

/* Notify all virtqueues on an interrupt. */
static irqreturn_t vm_interrupt(int irq, void *opaque)
{
	unsigned long flags;
	irqreturn_t ret = IRQ_NONE;
	struct virtio_light_vq_info *info;
	struct virtio_light_device *vm_dev = opaque;

#ifdef CONFIG_SOC_INT_SRC7
	writel(0, vm_dev->frontend_intr_reg);
#else
	/* clear interrupt */
	writel(vm_dev->clear_intr, vm_dev->frontend_intr_reg + 0x4);
#endif

	spin_lock_irqsave(&vm_dev->lock, flags);
	list_for_each_entry(info, &vm_dev->virtqueues, node)
		ret |= vring_interrupt(irq, info->vq);
	spin_unlock_irqrestore(&vm_dev->lock, flags);

	if (ret == IRQ_NONE)
		dev_dbg(&vm_dev->vdev.dev, "vq has no data\n");

	return ret;
}

static void vm_del_vq(struct virtqueue *vq)
{
	struct virtio_light_device *vm_dev = to_virtio_light_device(vq->vdev);
	struct virtio_light_vq_info *info = vq->priv;
	unsigned long flags;
	unsigned int index = vq->index;

	spin_lock_irqsave(&vm_dev->lock, flags);
	list_del(&info->node);
	spin_unlock_irqrestore(&vm_dev->lock, flags);

	writel(0, vm_dev->base + (VIRTIO_LIGHT_QUEUE_PFN + index * 4));

	vring_del_virtqueue(vq);

	kfree(info);
}

static void vm_del_vqs(struct virtio_device *vdev)
{
	struct virtio_light_device *vm_dev = to_virtio_light_device(vdev);
	struct virtqueue *vq, *n;

	list_for_each_entry_safe(vq, n, &vdev->vqs, list)
		vm_del_vq(vq);

	free_irq(platform_get_irq(vm_dev->pdev, 0), vm_dev);
}

static struct virtqueue *vm_setup_vq(struct virtio_device *vdev, unsigned int index,
				  void (*callback)(struct virtqueue *vq),
				  const char *name, bool ctx)
{
	struct virtio_light_device *vm_dev = to_virtio_light_device(vdev);
	struct virtio_light_vq_info *info;
	struct virtqueue *vq;
	unsigned long flags;
	unsigned int num;
	int err;
	u32 q_pfn;

	if (!name)
		return NULL;

	/* Allocate and fill out our active queue description */
	info = kmalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		err = -ENOMEM;
		goto error_kmalloc;
	}

	num = readl(vm_dev->base + VIRTIO_LIGHT_QUEUE_SIZE_MAX);
	if (num == 0) {
		err = -ENOENT;
		goto error_new_virtqueue;
	}

	/* Create the vring */
	vq = vring_create_virtqueue(index, num, VIRTIO_LIGHT_VRING_ALIGN,
				    vdev, true, true, ctx, vm_notify,
				    callback, name);
	if (!vq) {
		err = -ENOMEM;
		goto error_new_virtqueue;
	}

	/* Activate the queue */
	writel(virtqueue_get_vring_size(vq), vm_dev->base + VIRTIO_LIGHT_QUEUE_SIZE);

	q_pfn = virtqueue_get_desc_addr(vq) >> PAGE_SHIFT;
	/* Tell virtio queue(index) addr to host */
	writel(q_pfn, vm_dev->base + (VIRTIO_LIGHT_QUEUE_PFN + index * 4));
	writel(PAGE_SIZE, vm_dev->base + VIRTIO_LIGHT_QUEUE_ALIGN);

	vq->priv = info;
	info->vq = vq;

	spin_lock_irqsave(&vm_dev->lock, flags);
	list_add(&info->node, &vm_dev->virtqueues);
	spin_unlock_irqrestore(&vm_dev->lock, flags);

	return vq;

error_new_virtqueue:
	writel(0xdead, vm_dev->base + (VIRTIO_LIGHT_QUEUE_PFN + index * 4));
	kfree(info);

error_kmalloc:
	return ERR_PTR(err);
}

static void vm_get(struct virtio_device *vdev, unsigned int offset,
		   void *buf, unsigned int len)
{
	struct virtio_light_device *vm_dev = to_virtio_light_device(vdev);
	void __iomem *base = vm_dev->base + VIRTIO_LIGHT_CONFIG;
	u8 b;
	__le16 w;
	__le32 l;

	if (vm_dev->version == 1) {
		u8 *ptr = buf;
		int i;

		for (i = 0; i < len; i++)
			ptr[i] = readb(base + offset + i);
		return;
	}

	switch (len) {
	case 1:
		b = readb(base + offset);
		memcpy(buf, &b, sizeof b);
		break;
	case 2:
		w = cpu_to_le16(readw(base + offset));
		memcpy(buf, &w, sizeof w);
		break;
	case 4:
		l = cpu_to_le32(readl(base + offset));
		memcpy(buf, &l, sizeof l);
		break;
	case 8:
		l = cpu_to_le32(readl(base + offset));
		memcpy(buf, &l, sizeof l);
		l = cpu_to_le32(ioread32(base + offset + sizeof l));
		memcpy(buf + sizeof l, &l, sizeof l);
		break;
	default:
		BUG();
	}
}

static void vm_set(struct virtio_device *vdev, unsigned int offset,
		   const void *buf, unsigned int len)
{
	struct virtio_light_device *vm_dev = to_virtio_light_device(vdev);
	void __iomem *base = vm_dev->base + VIRTIO_LIGHT_CONFIG;
	u8 b;
	__le16 w;
	__le32 l;

	if (vm_dev->version == 1) {
		const u8 *ptr = buf;
		int i;

		for (i = 0; i < len; i++)
			writeb(ptr[i], base + offset + i);

		return;
	}

	switch (len) {
	case 1:
		memcpy(&b, buf, sizeof b);
		writeb(b, base + offset);
		break;
	case 2:
		memcpy(&w, buf, sizeof w);
		writew(le16_to_cpu(w), base + offset);
		break;
	case 4:
		memcpy(&l, buf, sizeof l);
		writel(le32_to_cpu(l), base + offset);
		break;
	case 8:
		memcpy(&l, buf, sizeof l);
		writel(le32_to_cpu(l), base + offset);
		memcpy(&l, buf + sizeof l, sizeof l);
		writel(le32_to_cpu(l), base + offset + sizeof l);
		break;
	default:
		BUG();
	}
}

static int vm_find_vqs(struct virtio_device *vdev, unsigned int nvqs,
		       struct virtqueue *vqs[],
		       vq_callback_t *callbacks[],
		       const char * const names[],
		       const bool *ctx,
		       struct irq_affinity *desc)
{
	int i, irq, err, queue_idx = 0;
	struct virtio_light_device *vm_dev = to_virtio_light_device(vdev);

	if (nvqs <= 0)
		return -EINVAL;

	irq = platform_get_irq(vm_dev->pdev, 0);
	if (irq < 0) {
		dev_err(&vdev->dev, "get irq resource failed for device %d\n",
				vm_dev->vdev.id.device);
	}

	err = request_irq(irq, vm_interrupt, IRQF_SHARED, dev_name(&vdev->dev), vm_dev);
	if (err) {
		dev_err(&vdev->dev, "irq register failed for device %d\n",
				vm_dev->vdev.id.device);
		return err;
	}

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

	writel(nvqs, vm_dev->base + VIRTIO_LIGHT_QUEUE_NUM);

	return 0;
}

static u64 vm_get_features(struct virtio_device *vdev)
{
	struct virtio_light_device *vm_dev = to_virtio_light_device(vdev);
	u64 features;

	features = readl(vm_dev->base + VIRTIO_LIGHT_DEVICE_FEATURES_HIGH);
	features <<= 32;
	features = readl(vm_dev->base + VIRTIO_LIGHT_DEVICE_FEATURES_LOW);

	return features;
}

static int vm_finalize_features(struct virtio_device *vdev)
{
	return 0;
}

static const char *vm_bus_name(struct virtio_device *vdev)
{
	struct virtio_light_device *vm_dev = to_virtio_light_device(vdev);

	return vm_dev->pdev->name;
}

static const struct virtio_config_ops virtio_light_config_ops = {
	.get		= vm_get,
	.set		= vm_set,
	.get_status	= vm_get_status,
	.set_status	= vm_set_status,
	.reset		= vm_reset,
	.find_vqs	= vm_find_vqs,
	.del_vqs	= vm_del_vqs,
	.get_features   = vm_get_features,
	.finalize_features = vm_finalize_features,
	.bus_name	= vm_bus_name,
};

static void virtio_light_release_dev(struct device *_d)
{
	struct virtio_device *vdev =
			container_of(_d, struct virtio_device, dev);
	struct virtio_light_device *vm_dev =
			container_of(vdev, struct virtio_light_device, vdev);
	struct platform_device *pdev = vm_dev->pdev;

	devm_kfree(&pdev->dev, vm_dev);
}

/* Platform device */
static int virtio_light_probe(struct platform_device *pdev)
{
	struct virtio_light_device *vm_dev;
	struct device *dev = &pdev->dev;
	unsigned long magic;
	int rc;

	vm_dev = devm_kzalloc(dev, sizeof(*vm_dev), GFP_KERNEL);
	if (!vm_dev)
		return -ENOMEM;

	vm_dev->vdev.dev.parent = dev;
	vm_dev->vdev.dev.release = virtio_light_release_dev;
	vm_dev->vdev.config = &virtio_light_config_ops;
	vm_dev->pdev = pdev;
	INIT_LIST_HEAD(&vm_dev->virtqueues);
	spin_lock_init(&vm_dev->lock);

	vm_dev->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(vm_dev->base))
		return PTR_ERR(vm_dev->base);

	/* Check magic value */
	magic = readl(vm_dev->base + VIRTIO_LIGHT_MAGIC_VALUE);
	if (magic != ('v' | 'i' << 8 | 'r' << 16 | 't' << 24)) {
		dev_warn(&pdev->dev, "Wrong magic value 0x%08lx!\n", magic);
		return -ENODEV;
	}

	/* Check device version */
	vm_dev->version = readl(vm_dev->base + VIRTIO_LIGHT_VERSION);
	if (vm_dev->version < 1 || vm_dev->version > 2) {
		dev_err(&pdev->dev, "Version %ld not supported!\n",
				vm_dev->version);
		return -ENXIO;
	}

	vm_dev->vdev.id.device = readl(vm_dev->base + VIRTIO_LIGHT_DEVICE_ID);
	if (vm_dev->vdev.id.device == 0) {
		/*
		 * virtio-mmio device with an ID 0 is a (dummy) placeholder
		 * with no function. End probing now with no error reported.
		 */
		return -ENODEV;
	}
	vm_dev->vdev.id.vendor = readl(vm_dev->base + VIRTIO_LIGHT_VENDOR_ID);

#ifdef CONFIG_SOC_INT_SRC7
	vm_dev->frontend_intr_reg = ioremap(0xFFEF018094, 4);
	vm_dev->backend_intr_reg = ioremap(0xFFFF019094, 4);
#else
	/* MPW use mailbox as interrupt */
	vm_dev->backend_intr_reg  = ioremap(0xffffc3b000, 0x100);
	vm_dev->frontend_intr_reg = ioremap(0xffefc50000, 0x100);
	vm_dev->enable_intr = 1;
	vm_dev->clear_intr = 1;
	writel(vm_dev->enable_intr, vm_dev->backend_intr_reg + 0xc);
	writel(vm_dev->clear_intr, vm_dev->backend_intr_reg + 0x4);
#endif

	dev_info(&vm_dev->vdev.dev, "virtio_light: detected a virtual device with id %d\n",
				vm_dev->vdev.id.device);

	platform_set_drvdata(pdev, vm_dev);

	rc = register_virtio_device(&vm_dev->vdev);
	if (rc)
		put_device(&vm_dev->vdev.dev);

	return rc;
}

static int virtio_light_remove(struct platform_device *pdev)
{
	struct virtio_light_device *vm_dev = platform_get_drvdata(pdev);
	unregister_virtio_device(&vm_dev->vdev);

	return 0;
}

/* Platform driver */
static const struct of_device_id virtio_light_match[] = {
	{ .compatible = "virtio,light", },
	{},
};
MODULE_DEVICE_TABLE(of, virtio_light_match);

static struct platform_driver virtio_light_driver = {
	.probe		= virtio_light_probe,
	.remove		= virtio_light_remove,
	.driver		= {
		.name	= "virtio-light",
		.of_match_table	= virtio_light_match,
	},
};

static int __init virtio_light_init(void)
{
	return platform_driver_register(&virtio_light_driver);
}

static void __exit virtio_light_exit(void)
{
	platform_driver_unregister(&virtio_light_driver);
}

module_init(virtio_light_init);
module_exit(virtio_light_exit);

MODULE_DESCRIPTION("Platform bus driver for light virtio devices");
MODULE_AUTHOR("Xianting Tian <xianting.tian@linux.alibaba.com>");
MODULE_LICENSE("GPL");
