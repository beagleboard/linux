// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Western Digital Corporation or its affiliates.
 *
 * Authors:
 *     Anup Patel <anup.patel@wdc.com>
 */

#include <linux/errno.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/kvm_host.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>

static unsigned int kvm_arch_irq;

static volatile unsigned char __iomem *kvm_arch_guest_intr_reg;

extern bool kvm_riscv_vcpu_notify(void);

void kvm_arch_notify_guest(void)
{
	writel(1, kvm_arch_guest_intr_reg);
}

static irqreturn_t kvm_arch_interrupt(int irq, void *opaque)
{
	struct kvm *kvm = (struct kvm *)opaque;

#ifdef CONFIG_SOC_INT_SRC7
	writel(0, kvm->arch.backend_intr_reg);
#else
	/* Clear mailbox interrupt */
	writel(kvm->arch.clear_intr, kvm->arch.backend_intr_reg + 0x4);
#endif

	if (kvm_riscv_vcpu_notify())
		wake_up(&kvm->arch.waitq);

	return IRQ_HANDLED;
}

u64 khv_reserved_memory = (u64)-1;
u64 khv_reserved_memory_size = (u64)-1;

int kvm_arch_init_vm(struct kvm *kvm, unsigned long type)
{
	kvm->arch.khv_base_phys_addr = 0x1fd000;
	kvm->arch.khv_base = (unsigned long)ioremap(kvm->arch.khv_base_phys_addr, 0x100);
	memset((void *)kvm->arch.khv_base, 0, 0x100);

	kvm->arch.io_switch = (struct khv_io *)kvm->arch.khv_base;

#ifdef CONFIG_SOC_INT_SRC7
	kvm->arch.backend_intr_reg  = ioremap(0xFFFF019094, 4);
	kvm->arch.frontend_intr_reg = ioremap(0xFFEF018094, 4);
	kvm_arch_guest_intr_reg = kvm->arch.frontend_intr_reg;
#else
	/* MPW use mailbox as interrupt */
	kvm->arch.backend_intr_reg  = ioremap(0xffffc3b000, 0x100);
	kvm->arch.frontend_intr_reg = ioremap(0xffefc50000, 0x100);
	kvm->arch.enable_intr = 1;
	kvm->arch.clear_intr = 1;
	writel(kvm->arch.enable_intr, kvm->arch.frontend_intr_reg + 0xc);
	writel(kvm->arch.clear_intr, kvm->arch.frontend_intr_reg + 0x4);
	kvm_arch_guest_intr_reg = kvm->arch.frontend_intr_reg + 0x10;
#endif

	init_waitqueue_head(&kvm->arch.waitq);

	if (request_irq(kvm_arch_irq, kvm_arch_interrupt, IRQF_SHARED,
			"khv-interrupt", kvm) < 0) {
		pr_err("kvm backend: register IRQ %d failed\n", kvm_arch_irq);
		return -EINVAL;
	}

	return 0;
}

void kvm_arch_destroy_vm(struct kvm *kvm)
{
	int i;

	for (i = 0; i < KVM_MAX_VCPUS; ++i) {
		if (kvm->vcpus[i]) {
			kvm_arch_vcpu_destroy(kvm->vcpus[i]);
			kvm->vcpus[i] = NULL;
		}
	}
}

int kvm_vm_ioctl_check_extension(struct kvm *kvm, long ext)
{
	int r;

	switch (ext) {
	case KVM_CAP_IOEVENTFD:
	case KVM_CAP_DEVICE_CTRL:
	case KVM_CAP_USER_MEMORY:
	case KVM_CAP_SYNC_MMU:
	case KVM_CAP_DESTROY_MEMORY_REGION_WORKS:
	case KVM_CAP_ONE_REG:
	case KVM_CAP_READONLY_MEM:
	case KVM_CAP_MP_STATE:
	case KVM_CAP_IMMEDIATE_EXIT:
		r = 1;
		break;
	case KVM_CAP_NR_VCPUS:
		r = 0xffff;
		break;
	case KVM_CAP_MAX_VCPUS:
		r = KVM_MAX_VCPUS;
		break;
	case KVM_CAP_NR_MEMSLOTS:
		r = KVM_USER_MEM_SLOTS;
		break;
	default:
		r = 0;
		break;
	}

	return r;
}

long kvm_arch_vm_ioctl(struct file *filp,
		       unsigned int ioctl, unsigned long arg)
{
	return 0;
}

static int khv_probe(struct platform_device *pdev)
{
	struct device_node *np;
	struct resource r;
	struct device *dev;
	int rc;

	dev = &pdev->dev;

	kvm_arch_irq = platform_get_irq(pdev, 0);
	if (kvm_arch_irq <= 0) {
		pr_err("Cannot get IRQ resource for kvm backend\n");
		return -EINVAL;
	}

	np = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!np) {
		goto out;
	}

	rc = of_address_to_resource(np, 0, &r);
	if (rc) {
		dev_err(dev, "No memory address assigned to the region\n");
		return -ENOMEM;
	}

	khv_reserved_memory = r.start;
	khv_reserved_memory_size = r.end - r.start + 1;
out:
	printk("%s, %d, irq: %d.\n", __func__, __LINE__, kvm_arch_irq);

	return 0;
}

static int khv_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id khv_of_table[] = {
	{ .compatible = "thead,khv-host" },
	{ }
};

MODULE_DEVICE_TABLE(of, khv_of_table);

static struct platform_driver khv_driver = {
	.probe  = khv_probe,
	.remove = khv_remove,
	.driver = {
		.name = "KVM-based Heterogeneous Virtualization",
		.of_match_table = khv_of_table,
	},
};
module_platform_driver(khv_driver);
