// SPDX-License-Identifier: GPL-2.0

#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/kvm_host.h>
#include <asm/csr.h>

/**
 * kvm_riscv_vcpu_mmio_return -- Handle MMIO loads after user space emulation
 *			     or in-kernel IO emulation
 *
 * @vcpu: The VCPU pointer
 * @run:  The VCPU run struct containing the mmio data
 */
int kvm_riscv_vcpu_mmio_return(struct kvm_vcpu *vcpu, struct kvm_run *run)
{
	struct kvm *kvm = vcpu->kvm;
	struct khv_io *io_switch = kvm->arch.io_switch;
	int len;

	if (run->mmio.is_write)
		goto out;

	if (io_switch->status & BIT(7))
		while(1)
			printk_once(KERN_INFO "Can't be mmio write here\n");

	len = 1 << ((io_switch->status >> 8) & 0xf);

	switch (len) {
	case 1:
		*((u8  *)&io_switch->value) = *((u8  *)run->mmio.data);
		break;
	case 2:
		*((u16 *)&io_switch->value) = *((u16 *)run->mmio.data);
		break;
	case 4:
		*((u32 *)&io_switch->value) = *((u32 *)run->mmio.data);
		break;
	case 8:
		*((u64 *)&io_switch->value) = *((u64 *)run->mmio.data);
		break;
	default:
		return -EOPNOTSUPP;
	};

out:
	if ((run->mmio.phys_addr & 0xff) != 0x50 ) { // skip VIRTIO_MMIO_QUEUE_NOTIFY
		// READY
		writeb(2, (void __iomem *)&io_switch->status);

		while (readb((void __iomem *)&io_switch->status) != 3)
			cpu_relax();
	}

	writel(0, (void __iomem *)&io_switch->status);

	return 0;
}

/*
 * Return > 0 to return to guest, < 0 on error, 0 (and set exit_reason) on
 * proper exit to userspace.
 */
int kvm_riscv_vcpu_exit(struct kvm_vcpu *vcpu, struct kvm_run *run,
			struct kvm_cpu_trap *trap)
{
	int ret;
	struct kvm *kvm = vcpu->kvm;
	struct khv_io *io_switch = kvm->arch.io_switch;
	int len;

	ret = -EFAULT;

	if (((io_switch->status >> 16) & 0xff) == 0x5a) {
		run->exit_reason = KVM_EXIT_SHUTDOWN;
		io_switch->status = 0;
		return 0;
	}

	run->exit_reason = KVM_EXIT_UNKNOWN;

	len = 1 << ((io_switch->status >> 8) & 0xf);

	/* Update MMIO details in kvm_run struct */
	run->mmio.phys_addr = io_switch->addr;
	run->mmio.len = len;

	if (io_switch->status & BIT(7))
		run->mmio.is_write = 1;
	else {
		run->mmio.is_write = 0;
		goto out;
	}

	/* Copy data to kvm_run instance */
	switch (len) {
	case 1:
		*((u8 *)run->mmio.data)  = *((u8  *)&io_switch->value);
		break;
	case 2:
		*((u16 *)run->mmio.data) = *((u16 *)&io_switch->value);
		break;
	case 4:
		*((u32 *)run->mmio.data) = *((u32 *)&io_switch->value);
		break;
	case 8:
		*((u64 *)run->mmio.data) = *((u64 *)&io_switch->value);
		break;
	default:
		pr_info("%s, %llx, len: %d.\n", __func__, io_switch->status, len);
		return -EOPNOTSUPP;
	};

	pr_debug("mmio: 0x%llx 0x%x %d %d\n",
		 run->mmio.phys_addr, *((u32 *)run->mmio.data),
		 len, run->mmio.is_write);

out:
	/* Try to handle MMIO access in the kernel */
	if (!kvm_io_bus_write(vcpu, KVM_MMIO_BUS,
			      io_switch->addr, len, run->mmio.data)) {
		/* Successfully handled MMIO access in the kernel so resume */
		vcpu->stat.mmio_exit_kernel++;
		kvm_riscv_vcpu_mmio_return(vcpu, run);
		return 1;
	}

	/* Exit to userspace for MMIO emulation */
	vcpu->stat.mmio_exit_user++;
	run->exit_reason = KVM_EXIT_MMIO;

	return 0;
}
