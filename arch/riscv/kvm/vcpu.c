// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Western Digital Corporation or its affiliates.
 *
 * Authors:
 *     Anup Patel <anup.patel@wdc.com>
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/kdebug.h>
#include <linux/module.h>
#include <linux/percpu.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/sched/signal.h>
#include <linux/fs.h>
#include <linux/kvm_host.h>
#include <asm/csr.h>
#include <asm/hwcap.h>
#include <asm/sbi.h>

struct kvm_stats_debugfs_item debugfs_entries[] = {
	VCPU_STAT("halt_successful_poll", halt_successful_poll),
	VCPU_STAT("halt_attempted_poll", halt_attempted_poll),
	VCPU_STAT("halt_poll_success_ns", halt_poll_success_ns),
	VCPU_STAT("halt_poll_fail_ns", halt_poll_fail_ns),
	VCPU_STAT("halt_poll_invalid", halt_poll_invalid),
	VCPU_STAT("halt_wakeup", halt_wakeup),
	VCPU_STAT("ecall_exit_stat", ecall_exit_stat),
	VCPU_STAT("wfi_exit_stat", wfi_exit_stat),
	VCPU_STAT("mmio_exit_user", mmio_exit_user),
	VCPU_STAT("mmio_exit_kernel", mmio_exit_kernel),
	VCPU_STAT("exits", exits),
	{ NULL }
};

static struct kvm_vcpu *kvm_vcpu = NULL;

#define KVM_RISCV_ISA_ALLOWED	(riscv_isa_extension_mask(a) | \
				 riscv_isa_extension_mask(c) | \
				 riscv_isa_extension_mask(d) | \
				 riscv_isa_extension_mask(f) | \
				 riscv_isa_extension_mask(i) | \
				 riscv_isa_extension_mask(m) | \
				 riscv_isa_extension_mask(s) | \
				 riscv_isa_extension_mask(u))

int kvm_arch_vcpu_precreate(struct kvm *kvm, unsigned int id)
{
	return 0;
}

int kvm_arch_vcpu_create(struct kvm_vcpu *vcpu)
{
	return 0;
}

int kvm_arch_vcpu_setup(struct kvm_vcpu *vcpu)
{
	return 0;
}

void kvm_arch_vcpu_postcreate(struct kvm_vcpu *vcpu)
{
}

void kvm_arch_vcpu_destroy(struct kvm_vcpu *vcpu)
{
}

int kvm_cpu_has_pending_timer(struct kvm_vcpu *vcpu)
{
	return 0;
}

void kvm_arch_vcpu_blocking(struct kvm_vcpu *vcpu)
{
}

void kvm_arch_vcpu_unblocking(struct kvm_vcpu *vcpu)
{
}

int kvm_arch_vcpu_runnable(struct kvm_vcpu *vcpu)
{
	return 0;
}

int kvm_arch_vcpu_should_kick(struct kvm_vcpu *vcpu)
{
	return 0;
}

bool kvm_arch_vcpu_in_kernel(struct kvm_vcpu *vcpu)
{
	return false;
}

bool kvm_arch_has_vcpu_debugfs(void)
{
	return false;
}

int kvm_arch_create_vcpu_debugfs(struct kvm_vcpu *vcpu)
{
	return 0;
}

vm_fault_t kvm_arch_vcpu_fault(struct kvm_vcpu *vcpu, struct vm_fault *vmf)
{
	return VM_FAULT_SIGBUS;
}

extern u64 khv_reserved_memory;
long kvm_arch_vcpu_async_ioctl(struct file *filp,
			       unsigned int ioctl, unsigned long arg)
{
	return -ENOIOCTLCMD;
}

long kvm_arch_vcpu_ioctl(struct file *filp,
			 unsigned int ioctl, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	long r = -EINVAL;

	switch (ioctl) {
	case KVM_SET_ONE_REG:
	case KVM_GET_ONE_REG: {
		struct kvm_one_reg reg;

		r = -EFAULT;

		if (copy_from_user(&reg, argp, sizeof(reg)))
			break;

		sbi_ecall(SBI_EXT_HSM, SBI_EXT_HSM_HART_START, reg.id, khv_reserved_memory + 0x200000, reg.addr, 0, 0, 0);

		if (cpu_online(reg.id) || !cpu_possible(reg.id)) {
			break;
		} else
			return 0;

		break;
	}
	default:
		break;
	}

	return r;
}

int kvm_arch_vcpu_ioctl_get_sregs(struct kvm_vcpu *vcpu,
				  struct kvm_sregs *sregs)
{
	return -EINVAL;
}

int kvm_arch_vcpu_ioctl_set_sregs(struct kvm_vcpu *vcpu,
				  struct kvm_sregs *sregs)
{
	return -EINVAL;
}

int kvm_arch_vcpu_ioctl_get_fpu(struct kvm_vcpu *vcpu, struct kvm_fpu *fpu)
{
	return -EINVAL;
}

int kvm_arch_vcpu_ioctl_set_fpu(struct kvm_vcpu *vcpu, struct kvm_fpu *fpu)
{
	return -EINVAL;
}

int kvm_arch_vcpu_ioctl_translate(struct kvm_vcpu *vcpu,
				  struct kvm_translation *tr)
{
	return -EINVAL;
}

int kvm_arch_vcpu_ioctl_get_regs(struct kvm_vcpu *vcpu, struct kvm_regs *regs)
{
	return -EINVAL;
}

int kvm_arch_vcpu_ioctl_set_regs(struct kvm_vcpu *vcpu, struct kvm_regs *regs)
{
	return -EINVAL;
}

void kvm_riscv_vcpu_flush_interrupts(struct kvm_vcpu *vcpu)
{
}

void kvm_riscv_vcpu_sync_interrupts(struct kvm_vcpu *vcpu)
{
}

int kvm_riscv_vcpu_set_interrupt(struct kvm_vcpu *vcpu, unsigned int irq)
{
	return 0;
}

int kvm_riscv_vcpu_unset_interrupt(struct kvm_vcpu *vcpu, unsigned int irq)
{
	return 0;
}

bool kvm_riscv_vcpu_has_interrupts(struct kvm_vcpu *vcpu, unsigned long mask)
{
	return false;
}

void kvm_riscv_vcpu_power_off(struct kvm_vcpu *vcpu)
{
}

void kvm_riscv_vcpu_power_on(struct kvm_vcpu *vcpu)
{
}

int kvm_arch_vcpu_ioctl_get_mpstate(struct kvm_vcpu *vcpu,
				    struct kvm_mp_state *mp_state)
{
	return 0;
}

int kvm_arch_vcpu_ioctl_set_mpstate(struct kvm_vcpu *vcpu,
				    struct kvm_mp_state *mp_state)
{
	return 0;
}

int kvm_arch_vcpu_ioctl_set_guest_debug(struct kvm_vcpu *vcpu,
					struct kvm_guest_debug *dbg)
{
	/* TODO; To be implemented later. */
	return -EINVAL;
}

void kvm_arch_vcpu_load(struct kvm_vcpu *vcpu, int cpu)
{
}

void kvm_arch_vcpu_put(struct kvm_vcpu *vcpu)
{
}

int kvm_arch_vcpu_ioctl_run(struct kvm_vcpu *vcpu)
{
	int ret;
	struct kvm_cpu_trap trap;
	struct kvm_run *run = vcpu->run;

	kvm_vcpu = vcpu;

	/* Mark this VCPU ran at least once */
	vcpu->arch.ran_atleast_once = true;

	vcpu->arch.srcu_idx = srcu_read_lock(&vcpu->kvm->srcu);

	/* Process MMIO value returned from user-space */
	if (run->exit_reason == KVM_EXIT_MMIO) {
		ret = kvm_riscv_vcpu_mmio_return(vcpu, vcpu->run);
		if (ret) {
			srcu_read_unlock(&vcpu->kvm->srcu, vcpu->arch.srcu_idx);
			return ret;
		}
	}

	if (run->immediate_exit) {
		printk("%s, immediate_exit.\n", __func__);
		srcu_read_unlock(&vcpu->kvm->srcu, vcpu->arch.srcu_idx);
		return -EINTR;
	}

	vcpu_load(vcpu);

	kvm_sigset_activate(vcpu);

	ret = 1;
	run->exit_reason = KVM_EXIT_UNKNOWN;

	while (ret > 0) {
		/* Check conditions before entering the guest */
		cond_resched();

		/*
		 * Exit if we have a signal pending so that we can deliver
		 * the signal to user space.
		 */
		if (signal_pending(current)) {
			ret = -EINTR;
			run->exit_reason = KVM_EXIT_INTR;
		}

		/*
		 * Ensure we set mode to IN_GUEST_MODE after we disable
		 * interrupts and before the final VCPU requests check.
		 * See the comment in kvm_vcpu_exiting_guest_mode() and
		 * Documentation/virtual/kvm/vcpu-requests.rst
		 */
		vcpu->mode = IN_GUEST_MODE;

		srcu_read_unlock(&vcpu->kvm->srcu, vcpu->arch.srcu_idx);
		smp_mb__after_srcu_read_unlock();

		ret = wait_event_interruptible(vcpu->kvm->arch.waitq,
				(atomic64_read((atomic64_t *)&vcpu->kvm->arch.io_switch->status) & RVBM_STATUS_MASK) == RVBM_STATUS_START);
		if (ret)
			while(1);

		vcpu->mode = OUTSIDE_GUEST_MODE;
		vcpu->stat.exits++;

		vcpu->arch.srcu_idx = srcu_read_lock(&vcpu->kvm->srcu);

		ret = kvm_riscv_vcpu_exit(vcpu, run, &trap);
	}

	kvm_sigset_deactivate(vcpu);

	vcpu_put(vcpu);

	srcu_read_unlock(&vcpu->kvm->srcu, vcpu->arch.srcu_idx);

	return ret;
}

bool kvm_riscv_vcpu_notify(void)
{
	struct kvm_vcpu *vcpu = kvm_vcpu;
	struct khv_io *io_switch;
	struct kvm_cpu_trap trap;
	bool ret = false;

	if (!vcpu)
		return true;
	io_switch = vcpu->kvm->arch.io_switch;

	/* Skip guest shutdown command */
	if (((io_switch->status >> 16) & 0xff) == 0x5a)
		return true;

	if ((io_switch->addr & 0xff) == 0x50)
		kvm_riscv_vcpu_exit(vcpu, vcpu->run, &trap);
	else
		ret = true;

	return ret;
}
