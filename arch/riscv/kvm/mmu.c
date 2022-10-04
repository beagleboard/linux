// SPDX-License-Identifier: GPL-2.0

#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/hugetlb.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/kvm_host.h>
#include <linux/sched/signal.h>

void kvm_arch_mmu_enable_log_dirty_pt_masked(struct kvm *kvm,
					     struct kvm_memory_slot *slot,
					     gfn_t gfn_offset,
					     unsigned long mask)
{
}

void kvm_arch_sync_dirty_log(struct kvm *kvm, struct kvm_memory_slot *memslot)
{
}

void kvm_arch_flush_remote_tlbs_memslot(struct kvm *kvm,
					struct kvm_memory_slot *memslot)
{
	kvm_flush_remote_tlbs(kvm);
}

void kvm_arch_free_memslot(struct kvm *kvm, struct kvm_memory_slot *free)
{
}

int kvm_arch_create_memslot(struct kvm *kvm, struct kvm_memory_slot *slot,
			    unsigned long npages)
{
	return 0;
}

void kvm_arch_memslots_updated(struct kvm *kvm, u64 gen)
{
}

void kvm_arch_flush_shadow_all(struct kvm *kvm)
{
}

void kvm_arch_flush_shadow_memslot(struct kvm *kvm,
				   struct kvm_memory_slot *slot)
{
}

void kvm_arch_commit_memory_region(struct kvm *kvm,
				const struct kvm_userspace_memory_region *mem,
				struct kvm_memory_slot *old,
				const struct kvm_memory_slot *new,
				enum kvm_mr_change change)
{
}

int kvm_arch_prepare_memory_region(struct kvm *kvm,
				struct kvm_memory_slot *memslot,
				const struct kvm_userspace_memory_region *mem,
				enum kvm_mr_change change)
{
	return 0;
}

int kvm_unmap_hva_range(struct kvm *kvm, unsigned long start,
			unsigned long end, unsigned int flags)
{
	return 0;
}

int kvm_set_spte_hva(struct kvm *kvm, unsigned long hva, pte_t pte)
{
	return 0;
}

int kvm_age_hva(struct kvm *kvm, unsigned long start, unsigned long end)
{
	return 0;
}

int kvm_test_age_hva(struct kvm *kvm, unsigned long hva)
{
	return 0;
}
