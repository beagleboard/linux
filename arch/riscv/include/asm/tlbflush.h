/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2009 Chen Liqin <liqin.chen@sunplusct.com>
 * Copyright (C) 2012 Regents of the University of California
 */

#ifndef _ASM_RISCV_TLBFLUSH_H
#define _ASM_RISCV_TLBFLUSH_H

#include <linux/mm_types.h>
#include <asm/smp.h>

#ifdef CONFIG_MMU
static inline void local_flush_tlb_all(void)
{
#ifdef CONFIG_NO_SFENCE_VMA
	csr_write(CSR_SMCIR, 1 << 26);
#else
	__asm__ __volatile__ ("sfence.vma" : : : "memory");
#endif
}

/* Flush one page from local TLB */
static inline void local_flush_tlb_page(unsigned long addr)
{
#ifdef CONFIG_NO_SFENCE_VMA
	csr_write(CSR_SMCIR, 1 << 26);
#else
	__asm__ __volatile__ ("sfence.vma %0" : : "r" (addr) : "memory");
#endif
}
#else /* CONFIG_MMU */
#define local_flush_tlb_all()			do { } while (0)
#define local_flush_tlb_page(addr)		do { } while (0)
#endif /* CONFIG_MMU */

#if defined(CONFIG_SMP) && defined(CONFIG_MMU)
void flush_tlb_all(void);
void flush_tlb_mm(struct mm_struct *mm);
void flush_tlb_page(struct vm_area_struct *vma, unsigned long addr);
void flush_tlb_range(struct vm_area_struct *vma, unsigned long start,
		     unsigned long end);
#else /* CONFIG_SMP && CONFIG_MMU */

#define flush_tlb_all() local_flush_tlb_all()
#define flush_tlb_page(vma, addr) local_flush_tlb_page(addr)

static inline void flush_tlb_range(struct vm_area_struct *vma,
		unsigned long start, unsigned long end)
{
	local_flush_tlb_all();
}

#define flush_tlb_mm(mm) flush_tlb_all()
#endif /* !CONFIG_SMP || !CONFIG_MMU */

/* Flush a range of kernel pages */
static inline void flush_tlb_kernel_range(unsigned long start,
	unsigned long end)
{
#ifdef CONFIG_NO_SFENCE_VMA
	csr_write(CSR_SMCIR, 1 << 26);
#else
	start &= PAGE_MASK;
	end   += PAGE_SIZE - 1;
	end   &= PAGE_MASK;

	while (start < end) {
		__asm__ __volatile__ ("sfence.vma %0" : : "r" (start) : "memory");
		start += PAGE_SIZE;
	}
#endif
}

#endif /* _ASM_RISCV_TLBFLUSH_H */
