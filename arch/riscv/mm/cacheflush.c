// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2017 SiFive
 */

#include <asm/cacheflush.h>
#include <asm/sbi.h>

#ifdef CONFIG_SMP

static void ipi_remote_fence_i(void *info)
{
	return local_flush_icache_all();
}

void flush_icache_all(void)
{
	local_flush_icache_all();

	if (IS_ENABLED(CONFIG_RISCV_SBI))
		sbi_remote_fence_i(NULL);
	else
		on_each_cpu(ipi_remote_fence_i, NULL, 1);
}
EXPORT_SYMBOL(flush_icache_all);

/*
 * Performs an icache flush for the given MM context.  RISC-V has no direct
 * mechanism for instruction cache shoot downs, so instead we send an IPI that
 * informs the remote harts they need to flush their local instruction caches.
 * To avoid pathologically slow behavior in a common case (a bunch of
 * single-hart processes on a many-hart machine, ie 'make -j') we avoid the
 * IPIs for harts that are not currently executing a MM context and instead
 * schedule a deferred local instruction cache flush to be performed before
 * execution resumes on each hart.
 */
void flush_icache_mm(struct mm_struct *mm, bool local)
{
	unsigned int cpu;
	cpumask_t others, *mask;

	preempt_disable();

	/* Mark every hart's icache as needing a flush for this MM. */
	mask = &mm->context.icache_stale_mask;
	cpumask_setall(mask);
	/* Flush this hart's I$ now, and mark it as flushed. */
	cpu = smp_processor_id();
	cpumask_clear_cpu(cpu, mask);
	local_flush_icache_all();

	/*
	 * Flush the I$ of other harts concurrently executing, and mark them as
	 * flushed.
	 */
	cpumask_andnot(&others, mm_cpumask(mm), cpumask_of(cpu));
	local |= cpumask_empty(&others);
	if (mm == current->active_mm && local) {
		/*
		 * It's assumed that at least one strongly ordered operation is
		 * performed on this hart between setting a hart's cpumask bit
		 * and scheduling this MM context on that hart.  Sending an SBI
		 * remote message will do this, but in the case where no
		 * messages are sent we still need to order this hart's writes
		 * with flush_icache_deferred().
		 */
		smp_mb();
	} else if (IS_ENABLED(CONFIG_RISCV_SBI)) {
		cpumask_t hartid_mask;

		riscv_cpuid_to_hartid_mask(&others, &hartid_mask);
		sbi_remote_fence_i(cpumask_bits(&hartid_mask));
	} else {
		on_each_cpu_mask(&others, ipi_remote_fence_i, NULL, 1);
	}

	preempt_enable();
}

#endif /* CONFIG_SMP */

#ifdef CONFIG_MMU
void flush_icache_pte(pte_t pte)
{
	struct page *page = pte_page(pte);

	if (!test_bit(PG_dcache_clean, &page->flags)) {
		flush_icache_all();
		set_bit(PG_dcache_clean, &page->flags);
	}
}
#endif /* CONFIG_MMU */

static bool thead_dma_init_flag = false;

#define sync_is()	asm volatile (".long 0x01a0000b")
void dma_wbinv_range(unsigned long start, unsigned long end)
{
	register unsigned long i asm("a0") = start & ~(L1_CACHE_BYTES - 1);

	if (!thead_dma_init_flag)
		return;

	for (; i < end; i += L1_CACHE_BYTES)
		asm volatile (".long 0x02b5000b"); /* dcache.cipa a0 */

	sync_is();
}

void dma_wb_range(unsigned long start, unsigned long end)
{
	register unsigned long i asm("a0") = start & ~(L1_CACHE_BYTES - 1);

	if (!thead_dma_init_flag)
		return;

	for (; i < end; i += L1_CACHE_BYTES)
		asm volatile (".long 0x0295000b"); /* dcache.cpa a0 */

	sync_is();
}

#define THEAD_VENDOR_ID       0x5b7

static int __init thead_dma_init(void)
{
	if (sbi_get_mvendorid() == THEAD_VENDOR_ID)
		thead_dma_init_flag = true;

	return 0;
}
arch_initcall(thead_dma_init);
