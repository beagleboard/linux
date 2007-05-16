#ifndef __PLAT_OMAP_DSP_MMU_H
#define __PLAT_OMAP_DSP_MMU_H

#ifdef CONFIG_ARCH_OMAP1

#ifdef CONFIG_ARCH_OMAP15XX
struct omap_mmu dsp_mmu = {
	.name		= "mmu:dsp",
	.type		= OMAP_MMU_DSP,
	.base		= DSP_MMU_BASE,
	.membase	= OMAP1510_DSP_BASE,
	.memsize	= OMAP1510_DSP_SIZE,
	.nr_tlb_entries	= 32,
	.addrspace	= 24,
	.irq		= INT_1510_DSP_MMU,
	.ops		= &omap1_mmu_ops,
};
#endif
#ifdef CONFIG_ARCH_OMAP16XX
struct omap_mmu dsp_mmu = {
	.name		= "mmu:dsp",
	.type		= OMAP_MMU_DSP,
	.base		= DSP_MMU_BASE,
	.membase	= OMAP16XX_DSP_BASE,
	.memsize	= OMAP16XX_DSP_SIZE,
	.nr_tlb_entries	= 32,
	.addrspace	= 24,
	.irq		= INT_1610_DSP_MMU,
	.ops		= &omap1_mmu_ops,
};
#endif
#else /* OMAP2 */
struct omap_mmu dsp_mmu = {
	.name		= "mmu:dsp",
	.type		= OMAP_MMU_DSP,
	.base		= DSP_MMU_24XX_VIRT,
	.membase	= DSP_MEM_24XX_VIRT,
	.memsize	= DSP_MEM_24XX_SIZE,
	.nr_tlb_entries	= 32,
	.addrspace	= 24,
	.irq		= INT_24XX_DSP_MMU,
	.ops		= &omap2_mmu_ops,
};

#define IOMAP_VAL	0x3f
#endif

#ifdef CONFIG_FB_OMAP_LCDC_EXTERNAL
static struct omapfb_notifier_block *omapfb_nb;
static int omapfb_ready;
#endif

/*
 * OMAP1 EMIFF access
 */
#ifdef CONFIG_ARCH_OMAP1
#define EMIF_PRIO_LB_MASK	0x0000f000
#define EMIF_PRIO_LB_SHIFT	12
#define EMIF_PRIO_DMA_MASK	0x00000f00
#define EMIF_PRIO_DMA_SHIFT	8
#define EMIF_PRIO_DSP_MASK	0x00000070
#define EMIF_PRIO_DSP_SHIFT	4
#define EMIF_PRIO_MPU_MASK	0x00000007
#define EMIF_PRIO_MPU_SHIFT	0
#define set_emiff_dma_prio(prio) \
	do { \
		omap_writel((omap_readl(OMAP_TC_OCPT1_PRIOR) & \
			     ~EMIF_PRIO_DMA_MASK) | \
			    ((prio) << EMIF_PRIO_DMA_SHIFT), \
			    OMAP_TC_OCPT1_PRIOR); \
	} while(0)
#else
#define set_emiff_dma_prio(prio)	do { } while (0)
#endif /* CONFIG_ARCH_OMAP1 */

#ifdef CONFIG_ARCH_OMAP1
static int dsp_mmu_itack(void)
{
	unsigned long dspadr;

	pr_info("omapdsp: sending DSP MMU interrupt ack.\n");
	if (!dsp_err_isset(ERRCODE_MMU)) {
		printk(KERN_ERR "omapdsp: DSP MMU error has not been set.\n");
		return -EINVAL;
	}
	dspadr = dsp_mmu.fault_address & ~(SZ_4K-1);
	/* FIXME: reserve TLB entry for this */
	omap_mmu_exmap(&dsp_mmu, dspadr, 0, SZ_4K, EXMAP_TYPE_MEM);
	pr_info("omapdsp: falling into recovery runlevel...\n");
	dsp_set_runlevel(RUNLEVEL_RECOVERY);
	omap_mmu_itack(&dsp_mmu);
	udelay(100);
	omap_mmu_exunmap(&dsp_mmu, dspadr);
	dsp_err_clear(ERRCODE_MMU);
	return 0;
}

/*
 * intmem_enable() / disable():
 * if the address is in DSP internal memories,
 * we send PM mailbox commands so that DSP DMA domain won't go in idle
 * when ARM is accessing to those memories.
 */
static int intmem_enable(void)
{
	int ret = 0;

	if (dsp_cfgstat_get_stat() == CFGSTAT_READY)
		ret = mbcompose_send(PM, PM_ENABLE, DSPREG_ICR_DMA);

	return ret;
}

static void intmem_disable(void) {
	if (dsp_cfgstat_get_stat() == CFGSTAT_READY)
		mbcompose_send(PM, PM_DISABLE, DSPREG_ICR_DMA);
}
#else
static int intmem_enable(void) { return 0; }
static void intmem_disable(void) { }
static int dsp_mmu_itack(void) { return 0; }
#endif

#ifdef CONFIG_ARCH_OMAP2
static inline void dsp_mem_ipi_init(void)
{
	int i, dspmem_pg_count;
	dspmem_pg_count = dspmem_size >> 12;
	for (i = 0; i < dspmem_pg_count; i++) {
		writel(i, DSP_IPI_INDEX);
		writel(DSP_IPI_ENTRY_ELMSIZEVALUE_16, DSP_IPI_ENTRY);
	}
	writel(1, DSP_IPI_ENABLE);
	writel(IOMAP_VAL, DSP_IPI_IOMAP);
}
#else
static inline void dsp_mem_ipi_init(void) { }
#endif

#endif /* __PLAT_OMAP_DSP_MMU_H */
