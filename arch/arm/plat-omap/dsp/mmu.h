#ifndef __PLAT_OMAP_DSP_MMU_H
#define __PLAT_OMAP_DSP_MMU_H

#ifdef CONFIG_ARCH_OMAP1

#ifdef CONFIG_ARCH_OMAP15XX
struct omap_mmu dsp_mmu = {
	.name		= "dsp",
	.type		= OMAP_MMU_DSP,
	.base		= DSP_MMU_BASE,
	.membase	= OMAP15XX_DSP_BASE,
	.memsize	= OMAP15XX_DSP_SIZE,
	.nr_tlb_entries	= 32,
	.addrspace	= 24,
	.ops		= &omap1_mmu_ops,
};
#endif
#ifdef CONFIG_ARCH_OMAP16XX
struct omap_mmu dsp_mmu = {
	.name		= "dsp",
	.type		= OMAP_MMU_DSP,
	.base		= DSP_MMU_BASE,
	.membase	= OMAP16XX_DSP_BASE,
	.memsize	= OMAP16XX_DSP_SIZE,
	.nr_tlb_entries	= 32,
	.addrspace	= 24,
	.ops		= &omap1_mmu_ops,
};
#endif
#else /* OMAP2 */
struct omap_mmu dsp_mmu = {
	.name		= "dsp",
	.type		= OMAP_MMU_DSP,
	.base		= DSP_MMU_24XX_VIRT,
	.membase	= DSP_MEM_24XX_VIRT,
	.memsize	= DSP_MEM_24XX_SIZE,
	.nr_tlb_entries	= 32,
	.addrspace	= 24,
	.ops		= &omap2_mmu_ops,
};

#define IOMAP_VAL	0x3f
#endif

static u32 dsp_fault_adr;

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

/*
 * workqueue for mmu int
 */
#ifdef CONFIG_ARCH_OMAP1
/*
 * MMU fault mask:
 * We ignore prefetch err.
 */
#define MMUFAULT_MASK \
	(DSP_MMU_FAULT_ST_PERM |\
	 DSP_MMU_FAULT_ST_TLB_MISS |\
	 DSP_MMU_FAULT_ST_TRANS)

static void do_mmu_int(struct work_struct *unused)
{
	unsigned long status;
	unsigned long adh, adl;
	unsigned long dp;

	status = omap_mmu_read_reg(&dsp_mmu, DSP_MMU_FAULT_ST);
	adh = omap_mmu_read_reg(&dsp_mmu, DSP_MMU_FAULT_AD_H);
	adl = omap_mmu_read_reg(&dsp_mmu, DSP_MMU_FAULT_AD_L);
	dp = adh & DSP_MMU_FAULT_AD_H_DP;
	dsp_fault_adr = MK32(adh & DSP_MMU_FAULT_AD_H_ADR_MASK, adl);

	/* if the fault is masked, nothing to do */
	if ((status & MMUFAULT_MASK) == 0) {
		pr_debug( "DSP MMU interrupt, but ignoring.\n");
		/*
		 * note: in OMAP1710,
		 * when CACHE + DMA domain gets out of idle in DSP,
		 * MMU interrupt occurs but DSP_MMU_FAULT_ST is not set.
		 * in this case, we just ignore the interrupt.
		 */
		if (status) {
			pr_debug( "%s%s%s%s\n",
				  (status & DSP_MMU_FAULT_ST_PREF)?
				  "  (prefetch err)" : "",
				  (status & DSP_MMU_FAULT_ST_PERM)?
				  "  (permission fault)" : "",
				  (status & DSP_MMU_FAULT_ST_TLB_MISS)?
				  "  (TLB miss)" : "",
				  (status & DSP_MMU_FAULT_ST_TRANS) ?
				  "  (translation fault)": "");
			pr_debug( "fault address = %#08x\n", dsp_fault_adr);
		}
		enable_irq(omap_dsp->mmu_irq);
		return;
	}


	pr_info("%s%s%s%s\n",
		(status & DSP_MMU_FAULT_ST_PREF)?
		(MMUFAULT_MASK & DSP_MMU_FAULT_ST_PREF)?
		"  prefetch err":
		"  (prefetch err)":
		"",
		(status & DSP_MMU_FAULT_ST_PERM)?
		(MMUFAULT_MASK & DSP_MMU_FAULT_ST_PERM)?
		"  permission fault":
		"  (permission fault)":
		"",
		(status & DSP_MMU_FAULT_ST_TLB_MISS)?
		(MMUFAULT_MASK & DSP_MMU_FAULT_ST_TLB_MISS)?
		"  TLB miss":
		"  (TLB miss)":
		"",
		(status & DSP_MMU_FAULT_ST_TRANS)?
		(MMUFAULT_MASK & DSP_MMU_FAULT_ST_TRANS)?
		"  translation fault":
		"  (translation fault)":
		"");

	pr_info("fault address = %#08x\n", dsp_fault_adr);

	if (dsp_cfgstat_get_stat() == CFGSTAT_READY)
		dsp_err_set(ERRCODE_MMU, (unsigned long)dsp_fault_adr);
	else {
		__dsp_mmu_itack(&dsp_mmu);

		pr_info("Resetting DSP...\n");
		dsp_cpustat_request(CPUSTAT_RESET);
		/*
		 * if we enable followings, semaphore lock should be avoided.
		 *
		 pr_info("Flushing DSP MMU...\n");
		 exmap_flush();
		 dsp_mmu_init();
		*/
	}

	enable_irq(omap_dsp->mmu_irq);
}
#elif defined(CONFIG_ARCH_OMAP2)
static void do_mmu_int(struct work_struct *unused)
{
	unsigned long status;

	status = omap_mmu_read_reg(&dsp_mmu, DSP_MMU_IRQSTATUS);
	dsp_fault_adr = omap_mmu_read_reg(&dsp_mmu, DSP_MMU_FAULT_AD);

#define MMU_IRQ_MASK \
	(DSP_MMU_IRQ_MULTIHITFAULT | \
	 DSP_MMU_IRQ_TABLEWALKFAULT | \
	 DSP_MMU_IRQ_EMUMISS | \
	 DSP_MMU_IRQ_TRANSLATIONFAULT | \
	 DSP_MMU_IRQ_TLBMISS)

	pr_info("%s%s%s%s%s\n",
		(status & DSP_MMU_IRQ_MULTIHITFAULT)?
		(MMU_IRQ_MASK & DSP_MMU_IRQ_MULTIHITFAULT)?
		"  multi hit":
		"  (multi hit)":
		"",
		(status & DSP_MMU_IRQ_TABLEWALKFAULT)?
		(MMU_IRQ_MASK & DSP_MMU_IRQ_TABLEWALKFAULT)?
		"  table walk fault":
		"  (table walk fault)":
		"",
		(status & DSP_MMU_IRQ_EMUMISS)?
		(MMU_IRQ_MASK & DSP_MMU_IRQ_EMUMISS)?
		"  EMU miss":
		"  (EMU miss)":
		"",
		(status & DSP_MMU_IRQ_TRANSLATIONFAULT)?
		(MMU_IRQ_MASK & DSP_MMU_IRQ_TRANSLATIONFAULT)?
		"  translation fault":
		"  (translation fault)":
		"",
		(status & DSP_MMU_IRQ_TLBMISS)?
		(MMU_IRQ_MASK & DSP_MMU_IRQ_TLBMISS)?
		"  TLB miss":
		"  (TLB miss)":
		"");

	pr_info("fault address = %#08x\n", dsp_fault_adr);

	if (dsp_cfgstat_get_stat() == CFGSTAT_READY)
		dsp_err_set(ERRCODE_MMU, (unsigned long)dsp_fault_adr);
	else {
		pr_info("Resetting DSP...\n");
		dsp_cpustat_request(CPUSTAT_RESET);
	}

	omap_mmu_disable(&dsp_mmu);
	omap_mmu_write_reg(&dsp_mmu, status, DSP_MMU_IRQSTATUS);
	omap_mmu_enable(&dsp_mmu, 0);

	enable_irq(omap_dsp->mmu_irq);
}
#endif

static DECLARE_WORK(mmu_int_work, do_mmu_int);

#ifdef CONFIG_ARCH_OMAP1
static int dsp_mmu_itack(void)
{
	unsigned long dspadr;

	pr_info("omapdsp: sending DSP MMU interrupt ack.\n");
	if (!dsp_err_isset(ERRCODE_MMU)) {
		printk(KERN_ERR "omapdsp: DSP MMU error has not been set.\n");
		return -EINVAL;
	}
	dspadr = dsp_fault_adr & ~(SZ_4K-1);
	/* FIXME: reserve TLB entry for this */
	omap_mmu_exmap(&dsp_mmu, dspadr, 0, SZ_4K, EXMAP_TYPE_MEM);
	pr_info("omapdsp: falling into recovery runlevel...\n");
	dsp_set_runlevel(RUNLEVEL_RECOVERY);
	__dsp_mmu_itack(&dsp_mmu);
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

#endif /* __PLAT_OMAP_DSP_MMU_H */
