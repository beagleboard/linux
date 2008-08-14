/*
 * linux/arch/arm/mach-omap1/mmu.c
 *
 * Support for non-MPU OMAP1 MMUs.
 *
 * Copyright (C) 2002-2005 Nokia Corporation
 *
 * Written by Toshihiro Kobayashi <toshihiro.kobayashi@nokia.com>
 *        and Paul Mundt <paul.mundt@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/types.h>
#include <linux/init.h>
#include <linux/rwsem.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include "mmu.h"
#include <asm/tlbflush.h>
#include <mach/dsp_common.h>

static void *dspvect_page;
#define DSP_INIT_PAGE	0xfff000

#define MMUFAULT_MASK (OMAP_MMU_FAULT_ST_PERM |\
		       OMAP_MMU_FAULT_ST_TLB_MISS |\
		       OMAP_MMU_FAULT_ST_TRANS)

static unsigned int get_cam_l_va_mask(u16 pgsz)
{
	switch (pgsz) {
	case OMAP_MMU_CAM_PAGESIZE_1MB:
		return OMAP_MMU_CAM_L_VA_TAG_L1_MASK |
		       OMAP_MMU_CAM_L_VA_TAG_L2_MASK_1MB;
	case OMAP_MMU_CAM_PAGESIZE_64KB:
		return OMAP_MMU_CAM_L_VA_TAG_L1_MASK |
		       OMAP_MMU_CAM_L_VA_TAG_L2_MASK_64KB;
	case OMAP_MMU_CAM_PAGESIZE_4KB:
		return OMAP_MMU_CAM_L_VA_TAG_L1_MASK |
		       OMAP_MMU_CAM_L_VA_TAG_L2_MASK_4KB;
	case OMAP_MMU_CAM_PAGESIZE_1KB:
		return OMAP_MMU_CAM_L_VA_TAG_L1_MASK |
		       OMAP_MMU_CAM_L_VA_TAG_L2_MASK_1KB;
	}
	return 0;
}

#define get_cam_va_mask(pgsz) \
	((u32)OMAP_MMU_CAM_H_VA_TAG_H_MASK << 22 | \
	 (u32)get_cam_l_va_mask(pgsz) << 6)

static int intmem_usecount;

/* for safety */
void dsp_mem_usecount_clear(void)
{
	if (intmem_usecount != 0) {
		printk(KERN_WARNING
		       "MMU: unbalanced memory request/release detected.\n"
		       "         intmem_usecount is not zero at where "
		       "it should be! ... fixed to be zero.\n");
		intmem_usecount = 0;
		omap_dsp_release_mem();
	}
}
EXPORT_SYMBOL_GPL(dsp_mem_usecount_clear);

void omap_mmu_itack(struct omap_mmu *mmu)
{
	omap_mmu_write_reg(mmu, OMAP_MMU_IT_ACK_IT_ACK, OMAP_MMU_IT_ACK);
}
EXPORT_SYMBOL(omap_mmu_itack);

static int omap1_mmu_mem_enable(struct omap_mmu *mmu, void *addr)
{
	int ret = 0;

	if (omap_mmu_internal_memory(mmu, addr)) {
		if (intmem_usecount++ == 0)
			ret = omap_dsp_request_mem();
	}

	return ret;
}

static int omap1_mmu_mem_disable(struct omap_mmu *mmu, void *addr)
{
	int ret = 0;

	if (omap_mmu_internal_memory(mmu, addr)) {
		if (--intmem_usecount == 0)
			omap_dsp_release_mem();
	} else
		ret = -EIO;

	return ret;
}

static inline void
omap1_mmu_read_tlb(struct omap_mmu *mmu, struct cam_ram_regset *cr)
{
	/* read a TLB entry */
	omap_mmu_write_reg(mmu, OMAP_MMU_LD_TLB_RD, OMAP_MMU_LD_TLB);

	cr->cam_h = omap_mmu_read_reg(mmu, OMAP_MMU_READ_CAM_H);
	cr->cam_l = omap_mmu_read_reg(mmu, OMAP_MMU_READ_CAM_L);
	cr->ram_h = omap_mmu_read_reg(mmu, OMAP_MMU_READ_RAM_H);
	cr->ram_l = omap_mmu_read_reg(mmu, OMAP_MMU_READ_RAM_L);
}

static inline void
omap1_mmu_load_tlb(struct omap_mmu *mmu, struct cam_ram_regset *cr)
{
	/* Set the CAM and RAM entries */
	omap_mmu_write_reg(mmu, cr->cam_h, OMAP_MMU_CAM_H);
	omap_mmu_write_reg(mmu, cr->cam_l, OMAP_MMU_CAM_L);
	omap_mmu_write_reg(mmu, cr->ram_h, OMAP_MMU_RAM_H);
	omap_mmu_write_reg(mmu, cr->ram_l, OMAP_MMU_RAM_L);
}

static ssize_t omap1_mmu_show(struct omap_mmu *mmu, char *buf,
			      struct omap_mmu_tlb_lock *tlb_lock)
{
	int i, len;

	len = sprintf(buf, "P: preserved, V: valid\n"
			   "ety P V size   cam_va     ram_pa ap\n");
			 /* 00: P V  4KB 0x300000 0x10171800 FA */

	for (i = 0; i < mmu->nr_tlb_entries; i++) {
		struct omap_mmu_tlb_entry ent;
		struct cam_ram_regset cr;
		struct omap_mmu_tlb_lock entry_lock;
		char *pgsz_str, *ap_str;

		/* read a TLB entry */
		entry_lock.base   = tlb_lock->base;
		entry_lock.victim = i;
		omap_mmu_read_tlb(mmu, &entry_lock, &cr);

		ent.pgsz  = cr.cam_l & OMAP_MMU_CAM_PAGESIZE_MASK;
		ent.prsvd = cr.cam_l & OMAP_MMU_CAM_P;
		ent.valid = cr.cam_l & OMAP_MMU_CAM_V;
		ent.ap    = cr.ram_l & OMAP_MMU_RAM_L_AP_MASK;
		ent.va = (u32)(cr.cam_h & OMAP_MMU_CAM_H_VA_TAG_H_MASK) << 22 |
			 (u32)(cr.cam_l & get_cam_l_va_mask(ent.pgsz)) << 6;
		ent.pa = (unsigned long)cr.ram_h << 16 |
			 (cr.ram_l & OMAP_MMU_RAM_L_RAM_LSB_MASK);

		pgsz_str = (ent.pgsz == OMAP_MMU_CAM_PAGESIZE_1MB)  ? " 1MB":
			   (ent.pgsz == OMAP_MMU_CAM_PAGESIZE_64KB) ? "64KB":
			   (ent.pgsz == OMAP_MMU_CAM_PAGESIZE_4KB)  ? " 4KB":
			   (ent.pgsz == OMAP_MMU_CAM_PAGESIZE_1KB)  ? " 1KB":
								     " ???";
		ap_str = (ent.ap == OMAP_MMU_RAM_L_AP_RO) ? "RO":
			 (ent.ap == OMAP_MMU_RAM_L_AP_FA) ? "FA":
			 (ent.ap == OMAP_MMU_RAM_L_AP_NA) ? "NA":
							   "??";

		if (i == tlb_lock->base)
			len += sprintf(buf + len, "lock base = %d\n",
				       tlb_lock->base);
		if (i == tlb_lock->victim)
			len += sprintf(buf + len, "victim    = %d\n",
				       tlb_lock->victim);
		len += sprintf(buf + len,
			       /* 00: P V  4KB 0x300000 0x10171800 FA */
			       "%02d: %c %c %s 0x%06lx 0x%08lx %s\n",
			       i,
			       ent.prsvd ? 'P' : ' ',
			       ent.valid ? 'V' : ' ',
			       pgsz_str, ent.va, ent.pa, ap_str);
	}

	return len;
}

static int exmap_setup_preserved_entries(struct omap_mmu *mmu)
{
	int n = 0;

	exmap_setup_preserved_mem_page(mmu, dspvect_page, DSP_INIT_PAGE, n++);

	return n;
}

static void exmap_clear_preserved_entries(struct omap_mmu *mmu)
{
	exmap_clear_mem_page(mmu, DSP_INIT_PAGE);
}

static int omap1_mmu_startup(struct omap_mmu *mmu)
{
	dspvect_page = (void *)__get_dma_pages(GFP_KERNEL, 0);
	if (dspvect_page == NULL) {
		dev_err(mmu->dev, "MMU %s: failed to allocate memory "
			"for vector table\n", mmu->name);
		return -ENOMEM;
	}

	mmu->nr_exmap_preserved = exmap_setup_preserved_entries(mmu);

	return 0;
}

static void omap1_mmu_shutdown(struct omap_mmu *mmu)
{
	exmap_clear_preserved_entries(mmu);

	if (dspvect_page != NULL) {
		unsigned long virt;

		down_read(&mmu->exmap_sem);

		virt = (unsigned long)omap_mmu_to_virt(mmu, DSP_INIT_PAGE);
		flush_tlb_kernel_range(virt, virt + PAGE_SIZE);
		free_page((unsigned long)dspvect_page);
		dspvect_page = NULL;

		up_read(&mmu->exmap_sem);
	}
}

static inline unsigned long omap1_mmu_cam_va(struct cam_ram_regset *cr)
{
	unsigned int page_size = cr->cam_l & OMAP_MMU_CAM_PAGESIZE_MASK;

	return (u32)(cr->cam_h & OMAP_MMU_CAM_H_VA_TAG_H_MASK)  << 22 |
	       (u32)(cr->cam_l & get_cam_l_va_mask(page_size)) << 6;
}

static struct cam_ram_regset *
omap1_mmu_cam_ram_alloc(struct omap_mmu *mmu, struct omap_mmu_tlb_entry *entry)
{
	struct cam_ram_regset *cr;

	if (entry->va & ~(get_cam_va_mask(entry->pgsz))) {
		dev_err(mmu->dev, "MMU %s: mapping vadr (0x%06lx) is not on"
			" an aligned boundary\n", mmu->name, entry->va);
		return ERR_PTR(-EINVAL);
	}

	cr = kmalloc(sizeof(struct cam_ram_regset), GFP_KERNEL);

	cr->cam_h = entry->va >> 22;
	cr->cam_l = (entry->va >> 6 & get_cam_l_va_mask(entry->pgsz)) |
		   entry->prsvd | entry->pgsz;
	cr->ram_h = entry->pa >> 16;
	cr->ram_l = (entry->pa & OMAP_MMU_RAM_L_RAM_LSB_MASK) | entry->ap;

	return cr;
}

static inline int omap1_mmu_cam_ram_valid(struct cam_ram_regset *cr)
{
	return cr->cam_l & OMAP_MMU_CAM_V;
}

static void omap1_mmu_interrupt(struct omap_mmu *mmu)
{
	unsigned long status;
	unsigned long adh, adl;
	unsigned long dp;
	unsigned long va;

	status = omap_mmu_read_reg(mmu, OMAP_MMU_FAULT_ST);
	adh = omap_mmu_read_reg(mmu, OMAP_MMU_FAULT_AD_H);
	adl = omap_mmu_read_reg(mmu, OMAP_MMU_FAULT_AD_L);
	dp = adh & OMAP_MMU_FAULT_AD_H_DP;
	va = (((adh & OMAP_MMU_FAULT_AD_H_ADR_MASK) << 16) | adl);

	/* if the fault is masked, nothing to do */
	if ((status & MMUFAULT_MASK) == 0) {
		pr_debug("MMU interrupt, but ignoring.\n");
		/*
		 * note: in OMAP1710,
		 * when CACHE + DMA domain gets out of idle in DSP,
		 * MMU interrupt occurs but MMU_FAULT_ST is not set.
		 * in this case, we just ignore the interrupt.
		 */
		if (status) {
			pr_debug("%s%s%s%s\n",
				 (status & OMAP_MMU_FAULT_ST_PREF)?
				 "  (prefetch err)" : "",
				 (status & OMAP_MMU_FAULT_ST_PERM)?
				 "  (permission fault)" : "",
				 (status & OMAP_MMU_FAULT_ST_TLB_MISS)?
				 "  (TLB miss)" : "",
				 (status & OMAP_MMU_FAULT_ST_TRANS) ?
				 "  (translation fault)": "");
			pr_debug("fault address = %#08lx\n", va);
		}
		enable_irq(mmu->irq);
		return;
	}

	pr_info("%s%s%s%s\n",
		(status & OMAP_MMU_FAULT_ST_PREF)?
		(MMUFAULT_MASK & OMAP_MMU_FAULT_ST_PREF)?
		"  prefetch err":
		"  (prefetch err)":
		"",
		(status & OMAP_MMU_FAULT_ST_PERM)?
		(MMUFAULT_MASK & OMAP_MMU_FAULT_ST_PERM)?
		"  permission fault":
		"  (permission fault)":
		"",
		(status & OMAP_MMU_FAULT_ST_TLB_MISS)?
		(MMUFAULT_MASK & OMAP_MMU_FAULT_ST_TLB_MISS)?
		"  TLB miss":
		"  (TLB miss)":
		"",
		(status & OMAP_MMU_FAULT_ST_TRANS)?
		(MMUFAULT_MASK & OMAP_MMU_FAULT_ST_TRANS)?
		"  translation fault":
		"  (translation fault)":
		"");
	pr_info("fault address = %#08lx\n", va);

	mmu->fault_address = va;
	schedule_work(&mmu->irq_work);
}

static pgprot_t omap1_mmu_pte_get_attr(struct omap_mmu_tlb_entry *entry)
{
	/* 4KB AP position as default */
	u32 attr = entry->ap >> 4;
	attr <<= ((entry->pgsz == OMAP_MMU_CAM_PAGESIZE_1MB) ? 6:0);
	return attr;
}

struct omap_mmu_ops omap1_mmu_ops = {
	.startup	= omap1_mmu_startup,
	.shutdown	= omap1_mmu_shutdown,
	.mem_enable	= omap1_mmu_mem_enable,
	.mem_disable	= omap1_mmu_mem_disable,
	.read_tlb	= omap1_mmu_read_tlb,
	.load_tlb	= omap1_mmu_load_tlb,
	.show		= omap1_mmu_show,
	.cam_va		= omap1_mmu_cam_va,
	.cam_ram_alloc	= omap1_mmu_cam_ram_alloc,
	.cam_ram_valid	= omap1_mmu_cam_ram_valid,
	.interrupt	= omap1_mmu_interrupt,
	.pte_get_attr	= omap1_mmu_pte_get_attr,
};
EXPORT_SYMBOL_GPL(omap1_mmu_ops);
