/*
 * linux/arch/arm/mach-omap2/mmu.c
 *
 * Support for non-MPU OMAP2 MMUs.
 *
 * Copyright (C) 2002-2007 Nokia Corporation
 *
 * Written by Toshihiro Kobayashi <toshihiro.kobayashi@nokia.com>
 *        and Paul Mundt <paul.mundt@nokia.com>
 *
 * TWL support: Hiroshi DOYU <Hiroshi.DOYU@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
#include <linux/types.h>
#include <linux/init.h>
#include <linux/rwsem.h>
#include <linux/device.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include "mmu.h"
#include <asm/arch/mmu.h>
#include <asm/tlbflush.h>
#include <asm/io.h>
#include <asm/sizes.h>

static void *dspvect_page;
#define DSP_INIT_PAGE	0xfff000

static inline void
omap2_mmu_read_tlb(struct omap_mmu *mmu, struct cam_ram_regset *cr)
{
	cr->cam = omap_mmu_read_reg(mmu, MMU_READ_CAM);
	cr->ram = omap_mmu_read_reg(mmu, MMU_READ_RAM);
}

static inline void
omap2_mmu_load_tlb(struct omap_mmu *mmu, struct cam_ram_regset *cr)
{
	/* Set the CAM and RAM entries */
	omap_mmu_write_reg(mmu, cr->cam | OMAP_MMU_CAM_V, MMU_CAM);
	omap_mmu_write_reg(mmu, cr->ram, MMU_RAM);
}

static void exmap_setup_iomap_page(struct omap_mmu *mmu, unsigned long phys,
				   unsigned long dsp_io_adr, int index)
{
	unsigned long dspadr;
	void *virt;
	struct omap_mmu_tlb_entry tlb_ent;

	dspadr = (IOMAP_VAL << 18) + (dsp_io_adr << 1);
	virt = omap_mmu_to_virt(mmu, dspadr);
	exmap_set_armmmu((unsigned long)virt, phys, PAGE_SIZE);
	INIT_EXMAP_TBL_ENTRY_4KB_PRESERVED(mmu->exmap_tbl + index, NULL, virt);
	INIT_TLB_ENTRY_4KB_ES32_PRESERVED(&tlb_ent, dspadr, phys);
	omap_mmu_load_pte_entry(mmu, &tlb_ent);
}

static void exmap_clear_iomap_page(struct omap_mmu *mmu,
				   unsigned long dsp_io_adr)
{
	unsigned long dspadr;
	void *virt;

	dspadr = (IOMAP_VAL << 18) + (dsp_io_adr << 1);
	virt = omap_mmu_to_virt(mmu, dspadr);
	exmap_clear_armmmu((unsigned long)virt, PAGE_SIZE);
	/* DSP MMU is shutting down. not handled here. */
}

#define OMAP24XX_MAILBOX_BASE	(L4_24XX_BASE + 0x94000)
#define OMAP2420_GPT5_BASE	(L4_24XX_BASE + 0x7c000)
#define OMAP2420_GPT6_BASE	(L4_24XX_BASE + 0x7e000)
#define OMAP2420_GPT7_BASE	(L4_24XX_BASE + 0x80000)
#define OMAP2420_GPT8_BASE	(L4_24XX_BASE + 0x82000)
#define OMAP24XX_EAC_BASE	(L4_24XX_BASE + 0x90000)
#define OMAP24XX_STI_BASE	(L4_24XX_BASE + 0x68000)
#define OMAP24XX_STI_CH_BASE	(L4_24XX_BASE + 0x0c000000)

static int exmap_setup_preserved_entries(struct omap_mmu *mmu)
{
	int i, n = 0;

	exmap_setup_preserved_mem_page(mmu, dspvect_page, DSP_INIT_PAGE, n++);

	/* REVISIT: This will need to be revisited for 3430 */
	exmap_setup_iomap_page(mmu, OMAP2_PRCM_BASE, 0x7000, n++);
	exmap_setup_iomap_page(mmu, OMAP24XX_MAILBOX_BASE, 0x11000, n++);

	if (cpu_is_omap2420()) {
		exmap_setup_iomap_page(mmu, OMAP2420_GPT5_BASE, 0xe000, n++);
		exmap_setup_iomap_page(mmu, OMAP2420_GPT6_BASE, 0xe800, n++);
		exmap_setup_iomap_page(mmu, OMAP2420_GPT7_BASE, 0xf000, n++);
		exmap_setup_iomap_page(mmu, OMAP2420_GPT8_BASE, 0xf800, n++);
		exmap_setup_iomap_page(mmu, OMAP24XX_EAC_BASE,  0x10000, n++);
		exmap_setup_iomap_page(mmu, OMAP24XX_STI_BASE, 0xc800, n++);
		for (i = 0; i < 5; i++)
			exmap_setup_preserved_mem_page(mmu,
				__va(OMAP24XX_STI_CH_BASE + i*SZ_4K),
				0xfb0000 + i*SZ_4K, n++);
	}

	return n;
}

static void exmap_clear_preserved_entries(struct omap_mmu *mmu)
{
	int i;

	exmap_clear_iomap_page(mmu, 0x7000);	/* PRCM registers */
	exmap_clear_iomap_page(mmu, 0x11000);	/* MAILBOX registers */

	if (cpu_is_omap2420()) {
		exmap_clear_iomap_page(mmu, 0xe000);	/* GPT5 */
		exmap_clear_iomap_page(mmu, 0xe800);	/* GPT6 */
		exmap_clear_iomap_page(mmu, 0xf000);	/* GPT7 */
		exmap_clear_iomap_page(mmu, 0xf800);	/* GPT8 */
		exmap_clear_iomap_page(mmu, 0x10000);	/* EAC */
		exmap_clear_iomap_page(mmu, 0xc800);	/* STI */
		for (i = 0; i < 5; i++)			/* STI CH */
			exmap_clear_mem_page(mmu, 0xfb0000 + i*SZ_4K);
	}

	exmap_clear_mem_page(mmu, DSP_INIT_PAGE);
}

#define MMU_IRQ_MASK \
	(OMAP_MMU_IRQ_MULTIHITFAULT | \
	 OMAP_MMU_IRQ_TABLEWALKFAULT | \
	 OMAP_MMU_IRQ_EMUMISS | \
	 OMAP_MMU_IRQ_TRANSLATIONFAULT)

static int omap2_mmu_startup(struct omap_mmu *mmu)
{
	dspvect_page = (void *)__get_dma_pages(GFP_KERNEL, 0);
	if (dspvect_page == NULL) {
		printk(KERN_ERR "MMU: failed to allocate memory "
				"for dsp vector table\n");
		return -ENOMEM;
	}

	mmu->nr_exmap_preserved = exmap_setup_preserved_entries(mmu);

	omap_mmu_write_reg(mmu, MMU_IRQ_MASK, MMU_IRQENABLE);

	return 0;
}

static void omap2_mmu_shutdown(struct omap_mmu *mmu)
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

static ssize_t omap2_mmu_show(struct omap_mmu *mmu, char *buf,
			      struct omap_mmu_tlb_lock *tlb_lock)
{
	int i, len;

	len = sprintf(buf, "P: preserved, V: valid\n"
			   "B: big endian, L:little endian, "
			   "M: mixed page attribute\n"
			   "ety P V size   cam_va     ram_pa E ES M\n");
			 /* 00: P V  4KB 0x300000 0x10171800 B 16 M */

	for (i = 0; i < mmu->nr_tlb_entries; i++) {
		struct omap_mmu_tlb_entry ent;
		struct cam_ram_regset cr;
		struct omap_mmu_tlb_lock entry_lock;
		char *pgsz_str, *elsz_str;

		/* read a TLB entry */
		entry_lock.base   = tlb_lock->base;
		entry_lock.victim = i;
		omap_mmu_read_tlb(mmu, &entry_lock, &cr);

		ent.pgsz   = cr.cam & OMAP_MMU_CAM_PAGESIZE_MASK;
		ent.prsvd  = cr.cam & OMAP_MMU_CAM_P;
		ent.valid  = cr.cam & OMAP_MMU_CAM_V;
		ent.va     = cr.cam & OMAP_MMU_CAM_VATAG_MASK;
		ent.endian = cr.ram & OMAP_MMU_RAM_ENDIANNESS;
		ent.elsz   = cr.ram & OMAP_MMU_RAM_ELEMENTSIZE_MASK;
		ent.pa     = cr.ram & OMAP_MMU_RAM_PADDR_MASK;
		ent.mixed  = cr.ram & OMAP_MMU_RAM_MIXED;

		pgsz_str = (ent.pgsz == OMAP_MMU_CAM_PAGESIZE_16MB) ? "64MB":
			   (ent.pgsz == OMAP_MMU_CAM_PAGESIZE_1MB)  ? " 1MB":
			   (ent.pgsz == OMAP_MMU_CAM_PAGESIZE_64KB) ? "64KB":
			   (ent.pgsz == OMAP_MMU_CAM_PAGESIZE_4KB)  ? " 4KB":
								     " ???";
		elsz_str = (ent.elsz == OMAP_MMU_RAM_ELEMENTSIZE_8)  ? " 8":
			   (ent.elsz == OMAP_MMU_RAM_ELEMENTSIZE_16) ? "16":
			   (ent.elsz == OMAP_MMU_RAM_ELEMENTSIZE_32) ? "32":
								      "??";

		if (i == tlb_lock->base)
			len += sprintf(buf + len, "lock base = %d\n",
				       tlb_lock->base);
		if (i == tlb_lock->victim)
			len += sprintf(buf + len, "victim    = %d\n",
				       tlb_lock->victim);

		len += sprintf(buf + len,
			       /* 00: P V  4KB 0x300000 0x10171800 B 16 M */
			       "%02d: %c %c %s 0x%06lx 0x%08lx %c %s %c\n",
			       i,
			       ent.prsvd ? 'P' : ' ',
			       ent.valid ? 'V' : ' ',
			       pgsz_str, ent.va, ent.pa,
			       ent.endian ? 'B' : 'L',
			       elsz_str,
			       ent.mixed ? 'M' : ' ');
	}

	return len;
}

#define get_cam_va_mask(pgsz) \
	(((pgsz) == OMAP_MMU_CAM_PAGESIZE_16MB) ? 0xff000000 : \
	 ((pgsz) == OMAP_MMU_CAM_PAGESIZE_1MB)  ? 0xfff00000 : \
	 ((pgsz) == OMAP_MMU_CAM_PAGESIZE_64KB) ? 0xffff0000 : \
	 ((pgsz) == OMAP_MMU_CAM_PAGESIZE_4KB)  ? 0xfffff000 : 0)

static inline unsigned long omap2_mmu_cam_va(struct cam_ram_regset *cr)
{
	unsigned int page_size = cr->cam & OMAP_MMU_CAM_PAGESIZE_MASK;
	unsigned int mask = get_cam_va_mask(cr->cam & page_size);

	return cr->cam & mask;
}

static struct cam_ram_regset *
omap2_mmu_cam_ram_alloc(struct omap_mmu_tlb_entry *entry)
{
	struct cam_ram_regset *cr;

	if (entry->va & ~(get_cam_va_mask(entry->pgsz))) {
		printk(KERN_ERR "MMU: mapping vadr (0x%06lx) is not on an "
		       "aligned boundary\n", entry->va);
		return ERR_PTR(-EINVAL);
	}

	cr = kmalloc(sizeof(struct cam_ram_regset), GFP_KERNEL);

	cr->cam = (entry->va & OMAP_MMU_CAM_VATAG_MASK) |
		  entry->prsvd | entry->pgsz;
	cr->ram = entry->pa | entry->endian | entry->elsz;

	return cr;
}

static inline int omap2_mmu_cam_ram_valid(struct cam_ram_regset *cr)
{
	return cr->cam & OMAP_MMU_CAM_V;
}

static void omap2_mmu_interrupt(struct omap_mmu *mmu)
{
	unsigned long status, va;

	status = MMU_IRQ_MASK & omap_mmu_read_reg(mmu, MMU_IRQSTATUS);
	va = omap_mmu_read_reg(mmu, MMU_FAULT_AD);

	pr_info("%s\n", (status & OMAP_MMU_IRQ_MULTIHITFAULT)		? "multi hit":"");
	pr_info("%s\n", (status & OMAP_MMU_IRQ_TABLEWALKFAULT)		? "table walk fault":"");
	pr_info("%s\n", (status & OMAP_MMU_IRQ_EMUMISS)			? "EMU miss":"");
	pr_info("%s\n", (status & OMAP_MMU_IRQ_TRANSLATIONFAULT)	? "translation fault":"");
	pr_info("%s\n", (status & OMAP_MMU_IRQ_TLBMISS)			? "TLB miss":"");
	pr_info("fault address = %#08lx\n", va);

	omap_mmu_disable(mmu);
	omap_mmu_write_reg(mmu, status, MMU_IRQSTATUS);

	mmu->fault_address = va;
	schedule_work(&mmu->irq_work);
}

static pgprot_t omap2_mmu_pte_get_attr(struct omap_mmu_tlb_entry *entry)
{
	u32 attr;

	attr = entry->mixed << 5;
	attr |= entry->endian;
	attr |= entry->elsz >> 3;
	attr <<= ((entry->pgsz & OMAP_MMU_CAM_PAGESIZE_4KB) ? 0:6);

	return attr;
}

struct omap_mmu_ops omap2_mmu_ops = {
	.startup	= omap2_mmu_startup,
	.shutdown	= omap2_mmu_shutdown,
	.read_tlb	= omap2_mmu_read_tlb,
	.load_tlb	= omap2_mmu_load_tlb,
	.show		= omap2_mmu_show,
	.cam_va		= omap2_mmu_cam_va,
	.cam_ram_alloc	= omap2_mmu_cam_ram_alloc,
	.cam_ram_valid	= omap2_mmu_cam_ram_valid,
	.interrupt	= omap2_mmu_interrupt,
	.pte_get_attr	= omap2_mmu_pte_get_attr,
};
EXPORT_SYMBOL_GPL(omap2_mmu_ops);

MODULE_LICENSE("GPL");
