/*
 * linux/arch/arm/plat-omap/mmu.c
 *
 * OMAP MMU management framework
 *
 * Copyright (C) 2002-2006 Nokia Corporation
 *
 * Written by Toshihiro Kobayashi <toshihiro.kobayashi@nokia.com>
 *        and Paul Mundt <lethal@linux-sh.org>
 *
 * TWL support: Hiroshi DOYU <Hiroshi.DOYU@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/mempool.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <asm/pgalloc.h>
#include <asm/pgtable.h>
#include <asm/arch/mmu.h>
#include <asm/sizes.h>
#include <asm/arch/dsp_common.h>

#if defined(CONFIG_ARCH_OMAP1)
#include "../mach-omap1/mmu.h"
#elif defined(CONFIG_ARCH_OMAP2)
#include "../mach-omap2/mmu.h"
#endif

/*
 * On OMAP2 MMU_LOCK_xxx_MASK only applies to the IVA and DSP, the camera
 * MMU has base and victim implemented in different bits in the LOCK
 * register (shifts are still the same), all of the other registers are
 * the same on all of the MMUs..
 */
#define MMU_LOCK_BASE_SHIFT		10
#define MMU_LOCK_VICTIM_SHIFT		4

#define CAMERA_MMU_LOCK_BASE_MASK	(0x7 << MMU_LOCK_BASE_SHIFT)
#define CAMERA_MMU_LOCK_VICTIM_MASK	(0x7 << MMU_LOCK_VICTIM_SHIFT)

#define is_aligned(adr, align)	(!((adr)&((align)-1)))
#define ORDER_1MB	(20 - PAGE_SHIFT)
#define ORDER_64KB	(16 - PAGE_SHIFT)
#define ORDER_4KB	(12 - PAGE_SHIFT)

#define MMU_CNTL_EMUTLBUPDATE	(1<<3)
#define MMU_CNTL_TWLENABLE	(1<<2)
#define MMU_CNTL_MMUENABLE	(1<<1)

static mempool_t *mempool_1M;
static mempool_t *mempool_64K;

#define omap_mmu_for_each_tlb_entry(mmu, entry)			\
	for (entry = mmu->exmap_tbl; prefetch(entry + 1),	\
	     entry < (mmu->exmap_tbl + mmu->nr_tlb_entries);	\
	     entry++)

#define to_dev(obj)	container_of(obj, struct device, kobj)

static void *mempool_alloc_from_pool(mempool_t *pool,
				     unsigned int __nocast gfp_mask)
{
	spin_lock_irq(&pool->lock);
	if (likely(pool->curr_nr)) {
		void *element = pool->elements[--pool->curr_nr];
		spin_unlock_irq(&pool->lock);
		return element;
	}

	spin_unlock_irq(&pool->lock);
	return mempool_alloc(pool, gfp_mask);
}

/*
 * kmem_reserve(), kmem_release():
 * reserve or release kernel memory for exmap().
 *
 * exmap() might request consecutive 1MB or 64kB,
 * but it will be difficult after memory pages are fragmented.
 * So, user can reserve such memory blocks in the early phase
 * through kmem_reserve().
 */
static void *omap_mmu_pool_alloc(unsigned int __nocast gfp, void *order)
{
	return (void *)__get_dma_pages(gfp, (unsigned int)order);
}

static void omap_mmu_pool_free(void *buf, void *order)
{
	free_pages((unsigned long)buf, (unsigned int)order);
}

int omap_mmu_kmem_reserve(struct omap_mmu *mmu, unsigned long size)
{
	unsigned long len = size;

	/* alignment check */
	if (!is_aligned(size, SZ_64K)) {
		dev_err(mmu->dev,
			"MMU %s: size(0x%lx) is not multiple of 64KB.\n",
			mmu->name, size);
		return -EINVAL;
	}

	if (size > (1 << mmu->addrspace)) {
		dev_err(mmu->dev,
			"MMU %s: size(0x%lx) is larger than external device "
			" memory space size (0x%x.\n", mmu->name, size,
			(1 << mmu->addrspace));
		return -EINVAL;
	}

	if (size >= SZ_1M) {
		int nr = size >> 20;

		if (likely(!mempool_1M))
			mempool_1M = mempool_create(nr, omap_mmu_pool_alloc,
						    omap_mmu_pool_free,
						    (void *)ORDER_1MB);
		else
			mempool_resize(mempool_1M, mempool_1M->min_nr + nr,
				       GFP_KERNEL);

		size &= ~(0xf << 20);
	}

	if (size >= SZ_64K) {
		int nr = size >> 16;

		if (likely(!mempool_64K))
			mempool_64K = mempool_create(nr, omap_mmu_pool_alloc,
						     omap_mmu_pool_free,
						     (void *)ORDER_64KB);
		else
			mempool_resize(mempool_64K, mempool_64K->min_nr + nr,
				       GFP_KERNEL);

		size &= ~(0xf << 16);
	}

	if (size)
		len -= size;

	return len;
}
EXPORT_SYMBOL_GPL(omap_mmu_kmem_reserve);

void omap_mmu_kmem_release(void)
{
	if (mempool_64K) {
		mempool_destroy(mempool_64K);
		mempool_64K = NULL;
	}

	if (mempool_1M) {
		mempool_destroy(mempool_1M);
		mempool_1M = NULL;
	}
}
EXPORT_SYMBOL_GPL(omap_mmu_kmem_release);

static void omap_mmu_free_pages(unsigned long buf, unsigned int order)
{
	struct page *page, *ps, *pe;

	ps = virt_to_page(buf);
	pe = virt_to_page(buf + (1 << (PAGE_SHIFT + order)));

	for (page = ps; page < pe; page++)
		ClearPageReserved(page);

	if ((order == ORDER_64KB) && likely(mempool_64K))
		mempool_free((void *)buf, mempool_64K);
	else if ((order == ORDER_1MB) && likely(mempool_1M))
		mempool_free((void *)buf, mempool_1M);
	else
		free_pages(buf, order);
}

/*
 * ARM MMU operations
 */
int exmap_set_armmmu(struct omap_mmu *mmu, unsigned long virt,
		     unsigned long phys, unsigned long size)
{
	long off;
	unsigned long sz_left;
	pmd_t *pmdp;
	pte_t *ptep;
	int prot_pmd, prot_pte;

	dev_dbg(mmu->dev,
		"MMU %s: mapping in ARM MMU, v=0x%08lx, p=0x%08lx, sz=0x%lx\n",
		mmu->name, virt, phys, size);

	prot_pmd = PMD_TYPE_TABLE | PMD_DOMAIN(DOMAIN_IO);
	prot_pte = L_PTE_PRESENT | L_PTE_YOUNG | L_PTE_DIRTY | L_PTE_WRITE;

	pmdp = pmd_offset(pgd_offset_k(virt), virt);
	if (pmd_none(*pmdp)) {
		ptep = pte_alloc_one_kernel(&init_mm, 0);
		if (ptep == NULL)
			return -ENOMEM;
		/* note: two PMDs will be set  */
		pmd_populate_kernel(&init_mm, pmdp, ptep);
	}

	off = phys - virt;
	for (sz_left = size;
	     sz_left >= PAGE_SIZE;
	     sz_left -= PAGE_SIZE, virt += PAGE_SIZE) {
		ptep = pte_offset_kernel(pmdp, virt);
		set_pte_ext(ptep, __pte((virt + off) | prot_pte), 0);
	}
	if (sz_left)
		BUG();

	return 0;
}
EXPORT_SYMBOL_GPL(exmap_set_armmmu);

void exmap_clear_armmmu(struct omap_mmu *mmu, unsigned long virt,
			unsigned long size)
{
	unsigned long sz_left;
	pmd_t *pmdp;
	pte_t *ptep;

	dev_dbg(mmu->dev,
		"MMU %s: unmapping in ARM MMU, v=0x%08lx, sz=0x%lx\n",
		mmu->name, virt, size);

	for (sz_left = size;
	     sz_left >= PAGE_SIZE;
	     sz_left -= PAGE_SIZE, virt += PAGE_SIZE) {
		pmdp = pmd_offset(pgd_offset_k(virt), virt);
		ptep = pte_offset_kernel(pmdp, virt);
		pte_clear(&init_mm, virt, ptep);
	}
	if (sz_left)
		BUG();
}
EXPORT_SYMBOL_GPL(exmap_clear_armmmu);

int exmap_valid(struct omap_mmu *mmu, void *vadr, size_t len)
{
	/* exmap_sem should be held before calling this function */
	struct exmap_tbl *ent;

start:
	omap_mmu_for_each_tlb_entry(mmu, ent) {
		void *mapadr;
		unsigned long mapsize;

		if (!ent->valid)
			continue;
		mapadr = (void *)ent->vadr;
		mapsize = 1 << (ent->order + PAGE_SHIFT);
		if ((vadr >= mapadr) && (vadr < mapadr + mapsize)) {
			if (vadr + len <= mapadr + mapsize) {
				/* this map covers whole address. */
				return 1;
			} else {
				/*
				 * this map covers partially.
				 * check rest portion.
				 */
				len -= mapadr + mapsize - vadr;
				vadr = mapadr + mapsize;
				goto start;
			}
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(exmap_valid);

/*
 * omap_mmu_exmap_use(), unuse():
 * when the mapped area is exported to user space with mmap,
 * the usecount is incremented.
 * while the usecount > 0, that area can't be released.
 */
void omap_mmu_exmap_use(struct omap_mmu *mmu, void *vadr, size_t len)
{
	struct exmap_tbl *ent;

	down_write(&mmu->exmap_sem);
	omap_mmu_for_each_tlb_entry(mmu, ent) {
		void *mapadr;
		unsigned long mapsize;

		if (!ent->valid)
			continue;
		mapadr = (void *)ent->vadr;
		mapsize = 1 << (ent->order + PAGE_SHIFT);
		if ((vadr + len > mapadr) && (vadr < mapadr + mapsize))
			ent->usecount++;
	}
	up_write(&mmu->exmap_sem);
}
EXPORT_SYMBOL_GPL(omap_mmu_exmap_use);

void omap_mmu_exmap_unuse(struct omap_mmu *mmu, void *vadr, size_t len)
{
	struct exmap_tbl *ent;

	down_write(&mmu->exmap_sem);
	omap_mmu_for_each_tlb_entry(mmu, ent) {
		void *mapadr;
		unsigned long mapsize;

		if (!ent->valid)
			continue;
		mapadr = (void *)ent->vadr;
		mapsize = 1 << (ent->order + PAGE_SHIFT);
		if ((vadr + len > mapadr) && (vadr < mapadr + mapsize))
			ent->usecount--;
	}
	up_write(&mmu->exmap_sem);
}
EXPORT_SYMBOL_GPL(omap_mmu_exmap_unuse);

/*
 * omap_mmu_virt_to_phys()
 * returns physical address, and sets len to valid length
 */
unsigned long
omap_mmu_virt_to_phys(struct omap_mmu *mmu, void *vadr, size_t *len)
{
	struct exmap_tbl *ent;

	if (omap_mmu_internal_memory(mmu, vadr)) {
		unsigned long addr = (unsigned long)vadr;
		*len = mmu->membase + mmu->memsize - addr;
		return addr;
	}

	/* EXRAM */
	omap_mmu_for_each_tlb_entry(mmu, ent) {
		void *mapadr;
		unsigned long mapsize;

		if (!ent->valid)
			continue;
		mapadr = (void *)ent->vadr;
		mapsize = 1 << (ent->order + PAGE_SHIFT);
		if ((vadr >= mapadr) && (vadr < mapadr + mapsize)) {
			*len = mapadr + mapsize - vadr;
			return __pa(ent->buf) + vadr - mapadr;
		}
	}

	/* valid mapping not found */
	return 0;
}
EXPORT_SYMBOL_GPL(omap_mmu_virt_to_phys);

/*
 * PTE operations
 */
static inline void
omap_mmu_alloc_section(struct mm_struct *mm, unsigned long virt,
		       unsigned long phys, int prot)
{
	pmd_t *pmdp = pmd_offset(pgd_offset(mm, virt), virt);
	if (virt & (1 << SECTION_SHIFT))
		pmdp++;
	*pmdp = __pmd((phys & SECTION_MASK) | prot | PMD_TYPE_SECT);
	flush_pmd_entry(pmdp);
}

static inline void
omap_mmu_alloc_supersection(struct mm_struct *mm, unsigned long virt,
			    unsigned long phys, int prot)
{
	int i;
	for (i = 0; i < 16; i += 1) {
		omap_mmu_alloc_section(mm, virt, phys, prot | PMD_SECT_SUPER);
		virt += (PGDIR_SIZE / 2);
	}
}

static inline int
omap_mmu_alloc_page(struct mm_struct *mm, unsigned long virt,
		    unsigned long phys, pgprot_t prot)
{
	pte_t *ptep;
	pmd_t *pmdp = pmd_offset(pgd_offset(mm, virt), virt);

	if (!(prot & PTE_TYPE_MASK))
		prot |= PTE_TYPE_SMALL;

	if (pmd_none(*pmdp)) {
		ptep = pte_alloc_one_kernel(mm, virt);
		if (ptep == NULL)
			return -ENOMEM;
		pmd_populate_kernel(mm, pmdp, ptep);
	}
	ptep = pte_offset_kernel(pmdp, virt);
	ptep -= PTRS_PER_PTE;
	*ptep = pfn_pte(phys >> PAGE_SHIFT, prot);
	flush_pmd_entry((pmd_t *)ptep);
	return 0;
}

static inline int
omap_mmu_alloc_largepage(struct mm_struct *mm, unsigned long virt,
			 unsigned long phys, pgprot_t prot)
{
	int i, ret;
	for (i = 0; i < 16; i += 1) {
		ret = omap_mmu_alloc_page(mm, virt, phys,
					  prot | PTE_TYPE_LARGE);
		if (ret)
			return -ENOMEM; /* only 1st time */
		virt += PAGE_SIZE;
	}
	return 0;
}

static int omap_mmu_load_pte(struct omap_mmu *mmu,
			     struct omap_mmu_tlb_entry *e)
{
	int ret = 0;
	struct mm_struct *mm = mmu->twl_mm;
	const unsigned long va = e->va;
	const unsigned long pa = e->pa;
	const pgprot_t prot = mmu->ops->pte_get_attr(e);

	spin_lock(&mm->page_table_lock);

	switch (e->pgsz) {
	case OMAP_MMU_CAM_PAGESIZE_16MB:
		omap_mmu_alloc_supersection(mm, va, pa, prot);
		break;
	case OMAP_MMU_CAM_PAGESIZE_1MB:
		omap_mmu_alloc_section(mm, va, pa, prot);
		break;
	case OMAP_MMU_CAM_PAGESIZE_64KB:
		ret = omap_mmu_alloc_largepage(mm, va, pa, prot);
		break;
	case OMAP_MMU_CAM_PAGESIZE_4KB:
		ret = omap_mmu_alloc_page(mm, va, pa, prot);
		break;
	default:
		BUG();
		break;
	}

	spin_unlock(&mm->page_table_lock);

	return ret;
}

static void omap_mmu_clear_pte(struct omap_mmu *mmu, unsigned long virt)
{
	pte_t *ptep, *end;
	pmd_t *pmdp;
	struct mm_struct *mm = mmu->twl_mm;

	spin_lock(&mm->page_table_lock);

	pmdp = pmd_offset(pgd_offset(mm, virt), virt);

	if (pmd_none(*pmdp))
		goto out;

	if (!pmd_table(*pmdp))
		goto invalidate_pmd;

	ptep = pte_offset_kernel(pmdp, virt);
	pte_clear(mm, virt, ptep);
	flush_pmd_entry((pmd_t *)ptep);

	/* zap pte */
	end = pmd_page_vaddr(*pmdp);
	ptep = end - PTRS_PER_PTE;
	while (ptep < end) {
		if (!pte_none(*ptep))
			goto out;
		ptep++;
	}
	pte_free_kernel(mm, pmd_page_vaddr(*pmdp));

 invalidate_pmd:
	pmd_clear(pmdp);
	flush_pmd_entry(pmdp);
 out:
	spin_unlock(&mm->page_table_lock);
}

/*
 * TLB operations
 */
static struct cam_ram_regset *
omap_mmu_cam_ram_alloc(struct omap_mmu *mmu, struct omap_mmu_tlb_entry *entry)
{
	return mmu->ops->cam_ram_alloc(mmu, entry);
}

static int omap_mmu_cam_ram_valid(struct omap_mmu *mmu,
				  struct cam_ram_regset *cr)
{
	return mmu->ops->cam_ram_valid(cr);
}

static inline void
omap_mmu_get_tlb_lock(struct omap_mmu *mmu, struct omap_mmu_tlb_lock *tlb_lock)
{
	unsigned long lock = omap_mmu_read_reg(mmu, OMAP_MMU_LOCK);
	int mask;

	mask = (mmu->type == OMAP_MMU_CAMERA) ?
			CAMERA_MMU_LOCK_BASE_MASK : MMU_LOCK_BASE_MASK;
	tlb_lock->base = (lock & mask) >> MMU_LOCK_BASE_SHIFT;

	mask = (mmu->type == OMAP_MMU_CAMERA) ?
			CAMERA_MMU_LOCK_VICTIM_MASK : MMU_LOCK_VICTIM_MASK;
	tlb_lock->victim = (lock & mask) >> MMU_LOCK_VICTIM_SHIFT;
}

static inline void
omap_mmu_set_tlb_lock(struct omap_mmu *mmu, struct omap_mmu_tlb_lock *lock)
{
	omap_mmu_write_reg(mmu,
			   (lock->base << MMU_LOCK_BASE_SHIFT) |
			   (lock->victim << MMU_LOCK_VICTIM_SHIFT),
			   OMAP_MMU_LOCK);
}

static inline void omap_mmu_flush(struct omap_mmu *mmu)
{
	omap_mmu_write_reg(mmu, 0x1, OMAP_MMU_FLUSH_ENTRY);
}

static inline void omap_mmu_ldtlb(struct omap_mmu *mmu)
{
	omap_mmu_write_reg(mmu, 0x1, OMAP_MMU_LD_TLB);
}

void omap_mmu_read_tlb(struct omap_mmu *mmu, struct omap_mmu_tlb_lock *lock,
		       struct cam_ram_regset *cr)
{
	/* set victim */
	omap_mmu_set_tlb_lock(mmu, lock);

	if (likely(mmu->ops->read_tlb))
		mmu->ops->read_tlb(mmu, cr);
}
EXPORT_SYMBOL_GPL(omap_mmu_read_tlb);

void omap_mmu_load_tlb(struct omap_mmu *mmu, struct cam_ram_regset *cr)
{
	if (likely(mmu->ops->load_tlb))
		mmu->ops->load_tlb(mmu, cr);

	/* flush the entry */
	omap_mmu_flush(mmu);

	/* load a TLB entry */
	omap_mmu_ldtlb(mmu);
}

int omap_mmu_load_tlb_entry(struct omap_mmu *mmu,
			    struct omap_mmu_tlb_entry *entry)
{
	struct omap_mmu_tlb_lock lock;
	struct cam_ram_regset *cr;
	int ret;

	clk_enable(mmu->clk);
	ret = omap_dsp_request_mem();
	if (ret < 0)
		goto out;

	omap_mmu_get_tlb_lock(mmu, &lock);
	for (lock.victim = 0; lock.victim < lock.base; lock.victim++) {
		struct cam_ram_regset tmp;

		/* read a TLB entry */
		omap_mmu_read_tlb(mmu, &lock, &tmp);
		if (!omap_mmu_cam_ram_valid(mmu, &tmp))
			goto found_victim;
	}
	omap_mmu_set_tlb_lock(mmu, &lock);

found_victim:
	/* The last entry cannot be locked? */
	if (lock.victim == (mmu->nr_tlb_entries - 1)) {
		dev_err(mmu->dev, "MMU %s: TLB is full.\n", mmu->name);
		return -EBUSY;
	}

	cr = omap_mmu_cam_ram_alloc(mmu, entry);
	if (IS_ERR(cr))
		return PTR_ERR(cr);

	omap_mmu_load_tlb(mmu, cr);
	kfree(cr);

	/* update lock base */
	if (lock.victim == lock.base)
		lock.base++;

	omap_mmu_set_tlb_lock(mmu, &lock);

	omap_dsp_release_mem();
out:
	clk_disable(mmu->clk);
	return 0;
}
EXPORT_SYMBOL_GPL(omap_mmu_load_tlb_entry);

static inline unsigned long
omap_mmu_cam_va(struct omap_mmu *mmu, struct cam_ram_regset *cr)
{
	return mmu->ops->cam_va(cr);
}

int omap_mmu_clear_tlb_entry(struct omap_mmu *mmu, unsigned long vadr)
{
	struct omap_mmu_tlb_lock lock;
	int i, ret = 0;
	int max_valid = 0;

	clk_enable(mmu->clk);
	ret = omap_dsp_request_mem();
	if (ret < 0)
		goto out;

	omap_mmu_get_tlb_lock(mmu, &lock);
	for (i = 0; i < lock.base; i++) {
		struct cam_ram_regset cr;

		/* read a TLB entry */
		lock.victim = i;
		omap_mmu_read_tlb(mmu, &lock, &cr);
		if (!omap_mmu_cam_ram_valid(mmu, &cr))
			continue;

		if (omap_mmu_cam_va(mmu, &cr) == vadr)
			/* flush the entry */
			omap_mmu_flush(mmu);
		else
			max_valid = i;
	}

	/* set new lock base */
	lock.base = lock.victim = max_valid + 1;
	omap_mmu_set_tlb_lock(mmu, &lock);

	omap_dsp_release_mem();
out:
	clk_disable(mmu->clk);
	return ret;
}
EXPORT_SYMBOL_GPL(omap_mmu_clear_tlb_entry);

static void omap_mmu_gflush(struct omap_mmu *mmu)
{
	struct omap_mmu_tlb_lock lock;
	int ret;

	clk_enable(mmu->clk);
	ret = omap_dsp_request_mem();
	if (ret < 0)
		goto out;

	omap_mmu_write_reg(mmu, 0x1, OMAP_MMU_GFLUSH);
	lock.base = lock.victim = mmu->nr_exmap_preserved;
	omap_mmu_set_tlb_lock(mmu, &lock);

	omap_dsp_release_mem();
out:
	clk_disable(mmu->clk);
}

int omap_mmu_load_pte_entry(struct omap_mmu *mmu,
			    struct omap_mmu_tlb_entry *entry)
{
	int ret = -1;
	/*XXX use PG_flag for prsvd */
	ret = omap_mmu_load_pte(mmu, entry);
	if (ret)
		return ret;
	if (entry->tlb)
		ret = omap_mmu_load_tlb_entry(mmu, entry);
	return ret;
}
EXPORT_SYMBOL_GPL(omap_mmu_load_pte_entry);

int omap_mmu_clear_pte_entry(struct omap_mmu *mmu, unsigned long vadr)
{
	int ret = omap_mmu_clear_tlb_entry(mmu, vadr);
	if (ret)
		return ret;
	omap_mmu_clear_pte(mmu, vadr);
	return ret;
}
EXPORT_SYMBOL_GPL(omap_mmu_clear_pte_entry);

/*
 * omap_mmu_exmap()
 *
 * MEM_IOCTL_EXMAP ioctl calls this function with padr=0.
 * In this case, the buffer for external device is allocated in this routine,
 * then it is mapped.
 * On the other hand, for example - frame buffer sharing, calls
 * this function with padr set. It means some known address space
 * pointed with padr is going to be shared with external device.
 */
int omap_mmu_exmap(struct omap_mmu *mmu, unsigned long devadr,
		   unsigned long padr, unsigned long size,
		   enum exmap_type type)
{
	unsigned long pgsz;
	void *buf;
	unsigned int order = 0;
	unsigned long unit;
	int prev = -1;
	unsigned long _devadr = devadr;
	unsigned long _padr = padr;
	void *_vadr = omap_mmu_to_virt(mmu, devadr);
	unsigned long _size = size;
	struct omap_mmu_tlb_entry tlb_ent;
	struct exmap_tbl *exmap_ent, *tmp_ent;
	int status;
	int idx;

#define MINIMUM_PAGESZ	SZ_4K
	/*
	 * alignment check
	 */
	if (!is_aligned(size, MINIMUM_PAGESZ)) {
		dev_err(mmu->dev,
			"MMU %s: size(0x%lx) is not multiple of 4KB.\n",
			mmu->name, size);
		return -EINVAL;
	}
	if (!is_aligned(devadr, MINIMUM_PAGESZ)) {
		dev_err(mmu->dev,
			"MMU %s: external device address(0x%lx) is not"
			" aligned.\n", mmu->name, devadr);
		return -EINVAL;
	}
	if (!is_aligned(padr, MINIMUM_PAGESZ)) {
		dev_err(mmu->dev,
			"MMU %s: physical address(0x%lx) is not aligned.\n",
			mmu->name, padr);
		return -EINVAL;
	}

	/* address validity check */
	if ((devadr < mmu->memsize) ||
	    (devadr >= (1 << mmu->addrspace))) {
		dev_err(mmu->dev,
			"MMU %s: illegal address/size for %s().\n",
			mmu->name, __func__);
		return -EINVAL;
	}

	down_write(&mmu->exmap_sem);

	/* overlap check */
	omap_mmu_for_each_tlb_entry(mmu, tmp_ent) {
		unsigned long mapsize;

		if (!tmp_ent->valid)
			continue;
		mapsize = 1 << (tmp_ent->order + PAGE_SHIFT);
		if ((_vadr + size > tmp_ent->vadr) &&
		    (_vadr < tmp_ent->vadr + mapsize)) {
			dev_err(mmu->dev, "MMU %s: exmap page overlap!\n",
				mmu->name);
			up_write(&mmu->exmap_sem);
			return -EINVAL;
		}
	}

start:
	buf = NULL;
	/* Are there any free TLB lines?  */
	for (idx = 0; idx < mmu->nr_tlb_entries; idx++)
		if (!mmu->exmap_tbl[idx].valid)
			goto found_free;

	dev_err(mmu->dev, "MMU %s: TLB is full.\n", mmu->name);
	status = -EBUSY;
	goto fail;

found_free:
	exmap_ent = mmu->exmap_tbl + idx;

	if ((_size >= SZ_1M) &&
	    (is_aligned(_padr, SZ_1M) || (padr == 0)) &&
	    is_aligned(_devadr, SZ_1M)) {
		unit = SZ_1M;
		pgsz = OMAP_MMU_CAM_PAGESIZE_1MB;
	} else if ((_size >= SZ_64K) &&
		   (is_aligned(_padr, SZ_64K) || (padr == 0)) &&
		   is_aligned(_devadr, SZ_64K)) {
		unit = SZ_64K;
		pgsz = OMAP_MMU_CAM_PAGESIZE_64KB;
	} else {
		unit = SZ_4K;
		pgsz = OMAP_MMU_CAM_PAGESIZE_4KB;
	}

	order = get_order(unit);

	/* buffer allocation */
	if (type == EXMAP_TYPE_MEM) {
		struct page *page, *ps, *pe;

		if ((order == ORDER_1MB) && likely(mempool_1M))
			buf = mempool_alloc_from_pool(mempool_1M, GFP_KERNEL);
		else if ((order == ORDER_64KB) && likely(mempool_64K))
			buf = mempool_alloc_from_pool(mempool_64K, GFP_KERNEL);
		else {
			buf = (void *)__get_dma_pages(GFP_KERNEL, order);
			if (buf == NULL) {
				status = -ENOMEM;
				goto fail;
			}
		}

		/* mark the pages as reserved; this is needed for mmap */
		ps = virt_to_page(buf);
		pe = virt_to_page(buf + unit);

		for (page = ps; page < pe; page++)
			SetPageReserved(page);

		_padr = __pa(buf);
	}

	/*
	 * mapping for ARM MMU:
	 * we should not access to the allocated memory through 'buf'
	 * since this area should not be cached.
	 */
	status = exmap_set_armmmu(mmu, (unsigned long)_vadr, _padr, unit);
	if (status < 0)
		goto fail;

	/* loading external device PTE entry */
	INIT_TLB_ENTRY(&tlb_ent, _devadr, _padr, pgsz);
	status = omap_mmu_load_pte_entry(mmu, &tlb_ent);
	if (status < 0) {
		exmap_clear_armmmu(mmu, (unsigned long)_vadr, unit);
		goto fail;
	}

	INIT_EXMAP_TBL_ENTRY(exmap_ent, buf, _vadr, type, order);
	exmap_ent->link.prev = prev;
	if (prev >= 0)
		mmu->exmap_tbl[prev].link.next = idx;

	if ((_size -= unit) == 0) {	/* normal completion */
		up_write(&mmu->exmap_sem);
		return size;
	}

	_devadr += unit;
	_vadr   += unit;
	_padr = padr ? _padr + unit : 0;
	prev = idx;
	goto start;

fail:
	up_write(&mmu->exmap_sem);
	if (buf)
		omap_mmu_free_pages((unsigned long)buf, order);
	omap_mmu_exunmap(mmu, devadr);
	return status;
}
EXPORT_SYMBOL_GPL(omap_mmu_exmap);

static unsigned long unmap_free_arm(struct omap_mmu *mmu,
				    struct exmap_tbl *ent)
{
	unsigned long size;

	/* clearing ARM MMU */
	size = 1 << (ent->order + PAGE_SHIFT);
	exmap_clear_armmmu(mmu, (unsigned long)ent->vadr, size);

	/* freeing allocated memory */
	if (ent->type == EXMAP_TYPE_MEM) {
		omap_mmu_free_pages((unsigned long)ent->buf, ent->order);
		dev_dbg(mmu->dev, "MMU %s: freeing 0x%lx bytes @ adr 0x%8p\n",
			mmu->name, size, ent->buf);
	}

	ent->valid = 0;
	return size;
}

int omap_mmu_exunmap(struct omap_mmu *mmu, unsigned long devadr)
{
	void *vadr;
	unsigned long size;
	int total = 0;
	struct exmap_tbl *ent;
	int idx;

	vadr = omap_mmu_to_virt(mmu, devadr);
	down_write(&mmu->exmap_sem);
	for (idx = 0; idx < mmu->nr_tlb_entries; idx++) {
		ent = mmu->exmap_tbl + idx;
		if (!ent->valid || ent->prsvd)
			continue;
		if (ent->vadr == vadr)
			goto found_map;
	}
	up_write(&mmu->exmap_sem);
	dev_warn(mmu->dev, "MMU %s: address %06lx not found in exmap_tbl.\n",
		 mmu->name, devadr);
	return -EINVAL;

found_map:
	if (ent->usecount > 0) {
		dev_err(mmu->dev, "MMU %s: exmap reference count is not 0.\n"
			"   idx=%d, vadr=%p, order=%d, usecount=%d\n",
			mmu->name, idx, ent->vadr, ent->order, ent->usecount);
		up_write(&mmu->exmap_sem);
		return -EINVAL;
	}
	/* clearing external device PTE entry */
	omap_mmu_clear_pte_entry(mmu, devadr);

	/* clear ARM MMU and free buffer */
	size = unmap_free_arm(mmu, ent);
	total += size;

	/* we don't free PTEs */

	/* flush TLB */
	flush_tlb_kernel_range((unsigned long)vadr, (unsigned long)vadr + size);

	/* check if next mapping is in same group */
	idx = ent->link.next;
	if (idx < 0)
		goto up_out;	/* normal completion */
	ent = mmu->exmap_tbl + idx;
	devadr += size;
	vadr   += size;
	if (ent->vadr == vadr)
		goto found_map;	/* continue */

	dev_err(mmu->dev, "MMU %s: illegal exmap_tbl grouping!\n"
		"expected vadr = %p, exmap_tbl[%d].vadr = %p\n",
		mmu->name, vadr, idx, ent->vadr);
	up_write(&mmu->exmap_sem);
	return -EINVAL;

up_out:
	up_write(&mmu->exmap_sem);
	return total;
}
EXPORT_SYMBOL_GPL(omap_mmu_exunmap);

void omap_mmu_exmap_flush(struct omap_mmu *mmu)
{
	struct exmap_tbl *ent;

	down_write(&mmu->exmap_sem);

	/* clearing TLB entry */
	omap_mmu_gflush(mmu);

	omap_mmu_for_each_tlb_entry(mmu, ent)
		if (ent->valid && !ent->prsvd)
			unmap_free_arm(mmu, ent);

	/* flush TLB */
	if (likely(mmu->membase))
		flush_tlb_kernel_range(mmu->membase + mmu->memsize,
				       mmu->membase + (1 << mmu->addrspace));

	up_write(&mmu->exmap_sem);
}
EXPORT_SYMBOL_GPL(omap_mmu_exmap_flush);

void exmap_setup_preserved_mem_page(struct omap_mmu *mmu, void *buf,
				    unsigned long devadr, int index)
{
	unsigned long phys;
	void *virt;
	struct omap_mmu_tlb_entry tlb_ent;

	phys = __pa(buf);
	virt = omap_mmu_to_virt(mmu, devadr);
	exmap_set_armmmu(mmu, (unsigned long)virt, phys, PAGE_SIZE);
	INIT_EXMAP_TBL_ENTRY_4KB_PRESERVED(mmu->exmap_tbl + index, buf, virt);
	INIT_TLB_ENTRY_4KB_PRESERVED(&tlb_ent, devadr, phys);
	omap_mmu_load_pte_entry(mmu, &tlb_ent);
}
EXPORT_SYMBOL_GPL(exmap_setup_preserved_mem_page);

void exmap_clear_mem_page(struct omap_mmu *mmu, unsigned long devadr)
{
	void *virt = omap_mmu_to_virt(mmu, devadr);

	exmap_clear_armmmu(mmu, (unsigned long)virt, PAGE_SIZE);
	/* DSP MMU is shutting down. not handled here. */
}
EXPORT_SYMBOL_GPL(exmap_clear_mem_page);

static void omap_mmu_reset(struct omap_mmu *mmu)
{
#if defined(CONFIG_ARCH_OMAP2) /* FIXME */
	int i;

	omap_mmu_write_reg(mmu, 0x2, OMAP_MMU_SYSCONFIG);

	for (i = 0; i < 10000; i++)
		if (likely(omap_mmu_read_reg(mmu, OMAP_MMU_SYSSTATUS) & 0x1))
			break;
#endif
}

void omap_mmu_disable(struct omap_mmu *mmu)
{
	omap_mmu_write_reg(mmu, 0x00, OMAP_MMU_CNTL);
}
EXPORT_SYMBOL_GPL(omap_mmu_disable);

void omap_mmu_enable(struct omap_mmu *mmu, int reset)
{
	u32 val = OMAP_MMU_CNTL_MMU_EN | MMU_CNTL_TWLENABLE;

	if (likely(reset))
		omap_mmu_reset(mmu);
#if defined(CONFIG_ARCH_OMAP2) /* FIXME */
	omap_mmu_write_reg(mmu, (u32)virt_to_phys(mmu->twl_mm->pgd),
			   OMAP_MMU_TTB);
#else
	omap_mmu_write_reg(mmu, (u32)virt_to_phys(mmu->twl_mm->pgd) & 0xffff,
			   OMAP_MMU_TTB_L);
	omap_mmu_write_reg(mmu, (u32)virt_to_phys(mmu->twl_mm->pgd) >> 16,
			   OMAP_MMU_TTB_H);
	val |= OMAP_MMU_CNTL_RESET_SW;
#endif
	omap_mmu_write_reg(mmu, val, OMAP_MMU_CNTL);
}
EXPORT_SYMBOL_GPL(omap_mmu_enable);

static irqreturn_t omap_mmu_interrupt(int irq, void *dev_id)
{
	struct omap_mmu *mmu = dev_id;

	if (likely(mmu->ops->interrupt))
		mmu->ops->interrupt(mmu);

	return IRQ_HANDLED;
}

static int omap_mmu_init(struct omap_mmu *mmu)
{
	struct omap_mmu_tlb_lock tlb_lock;
	int ret = 0;

	clk_enable(mmu->clk);
	ret = omap_dsp_request_mem();
	if (ret < 0)
		goto out;

	down_write(&mmu->exmap_sem);

	ret = request_irq(mmu->irq, omap_mmu_interrupt, IRQF_DISABLED,
			  mmu->name,  mmu);
	if (ret < 0) {
		dev_err(mmu->dev, "MMU %s: failed to register MMU interrupt:"
			" %d\n", mmu->name, ret);
		goto fail;
	}

	omap_mmu_disable(mmu);	/* clear all */
	udelay(100);
	omap_mmu_enable(mmu, 1);

	memset(&tlb_lock, 0, sizeof(struct omap_mmu_tlb_lock));
	omap_mmu_set_tlb_lock(mmu, &tlb_lock);

	if (unlikely(mmu->ops->startup))
		ret = mmu->ops->startup(mmu);
fail:
	up_write(&mmu->exmap_sem);
	omap_dsp_release_mem();
out:
	clk_disable(mmu->clk);

	return ret;
}

static void omap_mmu_shutdown(struct omap_mmu *mmu)
{
	free_irq(mmu->irq, mmu);

	if (unlikely(mmu->ops->shutdown))
		mmu->ops->shutdown(mmu);

	omap_mmu_exmap_flush(mmu);
	omap_mmu_disable(mmu); /* clear all */
}

/*
 * omap_mmu_mem_enable() / disable()
 */
int omap_mmu_mem_enable(struct omap_mmu *mmu, void *addr)
{
	if (unlikely(mmu->ops->mem_enable))
		return mmu->ops->mem_enable(mmu, addr);

	down_read(&mmu->exmap_sem);
	return 0;
}
EXPORT_SYMBOL_GPL(omap_mmu_mem_enable);

void omap_mmu_mem_disable(struct omap_mmu *mmu, void *addr)
{
	if (unlikely(mmu->ops->mem_disable)) {
		mmu->ops->mem_disable(mmu, addr);
		return;
	}

	up_read(&mmu->exmap_sem);
}
EXPORT_SYMBOL_GPL(omap_mmu_mem_disable);

/*
 * dsp_mem file operations
 */
static ssize_t intmem_read(struct omap_mmu *mmu, char *buf, size_t count,
			   loff_t *ppos)
{
	unsigned long p = *ppos;
	void *vadr = omap_mmu_to_virt(mmu, p);
	ssize_t size = mmu->memsize;
	ssize_t read;

	if (p >= size)
		return 0;
	clk_enable(mmu->memclk);
	read = count;
	if (count > size - p)
		read = size - p;
	if (copy_to_user(buf, vadr, read)) {
		read = -EFAULT;
		goto out;
	}
	*ppos += read;
out:
	clk_disable(mmu->memclk);
	return read;
}

static ssize_t exmem_read(struct omap_mmu *mmu, char *buf, size_t count,
			  loff_t *ppos)
{
	unsigned long p = *ppos;
	void *vadr = omap_mmu_to_virt(mmu, p);

	if (!exmap_valid(mmu, vadr, count)) {
		dev_err(mmu->dev, "MMU %s: external device address %08lx / "
			"size %08x is not valid!\n", mmu->name, p, count);
		return -EFAULT;
	}
	if (count > (1 << mmu->addrspace) - p)
		count = (1 << mmu->addrspace) - p;
	if (copy_to_user(buf, vadr, count))
		return -EFAULT;
	*ppos += count;

	return count;
}

static ssize_t omap_mmu_mem_read(struct kobject *kobj,
				 struct bin_attribute *attr,
				 char *buf, loff_t offset, size_t count)
{
	struct device *dev = to_dev(kobj);
	struct omap_mmu *mmu = dev_get_drvdata(dev);
	unsigned long p = (unsigned long)offset;
	void *vadr = omap_mmu_to_virt(mmu, p);
	int ret;

	if (omap_mmu_mem_enable(mmu, vadr) < 0)
		return -EBUSY;

	if (p < mmu->memsize)
		ret = intmem_read(mmu, buf, count, &offset);
	else
		ret = exmem_read(mmu, buf, count, &offset);

	omap_mmu_mem_disable(mmu, vadr);

	return ret;
}

static ssize_t intmem_write(struct omap_mmu *mmu, const char *buf, size_t count,
			    loff_t *ppos)
{
	unsigned long p = *ppos;
	void *vadr = omap_mmu_to_virt(mmu, p);
	ssize_t size = mmu->memsize;
	ssize_t written;

	if (p >= size)
		return 0;
	clk_enable(mmu->memclk);
	written = count;
	if (count > size - p)
		written = size - p;
	if (copy_from_user(vadr, buf, written)) {
		written = -EFAULT;
		goto out;
	}
	*ppos += written;
out:
	clk_disable(mmu->memclk);
	return written;
}

static ssize_t exmem_write(struct omap_mmu *mmu, char *buf, size_t count,
			   loff_t *ppos)
{
	unsigned long p = *ppos;
	void *vadr = omap_mmu_to_virt(mmu, p);

	if (!exmap_valid(mmu, vadr, count)) {
		dev_err(mmu->dev, "MMU %s: external device address %08lx "
			"/ size %08x is not valid!\n", mmu->name, p, count);
		return -EFAULT;
	}
	if (count > (1 << mmu->addrspace) - p)
		count = (1 << mmu->addrspace) - p;
	if (copy_from_user(vadr, buf, count))
		return -EFAULT;
	*ppos += count;

	return count;
}

static ssize_t omap_mmu_mem_write(struct kobject *kobj,
				  struct bin_attribute *attr,
				  char *buf, loff_t offset, size_t count)
{
	struct device *dev = to_dev(kobj);
	struct omap_mmu *mmu = dev_get_drvdata(dev);
	unsigned long p = (unsigned long)offset;
	void *vadr = omap_mmu_to_virt(mmu, p);
	int ret;

	if (omap_mmu_mem_enable(mmu, vadr) < 0)
		return -EBUSY;

	if (p < mmu->memsize)
		ret = intmem_write(mmu, buf, count, &offset);
	else
		ret = exmem_write(mmu, buf, count, &offset);

	omap_mmu_mem_disable(mmu, vadr);

	return ret;
}

static struct bin_attribute dev_attr_mem = {
	.attr	= {
		.name	= "mem",
		.owner	= THIS_MODULE,
		.mode	= S_IRUSR | S_IWUSR | S_IRGRP,
	},

	.read	= omap_mmu_mem_read,
	.write	= omap_mmu_mem_write,
};

/* To be obsolete for backward compatibility */
ssize_t __omap_mmu_mem_read(struct omap_mmu *mmu,
			    struct bin_attribute *attr,
			    char *buf, loff_t offset, size_t count)
{
	return omap_mmu_mem_read(&mmu->dev->kobj, attr, buf, offset, count);
}
EXPORT_SYMBOL_GPL(__omap_mmu_mem_read);

ssize_t __omap_mmu_mem_write(struct omap_mmu *mmu,
			     struct bin_attribute *attr,
			     char *buf, loff_t offset, size_t count)
{
	return omap_mmu_mem_write(&mmu->dev->kobj, attr, buf, offset, count);
}
EXPORT_SYMBOL_GPL(__omap_mmu_mem_write);

/*
 * sysfs files
 */
static ssize_t omap_mmu_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct omap_mmu *mmu = dev_get_drvdata(dev);
	struct omap_mmu_tlb_lock tlb_lock;
	int ret;

	clk_enable(mmu->clk);
	ret = omap_dsp_request_mem();
	if (ret < 0)
		goto out;

	down_read(&mmu->exmap_sem);

	omap_mmu_get_tlb_lock(mmu, &tlb_lock);

	ret = -EIO;
	if (likely(mmu->ops->show))
		ret = mmu->ops->show(mmu, buf, &tlb_lock);

	/* restore victim entry */
	omap_mmu_set_tlb_lock(mmu, &tlb_lock);

	up_read(&mmu->exmap_sem);
	omap_dsp_release_mem();
out:
	clk_disable(mmu->clk);

	return ret;
}

static DEVICE_ATTR(mmu, S_IRUGO, omap_mmu_show, NULL);

static ssize_t exmap_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct omap_mmu *mmu = dev_get_drvdata(dev);
	struct exmap_tbl *ent;
	int len;
	int i = 0;

	down_read(&mmu->exmap_sem);
	len = sprintf(buf, "  devadr     size         buf     size uc\n");
			 /* 0x300000 0x123000  0xc0171000 0x100000  0*/

	omap_mmu_for_each_tlb_entry(mmu, ent) {
		void *vadr;
		unsigned long size;
		enum exmap_type type;
		int idx;

		/* find a top of link */
		if (!ent->valid || (ent->link.prev >= 0))
			continue;

		vadr = ent->vadr;
		type = ent->type;
		size = 0;
		idx = i;
		do {
			ent = mmu->exmap_tbl + idx;
			size += PAGE_SIZE << ent->order;
		} while ((idx = ent->link.next) >= 0);

		len += sprintf(buf + len, "0x%06lx %#8lx",
			       virt_to_omap_mmu(mmu, vadr), size);

		if (type == EXMAP_TYPE_FB) {
			len += sprintf(buf + len, "    framebuf\n");
		} else {
			len += sprintf(buf + len, "\n");
			idx = i;
			do {
				ent = mmu->exmap_tbl + idx;
				len += sprintf(buf + len,
					       /* 0xc0171000 0x100000  0*/
					       "%19s0x%8p %#8lx %2d\n",
					       "", ent->buf,
					       PAGE_SIZE << ent->order,
					       ent->usecount);
			} while ((idx = ent->link.next) >= 0);
		}

		i++;
	}

	up_read(&mmu->exmap_sem);
	return len;
}

static ssize_t exmap_store(struct device *dev, struct device_attribute *attr,
			   const char *buf,
			   size_t count)
{
	struct omap_mmu *mmu = dev_get_drvdata(dev);
	unsigned long base = 0, len = 0;
	int ret;

	sscanf(buf, "%lx %lx", &base, &len);

	if (!base)
		return -EINVAL;

	if (len) {
		/* Add the mapping */
		ret = omap_mmu_exmap(mmu, base, 0, len, EXMAP_TYPE_MEM);
		if (ret < 0)
			return ret;
	} else {
		/* Remove the mapping */
		ret = omap_mmu_exunmap(mmu, base);
		if (ret < 0)
			return ret;
	}

	return count;
}

static DEVICE_ATTR(exmap, S_IRUGO | S_IWUSR, exmap_show, exmap_store);

static ssize_t mempool_show(struct class *class, char *buf)
{
	int min_nr_1M = 0, curr_nr_1M = 0;
	int min_nr_64K = 0, curr_nr_64K = 0;
	int total = 0;

	if (likely(mempool_1M)) {
		min_nr_1M  = mempool_1M->min_nr;
		curr_nr_1M = mempool_1M->curr_nr;
		total += min_nr_1M * SZ_1M;
	}
	if (likely(mempool_64K)) {
		min_nr_64K  = mempool_64K->min_nr;
		curr_nr_64K = mempool_64K->curr_nr;
		total += min_nr_64K * SZ_64K;
	}

	return sprintf(buf,
		       "0x%x\n"
		       "1M  buffer: %d (%d free)\n"
		       "64K buffer: %d (%d free)\n",
		       total, min_nr_1M, curr_nr_1M, min_nr_64K, curr_nr_64K);
}


static CLASS_ATTR(mempool, S_IRUGO, mempool_show, NULL);

static struct class omap_mmu_class = {
	.name		= "mmu",
};

int omap_mmu_register(struct omap_mmu *mmu)
{
	int ret;

	mmu->dev = device_create(&omap_mmu_class, NULL, 0, "%s", mmu->name);
	if (unlikely(IS_ERR(mmu->dev)))
		return PTR_ERR(mmu->dev);
	dev_set_drvdata(mmu->dev, mmu);

	mmu->exmap_tbl = kcalloc(mmu->nr_tlb_entries, sizeof(struct exmap_tbl),
				 GFP_KERNEL);
	if (!mmu->exmap_tbl)
		return -ENOMEM;

	mmu->twl_mm = mm_alloc();
	if (!mmu->twl_mm) {
		ret = -ENOMEM;
		goto err_mm_alloc;
	}

	init_rwsem(&mmu->exmap_sem);

	ret = omap_mmu_init(mmu);
	if (unlikely(ret))
		goto err_mmu_init;

	ret = device_create_file(mmu->dev, &dev_attr_mmu);
	if (unlikely(ret))
		goto err_dev_create_mmu;
	ret = device_create_file(mmu->dev, &dev_attr_exmap);
	if (unlikely(ret))
		goto err_dev_create_exmap;

	if (likely(mmu->membase)) {
		dev_attr_mem.size = mmu->memsize;
		ret = device_create_bin_file(mmu->dev,
					     &dev_attr_mem);
		if (unlikely(ret))
			goto err_bin_create_mem;
	}
	return 0;

err_bin_create_mem:
	device_remove_file(mmu->dev, &dev_attr_exmap);
err_dev_create_exmap:
	device_remove_file(mmu->dev, &dev_attr_mmu);
err_dev_create_mmu:
	omap_mmu_shutdown(mmu);
err_mmu_init:
	kfree(mmu->twl_mm);
	mmu->twl_mm = NULL;
err_mm_alloc:
	kfree(mmu->exmap_tbl);
	mmu->exmap_tbl = NULL;
	device_unregister(mmu->dev);
	return ret;
}
EXPORT_SYMBOL_GPL(omap_mmu_register);

void omap_mmu_unregister(struct omap_mmu *mmu)
{
	omap_mmu_shutdown(mmu);
	omap_mmu_kmem_release();

	device_remove_file(mmu->dev, &dev_attr_mmu);
	device_remove_file(mmu->dev, &dev_attr_exmap);

	if (likely(mmu->membase))
		device_remove_bin_file(mmu->dev, &dev_attr_mem);

	device_unregister(mmu->dev);

	kfree(mmu->exmap_tbl);
	mmu->exmap_tbl = NULL;

	if (mmu->twl_mm) {
		__mmdrop(mmu->twl_mm);
		mmu->twl_mm = NULL;
	}
}
EXPORT_SYMBOL_GPL(omap_mmu_unregister);

static int __init omap_mmu_class_init(void)
{
	int ret = class_register(&omap_mmu_class);
	if (!ret)
		ret = class_create_file(&omap_mmu_class, &class_attr_mempool);

	return ret;
}

static void __exit omap_mmu_class_exit(void)
{
	class_remove_file(&omap_mmu_class, &class_attr_mempool);
	class_unregister(&omap_mmu_class);
}

subsys_initcall(omap_mmu_class_init);
module_exit(omap_mmu_class_exit);

MODULE_LICENSE("GPL");
