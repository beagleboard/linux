/*
 * This file is part of OMAP DSP driver (DSP Gateway version 3.3.1)
 *
 * Copyright (C) 2002-2006 Nokia Corporation. All rights reserved.
 *
 * Contact: Toshihiro Kobayashi <toshihiro.kobayashi@nokia.com>
 *
 * Conversion to mempool API and ARM MMU section mapping
 * by Paul Mundt <paul.mundt@nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/fb.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/mempool.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/pgalloc.h>
#include <asm/pgtable.h>
#include <asm/arch/tc.h>
#include <asm/arch/omapfb.h>
#include <asm/arch/mailbox.h>
#include <asm/arch/dsp_common.h>
#include "uaccess_dsp.h"
#include "dsp_mbcmd.h"
#include "dsp.h"
#include "ioctl.h"
#include "ipbuf.h"

#ifdef CONFIG_ARCH_OMAP2
#define IOMAP_VAL	0x3f
#endif

#define SZ_1KB	0x400
#define SZ_4KB	0x1000
#define SZ_64KB	0x10000
#define SZ_1MB	0x100000
#define SZ_16MB	0x1000000
#define is_aligned(adr,align)	(!((adr)&((align)-1)))
#define ORDER_4KB	(12 - PAGE_SHIFT)
#define ORDER_64KB	(16 - PAGE_SHIFT)
#define ORDER_1MB	(20 - PAGE_SHIFT)

/*
 * absorb DSP MMU register size and location difference
 */
#if defined(CONFIG_ARCH_OMAP1)
typedef u16 dsp_mmu_reg_t;
#define dsp_mmu_read_reg(a)	omap_readw(a)
#define dsp_mmu_write_reg(v,a)	omap_writew(v,a)
#elif defined(CONFIG_ARCH_OMAP2)
typedef u32 dsp_mmu_reg_t;
#define dsp_mmu_read_reg(a)	readl(a)
#define dsp_mmu_write_reg(v,a)	writel(v,a)
#define dsp_ipi_read_reg(a)	readl(a)
#define dsp_ipi_write_reg(v,a)	writel(v,a)
#endif

#if defined(CONFIG_ARCH_OMAP1)

#define dsp_mmu_enable() \
	do { \
		dsp_mmu_write_reg(DSP_MMU_CNTL_MMU_EN | DSP_MMU_CNTL_RESET_SW, \
				  DSP_MMU_CNTL); \
	} while(0)
#define dsp_mmu_disable() \
	do { \
		dsp_mmu_write_reg(0, DSP_MMU_CNTL); \
	} while(0)
#define __dsp_mmu_itack() \
	do { \
		dsp_mmu_write_reg(DSP_MMU_IT_ACK_IT_ACK, DSP_MMU_IT_ACK); \
	} while(0)

#elif defined(CONFIG_ARCH_OMAP2)

#define dsp_mmu_enable() \
	do { \
		dsp_mmu_write_reg(DSP_MMU_CNTL_MMUENABLE, DSP_MMU_CNTL); \
	} while(0)
#define dsp_mmu_disable() \
	do { \
		dsp_mmu_write_reg(0, DSP_MMU_CNTL); \
	} while(0)
#define dsp_mmu_reset() \
	do { \
		dsp_mmu_write_reg(dsp_mmu_read_reg(DSP_MMU_SYSCONFIG) | \
				  DSP_MMU_SYSCONFIG_SOFTRESET, \
				  DSP_MMU_SYSCONFIG); \
	} while(0)

#endif /* CONFIG_ARCH_OMAP2 */

#define dsp_mmu_flush() \
	do { \
		dsp_mmu_write_reg(DSP_MMU_FLUSH_ENTRY_FLUSH_ENTRY, \
				  DSP_MMU_FLUSH_ENTRY); \
	} while(0)
#define __dsp_mmu_gflush() \
	do { \
		dsp_mmu_write_reg(DSP_MMU_GFLUSH_GFLUSH, DSP_MMU_GFLUSH); \
	} while(0)

/*
 * absorb register name difference
 */
#ifdef CONFIG_ARCH_OMAP1
#define DSP_MMU_CAM_P			DSP_MMU_CAM_L_P
#define DSP_MMU_CAM_V			DSP_MMU_CAM_L_V
#define DSP_MMU_CAM_PAGESIZE_MASK	DSP_MMU_CAM_L_PAGESIZE_MASK
#define DSP_MMU_CAM_PAGESIZE_1MB	DSP_MMU_CAM_L_PAGESIZE_1MB
#define DSP_MMU_CAM_PAGESIZE_64KB	DSP_MMU_CAM_L_PAGESIZE_64KB
#define DSP_MMU_CAM_PAGESIZE_4KB	DSP_MMU_CAM_L_PAGESIZE_4KB
#define DSP_MMU_CAM_PAGESIZE_1KB	DSP_MMU_CAM_L_PAGESIZE_1KB
#endif /* CONFIG_ARCH_OMAP1 */

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
#endif /* CONFIG_ARCH_OMAP1 */

enum exmap_type_e {
	EXMAP_TYPE_MEM,
	EXMAP_TYPE_FB
};

struct exmap_tbl_entry {
	unsigned int valid:1;
	unsigned int prsvd:1;	/* preserved */
	int usecount;		/* reference count by mmap */
	enum exmap_type_e type;
	void *buf;		/* virtual address of the buffer,
				 * i.e. 0xc0000000 - */
	void *vadr;		/* DSP shadow space,
				 * i.e. 0xe0000000 - 0xe0ffffff */
	unsigned int order;
	struct {
		int prev;
		int next;
	} link;			/* grouping */
};

#define INIT_EXMAP_TBL_ENTRY(ent,b,v,typ,od) \
	do {\
		(ent)->buf       = (b); \
		(ent)->vadr      = (v); \
		(ent)->valid     = 1; \
		(ent)->prsvd     = 0; \
		(ent)->usecount  = 0; \
		(ent)->type      = (typ); \
		(ent)->order     = (od); \
		(ent)->link.next = -1; \
		(ent)->link.prev = -1; \
	} while (0)

#define INIT_EXMAP_TBL_ENTRY_4KB_PRESERVED(ent,b,v) \
	do {\
		(ent)->buf       = (b); \
		(ent)->vadr      = (v); \
		(ent)->valid     = 1; \
		(ent)->prsvd     = 1; \
		(ent)->usecount  = 0; \
		(ent)->type      = EXMAP_TYPE_MEM; \
		(ent)->order     = 0; \
		(ent)->link.next = -1; \
		(ent)->link.prev = -1; \
	} while (0)

#define DSP_MMU_TLB_LINES	32
static struct exmap_tbl_entry exmap_tbl[DSP_MMU_TLB_LINES];
static int exmap_preserved_cnt;
static DECLARE_RWSEM(exmap_sem);

#ifdef CONFIG_FB_OMAP_LCDC_EXTERNAL
static struct omapfb_notifier_block *omapfb_nb;
static int omapfb_ready;
#endif

struct cam_ram_regset {
#if defined(CONFIG_ARCH_OMAP1)
	dsp_mmu_reg_t cam_h;
	dsp_mmu_reg_t cam_l;
	dsp_mmu_reg_t ram_h;
	dsp_mmu_reg_t ram_l;
#elif defined(CONFIG_ARCH_OMAP2)
	dsp_mmu_reg_t cam;
	dsp_mmu_reg_t ram;
#endif
};

struct tlb_entry {
	dsp_long_t va;
	unsigned long pa;
	dsp_mmu_reg_t pgsz, prsvd, valid;
#if defined(CONFIG_ARCH_OMAP1)
	dsp_mmu_reg_t ap;
#elif defined(CONFIG_ARCH_OMAP2)
	dsp_mmu_reg_t endian, elsz, mixed;
#endif
};

#if defined(CONFIG_ARCH_OMAP1)
#define INIT_TLB_ENTRY(ent,v,p,ps) \
	do { \
		(ent)->va = (v); \
		(ent)->pa = (p); \
		(ent)->pgsz = (ps); \
		(ent)->prsvd = 0; \
		(ent)->ap = DSP_MMU_RAM_L_AP_FA; \
	} while (0)
#define INIT_TLB_ENTRY_4KB_PRESERVED(ent,v,p) \
	do { \
		(ent)->va = (v); \
		(ent)->pa = (p); \
		(ent)->pgsz = DSP_MMU_CAM_PAGESIZE_4KB; \
		(ent)->prsvd = DSP_MMU_CAM_P; \
		(ent)->ap = DSP_MMU_RAM_L_AP_FA; \
	} while (0)
#elif defined(CONFIG_ARCH_OMAP2)
#define INIT_TLB_ENTRY(ent,v,p,ps) \
	do { \
		(ent)->va = (v); \
		(ent)->pa = (p); \
		(ent)->pgsz = (ps); \
		(ent)->prsvd = 0; \
		(ent)->endian = DSP_MMU_RAM_ENDIANNESS_LITTLE; \
		(ent)->elsz = DSP_MMU_RAM_ELEMENTSIZE_16; \
		(ent)->mixed = 0; \
	} while (0)
#define INIT_TLB_ENTRY_4KB_PRESERVED(ent,v,p) \
	do { \
		(ent)->va = (v); \
		(ent)->pa = (p); \
		(ent)->pgsz = DSP_MMU_CAM_PAGESIZE_4KB; \
		(ent)->prsvd = DSP_MMU_CAM_P; \
		(ent)->endian = DSP_MMU_RAM_ENDIANNESS_LITTLE; \
		(ent)->elsz = DSP_MMU_RAM_ELEMENTSIZE_16; \
		(ent)->mixed = 0; \
	} while (0)
#define INIT_TLB_ENTRY_4KB_ES32_PRESERVED(ent,v,p) \
	do { \
		(ent)->va = (v); \
		(ent)->pa = (p); \
		(ent)->pgsz = DSP_MMU_CAM_PAGESIZE_4KB; \
		(ent)->prsvd = DSP_MMU_CAM_P; \
		(ent)->endian = DSP_MMU_RAM_ENDIANNESS_LITTLE; \
		(ent)->elsz = DSP_MMU_RAM_ELEMENTSIZE_32; \
		(ent)->mixed = 0; \
	} while (0)
#endif

#if defined(CONFIG_ARCH_OMAP1)
#define cam_ram_valid(cr)	((cr).cam_l & DSP_MMU_CAM_V)
#elif defined(CONFIG_ARCH_OMAP2)
#define cam_ram_valid(cr)	((cr).cam & DSP_MMU_CAM_V)
#endif

struct tlb_lock {
	int base;
	int victim;
};

static int dsp_exunmap(dsp_long_t dspadr);

static void *dspvect_page;
static u32 dsp_fault_adr;
static struct mem_sync_struct mem_sync;

static ssize_t mmu_show(struct device *dev, struct device_attribute *attr,
			char *buf);
static ssize_t exmap_show(struct device *dev, struct device_attribute *attr,
			  char *buf);
static ssize_t mempool_show(struct device *dev, struct device_attribute *attr,
			    char *buf);

static struct device_attribute dev_attr_mmu =     __ATTR_RO(mmu);
static struct device_attribute dev_attr_exmap =   __ATTR_RO(exmap);
static struct device_attribute dev_attr_mempool = __ATTR_RO(mempool);

/*
 * special mempool function:
 * hope this goes to mm/mempool.c
 */
static void *mempool_alloc_from_pool(mempool_t *pool, gfp_t gfp_mask)
{
	unsigned long flags;

	spin_lock_irqsave(&pool->lock, flags);
	if (likely(pool->curr_nr)) {
		void *element = pool->elements[--pool->curr_nr];
		spin_unlock_irqrestore(&pool->lock, flags);
		return element;
	}
	spin_unlock_irqrestore(&pool->lock, flags);

	return mempool_alloc(pool, gfp_mask);
}

static __inline__ unsigned long lineup_offset(unsigned long adr,
					      unsigned long ref,
					      unsigned long mask)
{
	unsigned long newadr;

	newadr = (adr & ~mask) | (ref & mask);
	if (newadr < adr)
		newadr += mask + 1;
	return newadr;
}

int dsp_mem_sync_inc(void)
{
	if (dsp_mem_enable((void *)dspmem_base) < 0)
		return -1;
	if (mem_sync.DARAM)
		mem_sync.DARAM->ad_arm++;
	if (mem_sync.SARAM)
		mem_sync.SARAM->ad_arm++;
	if (mem_sync.SDRAM)
		mem_sync.SDRAM->ad_arm++;
	dsp_mem_disable((void *)dspmem_base);
	return 0;
}

/*
 * dsp_mem_sync_config() is called from mbox1 workqueue
 */
int dsp_mem_sync_config(struct mem_sync_struct *sync)
{
	size_t sync_seq_sz = sizeof(struct sync_seq);

#ifdef OLD_BINARY_SUPPORT
	if (sync == NULL) {
		memset(&mem_sync, 0, sizeof(struct mem_sync_struct));
		return 0;
	}
#endif
	if ((dsp_mem_type(sync->DARAM, sync_seq_sz) != MEM_TYPE_DARAM) ||
	    (dsp_mem_type(sync->SARAM, sync_seq_sz) != MEM_TYPE_SARAM) ||
	    (dsp_mem_type(sync->SDRAM, sync_seq_sz) != MEM_TYPE_EXTERN)) {
		printk(KERN_ERR
		       "omapdsp: mem_sync address validation failure!\n"
		       "  mem_sync.DARAM = 0x%p,\n"
		       "  mem_sync.SARAM = 0x%p,\n"
		       "  mem_sync.SDRAM = 0x%p,\n",
		       sync->DARAM, sync->SARAM, sync->SDRAM);
		return -1;
	}
	memcpy(&mem_sync, sync, sizeof(struct mem_sync_struct));
	return 0;
}

static mempool_t *kmem_pool_1M;
static mempool_t *kmem_pool_64K;

static void *dsp_pool_alloc(unsigned int __nocast gfp, void *order)
{
	return (void *)__get_dma_pages(gfp, (unsigned int)order);
}

static void dsp_pool_free(void *buf, void *order)
{
	free_pages((unsigned long)buf, (unsigned int)order);
}

static void dsp_kmem_release(void)
{
	if (kmem_pool_64K) {
		mempool_destroy(kmem_pool_64K);
		kmem_pool_64K = NULL;
	}

	if (kmem_pool_1M) {
		mempool_destroy(kmem_pool_1M);
		kmem_pool_1M = NULL;
	}
}

static int dsp_kmem_reserve(unsigned long size)
{
	unsigned long len = size;

	/* alignment check */
	if (!is_aligned(size, SZ_64KB)) {
		printk(KERN_ERR
		       "omapdsp: size(0x%lx) is not multiple of 64KB.\n", size);
		return -EINVAL;
	}

	if (size > DSPSPACE_SIZE) {
		printk(KERN_ERR
		       "omapdsp: size(0x%lx) is larger than DSP memory space "
		       "size (0x%x.\n", size, DSPSPACE_SIZE);
		return -EINVAL;
	}

	if (size >= SZ_1MB) {
		int nr = size >> 20;

		if (likely(!kmem_pool_1M))
			kmem_pool_1M = mempool_create(nr,
						      dsp_pool_alloc,
						      dsp_pool_free,
						      (void *)ORDER_1MB);
		else
			mempool_resize(kmem_pool_1M, kmem_pool_1M->min_nr + nr,
				       GFP_KERNEL);

		size &= ~(0xf << 20);
	}

	if (size >= SZ_64KB) {
		int nr = size >> 16;

		if (likely(!kmem_pool_64K))
			kmem_pool_64K = mempool_create(nr,
						       dsp_pool_alloc,
						       dsp_pool_free,
						       (void *)ORDER_64KB);
		else
			mempool_resize(kmem_pool_64K,
				       kmem_pool_64K->min_nr + nr, GFP_KERNEL);

		size &= ~(0xf << 16);
	}

	if (size)
		len -= size;

	return len;
}

static void dsp_mem_free_pages(unsigned long buf, unsigned int order)
{
	struct page *page, *ps, *pe;

	ps = virt_to_page(buf);
	pe = virt_to_page(buf + (1 << (PAGE_SHIFT + order)));

	for (page = ps; page < pe; page++)
		ClearPageReserved(page);

	if ((order == ORDER_64KB) && likely(kmem_pool_64K))
		mempool_free((void *)buf, kmem_pool_64K);
	else if ((order == ORDER_1MB) && likely(kmem_pool_1M))
		mempool_free((void *)buf, kmem_pool_1M);
	else
		free_pages(buf, order);
}

static inline void
exmap_alloc_pte(unsigned long virt, unsigned long phys, pgprot_t prot)
{
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;
	pte_t *pte;

	pgd = pgd_offset_k(virt);
	pud = pud_offset(pgd, virt);
	pmd = pmd_offset(pud, virt);

	if (pmd_none(*pmd)) {
		pte = pte_alloc_one_kernel(&init_mm, 0);
		if (!pte)
			return;

		/* note: two PMDs will be set  */
		pmd_populate_kernel(&init_mm, pmd, pte);
	}

	pte = pte_offset_kernel(pmd, virt);
	set_pte(pte, pfn_pte(phys >> PAGE_SHIFT, prot));
}

#if 0
static inline int
exmap_alloc_sect(unsigned long virt, unsigned long phys, int prot)
{
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;

	pgd = pgd_offset_k(virt);
	pud = pud_alloc(&init_mm, pgd, virt);
	pmd = pmd_alloc(&init_mm, pud, virt);

	if (virt & (1 << 20))
		pmd++;

	if (!pmd_none(*pmd))
		/* No good, fall back on smaller mappings. */
		return -EINVAL;

	*pmd = __pmd(phys | prot);
	flush_pmd_entry(pmd);

	return 0;
}
#endif

/*
 * ARM MMU operations
 */
static int exmap_set_armmmu(unsigned long virt, unsigned long phys,
			    unsigned long size)
{
	long off;
	pgprot_t prot_pte;
	int prot_sect;

	printk(KERN_DEBUG
	       "omapdsp: mapping in ARM MMU, v=0x%08lx, p=0x%08lx, sz=0x%lx\n",
	       virt, phys, size);

	prot_pte = __pgprot(L_PTE_PRESENT | L_PTE_YOUNG |
			    L_PTE_DIRTY | L_PTE_WRITE);

	prot_sect = PMD_TYPE_SECT | PMD_SECT_UNCACHED |
		    PMD_SECT_AP_WRITE | PMD_DOMAIN(DOMAIN_IO);

	if (cpu_architecture() <= CPU_ARCH_ARMv5)
		prot_sect |= PMD_BIT4;

	off = phys - virt;

	while ((virt & 0xfffff || (virt + off) & 0xfffff) && size >= PAGE_SIZE) {
		exmap_alloc_pte(virt, virt + off, prot_pte);

		virt += PAGE_SIZE;
		size -= PAGE_SIZE;
	}

	/* XXX: Not yet.. confuses dspfb -- PFM. */
#if 0
	while (size >= (PGDIR_SIZE / 2)) {
		if (exmap_alloc_sect(virt, virt + off, prot_sect) < 0)
			break;

		virt += (PGDIR_SIZE / 2);
		size -= (PGDIR_SIZE / 2);
	}
#endif

	while (size >= PAGE_SIZE) {
		exmap_alloc_pte(virt, virt + off, prot_pte);

		virt += PAGE_SIZE;
		size -= PAGE_SIZE;
	}

	BUG_ON(size);

	return 0;
}

	/* XXX: T.Kobayashi
	 * A process can have old mappings. if we want to clear a pmd,
	 * we need to do it for all proceeses that use the old mapping.
	 */
#if 0
static inline void
exmap_clear_pte_range(pmd_t *pmd, unsigned long addr, unsigned long end)
{
	pte_t *pte;

	pte = pte_offset_map(pmd, addr);
	do {
		if (pte_none(*pte))
			continue;

		pte_clear(&init_mm, addr, pte);
	} while (pte++, addr += PAGE_SIZE, addr != end);

	pte_unmap(pte - 1);
}

static inline void
exmap_clear_pmd_range(pud_t *pud, unsigned long addr, unsigned long end)
{
	pmd_t *pmd;
	unsigned long next;

	pmd = pmd_offset(pud, addr);
	do {
		next = pmd_addr_end(addr, end);

		if (addr & (1 << 20))
			pmd++;

		if ((pmd_val(*pmd) & PMD_TYPE_MASK) == PMD_TYPE_SECT) {
			*pmd = __pmd(0);
			clean_pmd_entry(pmd);
			continue;
		}

		if (pmd_none_or_clear_bad(pmd))
			continue;

		exmap_clear_pte_range(pmd, addr, next);
	} while (pmd++, addr = next, addr != end);
}

static inline void
exmap_clear_pud_range(pgd_t *pgd, unsigned long addr, unsigned long end)
{
	pud_t *pud;
	unsigned long next;

	pud = pud_offset(pgd, addr);
	do {
		next = pud_addr_end(addr, end);
		if (pud_none_or_clear_bad(pud))
			continue;

		exmap_clear_pmd_range(pud, addr, next);
	} while (pud++, addr = next, addr != end);
}
#endif

static void exmap_clear_armmmu(unsigned long virt, unsigned long size)
{
#if 0
	unsigned long next, end;
	pgd_t *pgd;

	printk(KERN_DEBUG
	       "omapdsp: unmapping in ARM MMU, v=%#010lx, sz=%#lx\n",
	       virt, size);

	pgd = pgd_offset_k(virt);
	end = virt + size;
	do {
		next = pgd_addr_end(virt, end);
		if (pgd_none_or_clear_bad(pgd))
			continue;

		exmap_clear_pud_range(pgd, virt, next);
	} while (pgd++, virt = next, virt != end);
#else
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;
	pte_t *pte;

	printk(KERN_DEBUG
	       "omapdsp: unmapping in ARM MMU, v=%#010lx, sz=%#lx\n",
	       virt, size);

	while (size >= PAGE_SIZE) {
		pgd = pgd_offset_k(virt);
		pud = pud_offset(pgd, virt);
		pmd = pmd_offset(pud, virt);
		pte = pte_offset_kernel(pmd, virt);

		pte_clear(&init_mm, virt, pte);
		size -= PAGE_SIZE;
		virt += PAGE_SIZE;
	}

	BUG_ON(size);
#endif
}

static int exmap_valid(void *vadr, size_t len)
{
	/* exmap_sem should be held before calling this function */
	int i;

start:
	for (i = 0; i < DSP_MMU_TLB_LINES; i++) {
		void *mapadr;
		unsigned long mapsize;
		struct exmap_tbl_entry *ent = &exmap_tbl[i];

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

enum dsp_mem_type_e dsp_mem_type(void *vadr, size_t len)
{
	void *ds = (void *)daram_base;
	void *de = (void *)daram_base + daram_size;
	void *ss = (void *)saram_base;
	void *se = (void *)saram_base + saram_size;
	int ret;

	if ((vadr >= ds) && (vadr < de)) {
		if (vadr + len > de)
			return MEM_TYPE_CROSSING;
		else
			return MEM_TYPE_DARAM;
	} else if ((vadr >= ss) && (vadr < se)) {
		if (vadr + len > se)
			return MEM_TYPE_CROSSING;
		else
			return MEM_TYPE_SARAM;
	} else {
		down_read(&exmap_sem);
		if (exmap_valid(vadr, len))
			ret = MEM_TYPE_EXTERN;
		else
			ret = MEM_TYPE_NONE;
		up_read(&exmap_sem);
		return ret;
	}
}

int dsp_address_validate(void *p, size_t len, char *fmt, ...)
{
	if (dsp_mem_type(p, len) <= 0) {
		if (fmt != NULL) {
			char s[64];
			va_list args;

			va_start(args, fmt);
			vsprintf(s, fmt, args);
			va_end(args);
			printk(KERN_ERR
			       "omapdsp: %s address(0x%p) and size(0x%x) is "
			       "not valid!\n"
			       "         (crossing different type of memories, or \n"
			       "          external memory space where no "
			       "actual memory is mapped)\n",
			       s, p, len);
		}
		return -1;
	}

	return 0;
}

/*
 * exmap_use(), unuse():
 * when the mapped area is exported to user space with mmap,
 * the usecount is incremented.
 * while the usecount > 0, that area can't be released.
 */
void exmap_use(void *vadr, size_t len)
{
	int i;

	down_write(&exmap_sem);
	for (i = 0; i < DSP_MMU_TLB_LINES; i++) {
		void *mapadr;
		unsigned long mapsize;
		struct exmap_tbl_entry *ent = &exmap_tbl[i];

		if (!ent->valid)
			continue;
		mapadr = (void *)ent->vadr;
		mapsize = 1 << (ent->order + PAGE_SHIFT);
		if ((vadr + len > mapadr) && (vadr < mapadr + mapsize))
			ent->usecount++;
	}
	up_write(&exmap_sem);
}

void exmap_unuse(void *vadr, size_t len)
{
	int i;

	down_write(&exmap_sem);
	for (i = 0; i < DSP_MMU_TLB_LINES; i++) {
		void *mapadr;
		unsigned long mapsize;
		struct exmap_tbl_entry *ent = &exmap_tbl[i];

		if (!ent->valid)
			continue;
		mapadr = (void *)ent->vadr;
		mapsize = 1 << (ent->order + PAGE_SHIFT);
		if ((vadr + len > mapadr) && (vadr < mapadr + mapsize))
			ent->usecount--;
	}
	up_write(&exmap_sem);
}

/*
 * dsp_virt_to_phys()
 * returns physical address, and sets len to valid length
 */
unsigned long dsp_virt_to_phys(void *vadr, size_t *len)
{
	int i;

	if (is_dsp_internal_mem(vadr)) {
		/* DSRAM or SARAM */
		*len = dspmem_base + dspmem_size - (unsigned long)vadr;
		return (unsigned long)vadr;
	}

	/* EXRAM */
	for (i = 0; i < DSP_MMU_TLB_LINES; i++) {
		void *mapadr;
		unsigned long mapsize;
		struct exmap_tbl_entry *ent = &exmap_tbl[i];

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

/*
 * DSP MMU operations
 */
#ifdef CONFIG_ARCH_OMAP1
static dsp_mmu_reg_t get_cam_l_va_mask(dsp_mmu_reg_t pgsz)
{
	switch (pgsz) {
	case DSP_MMU_CAM_PAGESIZE_1MB:
		return DSP_MMU_CAM_L_VA_TAG_L1_MASK |
		       DSP_MMU_CAM_L_VA_TAG_L2_MASK_1MB;
	case DSP_MMU_CAM_PAGESIZE_64KB:
		return DSP_MMU_CAM_L_VA_TAG_L1_MASK |
		       DSP_MMU_CAM_L_VA_TAG_L2_MASK_64KB;
	case DSP_MMU_CAM_PAGESIZE_4KB:
		return DSP_MMU_CAM_L_VA_TAG_L1_MASK |
		       DSP_MMU_CAM_L_VA_TAG_L2_MASK_4KB;
	case DSP_MMU_CAM_PAGESIZE_1KB:
		return DSP_MMU_CAM_L_VA_TAG_L1_MASK |
		       DSP_MMU_CAM_L_VA_TAG_L2_MASK_1KB;
	}
	return 0;
}
#endif /* CONFIG_ARCH_OMAP1 */

#if defined(CONFIG_ARCH_OMAP1)
#define get_cam_va_mask(pgsz) \
	((u32)DSP_MMU_CAM_H_VA_TAG_H_MASK << 22 | \
	 (u32)get_cam_l_va_mask(pgsz) << 6)
#elif defined(CONFIG_ARCH_OMAP2)
#define get_cam_va_mask(pgsz) \
	((pgsz == DSP_MMU_CAM_PAGESIZE_16MB) ? 0xff000000 : \
	 (pgsz == DSP_MMU_CAM_PAGESIZE_1MB)  ? 0xfff00000 : \
	 (pgsz == DSP_MMU_CAM_PAGESIZE_64KB) ? 0xffff0000 : \
	 (pgsz == DSP_MMU_CAM_PAGESIZE_4KB)  ? 0xfffff000 : 0)
#endif /* CONFIG_ARCH_OMAP2 */

static void get_tlb_lock(struct tlb_lock *tlb_lock)
{
	dsp_mmu_reg_t lock = dsp_mmu_read_reg(DSP_MMU_LOCK);

	tlb_lock->base = (lock & DSP_MMU_LOCK_BASE_MASK) >>
			 DSP_MMU_LOCK_BASE_SHIFT;
	tlb_lock->victim = (lock & DSP_MMU_LOCK_VICTIM_MASK) >>
			   DSP_MMU_LOCK_VICTIM_SHIFT;
}

static void set_tlb_lock(struct tlb_lock *tlb_lock)
{
	dsp_mmu_write_reg((tlb_lock->base   << DSP_MMU_LOCK_BASE_SHIFT) |
			  (tlb_lock->victim << DSP_MMU_LOCK_VICTIM_SHIFT),
			  DSP_MMU_LOCK);
}

static void __read_tlb(struct tlb_lock *tlb_lock, struct cam_ram_regset *cr)
{
	/* set victim */
	set_tlb_lock(tlb_lock);

#if defined(CONFIG_ARCH_OMAP1)
	/* read a TLB entry */
	dsp_mmu_write_reg(DSP_MMU_LD_TLB_RD, DSP_MMU_LD_TLB);

	cr->cam_h = dsp_mmu_read_reg(DSP_MMU_READ_CAM_H);
	cr->cam_l = dsp_mmu_read_reg(DSP_MMU_READ_CAM_L);
	cr->ram_h = dsp_mmu_read_reg(DSP_MMU_READ_RAM_H);
	cr->ram_l = dsp_mmu_read_reg(DSP_MMU_READ_RAM_L);
#elif defined(CONFIG_ARCH_OMAP2)
	cr->cam = dsp_mmu_read_reg(DSP_MMU_READ_CAM);
	cr->ram = dsp_mmu_read_reg(DSP_MMU_READ_RAM);
#endif
}

static void __load_tlb(struct cam_ram_regset *cr)
{
#if defined(CONFIG_ARCH_OMAP1)
	dsp_mmu_write_reg(cr->cam_h, DSP_MMU_CAM_H);
	dsp_mmu_write_reg(cr->cam_l, DSP_MMU_CAM_L);
	dsp_mmu_write_reg(cr->ram_h, DSP_MMU_RAM_H);
	dsp_mmu_write_reg(cr->ram_l, DSP_MMU_RAM_L);
#elif defined(CONFIG_ARCH_OMAP2)
	dsp_mmu_write_reg(cr->cam | DSP_MMU_CAM_V, DSP_MMU_CAM);
	dsp_mmu_write_reg(cr->ram, DSP_MMU_RAM);
#endif

	/* flush the entry */
	dsp_mmu_flush();

	/* load a TLB entry */
	dsp_mmu_write_reg(DSP_MMU_LD_TLB_LD, DSP_MMU_LD_TLB);
}

static int dsp_mmu_load_tlb(struct tlb_entry *tlb_ent)
{
	struct tlb_lock tlb_lock;
	struct cam_ram_regset cr;

#ifdef CONFIG_ARCH_OMAP1
	clk_enable(dsp_ck_handle);
	omap_dsp_request_mem();
#endif

	get_tlb_lock(&tlb_lock);
	for (tlb_lock.victim = 0;
	     tlb_lock.victim < tlb_lock.base;
	     tlb_lock.victim++) {
		struct cam_ram_regset tmp_cr;

		/* read a TLB entry */
		__read_tlb(&tlb_lock, &tmp_cr);
		if (!cam_ram_valid(tmp_cr))
			goto found_victim;
	}
	set_tlb_lock(&tlb_lock);

found_victim:
	/* The last (31st) entry cannot be locked? */
	if (tlb_lock.victim == 31) {
		printk(KERN_ERR "omapdsp: TLB is full.\n");
		return -EBUSY;
	}

	if (tlb_ent->va & ~get_cam_va_mask(tlb_ent->pgsz)) {
		printk(KERN_ERR
		       "omapdsp: mapping vadr (0x%06x) is not "
		       "aligned boundary\n", tlb_ent->va);
		return -EINVAL;
	}

#if defined(CONFIG_ARCH_OMAP1)
	cr.cam_h = tlb_ent->va >> 22;
	cr.cam_l = (tlb_ent->va >> 6 & get_cam_l_va_mask(tlb_ent->pgsz)) |
		   tlb_ent->prsvd | tlb_ent->pgsz;
	cr.ram_h = tlb_ent->pa >> 16;
	cr.ram_l = (tlb_ent->pa & DSP_MMU_RAM_L_RAM_LSB_MASK) | tlb_ent->ap;
#elif defined(CONFIG_ARCH_OMAP2)
	cr.cam = (tlb_ent->va & DSP_MMU_CAM_VATAG_MASK) |
		 tlb_ent->prsvd | tlb_ent->pgsz;
	cr.ram = tlb_ent->pa | tlb_ent->endian | tlb_ent->elsz;
#endif
	__load_tlb(&cr);

	/* update lock base */
	if (tlb_lock.victim == tlb_lock.base)
		tlb_lock.base++;
	tlb_lock.victim = tlb_lock.base;
	set_tlb_lock(&tlb_lock);

#ifdef CONFIG_ARCH_OMAP1
	omap_dsp_release_mem();
	clk_disable(dsp_ck_handle);
#endif
	return 0;
}

static int dsp_mmu_clear_tlb(dsp_long_t vadr)
{
	struct tlb_lock tlb_lock;
	int i;
	int max_valid = 0;

#ifdef CONFIG_ARCH_OMAP1
	clk_enable(dsp_ck_handle);
	omap_dsp_request_mem();
#endif

	get_tlb_lock(&tlb_lock);
	for (i = 0; i < tlb_lock.base; i++) {
		struct cam_ram_regset cr;
		dsp_long_t cam_va;
		dsp_mmu_reg_t pgsz;

		/* read a TLB entry */
		tlb_lock.victim = i;
		__read_tlb(&tlb_lock, &cr);
		if (!cam_ram_valid(cr))
			continue;

#if defined(CONFIG_ARCH_OMAP1)
		pgsz = cr.cam_l & DSP_MMU_CAM_PAGESIZE_MASK;
		cam_va = (u32)(cr.cam_h & DSP_MMU_CAM_H_VA_TAG_H_MASK) << 22 |
			 (u32)(cr.cam_l & get_cam_l_va_mask(pgsz)) << 6;
#elif defined(CONFIG_ARCH_OMAP2)
		pgsz = cr.cam & DSP_MMU_CAM_PAGESIZE_MASK;
		cam_va = cr.cam & get_cam_va_mask(pgsz);
#endif

		if (cam_va == vadr)
			/* flush the entry */
			dsp_mmu_flush();
		else
			max_valid = i;
	}

	/* set new lock base */
	tlb_lock.base   = max_valid + 1;
	tlb_lock.victim = max_valid + 1;
	set_tlb_lock(&tlb_lock);

#ifdef CONFIG_ARCH_OMAP1
	omap_dsp_release_mem();
	clk_disable(dsp_ck_handle);
#endif
	return 0;
}

static void dsp_mmu_gflush(void)
{
	struct tlb_lock tlb_lock;

#ifdef CONFIG_ARCH_OMAP1
	clk_enable(dsp_ck_handle);
	omap_dsp_request_mem();
#endif

	__dsp_mmu_gflush();
	tlb_lock.base   = exmap_preserved_cnt;
	tlb_lock.victim = exmap_preserved_cnt;
	set_tlb_lock(&tlb_lock);

#ifdef CONFIG_ARCH_OMAP1
	omap_dsp_release_mem();
	clk_disable(dsp_ck_handle);
#endif
}

/*
 * dsp_exmap()
 *
 * MEM_IOCTL_EXMAP ioctl calls this function with padr=0.
 * In this case, the buffer for DSP is allocated in this routine,
 * then it is mapped.
 * On the other hand, for example - frame buffer sharing, calls
 * this function with padr set. It means some known address space
 * pointed with padr is going to be shared with DSP.
 */
static int dsp_exmap(dsp_long_t dspadr, unsigned long padr, unsigned long size,
		     enum exmap_type_e type)
{
	dsp_mmu_reg_t pgsz;
	void *buf;
	unsigned int order = 0;
	unsigned long unit;
	int prev = -1;
	dsp_long_t _dspadr = dspadr;
	unsigned long _padr = padr;
	void *_vadr = dspbyte_to_virt(dspadr);
	unsigned long _size = size;
	struct tlb_entry tlb_ent;
	struct exmap_tbl_entry *exmap_ent;
	int status;
	int idx;
	int i;

#define MINIMUM_PAGESZ	SZ_4KB
	/*
	 * alignment check
	 */
	if (!is_aligned(size, MINIMUM_PAGESZ)) {
		printk(KERN_ERR
		       "omapdsp: size(0x%lx) is not multiple of 4KB.\n", size);
		return -EINVAL;
	}
	if (!is_aligned(dspadr, MINIMUM_PAGESZ)) {
		printk(KERN_ERR
		       "omapdsp: DSP address(0x%x) is not aligned.\n", dspadr);
		return -EINVAL;
	}
	if (!is_aligned(padr, MINIMUM_PAGESZ)) {
		printk(KERN_ERR
		       "omapdsp: physical address(0x%lx) is not aligned.\n",
		       padr);
		return -EINVAL;
	}

	/* address validity check */
	if ((dspadr < dspmem_size) ||
	    (dspadr >= DSPSPACE_SIZE) ||
	    ((dspadr + size > DSP_INIT_PAGE) &&
	     (dspadr < DSP_INIT_PAGE + PAGE_SIZE))) {
		printk(KERN_ERR
		       "omapdsp: illegal address/size for dsp_exmap().\n");
		return -EINVAL;
	}

	down_write(&exmap_sem);

	/* overlap check */
	for (i = 0; i < DSP_MMU_TLB_LINES; i++) {
		unsigned long mapsize;
		struct exmap_tbl_entry *tmp_ent = &exmap_tbl[i];

		if (!tmp_ent->valid)
			continue;
		mapsize = 1 << (tmp_ent->order + PAGE_SHIFT);
		if ((_vadr + size > tmp_ent->vadr) &&
		    (_vadr < tmp_ent->vadr + mapsize)) {
			printk(KERN_ERR "omapdsp: exmap page overlap!\n");
			up_write(&exmap_sem);
			return -EINVAL;
		}
	}

start:
	buf = NULL;
	/* Are there any free TLB lines?  */
	for (idx = 0; idx < DSP_MMU_TLB_LINES; idx++) {
		if (!exmap_tbl[idx].valid)
			goto found_free;
	}
	printk(KERN_ERR "omapdsp: DSP TLB is full.\n");
	status = -EBUSY;
	goto fail;

found_free:
	exmap_ent = &exmap_tbl[idx];

	/*
	 * we don't use
	 * 1KB mapping in OMAP1,
	 * 16MB mapping in OMAP2.
	 */
	if ((_size >= SZ_1MB) &&
	    (is_aligned(_padr, SZ_1MB) || (padr == 0)) &&
	    is_aligned(_dspadr, SZ_1MB)) {
		unit = SZ_1MB;
		pgsz = DSP_MMU_CAM_PAGESIZE_1MB;
	} else if ((_size >= SZ_64KB) &&
		   (is_aligned(_padr, SZ_64KB) || (padr == 0)) &&
		   is_aligned(_dspadr, SZ_64KB)) {
		unit = SZ_64KB;
		pgsz = DSP_MMU_CAM_PAGESIZE_64KB;
	} else {
		unit = SZ_4KB;
		pgsz = DSP_MMU_CAM_PAGESIZE_4KB;
	}

	order = get_order(unit);

	/* buffer allocation */
	if (type == EXMAP_TYPE_MEM) {
		struct page *page, *ps, *pe;

		if ((order == ORDER_1MB) && likely(kmem_pool_1M))
			buf = mempool_alloc_from_pool(kmem_pool_1M, GFP_KERNEL);
		else if ((order == ORDER_64KB) && likely(kmem_pool_64K))
			buf = mempool_alloc_from_pool(kmem_pool_64K,GFP_KERNEL);
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
	 * since this area should not be cashed.
	 */
	status = exmap_set_armmmu((unsigned long)_vadr, _padr, unit);
	if (status < 0)
		goto fail;

	/* loading DSP TLB entry */
	INIT_TLB_ENTRY(&tlb_ent, _dspadr, _padr, pgsz);
	status = dsp_mmu_load_tlb(&tlb_ent);
	if (status < 0) {
		exmap_clear_armmmu((unsigned long)_vadr, unit);
		goto fail;
	}

	INIT_EXMAP_TBL_ENTRY(exmap_ent, buf, _vadr, type, order);
	exmap_ent->link.prev = prev;
	if (prev >= 0)
		exmap_tbl[prev].link.next = idx;

	if ((_size -= unit) == 0) {	/* normal completion */
		up_write(&exmap_sem);
		return size;
	}

	_dspadr += unit;
	_vadr   += unit;
	_padr = padr ? _padr + unit : 0;
	prev = idx;
	goto start;

fail:
	up_write(&exmap_sem);
	if (buf)
		dsp_mem_free_pages((unsigned long)buf, order);
	dsp_exunmap(dspadr);
	return status;
}

static unsigned long unmap_free_arm(struct exmap_tbl_entry *ent)
{
	unsigned long size;

	/* clearing ARM MMU */
	size = 1 << (ent->order + PAGE_SHIFT);
	exmap_clear_armmmu((unsigned long)ent->vadr, size);

	/* freeing allocated memory */
	if (ent->type == EXMAP_TYPE_MEM) {
		dsp_mem_free_pages((unsigned long)ent->buf, ent->order);
		printk(KERN_DEBUG
		       "omapdsp: freeing 0x%lx bytes @ adr 0x%8p\n",
		       size, ent->buf);
	}
#ifdef CONFIG_FB_OMAP_LCDC_EXTERNAL
	else if (ent->type == EXMAP_TYPE_FB) {
		int status;
		if (omapfb_nb) {
			status = omapfb_unregister_client(omapfb_nb);
			if (!status)
				printk("omapfb_unregister_client(): "
				       "success\n");
			else
				printk("omapfb_runegister_client(): "
				       "failure(%d)\n", status);
			kfree(omapfb_nb);
			omapfb_nb = NULL;
			omapfb_ready = 0;
		}
	}
#endif

	return size;
}

static int dsp_exunmap(dsp_long_t dspadr)
{
	void *vadr;
	unsigned long size;
	int total = 0;
	struct exmap_tbl_entry *ent;
	int idx;

	vadr = dspbyte_to_virt(dspadr);
	down_write(&exmap_sem);
	for (idx = 0; idx < DSP_MMU_TLB_LINES; idx++) {
		ent = &exmap_tbl[idx];
		if ((!ent->valid) || ent->prsvd)
			continue;
		if (ent->vadr == vadr)
			goto found_map;
	}
	up_write(&exmap_sem);
	printk(KERN_WARNING
	       "omapdsp: address %06x not found in exmap_tbl.\n", dspadr);
	return -EINVAL;

found_map:
	if (ent->usecount > 0) {
		printk(KERN_ERR
		       "omapdsp: exmap reference count is not 0.\n"
		       "   idx=%d, vadr=%p, order=%d, usecount=%d\n",
		       idx, ent->vadr, ent->order, ent->usecount);
		up_write(&exmap_sem);
		return -EINVAL;
	}
	/* clearing DSP TLB entry */
	dsp_mmu_clear_tlb(dspadr);

	/* clear ARM MMU and free buffer */
	size = unmap_free_arm(ent);
	ent->valid = 0;
	total += size;

	/* we don't free PTEs */

	/* flush TLB */
	flush_tlb_kernel_range((unsigned long)vadr, (unsigned long)vadr + size);

	if ((idx = ent->link.next) < 0)
		goto up_out;	/* normal completion */
	ent = &exmap_tbl[idx];
	dspadr += size;
	vadr   += size;
	if (ent->vadr == vadr)
		goto found_map;	/* continue */

	printk(KERN_ERR
	       "omapdsp: illegal exmap_tbl grouping!\n"
	       "expected vadr = %p, exmap_tbl[%d].vadr = %p\n",
	       vadr, idx, ent->vadr);
	up_write(&exmap_sem);
	return -EINVAL;

up_out:
	up_write(&exmap_sem);
	return total;
}

static void exmap_flush(void)
{
	struct exmap_tbl_entry *ent;
	int i;

	down_write(&exmap_sem);

	/* clearing DSP TLB entry */
	dsp_mmu_gflush();

	for (i = 0; i < DSP_MMU_TLB_LINES; i++) {
		ent = &exmap_tbl[i];
		if (ent->valid && (!ent->prsvd)) {
			unmap_free_arm(ent);
			ent->valid = 0;
		}
	}

	/* flush TLB */
	flush_tlb_kernel_range(dspmem_base + dspmem_size,
			       dspmem_base + DSPSPACE_SIZE);
	up_write(&exmap_sem);
}

#ifdef CONFIG_OMAP_DSP_FBEXPORT
#ifndef CONFIG_FB
#error You configured OMAP_DSP_FBEXPORT, but FB was not configured!
#endif /* CONFIG_FB */

#ifdef CONFIG_FB_OMAP_LCDC_EXTERNAL
static int omapfb_notifier_cb(struct notifier_block *omapfb_nb,
			      unsigned long event, void *fbi)
{
	/* XXX */
	printk("omapfb_notifier_cb(): event = %s\n",
	       (event == OMAPFB_EVENT_READY)    ? "READY" :
	       (event == OMAPFB_EVENT_DISABLED) ? "DISABLED" : "Unknown");
	if (event == OMAPFB_EVENT_READY)
		omapfb_ready = 1;
	else if (event == OMAPFB_EVENT_DISABLED)
		omapfb_ready = 0;
	return 0;
}
#endif

static int dsp_fbexport(dsp_long_t *dspadr)
{
	dsp_long_t dspadr_actual;
	unsigned long padr_sys, padr, fbsz_sys, fbsz;
	int cnt;
#ifdef CONFIG_FB_OMAP_LCDC_EXTERNAL
	int status;
#endif

	printk(KERN_DEBUG "omapdsp: frame buffer export\n");

#ifdef CONFIG_FB_OMAP_LCDC_EXTERNAL
	if (omapfb_nb) {
		printk(KERN_WARNING
		       "omapdsp: frame buffer has been exported already!\n");
		return -EBUSY;
	}
#endif

	if (num_registered_fb == 0) {
		printk(KERN_INFO "omapdsp: frame buffer not registered.\n");
		return -EINVAL;
	}
	if (num_registered_fb != 1) {
		printk(KERN_INFO
		       "omapdsp: %d frame buffers found. we use first one.\n",
		       num_registered_fb);
	}
	padr_sys = registered_fb[0]->fix.smem_start;
	fbsz_sys = registered_fb[0]->fix.smem_len;
	if (fbsz_sys == 0) {
		printk(KERN_ERR
		       "omapdsp: framebuffer doesn't seem to be configured "
		       "correctly! (size=0)\n");
		return -EINVAL;
	}

	/*
	 * align padr and fbsz to 4kB boundary
	 * (should be noted to the user afterwards!)
	 */
	padr = padr_sys & ~(SZ_4KB-1);
	fbsz = (fbsz_sys + padr_sys - padr + SZ_4KB-1) & ~(SZ_4KB-1);

	/* line up dspadr offset with padr */
	dspadr_actual =
		(fbsz > SZ_1MB) ?  lineup_offset(*dspadr, padr, SZ_1MB-1) :
		(fbsz > SZ_64KB) ? lineup_offset(*dspadr, padr, SZ_64KB-1) :
		/* (fbsz > SZ_4KB) ? */ *dspadr;
	if (dspadr_actual != *dspadr)
		printk(KERN_DEBUG
		       "omapdsp: actual dspadr for FBEXPORT = %08x\n",
		       dspadr_actual);
	*dspadr = dspadr_actual;

	cnt = dsp_exmap(dspadr_actual, padr, fbsz, EXMAP_TYPE_FB);
	if (cnt < 0) {
		printk(KERN_ERR "omapdsp: exmap failure.\n");
		return cnt;
	}

	if ((padr != padr_sys) || (fbsz != fbsz_sys)) {
		printk(KERN_WARNING
"  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
"  !!  screen base address or size is not aligned in 4kB:           !!\n"
"  !!    actual screen  adr = %08lx, size = %08lx             !!\n"
"  !!    exporting      adr = %08lx, size = %08lx             !!\n"
"  !!  Make sure that the framebuffer is allocated with 4kB-order!  !!\n"
"  !!  Otherwise DSP can corrupt the kernel memory.                 !!\n"
"  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n",
		       padr_sys, fbsz_sys, padr, fbsz);
	}

#ifdef CONFIG_ARCH_OMAP1
	/* increase the DMA priority */
	set_emiff_dma_prio(15);
#endif

#ifdef CONFIG_FB_OMAP_LCDC_EXTERNAL
	omapfb_nb = kmalloc(sizeof(struct omapfb_notifier_block), GFP_KERNEL);
	if (omapfb_nb == NULL) {
		printk(KERN_ERR
		       "omapdsp: failed to allocate memory for omapfb_nb!\n");
		dsp_exunmap(dspadr_actual);
		return -ENOMEM;
	}
	status = omapfb_register_client(omapfb_nb, omapfb_notifier_cb, NULL);
	if (!status)
		printk("omapfb_register_client(): success\n");
	else
		printk("omapfb_register_client(): failure(%d)\n", status);
#endif

	return cnt;
}

#else /* CONFIG_OMAP_DSP_FBEXPORT */

static int dsp_fbexport(dsp_long_t *dspadr)
{
	printk(KERN_ERR "omapdsp: FBEXPORT function is not enabled.\n");
	return -EINVAL;
}

#endif /* CONFIG_OMAP_DSP_FBEXPORT */

static void exmap_setup_preserved_mem_page(void *buf, dsp_long_t dspadr,
					   int exmap_idx)
{
	unsigned long phys;
	void *virt;
	struct tlb_entry tlb_ent;

	phys = __pa(buf);
	virt = dspbyte_to_virt(dspadr);
	exmap_set_armmmu((unsigned long)virt, phys, PAGE_SIZE);
	INIT_EXMAP_TBL_ENTRY_4KB_PRESERVED(&exmap_tbl[exmap_idx], buf, virt);
	INIT_TLB_ENTRY_4KB_PRESERVED(&tlb_ent, dspadr, phys);
	dsp_mmu_load_tlb(&tlb_ent);
}

static void exmap_clear_mem_page(dsp_long_t dspadr)
{
	void *virt;

	virt = dspbyte_to_virt(dspadr);
	exmap_clear_armmmu((unsigned long)virt, PAGE_SIZE);
	/* DSP MMU is shutting down. not handled here. */
}

#ifdef CONFIG_ARCH_OMAP2
static void exmap_setup_iomap_page(unsigned long phys, unsigned long dsp_io_adr,
				   int exmap_idx)
{
	dsp_long_t dspadr;
	void *virt;
	struct tlb_entry tlb_ent;

	dspadr = (IOMAP_VAL << 18) + (dsp_io_adr << 1);
	virt = dspbyte_to_virt(dspadr);
	exmap_set_armmmu((unsigned long)virt, phys, PAGE_SIZE);
	INIT_EXMAP_TBL_ENTRY_4KB_PRESERVED(&exmap_tbl[exmap_idx], NULL, virt);
	INIT_TLB_ENTRY_4KB_ES32_PRESERVED(&tlb_ent, dspadr, phys);
	dsp_mmu_load_tlb(&tlb_ent);
}

static void exmap_clear_iomap_page(unsigned long dsp_io_adr)
{
	dsp_long_t dspadr;
	void *virt;

	dspadr = (IOMAP_VAL << 18) + (dsp_io_adr << 1);
	virt = dspbyte_to_virt(dspadr);
	exmap_clear_armmmu((unsigned long)virt, PAGE_SIZE);
	/* DSP MMU is shutting down. not handled here. */
}
#endif /* CONFIG_ARCH_OMAP2 */

#define OMAP2420_GPT5_BASE	(L4_24XX_BASE + 0x7c000)
#define OMAP2420_GPT6_BASE	(L4_24XX_BASE + 0x7e000)
#define OMAP2420_GPT7_BASE	(L4_24XX_BASE + 0x80000)
#define OMAP2420_GPT8_BASE	(L4_24XX_BASE + 0x82000)
#define OMAP24XX_EAC_BASE	(L4_24XX_BASE + 0x90000)

static int exmap_setup_preserved_entries(void)
{
	int n = 0;

	exmap_setup_preserved_mem_page(dspvect_page, DSP_INIT_PAGE, n++);
#ifdef CONFIG_ARCH_OMAP2
	exmap_setup_iomap_page(OMAP24XX_PRCM_BASE,     0x7000, n++);
#ifdef CONFIG_ARCH_OMAP2420
	exmap_setup_iomap_page(OMAP2420_GPT5_BASE,     0xe000, n++);
	exmap_setup_iomap_page(OMAP2420_GPT6_BASE,     0xe800, n++);
	exmap_setup_iomap_page(OMAP2420_GPT7_BASE,     0xf000, n++);
	exmap_setup_iomap_page(OMAP2420_GPT8_BASE,     0xf800, n++);
#endif /* CONFIG_ARCH_OMAP2420 */
	exmap_setup_iomap_page(OMAP24XX_EAC_BASE,     0x10000, n++);
	exmap_setup_iomap_page(OMAP24XX_MAILBOX_BASE, 0x11000, n++);
#endif /* CONFIG_ARCH_OMAP2 */

	return n;
}

static void exmap_clear_preserved_entries(void)
{
	exmap_clear_mem_page(DSP_INIT_PAGE);
#ifdef CONFIG_ARCH_OMAP2
	exmap_clear_iomap_page(0x7000);		/* PRCM */
#ifdef CONFIG_ARCH_OMAP2420
	exmap_clear_iomap_page(0xe000);		/* GPT5 */
	exmap_clear_iomap_page(0xe800);		/* GPT6 */
	exmap_clear_iomap_page(0xf000);		/* GPT7 */
	exmap_clear_iomap_page(0xf800);		/* GPT8 */
#endif /* CONFIG_ARCH_OMAP2420 */
	exmap_clear_iomap_page(0x10000);	/* EAC */
	exmap_clear_iomap_page(0x11000);	/* MAILBOX */
#endif /* CONFIG_ARCH_OMAP2 */
}

#ifdef CONFIG_ARCH_OMAP1
static int dsp_mmu_itack(void)
{
	unsigned long dspadr;

	printk(KERN_INFO "omapdsp: sending DSP MMU interrupt ack.\n");
	if (!dsp_err_isset(ERRCODE_MMU)) {
		printk(KERN_ERR "omapdsp: DSP MMU error has not been set.\n");
		return -EINVAL;
	}
	dspadr = dsp_fault_adr & ~(SZ_4K-1);
	dsp_exmap(dspadr, 0, SZ_4K, EXMAP_TYPE_MEM);	/* FIXME: reserve TLB entry for this */
	printk(KERN_INFO "omapdsp: falling into recovery runlevel...\n");
	dsp_set_runlevel(RUNLEVEL_RECOVERY);
	__dsp_mmu_itack();
	udelay(100);
	dsp_exunmap(dspadr);
	dsp_err_clear(ERRCODE_MMU);
	return 0;
}
#endif /* CONFIG_ARCH_OMAP1 */

#ifdef CONFIG_ARCH_OMAP2
#define MMU_IRQ_MASK \
	(DSP_MMU_IRQ_MULTIHITFAULT | \
	 DSP_MMU_IRQ_TABLEWALKFAULT | \
	 DSP_MMU_IRQ_EMUMISS | \
	 DSP_MMU_IRQ_TRANSLATIONFAULT | \
	 DSP_MMU_IRQ_TLBMISS)
#endif

static void dsp_mmu_init(void)
{
	struct tlb_lock tlb_lock;

#ifdef CONFIG_ARCH_OMAP1
	clk_enable(dsp_ck_handle);
	omap_dsp_request_mem();
#endif
	down_write(&exmap_sem);

#if defined(CONFIG_ARCH_OMAP1)
	dsp_mmu_disable();	/* clear all */
	udelay(100);
#elif defined(CONFIG_ARCH_OMAP2)
	dsp_mmu_reset();
#endif
	dsp_mmu_enable();

	/* DSP TLB initialization */
	tlb_lock.base   = 0;
	tlb_lock.victim = 0;
	set_tlb_lock(&tlb_lock);

	exmap_preserved_cnt = exmap_setup_preserved_entries();

#ifdef CONFIG_ARCH_OMAP2
	/* MMU IRQ mask setup */
	dsp_mmu_write_reg(MMU_IRQ_MASK, DSP_MMU_IRQENABLE);
#endif

	up_write(&exmap_sem);
#ifdef CONFIG_ARCH_OMAP1
	omap_dsp_release_mem();
	clk_disable(dsp_ck_handle);
#endif
}

static void dsp_mmu_shutdown(void)
{
	exmap_flush();
	exmap_clear_preserved_entries();
	dsp_mmu_disable();
}

#ifdef CONFIG_ARCH_OMAP1
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
#endif /* CONFIG_ARCH_OMAP1 */

/*
 * dsp_mem_enable() / disable()
 */
#ifdef CONFIG_ARCH_OMAP1
int intmem_usecount;
#endif

int dsp_mem_enable(void *adr)
{
	int ret = 0;

	if (is_dsp_internal_mem(adr)) {
#ifdef CONFIG_ARCH_OMAP1
		if (intmem_usecount++ == 0)
			ret = omap_dsp_request_mem();
#endif
	} else
		down_read(&exmap_sem);

	return ret;
}

void dsp_mem_disable(void *adr)
{
	if (is_dsp_internal_mem(adr)) {
#ifdef CONFIG_ARCH_OMAP1
		if (--intmem_usecount == 0)
			omap_dsp_release_mem();
#endif
	} else
		up_read(&exmap_sem);
}

/* for safety */
#ifdef CONFIG_ARCH_OMAP1
void dsp_mem_usecount_clear(void)
{
	if (intmem_usecount != 0) {
		printk(KERN_WARNING
		       "omapdsp: unbalanced memory request/release detected.\n"
		       "         intmem_usecount is not zero at where "
		       "it should be! ... fixed to be zero.\n");
		intmem_usecount = 0;
		omap_dsp_release_mem();
	}
}
#endif /* CONFIG_ARCH_OMAP1 */

/*
 * dsp_mem file operations
 */
static loff_t dsp_mem_lseek(struct file *file, loff_t offset, int orig)
{
	loff_t ret;

	mutex_lock(&file->f_dentry->d_inode->i_mutex);
	switch (orig) {
	case 0:
		file->f_pos = offset;
		ret = file->f_pos;
		break;
	case 1:
		file->f_pos += offset;
		ret = file->f_pos;
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&file->f_dentry->d_inode->i_mutex);
	return ret;
}

static ssize_t intmem_read(struct file *file, char __user *buf, size_t count,
			   loff_t *ppos)
{
	unsigned long p = *ppos;
	void *vadr = dspbyte_to_virt(p);
	ssize_t size = dspmem_size;
	ssize_t read;

	if (p >= size)
		return 0;
#ifdef CONFIG_ARCH_OMAP1
	clk_enable(api_ck_handle);
#endif
	read = count;
	if (count > size - p)
		read = size - p;
	if (copy_to_user(buf, vadr, read)) {
		read = -EFAULT;
		goto out;
	}
	*ppos += read;
out:
#ifdef CONFIG_ARCH_OMAP1
	clk_disable(api_ck_handle);
#endif
	return read;
}

static ssize_t exmem_read(struct file *file, char __user *buf, size_t count,
			  loff_t *ppos)
{
	unsigned long p = *ppos;
	void *vadr = dspbyte_to_virt(p);

	if (!exmap_valid(vadr, count)) {
		printk(KERN_ERR
		       "omapdsp: DSP address %08lx / size %08x "
		       "is not valid!\n", p, count);
		return -EFAULT;
	}
	if (count > DSPSPACE_SIZE - p)
		count = DSPSPACE_SIZE - p;
	if (copy_to_user(buf, vadr, count))
		return -EFAULT;
	*ppos += count;

	return count;
}

static ssize_t dsp_mem_read(struct file *file, char __user *buf, size_t count,
			    loff_t *ppos)
{
	int ret;
	void *vadr = dspbyte_to_virt(*(unsigned long *)ppos);

	if (dsp_mem_enable(vadr) < 0)
		return -EBUSY;
	if (is_dspbyte_internal_mem(*ppos))
		ret = intmem_read(file, buf, count, ppos);
	else
		ret = exmem_read(file, buf, count, ppos);
	dsp_mem_disable(vadr);

	return ret;
}

static ssize_t intmem_write(struct file *file, const char __user *buf,
			    size_t count, loff_t *ppos)
{
	unsigned long p = *ppos;
	void *vadr = dspbyte_to_virt(p);
	ssize_t size = dspmem_size;
	ssize_t written;

	if (p >= size)
		return 0;
#ifdef CONFIG_ARCH_OMAP1
	clk_enable(api_ck_handle);
#endif
	written = count;
	if (count > size - p)
		written = size - p;
	if (copy_from_user(vadr, buf, written)) {
		written = -EFAULT;
		goto out;
	}
	*ppos += written;
out:
#ifdef CONFIG_ARCH_OMAP1
	clk_disable(api_ck_handle);
#endif
	return written;
}

static ssize_t exmem_write(struct file *file, const char __user *buf,
			   size_t count, loff_t *ppos)
{
	unsigned long p = *ppos;
	void *vadr = dspbyte_to_virt(p);

	if (!exmap_valid(vadr, count)) {
		printk(KERN_ERR
		       "omapdsp: DSP address %08lx / size %08x "
		       "is not valid!\n", p, count);
		return -EFAULT;
	}
	if (count > DSPSPACE_SIZE - p)
		count = DSPSPACE_SIZE - p;
	if (copy_from_user(vadr, buf, count))
		return -EFAULT;
	*ppos += count;

	return count;
}

static ssize_t dsp_mem_write(struct file *file, const char __user *buf,
			     size_t count, loff_t *ppos)
{
	int ret;
	void *vadr = dspbyte_to_virt(*(unsigned long *)ppos);

	if (dsp_mem_enable(vadr) < 0)
		return -EBUSY;
	if (is_dspbyte_internal_mem(*ppos))
		ret = intmem_write(file, buf, count, ppos);
	else
		ret = exmem_write(file, buf, count, ppos);
	dsp_mem_disable(vadr);

	return ret;
}

static int dsp_mem_ioctl(struct inode *inode, struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	case MEM_IOCTL_MMUINIT:
		dsp_mmu_init();
		return 0;

	case MEM_IOCTL_EXMAP:
		{
			struct omap_dsp_mapinfo mapinfo;
			if (copy_from_user(&mapinfo, (void __user *)arg,
					   sizeof(mapinfo)))
				return -EFAULT;
			return dsp_exmap(mapinfo.dspadr, 0, mapinfo.size,
					 EXMAP_TYPE_MEM);
		}

	case MEM_IOCTL_EXUNMAP:
		return dsp_exunmap((unsigned long)arg);

	case MEM_IOCTL_EXMAP_FLUSH:
		exmap_flush();
		return 0;

	case MEM_IOCTL_FBEXPORT:
		{
			dsp_long_t dspadr;
			int ret;
			if (copy_from_user(&dspadr, (void __user *)arg,
					   sizeof(dsp_long_t)))
				return -EFAULT;
			ret = dsp_fbexport(&dspadr);
			if (copy_to_user((void __user *)arg, &dspadr,
					 sizeof(dsp_long_t)))
				return -EFAULT;
			return ret;
		}

#ifdef CONFIG_ARCH_OMAP1
	case MEM_IOCTL_MMUITACK:
		return dsp_mmu_itack();
#endif

	case MEM_IOCTL_KMEM_RESERVE:
		{
			__u32 size;
			if (copy_from_user(&size, (void __user *)arg,
					   sizeof(__u32)))
				return -EFAULT;
			return dsp_kmem_reserve(size);
		}

	case MEM_IOCTL_KMEM_RELEASE:
		dsp_kmem_release();
		return 0;

	default:
		return -ENOIOCTLCMD;
	}
}

static int dsp_mem_mmap(struct file *file, struct vm_area_struct *vma)
{
	/*
	 * FIXME
	 */
	return -ENOSYS;
}

static int dsp_mem_open(struct inode *inode, struct file *file)
{
	if (!capable(CAP_SYS_RAWIO))
		return -EPERM;

	return 0;
}

#ifdef CONFIG_FB_OMAP_LCDC_EXTERNAL
/*
 * fb update functions:
 * fbupd_response() is executed by the workqueue.
 * fbupd_cb() is called when fb update is done, in interrupt context.
 * mbox_fbupd() is called when KFUNC:FBCTL:UPD is received from DSP.
 */
static void fbupd_response(void *arg)
{
	int status;

	status = mbcompose_send(KFUNC, KFUNC_FBCTL, FBCTL_UPD);
	if (status < 0) {
		/* FIXME: DSP is busy !! */
		printk(KERN_ERR
		       "omapdsp: DSP is busy when trying to send FBCTL:UPD "
		       "response!\n");
	}
}

static DECLARE_WORK(fbupd_response_work, (void (*)(void *))fbupd_response,
		    NULL);

static void fbupd_cb(void *arg)
{
	schedule_work(&fbupd_response_work);
}

void mbox_fbctl_upd(void)
{
	struct omapfb_update_window win;
	volatile unsigned short *buf = ipbuf_sys_da->d;

	/* FIXME: try count sometimes exceeds 1000. */
	if (sync_with_dsp(&ipbuf_sys_da->s, TID_ANON, 5000) < 0) {
		printk(KERN_ERR "mbox: FBCTL:UPD - IPBUF sync failed!\n");
		return;
	}
	win.x = buf[0];
	win.y = buf[1];
	win.width = buf[2];
	win.height = buf[3];
	win.format = buf[4];
	release_ipbuf_pvt(ipbuf_sys_da);

	if (!omapfb_ready) {
		printk(KERN_WARNING
		       "omapdsp: fbupd() called while HWA742 is not ready!\n");
		return;
	}
	//printk("calling omapfb_update_window_async()\n");
	omapfb_update_window_async(registered_fb[1], &win, fbupd_cb, NULL);
}

#else /* CONFIG_FB_OMAP_LCDC_EXTERNAL */

void mbox_fbctl_upd(void)
{
}
#endif /* CONFIG_FB_OMAP_LCDC_EXTERNAL */

/*
 * sysfs files
 */

/* mmu */
static ssize_t mmu_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	int len;
	struct tlb_lock tlb_lock_org;
	int i;

#ifdef CONFIG_ARCH_OMAP1
	clk_enable(dsp_ck_handle);
	omap_dsp_request_mem();
#endif
	down_read(&exmap_sem);

	get_tlb_lock(&tlb_lock_org);

#if defined(CONFIG_ARCH_OMAP1)
	len = sprintf(buf, "P: preserved, V: valid\n"
			   "ety P V size   cam_va     ram_pa ap\n");
			 /* 00: P V  4KB 0x300000 0x10171800 FA */
#elif defined(CONFIG_ARCH_OMAP2)
	len = sprintf(buf, "P: preserved, V: valid\n"
			   "B: big endian, L:little endian, "
			   "M: mixed page attribute\n"
			   "ety P V size   cam_va     ram_pa E ES M\n");
			 /* 00: P V  4KB 0x300000 0x10171800 B 16 M */
#endif

	for (i = 0; i < DSP_MMU_TLB_LINES; i++) {
		struct cam_ram_regset cr;
		struct tlb_lock tlb_lock_tmp;
		struct tlb_entry ent;
#if defined(CONFIG_ARCH_OMAP1)
		char *pgsz_str, *ap_str;
#elif defined(CONFIG_ARCH_OMAP2)
		char *pgsz_str, *elsz_str;
#endif

		/* read a TLB entry */
		tlb_lock_tmp.base   = tlb_lock_org.base;
		tlb_lock_tmp.victim = i;
		__read_tlb(&tlb_lock_tmp, &cr);

#if defined(CONFIG_ARCH_OMAP1)
		ent.pgsz  = cr.cam_l & DSP_MMU_CAM_PAGESIZE_MASK;
		ent.prsvd = cr.cam_l & DSP_MMU_CAM_P;
		ent.valid = cr.cam_l & DSP_MMU_CAM_V;
		ent.ap    = cr.ram_l & DSP_MMU_RAM_L_AP_MASK;
		ent.va = (u32)(cr.cam_h & DSP_MMU_CAM_H_VA_TAG_H_MASK) << 22 |
			 (u32)(cr.cam_l & get_cam_l_va_mask(ent.pgsz)) << 6;
		ent.pa = (unsigned long)cr.ram_h << 16 |
			 (cr.ram_l & DSP_MMU_RAM_L_RAM_LSB_MASK);

		pgsz_str = (ent.pgsz == DSP_MMU_CAM_PAGESIZE_1MB)  ? " 1MB":
			   (ent.pgsz == DSP_MMU_CAM_PAGESIZE_64KB) ? "64KB":
			   (ent.pgsz == DSP_MMU_CAM_PAGESIZE_4KB)  ? " 4KB":
			   (ent.pgsz == DSP_MMU_CAM_PAGESIZE_1KB)  ? " 1KB":
								     " ???";
		ap_str = (ent.ap == DSP_MMU_RAM_L_AP_RO) ? "RO":
			 (ent.ap == DSP_MMU_RAM_L_AP_FA) ? "FA":
			 (ent.ap == DSP_MMU_RAM_L_AP_NA) ? "NA":
							   "??";
#elif defined(CONFIG_ARCH_OMAP2)
		ent.pgsz   = cr.cam & DSP_MMU_CAM_PAGESIZE_MASK;
		ent.prsvd  = cr.cam & DSP_MMU_CAM_P;
		ent.valid  = cr.cam & DSP_MMU_CAM_V;
		ent.va     = cr.cam & DSP_MMU_CAM_VATAG_MASK;
		ent.endian = cr.ram & DSP_MMU_RAM_ENDIANNESS;
		ent.elsz   = cr.ram & DSP_MMU_RAM_ELEMENTSIZE_MASK;
		ent.pa     = cr.ram & DSP_MMU_RAM_PADDR_MASK;
		ent.mixed  = cr.ram & DSP_MMU_RAM_MIXED;

		pgsz_str = (ent.pgsz == DSP_MMU_CAM_PAGESIZE_16MB) ? "64MB":
			   (ent.pgsz == DSP_MMU_CAM_PAGESIZE_1MB)  ? " 1MB":
			   (ent.pgsz == DSP_MMU_CAM_PAGESIZE_64KB) ? "64KB":
			   (ent.pgsz == DSP_MMU_CAM_PAGESIZE_4KB)  ? " 4KB":
								     " ???";
		elsz_str = (ent.elsz == DSP_MMU_RAM_ELEMENTSIZE_8)  ? " 8":
			   (ent.elsz == DSP_MMU_RAM_ELEMENTSIZE_16) ? "16":
			   (ent.elsz == DSP_MMU_RAM_ELEMENTSIZE_32) ? "32":
								      "??";
#endif

		if (i == tlb_lock_org.base)
			len += sprintf(buf + len, "lock base = %d\n",
				       tlb_lock_org.base);
		if (i == tlb_lock_org.victim)
			len += sprintf(buf + len, "victim    = %d\n",
				       tlb_lock_org.victim);
#if defined(CONFIG_ARCH_OMAP1)
		len += sprintf(buf + len,
			       /* 00: P V  4KB 0x300000 0x10171800 FA */
			       "%02d: %c %c %s 0x%06x 0x%08lx %s\n",
			       i,
			       ent.prsvd ? 'P' : ' ',
			       ent.valid ? 'V' : ' ',
			       pgsz_str, ent.va, ent.pa, ap_str);
#elif defined(CONFIG_ARCH_OMAP2)
		len += sprintf(buf + len,
			       /* 00: P V  4KB 0x300000 0x10171800 B 16 M */
			       "%02d: %c %c %s 0x%06x 0x%08lx %c %s %c\n",
			       i,
			       ent.prsvd ? 'P' : ' ',
			       ent.valid ? 'V' : ' ',
			       pgsz_str, ent.va, ent.pa,
			       ent.endian ? 'B' : 'L',
			       elsz_str,
			       ent.mixed ? 'M' : ' ');
#endif /* CONFIG_ARCH_OMAP2 */
	}

	/* restore victim entry */
	set_tlb_lock(&tlb_lock_org);

	up_read(&exmap_sem);
#ifdef CONFIG_ARCH_OMAP1
	omap_dsp_release_mem();
	clk_disable(dsp_ck_handle);
#endif
	return len;
}

/* exmap */
static ssize_t exmap_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	int len;
	int i;

	down_read(&exmap_sem);
	len = sprintf(buf, "  dspadr     size         buf     size uc\n");
			 /* 0x300000 0x123000  0xc0171000 0x100000  0*/
	for (i = 0; i < DSP_MMU_TLB_LINES; i++) {
		struct exmap_tbl_entry *ent = &exmap_tbl[i];
		void *vadr;
		unsigned long size;
		enum exmap_type_e type;
		int idx;

		/* find a top of link */
		if (!ent->valid || (ent->link.prev >= 0))
			continue;

		vadr = ent->vadr;
		type = ent->type;
		size = 0;
		idx = i;
		do {
			ent = &exmap_tbl[idx];
			size += PAGE_SIZE << ent->order;
		} while ((idx = ent->link.next) >= 0);

		len += sprintf(buf + len, "0x%06x %#8lx",
			       virt_to_dspbyte(vadr), size);

		if (type == EXMAP_TYPE_FB) {
			len += sprintf(buf + len, "    framebuf\n");
		} else {
			len += sprintf(buf + len, "\n");
			idx = i;
			do {
				ent = &exmap_tbl[idx];
				len += sprintf(buf + len,
					       /* 0xc0171000 0x100000  0*/
					       "%19s0x%8p %#8lx %2d\n",
					       "", ent->buf,
					       PAGE_SIZE << ent->order,
					       ent->usecount);
			} while ((idx = ent->link.next) >= 0);
		}
	}

	up_read(&exmap_sem);
	return len;
}

/* mempool */
static ssize_t mempool_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	int min_nr_1M = 0, curr_nr_1M = 0;
	int min_nr_64K = 0, curr_nr_64K = 0;
	int total = 0;

	if (likely(kmem_pool_1M)) {
		min_nr_1M  = kmem_pool_1M->min_nr;
		curr_nr_1M = kmem_pool_1M->curr_nr;
		total += min_nr_1M * SZ_1MB;
	}
	if (likely(kmem_pool_64K)) {
		min_nr_64K  = kmem_pool_64K->min_nr;
		curr_nr_64K = kmem_pool_64K->curr_nr;
		total += min_nr_64K * SZ_64KB;
	}

	return sprintf(buf,
		       "0x%x\n"
		       "1M  buffer: %d (%d free)\n"
		       "64K buffer: %d (%d free)\n",
		       total, min_nr_1M, curr_nr_1M, min_nr_64K, curr_nr_64K);
}

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
#endif /* CONFIG_ARCH_OMAP1 */

static void do_mmu_int(void)
{
#if defined(CONFIG_ARCH_OMAP1)

	dsp_mmu_reg_t status;
	dsp_mmu_reg_t adh, adl;
	dsp_mmu_reg_t dp;

	status = dsp_mmu_read_reg(DSP_MMU_FAULT_ST);
	adh = dsp_mmu_read_reg(DSP_MMU_FAULT_AD_H);
	adl = dsp_mmu_read_reg(DSP_MMU_FAULT_AD_L);
	dp = adh & DSP_MMU_FAULT_AD_H_DP;
	dsp_fault_adr = MK32(adh & DSP_MMU_FAULT_AD_H_ADR_MASK, adl);

	/* if the fault is masked, nothing to do */
	if ((status & MMUFAULT_MASK) == 0) {
		printk(KERN_DEBUG "DSP MMU interrupt, but ignoring.\n");
		/*
		 * note: in OMAP1710,
		 * when CACHE + DMA domain gets out of idle in DSP,
		 * MMU interrupt occurs but DSP_MMU_FAULT_ST is not set.
		 * in this case, we just ignore the interrupt.
		 */
		if (status) {
			printk(KERN_DEBUG "%s%s%s%s\n",
			       (status & DSP_MMU_FAULT_ST_PREF)?
					"  (prefetch err)" : "",
			       (status & DSP_MMU_FAULT_ST_PERM)?
					"  (permission fault)" : "",
			       (status & DSP_MMU_FAULT_ST_TLB_MISS)?
					"  (TLB miss)" : "",
			       (status & DSP_MMU_FAULT_ST_TRANS) ?
					"  (translation fault)": "");
			printk(KERN_DEBUG "fault address = %#08x\n",
			       dsp_fault_adr);
		}
		enable_irq(omap_dsp->mmu_irq);
		return;
	}

#elif defined(CONFIG_ARCH_OMAP2)

	dsp_mmu_reg_t status;

	status = dsp_mmu_read_reg(DSP_MMU_IRQSTATUS);
	dsp_fault_adr = dsp_mmu_read_reg(DSP_MMU_FAULT_AD);

#endif /* CONFIG_ARCH_OMAP2 */

	printk(KERN_INFO "DSP MMU interrupt!\n");

#if defined(CONFIG_ARCH_OMAP1)

	printk(KERN_INFO "%s%s%s%s\n",
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

#elif defined(CONFIG_ARCH_OMAP2)

	printk(KERN_INFO "%s%s%s%s%s\n",
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

#endif /* CONFIG_ARCH_OMAP2 */

	printk(KERN_INFO "fault address = %#08x\n", dsp_fault_adr);

	if (dsp_cfgstat_get_stat() == CFGSTAT_READY)
		dsp_err_set(ERRCODE_MMU, (unsigned long)dsp_fault_adr);
	else {
#ifdef CONFIG_ARCH_OMAP1
		__dsp_mmu_itack();
#endif
		printk(KERN_INFO "Resetting DSP...\n");
		dsp_cpustat_request(CPUSTAT_RESET);
		/*
		 * if we enable followings, semaphore lock should be avoided.
		 *
		printk(KERN_INFO "Flushing DSP MMU...\n");
		exmap_flush();
		dsp_mmu_init();
		 */
	}

#ifdef CONFIG_ARCH_OMAP2
	dsp_mmu_disable();
	dsp_mmu_write_reg(status, DSP_MMU_IRQSTATUS);
	dsp_mmu_enable();
#endif

	enable_irq(omap_dsp->mmu_irq);
}

static DECLARE_WORK(mmu_int_work, (void (*)(void *))do_mmu_int, NULL);

/*
 * DSP MMU interrupt handler
 */

static irqreturn_t dsp_mmu_interrupt(int irq, void *dev_id,
				     struct pt_regs *regs)
{
	disable_irq(omap_dsp->mmu_irq);
	schedule_work(&mmu_int_work);
	return IRQ_HANDLED;
}

/*
 *
 */
struct file_operations dsp_mem_fops = {
	.owner   = THIS_MODULE,
	.llseek  = dsp_mem_lseek,
	.read    = dsp_mem_read,
	.write   = dsp_mem_write,
	.ioctl   = dsp_mem_ioctl,
	.mmap    = dsp_mem_mmap,
	.open    = dsp_mem_open,
};

void dsp_mem_start(void)
{
#ifdef CONFIG_ARCH_OMAP1
	dsp_register_mem_cb(intmem_enable, intmem_disable);
#endif
}

void dsp_mem_stop(void)
{
	memset(&mem_sync, 0, sizeof(struct mem_sync_struct));
#ifdef CONFIG_ARCH_OMAP1
	dsp_unregister_mem_cb();
#endif
}

/*
 * later half of dsp memory initialization
 */
void dsp_mem_late_init(void)
{
#ifdef CONFIG_ARCH_OMAP2
	int i;
	int dspmem_pg_count;

	dspmem_pg_count = dspmem_size >> 12;
	for (i = 0; i < dspmem_pg_count; i++) {
		dsp_ipi_write_reg(i, DSP_IPI_INDEX);
		dsp_ipi_write_reg(DSP_IPI_ENTRY_ELMSIZEVALUE_16,
				  DSP_IPI_ENTRY);
	}
	dsp_ipi_write_reg(1, DSP_IPI_ENABLE);
	dsp_ipi_write_reg(IOMAP_VAL, DSP_IPI_IOMAP);
#endif
	dsp_mmu_init();
}

static char devid_mmu;

int __init dsp_mem_init(void)
{
	int i, ret;

	for (i = 0; i < DSP_MMU_TLB_LINES; i++)
		exmap_tbl[i].valid = 0;

	dspvect_page = (void *)__get_dma_pages(GFP_KERNEL, 0);
	if (dspvect_page == NULL) {
		printk(KERN_ERR
		       "omapdsp: failed to allocate memory "
		       "for dsp vector table\n");
		return -ENOMEM;
	}

	/*
	 * DSP MMU interrupt setup
	 */
	ret = request_irq(omap_dsp->mmu_irq, dsp_mmu_interrupt, SA_INTERRUPT,
			  "dsp_mmu",  &devid_mmu);
	if (ret) {
		printk(KERN_ERR
		       "failed to register DSP MMU interrupt: %d\n", ret);
		return ret;
	}

	/* MMU interrupt is not enabled until DSP runs */
	disable_irq(omap_dsp->mmu_irq);

	device_create_file(omap_dsp->dev, &dev_attr_mmu);
	device_create_file(omap_dsp->dev, &dev_attr_exmap);
	device_create_file(omap_dsp->dev, &dev_attr_mempool);

	return 0;
}

void dsp_mem_exit(void)
{
	free_irq(omap_dsp->mmu_irq, &devid_mmu);

	/* recover disable_depth */
	enable_irq(omap_dsp->mmu_irq);

#ifdef CONFIG_ARCH_OMAP1
	dsp_reset_idle_boot_base();
#endif
	dsp_mmu_shutdown();
	dsp_kmem_release();

	if (dspvect_page != NULL) {
		free_page((unsigned long)dspvect_page);
		dspvect_page = NULL;
	}

	device_remove_file(omap_dsp->dev, &dev_attr_mmu);
	device_remove_file(omap_dsp->dev, &dev_attr_exmap);
	device_remove_file(omap_dsp->dev, &dev_attr_mempool);
}
