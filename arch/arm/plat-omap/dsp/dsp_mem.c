/*
 * linux/arch/arm/mach-omap/dsp/dsp_mem.c
 *
 * OMAP DSP memory driver
 *
 * Copyright (C) 2002-2005 Nokia Corporation
 *
 * Written by Toshihiro Kobayashi <toshihiro.kobayashi@nokia.com>
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
 *
 * Toshihiro Kobayashi <toshihiro.kobayashi@nokia.com>
 * 2005/06/09:  DSP Gateway version 3.3
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/major.h>
#include <linux/fs.h>
#include <linux/bootmem.h>
#include <linux/fb.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/ioctls.h>
#include <asm/irq.h>
#include <asm/pgalloc.h>
#include <asm/pgtable.h>
#include <asm/arch/tc.h>
#include <asm/arch/dsp.h>
#include <asm/arch/dsp_common.h>
#include "uaccess_dsp.h"
#include "dsp.h"

#define SZ_1MB	0x100000
#define SZ_64KB	0x10000
#define SZ_4KB	0x1000
#define SZ_1KB	0x400
#define is_aligned(adr,align)	(!((adr)&((align)-1)))
#define ORDER_1MB	(20 - PAGE_SHIFT)
#define ORDER_64KB	(16 - PAGE_SHIFT)
#define ORDER_4KB	(12 - PAGE_SHIFT)

#define PGDIR_MASK		(~(PGDIR_SIZE-1))
#define PGDIR_ALIGN(addr)	(((addr)+PGDIR_SIZE-1)&(PGDIR_MASK))

#define dsp_mmu_enable() \
	do { \
		omap_writew(DSPMMU_CNTL_MMU_EN | DSPMMU_CNTL_RESET_SW, \
			    DSPMMU_CNTL); \
	} while(0)
#define dsp_mmu_disable() \
	do { omap_writew(0, DSPMMU_CNTL); } while(0)
#define dsp_mmu_flush() \
	do { \
		omap_writew(DSPMMU_FLUSH_ENTRY_FLUSH_ENTRY, \
			    DSPMMU_FLUSH_ENTRY); \
	} while(0)
#define __dsp_mmu_gflush() \
	do { omap_writew(DSPMMU_GFLUSH_GFLUSH, DSPMMU_GFLUSH); } while(0)
#define __dsp_mmu_itack() \
	do { omap_writew(DSPMMU_IT_ACK_IT_ACK, DSPMMU_IT_ACK); } while(0)

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

enum exmap_type {
	EXMAP_TYPE_MEM,
	EXMAP_TYPE_FB
};

struct exmap_tbl {
	unsigned int valid:1;
	unsigned int cntnu:1;	/* grouping */
	int usecount;		/* reference count by mmap */
	enum exmap_type type;
	void *buf;		/* virtual address of the buffer,
				 * i.e. 0xc0000000 - */
	void *vadr;		/* DSP shadow space,
				 * i.e. 0xe0000000 - 0xe0ffffff */
	unsigned int order;
};
#define DSPMMU_TLB_LINES	32
static struct exmap_tbl exmap_tbl[DSPMMU_TLB_LINES];
static DECLARE_RWSEM(exmap_sem);

static int dsp_exunmap(unsigned long dspadr);

static void *dspvect_page;
static unsigned long dsp_fault_adr;
static struct mem_sync_struct mem_sync;

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

void dsp_mem_sync_inc(void)
{
	/*
	 * FIXME: dsp_mem_enable()!!!
	 */
	if (mem_sync.DARAM)
		mem_sync.DARAM->ad_arm++;
	if (mem_sync.SARAM)
		mem_sync.SARAM->ad_arm++;
	if (mem_sync.SDRAM)
		mem_sync.SDRAM->ad_arm++;
}

/*
 * dsp_mem_sync_config() is called from mbx1 workqueue
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

/*
 * kmem_reserve(), kmem_release():
 * reserve or release kernel memory for exmap().
 *
 * exmap() might request consecutive 1MB or 64kB,
 * but it will be difficult after memory pages are fragmented.
 * So, user can reserve such memory blocks in the early phase
 * through kmem_reserve().
 */
struct kmem_pool {
	struct semaphore sem;
	unsigned long buf[16];
	int count;
};

#define KMEM_POOL_INIT(name) \
{ \
	.sem = __SEMAPHORE_INIT((name).sem, 1), \
}
#define DECLARE_KMEM_POOL(name) \
	struct kmem_pool name = KMEM_POOL_INIT(name)

DECLARE_KMEM_POOL(kmem_pool_1M);
DECLARE_KMEM_POOL(kmem_pool_64K);

static void dsp_kmem_release(void)
{
	int i;

	down(&kmem_pool_1M.sem);
	for (i = 0; i < kmem_pool_1M.count; i++) {
		if (kmem_pool_1M.buf[i])
			free_pages(kmem_pool_1M.buf[i], ORDER_1MB);
	}
	kmem_pool_1M.count = 0;
	up(&kmem_pool_1M.sem);

	down(&kmem_pool_64K.sem);
	for (i = 0; i < kmem_pool_64K.count; i++) {
		if (kmem_pool_64K.buf[i])
			free_pages(kmem_pool_64K.buf[i], ORDER_64KB);
	}
	kmem_pool_64K.count = 0;
	up(&kmem_pool_1M.sem);
}

static int dsp_kmem_reserve(unsigned long size)
{
	unsigned long buf;
	unsigned int order;
	unsigned long unit;
	unsigned long _size;
	struct kmem_pool *pool;
	int i;

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

	for (_size = size; _size; _size -= unit) {
		if (_size >= SZ_1MB) {
			unit = SZ_1MB;
			order = ORDER_1MB;
			pool = &kmem_pool_1M;
		} else {
			unit = SZ_64KB;
			order = ORDER_64KB;
			pool = &kmem_pool_64K;
		}

		buf = __get_dma_pages(GFP_KERNEL, order);
		if (!buf)
			return size - _size;
		down(&pool->sem);
		for (i = 0; i < 16; i++) {
			if (!pool->buf[i]) {
				pool->buf[i] = buf;
				pool->count++;
				buf = 0;
				break;
			}
		}
		up(&pool->sem);

		if (buf) {	/* pool is full */
			free_pages(buf, order);
			return size - _size;
		}
	}

	return size;
}

static unsigned long dsp_mem_get_dma_pages(unsigned int order)
{
	struct kmem_pool *pool;
	unsigned long buf = 0;
	int i;

	switch (order) {
		case ORDER_1MB:
			pool = &kmem_pool_1M;
			break;
		case ORDER_64KB:
			pool = &kmem_pool_64K;
			break;
		default:
			pool = NULL;
	}

	if (pool) {
		down(&pool->sem);
		for (i = 0; i < pool->count; i++) {
			if (pool->buf[i]) {
				buf = pool->buf[i];
				pool->buf[i] = 0;
				break;
			}
		}
		up(&pool->sem);
		if (buf)
			return buf;
	}

	/* other size or not found in pool */
	return __get_dma_pages(GFP_KERNEL, order);
}

static void dsp_mem_free_pages(unsigned long buf, unsigned int order)
{
	struct kmem_pool *pool;
	struct page *page, *ps, *pe;
	int i;

	ps = virt_to_page(buf);
	pe = virt_to_page(buf + (1 << (PAGE_SHIFT + order)));
	for (page = ps; page < pe; page++) {
		ClearPageReserved(page);
	}

	/*
	 * return buffer to kmem_pool or paging system
	 */
	switch (order) {
		case ORDER_1MB:
			pool = &kmem_pool_1M;
			break;
		case ORDER_64KB:
			pool = &kmem_pool_64K;
			break;
		default:
			pool = NULL;
	}

	if (pool) {
		down(&pool->sem);
		for (i = 0; i < pool->count; i++) {
			if (!pool->buf[i]) {
				pool->buf[i] = buf;
				buf = 0;
			}
		}
		up(&pool->sem);
	}

	/* other size or pool is filled */
	if (buf)
		free_pages(buf, order);
}

/*
 * ARM MMU operations
 */
static int exmap_set_armmmu(unsigned long virt, unsigned long phys,
			    unsigned long size)
{
	long off;
	unsigned long sz_left;
	pmd_t *pmdp;
	pte_t *ptep;
	int prot_pmd, prot_pte;

	printk(KERN_DEBUG
	       "omapdsp: mapping in ARM MMU, v=0x%08lx, p=0x%08lx, sz=0x%lx\n",
	       virt, phys, size);

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
		set_pte(ptep, __pte((virt + off) | prot_pte));
	}
	if (sz_left)
		BUG();

	return 0;
}

static void exmap_clear_armmmu(unsigned long virt, unsigned long size)
{
	unsigned long sz_left;
	pmd_t *pmdp;
	pte_t *ptep;

	printk(KERN_DEBUG
	       "omapdsp: unmapping in ARM MMU, v=0x%08lx, sz=0x%lx\n",
	       virt, size);

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

static int exmap_valid(void *vadr, size_t len)
{
	/* exmap_sem should be held before calling this function */
	int i;

start:
	for (i = 0; i < DSPMMU_TLB_LINES; i++) {
		void *mapadr;
		unsigned long mapsize;
		struct exmap_tbl *ent = &exmap_tbl[i];

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
	for (i = 0; i < DSPMMU_TLB_LINES; i++) {
		void *mapadr;
		unsigned long mapsize;
		struct exmap_tbl *ent = &exmap_tbl[i];

		if (!ent->valid)
			continue;
		mapadr = (void *)ent->vadr;
		mapsize = 1 << (ent->order + PAGE_SHIFT);
		if ((vadr + len > mapadr) && (vadr < mapadr + mapsize)) {
			ent->usecount++;
		}
	}
	up_write(&exmap_sem);
}

void exmap_unuse(void *vadr, size_t len)
{
	int i;

	down_write(&exmap_sem);
	for (i = 0; i < DSPMMU_TLB_LINES; i++) {
		void *mapadr;
		unsigned long mapsize;
		struct exmap_tbl *ent = &exmap_tbl[i];

		if (!ent->valid)
			continue;
		mapadr = (void *)ent->vadr;
		mapsize = 1 << (ent->order + PAGE_SHIFT);
		if ((vadr + len > mapadr) && (vadr < mapadr + mapsize)) {
			ent->usecount--;
		}
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
	for (i = 0; i < DSPMMU_TLB_LINES; i++) {
		void *mapadr;
		unsigned long mapsize;
		struct exmap_tbl *ent = &exmap_tbl[i];

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
static __inline__ unsigned short get_cam_l_va_mask(unsigned short slst)
{
	switch (slst) {
	case DSPMMU_CAM_L_SLST_1MB:
		return DSPMMU_CAM_L_VA_TAG_L1_MASK |
		       DSPMMU_CAM_L_VA_TAG_L2_MASK_1MB;
	case DSPMMU_CAM_L_SLST_64KB:
		return DSPMMU_CAM_L_VA_TAG_L1_MASK |
		       DSPMMU_CAM_L_VA_TAG_L2_MASK_64KB;
	case DSPMMU_CAM_L_SLST_4KB:
		return DSPMMU_CAM_L_VA_TAG_L1_MASK |
		       DSPMMU_CAM_L_VA_TAG_L2_MASK_4KB;
	case DSPMMU_CAM_L_SLST_1KB:
		return DSPMMU_CAM_L_VA_TAG_L1_MASK |
		       DSPMMU_CAM_L_VA_TAG_L2_MASK_1KB;
	}
	return 0;
}

static __inline__ void get_tlb_lock(int *base, int *victim)
{
	unsigned short lock = omap_readw(DSPMMU_LOCK);
	if (base != NULL)
		*base = (lock & DSPMMU_LOCK_BASE_MASK)
			>> DSPMMU_LOCK_BASE_SHIFT;
	if (victim != NULL)
		*victim = (lock & DSPMMU_LOCK_VICTIM_MASK)
			  >> DSPMMU_LOCK_VICTIM_SHIFT;
}

static __inline__ void set_tlb_lock(int base, int victim)
{
	omap_writew((base   << DSPMMU_LOCK_BASE_SHIFT) |
		    (victim << DSPMMU_LOCK_VICTIM_SHIFT), DSPMMU_LOCK);
}

static __inline__ void __read_tlb(unsigned short lbase, unsigned short victim,
				  unsigned short *cam_h, unsigned short *cam_l,
				  unsigned short *ram_h, unsigned short *ram_l)
{
	/* set victim */
	set_tlb_lock(lbase, victim);

	/* read a TLB entry */
	omap_writew(DSPMMU_LD_TLB_RD, DSPMMU_LD_TLB);

	if (cam_h != NULL)
		*cam_h = omap_readw(DSPMMU_READ_CAM_H);
	if (cam_l != NULL)
		*cam_l = omap_readw(DSPMMU_READ_CAM_L);
	if (ram_h != NULL)
		*ram_h = omap_readw(DSPMMU_READ_RAM_H);
	if (ram_l != NULL)
		*ram_l = omap_readw(DSPMMU_READ_RAM_L);
}

static __inline__ void __load_tlb(unsigned short cam_h, unsigned short cam_l,
				  unsigned short ram_h, unsigned short ram_l)
{
	omap_writew(cam_h, DSPMMU_CAM_H);
	omap_writew(cam_l, DSPMMU_CAM_L);
	omap_writew(ram_h, DSPMMU_RAM_H);
	omap_writew(ram_l, DSPMMU_RAM_L);

	/* flush the entry */
	dsp_mmu_flush();

	/* load a TLB entry */
	omap_writew(DSPMMU_LD_TLB_LD, DSPMMU_LD_TLB);
}

static int dsp_mmu_load_tlb(unsigned long vadr, unsigned long padr,
			    unsigned short slst, unsigned short prsvd,
			    unsigned short ap)
{
	int lbase, victim;
	unsigned short cam_l_va_mask;

	clk_enable(dsp_ck_handle);

	get_tlb_lock(&lbase, NULL);
	for (victim = 0; victim < lbase; victim++) {
		unsigned short cam_l;

		/* read a TLB entry */
		__read_tlb(lbase, victim, NULL, &cam_l, NULL, NULL);
		if (!(cam_l & DSPMMU_CAM_L_V))
			goto found_victim;
	}
	set_tlb_lock(lbase, victim);

found_victim:
	/* The last (31st) entry cannot be locked? */
	if (victim == 31) {
		printk(KERN_ERR "omapdsp: TLB is full.\n");
		return -EBUSY;
	}

	cam_l_va_mask = get_cam_l_va_mask(slst);
	if (vadr &
	    ~(DSPMMU_CAM_H_VA_TAG_H_MASK << 22 |
	      (unsigned long)cam_l_va_mask << 6)) {
		printk(KERN_ERR
		       "omapdsp: mapping vadr (0x%06lx) is not "
		       "aligned boundary\n", vadr);
		return -EINVAL;
	}

	__load_tlb(vadr >> 22, (vadr >> 6 & cam_l_va_mask) | prsvd | slst,
		   padr >> 16, (padr & DSPMMU_RAM_L_RAM_LSB_MASK) | ap);

	/* update lock base */
	if (victim == lbase)
		lbase++;
	set_tlb_lock(lbase, lbase);

	clk_disable(dsp_ck_handle);
	return 0;
}

static int dsp_mmu_clear_tlb(unsigned long vadr)
{
	int lbase;
	int i;
	int max_valid = 0;

	clk_enable(dsp_ck_handle);

	get_tlb_lock(&lbase, NULL);
	for (i = 0; i < lbase; i++) {
		unsigned short cam_h, cam_l;
		unsigned short cam_l_va_mask, cam_vld, slst;
		unsigned long cam_va;

		/* read a TLB entry */
		__read_tlb(lbase, i, &cam_h, &cam_l, NULL, NULL);

		cam_vld = cam_l & DSPMMU_CAM_L_V;
		if (!cam_vld)
			continue;

		slst = cam_l & DSPMMU_CAM_L_SLST_MASK;
		cam_l_va_mask = get_cam_l_va_mask(slst);
		cam_va = (unsigned long)(cam_h & DSPMMU_CAM_H_VA_TAG_H_MASK) << 22 |
			 (unsigned long)(cam_l & cam_l_va_mask) << 6;

		if (cam_va == vadr)
			/* flush the entry */
			dsp_mmu_flush();
		else
			max_valid = i;
	}

	/* set new lock base */
	set_tlb_lock(max_valid+1, max_valid+1);

	clk_disable(dsp_ck_handle);
	return 0;
}

static void dsp_mmu_gflush(void)
{
	clk_enable(dsp_ck_handle);

	__dsp_mmu_gflush();
	set_tlb_lock(1, 1);

	clk_disable(dsp_ck_handle);
}

/*
 * dsp_exmap()
 *
 * OMAP_DSP_MEM_IOCTL_EXMAP ioctl calls this function with padr=0.
 * In this case, the buffer for DSP is allocated in this routine,
 * then it is mapped.
 * On the other hand, for example - frame buffer sharing, calls
 * this function with padr set. It means some known address space
 * pointed with padr is going to be shared with DSP.
 */
static int dsp_exmap(unsigned long dspadr, unsigned long padr,
		     unsigned long size, enum exmap_type type)
{
	unsigned short slst;
	void *buf;
	unsigned int order = 0;
	unsigned long unit;
	unsigned int cntnu = 0;
	unsigned long _dspadr = dspadr;
	unsigned long _padr = padr;
	void *_vadr = dspbyte_to_virt(dspadr);
	unsigned long _size = size;
	struct exmap_tbl *exmap_ent;
	int status;
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
		       "omapdsp: DSP address(0x%lx) is not aligned.\n", dspadr);
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
	for (i = 0; i < DSPMMU_TLB_LINES; i++) {
		unsigned long mapsize;
		struct exmap_tbl *tmp_ent = &exmap_tbl[i];

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
	for (i = 0; i < DSPMMU_TLB_LINES; i++) {
		if (!exmap_tbl[i].valid)
			goto found_free;
	}
	printk(KERN_ERR "omapdsp: DSP TLB is full.\n");
	status = -EBUSY;
	goto fail;

found_free:
	exmap_ent = &exmap_tbl[i];

	if ((_size >= SZ_1MB) &&
	    (is_aligned(_padr, SZ_1MB) || (padr == 0)) &&
	    is_aligned(_dspadr, SZ_1MB)) {
		unit = SZ_1MB;
		slst = DSPMMU_CAM_L_SLST_1MB;
		order = ORDER_1MB;
	} else if ((_size >= SZ_64KB) &&
		   (is_aligned(_padr, SZ_64KB) || (padr == 0)) &&
		   is_aligned(_dspadr, SZ_64KB)) {
		unit = SZ_64KB;
		slst = DSPMMU_CAM_L_SLST_64KB;
		order = ORDER_64KB;
	} else /* if (_size >= SZ_4KB) */ {
		unit = SZ_4KB;
		slst = DSPMMU_CAM_L_SLST_4KB;
		order = ORDER_4KB;
	}
#if 0	/* 1KB is not enabled */
	else if (_size >= SZ_1KB) {
		unit = SZ_1KB;
		slst = DSPMMU_CAM_L_SLST_1KB;
		order = XXX;
	}
#endif

	/* buffer allocation */
	if (type == EXMAP_TYPE_MEM) {
		struct page *page, *ps, *pe;

		buf = (void *)dsp_mem_get_dma_pages(order);
		if (buf == NULL) {
			status = -ENOMEM;
			goto fail;
		}
		/* mark the pages as reserved; this is needed for mmap */
		ps = virt_to_page(buf);
		pe = virt_to_page(buf + unit);
		for (page = ps; page < pe; page++) {
			SetPageReserved(page);
		}
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
	status = dsp_mmu_load_tlb(_dspadr, _padr, slst, 0, DSPMMU_RAM_L_AP_FA);
	if (status < 0) {
		exmap_clear_armmmu((unsigned long)_vadr, unit);
		goto fail;
	}

	exmap_ent->buf      = buf;
	exmap_ent->vadr     = _vadr;
	exmap_ent->order    = order;
	exmap_ent->valid    = 1;
	exmap_ent->cntnu    = cntnu;
	exmap_ent->type     = type;
	exmap_ent->usecount = 0;

	if ((_size -= unit) == 0) {	/* normal completion */
		up_write(&exmap_sem);
		return size;
	}

	_dspadr += unit;
	_vadr   += unit;
	_padr = padr ? _padr + unit : 0;
	cntnu = 1;
	goto start;

fail:
	up_write(&exmap_sem);
	if (buf)
		dsp_mem_free_pages((unsigned long)buf, order);
	dsp_exunmap(dspadr);
	return status;
}

static unsigned long unmap_free_arm(struct exmap_tbl *ent)
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

	return size;
}

static int dsp_exunmap(unsigned long dspadr)
{
	void *vadr;
	unsigned long size;
	int total = 0;
	struct exmap_tbl *ent;
	int idx;

	vadr = dspbyte_to_virt(dspadr);
	down_write(&exmap_sem);
	for (idx = 0; idx < DSPMMU_TLB_LINES; idx++) {
		ent = &exmap_tbl[idx];
		if (!ent->valid)
			continue;
		if (ent->vadr == vadr)
			goto found_map;
	}
	up_write(&exmap_sem);
	printk(KERN_WARNING
	       "omapdsp: address %06lx not found in exmap_tbl.\n", dspadr);
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

	/* check if next mapping is in same group */
	if (++idx == DSPMMU_TLB_LINES)
		goto up_out;	/* normal completion */
	ent = &exmap_tbl[idx];
	if (!ent->valid || !ent->cntnu)
		goto up_out;	/* normal completion */

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
	struct exmap_tbl *ent;
	int i;

	down_write(&exmap_sem);

	/* clearing DSP TLB entry */
	dsp_mmu_gflush();

	/* exmap_tbl[0] should be preserved */
	for (i = 1; i < DSPMMU_TLB_LINES; i++) {
		ent = &exmap_tbl[i];
		if (ent->valid) {
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

static int dsp_fbexport(unsigned long *dspadr)
{
	unsigned long dspadr_actual;
	unsigned long padr_sys, padr, fbsz_sys, fbsz;
	int cnt;

	printk(KERN_DEBUG "omapdsp: frame buffer export\n");

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
		       "omapdsp: actual dspadr for FBEXPORT = %08lx\n",
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

	/* increase the DMA priority */
	set_emiff_dma_prio(15);

	return cnt;
}

#else /* CONFIG_OMAP_DSP_FBEXPORT */

static int dsp_fbexport(unsigned long *dspadr)
{
	printk(KERN_ERR "omapdsp: FBEXPORT function is not enabled.\n");
	return -EINVAL;
}

#endif /* CONFIG_OMAP_DSP_FBEXPORT */

static int dsp_mmu_itack(void)
{
	unsigned long dspadr;

	printk(KERN_INFO "omapdsp: sending DSP MMU interrupt ack.\n");
	if (!dsp_err_mmu_isset()) {
		printk(KERN_ERR "omapdsp: DSP MMU error has not been set.\n");
		return -EINVAL;
	}
	dspadr = dsp_fault_adr & ~(SZ_4K-1);
	dsp_exmap(dspadr, 0, SZ_4K, EXMAP_TYPE_MEM);	/* FIXME: reserve TLB entry for this */
	printk(KERN_INFO "omapdsp: falling into recovery runlevel...\n");
	dsp_runlevel(OMAP_DSP_MBCMD_RUNLEVEL_RECOVERY);
	__dsp_mmu_itack();
	udelay(100);
	dsp_exunmap(dspadr);
	dsp_err_mmu_clear();
	return 0;
}

static void dsp_mmu_init(void)
{
	unsigned long phys;
	void *virt;

	clk_enable(dsp_ck_handle);
	down_write(&exmap_sem);

	dsp_mmu_disable();	/* clear all */
	udelay(100);
	dsp_mmu_enable();

	/* mapping for ARM MMU */
	phys = __pa(dspvect_page);
	virt = dspbyte_to_virt(DSP_INIT_PAGE);	/* 0xe0fff000 */
	exmap_set_armmmu((unsigned long)virt, phys, PAGE_SIZE);
	exmap_tbl[0].buf      = dspvect_page;
	exmap_tbl[0].vadr     = virt;
	exmap_tbl[0].usecount = 0;
	exmap_tbl[0].order    = 0;
	exmap_tbl[0].valid    = 1;
	exmap_tbl[0].cntnu    = 0;

	/* DSP TLB initialization */
	set_tlb_lock(0, 0);
	/* preserved, full access */
	dsp_mmu_load_tlb(DSP_INIT_PAGE, phys, DSPMMU_CAM_L_SLST_4KB,
			 DSPMMU_CAM_L_P, DSPMMU_RAM_L_AP_FA);
	up_write(&exmap_sem);
	clk_disable(dsp_ck_handle);
}

static void dsp_mmu_shutdown(void)
{
	exmap_flush();
	dsp_mmu_disable();	/* clear all */
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

	if (dsp_is_ready())
		ret = dsp_mbsend(MBCMD(PM), OMAP_DSP_MBCMD_PM_ENABLE,
				 DSPREG_ICR_DMA_IDLE_DOMAIN);

	return ret;
}

static void intmem_disable(void) {
	if (dsp_is_ready())
		dsp_mbsend(MBCMD(PM), OMAP_DSP_MBCMD_PM_DISABLE,
			   DSPREG_ICR_DMA_IDLE_DOMAIN);
}

/*
 * dsp_mem_enable() / disable()
 */
int intmem_usecount;

int dsp_mem_enable(void *adr)
{
	int ret = 0;

	if (is_dsp_internal_mem(adr)) {
		if (intmem_usecount++ == 0)
			ret = omap_dsp_request_mem();
	} else
		down_read(&exmap_sem);

	return ret;
}

void dsp_mem_disable(void *adr)
{
	if (is_dsp_internal_mem(adr)) {
		if (--intmem_usecount == 0)
			omap_dsp_release_mem();
	} else
		up_read(&exmap_sem);
}

/* for safety */
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

/*
 * dsp_mem file operations
 */
static loff_t dsp_mem_lseek(struct file *file, loff_t offset, int orig)
{
	loff_t ret;

	down(&file->f_dentry->d_inode->i_sem);
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
	up(&file->f_dentry->d_inode->i_sem);
	return ret;
}

static ssize_t intmem_read(struct file *file, char *buf, size_t count,
			   loff_t *ppos)
{
	unsigned long p = *ppos;
	void *vadr = dspbyte_to_virt(p);
	ssize_t size = dspmem_size;
	ssize_t read;

	if (p >= size)
		return 0;
	clk_enable(api_ck_handle);
	read = count;
	if (count > size - p)
		read = size - p;
	if (copy_to_user(buf, vadr, read)) {
		read = -EFAULT;
		goto out;
	}
	*ppos += read;
out:
	clk_disable(api_ck_handle);
	return read;
}

static ssize_t exmem_read(struct file *file, char *buf, size_t count,
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

static ssize_t dsp_mem_read(struct file *file, char *buf, size_t count,
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

static ssize_t intmem_write(struct file *file, const char *buf, size_t count,
			    loff_t *ppos)
{
	unsigned long p = *ppos;
	void *vadr = dspbyte_to_virt(p);
	ssize_t size = dspmem_size;
	ssize_t written;

	if (p >= size)
		return 0;
	clk_enable(api_ck_handle);
	written = count;
	if (count > size - p)
		written = size - p;
	if (copy_from_user(vadr, buf, written)) {
		written = -EFAULT;
		goto out;
	}
	*ppos += written;
out:
	clk_disable(api_ck_handle);
	return written;
}

static ssize_t exmem_write(struct file *file, const char *buf, size_t count,
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
	if (copy_from_user(vadr, buf, count))
		return -EFAULT;
	*ppos += count;

	return count;
}

static ssize_t dsp_mem_write(struct file *file, const char *buf, size_t count,
			     loff_t *ppos)
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
	case OMAP_DSP_MEM_IOCTL_MMUINIT:
		dsp_mmu_init();
		return 0;

	case OMAP_DSP_MEM_IOCTL_EXMAP:
		{
			struct omap_dsp_mapinfo mapinfo;
			if (copy_from_user(&mapinfo, (void *)arg,
					   sizeof(mapinfo)))
				return -EFAULT;
			return dsp_exmap(mapinfo.dspadr, 0, mapinfo.size,
					 EXMAP_TYPE_MEM);
		}

	case OMAP_DSP_MEM_IOCTL_EXUNMAP:
		return dsp_exunmap((unsigned long)arg);

	case OMAP_DSP_MEM_IOCTL_EXMAP_FLUSH:
		exmap_flush();
		return 0;

	case OMAP_DSP_MEM_IOCTL_FBEXPORT:
		{
			unsigned long dspadr;
			int ret;
			if (copy_from_user(&dspadr, (void *)arg, sizeof(long)))
				return -EFAULT;
			ret = dsp_fbexport(&dspadr);
			if (copy_to_user((void *)arg, &dspadr, sizeof(long)))
				return -EFAULT;
			return ret;
		}

	case OMAP_DSP_MEM_IOCTL_MMUITACK:
		return dsp_mmu_itack();

	case OMAP_DSP_MEM_IOCTL_KMEM_RESERVE:
		{
			unsigned long size;
			if (copy_from_user(&size, (void *)arg, sizeof(long)))
				return -EFAULT;
			return dsp_kmem_reserve(size);
		}

	case OMAP_DSP_MEM_IOCTL_KMEM_RELEASE:
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

static int dsp_mem_release(struct inode *inode, struct file *file)
{
	return 0;
}

/*
 * sysfs files
 */
static ssize_t mmu_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	int len;
	int lbase, victim;
	int i;

	clk_enable(dsp_ck_handle);
	down_read(&exmap_sem);

	get_tlb_lock(&lbase, &victim);

	len = sprintf(buf, "p: preserved,  v: valid\n"
			   "ety       cam_va     ram_pa   sz ap\n");
			/* 00: p v 0x300000 0x10171800 64KB FA */
	for (i = 0; i < 32; i++) {
		unsigned short cam_h, cam_l, ram_h, ram_l;
		unsigned short cam_l_va_mask, prsvd, cam_vld, slst;
		unsigned long cam_va;
		unsigned short ram_l_ap;
		unsigned long ram_pa;
		char *pgsz_str, *ap_str;

		/* read a TLB entry */
		__read_tlb(lbase, i, &cam_h, &cam_l, &ram_h, &ram_l);

		slst = cam_l & DSPMMU_CAM_L_SLST_MASK;
		cam_l_va_mask = get_cam_l_va_mask(slst);
		pgsz_str = (slst == DSPMMU_CAM_L_SLST_1MB) ? " 1MB":
			   (slst == DSPMMU_CAM_L_SLST_64KB)? "64KB":
			   (slst == DSPMMU_CAM_L_SLST_4KB) ? " 4KB":
			                                     " 1KB";
		prsvd    = cam_l & DSPMMU_CAM_L_P;
		cam_vld  = cam_l & DSPMMU_CAM_L_V;
		ram_l_ap = ram_l & DSPMMU_RAM_L_AP_MASK;
		ap_str = (ram_l_ap == DSPMMU_RAM_L_AP_RO) ? "RO":
			 (ram_l_ap == DSPMMU_RAM_L_AP_FA) ? "FA":
			                                    "NA";
		cam_va = (unsigned long)(cam_h & DSPMMU_CAM_H_VA_TAG_H_MASK) << 22 |
			 (unsigned long)(cam_l & cam_l_va_mask) << 6;
		ram_pa = (unsigned long)ram_h << 16 |
			 (ram_l & DSPMMU_RAM_L_RAM_LSB_MASK);

		if (i == lbase)
			len += sprintf(buf + len, "lock base = %d\n", lbase);
		if (i == victim)
			len += sprintf(buf + len, "victim    = %d\n", victim);
		/* 00: p v 0x300000 0x10171800 64KB FA */
		len += sprintf(buf + len,
			       "%02d: %c %c 0x%06lx 0x%08lx %s %s\n",
			       i,
			       prsvd   ? 'p' : ' ',
			       cam_vld ? 'v' : ' ',
			       cam_va, ram_pa, pgsz_str, ap_str);
	}

	/* restore victim entry */
	set_tlb_lock(lbase, victim);

	up_read(&exmap_sem);
	clk_disable(dsp_ck_handle);
	return len;
}

static struct device_attribute dev_attr_mmu = __ATTR_RO(mmu);

static ssize_t exmap_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	int len;
	int i;

	down_read(&exmap_sem);
	len = sprintf(buf, "v: valid,  c: cntnu\n"
			   "ety           vadr        buf od uc\n");
			 /* 00: v c 0xe0300000 0xc0171800  0 */
	for (i = 0; i < DSPMMU_TLB_LINES; i++) {
		struct exmap_tbl *ent = &exmap_tbl[i];
		/* 00: v c 0xe0300000 0xc0171800  0 */
		len += sprintf(buf + len, "%02d: %c %c 0x%8p 0x%8p %2d %2d\n",
			       i,
			       ent->valid ? 'v' : ' ',
			       ent->cntnu ? 'c' : ' ',
			       ent->vadr, ent->buf, ent->order, ent->usecount);
	}

	up_read(&exmap_sem);
	return len;
}

static struct device_attribute dev_attr_exmap = __ATTR_RO(exmap);

static ssize_t kmem_pool_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	int count_1M, count_64K, total;

	count_1M = kmem_pool_1M.count;
	count_64K = kmem_pool_64K.count;
	total = count_1M * SZ_1MB + count_64K * SZ_64KB;

	return sprintf(buf, "0x%x %d %d\n", total, count_1M, count_64K);
}

static struct device_attribute dev_attr_kmem_pool = __ATTR_RO(kmem_pool);

/*
 * DSP MMU interrupt handler
 */

/*
 * MMU fault mask:
 * We ignore prefetch err.
 */
#define MMUFAULT_MASK \
	(DSPMMU_FAULT_ST_PERM |\
	 DSPMMU_FAULT_ST_TLB_MISS |\
	 DSPMMU_FAULT_ST_TRANS)
irqreturn_t dsp_mmu_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	unsigned short status;
	unsigned short adh, adl;
	unsigned short dp;

	status = omap_readw(DSPMMU_FAULT_ST);
	adh = omap_readw(DSPMMU_FAULT_AD_H);
	adl = omap_readw(DSPMMU_FAULT_AD_L);
	dp = adh & DSPMMU_FAULT_AD_H_DP;
	dsp_fault_adr = MKLONG(adh & DSPMMU_FAULT_AD_H_ADR_MASK, adl);
	/* if the fault is masked, nothing to do */
	if ((status & MMUFAULT_MASK) == 0) {
		printk(KERN_DEBUG "DSP MMU interrupt, but ignoring.\n");
		/*
		 * note: in OMAP1710,
		 * when CACHE + DMA domain gets out of idle in DSP,
		 * MMU interrupt occurs but DSPMMU_FAULT_ST is not set.
		 * in this case, we just ignore the interrupt.
		 */
		if (status) {
			printk(KERN_DEBUG "%s%s%s%s\n",
			       (status & DSPMMU_FAULT_ST_PREF)?
					"  (prefetch err)" : "",
			       (status & DSPMMU_FAULT_ST_PERM)?
					"  (permission fault)" : "",
			       (status & DSPMMU_FAULT_ST_TLB_MISS)?
					"  (TLB miss)" : "",
			       (status & DSPMMU_FAULT_ST_TRANS) ?
					"  (translation fault)": "");
			printk(KERN_DEBUG
			       "fault address = %s: 0x%06lx\n",
			       dp ? "DATA" : "PROGRAM",
			       dsp_fault_adr);
		}
		return IRQ_HANDLED;
	}

	printk(KERN_INFO "DSP MMU interrupt!\n");
	printk(KERN_INFO "%s%s%s%s\n",
	       (status & DSPMMU_FAULT_ST_PREF)?
			(MMUFAULT_MASK & DSPMMU_FAULT_ST_PREF)?
				"  prefetch err":
				"  (prefetch err)":
				"",
	       (status & DSPMMU_FAULT_ST_PERM)?
			(MMUFAULT_MASK & DSPMMU_FAULT_ST_PERM)?
				"  permission fault":
				"  (permission fault)":
				"",
	       (status & DSPMMU_FAULT_ST_TLB_MISS)?
			(MMUFAULT_MASK & DSPMMU_FAULT_ST_TLB_MISS)?
				"  TLB miss":
				"  (TLB miss)":
				"",
	       (status & DSPMMU_FAULT_ST_TRANS)?
			(MMUFAULT_MASK & DSPMMU_FAULT_ST_TRANS)?
				"  translation fault":
				"  (translation fault)":
				"");
	printk(KERN_INFO "fault address = %s: 0x%06lx\n",
	       dp ? "DATA" : "PROGRAM",
	       dsp_fault_adr);

	if (dsp_is_ready()) {
		/*
		 * If we call dsp_exmap() here,
		 * "kernel BUG at slab.c" occurs.
		 */
		/* FIXME */
		dsp_err_mmu_set(dsp_fault_adr);
	} else {
		disable_irq(INT_DSP_MMU);
		__dsp_mmu_itack();
		printk(KERN_INFO "Resetting DSP...\n");
		dsp_cpustat_request(CPUSTAT_RESET);
		enable_irq(INT_DSP_MMU);
		/*
		 * if we enable followings, semaphore lock should be avoided.
		 *
		printk(KERN_INFO "Flushing DSP MMU...\n");
		exmap_flush();
		dsp_mmu_init();
		 */
	}

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
	.release = dsp_mem_release,
};

void dsp_mem_start(void)
{
	dsp_register_mem_cb(intmem_enable, intmem_disable);
}

void dsp_mem_stop(void)
{
	memset(&mem_sync, 0, sizeof(struct mem_sync_struct));
	dsp_unregister_mem_cb();
}

int __init dsp_mem_init(void)
{
	int i;

	for (i = 0; i < DSPMMU_TLB_LINES; i++) {
		exmap_tbl[i].valid = 0;
	}

	dspvect_page = (void *)__get_dma_pages(GFP_KERNEL, 0);
	if (dspvect_page == NULL) {
		printk(KERN_ERR
		       "omapdsp: failed to allocate memory "
		       "for dsp vector table\n");
		return -ENOMEM;
	}
	dsp_mmu_init();
	dsp_set_idle_boot_base(IDLEPG_BASE, IDLEPG_SIZE);

	device_create_file(&dsp_device.dev, &dev_attr_mmu);
	device_create_file(&dsp_device.dev, &dev_attr_exmap);
	device_create_file(&dsp_device.dev, &dev_attr_kmem_pool);

	return 0;
}

void dsp_mem_exit(void)
{
	dsp_mmu_shutdown();
	dsp_kmem_release();

	if (dspvect_page != NULL) {
		unsigned long virt;

		down_read(&exmap_sem);

		virt = (unsigned long)dspbyte_to_virt(DSP_INIT_PAGE);
		flush_tlb_kernel_range(virt, virt + PAGE_SIZE);
		free_page((unsigned long)dspvect_page);
		dspvect_page = NULL;

		up_read(&exmap_sem);
	}

	device_remove_file(&dsp_device.dev, &dev_attr_mmu);
	device_remove_file(&dsp_device.dev, &dev_attr_exmap);
	device_remove_file(&dsp_device.dev, &dev_attr_kmem_pool);
}
