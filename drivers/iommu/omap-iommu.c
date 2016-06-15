/*
 * omap iommu: tlb and pagetable primitives
 *
 * Copyright (C) 2008-2010 Nokia Corporation
 * Copyright (C) 2013-2014 Texas Instruments Inc
 *
 * Written by Hiroshi DOYU <Hiroshi.DOYU@nokia.com>,
 *		Paul Mundt and Toshihiro Kobayashi
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/iommu.h>
#include <linux/omap-iommu.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/of_iommu.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>

#include <asm/cacheflush.h>

#include <linux/platform_data/iommu-omap.h>

#include "omap-iopgtable.h"
#include "omap-iommu.h"

#define to_iommu(dev)							\
	((struct omap_iommu *)platform_get_drvdata(to_platform_device(dev)))

#define for_each_iotlb_cr(obj, n, __i, cr)				\
	for (__i = 0;							\
	     (__i < (n)) && (cr = __iotlb_read_cr((obj), __i), true);	\
	     __i++)

/* bitmap of the page sizes currently supported */
#define OMAP_IOMMU_PGSIZES	(SZ_4K | SZ_64K | SZ_1M | SZ_16M)

/**
 * struct omap_iommu_device - omap iommu device data
 * @pgtable:	the page table
 * @iommu_dev:	an omap iommu instance attached to this domain.
 */
struct omap_iommu_device {
	u32 *pgtable;
	struct omap_iommu *iommu_dev;
};

/**
 * struct omap_iommu_domain - omap iommu domain
 * @attached:	flag to indicate if domain is already attached to
 * @num_iommus: number of iommus in this domain
 * @iommus:	omap iommu device data for all iommus in this domain
 * @dev:	Device using this domain.
 * @lock:	domain lock, should be taken when attaching/detaching
 */
struct omap_iommu_domain {
	u32 attached;
	u32 num_iommus;
	struct omap_iommu_device *iommus;
	struct device *dev;
	spinlock_t lock;
};

#define MMU_LOCK_BASE_SHIFT	10
#define MMU_LOCK_BASE_MASK	(0x1f << MMU_LOCK_BASE_SHIFT)
#define MMU_LOCK_BASE(x)	\
	((x & MMU_LOCK_BASE_MASK) >> MMU_LOCK_BASE_SHIFT)

#define MMU_LOCK_VICT_SHIFT	4
#define MMU_LOCK_VICT_MASK	(0x1f << MMU_LOCK_VICT_SHIFT)
#define MMU_LOCK_VICT(x)	\
	((x & MMU_LOCK_VICT_MASK) >> MMU_LOCK_VICT_SHIFT)

struct iotlb_lock {
	short base;
	short vict;
};

/* accommodate the difference between omap1 and omap2/3 */
static const struct iommu_functions *arch_iommu;

static struct platform_driver omap_iommu_driver;
static struct kmem_cache *iopte_cachep;

/**
 * omap_install_iommu_arch - Install archtecure specific iommu functions
 * @ops:	a pointer to architecture specific iommu functions
 *
 * There are several kind of iommu algorithm(tlb, pagetable) among
 * omap series. This interface installs such an iommu algorighm.
 **/
int omap_install_iommu_arch(const struct iommu_functions *ops)
{
	if (arch_iommu)
		return -EBUSY;

	arch_iommu = ops;
	return 0;
}
EXPORT_SYMBOL_GPL(omap_install_iommu_arch);

/**
 * omap_uninstall_iommu_arch - Uninstall archtecure specific iommu functions
 * @ops:	a pointer to architecture specific iommu functions
 *
 * This interface uninstalls the iommu algorighm installed previously.
 **/
void omap_uninstall_iommu_arch(const struct iommu_functions *ops)
{
	if (arch_iommu != ops)
		pr_err("%s: not your arch\n", __func__);

	arch_iommu = NULL;
}
EXPORT_SYMBOL_GPL(omap_uninstall_iommu_arch);

/**
 * omap_iommu_save_ctx - Save registers for pm off-mode support
 * @dev:	client device
 *
 * This should be treated as an deprecated API. It is preserved only
 * to maintain existing functionality for OMAP3 ISP driver, newer
 * SoCs will leverage the newer omap_iommu_domain_suspend() API and
 * associated runtime_suspend callback.
 **/
void omap_iommu_save_ctx(struct device *dev)
{
	struct omap_iommu *obj;
	struct omap_iommu_arch_data *arch_data = dev->archdata.iommu;

	while (arch_data->name) {
		obj = arch_data->iommu_dev;
		arch_iommu->save_ctx(obj);
		arch_data++;
	}
}
EXPORT_SYMBOL_GPL(omap_iommu_save_ctx);

/**
 * omap_iommu_restore_ctx - Restore registers for pm off-mode support
 * @dev:	client device
 *
 * This should be treated as an deprecated API. It is preserved only
 * to maintain existing functionality for OMAP3 ISP driver, newer
 * SoCs will leverage the newer omap_iommu_domain_resume() API and
 * associated runtime_resume callback.
 **/
void omap_iommu_restore_ctx(struct device *dev)
{
	struct omap_iommu *obj;
	struct omap_iommu_arch_data *arch_data = dev->archdata.iommu;

	while (arch_data->name) {
		obj = arch_data->iommu_dev;
		arch_iommu->restore_ctx(obj);
		arch_data++;
	}
}
EXPORT_SYMBOL_GPL(omap_iommu_restore_ctx);

static int iommu_enable(struct omap_iommu *obj)
{
	int ret;

	if (!arch_iommu)
		return -ENODEV;

	ret = pm_runtime_get_sync(obj->dev);
	if (ret < 0)
		pm_runtime_put_noidle(obj->dev);

	return ret < 0 ? ret : 0;
}

static void iommu_disable(struct omap_iommu *obj)
{
	if (!arch_iommu)
		return;

	pm_runtime_put_sync(obj->dev);
}

/*
 *	TLB operations
 */
void omap_iotlb_cr_to_e(struct cr_regs *cr, struct iotlb_entry *e)
{
	BUG_ON(!cr || !e);

	arch_iommu->cr_to_e(cr, e);
}
EXPORT_SYMBOL_GPL(omap_iotlb_cr_to_e);

static inline int iotlb_cr_valid(struct cr_regs *cr)
{
	if (!cr)
		return -EINVAL;

	return arch_iommu->cr_valid(cr);
}

static inline struct cr_regs *iotlb_alloc_cr(struct omap_iommu *obj,
					     struct iotlb_entry *e)
{
	if (!e)
		return NULL;

	return arch_iommu->alloc_cr(obj, e);
}

static u32 iotlb_cr_to_virt(struct cr_regs *cr)
{
	return arch_iommu->cr_to_virt(cr);
}

static u32 get_iopte_attr(struct iotlb_entry *e)
{
	return arch_iommu->get_pte_attr(e);
}

static u32 iommu_report_fault(struct omap_iommu *obj, u32 *da)
{
	return arch_iommu->fault_isr(obj, da);
}

static void iotlb_lock_get(struct omap_iommu *obj, struct iotlb_lock *l)
{
	u32 val;

	val = iommu_read_reg(obj, MMU_LOCK);

	l->base = MMU_LOCK_BASE(val);
	l->vict = MMU_LOCK_VICT(val);

}

static void iotlb_lock_set(struct omap_iommu *obj, struct iotlb_lock *l)
{
	u32 val;

	val = (l->base << MMU_LOCK_BASE_SHIFT);
	val |= (l->vict << MMU_LOCK_VICT_SHIFT);

	iommu_write_reg(obj, val, MMU_LOCK);
}

static void iotlb_read_cr(struct omap_iommu *obj, struct cr_regs *cr)
{
	arch_iommu->tlb_read_cr(obj, cr);
}

static void iotlb_load_cr(struct omap_iommu *obj, struct cr_regs *cr)
{
	arch_iommu->tlb_load_cr(obj, cr);

	iommu_write_reg(obj, 1, MMU_FLUSH_ENTRY);
	iommu_write_reg(obj, 1, MMU_LD_TLB);
}

/**
 * iotlb_dump_cr - Dump an iommu tlb entry into buf
 * @obj:	target iommu
 * @cr:		contents of cam and ram register
 * @buf:	output buffer
 **/
static inline ssize_t iotlb_dump_cr(struct omap_iommu *obj, struct cr_regs *cr,
				    char *buf)
{
	BUG_ON(!cr || !buf);

	return arch_iommu->dump_cr(obj, cr, buf);
}

/* only used in iotlb iteration for-loop */
static struct cr_regs __iotlb_read_cr(struct omap_iommu *obj, int n)
{
	struct cr_regs cr;
	struct iotlb_lock l;

	iotlb_lock_get(obj, &l);
	l.vict = n;
	iotlb_lock_set(obj, &l);
	iotlb_read_cr(obj, &cr);

	return cr;
}

/**
 * load_iotlb_entry - Set an iommu tlb entry
 * @obj:	target iommu
 * @e:		an iommu tlb entry info
 **/
#ifdef PREFETCH_IOTLB
static int load_iotlb_entry(struct omap_iommu *obj, struct iotlb_entry *e)
{
	int err = 0;
	struct iotlb_lock l;
	struct cr_regs *cr;

	if (!obj || !obj->nr_tlb_entries || !e)
		return -EINVAL;

	pm_runtime_get_sync(obj->dev);

	iotlb_lock_get(obj, &l);
	if (l.base == obj->nr_tlb_entries) {
		dev_warn(obj->dev, "%s: preserve entries full\n", __func__);
		err = -EBUSY;
		goto out;
	}
	if (!e->prsvd) {
		int i;
		struct cr_regs tmp;

		for_each_iotlb_cr(obj, obj->nr_tlb_entries, i, tmp)
			if (!iotlb_cr_valid(&tmp))
				break;

		if (i == obj->nr_tlb_entries) {
			dev_dbg(obj->dev, "%s: full: no entry\n", __func__);
			err = -EBUSY;
			goto out;
		}

		iotlb_lock_get(obj, &l);
	} else {
		l.vict = l.base;
		iotlb_lock_set(obj, &l);
	}

	cr = iotlb_alloc_cr(obj, e);
	if (IS_ERR(cr)) {
		pm_runtime_put_sync(obj->dev);
		return PTR_ERR(cr);
	}

	iotlb_load_cr(obj, cr);
	kfree(cr);

	if (e->prsvd)
		l.base++;
	/* increment victim for next tlb load */
	if (++l.vict == obj->nr_tlb_entries)
		l.vict = l.base;
	iotlb_lock_set(obj, &l);
out:
	pm_runtime_put_sync(obj->dev);
	return err;
}

#else /* !PREFETCH_IOTLB */

static int load_iotlb_entry(struct omap_iommu *obj, struct iotlb_entry *e)
{
	return 0;
}

#endif /* !PREFETCH_IOTLB */

static int prefetch_iotlb_entry(struct omap_iommu *obj, struct iotlb_entry *e)
{
	return load_iotlb_entry(obj, e);
}

/**
 * flush_iotlb_page - Clear an iommu tlb entry
 * @obj:	target iommu
 * @da:		iommu device virtual address
 *
 * Clear an iommu tlb entry which includes 'da' address.
 **/
static void flush_iotlb_page(struct omap_iommu *obj, u32 da)
{
	int i;
	struct cr_regs cr;

	pm_runtime_get_sync(obj->dev);

	for_each_iotlb_cr(obj, obj->nr_tlb_entries, i, cr) {
		u32 start;
		size_t bytes;

		if (!iotlb_cr_valid(&cr))
			continue;

		start = iotlb_cr_to_virt(&cr);
		bytes = iopgsz_to_bytes(cr.cam & 3);

		if ((start <= da) && (da < start + bytes)) {
			dev_dbg(obj->dev, "%s: %08x<=%08x(%x)\n",
				__func__, start, da, bytes);
			iotlb_load_cr(obj, &cr);
			iommu_write_reg(obj, 1, MMU_FLUSH_ENTRY);
			break;
		}
	}
	pm_runtime_put_sync(obj->dev);

	if (i == obj->nr_tlb_entries)
		dev_dbg(obj->dev, "%s: no page for %08x\n", __func__, da);
}

/**
 * flush_iotlb_all - Clear all iommu tlb entries
 * @obj:	target iommu
 **/
static void flush_iotlb_all(struct omap_iommu *obj)
{
	struct iotlb_lock l;

	pm_runtime_get_sync(obj->dev);

	l.base = 0;
	l.vict = 0;
	iotlb_lock_set(obj, &l);

	iommu_write_reg(obj, 1, MMU_GFLUSH);

	pm_runtime_put_sync(obj->dev);
}

#if defined(CONFIG_OMAP_IOMMU_DEBUG) || defined(CONFIG_OMAP_IOMMU_DEBUG_MODULE)

ssize_t omap_iommu_dump_ctx(struct omap_iommu *obj, char *buf, ssize_t bytes)
{
	if (!obj || !buf)
		return -EINVAL;

	pm_runtime_get_sync(obj->dev);

	bytes = arch_iommu->dump_ctx(obj, buf, bytes);

	pm_runtime_put_sync(obj->dev);

	return bytes;
}
EXPORT_SYMBOL_GPL(omap_iommu_dump_ctx);

static int
__dump_tlb_entries(struct omap_iommu *obj, struct cr_regs *crs, int num)
{
	int i;
	struct iotlb_lock saved;
	struct cr_regs tmp;
	struct cr_regs *p = crs;

	pm_runtime_get_sync(obj->dev);
	iotlb_lock_get(obj, &saved);

	for_each_iotlb_cr(obj, num, i, tmp) {
		if (!iotlb_cr_valid(&tmp))
			continue;
		*p++ = tmp;
	}

	iotlb_lock_set(obj, &saved);
	pm_runtime_put_sync(obj->dev);

	return  p - crs;
}

/**
 * omap_dump_tlb_entries - dump cr arrays to given buffer
 * @obj:	target iommu
 * @buf:	output buffer
 **/
size_t omap_dump_tlb_entries(struct omap_iommu *obj, char *buf, ssize_t bytes)
{
	int i, num;
	struct cr_regs *cr;
	char *p = buf;

	num = bytes / sizeof(*cr);
	num = min(obj->nr_tlb_entries, num);

	cr = kcalloc(num, sizeof(*cr), GFP_KERNEL);
	if (!cr)
		return 0;

	num = __dump_tlb_entries(obj, cr, num);
	for (i = 0; i < num; i++)
		p += iotlb_dump_cr(obj, cr + i, p);
	kfree(cr);

	return p - buf;
}
EXPORT_SYMBOL_GPL(omap_dump_tlb_entries);

int omap_foreach_iommu_device(void *data, int (*fn)(struct device *, void *))
{
	return driver_for_each_device(&omap_iommu_driver.driver,
				      NULL, data, fn);
}
EXPORT_SYMBOL_GPL(omap_foreach_iommu_device);

#endif /* CONFIG_OMAP_IOMMU_DEBUG_MODULE */

/*
 *	H/W pagetable operations
 */
static void flush_iopgd_range(u32 *first, u32 *last)
{
	dmac_flush_range(first, last);
	outer_flush_range(virt_to_phys(first), virt_to_phys(last));
}

static void flush_iopte_range(u32 *first, u32 *last)
{
	dmac_flush_range(first, last);
	outer_flush_range(virt_to_phys(first), virt_to_phys(last));
}

static void iopte_free(u32 *iopte)
{
	/* Note: freed iopte's must be clean ready for re-use */
	if (iopte)
		kmem_cache_free(iopte_cachep, iopte);
}

static u32 *iopte_alloc(struct omap_iommu *obj, u32 *iopgd, u32 da)
{
	u32 *iopte;

	/* a table has already existed */
	if (*iopgd)
		goto pte_ready;

	/*
	 * do the allocation outside the page table lock
	 */
	spin_unlock(&obj->page_table_lock);
	iopte = kmem_cache_zalloc(iopte_cachep, GFP_KERNEL);
	spin_lock(&obj->page_table_lock);

	if (!*iopgd) {
		if (!iopte)
			return ERR_PTR(-ENOMEM);

		*iopgd = virt_to_phys(iopte) | IOPGD_TABLE;
		flush_iopgd_range(iopgd, iopgd + 1);

		dev_vdbg(obj->dev, "%s: a new pte:%p\n", __func__, iopte);
	} else {
		/* We raced, free the reduniovant table */
		iopte_free(iopte);
	}

pte_ready:
	iopte = iopte_offset(iopgd, da);

	dev_vdbg(obj->dev,
		 "%s: da:%08x pgd:%p *pgd:%08x pte:%p *pte:%08x\n",
		 __func__, da, iopgd, *iopgd, iopte, *iopte);

	return iopte;
}

static int iopgd_alloc_section(struct omap_iommu *obj, u32 da, u32 pa, u32 prot)
{
	u32 *iopgd = iopgd_offset(obj, da);

	if ((da | pa) & ~IOSECTION_MASK) {
		dev_err(obj->dev, "%s: %08x:%08x should aligned on %08lx\n",
			__func__, da, pa, IOSECTION_SIZE);
		return -EINVAL;
	}

	*iopgd = (pa & IOSECTION_MASK) | prot | IOPGD_SECTION;
	flush_iopgd_range(iopgd, iopgd + 1);
	return 0;
}

static int iopgd_alloc_super(struct omap_iommu *obj, u32 da, u32 pa, u32 prot)
{
	u32 *iopgd = iopgd_offset(obj, da);
	int i;

	if ((da | pa) & ~IOSUPER_MASK) {
		dev_err(obj->dev, "%s: %08x:%08x should aligned on %08lx\n",
			__func__, da, pa, IOSUPER_SIZE);
		return -EINVAL;
	}

	for (i = 0; i < 16; i++)
		*(iopgd + i) = (pa & IOSUPER_MASK) | prot | IOPGD_SUPER;
	flush_iopgd_range(iopgd, iopgd + 16);
	return 0;
}

static int iopte_alloc_page(struct omap_iommu *obj, u32 da, u32 pa, u32 prot)
{
	u32 *iopgd = iopgd_offset(obj, da);
	u32 *iopte = iopte_alloc(obj, iopgd, da);

	if (IS_ERR(iopte))
		return PTR_ERR(iopte);

	*iopte = (pa & IOPAGE_MASK) | prot | IOPTE_SMALL;
	flush_iopte_range(iopte, iopte + 1);

	dev_vdbg(obj->dev, "%s: da:%08x pa:%08x pte:%p *pte:%08x\n",
		 __func__, da, pa, iopte, *iopte);

	return 0;
}

static int iopte_alloc_large(struct omap_iommu *obj, u32 da, u32 pa, u32 prot)
{
	u32 *iopgd = iopgd_offset(obj, da);
	u32 *iopte = iopte_alloc(obj, iopgd, da);
	int i;

	if ((da | pa) & ~IOLARGE_MASK) {
		dev_err(obj->dev, "%s: %08x:%08x should aligned on %08lx\n",
			__func__, da, pa, IOLARGE_SIZE);
		return -EINVAL;
	}

	if (IS_ERR(iopte))
		return PTR_ERR(iopte);

	for (i = 0; i < 16; i++)
		*(iopte + i) = (pa & IOLARGE_MASK) | prot | IOPTE_LARGE;
	flush_iopte_range(iopte, iopte + 16);
	return 0;
}

static int
iopgtable_store_entry_core(struct omap_iommu *obj, struct iotlb_entry *e)
{
	int (*fn)(struct omap_iommu *, u32, u32, u32);
	u32 prot;
	int err;

	if (!obj || !e)
		return -EINVAL;

	switch (e->pgsz) {
	case MMU_CAM_PGSZ_16M:
		fn = iopgd_alloc_super;
		break;
	case MMU_CAM_PGSZ_1M:
		fn = iopgd_alloc_section;
		break;
	case MMU_CAM_PGSZ_64K:
		fn = iopte_alloc_large;
		break;
	case MMU_CAM_PGSZ_4K:
		fn = iopte_alloc_page;
		break;
	default:
		fn = NULL;
		BUG();
		break;
	}

	prot = get_iopte_attr(e);

	spin_lock(&obj->page_table_lock);
	err = fn(obj, e->da, e->pa, prot);
	spin_unlock(&obj->page_table_lock);

	return err;
}

/**
 * omap_iopgtable_store_entry - Make an iommu pte entry
 * @obj:	target iommu
 * @e:		an iommu tlb entry info
 **/
int omap_iopgtable_store_entry(struct omap_iommu *obj, struct iotlb_entry *e)
{
	int err;

	flush_iotlb_page(obj, e->da);
	err = iopgtable_store_entry_core(obj, e);
	if (!err)
		prefetch_iotlb_entry(obj, e);
	return err;
}
EXPORT_SYMBOL_GPL(omap_iopgtable_store_entry);

/**
 * iopgtable_lookup_entry - Lookup an iommu pte entry
 * @obj:	target iommu
 * @da:		iommu device virtual address
 * @ppgd:	iommu pgd entry pointer to be returned
 * @ppte:	iommu pte entry pointer to be returned
 **/
static void
iopgtable_lookup_entry(struct omap_iommu *obj, u32 da, u32 **ppgd, u32 **ppte)
{
	u32 *iopgd, *iopte = NULL;

	iopgd = iopgd_offset(obj, da);
	if (!*iopgd)
		goto out;

	if (iopgd_is_table(*iopgd))
		iopte = iopte_offset(iopgd, da);
out:
	*ppgd = iopgd;
	*ppte = iopte;
}

static size_t iopgtable_clear_entry_core(struct omap_iommu *obj, u32 da)
{
	size_t bytes;
	u32 *iopgd = iopgd_offset(obj, da);
	int nent = 1;

	if (!*iopgd)
		return 0;

	if (iopgd_is_table(*iopgd)) {
		int i;
		u32 *iopte = iopte_offset(iopgd, da);

		bytes = IOPTE_SIZE;
		if (*iopte & IOPTE_LARGE) {
			nent *= 16;
			/* rewind to the 1st entry */
			iopte = iopte_offset(iopgd, (da & IOLARGE_MASK));
		}
		bytes *= nent;
		memset(iopte, 0, nent * sizeof(*iopte));
		flush_iopte_range(iopte, iopte + (nent - 1) * sizeof(*iopte));

		/*
		 * do table walk to check if this table is necessary or not
		 */
		iopte = iopte_offset(iopgd, 0);
		for (i = 0; i < PTRS_PER_IOPTE; i++)
			if (iopte[i])
				goto out;

		iopte_free(iopte);
		nent = 1; /* for the next L1 entry */
	} else {
		bytes = IOPGD_SIZE;
		if ((*iopgd & IOPGD_SUPER) == IOPGD_SUPER) {
			nent *= 16;
			/* rewind to the 1st entry */
			iopgd = iopgd_offset(obj, (da & IOSUPER_MASK));
		}
		bytes *= nent;
	}
	memset(iopgd, 0, nent * sizeof(*iopgd));
	flush_iopgd_range(iopgd, iopgd + (nent - 1) * sizeof(*iopgd));
out:
	return bytes;
}

/**
 * iopgtable_clear_entry - Remove an iommu pte entry
 * @obj:	target iommu
 * @da:		iommu device virtual address
 **/
static size_t iopgtable_clear_entry(struct omap_iommu *obj, u32 da)
{
	size_t bytes;

	spin_lock(&obj->page_table_lock);

	bytes = iopgtable_clear_entry_core(obj, da);
	flush_iotlb_page(obj, da);

	spin_unlock(&obj->page_table_lock);

	return bytes;
}

static void iopgtable_clear_entry_all(struct omap_iommu *obj)
{
	int i;

	spin_lock(&obj->page_table_lock);

	for (i = 0; i < PTRS_PER_IOPGD; i++) {
		u32 da;
		u32 *iopgd;

		da = i << IOPGD_SHIFT;
		iopgd = iopgd_offset(obj, da);

		if (!*iopgd)
			continue;

		if (iopgd_is_table(*iopgd))
			iopte_free(iopte_offset(iopgd, 0));

		*iopgd = 0;
		flush_iopgd_range(iopgd, iopgd + 1);
	}

	flush_iotlb_all(obj);

	spin_unlock(&obj->page_table_lock);
}

/*
 *	Device IOMMU generic operations
 */
static irqreturn_t iommu_fault_handler(int irq, void *data)
{
	u32 da, errs;
	u32 *iopgd, *iopte;
	struct omap_iommu *obj = data;
	struct iommu_domain *domain = obj->domain;
	struct omap_iommu_domain *omap_domain = domain->priv;

	if (!omap_domain->attached)
		return IRQ_NONE;

	errs = iommu_report_fault(obj, &da);
	if (errs == 0)
		return IRQ_HANDLED;

	/* Fault callback or TLB/PTE Dynamic loading */
	if (!report_iommu_fault(domain, obj->dev, da, 0))
		return IRQ_HANDLED;

	iommu_write_reg(obj, 0, MMU_IRQENABLE);

	iopgd = iopgd_offset(obj, da);

	if (!iopgd_is_table(*iopgd)) {
		dev_err(obj->dev, "%s: errs:0x%08x da:0x%08x pgd:0x%p *pgd:px%08x\n",
				obj->name, errs, da, iopgd, *iopgd);
		return IRQ_NONE;
	}

	iopte = iopte_offset(iopgd, da);

	dev_err(obj->dev, "%s: errs:0x%08x da:0x%08x pgd:0x%p *pgd:0x%08x pte:0x%p *pte:0x%08x\n",
			obj->name, errs, da, iopgd, *iopgd, iopte, *iopte);

	return IRQ_NONE;
}

static int device_match_by_alias(struct device *dev, void *data)
{
	struct omap_iommu *obj = to_iommu(dev);
	const char *name = data;

	pr_debug("%s: %s %s\n", __func__, obj->name, name);

	return strcmp(obj->name, name) == 0;
}

/**
 * omap_iommu_attach() - attach iommu device to an iommu domain
 * @name:	name of target omap iommu device
 * @iopgd:	page table
 **/
static struct omap_iommu *omap_iommu_attach(const char *name, u32 *iopgd)
{
	int err;
	struct device *dev;
	struct omap_iommu *obj;

	dev = driver_find_device(&omap_iommu_driver.driver, NULL,
				(void *)name,
				device_match_by_alias);
	if (!dev)
		return ERR_PTR(-ENODEV);

	obj = to_iommu(dev);

	spin_lock(&obj->iommu_lock);

	obj->iopgd = iopgd;
	err = iommu_enable(obj);
	if (err)
		goto err_enable;
	flush_iotlb_all(obj);

	spin_unlock(&obj->iommu_lock);

	dev_dbg(obj->dev, "%s: %s\n", __func__, obj->name);
	return obj;

err_enable:
	spin_unlock(&obj->iommu_lock);
	return ERR_PTR(err);
}

/**
 * omap_iommu_detach - release iommu device
 * @obj:	target iommu
 **/
static void omap_iommu_detach(struct omap_iommu *obj)
{
	if (!obj || IS_ERR(obj))
		return;

	spin_lock(&obj->iommu_lock);

	obj->iopgd = NULL;
	iommu_disable(obj);

	spin_unlock(&obj->iommu_lock);

	dev_dbg(obj->dev, "%s: %s\n", __func__, obj->name);
}

/**
 * omap_iommu_domain_suspend - suspend the iommu device
 * @domain: iommu domain attached to the target iommu device
 * @auto_suspend: flag to indicate system suspend or runtime suspend
 *
 * This API allows the client devices of IOMMU devices to suspend
 * the IOMMUs they control, after they are idled and suspended all
 * activity.
 **/
int omap_iommu_domain_suspend(struct iommu_domain *domain, bool auto_suspend)
{
	struct omap_iommu_domain *omap_domain = domain->priv;
	struct omap_iommu_device *iommu;
	struct omap_iommu *oiommu;
	int i;

	if (!omap_domain->attached)
		return 0;

	iommu = omap_domain->iommus;
	iommu += (omap_domain->num_iommus - 1);
	for (i = 0; i < omap_domain->num_iommus; i++, iommu--) {
		oiommu = iommu->iommu_dev;
		if (auto_suspend) {
			pm_runtime_put_sync(oiommu->dev);
		} else {
			pm_runtime_put_noidle(oiommu->dev);
			pm_generic_runtime_suspend(oiommu->dev);
			pm_runtime_disable(oiommu->dev);
			pm_runtime_set_suspended(oiommu->dev);
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(omap_iommu_domain_suspend);

/**
 * omap_iommu_domain_resume - resume the iommu device
 * @domain: iommu domain attached to the target iommu device
 * @auto_suspend: flag to indicate system suspend or runtime suspend
 *
 * This API allows the client devices of IOMMU devices to resume
 * the IOMMUs they control, before they can resume operations.
 **/
int omap_iommu_domain_resume(struct iommu_domain *domain, bool auto_suspend)
{
	struct omap_iommu_domain *omap_domain = domain->priv;
	struct omap_iommu_device *iommu;
	struct omap_iommu *oiommu;
	int i;

	if (!omap_domain->attached)
		return 0;

	iommu = omap_domain->iommus;
	for (i = 0; i < omap_domain->num_iommus; i++, iommu++) {
		oiommu = iommu->iommu_dev;
		if (auto_suspend) {
			pm_runtime_get_sync(oiommu->dev);
		} else {
			pm_runtime_set_active(oiommu->dev);
			pm_runtime_enable(oiommu->dev);
			pm_runtime_get_noresume(oiommu->dev);
			pm_generic_runtime_resume(oiommu->dev);
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(omap_iommu_domain_resume);

static void omap_iommu_save_tlb_entries(struct omap_iommu *obj)
{
	struct iotlb_lock lock;
	struct cr_regs cr;
	struct cr_regs *tmp;
	int i;

	/* check if there are any locked tlbs to save */
	iotlb_lock_get(obj, &lock);
	obj->num_cr_ctx = lock.base;
	if (!obj->num_cr_ctx)
		return;

	tmp = obj->cr_ctx;
	for_each_iotlb_cr(obj, obj->num_cr_ctx, i, cr)
		*tmp++ = cr;
}

static void omap_iommu_restore_tlb_entries(struct omap_iommu *obj)
{
	struct iotlb_lock l;
	struct cr_regs *tmp;
	int i;

	/* no locked tlbs to restore */
	if (!obj->num_cr_ctx)
		return;

	l.base = 0;
	tmp = obj->cr_ctx;
	for (i = 0; i < obj->num_cr_ctx; i++, tmp++) {
		l.vict = i;
		iotlb_lock_set(obj, &l);
		iotlb_load_cr(obj, tmp);
	}
	l.base = obj->num_cr_ctx;
	l.vict = i;
	iotlb_lock_set(obj, &l);
}

/**
 * omap_iommu_runtime_suspend - disable an iommu device
 * @dev:	iommu device
 *
 * This function performs all that is necessary to disable an
 * IOMMU device, either during final detachment from a client
 * device, or during system/runtime suspend of the device. This
 * includes programming all the appropriate IOMMU registers, and
 * managing the associated omap_hwmod's state and the device's
 * reset line. This function also saves the context of any
 * locked TLBs if suspending.
 **/
static int omap_iommu_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct iommu_platform_data *pdata = dev_get_platdata(dev);
	struct omap_iommu *obj = to_iommu(dev);
	int ret;

	/* save the TLBs only during suspend, and not for power down */
	if (obj->domain && obj->iopgd)
		omap_iommu_save_tlb_entries(obj);

	if (arch_iommu->disable)
		arch_iommu->disable(obj);

	if (pdata && pdata->device_idle)
		pdata->device_idle(pdev);

	if (pdata && pdata->assert_reset)
		pdata->assert_reset(pdev, pdata->reset_name);

	if (pdata && pdata->set_pwrdm_constraint) {
		ret = pdata->set_pwrdm_constraint(pdev, false, &obj->pwrst);
		if (ret) {
			dev_warn(dev, "pwrdm_constraint failed to be reset, status = %d\n",
				 ret);
		}
	}

	return 0;
}

/**
 * omap_iommu_runtime_resume - enable an iommu device
 * @dev:	iommu device
 *
 * This function performs all that is necessary to enable an
 * IOMMU device, either during initial attachment to a client
 * device, or during system/runtime resume of the device. This
 * includes programming all the appropriate IOMMU registers, and
 * managing the associated omap_hwmod's state and the device's
 * reset line. The function also restores any locked TLBs if
 * resuming after a suspend.
 **/
static int omap_iommu_runtime_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct iommu_platform_data *pdata = dev_get_platdata(dev);
	struct omap_iommu *obj = to_iommu(dev);
	int ret = 0;

	if (pdata && pdata->set_pwrdm_constraint) {
		ret = pdata->set_pwrdm_constraint(pdev, true, &obj->pwrst);
		if (ret) {
			dev_warn(dev, "pwrdm_constraint failed to be set, status = %d\n",
				 ret);
		}
	}

	if (pdata && pdata->deassert_reset) {
		ret = pdata->deassert_reset(pdev, pdata->reset_name);
		if (ret) {
			dev_err(dev, "deassert_reset failed: %d\n", ret);
			return ret;
		}
	}

	if (pdata && pdata->device_enable)
		pdata->device_enable(pdev);

	/* restore the TLBs only during resume, and not for power up */
	if (obj->domain)
		omap_iommu_restore_tlb_entries(obj);

	if (arch_iommu->enable)
		ret = arch_iommu->enable(obj);

	return ret;
}

static int omap_iommu_dra7_get_dsp_system_cfg(struct platform_device *pdev,
					      struct omap_iommu *obj,
					      resource_size_t start)
{
	struct device_node *np = pdev->dev.of_node;
	struct resource *res;

	if (!of_device_is_compatible(np, "ti,dra7-iommu"))
		return 0;

	/*
	 * Assign the SYS_MMU enable MMU index for DRA7 DSP MMUs,
	 * nothing to process for DRA7 IPU MMUs
	 */
	if (start == 0x40D01000 || start == 0x41501000)
		obj->id = 0;
	else if (start == 0x40D02000 || start == 0x41502000)
		obj->id = 1;
	else
		return 0;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dsp_system");
	if (res) {
		/*
		 * The DSP_SYSTEM space is common across two instances of MMU
		 * and there is no common object for storing common data, so the
		 * resource is deliberately not locked using request_mem_region
		 */
		obj->syscfgbase = devm_ioremap(obj->dev, res->start,
					       resource_size(res));
		if (!obj->syscfgbase)
			return -ENOMEM;
	} else {
		dev_err(&pdev->dev, "dsp_system register space is missing\n");
		return -EINVAL;
	}

	return 0;
}

/*
 *	OMAP Device MMU(IOMMU) detection
 */
static int omap_iommu_probe(struct platform_device *pdev)
{
	int err = -ENODEV;
	int irq;
	struct omap_iommu *obj;
	struct resource *res;
	struct iommu_platform_data *pdata = pdev->dev.platform_data;
	struct device_node *of = pdev->dev.of_node;

	obj = devm_kzalloc(&pdev->dev, sizeof(*obj) + MMU_REG_SIZE, GFP_KERNEL);
	if (!obj)
		return -ENOMEM;

	/*
	 * self-manage the ordering dependencies between omap_device_enable/idle
	 * and omap_device_assert/deassert_hardreset API
	 */
	if (pdev->dev.pm_domain) {
		dev_dbg(&pdev->dev, "device pm_domain is being reset\n");
		pdev->dev.pm_domain = NULL;
	}

	if (of) {
		obj->name = dev_name(&pdev->dev);
		obj->nr_tlb_entries = 32;
		err = of_property_read_u32(of, "ti,#tlb-entries",
					   &obj->nr_tlb_entries);
		if (err && err != -EINVAL)
			return err;
		if (obj->nr_tlb_entries != 32 && obj->nr_tlb_entries != 8)
			return -EINVAL;
		/*
		 * da_start and da_end are needed for omap-iovmm, so hardcode
		 * these values as used by OMAP3 ISP - the only user for
		 * omap-iovmm
		 */
		obj->da_start = 0;
		obj->da_end = 0xfffff000;
		if (of_find_property(of, "ti,iommu-bus-err-back", NULL))
			obj->has_bus_err_back = MMU_GP_REG_BUS_ERR_BACK_EN;
	} else {
		obj->nr_tlb_entries = pdata->nr_tlb_entries;
		obj->name = pdata->name;
		obj->da_start = pdata->da_start;
		obj->da_end = pdata->da_end;
	}
	if (obj->da_end <= obj->da_start)
		return -EINVAL;

	obj->dev = &pdev->dev;
	obj->ctx = (void *)obj + sizeof(*obj);
	obj->cr_ctx = devm_kzalloc(&pdev->dev,
				   sizeof(*obj->cr_ctx) * obj->nr_tlb_entries,
				   GFP_KERNEL);
	if (!obj->cr_ctx)
		return -ENOMEM;

	spin_lock_init(&obj->iommu_lock);
	mutex_init(&obj->mmap_lock);
	spin_lock_init(&obj->page_table_lock);
	INIT_LIST_HEAD(&obj->mmap);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	obj->regbase = devm_ioremap_resource(obj->dev, res);
	if (IS_ERR(obj->regbase))
		return PTR_ERR(obj->regbase);

	err = omap_iommu_dra7_get_dsp_system_cfg(pdev, obj, res->start);
	if (err)
		return err;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return -ENODEV;

	err = devm_request_irq(obj->dev, irq, iommu_fault_handler, IRQF_SHARED,
			       dev_name(obj->dev), obj);
	if (err < 0)
		return err;
	platform_set_drvdata(pdev, obj);

	pm_runtime_irq_safe(obj->dev);
	pm_runtime_enable(obj->dev);

	dev_info(&pdev->dev, "%s registered\n", obj->name);
	return 0;
}

static int omap_iommu_remove(struct platform_device *pdev)
{
	struct omap_iommu *obj = platform_get_drvdata(pdev);

	iopgtable_clear_entry_all(obj);

	pm_runtime_disable(obj->dev);

	dev_info(&pdev->dev, "%s removed\n", obj->name);
	return 0;
}

static const struct dev_pm_ops omap_iommu_pm_ops = {
	SET_RUNTIME_PM_OPS(omap_iommu_runtime_suspend,
			   omap_iommu_runtime_resume, NULL)
};

static struct of_device_id omap_iommu_of_match[] = {
	{ .compatible = "ti,omap2-iommu" },
	{ .compatible = "ti,omap4-iommu" },
	{ .compatible = "ti,dra7-iommu"	},
	{},
};
MODULE_DEVICE_TABLE(of, omap_iommu_of_match);

static struct platform_driver omap_iommu_driver = {
	.probe	= omap_iommu_probe,
	.remove	= omap_iommu_remove,
	.driver	= {
		.name	= "omap-iommu",
		.pm	= &omap_iommu_pm_ops,
		.of_match_table = of_match_ptr(omap_iommu_of_match),
	},
};

static void iopte_cachep_ctor(void *iopte)
{
	clean_dcache_area(iopte, IOPTE_TABLE_SIZE);
}

static u32 iotlb_init_entry(struct iotlb_entry *e, u32 da, u32 pa, int pgsz)
{
	memset(e, 0, sizeof(*e));

	e->da		= da;
	e->pa		= pa;
	e->valid	= MMU_CAM_V;
	/* FIXME: add OMAP1 support */
	e->pgsz		= pgsz;
	e->endian	= MMU_RAM_ENDIAN_LITTLE;
	e->elsz		= MMU_RAM_ELSZ_8;
	e->mixed	= 0;

	return iopgsz_to_bytes(e->pgsz);
}

static int omap_iommu_map(struct iommu_domain *domain, unsigned long da,
			 phys_addr_t pa, size_t bytes, int prot)
{
	struct omap_iommu_domain *omap_domain = domain->priv;
	struct omap_iommu_device *iommu;
	struct omap_iommu *oiommu;
	struct device *dev = omap_domain->dev;
	struct iotlb_entry e;
	int omap_pgsz;
	u32 ret = -EINVAL;
	int i;

	omap_pgsz = bytes_to_iopgsz(bytes);
	if (omap_pgsz < 0) {
		dev_err(dev, "invalid size to map: %d\n", bytes);
		return -EINVAL;
	}

	dev_dbg(dev, "mapping da 0x%lx to pa %pa size 0x%x\n", da, &pa, bytes);

	iotlb_init_entry(&e, da, pa, omap_pgsz);

	iommu = omap_domain->iommus;
	for (i = 0; i < omap_domain->num_iommus; i++, iommu++) {
		oiommu = iommu->iommu_dev;
		ret = omap_iopgtable_store_entry(oiommu, &e);
		if (ret) {
			dev_err(dev, "omap_iopgtable_store_entry failed: %d\n",
				ret);
			break;
		}
	}

	if (ret) {
		while (i--) {
			iommu--;
			oiommu = iommu->iommu_dev;
			iopgtable_clear_entry(oiommu, da);
		};
	}

	return ret;
}

static size_t omap_iommu_unmap(struct iommu_domain *domain, unsigned long da,
			    size_t size)
{
	struct omap_iommu_domain *omap_domain = domain->priv;
	struct omap_iommu_device *iommu;
	struct omap_iommu *oiommu;
	struct device *dev = omap_domain->dev;
	bool error = false;
	size_t bytes = 0;
	int i;

	dev_dbg(dev, "unmapping da 0x%lx size %u\n", da, size);

	iommu = omap_domain->iommus;
	for (i = 0; i < omap_domain->num_iommus; i++, iommu++) {
		oiommu = iommu->iommu_dev;
		bytes = iopgtable_clear_entry(oiommu, da);
		if (!bytes)
			error = true;
	}

	/*
	 * simplify return - we are only checking if any of the iommus
	 * reported an error, but not if all of them are unmapping the
	 * same number of entries. This should not occur due to the
	 * mirror programming.
	 */
	return error ? 0 : bytes;
}

static int omap_iommu_count(struct device *dev)
{
	int count = 0;
	struct omap_iommu_arch_data *arch_data = dev->archdata.iommu;

	while (arch_data->name) {
		count++;
		arch_data++;
	}

	return count;
}

/* caller should call cleanup if this function fails */
static int omap_iommu_attach_init(struct device *dev,
				  struct omap_iommu_domain *odomain)
{
	struct omap_iommu_device *iommu;
	int i;

	odomain->num_iommus = omap_iommu_count(dev);
	if (!odomain->num_iommus)
		return -EINVAL;

	odomain->iommus = kzalloc(odomain->num_iommus * sizeof(*iommu),
				  GFP_ATOMIC);
	if (!odomain->iommus)
		return -ENOMEM;

	iommu = odomain->iommus;
	for (i = 0; i < odomain->num_iommus; i++, iommu++) {
		iommu->pgtable = kzalloc(IOPGD_TABLE_SIZE, GFP_ATOMIC);
		if (!iommu->pgtable)
			return -ENOMEM;

		/*
		 * should never fail, but please keep this around to ensure
		 * we keep the hardware happy
		 */
		BUG_ON(!IS_ALIGNED((long)iommu->pgtable, IOPGD_TABLE_SIZE));

		clean_dcache_area(iommu->pgtable, IOPGD_TABLE_SIZE);
	}

	return 0;
}

static void omap_iommu_detach_fini(struct omap_iommu_domain *odomain)
{
	int i;
	struct omap_iommu_device *iommu = odomain->iommus;

	for (i = 0; iommu && i < odomain->num_iommus; i++, iommu++)
		kfree(iommu->pgtable);

	kfree(odomain->iommus);
	odomain->num_iommus = 0;
	odomain->iommus = NULL;
}

static int
omap_iommu_attach_dev(struct iommu_domain *domain, struct device *dev)
{
	struct omap_iommu_domain *omap_domain = domain->priv;
	struct omap_iommu_device *iommu;
	struct omap_iommu *oiommu;
	struct omap_iommu_arch_data *arch_data = dev->archdata.iommu;
	int ret = 0;
	int i;

	if (!arch_data || !arch_data->name) {
		dev_err(dev, "device doesn't have an associated iommu\n");
		return -EINVAL;
	}

	spin_lock(&omap_domain->lock);

	/* only a single client device can be attached to a domain */
	if (omap_domain->attached) {
		dev_err(dev, "iommu domain is already attached\n");
		ret = -EBUSY;
		goto out;
	}

	ret = omap_iommu_attach_init(dev, omap_domain);
	if (ret) {
		dev_err(dev, "failed to allocate required iommu data %d\n",
			ret);
		goto init_fail;
	}

	iommu = omap_domain->iommus;
	for (i = 0; i < omap_domain->num_iommus; i++, iommu++, arch_data++) {
		/* get a handle to and enable the omap iommu */
		oiommu = omap_iommu_attach(arch_data->name, iommu->pgtable);
		if (IS_ERR(oiommu)) {
			ret = PTR_ERR(oiommu);
			dev_err(dev, "can't get omap iommu: %d\n", ret);
			goto attach_fail;
		}
		oiommu->domain = domain;
		iommu->iommu_dev = oiommu;
		arch_data->iommu_dev = oiommu;
	}

	omap_domain->dev = dev;
	omap_domain->attached = 1;
	goto out;

attach_fail:
	while (i--) {
		iommu--;
		arch_data--;
		oiommu = iommu->iommu_dev;
		omap_iommu_detach(oiommu);
		iommu->iommu_dev = NULL;
		arch_data->iommu_dev = NULL;
		oiommu->domain = NULL;
	};
init_fail:
	omap_iommu_detach_fini(omap_domain);
out:
	spin_unlock(&omap_domain->lock);
	return ret;
}

static void _omap_iommu_detach_dev(struct omap_iommu_domain *omap_domain,
			struct device *dev)
{
	struct omap_iommu *oiommu;
	struct omap_iommu_arch_data *arch_data = dev->archdata.iommu;
	struct omap_iommu_device *iommu = omap_domain->iommus;
	int i;

	if (!omap_domain->attached) {
		dev_err(dev, "domain has no attached devices.\n");
		return;
	}

	/* only a single device is supported per domain for now */
	if (omap_domain->dev != dev) {
		dev_err(dev, "invalid attached device\n");
		return;
	}

	/*
	 * cleanup in the reverse order of attachment - this addresses
	 * any h/w dependencies between multiple instances, if any
	 */
	iommu += (omap_domain->num_iommus - 1);
	arch_data += (omap_domain->num_iommus - 1);
	for (i = 0; i < omap_domain->num_iommus; i++, iommu--, arch_data--) {
		oiommu = iommu->iommu_dev;
		iopgtable_clear_entry_all(oiommu);

		omap_iommu_detach(oiommu);
		iommu->iommu_dev = NULL;
		arch_data->iommu_dev = NULL;
		oiommu->domain = NULL;
	}

	omap_iommu_detach_fini(omap_domain);

	omap_domain->dev = NULL;
	omap_domain->attached = 0;
}

static void omap_iommu_detach_dev(struct iommu_domain *domain,
				 struct device *dev)
{
	struct omap_iommu_domain *omap_domain = domain->priv;

	spin_lock(&omap_domain->lock);
	_omap_iommu_detach_dev(omap_domain, dev);
	spin_unlock(&omap_domain->lock);
}

static int omap_iommu_domain_init(struct iommu_domain *domain)
{
	struct omap_iommu_domain *omap_domain;

	omap_domain = kzalloc(sizeof(*omap_domain), GFP_KERNEL);
	if (!omap_domain)
		return -ENOMEM;

	spin_lock_init(&omap_domain->lock);

	domain->priv = omap_domain;

	domain->geometry.aperture_start = 0;
	domain->geometry.aperture_end   = (1ULL << 32) - 1;
	domain->geometry.force_aperture = true;

	return 0;
}

static void omap_iommu_domain_destroy(struct iommu_domain *domain)
{
	struct omap_iommu_domain *omap_domain = domain->priv;

	domain->priv = NULL;

	/*
	 * An iommu device is still attached
	 * (currently, only one device can be attached) ?
	 */
	if (omap_domain->attached)
		_omap_iommu_detach_dev(omap_domain, omap_domain->dev);

	kfree(omap_domain);
}

static phys_addr_t omap_iommu_iova_to_phys(struct iommu_domain *domain,
					  dma_addr_t da)
{
	struct omap_iommu_domain *omap_domain = domain->priv;
	struct omap_iommu_device *iommu = omap_domain->iommus;
	struct omap_iommu *oiommu = iommu->iommu_dev;
	struct device *dev = oiommu->dev;
	u32 *pgd, *pte;
	phys_addr_t ret = 0;

	/*
	 * all the iommus within the domain will have identical programming,
	 * so perform the lookup using just the first iommu
	 */
	iopgtable_lookup_entry(oiommu, da, &pgd, &pte);

	if (pte) {
		if (iopte_is_small(*pte))
			ret = omap_iommu_translate(*pte, da, IOPTE_MASK);
		else if (iopte_is_large(*pte))
			ret = omap_iommu_translate(*pte, da, IOLARGE_MASK);
		else
			dev_err(dev, "bogus pte 0x%x, da 0x%llx", *pte,
							(unsigned long long)da);
	} else {
		if (iopgd_is_section(*pgd))
			ret = omap_iommu_translate(*pgd, da, IOSECTION_MASK);
		else if (iopgd_is_super(*pgd))
			ret = omap_iommu_translate(*pgd, da, IOSUPER_MASK);
		else
			dev_err(dev, "bogus pgd 0x%x, da 0x%llx", *pgd,
							(unsigned long long)da);
	}

	return ret;
}

static int omap_iommu_add_device(struct device *dev)
{
	struct omap_iommu_arch_data *arch_data, *tmp;
	struct device_node *np;
	struct platform_device *pdev;
	int num_iommus, i;
	int ret;

	/*
	 * Allocate the archdata iommu structure for DT-based devices.
	 *
	 * TODO: Simplify this when removing non-DT support completely from the
	 * IOMMU users.
	 */
	if (!dev->of_node)
		return 0;

	/*
	 * the cell-size for IOMMU nodes is 0, so retrieve the count
	 * using NULL as cells-name.
	 *
	 * XXX: Use the much simpler of_property_count_elems_of_size,
	 * if available
	 */
	num_iommus = of_count_phandle_with_args(dev->of_node, "iommus", NULL);
	if (num_iommus < 0)
		return 0;

	arch_data = kzalloc((num_iommus + 1) * sizeof(*arch_data), GFP_KERNEL);
	if (!arch_data)
		return -ENOMEM;

	for (i = 0, tmp = arch_data; i < num_iommus; i++, tmp++) {
		np = of_parse_phandle(dev->of_node, "iommus", i);
		if (!np) {
			ret = 0;
			goto err;
		}

		pdev = of_find_device_by_node(np);
		if (WARN_ON(!pdev)) {
			of_node_put(np);
			ret = -EINVAL;
			goto err;
		}

		tmp->name = kstrdup(dev_name(&pdev->dev), GFP_KERNEL);
		of_node_put(np);
	}

	dev->archdata.iommu = arch_data;
	return 0;

err:
	while (i--) {
		tmp--;
		kfree(tmp->name);
	}
	kfree(arch_data);
	return ret;
}

static void omap_iommu_remove_device(struct device *dev)
{
	struct omap_iommu_arch_data *arch_data = dev->archdata.iommu;
	struct omap_iommu_arch_data *tmp;

	if (!dev->of_node || !arch_data)
		return;

	tmp = arch_data;
	while (tmp->name) {
		kfree(tmp->name);
		tmp++;
	}
	kfree(arch_data);
}

static struct iommu_ops omap_iommu_ops = {
	.domain_init	= omap_iommu_domain_init,
	.domain_destroy	= omap_iommu_domain_destroy,
	.attach_dev	= omap_iommu_attach_dev,
	.detach_dev	= omap_iommu_detach_dev,
	.map		= omap_iommu_map,
	.unmap		= omap_iommu_unmap,
	.iova_to_phys	= omap_iommu_iova_to_phys,
	.add_device	= omap_iommu_add_device,
	.remove_device	= omap_iommu_remove_device,
	.pgsize_bitmap	= OMAP_IOMMU_PGSIZES,
};

static int __init omap_iommu_init(void)
{
	struct kmem_cache *p;
	const unsigned long flags = SLAB_HWCACHE_ALIGN;
	size_t align = 1 << 10; /* L2 pagetable alignement */

	p = kmem_cache_create("iopte_cache", IOPTE_TABLE_SIZE, align, flags,
			      iopte_cachep_ctor);
	if (!p)
		return -ENOMEM;
	iopte_cachep = p;

	bus_set_iommu(&platform_bus_type, &omap_iommu_ops);

	return platform_driver_register(&omap_iommu_driver);
}
/* must be ready before omap3isp is probed */
subsys_initcall(omap_iommu_init);

static void __exit omap_iommu_exit(void)
{
	kmem_cache_destroy(iopte_cachep);

	platform_driver_unregister(&omap_iommu_driver);
}
module_exit(omap_iommu_exit);

MODULE_DESCRIPTION("omap iommu: tlb and pagetable primitives");
MODULE_ALIAS("platform:omap-iommu");
MODULE_AUTHOR("Hiroshi DOYU, Paul Mundt and Toshihiro Kobayashi");
MODULE_LICENSE("GPL v2");
