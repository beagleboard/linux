// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2004 Benjamin Herrenschmidt, IBM Corp.
 *                    <benh@kernel.crashing.org>
 * Copyright (C) 2012 ARM Limited
 * Copyright (C) 2015 Regents of the University of California
 */

#include <linux/elf.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/binfmts.h>
#include <linux/err.h>
#include <asm/page.h>
#ifdef CONFIG_GENERIC_TIME_VSYSCALL
#include <vdso/datapage.h>
#else
#include <asm/vdso.h>
#endif

extern char vdso_start[], vdso_end[];

static unsigned int vdso_pages;
static struct page **vdso_pagelist;

/*
 * The vDSO data page.
 */
static union {
	struct vdso_data	data;
	u8			page[PAGE_SIZE];
} vdso_data_store __page_aligned_data;
struct vdso_data *vdso_data = &vdso_data_store.data;

static int __init __vdso_init(void)
{
	unsigned int i;

	vdso_pages = (vdso_end - vdso_start) >> PAGE_SHIFT;
	vdso_pagelist =
		kcalloc(vdso_pages + 1, sizeof(struct page *), GFP_KERNEL);
	if (unlikely(vdso_pagelist == NULL)) {
		pr_err("vdso: pagelist allocation failed\n");
		return -ENOMEM;
	}

	for (i = 0; i < vdso_pages; i++) {
		struct page *pg;

		pg = virt_to_page(vdso_start + (i << PAGE_SHIFT));
		vdso_pagelist[i] = pg;
	}
	vdso_pagelist[i] = virt_to_page(vdso_data);

	return 0;
}

#ifdef CONFIG_COMPAT
extern char compat_vdso_start[], compat_vdso_end[];

static unsigned int compat_vdso_pages;
static struct page **compat_vdso_pagelist;

/*
 * The compat vDSO data page.
 */
static union {
	struct vdso_data	data;
	u8			page[PAGE_SIZE];
} compat_vdso_data_store __page_aligned_data;
struct vdso_data *compat_vdso_data = &compat_vdso_data_store.data;

static int __init __compat_vdso_init(void)
{
	unsigned int i;

	compat_vdso_pages = (compat_vdso_end - compat_vdso_start) >> PAGE_SHIFT;
	compat_vdso_pagelist =
		kcalloc(compat_vdso_pages + 1, sizeof(struct page *), GFP_KERNEL);
	if (unlikely(compat_vdso_pagelist == NULL)) {
		pr_err("compat vdso: pagelist allocation failed\n");
		return -ENOMEM;
	}

	for (i = 0; i < compat_vdso_pages; i++) {
		struct page *pg;

		pg = virt_to_page(compat_vdso_start + (i << PAGE_SHIFT));
		compat_vdso_pagelist[i] = pg;
	}
	compat_vdso_pagelist[i] = virt_to_page(compat_vdso_data);

	return 0;
}
#endif

static int __init vdso_init(void)
{
	int ret = 0;

	ret = __vdso_init();
	if (ret)
		goto out;

#ifdef CONFIG_COMPAT
	ret = __compat_vdso_init();
#endif
out:
	return ret;
}
arch_initcall(vdso_init);

int arch_setup_additional_pages(struct linux_binprm *bprm,
	int uses_interp)
{
	struct mm_struct *mm = current->mm;
	unsigned long vdso_base, vdso_len;
	int ret;

	vdso_len = (vdso_pages + 1) << PAGE_SHIFT;

	if (mmap_write_lock_killable(mm))
		return -EINTR;

	vdso_base = get_unmapped_area(NULL, 0, vdso_len, 0, 0);
	if (IS_ERR_VALUE(vdso_base)) {
		ret = vdso_base;
		goto end;
	}

	/*
	 * Put vDSO base into mm struct. We need to do this before calling
	 * install_special_mapping or the perf counter mmap tracking code
	 * will fail to recognise it as a vDSO (since arch_vma_name fails).
	 */
	mm->context.vdso = (void *)vdso_base;

	ret =
	   install_special_mapping(mm, vdso_base, vdso_pages << PAGE_SHIFT,
		(VM_READ | VM_EXEC | VM_MAYREAD | VM_MAYWRITE | VM_MAYEXEC),
		vdso_pagelist);

	if (unlikely(ret)) {
		mm->context.vdso = NULL;
		goto end;
	}

	vdso_base += (vdso_pages << PAGE_SHIFT);
	ret = install_special_mapping(mm, vdso_base, PAGE_SIZE,
		(VM_READ | VM_MAYREAD), &vdso_pagelist[vdso_pages]);

	if (unlikely(ret))
		mm->context.vdso = NULL;
end:
	mmap_write_unlock(mm);
	return ret;
}

#ifdef CONFIG_COMPAT
int compat_arch_setup_additional_pages(struct linux_binprm *bprm,
				       int uses_interp)
{
	struct mm_struct *mm = current->mm;
	unsigned long vdso_base, vdso_len;
	int ret;

	vdso_len = (compat_vdso_pages + 1) << PAGE_SHIFT;

	mmap_write_lock(mm);
	vdso_base = get_unmapped_area(NULL, 0, vdso_len, 0, 0);
	if (IS_ERR_VALUE(vdso_base)) {
		ret = vdso_base;
		goto end;
	}

	/*
	 * Put vDSO base into mm struct. We need to do this before calling
	 * install_special_mapping or the perf counter mmap tracking code
	 * will fail to recognise it as a vDSO (since arch_vma_name fails).
	 */
	mm->context.vdso = (void *)vdso_base;

	ret =
	   install_special_mapping(mm, vdso_base, vdso_pages << PAGE_SHIFT,
		(VM_READ | VM_EXEC | VM_MAYREAD | VM_MAYWRITE | VM_MAYEXEC),
		compat_vdso_pagelist);

	if (unlikely(ret)) {
		mm->context.vdso = NULL;
		goto end;
	}

	vdso_base += (compat_vdso_pages << PAGE_SHIFT);
	ret = install_special_mapping(mm, vdso_base, PAGE_SIZE,
		(VM_READ | VM_MAYREAD), &compat_vdso_pagelist[compat_vdso_pages]);

	if (unlikely(ret))
		mm->context.vdso = NULL;
end:
	mmap_write_unlock(mm);
	return ret;
}
#endif

const char *arch_vma_name(struct vm_area_struct *vma)
{
	if (vma->vm_mm && (vma->vm_start == (long)vma->vm_mm->context.vdso))
		return "[vdso]";
	if (vma->vm_mm && (vma->vm_start ==
			   (long)vma->vm_mm->context.vdso + PAGE_SIZE))
		return "[vdso_data]";
	return NULL;
}
