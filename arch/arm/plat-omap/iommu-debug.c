/*
 * omap iommu: debugfs interface
 *
 * Copyright (C) 2008-2009 Nokia Corporation
 *
 * Written by Hiroshi DOYU <Hiroshi.DOYU@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>

#include <mach/iommu.h>
#include <mach/iovmm.h>

#include "iopgtable.h"

#define MAXCOLUMN 100 /* for short messages */

static DEFINE_MUTEX(iommu_debug_lock);
static char local_buffer[SZ_4K];

static struct dentry *iommu_debug_root;

static ssize_t debug_read_ver(struct file *file, char __user *userbuf,
			      size_t count, loff_t *ppos)
{
	u32 ver = iommu_arch_version();
	char buf[MAXCOLUMN], *p = buf;

	p += sprintf(p, "H/W version: %d.%d\n", (ver >> 4) & 0xf , ver & 0xf);

	return simple_read_from_buffer(userbuf, count, ppos, buf, p - buf);
}

static ssize_t debug_read_regs(struct file *file, char __user *userbuf,
			       size_t count, loff_t *ppos)
{
	struct iommu *obj = file->private_data;
	char *p = local_buffer;
	ssize_t bytes;

	mutex_lock(&iommu_debug_lock);
	p += iommu_dump_ctx(obj, p);
	bytes = simple_read_from_buffer(userbuf, count, ppos, local_buffer,
					p - local_buffer);
	mutex_unlock(&iommu_debug_lock);
	return bytes;
}

static ssize_t debug_read_tlb(struct file *file, char __user *userbuf,
			      size_t count, loff_t *ppos)
{
	struct iommu *obj = file->private_data;
	char *p = local_buffer;
	ssize_t bytes;

	mutex_lock(&iommu_debug_lock);
	p += sprintf(p, "%8s %8s\n", "cam:", "ram:");
	p += sprintf(p, "-----------------------------------------\n");
	p += dump_tlb_entries(obj, p);
	bytes = simple_read_from_buffer(userbuf, count, ppos, local_buffer,
					p - local_buffer);
	mutex_unlock(&iommu_debug_lock);
	return bytes;
}

static ssize_t debug_write_pagetable(struct file *file,
		     const char __user *userbuf, size_t count, loff_t *ppos)
{
	struct iotlb_entry e;
	struct cr_regs cr;
	int err;
	struct iommu *obj = file->private_data;
	char buf[MAXCOLUMN], *p = buf;

	count = min(count, sizeof(buf));

	mutex_lock(&iommu_debug_lock);
	if (copy_from_user(p, userbuf, count)) {
		mutex_unlock(&iommu_debug_lock);
		return -EFAULT;
	}

	sscanf(p, "%x %x", &cr.cam, &cr.ram);
	if (!cr.cam || !cr.ram) {
		mutex_unlock(&iommu_debug_lock);
		return -EINVAL;
	}

	iotlb_cr_to_e(&cr, &e);
	err = iopgtable_store_entry(obj, &e);
	if (err)
		dev_err(obj->dev, "%s: fail to store cr\n", __func__);

	mutex_unlock(&iommu_debug_lock);
	return count;
}

static ssize_t debug_read_pagetable(struct file *file, char __user *userbuf,
				    size_t count, loff_t *ppos)
{
	int i;
	u32 *iopgd;
	struct iommu *obj = file->private_data;
	char *p = local_buffer;
	ssize_t bytes;

	mutex_lock(&iommu_debug_lock);

	p += sprintf(p, "L: %8s %8s\n", "da:", "pa:");
	p += sprintf(p, "-----------------------------------------\n");

	spin_lock(&obj->page_table_lock);

	iopgd = iopgd_offset(obj, 0);
	for (i = 0; i < PTRS_PER_IOPGD; i++, iopgd++) {
		int j;
		u32 *iopte;

		if (!*iopgd)
			continue;

		if (!(*iopgd & IOPGD_TABLE)) {
			u32 da;

			da = i << IOPGD_SHIFT;
			p += sprintf(p, "1: %08x %08x\n", da, *iopgd);
			continue;
		}

		iopte = iopte_offset(iopgd, 0);

		for (j = 0; j < PTRS_PER_IOPTE; j++, iopte++) {
			u32 da;

			if (!*iopte)
				continue;

			da = (i << IOPGD_SHIFT) + (j << IOPTE_SHIFT);
			p += sprintf(p, "2: %08x %08x\n", da, *iopte);
		}
	}
	spin_unlock(&obj->page_table_lock);

	bytes = simple_read_from_buffer(userbuf, count, ppos, local_buffer,
					p - local_buffer);
	mutex_unlock(&iommu_debug_lock);
	return bytes;
}

static ssize_t debug_read_mmap(struct file *file, char __user *userbuf,
			       size_t count, loff_t *ppos)
{
	struct iommu *obj = file->private_data;
	char *p = local_buffer;
	struct iovm_struct *tmp;
	int uninitialized_var(i);
	ssize_t bytes;

	mutex_lock(&iommu_debug_lock);

	p += sprintf(p, "%-3s %-8s %-8s %6s %8s\n",
		     "No", "start", "end", "size", "flags");
	p += sprintf(p, "-------------------------------------------------\n");

	list_for_each_entry(tmp, &obj->mmap, list) {
		size_t len;

		len = tmp->da_end - tmp->da_start;
		p += sprintf(p, "%3d %08x-%08x %6x %8x\n",
			     i, tmp->da_start, tmp->da_end, len, tmp->flags);
		i++;
	}
	bytes = simple_read_from_buffer(userbuf, count, ppos, local_buffer,
					p - local_buffer);
	mutex_unlock(&iommu_debug_lock);
	return bytes;
}

static ssize_t debug_read_mem(struct file *file, char __user *userbuf,
			      size_t count, loff_t *ppos)
{
	struct iommu *obj = file->private_data;
	char *p = local_buffer;
	struct iovm_struct *area;
	ssize_t bytes;

	mutex_lock(&iommu_debug_lock);

	area = find_iovm_area(obj, (u32)ppos);
	if (IS_ERR(area)) {
		mutex_unlock(&iommu_debug_lock);
		return -EINVAL;
	}
	memcpy(p, area->va, count);
	p += count;

	bytes = simple_read_from_buffer(userbuf, count, ppos, local_buffer,
					p - local_buffer);
	mutex_unlock(&iommu_debug_lock);
	return bytes;
}

static ssize_t debug_write_mem(struct file *file, const char __user *userbuf,
			       size_t count, loff_t *ppos)
{
	struct iommu *obj = file->private_data;
	struct iovm_struct *area;
	char *p = local_buffer;

	count = min(count, sizeof(local_buffer));

	mutex_lock(&iommu_debug_lock);

	if (copy_from_user(p, userbuf, count)) {
		mutex_unlock(&iommu_debug_lock);
		return -EFAULT;
	}

	area = find_iovm_area(obj, (u32)ppos);
	if (IS_ERR(area)) {
		mutex_unlock(&iommu_debug_lock);
		return -EINVAL;
	}
	memcpy(area->va, p, count);
	mutex_unlock(&iommu_debug_lock);
	return count;
}

static int debug_open_generic(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

#define DEBUG_FOPS(name)						\
	static const struct file_operations debug_##name##_fops = {	\
		.open = debug_open_generic,				\
		.read = debug_read_##name,				\
		.write = debug_write_##name,				\
	};

#define DEBUG_FOPS_RO(name)						\
	static const struct file_operations debug_##name##_fops = {	\
		.open = debug_open_generic,				\
		.read = debug_read_##name,				\
	};

DEBUG_FOPS_RO(ver);
DEBUG_FOPS_RO(regs);
DEBUG_FOPS_RO(tlb);
DEBUG_FOPS(pagetable);
DEBUG_FOPS_RO(mmap);
DEBUG_FOPS(mem);

#define __DEBUG_ADD_FILE(attr, mode)					\
	{								\
		struct dentry *dent;					\
		dent = debugfs_create_file(#attr, mode, parent,		\
					   obj, &debug_##attr##_fops);	\
		if (!dent)						\
			return -ENOMEM;					\
	}

#define DEBUG_ADD_FILE(name) __DEBUG_ADD_FILE(name, 600)
#define DEBUG_ADD_FILE_RO(name) __DEBUG_ADD_FILE(name, 400)

static int iommu_debug_register(struct device *dev, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct iommu *obj = platform_get_drvdata(pdev);
	struct dentry *d, *parent;

	if (!obj || !obj->dev)
		return -EINVAL;

	d = debugfs_create_dir(obj->name, iommu_debug_root);
	if (!d)
		return -ENOMEM;
	parent = d;

	d = debugfs_create_u8("nr_tlb_entries", 400, parent,
			      (u8 *)&obj->nr_tlb_entries);
	if (!d)
		return -ENOMEM;

	DEBUG_ADD_FILE_RO(ver);
	DEBUG_ADD_FILE_RO(regs);
	DEBUG_ADD_FILE_RO(tlb);
	DEBUG_ADD_FILE(pagetable);
	DEBUG_ADD_FILE_RO(mmap);
	DEBUG_ADD_FILE(mem);

	return 0;
}

static int __init iommu_debug_init(void)
{
	struct dentry *d;
	int err;

	d = debugfs_create_dir("iommu", NULL);
	if (!d)
		return -ENOMEM;
	iommu_debug_root = d;

	err = foreach_iommu_device(d, iommu_debug_register);
	if (err)
		goto err_out;
	return 0;

err_out:
	debugfs_remove_recursive(iommu_debug_root);
	return err;
}
module_init(iommu_debug_init)

static void __exit iommu_debugfs_exit(void)
{
	debugfs_remove_recursive(iommu_debug_root);
}
module_exit(iommu_debugfs_exit)

MODULE_DESCRIPTION("omap iommu: debugfs interface");
MODULE_AUTHOR("Hiroshi DOYU <Hiroshi.DOYU@nokia.com>");
MODULE_LICENSE("GPL v2");
