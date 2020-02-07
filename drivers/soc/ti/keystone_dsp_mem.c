// SPDX-License-Identifier: GPL-2.0-only
/*
 * TI Keystone DSP Memory Mapping Driver
 *
 * Copyright (C) 2015-2020 Texas Instruments Incorporated - http://www.ti.com/
 */

#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <uapi/linux/keystone_dsp_mem.h>

#define KEYSTONE_ALIAS_PHYS_START	0x80000000ULL
#define KEYSTONE_ALIAS_PHYS_SIZE	0x80000000ULL /* 2G */

#define KEYSTONE_HIGH_PHYS_START	0x800000000ULL
#define KEYSTONE_HIGH_PHYS_LIMIT	(KEYSTONE_HIGH_PHYS_START + \
					 KEYSTONE_ALIAS_PHYS_SIZE)

#define to_alias_addr(addr)	(((addr) - KEYSTONE_HIGH_PHYS_START) + \
				 KEYSTONE_ALIAS_PHYS_START)

/**
 * struct keystone_dsp_mem - internal memory structure
 * @addr: physical address on the bus to access the memory region
 * @size: size of the memory region
 * @kobj: kobject for the sysfs directory file
 */
struct keystone_dsp_mem {
	phys_addr_t addr;
	resource_size_t size;
	struct kobject kobj;
};

#define to_dsp_mem(obj) container_of(obj, struct keystone_dsp_mem, kobj)

/**
 * struct keystone_dsp_mem_info - Keystone DSP Memory device structure
 * @misc: child miscdevice structure
 * @mem: memory region array pointer
 * @num_maps: number of memory regions
 */
struct keystone_dsp_mem_info {
	struct miscdevice misc;
	struct keystone_dsp_mem *mem;
	int num_maps;
};

static struct keystone_dsp_mem_info *dsp_mem;

#define to_dsp_mem_info(m) container_of(m, struct keystone_dsp_mem_info, misc)

static ssize_t mem_addr_show(struct keystone_dsp_mem *mem, char *buf)
{
	return sprintf(buf, "%pa\n", &mem->addr);
}

static ssize_t mem_size_show(struct keystone_dsp_mem *mem, char *buf)
{
	return sprintf(buf, "%pa\n", &mem->size);
}

struct mem_sysfs_entry {
	struct attribute attr;
	ssize_t (*show)(struct keystone_dsp_mem *mem, char *buf);
	ssize_t (*store)(struct keystone_dsp_mem *mem, const char *buf,
			 size_t len);
};

static struct mem_sysfs_entry addr_attribute =
	__ATTR(addr, 0444, mem_addr_show, NULL);
static struct mem_sysfs_entry size_attribute =
	__ATTR(size, 0444, mem_size_show, NULL);

static struct attribute *attrs[] = {
	&addr_attribute.attr,
	&size_attribute.attr,
	NULL,	/* sentinel */
};

static ssize_t mem_type_show(struct kobject *kobj, struct attribute *attr,
			     char *buf)
{
	struct keystone_dsp_mem *mem = to_dsp_mem(kobj);
	struct mem_sysfs_entry *entry;

	entry = container_of(attr, struct mem_sysfs_entry, attr);
	if (!entry->show)
		return -EIO;

	return entry->show(mem, buf);
}

static const struct sysfs_ops mem_sysfs_ops = {
	.show = mem_type_show,
};

static struct kobj_type mem_attr_type = {
	.sysfs_ops	= &mem_sysfs_ops,
	.default_attrs	= attrs,
};

static int keystone_dsp_mem_add_attrs(struct keystone_dsp_mem_info *dsp_mem)
{
	int i, ret;
	struct keystone_dsp_mem *mem;
	struct kobject *kobj_parent = &dsp_mem->misc.this_device->kobj;

	for (i = 0; i < dsp_mem->num_maps; i++) {
		mem = &dsp_mem->mem[i];
		kobject_init(&mem->kobj, &mem_attr_type);
		ret = kobject_add(&mem->kobj, kobj_parent, "memory%d", i);
		if (ret)
			goto err_kobj;
		ret = kobject_uevent(&mem->kobj, KOBJ_ADD);
		if (ret)
			goto err_kobj;
	}

	return 0;

err_kobj:
	for (; i >= 0; i--) {
		mem = &dsp_mem->mem[i];
		kobject_put(&mem->kobj);
	}
	return ret;
}

static void keystone_dsp_mem_del_attrs(struct keystone_dsp_mem_info *dsp_mem)
{
	int i;
	struct keystone_dsp_mem *mem;

	for (i = 0; i < dsp_mem->num_maps; i++) {
		mem = &dsp_mem->mem[i];
		kobject_put(&mem->kobj);
	}
}

static int keystone_dsp_mem_check_addr(struct keystone_dsp_mem_info *dsp_mem,
				       int mask, size_t size)
{
	size_t req_offset;
	u32 index;

	index = mask & KEYSTONE_DSP_MEM_MAP_INDEX_MASK;
	if (index >= dsp_mem->num_maps) {
		pr_err("%s: invalid mmap region index %d\n", __func__, index);
		return -EINVAL;
	}

	req_offset = (mask - index) << PAGE_SHIFT;
	if (req_offset + size < req_offset) {
		pr_err("%s: invalid request - overflow, mmap offset = 0x%zx size 0x%zx region %d\n",
		       __func__, req_offset, size, index);
		return -EINVAL;
	}

	if ((req_offset + size) > dsp_mem->mem[index].size) {
		pr_err("%s: invalid request - out of range, mmap offset 0x%zx size 0x%zx region %d\n",
		       __func__, req_offset, size, index);
		return -EINVAL;
	}

	return index;
}

/*
 * This is a custom mmap function following semantics based on the UIO
 * mmap implementation. The vm_pgoff passed in the vma structure is a
 * combination of the memory region index and the actual page offset in
 * that region. This checks if user request is in valid range before
 * providing mmap access.
 *
 * XXX: Evaluate this approach, as the MSMC memory can be mapped in whole
 * into userspace as it is not super-large, and the allowable kernel
 * unmapped DDR memory can be mmaped using traditional mmap semantics.
 */
static int keystone_dsp_mem_mmap(struct file *file, struct vm_area_struct *vma)
{
	size_t size = vma->vm_end - vma->vm_start;
	struct miscdevice *misc = file->private_data;
	struct keystone_dsp_mem_info *dsp_mem = to_dsp_mem_info(misc);
	int index;

	index = keystone_dsp_mem_check_addr(dsp_mem, vma->vm_pgoff, size);
	if (index < 0)
		return index;

	vma->vm_page_prot =
		phys_mem_access_prot(file,
				     (dsp_mem->mem[index].addr >> PAGE_SHIFT) +
				     (vma->vm_pgoff - index), size,
				     vma->vm_page_prot);

	if (remap_pfn_range(vma, vma->vm_start,
			    (dsp_mem->mem[index].addr >> PAGE_SHIFT) +
			    (vma->vm_pgoff - index), size, vma->vm_page_prot))
		return -EAGAIN;

	return 0;
}

static const struct file_operations keystone_dsp_mem_fops = {
	.owner	= THIS_MODULE,
	.mmap	= keystone_dsp_mem_mmap,
};

static int keystone_dsp_mem_parse(struct device_node *np, int index)
{
	phys_addr_t start, end, addr, size;
	struct resource res;
	resource_size_t rsize;
	int ret, j;

	if (!of_find_property(np, "no-map", NULL)) {
		pr_err("dsp reserved memory regions without no-map are not supported\n");
		return -EINVAL;
	}

	ret = of_address_to_resource(np, 0, &res);
	if (ret)
		return ret;

	/* make sure only aliased addresses are covered */
	rsize = resource_size(&res);
	start = res.start;
	end = res.start + rsize;
	if (start < KEYSTONE_HIGH_PHYS_START ||
	    start >= KEYSTONE_HIGH_PHYS_LIMIT ||
	    end > KEYSTONE_HIGH_PHYS_LIMIT) {
		pr_err("invalid address/size for keystone dsp memory carveout: %pa of size %pa\n",
		       &start, &rsize);
		return -EINVAL;
	}

	/* check for overlaps */
	start = to_alias_addr(start);
	end = to_alias_addr(end);
	for (j = 0; j < index; j++) {
		addr = dsp_mem->mem[j].addr;
		size = dsp_mem->mem[j].size;
		if ((end > addr && end <= addr + size) ||
		    (start >= addr && start < addr + size) ||
		    (start < addr && end > addr + size)) {
			pr_err("dsp memory carveout (%pa of size %pa) overlaps with (%pa of size %pa)\n",
			       &start, &rsize, &addr, &size);
			return -EINVAL;
		}
	}

	dsp_mem->mem[index].addr = to_alias_addr(res.start);
	dsp_mem->mem[index].size = resource_size(&res);

	return 0;
}

static int keystone_dsp_mem_init(void)
{
	struct miscdevice *misc;
	struct resource res;
	struct device_node *rmem_np, *sram_np, *np;
	int ret, i = 0;
	int num_maps = 0, num_sram = 0;

	if (!of_have_populated_dt())
		return -ENOTSUPP;

	/* module is supported only on TI Keystone SoCs */
	if (!of_machine_is_compatible("ti,keystone"))
		return -ENOTSUPP;

	/* count the number of DDR regions */
	rmem_np = of_find_node_by_path("/reserved-memory");
	if (rmem_np) {
		for_each_available_child_of_node(rmem_np, np) {
			if (of_device_is_compatible(np,
						    "ti,keystone-dsp-mem-pool"))
				num_maps++;
		}
	}

	for_each_compatible_node(sram_np, NULL, "ti,keystone-dsp-msm-ram") {
		if (!of_device_is_available(sram_np))
			continue;
		num_sram++;
	}

	if ((!num_maps && !num_sram) ||
	    (num_maps + num_sram > KEYSTONE_DSP_MEM_MAP_INDEX_MASK)) {
		ret = -EINVAL;
		goto put_rmem;
	}

	dsp_mem = kzalloc(sizeof(*dsp_mem), GFP_KERNEL);
	if (!dsp_mem) {
		ret = -ENOMEM;
		goto put_rmem;
	}

	dsp_mem->mem = kcalloc(num_maps + num_sram, sizeof(*dsp_mem->mem),
			       GFP_KERNEL);
	if (!dsp_mem->mem) {
		ret = -ENOMEM;
		goto free_dsp;
	}

	/* handle reserved-memory carveouts */
	if (num_maps) {
		for_each_available_child_of_node(rmem_np, np) {
			if (!of_device_is_compatible(np,
						"ti,keystone-dsp-mem-pool"))
				continue;

			ret = keystone_dsp_mem_parse(np, i);
			if (ret) {
				of_node_put(np);
				goto free_mem;
			}
			i++;
			dsp_mem->num_maps++;
		}
	}

	/* handle on-chip SRAM reserved regions */
	if (num_sram) {
		for_each_compatible_node(sram_np, NULL,
					 "ti,keystone-dsp-msm-ram") {
			if (!of_device_is_available(sram_np))
				continue;

			ret = of_address_to_resource(sram_np, 0, &res);
			if (ret) {
				ret = -EINVAL;
				of_node_put(sram_np);
				goto free_mem;
			}
			dsp_mem->mem[i].addr = res.start;
			dsp_mem->mem[i].size = resource_size(&res);
			i++;
			dsp_mem->num_maps++;
		}
	}

	misc = &dsp_mem->misc;
	misc->minor = MISC_DYNAMIC_MINOR;
	misc->name = "dspmem";
	misc->fops = &keystone_dsp_mem_fops;
	misc->parent = NULL;
	ret = misc_register(misc);
	if (ret) {
		pr_err("%s: could not register dspmem misc device\n", __func__);
		goto free_mem;
	}

	ret = keystone_dsp_mem_add_attrs(dsp_mem);
	if (ret) {
		pr_err("%s: error creating sysfs files (%d)\n", __func__, ret);
		goto unregister_misc;
	}
	of_node_put(rmem_np);

	pr_info("registered dspmem misc device\n");

	return 0;

unregister_misc:
	misc_deregister(&dsp_mem->misc);
free_mem:
	kfree(dsp_mem->mem);
free_dsp:
	kfree(dsp_mem);
	dsp_mem = NULL;
put_rmem:
	of_node_put(rmem_np);
	return ret;
}

static void keystone_dsp_mem_exit(void)
{
	keystone_dsp_mem_del_attrs(dsp_mem);

	misc_deregister(&dsp_mem->misc);

	kfree(dsp_mem->mem);
	kfree(dsp_mem);
	dsp_mem = NULL;
}

module_init(keystone_dsp_mem_init);
module_exit(keystone_dsp_mem_exit);

MODULE_AUTHOR("Suman Anna <s-anna@ti.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("TI Keystone DSP Memory Mapping Driver");
