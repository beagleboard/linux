/*
 * TI Keystone DSP Memory Mapping Driver
 *
 * Copyright (C) 2015-2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
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
 * @dev: device pointer
 * @misc: child miscdevice structure
 * @mem: memory region array pointer
 * @num_maps: number of memory regions
 */
struct keystone_dsp_mem_info {
	struct device *dev;
	struct miscdevice misc;
	struct keystone_dsp_mem *mem;
	int num_maps;
};

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
	ssize_t (*show)(struct keystone_dsp_mem *, char *);
	ssize_t (*store)(struct keystone_dsp_mem *, const char *, size_t);
};

static struct mem_sysfs_entry addr_attribute =
	__ATTR(addr, S_IRUGO, mem_addr_show, NULL);
static struct mem_sysfs_entry size_attribute =
	__ATTR(size, S_IRUGO, mem_size_show, NULL);

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
			goto err_add_kobj;
		ret = kobject_uevent(&mem->kobj, KOBJ_ADD);
		if (ret)
			goto err_event;
	}

	return 0;

err_event:
	i--;
err_add_kobj:
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
	struct device *dev = dsp_mem->dev;
	size_t req_offset;
	u32 index;

	index = mask & KEYSTONE_DSP_MEM_MAP_INDEX_MASK;
	if (index >= dsp_mem->num_maps) {
		dev_err(dev, "invalid mmap region index %d\n", index);
		return -EINVAL;
	}

	req_offset = (mask - index) << PAGE_SHIFT;
	if (req_offset + size < req_offset) {
		dev_err(dev, "invalid request - overflow, mmap offset = 0x%zx size 0x%zx region %d\n",
			req_offset, size, index);
		return -EINVAL;
	}

	if ((req_offset + size) > dsp_mem->mem[index].size) {
		dev_err(dev, "invalid request - out of range, mmap offset 0x%zx size 0x%zx region %d\n",
			req_offset, size, index);
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

	vma->vm_page_prot = phys_mem_access_prot(file,
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

static int keystone_dsp_mem_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct keystone_dsp_mem_info *dsp_mem;
	struct miscdevice *misc;
	struct resource *res, temp_res;
	char *name;
	int ret, i;
	int num_maps = 0;

	if (!np) {
		dev_err(dev, "only DT-based devices are supported\n");
		return -EINVAL;
	}

	dsp_mem = devm_kzalloc(dev, sizeof(*dsp_mem), GFP_KERNEL);
	if (!dsp_mem)
		return -ENOMEM;

	/*
	 * count the io resources agnostic of #address-cells and #size-cells.
	 * could use the platform device's num_resources directly, but let's
	 * not make any assumptions on the resources being all IOMEM resources.
	 */
	while (of_address_to_resource(np, num_maps, &temp_res) == 0)
		num_maps++;
	if (!num_maps || num_maps > KEYSTONE_DSP_MEM_MAP_INDEX_MASK)
		return -EINVAL;

	dsp_mem->mem = devm_kcalloc(dev, num_maps, sizeof(*dsp_mem->mem),
				    GFP_KERNEL);
	if (!dsp_mem->mem)
		return -ENOMEM;

	for (i = 0; i < num_maps; i++) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!res || resource_type(res) != IORESOURCE_MEM)
			return -EINVAL;
		dsp_mem->mem[i].addr = res->start;
		dsp_mem->mem[i].size = resource_size(res);
	}
	dsp_mem->num_maps = num_maps;
	dsp_mem->dev = dev;

	/*
	 * construct a user-friendly device name by discarding any prefixes
	 * from the DT-generated device name.
	 */
	name = strchr(dev_name(dev), '.');
	if (!name)
		name = strchr(dev_name(dev), ':');
	name = name ? name + 1 : (char *)dev_name(dev);

	misc = &dsp_mem->misc;
	misc->minor = MISC_DYNAMIC_MINOR;
	misc->name = name;
	misc->fops = &keystone_dsp_mem_fops;
	misc->parent = dev;
	ret = misc_register(misc);
	if (ret) {
		dev_err(dev, "could not register misc device %s\n", name);
		return ret;
	}

	platform_set_drvdata(pdev, dsp_mem);

	ret = keystone_dsp_mem_add_attrs(dsp_mem);
	if (ret) {
		dev_err(dsp_mem->dev, "error creating sysfs files (%d)\n", ret);
		misc_deregister(&dsp_mem->misc);
		return ret;
	}

	dev_info(dev, "registered misc device %s\n", name);

	return 0;
}

static int keystone_dsp_mem_remove(struct platform_device *pdev)
{
	struct keystone_dsp_mem_info *dsp_mem = platform_get_drvdata(pdev);

	keystone_dsp_mem_del_attrs(dsp_mem);

	misc_deregister(&dsp_mem->misc);

	return 0;
}

static const struct of_device_id keystone_dsp_mem_of_match[] = {
	{ .compatible = "ti,keystone-dsp-mem", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, keystone_dsp_mem_of_match);

static struct platform_driver keystone_dsp_mem_driver = {
	.probe	= keystone_dsp_mem_probe,
	.remove	= keystone_dsp_mem_remove,
	.driver	= {
		.name = "keystone-dsp-mem",
		.of_match_table = keystone_dsp_mem_of_match,
	},
};

module_platform_driver(keystone_dsp_mem_driver);
MODULE_AUTHOR("Suman Anna");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("TI Keystone DSP Memory Mapping Driver");
MODULE_ALIAS("platform:keystone-dsp-mem");
