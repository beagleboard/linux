/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2021 Alibaba Group Holding Limited.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/regulator/consumer.h>
#include <linux/of_platform.h>
#include <linux/err.h>
#include <dt-bindings/soc/thead,light-iopmp.h>

#define LIGHT_PMP_MISC_CTRL		0x04
#define LIGHT_PMP_DUMMY_ADDR		0x08
#define LIGHT_PMP_PAGE_LOCK0		0x40
#define LIGHT_PMP_ATTR_CFG0		0x80
#define LIGHT_DEFAULT_PMP_ATTR_CFG	0xc0
#define LIGHT_PMP_REGION0_SADDR		0x280
#define LIGHT_PMP_REGION0_EADDR		0x284

#define LIGHT_PMP_PAGE_LOCK0_REGIONS		0xFFFF
#define LIGHT_PMP_PAGE_LOCK0_DEFAULT_CFG	BIT(16)
#define LIGHT_PMP_PAGE_LOCK0_BYPASS_EN		BIT(17)
#define LIGHT_PMP_PAGE_LOCK0_DUMMY_ADDR		BIT(18)

#define LIGHT_IOPMP_REGION_NUM		16
#define LIGHT_IOPMP_ATTR_CFG_DEFAULT	0xFFFFFFFF
#define LIGHT_IOPMP_DUMMY_ADDR		0x800000
#define LIGHT_IOPMP_CTRL_BYPASS		0xFF000000

struct light_iopmp_entry {
        u32 reg_start[LIGHT_IOPMP_REGION_NUM];
        u32 reg_end[LIGHT_IOPMP_REGION_NUM];
        u32 entry_valid_num;
};

struct light_iopmp_cur_sys_entry {
	int iopmp_type;
	u32 start_addr;
	u32 end_addr;
	u32 attr;
	int lock;
};

struct light_iopmp_info {
	struct device *dev;

	int entries;
	struct light_iopmp_list *iopmp_list;
	struct light_iopmp_cur_sys_entry cur_entry;
	struct light_iopmp_entry iopmp_entry[];
};

static struct light_iopmp_list {
	int iopmp_type;
	resource_size_t offset;
	void __iomem *base;
} light_iopmp_lists[] = {
	{IOPMP_EMMC, 0xFFFC028000, NULL},
	{IOPMP_SDIO0, 0xFFFC029000, NULL},
	{IOPMP_SDIO1, 0xFFFC02A000, NULL},
	{IOPMP_USB0, 0xFFFC02E000, NULL},
	{IOPMP_AO, 0xFFFFC21000, NULL},
	{IOPMP_AUD, 0xFFFFC22000, NULL},
	{IOPMP_CHIP_DBG, 0xFFFFC37000, NULL},
	{IOPMP_EIP120I, 0xFFFF220000, NULL},
	{IOPMP_EIP120II, 0xFFFF230000, NULL},
	{IOPMP_EIP120III, 0xFFFF240000, NULL},
	{IOPMP_ISP0, 0xFFF4080000, NULL},
	{IOPMP_ISP1, 0xFFF4081000, NULL},
	{IOPMP_DW200, 0xFFF4082000, NULL},
	{IOPMP_VIPRE, 0xFFF4083000, NULL},
	{IOPMP_VENC, 0xFFFCC60000, NULL},
	{IOPMP_VDEC, 0xFFFCC61000, NULL},
	{IOPMP_G2D, 0xFFFCC62000, NULL},
	{IOPMP_FCE, 0xFFFCC63000, NULL},
	{IOPMP_NPU, 0xFFFF01C000, NULL},
	{IOPMP0_DPU, 0xFFFF520000, NULL},
	{IOPMP1_DPU, 0xFFFF521000, NULL},
	{IOPMP_GPU, 0xFFFF522000, NULL},
	{IOPMP_GMAC1, 0xFFFC001000, NULL},
	{IOPMP_GMAC2, 0xFFFC002000, NULL},
	{IOPMP_DMAC, 0xFFFFC20000, NULL},
	{IOPMP_TEE_DMAC, 0xFFFF250000, NULL},
	{IOPMP_DSP0, 0xFFFF058000, NULL},
	{IOPMP_DSP1, 0xFFFF059000, NULL},
};

static const struct light_iopmp_driver_data {
	struct light_iopmp_list *iopmp_list;
} light_iopmp_data = {
	.iopmp_list = light_iopmp_lists,
};

static int light_iopmp_get_entries(struct light_iopmp_list *match)
{
	if (match == light_iopmp_lists)
		return ARRAY_SIZE(light_iopmp_lists);

	return 0;
}

static int light_iopmp_get_type(struct light_iopmp_info *info, struct device_node *entry_np)
{
	int entry_size = info->entries;
	struct light_iopmp_list *match = info->iopmp_list;
	unsigned long type;

	if (kstrtoul(entry_np->name, 0, &type))
		return -EINVAL;

	if (type < match[0].iopmp_type || type > match[entry_size - 1].iopmp_type)
		return -EINVAL;

	return type;
}

static bool light_iopmp_type_valid(struct light_iopmp_info *info, int tap)
{
	int entry_size = info->entries;
	struct light_iopmp_list *match = info->iopmp_list;

	if (tap < match[0].iopmp_type || tap > match[entry_size - 1].iopmp_type)
		return false;

	return true;
}

static void light_iopmp_set_attr_bypass(struct light_iopmp_info *info, int type)
{
	void __iomem *regs;
	u32 val;

	if (!info->iopmp_list[type].base) {
		regs = devm_ioremap(info->dev, info->iopmp_list[type].offset, PAGE_SIZE);
		if (!regs) {
			dev_err(info->dev, "ioremap failed\n");
			return;
		}
		info->iopmp_list[type].base = regs;
	}

	val = readl(info->iopmp_list[type].base + LIGHT_PMP_PAGE_LOCK0);
	if (val & LIGHT_PMP_PAGE_LOCK0_BYPASS_EN) {
		dev_info(info->dev, "iopmp bypass_en is already locked, ignore the tap[%d] setting\n", type);
		return;
	}

	val = readl(info->iopmp_list[type].base + LIGHT_PMP_MISC_CTRL);
	val |= LIGHT_IOPMP_CTRL_BYPASS;
	writel(val, info->iopmp_list[type].base + LIGHT_PMP_MISC_CTRL);

	/* lock the iopmp */
	val = readl(info->iopmp_list[type].base + LIGHT_PMP_PAGE_LOCK0);
	val |= LIGHT_PMP_PAGE_LOCK0_BYPASS_EN;
	writel(val, info->iopmp_list[type].base + LIGHT_PMP_PAGE_LOCK0);
}

static void light_iopmp_set_attr_default(struct light_iopmp_info *info, int type, int attr)
{
	void __iomem *regs;
	u32 val;

	if (!info->iopmp_list[type].base) {
		regs = devm_ioremap(info->dev, info->iopmp_list[type].offset, PAGE_SIZE);
		if (!regs) {
			dev_err(info->dev, "ioremap failed\n");
			return;
		}
		info->iopmp_list[type].base = regs;
	}

	val = readl(info->iopmp_list[type].base + LIGHT_PMP_PAGE_LOCK0);
	if (val & LIGHT_PMP_PAGE_LOCK0_DEFAULT_CFG) {
		dev_info(info->dev, "iopmp default cfg is already locked, ignore the tap[%d] setting\n", type);
		return;
	}

	val = readl(info->iopmp_list[type].base + LIGHT_PMP_MISC_CTRL);
	if (val & LIGHT_IOPMP_CTRL_BYPASS) {
		dev_info(info->dev, "iopmp bypass is already enabled, ignore the tap[%d] setting\n", type);
		return;
	}

	writel(attr, info->iopmp_list[type].base + LIGHT_DEFAULT_PMP_ATTR_CFG);

	/* lock the iopmp */
	val = readl(info->iopmp_list[type].base + LIGHT_PMP_PAGE_LOCK0);
	val |= LIGHT_PMP_PAGE_LOCK0_DEFAULT_CFG;
	writel(val, info->iopmp_list[type].base + LIGHT_PMP_PAGE_LOCK0);
}

static void light_iopmp_set_attr(struct light_iopmp_info *info, int type, int attr,
				 int dummy_slave, struct light_iopmp_entry *entry)
{
	void __iomem *regs;
	bool dummy_slave_lock = false;
	u32 val;
	int i;

	if (!info->iopmp_list[type].base) {
		regs = devm_ioremap(info->dev, info->iopmp_list[type].offset, PAGE_SIZE);
		if (!regs) {
			dev_err(info->dev, "ioremap failed\n");
			return;
		}
		info->iopmp_list[type].base = regs;
	}

	val = readl(info->iopmp_list[type].base + LIGHT_PMP_PAGE_LOCK0);
	if (val & LIGHT_PMP_PAGE_LOCK0_REGIONS) {
		dev_info(info->dev, "iopmp regions cfg is already locked, ignore the tap[%d] setting\n", type);
		return;
	}

	if (val & LIGHT_PMP_PAGE_LOCK0_DUMMY_ADDR)
		dummy_slave_lock = true;

	val = readl(info->iopmp_list[type].base + LIGHT_PMP_MISC_CTRL);
	if (val & LIGHT_IOPMP_CTRL_BYPASS) {
		dev_info(info->dev, "iopmp bypass is already enabled, ignore the tap[%d] setting\n", type);
		return;
	}

	for (i = 0; i < entry->entry_valid_num; i++) {
		writel(entry->reg_start[i],
		       info->iopmp_list[type].base + LIGHT_PMP_REGION0_SADDR + (i << 3));
		writel(entry->reg_end[i],
		       info->iopmp_list[type].base + LIGHT_PMP_REGION0_EADDR + (i << 3));
		writel(attr, info->iopmp_list[type].base + LIGHT_PMP_ATTR_CFG0 + (i << 2));
	}

	if (!dummy_slave_lock)
		writel(dummy_slave, info->iopmp_list[type].base + LIGHT_PMP_DUMMY_ADDR);

	/* lock the iopmp */
	val = readl(info->iopmp_list[type].base + LIGHT_PMP_PAGE_LOCK0);
	val |= LIGHT_PMP_PAGE_LOCK0_REGIONS | LIGHT_PMP_PAGE_LOCK0_DUMMY_ADDR;
	writel(val, info->iopmp_list[type].base + LIGHT_PMP_PAGE_LOCK0);
}

static int light_iopmp_sys_set(struct light_iopmp_info *info)
{
	void __iomem *regs;
	int type = info->cur_entry.iopmp_type;
	bool dummy_slave_lock = false;
	u32 val;

	if (!info->iopmp_list[type].base) {
		regs = devm_ioremap(info->dev, info->iopmp_list[type].offset, PAGE_SIZE);
		if (!regs) {
			dev_err(info->dev, "ioremap failed\n");
			return -ENOMEM;
		}
		info->iopmp_list[type].base = regs;
	}

	val = readl(info->iopmp_list[type].base + LIGHT_PMP_MISC_CTRL);
	if (val & LIGHT_IOPMP_CTRL_BYPASS) {
		dev_err(info->dev, "iopmp bypass is already enabled, ignore the tap[%d] setting\n", type);
		return -EINVAL;
	}

	val = readl(info->iopmp_list[type].base + LIGHT_PMP_PAGE_LOCK0);
	if (val & LIGHT_PMP_PAGE_LOCK0_REGIONS) {
		dev_err(info->dev, "iopmp regions cfg is already locked, ignore the tap[%d] setting\n", type);
		return -EINVAL;
	}

	if (val & LIGHT_PMP_PAGE_LOCK0_DUMMY_ADDR)
		dummy_slave_lock = true;

	val = readl(info->iopmp_list[type].base + LIGHT_PMP_MISC_CTRL);
	if (val & LIGHT_IOPMP_CTRL_BYPASS) {
		dev_err(info->dev, "iopmp bypass is already enabled, ignore the tap[%d] setting\n", type);
		return -EINVAL;
	}

	/* set one region from userspace's config */
	writel(info->cur_entry.start_addr,
	       info->iopmp_list[type].base + LIGHT_PMP_REGION0_SADDR);
	writel(info->cur_entry.end_addr,
	       info->iopmp_list[type].base + LIGHT_PMP_REGION0_EADDR);
	writel(info->cur_entry.attr, info->iopmp_list[type].base + LIGHT_PMP_ATTR_CFG0);

	if (!dummy_slave_lock)
		writel(LIGHT_IOPMP_DUMMY_ADDR, info->iopmp_list[type].base + LIGHT_PMP_DUMMY_ADDR);

	/* lock the iopmp */
	if (info->cur_entry.lock) {
		val = LIGHT_PMP_PAGE_LOCK0_REGIONS | LIGHT_PMP_PAGE_LOCK0_DEFAULT_CFG |
		      LIGHT_PMP_PAGE_LOCK0_DUMMY_ADDR | LIGHT_PMP_PAGE_LOCK0_BYPASS_EN;
		writel(val, info->iopmp_list[type].base + LIGHT_PMP_PAGE_LOCK0);
	}

	return 0;
}

static ssize_t light_iopmp_tap_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct light_iopmp_info *info = platform_get_drvdata(pdev);
	ssize_t ret;

	ret = sprintf(buf, "%d\n", info->cur_entry.iopmp_type);
	return ret;
}

static ssize_t light_iopmp_start_addr_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct light_iopmp_info *info = platform_get_drvdata(pdev);
	ssize_t ret;

	ret = sprintf(buf, "0x%x\n", info->cur_entry.start_addr);
	return ret;
}

static ssize_t light_iopmp_end_addr_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct light_iopmp_info *info = platform_get_drvdata(pdev);
	ssize_t ret;

	ret = sprintf(buf, "0x%x\n", info->cur_entry.end_addr);
	return ret;
}

static ssize_t light_iopmp_attr_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct light_iopmp_info *info = platform_get_drvdata(pdev);
	ssize_t ret;

	ret = sprintf(buf, "0x%x\n", info->cur_entry.attr);
	return ret;
}

static ssize_t light_iopmp_lock_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct light_iopmp_info *info = platform_get_drvdata(pdev);
	ssize_t ret;

	ret = sprintf(buf, "%s\n", info->cur_entry.lock ? "locked" : "no lock");
	return ret;
}

static ssize_t light_iopmp_tap_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct light_iopmp_info *info = platform_get_drvdata(pdev);
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	if (light_iopmp_type_valid(info, (int)val))
		info->cur_entry.iopmp_type = val;
	else
		dev_err(dev, "invalid tap value\n");

	return count;
}

static ssize_t light_iopmp_start_addr_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct light_iopmp_info *info = platform_get_drvdata(pdev);
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	info->cur_entry.start_addr = (u32)val;

	return count;
}

static ssize_t light_iopmp_end_addr_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct light_iopmp_info *info = platform_get_drvdata(pdev);
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;
	if (!val)
		return -EINVAL;

	info->cur_entry.end_addr = (u32)val;

	return count;
}

static ssize_t light_iopmp_attr_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct light_iopmp_info *info = platform_get_drvdata(pdev);
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	info->cur_entry.attr = (u32)val;

	return count;
}

static ssize_t light_iopmp_lock_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct light_iopmp_info *info = platform_get_drvdata(pdev);
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	info->cur_entry.lock = (int)val;

	return count;
}

static ssize_t light_iopmp_set_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct light_iopmp_info *info = platform_get_drvdata(pdev);
	unsigned long val;
	int ret;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;
	if (!val)
		return count;

	if (!light_iopmp_type_valid(info, info->cur_entry.iopmp_type)) {
		dev_err(dev, "invalid iopmp tap ID\n");
		return -EINVAL;
	}

	if (info->cur_entry.end_addr <= info->cur_entry.start_addr) {
		dev_err(dev, "invalid end_addr/start_addr setting\n");
		return -EINVAL;
	}

	ret = light_iopmp_sys_set(info);
	if (ret)
		return -EINVAL;

	return count;
}

static DEVICE_ATTR(light_iopmp_tap, 0644, light_iopmp_tap_show, light_iopmp_tap_store);
static DEVICE_ATTR(light_iopmp_start_addr, 0644, light_iopmp_start_addr_show, light_iopmp_start_addr_store);
static DEVICE_ATTR(light_iopmp_end_addr, 0644, light_iopmp_end_addr_show, light_iopmp_end_addr_store);
static DEVICE_ATTR(light_iopmp_attr, 0644, light_iopmp_attr_show, light_iopmp_attr_store);
static DEVICE_ATTR(light_iopmp_lock, 0644, light_iopmp_lock_show, light_iopmp_lock_store);
static DEVICE_ATTR(light_iopmp_set, 0644, NULL, light_iopmp_set_store);

static struct attribute *light_iopmp_sysfs_entries[] = {
	&dev_attr_light_iopmp_tap.attr,
	&dev_attr_light_iopmp_start_addr.attr,
	&dev_attr_light_iopmp_end_addr.attr,
	&dev_attr_light_iopmp_attr.attr,
	&dev_attr_light_iopmp_lock.attr,
	&dev_attr_light_iopmp_set.attr,
	NULL
};

static const struct attribute_group dev_attr_light_iopmp_group = {
	.attrs = light_iopmp_sysfs_entries,
};

static int light_iopmp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct light_iopmp_driver_data *drvdata = NULL;
	struct light_iopmp_list *match;
	struct light_iopmp_info *info;
	struct device_node *entry_np;
	int size, entries;
	int ret;

	drvdata = device_get_match_data(dev);
	if (!drvdata) {
		dev_err(dev, "cannot get driver data\n");
		return -ENOENT;
	}
	match = drvdata->iopmp_list;
	entries = light_iopmp_get_entries(match);
	if (entries <= 0)
		return -ENOENT;

	size = sizeof(struct light_iopmp_entry) * entries + sizeof(*info);
	info = devm_kzalloc(dev, size, GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->entries = entries;
	info->iopmp_list = match;
	info->dev = dev;

	for_each_child_of_node(dev->of_node, entry_np) {
		int attr = LIGHT_IOPMP_ATTR_CFG_DEFAULT;
		int dummy_slave = LIGHT_IOPMP_DUMMY_ADDR;
		bool is_default_region = false;
		bool bypass_en = false;
		int region_size;
		int type;
		int i = 0, j;

		if (!of_device_is_available(entry_np))
			continue;

		type = light_iopmp_get_type(info, entry_np);
		if (type < 0) {
			dev_err(dev, "invalid iopmp tap:%d\n", type);
			continue;
		}

		ret = of_property_read_u32(entry_np, "attr", &attr);
		if (!ret && (attr == 0))
			attr = LIGHT_IOPMP_ATTR_CFG_DEFAULT;

		ret = of_property_read_u32(entry_np, "dummy_slave", &dummy_slave);
		if (!ret && (dummy_slave == 0))
			dummy_slave = LIGHT_IOPMP_DUMMY_ADDR;

		region_size = of_property_count_elems_of_size(entry_np, "regions", sizeof(u32));
		if (region_size <= 0) {
			is_default_region = of_property_read_bool(entry_np, "is_default_region");
			bypass_en = of_property_read_bool(entry_np, "bypass_en");

			if (!is_default_region && !bypass_en) {
				dev_err(dev, "missing/invalid reg property\n");
				continue;
			}

			/* config the iopmp entry */
			if (bypass_en)
				light_iopmp_set_attr_bypass(info, type);
			else if (is_default_region)
				light_iopmp_set_attr_default(info, type, attr);

		} else {
			region_size >>= 1;
			if (region_size > LIGHT_IOPMP_REGION_NUM) {
				dev_err(dev, "region size is invalid\n");
				continue;
			}

			for (j = 0; j < region_size; j++) {
				ret = of_property_read_u32_index(entry_np, "regions",
						j << 1, &info->iopmp_entry[i].reg_start[j]);
				if (ret)
					break;
				ret = of_property_read_u32_index(entry_np, "regions",
						(j << 1) + 1, &info->iopmp_entry[i].reg_end[j]);
				if (ret)
					break;

				/* check the region valid, otherwise drop the iopmp setting */
				if (j && info->iopmp_entry[i].reg_end[j - 1] > info->iopmp_entry[i].reg_start[j])
					break;
			}

			if (j == region_size) {
				info->iopmp_entry[i].entry_valid_num = region_size;

				/* config the iopmp entry */
				light_iopmp_set_attr(info, type, attr, dummy_slave, &info->iopmp_entry[i]);
			}
		}

		i++;
	}

	/* init the cur entry config */
	info->cur_entry.iopmp_type = -1;
	info->cur_entry.start_addr = 0;
	info->cur_entry.end_addr = 0;
	info->cur_entry.attr = 0;
	info->cur_entry.lock = false;

	platform_set_drvdata(pdev, info);

	ret = sysfs_create_group(&pdev->dev.kobj, &dev_attr_light_iopmp_group);
	if (ret) {
		dev_err(&pdev->dev, "Failed to create light iopmp debug sysfs.\n");
		return ret;
	}

	return 0;
}

static int light_iopmp_remove(struct platform_device *pdev)
{
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static int __maybe_unused light_iopmp_noirq_suspend(struct device *dev)
{
	/* the iopmp clocks depend on module theirself,
	 * we should keep them always enabled in clock's driver
	 */
	return 0;
}

static int __maybe_unused light_iopmp_noirq_resume(struct device *dev)
{
	/* TBD: restore IOPMP in noirq stage is too late ?
	 * we should restore these registers setting in early arch resume ?
	 */
	return 0;
}

static const struct dev_pm_ops light_iopmp_pm_ops = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(light_iopmp_noirq_suspend, light_iopmp_noirq_resume)
};

static const struct of_device_id light_iopmp_of_match[] = {
	{ .compatible = "thead,light-iopmp", .data = &light_iopmp_data},
	{},
};

static struct platform_driver light_iopmp_driver = {
	.driver = {
		.name           = "light-iopmp",
		.of_match_table = of_match_ptr(light_iopmp_of_match),
		.pm     	= &light_iopmp_pm_ops,
	},
	.probe = light_iopmp_probe,
	.remove = light_iopmp_remove,
};

static int __init light_iopmp_init(void)
{
        return platform_driver_register(&light_iopmp_driver);
}
arch_initcall(light_iopmp_init);

MODULE_ALIAS("platform:light-iopmp");
MODULE_AUTHOR("fugang.duan <duanfugang.dfg@linux.alibaba.com>");
MODULE_DESCRIPTION("Thead Light iopmp driver");
MODULE_LICENSE("GPL v2");
