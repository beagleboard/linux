// SPDX-License-Identifier: GPL-2.0
/*
 * TI K3 SoC info driver
 *
 * Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sys_soc.h>

#define CTRLMMR_WKUP_JTAGID_REG		0
/*
 * Bits:
 *  31-28 VARIANT	Device variant
 *  27-12 PARTNO	Part number
 *  11-1  MFG		Indicates TI as manufacturer (0x17)
 *  1			Always 1
 */
#define CTRLMMR_WKUP_JTAGID_VARIANT_SHIFT	(28)
#define CTRLMMR_WKUP_JTAGID_VARIANT_MASK	GENMASK(31, 28)

#define CTRLMMR_WKUP_JTAGID_PARTNO_SHIFT	(12)
#define CTRLMMR_WKUP_JTAGID_PARTNO_MASK		GENMASK(27, 12)

#define CTRLMMR_WKUP_JTAGID_MFG_SHIFT		(1)
#define CTRLMMR_WKUP_JTAGID_MFG_MASK		GENMASK(11, 1)

#define CTRLMMR_WKUP_JTAGID_MFG_TI		0x17

static const struct k3_soc_id {
	unsigned int id;
	const char *family_name;
} k3_soc_ids[] = {
	{ 0xBB5A, "AM65X" },
	{ 0xBB64, "J721E" },
};

int __init partno_to_names(unsigned int partno,
			   struct soc_device_attribute *soc_dev_attr)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(k3_soc_ids); i++)
		if (partno == k3_soc_ids[i].id) {
			soc_dev_attr->family = k3_soc_ids[i].family_name;
			return 0;
		}

	return -EINVAL;
}

static int __init am65s_chipinfo_init(void)
{
	struct soc_device_attribute *soc_dev_attr;
	struct soc_device *soc_dev;
	struct device_node *node;
	struct regmap *regmap;
	u32 jtag_id;
	u32 partno_id;
	u32 variant;
	u32 mfg;
	int ret;

	node = of_find_compatible_node(NULL, NULL, "ti,am654-chipid");
	if (!node)
		return -ENODEV;

	regmap = device_node_to_regmap(node);
	of_node_put(node);

	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	ret = regmap_read(regmap, CTRLMMR_WKUP_JTAGID_REG, &jtag_id);
	if (ret < 0)
		return ret;

	mfg = (jtag_id & CTRLMMR_WKUP_JTAGID_MFG_MASK) >>
	       CTRLMMR_WKUP_JTAGID_MFG_SHIFT;

	if (mfg != CTRLMMR_WKUP_JTAGID_MFG_TI) {
		pr_err("Invalid MFG SoC\n");
		return -ENODEV;
	}

	variant = (jtag_id & CTRLMMR_WKUP_JTAGID_VARIANT_MASK) >>
		  CTRLMMR_WKUP_JTAGID_VARIANT_SHIFT;
	variant++;

	partno_id = (jtag_id & CTRLMMR_WKUP_JTAGID_PARTNO_MASK) >>
		 CTRLMMR_WKUP_JTAGID_PARTNO_SHIFT;

	soc_dev_attr = kzalloc(sizeof(*soc_dev_attr), GFP_KERNEL);
	if (!soc_dev_attr)
		return -ENOMEM;

	soc_dev_attr->revision = kasprintf(GFP_KERNEL, "SR%x.0", variant);

	ret = partno_to_names(partno_id, soc_dev_attr);
	if (ret) {
		pr_err("Unknown SoC JTAGID[0x%08X]\n", jtag_id);
		ret = -ENODEV;
		goto err;
	}

	node = of_find_node_by_path("/");
	of_property_read_string(node, "model", &soc_dev_attr->machine);
	of_node_put(node);

	soc_dev = soc_device_register(soc_dev_attr);
	if (IS_ERR(soc_dev)) {
		ret = PTR_ERR(soc_dev);
		goto err;
	}

	pr_debug("Family:%s rev:%s JTAGID[0x%08x] Detected\n",
		 soc_dev_attr->family,
		 soc_dev_attr->revision, jtag_id);

	return 0;

err:
	kfree(soc_dev_attr->revision);
	kfree(soc_dev_attr);
	return ret;
}

subsys_initcall(am65s_chipinfo_init);
