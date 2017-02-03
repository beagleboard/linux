/*
 * PCIe host controller driver for HiSilicon SoCs
 *
 * Copyright (C) 2015 HiSilicon Co., Ltd. http://www.hisilicon.com
 *
 * Authors: Zhou Wang <wangzhou1@hisilicon.com>
 *          Dacai Zhu <zhudacai@hisilicon.com>
 *          Gabriele Paoloni <gabriele.paoloni@huawei.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/mfd/syscon.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/regmap.h>

#include "pcie-designware.h"

#define PCIE_SUBCTRL_SYS_STATE4_REG		0x6818
#define PCIE_HIP06_CTRL_OFF			0x1000
#define PCIE_SYS_STATE4				(PCIE_HIP06_CTRL_OFF + 0x31c)
#define PCIE_LTSSM_LINKUP_STATE			0x11
#define PCIE_LTSSM_STATE_MASK			0x3F

#define to_hisi_pcie(x)	dev_get_drvdata((x)->dev)

struct hisi_pcie;

struct pcie_soc_ops {
	int (*hisi_pcie_link_up)(struct hisi_pcie *hisi_pcie);
};

struct hisi_pcie {
	struct dw_pcie *pci;
	struct regmap *subctrl;
	u32 port_id;
	struct pcie_soc_ops *soc_ops;
};

/* HipXX PCIe host only supports 32-bit config access */
static int hisi_pcie_cfg_read(struct pcie_port *pp, int where, int size,
			      u32 *val)
{
	u32 reg;
	u32 reg_val;
	void *walker = &reg_val;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	void __iomem *base = pci->dbi_base;

	walker += (where & 0x3);
	reg = where & ~0x3;
	reg_val = dw_pcie_read_dbi(pci, base, reg, 0x4);

	if (size == 1)
		*val = *(u8 __force *) walker;
	else if (size == 2)
		*val = *(u16 __force *) walker;
	else if (size == 4)
		*val = reg_val;
	else
		return PCIBIOS_BAD_REGISTER_NUMBER;

	return PCIBIOS_SUCCESSFUL;
}

/* HipXX PCIe host only supports 32-bit config access */
static int hisi_pcie_cfg_write(struct pcie_port *pp, int where, int  size,
				u32 val)
{
	u32 reg_val;
	u32 reg;
	void *walker = &reg_val;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	void __iomem *base = pci->dbi_base;

	walker += (where & 0x3);
	reg = where & ~0x3;
	if (size == 4)
		dw_pcie_write_dbi(pci, base, reg, 0x4, val);
	else if (size == 2) {
		reg_val = dw_pcie_read_dbi(pci, base, reg, 0x4);
		*(u16 __force *) walker = val;
		dw_pcie_write_dbi(pci, base, reg, 0x4, reg_val);
	} else if (size == 1) {
		reg_val = dw_pcie_read_dbi(pci, base, reg, 0x4);
		*(u8 __force *) walker = val;
		dw_pcie_write_dbi(pci, base, reg, 0x4, reg_val);
	} else
		return PCIBIOS_BAD_REGISTER_NUMBER;

	return PCIBIOS_SUCCESSFUL;
}

static int hisi_pcie_link_up_hip05(struct hisi_pcie *hisi_pcie)
{
	u32 val;

	regmap_read(hisi_pcie->subctrl, PCIE_SUBCTRL_SYS_STATE4_REG +
		    0x100 * hisi_pcie->port_id, &val);

	return ((val & PCIE_LTSSM_STATE_MASK) == PCIE_LTSSM_LINKUP_STATE);
}

static int hisi_pcie_link_up_hip06(struct hisi_pcie *hisi_pcie)
{
	struct dw_pcie *pci = hisi_pcie->pci;
	void __iomem *base = pci->dbi_base;
	u32 val;

	val = dw_pcie_read_dbi(pci, base, PCIE_SYS_STATE4, 0x4);

	return ((val & PCIE_LTSSM_STATE_MASK) == PCIE_LTSSM_LINKUP_STATE);
}

static int hisi_pcie_link_up(struct dw_pcie *pci)
{
	struct hisi_pcie *hisi_pcie = to_hisi_pcie(pci);

	return hisi_pcie->soc_ops->hisi_pcie_link_up(hisi_pcie);
}

static struct dw_pcie_host_ops hisi_pcie_host_ops = {
	.rd_own_conf = hisi_pcie_cfg_read,
	.wr_own_conf = hisi_pcie_cfg_write,
};

static int hisi_add_pcie_port(struct hisi_pcie *hisi_pcie,
			      struct platform_device *pdev)
{
	struct dw_pcie *pci = hisi_pcie->pci;
	struct pcie_port *pp = &pci->pp;
	struct device *dev = &pdev->dev;
	int ret;
	u32 port_id;

	if (of_property_read_u32(dev->of_node, "port-id", &port_id)) {
		dev_err(dev, "failed to read port-id\n");
		return -EINVAL;
	}
	if (port_id > 3) {
		dev_err(dev, "Invalid port-id: %d\n", port_id);
		return -EINVAL;
	}
	hisi_pcie->port_id = port_id;

	pp->ops = &hisi_pcie_host_ops;

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(dev, "failed to initialize host\n");
		return ret;
	}

	return 0;
}

static const struct dw_pcie_ops dw_pcie_ops = {
	.link_up = hisi_pcie_link_up,
};

static int hisi_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dw_pcie *pci;
	struct hisi_pcie *hisi_pcie;
	const struct of_device_id *match;
	struct resource *reg;
	struct device_driver *driver;
	int ret;

	hisi_pcie = devm_kzalloc(dev, sizeof(*hisi_pcie), GFP_KERNEL);
	if (!hisi_pcie)
		return -ENOMEM;

	pci = devm_kzalloc(dev, sizeof(*pci), GFP_KERNEL);
	if (!pci)
		return -ENOMEM;

	pci->dev = dev;
	pci->ops = &dw_pcie_ops;

	hisi_pcie->pci = pci;

	driver = dev->driver;

	match = of_match_device(driver->of_match_table, dev);
	hisi_pcie->soc_ops = (struct pcie_soc_ops *) match->data;

	hisi_pcie->subctrl =
	syscon_regmap_lookup_by_compatible("hisilicon,pcie-sas-subctrl");
	if (IS_ERR(hisi_pcie->subctrl)) {
		dev_err(dev, "cannot get subctrl base\n");
		return PTR_ERR(hisi_pcie->subctrl);
	}

	reg = platform_get_resource_byname(pdev, IORESOURCE_MEM, "rc_dbi");
	pci->dbi_base = devm_ioremap_resource(dev, reg);
	if (IS_ERR(pci->dbi_base)) {
		dev_err(dev, "cannot get rc_dbi base\n");
		return PTR_ERR(pci->dbi_base);
	}

	platform_set_drvdata(pdev, hisi_pcie);

	ret = hisi_add_pcie_port(hisi_pcie, pdev);
	if (ret)
		return ret;

	dev_warn(dev, "only 32-bit config accesses supported; smaller writes may corrupt adjacent RW1C fields\n");

	return 0;
}

static struct pcie_soc_ops hip05_ops = {
		&hisi_pcie_link_up_hip05
};

static struct pcie_soc_ops hip06_ops = {
		&hisi_pcie_link_up_hip06
};

static const struct of_device_id hisi_pcie_of_match[] = {
	{
			.compatible = "hisilicon,hip05-pcie",
			.data	    = (void *) &hip05_ops,
	},
	{
			.compatible = "hisilicon,hip06-pcie",
			.data	    = (void *) &hip06_ops,
	},
	{},
};

static struct platform_driver hisi_pcie_driver = {
	.probe  = hisi_pcie_probe,
	.driver = {
		   .name = "hisi-pcie",
		   .of_match_table = hisi_pcie_of_match,
	},
};
builtin_platform_driver(hisi_pcie_driver);
