/*
 * PCIe host controller driver for HiSilicon Hip05 SoC
 *
 * Copyright (C) 2015 HiSilicon Co., Ltd. http://www.hisilicon.com
 *
 * Author: Zhou Wang <wangzhou1@hisilicon.com>
 *         Dacai Zhu <zhudacai@hisilicon.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include "pcie-designware.h"

#define PCIE_SUBCTRL_SYS_STATE4_REG                     0x6818
#define PCIE_LTSSM_LINKUP_STATE                         0x11
#define PCIE_LTSSM_STATE_MASK                           0x3F

#define to_hisi_pcie(x)	dev_get_drvdata((x)->dev)

struct hisi_pcie {
	struct regmap *subctrl;
	void __iomem *reg_base;
	u32 port_id;
	struct dw_pcie *pci;
};

static inline void hisi_pcie_apb_writel(struct hisi_pcie *pcie,
					u32 val, u32 reg)
{
	writel(val, pcie->reg_base + reg);
}

static inline u32 hisi_pcie_apb_readl(struct hisi_pcie *pcie, u32 reg)
{
	return readl(pcie->reg_base + reg);
}

/* Hip05 PCIe host only supports 32-bit config access */
static int hisi_pcie_cfg_read(struct pcie_port *pp, int where, int size,
			      u32 *val)
{
	u32 reg;
	u32 reg_val;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct hisi_pcie *pcie = to_hisi_pcie(pci);
	void *walker = &reg_val;

	walker += (where & 0x3);
	reg = where & ~0x3;
	reg_val = hisi_pcie_apb_readl(pcie, reg);

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

/* Hip05 PCIe host only supports 32-bit config access */
static int hisi_pcie_cfg_write(struct pcie_port *pp, int where, int  size,
				u32 val)
{
	u32 reg_val;
	u32 reg;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct hisi_pcie *pcie = to_hisi_pcie(pci);
	void *walker = &reg_val;

	walker += (where & 0x3);
	reg = where & ~0x3;
	if (size == 4)
		hisi_pcie_apb_writel(pcie, val, reg);
	else if (size == 2) {
		reg_val = hisi_pcie_apb_readl(pcie, reg);
		*(u16 __force *) walker = val;
		hisi_pcie_apb_writel(pcie, reg_val, reg);
	} else if (size == 1) {
		reg_val = hisi_pcie_apb_readl(pcie, reg);
		*(u8 __force *) walker = val;
		hisi_pcie_apb_writel(pcie, reg_val, reg);
	} else
		return PCIBIOS_BAD_REGISTER_NUMBER;

	return PCIBIOS_SUCCESSFUL;
}

static struct dw_pcie_host_ops hisi_pcie_host_ops = {
	.rd_own_conf = hisi_pcie_cfg_read,
	.wr_own_conf = hisi_pcie_cfg_write,
};

static int hisi_add_pcie_port(struct pcie_port *pp,
				     struct platform_device *pdev)
{
	int ret;
	u32 port_id;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct hisi_pcie *hisi_pcie = to_hisi_pcie(pci);

	if (of_property_read_u32(pdev->dev.of_node, "port-id", &port_id)) {
		dev_err(&pdev->dev, "failed to read port-id\n");
		return -EINVAL;
	}
	if (port_id > 3) {
		dev_err(&pdev->dev, "Invalid port-id: %d\n", port_id);
		return -EINVAL;
	}
	hisi_pcie->port_id = port_id;

	pp->ops = &hisi_pcie_host_ops;

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(&pdev->dev, "failed to initialize host\n");
		return ret;
	}

	return 0;
}

static int hisi_pcie_link_up(struct dw_pcie *pci)
{
	u32 val;
	struct hisi_pcie *hisi_pcie = to_hisi_pcie(pci);

	regmap_read(hisi_pcie->subctrl, PCIE_SUBCTRL_SYS_STATE4_REG +
		    0x100 * hisi_pcie->port_id, &val);

	return ((val & PCIE_LTSSM_STATE_MASK) == PCIE_LTSSM_LINKUP_STATE);
}

static const struct dw_pcie_ops hisi_dw_pcie_ops = {
	.link_up = hisi_pcie_link_up,
};

static int hisi_pcie_probe(struct platform_device *pdev)
{
	struct dw_pcie *pci;
	struct hisi_pcie *hisi_pcie;
	struct pcie_port *pp;
	struct resource *reg;
	int ret;

	hisi_pcie = devm_kzalloc(&pdev->dev, sizeof(*hisi_pcie), GFP_KERNEL);
	if (!hisi_pcie)
		return -ENOMEM;

	pci = devm_kzalloc(&pdev->dev, sizeof(*pci), GFP_KERNEL);
	if (!pci)
		return -ENOMEM;

	pci->dev = &pdev->dev;
	pci->ops = &hisi_dw_pcie_ops;

	pp = &pci->pp;

	hisi_pcie->subctrl =
	syscon_regmap_lookup_by_compatible("hisilicon,pcie-sas-subctrl");
	if (IS_ERR(hisi_pcie->subctrl)) {
		dev_err(pci->dev, "cannot get subctrl base\n");
		return PTR_ERR(hisi_pcie->subctrl);
	}

	reg = platform_get_resource_byname(pdev, IORESOURCE_MEM, "rc_dbi");
	hisi_pcie->reg_base = devm_ioremap_resource(&pdev->dev, reg);
	if (IS_ERR(hisi_pcie->reg_base)) {
		dev_err(pci->dev, "cannot get rc_dbi base\n");
		return PTR_ERR(hisi_pcie->reg_base);
	}

	hisi_pcie->pci = pci;
	hisi_pcie->pci->dbi_base = hisi_pcie->reg_base;

	ret = hisi_add_pcie_port(pp, pdev);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, hisi_pcie);

	dev_warn(pci->dev, "only 32-bit config accesses supported; smaller writes may corrupt adjacent RW1C fields\n");

	return 0;
}

static const struct of_device_id hisi_pcie_of_match[] = {
	{.compatible = "hisilicon,hip05-pcie",},
	{},
};

MODULE_DEVICE_TABLE(of, hisi_pcie_of_match);

static struct platform_driver hisi_pcie_driver = {
	.probe  = hisi_pcie_probe,
	.driver = {
		   .name = "hisi-pcie",
		   .of_match_table = hisi_pcie_of_match,
	},
};

module_platform_driver(hisi_pcie_driver);
