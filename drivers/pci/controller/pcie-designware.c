/*
 * Synopsys Designware PCIe controller driver
 *
 * Copyright (C) 2013 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Author: Jingoo Han <jg1.han@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/pci.h>

#include "pcie-designware.h"

int dw_pcie_read(void __iomem *addr, int size, u32 *val)
{
	if ((uintptr_t)addr & (size - 1)) {
		*val = 0;
		return PCIBIOS_BAD_REGISTER_NUMBER;
	}

	if (size == 4)
		*val = readl(addr);
	else if (size == 2)
		*val = readw(addr);
	else if (size == 1)
		*val = readb(addr);
	else {
		*val = 0;
		return PCIBIOS_BAD_REGISTER_NUMBER;
	}

	return PCIBIOS_SUCCESSFUL;
}

int dw_pcie_write(void __iomem *addr, int size, u32 val)
{
	if ((uintptr_t)addr & (size - 1))
		return PCIBIOS_BAD_REGISTER_NUMBER;

	if (size == 4)
		writel(val, addr);
	else if (size == 2)
		writew(val, addr);
	else if (size == 1)
		writeb(val, addr);
	else
		return PCIBIOS_BAD_REGISTER_NUMBER;

	return PCIBIOS_SUCCESSFUL;
}

void dw_pcie_read_dbi(struct dw_pcie *pci, void __iomem *base,
		      u32 reg, int size, u32 *val)
{
	int ret;

	if (pci->ops->read_dbi) {
		pci->ops->read_dbi(pci, base + reg, size, val);
		return;
	}

	ret = dw_pcie_read(base + reg, size, val);
	if (ret)
		dev_err(pci->dev, "read DBI address failed\n");
}

void dw_pcie_write_dbi(struct dw_pcie *pci, void __iomem *base, u32 reg,
		       int size, u32 val)
{
	int ret;

	if (pci->ops->write_dbi) {
		pci->ops->write_dbi(pci, base + reg, size, val);
		return;
	}

	ret = dw_pcie_write(base + reg, size, val);
	if (ret)
		dev_err(pci->dev, "write DBI address failed\n");
}

void dw_pcie_prog_outbound_atu(struct dw_pcie *pci, int index,
			       int type, u64 cpu_addr, u64 pci_addr,
			       u32 size)
{
	u32 val;
	void __iomem *base = pci->dbi_base;

	dw_pcie_write_dbi(pci, base, PCIE_ATU_VIEWPORT, 0x4,
			  PCIE_ATU_REGION_OUTBOUND | index);
	dw_pcie_write_dbi(pci, base, PCIE_ATU_LOWER_BASE, 0x4,
			  lower_32_bits(cpu_addr));
	dw_pcie_write_dbi(pci, base, PCIE_ATU_UPPER_BASE, 0x4,
			  upper_32_bits(cpu_addr));
	dw_pcie_write_dbi(pci, base, PCIE_ATU_LIMIT, 0x4,
			  lower_32_bits(cpu_addr + size - 1));
	dw_pcie_write_dbi(pci, base, PCIE_ATU_LOWER_TARGET, 0x4,
			  lower_32_bits(pci_addr));
	dw_pcie_write_dbi(pci, base, PCIE_ATU_UPPER_TARGET, 0x4,
			  upper_32_bits(pci_addr));
	dw_pcie_write_dbi(pci, base, PCIE_ATU_CR1, 0x4, type);
	dw_pcie_write_dbi(pci, base, PCIE_ATU_CR2, 0x4, PCIE_ATU_ENABLE);

	/*
	 * Make sure ATU enable takes effect before any subsequent config
	 * and I/O accesses.
	 */
	dw_pcie_read_dbi(pci, base, PCIE_ATU_CR2, 0x4, &val);
}

int dw_pcie_link_up(struct dw_pcie *pci)
{
	if (pci->ops->link_up)
		return pci->ops->link_up(pci);
	else
		return 0;
}

void dw_pcie_setup(struct dw_pcie *pci)
{
	u32 val;
	int ret;
	void __iomem *base = pci->dbi_base;
	struct device *dev = pci->dev;
	struct device_node *np = dev->of_node;

	ret = of_property_read_u32(np, "num-lanes", &pci->lanes);
	if (ret)
		pci->lanes = 0;

	/* set the number of lanes */
	dw_pcie_read_dbi(pci, base, PCIE_PORT_LINK_CONTROL, 0x4, &val);
	val &= ~PORT_LINK_MODE_MASK;
	switch (pci->lanes) {
	case 1:
		val |= PORT_LINK_MODE_1_LANES;
		break;
	case 2:
		val |= PORT_LINK_MODE_2_LANES;
		break;
	case 4:
		val |= PORT_LINK_MODE_4_LANES;
		break;
	case 8:
		val |= PORT_LINK_MODE_8_LANES;
		break;
	default:
		dev_err(pci->dev, "num-lanes %u: invalid value\n", pci->lanes);
		return;
	}
	dw_pcie_write_dbi(pci, base, PCIE_PORT_LINK_CONTROL, 0x4, val);

	/* set link width speed control register */
	dw_pcie_read_dbi(pci, base, PCIE_LINK_WIDTH_SPEED_CONTROL, 0x4, &val);
	val &= ~PORT_LOGIC_LINK_WIDTH_MASK;
	switch (pci->lanes) {
	case 1:
		val |= PORT_LOGIC_LINK_WIDTH_1_LANES;
		break;
	case 2:
		val |= PORT_LOGIC_LINK_WIDTH_2_LANES;
		break;
	case 4:
		val |= PORT_LOGIC_LINK_WIDTH_4_LANES;
		break;
	case 8:
		val |= PORT_LOGIC_LINK_WIDTH_8_LANES;
		break;
	}
	dw_pcie_write_dbi(pci, base, PCIE_LINK_WIDTH_SPEED_CONTROL, 0x4, val);
}

MODULE_AUTHOR("Jingoo Han <jg1.han@samsung.com>");
MODULE_DESCRIPTION("Designware PCIe host controller driver");
MODULE_LICENSE("GPL v2");
