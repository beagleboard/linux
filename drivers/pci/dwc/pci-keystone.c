/*
 * PCIe host controller driver for Texas Instruments Keystone SoCs
 *
 * Copyright (C) 2013-2014 Texas Instruments., Ltd.
 *		http://www.ti.com
 *
 * Author: Murali Karicheri <m-karicheri2@ti.com>
 * Implementation based on pci-exynos.c and pcie-designware.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/mfd/syscon.h>
#include <linux/msi.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_pci.h>
#include <linux/pci.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/resource.h>
#include <linux/signal.h>

#include "pcie-designware.h"

#define PCIE_VENDORID_MASK	0xffff
#define PCIE_DEVICEID_SHIFT	16

/* Application registers */
#define CMD_STATUS			0x004
#define LTSSM_EN_VAL		        BIT(0)
#define OB_XLAT_EN_VAL		        BIT(1)
#define IB_XLAT_EN_VAL			BIT(2)
#define DBI_CS2				BIT(5)

#define CFG_SETUP			0x008
#define CFG_BUS(x)			(((x) & 0xff) << 16)
#define CFG_DEVICE(x)			(((x) & 0x1f) << 8)
#define CFG_FUNC(x)			((x) & 0x7)
#define CFG_TYPE1			BIT(24)

#define OB_SIZE				0x030
#define OB_WIN_SIZE			8	/* 8MB */

/* IRQ register defines */
#define IRQ_EOI				0x050
#define MSI_IRQ				0x054
#define MSI_IRQ_STATUS(n)		(0x104 + ((n) << 4))
#define MSI_IRQ_ENABLE_SET(n)		(0x108 + ((n) << 4))
#define MSI_IRQ_ENABLE_CLR(n)		(0x10c + ((n) << 4))
#define MSI_IRQ_OFFSET			4

#define IRQ_STATUS(n)			(0x184 + ((n) << 4))
#define IRQ_ENABLE_SET(n)		(0x188 + ((n) << 4))
#define INTx_EN				BIT(0)

#define ERR_IRQ_STATUS			0x1c4
#define ERR_IRQ_ENABLE_SET		0x1c8
#define ERR_AER				BIT(5)	/* ECRC error */
#define ERR_AXI				BIT(4)	/* AXI tag lookup fatal error */
#define ERR_CORR			BIT(3)	/* Correctable error */
#define ERR_NONFATAL			BIT(2)	/* Non-fatal error */
#define ERR_FATAL			BIT(1)	/* Fatal error */
#define ERR_SYS				BIT(0)	/* System error */
#define ERR_IRQ_ALL			(ERR_AER | ERR_AXI | ERR_CORR | \
					 ERR_NONFATAL | ERR_FATAL | ERR_SYS)

#define OB_OFFSET_INDEX(n)		(0x200 + (8 * (n)))
#define OB_ENABLEN			BIT(0)

#define OB_OFFSET_HI(n)			(0x204 + (8 * (n)))

#define KS_PCIE_DEV_TYPE_MASK		(0x3 << 1)
#define KS_PCIE_DEV_TYPE(mode)		((mode) << 1)

#define AM654_PCIE_DEV_TYPE_MASK		0x3

#define EP				0x0
#define LEG_EP				0x1
#define RC				0x2

#define KS_PCIE_SYSCLOCKOUTEN		BIT(0)

#define IB_BAR(n)			(0x300 + (0x10 * (n)))
#define IB_START_LO(n)			(0x304 + (0x10 * (n)))
#define IB_START_HI(n)			(0x308 + (0x10 * (n)))
#define IB_OFFSET(n)			(0x30c + (0x10 * (n)))

#define APP_ADDR_SPACE_0		(16 * SZ_1K)

#define WIN_INDEX_MASK			0x1f
#define WIN_INDEX_SHIFT			20
#define WIN_SIZE			SZ_1M

#define AM654_WIN_SIZE			SZ_64K

#define PCIE_LEGACY_IRQ_ENABLE_SET(n)	(0x188 + (0x10 * ((n) - 1)))
#define PCIE_LEGACY_IRQ_ENABLE_CLR(n)	(0x18c + (0x10 * ((n) - 1)))
#define PCIE_EP_IRQ_SET			0x64
#define PCIE_EP_IRQ_CLR			0x68
#define INT_ENABLE			BIT(0)

#define MAX_OB_WINDOWS			32

#define to_keystone_pcie(x)		dev_get_drvdata((x)->dev)

#define PCI_DEVICE_ID_TI_AM654X		0xb00c

#define EXP_CAP_ID_OFFSET              0x70

/*
 * Size of BAR0, BAR2, BAR5 are from RESBAR.
 * TODO: How are sizes of other BARs determined.
 */
static size_t bar_size[] = { SZ_1M, SZ_64K, SZ_1M, SZ_64K, 256, SZ_1M };

static int ks_pcie_start_link(struct dw_pcie *pci);
static void ks_pcie_stop_link(struct dw_pcie *pci);

struct ks_pcie_of_data {
	enum dw_pcie_device_mode mode;
	const struct dw_pcie_ops *ops;
	const struct dw_pcie_host_ops *host_ops;
	const struct dw_pcie_ep_ops *ep_ops;
	unsigned int version;
};

struct ks_pcie_outbound_win {
	u64	cpu_addr;
	u8	no_of_regions;
};

struct keystone_pcie {
	struct dw_pcie		*pci;
	int			num_lanes;
	u32			num_ob_windows;
	struct phy		**phy;
	struct device_link	**link;
	int			msi_host_irq;
	struct			device_node *msi_intc_np;
	struct irq_domain	*legacy_irq_domain;
	struct device_node	*np;

	/* Application register space */
	void __iomem		*va_app_base;	/* DT 1st resource */
	struct resource		app;
	unsigned long		ob_window_map;
	struct ks_pcie_outbound_win *ob_win;
};

static phys_addr_t ks_pcie_get_msi_addr(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct keystone_pcie *ks_pcie = to_keystone_pcie(pci);

	return ks_pcie->app.start + MSI_IRQ;
}

static u32 ks_pcie_app_readl(struct keystone_pcie *ks_pcie, u32 offset)
{
	return readl(ks_pcie->va_app_base + offset);
}

static void ks_pcie_app_writel(struct keystone_pcie *ks_pcie, u32 offset,
			       u32 val)
{
	writel(val, ks_pcie->va_app_base + offset);
}

/**
 * ks_pcie_set_dbi_mode() - Set DBI mode to access overlaid BAR mask
 * registers
 *
 * Since modification of dbi_cs2 involves different clock domain, read the
 * status back to ensure the transition is complete.
 */
static void ks_pcie_set_dbi_mode(struct keystone_pcie *ks_pcie)
{
	u32 val;

	val = ks_pcie_app_readl(ks_pcie, CMD_STATUS);
	val |= DBI_CS2;
	ks_pcie_app_writel(ks_pcie, CMD_STATUS, val);

	do {
		val = ks_pcie_app_readl(ks_pcie, CMD_STATUS);
	} while (!(val & DBI_CS2));
}

/**
 * ks_pcie_clear_dbi_mode() - Disable DBI mode
 *
 * Since modification of dbi_cs2 involves different clock domain, read the
 * status back to ensure the transition is complete.
 */
static void ks_pcie_clear_dbi_mode(struct keystone_pcie *ks_pcie)
{
	u32 val;

	val = ks_pcie_app_readl(ks_pcie, CMD_STATUS);
	val &= ~DBI_CS2;
	ks_pcie_app_writel(ks_pcie, CMD_STATUS, val);

	do {
		val = ks_pcie_app_readl(ks_pcie, CMD_STATUS);
	} while (val & DBI_CS2);
}

static u32 ks_pcie_read_dbi2(struct dw_pcie *pci, void __iomem *base,
			     u32 reg, size_t size)
{
	struct keystone_pcie *ks_pcie = to_keystone_pcie(pci);
	u32 val;

	ks_pcie_set_dbi_mode(ks_pcie);
	dw_pcie_read(base + reg, size, &val);
	ks_pcie_clear_dbi_mode(ks_pcie);
	return val;
}

static void ks_pcie_write_dbi2(struct dw_pcie *pci, void __iomem *base,
			       u32 reg, size_t size, u32 val)
{
	struct keystone_pcie *ks_pcie = to_keystone_pcie(pci);

	ks_pcie_set_dbi_mode(ks_pcie);
	dw_pcie_write(base + reg, size, val);
	ks_pcie_clear_dbi_mode(ks_pcie);
}

static void ks_pcie_msi_irq_ack(int irq, struct pcie_port *pp)
{
	u32 reg_offset, bit_pos;
	struct keystone_pcie *ks_pcie;
	struct dw_pcie *pci;

	pci = to_dw_pcie_from_pp(pp);
	ks_pcie = to_keystone_pcie(pci);

	reg_offset = irq % 8;
	bit_pos = irq >> 3;
	ks_pcie_app_writel(ks_pcie, MSI_IRQ_STATUS(reg_offset), BIT(bit_pos));
	ks_pcie_app_writel(ks_pcie, IRQ_EOI, MSI_IRQ_OFFSET + reg_offset);
}

static void ks_pcie_msi_set_irq(struct pcie_port *pp, int irq)
{
	u32 reg_offset, bit_pos;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct keystone_pcie *ks_pcie = to_keystone_pcie(pci);

	reg_offset = irq % 8;
	bit_pos = irq >> 3;
	ks_pcie_app_writel(ks_pcie, MSI_IRQ_ENABLE_SET(reg_offset),
			   BIT(bit_pos));
}

static void ks_pcie_msi_clear_irq(struct pcie_port *pp, int irq)
{
	u32 reg_offset, bit_pos;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct keystone_pcie *ks_pcie = to_keystone_pcie(pci);

	reg_offset = irq % 8;
	bit_pos = irq >> 3;
	ks_pcie_app_writel(ks_pcie, MSI_IRQ_ENABLE_CLR(reg_offset),
			   BIT(bit_pos));
}

static int ks_pcie_msi_host_init(struct pcie_port *pp)
{
	return dw_pcie_allocate_domains(pp);
}

static int ks_pcie_am654_msi_host_init(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct device *dev = pci->dev;

	dev_vdbg(dev, "dummy function so that DW core doesn't configure MSI\n");

	return 0;
}

void ks_pcie_enable_error_irq(struct keystone_pcie *ks_pcie)
{
	ks_pcie_app_writel(ks_pcie, ERR_IRQ_ENABLE_SET, ERR_IRQ_ALL);
}

static int ks_pcie_rd_other_conf(struct pcie_port *pp, struct pci_bus *bus,
				 unsigned int devfn, int where, int size,
				 u32 *val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct keystone_pcie *ks_pcie = to_keystone_pcie(pci);
	u32 reg;

	reg = CFG_BUS(bus->number) | CFG_DEVICE(PCI_SLOT(devfn)) |
		CFG_FUNC(PCI_FUNC(devfn));
	if (bus->parent->number != pp->root_bus_nr)
		reg |= CFG_TYPE1;
	ks_pcie_app_writel(ks_pcie, CFG_SETUP, reg);

	return dw_pcie_read(pp->va_cfg0_base + where, size, val);
}

static int ks_pcie_wr_other_conf(struct pcie_port *pp, struct pci_bus *bus,
				 unsigned int devfn, int where, int size,
				 u32 val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct keystone_pcie *ks_pcie = to_keystone_pcie(pci);
	u32 reg;

	reg = CFG_BUS(bus->number) | CFG_DEVICE(PCI_SLOT(devfn)) |
		CFG_FUNC(PCI_FUNC(devfn));
	if (bus->parent->number != pp->root_bus_nr)
		reg |= CFG_TYPE1;
	ks_pcie_app_writel(ks_pcie, CFG_SETUP, reg);

	return dw_pcie_write(pp->va_cfg0_base + where, size, val);
}

/**
 * ks_pcie_v3_65_scan_bus() - keystone scan_bus post initialization
 *
 * This sets BAR0 to enable inbound access for MSI_IRQ register
 */
static void ks_pcie_v3_65_scan_bus(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct keystone_pcie *ks_pcie = to_keystone_pcie(pci);

	/* Configure and set up BAR0 */
	ks_pcie_set_dbi_mode(ks_pcie);

	/* Enable BAR0 */
	dw_pcie_writel_dbi(pci, PCI_BASE_ADDRESS_0, 1);
	dw_pcie_writel_dbi(pci, PCI_BASE_ADDRESS_0, SZ_4K - 1);

	ks_pcie_clear_dbi_mode(ks_pcie);

	 /*
	  * For BAR0, just setting bus address for inbound writes (MSI) should
	  * be sufficient.  Use physical address to avoid any conflicts.
	  */
	dw_pcie_writel_dbi(pci, PCI_BASE_ADDRESS_0, ks_pcie->app.start);
}

/**
 * ks_pcie_link_up() - Check if link up
 */
static int ks_pcie_link_up(struct dw_pcie *pci)
{
	u32 val;

	val = dw_pcie_readl_dbi(pci, PCIE_PORT_DEBUG0);
	val &= PORT_LOGIC_LTSSM_STATE_MASK;
	return (val == PORT_LOGIC_LTSSM_STATE_L0);
}

#ifdef CONFIG_PCI
static void ks_pcie_quirk(struct pci_dev *dev)
{
	struct pci_bus *bus = dev->bus;
	struct pci_dev *bridge;
	static const struct pci_device_id rc_pci_devids[] = {
		{ PCI_DEVICE(PCI_VENDOR_ID_TI, PCI_DEVICE_ID_TI_K2HK),
		 .class = PCI_CLASS_BRIDGE_PCI << 8, .class_mask = ~0, },
		{ PCI_DEVICE(PCI_VENDOR_ID_TI, PCI_DEVICE_ID_TI_K2E),
		 .class = PCI_CLASS_BRIDGE_PCI << 8, .class_mask = ~0, },
		{ PCI_DEVICE(PCI_VENDOR_ID_TI, PCI_DEVICE_ID_TI_K2L),
		 .class = PCI_CLASS_BRIDGE_PCI << 8, .class_mask = ~0, },
		{ PCI_DEVICE(PCI_VENDOR_ID_TI, PCI_DEVICE_ID_TI_K2G),
		 .class = PCI_CLASS_BRIDGE_PCI << 8, .class_mask = ~0, },
		{ PCI_DEVICE(PCI_VENDOR_ID_TI, PCI_DEVICE_ID_TI_AM654X),
		 .class = PCI_CLASS_BRIDGE_PCI << 8, .class_mask = ~0, },
		{ 0, },
	};

	if (pci_is_root_bus(bus))
		bridge = dev;

	/* look for the host bridge */
	while (!pci_is_root_bus(bus)) {
		bridge = bus->self;
		bus = bus->parent;
	}

	if (!bridge)
		return;

	/*
	 * Keystone PCI controller has a h/w limitation of
	 * 256 bytes maximum read request size.  It can't handle
	 * anything higher than this.  So force this limit on
	 * all downstream devices.
	 */
	if (pci_match_id(rc_pci_devids, bridge)) {
		if (pcie_get_readrq(dev) > 256) {
			dev_info(&dev->dev, "limiting MRRS to 256\n");
			pcie_set_readrq(dev, 256);
		}
	}
}
DECLARE_PCI_FIXUP_ENABLE(PCI_ANY_ID, PCI_ANY_ID, ks_pcie_quirk);
#endif

static void ks_pcie_msi_irq_handler(struct irq_desc *desc)
{
	u32 reg;
	u32 vector;
	int pos;
	int virq;
	unsigned int irq = irq_desc_get_irq(desc);
	struct keystone_pcie *ks_pcie = irq_desc_get_handler_data(desc);
	u32 offset = irq - ks_pcie->msi_host_irq;
	struct dw_pcie *pci = ks_pcie->pci;
	struct pcie_port *pp = &pci->pp;
	struct device *dev = pci->dev;
	struct irq_chip *chip = irq_desc_get_chip(desc);

	chained_irq_enter(chip, desc);

	reg = ks_pcie_app_readl(ks_pcie, MSI_IRQ_STATUS(offset));
	for (pos = 0; pos < 4; pos++) {
		if (!(reg & BIT(pos)))
			continue;

		vector = offset + (pos << 3);
		virq = irq_linear_revmap(pp->irq_domain, vector);
		dev_dbg(dev, "irq: bit %d, vector %d, virq %d\n", pos, vector,
			virq);
		generic_handle_irq(virq);
	}

	chained_irq_exit(chip, desc);
}

/**
 * ks_pcie_legacy_irq_handler() - Handle legacy interrupt
 * @irq: IRQ line for legacy interrupts
 * @desc: Pointer to irq descriptor
 *
 * Traverse through pending legacy interrupts and invoke handler for each. Also
 * takes care of interrupt controller level mask/ack operation.
 */
static void ks_pcie_legacy_irq_handler(struct irq_desc *desc)
{
	int i;
	u32 reg;
	int virq;
	struct keystone_pcie *ks_pcie = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);

	chained_irq_enter(chip, desc);

	for (i = 0; i < PCI_NUM_INTX; i++) {
		reg = ks_pcie_app_readl(ks_pcie, IRQ_STATUS(i));
		if (!(reg & INTx_EN))
			continue;

		virq = irq_linear_revmap(ks_pcie->legacy_irq_domain, i);
		generic_handle_irq(virq);
		ks_pcie_app_writel(ks_pcie, IRQ_STATUS(i), INTx_EN);
		ks_pcie_app_writel(ks_pcie, IRQ_EOI, i);
	}

	chained_irq_exit(chip, desc);
}

static int ks_pcie_config_msi_irq(struct keystone_pcie *ks_pcie)
{
	struct device *dev = ks_pcie->pci->dev;
	struct device_node *np = ks_pcie->np;
	struct device_node *intc_np;
	int irq_count;
	int irq;
	int i;

	intc_np = of_get_child_by_name(np, "msi-interrupt-controller");
	if (!intc_np) {
		if (of_device_is_compatible(np, "ti,am654-pcie-rc"))
			return 0;
		dev_WARN(dev, "msi-interrupt-controller node is absent\n");
		return -EINVAL;
	}

	irq_count = of_irq_count(intc_np);
	if (!irq_count) {
		dev_err(dev, "No IRQ entries in msi-interrupt-controller\n");
		return -EINVAL;
	}

	for (i = 0; i < irq_count; i++) {
		irq = irq_of_parse_and_map(intc_np, i);
		if (!irq)
			return -EINVAL;

		if (!ks_pcie->msi_host_irq)
			ks_pcie->msi_host_irq = irq;

		irq_set_chained_handler_and_data(irq, ks_pcie_msi_irq_handler,
						 ks_pcie);
	}

	return 0;
}

static int ks_pcie_intx_map(struct irq_domain *domain, unsigned int irq,
			    irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &dummy_irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

static const struct irq_domain_ops ks_pcie_intx_domain_ops = {
	.map = ks_pcie_intx_map,
};

static int ks_pcie_config_legacy_irq(struct keystone_pcie *ks_pcie)
{
	struct device *dev = ks_pcie->pci->dev;
	struct irq_domain *legacy_irq_domain;
	struct device_node *np = ks_pcie->np;
	struct device_node *intc_np;
	int irq_count;
	int irq;
	int i;

	intc_np = of_get_child_by_name(np, "legacy-interrupt-controller");
	if (!intc_np) {
		dev_WARN(dev, "legacy-interrupt-controller node is absent\n");
		return -EINVAL;
	}

	irq_count = of_irq_count(intc_np);
	if (!irq_count) {
		dev_err(dev, "No IRQ entries in legacy-interrupt-controller\n");
		return -EINVAL;
	}

	for (i = 0; i < irq_count; i++) {
		irq = irq_of_parse_and_map(intc_np, i);
		if (!irq)
			return -EINVAL;
		irq_set_chained_handler_and_data(irq,
						 ks_pcie_legacy_irq_handler,
						 ks_pcie);
	}

	legacy_irq_domain = irq_domain_add_linear(intc_np,  PCI_NUM_INTX,
						  &ks_pcie_intx_domain_ops,
						  NULL);
	if (!legacy_irq_domain) {
		dev_err(dev, "Failed to add irq domain for legacy irqs\n");
		return -EINVAL;
	}
	ks_pcie->legacy_irq_domain = legacy_irq_domain;

	for (i = 0; i < PCI_NUM_INTX; i++)
		ks_pcie_app_writel(ks_pcie, IRQ_ENABLE_SET(i), INTx_EN);

	return 0;
}

#ifdef CONFIG_ARM
/*
 * When a PCI device does not exist during config cycles, keystone host gets a
 * bus error instead of returning 0xffffffff. This handler always returns 0
 * for this kind of faults.
 */
static int ks_pcie_fault(unsigned long addr, unsigned int fsr,
			 struct pt_regs *regs)
{
	unsigned long instr = *(unsigned long *) instruction_pointer(regs);

	if ((instr & 0x0e100090) == 0x00100090) {
		int reg = (instr >> 12) & 15;

		regs->uregs[reg] = -1;
		regs->ARM_pc += 4;
	}

	return 0;
}
#endif

static void ks_pcie_setup_mem_space(struct keystone_pcie *ks_pcie)
{
	u32 val;
	u32 num_ob_windows = ks_pcie->num_ob_windows;
	struct dw_pcie *pci = ks_pcie->pci;
	struct device *dev = pci->dev;
	struct device_node *np = dev->of_node;
	struct pcie_port *pp = &pci->pp;
	u64 start = pp->mem->start;
	u64 end = pp->mem->end;
	int i;

	if (of_device_is_compatible(np, "ti,am654-pcie-rc"))
		return;

	val = ilog2(OB_WIN_SIZE);
	ks_pcie_app_writel(ks_pcie, OB_SIZE, val);

	/* Using Direct 1:1 mapping of RC <-> PCI memory space */
	for (i = 0; i < num_ob_windows && (start < end); i++) {
		ks_pcie_app_writel(ks_pcie, OB_OFFSET_INDEX(i),
				   lower_32_bits(start) | OB_ENABLEN);
		ks_pcie_app_writel(ks_pcie, OB_OFFSET_HI(i),
				   upper_32_bits(start));
		start += OB_WIN_SIZE;
	}

	val = ks_pcie_app_readl(ks_pcie, CMD_STATUS);
	val |= OB_XLAT_EN_VAL;
	ks_pcie_app_writel(ks_pcie, CMD_STATUS, val);
}

static int __init ks_pcie_init_id(struct keystone_pcie *ks_pcie)
{
	int ret;
	unsigned int id;
	struct regmap *devctrl_regs;
	struct dw_pcie *pci = ks_pcie->pci;
	struct device *dev = pci->dev;
	struct device_node *np = dev->of_node;

	devctrl_regs = syscon_regmap_lookup_by_phandle(np, "ti,syscon-pcie-id");
	if (IS_ERR(devctrl_regs))
		return PTR_ERR(devctrl_regs);

	ret = regmap_read(devctrl_regs, 0, &id);
	if (ret)
		return ret;

	dw_pcie_dbi_ro_wr_en(pci);
	dw_pcie_writew_dbi(pci, PCI_VENDOR_ID, id & PCIE_VENDORID_MASK);
	dw_pcie_writew_dbi(pci, PCI_DEVICE_ID, id >> PCIE_DEVICEID_SHIFT);
	dw_pcie_dbi_ro_wr_dis(pci);

	return 0;
}

static int __init ks_pcie_host_init(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct keystone_pcie *ks_pcie = to_keystone_pcie(pci);
	int ret;

	ret = ks_pcie_config_legacy_irq(ks_pcie);
	if (ret)
		return ret;

	ret = ks_pcie_config_msi_irq(ks_pcie);
	if (ret)
		return ret;

	dw_pcie_setup_rc(pp);

	ks_pcie_set_dbi_mode(ks_pcie);
	dw_pcie_writel_dbi(pci, PCI_BASE_ADDRESS_0, 0);
	dw_pcie_writel_dbi(pci, PCI_BASE_ADDRESS_1, 0);
	ks_pcie_clear_dbi_mode(ks_pcie);

	ks_pcie_setup_mem_space(ks_pcie);
	writew(PCI_IO_RANGE_TYPE_32 | (PCI_IO_RANGE_TYPE_32 << 8),
			pci->dbi_base + PCI_IO_BASE);

	ret = ks_pcie_init_id(ks_pcie);
	if (ret < 0)
		return ret;

#ifndef CONFIG_ARM64
	/*
	 * PCIe access errors that result into OCP errors are caught by ARM as
	 * "External aborts"
	 */
	hook_fault_code(17, ks_pcie_fault, SIGBUS, 0,
			"Asynchronous external abort");
#endif

	ks_pcie_start_link(pci);
	dw_pcie_wait_for_link(pci);

	return 0;
}

static const struct dw_pcie_host_ops ks_pcie_host_ops = {
	.rd_other_conf = ks_pcie_rd_other_conf,
	.wr_other_conf = ks_pcie_wr_other_conf,
	.host_init = ks_pcie_host_init,
	.msi_set_irq = ks_pcie_msi_set_irq,
	.msi_clear_irq = ks_pcie_msi_clear_irq,
	.get_msi_addr = ks_pcie_get_msi_addr,
	.msi_host_init = ks_pcie_msi_host_init,
	.msi_irq_ack = ks_pcie_msi_irq_ack,
	.scan_bus = ks_pcie_v3_65_scan_bus,
};

static const struct dw_pcie_host_ops ks_pcie_am654_host_ops = {
	.host_init = ks_pcie_host_init,
	.msi_host_init = ks_pcie_am654_msi_host_init,
};

static irqreturn_t ks_pcie_err_irq_handler(int irq, void *priv)
{
	u32 reg;
	struct keystone_pcie *ks_pcie = priv;
	struct device *dev = ks_pcie->pci->dev;

	reg = ks_pcie_app_readl(ks_pcie, ERR_IRQ_STATUS);

	if (reg & ERR_SYS)
		dev_err(dev, "System Error\n");

	if (reg & ERR_FATAL)
		dev_err(dev, "Fatal Error\n");

	if (reg & ERR_NONFATAL)
		dev_err(dev, "Non Fatal Error\n");

	if (reg & ERR_CORR)
		dev_err(dev, "Correctable Error\n");

	if (reg & ERR_AXI)
		dev_err(dev, "AXI tag lookup fatal Error\n");

	if (reg & ERR_AER)
		dev_err(dev, "ECRC Error\n");

	ks_pcie_app_writel(ks_pcie, ERR_IRQ_STATUS, reg);

	return IRQ_HANDLED;
}

static int __init ks_pcie_add_pcie_port(struct keystone_pcie *ks_pcie,
					struct platform_device *pdev)
{
	struct dw_pcie *pci = ks_pcie->pci;
	struct pcie_port *pp = &pci->pp;
	struct device *dev = &pdev->dev;
	struct resource *res;
	int ret;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "config");
	pp->va_cfg0_base = devm_pci_remap_cfg_resource(dev, res);
	if (IS_ERR(pp->va_cfg0_base))
		return PTR_ERR(pp->va_cfg0_base);

	pp->va_cfg1_base = pp->va_cfg0_base;

	pp->root_bus_nr = -1;
	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(dev, "failed to initialize host\n");
		return ret;
	}

	return 0;
}

static void ks_pcie_raise_legacy_irq(struct keystone_pcie *ks_pcie)
{
	struct dw_pcie *pci = ks_pcie->pci;
	u8 int_pin;

	int_pin = dw_pcie_readb_dbi(pci, PCI_INTERRUPT_PIN);
	if (int_pin == 0 || int_pin > 4)
		return;

	ks_pcie_app_writel(ks_pcie, PCIE_LEGACY_IRQ_ENABLE_SET(int_pin),
			   INT_ENABLE);
	ks_pcie_app_writel(ks_pcie, PCIE_EP_IRQ_SET, INT_ENABLE);
	mdelay(1);
	ks_pcie_app_writel(ks_pcie, PCIE_EP_IRQ_CLR, INT_ENABLE);
	ks_pcie_app_writel(ks_pcie, PCIE_LEGACY_IRQ_ENABLE_CLR(int_pin),
			   INT_ENABLE);
}

static void ks_pcie_ep_init(struct dw_pcie_ep *ep)
{
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);
	struct keystone_pcie *ks_pcie = to_keystone_pcie(pci);
	int flags;
	u32 val;

	ep->page_size = WIN_SIZE;
	flags = PCI_BASE_ADDRESS_SPACE_MEMORY | PCI_BASE_ADDRESS_MEM_TYPE_32;
	dw_pcie_writel_dbi2(pci, PCI_BASE_ADDRESS_0, APP_ADDR_SPACE_0 - 1);
	dw_pcie_writel_dbi(pci, PCI_BASE_ADDRESS_0, flags);

	ks_pcie_app_writel(ks_pcie, OB_SIZE, 0x0);
	val = ks_pcie_app_readl(ks_pcie, CMD_STATUS);
	ks_pcie_app_writel(ks_pcie, CMD_STATUS, val | OB_XLAT_EN_VAL |
			   IB_XLAT_EN_VAL);
}

static void ks_pcie_am654_ep_init(struct dw_pcie_ep *ep)
{
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);
	int flags;

	ep->page_size = AM654_WIN_SIZE;
	flags = PCI_BASE_ADDRESS_SPACE_MEMORY | PCI_BASE_ADDRESS_MEM_TYPE_32;
	dw_pcie_writel_dbi2(pci, PCI_BASE_ADDRESS_0, APP_ADDR_SPACE_0 - 1);
	dw_pcie_writel_dbi(pci, PCI_BASE_ADDRESS_0, flags);
}

static int ks_pcie_raise_irq(struct dw_pcie_ep *ep, u8 func_no,
			     enum pci_epc_irq_type type,
			     u8 interrupt_num)
{
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);
	struct keystone_pcie *ks_pcie = to_keystone_pcie(pci);

	switch (type) {
	case PCI_EPC_IRQ_LEGACY:
		ks_pcie_raise_legacy_irq(ks_pcie);
		break;
	case PCI_EPC_IRQ_MSI:
		dw_pcie_ep_raise_msi_irq(ep, func_no, interrupt_num);
		break;
	default:
		dev_err(pci->dev, "UNKNOWN IRQ type\n");
		return -EINVAL;
	}

	return 0;
}

static const struct dw_pcie_ep_ops ks_pcie_ep_ops = {
	.ep_init = ks_pcie_ep_init,
	.raise_irq = ks_pcie_raise_irq,
};

static int ks_pcie_am654_set_bar(struct dw_pcie_ep *ep, u8 func_no,
				 struct pci_epf_bar *epf_bar)
{
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);
	struct keystone_pcie *ks_pcie = to_keystone_pcie(pci);
	enum pci_barno bar = epf_bar->barno;
	u32 reg = PCI_BASE_ADDRESS_0 + (4 * bar);
	struct device *dev = pci->dev;
	enum dw_pcie_as_type as_type;
	size_t size = epf_bar->size;
	int flags = epf_bar->flags;
	int ret;

	if (bar == BAR_0) {
		epf_bar->flags |= PCI_BASE_ADDRESS_MEM_TYPE_64;
		dev_err(dev, "BAR_0 is reserved\n");
		return -EINVAL;
	}

	if (bar_size[bar] != size) {
		dev_info(dev, "BAR%d: Requested size %lld not supported, changed to %lld\n",
			 bar, (long long)size, (long long)bar_size[bar]);
	}

	epf_bar->size = bar_size[bar];

	if (!(flags & PCI_BASE_ADDRESS_SPACE))
		as_type = DW_PCIE_AS_MEM;
	else
		as_type = DW_PCIE_AS_IO;

	ret = dw_pcie_ep_inbound_atu(ep, bar, epf_bar->phys_addr, as_type);
	if (ret)
		return ret;

	ks_pcie_set_dbi_mode(ks_pcie);
	dw_pcie_writel_dbi(pci, reg, lower_32_bits(size - 1));
	if (flags & PCI_BASE_ADDRESS_MEM_TYPE_64)
		dw_pcie_writel_dbi(pci, reg + 4, upper_32_bits(size - 1));
	ks_pcie_clear_dbi_mode(ks_pcie);

	return 0;
}

static const struct dw_pcie_ep_ops ks_pcie_am654_ep_ops = {
	.ep_init = ks_pcie_am654_ep_init,
	.raise_irq = ks_pcie_raise_irq,
	.set_bar = ks_pcie_am654_set_bar,
};

static int __init ks_pcie_add_pcie_ep(struct keystone_pcie *ks_pcie,
				      struct platform_device *pdev)
{
	int ret;
	struct dw_pcie_ep *ep;
	struct resource *res;
	struct device *dev = &pdev->dev;
	struct dw_pcie *pci = ks_pcie->pci;

	ep = &pci->ep;

	if (ks_pcie->num_ob_windows > MAX_OB_WINDOWS)
		return -EINVAL;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "addr_space");
	if (!res)
		return -EINVAL;

	ep->phys_base = res->start;
	ep->addr_size = resource_size(res);

	ks_pcie->ob_win = devm_kzalloc(dev, sizeof(*ks_pcie->ob_win) *
				       ks_pcie->num_ob_windows, GFP_KERNEL);
	if (!ks_pcie->ob_win)
		return -ENOMEM;

	ret = dw_pcie_ep_init(ep);
	if (ret) {
		dev_err(dev, "failed to initialize endpoint\n");
		return ret;
	}

	return 0;
}

static void ks_pcie_disable_outbound_atu(struct keystone_pcie *ks_pcie,
					 phys_addr_t addr)
{
	u8 index;
	u8 regions;
	u64 cpu_addr;

	index = (addr >> WIN_INDEX_SHIFT) & WIN_INDEX_MASK;
	regions = ks_pcie->ob_win[index].no_of_regions;
	cpu_addr = ks_pcie->ob_win[index].cpu_addr;

	WARN_ON(cpu_addr != addr);

	while (regions--) {
		ks_pcie_app_writel(ks_pcie, OB_OFFSET_INDEX(index), 0x0);
		clear_bit(index++, &ks_pcie->ob_window_map);
	}
}

static void ks_pcie_disable_inbound_atu(struct keystone_pcie *ks_pcie,
					int index)
{
	ks_pcie_app_writel(ks_pcie, IB_BAR(index), 0x0);
	ks_pcie_app_writel(ks_pcie, IB_START_LO(index), 0x0);
	ks_pcie_app_writel(ks_pcie, IB_START_HI(index), 0x0);
	ks_pcie_app_writel(ks_pcie, IB_OFFSET(index), 0x0);
}

static void ks_pcie_disable_atu(struct dw_pcie *pci, phys_addr_t addr,
				int index, enum dw_pcie_region_type type)
{
	struct keystone_pcie *ks_pcie = to_keystone_pcie(pci);

	switch (type) {
	case DW_PCIE_REGION_INBOUND:
		ks_pcie_disable_inbound_atu(ks_pcie, index);
		break;
	case DW_PCIE_REGION_OUTBOUND:
		ks_pcie_disable_outbound_atu(ks_pcie, addr);
		break;
	default:
		return;
	}
}

static int ks_pcie_inbound_atu(struct dw_pcie *pci, u32 index,
			       enum pci_barno bar, dma_addr_t cpu_addr)
{
	struct keystone_pcie *ks_pcie = to_keystone_pcie(pci);
	struct device *dev = pci->dev;

	if (bar == BAR_0) {
		dev_err(dev, "BAR_0 is reserved\n");
		return -EINVAL;
	}

	ks_pcie_app_writel(ks_pcie, IB_BAR(index), bar);
	ks_pcie_app_writel(ks_pcie, IB_OFFSET(index), lower_32_bits(cpu_addr));

	return 0;
}

static int ks_pcie_check_free(struct keystone_pcie *ks_pcie, u8 index,
			      u8 regions)
{
	while (regions--) {
		if (test_bit(index++, &ks_pcie->ob_window_map))
			return false;
	}

	return true;
}

static int ks_pcie_outbound_atu(struct dw_pcie *pci, u64 cpu_addr, u64 pci_addr,
				size_t size)
{
	u8 index;
	u8 regions;
	struct keystone_pcie *ks_pcie = to_keystone_pcie(pci);

	index = (cpu_addr >> WIN_INDEX_SHIFT) & WIN_INDEX_MASK;
	regions = ((size - 1) >> WIN_INDEX_SHIFT) + 1;

	if (!ks_pcie_check_free(ks_pcie, index, regions))
		return -ENOMEM;
	if (index + regions > ks_pcie->num_ob_windows)
		return -ENOMEM;

	ks_pcie->ob_win[index].cpu_addr = cpu_addr;
	ks_pcie->ob_win[index].no_of_regions = regions;

	while (regions--) {
		ks_pcie_app_writel(ks_pcie, OB_OFFSET_INDEX(index),
				   lower_32_bits(pci_addr) | OB_ENABLEN);
		ks_pcie_app_writel(ks_pcie, OB_OFFSET_HI(index),
				   upper_32_bits(pci_addr));
		set_bit(index++, &ks_pcie->ob_window_map);
		pci_addr += WIN_SIZE;
	}

	return 0;
}

static void ks_pcie_stop_link(struct dw_pcie *pci)
{
	u32 val;
	struct keystone_pcie *ks_pcie = to_keystone_pcie(pci);

	/* Disable Link training */
	val = ks_pcie_app_readl(ks_pcie, CMD_STATUS);
	val &= ~LTSSM_EN_VAL;
	ks_pcie_app_writel(ks_pcie, CMD_STATUS, LTSSM_EN_VAL | val);
}

static int ks_pcie_start_link(struct dw_pcie *pci)
{
	u32 val;
	struct device *dev = pci->dev;
	struct keystone_pcie *ks_pcie = to_keystone_pcie(pci);

	if (dw_pcie_link_up(pci)) {
		dev_WARN(dev, "Link already up\n");
		return 0;
	}

	/* Initiate Link Training */
	val = ks_pcie_app_readl(ks_pcie, CMD_STATUS);
	ks_pcie_app_writel(ks_pcie, CMD_STATUS, LTSSM_EN_VAL | val);

	return 0;
}

static const struct dw_pcie_ops ks_pcie_dw_pcie_ops = {
	.start_link = ks_pcie_start_link,
	.stop_link = ks_pcie_stop_link,
	.link_up = ks_pcie_link_up,
	.read_dbi2 = ks_pcie_read_dbi2,
	.write_dbi2 = ks_pcie_write_dbi2,
	.inbound_atu = ks_pcie_inbound_atu,
	.outbound_atu = ks_pcie_outbound_atu,
	.disable_atu = ks_pcie_disable_atu,
};

static const struct dw_pcie_ops ks_pcie_am654_pcie_ops = {
	.start_link = ks_pcie_start_link,
	.stop_link = ks_pcie_stop_link,
	.link_up = ks_pcie_link_up,
	.read_dbi2 = ks_pcie_read_dbi2,
	.write_dbi2 = ks_pcie_write_dbi2,
};

static void ks_pcie_disable_phy(struct keystone_pcie *ks_pcie)
{
	int num_lanes = ks_pcie->num_lanes;

	while (num_lanes--) {
		phy_power_off(ks_pcie->phy[num_lanes]);
		phy_exit(ks_pcie->phy[num_lanes]);
	}
}

static int ks_pcie_enable_phy(struct keystone_pcie *ks_pcie)
{
	int i;
	int ret;
	int num_lanes = ks_pcie->num_lanes;

	for (i = 0; i < num_lanes; i++) {
		ret = phy_reset(ks_pcie->phy[i]);
		if (ret < 0)
			goto err_phy;

		ret = phy_init(ks_pcie->phy[i]);
		if (ret < 0)
			goto err_phy;

		ret = phy_power_on(ks_pcie->phy[i]);
		if (ret < 0) {
			phy_exit(ks_pcie->phy[i]);
			goto err_phy;
		}
	}

	return 0;

err_phy:
	while (--i >= 0) {
		phy_power_off(ks_pcie->phy[i]);
		phy_exit(ks_pcie->phy[i]);
	}

	return ret;
}

static int ks_pcie_set_mode(struct device *dev, enum dw_pcie_device_mode mode)
{
	struct device_node *np = dev->of_node;
	struct regmap *syscon;
	u32 val;
	u32 mask;
	int ret = 0;

	syscon = syscon_regmap_lookup_by_phandle(np, "ti,syscon-pcie-mode");
	if (IS_ERR(syscon))
		return 0;

	mask = KS_PCIE_DEV_TYPE_MASK | KS_PCIE_SYSCLOCKOUTEN;

	switch (mode) {
	case DW_PCIE_RC_TYPE:
		val = KS_PCIE_DEV_TYPE(RC) | KS_PCIE_SYSCLOCKOUTEN;
		break;
	case DW_PCIE_EP_TYPE:
		val = KS_PCIE_DEV_TYPE(EP);
		break;
	default:
		dev_err(dev, "INVALID device type %d\n", mode);
		return -EINVAL;
	}

	ret = regmap_update_bits(syscon, 0, mask, val);
	if (ret) {
		dev_err(dev, "failed to set pcie mode\n");
		return ret;
	}

	return 0;
}

static int ks_pcie_am654_set_mode(struct device *dev,
				  enum dw_pcie_device_mode mode)
{
	struct device_node *np = dev->of_node;
	struct regmap *syscon;
	u32 val;
	u32 mask;
	int ret = 0;

	syscon = syscon_regmap_lookup_by_phandle(np, "ti,syscon-pcie-mode");
	if (IS_ERR(syscon))
		return 0;

	mask = AM654_PCIE_DEV_TYPE_MASK;

	switch (mode) {
	case DW_PCIE_RC_TYPE:
		val = RC;
		break;
	case DW_PCIE_EP_TYPE:
		val = EP;
		break;
	default:
		dev_err(dev, "INVALID device type %d\n", mode);
		return -EINVAL;
	}

	ret = regmap_update_bits(syscon, 0, mask, val);
	if (ret) {
		dev_err(dev, "failed to set pcie mode\n");
		return ret;
	}

	return 0;
}

static void ks_pcie_set_link_speed(struct dw_pcie *pci, int link_speed)
{
	u32 val;

	dw_pcie_dbi_ro_wr_en(pci);

	val = dw_pcie_readl_dbi(pci, EXP_CAP_ID_OFFSET + PCI_EXP_LNKCAP);
	if ((val & PCI_EXP_LNKCAP_SLS) != link_speed) {
		val &= ~((u32)PCI_EXP_LNKCAP_SLS);
		val |= link_speed;
		dw_pcie_writel_dbi(pci, EXP_CAP_ID_OFFSET + PCI_EXP_LNKCAP,
				   val);
	}

	val = dw_pcie_readl_dbi(pci, EXP_CAP_ID_OFFSET + PCI_EXP_LNKCTL2);
	if ((val & PCI_EXP_LNKCAP_SLS) != link_speed) {
		val &= ~((u32)PCI_EXP_LNKCAP_SLS);
		val |= link_speed;
		dw_pcie_writel_dbi(pci, EXP_CAP_ID_OFFSET + PCI_EXP_LNKCTL2,
				   val);
	}

	dw_pcie_dbi_ro_wr_dis(pci);
}

static const struct ks_pcie_of_data ks_pcie_rc_of_data = {
	.ops = &ks_pcie_dw_pcie_ops,
	.host_ops = &ks_pcie_host_ops,
	.mode = DW_PCIE_RC_TYPE,
	.version = 0x365A,
};

static const struct ks_pcie_of_data ks_pcie_ep_of_data = {
	.ops = &ks_pcie_dw_pcie_ops,
	.ep_ops = &ks_pcie_ep_ops,
	.mode = DW_PCIE_EP_TYPE,
	.version = 0x365A,
};

static const struct ks_pcie_of_data ks_pcie_am654_rc_of_data = {
	.ops = &ks_pcie_am654_pcie_ops,
	.host_ops = &ks_pcie_am654_host_ops,
	.mode = DW_PCIE_RC_TYPE,
	.version = 0x490A,
};

static const struct ks_pcie_of_data ks_pcie_am654_ep_of_data = {
	.ops = &ks_pcie_am654_pcie_ops,
	.ep_ops = &ks_pcie_am654_ep_ops,
	.mode = DW_PCIE_EP_TYPE,
	.version = 0x490A,
};

static const struct of_device_id ks_pcie_of_match[] = {
	{
		.type = "pci",
		.data = &ks_pcie_rc_of_data,
		.compatible = "ti,keystone-pcie",
	},
	{
		.data = &ks_pcie_ep_of_data,
		.compatible = "ti,keystone-pcie-ep",
	},
	{
		.data = &ks_pcie_am654_rc_of_data,
		.compatible = "ti,am654-pcie-rc",
	},
	{
		.data = &ks_pcie_am654_ep_of_data,
		.compatible = "ti,am654-pcie-ep",
	},
	{ },
};

static int __init ks_pcie_probe(struct platform_device *pdev)
{
	enum dw_pcie_device_mode mode = DW_PCIE_RC_TYPE;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	const struct ks_pcie_of_data *data;
	const struct of_device_id *match;
	struct dw_pcie *pci;
	struct keystone_pcie *ks_pcie;
	struct device_link **link;
	struct gpio_desc *gpiod;
	void __iomem *atu_base;
	struct resource *res;
	void __iomem *base;
	u32 num_ob_windows;
	struct phy **phy;
	int link_speed;
	u32 num_lanes;
	char name[10];
	int ret;
	int irq;
	int i;

	match = of_match_device(of_match_ptr(ks_pcie_of_match), dev);
	data = (struct ks_pcie_of_data *)match->data;
	if (data)
		mode = (enum dw_pcie_device_mode)data->mode;

	ks_pcie = devm_kzalloc(dev, sizeof(*ks_pcie), GFP_KERNEL);
	if (!ks_pcie)
		return -ENOMEM;

	pci = devm_kzalloc(dev, sizeof(*pci), GFP_KERNEL);
	if (!pci)
		return -ENOMEM;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbics");
	base = devm_ioremap_resource(dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	pci->dbi_base = base;
	pci->dbi_base2 = base;
	pci->dev = dev;
	pci->ops = data->ops;
	pci->version = data->version;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "app");
	ks_pcie->va_app_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(ks_pcie->va_app_base))
		return PTR_ERR(ks_pcie->va_app_base);

	if ((dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(48)) != 0) &&
	    dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32) != 0)) {
		dev_err(dev, "Cannot set DMA mask\n");
		return -EINVAL;
	}

	ks_pcie->app = *res;

	ret = of_property_read_u32(np, "num-lanes", &num_lanes);
	if (ret)
		num_lanes = 1;

	phy = devm_kzalloc(dev, sizeof(*phy) * num_lanes, GFP_KERNEL);
	if (!phy)
		return -ENOMEM;

	link = devm_kzalloc(dev, sizeof(*link) * num_lanes, GFP_KERNEL);
	if (!link)
		return -ENOMEM;

	for (i = 0; i < num_lanes; i++) {
		snprintf(name, sizeof(name), "pcie-phy%d", i);
		phy[i] = devm_phy_optional_get(dev, name);
		if (IS_ERR(phy[i])) {
			ret = PTR_ERR(phy[i]);
			goto err_link;
		}

		if (!phy[i])
			continue;

		link[i] = device_link_add(dev, &phy[i]->dev, DL_FLAG_STATELESS);
		if (!link[i]) {
			ret = -EINVAL;
			goto err_link;
		}
	}

	ks_pcie->num_lanes = num_lanes;
	ks_pcie->phy = phy;

	gpiod = devm_gpiod_get_optional(dev, "reset",
					GPIOD_OUT_LOW);
	if (IS_ERR(gpiod)) {
		ret = PTR_ERR(gpiod);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to get reset GPIO\n");
		goto err_link;
	}

	ret = ks_pcie_enable_phy(ks_pcie);
	if (ret) {
		dev_err(dev, "failed to enable phy\n");
		goto err_link;
	}

	ret = of_property_read_u32(np, "num-ob-windows", &num_ob_windows);
	if (ret < 0) {
		dev_err(dev, "unable to read *num-ob-windows* property\n");
		return ret;
	}

	ks_pcie->pci = pci;
	ks_pcie->link = link;
	ks_pcie->np = np;
	ks_pcie->num_ob_windows = num_ob_windows;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "missing IRQ resource: %d\n", irq);
		ret = irq;
		goto err_get_irq;
	}

	ret = devm_request_irq(dev, irq, ks_pcie_err_irq_handler, IRQF_SHARED,
			       "ks-pcie-error-irq", ks_pcie);
	if (ret < 0) {
		dev_err(dev, "failed to request error IRQ %d\n", irq);
		goto err_get_irq;
	}

	platform_set_drvdata(pdev, ks_pcie);

	pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		dev_err(dev, "pm_runtime_get_sync failed\n");
		goto err_get_sync;
	}

	if (pci->version >= 0x480A) {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "atu");
		atu_base = devm_ioremap_resource(dev, res);
		if (IS_ERR(atu_base))
			return PTR_ERR(atu_base);

		pci->atu_base = atu_base;

		ret = ks_pcie_am654_set_mode(dev, mode);
		if (ret < 0)
			goto err_get_sync;
	} else {
		ret = ks_pcie_set_mode(dev, mode);
		if (ret < 0)
			goto err_get_sync;
	}

	link_speed = of_pci_get_max_link_speed(np);
	if (link_speed < 0)
		link_speed = 2;

	ks_pcie_set_link_speed(pci, link_speed);

	switch (mode) {
	case DW_PCIE_RC_TYPE:
		if (!IS_ENABLED(CONFIG_PCI_KEYSTONE_HOST)) {
			ret = -ENODEV;
			goto err_get_sync;
		}

		/*
		 * "Power Sequencing and Reset Signal Timings" table in
		 * PCI EXPRESS CARD ELECTROMECHANICAL SPECIFICATION, REV. 2.0
		 * indicates PERST# should be deasserted after minimum of 100us
		 * once REFCLK is stable. The REFCLK to the connector in RC
		 * mode is selected while enabling the PHY. So deassert PERST#
		 * after 100 us.
		 */
		if (gpiod) {
			usleep_range(100, 200);
			gpiod_set_value_cansleep(gpiod, 1);
		}

		pci->pp.ops = data->host_ops;
		ret = ks_pcie_add_pcie_port(ks_pcie, pdev);
		if (ret < 0)
			goto err_get_sync;
		break;
	case DW_PCIE_EP_TYPE:
		if (!IS_ENABLED(CONFIG_PCI_KEYSTONE_EP)) {
			ret = -ENODEV;
			goto err_get_sync;
		}

		pci->ep.ops = data->ep_ops;
		ret = ks_pcie_add_pcie_ep(ks_pcie, pdev);
		if (ret < 0)
			goto err_get_sync;
		break;
	default:
		dev_err(dev, "INVALID device type %d\n", mode);
	}

	ks_pcie_enable_error_irq(ks_pcie);

	return 0;

err_get_sync:
	pm_runtime_put(dev);
	pm_runtime_disable(dev);

err_get_irq:
	ks_pcie_disable_phy(ks_pcie);

err_link:
	while (--i >= 0 && link[i])
		device_link_del(link[i]);

	return ret;
}

static struct platform_driver ks_pcie_driver __refdata = {
	.probe  = ks_pcie_probe,
	.driver = {
		.name	= "keystone-pcie",
		.of_match_table = of_match_ptr(ks_pcie_of_match),
	},
};
builtin_platform_driver(ks_pcie_driver);
