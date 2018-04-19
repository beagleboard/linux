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

#include <linux/irqchip/chained_irq.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irqdomain.h>
#include <linux/init.h>
#include <linux/mfd/syscon.h>
#include <linux/msi.h>
#include <linux/of_irq.h>
#include <linux/of.h>
#include <linux/of_pci.h>
#include <linux/platform_device.h>
#include <linux/phy/phy.h>
#include <linux/regmap.h>
#include <linux/resource.h>
#include <linux/signal.h>

#include "pcie-designware.h"

#define DRIVER_NAME	"keystone-pcie"

#define PCIE_VENDORID_MASK	0xffff
#define PCIE_DEVICEID_SHIFT	16

/* DEV_STAT_CTRL */
#define PCIE_CAP_BASE		0x70

/* Application register defines */
#define LTSSM_EN_VAL		        BIT(0)
#define LTSSM_STATE_MASK		0x1f
#define LTSSM_STATE_L0			0x11
#define DBI_CS2_EN_VAL			0x20
#define OB_XLAT_EN_VAL		        2

/* Application registers */
#define CMD_STATUS			0x004
#define CFG_SETUP			0x008
#define OB_SIZE				0x030
#define CFG_PCIM_WIN_SZ_IDX		3
#define CFG_PCIM_WIN_CNT		32
#define SPACE0_REMOTE_CFG_OFFSET	0x1000
#define OB_OFFSET_INDEX(n)		(0x200 + (8 * n))
#define OB_OFFSET_HI(n)			(0x204 + (8 * n))

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

/* Error IRQ bits */
#define ERR_AER		BIT(5)	/* ECRC error */
#define ERR_AXI		BIT(4)	/* AXI tag lookup fatal error */
#define ERR_CORR	BIT(3)	/* Correctable error */
#define ERR_NONFATAL	BIT(2)	/* Non-fatal error */
#define ERR_FATAL	BIT(1)	/* Fatal error */
#define ERR_SYS		BIT(0)	/* System (fatal, non-fatal, or correctable) */
#define ERR_IRQ_ALL	(ERR_AER | ERR_AXI | ERR_CORR | \
			 ERR_NONFATAL | ERR_FATAL | ERR_SYS)
#define ERR_FATAL_IRQ	(ERR_FATAL | ERR_AXI)
#define ERR_IRQ_STATUS_RAW		0x1c0
#define ERR_IRQ_STATUS			0x1c4
#define ERR_IRQ_ENABLE_SET		0x1c8
#define ERR_IRQ_ENABLE_CLR		0x1cc

/* Config space registers */
#define DEBUG0				0x728

#define to_keystone_pcie(x)	dev_get_drvdata((x)->dev)

static int ks_pcie_start_link(struct dw_pcie *pci);
static void ks_pcie_stop_link(struct dw_pcie *pci);

struct keystone_pcie {
	struct dw_pcie		*pci;
	struct	clk		*clk;
	int			msi_host_irq;
	struct			device_node *msi_intc_np;
	struct irq_domain	*legacy_irq_domain;
	struct device_node	*np;

	/* Application register space */
	void __iomem		*va_app_base;	/* DT 1st resource */
	struct resource		app;
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

void ks_pcie_enable_error_irq(struct keystone_pcie *ks_pcie)
{
	ks_pcie_app_writel(ks_pcie, ERR_IRQ_ENABLE_SET, ERR_IRQ_ALL);
}

static irqreturn_t ks_pcie_handle_error_irq(struct keystone_pcie *ks_pcie)
{
	u32 status;

	status = ks_pcie_app_readl(ks_pcie, ERR_IRQ_STATUS_RAW) & ERR_IRQ_ALL;
	if (!status)
		return IRQ_NONE;

	if (status & ERR_FATAL_IRQ)
		dev_err(ks_pcie->pci->dev, "fatal error (status %#010x)\n",
			status);

	/* Ack the IRQ; status bits are RW1C */
	ks_pcie_app_writel(ks_pcie, ERR_IRQ_STATUS, status);
	return IRQ_HANDLED;
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
	ks_pcie_app_writel(ks_pcie, CMD_STATUS, DBI_CS2_EN_VAL | val);

	do {
		val = ks_pcie_app_readl(ks_pcie, CMD_STATUS);
	} while (!(val & DBI_CS2_EN_VAL));
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
	ks_pcie_app_writel(ks_pcie, CMD_STATUS, ~DBI_CS2_EN_VAL & val);

	do {
		val = ks_pcie_app_readl(ks_pcie, CMD_STATUS);
	} while (val & DBI_CS2_EN_VAL);
}

static void ks_pcie_setup_rc_app_regs(struct keystone_pcie *ks_pcie)
{
	struct dw_pcie *pci = ks_pcie->pci;
	struct pcie_port *pp = &pci->pp;
	u32 start = pp->mem->start, end = pp->mem->end;
	int i, tr_size;
	u32 val;

	/* Disable BARs for inbound access */
	ks_pcie_set_dbi_mode(ks_pcie);
	dw_pcie_writel_dbi(pci, PCI_BASE_ADDRESS_0, 0);
	dw_pcie_writel_dbi(pci, PCI_BASE_ADDRESS_1, 0);
	ks_pcie_clear_dbi_mode(ks_pcie);

	/* Set outbound translation size per window division */
	ks_pcie_app_writel(ks_pcie, OB_SIZE, CFG_PCIM_WIN_SZ_IDX & 0x7);

	tr_size = (1 << (CFG_PCIM_WIN_SZ_IDX & 0x7)) * SZ_1M;

	/* Using Direct 1:1 mapping of RC <-> PCI memory space */
	for (i = 0; (i < CFG_PCIM_WIN_CNT) && (start < end); i++) {
		ks_pcie_app_writel(ks_pcie, OB_OFFSET_INDEX(i), start | 1);
		ks_pcie_app_writel(ks_pcie, OB_OFFSET_HI(i), 0);
		start += tr_size;
	}

	/* Enable OB translation */
	val = ks_pcie_app_readl(ks_pcie, CMD_STATUS);
	ks_pcie_app_writel(ks_pcie, CMD_STATUS, OB_XLAT_EN_VAL | val);
}

/**
 * ks_pcie_cfg_setup() - Set up configuration space address for a device
 *
 * @ks_pcie: ptr to keystone_pcie structure
 * @bus: Bus number the device is residing on
 * @devfn: device, function number info
 *
 * Forms and returns the address of configuration space mapped in PCIESS
 * address space 0.  Also configures CFG_SETUP for remote configuration space
 * access.
 *
 * The address space has two regions to access configuration - local and remote.
 * We access local region for bus 0 (as RC is attached on bus 0) and remote
 * region for others with TYPE 1 access when bus > 1.  As for device on bus = 1,
 * we will do TYPE 0 access as it will be on our secondary bus (logical).
 * CFG_SETUP is needed only for remote configuration access.
 */
static void __iomem *ks_pcie_cfg_setup(struct keystone_pcie *ks_pcie, u8 bus,
				       unsigned int devfn)
{
	u8 device = PCI_SLOT(devfn), function = PCI_FUNC(devfn);
	struct dw_pcie *pci = ks_pcie->pci;
	struct pcie_port *pp = &pci->pp;
	u32 regval;

	if (bus == 0)
		return pci->dbi_base;

	regval = (bus << 16) | (device << 8) | function;

	/*
	 * Since Bus#1 will be a virtual bus, we need to have TYPE0
	 * access only.
	 * TYPE 1
	 */
	if (bus != 1)
		regval |= BIT(24);

	ks_pcie_app_writel(ks_pcie, CFG_SETUP, regval);
	return pp->va_cfg0_base;
}

static int ks_pcie_rd_other_conf(struct pcie_port *pp, struct pci_bus *bus,
				 unsigned int devfn, int where, int size,
				 u32 *val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct keystone_pcie *ks_pcie = to_keystone_pcie(pci);
	u8 bus_num = bus->number;
	void __iomem *addr;

	addr = ks_pcie_cfg_setup(ks_pcie, bus_num, devfn);

	return dw_pcie_read(addr + where, size, val);
}

static int ks_pcie_wr_other_conf(struct pcie_port *pp, struct pci_bus *bus,
				 unsigned int devfn, int where, int size,
				 u32 val)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct keystone_pcie *ks_pcie = to_keystone_pcie(pci);
	u8 bus_num = bus->number;
	void __iomem *addr;

	addr = ks_pcie_cfg_setup(ks_pcie, bus_num, devfn);

	return dw_pcie_write(addr + where, size, val);
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

	val = dw_pcie_readl_dbi(pci, DEBUG0);
	return (val & LTSSM_STATE_MASK) == LTSSM_STATE_L0;
}

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
		dev_WARN(dev, "msi-interrupt-controller node is absent\n");
		return 0;
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

static int __init ks_pcie_init_id(struct keystone_pcie *ks_pcie)
{
	int ret;
	u32 index;
	unsigned int id;
	struct regmap *devctrl_regs;
	struct dw_pcie *pci = ks_pcie->pci;
	struct device *dev = pci->dev;
	struct device_node *np = dev->of_node;

	devctrl_regs = syscon_regmap_lookup_by_phandle(np, "ti,syscon-dev");
	if (IS_ERR(devctrl_regs))
		return PTR_ERR(devctrl_regs);

	ret = of_property_read_u32_index(np, "ti,syscon-dev", 1, &index);
	if (ret)
		return ret;

	ret = regmap_read(devctrl_regs, index, &id);
	if (ret)
		return ret;

	dw_pcie_writew_dbi(pci, PCI_VENDOR_ID, id & PCIE_VENDORID_MASK);
	dw_pcie_writew_dbi(pci, PCI_DEVICE_ID, id >> PCIE_DEVICEID_SHIFT);

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

	ks_pcie_setup_rc_app_regs(ks_pcie);
	writew(PCI_IO_RANGE_TYPE_32 | (PCI_IO_RANGE_TYPE_32 << 8),
			pci->dbi_base + PCI_IO_BASE);

	ret = ks_pcie_init_id(ks_pcie);
	if (ret < 0)
		return ret;

	/*
	 * PCIe access errors that result into OCP errors are caught by ARM as
	 * "External aborts"
	 */
	hook_fault_code(17, ks_pcie_fault, SIGBUS, 0,
			"Asynchronous external abort");

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

static irqreturn_t ks_pcie_err_irq_handler(int irq, void *priv)
{
	struct keystone_pcie *ks_pcie = priv;

	return ks_pcie_handle_error_irq(ks_pcie);
}

static int __init ks_pcie_add_pcie_port(struct keystone_pcie *ks_pcie,
					struct platform_device *pdev)
{
	struct dw_pcie *pci = ks_pcie->pci;
	struct pcie_port *pp = &pci->pp;
	struct device *dev = &pdev->dev;
	struct resource *res;
	int ret;

	/* Index 0 is the config reg. space address */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pci->dbi_base = devm_pci_remap_cfg_resource(dev, res);
	if (IS_ERR(pci->dbi_base))
		return PTR_ERR(pci->dbi_base);

	/*
	 * We set these same and is used in pcie rd/wr_other_conf
	 * functions
	 */
	pp->va_cfg0_base = pci->dbi_base + SPACE0_REMOTE_CFG_OFFSET;
	pp->va_cfg1_base = pp->va_cfg0_base;

	/* Index 1 is the application reg. space address */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	ks_pcie->va_app_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(ks_pcie->va_app_base))
		return PTR_ERR(ks_pcie->va_app_base);

	ks_pcie->app = *res;

	pp->root_bus_nr = -1;
	pp->ops = &ks_pcie_host_ops;
	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(dev, "failed to initialize host\n");
		return ret;
	}

	return 0;
}

static const struct of_device_id ks_pcie_of_match[] = {
	{
		.type = "pci",
		.compatible = "ti,keystone-pcie",
	},
	{ },
};

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
};

static int __init ks_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dw_pcie *pci;
	struct keystone_pcie *ks_pcie;
	struct phy *phy;
	int ret;
	int irq;

	ks_pcie = devm_kzalloc(dev, sizeof(*ks_pcie), GFP_KERNEL);
	if (!ks_pcie)
		return -ENOMEM;

	pci = devm_kzalloc(dev, sizeof(*pci), GFP_KERNEL);
	if (!pci)
		return -ENOMEM;

	pci->dev = dev;
	pci->ops = &ks_pcie_dw_pcie_ops;

	ks_pcie->pci = pci;

	/* initialize SerDes Phy if present */
	phy = devm_phy_get(dev, "pcie-phy");
	if (PTR_ERR_OR_ZERO(phy) == -EPROBE_DEFER)
		return PTR_ERR(phy);

	if (!IS_ERR_OR_NULL(phy)) {
		ret = phy_init(phy);
		if (ret < 0)
			return ret;
	}

	ks_pcie->np = dev->of_node;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(dev, "missing IRQ resource: %d\n", irq);
		return irq;
	}

	ret = devm_request_irq(dev, irq, ks_pcie_err_irq_handler, IRQF_SHARED,
			       "ks-pcie-error-irq", ks_pcie);
	if (ret < 0) {
		dev_err(dev, "failed to request error IRQ %d\n", irq);
		return ret;
	}

	platform_set_drvdata(pdev, ks_pcie);
	ks_pcie->clk = devm_clk_get(dev, "pcie");
	if (IS_ERR(ks_pcie->clk)) {
		dev_err(dev, "Failed to get pcie rc clock\n");
		return PTR_ERR(ks_pcie->clk);
	}
	ret = clk_prepare_enable(ks_pcie->clk);
	if (ret)
		return ret;

	ret = ks_pcie_add_pcie_port(ks_pcie, pdev);
	if (ret < 0)
		goto fail_clk;

	ks_pcie_enable_error_irq(ks_pcie);

	return 0;
fail_clk:
	clk_disable_unprepare(ks_pcie->clk);

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
