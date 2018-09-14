/**
 * Synopsys DesignWare PCIe Endpoint controller driver
 *
 * Copyright (C) 2017 Texas Instruments
 * Author: Kishon Vijay Abraham I <kishon@ti.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 of
 * the License as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/of.h>

#include "pcie-designware.h"
#include <linux/pci-epc.h>
#include <linux/pci-epf.h>

void dw_pcie_ep_linkup(struct dw_pcie_ep *ep)
{
	struct pci_epc *epc = ep->epc;

	pci_epc_linkup(epc);
}

static void __dw_pcie_ep_reset_bar(struct dw_pcie *pci, enum pci_barno bar,
				   int flags)
{
	u32 reg;

	reg = PCI_BASE_ADDRESS_0 + (4 * bar);
	dw_pcie_dbi_ro_wr_en(pci);
	dw_pcie_writel_dbi2(pci, reg, 0x0);
	dw_pcie_writel_dbi(pci, reg, 0x0);
	if (flags & PCI_BASE_ADDRESS_MEM_TYPE_64) {
		dw_pcie_writel_dbi2(pci, reg + 4, 0x0);
		dw_pcie_writel_dbi(pci, reg + 4, 0x0);
	}
	dw_pcie_dbi_ro_wr_dis(pci);
}

void dw_pcie_ep_reset_bar(struct dw_pcie *pci, enum pci_barno bar)
{
	__dw_pcie_ep_reset_bar(pci, bar, 0);
}

static int dw_pcie_ep_data_transfer(struct pci_epc *epc, struct pci_epf *epf,
				    dma_addr_t dma_dst, dma_addr_t dma_src,
				    size_t len)
{
	return pci_epf_data_transfer(epf, dma_dst, dma_src, len);
}

static int dw_pcie_ep_epf_init(struct pci_epc *epc, struct pci_epf *epf)
{
	return pci_epf_init_dma_chan(epf);
}

static void dw_pcie_ep_epf_exit(struct pci_epc *epc, struct pci_epf *epf)
{
	pci_epf_clean_dma_chan(epf);
}

static int dw_pcie_ep_write_header(struct pci_epc *epc, u8 func_no,
				   struct pci_epf_header *hdr)
{
	struct dw_pcie_ep *ep = epc_get_drvdata(epc);
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);

	dw_pcie_dbi_ro_wr_en(pci);
	dw_pcie_writew_dbi(pci, PCI_VENDOR_ID, hdr->vendorid);
	dw_pcie_writew_dbi(pci, PCI_DEVICE_ID, hdr->deviceid);
	dw_pcie_writeb_dbi(pci, PCI_REVISION_ID, hdr->revid);
	dw_pcie_writeb_dbi(pci, PCI_CLASS_PROG, hdr->progif_code);
	dw_pcie_writew_dbi(pci, PCI_CLASS_DEVICE,
			   hdr->subclass_code | hdr->baseclass_code << 8);
	dw_pcie_writeb_dbi(pci, PCI_CACHE_LINE_SIZE,
			   hdr->cache_line_size);
	dw_pcie_writew_dbi(pci, PCI_SUBSYSTEM_VENDOR_ID,
			   hdr->subsys_vendor_id);
	dw_pcie_writew_dbi(pci, PCI_SUBSYSTEM_ID, hdr->subsys_id);
	dw_pcie_writeb_dbi(pci, PCI_INTERRUPT_PIN,
			   hdr->interrupt_pin);
	dw_pcie_dbi_ro_wr_dis(pci);

	return 0;
}

int dw_pcie_ep_inbound_atu(struct dw_pcie_ep *ep, enum pci_barno bar,
			   dma_addr_t cpu_addr, enum dw_pcie_as_type as_type)
{
	int ret;
	u32 free_win;
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);

	free_win = find_first_zero_bit(ep->ib_window_map, ep->num_ib_windows);
	if (free_win >= ep->num_ib_windows) {
		dev_err(pci->dev, "no free inbound window\n");
		return -EINVAL;
	}

	if (pci->ops->inbound_atu) {
		ret = pci->ops->inbound_atu(pci, free_win, bar, cpu_addr);
		if (ret)
			return ret;
	} else {
		ret = dw_pcie_prog_inbound_atu(pci, free_win, bar, cpu_addr,
					       as_type);
		if (ret < 0) {
			dev_err(pci->dev, "Failed to program IB window\n");
			return ret;
		}
	}

	ep->bar_to_atu[bar] = free_win;
	set_bit(free_win, ep->ib_window_map);

	return 0;
}

static int dw_pcie_ep_outbound_atu(struct dw_pcie_ep *ep, phys_addr_t phys_addr,
				   u64 pci_addr, size_t size)
{
	u32 free_win;
	int ret;
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);

	if (pci->ops->outbound_atu) {
		ret = pci->ops->outbound_atu(pci, phys_addr, pci_addr, size);
		return ret;
	}

	free_win = find_first_zero_bit(ep->ob_window_map, ep->num_ob_windows);
	if (free_win >= ep->num_ob_windows) {
		dev_err(pci->dev, "no free outbound window\n");
		return -EINVAL;
	}

	dw_pcie_prog_outbound_atu(pci, free_win, PCIE_ATU_TYPE_MEM,
				  phys_addr, pci_addr, size);

	set_bit(free_win, ep->ob_window_map);
	ep->outbound_addr[free_win] = phys_addr;

	return 0;
}

static void dw_pcie_ep_clear_bar(struct pci_epc *epc, u8 func_no,
				 struct pci_epf_bar *epf_bar)
{
	struct dw_pcie_ep *ep = epc_get_drvdata(epc);
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);
	enum pci_barno bar = epf_bar->barno;
	u32 atu_index = ep->bar_to_atu[bar];

	__dw_pcie_ep_reset_bar(pci, bar, epf_bar->flags);

	if (pci->ops->disable_atu)
		pci->ops->disable_atu(pci, 0, atu_index,
				      DW_PCIE_REGION_INBOUND);
	else
		dw_pcie_disable_atu(pci, atu_index, DW_PCIE_REGION_INBOUND);
	clear_bit(atu_index, ep->ib_window_map);
}

static int dw_pcie_ep_set_bar(struct pci_epc *epc, u8 func_no,
			      struct pci_epf_bar *epf_bar)
{
	int ret;
	struct dw_pcie_ep *ep = epc_get_drvdata(epc);
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);
	enum pci_barno bar = epf_bar->barno;
	size_t size = epf_bar->size;
	int flags = epf_bar->flags;
	enum dw_pcie_as_type as_type;
	u32 reg = PCI_BASE_ADDRESS_0 + (4 * bar);

	if (ep->ops->set_bar) {
		ret = ep->ops->set_bar(ep, func_no, epf_bar);
		return ret;
	}

	if (!(flags & PCI_BASE_ADDRESS_SPACE))
		as_type = DW_PCIE_AS_MEM;
	else
		as_type = DW_PCIE_AS_IO;

	ret = dw_pcie_ep_inbound_atu(ep, bar, epf_bar->phys_addr, as_type);
	if (ret)
		return ret;

	dw_pcie_dbi_ro_wr_en(pci);

	dw_pcie_writel_dbi2(pci, reg, lower_32_bits(size - 1));
	dw_pcie_writel_dbi(pci, reg, flags);

	if (flags & PCI_BASE_ADDRESS_MEM_TYPE_64) {
		dw_pcie_writel_dbi2(pci, reg + 4, upper_32_bits(size - 1));
		dw_pcie_writel_dbi(pci, reg + 4, 0);
	}

	dw_pcie_dbi_ro_wr_dis(pci);

	return 0;
}

static int dw_pcie_find_index(struct dw_pcie_ep *ep, phys_addr_t addr,
			      u32 *atu_index)
{
	u32 index;

	for (index = 0; index < ep->num_ob_windows; index++) {
		if (ep->outbound_addr[index] != addr)
			continue;
		*atu_index = index;
		return 0;
	}

	return -EINVAL;
}

static void dw_pcie_ep_unmap_addr(struct pci_epc *epc, u8 func_no,
				  phys_addr_t addr)
{
	int ret;
	u32 atu_index;
	struct dw_pcie_ep *ep = epc_get_drvdata(epc);
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);

	if (pci->ops->disable_atu) {
		pci->ops->disable_atu(pci, addr, 0, DW_PCIE_REGION_OUTBOUND);
		return;
	}

	ret = dw_pcie_find_index(ep, addr, &atu_index);
	if (ret < 0)
		return;

	dw_pcie_disable_atu(pci, atu_index, DW_PCIE_REGION_OUTBOUND);
	clear_bit(atu_index, ep->ob_window_map);
}

static int dw_pcie_ep_map_addr(struct pci_epc *epc, u8 func_no,
			       phys_addr_t addr,
			       u64 pci_addr, size_t size)
{
	int ret;
	struct dw_pcie_ep *ep = epc_get_drvdata(epc);
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);

	ret = dw_pcie_ep_outbound_atu(ep, addr, pci_addr, size);
	if (ret) {
		dev_err(pci->dev, "failed to enable address\n");
		return ret;
	}

	return 0;
}

static int dw_pcie_ep_get_msi(struct pci_epc *epc, u8 func_no)
{
	int val;
	struct dw_pcie_ep *ep = epc_get_drvdata(epc);
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);

	val = dw_pcie_readw_dbi(pci, MSI_MESSAGE_CONTROL);
	if (!(val & MSI_CAP_MSI_EN_MASK))
		return -EINVAL;

	val = (val & MSI_CAP_MME_MASK) >> MSI_CAP_MME_SHIFT;
	return val;
}

static int dw_pcie_ep_set_msi(struct pci_epc *epc, u8 func_no, u8 encode_int)
{
	int val;
	struct dw_pcie_ep *ep = epc_get_drvdata(epc);
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);

	val = dw_pcie_readw_dbi(pci, MSI_MESSAGE_CONTROL);
	val &= ~MSI_CAP_MMC_MASK;
	val |= (encode_int << MSI_CAP_MMC_SHIFT) & MSI_CAP_MMC_MASK;
	dw_pcie_dbi_ro_wr_en(pci);
	dw_pcie_writew_dbi(pci, MSI_MESSAGE_CONTROL, val);
	dw_pcie_dbi_ro_wr_dis(pci);

	return 0;
}

static int dw_pcie_ep_raise_irq(struct pci_epc *epc, u8 func_no,
				enum pci_epc_irq_type type, u8 interrupt_num)
{
	struct dw_pcie_ep *ep = epc_get_drvdata(epc);

	if (!ep->ops->raise_irq)
		return -EINVAL;

	return ep->ops->raise_irq(ep, func_no, type, interrupt_num);
}

static void dw_pcie_ep_stop(struct pci_epc *epc)
{
	struct dw_pcie_ep *ep = epc_get_drvdata(epc);
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);

	if (!pci->ops->stop_link)
		return;

	pci->ops->stop_link(pci);
}

static int dw_pcie_ep_start(struct pci_epc *epc)
{
	struct dw_pcie_ep *ep = epc_get_drvdata(epc);
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);

	if (!pci->ops->start_link)
		return -EINVAL;

	return pci->ops->start_link(pci);
}

static const struct pci_epc_ops epc_ops = {
	.epf_init		= dw_pcie_ep_epf_init,
	.epf_exit		= dw_pcie_ep_epf_exit,
	.data_transfer		= dw_pcie_ep_data_transfer,
	.write_header		= dw_pcie_ep_write_header,
	.set_bar		= dw_pcie_ep_set_bar,
	.clear_bar		= dw_pcie_ep_clear_bar,
	.map_addr		= dw_pcie_ep_map_addr,
	.unmap_addr		= dw_pcie_ep_unmap_addr,
	.set_msi		= dw_pcie_ep_set_msi,
	.get_msi		= dw_pcie_ep_get_msi,
	.raise_irq		= dw_pcie_ep_raise_irq,
	.start			= dw_pcie_ep_start,
	.stop			= dw_pcie_ep_stop,
};

int dw_pcie_ep_raise_msi_irq(struct dw_pcie_ep *ep, u8 func_no,
			     u8 interrupt_num)
{
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);
	struct pci_epc *epc = ep->epc;
	unsigned int aligned_offset;
	u16 msg_ctrl, msg_data;
	u32 msg_addr_lower, msg_addr_upper;
	u64 msg_addr;
	bool has_upper;
	int ret;

	/* Raise MSI per the PCI Local Bus Specification Revision 3.0, 6.8.1. */
	msg_ctrl = dw_pcie_readw_dbi(pci, MSI_MESSAGE_CONTROL);
	has_upper = !!(msg_ctrl & PCI_MSI_FLAGS_64BIT);
	msg_addr_lower = dw_pcie_readl_dbi(pci, MSI_MESSAGE_ADDR_L32);
	if (has_upper) {
		msg_addr_upper = dw_pcie_readl_dbi(pci, MSI_MESSAGE_ADDR_U32);
		msg_data = dw_pcie_readw_dbi(pci, MSI_MESSAGE_DATA_64);
	} else {
		msg_addr_upper = 0;
		msg_data = dw_pcie_readw_dbi(pci, MSI_MESSAGE_DATA_32);
	}
	aligned_offset = msg_addr_lower & (epc->mem->page_size - 1);
	msg_addr = ((u64)msg_addr_upper) << 32 |
			(msg_addr_lower & ~aligned_offset);
	ret = dw_pcie_ep_map_addr(epc, func_no, ep->msi_mem_phys, msg_addr,
				  epc->mem->page_size);
	if (ret)
		return ret;

	writel(msg_data | (interrupt_num - 1), ep->msi_mem + aligned_offset);

	dw_pcie_ep_unmap_addr(epc, func_no, ep->msi_mem_phys);

	return 0;
}

void dw_pcie_ep_exit(struct dw_pcie_ep *ep)
{
	struct pci_epc *epc = ep->epc;

	pci_epc_mem_free_addr(epc, ep->msi_mem_phys, ep->msi_mem,
			      epc->mem->page_size);

	pci_epc_mem_exit(epc);
}

static unsigned int dw_pcie_ep_find_ext_capability(struct dw_pcie *pci, int cap)
{
	u32 header;
	int pos = PCI_CFG_SPACE_SIZE;

	while (pos) {
		header = dw_pcie_readl_dbi(pci, pos);
		if (PCI_EXT_CAP_ID(header) == cap)
			return pos;

		pos = PCI_EXT_CAP_NEXT(header);
		if (!pos)
			break;
	}

	return 0;
}

int dw_pcie_ep_init(struct dw_pcie_ep *ep)
{
	int i;
	int ret;
	u32 reg;
	void *addr;
	unsigned int nbars;
	unsigned int offset;
	struct pci_epc *epc;
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);
	struct device *dev = pci->dev;
	struct device_node *np = dev->of_node;

	if (!pci->dbi_base || !pci->dbi_base2) {
		dev_err(dev, "dbi_base/deb_base2 is not populated\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "num-ib-windows", &ep->num_ib_windows);
	if (ret < 0) {
		dev_err(dev, "unable to read *num-ib-windows* property\n");
		return ret;
	}
	if (ep->num_ib_windows > MAX_IATU_IN) {
		dev_err(dev, "invalid *num-ib-windows*\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(np, "num-ob-windows", &ep->num_ob_windows);
	if (ret < 0) {
		dev_err(dev, "unable to read *num-ob-windows* property\n");
		return ret;
	}
	if (ep->num_ob_windows > MAX_IATU_OUT) {
		dev_err(dev, "invalid *num-ob-windows*\n");
		return -EINVAL;
	}

	ep->ib_window_map = devm_kzalloc(dev, sizeof(long) *
					 BITS_TO_LONGS(ep->num_ib_windows),
					 GFP_KERNEL);
	if (!ep->ib_window_map)
		return -ENOMEM;

	ep->ob_window_map = devm_kzalloc(dev, sizeof(long) *
					 BITS_TO_LONGS(ep->num_ob_windows),
					 GFP_KERNEL);
	if (!ep->ob_window_map)
		return -ENOMEM;

	addr = devm_kzalloc(dev, sizeof(phys_addr_t) * ep->num_ob_windows,
			    GFP_KERNEL);
	if (!addr)
		return -ENOMEM;
	ep->outbound_addr = addr;

	if (ep->ops->ep_init)
		ep->ops->ep_init(ep);

	epc = devm_pci_epc_create(dev, &epc_ops);
	if (IS_ERR(epc)) {
		dev_err(dev, "failed to create epc device\n");
		return PTR_ERR(epc);
	}

	ret = of_property_read_u8(np, "max-functions", &epc->max_functions);
	if (ret < 0)
		epc->max_functions = 1;

	ret = __pci_epc_mem_init(epc, ep->phys_base, ep->addr_size,
				 ep->page_size);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize address space\n");
		return ret;
	}

	ep->msi_mem = pci_epc_mem_alloc_addr(epc, &ep->msi_mem_phys,
					     epc->mem->page_size);
	if (!ep->msi_mem) {
		dev_err(dev, "Failed to reserve memory for MSI\n");
		return -ENOMEM;
	}

	ep->epc = epc;
	epc_set_drvdata(epc, ep);

	offset = dw_pcie_ep_find_ext_capability(pci, PCI_EXT_CAP_ID_REBAR);
	if (offset) {
		reg = dw_pcie_readl_dbi(pci, offset + PCI_REBAR_CTRL);
		nbars = (reg & PCI_REBAR_CTRL_NBAR_MASK) >>
			PCI_REBAR_CTRL_NBAR_SHIFT;

		dw_pcie_dbi_ro_wr_en(pci);
		for (i = 0; i < nbars; i++, offset += 8)
			dw_pcie_writel_dbi(pci, offset + 4, 0x0);
		dw_pcie_dbi_ro_wr_dis(pci);
	}

	dw_pcie_setup(pci);

	return 0;
}
