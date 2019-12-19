// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2017 Cadence
// Cadence PCIe endpoint controller driver.
// Author: Cyrille Pitchen <cyrille.pitchen@free-electrons.com>

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pci-epc.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/sizes.h>

#include "pcie-cadence.h"

#define CDNS_PCIE_EP_MIN_APERTURE		128	/* 128 bytes */
#define CDNS_PCIE_EP_IRQ_PCI_ADDR_NONE		0x1
#define CDNS_PCIE_EP_IRQ_PCI_ADDR_LEGACY	0x3
#define CDNS_PCIE_EP_MSIX_BAR				0x5

/**
 * struct cdns_pcie_ep - private data for this PCIe endpoint controller driver
 * @pcie: Cadence PCIe controller
 * @dev: pointer to PCIe EP device
 * @max_regions: maximum number of regions supported by hardware
 * @ob_region_map: bitmask of mapped outbound regions
 * @ob_addr: base addresses in the AXI bus where the outbound regions start
 * @irq_phys_addr: base address on the AXI bus where the MSI/legacy IRQ
 *		   dedicated outbound regions is mapped.
 * @irq_cpu_addr: base address in the CPU space where a write access triggers
 *		  the sending of a memory write (MSI) / normal message (legacy
 *		  IRQ) TLP through the PCIe bus.
 * @irq_pci_addr: used to save the current mapping of the MSI/legacy IRQ
 *		  dedicated outbound region.
 * @irq_pci_fn: the latest PCI function that has updated the mapping of
 *		the MSI/legacy IRQ dedicated outbound region.
 * @irq_pending: bitmask of asserted legacy IRQs.
 * @epf: info about physical functions attached to the endpoint controller
 */
struct cdns_pcie_ep {
	struct cdns_pcie		pcie;
	struct device			*dev;
	u32				max_regions;
	unsigned long			ob_region_map;
	phys_addr_t			*ob_addr;
	phys_addr_t			irq_phys_addr;
	void __iomem			*irq_cpu_addr;
	u64				irq_pci_addr;
	u8				irq_pci_fn;
	u8				irq_pending;
	struct cdns_pcie_epf		*epf;
};

static int cdns_pcie_ep_epf_init(struct pci_epc *epc, struct pci_epf *epf)
{
	return pci_epf_init_dma_chan(epf);
}

static void cdns_pcie_ep_epf_exit(struct pci_epc *epc, struct pci_epf *epf)
{
	pci_epf_clean_dma_chan(epf);
}

static int cdns_pcie_ep_data_transfer(struct pci_epc *epc, struct pci_epf *epf,
				      dma_addr_t dma_dst, dma_addr_t dma_src,
				      size_t len)
{
	return pci_epf_data_transfer(epf, dma_dst, dma_src, len);
}

static int cdns_pcie_ep_write_header(struct pci_epc *epc, u8 fn, u8 vfn,
				     struct pci_epf_header *hdr)
{
	struct cdns_pcie_ep *ep = epc_get_drvdata(epc);
	u32 cap = CDNS_PCIE_EP_FUNC_SRIOV_CAP_OFFSET;
	struct cdns_pcie *pcie = &ep->pcie;
	u32 reg;

	if (vfn > 1) {
		dev_dbg(&epc->dev, "Only Virtual Function #1 has deviceID\n");
		return 0;
	} else if (vfn == 1) {
		reg = cap + PCI_SRIOV_VF_DID;
		cdns_pcie_ep_fn_writew(pcie, fn, reg, hdr->deviceid);
		return 0;
	}

	cdns_pcie_ep_fn_writew(pcie, fn, PCI_DEVICE_ID, hdr->deviceid);
	cdns_pcie_ep_fn_writeb(pcie, fn, PCI_REVISION_ID, hdr->revid);
	cdns_pcie_ep_fn_writeb(pcie, fn, PCI_CLASS_PROG, hdr->progif_code);
	cdns_pcie_ep_fn_writew(pcie, fn, PCI_CLASS_DEVICE,
			       hdr->subclass_code | hdr->baseclass_code << 8);
	cdns_pcie_ep_fn_writeb(pcie, fn, PCI_CACHE_LINE_SIZE,
			       hdr->cache_line_size);
	cdns_pcie_ep_fn_writew(pcie, fn, PCI_SUBSYSTEM_ID, hdr->subsys_id);
	cdns_pcie_ep_fn_writeb(pcie, fn, PCI_INTERRUPT_PIN, hdr->interrupt_pin);

	/*
	 * Vendor ID can only be modified from function 0, all other functions
	 * use the same vendor ID as function 0.
	 */
	if (fn == 0) {
		/* Update the vendor IDs. */
		u32 id = CDNS_PCIE_LM_ID_VENDOR(hdr->vendorid) |
			 CDNS_PCIE_LM_ID_SUBSYS(hdr->subsys_vendor_id);

		cdns_pcie_writel(pcie, CDNS_PCIE_LM_ID, id);
	}

	return 0;
}

static int cdns_pcie_ep_set_bar(struct pci_epc *epc, u8 fn, u8 vfn,
				struct pci_epf_bar *epf_bar)
{
	struct cdns_pcie_ep *ep = epc_get_drvdata(epc);
	u32 cap = CDNS_PCIE_EP_FUNC_SRIOV_CAP_OFFSET;
	struct cdns_pcie_epf *epf = &ep->epf[fn];
	struct cdns_pcie *pcie = &ep->pcie;
	dma_addr_t bar_phys = epf_bar->phys_addr;
	enum pci_barno bar = epf_bar->barno;
	int flags = epf_bar->flags;
	u32 addr0, addr1, reg, cfg, b, aperture, ctrl;
	u32 first_vf_offset, stride;
	u64 sz;

	/* BAR size is 2^(aperture + 7) */
	sz = max_t(size_t, epf_bar->size, CDNS_PCIE_EP_MIN_APERTURE);
	/*
	 * roundup_pow_of_two() returns an unsigned long, which is not suited
	 * for 64bit values.
	 */
	sz = 1ULL << fls64(sz - 1);
	aperture = ilog2(sz) - 7; /* 128B -> 0, 256B -> 1, 512B -> 2, ... */

	if ((flags & PCI_BASE_ADDRESS_SPACE) == PCI_BASE_ADDRESS_SPACE_IO) {
		ctrl = CDNS_PCIE_LM_BAR_CFG_CTRL_IO_32BITS;
	} else {
		bool is_prefetch = !!(flags & PCI_BASE_ADDRESS_MEM_PREFETCH);
		bool is_64bits = sz > SZ_2G;

		if (is_64bits && (bar & 1))
			return -EINVAL;

		if (is_64bits && !(flags & PCI_BASE_ADDRESS_MEM_TYPE_64))
			epf_bar->flags |= PCI_BASE_ADDRESS_MEM_TYPE_64;

		if (is_64bits && is_prefetch)
			ctrl = CDNS_PCIE_LM_BAR_CFG_CTRL_PREFETCH_MEM_64BITS;
		else if (is_prefetch)
			ctrl = CDNS_PCIE_LM_BAR_CFG_CTRL_PREFETCH_MEM_32BITS;
		else if (is_64bits)
			ctrl = CDNS_PCIE_LM_BAR_CFG_CTRL_MEM_64BITS;
		else
			ctrl = CDNS_PCIE_LM_BAR_CFG_CTRL_MEM_32BITS;
	}

	addr0 = lower_32_bits(bar_phys);
	addr1 = upper_32_bits(bar_phys);

	if (vfn == 1) {
		if (bar < BAR_4) {
			reg = CDNS_PCIE_LM_EP_VFUNC_BAR_CFG0(fn);
			b = bar;
		} else {
			reg = CDNS_PCIE_LM_EP_VFUNC_BAR_CFG1(fn);
			b = bar - BAR_4;
		}
	} else {
		if (bar < BAR_4) {
			reg = CDNS_PCIE_LM_EP_FUNC_BAR_CFG0(fn);
			b = bar;
		} else {
			reg = CDNS_PCIE_LM_EP_FUNC_BAR_CFG1(fn);
			b = bar - BAR_4;
		}
	}

	if (vfn > 0) {
		first_vf_offset = cdns_pcie_ep_fn_readw(pcie, fn, cap +
							PCI_SRIOV_VF_OFFSET);
		stride = cdns_pcie_ep_fn_readw(pcie, fn, cap +
					       PCI_SRIOV_VF_STRIDE);
		fn = fn + first_vf_offset + ((vfn - 1) * stride);
		epf = &epf->epf[vfn - 1];
	}

	cdns_pcie_writel(pcie, CDNS_PCIE_AT_IB_EP_FUNC_BAR_ADDR0(fn, bar),
			 addr0);
	cdns_pcie_writel(pcie, CDNS_PCIE_AT_IB_EP_FUNC_BAR_ADDR1(fn, bar),
			 addr1);

	cfg = cdns_pcie_readl(pcie, reg);
	cfg &= ~(CDNS_PCIE_LM_EP_FUNC_BAR_CFG_BAR_APERTURE_MASK(b) |
		 CDNS_PCIE_LM_EP_FUNC_BAR_CFG_BAR_CTRL_MASK(b));
	cfg |= (CDNS_PCIE_LM_EP_FUNC_BAR_CFG_BAR_APERTURE(b, aperture) |
		CDNS_PCIE_LM_EP_FUNC_BAR_CFG_BAR_CTRL(b, ctrl));
	cdns_pcie_writel(pcie, reg, cfg);

	epf->epf_bar[bar] = epf_bar;

	return 0;
}

static void cdns_pcie_ep_clear_bar(struct pci_epc *epc, u8 fn, u8 vfn,
				   struct pci_epf_bar *epf_bar)
{
	struct cdns_pcie_ep *ep = epc_get_drvdata(epc);
	u32 cap = CDNS_PCIE_EP_FUNC_SRIOV_CAP_OFFSET;
	struct cdns_pcie_epf *epf = &ep->epf[fn];
	struct cdns_pcie *pcie = &ep->pcie;
	enum pci_barno bar = epf_bar->barno;
	u32 first_vf_offset, stride;
	u32 reg, cfg, b, ctrl;

	if (vfn == 1) {
		if (bar < BAR_4) {
			reg = CDNS_PCIE_LM_EP_VFUNC_BAR_CFG0(fn);
			b = bar;
		} else {
			reg = CDNS_PCIE_LM_EP_VFUNC_BAR_CFG1(fn);
			b = bar - BAR_4;
		}
	} else {
		if (bar < BAR_4) {
			reg = CDNS_PCIE_LM_EP_FUNC_BAR_CFG0(fn);
			b = bar;
		} else {
			reg = CDNS_PCIE_LM_EP_FUNC_BAR_CFG1(fn);
			b = bar - BAR_4;
		}
	}

	if (vfn > 0) {
		first_vf_offset = cdns_pcie_ep_fn_readw(pcie, fn, cap +
							PCI_SRIOV_VF_OFFSET);
		stride = cdns_pcie_ep_fn_readw(pcie, fn, cap +
					       PCI_SRIOV_VF_STRIDE);
		fn = fn + first_vf_offset + ((vfn - 1) * stride);
		epf = &epf->epf[vfn - 1];
	}

	ctrl = CDNS_PCIE_LM_BAR_CFG_CTRL_DISABLED;
	cfg = cdns_pcie_readl(pcie, reg);
	cfg &= ~(CDNS_PCIE_LM_EP_FUNC_BAR_CFG_BAR_APERTURE_MASK(b) |
		 CDNS_PCIE_LM_EP_FUNC_BAR_CFG_BAR_CTRL_MASK(b));
	cfg |= CDNS_PCIE_LM_EP_FUNC_BAR_CFG_BAR_CTRL(b, ctrl);
	cdns_pcie_writel(pcie, reg, cfg);

	cdns_pcie_writel(pcie, CDNS_PCIE_AT_IB_EP_FUNC_BAR_ADDR0(fn, bar), 0);
	cdns_pcie_writel(pcie, CDNS_PCIE_AT_IB_EP_FUNC_BAR_ADDR1(fn, bar), 0);

	epf->epf_bar[bar] = NULL;
}

static int cdns_pcie_ep_map_addr(struct pci_epc *epc, u8 fn, u8 vfn,
				 phys_addr_t addr, u64 pci_addr, size_t size)
{
	struct cdns_pcie_ep *ep = epc_get_drvdata(epc);
	struct cdns_pcie *pcie = &ep->pcie;
	u32 cap = CDNS_PCIE_EP_FUNC_SRIOV_CAP_OFFSET;
	u32 first_vf_offset, stride;
	u32 r;

	if (vfn > 0) {
		first_vf_offset = cdns_pcie_ep_fn_readw(pcie, fn, cap +
							PCI_SRIOV_VF_OFFSET);
		stride = cdns_pcie_ep_fn_readw(pcie, fn, cap +
					       PCI_SRIOV_VF_STRIDE);
		fn = fn + first_vf_offset + ((vfn - 1) * stride);
	}

	r = find_first_zero_bit(&ep->ob_region_map,
				sizeof(ep->ob_region_map) * BITS_PER_LONG);
	if (r >= ep->max_regions - 1) {
		dev_err(&epc->dev, "no free outbound region\n");
		return -EINVAL;
	}

	cdns_pcie_set_outbound_region(pcie, fn, r, false, addr, pci_addr, size);

	set_bit(r, &ep->ob_region_map);
	ep->ob_addr[r] = addr;

	return 0;
}

static void cdns_pcie_ep_unmap_addr(struct pci_epc *epc, u8 fn, u8 vfn,
				    phys_addr_t addr)
{
	struct cdns_pcie_ep *ep = epc_get_drvdata(epc);
	struct cdns_pcie *pcie = &ep->pcie;
	u32 r;

	for (r = 0; r < ep->max_regions - 1; r++)
		if (ep->ob_addr[r] == addr)
			break;

	if (r == ep->max_regions - 1)
		return;

	cdns_pcie_reset_outbound_region(pcie, r);

	ep->ob_addr[r] = 0;
	clear_bit(r, &ep->ob_region_map);
}

static int cdns_pcie_ep_set_msi(struct pci_epc *epc, u8 fn, u8 vfn, u8 mmc)
{
	struct cdns_pcie_ep *ep = epc_get_drvdata(epc);
	struct cdns_pcie *pcie = &ep->pcie;
	u32 sriov_cap = CDNS_PCIE_EP_FUNC_SRIOV_CAP_OFFSET;
	u32 cap = CDNS_PCIE_EP_FUNC_MSI_CAP_OFFSET;
	u32 first_vf_offset, stride;
	u16 flags;

	if (vfn > 0) {
		first_vf_offset = cdns_pcie_ep_fn_readw(pcie, fn, sriov_cap +
							PCI_SRIOV_VF_OFFSET);
		stride = cdns_pcie_ep_fn_readw(pcie, fn, sriov_cap +
					       PCI_SRIOV_VF_STRIDE);
		fn = fn + first_vf_offset + ((vfn - 1) * stride);
	}

	/*
	 * Set the Multiple Message Capable bitfield into the Message Control
	 * register.
	 */
	flags = cdns_pcie_ep_fn_readw(pcie, fn, cap + PCI_MSI_FLAGS);
	flags = (flags & ~PCI_MSI_FLAGS_QMASK) | (mmc << 1);
	flags |= PCI_MSI_FLAGS_64BIT;
	flags &= ~PCI_MSI_FLAGS_MASKBIT;
	cdns_pcie_ep_fn_writew(pcie, fn, cap + PCI_MSI_FLAGS, flags);

	return 0;
}

static int cdns_pcie_ep_get_msi(struct pci_epc *epc, u8 fn, u8 vfn)
{
	struct cdns_pcie_ep *ep = epc_get_drvdata(epc);
	struct cdns_pcie *pcie = &ep->pcie;
	u32 sriov_cap = CDNS_PCIE_EP_FUNC_SRIOV_CAP_OFFSET;
	u32 cap = CDNS_PCIE_EP_FUNC_MSI_CAP_OFFSET;
	u32 first_vf_offset, stride;
	u16 flags, mme;

	if (vfn > 0) {
		first_vf_offset = cdns_pcie_ep_fn_readw(pcie, fn, sriov_cap +
							PCI_SRIOV_VF_OFFSET);
		stride = cdns_pcie_ep_fn_readw(pcie, fn, sriov_cap +
					       PCI_SRIOV_VF_STRIDE);
		fn = fn + first_vf_offset + ((vfn - 1) * stride);
	}

	/* Validate that the MSI feature is actually enabled. */
	flags = cdns_pcie_ep_fn_readw(pcie, fn, cap + PCI_MSI_FLAGS);
	if (!(flags & PCI_MSI_FLAGS_ENABLE))
		return -EINVAL;

	/*
	 * Get the Multiple Message Enable bitfield from the Message Control
	 * register.
	 */
	mme = (flags & PCI_MSI_FLAGS_QSIZE) >> 4;

	return mme;
}

static int cdns_pcie_ep_get_msix(struct pci_epc *epc, u8 func_no, u8 vfunc_no)
{
	u32 sriov_cap = CDNS_PCIE_EP_FUNC_SRIOV_CAP_OFFSET;
	struct cdns_pcie_ep *ep = epc_get_drvdata(epc);
	struct cdns_pcie *pcie = &ep->pcie;
	u32 cap = CDNS_PCIE_EP_FUNC_MSIX_CAP_OFFSET;
	u32 first_vf_offset, stride;
	u32 val, reg;

	if (vfunc_no > 0) {
		first_vf_offset = cdns_pcie_ep_fn_readw(pcie, func_no, sriov_cap
							+ PCI_SRIOV_VF_OFFSET);
		stride = cdns_pcie_ep_fn_readw(pcie, func_no, sriov_cap +
					       PCI_SRIOV_VF_STRIDE);
		func_no = func_no + first_vf_offset + ((vfunc_no - 1) * stride);
	}

	reg = cap + PCI_MSIX_FLAGS;
	val = cdns_pcie_ep_fn_readw(pcie, func_no, reg);
	if (!(val & PCI_MSIX_FLAGS_ENABLE))
		return -EINVAL;

	val &= PCI_MSIX_FLAGS_QSIZE;

	return val;
}

static int cdns_pcie_ep_set_msix(struct pci_epc *epc, u8 fn, u8 vfn,
				 u16 interrupts, enum pci_barno bir, u32 offset)
{
	u32 sriov_cap = CDNS_PCIE_EP_FUNC_SRIOV_CAP_OFFSET;
	struct cdns_pcie_ep *ep = epc_get_drvdata(epc);
	struct cdns_pcie *pcie = &ep->pcie;
	u32 cap = CDNS_PCIE_EP_FUNC_MSIX_CAP_OFFSET;
	u32 first_vf_offset, stride;
	u32 val, reg;

	if (vfn > 0) {
		first_vf_offset = cdns_pcie_ep_fn_readw(pcie, fn, sriov_cap +
							PCI_SRIOV_VF_OFFSET);
		stride = cdns_pcie_ep_fn_readw(pcie, fn, sriov_cap +
					       PCI_SRIOV_VF_STRIDE);
		fn = fn + first_vf_offset + ((vfn - 1) * stride);
	}

	reg = cap + PCI_MSIX_FLAGS;
	val = cdns_pcie_ep_fn_readw(pcie, fn, reg);
	val &= ~PCI_MSIX_FLAGS_QSIZE;
	val |= interrupts;
	cdns_pcie_ep_fn_writew(pcie, fn, reg, val);

	/* Set MSIX BAR and offset */
	reg = cap + PCI_MSIX_TABLE;
	val = offset | bir;
	cdns_pcie_ep_fn_writel(pcie, fn, reg, val);

	/* Set PBA BAR and offset.  BAR must match MSIX BAR */
	reg = cap + PCI_MSIX_PBA;
	val = (offset + (interrupts * PCI_MSIX_ENTRY_SIZE)) | bir;
	cdns_pcie_ep_fn_writel(pcie, fn, reg, val);

	return 0;
}

static void cdns_pcie_ep_assert_intx(struct cdns_pcie_ep *ep, u8 fn, u8 vfn,
				     u8 intx, bool is_asserted)
{
	struct cdns_pcie *pcie = &ep->pcie;
	u32 offset;
	u16 status;
	u8 msg_code;

	intx &= 3;

	/* Set the outbound region if needed. */
	if (unlikely(ep->irq_pci_addr != CDNS_PCIE_EP_IRQ_PCI_ADDR_LEGACY ||
		     ep->irq_pci_fn != fn)) {
		/* First region was reserved for IRQ writes. */
		cdns_pcie_set_outbound_region_for_normal_msg(pcie, fn, 0,
							     ep->irq_phys_addr);
		ep->irq_pci_addr = CDNS_PCIE_EP_IRQ_PCI_ADDR_LEGACY;
		ep->irq_pci_fn = fn;
	}

	if (is_asserted) {
		ep->irq_pending |= BIT(intx);
		msg_code = MSG_CODE_ASSERT_INTA + intx;
	} else {
		ep->irq_pending &= ~BIT(intx);
		msg_code = MSG_CODE_DEASSERT_INTA + intx;
	}

	status = cdns_pcie_ep_fn_readw(pcie, fn, PCI_STATUS);
	if (((status & PCI_STATUS_INTERRUPT) != 0) ^ (ep->irq_pending != 0)) {
		status ^= PCI_STATUS_INTERRUPT;
		cdns_pcie_ep_fn_writew(pcie, fn, PCI_STATUS, status);
	}

	offset = CDNS_PCIE_NORMAL_MSG_ROUTING(MSG_ROUTING_LOCAL) |
		 CDNS_PCIE_NORMAL_MSG_CODE(msg_code) |
		 CDNS_PCIE_MSG_NO_DATA;
	writel(0, ep->irq_cpu_addr + offset);
}

static int cdns_pcie_ep_send_legacy_irq(struct cdns_pcie_ep *ep, u8 fn, u8 vfn,
					u8 intx)
{
	u16 cmd;

	cmd = cdns_pcie_ep_fn_readw(&ep->pcie, fn, PCI_COMMAND);
	if (cmd & PCI_COMMAND_INTX_DISABLE)
		return -EINVAL;

	cdns_pcie_ep_assert_intx(ep, fn, vfn, intx, true);
	/*
	 * The mdelay() value was taken from dra7xx_pcie_raise_legacy_irq()
	 * from drivers/pci/dwc/pci-dra7xx.c
	 */
	mdelay(1);
	cdns_pcie_ep_assert_intx(ep, fn, vfn, intx, false);
	return 0;
}

static int cdns_pcie_ep_send_msi_irq(struct cdns_pcie_ep *ep, u8 fn, u8 vfn,
				     u8 interrupt_num)
{
	struct cdns_pcie *pcie = &ep->pcie;
	u32 sriov_cap = CDNS_PCIE_EP_FUNC_SRIOV_CAP_OFFSET;
	u32 cap = CDNS_PCIE_EP_FUNC_MSI_CAP_OFFSET;
	u16 flags, mme, data, data_mask;
	u8 msi_count;
	u64 pci_addr, pci_addr_mask = 0xff;
	u32 first_vf_offset, stride;

	if (vfn > 0) {
		first_vf_offset = cdns_pcie_ep_fn_readw(pcie, fn, sriov_cap +
							PCI_SRIOV_VF_OFFSET);
		stride = cdns_pcie_ep_fn_readw(pcie, fn, sriov_cap +
					       PCI_SRIOV_VF_STRIDE);
		fn = fn + first_vf_offset + ((vfn - 1) * stride);
	}

	/* Check whether the MSI feature has been enabled by the PCI host. */
	flags = cdns_pcie_ep_fn_readw(pcie, fn, cap + PCI_MSI_FLAGS);
	if (!(flags & PCI_MSI_FLAGS_ENABLE))
		return -EINVAL;

	/* Get the number of enabled MSIs */
	mme = (flags & PCI_MSI_FLAGS_QSIZE) >> 4;
	msi_count = 1 << mme;
	if (!interrupt_num || interrupt_num > msi_count)
		return -EINVAL;

	/* Check whether MSI is masked */
	data = cdns_pcie_ep_fn_readw(pcie, fn, cap + PCI_MSI_MASK_64);
	if (data & (1 << (interrupt_num - 1)))
		return -EINVAL;

	/* Compute the data value to be written. */
	data_mask = msi_count - 1;
	data = cdns_pcie_ep_fn_readw(pcie, fn, cap + PCI_MSI_DATA_64);
	data = (data & ~data_mask) | ((interrupt_num - 1) & data_mask);

	/* Get the PCI address where to write the data into. */
	pci_addr = cdns_pcie_ep_fn_readl(pcie, fn, cap + PCI_MSI_ADDRESS_HI);
	pci_addr <<= 32;
	pci_addr |= cdns_pcie_ep_fn_readl(pcie, fn, cap + PCI_MSI_ADDRESS_LO);
	pci_addr &= GENMASK_ULL(63, 2);

	/* Set the outbound region if needed. */
	if (unlikely(ep->irq_pci_addr != (pci_addr & ~pci_addr_mask) ||
		     ep->irq_pci_fn != fn)) {
		/* First region was reserved for IRQ writes. */
		cdns_pcie_set_outbound_region(pcie, fn, 0,
					      false,
					      ep->irq_phys_addr,
					      pci_addr & ~pci_addr_mask,
					      pci_addr_mask + 1);
		ep->irq_pci_addr = (pci_addr & ~pci_addr_mask);
		ep->irq_pci_fn = fn;
	}
	writel(data, ep->irq_cpu_addr + (pci_addr & pci_addr_mask));

	return 0;
}

static int cdns_pcie_ep_map_msi_irq(struct pci_epc *epc, u8 fn, u8 vfn,
				    phys_addr_t addr, u8 interrupt_num,
				    u32 entry_size, u32 *msi_data)
{
	u32 sriov_cap = CDNS_PCIE_EP_FUNC_SRIOV_CAP_OFFSET;
	struct cdns_pcie_ep *ep = epc_get_drvdata(epc);
	u32 cap = CDNS_PCIE_EP_FUNC_MSI_CAP_OFFSET;
	struct cdns_pcie *pcie = &ep->pcie;
	u16 flags, mme, data, data_mask;
	u32 first_vf_offset, stride;
	u8 msi_count;
	u64 pci_addr;
	int ret;
	int i;

	if (vfn > 0) {
		first_vf_offset = cdns_pcie_ep_fn_readw(pcie, fn, sriov_cap +
							PCI_SRIOV_VF_OFFSET);
		stride = cdns_pcie_ep_fn_readw(pcie, fn, sriov_cap +
					       PCI_SRIOV_VF_STRIDE);
		fn = fn + first_vf_offset + ((vfn - 1) * stride);
	}

	/* Check whether the MSI feature has been enabled by the PCI host. */
	flags = cdns_pcie_ep_fn_readw(pcie, fn, cap + PCI_MSI_FLAGS);
	if (!(flags & PCI_MSI_FLAGS_ENABLE))
		return -EINVAL;

	/* Get the number of enabled MSIs */
	mme = (flags & PCI_MSI_FLAGS_QSIZE) >> 4;
	msi_count = 1 << mme;
	if (!interrupt_num || interrupt_num > msi_count)
		return -EINVAL;

	/* Compute the data value to be written. */
	data_mask = msi_count - 1;
	data = cdns_pcie_ep_fn_readw(pcie, fn, cap + PCI_MSI_DATA_64);
	data = data & ~data_mask;

	/* Get the PCI address where to write the data into. */
	pci_addr = cdns_pcie_ep_fn_readl(pcie, fn, cap + PCI_MSI_ADDRESS_HI);
	pci_addr <<= 32;
	pci_addr |= cdns_pcie_ep_fn_readl(pcie, fn, cap + PCI_MSI_ADDRESS_LO);
	pci_addr &= GENMASK_ULL(63, 2);

	for (i = 0; i < interrupt_num; i++) {
		ret = cdns_pcie_ep_map_addr(epc, fn, vfn, addr, pci_addr,
					    entry_size);
		if (ret)
			return ret;
		addr = addr + entry_size;
	}

	*msi_data = data;

	return 0;
}

static int cdns_pcie_ep_send_msix_irq(struct cdns_pcie_ep *ep, u8 fn, u8 vfn,
				      u16 interrupt_num)
{
	u32 sriov_cap = CDNS_PCIE_EP_FUNC_SRIOV_CAP_OFFSET;
	u32 cap = CDNS_PCIE_EP_FUNC_MSIX_CAP_OFFSET;
	u32 tbl_offset, msg_data, reg, vec_ctrl;
	struct cdns_pcie *pcie = &ep->pcie;
	struct pci_epf_msix_tbl *msix_tbl;
	struct pci_epf_bar *epf_bar;
	u32 first_vf_offset, stride;
	struct cdns_pcie_epf *epf;
	u64 pci_addr_mask = 0xff;
	u64 msg_addr;
	u16 flags;
	u8 bir;

	epf = &ep->epf[fn];

	if (vfn > 0) {
		first_vf_offset = cdns_pcie_ep_fn_readw(pcie, fn, sriov_cap +
							PCI_SRIOV_VF_OFFSET);
		stride = cdns_pcie_ep_fn_readw(pcie, fn, sriov_cap +
					       PCI_SRIOV_VF_STRIDE);
		fn = fn + first_vf_offset + ((vfn - 1) * stride);
		epf = &epf->epf[vfn - 1];
	}

	/* Check whether the MSI-X feature has been enabled by the PCI host. */
	flags = cdns_pcie_ep_fn_readw(pcie, fn, cap + PCI_MSIX_FLAGS);
	if (!(flags & PCI_MSIX_FLAGS_ENABLE))
		return -EINVAL;

	reg = cap + PCI_MSIX_TABLE;
	tbl_offset = cdns_pcie_ep_fn_readl(pcie, fn, reg);
	bir = tbl_offset & PCI_MSIX_TABLE_BIR;
	tbl_offset &= PCI_MSIX_TABLE_OFFSET;

	epf_bar = epf->epf_bar[bir];
	msix_tbl = epf_bar->addr;
	msix_tbl = (struct pci_epf_msix_tbl *)((char *)msix_tbl + tbl_offset);

	msg_addr = msix_tbl[(interrupt_num - 1)].msg_addr;
	msg_data = msix_tbl[(interrupt_num - 1)].msg_data;
	vec_ctrl = msix_tbl[(interrupt_num - 1)].vector_ctrl;

	/* Set the outbound region if needed. */
	if (unlikely(ep->irq_pci_addr != (msg_addr & ~pci_addr_mask) ||
		     ep->irq_pci_fn != fn)) {
		/* First region was reserved for IRQ writes. */
		cdns_pcie_set_outbound_region(pcie, fn, 0,
					      false,
					      ep->irq_phys_addr,
					      msg_addr & ~pci_addr_mask,
					      pci_addr_mask + 1);
		ep->irq_pci_addr = (msg_addr & ~pci_addr_mask);
		ep->irq_pci_fn = fn;
	}
	writel(msg_data, ep->irq_cpu_addr + (msg_addr & pci_addr_mask));

	return 0;
}

static int cdns_pcie_ep_raise_irq(struct pci_epc *epc, u8 fn, u8 vfn,
				  enum pci_epc_irq_type type,
				  u16 interrupt_num)
{
	struct cdns_pcie_ep *ep = epc_get_drvdata(epc);
	u32 link_status;

	/* Can't send an IRQ if the link is down. */
	link_status = cdns_pcie_readl(&ep->pcie, CDNS_PCIE_LM_BASE);
	if (!(link_status & 0x1))
		return -EINVAL;

	switch (type) {
	case PCI_EPC_IRQ_LEGACY:
		return cdns_pcie_ep_send_legacy_irq(ep, fn, vfn, 0);

	case PCI_EPC_IRQ_MSI:
		return cdns_pcie_ep_send_msi_irq(ep, fn, vfn, interrupt_num);

	case PCI_EPC_IRQ_MSIX:
		return cdns_pcie_ep_send_msix_irq(ep, fn, vfn, interrupt_num);

	default:
		break;
	}

	return -EINVAL;
}

static int cdns_pcie_ep_start(struct pci_epc *epc)
{
	struct cdns_pcie_ep *ep = epc_get_drvdata(epc);
	struct cdns_pcie *pcie = &ep->pcie;
	struct pci_epf *epf;
	int ret = 0;
	u32 cfg;

	/*
	 * BIT(0) is hardwired to 1, hence function 0 is always enabled
	 * and can't be disabled anyway.
	 */
	cfg = BIT(0);
	list_for_each_entry(epf, &epc->pci_epf, list)
		cfg |= BIT(epf->func_no);
	cdns_pcie_writel(pcie, CDNS_PCIE_LM_EP_FUNC_CFG, cfg);

	ret = cdns_pcie_start_link(pcie, true);
	if (ret)
		dev_err(ep->dev, "Failed to start link\n");

	return ret;
}

static const struct pci_epc_features cdns_pcie_epc_features = {
	.linkup_notifier = false,
	.msi_capable = true,
	.msix_capable = true,
	.align = 256,
};

static const struct pci_epc_features cdns_pcie_epc_vf_features = {
	.linkup_notifier = false,
	.msi_capable = true,
	.msix_capable = true,
	.align = 65536,
};

static const struct pci_epc_features*
cdns_pcie_ep_get_features(struct pci_epc *epc, u8 func_no, u8 vfunc_no)
{
	if (!vfunc_no)
		return &cdns_pcie_epc_features;

	return &cdns_pcie_epc_vf_features;
}

static const struct pci_epc_ops cdns_pcie_epc_ops = {
	.epf_init	= cdns_pcie_ep_epf_init,
	.epf_exit	= cdns_pcie_ep_epf_exit,
	.data_transfer	= cdns_pcie_ep_data_transfer,
	.write_header	= cdns_pcie_ep_write_header,
	.set_bar	= cdns_pcie_ep_set_bar,
	.clear_bar	= cdns_pcie_ep_clear_bar,
	.map_addr	= cdns_pcie_ep_map_addr,
	.unmap_addr	= cdns_pcie_ep_unmap_addr,
	.set_msi	= cdns_pcie_ep_set_msi,
	.get_msi	= cdns_pcie_ep_get_msi,
	.set_msix	= cdns_pcie_ep_set_msix,
	.get_msix	= cdns_pcie_ep_get_msix,
	.raise_irq	= cdns_pcie_ep_raise_irq,
	.map_msi_irq	= cdns_pcie_ep_map_msi_irq,
	.start		= cdns_pcie_ep_start,
	.get_features	= cdns_pcie_ep_get_features,
};

static struct cdns_pcie_ep_data cdns_ti_pcie_ep_data = {
	.read = cdns_pcie_read32,
	.write = cdns_pcie_write32,
};

static const struct of_device_id cdns_pcie_ep_of_match[] = {
	{ .compatible = "cdns,cdns-pcie-ep",
	},
	{ .compatible = "ti,j721e-cdns-pcie-ep",
	  .data = &cdns_ti_pcie_ep_data,
	},
	{ },
};

static int cdns_pcie_ep_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	const struct cdns_pcie_ep_data *data;
	const struct of_device_id *match;
	struct cdns_pcie_epf *epf;
	struct cdns_pcie_ep *ep;
	struct cdns_pcie *pcie;
	struct pci_epc *epc;
	struct resource *res;
	int ret;
	int phy_count;
	u8 *max_vfs;
	int i;

	match = of_match_device(of_match_ptr(cdns_pcie_ep_of_match), dev);
	if (!match)
		return -EINVAL;

	ep = devm_kzalloc(dev, sizeof(*ep), GFP_KERNEL);
	if (!ep)
		return -ENOMEM;

	ep->dev = dev;

	pcie = &ep->pcie;
	pcie->is_rc = false;
	pcie->plat_data = pdev->dev.platform_data;

	data = (struct cdns_pcie_ep_data *)match->data;
	if (data) {
		if (data->read)
			pcie->read = data->read;
		if (data->write)
			pcie->write = data->write;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "reg");
	pcie->reg_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(pcie->reg_base)) {
		dev_err(dev, "missing \"reg\"\n");
		return PTR_ERR(pcie->reg_base);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "mem");
	if (!res)
		dev_dbg(dev, "missing \"mem\"\n");
	pcie->mem_res = res;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "addr_space");
	if (!res) {
		dev_err(dev, "missing \"addr_space\"\n");
		return -EINVAL;
	}
	pcie->addr_res = res;

	ret = of_property_read_u32(np, "cdns,max-outbound-regions",
				   &ep->max_regions);
	if (ret < 0) {
		dev_err(dev, "missing \"cdns,max-outbound-regions\"\n");
		return ret;
	}
	ep->ob_addr = devm_kcalloc(dev,
				   ep->max_regions, sizeof(*ep->ob_addr),
				   GFP_KERNEL);
	if (!ep->ob_addr)
		return -ENOMEM;

	ret = cdns_pcie_init_phy(dev, pcie);
	if (ret) {
		dev_err(dev, "failed to init phy\n");
		return ret;
	}
	platform_set_drvdata(pdev, pcie);
	pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		dev_err(dev, "pm_runtime_get_sync() failed\n");
		goto err_get_sync;
	}

	/* Disable all but function 0 (anyway BIT(0) is hardwired to 1). */
	cdns_pcie_writel(pcie, CDNS_PCIE_LM_EP_FUNC_CFG, BIT(0));

	epc = devm_pci_epc_create(dev, &cdns_pcie_epc_ops);
	if (IS_ERR(epc)) {
		dev_err(dev, "failed to create epc device\n");
		ret = PTR_ERR(epc);
		goto err_init;
	}

	epc_set_drvdata(epc, ep);

	if (of_property_read_u8(np, "max-functions", &epc->max_functions) < 0)
		epc->max_functions = 1;

	ep->epf = devm_kcalloc(dev, epc->max_functions, sizeof(*ep->epf),
			       GFP_KERNEL);
	if (!ep->epf)
		return -ENOMEM;

	max_vfs = devm_kcalloc(dev, epc->max_functions, sizeof(*max_vfs),
			       GFP_KERNEL);
	if (!max_vfs)
		return -ENOMEM;

	of_property_read_u8_array(np, "max-virtual-functions", max_vfs,
				  epc->max_functions);

	for (i = 0; i < epc->max_functions; i++) {
		epf = &ep->epf[i];
		if (max_vfs[i] == 0)
			continue;
		epf->epf = devm_kcalloc(dev, max_vfs[i], sizeof(*ep->epf),
					GFP_KERNEL);
		if (!epf->epf)
			return -ENOMEM;
	}

	ret = pci_epc_mem_init(epc, pcie->addr_res->start,
			       resource_size(pcie->addr_res));
	if (ret < 0) {
		dev_err(dev, "failed to initialize the memory space\n");
		goto err_init;
	}

	ep->irq_cpu_addr = pci_epc_mem_alloc_addr(epc, &ep->irq_phys_addr,
						  SZ_128K);
	if (!ep->irq_cpu_addr) {
		dev_err(dev, "failed to reserve memory space for MSI\n");
		ret = -ENOMEM;
		goto free_epc_mem;
	}
	ep->irq_pci_addr = CDNS_PCIE_EP_IRQ_PCI_ADDR_NONE;
	/* Reserve region 0 for IRQs */
	set_bit(0, &ep->ob_region_map);

	return 0;

 free_epc_mem:
	pci_epc_mem_exit(epc);

 err_init:
	pm_runtime_put_sync(dev);

 err_get_sync:
	pm_runtime_disable(dev);
	cdns_pcie_disable_phy(pcie);
	phy_count = pcie->phy_count;
	while (phy_count--)
		device_link_del(pcie->link[phy_count]);

	return ret;
}

static void cdns_pcie_ep_shutdown(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct cdns_pcie *pcie = dev_get_drvdata(dev);
	int ret;

	ret = pm_runtime_put_sync(dev);
	if (ret < 0)
		dev_dbg(dev, "pm_runtime_put_sync failed\n");

	pm_runtime_disable(dev);

	cdns_pcie_disable_phy(pcie);
}

static struct platform_driver cdns_pcie_ep_driver = {
	.driver = {
		.name = "cdns-pcie-ep",
		.of_match_table = cdns_pcie_ep_of_match,
		.pm	= &cdns_pcie_pm_ops,
	},
	.probe = cdns_pcie_ep_probe,
	.shutdown = cdns_pcie_ep_shutdown,
};
builtin_platform_driver(cdns_pcie_ep_driver);
