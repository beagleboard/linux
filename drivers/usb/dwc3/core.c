/**
 * core.c - DesignWare USB3 DRD Controller Core file
 *
 * Copyright (C) 2010-2011 Texas Instruments Incorporated - http://www.ti.com
 *
 * Authors: Felipe Balbi <balbi@ti.com>,
 *	    Sebastian Andrzej Siewior <bigeasy@linutronix.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2  of
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

#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>
#include <linux/acpi.h>
#include <linux/pinctrl/consumer.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/of.h>
#include <linux/usb/otg.h>

#include "core.h"
#include "gadget.h"
#include "io.h"

#include "debug.h"

#define DWC3_DEFAULT_AUTOSUSPEND_DELAY	5000 /* ms */

/**
 * dwc3_get_dr_mode - Validates and sets dr_mode
 * @dwc: pointer to our context structure
 */
static int dwc3_get_dr_mode(struct dwc3 *dwc)
{
	enum usb_dr_mode mode;
	struct device *dev = dwc->dev;
	unsigned int hw_mode;

	if (dwc->dr_mode == USB_DR_MODE_UNKNOWN)
		dwc->dr_mode = USB_DR_MODE_OTG;

	mode = dwc->dr_mode;
	hw_mode = DWC3_GHWPARAMS0_MODE(dwc->hwparams.hwparams0);

	switch (hw_mode) {
	case DWC3_GHWPARAMS0_MODE_GADGET:
		if (IS_ENABLED(CONFIG_USB_DWC3_HOST)) {
			dev_err(dev,
				"Controller does not support host mode.\n");
			return -EINVAL;
		}
		mode = USB_DR_MODE_PERIPHERAL;
		break;
	case DWC3_GHWPARAMS0_MODE_HOST:
		if (IS_ENABLED(CONFIG_USB_DWC3_GADGET)) {
			dev_err(dev,
				"Controller does not support device mode.\n");
			return -EINVAL;
		}
		mode = USB_DR_MODE_HOST;
		break;
	default:
		if (IS_ENABLED(CONFIG_USB_DWC3_HOST))
			mode = USB_DR_MODE_HOST;
		else if (IS_ENABLED(CONFIG_USB_DWC3_GADGET))
			mode = USB_DR_MODE_PERIPHERAL;
	}

	if (mode != dwc->dr_mode) {
		dev_warn(dev,
			 "Configuration mismatch. dr_mode forced to %s\n",
			 mode == USB_DR_MODE_HOST ? "host" : "gadget");

		dwc->dr_mode = mode;
	}

	return 0;
}

void dwc3_set_mode(struct dwc3 *dwc, u32 mode)
{
	u32 reg;

	reg = dwc3_readl(dwc->regs, DWC3_GCTL);
	reg &= ~(DWC3_GCTL_PRTCAPDIR(DWC3_GCTL_PRTCAP_OTG));
	reg |= DWC3_GCTL_PRTCAPDIR(mode);
	dwc->current_mode = mode;
	dwc3_writel(dwc->regs, DWC3_GCTL, reg);
}

u32 dwc3_core_fifo_space(struct dwc3_ep *dep, u8 type)
{
	struct dwc3		*dwc = dep->dwc;
	u32			reg;

	dwc3_writel(dwc->regs, DWC3_GDBGFIFOSPACE,
			DWC3_GDBGFIFOSPACE_NUM(dep->number) |
			DWC3_GDBGFIFOSPACE_TYPE(type));

	reg = dwc3_readl(dwc->regs, DWC3_GDBGFIFOSPACE);

	return DWC3_GDBGFIFOSPACE_SPACE_AVAILABLE(reg);
}

/**
 * dwc3_core_soft_reset - Issues core soft reset and PHY reset
 * @dwc: pointer to our context structure
 */
static int dwc3_core_soft_reset(struct dwc3 *dwc)
{
	u32		reg;
	int		retries = 1000;
	int		ret;

	usb_phy_init(dwc->usb2_phy);
	usb_phy_init(dwc->usb3_phy);
	ret = phy_init(dwc->usb2_generic_phy);
	if (ret < 0)
		return ret;

	ret = phy_init(dwc->usb3_generic_phy);
	if (ret < 0) {
		phy_exit(dwc->usb2_generic_phy);
		return ret;
	}

	/*
	 * We're resetting only the device side because, if we're in host mode,
	 * XHCI driver will reset the host block. If dwc3 was configured for
	 * host-only mode, then we can return early.
	 */
	if (dwc->dr_mode == USB_DR_MODE_HOST)
		return 0;

	reg = dwc3_readl(dwc->regs, DWC3_DCTL);
	reg |= DWC3_DCTL_CSFTRST;
	dwc3_writel(dwc->regs, DWC3_DCTL, reg);

	do {
		reg = dwc3_readl(dwc->regs, DWC3_DCTL);
		if (!(reg & DWC3_DCTL_CSFTRST))
			return 0;

		udelay(1);
	} while (--retries);

	return -ETIMEDOUT;
}

/**
 * dwc3_soft_reset - Issue soft reset
 * @dwc: Pointer to our controller context structure
 */
static int dwc3_soft_reset(struct dwc3 *dwc)
{
	unsigned long timeout;
	u32 reg;

	timeout = jiffies + msecs_to_jiffies(500);
	dwc3_writel(dwc->regs, DWC3_DCTL, DWC3_DCTL_CSFTRST);
	do {
		reg = dwc3_readl(dwc->regs, DWC3_DCTL);
		if (!(reg & DWC3_DCTL_CSFTRST))
			break;

		if (time_after(jiffies, timeout)) {
			dev_err(dwc->dev, "Reset Timed Out\n");
			return -ETIMEDOUT;
		}

		cpu_relax();
	} while (true);

	return 0;
}

/*
 * dwc3_frame_length_adjustment - Adjusts frame length if required
 * @dwc3: Pointer to our controller context structure
 */
static void dwc3_frame_length_adjustment(struct dwc3 *dwc)
{
	u32 reg;
	u32 dft;

	if (dwc->revision < DWC3_REVISION_250A)
		return;

	if (dwc->fladj == 0)
		return;

	reg = dwc3_readl(dwc->regs, DWC3_GFLADJ);
	dft = reg & DWC3_GFLADJ_30MHZ_MASK;
	if (!dev_WARN_ONCE(dwc->dev, dft == dwc->fladj,
	    "request value same as default, ignoring\n")) {
		reg &= ~DWC3_GFLADJ_30MHZ_MASK;
		reg |= DWC3_GFLADJ_30MHZ_SDBND_SEL | dwc->fladj;
		dwc3_writel(dwc->regs, DWC3_GFLADJ, reg);
	}
}

/**
 * dwc3_free_one_event_buffer - Frees one event buffer
 * @dwc: Pointer to our controller context structure
 * @evt: Pointer to event buffer to be freed
 */
static void dwc3_free_one_event_buffer(struct dwc3 *dwc,
		struct dwc3_event_buffer *evt)
{
	dma_free_coherent(dwc->sysdev, evt->length, evt->buf, evt->dma);
}

/**
 * dwc3_alloc_one_event_buffer - Allocates one event buffer structure
 * @dwc: Pointer to our controller context structure
 * @length: size of the event buffer
 *
 * Returns a pointer to the allocated event buffer structure on success
 * otherwise ERR_PTR(errno).
 */
static struct dwc3_event_buffer *dwc3_alloc_one_event_buffer(struct dwc3 *dwc,
		unsigned length)
{
	struct dwc3_event_buffer	*evt;

	evt = devm_kzalloc(dwc->dev, sizeof(*evt), GFP_KERNEL);
	if (!evt)
		return ERR_PTR(-ENOMEM);

	evt->dwc	= dwc;
	evt->length	= length;
	evt->buf	= dma_alloc_coherent(dwc->sysdev, length,
			&evt->dma, GFP_KERNEL);
	if (!evt->buf)
		return ERR_PTR(-ENOMEM);

	return evt;
}

/**
 * dwc3_free_event_buffers - frees all allocated event buffers
 * @dwc: Pointer to our controller context structure
 */
static void dwc3_free_event_buffers(struct dwc3 *dwc)
{
	struct dwc3_event_buffer	*evt;

	evt = dwc->ev_buf;
	if (evt)
		dwc3_free_one_event_buffer(dwc, evt);
}

/**
 * dwc3_alloc_event_buffers - Allocates @num event buffers of size @length
 * @dwc: pointer to our controller context structure
 * @length: size of event buffer
 *
 * Returns 0 on success otherwise negative errno. In the error case, dwc
 * may contain some buffers allocated but not all which were requested.
 */
static int dwc3_alloc_event_buffers(struct dwc3 *dwc, unsigned length)
{
	struct dwc3_event_buffer *evt;

	evt = dwc3_alloc_one_event_buffer(dwc, length);
	if (IS_ERR(evt)) {
		dev_err(dwc->dev, "can't allocate event buffer\n");
		return PTR_ERR(evt);
	}
	dwc->ev_buf = evt;

	return 0;
}

/**
 * dwc3_event_buffers_setup - setup our allocated event buffers
 * @dwc: pointer to our controller context structure
 *
 * Returns 0 on success otherwise negative errno.
 */
static int dwc3_event_buffers_setup(struct dwc3 *dwc)
{
	struct dwc3_event_buffer	*evt;

	evt = dwc->ev_buf;
	dwc3_trace(trace_dwc3_core,
			"Event buf %p dma %08llx length %d\n",
			evt->buf, (unsigned long long) evt->dma,
			evt->length);

	evt->lpos = 0;

	dwc3_writel(dwc->regs, DWC3_GEVNTADRLO(0),
			lower_32_bits(evt->dma));
	dwc3_writel(dwc->regs, DWC3_GEVNTADRHI(0),
			upper_32_bits(evt->dma));
	dwc3_writel(dwc->regs, DWC3_GEVNTSIZ(0),
			DWC3_GEVNTSIZ_SIZE(evt->length));
	dwc3_writel(dwc->regs, DWC3_GEVNTCOUNT(0), 0);

	return 0;
}

static void dwc3_event_buffers_cleanup(struct dwc3 *dwc)
{
	struct dwc3_event_buffer	*evt;

	evt = dwc->ev_buf;

	evt->lpos = 0;

	dwc3_writel(dwc->regs, DWC3_GEVNTADRLO(0), 0);
	dwc3_writel(dwc->regs, DWC3_GEVNTADRHI(0), 0);
	dwc3_writel(dwc->regs, DWC3_GEVNTSIZ(0), DWC3_GEVNTSIZ_INTMASK
			| DWC3_GEVNTSIZ_SIZE(0));
	dwc3_writel(dwc->regs, DWC3_GEVNTCOUNT(0), 0);
}

static int dwc3_alloc_scratch_buffers(struct dwc3 *dwc)
{
	if (!dwc->has_hibernation)
		return 0;

	if (!dwc->nr_scratch)
		return 0;

	dwc->scratchbuf = kmalloc_array(dwc->nr_scratch,
			DWC3_SCRATCHBUF_SIZE, GFP_KERNEL);
	if (!dwc->scratchbuf)
		return -ENOMEM;

	return 0;
}

static int dwc3_setup_scratch_buffers(struct dwc3 *dwc)
{
	dma_addr_t scratch_addr;
	u32 param;
	int ret;

	if (!dwc->has_hibernation)
		return 0;

	if (!dwc->nr_scratch)
		return 0;

	 /* should never fall here */
	if (!WARN_ON(dwc->scratchbuf))
		return 0;

	scratch_addr = dma_map_single(dwc->sysdev, dwc->scratchbuf,
			dwc->nr_scratch * DWC3_SCRATCHBUF_SIZE,
			DMA_BIDIRECTIONAL);
	if (dma_mapping_error(dwc->sysdev, scratch_addr)) {
		dev_err(dwc->sysdev, "failed to map scratch buffer\n");
		ret = -EFAULT;
		goto err0;
	}

	dwc->scratch_addr = scratch_addr;

	param = lower_32_bits(scratch_addr);

	ret = dwc3_send_gadget_generic_command(dwc,
			DWC3_DGCMD_SET_SCRATCHPAD_ADDR_LO, param);
	if (ret < 0)
		goto err1;

	param = upper_32_bits(scratch_addr);

	ret = dwc3_send_gadget_generic_command(dwc,
			DWC3_DGCMD_SET_SCRATCHPAD_ADDR_HI, param);
	if (ret < 0)
		goto err1;

	return 0;

err1:
	dma_unmap_single(dwc->sysdev, dwc->scratch_addr, dwc->nr_scratch *
			DWC3_SCRATCHBUF_SIZE, DMA_BIDIRECTIONAL);

err0:
	return ret;
}

static void dwc3_free_scratch_buffers(struct dwc3 *dwc)
{
	if (!dwc->has_hibernation)
		return;

	if (!dwc->nr_scratch)
		return;

	 /* should never fall here */
	if (!WARN_ON(dwc->scratchbuf))
		return;

	dma_unmap_single(dwc->sysdev, dwc->scratch_addr, dwc->nr_scratch *
			DWC3_SCRATCHBUF_SIZE, DMA_BIDIRECTIONAL);
	kfree(dwc->scratchbuf);
}

static void dwc3_core_num_eps(struct dwc3 *dwc)
{
	struct dwc3_hwparams	*parms = &dwc->hwparams;

	dwc->num_in_eps = DWC3_NUM_IN_EPS(parms);
	dwc->num_out_eps = DWC3_NUM_EPS(parms) - dwc->num_in_eps;

	dwc3_trace(trace_dwc3_core, "found %d IN and %d OUT endpoints",
			dwc->num_in_eps, dwc->num_out_eps);
}

static void dwc3_cache_hwparams(struct dwc3 *dwc)
{
	struct dwc3_hwparams	*parms = &dwc->hwparams;

	parms->hwparams0 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS0);
	parms->hwparams1 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS1);
	parms->hwparams2 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS2);
	parms->hwparams3 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS3);
	parms->hwparams4 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS4);
	parms->hwparams5 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS5);
	parms->hwparams6 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS6);
	parms->hwparams7 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS7);
	parms->hwparams8 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS8);
}

/**
 * dwc3_phy_setup - Configure USB PHY Interface of DWC3 Core
 * @dwc: Pointer to our controller context structure
 *
 * Returns 0 on success. The USB PHY interfaces are configured but not
 * initialized. The PHY interfaces and the PHYs get initialized together with
 * the core in dwc3_core_init.
 */
static int dwc3_phy_setup(struct dwc3 *dwc)
{
	u32 reg;
	int ret;

	reg = dwc3_readl(dwc->regs, DWC3_GUSB3PIPECTL(0));

	/*
	 * Above 1.94a, it is recommended to set DWC3_GUSB3PIPECTL_SUSPHY
	 * to '0' during coreConsultant configuration. So default value
	 * will be '0' when the core is reset. Application needs to set it
	 * to '1' after the core initialization is completed.
	 */
	if (dwc->revision > DWC3_REVISION_194A)
		reg |= DWC3_GUSB3PIPECTL_SUSPHY;

	if (dwc->u2ss_inp3_quirk)
		reg |= DWC3_GUSB3PIPECTL_U2SSINP3OK;

	if (dwc->dis_rxdet_inp3_quirk)
		reg |= DWC3_GUSB3PIPECTL_DISRXDETINP3;

	if (dwc->req_p1p2p3_quirk)
		reg |= DWC3_GUSB3PIPECTL_REQP1P2P3;

	if (dwc->del_p1p2p3_quirk)
		reg |= DWC3_GUSB3PIPECTL_DEP1P2P3_EN;

	if (dwc->del_phy_power_chg_quirk)
		reg |= DWC3_GUSB3PIPECTL_DEPOCHANGE;

	if (dwc->lfps_filter_quirk)
		reg |= DWC3_GUSB3PIPECTL_LFPSFILT;

	if (dwc->rx_detect_poll_quirk)
		reg |= DWC3_GUSB3PIPECTL_RX_DETOPOLL;

	if (dwc->tx_de_emphasis_quirk)
		reg |= DWC3_GUSB3PIPECTL_TX_DEEPH(dwc->tx_de_emphasis);

	if (dwc->dis_u3_susphy_quirk)
		reg &= ~DWC3_GUSB3PIPECTL_SUSPHY;

	if (dwc->dis_del_phy_power_chg_quirk)
		reg &= ~DWC3_GUSB3PIPECTL_DEPOCHANGE;

	dwc3_writel(dwc->regs, DWC3_GUSB3PIPECTL(0), reg);

	reg = dwc3_readl(dwc->regs, DWC3_GUSB2PHYCFG(0));

	/* Select the HS PHY interface */
	switch (DWC3_GHWPARAMS3_HSPHY_IFC(dwc->hwparams.hwparams3)) {
	case DWC3_GHWPARAMS3_HSPHY_IFC_UTMI_ULPI:
		if (dwc->hsphy_interface &&
				!strncmp(dwc->hsphy_interface, "utmi", 4)) {
			reg &= ~DWC3_GUSB2PHYCFG_ULPI_UTMI;
			break;
		} else if (dwc->hsphy_interface &&
				!strncmp(dwc->hsphy_interface, "ulpi", 4)) {
			reg |= DWC3_GUSB2PHYCFG_ULPI_UTMI;
			dwc3_writel(dwc->regs, DWC3_GUSB2PHYCFG(0), reg);
		} else {
			/* Relying on default value. */
			if (!(reg & DWC3_GUSB2PHYCFG_ULPI_UTMI))
				break;
		}
		/* FALLTHROUGH */
	case DWC3_GHWPARAMS3_HSPHY_IFC_ULPI:
		/* Making sure the interface and PHY are operational */
		ret = dwc3_soft_reset(dwc);
		if (ret)
			return ret;

		udelay(1);

		ret = dwc3_ulpi_init(dwc);
		if (ret)
			return ret;
		/* FALLTHROUGH */
	default:
		break;
	}

	switch (dwc->hsphy_mode) {
	case USBPHY_INTERFACE_MODE_UTMI:
		reg &= ~(DWC3_GUSB2PHYCFG_PHYIF_MASK |
		       DWC3_GUSB2PHYCFG_USBTRDTIM_MASK);
		reg |= DWC3_GUSB2PHYCFG_PHYIF(UTMI_PHYIF_8_BIT) |
		       DWC3_GUSB2PHYCFG_USBTRDTIM(USBTRDTIM_UTMI_8_BIT);
		break;
	case USBPHY_INTERFACE_MODE_UTMIW:
		reg &= ~(DWC3_GUSB2PHYCFG_PHYIF_MASK |
		       DWC3_GUSB2PHYCFG_USBTRDTIM_MASK);
		reg |= DWC3_GUSB2PHYCFG_PHYIF(UTMI_PHYIF_16_BIT) |
		       DWC3_GUSB2PHYCFG_USBTRDTIM(USBTRDTIM_UTMI_16_BIT);
		break;
	default:
		break;
	}

	/*
	 * Above 1.94a, it is recommended to set DWC3_GUSB2PHYCFG_SUSPHY to
	 * '0' during coreConsultant configuration. So default value will
	 * be '0' when the core is reset. Application needs to set it to
	 * '1' after the core initialization is completed.
	 */
	if (dwc->revision > DWC3_REVISION_194A)
		reg |= DWC3_GUSB2PHYCFG_SUSPHY;

	if (dwc->dis_u2_susphy_quirk)
		reg &= ~DWC3_GUSB2PHYCFG_SUSPHY;

	if (dwc->dis_enblslpm_quirk)
		reg &= ~DWC3_GUSB2PHYCFG_ENBLSLPM;

	if (dwc->dis_u2_freeclk_exists_quirk)
		reg &= ~DWC3_GUSB2PHYCFG_U2_FREECLK_EXISTS;

	dwc3_writel(dwc->regs, DWC3_GUSB2PHYCFG(0), reg);

	return 0;
}

static void dwc3_core_exit(struct dwc3 *dwc)
{
	dwc3_event_buffers_cleanup(dwc);

	usb_phy_shutdown(dwc->usb2_phy);
	usb_phy_shutdown(dwc->usb3_phy);
	phy_exit(dwc->usb2_generic_phy);
	phy_exit(dwc->usb3_generic_phy);

	usb_phy_set_suspend(dwc->usb2_phy, 1);
	usb_phy_set_suspend(dwc->usb3_phy, 1);
	phy_power_off(dwc->usb2_generic_phy);
	phy_power_off(dwc->usb3_generic_phy);
}

/**
 * dwc3_core_init - Low-level initialization of DWC3 Core
 * @dwc: Pointer to our controller context structure
 *
 * Returns 0 on success otherwise negative errno.
 */
static int dwc3_core_init(struct dwc3 *dwc)
{
	u32			hwparams4 = dwc->hwparams.hwparams4;
	u32			reg;
	int			ret;

	reg = dwc3_readl(dwc->regs, DWC3_GSNPSID);
	/* This should read as U3 followed by revision number */
	if ((reg & DWC3_GSNPSID_MASK) == 0x55330000) {
		/* Detected DWC_usb3 IP */
		dwc->revision = reg;
	} else if ((reg & DWC3_GSNPSID_MASK) == 0x33310000) {
		/* Detected DWC_usb31 IP */
		dwc->revision = dwc3_readl(dwc->regs, DWC3_VER_NUMBER);
		dwc->revision |= DWC3_REVISION_IS_DWC31;
	} else {
		dev_err(dwc->dev, "this is not a DesignWare USB3 DRD Core\n");
		ret = -ENODEV;
		goto err0;
	}

	/*
	 * Write Linux Version Code to our GUID register so it's easy to figure
	 * out which kernel version a bug was found.
	 */
	dwc3_writel(dwc->regs, DWC3_GUID, LINUX_VERSION_CODE);

	/* Handle USB2.0-only core configuration */
	if (DWC3_GHWPARAMS3_SSPHY_IFC(dwc->hwparams.hwparams3) ==
			DWC3_GHWPARAMS3_SSPHY_IFC_DIS) {
		if (dwc->maximum_speed == USB_SPEED_SUPER)
			dwc->maximum_speed = USB_SPEED_HIGH;
	}

	/* issue device SoftReset too */
	ret = dwc3_soft_reset(dwc);
	if (ret)
		goto err0;

	ret = dwc3_core_soft_reset(dwc);
	if (ret)
		goto err0;

	ret = dwc3_phy_setup(dwc);
	if (ret)
		goto err0;

	reg = dwc3_readl(dwc->regs, DWC3_GCTL);
	reg &= ~DWC3_GCTL_SCALEDOWN_MASK;

	switch (DWC3_GHWPARAMS1_EN_PWROPT(dwc->hwparams.hwparams1)) {
	case DWC3_GHWPARAMS1_EN_PWROPT_CLK:
		/**
		 * WORKAROUND: DWC3 revisions between 2.10a and 2.50a have an
		 * issue which would cause xHCI compliance tests to fail.
		 *
		 * Because of that we cannot enable clock gating on such
		 * configurations.
		 *
		 * Refers to:
		 *
		 * STAR#9000588375: Clock Gating, SOF Issues when ref_clk-Based
		 * SOF/ITP Mode Used
		 */
		if ((dwc->dr_mode == USB_DR_MODE_HOST ||
				dwc->dr_mode == USB_DR_MODE_OTG) &&
				(dwc->revision >= DWC3_REVISION_210A &&
				dwc->revision <= DWC3_REVISION_250A))
			reg |= DWC3_GCTL_DSBLCLKGTNG | DWC3_GCTL_SOFITPSYNC;
		else
			reg &= ~DWC3_GCTL_DSBLCLKGTNG;
		break;
	case DWC3_GHWPARAMS1_EN_PWROPT_HIB:
		/* enable hibernation here */
		dwc->nr_scratch = DWC3_GHWPARAMS4_HIBER_SCRATCHBUFS(hwparams4);

		/*
		 * REVISIT Enabling this bit so that host-mode hibernation
		 * will work. Device-mode hibernation is not yet implemented.
		 */
		reg |= DWC3_GCTL_GBLHIBERNATIONEN;
		break;
	default:
		dwc3_trace(trace_dwc3_core, "No power optimization available\n");
	}

	/* check if current dwc3 is on simulation board */
	if (dwc->hwparams.hwparams6 & DWC3_GHWPARAMS6_EN_FPGA) {
		dwc3_trace(trace_dwc3_core,
				"running on FPGA platform\n");
		dwc->is_fpga = true;
	}

	WARN_ONCE(dwc->disable_scramble_quirk && !dwc->is_fpga,
			"disable_scramble cannot be used on non-FPGA builds\n");

	if (dwc->disable_scramble_quirk && dwc->is_fpga)
		reg |= DWC3_GCTL_DISSCRAMBLE;
	else
		reg &= ~DWC3_GCTL_DISSCRAMBLE;

	if (dwc->u2exit_lfps_quirk)
		reg |= DWC3_GCTL_U2EXIT_LFPS;

	/*
	 * WORKAROUND: DWC3 revisions <1.90a have a bug
	 * where the device can fail to connect at SuperSpeed
	 * and falls back to high-speed mode which causes
	 * the device to enter a Connect/Disconnect loop
	 */
	if (dwc->revision < DWC3_REVISION_190A)
		reg |= DWC3_GCTL_U2RSTECN;

	dwc3_writel(dwc->regs, DWC3_GCTL, reg);

	dwc3_core_num_eps(dwc);

	ret = dwc3_setup_scratch_buffers(dwc);
	if (ret)
		goto err1;

	/* Adjust Frame Length */
	dwc3_frame_length_adjustment(dwc);

	usb_phy_set_suspend(dwc->usb2_phy, 0);
	usb_phy_set_suspend(dwc->usb3_phy, 0);
	ret = phy_power_on(dwc->usb2_generic_phy);
	if (ret < 0)
		goto err2;

	ret = phy_power_on(dwc->usb3_generic_phy);
	if (ret < 0)
		goto err3;

	ret = dwc3_event_buffers_setup(dwc);
	if (ret) {
		dev_err(dwc->dev, "failed to setup event buffers\n");
		goto err4;
	}

	switch (dwc->dr_mode) {
	case USB_DR_MODE_PERIPHERAL:
		dwc3_set_mode(dwc, DWC3_GCTL_PRTCAP_DEVICE);
		break;
	case USB_DR_MODE_HOST:
		dwc3_set_mode(dwc, DWC3_GCTL_PRTCAP_HOST);
		break;
	case USB_DR_MODE_OTG:
		dwc3_set_mode(dwc, DWC3_GCTL_PRTCAP_OTG);
		break;
	default:
		dev_warn(dwc->dev, "Unsupported mode %d\n", dwc->dr_mode);
		break;
	}

	/*
	 * ENDXFER polling is available on version 3.10a and later of
	 * the DWC_usb3 controller. It is NOT available in the
	 * DWC_usb31 controller.
	 */
	if (!dwc3_is_usb31(dwc) && dwc->revision >= DWC3_REVISION_310A) {
		reg = dwc3_readl(dwc->regs, DWC3_GUCTL2);
		reg |= DWC3_GUCTL2_RST_ACTBITLATER;
		dwc3_writel(dwc->regs, DWC3_GUCTL2, reg);
	}

	return 0;

err4:
	phy_power_off(dwc->usb3_generic_phy);

err3:
	phy_power_off(dwc->usb2_generic_phy);

err2:
	usb_phy_set_suspend(dwc->usb2_phy, 1);
	usb_phy_set_suspend(dwc->usb3_phy, 1);

err1:
	usb_phy_shutdown(dwc->usb2_phy);
	usb_phy_shutdown(dwc->usb3_phy);
	phy_exit(dwc->usb2_generic_phy);
	phy_exit(dwc->usb3_generic_phy);

err0:
	return ret;
}

static int dwc3_core_get_phy(struct dwc3 *dwc)
{
	struct device		*dev = dwc->dev;
	struct device_node	*node = dev->of_node;
	int ret;

	if (node) {
		dwc->usb2_phy = devm_usb_get_phy_by_phandle(dev, "usb-phy", 0);
		dwc->usb3_phy = devm_usb_get_phy_by_phandle(dev, "usb-phy", 1);
	} else {
		dwc->usb2_phy = devm_usb_get_phy(dev, USB_PHY_TYPE_USB2);
		dwc->usb3_phy = devm_usb_get_phy(dev, USB_PHY_TYPE_USB3);
	}

	if (IS_ERR(dwc->usb2_phy)) {
		ret = PTR_ERR(dwc->usb2_phy);
		if (ret == -ENXIO || ret == -ENODEV) {
			dwc->usb2_phy = NULL;
		} else if (ret == -EPROBE_DEFER) {
			return ret;
		} else {
			dev_err(dev, "no usb2 phy configured\n");
			return ret;
		}
	}

	if (IS_ERR(dwc->usb3_phy)) {
		ret = PTR_ERR(dwc->usb3_phy);
		if (ret == -ENXIO || ret == -ENODEV) {
			dwc->usb3_phy = NULL;
		} else if (ret == -EPROBE_DEFER) {
			return ret;
		} else {
			dev_err(dev, "no usb3 phy configured\n");
			return ret;
		}
	}

	dwc->usb2_generic_phy = devm_phy_get(dev, "usb2-phy");
	if (IS_ERR(dwc->usb2_generic_phy)) {
		ret = PTR_ERR(dwc->usb2_generic_phy);
		if (ret == -ENOSYS || ret == -ENODEV) {
			dwc->usb2_generic_phy = NULL;
		} else if (ret == -EPROBE_DEFER) {
			return ret;
		} else {
			dev_err(dev, "no usb2 phy configured\n");
			return ret;
		}
	}

	dwc->usb3_generic_phy = devm_phy_get(dev, "usb3-phy");
	if (IS_ERR(dwc->usb3_generic_phy)) {
		ret = PTR_ERR(dwc->usb3_generic_phy);
		if (ret == -ENOSYS || ret == -ENODEV) {
			dwc->usb3_generic_phy = NULL;
		} else if (ret == -EPROBE_DEFER) {
			return ret;
		} else {
			dev_err(dev, "no usb3 phy configured\n");
			return ret;
		}
	}

	return 0;
}

static int dwc3_drd_start_host(struct dwc3 *dwc, int on, bool skip);
static int dwc3_drd_start_gadget(struct dwc3 *dwc, int on);

/* dwc->lock must be held */
void dwc3_drd_statemachine(struct dwc3 *dwc, int id, int vbus)
{
	enum usb_otg_state new_state;
	int protocol;

	if (id == dwc->otg_fsm.id && vbus == dwc->otg_fsm.b_sess_vld)
		return;

	dwc->otg_fsm.id = id;
	dwc->otg_fsm.b_sess_vld = vbus;

	if (!id) {
		new_state = OTG_STATE_A_HOST;
	} else{
		if (vbus)
			new_state = OTG_STATE_B_PERIPHERAL;
		else
			new_state = OTG_STATE_B_IDLE;
	}

	if (dwc->otg.state == new_state)
		return;

	protocol = dwc->otg_fsm.protocol;
	switch (new_state) {
	case OTG_STATE_B_IDLE:
		if (protocol == PROTO_GADGET)
			dwc3_drd_start_gadget(dwc, 0);
		else if (protocol == PROTO_HOST)
			dwc3_drd_start_host(dwc, 0, 0);
		dwc->otg_fsm.protocol = PROTO_UNDEF;
		break;
	case OTG_STATE_B_PERIPHERAL:
		if (protocol == PROTO_HOST)
			dwc3_drd_start_host(dwc, 0, 0);

		if (protocol != PROTO_GADGET) {
			dwc->otg_fsm.protocol = PROTO_GADGET;
			dwc3_drd_start_gadget(dwc, 1);
		}
		break;
	case OTG_STATE_A_HOST:
		if (protocol == PROTO_GADGET)
			dwc3_drd_start_gadget(dwc, 0);

		if (protocol != PROTO_HOST) {
			dwc->otg_fsm.protocol = PROTO_HOST;
			dwc3_drd_start_host(dwc, 1, 0);
		}
		break;
	default:
		dev_err(dwc->dev, "drd: invalid usb-drd state: %s\n",
			usb_otg_state_string(new_state));
		return;
	}

	dwc->otg.state = new_state;
}

/* dwc->lock must be held */
static void dwc3_otg_fsm_sync(struct dwc3 *dwc)
{
	u32 reg, osts;
	int id, vbus;

	/*
	 * calling dwc3_otg_fsm_sync() during resume breaks host
	 * if adapter was removed during suspend as xhci driver
	 * is not prepared to see hcd removal before xhci_resume.
	 */
	if (dwc->otg_prevent_sync)
		return;

	if (dwc->edev) {
		/* get ID */
		id = extcon_get_state(dwc->edev, EXTCON_USB_HOST);
		/* Host means ID == 0 */
		id = !id;

		/* get VBUS */
		vbus = extcon_get_state(dwc->edev, EXTCON_USB);
		dwc3_drd_statemachine(dwc, id, vbus);

		return;
	}

	do {
		reg = dwc3_readl(dwc->regs, DWC3_OSTS);

		id = !!(reg & DWC3_OSTS_CONIDSTS);
		vbus = !!(reg & DWC3_OSTS_BSESVLD);

		dwc3_drd_statemachine(dwc, id, vbus);
		osts = dwc3_readl(dwc->regs, DWC3_OSTS);
		/*
		 * OTG status might have changed and if we don't re-check
		 * here we will loose events as OTG events have been
		 * temporarily disabled.
		 */
		/* FIXME: there is still a small time window where we can
		 * miss an OTG event. i.e. from here till where we enable the
		 * OEVTEN in OTG IRQ thread handler. Try another solution
		 * where OEVTEN can be enabled but otg irq in GIC is disabled.
		 * so any OTG changes can re-trigger the OTG IRQ after current
		 * ISR ends.
		 */
	} while (osts != reg);
}

static void dwc3_drd_work(struct work_struct *work)
{
	struct dwc3 *dwc = container_of(work, struct dwc3,
					otg_work);

	spin_lock(&dwc->lock);
	dwc3_otg_fsm_sync(dwc);
	spin_unlock(&dwc->lock);
}

static int dwc3_drd_notifier(struct notifier_block *nb,
			     unsigned long event, void *ptr)
{
	struct dwc3 *dwc = container_of(nb, struct dwc3, edev_nb);

	if (!dwc->otg_prevent_sync)
		queue_work(system_power_efficient_wq, &dwc->otg_work);

	return NOTIFY_DONE;
}

static void dwc3_otg_mask_irq(struct dwc3 *dwc)
{
	dwc->oevten = dwc3_readl(dwc->regs, DWC3_OEVTEN);
	dwc3_writel(dwc->regs, DWC3_OEVTEN, 0);
}

static void dwc3_otg_unmask_irq(struct dwc3 *dwc)
{
	dwc3_writel(dwc->regs, DWC3_OEVTEN, dwc->oevten);
}

static void dwc3_otg_disable_events(struct dwc3 *dwc, u32 disable_mask)
{
	dwc->oevten &= ~(disable_mask);
	dwc3_writel(dwc->regs, DWC3_OEVTEN, dwc->oevten);
}

static void dwc3_otg_enable_events(struct dwc3 *dwc, u32 enable_mask)
{
	dwc->oevten |= (enable_mask);
	dwc3_writel(dwc->regs, DWC3_OEVTEN, dwc->oevten);
}

#define DWC3_OTG_ALL_EVENTS	(DWC3_OEVTEN_XHCIRUNSTPSETEN | \
		DWC3_OEVTEN_DEVRUNSTPSETEN | DWC3_OEVTEN_HIBENTRYEN | \
		DWC3_OEVTEN_CONIDSTSCHNGEN | DWC3_OEVTEN_HRRCONFNOTIFEN | \
		DWC3_OEVTEN_HRRINITNOTIFEN | DWC3_OEVTEN_ADEVIDLEEN | \
		DWC3_OEVTEN_ADEVBHOSTENDEN | DWC3_OEVTEN_ADEVHOSTEN | \
		DWC3_OEVTEN_ADEVHNPCHNGEN | DWC3_OEVTEN_ADEVSRPDETEN | \
		DWC3_OEVTEN_ADEVSESSENDDETEN | DWC3_OEVTEN_BDEVHOSTENDEN | \
		DWC3_OEVTEN_BDEVHNPCHNGEN | DWC3_OEVTEN_BDEVSESSVLDDETEN | \
		DWC3_OEVTEN_BDEVVBUSCHNGE)

static irqreturn_t dwc3_otg_thread_irq(int irq, void *_dwc)
{
	struct dwc3 *dwc = _dwc;

	spin_lock(&dwc->lock);
	if ((dwc->otg_fsm.protocol == PROTO_HOST) &&
	    !(dwc->oevt & DWC3_OEVT_DEVICEMODE)) {
		dwc3_drd_start_host(dwc, true, 1);
	}

	dwc3_otg_fsm_sync(dwc);
	dwc3_otg_unmask_irq(dwc);
	spin_unlock(&dwc->lock);

	return IRQ_HANDLED;
}

static irqreturn_t dwc3_otg_irq(int irq, void *_dwc)
{
	struct dwc3 *dwc = _dwc;
	irqreturn_t ret = IRQ_NONE;

	dwc->oevt = dwc3_readl(dwc->regs, DWC3_OEVT);
	if (dwc->oevt) {
		dwc3_writel(dwc->regs, DWC3_OEVT, dwc->oevt);
		/* ignore OTG events if not in OTG mode */
		if (dwc->sw_drd_mode)
			return IRQ_HANDLED;
		dwc3_otg_mask_irq(dwc);
		ret = IRQ_WAKE_THREAD;
	}

	return ret;
}

/* --------------------- Dual-Role management ------------------------------- */
static void dwc3_otgregs_init(struct dwc3 *dwc)
{
	u32 reg;

	/*
	 * Prevent host/device reset from resetting OTG core.
	 * If we don't do this then xhci_reset (USBCMD.HCRST) will reset
	 * the signal outputs sent to the PHY, the OTG FSM logic of the
	 * core and also the resets to the VBUS filters inside the core.
	 */
	reg = dwc3_readl(dwc->regs, DWC3_OCFG);
	reg |= DWC3_OCFG_SFTRSTMASK;
	dwc3_writel(dwc->regs, DWC3_OCFG, reg);

	/* Disable hibernation for simplicity */
	reg = dwc3_readl(dwc->regs, DWC3_GCTL);
	reg &= ~DWC3_GCTL_GBLHIBERNATIONEN;
	dwc3_writel(dwc->regs, DWC3_GCTL, reg);

	/*
	 * Initialize OTG registers as per
	 * Figure 11-4 OTG Driver Overall Programming Flow
	 */
	/* OCFG.SRPCap = 0, OCFG.HNPCap = 0 */
	reg = dwc3_readl(dwc->regs, DWC3_OCFG);
	reg &= ~(DWC3_OCFG_SRPCAP | DWC3_OCFG_HNPCAP);
	dwc3_writel(dwc->regs, DWC3_OCFG, reg);
	/* OEVT = FFFF */
	dwc3_writel(dwc->regs, DWC3_OEVT, ~0);
	/* OEVTEN = 0 */
	dwc3_otg_disable_events(dwc, DWC3_OTG_ALL_EVENTS);
	/* OEVTEN.ConIDStsChngEn = 1. Instead we enable all events */
	dwc3_otg_enable_events(dwc, DWC3_OTG_ALL_EVENTS);
	/*
	 * OCTL.PeriMode = 1, OCTL.DevSetHNPEn = 0, OCTL.HstSetHNPEn = 0,
	 * OCTL.HNPReq = 0
	 */
	reg = dwc3_readl(dwc->regs, DWC3_OCTL);
	reg |= DWC3_OCTL_PERIMODE;
	reg &= ~(DWC3_OCTL_DEVSETHNPEN | DWC3_OCTL_HSTSETHNPEN |
		 DWC3_OCTL_HNPREQ);
	dwc3_writel(dwc->regs, DWC3_OCTL, reg);
}

/* dwc->lock must be held */
static int dwc3_drd_start_host(struct dwc3 *dwc, int on, bool skip)
{
	u32 reg;

	if (!dwc->edev && !dwc->sw_drd_mode)
		goto otg;

	if (on)
		dwc3_set_mode(dwc, DWC3_GCTL_PRTCAP_HOST);

	if (!skip) {
		spin_unlock(&dwc->lock);

		/* start or stop the HCD */
		if (on)
			dwc3_host_init(dwc);
		else
			dwc3_host_exit(dwc);

		spin_lock(&dwc->lock);
	}

	return 0;

otg:
	/* switch OTG core */
	if (on) {
		/* As per Figure 11-10 A-Device Flow Diagram */
		/* OCFG.HNPCap = 0, OCFG.SRPCap = 0 */
		reg = dwc3_readl(dwc->regs, DWC3_OCFG);
		reg &= ~(DWC3_OCFG_SRPCAP | DWC3_OCFG_HNPCAP);
		dwc3_writel(dwc->regs, DWC3_OCFG, reg);

		/*
		 * OCTL.PeriMode=0, OCTL.TermSelDLPulse = 0,
		 * OCTL.DevSetHNPEn = 0, OCTL.HstSetHNPEn = 0
		 */
		reg = dwc3_readl(dwc->regs, DWC3_OCTL);
		reg &= ~(DWC3_OCTL_PERIMODE | DWC3_OCTL_TERMSELIDPULSE |
			 DWC3_OCTL_DEVSETHNPEN | DWC3_OCTL_HSTSETHNPEN);
		dwc3_writel(dwc->regs, DWC3_OCTL, reg);

		/*
		 * OCFG.DisPrtPwrCutoff = 0/1
		 */
		reg = dwc3_readl(dwc->regs, DWC3_OCFG);
		reg &= ~DWC3_OCFG_DISPWRCUTTOFF;
		dwc3_writel(dwc->regs, DWC3_OCFG, reg);

		/* start the xHCI host driver */
		if (!skip) {
			spin_unlock(&dwc->lock);
			dwc3_host_init(dwc);
			spin_lock(&dwc->lock);
		}

		/*
		 * OCFG.SRPCap = 1, OCFG.HNPCap = GHWPARAMS6.HNP_CAP
		 * We don't want SRP/HNP for simple dual-role so leave
		 * these disabled.
		 */

		/*
		 * OEVTEN.OTGADevHostEvntEn = 1
		 * OEVTEN.OTGADevSessEndDetEvntEn = 1
		 * We don't want HNP/role-swap so leave these disabled.
		 */

		/* GUSB2PHYCFG.ULPIAutoRes = 1/0, GUSB2PHYCFG.SusPHY = 1 */
		if (!dwc->dis_u2_susphy_quirk) {
			reg = dwc3_readl(dwc->regs, DWC3_GUSB2PHYCFG(0));
			reg |= DWC3_GUSB2PHYCFG_SUSPHY;
			dwc3_writel(dwc->regs, DWC3_GUSB2PHYCFG(0), reg);
		}

		/* Set Port Power to enable VBUS: OCTL.PrtPwrCtl = 1 */
		reg = dwc3_readl(dwc->regs, DWC3_OCTL);
		reg |= DWC3_OCTL_PRTPWRCTL;
		dwc3_writel(dwc->regs, DWC3_OCTL, reg);
	} else {
		/*
		 * Exit from A-device flow as per
		 * Figure 11-4 OTG Driver Overall Programming Flow
		 */
		/* stop the HCD */
		if (!skip) {
			spin_unlock(&dwc->lock);
			dwc3_host_exit(dwc);
			spin_lock(&dwc->lock);
		}

		/*
		 * OEVTEN.OTGADevBHostEndEvntEn=0, OEVTEN.OTGADevHNPChngEvntEn=0
		 * OEVTEN.OTGADevSessEndDetEvntEn=0,
		 * OEVTEN.OTGADevHostEvntEn = 0
		 * But we don't disable any OTG events
		 */

		/* OCTL.HstSetHNPEn = 0, OCTL.PrtPwrCtl=0 */
		reg = dwc3_readl(dwc->regs, DWC3_OCTL);
		reg &= ~(DWC3_OCTL_HSTSETHNPEN | DWC3_OCTL_PRTPWRCTL);
		dwc3_writel(dwc->regs, DWC3_OCTL, reg);

		/* Initialize OTG registers */
		dwc3_otgregs_init(dwc);
	}

	return 0;
}

/* dwc->lock must be held */
static int dwc3_drd_start_gadget(struct dwc3 *dwc, int on)
{
	u32 reg;

	if (on)
		dwc3_event_buffers_setup(dwc);

	if (!dwc->edev && !dwc->sw_drd_mode)
		goto otg;

	if (on) {
		dwc3_set_mode(dwc, DWC3_GCTL_PRTCAP_DEVICE);
		/* start the Peripheral driver  */
		if (dwc->gadget_driver) {
			__dwc3_gadget_start(dwc);
			if (dwc->gadget_pullup)
				dwc3_gadget_run_stop(dwc, true, false);
		}
	} else {
		/* stop the Peripheral driver */
		if (dwc->gadget_driver) {
			if (dwc->gadget_pullup)
				dwc3_gadget_run_stop(dwc, false, false);
			spin_unlock(&dwc->lock);
			if (dwc->gadget_driver->disconnect)
				dwc->gadget_driver->disconnect(&dwc->gadget);
			spin_lock(&dwc->lock);
			__dwc3_gadget_stop(dwc);
		}
	}

	return 0;

otg:
	if (on) {
		/* As per Figure 11-20 B-Device Flow Diagram */

		/*
		 * OCFG.HNPCap = GHWPARAMS6.HNP_CAP, OCFG.SRPCap = 1
		 * but we set them to 0 for simple dual-role operation.
		 */
		reg = dwc3_readl(dwc->regs, DWC3_OCFG);
		reg &= ~(DWC3_OCFG_SRPCAP | DWC3_OCFG_HNPCAP);
		/* OCFG.OTGSftRstMsk = 0/1 */
		reg |= DWC3_OCFG_SFTRSTMASK;
		dwc3_writel(dwc->regs, DWC3_OCFG, reg);
		/*
		 * OCTL.PeriMode = 1
		 * OCTL.TermSelDLPulse = 0/1, OCTL.HNPReq = 0
		 * OCTL.DevSetHNPEn = 0, OCTL.HstSetHNPEn = 0
		 */
		reg = dwc3_readl(dwc->regs, DWC3_OCTL);
		reg |= DWC3_OCTL_PERIMODE;
		reg &= ~(DWC3_OCTL_TERMSELIDPULSE | DWC3_OCTL_HNPREQ |
			 DWC3_OCTL_DEVSETHNPEN | DWC3_OCTL_HSTSETHNPEN);
		dwc3_writel(dwc->regs, DWC3_OCTL, reg);
		/* OEVTEN.OTGBDevSesVldDetEvntEn = 1 */
		dwc3_otg_enable_events(dwc, DWC3_OEVT_BDEVSESSVLDDET);
		/* GUSB2PHYCFG.ULPIAutoRes = 0, GUSB2PHYCFG0.SusPHY = 1 */
		if (!dwc->dis_u2_susphy_quirk) {
			reg = dwc3_readl(dwc->regs, DWC3_GUSB2PHYCFG(0));
			reg |= DWC3_GUSB2PHYCFG_SUSPHY;
			dwc3_writel(dwc->regs, DWC3_GUSB2PHYCFG(0), reg);
		}
		/* GCTL.GblHibernationEn = 0 */
		reg = dwc3_readl(dwc->regs, DWC3_GCTL);
		reg &= ~DWC3_GCTL_GBLHIBERNATIONEN;
		dwc3_writel(dwc->regs, DWC3_GCTL, reg);

		/* start the Peripheral driver  */
		if (dwc->gadget_driver) {
			__dwc3_gadget_start(dwc);
			if (dwc->gadget_pullup)
				dwc3_gadget_run_stop(dwc, true, false);
		}
	} else {
		/*
		 * Exit from B-device flow as per
		 * Figure 11-4 OTG Driver Overall Programming Flow
		 */
		/* stop the Peripheral driver */
		if (dwc->gadget_driver) {
			if (dwc->gadget_pullup)
				dwc3_gadget_run_stop(dwc, false, false);
			spin_unlock(&dwc->lock);
			if (dwc->gadget_driver->disconnect)
				dwc->gadget_driver->disconnect(&dwc->gadget);
			spin_lock(&dwc->lock);
			__dwc3_gadget_stop(dwc);
		}

		/*
		 * OEVTEN.OTGBDevHNPChngEvntEn = 0
		 * OEVTEN.OTGBDevVBusChngEvntEn = 0
		 * OEVTEN.OTGBDevBHostEndEvntEn = 0
		 */
		reg = dwc3_readl(dwc->regs, DWC3_OEVTEN);
		reg &= ~(DWC3_OEVT_BDEVHNPCHNG | DWC3_OEVT_BDEVVBUSCHNG |
			 DWC3_OEVT_BDEVBHOSTEND);
		dwc3_writel(dwc->regs, DWC3_OEVTEN, reg);

		/* OCTL.DevSetHNPEn = 0, OCTL.HNPReq = 0, OCTL.PeriMode=1 */
		reg = dwc3_readl(dwc->regs, DWC3_OCTL);
		reg &= ~(DWC3_OCTL_DEVSETHNPEN | DWC3_OCTL_HNPREQ);
		reg |= DWC3_OCTL_PERIMODE;
		dwc3_writel(dwc->regs, DWC3_OCTL, reg);

		/* Initialize OTG registers */
		dwc3_otgregs_init(dwc);
	}

	return 0;
}

static int dwc3_otg_get_irq(struct dwc3 *dwc)
{
	struct platform_device *dwc3_pdev = to_platform_device(dwc->dev);
	int irq;

	irq = platform_get_irq_byname(dwc3_pdev, "otg");
	if (irq > 0)
		goto out;

	if (irq == -EPROBE_DEFER)
		goto out;

	irq = platform_get_irq_byname(dwc3_pdev, "dwc_usb3");
	if (irq > 0)
		goto out;

	if (irq == -EPROBE_DEFER)
		goto out;

	irq = platform_get_irq(dwc3_pdev, 0);
	if (irq > 0)
		goto out;

	if (irq != -EPROBE_DEFER)
		dev_err(dwc->dev, "missing otg IRQ\n");

	if (!irq)
		irq = -EINVAL;

out:
	return irq;
}

/* dwc->lock must be held */
static void dwc3_otg_core_init(struct dwc3 *dwc)
{
	u32 reg;

	/* force drd state machine update the first time */
	dwc->otg_fsm.b_sess_vld = -1;
	dwc->otg_fsm.id = -1;

	if (dwc->edev)
		return;

	/*
	 * As per Figure 11-4 OTG Driver Overall Programming Flow,
	 * block "Initialize GCTL for OTG operation".
	 */
	/* GCTL.PrtCapDir=2'b11 */
	dwc3_set_mode(dwc, DWC3_GCTL_PRTCAP_OTG);
	/* GUSB2PHYCFG0.SusPHY=0 */
	reg = dwc3_readl(dwc->regs, DWC3_GUSB2PHYCFG(0));
	reg &= ~DWC3_GUSB2PHYCFG_SUSPHY;
	dwc3_writel(dwc->regs, DWC3_GUSB2PHYCFG(0), reg);

	/* Initialize OTG registers */
	dwc3_otgregs_init(dwc);
}

/* dwc->lock must be held */
static void dwc3_otg_core_exit(struct dwc3 *dwc)
{
	if (dwc->edev)
		return;

	/* disable all otg irqs */
	dwc3_otg_disable_events(dwc, DWC3_OTG_ALL_EVENTS);
	/* clear all events */
	dwc3_writel(dwc->regs, DWC3_OEVT, ~0);
}

static int dwc3_drd_init(struct dwc3 *dwc)
{
	int ret, irq;
	unsigned long flags;

	INIT_WORK(&dwc->otg_work, dwc3_drd_work);

	/* If extcon device is present we don't rely on OTG core for ID event */
	if (dwc->edev) {
		int id, vbus;

		dwc->edev_nb.notifier_call = dwc3_drd_notifier;
		ret = extcon_register_notifier(dwc->edev, EXTCON_USB,
					       &dwc->edev_nb);
		if (ret < 0) {
			dev_err(dwc->dev, "Couldn't register USB cable notifier\n");
			return -ENODEV;
		}

		ret = extcon_register_notifier(dwc->edev, EXTCON_USB_HOST,
					       &dwc->edev_nb);
		if (ret < 0) {
			dev_err(dwc->dev, "Couldn't register USB-HOST cable notifier\n");
			ret = -ENODEV;
			goto extcon_fail;
		}

		/* sanity check id & vbus states */
		id = extcon_get_state(dwc->edev, EXTCON_USB_HOST);
		vbus = extcon_get_state(dwc->edev, EXTCON_USB);
		if (id < 0 || vbus < 0) {
			dev_err(dwc->dev, "Invalid USB cable state. id %d, vbus %d\n",
				id, vbus);
			ret = -ENODEV;
			goto fail;
		}

		ret = dwc3_gadget_init(dwc);
		if (ret)
			goto fail;

		spin_lock_irqsave(&dwc->lock, flags);
		dwc3_otg_core_init(dwc);
		dwc3_otg_fsm_sync(dwc);
		spin_unlock_irqrestore(&dwc->lock, flags);

		return 0;

fail:
		extcon_unregister_notifier(dwc->edev, EXTCON_USB_HOST,
					   &dwc->edev_nb);
extcon_fail:
		extcon_unregister_notifier(dwc->edev, EXTCON_USB,
					   &dwc->edev_nb);

		return ret;
	}

	irq = dwc3_otg_get_irq(dwc);
	if (irq < 0)
		return irq;

	dwc->otg_irq = irq;

	/* disable all otg irqs */
	dwc3_otg_disable_events(dwc, DWC3_OTG_ALL_EVENTS);
	/* clear all events */
	dwc3_writel(dwc->regs, DWC3_OEVT, ~0);

	irq_set_status_flags(dwc->otg_irq, IRQ_NOAUTOEN);
	ret = request_threaded_irq(dwc->otg_irq, dwc3_otg_irq,
				   dwc3_otg_thread_irq,
				   IRQF_SHARED, "dwc3-otg", dwc);
	if (ret) {
		dev_err(dwc->dev, "failed to request irq #%d --> %d\n",
			dwc->otg_irq, ret);
		ret = -ENODEV;
		return ret;
	}

	ret = dwc3_gadget_init(dwc);
	if (ret) {
		free_irq(dwc->otg_irq, dwc);
		return ret;
	}

	spin_lock_irqsave(&dwc->lock, flags);
	dwc3_otg_core_init(dwc);
	dwc3_otg_fsm_sync(dwc);
	spin_unlock_irqrestore(&dwc->lock, flags);

	return 0;
}

static void dwc3_drd_exit(struct dwc3 *dwc)
{
	unsigned long flags;

	spin_lock(&dwc->lock);
	dwc->otg_prevent_sync = true;
	spin_unlock(&dwc->lock);
	cancel_work_sync(&dwc->otg_work);

	spin_lock_irqsave(&dwc->lock, flags);
	dwc3_otg_core_exit(dwc);
	if (dwc->otg_fsm.protocol == PROTO_HOST)
		dwc3_drd_start_host(dwc, 0, 0);
	dwc->otg_fsm.protocol = PROTO_UNDEF;
	if (dwc->edev) {
		extcon_unregister_notifier(dwc->edev, EXTCON_USB_HOST,
					   &dwc->edev_nb);
		extcon_unregister_notifier(dwc->edev, EXTCON_USB,
					   &dwc->edev_nb);
	} else {
		free_irq(dwc->otg_irq, dwc);
	}
	spin_unlock_irqrestore(&dwc->lock, flags);

	dwc3_gadget_exit(dwc);
}

/* -------------------------------------------------------------------------- */

static int dwc3_core_init_mode(struct dwc3 *dwc)
{
	struct device *dev = dwc->dev;
	int ret;

	switch (dwc->dr_mode) {
	case USB_DR_MODE_PERIPHERAL:
		ret = dwc3_gadget_init(dwc);
		if (ret) {
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "failed to initialize gadget\n");
			return ret;
		}
		break;
	case USB_DR_MODE_HOST:
		ret = dwc3_host_init(dwc);
		if (ret) {
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "failed to initialize host\n");
			return ret;
		}
		break;
	case USB_DR_MODE_OTG:
		ret = dwc3_drd_init(dwc);
		if (ret) {
			if (ret != -EPROBE_DEFER)
				dev_err(dev, "failed to initialize dual-role\n");
			return ret;
		}
		break;
	default:
		dev_err(dev, "Unsupported mode of operation %d\n", dwc->dr_mode);
		return -EINVAL;
	}

	return 0;
}

static void dwc3_core_exit_mode(struct dwc3 *dwc)
{
	switch (dwc->dr_mode) {
	case USB_DR_MODE_PERIPHERAL:
		dwc3_gadget_exit(dwc);
		break;
	case USB_DR_MODE_HOST:
		dwc3_host_exit(dwc);
		break;
	case USB_DR_MODE_OTG:
		dwc3_drd_exit(dwc);
		break;
	default:
		/* do nothing */
		break;
	}
}

#define DWC3_ALIGN_MASK		(16 - 1)

static int dwc3_probe(struct platform_device *pdev)
{
	struct device		*dev = &pdev->dev;
	struct resource		*res;
	struct dwc3		*dwc;
	u8			lpm_nyet_threshold;
	u8			tx_de_emphasis;
	u8			hird_threshold;

	int			ret;

	void __iomem		*regs;
	void			*mem;

	mem = devm_kzalloc(dev, sizeof(*dwc) + DWC3_ALIGN_MASK, GFP_KERNEL);
	if (!mem)
		return -ENOMEM;

	dwc = PTR_ALIGN(mem, DWC3_ALIGN_MASK + 1);
	dwc->mem = mem;
	dwc->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "missing memory resource\n");
		return -ENODEV;
	}

	dwc->xhci_resources[0].start = res->start;
	dwc->xhci_resources[0].end = dwc->xhci_resources[0].start +
					DWC3_XHCI_REGS_END;
	dwc->xhci_resources[0].flags = res->flags;
	dwc->xhci_resources[0].name = res->name;

	res->start += DWC3_GLOBALS_REGS_START;

	/*
	 * Request memory region but exclude xHCI regs,
	 * since it will be requested by the xhci-plat driver.
	 */
	regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(regs)) {
		ret = PTR_ERR(regs);
		goto err0;
	}

	dwc->regs	= regs;
	dwc->regs_size	= resource_size(res);

	/* default to highest possible threshold */
	lpm_nyet_threshold = 0xff;

	/* default to -3.5dB de-emphasis */
	tx_de_emphasis = 1;

	/*
	 * default to assert utmi_sleep_n and use maximum allowed HIRD
	 * threshold value of 0b1100
	 */
	hird_threshold = 12;

	dwc->maximum_speed = usb_get_maximum_speed(dev);
	dwc->dr_mode = usb_get_dr_mode(dev);
	dwc->hsphy_mode = of_usb_get_phy_mode(dev->of_node);

	dwc->sysdev_is_parent = device_property_read_bool(dev,
				"linux,sysdev_is_parent");
	if (dwc->sysdev_is_parent)
		dwc->sysdev = dwc->dev->parent;
	else
		dwc->sysdev = dwc->dev;

	dwc->has_lpm_erratum = device_property_read_bool(dev,
				"snps,has-lpm-erratum");
	device_property_read_u8(dev, "snps,lpm-nyet-threshold",
				&lpm_nyet_threshold);
	dwc->is_utmi_l1_suspend = device_property_read_bool(dev,
				"snps,is-utmi-l1-suspend");
	device_property_read_u8(dev, "snps,hird-threshold",
				&hird_threshold);
	dwc->usb3_lpm_capable = device_property_read_bool(dev,
				"snps,usb3_lpm_capable");

	dwc->disable_scramble_quirk = device_property_read_bool(dev,
				"snps,disable_scramble_quirk");
	dwc->u2exit_lfps_quirk = device_property_read_bool(dev,
				"snps,u2exit_lfps_quirk");
	dwc->u2ss_inp3_quirk = device_property_read_bool(dev,
				"snps,u2ss_inp3_quirk");
	dwc->req_p1p2p3_quirk = device_property_read_bool(dev,
				"snps,req_p1p2p3_quirk");
	dwc->del_p1p2p3_quirk = device_property_read_bool(dev,
				"snps,del_p1p2p3_quirk");
	dwc->del_phy_power_chg_quirk = device_property_read_bool(dev,
				"snps,del_phy_power_chg_quirk");
	dwc->lfps_filter_quirk = device_property_read_bool(dev,
				"snps,lfps_filter_quirk");
	dwc->rx_detect_poll_quirk = device_property_read_bool(dev,
				"snps,rx_detect_poll_quirk");
	dwc->dis_u3_susphy_quirk = device_property_read_bool(dev,
				"snps,dis_u3_susphy_quirk");
	dwc->dis_u2_susphy_quirk = device_property_read_bool(dev,
				"snps,dis_u2_susphy_quirk");
	dwc->dis_enblslpm_quirk = device_property_read_bool(dev,
				"snps,dis_enblslpm_quirk");
	dwc->dis_rxdet_inp3_quirk = device_property_read_bool(dev,
				"snps,dis_rxdet_inp3_quirk");
	dwc->dis_u2_freeclk_exists_quirk = device_property_read_bool(dev,
				"snps,dis-u2-freeclk-exists-quirk");
	dwc->dis_del_phy_power_chg_quirk = device_property_read_bool(dev,
				"snps,dis-del-phy-power-chg-quirk");

	dwc->tx_de_emphasis_quirk = device_property_read_bool(dev,
				"snps,tx_de_emphasis_quirk");
	device_property_read_u8(dev, "snps,tx_de_emphasis",
				&tx_de_emphasis);
	device_property_read_string(dev, "snps,hsphy_interface",
				    &dwc->hsphy_interface);
	device_property_read_u32(dev, "snps,quirk-frame-length-adjustment",
				 &dwc->fladj);
	dwc->devctrl_halt_quirk = device_property_read_bool(dev,
				"snps,devctrl_halt_quirk");

	dwc->lpm_nyet_threshold = lpm_nyet_threshold;
	dwc->tx_de_emphasis = tx_de_emphasis;

	dwc->hird_threshold = hird_threshold
		| (dwc->is_utmi_l1_suspend << 4);

	if (dev->of_node) {
		if (of_property_read_bool(dev->of_node, "extcon"))
			dwc->edev = extcon_get_edev_by_phandle(dev, 0);

		if (IS_ERR(dwc->edev))
			return PTR_ERR(dwc->edev);
	}

	platform_set_drvdata(pdev, dwc);
	dwc3_cache_hwparams(dwc);

	ret = dwc3_core_get_phy(dwc);
	if (ret)
		goto err0;

	spin_lock_init(&dwc->lock);

	pm_runtime_set_active(dev);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_set_autosuspend_delay(dev, DWC3_DEFAULT_AUTOSUSPEND_DELAY);
	pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);
	if (ret < 0)
		goto err1;

	pm_runtime_forbid(dev);

	ret = dwc3_alloc_event_buffers(dwc, DWC3_EVENT_BUFFERS_SIZE);
	if (ret) {
		dev_err(dwc->dev, "failed to allocate event buffers\n");
		ret = -ENOMEM;
		goto err2;
	}

	ret = dwc3_get_dr_mode(dwc);
	if (ret)
		goto err3;

	ret = dwc3_alloc_scratch_buffers(dwc);
	if (ret)
		goto err3;

	ret = dwc3_core_init(dwc);
	if (ret) {
		dev_err(dev, "failed to initialize core\n");
		goto err4;
	}

	/* Check the maximum_speed parameter */
	switch (dwc->maximum_speed) {
	case USB_SPEED_LOW:
	case USB_SPEED_FULL:
	case USB_SPEED_HIGH:
	case USB_SPEED_SUPER:
	case USB_SPEED_SUPER_PLUS:
		break;
	default:
		dev_err(dev, "invalid maximum_speed parameter %d\n",
			dwc->maximum_speed);
		/* fall through */
	case USB_SPEED_UNKNOWN:
		/* default to superspeed */
		dwc->maximum_speed = USB_SPEED_SUPER;

		/*
		 * default to superspeed plus if we are capable.
		 */
		if (dwc3_is_usb31(dwc) &&
		    (DWC3_GHWPARAMS3_SSPHY_IFC(dwc->hwparams.hwparams3) ==
		     DWC3_GHWPARAMS3_SSPHY_IFC_GEN2))
			dwc->maximum_speed = USB_SPEED_SUPER_PLUS;

		break;
	}

	ret = dwc3_core_init_mode(dwc);
	if (ret)
		goto err5;

	dwc3_debugfs_init(dwc);
	pm_runtime_put(dev);

	return 0;

err5:
	dwc3_event_buffers_cleanup(dwc);

err4:
	dwc3_free_scratch_buffers(dwc);

err3:
	dwc3_free_event_buffers(dwc);
	dwc3_ulpi_exit(dwc);

err2:
	pm_runtime_allow(&pdev->dev);

err1:
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

err0:
	/*
	 * restore res->start back to its original value so that, in case the
	 * probe is deferred, we don't end up getting error in request the
	 * memory region the next time probe is called.
	 */
	res->start -= DWC3_GLOBALS_REGS_START;

	return ret;
}

static int dwc3_remove(struct platform_device *pdev)
{
	struct dwc3	*dwc = platform_get_drvdata(pdev);
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	pm_runtime_get_sync(&pdev->dev);
	/*
	 * restore res->start back to its original value so that, in case the
	 * probe is deferred, we don't end up getting error in request the
	 * memory region the next time probe is called.
	 */
	res->start -= DWC3_GLOBALS_REGS_START;

	dwc3_debugfs_exit(dwc);
	dwc3_core_exit_mode(dwc);

	dwc3_core_exit(dwc);
	dwc3_ulpi_exit(dwc);

	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_allow(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	dwc3_free_event_buffers(dwc);
	dwc3_free_scratch_buffers(dwc);

	return 0;
}

#ifdef CONFIG_PM
static int dwc3_suspend_common(struct dwc3 *dwc)
{
	unsigned long	flags;

	spin_lock_irqsave(&dwc->lock, flags);

	switch (dwc->dr_mode) {
	case USB_DR_MODE_PERIPHERAL:
		dwc3_gadget_suspend(dwc);
		break;
	case USB_DR_MODE_OTG:
		switch (dwc->otg_fsm.protocol) {
		case PROTO_GADGET:
			dwc3_gadget_suspend(dwc);
			break;
		case PROTO_HOST:
		case PROTO_UNDEF:
		default:
			/* nothing */
			break;
		}
		break;
	case USB_DR_MODE_HOST:
	case USB_DR_MODE_UNKNOWN:
	default:
		/* do nothing */
		break;
	}

	if (dwc->dr_mode == USB_DR_MODE_OTG)
		dwc3_otg_core_exit(dwc);

	spin_unlock_irqrestore(&dwc->lock, flags);
	dwc3_core_exit(dwc);

	return 0;
}

static int dwc3_resume_common(struct dwc3 *dwc)
{
	unsigned long	flags;
	int		ret;

	ret = dwc3_core_init(dwc);
	if (ret)
		return ret;

	if (dwc->dr_mode == USB_DR_MODE_OTG &&
	    dwc->edev) {
		if (dwc->otg_fsm.protocol == PROTO_HOST)
			dwc3_set_mode(dwc, DWC3_GCTL_PRTCAP_HOST);
		else if (dwc->otg_fsm.protocol == PROTO_GADGET)
			dwc3_set_mode(dwc, DWC3_GCTL_PRTCAP_DEVICE);
	}
	spin_lock_irqsave(&dwc->lock, flags);

	switch (dwc->dr_mode) {
	case USB_DR_MODE_PERIPHERAL:
		dwc3_gadget_resume(dwc);
		break;
	case USB_DR_MODE_OTG:
		switch (dwc->otg_fsm.protocol) {
		case PROTO_GADGET:
			dwc3_gadget_resume(dwc);
			break;
		case PROTO_HOST:
			break;
		case PROTO_UNDEF:
		default:
			/* nothing */
			break;
		}
		break;
	case USB_DR_MODE_HOST:
	case USB_DR_MODE_UNKNOWN:
	default:
		/* do nothing */
		break;
	}

	if (dwc->dr_mode == USB_DR_MODE_OTG) {
		dwc3_otg_core_init(dwc);
		if ((dwc->otg_fsm.protocol == PROTO_HOST) &&
		    !dwc->edev) {
			dwc3_drd_start_host(dwc, true, 1);
		}
	}

	spin_unlock_irqrestore(&dwc->lock, flags);

	return 0;
}

static int dwc3_runtime_checks(struct dwc3 *dwc)
{
	switch (dwc->dr_mode) {
	case USB_DR_MODE_PERIPHERAL:
	case USB_DR_MODE_OTG:
		if (dwc->connected)
			return -EBUSY;
		break;
	case USB_DR_MODE_HOST:
	default:
		/* do nothing */
		break;
	}

	return 0;
}

static int dwc3_runtime_suspend(struct device *dev)
{
	struct dwc3     *dwc = dev_get_drvdata(dev);
	int		ret;

	if (dwc3_runtime_checks(dwc))
		return -EBUSY;

	ret = dwc3_suspend_common(dwc);
	if (ret)
		return ret;

	device_init_wakeup(dev, true);

	return 0;
}

static int dwc3_runtime_resume(struct device *dev)
{
	struct dwc3     *dwc = dev_get_drvdata(dev);
	int		ret;

	device_init_wakeup(dev, false);

	ret = dwc3_resume_common(dwc);
	if (ret)
		return ret;

	switch (dwc->dr_mode) {
	case USB_DR_MODE_PERIPHERAL:
	case USB_DR_MODE_OTG:
		dwc3_gadget_process_pending_events(dwc);
		break;
	case USB_DR_MODE_HOST:
	case USB_DR_MODE_UNKNOWN:
	default:
		/* do nothing */
		break;
	}

	pm_runtime_mark_last_busy(dev);
	pm_runtime_put(dev);

	return 0;
}

static int dwc3_runtime_idle(struct device *dev)
{
	struct dwc3     *dwc = dev_get_drvdata(dev);

	switch (dwc->dr_mode) {
	case USB_DR_MODE_PERIPHERAL:
	case USB_DR_MODE_OTG:
		if (dwc3_runtime_checks(dwc))
			return -EBUSY;
		break;
	case USB_DR_MODE_HOST:
	default:
		/* do nothing */
		break;
	}

	pm_runtime_mark_last_busy(dev);
	pm_runtime_autosuspend(dev);

	return 0;
}
#endif /* CONFIG_PM */

#ifdef CONFIG_PM_SLEEP
static int dwc3_prepare(struct device *dev)
{
	struct dwc3	*dwc = dev_get_drvdata(dev);
	unsigned long	flags;

	spin_lock_irqsave(&dwc->lock, flags);
	dwc->otg_prevent_sync = true;
	spin_unlock_irqrestore(&dwc->lock, flags);

	return 0;
}

static void dwc3_complete(struct device *dev)
{
	struct dwc3	*dwc = dev_get_drvdata(dev);
	unsigned long	flags;

	spin_lock_irqsave(&dwc->lock, flags);
	dwc->otg_prevent_sync = false;
	spin_unlock_irqrestore(&dwc->lock, flags);
	if (dwc->dr_mode == USB_DR_MODE_OTG)
		queue_work(system_power_efficient_wq, &dwc->otg_work);
}

static int dwc3_suspend(struct device *dev)
{
	struct dwc3	*dwc = dev_get_drvdata(dev);
	int		ret;

	ret = dwc3_suspend_common(dwc);
	if (ret)
		return ret;

	pinctrl_pm_select_sleep_state(dev);

	return 0;
}

static int dwc3_resume(struct device *dev)
{
	struct dwc3	*dwc = dev_get_drvdata(dev);
	int		ret;

	pinctrl_pm_select_default_state(dev);

	ret = dwc3_resume_common(dwc);
	if (ret)
		return ret;

	pm_runtime_disable(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops dwc3_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(dwc3_suspend, dwc3_resume)
	SET_RUNTIME_PM_OPS(dwc3_runtime_suspend, dwc3_runtime_resume,
			dwc3_runtime_idle)
#ifdef CONFIG_PM_SLEEP
	.prepare = dwc3_prepare,
	.complete = dwc3_complete,
#endif
};

#ifdef CONFIG_OF
static const struct of_device_id of_dwc3_match[] = {
	{
		.compatible = "snps,dwc3"
	},
	{
		.compatible = "synopsys,dwc3"
	},
	{ },
};
MODULE_DEVICE_TABLE(of, of_dwc3_match);
#endif

#ifdef CONFIG_ACPI

#define ACPI_ID_INTEL_BSW	"808622B7"

static const struct acpi_device_id dwc3_acpi_match[] = {
	{ ACPI_ID_INTEL_BSW, 0 },
	{ },
};
MODULE_DEVICE_TABLE(acpi, dwc3_acpi_match);
#endif

static struct platform_driver dwc3_driver = {
	.probe		= dwc3_probe,
	.remove		= dwc3_remove,
	.driver		= {
		.name	= "dwc3",
		.of_match_table	= of_match_ptr(of_dwc3_match),
		.acpi_match_table = ACPI_PTR(dwc3_acpi_match),
		.pm	= &dwc3_dev_pm_ops,
	},
};

module_platform_driver(dwc3_driver);

MODULE_ALIAS("platform:dwc3");
MODULE_AUTHOR("Felipe Balbi <balbi@ti.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("DesignWare USB3 DRD Controller Driver");
