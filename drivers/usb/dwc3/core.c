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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/list.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/of.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/of.h>
#include <linux/usb/otg.h>

#include "platform_data.h"
#include "core.h"
#include "gadget.h"
#include "io.h"

#include "debug.h"

/* -------------------------------------------------------------------------- */

void dwc3_set_mode(struct dwc3 *dwc, u32 mode)
{
	u32 reg;

	reg = dwc3_readl(dwc->regs, DWC3_GCTL);
	reg &= ~(DWC3_GCTL_PRTCAPDIR(DWC3_GCTL_PRTCAP_OTG));
	reg |= DWC3_GCTL_PRTCAPDIR(mode);
	dwc->current_mode = mode;
	dwc3_writel(dwc->regs, DWC3_GCTL, reg);
}

/**
 * dwc3_core_soft_reset - Issues core soft reset and PHY reset
 * @dwc: pointer to our context structure
 */
static int dwc3_core_soft_reset(struct dwc3 *dwc)
{
	u32		reg;
	int		ret;

	/* Before Resetting PHY, put Core in Reset */
	reg = dwc3_readl(dwc->regs, DWC3_GCTL);
	reg |= DWC3_GCTL_CORESOFTRESET;
	dwc3_writel(dwc->regs, DWC3_GCTL, reg);

	/* Assert USB3 PHY reset */
	reg = dwc3_readl(dwc->regs, DWC3_GUSB3PIPECTL(0));
	reg |= DWC3_GUSB3PIPECTL_PHYSOFTRST;
	dwc3_writel(dwc->regs, DWC3_GUSB3PIPECTL(0), reg);

	/* Assert USB2 PHY reset */
	reg = dwc3_readl(dwc->regs, DWC3_GUSB2PHYCFG(0));
	reg |= DWC3_GUSB2PHYCFG_PHYSOFTRST;
	dwc3_writel(dwc->regs, DWC3_GUSB2PHYCFG(0), reg);

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
	mdelay(100);

	/* Clear USB3 PHY reset */
	reg = dwc3_readl(dwc->regs, DWC3_GUSB3PIPECTL(0));
	reg &= ~DWC3_GUSB3PIPECTL_PHYSOFTRST;
	dwc3_writel(dwc->regs, DWC3_GUSB3PIPECTL(0), reg);

	/* Clear USB2 PHY reset */
	reg = dwc3_readl(dwc->regs, DWC3_GUSB2PHYCFG(0));
	reg &= ~DWC3_GUSB2PHYCFG_PHYSOFTRST;
	dwc3_writel(dwc->regs, DWC3_GUSB2PHYCFG(0), reg);

	mdelay(100);

	/* After PHYs are stable we can take Core out of reset state */
	reg = dwc3_readl(dwc->regs, DWC3_GCTL);
	reg &= ~DWC3_GCTL_CORESOFTRESET;
	dwc3_writel(dwc->regs, DWC3_GCTL, reg);

	return 0;
}

/**
 * dwc3_free_one_event_buffer - Frees one event buffer
 * @dwc: Pointer to our controller context structure
 * @evt: Pointer to event buffer to be freed
 */
static void dwc3_free_one_event_buffer(struct dwc3 *dwc,
		struct dwc3_event_buffer *evt)
{
	dma_free_coherent(dwc->dev, evt->length, evt->buf, evt->dma);
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
	evt->buf	= dma_alloc_coherent(dwc->dev, length,
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
	int i;

	for (i = 0; i < dwc->num_event_buffers; i++) {
		evt = dwc->ev_buffs[i];
		if (evt)
			dwc3_free_one_event_buffer(dwc, evt);
	}
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
	int			num;
	int			i;

	num = DWC3_NUM_INT(dwc->hwparams.hwparams1);
	dwc->num_event_buffers = num;

	dwc->ev_buffs = devm_kzalloc(dwc->dev, sizeof(*dwc->ev_buffs) * num,
			GFP_KERNEL);
	if (!dwc->ev_buffs) {
		dev_err(dwc->dev, "can't allocate event buffers array\n");
		return -ENOMEM;
	}

	for (i = 0; i < num; i++) {
		struct dwc3_event_buffer	*evt;

		evt = dwc3_alloc_one_event_buffer(dwc, length);
		if (IS_ERR(evt)) {
			dev_err(dwc->dev, "can't allocate event buffer\n");
			return PTR_ERR(evt);
		}
		dwc->ev_buffs[i] = evt;
	}

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
	int				n;

	for (n = 0; n < dwc->num_event_buffers; n++) {
		evt = dwc->ev_buffs[n];
		dev_dbg(dwc->dev, "Event buf %p dma %08llx length %d\n",
				evt->buf, (unsigned long long) evt->dma,
				evt->length);

		evt->lpos = 0;

		dwc3_writel(dwc->regs, DWC3_GEVNTADRLO(n),
				lower_32_bits(evt->dma));
		dwc3_writel(dwc->regs, DWC3_GEVNTADRHI(n),
				upper_32_bits(evt->dma));
		dwc3_writel(dwc->regs, DWC3_GEVNTSIZ(n),
				DWC3_GEVNTSIZ_SIZE(evt->length));
		dwc3_writel(dwc->regs, DWC3_GEVNTCOUNT(n), 0);
	}

	return 0;
}

static void dwc3_event_buffers_cleanup(struct dwc3 *dwc)
{
	struct dwc3_event_buffer	*evt;
	int				n;

	for (n = 0; n < dwc->num_event_buffers; n++) {
		evt = dwc->ev_buffs[n];

		evt->lpos = 0;

		dwc3_writel(dwc->regs, DWC3_GEVNTADRLO(n), 0);
		dwc3_writel(dwc->regs, DWC3_GEVNTADRHI(n), 0);
		dwc3_writel(dwc->regs, DWC3_GEVNTSIZ(n), DWC3_GEVNTSIZ_INTMASK
				| DWC3_GEVNTSIZ_SIZE(0));
		dwc3_writel(dwc->regs, DWC3_GEVNTCOUNT(n), 0);
	}
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

	scratch_addr = dma_map_single(dwc->dev, dwc->scratchbuf,
			dwc->nr_scratch * DWC3_SCRATCHBUF_SIZE,
			DMA_BIDIRECTIONAL);
	if (dma_mapping_error(dwc->dev, scratch_addr)) {
		dev_err(dwc->dev, "failed to map scratch buffer\n");
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
	dma_unmap_single(dwc->dev, dwc->scratch_addr, dwc->nr_scratch *
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

	dma_unmap_single(dwc->dev, dwc->scratch_addr, dwc->nr_scratch *
			DWC3_SCRATCHBUF_SIZE, DMA_BIDIRECTIONAL);
	kfree(dwc->scratchbuf);
}

static void dwc3_core_num_eps(struct dwc3 *dwc)
{
	struct dwc3_hwparams	*parms = &dwc->hwparams;

	dwc->num_in_eps = DWC3_NUM_IN_EPS(parms);
	dwc->num_out_eps = DWC3_NUM_EPS(parms) - dwc->num_in_eps;

	dev_vdbg(dwc->dev, "found %d IN and %d OUT endpoints\n",
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
 */
static void dwc3_phy_setup(struct dwc3 *dwc)
{
	u32 reg;

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

	dwc3_writel(dwc->regs, DWC3_GUSB3PIPECTL(0), reg);

	mdelay(100);

	reg = dwc3_readl(dwc->regs, DWC3_GUSB2PHYCFG(0));

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

	dwc3_writel(dwc->regs, DWC3_GUSB2PHYCFG(0), reg);

	mdelay(100);
}

/**
 * dwc3_core_init - Low-level initialization of DWC3 Core
 * @dwc: Pointer to our controller context structure
 *
 * Returns 0 on success otherwise negative errno.
 */
static int dwc3_core_init(struct dwc3 *dwc)
{
	unsigned long		timeout;
	u32			hwparams4 = dwc->hwparams.hwparams4;
	u32			reg;
	int			ret;

	reg = dwc3_readl(dwc->regs, DWC3_GSNPSID);
	/* This should read as U3 followed by revision number */
	if ((reg & DWC3_GSNPSID_MASK) != 0x55330000) {
		dev_err(dwc->dev, "this is not a DesignWare USB3 DRD Core\n");
		ret = -ENODEV;
		goto err0;
	}
	dwc->revision = reg;

	/* issue device SoftReset too */
	timeout = jiffies + msecs_to_jiffies(500);
	dwc3_writel(dwc->regs, DWC3_DCTL, DWC3_DCTL_CSFTRST);
	do {
		reg = dwc3_readl(dwc->regs, DWC3_DCTL);
		if (!(reg & DWC3_DCTL_CSFTRST))
			break;

		if (time_after(jiffies, timeout)) {
			dev_err(dwc->dev, "Reset Timed Out\n");
			ret = -ETIMEDOUT;
			goto err0;
		}

		cpu_relax();
	} while (true);

	ret = dwc3_core_soft_reset(dwc);
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
		break;
	default:
		dev_dbg(dwc->dev, "No power optimization available\n");
	}

	/* check if current dwc3 is on simulation board */
	if (dwc->hwparams.hwparams6 & DWC3_GHWPARAMS6_EN_FPGA) {
		dev_dbg(dwc->dev, "it is on FPGA board\n");
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

	dwc3_core_num_eps(dwc);

	dwc3_writel(dwc->regs, DWC3_GCTL, reg);

	dwc3_phy_setup(dwc);

	ret = dwc3_alloc_scratch_buffers(dwc);
	if (ret)
		goto err1;

	ret = dwc3_setup_scratch_buffers(dwc);
	if (ret)
		goto err2;

	return 0;

err2:
	dwc3_free_scratch_buffers(dwc);

err1:
	usb_phy_shutdown(dwc->usb2_phy);
	usb_phy_shutdown(dwc->usb3_phy);
	phy_exit(dwc->usb2_generic_phy);
	phy_exit(dwc->usb3_generic_phy);

err0:
	return ret;
}

static void dwc3_core_exit(struct dwc3 *dwc)
{
	dwc3_free_scratch_buffers(dwc);
	usb_phy_shutdown(dwc->usb2_phy);
	usb_phy_shutdown(dwc->usb3_phy);
	phy_exit(dwc->usb2_generic_phy);
	phy_exit(dwc->usb3_generic_phy);
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

/* Get OTG events and sync it to OTG fsm */
static void dwc3_otg_fsm_sync(struct dwc3 *dwc)
{
	u32 reg;
	int id, vbus;

	reg = dwc3_readl(dwc->regs, DWC3_OSTS);
	dev_dbg(dwc->dev, "otgstatus 0x%x\n", reg);

	id = !!(reg & DWC3_OSTS_CONIDSTS);
	vbus = !!(reg & DWC3_OSTS_BSESVLD);

	if (id != dwc->fsm->id || vbus != dwc->fsm->vbus) {
		dev_dbg(dwc->dev, "id %d vbus %d\n", id, vbus);
		dwc->fsm->id = id;
		dwc->fsm->vbus = vbus;
		usb_otg_sync_inputs(dwc->fsm);
	}
}

static irqreturn_t dwc3_otg_thread_irq(int irq, void *_dwc)
{
	struct dwc3 *dwc = _dwc;
	unsigned long flags;
	irqreturn_t ret = IRQ_NONE;

	spin_lock_irqsave(&dwc->lock, flags);
	dwc3_otg_fsm_sync(dwc);
	/* unmask interrupts */
	dwc3_writel(dwc->regs, DWC3_OEVTEN, dwc->oevten);
	spin_unlock_irqrestore(&dwc->lock, flags);

	return ret;
}

static irqreturn_t dwc3_otg_irq(int irq, void *_dwc)
{
	struct dwc3 *dwc = _dwc;
	irqreturn_t ret = IRQ_NONE;
	u32 reg;

	spin_lock(&dwc->lock);

	reg = dwc3_readl(dwc->regs, DWC3_OEVT);
	if (reg) {
		dwc3_writel(dwc->regs, DWC3_OEVT, reg);
		/* mask interrupts till processed */
		dwc->oevten = dwc3_readl(dwc->regs, DWC3_OEVTEN);
		dwc3_writel(dwc->regs, DWC3_OEVTEN, 0);
		ret = IRQ_WAKE_THREAD;
	}

	spin_unlock(&dwc->lock);

	return ret;
}

/* --------------------- Dual-Role management ------------------------------- */

static void dwc3_drd_fsm_sync(struct dwc3 *dwc)
{
	int id, vbus;

	/* get ID */
	id = extcon_get_cable_state(dwc->edev, "USB-HOST");
	/* Host means ID == 0 */
	id = !id;

	/* get VBUS */
	vbus = extcon_get_cable_state(dwc->edev, "USB");
	dev_dbg(dwc->dev, "id %d vbus %d\n", id, vbus);

	dwc->fsm->id = id;
	dwc->fsm->vbus = vbus;
	usb_otg_sync_inputs(dwc->fsm);
}

static int dwc3_drd_start_host(struct otg_fsm *fsm, int on)
{
	struct device *dev = usb_otg_fsm_to_dev(fsm);
	struct dwc3 *dwc = dev_get_drvdata(dev);
	u32 reg;

	dev_dbg(dwc->dev, "%s: %d\n", __func__, on);
	if (dwc->edev) {
		if (on)
			dwc3_set_mode(dwc, DWC3_GCTL_PRTCAP_HOST);

		return 0;
	}

	/* switch OTG core */
	if (on) {
		/* OCTL.PeriMode = 0 */
		reg = dwc3_readl(dwc->regs, DWC3_OCTL);
		reg &= ~DWC3_OCTL_PERIMODE;
		dwc3_writel(dwc->regs, DWC3_OCTL, reg);
		/* unconditionally turn on VBUS */
		reg |= DWC3_OCTL_PRTPWRCTL;
		dwc3_writel(dwc->regs, DWC3_OCTL, reg);
	} else {
		/* turn off VBUS */
		reg = dwc3_readl(dwc->regs, DWC3_OCTL);
		reg &= ~DWC3_OCTL_PRTPWRCTL;
		dwc3_writel(dwc->regs, DWC3_OCTL, reg);
		/* OCTL.PeriMode = 1 */
		reg = dwc3_readl(dwc->regs, DWC3_OCTL);
		reg |= DWC3_OCTL_PERIMODE;
		dwc3_writel(dwc->regs, DWC3_OCTL, reg);
	}

	return 0;
}

static int dwc3_drd_start_gadget(struct otg_fsm *fsm, int on)
{
	struct device *dev = usb_otg_fsm_to_dev(fsm);
	struct dwc3 *dwc = dev_get_drvdata(dev);
	u32 reg;

	dev_dbg(dwc->dev, "%s: %d\n", __func__, on);
	if (on)
		dwc3_event_buffers_setup(dwc);

	if (dwc->edev) {
		if (on)
			dwc3_set_mode(dwc, DWC3_GCTL_PRTCAP_DEVICE);

		return 0;
	}

	/* switch OTG core */
	if (on) {
		/* OCTL.PeriMode = 1 */
		reg = dwc3_readl(dwc->regs, DWC3_OCTL);
		reg |= DWC3_OCTL_PERIMODE;
		dwc3_writel(dwc->regs, DWC3_OCTL, reg);
		/* GUSB2PHYCFG0.SusPHY = 1 */
		if (!dwc->dis_u2_susphy_quirk) {
			reg = dwc3_readl(dwc->regs, DWC3_GUSB2PHYCFG(0));
			reg |= DWC3_GUSB2PHYCFG_SUSPHY;
			dwc3_writel(dwc->regs, DWC3_GUSB2PHYCFG(0), reg);
		}
	} else {
		/* GUSB2PHYCFG0.SusPHY=0 */
		if (!dwc->dis_u2_susphy_quirk) {
			reg = dwc3_readl(dwc->regs, DWC3_GUSB2PHYCFG(0));
			reg &= ~DWC3_GUSB2PHYCFG_SUSPHY;
			dwc3_writel(dwc->regs, DWC3_GUSB2PHYCFG(0), reg);
		}
		/* OCTL.PeriMode = 1 */
		reg = dwc3_readl(dwc->regs, DWC3_OCTL);
		reg |= DWC3_OCTL_PERIMODE;
		dwc3_writel(dwc->regs, DWC3_OCTL, reg);
	}

	return 0;
}

static struct otg_fsm_ops dwc3_drd_ops = {
	.start_host = dwc3_drd_start_host,
	.start_gadget = dwc3_drd_start_gadget,
};

static int dwc3_drd_notifier(struct notifier_block *nb,
		unsigned long event, void *ptr)
{
	struct dwc3 *dwc = container_of(nb, struct dwc3, otg_nb);

	dwc3_drd_fsm_sync(dwc);

	return NOTIFY_DONE;
}

static int dwc3_drd_init(struct dwc3 *dwc)
{
	int ret, id, vbus;
	int irq;
	u32 reg;
	struct dwc3_hwparams *parms = &dwc->hwparams;
	unsigned long flags;

	/* If extcon device is not present we rely on OTG core for ID event */
	if (!dwc->edev) {
		dev_dbg(dwc->dev, "No extcon device found for OTG mode\n");
		goto try_otg_core;
	}

	dwc->otg_nb.notifier_call = dwc3_drd_notifier;
	ret = extcon_register_notifier(dwc->edev, &dwc->otg_nb);
	if (ret < 0) {
		dev_err(dwc->dev, "Couldn't register USB cable notifier\n");
		return -ENODEV;
	}

	/* sanity check id & vbus states */
	id = extcon_get_cable_state(dwc->edev, "USB-HOST");
	vbus = extcon_get_cable_state(dwc->edev, "USB");
	if (id < 0 || vbus < 0) {
		dev_err(dwc->dev, "Invalid USB cable state. id %d, vbus %d\n",
			id, vbus);
		ret = -ENODEV;
		goto fail;
	}

	/* register parent as OTG device with USB core */
	dwc->fsm = usb_otg_register(dwc->dev, &dwc3_drd_ops, true);
	if (IS_ERR(dwc->fsm)) {
		dev_err(dwc->dev, "Failed to register with OTG core\n");
		ret = -ENODEV;
		goto fail;
	}

	dwc3_drd_fsm_sync(dwc);

	return 0;

fail:
	extcon_unregister_notifier(dwc->edev, &dwc->otg_nb);

	return ret;

try_otg_core:
	/* get OTG capabilities */
	dwc->otg_has_hnp_rsp = !!(parms->hwparams6 & DWC3_GHWPARAMS6_HNPSUPPORT);
	dwc->otg_has_srp = !!(parms->hwparams6 & DWC3_GHWPARAMS6_SRPSUPPORT);
	dwc->otg_has_adp = !!(parms->hwparams6 & DWC3_GHWPARAMS6_ADPSUPPORT);
	dwc->otg_has_bc = !!(parms->hwparams6 & DWC3_GHWPARAMS6_BCSUPPORT);
	dwc->otg_has_otg3 = !!(parms->hwparams6 & DWC3_GHWPARAMS6_OTG3SUPPORT);

	dev_dbg(dwc->dev, "otg v%s - srp: %d, hnp/rsp:%d, adp:%d, bc:%d\n",
		 dwc->otg_has_otg3 ? "3.0":"2.0",
		 dwc->otg_has_srp, dwc->otg_has_hnp_rsp,
		 dwc->otg_has_adp, dwc->otg_has_bc);


	/* Some SoCs have OTG events in a separate IRQ, try that first */
	irq = dwc->otg_irq;
	if (!irq)
		irq = dwc->gadget_irq;
	if (!irq) {
		dev_err(dwc->dev, "failed to get irq for OTG\n");
		return -ENODEV;
	}

	/* register parent as OTG device with USB core */
	dwc->fsm = usb_otg_register(dwc->dev, &dwc3_drd_ops, true);
	if (IS_ERR(dwc->fsm)) {
		dev_err(dwc->dev, "Failed to register with OTG core\n");
		return -ENODEV;
	}

	/* disable all irqs */
	dwc3_writel(dwc->regs, DWC3_OEVTEN, 0);
	/* clear all events */
	dwc3_writel(dwc->regs, DWC3_OEVT, ~0);

	ret = request_threaded_irq(irq, dwc3_otg_irq, dwc3_otg_thread_irq,
				   IRQF_SHARED, "dwc3-otg", dwc);
	if (ret) {
		dev_err(dwc->dev, "failed to request irq #%d --> %d\n",
			irq, ret);
		ret = -ENODEV;
		goto error;
	}

	spin_lock_irqsave(&dwc->lock, flags);

	/* we need to set OTG to get events from OTG core */
	dwc3_set_mode(dwc, DWC3_GCTL_PRTCAP_OTG);
	/* GUSB2PHYCFG0.SusPHY=0 */
	if (!dwc->dis_u2_susphy_quirk) {
		reg = dwc3_readl(dwc->regs, DWC3_GUSB2PHYCFG(0));
		reg &= ~DWC3_GUSB2PHYCFG_SUSPHY;
		dwc3_writel(dwc->regs, DWC3_GUSB2PHYCFG(0), reg);
	}

	/* Initialize OTG registers */
	/*
	 * Prevent host/device reset from resetting OTG core.
	 * If we don't do this then xhci_reset (USBCMD.HCRST) will reset
	 * the signal outputs sent to the PHY, the OTG FSM logic of the
	 * core and also the resets to the VBUS filters inside the core.
	 */
	reg = DWC3_OCFG_SFTRSTMASK;
	dwc3_writel(dwc->regs, DWC3_OCFG, reg);
	/* Enable ID event interrupt */
	dwc3_writel(dwc->regs, DWC3_OEVTEN, DWC3_OEVTEN_CONIDSTSCHNGEN |
			DWC3_OEVTEN_BDEVVBUSCHNGE |
			DWC3_OEVTEN_BDEVSESSVLDDETEN);
	/* OCTL.PeriMode = 1 */
	dwc3_writel(dwc->regs, DWC3_OCTL, DWC3_OCTL_PERIMODE);

	spin_unlock_irqrestore(&dwc->lock, flags);

	dwc3_otg_fsm_sync(dwc);
	usb_otg_sync_inputs(dwc->fsm);

	return 0;

error:
	usb_otg_unregister(dwc->dev);

	return ret;
}

static void dwc3_drd_exit(struct dwc3 *dwc)
{
	usb_otg_unregister(dwc->dev);
	extcon_unregister_notifier(dwc->edev, &dwc->otg_nb);
}

/* -------------------------------------------------------------------------- */

static int dwc3_core_init_mode(struct dwc3 *dwc)
{
	struct device *dev = dwc->dev;
	int ret;

	switch (dwc->dr_mode) {
	case USB_DR_MODE_PERIPHERAL:
		dwc3_set_mode(dwc, DWC3_GCTL_PRTCAP_DEVICE);
		ret = dwc3_gadget_init(dwc);
		if (ret) {
			dev_err(dev, "failed to initialize gadget\n");
			return ret;
		}
		break;
	case USB_DR_MODE_HOST:
		dwc3_set_mode(dwc, DWC3_GCTL_PRTCAP_HOST);
		ret = dwc3_host_init(dwc);
		if (ret) {
			dev_err(dev, "failed to initialize host\n");
			return ret;
		}
		break;
	case USB_DR_MODE_OTG:
		ret = dwc3_drd_init(dwc);
		if (ret) {
			dev_err(dev, "failed to initialize drd\n");
			return ret;
		}

		ret = dwc3_host_init(dwc);
		if (ret) {
			dev_err(dev, "failed to initialize host\n");
			return ret;
		}

		ret = dwc3_gadget_init(dwc);
		if (ret) {
			dev_err(dev, "failed to initialize gadget\n");
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
		dwc3_host_exit(dwc);
		dwc3_gadget_exit(dwc);
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
	struct dwc3_platform_data *pdata = dev_get_platdata(dev);
	struct device_node	*node = dev->of_node;
	struct resource		*res;
	struct dwc3		*dwc;
	u8			lpm_nyet_threshold;
	u8			tx_de_emphasis;
	u8			hird_threshold;

	int			ret;

	void __iomem		*regs;
	void			*mem;

	mem = devm_kzalloc(dev, sizeof(*dwc) + DWC3_ALIGN_MASK, GFP_KERNEL);
	if (!mem) {
		dev_err(dev, "not enough memory\n");
		return -ENOMEM;
	}
	dwc = PTR_ALIGN(mem, DWC3_ALIGN_MASK + 1);
	dwc->mem = mem;
	dwc->dev = dev;

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(dev, "missing IRQ\n");
		return -ENODEV;
	}
	dwc->xhci_resources[1].start = res->start;
	dwc->xhci_resources[1].end = res->end;
	dwc->xhci_resources[1].flags = res->flags;
	dwc->xhci_resources[1].name = res->name;

	dwc->otg_irq = platform_get_irq_byname(pdev, "otg");
	if (!dwc->otg_irq)
		dev_err(dev, "missing OTG IRQ\n");

	dwc->gadget_irq = platform_get_irq_byname(pdev, "peripheral");
	if (!dwc->gadget_irq)
		dev_err(dev, "missing peripheral IRQ\n");

	dwc->xhci_irq = platform_get_irq_byname(pdev, "host");
	if (!dwc->xhci_irq)
		dev_err(dev, "missing HOST IRQ\n");

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

	if (node) {
		if (of_property_read_bool(node, "extcon"))
			dwc->edev = extcon_get_edev_by_phandle(dev, 0);
		else if (of_property_read_bool(dev->parent->of_node, "extcon"))
			dwc->edev = extcon_get_edev_by_phandle(dev->parent, 0);

		if (IS_ERR(dwc->edev)) {
			dev_vdbg(dev, "couldn't get extcon device\n");
			return -EPROBE_DEFER;
		}

		dwc->maximum_speed = of_usb_get_maximum_speed(node);
		dwc->has_lpm_erratum = of_property_read_bool(node,
				"snps,has-lpm-erratum");
		of_property_read_u8(node, "snps,lpm-nyet-threshold",
				&lpm_nyet_threshold);
		dwc->is_utmi_l1_suspend = of_property_read_bool(node,
				"snps,is-utmi-l1-suspend");
		of_property_read_u8(node, "snps,hird-threshold",
				&hird_threshold);

		dwc->needs_fifo_resize = of_property_read_bool(node,
				"tx-fifo-resize");
		dwc->dr_mode = of_usb_get_dr_mode(node);

		dwc->disable_scramble_quirk = of_property_read_bool(node,
				"snps,disable_scramble_quirk");
		dwc->u2exit_lfps_quirk = of_property_read_bool(node,
				"snps,u2exit_lfps_quirk");
		dwc->u2ss_inp3_quirk = of_property_read_bool(node,
				"snps,u2ss_inp3_quirk");
		dwc->req_p1p2p3_quirk = of_property_read_bool(node,
				"snps,req_p1p2p3_quirk");
		dwc->del_p1p2p3_quirk = of_property_read_bool(node,
				"snps,del_p1p2p3_quirk");
		dwc->del_phy_power_chg_quirk = of_property_read_bool(node,
				"snps,del_phy_power_chg_quirk");
		dwc->lfps_filter_quirk = of_property_read_bool(node,
				"snps,lfps_filter_quirk");
		dwc->rx_detect_poll_quirk = of_property_read_bool(node,
				"snps,rx_detect_poll_quirk");
		dwc->dis_u3_susphy_quirk = of_property_read_bool(node,
				"snps,dis_u3_susphy_quirk");
		dwc->dis_u2_susphy_quirk = of_property_read_bool(node,
				"snps,dis_u2_susphy_quirk");

		dwc->tx_de_emphasis_quirk = of_property_read_bool(node,
				"snps,tx_de_emphasis_quirk");
		of_property_read_u8(node, "snps,tx_de_emphasis",
				&tx_de_emphasis);
	} else if (pdata) {
		if (pdata->extcon) {
			dwc->edev = extcon_get_extcon_dev(pdata->extcon);
			if (!dwc->edev) {
				dev_vdbg(dev, "couldn't get extcon device\n");
				return -EPROBE_DEFER;
			}
		}
		dwc->maximum_speed = pdata->maximum_speed;
		dwc->has_lpm_erratum = pdata->has_lpm_erratum;
		if (pdata->lpm_nyet_threshold)
			lpm_nyet_threshold = pdata->lpm_nyet_threshold;
		dwc->is_utmi_l1_suspend = pdata->is_utmi_l1_suspend;
		if (pdata->hird_threshold)
			hird_threshold = pdata->hird_threshold;

		dwc->needs_fifo_resize = pdata->tx_fifo_resize;
		dwc->dr_mode = pdata->dr_mode;

		dwc->disable_scramble_quirk = pdata->disable_scramble_quirk;
		dwc->u2exit_lfps_quirk = pdata->u2exit_lfps_quirk;
		dwc->u2ss_inp3_quirk = pdata->u2ss_inp3_quirk;
		dwc->req_p1p2p3_quirk = pdata->req_p1p2p3_quirk;
		dwc->del_p1p2p3_quirk = pdata->del_p1p2p3_quirk;
		dwc->del_phy_power_chg_quirk = pdata->del_phy_power_chg_quirk;
		dwc->lfps_filter_quirk = pdata->lfps_filter_quirk;
		dwc->rx_detect_poll_quirk = pdata->rx_detect_poll_quirk;
		dwc->dis_u3_susphy_quirk = pdata->dis_u3_susphy_quirk;
		dwc->dis_u2_susphy_quirk = pdata->dis_u2_susphy_quirk;

		dwc->tx_de_emphasis_quirk = pdata->tx_de_emphasis_quirk;
		if (pdata->tx_de_emphasis)
			tx_de_emphasis = pdata->tx_de_emphasis;
	}

	/* default to superspeed if no maximum_speed passed */
	if (dwc->maximum_speed == USB_SPEED_UNKNOWN)
		dwc->maximum_speed = USB_SPEED_SUPER;

	dwc->lpm_nyet_threshold = lpm_nyet_threshold;
	dwc->tx_de_emphasis = tx_de_emphasis;

	dwc->hird_threshold = hird_threshold
		| (dwc->is_utmi_l1_suspend << 4);

	ret = dwc3_core_get_phy(dwc);
	if (ret)
		goto err0;

	spin_lock_init(&dwc->lock);
	platform_set_drvdata(pdev, dwc);

	dwc->regs	= regs;
	dwc->regs_size	= resource_size(res);

	dev->dma_mask	= dev->parent->dma_mask;
	dev->dma_parms	= dev->parent->dma_parms;
	dma_set_coherent_mask(dev, dev->parent->coherent_dma_mask);

	pm_runtime_enable(dev);
	pm_runtime_get_sync(dev);
	pm_runtime_forbid(dev);

	dwc3_cache_hwparams(dwc);

	ret = dwc3_alloc_event_buffers(dwc, DWC3_EVENT_BUFFERS_SIZE);
	if (ret) {
		dev_err(dwc->dev, "failed to allocate event buffers\n");
		ret = -ENOMEM;
		goto err1;
	}

	if (IS_ENABLED(CONFIG_USB_DWC3_HOST))
		dwc->dr_mode = USB_DR_MODE_HOST;
	else if (IS_ENABLED(CONFIG_USB_DWC3_GADGET))
		dwc->dr_mode = USB_DR_MODE_PERIPHERAL;

	if (dwc->dr_mode == USB_DR_MODE_UNKNOWN)
		dwc->dr_mode = USB_DR_MODE_OTG;

	ret = dwc3_core_init(dwc);
	if (ret) {
		dev_err(dev, "failed to initialize core\n");
		goto err1;
	}

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

	ret = dwc3_core_init_mode(dwc);
	if (ret)
		goto err5;

	ret = dwc3_debugfs_init(dwc);
	if (ret) {
		dev_err(dev, "failed to initialize debugfs\n");
		goto err6;
	}

	pm_runtime_allow(dev);

	return 0;

err6:
	dwc3_core_exit_mode(dwc);

err5:
	dwc3_event_buffers_cleanup(dwc);

err4:
	phy_power_off(dwc->usb3_generic_phy);

err3:
	phy_power_off(dwc->usb2_generic_phy);

err2:
	usb_phy_set_suspend(dwc->usb2_phy, 1);
	usb_phy_set_suspend(dwc->usb3_phy, 1);
	dwc3_core_exit(dwc);

err1:
	dwc3_free_event_buffers(dwc);

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

	/*
	 * restore res->start back to its original value so that, in case the
	 * probe is deferred, we don't end up getting error in request the
	 * memory region the next time probe is called.
	 */
	res->start -= DWC3_GLOBALS_REGS_START;

	dwc3_debugfs_exit(dwc);
	dwc3_core_exit_mode(dwc);
	dwc3_event_buffers_cleanup(dwc);
	dwc3_free_event_buffers(dwc);

	usb_phy_set_suspend(dwc->usb2_phy, 1);
	usb_phy_set_suspend(dwc->usb3_phy, 1);
	phy_power_off(dwc->usb2_generic_phy);
	phy_power_off(dwc->usb3_generic_phy);

	dwc3_core_exit(dwc);

	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int dwc3_suspend(struct device *dev)
{
	struct dwc3	*dwc = dev_get_drvdata(dev);
	unsigned long	flags;

	spin_lock_irqsave(&dwc->lock, flags);

	/* Save OTG state only if we're really using it */
	if (dwc->current_mode == DWC3_GCTL_PRTCAP_OTG) {
		dwc->ocfg = dwc3_readl(dwc->regs, DWC3_OCFG);
		dwc->octl = dwc3_readl(dwc->regs, DWC3_OCTL);
		dwc->oevt = dwc3_readl(dwc->regs, DWC3_OEVT);
		dwc->oevten = dwc3_readl(dwc->regs, DWC3_OEVTEN);
		dwc3_writel(dwc->regs, DWC3_OEVTEN, 0);
		disable_irq(dwc->otg_irq);
	}

	if (dwc->dr_mode == USB_DR_MODE_PERIPHERAL ||
	    ((dwc->dr_mode == USB_DR_MODE_OTG) && dwc->fsm->protocol == PROTO_GADGET))
		dwc3_gadget_suspend(dwc);

	dwc3_event_buffers_cleanup(dwc);

	dwc->gctl = dwc3_readl(dwc->regs, DWC3_GCTL);
	spin_unlock_irqrestore(&dwc->lock, flags);

	usb_phy_shutdown(dwc->usb3_phy);
	usb_phy_shutdown(dwc->usb2_phy);
	phy_exit(dwc->usb2_generic_phy);
	phy_exit(dwc->usb3_generic_phy);

	return 0;
}

static int dwc3_resume(struct device *dev)
{
	struct dwc3	*dwc = dev_get_drvdata(dev);
	unsigned long	flags;
	int		ret;

	usb_phy_init(dwc->usb3_phy);
	usb_phy_init(dwc->usb2_phy);
	ret = phy_init(dwc->usb2_generic_phy);
	if (ret < 0)
		return ret;

	ret = phy_init(dwc->usb3_generic_phy);
	if (ret < 0)
		goto err_usb2phy_init;

	spin_lock_irqsave(&dwc->lock, flags);

	dwc3_writel(dwc->regs, DWC3_GCTL, dwc->gctl);
	dwc3_event_buffers_setup(dwc);

	/* Restore OTG state only if we're really using it */
	if (dwc->current_mode == DWC3_GCTL_PRTCAP_OTG) {
		dwc3_writel(dwc->regs, DWC3_OCFG, dwc->ocfg);
		dwc3_writel(dwc->regs, DWC3_OCTL, dwc->octl);
		dwc3_writel(dwc->regs, DWC3_OEVT, dwc->oevt);
		dwc3_writel(dwc->regs, DWC3_OEVTEN, dwc->oevten);
		enable_irq(dwc->otg_irq);
	}

	if (dwc->dr_mode == USB_DR_MODE_PERIPHERAL ||
	    ((dwc->dr_mode == USB_DR_MODE_OTG) && dwc->fsm->protocol == PROTO_GADGET))
		dwc3_gadget_resume(dwc);

	spin_unlock_irqrestore(&dwc->lock, flags);

	pm_runtime_disable(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	return 0;

err_usb2phy_init:
	phy_exit(dwc->usb2_generic_phy);

	return ret;
}

static const struct dev_pm_ops dwc3_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(dwc3_suspend, dwc3_resume)
};

#define DWC3_PM_OPS	&(dwc3_dev_pm_ops)
#else
#define DWC3_PM_OPS	NULL
#endif

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

static struct platform_driver dwc3_driver = {
	.probe		= dwc3_probe,
	.remove		= dwc3_remove,
	.driver		= {
		.name	= "dwc3",
		.of_match_table	= of_match_ptr(of_dwc3_match),
		.pm	= DWC3_PM_OPS,
	},
};

module_platform_driver(dwc3_driver);

MODULE_ALIAS("platform:dwc3");
MODULE_AUTHOR("Felipe Balbi <balbi@ti.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("DesignWare USB3 DRD Controller Driver");
