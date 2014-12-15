/**
 * otg.c - DesignWare USB3 DRD Controller OTG
 *
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com
 *
 * Authors: George Cherian <george.cherian@ti.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2  of
 * the License as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>

#include <linux/usb/drd.h>
#include "core.h"
#include "io.h"

#define DWC3_GSTS_OTG_IP (1 << 10)

static irqreturn_t dwc3_otg_interrupt(int irq , void *_dwc)
{
	struct dwc3 *dwc = _dwc;
	u32 reg;

	spin_lock(&dwc->lock);
	reg = dwc3_readl(dwc->regs, DWC3_GSTS);
	if (reg & DWC3_GSTS_OTG_IP) {
		reg = dwc3_readl(dwc->regs, DWC3_OEVT);
		dev_vdbg(dwc->dev, "OTG Interrupt %x\n", reg);
		dwc3_writel(dwc->regs, DWC3_OEVT, reg);
		spin_unlock(&dwc->lock);
		return IRQ_WAKE_THREAD;
	}

	spin_unlock(&dwc->lock);
	return IRQ_NONE;
}

static irqreturn_t dwc3_otg_thread_interrupt(int irq, void *_dwc)
{
	struct dwc3 *dwc = _dwc;
	u32 reg = dwc3_readl(dwc->regs, DWC3_OSTS);

	dev_vdbg(dwc->dev, "OTG thread interrupt\n");
	if ((reg & DWC3_OSTS_CONIDSTS)) {
		usb_drd_stop_hcd(dwc->dev);
		dwc3_writel(dwc->regs, DWC3_OCFG, DWC3_OCFG_SFTRSTMASK);
		dwc3_writel(dwc->regs, DWC3_OCTL,
			    DWC3_OCTL_SESREQ | DWC3_OCTL_PERIMODE);
		if (usb_drd_get_state(dwc->dev) & DRD_DEVICE_REGISTERED) {
			usb_drd_start_udc(dwc->dev);
		} else {
			dwc3_core_gadget_helper(dwc);
			dwc3_gadget_init(dwc);
		}
		dwc3_writel(dwc->regs, DWC3_OEVTEN,
			    DWC3_OEVTEN_CONIDSTSCHNGEN);
	} else if (!(reg & DWC3_OSTS_CONIDSTS)) {
		usb_drd_stop_udc(dwc->dev);
		dwc3_writel(dwc->regs, DWC3_OCFG,
			    DWC3_OCFG_DISPWRCUTTOFF | DWC3_OCFG_SFTRSTMASK);
		dwc3_writel(dwc->regs, DWC3_OCTL, DWC3_OCTL_PRTPWRCTL);
		if (usb_drd_get_state(dwc->dev) & DRD_HOST_REGISTERED)
			usb_drd_start_hcd(dwc->dev);
		else
			dwc3_host_init(dwc);

		dwc3_writel(dwc->regs, DWC3_OEVTEN,
			    DWC3_OEVTEN_CONIDSTSCHNGEN);
	}

	return IRQ_HANDLED;
}

int dwc3_otg_init(struct dwc3 *dwc)
{
	u32 reg, ret;

	usb_drd_add(dwc->dev);
	dwc3_writel(dwc->regs, DWC3_OEVT, 0xFFFF);
	if (dwc->otg_irq > 0) {
		ret = devm_request_threaded_irq(dwc->dev, dwc->otg_irq,
						dwc3_otg_interrupt,
						dwc3_otg_thread_interrupt,
						IRQF_SHARED, "dwc3-otg", dwc);
	} else {
		WARN(1, "Trying to request invalid otg_irq");
		return -ENODEV;
	}

	dwc3_writel(dwc->regs, DWC3_OEVTEN, DWC3_OEVTEN_CONIDSTSCHNGEN);
	dwc3_writel(dwc->regs, DWC3_OCTL, DWC3_OCTL_PERIMODE);

	reg = dwc3_readl(dwc->regs, DWC3_OSTS);
	if ((reg & DWC3_OSTS_CONIDSTS)) {
		dev_vdbg(dwc->dev, "Gadget  init\n");
		dwc3_writel(dwc->regs, DWC3_OCFG, DWC3_OCFG_SFTRSTMASK);
		dwc3_writel(dwc->regs, DWC3_OCTL,
			    DWC3_OCTL_SESREQ | DWC3_OCTL_PERIMODE);
		dwc3_gadget_init(dwc);
		dwc3_writel(dwc->regs, DWC3_OEVTEN,
			    DWC3_OEVTEN_CONIDSTSCHNGEN);

	} else if (!(reg & DWC3_OSTS_CONIDSTS)) {
		dev_vdbg(dwc->dev, "Host  init\n");
		dwc3_writel(dwc->regs, DWC3_OCFG,
			    DWC3_OCFG_DISPWRCUTTOFF | DWC3_OCFG_SFTRSTMASK);
		dwc3_writel(dwc->regs, DWC3_OCTL, DWC3_OCTL_PRTPWRCTL);
		dwc3_host_init(dwc);
		dwc3_writel(dwc->regs, DWC3_OEVTEN,
			    DWC3_OEVTEN_CONIDSTSCHNGEN);
	}

	return 0;
}

void dwc3_otg_suspend(struct dwc3 *dwc)
{
	dwc->ocfg = dwc3_readl(dwc->regs, DWC3_OCFG);
	dwc->octl = dwc3_readl(dwc->regs, DWC3_OCTL);
	dwc->oevt = dwc3_readl(dwc->regs, DWC3_OEVT);
	dwc->oevten = dwc3_readl(dwc->regs, DWC3_OEVTEN);
	dwc->osts = dwc3_readl(dwc->regs, DWC3_OSTS);
}

void dwc3_otg_resume(struct dwc3 *dwc)
{
	dwc3_writel(dwc->regs, DWC3_OCFG, dwc->ocfg);
	dwc3_writel(dwc->regs, DWC3_OCTL, dwc->octl);
	dwc3_writel(dwc->regs, DWC3_OEVT, dwc->oevt);
	dwc3_writel(dwc->regs, DWC3_OEVTEN, dwc->oevten);
	dwc3_writel(dwc->regs, DWC3_OSTS, dwc->osts);
}
