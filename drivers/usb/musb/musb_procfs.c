/******************************************************************
 * Copyright 2005 Mentor Graphics Corporation
 * Copyright (C) 2005-2006 by Texas Instruments
 *
 * This file is part of the Inventra Controller Driver for Linux.
 *
 * The Inventra Controller Driver for Linux is free software; you
 * can redistribute it and/or modify it under the terms of the GNU
 * General Public License version 2 as published by the Free Software
 * Foundation.
 *
 * The Inventra Controller Driver for Linux is distributed in
 * the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with The Inventra Controller Driver for Linux ; if not,
 * write to the Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA  02111-1307  USA
 *
 * ANY DOWNLOAD, USE, REPRODUCTION, MODIFICATION OR DISTRIBUTION
 * OF THIS DRIVER INDICATES YOUR COMPLETE AND UNCONDITIONAL ACCEPTANCE
 * OF THOSE TERMS.THIS DRIVER IS PROVIDED "AS IS" AND MENTOR GRAPHICS
 * MAKES NO WARRANTIES, EXPRESS OR IMPLIED, RELATED TO THIS DRIVER.
 * MENTOR GRAPHICS SPECIFICALLY DISCLAIMS ALL IMPLIED WARRANTIES
 * OF MERCHANTABILITY; FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT.  MENTOR GRAPHICS DOES NOT PROVIDE SUPPORT
 * SERVICES OR UPDATES FOR THIS DRIVER, EVEN IF YOU ARE A MENTOR
 * GRAPHICS SUPPORT CUSTOMER.
 ******************************************************************/

/*
 * Inventra Controller Driver (ICD) for Linux.
 *
 * The code managing debug files (currently in procfs).
 */

#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <asm/uaccess.h>	/* FIXME remove procfs writes */
#include <asm/arch/hardware.h>

#include "musbdefs.h"

#include "davinci.h"


const char *otg_state_string(struct musb *musb)
{
	switch (musb->xceiv.state) {
	case OTG_STATE_A_IDLE:		return "a_idle";
	case OTG_STATE_A_WAIT_VRISE:	return "a_wait_vrise";
	case OTG_STATE_A_WAIT_BCON:	return "a_wait_bcon";
	case OTG_STATE_A_HOST:		return "a_host";
	case OTG_STATE_A_SUSPEND:	return "a_suspend";
	case OTG_STATE_A_PERIPHERAL:	return "a_peripheral";
	case OTG_STATE_A_WAIT_VFALL:	return "a_wait_vfall";
	case OTG_STATE_A_VBUS_ERR:	return "a_vbus_err";
	case OTG_STATE_B_IDLE:		return "b_idle";
	case OTG_STATE_B_SRP_INIT:	return "b_srp_init";
	case OTG_STATE_B_PERIPHERAL:	return "b_peripheral";
	case OTG_STATE_B_WAIT_ACON:	return "b_wait_acon";
	case OTG_STATE_B_HOST:		return "b_host";
	default:			return "UNDEFINED";
	}
}

#ifdef CONFIG_USB_MUSB_HDRC_HCD

static int dump_qh(struct musb_qh *qh, char *buf, unsigned max)
{
	int				count;
	int				tmp;
	struct usb_host_endpoint	*hep = qh->hep;
	struct urb			*urb;

	count = snprintf(buf, max, "    qh %p dev%d ep%d%s max%d\n",
			qh, qh->dev->devnum, qh->epnum,
			({ char *s; switch (qh->type) {
			case USB_ENDPOINT_XFER_BULK:
				s = "-bulk"; break;
			case USB_ENDPOINT_XFER_INT:
				s = "-int"; break;
			case USB_ENDPOINT_XFER_CONTROL:
				s = ""; break;
			default:
				s = "iso"; break;
			}; s; }),
			qh->maxpacket);
	if (count <= 0)
		return 0;
	buf += count;
	max -= count;

	list_for_each_entry(urb, &hep->urb_list, urb_list) {
		tmp = snprintf(buf, max, "\t%s urb %p %d/%d\n",
				usb_pipein(urb->pipe) ? "in" : "out",
				urb, urb->actual_length,
				urb->transfer_buffer_length);
		if (tmp <= 0)
			break;
		tmp = min(tmp, (int)max);
		count += tmp;
		buf += tmp;
		max -= tmp;
	}
	return count;
}

static int
dump_queue(struct list_head *q, char *buf, unsigned max)
{
	int		count = 0;
	struct musb_qh	*qh;

	list_for_each_entry(qh, q, ring) {
		int	tmp;

		tmp = dump_qh(qh, buf, max);
		if (tmp <= 0)
			break;
		tmp = min(tmp, (int)max);
		count += tmp;
		buf += tmp;
		max -= tmp;
	}
	return count;
}

#endif	/* HCD */

#ifdef CONFIG_USB_GADGET_MUSB_HDRC
static int dump_ep(struct musb_ep *ep, char *buffer, unsigned max)
{
	char		*buf = buffer;
	int		code = 0;
	void __iomem	*regs = ep->hw_ep->regs;
	char		*mode = "1buf";

	if (ep->is_in) {
		if (ep->hw_ep->tx_double_buffered)
			mode = "2buf";
	} else {
		if (ep->hw_ep->rx_double_buffered)
			mode = "2buf";
	}

	do {
		struct usb_request	*req;

		code = snprintf(buf, max,
				"\n%s (hw%d): %s%s, csr %04x maxp %04x\n",
				ep->name, ep->bEndNumber,
				mode, ep->dma ? " dma" : "",
				musb_readw(regs,
					(ep->is_in || !ep->bEndNumber)
						? MGC_O_HDRC_TXCSR
						: MGC_O_HDRC_RXCSR),
				musb_readw(regs, ep->is_in
						? MGC_O_HDRC_TXMAXP
						: MGC_O_HDRC_RXMAXP)
				);
		if (code <= 0)
			break;
		code = min(code, (int) max);
		buf += code;
		max -= code;

		if (is_cppi_enabled() && ep->bEndNumber) {
			unsigned	cppi = ep->bEndNumber - 1;
			void __iomem	*base = ep->pThis->ctrl_base;
			unsigned	off1 = cppi << 2;
			void __iomem	*ram = base;
			char		tmp[16];

			if (ep->is_in) {
				ram += DAVINCI_TXCPPI_STATERAM_OFFSET(cppi);
				tmp[0] = 0;
			} else {
				ram += DAVINCI_RXCPPI_STATERAM_OFFSET(cppi);
				snprintf(tmp, sizeof tmp, "%d left, ",
					musb_readl(base,
					DAVINCI_RXCPPI_BUFCNT0_REG + off1));
			}

			code = snprintf(buf, max, "%cX DMA%d: %s"
					"%08x %08x, %08x %08x; "
					"%08x %08x %08x .. %08x\n",
				ep->is_in ? 'T' : 'R',
				ep->bEndNumber - 1, tmp,
				musb_readl(ram, 0 * 4),
				musb_readl(ram, 1 * 4),
				musb_readl(ram, 2 * 4),
				musb_readl(ram, 3 * 4),
				musb_readl(ram, 4 * 4),
				musb_readl(ram, 5 * 4),
				musb_readl(ram, 6 * 4),
				musb_readl(ram, 7 * 4));
			if (code <= 0)
				break;
			code = min(code, (int) max);
			buf += code;
			max -= code;
		}

		if (list_empty(&ep->req_list)) {
			code = snprintf(buf, max, "\t(queue empty)\n");
			if (code <= 0)
				break;
			code = min(code, (int) max);
			buf += code;
			max -= code;
			break;
		}
		list_for_each_entry (req, &ep->req_list, list) {
			code = snprintf(buf, max, "\treq %p, %s%s%d/%d\n",
					req,
					req->zero ? "zero, " : "",
					req->short_not_ok ? "!short, " : "",
					req->actual, req->length);
			if (code <= 0)
				break;
			code = min(code, (int) max);
			buf += code;
			max -= code;
		}
	} while(0);
	return buf - buffer;
}
#endif

static int
dump_end_info(struct musb *pThis, u8 bEnd, char *aBuffer, unsigned max)
{
	int			code = 0;
	char			*buf = aBuffer;
	struct musb_hw_ep	*pEnd = &pThis->aLocalEnd[bEnd];

	do {
		MGC_SelectEnd(pThis->pRegs, bEnd);
#ifdef CONFIG_USB_MUSB_HDRC_HCD
		if (is_host_active(pThis)) {
			int		dump_rx, dump_tx;
			void __iomem	*regs = pEnd->regs;

			/* TEMPORARY (!) until we have a real periodic
			 * schedule tree ...
			 */
			if (!bEnd) {
				/* control is shared, uses RX queue
				 * but (mostly) shadowed tx registers
				 */
				dump_tx = !list_empty(&pThis->control);
				dump_rx = 0;
			} else if (pEnd == pThis->bulk_ep) {
				dump_tx = !list_empty(&pThis->out_bulk);
				dump_rx = !list_empty(&pThis->in_bulk);
			} else if (pThis->periodic[bEnd]) {
				struct usb_host_endpoint	*hep;

				hep = pThis->periodic[bEnd]->hep;
				dump_rx = hep->desc.bEndpointAddress
						& USB_ENDPOINT_DIR_MASK;
				dump_tx = !dump_rx;
			} else
				break;
			/* END TEMPORARY */


			if (dump_rx) {
				code = snprintf(buf, max,
					"\nRX%d: %s rxcsr %04x interval %02x "
					"max %04x type %02x; "
					"dev %d hub %d port %d"
					"\n",
					bEnd,
					pEnd->rx_double_buffered
						? "2buf" : "1buf",
					musb_readw(regs, MGC_O_HDRC_RXCSR),
					musb_readb(regs, MGC_O_HDRC_RXINTERVAL),
					musb_readw(regs, MGC_O_HDRC_RXMAXP),
					musb_readb(regs, MGC_O_HDRC_RXTYPE),
					/* FIXME:  assumes multipoint */
					musb_readb(pThis->pRegs,
						MGC_BUSCTL_OFFSET(bEnd,
						MGC_O_HDRC_RXFUNCADDR)),
					musb_readb(pThis->pRegs,
						MGC_BUSCTL_OFFSET(bEnd,
						MGC_O_HDRC_RXHUBADDR)),
					musb_readb(pThis->pRegs,
						MGC_BUSCTL_OFFSET(bEnd,
						MGC_O_HDRC_RXHUBPORT))
					);
				if (code <= 0)
					break;
				code = min(code, (int) max);
				buf += code;
				max -= code;

				if (is_cppi_enabled()
						&& bEnd
						&& pEnd->rx_channel) {
					unsigned	cppi = bEnd - 1;
					unsigned	off1 = cppi << 2;
					void __iomem	*base;
					void __iomem	*ram;
					char		tmp[16];

					base = pThis->ctrl_base;
					ram = DAVINCI_RXCPPI_STATERAM_OFFSET(
							cppi) + base;
					snprintf(tmp, sizeof tmp, "%d left, ",
						musb_readl(base,
						DAVINCI_RXCPPI_BUFCNT0_REG
								+ off1));

					code = snprintf(buf, max,
						"    rx dma%d: %s"
						"%08x %08x, %08x %08x; "
						"%08x %08x %08x .. %08x\n",
						cppi, tmp,
						musb_readl(ram, 0 * 4),
						musb_readl(ram, 1 * 4),
						musb_readl(ram, 2 * 4),
						musb_readl(ram, 3 * 4),
						musb_readl(ram, 4 * 4),
						musb_readl(ram, 5 * 4),
						musb_readl(ram, 6 * 4),
						musb_readl(ram, 7 * 4));
					if (code <= 0)
						break;
					code = min(code, (int) max);
					buf += code;
					max -= code;
				}

				if (pEnd == pThis->bulk_ep
						&& !list_empty(
							&pThis->in_bulk)) {
					code = dump_queue(&pThis->in_bulk,
							buf, max);
					if (code <= 0)
						break;
					code = min(code, (int) max);
					buf += code;
					max -= code;
				} else if (pThis->periodic[bEnd]) {
					code = dump_qh(pThis->periodic[bEnd],
							buf, max);
					if (code <= 0)
						break;
					code = min(code, (int) max);
					buf += code;
					max -= code;
				}
			}

			if (dump_tx) {
				code = snprintf(buf, max,
					"\nTX%d: %s txcsr %04x interval %02x "
					"max %04x type %02x; "
					"dev %d hub %d port %d"
					"\n",
					bEnd,
					pEnd->tx_double_buffered
						? "2buf" : "1buf",
					musb_readw(regs, MGC_O_HDRC_TXCSR),
					musb_readb(regs, MGC_O_HDRC_TXINTERVAL),
					musb_readw(regs, MGC_O_HDRC_TXMAXP),
					musb_readb(regs, MGC_O_HDRC_TXTYPE),
					/* FIXME:  assumes multipoint */
					musb_readb(pThis->pRegs,
						MGC_BUSCTL_OFFSET(bEnd,
						MGC_O_HDRC_TXFUNCADDR)),
					musb_readb(pThis->pRegs,
						MGC_BUSCTL_OFFSET(bEnd,
						MGC_O_HDRC_TXHUBADDR)),
					musb_readb(pThis->pRegs,
						MGC_BUSCTL_OFFSET(bEnd,
						MGC_O_HDRC_TXHUBPORT))
					);
				if (code <= 0)
					break;
				code = min(code, (int) max);
				buf += code;
				max -= code;

				if (is_cppi_enabled()
						&& bEnd
						&& pEnd->tx_channel) {
					unsigned	cppi = bEnd - 1;
					void __iomem	*base;
					void __iomem	*ram;

					base = pThis->ctrl_base;
					ram = DAVINCI_RXCPPI_STATERAM_OFFSET(
							cppi) + base;
					code = snprintf(buf, max,
						"    tx dma%d: "
						"%08x %08x, %08x %08x; "
						"%08x %08x %08x .. %08x\n",
						cppi,
						musb_readl(ram, 0 * 4),
						musb_readl(ram, 1 * 4),
						musb_readl(ram, 2 * 4),
						musb_readl(ram, 3 * 4),
						musb_readl(ram, 4 * 4),
						musb_readl(ram, 5 * 4),
						musb_readl(ram, 6 * 4),
						musb_readl(ram, 7 * 4));
					if (code <= 0)
						break;
					code = min(code, (int) max);
					buf += code;
					max -= code;
				}

				if (pEnd == pThis->control_ep
						&& !list_empty(
							&pThis->control)) {
					code = dump_queue(&pThis->control,
							buf, max);
					if (code <= 0)
						break;
					code = min(code, (int) max);
					buf += code;
					max -= code;
				} else if (pEnd == pThis->bulk_ep
						&& !list_empty(
							&pThis->out_bulk)) {
					code = dump_queue(&pThis->out_bulk,
							buf, max);
					if (code <= 0)
						break;
					code = min(code, (int) max);
					buf += code;
					max -= code;
				} else if (pThis->periodic[bEnd]) {
					code = dump_qh(pThis->periodic[bEnd],
							buf, max);
					if (code <= 0)
						break;
					code = min(code, (int) max);
					buf += code;
					max -= code;
				}
			}
		}
#endif
#ifdef CONFIG_USB_GADGET_MUSB_HDRC
		if (is_peripheral_active(pThis)) {
			code = 0;

			if (pEnd->ep_in.desc || !bEnd) {
				code = dump_ep(&pEnd->ep_in, buf, max);
				if (code <= 0)
					break;
				code = min(code, (int) max);
				buf += code;
				max -= code;
			}
			if (pEnd->ep_out.desc) {
				code = dump_ep(&pEnd->ep_out, buf, max);
				if (code <= 0)
					break;
				code = min(code, (int) max);
				buf += code;
				max -= code;
			}
		}
#endif
	} while (0);

	return buf - aBuffer;
}

/** Dump the current status and compile options.
 * @param pThis the device driver instance
 * @param buffer where to dump the status; it must be big enough hold the
 * result otherwise "BAD THINGS HAPPENS(TM)".
 */
static int dump_header_stats(struct musb *pThis, char *buffer)
{
	int code, count = 0;
	const void __iomem *pBase = pThis->pRegs;

	*buffer = 0;
	count = sprintf(buffer, "Status: %sHDRC, Mode=%s "
				"(Power=%02x, DevCtl=%02x)\n",
			(pThis->bIsMultipoint ? "M" : ""), MUSB_MODE(pThis),
			musb_readb(pBase, MGC_O_HDRC_POWER),
			musb_readb(pBase, MGC_O_HDRC_DEVCTL));
	if (count <= 0)
		return 0;
	buffer += count;

	code = sprintf(buffer, "OTG state: %s; %sactive\n",
			otg_state_string(pThis),
			pThis->is_active ? "" : "in");
	if (code <= 0)
		goto done;
	buffer += code;
	count += code;

	code = sprintf(buffer,
			"Options: "
#ifdef CONFIG_USB_INVENTRA_FIFO
			"pio"
#elif defined(CONFIG_USB_TI_CPPI_DMA)
			"cppi-dma"
#elif defined(CONFIG_USB_INVENTRA_DMA)
			"musb-dma"
#elif defined(CONFIG_USB_TUSB_OMAP_DMA)
			"tusb-omap-dma"
#else
			"?dma?"
#endif
			", "
#ifdef CONFIG_USB_MUSB_OTG
			"otg (peripheral+host)"
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
			"peripheral"
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
			"host"
#endif
			", debug=%d [eps=%d]\n",
		debug,
		pThis->bEndCount);
	if (code <= 0)
		goto done;
	count += code;
	buffer += code;

#ifdef	CONFIG_USB_GADGET_MUSB_HDRC
	code = sprintf(buffer, "Peripheral address: %02x\n",
			musb_readb(pThis, MGC_O_HDRC_FADDR));
	if (code <= 0)
		goto done;
	buffer += code;
	count += code;
#endif

#ifdef	CONFIG_USB_MUSB_HDRC_HCD
	code = sprintf(buffer, "Root port status: %08x\n",
			pThis->port1_status);
	if (code <= 0)
		goto done;
	buffer += code;
	count += code;
#endif

#ifdef	CONFIG_ARCH_DAVINCI
	code = sprintf(buffer,
			"DaVinci: ctrl=%02x stat=%1x phy=%03x\n"
			"\trndis=%05x auto=%04x intsrc=%08x intmsk=%08x"
			"\n",
			musb_readl(pThis->ctrl_base, DAVINCI_USB_CTRL_REG),
			musb_readl(pThis->ctrl_base, DAVINCI_USB_STAT_REG),
			__raw_readl(IO_ADDRESS(USBPHY_CTL_PADDR)),
			musb_readl(pThis->ctrl_base, DAVINCI_RNDIS_REG),
			musb_readl(pThis->ctrl_base, DAVINCI_AUTOREQ_REG),
			musb_readl(pThis->ctrl_base,
					DAVINCI_USB_INT_SOURCE_REG),
			musb_readl(pThis->ctrl_base,
					DAVINCI_USB_INT_MASK_REG));
	if (code <= 0)
		goto done;
	count += code;
	buffer += code;
#endif	/* DAVINCI */

#ifdef CONFIG_USB_TUSB6010
	code = sprintf(buffer,
			"TUSB6010: devconf %08x, phy enable %08x drive %08x"
			"\n\totg %03x timer %08x"
			"\n\tprcm conf %08x mgmt %08x; int src %08x mask %08x"
			"\n",
			musb_readl(pThis->ctrl_base, TUSB_DEV_CONF),
			musb_readl(pThis->ctrl_base, TUSB_PHY_OTG_CTRL_ENABLE),
			musb_readl(pThis->ctrl_base, TUSB_PHY_OTG_CTRL),
			musb_readl(pThis->ctrl_base, TUSB_DEV_OTG_STAT),
			musb_readl(pThis->ctrl_base, TUSB_DEV_OTG_TIMER),
			musb_readl(pThis->ctrl_base, TUSB_PRCM_CONF),
			musb_readl(pThis->ctrl_base, TUSB_PRCM_MNGMT),
			musb_readl(pThis->ctrl_base, TUSB_INT_SRC),
			musb_readl(pThis->ctrl_base, TUSB_INT_MASK));
	if (code <= 0)
		goto done;
	count += code;
	buffer += code;
#endif	/* DAVINCI */

	if (is_cppi_enabled() && pThis->pDmaController) {
		code = sprintf(buffer,
				"CPPI: txcr=%d txsrc=%01x txena=%01x; "
				"rxcr=%d rxsrc=%01x rxena=%01x "
				"\n",
				musb_readl(pThis->ctrl_base,
						DAVINCI_TXCPPI_CTRL_REG),
				musb_readl(pThis->ctrl_base,
						DAVINCI_TXCPPI_RAW_REG),
				musb_readl(pThis->ctrl_base,
						DAVINCI_TXCPPI_INTENAB_REG),
				musb_readl(pThis->ctrl_base,
						DAVINCI_RXCPPI_CTRL_REG),
				musb_readl(pThis->ctrl_base,
						DAVINCI_RXCPPI_RAW_REG),
				musb_readl(pThis->ctrl_base,
						DAVINCI_RXCPPI_INTENAB_REG));
		if (code <= 0)
			goto done;
		count += code;
		buffer += code;
	}

#ifdef CONFIG_USB_GADGET_MUSB_HDRC
	if (is_peripheral_enabled(pThis)) {
		code = sprintf(buffer, "Gadget driver: %s\n",
				pThis->pGadgetDriver
					? pThis->pGadgetDriver->driver.name
					: "(none)");
		if (code <= 0)
			goto done;
		count += code;
		buffer += code;
	}
#endif

done:
	return count;
}

/* Write to ProcFS
 *
 * C soft-connect
 * c soft-disconnect
 * I enable HS
 * i disable HS
 * s stop session
 * F force session (OTG-unfriendly)
 * E rElinquish bus (OTG)
 * H request host mode
 * h cancel host request
 * T start sending TEST_PACKET
 * D<num> set/query the debug level
 */
static int musb_proc_write(struct file *file, const char __user *buffer,
			unsigned long count, void *data)
{
	char cmd;
	u8 bReg;
	struct musb *musb = (struct musb *)data;
	void __iomem *pBase = musb->pRegs;

	/* MOD_INC_USE_COUNT; */

	if (unlikely(copy_from_user(&cmd, buffer, 1)))
		return -EFAULT;

	switch (cmd) {
	case 'C':
		if (pBase) {
			bReg = musb_readb(pBase, MGC_O_HDRC_POWER)
					| MGC_M_POWER_SOFTCONN;
			musb_writeb(pBase, MGC_O_HDRC_POWER, bReg);
		}
		break;

	case 'c':
		if (pBase) {
			bReg = musb_readb(pBase, MGC_O_HDRC_POWER)
					& ~MGC_M_POWER_SOFTCONN;
			musb_writeb(pBase, MGC_O_HDRC_POWER, bReg);
		}
		break;

	case 'I':
		if (pBase) {
			bReg = musb_readb(pBase, MGC_O_HDRC_POWER)
					| MGC_M_POWER_HSENAB;
			musb_writeb(pBase, MGC_O_HDRC_POWER, bReg);
		}
		break;

	case 'i':
		if (pBase) {
			bReg = musb_readb(pBase, MGC_O_HDRC_POWER)
					& ~MGC_M_POWER_HSENAB;
			musb_writeb(pBase, MGC_O_HDRC_POWER, bReg);
		}
		break;

	case 'F':
		bReg = musb_readb(pBase, MGC_O_HDRC_DEVCTL);
		bReg |= MGC_M_DEVCTL_SESSION;
		musb_writeb(pBase, MGC_O_HDRC_DEVCTL, bReg);
		break;

	case 'H':
		if (pBase) {
			bReg = musb_readb(pBase, MGC_O_HDRC_DEVCTL);
			bReg |= MGC_M_DEVCTL_HR;
			musb_writeb(pBase, MGC_O_HDRC_DEVCTL, bReg);
			//MUSB_HST_MODE( ((struct musb*)data) );
			//WARN("Host Mode\n");
		}
		break;

	case 'h':
		if (pBase) {
			bReg = musb_readb(pBase, MGC_O_HDRC_DEVCTL);
			bReg &= ~MGC_M_DEVCTL_HR;
			musb_writeb(pBase, MGC_O_HDRC_DEVCTL, bReg);
		}
		break;

	case 'T':
		if (pBase) {
			musb_load_testpacket(musb);
			musb_writeb(pBase, MGC_O_HDRC_TESTMODE,
					MGC_M_TEST_PACKET);
		}
		break;

#if (MUSB_DEBUG>0)
		/* set/read debug level */
	case 'D':{
			if (count > 1) {
				char digits[8], *p = digits;
				int i = 0, level = 0, sign = 1;
				int len = min(count - 1, (unsigned long)8);

				if (copy_from_user(&digits, &buffer[1], len))
					return -EFAULT;

				/* optional sign */
				if (*p == '-') {
					len -= 1;
					sign = -sign;
					p++;
				}

				/* read it */
				while (i++ < len && *p > '0' && *p < '9') {
					level = level * 10 + (*p - '0');
					p++;
				}

				level *= sign;
				DBG(1, "debug level %d\n", level);
				debug = level;
			}
		}
		break;


	case '?':
		INFO("?: you are seeing it\n");
		INFO("C/c: soft connect enable/disable\n");
		INFO("I/i: hispeed enable/disable\n");
		INFO("F: force session start\n");
		INFO("H: host mode\n");
		INFO("T: start sending TEST_PACKET\n");
		INFO("D: set/read dbug level\n");
		break;
#endif

	default:
		ERR("Command %c not implemented\n", cmd);
		break;
	}

	musb_platform_try_idle(musb, 0);

	return count;
}

static int musb_proc_read(char *page, char **start,
			off_t off, int count, int *eof, void *data)
{
	char *buffer = page;
	int code = 0;
	unsigned long	flags;
	struct musb	*pThis = data;
	unsigned	bEnd;

	count -= off;
	count -= 1;		/* for NUL at end */
	if (count <= 0)
		return -EINVAL;

	spin_lock_irqsave(&pThis->Lock, flags);

	code = dump_header_stats(pThis, buffer);
	if (code > 0) {
		buffer += code;
		count -= code;
	}

	/* generate the report for the end points */
	// REVISIT ... not unless something's connected!
	for (bEnd = 0; count >= 0 && bEnd < pThis->bEndCount;
			bEnd++) {
		code = dump_end_info(pThis, bEnd, buffer, count);
		if (code > 0) {
			buffer += code;
			count -= code;
		}
	}

	musb_platform_try_idle(pThis, 0);

	spin_unlock_irqrestore(&pThis->Lock, flags);
	*eof = 1;

	return buffer - page;
}

void __devexit musb_debug_delete(char *name, struct musb *musb)
{
	if (musb->pProcEntry)
		remove_proc_entry(name, NULL);
}

struct proc_dir_entry *__init
musb_debug_create(char *name, struct musb *data)
{
	struct proc_dir_entry	*pde;

	/* FIXME convert everything to seq_file; then later, debugfs */

	if (!name)
		return NULL;

	data->pProcEntry = pde = create_proc_entry(name,
					S_IFREG | S_IRUGO | S_IWUSR, NULL);
	if (pde) {
		pde->data = data;
		// pde->owner = THIS_MODULE;

		pde->read_proc = musb_proc_read;
		pde->write_proc = musb_proc_write;

		pde->size = 0;

		pr_debug("Registered /proc/%s\n", name);
	} else {
		pr_debug("Cannot create a valid proc file entry");
	}

	return pde;
}
