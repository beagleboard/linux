/*
 * Copyright (C) 2005-2006 by Texas Instruments
 * Copyright (c) 2008, MontaVista Software, Inc. <source@mvista.com>
 *
 * This program is free software; you can distribute it and/or modify it
 * under the terms of the GNU General Public License (Version 2) as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 */

#ifndef _CPPI41_DMA_H_
#define _CPPI41_DMA_H_
#include <plat/usb.h>

/* USBSS RTL versions */
#define USBSS_RTL_VERSION_MASK	0xF
#define USBSS_RTL_VERSION_D	0xD

/**
 * struct usb_cppi41_info - CPPI 4.1 USB implementation details
 * @dma_block:	DMA block number
 * @ep_dma_ch:	DMA channel numbers used for EPs 1 .. Max_EP
 * @q_mgr:	queue manager number
 * @num_tx_comp_q: number of the Tx completion queues
 * @num_rx_comp_q: number of the Rx queues
 * @tx_comp_q:	pointer to the list of the Tx completion queue numbers
 * @rx_comp_q:	pointer to the list of the Rx queue numbers
 */
struct usb_cppi41_info {
	u8 dma_block;
	u8 ep_dma_ch[USB_CPPI41_NUM_CH];
	u8 q_mgr;
	u8 num_tx_comp_q;
	u8 num_rx_comp_q;
	u16 *tx_comp_q;
	u16 *rx_comp_q;
	u8 bd_intr_ctrl;
	u8 grndis_for_host_rx;
	u32 version;
};

extern struct usb_cppi41_info usb_cppi41_info[];

/**
 * cppi41_completion - Tx/Rx completion queue interrupt handling hook
 * @musb:	the controller
 * @rx:	bitmask having bit N set if Rx queue N is not empty
 * @tx:	bitmask having bit N set if Tx completion queue N is not empty
 */
void cppi41_completion(struct musb *musb, u32 rx, u32 tx);

/**
 * cppi41_handle_txfifo_intr - Handles tx fifo empty interupts
 * @musb:	the controller
 */
void cppi41_handle_txfifo_intr(struct musb *musb, u16 usbintr);
void txfifoempty_int_enable(struct musb *musb, u8 ep_num);
void txfifoempty_int_disable(struct musb *musb, u8 ep_num);
#endif	/* _CPPI41_DMA_H_ */
