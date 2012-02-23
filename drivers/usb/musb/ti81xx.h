/*
 * Copyright (C) 2005-2006 by Texas Instruments
 *
 * The Inventra Controller Driver for Linux is free software; you
 * can redistribute it and/or modify it under the terms of the GNU
 * General Public License version 2 as published by the Free Software
 * Foundation.
 */

#ifndef __MUSB_HDRDF_H__
#define __MUSB_HDRDF_H__

#define TI81XX_USB_CPPIDMA_BASE 0x47402000
#define TI81XX_USB_CPPIDMA_LEN  0x5FFF
#define TI81XX_IRQ_USBSS        17

/* Netra USB susbsystem register offsets */
#define USBSS_REVISION			0x0000
#define USBSS_SYSCONFIG			0x0010
/* USBSS EOI interrupt register */
#define USBSS_IRQ_EOI			0x0020
/* USBSS interrupt generation/status register */
#define USBSS_IRQ_STATUS_RAW		0x0024
/* USBSS interrupt status register */
#define USBSS_IRQ_STATUS		0x0028
/* USBSS interrupt enable register */
#define USBSS_IRQ_ENABLE_SET		0x002c
/* USBSS interrupt clear register */
#define USBSS_IRQ_ENABLE_CLEAR		0x0030
/* USB0: TxDMA 8bit tx completion interrupt pacing
	threshold value for ep1..15 */
#define USBSS_IRQ_DMA_THRESHOLD_TX0	0x0100
/* USB0: RxDMA 8bit rx completion interrupt pacing
	threshold value for ep1..15 */
#define USBSS_IRQ_DMA_THRESHOLD_RX0	0x0110
/* USB1: TxDMA 8bit tx completion interrupt pacing
	threshold value for ep1..15 */
#define USBSS_IRQ_DMA_THRESHOLD_TX1	0x0120
/* USB1: RxDMA 8bit rx completion interrupt pacing
	threshold value for ep1..15 */
#define USBSS_IRQ_DMA_THRESHOLD_RX1	0x0130
/* USB0: TxDMA threshold enable tx completion for ep1..ep15
	RxDMA threshold enable rx completion for ep1..ep15 */
#define USBSS_IRQ_DMA_ENABLE_0		0x0140
/* USB1: TxDMA threshold enable for ep1..ep15
	RxDMA threshold enable for ep1..ep15 */
#define USBSS_IRQ_DMA_ENABLE_1		0x0144
/* USB0: TxDMA Frame threshold for tx completion for ep1..ep15
	RxDMA Frame threshold for rx completion for ep1..ep15 */
#define USBSS_IRQ_FRAME_THRESHOLD_TX0	0x0200
#define USBSS_IRQ_FRAME_THRESHOLD_RX0	0x0210
/* USB1: TxDMA Frame threshold for tx completion for ep1..ep15
	RxDMA Frame threshold for rx completion for ep1..ep15 */
#define USBSS_IRQ_FRAME_THRESHOLD_TX1	0x0220
#define USBSS_IRQ_FRAME_THRESHOLD_RX1	0x0230
/* USB0: Frame threshold enable tx completion for ep1..ep15
	Frame threshold enable rx completion for ep1..ep15 */
#define USBSS_IRQ_FRAME_ENABLE_0	0x0240
#define USBSS_IRQ_FRAME_ENABLE_1	0x0244


/* USB 2.0 OTG module registers */
#define USB_REVISION_REG        0x0000
#define USB_CTRL_REG            0x0014
#define USB_STAT_REG            0x0018
#define	USB_IRQ_MERGED_STATUS	0x0020
#define USB_IRQ_EOI		0x0024
#define	USB_IRQ_STATUS_RAW_0	0x0028
#define	USB_IRQ_STATUS_RAW_1	0x002c
#define	USB_IRQ_STATUS_0	0x0030
#define	USB_IRQ_STATUS_1	0x0034
#define	USB_IRQ_ENABLE_SET_0	0x0038
#define	USB_IRQ_ENABLE_SET_1	0x003c
#define	USB_IRQ_ENABLE_CLR_0	0x0040
#define	USB_IRQ_ENABLE_CLR_1	0x0044

#define USB_EP_INTR_SET_REG		(USB_IRQ_ENABLE_SET_0)
#define USB_CORE_INTR_SET_REG		(USB_IRQ_ENABLE_SET_1)
#define USB_EP_INTR_CLEAR_REG		(USB_IRQ_ENABLE_CLR_0)
#define USB_CORE_INTR_CLEAR_REG		(USB_IRQ_ENABLE_CLR_1)
#define USB_EP_INTR_STATUS_REG		(USB_IRQ_STATUS_0)
#define USB_CORE_INTR_STATUS_REG	(USB_IRQ_STATUS_1)

#define USB_GRNDIS_EPSIZE_OFFS	0X0080
#define USB_SRP_FIX_TIME_REG    0x00d4
#define USB_TH_XDMA_IDLE_REG    0x00dc
#define USB_PHY_UTMI_REG	0x00e0
#define USB_PHY_UTMI_LB_REG	0x00e4
#define USB_MODE_REG		0x00e8

#define QUEUE_THRESHOLD_INTR_ENABLE_REG 0xc0
#define QUEUE_63_THRESHOLD_REG  0xc4
#define QUEUE_63_THRESHOLD_INTR_CLEAR_REG 0xc8
#define QUEUE_65_THRESHOLD_REG  0xd4
#define QUEUE_65_THRESHOLD_INTR_CLEAR_REG 0xd8

/* Control register bits */
#define USB_SOFT_RESET_MASK     1

/* Mode register bits */
#define USB_MODE_SHIFT(n)       ((((n) - 1) << 1))
#define USB_MODE_MASK(n)        (3 << USB_MODE_SHIFT(n))
#define USB_RX_MODE_SHIFT(n)    USB_MODE_SHIFT(n)
#define USB_TX_MODE_SHIFT(n)    USB_MODE_SHIFT(n)
#define USB_RX_MODE_MASK(n)     USB_MODE_MASK(n)
#define USB_TX_MODE_MASK(n)     USB_MODE_MASK(n)
#define USB_TRANSPARENT_MODE    0
#define USB_RNDIS_MODE          1
#define USB_CDC_MODE            2
#define USB_GENERIC_RNDIS_MODE  3

/* AutoReq register bits */
#define USB_RX_AUTOREQ_SHIFT(n) (((n) - 1) << 1)
#define USB_RX_AUTOREQ_MASK(n)  (3 << USB_RX_AUTOREQ_SHIFT(n))
#define USB_NO_AUTOREQ          0
#define USB_AUTOREQ_ALL_BUT_EOP 1
#define USB_AUTOREQ_ALWAYS      3

/* Teardown register bits */
#define USB_TX_TDOWN_SHIFT(n)   (16 + (n))
#define USB_TX_TDOWN_MASK(n)    (1 << USB_TX_TDOWN_SHIFT(n))
#define USB_RX_TDOWN_SHIFT(n)   (n)
#define USB_RX_TDOWN_MASK(n)    (1 << USB_RX_TDOWN_SHIFT(n))

/* USB interrupt register bits */
#define USB_INTR_USB_SHIFT      0
#define USB_INTR_USB_MASK       (0x1ff << USB_INTR_USB_SHIFT) /* 8 Mentor */
				/* interrupts and DRVVBUS interrupt */
#define USB_INTR_DRVVBUS        0x100
#define USB_INTR_RX_SHIFT       16
#define USB_INTR_TX_SHIFT       0

#define USB_MENTOR_CORE_OFFSET  0x400
#define USB_CPPI41_NUM_CH       15

#define MAX_MUSB_INSTANCE	2
/* CPPI 4.1 queue manager registers */
#define QMGR_PEND0_REG		0x4090
#define QMGR_PEND1_REG		0x4094
#define QMGR_PEND2_REG		0x4098

#define QMGR_RGN_OFFS		0x4000
#define QMRG_DESCRGN_OFFS	0x5000
#define QMGR_REG_OFFS		0x6000
#define QMGR_STAT_OFFS		0x7000
#define DMA_GLBCTRL_OFFS	0x2000
#define DMA_CHCTRL_OFFS		0x2800
#define DMA_SCHED_OFFS		0x3000
#define DMA_SCHEDTBL_OFFS	0x3800

#define USB_TX_EP_MASK		0xffff		/* EP0 + 15 Tx EPs */
#define USB_RX_EP_MASK		0xfffe		/* 15 Rx EPs */

#define USB_TX_INTR_MASK	(USB_TX_EP_MASK << USB_INTR_TX_SHIFT)
#define USB_RX_INTR_MASK	(USB_RX_EP_MASK << USB_INTR_RX_SHIFT)

#define A_WAIT_BCON_TIMEOUT	1100		/* in ms */

#define USBSS_INTR_RX_STARV	0x00000001
#define USBSS_INTR_PD_CMPL	0x00000004
#define USBSS_INTR_TX_CMPL	0x00000500
#define USBSS_INTR_RX_CMPL	0x00000A00
#define USBSS_INTR_FLAGS	(USBSS_INTR_PD_CMPL | USBSS_INTR_TX_CMPL \
					| USBSS_INTR_RX_CMPL)

#define	USBMODE_USBID_MUXSEL	0x80
#define	USBMODE_USBID_HIGH	0x100

#define	USB0PORT_MODEMASK	0x0f
#define	USB1PORT_MODEMASK	0xf0
#define USB1PORT_MODESHIFT	4
extern void usb_nop_xceiv_register(int id);
#endif
