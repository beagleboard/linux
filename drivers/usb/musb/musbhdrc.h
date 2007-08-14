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

#ifndef __MUSB_HDRC_DEFS_H__
#define __MUSB_HDRC_DEFS_H__

/*
 * HDRC-specific definitions
 */

#define MGC_MAX_USB_ENDS       16

#define MGC_END0_FIFOSIZE      64	/* this is non-configurable */

/*
 *     MUSBMHDRC Register map
 */

/* Common USB registers */

#define MGC_O_HDRC_FADDR	0x00	/* 8-bit */
#define MGC_O_HDRC_POWER	0x01	/* 8-bit */

#define MGC_O_HDRC_INTRTX	0x02	/* 16-bit */
#define MGC_O_HDRC_INTRRX       0x04
#define MGC_O_HDRC_INTRTXE      0x06
#define MGC_O_HDRC_INTRRXE      0x08
#define MGC_O_HDRC_INTRUSB      0x0A	/* 8 bit */
#define MGC_O_HDRC_INTRUSBE     0x0B	/* 8 bit */
#define MGC_O_HDRC_FRAME        0x0C
#define MGC_O_HDRC_INDEX        0x0E	/* 8 bit */
#define MGC_O_HDRC_TESTMODE     0x0F	/* 8 bit */

/* Get offset for a given FIFO from musb->mregs */
#ifdef	CONFIG_USB_TUSB6010
#define MUSB_FIFO_OFFSET(epnum)	(0x200 + ((epnum) * 0x20))
#else
#define MUSB_FIFO_OFFSET(epnum)	(0x20 + ((epnum) * 4))
#endif

/* Additional Control Registers */

#define MGC_O_HDRC_DEVCTL	0x60	/* 8 bit */

/* These are always controlled through the INDEX register */
#define MGC_O_HDRC_TXFIFOSZ	0x62	/* 8-bit (see masks) */
#define MGC_O_HDRC_RXFIFOSZ	0x63	/* 8-bit (see masks) */
#define MGC_O_HDRC_TXFIFOADD	0x64	/* 16-bit offset shifted right 3 */
#define MGC_O_HDRC_RXFIFOADD	0x66	/* 16-bit offset shifted right 3 */

// vctrl/vstatus:  optional vendor utmi+phy register at 0x68
#define MGC_O_HDRC_HWVERS	0x6C	/* 8 bit */

#define MGC_O_HDRC_EPINFO	0x78	/* 8 bit */
#define MGC_O_HDRC_RAMINFO	0x79	/* 8 bit */
#define MGC_O_HDRC_LINKINFO	0x7a	/* 8 bit */
#define MGC_O_HDRC_VPLEN	0x7b	/* 8 bit */
#define MGC_O_HDRC_HS_EOF1	0x7c	/* 8 bit */
#define MGC_O_HDRC_FS_EOF1	0x7d	/* 8 bit */
#define MGC_O_HDRC_LS_EOF1	0x7e	/* 8 bit */

/* offsets to endpoint registers */
#define MGC_O_HDRC_TXMAXP	0x00
#define MGC_O_HDRC_TXCSR	0x02
#define MGC_O_HDRC_CSR0		MGC_O_HDRC_TXCSR	/* re-used for EP0 */
#define MGC_O_HDRC_RXMAXP	0x04
#define MGC_O_HDRC_RXCSR	0x06
#define MGC_O_HDRC_RXCOUNT	0x08
#define MGC_O_HDRC_COUNT0	MGC_O_HDRC_RXCOUNT	/* re-used for EP0 */
#define MGC_O_HDRC_TXTYPE	0x0A
#define MGC_O_HDRC_TYPE0	MGC_O_HDRC_TXTYPE	/* re-used for EP0 */
#define MGC_O_HDRC_TXINTERVAL	0x0B
#define MGC_O_HDRC_NAKLIMIT0	MGC_O_HDRC_TXINTERVAL	/* re-used for EP0 */
#define MGC_O_HDRC_RXTYPE	0x0C
#define MGC_O_HDRC_RXINTERVAL	0x0D
#define MGC_O_HDRC_FIFOSIZE	0x0F
#define MGC_O_HDRC_CONFIGDATA	MGC_O_HDRC_FIFOSIZE	/* re-used for EP0 */

/* offsets to endpoint registers in indexed model (using INDEX register) */
#define MGC_INDEXED_OFFSET(_epnum, _bOffset)	\
	(0x10                   + (_bOffset))

/* offsets to endpoint registers in flat models */
#define MGC_FLAT_OFFSET(_epnum, _bOffset)	\
	(0x100 + (0x10*(_epnum)) + (_bOffset))

#ifdef CONFIG_USB_TUSB6010
/* TUSB6010 EP0 configuration register is special */
#define MGC_TUSB_OFFSET(_epnum, _bOffset)	\
	(0x10 + _bOffset)
#include "tusb6010.h"		/* needed "only" for TUSB_EP0_CONF */
#endif

/* "bus control"/target registers, for host side multipoint (external hubs) */
#define MGC_O_HDRC_TXFUNCADDR	0x00
#define MGC_O_HDRC_TXHUBADDR	0x02
#define MGC_O_HDRC_TXHUBPORT	0x03

#define MGC_O_HDRC_RXFUNCADDR	0x04
#define MGC_O_HDRC_RXHUBADDR	0x06
#define MGC_O_HDRC_RXHUBPORT	0x07

#define MGC_BUSCTL_OFFSET(_epnum, _bOffset) \
	(0x80 + (8*(_epnum)) + (_bOffset))

/*
 *     MUSBHDRC Register bit masks
 */

/* POWER */

#define MGC_M_POWER_ISOUPDATE   0x80
#define MGC_M_POWER_SOFTCONN    0x40
#define MGC_M_POWER_HSENAB	0x20
#define MGC_M_POWER_HSMODE	0x10
#define MGC_M_POWER_RESET       0x08
#define MGC_M_POWER_RESUME      0x04
#define MGC_M_POWER_SUSPENDM    0x02
#define MGC_M_POWER_ENSUSPEND   0x01

/* INTRUSB */
#define MGC_M_INTR_SUSPEND    0x01
#define MGC_M_INTR_RESUME     0x02
#define MGC_M_INTR_RESET      0x04
#define MGC_M_INTR_BABBLE     0x04
#define MGC_M_INTR_SOF        0x08
#define MGC_M_INTR_CONNECT    0x10
#define MGC_M_INTR_DISCONNECT 0x20
#define MGC_M_INTR_SESSREQ    0x40
#define MGC_M_INTR_VBUSERROR  0x80	/* FOR SESSION END */

/* DEVCTL */
#define MGC_M_DEVCTL_BDEVICE    0x80
#define MGC_M_DEVCTL_FSDEV      0x40
#define MGC_M_DEVCTL_LSDEV      0x20
#define MGC_M_DEVCTL_VBUS       0x18
#define MGC_S_DEVCTL_VBUS       3
#define MGC_M_DEVCTL_HM         0x04
#define MGC_M_DEVCTL_HR         0x02
#define MGC_M_DEVCTL_SESSION    0x01

/* TESTMODE */

#define MGC_M_TEST_FORCE_HOST   0x80
#define MGC_M_TEST_FIFO_ACCESS  0x40
#define MGC_M_TEST_FORCE_FS     0x20
#define MGC_M_TEST_FORCE_HS     0x10
#define MGC_M_TEST_PACKET       0x08
#define MGC_M_TEST_K            0x04
#define MGC_M_TEST_J            0x02
#define MGC_M_TEST_SE0_NAK      0x01

/* allocate for double-packet buffering (effectively doubles assigned _SIZE) */
#define MGC_M_FIFOSZ_DPB	0x10
/* allocation size (8, 16, 32, ... 4096) */
#define MGC_M_FIFOSZ_SIZE	0x0f

/* CSR0 */
#define MGC_M_CSR0_FLUSHFIFO      0x0100
#define MGC_M_CSR0_TXPKTRDY       0x0002
#define MGC_M_CSR0_RXPKTRDY       0x0001

/* CSR0 in Peripheral mode */
#define MGC_M_CSR0_P_SVDSETUPEND  0x0080
#define MGC_M_CSR0_P_SVDRXPKTRDY  0x0040
#define MGC_M_CSR0_P_SENDSTALL    0x0020
#define MGC_M_CSR0_P_SETUPEND     0x0010
#define MGC_M_CSR0_P_DATAEND      0x0008
#define MGC_M_CSR0_P_SENTSTALL    0x0004

/* CSR0 in Host mode */
#define MGC_M_CSR0_H_DIS_PING	0x0800
#define MGC_M_CSR0_H_WR_DATATOGGLE   0x0400	/* set to allow setting: */
#define MGC_M_CSR0_H_DATATOGGLE	    0x0200	/* data toggle control */
#define MGC_M_CSR0_H_NAKTIMEOUT   0x0080
#define MGC_M_CSR0_H_STATUSPKT    0x0040
#define MGC_M_CSR0_H_REQPKT       0x0020
#define MGC_M_CSR0_H_ERROR        0x0010
#define MGC_M_CSR0_H_SETUPPKT     0x0008
#define MGC_M_CSR0_H_RXSTALL      0x0004

/* CSR0 bits to avoid zeroing (write zero clears, write 1 ignored) */
#define MGC_M_CSR0_P_WZC_BITS	\
	( MGC_M_CSR0_P_SENTSTALL )
#define MGC_M_CSR0_H_WZC_BITS	\
	( MGC_M_CSR0_H_NAKTIMEOUT | MGC_M_CSR0_H_RXSTALL \
	| MGC_M_CSR0_RXPKTRDY )


/* TxType/RxType */
#define MGC_M_TYPE_SPEED	0xc0
#define MGC_S_TYPE_SPEED	6
#define MGC_TYPE_SPEED_HIGH	1
#define MGC_TYPE_SPEED_FULL	2
#define MGC_TYPE_SPEED_LOW	3
#define MGC_M_TYPE_PROTO	0x30	/* implicitly zero for ep0 */
#define MGC_S_TYPE_PROTO	4
#define MGC_M_TYPE_REMOTE_END	0xf	/* implicitly zero for ep0 */

/* CONFIGDATA */

#define MGC_M_CONFIGDATA_MPRXE      0x80	/* auto bulk pkt combining */
#define MGC_M_CONFIGDATA_MPTXE      0x40	/* auto bulk pkt splitting */
#define MGC_M_CONFIGDATA_BIGENDIAN  0x20
#define MGC_M_CONFIGDATA_HBRXE      0x10	/* HB-ISO for RX */
#define MGC_M_CONFIGDATA_HBTXE      0x08	/* HB-ISO for TX */
#define MGC_M_CONFIGDATA_DYNFIFO    0x04	/* dynamic FIFO sizing */
#define MGC_M_CONFIGDATA_SOFTCONE   0x02	/* SoftConnect */
#define MGC_M_CONFIGDATA_UTMIDW     0x01	/* data width 0/1 => 8/16bits */

/* TXCSR in Peripheral and Host mode */

#define MGC_M_TXCSR_AUTOSET       0x8000
#define MGC_M_TXCSR_MODE          0x2000
#define MGC_M_TXCSR_DMAENAB       0x1000
#define MGC_M_TXCSR_FRCDATATOG    0x0800
#define MGC_M_TXCSR_DMAMODE       0x0400
#define MGC_M_TXCSR_CLRDATATOG    0x0040
#define MGC_M_TXCSR_FLUSHFIFO     0x0008
#define MGC_M_TXCSR_FIFONOTEMPTY  0x0002
#define MGC_M_TXCSR_TXPKTRDY      0x0001

/* TXCSR in Peripheral mode */

#define MGC_M_TXCSR_P_ISO         0x4000
#define MGC_M_TXCSR_P_INCOMPTX    0x0080
#define MGC_M_TXCSR_P_SENTSTALL   0x0020
#define MGC_M_TXCSR_P_SENDSTALL   0x0010
#define MGC_M_TXCSR_P_UNDERRUN    0x0004

/* TXCSR in Host mode */

#define MGC_M_TXCSR_H_WR_DATATOGGLE   0x0200
#define MGC_M_TXCSR_H_DATATOGGLE      0x0100
#define MGC_M_TXCSR_H_NAKTIMEOUT  0x0080
#define MGC_M_TXCSR_H_RXSTALL     0x0020
#define MGC_M_TXCSR_H_ERROR       0x0004

/* TXCSR bits to avoid zeroing (write zero clears, write 1 ignored) */
#define MGC_M_TXCSR_P_WZC_BITS	\
	( MGC_M_TXCSR_P_INCOMPTX | MGC_M_TXCSR_P_SENTSTALL \
	| MGC_M_TXCSR_P_UNDERRUN | MGC_M_TXCSR_FIFONOTEMPTY )
#define MGC_M_TXCSR_H_WZC_BITS	\
	( MGC_M_TXCSR_H_NAKTIMEOUT | MGC_M_TXCSR_H_RXSTALL \
	| MGC_M_TXCSR_H_ERROR | MGC_M_TXCSR_FIFONOTEMPTY )


/* RXCSR in Peripheral and Host mode */

#define MGC_M_RXCSR_AUTOCLEAR     0x8000
#define MGC_M_RXCSR_DMAENAB       0x2000
#define MGC_M_RXCSR_DISNYET       0x1000
#define MGC_M_RXCSR_PID_ERR       0x1000
#define MGC_M_RXCSR_DMAMODE       0x0800
#define MGC_M_RXCSR_INCOMPRX      0x0100
#define MGC_M_RXCSR_CLRDATATOG    0x0080
#define MGC_M_RXCSR_FLUSHFIFO     0x0010
#define MGC_M_RXCSR_DATAERROR     0x0008
#define MGC_M_RXCSR_FIFOFULL      0x0002
#define MGC_M_RXCSR_RXPKTRDY      0x0001

/* RXCSR in Peripheral mode */

#define MGC_M_RXCSR_P_ISO         0x4000
#define MGC_M_RXCSR_P_SENTSTALL   0x0040
#define MGC_M_RXCSR_P_SENDSTALL   0x0020
#define MGC_M_RXCSR_P_OVERRUN     0x0004

/* RXCSR in Host mode */

#define MGC_M_RXCSR_H_AUTOREQ     0x4000
#define MGC_M_RXCSR_H_WR_DATATOGGLE   0x0400
#define MGC_M_RXCSR_H_DATATOGGLE        0x0200
#define MGC_M_RXCSR_H_RXSTALL     0x0040
#define MGC_M_RXCSR_H_REQPKT      0x0020
#define MGC_M_RXCSR_H_ERROR       0x0004

/* RXCSR bits to avoid zeroing (write zero clears, write 1 ignored) */
#define MGC_M_RXCSR_P_WZC_BITS	\
	( MGC_M_RXCSR_P_SENTSTALL | MGC_M_RXCSR_P_OVERRUN \
	| MGC_M_RXCSR_RXPKTRDY )
#define MGC_M_RXCSR_H_WZC_BITS	\
	( MGC_M_RXCSR_H_RXSTALL | MGC_M_RXCSR_H_ERROR \
	| MGC_M_RXCSR_DATAERROR | MGC_M_RXCSR_RXPKTRDY )


/* HUBADDR */
#define MGC_M_HUBADDR_MULTI_TT		0x80


#endif	/* __MUSB_HDRC_DEFS_H__ */
